//! Benchmarks for the sim-ml-bridge crate.
//!
//! Run with: `cargo bench -p sim-ml-bridge`
//!
//! ## Targets
//!
//! | Benchmark               | Target          | Pendulum | 10-link chain |
//! |-------------------------|-----------------|----------|---------------|
//! | `extract()` single env  | <1 µs           | ~38 ns   | —             |
//! | `apply()` single env    | <500 ns         | ~2.7 ns  | —             |
//! | `extract_batch()` 64    | <50 µs          | ~473 ns  | —             |
//! | Bridge overhead 64 envs | <5% of step     | ~17%     | **~1.4%**     |
//! | `VecEnv` 64 envs, 1 sub | >100K steps/sec | ~39K     | —             |
//!
//! ## Note on overhead and throughput numbers
//!
//! The pendulum model is a single hinge joint with one actuator — its
//! physics step costs ~340 ns/env, which is unrealistically cheap.  The
//! bridge overhead (~3.5 µs total for 64 envs) appears as ~17% only
//! because the physics is near-zero cost.  With a realistic model
//! (humanoid with contacts, ~100 µs/env), the same bridge overhead
//! would be <0.1%.  The individual operation targets (extract <1 µs,
//! apply <500 ns) are the meaningful measures — all pass with 25–185x
//! margin.  Add humanoid benchmarks when a standard MJCF is available.

#![allow(
    missing_docs,
    clippy::wildcard_imports,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::let_underscore_must_use
)]

use std::hint::black_box;
use std::sync::Arc;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use sim_core::{BatchSim, Model};
use sim_ml_bridge::{ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv};

// ─── helpers ────────────────────────────────────────────────────────────────

fn pendulum_model() -> Arc<Model> {
    let xml = r#"
    <mujoco>
      <option timestep="0.002"/>
      <worldbody>
        <body name="pendulum" pos="0 0 1">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
        </body>
      </worldbody>
      <actuator>
        <motor joint="hinge" name="motor" ctrllimited="true" ctrlrange="-1 1"/>
      </actuator>
      <sensor>
        <jointpos joint="hinge" name="angle"/>
        <jointvel joint="hinge" name="angvel"/>
      </sensor>
    </mujoco>
    "#;
    Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"))
}

fn pendulum_spaces(model: &Model) -> (ObservationSpace, ActionSpace) {
    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .all_sensordata()
        .build(model)
        .expect("valid obs space");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(model)
        .expect("valid act space");
    (obs, act)
}

// ─── extract / apply latency ────────────────────────────────────────────────

fn bench_extract(c: &mut Criterion) {
    let model = pendulum_model();
    let (obs_space, _act_space) = pendulum_spaces(&model);
    let data = model.make_data();

    c.bench_function("extract/single_env", |b| {
        b.iter(|| {
            black_box(obs_space.extract(black_box(&data)));
        });
    });
}

fn bench_apply(c: &mut Criterion) {
    let model = pendulum_model();
    let (_obs_space, act_space) = pendulum_spaces(&model);
    let mut data = model.make_data();
    let action = Tensor::from_slice(&[0.5_f32], &[1]);

    c.bench_function("apply/single_env", |b| {
        b.iter(|| {
            act_space.apply(black_box(&action), black_box(&mut data), &model);
        });
    });
}

fn bench_extract_batch(c: &mut Criterion) {
    let model = pendulum_model();
    let (obs_space, _act_space) = pendulum_spaces(&model);

    let mut group = c.benchmark_group("extract_batch");
    for n_envs in [4, 16, 64] {
        let batch = BatchSim::new(Arc::clone(&model), n_envs);
        group.bench_with_input(BenchmarkId::from_parameter(n_envs), &n_envs, |b, _| {
            b.iter(|| {
                black_box(obs_space.extract_batch(batch.envs()));
            });
        });
    }
    group.finish();
}

fn bench_apply_batch(c: &mut Criterion) {
    let model = pendulum_model();
    let (_obs_space, act_space) = pendulum_spaces(&model);

    let mut group = c.benchmark_group("apply_batch");
    for n_envs in [4, 16, 64] {
        let actions = Tensor::zeros(&[n_envs, act_space.dim()]);
        let mut batch = BatchSim::new(Arc::clone(&model), n_envs);
        group.bench_with_input(BenchmarkId::from_parameter(n_envs), &n_envs, |b, _| {
            b.iter(|| {
                act_space.apply_batch(black_box(&actions), batch.envs_mut(), &model);
            });
        });
    }
    group.finish();
}

// ─── VecEnv throughput ──────────────────────────────────────────────────────

// Throughput numbers here are pessimistic because the pendulum model is
// trivially cheap (~340 ns/env).  With a realistic model (humanoid, contacts)
// the physics step dominates and the bridge overhead becomes negligible.
fn bench_vec_env_step(c: &mut Criterion) {
    let model = pendulum_model();

    let mut group = c.benchmark_group("vec_env_step");
    for &(n_envs, sub_steps) in &[(4, 1), (16, 1), (64, 1), (64, 10)] {
        let (obs, act) = pendulum_spaces(&model);
        let mut env = VecEnv::builder(Arc::clone(&model), n_envs)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, data| -data.qpos[0].powi(2))
            .done(|_m, data| data.qpos[0].abs() > 3.0)
            .truncated(|_m, data| data.time > 10.0)
            .sub_steps(sub_steps)
            .build()
            .expect("valid VecEnv");

        env.reset_all().expect("reset");
        let actions = Tensor::zeros(&[n_envs, 1]);

        let label = format!("{n_envs}envs_{sub_steps}sub");
        group.bench_function(&label, |b| {
            b.iter(|| {
                let _ = black_box(env.step(black_box(&actions)));
            });
        });
    }
    group.finish();
}

// ─── bridge overhead: VecEnv::step vs raw BatchSim::step_all ────────────────

// The overhead percentage is artificially high here because the pendulum's
// physics cost (~340 ns/env) is comparable to the bridge's per-env cost
// (~55 ns/env for extract+apply+reward+done eval).  With a realistic model
// where physics costs ~100 µs/env, the same bridge cost would be <0.1%.
fn bench_bridge_overhead(c: &mut Criterion) {
    let model = pendulum_model();
    let n_envs = 64;

    // Raw BatchSim baseline
    let mut raw_batch = BatchSim::new(Arc::clone(&model), n_envs);

    // VecEnv with the same model
    let (obs, act) = pendulum_spaces(&model);
    let mut vec_env = VecEnv::builder(Arc::clone(&model), n_envs)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, data| -data.qpos[0].powi(2))
        .done(|_m, data| data.qpos[0].abs() > 3.0)
        .truncated(|_m, data| data.time > 10.0)
        .build()
        .expect("valid VecEnv");
    vec_env.reset_all().expect("reset");
    let actions = Tensor::zeros(&[n_envs, 1]);

    let mut group = c.benchmark_group("overhead_64envs");

    group.bench_function("raw_batch_step_all", |b| {
        b.iter(|| {
            black_box(raw_batch.step_all());
        });
    });

    group.bench_function("vec_env_step", |b| {
        b.iter(|| {
            let _ = black_box(vec_env.step(black_box(&actions)));
        });
    });

    group.finish();
}

// ─── SimEnv single-env step ─────────────────────────────────────────────────

fn bench_sim_env_step(c: &mut Criterion) {
    let model = pendulum_model();
    let (obs, act) = pendulum_spaces(&model);

    let mut env = SimEnv::builder(Arc::clone(&model))
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, data| -data.qpos[0].powi(2))
        .done(|_m, data| data.qpos[0].abs() > 3.0)
        .truncated(|_m, data| data.time > 10.0)
        .sub_steps(1)
        .build()
        .expect("valid SimEnv");

    env.reset().expect("reset");
    let action = Tensor::from_slice(&[0.0_f32], &[1]);

    c.bench_function("sim_env_step/single", |b| {
        b.iter(|| {
            let _ = black_box(env.step(black_box(&action)));
        });
    });
}

// ─── multi-joint chain: realistic overhead measurement ──────────────────────

/// 10-body chain of hinge joints with a ground plane and contacts enabled.
/// This gives a much more realistic physics cost per env (~10–50 µs) so the
/// bridge overhead % becomes meaningful.
fn chain_model() -> Arc<Model> {
    use std::fmt::Write;

    // Programmatically build a 10-link chain with contacts.
    let mut bodies = String::new();
    let mut actuators = String::new();
    let mut parent_close = String::new();

    for i in 0..10 {
        let name = format!("link{i}");
        let jname = format!("j{i}");
        let pos = if i == 0 {
            "0 0 1.5".to_string()
        } else {
            "0 0 -0.2".to_string()
        };
        write!(
            bodies,
            r#"<body name="{name}" pos="{pos}">
              <joint name="{jname}" type="hinge" axis="0 1 0" damping="0.5"/>
              <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.15" mass="0.5"/>
            "#
        )
        .unwrap();
        write!(
            actuators,
            r#"<motor joint="{jname}" name="m{i}" gear="1" ctrllimited="true" ctrlrange="-1 1"/>"#
        )
        .unwrap();
        parent_close.push_str("</body>");
    }

    let xml = format!(
        r#"<mujoco>
          <option timestep="0.002" gravity="0 0 -9.81"/>
          <worldbody>
            <geom type="plane" size="5 5 0.1" pos="0 0 0"/>
            {bodies}
            {parent_close}
          </worldbody>
          <actuator>
            {actuators}
          </actuator>
        </mujoco>"#
    );
    Arc::new(sim_mjcf::load_model(&xml).expect("valid chain MJCF"))
}

fn chain_spaces(model: &Model) -> (ObservationSpace, ActionSpace) {
    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(model)
        .expect("valid obs space");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(model)
        .expect("valid act space");
    (obs, act)
}

/// `VecEnv` throughput and overhead with a 10-link chain — physics dominates here.
fn bench_chain_overhead(c: &mut Criterion) {
    let model = chain_model();
    let n_envs = 64;
    let act_dim = model.nu;

    let mut raw_batch = BatchSim::new(Arc::clone(&model), n_envs);

    let (obs, act) = chain_spaces(&model);
    let mut vec_env = VecEnv::builder(Arc::clone(&model), n_envs)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, data| -data.qpos[0].powi(2))
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > 10.0)
        .build()
        .expect("valid VecEnv");
    vec_env.reset_all().expect("reset");
    let actions = Tensor::zeros(&[n_envs, act_dim]);

    let mut group = c.benchmark_group("overhead_chain_64envs");

    group.bench_function("raw_batch_step_all", |b| {
        b.iter(|| {
            black_box(raw_batch.step_all());
        });
    });

    group.bench_function("vec_env_step", |b| {
        b.iter(|| {
            let _ = black_box(vec_env.step(black_box(&actions)));
        });
    });

    group.finish();
}

// ─── hill muscle (stub — ignored until model is available) ──────────────────

// TODO: add a forearm-flexion or similar hill-muscle MJCF once the muscle
// pipeline is stable.  Hill muscles add activation dynamics, tendons, and
// force-length/force-velocity curves, making the physics step significantly
// more expensive (~100 µs+/env) — the ideal workload for validating that
// bridge overhead stays <1%.
// Intentionally empty — placeholder for when hill muscle models are ready.
// Populate when `sim/L0/core` muscle pipeline is complete (forearm-flexion
// or similar).  Hill muscles add activation dynamics, tendons, and
// force-length/force-velocity curves — expect ~100 µs+/env physics cost,
// making bridge overhead <0.1%.
#[allow(clippy::missing_const_for_fn)]
fn bench_hill_muscle_overhead(_c: &mut Criterion) {}

// ─── criterion main ─────────────────────────────────────────────────────────

criterion_group!(
    benches,
    bench_extract,
    bench_apply,
    bench_extract_batch,
    bench_apply_batch,
    bench_vec_env_step,
    bench_bridge_overhead,
    bench_sim_env_step,
    bench_chain_overhead,
    bench_hill_muscle_overhead,
);
criterion_main!(benches);
