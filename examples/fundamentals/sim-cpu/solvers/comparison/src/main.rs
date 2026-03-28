//! Solver Comparison — Headless Convergence Analysis
//!
//! Runs all 3 constraint solvers (PGS, CG, Newton) on an identical two-box
//! stacking scene for 5 seconds, then prints a formatted comparison table
//! with convergence speed, constraint satisfaction, and stability metrics.
//!
//! No Bevy window — pure sim-core + sim-mjcf.
//!
//! Validates (7 checks):
//! 1. All solvers stable: max z-drift of `box_a` < 1mm
//! 2. Newton converges fast: avg iterations <= 5
//! 3. PGS converges: avg iterations <= 50
//! 4. CG faster than PGS: `avg_iter_CG` < `avg_iter_PGS`
//! 5. Newton faster than CG: `avg_iter_Newton` < `avg_iter_CG`
//! 6. Constraint violation < 1mm for all solvers
//! 7. Energy stable: |drift| < 1% for all solvers
//!
//! Run with: `cargo run -p example-solver-comparison --release`

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::too_many_lines
)]

const SIM_TIME: f64 = 5.0;
const DT: f64 = 0.002;
const STEPS: usize = (SIM_TIME / DT) as usize;
/// Settling time before measuring drift (skip initial transient).
const SETTLE_TIME: f64 = 0.5;
const SETTLE_STEPS: usize = (SETTLE_TIME / DT) as usize;

fn mjcf(solver: &str) -> String {
    format!(
        r#"
<mujoco model="solver-{solver}">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="{DT}" solver="{solver}"
          iterations="100" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01"
          friction="0.5 0.005 0.001" rgba="0.35 0.35 0.38 1"/>

    <body name="box_a" pos="0 0 0.1">
      <joint type="free" name="jnt_a"/>
      <geom name="box_a" type="box" size="0.1 0.1 0.1" mass="1.0"
            friction="0.5 0.005 0.001" rgba="0.82 0.22 0.15 1"/>
    </body>

    <body name="box_b" pos="0 0 0.3">
      <joint type="free" name="jnt_b"/>
      <geom name="box_b" type="box" size="0.1 0.1 0.1" mass="0.5"
            friction="0.5 0.005 0.001" rgba="0.15 0.45 0.82 1"/>
    </body>
  </worldbody>
</mujoco>
"#
    )
}

struct SolverResult {
    name: &'static str,
    avg_iter: f64,
    max_iter: usize,
    max_drift_mm: f64,
    max_depth_mm: f64,
    energy_drift_pct: f64,
    fallback_count: Option<usize>,
}

fn run_solver(name: &'static str, mjcf_solver: &str) -> SolverResult {
    let xml = mjcf(mjcf_solver);
    let model = sim_mjcf::load_model(&xml).expect("MJCF should parse");
    let mut data = model.make_data();

    // Forward to compute initial contacts and energy baseline
    data.forward(&model).expect("forward should succeed");

    let mut total_iter: usize = 0;
    let mut max_iter: usize = 0;
    let mut max_depth: f64 = 0.0;

    // box_a z-position after settling (measured once settle period ends)
    let mut rest_z: Option<f64> = None;
    let mut max_drift: f64 = 0.0;

    // Energy at settled state (post-transient baseline)
    let mut energy_settled: Option<f64> = None;

    // Newton fallback tracking
    let track_fallback = mjcf_solver == "Newton";
    let mut fallback_count: usize = 0;

    for step in 0..STEPS {
        data.step(&model).expect("step should succeed");

        // Solver iteration stats
        total_iter += data.solver_niter;
        if data.solver_niter > max_iter {
            max_iter = data.solver_niter;
        }

        // Contact penetration depth (only post-settling to exclude transient)
        if step >= SETTLE_STEPS {
            for contact in &data.contacts[..data.ncon] {
                if contact.depth > max_depth {
                    max_depth = contact.depth;
                }
            }
        }

        // box_a z-position (qpos[2] for first free joint)
        let box_a_z = data.qpos[2];

        if step == SETTLE_STEPS {
            rest_z = Some(box_a_z);
            energy_settled = Some(data.total_energy());
        }
        if step > SETTLE_STEPS
            && let Some(rz) = rest_z
        {
            let drift = (box_a_z - rz).abs();
            if drift > max_drift {
                max_drift = drift;
            }
        }

        // Newton fallback detection
        if track_fallback && !data.newton_solved {
            fallback_count += 1;
        }
    }

    let energy_final = data.total_energy();
    let energy_base = energy_settled.unwrap_or(data.energy_initial);
    let energy_drift_pct = if energy_base.abs() > 1e-15 {
        (energy_final - energy_base) / energy_base.abs() * 100.0
    } else {
        (energy_final - energy_base) * 100.0
    };

    SolverResult {
        name,
        avg_iter: total_iter as f64 / STEPS as f64,
        max_iter,
        max_drift_mm: max_drift * 1000.0,
        max_depth_mm: max_depth * 1000.0,
        energy_drift_pct,
        fallback_count: if track_fallback {
            Some(fallback_count)
        } else {
            None
        },
    }
}

fn main() {
    println!("Running 3 solvers for {SIM_TIME}s each (dt={DT}, {STEPS} steps)...\n");

    let results = vec![
        run_solver("PGS", "PGS"),
        run_solver("CG", "CG"),
        run_solver("Newton", "Newton"),
    ];

    // Print comparison table
    println!("=== Solver Comparison (t = {SIM_TIME}s, dt = {DT}) ===\n");
    println!(
        "  {:<12} {:>10} {:>10} {:>12} {:>14} {:>12} {:>10}",
        "Solver", "Avg iter", "Max iter", "Drift(mm)", "Max depth(mm)", "E drift(%)", "Fallback"
    );
    println!("  {}", "\u{2500}".repeat(82));
    for r in &results {
        let fallback = r
            .fallback_count
            .map_or_else(|| "n/a".to_string(), |n| format!("{n}/{STEPS}"));
        println!(
            "  {:<12} {:>10.1} {:>10} {:>12.3} {:>14.4} {:>+12.4}% {:>10}",
            r.name,
            r.avg_iter,
            r.max_iter,
            r.max_drift_mm,
            r.max_depth_mm,
            r.energy_drift_pct,
            fallback,
        );
    }
    println!();

    // Validation checks
    let pgs = &results[0];
    let cg = &results[1];
    let newton = &results[2];

    let checks: Vec<(&str, bool, String)> = vec![
        (
            "All solvers stable: max drift < 1mm",
            results.iter().all(|r| r.max_drift_mm < 1.0),
            format!(
                "max drifts: PGS={:.3}mm, CG={:.3}mm, Newton={:.3}mm",
                pgs.max_drift_mm, cg.max_drift_mm, newton.max_drift_mm,
            ),
        ),
        (
            "Newton converges fast: avg iter <= 5",
            newton.avg_iter <= 5.0,
            format!("avg={:.1}", newton.avg_iter),
        ),
        (
            "PGS converges: avg iter <= 50",
            pgs.avg_iter <= 50.0,
            format!("avg={:.1}", pgs.avg_iter),
        ),
        (
            "CG faster than PGS: avg_iter_CG < avg_iter_PGS",
            cg.avg_iter < pgs.avg_iter,
            format!("CG={:.1} vs PGS={:.1}", cg.avg_iter, pgs.avg_iter),
        ),
        (
            "Newton faster than CG: avg_iter_Newton < avg_iter_CG",
            newton.avg_iter < cg.avg_iter,
            format!("Newton={:.1} vs CG={:.1}", newton.avg_iter, cg.avg_iter),
        ),
        (
            "Constraint violation < 1mm for all solvers",
            results.iter().all(|r| r.max_depth_mm < 1.0),
            format!(
                "max depths: PGS={:.4}mm, CG={:.4}mm, Newton={:.4}mm",
                pgs.max_depth_mm, cg.max_depth_mm, newton.max_depth_mm,
            ),
        ),
        (
            "Energy stable: |drift| < 1% for all solvers",
            results.iter().all(|r| r.energy_drift_pct.abs() < 1.0),
            format!(
                "drifts: PGS={:+.4}%, CG={:+.4}%, Newton={:+.4}%",
                pgs.energy_drift_pct, cg.energy_drift_pct, newton.energy_drift_pct,
            ),
        ),
    ];

    let mut pass_count = 0;
    let total = checks.len();
    for (name, pass, detail) in &checks {
        let status = if *pass {
            pass_count += 1;
            "PASS"
        } else {
            "FAIL"
        };
        println!("  [{status}] {name}: {detail}");
    }
    println!();

    if pass_count == total {
        println!("  PASS: {pass_count}/{total} checks passed");
    } else {
        println!("  FAIL: {pass_count}/{total} checks passed");
        std::process::exit(1);
    }
}
