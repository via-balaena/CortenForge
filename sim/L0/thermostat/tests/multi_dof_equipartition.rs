//! Phase 2 integration tests — multi-DOF equipartition.
//!
//! Per Phase 2 spec §5–§9: validates the `LangevinThermostat` on
//! multi-DOF systems without any code changes to `sim-thermostat/src/`.
//!
//! - **Model A** (free body, 6 DOF): per-DOF `⟨½ M_ii v_i²⟩ = ½k_BT`
//!   with asymmetric inertia (`I_xx ≠ I_yy ≠ I_zz`).
//! - **Model B** (2-link hinge chain, 2 DOF): total KE
//!   `⟨½ v^T M(q) v⟩ = k_BT` and generalized per-DOF
//!   `⟨v_i · (M(q)·v)_i⟩ = k_BT` with non-diagonal,
//!   configuration-dependent mass matrix.
//!
//! Phase 1's equipartition gate must already pass — these tests extend
//! it to multi-DOF systems. The thermostat implementation is unchanged.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use sim_core::{DVector, ENABLE_ENERGY, Integrator};
use sim_thermostat::test_utils::{WelfordOnline, assert_within_n_sigma};
use sim_thermostat::{LangevinThermostat, PassiveStack};

// ─── MJCF models ─────────────────────────────────────────────────────

/// Model A — free body with asymmetric inertia (6 DOF).
///
/// Mass matrix is `M = diag(1, 1, 1, 0.5, 1.0, 1.5)` — three distinct
/// principal moments so the three rotational DOFs have different
/// equilibrium velocity variances. A sphere (uniform `I`) would pass
/// even if the thermostat treated all DOFs identically; the asymmetric
/// box is the informative test.
///
/// `contype="0" conaffinity="0"` disables collision.
/// No gravity, no springs — pure thermal motion.
const FREE_BODY_XML: &str = r#"
<mujoco model="free_body_6dof">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.5 1.0 1.5"/>
      <geom type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model B — 2-link hinge chain (2 DOF).
///
/// Two links rotating around the y-axis (planar chain in x-z plane).
/// Joint springs (`stiffness="20"`) provide a restoring potential that
/// keeps joint angles in the small-angle regime (~0.22 rad). This is
/// load-bearing: the Euler-Maruyama integrator introduces systematic
/// bias when the configuration-dependent mass matrix M(q) changes
/// significantly per step. At stiffness=1 (angles ~1.9 rad), the KE
/// inflates to ~3.8× expected; at stiffness=20 (angles ~0.22 rad),
/// both per-DOF tests pass within 1σ. This is an integrator
/// limitation, not a thermostat limitation — the force-space FDT is
/// correct. BAOAB or GJF integrators would remove this constraint
/// (clear upgrade path, not Phase 2 work).
///
/// `damping="0"` — thermostat owns damping (Q4).
///
/// Mass and inertia come entirely from `<inertial>` elements;
/// `geom mass="0"` prevents double-counting. The 2×2 mass matrix
/// `M(q)` is non-diagonal and configuration-dependent (off-diagonal
/// coupling via `cos(q₂)`).
const HINGE_CHAIN_XML: &str = r#"
<mujoco model="hinge_chain_2dof">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="link1">
      <joint name="j1" type="hinge" axis="0 1 0"
             stiffness="20" damping="0" springref="0"/>
      <inertial pos="0.25 0 0" mass="1" diaginertia="0.01 0.1 0.1"/>
      <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05"
            contype="0" conaffinity="0" mass="0"/>
      <body name="link2" pos="0.5 0 0">
        <joint name="j2" type="hinge" axis="0 1 0"
               stiffness="20" damping="0" springref="0"/>
        <inertial pos="0.25 0 0" mass="1" diaginertia="0.01 0.1 0.1"/>
        <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05"
              contype="0" conaffinity="0" mass="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

// ─── §5.1 free body model invariants ─────────────────────────────────

/// Verifies Model A satisfies every property from spec §5.1.
#[test]
fn test_free_body_model_invariants() {
    let model = sim_mjcf::load_model(FREE_BODY_XML).expect("MJCF should load");

    assert_eq!(model.nv, 6, "free body should have 6 velocity DOFs");
    assert_eq!(
        model.nq, 7,
        "free body should have 7 position DOFs (3 pos + 4 quat)"
    );
    assert_eq!(model.timestep, 0.001, "timestep must be 0.001");
    assert!(
        matches!(model.integrator, Integrator::Euler),
        "integrator must be Euler, got {:?}",
        model.integrator,
    );
    assert_eq!(model.body_mass[1], 1.0, "body mass must be 1.0");

    // All 6 DOFs must have zero model damping — thermostat owns damping.
    for i in 0..6 {
        assert_eq!(
            model.dof_damping[i], 0.0,
            "dof_damping[{i}] must be zero — thermostat owns damping",
        );
    }
}

// ─── §6.1 hinge chain model invariants ───────────────────────────────

/// Verifies Model B satisfies every property from spec §6.1, including
/// the non-trivial off-diagonal coupling in M(q) at non-zero angles.
#[test]
fn test_hinge_chain_model_invariants() {
    let model = sim_mjcf::load_model(HINGE_CHAIN_XML).expect("MJCF should load");

    assert_eq!(model.nv, 2, "hinge chain should have 2 velocity DOFs");
    assert_eq!(model.nq, 2, "hinge chain should have 2 position DOFs");
    assert_eq!(model.timestep, 0.001, "timestep must be 0.001");
    assert!(
        matches!(model.integrator, Integrator::Euler),
        "integrator must be Euler, got {:?}",
        model.integrator,
    );

    for i in 0..2 {
        assert_eq!(
            model.dof_damping[i], 0.0,
            "dof_damping[{i}] must be zero — thermostat owns damping",
        );
    }

    // Verify M is 2×2 and has non-zero off-diagonal at a non-zero angle,
    // confirming the mass matrix is truly coupled.
    let mut data = model.make_data();
    data.qpos[1] = 0.5; // non-zero angle activates coupling
    data.forward(&model).expect("forward");
    assert_eq!(data.qM.nrows(), 2, "qM should be 2×2");
    assert_eq!(data.qM.ncols(), 2, "qM should be 2×2");
    assert!(
        data.qM[(0, 1)].abs() > 1e-6,
        "M should have non-zero off-diagonal at non-zero q[1], got M_01={}",
        data.qM[(0, 1)],
    );
}

// ─── §7 free body equipartition (6 DOF) ─────────────────────────────

/// Per-DOF equipartition on a 6-DOF free body with asymmetric inertia.
///
/// M = diag(1, 1, 1, 0.5, 1.0, 1.5). Each DOF is tested independently:
/// `⟨½ M_ii v_i²⟩ = ½k_BT`. The three rotational DOFs have different
/// effective masses, so the test catches any per-DOF temperature error.
///
/// Two-level Welford: 100 trajectories, per-trajectory inner accumulator
/// reduced to a scalar mean, pushed into per-DOF top-level accumulators.
///
/// Burn-in and measurement windows scaled to the slowest DOF (`I_zz`=1.5):
/// `τ_eq = M/γ = 15` time units = `15,000` steps. `n_burn_in = 5·τ_eq`,
/// `n_measure = 20·τ_eq`.
#[test]
fn test_free_body_equipartition() {
    let m_diag: [f64; 6] = [1.0, 1.0, 1.0, 0.5, 1.0, 1.5];
    let n_burn_in = 75_000_usize;
    let n_measure = 300_000_usize;
    let n_traj = 100_usize;
    let seed_base = 0xFACE_B00D_u64;
    let k_b_t = 1.0_f64;
    let gamma_value = 0.1_f64;

    // Per-DOF top-level accumulators over the 100 trajectory means.
    let mut across: Vec<WelfordOnline> = (0..6).map(|_| WelfordOnline::new()).collect();

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(FREE_BODY_XML).expect("load");
        let mut data = model.make_data();

        PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed_base + i as u64,
            ))
            .build()
            .install(&mut model);

        // Burn in — no measurement.
        for _ in 0..n_burn_in {
            data.step(&model).expect("burn-in step");
        }

        // Per-trajectory inner accumulators: one per DOF.
        let mut traj: Vec<WelfordOnline> = (0..6).map(|_| WelfordOnline::new()).collect();
        for _ in 0..n_measure {
            data.step(&model).expect("measure step");
            for (dof, m_ii) in m_diag.iter().enumerate() {
                traj[dof].push(0.5 * m_ii * data.qvel[dof] * data.qvel[dof]);
            }
        }

        for dof in 0..6 {
            across[dof].push(traj[dof].mean());
        }
    }

    let expected = 0.5 * k_b_t;
    let dof_names = ["v_x", "v_y", "v_z", "ω_x", "ω_y", "ω_z"];
    for (dof, name) in dof_names.iter().enumerate() {
        assert_within_n_sigma(
            across[dof].mean(),
            expected,
            across[dof].std_error_of_mean(),
            3.0,
            &format!(
                "free body equipartition DOF {dof} ({name}), M_ii={}",
                m_diag[dof],
            ),
        );
    }
}

// ─── §8 hinge chain equipartition (2 DOF) ───────────────────────────

/// Total KE + generalized per-DOF equipartition on a 2-DOF hinge chain
/// with non-diagonal, configuration-dependent mass matrix M(q).
///
/// Two measurement types per step:
/// - Total KE via `data.energy_kinetic`: `⟨T_kin⟩ = (n_dof/2)·k_BT = k_BT`.
/// - Generalized per-DOF: `⟨v_i · (M(q)·v)_i⟩ = k_BT` for each DOF.
///
/// **Measurement timing**: after `step()`, `qvel` is post-integration
/// but `qM` is pre-integration (computed from old `qpos` during the
/// forward pass). The lag is O(h) = O(0.001) — negligible vs. the ~3%
/// tolerance. `energy_kinetic` is `½ v_old^T M(q_old) v_old` — the KE
/// at the start of the step.
#[test]
fn test_hinge_chain_equipartition() {
    let n_burn_in = 50_000_usize;
    let n_measure = 200_000_usize;
    let n_traj = 100_usize;
    let seed_base = 0xDEAD_F00D_u64;
    let k_b_t = 1.0_f64;
    let gamma_value = 0.1_f64;

    let mut across_total_ke = WelfordOnline::new();
    let mut across_gen: Vec<WelfordOnline> = (0..2).map(|_| WelfordOnline::new()).collect();

    for i in 0..n_traj {
        let mut model = sim_mjcf::load_model(HINGE_CHAIN_XML).expect("load");
        model.enableflags |= ENABLE_ENERGY;
        let mut data = model.make_data();

        PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(model.nv, gamma_value),
                k_b_t,
                seed_base + i as u64,
            ))
            .build()
            .install(&mut model);

        // Burn in — no measurement.
        for _ in 0..n_burn_in {
            data.step(&model).expect("burn-in step");
        }

        let mut traj_total_ke = WelfordOnline::new();
        let mut traj_gen: Vec<WelfordOnline> = (0..2).map(|_| WelfordOnline::new()).collect();
        for _ in 0..n_measure {
            data.step(&model).expect("measure step");

            // Total KE — pre-computed by the energy stage.
            traj_total_ke.push(data.energy_kinetic);

            // Generalized per-DOF: v_i · (M(q)·v)_i.
            // qM is from the forward pass (pre-integration qpos);
            // qvel is post-integration. O(h) lag is negligible.
            let mv = &data.qM * &data.qvel;
            for dof in 0..2 {
                traj_gen[dof].push(data.qvel[dof] * mv[dof]);
            }
        }

        across_total_ke.push(traj_total_ke.mean());
        for dof in 0..2 {
            across_gen[dof].push(traj_gen[dof].mean());
        }
    }

    // Total KE: ⟨T_kin⟩ = (n_dof/2) · k_BT = 2/2 · 1.0 = 1.0
    let expected_total = k_b_t;
    assert_within_n_sigma(
        across_total_ke.mean(),
        expected_total,
        across_total_ke.std_error_of_mean(),
        3.0,
        "hinge chain total KE: ⟨½v^T M v⟩ = k_BT",
    );

    // Generalized per-DOF: ⟨v_i · (M(q)·v)_i⟩ = k_BT
    for (dof, acc) in across_gen.iter().enumerate() {
        assert_within_n_sigma(
            acc.mean(),
            k_b_t,
            acc.std_error_of_mean(),
            3.0,
            &format!("hinge chain generalized equipartition DOF {dof}"),
        );
    }
}

// ─── §9 multi-DOF reproducibility ───────────────────────────────────

/// Bit-for-bit reproducibility on the 6-DOF free body.
#[test]
fn test_reproducibility_free_body() {
    assert_multi_dof_reproducibility(FREE_BODY_XML, 6, 7, 10_000);
}

/// Bit-for-bit reproducibility on the 2-DOF hinge chain.
#[test]
fn test_reproducibility_hinge_chain() {
    assert_multi_dof_reproducibility(HINGE_CHAIN_XML, 2, 2, 10_000);
}

/// Shared helper: two simulations with identical parameters and seed
/// must produce bit-for-bit identical `qpos` and `qvel` after N steps.
/// Hard f64 equality, no `assert_relative_eq!`.
fn assert_multi_dof_reproducibility(xml: &str, nv: usize, nq: usize, n_steps: usize) {
    let seed = 0x00C0_FFEE_u64;
    let gamma_value = 0.1;
    let k_b_t = 1.0;

    let mut model1 = sim_mjcf::load_model(xml).expect("load 1");
    let mut data1 = model1.make_data();
    PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model1.nv, gamma_value),
            k_b_t,
            seed,
        ))
        .build()
        .install(&mut model1);

    let mut model2 = sim_mjcf::load_model(xml).expect("load 2");
    let mut data2 = model2.make_data();
    PassiveStack::builder()
        .with(LangevinThermostat::new(
            DVector::from_element(model2.nv, gamma_value),
            k_b_t,
            seed,
        ))
        .build()
        .install(&mut model2);

    for _ in 0..n_steps {
        data1.step(&model1).expect("sim 1 step");
        data2.step(&model2).expect("sim 2 step");
    }

    for i in 0..nq {
        assert_eq!(
            data1.qpos[i], data2.qpos[i],
            "reproducibility: qpos[{i}] must match bit-for-bit after {n_steps} steps",
        );
    }
    for i in 0..nv {
        assert_eq!(
            data1.qvel[i], data2.qvel[i],
            "reproducibility: qvel[{i}] must match bit-for-bit after {n_steps} steps",
        );
    }
}
