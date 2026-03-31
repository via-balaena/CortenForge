//! Stress test — headless validation of the muscle actuator system.
//!
//! 24 checks covering: activation dynamics (7), MuJoCo curve pinning (4),
//! Hill curve pinning (3), MuJoCo full-pipeline force (2), HillMuscle
//! full-pipeline force (5), dynamic behavior (3).
//!
//! Run: `cargo run -p example-muscle-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::while_float
)]

use sim_core::forward::{
    hill_active_fl, hill_force_velocity, hill_passive_fl, muscle_activation_dynamics,
    muscle_gain_length, muscle_gain_velocity, muscle_passive_force,
};

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

// ============================================================================
// Group A: Activation Dynamics (shared by Muscle and HillMuscle)
// ============================================================================

// ── A1: Activation asymmetry ──────────────────────────────────────────────

fn check_a1_activation_asymmetry() -> (u32, u32) {
    let dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let dt = 0.001;

    // Rise: ctrl=1 from act=0 until act≥0.9
    let mut act: f64 = 0.0;
    let mut rise_steps = 0u32;
    while act < 0.9 {
        let dact = muscle_activation_dynamics(1.0, act, &dynprm);
        act += dt * dact;
        act = act.clamp(0.0, 1.0);
        rise_steps += 1;
        if rise_steps > 10_000 {
            break;
        }
    }

    // Fall: ctrl=0 from act=1 until act≤0.1
    act = 1.0;
    let mut fall_steps = 0u32;
    while act > 0.1 {
        let dact = muscle_activation_dynamics(0.0, act, &dynprm);
        act += dt * dact;
        act = act.clamp(0.0, 1.0);
        fall_steps += 1;
        if fall_steps > 10_000 {
            break;
        }
    }

    let p = check(
        "A1 Activation asymmetry",
        rise_steps < fall_steps,
        &format!("rise={rise_steps} steps, fall={fall_steps} steps (rise must be faster)"),
    );
    (u32::from(p), 1)
}

// ── A2: Hard-switch exact value (activation path) ─────────────────────────

fn check_a2_hard_switch_activation() -> (u32, u32) {
    let dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let act_dot = muscle_activation_dynamics(0.8, 0.3, &dynprm);
    // act_clamped=0.3, tau_act=0.01×(0.5+0.45)=0.0095, dctrl=0.5>0
    // act_dot = 0.5 / 0.0095 = 52.631578947368421
    let expected = 0.5 / 0.0095;
    let err = (act_dot - expected).abs();

    let p = check(
        "A2 Hard-switch activation",
        err < 1e-10,
        &format!("act_dot={act_dot:.15}, expected={expected:.15}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── A3: Hard-switch exact value (deactivation path) ───────────────────────

fn check_a3_hard_switch_deactivation() -> (u32, u32) {
    let dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let act_dot = muscle_activation_dynamics(0.2, 0.6, &dynprm);
    // act_clamped=0.6, tau_deact=0.04/(0.5+0.9)=0.04/1.4, dctrl=-0.4
    let tau_deact = 0.04 / (0.5 + 1.5 * 0.6);
    let expected = -0.4 / tau_deact;
    let err = (act_dot - expected).abs();

    let p = check(
        "A3 Hard-switch deactivation",
        err < 1e-10,
        &format!("act_dot={act_dot:.10}, expected={expected:.10}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── A4: Boundary — act=0 ──────────────────────────────────────────────────

fn check_a4_boundary_act_zero() -> (u32, u32) {
    let dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let act_dot = muscle_activation_dynamics(0.5, 0.0, &dynprm);
    // tau_act=0.01×0.5=0.005, dctrl=0.5
    let expected = 0.5 / 0.005; // 100.0
    let err = (act_dot - expected).abs();

    let p = check(
        "A4 Boundary act=0",
        err < 1e-10,
        &format!("act_dot={act_dot:.10}, expected={expected:.10}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── A5: Boundary — act=1 ──────────────────────────────────────────────────

fn check_a5_boundary_act_one() -> (u32, u32) {
    let dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let act_dot = muscle_activation_dynamics(0.5, 1.0, &dynprm);
    // tau_deact=0.04/2.0=0.02, dctrl=-0.5
    let expected = -0.5 / 0.02; // -25.0
    let err = (act_dot - expected).abs();

    let p = check(
        "A5 Boundary act=1",
        err < 1e-10,
        &format!("act_dot={act_dot:.10}, expected={expected:.10}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── A6: Ctrl clamping ─────────────────────────────────────────────────────

fn check_a6_ctrl_clamping() -> (u32, u32) {
    let dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let dot_clamped = muscle_activation_dynamics(1.5, 0.5, &dynprm);
    let dot_ref = muscle_activation_dynamics(1.0, 0.5, &dynprm);
    let err = (dot_clamped - dot_ref).abs();

    let p = check(
        "A6 Ctrl clamping",
        err < 1e-15,
        &format!("ctrl=1.5 → {dot_clamped:.15}, ctrl=1.0 → {dot_ref:.15}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── A7: Smooth blend ──────────────────────────────────────────────────────

fn check_a7_smooth_blend() -> (u32, u32) {
    let dynprm = [0.01, 0.04, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let act_dot = muscle_activation_dynamics(0.55, 0.5, &dynprm);
    // Precomputed from spec equations
    let err = (act_dot - 2.798_737).abs();

    let p = check(
        "A7 Smooth blend",
        err < 1e-3,
        &format!("act_dot={act_dot:.6}, expected≈2.798737, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group B: MuJoCo Muscle Curve Pinning
// ============================================================================

// ── B1: MuJoCo FL curve — pinned values ───────────────────────────────────

fn check_b1_mujoco_fl_pinned() -> (u32, u32) {
    let mut p = 0u32;
    let t = 5;

    // lmin=0.5, lmax=1.6 → a=0.75, b=1.3
    let fl_at_lmin = muscle_gain_length(0.5, 0.5, 1.6);
    p += u32::from(check(
        "B1a FL(0.5)=0",
        fl_at_lmin.abs() < 1e-10,
        &format!("FL(0.5)={fl_at_lmin:.15}"),
    ));

    let fl_at_a = muscle_gain_length(0.75, 0.5, 1.6);
    p += u32::from(check(
        "B1b FL(0.75)=0.5",
        (fl_at_a - 0.5).abs() < 1e-10,
        &format!("FL(0.75)={fl_at_a:.15}"),
    ));

    let fl_peak = muscle_gain_length(1.0, 0.5, 1.6);
    p += u32::from(check(
        "B1c FL(1.0)=1.0",
        (fl_peak - 1.0).abs() < 1e-10,
        &format!("FL(1.0)={fl_peak:.15}"),
    ));

    let fl_at_b = muscle_gain_length(1.3, 0.5, 1.6);
    p += u32::from(check(
        "B1d FL(1.3)=0.5",
        (fl_at_b - 0.5).abs() < 1e-10,
        &format!("FL(1.3)={fl_at_b:.15}"),
    ));

    let fl_at_lmax = muscle_gain_length(1.6, 0.5, 1.6);
    p += u32::from(check(
        "B1e FL(1.6)=0",
        fl_at_lmax.abs() < 1e-10,
        &format!("FL(1.6)={fl_at_lmax:.15}"),
    ));

    (p, t)
}

// ── B2: MuJoCo FL — outside range ─────────────────────────────────────────

fn check_b2_mujoco_fl_outside() -> (u32, u32) {
    let mut p = 0u32;
    let t = 2;

    let fl_below = muscle_gain_length(0.3, 0.5, 1.6);
    p += u32::from(check(
        "B2a FL(0.3)=0",
        fl_below == 0.0,
        &format!("FL(0.3)={fl_below}"),
    ));

    let fl_above = muscle_gain_length(1.8, 0.5, 1.6);
    p += u32::from(check(
        "B2b FL(1.8)=0",
        fl_above == 0.0,
        &format!("FL(1.8)={fl_above}"),
    ));

    (p, t)
}

// ── B3: MuJoCo FV curve — pinned values ──────────────────────────────────

fn check_b3_mujoco_fv_pinned() -> (u32, u32) {
    let mut p = 0u32;
    let t = 4;

    let fv_max_short = muscle_gain_velocity(-1.0, 1.4);
    p += u32::from(check(
        "B3a FV(-1.0)=0",
        fv_max_short.abs() < 1e-10,
        &format!("FV(-1.0)={fv_max_short:.15}"),
    ));

    let fv_concentric = muscle_gain_velocity(-0.5, 1.4);
    p += u32::from(check(
        "B3b FV(-0.5)=0.25",
        (fv_concentric - 0.25).abs() < 1e-10,
        &format!("FV(-0.5)={fv_concentric:.15}"),
    ));

    let fv_iso = muscle_gain_velocity(0.0, 1.4);
    p += u32::from(check(
        "B3c FV(0.0)=1.0",
        (fv_iso - 1.0).abs() < 1e-10,
        &format!("FV(0.0)={fv_iso:.15}"),
    ));

    // y = fvmax - 1 = 0.4, V=0.4 >= y → plateau
    let fv_plateau = muscle_gain_velocity(0.4, 1.4);
    p += u32::from(check(
        "B3d FV(0.4)=1.4",
        (fv_plateau - 1.4).abs() < 1e-10,
        &format!("FV(0.4)={fv_plateau:.15}"),
    ));

    (p, t)
}

// ── B4: MuJoCo FP curve — pinned values ──────────────────────────────────

fn check_b4_mujoco_fp_pinned() -> (u32, u32) {
    let mut p = 0u32;
    let t = 4;

    let fp_below = muscle_passive_force(0.8, 1.6, 0.3);
    p += u32::from(check(
        "B4a FP(0.8)=0",
        fp_below == 0.0,
        &format!("FP(0.8)={fp_below}"),
    ));

    let fp_optimal = muscle_passive_force(1.0, 1.6, 0.3);
    p += u32::from(check(
        "B4b FP(1.0)=0",
        fp_optimal == 0.0,
        &format!("FP(1.0)={fp_optimal}"),
    ));

    // b = 0.5*(1+1.6)=1.3, at b: fpmax * 0.5 = 0.15
    let fp_at_b = muscle_passive_force(1.3, 1.6, 0.3);
    p += u32::from(check(
        "B4c FP(1.3)=0.15",
        (fp_at_b - 0.15).abs() < 1e-10,
        &format!("FP(1.3)={fp_at_b:.15}"),
    ));

    // Above b: linear, FP(1.5) = fpmax*(0.5 + (1.5-1.3)/(1.3-1.0)) = 0.3*(0.5+0.6667) = 0.35
    let fp_above = muscle_passive_force(1.5, 1.6, 0.3);
    let expected = 0.3 * (0.5 + (1.5 - 1.3) / (1.3 - 1.0));
    p += u32::from(check(
        "B4d FP(1.5)=0.35",
        (fp_above - expected).abs() < 1e-10,
        &format!("FP(1.5)={fp_above:.15}, expected={expected:.15}"),
    ));

    (p, t)
}

// ============================================================================
// Group C: HillMuscle Curve Pinning
// ============================================================================

// ── C1: Hill FL curve — pinned values ─────────────────────────────────────

fn check_c1_hill_fl_pinned() -> (u32, u32) {
    let mut p = 0u32;
    let t = 6;

    // Below l_min=0.5
    let fl_below = hill_active_fl(0.49);
    p += u32::from(check(
        "C1a FL(0.49)=0",
        fl_below == 0.0,
        &format!("FL(0.49)={fl_below}"),
    ));

    // At l_min boundary — nonzero
    let fl_lmin = hill_active_fl(0.5);
    p += u32::from(check(
        "C1b FL(0.5)>0",
        fl_lmin > 0.0,
        &format!("FL(0.5)={fl_lmin:.10}"),
    ));

    // Ascending side: w_asc=0.45
    // FL(0.8) = exp(-((0.8-1.0)/0.45)^2) = exp(-(0.2/0.45)^2) = exp(-0.197531)
    let expected_08 = (-((0.8 - 1.0) / 0.45_f64).powi(2)).exp();
    let fl_08 = hill_active_fl(0.8);
    p += u32::from(check(
        "C1c FL(0.8)",
        (fl_08 - expected_08).abs() < 1e-10,
        &format!("FL(0.8)={fl_08:.10}, expected={expected_08:.10}"),
    ));

    // Peak
    let fl_peak = hill_active_fl(1.0);
    p += u32::from(check(
        "C1d FL(1.0)=1.0",
        (fl_peak - 1.0).abs() < 1e-10,
        &format!("FL(1.0)={fl_peak:.15}"),
    ));

    // Descending side: w_desc=0.56
    // FL(1.2) = exp(-((1.2-1.0)/0.56)^2) = exp(-(0.2/0.56)^2)
    let expected_12 = (-((1.2 - 1.0) / 0.56_f64).powi(2)).exp();
    let fl_12 = hill_active_fl(1.2);
    p += u32::from(check(
        "C1e FL(1.2)",
        (fl_12 - expected_12).abs() < 1e-10,
        &format!("FL(1.2)={fl_12:.10}, expected={expected_12:.10}"),
    ));

    // Above l_max=1.6
    let fl_above = hill_active_fl(1.61);
    p += u32::from(check(
        "C1f FL(1.61)=0",
        fl_above == 0.0,
        &format!("FL(1.61)={fl_above}"),
    ));

    (p, t)
}

// ── C2: Hill FV curve — pinned values ─────────────────────────────────────

fn check_c2_hill_fv_pinned() -> (u32, u32) {
    let mut p = 0u32;
    let t = 5;

    let a = 0.25_f64;
    let fv_max = 1.5_f64;

    let fv_short = hill_force_velocity(-1.0);
    p += u32::from(check(
        "C2a FV(-1.0)=0",
        fv_short == 0.0,
        &format!("FV(-1.0)={fv_short}"),
    ));

    // Concentric: (1+V)/(1-V/a) = (1-0.5)/(1+0.5/0.25) = 0.5/3.0
    let expected_conc = (1.0 - 0.5) / (1.0 + 0.5 / a);
    let fv_conc = hill_force_velocity(-0.5);
    p += u32::from(check(
        "C2b FV(-0.5)",
        (fv_conc - expected_conc).abs() < 1e-10,
        &format!("FV(-0.5)={fv_conc:.10}, expected={expected_conc:.10}"),
    ));

    let fv_iso = hill_force_velocity(0.0);
    p += u32::from(check(
        "C2c FV(0.0)=1.0",
        (fv_iso - 1.0).abs() < 1e-10,
        &format!("FV(0.0)={fv_iso:.15}"),
    ));

    // Eccentric: 1 + (fv_max-1)*V/(V+a) = 1 + 0.5*0.5/(0.5+0.25)
    let expected_ecc = 1.0 + (fv_max - 1.0) * 0.5 / (0.5 + a);
    let fv_ecc = hill_force_velocity(0.5);
    p += u32::from(check(
        "C2d FV(0.5)",
        (fv_ecc - expected_ecc).abs() < 1e-10,
        &format!("FV(0.5)={fv_ecc:.10}, expected={expected_ecc:.10}"),
    ));

    // Approaching plateau: V=1.0
    let expected_plat = 1.0 + (fv_max - 1.0) * 1.0 / (1.0 + a);
    let fv_plat = hill_force_velocity(1.0);
    p += u32::from(check(
        "C2e FV(1.0)",
        (fv_plat - expected_plat).abs() < 1e-10,
        &format!("FV(1.0)={fv_plat:.10}, expected={expected_plat:.10}"),
    ));

    (p, t)
}

// ── C3: Hill FP curve — pinned values ─────────────────────────────────────

fn check_c3_hill_fp_pinned() -> (u32, u32) {
    let mut p = 0u32;
    let t = 4;
    let shape = 4.0_f64;

    let fp_below = hill_passive_fl(0.9);
    p += u32::from(check(
        "C3a FP(0.9)=0",
        fp_below == 0.0,
        &format!("FP(0.9)={fp_below}"),
    ));

    let fp_optimal = hill_passive_fl(1.0);
    p += u32::from(check(
        "C3b FP(1.0)=0",
        fp_optimal == 0.0,
        &format!("FP(1.0)={fp_optimal}"),
    ));

    // FP(1.25) = (exp(4*0.25)-1)/(exp(4)-1) = (e^1 - 1)/(e^4 - 1)
    let expected_125 = (shape * 0.25).exp_m1() / shape.exp_m1();
    let fp_125 = hill_passive_fl(1.25);
    p += u32::from(check(
        "C3c FP(1.25)",
        (fp_125 - expected_125).abs() < 1e-10,
        &format!("FP(1.25)={fp_125:.10}, expected={expected_125:.10}"),
    ));

    // FP(1.5) = (exp(4*0.5)-1)/(exp(4)-1) = (e^2 - 1)/(e^4 - 1)
    let expected_15 = (shape * 0.5).exp_m1() / shape.exp_m1();
    let fp_15 = hill_passive_fl(1.5);
    p += u32::from(check(
        "C3d FP(1.5)",
        (fp_15 - expected_15).abs() < 1e-10,
        &format!("FP(1.5)={fp_15:.10}, expected={expected_15:.10}"),
    ));

    (p, t)
}

// ============================================================================
// Group D: MuJoCo Muscle Full Pipeline
// ============================================================================

/// MuJoCo `<muscle>` on a hinge forearm — explicit F0.
const MUJOCO_MUSCLE_MJCF: &str = r#"
<mujoco model="mujoco-muscle">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <muscle name="bicep" joint="elbow" force="500"
            range="0.75 1.05" lmin="0.5" lmax="1.6"
            vmax="1.0" fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
  </actuator>
</mujoco>
"#;

// ── D1: MuJoCo muscle — isometric force at steady state ──────────────────

fn check_d1_mujoco_isometric() -> (u32, u32) {
    let model = sim_mjcf::load_model(MUJOCO_MUSCLE_MJCF).expect("parse");
    let mut data = model.make_data();

    // ctrl=1, simulate until activation is near 1.0
    data.ctrl[0] = 1.0;
    for _ in 0..2_000 {
        data.step(&model).expect("step");
    }

    let force = data.actuator_force[0];
    let f0 = model.actuator_gainprm[0][2]; // resolved F0

    // At steady state: activation≈1, force should be dominated by -F0*FL*FV
    // The exact value depends on joint position (which changes as muscle acts),
    // but force must be negative (pulling) and substantial
    let p = check(
        "D1 MuJoCo isometric",
        force < -1.0 && f0 > 0.0,
        &format!("force={force:.4}, F0={f0:.4} (force must be negative, F0 positive)"),
    );
    (u32::from(p), 1)
}

// ── D2: MuJoCo muscle — passive force at stretch ─────────────────────────

fn check_d2_mujoco_passive() -> (u32, u32) {
    let model = sim_mjcf::load_model(MUJOCO_MUSCLE_MJCF).expect("parse");
    let mut data = model.make_data();

    // No activation (ctrl=0, act=0), stretch joint to positive angle.
    // Use forward() to compute force at this static position.
    data.ctrl[0] = 0.0;
    data.qpos[0] = 1.2; // stretch beyond neutral
    data.forward(&model).expect("forward");

    let force = data.actuator_force[0];

    // With act=0: force = gain*0 + bias = bias = -F0 * FP(norm_len)
    // Passive force should resist stretch (negative force).
    let p = check(
        "D2 MuJoCo passive",
        force < 0.0,
        &format!("force={force:.6} (must be negative — passive resists stretch)"),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group E: HillMuscle Full Pipeline
// ============================================================================

/// HillMuscle with explicit F0=1000, zero pennation.
///
/// Parameter design for joint transmission:
///   actuator_length = gear × qpos.  At qpos=0, length=0.
///   fiber_length = (length − L_slack) / cos(α)
///   norm_len = fiber_length / L_opt
///
///   L_opt = 1.0, L_slack = 0.0, penn = 0:
///     At qpos=1.0: fiber = (1 − 0) / 1 = 1.0, norm_len = 1.0 (optimal)
///     At qpos=1.5: norm_len = 1.5 (stretched, passive force active)
///     At qpos=0.5: norm_len = 0.5 (shortened, FL at boundary)
const HILL_ZERO_PENN_MJCF: &str = r#"
<mujoco model="hill-zero-penn">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <general name="hill_muscle" joint="elbow"
             dyntype="hillmuscle"
             gainprm="0.75 1.05 1000 200 1.0 0.0 10.0 0.0 35.0"
             dynprm="0.01 0.04 0.0"/>
  </actuator>
</mujoco>
"#;

/// HillMuscle with pennation angle = 20° (0.349066 rad).
/// Same L_slack/L_opt as zero-penn model for direct comparison.
const HILL_PENN_20_MJCF: &str = r#"
<mujoco model="hill-penn-20">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <general name="hill_muscle" joint="elbow"
             dyntype="hillmuscle"
             gainprm="0.75 1.05 1000 200 1.0 0.0 10.0 0.349066 35.0"
             dynprm="0.01 0.04 0.0"/>
  </actuator>
</mujoco>
"#;

/// HillMuscle with F0=-1 (auto-compute from scale/acc0).
const HILL_AUTO_F0_MJCF: &str = r#"
<mujoco model="hill-auto-f0">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <general name="hill_muscle" joint="elbow"
             dyntype="hillmuscle"
             gainprm="0.75 1.05 -1 200 1.0 0.0 10.0 0.0 35.0"
             dynprm="0.01 0.04 0.0"/>
  </actuator>
</mujoco>
"#;

/// Mixed model: one MuJoCo `<muscle>`, one HillMuscle on separate joints.
const MIXED_MJCF: &str = r#"
<mujoco model="mixed-muscle">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="arm_a" pos="0 0 1">
      <body name="forearm_a" pos="0 0 0">
        <joint name="elbow_a" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
    <body name="arm_b" pos="0.5 0 1">
      <body name="forearm_b" pos="0 0 0">
        <joint name="elbow_b" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <muscle name="mj_bicep" joint="elbow_a" force="500"
            range="0.75 1.05" lmin="0.5" lmax="1.6"
            vmax="1.0" fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
    <general name="hill_bicep" joint="elbow_b"
             dyntype="hillmuscle"
             gainprm="0.75 1.05 500 200 1.0 0.0 10.0 0.0 35.0"
             dynprm="0.01 0.04 0.0"/>
  </actuator>
</mujoco>
"#;

// ── E1: HillMuscle — isometric at optimal fiber length, zero pennation ───

fn check_e1_hill_isometric_zero_penn() -> (u32, u32) {
    let model = sim_mjcf::load_model(HILL_ZERO_PENN_MJCF).expect("parse");
    let mut data = model.make_data();

    // Position at optimal fiber length. Set activation directly to 1.0.
    // Use forward() only (no step) to compute force at this exact position.
    data.qpos[0] = 1.0;
    data.ctrl[0] = 1.0;
    let act_adr = model.actuator_act_adr[0];
    data.act[act_adr] = 1.0; // bypass dynamics — set activation directly
    data.forward(&model).expect("forward");

    let force = data.actuator_force[0];
    let f0 = model.actuator_gainprm[0][2]; // should be 1000

    // At optimal length: FL(1.0)=1.0, FV(0.0)=1.0, cos(0)=1.0
    // force = gain * act + bias = (-F0*FL*FV*cos) * 1.0 + (-F0*FP*cos)
    // FP(1.0) = 0, so force ≈ -F0
    let err_pct = ((force + f0) / f0).abs() * 100.0;
    let p = check(
        "E1 Hill isometric (penn=0)",
        force < -0.99 * f0 && err_pct < 1.0,
        &format!("force={force:.4}, expected≈{:.4}, err={err_pct:.2}%", -f0),
    );
    (u32::from(p), 1)
}

// ── E2: HillMuscle — pennation reduces force ─────────────────────────────

fn check_e2_hill_pennation() -> (u32, u32) {
    let model_0 = sim_mjcf::load_model(HILL_ZERO_PENN_MJCF).expect("parse");
    let model_20 = sim_mjcf::load_model(HILL_PENN_20_MJCF).expect("parse");

    let mut data_0 = model_0.make_data();
    let mut data_20 = model_20.make_data();

    // Static comparison: same position, same activation, different pennation
    data_0.qpos[0] = 1.0;
    data_20.qpos[0] = 1.0;
    data_0.ctrl[0] = 1.0;
    data_20.ctrl[0] = 1.0;
    data_0.act[model_0.actuator_act_adr[0]] = 1.0;
    data_20.act[model_20.actuator_act_adr[0]] = 1.0;
    data_0.forward(&model_0).expect("forward");
    data_20.forward(&model_20).expect("forward");

    let force_0 = data_0.actuator_force[0];
    let force_20 = data_20.actuator_force[0];

    // Pennation reduces force: |force_20| < |force_0|
    // Expected ratio ≈ cos(20°) ≈ 0.9397 (both active and passive scale by cos)
    let ratio = force_20 / force_0;
    let expected_ratio = (20.0_f64.to_radians()).cos();
    let p = check(
        "E2 Pennation reduces force",
        force_0.abs() > force_20.abs() && force_20 < 0.0 && (ratio - expected_ratio).abs() < 0.02,
        &format!(
            "force(penn=0)={force_0:.4}, force(penn=20°)={force_20:.4}, ratio={ratio:.4}, expected≈{expected_ratio:.4}",
        ),
    );
    (u32::from(p), 1)
}

// ── E3: HillMuscle — passive force at stretch ────────────────────────────

fn check_e3_hill_passive() -> (u32, u32) {
    let model = sim_mjcf::load_model(HILL_ZERO_PENN_MJCF).expect("parse");
    let mut data = model.make_data();

    // No activation, stretch joint beyond optimal (norm_len > 1.0)
    // At qpos=1.3: fiber=1.3, norm_len=1.3 → passive force active
    data.ctrl[0] = 0.0;
    data.qpos[0] = 1.3;
    // Activation = 0 (no active force, only passive)
    data.forward(&model).expect("forward");

    let force = data.actuator_force[0];
    let f0 = model.actuator_gainprm[0][2];
    // Expected: bias = -F0 * FP(1.3) * cos(0), FP(1.3) from exponential curve
    let expected_fp = hill_passive_fl(1.3);

    let p = check(
        "E3 Hill passive at stretch",
        force < -0.1,
        &format!("force={force:.6}, F0={f0:.1}, FP(1.3)={expected_fp:.6} (must be negative)"),
    );
    (u32::from(p), 1)
}

// ── E4: HillMuscle — F0 auto-computation ─────────────────────────────────

fn check_e4_hill_f0_auto() -> (u32, u32) {
    let model = sim_mjcf::load_model(HILL_AUTO_F0_MJCF).expect("parse");

    let f0 = model.actuator_gainprm[0][2];
    let acc0 = model.actuator_acc0[0];
    let scale = model.actuator_gainprm[0][3]; // 200

    // F0 = scale / acc0
    let expected = scale / acc0;
    let err = (f0 - expected).abs();

    let mut p = 0u32;
    let t = 2;

    p += u32::from(check(
        "E4a F0 auto-resolved positive",
        f0 > 0.0,
        &format!("F0={f0:.6} (must be positive)"),
    ));

    p += u32::from(check(
        "E4b F0 = scale/acc0",
        err < 1e-6,
        &format!("F0={f0:.6}, scale/acc0={expected:.6}, err={err:.2e}"),
    ));

    (p, t)
}

// ── E5: HillMuscle — activation dynamics identical to Muscle ─────────────

fn check_e5_hill_dynamics_matches_muscle() -> (u32, u32) {
    let model = sim_mjcf::load_model(MIXED_MJCF).expect("parse");
    let mut data = model.make_data();

    // Both get same ctrl
    data.ctrl[0] = 0.7; // MuJoCo muscle
    data.ctrl[1] = 0.7; // HillMuscle

    // Step once to get act_dot populated
    data.step(&model).expect("step");

    let act_dot_mj = data.act_dot[model.actuator_act_adr[0]];
    let act_dot_hill = data.act_dot[model.actuator_act_adr[1]];
    let err = (act_dot_mj - act_dot_hill).abs();

    let p = check(
        "E5 Dynamics: Muscle == HillMuscle",
        err < 1e-15,
        &format!("act_dot_mj={act_dot_mj:.15}, act_dot_hill={act_dot_hill:.15}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group F: Dynamic Behavior
// ============================================================================

// ── F1: Activation rise time constant ─────────────────────────────────────

fn check_f1_activation_rise_time() -> (u32, u32) {
    let model = sim_mjcf::load_model(MUJOCO_MUSCLE_MJCF).expect("parse");
    let mut data = model.make_data();

    data.ctrl[0] = 1.0;
    let act_adr = model.actuator_act_adr[0];

    // Track time to reach 63.2% activation (1 - 1/e)
    let target = 1.0 - (-1.0_f64).exp(); // 0.6321...
    let mut rise_time = 0.0;
    let mut found = false;

    for step in 0..5_000 {
        data.step(&model).expect("step");
        let act = data.act[act_adr];
        if act >= target && !found {
            rise_time = (step + 1) as f64 * model.timestep;
            found = true;
            break;
        }
    }

    // With tau_act=0.01 and activation-dependent scaling, effective tau
    // starts at 0.005 (act=0) and grows to 0.0095 (act=0.3). The rise
    // time to 63.2% should be in the range 5-20ms.
    let p = check(
        "F1 Activation rise time",
        found && rise_time > 0.003 && rise_time < 0.030,
        &format!("rise_to_63%={rise_time:.4}s (expect 5-20ms range)"),
    );
    (u32::from(p), 1)
}

// ── F2: Forearm flexion — muscle lifts load ───────────────────────────────

/// HillMuscle with gravity — moderate F0 for stable dynamics.
const HILL_GRAVITY_MJCF: &str = r#"
<mujoco model="hill-flexion">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="0.5"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <general name="hill_muscle" joint="elbow"
             dyntype="hillmuscle"
             gainprm="0.75 1.05 50 200 1.0 0.0 10.0 0.0 35.0"
             dynprm="0.01 0.04 0.0"/>
  </actuator>
</mujoco>
"#;

fn check_f2_forearm_flexion() -> (u32, u32) {
    let model = sim_mjcf::load_model(HILL_GRAVITY_MJCF).expect("parse");
    let mut data = model.make_data();

    // Start at optimal fiber length (qpos=1.0), ramp ctrl to 1.
    data.qpos[0] = 1.0;
    data.ctrl[0] = 1.0;

    // Step a few times to let activation build, then record force
    let mut max_force_mag = 0.0_f64;
    for _ in 0..2_000 {
        data.step(&model).expect("step");
        max_force_mag = max_force_mag.max(data.actuator_force[0].abs());
    }

    let angle = data.qpos[0];

    let mut p = 0u32;
    let t = 2;

    p += u32::from(check(
        "F2a Forearm moved",
        (angle - 1.0).abs() > 0.01,
        &format!("angle={angle:.4} rad, initial=1.0 (must have moved)"),
    ));

    p += u32::from(check(
        "F2b Muscle produced force",
        max_force_mag > 1.0,
        &format!("max |force|={max_force_mag:.4} (must be substantial)"),
    ));

    (p, t)
}

// ── F3: Cocontraction — opposing muscles ──────────────────────────────────

/// Two HillMuscle actuators pulling toward opposite sides.
/// Agonist: gear=1, L_slack=0 → optimal at qpos=1.0 (pulls negative)
/// Antagonist: gear=1, L_slack=1.0 → fiber=(length-1.0), optimal at qpos=2.0
///   At qpos=1.0: antagonist fiber=0, norm_len=0 → FL≈0, minimal force
///   But at qpos=0.5: agonist norm_len=0.5 (boundary), antagonist norm_len=-0.5 (zero)
/// Better approach: use MuJoCo `<muscle>` for cocontraction since it handles
/// length normalization properly via lengthrange.
const COCONTRACTION_MJCF: &str = r#"
<mujoco model="cocontraction">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true" damping="0.5"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <muscle name="agonist" joint="elbow" gear="1" force="50"
            range="0.75 1.05" lmin="0.5" lmax="1.6"
            vmax="1.0" fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
    <muscle name="antagonist" joint="elbow" gear="-1" force="50"
            range="0.75 1.05" lmin="0.5" lmax="1.6"
            vmax="1.0" fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
  </actuator>
</mujoco>
"#;

fn check_f3_cocontraction() -> (u32, u32) {
    let model = sim_mjcf::load_model(COCONTRACTION_MJCF).expect("parse");
    let mut data = model.make_data();

    // Both muscles fully activated. MuJoCo <muscle> with opposite gear signs.
    data.ctrl[0] = 1.0;
    data.ctrl[1] = 1.0;

    // Step to let activation build, track max forces
    let mut max_ago = 0.0_f64;
    let mut max_ant = 0.0_f64;
    for _ in 0..2_000 {
        data.step(&model).expect("step");
        max_ago = max_ago.max(data.actuator_force[0].abs());
        max_ant = max_ant.max(data.actuator_force[1].abs());
    }
    let angle = data.qpos[0];

    let mut p = 0u32;
    let t = 3;

    // Joint should stay near initial position (balanced opposing forces)
    p += u32::from(check(
        "F3a Joint near equilibrium",
        angle.abs() < 0.5,
        &format!("angle={angle:.4} rad, initial=0.0 (must stay near start)"),
    ));

    p += u32::from(check(
        "F3b Agonist active",
        max_ago > 1.0,
        &format!("max |agonist_force|={max_ago:.4} (must be nonzero)"),
    ));

    p += u32::from(check(
        "F3c Antagonist active",
        max_ant > 1.0,
        &format!("max |antagonist_force|={max_ant:.4} (must be nonzero)"),
    ));

    (p, t)
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    println!("=== Muscle System — Stress Test ===\n");

    let mut total = 0u32;
    let mut passed = 0u32;

    // ── Group A: Activation Dynamics ──
    println!("-- A1. Activation asymmetry --");
    let (p, t) = check_a1_activation_asymmetry();
    passed += p;
    total += t;

    println!("\n-- A2. Hard-switch activation --");
    let (p, t) = check_a2_hard_switch_activation();
    passed += p;
    total += t;

    println!("\n-- A3. Hard-switch deactivation --");
    let (p, t) = check_a3_hard_switch_deactivation();
    passed += p;
    total += t;

    println!("\n-- A4. Boundary act=0 --");
    let (p, t) = check_a4_boundary_act_zero();
    passed += p;
    total += t;

    println!("\n-- A5. Boundary act=1 --");
    let (p, t) = check_a5_boundary_act_one();
    passed += p;
    total += t;

    println!("\n-- A6. Ctrl clamping --");
    let (p, t) = check_a6_ctrl_clamping();
    passed += p;
    total += t;

    println!("\n-- A7. Smooth blend --");
    let (p, t) = check_a7_smooth_blend();
    passed += p;
    total += t;

    // ── Group B: MuJoCo Curve Pinning ──
    println!("\n-- B1. MuJoCo FL pinned values --");
    let (p, t) = check_b1_mujoco_fl_pinned();
    passed += p;
    total += t;

    println!("\n-- B2. MuJoCo FL outside range --");
    let (p, t) = check_b2_mujoco_fl_outside();
    passed += p;
    total += t;

    println!("\n-- B3. MuJoCo FV pinned values --");
    let (p, t) = check_b3_mujoco_fv_pinned();
    passed += p;
    total += t;

    println!("\n-- B4. MuJoCo FP pinned values --");
    let (p, t) = check_b4_mujoco_fp_pinned();
    passed += p;
    total += t;

    // ── Group C: Hill Curve Pinning ──
    println!("\n-- C1. Hill FL pinned values --");
    let (p, t) = check_c1_hill_fl_pinned();
    passed += p;
    total += t;

    println!("\n-- C2. Hill FV pinned values --");
    let (p, t) = check_c2_hill_fv_pinned();
    passed += p;
    total += t;

    println!("\n-- C3. Hill FP pinned values --");
    let (p, t) = check_c3_hill_fp_pinned();
    passed += p;
    total += t;

    // ── Group D: MuJoCo Full Pipeline ──
    println!("\n-- D1. MuJoCo isometric force --");
    let (p, t) = check_d1_mujoco_isometric();
    passed += p;
    total += t;

    println!("\n-- D2. MuJoCo passive at stretch --");
    let (p, t) = check_d2_mujoco_passive();
    passed += p;
    total += t;

    // ── Group E: HillMuscle Full Pipeline ──
    println!("\n-- E1. Hill isometric (penn=0) --");
    let (p, t) = check_e1_hill_isometric_zero_penn();
    passed += p;
    total += t;

    println!("\n-- E2. Pennation reduces force --");
    let (p, t) = check_e2_hill_pennation();
    passed += p;
    total += t;

    println!("\n-- E3. Hill passive at stretch --");
    let (p, t) = check_e3_hill_passive();
    passed += p;
    total += t;

    println!("\n-- E4. Hill F0 auto-computation --");
    let (p, t) = check_e4_hill_f0_auto();
    passed += p;
    total += t;

    println!("\n-- E5. Dynamics: Muscle == HillMuscle --");
    let (p, t) = check_e5_hill_dynamics_matches_muscle();
    passed += p;
    total += t;

    // ── Group F: Dynamic Behavior ──
    println!("\n-- F1. Activation rise time --");
    let (p, t) = check_f1_activation_rise_time();
    passed += p;
    total += t;

    println!("\n-- F2. Forearm flexion --");
    let (p, t) = check_f2_forearm_flexion();
    passed += p;
    total += t;

    println!("\n-- F3. Cocontraction --");
    let (p, t) = check_f3_cocontraction();
    passed += p;
    total += t;

    // ── Summary ──
    println!("\n============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS — muscle system verified");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
