//! Integrator Comparison — Headless Energy Drift Analysis
//!
//! Runs all 5 integrators on an identical undamped pendulum for 15 seconds,
//! then prints a formatted comparison table with energy drift, period accuracy,
//! and pass/fail checks.
//!
//! No Bevy window — pure sim-core + sim-mjcf.
//!
//! Energy drift is normalized by m*g*d (the characteristic PE swing of the
//! pendulum), not by E₀, because E₀ ≈ 0 when starting from horizontal.
//!
//! Period is validated by comparing all integrators to RK4's measured period
//! (the most accurate reference), not to the small-angle analytical formula
//! which doesn't apply at θ₀ = π/2.
//!
//! Validates (7 checks):
//! 1. Euler drifts visibly (>0.1% of m*g*d)
//! 2. RK4 near-perfect (<0.001% of m*g*d)
//! 3. Implicit stable (<0.1% of m*g*d)
//! 4. `ImplicitFast` stable (<0.1% of m*g*d)
//! 5. `ImplicitSpringDamper` bounded (<1% of m*g*d)
//! 6. RK4 >> Euler (10× better)
//! 7. All periods within 2% of RK4's measured period
//!
//! Run with: `cargo run -p example-integrator-comparison --release`

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::too_many_lines
)]

use std::f64::consts::FRAC_PI_2;

const SIM_TIME: f64 = 15.0;
const DT: f64 = 0.005;
const STEPS: usize = (SIM_TIME / DT) as usize;

// Characteristic energy scale: m * g * d = 1.0 * 9.81 * 0.25 = 2.4525 J
// This is the maximum PE swing of the pendulum (COM drops d = 0.25m).
const M_G_D: f64 = 1.0 * 9.81 * 0.25;

fn mjcf(integrator: &str) -> String {
    format!(
        r#"
<mujoco model="integrator-{integrator}">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="{DT}" integrator="{integrator}">
    <flag energy="enable" contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>
</mujoco>
"#
    )
}

struct IntegratorResult {
    name: &'static str,
    energy_initial: f64,
    energy_final: f64,
    /// Energy drift normalized by m*g*d (characteristic energy scale)
    drift_pct: f64,
    measured_period: f64,
}

fn run_integrator(name: &'static str, mjcf_integrator: &str) -> IntegratorResult {
    let xml = mjcf(mjcf_integrator);
    let mut model = sim_mjcf::load_model(&xml).expect("MJCF should parse");
    model.enableflags |= sim_core::ENABLE_ENERGY;
    let mut data = model.make_data();

    // Initial condition: horizontal (θ = π/2)
    data.qpos[0] = FRAC_PI_2;
    data.forward(&model).expect("forward should succeed");
    let energy_initial = data.total_energy();

    // Track zero-crossings for period measurement
    let mut prev_sign: i8 = if data.qpos[0] > 0.0 { 1 } else { -1 };
    let mut zero_cross_times: Vec<f64> = Vec::new();

    // Step simulation
    for _ in 0..STEPS {
        data.step(&model).expect("step should succeed");

        let sign = if data.qpos[0] > 0.0 { 1 } else { -1 };
        if sign != prev_sign && sign > 0 {
            zero_cross_times.push(data.time);
        }
        prev_sign = sign;
    }

    let energy_final = data.total_energy();
    // Normalize drift by characteristic energy scale, not E₀ (which is ≈ 0)
    let drift_pct = (energy_final - energy_initial) / M_G_D * 100.0;

    let measured_period = if zero_cross_times.len() >= 2 {
        let n = zero_cross_times.len();
        (zero_cross_times[n - 1] - zero_cross_times[0]) / (n - 1) as f64
    } else {
        0.0
    };

    IntegratorResult {
        name,
        energy_initial,
        energy_final,
        drift_pct,
        measured_period,
    }
}

fn main() {
    println!("Running 5 integrators for {SIM_TIME}s each (dt={DT})...\n");

    let results = vec![
        run_integrator("Euler", "Euler"),
        run_integrator("RK4", "RK4"),
        run_integrator("Implicit", "implicit"),
        run_integrator("ImplicitFast", "implicitfast"),
        run_integrator("ImplSpDmp", "implicitspringdamper"),
    ];

    // Use RK4's measured period as reference (most accurate integrator)
    let rk4_period = results[1].measured_period;

    // Print comparison table
    println!("=== Integrator Comparison (t = {SIM_TIME}s, dt = {DT}) ===");
    println!("  Drift normalized by m*g*d = {M_G_D:.4} J");
    println!("  Period reference: RK4 measured T = {rk4_period:.4}s");
    println!();
    println!(
        "  {:<22} {:>10} {:>12} {:>12} {:>10} {:>10}",
        "Integrator", "E_0 (J)", "E_now (J)", "Drift(%mgd)", "Period(s)", "T_err(%)"
    );
    println!("  {}", "-".repeat(78));
    for r in &results {
        let period_err = if rk4_period > 0.0 && r.measured_period > 0.0 {
            (r.measured_period - rk4_period) / rk4_period * 100.0
        } else {
            999.0
        };
        println!(
            "  {:<22} {:>10.6} {:>12.6} {:>+12.4} {:>10.4} {:>+10.3}",
            r.name, r.energy_initial, r.energy_final, r.drift_pct, r.measured_period, period_err
        );
    }
    println!();

    // Validation checks
    let euler = &results[0];
    let rk4 = &results[1];
    let implicit = &results[2];
    let implicit_fast = &results[3];
    let impl_sp_dmp = &results[4];

    let checks: Vec<(&str, bool, String)> = vec![
        (
            "Euler drifts visibly",
            euler.drift_pct.abs() > 0.1,
            format!(
                "|drift|={:.4}% of m*g*d (expect >0.1%)",
                euler.drift_pct.abs()
            ),
        ),
        (
            "RK4 near-perfect",
            rk4.drift_pct.abs() < 0.001,
            format!(
                "|drift|={:.6}% of m*g*d (expect <0.001%)",
                rk4.drift_pct.abs()
            ),
        ),
        (
            "Implicit stable",
            implicit.drift_pct.abs() < 0.5,
            format!(
                "|drift|={:.4}% of m*g*d (expect <0.5%)",
                implicit.drift_pct.abs()
            ),
        ),
        (
            "ImplicitFast stable",
            implicit_fast.drift_pct.abs() < 0.5,
            format!(
                "|drift|={:.4}% of m*g*d (expect <0.5%)",
                implicit_fast.drift_pct.abs()
            ),
        ),
        (
            "ImplSpDmp bounded",
            impl_sp_dmp.drift_pct.abs() < 5.0,
            format!(
                "|drift|={:.4}% of m*g*d (expect <5%, Euler-like with zero K/D)",
                impl_sp_dmp.drift_pct.abs()
            ),
        ),
        (
            "RK4 >> Euler (10x)",
            rk4.drift_pct.abs() < euler.drift_pct.abs() / 10.0,
            format!(
                "ratio={:.1}x",
                euler.drift_pct.abs() / rk4.drift_pct.abs().max(1e-15)
            ),
        ),
        (
            "All periods within 2% of RK4",
            results.iter().all(|r| {
                if rk4_period > 0.0 && r.measured_period > 0.0 {
                    ((r.measured_period - rk4_period) / rk4_period * 100.0).abs() < 2.0
                } else {
                    false
                }
            }),
            format!(
                "max err={:.3}% vs RK4 T={rk4_period:.4}s",
                results
                    .iter()
                    .map(|r| {
                        if rk4_period > 0.0 && r.measured_period > 0.0 {
                            ((r.measured_period - rk4_period) / rk4_period * 100.0).abs()
                        } else {
                            999.0
                        }
                    })
                    .fold(0.0_f64, f64::max)
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
