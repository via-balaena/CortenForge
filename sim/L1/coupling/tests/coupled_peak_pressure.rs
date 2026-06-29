//! De-escalation pressure story, measured — the contrast total force cannot see.
//!
//! Closes caveat #3 of the RQ1 impact study (`impact_recoverability.rs`): "only total
//! `force_on_soft` is publicly exposed, so pressure is force / contact-area (an average)."
//! [`StaggeredCoupling::coupled_trajectory_peak_pressure`] now measures the PEAK per-face
//! contact pressure over an impact rollout (built on the per-pair
//! [`sim_soft::ContactPairReadout::pressure`] + the [`sim_soft::peak_contact_pressure`]
//! reduction).
//!
//! The falsifiable claim: the SAME 1 kg striker, into the SAME soft buffer, through a
//! **concentrated** finite collider (a sphere) vs a **broad** one (the half-space slab),
//! produces opposite readouts — the sphere reads a HIGHER peak pressure at a LOWER total
//! force (the load is squeezed onto a small patch), the slab the reverse. That is exactly
//! the distinction a force-only readout misses, and why pressure is the binding metric for
//! a concentrated contact.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED;

const MASS: f64 = 1.0; // kg — a ~1 kg limb segment
const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.10; // m — soft cube side (spans z∈[0,0.10])
const DT: f64 = 1.0e-3;
const N_STEPS: usize = 80;
const CLEARANCE: f64 = 0.005;
const D_HAT: f64 = 1.0e-2;
const KAPPA: f64 = 3.0e4;

/// One gentle strike of a `MASS`-kg limb at `v_impact` (m/s) into the measured-Ecoflex
/// buffer, contacting through either the broad half-space slab (`sphere_radius = None`) or
/// a concentrated finite sphere (`Some(r)`). Returns `(peak_pressure_Pa, peak_total_force_N)`.
fn run(sphere_radius: Option<f64>, v_impact: f64) -> (f64, f64) {
    let block_top = EDGE;
    let start_z = block_top + D_HAT + CLEARANCE; // just touching, force ≈ 0
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="limb" pos="0 0 {start_z}">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="{MASS}"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("limb MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    data.qvel[2] = -v_impact; // downward strike

    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model,
        data,
        1,
        CLEARANCE,
        N_PER_EDGE,
        EDGE,
        ECOFLEX_00_30_MEASURED.mu,
        DT,
        KAPPA,
        D_HAT,
        0.0,
    );
    if let Some(r) = sphere_radius {
        coupling = coupling.with_sphere_collider(r);
    }
    let peak = coupling.coupled_trajectory_peak_pressure(N_STEPS);
    (peak.peak_pressure, peak.peak_total_force)
}

#[test]
fn sphere_reads_higher_pressure_lower_force_than_slab() {
    let v = 1.0;
    let (slab_p, slab_f) = run(None, v);
    let (sphere_p, sphere_f) = run(Some(0.04), v);

    eprintln!(
        "\n=== Measured pressure-vs-force contrast (1 kg @ {v} m/s, measured Ecoflex) ===\n\
         broad SLAB  : peak pressure {:8.1} kPa   peak total force {:7.1} N\n\
         concentrated SPHERE: peak pressure {:8.1} kPa   peak total force {:7.1} N",
        slab_p / 1e3,
        slab_f,
        sphere_p / 1e3,
        sphere_f,
    );

    // Both must actually engage (finite, non-trivial pressure) — else the contrast is vacuous.
    assert!(
        slab_p.is_finite() && slab_p > 0.0 && sphere_p.is_finite() && sphere_p > 0.0,
        "both contacts must register finite pressure (slab {slab_p}, sphere {sphere_p})",
    );
    // The honest pressure-vs-force story: the sphere concentrates the load onto a small
    // patch → HIGHER peak pressure but LOWER total force than the broad slab.
    assert!(
        sphere_p > slab_p,
        "concentrated sphere should read higher peak pressure than the broad slab: \
         sphere {sphere_p:.1} Pa vs slab {slab_p:.1} Pa",
    );
    assert!(
        sphere_f < slab_f,
        "concentrated sphere should read lower total force than the broad slab: \
         sphere {sphere_f:.1} N vs slab {slab_f:.1} N",
    );
}
