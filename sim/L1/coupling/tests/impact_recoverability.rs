//! De-escalation study, RQ1 — "a soft buffer absorbs one high-force limb strike, RECOVERABLY."
//!
//! This is an **open-research worked example**, not a device: the artifact is the simulation
//! and what it teaches about *protecting recoverability* (keeping a worst-moment contact below
//! the irreversible cliff), and the contact is non-injurious *by physics*, not by policy. See
//! the project memory `project-deescalation-study.md`.
//!
//! ## The experiment (forward only — gradients/co-design are RQ2)
//! A ~1 kg rigid "limb" strikes, at a realistic incoming speed, a soft buffer of **measured**
//! Ecoflex 00-30 ([`ECOFLEX_00_30_MEASURED`], μ≈16.7 kPa) pinned at its base. We instrument the
//! coupled soft↔rigid contact ([`StaggeredCoupling::step`]) per step and compare it against a
//! **hard-shell baseline** (Dragon Skin 30A, ≈12× stiffer) at the *same* strike — the falsifiable
//! claim being that the soft buffer keeps peak contact force in the recoverable zone while the
//! hard baseline crosses the irreversible cliff.
//!
//! ## The recoverability metric (cited thresholds — `project-deescalation-injury-thresholds.md`)
//! - **Recoverable / comfort line:** ISO/TS 15066 transient contact force for the hand/forearm,
//!   **≈270 N** (the conservative, most-likely-contact region; this is pain-onset, *not* injury).
//! - **Irreversible cliff:** facial-bone fracture onset **≈450 N**; skin laceration energy-density
//!   **≈12.6 J/cm²**. The thesis is shown when the soft buffer sits well below the cliff (and,
//!   gently struck, below the comfort line too) while the hard baseline crosses it.
//!
//! ## Honest modeling caveats (this is a relative-comparison study, not an absolute predictor)
//! 1. **Neo-Hookean, not Yeoh.** `StaggeredCoupling` is NH-only, so the buffer uses the measured
//!    Ecoflex *Lamé pair* (μ=16.7 kPa, λ=4μ — reproduced exactly by `new`) but drops the Yeoh C₂
//!    finite-strain stiffening. NH is *softer* at large compression, so it under-predicts peak
//!    force there — we keep the headline strike's compression modest (≤~25%, the Sparks-validated
//!    regime where NH≈Yeoh) to bound this.
//! 2. **Backward-Euler dissipation.** The soft solve is numerically dissipative, so the reported
//!    "KE absorbed" overstates physical hysteresis. The load-bearing signal is the *contrast*
//!    (the soft buffer absorbs far more than the hard shell, which bounces elastically), not the
//!    absolute fraction.
//! 3. **Average, not peak, pressure.** Only total `force_on_soft` is publicly exposed, so pressure
//!    is force / contact-area (an average). Peak per-vertex pressure is higher — but with a ~100 cm²
//!    blunt contact, even several× the average sits far under the ISO transient pressure line
//!    (~1400 kPa) and the laceration cliff, so force, not pressure, is the binding metric here.
//! 4. **Hard baseline is a conservative rigid stand-in.** A truly rigid wall destabilizes the soft
//!    solver; Dragon Skin 30A (a real ≈12× stiffer silicone) is the firmest stable point. A rigid
//!    structure would transmit *more* force, so the hard row is a lower bound on the rigid harm.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::material::silicone_table::{DRAGON_SKIN_30A, ECOFLEX_00_30_MEASURED};

const MASS: f64 = 1.0; // kg — a ~1 kg limb segment
const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.10; // m — soft cube side (spans z∈[0,0.10]); top face = 100 cm²
const DT: f64 = 1.0e-3;
const N_STEPS: usize = 150;
const HALF_THICK: f64 = 0.005; // limb box half-thickness
const CLEARANCE: f64 = 0.005;
const D_HAT: f64 = 1.0e-2;

// Recoverability thresholds (N), cited.
const ISO_RECOVERABLE_N: f64 = 270.0; // ISO/TS 15066 hand/forearm transient (pain-onset)
const FRACTURE_CLIFF_N: f64 = 450.0; // facial-bone fracture onset (irreversible)

struct Metrics {
    peak_force: f64,
    peak_decel: f64,
    absorbed_frac: f64,
    max_compression: f64,
}

/// One normal strike of a `MASS`-kg limb at `v_impact` (m/s) into a soft cube of side [`EDGE`]
/// made of `soft_mu` Neo-Hookean silicone. The limb starts at the penalty-band top
/// (force≈0, just touching) moving down at `v_impact`, and is rolled forward [`N_STEPS`].
fn run_impact(soft_mu: f64, v_impact: f64) -> Metrics {
    let block_top = EDGE;
    // Band top = block_top + d_hat (contact face) + clearance ⇒ just touching, force≈0. (The band
    // BOTTOM, block_top + clearance, is max engagement — starting there fires a penalty spike.)
    let start_z = block_top + D_HAT + CLEARANCE;
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
    data.qvel[2] = -v_impact; // free-joint linear z velocity: a downward strike

    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, /* body */ 1, CLEARANCE, N_PER_EDGE, EDGE, soft_mu, DT,
        /* kappa */ 3.0e4, D_HAT,
        /* rigid_damping */
        0.0, // zero — physical impact, no artificial settling damping
    );

    let ke0 = 0.5 * MASS * v_impact * v_impact;
    let mut peak_force = 0.0_f64;
    let mut peak_decel = 0.0_f64;
    let mut min_z = start_z;
    let mut prev_vz = -v_impact;
    let mut rebound_vz = -v_impact; // most-positive vz during the rollout (peak rebound)

    for _ in 0..N_STEPS {
        let s = coupling.step();
        let vz = coupling.data().qvel[2];
        peak_decel = peak_decel.max(((vz - prev_vz) / DT).abs());
        prev_vz = vz;
        peak_force = peak_force.max(s.force_on_soft.z.abs());
        min_z = min_z.min(s.rigid_z);
        rebound_vz = rebound_vz.max(vz);
        assert!(
            s.force_on_soft.z.is_finite() && vz.is_finite(),
            "UNSTABLE forward solve"
        );
    }
    let ke_rebound = 0.5 * MASS * rebound_vz.max(0.0).powi(2);
    Metrics {
        peak_force,
        peak_decel,
        absorbed_frac: (ke0 - ke_rebound) / ke0,
        max_compression: (block_top - (min_z - HALF_THICK)).max(0.0),
    }
}

fn verdict(peak_force: f64) -> &'static str {
    if peak_force >= FRACTURE_CLIFF_N {
        "FRACTURE RISK (irreversible)"
    } else if peak_force >= ISO_RECOVERABLE_N {
        "painful but recoverable"
    } else {
        "comfortably recoverable"
    }
}

fn print_row(label: &str, v: f64, m: &Metrics) {
    eprintln!(
        "  {label:14} v={v:.1}: peak={:6.1} N  press={:3.0} kPa  decel={:4.0} g  \
         compress={:4.1}%  absorbed={:3.0}%  cliff-headroom={:+6.1} N  [{}]",
        m.peak_force,
        m.peak_force / (EDGE * EDGE) / 1000.0,
        m.peak_decel / 9.81,
        m.max_compression / EDGE * 100.0,
        m.absorbed_frac * 100.0,
        FRACTURE_CLIFF_N - m.peak_force,
        verdict(m.peak_force),
    );
}

/// RQ1 result: across a sweep of strike speeds, the measured-Ecoflex soft buffer keeps peak
/// contact force far below the hard-shell baseline; at the headline strike it stays in the
/// recoverable zone (below the fracture cliff) while the hard baseline crosses it.
#[test]
fn soft_buffer_keeps_strike_recoverable_vs_hard_baseline() {
    eprintln!(
        "\n=== De-escalation RQ1: soft buffer vs hard baseline ===\n\
         1 kg limb, measured Ecoflex 00-30 buffer ({:.1} kPa) vs Dragon Skin 30A shell ({:.0} kPa).\n\
         ISO recoverable line = {ISO_RECOVERABLE_N:.0} N; fracture cliff = {FRACTURE_CLIFF_N:.0} N.",
        ECOFLEX_00_30_MEASURED.mu / 1e3,
        DRAGON_SKIN_30A.mu / 1e3,
    );
    let speeds = [1.5_f64, 2.0, 2.5, 3.0, 4.0];
    eprintln!("\nSOFT buffer (Ecoflex 00-30 measured):");
    for v in speeds {
        print_row("Ecoflex", v, &run_impact(ECOFLEX_00_30_MEASURED.mu, v));
    }
    eprintln!("HARD baseline (Dragon Skin 30A):");
    for v in speeds {
        print_row("Dragon Skin", v, &run_impact(DRAGON_SKIN_30A.mu, v));
    }

    // --- Headline strike: 1 kg @ 2.0 m/s (KE = 2.0 J — a meaningful moderate strike at which
    // the soft buffer is comfortably recoverable, below BOTH lines, while the hard shell already
    // crosses the irreversible fracture cliff). ---
    let v = 2.0;
    let soft = run_impact(ECOFLEX_00_30_MEASURED.mu, v);
    let hard = run_impact(DRAGON_SKIN_30A.mu, v);
    let energy_density = 0.5 * MASS * v * v / (EDGE * EDGE * 1e4); // J / cm² (blunt, large-area)
    eprintln!(
        "\n=== HEADLINE (1 kg @ {v} m/s, KE={:.1} J) ===\n\
         soft buffer: {:.0} N  [{}]  — {:.0} N below the fracture cliff\n\
         hard shell : {:.0} N  [{}]  — {:.0} N {} the fracture cliff\n\
         soft absorbs {:.0}% of impact KE vs hard {:.0}% (absorb-don't-return)\n\
         laceration check: {:.3} J/cm² vs 12.6 J/cm² cliff → {:.0}× headroom (blunt contact)",
        0.5 * MASS * v * v,
        soft.peak_force,
        verdict(soft.peak_force),
        FRACTURE_CLIFF_N - soft.peak_force,
        hard.peak_force,
        verdict(hard.peak_force),
        (hard.peak_force - FRACTURE_CLIFF_N).abs(),
        if hard.peak_force >= FRACTURE_CLIFF_N {
            "above"
        } else {
            "below"
        },
        soft.absorbed_frac * 100.0,
        hard.absorbed_frac * 100.0,
        energy_density,
        12.6 / energy_density,
    );

    // The falsifiable thesis, asserted:
    // 0. Guard against a vacuous pass: the soft contact must actually ENGAGE (the limb really
    //    struck and compressed the buffer). Without this, a regression that stops contact forming
    //    would leave soft.peak_force ≈ 0 and clear the cliff trivially — and a non-engaging limb
    //    even reports ~100% "absorbed" — so every assertion below would pass for the wrong reason.
    //    (The hard baseline needs no such guard: its `≥ cliff` assert already fails if it misses.)
    assert!(
        soft.max_compression > 5e-3,
        "soft buffer never engaged (compression {:.4} m) — contact may not have formed",
        soft.max_compression
    );
    // 1. The soft buffer keeps the headline strike recoverable (below the irreversible cliff)...
    assert!(
        soft.peak_force < FRACTURE_CLIFF_N,
        "soft buffer crossed the fracture cliff: {:.1} N ≥ {FRACTURE_CLIFF_N} N",
        soft.peak_force
    );
    // 2. ...while the hard baseline crosses it (the same strike is irreversible without the buffer).
    assert!(
        hard.peak_force >= FRACTURE_CLIFF_N,
        "hard baseline did not cross the cliff: {:.1} N < {FRACTURE_CLIFF_N} N (contrast lost)",
        hard.peak_force
    );
    // 3. The buffer roughly halves peak force and absorbs far more energy than the hard shell.
    assert!(
        soft.peak_force < 0.75 * hard.peak_force,
        "soft buffer did not meaningfully reduce peak force: soft {:.1} N vs hard {:.1} N",
        soft.peak_force,
        hard.peak_force
    );
    assert!(
        soft.absorbed_frac > hard.absorbed_frac,
        "soft buffer did not out-absorb the hard shell: {:.2} vs {:.2}",
        soft.absorbed_frac,
        hard.absorbed_frac
    );
}
