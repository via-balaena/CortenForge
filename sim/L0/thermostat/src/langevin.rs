//! `LangevinThermostat` — explicit Langevin thermostat via Euler-Maruyama.
//!
//! Per chassis design + spec §3, the thermostat writes the
//! fluctuation–dissipation pair `(−γ·v, σ·z)` into the per-DOF
//! accumulator on every step:
//!
//! ```text
//! qfrc_out[i] += −γ_i · qvel[i]  +  sqrt(2 · γ_i · k_B·T / h) · z_i,    z_i ~ N(0, 1)
//!                └── damping ──┘   └────── FDT-paired noise ──────┘
//! ```
//!
//! `γ_i` and `k_B·T` are owned by this struct — `model.dof_damping`
//! stays at zero (Q4 resolution, recon log part 2). The fluctuation–
//! dissipation relation `σ² = 2γkT/h` is the only physics statement
//! the implementation makes; everything else is bookkeeping.
//!
//! The discretization-bias temperature error is `O(h·γ/M)`. At the
//! Phase 1 central parameter set (`h=0.001`, `γ=0.1`, `M=1`) that is
//! ≈ `10⁻⁴` of `½kT` — well below the §7 sampling-error tolerance of
//! 4.5%. The gate passes with margin, not at threshold.
//!
//! Higher-order schemes (BAOAB, GJF) reduce this further but are not
//! needed for Phase 1; the upgrade path is to swap the
//! `PassiveComponent` impl without touching the chassis.
//!
//! ## RNG and `cb_passive`
//!
//! The struct holds an owned `ChaCha8Rng` behind a `std::sync::Mutex`
//! because `cb_passive` is `Fn` (not `FnMut`), so the closure can
//! only call `&self` methods on the components it captures and any
//! mutable state must use interior mutability. `ChaCha8Rng` is the
//! chassis-mandated PRNG choice from recon log part 6 (Scheme B):
//! bit-stable across `rand`/`rand_chacha` versions, no silent
//! reproducibility hazard.
//!
//! Mutex poisoning is structurally impossible in this codebase
//! because the only thread that ever holds the lock is the
//! single-threaded simulation step's `cb_passive` callback, and the
//! only operation under the lock is `StandardNormal.sample`, which
//! does not panic on any input. The `unwrap_or_else(PoisonError::into_inner)`
//! recovery in `apply` is the std idiom for "poison cannot occur, but
//! if it somehow did, the inner state is still valid — keep going."

use std::sync::Mutex;
use std::sync::PoisonError;
use std::sync::atomic::{AtomicBool, Ordering};

use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, StandardNormal};
use sim_core::{DVector, Data, Model};

use crate::component::{PassiveComponent, Stochastic};
use crate::diagnose::Diagnose;

/// Explicit Langevin thermostat implementing the
/// fluctuation–dissipation pair `(−γ·v, σ·z)` via Euler-Maruyama.
///
/// Construct via [`LangevinThermostat::new`], add to a stack via
/// `PassiveStack::builder().with(thermostat).build()`, install onto
/// a `Model` via `stack.install(&mut model)`, and step the
/// simulation normally with `data.step(&model)?`.
///
/// Implements three traits from the chassis surface:
/// - [`PassiveComponent`] (the M5 contract — the apply method that
///   writes forces into `qfrc_out`).
/// - [`Stochastic`] (Decision 7 — the gating opt-in that lets
///   `PassiveStack::disable_stochastic` zero the noise contribution
///   for finite-difference and autograd contexts).
/// - [`Diagnose`] (Decision 4 — the minimal one-line introspection
///   trait for debugging and test failure messages).
pub struct LangevinThermostat {
    gamma: DVector<f64>,
    k_b_t: f64,
    seed: u64,
    rng: Mutex<ChaCha8Rng>,
    stochastic_active: AtomicBool,
}

impl LangevinThermostat {
    /// Construct a thermostat with per-DOF damping coefficients
    /// `gamma`, bath temperature `k_b_t`, and PRNG `seed`.
    ///
    /// `gamma.len()` must match the simulation's `model.nv` at
    /// install time; the apply method indexes `qvel` and the seeded
    /// PRNG draws by `i in 0..gamma.len()`. Mismatches between
    /// `gamma.len()` and `model.nv` are not detected at construction
    /// time (the thermostat doesn't see the model until install), so
    /// the caller is responsible for matching them. Phase 1's
    /// integration test fixture (spec §6) verifies this against the
    /// loaded MJCF.
    ///
    /// `seed` is retained on the struct for `diagnostic_summary` but
    /// is otherwise consumed only at construction time — the
    /// underlying `ChaCha8Rng` advances with each `apply` call and
    /// the seed value itself is never re-read at apply-time.
    #[must_use]
    pub fn new(gamma: DVector<f64>, k_b_t: f64, seed: u64) -> Self {
        Self {
            gamma,
            k_b_t,
            seed,
            rng: Mutex::new(ChaCha8Rng::seed_from_u64(seed)),
            stochastic_active: AtomicBool::new(true),
        }
    }
}

impl PassiveComponent for LangevinThermostat {
    // The MutexGuard is intentionally held for the entire noise loop:
    // we want exactly one lock acquisition per apply call, not one
    // per DOF. cb_passive is single-threaded so the lock is always
    // uncontended, and re-locking per DOF would be slower than the
    // current pattern. clippy::significant_drop_tightening assumes a
    // contention regime that does not apply here.
    #[allow(clippy::significant_drop_tightening)]
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        // Read h fresh per step (recon log part 8): cost is one f64
        // load + one sqrt per DOF, robust to mid-simulation timestep
        // mutation.
        let h = model.timestep;
        let active = self.stochastic_active.load(Ordering::Relaxed);
        let n_dofs = self.gamma.len();

        // Damping (-γ·v) is unconditional — it is the deterministic
        // half of the FD pair and runs in both stochastic-active and
        // stochastic-inactive states. Per Decision 7 §spec 4.2:
        // "When inactive, apply produces only the deterministic
        // part (-γ·v for the Langevin thermostat)."
        for i in 0..n_dofs {
            qfrc_out[i] += -self.gamma[i] * data.qvel[i];
        }

        if !active {
            // Stochastic disabled — do not advance the RNG. This is
            // load-bearing for the Phase 5 finite-difference path:
            // two FD evaluations under disable_stochastic must
            // produce identical deterministic forces, which means
            // the RNG must not advance between them.
            return;
        }

        // Lock the RNG once for the whole apply call. cb_passive is
        // single-threaded so contention does not exist; install_per_env
        // gives each parallel env its own thermostat instance per
        // chassis Decision 3 so cross-env contention does not exist
        // either. Mutex poisoning is structurally impossible (see
        // module-level doc) — unwrap_or_else(PoisonError::into_inner)
        // is the std idiom for "recover the inner value if poisoned,
        // proceed anyway."
        let mut rng = self.rng.lock().unwrap_or_else(PoisonError::into_inner);

        for i in 0..n_dofs {
            let gamma_i = self.gamma[i];
            // σ² = 2·γ·k_B·T / h. Sqrt is per-DOF because gamma can
            // vary; for the Phase 1 1-DOF case it computes once.
            let sigma = (2.0 * gamma_i * self.k_b_t / h).sqrt();
            let z: f64 = StandardNormal.sample(&mut *rng);
            qfrc_out[i] += sigma * z;
        }
    }

    fn as_stochastic(&self) -> Option<&dyn Stochastic> {
        Some(self)
    }
}

impl Stochastic for LangevinThermostat {
    fn set_stochastic_active(&self, active: bool) {
        self.stochastic_active.store(active, Ordering::Relaxed);
    }

    fn is_stochastic_active(&self) -> bool {
        self.stochastic_active.load(Ordering::Relaxed)
    }
}

impl Diagnose for LangevinThermostat {
    fn diagnostic_summary(&self) -> String {
        format!(
            "LangevinThermostat(kT={:.6}, n_dofs={}, seed={})",
            self.k_b_t,
            self.gamma.len(),
            self.seed,
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn new_initializes_stochastic_active_to_true() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42);
        assert!(t.is_stochastic_active());
    }

    #[test]
    fn set_stochastic_active_roundtrips() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42);
        t.set_stochastic_active(false);
        assert!(!t.is_stochastic_active());
        t.set_stochastic_active(true);
        assert!(t.is_stochastic_active());
    }

    #[test]
    fn as_stochastic_returns_some_self() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42);
        let view = t.as_stochastic();
        assert!(view.is_some());
        // Round-trip the flag through the dyn Stochastic view to
        // confirm it's pointing at the same AtomicBool.
        let view = view.unwrap();
        assert!(view.is_stochastic_active());
        view.set_stochastic_active(false);
        assert!(!t.is_stochastic_active());
    }

    #[test]
    fn diagnostic_summary_format_matches_spec() {
        let t = LangevinThermostat::new(DVector::from_element(3, 0.5), 1.25, 0x00C0_FFEE);
        // Spec §4.3 format: "LangevinThermostat(kT={:.6}, n_dofs={}, seed={})"
        assert_eq!(
            t.diagnostic_summary(),
            "LangevinThermostat(kT=1.250000, n_dofs=3, seed=12648430)",
        );
    }

    #[test]
    fn apply_writes_pure_damping_when_stochastic_inactive() {
        // Build a real 1-DOF SHO model so model.timestep is set.
        let xml = r#"
        <mujoco model="apply_test">
          <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
          <worldbody>
            <body name="particle">
              <joint name="x" type="slide" axis="1 0 0"
                     stiffness="1" damping="0" springref="0" ref="0"/>
              <geom type="sphere" size="0.05" mass="1"/>
            </body>
          </worldbody>
        </mujoco>"#;
        let model = sim_mjcf::load_model(xml).unwrap();
        let mut data = model.make_data();
        data.qvel[0] = 1.0;

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42);
        t.set_stochastic_active(false);

        let mut qfrc_out: DVector<f64> = DVector::zeros(model.nv);
        t.apply(&model, &data, &mut qfrc_out);

        // Pure damping at qvel=1, gamma=0.1: qfrc_out[0] = -0.1
        assert!(
            (qfrc_out[0] - (-0.1)).abs() < 1e-15,
            "stochastic-inactive apply should produce -γ·v exactly: \
             expected -0.1, got {}",
            qfrc_out[0],
        );
    }

    #[test]
    fn apply_does_not_advance_rng_when_stochastic_inactive() {
        // Two consecutive applies under stochastic-inactive should
        // produce identical qfrc_out and identical RNG state — i.e.
        // running apply twice must NOT advance the underlying ChaCha
        // stream. This is the Phase 5 finite-difference invariant
        // (Decision 7).
        let xml = r#"
        <mujoco model="rng_advance_test">
          <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
          <worldbody>
            <body name="particle">
              <joint name="x" type="slide" axis="1 0 0"
                     stiffness="1" damping="0" springref="0" ref="0"/>
              <geom type="sphere" size="0.05" mass="1"/>
            </body>
          </worldbody>
        </mujoco>"#;
        let model = sim_mjcf::load_model(xml).unwrap();
        let mut data = model.make_data();
        data.qvel[0] = 0.5;

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42);
        t.set_stochastic_active(false);

        // Two passes — both should hit the early-return before the
        // RNG lock. Re-enable stochastic and the RNG should still be
        // at its seeded initial state.
        let mut q1: DVector<f64> = DVector::zeros(model.nv);
        t.apply(&model, &data, &mut q1);
        let mut q2: DVector<f64> = DVector::zeros(model.nv);
        t.apply(&model, &data, &mut q2);
        assert_eq!(q1[0], q2[0]);

        // Now flip stochastic on and grab the next noise sample.
        // Build a fresh thermostat with the same seed and grab its
        // first noise sample directly. They must be equal — proving
        // the inactive applies did not advance the RNG.
        t.set_stochastic_active(true);
        let mut q_active: DVector<f64> = DVector::zeros(model.nv);
        t.apply(&model, &data, &mut q_active);

        let t_fresh = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42);
        let mut q_fresh: DVector<f64> = DVector::zeros(model.nv);
        t_fresh.apply(&model, &data, &mut q_fresh);

        assert_eq!(
            q_active[0], q_fresh[0],
            "RNG should not have advanced during stochastic-inactive applies; \
             active first-sample = {}, fresh first-sample = {}",
            q_active[0], q_fresh[0],
        );
    }

    #[test]
    fn apply_accumulates_into_existing_qfrc_out_with_plus_equals() {
        // The M5 contract: components accumulate with += not =. If
        // qfrc_out already has a value (e.g. from mj_passive's
        // spring/damper aggregation, or from an earlier component
        // in the stack), apply must ADD to it, not overwrite.
        let xml = r#"
        <mujoco model="accumulate_test">
          <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
          <worldbody>
            <body name="particle">
              <joint name="x" type="slide" axis="1 0 0"
                     stiffness="1" damping="0" springref="0" ref="0"/>
              <geom type="sphere" size="0.05" mass="1"/>
            </body>
          </worldbody>
        </mujoco>"#;
        let model = sim_mjcf::load_model(xml).unwrap();
        let mut data = model.make_data();
        data.qvel[0] = 1.0;

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42);
        t.set_stochastic_active(false);

        // Pre-load qfrc_out with a sentinel value.
        let mut qfrc_out: DVector<f64> = DVector::from_element(model.nv, 7.0);
        t.apply(&model, &data, &mut qfrc_out);

        // Expected: 7.0 + (-0.1) = 6.9
        assert!(
            (qfrc_out[0] - 6.9).abs() < 1e-15,
            "apply should accumulate with +=: expected 6.9, got {}",
            qfrc_out[0],
        );
    }
}
