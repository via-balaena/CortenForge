//! `LangevinThermostat` — explicit Langevin thermostat via Euler-Maruyama.
//!
//! The thermostat writes the fluctuation–dissipation pair `(−γ·v, σ·z)` into the per-DOF
//! accumulator on every step:
//!
//! ```text
//! qfrc_out[i] += −γ_i · qvel[i]  +  sqrt(2 · γ_i · k_B·T / h) · z_i,    z_i ~ N(0, 1)
//!                └── damping ──┘   └────── FDT-paired noise ──────┘
//! ```
//!
//! `γ_i` and `k_B·T` are owned by this struct — `model.dof_damping`
//! stays at zero. The fluctuation–
//! dissipation relation `σ² = 2γkT/h` is the only physics statement
//! the implementation makes; everything else is bookkeeping.
//!
//! The discretization-bias temperature error is `O(h·γ/M)`: at `h = 0.001`,
//! `γ = 0.1`, `M = 1` that is `≈ 10⁻⁴` of `½kT`.
//!
//! The damping is computed from each step's starting velocity, so under
//! the Euler integrator, for a diagonal mass matrix, it alone multiplies a
//! DOF's velocity by `1 − γh/M` per step (`M` the DOF's mass or inertia):
//! stable only for `γh/M < 2`. With coupled DOFs the eigenvalues of
//! `h·M⁻¹·Γ` set the limit instead.
//!
//! The thermostat is measured under the Euler integrator. RK4 calls the
//! passive callback four times per step and the thermostat draws fresh
//! noise at each call, so `validate` refuses RK4 (and
//! [`crate::PassiveStack::try_install`] with it). The implicit integrators
//! have not been measured. Changing `model.integrator` after install
//! bypasses the check.
//!
//! ## RNG and `cb_passive`
//!
//! The thermostat holds no mutable RNG state. Noise at step `s` for DOF `d` is computed as
//!
//! ```text
//! (counter, stream) = noise_position(traj_id, s, group)
//! block = chacha8_block(master_key, counter, stream)
//! z_d   = box_muller_from_block(block)[d - group*8]
//! ```
//!
//! where `master_key` is expanded once at construction from the
//! user-supplied `master_seed: u64`, `traj_id` is set per env by an
//! `install_per_env` factory, `s` is the component's own `AtomicU64`
//! counter (advanced once per `apply` call, gated by
//! `stochastic_active`), and `group = d / 8` handles DOF counts above
//! 8. Each `(traj_id, s, group)` has its own block, so every DOF group
//! at every step draws independent noise.
//!
//! Because the PRF is a pure function of integers, a thermostat's noise
//! at a given step depends only on `(master_seed, traj_id, step)`, not on
//! thread scheduling. That holds when each env has its own thermostat
//! (`install_per_env`, `BatchSim::new_per_env`). A stack shared by several
//! `Data` (a cloned `Model`, or `BatchSim::new`) shares one step counter,
//! so which env draws which step depends on the order of the calls.
//!
//! The step index takes 48 bits of the noise position, so a thermostat panics
//! once its step counter reaches `2^48` (after about `2.8·10¹⁴` noise draws). See
//! [`crate::prf`] for the primitives.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use sim_core::{DVector, Data, Integrator, Model};

use crate::component::{PassiveComponent, Stochastic, check_ctrl, check_dof, clamped_ctrl};
use crate::diagnose::Diagnose;
use crate::error::ThermostatError;
use crate::params::{Domain, or_panic};
use crate::prf;

/// Explicit Langevin thermostat implementing the
/// fluctuation–dissipation pair `(−γ·v, σ·z)` via Euler-Maruyama.
///
/// Construct via [`LangevinThermostat::new`], add to a stack via
/// `PassiveStack::builder().with(thermostat).build()`, install onto
/// a `Model` via `stack.try_install(&mut model)?`, and step the
/// simulation normally with `data.step(&model)?`.
///
/// Implements three traits:
/// - [`PassiveComponent`]: `apply` writes the forces into `qfrc_out`.
/// - [`Stochastic`]: `PassiveStack::disable_stochastic` switches the noise
///   off, for finite-difference and autograd contexts.
/// - [`Diagnose`]: a one-line summary for debugging and test failures.
///
/// # Panics
///
/// `apply` panics once the step counter reaches `2^48` (after about `2.8·10¹⁴`
/// noise draws): the step index takes 48 bits of the noise position.
pub struct LangevinThermostat {
    gamma: DVector<f64>,
    k_b_t: f64,
    /// User-facing seed (D7). Retained alongside `master_key` for
    /// display in `diagnostic_summary` — the key space is larger than
    /// the seed space, so reconstructing `u64` from `[u8; 32]` would
    /// be lossy.
    master_seed: u64,
    /// 32-byte `ChaCha8` key, expanded once at construction from
    /// `master_seed` via [`crate::prf::expand_master_seed`] (D7).
    master_key: [u8; 32],
    /// Per-env trajectory identifier (D2). Typically the env index
    /// under a `PassiveStack::install_per_env` factory; any distinct
    /// `u64` value produces a disjoint noise stream at the same
    /// `master_seed`.
    traj_id: u64,
    /// Step index, advanced once per `apply` call (gated by
    /// `stochastic_active`). Atomic so the containing stack's
    /// `cb_passive` closure can hold `&self`.
    counter: AtomicU64,
    stochastic_active: AtomicBool,
    /// Optional ctrl index for runtime temperature modulation (D2).
    /// When `Some(idx)`, `apply` reads `data.ctrl[idx]` as a multiplier
    /// on `k_b_t`. When `None`, `k_b_t` is used directly.
    k_b_t_ctrl: Option<usize>,
}

/// The largest temperature multiplier [`LangevinThermostat::ctrl_multiplier`] returns.
const MAX_CTRL_MULTIPLIER: f64 = 10.0;

impl LangevinThermostat {
    /// The most DOFs a thermostat acts on: each group of 8 DOFs needs its own
    /// noise stream, and there are `2^16` of them.
    pub const MAX_DOFS: usize = 1 << 19;

    /// Construct a thermostat with per-DOF damping coefficients
    /// `gamma`, bath temperature `k_b_t`, master seed `master_seed`,
    /// and trajectory id `traj_id`.
    ///
    /// The thermostat acts on DOFs `0..gamma.len()`: a `gamma` shorter than
    /// the model's DOF count leaves the other DOFs alone, and a longer one
    /// is refused at install (see [`PassiveComponent::validate`]). So is a
    /// `gamma` and `k_b_t` whose noise variance `2·γ·kT/h` (×10 under
    /// [`Self::with_ctrl_temperature`]) is not finite at the model's timestep
    /// `h`; a timestep changed after install is not checked again.
    ///
    /// `master_seed` is expanded once at construction into a 32-byte
    /// `ChaCha8` key via `prf::expand_master_seed` (a private helper
    /// in this crate's `prf` module); the thermostat's
    /// noise stream at step `s` for DOF `d` is a pure function of
    /// `(master_key, traj_id, s, d)`. `traj_id` is typically the env
    /// index under an `install_per_env` factory, but it can be any
    /// `u64` — distinct `traj_id` values at the same `master_seed`
    /// produce disjoint noise streams by the PRF's construction.
    ///
    /// # Panics
    ///
    /// If [`Self::try_new`] refuses the parameters: `gamma` has more than
    /// [`Self::MAX_DOFS`] (`2^19`) entries, or an entry of `gamma`, or
    /// `k_b_t`, is negative or not finite.
    #[must_use]
    #[track_caller]
    pub fn new(gamma: DVector<f64>, k_b_t: f64, master_seed: u64, traj_id: u64) -> Self {
        or_panic(Self::try_new(gamma, k_b_t, master_seed, traj_id))
    }

    /// [`Self::new`], returning the refusal instead of panicking.
    ///
    /// # Errors
    ///
    /// [`ThermostatError::TooManyDofs`] if `gamma` has more than
    /// [`Self::MAX_DOFS`] entries; [`ThermostatError::InvalidParameter`] if
    /// an entry of `gamma`, or `k_b_t`, is negative or not finite.
    pub fn try_new(
        gamma: DVector<f64>,
        k_b_t: f64,
        master_seed: u64,
        traj_id: u64,
    ) -> Result<Self, ThermostatError> {
        const COMPONENT: &str = "LangevinThermostat";
        if gamma.len() > Self::MAX_DOFS {
            return Err(ThermostatError::TooManyDofs {
                component: COMPONENT,
                dofs: gamma.len(),
                max: Self::MAX_DOFS,
            });
        }
        Domain::NonNegative.check_each(COMPONENT, "gamma", gamma.as_slice())?;
        Domain::NonNegative.check(COMPONENT, "k_b_t", k_b_t)?;
        Ok(Self {
            gamma,
            k_b_t,
            master_seed,
            master_key: prf::expand_master_seed(master_seed),
            traj_id,
            counter: AtomicU64::new(0),
            stochastic_active: AtomicBool::new(true),
            k_b_t_ctrl: None,
        })
    }

    /// The temperature multiplier read from control value `ctrl` under
    /// [`Self::with_ctrl_temperature`]: `ctrl` clamped to `[0, 10]`, with a
    /// bad value (`NaN`, infinite, or beyond ±1e10) counting as 0.
    #[must_use]
    pub fn ctrl_multiplier(ctrl: f64) -> f64 {
        clamped_ctrl(ctrl, MAX_CTRL_MULTIPLIER)
    }

    /// Enable runtime temperature modulation via a ctrl channel.
    ///
    /// When set, `apply` reads `data.ctrl[ctrl_idx]` as a multiplier on
    /// the base `k_b_t`. The effective temperature is
    /// `k_b_t * Self::ctrl_multiplier(ctrl)`: the control clamped to
    /// `[0, 10]`, with a bad value counting as 0. At multiplier 0, the
    /// thermostat produces pure damping (no noise). At 10, the effective
    /// temperature is 10× the base. The model needs control channel
    /// `ctrl_idx`: install refuses a model without it.
    ///
    /// Without calling this method, `apply` uses the base `k_b_t`.
    #[must_use]
    pub const fn with_ctrl_temperature(mut self, ctrl_idx: usize) -> Self {
        self.k_b_t_ctrl = Some(ctrl_idx);
        self
    }
}

impl PassiveComponent for LangevinThermostat {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        // Read h fresh per step (recon log part 8): cost is one f64
        // load + one sqrt per DOF, robust to mid-simulation timestep
        // mutation.
        let h = model.timestep;
        let n_dofs = self.gamma.len();

        // Effective temperature: base kT scaled by ctrl multiplier
        // when with_ctrl_temperature is active. Damping is NOT affected
        // by the multiplier — only the FDT-paired noise amplitude
        // changes. This preserves the FDT relation at the effective
        // temperature.
        let k_b_t = self.k_b_t_ctrl.map_or(self.k_b_t, |idx| {
            self.k_b_t * Self::ctrl_multiplier(data.ctrl[idx])
        });

        // Damping (-γ·v) is unconditional — it is the deterministic
        // half of the FD pair and runs in both stochastic-active and
        // stochastic-inactive states. Per Decision 7 §spec 4.2:
        // "When inactive, apply produces only the deterministic
        // part (-γ·v for the Langevin thermostat)."
        for i in 0..n_dofs {
            qfrc_out[i] += -self.gamma[i] * data.qvel[i];
        }

        // Gating early-return BEFORE the counter advance. Load-bearing
        // for the FD invariant: no counter advance under
        // disable_stochastic ⇒ post-re-enable PRF coordinate equals
        // pre-disable PRF coordinate.
        if !self.stochastic_active.load(Ordering::Relaxed) {
            return;
        }

        // Counter advance (one per apply call). `fetch_add` is atomic, so
        // each call gets its own step index whatever the ordering. A stack
        // shared by several envs shares this counter (see the module doc).
        let step_index = self.counter.fetch_add(1, Ordering::Relaxed);

        // DOFs in groups of 8: one ChaCha8 block yields 64 bytes = 8
        // f64 Box-Muller samples. The common path (n_dofs ≤ 8) does
        // exactly one iteration.
        let n_groups = n_dofs.div_ceil(8);
        for group in 0..n_groups {
            let (counter, stream) = prf::noise_position(self.traj_id, step_index, group as u64);
            let block = prf::chacha8_block(&self.master_key, counter, stream);
            let gaussians = prf::box_muller_from_block(&block);
            let dof_start = group * 8;
            let dof_end = (dof_start + 8).min(n_dofs);
            for dof in dof_start..dof_end {
                let gamma_i = self.gamma[dof];
                // σ² = 2·γ·k_B·T_eff / h.
                let sigma = (2.0 * gamma_i * k_b_t / h).sqrt();
                qfrc_out[dof] += sigma * gaussians[dof - dof_start];
            }
        }
    }

    fn as_stochastic(&self) -> Option<&dyn Stochastic> {
        Some(self)
    }

    fn as_diagnose(&self) -> Option<&dyn Diagnose> {
        Some(self)
    }

    /// Accepts a `gamma` shorter than the model's DOF count: the thermostat acts on the
    /// first DOFs only. Refuses a noise variance that is not finite at the model's timestep.
    fn validate(&self, model: &Model) -> Result<(), ThermostatError> {
        if let Some(last) = self.gamma.len().checked_sub(1) {
            check_dof(model, last, "LangevinThermostat")?;
        }
        // The largest variance `apply` can compute, in its order of operations.
        let k_b_t = self.k_b_t * self.k_b_t_ctrl.map_or(1.0, |_| MAX_CTRL_MULTIPLIER);
        let h = model.timestep;
        if let Some(dof) = self
            .gamma
            .iter()
            .position(|&gamma_i| !(2.0 * gamma_i * k_b_t / h).is_finite())
        {
            return Err(ThermostatError::NoiseOverflow {
                component: "LangevinThermostat",
                dof,
                timestep: h,
            });
        }
        if model.integrator == Integrator::RungeKutta4 {
            return Err(ThermostatError::UnsupportedIntegrator {
                component: "LangevinThermostat",
                integrator: model.integrator,
                reason: "RK4 calls the passive callback four times per step, and the thermostat \
                         draws fresh noise at each call",
            });
        }
        self.k_b_t_ctrl
            .map_or(Ok(()), |ctrl| check_ctrl(model, ctrl, "LangevinThermostat"))
    }
}

impl Stochastic for LangevinThermostat {
    fn set_stochastic_active(&self, active: bool) {
        self.stochastic_active.store(active, Ordering::Relaxed);
    }

    fn is_stochastic_active(&self) -> bool {
        self.stochastic_active.load(Ordering::Relaxed)
    }

    fn reset_stochastic(&self) {
        self.counter.store(0, Ordering::Relaxed);
    }
}

impl Diagnose for LangevinThermostat {
    fn diagnostic_summary(&self) -> String {
        // D8: static-only tuple. The runtime `counter` is deliberately
        // excluded — `diagnostic_summary` reports the configuration
        // that identifies this thermostat instance, not its running
        // state.
        self.k_b_t_ctrl.map_or_else(
            || {
                format!(
                    "LangevinThermostat(kT={:.6}, n_dofs={}, master_seed={}, traj_id={})",
                    self.k_b_t,
                    self.gamma.len(),
                    self.master_seed,
                    self.traj_id,
                )
            },
            |idx| {
                format!(
                    "LangevinThermostat(kT={:.6}, n_dofs={}, master_seed={}, traj_id={}, ctrl_temp={})",
                    self.k_b_t,
                    self.gamma.len(),
                    self.master_seed,
                    self.traj_id,
                    idx,
                )
            },
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn new_initializes_stochastic_active_to_true() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42, 0);
        assert!(t.is_stochastic_active());
    }

    #[test]
    fn set_stochastic_active_roundtrips() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42, 0);
        t.set_stochastic_active(false);
        assert!(!t.is_stochastic_active());
        t.set_stochastic_active(true);
        assert!(t.is_stochastic_active());
    }

    #[test]
    fn as_stochastic_returns_some_self() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42, 0);
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
        let t = LangevinThermostat::new(DVector::from_element(3, 0.5), 1.25, 0x00C0_FFEE, 0);
        // D8: static-only tuple (kT, n_dofs, master_seed, traj_id).
        // Runtime counter is deliberately excluded.
        assert_eq!(
            t.diagnostic_summary(),
            "LangevinThermostat(kT=1.250000, n_dofs=3, master_seed=12648430, traj_id=0)",
        );
    }

    #[test]
    fn apply_writes_pure_damping_when_stochastic_inactive() {
        // Build a real 1-DOF SHO model so model.timestep is set.
        let model = sim_core::test_fixtures::sho_1d();
        let mut data = model.make_data();
        data.qvel[0] = 1.0;

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0);
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
        let model = sim_core::test_fixtures::sho_1d();
        let mut data = model.make_data();
        data.qvel[0] = 0.5;

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0);
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

        let t_fresh = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0);
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
        let model = sim_core::test_fixtures::sho_1d();
        let mut data = model.make_data();
        data.qvel[0] = 1.0;

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0);
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

    // ── with_ctrl_temperature tests ─────────────────────────────────────

    #[test]
    fn default_has_no_ctrl_temperature() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42, 0);
        let s = t.diagnostic_summary();
        assert!(
            !s.contains("ctrl_temp"),
            "default thermostat should not have ctrl_temp: {s}",
        );
    }

    #[test]
    fn with_ctrl_temperature_shows_in_diagnostic() {
        let t = LangevinThermostat::new(DVector::from_element(1, 0.1), 1.0, 42, 0)
            .with_ctrl_temperature(0);
        let s = t.diagnostic_summary();
        assert!(
            s.contains("ctrl_temp=0"),
            "diagnostic should show ctrl_temp=0: {s}",
        );
    }

    #[test]
    fn ctrl_temperature_damping_unchanged() {
        // Damping (-γ·v) must NOT depend on the ctrl temperature
        // multiplier — damping is a bath-coupling property, not a
        // temperature property.
        let model = sim_core::test_fixtures::stochastic_resonance();
        let mut data = model.make_data();
        data.qvel[0] = 1.0;
        data.ctrl[0] = 5.0; // 5× multiplier

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0)
            .with_ctrl_temperature(0);
        t.set_stochastic_active(false);

        let mut qfrc_out: DVector<f64> = DVector::zeros(model.nv);
        t.apply(&model, &data, &mut qfrc_out);

        // Damping = -γ·v = -0.1·1.0 = -0.1, independent of ctrl
        assert!(
            (qfrc_out[0] - (-0.1)).abs() < 1e-15,
            "damping should be -0.1 regardless of ctrl multiplier: got {}",
            qfrc_out[0],
        );
    }

    #[test]
    fn ctrl_temperature_scales_noise() {
        // Two thermostats, same seed: one with kT=2.0 (no ctrl),
        // one with kT=1.0 + ctrl=2.0. Both should produce identical
        // noise because effective kT is 2.0 in both cases.
        let model = sim_core::test_fixtures::stochastic_resonance();

        // Thermostat A: kT=2.0, no ctrl
        let t_a = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 2.0, 42, 0);
        let data_a = model.make_data();
        // qvel already 0 from make_data → zero damping

        let mut q_a: DVector<f64> = DVector::zeros(model.nv);
        t_a.apply(&model, &data_a, &mut q_a);

        // Thermostat B: kT=1.0, ctrl=2.0 → effective kT=2.0
        let t_b = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0)
            .with_ctrl_temperature(0);
        let mut data_b = model.make_data();
        data_b.qvel[0] = 0.0;
        data_b.ctrl[0] = 2.0;

        let mut q_b: DVector<f64> = DVector::zeros(model.nv);
        t_b.apply(&model, &data_b, &mut q_b);

        assert_eq!(
            q_a[0], q_b[0],
            "same seed, same effective kT=2.0 should produce identical noise: \
             A={}, B={}",
            q_a[0], q_b[0],
        );
    }

    #[test]
    fn ctrl_temperature_clamps_negative_to_zero() {
        // Negative ctrl → clamped to 0.0 → effective kT = 0 → no noise.
        // The fixture's actuator has ctrlrange (0, 10) but apply() reads
        // data.ctrl[0] directly, bypassing actuator-level clamping —
        // the clamping under test lives in LangevinThermostat itself.
        let model = sim_core::test_fixtures::stochastic_resonance();
        let mut data = model.make_data();
        data.ctrl[0] = -5.0; // negative → clamped to 0

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0)
            .with_ctrl_temperature(0);

        let mut qfrc_out: DVector<f64> = DVector::zeros(model.nv);
        t.apply(&model, &data, &mut qfrc_out);

        // kT_eff = 0 → σ = 0 → noise = 0. Damping also 0 (qvel=0).
        assert_eq!(
            qfrc_out[0], 0.0,
            "negative ctrl should clamp to zero noise: got {}",
            qfrc_out[0],
        );
    }

    #[test]
    fn ctrl_temperature_clamps_above_ten() {
        // ctrl=20 → clamped to 10 → effective kT = 10. Should match
        // a thermostat with kT=10 directly. The fixture's actuator has
        // ctrlrange (0, 10) but apply() reads data.ctrl[0] directly,
        // bypassing actuator-level clamping — the clamping under test
        // lives in LangevinThermostat itself.
        let model = sim_core::test_fixtures::stochastic_resonance();

        // Thermostat A: kT=10, no ctrl
        let t_a = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 10.0, 42, 0);
        let data_a = model.make_data();
        let mut q_a: DVector<f64> = DVector::zeros(model.nv);
        t_a.apply(&model, &data_a, &mut q_a);

        // Thermostat B: kT=1.0, ctrl=20 → clamped to 10 → effective kT=10
        let t_b = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0)
            .with_ctrl_temperature(0);
        let mut data_b = model.make_data();
        data_b.ctrl[0] = 20.0;
        let mut q_b: DVector<f64> = DVector::zeros(model.nv);
        t_b.apply(&model, &data_b, &mut q_b);

        assert_eq!(
            q_a[0], q_b[0],
            "ctrl=20 should clamp to 10, matching kT=10 thermostat: \
             A={}, B={}",
            q_a[0], q_b[0],
        );
    }

    #[test]
    fn ctrl_temperature_reads_a_bad_control_as_zero() {
        // A bad control (NaN, ±∞, beyond ±1e10) → multiplier 0 → damping only.
        let model = sim_core::test_fixtures::stochastic_resonance();
        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0)
            .with_ctrl_temperature(0);
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 2e10, -2e10] {
            let mut data = model.make_data();
            data.qvel[0] = 1.0;
            data.ctrl[0] = bad;
            let mut qfrc_out: DVector<f64> = DVector::zeros(model.nv);
            t.apply(&model, &data, &mut qfrc_out);
            assert_eq!(qfrc_out[0], -0.1, "ctrl {bad}: expected damping only");
        }
    }

    /// sim-core's actuation stage sets a bad control to 0 before passive forces run; with
    /// actuation disabled the thermostat gets the bad value itself, and reads it as 0.
    #[test]
    fn a_bad_control_is_read_as_zero_with_actuation_on_or_off() {
        for disable_actuation in [false, true] {
            let mut model = sim_core::test_fixtures::stochastic_resonance();
            if disable_actuation {
                model.disableflags |= sim_core::DISABLE_ACTUATION;
            }
            crate::PassiveStack::builder()
                .with(
                    LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0)
                        .with_ctrl_temperature(0),
                )
                .build()
                .try_install(&mut model)
                .unwrap();
            let mut data = model.make_data();
            data.qvel[0] = 1.0;
            data.ctrl[0] = f64::NAN;
            data.forward(&model).unwrap();
            assert_eq!(data.ctrl[0].is_nan(), disable_actuation);
            assert_eq!(
                data.qfrc_passive[0], -0.1,
                "actuation disabled: {disable_actuation}"
            );
        }
    }

    /// The refusal `LangevinThermostat::try_new(gamma, k_b_t, ..)` returns, if any.
    fn refusal(gamma: &[f64], k_b_t: f64) -> Option<ThermostatError> {
        LangevinThermostat::try_new(DVector::from_column_slice(gamma), k_b_t, 42, 0).err()
    }

    /// The parameter named by an `InvalidParameter` refusal.
    fn refused_parameter(refusal: Option<ThermostatError>) -> Option<String> {
        match refusal {
            Some(ThermostatError::InvalidParameter { parameter, .. }) => Some(parameter),
            _ => None,
        }
    }

    #[test]
    fn try_new_refuses_negative_or_non_finite_damping_and_temperature() {
        for bad in [-0.1, f64::NAN, f64::INFINITY] {
            assert_eq!(
                refused_parameter(refusal(&[0.1, bad], 1.0)).as_deref(),
                Some("gamma[1]"),
                "gamma[1] = {bad}"
            );
            assert_eq!(
                refused_parameter(refusal(&[0.1, 0.1], bad)).as_deref(),
                Some("k_b_t"),
                "k_b_t = {bad}"
            );
        }
        assert_eq!(
            refusal(&[0.0, 0.1], 0.0),
            None,
            "zero damping and temperature are allowed"
        );
    }

    #[test]
    #[should_panic(
        expected = "LangevinThermostat: gamma[0] must be finite and non-negative, got -1"
    )]
    fn new_panics_with_the_refusal() {
        let _t = LangevinThermostat::new(DVector::from_element(1, -1.0), 1.0, 0, 0);
    }

    /// A noise variance `2·γ·kT/h` that overflows `f64` is refused at install, under ctrl
    /// temperature at its largest multiplier, 10. The fixture's timestep is 1e-3.
    #[test]
    fn install_refuses_a_noise_variance_that_overflows() {
        let verdict = |gamma: f64, ctrl: bool| {
            let mut model = sim_core::test_fixtures::stochastic_resonance();
            let mut thermostat =
                LangevinThermostat::new(DVector::from_element(1, gamma), 1.0, 0, 0);
            if ctrl {
                thermostat = thermostat.with_ctrl_temperature(0);
            }
            crate::PassiveStack::builder()
                .with(thermostat)
                .build()
                .try_install(&mut model)
        };
        let overflow = Err(ThermostatError::NoiseOverflow {
            component: "LangevinThermostat",
            dof: 0,
            timestep: 1e-3,
        });
        assert_eq!(verdict(1e306, false), overflow);
        assert_eq!(verdict(1e304, false), Ok(()));
        assert_eq!(verdict(1e304, true), overflow);
        assert_eq!(verdict(1e303, true), Ok(()));
    }

    #[test]
    fn ctrl_multiplier_clamps_and_reads_a_bad_control_as_zero() {
        for (ctrl, expected) in [
            (0.5, 0.5),
            (20.0, 10.0),
            (1e10, 10.0),
            (-1.0, 0.0),
            (f64::NAN, 0.0),
            (f64::INFINITY, 0.0),
            (f64::NEG_INFINITY, 0.0),
            (2e10, 0.0),
        ] {
            assert_eq!(
                LangevinThermostat::ctrl_multiplier(ctrl),
                expected,
                "ctrl {ctrl}"
            );
        }
    }

    #[test]
    fn ctrl_zero_produces_zero_noise() {
        // ctrl=0 → kT_eff=0 → σ=0 → pure damping (+ RNG advances)
        let model = sim_core::test_fixtures::stochastic_resonance();
        let mut data = model.make_data();
        data.qvel[0] = 1.0;
        data.ctrl[0] = 0.0;

        let t = LangevinThermostat::new(DVector::from_element(model.nv, 0.1), 1.0, 42, 0)
            .with_ctrl_temperature(0);

        let mut qfrc_out: DVector<f64> = DVector::zeros(model.nv);
        t.apply(&model, &data, &mut qfrc_out);

        // Pure damping: -γ·v = -0.1·1.0 = -0.1, no noise
        assert!(
            (qfrc_out[0] - (-0.1)).abs() < 1e-15,
            "ctrl=0 should produce pure damping: expected -0.1, got {}",
            qfrc_out[0],
        );
    }

    /// The noise force on each of `n` DOFs at step `step` of trajectory `traj_id`: one
    /// `apply` on an `n`-slide chain at rest, so the damping term is zero.
    fn noise_forces(n: usize, traj_id: u64, step: u64) -> Vec<f64> {
        let model = sim_core::test_fixtures::bistable_chain(n);
        let data = model.make_data();
        let t = LangevinThermostat::new(DVector::from_element(n, 0.5), 1.0, 42, traj_id);
        t.counter.store(step, Ordering::Relaxed);
        let mut qfrc_out: DVector<f64> = DVector::zeros(n);
        t.apply(&model, &data, &mut qfrc_out);
        qfrc_out.iter().copied().collect()
    }

    /// No two DOF groups share noise at any step. Before each group had its own stream,
    /// group `g` at step `s` drew the same block as group 0 at step `s + g`, so DOFs 0–7
    /// replayed DOFs 8–15's noise one step later. 17 DOFs give two full groups and a
    /// partial third.
    #[test]
    fn dof_groups_draw_distinct_noise() {
        let n = 17;
        let steps = 0..4u64;
        let mut first_values = Vec::new();
        let mut full_blocks = Vec::new();
        for step in steps {
            let f = noise_forces(n, 0, step);
            for group in 0..n.div_ceil(8) {
                first_values.push(f[group * 8].to_bits());
                if group * 8 + 8 <= n {
                    let block: Vec<u64> = f[group * 8..group * 8 + 8]
                        .iter()
                        .map(|v| v.to_bits())
                        .collect();
                    full_blocks.push(block);
                }
            }
        }
        let distinct_first: std::collections::HashSet<_> = first_values.iter().collect();
        assert_eq!(
            distinct_first.len(),
            first_values.len(),
            "a noise value repeated"
        );
        for (i, a) in full_blocks.iter().enumerate() {
            for b in &full_blocks[i + 1..] {
                assert_ne!(a, b, "two DOF groups drew the same 8 noise values");
            }
        }
    }

    /// Group 0's noise is unchanged by giving each group its own stream: pinned bits of DOFs
    /// 0 and 7, recorded before the change (at `e2d42077`), for trajectory ids and steps up to
    /// 2^32 − 1 (above that the noise changed by design). A 17-DOF run's first 8 DOFs match
    /// the 8-DOF run.
    #[test]
    fn group_zero_noise_is_unchanged() {
        const PINS: [(u64, u64, u64, u64); 6] = [
            (0, 0, 0x403a_5529_e5c6_5d49, 0xc046_e337_607c_c8e3),
            (0, 0xFFFF_FFFF, 0xc040_ab5b_1e69_0166, 0xc02f_8f6b_e841_d60b),
            (7, 0, 0x401f_dae6_6ecd_fa57, 0x4045_6b4b_62f0_ff6e),
            (7, 0xFFFF_FFFF, 0xc031_67e9_d2b3_c66c, 0xc044_fe83_77bb_795f),
            (0xFFFF_FFFF, 0, 0x4041_8479_e306_ec81, 0x4025_4797_3cfc_ceb3),
            (
                0xFFFF_FFFF,
                0xFFFF_FFFF,
                0x403f_b3a8_1f2e_aa7c,
                0x3ff3_8049_d5a3_86bb,
            ),
        ];
        for (traj, step, dof0, dof7) in PINS {
            let f8 = noise_forces(8, traj, step);
            assert_eq!(
                f8[0].to_bits(),
                dof0,
                "DOF 0 moved at (traj {traj}, step {step})"
            );
            assert_eq!(
                f8[7].to_bits(),
                dof7,
                "DOF 7 moved at (traj {traj}, step {step})"
            );
            let f17 = noise_forces(17, traj, step);
            assert_eq!(
                f17[..8],
                f8[..],
                "17-DOF run's group 0 differs at (traj {traj}, step {step})"
            );
        }
    }

    /// Trajectory ids that differ only above bit 32, and steps 0 and 2^32, used to share noise.
    #[test]
    fn noise_differs_across_the_32_bit_boundaries() {
        let base = noise_forces(8, 0, 0);
        assert_ne!(
            noise_forces(8, 1 << 32, 0),
            base,
            "traj_id 2^32 shares traj 0's noise"
        );
        assert_ne!(
            noise_forces(8, 0, 1 << 32),
            base,
            "step 2^32 shares step 0's noise"
        );
    }

    #[test]
    fn try_new_takes_max_dofs_and_refuses_one_more() {
        let max = LangevinThermostat::MAX_DOFS;
        assert_eq!(refusal(&vec![0.1; max], 1.0), None);
        assert!(matches!(
            refusal(&vec![0.1; max + 1], 1.0),
            Some(ThermostatError::TooManyDofs { dofs, max: m, .. }) if dofs == max + 1 && m == max
        ));
    }

    #[test]
    #[should_panic(expected = "LangevinThermostat supports at most 524288 DOFs")]
    fn new_refuses_more_than_2_pow_19_dofs() {
        let _t = LangevinThermostat::new(DVector::from_element((1 << 19) + 1, 0.1), 1.0, 0, 0);
    }
}
