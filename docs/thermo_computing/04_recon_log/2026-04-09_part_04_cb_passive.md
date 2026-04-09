# 2026-04-09 (part 4) — Item 3 first finding: `cb_passive` exists, item 2 REVISED

> Extracted from `MASTER_PLAN.md` §7 part 04 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Started item 3 (existing usage patterns for writing into
  `qfrc_applied`) under sharpen-the-axe. The very first searches for
  `cb_passive` returned hits, exposing a documented hook that materially
  changes item 2's decision. Stopped item 3 immediately to surface the
  finding, per the "stop and ask when uncertain" principle.
- **What `cb_passive` is** — a documented user passive-force callback hook
  on `Model`, analogous to `cb_control`. Citations:
  - `model.rs:1002` — `pub cb_passive: Option<super::callbacks::CbPassive>`
  - `model.rs:1242, 1247` — setter and clearer methods
  - `model_init.rs:396` — initialized to `None` (opt-in by default)
  - `callbacks.rs:37-41` — type definition + contract
  - `forward/passive.rs:719` — invocation site

  Type: `Callback<dyn Fn(&Model, &mut Data) + Send + Sync>` — `Fn` (not
  `FnMut`), thread-safe, `Send + Sync` for cross-thread sharing in
  BatchSim.

  Documented contract (`callbacks.rs:37-41`): *"Passive force callback:
  called at end of `mj_fwd_passive()`. Use to inject custom passive forces
  (e.g., viscous drag, spring models). The callback may modify
  `data.qfrc_passive` or any other Data field."*
- **What `qfrc_passive` is** — engine-owned, **auto-cleared every step**.
  In `forward/passive.rs`:
  - Line 368 — `mj_fwd_passive()` *starts* by zeroing `qfrc_passive`,
    `qfrc_spring`, `qfrc_damper`, `qfrc_fluid`, `qfrc_gravcomp`
    (sleep-aware: only awake DOFs cleared if sleeping).
  - Lines 388–665 — spring, damper, tendon, flex, bending, fluid, and
    gravcomp forces accumulate into per-component arrays.
  - Line 681 — aggregation: `qfrc_passive = qfrc_spring + qfrc_damper`
    (assignment, not accumulation).
  - Lines 683–706 — gravcomp and fluid added optionally.
  - Line 719 — `cb_passive` fires *after* aggregation; the callback sees
    the populated `qfrc_passive` and can `+=` into it freely.
  - Lines 723–731 — plugin passive forces fire *after* `cb_passive`.

  `qfrc_passive` then flows into `qfrc_smooth` via
  `constraint/mod.rs:78-87` — same path as `qfrc_applied`, same projection
  by the constraint solver, same wake-up support.
- **Why this changes item 2**: Item 2's Option A (sole-writer overwrite of
  `qfrc_applied`) was forced by the fact that `qfrc_applied` is user-owned
  and persistent across steps — any additive scheme could silently compound
  noise. `qfrc_passive` is the opposite: engine-owned, auto-cleared. The
  thermostat can `+=` into it with zero risk of compounding because the
  next step starts from zero by construction. **The Option A vs B
  trade-off dissolves entirely.**
- **And the ontology is correct**: Langevin damping + FDT-paired stochastic
  forcing is *literally a passive thermal force*. The right sim-core
  categorization is:
  - `qfrc_actuator` — controlled actuator forces (motors, muscles)
  - `qfrc_passive` — passive forces (springs, damping, fluid drag,
    **thermal bath**)
  - `qfrc_applied` — external user-applied forces (RL action injection,
    manual loads)
  - `qfrc_constraint` — constraint reaction forces

  The thermostat belongs in `qfrc_passive`. Item 2 was solving the right
  problem in the wrong category.
- **Trade-off comparison** (cb_passive vs item 2 Option A):

  | Concern | Option A (qfrc_applied overwrite) | cb_passive + qfrc_passive |
  |---|---|---|
  | Compounding-safe? | Yes (overwrite) | Yes (auto-cleared) |
  | Composes with RL writers of `qfrc_applied`? | No | Yes (different field) |
  | Composes with multiple thermostats / passive plugins? | N/A | Yes |
  | Requires split-step API? | Yes (manual hook) | No (`step()` works) |
  | Idiomatic? | Hijacks user-input field | Uses the documented hook for exactly this purpose |
  | Forward-looking Option C still needed? | Yes, eventually | No — composability achieved already |

  cb_passive dominates on every axis except one — the RNG constraint —
  which is solvable.
- **The RNG constraint and its solution**: `cb_passive` is `Fn`, not
  `FnMut`. The closure cannot capture mutable state directly. Solution:
  the `LangevinThermostat` struct owns its state behind interior
  mutability, the closure captures `Arc<LangevinThermostat>`:

  ```rust
  pub struct LangevinThermostat {
      gamma: DVector<f64>,
      k_b_t: f64,
      rng: Mutex<ChaCha8Rng>,  // PRNG choice = recon item 4
  }

  impl LangevinThermostat {
      pub fn install(self: Arc<Self>, model: &mut Model) {
          let me = Arc::clone(&self);
          model.set_passive_callback(move |m, d| me.apply(m, d));
      }

      fn apply(&self, model: &Model, data: &mut Data) {
          let h = model.timestep;
          let mut rng = self.rng.lock().unwrap();
          for i in 0..self.gamma.len() {
              let g = self.gamma[i];
              let damping = -g * data.qvel[i];
              let noise = (2.0 * g * self.k_b_t / h).sqrt()
                  * standard_normal(&mut *rng);
              data.qfrc_passive[i] += damping + noise;
          }
      }
  }
  ```

  Properties: thread-safe (matches `Send + Sync` bound), reproducible
  (RNG is per-instance, seedable), local (no `Data` mutation needed).
  PRNG choice is recon item 4 — placeholder.
- **Caveats for the spec** (NOT Phase 1 blockers; flagged for later):
  1. **Derivatives / FD perturbation** — `cb_passive` *will* fire inside
     `forward_skip()` because `mj_fwd_passive` is called from
     `forward_acc`, which `forward_skip` calls. For deterministic
     derivative computation in Phase 5, the thermostat needs gating or
     RNG snapshot/restore. Phase 1 doesn't compute derivatives.
  2. **Plugin passive forces fire after `cb_passive`** — not a Phase 1
     issue (no plugins), but worth flagging in the spec.
  3. **Mutex lock contention** — one lock per step is negligible for
     single-env sims. For BatchSim parallel envs, each env needs its
     OWN thermostat instance with its own RNG, not a single shared
     thermostat. The API should make the wrong choice hard.
- **DECISION REVISED**: Item 2's Option A is **superseded**. The Phase 1
  thermostat lives in `cb_passive` and writes into `qfrc_passive` with
  accumulation semantics. The forward-looking Option C (dedicated
  `qfrc_thermostat` field) is no longer needed — `cb_passive` already
  achieves the composability goal. Item 2's log entry remains for
  historical reasoning; the decision-trail is the value, not just the
  current decision.
- **Phase 1 section in The Gap UPDATED in this same commit** to reflect
  the new algorithm. The old text described the `qfrc_applied` approach
  and is no longer accurate. Reading the master plan top-to-bottom now
  yields a consistent design.
- **Item 3 status**: First finding extracted (cb_passive exists — the
  most consequential bit). Item 3 is **otherwise still open**: we still
  need to read existing patterns for writing into `qfrc_applied`
  (`ml-bridge/src/space.rs`, `tests/integration/split_step.rs`,
  `examples/coupled_pendulums.rs`) to understand how RL controllers
  inject forces, even though the thermostat doesn't go through that
  path — the knowledge is relevant for Phase 7 composition. That recon
  happens in a fresh session.
- **Next action (fresh session)**: Continue item 3 — existing
  `qfrc_applied` usage patterns in `ml-bridge/src/space.rs`,
  `tests/integration/split_step.rs`, and `examples/coupled_pendulums.rs`.
  Then proceed to items 4–7 (PRNG conventions, `Data` field public
  mutability, `model.timestep` variability, statistical-test
  conventions, thermostat module location). After all Phase-1-blocking
  items are answered, draft `PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`.

