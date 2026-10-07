> Research for *A Double Dose of Detail*, written during planning by a read-only researcher at `3520544e`. Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid spec — sim-core callbacks and control (core-C1, core-C2, core-C3, core-C4, core-L4, core-B1, ledger-L31's FD item, FD under RK4)

Researcher scratch: `SCRATCH/rigid_spec/core_callbacks/` (SCRATCH = `$SCRATCH`).
Repo: `main` @ `3520544e747f6ba53e863b20b3c292cdeb9bb9ad`, `git status --short` empty before and after this work (read-only; nothing written in the repo).

## How this was checked (and what it cannot see)

| what | how | output file |
|---|---|---|
| MuJoCo 3.5.0 order/counts per entry point × integrator × actuation flag; spring+damper off; nv = 0; sleep | `mujoco==3.5.0` Python, `py/order_counts.py` | `py/order_counts.out` |
| MuJoCo 3.5.0 FD counts (transitionFD centered/forward, sensors on/off; inverseFD), RK4 refusal text, ctrl-writing control callback vs `B`, `C`/`D` semantics | `py/fd_counts.py` | `py/fd_counts.out` |
| MuJoCo 3.5.0 bad-ctrl cases (clamped copy) | `py/l4_ctrl.py` | `py/l4_ctrl.out` |
| the same matrix on our sim-core | one probe source `probe_src/main.rs` built twice: against the repo (`now/`) and against a patched export of HEAD (`after_repo/`, built by `after_probe/`) | `now.out`, `after.out` (before the hybrid fix), `after_hybfix.out` (final) |
| proposed new tests, run on main and on the patch | `probe_src/new_tests.rs` (10 tests) | `nt_now.out`, `nt_after_probe.out`, `nt2_*.out` |
| existing suites on the patch | `cargo test` in `after_repo` (own target dir): sim-core `--lib` (720 pass), `sim-conformance-tests --test integration` (1335 pass, 3 fail, 27 ignored), sim-thermostat + sim-therm-env + sim-ml-chassis `--no-fail-fast` (1 fail), sim-mjcf (all pass), `example-derivatives-stress-test` (31/32) | `t_core_lib.out`, `t_integ.out`, `t_therm.out`, `t_mjcf.out`, `ex_stress_after.out` |
| hook clippy on the patched sim-core | `cargo clippy -p cortenforge-sim-core --all-targets --all-features -- -D warnings`: exit 0; made to fail once with an injected `clone_on_copy` (exit 101), then restored | `clippy_core.out`, `clippy_lie.out` |
| numeric neutrality without callbacks | corpus harness (1,584 docs, model fingerprint + 100-step trajectory fingerprint) built against main ×2 and the patch ×2, compared with the 10 earlier main runs (`rigid_plan_tests/emb_1..10.jsonl`) as a nondeterminism mask | `emb_now_*.jsonl`, `emb_after*.jsonl`, `cmp2.py` |
| BatchSim determinism with a shared stateful callback | `b1/` (sim-core `parallel`), `RAYON_NUM_THREADS` = 1, 1, 8, 8, 8 | console, quoted below |
| prototype patch | `prototype.diff` (7 files, 223 changed lines) | |

What the method cannot see: (a) each proposed commit was NOT built or tested on its own — only the full prototype (all of C-1…C-4 together) and the "no hybrid fix" variant were; (b) `grade`, `doc-theft`, `--release` nextest, licensed gates, the 27 ignored golden tests and the Bevy/example validators other than the derivatives stress test were not run; (c) the corpus check runs no callbacks, only mid-range ctrl, 100 steps, and masks the 71 docs whose build is nondeterministic across processes (ledger-L27); (d) the FD-count formulas are fitted to two models (nv=1,nu=1,na=0 and nv=2,nu=2,na=1) on both engines, not derived for every column class (muscles, ball/free joints were not exercised).

---

## 1. Callback order and counts (core-C1 behaviour, core-C3)

### Now (HEAD)
- Passive forces, `cb_passive` and passive plugins run in the ACCELERATION stage: `forward_acc` calls `mj_fwd_passive` after `mj_fwd_actuation` and `mj_rne` (`sim/L0/core/src/forward/mod.rs:537-541`); the callback is at `forward/passive.rs:722-725`.
- `cb_control` fires in `forward_core` only when `compute_sensors` (so not on RK4 stages) and actuation is enabled (`forward/mod.rs:425-430`); in `step1` ungated (`:146-149`); never in `forward_skip` (`:396-402`, whose comment claims MuJoCo skips it — false, see Target).
- `step2` runs `forward_acc` and therefore the passive callback (`:188`).
- `mj_fwd_passive` returns before the callback when `nv == 0` (`passive.rs:353-356`), and when springs AND dampers are disabled (`:383-385`).

Measured (`now.out`): `forward` = `CP`; `forward_skip(any stage)` = `P`; `step1` = `C` (also with actuation off); `step2` = `P`; `step` Euler/implicit/implicitfast/ISD = `CP`; RK4 `step` = `CPPPP`; nv = 0 `step` = `C`.

### Target (MuJoCo 3.5.0, measured in `py/order_counts.out`)
- Passive runs inside the velocity stage: `mj_fwdVelocity` calls `mj_passive` (`engine_forward.c:250`); the callback is at `engine_passive.c:703-705`, followed by passive plugins (`:707-722`).
- `mj_forwardSkip` fires `mjcb_control` after the velocity stage whenever actuation is enabled, whatever `skipstage` is (`engine_forward.c:1399-1401`) — so control fires on every RK4 stage (`mj_RungeKutta` calls `mj_forwardSkip(m, d, mjSTAGE_NONE, 1)` at `:1100`).
- `mj_step1` fires both, control UNGATED (`:1460-1486`, control at `:1481-1483`); `mj_step2` fires neither (`:1492-1514`). core-C3 is therefore parity, not a bug: no gate is added.
- `mj_passive` has no `nv == 0` return (`engine_passive.c:638-704`); it returns before the callback only when both springs and dampers are disabled (`:658-661`).

Table (P = `cb_passive` calls, C = `cb_control` calls; "act off" = `DISABLE_ACTUATION`). MuJoCo and "now" measured; "after" measured on the prototype.

| entry point | integrator | MuJoCo 3.5.0 (order) | ours now (order) | ours after (order) |
|---|---|---|---|---|
| `forward` | any | 1 P, 1 C (`PC`) | 1, 1 (`CP`) | 1, 1 (`PC`) |
| `forward`, act off | any | 1, 0 | 1, 0 | 1, 0 |
| `forward_skip(None \| Pos, skipsensor any)` | any | 1, 1 (`PC`) | 1, 0 | 1, 1 (`PC`) |
| same, act off | any | 1, 0 | 1, 0 | 1, 0 |
| `forward_skip(Vel, any)` | any | 0, 1 | 1, 0 | 0, 1 |
| same, act off | any | 0, 0 | 1, 0 | 0, 0 |
| `step1` (act on or off) | any | 1, 1 (`PC`) | 0, 1 | 1, 1 (`PC`) |
| `step2` | any | 0, 0 | 1, 0 | 0, 0 |
| `step` | Euler, Implicit, ImplicitFast (ISD: ours only) | 1, 1 (`PC`) | 1, 1 (`CP`) | 1, 1 (`PC`) |
| `step`, act off | same | 1, 0 | 1, 0 | 1, 0 |
| `step` | RK4 | 4, 4 (`PCPCPCPC`) | 4, 1 (`CPPPP`) | 4, 4 (`PCPCPCPC`) |
| `step`, act off | RK4 | 4, 0 | 4, 0 | 4, 0 |
| `step`, springs+dampers off | Euler | 0, 1 | 0, 1 | 0, 1 |
| `step`, nv = 0 | Euler | 1, 1 | 0, 1 | 1, 1 |
| `step` on which a tree falls asleep (`ENABLE_SLEEP`) | Euler | 2, 2 (`PCPC`, step 76) | 1, 1 (fell asleep at step 69) | 1, 1 — **deviation, see Open Q3** |
| `step` after a bad `qacc` auto-reset | any | +1, +1 (`mj_checkAcc` re-runs `mj_forward`, `engine_forward.c:106-108`) | +1, +1 (`check.rs:96-106`) | unchanged (source reading only, not run) |

### Change (exact new call sequence; prototype in `prototype.diff`)
Split the private `Data::forward_pos_vel` (`forward/mod.rs:442-519`) into two private stage functions and one helper; delete the duplicated position block inside `forward_skip` (`:331-377` is a line-for-line copy of `:446-504`, checked with `diff`):

```rust
// forward/mod.rs (all private)
fn forward_pos(&mut self, model: &Model, compute_sensors: bool);   // wake, FK, flex, CRBA, transmissions, collision, sensor_pos, energy_pos (body unchanged)
fn forward_vel(&mut self, model: &Model, compute_sensors: bool);   // mj_fwd_velocity, mj_actuator_length, passive::mj_fwd_passive, sensor_vel, energy_vel
fn fire_control_gated(&mut self, model: &Model);                   // cb_control unless DISABLE_ACTUATION
```
- `forward_core(cs)` = `forward_pos(cs)`; `forward_vel(cs)`; `fire_control_gated`; `forward_acc(cs)` — the `compute_sensors &&` gate at `:425` is dropped, so RK4 stages (`forward_skip_sensors`) fire control.
- `forward_skip(s, ss)` = `if s < Pos {forward_pos(!ss)}`; `if s < Vel {forward_vel(!ss); energy_initial capture as today}`; `fire_control_gated`; `forward_acc(!ss)`.
- `step1` = checks; `forward_pos(true)`; `forward_vel(true)`; `cb_control` ungated (as today).
- `step2`, `step`, `mj_runge_kutta` unchanged in code; their counts change through the functions above.
- `forward_acc`: delete the `passive::mj_fwd_passive` call (`:541`); `mj_gravcomp_to_actuator` (`:545`) stays after actuation (it reads `qfrc_gravcomp`, now computed earlier in the same pass).
- `passive.rs`: extract `fn mj_passive_user(model, data)` (callback, then passive plugins, `:722-739`); the `nv == 0` branch calls it unless springs and dampers are both disabled, then returns. No public signature changes.

Numeric neutrality (no callbacks): passive force code reads no acceleration-stage output (`grep` of `passive.rs` for `qacc|qfrc_bias|qfrc_actuator|actuator_*|ctrl|act|cacc|cfrc|qfrc_smooth|efc_`: no hits; `mj_gravcomp`, `rne.rs:344-390`, reads position-stage fields). Measured: M2 trajectory fingerprints (5 integrators, `step` ×500 and `step1/step2` ×500) identical now vs after; corpus: 0 of the docs that are deterministic across 12 main runs changed model or trajectory fingerprint (1,478 docs with ok trajectories, 71 masked).

**Consequence that needs commit C-1 first:** `forward_skip(Vel)` no longer recomputes `qfrc_passive`. The hybrid path's sensor-only FD columns call `forward_skip(Pos|Vel)` right after a `forward()` at a post-step state (`derivatives/hybrid.rs:3013, 3033, 3078, 3098, 3305, 3321`), i.e. on stale stage caches. On main this already makes hybrid `C` differ from pure-FD `C` (measured, M1 Euler, `jointvel` row: hybrid 0.92626 vs FD 0.98901, `now.out`); with this change and no fix, hybrid `D` became `[-0.1918, -19.18]` against FD `[0.0010988, 0.10988]` (`after.out`), and **no existing test caught it** (`t_integ_nofix.out`: `t4_hybrid_matches_fd_sensor_derivatives` still passes; it uses a 5 % tolerance on C). See section 7.

### Tests to add (all run; "main" = against the repo, "after" = prototype)
In `sim/L0/tests/integration/callbacks.rs`:
- `callbacks_fire_in_mujoco_order_and_count` — the table's rows (5 integrators × actuation on/off; forward, forward_skip ×3 stages ×2 skipsensor, step1, step2, step) on a damped 1-hinge + motor + 2 sensors. Main: FAILS (`forward, Euler act_off=false: left "CP"`). After: passes.
- `passive_callback_fires_on_a_model_without_dofs` — main FAILS (`"C"`), after passes.
- `passive_callback_skipped_when_springs_and_dampers_are_disabled` — pin (passes on both).
- `rk4_reevaluates_the_control_callback_at_each_stage` — PD law as callback vs written into ctrl before each RK4 step: main FAILS (bit-identical, 8.28845361698774208e-1 both), after passes (8.28932973307600962e-1 vs 8.28845361698774208e-1).

### Tests/examples that flip
None from this commit alone in the suites run (sim-core lib, integration, thermostat/therm-env/ml-chassis, sim-mjcf all pass on the full prototype except the flips listed under sections 3 and 5). `sim/L0/tests/integration/callbacks.rs` (8 tests) passes unchanged.

### Downstream
- Only non-test `set_passive_callback` user: sim-thermostat `PassiveStack::try_install` (`thermostat/src/stack.rs:176`); `ising_learner.rs:249,353,936` only test/clear the slot. Only `set_control_callback` user: `sim/L0/tests/integration/callbacks.rs:65,419` (`git grep`, whole workspace incl. examples).
- sim-thermostat: count per step unchanged (Euler/implicit 1; RK4 is refused at install since #995, `langevin.rs:300-306`). Measured with a ctrl-temperature thermostat: 2000-step fingerprints identical now vs after (Euler and ImplicitFast, `fe8c756727139ef4`). What changes: the passive callback now runs BEFORE actuation, so it reads `ctrl` as written. With ch0 = 2.0 and ch1 = NaN, the first `forward` gives noise `[0, 0]` on main (actuation had zeroed every ctrl) and `[59.89, 105.76]` after — the same as with ch1 = 0.5 (`now.out`/`after.out`, "ctrl-temp"). Docs that become false here: `thermostat/src/component.rs:215-216` ("the value sim-core's actuation stage sets it to") and the test doc `langevin.rs:644-645`.
- sim-therm-env: Euler (`therm-env/src/builder.rs:30`), at most one ctrl channel (`:333`), observations qpos/qvel only (`:355-358`) — the other-channel case cannot arise; its tests pass on the prototype.
- sim-ml-chassis: no callbacks; `SimEnv::step`'s extra `forward()` (`ml-chassis/src/env.rs:145`) keeps firing passive twice per step (Envs PR); tests pass.
- plugin passive dispatch moves with `mj_fwd_passive` into the velocity stage (MuJoCo: same function, `engine_passive.c:707-722`); test comment `sim/L0/core/src/plugin.rs:557-558` becomes stale.

### Open questions
See Q3 (sleep re-forward).

---

## 2. Docs for core-C1, core-C2, core-C3 (callback timing)

Replace `CbPassive`/`CbControl` docs (`sim/L0/core/src/types/callbacks.rs:37-46`):

```rust
/// Passive force callback, the counterpart of MuJoCo's `mjcb_passive`.
///
/// Runs at the end of the passive-force computation in the VELOCITY stage, after
/// `qfrc_spring`, `qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid` and their sum
/// `qfrc_passive` are written and before passive plugins. Add custom forces to
/// `qfrc_passive`. It runs before [`CbControl`] in the same pass, so it reads `ctrl`
/// as the caller left it; actuator forces, `qfrc_bias`, constraint forces and `qacc`
/// have not been computed for this pass yet.
///
/// # When it runs (as MuJoCo 3.5.0)
///
/// - once per [`Data::forward`], [`Data::step1`], and
///   [`Data::forward_skip`] with [`MjStage::None`] or [`MjStage::Pos`];
/// - once per [`Data::step`] with Euler and the implicit integrators, four times
///   with RK4 (once per stage);
/// - never in [`Data::step2`] or `forward_skip(MjStage::Vel, _)`;
/// - **never while both `DISABLE_SPRING` and `DISABLE_DAMPER` are set**: passive
///   forces are skipped as a whole, callback and passive plugins included
///   (MuJoCo's `mj_passive` does the same). A component that must always act — a
///   thermostat — goes silent under those two flags;
/// - also on a model with no degrees of freedom;
/// - inside finite-difference derivatives, many times per call (counts in the
///   [`derivatives`](crate::derivatives) module docs).
///
/// Not matched: on the step that puts a tree to sleep, MuJoCo re-runs the forward
/// pass once more (both callbacks fire twice); sim-core does not.
pub type CbPassive = …;

/// Control callback, the counterpart of MuJoCo's `mjcb_control`.
///
/// Runs after the velocity stage (after [`CbPassive`]) and before actuation. Set
/// `ctrl` (or `qfrc_applied` / `xfrc_applied`) here. The actuation stage reads a
/// clamped copy of `ctrl`; `ctrl` keeps what the callback wrote.
///
/// # When it runs (as MuJoCo 3.5.0)
///
/// - once per [`Data::forward`] and per [`Data::forward_skip`] (every
///   [`MjStage`]), unless `DISABLE_ACTUATION` is set;
/// - in [`Data::step`]: once with Euler and the implicit integrators, four times
///   with RK4 — each RK4 stage calls it at that stage's trial state and time, so a
///   state-dependent controller is re-evaluated (unless `DISABLE_ACTUATION`);
/// - once per [`Data::step1`], **even with `DISABLE_ACTUATION` set** (MuJoCo's
///   `mj_step1` does not check the flag); never in [`Data::step2`];
/// - inside finite-difference derivatives. A callback that overwrites `ctrl`
///   overwrites the control perturbation too, so finite-difference `B` comes back
///   zero, as in MuJoCo (see the [`derivatives`](crate::derivatives) docs).
pub type CbControl = …;
```

Two sentences above are true only once C-4 lands — CbPassive's "reads `ctrl` as the caller left it" (between C-2 and C-4 the actuation stage still zeroes a bad `ctrl` after passive runs, so the next pass reads zeros) and CbControl's "reads a clamped copy … `ctrl` keeps what the callback wrote": put them in C-4.

Other doc sites this commit must rewrite (every one re-read at HEAD; each states the old stage placement or the old counts):
`forward/mod.rs:7-19` (module "Pipeline Split"), `:99-131` (`step1`: fires both callbacks), `:154-178` (`step2`: no callbacks; passive no longer in its stage list), `:269-283` (`forward` stage list: passive + `cb_passive` in stage 2, `cb_control` between 2 and 3), `:296-321` (`forward_skip`: per-stage callback behaviour), **`:396-401` (the false "MuJoCo's mj_forwardSkip skips mjcb_control" comment — delete)**, `:406-412` (`forward_core`), `:435-441`, `:521-528` and `:538-539` (`forward_acc` "passive" mentions); `MjStage` doc `:78-96` (Vel also skips passive forces); `forward/actuation.rs:471-472`; `forward/passive.rs:352-356` (the "S4.7a nv == 0 guard" comment); `integrate/rk4.rs:16-17` (stages fire both callbacks); `types/model.rs:1015-1018` (field docs → "see [`CbPassive`]"/"[`CbControl`]"); `plugin.rs:557-558`; `thermostat/src/component.rs:215-216` → "A bad value … reads as 0." (drop "the value sim-core's actuation stage sets it to"); `thermostat/src/langevin.rs:644-645`; `sim/docs/MUJOCO_REFERENCE.md:15-37` and `sim/docs/ARCHITECTURE.md:270-292` (pipeline listings place `mj_fwd_passive` after `mj_rne`).

---

## 3. Finite differences refuse RK4

### Now
Public FD entry points and their error type (all `Result<_, StepError>`): `mjd_transition_fd` (`derivatives/fd.rs:61-65`), `mjd_inverse_fd` (`fd.rs:476-480`), `mjd_transition_hybrid` (`hybrid.rs:2350-2354`), `mjd_transition` (`mod.rs:222-226`), `Data::transition_derivatives` (`mod.rs:270-274`), `validate_analytical_vs_fd` (`mod.rs:321`, via `mjd_transition`), `fd_convergence_check` (`mod.rs:352`, via `mjd_transition_fd`). RK4 runs: `mjd_transition` routes it to pure FD (`mod.rs:241-249`), `mjd_transition_hybrid` too (`hybrid.rs:2530-2534`). Measured `Ok` for all five callable entry points under RK4 (`now.out`). The parameter Jacobians (`param.rs` `mjd_damping_jacobian` etc.) already panic for any non-Euler integrator by `assert` (`param.rs:86-90` `assert_damping_scope`, and siblings) — unchanged. `StepError` is `#[non_exhaustive]`, `Copy + Eq` (`types/enums.rs:824-835`); `Integrator` is `Copy + Eq + Hash` (`:801-803`).

### Target
MuJoCo 3.5.0 refuses: `mjd_transitionFD` "RK4 integrator is not supported" (`engine_derivative_fd.c:544-546`), `mjd_inverseFD` the same (`:614-616`). Measured: Python raises `FatalError: mjd_transitionFD: RK4 integrator is not supported` / `mjd_inverseFD: …` (`py/fd_counts.out`). (`mjd_stepFD` is not in `mujoco.h`; it runs RK4 ignoring `skipstage`, `:133-136`.)

### Change
```rust
// types/enums.rs — StepError gains (additive: the enum is #[non_exhaustive])
/// Finite-difference derivatives refuse this integrator (MuJoCo 3.5.0
/// `mjd_transitionFD` / `mjd_inverseFD`: "RK4 integrator is not supported").
UnsupportedIntegrator { integrator: Integrator },
// Display: "finite-difference derivatives do not support the {integrator:?} integrator"

// derivatives/mod.rs
pub(crate) fn check_fd_integrator(model: &Model) -> Result<(), StepError>;  // Err for RungeKutta4
```
Called FIRST (before the `eps`/`nhistory` asserts) in `mjd_transition_fd`, `mjd_transition_hybrid`, `mjd_inverse_fd`; the others inherit it. The RK4 arm at `hybrid.rs:2530-2534` becomes unreachable (make it `unreachable!` like `:2670`, or delete the arm's FD fallback). Measured after: all five entry points return `Err(UnsupportedIntegrator { integrator: RungeKutta4 })`, zero callbacks fired before the refusal (`after.out`).

### Tests to add
`finite_differences_refuse_rk4` (integration `derivatives.rs`): the five entry points × `use_analytical` both → `Err(UnsupportedIntegrator { integrator: RungeKutta4 })`. Main: FAILS; after: passes.

### Tests/examples that flip (CI-run unless noted)
- `sim/L0/tests/integration/derivatives.rs:451` `test_integrator_coverage_rk4` (asserts `is_ok`) → rewrite to assert the refusal.
- `sim/L0/tests/integration/derivatives.rs:464` `test_rk4_differs_from_euler` (unwraps RK4 FD) → delete (its claim has no subject once RK4 is refused).
- Validator example `examples/fundamentals/sim-cpu/derivatives/stress-test/src/main.rs:757-772,786-788` (check 23, run by validate-examples): measured `31/32 … 1 FAILED`, exit 1, on the prototype (`ex_stress_after.out`) → change check 23 to "RK4 refused with `UnsupportedIntegrator`"; README row `stress-test/README.md:35`.
- Docs: `derivatives/mod.rs:7-8` ("works with any integrator"), `:168-172`, `:206-210`, `:241-249`; `hybrid.rs:2336`; `examples/fundamentals/sim-cpu/derivatives/README.md:12`; `sim/docs/MUJOCO_REFERENCE.md:857-858` ("Handles any integrator including RK4").

### Downstream
In-tree FD callers (`git grep`) never pass RK4 outside the two tests and the validator above: `sim/L1/coupling/src/articulated.rs:407` returns early unless Euler before `transition_derivatives` (`:424`, `:546`); `param.rs` trajectory Jacobians assert Euler first. External callers matching `StepError` already need a wildcard arm (`#[non_exhaustive]`).

### Open questions
Q1 (variant shape), Q2 (`nhistory` panics).

---

## 4. Finite-difference callback counts (documented, not matched) — core-C1's FD half, ledger-L31's second item

Measured (`py/fd_counts.out`, `now.out`, `after_hybfix.out`) on M1 (nv 1, na 0, nu 1) and M2 (nv 2, na 1, nu 2), Euler and ImplicitFast (identical counts). Formulas below reproduce all measured cells; they were not checked on other column classes.

Proposed `## Callbacks during finite differences` section for the `derivatives` module docs (`derivatives/mod.rs:1-55`), after the prototype's behaviour:

```text
Finite differences step or forward the model many times, and every pass fires the
model's callbacks (cb_passive, cb_control, sensor/actuator callbacks). Calls per
derivative call, with nv dofs, na activations, nu actuators (cb_control counts drop
to 0 under DISABLE_ACTUATION):

| function                                   | cb_passive and cb_control each      |
|--------------------------------------------|-------------------------------------|
| mjd_transition_fd, centered                | 2·(2nv + na + nu) + 1               |
| mjd_transition_fd, forward differences     | (2nv + na + nu) + 1                 |
|   … with compute_sensor_derivatives        | twice the above                     |
| mjd_transition / transition_derivatives    | 1 + (columns done by FD)·(2 centered, 1 forward); |
|   (hybrid, the default)                    | with sensor derivatives, as mjd_transition_fd |
| mjd_inverse_fd, centered                   | 4nv + 1                             |
| mjd_inverse_fd, forward differences        | 2nv + 2                             |

MuJoCo 3.5.0 makes fewer calls, because its control and velocity columns skip
stages: mjd_transitionFD centered fires cb_passive 4nv + 1 and cb_control
4nv + 2na + 2nu + 1 times (forward: 2nv + 1 and 2nv + na + nu + 1), with or without
sensor derivatives; mjd_inverseFD fires cb_passive 2nv + 1 times and cb_control
never. sim-core's sensor derivatives re-run forward() after every step to read the
sensors at the stepped state, which doubles the count.

Two consequences:
- A stochastic passive callback draws noise on every one of these calls, so the
  derivatives are noisy. Wrap the call in sim-thermostat's
  PassiveStack::disable_stochastic.
- A control callback that writes ctrl overwrites the control perturbation: the
  finite-difference B columns come back zero (MuJoCo: the same, measured). The
  hybrid path's analytic B columns do not run the callback and are not zeroed.
```

Measured values behind it: our pure FD M1 centered 7/7, forward 4/4, ×2 with sensors (14, 8); M2 15/15, 8/8, 30, 16 (now = after for non-RK4). Hybrid M1 1/1 (both diff modes), M2 3/3 centered, 2/2 forward; with sensors M1 14/8 now → 14/14 after, M2 30/18 now → 30/30 after. Inverse FD M1 5/0 now → 5/5 after (centered), 4/0 → 4/4 (forward); M2 9/0 → 9/9, 6/0 → 6/6. MuJoCo: transitionFD M1 5/7 (centered), 3/4 (forward); M2 9/15, 5/8; sensors change nothing; inverseFD M1 3/0, M2 5/0. Ctrl-writing callback: MuJoCo B = `[0, 0]`; ours pure FD `[0, 0]` (now and after), hybrid `[0.0011110, 0.11110]` with and without the callback.

Also fix `fd.rs:29-33` and `sim/docs/MUJOCO_REFERENCE.md:857`: centered cost is `2·(2nv+na+nu) + 1` steps (the nominal step runs unconditionally, `fd.rs:99`), measured as the callback count above.

Test (pin, passes on main and after): `a_ctrl_writing_control_callback_zeroes_fd_b`.

Intentional-divergence rows for `sim/docs/MUJOCO_CONFORMANCE.md:321` from this section: "FD callback counts" (above) and "sensor derivatives read sensors at the stepped state" — the latter is Open Q4, not decided here.

---

## 5. core-L4 — bad-ctrl check on the clamped copy, `ctrl` left as written

### Now
`mj_fwd_actuation` checks the RAW `data.ctrl`, warns, and zeroes `data.ctrl` itself (`forward/actuation.rs:487-497`); clamping happens afterwards per actuator (`:506-511`). Measured (`now.out`, 3 actuators: motor limited ±1, motor unlimited, filter limited ±1): `ctrl[0] = 1e11` on the limited motor → every `ctrl` zeroed, `BadCtrl` count 1, all forces 0; `inf` on the limited motor → the same; NaN → `ctrl` zeroed; the warning count stays 1 after a following `step` (ctrl is now 0).

### Target (MuJoCo 3.5.0 `mj_fwdActuation`, `engine_forward.c:297-319`)
A local copy of `ctrl` (`:297-305`; delayed actuators read the history buffer there), clamped unless `mjDSBL_CLAMPCTRL` (`:307-310`, only `ctrllimited` actuators), checked with `mju_isBad` (NaN or |x| > 1e10, `engine_util_misc.c:1608-1610`); on the first bad value `mj_warning(d, mjWARN_BADCTRL, i)` and the COPY is zeroed (`:312-319`). `d->ctrl` is never written. Measured (`py/l4_ctrl.out`):

| case | MuJoCo 3.5.0 | ours now | ours after |
|---|---|---|---|
| 1e11 on limited | no warning, force `[1, 0.5, 0]`, act_dot 5, ctrl kept | warn, ctrl zeroed, force 0 | = MuJoCo |
| inf on limited | no warning, force `[1, 0.5, 0]`, ctrl kept | warn, zeroed | = MuJoCo |
| NaN on limited | warn (info 0), force 0, act_dot 0, ctrl kept NaN; count 2 after `forward`+`step` | warn, zeroed, count 1 | = MuJoCo |
| 1e11 on unlimited | warn (info 1), force 0, ctrl kept | warn, zeroed | = MuJoCo |
| 1e11 on limited, `DISABLE_CLAMPCTRL` | warn, force 0, ctrl kept | warn, zeroed | = MuJoCo |
| NaN on unlimited | warn (info 1), force 0, ctrl kept | warn, zeroed | = MuJoCo |

In every case `divergence_detected()` stays false (it reads only `BadQpos/BadQvel/BadQacc`, `types/data.rs:923-927`); no reset; `time` advances.

### Change
`forward/actuation.rs` only, no public signature change:
```rust
/// The control input actuator `i` acts on: `data.ctrl[i]` clamped to its
/// `ctrlrange` unless DISABLE_CLAMPCTRL is set.
fn actuator_ctrl_input(model: &Model, data: &Data, i: usize) -> f64;
```
`let bad = (0..nu).find(|&i| is_bad(actuator_ctrl_input(model, data, i)));` → `mj_warning(data, Warning::BadCtrl, i)` once; per actuator `let ctrl = if bad.is_some() { 0.0 } else { actuator_ctrl_input(model, data, i) };` (replaces `:487-511`). No allocation, no new `Data` field. sim-core then never writes `data.ctrl` during a step (other writers: `Data::reset`, keyframes).

Docs: `StepError` (`types/enums.rs:820-823`) — the "instead of silently correcting issues" sentence is false (auto-reset stays, by decision): "Errors that stop a step. A bad state is NOT an error: a NaN, ±inf or |x| > 1e10 in `qpos`, `qvel` or `qacc` resets the `Data` (unless `DISABLE_AUTORESET`) and the step returns `Ok` — check [`Data::divergence_detected`]; a bad control (after clamping to `ctrlrange`) makes every actuator act on 0 for that pass, leaves `ctrl` as written, and counts [`Warning::BadCtrl`] (not part of `divergence_detected`). Both match MuJoCo." Same sentence in `Data::step` (`forward/mod.rs:222-224`).

### Tests to add
`bad_ctrl_check_runs_on_the_clamped_input_and_leaves_ctrl_alone` (integration `runtime_flags.rs`): 1e11 on a ±1 motor → count 0, forces `[1.0, 0.5]`, ctrl kept; NaN → count 1, forces 0, `ctrl[0]` NaN, `ctrl[1]` 0.5, no divergence. Main: FAILS (`left: 1`); after: passes.
sim-thermostat: `thermostat_reads_its_own_channel_when_another_is_bad` (main FAILS: noise 0; after passes). Note it already passes after section 1's commit alone by reasoning (passive then reads ctrl before actuation in a single `forward`), so it belongs with section 1; a two-`forward` variant would pin section 5 (not written).

### Tests that flip (CI-run)
- `sim/L0/tests/integration/runtime_flags.rs:718` `ac29_ctrl_validation` (`ctrl[i] == 0.0`, `:726-732`). New expectation: `data.ctrl[0].is_nan()`; every `actuator_force` 0; `warnings[BadCtrl].count > 0`; `!divergence_detected()`. Header comment `:714`.
- `sim/L0/thermostat/src/langevin.rs:647` `a_bad_control_is_read_as_zero_with_actuation_on_or_off`: fails at `:665` (`data.ctrl[0].is_nan() == disable_actuation`; measured left true, right false). New expectation: `data.ctrl[0].is_nan()` for both flags; `qfrc_passive[0] == -0.1` unchanged.

### Downstream
`clamped_ctrl` (`thermostat/src/component.rs:217-223`) keeps reading a bad value as 0 — behaviour unchanged for its own channel. sim-ml-chassis: an observation that includes `ctrl` (`ml-chassis/src/space.rs:79`) now reports a NaN action as NaN on later steps instead of 0 (MuJoCo behaviour); `ActionSpace` clamping (`space.rs:643`) passes NaN through (Envs PR). No in-tree code reads `data.ctrl` expecting the sanitised value (`git grep` of `.ctrl` in sim crates; not exhaustive beyond the sim crates and examples).

Not in this item, found while reading: our actuation never reads the ctrl history buffer, so actuators with `actuator_delay > 0` act on the current `ctrl` (MuJoCo 3.5.0 `engine_forward.c:301-305`). Found by `grep` only (`history` appears in sim-core only in reset/init, `types/data.rs:1108,1277`, `model_init.rs:521`); not measured. The clamped-input helper is where it would go.

---

## 6. core-C4 — setters replace; chaining by clone (decision 12)

### Now
Seven setters (`types/model.rs:1251-1357`) each overwrite one `Option<Callback<…>>` slot with no doc saying so. `Callback<F: ?Sized>(pub Arc<F>)` with manual `Clone` (`types/callbacks.rs:21-27`); `Model.cb_*` are pub fields (`model.rs:1015-1028`). Chaining by cloning works today (measured: `chaining_a_second_passive_callback_by_cloning_the_first` passes on main and after; the earlier reviewer's probe: "first=1 second=1").

### Target / Change (docs only)
On `set_passive_callback` (the others get the same first paragraph and a pointer to this example; the return-value callbacks add "you decide how to combine the two results"):

```rust
/// Set the passive force callback, replacing any callback already set.
///
/// There is one slot. To run two callbacks, clone the current one out of
/// [`Model::cb_passive`] and call it from the new closure:
///
/// ```
/// # use sim_core::Model;
/// let mut model = Model::n_link_pendulum(1, 1.0, 0.1);
/// model.set_passive_callback(|_, data| data.qfrc_passive[0] += 1.0);
/// let first = model.cb_passive.clone();
/// model.set_passive_callback(move |m, data| {
///     if let Some(cb) = &first {
///         (cb.0)(m, data);
///     }
///     data.qfrc_passive[0] += 2.0;
/// });
/// ```
```
What this freezes as public API: the pub `cb_*` fields and their `Option<Callback<dyn Fn(...) + Send + Sync>>` types; `Callback`'s pub tuple field `.0` and its `Arc` representation; `Callback: Clone` sharing the closure (clones share captured state — the same fact B1 documents). Interaction to state in sim-thermostat (not core): `PassiveStack::try_install` refuses a model whose slot is set (`stack.rs:154`), so chain AFTER installing a stack.

Test: the example as a doctest (it ran green as a test on both, `nt2_*.out`).

---

## 7. NEW (not a triage row) — hybrid sensor-only FD columns use stale stage caches

### Now
`mjd_transition_hybrid`'s sensor-only columns run `forward_skip(Pos|Vel, false)` (`hybrid.rs:3013, 3033, 3078, 3098, 3305, 3321`) on a scratch whose previous column ended with `forward()` at a post-step state, so skipped stages hold that state's results. Measured on main: hybrid `C`/`D` differ from pure-FD `C`/`D` for all four non-RK4 integrators (fingerprints, `now.out`; M1 Euler `jointvel` row 0.92626 vs 0.98901). `t4_hybrid_matches_fd_sensor_derivatives` (`derivatives.rs:2208`) allows 5 % on C and uses a different model; it passes on main and on the prototype without the fix.

### Change
Those six calls use `MjStage::None`. Measured: hybrid `C` and `D` become bit-identical to pure FD for Euler, Implicit, ImplicitFast, ISD (`after_hybfix.out`); A/B unchanged. Cost: full pipelines in those columns (time not measured).

### Test
`hybrid_sensor_derivatives_equal_pure_fd` (`assert_eq!` on `C` and `D`, damped hinge, 4 integrators): main FAILS (`C, Euler`), prototype-without-fix fails on D (by the `after.out` values), with the fix passes. Ledger: this needs a row (Jon / the planner decides where).

---

## 8. core-B1 — determinism claims with shared callback state

### Now
`BatchSim::step_all` promises "Output is independent of thread count and scheduling order" (`sim/L0/core/src/batch.rs:232-236`); `callbacks.rs:6-8` says "`Fn` (not `FnMut`) is thread-safe with immutable captures". Measured (`b1/`, `parallel` feature, 16 envs sharing one model with a `LangevinThermostat` stack, 200 steps): `RAYON_NUM_THREADS=1` twice → identical (`ac7c4321b80fda4c`); `=8` three times → three different results (`442923099e21764f`, `3b0fad47909f2404`, `bcf5108973201634`).

### Change (docs only)
`batch.rs` `# Determinism`:
```rust
/// # Determinism
///
/// Each environment's step reads its own [`Data`] and the shared [`Model`]. The
/// output is independent of thread count and scheduling order when the model's
/// callbacks are pure functions of `(&Model, &Data)`. A callback with shared
/// mutable state — a counter or RNG in the closure, such as a `sim-thermostat`
/// `PassiveStack` installed on the shared model — is called from every
/// environment in the order the threads reach it, so results then depend on
/// scheduling (measured: 8 threads, three runs, three results). Give each
/// environment its own callback state with [`BatchSim::new_per_env`].
```
`callbacks.rs:6-8`: "`Fn + Send + Sync`: callable from several threads at once (BatchSim steps environments in parallel over one shared `Model`). Interior mutability (an atomic counter, an RNG behind a `Mutex`) is allowed, but what it returns then depends on the order threads call it — see `BatchSim::step_all`. `Model::clone` shares the closure and its state." If core-B3 adds `BatchSim::forward_all`, the same paragraph applies to it.

---

## Commit list (this area), in order

| # | commit | carries tests | flips |
|---|---|---|---|
| C-1 | `fix(sim-core): hybrid sensor derivatives run every stage` (section 7) | `hybrid_sensor_derivatives_equal_pure_fd` | none measured (t4 still passes; consider tightening it to `assert_eq!`) |
| C-2 | `feat(sim-core)!: callbacks fire in MuJoCo's order and counts` (section 1 code + section 2 docs + thermostat doc lines `component.rs:215-216`, `langevin.rs:644-645`) | `callbacks_fire_in_mujoco_order_and_count`, `passive_callback_fires_on_a_model_without_dofs`, `passive_callback_skipped_when_springs_and_dampers_are_disabled`, `rk4_reevaluates_the_control_callback_at_each_stage`, `thermostat_reads_its_own_channel_when_another_is_bad` | none (measured on the full prototype) |
| C-3 | `feat(sim-core)!: finite differences refuse RK4` (section 3) | `finite_differences_refuse_rk4` | `derivatives.rs:451`, `:464`; validator stress-test check 23 + README; docs listed |
| C-4 | `fix(sim-core)!: the bad-ctrl check reads the clamped input and leaves ctrl alone` (section 5) | `bad_ctrl_check_runs_on_the_clamped_input_and_leaves_ctrl_alone` | `runtime_flags.rs:718` ac29; `thermostat/src/langevin.rs:647` |
| C-5 | `docs(sim-core): finite-difference callback counts, setters replace, BatchSim determinism` (sections 4, 6, 8 + divergence-table rows) | doctest of section 6; `a_ctrl_writing_control_callback_zeroes_fd_b` | none |

C-1 must precede C-2 (C-2 without it ships wrong hybrid `D` with CI green, measured). C-3 and C-4 are independent of C-2 in code (C-4's thermostat test flip does not depend on C-2: the `is_nan` assertion flips only when actuation stops writing `ctrl`). C-5 documents the counts after C-2 and C-3. Each commit compiling green on its own is NOT verified (only the union and the union-minus-C-1).

## Dependencies on the sim-core state/lifecycle area (core-L1/L2/L3/L5/L6/L7, B2/B3, D1–D4)

- **Pre-step check (timestep + core-L3 lengths)** edits the tops of `step`, `step1`, `step2`, `forward`, `forward_skip` (`forward/mod.rs`); C-2 restructures `forward_skip`/`forward_core`/`step1` bodies. Textual conflict only; land theirs first (design order) and rebase C-2.
- **core-L1 `energy_initial`**: the capture lives only in `forward_skip`'s velocity block (`forward/mod.rs:388-390`); C-2 keeps it exactly there (behaviour-neutral). If L1 moves the capture, `forward_vel` is the single place both `forward` and `forward_skip` pass through.
- **core-L6 ISD `forward()` purity** edits `forward_acc`/`mj_fwd_acceleration`; C-2 deletes one line of `forward_acc`. Related measurement for them: under ISD, pure-FD `A` differs with sensor derivatives on vs off (fingerprints `c64c4f82b447a287` vs `493b5f30ebd27e07`, `now.out`), because the sensor pass's `forward()` writes `qvel`.
- **core-L7 plugins**: passive plugins move with `mj_fwd_passive` into the velocity stage (as MuJoCo).
- **StepError**: C-3 adds `UnsupportedIntegrator`; the other area may add variants (non_exhaustive, additive). Agree on naming together.
- **core-B3 `forward_all`**: B1's doc must cover it if added.
- **Sleep** (ledger-L31 sleep panic, core-L5 commit): Open Q3.

## Open questions (only what the fixed decisions do not settle)

**Q1 — shape of the RK4 refusal.** (a) `StepError::UnsupportedIntegrator { integrator }` (prototype; additive; the derivative functions already return `StepError`). (b) A new `#[non_exhaustive] DerivativeError` for the derivative API (breaking: 7 public signatures). Recommend (a). If wrong: later derivative-only refusals (Q2, MuJoCo's noslip refusal in `mjd_inverseFD`, `engine_derivative_fd.c:618-620`) pile variants into an enum `step()` never returns; switching to (b) after 0.10 is a second break.

**Q2 — `nhistory > 0` in FD panics today** (`fd.rs:72-76`, `hybrid.rs:2361-2364`), where MuJoCo errors ("delays are not supported", `engine_derivative_fd.c:547-549`). Convert to an `Err` in C-3 (same check function) or leave the panic. Recommend convert (the ledger-L19 precedent: Result over panic). If wrong: one more variant frozen for a case nobody hits in-tree (0 FD callers with history, by `grep`).

**Q3 — sleep re-forward.** MuJoCo re-runs `mj_forwardSkip(m, d, mjSTAGE_POS, 0)` inside `mj_advance` when trees fall asleep (`engine_forward.c:899-905`), so that step fires both callbacks twice; ours sleeps after integrating (`forward/mod.rs:253-259`) and never re-forwards (measured `PC` every step; MuJoCo `PCPC` on the sleep step). Matching means moving sleep inside the integrators' advance — a sleep-pipeline change (sleep timing already differs: step 69 ours vs 76 MuJoCo on the same model, not chased). Recommend: document as a deviation now (text in section 2), leave the restructure to a sleep item. If wrong: a callback user with `ENABLE_SLEEP` sees one fewer call per sleep transition until the sleep work lands.

**Q4 — sensor-derivative semantics.** Ours `C = ∂sensordata_{t+1}/∂x_t` (documented, `derivatives/mod.rs:124-125`; measured M1 Euler `jointpos` row `[0.99946, 0.00989]` = A's first row); MuJoCo's `C` is the sensors computed during the step, at the current state (measured `[1.0, 0.0]`, `D = 0`). Not one of my rows; it decides whether FD needs the extra `forward()` per column (the doubled counts in section 4). Recommend raising it to Jon as its own parity item; this spec documents the current semantics.
