> Research for *A Double Dose of Detail*, written during planning by a read-only researcher at `3520544e`. Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid spec — area `core_state`: sim-core state, lifecycle, BatchSim, docs

Researcher section, 2026-10-06. Repo `main` @ `3520544e` (clean before; re-checked after, see the end). MuJoCo citations are **3.5.0** (`SCRATCH/mj350src/mujoco`, tag 3.5.0 / 881544c) unless marked.

## How this section was built (and what it rests on)

- **Measured on main** with a scratch probe against the repo crates (`core_state/probe_main`, outputs `core_state/out_main_*.txt`).
- **Measured as a patch.** Every code change below was applied to a scratch copy of `sim-core`, `sim-mjcf`, `sim-urdf`, `sim-thermostat`, `sim-ml-chassis`, `sim-conformance-tests` (`core_state/ws_fix`, own `git` log per commit; the unpatched twin is `core_state/ws_base` with its **own** target dir). The full patch against the repo files is `core_state/core_state_measured.patch` (22 files). Each step was run against: sim-core lib (717 tests), sim-mjcf lib (385), sim-conformance-tests `integration` (1338 + 27 ignored) and `mujoco_conformance` (83), sim-thermostat lib (250 → 251), sim-ml-chassis lib (361), 25 headless CI validators (`examples/**/stress-test` whose deps are within sim-core/sim-mjcf/sim-urdf/sim-ml-chassis/sim-types/cf-geometry: 23 byte-compared base vs fix at the final patch, plus `integrators/stress-test` and `solvers/stress-test` compared at commit 3 and again at the final patch), and `cargo clippy -p sim-core -p sim-thermostat -p sim-mjcf --all-targets --all-features -- -D warnings` with the repo's `clippy.toml` (exit 0) and `cargo fmt --check` (exit 0 after formatting).
- **"Fails on main, passes after"** is measured, not argued: `core_state/spec_tests_src/tests/common_api.rs` (14 tests) compiled against both copies — **12 fail on main and pass on the fix; 2 pass on both** (pins, said where). `new_api.rs` (7 tests) uses the new API, so it compiles only against the fix, where all 7 pass. Logs: `core_state/spec_base.log`, `core_state/spec_fix.log`.
- **⚠ Method trap I hit and corrected:** cargo hashes path packages relative to the workspace root. Two copies of the workspace sharing one `CARGO_TARGET_DIR` reuse each other's artifacts ("Fresh"). Two early "base" runs had read the fix's build, and both were discarded and re-run with `target_base`. Every base number below comes from the separate target dir.

---

## 1. One pre-step check on every `Result` entry point (decisions 7 + 21; core-L3; ledger-L24 core half; core-D3 Display)

**Now**
- The `Result`-returning pipeline entry points on `Data`: `step` (`forward/mod.rs:225`), `step1` (`:133`), `step2` (`:179`), `forward` (`:284`), `forward_skip` (`:322`). All return `Result<(), StepError>`. `forward_skip_sensors` (`:292`) is `pub(crate)`, called only by RK4 stages inside `step`. `Data::integrate` (`integrate/mod.rs:39`) and `Data::inverse` (`inverse.rs:33`) return `()`.
- Timestep: only `step`/`step1` check it (`:226-229`, `:134-137`). **Measured** (`out_main_timestep.txt`) at h ∈ {0, −1e-3, NaN, ∞}: `forward`, `forward_skip` and `step2` return Ok. `step2` moves time 0 → −0.001 at h = −1e-3 and to NaN/∞ at NaN/∞. `integrate` does the same.
- Shape: nothing ties a `Data` to its `Model`. **Measured** (`out_main_shape.txt`). With `a` = `n_link_pendulum(2)` and `b` = the same plus `add_ground_plane()`, the two have identical nq/nv/na/nu/nbody and different ngeom. `data(a).step(b)` panics at `forward/position.rs:186` (index out of bounds), while `data(b).step(a)` returns Ok. `data(pend2).step(pend3)` panics at `forward/check.rs:24`. `data(pend3).step(pend2)` panics in nalgebra `copy_from`. `data(pend2).inverse(pend3)` panics at `forward/acceleration.rs:403`. A user-assigned `qpos` of the wrong length panics at `check.rs:24`. A `ctrl` longer than `nu` steps Ok. ⇒ **a 5-field check (qpos/qvel/act/ctrl/xpos) is not sufficient**: the ngeom case passes it and still panics.
- `StepError` (`types/enums.rs:820-836`): `#[derive(Debug, Clone, Copy, PartialEq, Eq)]`, `#[non_exhaustive]`, unit variants. Display of `InvalidTimestep` is "timestep is zero or negative" (`:846`), which is also printed for NaN/∞ (core-D3).
- Consumers needing derives: `vec![None; n]` of `Option<StepError>` (`ml-chassis/src/vec_env.rs:168`, needs Clone). `assert_eq!` against `StepError` (`tests/integration/implicit_integration.rs:1031`, PartialEq). No workspace type that derives `Eq` contains a `StepError` (git grep).

**Target.** The five entry points refuse, before any work, a timestep that is not positive and finite, and a `Data` whose arrays do not fit the model.
- Deviation, **stricter**. MuJoCo 3.5.0 has no step-time timestep check: `mj_step` (`engine_forward.c:1422-1457`) calls only `mj_checkPos/Vel/Acc`. The only timestep check that a grep of `src/` for `timestep` next to an error, a warning or `<= 0` finds is `_resetData`'s "history buffers require positive timestep" (`engine_io.c:1268-1270`); a check written without the word `timestep` would not show up in that grep. **Measured** on `mujoco==3.5.0` (`core_state/mj_timestep.py`): `mj_step` runs at h = 0 (time stays 0) and at h = −0.001 (time −0.01 after 10 steps, no warning). At h = NaN, the model loads with "XML contains a NaN" and stepping makes time and qpos NaN.
- Shape deviation, **stricter**. MuJoCo stores `d->signature = m->signature` (`engine_io.c:1545`), and no engine code reads `d->signature` (grep of `src/`; the only `signature` comparisons are spec-vs-model in `user_model.cc:5209,5368`). **Measured**: the Python `mj_step(b, data(a))` (a has nq 1, b has nq 2) returned without error, i.e. unchecked access.

**Change** (measured patch, `forward/check.rs`, `forward/mod.rs`, `types/enums.rs`)
```rust
// types/enums.rs — StepError keeps #[derive(Debug, Clone, Copy, PartialEq, Eq)] + #[non_exhaustive]
/// `model.timestep` is not positive and finite (zero, negative, NaN or infinite).
InvalidTimestep,
/// A `Data` array does not have the length `model` requires: the `Data` was made by
/// another model, or the caller resized one of its arrays.
DataShapeMismatch { field: &'static str, expected: usize, actual: usize },
// Display: "timestep must be positive and finite";
//          "data.{field} has length {actual}, but the model needs {expected}: the Data was not made by this model"

// forward/check.rs (module stays pub(crate); fns are `pub fn` — clippy redundant_pub_crate)
pub fn check_step_inputs(model: &Model, data: &Data) -> Result<(), StepError>; // timestep, then shape
pub fn check_data_shape(model: &Model, data: &Data) -> Result<(), StepError>;
```
- `check_data_shape` compares 24 lengths. First the inputs a caller writes: qpos/nq, qvel/nv, act/na, ctrl/nu, qfrc_applied/nv, xfrc_applied/nbody, mocap_pos/nmocap, mocap_quat/nmocap. Then one array per other dimension `make_data` sizes from (`model_init.rs:495-854`): xpos/nbody, xanchor/njnt, geom_xpos/ngeom, site_xpos/nsite, ten_length/ntendon, wrap_xpos/2·nwrap, eq_violation/6·neq, flexvert_xpos/nflexvert, flexedge_length/nflexedge, flexedge_J/flexedge_J_colind.len(), qLD_data/qLD_nnz, sensordata/nsensordata, history/nhistory, tree_asleep/ntree, plugin_state/npluginstate, plugin_data/nplugin.
- **Sufficient for:** a `Data` made by another model, and a resized input array.
- **Not caught:** a caller who resizes a derived array (e.g. `cvel`); the doc says so.
- Cost: 24 `usize` comparisons per call, not timed.
- Called first in `step`, `step1`, `step2`, `forward`, `forward_skip`. It replaces the two old timestep checks. `step`'s internal `self.forward(model)?` (`:239`, `:247`) re-runs it (harmless). Calling `forward_core` there would skip the re-check (optional).
- `BatchSim::step_all` doc (`batch.rs:222-236`) lists `DataShapeMismatch` with the non-recoverable errors.
- `Data::integrate` / `Data::inverse`: **document** (decision 21 as worded: "`integrate`/`inverse` documented").
  - `integrate`: "does not check `model.timestep` or that `self` was made by `model`; `step()`/`step2()` do. At h ≤ 0 time runs backwards, at NaN the state becomes NaN (measured on main), and a mismatched `Data` panics on an index."
  - `inverse`: it does not read `model.timestep` (`inverse.rs:33-47`). Add `# Panics` for a `Data` not made by `model` (measured: `acceleration.rs:403`).
- The `debug_assert!(model.timestep > 0.0)` at `acceleration.rs:96-99` becomes unreachable through public entry points. Keep it.

**Tests to add** (all in `spec_tests_src/tests/`)
- `forward_refuses_a_timestep_that_is_not_positive_and_finite`: all five entry points at h ∈ {0, −1e-3, NaN, ∞}. **Main: FAILS** (forward Ok).
- `step2_at_negative_timestep_does_not_run_time_backwards`. **Main: FAILS.**
- `step_refuses_data_made_by_another_model`: the a/b ngeom case plus pend2/pend3. **Main: FAILS** (panic).
- New API: `step_names_the_mismatched_field` (`Err(DataShapeMismatch{field:"geom_xpos",expected:1,actual:0})`) and `invalid_timestep_says_what_it_is`.

**Tests that flip**
- `sim/L0/core/src/sensor/sensor_tests.rs:1325` `test_sensor_write_with_undersized_buffer_does_not_panic` (CI: sim-core lib). It shrinks `sensordata` to 0 and `forward().unwrap()`s, which is now `Err(DataShapeMismatch{"sensordata",1,0})`.
- Rewrite it as `test_forward_refuses_an_undersized_sensordata` asserting that `Err`. This is in the patch; after it, 717/717 pass.
- **The only test flip in the suites listed above** (`test_c1.log`). The 23 byte-compared validators are identical at the final patch, which includes this commit.

**Downstream.** None break at compile time: `StepError` is `#[non_exhaustive]`, and the new variant keeps Copy/Eq.
- Behaviour changes only for callers that today run `forward`/`forward_skip`/`step2` at h ≤ 0, or with a mismatched `Data`.
- `git grep` for non-positive timesteps in `.rs` finds only sim-mjcf `validation.rs:849` and sim-types `config.rs:387` (both config validation, not stepping).
- FD: `mjd_inverse_fd` (`derivatives/fd.rs:510+`) and `mjd_transition_hybrid` (`hybrid.rs:2894+`) call `forward_skip`, so they now return `Err` at h ≤ 0. `mjd_transition_fd` already did (it calls `step`, `fd.rs:99`).
- Not run: sim-gpu, sim-coupling, cf-design, cf-codesign, sim-rl, sim-opt, therm-env, sim-bevy tests.

**Open questions**
- (a) `integrate`: doc (settled wording) vs `-> Result<(), StepError>` running the same check.
  - Callers outside sim-core today: 0 (git grep: only `hybrid.rs` ×8, `fd.rs:765`, `plugin.rs:615`, `forward/mod.rs:194,249`).
  - **Recommend doc** (decision 21's text).
  - If wrong: a direct `integrate` caller at h ≤ 0 keeps getting backwards time silently, as `mj_Euler` does.
- (b) No variant-level `#[non_exhaustive]` on `DataShapeMismatch`. That follows ledger-L20's dropped precedent: tests construct the variant for `assert_eq!`.

---

## 2. ledger-L24 thermostat half: install refuses h ≤ 0

**Now.** `LangevinThermostat::validate` (`thermostat/src/langevin.rs:283-310`) checks only `!(2γkT/h).is_finite()`. **Measured** (`out_main_thermo.txt`):
- h = 0 and h = NaN → `NoiseOverflow{timestep: 0/NaN}`, which blames γ·kT;
- h = −1e-3 → `Ok`, then `forward` gives `qfrc_passive[0] = NaN`.

`ThermostatError` is `#[derive(Debug, Clone, PartialEq)]`, `#[non_exhaustive]` (`error.rs:9-11`).

**Target.** Refuse h that is not positive and finite at install, before the variance check. This is not a MuJoCo item, and sim-core's `step`/`forward` refuse it too after §1.

**Change** (measured)
```rust
/// The model's timestep is not positive and finite. sim-core's `step` and `forward`
/// refuse such a model too (`StepError::InvalidTimestep`); install refuses it first.
InvalidTimestep { component: &'static str, timestep: f64 },
// Display: "{component}: the model's timestep must be positive and finite, got {timestep}"
```
- `validate` checks it first.
- `InvalidParameter` was rejected: it is "a constructor parameter", and the timestep belongs to the model. Using it would stretch a variant's meaning, the objection that reverted round 1 (ledger-L24).
- Update the `validate` doc at `langevin.rs:282`.

**Tests.**
- `thermostat_install_refuses_a_negative_timestep`: **main FAILS** (`Ok`).
- New API: `thermostat_names_the_timestep` (h = 0 and −1e-3 → `InvalidTimestep`).
- Flips: none. `install_refuses_a_noise_variance_that_overflows` (`langevin.rs:715-740`) uses h = 1e-3. Thermostat lib 250/250 pass.

**Downstream.** therm-env wraps `ThermostatError` (non_exhaustive, so additive). The sim-mjcf half (drop the `> 1` rule) belongs to the mjcf area.

---

## 3. core-L6: ISD `forward()` stops writing qvel

**Now.**
- `mj_fwd_acceleration_implicit` writes `qvel = v_new` (`forward/acceleration.rs:230-234`). It is reached from `forward_acc` when `!newton_solved` (`forward/mod.rs:569-571`).
- `integrate`'s ISD branch then only integrates positions (`integrate/mod.rs:166-182`).
- **Measured** (`out_main_isd.txt`): qvel 0 → −0.00994 → −0.01982 over two `forward()` calls on a spring.

**Target.** `forward()` leaves the state unchanged, as its doc says (`forward/mod.rs:271-273`); `integrate` applies `v_new`. ISD is not a MuJoCo 3.5.0 integrator (`mjmodel.h:180-183` lists Euler, RK4, implicit, implicitfast), so there is no MuJoCo line to match. MuJoCo's `mj_forward` never writes qvel (`engine_forward.c:1365-1419`).

**Change** (measured; re-derived from the planner's `core_fix` at 3520544e)
- `acceleration.rs`: drop the `qvel` write and keep `qacc = (v_new − qvel)/h`.
- `integrate/mod.rs` ISD branch: `else { self.qvel.copy_from(&self.scratch_v_new); }`.
- `data.rs:544-545` `scratch_v_new` doc: it now carries `v_new` from the acceleration stage to `integrate`.
- Comments to rewrite:
  - `integrate/mod.rs:36-38` ("Implicit: Velocity was already updated…") and `:160-161`.
  - `derivatives/mod.rs:212-218` ("its forward pass overwrites qvel but leaves cvel stale").
  - `hybrid.rs:2384-2397`: the ISD `mj_fwd_velocity` refresh and its comment. Delete both. The refresh recomputes `cvel` from the same `qvel` after L6; with it removed, all suites stayed green (`test_c2b.log`, including the transition harness), which shows the result is unchanged within those tests' tolerances, not bit-identity.
  - `hybrid.rs:2467-2468`, `:2742-2745` ("(v⁺⁺ − v⁺)/h"), `:2780-2786` ("the initial forward() … updates qvel to v⁺").

**Tests**
- `implicitspringdamper_forward_does_not_change_the_state`: **main FAILS.** This is the pin.
- `split_step.rs:70/:78` 1e-12 → `to_bits()`. **Measured: passes on main too** (separate target), so it tightens but does **not** pin L6. Keep it as a tightening.

**Tests/examples that flip:** none in any suite listed in "How this section was built" (`test_c2.log`).

**Output changes (validators still pass)**
- `examples/fundamentals/sim-cpu/integrators/stress-test` (CI validator) ImplSpDmp row: E_now 0.008872 → 0.005939 and drift +0.3617 % → +0.2422 %. That row now equals the Euler row, as its message "Euler-like with zero K/D" expects. Check 5 (`< 5 %`) still PASS; 7/7.
- The Bevy demo `integrators/implicit-spring-damper` (not run in CI, threshold 5 % at `main.rs:220`) runs the same scene; not run.

**Expected-change list**, for ISD models without active constraints (`!newton_solved`) that read state after `forward()`:
- acc-stage sensors; `cacc`/`cfrc_*` (`mj_body_accumulators` reads `qvel`, `acceleration.rs:600`); `solver_fwdinv` under `ENABLE_FWDINV`;
- any trajectory that called `forward()` before stepping: each such call advanced qvel once.

**What was searched:** the 32 ISD docs of the static corpus have only `jointpos`/`jointvel` sensors (grep), and the Rust test suites passed. `format!` templates outside the run suites were not searched individually.

**Downstream.**
- ml-chassis `SimEnv::step`'s post-step `forward()` stops double-advancing ISD (FINDINGS 1219).
- Not run: sim-rl, sim-opt, therm-env.

---

## 4. core-L2: `reset_to_keyframe` = validate → `reset` → copy

**Now.** `data.rs:1217-1291` copies the keyframe and clears a hand-picked subset. Warnings, energy, plugin state, passive/constraint forces and `stat_meaninertia` stay.

**Measured** (`out_main_keyframe.txt`), after a NaN auto-reset and 5 steps:
- `divergence_detected()` is **true** after `reset_to_keyframe` and false after `reset` + copy.
- The `{:#?}` field diff between the two is `stat_meaninertia`, `warnings`, `energy_potential` (this scenario).
- After `forward` + 20 steps, both are bit-identical in qpos/qvel.

**Target.** MuJoCo `mj_resetDataKeyframe` (`engine_io.c:1562-1576`): `_resetData`, then copy time/qpos/qvel/act/mpos/mquat/ctrl if `0 ≤ key < nkey`.
- Deviation, **stricter**: an out-of-range index returns `Err` and leaves `Data` unchanged. MuJoCo resets and silently skips the copy.

**Change** (measured): validate the index → `self.reset(model)` → copy the seven fields. Doc rewritten with the MuJoCo citation. The duplicated history-init block goes, because `reset` does it.

**Tests**
- `reset_to_keyframe_is_a_full_reset`: **main FAILS** (divergence stays).
- `reset_to_keyframe_with_a_bad_index_leaves_data_unchanged`: **passes on main** (pins the validate-first order).
- Stay green: `tests/integration/keyframes.rs` ac04 (:269), ac05 (:292), ac06 (:323, pins the `Err`), ac07 (:343), ac08 (:378), ac26 (:936) all pass. The keyframes validator output is byte-identical.

**Downstream.** 10 example call sites (projectile, spinning-toss, keyframes multi-body/save-restore: Bevy, not run; `keyframes/stress-test` validator: identical output).

**Nearby, not mine.**
- `Data::reset` sets `stat_meaninertia = 0.0` (`data.rs:1182`) while `make_data` sets 1.0 (`model_init.rs:693`). Readers overwrite it first (`constraint/mod.rs:75-77`).
- `reset` initialises only actuator history, while MuJoCo also initialises sensor history (`engine_io.c:1395-1427`).

---

## 5. core-L1: `energy_initial` (decision 6 → option (a))

**Now.** Only `forward_skip` captures it, with a `== 0.0` sentinel (`forward/mod.rs:385-390`). `forward()`/`step1()` go through `forward_pos_vel` (`:442-519`), which never captures.

**Measured** (`out_main_energy.txt`):
- q0 = 0.3: after `forward` and 100 steps, `energy_initial` = 0 while total = −2.3430.
- At θ = π/2 the first total energy is −5.4e-16, not 0.
- With the sentinel, a true first energy of exactly 0 is overwritten by the next one (test below).

**Target.** Captured once, at the first forward pass that computes energy after `make_data`/`reset`, and cleared by `reset` (so also by auto-reset and by §4). Not a MuJoCo field (MuJoCo has `d->energy[2]` only), so this is an extension.

**Change** (measured)
- Fields:
  - `pub(crate) energy_initial_captured: bool` on `Data`: set false in `make_data` (`model_init.rs:690`) and `reset` (`data.rs:1190`), copied in `Clone` (`:815`).
  - The public field stays `pub energy_initial: f64`; its doc is rewritten (0.0 until captured; auto-reset re-captures; not MuJoCo).
- Capture:
  - `fn capture_energy_initial(&mut self)` is called right after `mj_energy_vel` in both `forward_pos_vel` (which `forward`, `step`, `step1` and RK4 stages reach) and `forward_skip`'s velocity stage.
  - RK4 stages cannot capture first, because `step` runs `forward` before them.
- **`data_reset_field_inventory` (`data.rs:1310`, `EXPECTED_SIZE = 4416`) does NOT move.** Measured: adding the bool left `size_of::<Data>()` at 4416 (padding). The guard is blind to a field that fits in padding, so it cannot be relied on to catch this field. Option (b) `Option<f64>` would grow it (not measured).

**Tests**
- `forward_captures_energy_initial`: **main FAILS.**
- `energy_initial_is_captured_once_even_when_zero` (no gravity, at rest, then qvel = 1): **main FAILS** ("baseline moved to 0.05416666666666667", `spec_base.log`).
- `reset_clears_the_energy_baseline`: **main FAILS.**

**Validators**
- `integrators/stress-test` (CI): the E_0 column prints `-0.000000` instead of `0.000000` (−5.4e-16 now captured). Drift changes by ~2e-14 % (not separately measured); 7/7 PASS.
- `solvers/stress-test` (CI): byte-identical. Its `energy_settled` is always `Some`, so the `energy_initial` fallback (`main.rs:141`) is not reached.
- The 5 Bevy integrator demos (`euler`, `implicit`, `implicit-fast`, `implicit-spring-damper`, `rk4`; checks at `main.rs:~210-222`) are not CI-run (no `example_kind`, Bevy deps). They print E₀ ≈ −5e-16 instead of 0 — inferred from the same scene, not run.

**Open question.** Make the flag `pub`? **Recommend `pub(crate)`.** Readers re-baseline by writing `energy_initial`. If wrong: a caller cannot tell "captured 0.0" from "not captured".

---

## 6. core-L7: plugins (decision 9 → RK4 advance + `try_make_data`, no `Plugin::copy_data`)

**Now.**
- **Measured** (`out_main_plugin.txt`): `advance()` calls in 10 steps are Euler 10, ImplicitFast 10, **RK4 0**. `rk4.rs:24-210` never calls it; only `integrate` does (`integrate/mod.rs:209-214`).
- `Data::clone` sets every `plugin_data` to `None` (`data.rs:890-892`); measured `Some(7)` → `None`.
- `make_data` panics on a plugin `init` error (`model_init.rs:895-900`); measured.
- In-tree: 0 `Plugin` implementors outside sim-core's own tests (git grep `impl Plugin for` finds only Bevy plugins), and the MJCF builder sets `nplugin: 0` (`mjcf/src/builder/build.rs:491`).

**Target**
- RK4 advances plugins once per step, after the time advance: MuJoCo `mj_RungeKutta` → `mj_advance` (`engine_forward.c:1120-1121`), whose plugin loop is `:923-936`, after time (`:920-921`).
- `try_make_data` returns the init error. MuJoCo `mj_initPlugin` raises `mjERROR` (`engine_io.c:1003-1008`), so this is parity in Rust's error channel.
- `Data::clone` dropping `plugin_data` is a **stated limitation**: MuJoCo `mj_copyData` calls `plugin->copy` (`engine_io.c:1240-1246`), and we add no copy hook.

**Change** (measured)
- `pub(crate) fn advance_plugins(model, data)` in `integrate/mod.rs`, used by `integrate` and at the end of `mj_runge_kutta`.
- New `#[derive(Debug, Clone, PartialEq, Eq)] #[non_exhaustive] pub enum MakeDataError { PluginInit { instance: usize, message: String } }` with Display "plugin init failed for instance {i}: {message}" (the text `make_data` panics with today). Exported from `lib.rs`.
- `pub fn try_make_data(&self) -> Result<Data, MakeDataError>`; `make_data` = `try_make_data().unwrap_or_else(|e| panic!("{e}"))`.
- `# Panics` stays for `validate_joint_layout` (`model_init.rs:458`), which still panics. The joint-layout refusal belongs to mjcf-H3.
- Doc on `impl Clone for Data` and on `Plugin`: clones and FD scratch copies (`fd.rs:87`, `hybrid.rs`) see `plugin_data == None`.

**Tests.**
- `rk4_advances_plugins_once_per_step`: **main FAILS** (0).
- New API: `try_make_data_returns_a_plugin_init_failure`.
- Flips: none (`plugin.rs` t11 `:609` tests Euler advance; still passes).

**Open question.** Also call `Plugin::reset` at the end of `try_make_data`? MuJoCo `mj_makeData` = `mj_initPlugin` then `mj_resetData` (`engine_io.c:1110-1111`), which calls `plugin->reset` (`:1533-1541`); ours never resets at `make_data`.
- **Recommend yes** (0 in-tree plugins, parity).
- If wrong: plugin state starts at zero rather than at the plugin's reset values.

---

## 7. ledger-L31 (sleep panic) + core-L5 (decision 8 → (a) `Model::recompute_derived()`)

### ledger-L31: root cause, measured

- **Measured** (`out_main_sleep.txt`): `ENABLE_SLEEP` on `n_link_pendulum`, `free_body`, `double_pendulum`, `spherical_pendulum`, `multi_joint_body` panics at `island/sleep.rs:538:36`. That is `mj_wake`'s `model.dof_treeid[dof]` with `dof_treeid` empty (ntree = 0).
- **Cause.** The factories (`types/model_factories.rs`) and `test_fixtures::builders::finalize` (`:802-813`) never compute the tree tables. Tree discovery exists only as private functions in sim-mjcf (`builder/build.rs:690-745` `discover_kinematic_trees`, `:750-806` `compute_tendon_tree_mapping`, `:810-956` `resolve_sleep_policies`) and as a second private copy in cf-design (`mechanism/model_builder.rs:~1160-1226`, every tree `SleepPolicy::Never`).
- **Second panic behind it** (measured): with trees added, the next panic is `sleep.rs:161:75`, because `dof_length` is empty. The public `compute_dof_lengths` itself panics on a factory model (`model_init.rs:1288`, measured), since it writes `dof_length[dof]` without sizing it.
- **With trees + sized `dof_length`** (probe copy of sim-mjcf's routine): 2000 sleep-enabled steps run on `n_link_pendulum(2)` and `free_body`.
- **So:** `recompute_derived` fixes L31 only if it computes trees and `dof_length` **and** the factories/`finalize` call it. An opt-in that the default factories do not call leaves the default path panicking, as the design said.

### Change (measured)

- **New `types/model_trees.rs`** — `pub fn compute_kinematic_trees(&mut self)`. It writes `ntree`, `tree_body_adr/num`, `tree_dof_adr/num`, `body_treeid` (world = `usize::MAX`), `dof_treeid`, `tendon_treenum`, `tendon_tree`, and resolves AUTO sleep policies (sim-mjcf steps 1, 1b and 3, copied).
  - An explicit `Never/Allowed/Init` survives when `ntree` is unchanged.
  - MuJoCo computes the same tables in `setFixed` (`engine_setconst.c:106-283`), part of `mj_setConst`.
- **sim-mjcf `build()`** (`build.rs:48-50`) calls it, then a 20-line `apply_explicit_sleep_policies` (old step 2). It deletes its 3 private functions (~250 lines).
  - **Measured equivalence:** tree fields (all 10 plus `dof_length`) are identical between old and new builders for all 1,584 corpus docs (1,478 load). Test: `core_state/trees_base.txt` vs `trees_fix.txt`, 0 diff lines. The base/fix binaries were checked to differ: factory `ntree` 0 vs 1.
- **`compute_dof_lengths`** sizes `dof_length` (`resize(nv, 1.0)`).
- **`pub fn recompute_derived(&mut self)`** runs, in build order: `compute_ancestors`, `compute_implicit_params`, `compute_qld_csr_metadata`, `compute_spatial_tendon_length0`, `compute_actuator_params`, `compute_stat_meaninertia`, `compute_invweight0`, `compute_kinematic_trees`, `compute_dof_lengths`.
  - MuJoCo analogue: `mj_setConst` (`engine_setconst.c:1089-1101`, `mujoco.h:278-279`).
  - **Measured no-op on unedited models:** on a fresh MJCF build, `recompute_derived` changes no field of `{:#?}` for 1,478/1,478 loadable corpus docs (`core_state/idem.txt`).
- **Factories and `finalize`** call `recompute_derived()` (patch commit `c6d-exp`). This adds trees + `dof_length` everywhere, `stat_meaninertia` to all factories, and `invweight0` to `multi_joint_body`/`spherical_pendulum`/`free_body`, which lacked them.

### Derived-field table for the docs (core-L5: "what to call after editing what")

Read from the code at 3520544e.

| You edit (pub `Model` field) | Stale derived field(s) | Recompute |
|---|---|---|
| `jnt_stiffness`, `jnt_damping` (hinge/slide), `dof_damping` (ball/free), `qpos_spring` | `implicit_stiffness/damping/springref` (eulerdamp `integrate/mod.rs:83-93`, ISD, implicit derivatives) | `compute_implicit_params` (`model_init.rs:967`) |
| body/joint tree (`body_parent`, `body_jnt_*`, `body_dof_*`) | `body_ancestor_joints/mask`, `body_weldid` | `compute_ancestors` (`:916`) |
| `dof_parent` | `qLD_rownnz/rowadr/colind`, `qLD_nnz` (**changes `Data.qLD_data`'s length → make a new `Data`**; §1 refuses the old one) | `compute_qld_csr_metadata` (`dynamics/factor.rs:20`) |
| masses, inertias, body poses, `qpos0`, joint axes | `body_subtreemass`, `body/dof/tendon_invweight0`; `stat_meaninertia` | `compute_invweight0` (`:1010`), `compute_stat_meaninertia` (`:1206`) |
| gears, transmissions, joint/tendon ranges, masses | `actuator_lengthrange`, `actuator_acc0`. **Consumed inputs:** `biasprm[2]` dampratio→−damping (`fiber.rs:200-204`) and muscle `gainprm[2]` F0 (`:233-237`) are overwritten on the first call, so later edits to them are not re-derived (same as MuJoCo `set0`, `engine_setconst.c:869-905`) | `compute_actuator_params` (`forward/fiber.rs:28`) |
| spatial tendon geometry, `qpos0` | spatial `tendon_length0`, `lengthspring` sentinel (consumed) | `compute_spatial_tendon_length0` (`tendon/mod.rs:55`) |
| `body_rootid`, `body_dof_*`, actuators, tendons | tree tables, tendon trees, AUTO sleep policies | `compute_kinematic_trees` (new) |
| `body_pos`, tree, joint types | `dof_length` | `compute_dof_lengths` (`model_init.rs:1272`) |
| `geom_size`/type, mesh/hfield/sdf | `geom_rbound`, `geom_aabb` | **sim-mjcf-private** `compute_geom_bounding_radii` (`build.rs:568`) |
| fixed tendon wraps, `qpos0`/`qpos_spring` | fixed `tendon_length0`, `lengthspring` | **sim-mjcf-private** `compute_fixed_tendon_lengths` (`build.rs:645`) |
| `actuator_nsample`, `sensor_nsample` | `*_historyadr`, `nhistory` (Data shape) | **sim-mjcf-private** `compute_history_addresses` (`build.rs:520`) |

Damping note for the doc: hinge/slide passive damping reads `jnt_damping` (`passive.rs:918`), ball/free reads `dof_damping` (`:999`). MuJoCo has only `dof_damping`; decision 8 (b) would remove that split.

### Tests

- `sleep_runs_on_factory_models`: **main FAILS** (panic).
- `compute_dof_lengths_sizes_dof_length`: **main FAILS.**
- New API: `recompute_derived_follows_a_damping_edit`.

### Flips

None across sim-core lib, sim-mjcf lib, conformance, ml-chassis lib, thermostat lib and the 23 byte-compared validators (the integrator stress-test's output equals its commit-3 output), with **factories and `finalize` calling the full `recompute_derived`** (`test_c6d.log`, `thermo_lib_fix.log`, `val_*`). The minimal variant (trees + `dof_length` only, `c6a`) also flips nothing.

### Downstream

- `sim-mjcf` `build.rs`: compiles; `ActuatorTransmission`/`WrapType` imports drop.
- Factory/`finalize` consumers **not run**: `sim/L0/gpu/src/pipeline/tests.rs` (46 factory uses), `tools/cf-codesign/src/lib.rs` (12), `examples/.../derivatives/stress-test` (run: identical output), `sim/L1/coupling/tests`, sim-rl / sim-opt (via test-fixtures).
- Risk if numbers move there: `stat_meaninertia` (solver scaling) and `invweight0` (constraint regularisation) on factory models that have constraints.

### Open questions

- (a) Factories/`finalize` → full `recompute_derived` (recommended: one derivation for every producer) vs trees + `dof_length` only.
  - Both measured: 0 flips in the suites run.
  - If the full version is wrong for a downstream suite, that suite's numbers move on constraint-bearing factory models.
- (b) Move `compute_geom_bounding_radii` and `compute_fixed_tendon_lengths` (model-only: they read only `Model` fields, `build.rs:568-614`, `:645-683`) into sim-core and into `recompute_derived`, so a `geom_size` edit is covered.
  - **Recommend yes; not in the measured patch.**
  - If not moved: the doc table must say those two are not recomputable outside sim-mjcf.
- (c) Switch cf-design's private tree copy to `compute_kinematic_trees` plus its `Never` override.
  - No bug today; it removes a third copy.
  - Optional (cf-design tests not run).
- (d) Decision 8 itself, (a) vs (b), stays Jon's. Under (b), the damping row above disappears.

---

## 8. core-B2, core-B3, ledger-L19 (decision 10)

**Now**

| Item | Today | Referent |
|---|---|---|
| B2 | `BatchSim::reset` says it does **not** zero `qfrc_applied`/`xfrc_applied`, but `Data::reset` does | `batch.rs:289-290`; `data.rs:1138-1141` |
| B3 | `model()` returns env 0's model; there is no `forward_all`, no `model_of`, and no shape check | `batch.rs:178-183` |
| L19 | `new_per_env(prototype: &Arc<S>, n, factory)` panics through `PerEnvStack::install_per_env` with no `# Panics` doc | `batch.rs:102-139` |
| Trait | `PerEnvStack::install_per_env` returns `EnvBatch<Self>` and has no error path | `batch.rs:351-390` |

The `prototype` argument is unused: `PassiveStack`'s impl does `let _ = self;` (`thermostat/src/stack.rs:290-319`). That impl panics on a shared stack or a refused model (`:307-314`; tests `:771-799`).

**Target / Change** (measured, recommended shape "C")
```rust
pub trait PerEnvStack: Send + Sync + 'static {
    type Error: std::error::Error + Send + Sync + 'static;
    /// Install the stack's callback(s) on `model`, or refuse it, leaving `model` unchanged.
    fn install_on(self: &Arc<Self>, model: &mut Model) -> Result<(), Self::Error>;
}
#[derive(Debug)] // spec: add Clone, PartialEq (derive bounds them on E)
#[non_exhaustive]
pub enum PerEnvError<E> {
    NoEnvs,
    SharedStack { env: usize, earlier: usize },
    Install { env: usize, source: E },
    ShapeMismatch { env: usize, field: &'static str, expected: usize, actual: usize },
} // Display + Error (source() = Install's source)
impl BatchSim {
    pub fn try_new_per_env<S: PerEnvStack, F: FnMut(usize) -> (Model, Arc<S>)>(n: usize, factory: F)
        -> Result<Self, PerEnvError<S::Error>>;
    /// # Panics — on any PerEnvError, "BatchSim::new_per_env: {e}"
    pub fn new_per_env<S, F>(n: usize, factory: F) -> Self;
    pub fn model_of(&self, i: usize) -> Option<&Model>;
    pub fn forward_all(&mut self) -> Vec<Option<StepError>>; // spec: same rayon dispatch as step_all
}
// EnvBatch is deleted; `impl PerEnvStack for PassiveStack { type Error = ThermostatError; install_on = try_install }`
```
- sim-core owns the loop: the shared-stack check (`Arc::ptr_eq`), install, and the shape check.
- The shape check reuses §1's `check_data_shape(&models[0], &envs[i])`.
- `model()` loses its `# Panics` (n = 0 is refused). Its doc says env 0's parameters and callbacks, with the shape checked for all envs.
- B2: the `reset` doc says it zeroes the applied forces via `Data::reset`.

**Tests**
- New API: `per_env_batch_refuses_models_of_different_shapes` (also `NoEnvs`) and `model_of_returns_each_envs_model_and_forward_all_runs`.
- Thermostat tests ported from `install_per_env`: `stack.rs:576` → `new_per_env_builds_n_envs_with_callbacks_set`. The 3 `should_panic` tests (`:771-799`) become 3 `try_new_per_env_*` tests matching `PerEnvError::{Install{source: DofOutOfRange{dof:2,..}}, SharedStack{1,0}, Install{source: PassiveCallbackInstalled}}`, plus `new_per_env_panics_on_a_refused_model`. Thermostat lib 251/251.
- B2: `batch_reset_zeroes_applied_forces` **passes on main** (doc-only fix; the test pins the corrected doc).

**Flips / downstream (breaking)**
- `tests/integration/batch_sim.rs:276-302`: drop the prototype (passes after).
- sim-thermostat: `stack.rs` impl, tests and module docs (`stack.rs:20-24, 59, 110-114, 272-289`); `lib.rs:22` (`EnvBatch`); `langevin.rs:43, 52, 100, 139` (the word `install_per_env`).
- `sim/L0/rl-baselines/tests/d2c_sr_rematch.rs:114`: comment.
- `docs/studies/ml_chassis_refactor/**` and `docs/thermo_computing/02_foundations/chassis_design.md` (~200 mentions) are dated design records. Not rewritten.
- The Envs PR (ledger-L11: `VecEnv`/`build_vec` → `new_per_env`) is the first new caller.

**Open question.** Shape C (above) vs B: keep `install_per_env` and add `type Error`, returning `Result<EnvBatch<Self>, Self::Error>`. Under B:
- Each implementor re-implements the sharing check, and `ThermostatError` needs env-indexed variants.
- sim-core still needs `PerEnvError` for `ShapeMismatch`.

**Recommend C.** If wrong, an implementor that must see all envs at once cannot do it in the trait; the factory sees the env index.

---

## 9. core-D4 (decision 11 → delete)

**Now**
- `sim_types::{SimulationConfig, SolverConfig, Gravity}` (`sim/L0/types/src/config.rs`, 533 lines, 15 tests) are not read by sim-core: `git grep` finds no hit under `sim/L0/core`.
- The only producer path is sim-mjcf `config.rs:12-47` (`From<&MjcfOption>`), plus `ExtendedSolverConfig` (`:49-170`, 5 tests, `pub base: SimulationConfig`). `ExtendedSolverConfig` has 0 consumers outside its own tests and duplicates the pub `MjcfOption` (exported `mjcf/src/lib.rs:202`).
- `SimError` (`types/src/error.rs`) and `sim_types::Result` (`lib.rs:62-63`) are produced only by `SimulationConfig::validate` and the `SolverConfig` validation in `config.rs`.

**Change (by grep; not compiled)**

| Location | Change |
|---|---|
| `sim/L0/types/src/config.rs` | delete |
| `sim/L0/types/src/lib.rs` | `:7-8` doc bullets, `:30-38` doctest, `:52` `mod config;`, `:56` re-export (and `:57`, `:62-63` + `error.rs` if SimError goes) |
| `sim/L0/types/Cargo.toml:3` description and `README.md:3` | change together: README is generated from the description, checked by `xtask publish-set`, `xtask/src/publish_set.rs:455-465` |
| `sim/L0/mjcf/src/config.rs` | delete |
| `mjcf/src/lib.rs` | `:181` `mod config;`, `:192` `pub use config::ExtendedSolverConfig;` |
| `sim/L0/mjcf/Cargo.toml` | drop the `sim-types` dependency (its only use is `config.rs:7`) |
| `sim/sim/src/lib.rs` | `:13`, `:87` docs; `:133` prelude `SimError, SimulationConfig`; tests `:152-153`, `:160` |
| `cortenforge/src/lib.rs:124` | size test → `crate::sim::types::Pose` |
| Docs | `sim/docs/ARCHITECTURE.md:405-406`; `sim/docs/MUJOCO_GAP_ANALYSIS.md:1206-1210`. `sim/docs/todo/**` mentions are historical |

**Tests.** 15 + 5 (+3 SimError) tests deleted with their types. Nothing to add: deletion is checked by compile.

**Downstream.** sim-types reaches every sim crate; the break is limited to the files above (git grep).

**Open question.** Also delete `SimError`/`Result`, and `ExtendedSolverConfig` whole?
- **Recommend yes**: both have no producer or consumer left.
- If wrong, 0.10 freezes a dead error type and a duplicate of `MjcfOption`.

---

## 10. Docs: core-D1, core-D2, core-D3 rest, ledger-L25, `MUJOCO_CONFORMANCE.md`

### core-D1

| Location | Today | New text |
|---|---|---|
| `lib.rs:8` | "[`Model`] is static (immutable after loading)" | Model fields are pub and may be edited between steps; after editing a field a derived cache reads, call `Model::recompute_derived` (table §7) |
| `lib.rs:24` | "One step: forward() then integrate()" | `step()` = state checks → `forward()` → acc check → integrate (Euler/implicit/RK4) → sleep update → warm-start save (`forward/mod.rs:225-267`); calling `forward()` + `integrate()` by hand skips the checks, sleep and warm-start |
| `model.rs:3-6`, `:32` | "static, immutable", "Immutable after construction" | same correction as `lib.rs:8` |
| `forward/mod.rs:315-317` | "The FD perturbation loop calls `forward_skip() + integrate()` to replace `step()`" | stale: `mjd_transition_fd` calls `step()` (`derivatives/fd.rs:99, 150, 177, 227, 250`); `forward_skip` is used by `mjd_inverse_fd` (`fd.rs:510-612`) and the hybrid sensor columns (`hybrid.rs:2893-2915`) |

Same family, found by the sweep: `data.rs:27-29` says "`qpos` and `qvel` are the ONLY state variables". Replace it with MuJoCo's split, `mjdata.h:26-53`:
- physics state: time, qpos, qvel, act, history, plugin state;
- inputs: ctrl, qfrc_applied, xfrc_applied, mocap pose;
- warm start: `qacc_warmstart`.

### core-D2

- Remove `data.rs:23` "All arrays pre-allocated - no heap allocation during simulation", `:535` "(for allocation-free stepping)", `:553` "no heap allocation during stepping", `model_init.rs:796`.
- **Measured** with a counting global allocator (`probe_main/src/bin/allocs.rs`), steps 101-200 at main:
  - `n_link_pendulum(2)` undamped Euler: **8.0** allocations/step;
  - damped (eulerdamp): **11.0**;
  - MJCF box on plane (4 contacts): **67.0**.
- Allocating sites (read): the efc arrays are rebuilt every step (`constraint/mod.rs:313-333`), the solver input clones `qM`/`qfrc_smooth` (`:410-411`), eulerdamp clones `qLD` and allocates `rhs` (`integrate/mod.rs:89-90, 102`).
- New text: arrays sized by the model (state, body/geom/site/tendon, `scratch_*`, `rk4_*`, `deriv_*`) are allocated by `make_data`, and `contacts` reserves 256. The constraint stage and eulerdamp allocate every step.

### core-D3 rest

- `step2` doc (`forward/mod.rs:155-158`, `:169-173`) and `step1` doc (`:123-126`): `step2` integrates with the model's integrator (`integrate/mod.rs:63` dispatches), and only RK4 falls back to Euler (`:185-197`). MuJoCo `mj_step2`: "integrate with Euler or implicit; RK4 defaults to Euler" (`engine_forward.c:1505-1510`).
- `step2` `# Errors` adds `InvalidTimestep` and `DataShapeMismatch`.
- `batch.rs:59-61` "warmstart `HashMap`": `Data` has no `HashMap` (grep count 0). Replace with "`qacc_warmstart` and contact/constraint vectors".
- The Display text is in §1.

### ledger-L25 (SDF; FINDINGS 300-311)

- **Single-threaded step.** Add to `lib.rs` crate docs and to `Data::step`: one step runs on the calling thread. The `parallel` feature (`Cargo.toml:58`) parallelises only `BatchSim::step_all` across envs; `batch.rs` is the only rayon user in `src/` (git grep).
- **`sdf_maxcontact`** (`model.rs:923-927`):
  - It applies to SDF–SDF pairs: it is passed to the octree tier and then capped (`collision/sdf_collide.rs:172-189`). The SDF–plane octree tier uses a fixed 8 (`sdf/shape.rs:233`).
  - The cap is applied to contacts already found, so it bounds the constraint solve, not detection.
  - The user's measurement, unreproduced: ~250 µs per item-drum pair at a 10 mm cell, at any cap.
- **Octree cell.** The octree leaf is 2× the finer of the two shapes' SDF grid cells (`sdf/octree_detect.rs:61`), so the SDF cell size (cf-design's resolution) sets the detection cost for the octree tier too.
- The cf-design half of L25 is not Rigid.

### `sim/docs/MUJOCO_CONFORMANCE.md`

- **Memory model** row (`:327`, "Pre-allocated pools | Dynamic allocation | Rust idioms, safety").
  - MuJoCo: `mj_makeData` allocates one buffer + arena (`engine_io.c:1069-1081`), and steps take scratch from the arena (`mj_markStack`/`mjSTACKALLOC`, e.g. `engine_forward.c:1049-1054`).
  - Ours: model-sized arrays at `make_data`; the constraint stage and eulerdamp allocate per step (8 / 11 / 67 per step measured above). The rationale "safety" has no referent: drop it or say "not yet pooled".
- **"3. Real-World Model Loading ✅ COMPLETE"** (`:150-212`), the progress row (`:396`), and the claim at `:59-61` that the suite "covers model loading (Menagerie + DM Control)". All false today:
  - The loading tests (`mujoco_conformance/menagerie.rs`, `dm_control.rs`) were deleted in `b56753d1` (2026-01-27, three days after "Completed 2026-01-24"), and no test loads the submodules (git grep). CI never checks them out (ledger-L28).
  - **Measured** (planner harness, `SCRATCH/rigid_plan_tests/sub_1.jsonl`): 53/253 submodule files load, 17/188 Menagerie and 36/65 dm_control.
  - Of the table's 16 Menagerie models, **1 loads**: `agility_cassie`, listed as "agility_digit", which does not exist. The others fail on `mesh '': duplicate mesh name` (mjcf-S15) or "expected 3 values, got 1" (single-value friction, ledger-L28).
  - Of its 19 dm_control models, **13 load**; humanoid, humanoid_CMU, manipulator, dog and stacker fail on friction, and quadruped on hfield.
  - Rewrite the status with these counts and the date. It cannot say "complete".
- **Intentional Divergences** (`:321`): add this area's deviations — timestep refusal (§1), shape refusal (§1), keyframe bad index (§4), `energy_initial` extension (§5), clone drops `plugin_data` (§6).

---

## Commit list (this area; order for the PR)

The scratch log measured these steps in a slightly different order: the `sensor_tests.rs` rewrite and the `hybrid.rs` refresh removal came last there. At every step the only red test was the one commit 1 rewrites. Each listed commit compiles and is green in that sense; a strict per-commit re-run in this order was not done.

1. **`fix(sim-core): one pre-step check on every Result entry point; thermostat refuses h ≤ 0`** — §1 + §2 (+ the `sensor_tests.rs:1325` rewrite). ws_fix `e8154ec`, `a4d252b`.
2. **`fix(sim-core): implicitspringdamper forward() leaves qvel unchanged`** — §3 (+ `hybrid.rs` refresh removal, comments, `split_step` `to_bits`). `46f59c3`, `f3f1afd`.
3. **`fix(sim-core): reset_to_keyframe resets fully; energy_initial captured once per reset`** — §4 + §5. `da0f2fc`.
4. **`fix(sim-core): RK4 advances plugins; Model::try_make_data`** — §6. `8bb9e54`.
5. **`feat(sim-core): kinematic trees in sim-core; Model::recompute_derived`** — §7 (sim-core + sim-mjcf `build.rs`). `1cfe179`, `176f092`, `7554d75`, `a1a020e`.
6. **`feat(sim-core)!: per-env BatchSim returns errors; model_of, forward_all`** — §8 (sim-core + sim-thermostat + conformance test). `ed8557d`. Depends on 1 (`check_data_shape`).
7. **`refactor!: delete SimulationConfig, SolverConfig, Gravity`** — §9 (sim-types, sim-mjcf, `sim`, `cortenforge`). Not measured.
8. **Docs** — §10 + B2. This can merge into the design's docs commit 9 with the callbacks area's C1/C2/C4/L4/B1 docs.

## Dependencies on the callbacks area (core-C1/C3/C4/L4/B1, callback order)

- **Commit 1 edits the same five entry points** that the callbacks commit edits (`cb_control` in `step1` `:146-149`, `forward_core` `:425-430`, `forward_skip` `:396-401`; passive in `step2`). The overlap is textual, and either order works. **Land 1 first** (design order 1 → 4) so their step2/step1 rewrite starts after the check line.
- **Their FD-under-RK4 refusal flips a CI validator:** `examples/fundamentals/sim-cpu/derivatives/stress-test/src/main.rs:786-788` (`check_23_rk4` expects `mjd_transition_fd` Ok under RK4 — by reading: `check_integrator` passes on `result.is_ok()`, `:756-768`; not run with their change). Also `check_21_implicit` is labelled "ImplicitSpringDamper" but loads `integrator="implicit"` (`:778-780`).
- **A false comment for them:** `forward/mod.rs:396-400` says MuJoCo's `mj_forwardSkip` skips `mjcb_control`; 3.5.0 calls it, gated on actuation (`engine_forward.c:1399-1401`).
- **§5's capture sits after `mj_energy_vel`.** If they move `mj_fwd_passive` into the velocity stage (MuJoCo calls `mj_passive` from `mj_fwdVelocity`, `engine_forward.c:250`), keep the capture after the energy computation.
- **Commit 4 and their RK4 control-count change** both touch `rk4.rs`/`forward_skip_sensors`. The overlap is textual only.
- **Commit 6 and their B1** (step_all determinism doc) both edit `batch.rs` docs.
- **ledger-L31's second item** (a ctrl-writing control callback zeroes the FD `B`) and core-C2 docs are theirs.

## Found in passing (not my rows; for the owner)

- **Actuator/sensor history has no runtime effect.** `actuator_delay`, `sensor_delay` and `nsample` are parsed and `Data.history` is sized/reset, but nothing in `sim/L0/core/src` reads or writes `history` during a step (git grep). MuJoCo inserts samples in `mj_advance` (`engine_forward.c:837-884`).
  - Under the parity rule this is "silently does other than asked", and it bears on decision 22: the crate accepts 3.5.0 `nsample`/`interp`/`interval`.
  - Not measured by a run.
- `MUJOCO_CONFORMANCE.md` cites MuJoCo **3.5.0** reference data (`:61, :316, :397, :401, :588`), while `tests/mujoco_conformance/mod.rs:3-8` and the design say **3.4.0**.

## What I checked, and what this method cannot see

- **Checked:** every rule by a test that fails on main (12) or by new API (7); patch-level test runs of the suites in "How this section was built"; 23 headless validators byte-identical base vs fix; the integrator stress-test changed only the rows stated in §3 and §5 (7/7 PASS), the solver stress-test is identical (both re-run at the final patch: `integ_fix_final.out`, `solver_fix_final.out`); corpus-wide tree equality and `recompute_derived` idempotency; clippy/fmt on the patched crates.
- **Not run:**
  - Downstream test suites: sim-gpu, sim-coupling, cf-design, cf-codesign, sim-rl, sim-opt, therm-env, sim-bevy, the thermostat integration tests (`tests/*.rs`).
  - The 5 Bevy integrator demos.
  - Licensed gates and `grade`.
- **Not timed:** the cost of the 24-length check.
- **Not compiled:** core-D4 (§9) and the §7(b) move.
- **Corpus scope:** the corpus comparisons see the 1,584 static docs only, not the 157 `format!` templates or the submodule files.
- **MuJoCo shape behaviour:** observed only through the Python bindings, which do not check either.
- `git -C …/cortenforge status --short` and HEAD: before `3520544e`, clean; after, `3520544e`, clean (nothing written in the repo).
- Cleanup: all of this area's `target*/` dirs are deleted, and so are the `tests/assets` copies inside `ws_base`/`ws_fix` (3.6 GB, submodule checkouts). Re-running the conformance suite there means re-copying `sim/L0/tests/assets`. The sources, the per-commit `git` log in `ws_fix`, every log and the measured patch remain (22 MB).
