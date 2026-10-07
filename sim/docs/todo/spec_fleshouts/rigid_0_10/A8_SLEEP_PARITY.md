> Appendix to `RIGID_SPEC.md`, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this appendix and `RIGID_SPEC.md` disagree, `RIGID_SPEC.md` wins.

# Rigid spec — sim-core sleeping at MuJoCo 3.5.0 parity (sleep re-forward, sleep timing, and the rest of the sleep pipeline)

Researcher scratch: `SCRATCH/rigid_spec/sleep_parity/` (SCRATCH = `$SCRATCH`).
Repo: read-only. `git status --short` empty before and after; HEAD was `ea11c8d6` at start and `139f7b25` at the end (the coordinator's docs commit landed meanwhile; `git diff --stat 3520544e HEAD -- sim/L0` is empty, so every `file:line` below is `main` @ `3520544e`).

## How this was checked, and what the method cannot see

| what | how | output |
|---|---|---|
| MuJoCo 3.5.0 sleep pipeline | source read: `engine_sleep.c`, `engine_forward.c` (`mj_advance`, integrators, `mj_fwdPosition`, `mj_fwdAcceleration`), `engine_core_smooth.c`, `engine_collision_driver.c`, `engine_core_constraint.c`, `engine_island.c`, `engine_io.c` (`mj_resetData`), `engine_setconst.c`, `user_model.cc`, `doc/programming/simulation.rst:1091-1215` | cited per item |
| per-step traces, MuJoCo vs ours | `py/trace2.py` (mujoco 3.5.0) and `probe_src/trace2.rs` (built against `main` and against the prototype); per step: `tree_asleep` before/after, `ntree_awake`, `nbody_awake`, `ncon`, `nefc`, `nisland`, `qpos`, `qvel`, `qacc`, `qacc_warmstart`, `sensordata`, `act`, and the callback log (`P` = passive, `C` = control) | `runs/*.{mj,main,proto}.jsonl`, `fixture_table.md` |
| fixture confirmation | every fixture also run with sleep disabled on both engines; a fixture is used only if its no-sleep trajectories agree before the sleep step | `runs_ns/` |
| prototype | scratch workspace `ws/` = the core_state researcher's patched copy (K1, K3, K4, K5, K6, K11 prototypes) + the core_callbacks K7 prototype (`k7_as_applied.diff`, its 3 rejects resolved by hand) + my changes (`prototype.diff`, 13 files, +514/−393) | `prototype.diff` |
| suites on the prototype vs its own base (base = same workspace without my diff) | `cargo test` in scratch, own target: sim-core `--lib`, `sim-conformance-tests --test integration` and `--test mujoco_conformance`, sim-mjcf; validator `example-sleep-wake-stress-test` | `t_ws*_*.out`, `val_sleep_ws*.out` |
| non-sleep bit identity, corpus | 1,584 corpus docs, harness `harness/src/main.rs`: 200 steps, `traj_fp` (qpos/qvel/act/time bits) and `aux_fp` (qacc, qacc_warmstart, sensordata, ncon, nefc, nisland bits, every step); base run twice + prototype once; the 71 docs known to vary across processes (ledger-L27) masked | `corpus_*.jsonl`, `py/corpus_cmp2.py` |
| corpus sleep docs vs MuJoCo | the 80 loadable corpus docs that enable sleep, 1,500 steps, MuJoCo vs prototype vs main, plus sleep-disabled copies to confirm the fixture | `corpus_sleep/`, `py/corpus_sleep_cmp.py` |
| checker made to fail | `py/summ.py` / `cmp2.py` report the mismatches on `main` (table below); the corpus comparison reported 11 load panics in prototype v1 (a flex-edge index bug, fixed) | — |

**What the method cannot see.** (a) Each proposed commit was not built or tested alone — only the full stack. (b) `clippy`, `grade`, `doc-theft`, licensed gates, `--release` nextest, sim-thermostat, therm-env, ml-chassis, sim-gpu, cf-design, sim-coupling suites, and every validator except `sleep-wake/stress-test` were not run on the prototype. (c) Not exercised against MuJoCo: multi-tree tendon wake (`mj_wakeTendon`), flex trees, mocap bodies, explicit `<pair>` filtering, the contact-filter callback order, sleep with keyframe resets, sensors other than framepos/framelinvel/frameangvel/jointpos/jointvel/actuatorfrc/accelerometer, `step1`/`step2` on a sleep step, finite differences with sleep. (d) Stale-array semantics were compared only for `qacc`, `qacc_warmstart`, `sensordata` and (two tests) `cfrc_int`/`qfrc_fluid` — not `qfrc_bias`, `cvel`, `cacc`, `efc_*` of sleeping bodies. (e) The qpos-change wake moved before CRBA (SP-5) matches MuJoCo on `rbox`, but that fixture also sets `qvel`, so the move's own effect is not isolated.

---

## 0. Why the sleep timing differed ("step 69 ours vs 76 MuJoCo")

**The A2 fixture is not a sleep fixture.** Its box starts exactly touching the plane (`pos="0 0 0.1"`, half-size 0.1). MuJoCo emits the 4 contacts at `dist = 0.0` and excludes them as "in gap" (`con->exclude = (con->dist >= includemargin)`, `engine_collision_driver.c:1414-1415`; excluded contacts make no rows, `engine_core_constraint.c:1939-1941`) — measured: `exclude 1` on all 4, `nefc 0` (`py/gap_check.py`). Ours assembles every contact (`constraint/assembly.rs:237-244`): `nefc 16` at step 0. After step 0 MuJoCo's box falls freely (`qvel_z = −0.01962`), ours does not (`−5.3e-4`) (`py/box_mj.jsonl` vs `box_ours.jsonl`). The first differing quantity is therefore `nefc` at step 0, a contact rule outside this area (§9, O1); the 69-vs-76 gap is mostly that.

**On confirmed fixtures** (box at `z = 0.0995` or dropped from 0.15: no-sleep trajectories agree to ≤ 1e-17 until velocities reach ~1e-10, `runs_ns/`), the first differing sleep quantity is the **countdown `tree_asleep` at step 0**: MuJoCo −10, ours −11. Cause: MuJoCo decides sleep inside `mj_advance`, after the activations and **before** the velocity update, so it tests the `qvel` the step started with (`engine_forward.c:898-912`); ours decides after `integrate()` (`forward/mod.rs:253-259`, `:196-201`), testing the integrated `qvel`. Ours sleeps one step early: 85 vs 86 (`box_rest`), 178 vs 179 (`box_drop2`), 71 vs 72 (`sstack`).

Larger gaps on other fixtures (`chain2` 4118 vs 2792, `act_sleep` 798 vs 636, `limit` 229 vs 219, `box_noisland` sleeps at 85 vs never) come together with other differences listed below (dof_length and the tolerance formula, the island guard, friction rows); each fixture's gap was not attributed to one item — the prototype changed them together, and every fixture then matched.

## Result of the prototype (all items below together)

Discrete fields = `tree_asleep`, `ntree_awake`, `nbody_awake`, `ncon`, `nefc`, `nisland`, callback log, compared at every step.

| fixture | steps | MuJoCo 3.5.0 transitions | main | prototype | prototype first discrete mismatch | prototype max abs diff qpos / qvel / qacc / sensordata |
|---|---|---|---|---|---|---|
| box_rest | 3000 | [86] | [85] | [86] | none | 1e-19 / 3e-18 / 2e-15 / 2e-18 |
| box_drop2 | 3000 | [179] | [178] | [179] | none | 9e-18 / 2e-15 / 2e-13 / 1e-16 |
| sstack (2 spheres, one island) | 3000 | [72] | [71] | [72] | none | 2e-18 / 5e-17 / 7e-15 / 3e-18 |
| swake (wake on contact) | 3000 | [54, 124, 248] | [53, 124, 247] | [54, 124, 248] | none | 9e-17 / 8e-16 / 2e-13 / 5e-17 |
| float (zero g) | 3000 | [9] | [9] | [9] | none | 0 / 0 / 0 / 0 |
| table (box on a static body) | 3000 | [86] | [85] | [86] | none | 8e-19 / 4e-16 / 4e-14 / 0 |
| chain2 (2 hinges, damped) | 6000 | [2792] | [4118] | [2792] | none | 5e-17 / 5e-16 / 9e-14 / 3e-16 |
| act (actuated, AUTO policy) | 3000 | [] | [] | [] | none | 1e-17 / 1e-16 / 3e-14 / 4e-16 |
| act_sleep (actuated, `sleep="allowed"`, filter dyn) | 3000 | [636] | [798] | [636] | none | 2e-18 / 8e-17 / 3e-14 / 2e-16 |
| box_rest implicit / implicitfast | 3000 | [86] | [85] | [86] | none | 1e-17 / 7e-15 / 2e-12 / 2e-16 |
| box_rest RK4 | 3000 | [83, 84, 93, 94]…584 | [] | [83, 84, 93, 94]…584 | none | 1e-13 / 5e-12 / 9e-10 / 5e-12 |
| box_noisland (`island="disable"`) | 3000 | [] | [85] | [] | none | 1e-13 / 5e-12 / 8e-10 / 5e-12 |
| box_never / box_init | 3000 | [] / [] | [] / [] | [] / [] | none / none | ≤1e-13 / 0 |
| eqpair (connect between 2 spheres) | 3000 | [54, 271] | [53, 270] | [54, 271] | none | 1e-15 / 1e-14 / 1e-12 / 3e-18 |
| limit (hinge on a limit + frictionloss; free sphere) | 3000 | [219] | [229] | [219] | none | 4e-15 / 4e-15 / 5e-13 / 4e-15 |
| box_uw_qvel / xfrc / qpos / negzero (user wakes at 200) | 600 | [86,200,229] / [86,200,213] / [86,200,209] / [86,200] | [85] / [85,200,212] / [85,200,209] / [85,200,209,…81] | identical to MuJoCo | none | ≤1e-13 / ≤6e-12 / ≤8e-10 / ≤6e-12 |
| rbox (non-cube box rotated + spun while asleep) | 600 | [86, 200, 238] | [85, 200, 245] | [86, 200, 238] | none | 3e-16 / 7e-16 / 9e-14 / 7e-16 |

Callbacks on the sleep step (`box_rest`, step 86): MuJoCo `PCPC`, prototype `PCPC`, main `CP` (no extra pass, sleeps at 85). RK4 sleep step (83): MuJoCo and prototype `PCPCPCPCPC`, main `CPPPP`. Sensor `framelinvel` after the sleep step: MuJoCo and prototype `0.0`; main keeps `5.39e-05` for the sleeping body.

MuJoCo **3.4.0** gives bit-identical traces to 3.5.0 on `box_rest`, `swake`, `chain2`, `box_rest_RK4`, `eqpair` (`runs/*.mj340.jsonl`); 3.4.0→3.5.0 changes in this code are FLEXVERT cases and history buffers only (source diff), so the golden-data pin at 3.4.0 does not conflict.

**Corpus, sleep-enabled docs (80):** 70 have no-sleep trajectories that agree with MuJoCo before the first sleep; on those, the prototype's transitions equal MuJoCo's in **70/70** (main: 41/70). 9 of the 70 still show an `ncon` mismatch — all at a contact with `dist` exactly 0 (sphere on plane at `z = r`): MuJoCo counts it in `ncon` and excludes it from rows, ours does not emit it (`0e9ce360330d515c` from `sleeping.rs:251`, `7ad5a27cd99555d3` from `sleeping.rs:2498`); same rule as O1, `nefc` agrees. 5 docs cannot run on MuJoCo (load failures), 5 diverge before sleeping.

**Corpus, sleep-disabled docs:** of 1,328 loadable docs that are deterministic across processes and do not enable sleep, `traj_fp` and `aux_fp` are identical between base and prototype for **all 1,328** (integrators: Euler 1,165, RK4 93, ISD 30, Implicit 27, ImplicitFast 13). Model fields that change: `dof_length` (1,087 docs) and the tree tables (240 docs) — read only by sleep. With sleep forced on for every loadable doc (1,407), neither base nor prototype errs or panics. 18 small no-sleep fixtures (`models_ns/`): 17 byte-identical over 1,000 steps on every traced field; `table` differs only in `tree_asleep`/`ntree_awake` (its static body is no longer a tree).

---

## SP-1. Where sleep is decided; the re-forward on the sleep step

**Now.**
- `step` sleeps after integrating: `crate::island::mj_sleep` + `mj_update_sleep_arrays` at `forward/mod.rs:253-259`; `step2` the same at `:196-201`. The test reads the integrated `qvel` (§0).
- No forward pass runs after trees fall asleep; `sleep_trees` (`island/sleep.rs:175-212`) instead zeroes `qvel`, `qacc`, `qfrc_bias`, `qfrc_passive`, `qfrc_constraint`, `qfrc_actuator`, `cvel`, `cacc_bias`, `cfrc_bias` and re-syncs poses with its own FK copy (`sync_tree_fk`, `:217-284`). Sensors keep the pre-sleep velocity values (main `framelinvel` 5.39e-05 on a sleeping box).
- `Data::integrate` is `pub fn integrate(&mut self, model: &Model)` (`integrate/mod.rs:39`); RK4's final advance (`integrate/rk4.rs:148-210`) integrates every dof and has no sleep step.
- Islands are discovered in the acceleration stage (`forward/mod.rs:547-551`).

**Target (MuJoCo 3.5.0).** `mj_advance` (`engine_forward.c:833-939`), called by `mj_Euler` (`:1008`), `mj_implicit` (`:1348`) and `mj_RungeKutta` (`:1121`, after resetting qpos/qvel/act/time to the step's start, `:1114-1118`): history → activations → `mj_sleep`; if any tree slept, `mj_forwardSkip(m, d, mjSTAGE_POS, 0)` then `mj_updateSleep` (`:898-905`) → velocity update of awake dofs only, with the acceleration computed BEFORE the re-forward (`qacc` is the integrator's local buffer, `:907-912`) → positions of awake bodies only (`:914-918`) → time → plugins → warm-start copy of `d->qacc`. During the re-forward the sleep arrays still mark the slept trees awake (they are updated after it), so their velocity- and acceleration-stage quantities, sensors and both callbacks are recomputed at `qvel = 0`. MuJoCo's documentation states the requirement: "on any timestep where islands are put to sleep, all velocity-dependent quantities must be recomputed before the sleep state is propagated using a call to mj_forwardSkip" (`doc/programming/simulation.rst:1130-1132`). Islands are a position-stage product (`mj_island` after `mj_makeConstraint`, `engine_forward.c:204-205`), so the re-forward keeps them.

**Change** (prototype: `integrate/mod.rs`, `integrate/rk4.rs`, `forward/mod.rs`).
```rust
// integrate/mod.rs
pub fn integrate(&mut self, model: &Model) -> Result<(), StepError>;     // was -> ()
pub(crate) enum VelSource { Acc(Option<DVector<f64>>), Set(Option<DVector<f64>>) }
pub(crate) fn sleep_and_reforward(&mut self, model: &Model, source: VelSource)
    -> Result<VelSource, StepError>;
// forward/mod.rs
pub(crate) fn forward_skip_unchecked(&mut self, model: &Model, skipstage: MjStage, skipsensor: bool)
    -> Result<(), StepError>;       // forward_skip = K1's check + this
```
- `integrate`: activations (unchanged) → compute the velocity source (Euler: `qacc` or the eulerdamp solve `rhs`; Implicit/ImplicitFast/ISD-Newton/RK4-fallback: `qacc`; ISD non-Newton: `qvel = scratch_v_new`, K3's path) → `sleep_and_reforward` (if `mj_sleep` slept any tree: snapshot the source — the re-forward overwrites `qacc` and `scratch_v_new` — then `forward_skip_unchecked(Pos, false)`, then `mj_update_sleep_arrays`) → velocity update over `dof_awake_ind` → positions (already skip `Asleep` bodies, `integrate/euler.rs:69-72`) → time → plugins. No allocation on a step where nothing sleeps (the snapshot is taken only when a tree slept).
- `mj_runge_kutta`: after the B-combination, set `time = t0`, `qpos = rk4_qpos_saved`, `qvel = rk4_qvel[0]`; activations; `mj_sleep` + re-forward + arrays; `qvel += h·dX_acc` for awake dofs; positions from the saved state, then restore the joints of sleeping bodies (equivalent to MuJoCo's `mj_integratePosInd` with `body_awake_ind`); normalize; `time = t0 + h`; plugins (K5).
- `step`/`step2`: delete the post-integration sleep blocks; `self.integrate(model)?`.
- `forward_pos`: `mj_island` moves to the end of the position stage (after the equality wake, before body transmissions), still gated on `ENABLE_SLEEP` (keeps non-sleep models bit-identical; see O3); deleted from `forward_acc`.
- sim-mjcf: delete `guard_rk4_sleep` and its call (`builder/build.rs:52`, `:963-969`) — see SP-9.
- `derivatives/hybrid.rs`: the 8 `scratch.integrate(model);` calls become `scratch.integrate(model)?;` (all inside `Result` functions; measured: compiles).

If the re-forward fails (implicit factorization), `integrate` returns `Err` with activations advanced and the slept trees' `qvel` zeroed; positions not advanced. MuJoCo has no failure path there. Document on `integrate`.

**Tests to add** (all MuJoCo values measured with 3.5.0, `py/trace2.py`; "main" = measured with `probe_main`):
- `sleep_step_matches_mujoco`: `box_rest` model (`models/box_rest.xml`) → the first `step()` call (0-based) after which `tree_asleep[0] >= 0` is 86; main 85 (FAILS).
- `sleep_step_reforward_fires_both_callbacks`: same model, per-step log on step 86 = `"PCPC"`, other steps `"PC"`; main `"CP"` (FAILS).
- `sleep_step_sensors_see_zero_velocity`: `framelinvel` of the box after step 86 is exactly `[0, 0, 0]`; main `5.392e-05` (FAILS).
- `sleep_step_matches_mujoco_implicit` (implicit, implicitfast: 86; main 85, FAILS) and `sleep_wakes_and_sleeps_with_contact` (`sstack`: 72; main 71, FAILS).

**Tests/examples that flip:** see SP-7 and SP-9 (all flips are listed there by cause).

**Downstream.** `Data::integrate` callers: only sim-core (`forward/mod.rs:194,249`, `derivatives/hybrid.rs:2894,2914,3014,3034,3079,3099,3306,3322`, tests `derivatives/fd.rs:765`, `plugin.rs:615`) — `git grep -n "\.integrate("` over `sim examples design tools`: the other hits are `soft-explicit`/`sim-gpu`/`cf-sim-research` executors with a different signature. sim-thermostat: a passive callback now also fires on the re-forward of a sleep step (MuJoCo does the same); `git grep ENABLE_SLEEP sim/L0/thermostat sim/L0/therm-env` has no hits, so no in-tree thermostat model sleeps.

## SP-2. The tolerance test and `dof_length`

**Now.** `tree_velocity_below_threshold` (`island/sleep.rs:157-166`): a dof blocks sleep when `|qvel| > sleep_tolerance · dof_length` (velocity DIVIDED by length, non-strict). `dof_length` for rotational dofs = a subtree extent built from `body_pos` distances, 1.0 when below 1e-10 (`types/model_init.rs:1240-1264`). Measured: `chain2` ours `[0.3, 1.0]`, MuJoCo `[0.18, 0.18]`; a free box ours `1.0`, MuJoCo `0.1732` on its rotational dofs.

**Target.** `treeCanSleep` (`engine_sleep.c:125-152`): policy, then `xfrc_applied` and `qfrc_applied` of the tree compared **bytewise** to zero (`mju_isZeroByte`; `−0.0` blocks), then with `tol > 0` `isSmaller`: `max_i(dof_length_i · |qvel_i|) < tol` (velocity MULTIPLIED by length, strict, early exit at `max >= tol`, `:112-121`); with `tol == 0` every `qvel` byte zero. Body size (`setStat`, `engine_setconst.c:976-1025`, at `qpos0`): max over joints of the distance from the body's (and its parent's) centre of mass to the joint anchor; then max with `geom_rbound + |com − geom_xpos|` over geoms with `rbound > 0` (planes have `rbound 0` in MuJoCo, measured); then flex edge rest lengths; every body at least 1e-5; world 0. Rotational dofs take their body's size (`:1029-1043`).

**Change.** `tree_can_sleep(model, data, tree, tol: f64) -> bool` as above (`to_bits() != 0` for the byte tests). `compute_body_lengths` reimplemented as `setStat` (it runs `make_data` + `mj_fwd_position` at `qpos0`; ours stores planes as `rbound = ∞`, so the geom term skips non-finite radii). `pub fn compute_dof_lengths(model: &mut Model)` keeps its signature; it now needs a Model that can `make_data`. Prototype: `types/model_init.rs` (+61/−).

**Tests to add.** `dof_length_matches_mujoco`: `chain2` → `[0.18, 0.18]`; free box → `[1, 1, 1, 0.17320508075688776 ×3]`; the 2-hinge doc in `py/` (`hinge pos="0 0 0.3"`) → `[0.3354101966249685, 0.2]` — main FAILS on all three. `rotational_sleep_matches_mujoco`: `chain2` with `qvel = [1.5, −1.0]` at step 0 sleeps at 2792; main 4118 (FAILS). `negative_zero_force_blocks_sleep`: `box_uw_negzero` (`qfrc_applied[2] = −0.0` at step 200) → woken at 200, never sleeps again in 600 steps; main sleeps again at 209 and cycles (FAILS).

**Flips.** `sleeping.rs:1025` `test_dof_length_computation`, `:1399` `_hinge_1m`, `:1428` `_hinge_01m`, `:1487` `_free_joint`, `:1533` `_nonuniform_threshold` (all CI-run; they pin the old formula — rewrite them to MuJoCo's values).

**Downstream.** cf-design calls `sim_core::compute_dof_lengths` (`design/cf-design/src/mechanism/model_builder.rs:952`); its trees are all `SleepPolicy::Never` (`:1218`), so only the stored value changes.

## SP-3. Kinematic trees and automatic sleep policies

**Now.** One tree per world child (`body_rootid`), including static bodies with no dofs (`sim-mjcf builder/build.rs:690-745`; K6 moves this unchanged into `Model::compute_kinematic_trees`). Measured: `table.xml` ours `ntree 2` (the static table is a tree that "sleeps" vacuously after 10 steps), MuJoCo 1. A static root with two moving children is one tree in ours, two in MuJoCo. Static bodies get `SleepState::Awake` (`island/sleep.rs:426-439`, "No tree info → treat as awake"). Policies (`build.rs:810-957`): actuated trees → AutoNever; a multi-tree tendon with stiffness, damping **or a limit** marks its first two trees; explicit policies applied after AUTO; a non-root `sleep` attribute warns and propagates; no flex rule.

**Target.** A tree starts at every dof whose parent dof is none (`user_model.cc:3006-3013`, counted at `:2100-2124`); `body_treeid` = the weld body's first dof's tree, else −1 (`engine_setconst.c:108-116`); tree body/dof address tables scanned in order (`:118-139`). Static bodies are `mjS_STATIC` (a mocap root `mjS_AWAKE`) and are listed in `body_awake_ind` (`engine_sleep.c:62-82`). User policies first (`user_model.cc:3036-3047`, error "sleep policy only allowed for movable root bodies"); AUTO resolution only touches AUTO trees (`engine_setconst.c:158-279`): actuated (joint/site/slider-crank's first site/body/every tree of a tendon), a tendon spanning > 2 trees, or 2 trees with stiffness or damping (limits do not count; an explicit `allowed`/`init` there is an error, `:232-243`), trees holding flex bodies.

**Change.** `compute_kinematic_trees` (K6's `types/model_trees.rs`) computes MuJoCo's tables from `dof_parent`, `body_weldid`, `body_dof_adr/num`; `resolve_auto_sleep_policies` drops the limit rule, marks every tree of a qualifying tendon, adds the flex rule; `mj_update_sleep_arrays` sets `SleepState::Static` for tree-less bodies (`Awake` for a mocap root). `dynamics/rne.rs:362` indexes `tree_awake[body_treeid[b]]` — with static bodies tree-less this panics for a gravcomp body on a static body while sleep is enabled; guard with `tree < ntree` (found by `grep`, not by a failing run). sim-mjcf: the two MuJoCo compile errors (non-root `sleep`, explicit policy on a tendon-coupled tree) belong in the MJCF series (A5's validation area) with `MjcfError`.

**Tests to add.** `static_body_is_not_a_tree`: `table.xml` → `ntree == 1`, `body_sleep_state[table] == Static`; main `ntree 2` (FAILS). `box_on_static_body_sleeps_as_mujoco`: 86; main 85 (FAILS).

**Flips.** None in the suites run (the `ntree` assertions at `sleeping.rs:311,349,711,1925,4412,4587` pass on the prototype). Corpus: tree tables change for 240 non-sleep docs (no trajectory change).

**Downstream.** cf-design's own tree copy (`model_builder.rs:~1160-1226`) — K6 replaces it with `compute_kinematic_trees`, so cf-design models get MuJoCo's trees after this commit (not run). `sim/L1/bevy/src/examples.rs:1052-1055` colours by `body_sleep_state` (`Static` arm exists).

## SP-4. Wake rules

**Now** (`island/sleep.rs`). `mj_wake` (`:515-546`): applied forces only, called only when sleep is enabled (`forward/mod.rs:451`); a user `qvel` edit does not wake (measured `box_uw_qvel`: main never wakes). `mj_check_qpos_changed` runs after CRBA (`forward/mod.rs:464-469`), so a woken tree's mass-matrix rows were skipped that pass (CRBA skips sleeping trees, `dynamics/crba.rs:54-71`). `mj_wake_collision` (`:551-580`): every contact incl. flex; static partners judged by `body_sleep_state`; the woken cycle gets `-(1+MIN_AWAKE)`. `mj_wake_tendon` (`:620-665`) also merges two sleeping cycles. `mj_wake_tree` (`:723-753`) always sets `-(1+MIN_AWAKE)` and eagerly updates `tree_awake`/body states.

**Target** (`engine_sleep.c`). `mj_wake` (`:240-275`): sleep disabled → wake every sleeping tree; else a sleeping tree wakes when its `tree_awake` mark is set (pose mismatch from `mj_kinematics1`, `engine_core_smooth.c:162-179`) or `!treeCanSleep(…, 0)` — policy never, any force byte or any `qvel` byte non-zero; it runs between `kinematics1` and `kinematics2` (`:236-241`), before CRB. `mj_wakeCollision` (`:279-329`): geom–geom contacts only, static partner (tree −1) skipped, the woken cycle takes the AWAKE tree's countdown (`wakeval`). `mj_wakeTendon` (`:333-362`): only awake≠asleep, with `wakeval`. `mj_wakeEquality` (`:366-455`): `kAwake`; both asleep in different cycles → wake both. `mj_wakeTree` (`:191-234`): awake tree → `min(wakeval, current)`; derived arrays refreshed by the caller.

**Change.** `mj_wake` rewritten (always called from `forward_pos`); `mj_wake_tree(data, tree, wakeval: i32) -> usize` (no eager array update); collision/tendon/equality as above; `mj_check_qpos_changed` + `mj_update_sleep_arrays` moved to right after `mj_fwd_position`, before flex/CRBA.

**Tests to add.** `wake_on_contact_inherits_countdown`: `swake` → `tree_asleep[A] == −10` after step 124, final sleep at 248; main −11 and 247 (FAILS). `user_qvel_wakes_sleeping_tree`: `box_uw_qvel` → transitions `[86, 200, 229]`; main `[85]` (FAILS). `user_qpos_and_xfrc_wake`: `box_uw_qpos` `[86,200,209]`, `box_uw_xfrc` `[86,200,213]` (main `[85,…]`, FAILS). `disabling_sleep_wakes_all`: set `ENABLE_SLEEP` off on a sleeping model → next `forward` leaves every `tree_asleep < 0` (source-derived from `engine_sleep.c:243-250`; not run on either engine).

**Flips.** None in the suites run.

## SP-5. Sleeping trees in collision, constraints and islands

**Now.** Broad phase skips only asleep–asleep pairs, after the contact-filter callback (`collision/mod.rs:518-540`); explicit pairs likewise (`:586-593`). Measured: a box asleep on the plane keeps `ncon 4`, `nefc 16` (main, `box_rest` steps 87+). Assembly has no sleep filter (`constraint/assembly.rs:137-230`, `:284-…`); `mj_fwd_constraint` zeroes `qacc`/`qfrc_constraint` of sleeping trees afterwards (`constraint/mod.rs:474-492`). `mj_sleep` has no island guard (`island/sleep.rs:69-119`); island edges come from contacts, equalities, joint and tendon limits (`island/mod.rs:58-149`) — no friction rows.

**Target.** `filterBodyPair` (`engine_collision_driver.c:160-187`, before narrow phase and the callback): skip asleep–asleep and asleep–welded-to-world; explicit pairs skipped unless one body is AWAKE (`:1465-1472`); docs: "all contacts within an island and between the island and static bodies are skipped" (`doc/computation/index.rst:1745-1750`). Rows skipped for sleeping equalities (neither object awake), dof friction and joint limits (`engine_core_constraint.c:389-413, 700-716, 767-783`, counts `:1657-1675, 1817-1827, 1858-1867`); tendon friction/limit rows are not filtered. `mj_sleep`: `if (d->nefc && !nisland) return 0` (`engine_sleep.c:507-510`). Islands come from every row incl. friction (`engine_island.c:316-370`).

**Change.** Collision filter as MuJoCo using `model.body_weldid` (moved before the callback). Assembly: `crate::island::constraint_sleep_filter`, `equality_asleep`, `dof_asleep` (array state) in both the count and row phases (`assembly.rs`, `equality_assembly.rs`). `mj_sleep` guard on `!efc_type.is_empty() && nisland == 0`. `mj_island`: dof-friction self-edges and tendon-friction edges. The guard needs both: without friction edges, `limit.xml` never counts down (measured on prototype v1: `asleep_after [-11,-11]` at step 0 vs MuJoCo `[-10,-10]`).

**Tests to add.** `asleep_on_static_makes_no_contacts`: `box_rest` steps 87+ → `ncon 0`, `nefc 0`; main `4`/`16` (FAILS). `sleeping_rows_dropped`: `eqpair` → `nefc 0` from step 55; `limit` → `nefc 0` from step 220 (prototype v1 without the filters: 3 and 2). `island_disabled_blocks_sleep_with_constraints`: `box_noisland` → never sleeps in 3000 steps; main sleeps at 85 (FAILS). `friction_rows_make_islands`: `limit` sleeps at 219; main 229 (FAILS).

**Flips.** `sleeping.rs:3669` `test_island_delassus_equivalence` (CI): its `DISABLE_ISLAND` copy no longer sleeps (MuJoCo's guard), so `qfrc_constraint` diverges at step 236 — rewrite to compare with sleep disabled in both, or only until the first sleep.

## SP-6. Init-sleep (`sleep="init"`)

**Now.** `reset_sleep_state` → `validate_init_sleep` (`island/sleep.rs:294-386`): union-find over equalities and multi-tree tendons, `sleep_trees` + FK sync, no forward pass; on failure `log::warn!` and every tree awake. Measured `box_init`: framepos sensor `0.0` (never computed), MuJoCo `0.0995`. A contact-coupled mixed island (`models/initmix_contact.xml`) and an equality-coupled one (`initmix.xml`) both load on main.

**Target.** `mj_resetData` (`engine_io.c:1443-1505`): with sleep enabled and any `init` tree, run `mj_forward`, set init trees to −1 and the rest to `kAwake`, `mj_sleep`; if fewer than all init trees slept, **error**; then clear `qacc_smooth`, `qfrc_smooth`, the constraint arena. Measured: MuJoCo 3.5.0 REFUSES both `initmix` models at load (`"1 trees were marked as sleep='init' but only 0 could be slept"`), since compiling builds an `mjData`. Docs: `simulation.rst:1137-1145`.

**Change (prototype).** `reset_sleep_state`: all `kAwake`, arrays; if sleep enabled and any `Init`: `data.forward(model)`, mark, `mj_sleep`, warn on a short count, zero `qacc_smooth`/`qfrc_smooth`. Failure handling is Open Q3. `validate_init_sleep` becomes dead (kept `#[allow(dead_code)]` in the prototype; delete).

**Tests to add.** `init_tree_has_reset_values`: `box_init` → framepos z after step 0 is `0.0995`; main `0.0` (FAILS). `init_mixed_island_refused` (per Q3): `initmix.xml`, `initmix_contact.xml` → refused; main loads both (FAILS).

**Flips.** `fluid_forces.rs:1495` `t47_wind_sleep_qfrc_fluid_zero` (CI): asserts `qfrc_fluid[6] == 0` for an init-asleep body in wind; prototype `4.09955742875642726e-3`, MuJoCo 3.5.0 `0.004099557428756427` (measured, `py/flipcheck.py`: the reset's forward computes it, and sleeping dofs keep it).

## SP-7. Arrays of sleeping dofs (`qacc`, `qacc_smooth`, warm start)

**Now.** Sleeping dofs report `qacc = 0` (`constraint/mod.rs:474-492`, and `sleep_trees`).

**Target (measured).** MuJoCo computes `qfrc_smooth`/`qacc_smooth` for awake dofs only (`engine_forward.c:574-605`), so a sleeping dof keeps the values from its last awake pass (the re-forward); its `qacc` is that stale `qacc_smooth` (copied whole when `nefc == 0`, `:746-751`, and for dofs outside islands, `:670-676`). Measured on `box_rest`: sleep step `qacc_z = 0.00343` (re-forward at `qvel = 0`), afterwards `−9.81`; `qacc_warmstart` the same. Accumulators of the sleeping body: `cfrc_int = 0` (MuJoCo, `py/flipcheck.py`).

**Change (prototype).** `mj_fwd_constraint`: keep the previous `qacc_smooth`/`qfrc_smooth` entries of dofs whose tree is asleep per `tree_awake`; for those dofs `qacc = qacc_smooth`, `qfrc_constraint = qfrc_frictionloss = 0`. Using the arrays (not `tree_asleep`) is what lets the re-forward solve the trees that just slept. With this, `qacc` and `qacc_warmstart` match MuJoCo to ≤ 9e-10 on every fixture. Open Q1 decides whether to keep it.

**Flips (only under Q1 = parity).** `sleeping.rs:425` `test_sleep_zeros_velocity`, `:2365` `test_sleep_trees_zeros_all_arrays`, `:3276` `test_indirection_vel_integration_equivalence`, `:5038` `test_partial_ldl_solve_zero_sleeping_rhs` (all assert `qacc == 0` for a sleeping dof; prototype `−9.81`); `body_accumulators.rs:366` `sleep_state_body_accumulators` and `sensors_phase4.rs:629` `d12_acc_sensor_sleep_computes_all` (assert `cfrc_int` of a sleeping body non-zero; MuJoCo and prototype 0); validator `examples/fundamentals/sim-cpu/sleep-wake/stress-test/src/main.rs:285` "Sleeping qacc = 0" (prototype `max |qacc| = 1.50e1`, FAIL — CI validate-examples). The sleep-vs-no-sleep equivalence pins `sleeping.rs:2790` `test_disable_island_bit_identical`, `:3365` `test_all_awake_bit_identical`, `:4086` `test_selective_crba_all_awake_noop`, `:4842` `test_partial_ldl_all_awake_noop` compare runs where nothing sleeps; they pass unchanged on the prototype.

## SP-8. `Data::integrate` signature

`pub fn integrate(&mut self, model: &Model) -> Result<(), StepError>` (breaking; A1 had listed `integrate` as "documented, unchanged" because it returned `()`). The sleep step needs a forward pass inside the advance (SP-1); its error must surface. Alternative in Open Q4.

## SP-9. RK4 with sleep

**Now.** sim-mjcf disables `ENABLE_SLEEP` for RK4 with a `warn!` (`builder/build.rs:963-969`); a code-built RK4 model with sleep runs the end-of-step sleep with an unfiltered RK4 advance. Measured `box_rest_RK4`: main never sleeps.

**Target (measured).** MuJoCo sleeps under RK4 through the same `mj_advance` — and wakes the tree on the next step, every time: the stored poses come from the last RK4 stage (the re-forward skips the position stage), so `mj_kinematics1` sees a pose mismatch (measured `py/rk4_wake.py`: stored `xpos_z 0.09989117078535442` vs FK(qpos) `0.09989108144712382` after the RK4 sleep step; under Euler they match bit for bit). Result: 584 sleep/wake transitions in 3000 steps; 3.4.0 identical. MuJoCo marks sleeping "a new feature (Nov 2025) that is subject to change and may have latent bugs" (`simulation.rst:1182`).

**Change (prototype = parity).** Remove the guard; RK4 advance as SP-1. Prototype matches all 584 transitions. A variant that re-syncs poses with the existing `sync_tree_fk` at sleep time still wakes at step 84 (measured; its poses are not bit-equal to `mj_fwd_position`'s), so a "stays asleep" deviation needs a bit-exact pose refresh — not prototyped. Open Q2.

**Flips.** `sleeping.rs:666` `test_rk4_sleep_warning` (pins the guard: asserts `enableflags & ENABLE_SLEEP == 0` after loading an RK4 model; prototype keeps the flag, `32 = 1 << 5`).

---

## Commit list (after K7; each to carry its tests and flips; per-commit greenness NOT measured — only the stack)

| # | commit | items | flips |
|---|---|---|---|
| S1 | `fix(sim-core): kinematic trees and automatic sleep policies as MuJoCo` | SP-3 (+ `rne.rs:362` guard) | none measured |
| S2 | `fix(sim-core): dof_length from MuJoCo's body sizes; MuJoCo's sleep tolerance test` | SP-2 | `sleeping.rs` dof_length ×5 |
| S3 | `fix(sim-core): sleep decided in the advance; re-forward on the sleep step` (breaking: `integrate` returns `Result`) | SP-1, SP-5, SP-7, SP-8, SP-9 (sim-mjcf guard deleted) | SP-5, SP-7, SP-9 lists |
| S4 | `fix(sim-core): wake rules and init-sleep as MuJoCo` | SP-4, SP-6 | `fluid_forces.rs:1495` |

Splitting S3 further (filters apart from the timing) was not tried; the island guard without the collision and constraint filters stops other trees sleeping while one sleeps on the floor (asleep contacts keep `nefc > 0` while the sleeping tree is outside every island — reasoning from the code, not run).

**Dependencies.** K1 (S3 splits K1's checked `forward_skip` into a checked wrapper + `forward_skip_unchecked`); K3 (S3's ISD path reads `scratch_v_new`); K5 (S3 rewrites the RK4 tail where K5 adds the plugin advance); K6 (S1 edits `compute_kinematic_trees`; keep K6 a pure move so its corpus A/B stays "identical"); K7 (the re-forward fires `PC` only with K7's `forward_skip`; without K7 it fires `P`). K8/K9 touch `derivatives/hybrid.rs` call sites of `integrate` (textual). K14 docs: replace A2's `CbPassive` line "Not matched: on the step that puts a tree to sleep…" with "on the step that puts a tree to sleep, the forward pass runs again from the velocity stage, so both callbacks fire twice (also in `step2`)"; drop the sleep rows from the divergences list; rewrite the `island/sleep.rs` module docs ("Called at the end of `step()`", `:57-58`). MJCF series: the two compile errors of SP-3 and Q3's load refusal need `MjcfError` (M1).

## Open questions

**Q1 — `qacc` of a sleeping dof: MuJoCo's stale `qacc_smooth` or 0.** (a) parity (prototype): `qacc` = last unconstrained acceleration (−9.81 for a box at rest), `qacc_warmstart` bit-comparable to MuJoCo. (b) keep 0 and list it as a readout divergence. Measured: zeroing `qacc` (in the `nefc > 0` path) changed post-wake `qpos`/`qvel` by ≤ 1.2e-14 on `eqpair` — warm start did not move the solve there; MuJoCo's docs say a woken island "will behave exactly as if it was awake all along" (`index.rst:1749-1750`). Recommend (a), parity. If wrong: 6 CI tests + 1 validator check flip for a value users may read as "the body accelerates at −g".

**Q2 — RK4 with sleep.** (a) parity: the tree sleeps one step in ten at rest (prototype, matches 584 MuJoCo transitions); (b) refuse `ENABLE_SLEEP` with RK4 (stricter; MuJoCo runs it; 1 in-tree doc, `sleeping.rs:118`); (c) make sleep stay (lenient deviation; needs a bit-exact pose refresh of slept trees; the naive one failed). Recommend (a): it is defined, testable against MuJoCo bit for bit, and today's silent disable is itself "other than asked". If wrong: RK4 users who enable sleep get periodic one-step naps instead of a refusal or a lasting sleep.

**Q3 — init trees that cannot sleep.** MuJoCo refuses at load (measured, both contact- and equality-coupled). (a) refuse: `MakeDataError::InitSleep { tree, root_body }` from K5's `try_make_data`, with sim-mjcf's `load_model` returning it as an `MjcfError` (it already builds Data for `compute_invweight0`); `make_data` panics (documented); `Data::reset` keeps `()` (the same model resets the same way). (b) keep warn-and-continue (deviation). Recommend (a). If wrong: a file MuJoCo refuses loads here with the init tree left ready (−1) but awake.

**Q4 — `integrate` returning `Result`.** (a) as SP-8; (b) keep `()`, run the sleep block in `step`/`step2` before `integrate`. (b) breaks MuJoCo's order: activations advance before the re-forward in MuJoCo (`engine_forward.c:886-905`), so actuator forces and `actuatorfrc` sensors on the sleep step would use the old `act` (measured `act_sleep` matches MuJoCo only with the activation update first). Recommend (a).

**Q5 — islands when sleep is disabled.** MuJoCo computes islands whenever `nefc > 0` and islands are not disabled (`engine_island.c:378-380`); ours only with sleep. Measured: `data.nisland` 1 vs 0 on `box_rest` without sleep; dynamics agree (our solver ignores islands, `constraint/mod.rs:49-54`). Recommend leaving it (keeps non-sleep models bit-identical) and listing it as a readout divergence. If wrong: `nisland()` reads 0 for every non-sleep model.

## 9. Found outside this area (not fixed here)

- **O1 zero-distance contacts.** MuJoCo excludes contacts with `dist >= includemargin` from rows (`engine_collision_driver.c:1414-1415`, `engine_core_constraint.c:1939-1941`); ours makes rows for every contact (`assembly.rs:237-244`), and ours does not emit a sphere–plane contact at `dist == 0` that MuJoCo emits (and excludes). Causes A2's 69-vs-76 and the residual `ncon` mismatches in 9 corpus sleep docs. No row owns it.
- **O2 `xfrc_applied` layout.** MuJoCo stores force then torque (`mjdata.h:266`; the trace needed index 8 for force-z); ours reads `[torque, force]` (`constraint/mod.rs:96-97`; index 11). The `box_uw_xfrc` comparison uses each engine's layout.
- **O3** as Q5.
- **O4** `rne.rs:362` (SP-3).
