> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Area: actuator `lengthrange` at MuJoCo 3.5.0 parity

Researcher section for the Rigid spec. Read-only on the repo (HEAD `ea11c8d6` at start, `139f7b25` at the end — a docs-only commit to RIGID_SPEC.md made by someone else meanwhile; code = `main` @ `3520544e` throughout, `git diff --stat 3520544e HEAD -- sim/L0 design tools examples` empty; `git status --short` empty before and after). Scratch: `$SCRATCH/rigid_spec/lengthrange/` (`core_lr/`, `mjcf_lr/` = patched copies of `sim/L0/core` and `sim/L0/mjcf`; `harness_{orig,lr}/` = per-actuator lengthrange + 100-step trajectory dump; `scripts/oracle.py` = the same dump through `mujoco==3.5.0`; `scripts/compare.py`, `scripts/table.py`; `cases*/` synthetic docs; `out/` every result quoted below).

MuJoCo citations are 3.5.0 (`$SCRATCH/mj350src/mujoco`, 881544c). `evalAct` and `mj_setLengthRange` are byte-identical in 3.4.0 (diffed), and 3.4.0's `mjCModel::LengthRange` installs the same flags (`user_model.cc:2318-2319` at 3.4.0).

---

## 0. What was measured, and how

| what | method | result |
|---|---|---|
| corpus, main vs MuJoCo | `harness_orig` vs `oracle.py` on the 233 corpus docs containing `<actuator` | 207 docs load in both; 255 actuators: 21 `joint/fixed` + 1 `tendon/fixed` differ (ours copies limits, MuJoCo (0,0)), all 13 `joint/muscle` differ (≤ 1.14e-3), 9 muscle docs' 100-step trajectories differ (≤ 2.8e-4 in qpos) |
| corpus, port vs MuJoCo | `harness_lr` (patched copies) vs `oracle.py` | 255/255 actuators within **3.2e-13**; the 9 muscle trajectories within **5.1e-14** (qpos) |
| synthetic, port vs MuJoCo | 85 docs (`cases*/`): joint (hinge, slide, ball), jointinparent, fixed and spatial tendon, site with and without refsite, slider-crank, body (free-joint transmission not tested), gear ±, integrators, flags, `<compiler><lengthrange>` options, explicit values, unlimited joints, geometry-limited tendons | 75 agree in status (15 refused by both with the same message kind); of the 60 loaded by both, 59 agree in value (worst 1.9e-8, then 2.7e-10, then ≤ 7.3e-12) and 1 differs by 6.4e-3 from 3-value `solimplimit` parsing (§6.5); the 10 status differences all have a cause outside this area (§6) |
| non-convergence | MuJoCo's message, the port, and a closed form of the iteration | all three agree to 10 digits (§3) |
| tests | patched copies: sim-core lib (720), sim-mjcf lib (385), the 5 integration files that mention muscles (193), the `muscles/stress-test` validator | flips listed in §5; validator 51/51 PASS |

What this method cannot see: tests outside the 5 integration files (none mentions `muscle` or `lengthrange`, `git grep -il`), downstream crates' suites (cf-design, cf-osim, cf-mjcf-emit — not run), the 157 `format!` templates except those the run tests build, grade, clippy, and the Bevy muscle examples (never run in CI).

---

## 1. MuJoCo 3.5.0's algorithm (file:line)

**Entry.** `mjCModel::Compile` → after `mj_setConst` and `AutoSpringDamper`, `LengthRange(m, d)` (`user_model.cc:5126-5132`). `mj_setConst` explicitly excludes lengthrange (`engine_setconst.c:1088`). The data it gets was made with contacts and sleep cleared (`user_model.cc:5110-5112`).

**Options** (`mjLROpt`, `mjmodel.h:477-491`; defaults `engine_init.c:234-246`): `mode=muscle`, `useexisting=1`, **`uselimit=0`**, `accel=20`, `maxforce=0`, `timeconst=1`, `timestep=0.01`, `inttotal=10`, `interval=2`, `tolrange=0.05`. XML: `<compiler><lengthrange>` with exactly those 10 attributes, at most once (`xml_native_reader.cc:105-107`), read at `:1154-1176` (`mode` keyword map `none/muscle/muscleuser/all`, `:825-829`; the two flags via `bool_map`; the 7 numbers via `ReadAttr`, 1 value each). The per-actuator `lengthrange="lo hi"` attribute exists on every actuator element (`:391-438`) **but on no `<default>` actuator element** (`:184-206`) — measured: `<default><muscle lengthrange=…>` → "Schema violation: unrecognized attribute: 'lengthrange'". The explicit value is copied into the model as given (`user_model.cc:3784`), default (0,0), no check.

**`mjCModel::LengthRange`** (`user_model.cc:2408-2507`):
1. Saves `m->opt`, then **replaces** `disableflags` with `FRICTIONLOSS | CONTACT | SPRING | DAMPER | GRAVITY | ACTUATION` (`:2411-2412`; the user's own disable flags, e.g. `constraint`, `limit`, `autoreset`, are cleared for the run); `timestep = LRopt.timestep` if > 0, else the model's (`:2413-2415`). Restores `m->opt` at the end (`:2505`).
2. Single-threaded or one thread per chunk of actuators (`:2443-2502`); each thread has its own `mjData`, and every side starts from `mj_resetData`, so results do not depend on threading. The first failing actuator's message becomes a **compile error** (`:2447-2449`, `:2497-2501`).

**`mj_setLengthRange(m, d, index, opt, …)`** (`engine_setconst.c:1145-1272`):
1. **Mode filter** (`:1152-1162`): `ismuscle = gaintype==MUSCLE || biastype==MUSCLE`; `isuser` likewise with USER. `none` skips all; `muscle` skips `!ismuscle`; `muscleuser` skips `!ismuscle && !isuser`; `all` skips none. Skipped actuators keep whatever is in the model (explicit or (0,0)).
2. **useexisting** (`:1164-1166`): skip if `lo < hi`.
3. **uselimit** (`:1172-1199`): joint/jointinparent with a limited joint, or tendon with a limited tendon → copy `jnt_range`/`tendon_range` **raw (no gear)** and return.
4. **Simulation, two sides** (`:1201-1235`): side 0 pushes negative, side 1 positive. Per side: `mj_resetData` (qpos0, qvel 0, time 0); `while (time < inttotal)`: `len = evalAct(…)`; `if (time == 0)` → "Unstable lengthrange simulation in actuator %d"; if `time > inttotal − interval` (time *after* the step) track min/max with `lmin/lmax` initialised to 0 and an `updated` flag. Side 0 stores `lmin[0]`, side 1 stores `lmax[1]`.
5. **`evalAct`** (`:1107-1141`): `qvel *= exp(−h / max(0.01, timeconst))`; `mj_step1`; dense `actuator_moment` row; `x = M⁻¹·moment`; `qfrc_applied = moment · (2·side−1)·accel / max(mjMINVAL, ‖x‖)` (so `‖M⁻¹ qfrc‖ = accel`); if `maxforce > 0` and `‖qfrc‖ > maxforce` rescale; `mj_step2` (RK4 integrates with Euler, `engine_forward.c:1505-1510`). Returns `actuator_length` — computed by step1, i.e. at the configuration *before* the integration.
6. **Checks** (`:1238-1269`): `dif = hi − lo ≤ 0` → "Invalid lengthrange (%g, %g) in actuator %d"; `lmax[s] − lmin[s] > tolrange·dif` for side 0 then side 1 → "Lengthrange computation did not converge in actuator %d: eval (…) range (…)".

**The `time == 0` test can only fire when the step is 0.** A reset in `mj_checkPos/Vel/Acc` (`engine_forward.c:53-112`) zeroes time, but `mj_step2` then always integrates and `mj_advance` adds `timestep` (`engine_forward.c:921`), so after a reset the side restarts from qpos0 with `time = h`, not 0. Measured: a model `timestep="0"` with the default LR timestep 0.01 loads in MuJoCo (`cases5/lim1deg_timestep0`). No input I tried reached a reset during the run (2 degenerate probes, `cases6/`), so whether MuJoCo can loop forever there is not measured.

MuJoCo's documentation states the intent for the unlimited case: "if the actuator is attached to the joint, or to a fixed tendon equal to the joint, then it is unlimited. The compiler will return an error in this case" (`doc/modeling.rst:774-776`; also `doc/XMLreference.rst:960-964`), and lists the disabled forces (`modeling.rst:766-770`).

---

## 2. Items

### LR-1 — explicit `lengthrange` (A5-Q4, decided: honour it)

**Now.** Parsed on every actuator (`parser/actuator.rs:210-213`) and in defaults (`parser/defaults.rs:274-278`, merged `defaults.rs:442-443`, `:928`), then dropped: the builder pushes `(0.0, 0.0)` (`builder/actuator.rs:276-277`). Measured: muscle `lengthrange="-0.2 0.7"` → main (−0.5236, 1.0472) (the joint range); motor `"0.2 0.7"` → main (−0.5236, 1.0472).

**Target.** Copied as given (`user_model.cc:3784`); `useexisting` keeps it when `lo < hi`; otherwise (equal or backwards) a muscle is simulated and a non-muscle keeps the given pair. Measured MuJoCo: muscle `-0.2 0.7` → (−0.2, 0.7); motor `0.7 0.2` → (0.7, 0.2); muscle `0.5 0.5` and `0.7 0.2` → simulated (−0.524169, 1.04777). Port agrees on all four.

**Change.** `builder/actuator.rs:277`: `self.actuator_lengthrange.push(actuator.lengthrange.unwrap_or((0.0, 0.0)));`. The default-class plumbing (`MjcfActuatorDefaults.lengthrange`, `types.rs:778`; `parser/defaults.rs:274-278`; `defaults.rs:442-443`, `:928`) becomes unreachable once the schema pass refuses the attribute (MuJoCo has none, measured) — delete it in that commit (see Dependencies).

### LR-2 — the automatic computation

**Now.** `Model::compute_actuator_params` (`forward/fiber.rs:28`):
- *Phase 1* (`fiber.rs:33-130`) copies the joint/tendon range for **every** actuator, gear-scaled (the "DT-106" deviation, comment `:39-56`), and for an unlimited *fixed* tendon sums the joints' ranges (`:85-110`); an unlimited spatial tendon logs a warning (`:75-84`).
- *Phase 3b* (`fiber.rs:207-221`) calls `muscle::mj_set_length_range` with `LengthRangeOpt::default()`, which has **`uselimit: true`** (`types/model.rs:1128-1143`; its doc says "identical defaults", `:1103` — false, MuJoCo's is 0). That function (`forward/muscle.rs:33-109`): mode filter counts `HillMuscle` as muscle (`:36-42`), runs with gravity, contacts, springs, dampers and actuation **on** (comment `:116-118` "MuJoCo does NOT disable gravity or contacts" — false, `user_model.cc:2411-2412`), loops a fixed `ceil(inttotal/timestep)` steps measuring `time_before >= inttotal−interval` (`:147-231`), and on any failure leaves (0,0) silently (comment `:101-107` "MuJoCo silently leaves the range at (0, 0)" — false, `user_model.cc:2447-2449` throws).

Consequences measured on the corpus (`out/orig_corpus.jsonl`): a motor on a limited joint gets the joint range (MuJoCo (0,0); 21 actuators); a muscle on a limited joint gets the exact joint range (MuJoCo simulates and lands past the soft limit: ±0.0180232 on ±1°, ours ±0.0174533; all 13 corpus joint muscles); a muscle on a spatial tendon gets (0,0) (`ext_arm26`: MuJoCo (0.1024, 0.3404)…, ours (0,0) for all 6; 100-step qpos differs by 0.58).

**Target.** §1 exactly. MuJoCo 3.5.0 `user_model.cc:2408-2507`, `engine_setconst.c:1107-1272`.

**Change (prototyped, `core_lr/`, `mjcf_lr/`).**
```rust
// sim-core types/model.rs
#[derive(Debug, Clone, PartialEq)]                     // + PartialEq (MjcfCompiler derives it)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))] // MjcfModel is serde
pub struct LengthRangeOpt { /* fields unchanged */ }   // Default: uselimit = false (engine_init.c:234-246)

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LengthRangeMode { None, #[default] Muscle, MuscleUser, All }   // unchanged otherwise

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum LengthRangeError {                             // replaces the two-variant enum (no external user, `git grep`)
    #[error("Unstable lengthrange simulation in actuator {actuator}")]
    Unstable { actuator: usize },
    #[error("Invalid lengthrange ({}, {}) in actuator {actuator}", range.0, range.1)]
    InvalidRange { actuator: usize, range: (f64, f64) },
    #[error("Lengthrange computation did not converge in actuator {actuator}: eval ({}, {}) range ({}, {})", eval.0, eval.1, range.0, range.1)]
    NotConverged { actuator: usize, side: usize, eval: (f64, f64), range: (f64, f64) },
}

impl Model {
    /// MuJoCo's compiler step `mjCModel::LengthRange`. Not part of
    /// `compute_actuator_params` / `recompute_derived` (engine_setconst.c:1088).
    pub fn set_length_range(&mut self, opt: &LengthRangeOpt) -> Result<(), LengthRangeError>;
}
```
- Re-export `LengthRangeOpt`, `LengthRangeMode`, `LengthRangeError` at the crate root (today only `sim_core::types::…`).
- `compute_actuator_params`: delete Phase 1 and Phase 3b; its doc drops "lengthrange" (it keeps acc0, dampratio, F0).
- `forward/muscle.rs`: `mj_set_length_range` (crate-private, `Result`) on one clone with `disableflags = LR_DISABLEFLAGS` (replacing), `enableflags &= !ENABLE_SLEEP`, `timestep = opt.timestep` if > 0; per actuator the §1 steps in order; mode filter is MuJoCo's literal one (`GainType::Muscle || BiasType::Muscle`; Q-LR3); `uselimit` copies raw (Q-LR2); `while data.time < opt.inttotal`, measure `data.time > inttotal − interval` after the step, `lmin/lmax` start at 0 with `updated`. `eval_act` keeps `build_actuator_moment` (`fiber.rs:250`) for the moment (measured equal to MuJoCo's on joint, jointinparent, fixed/spatial tendon, site±refsite, slider-crank, body). It should call the crate-private stages of `step1`/`step2` rather than `Data::step2`, which logs a warning on every RK4 step (`forward/mod.rs:181-185`) — 2,000 per muscle per load otherwise.
- Reset handling: Q-LR4.

sim-mjcf: `model_from_mjcf` calls `model.set_length_range(&mjcf.compiler.lengthrange)` right after `builder.build()` (`builder/mod.rs:335`) — after trees and sleep policy exist, as MuJoCo runs it after `mj_setConst` — and maps the error (Q-LR5).

**Measured agreement of the prototype** — every number from `out/*`:

| case (`cases*/`) | MuJoCo 3.5.0 | main | port − MuJoCo |
|---|---|---|---|
| muscle, hinge ±1° | (−0.018023230873209384, +same) | (−0.0174533, +0.0174533) | 1.0e-13 |
| muscle, hinge −30…60° (also with gravity + floor; with stiffness/damping/frictionloss/armature; `constraint`/`limit` disabled by the user; RK4, implicit, implicitfast; `sleep` enabled; model timestep 0.0005; `actearly`) | (−0.524168713951565, 1.0477674895498639) | (−0.523599, 1.047198) | ≤ 9.1e-13 |
| muscle, gear 2 / gear −1.5 / range 10…80° (qpos0 outside) / joint `ref` | simulated, gear-scaled by the dynamics | gear·range copy | ≤ 2.1e-13 |
| muscle, slide −0.1…0.25 | (−0.1005699383532661, 0.25056993835326613) | (−0.1, 0.25) | 1.0e-13 |
| muscle, fixed tendon limited / unlimited over limited joints | (−0.3004055017804072, 0.5002993909329414) / (−0.458367, 0.720166) | tendon range / joint-sum | 2.5e-12 / 8.6e-13 |
| muscle, spatial tendon limited / unlimited / wrapping a cylinder | (0.509531, 0.600032) / (0.509531, 0.633723) / (0.48072531165329563, 0.6216426647338462) | (0.3, 0.6) / (0,0) / (0,0) | 2.9e-13 / **1.9e-8** / 3.5e-13 |
| `<general>` muscle via jointinparent / site+refsite / slider-crank | (−0.524169, 1.04777) / (0.21301918550660875, 0.3039607794499555) / (−0.3467385910330431, 0) | copy / (0,0) / (−0.34641, 0) | ≤ 5.1e-13 |
| two muscles on a 2-link chain | (−0.5239686…, 1.0475311…), (−0.5502649…, +same) | limit copies | **2.7e-10** |
| muscle on a multi-joint body (P-L32 layout) | (−0.524169, 1.04777) | copy | 9.3e-14 |
| `ext_arm26.xml` (MuJoCo's `model/tendon_arm`, 6 muscles on spatial tendons) | 6 ranges | all (0,0) | 7.3e-12; 100-step qpos 5.7e-12 (main 0.58) |
| motor / position on limited hinge; motor on unlimited fixed tendon | (0,0) | ranges | exact |
| body with an unlimited hinge, spatial tendon to a world site (offset rim) | (0.2881037990682712, 0.7118962009832839) | (0,0) | 1.1e-16; vs. the swept geometric range (0.28810379899582905, 0.711896201004171): 7.2e-11 |
| muscle on unlimited hinge / unlimited slide / unlimited hinge with stiffness, damping or gravity / `mode="all"` motor on unlimited hinge | refused, "did not converge … eval (−181.003, −140.808)" | loads, (0,0) | same refusal, same numbers |
| spatial tendon with coincident sites; `<general>` muscle on a site without refsite; adhesion with `mode="all"`; `inttotal="0"` | refused, "Invalid lengthrange (0, 0)" | loads | same refusal |
| hinge rim exactly in line with the anchor at qpos0 (`cases4/geom_spin_tendon`) | refused, "Invalid lengthrange (0.3, 0.3)" | loads, (0,0) | same refusal (§3.3) |

The two largest residuals (1.9e-8, 2.7e-10) also show in the plain 100-step trajectory of the same models (4.3e-9, 2.0e-11, main and port alike), i.e. they come with the dynamics, not the lengthrange loop; what differs in the dynamics has not been isolated.

### LR-3 — the 4 docs MuJoCo refuses ("did not converge")

**The docs.** `actuator_phase5.rs:194` (`161ce23cab5d0727`), `parser/tests.rs:1454` (`4cc24b1b4255eca2`), `phase7_spec_a.rs:271` (`a9eed09f93777e21`), `parser/tests.rs:1492` (`d7687002cbdc2625`). All four: a `<muscle>` on a hinge with no range. MuJoCo 3.5.0, all four: `Lengthrange computation did not converge in actuator 0: eval (-181.003, -140.808) range (-181.003, 181.003)`.

**Why the iteration does not converge (measured).** The applied force is normalised so that `|qacc| = accel` exactly, whatever the inertia (§1 step 5) — which is why four different models print identical numbers. On an unlimited 1-dof joint the iteration is `v ← v·e^{−h/τ} + a·h`, `q ← q + h·v` with no constraint; its velocity tends to `a·h/(1−e^{−h/τ}) = 20.1002` rad/s, so over the last `interval` = 2 s the length still moves 40.2 while `tolrange·dif = 0.05 × 362.0 = 18.1`. A closed form of exactly that loop (`scripts/closed_form.py`, output `out/closed_form.txt`) gives side 0 `eval (−181.0027413, −140.8082092)`, `range (−181.0027413, +181.0027413)` — MuJoCo's printed digits, and the port's `(−181.0027413207507, −140.80820919359223)`.

**What the "right" lengthrange would be.** The transmission length of a muscle on an unlimited hinge or slide is `gear·q` with `q` unbounded: there is no finite feasible range to be right about. The iteration's output when it *does* converge is set by the options, not the model: with `<lengthrange inttotal="T"/>`, MuJoCo and the port both converge for T ≥ 22 and return ±422.003 (22), ±582.804 (30), ±783.806 (40), ±1186.01 (60) — linear in T, the closed form reproduces each; T = 20 still fails (`cases3/`, `out/*_cases3.jsonl`). MuJoCo documents the refusal as intended for exactly this configuration (`doc/modeling.rst:774-776`).

**Is there a test that shows our value is right?** No: no value is right, so no test can show one. **Under Jon's condition these docs are not loadable and come back to him**, with this evidence: MuJoCo's refusal here is its documented behaviour on an unbounded input, not a defect on a valid one. *Recommendation:* parity — refuse with MuJoCo's message (the port does). If Jon wants them loaded anyway, the only candidates are (a) (0,0) as on main, which makes the muscle's normalised length `prm[0] + len/1e-10` (`actuation.rs:587-588`) — force garbage as soon as the joint moves (main's 100-step `afrc` on the coincident-site case: −2.5e11) — or (b) an arbitrary finite value; neither can be tested right.

**What the 4 docs are for, and how they flip.** Only 2 are loaded: `actuator_phase5.rs:193` (asserts the (0,0); its doc comment "MuJoCo also fails silently here" is false) and `phase7_spec_a.rs:270` (a defaults test; asserts `gainprm`). The two `parser/tests.rs` ones call `parse_mjcf_str` only and do not flip (and `parser/tests.rs:1551`, `f7d1a8a1467cd89a`, also contains such a muscle: parse-only; MuJoCo refuses it earlier for the adhesion's control range). Running the tests also found **a 5th model outside the static corpus**: the `format!` template `activation_clamping.rs:65-85` (`build_muscle_model`, unlimited hinge) — 3 tests flip.

### LR-3b — found: a geometry-limited case MuJoCo refuses although the range is known

`cases4/geom_spin_tendon.xml`: a spatial tendon from a world site to a rim site on an unlimited hinge, the rim exactly in line with the anchor at qpos0. The swept geometric range is (0.3, 0.7) (2,000,000-point sweep = closed form |d∓r|). MuJoCo: "Invalid lengthrange (0.3, 0.3)" — at qpos0 the moment arm is 0, so the applied force is 0 and nothing moves (§1 step 5). The port does the same. This *is* MuJoCo failing on a valid input whose right value a test can show (the sweep); but our port computes no other value, so Jon's lenient rule has nothing to load. Not in the corpus. *Recommendation:* parity (refuse, listed in the divergences doc as a known MuJoCo limitation we share); a perturbed start or a sweep fallback would be a feature beyond MuJoCo.

### LR-4 — `<compiler><lengthrange>` (today: children skipped, `parser/mod.rs:90-93`; spec A4 §4 lists it as a stated limitation)

**Recommendation: implement it in this PR.** Reasons, each measured: (1) sim-core already has the options struct, so the work is the parser (10 attributes) plus a field `MjcfCompiler.lengthrange: LengthRangeOpt` — prototyped in `mjcf_lr/src/parser/mod.rs` (~60 lines); (2) 17 option cases agree with MuJoCo in status and value (plus the 7 `inttotal` cases of LR-3) (`mode` all/none, `useexisting="false"`, `uselimit` with gear 1 and 2, `accel/timeconst/timestep/inttotal/interval/tolrange/maxforce`, `timestep` 0 and −1, `inttotal` 0, `interval > inttotal`, `mode="all"` on site, slider-crank, adhesion, unlimited); (3) after LR-3 the user's MuJoCo-native ways out of a refusal are `mode="none"` or an explicit value — refusing the element removes one of them. In-tree use: 1 (`parser/tests.rs:553`, an empty `<lengthrange/>`); submodules and MuJoCo's `model/`: 0 (`grep`).
**What would differ if wrong (keep refusing):** `parser/tests.rs:553` flips to a refusal, and a MuJoCo model carrying the element fails to load in ours.
`mode="muscleuser"` cannot be exercised until `gaintype="user"` loads (ours: "unknown gaintype 'user'", A4's area).

### LR-5 — `uselimit` and gear (the DT-106 comment)

MuJoCo copies `jnt_range`/`tendon_range` raw under `uselimit` (`engine_setconst.c:1179-1193`); measured `uselimit="true"`, muscle gear 2 on −30…60°: MuJoCo (−0.5235987755982988, 1.0471975511965976), main (−1.0472, 2.0944). The simulated path is gear-scaled by the dynamics (actuator length = gear·q). See Q-LR2.

---

## 3. Tests to add (each fails on main; every "main" value below was measured with `harness_orig`)

| # | test (crate) | input | asserts | main |
|---|---|---|---|---|
| T1 | `lengthrange_simulated_past_the_soft_limit` (sim-mjcf) | `<muscle>` on hinge `range="-1 1"` | (−0.018023230873209384, +same) ± 1e-9 | ±0.0174533 |
| T2 | `lengthrange_not_computed_for_motors` | `<motor>`, `<position>` on limited hinge | (0, 0) | joint range |
| T3 | `explicit_lengthrange_kept` | muscle `-0.2 0.7`; motor `0.2 0.7`; motor `0.7 0.2`; muscle `0.7 0.2` | kept, kept, kept, simulated (−0.524168713951565, 1.0477674895498639) | joint range ×4 |
| T4 | `muscle_on_unlimited_joint_refused` (rewrites `actuator_phase5.rs:193`) | the T8 doc; slide variant; hinge with `stiffness`, `damping`, gravity | `Err`, Display contains "Lengthrange computation did not converge in actuator 0" | loads, (0,0) |
| T5 | `lengthrange_simulation_disables_gravity_contact_passive` | hinge −30…60° muscle with gravity + floor; with stiffness/damping/frictionloss | both equal the plain case (−0.524168713951565, 1.0477674895498639) ± 1e-9 | joint range (a port that left gravity on would also fail it) |
| T6 | `lengthrange_every_transmission` | fixed tendon limited; spatial wrapping; site+refsite; slider-crank (values §2 LR-2) | ± 1e-9 | ranges / (0,0) |
| T7 | `geometric_lengthrange_matches_sweep` | `cases4/geom_spin_tendon_offset.xml` | (0.5 − √(0.2²+0.07²), 0.5 + √…) ± 1e-9 — the independent oracle that shows the computed value is right | (0, 0) |
| T8 | `compiler_lengthrange_options` (needs LR-4) | `mode="all"` motor; `mode="none"` muscle; `uselimit` gear 2; `inttotal="0.5" interval="0.1"` (−0.5244349583492071, 1.0527977532533574); adhesion `mode="all"` → `Err` "Invalid lengthrange (0, 0) in actuator 0" | values/refusals as MuJoCo | element ignored |
| T9 | `unlimited_lengthrange_is_set_by_the_options` | unlimited hinge muscle, `inttotal` 22 / 30 | ±422.003… / ±582.804… (closed form, 1e-9 rel.) and 20 → `Err` | (0,0) |
| T10 | `hill_and_millard_not_simulated` | `dyntype="hillmuscle"` on limited and on unlimited hinge | loads, (0,0) both (Q-LR3) | (−0.5236, 1.0472) on the limited one |
| T11 | `length_range_opt_default_is_mujocos` (sim-core) | `LengthRangeOpt::default()` | `uselimit == false`, the other 9 as `engine_init.c:234-246` | `uselimit == true` |
| T12 | `set_length_range_errors` (sim-core) | API: body transmission + `mode: All`; unlimited hinge; model and option timestep 0 | `InvalidRange{range:(0,0)}`; `NotConverged{side:0, eval:(−181.0027413207507, −140.80820919359223), range:(−181.0027413207507, 181.0027413207507)}`; `Unstable` | does not compile (API absent) |

## 4. Tests and examples that flip (measured on the patched copies)

| file:line | kind | why | new expectation |
|---|---|---|---|
| `sim/L0/core/src/forward/fiber.rs:569` `test_muscle_force_at_optimal_length` | CI unit | relied on Phase 1's copy; force becomes −5.8e11 | call `set_length_range` (or set the range) first |
| `fiber.rs:1815` `test_hill_lengthrange_mode_filter` | CI unit | pins Hill inside the mode filter | assert Hill stays (0,0) (Q-LR3) |
| `fiber.rs:1421` `test_lengthrange_simulation_motor_mode_all` | CI unit | passes, but ignores the new `Result` (`unused_must_use` → error under `-D warnings`) | `?`/`expect` |
| `sim/L0/mjcf/src/builder/actuator.rs:967` `test_general_muscle_equivalence` | CI unit | helper `general_actuator_model` (`:842-858`) has an unlimited joint | give the helper's joint a range, or both muscles a `lengthrange` |
| `sim/L0/tests/integration/activation_clamping.rs:213, 229, 255` (template `:65-85`) | CI | muscle on unlimited hinge → refused | add `range` to the template's joint |
| `actuator_phase5.rs:193` `test_lengthrange_muscle_unlimited_silently_fails` | CI | refused | becomes T4 |
| `actuator_phase5.rs:125` `test_lengthrange_from_joint_limits` (assert `:148`) | CI | gear·range copy → simulated | (−2.0955349790997277, +same) |
| `actuator_phase5.rs:225` `test_lengthrange_muscle_limited_from_limits` (assert `:246`) | CI | same | (−3.1427325302963256, +same) |
| `phase7_spec_a.rs:270` `t9_muscle_range_defaults_cascade` | CI | refused | add a joint `range` (the test is about `gainprm`) |
| `examples/fundamentals/sim-cpu/muscles/stress-test` | validator | **does not flip**: 51/51 PASS; printed D1 force −227.4953 → −227.5972, D2 −0.178844 → −0.177830 | — |
| `parser/tests.rs:553` | CI unit | passes; with LR-4 it should also assert the parsed options | — |
| muscles `activation`, `cocontraction`, `force-length`, `forearm-flexion` | Bevy, never run | load; lengthrange ±1.57 → ±1.5705699383532663 | — |

**Expected-change list** (corpus, `out/expected_change.txt`; buckets as A6 §4):
- **ok → err (4 + 1):** the four LR-3 docs; `f7d1a8a1467cd89a` (`parser/tests.rs:1551`, parse-only; MuJoCo refuses it for its adhesion).
- **ok → ok, trajectory changed (9, all MuJoCo-`<muscle>` docs; each now equals MuJoCo's 100-step state to ≤ 5.1e-14):** `muscles/activation:41`, `muscles/stress-test:677` (its Hill actuator's state is bit-identical; the change is its `<muscle>`), `muscles/force-length:44`, `muscles/forearm-flexion:34`, `actuator_phase5.rs:226`, `muscles/stress-test:498`, `actuator_phase5.rs:127`, `muscles/stress-test:978`, `muscles/cocontraction:38`.
- **ok → ok, `actuator_lengthrange` changed, trajectory bit-identical (15):** 11 motor/position/cylinder docs (`sim/L1/bevy/examples/2dof_arm.rs:45`, `mjcf_cartpole.rs:44`, `joint-limits/stress-test:71,149`, `joint-limits/slide-limits:41`, `actuators/cylinder:46`, `sleeping.rs:1901`, `runtime_flags.rs:27`, `mjb.rs:387/533`, two `sim-ml` spec docs) and the 4 Hill docs (`muscles/stress-test:592,621,649,907`; → (0,0)).

**Hill / Millard.** Their force never reads `actuator_lengthrange` (`actuation.rs:595-630`, `:658-681`; `hybrid.rs` reads it only under `GainType::Muscle`), and measured: the 4 Hill-only corpus docs' trajectories are bit-identical, and in the mixed doc the Hill actuator's joint state is bit-identical. Millard (cf-osim, cf-mjcf-emit) was never in the filter; its suites were not run.

## 5. Downstream

- `design/cf-design/src/mechanism/model_builder.rs:940` calls `compute_actuator_params`, whose LR side effect goes away; its `ActuatorKind::Muscle` (`:897-907`) builds `GainType::Muscle` with `gainprm = [1, 0, 0, …]` (range (1, 0), which MuJoCo itself refuses: "range[0]<range[1] required in muscle", measured on `<general>` muscles). No cf-design test builds a muscle into a Model (`git grep ActuatorKind::Muscle`). Q-LR6.
- sim-mjcf `MjcfCompiler` gains a field (struct is never built by literal outside sim-mjcf, `git grep "MjcfCompiler {"`); `.mjb` (serde) needs the serde derives above.
- No other crate reads `actuator_lengthrange` or calls `mj_set_length_range` (`git grep`: sim-core, sim-mjcf, cf-design `:914` push only, tests, one example comment).
- Docs that state the old behaviour: `sim/docs/MUJOCO_REFERENCE.md:265-272` ("computed from tendon limits … gear-scaled"), `:961`; `ARCHITECTURE.md:548`; `MUJOCO_GAP_ANALYSIS.md:537-541`, `:1096` ("lengthrange … defaultable"); `MUJOCO_CONFORMANCE.md:93` (same); `POST_V1_ROADMAP.md:170` (DT-10 `<lengthrange>` child deferred), `:247` (DT-106); `examples/fundamentals/sim-cpu/muscles/README.md:10`, `forearm-flexion/README.md:35` ("computed from joint limits"); the false comments `muscle.rs:101-107,116-118`, `model.rs:1103`, `actuator_phase5.rs:184-189`.

## 6. Found outside this area (hand-offs, measured)

1. **Ball / free joint transmission (core).** Ours computes a length only for 1-dof joints and applies `gear[0]` to the first dof (`actuation.rs:392-404`, `:717-723`); MuJoCo uses the quaternion's axis-angle against the 3-vector gear and a 3-dof moment (ball), 6-dof for free (`engine_core_smooth.c:1311-1360`). Measured: `<muscle>` on a limited ball joint — MuJoCo lengthrange (−0.785675, 0.785675); main (0, 0.785398); the port refuses it ("Invalid lengthrange (0, 0)"), because our length is always 0 — **a new ok→err the port creates unless the transmission is fixed** (0 corpus docs; the 2 corpus actuators on free joints use `gear="1 0 0 0 0 0"` and agree to 1e-15). Owner: core; parity or a stated-limitation refusal for non-scalar gear.
2. **Muscle force clamps** use `.max(1e-10)` (`actuation.rs:587-589`, `:652-653`; `hybrid.rs:155-156`, `:975-976`); MuJoCo uses `mjMINVAL` = 1e-15 (`engine_util_misc.c:670-674`, `:713-716`). Only visible with a near-zero range: `mode="none"` muscle, 100-step qpos differs from MuJoCo by 1.7e4.
3. **`BiasType::Muscle` reads `gainprm`** (`actuation.rs:650`, comment "muscle uses gainprm for both"); MuJoCo reads `biasprm` (`engine_forward.c:459-475`). Equal for `<muscle>`; a `<general biastype="muscle">` with its own `biasprm` differs (measured 100-step qpos 2.7e-4, main and port alike).
4. **Muscle F0** is resolved at compile only when `dyntype` is a muscle (`fiber.rs:223-239`, Phase 4); MuJoCo computes `scale/acc0` on every call whenever `force < 0`, for any `gaintype/biastype="muscle"` (`engine_util_misc.c:665-667`, `:708-710`). Measured: `<general gaintype="muscle">` without `dyntype` → 100-step qpos differs by 1.0.
5. **3-value `solimplimit` is dropped** (`attr_fixed::<5>`, `parser/attrs.rs:148-162`) — already A4 §2.3. It is the only reason `cases/muscle_soft_limit` differs (6.4e-3); with 5 values the port agrees to 8.5e-13. Same equilibrium probe without any actuator: ours with `solimplimit="0.8 0.9 0.01"` equals the default bit-for-bit.
6. `muscle_qpos0_ref` (joint `ref="20"`): lengthrange agrees, 100-step qpos differs 2.9e-2 on main and port alike — not isolated.
7. MuJoCo refuses `<muscle site=…>` and `<general>` muscles with default `gainprm` ("range[0]<range[1] required in muscle") — A5's muscle checks.

## 7. Open questions (not settled by the fixed decisions)

- **Q-LR1 (Jon) — the 4 non-converging docs.** Jon's lenient rule assumed MuJoCo fails through a defect; measured, it refuses an unbounded input by design and no right value exists (§LR-3). *Recommend:* parity — refuse; rewrite the 2 build-time tests + the template (§4). If wrong (load them): they need an arbitrary value that no test can show right, and the muscle's force is undefined.
- **Q-LR2 — `uselimit` raw vs gear-scaled (retire DT-106).** *Recommend* MuJoCo's raw copy: it is what the documented attribute does ("these limits will be copied", `XMLreference.rst:912-916`), and it is reachable only by setting `uselimit="true"` (0 uses in tree, submodules, MuJoCo's models). If wrong: such a model's muscle force differs from MuJoCo's (measured 100-step qpos 2.9e-2 at gear 2).
- **Q-LR3 — HillMuscle in the mode filter.** *Recommend* MuJoCo's literal filter (gain/bias type `Muscle` only): no Hill/Millard force reads the range, and with failures now fatal, keeping Hill in would refuse a Hill actuator on an unlimited joint (it loads today and in the port). If wrong: Hill actuators' `actuator_lengthrange` field is (0,0) instead of a simulated range — nothing reads it.
- **Q-LR4 — a reset during the run.** MuJoCo restarts the side from qpos0 and keeps going (§1); its "Unstable" test cannot fire for h > 0. *Recommend* stricter: any reset during the run → `LengthRangeError::Unstable` (MuJoCo's own message for this case), listed as a deviation — MuJoCo then measures a different trajectory than asked, and a model that resets every few steps never reaches `inttotal` (not measured either way: no input I tried resets). If wrong: a model whose LR run resets once loads in MuJoCo and is refused by ours.
- **Q-LR5 — error shape in `MjcfError`.** *Recommend* `LengthRange { actuator: String, #[source] source: LengthRangeError, at: Location }` with Display = MuJoCo's text (A3 keeps MuJoCo's messages); the alternative is `InvalidModel { reason }`. If wrong: callers parse text to find which actuator failed.
- **Q-LR6 — cf-design.** *Recommend* cf-design calls `set_length_range` after `compute_actuator_params` (its MJCF export of the same mechanism goes through the MJCF path, which will); its muscle parameters are invalid by MuJoCo's checks regardless (§5) — a cf-design item. If wrong: cf-design muscles keep (0,0), as today for spatial-tendon muscles.

## 8. Commit list (this area)

| # | commit | contents | flips | depends |
|---|---|---|---|---|
| LR-a | `fix(sim-mjcf): actuator lengthrange computed and kept as MuJoCo 3.5.0` (body: sim-core changes, breaking `LengthRangeError`, `compute_actuator_params` no longer sets lengthrange) | §LR-1 + §LR-2 in both crates — they cannot be split green: removing Phase 1/3b without the builder's call leaves every MJCF muscle at (0,0) | §4 table (9 tests) + T1–T7, T9–T12 | K6 (recompute_derived lists `compute_actuator_params`; its doc must say lengthrange is not re-derived, `engine_setconst.c:1088`), M1 (error variant), K1 (pre-step check now runs inside the loop: the clone must pass it) |
| LR-b | `feat(sim-mjcf): <compiler><lengthrange>` | §LR-4; delete the default-class `lengthrange` plumbing | `parser/tests.rs:553` (extend) + T8 | LR-a, M15 (schema lists `compiler/lengthrange` with its 10 attributes, `?`; refuses `lengthrange` on `<default>` actuators), M10 (repeated `<compiler>` merges these options too), M6/M7 (keyword exactness, non-finite refusal for the 7 numbers) |
| — | docs into M26 | §5 docs list; divergences table rows: Q-LR4 (if stricter), LR-3b (known MuJoCo limitation, shared) | — | — |

P-L32/K13: the multi-joint-body muscle case agrees at 9.3e-14 with HEAD's dynamics (not re-run with K13's fix); since K13 lands in the core series, LR-a's expected-change list must be re-taken against the core-series head, as RIGID_SPEC §7 requires for every MJCF commit. M25 (delay) does not interact by reading (actuation is disabled during the run, `user_model.cc:2411-2412`); not measured.

## 9. What I could not see

The integration suite outside the 5 muscle files, cf-design / cf-osim / cf-mjcf-emit / sim-bevy suites, grade, clippy at `-D warnings`, the runtime-generated MJCF beyond what the run tests build, `.mjb` round-trip of the new `MjcfCompiler` field, the threaded-vs-serial equality on a model large enough for MuJoCo to thread unevenly (arm26's 6 muscles agree at 7.3e-12), and what in the dynamics produces the 1.9e-8 / 2.7e-10 residuals. The prototype is a scratch patch, not the commit: it maps errors through `ModelConversionError`, still calls `Data::step2` (RK4 warning), and has no reset guard.
