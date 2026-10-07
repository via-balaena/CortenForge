> Research for *A Double Dose of Detail*, written during planning by a read-only researcher at `3520544e`. Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid spec — sim-mjcf validation rules (compiler-side checks) at MuJoCo 3.5.0

Area owner: researcher "mjcf_validation". Rows: mjcf-H1, mjcf-S9, mjcf-H3, mjcf-H4, ledger-L1, mjcf-S6, mjcf-S7, mjcf-S12, mjcf-S13, mjcf-D1, mjcf-D2, the timestep load rule (ledger-L24's sim-mjcf half). Repo read at `main` 3520544e (clean before and after: `git status --short` printed 0 lines both times). Scratch evidence: `SCRATCH/rigid_spec/mjcf_validation/` (`out/all_cases.tsv` = every case with its MuJoCo 3.5.0 and our result).

## 0. What I checked, how, and what the method cannot see

- **Oracle.** MuJoCo 3.5.0 source (`SCRATCH/mj350src/mujoco`, tag 3.5.0, 881544c, confirmed with `git log -1`) and `mujoco==3.5.0` (`from_xml_string`, one subprocess per case, `scripts/run_cases.py`). Every citation below is a 3.5.0 `file:line` under `src/`.
- **Cases.** 276 MJCF cases: the cold reviewer's 118 (`rigid_review_mjcf/out/case_*.json`), 135 new (`scripts/gen_cases.py` → `cases_new/`), 23 extra (`cases_x/`). All 276 were run through BOTH `mujoco==3.4.0` and `mujoco==3.5.0`: **0 differ** (118/118 and 158/158 identical ok/err, message and probe value).
- **Ours on main.** A scratch probe (`probe/`, path deps on sim-mjcf/sim-core/sim-urdf, own `target/`) loads each case in its own process under `timeout 20`, catches panics, and optionally steps 3 times. Excluded from our loader per the brief: NaN/±inf geom mass/density/fullinertia, the classless nested default, the deep-nesting cases.
- **Corpus.** The 1,584 static in-tree docs: our result `rigid_plan_tests/emb_1.jsonl`, MuJoCo 3.5.0 `rigid_oracle_350/corpus350.jsonl`, doc→`file:line` via `corpus/manifest.jsonl` (`scripts/flips.py`). MuJoCo reports only its FIRST error, so a doc refused first for a schema reason hides my rules. To look past that I stripped the offending attribute/element/keyword and re-ran MuJoCo (`scripts/sanitize_rerun.py`, plus a pass converting our frame-sensor `site=`/`body=` to `objtype`/`objname`): 93 of the 264 ours-ok/MuJoCo-err docs then LOAD and no new my-rule refusal appeared; **~130 docs stay opaque** (66 old-format flex → "required attribute missing: 'body'", 36 stuck on keywords/too-much-data, 16 composite curve, others). For those my flip list is blind; the branch A/B in the verification plan is what sees them.
- **Self-check of S7 on our own numbers.** The S7 mass rules will run on OUR computed body arrays, not MuJoCo's. Probe mode `s7scan` applied MuJoCo's moving-body and triangle rules to our final `Model` for all 1,478 docs we load: non-flex hits = **exactly** MuJoCo's 10 moving-body refusals and 4 triangle refusals (same docs; triangle margins −1.0e-2…−1.9e-2, nothing at rounding level). 73 extra hits were all flex docs (our flex vertex bodies are built after the mass pipeline with zero inertia) ⇒ the check must run before `process_flex_bodies`.
- **URDF.** The 43 in-tree URDF string literals (`git grep -l '<robot'`) converted by `sim_urdf::urdf_to_mjcf` and fed to MuJoCo 3.5.0 (`urdf/`).
- **Cannot see:** the 157 `format!` templates and runtime MJCF other than URDF (I read the few the design names); the 253 submodule XML files; MuJoCo's second-and-later errors in the ~130 opaque docs; timing/CI cost. Nothing here was run in the repo (`cargo test` not run anywhere).

## 1. The reviewer's four 3.4.0 DIFFERS, re-checked at 3.5.0 — all four hold

| rule | 3.5.0 measured | 3.5.0 source |
|---|---|---|
| NaN | `size="nan"` → "nan size in geom". NaN = "not set" for geom mass (loads with density mass 4.18879), inertial pos (→ body frame, ipos [0,0,0]), fullinertia (→ inertia 0 → moving-body error), fromto (capsule → "size 1 must be positive"). NaN body pos, joint damping, joint axis, ctrlrange, range, quat, timestep, friction LOAD and keep NaN (the source emits "XML contains a 'NaN'" at `xml_util.cc:631,709`; my 3.5.0 runs did not capture warnings). | `user_objects.cc:3771-3775`; `mjuu_defined` uses at `2405-2419`, `2437-2445`, `3673`, `3778-3785`; warning `xml/xml_util.cc:631,709` |
| inf | `mass="inf"` and `density="inf"` LOAD (body_mass inf); `size="1e120"` LOADS with body_mass inf (overflow); `mass="-inf"` → "mass, inertia or density are negative in geom". `1e400` → "number is too large". | `user_objects.cc:3805-3807`; `xml_util.cc:711-712` |
| ball range | only `range[0] != 0` is checked when limited: `0 0`, `0 -10`, `limited` with no range (→ `0 0`) LOAD limited; `45 0` errors; auto `45 0` and auto `0 -10` load UNLIMITED. | `user_objects.cc:2908-2914` |
| unknown attr under `<frame>` | `<frame bogus>`, a geom in a frame, a body in a frame and a geom in that body with `bogus=` all LOAD; an unknown ELEMENT in a frame errors ("unrecognized model element"). Cause: the schema treats `frame` as `body` (`NameMatch`) but recurses only into children named `body`. | `xml/xml_util.cc:474-485`, `516-519`; `xml/xml_native_reader.cc:3818` |

(Decision 19 already settles refusing unknown attributes under frames; that is the parser area's mechanism.)

## 2. What resolved information each rule needs (for the researcher placing validation)

| rule | needs | earliest correct position (ours) | MuJoCo position |
|---|---|---|---|
| non-finite number (decision 3) | the attribute text only | the one float-token parser, at parse (covers `<default>` values, which go through the same helpers, `parser/defaults.rs:129` etc.) | XML read (warns only) |
| `<inertial>` requires `pos` + `mass`; `fullinertia` + orientation | attribute presence | `parse_inertial_attrs` (`parser/body.rs:455`) | XML read, `xml_native_reader.cc:3530-3537` |
| duplicate names (S12) | names after frame/composite expansion; BEFORE `discardvisual`/`fusestatic` remove elements | after `expand_composites`, before `apply_discardvisual` (`builder/mod.rs:246-262`) | at name set + `ProcessLists` (`user_api.cc:1743`, `user_model.cc:4405-4475`) |
| joint axis too small | joint type + axis after `<default>` | `process_joint` after `apply_to_joint` (`builder/body.rs:234-236`) | `user_objects.cc:2956-2960` |
| `*range`/`*limited` helper, damper/adhesion, actlimited/dyntype, muscle prm | element after `<default>`, `compiler.autolimits`, `compiler.angle`, joint type | `process_joint`, `process_tendons`, `process_actuator` | `user_objects.cc:2895-2943`, `6412-6436`, `6869-6944`; `user_api.cc:972-1064` |
| geom condim, checksize, negative geom mass/density, plane-in-moving-body | geom after `<default>` AND after `fromto`/mesh fit; body's weld status (any joint on body or ancestors) | `process_geom` / `resolve_body_inertia` (needs `mjcf-S3` default size first) | `user_objects.cc:3650-3807` |
| pair condim | pair after `<default><pair>` | `process_contact` | `user_objects.cc:5666-5669` |
| inertial combos, fullinertia PD, triangle, negative, non-finite derived mass | body's resolved geoms/inertial; compiler `balanceinertia`/`boundmass`/`boundinertia` | per body inside the mass pipeline (`apply_mass_pipeline`, `builder/mass.rs:29`) | `user_objects.cc:2404-2470` |
| moving-body mass ≥ mjMINVAL (static-child exception) | whole tree after bound/balance, BEFORE `settotalmass`, EXCLUDING flex vertex bodies | between `apply_mass_pipeline`'s balance step and its settotalmass step; before `process_flex_bodies` (`builder/mod.rs:322-325`) | `user_model.cc:4993-4998`, `5310-5330`; settotalmass later at `5041-5042` |
| joint layout (H3) | joint types per body after defaults, frames, composites, fusestatic; parent after fusestatic | over builder arrays before `builder.build()` (`builder/mod.rs:335`) — `build()` reaches `make_data` via `compute_actuator_params` (`build.rs:39`, `fiber.rs:134`) | body compile `user_objects.cc:2519-2534`; `CopyTree` `user_model.cc:2715-2727` |
| hfield size > 0 | hfield asset | `process_hfield` (`builder/mesh.rs:89`) | `user_objects.cc:4471-4474` |
| vertex-only mesh hull | mesh asset | `convert_mjcf_mesh` (`builder/mesh.rs:345`) | `user_mesh.cc:1567-1574`, `1806-1808`, `2040` |
| timestep | option | `validate_option` | none (MuJoCo loads 0, −0.001, NaN, 2) |

`validate()` today runs before frames/composites (`builder/mod.rs:234`), so every per-element rule above that lives in `validate()` is blind inside frames (mjcf-S13, §3.7).

**Can today's `validate()` position (before defaults, frames, composites, fusestatic) see the value?** Yes only for: non-finite numbers, `<inertial>` attribute presence/combinations, hfield size, timestep. Every other rule above reads something a `<default>` class can supply (joint type, axis, range/limited, ctrlrange, condim, geom size, mass/density) or something computed later (fromto/mesh size, body inertia, weld status, parents after fusestatic), so it cannot run there. Each rule's minimal must-flip doc is the case named in its "Tests to add" (XML in `mjcf_validation/cases_new/`, `cases_x/`, `rigid_review_mjcf/out/`; results in `out/all_cases.tsv`).

## 3. Items

### 3.1 mjcf-H1 + mjcf-S9 (non-finite numbers; decision 3: refuse every NaN and ±inf) and the load hang

**Now.**
- Floats are parsed by `parse_float_attr` (`parser/attrs.rs:42-44`, `.parse().ok()`), `parse_float_array` (`attrs.rs:96-103`), and ~30 direct `.parse()` sites (`parser/deformable.rs:122-518`, `parser/asset.rs:150,159`, `parser/body.rs:231`, `parser/equality.rs:218`). Per the `f64::from_str` grammar `nan`, `inf`, `infinity` parse (case-insensitive); measured: `NaN`, `nan`, `inf` parse and `1e400` becomes `inf`. No helper checks finiteness.
- Partial finiteness checks exist: `validate_option` (`validation.rs:43-139`: timestep, tolerance, impratio, gravity, wind, density, viscosity…), inertial mass/diaginertia (`validation.rs:222-243`), geom size/fromto (`validation.rs:380-403`), fixed-tendon coef (`validation.rs:476`), keyframes (`builder/mod.rs:144-185`, pinned by `sim/L0/tests/integration/keyframes.rs:805` ac22 and `:836` ac23), springlength (`parser/tendon.rs:158-185`, pinned by `tendon_springlength.rs:222-262`).
- Measured on main: `pos="nan 0 0"` loads NaN; `pos="inf 0 0"` loads; `axis="nan 0 0"` loads as Z (`safe_normalize_axis`, `attrs.rs:12-15`: `NaN > 1e-10` is false); `damping="nan"` loads and qpos is NaN after 3 steps; `ctrlrange="nan 1"` limited loads and the first step panics in `f64::clamp`; `range="nan 1"` loads; `quat="nan 0 0 0"` loads; `friction="nan …"` loads; `<default><geom size="nan"/>` loads (S3: default size is not read at all).
- **The hang.** `extract_inertial_properties` (`builder/mass.rs:89-135`) and `compute_inertia_from_geoms` (`mass.rs:145-249`) call `symmetric_eigen` then `Rotation3::from_matrix(&rot)` (`mass.rs:118`, `:246`). Measured (probe `eigen-*`): `symmetric_eigen` RETURNED on all three non-finite matrices tried (NaN diagonal, inf diagonal, NaN off-diagonal); `from_matrix` on the NaN-off-diagonal eigenvectors did not return within 5 s (`timeout` rc 124). nalgebra 0.34.2 `from_matrix` = `from_matrix_eps(m, eps, 0, identity)` and `0` means `usize::MAX` iterations (`rotation_specialization.rs:736-741`). Finite input reaches the same path through overflow: MuJoCo 3.5.0 loads `<geom size="1e120"/>` on a free body with body_mass = inf (measured), so our volume (`builder/geom.rs:371-405`) overflows to inf, the tensor goes non-finite, and the eigen path follows. Ours on `size="1e120"`: not run (same mechanism as the forbidden inputs).

**Target.**
1. *Parse:* every float token in MJCF goes through one function that refuses NaN, ±inf and overflow-to-inf. Deviation: stricter — MuJoCo warns and keeps NaN, loads inf (§1). This also replaces MuJoCo's "number is too large" (`xml_util.cc:711-712`) for `1e400`, which we refuse with the non-finite error (both refuse).
2. *Derived values:* the builder refuses a non-finite body mass, inertia tensor, eigenvalue or eigenvector (deviation: stricter — MuJoCo silently produces inf, measured `size="1e120"`). This guard is needed regardless of (1): overflow from finite input, caller-built `MjcfModel`, and `.mjb` input (§3.8) all bypass the parser.
3. *No unbounded loop:* `Rotation3::from_matrix_unchecked` after the determinant flip.

**Change.**
- In the parser area's converted helpers (`parser/attrs.rs`): one token parser, used by every float helper and by the deformable/asset/body/equality direct sites:
  ```rust
  /// Parse one MJCF float token. Refuses NaN, ±inf and values that overflow to ±inf.
  pub(super) fn parse_f64(token: &str, element: &str, attribute: &str) -> Result<f64>;
  ```
  The error must name element and attribute, and its `Display` must contain "finite" (keeps `keyframes.rs` ac22/ac23 green; they assert `contains("finite") || contains("NaN"/"time")`).
- `builder/mass.rs`:
  ```rust
  /// Principal inertia and axes of a symmetric tensor. Refuses a non-finite tensor or
  /// a non-finite eigen-decomposition; no iterative rotation fit.
  fn principal_axes(tensor: &Matrix3<f64>, body: &str) -> Result<(Vector3<f64>, UnitQuaternion<f64>)>;
  pub(crate) fn extract_inertial_properties(inertial: &MjcfInertial, body: &str) -> Result<(f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>)>;
  pub(crate) fn compute_inertia_from_geoms(…, body: &str) -> Result<(f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>)>;
  ```
  plus `ModelBuilder::resolve_body_inertia` (`builder/body.rs:287`) returns `Result`. These are inside private `mod builder` (`lib.rs:42`), so no public signature changes. `principal_axes`: check `tensor.iter().all(is_finite)`, `symmetric_eigen`, check eigenvalues/vectors finite, det flip, `Rotation3::from_matrix_unchecked(rot)` (precedent: `builder/orientation.rs:111`), and check `mass.is_finite()`.
- **Bits.** Measured (probe `ab-unchecked`, 20,000 random SPD + diagonal matrices): `from_matrix_unchecked` gives a bit-identical quaternion in 5,027/20,000 (the diagonal quarter), others differ by ≤ 1.1e-15. `from_matrix_eps(&rot, f64::EPSILON, 64, identity)` was bit-identical in 20,000/20,000, but keeps nalgebra's inner perturbation `loop {}` (unbounded, `rotation_specialization.rs:770-779`). See open question Q1.

**Tests to add** (all fail on main as stated; measured unless noted):
- `nan_rejected_everywhere`: `pos="nan 0 0"` (body), `axis="nan 0 0"`, `damping="nan"`, `ctrlrange="nan 1"`, `range="-inf inf"`, `quat="nan 0 0 0"`, `friction="nan 0.005 0.0001"`, `<default><geom size="nan"/>`, `size="1e400"` → `Err` naming the attribute. Main: all load except `1e400` (refused today by `check_geom_finite` as "size values must be finite", a different message).
- `hang_is_an_error_typed_model`: an `MjcfModel` built in code with `fullinertia: Some([1.0, 1.0, 1.0, f64::NAN, 0.0, 0.0])` on a free body → `model_from_mjcf` returns `Err`. Main: hangs (from_matrix measured to hang on these eigenvectors; the full loader was not run on it, per the brief). Pre-registration "make it fail once" on main must be run under `timeout`.
- `mass_overflow_is_an_error`: `<geom size="1e120"/>` on a free body → `Err` (finite input). Main: not run (expected hang, unmeasured).
- `from_matrix_unchecked_matches`: for the existing fullinertia tests, iquat within 1e-14 of the old value (pins Q1's bit change as bounded).

**Tests/examples that flip.** Corpus: the only static docs carrying NaN/inf are the three already refused (`keyframes.rs:805`, `:836`, `tendon_springlength.rs:225`) — message changes only; with the message containing "finite" no test flips (CI-run). `validation.rs:870-1071` unit tests build typed NaN models and keep passing (validate's typed checks stay). If Q1 = unchecked: every doc with a non-diagonal body tensor changes `model_fp` by ≤ 1.1e-15 in `body_iquat` (expected-change list; trajectory bits may follow) — not measured on the corpus.

**Downstream.** None compile-facing. Behaviour: any in-tree MJCF with NaN/inf (3 static docs, all tests that already expect `Err`).

### 3.2 mjcf-S9 (joint axis)

**Now.** Parser normalises at parse time with a Z fallback (`parser/body.rs:284-285` → `attrs.rs:12-15`, threshold `norm > 1e-10`); default-class axis is not normalised (`parser/defaults.rs:129`); builder normalises again and warns + substitutes Z (`builder/joint.rs:73-84`). Measured on main: `axis="0 0 0"`, `1e-10`, `9e-8`, `1e-7`, slide `0 0 0`, default-class `0 0 0`, zero axis in a body under a rotated frame → all LOAD (Z).

**Target.** MuJoCo 3.5.0: ball and free axes are forced to (0,0,1) (`user_objects.cc:2945-2950`); otherwise the axis is rotated by the frame and `mjuu_normvec` returns 0 when the SQUARED norm < `mjEPS` = 1e-14 (`user_util.cc:149-159`, `user_util.h:29`), then "axis too small in joint" (`user_objects.cc:2956-2960`). Measured at 3.5.0: `0 0 1e-7` (squared 9.999999999999999e-15) and `0 0 9e-8` refused; ball/free `axis="0 0 0"` load with (0,0,1). NaN axis: MuJoCo loads NaN (the comparison is false); ours refuses at parse (§3.1).

**Change.** `parser/body.rs:285` stores the raw vector (drop `safe_normalize_axis`, delete it from `attrs.rs`). In `process_joint` (`builder/joint.rs:73-84`): for Ball/Free push (0,0,1); for Hinge/Slide `if !(axis.norm_squared() >= 1e-14) → Err(axis too small in joint '<name>')` (NaN-robust form), else normalise. Frame rotation does not change the norm, so the check is valid before or after `expand_frames`.

**Tests to add.** The 7 cases above → `Err`; `axis_ball_zero`, `axis_free_zero` → `Ok`, `jnt_axis == (0,0,1)`. Main: all 9 load (measured).

**Flips.** Corpus: 0 (MuJoCo refused no corpus doc with "axis too small"). `git grep` found no test asserting the Z fallback.

### 3.3 mjcf-H3 (joint layout; decision 17: refuse ball-then-slide as a stated limitation)

**Now.** No MJCF check. sim-core `Model::validate_joint_layout` (`types/model_init.rs:458-482`) asserts "free must be the body's only joint" and "ball must be the LAST joint", called by `make_data` (`model_init.rs:494`), which the builder reaches inside `build()` (`build.rs:39` → `fiber.rs:134`). Measured on main: ball→hinge, ball→ball, ball→slide, ball→slide→hinge, ball(from a default class)→hinge, free+hinge, free+slide → PANIC inside `load_model`; seven hinges on one body LOADS (and steps to NaN); free joint under a non-world body (directly, through one or two nested frames) LOADS.

**Target (3.5.0, measured and cited).**
- `more than 6 dofs in body '<name>'` (`user_objects.cc:2519-2526`) — fires first for free + anything.
- `ball followed by rotation in body '<name>'` — a hinge or ball after a ball (`2528-2535`).
- `free joint can only be used on top level` — parent ≠ world AFTER frames are dissolved and fusestatic (`user_model.cc:2725-2727`; fusestatic at `4937-4938` precedes `CopyTree` at `5021`). Measured: a free body inside a world-level `<frame>` (one or two levels) LOADS; a free body inside a frame inside a body is refused; a free body under a static body with `fusestatic="true"` LOADS. (`free joint can only appear by itself`, `2722-2723`, is unreachable from XML: any second joint adds ≥ 1 dof and trips the 6-dof check first.)
- Deviation, stated limitation: ball followed by slide (MuJoCo loads it, measured nv 4 and 5). Refuse.
- hinge→ball and slide→ball: MuJoCo loads; sim-core accepts them (ball last). Keep loading. **But see §4.1:** our dynamics for ANY body with ≥ 2 joints disagree with MuJoCo (hinge→ball qvel off by 1.1e-3 after one 2 ms step; hinge→hinge 2.3e-4; slide→hinge 3.0e-3), while one joint per body agrees to 3e-16. That is not specific to the ball; it is reported to the core area, not handled by a layout refusal.

**Change.**
```rust
impl ModelBuilder {
    /// MuJoCo's per-body joint-layout rules plus our ball-then-slide limitation.
    /// Runs on the builder arrays after defaults, frames, composites and fusestatic,
    /// before `build()` (which reaches sim-core's panicking layout assert via make_data).
    pub(crate) fn check_joint_layout(&self) -> Result<()>;
}
```
Reads `jnt_type`, `body_jnt_adr`, `body_jnt_num`, `body_parent`. Order of checks per body = MuJoCo's: dofs > 6, then ball-then-rotation, then ball-then-slide (ours), then free top-level. Called in `model_from_mjcf` immediately before `builder.build()` (`builder/mod.rs:335`). After it passes, sim-core's two asserts cannot fire for MJCF input (free alone ⇐ 6-dof rule; ball last ⇐ rotation rule ∪ slide limitation).

sim-core's own check: recommend it become fallible — `pub fn check_joint_layout(&self) -> Result<(), JointLayoutError>` in sim-core, `make_data` keeps its documented panic by calling it, and core-L7's `try_make_data` returns the error. That is the core area's API (dependency, §6); MJCF does not depend on it.

**Tests to add.** `Err` for: ball_then_hinge, ball_then_ball, ball_slide_hinge, ball_then_slide (limitation wording), default_type_ball_then_hinge, frame_body_ball_hinge, seven_hinges, free_plus_hinge, free_plus_slide, free_nested, free_in_body_in_frame_in_body, free_nested_frames. `Ok` for: hinge_then_ball, slide_then_ball, three_slide_three_hinge (6 dofs), world_frame_frame_free, free_under_static_fusestatic. Main (measured): of the 12 `Err` cases, 8 PANIC and 4 load; the 5 `Ok` cases load.

**Flips.** Corpus: 0 (no doc trips these; the ones that would have panicked were never committed). Templates not checked.

**Downstream.** None compile-facing.

### 3.4 mjcf-H4 + ledger-L1 + mjcf-S6 (`*range`/`*limited`; decision 18: refuse an auto backwards range)

**Now.**
- Joint: `limited = joint.limited.unwrap_or(autolimits && range.is_some())` (`builder/joint.rs:85-95`); range defaults to (−π, π) and is always degree-converted (`joint.rs:104-111`); free joints keep `limited`.
- Tendon: same pattern (`builder/tendon.rs:27-40`, default range ±`f64::MAX`); `validate_tendon_params` refuses limited without range and `min >= max` (`validation.rs:690-707`).
- Actuator: ctrllimited forced true for damper/adhesion (`builder/actuator.rs:40-53`); `ctrlrange.unwrap_or((-1.0, 1.0))` (`:93-97`); forcerange/actrange auto from `is_some()` (`:98-131`); muscles get `actlimited=true, actrange=(0,1)` unless set (`:161-172`); no check of any range; `lengthrange` parsed (`parser/actuator.rs:210-213`) and dropped (`builder/actuator.rs:277`).
- `actuatorfrcrange`/`actuatorfrclimited` (joint, tendon): never parsed; 0 in-tree uses (`git grep`).
- Measured on main: ctrl/force/act ranges `1 -1` (auto or explicit) LOAD and the first step PANICS in `f64::clamp` (`forward/actuation.rs:510`, `:705`, `:457`; RK4 `integrate/rk4.rs:116,202`); explicit hinge/slide `1 -1`, `0 0`, `limited` with no range, default-class `1 -1` LOAD; auto `1 1` loads limited and locked; ball auto `0 -10` loads limited; ctrllimited with no ctrlrange gets (−1, 1); damper/adhesion with no or negative ctrlrange load; `actlimited` with `dyntype="none"` loads; autolimits=false + range (joint, tendon, ctrlrange, forcerange) load unlimited without error; muscle `lmin`/`range`/`scale`/`tausmooth` violations load.

**Target (3.5.0).**
- `islimited`: explicit true, or auto AND `range[0] < range[1]` (`user_objects.cc:184-189`). Range absent ≡ (0,0); "has a range" ≡ ≠ (0,0) (`2902`, `6413`, `6870-6880`).
- `checklimited`: autolimits=false AND auto AND has a range → "`<entity>` has \`<attr>range\` but not \`<attr>limited\`…" (`171-181`; called `2903`, `6415`, `6871/6875/6879`).
- Joints: free → limited forced false (`2895-2897`, measured: `limited="true" range="0 1"` on a free joint loads unlimited); limited non-ball `range[0] >= range[1]` → "range[0] should be smaller than range[1] in joint" (`2908-2910`); limited ball `range[0] != 0` → "range[0] should be 0 in ball joint" (`2912-2913`); degree conversion only when limited (`2916-2924`).
- Tendons: limited `range[0] >= range[1]` → "invalid limits in tendon" (`6418-6421`).
- Actuators: "invalid force range for actuator" / "invalid control range for actuator" / "invalid actrange for actuator" when limited and `[0] >= [1]` (`6883-6890`); "actrange specified but dyntype is 'none' in actuator" (`6892-6893`).
- Damper/adhesion: ctrllimited forced 1 (`user_api.cc:975`, `1057`); negative ctrlrange → "damper/adhesion control range cannot be negative" (`984`, `1064`), checked on the DEFAULT-RESOLVED actuator (the actuator is created from its class, `xml_native_reader.cc:4023`, attributes read at `:2303`, then the setter at `:2446`/`:2486`); no ctrlrange → (0,0) → "invalid control range" (measured).
- Muscle (gaintype or biastype muscle, incl. `<general gaintype="muscle">`): `range[0]<range[1]`, `lmin<1<lmax`, positive `scale, vmax, fpmax, fvmax` (`user_objects.cc:6914-6944`); `tausmooth < 0` → "muscle tausmooth cannot be negative" (`user_api.cc:1025-1026`).
- Deviation (decision 18), stricter: auto with a range AND NOT `range[0] < range[1]` → refuse (MuJoCo silently leaves it unlimited, measured for joint `1 0`, `1 1`, ball `45 0`, ball `0 -10`, tendon `1 0`, `1 1`, spatial tendon `1 -1`, ctrl/force/act `1 -1`, ctrl `1 1`, default-class ctrlrange `1 -1`). NaN-robust form `!(lo < hi)`.

**Change.**
```rust
/// Which `*range`/`*limited` pair (MuJoCo `checklimited`'s entity + attr prefix).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LimitKind { Joint(MjJointType), Tendon, ActuatorCtrl, ActuatorForce, ActuatorAct }

/// MuJoCo's islimited + checklimited + per-kind range checks, plus decision 18.
/// `limited: None` = auto; `range: None` = absent (MuJoCo's (0, 0)).
/// Returns Some(range) when the limit is active, None when unlimited.
pub(crate) fn resolve_limit(
    kind: LimitKind,
    limited: Option<bool>,
    range: Option<(f64, f64)>,
    autolimits: bool,
) -> std::result::Result<Option<(f64, f64)>, LimitError>;

pub(crate) enum LimitError {
    RangeWithoutLimited,                     // checklimited
    AutoRangeNotIncreasing { lo: f64, hi: f64 }, // decision 18 (deviation)
    NotIncreasing { lo: f64, hi: f64 },      // joint / tendon / actuator
    BallLowerNotZero { lo: f64 },            // ball
}
```
One module (`builder/limits.rs`), called from `process_joint`, `process_tendons`, `process_actuator`. Callers map `LimitError` into the errors area's variant with element kind + name + attribute (Display carries MuJoCo's text). Then: joint range stored radian-converted when limited; damper/adhesion: force `limited=Some(true)` and refuse a negative bound BEFORE `resolve_limit`; `actlimited && dyntype == None` → error; muscle checks after `compute_gain_bias_params` (`builder/actuator.rs:198`); `tausmooth < 0` → error. The ctrl/force arrays keep (−∞, ∞) when unlimited (as today). `validation.rs:690-707` (tendon limited/range) is replaced by the helper (one place). After this, every limited range reaching sim-core from MJCF is finite with `lo < hi` (ball: `lo == 0`), so the clamps at `actuation.rs:457,510,705` and `rk4.rs:116,202` cannot panic for MJCF input.

sim-core for a hand-edited `Model`: recommend the core area add the range invariant (limited ⇒ finite, lo ≤ hi) to the same fallible model check as the joint layout (run by `try_make_data`/the pre-step check), rather than a silent `max/min` clamp, which with lo > hi silently returns hi. Owned by core (§6).

**Tests to add** (main results measured): joint/tendon/actuator `limited=true` + `1 -1` → `Err` (main: load); auto `1 -1` and `1 1` for joint, tendon, spatial tendon, ctrl, force, act → `Err` (main: load; ctrl/force/act panic on step); ball explicit `45 0` → `Err`, ball explicit `0 0`, `0 -10`, `limited` without range → `Ok` limited with MuJoCo's range (main: `45 0` loads; `limited` without range gets ±0.0548 rad, MuJoCo (0,0)); ball auto `45 0` / `0 -10` → `Err` (deviation); autolimits=false + range on joint/tendon/ctrl/force → `Err` (main: load); `autolimits=false` + `range="0 0"` → `Ok` (main: Ok — must stay); free joint `limited=true` → `Ok`, `jnt_limited == false` (main: limited true); ctrllimited without ctrlrange → `Err` (main: (−1,1)); damper/adhesion with none or negative ctrlrange → `Err`; `actlimited` + dyntype none → `Err`; muscle `lmin=1.2`, `range="1 0.5"`, `scale=0`, `tausmooth=-1` → `Err`; default-class `range="1 -1"` with element `limited=true` → `Err`.

**Flips** (CI-run unless noted):
- `sim/L0/tests/integration/ball_joint_limits.rs:485` (`test_ball_limit_range_symmetry`, its `"45 0"` iteration) and `:1213` (`test_ball_limit_reversed_range_parsing`) — explicit limited ball `45 0`.
- `examples/fundamentals/sim-cpu/joint-limits/stress-test/src/main.rs:175` `MODEL_LOCKED` (hinge `limited="true" range="0 0"`) — **validator** (validate-examples).
- `sim/L0/mjcf/src/builder/joint.rs:661-683` (`test_autolimits_false_requires_explicit_limited`).
- `sim/L0/tests/integration/phase7_spec_a.rs:465` (`t14_adhesion_gain_defaults_cascade`) and `:496` (`t15_damper_kv_defaults_cascade`) — no ctrlrange.
- `sim/L0/mjcf/src/builder/compiler.rs:516` (`test_fusestatic_protects_actuator_referenced_body`) — MuJoCo: "invalid control range".
- `sim/L0/mjcf/src/parser/tests.rs:1551` (`test_parse_mixed_actuator_types`) — MuJoCo refuses, but the test only parses (`parse_mjcf_str`), so it does NOT flip unless a rule moves into the parser.
- Corpus static scan for auto ranges with lo ≥ hi (`scripts/scan_auto_ranges.py`; on my own case set it flags all 7 auto positives plus 1 default-level false positive): **0** docs.

**Open questions:** Q2 (auto `lo == hi`), Q3 (stored range of unlimited joints), Q4 (`lengthrange`), Q5 (`actuatorfrcrange`), Q6 (muscle actlimited default).

### 3.5 mjcf-S7 (sizes, masses, inertias)

**Now** (measured on main unless noted): `size="-0.1"` loads (body mass −4.19); `box size=".1 0 .1"` loads; a capsule with only `fromto` and no radius loads; a geom with no size gets 0.1 (`types.rs:1332`); geom `mass="-1"` and `density="-1"` load (moving and static); `diaginertia="0 0 0"` on a free body loads; `<inertial>` without `diaginertia` gets 0.001 (`builder/mass.rs:129-134`); `<inertial>` without `mass` gets 1.0 (`types.rs:1111`), without `pos` gets 0; `diaginertia` + `fullinertia` loads (fullinertia wins, `mass.rs:93-127`); `quat`/`euler` + `fullinertia` loads; a non-positive-definite `fullinertia` is `abs()`'d (`1 1 1 2 0 0` → (3,1,1), `mass.rs:104-108`); a triangle violation loads unless `balanceinertia` (`mass.rs:34-46`); moving bodies with zero/1e-16/1e-20 mass or inertia load (the zero-mass ones step to NaN); a plane in a moving body or in a static child of a moving body loads; a plane in a static body adds mass 1.0 and an hfield in a moving body adds ~1.0 (volume fallback 0.001, `geom.rs:405`; MuJoCo 0 and 6001); hfield `size="… 0"` loads; `<inertial mass="0">` on a STATIC body is refused (`validation.rs:223`; MuJoCo loads it).

**Target (3.5.0; measured).**
- checksize after fromto/mesh: plane `size[2] <= 0` → "plane size(3) must be positive"; other types `size[i] <= 0` for the type's count → "size %d must be positive in geom" (`user_objects.cc:150-166`, called `3758`). MuJoCo's default size is 0, so a geom with no size (and no class size) is refused.
- Negative geom mass/inertia/density → "mass, inertia or density are negative in geom" (`3805-3807`, for geoms that contribute inertia).
- "plane only allowed in static bodies" (`3666-3668`; static = weld id 0, i.e. no joint on the body or an ancestor; measured for a static child of a moving body).
- Volume: plane 0; hfield = 8·sx·sy·sz with sz = 0.25·z_top + 0.5·z_bottom (`3115-3200`, `3760-3765`); mesh/sdf = mesh volume.
- `<inertial>`: `pos` and `mass` required (`xml_native_reader.cc:3530-3532`); `fullinertia` with an orientation → error (`3535-3537`, and `user_objects.cc:2405-2406`); `fullinertia` with `diaginertia` → error (`2408-2409`); fullinertia min eigenvalue < 1e-14 → "error 'inertia must have positive eigenvalues' in fullinertia" (`user_util.cc:785-802`, `user_objects.cc:2412-2417`) — also for static bodies (measured).
- Per non-world body: `max(mass, boundmass)`, `max(inertia, boundinertia)`, then negative → "mass and inertia cannot be negative", then triangle → balance or "inertia must satisfy A + B >= C; use 'balanceinertia' to fix" (`2449-2470`) — also for static bodies (measured `static_triangle`).
- Moving body (has a joint) with mass or any principal inertia < mjMINVAL = 1e-15 (`mjtnum.h:26`) → "mass and inertia of moving bodies must be larger than mjMINVAL", unless a static descendant chain carries valid mass (`user_model.cc:4993-4998`, `5310-5330`; measured: static child and static grandchild both rescue; a moving child does not). Runs before `settotalmass` (`5041`).
- hfield size: all 4 > 0 → "size parameter is not positive in hfield" (`user_objects.cc:4471-4474`).
- Deviations, stricter: a NEGATIVE body mass or inertia (from `<inertial>`) is refused for every body. MuJoCo clamps it to 0 silently via `max(·, bound)` (measured: static `diaginertia="-1 1 1"` → [0,1,1]; static `mass="-1"` → 0). Ours already refuses these (`validation.rs:222-243`); keep, change `mass <= 0` to `mass < 0` so a zero inertial mass on a static body loads (parity).

**Change.**
- Parser (`parse_inertial_attrs`, `parser/body.rs:455`): missing `pos` or `mass` → error; `fullinertia` together with `quat`/an orientation → error; `diaginertia` together with `fullinertia` → error. (MjcfInertial keeps its typed defaults for code-built models.)
- `types.rs:1332`: `MjcfGeom::default().size` becomes `vec![]` (MuJoCo default 0); `mass.rs:129-134` default inertia → zeros; `geom.rs:405` volume fallback → per-type MuJoCo volume (plane 0, hfield box, mesh/sdf from mesh, else error). **Needs mjcf-S3 first** (default-class geom size inherited), or every geom that takes its size from a class would be refused.
- `process_geom`: `validate_condim` (§3.8) plus checksize on the resolved size after fromto/mesh, negative mass/density, plane-in-moving-body.
- `apply_mass_pipeline` (`mass.rs:29-77`) reordered to MuJoCo's: per non-world body bound → negative refuse → triangle (balance or refuse) → then `check_moving_body_mass()` → then settotalmass:
  ```rust
  impl ModelBuilder {
      pub(crate) fn apply_mass_pipeline(&mut self) -> Result<()>; // was ()
      /// user_model.cc:4993-4998 + CheckBodyMassInertia 5310-5330; flex bodies are not built yet.
      fn check_moving_body_mass(&self) -> Result<()>;
  }
  ```
- `process_hfield` (`builder/mesh.rs:89`): refuse any size ≤ 0.
- sim-urdf (`sim/L0/urdf/src/converter.rs:449-461`): ALWAYS emit `pos="x y z"` on `<inertial>`, today omitted when zero (`:452`). Without this, 28 of the 37 convertible in-tree URDFs become refusals (measured, MuJoCo "required attribute missing: 'pos'").

**Tests to add.** One `Err` test per bullet above with the measured case (names in `out/all_cases.tsv`: sphere_size_neg, box_size_one_zero, capsule_no_size_fromto, geom_no_size_moving, default_geom_no_size_in_class, geom_mass_neg_moving/static, geom_density_neg_moving, plane_in_moving_body, plane_static_child_of_moving, diaginertia_zero_moving, inertial_no_inertia, inertial_no_mass, inertial_no_pos, diag_and_full, inertial_orientation_and_full, fullinertia_not_pd, static_fullinertia_not_pd, inertia_triangle, static_triangle, moving_body_no_mass, moving_mass_1e-16, moving_inertia_1e-20, moving_nomass_moving_child, inertiafromgeom_false_no_inertial_moving, hfield size z=0) — main: all load except `inertial_neg_mass`/`diaginertia_neg` (already refused). `Ok` tests: moving_mass_1e-14, moving_nomass_static_grandchild, boundmass_rescues (0.1), inertia_triangle with balanceinertia (5/3 each), inertial_mass0_static (main: refused — flips to Ok), mocap geom `mass="0"`, plane_static_mass == 0 (main 1.0), hfield_moving_mass == 6001 (main 2.0).

**Flips.** CI-run tests (shared constants list the tests that use them; counts = `grep -c` of the name minus its definition):
- moving-body mass: `tools/cf-osim/tests/r4_micro_spike.rs:30` `PULLEY_KNEE` (5 uses); `sim/L0/tests/integration/spatial_tendons.rs:85` `MODEL_C` (5), `:112` `MODEL_D` (6), `:261` `MODEL_I` (3); `fluid_forces.rs:119` `MODEL_O` (1), `:185` `MODEL_Z` (1); `builder/compiler.rs:611` (`test_fusestatic_site_with_axisangle`); `builder/mass.rs:328-345` (`test_inertiafromgeom_auto_no_geoms_gives_zero`, deliberately massless); `sensor_phase6.rs:1543-1547` (`t39_framequat_body_vs_xbody`); `fluid_derivatives.rs:1391-1393` (`t23_massless_body_skipped`, deliberately massless).
- hfield size: `builder/mod.rs:1247-1250` (`test_hfield_inline_elevation_still_works`); `raycast_heightfield.rs:14` `HFIELD_MJCF` (3 uses).
- timestep: see §3.9.
- Examples: triangle — `free-joint/stress-test/src/main.rs:29` (**validator**), `free-joint/spinning-toss/src/main.rs:40`, `free-joint/tumble/src/main.rs:46`, `urdf-loading/inertia/src/main.rs:64` (not validators: never run in CI); `<inertial>` without pos — `urdf-loading/stress-test/src/main.rs:120` `ARM_MJCF` (**validator**); hfield — `raycasting/heightfield/src/main.rs:65` (not a validator).
- URDF (runtime, not in the corpus), after the converter emits `pos`: triangle in `urdf-loading/stress-test/src/main.rs:283` `FULL_INERTIA_URDF` (**validator**) and `urdf-loading/inertia/src/main.rs:40` (not a validator); MuJoCo's own URDF importer refuses both too (measured: `from_xml_string` on the URDF → "inertia must satisfy A + B >= C"). Fix the example inertias, or open question Q7.
- `sim/L0/urdf/src/validation.rs:233-236` documents that sim-urdf deliberately accepts triangle-violating inertias; that sentence becomes false for loading (Q7).

**Downstream.** sim-urdf (converter output), cf-osim (test), examples above. `MjcfGeom::default()` size change: 0 construction sites outside sim-mjcf (`git grep`).

### 3.6 mjcf-S12 (duplicate names)

**Now.** `validate()` refuses duplicate body, joint and actuator names but walks only `worldbody.children` (`validation.rs:186-215`, `:301-310`), so a body named `world` and anything inside a `<frame>` escape it; meshes and hfields are refused in the builder (`builder/mesh.rs:45-50`, `:96`); everything else is last-wins in a `HashMap` (`builder/geom.rs:89`, `tendon.rs:21`, `sensor.rs:123-124`, `equality.rs:131/216/276`). Measured on main: duplicate site, tendon, sensor, key, pair, exclude, equality (weld vs connect), frame, body-in-frame vs body, geom-in-frame vs geom, site in world vs body, a user body named `world` (measured through the converted URDFs below) → all LOAD.

**Target.** MuJoCo 3.5.0: per object type, non-empty names unique, "repeated name '<n>' in <type>" (`user_model.cc:4448-4475`; at name set `user_api.cc:1734-1749`; lists `user_model.cc:854-877` — body, joint, geom, site, camera, light, flex, mesh, skin, hfield, texture, material, pair, exclude, equality (all equality kinds share one list), tendon, actuator, sensor, numeric, text, tuple, key, plugin, frame). The world body is named `world`. Different types may share a name (measured: body `x` + geom `x` loads). Checked at XML read, i.e. before discardvisual/fusestatic remove anything.

**Change.**
```rust
/// One pass over every named object after frames and composites are expanded and
/// before discardvisual/fusestatic. Empty names exempt; "world" pre-seeded in bodies.
pub(crate) fn check_unique_names(mjcf: &MjcfModel) -> Result<()>;
```
Replaces the three checks in `validate()` and the two in `builder/mesh.rs` (one rule, one place); error = the errors area's `DuplicateName { kind, name }` (mjcf-E1). Kinds we cannot see because the parser skips the element (camera, light, texture, material, numeric, text, tuple) stay unchecked until the parser area decides decision 19 for them.
sim-urdf converter: (a) a URDF link named `world` maps onto `<worldbody>` (MuJoCo's URDF importer does this, `xml/xml_urdf.cc:727`; measured: MuJoCo loads the in-tree "world"-link URDFs with nbody 2), today it emits `<body name="world">` (4 in-tree URDFs: `urdf-loading/inertia:40`, `stress-test:208,283,305`); (b) geom names made unique within the generated document (`converter.rs:490-535` copies URDF `<collision name>`/`<visual name>`, which URDF does not require to be unique; 0 in-tree URDFs name them, `git grep '<collision name='` = 0).

**Tests to add.** The measured duplicate cases (`dup_*` in `out/all_cases.tsv`) → `Err` naming kind and name; `dup_unnamed_geoms_ok`, `dup_sensor_actuator_cross`, `dup_name_cross_kind` → `Ok`. Converter: a URDF with a `world` root link loads with nbody = 1 + links−1; two collisions named `c` in different links load.

**Flips.** Corpus: 0 static docs (MuJoCo refused none with "repeated name"). `sim/L0/mjcf/src/lib.rs:293-310` and `validation.rs:741-761` assert the old variants/messages (`DuplicateBody`, `DuplicateJoint`, `Unsupported` + "validation failed") — they change with mjcf-E1/this commit. URDF validator `urdf-loading/stress-test` cases at `:208`, `:305` (world link) flip if the converter is not fixed in the same commit.

### 3.7 mjcf-S13 (frames; decision 5: joints/freejoint/inertial directly in `<frame>` stay refused as a stated limitation)

**Now.** `validate()` runs before `expand_frames` (`builder/mod.rs:234` vs `:246`) and its walk never enters `body.frames` (`validation.rs:166-262`). `<joint>`, `<freejoint>`, `<inertial>` directly in a frame are refused at parse (`parser/body.rs:567`, `:595`), pinned by `builder/frame.rs:673/697/716`. Measured on main: a frame-nested geom with `size="nan"` or `size="0"` or `condim="2"`, a duplicate joint in a body inside a frame, a backwards limited range in a body inside a frame, a massless moving body inside a frame, a free body inside a frame inside a body → all LOAD.

**Target.** Every rule in this section applies to frame content exactly as elsewhere (MuJoCo dissolves frames while reading, `xml_native_reader.cc:3629`; measured 3.5.0 refusals for each case above). Decision 5: MuJoCo LOADS joint/freejoint/inertial in a frame (measured: `jnt_pos` [1,0,0]; freejoint njnt 1; an inertial in a frame IGNORES the frame's pos, ipos [0,0,0]); ours keeps refusing, message changed to name it a stated limitation.

**Change.** None of its own: with the per-element rules in the builder (after `expand_frames`) and S12/H3/moving-mass run after expansion (§2), frame content is covered. What moves into frames' scope = everything `validate()` checked per body/geom (duplicate names, inertial checks, geom finiteness) — so `validate()`'s tree walk must either run after `expand_frames`/`expand_composites` on the working copy, or recurse into `body.frames` (pub `validate()` is called on caller-built models: `sim/L0/tests/integration/site_transmission.rs:786`). Placement is the pipeline researcher's call; my requirement is "after expansion", with the composite caveat that this validates generated bodies for the first time (unmeasured flip risk; the corpus has cable composites).

**Tests.** The 7 frame cases → `Err` (main: `Ok`, measured). `frame.rs:673/697/716` stay green (limitation kept); their asserted substring "not allowed inside <frame>" must survive the message change.

### 3.8 mjcf-D1, mjcf-D2 (decision 13: convex hull for a vertex-only mesh)

**D1 Now.** A `<mesh vertex=…>` without `face` is refused, "embedded vertex data requires face data" (`builder/mesh.rs:412-418`), though `lib.rs:95` documents it. 4 corpus docs hit it (`parser/tests.rs:1080`, `:1137`, `:1154`, `:1984`), all parse-only tests (`parse_mjcf_str`), so none flips.
**D1 Target.** MuJoCo: no faces → convex hull (`user_mesh.cc:1567-1574`); fewer than 4 vertices → "at least 4 vertices required" (`1806-1808`); a degenerate (coplanar) set → "qhull error" (`2040`). Measured: 6-vertex set → 6 verts, 6 faces, mass 500.0; 3 vertices and 4 coplanar vertices → refused.
**D1 Change.** In `convert_mjcf_mesh` (`mesh.rs:345`), `face == None` → `cf_geometry::convex_hull(&points, maxhullvert)` (`design/cf-geometry/src/convex_hull.rs:167`, returns `None` for < 4 or degenerate points) → mesh = hull vertices + faces; `None` → error ("at least 4 vertices required" or "degenerate: no convex hull"). Fix `lib.rs:61` (textures/materials are skipped) and `:95`. Depends on ledger-L27 (hull output order is nondeterministic across processes today) landing first, or vertex-only meshes inherit that nondeterminism.
**D1 Tests.** 6-vertex hull loads with nonzero mass; 3 vertices and coplanar → `Err`. Main: all refused with "requires face data".
**D1 open:** Q8 (meshes WITH faces whose hull is degenerate).

**D2 Now / Target / Change.**
- Geom condim: `validate_condim` (`builder/geom.rs:272-308`) warns and rounds 2→3, 5→6, >6→6, ≤0→3. MuJoCo: "invalid condim in geom" unless ∈ {1,3,4,6} (`user_objects.cc:3650-3654`; measured 0, 2 (default class), 5, 7). Change: `fn validate_condim(condim: i32, geom: Option<&str>) -> Result<i32>`. 0 corpus docs use an invalid condim (all `condim=` occurrences in the corpus: "1"×23, "3"×47, "4"×4, "6"×9); no test pins the rounding.
- Pair condim: stored unchecked (`builder/contact.rs:53-55`). MuJoCo "invalid condim in contact pair" (`5666-5669`). Same check in `process_contact`.
- `<contact/>` (self-closing) replaces the model's whole contact block (`parser/mod.rs:183-184`); a non-empty `<contact>` appends (`:156-160`). MuJoCo appends (measured: pair then `<contact/>` → npair 1; two blocks → 2). Change: the empty element is a no-op. The only `<contact/>` in tests is flex-internal (`builder/mod.rs:1390`), a different parser path.
- `.mjb` (`src/mjb.rs:268-281`, feature `mjb`): `bincode::serde::decode_from_std_read(reader, config::standard())` with no limit. bincode 2.0.1 decodes a `String`/`Vec<u8>` by reading a length and allocating it before reading (`impl_alloc.rs:264-276`), so a crafted length allocates that many bytes; serde-driven `Vec<T>` preallocation is capped by serde's size hint. With `config::standard().with_limit::<N>()` the length is claimed against N before allocating (`de/mod.rs:182-186`). Change: `pub const MJB_DECODE_LIMIT: usize = 1 << 28;` and `.with_limit::<MJB_DECODE_LIMIT>()` in `load_mjb_reader`; `LimitExceeded` → `MjcfError::MjbDeserialize`. Not measured (no crafted file was decoded). Also not covered: a crafted `.mjb` with deeply nested `MjcfBody.children` recurses in serde (stack) — mjcf-H2's depth limit must cover `model_from_mjcf`'s input from `.mjb` too. And `.mjb` input bypasses the parse-time non-finite refusal (Q9).
- Found while checking: two `<pair>`s on the same geoms are deduplicated last-wins (`builder/contact.rs:107-120`, pinned by `contact.rs:261` `pair_dedup_last_wins_under_canonical_key`); MuJoCo keeps both (measured npair 2, condims [3, 6]). Q10.

### 3.9 Timestep at load (ledger-L24's sim-mjcf half; decision 21)

**Now.** `validate_option` refuses non-finite and ≤ 0 (`validation.rs:43-48`) and > 1 (`:49-57`). **Target.** MuJoCo loads 0, −0.001, NaN, 1.5 and 2 (measured, no load-time check exists). Drop the `> 1` rule; keep ≤ 0 / non-finite as a stricter deviation. **Test.** `timestep="1.5"` loads (main: refused, measured). **Flip.** `validation.rs:862-867` ("Very large timestep" asserts `Err`), CI-run.

## 4. Found while checking, outside my rows (hand-offs)

1. **Multi-joint bodies diverge from MuJoCo (core area).** Probe `dyn`/`fwd`, Euler, default timestep, qvel = [1.0, 0.5, −0.3, 0.2, …]: one joint per body agrees with MuJoCo 3.5.0 (ball alone: qvel 3.3e-16 after 1 step, 2.8e-14 after 500; hinge body with a ball child: 1.9e-16 and 1.1e-13). Two or more joints on ONE body: qvel differs after ONE step by 2.3e-4 (hinge+hinge), 3.0e-3 (slide+hinge and hinge+slide), 9.2e-4 (3 slides + 3 hinges), 1.1e-3 (hinge→ball), 1.6e-3 (slide→ball, also with gravity off). At t = 0 `qM` agrees to ≤ 2e-17 and `qfrc_bias` does not (hinge+hinge dof 0: MuJoCo 0.6116, ours 0.5528). A sphere centred on the joints shows no difference. Cause not isolated beyond "velocity-dependent bias for same-body joints". 18 corpus docs (ours-ok) have a multi-joint body (20 such bodies: 13 hinge+hinge, 4 three hinges, 1 slide+hinge, 1 hinge+slide, 1 slide+slide); no golden conformance model does (`tools/cf-mjcf-emit/tests/assets/knee_ref.xml` does, but MuJoCo refuses it for `polycoef`). This makes "hinge→ball is supported" (decision 17's carve-out) true for the layout and false for the dynamics.
2. **Muscle activation limits.** MuJoCo 3.5.0 `<muscle>` has `actlimited=0`, `actrange=(0,0)`, `ctrllimited=0` (measured); ours forces `actlimited`, (0,1) (`builder/actuator.rs:161-172`, comment says MuJoCo does this). Q6.
3. **Lengthrange.** MuJoCo keeps an explicit `lengthrange` (measured [0.2, 0.7] for muscle and motor); ours drops it. Without one, MuJoCo leaves a motor at (0,0) and simulates a muscle's (±0.01802 on a ±1° joint); ours copies the joint range for both (`forward/fiber.rs:28-80`, ±0.01745). Q4.
4. **Principal-axis order.** `fullinertia="2 2 2 0.5 0 0"`: MuJoCo body_inertia (2.5, 2.0, 1.5), ours (2.5, 1.5, 2.0) with a different iquat (same physical tensor). Not a validation issue; affects `body_inertia` comparisons.
5. **Tendon springlength.** `springlength="-0.5"` is refused by ours (`parser/tendon.rs:165`, pinned `tendon_springlength.rs:199`) and loaded by MuJoCo 3.5.0 (corpus).
6. **sim-core collision** calls `UnitQuaternion::from_matrix` (unbounded, same as §3.1) at `collision/narrow.rs:175-176`, `mesh_collide.rs:144-145`, `hfield.rs:82,86,111`, `flex_narrow.rs:165,210,224` (`grep`); a NaN `geom_xmat` there is not traced.
7. **Mass pipeline order.** Ours balances before bounding (`mass.rs:34-63`); MuJoCo bounds, then balances (`user_objects.cc:2449-2470`). Fixed by the §3.5 reorder.

## 5. Commit list (this area; each compiles, carries its flips, its must-flip doc, and its 3.5.0 citation)

1. **H1 hang → error.** `principal_axes` + `Result` through `resolve_body_inertia`; `from_matrix_unchecked` (Q1). Tests §3.1 (typed NaN, overflow). Flips: none (bit changes ≤ 1.1e-15 only).
2. **Non-finite refusal (decision 3)** in the parser's float token function (after the parser area's helper conversion). Tests §3.1. Flips: none if the message contains "finite".
3. **Joint layout (H3)** `check_joint_layout` before `build()`. Tests §3.3. Flips: none found.
4. **Joint axis (S9).** Raw axis at parse; `axis too small` in `process_joint`. Tests §3.2. Flips: none found.
5. **`*range` helper (H4, ledger-L1, S6)** incl. free/ball, damper/adhesion, actlimited/dyntype, muscle checks; tendon check moved out of `validate()`. Flips: §3.4 list incl. the joint-limits validator.
6. **Masses/sizes/inertias (S7)** incl. removing the 0.1 / 0.001 / 1.0 / volume defaults, pipeline reorder, moving-body check, hfield size, `<inertial>` attribute rules, sim-urdf converter `pos`. Requires mjcf-S3. Flips: §3.5 list incl. two validators.
7. **Duplicate names (S12)** + sim-urdf `world` link and geom-name dedupe. Flips: lib.rs/validation.rs variant tests (shared with mjcf-E1).
8. **Frames (S13)**: limitation wording; tests that frame content is validated (needs the pipeline researcher's placement).
9. **D2**: geom/pair condim, `<contact/>` append, `.mjb` decode limit (+ Q10 if accepted).
10. **D1**: vertex-only hull (after ledger-L27) + `lib.rs` docs.
11. **Timestep**: drop `> 1`. Flip `validation.rs:862-867`.

Commits 3–11 are independent of each other except 6 → after mjcf-S3, 10 → after ledger-L27; 1 before 2 so commit 2's NaN tests never reach a hang. They can be squashed into the design's "panics/hangs" (1, 3) and "validation after defaults" (2, 4–11) commits.

## 6. Dependencies on other areas

- **Errors (mjcf-E1):** variants for non-finite number (element, attribute, value), axis too small, joint layout (more than 6 dofs / ball followed by rotation / ball followed by slide [limitation] / free not top level), limits (`LimitError` + element kind + name + attribute), inertia/mass/size rules, duplicate name (kind, name), invalid condim (geom/pair), missing attribute (element, attribute), stated limitation (what), mjb limit. `Display` should contain MuJoCo's message text. Commits 1–11 return these; until E1 lands they would be `ModelConversionError { message }`.
- **Parser (S1/S2):** the single float token function (my commit 2 adds the finiteness check inside it, or E1/S2 adds it there with my tests); routing the ~30 direct `.parse()` sites through it; `<inertial>` attribute presence; refusing `actuatorfrcrange`/`actuatorfrclimited` and the other unread attributes (decision 19) — Q5; unknown attributes under `<frame>` (decision 19).
- **Defaults (S3, S10, S11, L26):** S3 before my commit 6; the range helper reads post-default `Option` fields (ctrllimited etc. are already `Option`, `types.rs:2663+`).
- **Pipeline placement (the researcher designing WHERE):** §2's table is the requirement list; S12 before discardvisual/fusestatic; moving-mass before settotalmass and flex; H3 after fusestatic, before `build()`.
- **Core:** fallible joint-layout check and range invariant for hand-edited models (`try_make_data`, core-L7); §4.1 multi-joint dynamics; §4.6 collision `from_matrix`.
- **ledger-L27** before D1.
- **H2 (depth)** must also bound `.mjb` input.

## 7. Open questions (only ones the fixed decisions leave open)

- **Q1 `from_matrix_unchecked` vs bounded `from_matrix_eps`.** Measured: unchecked changes the quaternion by ≤ 1.1e-15 in 75 % of non-diagonal tensors; `from_matrix_eps(…, 64, identity)` was bit-identical in 20,000/20,000 but keeps nalgebra's unbounded inner loop. Recommend **unchecked** (no loop by construction; the input is already orthonormal to rounding). If wrong: the corpus A/B shows an extra ok→ok-changed bucket (non-diagonal inertia docs) that bounded `eps` would have avoided.
- **Q2 auto `lo == hi` (non-zero).** Decision 18 says "backwards"; MuJoCo treats `1 1` exactly like `1 0` (silently unlimited), and explicit `1 1` is an error. Recommend refusing `!(lo < hi)` (equal included). If wrong: an auto `range="1 1"` (0 in the corpus) is refused instead of loading unlimited.
- **Q3 stored range of an unlimited joint.** MuJoCo stores the given range unconverted, (0,0) when absent; ours (−π, π)·deg2rad. Recommend MuJoCo's values (behaviour-free; changes `model_fp` for most models, so check `traj_fp` is unchanged in the A/B). If wrong: model fingerprints churn with no behaviour benefit.
- **Q4 `lengthrange`.** Recommend honouring it (push the parsed value; make `compute_actuator_params` Phase 1 respect `useexisting` first, MuJoCo's order `engine_setconst.c:1164`), and handing the "motor copies joint range / muscle not simulated" difference (§4.3) to core. If wrong (refuse instead): 2 corpus docs carrying `lengthrange` stop loading.
- **Q5 `actuatorfrcrange`/`actuatorfrclimited`.** Recommend refuse as a stated limitation (0 in-tree uses; implementing needs new sim-core Model fields and a clamp in actuation); the helper already has the shape for it (MuJoCo `2927-2943`, `6424-6436`, incl. tendon "range must contain 0"). If wrong: a model using it fails to load instead of loading with unclamped joint actuator force (today it silently ignores it).
- **Q6 muscle `actlimited` default.** Recommend parity for MuJoCo `<muscle>`/`gaintype="muscle"` (no forced limit) and keep the forced (0,1) for our Hill/Millard extensions (documented). Not measured what changes in trajectories. If wrong: muscle activations can leave [0,1] under Euler overshoot where they were clamped before.
- **Q7 URDF triangle violations.** sim-urdf documents accepting them (`validation.rs:233-236`); MuJoCo's own URDF importer refuses them (measured). Recommend refuse (parity) and fix the two example inertias. Alternative: the converter writes `balanceinertia="true"` — silently changes inertia, which the rule calls a refusal case. If wrong: real URDFs with slightly bad inertia fail to load.
- **Q8 meshes WITH faces whose hull is degenerate** (flat mesh used for collision): MuJoCo computes the hull when needed and refuses (`8b802de5…`, `mesh_inertia_modes.rs:341` `t16_degenerate_mesh_shell`; `exactmeshinertia.rs:355` `ac7_zero_volume_degenerate_mesh` after stripping our `exactmeshinertia`). Ours loads with `convex_hull = None`. Not in D1's text. Recommend: in scope only if collision on such a mesh misbehaves today (not measured); otherwise list as a known divergence. If wrong: two CI tests keep pinning a model MuJoCo refuses.
- **Q9 typed and `.mjb` input.** Decision 3 is enforced at parse; `model_from_mjcf` on a caller-built or `.mjb` `MjcfModel` sees no parse. Options: (a) builder guards at every hang/panic point (§3.1, §3.4) + `validate()`'s existing typed checks + docs saying the typed API does not re-check every field; (b) an exhaustive finiteness walk over `MjcfModel` (every field, every type — a new field silently escapes it). Recommend (a). If wrong: a programmatic NaN in, e.g., joint damping loads and NaNs at the first step (sim-core's state check then fires).
- **Q10 duplicate pairs.** MuJoCo keeps both pairs on the same geoms; ours keeps the last. Recommend parity in the D2 commit (flips `builder/contact.rs:261`). If wrong: models with repeated pairs get two contacts per pair instead of one (what MuJoCo does).
