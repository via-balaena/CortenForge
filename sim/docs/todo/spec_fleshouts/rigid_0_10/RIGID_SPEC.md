# Rigid — sim-core and sim-mjcf fixes for 0.10.0 (spec)

**Status:** draft for stress test, 2026-10-06. Base `main` @ `3520544e`. One PR, commits separated by area, ultrareview at the end.

**How to read.** This file is the spec. Appendices A1–A6 are the planning researchers' area sections (read-only research at `3520544e`, with probes, MuJoCo 3.5.0 citations and measured cases). This file settles every question they left open, orders their commits into one series, and wins wherever it disagrees with them. Paths under `$SCRATCH` are the planning session's scratch, not in the repo; each claim is re-established by the tests its commit adds.

| appendix | area |
|---|---|
| [A1](A1_CORE_STATE.md) | sim-core state, lifecycle, BatchSim, docs |
| [A2](A2_CORE_CALLBACKS.md) | sim-core callbacks, finite differences, bad-ctrl check |
| [A3](A3_MJCF_ERRORS_DEFAULTS.md) | `MjcfError`, default classes, where validation runs |
| [A4](A4_MJCF_PARSER.md) | schema pre-pass, attribute reader, allowlist, parser items |
| [A5](A5_MJCF_VALIDATION.md) | compiler-side checks at MuJoCo 3.5.0 |
| [A6](A6_DETERMINISM_VERIFICATION.md) | build determinism, verification protocol, flip table |

---

## 1. Scope

An outside user reviewed the published 0.9.0 crates; the rows below are theirs (triaged), plus items found while planning (P-rows).

| row | kind | what |
|---|---|---|
| core-C1 | docs | callback call counts undocumented (and they differ from MuJoCo — A2 §1) |
| core-C2 | docs | springs+dampers off ⇒ the passive callback never runs |
| core-C3 | bug→parity | `step1` runs the control callback with actuation disabled (MuJoCo does too) |
| core-C4 | API | one slot per callback |
| core-L1 | bug | `energy_initial` never set by `forward` |
| core-L2 | bug | `reset_to_keyframe` is partial |
| core-L3 | API | a Data/Model mismatch panics |
| core-L4 | API | bad state auto-resets (parity, stays); the bad-ctrl check differs from MuJoCo |
| core-L5 | API | Model edits don't update derived caches |
| core-L6 | bug | implicitspringdamper: `forward()` changes qvel |
| core-L7 | bug | plugin data lost on clone; `make_data` panics; RK4 skips plugin advance |
| core-B1 | docs | `step_all` determinism fails with a stateful callback |
| core-B2 | docs | `BatchSim::reset` doc wrong |
| core-B3 | API | `model()` = env 0; no `forward_all`; no shape check |
| core-D1…D4 | docs/API | "immutable Model", "no allocation", 3 small errors; `SimulationConfig` never read |
| mjcf-H1 | bug | NaN/inf geom mass/density/fullinertia hangs `load_model` |
| mjcf-H2 | bug | deep nesting overflows the stack |
| mjcf-H3 | bug | ball then hinge on one body panics in `load_model` |
| mjcf-H4 | bug | ctrlrange lo > hi loads, first step panics |
| mjcf-S1 | bug | typos and unknown elements load silently; `intvelocity` missing |
| mjcf-S2 | bug | unparseable values fall back to defaults |
| mjcf-S3 | bug | default geom `size` not inherited |
| mjcf-S4 | bug | self-closing `<body/>` dropped |
| mjcf-S5 | bug | a 2nd `<compiler>`/`<option>` replaces the first |
| mjcf-S6 | bug | joint/ctrl ranges unchecked |
| mjcf-S7 | bug | impossible sizes, masses, inertias load |
| mjcf-S8 | bug | geometry attributes misread |
| mjcf-S9 | bug | NaN body pos loads; zero axis → Z |
| mjcf-S10 | bug | undefined class loads; "main" unknown; top-level defaults overwrite |
| mjcf-S11 | bug | an explicit value equal to the default is overwritten by the class |
| mjcf-S12 | bug | duplicate names: last wins |
| mjcf-S13 | bug | joints under `<frame>` invisible to validation |
| mjcf-S14 | bug | `<include` inside a comment refused |
| mjcf-S15 | API | an unnamed mesh gets `""` |
| mjcf-E1, E2 | API | errors flattened to `Unsupported`; no location |
| mjcf-D1, D2 | docs→code | vertex-only mesh; condim; empty `<contact/>`; `.mjb` size |
| P-L1 | bug | joint `limited` with no range gets ±π·deg2rad; `ctrllimited` with no range gets (−1, 1) |
| P-L19 | API | `new_per_env` panics undocumented |
| P-L24 | bug | a timestep ≤ 0 reaches `forward`/`step2`/thermostat |
| P-L25 | docs | SDF facts (single-threaded step, `sdf_maxcontact`, octree cell) |
| P-L26 | hang | a classless nested `<default>` loops forever |
| P-L27 | bug | model construction depends on hash order (71 of 1,478 corpus docs vary across processes) |
| P-L28 | compat | 200 of 253 MuJoCo submodule models fail to load |
| P-L31 | panic | `ENABLE_SLEEP` on a factory-built model panics; a ctrl-writing callback zeroes FD `B` |
| P-L32 | physics | any body with ≥ 2 joints diverges from MuJoCo (`qfrc_bias`); **under investigation, §5** |
| P-L33 | bug | hybrid sensor derivatives read stale caches (wrong on `main`, CI green) |
| P-L34 | bug | actuator/sensor delay is parsed but never applied |
| P-L35 | hang/bug | `quat="0 0 0 0"` hangs; `MjcfModel::all_bodies()` skips levels |

Also fixed because the rules need it (A3 §4, A5 §4): defaults ignore `<frame>` composition; `discardvisual`/`fusestatic` read unresolved classes; composite elements take the user's defaults; the composite `curve` keyword map is wrong (`"s"` builds zero-length cables); sim-urdf emits `<inertial>` without `pos` (28 of 37 in-tree URDFs) and a `world` link as a duplicate body.

---

## 2. Decisions (Jon, 2026-10-06)

- **One PR**: *"one pr, separate neatly by commits, then ultrareview at the end"*.
- **The parity rule** (applies to every row): **match MuJoCo, unless MuJoCo panics, hangs, or silently does something other than the file or call asked — then refuse ("stricter"); or does something we cannot — then refuse ("stated limitation"). Every deviation is listed and documented** in the Intentional Divergences table (`sim/docs/MUJOCO_CONFORMANCE.md`).
- **MJCF parity target: MuJoCo 3.5.0** for loading; golden numeric data stay 3.4.0. (276 rule cases agree between 3.4.0 and 3.5.0, A5 §0.)
- **Unknown attributes and elements are errors**, with an allowlist of the extensions in-tree code uses; MuJoCo-valid attributes we never implement are errors (stated limitation).
- **Process**: think it out → spec → compact → stress-test the spec → implement off the stress-tested spec.
- **Visual-only and capacity-hint elements: accepted with no effect** (a third verdict), each listed — `camera light material texture visual skin`, `<statistic>` except `meaninertia`, `<size memory njmax nconmax nkey nstack nuserdata>`.
- **Where MuJoCo itself fails, load:** *"wouldnt it make sense to load all 3? parity doesn't mean we have to inherit its bugs"* — a fourth deviation kind, **lenient**: MuJoCo refuses a valid input through its own defect; we load it and list it, **provided a test shows our result is right**. Applies to MuJoCo's 2-body cable refusal, its non-converging lengthrange, and qhull on a flat mesh. Where such a test cannot be written, the item comes back to Jon.
- **Delay/history (P-L34): implement in Rigid** (MuJoCo 3.5.0's history buffers, delayed actuation and sensors).
- **The known divergences: fix in Rigid** — sleep re-forward and sleep timing, dim-3 tet re-orientation, flex boundary flaps, STL vertex deduplication, principal-axis order of a full inertia, hull face order. ⚠ Hull face order comes from qhull (C/C++, excluded by the no-C++ rule); matching its exact order means porting qhull's tie-breaking. The spec matches and tests the hull as SETS (vertices, faces) and keeps our order unless Jon wants the port.

Consequences the rule settles directly: refuse every NaN/±inf number; refuse ball-then-slide; refuse an automatic backwards range; refuse a timestep ≤ 0 or NaN and drop the `> 1` rule; callbacks follow MuJoCo's order and counts; the bad-ctrl check reads the clamped copy; finite differences refuse RK4; **sensor derivatives take MuJoCo's semantics** (C at the current state, A2 Q4 — flips `tests/integration/derivatives.rs` and the `sensor-jacobians` and `derivatives/stress-test` examples).

---

## 3. Settled in this spec

Each line: the call, why, and where the appendix records what would differ if it is wrong. The stress test should attack these.

| # | question | call | why |
|---|---|---|---|
| 2 | `MjcfError` shape | A3 §1's enum: `#[non_exhaustive]`, every variant struct-shaped and `#[non_exhaustive]`, `String` names, `Location` opaque, `Io{path,source}`, `Unsupported{feature}` reserved for stated limitations; plus `#[cfg(feature="mjb")] MjbTooLarge{size, limit}` | nothing outside sim-mjcf constructs it (A3 §1); A4 §10's names map onto it (§6 below) |
| E2 | locations | message only: line + element path at parse, element + body at build (A3 §2) | no field on `Mjcf*` structs |
| 4 | S11 | 14 actuator + 2 sensor fields become `Option<T>` (`Prefix<N>`/`TfAuto` where partial); sensor defaults stay on the allowlist | A3 §3 |
| 5 | joints/freejoint/inertial directly in `<frame>` | refused, `Unsupported` (limitation); if ever implemented, MuJoCo ignores the frame's transform for `<inertial>` | A5 §3.7 |
| 6 | `energy_initial` | captured once per reset, crate-private flag cleared by `reset` | A1 §5 |
| 7 | Data/Model shape | `StepError::DataShapeMismatch{field, expected, actual}` over **24** lengths on `step`, `step1`, `step2`, `forward`, `forward_skip`; `integrate`/`inverse` documented, unchanged | 5 lengths let a geom-count mismatch through (A1 §1) |
| 8 | derived caches | additive `Model::recompute_derived()` (MuJoCo's `mj_setConst` analogue), trees computed in sim-core, factories call it; geom bounding radii and fixed tendon lengths move into it; cf-design's tree copy switches to it | one derivation for every producer; fixes P-L31 (A1 §7) |
| 9 | plugins | RK4 advances plugins; `try_make_data` + `MakeDataError`; `make_data` runs plugin `reset` (MuJoCo: `mj_initPlugin` then `mj_resetData`); no `Plugin::copy_data` | A1 §6 |
| 10 | per-env BatchSim | `try_new_per_env -> Result<_, PerEnvError<_>>` + panicking twin with `# Panics`; `PerEnvStack` reduced to `install_on`; `model_of`, `forward_all`; `EnvBatch` and the unused `prototype` argument deleted | P-L19 precedent ("Add them all now"), A1 §8 shape C |
| 11 | `SimulationConfig`, `SolverConfig`, `Gravity` | deleted, with sim-mjcf `config.rs`, `ExtendedSolverConfig`, and sim-mjcf's sim-types dependency; `SimError` deleted if it has no consumer at implementation time (measure) | no functional consumer (A1 §9, A3 §5) |
| 12 | chaining callbacks | documented clone pattern (freezes the pub `Callback.0`) | A2 §6 |
| 13 | vertex-only mesh; `.mjb` | convex hull, as MuJoCo; `.mjb` decode size limit → `MjbTooLarge` | A5 §3.8 |
| 14 | L27 hull order | in scope; flex edges in MuJoCo 3.5.0's order; hull in our own deterministic order (MuJoCo's comes from qhull — listed as a limitation); `clippy::iter_over_hash_type` denied in cf-geometry and sim-mjcf; mesh-io STEP keys sorted too | A6 §1 |
| 15 | L30 | the corpus covers all 184 example files with MJCF; this PR verifies them; whether CI loads them later stays open (ledger) | A6 §3 |
| 16 | depth | MuJoCo's limit (self-closing refused at depth 500, open/close at 499); the 6 recursive pub entry points run on a 64 MiB thread; a 4096-deep guard for code-built trees; `all_bodies` iterative | 20.7 MB stack needed at opt-0 (A4 §1.4) |
| A2-Q1 | FD refusal shape | `StepError::UnsupportedIntegrator{integrator}` | derivative fns already return `StepError` |
| A2-Q2 | FD with history | panic → `Err` | P-L19 precedent |
| A3-O1 | actuator default cross-talk | refuse an element whose class actuator default came from a different shortcut kind (limitation; 0 corpus docs) | A3 §7 |
| A3-O2 | repeated top-level defaults | MuJoCo's snapshot-at-creation | parity |
| A3-O3 | `DefaultResolver` | crate-private | re-export later is additive |
| A3-O4 | negative muscle parameters | refused (MuJoCo silently ignores them), except `force`, refused only when the class sets `force ≥ 0` | rule: "silently other than asked" |
| A3-O5 | cylinder `bias[1..3]` | refused when non-zero (MuJoCo reads 3 values into one double) | rule: stricter |
| A4-Q1 | flex `<vertex>`/`<element>` children | implement MuJoCo's `vertex=`/`element=` attributes, rewrite the 81 docs, refuse the children | one spelling |
| A4-Q2 | `muscle@actearly` | allowlist | implemented, 1 use |
| A4-Q3 | `<deformable><flexcomp>` | allowlist (path differs from MuJoCo's body-level `flexcomp`) | 3 docs |
| A4-Q4 | overlay ↔ parser sync | read-tracking assertion in `cfg(test)` | otherwise an accepted attribute can silently go unread again |
| A4-Q5 | `MjcfBody` drop recursion | measure drop alone first; iterative `Drop` if > 512 KB | A4 §11 |
| A4-Q6 | hex floats | refused, limitation (0 uses) | MuJoCo accepts them via `strtod` |
| A4-Q7 | `freejoint damping` (3 examples) | removed from the docs (it was never read; behaviour unchanged) | dead attribute |
| A4-Q9 | weld `site1`/`site2` | refused, limitation (0 uses) | orientation part not worked through |
| A5-Q1 | rotation fit | `from_matrix_unchecked` after the det flip | no loop by construction |
| A5-Q2 | automatic `lo == hi` | refused with lo > hi | MuJoCo treats it as silently unlimited |
| A5-Q3 | stored range of an unlimited joint | MuJoCo's stored values | behaviour-free; check `traj_fp` unchanged |
| A5-Q4 | `lengthrange` | honoured | MuJoCo keeps an explicit one |
| A5-Q5 | `actuatorfrcrange`/`actuatorfrclimited` | refused, limitation (0 in-tree uses) | needs new Model fields |
| A5-Q6 | muscle `actlimited` default | parity for MuJoCo `<muscle>`; forced (0,1) kept for our Hill/Millard extensions, documented | A5 §4.2 |
| A5-Q7 | URDF inertia triangle | refused, as MuJoCo's own URDF importer; fix the two example inertias | parity |
| A5-Q9 | caller-built and `.mjb` input | builder guards at every hang/panic point + docs that the typed API does not re-check every field | an exhaustive walk misses new fields |
| A5-Q10 | repeated `<pair>` | both kept (parity) | flips `builder/contact.rs:261` |
| A6-Q2 | flex element length ≠ dim+1 | refused | MuJoCo `user_mesh.cc:4108-4116` |
| A6-Q4 | Model's hash fields | left | no byte-comparing consumer |
| flex `density` | read nowhere today (71 docs set it, 44 to ≠ 1000) | removed from the docs, then refused by the schema | MuJoCo's `<flex>` has no `density`; dead |
| A1 open | `energy` flag `pub` | `pub(crate)` | A1 §5 |

---

## 4. Open for Jon

**Answered 2026-10-06** (items 1–4, recorded in §2): accepted with no effect; load where MuJoCo fails, with a correctness test; implement delay; fix the divergences. Items below are kept for the record; item 5 is still open.


1. **Visual-only and capacity-hint elements** (`camera light material texture visual skin`, `<statistic>` except `meaninertia`, `<size memory njmax nconmax nkey nstack nuserdata>`). Read literally, "MuJoCo-valid things we don't implement are errors" refuses them: 0 in-tree docs flip, but **47 of the 53 submodule models that load today stop loading** (226 of 253 use one). Proposed: a third verdict, **accepted with no effect**, for elements whose only effect is rendering or allocation capacity, each listed in the divergences table. *Recommend accept-with-no-effect*: nothing the simulation computes differs, so the file is not being silently disobeyed; refusing defeats P-L28.
2. **Where MuJoCo itself fails.** (a) MuJoCo 3.5.0 refuses its own 2-body cables (internal exclude naming, "body 'B_1' not found", 8 corpus docs once `curve` is fixed); (b) lengthrange computation does not converge (4); (c) qhull fails on a flat mesh (1 + 2 CI tests). *Recommend*: (a) load and list (MuJoCo's defect, our result is defined); (b) and (c) parity — refuse — because our value for those quantities is not shown to be right.
3. **P-L34 delay/history.** Implement MuJoCo 3.5.0's history (samples inserted in `mj_advance`, actuation and sensors read delayed values) or refuse `delay > 0` / `nsample > 0` as a stated limitation. *Recommend refuse in Rigid* (23 in-tree docs become refusal tests; the Model/Data fields keep MuJoCo's shape) and implement later as a feature.
4. **Known divergences documented, not fixed here** — each needs Jon's DONE-or-DROPPED in the 0.10 ledger: sleep re-forwards on the sleep step and sleep timing (step 69 vs MuJoCo 76, A2 Q3); dim-3 tet re-orientation and boundary flaps (A6 §1.10); STL vertices not deduplicated (3× MuJoCo on `fourier_n1`); principal-axis order of a full inertia (A5 §4.4); hull face order (limitation). *Recommend*: document all in the divergences table; fix tet orientation and flaps with the flex work after 0.10.
5. **P-L32 multi-joint dynamics** — in Rigid or its own PR. Waiting on the isolation (§5).

Also for Jon's review, though the rule settles them: the 14 earlier decisions are in §3 as recommended; sensor derivatives change semantics (§2).

---

## 5. P-L32 (pending)

Any body with ≥ 2 joints diverges from MuJoCo 3.5.0 after one Euler step (qvel 2.3e-4…3.0e-3); one joint per body agrees to ~1e-16; at t = 0 `qM` agrees to ≤ 2e-17 and `qfrc_bias` does not (hinge+hinge dof 0: 0.6116 vs 0.5528). Cause not isolated (A5 §4.1). An investigation is finding the first differing intermediate quantity and testing a fix in a scratch copy; this section is filled when it reports.

---

## 6. The error type (mjcf-E1) — reconciliation

A3 §1 is the enum. A4 §10's requested variants map as: `UnknownElement{path}` → `UnknownElement{element, parent, at}`; `UnsupportedElement`/`UnsupportedAttribute` → `Unsupported{feature, at}` (feature text names the element/attribute); `DuplicateElement` → `RepeatedElement`; `DepthExceeded` → `TooDeep`; `AttributeLength` → `WrongValueCount`; `BadNumber`, `InvalidKeyword`, `MissingAttribute` as named. A5 §6's: non-finite → `NonFinite`; axis, limits, condim, inertia/mass/size → `InvalidValue`; joint layout → `InvalidModel`; duplicate name → `DuplicateName`; limitations → `Unsupported`; `.mjb` → `MjbTooLarge`. Display texts keep the 81 asserted substrings (A3 §1).

---

## 7. Commit series

Core first, then MJCF. Titles carry no `!` (the commit-msg hook refuses it); a breaking change says so in its body. Each commit compiles, is green alone, carries the tests it flips, its must-fail test(s), and (MJCF) its MuJoCo 3.5.0 citation. "Must-fail" = fails at the parent, passes at the commit. **The MJCF harness baseline is taken at the head of the core series, not at `main`** (K13 changes multi-joint trajectories if it lands).

### Core series

| # | commit | rows | detail | depends |
|---|---|---|---|---|
| K1 | `fix(sim-core): one pre-step check on every Result entry point; thermostat refuses h ≤ 0` | core-L3, core-D3 (display), P-L24 | A1 §1–2 | — |
| K2 | `fix(sim-core): hybrid sensor derivatives run every stage` | P-L33 | A2 §7 | — |
| K3 | `fix(sim-core): implicitspringdamper forward() leaves qvel unchanged` | core-L6 | A1 §3 | K1 |
| K4 | `fix(sim-core): reset_to_keyframe resets fully; energy_initial captured once per reset` | core-L2, core-L1 | A1 §4–5 | K1 |
| K5 | `fix(sim-core): RK4 advances plugins; Model::try_make_data` | core-L7 | A1 §6 | — |
| K6 | `feat(sim-core): kinematic trees in sim-core; Model::recompute_derived` | core-L5, P-L31 | A1 §7 (+ radii, tendon lengths, cf-design) | — |
| K7 | `feat(sim-core): callbacks fire in MuJoCo's order and counts` | core-C1, C2, C3 | A2 §1–2 | K2 (else hybrid `D` ships wrong, measured) |
| K8 | `feat(sim-core): finite differences refuse RK4 and history` | core-C1 (FD) | A2 §3 | K7 |
| K9 | `feat(sim-core): sensor derivatives at the current state` | rule (A2 Q4) | to be specified from A2 §4 at implementation; FD per-column `forward()` goes | K8 |
| K10 | `fix(sim-core): the bad-ctrl check reads the clamped input and leaves ctrl alone` | core-L4 | A2 §5 | — |
| K11 | `feat(sim-core): per-env BatchSim returns errors; model_of, forward_all` | core-B3, P-L19 | A1 §8 | K1 |
| K12 | `refactor: delete SimulationConfig, SolverConfig, Gravity` | core-D4 | A1 §9 + A3 §5 (sim-mjcf `config.rs`) | — |
| K13 | `fix(sim-core): multi-joint bodies match MuJoCo's bias force` | P-L32 | §5 | if Jon puts it in Rigid |
| K14 | `docs(sim-core): …` | core-C1, C4, B1, B2, D1, D2, D3, P-L25, P-L31 (FD `B`) | A1 §10, A2 §2, §4, §6, §8 | all K |

### MJCF series

| # | commit | rows | detail | depends |
|---|---|---|---|---|
| M1 | `feat(sim-mjcf): typed MjcfError with locations` | mjcf-E1, E2 | A3 §1–2 | — |
| M2 | `fix(cf-geometry): convex hull no longer depends on hash order` | P-L27 | A6 §1, gate | — |
| M3 | `fix(sim-mjcf): flex edges in MuJoCo's order` | P-L27 | A6 §1 (+ A6-Q2 refusal) | M1 |
| M4 | `fix(sim-mjcf): a non-finite inertia is an error, not a hang` | mjcf-H1 (hang) | A5 §3.1 `principal_axes` | M1 |
| M5 | `fix(sim-mjcf): depth-safe traversals on a large stack` | mjcf-H2 (part 1), P-L35 (`all_bodies`) | A4 §1.4, commit 1 | M1 |
| M6 | `feat(sim-mjcf): one attribute reader with MuJoCo's rules` | mjcf-S2 (numbers, lengths, required), P-L28 friction, mjcf-S8 (zero quat, multiple orientations), P-L35 (zero-quat hang) | A4 §2, commit 2 | M1 |
| M7 | `feat(sim-mjcf): every non-finite number is refused` | mjcf-S9, mjcf-H1 (decision) | A5 §3.1 | M6 |
| M8 | `feat(sim-mjcf): keywords exact; composite curve keywords as MuJoCo` | mjcf-S2 (keywords), curve map | A4 commit 3 | M6 |
| M9 | `fix(sim-mjcf): self-closing elements are not dropped` | mjcf-S4 | A4 §8.2 | M1 |
| M10 | `fix(sim-mjcf): repeated <compiler>/<option> merge` | mjcf-S5 | A4 §8.3 | M1 |
| M11 | `feat(sim-mjcf): defaults resolved in one pass before frames and composites` | mjcf-S10, P-L26, frame/composite/compiler-pass order bugs | A3 §3–4 (`validate_resolved` home) | M9 |
| M12 | `feat(sim-mjcf): explicit values survive their class` | mjcf-S11 | A3 §3 | M6, M8, M11 |
| M13 | `feat(sim-mjcf): default geom size, element-wise overlay, size must be positive` | mjcf-S3 | A3 §3 | M8, M11 |
| M14 | `feat(sim-mjcf): MuJoCo forms` | mjcf-S1 (`intvelocity`), mjcf-S8 (xyaxes/zaxis, box fromto, plane size), mjcf-S15, P-L28 (connect `site1`/`site2`) | A4 commit 6 | M6 |
| M15 | `feat(sim-mjcf): schema pre-pass at MuJoCo 3.5.0 with the extension allowlist` | mjcf-S1, H2 (depth), S14, decision 19, §4 item 1 | A4 §1, §3–5, commit 7 | M14 |
| M16 | `feat(sim-mjcf): joint layout as MuJoCo` | mjcf-H3 | A5 §3.3 | M11 |
| M17 | `feat(sim-mjcf): joint axis too small is an error` | mjcf-S9 | A5 §3.2 | M11 |
| M18 | `feat(sim-mjcf): one limit helper for joints, tendons, actuators` | mjcf-H4, S6, P-L1 | A5 §3.4 | M12 |
| M19 | `feat(sim-mjcf): sizes, masses and inertias as MuJoCo; sim-urdf writes inertial pos` | mjcf-S7 | A5 §3.5 | M13 |
| M20 | `feat(sim-mjcf): duplicate names are errors; sim-urdf world link` | mjcf-S12 | A5 §3.6 | M11 |
| M21 | `fix(sim-mjcf): frame contents are validated` | mjcf-S13 | A5 §3.7 | M11 |
| M22 | `fix(sim-mjcf): condim, <contact/> appends, repeated pairs, .mjb limit` | mjcf-D2 | A5 §3.8 | M11 |
| M23 | `feat(sim-mjcf): a vertex-only mesh gets its convex hull` | mjcf-D1 | A5 §3.8 | M2 |
| M24 | `fix(sim-mjcf): timestep > 1 loads` | P-L24 (mjcf half) | A5 §3.9 | M1 |
| M25 | `feat(sim-mjcf): delay and history` | P-L34 | §4 item 3 | M11 |
| M26 | `docs: MuJoCo divergences, sim-mjcf docs, conformance status` | D-docs, the divergences table, the stale "Real-World Model Loading ✅ COMPLETE" (its tests were deleted in `b56753d1`; 1 of the 16 listed Menagerie models loads) | A1 §10, A4, A5 | all |

---

## 8. Verification

A6 §4 is the protocol; the rules it enforces:

- **Before the first commit**: baseline on `main` — corpus harness ×10 (the determinism mask, 71 docs), repo and submodule `.xml` (streaming-fingerprint harness only; a `{:#?}` fingerprint of `fourier_n1` is 2.39 GB of text), MuJoCo 3.5.0 oracle, `cargo xtask licensed-gates --run` (main carried 4 red gates when last run, 2026-09-21: a different set is a new baseline, not a regression), the 24 ignored golden-flag tests' residuals, `--features mjb`, `--no-default-features`, the full validator fleet.
- **Per MJCF commit**: the five buckets (unchanged · ok→ok changed must be on the commit's pre-registered expected-change list · ok→err must map to the commit's rule · err→ok must be MuJoCo-ok · err→err message change), and NEW NONDETERMINISTIC empty from M3 on. Compare MuJoCo by status, not message (its message varies for 2 docs).
- **Per commit, every rule**: its must-fail test made to fail at the parent, written down before the code.
- **Runtime-generated MJCF** (157 `format!` templates, cf-design, cf-mjcf-emit, sim-urdf, therm-env, cf-codesign, cf-fsu-model, cf-osim): capture in a detached scratch worktree with an uncommitted dump patch; the push guard `grep -c CF-SCRATCH-MJCF-DUMP` must print 0 on the PR diff, after being made to print ≥ 1 on the worktree.
- **Oracle**: every new refusal is MuJoCo-refused or on the divergences list; no MuJoCo-ok doc newly fails unless listed.
- **Before push**: grade every touched crate and its downstream (`RAYON_NUM_THREADS=1`), clippy exactly as the hook runs it, doc-theft, licensed gates at the head, the review rounds.

---

## 9. What planning could not see

No Rust for the MJCF areas was written (A4's flip list comes from a Python model of the parser, checked against MuJoCo); the core areas' patches were applied to scratch copies but per-commit greenness in this order was not run; the 157 templates and runtime MJCF were not captured; ~130 corpus docs hide later errors behind MuJoCo's first; the opt-0 stack for composites; downstream suites (sim-gpu, sim-coupling, cf-design, cf-codesign, sim-rl, sim-opt, therm-env); grade; licensed gates. A `<composite type="cable" count="1000">` reached ~2 GB RSS during planning, cause not isolated — run it only under an RSS watchdog.
