# Every open question, and its answer

> **Answers to the 37 questions classed (c) (2026-10-07).** Jon answered four (the wrong-value rule, FMA, flex meaning, ultrareview — see *The decisions*), and two more on 2026-10-07 (Q103, Q122: refuse, as MuJoCo); the rest follow from the parity rule or a technical reason:
>
> | Q | answer |
> |---|---|
> | Q01 | `integrate` returns `Result` (C3) |
> | Q08 | (a) additive `recompute_derived` (11-settled #8; not objected to) |
> | Q16 | fallible via `try_make_data` (C8) |
> | Q17 | fallible range check in `try_make_data` and `recompute_derived` (Jon's "a hard error, not silent") |
> | Q18, Q24, Q26, Q48, Q111, Q115 | **wrong-value rule** (Jon): keep ours, listed with its test |
> | Q43 | wrong-value rule, with the test: a body welded to the world at rest reads the same as one jointed at rest (+g) |
> | Q44 | wrong-value rule only if a test against an independent truth is written; otherwise parity |
> | Q20 | yes — MuJoCo has these functions (parity) |
> | Q22, Q23 | refuse at `try_make_data`/reset, as MuJoCo refuses (parity) |
> | Q28, Q30 | islands computed always, one partition shared by the solver and sleep (parity; A18's island-wise solve) |
> | Q40 | implement (C5) |
> | Q46, Q69, Q72 | parity (massless-body inertial frame; `mj_broadphase`'s touching test; normal renormalisation) |
> | Q54 | plain arithmetic (C1) |
> | Q63 | adopt MuJoCo's mesh frame in A10's mass commit — `geom_pos`/`geom_quat` of mesh geoms change meaning (listed as a 0.10 break) |
> | Q67 | `HeightFieldData` gains x/y spacing |
> | Q73 | fix in Rigid (fix every divergence) |
> | Q81 | MuJoCo's meaning (C7) |
> | Q90–Q95 | implement (C2) |
> | Q96 | `eig3` (C6) |
> | Q103 | refuse where MuJoCo needs a hull; load the shell-without-collision case MuJoCo loads (Jon, 2026-10-07: no test can show our result right, A10 §5) |
> | Q118 | load ASCII STL and > 200,000 faces under the lenient kind, each with a test that the result equals the binary/split form |
> | Q122 | refuse, as MuJoCo (Jon, 2026-10-07: no right value exists, A11 §LR-3) |
> | Q129, U12 | refused as stated limitations (non-scalar ball/free transmission; flex membrane modes we do not implement) |


Every open question the 21 research sections raise, deduplicated, with the sections, the options, the sections' recommendation, and a class. Input: *A Double Dose of Detail* at `caa1c1c0`. Commit ids `P…`/`L…` refer to 20-rigid-physics.md and 22-rigid-loading.md; conflicts `C1…C11` are described there.

## How the class was assigned

- **(a) settled by a fixed decision.** The answer follows from 01-decisions: the parity rule (match MuJoCo) or one of its four deviation kinds — **stricter** (MuJoCo panics, hangs or silently does other than asked → refuse), **stated limitation** (MuJoCo-valid, we do not implement → refuse), **accepted with no effect** (visual-only and capacity hints), **lenient** (MuJoCo refuses a valid input through its own defect → load, if a test shows our result right) — or a named decision (P-L32 in Rigid; fix the known divergences; fix every census cluster; sensor-derivative semantics; delay; hull order).
- **(b) technical.** A clear recommendation with no owner-level consequence. Where 11-settled already adopted it, that is said; 11-settled records calls for review, not fixed decisions (12-open-for-jon: "the 14 earlier decisions are in 11-settled as recommended").
- **(c) needs Jon.** It changes scope, changes what a public API means to existing callers, is a deviation outside the four kinds, has no recommendation, or two sections disagree.

**Tally: 130 questions — (a) 47, (b) 46, (c) 37.** Separately, 17 findings no section's commit list carried (end of file): (a) 9, (b) 7, (c) 1; the series now carries all 9 (a) rows and the (c) row.

## 1. sim-core state, API and callbacks

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q01 | `Data::integrate` return type | A1 §1(a); A8 SP-8, Q4; A13 Q2 | document only / `-> Result<(), StepError>` / keep `()` and carry `qacc_implicit` | A1: document; A8: `Result` (the sleep step's re-forward can fail); A13: option B, partly because it "keeps `integrate()` returning `()`" | **(c)** | C3: sections disagree; 11-settled #7 says "documented, unchanged" |
| Q02 | `DataShapeMismatch` without a variant-level `#[non_exhaustive]` | A1 §1(b) | add / leave | leave (tests construct the variant; ledger-L20 precedent) | (b) | 11-settled #7 adopts the variant |
| Q03 | `energy_initial` capture flag public? | A1 §5 | `pub` / `pub(crate)` | `pub(crate)` | (b) | adopted, 11-settled "A1 open" |
| Q04 | `make_data` runs `Plugin::reset` | A1 §6 | yes / no | yes (MuJoCo: `mj_initPlugin` then `mj_resetData`) | (b) | adopted, 11-settled #9 |
| Q05 | factories and `finalize` call the full `recompute_derived` | A1 §7(a) | full / trees + `dof_length` only | full (both measured, 0 flips) | (b) | adopted, 11-settled #8 |
| Q06 | move geom radii, fixed tendon lengths and history addresses into `recompute_derived` | A1 §7(b); A7 Q5 | move / keep sim-mjcf-private | move (model-only functions) | (b) | 11-settled #8 adopts the first two ("not in the measured patch"); A7 Q5 is not in 11-settled |
| Q07 | cf-design's private tree copy → `compute_kinematic_trees` | A1 §7(c) | switch / leave | optional; cf-design tests not run | (b) | adopted, 11-settled #8 |
| Q08 | decision 8: additive `recompute_derived` (a) vs removing the `jnt_damping`/`dof_damping` split (b) | A1 §7(d) | (a) / (b) | none: "stays Jon's" | **(c)** | A1 assigns it to Jon; 11-settled #8 records (a) for review |
| Q09 | per-env `BatchSim` trait shape | A1 §8 | C (`install_on`, sim-core owns the loop) / B | C | (b) | adopted, 11-settled #10 |
| Q10 | delete `SimError`/`Result` and `ExtendedSolverConfig` too | A1 §9 | delete / keep | delete | (b) | adopted, 11-settled #11 (`SimError` "if no consumer… (measure)") |
| Q11 | shape of the FD RK4 refusal | A2 Q1 | `StepError::UnsupportedIntegrator` / a new `DerivativeError` | `StepError` variant | (b) | adopted, 11-settled A2-Q1 |
| Q12 | FD with `nhistory > 0`: panic → `Err` | A2 Q2 | convert / leave | convert | (b) | adopted, 11-settled A2-Q2 |
| Q13 | sleep re-forward on the sleep step | A2 Q3 | match now / document | document now, leave the restructure to sleep work | (a) | 01-decisions: "fix in Rigid — sleep re-forward and sleep timing" (A8 S3 = P22) |
| Q14 | sensor-derivative semantics | A2 Q4 | ours (next-step sensors) / MuJoCo's (current state) | raise to Jon | (a) | 01-decisions: "sensor derivatives take MuJoCo's semantics" (P14) |
| Q15 | where the hybrid stale-cache fix gets its row | A2 §7 | — | "needs a row (Jon / the planner decides where)" | (b) | 10-scope lists it as P-L33; P06 carries it (K2 of the single series, `20-commit-series.md` at `caa1c1c0`) |
| Q16 | sim-core joint-layout check: panic or `Result` | A1 §6; A5 §3.3 | keep `# Panics` / `check_joint_layout -> Result<(), JointLayoutError>` via `try_make_data` | A1: panic stays; A5: fallible | **(c)** | C8: sections disagree |
| Q17 | range invariant (limited ⇒ finite, lo ≤ hi) for a hand-edited `Model` | A5 §3.4, §6 | fallible model check / silent clamp | fallible check, "owned by core" | **(c)** | no core section takes it; adds a refusal at `try_make_data`/pre-step |

## 2. Delay and history

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q18 | RK4 with a delayed sensor | A7 Q1 | (C) sample at step start / (R) refuse / (P) reproduce MuJoCo's stage-stale sample | C, as a listed deviation backed by a bitwise test | **(c)** | "the fixed rule read literally says R … this is Jon's call" (A7 Q1); C is outside the four kinds |
| Q19 | `delay > nsample·timestep` | A7 Q2 | refuse / parity | refuse; tolerance `(1+1e-12)` is "my choice, no MuJoCo referent" | (a) | stricter: MuJoCo silently acts on a shorter delay than written (A7 §1.7) |
| Q20 | the four history API functions in Rigid | A7 Q3 | yes / no | yes ("without them, history-only mode has no reader") | **(c)** | scope; freezes 4 methods + `HistoryError` in 0.10 |
| Q21 | `InterpolationType: From<i32>` → `TryFrom<i32>` | A7 Q4 | yes / no | yes (0 callers) | (b) | — |
| Q22 | user/plugin sensors with `delay > 0` in a code-built `Model` | A7 Q6 | document as limitation / refuse in the pre-step check (O(nsensor)) | document | **(c)** | MuJoCo errors; the recommendation loads — outside the four kinds |
| Q23 | `make_data`/`reset` with `nhistory > 0` and `timestep ≤ 0` | A7 Q7 | new error / none | none (P04 refuses at stepping; sim-mjcf at load) | **(c)** | MuJoCo refuses at reset; accepting until stepping is outside the four kinds |
| Q24 | MuJoCo's stale `mj_step1`+`mj_step2` accelerometer (Δ 5.9) | A7 §8 F3 | list as a divergence / nothing | "A2 may want to list it as a lenient divergence" | **(c)** | MuJoCo does not refuse, so not the lenient kind; no owner |

## 3. Sleep

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q25 | `qacc` of a sleeping dof | A8 Q1 | MuJoCo's stale `qacc_smooth` / 0 | parity | (a) | parity rule (flips 6 CI tests + 1 validator check) |
| Q26 | RK4 with sleep | A8 Q2; A19 §5.4 | parity (584 transitions) / refuse / keep asleep | A8: parity; A19: keep asleep (lenient), "A8's Q2 option (a) is replaced" | **(c)** | C4; MuJoCo does not refuse, so "lenient" is outside the decision's wording |
| Q27 | init trees that cannot sleep | A8 Q3 | refuse (`MakeDataError::InitSleep`) / warn | refuse | (a) | parity: MuJoCo refuses at load |
| Q28 | islands when sleep is disabled | A8 Q5 | compute as MuJoCo / leave, list | leave, list as a readout divergence | **(c)** | a deviation outside the four kinds |
| Q29 | a tree woken during an RK4 stage | A19 Q5.1 | snapshot at step start / live flags | snapshot ("not measured") | (b) | — |
| Q30 | one island partition (efc rows, A18 P2) or two (sleep's `mj_island`) | A18 §14 | one / two | none: "the sleep area's call"; A8 does not address it | **(c)** | no recommendation; two sections own the data |

## 4. Dynamics and kinematics

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q31 | GPU `rne.wgsl` in the P-L32 commit (P24) | A12 Q1 | now / later | now, per joint | (b) | adopted by 21-multi-joint-bias |
| Q32 | pre-existing hinge-then-slide derivative gap | A12 Q2 | own ledger row | — | (b) | isolated and fixed by A14 L36a (P25) |
| Q33 | P-L32 in Rigid or its own PR | A12 Q3 | Rigid / own PR | Rigid | (a) | 01-decisions default: "P-L32 in Rigid" |
| Q34 | A13's D1, D2, D3 in Rigid | A13 Q1 | in / ledger | D1, D2 in; D3 only if the flex rewrites land | (a) | 01-decisions: "P-L32 in Rigid with every other isolated fix"; D3's condition is carried into L31 |
| Q35 | D2's shape | A13 Q2 | A: solve in `integrate()` / B: crate-private `qacc_implicit` | B (measured, 0 flips) | (b) | its stated premise touches Q01 |
| Q36 | `ConstraintType::FlexEdge` | A13 Q3 | keep label / `Equality` with `efc_id` = equality id | `Equality` | (b) | breaking anyway; users are sim-core + 2 tests |
| Q37 | which flex docs get `<equality><flex>` | A13 Q4 | add where tests need edges / let rows go | add; "which of the 75 those are was not determined" | (b) | — |
| Q38 | `equality="vert"` / `<flexvert>` | A13 Q5 | refuse / implement | refuse | (a) | stated limitation |
| Q39 | where A14's new harness cases go | A14 Q1 | shared matrix / CPU-only list | CPU-only | (b) | — |
| Q40 | ball joint with `pos ≠ 0` | A14 Q2; A19 §2.3 | refuse (limitation) / implement | A14: refuse; A19: implement on CPU | **(c)** | C5 |
| Q41 | `ref`/`qpos0` in forward kinematics, in Rigid | A14 Q3 | Rigid / later | — (A14: "Jon's call") | (a) | 01-decisions "fix every census cluster": L45ref is one (A16 §2); A19 R2 = P28 |
| Q42 | offset ball on the GPU | A19 Q2.1 | implement / sim-gpu refuses | refuse | (a) | stated limitation (applies only if Q40 = implement) |
| Q43 | accelerometer on a world-welded body | A19 Q1.1 | keep ours (+g) / MuJoCo's 0 | keep ours | **(c)** | MuJoCo does not refuse; called "lenient" but outside the four kinds |
| Q44 | world body `cfrc_int[0]` | A19 Q1.2 | keep ours (transported) / MuJoCo's mixed reference | keep ours | **(c)** | deviation outside the four kinds |
| Q45 | quaternion tangent convention of transition `A` | A19 Q3.1 | MuJoCo's / ours | MuJoCo's | (a) | parity rule; values of public `mjd_quat_integrate`, `mj_differentiate_pos` change |
| Q46 | inertial frame of a massless body | A20 "Open for Jon" 1 | copy MuJoCo's parent-frame pose / keep ours, list | keep ours | **(c)** | deviation outside the four kinds |
| Q47 | ellipsoid `shellinertia` | A20 "Open for Jon" 3 | MuJoCo's ε-shell / ours | parity | (a) | parity rule |
| Q48 | `fusestatic`: MuJoCo breaks its own "fused == unfused" invariant | A16 Q1; A20 "Open for Jon" 2 | parity with the defect / keep the invariant, list | keep the invariant | **(c)** | a correctness deviation, outside the four kinds |

## 5. Constraint solver

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q49 | islands in Rigid | A18 Q1 | in / monolithic only | in (0 census verdicts differ either way) | (b) | — |
| Q50 | `Data::newton_solved` | A18 Q2 | delete / crate-private / keep pub | crate-private | (b) | always true without the PGS fallback |
| Q51 | solver statistics shape | A18 Q3 | MuJoCo's per-island arrays / sum + concatenation | sum + concatenation, MuJoCo's `SolverStat` fields | (b) | — |
| Q52 | mixed-sign `solref`/`solreffriction` | A18 Q4 | parity (replace + warn) / refuse at load | parity | (a) | parity: MuJoCo warns, so it is not silent |
| Q53 | PGS/noslip scale and `Data::stat_meaninertia` | A18 Q5 | MuJoCo's constant / leave | leave until a fixture fails (0 of 1,408 docs change) | (b) | — |
| Q54 | emulate the reference build's FMA | A10 Q2.1; A17 §2.2, Q1; A18 §3.4, Q6; A21 §1.3, Q2 | `f64::mul_add` where clang contracts / plain arithmetic | A10, A17: emulate; A18, A21: accept | **(c)** | C1 |
| Q55 | solver: full port vs patches | A18 Q7 | port / patches | port | (b) | — |

## 6. Collision

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q56 | MULTICCD | A15 Q1; A17 §4, Q4 | port / ledger row | A15: ledger row; A17 ported the perturbation search | (b) | A17 C1/C3 (P48/P50) |
| Q57 | port `mjraw_CapsuleBox` | A15 Q2 | port / ledger row | port | (a) | parity; census NEW-PAIRCOUNT; A21 R5 = P45 |
| Q58 | sphere–box position offset and deep branch | A15 Q3 | fix / list | fix | (a) | census NEW-POS; A21 R4 = P44 |
| Q59 | height fields | A15 Q4 | ledger row | — | (b) | A17 isolated and ported it (P52, L45) |
| Q60 | contact inside an ellipsoid's margin | A15 Q5 | ledger row | — | (b) | A21 G (P47) and A17's port |
| Q61 | contact geom order | A15 Q6; A21 §7.2 | MuJoCo's type order / index order | A15: none; A21: MuJoCo's | (a) | parity rule (A21 R1 = P41); the CCD-port interaction is C10 |
| Q62 | degenerate pairs (concentric spheres, crossing capsules) | A16 Q2 | match MuJoCo | match | (a) | parity |
| Q63 | MuJoCo's mesh frame (re-centring at the mesh CoM/principal frame) | A17 Q2; A20 "Open for Jon" 4 | adopt fully in A10's mass commit / give the CCD the CoM only / list as a representation divergence | A17: adopt fully; A20: no owner | **(c)** | no commit owns it (A10 MH1–MH5 do not); changes mesh `geom_pos`/`geom_quat` meaning |
| Q64 | qhull's hull graph | A17 Q3 | port / limitation | limitation | (a) | 01-decisions: keep our hull order unless Jon wants the port |
| Q65 | mesh polygons + mesh multi-contact in Rigid-physics | A17 Q4 | port / ledger row | port (C3 without it reddens a test) | (b) | — |
| Q66 | `<flag nativeccd="disable">` | A17 Q5 | refuse / port MPR / silent no-op | refuse | (a) | stated limitation; the rule forbids the silent no-op |
| Q67 | non-square height-field cells | A17 Q6 | `HeightFieldData` gains x/y spacing / refuse dx ≠ dy | spacing | **(c)** | a public change in a published crate vs a limitation |
| Q68 | `mod.rs:1602` (mesh with no hull) | A17 §10 | MuJoCo's no-graph semantics / keep "no hull → none" as a deviation | not stated | (a) | parity; keeping it is a deviation outside the four kinds |
| Q69 | MuJoCo's broadphase culls touching pairs | A21 Q1 | port `mj_broadphase` / keep ours, "lenient" | keep ours | **(c)** | MuJoCo does not refuse; outside the four kinds |
| Q70 | capsule–cylinder routing | A21 Q3 | via EPA now / analytic / with the CCD port | with the CCD port | (b) | A17 C3 does it (P50) |
| Q71 | `contacts_*` helpers count excluded contacts | A21 Q4 | count all / filter | count all | (b) | — |
| Q72 | renormalise the stored contact normal | A21 Q5; A17 §2.3, §12 | leave / renormalise | A21: leave; A17: needed for bitwise normals | **(c)** | C9 |
| Q73 | contact order by MuJoCo's body-pair signature | A21 Q6 | Rigid / ledger row | ledger row | **(c)** | scope deferral of a known divergence |
| Q74 | MuJoCo's box–box filter empties a deep overlap | A21 Q7 | parity / lenient keep | parity + ledger row | (a) | parity |

## 7. MJCF errors, defaults and parser

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q75 | actuator default cross-talk | A3 O-1 | port MuJoCo's shared default / refuse / leave | refuse | (a) | stated limitation (11-settled A3-O1) |
| Q76 | repeated top-level defaults | A3 O-2 | MuJoCo's snapshot / live inheritance | snapshot | (a) | parity |
| Q77 | `DefaultResolver` public | A3 O-3 | public / crate-private | crate-private | (b) | adopted, 11-settled A3-O3 |
| Q78 | negative muscle parameters | A3 O-4 | refuse / parity ("negative = None") | refuse except `force` | (a) | stricter: MuJoCo silently replaces them |
| Q79 | cylinder `bias[1..3]` | A3 O-5 | refuse / parity | refuse | (a) | stricter |
| Q80 | 2-body cables MuJoCo refuses | A3 O-6; A9 §3 | load / refuse | load | (a) | lenient, named in 01-decisions; test A9 §3.5 |
| Q81 | flex attribute form and what `<flex body=…>` means; also `node=`, `<elasticity damping>`, an empty `element`, element- vs vertex-based flex contacts | A4 Q1; A20 §1B.4, §1B.7, "Open for Jon" 5 | MuJoCo's meaning + explicit vertex bodies / keep ours as an extension, flex outside the census | A4: rewrite as "same meaning"; A20: the meaning differs, a spec decision is missing | **(c)** | C7 |
| Q82 | `muscle@actearly` | A4 Q2 | allowlist / rewrite | allowlist | (b) | adopted |
| Q83 | `<deformable><flexcomp>` | A4 Q3 | allowlist / rename | allowlist | (b) | adopted |
| Q84 | overlay ↔ parser sync guard | A4 Q4 | read-tracking in `cfg(test)` / none | read-tracking | (b) | adopted |
| Q85 | iterative `Drop` for `MjcfBody` | A4 Q5 | add / leave | measure drop alone; add if > 512 KB | (b) | adopted |
| Q86 | hex floats | A4 Q6 | refuse / implement | refuse | (a) | stated limitation |
| Q87 | `freejoint damping` in 3 examples | A4 Q7 | remove / `<joint type="free" damping>` | remove | (b) | adopted |
| Q88 | visual-only and capacity-hint elements | A4 Q8 | refuse / accept with no effect | none (Jon's) | (a) | 01-decisions: accepted with no effect |
| Q89 | weld `site1`/`site2` | A4 Q9 | refuse / implement | refuse | (a) | stated limitation |
| Q90 | `<compiler><lengthrange>` | A4 §4; A11 LR-4 | refuse / implement | A4: refuse; A11: implement | **(c)** | C2 |
| Q91 | weld `torquescale` | A4 §4, §8.6; A13 §1.9; A18 §8.2 | refuse / implement | A4: refuse; A18: implement | **(c)** | C2 |
| Q92 | flex `elastic2d` | A4 §4; A9 F3 | refuse / implement `bend` | A4: refuse; A9: implement | **(c)** | C2 |
| Q93 | compiler `inertiagrouprange` | A4 §4; A10 Q2.3 | refuse / implement | A4: refuse; A10: implement | **(c)** | C2 |
| Q94 | `<equality><flex>` | A4 §4; A13 D3 | refuse / implement | A4: refuse; A13: implement | **(c)** | C2 |
| Q95 | `<inertial>` `axisangle xyaxes zaxis euler` | A4 §4; A20 §1A.3.9 | refuse / implement | A4: refuse; A20: implement (sim-urdf emits `euler`) | **(c)** | C2 |

## 8. MJCF validation

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q96 | principal axes: `from_matrix_unchecked` or eig3 | A5 Q1; A10 §2.4 | unchecked / bounded `from_matrix_eps` / MuJoCo's `eig3` | A5: unchecked; A10: eig3, "A5-Q1 is moot" | **(c)** | C6 |
| Q97 | automatic `lo == hi` | A5 Q2 | refuse / unlimited | refuse | (a) | stricter; 01-decisions "refuse an automatic backwards range" |
| Q98 | stored range of an unlimited joint | A5 Q3 | MuJoCo's / ours | MuJoCo's | (a) | parity |
| Q99 | explicit `lengthrange` | A5 Q4 | honour / refuse | honour | (a) | parity |
| Q100 | `actuatorfrcrange`/`actuatorfrclimited` | A5 Q5 | refuse / implement | refuse | (a) | stated limitation |
| Q101 | muscle `actlimited` default | A5 Q6 | parity for `<muscle>`; extensions keep (0,1) | that | (a) | parity |
| Q102 | URDF inertia triangle violations | A5 Q7 | refuse / `balanceinertia` | refuse, fix two example inertias | (a) | parity with MuJoCo's own URDF importer |
| Q103 | flat or degenerate mesh that needs a hull | A5 Q8; A10 §5, Q5.1 | refuse where MuJoCo needs a hull / lenient load | refuse; load the shell-without-collision case MuJoCo loads | **(c)** | 01-decisions: where no test can show our result right, "the item comes back to Jon" — A10 §5 found none; Jon decided refuse (2026-10-07) |
| Q104 | caller-built and `.mjb` input | A5 Q9 | builder guards + docs / exhaustive walk | guards + docs | (b) | adopted, 11-settled A5-Q9 |
| Q105 | repeated `<pair>` | A5 Q10 | keep both / last wins | keep both | (a) | parity |

## 9. Determinism

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q106 | `iter_over_hash_type` gate | A6 Q1 | cf-geometry + sim-mjcf / also sim-core / none | cf-geometry + sim-mjcf | (b) | adopted, 11-settled #14 |
| Q107 | flex element length ≠ dim+1 | A6 Q2 | refuse / fall back | refuse | (a) | parity (`user_mesh.cc:4108-4116`) |
| Q108 | mesh-io STEP iteration | A6 Q3 | sort / leave | A6: leave | (b) | 11-settled #14 chose "STEP keys sorted too" |
| Q109 | `Model`'s hash fields | A6 Q4 | leave / order | leave | (b) | adopted, 11-settled A6-Q4 |

## 10. Flex and composites

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q110 | `flexelem_volume0` after re-orientation | A9 Q1 | unsigned / delete the field / signed | unsigned (0 flips) | (b) | — |
| Q111 | cotangent bending-coefficient order | A9 Q2 | inherit MuJoCo's / keep ours | keep ours, listed as a correctness deviation | **(c)** | outside the four kinds ("refusing would refuse valid meshes") |
| Q112 | pins + bending | A9 Q3 | refuse (parity) / load | load | (a) | lenient with a test (A9 §2.6 #9); a new instance beyond the three 01-decisions names |
| Q113 | non-manifold edge + bending | A9 Q4 | MuJoCo's first+last pairing / refuse | refuse | (a) | stricter |
| Q114 | cable vertices through f32 | A9 Q5 | parity / f64 | parity | (a) | parity |
| Q115 | 1-body cables (frame and sites) | A9 §3.6, Q6 | ours (defined frame, two sites) / MuJoCo's (UB frame, one site) | ours | **(c)** | MuJoCo loads with a NaN frame, so the lenient kind does not fit and the stricter kind would refuse |
| Q116 | `bending_model` without `elastic2d` | A9 Q7 | refuse / imply bend / silent | refuse | (b) | — |
| Q117 | trailing whitespace in `curve` | A9 Q8 | accept / refuse | accept | (a) | lenient with a test; the parser owner should confirm against "keywords exact" |

## 11. Mesh and inertia

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q118 | ASCII STL; > 200,000 faces | A10 Q1.1 (a), (b) | lenient load / refuse | lenient | **(c)** | MuJoCo's refusal is its format design; no section shows a defect, so the lenient condition is not met as worded |
| Q119 | STL coordinate > 2^30 | A10 Q1.1 (c) | refuse | refuse | (a) | parity |
| Q120 | inherit `eig3`'s inaccuracy | A10 Q2.2 | parity / accurate solver + MuJoCo order | parity | (a) | 01-decisions: fix the principal-axis-order divergence |
| Q121 | embedded mesh vertices as f32 | A10 Q3.1 | parity / f64 | parity | (a) | parity |

## 12. Lengthrange

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q122 | the 4 docs MuJoCo refuses as "did not converge" | A11 Q-LR1 | refuse / load with (0,0) / an arbitrary value | refuse | **(c)** | 01-decisions listed this as lenient, but A11 §LR-3 shows no right value exists, so no test can be written — "the item comes back to Jon"; Jon decided refuse (2026-10-07) |
| Q123 | `uselimit`: raw or gear-scaled | A11 Q-LR2 | raw / scaled | raw | (a) | parity |
| Q124 | Hill/Millard in the mode filter | A11 Q-LR3 | MuJoCo's literal filter / include | literal filter | (b) | — |
| Q125 | a reset during the lengthrange run | A11 Q-LR4 | `Unstable` / MuJoCo's restart | `Unstable` ("not measured either way") | (a) | stricter |
| Q126 | lengthrange error in `MjcfError` | A11 Q-LR5 | `LengthRange { actuator, source, at }` / `InvalidModel` | `LengthRange` | (b) | — |
| Q127 | cf-design calls `set_length_range` | A11 Q-LR6 | yes / no | yes | (b) | — |
| Q128 | geometry-limited case MuJoCo refuses | A11 §LR-3b | parity, listed | parity | (a) | lenient has nothing to load (our port computes the same) |
| Q129 | ball/free joint transmission | A11 §6 item 1 | parity / stated-limitation refusal for non-scalar gear | none | **(c)** | no owner; LR-a creates a new ok→err unless it is fixed |

## 13. The census gate

| # | question | sections | options | recommendation | class | basis |
|---|---|---|---|---|---|---|
| Q130 | gate parameters | A20 "Open for Jon" 6 | 19 checkpoints (8.1 MB) or every step (28.3 MB); pin classes or labels; snapshot or extract | 19 / classes / append-only snapshot | (b) | — |

## Findings no section's commit list carried

Measured defects or gaps a section handed off and no section's commit list picked up. The carrier column names the commit of 20-rigid-physics or 22-rigid-loading that now takes each (a) and (c) row; (b) rows have none.

| # | finding | sections | class | note | carrier |
|---|---|---|---|---|---|
| U01 | a limit sensor on an unlimited joint loads; MuJoCo refuses (`mjcf_sensors.rs:221`) | A6 §5.1, §5.3 | (a) | NO-ROW refusal kind; parity | L21 |
| U02 | `step2` under an RK4 model ignores eulerdamp (qvel 2.6e-2 after 20 steps) | A7 §8 F2 | (a) | MuJoCo's `mj_step2` calls `mj_Euler` | P08a |
| U03 | collision calls the unbounded `UnitQuaternion::from_matrix` (`narrow.rs:175-176`, `mesh_collide.rs:144-145`, `hfield.rs:82,86,111`, `flex_narrow.rs:165,210,224`); a NaN `geom_xmat` "not traced" | A5 §4.6 | (b) | same mechanism as the H1 hang | — |
| U04 | muscle force clamps use 1e-10, MuJoCo `mjMINVAL` 1e-15 | A11 §6 item 2 | (a) | parity | P33 |
| U05 | `BiasType::Muscle` reads `gainprm`, MuJoCo `biasprm` | A11 §6 item 3 | (a) | parity | P33 |
| U06 | a flex doc plus any `gravcomp` body panics at `rne.rs:357` | A20 §1B.5 | (a) | MuJoCo loads it | L10a |
| U07 | an empty flex `element` loads a 1-vertex flex; MuJoCo refuses | A20 §1B.3 C7 | (a) | "no spec row" | L10 |
| U08 | `<deformable><flexcomp spacing="…">` parsed as one f64; `pos` not applied | A13 §5 | (b) | our extension (A4-Q3) | — |
| U09 | our flex grid generator's vertex order and diagonal differ from MuJoCo's | A13 §5 | (b) | matters for D3's fixture (A13 §3.7) | — |
| U10 | sim-urdf writes URDF `rpy` as intrinsic `xyz` | A20 §1A.3.9 | (a) | parity with MuJoCo's URDF reader; 3.4e-3 inertia residual "not isolated" | L43 |
| U11 | spatial tendon velocity over a sphere wrap reads 0 at the initial forward | A20 §1A.3.9 | (a) | not isolated | P29 (isolates it) |
| U12 | no dim-3 or 2-D membrane flex elasticity in sim-core | A9 §7 H6 | **(c)** | implement later or refuse as a limitation: scope | L32 |
| U13 | 17 OBJ files with a face count ≠ MuJoCo's; MuJoCo's OBJ graph ids disagree with its `mesh_vert` | A10 §4.1, §6 item 8 | (b) | not isolated | — |
| U14 | MuJoCo refuses < 4 vertices for every mesh; ours with faces + 3 vertices "not measured" | A10 §6 item 9 | (a) | parity | L38 |
| U15 | `<composite type="cable" count="1000">` reached ~2 GB RSS; cause not isolated | A4 §1.4; 41-what-planning | (b) | "whoever owns composites should look before Rigid ships" | — |
| U16 | `Data::reset` sets `stat_meaninertia = 0.0`, `make_data` 1.0 | A1 §4 | (b) | readers overwrite it first | — |
| U17 | box–box contact normals differ (`28695e44` step 56, `496f2186` step 33) | A18 §5.4, §8.1, §16 | (b) | whether A21 R6 (P46) fixes them is not stated | — |
