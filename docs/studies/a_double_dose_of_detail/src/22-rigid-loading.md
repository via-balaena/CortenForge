# Rigid-loading: the commit series

> **Conflicts resolved (2026-10-07).** The draft below lists conflicts C1–C11 between research sections without picking; these are the answers, and every inline mention of a C-number means this:
> - **C1 FMA → plain arithmetic** (Jon). No `mul_add` emulation; bit-exact tests compare against MuJoCo's C compiled without FMA (as A21 did); the census gate compares at 1e-9 and marks the few docs where a fused rounding flips a discrete choice.
> - **C2 → implement all six** (`<compiler><lengthrange>`, weld `torquescale`, `elastic2d`, `inertiagrouprange`, `<equality><flex>`, `<inertial>` orientation alternatives); A4 §4's refused list loses them.
> - **C3 → `Data::integrate` returns `Result`** (A8); this replaces 11-settled #7's "documented, unchanged".
> - **C4 → A19's fix for sleep under RK4** (MuJoCo computes a wrong value; Jon's wrong-value rule, with A19's three tests).
> - **C5 → implement a ball joint's `pos ≠ 0`** (A19).
> - **C6 → MuJoCo's `eig3`** (A10), in plain arithmetic (C1); A5-Q1 is moot.
> - **C7 → MuJoCo's flex meaning** (Jon): `<flex body=… vertex=…>` names existing bodies; in-tree flex docs are rewritten to declare their vertex bodies; our `<vertex>`/`<element>` child form is refused.
> - **C8 → the joint-layout check is fallible**, through `try_make_data` (`MakeDataError`).
> - **C9 → renormalise the stored contact normal as `mj_setContact` does** (parity).
> - **C10 → A21 R1's geom order is the global rule**; A17's port follows it.
> - **C11 → A21 R5 supersedes A15 F2** (as the draft already says).
>
> **Spec gaps (no research content yet — the stress test must press on these):** the `xfrc_applied` type change; sensor derivatives at the current state (from A2 §4/Q4); the sleep-policy compile fixes A8 assigns to the MJCF series. **Not measured:** no section ran these commits in this order; per-commit census counts are estimates from separate bases.


Draft for the stress test. Input: *A Double Dose of Detail* at `caa1c1c0`, Parts 0–3 and research A1–A21. Nothing here was built or run; every statement points at the section it rests on, and "not measured" / "not isolated" are kept where a section says so. Parts 0–3 win over the research where they disagree (00-how-to-read).

**What this PR holds** (01-decisions, second round): sim-mjcf errors, defaults, parser, validation, mesh/STL, inertia. It lands after Rigid-physics (`P01…P53`, 20-rigid-physics.md). Commits that must also change sim-core, cf-geometry, mesh-io or sim-urdf are marked **cross-layer** or **cross-crate**, with the section that forces it.

## Conventions

As in 20-rigid-physics.md: `L01…L50` is the order here; "src" is the book's or the section's id; titles carry no `!`; must-fail tests are the ones the research names; census counts are what a section measured on the base it names — **not measured as a series**. In addition:
- Every MJCF commit carries its MuJoCo 3.5.0 citation (the earlier single series) and is checked by the five buckets of A6 §4(b) (unchanged · ok→ok changed on the pre-registered list · ok→err mapped to this commit's rule · err→ok MuJoCo-ok · err→err message change). From L02 on, "NEW NONDETERMINISTIC" must stay empty (A6 §4(b), §6).
- Every commit re-blesses `verdicts.tsv`; a deliberate new refusal of a MuJoCo-loading doc is a hand-edited regression with its `divergence=<ID>` (A20 §2.4, §2.7). Commits that rewrite in-tree MJCF also append-refresh the gate's corpus snapshot (A20 §2.2, §2.7).
- Per-commit greenness in this order was not run by any section (A3 says each of its four is "green on its own" as a design statement; A4 §9 likewise; A5 §5; none ran a series).

## Summary

| # | src | title | breaking | depends on |
|---|---|---|---|---|
| L01 | M1 (A3 #1) | `feat(sim-mjcf): typed MjcfError with locations` | **yes** | — |
| L02 | M3 (A6) | `fix(sim-mjcf): flex edges in MuJoCo's order, hinges in edge order` | behaviour | L01 |
| L03 | M4 (A5 #1) + A10 MH2 | `fix(sim-mjcf): a non-finite inertia is an error, not a hang; principal axes as MuJoCo (mjuu_eig3)` | behaviour | L01 |
| L04 | M5 (A4 #1) | `fix(sim-mjcf): depth-safe traversals on a large stack` | behaviour | L01 |
| L05 | M6 (A4 #2) | `feat(sim-mjcf): one attribute reader with MuJoCo's rules` | **yes** | L01 |
| L06 | M7 (A5 #2) | `feat(sim-mjcf): every non-finite number is refused` | behaviour | L05, L03 |
| L07 | M8 (A4 #3) | `feat(sim-mjcf): keywords exact; composite curve keywords as MuJoCo` | **yes** (pub `from_str`) | L05 |
| L08 | A9 C1 | `fix(sim-mjcf): cable vertices pass through f32 as MuJoCo` | behaviour | L07 |
| L09 | M10 (A4 #5) | `fix(sim-mjcf): repeated <compiler>/<option> merge` | no | L05 |
| L10 | M14 part (A4 #6) | `feat(sim-mjcf): <flex vertex= element=> in MuJoCo's attribute form` | not stated | L05, L02 |
| L11 | M14 part | `feat(sim-mjcf): intvelocity actuator; user sensor dim` | **yes** (enum variant) | L05 |
| L12 | M14 part | `feat(sim-mjcf): connect body and site forms as MuJoCo` | **yes** (`MjcfConnect`) | L05 |
| L13 | M14 part | `fix(sim-mjcf): an unnamed mesh or hfield takes MuJoCo's name from its file` | behaviour | L05 |
| L14 | M14 part + A14 L39b + A20 §1A.3.6 | `fix(sim-mjcf): body xyaxes/zaxis, fromto size and frame, plane size as MuJoCo` | **yes** (`MjcfBody` fields) | L05 |
| L15 | M9 (A4 #4) | `fix(sim-mjcf): self-closing elements are not dropped` | no | L12, L10 |
| L16 | M11 (A3 #2) | `feat(sim-mjcf): defaults resolved in one pass before frames and composites` | **yes** (un-exports) | L15 |
| L17 | M12 (A3 #3) | `feat(sim-mjcf): explicit values survive their class` | **yes** (16 `Option` fields) | L05, L07, L16 |
| L18 | M13 (A3 #4) | `feat(sim-mjcf): default geom size, element-wise overlay, size must be positive` | **yes** (`size: Option`) | L07, L16 |
| L19 | M16 (A5 #3) | `feat(sim-mjcf): joint layout as MuJoCo` | behaviour | L16 |
| L20 | M17 (A5 #4) + A14/A19 | `feat(sim-mjcf): joint axis too small is an error; ball and free axes are (0,0,1)` | behaviour | L16 |
| L21 | M18 (A5 #5) | `feat(sim-mjcf): one limit helper for joints, tendons, actuators` | behaviour | L17 |
| L22 | M19 (A5 #6) | `feat(sim-mjcf): sizes, masses and inertias as MuJoCo; sim-urdf writes inertial pos` (cross-crate) | behaviour | L18 |
| L23 | M20 (A5 #7) | `feat(sim-mjcf): duplicate names are errors; sim-urdf world link` (cross-crate) | behaviour | L16 |
| L24 | M21 (A5 #8) | `fix(sim-mjcf): frame contents are validated` | behaviour | L16 |
| L25 | M22 part (A5 #9) | `fix(sim-mjcf): geom and pair condim checked as MuJoCo` | behaviour | L16 |
| L26 | M22 part | `fix(sim-mjcf): an empty <contact/> appends; repeated pairs are both kept` | behaviour | L16 |
| L27 | M22 part | `fix(sim-mjcf): .mjb decoding is size-limited` | behaviour | L01 |
| L28 | M24 (A5 #11) | `fix(sim-mjcf): timestep > 1 loads` | behaviour | L01 |
| L29 | A9 F1 | `fix(sim-mjcf): dim-3 flex elements oriented as MuJoCo` | behaviour | L02 |
| L30 | A9 F2 | `fix(sim-mjcf): flex flaps as MuJoCo — dim 2 only, boundary [opp, -1]` | behaviour | L02 |
| L31 | A13 D3 | `feat: flex edge constraints only through a flex equality, as MuJoCo` (cross-layer) | **yes** | L02, P11, L10 |
| L32 | A9 F3 | `feat(sim-mjcf): flex elastic2d — bending only when asked, as MuJoCo` | `.mjb` version | L30, L01, L05, L07 |
| L33 | A10 MH3 | `fix(sim-mjcf): body inertial frame as MuJoCo: one geom copied, orientation alternatives` | behaviour | L03, L22, L14 |
| L34 | A20 R3-L1 | `fix(sim-mjcf): plane and hfield geoms have MuJoCo's volume; zero-volume mass is zero` | behaviour | L33 |
| L35 | A10 MH4 | `fix(sim-mjcf): mesh mass properties as MuJoCo (legacy default, centroid apex, exact-mode orientation check)` | **yes** (`MeshInertia` default) | L03, L01 |
| L36 | A10 MH5 | `fix(sim-mjcf,mesh-io): STL vertices deduplicated as MuJoCo; non-finite vertices refused; left-handed scale; binary STL by size` (cross-crate) | behaviour | P03, L01 |
| L37 | A10 MH6 | `fix(sim-mjcf): a mesh that needs a hull and has none is an error` | behaviour | P03, L35 |
| L38 | M23 (A5 #10) | `feat(sim-mjcf): a vertex-only mesh gets its convex hull` | behaviour | P02, L37 |
| L39 | A20 R3-L2 | `fix(sim-mjcf): discardvisual keeps the discarded geoms' inertia; sim-urdf emits no visual geoms` (cross-crate) | behaviour | L16 |
| L40 | A20 R3-L3 | `fix(sim-mjcf): ellipsoid shell inertia as MuJoCo` | behaviour | — |
| L41 | A18 L1 + A20 R3-L4 | `fix(sim-mjcf): weld relpose as MuJoCo; parse torquescale` | not stated | L01, P39 |
| L42 | A20 R3-L5 | `fix(sim-mjcf): fusestatic accumulates body inertias, moves fromto geoms` | behaviour | L16, L03 |
| L43 | A20 R3-L6 | `fix(sim-mjcf): <inertial> orientation alternatives` | behaviour | L05 |
| L44 | A11 LR-a | `fix(sim-mjcf): actuator lengthrange computed and kept as MuJoCo 3.5.0` (cross-layer) | **yes** | P11, L01, P04 |
| L45 | A17 C6 | `fix(sim-mjcf): height-field data as MuJoCo (rows bottom-to-top, normalised, grid kept)` | behaviour | P52 |
| L46 | A8 SP-3/Q3 (unplaced) | `fix(sim-mjcf): sleep-policy compile errors and init-sleep refusal as MuJoCo` | behaviour | L01, P20, P23 |
| L47 | M15 (A4 #7) | `feat(sim-mjcf): schema pre-pass at MuJoCo 3.5.0 with the extension allowlist` | behaviour | L10–L14, L05, L07, L31, L32, L41 |
| L48 | M25 = A7 DH-3 | `feat(sim-mjcf): delay and history as MuJoCo 3.5.0` | **yes** (`MjcfSensor.interval`) | L01, L05, L16, L47, P08 |
| L49 | A11 LR-b | `feat(sim-mjcf): <compiler><lengthrange>` | additive field | L44, L47, L09, L05, L06 |
| L50 | M26 | `docs: MuJoCo divergences, sim-mjcf docs, conformance status` | no | all |

50 commits.

## What fixes the order

1. **L01 first:** "This is the PR's FIRST mjcf commit" (A3 §1); every later rule returns its variants (A4 §10, A5 §6, A10 §8, A11 §8). Everything frozen in this PR freezes together, so variants later commits need are added now (A3 §6, "Dependencies").
2. **L02 after L01** (A6 §7: M3's refusal needs the error type).
3. **L03 before L06** ("1 before 2 so commit 2's NaN tests never reach a hang", A5 §5); L03 = M4 + MH2 because "eig3 **is** M4's `principal_axes`" (A10 §7).
4. **L05 → L06, L07, L09, L10–L14** (A4 §9; A5 §6: the non-finite check sits in the reader's token function).
5. **L07 before L18**: `checksize` must not land before the curve keyword fix, or `composite.rs` t1 goes red (A3 §3, §6). **L07 → L08** (A9 §5).
6. **L12 before L15.** M9 without the connect-anchor refusal turns 5 both-refused docs into lenient loads; the gate caught it (A20 §2.10). L12 holds that refusal (A4 §9 commit 6 "connect form rule").
7. **L15 before L16**: the self-closing `<default class/>` arm must be in before S10 refuses undefined classes (A4 §8.2, §10).
8. **L16 → L17 → L21; L16 → L18 → L22; L16 → L19, L20, L23–L26** (the earlier single series M12/M13/M16–M22; A5 §5: S7 needs S3; A5 §2: S12 after expansion and before discardvisual/fusestatic).
9. **L22 before L33** (A10 §7: MH3 after M19, else `fluid_derivatives.rs:1391` and `fluid_forces.rs:119,185` go NaN instead of refusing). **L14 before L33** (the fromto axis moves into L14 — see "Merged"; A10 §2.3 measured that the one-geom copy without the fromto change is worse than main).
10. **L02 → L29, L30, L31** (A9 §5; A13 §4). **L30 → L32**; **L31 and L32 before L47** — otherwise the schema pre-pass refuses `<edge equality>`, `<equality><flex>` and `elastic2d` as unimplemented (A9 §5, A13 §4).
11. **P03 (MH1) → L36, L37; L35 → L37; L37 → L38** (A10 §7–§8: hull built once on deduplicated points; M23 after MH6).
12. **L44 needs P11, L01, P04** (A11 §8); it is one commit because "they cannot be split green" (A11 §8).
13. **L45 after P52** (A17 §12).
14. **L47 after L10–L14** ("6 before 7 so the docs can be rewritten to MuJoCo forms before the pre-pass refuses our spellings; 2–3 before 7", A4 §9).
15. **L48 after L01, L05, L16, L47** (A7 §6) and after P08 (A13 §4).
16. **L49 after L44, L47, L09, L05/L06** (A11 §8).

## Commits

### L01 · M1 · `feat(sim-mjcf): typed MjcfError with locations`
- **Closes:** mjcf-E1, mjcf-E2.
- **Implements:** A3 §1's enum (`#[non_exhaustive]`, struct-shaped variants, `String` names, opaque `Location`, `Io { path, source }`, `Unsupported` only for stated limitations; plus `#[cfg(feature = "mjb")] MjbTooLarge`) (11-settled #2); A3 §2 message-only locations (11-settled E2); the variant mapping of 30-error-type (A4 §10 and A5 §6 names). A11 Q-LR5's `LengthRange` variant, if adopted, belongs here (A3 §6: add variants now).
- **Must-fail:** A3 §1 tests 1–4 (duplicate body → `DuplicateName`; missing file → `Io` with path — "by reading, not run"; `<composite type="grid">` → `InvalidValue`; unknown tendon joint → `UndefinedReference`) and A3 §2's two location tests.
- **Flips:** `lib.rs:293-310`; variant renames in `validation.rs:747-1071`, `lib.rs:281`, `parser/tests.rs:194,1360`, `mjb.rs` (needs a local `--features mjb` run), `include.rs:427,449,471`, `error.rs:310-364`, `fluid_forces.rs:1197,1217`; the doc comment `cf-design/src/mechanism/builder.rs:246`. With the 81 asserted substrings kept, only `lib.rs:308` flips on text (A3 §1).
- **Census:** err→err message changes wholesale (A6 §4(b)); the gate records status only (A20 §2.1).
- **Breaking:** `model_from_mjcf` returns `Result<Model, MjcfError>`; `ModelConversionError` deleted (A3 §1).

### L02 · M3 · `fix(sim-mjcf): flex edges in MuJoCo's order, hinges in edge order`
- **Closes:** P-L27, sim-mjcf half.
- **Implements:** A6 §1.3 (first-appearance edge order, `local_edges`, hinges in edge order); `#![deny(clippy::iter_over_hash_type)]` at `sim/L0/mjcf/src/lib.rs:149` (11-settled #14); the refusal of `dim ∉ 1..=3` or element length ≠ dim+1 (A6 Q2 → 11-settled A6-Q2).
- **Must-fail:** `flex_edges_follow_mujoco_order`, `flex_build_is_repeatable` (A6 §1.6).
- **Flips:** none measured (A6 §1.7).
- **Census:** flex is invisible to it (A16 §5). Harness ×10 + submodules ×3: 0 nondeterministic docs with both hash fixes (A6 §6).

### L03 · M4 + A10 MH2 · `fix(sim-mjcf): a non-finite inertia is an error, not a hang; principal axes as MuJoCo (mjuu_eig3)`
- **Merged:** A5 commit 1 and A10 MH2 — "eig3 **is** M4's `principal_axes`, so merge MH2 into M4" (A10 §7). **Conflicts with 11-settled A5-Q1** (C6).
- **Closes:** mjcf-H1 (hang); the known divergence "principal-axis order of a full inertia" (01-decisions).
- **Implements:** A5 §3.1 (`Result` through `resolve_body_inertia`; refuse a non-finite tensor, eigen-decomposition or mass) with A10 §2.4's `builder/eig3.rs` (`eig3`, `full_inertia`, `global_inertia`, `offcenter`); the non-finite guard sits on eig3's output (A10 §2.4). Fused multiply-adds as the reference build (A10 Q2.1) — C1.
- **Must-fail:** `hang_is_an_error_typed_model` (make it fail once on main only under `timeout`), `mass_overflow_is_an_error` (A5 §3.1); `eig3_matches_mujoco_bitwise` (A10 §2.4 #1). (`from_matrix_unchecked_matches` only if A5-Q1 stands.)
- **Flips:** `mesh_inertia_modes.rs:218` t9 → MuJoCo's decreasing order (A10 §2.3, §7).
- **Census:** NEW-FRAMEQUAT `f0b36671` agrees once MH2 lands (A19 §4.1). Corpus, MH2–MH5 together: 271 docs `model_fp`, 66 trajectory (A10 §7).

### L04 · M5 · `fix(sim-mjcf): depth-safe traversals on a large stack`
- **Closes:** mjcf-H2 part 1; P-L35 (`all_bodies`).
- **Implements:** A4 §1.4: `on_large_stack` (64 MiB) around the six recursive pub entry points; iterative `body()`/`joint()`; the `all_bodies` fix; the 4096 tree-depth guard, checked at the entry of `model_from_mjcf`, `validate`, `validate_tendons` and after `expand_composites` (A4 §1.4; 11-settled #16). A5 §3.8 notes that decoding a deeply nested `.mjb` already recurses in serde before `model_from_mjcf`; whether this commit covers that path is not stated.
- **Must-fail:** `load_at_depth_limit_from_small_thread` (its own test file), `caller_built_tree_depth_guard` ("by the slopes; not run"), `all_bodies_returns_every_level` (A4 §1.5).
- **Note:** drop recursion is a stated residual; measure drop alone, iterative `Drop` if > 512 KB (A4-Q5 → 11-settled).

### L05 · M6 · `feat(sim-mjcf): one attribute reader with MuJoCo's rules`
- **Closes:** mjcf-S2 (numbers, lengths, required attributes), P-L28 single-value friction, mjcf-S8 (zero quaternion, multiple orientations), P-L35 (zero-quaternion hang) (the earlier single series M6).
- **Implements:** A4 §2.3 (`Attrs`; `Prefix<N>` and `TfAuto` pub types; exact lengths; trim; underflow; empty strings; entity unescape; the required attributes of §2.4 except sensor targets and user `dim`; `quat`; multiple-orientation refusal). Merging moves to `over`/`over_prefix` in the defaults file (A4 §2.3). This fixes A13 §1.1's one-value `solref` and A20's partial `polycoef` (A20 §1A.2).
- **Must-fail:** A4 §2.5's number/length cases (`timestep="0.01s"`, `damping="abc"`, `mass="2kg"`, `mass="2,5"`, `mass=" 2 "`, `condim="3.0"`, `group="99999999999"`, `pos="0 0 0 1"`, `friction="1 2 3 4"`, `gear` ×7, `mass="1e-999"`, `name="a&amp;b"`, `friction="0.7"`, `solref="0.05"`, `fluidcoef="0.5 0.25"`); A4 §8.4's zero quaternion ("must not be run against main (hangs)") and multiple orientation.
- **Flips:** 19 five-value `friction` rewrites (class E); `inertial` without `pos` in validator `urdf-loading/stress-test`; the multiple-orientation site `builder/body.rs:702`; `fluid_forces.rs:1205-1220` T39 err→ok (A4 §7, §9).
- **Census:** `5eb35113` (partial `polycoef`) flips (A20 §1A.2).
- **Breaking:** pub field types become `Prefix<N>`/`TfAuto`; new pub types (A4 §2.3).

### L06 · M7 · `feat(sim-mjcf): every non-finite number is refused`
- **Closes:** mjcf-S9 (NaN body pos), mjcf-H1 (decision: refuse every NaN/±inf, 01-decisions consequences).
- **Implements:** A5 §3.1 target 1 (`parse_f64` in the reader's token function; message contains "finite").
- **Must-fail:** `nan_rejected_everywhere` (A5 §3.1).
- **Flips:** none, if the message contains "finite" (keeps `keyframes.rs` ac22/ac23) (A5 §3.1).
- **Note:** caller-built and `.mjb` input see no parse; builder guards plus docs (A5-Q9 → 11-settled).

### L07 · M8 · `feat(sim-mjcf): keywords exact; composite curve keywords as MuJoCo`
- **Closes:** mjcf-S2 (keywords); the `curve` keyword map (10-scope "also fixed").
- **Implements:** A4 §2.3 keyword tables (`keywords.rs`; allowlisted extension keywords; dead joint/geom aliases removed; pub `from_str` exact; `TfAuto` for `*limited`) and A9 §4.2's rewrites, including the template arguments at `composites/stress-test/src/main.rs:69,86,114,245` and `:282` (`"s c 0"` → `"sin(s) cos(s) 0"`), which the corpus extractor cannot see.
- **Must-fail:** A4 §2.5's keyword cases (`integrator="RK45"`, `"euler"`, `solver="newton"`, `angle="RADIAN"`, `<flag gravity="off"/>`/`"true"`, `limited="1"`, `limited="auto"`, `type=" box"`); `cable_curve_keywords_as_mujoco` (A9 §4.4).
- **Flips:** class D rewrites, 19 docs (A4 §7); with the rewrites 1 doc changes (`composite.rs:25`, passes) and the composite validator's stdout is byte-identical; without them 9 integration and 2 lib tests flip (A9 §4.5).
- **Census:** with L08, `263f733b` agrees (A20 §1A.2); A9's 17 curve docs move `mj-refuses` → `both-refuse` (A20 §2.10).
- **Open:** trailing whitespace in `curve` (A9 Q8) vs "keywords exact".

### L08 · A9 C1 · `fix(sim-mjcf): cable vertices pass through f32 as MuJoCo`
- **Implements:** A9 §4.2 C1; the comment fix `builder/composite.rs:390-393`; pins for the 2-body cable (lenient, named by 01-decisions) and the 1-body cable (A9 §3.5, §3.6).
- **Must-fail:** `cable_vertices_round_through_f32` (A9 §4.4); pins `two_body_cable_matches_mujoco_generator` and `one_body_cable_frame_is_defined` pass on main (A9 §3.5–§3.6).
- **Flips:** 10 docs model + trajectory; 0 tests (A9 §4.5).

### L09 · M10 · `fix(sim-mjcf): repeated <compiler>/<option> merge`
- **Closes:** mjcf-S5. **Implements:** A4 §8.3. **Must-fail:** the two measured cases. **Flips:** none (A4 §8.3).

### L10 · M14 part · `feat(sim-mjcf): <flex vertex= element=> in MuJoCo's attribute form`
- **Split from M14** (A4 §9 commit 6 bundles six unrelated forms; each touches different elements, and this one rewrites the 81 flex docs that L31 and L32 also rewrite — A9 §5, A13 §4).
- **Closes:** A4-Q1 (11-settled); A6 §1.10 / A20 §1B.3 C4 (`vertex=`/`element=` ignored today). **Conflicts with A20 §1B.4** (C7: MuJoCo's `body` names existing bodies as vertices; ours creates them).
- **Implements:** the attribute form and the class A rewrites (74 docs + 7 templates); the child form is refused by L47 (A4 §3).
- **Must-fail:** none named beyond A4 §9's "measured cases"; L15's self-closing flex test needs this form (A4 §8.2).
- **Census:** flex stays outside the census; at most 72 docs become comparable, and only with explicit vertex bodies that "no appendix specifies" (A20 §1B.2–§1B.4).

### L11 · M14 part · `feat(sim-mjcf): intvelocity actuator; user sensor dim`
- **Closes:** mjcf-S1 (`intvelocity`). **Implements:** A4 §5 (`mjs_setToIntVelocity`), A4 §4 (user `dim="1"` only).
- **Must-fail:** `intvelocity` model fields equal the measured 3.5.0 values (A4 §5).
- **Flips:** `callbacks.rs:199-212` gains `dim="1"`; `parser/tests.rs:2283` (`dim="3"`) becomes a refusal test (A4 §9).
- **Breaking:** `MjcfActuatorType::IntVelocity` (A4 §5).

### L12 · M14 part · `feat(sim-mjcf): connect body and site forms as MuJoCo`
- **Closes:** P-L28 (connect `site1`/`site2`). **Implements:** A4 §8.6 (site form lowered to bodies; mixing refused; body form needs `body1` and `anchor`).
- **Must-fail:** A4 §8.6's measured cases.
- **Flips:** class G connect: `equality_constraints.rs` ×2 plus the six parse-only connect tests (A4 §9).
- **Order:** before L15 (A20 §2.10). **Breaking:** `MjcfConnect` fields (A4 §8.6).

### L13 · M14 part · `fix(sim-mjcf): an unnamed mesh or hfield takes MuJoCo's name from its file`
- **Closes:** mjcf-S15. **Implements:** A4 §8.6 (file stem; "empty name in mesh" when neither). **Must-fail:** A4 §8.6's measured case.

### L14 · M14 part + A14 L39b + A20 §1A.3.6 · `fix(sim-mjcf): body xyaxes/zaxis, fromto size and frame, plane size as MuJoCo`
- **Merged:** A14 L39b belongs in M14 "so that a box's size and frame match MuJoCo together" (A14 §2.5); A10 §2.4's fromto change is the same code and moves here; A20 §1A.3.6 puts the rotated-frame fromto "into M14".
- **Closes:** mjcf-S8 (`xyaxes`/`zaxis`, box/ellipsoid `fromto`, plane size); ledger-L39b = census L45b `fromto`, 166 docs (A16 §2).
- **Implements:** A4 §8.4; `compute_fromto_pose(fromto, size, geom_type)` with `mjuu_z2quat(from − to)` (A14 §2.3); fromto resolved in a rotated frame's coordinates, after defaults (A20 §1A.3.6).
- **Must-fail:** A4 §8.4's cases; A14 §2.6 (capsule `geom_quat` literals; box frame with sizes; `framequat objtype="geom"` golden); `fromto_axis_is_from_minus_to` (A10 §2.4 #4); the `4eed75d1` literal (A20 §1A.3.6).
- **Flips:** plane `geom_size` changes in 282 docs ("whether any test asserts a plane's `geom_size` was not checked", A4 §8.4); `geom_quat` in 200 docs, trajectories in 21 (20 at ≤ 8.4e-14; the chaotic `equality-constraints/stress-test:388` validator still passes, two printed lines change) (A14 §2.4).
- **Census:** `fromto` 156 of 166 flip alone; the rotated-frame pair 2/2 (A20 §1A.2).
- **Breaking:** new pub `MjcfBody` fields (A4 §8.4).

### L15 · M9 · `fix(sim-mjcf): self-closing elements are not dropped`
- **Closes:** mjcf-S4. **Implements:** A4 §8.2 (`Start | Empty` with `empty: bool`).
- **Must-fail:** self-closing mocap body (main: nbody 1, nmocap 0); `<default><default class="e"/></default>` resolves; self-closing flex builds (needs L10) (A4 §8.2).
- **Flips:** `parser/tests.rs:1256` and seven other self-closing-body docs, build-err → build-ok or the connect error (A4 §8.2).
- **Census:** `94e0077d` flips (A20 §1A.2); 5 both-refused docs would become lenient loads without L12 first (A20 §2.10).

### L16 · M11 · `feat(sim-mjcf): defaults resolved in one pass before frames and composites`
- **Closes:** mjcf-S10, P-L26, and 10-scope's "defaults ignore `<frame>` composition; discardvisual/fusestatic read unresolved classes; composite elements take the user's defaults" (A3 §4).
- **Implements:** A3 §3 (ordered snapshot `DefaultResolver`, crate-private: 11-settled A3-O2, O3), A3 §4 option (b) (`resolve::apply_defaults`, written iteratively; `validate_resolved`; `validate_tendons` folded in and un-exported). d10 is `RepeatedElement` from the schema pass, or a seen-set here if S1 lacks multiplicity (A3 §3).
- **Must-fail:** d01–d09, d12, d15, d11, both L26 shapes, f01, f02, k01, k02, c01, c02 (A3 §6).
- **Flips:** `default_classes.rs:276`; eight `defaults.rs` unit tests; AC33/AC35 keep passing if `UnknownClass`'s Display carries the name (A3 §3).
- **Census:** corpus model changes from the reordering "are 0 by these scans (not measured by running a branch)" (A3 §4).
- **Note:** the L26 guarantee is structural — no loop revisits an entry; a timeout does not bound RAM (A3 §3).

### L17 · M12 · `feat(sim-mjcf): explicit values survive their class`
- **Closes:** mjcf-S11.
- **Implements:** A3 §3 (16 `Option` fields; sensor defaults stay allowlisted, 11-settled #4) and A3-O4 (negative muscle parameters refused except the `force` rule; 11-settled A3-O4). **Not placed by research:** A3-O1 (refuse cross-shortcut actuator defaults; A3 says "decide this before commit 3") and A3-O5 (cylinder `bias[1..3]` refused; "belongs to whichever area owns actuator attributes", A3 §7) — both 11-settled; this commit owns the actuator default fields, so it is the candidate.
- **Must-fail:** s11_gear, s11_kp, s11_area, s11_adhesion_gain, s11_muscle2, s11_sensor_noise (A3 §3).
- **Flips:** compile-level in `defaults.rs:1273-1298,1407-1446` and `parser/tests.rs` (A3 §3); for A3-O5, the example `actuators/cylinder/src/main.rs:46` needs rewriting (A20 §1A.2).
- **Census:** cylinder `bias` 2 docs become refusals (A20 §1A.2).

### L18 · M13 · `feat(sim-mjcf): default geom size, element-wise overlay, size must be positive`
- **Closes:** mjcf-S3. **Implements:** A3 §3 (`size: Option<Vec<f64>>`, `overlay`, default 0, `checksize`).
- **Must-fail:** s3_01, s3_02, ov_class_size_chain, s3_08, s3_03, s3_05, s3_12 (A3 §3).
- **Flips:** compile-level only; 0 behavioural flips in the static corpus (A3 §3).

### L19 · M16 · `feat(sim-mjcf): joint layout as MuJoCo`
- **Closes:** mjcf-H3; ball-then-slide refused (01-decisions consequences). **Implements:** A5 §3.3 (`check_joint_layout` before `build()`).
- **Must-fail:** the 12 `Err` cases and 5 `Ok` cases of A5 §3.3 (main: 8 panic, 4 load).
- **Flips:** none (A5 §3.3). The sim-core side is C8.

### L20 · M17 + A14 + A19 · `feat(sim-mjcf): joint axis too small is an error; ball and free axes are (0,0,1)`
- **Merged:** A14 §5's M17 addition and A19 §6's builder row — force ball/free `jnt_axis = (0,0,1)` before the "axis too small" check.
- **Closes:** mjcf-S9 (zero axis → Z).
- **Must-fail:** the 7 `Err` cases; `axis_ball_zero`, `axis_free_zero` (A5 §3.2); X7 `jnt_axis == (0,0,1)` and `<joint type="ball" axis="0 0 0"/>` loads (A14 §3.3).
- **Flips:** none (A5 §3.2).

### L21 · M18 · `feat(sim-mjcf): one limit helper for joints, tendons, actuators`
- **Closes:** mjcf-H4, mjcf-S6, P-L1.
- **Implements:** A5 §3.4 with 11-settled A5-Q2 (`lo == hi` refused), A5-Q3 (MuJoCo's stored range), A5-Q6 (MuJoCo `<muscle>` gets no forced limit; Hill/Millard keep (0,1)).
- **Must-fail:** A5 §3.4's list.
- **Flips:** `ball_joint_limits.rs:485`, `:1213`; validator `joint-limits/stress-test/src/main.rs:175`; `builder/joint.rs:661-683`; `phase7_spec_a.rs:465`, `:496`; `builder/compiler.rs:516` (A5 §3.4).
- **Census:** `bc9d11e0` (free joint `limited`) flips; muscle `actlimited` is one of three causes of the muscle cluster (A20 §1A.2).

### L22 · M19 · `feat(sim-mjcf): sizes, masses and inertias as MuJoCo; sim-urdf writes inertial pos` — cross-crate (sim-urdf)
- **Closes:** mjcf-S7; 10-scope's "sim-urdf emits `<inertial>` without `pos` (28 of 37 in-tree URDFs)".
- **Implements:** A5 §3.5 (the mass pipeline in MuJoCo's order, before flex bodies; `mass < 0` instead of `≤ 0`) and A5-Q7 (URDF triangle violations refused; fix the two example inertias; 11-settled).
- **Must-fail:** A5 §3.5's named cases (`sphere_size_neg` … `hfield z=0`) and its `Ok` cases.
- **Flips:** `r4_micro_spike.rs:30`, `spatial_tendons.rs` `MODEL_C`/`D`/`I`, `fluid_forces.rs` `MODEL_O`/`Z`, `builder/compiler.rs:611`, `builder/mass.rs:328-345`, `sensor_phase6.rs:1543-1547`, `fluid_derivatives.rs:1391-1393`, `builder/mod.rs:1247-1250`, `raycast_heightfield.rs:14`; validators `free-joint/stress-test:29`, `urdf-loading/stress-test:120` and `:283` (A5 §3.5).
- **Census:** moving-body mass, 10 docs ok→err (A6 §5.1).

### L23 · M20 · `feat(sim-mjcf): duplicate names are errors; sim-urdf world link` — cross-crate (sim-urdf)
- **Closes:** mjcf-S12; 10-scope's sim-urdf `world` link.
- **Implements:** A5 §3.6 (`check_unique_names` after expansion; converter maps a `world` link onto `<worldbody>` and dedupes geom names).
- **Must-fail:** the `dup_*` cases; `dup_unnamed_geoms_ok`, `dup_sensor_actuator_cross`, `dup_name_cross_kind`; the converter tests (A5 §3.6).
- **Flips:** `lib.rs:293-310`, `validation.rs:741-761` (shared with L01); URDF validator cases `:208`, `:305` flip unless the converter is fixed in this commit (A5 §3.6).

### L24 · M21 · `fix(sim-mjcf): frame contents are validated`
- **Closes:** mjcf-S13; the limitation wording for joints/inertial in `<frame>` (11-settled #5).
- **Must-fail:** the 7 frame cases (A5 §3.7). **Flips:** `frame.rs:673/697/716` stay green with "not allowed inside <frame>" kept.

### L25 · M22 part · `fix(sim-mjcf): geom and pair condim checked as MuJoCo`
- **Split from M22** (A5 commit 9 bundles condim checks, `<contact/>` parsing, `.mjb` decoding and pair dedupe: four rules in four files, and only the `.mjb` one needs `--features mjb`, A6 §4(e)).
- **Closes:** mjcf-D2 (condim). **Implements:** A5 §3.8 (`validate_condim` returns `Result`; pair condim in `process_contact`).
- **Must-fail:** none named; measured cases condim 0, 2 (default class), 5, 7. **Flips:** none (no test pins the rounding) (A5 §3.8).

### L26 · M22 part · `fix(sim-mjcf): an empty <contact/> appends; repeated pairs are both kept`
- **Closes:** mjcf-D2 (empty `<contact/>`); A5-Q10 (11-settled).
- **Must-fail:** none named; measured: pair then `<contact/>` → npair 1, two blocks → 2, repeated pairs npair 2 (A5 §3.8).
- **Flips:** `builder/contact.rs:261` `pair_dedup_last_wins_under_canonical_key` (11-settled A5-Q10). **Census:** the A5-Q10 doc (A16 §2).

### L27 · M22 part · `fix(sim-mjcf): .mjb decoding is size-limited`
- **Closes:** mjcf-D2 (`.mjb` size); 11-settled #13 (`MjbTooLarge`).
- **Implements:** A5 §3.8 (`MJB_DECODE_LIMIT = 1 << 28`, `with_limit`) — "Not measured (no crafted file was decoded)".
- **Must-fail:** none named. Run locally with `--features mjb` (A6 §4(e)).

### L28 · M24 · `fix(sim-mjcf): timestep > 1 loads`
- **Closes:** P-L24, sim-mjcf half (drop the `> 1` rule; keep ≤ 0 / non-finite as stricter).
- **Must-fail:** `timestep="1.5"` loads (A5 §3.9). **Flips:** `validation.rs:862-867`.

### L29 · A9 F1 · `fix(sim-mjcf): dim-3 flex elements oriented as MuJoCo`
- **Closes:** the known divergence "dim-3 tet re-orientation" (01-decisions).
- **Implements:** A9 §1.3 (`orient_tetrahedra`) and A9 Q1's unsigned `flexelem_volume0` (open-questions).
- **Must-fail:** `flex_tet_reoriented_as_mujoco`, `flex_tets_all_outward_after_build`, `flexelem_volume0_is_unsigned` (A9 §1.5).
- **Flips:** `flex_unified::ac3_solid_compression` unless the volume is unsigned (then 0) (A9 §1.6).

### L30 · A9 F2 · `fix(sim-mjcf): flex flaps as MuJoCo — dim 2 only, boundary [opp, -1]`
- **Closes:** the known divergence "flex boundary flaps" (01-decisions).
- **Must-fail:** A9 §2.6 #1 and #3. **Flips:** 46 docs model-only, 0 trajectory, 0 tests (A9 §2.7).

### L31 · A13 D3 · `feat: flex edge constraints only through a flex equality, as MuJoCo` — cross-layer (sim-core + sim-mjcf)
- **In Rigid-loading although it changes sim-core** because it needs M3's edge order (L02) and the MJCF parse, and must precede M15 (A13 §4).
- **Closes:** ledger-L39c (A13 §3) = census ledger-L44c (A16 §3).
- **Implements:** A13 §3.5: `EqualityType::Flex`; `flexedge_invweight0` derived in `recompute_derived` (P11); `flex_edgeequality`; `flex_edge_solref`/`solimp` removed; parse `<equality><flex>` and `<flexcomp><edge equality solref solimp>`; refuse `equality="vert"` and `<flexvert>` (stated limitation, A13 Q5); `ConstraintType::FlexEdge` per A13 Q3; doc rewrites per A13 Q4. **Conflicts with A4 §4** (C2: `<equality><flex>` on the refused list).
- **Must-fail:** `flex_rows_only_with_edge_equality`, `flex_edge_rows_match_mujoco_3_5_0` (A13 §3.7).
- **Flips:** without the rewrites, `flex_unified::ac5_edge_constraint_stiffness`, `ac20_bending_stability_clamp`, `flex_flex_collision::t09_full_forward_step_no_panic`; with every flex treated as requesting edges, 0 failures (A13 §3.6).
- **Census:** 0 — no flex doc is loaded by both engines (A16 §3). "Item 3's MJCF side … was not prototyped" (A13 §7).
- **Breaking:** `EqualityType` is not `#[non_exhaustive]`; Model fields removed (A13 §3.5).

### L32 · A9 F3 · `feat(sim-mjcf): flex elastic2d — bending only when asked, as MuJoCo`
- **Closes:** A9 item 2b; DT-86 rows. **Conflicts with A4 §4** (C2: `elastic2d` on the refused list).
- **Implements:** A9 §2.4 2b (`FlexElastic2d`; bending only for `bend`/`both`; `stretch`/`both` refused as a limitation; refusals per A9 Q4, Q7; pins + bending load per A9 Q3) and the 39 `elastic2d="bend"` rewrites.
- **Must-fail:** A9 §2.6 #2, #4, #5, #6, #7; pins #8 (bending-order deviation) and #9 (pins + bending).
- **Flips:** with the rewrites, 2 docs model-only, 0 tests; without them, 12 CI tests flip (A9 §2.7).
- **Note:** `.mjb` version bump (A9 §2.8); L10 rewrites the same docs, "whichever lands second rebases the other's rewrites" (A9 §5).

### L33 · A10 MH3 · `fix(sim-mjcf): body inertial frame as MuJoCo: one geom copied, orientation alternatives`
- **Closes:** ledger-L43c, census "rotated inertia" (A16 §2; `636fc5d5`, A20 §1A.2).
- **Implements:** A10 §2.4 (MuJoCo's geom selection, one-geom copy, orientation alternatives through `resolve_orientation`, mesh principal frame from unit-density eig3); the fromto part is in L14. `inertiagrouprange` implemented here per A10 Q2.3 — **conflicts with A4 §4** (C2).
- **Must-fail:** `single_geom_body_copies_geom_frame`, `geom_euler_rotates_body_inertia` (A10 §2.4).
- **Flips:** `fluid_derivatives.rs:1391` t23 only if before L22 (A10 §7); here it is after.
- **Census:** `636fc5d5` flips (A20 §1A.2); outlier `collision_primitives.rs:1279` moves 0.189 (A10 §2.3).

### L34 · A20 R3-L1 · `fix(sim-mjcf): plane and hfield geoms have MuJoCo's volume; zero-volume mass is zero`
- **Must-fail:** `plane_geom_has_no_mass` (A20 §1A.3.1). **Flips:** 8 docs `model_fp`, 0 trajectory.
- **Census:** plane-only body mass 6/6 (A20 §1A.2). The massless body's inertial frame is open (A20 "Open for Jon" 1).

### L35 · A10 MH4 · `fix(sim-mjcf): mesh mass properties as MuJoCo (legacy default, centroid apex, exact-mode orientation check)`
- **Implements:** A10 §3 (`mesh_mass_properties`; `MeshInertia` default `Legacy`; exact-mode orientation check). Mesh re-centring is **not** in it (A20 "Open for Jon" 4; A17 Q2).
- **Must-fail:** `t10` → MuJoCo's values; `t1` renamed `t1_default_mode_is_legacy` plus an L-shape with no attribute (mass 5322.55); `exact_mode_refuses_inconsistent_orientation` ("not run"); the L-prism regression (A10 §3).
- **Flips:** `mesh_inertia_modes.rs:241` t10, `:52` t1 (A10 §7).

### L36 · A10 MH5 · `fix(sim-mjcf,mesh-io): STL vertices deduplicated as MuJoCo; non-finite vertices refused; left-handed scale; binary STL by size` — cross-crate (mesh-io)
- **Closes:** the known divergence "STL vertex deduplication" (01-decisions); the NaN-STL hang (A10 §1).
- **Implements:** A10 §1 (dedupe in sim-mjcf for `.stl` before scale; non-finite vertices refused for every format; left-handed winding swap; `mesh-io` binary detection by size). ASCII STL, > 200,000 faces and |coord| > 2^30 are open (A10 Q1.1).
- **Must-fail:** `stl_vertices_deduplicated_in_first_appearance_order`, `stl_negative_zero_merges`, `stl_left_handed_scale_flips_winding`, `stl_nonfinite_vertex_is_error` (main hangs: make it fail once only under `timeout`), `binary_stl_with_solid_header_loads` (A10 §1).
- **Flips:** none measured; 53 submodule STL files change from error to load (model-level status of their 5 models "not measured") (A10 §1).

### L37 · A10 MH6 · `fix(sim-mjcf): a mesh that needs a hull and has none is an error`
- **Closes:** the flat-mesh item of the lenient decision — but A10 §5 finds **no test can show our collision on a flat mesh right**, so under 01-decisions it "comes back to Jon" (A10 Q5.1; open-questions). This commit is the parity option A10 recommends.
- **Must-fail:** `flat_mesh_with_collision_is_refused`, `flat_shell_mesh_without_collision_loads`, `flat_mesh_legacy_volume_too_small` (A10 §5).
- **Flips:** `mesh_inertia_modes.rs:339` t16 (rewrite with `contype="0" conaffinity="0"`); `exactmeshinertia.rs:354` ac7 (rewrite as a refusal) (A10 §5).
- **Note:** re-measure the verdict on top of P40 — A15 found its EPA defect makes box, cylinder and ellipsoid fall through convex mesh slabs, and whether it also caused the planar-hull fall-through "was not measured" (A15 §4.4, §8).

### L38 · M23 · `feat(sim-mjcf): a vertex-only mesh gets its convex hull`
- **Closes:** mjcf-D1 (11-settled #13).
- **Must-fail:** a 6-vertex hull loads with nonzero mass; 3 vertices and 4 coplanar vertices → `Err` (A5 §3.8).
- **Flips:** none (the 4 corpus docs are parse-only tests) (A5 §3.8). **Census:** the 4 vertex-only `ours-refused` docs (A20 §2.6).

### L39 · A20 R3-L2 · `fix(sim-mjcf): discardvisual keeps the discarded geoms' inertia; sim-urdf emits no visual geoms` — cross-crate (sim-urdf)
- **Implements:** A20 §1A.3.2; the converter change goes in the same commit, or URDF links without `<inertial>` gain their visual mass (16.52 kg vs 0.5236 kg measured).
- **Must-fail:** the `5f4d4961` and `b6ad1797` literals (A20 §1A.3.2). Check the `urdf-loading/*` validators.
- **Census:** 2/2 (A20 §1A.2).

### L40 · A20 R3-L3 · `fix(sim-mjcf): ellipsoid shell inertia as MuJoCo`
- **Must-fail:** tighten `mesh_inertia_modes.rs:195` t8 to 1e-9 relative (fails on main by 1.1 %) (A20 §1A.3.3). **Census:** 1/1.

### L41 · A18 L1 + A20 R3-L4 · `fix(sim-mjcf): weld relpose as MuJoCo; parse torquescale`
- **Merged:** both sections propose the same relpose rule (A18 §8.1; A20 §1A.3.4). Parsing `torquescale` completes P39 (A18 §13) — **C2**.
- **Must-fail:** `weld_relpose_as_mujoco` / the `496f2186` literal (A18 §10; A20 §1A.3.4).
- **Census:** `496f2186`'s model agrees; it then differs from step 33 at a box–box contact (A18 §8.1; A20 §1A.4).

### L42 · A20 R3-L5 · `fix(sim-mjcf): fusestatic accumulates body inertias, moves fromto geoms`
- **Implements:** A20 §1A.3.5 (keeps "fused == unfused"; fixes main's three defects); the commit body states the deviation from MuJoCo's frame-order defect — open (A16 Q1; A20 "Open for Jon" 2).
- **Must-fail:** fused `qacc` == unfused `qacc` to 1e-5 on the four fixtures (main fails by 23.9–37.6) (A20 §1A.3.5).
- **Census:** the 2 corpus docs stay different by decision, marked as the deviation (A20 §1A.2, §1A.3.5).

### L43 · A20 R3-L6 · `fix(sim-mjcf): <inertial> orientation alternatives`
- **Implements:** A20 §1A.3.9 (sim-urdf already emits `<inertial euler=…>`). **Conflicts with A4 §4** (C2: inertial `axisangle xyaxes zaxis euler` on the refused list).
- **Must-fail:** none named (the `fs_tool` fixture was measured, A20 §1A.3.9).

### L44 · A11 LR-a · `fix(sim-mjcf): actuator lengthrange computed and kept as MuJoCo 3.5.0` — cross-layer (sim-core)
- **In Rigid-loading although most of it is sim-core:** it needs L01's error variant (A11 §8, Q-LR5) and "cannot be split green" (A11 §8). Body: sim-core changes, the breaking `LengthRangeError`, and `compute_actuator_params` no longer sets lengthrange (A11 §8).
- **Closes:** A5-Q4 (explicit `lengthrange` honoured; 11-settled); ledger-L37 (A16 §2); A6 §5.3's NO-ROW "lengthrange did not converge".
- **Implements:** A11 §2 LR-1 + LR-2 (`LengthRangeOpt` default `uselimit = false`; `Model::set_length_range`; MuJoCo's mode filter; raw `uselimit` copy; called after `builder.build()`).
- **Must-fail:** T1–T7, T9–T12 (A11 §3).
- **Flips:** `fiber.rs:569`, `:1815`, `:1421`; `builder/actuator.rs:967`; `activation_clamping.rs:213`, `:229`, `:255` (template `:65-85`); `actuator_phase5.rs:193`, `:125`, `:225`; `phase7_spec_a.rs:270` (A11 §4).
- **Census:** ok→err 4 + 1; 9 muscle docs' trajectories now equal MuJoCo's (≤ 5.1e-14); 15 docs change `actuator_lengthrange` only (A11 §4); 4 docs `mj-refuses` → `both-refuse` (A20 §2.10). The 4 non-converging docs are open (A11 Q-LR1).

### L45 · A17 C6 · `fix(sim-mjcf): height-field data as MuJoCo (rows bottom-to-top, normalised, grid kept)`
- **Implements:** A17 §11 R3-C sim-mjcf part (f32, rows flipped, MuJoCo normalisation, no resampling; PNG likewise); cf-geometry `HeightFieldData` x/y spacing only if A17 Q6 keeps the grid (open).
- **Must-fail:** `hfield_rows_bottom_to_top`, `hfield_elevation_normalised`, `bodies_rest_on_bumpy_hfield_as_mujoco`, `nonsquare_hfield_as_mujoco` (A17 §11 R3-C tests 1, 2, 4, 6).
- **Flips:** none in the suites run; the raycasting validator's stdout is unchanged (A17 §11 R3-C).

### L46 · unplaced · `fix(sim-mjcf): sleep-policy compile errors and init-sleep refusal as MuJoCo`
- **No section places this commit.** A8 says the two MuJoCo compile errors of SP-3 — "sleep policy only allowed for movable root bodies" and an explicit `allowed`/`init` on a tendon-coupled tree (`engine_setconst.c:232-243`) — and Q3's load refusal "need `MjcfError`" and "belong in the MJCF series (A5's validation area)" (A8 SP-3, Dependencies). A6 §5.1 lists "sleep-policy compile check" as a NO-ROW ok→err kind, 2 docs (`fluid_derivatives.rs:2768`, `sleeping.rs:2615`).
- **Must-fail:** none named.

### L47 · M15 · `feat(sim-mjcf): schema pre-pass at MuJoCo 3.5.0 with the extension allowlist`
- **Closes:** mjcf-S1, mjcf-H2 (depth limit), mjcf-S14; the decisions "unknown attributes and elements are errors, with an allowlist" and "visual-only and capacity-hint elements: accepted with no effect" (01-decisions); 11-settled flex `density` (removed from docs, then refused), A4-Q2/Q3 (allowlist), A4-Q4 (read-tracking sync guard), A4-Q6 (hex floats refused), A4-Q7 (freejoint damping removed), A4-Q9 (weld site form refused), A5-Q5 (`actuatorfrcrange` refused).
- **Implements:** A4 §1.3 (generated `mujoco_3_5_0.rs` + golden schema text + sync test; overlay; worldbody and frame rules; include detection; `_` arms become internal errors) and the frame-sensor `objtype`/`objname` requirement with the class B rewrites (A4 §9). A17 Q5's `nativeccd="disable"` refusal has no stated home.
- **Must-fail:** A4 §1.5's list; refusal tests for the 12 dropped sensors and the limitations (A4 §9).
- **Flips:** classes A, B, C, F (A4 §7); dt16 `parser/tests.rs:2026-2050`; plugin tests t1–t13; `exactmeshinertia.rs` tests become refusals; `adhesion.rs` ac6; `sensor_phase6.rs` pins; `runtime_flags.rs:1385`; `parser/tests.rs:553` (`<compiler><lengthrange/>`) becomes a refusal until L49 (A4 §3, §7).
- **Census:** the corpus re-run must show new refusals ⊆ 3.5.0's refusals ∪ the listed deviations (A4 §9). With A4's recommended allowlist, 171 docs that parse today would be refused before the doc fixes; one of them (`parser/tests.rs:553`) is MuJoCo-loadable (A4 §3).
- **Note:** if C2 resolves as "implement", L31, L32, L33, L41, L43 and L49 must land so that the overlay accepts what they read; the sync guard asserts every accepted attribute is read (A4 §1.3).

### L48 · M25 = A7 DH-3 · `feat(sim-mjcf): delay and history as MuJoCo 3.5.0`
- **Closes:** P-L34, sim-mjcf half.
- **Implements:** A7 H-3: `interval` as `period [phase]`; the 2^24 caps; positive phase and `phase ≤ −period` refused; schema excludes `<user>`/`<plugin>` sensors; stricter rows for negative delay, interval without `nsample`, phase without period; `delay > nsample·timestep` per A7 Q2 (open).
- **Must-fail:** one test per rule (A7 H-3).
- **Flips:** `sensor_phase6_spec_d.rs:39` (add `nsample="1"`), `:413` and `:436` (become refusal tests); three more under A7 Q2 (`actuator_phase5.rs:818`, `:1128`, `sensor_phase6_spec_d.rs:14`) (A7 H-3).
- **Corpus:** 3 MuJoCo-ok docs refused; 3 more under Q2 (A7 H-3).
- **Breaking:** `MjcfSensor.interval: Option<(f64, f64)>`; `with_interval(period, phase)` (A7 H-3).

### L49 · A11 LR-b · `feat(sim-mjcf): <compiler><lengthrange>`
- **Implements:** A11 §2 LR-4; deletes the default-class `lengthrange` plumbing (A11 §8). **Conflicts with A4 §4** (C2).
- **Must-fail:** T8; extend `parser/tests.rs:553` (A11 §3, §8).

### L50 · M26 · `docs: MuJoCo divergences, sim-mjcf docs, conformance status`
- **Implements:** the divergences table with an ID column (A20 §2.4); rows from A1 §10, A5, A7 H-4, A9 §6, A10 §7, A11 §8, A17 R3-B, A19 §6, A20 §1A.3.5; the stale "Real-World Model Loading ✅ COMPLETE" (A1 §10; the earlier single series M26); `MUJOCO_CONFORMANCE.md` 3.5.0 vs 3.4.0 references (A1 "Found in passing"; A20 §2.1); the lengthrange docs list (A11 §5); `POST_V1_ROADMAP.md` DT-107/DT-108 (A7 H-4).

## Merged, split, dropped

| change | sections | why |
|---|---|---|
| merged → L03 | A5 M4 + A10 MH2 | "eig3 **is** M4's `principal_axes`" (A10 §7) |
| merged → L14 | A4 M14 geometry + A14 L39b + A20 §1A.3.6 + A10 §2.4 fromto | same function; size and frame must match together (A14 §2.5; A20 §1A.6) |
| merged → L20 | A5 M17 + A14 M17 addition + A19 builder row | same check site (A14 §5; A19 §6) |
| merged → L41 | A18 L1 + A20 R3-L4 | same relpose rule (A18 §8.1; A20 §1A.3.4) |
| split → L10–L14 | A4 commit 6 (M14) | six unrelated forms; L12 must precede L15 (A20 §2.10) and L10 shares its rewrites with L31/L32 (A9 §5, A13 §4) |
| split → L25–L27 | A5 commit 9 (M22) | four rules in four files; only `.mjb` needs `--features mjb` (A6 §4(e)) |
| moved | A10 MH3's fromto axis → L14 | MH3 then depends on L14 (A10 §2.3 measured the copy without the fromto change as worse) |
| unplaced → L46 | A8 SP-3 / Q3 MJCF half | A8 assigns it to the MJCF series; no section wrote the commit |

## Conflicts that touch this PR

Full descriptions in 20-rigid-physics.md (C1–C5, C8–C11). Loading-only:

| id | conflict | sections | commits |
|---|---|---|---|
| C2 | A4 §4's limitation list vs implementations: `<compiler><lengthrange>` (A11 LR-4), weld `torquescale` (A18 §8.2), flex `elastic2d` (A9 F3), `inertiagrouprange` (A10 Q2.3), `<equality><flex>` (A13 D3), `<inertial>` orientation alternatives (A20 §1A.3.9) | A4 vs A9, A10, A11, A13, A18, A20 | L31, L32, L33, L41, L43, L47, L49 |
| C6 | **Principal axes.** `principal_axes` with `Rotation3::from_matrix_unchecked` after the determinant flip (A5 §3.1, Q1; 11-settled A5-Q1) **vs** MuJoCo's `mjuu_eig3`, which "replaces A5 §3.1's `principal_axes` … so A5-Q1 is moot" (A10 §2.4) | A5, A10 | L03 |
| C7 | **Flex attribute form.** "Same meaning, MuJoCo spelling, mechanical rewrite" (A4 Q1; 11-settled A4-Q1) **vs** "not the same meaning": MuJoCo's `<flex body=…>` names existing bodies as vertices, ours creates them; without a decision flex stays outside the census and the gate (A20 §1B.4) | A4, A20 | L10, L31, L32 |
| C1 | FMA (fused `eig3`, A10 Q2.1) | A10 vs A18, A21 | L03 |
