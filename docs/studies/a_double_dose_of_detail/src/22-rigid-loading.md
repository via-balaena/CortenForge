# Rigid-loading: the commit series

> **Conflicts resolved (2026-10-07).** C1–C11 are conflicts between research sections; these are the answers, and every inline mention of a C-number means this:
> - **C1 FMA → plain arithmetic** (Jon). No `mul_add` emulation; bit-exact tests compare against MuJoCo's C compiled without FMA (as A21 did); the census gate compares at 1e-9 against goldens from that build (40-verification).
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
> **Not measured:** no section ran these commits in this order; per-commit census counts are estimates from separate bases.


Input: *A Double Dose of Detail* at `caa1c1c0`, Parts 0–3 and research A1–A21, revised after stress-test round 1 (reviewers R1–R5, 2026-10-07). An M-number (M1…M26) is the earlier single series, `20-commit-series.md` at `caa1c1c0`. Nothing here was built or run; every statement points at the section or reviewer it rests on, and "not measured" / "not isolated" are kept where a section says so. Parts 0–3 win over the research where they disagree (00-how-to-read).

**What this PR holds** (01-decisions, second round): sim-mjcf errors, defaults, parser, validation, mesh/STL, inertia. It lands after Rigid-physics (`P01…P53`, 20-rigid-physics.md). A commit that must also change sim-core, cf-geometry, mesh-io, sim-urdf or cf-design is marked **cross-layer** or **cross-crate** in its body, with the section that forces it.

## Conventions

As in 20-rigid-physics.md: `L01…L50` name the commits, a letter suffix (`L10a`) is a commit added after the stress test, and the order is the Summary's row order (L49 sits before L47); "src" is the book's or the section's id; titles carry no `!` and one scope (the commit-msg hook); must-fail tests are the ones the research or a reviewer's draft names; census counts are what a section measured on the base it names — **not measured as a series**. In addition:
- Every MJCF commit carries its MuJoCo 3.5.0 citation and is checked by the five buckets of A6 §4(b) (unchanged · ok→ok changed on the pre-registered list · ok→err mapped to this commit's rule · err→ok MuJoCo-ok · err→err message change). From L02 on, "NEW NONDETERMINISTIC" must stay empty (A6 §4(b), §6).
- Every commit re-blesses `verdicts.tsv`; a deliberate new refusal of a MuJoCo-loading doc is a hand-edited regression with its `divergence=<ID>` (A20 §2.4, §2.7). Commits that rewrite in-tree MJCF also append-refresh the gate's corpus snapshot (A20 §2.2, §2.7).
- **Divergence rows:** each commit adds, in the same commit, the divergences-table row of every deviation it introduces, including each `divergence=<ID>` its verdicts name. The registry and its ID column are added by P01 (20-rigid-physics.md). L50 adds only the rows of deviations that exist at `main` and stay.
- **Bit-exact goldens** come from the unfused oracle named in 40-verification § The unfused oracle (C1), not from the arm64 wheel.
- **Hazards:** a must-fail whose parent side hangs or grows is marked either "established by reading; do not run at the parent" or "run only under the 2.5 GB RSS watchdog (40-verification)".
- A research sketch that returns `ModelConversionError` means L01's `MjcfError`.
- Per-commit greenness in this order was not run by any section (A3 says each of its four is "green on its own" as a design statement; A4 §9 likewise; A5 §5; none ran a series).

## Summary

| # | src | title | breaking | depends on |
|---|---|---|---|---|
| L01 | M1 (A3 #1) | `feat(sim-mjcf): typed MjcfError with locations` | **yes** | — |
| L02 | M3 (A6) | `fix(sim-mjcf): flex edges in MuJoCo's order, hinges in edge order` | behaviour | L01 |
| L03 | M4 (A5 #1) + A10 MH2 | `fix(sim-mjcf): a non-finite inertia is an error, not a hang; principal axes as MuJoCo (mjuu_eig3)` | behaviour | L01 |
| L04 | M5 (A4 #1) | `fix(sim-mjcf): depth-safe traversals on a large stack` | behaviour | L01 |
| L05 | M6 (A4 #2) | `feat(sim-mjcf): one attribute reader with MuJoCo's rules` (cross-crate) | **yes** | L01 |
| L06 | M7 (A5 #2) | `feat(sim-mjcf): every non-finite number is refused` | behaviour | L05, L03 |
| L07 | M8 (A4 #3) | `feat(sim-mjcf): keywords exact; composite curve keywords as MuJoCo` | **yes** (pub `from_str`) | L05 |
| L08 | A9 C1 | `fix(sim-mjcf): cable vertices pass through f32 as MuJoCo` | behaviour | L07 |
| L09 | M10 (A4 #5) | `fix(sim-mjcf): repeated <compiler>/<option> merge` | no | L05 |
| L10 | M14 part (A4 #6); C7 (R4) | `feat(sim-mjcf): <flex body= vertex= element=> with MuJoCo's meaning; in-tree docs declare their vertex bodies` (cross-layer) | behaviour | L01, L02, L05 |
| L10a | U06 (A20 §1B.5) | `fix(sim-mjcf): flex vertex bodies get body_gravcomp and jnt_actgravcomp entries` | behaviour | L10 |
| L11 | M14 part | `feat(sim-mjcf): intvelocity actuator; user sensor dim` | **yes** (enum variant) | L05 |
| L12 | M14 part | `feat(sim-mjcf): connect body and site forms as MuJoCo` | **yes** (`MjcfConnect`) | L05 |
| L13 | M14 part | `fix(sim-mjcf): an unnamed mesh or hfield takes MuJoCo's name from its file` | behaviour | L05 |
| L14 | M14 part + A14 L39b + A20 §1A.3.6 | `fix(sim-mjcf): body xyaxes/zaxis, fromto size and frame, plane size as MuJoCo` | **yes** (`MjcfBody` fields) | L05 |
| L15 | M9 (A4 #4) | `fix(sim-mjcf): self-closing elements are not dropped` | no | L12, L10 |
| L16 | M11 (A3 #2) | `feat(sim-mjcf): defaults resolved in one pass before frames and composites` | **yes** (un-exports) | L15 |
| L17 | M12 (A3 #3) | `feat(sim-mjcf): explicit values survive their class` | **yes** (`Option` fields) | L05, L07, L16 |
| L18 | M13 (A3 #4) | `feat(sim-mjcf): default geom size, element-wise overlay, size must be positive` | behaviour (`size`'s type is L05's) | L07, L16 |
| L19 | M16 (A5 #3) | `feat(sim-mjcf): joint layout as MuJoCo` | behaviour | L16 |
| L20 | M17 (A5 #4) + A14/A19 | `feat(sim-mjcf): joint axis too small is an error; ball and free axes are (0,0,1)` | behaviour | L16 |
| L21 | M18 (A5 #5) | `feat(sim-mjcf): one limit helper for joints, tendons, actuators` | behaviour | L17 |
| L22 | M19 (A5 #6) | `feat(sim-mjcf): sizes, masses and inertias as MuJoCo` (cross-crate) | behaviour | L18 |
| L23 | M20 (A5 #7) | `feat(sim-mjcf): duplicate names are errors; sim-urdf world link` (cross-crate) | behaviour | L16 |
| L24 | M21 (A5 #8) | `fix(sim-mjcf): frame contents are validated` | behaviour | L16 |
| L25 | M22 part (A5 #9) | `fix(sim-mjcf): geom and pair condim checked as MuJoCo` | behaviour | L16 |
| L26 | M22 part | `fix(sim-mjcf): an empty <contact/> appends; repeated pairs are both kept` | behaviour | L16 |
| L27 | M22 part | `fix(sim-mjcf): .mjb decoding is size-limited` | behaviour | L01 |
| L28 | M24 (A5 #11) | `fix(sim-mjcf): timestep > 1 loads` | behaviour | L01 |
| L29 | A9 F1 | `fix(sim-mjcf): dim-3 flex elements oriented as MuJoCo` | behaviour | L02 |
| L30 | A9 F2 | `fix(sim-mjcf): flex flaps as MuJoCo — dim 2 only, boundary [opp, -1]` | behaviour | L02 |
| L31 | A13 D3 (loading half) | `feat(sim-mjcf): flex edge constraints only through a flex equality, as MuJoCo` | behaviour | L02, L10, P36a |
| L32 | A9 F3 | `feat(sim-mjcf): flex elastic2d — bending only when asked, as MuJoCo` | behaviour | L30, L10, L01, L05, L07 |
| L33 | A10 MH3 | `fix(sim-mjcf): body inertial frame as MuJoCo: one geom copied, orientation alternatives` | behaviour | L03, L22, L14 |
| L34 | A20 R3-L1 | `fix(sim-mjcf): plane and hfield geoms have MuJoCo's volume; zero-volume mass is zero` | behaviour | L33 |
| L35 | A10 MH4 | `fix(sim-mjcf): mesh mass properties as MuJoCo (legacy default, centroid apex, exact-mode orientation check)` | **yes** (`MeshInertia` default; mesh frame) | L03, L01 |
| L36 | A10 MH5 | `fix(sim-mjcf): STL vertices deduplicated as MuJoCo; non-finite vertices refused; left-handed scale; binary STL by size` (cross-crate) | behaviour | P03, L01 |
| L37 | A10 MH6 | `fix(sim-mjcf): a mesh that needs a hull and has none is an error` | behaviour | P03, L35 |
| L38 | M23 (A5 #10) | `feat(sim-mjcf): a vertex-only mesh gets its convex hull` | behaviour | P02, L37 |
| L39 | A20 R3-L2 | `fix(sim-mjcf): discardvisual keeps the discarded geoms' inertia; sim-urdf emits no visual geoms` (cross-crate) | behaviour | L16 |
| L40 | A20 R3-L3 | `fix(sim-mjcf): ellipsoid shell inertia as MuJoCo` | behaviour | — |
| L41 | A18 L1 + A20 R3-L4 | `fix(sim-mjcf): weld relpose as MuJoCo; parse torquescale` | not stated | L01, P39 |
| L42 | A20 R3-L5 | `fix(sim-mjcf): fusestatic accumulates body inertias, moves fromto geoms` | behaviour | L16, L03 |
| L43 | A20 R3-L6 | `fix(sim-mjcf): <inertial> orientation alternatives` (cross-crate) | behaviour | L05 |
| L44 | A11 LR-a | `fix(sim-mjcf): actuator lengthrange computed and kept as MuJoCo 3.5.0` (cross-layer, cross-crate) | **yes** | P11, L01, P04 |
| L45 | A17 C6 | `fix(sim-mjcf): height-field data as MuJoCo (rows bottom-to-top, normalised, grid kept)` (cross-layer, cross-crate) | **yes** (`HeightFieldData`) | P52 |
| L46 | A8 SP-3/Q3 (R4) | `fix(sim-mjcf): sleep-policy compile errors and init-sleep refusal as MuJoCo` | behaviour | L01, P11, P20, P23 |
| L49 | A11 LR-b | `feat(sim-mjcf): <compiler><lengthrange>` | additive field | L44, L09, L05, L06 |
| L47 | M15 (A4 #7) | `feat(sim-mjcf): schema pre-pass at MuJoCo 3.5.0 with the extension allowlist` | behaviour | L05, L07, L10–L14, L31, L32, L33, L41, L43, L49 |
| L48 | M25 = A7 DH-3 | `feat(sim-mjcf): delay and history as MuJoCo 3.5.0` | **yes** (`MjcfSensor.interval`) | L01, L05, L16, L47, P08 |
| L50 | M26 | `docs: MuJoCo divergences, sim-mjcf docs, conformance status` | no | all |

51 commits (50 + L10a).

## What fixes the order

1. **L01 first:** "This is the PR's FIRST mjcf commit" (A3 §1); every later rule returns its variants (A4 §10, A5 §6, A10 §8, A11 §8). Everything frozen in this PR freezes together, so variants later commits need are added now (A3 §6, "Dependencies").
2. **L02 after L01** (A6 §7: M3's refusal needs the error type).
3. **L03 before L06** ("1 before 2 so commit 2's NaN tests never reach a hang", A5 §5); L03 = M4 + MH2 because "eig3 **is** M4's `principal_axes`" (A10 §7).
4. **L05 → L06, L07, L09, L10–L14** (A4 §9; A5 §6: the non-finite check sits in the reader's token function).
5. **L07 before L18**: `checksize` must not land before the curve keyword fix, or `composite.rs` t1 goes red (A3 §3, §6). **L07 → L08** (A9 §5).
6. **L12 before L15.** M9 without the connect-anchor refusal turns 5 both-refused docs into lenient loads; the gate caught it (A20 §2.10). L12 holds that refusal (A4 §9 commit 6 "connect form rule").
7. **L15 before L16**: the self-closing `<default class/>` arm must be in before S10 refuses undefined classes (A4 §8.2, §10).
8. **L16 → L17 → L21; L16 → L18 → L22; L16 → L19, L20, L23–L26** (M12/M13/M16–M22; A5 §5: S7 needs S3; A5 §2: S12 after expansion and before discardvisual/fusestatic).
9. **L22 before L33** (A10 §7: MH3 after M19, else `fluid_derivatives.rs:1391` and `fluid_forces.rs:119,185` go NaN instead of refusing). **L14 before L33** (the fromto axis moves into L14 — see "Merged"; A10 §2.3 measured that the one-geom copy without the fromto change is worse than main).
10. **L10 before L15, L31, L32, L47** (R4): L15's self-closing flex test, L31's fixture and L32's rewrites use L10's vertex-body docs, and the child form is refused only once the docs are rewritten. **L10 → L10a:** both edit `create_flex_vertex_body`. **L02 → L29, L30, L31** (A9 §5; A13 §4). **L30 → L32**; **L31 and L32 before L47** — otherwise the schema pre-pass refuses `<edge equality>`, `<equality><flex>` and `elastic2d` as unimplemented (A9 §5, A13 §4).
11. **P03 (MH1) → L36, L37; L35 → L37; L37 → L38** (A10 §7–§8: hull built once on deduplicated points; M23 after MH6).
12. **L44 needs P11, L01, P04** (A11 §8).
13. **L45 after P52** (A17 §12).
14. **L47 after L10–L14** ("6 before 7 so the docs can be rewritten to MuJoCo forms before the pre-pass refuses our spellings; 2–3 before 7", A4 §9) **and after L31, L32, L33, L41, L43, L49**: C2 implements what those read, and L47's sync guard asserts every accepted attribute is read (A4 §1.3).
15. **L48 after L01, L05, L16, L47** (A7 §6) and after P08 (A13 §4).
16. **L49 after L44, L09, L05, L06** (A11 §8) **and before L47** (R2, R3, R5). A11 §8 puts LR-b after M15 so that the schema lists `compiler/lengthrange`; MuJoCo 3.5.0's schema lists it already (`xml_native_reader.cc:104-107`), so L47's generated schema accepts it.

## Commits

### L01 · M1 · `feat(sim-mjcf): typed MjcfError with locations`
- **Closes:** mjcf-E1, mjcf-E2.
- **Implements:** A3 §1's enum (`#[non_exhaustive]`, struct-shaped variants, `String` names, opaque `Location`, `Io { path, source }`, `Unsupported` only for stated limitations; plus `#[cfg(feature = "mjb")] MjbTooLarge`) (11-settled #2); A3 §2 message-only locations (11-settled E2); the variant mapping of 30-error-type (A4 §10 and A5 §6 names); A11 Q-LR5's `LengthRange { actuator, source, at }` variant (Q126; A3 §6: add variants now).
- **`.mjb` version:** `MJB_VERSION` (`mjb.rs:52`) goes from 3 to 4 here, once for the PR. `.mjb` is bincode of `MjcfModel` (`mjb.rs:175-176`), and later commits change that shape (A9 §2.8; R2); none of them bumps it again.
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
- **Merged:** A5 commit 1 and A10 MH2 — "eig3 **is** M4's `principal_axes`, so merge MH2 into M4" (A10 §7). C6: `eig3` replaces A5 §3.1's `principal_axes`.
- **Closes:** mjcf-H1 (hang); the known divergence "principal-axis order of a full inertia" (01-decisions).
- **Implements:** A5 §3.1 (`Result` through `resolve_body_inertia`; refuse a non-finite tensor, eigen-decomposition or mass) with A10 §2.4's `builder/eig3.rs` (`eig3`, `full_inertia`, `global_inertia`, `offcenter`); the non-finite guard sits on eig3's output (A10 §2.4). Plain arithmetic (C1).
- **Must-fail:** `hang_is_an_error_typed_model` and `mass_overflow_is_an_error` (A5 §3.1) — run only under the 2.5 GB RSS watchdog (40-verification); `eig3_matches_mujoco_bitwise` (A10 §2.4 #1), with its literals from the unfused oracle — A10's came from the fused arm64 wheel, which a plain transcription matched on 251 of 20,211 inputs (A10 §2.3).
- **Flips:** `mesh_inertia_modes.rs:218` t9 → MuJoCo's decreasing order (A10 §2.3, §7).
- **Census:** NEW-FRAMEQUAT `f0b36671` agrees once MH2 lands (A19 §4.1). Corpus, MH2–MH5 together: 271 docs `model_fp`, 66 trajectory (A10 §7). Both were measured with the fused port against the fused (arm64) wheel; under plain arithmetic they are not measured (`f0b36671` has two equal principal moments, so its axis order is a discrete choice, R2).

### L04 · M5 · `fix(sim-mjcf): depth-safe traversals on a large stack`
- **Closes:** mjcf-H2 part 1; P-L35 (`all_bodies`).
- **Implements:** A4 §1.4: `on_large_stack` (64 MiB) around the six recursive pub entry points; iterative `body()`/`joint()`; the `all_bodies` fix; the 4096 tree-depth guard, checked at the entry of `model_from_mjcf`, `validate`, `validate_tendons` and after `expand_composites` (A4 §1.4; 11-settled #16). A5 §3.8 notes that decoding a deeply nested `.mjb` already recurses in serde before `model_from_mjcf`; whether this commit covers that path is not stated.
- **Must-fail:** `load_at_depth_limit_from_small_thread` (its own test file), `caller_built_tree_depth_guard` ("by the slopes; not run"), `all_bodies_returns_every_level` (A4 §1.5).
- **Note:** drop recursion is a stated residual; measure drop alone, iterative `Drop` if > 512 KB (A4-Q5 → 11-settled).

### L05 · M6 · `feat(sim-mjcf): one attribute reader with MuJoCo's rules` — cross-crate (sim-urdf)
- **Closes:** mjcf-S2 (numbers, lengths, required attributes), P-L28 single-value friction, mjcf-S8 (zero quaternion, multiple orientations), P-L35 (zero-quaternion hang); 10-scope's "sim-urdf emits `<inertial>` without `pos` (28 of 37 in-tree URDFs)".
- **Implements:** A4 §2.3 (`Attrs`; `Prefix<N>` and `TfAuto` pub types; exact lengths; trim; underflow; empty strings; entity unescape; the required attributes of §2.4 except sensor targets and user `dim`; `quat`; multiple-orientation refusal). Merging moves to `over`/`over_prefix` in the defaults file (A4 §2.3). This fixes A13 §1.1's one-value `solref` and A20's partial `polycoef` (A20 §1A.2). Non-finite tokens are L06's.
- **Owns** (one place each, R2): `<inertial>` `pos` and `mass` required (A4 §2.4); the field types `gear: Option<Prefix<6>>`, `dynprm: Option<Prefix<10>>`, `gainprm`/`biasprm: Option<Prefix<9>>` and geom `size: Option<Prefix<3>>` (A4 §2.3, §10; the `Option` is A3 §3's). L17 and L18 use these types.
- **sim-urdf** (ledger-L64): every revolute or prismatic `<limit>` is written `limited="true"`, a missing bound as 0 (`urdf/src/parser.rs:411-412`, `converter.rs:350-362`). MuJoCo's importer leaves a joint with one bound given, or `lower > upper`, unlimited, and otherwise leaves `limited` automatic, so `lower == upper` is unlimited too (`xml_urdf.cc:499-508`, `user_objects.cc:184-189`). Ours: a range that does not increase is refused at `make_data` since P10, and `upper="1"` alone loads limited to (0, 1) (by reading). `UrdfJointLimit` then needs to carry which bounds were given.
- **sim-urdf:** the converter writes `<inertial pos>` only when it is non-zero (`urdf/src/converter.rs:451-454`); it now always writes it (A5 §3.5). Without that, 28 of the 37 convertible in-tree URDFs are refused (A5 §3.5) and `urdf/src/lib.rs:165` `test_two_link_arm_model_data` fails (its `base_link` inertial has no `<origin>`; R2, by reading).
- **Must-fail:** A4 §2.5's number/length cases (`timestep="0.01s"`, `damping="abc"`, `mass="2kg"`, `mass="2,5"`, `mass=" 2 "`, `condim="3.0"`, `group="99999999999"`, `pos="0 0 0 1"`, `friction="1 2 3 4"`, `gear` ×7, `mass="1e-999"`, `name="a&amp;b"`, `friction="0.7"`, `solref="0.05"`, `fluidcoef="0.5 0.25"`); `s11_gear` (A3 §3; it passes at L17's parent once this commit's `over_prefix` lands, R2); A4 §8.4's multiple orientation; A4 §8.4's zero quaternion — run only under the 2.5 GB RSS watchdog (40-verification). A4 measured main not returning within 20 s; with L03 in, the parent may return a different `Err` instead (R2, by reading), so the test asserts MuJoCo's "zero quaternion is not allowed".
- **Flips:** 19 five-value `friction` rewrites (class E); `inertial` without `pos` in validator `urdf-loading/stress-test/src/main.rs:120` `ARM_MJCF`; the multiple-orientation site `builder/body.rs:702`; `fluid_forces.rs:1205-1220` T39 err→ok (A4 §7, §9).
- **Census:** `5eb35113` (partial `polycoef`) flips (A20 §1A.2).
- **Breaking:** pub field types become `Prefix<N>`/`TfAuto`; new pub types (A4 §2.3).

### L06 · M7 · `feat(sim-mjcf): every non-finite number is refused`
- **Closes:** mjcf-S9 (NaN body pos), mjcf-H1 (decision: refuse every NaN/±inf, 01-decisions consequences).
- **Implements:** A5 §3.1 target 1 (`parse_f64` in L05's token function; message contains "finite"); an overflowing literal gets MuJoCo's "number is too large" (A4 §2.3). This commit is the one owner of the non-finite rule (R2).
- **Must-fail:** `nan_rejected_everywhere` (A5 §3.1).
- **Flips:** none, if the message contains "finite" (keeps `keyframes.rs` ac22/ac23) (A5 §3.1).
- **Note:** caller-built and `.mjb` input see no parse; builder guards plus docs (A5-Q9 → 11-settled).

### L07 · M8 · `feat(sim-mjcf): keywords exact; composite curve keywords as MuJoCo`
- **Closes:** mjcf-S2 (keywords); the `curve` keyword map (10-scope "also fixed").
- **Implements:** A4 §2.3 keyword tables (`keywords.rs`; allowlisted extension keywords; dead joint/geom aliases removed; pub `from_str` exact; `TfAuto` for `*limited`) and A9 §4.2's rewrites, including the template arguments at `composites/stress-test/src/main.rs:69,86,114,245` and `:282` (`"s c 0"` → `"sin(s) cos(s) 0"`), which the corpus extractor cannot see. Trailing whitespace in `curve` is accepted, lenient, with a test that it equals the trimmed form (Q117; A9 Q8).
- **Must-fail:** A4 §2.5's keyword cases (`integrator="RK45"`, `"euler"`, `solver="newton"`, `angle="RADIAN"`, `<flag gravity="off"/>`/`"true"`, `limited="1"`, `limited="auto"`, `type=" box"`); `cable_curve_keywords_as_mujoco` (A9 §4.4).
- **Flips:** class D rewrites, 19 docs (A4 §7); with the rewrites 1 doc changes (`composite.rs:25`, passes) and the composite validator's stdout is byte-identical; without them 9 integration and 2 lib tests flip (A9 §4.5).
- **Census:** with L08, `263f733b` agrees (A20 §1A.2); A9's 17 curve docs move `mj-refuses` → `both-refuse` (A20 §2.10).

### L08 · A9 C1 · `fix(sim-mjcf): cable vertices pass through f32 as MuJoCo`
- **Implements:** A9 §4.2 C1; the comment fix `builder/composite.rs:390-393`; pins for the 2-body cable (lenient, named by 01-decisions) and the 1-body cable (A9 §3.5, §3.6).
- **Must-fail:** `cable_vertices_round_through_f32` (A9 §4.4); pins `two_body_cable_matches_mujoco_generator` and `one_body_cable_frame_is_defined` pass on main (A9 §3.5–§3.6).
- **Flips:** 10 docs model + trajectory; 0 tests (A9 §4.5).

### L09 · M10 · `fix(sim-mjcf): repeated <compiler>/<option> merge`
- **Closes:** mjcf-S5. **Implements:** A4 §8.3. **Must-fail:** the two measured cases. **Flips:** none (A4 §8.3).

### L10 · M14 part + C7 · `feat(sim-mjcf): <flex body= vertex= element=> with MuJoCo's meaning; in-tree docs declare their vertex bodies` — cross-layer (sim-core: one deletion)
- **Split from M14** (A4 §9 commit 6 bundles six unrelated forms; each touches different elements). R4's draft.
- **Also** (ledger-L63): in 73 flex docs the per-joint arrays `jnt_name`, `jnt_margin`, `jnt_solref`, `jnt_solimp`, `jnt_group`, `jnt_actgravcomp`, `jnt_user` are empty, and in 77 `body_gravcomp` and `body_user` are short: the flex child form's generated bodies and joints skip those pushes. Run the per-element length check of sim-core's factory test (`factory_models_size_every_per_element_array`) over the loadable census docs; 0 short is the gate.
- **Closes:** C7 / Q81 (MuJoCo's flex meaning; A4-Q1's "same meaning" is superseded); A6 §1.10 / A20 §1B.3 C4 (`vertex=`/`element=` ignored today); U07 (an empty `element` loads a 1-vertex flex; MuJoCo refuses).
- **Now.**
  - The parser reads `body` (`parser/deformable.rs:233-235`) and `node` (`:237-239`) as name lists and reads the `<vertex>`/`<element>` children (A4 §3). It ignores `vertex=`/`element=`; A20 measured 72 of 79 docs loading with `nflexvert = 0`.
  - The builder (`builder/flex.rs:231`, `create_flex_vertex_body`) creates one body per vertex, with 3 world-axis slides, or none if pinned. `body[i]` is that body's *parent*; an unknown name warns and falls back to world (`:254-271`). Vertex masses are lumped with density 1000 and a floor (`compute_vertex_masses`, `:639`).
  - sim-core: `flexvert_xpos[i] = xpos[flexvert_bodyid[i]]` (`dynamics/flex.rs:14-17`). `flex_damping` acts twice: as absolute vertex damping (`forward/passive.rs:474-490`) and as cotangent-bending Rayleigh damping (`:569`).
- **Target** (MuJoCo 3.5.0).
  - `body` and `element` are required (`xml_native_reader.cc:1478,1488`).
  - `vertbody` names existing bodies; an unknown name is an error (`user/user_mesh.cc:4079-4090`).
  - Without `vertex` the flex is centered: vertices sit at the body origins (`:4130-4133`). With `vertex` there is 1 body (rigid) or nvert bodies (`:4134-4141`).
  - `node` means trilinear interpolation (`:4105`, `:4148-4154`). An empty `element` is "elem is empty" (`:4111-4113`).
  - `<flexcomp>` creates its own bodies and lives under `<body>` (`xml_native_reader.cc:312-327`).
- **Change.**
  1. **Parser.** `MjcfFlex.body` names the vertex bodies; `vertex=` and `element=` are read as attributes. Refused: the `<vertex>`/`<element>` child forms; `node=` (stated limitation); an empty `element` (parity, U07); `<flex><pin>` and `<flex mass>`. `<pin>` and `mass` stay on `<deformable><flexcomp>` (A4-Q3).
  2. **Builder, for `<flex>`.** `flexvert_bodyid[i]` = the id of `body[i]`; no bodies are created. `flexvert_mass` = that body's mass; `invmass` = 0 iff the body has no dofs. An unknown name → `UndefinedReference`. The supported subset; anything else is a stated-limitation refusal:
     - (a) nvert distinct names, so a single-body rigid flex is refused;
     - (b) `vertex=` omitted, or every offset zero;
     - (c) each vertex body has no joints, or exactly 3 slides on axes x, y, z in that order, with an unrotated frame and a parent that is world or has no dofs.

     Rule (c) is where sim-core's flex assumption is checked: the flex Jacobian maps a vertex's 3 dofs straight onto world x, y, z (`constraint/jacobian.rs:62-74`, `:194-202`), and the vertex position is its body's `xpos` (`dynamics/flex.rs:14-17`).
  3. **Flexcomp.** `<deformable><flexcomp>` keeps generating bodies; `create_flex_vertex_body` becomes flexcomp-only and gives each slide it creates the flex's `flex_damping` as joint damping, in place of the vertex damping step 4 deletes (`passive.rs:474-490`).
  4. **Doc rewrites** (A20 §1B.4, stage S4), R4's counts: the 92 `<flex>` elements with `<vertex>` children (`flex_unified.rs` 59, `flex_flex_collision.rs` 25, `mjcf/src/parser/tests.rs` 4, `deformable_friction_dt25.rs` 3, `runtime_flags.rs` 1); the 7 `format!` templates (A4 §7); the living markdown that describes `<flex>` as a direct vertex/element specification (`sim/docs/ARCHITECTURE.md:476`, `MUJOCO_CONFORMANCE.md:96`, `MUJOCO_GAP_ANALYSIS.md:1109`; the historical `sim/docs/todo/**` stay).
     - For each vertex, a `<body name="<flex>_v<i>" pos="<vertex i>">` at the end of `<worldbody>`, in vertex order, which keeps today's body ids. It holds 3 slides (none if pinned) and `<inertial pos="0 0 0" mass="<today's lumped mass>" diaginertia="1e-15 1e-15 1e-15"/>` (MuJoCo's `mjMINVAL`); the masses come from today's `compute_vertex_masses`.
     - The flex becomes `<flex body="…" element="…">`, with no `vertex`. In the templates, `{sqrt3_2}` moves into `pos`.
     - The `node=` docs (`flex_unified.rs:1264,1322,1398,1447`): the node bodies become the 3-slide vertex bodies. This moves the geom mass onto the vertices; the effect on the tests' tolerances was not checked. `:1362` becomes a refusal test.
     - The `body="0"` tests (`builder/mod.rs:1353,1387`) get 4 named pinned bodies.
     - **Damping per slide:** where a test relies on today's absolute vertex damping, `damping="d"` goes on each slide; by reading, `passive.rs:486-488` has the same form as joint damping (not measured). Then `passive.rs:474-490` is deleted (the sim-core part), which leaves `<elasticity damping>` its MuJoCo meaning (`:569`; `engine_passive.c:265`). By reading, the deleted loop also damps flexcomp-generated vertices (it runs over every flex vertex, `passive.rs:476-489`); what the deletion does to them is not yet measured (guard below).
- **Must-fail** (not run).
  - `flex_body_names_existing_bodies`: 3 declared 3-slide bodies + `<flex body="a b c" element="0 1 2">` → nbody 4, nflexvert 3, `flexvert_bodyid` = the ids of a, b, c. Main: `nflexvert` = 0 (A20 §1B.2).
  - `flex_unknown_vertex_body_is_an_error`: main warns and loads.
  - `flex_child_vertex_form_is_refused`, `flex_node_is_refused`, `flex_empty_element_is_refused` (U07), `flex_pin_and_mass_are_refused`: main loads each.
  - `flex_vertex_body_with_a_hinge_is_refused`: main makes the hinge body a *parent*.
  - Must not change: `flexcomp_vertex_damping_unchanged`: a `<deformable><flexcomp>` with `<elasticity damping>` > 0 (no in-tree test sets one), `qpos` and `qvel` after 100 steps bit-identical to the parent's.
  - A one-off A/B, not in CI: each rewritten doc has the same `nbody`, `body_mass`, `body_pos`, `jnt_*`, `flexvert_*` and `flexedge_*` as the old form at the parent.
- **Flips:** every in-tree flex doc, through the rewrites; `flex_unified.rs:1362`.
- **Census:** append the rewritten forms to the snapshot with their MuJoCo 3.5.0 goldens (`gen_census_golden.py`); the old forms move to both-refuse. At main, under e2: 0 of 72 agree, and 35 of 66 are nondeterministic until L02 (A20 §1B.5). The classes after this commit were not measured.
- **Divergence row:** flex contacts are element-based in MuJoCo and vertex-based in ours (A20 §1B.7; Q81); no research ports them.
- **Downstream:** none outside `sim/L0` (R4).
- **Note:** U06 (the `rne.rs:357` gravcomp panic): by reading, `<flex>` vertex bodies are now ordinary MJCF bodies and avoid it; the flexcomp path keeps it until L10a. Not run. The body-level `<flexcomp>` (`nflex = 0` today, A13 §5) is on L47's refused list (A4 §4).

### L10a · U06 · `fix(sim-mjcf): flex vertex bodies get body_gravcomp and jnt_actgravcomp entries`
- **Closes:** U06 (13-open-questions; A20 §1B.5).
- **Now.** `create_flex_vertex_body` (`mjcf/src/builder/flex.rs:231`) pushes no `body_gravcomp` for the vertex body and no `jnt_actgravcomp` for its slides, which `builder/body.rs:220` and `builder/joint.rs:137` push for every other body and joint. A flex doc plus any `gravcomp` body then panics in `forward` at `dynamics/rne.rs:357` ("index out of bounds: the len is 2 but the index is 2"); MuJoCo loads such a doc (A20 §1B.5, measured).
- **Change.** The vertex body pushes `body_gravcomp` 0.0, each of its slides `jnt_actgravcomp` false.
- **Must-fail:** `flexcomp_with_a_gravcomp_body_steps`: a `<deformable><flexcomp>` plus one body with `gravcomp="1"`; `step` returns `Ok`. Main panics, by reading: A20 §1B.5 measured the panic on a `<flex>` doc (`flex_gravcomp.xml`), not a `<flexcomp>`.
- **Census:** not measured. After L10 only `<deformable><flexcomp>`, our extension (11-settled A4-Q3), reaches `create_flex_vertex_body`.
- **Note:** it edits sim-mjcf only, so it is in this PR (split by layer, 01-decisions). L10 leaves `create_flex_vertex_body` serving `<flexcomp>` only (C7), so the fix stays needed.

### L11 · M14 part · `feat(sim-mjcf): intvelocity actuator; user sensor dim`
- **Closes:** mjcf-S1 (`intvelocity`). **Implements:** A4 §5 (`mjs_setToIntVelocity`), A4 §4 (user `dim="1"` only).
- **Must-fail:** `intvelocity` model fields equal the measured 3.5.0 values (A4 §5).
- **Flips:** `callbacks.rs:199-212` gains `dim="1"`; `parser/tests.rs:2283` (`dim="3"`) becomes a refusal test (A4 §9).
- **Census:** P18 added `bcbf72b425d3ab65` (`<user dim="1"/>` on a hinge, from `history.rs` `try_make_data_refuses_a_delayed_user_sensor`), verdict `model:nsensordata`: sim-mjcf gives that user sensor dimension 0, MuJoCo 1.
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
- **Implements:** A4 §8.4 (a plane's size as given; its positivity check is L18's `checksize`); `compute_fromto_pose(fromto, size, geom_type)` with `mjuu_z2quat(from − to)` (A14 §2.3); fromto resolved in a rotated frame's coordinates, after defaults (A20 §1A.3.6).
- **Must-fail:** A4 §8.4's cases except the plane `size(3)` refusals (L18's); A14 §2.6 (capsule `geom_quat` literals; box frame with sizes; `framequat objtype="geom"` golden); `fromto_axis_is_from_minus_to` (A10 §2.4 #4); the `4eed75d1` literal (A20 §1A.3.6). A14's and A20's literals came from the fused arm64 wheel; they are regenerated from the unfused oracle.
- **Flips:** plane `geom_size` changes in 282 docs ("whether any test asserts a plane's `geom_size` was not checked", A4 §8.4); `geom_quat` in 200 docs, trajectories in 21 (20 at ≤ 8.4e-14; the chaotic `equality-constraints/stress-test:388` validator still passes, two printed lines change) (A14 §2.4).
- **Census:** `fromto` 156 of 166 flip alone; the rotated-frame pair 2/2 (A20 §1A.2).
- **Breaking:** new pub `MjcfBody` fields (A4 §8.4).

### L15 · M9 · `fix(sim-mjcf): self-closing elements are not dropped`
- **Closes:** mjcf-S4. **Implements:** A4 §8.2 (`Start | Empty` with `empty: bool`).
- **Must-fail:** self-closing mocap body (main: nbody 1, nmocap 0); `<default><default class="e"/></default>` resolves (A3's d15; this commit owns it, L16 does not repeat it); self-closing flex builds, in L10's form (A4 §8.2).
- **Flips:** `parser/tests.rs:1256` and seven other self-closing-body docs, build-err → build-ok or the connect error (A4 §8.2).
- **Census:** `94e0077d` flips (A20 §1A.2); 5 both-refused docs would become lenient loads without L12 first (A20 §2.10).

### L16 · M11 · `feat(sim-mjcf): defaults resolved in one pass before frames and composites`
- **Closes:** mjcf-S10, P-L26, and 10-scope's "defaults ignore `<frame>` composition; discardvisual/fusestatic read unresolved classes; composite elements take the user's defaults" (A3 §4).
- **Implements:** A3 §3 (ordered snapshot `DefaultResolver`, crate-private: 11-settled A3-O2, O3), A3 §4 option (b) (`resolve::apply_defaults`, written iteratively; `validate_resolved`; `validate_tendons` folded in and un-exported). d10 is `RepeatedElement` from a seen-set in `parse_default`; L47's schema pass lands later (A3 §3).
- **Must-fail:** d01–d09, d12, d11, f01, f02, k01, k02, c01, c02 (A3 §6); both ledger-L26 shapes — established by reading (A3 ledger-L26; `defaults.rs:779-785` pushes onto `chain` without bound); do not run at the parent. d15 is L15's.
- **Flips:** `default_classes.rs:276`; eight `defaults.rs` unit tests; AC33/AC35 keep passing if `UnknownClass`'s Display carries the name (A3 §3).
- **Census:** corpus model changes from the reordering "are 0 by these scans (not measured by running a branch)" (A3 §4).
- **Note:** the ledger-L26 guarantee is structural — no loop revisits an entry; a timeout does not bound RAM (A3 §3).

### L17 · M12 · `feat(sim-mjcf): explicit values survive their class`
- **Closes:** mjcf-S11.
- **Implements:** A3 §3's `Option` fields (sensor defaults stay allowlisted, 11-settled #4); a field L05 already typed (`gear`, `dynprm`, `gainprm`, `biasprm`) keeps L05's `Option<Prefix<N>>`. A3-O4 (negative muscle parameters refused except the `force` rule), A3-O1 (refuse an element whose class actuator default came from a different shortcut kind) and A3-O5 (cylinder `bias[1..3]` refused when non-zero), all 11-settled; this commit owns the actuator default fields.
- **Must-fail:** s11_kp, s11_area, s11_adhesion_gain, s11_muscle2, s11_sensor_noise (A3 §3). `s11_gear` is L05's.
- **Flips:** compile-level in `defaults.rs:1273-1298,1407-1446` and `parser/tests.rs` (A3 §3); for A3-O5, the example `actuators/cylinder/src/main.rs:46` needs rewriting (A20 §1A.2).
- **Census:** cylinder `bias` 2 docs become refusals (A20 §1A.2).

### L18 · M13 · `feat(sim-mjcf): default geom size, element-wise overlay, size must be positive`
- **Closes:** mjcf-S3. **Implements:** A3 §3: default size 0; the overlay through L05's `over_prefix` (`size` is L05's `Option<Prefix<3>>`); `checksize`. This commit is the one owner of MuJoCo's size check (`user_objects.cc:152-167`), plane `size(3)` included.
- **Must-fail:** s3_01, s3_02, ov_class_size_chain, s3_08, s3_03, s3_05, s3_12 (A3 §3); A5 §3.5's geom-size cases `sphere_size_neg`, `box_size_one_zero`, `capsule_no_size_fromto`, `geom_no_size_moving`, `default_geom_no_size_in_class`; A4 §8.4's plane `size(3)` cases. Whether s3_02 and ov_class_size_chain already pass at this commit's parent (after L05's `over_prefix`) was not checked; any that do move to L05.
- **Flips:** compile-level only; 0 behavioural flips in the static corpus (A3 §3).

### L19 · M16 · `feat(sim-mjcf): joint layout as MuJoCo`
- **Closes:** mjcf-H3; ball-then-slide refused (01-decisions consequences). **Implements:** A5 §3.3 (`check_joint_layout` before `build()`).
- **Must-fail:** the 12 `Err` cases and 5 `Ok` cases of A5 §3.3 (main before Rigid: 8 panic, 4 load; since P11 all 12 load, chunk 2b's review).
- **Flips:** none (A5 §3.3). The sim-core side (C8) is Rigid-physics' (20-rigid-physics.md).

### L20 · M17 + A14 + A19 · `feat(sim-mjcf): joint axis too small is an error; ball and free axes are (0,0,1)`
- **Merged:** A14 §5's M17 addition and A19 §6's builder row — force ball/free `jnt_axis = (0,0,1)` before the "axis too small" check.
- **Closes:** mjcf-S9 (zero axis → Z).
- **Must-fail:** the 7 `Err` cases; `axis_ball_zero`, `axis_free_zero` (A5 §3.2); X7 `jnt_axis == (0,0,1)` and `<joint type="ball" axis="0 0 0"/>` loads (A14 §3.3).
- **Flips:** none (A5 §3.2).

### L21 · M18 · `feat(sim-mjcf): one limit helper for joints, tendons, actuators`
- **Closes:** mjcf-H4, mjcf-S6, P-L1; U01 (a limit sensor on an unlimited joint loads; MuJoCo refuses).
- **Implements:** A5 §3.4 with 11-settled A5-Q2 (`lo == hi` refused), A5-Q3 (MuJoCo's stored range), A5-Q6 (MuJoCo `<muscle>` gets no forced limit; Hill/Millard keep (0,1)). **U01** (parity): `jointlimitfrc` on an unlimited joint and `tendonlimitfrc` on an unlimited tendon are refused with MuJoCo's "joint must be limited in sensor" / "tendon must be limited in sensor" (`user_objects.cc:7442-7468`), checked on the `limited` this commit resolves.
- **Must-fail:** A5 §3.4's list; `limit_sensor_on_unlimited_joint_refused` (main loads; A6 §5.1).
- **Flips:** `builder/joint.rs:661-683`; `phase7_spec_a.rs:465`, `:496`; `builder/compiler.rs:516` (A5 §3.4); U01: `mjcf_sensors.rs:220` `test_joint_limit_frc_sensor` (its joint has no range) gets a `range`. `ball_joint_limits.rs:485` and `:1213`, which A5 §3.4 lists here, and validator `joint-limits/stress-test` check 10 flipped at P10.
- **Also** (ledger-L69): an automatic ball range `(0, hi)` with `hi < 0` loads limited (the helper must require `range[0] < range[1]` for a ball's automatic limit too; MuJoCo leaves it unlimited, measured); a range of `0 0` is no range since chunk 2b (`has_range`).
- **Census:** `bc9d11e0` (free joint `limited`) flips; muscle `actlimited` is one of three causes of the muscle cluster (A20 §1A.2); A6 §5.1's one limit-sensor doc moves to both-refuse.

### L22 · M19 · `feat(sim-mjcf): sizes, masses and inertias as MuJoCo` — cross-crate (sim-urdf: one comment)
- **Closes:** mjcf-S7.
- **Implements:** A5 §3.5 (the mass pipeline in MuJoCo's order, before flex bodies; `mass < 0` instead of `≤ 0`) and A5-Q7 (URDF triangle violations refused; fix the two example inertias; 11-settled). The geom size check is L18's; `<inertial>` `pos`/`mass` required and the sim-urdf `pos` fix are L05's. sim-urdf: the comment at `urdf/src/validation.rs:233-236` (the triangle inequality is "intentionally NOT enforced") becomes false for loading and is rewritten (A5 §3.5).
- **Must-fail:** A5 §3.5's named cases and its `Ok` cases, except the geom-size cases (L18's) and `inertial_no_pos`, `inertial_no_mass` (L05's).
- **Flips:** `r4_micro_spike.rs:30`, `spatial_tendons.rs` `MODEL_C`/`D`/`I`, `fluid_forces.rs` `MODEL_O`/`Z`, `builder/compiler.rs:611`, `builder/mass.rs:328-345`, `sensor_phase6.rs:1543-1547`, `fluid_derivatives.rs:1391-1393`, `builder/mod.rs:1247-1250`, `raycast_heightfield.rs:14`; validators `free-joint/stress-test:29`, `urdf-loading/stress-test:283`; examples that are not validators: `free-joint/spinning-toss/src/main.rs:40`, `free-joint/tumble/src/main.rs:46`, `urdf-loading/inertia/src/main.rs:40` and `:64`, `raycasting/heightfield/src/main.rs:65` (A5 §3.5; R2).
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
- **Must-fail:** `condim_checked_as_mujoco`: geom condim 0, 2 (through a default class), 5, 7 → "invalid condim in geom" (`user_objects.cc:3650-3654`; measured on MuJoCo, A5 §3.8), and a pair condim 2 → "invalid condim in contact pair" (`:5666-5669`, by reading). Main warns and rounds. **Flips:** none (no test pins the rounding) (A5 §3.8).

### L26 · M22 part · `fix(sim-mjcf): an empty <contact/> appends; repeated pairs are both kept`
- **Closes:** mjcf-D2 (empty `<contact/>`); A5-Q10 (11-settled).
- **Must-fail:** `empty_contact_appends` (pair then `<contact/>` → npair 1; two blocks → 2) and `repeated_pairs_both_kept` (npair 2, condims [3, 6]) (A5 §3.8, measured on MuJoCo). Main: the empty element replaces the block; the repeated pair is last-wins.
- **Flips:** `builder/contact.rs:261` `pair_dedup_last_wins_under_canonical_key` (11-settled A5-Q10). **Census:** the A5-Q10 doc (A16 §2).

### L27 · M22 part · `fix(sim-mjcf): .mjb decoding is size-limited`
- **Closes:** mjcf-D2 (`.mjb` size); 11-settled #13 (`MjbTooLarge`).
- **Implements:** A5 §3.8 (`MJB_DECODE_LIMIT = 1 << 28`, `with_limit`) — "Not measured (no crafted file was decoded)".
- **Must-fail:** `mjb_claimed_length_over_limit_is_refused`: a `.mjb` whose length-prefixed `String`/`Vec<u8>` claims `MJB_DECODE_LIMIT + 1` bytes → `MjbTooLarge` (R2). At the parent bincode allocates the claimed length before reading (A5 §3.8, by reading), about 256 MiB: run only under the 2.5 GB RSS watchdog (40-verification). Run locally with `--features mjb` (A6 §4(e)).

### L28 · M24 · `fix(sim-mjcf): timestep > 1 loads`
- **Closes:** P-L24, sim-mjcf half (drop the `> 1` rule; keep ≤ 0 / non-finite as stricter).
- **Must-fail:** `timestep="1.5"` loads (A5 §3.9). **Flips:** `validation.rs:862-867`.

### L29 · A9 F1 · `fix(sim-mjcf): dim-3 flex elements oriented as MuJoCo`
- **Closes:** the known divergence "dim-3 tet re-orientation" (01-decisions).
- **Implements:** A9 §1.3 (`orient_tetrahedra`) and the unsigned `flexelem_volume0` (Q110; A9 Q1).
- **Must-fail:** `flex_tet_reoriented_as_mujoco`, `flex_tets_all_outward_after_build`, `flexelem_volume0_is_unsigned` (A9 §1.5).
- **Flips:** `flex_unified::ac3_solid_compression` unless the volume is unsigned (then 0) (A9 §1.6).

### L30 · A9 F2 · `fix(sim-mjcf): flex flaps as MuJoCo — dim 2 only, boundary [opp, -1]`
- **Closes:** the known divergence "flex boundary flaps" (01-decisions).
- **Must-fail:** A9 §2.6 #1 and #3. **Flips:** 46 docs model-only, 0 trajectory, 0 tests (A9 §2.7).

### L31 · A13 D3, loading half · `feat(sim-mjcf): flex edge constraints only through a flex equality, as MuJoCo`
- **The core rule is Rigid-physics'** (R5; A13 §3.6 measured `ISO_FLEX=1`, every flex treated as requesting edges, with 0 test failures): `EqualityType::Flex`, `flexedge_invweight0`, `flex_edgeequality`, the removed `flex_edge_solref`/`solimp` and `ConstraintType::FlexEdge` → `Equality` (Q36) land in the physics commit carrying L31's core rule (chapter 20, P36a). This commit is the loading half.
- **Closes:** ledger-L39c (A13 §3) = census ledger-L44c (A16 §3), with that physics commit.
- **Implements:** A13 §3.5's sim-mjcf part: parse `<equality><flex flex=…/>` (C2) and `<flexcomp><edge equality solref solimp>` (`true` creates the equality); edge equalities only for a flex an equality names; refuse `equality="vert"` and `<equality><flexvert>` (stated limitation, A13 Q5); delete `compute_edge_solref`. Doc rewrites, on L10's vertex-body docs: `<equality><flex flex="…"/>` wherever a test needs edges to hold, at least `ac5`, `ac20`, `t09` (Q37, A13 Q4; which others was not determined).
- **Must-fail:** `flex_edge_rows_match_mujoco_3_5_0` (A13 §3.7), on L10's declared-body form; `flex_edge_rows_only_where_the_doc_asks`: the same fixture without `<equality><flex>` → 0 flex edge rows. Parent: P36a's builder gives every flex an equality, so it has rows.
- **Flips:** without the rewrites, `flex_unified::ac5_edge_constraint_stiffness`, `ac20_bending_stability_clamp`, `flex_flex_collision::t09_full_forward_step_no_panic` (A13 §3.6, `ISO_FLEX=2`).
- **Census:** on L10's rewritten flex docs, not measured. "Item 3's MJCF side … was not prototyped" (A13 §7).
- **Breaking:** behaviour: a flex no equality names has no edge rows. The Model changes are the physics commit's.

### L32 · A9 F3 · `feat(sim-mjcf): flex elastic2d — bending only when asked, as MuJoCo`
- **Closes:** A9 item 2b; DT-86 rows; U12 (stated limitation).
- **Implements:** A9 §2.4 2b (`FlexElastic2d`; bending only for `bend`/`both`; `stretch`/`both` refused as a limitation; refusals per A9 Q4, Q7; pins + bending load per A9 Q3) — C2 implements `elastic2d` — and the 39 `elastic2d="bend"` rewrites, applied to L10's vertex-body docs. **U12:** sim-core has no dim-3 or 2-D membrane elasticity (A9 §7 H6), so a dim-3 flex with `<elasticity young>` > 0 is refused as a stated limitation, as `stretch`/`both` are. Today a dim-3 `young` becomes a bending stiffness (`builder/flex.rs:773`), not MuJoCo's tet elasticity (`user_mesh.cc:4347-4361`).
- **Must-fail:** A9 §2.6 #2, #4, #5, #6, #7; pins #8 (bending-order deviation) and #9 (pins + bending); `flex_dim3_elasticity_is_refused` (U12; main loads).
- **Flips:** with the rewrites, 2 docs model-only, 0 tests; without them, 12 CI tests flip (A9 §2.7). U12: `flex_unified.rs:399` `ac3_solid_compression` — its doc (`:61-77`) is the only in-tree dim-3 flex (`git grep 'dim="3"'`) and sets `young="100"`; drop `young` or make it the refusal test (which keeps its intent was not checked).
- **Note:** the `.mjb` shape changes here; the version was bumped in L01.

### L33 · A10 MH3 · `fix(sim-mjcf): body inertial frame as MuJoCo: one geom copied, orientation alternatives`
- **Closes:** ledger-L43c, census "rotated inertia" (A16 §2; `636fc5d5`, A20 §1A.2).
- **Implements:** A10 §2.4 (MuJoCo's geom selection, one-geom copy, orientation alternatives through `resolve_orientation`, mesh principal frame from unit-density eig3); the fromto part is in L14. `inertiagrouprange` is implemented here (A10 Q2.3; C2).
- **Must-fail:** `single_geom_body_copies_geom_frame`, `geom_euler_rotates_body_inertia` (A10 §2.4).
- **Flips:** `fluid_derivatives.rs:1391` t23 only if before L22 (A10 §7); here it is after.
- **Census:** `636fc5d5` flips (A20 §1A.2); outlier `collision_primitives.rs:1279` moves 0.189 (A10 §2.3).

### L34 · A20 R3-L1 · `fix(sim-mjcf): plane and hfield geoms have MuJoCo's volume; zero-volume mass is zero`
- **Implements:** A20 §1A.3.1 (plane volume 0; hfield the box of its size; an explicit mass with volume ≤ 1e-14 → 0) and **Q46** (parity): a body with no mass gets MuJoCo's inertial pose, its parent-frame pose copied into `ipos`/`iquat` (`user_objects.cc:2443-2446`).
- **Must-fail:** `plane_geom_has_no_mass` (A20 §1A.3.1); `massless_body_inertial_frame_as_mujoco`: a massless body at `pos="1 0 0" euler="0 30 0"` → `ipos` (1, 0, 0), `xipos` (1.866, 0, −0.5) (A20 §1A.3.1, measured on MuJoCo); main `ipos` 0.
- **Flips:** planes: 8 docs `model_fp`, 0 trajectory. Q46: 32 docs `model_fp`; `traj_fp` changes only in the 3 `mass="1e-20"` docs L22 refuses (A20 §1A.3.1).
- **Census:** plane-only body mass 6/6 (A20 §1A.2); Q46 changes no class (measured, A20 §1A.3.1).

### L35 · A10 MH4 · `fix(sim-mjcf): mesh mass properties as MuJoCo (legacy default, centroid apex, exact-mode orientation check)`
- **Implements:** A10 §3 (`mesh_mass_properties`; `MeshInertia` default `Legacy`; exact-mode orientation check). **Q63:** MuJoCo's mesh frame — vertices re-centred at the mesh's centre of mass and rotated to its principal axes (`user_mesh.cc:1676-1686`), and the geom frame moved there, so `geom_pos`/`geom_quat` absorb the mesh's offset (`user_objects.cc:3749-3754`; A17 Q2 (a)). **Q121:** embedded vertices rounded to f32, as MuJoCo (A10 Q3.1). Neither was prototyped (A20 §1A.2).
- **Must-fail:** `t10` → MuJoCo's values; `t1` renamed `t1_default_mode_is_legacy` plus an L-shape with no attribute (mass 5322.55); `exact_mode_refuses_inconsistent_orientation` ("not run"); the L-prism regression (A10 §3); `mesh_geom_frame_as_mujoco`: a mesh whose centre of mass is off its origin → `geom_pos`, `geom_quat` and vertices equal MuJoCo 3.5.0's (main: the authored pose); `embedded_mesh_vertices_are_f32`: `9b0854df`'s tetra within 2 ulps of MuJoCo (A10 §2.3 measured that with the XML pre-rounded to f32; main 2.6e-8 relative).
- **Flips:** `mesh_inertia_modes.rs:241` t10, `:52` t1 (A10 §7); the mesh-frame flips were not counted.
- **Census:** the mesh-frame cluster, 3 docs (`cb2e58ac`, `7b6eff02`, `9b0854df`); trajectories already agree on all 3, `qM` differs in 2 (A20 §1A.2).
- **Downstream** (`git grep` at HEAD):
  - sim-bevy `sim/L1/bevy/src/model_data.rs:675-677` builds a mesh geom's render mesh from `mesh_data` and places it by the geom pose; both change together, so the placement is unchanged by reading (not run).
  - cf-design `mechanism/model_builder.rs` builds its `Model` without sim-mjcf (0 `sim_mjcf` hits) and keeps its own mesh frame. `mechanism/mjcf.rs:914` `the_file_places_geometry_where_the_model_does` compares the exported `pos` with that code-built `Model`, not with a sim-mjcf load, so L35 does not reach it by reading.
  - sim-gpu `pipeline/tests.rs`: no mesh geom (`mesh=`, `type="mesh"`, `GeomType::Mesh`, `mesh_data`: 0 hits); not a reader.
  - cf-design-tests load meshes through sim-mjcf (`l4_physics.rs:128-129`); what they assert about a mesh geom's pose was not checked.
- **Breaking:** the `MeshInertia` default; the meaning of a mesh geom's `geom_pos`/`geom_quat` and of `TriangleMeshData` vertices (Q63, listed as a 0.10 break).

### L36 · A10 MH5 · `fix(sim-mjcf): STL vertices deduplicated as MuJoCo; non-finite vertices refused; left-handed scale; binary STL by size` — cross-crate (mesh-io)
- **Closes:** the known divergence "STL vertex deduplication" (01-decisions); the NaN-STL hang (A10 §1); Q108 (11-settled #14: mesh-io STEP keys sorted).
- **Implements:** A10 §1 (dedupe in sim-mjcf for `.stl` before scale; non-finite vertices refused for every format; left-handed winding swap; `mesh-io` binary detection by size). **Q118** (lenient): ASCII STL loads with its values rounded to f32, and more than 200,000 faces load; MuJoCo's `LoadSTL` refuses both (`user_mesh.cc:1229-1241`). **Q119** (parity): |coord| > 2^30 is refused (`:1258-1261`). **Q108:** mesh-io `step.rs:103` iterates truck's `table.shell` hash map; it iterates in sorted key order instead.
- **Must-fail:** `stl_vertices_deduplicated_in_first_appearance_order`, `stl_negative_zero_merges`, `stl_left_handed_scale_flips_winding`, `binary_stl_with_solid_header_loads` (A10 §1); `stl_nonfinite_vertex_is_error` — main hangs and the mechanism was not isolated (A10 §1): run only under the 2.5 GB RSS watchdog (40-verification); `ascii_stl_equals_binary_stl` (one mesh in both encodings → a bit-identical Model) and a mesh over 200,000 faces equal to the same faces split under the limit (Q118's tests); `stl_coordinate_over_2_pow_30_is_refused` (Q119). Q108: hash order cannot be made to fail on demand; the gate is `#![deny(clippy::iter_over_hash_type)]` in mesh-io, made to fire on `step.rs:103` under `--features step` before the fix (not run).
- **Flips:** none measured; 53 submodule STL files change from error to load (model-level status of their 5 models "not measured") (A10 §1).
- **Hull as sets** (01-decisions, known divergences: the hull is tested as SETS; R3): `stl_hull_matches_mujoco_as_sets` — for meshes picked from the STL files A10 §4.1 measured equal as sets (1,702 of 2,520 submodule STLs, with P03's hull and this commit's dedupe), our referenced hull vertices equal MuJoCo 3.5.0's `mesh_graph` vertices by coordinates, and our faces equal its faces as triples up to rotation, both compared as sets (A10 §4.1, `cmp_hull.py`). Whether it fails at the parent was not checked.
- **Divergence row:** hull vertex and face order, coplanar-facet triangulation, near-coplanar vertex inclusion and `maxhullvert`'s vertex choice differ from qhull's (stated limitation; A10 §4.1, §7; 11-settled #14).
- **Downstream:** the mesh-io change reaches 25 callers outside mesh-io and sim; it is behaviour-only (A10 §1; R2).

### L37 · A10 MH6 · `fix(sim-mjcf): a mesh that needs a hull and has none is an error`
- **Closes:** the flat-mesh item: refused where MuJoCo needs a hull, and the shell without collision that MuJoCo loads, loads (Q103; A10 §5 found no test that can show our collision on a flat mesh right).
- **Must-fail:** `flat_mesh_with_collision_is_refused`, `flat_shell_mesh_without_collision_loads`, `flat_mesh_legacy_volume_too_small` (A10 §5).
- **Flips:** `mesh_inertia_modes.rs:339` t16 (rewrite with `contype="0" conaffinity="0"`); `exactmeshinertia.rs:354` ac7 (rewrite as a refusal) (A10 §5).
- **Note:** re-measure the verdict on top of P40 — A15 found its EPA defect makes box, cylinder and ellipsoid fall through convex mesh slabs, and whether it also caused the planar-hull fall-through "was not measured" (A15 §4.4, §8).

### L38 · M23 · `feat(sim-mjcf): a vertex-only mesh gets its convex hull`
- **Closes:** mjcf-D1 (11-settled #13); U14 (parity): MuJoCo refuses fewer than 4 vertices for every mesh, with faces or without ("at least 4 vertices required", `user_mesh.cc:1806-1808`).
- **Must-fail:** a 6-vertex hull loads with nonzero mass; 3 vertices and 4 coplanar vertices → `Err` (A5 §3.8); `mesh_with_faces_and_3_vertices_is_refused` (U14; main "not measured", A10 §6 item 9 — if main already refuses it, it is a pin).
- **Flips:** none (the 4 corpus docs are parse-only tests) (A5 §3.8). **Census:** the 4 vertex-only `ours-refused` docs (A20 §2.6).

### L39 · A20 R3-L2 · `fix(sim-mjcf): discardvisual keeps the discarded geoms' inertia; sim-urdf emits no visual geoms` — cross-crate (sim-urdf)
- **Implements:** A20 §1A.3.2; the converter change goes in the same commit, or URDF links without `<inertial>` gain their visual mass (16.52 kg vs 0.5236 kg measured).
- **Must-fail:** the `5f4d4961` and `b6ad1797` literals (A20 §1A.3.2). Check the `urdf-loading/*` validators.
- **Census:** 2/2 (A20 §1A.2).

### L40 · A20 R3-L3 · `fix(sim-mjcf): ellipsoid shell inertia as MuJoCo`
- **Must-fail:** tighten `mesh_inertia_modes.rs:195` t8 to 1e-9 relative (fails on main by 1.1 %) (A20 §1A.3.3). **Census:** 1/1.

### L41 · A18 L1 + A20 R3-L4 · `fix(sim-mjcf): weld relpose as MuJoCo; parse torquescale`
- **Merged:** both sections propose the same relpose rule (A18 §8.1; A20 §1A.3.4). Parsing `torquescale` completes P39 (A18 §13) — C2.
- **Must-fail:** `weld_relpose_as_mujoco` / the `496f2186` literal (A18 §10; A20 §1A.3.4); `weld_torquescale_matches_mujoco` in its XML form: `weld_ts_0.5.xml`, 500 steps, 1e-12 (A18 §10). The parent ignores the attribute and writes 1.0 (8.1e-2 off). P39 tests the same rule with `eq_data[10]` set in code (20-rigid-physics.md).
- **Census:** `496f2186`'s model agrees; it then differs from step 33 at a box–box contact (A18 §8.1; A20 §1A.4).

### L42 · A20 R3-L5 · `fix(sim-mjcf): fusestatic accumulates body inertias, moves fromto geoms`
- **Implements:** A20 §1A.3.5 (keeps "fused == unfused"; fixes main's three defects). MuJoCo's frame-order defect is not inherited: wrong-value rule, listed with this commit's test (Q48; A16 Q1).
- **Must-fail:** fused `qacc` == unfused `qacc` to 1e-5 on the four fixtures (main fails by 23.9–37.6) (A20 §1A.3.5).
- **Census:** the 2 corpus docs stay different, marked with this commit's divergence row (A20 §1A.2, §1A.3.5).

### L43 · A20 R3-L6 · `fix(sim-mjcf): <inertial> orientation alternatives` — cross-crate (sim-urdf)
- **Implements:** A20 §1A.3.9: `euler`, `axisangle`, `xyaxes`, `zaxis` resolved as MuJoCo (`user_objects.cc:2419-2424`) and refused with `fullinertia` (`:2404-2406`); sim-urdf already emits `<inertial euler=…>` (`urdf/src/converter.rs:456-458`). C2 implements these. **U10** (parity with MuJoCo's URDF reader): sim-urdf writes `eulerseq="xyz"` (`converter.rs:177`), which reads URDF `rpy` as intrinsic x-y-z; it writes `eulerseq="XYZ"` instead (A20 §1A.3.9 measured agreement to 1e-8). This changes every multi-axis `rpy` the converter emits, not only inertials.
- **Must-fail:** `inertial_euler_rotates_iquat`: A20's `fs_tool` → `body_iquat` (0.98877, 0, 0.14944, 0), MuJoCo's (A20 §1A.3.9); main identity. `urdf_rpy_as_mujoco`: `rpy="0.2 -0.6 1.1"` → `body_quat` (0.79496, 0.23500, −0.20083, 0.52200), MuJoCo's URDF reader (A20 §1A.3.9); main (0.82580, −0.07238, −0.30053, 0.47170).
- **Flips:** the one in-tree URDF with a multi-axis `rpy` is a parser test (`urdf/src/parser.rs:569`); whether it reads the converted orientation was not checked. U10's 3.4e-3 inertia residual after the fix is not isolated (A20 §1A.3.9).

### L44 · A11 LR-a · `fix(sim-mjcf): actuator lengthrange computed and kept as MuJoCo 3.5.0` — cross-layer (sim-core), cross-crate (cf-design)
- **In Rigid-loading although most of it is sim-core:** it needs L01's error variant (A11 §8, Q-LR5). Body: sim-core changes, the breaking `LengthRangeError`, and `compute_actuator_params` no longer sets lengthrange (A11 §8).
- **One commit.** It could split into an additive core (`Model::set_length_range`, `LengthRangeError`, MuJoCo's mode filter; called by nothing) and a switch (the `LengthRangeOpt` default, the builder's call after `builder.build()`, `compute_actuator_params` no longer setting lengthrange) (R5; not measured).
- **Closes:** A5-Q4 (explicit `lengthrange` honoured; 11-settled); ledger-L37 (A16 §2); A6 §5.3's NO-ROW "lengthrange did not converge" — the 4 docs are refused with MuJoCo's message (Q122).
- **Implements:** A11 §2 LR-1 + LR-2 (`LengthRangeOpt` default `uselimit = false`; `Model::set_length_range`; MuJoCo's mode filter; raw `uselimit` copy; called after `builder.build()`). **Q129** (stated limitation): an actuator on a ball or free joint with a non-scalar gear (a nonzero `gear[1..]`) → `Unsupported`. Ours applies `gear[0]` to the first dof and has no ball/free length (`forward/actuation.rs:390-404`); MuJoCo uses the whole gear (`engine_core_smooth.c:1311-1360`; A11 §6 item 1). A ball-joint muscle with a scalar gear is still refused, by "Invalid lengthrange (0, 0)", because its length is 0 in ours (A11 §6 item 1).
- **cf-design (Q127):** `mechanism/model_builder.rs:940` calls `compute_actuator_params`, whose lengthrange side effect goes away; it calls `set_length_range` after it (A11 §5, Q-LR6).
- **Keep the skip:** `build()` skips the derivations on a joint layout or a range `try_make_data` refuses (P11 and chunk 2b's review), because the length-range simulation steps the model and a backwards range panicked in `f64::clamp` while loading; `set_length_range` after `build()` runs under the same check (`a_backwards_range_loads_and_make_data_refuses_it`). `Model::recompute_derived`'s doc table lists `actuator_lengthrange` under `compute_actuator_params`: move it.
- **Must-fail:** T1–T7, T9–T12 (A11 §3); `ball_joint_nonscalar_gear_is_refused` (Q129; main loads).
- **Flips:** `fiber.rs:569`, `:1815`, `:1421`; `builder/actuator.rs:967`; `activation_clamping.rs:213`, `:229`, `:255` (template `:65-85`); `actuator_phase5.rs:193`, `:125`, `:225`; `phase7_spec_a.rs:270` (A11 §4).
- **Census:** ok→err 4 + 1; 9 muscle docs' trajectories now equal MuJoCo's (≤ 5.1e-14); 15 docs change `actuator_lengthrange` only (A11 §4); 4 docs `mj-refuses` → `both-refuse` (A20 §2.10).

### L45 · A17 C6 · `fix(sim-mjcf): height-field data as MuJoCo (rows bottom-to-top, normalised, grid kept)` — cross-layer (sim-core), cross-crate (cf-geometry)
- **Implements:** A17 §11 R3-C sim-mjcf part (f32, rows flipped, MuJoCo normalisation, no resampling; PNG likewise). **Q67:** cf-geometry `HeightFieldData` (`design/cf-geometry/src/heightfield.rs:54`, pub) gains separate x/y spacing, and `new(…, cell_size)` (`:78`) changes with it. Constructors (`git grep`): `collision/flex_collide.rs`, `collision/hfield.rs`, sim-mjcf `builder/mesh.rs`, cf-geometry's own tests (`src/heightfield.rs:425-589`). Readers: `flex_narrow.rs:205`, `sdf_collide.rs:226`, `raycast.rs:298`, `builder/build.rs:582` (A17 §11 R3-C).
- **Must-fail:** `hfield_rows_bottom_to_top`, `hfield_elevation_normalised`, `bodies_rest_on_bumpy_hfield_as_mujoco`, `nonsquare_hfield_as_mujoco` (A17 §11 R3-C tests 1, 2, 4, 6). Test 5, `hfield_contacts_match_mujoco_bitwise`, is P52's, with its field set in code.
- **Flips:** none in the suites run; the raycasting validator's stdout is unchanged (A17 §11 R3-C).
- **Breaking:** `HeightFieldData` (pub, cf-geometry).

### L46 · A8 SP-3/Q3 · `fix(sim-mjcf): sleep-policy compile errors and init-sleep refusal as MuJoCo`
- **Closes:** A8 SP-3's two compile errors ("belong in the MJCF series … with `MjcfError`", A8 Dependencies); the load half of Q27 (A8 Q3 (a), parity); A6 §5.1's NO-ROW "sleep-policy compile check" (2 docs: `fluid_derivatives.rs:2768`, `sleeping.rs:2615`). R4's draft.
- **Now** (lines at `main`; P11 keeps the explicit-policy step in sim-mjcf as `apply_explicit_sleep_policies` and moves the rest of `build.rs:810-956` into sim-core, A1 §7).
  - A non-root `sleep` attribute: `warn!`, then the policy is applied to the tree (`builder/build.rs:934-950`); after P20 it is skipped silently on a static body.
  - An explicit `allowed`/`init` on a tendon-coupled tree: the explicit policy wins.
  - An explicit `auto` is a no-op since chunk 2b's review fixes (`an_explicit_auto_sleep_policy_is_the_default`, MuJoCo's values), so the must-fail `explicit_auto_on_actuated_tree_never_sleeps` below would pass at this entry's parent; before them it became `AutoAllowed`.
  - An init tree that cannot sleep is warned (`island/sleep.rs:300-304`). After P23 `try_make_data` refuses it, but `load_model` does not.
- **Target** (MuJoCo 3.5.0), in this order:
  1. `user/user_model.cc:3036-3048`: a non-auto policy on a body that is not a movable root → "sleep policy only allowed for movable root bodies"; an explicit `auto` is skipped (`:3041`).
  2. `engine/engine_setconst.c:205-245`: for a tendon with `treenum > 2`, or `treenum == 2` and nonzero stiffness or damping, an explicit `ALLOWED`/`INIT` on any tree its wraps touch → the errors at `:234-243`. Limits do not count.
  3. `user_model.cc:5160-5179`: a final `mj_makeData` raises the init-sleep error (`engine_io.c:1473-1493`). A8 measured MuJoCo refusing both `initmix` fixtures at load.
- **Change.**
  - An explicit non-auto policy on a body with no tree, or not its tree's first body (`tree_body_adr[tree] != body_id`, P20's tables) → `MjcfError::InvalidValue { element: "body", attribute: "sleep", reason: "sleep policy only allowed for movable root bodies", at }`, replacing the `warn!`.
  - Rule 2 → `MjcfError::InvalidModel` with MuJoCo's text and our ids, at `Location::none()` (MuJoCo's is an engine error with no element). The trees come from P20's `Model::tendon_trees`.
  - At the end of `model_from_mjcf`, if `ENABLE_SLEEP` is set and any tree is `Init`: `model.try_make_data()`, mapping any `MakeDataError` → `MjcfError::InvalidModel { reason: e.to_string(), at: Location::none() }`; the `Data` is dropped.
  - **P23 supplies** (20-rigid-physics.md): `MakeDataError::InitSleep { marked, slept, tree, root_body }`, which carries MuJoCo's two counts (`engine_io.c:1490-1492`).
- **Must-fail** (not run): `sleep_policy_on_child_body_refused` (main warns); `sleep_policy_on_static_body_refused`; `init_on_damped_cross_tree_tendon_refused` (fixture `fluid_derivatives.rs:2768`; MuJoCo refuses it, A6 §5.1); `allowed_on_three_tree_tendon_refused`; `init_mixed_island_refused_at_load` (A8's `initmix.xml`, `initmix_contact.xml` and the T60 doc); `explicit_auto_on_actuated_tree_never_sleeps` (main `AutoAllowed`). Pins: `explicit_auto_on_child_body_loads`, `init_on_limited_only_cross_tree_tendon_loads`.
- **Flips:** `fluid_derivatives.rs:2990` t56 (rewrite without `sleep="init"`); `:3045` t57 (delete, or make it the refusal test); `sleeping.rs:2612` T60 (its assertion moves to `load_model`). Every other in-tree `sleep=` is on a world child (`git grep`, R4).
- **Downstream:** cf-design sets every tree `Never` (after P11, over `compute_kinematic_trees`, A1 §7(c)); sim-urdf emits no `sleep`.
- **Census:** the 2 docs move `ours-ok/mj-refuses` → `both-refuse` (A6 §5.1 measured MuJoCo's side).
- **Open:** code-built models. Rule 2 is a setconst rule in MuJoCo, so it also refuses a programmatic `Model`; as written, ours refuses only MJCF. R4 recommends also refusing in `try_make_data` (`MakeDataError::SleepPolicy { tree, tendon }`); otherwise it is a divergence row.

### L49 · A11 LR-b · `feat(sim-mjcf): <compiler><lengthrange>`
- **Before L47** (R2, R3, R5): L47's schema then accepts the element, which MuJoCo 3.5.0's schema lists (`xml_native_reader.cc:104-107`), so `parser/tests.rs:553` never becomes a refusal.
- **Implements:** A11 §2 LR-4 (10 attributes; `MjcfCompiler.lengthrange: LengthRangeOpt`); deletes the default-class `lengthrange` plumbing (A11 §8). C2 implements it.
- **Must-fail:** T8; extend `parser/tests.rs:553` (A11 §3, §8).

### L47 · M15 · `feat(sim-mjcf): schema pre-pass at MuJoCo 3.5.0 with the extension allowlist`
- **Closes:** mjcf-S1, mjcf-H2 (depth limit), mjcf-S14; the decisions "unknown attributes and elements are errors, with an allowlist" and "visual-only and capacity-hint elements: accepted with no effect" (01-decisions); 11-settled flex `density` (removed from docs, then refused), A4-Q2/Q3 (allowlist), A4-Q4 (read-tracking sync guard), A4-Q6 (hex floats refused), A4-Q7 (freejoint damping removed), A4-Q9 (weld site form refused), A5-Q5 (`actuatorfrcrange` refused); Q66 (`<flag nativeccd="disable">` refused, stated limitation; A17 Q5).
- **Implements:** A4 §1.3 (generated `mujoco_3_5_0.rs` + golden schema text + sync test; overlay; worldbody and frame rules; include detection; `_` arms become internal errors) and the frame-sensor `objtype`/`objname` requirement with the class B rewrites (A4 §9). The overlay accepts what L31, L32, L33, L41, L43 and L49 read (C2); the sync guard asserts every accepted attribute is read (A4 §1.3). `lengthrange` on a `<default>` actuator is refused: MuJoCo's default actuators have none (`xml_native_reader.cc:184-200`; A11 §8).
- **Three verdicts** (01-decisions): accepted and read; refused; **accepted with no effect** — `camera light material texture visual skin`, `<statistic>` except `meaninertia`, `<size memory njmax nconmax nkey nstack nuserdata>`. These get explicit skip arms (not the `_` internal-error arm) and an exemption in the read-tracking guard, and each has its divergence row. `<statistic meaninertia>` stays refused (A4 §4).
- **Must-fail:** A4 §1.5's list; refusal tests for the 12 dropped sensors and the limitations (A4 §9); one load test per accepted-with-no-effect element (0 corpus docs use them, so only these tests can catch a wrong verdict, R2); `nativeccd_disable_is_refused` (Q66).
- **Flips:** classes B, C, F (A4 §7; class A is L10's); dt16 `parser/tests.rs:2026-2050`; plugin tests t1–t13; `exactmeshinertia.rs` tests become refusals; `adhesion.rs` ac6; `sensor_phase6.rs` pins; `runtime_flags.rs:1385`. Q66 flips nothing in-tree by reading: `collision/mod.rs:1269` and `golden_flags.rs:325` set `DISABLE_NATIVECCD` in code, not through MJCF; a code-built `Model` with that flag is refused by P50 (20-rigid-physics.md).
- **Census:** the corpus re-run must show new refusals ⊆ 3.5.0's refusals ∪ the listed deviations (A4 §9). With A4's recommended allowlist, 171 docs that parse today would be refused before the doc fixes (A4 §3).

### L48 · M25 = A7 DH-3 · `feat(sim-mjcf): delay and history as MuJoCo 3.5.0`
- **Closes:** P-L34, sim-mjcf half.
- **Implements:** A7 H-3: `interval` as `period [phase]`; the 2^24 caps; positive phase and `phase ≤ −period` refused; schema excludes `<user>`/`<plugin>` sensors; stricter rows for negative delay, interval without `nsample`, phase without period; `delay > nsample·timestep` refused with the tolerance `(1 + 1e-12)` (Q19, stricter; A7 Q2: the tolerance is "my choice, no MuJoCo referent").
- **Must-fail:** one test per rule (A7 H-3). The 2^24 cap test (`nsample="16777217"`) allocates about 0.27 GB per actuator and 0.54 GB per dim-3 sensor at the parent if it builds `Data` (R2, arithmetic from A7 §1.1): run only under the 2.5 GB RSS watchdog (40-verification).
- **Flips:** `sensor_phase6_spec_d.rs:39` (add `nsample="1"`), `:413` and `:436` (become refusal tests); under Q19, `actuator_phase5.rs:818`, `:1128`, `sensor_phase6_spec_d.rs:14` (A7 H-3). Until this commit, `integration/history.rs` sets the phase of the sensor models' `interval="0.03 -0.01"` sensor in code (`model_of`, `api_model`); here the parser reads it, and that line goes.
- **Corpus:** 6 MuJoCo-ok docs refused, 3 of them by Q19 (A7 H-3).
- **Breaking:** `MjcfSensor.interval: Option<(f64, f64)>`; `with_interval(period, phase)` (A7 H-3).

### L50 · M26 · `docs: MuJoCo divergences, sim-mjcf docs, conformance status`
- **Implements:** each divergence row a commit introduces arrives with that commit (Conventions); L50 adds the rows of deviations that exist at `main` and stay, enumerated from the research (for example A9 §6's stricter composite `initial`, `builder/composite.rs:127-134`). The rest is prose: the stale "Real-World Model Loading ✅ COMPLETE" (A1 §10); `MUJOCO_CONFORMANCE.md` 3.5.0 vs 3.4.0 references (A1 "Found in passing"; A20 §2.1); the lengthrange docs list (A11 §5); `POST_V1_ROADMAP.md` DT-107/DT-108 (A7 H-4).

## Merged, split, dropped

| change | sections | why |
|---|---|---|
| merged → L03 | A5 M4 + A10 MH2 | "eig3 **is** M4's `principal_axes`" (A10 §7) |
| merged → L14 | A4 M14 geometry + A14 L39b + A20 §1A.3.6 + A10 §2.4 fromto | same function; size and frame must match together (A14 §2.5; A20 §1A.6) |
| merged → L20 | A5 M17 + A14 M17 addition + A19 builder row | same check site (A14 §5; A19 §6) |
| merged → L41 | A18 L1 + A20 R3-L4 | same relpose rule (A18 §8.1; A20 §1A.3.4) |
| split → L10–L14 | A4 commit 6 (M14) | six unrelated forms; L12 must precede L15 (A20 §2.10) and L10's rewrites carry L31's and L32's (R4) |
| split → L25–L27 | A5 commit 9 (M22) | four rules in four files; only `.mjb` needs `--features mjb` (A6 §4(e)) |
| moved | A10 MH3's fromto axis → L14 | MH3 then depends on L14 (A10 §2.3 measured the copy without the fromto change as worse) |
| unplaced → L46 | A8 SP-3 / Q3 MJCF half | A8 assigns it to the MJCF series; R4 wrote the commit |
| moved → P36a (chapter 20) | L31's core rule (A13 §3.5, sim-core) | it is green alone: `ISO_FLEX=1`, 0 failures (A13 §3.6; R5) |
| moved → L05 | the sim-urdf inertial `pos` fix, from L22 | L05 makes `pos` required, so L05 is not green without it (R2) |
| one owner | non-finite → L06; d15 → L15; `checksize`, plane `size(3)` included → L18; `<inertial>` `pos`/`mass` required → L05; the `gear`/`size` types → L05 | each was placed twice (R2) |
| reordered | L49 before L47 | L47 needed L49 and L49 depended on L47 (R2, R3, R5) |

## Conflicts that touch this PR

Full descriptions in 20-rigid-physics.md (C1–C5, C8–C11). Loading-only:

| id | conflict | sections | commits | resolution |
|---|---|---|---|---|
| C2 | A4 §4's limitation list vs implementations: `<compiler><lengthrange>` (A11 LR-4), weld `torquescale` (A18 §8.2), flex `elastic2d` (A9 F3), `inertiagrouprange` (A10 Q2.3), `<equality><flex>` (A13 D3), `<inertial>` orientation alternatives (A20 §1A.3.9) | A4 vs A9, A10, A11, A13, A18, A20 | L31, L32, L33, L41, L43, L47, L49 | implement all six; L47 lands after them |
| C6 | **Principal axes.** `principal_axes` with `Rotation3::from_matrix_unchecked` after the determinant flip (A5 §3.1, Q1; 11-settled A5-Q1) **vs** MuJoCo's `mjuu_eig3`, which "replaces A5 §3.1's `principal_axes` … so A5-Q1 is moot" (A10 §2.4) | A5, A10 | L03 | `eig3`, in plain arithmetic |
| C7 | **Flex attribute form.** "Same meaning, MuJoCo spelling, mechanical rewrite" (A4 Q1; 11-settled A4-Q1) **vs** "not the same meaning": MuJoCo's `<flex body=…>` names existing bodies as vertices, ours creates them; without a decision flex stays outside the census and the gate (A20 §1B.4) | A4, A20 | L10, L31, L32 | MuJoCo's meaning; L10 declares the vertex bodies |
| C1 | FMA (fused `eig3`, A10 Q2.1) | A10 vs A18, A21 | L03, L14 | plain arithmetic; goldens from the unfused oracle (40-verification) |
