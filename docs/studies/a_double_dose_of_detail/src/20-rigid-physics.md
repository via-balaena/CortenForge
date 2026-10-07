# Rigid-physics: the commit series

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


Draft for the stress test. Input: *A Double Dose of Detail* at `caa1c1c0` — Parts 0–3 (`00`–`41`) and research A1–A21 (`src/research/a01…a21`). Nothing here was built or run; every statement points at the section it rests on, and "not measured" / "not isolated" are kept where a section says so. Where a section and Parts 0–3 disagree, Parts 0–3 win (00-how-to-read).

**What this PR holds** (01-decisions, second round): sim-core physics including cf-geometry collision, constraints, dynamics, the sleep and delay runtime, and the census gate. A commit that must also edit sim-mjcf, sim-thermostat or sim-gpu to stay green is marked **cross-layer**, with the section that forces it.

## Conventions

- `P01…P53` is the order in this PR. "src" is the id the book (the earlier single series) or the research section used.
- Titles are the research titles with `!` removed (the commit-msg hook refuses it, the earlier single series); "breaking" is a field instead.
- **Must-fail** = fails at the parent, passes at the commit (the earlier single series). Only tests the research names are listed; "none named" means none.
- **Census** = the gate's `verdicts.tsv` (A20 §2.4). Every commit after P01 carries its re-blessed file, and its improvements are its census flip list (A20 §2.7). Doc counts below are what a section measured, on the base that section names; **the gate count per commit was not measured as a series** — A20 §2.10 measured end states only.
- **Per-commit greenness in this order was not run** by any section (A1 commit list; A2 commit list; A8 commit list; A13 §7; A17 §12; A18 §13; A19 §6). A21 §11 compiled each commit but ran suites at the head only.
- Conflicts between sections are `C1…C11` (end of file). They are listed, not resolved.

## Summary

| # | src | title | breaking | depends on |
|---|---|---|---|---|
| P01 | A20 G1 | `test(sim-conformance): MuJoCo 3.5.0 parity census gate` | no | — |
| P02 | M2 (A6) | `fix(cf-geometry): convex hull no longer depends on hash order` | no | — |
| P03 | A10 MH1 | `fix(cf-geometry): convex hull is convex: exact orientation predicates, no unreferenced vertices` | behaviour | P02 |
| P04 | K1 (A1 #1) | `fix(sim-core): one pre-step check on every Result entry point; thermostat refuses h ≤ 0` | behaviour | — |
| P05 | decision | `fix(sim-core): xfrc_applied in MuJoCo's force-then-torque order, in a new type` | **yes** | P04 |
| P06 | K2 (A2 C-1) | `fix(sim-core): hybrid sensor derivatives run every stage` | no | — |
| P07 | K3 (A1 #2) | `fix(sim-core): implicitspringdamper forward() leaves qvel unchanged` | behaviour | P04 |
| P08 | A13 D2 | `fix(sim-core): implicit integrators keep the explicit qacc; the implicit solve feeds the integrator (MuJoCo mj_implicitSkip)` | behaviour | P07 |
| P09 | K4 (A1 #3) | `fix(sim-core): reset_to_keyframe resets fully; energy_initial captured once per reset` | behaviour | P04 |
| P10 | K5 (A1 #4) | `fix(sim-core): RK4 advances plugins; Model::try_make_data` | additive | — |
| P11 | K6 (A1 #5) | `feat(sim-core): kinematic trees in sim-core; Model::recompute_derived` (cross-layer) | additive | — |
| P12 | K7 (A2 C-2) | `feat(sim-core): callbacks fire in MuJoCo's order and counts` | **yes** (behaviour) | P04, P06 |
| P13 | K8 (A2 C-3) | `feat(sim-core): finite differences refuse RK4 and history` | **yes** | P12 |
| P14 | K9 | `feat(sim-core): sensor derivatives at the current state` | **yes** | P13, P08 |
| P15 | K10 (A2 C-4) | `fix(sim-core): the bad-ctrl check reads the clamped input and leaves ctrl alone` | **yes** (behaviour) | — |
| P16 | K11 (A1 #6) | `feat(sim-core): per-env BatchSim returns errors; model_of, forward_all` (cross-crate) | **yes** | P04 |
| P17 | K12 (A1 #7) | `refactor: delete SimulationConfig, SolverConfig, Gravity` (cross-layer) | **yes** | — |
| P18 | A7 DH-1 | `feat(sim-core): actuator and sensor delays act, as MuJoCo 3.5.0` | **yes** (TryFrom) | P04, P09, P10, P12, P15 |
| P19 | A7 DH-2 | `feat(sim-core): read and initialise history buffers` | additive | P18 |
| P20 | A8 S1 | `fix(sim-core): kinematic trees and automatic sleep policies as MuJoCo` | behaviour | P11, P12 |
| P21 | A8 S2 | `fix(sim-core): dof_length from MuJoCo's body sizes; MuJoCo's sleep tolerance test` | behaviour | P20 |
| P22 | A8 S3 | `fix(sim-core): sleep decided in the advance; re-forward on the sleep step` (cross-layer) | **yes** | P04, P07, P08, P10, P12, P18, P21 |
| P23 | A8 S4 | `fix(sim-core): wake rules and init-sleep as MuJoCo` | behaviour | P22, P10 |
| P24 | K13 (A12) | `fix(sim-core): the velocity product of a multi-joint body uses the earlier joints' velocity (MuJoCo mj_comVel)` (cross-layer) | behaviour | — |
| P25 | A14 L36a | `fix(sim-core): analytic position derivatives follow the origin a body's own joints move` | behaviour | P24 |
| P26 | A13 D1 | `fix(sim-core): a connect's and a weld's impedance come from the norm of their violation (MuJoCo getposdim)` | behaviour | — |
| P27 | A19 R1 | `fix(sim-core): body accumulators match mj_rnePostConstraint` | behaviour | P24, P05 |
| P28 | A19 R2 (+A14 L36b) | `fix(sim-core): forward kinematics as mj_kinematics1 (qpos − qpos0, off-centre ball, xaxis/xanchor)` (cross-layer) | behaviour | P25, P20–P23 |
| P29 | A19 R2b = A20 R3-P2 | `fix(sim-core): spatial tendon spring length resolved at qpos_spring` | behaviour | P28 |
| P30 | A19 R3 | `fix(sim-core): implicit integrators restrict D to M's sparsity (MuJoCo qH/qLU)` | behaviour | P11, P08 |
| P31 | A19 R4 | `fix(sim-core): transition derivatives use MuJoCo's tangent convention for quaternion joints` | behaviour (public fns) | P13, P14, P25 |
| P32 | A19 R5 | `fix(sim-core): RK4 keeps a sleeping tree asleep` | behaviour | P20–P22, P10 |
| P33 | A20 R3-P1 | `fix(sim-core): muscle force stays -1 in the Model, resolved per call as MuJoCo` | behaviour | P01 |
| P34 | A18 P1 | `fix(sim-core): Newton and CG run MuJoCo's primal solver` | **yes** | P08, P07 |
| P35 | A18 P2 | `feat(sim-core): constraint islands solved separately, as MuJoCo` | behaviour | P34, P11 |
| P36 | A18 P3 | `fix(sim-core): a constraint with an all-zero Jacobian adds no rows (MuJoCo mj_addConstraint)` | behaviour | — |
| P37 | A18 P4 | `fix(sim-core): solref and solreffriction format checks as MuJoCo getsolparam` | behaviour | — |
| P38 | A18 P5 | `fix(sim-core): elliptic friction rows take MuJoCo's impratio and friction-ratio R; diagApprox from R` | behaviour | — |
| P39 | A18 P6 | `feat: weld torquescale` (cross-layer) | behaviour | — |
| P40 | A15 F1 | `fix(cf-geometry,sim-core): EPA witness points from the closest face; GJK contact at their midpoint (MuJoCo epaWitness)` | semantics of public fields | — |
| P41 | A21 R1 (O) | `fix(sim-core): contacts carry MuJoCo's geom order (lower mjtGeom first)` | **yes** (contact order/sign) | — |
| P42 | A21 R2 (F) | `fix(sim-core): contact tangent frame as MuJoCo's mju_makeFrame, with the plane–capsule axis hint` | behaviour | — |
| P43 | A21 R3 (I) (+A18 §5) | `fix(sim-core): contacts listed at dist ≤ margin get constraint rows only below margin − gap` | **yes** (`ncon` meaning) | P22 |
| P44 | A21 R4 (P) | `fix(sim-core): sphere–sphere, –capsule, –cylinder, –box ported from MuJoCo` | behaviour | — |
| P45 | A21 R5 (C) | `fix(sim-core): capsule–capsule and capsule–box ported from MuJoCo` | behaviour | — |
| P46 | A21 R6 (B) | `fix(sim-core): box–box ported from MuJoCo with the driver's box–box filter` | behaviour | — |
| P47 | A21 R7 (G) | `fix(cf-geometry): gjk_distance stops on MuJoCo's Frank–Wolfe gap` | semantics of a published fn | — |
| P48 | A17 C1 | `feat(sim-core): MuJoCo's native convex collision (mjc_ccd) ported` | no | — |
| P49 | A17 C2 | `feat(sim-core): mesh polygons and MuJoCo's mesh multi-contact` | no | P48 |
| P50 | A17 C3 (+A21 R8) | `fix(sim-core): convex pairs collide through mjc_Convex as MuJoCo` | behaviour | P48, P49, P03, P40, P41, P42 |
| P51 | A17 C4 | `fix(sim-core): plane–mesh contacts as MuJoCo (midpoint, hull-graph neighbours)` | **yes** if `collide_mesh_plane` is deleted | P50 |
| P52 | A17 C5 | `fix(sim-core): height fields collide as MuJoCo (prism order, native CCD) and are bounded as MuJoCo` | behaviour | P48, P11 |
| P53 | K14 | `docs(sim-core): …` | no | all |

53 commits.

## What fixes the order

Each constraint below is a dependency note from a section; the order above satisfies all of them.

1. **P01 first.** The gate is Rigid-physics' first commit, blessed at `main` (floor 783), and every later commit in both PRs re-blesses `verdicts.tsv` (A20 §2.7, "Commits from this section").
2. **P02 → P03.** Same file; MH1 comes after M2 because M2's gate and expected-change list were measured on the float version (A10 §7). They sit early because the hash-order fix lets the per-commit harness run once instead of ×10 (A6 §4(b)); the 60 flex docs stay nondeterministic until M3 in Rigid-loading (A6 §1.4).
3. **P04 before P07, P09, P16** (book K3/K4/K11 → K1); **P16 needs `check_data_shape`** (A1 commit list #6).
4. **P05 after P04, before P27.** P04's shape check names `xfrc_applied/nbody` (A1 §1); R1's Step 1 swaps the `xfrc_applied` halves, "whichever lands second edits that line" (A19 §1.5).
5. **P06 before P12.** K7 without K2 ships a wrong hybrid `D` with CI green, measured (A2 commit list).
6. **P07 → P08.** K3 edits the same three places D2 edits (A13 §4). This is the "A13 D2 after A1 K3" rule.
7. **P08 before P14, P22, P30, P34, and before DH-3 (Rigid-loading L48).** K9's sensor-derivative goldens must be generated after D2 (A13 §4); a sleep advance helper takes `qacc_implicit` (A13 §4); R3 edits D2's functions (A19 §6); P1 replaces the `newton_solved` flag D2 gates on — "land D2 first" (A18 §14); M25's golden excludes the accelerometer unless D2 precedes it (A13 §4).
8. **P12 → P13 → P14** (book K8 → K7, K9 → K8).
9. **P18 after P04, P09, P10, P12, P15** (A7 §6), and **before P22**: MuJoCo inserts history before `mj_sleep`; if sleep moves into a shared advance, insertion stays first in it (A7 §7). This is "history insertion before sleep".
10. **P20–P23 after P11 and P12; P22 after P04, P07, P10** (A8 commit list, Dependencies). Keep P11 a pure move so its corpus A/B stays identical (A8 Dependencies).
11. **P24 → P25** (A14 §5); **P25 → P28** (non-root offset-ball derivatives need L36a, A19 §6); **P28 → P29** (A20 §1A.6: the test needs both).
12. **P24 → P27; P05 → P27** (A19 §6: same loop; xfrc order).
13. **P32 after P20–P22 and P10** (A19 §6).
14. **P33 alone in physics.** A20 §1A.6: "A11 LR-a if that lands in P; else alone" — LR-a lands in Rigid-loading (L44) because it needs M1's error variant (A11 §8).
15. **P34 → P35; P35 needs P11's tree data** (A18 §14).
16. **P40 before P50** (A21 §11: EPA before R8; A17 §12 keeps A15 F1 for flex–mesh-hull EPA). **P03 before P50** (A17 §12: the port's hill-climb uses hull + adjacency). **P42 before P50** (A17 §12: NEW-FRAME is a dependency). **P41 before P50** (C10).
17. **P43 carries A18 §5's box–plane per-corner rule**: it "must land with L41's zero-distance contacts" (A18 §5.4, §16). This is "L41 inclusion before A18's box–plane corner rule" — here they are one commit.
18. **P48 → P49 → P50 → P51; P52 after P48 and P11** (A17 §12: C3 without C2 turns `mod.rs:1367` and three mesh-collision validator checks red, measured).
19. **"The MJCF harness baseline is taken at the head of the core series"** (the earlier single series); P24 and P26 change trajectories, so both sit in this PR (A13 §4: D1 "belongs in the core series").

## Commits

### P01 · A20 G1 · `test(sim-conformance): MuJoCo 3.5.0 parity census gate`
- **Closes:** the decision "the census becomes a permanent executable gate in Rigid (MuJoCo 3.5.0 per-doc golden data + a ratchet test whose agree count can only rise)" (01-decisions).
- **Implements:** A20 §2.1–§2.7. Snapshot of the 1,584 docs plus manifest, append-only (§2.2). Golden per doc: status only (no message), model fields as the census compares them, excitations e1 and e2, `forward` dumps at steps 0/1/100, 19 checkpoints (§2.1). Tolerance 1e-9 on each doc's maximum (§2.3). `verdicts.tsv` pins the class, not the label; an improvement fails until `CENSUS_BLESS=1`; a regression is never auto-blessed; `divergence=<ID>` must name an ID in the Intentional Divergences table, which needs an ID column added (§2.4). Module `layer_e_census.rs` in `mujoco_conformance`, `serde_json` dev-dependency (§2.5). Generator `sim/L0/tests/scripts/gen_census_golden.py` pinned to `mujoco==3.5.0`, beside the 3.4.0 golden (§2.1). A JSON golden harness needs `serde_json`'s `float_roundtrip` (A7 §8 F4).
- **Must-fail:** the gate was made to fail: main against the 837-floor file reports 61 regressions; `body_mass[1]` perturbations; a dangling divergence ID; a divergence doc that agrees; 12 `ours-refused` docs without notes (A20 §2.4 table).
- **Flips:** none; it compiles and passes at `main` (A20 §2.7, measured on the prototype).
- **Census:** floor **783** (e1+e2, dumps 0/1/100), with `known=`/`divergence=` notes (A20 §2.6–§2.7). The 71 hash-order docs gave the same verdict in 60/60 processes, so no `nondet=` row is needed (A20 §2.4).
- **Open:** checkpoints, class pinning, append-only (A20 "Open for Jon" 6; open-questions).

### P02 · M2 · `fix(cf-geometry): convex hull no longer depends on hash order`
- **Closes:** P-L27, cf-geometry half (10-scope; A6 §1).
- **Implements:** A6 §1.3 (sorted candidates; BFS `visible` as a `Vec`); `#![deny(clippy::iter_over_hash_type)]` at `design/cf-geometry/src/lib.rs:23` (A6 §1.9; 11-settled #14).
- **Must-fail:** `hull_is_identical_across_repeated_calls` (main: 6/3/3 distinct results in 50 calls) (A6 §1.6 #1); the lint gate fails on the original code (A6 §1.9).
- **Flips:** none measured; four validators byte-identical (A6 §1.7). `licensed-gates --run --only cf-design-tests` is mandatory (rung4/rung4b call `convex_hull`) (A6 §1.7, §6).
- **Census:** not measured. Corpus harness: the hull-only build leaves 60 nondeterministic docs (A6 §1.4).
- **Depends:** none (A6 §6: M2 and M3 independent).

### P03 · A10 MH1 · `fix(cf-geometry): convex hull is convex: exact orientation predicates, no unreferenced vertices`
- **Closes:** A10 §4.2 (our hull is not convex on real meshes; worst 5.35 %); no triage row.
- **Implements:** A10 §4.2: `orient3d` for visibility and conflict tests; output keeps face-referenced vertices only; `robust = "1.2"` as a workspace dependency (pure Rust, already in `Cargo.lock` via `spade`).
- **Must-fail:** `hull_contains_every_input_point` (synth8, main 6.6e-2); a seeded near-coplanar property test (its failure rate on main **was not measured**: keep only seeds that fail on main); `hull_has_no_unreferenced_vertices` (A10 §4.2).
- **Flips:** none measured in CI; licensed rung4/rung4b not run (A10 §4.2, §7).
- **Census:** not measured. Corpus: 5 docs change `model_fp` (all `flex_unified.rs`), trajectories ≤ 5e-16; the unreferenced-vertex removal was not run over the corpus (A10 §4.2, §7).
- **Depends:** P02 (A10 §7).

### P04 · K1 · `fix(sim-core): one pre-step check on every Result entry point; thermostat refuses h ≤ 0`
- **Closes:** core-L3, core-D3 (Display), P-L24 core half (the earlier single series K1).
- **Implements:** A1 §1 (`check_step_inputs`, `check_data_shape` over 24 lengths; 11-settled #7) and A1 §2 (`ThermostatError::InvalidTimestep`). `integrate`/`inverse` documented, not changed (A1 §1; but see C3).
- **Must-fail:** `forward_refuses_a_timestep_that_is_not_positive_and_finite`, `step2_at_negative_timestep_does_not_run_time_backwards`, `step_refuses_data_made_by_another_model`, `thermostat_install_refuses_a_negative_timestep`; new API `step_names_the_mismatched_field`, `invalid_timestep_says_what_it_is`, `thermostat_names_the_timestep` (A1 §1–§2).
- **Flips:** `sensor_tests.rs:1325` → rewritten as `test_forward_refuses_an_undersized_sensordata` (A1 §1).
- **Census:** not measured.
- **Breaking:** none at compile time (both enums `#[non_exhaustive]`); FD entry points now return `Err` at h ≤ 0 (A1 §1 Downstream).

### P05 · decision · `fix(sim-core): xfrc_applied in MuJoCo's force-then-torque order, in a new type`
- **Closes:** the second-round decision "`xfrc_applied` → MuJoCo's force-first order AND a new type, so 0.9 code fails to compile" (01-decisions).
- **Implements:** **no research section specifies this change.** Referents only: MuJoCo stores force then torque (`mjdata.h:266`), ours reads `[torque, force]` (`constraint/mod.rs:96-97`) (A8 §9 O2); `cfrc_ext` stays `[torque; force]`, so A19 R1's Step 1 swaps halves (A19 §1.5); `xfrc_applied/nbody` is one of P04's 24 lengths (A1 §1).
- **Must-fail:** none named. **Flips:** not measured. **Census:** the census never sets `xfrc_applied` (A16 §5, ledger-L40).
- **Breaking:** yes, by decision.

### P06 · K2 · `fix(sim-core): hybrid sensor derivatives run every stage`
- **Closes:** P-L33 (10-scope; A2 §7).
- **Implements:** A2 §7: the six `forward_skip(Pos|Vel)` calls in `mjd_transition_hybrid` use `MjStage::None`.
- **Must-fail:** `hybrid_sensor_derivatives_equal_pure_fd` (main fails on `C`, Euler) (A2 §7).
- **Flips:** none measured; `t4_hybrid_matches_fd_sensor_derivatives` still passes (A2 commit list).
- **Census:** the census has no derivative step (A16 §5).
- **Note:** P14 changes sensor-derivative semantics; A2 §7's test was written against today's semantics, and whether it stands unchanged after P14 is not stated (A2 Q4).

### P07 · K3 · `fix(sim-core): implicitspringdamper forward() leaves qvel unchanged`
- **Closes:** core-L6.
- **Implements:** A1 §3, including the `hybrid.rs` refresh removal, comment rewrites and `split_step` `to_bits` tightening (A1 commit list #2).
- **Must-fail:** `implicitspringdamper_forward_does_not_change_the_state` (A1 §3). `split_step.rs:70/:78` → `to_bits` passes on main too (a tightening, not a pin).
- **Flips:** none; the integrators stress-test validator's ImplSpDmp row changes (E_now 0.008872 → 0.005939), 7/7 PASS (A1 §3).

### P08 · A13 D2 · `fix(sim-core): implicit integrators keep the explicit qacc; the implicit solve feeds the integrator (MuJoCo mj_implicitSkip)`
- **Closes:** ledger-L38a (A13 §2); census cluster ledger-L44b, 25 docs (A16 §2).
- **Implements:** A13 §2.4 option B: crate-private `qacc_implicit`; `forward_acc` runs the explicit acceleration when `!newton_solved`; the three derivative sites read `qacc_implicit` (all three needed, measured).
- **Must-fail:** `implicit_accelerometer_matches_mujoco_3_5_0`, `forward_qacc_is_explicit_under_implicit`, `implicit_warmstart_is_explicit`; guards: `derivatives.rs:1615` and two `transition_matrix_harness` tests, which fail if a site is missed (A13 §2.6).
- **Flips:** none; validators `integrator` and `composite` stress-tests stdout identical (A13 §2.5).
- **Census:** 25 API docs → agree, 1 moves to "implicitfast + fluid" (A16 §3). Corpus: trajectory bits of 4 implicit docs with constraint rows (A13 §2.3).
- **Breaking:** behaviour of `data.qacc` under implicit integrators (A13 §2.5 downstream readers).

### P09 · K4 · `fix(sim-core): reset_to_keyframe resets fully; energy_initial captured once per reset`
- **Closes:** core-L2, core-L1.
- **Implements:** A1 §4 (validate → `reset` → copy) and §5 (`pub(crate)` capture flag; 11-settled #6 and "A1 open").
- **Must-fail:** `reset_to_keyframe_is_a_full_reset`, `forward_captures_energy_initial`, `energy_initial_is_captured_once_even_when_zero`, `reset_clears_the_energy_baseline`; pin `reset_to_keyframe_with_a_bad_index_leaves_data_unchanged` (passes on main) (A1 §4–§5).
- **Flips:** none; the integrators stress-test prints `E_0 = -0.000000` (A1 §5).
- **Note:** the `data_reset_field_inventory` size guard cannot see a field that fits in padding (A1 §5).

### P10 · K5 · `fix(sim-core): RK4 advances plugins; Model::try_make_data`
- **Closes:** core-L7.
- **Implements:** A1 §6, plus `make_data` running plugin `reset` (A1 §6 open question → 11-settled #9).
- **Must-fail:** `rk4_advances_plugins_once_per_step`; new API `try_make_data_returns_a_plugin_init_failure` (A1 §6).
- **Flips:** none (A1 §6).
- **Note:** `# Panics` stays for `validate_joint_layout` (A1 §6) — see C8.

### P11 · K6 · `feat(sim-core): kinematic trees in sim-core; Model::recompute_derived` — cross-layer
- **Closes:** core-L5, P-L31 (sleep panic).
- **Implements:** A1 §7's measured patch, with factories and `finalize` calling the full `recompute_derived` (A1 §7(a)). 11-settled #8 also moves geom bounding radii and fixed tendon lengths into it — "not in the measured patch" (A1 §7(b)) — and switches cf-design's tree copy (A1 §7(c), cf-design tests not run). A7 Q5 recommends moving `compute_history_addresses` in too (open-questions). A17 C5 puts hfield bounds into the moved radii function (A17 §11 R3-C).
- **Cross-layer:** sim-mjcf `build()` calls `compute_kinematic_trees` and deletes its three private functions (A1 §7).
- **Must-fail:** `sleep_runs_on_factory_models`, `compute_dof_lengths_sizes_dof_length`; new API `recompute_derived_follows_a_damping_edit` (A1 §7).
- **Flips:** none across the suites run (A1 §7).
- **Census:** not measured. Tree fields identical between old and new builders for all 1,584 docs; `recompute_derived` changes no field on 1,478/1,478 unedited loads (A1 §7).

### P12 · K7 · `feat(sim-core): callbacks fire in MuJoCo's order and counts`
- **Closes:** core-C1 (behaviour), core-C2, core-C3 (the earlier single series K7).
- **Implements:** A2 §1 code, §2 docs, and the thermostat doc lines `component.rs:215-216`, `langevin.rs:644-645`. The two doc sentences that are true only after P15 go in P15 (A2 §2).
- **Must-fail:** `callbacks_fire_in_mujoco_order_and_count`, `passive_callback_fires_on_a_model_without_dofs`, `rk4_reevaluates_the_control_callback_at_each_stage`, `thermostat_reads_its_own_channel_when_another_is_bad`; pin `passive_callback_skipped_when_springs_and_dampers_are_disabled` (A2 §1, §5, commit list).
- **Flips:** none (A2 commit list).
- **Census:** callbacks are outside the census (A16 §5).

### P13 · K8 · `feat(sim-core): finite differences refuse RK4 and history`
- **Closes:** core-C1, FD half (the earlier single series K8).
- **Implements:** A2 §3 (`StepError::UnsupportedIntegrator { integrator }`, 11-settled A2-Q1) and A2 Q2 (the `nhistory > 0` panic becomes `Err`, 11-settled A2-Q2). A2 does not name the variant for the history refusal.
- **Must-fail:** `finite_differences_refuse_rk4` (A2 §3).
- **Flips:** `derivatives.rs:451` (rewrite to the refusal), `derivatives.rs:464` (delete); validator `derivatives/stress-test` check 23 and `README.md:35`; the doc lines A2 §3 lists.

### P14 · K9 · `feat(sim-core): sensor derivatives at the current state`
- **Closes:** the decision "sensor derivatives take MuJoCo's semantics (C at the current state, A2 Q4)" (01-decisions).
- **Implements:** **not specified by research**: "to be specified from A2 §4 at implementation; FD per-column `forward()` goes" (the earlier single series K9). A2 Q4 measured the two semantics; A2 §4 measured that ours re-runs `forward()` per column, which doubles the callback count.
- **Must-fail:** none named.
- **Flips:** `tests/integration/derivatives.rs`, the `sensor-jacobians` and `derivatives/stress-test` examples (01-decisions).

### P15 · K10 · `fix(sim-core): the bad-ctrl check reads the clamped input and leaves ctrl alone`
- **Closes:** core-L4.
- **Implements:** A2 §5 (`actuator_ctrl_input`), plus the CbPassive/CbControl sentences A2 §2 assigns to C-4. P18's delayed read goes inside `actuator_ctrl_input` (A7 H-1 step 4).
- **Must-fail:** `bad_ctrl_check_runs_on_the_clamped_input_and_leaves_ctrl_alone` (A2 §5).
- **Flips:** `runtime_flags.rs:718` `ac29_ctrl_validation`; `thermostat/src/langevin.rs:647` (A2 §5).

### P16 · K11 · `feat(sim-core): per-env BatchSim returns errors; model_of, forward_all` — cross-crate (sim-thermostat)
- **Closes:** core-B3, P-L19, plus the core-B2 doc (A1 §8).
- **Implements:** A1 §8 shape C (11-settled #10).
- **Must-fail (new API):** `per_env_batch_refuses_models_of_different_shapes`, `model_of_returns_each_envs_model_and_forward_all_runs`; thermostat tests ported from `install_per_env`; pin `batch_reset_zeroes_applied_forces` (A1 §8).
- **Flips:** `tests/integration/batch_sim.rs:276-302`; sim-thermostat `stack.rs` impl, tests and docs, `lib.rs:22`, the `install_per_env` mentions in `langevin.rs`; a comment in `rl-baselines` (A1 §8).
- **Note:** the B1 determinism paragraph must cover `forward_all` (A2 §8).

### P17 · K12 · `refactor: delete SimulationConfig, SolverConfig, Gravity` — cross-layer (sim-mjcf)
- **Closes:** core-D4.
- **Implements:** A1 §9 and A3 §5: sim-mjcf's `config.rs`, `ExtendedSolverConfig` and the `sim-types` dependency go in the same commit, or it does not compile (A3 §5). `SimError` goes "if it has no consumer at implementation time (measure)" (11-settled #11).
- **Must-fail:** none; the deletion is checked by compile (A1 §9). **Not compiled** during planning (A1 "What I checked").
- **Flips:** 15 + 5 (+3) tests deleted with their types (A1 §9).

### P18 · A7 DH-1 · `feat(sim-core): actuator and sensor delays act, as MuJoCo 3.5.0`
- **Closes:** P-L34 runtime half (decision "Delay/history (P-L34): implement in Rigid").
- **Implements:** A7 H-1 (crate-private `history.rs`, init in `make_data`/`reset`, delayed read in `actuator_ctrl_input`, sensor extraction and gate, postprocess skip, insertion in `integrate` and RK4) with RK4 sensor samples at the step-start state (option C, A7 Q1 — open), plus A7 Q4 (`InterpolationType: From<i32>` → `TryFrom<i32>`).
- **Must-fail:** `delayed_actuators_match_mujoco_3_5_0`, `delayed_sensors_match_mujoco_3_5_0`, `sensor_history_initial_state_matches_mujoco`, `delayed_sensor_reads_the_value_one_step_earlier`; `interpolated_value_is_not_reclamped_by_cutoff` (made to fail by removing the postprocess skip) (A7 H-1). Golden assets are the one 3.5.0 golden set (A7 H-1).
- **Flips:** none (720 lib tests; the integration modules run) (A7 H-1, §4).
- **Census:** ledger-L34 sensor delay, 1 doc (A16 §2). Corpus: all 1,421 deterministic docs with `nhistory = 0` bit-identical; the 22 with `nhistory > 0` changed or refused (A7 §4).
- **Breaking:** the `TryFrom` change (0 in-tree callers, A7 Q4).

### P19 · A7 DH-2 · `feat(sim-core): read and initialise history buffers`
- **Implements:** A7 H-2 (`HistoryError`, four `Data` methods). In Rigid only if A7 Q3 is answered yes (open-questions).
- **Must-fail:** the four reads/inits against the API golden, and one test per error variant — new API, so they do not compile on main (A7 H-2).

### P20 · A8 S1 · `fix(sim-core): kinematic trees and automatic sleep policies as MuJoCo`
- **Closes:** part of "sleep re-forward and sleep timing: fix in Rigid" (01-decisions); census sleep cluster, 20 docs (A16 §2).
- **Implements:** A8 SP-3, plus the `rne.rs:362` `tree < ntree` guard. The two MuJoCo compile errors of SP-3 go to the MJCF series (Rigid-loading L46).
- **Must-fail:** `static_body_is_not_a_tree`, `box_on_static_body_sleeps_as_mujoco` (A8 SP-3).
- **Flips:** none in the suites run; tree tables change in 240 non-sleep docs, no trajectory change (A8 SP-3).

### P21 · A8 S2 · `fix(sim-core): dof_length from MuJoCo's body sizes; MuJoCo's sleep tolerance test`
- **Must-fail:** `dof_length_matches_mujoco`, `rotational_sleep_matches_mujoco`, `negative_zero_force_blocks_sleep` (A8 SP-2).
- **Flips:** `sleeping.rs:1025`, `:1399`, `:1428`, `:1487`, `:1533` (A8 SP-2).

### P22 · A8 S3 · `fix(sim-core): sleep decided in the advance; re-forward on the sleep step` — cross-layer (sim-mjcf)
- **Closes:** sleep re-forward and sleep timing (01-decisions; A2 Q3).
- **Implements:** A8 SP-1, SP-5, SP-7 (sleeping `qacc` = MuJoCo's stale `qacc_smooth`, A8 Q1), SP-8 (`integrate` → `Result`; see C3), SP-9 (sim-mjcf's `guard_rk4_sleep` deleted; RK4 path — but see P32 and C4). "Splitting S3 further was not tried" (A8 commit list).
- **Must-fail:** `sleep_step_matches_mujoco`, `sleep_step_reforward_fires_both_callbacks`, `sleep_step_sensors_see_zero_velocity`, `sleep_step_matches_mujoco_implicit`, `sleep_wakes_and_sleeps_with_contact` (SP-1); `asleep_on_static_makes_no_contacts`, `sleeping_rows_dropped`, `island_disabled_blocks_sleep_with_constraints`, `friction_rows_make_islands` (SP-5).
- **Flips:** `sleeping.rs:3669` (SP-5); `sleeping.rs:425`, `:2365`, `:3276`, `:5038`, `body_accumulators.rs:366`, `sensors_phase4.rs:629` and validator `sleep-wake/stress-test:285` (SP-7, under A8 Q1 = parity); `sleeping.rs:666` (SP-9).
- **Census:** not measured on the census. All S-items together: 70/70 corpus sleep docs have MuJoCo's transitions (main 41/70); 1,328 non-sleep docs bit-identical (A8 "Result of the prototype").

### P23 · A8 S4 · `fix(sim-core): wake rules and init-sleep as MuJoCo`
- **Implements:** A8 SP-4, SP-6; init trees that cannot sleep are refused through `MakeDataError::InitSleep` from P10's `try_make_data` (A8 Q3). The load-time mapping is Rigid-loading L46.
- **Must-fail:** `wake_on_contact_inherits_countdown`, `user_qvel_wakes_sleeping_tree`, `user_qpos_and_xfrc_wake`, `disabling_sleep_wakes_all` ("not run on either engine"), `init_tree_has_reset_values`, `init_mixed_island_refused` (A8 SP-4, SP-6).
- **Flips:** `fluid_forces.rs:1495` `t47_wind_sleep_qfrc_fluid_zero` (A8 SP-6).

### P24 · K13 · `fix(sim-core): the velocity product of a multi-joint body uses the earlier joints' velocity (MuJoCo mj_comVel)` — cross-layer (sim-gpu)
- **Closes:** P-L32 (decision default "P-L32 in Rigid", 01-decisions).
- **Implements:** A12 §4: forward paths, body accumulators, `mjd_rne_vel`, `mjd_rne_pos`; and `rne.wgsl` stepping the partial velocity per joint (A12 Q1). "The GPU shader gets the same change in that commit (not yet written or run)" (21-multi-joint-bias).
- **Must-fail:** same-body vs split-body `qacc` invariant; MuJoCo 3.5.0 golden `qfrc_bias` on `hinge_hinge_g0` (+ slide+hinge, hinge→ball); implicit integrator vs MuJoCo; guard `validate_analytical_vs_fd` (does not fail on main) (A12 §6).
- **Flips:** none pinned. The forward half alone fails `derivatives::test_pos_deriv_multi_joint_body` (2.6e-2 > 1e-4), so forward and derivatives are one commit; GPU T15c (tolerance 1e-3) fails on a GPU machine unless the shader changes (A12 §5).
- **Census:** L32 cluster, 5 docs → agree (A16 §3). Corpus: 6 docs change trajectory (A12 §5).

### P25 · A14 L36a · `fix(sim-core): analytic position derivatives follow the origin a body's own joints move`
- **Closes:** ledger-L36a; the pre-existing gap of A12 Q2.
- **Must-fail (measured at the K13 parent):** harness cases `hinge_then_slide_root`, `offset_hinge_child`, `hinge_then_offset_hinge`, `chain3_offset`; MJCF derivative tests F1 and F5; factory `multi_joint_body` (A14 §1.6). Placement of the harness cases: open (A14 Q1).
- **Flips:** none (A14 §1.5). **Census:** none (derivatives).
- **Note:** kept separate from P24 because family B also hits single-joint bodies (A14 §5).

### P26 · A13 D1 · `fix(sim-core): a connect's and a weld's impedance come from the norm of their violation (MuJoCo getposdim)`
- **Closes:** ledger-L39a (A13); census cluster ledger-L44a (A16 §2).
- **Must-fail:** `connect_impedance_is_shared_across_rows`, `weld_matches_mujoco_3_5_0`; the cable check promoted against MuJoCo's explicit XML (needs M8, or the explicit fixture) (A13 §1.8).
- **Flips:** none; the equality stress-test validator prints changed numbers (A13 §1.7). Bits change in 45 of 55 deterministic connect/weld docs; why the other 10 do not "was not examined" (A13 §1.6).
- **Census:** +24 docs → agree, 3 move to an L41 label (A16 §3).

### P27 · A19 R1 · `fix(sim-core): body accumulators match mj_rnePostConstraint`
- **Closes:** census NEW-ACCEL (A16 §2); the `cfrc_ext` findings of A19 §1.3.
- **Implements:** A19 §1.5 (free-joint term through one helper shared with `rne.rs`; `cfrc_ext` at `xpos`; connect/weld forces; `xfrc_applied` torque moved to `xpos`). Static-body readings and `cfrc_int[0]` stay ours (A19 Q1.1, Q1.2 — open).
- **Must-fail:** `accelerometer_free_body_matches_mujoco_3_5_0`, `force_sensor_includes_connect_force`, `torque_sensor_contact_lever_at_origin`, `xfrc_torque_moved_to_origin`; deviation pin `static_site_reads_gravity` (passes on main) (A19 §1.7).
- **Flips:** none (A19 §1.8).
- **Census:** 7 of 8 NEW-ACCEL docs → agree, plus `29661ba2` and `72212e25` under e2; 0 regressions (A19 §1.6).

### P28 · A19 R2 · `fix(sim-core): forward kinematics as mj_kinematics1 (qpos − qpos0, off-centre ball, xaxis/xanchor)` — cross-layer (sim-gpu)
- **Closes:** ledger-L45 FK half = census L45ref (A16 §2); A14 L36b core part, which R2 takes over (A19 §2.3); A11's unexplained `muscle_qpos0_ref` residual (A19 §2.4).
- **Implements:** A19 §2.3: FK, ball velocity and subspace, the sleep FK copy, `fk.wgsl` subtracting `qpos0`; an offset ball on the GPU either implemented or refused (A19 Q2.1). **Conflicts with A14 Q2** (C5).
- **Must-fail:** `hinge_ref_fk_matches_mujoco`, `slide_ref_fk`, `offset_ball_matches_mujoco` (the chain case needs P25), and A14's X1/X3/X5 `xaxis`/`xanchor` literals (A19 §2.6; A14 §3.3).
- **Flips:** none; validator `joint_limits` prints 5 changed values (A19 §2.5).
- **Census:** L45ref 4 docs + 5 more → agree; 0 regressions (A19 §2.5). Corpus: 9 docs with nonzero `ref` change (A19 §2.5).

### P29 · A19 R2b = A20 R3-P2 · `fix(sim-core): spatial tendon spring length resolved at qpos_spring`
- **Merged:** the same change, proposed by both sections (A19 §2.3 last bullet; A20 §1A.3.7).
- **Must-fail:** `spatial_lengthspring_at_springref` (A19 §2.6 #5); the `tendon-limits` doc gives `tendon_lengthspring == 0.7071067811865476` (A20 §1A.3.7).
- **Census:** `251c5904` (A19 §2.5); 2/2 with the FK fix (A20 §1A.2).

### P30 · A19 R3 · `fix(sim-core): implicit integrators restrict D to M's sparsity (MuJoCo qH/qLU)`
- **Closes:** census NEW-IFLUID (A16 §2).
- **Implements:** A19 §4.3; `dof_simplenum` (or `body_simple`) becomes a derived field in `recompute_derived`.
- **Must-fail:** `implicitfast_fluid_matches_mujoco_3_5_0`, `implicit_cross_branch_tendon_damping` (A19 §4.3).
- **Flips:** none. **Census:** `d58bb68a` → agree, 0 regressions (A19 §4.3).
- **Note:** the simple-DOF flags equal MuJoCo's only after MH2 (Rigid-loading L03) and mesh re-centring, which no commit owns (A19 §4.3; A20 "Open for Jon" 4).

### P31 · A19 R4 · `fix(sim-core): transition derivatives use MuJoCo's tangent convention for quaternion joints`
- **Closes:** A14 §4.3 (ball/free position block of `A`).
- **Must-fail:** `ball_transition_matches_mujoco_fd`, `differentiate_pos_tiny_rotation`, `quat_integrate_jacobians` (A19 §3.5).
- **Flips:** none; the `derivatives` validator prints one changed value (A19 §3.4).
- **Breaking:** values of the public `mjd_quat_integrate` and `mj_differentiate_pos` (A19 §3.3, Q3.1).

### P32 · A19 R5 · `fix(sim-core): RK4 keeps a sleeping tree asleep`
- **Closes:** census "sleep off under RK4" (A16 §2; `d22fcd34`, A20 §1A.2).
- **Implements:** A19 §5.2 on top of P22, as a lenient deviation (A19 §5.4). **Conflicts with A8 Q2** (C4).
- **Must-fail:** `rk4_sleeping_tree_stays_asleep`, `sleeping_pose_equals_fk_of_qpos`, `rk4_sleep_step_does_not_disturb_awake_sensors`; optional `rk4_sleep_matches_mujoco_without_its_wake_defects` (A19 §5.4).
- **Flips:** A8's prototype suite gave identical per-test outcomes with the switch on and off (A19 §5.3).
- **Census:** not measured (prototyped on A8's workspace, A19 §8); the gate needs a divergence mark (A19 §6).

### P33 · A20 R3-P1 · `fix(sim-core): muscle force stays -1 in the Model, resolved per call as MuJoCo`
- **Closes:** A20 §1A.3.8; A11 §6 item 4.
- **Must-fail:** none named. Measured fixture: `<general gaintype="muscle" …>`, MuJoCo `qfrc_actuator` −6.482484378125003, main +0.69 (A20 §1A.3.8).
- **Census:** the muscle cluster (9 docs) flips only with A5-Q6 (L21), A11 LR-a (L44) and this together; alone: 6 `model_fp` changes, 0 trajectory changes on the muscle-shortcut docs (A20 §1A.2, §1A.3.8).
- **Note:** `git grep` readers of `actuator_gainprm[i][2]` before implementing (A20 §1A.3.8).

### P34 · A18 P1 · `fix(sim-core): Newton and CG run MuJoCo's primal solver`
- **Closes:** census Newton (9 docs) and CG (ledger-L44) clusters (A16 §2; A18 §1).
- **Implements:** A18 §4 (one primal solver for Newton and CG; no PGS fallback; `SolverStat` in MuJoCo's shape; `newton_solved` crate-private per A18 Q2); un-ignore the 24 `golden_flags` tests; `docs/KNOWN_GAPS.md` Gap 1 (A18 §13).
- **Must-fail:** the 24 `golden_flags` tests; `chol_update_minus_removes_row`, `newton_matches_mujoco_3_5_0_iterates`, `newton_has_no_pgs_fallback`, `cg_matches_mujoco_3_5_0`, `cg_keeps_its_iterate`, `solver_niter_cleared_without_rows` (A18 §10).
- **Flips:** `flex_unified.rs:2269` `spec_a_t4_…_bit_identical` (re-capture or tolerance); the GPU-oracle assertion on `newton_solved` (`test_fixtures/conformance.rs:827`, `:937`) (A18 §11–§13).
- **Census:** 6 of the 9 Newton docs agree (≤ 8.1e-15), 3 move to other clusters (A18 §2.5); `443824ba` agrees (A18 §3.3). 263 corpus docs change bits (A18 §13).
- **Breaking:** `SolverStat` fields; `Data::newton_solved` leaves the public API (A18 §4, Q2).

### P35 · A18 P2 · `feat(sim-core): constraint islands solved separately, as MuJoCo`
- **Implements:** A18 §4 islands (efc-based partition, island scale). In Rigid per A18 Q1 (open-questions). The partition source vs the sleep code's `mj_island` is unresolved (A18 §14; open-questions).
- **Must-fail:** `islands_iterate_as_mujoco` (A18 §10).
- **Flips:** the equality-constraints validator prints `solver_niter` 2 (A18 §11).
- **Census:** 0 verdicts change relative to P34; 50 docs change bits (A18 §4, §13).

### P36 · A18 P3 · `fix(sim-core): a constraint with an all-zero Jacobian adds no rows (MuJoCo mj_addConstraint)`
- **Closes:** census NEW-EQSTATIC, 5 API docs (A16 §2).
- **Must-fail:** `empty_equality_adds_no_rows` (A18 §10).
- **Census:** 5 of 5 agree; 0 trajectory bits change (A18 §7).
- **Note:** also drops zero-Jacobian flex edge rows, which interacts with D3 (Rigid-loading L31) (A18 §14).

### P37 · A18 P4 · `fix(sim-core): solref and solreffriction format checks as MuJoCo getsolparam`
- **Closes:** census `<pair solreffriction>` doc `d81df1fa` (A16 §2; A18 §6.1).
- **Must-fail:** `mixed_solreffriction_uses_solref`, `mixed_solref_uses_default` (A18 §10).
- **Flips:** `unified_solvers.rs:1164-1204` `test_s31…` (rewrite); validator `derivatives/stress-test` check 30 — change its fixture to the direct format in this commit (A18 §11).

### P38 · A18 P5 · `fix(sim-core): elliptic friction rows take MuJoCo's impratio and friction-ratio R; diagApprox from R`
- **Closes:** census doc `2cb92700` (A18 §6.2).
- **Must-fail:** `elliptic_friction_rows_as_mujoco`, `diag_approx_as_mujoco` (A18 §10).
- **Census:** 2 corpus docs change bits (A18 §6.2).

### P39 · A18 P6 · `feat: weld torquescale` — cross-layer (sim-mjcf builder)
- **Implements:** A18 §8.2: core scaling of the rotational rows, and the builder writing `eq_data[10] = 1.0` by default in the **same** commit — otherwise every MJCF weld loses its rotational rows. Parsing the attribute is Rigid-loading L41. **Conflicts with A4 §4/§8.6** (C2).
- **Must-fail:** `weld_torquescale_matches_mujoco` (A18 §10). **Census:** 0 corpus docs change bits (A18 §8.2).

### P40 · A15 F1 · `fix(cf-geometry,sim-core): EPA witness points from the closest face; GJK contact at their midpoint (MuJoCo epaWitness)`
- **Closes:** ledger-L42a (A15 §7).
- **Must-fail:** `ellipsoid_rests_on_box_as_mujoco`, `cylinder_rests_on_box_as_mujoco`, `box_cylinder_ellipsoid_rest_on_convex_mesh_slab_as_mujoco`, `epa_contact_point_is_between_the_two_surfaces_in_both_orders`, `epa_witness_difference_is_depth_times_normal` (A15 §6 #2, #3, #4, #6, #7).
- **Flips:** none in the suites run (A15 §5); the mesh-collision validator's F section improves.
- **Census:** measured only together with F2: 1 doc → agree, 2 move from position to frame (A16 §3). **F1 alone: not measured.**
- **Breaking:** semantics of `cf_geometry::Penetration::{point_a, point_b}` and `GjkContact::point` (A15 §4).
- **Note:** still needed after P50 for flex–mesh-hull EPA and cf-geometry's own callers (A17 §11 R3-A). Re-measure the flat-mesh verdict (Rigid-loading L37) on top of it (A15 §4.4, §8).

### P41 · A21 R1 (O) · `fix(sim-core): contacts carry MuJoCo's geom order (lower mjtGeom first)`
- **Closes:** A15 Q6.
- **Must-fail:** `o_sphere_box_order` (A21 §7.2).
- **Flips:** `collision_primitives.rs:340`, `:558`, `:923`, `:968` (A21 §7.2).
- **Census:** 0 class changes (the census matches contacts order-free); 26 corpus docs change bits (A21 §7.2, §3).
- **Breaking:** contact geom order and normal sign (A21 §11). See C10 for P50.

### P42 · A21 R2 (F) · `fix(sim-core): contact tangent frame as MuJoCo's mju_makeFrame, with the plane–capsule axis hint`
- **Closes:** census NEW-FRAME (28 docs) in part (A16 §2): the other 10 trace to box–box FMA, solver termination and broadphase (A21 §9).
- **Must-fail:** `f_sphere_sphere_x`, `f_plane_capsule` (A21 §4).
- **Census:** +18 (13 by the default rule, 5 by the hint) (A21 §2). The stored normal is not renormalised (A21 §4) — C9.

### P43 · A21 R3 (I) + A18 §5 · `fix(sim-core): contacts listed at dist ≤ margin get constraint rows only below margin − gap`
- **Merged:** A18 §5's box–plane per-corner rule (A21 §8 ports `mjc_PlaneBox` in I; A18 §5.4: the rule "should land with or after L41's zero-distance fix").
- **Closes:** ledger-L41 (census L41 14 docs, L41list 30, incl. "box–plane corners at dist +0.35") (A16 §2); A8 §9 O1 (zero-distance contacts); A18 §5 late onset.
- **Must-fail:** `i_sphere_touching_plane`, `i_box_plane_gap`, `i_static_pair` (A21 §8); A18's plane–box test on `830708de` (A18 §10).
- **Flips:** `cg_solver.rs:307` `test_cg_single_contact_direct` (A21 §8).
- **Census:** L41list 29/30, L41 3/14, NEW-LATE +3 (A21 §2). On A18's base the box–plane rule alone brought 5 of 6 late-onset docs, `a0ff89f4`, `1a42c10c`, `74908604` to agree; 18 corpus docs change bits (A18 §5.3).
- **Breaking:** `ncon` now counts excluded contacts; `Contact::is_excluded` is additive (A21 §11). The `contacts_*` helpers' meaning is open (A21 Q4).
- **Note:** excluded contacts no longer join islands; the sleep census docs were not re-checked against a sleep fix (A21 §11).

### P44 · A21 R4 (P) · `fix(sim-core): sphere–sphere, –capsule, –cylinder, –box ported from MuJoCo`
- **Closes:** census NEW-POS and NEW-DEGEN (A16 §2); A15 Q3 (sphere–box).
- **Must-fail:** `p_concentric_spheres`, `p_sphere_in_box`, `p_sphere_on_cylinder_axis`, `p_sphere_box_shallow`, `p_cylinder_sphere_side` (A21 §5).
- **Flips:** `pair_convex.rs:442`, `:651`; `pair_cylinder.rs:925`, `:1030` (A21 §5).
- **Census:** NEW-POS 8/8, NEW-DEGEN 14/18; `mocap-bodies/tilt-drop` also agrees (A21 §2).

### P45 · A21 R5 (C) · `fix(sim-core): capsule–capsule and capsule–box ported from MuJoCo`
- **Supersedes A15 F2** (A21 §6: C deletes the lines F2 fixes; A21 §11: "A15 F2 dropped or before R5"). A15's capsule tests `capsule_rests_on_box_as_mujoco` and `capsule_box_normal_points_from_geom1_to_geom2_and_pos_is_the_midpoint` move here; A15's 7 regression tests pass with all of A21's commits (A21 §6).
- **Closes:** A15 Q2; census NEW-PAIRCOUNT (A16 §2).
- **Must-fail:** `c_collinear_capsules`, `c_crossing_capsules`, `c_capsule_box_edge` (A21 §6).
- **Flips:** `collision_primitives.rs:504` `capsule_capsule_endpoint_endpoint` (A21 §6).
- **Census:** NEW-DEGEN +2, NEW-PAIRCOUNT 1; `f8879911` matches MuJoCo's single contact at t0 (A21 §2, §6).

### P46 · A21 R6 (B) · `fix(sim-core): box–box ported from MuJoCo with the driver's box–box filter`
- **Must-fail:** `b_box_box_random` (A21 §10.1).
- **Census:** +2 (one L41, one RK4MOCAP `equality_constraints.rs:1232`); 16 docs change bits (A21 §2, §3). The filter can empty a deep overlap (A21 Q7, open-questions).

### P47 · A21 R7 (G) · `fix(cf-geometry): gjk_distance stops on MuJoCo's Frank–Wolfe gap`
- **Closes:** A15 Q5 (with P50).
- **Must-fail:** `gjk_distance_box_ellipsoid_near_touching`, `g_ellipsoid_box_margin` (A21 §7.1).
- **Census:** 0; no corpus doc changes (A21 §3).
- **Breaking:** semantics of the published `gjk_distance` (A21 §7.1).
- **Note:** P50 deletes the margin-zone branch in `narrow.rs` that G feeds (A17 §11 R3-A); G still serves distance sensors, `closest_point.rs` and the mesh margin zone (A21 §7.1 Downstream, §11).

### P48 · A17 C1 · `feat(sim-core): MuJoCo's native convex collision (mjc_ccd) ported`
- **Implements:** A17 §2.1, unused by dispatch; multiply-adds written as `f64::mul_add` where the reference build contracts (A17 §2.2, Q1) — C1.
- **Must-fail:** `mjc_ccd_matches_mujoco_bitwise` (new function; made to fail by a 1-ulp change and by the unfused build) (A17 §11 test 1).

### P49 · A17 C2 · `feat(sim-core): mesh polygons and MuJoCo's mesh multi-contact`
- **Implements:** A17 §4.2 / Q4 — **not prototyped**. Must-fail: a box-on-mesh MULTICCD test vs MuJoCo (A17 §12).

### P50 · A17 C3 + A21 R8 · `fix(sim-core): convex pairs collide through mjc_Convex as MuJoCo`
- **Merged:** A21 R8 (capsule–cylinder through the convex solver). A21 Q3 recommends landing R8 "together with a port of MuJoCo's native CCD"; C3 routes capsule–cylinder through `mjc_Convex` and deletes `collide_cylinder_capsule` (A17 §11 R3-A).
- **Closes:** census NEW-CONVEX (A16 §2); A15 Q1 MULTICCD perturbation (A17 §4.1); A17 item 4 (tumbling ellipsoid).
- **Must-fail:** `capsule_cylinder_deep_matches_mujoco`, `capsule_cylinder_coaxial_tie_matches_mujoco`, `multiccd_cylinder_on_box_as_mujoco`, `multiccd_skips_ellipsoids`, `ellipsoid3_tumble_rests_as_mujoco`, `convex_contact_at_exact_touch_is_absent` (A17 §11 tests 2–7); `y_capsule_cylinder_parallel` (A21 §10.1).
- **Flips:** `collision_primitives.rs:1090` `cylinder_capsule_perpendicular` (asserts at `:1124`) → MuJoCo's depth; `core/src/collision/mod.rs:1535`, `:1602`, `:1731` (A17 §10); `pair_cylinder.rs:1086` (A21 §10.2). Without P49, `mod.rs:1367` and the mesh-collision validator's checks 6, 16, 17 are red (A17 §10).
- **Census:** `3a13c90a` state → API, `6369216c` API → agree; agree 796 → 797 on A17's base (A17 §9).
- **Note:** the A21 Y measurements used cf-geometry's EPA; this merge routes the pair through the port instead (A21 Q3 option (c)). See C10 on the normal's sign.

### P51 · A17 C4 · `fix(sim-core): plane–mesh contacts as MuJoCo (midpoint, hull-graph neighbours)`
- **Closes:** census NEW-MESHPLANE (A16 §2).
- **Must-fail:** `mesh_plane_contacts_at_midpoint` (A17 §11 R3-B).
- **Limitation row:** tie order comes from our hull graph, not qhull's (A17 R3-B; 01-decisions hull order).
- **Breaking:** if `collide_mesh_plane` (pub) is deleted, as recommended (A17 R3-B).

### P52 · A17 C5 · `fix(sim-core): height fields collide as MuJoCo (prism order, native CCD) and are bounded as MuJoCo`
- **Implements:** A17 §11 R3-C sim-core part; bounds in P11's `recompute_derived` (absorbs A17 C7). The data side is Rigid-loading L45.
- **Must-fail:** `hfield_bounds_are_centred`, `hfield_contacts_match_mujoco_bitwise` (A17 §11 R3-C tests 3, 5).
- **Note:** the bumpy-field fall-through is fixed by the bounds alone (A17 §5.1); bodies fall through until this lands (A17 §12).

### P53 · K14 · `docs(sim-core): …`
- **Closes:** core-C1, C4, B1, B2, D1, D2, D3, P-L25, P-L31 FD `B` (the earlier single series K14).
- **Implements:** A1 §10; A2 C-5 (FD callback counts, setters replace, BatchSim determinism; doctest; pin `a_ctrl_writing_control_callback_zeroes_fd_b`) (A2 §4, §6, §8); A7 H-4 sim-core part; A8's K14 notes (the CbPassive line, `island/sleep.rs` module docs) (A8 Dependencies); `docs/KNOWN_GAPS.md` Gap 1 if not in P34 (A18 §16); divergence rows from A1 §10, A2 §4, A7 H-4, A19 §6, and A21 Q1/Q5/Q7 if adopted. The Intentional Divergences table needs its ID column by P01, whose `divergence=` notes name IDs (A20 §2.4, §2.7).

## Merged, split, dropped

| change | sections | why |
|---|---|---|
| merged → P29 | A19 R2b, A20 R3-P2 | the same change (A19 §2.3; A20 §1A.3.7) |
| merged → P50 | A21 R8 into A17 C3 | A21 Q3 (c): land with the CCD port; C3 routes the pair through it (A17 §11 R3-A) |
| merged → P43 | A18 §5 box–plane into A21 R3 | must land with L41's zero-distance contacts (A18 §5.4, §16) |
| merged → P28 | A14 L36b core into A19 R2 | "absorbs A14's L36b core commit" (A19 §2.3) |
| merged → P13 | A2 Q2 into K8 | "Convert to an `Err` in C-3 (same check function)" (A2 Q2) |
| merged → P53 | A1 commit 8, A2 C-5 into K14 | A1 commit list #8 |
| dropped | A15 F2 | superseded by A21 R5 (A21 §6, §11); its tests move to P45 |
| absorbed | A17 C7 into P11/P52 | C7 exists "only if K6 does not take geom bounds" (A17 §12); 11-settled #8 moves geom radii into `recompute_derived` |
| kept separate | A14 L36a from K13 | family B hits single-joint bodies, a different defect (A14 §5) |
| not split | A8 S3, A21 R3 | splitting S3 "was not tried" and the island guard needs the filters (A8 commit list); R3's two halves must land together (A18 §5.4) |

## Census trajectory measured by the research (end states, not per commit)

| base → variant | e1 agree | source |
|---|---|---|
| main → census prototypes (L32, L44a/b/c, L42) | 795 → 850 | A16 §3 (A16's rules) |
| gate rules: main / census prototypes / every Part 1 prototype | 783 / 837 / 1,012 | A20 §2.10 (e1+e2, dumps 0/1/100) |
| census base → all of A21's commits | 850 → 931 | A21 §2 |
| A18's base → every A18 fix incl. plane–box | 847 → 867 | A18 §1 |
| A19's base → all A19 switches | 847 → 859 | A19 Summary |
| A17's fix copy → port | 796 → 797 | A17 §9 |

None of these was measured in this PR's order. Every section reports 0 docs moving from agree to differ under its own changes (A16 §3, A17 §9, A18 §1, A19 Summary, A20 §1A.1, A21 §2).

## Conflicts that touch this PR

Listed, not resolved. Each is also in open-questions.md as class (c).

| id | conflict | sections | commits |
|---|---|---|---|
| C1 | **FMA.** Emulate the reference build's contraction with `f64::mul_add` (A17 §2.2, Q1: bit-identical to the arm64 wheel; A10 §2.3, Q2.1: fused `eig3` bit-exact) **vs** accept roundoff differences and do not emulate (A18 §3.4, Q6; A21 §1.3, Q2: MuJoCo's own C compiled without contraction equals the ports). A17 §7 also hands off an integrator ulp: MuJoCo's `qvel` update equals `fma(qacc, h, qvel)`. | A10, A17, A18, A21 | P34, P41–P50, Rigid-loading L03 |
| C2 | **A4 §4's limitation list vs sections that implement items on it:** weld `torquescale` (A4 §4, §8.6 refuse; A13 §1.9 "refused or implemented"; A18 §8.2 implement), `<compiler><lengthrange>` (A11 LR-4), flex `elastic2d` (A9 F3), compiler `inertiagrouprange` (A10 Q2.3), `<equality><flex>` (A13 D3), `<inertial>` `axisangle xyaxes zaxis euler` (A20 §1A.3.9). | A4 vs A9, A10, A11, A13, A18, A20 | P39; Rigid-loading L31, L32, L33, L41, L43, L47, L49 |
| C3 | **`Data::integrate` signature.** Documented, unchanged (A1 §1, open question (a); 11-settled #7); A13 Q2 picks option B partly because it "keeps `integrate()` returning `()`, as K1 assumes" **vs** `-> Result<(), StepError>` because the sleep step's re-forward can fail (A8 SP-8, Q4). | A1, A8, A13 | P04, P08, P22 |
| C4 | **RK4 + sleep.** Parity: MuJoCo's 584 sleep/wake transitions (A8 Q2 (a)) **vs** a lenient deviation that keeps the tree asleep, which "replaces" A8's option (a) (A19 §5.4). | A8, A19 | P22, P32 |
| C5 | **Ball joint `pos ≠ 0`.** Refuse as a stated limitation, ledger row to implement (A14 Q2) **vs** implement on the CPU, refuse or implement on the GPU (A19 §2.3, Q2.1). | A14, A19 | P28 |
| C8 | **sim-core joint-layout check.** `# Panics` stays for `validate_joint_layout` (A1 §6) **vs** make it fallible, `check_joint_layout -> Result<(), JointLayoutError>`, returned by `try_make_data` (A5 §3.3). | A1, A5 | P10 |
| C9 | **Stored contact normal.** Not renormalised; recommend leave and list (A21 §4, Q5) **vs** renormalisation through `mju_makeFrame` is needed for bitwise hfield/convex normals and is listed as a dependency on NEW-FRAME (A17 §2.3, §12). | A17, A21 | P42, P50, P52 |
| C10 | **Contact geom order inside the CCD port.** A17's dispatch negates the normal to keep today's index order, "A15 Q6 stays open" (A17 §2.1) **vs** A21 R1 adopts MuJoCo's type order (A21 §7.2). An interaction rather than a disagreement: with P41 first, P50's negation is no longer wanted; neither section specifies that edit. | A17, A21 | P41, P50 |
| C11 | **Capsule–box.** A15 F2's two-line fix vs A21 R5's port. Resolved by A21 itself ("A15's F2 dropped or before R5", A21 §11); listed because the brief named it. | A15, A21 | P45 |

C6 and C7 touch Rigid-loading only (see 22-rigid-loading.md).
