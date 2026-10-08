# Rigid-physics: the commit series

> **Conflicts resolved (2026-10-07).** These are the answers to conflicts C1–C11 (the table at the end of this file records each one and its carrier), and every inline mention of a C-number means this:
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
> **Specified after the stress test, from R4's drafts:** P05 (the `xfrc_applied` type) and P14 (sensor derivatives at the current state). **Not measured:** no section ran these commits in this order; per-commit census counts are estimates from separate bases.


Input: *A Double Dose of Detail* at `caa1c1c0` — Parts 0–3 (`00`–`41`) and research A1–A21 (`src/research/a01…a21`), revised after stress-test round 1 (reviewers R1–R5, 2026-10-07). Nothing here was built or run; every statement points at the section it rests on, and "not measured" / "not isolated" are kept where a section says so. Where a section and Parts 0–3 disagree, Parts 0–3 win (00-how-to-read).

**What this PR holds** (01-decisions, second round): sim-core physics including cf-geometry collision, constraints, dynamics, the sleep and delay runtime, and the census gate. A commit that must also edit sim-mjcf, sim-thermostat or sim-gpu to stay green is marked **cross-layer** (another crate: **cross-crate**), with the section that forces it.

## Conventions

- `P01…P53` is the order in this PR; a letter suffix (`P08a`) is a commit added after the stress test, at the position it belongs. "src" is the id the earlier series used (`src/20-commit-series.md` at `caa1c1c0`: the K…/M… ids) or the research section used.
- Titles are the research titles with `!` removed (the commit-msg hook refuses it); "breaking" is a field instead. One scope per title; a commit that edits another crate says so in its body ("cross-crate", "cross-layer").
- **`file:line`** is where the research found the code; the commits before an entry move lines. Re-locate each by its content (the name it carries) before using it.
- **Must-fail** = fails at the parent, passes at the commit (40-verification). Only tests the research or the stress test names are listed; "none named" means none.
- **Census** = the gate's `verdicts.tsv` (A20 §2.4). Every commit after P01 carries its re-blessed file, and its improvements are its census flip list (A20 §2.7). Doc counts below are what a section measured, on the base that section names; **the gate count per commit was not measured as a series** — A20 §2.10 measured end states only.
- **Divergence rows:** each commit adds, in the same commit, the rows of the divergence registry (P01) that its `verdicts.tsv` notes name and that its own deviations need. P53 collects prose only.
- **Checks:** besides its own crates, every commit runs `cargo test -p sim-gpu`: sim-gpu's GPU-vs-CPU suite compares against sim-core's CPU pipeline (`gpu/src/pipeline/conformance_tests.rs`), and CI runs it on lavapipe (`.github/workflows/quality-gate.yml:600-606`).
- **Mesh pairs** (R5 S6): MuJoCo stores a mesh re-centred at its centre of mass, rotated to its principal axes, in f32, and moves the geom frame there (A17 §8 item 1, §12); our loader does that only from Rigid-loading L35. A test here that compares a mesh pair with an oracle golden (P49's box-on-mesh test, any mesh row of P48's table) writes that frame into the `Model` in code after load — the mesh vertices, `geom_pos` and `geom_quat` from the oracle's compiled model — as P52's test writes `hfield_data`.
- **Bit-exact and ≤ 1e-12 goldens** (A17 §11, A21 §10.1 and the rest) come from the unfused oracle named in 40-verification § The unfused oracle (C1), not from the arm64 wheel; a value a section measured against the wheel is re-generated there.
- **Residual owners:** a commit named as owner in 40-verification's residual table also isolates those docs: it fixes them, or records the first differing quantity and the divergence row.
- **Per-commit greenness in this order was not run** by any section (A1 commit list; A2 commit list; A8 commit list; A13 §7; A17 §12; A18 §13; A19 §6). A21 §11 compiled each commit but ran suites at the head only.
- Conflicts between sections are `C1…C11`, resolved in the header; the table at the end of this file gives each resolution and its carrier. A17's own commit ids are written `A17 C1`…`A17 C7`.

## Summary

| # | src | title | breaking | depends on |
|---|---|---|---|---|
| P01 | A20 G1 | `test(sim-conformance): MuJoCo 3.5.0 parity census gate` | no | — |
| P02 | M2 (A6) | `fix(cf-geometry): convex hull no longer depends on hash order` | no | — |
| P03 | A10 MH1 | `fix(cf-geometry): convex hull is convex: exact orientation predicates, no unreferenced vertices` | behaviour | P02 |
| P04 | K1 (A1 #1) | `fix(sim-core): one pre-step check on every Result entry point; thermostat refuses h ≤ 0` | behaviour | — |
| P05 | decision (R4 draft) | `fix(sim-core): xfrc_applied in MuJoCo's force-then-torque order, in a new type` (cross-crate) | **yes** | P04 |
| P06 | K2 (A2 C-1) | `fix(sim-core): hybrid sensor derivatives run every stage` | no | — |
| P07 | K3 (A1 #2) | `fix(sim-core): implicitspringdamper forward() leaves qvel unchanged` | behaviour | P04 |
| P08 | A13 D2 | `fix(sim-core): implicit integrators keep the explicit qacc; the implicit solve feeds the integrator (MuJoCo mj_implicitSkip)` | behaviour | P07 |
| P08a | U02 (A7 §8 F2) | `fix(sim-core): step2 under RK4 takes MuJoCo's Euler step, eulerdamp included` | behaviour | P08 |
| P09 | K4 (A1 #3) | `fix(sim-core): reset_to_keyframe resets fully; energy_initial captured once per reset` | behaviour | P04 |
| P10 | K5 (A1 #4) + C8 + Q17 | `fix(sim-core): RK4 advances plugins; Model::try_make_data` | **yes** (`validate_joint_layout` deleted) | — |
| P11 | K6 (A1 #5) + Q17 | `feat(sim-core): kinematic trees in sim-core; Model::recompute_derived` (cross-layer) | additive | P10 |
| P12 | K7 (A2 C-2) | `feat(sim-core): callbacks fire in MuJoCo's order and counts` (cross-crate) | **yes** (behaviour) | P04, P06 |
| P13 | K8 (A2 C-3) | `feat(sim-core): finite differences refuse RK4 and history` | **yes** | P12 |
| P14 | K9 (R4 draft) | `feat(sim-core): sensor derivatives at the current state` | **yes** | P13, P06 |
| P15 | K10 (A2 C-4) | `fix(sim-core): the bad-ctrl check reads the clamped input and leaves ctrl alone` (cross-crate) | **yes** (behaviour) | — |
| P16 | K11 (A1 #6) | `feat(sim-core): per-env BatchSim returns errors; model_of, forward_all` (cross-crate) | **yes** | P04, P05 |
| P17 | K12 (A1 #7) | `refactor: delete SimulationConfig, SolverConfig, Gravity` (cross-layer, cross-crate) | **yes** | — |
| P18 | A7 DH-1 | `feat(sim-core): actuator and sensor delays act, as MuJoCo 3.5.0` (cross-layer) | **yes** (TryFrom) | P04, P09, P10, P11, P12, P15 |
| P19 | A7 DH-2 | `feat(sim-core): read and initialise history buffers` | additive | P18 |
| P20 | A8 S1 | `fix(sim-core): kinematic trees and automatic sleep policies as MuJoCo` | behaviour | P11, P12 |
| P21 | A8 S2 | `fix(sim-core): dof_length from MuJoCo's body sizes; MuJoCo's sleep tolerance test` | behaviour | P20, P05 |
| P22 | A8 S3 | `fix(sim-core): sleep decided in the advance; re-forward on the sleep step` (cross-layer) | **yes** | P04, P07, P08, P10, P12, P18, P21 |
| P23 | A8 S4 | `fix(sim-core): wake rules and init-sleep as MuJoCo` | **yes** (`SleepError` deleted) | P22, P10, P18, P05 |
| P24 | K13 (A12) | `fix(sim-core): the velocity product of a multi-joint body uses the earlier joints' velocity (MuJoCo mj_comVel)` (cross-layer) | behaviour | — |
| P25 | A14 L36a | `fix(sim-core): analytic position derivatives follow the origin a body's own joints move` | behaviour | P24 |
| P26 | A13 D1 | `fix(sim-core): a connect's and a weld's impedance come from the norm of their violation (MuJoCo getposdim)` | behaviour | — |
| P27 | A19 R1 | `fix(sim-core): body accumulators match mj_rnePostConstraint` | behaviour | P24, P05 |
| P28 | A19 R2 (+A14 L36b) | `fix(sim-core): forward kinematics as mj_kinematics1 (qpos − qpos0, off-centre ball, xaxis/xanchor)` (cross-layer) | behaviour | P25, P20–P23 |
| P29 | A19 R2b = A20 R3-P2 | `fix(sim-core): spatial tendon spring length resolved at qpos_spring` | behaviour | P28 |
| P30 | A19 R3 | `fix(sim-core): implicit integrators restrict D to M's sparsity (MuJoCo qH/qLU)` | behaviour | P11, P08 |
| P31 | A19 R4 | `fix(sim-core): transition derivatives use MuJoCo's tangent convention for quaternion joints` | behaviour (public fns) | P13, P14, P25 |
| P32 | A19 R5 | `fix(sim-core): RK4 keeps a sleeping tree asleep` | behaviour | P20–P22, P10 |
| P33 | A20 R3-P1 | `fix(sim-core): muscle force stays -1 in the Model, resolved per call as MuJoCo` (cross-crate) | behaviour | P01 |
| P34 | A18 P1 | `fix(sim-core): Newton and CG run MuJoCo's primal solver` | **yes** | P08, P07 |
| P35 | A18 P2 | `feat(sim-core): constraint islands solved separately, as MuJoCo` | behaviour | P34, P11 |
| P36 | A18 P3 | `fix(sim-core): a constraint with an all-zero Jacobian adds no rows (MuJoCo mj_addConstraint)` | behaviour | — |
| P36a | A13 D3 core (from Rigid-loading L31) | `feat(sim-core): flex edge rows only through a flex equality, as MuJoCo` (cross-layer) | **yes** | P11, P30, P36 |
| P37 | A18 P4 | `fix(sim-core): solref and solreffriction format checks as MuJoCo getsolparam` | behaviour | — |
| P38 | A18 P5 | `fix(sim-core): elliptic friction rows take MuJoCo's impratio and friction-ratio R; diagApprox from R` | behaviour | — |
| P39 | A18 P6 | `feat: weld torquescale` (cross-layer) | behaviour | — |
| P40 | A15 F1 | `fix(cf-geometry): EPA witness points from the closest face; GJK contact at their midpoint (MuJoCo epaWitness)` (cross-crate) | semantics of public fields | — |
| P41 | A21 R1 (O) | `fix(sim-core): contacts carry MuJoCo's geom order (lower mjtGeom first)` | **yes** (contact order/sign) | — |
| P42 | A21 R2 (F) | `fix(sim-core): contact tangent frame as MuJoCo's mju_makeFrame, with the plane–capsule axis hint` | behaviour | — |
| P43 | A21 R3 (I) (+A18 §5) | `fix(sim-core): contacts listed at dist ≤ margin get constraint rows only below margin − gap` | **yes** (`ncon` meaning) | P22 |
| P43a | Q69 (A21 Q1) | `fix(sim-core): broadphase as MuJoCo's mj_broadphase, float bounds included` | behaviour | P43 |
| P44 | A21 R4 (P) | `fix(sim-core): sphere–sphere, –capsule, –cylinder, –box ported from MuJoCo` | behaviour | — |
| P45 | A21 R5 (C) | `fix(sim-core): capsule–capsule and capsule–box ported from MuJoCo` | behaviour | — |
| P46 | A21 R6 (B) | `fix(sim-core): box–box ported from MuJoCo with the driver's box–box filter` | behaviour | — |
| P47 | A21 R7 (G) | `fix(cf-geometry): gjk_distance stops on MuJoCo's Frank–Wolfe gap` | semantics of a published fn | — |
| P48 | A17 C1 | `feat(sim-core): MuJoCo's native convex collision (mjc_ccd) ported` | no | — |
| P49 | A17 C2 | `feat(sim-core): mesh polygons and MuJoCo's mesh multi-contact` | no | P48 |
| P50 | A17 C3 (+A21 R8) | `fix(sim-core): convex pairs collide through mjc_Convex as MuJoCo` | behaviour | P48, P49, P03, P40, P41, P42, P10, P34 |
| P51 | A17 C4 | `fix(sim-core): plane–mesh contacts as MuJoCo (midpoint, hull-graph neighbours)` | **yes** (`collide_mesh_plane` deleted) | P50 |
| P52 | A17 C5 | `fix(sim-core): height fields collide as MuJoCo (prism order, native CCD) and are bounded as MuJoCo` | behaviour | P48, P11, P42 |
| P53 | K14 | `docs(sim-core): state, callbacks, finite differences and BatchSim documented as they behave` | no | all |

56 commits (53 + P08a, P36a, P43a).

## What fixes the order

Each constraint below is a dependency note from a section; the order above satisfies all of them.

1. **P01 first.** The gate is Rigid-physics' first commit, blessed at `main` (floor: see P01), and every later commit in both PRs re-blesses `verdicts.tsv` (A20 §2.7, "Commits from this section").
2. **P02 → P03.** Same file; MH1 comes after M2 because M2's gate and expected-change list were measured on the float version (A10 §7). The corpus harness runs ×10 at every Rigid-physics commit: the hull fix alone leaves 60 flex docs nondeterministic until Rigid-loading L02 (A6 §1.4, §6).
3. **P04 before P07, P09, P16** (K3/K4/K11 → K1); **P16 needs `check_data_shape`** (A1 commit list #6).
4. **P05 after P04, before P16, P21, P23, P27.** P04's shape check names `xfrc_applied/nbody` (A1 §1); those four commits' tests write the field in the new type (R4); R1's Step 1 swaps the `xfrc_applied` halves, "whichever lands second edits that line" (A19 §1.5).
5. **P06 before P12.** K7 without K2 ships a wrong hybrid `D` with CI green, measured (A2 commit list).
6. **P07 → P08.** K3 edits the same three places D2 edits (A13 §4). This is the "A13 D2 after A1 K3" rule.
7. **P08 before P08a, P22, P30, P34, and before DH-3 (Rigid-loading L48).** P08a edits the same `match` in `integrate/mod.rs` (U02); a sleep advance helper takes `qacc_implicit` (A13 §4); R3 edits D2's functions (A19 §6); P1 replaces the `newton_solved` flag D2 gates on — "land D2 first" (A18 §14); M25's golden excludes the accelerometer unless D2 precedes it (A13 §4).
8. **P12 → P13 → P14** (K8 → K7, K9 → K8); **P06 → P14** (P14 keeps P06's `MjStage::None`, R4).
9. **P18 after P04, P09, P10, P12, P15** (A7 §6), and **before P22**: MuJoCo inserts history before `mj_sleep`; if sleep moves into a shared advance, insertion stays first in it (A7 §7). This is "history insertion before sleep". **P10, P11 → P18:** P18's refusals sit in `try_make_data` and its history addresses in `recompute_derived`. **P18 → P23:** P23's init-sleep refusal goes through P18's `try_reset`.
10. **P10 → P11** (`recompute_derived` runs P10's range check, Q17). **P20–P23 after P11 and P12; P22 after P04, P07, P10** (A8 commit list, Dependencies). Keep P11 a pure move so its corpus A/B stays identical (A8 Dependencies).
11. **P24 → P25** (A14 §5); **P25 → P28** (non-root offset-ball derivatives need L36a, A19 §6); **P28 → P29** (A20 §1A.6: the test needs both).
12. **P24 → P27; P05 → P27** (A19 §6: same loop; xfrc order).
13. **P32 after P20–P22 and P10** (A19 §6).
14. **P33 alone in physics.** A20 §1A.6: "A11 LR-a if that lands in P; else alone" — LR-a lands in Rigid-loading (L44) because it needs M1's error variant (A11 §8).
15. **P34 → P35; P35 needs P11's tree data** (A18 §14).
16. **P40 before P50** (A21 §11: EPA before R8; A17 §12 keeps A15 F1 for flex–mesh-hull EPA). **P03 before P50** (A17 §12: the port's hill-climb uses hull + adjacency). **P42 before P50** (A17 §12: NEW-FRAME is a dependency). **P41 before P50** (C10).
17. **P43 carries A18 §5's box–plane per-corner rule**: it "must land with L41's zero-distance contacts" (A18 §5.4, §16). That L41 is ledger-L41 (P43's own zero-distance rule), not Rigid-loading L41; here they are one commit. **P43 → P43a**, for the census, not the code: two of P43a's five docs (`1efe8000`, `eaace9f8`) differ only in the contact list once P43 is in (A21 §9).
18. **P48 → P49 → P50 → P51; P52 after P48, P11 and P42** (A17 §12: A17 C3 without A17 C2 turns `mod.rs:1367` and three mesh-collision validator checks red, measured; P52's bitwise test needs P42's renormalised normal, A17 §2.3).
19. **"The MJCF harness baseline is taken at the head of the core series"** (A13 §4); P24 and P26 change trajectories, so both sit in this PR (A13 §4: D1 "belongs in the core series").
20. **P11, P30, P36 → P36a.** `flexedge_invweight0` is derived in `recompute_derived` (A13 §3.5) and its fast branch tests MuJoCo's simple-body flag (`engine_setconst.c:733`), which P30 derives; P36 drops zero-Jacobian rows first (A18 §14). P36a is Rigid-loading L31's core rule; L31 keeps the parse and the doc rewrites.

## Commits

### P01 · A20 G1 · `test(sim-conformance): MuJoCo 3.5.0 parity census gate`
- **Closes:** the decision "the census becomes a permanent executable gate in Rigid (MuJoCo 3.5.0 per-doc golden data + a ratchet test whose agree count can only rise)" (01-decisions).
- **Implements:** A20 §2.1–§2.7. Snapshot of the 1,584 docs plus manifest, append-only (§2.2). Golden per doc: status only (no message), model fields as the census compares them, excitations e1 and e2, `forward` dumps at steps 0/1/100, 19 checkpoints (§2.1). Tolerance 1e-9 on each doc's maximum (§2.3). `verdicts.tsv` pins the class, not the label; an improvement fails until `CENSUS_BLESS=1`; a regression is never auto-blessed (§2.4). Module `layer_e_census.rs` in `mujoco_conformance`; `serde_json` dev-dependency with `float_roundtrip` (A7 §8 F4), a feature that unifies into every crate built with `serde_json` in the same build (R1). Gate parameters as Q130 recommends: 19 checkpoints, classes pinned, append-only snapshot.
- **Oracle and generator (C1):** P01 adds to `sim/L0/tests/scripts/`, beside the 3.4.0 generators (A20 §2.1), the oracle build script `build_mujoco_oracle.sh`, which builds MuJoCo 3.5.0 and its Python bindings from the 3.5.0 source tag with `-ffp-contract=off` (40-verification, *The unfused oracle*), and `gen_census_golden.py`, which runs on those bindings in place of A20 §2.1's PyPI pin, records the oracle and its platform in `meta.json`, and refuses to run on any other (R5). P01 also commits there the corpus extractor, the corpus harness with `harness2`, and `watchdog.py` (40-verification, *Before implementation*).
- **Notes** in `verdicts.tsv` (A20 §2.4, §2.6): `divergence=<ID>`, `known=<row>`, `nondet=`, and one more:
  - `fixed_by=<Pnn>` / `fixed_by=<Lnn>` (a transient regression): a commit that regresses a doc which a later commit fixes hand-edits the row with that later commit's id, and the later commit clears it. `fixed_by=P` occurs 0 times in `verdicts.tsv` at P53, and `fixed_by=` 0 times at L50.
- **Divergence registry:** `sim/L0/tests/assets/census/divergences.tsv` (ID, area, MuJoCo, ours, kind, test), the file every `divergence=<ID>` is checked against. It lives under `sim/L0/tests/` because `xtask/src/affected.rs:95-108` maps a file under `sim/docs/` to no crate (R5). The Intentional Divergences table (`sim/docs/MUJOCO_CONFORMANCE.md:321-333`) gains an ID column and points at the registry. P01's rows (A20 §2.6): non-finite keyframe and springlength numbers refused (stricter; 3 docs); `<inertial>` directly in `<frame>` refused (stated limitation; 1 doc).
- **Append-only, enforced:** the snapshot is `sim/L0/tests/assets/census/docs/` (one file per doc, plus the manifest `manifest.tsv`) and the golden `sim/L0/tests/assets/census/golden/` (one JSON per doc, plus `meta.json`). A CI step in `quality-gate.yml` fails when a commit modifies or deletes a file under `sim/L0/tests/assets/census/` other than `meta.json`, `verdicts.tsv`, `divergences.tsv` and the manifest (`git diff --name-only --diff-filter=MD <base>...HEAD` over that directory, those four excluded, non-empty), or removes a line from the manifest (`git diff --numstat` of it counts a deleted line). It is made to fail once on a scratch commit that deletes one golden file, and once on one that deletes a manifest line.
- **Drift check, per commit:** the extractor P01 commits runs in drift mode (1.9 s, A20 §2.2) at every commit of both PRs; a commit whose in-tree MJCF it reports changed appends the new docs and their goldens in the same commit.
- **Must-fail:** the gate was made to fail: main against the 837-floor file reports 61 regressions; `body_mass[1]` perturbations; a dangling divergence ID; a divergence doc that agrees; 12 `ours-refused` docs without notes (A20 §2.4 table). Added: the append-only step above.
- **Flips:** none; it compiles and passes at `main` (A20 §2.7, measured on the prototype on macOS arm64). Before landing, P01's test runs once on Linux x86-64, CI's platform (A20 §2.8 #3: not measured).
- **Census:** floor **783** (e1+e2, dumps 0/1/100) was measured against the arm64 wheel's golden (A20 §2.1); P01 blesses the floor measured against the oracle's golden, with `known=`/`divergence=` notes (A20 §2.6–§2.7). The 71 hash-order docs gave the same verdict in 60/60 processes, so no `nondet=` row is needed (A20 §2.4).

### P02 · M2 · `fix(cf-geometry): convex hull no longer depends on hash order`
- **Closes:** P-L27, cf-geometry half (10-scope; A6 §1).
- **Implements:** A6 §1.3 (sorted candidates; BFS `visible` as a `Vec`); `#![deny(clippy::iter_over_hash_type)]` at `design/cf-geometry/src/lib.rs:23` (A6 §1.9; 11-settled #14).
- **Must-fail:** `hull_is_identical_across_repeated_calls` (main: 6/3/3 distinct results in 50 calls) (A6 §1.6 #1); the lint gate fails on the original code (A6 §1.9).
- **Flips:** none measured; four validators byte-identical (A6 §1.7). `licensed-gates --run --only cf-design-tests` is mandatory (rung4/rung4b call `convex_hull`) (A6 §1.7, §6).
- **Census:** not measured. Corpus harness: the hull-only build leaves 60 nondeterministic docs (A6 §1.4).
- **Depends:** none (A6 §6: M2 and M3 independent).

### P03 · A10 MH1 · `fix(cf-geometry): convex hull is convex: exact orientation predicates, no unreferenced vertices`
- **Closes:** A10 §4.2 (our hull is not convex on real meshes; worst 5.35 %); no triage row.
- **Implements:** A10 §4.2: `orient3d` for visibility and conflict tests; output keeps face-referenced vertices only; `robust = "1.2"` as a workspace dependency (pure Rust, already in `Cargo.lock` via `spade`), named in cf-geometry's crate doc (`lib.rs:20-21` says "Pure `nalgebra` + optional `serde`").
- **Must-fail:** `hull_contains_every_input_point` (synth8, main 6.6e-2); a seeded near-coplanar property test (its failure rate on main **was not measured**: keep only seeds that fail on main); `hull_has_no_unreferenced_vertices` (A10 §4.2).
- **Flips:** none measured in CI; licensed rung4/rung4b not run (A10 §4.2, §7). `licensed-gates --run --only cf-design-tests` is mandatory here too: rung4/rung4b call `convex_hull` (A6 §1.7, §6).
- **Census:** not measured. Corpus: 5 docs change `model_fp` (all `flex_unified.rs`), trajectories ≤ 5e-16; the unreferenced-vertex removal was not run over the corpus (A10 §4.2, §7).
- **Depends:** P02 (A10 §7).

### P04 · K1 · `fix(sim-core): one pre-step check on every Result entry point; thermostat refuses h ≤ 0`
- **Closes:** core-L3, core-D3 (Display), P-L24 core half.
- **Implements:** A1 §1 (`check_step_inputs`, `check_data_shape` over 24 lengths; 11-settled #7) and A1 §2 (`ThermostatError::InvalidTimestep`). `inverse` documented, not changed (A1 §1); `integrate` becomes a checked `Result` entry point in P22 (C3).
- **Must-fail:** `forward_refuses_a_timestep_that_is_not_positive_and_finite`, `step2_at_negative_timestep_does_not_run_time_backwards`, `step_refuses_data_made_by_another_model`, `thermostat_install_refuses_a_negative_timestep`; new API `step_names_the_mismatched_field`, `invalid_timestep_says_what_it_is`, `thermostat_names_the_timestep` (A1 §1–§2).
- **Flips:** `sensor_tests.rs:1325` → rewritten as `test_forward_refuses_an_undersized_sensordata` (A1 §1).
- **Census:** not measured.
- **Breaking:** none at compile time (both enums `#[non_exhaustive]`); FD entry points now return `Err` at h ≤ 0 (A1 §1 Downstream).
- **Divergence rows:** a timestep ≤ 0 or non-finite refused; a `Data` made by another model refused (both stricter; A1 §10).

### P05 · decision (R4 draft) · `fix(sim-core): xfrc_applied in MuJoCo's force-then-torque order, in a new type` — cross-crate (sim-ml-chassis, sim-coupling, examples)
- **Closes:** the second-round decision "`xfrc_applied` → MuJoCo's force-first order AND a new type, so 0.9 code fails to compile" (01-decisions), with D1 (Jon, 2026-10-07): the sim-ml-chassis builder method is renamed too.
- **Now.** `pub xfrc_applied: Vec<SpatialVector>` (`types/data.rs:204-205`), `SpatialVector = Vector6<f64>` (`dynamics/spatial.rs:15`), laid out `[torque; force]`. Readers: `constraint/mod.rs:92-97` and `forward/acceleration.rs:128-133` split the halves into `mj_apply_ft`; `acceleration.rs:402-404` copies `cfrc_ext[b] = xfrc_applied[b]`; `island/sleep.rs:137-140` tests by value, `:529-530` by bits. Writers: `types/model_init.rs:601` (alloc), `data.rs:741` (Clone), `:1139-1141` and `:1261-1263` (resets).
- **Target** (MuJoCo 3.5.0): force then torque (`include/mujoco/mjdata.h:266`); `mj_xfrcAccumulate` calls `mj_applyFT(m,d,xfrc+6*i,xfrc+6*i+3,xipos…)` (`engine/engine_support.c:504-520`); `cfrc_ext` stays torque:force (`engine/engine_core_smooth.c:2503-2510`; A19 §1.5); `treeCanSleep` compares bytes (`engine/engine_sleep.c:132-137`).
- **Change** (sim-core):
  ```rust
  /// Force and torque applied to one body at its centre of mass (`xipos`), world frame:
  /// one row of MuJoCo's `mjData.xfrc_applied`, force first (`mjdata.h:266`).
  #[derive(Debug, Clone, Copy, PartialEq, Default)]
  pub struct BodyWrench { pub force: Vector3<f64>, pub torque: Vector3<f64> }
  impl BodyWrench {
      #[must_use] pub fn new(force: Vector3<f64>, torque: Vector3<f64>) -> Self;
      /// MuJoCo's row `[fx, fy, fz, tx, ty, tz]`.
      #[must_use] pub fn from_mujoco_row(row: [f64; 6]) -> Self;
      #[must_use] pub fn to_mujoco_row(&self) -> [f64; 6];
      /// `mju_isZeroByte`: every bit zero (−0.0 is not zero).
      #[must_use] pub fn is_zero_bytes(&self) -> bool;
  }
  // Data: pub xfrc_applied: Vec<BodyWrench>;  lib.rs: pub use … BodyWrench;
  ```
  - Not implemented (D1): `Index<usize>`, `IndexMut<usize>`, `Deref<Target = Vector6<f64>>`, `From<Vector6<f64>>`, `From<[f64; 6]>`, `Into<SpatialVector>`. Each lets a 0.9 form compile with the halves swapped.
  - Core sites: `mj_apply_ft(…, &w.force, &w.torque, …)` at both projection sites, the skip test staying a value test (`mju_isZero`, `engine_support.c:515`); `acceleration.rs:403` writes `cfrc_ext` torque then force (P27 later moves the torque to `xpos`); `sleep.rs:529` becomes `!w.is_zero_bytes()`; `sleep.rs:137` keeps its value test (the byte test is P21's); resets and the alloc use `BodyWrench::default()`. Docs: the field (`data.rs:203-205`) and `cfrc_ext` (`:652-654`).
- **Rule at every call site:** a behaviour-preserving rewrite. `[k]` with k < 3 becomes `.torque[k]`, k ≥ 3 becomes `.force[k − 3]`, and `Vector6::new(t…, f…)` becomes `BodyWrench::new(f, t)`. A comment that calls an index 0–2 write a force is corrected (`sleeping.rs:477` "Force in Z", `:5059`, `fluid_derivatives.rs:2190`). There are 48 index 0–2 writes (`git grep -cP 'xfrc_applied\[[^\]]*\]\[[012]\]\s*='`: `sleeping.rs` 37, `fluid_derivatives.rs` 8, `xfrc_applied.rs` 2, `body_accumulators.rs` 1).
- **Cross-crate edits** (`git grep -l xfrc_applied -- '*.rs'` → 40 files, all below or in sim-core):
  - **sim-ml-chassis** (D1): `ActionSpaceBuilder::xfrc_applied(body_range)` (`space.rs:821`) is renamed `body_wrench(body_range)` and the private `Injector::XfrcApplied` (`:616`) becomes `Injector::BodyWrench`. The action is `[force, torque]` per body, written through `BodyWrench::from_mujoco_row` (`:655`), so 0.9 code calling `.xfrc_applied(…)` fails to compile. The docs `:607`, `:615`, `:819`, the range-check label `:881` and the tests `:1501-1513` (asserts by index), `:1594` and `:1689-1698` follow.
  - **sim-coupling** (29 src writes + 1 test): `articulated.rs:147,665`; `articulated_grad.rs:341,689,997`; `bonded.rs:424,425`; `control.rs:49,115,339,656,833,902`; `freebody.rs:161,377,660`; `policy_grad.rs:318,682,1049,1134,1257,1430`; `single_step.rs:171`; `step.rs:103,165,327,416`; `tangential.rs:270,493`; `tests/rigid_multidof_response.rs:38`. One crate-private helper, `fn xfrc_from_torque_force(w: &SpatialVector) -> BodyWrench`, at every write keeps the crate's own `[τ; f]` wrenches and the `[τ; f]` columns of the pub `rigid_xfrc_column` (`vjp.rs:109-115`) unchanged. The docs `vjp.rs:31-36,76,111` and `lib.rs:14,40` say which layout is whose; the doc at `control.rs:720` names `xfrc_applied[body].z`, which `BodyWrench` does not have, and becomes `.force.z`.
  - **cf-codesign:** two doc lines name the field (`tools/cf-codesign/src/lib.rs:58`, `:1590`), no layout and no code; no edit.
  - **Examples (8 crates):** `composites/cable-loaded/src/main.rs:209-212`; `inverse-dynamics/stress-test/src/main.rs:569-570`; `sleep-wake/island-groups/src/main.rs:212,214`; `sleep-wake/stress-test/src/main.rs` (14 writes, `:333`…`:618`); `sim-ml/spaces/stress-test/src/main.rs:177` (the renamed builder method); `integration/full-pipeline/src/main.rs:255,262`; `integration/sim-informed-design/src/main.rs:160,170`; `integration/two-way-striker-viewer/src/main.rs:12` (a comment).
  - **Integration tests (9 files):** writes in `sleeping.rs`, `fluid_derivatives.rs`, `xfrc_applied.rs:26,74,105,137`, `body_accumulators.rs:112-114` (its element-wise compare `:119-126` compares halves) and `keyframes.rs:384,396`; doc mentions in `fluid_forces.rs:1635`, `inverse_dynamics.rs:9-10,52`, `mod.rs:282-286` and `mujoco_conformance/layer_a.rs:24`.
  - **No reference** (`git grep -l` 0): sim-gpu, sim-mjcf, sim-thermostat, sim-rl, rl-baselines, therm-env, soft, soft-explicit, urdf, opt, the bevy crates, fsu-model, `design/*`, the golden generators (`*.py`).
- **Must-fail** (not run):
  1. `xfrc_applied_is_force_then_torque`: a free sphere, mass 2, `diaginertia="0.1 0.1 0.1"`, COM at the origin, gravity 0; `from_mujoco_row([0,0,20, 0,5,0])`; `forward` → linear `qacc` (0,0,10), angular (0,50,0) (F = ma, τ = Iα). Main: does not compile.
  2. Two `compile_fail` doctests, one per 0.9 form so each must fail by itself: `data.xfrc_applied[1][5] = 1.0;` and `data.xfrc_applied[1] = nalgebra::Vector6::zeros();`. Both compile at the parent. Beside them, a passing doctest of the same setup.
  3. sim-ml-chassis: `body_wrench(1..2)` with the action `[1..6]` gives force (1,2,3) and torque (4,5,6).
- **Flips:** rewrites only, by the rule above. Research tests that land later and write the old type are ported there: A1 §8 `batch_reset_zeroes_applied_forces` (P16), A8's byte test in `tree_can_sleep` (P21), A8 SP-4 `user_qpos_and_xfrc_wake` (P23), A19 §1.7 #4 and R1 Step 1 (P27).
- **Census:** none (the census never sets `xfrc_applied`, A16 §5, ledger-L40).
- **Breaking:** the field type; the sim-ml-chassis method name and action layout. sim-coupling changes internally only.

### P06 · K2 · `fix(sim-core): hybrid sensor derivatives run every stage`
- **Closes:** P-L33 (10-scope; A2 §7).
- **Implements:** A2 §7: the six `forward_skip(Pos|Vel)` calls in `mjd_transition_hybrid` use `MjStage::None`.
- **Must-fail:** `hybrid_sensor_derivatives_equal_pure_fd` (main fails on `C`, Euler) (A2 §7).
- **Flips:** none measured; `t4_hybrid_matches_fd_sensor_derivatives` still passes (A2 commit list).
- **Census:** the census has no derivative step (A16 §5).
- **Note:** by reading, the test stands after P14: it compares hybrid with pure FD (A2 §7), not with literals. P06's `MjStage::None` stays needed after P14: ours runs position columns before velocity columns (`hybrid.rs:2870-2925`, then `:2992-3045`), MuJoCo runs ctrl → act → vel → pos (`engine_derivative_fd.c:339-510`) (R4). Not run.

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
- **Note:** option B stands on A13's measurement (bit-identical to main, 0 flips); its other premise, that `integrate()` keeps returning `()`, is void under C3 (R4).

### P08a · U02 · `fix(sim-core): step2 under RK4 takes MuJoCo's Euler step, eulerdamp included`
- **Closes:** U02 (13-open-questions; A7 §8 F2).
- **Now.** `integrate`'s `Integrator::RungeKutta4` arm adds `qacc·h` without eulerdamp (`integrate/mod.rs:185-197`); `step2` reaches it on an RK4 model.
- **Target.** `mj_step2` calls `mj_Euler` for every integrator but the implicit pair (`engine_forward.c:1505-1512`), and `mj_Euler` damps implicitly.
- **Change.** One `Integrator::Euler | Integrator::RungeKutta4` arm.
- **Must-fail:** `rk4_step2_is_the_euler_step`: an RK4 model with joint damping 0.05, 20 × `step1` + `step2`, `qvel` bit-equal to the same model with `Integrator::Euler`. A7 measured main's `qvel` 2.6e-2 off MuJoCo after 20 steps, and 2.8e-17 with `eulerdamp="disable"` (A7 §8 F2); against our own Euler, not run.
- **Census:** none (the gate does not call `step1`/`step2`, A20 §2.8).

### P09 · K4 · `fix(sim-core): reset_to_keyframe resets fully; energy_initial captured once per reset`
- **Closes:** core-L2, core-L1.
- **Implements:** A1 §4 (validate → `reset` → copy) and §5 (`pub(crate)` capture flag; 11-settled #6 and "A1 open").
- **Must-fail:** `reset_to_keyframe_is_a_full_reset`, `forward_captures_energy_initial`, `energy_initial_is_captured_once_even_when_zero`, `reset_clears_the_energy_baseline`; pin `reset_to_keyframe_with_a_bad_index_leaves_data_unchanged` (passes on main) (A1 §4–§5).
- **Flips:** none; the integrators stress-test prints `E_0 = -0.000000` (A1 §5).
- **Note:** the `data_reset_field_inventory` size guard cannot see a field that fits in padding (A1 §5).
- **Divergence rows:** a bad keyframe index leaves `Data` unchanged; `energy_initial` captured once per reset (A1 §10).

### P10 · K5 + C8 + Q17 · `fix(sim-core): RK4 advances plugins; Model::try_make_data`
- **Closes:** core-L7; C8 (Q16); Q17's `try_make_data` half.
- **Implements:** A1 §6, plus `make_data` running plugin `reset` (A1 §6 open question → 11-settled #9). `try_make_data` runs the joint-layout check, then the range check, then plugin `init`, then the reset (plugin `reset`; P18 and P23 add their refusals there): MuJoCo's `mj_makeData` is `mj_initPlugin` then `mj_resetData` (`engine_io.c:1110-1111`; R4). `make_data` = `try_make_data().unwrap_or_else(|e| panic!("{e}"))`.
- **`pub(crate) fn Model::make_data_for_derivation(&self) -> Data`:** no refusals and no plugin `init`. `build()` returns `Model`, not `Result` (`mjcf/src/builder/build.rs:27`), and its four derivations (`:38-41`) each call `make_data` (`tendon/mod.rs:62`, `forward/fiber.rs:134`, `types/model_init.rs:1211`, `:1033`), so a refusal there would panic inside `load_model`; those four sites call this instead.
- **C8** (R4 §6; A5 §3.3):
  ```rust
  #[derive(Debug, Clone, Copy, PartialEq, Eq)] #[non_exhaustive]
  pub enum JointLayoutError {
      TooManyDofs { body: usize, ndof: usize },               // MuJoCo "more than 6 dofs"
      FreeNotAlone { body: usize, joint: usize, njnt: usize },
      BallNotLast { body: usize, joint: usize },
  }   // Display keeps today's assert substrings ("must be the LAST joint", "must be the body's only joint")
  impl Model { pub fn check_joint_layout(&self) -> Result<(), JointLayoutError>; }
  // MakeDataError gains JointLayout(JointLayoutError); validate_joint_layout (model_init.rs:458-482) is deleted.
  ```
  `TooManyDofs` is checked first, in MuJoCo's order (`user_objects.cc:2523-2526` before `:2528`); A5 measured seven hinges on one body loading and stepping to NaN on main. Not checked: a non-root free joint (sim-core supports it, `test_fixtures/conformance.rs:249-267`; MJCF refuses it in L19) and arrays shorter than `nbody`, which still panic, as the doc says (A5-Q9). L19 refuses bad layouts in MJCF first, so this check is the stricter kind for code-built models.
- **Q17** (A5 §3.4): `pub fn Model::check_ranges(&self) -> Result<(), RangeError>`. A limited range passes MuJoCo's compile check: `lo < hi` for a hinge or slide joint, a tendon, and an actuator's ctrl, force and act range (`user_objects.cc:2909`, `:6419`, `:6883-6890`), and `lo == 0` for a ball joint (`:2912`); both bounds must also be finite (stricter). `RangeError { field: &'static str, index: usize }` (`#[non_exhaustive]`, `Eq`); `MakeDataError` gains `Range(RangeError)`. P11 runs the same check in `recompute_derived`.
- **Must-fail:** `rk4_advances_plugins_once_per_step`; new API `try_make_data_returns_a_plugin_init_failure` (A1 §6); `try_make_data_refuses_ball_before_hinge`, `try_make_data_refuses_a_free_joint_sharing_its_body`, `try_make_data_refuses_seven_dofs_on_one_body`, and the pin `make_data_still_panics_on_a_bad_layout` (R4 §6); `try_make_data_refuses_a_backwards_limited_ctrlrange` (`actuator_ctrlrange` set to (1, −1) in code; main builds the `Data` and panics at the first step in `f64::clamp`, A5 §3.4).
- **Flips:** `model_init.rs:1334`, `:1339`, `:1361` (rewritten on `check_joint_layout`); the comment at `examples/fundamentals/sim-cpu/raycasting/stress-test/src/main.rs:145`. `ball_joint_limits.rs:485` `test_ball_limit_range_symmetry` (its `"45 0"` case) and `:1213` `test_ball_limit_reversed_range_parsing`: each loads a limited ball joint with `range="45 0"` and calls `make_data`, which now panics (by reading; A5 §3.4 had listed both as L21's flips). The `"45 0"` case is dropped, and `:1213` becomes a refusal test with `jnt_range` set in code, so L21's load refusal does not flip them again. Other in-tree models, MJCF or code-built, with a limited range this check refuses: not checked. From this commit the census harness builds `Data` with `try_make_data` and records an `Err` as `ours-refused`.
- **Breaking:** `validate_joint_layout` deleted (0 callers outside `model_init.rs`, R4).
- **Divergence rows:** a cloned `Data` drops `plugin_data` (stated limitation, A1 §6, §10); a code-built `Model` with a bad joint layout or range is refused (stricter).

### P11 · K6 + Q17 · `feat(sim-core): kinematic trees in sim-core; Model::recompute_derived` — cross-layer
- **Closes:** core-L5, P-L31 (sleep panic).
- **Implements:** A1 §7's measured patch, with factories and `finalize` calling the full `recompute_derived` (A1 §7(a)). 11-settled #8 also moves geom bounding radii and fixed tendon lengths into it — "not in the measured patch" (A1 §7(b)) — and switches cf-design's tree copy (A1 §7(c), cf-design tests not run). A17 C5 puts hfield bounds into the moved radii function (A17 §11 R3-C).
- **Q17:** `pub fn recompute_derived(&mut self) -> Result<(), RangeError>` runs P10's `check_ranges` first and changes nothing on `Err`; factories and `finalize` `.expect` it.
- **Cross-layer:** sim-mjcf `build()` calls `compute_kinematic_trees` and deletes its three private functions (A1 §7).
- **Must-fail:** `sleep_runs_on_factory_models`, `compute_dof_lengths_sizes_dof_length`; new API `recompute_derived_follows_a_damping_edit` (A1 §7), `recompute_derived_refuses_a_backwards_limited_range`.
- **Flips:** none across the suites run (A1 §7).
- **Census:** not measured. Tree fields identical between old and new builders for all 1,584 docs; `recompute_derived` changes no field on 1,478/1,478 unedited loads (A1 §7).

### P12 · K7 · `feat(sim-core): callbacks fire in MuJoCo's order and counts` — cross-crate (sim-thermostat)
- **Closes:** core-C1 (behaviour), core-C2, core-C3.
- **Implements:** A2 §1 code, §2 docs, and the sim-thermostat doc lines `component.rs:215-216`, `langevin.rs:644-645`. The two doc sentences that are true only after P15 go in P15 (A2 §2).
- **Must-fail:** `callbacks_fire_in_mujoco_order_and_count`, `passive_callback_fires_on_a_model_without_dofs`, `rk4_reevaluates_the_control_callback_at_each_stage`, `thermostat_reads_its_own_channel_when_another_is_bad`; pin `passive_callback_skipped_when_springs_and_dampers_are_disabled` (A2 §1, §5, commit list).
- **Flips:** none (A2 commit list).
- **Census:** callbacks are outside the census (A16 §5).

### P13 · K8 · `feat(sim-core): finite differences refuse RK4 and history`
- **Closes:** core-C1, FD half.
- **Implements:** A2 §3 (`StepError::UnsupportedIntegrator { integrator }`, 11-settled A2-Q1) and A2 Q2 (the `nhistory > 0` panic at `derivatives/fd.rs:73-76` and `hybrid.rs:2361-2364` becomes `Err`, 11-settled A2-Q2). A2 names no variant; it is `StepError::UnsupportedHistory { nhistory }`, beside `UnsupportedIntegrator`, because the derivative functions return `StepError` (A2 Q1), with MuJoCo's text "delays are not supported" (`engine_derivative_fd.c:547-549`).
- **Must-fail:** `finite_differences_refuse_rk4` (A2 §3); `finite_differences_refuse_history` (a model with an actuator `delay`, so `nhistory > 0`; main panics).
- **Flips:** `derivatives.rs:451` (rewrite to the refusal), `derivatives.rs:464` (delete); validator `derivatives/stress-test` check 23 and `README.md:35`; the doc lines A2 §3 lists.

### P14 · K9 (R4 draft) · `feat(sim-core): sensor derivatives at the current state`
- **Closes:** the decision "sensor derivatives take MuJoCo's semantics (C at the current state, A2 Q4)" (01-decisions).
- **Now.** Pure FD re-runs `forward()` after every `step()`: `derivatives/fd.rs:100-108` (nominal), `:151-157`, `:178-184` (state ±), `:228-234`, `:251-257` (ctrl ±). The hybrid path does the same: nominal `hybrid.rs:2753-2759`; piggyback columns `:2944-2946`, `:2968-2970`, `:3130-3132`, `:3154-3156`, `:3357-3359`, `:3378-3380`; sensor-only columns (`forward_skip` + `integrate` + `forward`) `:2893-2896`, `:2913-2916`, `:3013-3016`, `:3033-3036`, `:3078-3081`, `:3098-3101`, `:3305-3308`, `:3321-3324`. The docs give C and D as `∂sensordata_{t+1}/∂x_t`, `∂sensordata_{t+1}/∂u_t` (`derivatives/mod.rs:124,131`).
- **Target.** `mjd_transitionFD` (`engine/engine_derivative_fd.c:542-593`) → `mjd_stepFD` (`:295-530`): each column calls `mj_stepSkip` (`:113-147`, forwardSkip then the integrator), then `getState(m,d,next,sensor)` (`:37-42`), which copies the `sensordata` computed inside `mj_forwardSkip` at the perturbed current state; no forward after the step. Measured (A2 `fd_counts.out`): M1 Euler C = [[1.000000000001, 0.0], [0.0, 0.9999999999732445]], D = [0.0, 0.0]; callback counts equal with and without sensors.
- **Change.**
  - `fd.rs`: delete the 5 `if compute_sensors { scratch.forward(model)?; }` blocks and read `sensordata` right after `step()`; rewrite the comment at `:100-102`.
  - `hybrid.rs`: delete the post-step `forward()` at the nominal and the 6 piggyback sites; at the 8 sensor-only sites delete `integrate` and `forward` and read `sensordata` right after `forward_skip`. P06's `MjStage::None` stays.
  - Docs: `mod.rs:124-136` become `∂sensordata_t/∂x_t`, `∂sensordata_t/∂u_t` ("as MuJoCo's `mjd_transitionFD`"), and `DerivativeConfig::compute_sensor_derivatives` (`mod.rs:175-187`) matches.
  - No public signature changes; the values of C and D change.
- **Must-fail** (not run):
  1. `sensor_derivatives_at_current_state_match_mujoco_3_5_0`: A2's M1 (`fd_counts.py:7-11`: a hinge, axis 0 1 0, damping 0.1; a capsule fromto 0 0 0 .5 0 0, size .05, mass 1; a motor; `jointpos` + `jointvel`; Euler); qpos 0.2, qvel 0.3, ctrl 0.4, `forward`; `mjd_transition_fd`, centered, eps 1e-6 → C equals MuJoCo's to 1e-9, D = [0, 0] exactly. Main: C row 0 = [0.99946, 0.00989] (A2 Q4).
  2. The same through `mjd_transition_hybrid`; main fails.
  3. `sensor_derivatives_do_not_add_callbacks`: M1 Euler centered, callback counts equal with sensors on and off. Main, pure FD: 14 vs 7 (A2 §4, measured). That P14 makes them equal is from reading.
- **Flips** (by reading): `sim/L0/tests/integration/derivatives.rs:2116` t3 (its reference re-runs `forward()` after `step()`, `:2151-2152`, `:2169`, `:2187`, `:2193`; rewrite the reference); `:2445` t11 (asserts `|C[s,nv+s]| > 1e-6` at `:2474-2482`, but the jointpos velocity column is exactly 0; new expectation: position block = I to 1e-9, velocity block = 0, D = 0; its doc `:2435-2442`); `examples/fundamentals/sim-cpu/derivatives/sensor-jacobians` (the "D not all zeros" check, `d_nonzero` at 1e-15, fails; README `:9`, `:31-37`; adding `<actuatorfrc actuator="torque"/>` gives D a nonzero row). Pass by reading: t1, t2, t4 (5 %), t5–t8, t10, t12, `linearize-pendulum` and `derivatives/stress-test`, whose 3 sensor checks test only `Some`/`None` (`main.rs:603-672`).
- **Note:** on a step where a tree falls asleep, pure FD reads the sensors P22's re-forward computes inside `step()`, and the hybrid sensor-only columns do not, so the two paths differ on that step (by reading; not measured). The hybrid doc says so.
- **Census:** none (A16 §5).
- **Breaking:** the values of C and D. Readers (`git grep`): sim-core, `derivatives.rs` and 3 examples; sim-coupling reads neither.

### P15 · K10 · `fix(sim-core): the bad-ctrl check reads the clamped input and leaves ctrl alone` — cross-crate (sim-thermostat)
- **Closes:** core-L4.
- **Implements:** A2 §5 (`actuator_ctrl_input`), plus the CbPassive/CbControl sentences A2 §2 assigns to C-4. P18's delayed read goes inside `actuator_ctrl_input` (A7 H-1 step 4).
- **Must-fail:** `bad_ctrl_check_runs_on_the_clamped_input_and_leaves_ctrl_alone` (A2 §5).
- **Flips:** `runtime_flags.rs:718` `ac29_ctrl_validation`; `thermostat/src/langevin.rs:647` (A2 §5).

### P16 · K11 · `feat(sim-core): per-env BatchSim returns errors; model_of, forward_all` — cross-crate (sim-thermostat)
- **Closes:** core-B3, P-L19, plus the core-B2 doc (A1 §8).
- **Implements:** A1 §8 shape C (11-settled #10).
- **Must-fail (new API):** `per_env_batch_refuses_models_of_different_shapes`, `model_of_returns_each_envs_model_and_forward_all_runs`; thermostat tests ported from `install_per_env`; pin `batch_reset_zeroes_applied_forces` (A1 §8), written on P05's `BodyWrench`.
- **Flips:** `tests/integration/batch_sim.rs:276-302`; sim-thermostat `stack.rs` impl, tests and docs, `lib.rs:22`, the `install_per_env` mentions in `langevin.rs`; a comment in `rl-baselines` (A1 §8).
- **Note:** the B1 determinism paragraph must cover `forward_all` (A2 §8).
- **Also** (ledger-L67): `BatchSim::new` calls `make_data`, which panics on a model `try_make_data` refuses (P10), and its doc has no `# Panics`.

### P17 · K12 · `refactor: delete SimulationConfig, SolverConfig, Gravity` — cross-layer (sim-mjcf), cross-crate (sim, cortenforge)
- **Closes:** core-D4.
- **Implements:** A1 §9 and A3 §5: sim-mjcf's `config.rs`, `ExtendedSolverConfig` and the `sim-types` dependency go in the same commit, or it does not compile (A3 §5). `SimError` goes "if it has no consumer at implementation time (measure)" (11-settled #11). The `sim/sim` umbrella re-exports `sim_types` and `SimulationConfig`/`SimError` (`sim/sim/src/lib.rs:88`, `:133`), and `cortenforge/src/lib.rs:124` names `SimulationConfig`; both change here (R1).
- **Must-fail:** none; the deletion is checked by compile (A1 §9). **Not compiled** during planning (A1 "What I checked").
- **Flips:** 15 + 5 (+3) tests deleted with their types (A1 §9).

### P18 · A7 DH-1 · `feat(sim-core): actuator and sensor delays act, as MuJoCo 3.5.0` — cross-layer (sim-mjcf)
- **Closes:** P-L34 runtime half (decision "Delay/history (P-L34): implement in Rigid"); Q18, Q22, Q23, Q24; A7 Q5.
- **Implements:** A7 H-1 (crate-private `history.rs`, init in `make_data`/`reset`, delayed read in `actuator_ctrl_input`, sensor extraction and gate, postprocess skip, insertion in `integrate` and RK4) with RK4 sensor samples at the step-start state (option C, kept under the wrong-value rule, Q18), plus A7 Q4 (`InterpolationType: From<i32>` → `TryFrom<i32>`).
- **History addresses** (A7 Q5): `compute_history_addresses` (`mjcf/src/builder/build.rs:520`) moves into sim-core's `recompute_derived`; sim-mjcf's private copy goes.
- **Refusals at `try_make_data` and at reset** (Q22, Q23): A7 H-1 step 7, which copies `sensordata` into the buffer of a user or plugin sensor with `delay > 0`, becomes a refusal, `DelayedUserSensor { sensor }` (MuJoCo `mjERROR`s on that sensor, A7 H-1 step 7); `nhistory > 0` with `timestep ≤ 0` is refused as `InvalidTimestep` (MuJoCo `_resetData`, `engine_io.c:1266-1270`). Both are variants of `MakeDataError` and of `ResetError` (`types/enums.rs:888-899`, `#[non_exhaustive]`).
- **Reset signature:** `pub fn Data::try_reset(&mut self, model: &Model) -> Result<(), ResetError>` is the refusing reset. `Data::reset(&mut self, model: &Model)` keeps `()` and panics with the error's Display (`# Panics`), as `make_data` does; `reset_to_keyframe` already returns `Result<(), ResetError>`; `BatchSim::reset(i) -> Option<()>` (`batch.rs:293`) inherits the panic, documented.
- **Must-fail:** `delayed_actuators_match_mujoco_3_5_0`, `delayed_sensors_match_mujoco_3_5_0`, `sensor_history_initial_state_matches_mujoco`, `delayed_sensor_reads_the_value_one_step_earlier`; `interpolated_value_is_not_reclamped_by_cutoff` (made to fail by removing the postprocess skip) (A7 H-1). The phase-rounding case sets `model.sensor_interval[i].1` in code: the parser reads a one-value `interval` until L48 (A7 §6). Golden assets: `assets/golden/history/`, MuJoCo 3.5.0 (A7 H-1). New API: `try_make_data_refuses_a_delayed_user_sensor`, `try_reset_refuses_history_with_a_nonpositive_timestep`. Pin (Q24): `step1_step2_accelerometer_equals_step`; it passes on main, which has no stale step1/step2 accelerometer (A7 §8 F3).
- **Flips:** none (720 lib tests; the integration modules run) (A7 H-1, §4).
- **Census:** ledger-L34 sensor delay, 1 doc (A16 §2). Corpus: all 1,421 deterministic docs with `nhistory = 0` bit-identical; the 22 with `nhistory > 0` changed, or refused once L48's rules land (A7 §4).
- **Breaking:** the `TryFrom` change (0 in-tree callers, A7 Q4).
- **Divergence rows:** RK4 samples a delayed sensor at the step-start state (wrong-value rule, Q18; test `delayed_sensor_reads_the_value_one_step_earlier`); MuJoCo's `mj_step1` + `mj_step2` accelerometer is stale, Δ 5.9 against `mj_step` (A7 §2), ours equals `step`'s (wrong-value rule, Q24; test `step1_step2_accelerometer_equals_step`).

### P19 · A7 DH-2 · `feat(sim-core): read and initialise history buffers`
- **Implements:** A7 H-2 (`HistoryError`, four `Data` methods); in Rigid (Q20).
- **Must-fail:** the four reads/inits against the API golden, and one test per error variant — new API, so they do not compile on main (A7 H-2).

### P20 · A8 S1 · `fix(sim-core): kinematic trees and automatic sleep policies as MuJoCo`
- **Closes:** part of "sleep re-forward and sleep timing: fix in Rigid" (01-decisions); census sleep cluster, 20 docs (A16 §2).
- **Implements:** A8 SP-3, plus the `rne.rs:362` `tree < ntree` guard. SP-3 marks every tree of a qualifying tendon through a new `pub fn Model::tendon_trees(&self, tendon: usize) -> impl Iterator<Item = usize> + '_` (the trees its wraps touch), which Rigid-loading L46 also calls. The two MuJoCo compile errors of SP-3 go to the MJCF series (Rigid-loading L46).
- **Where:** since P11 the tree code is sim-core's `Model::compute_kinematic_trees` (`types/model_trees.rs`); A8 cites it in sim-mjcf's builder (`build.rs:690-745`, `:810-957` then). The explicit `sleep=` attributes stay in sim-mjcf, `apply_explicit_sleep_policies`, where an explicit `auto` is a no-op since chunk 2b's review fixes.
- **Not pinned before this entry:** the automatic-policy rules for site, body and slider-crank transmissions and `JointInParent`, the tendon rule's `tendon_limited` clause, and a dof-less tree's `tree_dof_adr` (single-site mutants of each passed every suite, chunk 2b's review).
- **Must-fail:** `static_body_is_not_a_tree`, `box_on_static_body_sleeps_as_mujoco` (A8 SP-3).
- **Flips:** none in the suites run; tree tables change in 240 non-sleep docs, no trajectory change (A8 SP-3).

### P21 · A8 S2 · `fix(sim-core): dof_length from MuJoCo's body sizes; MuJoCo's sleep tolerance test`
- **Must-fail:** `dof_length_matches_mujoco`, `rotational_sleep_matches_mujoco`, `negative_zero_force_blocks_sleep` (A8 SP-2); A8's byte test in `tree_can_sleep` is written on P05's `BodyWrench` (`is_zero_bytes`).
- **Flips:** `sleeping.rs` `test_dof_length_computation`, `test_dof_length_hinge_1m`, `test_dof_length_hinge_01m`, `test_dof_length_free_joint`, `test_dof_length_nonuniform_threshold` (A8 SP-2).

### P22 · A8 S3 · `fix(sim-core): sleep decided in the advance; re-forward on the sleep step` — cross-layer (sim-mjcf)
- **Closes:** sleep re-forward and sleep timing (01-decisions; A2 Q3); C3 (Q01); Q28 and Q30, with P35 and P43.
- **Implements:** A8 SP-1, SP-5, SP-7 (sleeping `qacc` = MuJoCo's stale `qacc_smooth`, A8 Q1), SP-8, SP-9 (sim-mjcf's `guard_rk4_sleep` deleted; the RK4 path then takes P32's fix, C4). "Splitting S3 further was not tried" (A8 commit list).
- **C3** (R4 §5): `pub fn integrate(&mut self, model: &Model) -> Result<(), StepError>`. As a `Result` entry point it runs P04's `check_step_inputs`; `step`/`step2` call a crate-private `integrate_unchecked`, as A8 splits `forward_skip`/`forward_skip_unchecked`. Callers after P14 (`git grep -nE "\.integrate\("`): `step2` and `step` in `forward/mod.rs`, and three tests returning `()`, `dt53_forward_skip_plus_integrate` (`derivatives/fd.rs`), `t11_plugin_advance_in_integration` (`plugin.rs`) and `post_step_qvel_is_the_velocity_integrate_leaves` (`derivatives/integration.rs`) (`.expect`); the 8 `hybrid.rs` calls went in P14. The doc at `integrate/mod.rs:26-38` says that sleep is decided inside `integrate`, gives the `# Errors` and A8's failure state ("activations advanced and the slept trees' `qvel` zeroed; positions not advanced"), and replaces P04's "does not check `model.timestep`". A direct `forward()` + `integrate()` caller now gets the sleep step (parity: `mj_stepSkip` → `mj_Euler` → `mj_advance` runs `mj_sleep`); the effect on FD with sleep enabled was not measured.
- **Islands** (Q28, Q30): one partition, MuJoCo's. It is built from the efc rows (each row joins the trees of its nonzero Jacobian columns, `engine_island.c:144-370`, A18 §4) whenever `nefc > 0` and `DISABLE_ISLAND` is off (`engine_island.c:378-380`), with or without sleep. Sleep uses it here and P35's solver uses the same one. A8's prototype built islands from raw sources plus SP-5's friction edges, with sleep only (A8 Q5); the efc-row partition was not run on A8's fixtures. The comment at `runtime_flags.rs:1652-1653` ("gated on ENABLE_SLEEP") changes.
- **Must-fail:** `sleep_step_matches_mujoco`, `sleep_step_reforward_fires_both_callbacks`, `sleep_step_sensors_see_zero_velocity`, `sleep_step_matches_mujoco_implicit`, `sleep_wakes_and_sleeps_with_contact` (SP-1); `asleep_on_static_makes_no_contacts`, `sleeping_rows_dropped`, `island_disabled_blocks_sleep_with_constraints`, `friction_rows_make_islands` (SP-5); `integrate_refuses_a_timestep_that_is_not_positive_and_finite`, `integrate_refuses_data_made_by_another_model` (C3, R4 §5); `islands_exist_without_sleep` (`box_rest` with sleep off: `nisland` 1 as MuJoCo, main 0, A8 Q5).
- **Flips:** `sleeping.rs` `test_island_delassus_equivalence` (SP-5); `sleeping.rs` `test_sleep_zeros_velocity`, `test_sleep_trees_zeros_all_arrays`, `test_indirection_vel_integration_equivalence`, `test_partial_ldl_solve_zero_sleeping_rhs`, `body_accumulators.rs:366`, `sensors_phase4.rs:629` and validator `sleep-wake/stress-test:285` (SP-7, under A8 Q1 = parity); `sleeping.rs` `test_rk4_sleep_warning` (SP-9); the three tests above that call `integrate` (C3).
- **Census:** `d22fcd34` (`rk4_sleep_mjcf` in `sleeping.rs`, the one in-tree RK4 doc with sleep) → agree: its only difference is `enableflags`, which the deleted RK4 guard sets (A20 §1A.2, A8 SP-9), and it never sleeps and agrees at 2.8e-14 with or without P32's fix (A19 §5.3). Otherwise not measured on the census. All S-items together: 70/70 corpus sleep docs have MuJoCo's transitions (main 41/70); 1,328 non-sleep docs bit-identical (A8 "Result of the prototype").
- **Docs earlier commits wrote that this changes:** P12's `CbPassive` doc says MuJoCo runs the forward pass once more on the step that puts a tree to sleep and sim-core does not: delete it. P14 left this here: on that step pure FD reads the sensors of the re-forward and the hybrid's sensor-only columns do not, so the two paths differ; the hybrid doc says so.
- **Breaking:** `integrate`'s signature (0 callers outside sim-core, R4 §5).

### P23 · A8 S4 · `fix(sim-core): wake rules and init-sleep as MuJoCo`
- **Implements:** A8 SP-4, SP-6; init trees that cannot sleep are refused at `try_make_data` and at P18's `try_reset` (A8 Q3; MuJoCo errors in every reset, `engine_io.c:1473-1493`). The load-time mapping is Rigid-loading L46. Amendments (R4):
  - `MakeDataError::InitSleep { marked, slept, tree, root_body }` and the same `ResetError` variant (additive); the Display is MuJoCo's text (`engine_io.c:1490-1492`) with our ids.
  - The init-sleep `forward` can return `Err` (implicit factorization), which MuJoCo's `mj_forward` cannot: `MakeDataError::InitForward(StepError)` and `ResetError::InitForward(StepError)`. Whether any in-tree init model reaches it: not measured.
  - P10's `make_data_for_derivation` also skips init-sleep: every tree awake, as MuJoCo's sleep-disabled derivation `Data` (`user_model.cc:5108-5113`); L44's lengthrange calls it too.
  - With sleep enabled and no init tree, MuJoCo's reset computes the kinematics, centres of mass, cameras and tendons (`engine_io.c:1453-1458`); `start_sleep` computes none of them (the `Data::reset` doc says so). Whether anything reads them before the first forward pass: not measured.
  - `SleepError` (`types/enums.rs:853-886`) is deleted with `validate_init_sleep`; `island::reset_sleep_state` (`island/sleep.rs:294`) becomes `pub(crate)` (no caller outside sim-core, `git grep`).
- **Must-fail:** `wake_on_contact_inherits_countdown`, `user_qvel_wakes_sleeping_tree`, `user_qpos_and_xfrc_wake` (written on P05's `BodyWrench`), `disabling_sleep_wakes_all` ("not run on either engine"), `init_tree_has_reset_values`, `init_mixed_island_refused` (A8 SP-4, SP-6).
- **Flips:** `fluid_forces.rs:1495` `t47_wind_sleep_qfrc_fluid_zero` (A8 SP-6); `sleeping.rs:2612` `test_init_sleep_mixed_island_warning` (T60), whose mixed init doc `make_data` now refuses, rewritten as the refusal test (`try_make_data` → `Err(InitSleep)`) (R4; A6 §5.1 measured MuJoCo refusing that doc).
- **Breaking:** `SleepError` (pub, exported, not `#[non_exhaustive]`, 0 users outside sim-core) deleted; `island::reset_sleep_state` leaves the public API.

### P24 · K13 · `fix(sim-core): the velocity product of a multi-joint body uses the earlier joints' velocity (MuJoCo mj_comVel)` — cross-layer (sim-gpu)
- **Closes:** P-L32 (decision default "P-L32 in Rigid", 01-decisions).
- **Implements:** A12 §4: forward paths, body accumulators, `mjd_rne_vel`, `mjd_rne_pos`; and `rne.wgsl` stepping the partial velocity per joint (A12 Q1). "The GPU shader gets the same change in that commit (not yet written or run)" (21-multi-joint-bias).
- **Must-fail:** same-body vs split-body `qacc` invariant; MuJoCo 3.5.0 golden `qfrc_bias` on `hinge_hinge_g0` (+ slide+hinge, hinge→ball); implicit integrator vs MuJoCo; guard `validate_analytical_vs_fd` (does not fail on main) (A12 §6).
- **Flips:** none pinned. The forward half alone fails `derivatives::test_pos_deriv_multi_joint_body` (2.6e-2 > 1e-4), so forward and derivatives are one commit; GPU T15c (tolerance 1e-3) fails unless the shader changes (A12 §5); CI runs it on lavapipe (Conventions, Checks).
- **Census:** L32 cluster, 5 docs → agree (A16 §3). Corpus: 6 docs change trajectory (A12 §5).

### P25 · A14 L36a · `fix(sim-core): analytic position derivatives follow the origin a body's own joints move`
- **Closes:** ledger-L36a; the pre-existing gap of A12 Q2.
- **Must-fail (measured at the K13 parent):** harness cases `hinge_then_slide_root`, `offset_hinge_child`, `hinge_then_offset_hinge`, `chain3_offset`; MJCF derivative tests F1 and F5; factory `multi_joint_body` (A14 §1.6). The harness cases go in the CPU-only list (Q39).
- **Flips:** none (A14 §1.5). **Census:** none (derivatives).
- **Note:** kept separate from P24 because family B also hits single-joint bodies (A14 §5).

### P26 · A13 D1 · `fix(sim-core): a connect's and a weld's impedance come from the norm of their violation (MuJoCo getposdim)`
- **Closes:** ledger-L39a (A13); census cluster ledger-L44a (A16 §2).
- **Must-fail:** `connect_impedance_is_shared_across_rows`, `weld_matches_mujoco_3_5_0`; the cable check promoted against MuJoCo's explicit XML fixture, not the composite form, which needs Rigid-loading L07 (A13 §1.8).
- **Carried from P08** (ledger L57): A13 §2.6's `implicit_warmstart_is_explicit` as specified (`conn_free2_implicitfast_PGS`, 500 steps against MuJoCo at 1e-12) is in the repository as `connect_under_implicitfast_pgs_matches_mujoco_3_5_0` (`sim/L0/tests/integration/implicit_integration.rs`), `#[ignore]`d, with the fixture and MuJoCo 3.5.0's state at steps 1, 10, 100 and 500; this commit removes the `#[ignore]`. Before it, the test fails at step 10: `qvel` 1.4e-8 off (8.4e-3 at step 100). A13 §1.4 measured the same fixture with both fixes at 2.6e-15 (max |Δqpos| over 500 steps). The fixture's census doc `37f05467b082fc7d` reads `model:geom_quat;dyn:e1:s1:qfrc_constraint` until then. P08 landed a test named `implicit_warmstart_is_explicit` that needs no constraint.
- **Flips:** none; the equality stress-test validator prints changed numbers (A13 §1.7). Bits change in 45 of 55 deterministic connect/weld docs; why the other 10 do not "was not examined" (A13 §1.6).
- **Census:** +24 docs → agree, 3 move to an L41 label (A16 §3).

### P27 · A19 R1 · `fix(sim-core): body accumulators match mj_rnePostConstraint`
- **Closes:** census NEW-ACCEL (A16 §2); the `cfrc_ext` findings of A19 §1.3.
- **Implements:** A19 §1.5 (free-joint term through one helper shared with `rne.rs`; `cfrc_ext` at `xpos`; connect/weld forces; `xfrc_applied` torque moved to `xpos`). Two readings stay ours under the wrong-value rule, each with its test: the static-body accelerometer (Q43) and `cfrc_int[0]`, which MuJoCo sums from the roots' wrenches without moving them to one point (`engine_core_smooth.c:2676-2679`; Q44, A19 Q1.2).
- **Must-fail:** `accelerometer_free_body_matches_mujoco_3_5_0`, `force_sensor_includes_connect_force`, `torque_sensor_contact_lever_at_origin`, `xfrc_torque_moved_to_origin` (written on P05's `BodyWrench`); deviation pin `static_site_reads_gravity` (Q43; passes on main) (A19 §1.7). Wrong-value pin, passing on main by reading: `world_site_torque_is_the_sum_of_root_wrenches_moved_to_it` (Q44, the test A19 Q1.2 describes).
- **Flips:** none (A19 §1.8).
- **Census:** 7 of 8 NEW-ACCEL docs → agree, plus `29661ba2` and `72212e25` under e2; 0 regressions (A19 §1.6).
- **Divergence rows:** the static-body accelerometer reads +g (Q43); `cfrc_int[0]` is referenced at the world origin (Q44).

### P28 · A19 R2 · `fix(sim-core): forward kinematics as mj_kinematics1 (qpos − qpos0, off-centre ball, xaxis/xanchor)` — cross-layer (sim-gpu)
- **Closes:** ledger-L45 FK half = census L45ref (A16 §2); A14 L36b core part, which R2 takes over (A19 §2.3); A11's unexplained `muscle_qpos0_ref` residual (A19 §2.4).
- **Implements:** A19 §2.3 (C5): FK, ball velocity and subspace, the sleep FK copy, `fk.wgsl` subtracting `qpos0` for hinge and slide.
- **GPU offset ball** (C5, Q42: stated limitation; R4): the refusal already exists, because `GpuPhysicsPipeline::new_batched` refuses every non-free joint (`gpu/src/pipeline/orchestrator.rs:130-133`). No shader change for the ball; doc lines on `GpuFkPipeline` and `GpuModelBuffers::upload` say the public sub-pipelines validate nothing and rotate a ball about the body origin.
- **Must-fail:** `hinge_ref_fk_matches_mujoco`, `slide_ref_fk`, `offset_ball_matches_mujoco` (the chain case needs P25), and A14's X1/X3/X5 `xaxis`/`xanchor` literals (A19 §2.6; A14 §3.3); `gpu_fk_hinge_ref_is_identity_at_qpos0` (A19 §2.6 test 1's `ref="30"` fixture through `GpuFkPipeline`; no sim-gpu test sets `model.qpos0`, `git grep qpos0 -- sim/L0/gpu` finds one local variable) (R4). Pin: `gpu_pipeline_refuses_a_ball_joint`.
- **Flips:** none; validator `joint_limits` prints 5 changed values (A19 §2.5).
- **Census:** L45ref 4 docs + 5 more → agree; 0 regressions (A19 §2.5). Corpus: 9 docs with nonzero `ref` change (A19 §2.5). P11 added one more doc of the cluster, `f55567dc9d0dc54b` (`model:geom_quat;dyn:e1:t0:xpos`, a hinge with `ref="0.3"`; ledger-L68).
- **Downstream:** cf-design sets a ball's `jnt_pos` from its anchor (`mechanism/model_builder.rs:525-528`, `:554-562`); its in-tree balls (cf-trike `lib.rs:1234`, `:1308`, `:1494`) are one joint per body, so `jnt_pos = 0` (by reading; R4).
- **Divergence row:** sim-gpu refuses ball joints in `GpuPhysicsPipeline`; its public sub-pipelines ignore a ball's `jnt_pos` (stated limitation, Q42).

### P29 · A19 R2b = A20 R3-P2 · `fix(sim-core): spatial tendon spring length resolved at qpos_spring`
- **Merged:** the same change, proposed by both sections (A19 §2.3 last bullet; A20 §1A.3.7).
- **Must-fail:** `spatial_lengthspring_at_springref` (A19 §2.6 #5); the `tendon-limits` doc gives `tendon_lengthspring == 0.7071067811865476` (A20 §1A.3.7).
- **Census:** `251c5904` (A19 §2.5); 2/2 with the FK fix (A20 §1A.2).
- **Isolates:** U11 (spatial tendon velocity over a sphere wrap reads 0 at the initial forward; A20 §1A.3.9, not isolated): fixed here, or the first differing quantity and a divergence row recorded here.

### P30 · A19 R3 · `fix(sim-core): implicit integrators restrict D to M's sparsity (MuJoCo qH/qLU)`
- **Closes:** census NEW-IFLUID (A16 §2).
- **Implements:** A19 §4.3; MuJoCo's `body_simple` (`user_model.cc:2750-2860`) and `dof_simplenum` (`:3942-3962`) become derived fields in `recompute_derived`. P36a also reads `body_simple`.
- **Must-fail:** `implicitfast_fluid_matches_mujoco_3_5_0`, `implicit_cross_branch_tendon_damping` (A19 §4.3).
- **Flips:** none. **Census:** `d58bb68a` → agree, 0 regressions (A19 §4.3).
- **Note:** the simple-DOF flags equal MuJoCo's only after MH2 (Rigid-loading L03) and MuJoCo's mesh frame (Rigid-loading L35, Q63) (A19 §4.3).

### P31 · A19 R4 · `fix(sim-core): transition derivatives use MuJoCo's tangent convention for quaternion joints`
- **Closes:** A14 §4 item 3 (ball/free position block of `A`).
- **Must-fail:** `ball_transition_matches_mujoco_fd`, `differentiate_pos_tiny_rotation`, `quat_integrate_jacobians` (A19 §3.5).
- **Flips:** none; the `derivatives` validator prints one changed value (A19 §3.4).
- **Breaking:** values of the public `mjd_quat_integrate` and `mj_differentiate_pos` (A19 §3.3, Q3.1). Downstream (A19 §3.6): sim-coupling calls `mj_differentiate_pos` (`articulated.rs:300`) and keeps its own `right_jacobian_so3` (`vjp.rs:127`), whose comment becomes stale; `docs/keystone/quaternion_joints_recon.md:116-119` too.

### P32 · A19 R5 · `fix(sim-core): RK4 keeps a sleeping tree asleep`
- **Closes:** C4 (Q26); Q29; census "sleep off under RK4" (A16 §2), whose doc `d22fcd34` flips at P22.
- **Implements:** A19 §5.2 on top of P22, as a lenient deviation under the wrong-value rule (A19 §5.4). The awake set is taken at the step's start: a tree woken during an RK4 stage stays frozen until the step ends (Q29), where A19's prototype re-reads the live flags at each stage (A19 Q5.1).
- **Must-fail:** `rk4_sleeping_tree_stays_asleep`, `sleeping_pose_equals_fk_of_qpos`, `rk4_sleep_step_does_not_disturb_awake_sensors`; optional `rk4_sleep_matches_mujoco_without_its_wake_defects` (A19 §5.4). For Q29 none is named: no section built a fixture that wakes a tree inside an RK4 step (A19 Q5.1).
- **Flips:** A8's prototype suite gave identical per-test outcomes with the switch on and off (A19 §5.3).
- **Census:** not measured (prototyped on A8's workspace, A19 §8).
- **Divergence row:** under RK4 a sleeping tree stays asleep until a wake rule fires; MuJoCo wakes it (lenient, C4; the three tests above).

### P33 · A20 R3-P1 · `fix(sim-core): muscle force stays -1 in the Model, resolved per call as MuJoCo` — cross-crate (sim-bevy, examples)
- **Closes:** A20 §1A.3.8; A11 §6 items 2, 3, 4 (U04, U05).
- **Implements:** A20 §1A.3.8 (Phase 4 of `compute_actuator_params` deleted; one helper `muscle_f0(prm, acc0)` resolves `scale / max(mjMINVAL, acc0)` per call, in the gain from `gainprm` and in the bias from `biasprm`, `engine_util_misc.c:665-667`, `:708-710`). With it:
  - U04: the muscle clamps `.max(1e-10)` (`forward/actuation.rs:587-589`, `:652-653`; `derivatives/hybrid.rs:155-156`, `:975-976`) become `mjMINVAL` = 1e-15 (`engine_util_misc.c:670-674`, `:713-716`).
  - U05: `BiasType::Muscle` reads `actuator_biasprm` (`actuation.rs:650` reads `gainprm`; MuJoCo `engine_forward.c:459-475`). The builder already writes `biasprm = gainprm` for `<muscle>` (`mjcf/src/builder/actuator.rs:523`).
- **Must-fail:** `general_muscle_f0_matches_mujoco_3_5_0`: A20's fixture `<general gaintype="muscle" biastype="muscle" dyntype="none" … lengthrange="-1 1">`, with `actuator_lengthrange[0] = (-1.0, 1.0)` set in code after load, because the parser drops `lengthrange` until L44 (`builder/actuator.rs:277`, A5 §3.4) → `qfrc_actuator` −6.482484378125003 (A20 §1A.3.8). Main gave +0.69 from the XML; with the range set in code, not run. `general_biastype_muscle_reads_biasprm` (U05: a `<general biastype="muscle">` with its own `biasprm`; A11 measured 100-step `qpos` 2.7e-4 off MuJoCo). `muscle_clamps_use_mjminval` (U04: a lengthrange set in code narrower than 1e-10, `qfrc_actuator` against MuJoCo given the same `actuator_lengthrange`; A11 measured a `mode="none"` muscle 1.7e4 off after 100 steps).
- **Docs:** `Model::recompute_derived`'s doc lists a muscle's `actuator_gainprm[2]` among the inputs consumed on the first computation (`types/model_derived.rs`): delete it.
- **Census:** the muscle cluster (9 docs) flips only with A5-Q6 (L21), A11 LR-a (L44) and this together; alone: 6 `model_fp` changes, 0 trajectory changes on the muscle-shortcut docs (A20 §1A.2, §1A.3.8).
- **Readers of `actuator_gainprm[i][2]`** as the resolved F0 (`git grep`): `examples/fundamentals/sim-cpu/muscles/forearm-flexion/src/main.rs:145`, `muscles/cocontraction/src/main.rs:203`, `muscles/stress-test/src/main.rs:540`, `:733`, `:797`, `:814` (a validator). `sim/L1/bevy/src/model_data.rs:925` reads it for the muscle colour (`.max(1.0)`). They read −1 where F0 was not given, so each calls `muscle_f0`, made `pub` for them. Not run.

### P34 · A18 P1 · `fix(sim-core): Newton and CG run MuJoCo's primal solver`
- **Closes:** census Newton (9 docs) and CG (ledger-L44) clusters (A16 §2; A18 §1).
- **Implements:** A18 §4 (one primal solver for Newton and CG; no PGS fallback; `SolverStat` in MuJoCo's shape; `newton_solved` crate-private per A18 Q2); un-ignore the 24 `golden_flags` tests; `docs/KNOWN_GAPS.md` Gap 1 (A18 §13).
- **Must-fail:** the 24 `golden_flags` tests; `chol_update_minus_removes_row`, `newton_matches_mujoco_3_5_0_iterates`, `newton_has_no_pgs_fallback`, `cg_matches_mujoco_3_5_0`, `cg_keeps_its_iterate`, `solver_niter_cleared_without_rows` (A18 §10).
- **Flips:** `flex_unified.rs:2269` `spec_a_t4_…_bit_identical` (re-capture or tolerance) (A18 §11–§13). Every reader of `newton_solved` outside the solver (`git grep newton_solved`): the GPU-oracle assertion `test_fixtures/conformance.rs:827`, `:937`; `integration/newton_solver.rs:187`, `:1449`, `:1541`, `:1880`; `examples/fundamentals/sim-cpu/solvers/newton/src/main.rs:128`, `:206`, `:244`; the validator `solvers/stress-test/src/main.rs:135`, which tracks the PGS fallback this commit deletes (R1).
- **Checks:** `cargo test -p sim-gpu` is required here: its GPU-vs-CPU suite compares against the solver this commit replaces (`gpu/src/pipeline/conformance_tests.rs`; R5).
- **Census:** 6 of the 9 Newton docs agree (≤ 8.1e-15), 3 move to other clusters (A18 §2.5); `443824ba` agrees (A18 §3.3). 263 corpus docs change bits (A18 §13).
- **Breaking:** `SolverStat` fields; `Data::newton_solved` leaves the public API (A18 §4, Q2).

### P35 · A18 P2 · `feat(sim-core): constraint islands solved separately, as MuJoCo`
- **Implements:** A18 §4 islands (island scale; one solve per island) over P22's partition, the one sleep uses (Q28, Q30). In Rigid (Q49).
- **Must-fail:** `islands_iterate_as_mujoco` (A18 §10).
- **Also** (ledger-L65): MuJoCo builds the constraint rows before `mjcb_control` (`mj_makeConstraint` and `mj_island` in `mj_fwdPosition`, `mj_referenceConstraint` in `mj_fwdVelocity`); sim-core builds them in `forward_acc`, after `cb_control`, so the callback reads the previous pass's rows. Moving the assembly flips the registry row `D-CONTROL-CONSTRAINT-ROWS` and its pin `control_callback_reads_the_previous_pass_constraint_rows`.
- **Flips:** the equality-constraints validator prints `solver_niter` 2 (A18 §11).
- **Census:** 0 verdicts change relative to P34; 50 docs change bits (A18 §4, §13), measured with A18's own partition.

### P36 · A18 P3 · `fix(sim-core): a constraint with an all-zero Jacobian adds no rows (MuJoCo mj_addConstraint)`
- **Closes:** census NEW-EQSTATIC, 5 API docs (A16 §2).
- **Must-fail:** `empty_equality_adds_no_rows` (A18 §10).
- **Census:** 5 of 5 agree; 0 trajectory bits change (A18 §7).
- **Note:** also drops zero-Jacobian flex edge rows, rigid edges among them (A18 §14; A13 §3.4); P36a then skips rigid edges by MuJoCo's rule.

### P36a · A13 D3 core · `feat(sim-core): flex edge rows only through a flex equality, as MuJoCo` — cross-layer (sim-mjcf)
- **Split from Rigid-loading L31:** its sim-core rule lands here; L31 keeps the parse (`<equality><flex>`, `<flexcomp><edge equality solref solimp>`), the `equality="vert"`/`<flexvert>` refusal, the doc rewrites and `flex_edge_rows_match_mujoco_3_5_0`, whose fixture needs C7's vertex bodies (A13 §3.7). A13 §3.6 measured the rule on today's docs with every flex treated as requesting edges (`ISO_FLEX=1`, `research/a13-isolate-dynamics.md:307`): 0 test failures.
- **Closes:** ledger-L39c, sim-core half (A13 §3); Q36.
- **Target** (A13 §3.1): edge rows exist only for a flex named by an active `mjEQ_FLEX` equality, one row per non-rigid edge (`engine_core_constraint.c:616-643`), typed `mjCNSTR_EQUALITY` with `efc_id` = the equality's id and the equality's `solref`/`solimp`; `diagApprox = flexedge_invweight0[e]`, from `mj_setConst` (`engine_setconst.c:716-750`: 0 for a rigid edge, `(1/m₁ + 1/m₂)/2` when both vertex bodies have `body_simple == 2`, `J·M⁻¹·Jᵀ` otherwise).
- **Change** (A13 §3.5; Q36):
  - sim-core: `EqualityType::Flex` (`eq_obj1id` = flex id). `Model::flexedge_invweight0` and `Model::flex_edgeequality` are derived in `recompute_derived` (the flag from the model's `Flex` equalities, which MuJoCo sets at compile, `user_model.cc:3432-3446`). The flex rows leave the `FlexEdge` block (`constraint/assembly.rs:151`, `equality_assembly.rs:77-136`) for the equality loop, as `ConstraintType::Equality` rows; `ConstraintType::FlexEdge` goes. `mj_flex`'s Jacobian skip tests `!flex_edgeequality[f]` in place of `flex_edge_solref[f] == [0, 0]` (`dynamics/flex.rs:37-41`; MuJoCo `engine_core_smooth.c:683-686`). `Model::flex_edge_solref` and `flex_edge_solimp` are removed.
  - sim-mjcf: the builder appends one active `EqualityType::Flex` per flex with the solref/solimp it writes today (`builder/flex.rs:131`, `:148-149`; `compute_edge_solref`, `:753`), so every in-tree flex keeps its rows. L31 then creates the equality only where the doc asks.
- **Must-fail** (built in code; new API, so they do not compile at the parent):
  - `flex_rows_only_with_edge_equality`: A13's 3×3 grid with 3 pins in today's `<deformable><flexcomp>` form (A13 §3.3) → 14 flex rows; with the builder's equality set inactive in code (`model.eq_active[k] = false`) → 0.
  - `flex_edge_diag_approx_is_invweight0`: on the same grid each flex row's `efc_diagApprox` equals `flexedge_invweight0[e]` by the formula above. Parent: `MJ_MINVAL` (`equality_assembly.rs:100`).
- **Flips:** `flex_unified.rs:2353-2364` (finds rows by `ConstraintType::FlexEdge` and reads `efc_id` as an edge index) and `phase13_diagnostic.rs:83` (names `FlexEdge`). A13 measured `ISO_FLEX=1` (rigid edges skipped, `diagApprox` as MuJoCo, the `FlexEdge` label kept) at 0 test failures, 26 deterministic corpus docs changing trajectory bits and 5 changing `nefc` (A13 §3.6), and the label change as trajectory-neutral (A13 Q3). This commit as written was not run.
- **Census:** 0 (no flex doc is loaded by both engines, A16 §3).
- **Breaking:** `EqualityType` gains a variant and is not `#[non_exhaustive]` (`types/enums.rs:331-332`; no exhaustive `match` outside sim-core, A13 §3.5); `ConstraintType::FlexEdge` removed; the two `Model` fields removed (only sim-mjcf writes them: `builder/flex.rs:148-149`, `build.rs:251-252`, `init.rs:270-271`, `mod.rs:721-722`).

### P37 · A18 P4 · `fix(sim-core): solref and solreffriction format checks as MuJoCo getsolparam`
- **Closes:** census `<pair solreffriction>` doc `d81df1fa` (A16 §2; A18 §6.1).
- **Must-fail:** `mixed_solreffriction_uses_solref`, `mixed_solref_uses_default` (A18 §10).
- **Flips:** `unified_solvers.rs:1164-1204` `test_s31…` (rewrite); validator `derivatives/stress-test` check 30 — change its fixture to the direct format in this commit (A18 §11).

### P38 · A18 P5 · `fix(sim-core): elliptic friction rows take MuJoCo's impratio and friction-ratio R; diagApprox from R`
- **Closes:** census doc `2cb92700` (A18 §6.2).
- **Must-fail:** `elliptic_friction_rows_as_mujoco`, `diag_approx_as_mujoco` (A18 §10).
- **Census:** 2 corpus docs change bits (A18 §6.2).

### P39 · A18 P6 · `feat: weld torquescale` — cross-layer (sim-mjcf builder)
- **Implements:** A18 §8.2 (C2: implemented): core scaling of the rotational rows, and the builder writing `eq_data[10] = 1.0` by default in the **same** commit — otherwise every MJCF weld loses its rotational rows. Parsing the attribute is Rigid-loading L41.
- **Must-fail:** `weld_torquescale_matches_mujoco` (A18 §10), with `model.eq_data[k][10] = 0.5` set in code after load: until L41 the parser ignores `torquescale` and the builder writes 1.0 (R1). The XML form of the test moves to L41. **Census:** 0 corpus docs change bits (A18 §8.2).

### P40 · A15 F1 · `fix(cf-geometry): EPA witness points from the closest face; GJK contact at their midpoint (MuJoCo epaWitness)` — cross-crate (sim-core)
- **Closes:** ledger-L42a (A15 §7).
- **Cross-crate:** the EPA witness is cf-geometry's (`Penetration`, `design/cf-geometry/src/query/epa.rs:22`); the GJK contact point is sim-core's (`GjkContact`, `sim/L0/core/src/gjk_epa.rs:61`).
- **Must-fail:** `ellipsoid_rests_on_box_as_mujoco`, `cylinder_rests_on_box_as_mujoco`, `box_cylinder_ellipsoid_rest_on_convex_mesh_slab_as_mujoco`, `epa_contact_point_is_between_the_two_surfaces_in_both_orders`, `epa_witness_difference_is_depth_times_normal` (A15 §6 #2, #3, #4, #6, #7).
- **Flips:** none in the suites run (A15 §5); the mesh-collision validator's F section improves.
- **Census:** measured only together with F2: 1 doc → agree, 2 move from position to frame (A16 §3). **F1 alone: not measured.**
- **Breaking:** semantics of `cf_geometry::Penetration::{point_a, point_b}` and `GjkContact::point` (A15 §4).
- **Note:** still needed after P50 for flex–mesh-hull EPA and cf-geometry's own callers (A17 §11 R3-A). Re-measure the flat-mesh verdict (Rigid-loading L37) on top of it (A15 §4.4, §8).

### P41 · A21 R1 (O) · `fix(sim-core): contacts carry MuJoCo's geom order (lower mjtGeom first)`
- **Closes:** A15 Q6; Q73.
- **Q73** (fix in Rigid): contacts are ordered as MuJoCo lists them, not by A21's `(min(g1,g2), max(g1,g2))` key, which differs for multi-geom bodies and merged `<pair>`s (A21 Q6): body-flex pairs in broadphase signature order `(bf1<<16)+bf2`, with the `<pair>`s merged by `pair_signature` (`engine_collision_driver.c:290-330`), and within a body pair the geom-pair order `contactcompare` gives with the type swap undone (`:226-256`, `:353-370`). The efc rows follow that order; its effect on results was not measured (A21 Q6).
- **Must-fail:** `o_sphere_box_order` (A21 §7.2); `contacts_in_mujoco_body_pair_order`: body 1 holds geoms 0 and 1, bodies 2 and 3 one geom each; each of geoms 0 and 1 touches geoms 2 and 3, and bodies 2 and 3 do not touch → (0,2), (1,2), (0,3), (1,3), where A21's `(min, max)` key gives (0,2), (0,3), (1,2), (1,3) (by reading `engine_collision_driver.c:290-370`; not run). The parent's order was not checked.
- **Flips:** `collision_primitives.rs:340`, `:558`, `:923`, `:968` (A21 §7.2).
- **Census:** 0 class changes (the census matches contacts order-free); 26 corpus docs change bits (A21 §7.2, §3).
- **Breaking:** contact geom order and normal sign (A21 §11). P50–P52 build their contacts in this order (C10).

### P42 · A21 R2 (F) · `fix(sim-core): contact tangent frame as MuJoCo's mju_makeFrame, with the plane–capsule axis hint`
- **Closes:** census NEW-FRAME (28 docs) in part (A16 §2): the other 10 differ from the fused wheel by fused rounding at box–box corners, solver termination and broadphase (A21 §9); against the unfused oracle they are not measured; C9 (Q72).
- **C9** (R4): MuJoCo's `mj_setContact` (`engine_collision_driver.c:1402-1428`) calls `mju_makeFrame` (`:1418`), which normalises the normal in place first (`engine_util_spatial.c:508-514`), after the type-order swap (`:1475-1480`), on every contact path. Ours stores the normal as the collider wrote it, at 6 constructors: `make_contact_from_geoms` (`collision/narrow.rs:424`, frame at `:447`), `make_contact_flex_rigid` / `_flex_self` / `_flex_flex` (`collision/flex_narrow.rs:247`, `:307`, `:356`), `Contact::with_solver_params` (`types/contact_types.rs:180-224`, frame at `:201`) and `Contact::with_condim` (`:243-320`, frame at `:277`); `compute_tangent_frame` normalises a private copy by division (`:342`).
  - A crate-private helper, `pub(crate) fn make_frame(normal: &Vector3<f64>, hint: &Vector3<f64>) -> (Vector3<f64>, [Vector3<f64>; 2])`, normalises once by reciprocal multiply as `mju_normalize3`; all 6 constructors store the normal it returns, and the plane–capsule override uses one `make_frame` call.
  - P41's `order_like_mujoco` negates `normal` and `frame[1]` and keeps `frame[0]` instead of rebuilding the frame, so a swapped contact is normalised once, as in MuJoCo. By reading `mju_makeFrame`, negating equals rebuilding from −n bit for bit.
- **Must-fail:** `f_sphere_sphere_x`, `f_plane_capsule` (A21 §4); `contact_normal_is_normalised_as_mj_setcontact` (input (0,1,1) → stored (0, 0.7071067811865475, 0.7071067811865475); main stores (0,1,1)); guard `swapped_contact_is_normalised_once` (fails if `order_like_mujoco` renormalises) (R4).
- **Flips:** none expected: every in-tree caller of the pub constructors passes a unit axis normal (`constraint/jacobian.rs:635,684,731,805,873,916`, `sensor_tests.rs:172`, `sensor_phase6.rs:343,413,444`) (R4).
- **Census:** +18 (13 by the default rule, 5 by the hint) (A21 §2) and A21 §3's 95 changed corpus docs, both measured without the renormalisation; with it, not measured. Re-bless from a measured run.
- **Divergence row:** a zero or non-finite normal gets a default frame (`contact_types.rs:336-340`) where MuJoCo raises "xaxis of contact frame undefined" (`engine_util_spatial.c:512-514`).

### P43 · A21 R3 (I) + A18 §5 · `fix(sim-core): contacts listed at dist ≤ margin get constraint rows only below margin − gap`
- **Merged:** A18 §5's box–plane per-corner rule (A21 §8 ports `mjc_PlaneBox` in I; A18 §5.4: the rule "should land with or after L41's zero-distance fix", where L41 is ledger-L41).
- **Closes:** ledger-L41 (census L41 14 docs, L41list 30, incl. "box–plane corners at dist +0.35") (A16 §2); A8 §9 O1 (zero-distance contacts); A18 §5 late onset.
- **Must-fail:** `i_sphere_touching_plane`, `i_box_plane_gap`, `i_static_pair` (A21 §8); A18's plane–box test on `830708de` (A18 §10).
- **Flips:** `cg_solver.rs:307` `test_cg_single_contact_direct` (A21 §8).
- **Census:** L41list 29/30, L41 3/14, NEW-LATE +3 (A21 §2). On A18's base the box–plane rule alone brought 5 of 6 late-onset docs, `a0ff89f4`, `1a42c10c`, `74908604` to agree; 18 corpus docs change bits (A18 §5.3).
- **Breaking:** `ncon` now counts excluded contacts; `Contact::is_excluded` is additive (A21 §11). The `contacts_*` helpers count them too (Q71).
- **Note:** excluded contacts make no rows, so they join no island of P22's row partition (Q28, Q30), and A21 §8's separate `island/mod.rs` edit is not needed. The sleep census docs were not re-checked against a sleep fix (A21 §11).

### P43a · Q69 · `fix(sim-core): broadphase as MuJoCo's mj_broadphase, float bounds included`
- **Closes:** Q69 (parity: MuJoCo's broadphase culls touching and barely overlapping pairs, A21 Q1).
- **Now.** An f64 sweep-and-prune with an inclusive test (`forward/position.rs:370-379`, called at `collision/mod.rs:511-512`) lists touching pairs (A21 §9).
- **Target.** `mj_broadphase` and `mj_SAP` (`engine_collision_driver.c:1021-1300`): covariance frame, body AAMMs, bounds cast to float (`:1036-1038`), stable sort (A21 Q1 option (a)). Which comparison drops these pairs was not isolated (A21 §9).
- **Change.** Port both; they produce the broadphase pair list.
- **Must-fail:** `broadphase_culls_touching_spheres_as_mujoco`: MuJoCo's own state of `31833131` at step 6 (overlap 5.6e-17): margin 0 → no contact, margin 1e-6 → one, as MuJoCo lists them (A21 §9, `bp_probe.py`). Main lists the contact at margin 0.
- **Census:** the 5 docs A21 §9 traces to this (`05b1f2d2`, `31833131`, `140b7da3`, `1efe8000`, `eaace9f8`); with the port, not measured.

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
- **Census:** NEW-DEGEN +2, NEW-PAIRCOUNT 1; `f8879911` matches MuJoCo's single contact at t0 (A21 §2, §6); its later residual is solver termination (A21 §9), which this commit isolates (40-verification).

### P46 · A21 R6 (B) · `fix(sim-core): box–box ported from MuJoCo with the driver's box–box filter`
- **Must-fail:** `b_box_box_random` (A21 §10.1).
- **Census:** +2 (one L41, one RK4MOCAP `equality_constraints.rs:1232`); 16 docs change bits (A21 §2, §3). The filter can empty a deep overlap: parity, with a ledger row (Q74; A21 Q7).

### P47 · A21 R7 (G) · `fix(cf-geometry): gjk_distance stops on MuJoCo's Frank–Wolfe gap`
- **Closes:** A15 Q5 (with P50).
- **Must-fail:** `gjk_distance_box_ellipsoid_near_touching`, `g_ellipsoid_box_margin` (A21 §7.1).
- **Census:** 0; no corpus doc changes (A21 §3).
- **Breaking:** semantics of the published `gjk_distance` (A21 §7.1).
- **Note:** P50 deletes the margin-zone branch in `narrow.rs` that G feeds (A17 §11 R3-A); G still serves distance sensors, `closest_point.rs` and the mesh margin zone (A21 §7.1 Downstream, §11).

### P48 · A17 C1 · `feat(sim-core): MuJoCo's native convex collision (mjc_ccd) ported`
- **Implements:** A17 §2.1 in plain arithmetic (C1), unused by dispatch until P50. The module carries `#[allow(dead_code, reason = "dispatch arrives in P50")]`, which P50 deletes: under the hook's `clippy --all-targets -D warnings`, a `pub(crate)` item reached only from tests is dead code in the lib target (R1; not compiled).
- **Must-fail:** `mjc_ccd_matches_mujoco_bitwise` (new function): A17's table of geom types, sizes, poses and margins → dist, x1, x2 and iterations, generated by the unfused oracle (40-verification § The unfused oracle); made to fail by a 1-ulp change (A17 §11 test 1). A17's 3,000-call agreement is the fused port against the arm64 wheel (A17 §2.3); the plain port against MuJoCo's C built without contraction was compared at `6369216c` only, and agreed (A17 §2.2).

### P49 · A17 C2 · `feat(sim-core): mesh polygons and MuJoCo's mesh multi-contact`
- **Implements:** A17 §4.2 / Q4 — **not prototyped**. Its additions carry the same `#[allow(dead_code, reason = …)]` until P50. Must-fail: a box-on-mesh MULTICCD test vs MuJoCo (A17 §12), golden from the unfused oracle.

### P50 · A17 C3 + A21 R8 · `fix(sim-core): convex pairs collide through mjc_Convex as MuJoCo`
- **Merged:** A21 R8 (capsule–cylinder through the convex solver). A21 Q3 recommends landing R8 "together with a port of MuJoCo's native CCD"; A17 C3 routes capsule–cylinder through `mjc_Convex` and deletes `collide_cylinder_capsule` (A17 §11 R3-A).
- **Closes:** census NEW-CONVEX (A16 §2); A15 Q1 MULTICCD perturbation (A17 §4.1); A17 item 4 (tumbling ellipsoid); C10 for convex pairs.
- **C10:** the dispatcher calls each port in MuJoCo's type order and builds the contacts with that `geom1, geom2`; A17's index-order negation of the normal (prototype `narrow.rs:567`) is not written, because P41 already orders contacts as MuJoCo (R4). P48's and P49's `#[allow(dead_code)]` go.
- **Q66, code-built** (stated limitation; L47 refuses the MJCF flag): `try_make_data` refuses a `Model` with `DISABLE_NATIVECCD` set, as `MakeDataError::Unsupported { feature }`. MuJoCo then collides convex pairs with libccd's MPR and `mjc_fixNormal` (`engine_collision_convex.c:822-853`, `:1473`), which we do not port (A17 Q5); today the flag is a silent no-op (`mjcf/src/builder/mod.rs:972`). A flag set on the `Model` after `make_data` is not checked.
- **Must-fail:** `capsule_cylinder_deep_matches_mujoco`, `capsule_cylinder_coaxial_tie_matches_mujoco`, `multiccd_cylinder_on_box_as_mujoco`, `multiccd_skips_ellipsoids`, `ellipsoid3_tumble_rests_as_mujoco`, `convex_contact_at_exact_touch_is_absent` (A17 §11 tests 2–7); `y_capsule_cylinder_parallel` (A21 §10.1); `convex_dispatch_returns_mujoco_order` (`collide_geoms` with geom 0 a cylinder and geom 1 a capsule → `geom1 == 1`, normal from capsule to cylinder; R4); `try_make_data_refuses_disable_nativeccd` (Q66; main builds the `Data`). Under C1, A17 §11 tests 2–7 take their expected values from the unfused oracle, not from the fused wheel.
- **Flips:** `collision_primitives.rs:1090` `cylinder_capsule_perpendicular` (asserts at `:1124`) → MuJoCo's depth; `core/src/collision/mod.rs:1535`, `:1602` (rewritten to MuJoCo's no-graph semantics, a contact from the vertex set: Q68), `:1731` (A17 §10); `pair_cylinder.rs:1086` (A21 §10.2); Q66: `collision/mod.rs:1269` `test_disable_nativeccd_no_crash` (pins the no-op; deleted) and `golden_flags.rs:325` `golden_disable_nativeccd` (un-ignored by P34; becomes a refusal test) (A17 Q5). Without P49, `mod.rs:1367` and the mesh-collision validator's checks 6, 16, 17 are red (A17 §10).
- **Census:** with the fused port against the arm64 wheel: `3a13c90a` state → API, `6369216c` API → agree; agree 796 → 797 on A17's base (A17 §9). Under C1 the plain port takes another EPA face on `6369216c`, 1.4e-6 off the fused wheel's normal and 2.8e-8 in pos (A17 Q1); its agreement with the unfused oracle's golden is not measured, and P50's re-bless decides its verdict.
- **Note:** the A21 Y measurements used cf-geometry's EPA; this merge routes the pair through the port instead (A21 Q3 option (c)).
- **Divergence row:** `DISABLE_NATIVECCD` refused (stated limitation, Q66), here for a code-built `Model` and at L47 for MJCF.

### P51 · A17 C4 · `fix(sim-core): plane–mesh contacts as MuJoCo (midpoint, hull-graph neighbours)`
- **Closes:** census NEW-MESHPLANE (A16 §2).
- **Implements:** A17 §11 R3-B; the plane–mesh contacts are built plane-first (MuJoCo's type order, `mjmodel.h:100-101`), no negation (C10; prototype `mesh_collide.rs:607`).
- **Must-fail:** `mesh_plane_contacts_at_midpoint` (A17 §11 R3-B).
- **Limitation row:** tie order comes from our hull graph, not qhull's (A17 R3-B; 01-decisions hull order).
- **Breaking:** `collide_mesh_plane` (pub) deleted, as A17 R3-B recommends; the comments naming it change (`examples/fundamentals/sim-cpu/mesh-collision/mesh-on-plane/src/main.rs:5`, `integration/mesh_contact_force_diagnostic.rs:170`).

### P52 · A17 C5 · `fix(sim-core): height fields collide as MuJoCo (prism order, native CCD) and are bounded as MuJoCo`
- **Implements:** A17 §11 R3-C sim-core part; bounds in P11's `recompute_derived` (absorbs A17 C7). The data side is Rigid-loading L45. The contacts are built hfield-first (MuJoCo's type order), no negation (C10; prototype `hfield.rs:245`).
- **Must-fail:** `hfield_bounds_are_centred`, `hfield_contacts_match_mujoco_bitwise` (A17 §11 R3-C tests 3, 5). Test 5 sets `model.hfield_data[0]` in code to a square field in MuJoCo's convention (rows bottom-to-top, elevation normalised, f32 values; A17 §5.1 a, b), so it does not wait for L45; its golden comes from the unfused oracle (C1), and it needs P42's renormalised normal (A17 §2.3).
- **Flips:** `collision/hfield.rs:954` `test_swapped_geom_order` (asserts `geom1 == 1`, `geom2 == 0` at `:980-981`; the hfield now comes first) (R4).
- **Note:** the bumpy-field fall-through is fixed by the bounds alone (A17 §5.1); bodies fall through until this lands (A17 §12).

### P53 · K14 · `docs(sim-core): state, callbacks, finite differences and BatchSim documented as they behave`
- **Closes:** core-C1, core-C4, core-B1, core-D1, core-D2, core-D3 (the rest after P04's Display), P-L25, P-L31 FD `B`. core-B2 (the `BatchSim::reset` doc and its pin) is P16's.
- **Implements:** A1 §10's prose; A2 C-5's setters-replace and BatchSim-determinism docs, its doctest and the pin `a_ctrl_writing_control_callback_zeroes_fd_b` (A2 §6, §8); A7 H-4's sim-core prose; A8's K14 notes (the CbPassive line, `island/sleep.rs` module docs) (A8 Dependencies); `docs/KNOWN_GAPS.md` Gap 1 if not in P34 (A18 §16). FD callback counts are documented as they are: equal with and without sensors (P14); pure FD's control and activation columns run the passive callback where MuJoCo's skip the velocity stage, so B holds a passive callback's dependence on `ctrl` (registry `D-FD-CTRL-COLUMNS`, wrong-value: Jon kept the true derivative, chunk 2b's review); the inverse finite differences fire no control callback, as MuJoCo's. With a control callback that writes `ctrl`, pure FD's B is 0 and the hybrid's analytic B is not, and the hybrid takes pure FD under an active constraint (P14): say so where the pin `a_ctrl_writing_control_callback_zeroes_fd_b` is documented.
- **Prose only:** each divergence row lands with the commit that introduces it (Conventions). Not carried, because the decisions removed them: A2 §4's FD count table, measured before P14, and its row "sensor derivatives read sensors at the stepped state" (P14); A7 H-4's Q6 and Q7 rows (P18 refuses both, Q22, Q23).

## Merged, split, dropped

| change | sections | why |
|---|---|---|
| merged → P29 | A19 R2b, A20 R3-P2 | the same change (A19 §2.3; A20 §1A.3.7) |
| merged → P50 | A21 R8 into A17 C3 | A21 Q3 (c): land with the CCD port; A17 C3 routes the pair through it (A17 §11 R3-A) |
| merged → P43 | A18 §5 box–plane into A21 R3 | must land with ledger-L41's zero-distance contacts (A18 §5.4, §16) |
| merged → P33 | U04, U05 into A20 R3-P1 | the same muscle formulas (A11 §6 items 2–4) |
| split → P36a | A13 D3's sim-core rule, from Rigid-loading L31 | 0 test failures under `ISO_FLEX=1` (A13 §3.6); L31 keeps the parse and the doc rewrites |
| merged → P28 | A14 L36b core into A19 R2 | "absorbs A14's L36b core commit" (A19 §2.3) |
| merged → P13 | A2 Q2 into K8 | "Convert to an `Err` in C-3 (same check function)" (A2 Q2) |
| merged → P53 | A1 commit 8, A2 C-5 into K14 | A1 commit list #8 |
| dropped | A15 F2 | superseded by A21 R5 (A21 §6, §11); its tests move to P45 |
| absorbed | A17 C7 into P11/P52 | A17 C7 exists "only if K6 does not take geom bounds" (A17 §12); 11-settled #8 moves geom radii into `recompute_derived` |
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
| A17's fix copy → fused port | 796 → 797 | A17 §9 |

None of these was measured in this PR's order, and each was measured against goldens from the fused arm64 wheel (A20 §2.8 #3), none against the unfused oracle (C1). Every section reports 0 docs moving from agree to differ under its own changes (A16 §3, A17 §9, A18 §1, A19 Summary, A20 §1A.1, A21 §2).

## Conflicts that touch this PR

Resolved 2026-10-07 (header); the last column is the resolution and the commits that carry it.

| id | conflict | sections | resolution → commits |
|---|---|---|---|
| C1 | **FMA.** Emulate the reference build's contraction with `f64::mul_add` (A17 §2.2, Q1: bit-identical to the arm64 wheel; A10 §2.3, Q2.1: fused `eig3` bit-exact) **vs** accept roundoff differences and do not emulate (A18 §3.4, Q6; A21 §1.3, Q2: MuJoCo's own C compiled without contraction equals the ports). A17 §7 also hands off an integrator ulp: MuJoCo's `qvel` update equals `fma(qacc, h, qvel)`. | A10, A17, A18, A21 | Plain arithmetic (Jon); bitwise and census goldens from the unfused oracle (40-verification) → P01, P48, P50, P52; Rigid-loading L03 |
| C2 | **A4 §4's limitation list vs sections that implement items on it:** weld `torquescale` (A4 §4, §8.6 refuse; A13 §1.9 "refused or implemented"; A18 §8.2 implement), `<compiler><lengthrange>` (A11 LR-4), flex `elastic2d` (A9 F3), compiler `inertiagrouprange` (A10 Q2.3), `<equality><flex>` (A13 D3), `<inertial>` `axisangle xyaxes zaxis euler` (A20 §1A.3.9). | A4 vs A9, A10, A11, A13, A18, A20 | Implement all six → P39, P36a; Rigid-loading L31, L32, L33, L41, L43, L49 |
| C3 | **`Data::integrate` signature.** Documented, unchanged (A1 §1, open question (a); 11-settled #7); A13 Q2 picks option B partly because it "keeps `integrate()` returning `()`, as K1 assumes" **vs** `-> Result<(), StepError>` because the sleep step's re-forward can fail (A8 SP-8, Q4). | A1, A8, A13 | `Result`, and `integrate` runs the pre-step check → P22 |
| C4 | **RK4 + sleep.** Parity: MuJoCo's 584 sleep/wake transitions (A8 Q2 (a)) **vs** a lenient deviation that keeps the tree asleep, which "replaces" A8's option (a) (A19 §5.4). | A8, A19 | A19's fix, listed under the wrong-value rule → P32 |
| C5 | **Ball joint `pos ≠ 0`.** Refuse as a stated limitation, ledger row to implement (A14 Q2) **vs** implement on the CPU, refuse or implement on the GPU (A19 §2.3, Q2.1). | A14, A19 | Implement on the CPU; the GPU refuses, as it already does (Q42) → P28 |
| C8 | **sim-core joint-layout check.** `# Panics` stays for `validate_joint_layout` (A1 §6) **vs** make it fallible, `check_joint_layout -> Result<(), JointLayoutError>`, returned by `try_make_data` (A5 §3.3). | A1, A5 | Fallible through `try_make_data` → P10 |
| C9 | **Stored contact normal.** Not renormalised; recommend leave and list (A21 §4, Q5) **vs** renormalisation through `mju_makeFrame` is needed for bitwise hfield/convex normals and is listed as a dependency on NEW-FRAME (A17 §2.3, §12). | A17, A21 | Renormalise once, as `mj_setContact` → P42 |
| C10 | **Contact geom order inside the CCD port.** A17's dispatch negates the normal to keep today's index order, "A15 Q6 stays open" (A17 §2.1) **vs** A21 R1 adopts MuJoCo's type order (A21 §7.2). | A17, A21 | MuJoCo's type order everywhere, no negation in the ports → P41, P50, P51, P52 |
| C11 | **Capsule–box.** A15 F2's two-line fix vs A21 R5's port. | A15, A21 | A21 R5 ("A15's F2 dropped or before R5", A21 §11) → P45 |

C6 and C7 touch Rigid-loading only (see 22-rigid-loading.md).
