# Phase 2 — C1 chunk 1a, Multi-Element FEM (NO SDF) Scope

**Status:** Scope memo — third PR of the six-phase foundation-up sequence. Pre-code recon complete (read-only session 2026-04-26 night); this memo is the draft, ready to execute.
**Date:** 2026-04-26 night, post PR #217 (`788b2178`) — Phase 1 BF-7+BF-8 preemptive book reconciliation MERGED.
**Branch:** `feature/phase-2-multi-element-fem`, off main `788b2178`.
**Master-architect delegation:** re-confirmed 2026-04-26 night, durable through Phase 2 PR merge. See [`feedback_master_architect_delegation`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_master_architect_delegation.md).
**Follows:** `project_gameplan.md` (memory) — Phase 2 of the six-phase foundation-up sequence. First production-code phase since the walking skeleton (PR #214 `fcd559e4`); Phase 0 absorbed by PRs #213+#216; Phase 1 shipped as #217.
**Target:** ~600–1000 net new lines across `sim/L0/soft/src/{mesh,solver,differentiable,observable,readout}/` + 3 new test files; plus ~50-line BF-9 pseudocode rewrite in two Part 5 Ch 00 chapter files. One PR.

Phase 2 takes the **smallest possible step from the walking skeleton's 1-tet scaffold to multi-element FEM**, isolating the new failure surface to *assembly machinery + gradient aggregation* — no SDF, no mesher, no new material, no contact, no GPU. The walking skeleton validated that the seven-trait composition exists and one-element forward+backward+factor-on-tape works. Phase 2 validates that the same composition holds across N elements and across element-shared vertices.

**Why this matters under the sharpened scope (2026-04-24).** Every later phase rests on multi-element machinery being trustable: Phase 3 feeds an SDF-meshed body into the same assembly path; Phase 4 layers materials per-element; Phase 5 contact loads land on multi-element surfaces. If multi-element assembly has a bug, every downstream phase inherits it under load, with the SDF/material/contact noise of those phases obscuring the root cause. Phase 2 isolates the fault domain.

## 0. Baseline — what shipped, what's 1-tet-pinned, what's already generic

Verified via the [Phase 2 readiness recon (read-only, 2026-04-26 night)](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/MEMORY.md#resume-here).

**Already generic, no Phase 2 work required:**
- Seven traits — `Material`, `Element<const N, const G>`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable`. All declare interfaces that don't pin element count.
- Concrete impls of trait *surfaces* (not the assembly that consumes them) — `NeoHookean` (per-Gauss-point on `Matrix3<f64>`), `Tet4` (already const-generic at `Element<4, 1>`), `NullContact`, `BasicObservable` (the trait, not its body), `IndexOp`/`DivOp` autograd ops.
- Chassis side — `Tape::push_custom`, `VjpOp`, `Tensor<f64>`. PR #213 shipped these in their final shape.
- `CpuNewtonSolver<M, E, Msh, C, N, G>` struct generics — six type/const parameters already declared. The struct wrapper is generic; only the bodies are 1-tet-pinned.
- All five γ-locked readout types — `RewardBreakdown`, `RewardWeights`, `EditResult`, `GradientEstimate`, `ResidualCorrections`. Pure data carriers.
- Three of seven walking-skeleton tests — `element_fd.rs`, `material_fd.rs`, `invariant_3_factor.rs`. Element/material/factor-ownership tests are scene-agnostic; carry forward unchanged.

**1-tet-pinned, six concrete sites Phase 2 must extend or rewrite** (four are in-place rewrites — solver assembly / IFT adjoint / reward field / on-tape composition; two are coexistence — `SingleTetMesh` + `one_tet_cube()` stay alongside their new multi-tet siblings per Decision E):

| Site | Cite | What's hardcoded |
|---|---|---|
| Solver assembly machinery | `sim/L0/soft/src/solver/backward_euler.rs` | `N_DOF=12`, `N_FREE=3`, `FREE_OFFSET=9` constants (lines 42-49); `assert!(N==4)` (line 147); `for a in 0..4` loops in `reference_geometry`/`deformation_gradient`/`assemble_internal_force` (lines 170-176, 481-488, 500-509); mass divided by literal `4.0` (line 182); `assemble_free_tangent_block` only emits one vertex's 3-DOF block via `grad_x_n[(3, _)]` (lines 569-592); `free_block_pattern_triplets` is a fixed 3×3 dense lower-triangle (lines 597-605); stack arrays `[f64; N_DOF]` everywhere (lines 282-286, 351-396, 618-654); `external_force` is `v_3`-pinned (lines 521-538). |
| IFT adjoint primal patterns | `sim/L0/soft/src/differentiable/newton_vjp.rs` | Duplicate `N_DOF`/`N_FREE`/`FREE_OFFSET` (lines 70-77); `vjp` hardcodes Stage-1 `[0,0,-1]` and Stage-2 `-I₃` closed forms (lines 166-213). |
| Mesh impl | `sim/L0/soft/src/mesh/single_tet.rs` | Entire file is `SingleTetMesh` (4 vertices, 1 tet). |
| Reward field on the scene | `sim/L0/soft/src/observable/basic.rs` | `FREE_DOF_Z=11` (line 25); `peak_bound = x_final[11]` and `stiffness_bound = θ[0]/x_final[11]` (lines 62-84). |
| On-tape reward composition | `sim/L0/soft/src/readout/skeleton_forward_map.rs` | Duplicate `N_DOF=12`, `FREE_DOF_Z=11` (lines 59-63); `IndexOp::new(FREE_DOF_Z, N_DOF)` pulls one DOF (lines 122-127). |
| Scene constructor | `sim/L0/soft/src/readout/scene.rs` | `one_tet_cube()` constructor (lines 26-41). |

**Book chapters carrying BF-9 drift (eligible for Phase 2 absorption per Decision C):**
- `docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler.md` lines 68-91 — free-standing `pub fn step` pseudocode block.
- `docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler/01-newton.md` lines 61-84 — same pseudocode shape, sub-leaf.

Both blocks pre-date PR #213's `push_custom` chassis API and combine four drift classes (BF-1 `Llt<f64>` → `Llt<I, f64>`; BF-2 incomplete `try_new_with_symbolic` signature; BF-8 `theta: &Tensor<f64>` → `theta_var: Var`; BF-4/BF-6 `tape.record_ift_step(&step)` → `tape.push_custom(&[theta_var], x_final_tensor, Box::new(NewtonStepVjp::new(factor)))`).

## 1. Load-bearing invariants

Three new invariants gate Phase 2; one optional. Each is a code-level check that one specific multi-element claim holds.

| # | Invariant | What green looks like |
|---|---|---|
| **II-1** | **Multi-element assembly determinism scales 1×N to N×1.** | An N-tet mesh whose tets are mechanically isolated (each tet's three corners Dirichlet-pinned, only its `v_3`-equivalent free) produces bit-equal per-tet `x_final` across runs AND bit-equal per-tet `x_final` to the 1-tet skeleton's `x_final` from `tests/solver_convergence.rs::stage_1_traction_converges`. |
| **II-2** | **Gradient aggregation across N elements is correct.** | Same isolated-N-tet scene with one shared Stage-1 θ driving identical `+ẑ` traction on every tet's free vertex. The aggregate gradient `∂(Σ_i v_3_i.z)/∂θ` equals the deterministic-order sum of N copies of the 1-tet skeleton's analytic grad from `tests/invariant_4_5_gradcheck.rs::stage_1_gradcheck_central_fd` (bit-equal under `to_bits` only when summed in the same order the assembly walks elements; per-tet contribution bit-equals baseline). FP non-associativity of summation makes "divide aggregate by N then bit-compare" *not* a valid form of this assertion — the test must instrument per-tet contributions OR re-create the assembly's sum order outside the solver to compute the expected aggregate. |
| **II-3** | **Shared-vertex tangent assembly is correct.** | A 2-tet mesh with one shared face (and therefore three shared vertices, one of which is free), Stage-1 θ on the shared free vertex, FD-vs-analytic gradcheck holds at 5-digit relative-error bar. Catches double-counted Hessian blocks, wrong sparsity pattern, sign errors in shared-vertex accumulation. |
| II-4 *(optional)* | **N-element scaling stays in the band sparse Cholesky predicts.** | Wall-clock + peak heap on the isolated-N-tet scene scale ~O(N) for assembly and ~O(N) for solve (block-diagonal sparsity); not blocking. Recommend-first per `feedback_recommend_first_deep_specialist`. |

**Why these three together are the gate:** II-1 isolates *machinery determinism* (the assembly loop runs N times correctly). II-2 isolates *gradient aggregation* (the VJP path accumulates correctly across N parents-equivalent). II-3 isolates *shared-vertex correctness* (the actual new physics — multi-element vertex sharing). II-1 and II-2 must pass before II-3 is attempted; II-3's failure mode otherwise can't be disentangled from machinery bugs.

The walking-skeleton invariants I-1 through I-6 stay implicitly active — Phase 2 must not regress any of them. Existing tests (`solver_convergence`, `forward_map_gradcheck`, `invariant_3_factor`, `invariant_4_5_gradcheck`, `invariant_6_gpu_probe`, `element_fd`, `material_fd`) stay green.

## 2. Concrete changes — file-by-file plan

### New files

| Path | Contents |
|---|---|
| `sim/L0/soft/src/mesh/hand_built.rs` | `HandBuiltTetMesh` struct + builder. Constructors `two_isolated_tets()` and `two_tet_shared_face()`. Implements full `Mesh` trait. Carries Dirichlet pinned-vertex set + load-application vertex set as struct fields (per Decisions K + L below). |
| `sim/L0/soft/tests/multi_element_isolation.rs` | II-1 test — isolated N=2 tets, per-tet `x_final` bit-equality across runs and against 1-tet baseline. |
| `sim/L0/soft/tests/multi_element_grad_scaling.rs` | II-2 test — isolated N=2 tets, gradient `∂(Σ v_3.z)/∂θ` equals 2 × 1-tet baseline grad bit-equally. |
| `sim/L0/soft/tests/shared_vertex_gradcheck.rs` | II-3 test — 2-tet shared-face, Stage-1 θ on shared free vertex, FD-vs-analytic at 5-digit relative-error bar. |

### Modified files (production code)

| Path | Nature of change |
|---|---|
| `sim/L0/soft/src/solver/backward_euler.rs` | Bulk of Phase 2. Replace `N_DOF`/`N_FREE`/`FREE_OFFSET` constants with runtime-derived values (per Decision F). Replace `[f64; N_DOF]` stack arrays with `Vec<f64>` for full-DOF state (per Decision G); keep `SMatrix<f64, 4, 3>` for per-element locals. Generalize `assemble_free_tangent_block` to accumulate every element's contribution into the sparse free-block via element-vertex incidence (per Decision J). Replace fixed 3×3 `free_block_pattern_triplets` with general sparse pattern built at solver-construction time. Generalize `external_force` to read load map from solver state (per Decision L). Wire Dirichlet pinned-vertex set through `solve_impl` / `factor_at_position` from solver-construction time (per Decision K). Add `for_each_element` loop wrapping `reference_geometry`/`deformation_gradient`/`assemble_internal_force`. |
| `sim/L0/soft/src/differentiable/newton_vjp.rs` | Replace duplicate `N_DOF`/`N_FREE`/`FREE_OFFSET` constants with runtime values stashed in `NewtonStepVjp` at construction. Generalize `vjp`'s closed-form `(∂r/∂θ)_free` patterns to read from a load-map descriptor (per Decision I). |
| `sim/L0/soft/src/observable/basic.rs` | Generalize `reward_breakdown` to compute `peak_bound` over all free DOFs (e.g., max signed-`z` displacement across loaded vertices) — exact reward shape decided at commit time but constraint: no new γ-locked field, no breaking change to `RewardBreakdown` struct, NaN-sentinel pattern preserved on `pressure_uniformity` + `coverage`. |
| `sim/L0/soft/src/readout/skeleton_forward_map.rs` | Generalize `build_reward_on_tape` to mirror the generalized `BasicObservable::reward_breakdown` on-tape. The single `IndexOp::new(FREE_DOF_Z, N_DOF)` becomes either a sum-IndexOp chain or a new `ReduceMaxOp` (commit-time decision; constraint: no chassis change). |
| `sim/L0/soft/src/readout/scene.rs` | Add `n_isolated_tets(n: usize)` and `two_tet_shared_face()` constructors returning `(impl Mesh, SceneInitial)`. Existing `one_tet_cube()` stays as-is (Decision E). |
| `sim/L0/soft/src/mesh/mod.rs` | Re-export `HandBuiltTetMesh` and constructors. |
| `sim/L0/soft/src/lib.rs` | Re-export new types in the public surface block (lines 35-46). Update `SkeletonSolver` doc comment to clarify it's the 1-tet build (multi-tet is a separate alias or inline type per Decision H). |
| `sim/L0/soft/Cargo.toml` | No change expected. Tier metadata stays L0; coverage profile stays integration-only. If Phase 2 implementation surfaces a need to bump deps or features, raise as material plan change per delegation counterweight. |

### Modified files (book — BF-9)

| Path | Edit |
|---|---|
| `docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler.md` | Lines 68-91: rewrite the `pub fn step` pseudocode body to use the post-PR-#213 chassis API. Parameter swap `theta: &Tensor<f64>` → `theta_var: Var`. Type fix `Llt<f64>` → `Llt<I, f64>`. Signature fix on `try_new_with_symbolic` to include explicit `SymbolicLlt` + `Side`. Replace `tape.record_ift_step(&step)` with `tape.push_custom(&[theta_var], x_final_tensor, Box::new(NewtonStepVjp::new(factor)))`. Surrounding prose at lines 92-96 needs minor coherence edits to match the new code block. |
| `docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler/01-newton.md` | Lines 61-84: same rewrite as the parent chapter, sub-leaf version. Surrounding prose at 85-87 likewise needs coherence edits. |
| `sim/docs/todo/soft_body_walking_skeleton_scope.md §13` | Close BF-9 entry — append "**Closed 2026-04-26** by Phase 2 PR (`feature/phase-2-multi-element-fem`) — both Part 5 Ch 00 chapters now use the post-PR-#213 chassis API." Mirror BF-7/BF-8's closure footer pattern. |

## 3. Locked decisions

Master-architect delegation is active per `feedback_master_architect_delegation`; locked here without per-decision user check. Counterweights still apply: confirm before any commit/push.

**Decision A: Gate scene is N=2 only — both isolated and shared-face variants.**
6-tet hex split and 8-tet 2×2×1 are intentionally deferred to a Phase 2 follow-on PR. N=2 is genuinely the minimum that exercises multi-element machinery (II-1/II-2 on isolated) AND the new physics of vertex sharing (II-3 on shared-face). Larger N adds debug surface without a new code path. Per `feedback_baby_steps`.

**Decision B: Three load-bearing invariants — II-1, II-2, II-3 (Section 1).**
II-4 stays optional, recommend-first. Do NOT add an "II-5: layered material" or "II-6: contact" invariant — those belong to Phases 4 and 5 respectively.

**Decision C: Absorb BF-9 into Phase 2.**
Per Phase 1's deferred-decision (`MEMORY.md` resume-here): "Phase 2 scope memo MUST explicitly decide whether to absorb BF-9 — if Phase 2's production code path touches Part 5 Ch 00, BF-9 becomes cheap-and-on-path." Phase 2's production code IS the impl of Part 5 Ch 00 + 01-newton.md. The fix is doc-only swapping (BF-1+BF-2+BF-8+BF-4-class drift, all already characterized in `soft_body_walking_skeleton_scope.md §13`'s BF entries). On-path AND cheap. Per `feedback_explicit_deferrals`. ~50-line two-chapter rewrite, two commits.

**Decision D: Pre-squash tag — yes.**
Phase 2 is expected to land in 8-12 commits (Section 8 sequencing). BF-9 absorption touches book chapters with implicit "post-PR-#213 chassis API" anchors. Both triggers from `feedback_pre_squash_tag` fire. Tag as `feature/phase-2-multi-element-fem-pre-squash` immediately before squash-merge.

**Decision E: Backward compatibility for the 1-tet skeleton — keep.**
`SingleTetMesh`, `SoftScene::one_tet_cube()`, `SkeletonSolver` type alias, and all seven existing tests stay in tree. The 1-tet path becomes a regression net validating that the generalized solver still produces bit-equal walking-skeleton output. Removing it would lose the regression net AND bury the smallest-possible-test-scene primitive every future debugging session benefits from. Cost is ~zero — the generalized solver code path consumes any `Mesh` impl, so `SingleTetMesh` rides through the same code as `HandBuiltTetMesh` without any 1-tet-specific branch.

**Decision F: `N_DOF` / `N_FREE` / `FREE_OFFSET` constants — derive at runtime per-mesh.**
Replace module-level `const N_DOF: usize = 12` (etc.) with values held on `CpuNewtonSolver` at construction time. Computed as `n_dof = 3 * mesh.n_vertices()`; `pinned_dofs` and `free_dofs` are sets derived from the scene's Dirichlet vertex list (Decision K). Self-containment per-module is preserved — each module that needs them stashes them locally; no cross-module export. Matches the skeleton's deliberate self-containment choice (`sim/L0/soft/src/differentiable/newton_vjp.rs:69`).

**Decision G: Stack arrays vs `Vec<f64>` — `Vec<f64>` for full-DOF state, `SMatrix` for per-element locals.**
`x_curr`, `x_prev_arr`, `v_prev_arr`, `f_int`, `f_ext`, `r_full` — all become `Vec<f64>`. Per-element `grad_x_n: SMatrix<f64, 4, 3>` and `tangent: SMatrix<f64, 9, 9>` stay `SMatrix` — they're stack-allocated by the const-generic type, and Tet4's N=4/G=1 keeps them tiny. Trade-off accepted: one heap allocation per `solve_impl` call, in the Newton-iter hot path. At 2-tet scale this is dwarfed by faer's sparse-LLᵀ allocations. Phase E will revisit if profiling shows the alloc dominates.

**Decision H: `CpuNewtonSolver` const generics — keep `<M, E, Msh, C, N, G>`.**
Do NOT add a const for vertex/DOF count. Multi-tet vertex count is mesh-runtime data. Add a new type alias:
```rust
pub type CpuTet4NHSolver<Msh> = CpuNewtonSolver<NeoHookean, Tet4, Msh, NullContact, 4, 1>;
pub type SkeletonSolver = CpuTet4NHSolver<SingleTetMesh>;  // unchanged behavior
```
Keeps the skeleton alias bit-stable (Decision E); multi-tet builds use `CpuTet4NHSolver<HandBuiltTetMesh>`.

**Decision I: θ parameterization for the multi-tet scene — extend, don't replace.**
Stage 1 (length-1 scalar = `+ẑ` traction magnitude) and Stage 2 (length-3 full traction vector) keep their semantics; what changes is *which vertex* receives the load. The solver carries a load-application vertex list (Decision L). For II-1/II-2 isolated tests: every isolated tet's free vertex is in the load list; one shared θ drives the same magnitude into each. For II-3 shared-face test: only the shared free vertex is in the load list. `NewtonStepVjp::vjp`'s closed-form `(∂r/∂θ)_free` patterns generalize to "for each loaded vertex, accumulate `λ[…]` into `parent_cot[…]`." No new chassis op needed; closed forms still expressible per-load-vertex.

**Decision J: Free-block sparsity — drop the fixed 3×3 dense pattern; build per-mesh at construction.**
Replace `free_block_pattern_triplets` with a `build_free_block_pattern(mesh, dirichlet_set)` function called once at solver construction. Walks element-vertex incidence to emit triplets at every (free-DOF, free-DOF) pair where any element couples them. Caches the resulting `SymbolicLlt<usize>` on the solver (matches today's "symbolic factor built once, refactored numerically per Newton iter" pattern). Sparse-by-construction — block-diagonal for isolated tets, banded for shared-face.

**Decision K: Dirichlet boundary — separate scene-config struct, not a Mesh trait method.**
Add a `BoundaryConditions` (or `SceneTopology`) struct holding `pinned_vertices: Vec<VertexId>` (and later, with Phase 5, contact pairs). Pass to `CpuNewtonSolver::new` as a sixth argument. Does NOT extend the `Mesh` trait — Mesh is structural geometry, not boundary conditions. The `SoftScene::*` constructors emit `(impl Mesh, BoundaryConditions, SceneInitial)` instead of today's `(SingleTetMesh, SceneInitial)` 2-tuple. `SoftScene::one_tet_cube()` becomes a 3-tuple variant; signature break is acceptable (single user inside the test fixtures, both updated atomically).

**Decision L: Multi-element load application — same scene-config struct, new field.**
`BoundaryConditions` carries `loaded_vertices: Vec<(VertexId, LoadAxis)>` where `LoadAxis` describes which θ-component drives which DOF. For Stage 1: one `(vertex_id, AxisZ)` per loaded vertex; the single θ scalar broadcasts to `+ẑ` on each. For Stage 2: one `(vertex_id, FullVector)` per loaded vertex; θ is `[3 * n_loaded]` with each triple going to one vertex's xyz. Keeps the load model orthogonal to the mesh impl.

**Decision M: Determinism guarantees — preserve I-5 verbatim.**
The walking skeleton's I-5 (determinism in θ) committed to disabling faer's `rayon` feature, using sorted triplet builds, and avoiding `HashMap` on numeric paths (scope §15 D-1..D-7). All seven mitigations carry forward into the multi-element machinery. Sparse-pattern build per Decision J emits triplets in `(col, row)` sorted order. The newly-required vertex-iteration order in the `for_each_element` loop must be deterministic (use the mesh's natural tet-id ordering, not a HashMap walk). The "I-5 carry-forward" verification rides on R-2 in Section 6 — the 2-tet bit-equality check in `multi_element_isolation`. Larger-N (6/8) determinism is out of scope per Decision A; future PRs scaling N must re-verify I-5 at the new scale.

## 4. Test harness

Phase 2 is Rust-code-heavy; verification surface is `cargo test -p sim-soft` plus `cargo xtask grade sim-soft`.

- **Existing seven tests still pass** — `cargo test -p sim-soft --test element_fd --test material_fd --test invariant_3_factor --test invariant_4_5_gradcheck --test forward_map_gradcheck --test solver_convergence` (skip `invariant_6_gpu_probe` unless on a GPU-equipped machine; CI handles via the gpu-probe feature). Regression net per Decision E.
- **Three new invariant tests pass** — `multi_element_isolation`, `multi_element_grad_scaling`, `shared_vertex_gradcheck`. Each gates one of II-1/II-2/II-3.
- **`cargo xtask grade sim-soft`** stays A under the `integration-only` profile. Coverage may dip from current 12% as new production code lands; integration-only profile lets that hold A. If non-integration-only criteria slip, follow `feedback_xtask_grade_opacity`: per-criterion check first, full grade as final confirmation.
- **`mdbook build`** clean from `docs/studies/soft_body_architecture/` after BF-9 commits — both edited Part 5 Ch 00 chapters resolve their internal links.
- **Grep audit** — `grep -rn "tape.record_ift_step\|Llt<f64>" docs/studies/soft_body_architecture/src/50-time-integration/` returns empty post-BF-9.

## 5. Green checklist

- [ ] Branch `feature/phase-2-multi-element-fem` created off main `788b2178` (done — this commit's parent).
- [ ] All seven existing `sim-soft` tests still green (regression net per Decision E).
- [ ] II-1 test (`multi_element_isolation`) green.
- [ ] II-2 test (`multi_element_grad_scaling`) green.
- [ ] II-3 test (`shared_vertex_gradcheck`) green.
- [ ] `cargo xtask grade sim-soft` A under integration-only.
- [ ] `mdbook build` clean from `docs/studies/soft_body_architecture/` post-BF-9.
- [ ] BF-9 closure footer added to `soft_body_walking_skeleton_scope.md §13`.
- [ ] Pre-squash tag `feature/phase-2-multi-element-fem-pre-squash` created on the multi-commit branch tip immediately before squash-merge (per Decision D).
- [ ] CI Quality Gate green — both `tests-debug` and `grade` jobs pass; CI runtime stays inside ~22-25 min budget.
- [ ] Risk-mitigation pre-merge re-read per `feedback_risk_mitigation_review` and `feedback_thorough_review_before_commit`.
- [ ] Platform-agility invariant: did Phase 2 require modifying chassis (`sim-ml-chassis`)? **Expected: no.** If yes, surface as a material plan change before merge (per `feedback_agility_test`).

## 6. Stress-test rounds

| # | Round | Status | Result / Hook |
|---|---|---|---|
| R-1 | 1-tet vs multi-tet code path overlap audit | **Done pre-code (recon session)** | Six concrete 1-tet specialization sites enumerated in §0; seven trait surfaces and five γ-locked types confirmed already generic. No surprise discoveries expected during Phase 2 implementation; if one surfaces, it's a material plan change per delegation counterweight. |
| R-2 | Determinism (I-5) carry-forward at 2-tet scale | **Live during commit 5 (II-1 test)** | Two-run bit-equality check in `multi_element_isolation` is the explicit detector. Failure mode: rayon re-enabled inadvertently, HashMap leaked into vertex iteration, sparse triplet build out of sorted order. Decision M lists the seven D-1..D-7 mitigations to preserve. |
| R-3 | BF-9 cross-chapter coherence audit | **Live during commits 10 + 11 (BF-9 rewrite)** | Both Part 5 Ch 00 + 01-newton.md chapters share prose surrounding the pseudocode block; rewriting one without the other would create drift. Sequence: rewrite parent chapter first (commit 10), then sub-leaf (commit 11), then re-read both for prose coherence. |
| R-4 | Regression on 1-tet path | **Live during every commit** | After every commit touching `solver/`, `differentiable/`, `observable/`, `readout/`: `cargo test -p sim-soft --test solver_convergence --test invariant_4_5_gradcheck --test forward_map_gradcheck`. Bit-equality of 1-tet output against pre-Phase-2 baseline is the regression detector. |
| R-5 | Risk-mitigation review per `feedback_risk_mitigation_review` | **Live pre-squash-merge** | Specific lenses: (i) does any new heap allocation in `solve_impl` introduce non-determinism via allocator address-dependence? Mitigation: faer's sparse internals are allocator-agnostic at f64 precision; assert via I-5 carry-forward (R-2). (ii) does the load-map descriptor (Decision L) admit a misconfiguration that produces silent NaN (e.g., `loaded_vertices` empty but θ non-empty)? Mitigation: `CpuNewtonSolver::new` validates the descriptor; panic-on-mismatch is the right failure mode for the skeleton-sized API. (iii) does generalizing `peak_bound` to multi-vertex change the walking-skeleton's primal output? Mitigation: 1-tet alias path keeps the single-DOF primal exactly; multi-tet's primal is new, no regression possible. |
| R-6 | Pre-merge cold re-read per `feedback_thorough_review_before_commit` | **Live pre-squash-merge** | Per-leaf-verdict re-read of every modified file, not just the diff. Phase 1's pre-merge re-read caught 3 issues across 5 commits; Phase 2's larger surface justifies the same discipline. |

## 7. What Phase 2 does NOT do

- Does NOT introduce SDF input. Mesh is hand-built per Decision A. SDF→tet bridge is Phase 3.
- Does NOT add new material models. NeoHookean only. Mooney-Rivlin / Ogden / corotational deferred to Phase D per spec §8 / Phase H per gameplan.
- Does NOT add multi-material (per-element `Material` assignment). Phase 4.
- Does NOT add contact. NullContact only. Phase 5.
- Does NOT touch GPU. CPU only. Phase E.
- Does NOT change the seven trait surfaces. New `HandBuiltTetMesh` impl, no trait-method additions.
- Does NOT modify `sim-ml-chassis`. If implementation surfaces a need to, that's a material plan change requiring user surface (`feedback_agility_test` + delegation counterweight).
- Does NOT touch Phase G's `fd_wrapper`, Phase E's `time_adjoint`, or any other unimplemented stub bodies. They stay `unimplemented!("skeleton phase 2")`.
- Does NOT fix BF-1, BF-2, BF-3, BF-4, BF-5, BF-6 (per Phase 1 Decision A precedent — drip-cadence). BF-9 is the only book finding folded in (per Decision C above).
- Does NOT add new γ-locked API types. RewardBreakdown stays four-field; downstream optimizers see the same struct.
- Does NOT remove `SingleTetMesh` or any 1-tet-specific path (Decision E).

## 8. Sequencing within the PR

Twelve commits, foundation-up. Each commit either passes existing tests OR adds new tests that gate the change.

1. **Commit 1: Decision K + L scaffolding.** Add `BoundaryConditions` struct (or chosen name) with `pinned_vertices` + `loaded_vertices` fields. Update `SoftScene::one_tet_cube()` to emit it; update existing tests to consume the new tuple shape. **Expected diff:** ~80 lines net. **Gate:** all seven existing tests still green.
2. **Commit 2: `HandBuiltTetMesh` impl.** New `mesh/hand_built.rs` with `Mesh` trait impl + two constructors. Add `mesh::*` re-exports. Unit tests in the mesh module verify counts/positions/adjacency on both isolated-2 and shared-face-2 builds. **Gate:** new mesh tests green; no solver/observable/readout changes yet.
3. **Commit 3: Solver const-generic alias + signature swap.** Add `CpuTet4NHSolver<Msh>` alias per Decision H. Make `CpuNewtonSolver::new` consume `BoundaryConditions`. Internal const constants stay as today (commit doesn't touch assembly bodies); just plumbing. **Gate:** all seven existing tests still green; 1-tet path unchanged.
4. **Commit 4: Generalize solver assembly machinery.** The biggest commit. Replace `N_DOF`/`N_FREE`/`FREE_OFFSET` constants with runtime values. Replace stack arrays with Vec. Generalize `for a in 0..4` loops over mesh elements. Generalize `assemble_free_tangent_block` and `free_block_pattern_triplets` per Decision J. Generalize `external_force` to read load map. **Gate:** all seven existing tests still green (1-tet path is now exercised through the generalized solver — biggest regression-net moment of the PR).
5. **Commit 5: II-1 isolated-N-tet determinism test.** New `tests/multi_element_isolation.rs`. **Gate:** II-1 green. Failure here means commit 4 introduced an assembly bug; bisect against commit 3 baseline.
6. **Commit 6: Generalize NewtonStepVjp closed forms.** Update `differentiable/newton_vjp.rs` to handle multi-loaded-vertex `(∂r/∂θ)_free` patterns per Decision I. **Gate:** existing tests + II-1 still green.
7. **Commit 7: II-2 gradient-aggregation test.** New `tests/multi_element_grad_scaling.rs`. **Gate:** II-2 green. Failure here means commit 6 has a gradient accumulation bug.
8. **Commit 8: II-3 shared-vertex gradcheck.** New `tests/shared_vertex_gradcheck.rs` against the 2-tet shared-face mesh. May require a follow-on micro-fix to the assembly path (commit 8.5) if a shared-vertex bug surfaces. **Gate:** II-3 green at 5-digit relative-error bar.
9. **Commit 9: Generalize `BasicObservable::reward_breakdown` + `SkeletonForwardMap::build_reward_on_tape`.** Multi-vertex `peak_bound` semantics per §2 row 4-5. **Gate:** existing `forward_map_gradcheck` + new full-stack gradcheck on the 2-tet scene (extend `forward_map_gradcheck` or add a new test).
10. **Commit 10: BF-9 — Part 5 Ch 00 parent chapter pseudocode rewrite.** ~25 lines edited at lines 68-91 plus surrounding prose coherence. **Gate:** `mdbook build` clean.
11. **Commit 11: BF-9 — Part 5 Ch 00 sub-leaf 01-newton.md pseudocode rewrite.** ~25 lines edited at lines 61-84 plus surrounding prose coherence. **Gate:** `mdbook build` clean; cross-chapter coherence audit per R-3.
12. **Commit 12: Close BF-9 in `soft_body_walking_skeleton_scope.md §13` + final cold re-read fixes.** Append closure footer to BF-9 entry. Apply any fixes surfaced by R-5 + R-6 cold re-reads. **Gate:** all of §5 green checklist green.

Tag `feature/phase-2-multi-element-fem-pre-squash` on commit 12's tip. Squash-merge.

## 9. Open decisions

All scope-memo-level decisions are locked in §3 (Decisions A–M). The following live items resolve at commit time, not at scope-memo time:

- **Exact 2-tet shared-face geometry** — which face do the two tets share, what's the resulting Dirichlet pattern, where does the load go on the shared free vertex? Resolve at commit 2 time. Constraint: must produce a non-degenerate shared-vertex Hessian block (avoid placing the shared free vertex along a symmetry axis that zeroes the off-diagonal coupling).
- **Multi-vertex `peak_bound` semantics** — sum-of-z-displacements? max-of-signed-z-displacements? Either preserves the NaN-skip contract on `pressure_uniformity`/`coverage`. Resolve at commit 9 time when generalizing `BasicObservable::reward_breakdown`.
- **Whether `BasicObservable` becomes scene-aware** — should `reward_breakdown` know about the load map (to know which vertices' z-displacements to sample), or is it generic over all free DOFs? Resolve at commit 9 time. Constraint: must not regress 1-tet `forward_map_gradcheck` semantics.
- **Test-data magnitudes** — Stage-1 θ scaling for 2-tet (per-tet load equal to 1-tet's 10 N? Or aggregate 10 N split across 2 tets?). Resolve at commit 5/7 time. Constraint: stay inside the I-5 D-2 / SD-3 well-conditioned band Stage 1 validated.
