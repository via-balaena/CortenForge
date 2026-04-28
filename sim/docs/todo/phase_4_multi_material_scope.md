# Phase 4 — C1 chunk 2, Multi-Material (Layered Bonded) Scope

**Status:** Scope memo — fifth PR of the six-phase foundation-up sequence. Pre-code recon complete (read-only session 2026-04-27, post-merge of PR #219); this memo is the draft, ready to execute.
**Date:** 2026-04-27, post PR #219 (`c3729d4a`) — Phase 3 SDF→tet bridge MERGED.
**Branch:** `feature/phase-4-multi-material`, off main `c3729d4a`.
**Master-architect delegation:** confirmed 2026-04-27, durable through Phase 4 PR merge. See [`feedback_master_architect_delegation`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_master_architect_delegation.md).
**Follows:** `project_gameplan.md` (memory) — Phase 4 of the six-phase foundation-up sequence. Builds on PR #218 (Phase 2 multi-element FEM, `f3f16c8f`) + PR #219 (Phase 3 SDF→tet bridge, `c3729d4a`).
**Target:** ~900–1300 net new lines across `sim/L0/soft/src/{field,material,mesh,sdf_bridge,solver}/` + 7 new test files + 1 forward-looking coordination memo (`cf_design_material_handoff_scope.md`). One PR.

Phase 4 introduces **per-element material assignment as a property of the mesh**, fed by typed spatial `Field<T>` instances aggregated into a `MaterialField`. The layered silicone device (outer Ecoflex 00-30 / middle composite / inner Ecoflex 00-30) is the load-bearing test scene; its bonded-multi-material structure is the simplest realistic instance of the broader machinery the book commits to.

**Why this matters under the sharpened scope (2026-04-24).** Phase 4 is where `sim-soft` stops being a single-material FEM solver and starts being a design surface for spatially heterogeneous soft bodies. The architectural calls made here lock in (or unlock) every later extension: spatially-varying stiffness fields (Part 2 §09 Phase H), HGO anisotropic decoration (Part 2 §06 Phase H), viscoelastic decoration (Part 2 §07 Phase H), thermal coupling (Part 2 §08 Phase H), differentiable design w.r.t. material parameters (Part 6 + Part 10). The book has done the architectural work in Pass-3 sub-leaves; this phase implements what the book commits to without locking out any decorator path.

## 0. Baseline — what's already here, what gets extended, what's net new

Verified via the Phase 4 readiness recon (read-only Explore agent, 2026-04-27 post-merge).

**Already in place, no Phase 4 modification required:**
- `Material` trait at `sim/L0/soft/src/material/mod.rs:14-33` matches the book's four-item commitment exactly (`energy(F) → ψ`, `first_piola(F) → P`, `tangent(F) → ∂P/∂F`, `validity() → ValidityDomain`); `Send + Sync`; `ValidityDomain` six-slot shape per Part 2 Ch 00 §02. **No trait surface change in Phase 4** — composition decorators (`Viscoelastic<M>`, `Anisotropic<M, R>`, `Thermal<M>`) are Phase H and the trait stays scalar-parameterized so they can drop in.
- `NeoHookean { mu, lambda }` impl at `sim/L0/soft/src/material/neo_hookean.rs:27-30`. The single concrete constitutive type Phase 4 ships against; type stays unchanged, but its lifecycle moves from `solver-owns-one-instance` to `mesh-owns-Vec<NeoHookean>`.
- `Mesh` trait at `sim/L0/soft/src/mesh/mod.rs:57-80` — `n_tets`, `n_vertices`, `tet_vertices`, `positions`, `adjacency`, `quality`, `equals_structurally`. Phase 4 *adds* one method (`materials`) with **no default impl** (Decision in §9 — forces explicit migration so every mesh impl carries a real cache, not a fallback empty slice).
- `Sdf` trait at `sim/L0/soft/src/sdf_bridge/sdf.rs` (re-exported from `sdf_bridge/mod.rs:25`) — `eval` + `grad`. Stays geometry-only; the new `Field<T>` trait is parallel, not a sub-trait.
- `BoundaryConditions` (`sim/L0/soft/src/solver/scene.rs`, Phase 2 Decision K) — pass-through; multi-material is orthogonal to Dirichlet/load configuration.
- Phase 3 SDF mesher (`SdfMeshedTetMesh::from_sdf`, BCC + Labelle-Shewchuk Isosurface Stuffing). Mesher stays interface-agnostic; the `MaterialField` is sampled at tet centroids *after* the geometric mesh is produced, not by guiding the mesher itself. **Mesh-aligned-to-interface-SDF is Phase H** (per Part 3 §03's named distinction between graded transitions and topological discontinuities).
- All 103 `sim-soft` tests, three Phase 2 invariants (II-1/II-2/II-3), three Phase 3 invariants (III-1/III-2/III-3) — regression net.

**Existing concrete sites Phase 4 must extend:**

| Site | Cite | What changes |
|---|---|---|
| Solver material storage | `sim/L0/soft/src/solver/backward_euler.rs:113` (struct field `material: M`) | Field deleted; replaced by reads from `mesh.materials()`. `M: Material` generic stays on the struct for now (keeps the type alias `CpuTet4NHSolver<Msh>` valid) but is unused for storage; followup audit may drop it post-Phase-4. |
| Solver `new()` signature | `sim/L0/soft/src/solver/backward_euler.rs:196-203` | First argument `material: M` becomes implicit (read from `mesh.materials()`). API break; all call sites update atomically (limited to test fixtures + `SoftScene::*` constructors). |
| Solver assembly hot points | `sim/L0/soft/src/solver/backward_euler.rs:751` (`first_piola`) and `:808` (`tangent`) | `self.material.first_piola(&f)` → `mesh.materials()[tet_id as usize].first_piola(&f)`. Same for `tangent`. The `tet_id` is already in scope at both sites. |
| `SingleTetMesh` | `sim/L0/soft/src/mesh/single_tet.rs:25` (`pub fn new() -> Self`) | Carry `materials: Vec<NeoHookean>` of length 1. Existing constructor's signature gains a `MaterialField` parameter (exact form per §9 open decision). API break absorbed atomically by SoftScene call sites. |
| `HandBuiltTetMesh` | `sim/L0/soft/src/mesh/hand_built.rs` (named constructors `two_isolated_tets()` + `two_tet_shared_face()`) | Carry `Vec<NeoHookean>` of length `n_tets`. Existing named constructors gain a `MaterialField` parameter (uniform field for existing tests). Commit 5 adds `with_material_field` for explicit multi-region scenes. API break absorbed atomically by SoftScene call sites. |
| `SdfMeshedTetMesh` | `sim/L0/soft/src/sdf_bridge/sdf_meshed_tet_mesh.rs` | Carry `Vec<NeoHookean>` of length `n_tets`; populated during `dispatch_case` by sampling `MeshingHints::material_field` at each emitted tet's centroid. Hints carry an `Option<MaterialField>`; `None` defaults to a uniform synthesized field (Decision in §9 open list). |
| `MeshingHints` | `sim/L0/soft/src/sdf_bridge/mod.rs:59-66` | New optional field `material_field: Option<MaterialField>`. When `None`, all output tets receive a configurable uniform default. |
| Scene constructors | `sim/L0/soft/src/readout/scene.rs` | `one_tet_cube()` / `n_isolated_tets()` / `two_tet_shared_face()` synthesize a `MaterialField::uniform(skeleton_default_mu, skeleton_default_lambda)` internally and pass it to the mesh constructor. New `layered_silicone_sphere()` constructor for IV-5 builds a non-uniform `MaterialField` with `LayeredScalarField` shells. |

**Net-new files:**

| Path | Contents |
|---|---|
| `sim/L0/soft/src/field/mod.rs` | `Field<T>` trait (`fn sample(&self, x_ref: Vec3) -> T`); blanket `Send + Sync` bound. Re-exports `ConstantField`, `LayeredScalarField`, `BlendedScalarField`. |
| `sim/L0/soft/src/field/constant.rs` | `ConstantField<T: Clone>` returns the same value at every point. Generic in `T`. |
| `sim/L0/soft/src/field/layered.rs` | `LayeredScalarField` — N-shell concentric step field over `f64`, keyed on `Box<dyn Sdf>` with sorted threshold list. `BlendedScalarField` — smoothstep / Hermite sigmoid blend between two `Box<dyn Field<f64>>` weighted by an SDF (Part 7 §02 §01 composition pattern). |
| `sim/L0/soft/src/material/material_field.rs` | `MaterialField` aggregator: `mu: Box<dyn Field<f64>>`, `lambda: Box<dyn Field<f64>>`. Method `sample(x_ref) -> NeoHookean`. Constructors `uniform(mu, lambda)`, `from_fields(mu_field, lambda_field)`. |
| `sim/L0/soft/tests/field_unit.rs` | Field trait basics — constant returns same; layered returns per-shell; blended returns smoothstep mid-band. |
| `sim/L0/soft/tests/material_field_sample.rs` | `MaterialField::sample` produces correct `NeoHookean` instances at known points; uniform field bit-equal to `NeoHookean::from_lame` direct construction. |
| `sim/L0/soft/tests/multi_material_passthrough.rs` | **IV-1** — single-region (uniform) `MaterialField` produces bit-equal `x_final` to pre-Phase-4 baseline on Phase 2 multi-element scenes and Phase 3 SDF-meshed scenes. |
| `sim/L0/soft/tests/multi_material_continuity.rs` | **IV-2** — 2-region bilayer hand-built mesh; verify displacement field is C⁰ across the interface under uniform load (FEM-structural; the test is for indexing-bug detection). |
| `sim/L0/soft/tests/bonded_bilayer_beam.rs` | **IV-3** — bonded-bilayer cantilever beam under tip force, compare displacement field to closed-form linear-elastic composite Timoshenko solution. Small-strain hyperelastic regime. |
| `sim/L0/soft/tests/sdf_material_tagging.rs` | **IV-4** — 3-shell `LayeredScalarField` over concentric `SphereSdf`; verify per-tet `NeoHookean` is in expected per-shell parameter set; misclassification rate at shell boundaries decreases monotonically under refinement. |
| `sim/L0/soft/tests/concentric_lame_shells.rs` | **IV-5** — three-shell thick-walled hollow sphere meshed by `SdfMeshedTetMesh`, internal-pressure boundary condition, displacement field matches piecewise-Lamé analytical solution (small-strain regime). |
| `sim/L0/soft/tests/interface_band_flagging.rs` | **IV-6** — interface-tet flag count decreases monotonically under refinement; flag values are accurate (`|φ| < L_e`). |
| `sim/L0/soft/tests/multi_material_validity.rs` | **IV-7** — degenerate one region's element below its `ValidityDomain`; verify region-attributed fail-closed semantics. |
| `sim/L0/soft/tests/material_grad_hook.rs` | **IV-8** — FD-checked ∂(displacement)/∂μ_layer on the layered scene; sets up Part 6 / Part 10 differentiable-design without implementing adjoint. |
| `sim/docs/todo/cf_design_material_handoff_scope.md` | Forward-looking coordination memo: names the in-memory `(SdfField, MaterialField)` boundary per Part 7 §00; lists what cf-design must author when its first multi-material consumer lands. **Does not** implement cf-design changes. |

## 1. Load-bearing invariants

Eight new invariants gate Phase 4. IV-3 and IV-5 are the **load-bearing scientific tests** (analytical comparisons against closed-form continuum solutions); the others are structural / hygiene / regression-net.

| # | Invariant | What green looks like |
|---|---|---|
| **IV-1** | **Single-region passthrough is bit-equal.** | Uniform `MaterialField::uniform(mu, lambda)` produces bit-equal `x_final` to pre-Phase-4 baseline on every Phase 2 (II-1/II-2/II-3) and Phase 3 (III-1/III-2/III-3) test scene. Zero-regression gate. |
| **IV-2** | **Interface continuity (FEM-structural).** | 2-region bilayer hand-built mesh under uniform load: per-vertex displacement at every shared interface vertex is identical when read from either A-side tets or B-side tets (continuity is structural by FEM construction; test catches indexing bugs). |
| **IV-3** | **Bonded-bilayer beam matches Timoshenko closed-form.** | Cantilever bilayer beam (e.g., Ecoflex 00-30 ↔ Ecoflex 00-30 + 15 wt% carbon-black, ratio ~2×) under tip force, hand-built tet mesh, compare displacement field to classical linear-elastic composite-beam solution. Mesh-resolution-bound error at three refinement levels (h, h/2, h/4) shows ~O(h²) convergence. **Book-named regression test** per Part 3 §03 §00. |
| **IV-4** | **SDF-driven region tagging is correct under refinement.** | 3-shell `LayeredScalarField` over concentric `SphereSdf`: every tet's `NeoHookean` instance matches the expected per-shell value at all centroids except a thin band near each shell boundary; the **straddler fraction** (`straddlers / n_tets`) strictly decreases as the mesher's `cell_size` halves. (Fraction not absolute count: 3D interface scaling — total tets grow as `O(1/h³)`, interface-band tets grow as `O(1/h²)`, so absolute count rises under refinement while fraction shrinks as `O(h)`. See `tests/sdf_material_tagging.rs` module doc "Monotonicity is on the straddler *fraction*, not the count" for the derivation.) |
| **IV-5** | **Three-shell concentric Lamé matches analytical.** | Three-shell thick-walled hollow sphere with internal pressure, meshed by `SdfMeshedTetMesh`, materials Ecoflex 00-30 / Ecoflex 00-30 + 15 wt% carbon-black / Ecoflex 00-30 from Part 1 §04. Displacement field matches piecewise-Lamé closed-form solution within mesh-resolution-bound error at three refinement levels. **Pulls IV-3 forward to the full SDF→FEM pipeline.** |
| **IV-6** | **Interface-tet flag is correct and refinement-monotone.** | `Mesh::interface_flags` populated by `|φ(centroid)| < L_e` rule per Part 7 §02 §01. Test: under `cell_size` halving the **flagged fraction** (`flagged / n_tets`) strictly decreases at every refinement level. (Fraction not absolute count, same 3D-scaling reasoning as IV-4 — see `tests/interface_band_flagging.rs` module doc "Why fraction not absolute count" for the derivation.) |
| **IV-7** | **Validity domain composition fails closed with tet attribution.** | Degenerate one region's element by driving its deformation below its declared `ValidityDomain`; solver fails with a structured panic naming the violating tet's id and the validity slot. No silent NaN, no swallowed panic. Layer/region attribution is test-side: the test author knows which `MaterialField` produced the layered scene and which tet_id range belongs to which layer. |
| **IV-8** | **∂(displacement)/∂μ_layer is FD-stable.** | On the layered scene, FD-vs-analytic gradient (numerical perturbation of one layer's μ) at the 5-digit relative-error bar. Doesn't implement reverse-mode adjoint — proves the derivative is well-defined and the API admits parametric tuning. Sets up Part 10 differentiable design. |

The walking-skeleton invariants I-1..I-6 + Phase 2 II-1/II-2/II-3 + Phase 3 III-1/III-2/III-3 stay implicitly active. IV-1 is the regression-net detector for any of them silently breaking.

**Why this set is the gate:** IV-1 isolates *backward-compatibility* (the new architecture doesn't break old scenes). IV-2 isolates *machinery correctness* (multi-material assembly aggregates without indexing bugs). IV-3 isolates *FEM physics correctness* on hand-built meshes (decoupled from SDF mesher noise). IV-4 isolates *mesher integration* (the SDF→tet pipeline tags correctly). IV-5 is the *integrated end-to-end physics gate*. IV-6/IV-7 are *diagnostic/safety hygiene*. IV-8 is the *forward-looking differentiability hook*. IV-3 must pass before IV-5 is attempted; IV-5 failure otherwise can't be disentangled from IV-4 mesher bugs.

## 2. Concrete changes — file-by-file plan

(See §0 tables for new/modified files. Highlights below.)

### Modified production code

| Path | Nature of change |
|---|---|
| `sim/L0/soft/src/solver/backward_euler.rs` | Drop `material: M` field. `new()` no longer takes `material: M`. Lines `:751` and `:808`: replace `self.material.{first_piola,tangent}(&f)` with `mesh.materials()[tet_id as usize].{first_piola,tangent}(&f)`. Generic `M: Material` retained on the struct (Decision G); type aliases unchanged. |
| `sim/L0/soft/src/mesh/mod.rs` | Add `fn materials(&self) -> &[NeoHookean]` to the `Mesh` trait. Phase 4 monomorphizes against `NeoHookean` (Decision G). Phase H may generalize to `&[Box<dyn Material>]` when HGO joins; not done preemptively. |
| `sim/L0/soft/src/mesh/single_tet.rs`, `mesh/hand_built.rs` | Carry `materials: Vec<NeoHookean>`; populate via `MaterialField::sample` at each centroid at construction time. Backward-compat `with_uniform_material(...)` constructors. |
| `sim/L0/soft/src/sdf_bridge/mod.rs` | Add `MaterialField` to `MeshingHints`: `pub material_field: Option<MaterialField>`. When `None`, mesher emits with `MaterialField::uniform(default_mu, default_lambda)`. |
| `sim/L0/soft/src/sdf_bridge/sdf_meshed_tet_mesh.rs` | During `dispatch_case`, after each tet emission compute centroid in reference space and call `material_field.sample(centroid)`; push the resulting `NeoHookean` to `materials`. Stuff this into the new `materials: Vec<NeoHookean>` field. |
| `sim/L0/soft/src/material/mod.rs` | `pub use material_field::MaterialField;`. |
| `sim/L0/soft/src/lib.rs` | Re-export `Field`, `ConstantField`, `LayeredScalarField`, `BlendedScalarField`, `MaterialField` in the public surface block. |
| `sim/L0/soft/src/readout/scene.rs` | Add `layered_silicone_sphere()` (3-shell concentric, Ecoflex 00-30 + composite + Ecoflex 00-30) constructor for IV-5. Existing `one_tet_cube()` / `n_isolated_tets()` / `two_tet_shared_face()` updated to use new mesh constructors via uniform default. |
| `sim/L0/soft/Cargo.toml` | No change expected. Tier metadata stays L0. |

### Book updates (BF-* candidates)

Phase 4 implementation may surface drift between the book's Pass-3 sub-leaves and what's natural in code. The chapters most likely to need touch-up:
- `docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment.md` and sub-leaves — book's `Field<Output = T>` syntax differs from idiomatic Rust `Field<T>`. Decision: code uses `Field<T>` (more idiomatic in 2024-edition Rust); add a one-line clarifying note in the book chapter on first occurrence. Track as **BF-11** if found drifting beyond syntax.
- `docs/studies/soft_body_architecture/src/20-materials/09-spatial-fields.md` — same naming reconciliation if needed.
- `docs/studies/soft_body_architecture/src/30-discretization/03-interfaces/00-bonded.md` — book references `MixedTetMesh` as an aspirational name; current code uses `HandBuiltTetMesh` / `SdfMeshedTetMesh`. Decision: leave the book's aspirational name; add an in-code-comment forward-pointer to the eventual unified type in Phase H. Not a BF.

Any BF surfaced during commit-time stress rounds gets logged per `feedback_explicit_deferrals` and `feedback_pre_squash_tag`.

## 3. Locked decisions

Master-architect delegation is active per `feedback_master_architect_delegation`; locked here without per-decision user check. Counterweights apply: confirm before any commit / push.

**Decision A: `Field<T>` trait is generic in `T` from day one.**
Phase 4 only uses `T = f64` (scalar μ, λ). `T = Vec3` (HGO fiber direction) lands in Phase H. Defining the trait generic now costs one line in the trait declaration and prevents a future breaking change. Per `feedback_no_allergy_to_bold_choices` — this is the more-options call, not the cheaper-now call.

**Decision B: Per-tet `Vec<NeoHookean>` instances on the mesh, not parameter tuples.**
Each tet stores a complete `NeoHookean { mu, lambda }` instance, not a `(mu, lambda)` tuple paired with a shared evaluator. Matches Part 2 §09 §00's "produces a per-element `Material` instance" wording exactly. Performance-equivalent (`NeoHookean` is a 16-byte struct); semantically cleaner. **`Vec<Box<dyn Material>>` is rejected for Phase 4** since only `NeoHookean` exists; revisit at Phase H when HGO/etc. join.

**Decision C: Per-element centroid sampling for Tet4 (book default).**
One `Field::sample` call per tet centroid at mesh-build time. Per Part 7 §02 §00. Matches the one-Gauss-point integration rule for Tet4 with no over- or under-resolution. Per-Gauss-point sampling is Phase H (Tet10).

**Decision D: Two analytic regression tests — IV-3 (Timoshenko bilayer beam) AND IV-5 (concentric Lamé shells).**
IV-3 isolates FEM machinery on hand-built meshes (no SDF mesher noise). IV-5 exercises the full SDF→FEM pipeline. Two analytic comparisons is cheap insurance; running only one would either miss mesher integration bugs (skip IV-5) or miss FEM machinery bugs camouflaged by mesher quality (skip IV-3). Per `feedback_no_allergy_to_bold_choices` — bold here is "ship two analytic tests."

**Decision E: Bonded interfaces only — no mesh alignment to interface SDFs.**
Phase 4 covers continuous-mesh bonded interfaces between layers with modest stiffness contrast (1.2–2.8× per Part 1 §04 carbon-black composite data). Centroid-tag-with-misclassification-band is the documented Phase B default per Part 3 §03 §00. **Mesh-aligned-to-interface-SDF is Phase H** for high-contrast (e.g., silicone↔steel ~10⁴×) scenes. Sliding interfaces are Phase H + IPC.

**Decision F: No anisotropic / viscoelastic / thermal decorators in Phase 4.**
`Anisotropic<M, R>`, `Viscoelastic<M>`, `Thermal<M>` decorators are Part 2 Ch 00 §01 composition rules — Phase H. The trait surface doesn't change in Phase 4; decorators drop in cleanly when their phase arrives.

**Decision G: `NeoHookean` is the only constitutive type in Phase 4.**
`Mesh::materials() -> &[NeoHookean]` (concrete type, not `&[Box<dyn Material>]`). Phase H will generalize to a trait-object slice when HGO etc. join. Solver's `M: Material` generic is technically dead post-Phase-4 (since the mesh holds materials, not the solver) but stays for type-alias compatibility — followup audit may drop it cleanly.

**Decision H: Pre-squash tag — yes.**
Phase 4 is expected to land in 11–13 commits (§8 sequencing). Touches Part 7 Ch 02, Part 2 Ch 09, Part 3 Ch 03 sub-leaves with implicit reconciliation hooks. Both triggers from `feedback_pre_squash_tag` fire. Tag as `feature/phase-4-multi-material-pre-squash` immediately before squash-merge.

**Decision I: Master-architect delegation through merge.**
Confirmed for Phase 4 PR merge. Mirror of Phase 3 delegation. Counterweights apply: project-scope or UX changes surface to user; high-stakes calls flagged; never commit/push without confirmation.

**Decision J: Material parameters pulled from book Part 1 §04 directly.**
IV-3 and IV-5 use Ecoflex 00-30 (small-strain $E$ from the Shore-to-modulus correlation per Part 1 §04 §00, treated as near-incompressible $\nu \approx 0.49$, with explicit $(\mu, \lambda)$ derivation in the test docstring) for the outer/inner shells, and Ecoflex 00-30 + 15 wt% carbon-black composite (mechanical 2.0× multiplier per Part 1 §04 §02) for the middle shell. **Exact $(\mu, \lambda)$ values resolve at commit 8/11 time**, sourced from `appendices/02-material-db.md` if Pass-3 ready, otherwise from the Ecoflex sub-leaf with citation in the test file. **Not** wired through a programmatic database lookup yet (Phase H follow-on); citation in test docstring is sufficient for Phase 4.

**Decision K: Interface-tet flag is diagnostic only, not solver-affecting.**
Per Part 7 §02 §01: "the flag is metadata the downstream passes read." Phase 4 populates `MaterialCache::interface_flags` and exposes a count via API; the Newton hot path does not branch on it. Adaptive refinement (Part 7 Ch 03) is Phase H — the flag's eventual consumer.

**Decision L: cf-design coordination memo only — no cf-design implementation in Phase 4.**
`sim/docs/todo/cf_design_material_handoff_scope.md` describes the in-memory `(SdfField, MaterialField)` boundary per Part 7 §00. Names what `cf-design::Material` (currently `{density, youngs_modulus, color, process}`) needs to expose to produce a `MaterialField`. **Implementation deferred** — no cf-design crate touched in this PR. Deferral named, not silent (per `feedback_explicit_deferrals`).

**Decision M: No new γ-locked API types.**
`MaterialField` is internal-API-shaped (per-parameter `Box<dyn Field<f64>>` slots). The γ-locked `ForwardMap` / `RewardBreakdown` / `EditResult` / `GradientEstimate` / `MeasurementReport` etc. are unchanged. Future Part 10 differentiable-design API may add a γ-class commitment; deferred to that work.

**Decision N: Determinism (I-5) carry-forward at multi-material scale.**
The walking skeleton's seven I-5 mitigations (rayon disabled, sorted triplet builds, no HashMap on numeric paths) carry forward. New: `MaterialField::sample` must be deterministic at each `x_ref` (no thread-local RNG, no `HashMap`-iter-order floats). Field impls (`ConstantField`, `LayeredScalarField`, `BlendedScalarField`) are pure functions of position; satisfied by construction. Verified by IV-1 bit-equality.

**Decision O: Re-sampling vs re-meshing distinction is documented but NOT exposed as a first-class API in Phase 4.**
Per Part 7 §02 §00: material-only edits re-sample; geometry edits re-mesh. Phase 4 *establishes the pattern* — every mesh constructor accepting a `MaterialField` invokes the same per-tet centroid sampling pass, so a future `resample_materials(&MaterialField)` method would just be the second-call form of construction. **An explicit `resample_materials` API ships when Phase H Ch 04 live-remesh begins** (warm-start invalidation patterns are part of that scope, not P4's). Phase 4 commits document the contract in `MaterialField` and per-mesh constructor docstrings — pure-function-of-position determinism + idempotence on identical fields — without a dedicated method.

**Decision P: Backward compatibility — preserved.**
All existing `SoftScene::*` constructors stay in tree with the same shape; they emit meshes carrying `MaterialField::uniform(default_mu, default_lambda)` cached. Phase 2/3 invariant tests run *through* the new architecture and are the regression net per Decision E precedent. `SkeletonSolver` type alias unchanged.

**Decision Q: Validity-domain composition — first-violator-wins, structured panic with tet attribution only.**
The solver's existing failure mode is panic-on-unrecoverable (matches Phase 2's "construction-time panic-validate" pattern, e.g., load-on-orphan-vertex assertions). Phase 4 extends this: at step start, the solver iterates per-tet materials and checks `validity()` against the current deformation gradient; the first violation panics with a structured message naming `(tet_id, validity_slot_violated)`. **No new `SolverError` enum, no new `Result`-returning surface, no γ-locked API change** (per Decision M). The `MeshingError` enum is mesher-side and unchanged. **No `RegionLabel` enum, no parallel `Vec<RegionLabel>` on the mesh** — layer attribution is test-side (the test author knows which `MaterialField` they passed to mesh construction and can correlate `tet_id` back to layer). Phase H may add explicit region propagation when adaptive refinement / live re-mesh blur the tet_id → layer mapping, but Phase 4 doesn't need it.

## 4. Test harness

Phase 4 is Rust-code-heavy; verification surface is `cargo test -p sim-soft` plus `cargo xtask grade sim-soft`.

- **Existing 103 sim-soft tests still pass** — full regression net per Decision P. Failure of any pre-Phase-4 test is the IV-1 zero-regression gate failing.
- **Eight new invariant tests pass** — IV-1 through IV-8.
- **`cargo xtask grade sim-soft`** stays A across 7 automated criteria under the `integration-only` profile. Coverage may dip on the soft crate as new production code lands; integration-only profile lets that hold A. Per `feedback_xtask_grade_opacity`: per-criterion check during iteration, full grade as final confirmation.
- **`mdbook build`** clean from `docs/studies/soft_body_architecture/` post any BF-* edits.
- **Grep audit** — `grep -rn "self\.material\." sim/L0/soft/src/solver/` returns empty post-Decision G implementation. `grep -rn "material: M" sim/L0/soft/src/solver/backward_euler.rs` returns only the type-alias-compat generic, not the deleted field.
- **CI Quality Gate** — fast tier green by ~5 min, test matrix ~20 min per `reference_ci_timing`. Total budget ~22m unchanged.

## 5. Green checklist

- [ ] Branch `feature/phase-4-multi-material` created off main `c3729d4a`.
- [ ] All 103 existing `sim-soft` tests still green (regression net per Decision P).
- [ ] IV-1 (passthrough) green on every Phase 2 + Phase 3 invariant scene.
- [ ] IV-2 (interface continuity) green.
- [ ] IV-3 (Timoshenko bilayer beam) green at three refinement levels with O(h²) convergence.
- [ ] IV-4 (SDF region tagging) green with monotonic misclassification reduction.
- [ ] IV-5 (concentric Lamé shells) green at three refinement levels with O(h²) convergence.
- [ ] IV-6 (interface flag) green.
- [ ] IV-7 (validity composition) green.
- [ ] IV-8 (∂u/∂μ_layer FD) green at 5-digit relative-error bar.
- [ ] `cargo xtask grade sim-soft` A across 7 automated criteria under integration-only profile.
- [ ] `mdbook build` clean from `docs/studies/soft_body_architecture/` post any BF-* edits.
- [ ] `cf_design_material_handoff_scope.md` authored and lints-clean.
- [ ] Pre-squash tag `feature/phase-4-multi-material-pre-squash` created on tip immediately before squash-merge (per Decision H).
- [ ] CI Quality Gate green — `tests-debug` and `grade` jobs both pass; runtime inside ~22-25 min budget.
- [ ] Risk-mitigation pre-merge re-read per `feedback_risk_mitigation_review` and `feedback_thorough_review_before_commit`.
- [ ] Platform-agility invariant: did Phase 4 require modifying `sim-ml-chassis`? **Expected: no.** If yes, surface as material plan change (per `feedback_agility_test`).

## 6. Stress-test rounds

| # | Round | Status | Result / Hook |
|---|---|---|---|
| R-1 | Pre-code recon — current `Material` / `Mesh` / `Sdf` / solver shape vs book commitments | **Done pre-memo (Explore agent + scope-memo cold re-read, 2026-04-27)** | `Material` matches book exactly; `Mesh` extension is one new method (no default impl); `Sdf` stays separate from new `Field<T>`; solver hot points isolated to two lines (`backward_euler.rs:751` + `:808`). Drift caught in cold re-read: `Sdf` is in `sdf_bridge/sdf.rs` not `mod.rs` (citation fixed); `SingleTetMesh::new()` takes zero args today (existing constructors must gain a `MaterialField` param — API break absorbed by SoftScene atomically per Decision P precedent). No further surprises expected during implementation; if one surfaces, material plan change per delegation counterweight. |
| R-2 | Determinism (I-5) carry-forward at multi-material scale | **Live during commit 5 (IV-1 test)** | IV-1 bit-equality on Phase 2 II-1 + Phase 3 III-1 scenes is the explicit detector. Failure mode: `MaterialField::sample` introduces non-determinism (e.g., a `HashMap` iteration in `LayeredScalarField`'s shell sort), or `Vec<NeoHookean>` allocation pattern leaks through faer. Decision N's mitigation list applies. |
| R-3 | Book ↔ code naming reconciliation (`Field<Output = T>` vs `Field<T>`, `MixedTetMesh` aspirational) | **Live during commits 1-3** | Decision: code uses idiomatic `Field<T>`; book chapter additions or one-line clarifying notes per §2 "Book updates" above (book's typed-output style is informational, code uses generic-T). Track as BF-11 only if drift extends beyond syntax. |
| R-4 | Regression on Phase 2 + Phase 3 paths | **Live during every commit** | After every commit touching `solver/`, `mesh/`, `sdf_bridge/`, `material/`, or `field/`: `cargo test -p sim-soft --tests`. Bit-equality of pre-Phase-4 invariants is the regression detector. |
| R-5 | Risk-mitigation review per `feedback_risk_mitigation_review` | **Live pre-squash-merge** | Specific lenses: (i) Does dropping `material: M` from the solver leave dead generic-bound code that bit-rots? Mitigation: explicit followup task to audit / drop `M: Material` from `CpuNewtonSolver` post-Phase-4 (Phase H candidate). (ii) Does `MeshingHints::material_field: Option<MaterialField>` admit a misconfiguration where `None` produces silently-wrong material? Mitigation: `None` defaults to `MaterialField::uniform(NeoHookean::skeleton_default())` which is loud-named and matches walking-skeleton semantics; documented explicitly in the field's docstring. (iii) Does centroid-sampling produce IV-3-failing aliasing at the bilayer beam interface for low-resolution meshes? Mitigation: convergence study at three refinement levels (h, h/2, h/4) — if h-level misses Timoshenko within bounds, flag as a Phase H interface-aware-mesher trigger, NOT a Phase 4 fix-on-the-fly. |
| R-6 | Pre-merge cold re-read per `feedback_thorough_review_before_commit` | **Live pre-squash-merge** | Per-leaf-verdict re-read of every modified file, not just the diff. Phase 2's pre-merge re-read caught 3 issues across 5 commits; Phase 3's caught grader fixups (commit 12 / `310b4f9d`). Phase 4's larger surface (~13 commits) justifies the same discipline. |

## 7. What Phase 4 does NOT do

- Does NOT add new constitutive models. NeoHookean only. HGO / Mooney-Rivlin / Ogden / Anisotropic / Viscoelastic / Thermal are Phase H.
- Does NOT add per-Gauss-point sampling for Tet10. Tet10 element type is Phase H.
- Does NOT add interface-aware mesh refinement. The BCC mesher stays interface-agnostic; centroid-tag with misclassification band is the Phase B default per Part 3 §03 §00.
- Does NOT add adaptive refinement based on the interface flag. Flag is diagnostic-only per Decision K. Adaptive refinement is Phase H.
- Does NOT add sliding interfaces or IPC. Phase 5 + Phase H.
- Does NOT touch `cf-design` crate. Coordination memo only per Decision L.
- Does NOT add electrical or thermal coupling for the carbon-black composite middle layer. Mechanical-only per gameplan §Phase 4.
- Does NOT touch GPU. CPU only. Phase E.
- Does NOT modify `sim-ml-chassis`. If implementation surfaces a need to, that's a material plan change requiring user surface (`feedback_agility_test` + delegation counterweight).
- Does NOT add new γ-locked API types per Decision M.
- Does NOT remove `SingleTetMesh`, `HandBuiltTetMesh`, or any Phase 2/3 path (Decision P).
- Does NOT implement reverse-mode adjoint for material-parameter gradients. IV-8 is FD-only; full adjoint is Part 6 / post-Phase-H work.

## 8. Sequencing within the PR

Thirteen commits, foundation-up. Each commit either passes existing tests OR adds new tests that gate the change.

1. **Commit 1: `Field<T>` trait + `ConstantField<T>`.** New `sim/L0/soft/src/field/mod.rs` and `field/constant.rs`. Re-exports in `lib.rs`. Unit test in `tests/field_unit.rs` (`ConstantField` only). **Gate:** all existing tests still green; new field unit test green.
2. **Commit 2: `LayeredScalarField` + `BlendedScalarField`.** New `field/layered.rs`. Extend `tests/field_unit.rs` with concentric-shell + smoothstep-blend cases. **Gate:** field unit tests green.
3. **Commit 3: `MaterialField` aggregator.** New `material/material_field.rs` with `uniform(...)` and `from_fields(...)` constructors and `sample(x_ref) -> NeoHookean`. New `tests/material_field_sample.rs`. **Gate:** sampling tests green.
4. **Commit 4: `Mesh::materials` trait extension + per-mesh `Vec<NeoHookean>` storage + constructor API break.** Add the trait method to `Mesh` (no default impl per §9 recommendation — forces explicit migration). Update **all three** existing mesh impls — `SingleTetMesh`, `HandBuiltTetMesh`, `SdfMeshedTetMesh` — to carry `materials: Vec<NeoHookean>`. Existing mesh constructors gain a `MaterialField` parameter (per §9 open decision; `MaterialField` recommended over `NeoHookean` direct for unified call shape). All SoftScene call sites updated atomically to synthesize `MaterialField::uniform(skeleton_default_mu, skeleton_default_lambda)` and thread it through. `SdfMeshedTetMesh` reads its field from `MeshingHints` (commit 9 adds the field; commit 4 uses a uniform synthesized default if hints don't carry one — i.e., commit 4 ships the uniform-default fallback inside `MeshingHints` construction, not the consumer-facing `material_field` field). **Gate:** all 103 existing tests still green (Phase 2 II-* + Phase 3 III-* go through new architecture but with uniform field; no semantic change). **API break gate:** zero `cargo` warnings about unused `material: M` in solver (still present at this commit, used at `:751` + `:808`); `cargo build -p sim-soft` clean.
5. **Commit 5: `HandBuiltTetMesh::with_material_field` named constructor + IV-1 passthrough test.** Add an additional named constructor on `HandBuiltTetMesh` for explicit multi-region scenes (e.g., a 2-region bilayer with two distinct `LayeredScalarField` regions). Differs from commit 4's path only in test ergonomics — multi-region tests don't have to construct a `LayeredScalarField` from inside the test. Commit 4's uniform-field default still drives every existing test. New `tests/multi_material_passthrough.rs` runs Phase 2 II-1/II-2/II-3 + Phase 3 III-1/III-2/III-3 on uniform `MaterialField`, asserts bit-equality to pre-Phase-4 baseline. **Gate:** IV-1 green.
6. **Commit 6: Solver assembly migration + IV-1 re-confirmation.** Drop `material: M` field from `CpuNewtonSolver`. `new()` no longer takes `material: M`. `:751` and `:808` read from `mesh.materials()[tet_id as usize]`. **Gate:** IV-1 still green (this is the structural moment).
7. **Commit 7: 2-region bilayer + IV-2 interface continuity.** Extend `HandBuiltTetMesh` constructors to support 2-region tagging via a per-tet material field. New `tests/multi_material_continuity.rs`. **Gate:** IV-2 green.
8. **Commit 8: IV-3 bonded-bilayer beam (Timoshenko).** New `tests/bonded_bilayer_beam.rs`. Hand-built bilayer cantilever, three refinement levels, O(h²) convergence assertion. Material parameters from Part 1 §04 (Decision J). **Gate:** IV-3 green; **load-bearing scientific commit**.
9. **Commit 9: `MeshingHints::material_field` extension + `SdfMeshedTetMesh::from_sdf` consumes it.** Add the optional `material_field` field on `MeshingHints` and thread it through the BCC dispatch loop so non-uniform `MaterialField` instances populate the per-tet cache (commit 4 already wired uniform-default adoption on `SdfMeshedTetMesh`). **Gate:** existing III-1/III-2/III-3 still green; new uniform-field SDF-meshed scene matches pre-Phase-4 baseline.
10. **Commit 10: IV-4 SDF region tagging.** New `tests/sdf_material_tagging.rs`. 3-shell concentric `SphereSdf` + `LayeredScalarField`; verify per-tet labels at three refinement levels. **Gate:** IV-4 green.
11. **Commit 11: IV-5 concentric Lamé end-to-end.** Add `SoftScene::layered_silicone_sphere` constructor. New `tests/concentric_lame_shells.rs`. Material parameters from Part 1 §04 (Decision J). **Gate:** IV-5 green; **load-bearing scientific commit**.
12. **Commit 12: IV-6 interface-tet flagging + IV-7 validity composition.** New `tests/interface_band_flagging.rs` and `tests/multi_material_validity.rs`. Implement `MaterialCache::interface_flags` (populated when `MaterialField` carries an interface SDF reference) and the region-attributed validity error path. **Gate:** IV-6 + IV-7 green.
13. **Commit 13: IV-8 gradient hook + cf-design coordination memo + scope memo close-out + grader fixups.** New `tests/material_grad_hook.rs` (FD-checked ∂u/∂μ_layer). New `sim/docs/todo/cf_design_material_handoff_scope.md`. Apply any cold re-read fixes from R-5 + R-6. Final `cargo xtask grade sim-soft` A-across-7 confirmation. **Gate:** all of §5 green checklist green.

Tag `feature/phase-4-multi-material-pre-squash` on commit 13's tip (or commit 12 if grader fixups land in 13). Squash-merge.

## 9. Open decisions

All scope-memo-level decisions are locked in §3 (Decisions A–Q). The following live items resolve at commit time, not at scope-memo time:

- **Exact 2-region bilayer geometry for IV-2 / IV-3** — beam aspect ratio, layer thickness ratio, tip force magnitude. Resolve at commit 7/8 time. Constraint: must produce a mesh-resolution-recoverable Timoshenko prediction at three refinement levels (h, h/2, h/4) with O(h²) convergence.
- **Mesh refinement levels for IV-3 / IV-5** — concrete `cell_size` values for h, h/2, h/4. Constraint: h/4 must complete in < 60s on the dev machine to keep CI runtime inside budget. Resolve at commit 8/11 time.
- **`Mesh::materials` default impl** — `default_impl_returning_empty_slice` (forces all impls to opt in via deprecation warning) vs no-default (compile error forces immediate migration). Resolve at commit 4 time. Recommend no-default (forces explicit migration; cleaner). Bookkeeping cost ~5 lines per `Mesh` impl.
- **Mesh constructor signature shape** — `material: NeoHookean` (simple, matches today's solver call shape) vs `material_field: MaterialField` (unified across uniform + multi-region scenes; commit 3 already lands the type). Resolve at commit 4 time. Recommend `MaterialField` — same call shape across uniform and multi-region scenes; uniform is just `MaterialField::uniform(...)`; no double-API.
- **`MaterialField::sample` return type** — concrete `NeoHookean` (Decision B) vs generic `M: Material` via a constructor closure. Decision B locks concrete; this entry is a sanity check at commit 3 to confirm there's no Phase H painful-corner the closure variant would avoid. Expected: no, concrete stays.
- **Test material-parameter values for IV-3 (the bilayer)** — Ecoflex 00-30 + 15 wt% carbon-black is an Ecoflex 00-30 ↔ Ecoflex 00-30 × 2.0 ratio. Alternative: Ecoflex 00-30 ↔ Dragon Skin 30A for ~10× ratio (more discriminating but tougher mesh-resolution requirement). Resolve at commit 8 time. Recommend 2.0× for Phase 4 (matches the layered silicone device's actual middle layer); Dragon Skin pairing is a Phase H follow-on stress test.

---

**Forward citations (Phase H placeholders this scope memo names):**
- Per-Gauss-point Tet10 sampling — Phase H (Part 7 §02 §00 last bullet).
- Anisotropic / viscoelastic / thermal decorators — Phase H (Part 2 §06 / §07 / §08).
- Interface-aware mesh refinement — Phase H (Part 7 Ch 03).
- Mesh-aligned-to-interface-SDF — Phase H (Part 3 §03 §00 high-contrast extension).
- Sliding interfaces + IPC — Phase H (Part 3 §03 §01) + Phase 5.
- Material-parameter reverse-mode adjoint — Part 6 / post-Phase-H.
- cf-design `MaterialField` authoring — coordination memo only; implementation post-Phase-4.
- Drop `M: Material` generic from `CpuNewtonSolver` — followup audit, low priority.

**Citations:**
- Book Part 7 §00 SDF primitive (`docs/studies/soft_body_architecture/src/70-sdf-pipeline/00-sdf-primitive.md`)
- Book Part 7 §02 material assignment (`02-material-assignment.md` + `02-material-assignment/00-sampling.md` + `02-material-assignment/01-composition.md`)
- Book Part 2 §00 trait hierarchy (`20-materials/00-trait-hierarchy.md` + `00-trait-hierarchy/00-trait-surface.md` + `00-trait-hierarchy/01-composition.md`)
- Book Part 2 §09 spatial fields (`20-materials/09-spatial-fields.md` + `09-spatial-fields/00-sdf-valued.md` + `09-spatial-fields/01-sampling.md`)
- Book Part 2 §06 anisotropic (`20-materials/06-anisotropic.md`) — forward citation only
- Book Part 3 §03 interfaces (`30-discretization/03-interfaces.md` + `03-interfaces/00-bonded.md`)
- Book Part 1 §04 material data (`10-physical/04-material-data.md` + `04-material-data/00-ecoflex.md` + `04-material-data/02-carbon-black.md`)
- Phase 2 scope memo (`sim/docs/todo/phase_2_multi_element_fem_scope.md`) — structural template
- Phase 3 scope memo (`sim/docs/todo/phase_3_sdf_tet_bridge_scope.md`) — structural template
