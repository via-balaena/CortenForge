# sim-soft — Tet10 quadratic-element implementation plan

> **Authored 2026-07-22.** The implementation roadmap for adding the
> **Tet10** (10-node quadratic tetrahedron) element to `sim-soft`, written
> after a full-surface recon at `main @ 42b0e191` and immediately after the
> F-bar near-incompressible arc (#677) landed and was measured against an
> analytic oracle. Status: **QUEUED, not started — this doc is the plan the
> build follows, not a record of work done.**
>
> The element *design* already lives in the spec book
> ([`30-discretization/00-element-choice/01-tet10.md`](studies/soft_body_architecture/src/30-discretization/00-element-choice/01-tet10.md)):
> 10 nodes (4 corners + 6 edge-midpoints), quadratic shape functions,
> 4-point Stroud Gauss rule, 30×30 element stiffness, per-Gauss-point
> material sampling. This doc is the *how-we-port-it*: what the current
> code actually requires, in what order, gated by which oracles.
>
> **TL;DR.** Tet10 is the accurate soft-element foundation — it resolves the
> curved contact patch (the ~9% Tet4 floor #676 documented) and is the
> established higher-order path for near-incompressibility (ν≈0.49 Ecoflex),
> where the shipped Tet4 F-bar is a *qualitative* cure only (~5–21%
> over-soft, see §1). The port is **large and additive-in-name-only**: the
> const-generic `Element<N,G>` boundary is real, but `ElementGeometry`, the
> assembly/sensitivity/F-bar kernels, the `Mesh` connectivity type, and the
> centroid material cache are all concretely Tet4 below it. The build order
> is designed to reach the **Lamé + #676 oracle validation as early as
> possible** (fail-fast on whether pure Tet10 is accurate at ν=0.49), then
> harden the gradient path (the silent-wrong risk) under FD gates.

---

## 1. Why Tet10 — the two demands, and the honest accuracy expectation

Two independent gaps motivate Tet10:

- **Demand #1 — the ~9% Tet4 contact-patch floor.** The #676
  `bonded_layer_indentation` gate proved (3-level cell convergence) that
  Tet4 over-stiffens the curved contact patch by a *mesh-converged* ~9%.
  Quadratic shape functions resolve the curvature Tet4's constant strain
  cannot. **Tet10 serves this definitively** — it is the element the #676
  gate docstring itself names as the cure.

- **Demand #2 — quantitative near-incompressible (ν→0.49) accuracy.**
  Every standalone soft gate runs at ν=0.4 because Tet4 volumetrically
  locks as ν→0.5, and Ecoflex 00-30 is ν≈0.49.

**The honest accuracy expectation for demand #2 (do not overclaim):** Tet10
substantially improves near-incompressibility (≈2.5× the Tet4
DOF-to-constraint ratio), but the spec is explicit that *quadratic
displacement alone does not fully cure volumetric locking at strict
incompressibility* — ν→0.5 wants a mixed formulation (Taylor-Hood P2-P1) on
top ([`05-incompressibility/01-mixed-up.md`](studies/soft_body_architecture/src/20-materials/05-incompressibility/01-mixed-up.md)).
At **ν=0.49** (milder than the 0.499 the spec caveats), pure-displacement
Tet10 is *plausibly* accurate-enough — but that is exactly what the Lamé
oracle must decide **empirically and early** (§4), not an assumption to build
on. Two consequences: (a) the F-bar cure **does not carry over** — its whole
premise is the single-Gauss-point per-element volumetric constraint
(`fbar.rs`), which multi-Gauss-point Tet10 breaks; (b) if pure Tet10 is not
accurate-enough at ν=0.49, the cure-on-top is Taylor-Hood P2-P1, not F-bar.

**Why not the cheaper Tet4-only cures (recorded so we don't relitigate):**
the shipped **nodal-averaged F-bar over-softens** (measured vs the Lamé
oracle: ~5% at ν=0.4 → ~21% at ν=0.49, mesh-converged; the nodal-star patch
average over-relaxes the volumetric constraint — a *scheme* property, not a
material or solver bug; a dev/vol-split material was spiked and gives
identical results at this strain, so it is not the material). **mixed-u-p
P1-P0 on Tet4** is the spec's stated primary cure but is theoretically
finicky for tets (inf-sup stability). Both are lower-confidence than the
established higher-order path. Tet10 is chosen for *quality/ceiling*: it is
the textbook-robust near-incompressible element **and** the only cure for
demand #1.

---

## 2. What the port touches — recon sizing (`main @ 42b0e191`)

"Tet10 is additive" is aspirational. The const-generic
`CpuNewtonSolver<E, Msh, C, M, const N, const G>` and the `Element<N,G>` /
`Material` traits are genuinely element-agnostic — but the const generics are
**decorative below `new()`**: `construct.rs::new` hard-asserts `N == 4`, and
every assembly body ignores `N`/`G` and uses concrete Tet4 types.

| Area | Anchor | Size / risk |
|---|---|---|
| `Element<10,4>` impl (shape fns + Stroud table) | new sibling of `element/tet4.rs` | **small / low** |
| Tet4→Tet10 edge-enrichment helper (for oracle meshes) | new; reuse the sorted-key dedup of `mesh/mod.rs::boundary_faces_from_topology` | **small / low** |
| `Mesh` 10-node connectivity + the orphan-pin footgun | `mesh/mod.rs:76` (`tet_vertices → [VertexId;4]`), `construct.rs:186` | **large / high** |
| `ElementGeometry` → per-Gauss-point; `deformation_gradient`/`extract_element_dof_values` → 30-DOF; 6 assembly kernels `0..4`→`0..N`, single-GP → `for q in 0..G` | `solver/backward_euler/mod.rs:68`, `helpers.rs:17`, `assembly.rs:150-558` | **large / medium** |
| Per-Gauss-point material sampling (bypass the per-tet centroid cache) | `mesh/mod.rs:167` (`materials_from_field`), spec `01-tet10.md:46` | **medium / medium** |
| Gradient / adjoint RHS per-GP (the silent-wrong risk) | `sensitivities.rs:825` (`assemble_material_residual_grad`), `assembly.rs:472` (`internal_force_tangent_matvec`) | **medium / high** |
| F-bar / mixed-u-p for incompressible Tet10 (only if §4 shows pure Tet10 insufficient) | `fbar.rs` premise breaks; `01-mixed-up.md:24` | **large / high** (out of the initial scope) |

**The single biggest obstacle** is the `Mesh` connectivity widening (§3.2),
with the multi-Gauss-point lift (§3.3) second. The element math itself is the
*easy* part.

---

## 3. Architecture decisions

### 3.1 Migrate-vs-parallel: parallel/additive, generic-over-`(N,G)`

Add Tet10 **alongside** Tet4, keeping Tet4 byte-identical — do NOT migrate
(migrating would re-baseline every existing gate and delete the cheap
constant-strain element). The const-generic `<N,G>` scaffold + the
forward-carried `element` field were built for exactly this. Realise it by
making the assembly bodies genuinely generic over `(N,G)` with Tet4 as the
`(4,1)` special case — **not** a duplicated Tet10 assembly path. Tet4
byte-identity is a hard gate: prove it (sorted code-line multiset / a
regression run of the full non-Tet10 suite), because a float-reordering in
the generic G=1 path silently perturbs every existing gate.

### 3.2 Mesh connectivity: keep corners, add midside nodes additively

`tet_vertices → [VertexId;4]` is the corner topology, and the geometry,
quality metrics, and boundary-face extraction *correctly* use the 4 corners
(a straight-edged Tet10's geometry *is* its 4 corners). **Keep
`tet_vertices` unchanged**; surface the 6 midside nodes through an *additive*
channel (a second method, or a `Tet10` connectivity type; the trait doc at
`mesh/mod.rs:145` already anticipates lifting topology into a `MeshTopology`
supertrait).

**★ Load-bearing footgun (do not trip):** `construct.rs:186-214` auto-pins
(Dirichlet) every vertex NOT returned by `tet_vertices` as an "orphan." If
midside nodes are not surfaced to the free-DOF construction, all 6-per-edge
Tet10 DOFs get **silently pinned** and the element degrades to Tet4 with no
error. The connectivity change and the free-DOF/orphan logic must land
together.

### 3.3 Single → multi-Gauss-point

`ElementGeometry` currently holds one `grad_x_n: SMatrix<f64,4,3>` + one
`volume` (a single centroid Gauss point, valid only because Tet4's
parametric gradients are constant). Tet10's `F` varies linearly across the
element, so both the shape gradients (`grad_x_n^q = ∇_ξN(ξ_q)·J_q⁻¹`) and the
weight (`w_q·detJ_q`) are **per Gauss point**, and `J_q = Σ_i X_i⊗∇_ξN_i(ξ_q)`
is the isoparametric Jacobian (no longer the corner Jacobian). Generalise
`ElementGeometry` to per-GP storage (`[…;G]` or `Vec`), and every kernel that
today does `deformation_gradient(&x_elem, &geom.grad_x_n)` + `materials[tet_id]`
once becomes a `for q in 0..G` accumulation. Tet4 monomorphises to `G=1` and
must reproduce today's arithmetic bit-for-bit.

### 3.4 Per-Gauss-point material sampling

The per-tet `Vec<M>` material cache (sampled at the centroid,
`materials_from_field`) is Tet4-specific. Tet10 samples per Gauss point
(`[M;G]` per tet, or a `materials[tet_id*G + q]` flattening); the
`MaterialField` is already position-valued, so this is a cache-shape change,
not a field change. The `Material` trait surface is unchanged.

---

## 4. Oracle-driven validation — the fail-fast strategy

Unlike the F-bar arc (where a material swap was a cheap spike), Tet10
produces **no accuracy signal until most of the forward stack exists**. So
the build order (§5) is arranged to reach the oracle validation as early as
possible, and two ready, *already-validated* oracles gate it:

- **Lamé single-shell hollow sphere** (`tests/concentric_lame_shells.rs`
  machinery: `SoftScene::layered_silicone_sphere` + a single static
  `replay_step`, no IPC ramp — cheap). Plain Tet4 converges to the analytic
  at 0.993 (validated), so the oracle is trustworthy. **Target: Tet10 at
  ν=0.49 → ~1.0.** This is the decisive fail-fast: if pure Tet10 lands near
  1.0, demand #2 is met; if it plateaus off (as F-bar did at ~1.21), Tet10
  needs Taylor-Hood on top, and we learn it *immediately* rather than after
  building the whole rung on a false assumption.

- **#676 bonded-layer indentation** (`tests/bonded_layer_indentation.rs`).
  **Target: the ~9% Tet4 element-order floor tightens** (its documented
  `[1.00,1.20]` band should tighten toward 1.0). This closes demand #1.

**Both oracle meshes are enrichable for free.** `cantilever_bilayer_beam`
(#676) and the BCC `SdfMeshedTetMesh` (Lamé) both emit conforming corner
tets; the Tet4→Tet10 edge-enrichment helper (§2) turns either into a
conforming Tet10 mesh — no new mesher, and a stronger check than the spec
asks for (it treats Tet10 as a fresh regression baseline).

---

## 5. Gating ladder — build order (each step: commit → gating-review → next)

1. **`Element<10,4>` primitive.** Quadratic shape functions + 4-point
   symmetric Stroud rule (`a=(5+3√5)/20`, `b=(5−√5)/20`, weights `V/4`).
   Unit-test standalone: partition of unity, Kronecker-delta at nodes,
   `shape_gradients` vs finite-difference of `shape_functions`, quadrature
   integrates degree-2 monomials exactly. Public API, zero solver
   integration → zero risk. *(This is the one piece safe to land in
   isolation, even ahead of the rest.)*
2. **Edge-enrichment helper** `enrich_tet4_to_tet10`. Unit-test: shared
   edges dedup to one midside node, midpoints at edge centres, conforming
   output. Reused by both oracles.
3. **Mesh connectivity** — surface midside nodes additively, footgun-safe
   (§3.2). Land with the free-DOF/orphan logic.
4. **Multi-Gauss-point forward assembly** (§3.3–3.4), Tet4 byte-identity
   proven. Forward solve only.
5. **★ ORACLE VALIDATION (fail-fast, §4).** Enriched Lamé sphere → ν=0.49
   accuracy; enriched #676 → contact-floor tightening. **Decision gate:** is
   pure Tet10 accurate at ν=0.49, or is Taylor-Hood needed? Do not proceed
   past here without the answer.
6. **Gradient path** — per-GP adjoint RHS (§3.5 risk), lift the `N==4` and
   fbar guards for Tet10 as each is made Tet10-correct, **FD-gradcheck**
   every channel (`material_sensitivity`, `dirichlet_reaction_sensitivity`,
   `state_sensitivity`). This is the silent-wrong-risk step — the linear
   solve succeeds with a wrong-shape RHS and returns plausible-but-wrong
   gradients, so nothing but FD gates catches it.

Per the standing covenant: per-crate cargo only, soft tests `--release`,
`xtask grade` before push, gating cold-reads before merge.

---

## 6. Risks the build must actively manage

- **Gradient silent-wrong (highest).** The material/reaction adjoint RHS
  (`assemble_material_residual_grad`, `internal_force_tangent_matvec`) must
  move to per-GP in lockstep with the forward `f_int`; a shape mismatch does
  not error, it biases the gradient. FD gates are the only net.
- **Tet4 byte-identity.** The generic G=1 path must not reorder Tet4's float
  ops; verify against the full existing suite.
- **The orphan-pin footgun (§3.2)** silently degrading Tet10 to Tet4.
- **Accuracy is unproven until step 5.** Treat "Tet10 fixes ν=0.49" as a
  hypothesis to falsify at the Lamé oracle, not a premise.

---

## 7. Open questions carried into the build

- **Pure Tet10 vs Tet10 + Taylor-Hood P2-P1 at ν=0.49** — resolved
  empirically at step 5.
- **F-bar on Tet10** — its single-Gauss-point premise breaks; if a cure is
  needed it is mixed/Taylor-Hood, a redesign, not a port of `fbar.rs`.
- **Mixed-element Tet4↔Tet10 interfaces** (band-`sdf` tagging, midside-node
  DOF elimination per `03-mixed.md`) — deferred; the initial rung is
  uniform-Tet10 test meshes only.

---

*Companion state (this session's F-bar findings, the Lamé oracle, the
recon anchors) is mirrored in the session memory
`project-tet10-fbar-element-upgrade`.*
