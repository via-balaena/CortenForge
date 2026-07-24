# sim-soft — Tet10 quadratic-element implementation plan

> **Authored 2026-07-22; diamonded 2026-07-23** after a five-front
> code-grounded pressure-test (every load-bearing claim verified against the
> actual source at `main`). The implementation roadmap for adding the
> **Tet10** (10-node quadratic tetrahedron) element to `sim-soft`. Status:
> **IN PROGRESS — rungs 1–5 landed (#680/#681/#682/#683/#685/#686); the forward
> solver assembles the REAL multi-Gauss-point Tet10 stiffness over all 10 nodes
> (§3.3), and the rung-5 element-correctness gates (finite-rotation RBM +
> asymmetric-patch ordering detector) validate it (§5 step 5). NEXT = rung 6
> (the ν=0.4 baseline → ν=0.49 ACCEPT/REJECT oracle decision — heavy,
> user-in-loop, its own session).** ⚠ **Rung 4 landed
> via a TWO-CACHE design, not the in-place `ElementGeometry` generalization the
> §3.3/§3.4 forward text below describes** — see the "★ RUNG 4 LANDED (design
> delta)" note in §3.3: generalizing `ElementGeometry` in place would have forced
> an `fbar.rs` signature change, breaking the hard "no fbar changes" isolation, so
> the per-Gauss-point data lives in a NEW `GaussGeometry<N,G>` cache while the
> single-point `ElementGeometry` is RETAINED (not replaced) for the Tet4-flavored
> consumers. This doc is the forward plan the build follows; the landed rungs'
> code-verified deltas live in the `project-tet10-fbar-element-upgrade` session
> memory.
>
> The element *design* already lives in the spec book
> ([`30-discretization/00-element-choice/01-tet10.md`](studies/soft_body_architecture/src/30-discretization/00-element-choice/01-tet10.md), path relative to `docs/`):
> 10 nodes (4 corners + 6 edge-midpoints), quadratic shape functions,
> 4-point Stroud Gauss rule, per-Gauss-point material sampling. This doc is
> the *how-we-port-it*: what the current code actually requires, in what
> order, gated by which oracles.
>
> **TL;DR.** Tet10 is the accurate soft-element foundation — it improves the
> curved contact patch (the ~9% Tet4 floor #676 documented) and is the
> established higher-order path for near-incompressibility (ν≈0.49 Ecoflex),
> where the shipped Tet4 F-bar is a *qualitative* cure only (~5–21%
> over-soft, see §1). **The center of gravity is the `Mesh` connectivity +
> the whole surface/boundary path, NOT the element math.** The const-generic
> `Element<N,G>` boundary is real, but `tet_vertices → [VertexId;4]` is the
> load-bearing spine that mass, Hessian-incidence, boundary faces, BC/load
> predicates, tributary areas, contact, and every adjoint channel all ride
> on. The build order is arranged to (a) stage the connectivity plumbing (3a,
> midsides pinned = bit-identical Tet4) then land the **atomic unpin-trio**
> `{free-DOF + HRZ mass + Hessian-incidence}` (3b), (b) prove Tet10 forward
> correctness at **element scale** — including a rank/eigenspectrum gate —
> before the integrated oracle, and (c) reach the **Lamé accuracy decision**
> through a real committed baseline, a ν=0.4 match-or-beat anchor, and a
> pre-registered ν=0.49 three-way (convergence-aware) gate, not an
> uncommitted number.

---

## 1. Why Tet10 — the two demands, and the honest accuracy expectation

Two independent gaps motivate Tet10:

- **Demand #1 — the ~9% Tet4 contact-patch floor.** The #676
  `bonded_layer_indentation` gate proved (3-level cell convergence) that
  Tet4 over-stiffens the curved contact patch by a *mesh-converged* ~9%
  (measured RATIOs 1.051/1.105/1.130 in a `[1.00, 1.20]` band; the ~9% is
  the mid-χ residual, not a literal constant). The test's own docstring
  attributes the floor to "Tet4 element over-stiffness on the curved contact
  patch" — an **element-order** remedy, and it names Tet10 as the fix.

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
on. Note the Lamé oracle's *own* docstring ties the ν→0.5 regime to F-bar as
well, so "pure Tet10 cures ν=0.49" is if anything contra-indicated by the
oracle's framing — all the more reason it is a hypothesis to falsify, not a
premise. Two consequences: (a) the F-bar cure **does not carry over** — its
whole premise is the single-Gauss-point per-element volumetric constraint
(`fbar.rs`), which multi-Gauss-point Tet10 breaks; (b) if pure Tet10 is not
accurate-enough at ν=0.49, the cure-on-top is Taylor-Hood P2-P1, not F-bar.

**The honest accuracy expectation for demand #1:** Tet10 will move the #676
*net-force* ratio toward the analytic (the gate's measurable is a robust
force sum, and contact auto-includes the new midside DOFs — §3.5). But
straight-edged Tet10 does **not** give a curved/isoparametric contact
surface, and node-collocated barrier contact on a quadratic face is
*inconsistent* (under uniform pressure a quadratic triangle loads midsides,
not corners) — so bare Tet10 would cure the *net-force* metric while leaving
the **local** contact pressure/patch mechanics wrong. **Per the "exact
geometry IS the exact physics — obsessively-perfect contacts are the point"
standard, consistent quadratic-face contact is a first-class rung of this
plan, not a deferral.** ⚠ **CORRECTION (round-3 meta-audit): the
genuinely-consistent scheme is the SURFACE-INTEGRATED barrier `E = ∫ b(sd) dA`
at face Gauss points — NOT the "nodal lumping" a prior draft proposed.**
Lumping redistributes per-vertex *forces*; the consistent P2 load `∫p·N_i dA`
requires integrating a *traction*, and the P2 corner area is zero (the
redistribution is singular exactly at the corners it must zero out). So
lumping gives net-force smoothing, *not* correct local mechanics. **Decision — isolated rung (chosen over building contact in-sequence):** the Tet10 element foundation lands first (steps 1–7, demand #1
validated on the robust #676 *net-force* metric), then the real
surface-integrated barrier + curved surface land as **rung 8**, its own
isolated consistent-contact rung on the proven foundation — the
obsessively-perfect-contacts work, sequenced right after the base rather than
tangled into it. (The isoparametric curved surface — projecting boundary
midside nodes onto the true surface — shares rung 8's face-primitive machinery
and lands with it.)

**Why not the cheaper Tet4-only cures (recorded so we don't relitigate):**
the shipped **nodal-averaged F-bar over-softens** (measured vs the Lamé
oracle: ~5% at ν=0.4 → ~21% at ν=0.49, mesh-converged; the nodal-star patch
average over-relaxes the volumetric constraint — a *scheme* property, not a
material or solver bug; a dev/vol-split material was spiked and gives
identical results at this strain, so it is not the material). **mixed-u-p
P1-P0 on Tet4** is the spec's stated primary cure but is theoretically
finicky for tets (inf-sup stability). Both are lower-confidence than the
established higher-order path. Tet10 is chosen for *quality/ceiling*: it is
the textbook-robust near-incompressible element **and** the element-order
remedy for demand #1.

---

## 2. What the port touches — recon sizing (authoring baseline; verified at `main` pre-rung-1)

"Tet10 is additive" is aspirational and, as scoped in the first draft, was
too narrow. The const-generic `CpuNewtonSolver<E, Msh, C, M, const N, const
G>` (`solver/backward_euler/mod.rs:86`) and the `Element<N,G>` trait
(`element/mod.rs:17`) are genuinely element-agnostic — Tet4 impls
`Element<4,1>` (`element/tet4.rs:24`). At authoring the const generics were
**decorative below `new()`, and `G` decorative even inside it**:
`construct.rs` hard-asserted `N == 4`, `gauss_points()` was *never called*
anywhere (the centroid `(0.25,0.25,0.25)` was hardcoded), and every assembly
body used concrete Tet4 types. **(Rungs 1–3b have since lifted the `N == 4`
assert and wired `gauss_points()` into the HRZ mass — §3.2; the forward
stiffness assembly stays Tet4-shaped until rung 4.)**

| Area | Anchor | Size / risk |
|---|---|---|
| `Element<10,4>` impl (shape fns + Stroud table + real `gauss_points`) | new sibling of `element/tet4.rs` | **small / low** |
| Tet4→Tet10 edge-enrichment helper (a mesh *producer*; needs §3.2 connectivity to be consumable) | new; reuse the sorted-key dedup of `mesh/mod.rs:251` | **small / low** |
| **`Mesh` connectivity — the load-bearing spine.** `tet_vertices → [VertexId;4]` is a *public trait signature*; every mesh impl (`single_tet.rs:99`, `hand_built.rs:328`, `sdf_meshed_tet_mesh.rs:391`) and every consumer reads exactly 4. Additive 10-node channel (`MeshTopology`) + the orphan-pin footgun. | `mesh/mod.rs:76`, `construct.rs:186-214` | **large / high** |
| **Boundary/surface path — must move in lockstep with connectivity.** `boundary_faces_from_topology` emits 3-node faces from 4 corners; Tet10 surface midside nodes are otherwise classified *interior* → auto-pinned even if surfaced. Plus `boundary_vertex_areas`, BC/load position-predicates, cavity tributary-area (`N_loaded`). | `mesh/mod.rs:251`, `readout/scene.rs:226-296` | **medium / high** |
| Per-Gauss-point geometry: `ElementGeometry{grad_x_n:SMatrix<4,3>, volume}` → array of `(grad_x_n, weight)` over G (cardinality **1→4**: the shape gradients `∇_ξN(ξ_q)` and material `F(ξ_q)` vary per-GP so `grad_x_n` genuinely differs across the 4 GPs). **⚠ For a straight-edged Tet10 the isoparametric map collapses to affine, so `J`, `J⁻¹`, `detJ` are CONSTANT** — do not describe the Jacobian as per-GP (it only becomes per-GP under the deferred rung-8 curved surface). Tet4's `(4,1)` path keeps the **edge-vector Jacobian** and the `det.abs()/6.0` weight — a distinct branch, not the Tet10 Σ-form monomorphized (§3.1). | `mod.rs:68`, `construct.rs:221-244` | **large / medium** |
| Multi-GP forward assembly: `deformation_gradient`/`extract_element_dof_values` → 30-DOF (`[f64;12]→[f64;30]`, `SMatrix<4,3>→<10,3>`); the six kernels `for a in 0..4`→`0..N`, single-GP → `for q in 0..G`. **No `12×12`/`30×30` stiffness type exists** — stiffness is 3×3 blocks from a 9×9 *material* tangent; do not size a matrix-dim row that isn't there. | `mod.rs`, `helpers.rs:17,36`, `assembly.rs:150-558` | **large / medium** |
| **Consistent-contact rung 8 (the isolated-rung decision — its own rung AFTER the element foundation).** ⚠ Round-3 meta-audit falsified the "nodal lumping" scheme (net-force smoothing, NOT the consistent P2 load — barrier is a force not a traction; P2 corner area = 0 → singular redistribution). The real scheme = **surface-integrated barrier `E=∫b(sd)dA` at face Gauss points + isoparametric curved surface + `boundary_vertex_areas` 6-node**, together (shared face primitive). Needs a new `#[non_exhaustive]` `ContactPair::Face` variant = a **downstream break** (coupling uses irrefutable `let Vertex{..}` — `contact_readout.rs:154`, `single_step.rs:35`). **Friction reconciled with the face-distributed force.** | `contact/ipc.rs:213-310`, `mesh/mod.rs:313` | **large / high (rung 8)** |
| Per-Gauss-point material sampling (bypass the per-tet centroid cache) | `mesh/mod.rs:154` (`materials_from_field`), spec `01-tet10.md:46` | **medium / medium** |
| **Mass lumping — use HRZ (Hinton–Rock–Zienkiewicz) diagonal-scaling, NOT a divisor swap.** `density * volume / 4.0` (`construct.rs:263`) is Tet4-specific; naive row-sum lumping of Tet10 gives **negative corner masses** (→ indefinite tangent → Cholesky failure at small dt). HRZ takes the positive consistent-mass diagonal `∫ρN_i²dV` (evaluated at the 4 Stroud GPs — **rides on §3.3 multi-GP weights**) scaled to preserve total mass. Only a diagonal `Vec<f64>` is ever needed (no consistent matrix). **⚠ Every current oracle is `STATIC_DT` → mass-blind; needs a dynamics gate (§5 steps 3b/7).** | `construct.rs:259-270` | **medium / medium** |
| Gradient / adjoint RHS per-GP (the silent-wrong risk, but *narrower* than diffuse — §3.5) | `sensitivities.rs:825`, `assembly.rs:472` | **medium / high** |
| **`fbar.rs` — a whole second 4-node assembler (~15 sites).** EXPLICITLY EXCLUDED from the initial rungs (steps 1–7): off by default, and every differentiable path already `assert!(!config.fbar)` at `factor.rs:466`. Do not size it small; defer it wholesale. **Belt-and-suspenders: add a `forward`-path assert rejecting `config.fbar && N != 4`** (the differentiable path is guarded, but a user forcing `fbar=true` on a forward Tet10 solve is currently ungated). | `fbar.rs` | **out of initial scope** |
| Taylor-Hood / mixed-u-p for incompressible Tet10 (only if §4 shows pure Tet10 insufficient) | `01-mixed-up.md:24` | **large / high** (out of initial scope) |

**The single biggest obstacle** is the `Mesh` connectivity + boundary path
(§3.2), which is *cross-cutting* — it touches assembly, adjoint, contact,
mass, and diagnostics at once. The multi-Gauss-point lift (§3.3) is second.
The element math itself is the *easy* part.

---

## 3. Architecture decisions

### 3.1 Migrate-vs-parallel: parallel/additive, generic-over-`(N,G)`

Add Tet10 **alongside** Tet4, keeping Tet4 byte-identical — do NOT migrate
(migrating would re-baseline every existing gate and delete the cheap
constant-strain element). Realise it by making the assembly bodies genuinely
generic over `(N,G)` with Tet4 as the `(4,1)` special case — **not** a
duplicated Tet10 assembly path. Tet4 byte-identity is a hard gate: prove it
(a regression run of the full non-Tet10 suite), because a float-reordering in
the generic G=1 path silently perturbs every existing gate.

**Byte-identity is achievable but hinges on two specific arithmetic-order
traps — the refactor must respect both, not just "run the loop once":**
- **★ Weight: keep `det.abs()/6.0`, do NOT reconstruct it as `(1.0/6.0)·detJ`.**
  `1.0/6.0` is not representable, so `(1/6)*|detJ| ≠ |detJ|/6.0` at the ULP —
  a silent bit-break. The Tet4 `(4,1)` weight must remain the literal
  `construct.rs:232` expression.
- **★ Jacobian: keep the edge-vector form for Tet4 — a DISTINCT branch, not
  the Tet10 Σ-form monomorphized.** Tet4 builds `J` from 3 corner edge
  vectors (`construct.rs:228`); routing it through the isoparametric
  `J = Σ Xᵢ⊗∇_ξNᵢ` accumulation reorders the arithmetic and is NOT guaranteed
  bit-equal. (The shape-gradient side `grad_xi_n · j_0_inv` at `:236` is
  already generic matmul — only the Jacobian + weight are Tet4-special.)
- **The G-loop itself is bit-safe.** The int-force `f_int += volume·sum`
  wraps to a single-iteration `+= w_0·sum_0` op-for-op; the tangent block's
  assign→accumulate change differs only at `-0.0` (numerically inert). So the
  loop is fine; the two traps above are the real hazards.

**Golden bit-tests already exist and WILL catch NH-path drift** —
`contact_passthrough.rs` (also the solve-path *determinism* gate: run-to-run
`to_bits()`) and `invariant_iv_1_uniform_passthrough.rs` pin `x_final`. **But
they cover only ~4 small Neo-Hookean fixtures; Yeoh, roller-BC, F-bar, and the
Lamé sphere are tolerance-only** and would miss ULP drift. **Extend the golden
bit-set to ≥1 Yeoh scene and ≥1 roller scene before the step-4 refactor** —
this doubles as the determinism-net extension (`sdf_pipeline_determinism.rs`
is mesh-only, no solve-path coverage).

**★ Load-bearing invariant the whole byte-identity strategy rests on: the
assembler MUST stay serial.** rayon is deliberately disabled
(`Cargo.toml`, "round-6 determinism") because work-stealing reorders FP
reductions bit-non-deterministically; the `BTreeMap` sorts *keys* but does not
rescue a parallel `+=` from FP-reorder. Do **not** introduce `rayon`/`par_iter`
into the larger Tet10 element loop as an "optimization" — it would silently
break every `to_bits()` golden. (This rung is CPU-only; there is no GPU
assembly path to touch.)

- **A live lossy truncation exists at `construct.rs:237-242`:** `new()`
  already computes `grad_x_n_generic: SMatrix<f64,N,3>` generically, then
  copies only the first 4 rows into a concrete `SMatrix<4,3>`. That narrowing
  is where nodes 4–9 are currently discarded; it is the natural seam to
  generalize, not dead code to delete.

### 3.2 Mesh connectivity + boundary: additive, and ATOMIC with mass/Hessian/geometry

`tet_vertices → [VertexId;4]` is the corner topology, and the geometry,
quality metrics, and boundary-face extraction *correctly* use the 4 corners
(a straight-edged Tet10's geometry *is* its 4 corners). **Keep
`tet_vertices` unchanged**; surface the 6 midside nodes through an *additive*
channel (a second method, or a `MeshTopology` supertrait — note the trait doc
at `mesh/mod.rs:145` anticipates such an extraction, but for *cross-`M`
object-safety*, not higher-order nodes; it is evidence the extraction is
acceptable, not that it was designed for Tet10).

**The genuine atomic core is the 3-item DOF unpin-trio — plus the
corner-geometry that keeps the freed element mechanically Tet4, which the
build proved is NOT automatic (see the CODE-VERIFIED note below) — and there
is a safe staged intermediate: split this into 3a (plumbing) and 3b (the
unpin-trio + that geometry).**

- **3a — safe plumbing, midsides auto-pinned (a valid smaller commit).**
  Land the additive `MeshTopology` 10-node channel + `enrich_tet4_to_tet10`
  producer + midside nodes in `positions()`/`n_vertices`, but leave
  `tet_vertices` at 4 corners so midsides are *unreferenced* and the
  orphan-pin (`construct.rs:186-214`) auto-Dirichlet-clamps them. The result
  is **provably bit-identical Tet4** (midsides are inert pinned points at
  rest) — the golden tests stay green. This is *deliberate* Tet4, not the
  "silent degradation" footgun; it lands all the connectivity/boundary
  plumbing with zero physics change.
- **3b — the atomic unpin-trio (genuinely inseparable).** The moment a
  midside DOF is *unpinned*, three things must land together or the factor
  crashes: **{free-DOF unpin + Tet10 mass (HRZ) + Hessian-incidence widening}**.
  This is forced by the symbolic-pattern path: the mass-diagonal scatter emits
  a `(k,k)` entry for every free DOF (`assembly.rs:419-423`), and a `(k,k)`
  not present in the cached symbolic pattern (built from corner incidence in
  `construct.rs` §4, factored via the `build_symbolic_factors` helper) is a
  pattern mismatch → panic at `factor.rs:387`.
  So incidence-widening is mandatory on unpin, and a *non-zero* (positive)
  mass diagonal is mandatory to factor — the negative-corner-mass landmine
  (§2 mass row) bites exactly here. Land 3b together with, or immediately
  before, the step-4 per-GP geometry.
  - **★ CODE-VERIFIED (rung 3b, merged #683) — two refinements the build
    proved.** (a) **The 3b incidence widening is NOT `0..N`.** It is the
    corner-corner element incidence **plus one `(k,k)` per free DOF** (mirroring
    the mass-diagonal scatter exactly). Load-bearing invariant: **the symbolic
    pattern must equal the assembled numeric pattern *exactly* — a superset
    (e.g. the corner↔midside blocks a naive `0..N` adds but 3b's stiffness-free
    midsides never fill) silently CORRUPTS faer's `try_new_with_symbolic`
    numeric read (a wrong factor, not a crash).** Rung 4 reaches `0..N`
    legitimately because its multi-GP stiffness *fills* those blocks — so **rung
    4 widens the symbolic AND numeric patterns in lockstep** (never symbolic
    alone). (b) **"Still mechanically Tet4" is NOT automatic — it lands with
    the trio.** Tet10's shape gradients VANISH on the four corners at the
    centroid (∂N=0 there), so
    the direct-form `F = Σ xₐ⊗∇Nₐ` would give `F=0` → NaN. 3b's corner geometry
    therefore uses the linear barycentric gradients (a Tet4 constant-strain
    block). **Rung 4 landed (design delta): this single-point corner block is
    RETAINED in `ElementGeometry` — it still feeds the validity gate, the
    rung-7-guarded material adjoint, F-bar, and the lumped mass — while the real
    per-Gauss-point stiffness geometry lives in a separate `GaussGeometry<N,G>`
    cache the forward kernels integrate over (§3.3). The forward *stiffness* is
    replaced by per-GP; the corner block is not deleted.**
- **Per-GP geometry and boundary extraction are NOT part of the atomic core**
  — they are *correctness* concerns (real Tet10 stiffness / valid pressure
  diagnostics), stageable separately. A midside that is free + has a positive
  mass diagonal + has a Hessian-pattern slot converges safely as a
  stiffness-free floating mass (still mechanically Tet4, but non-singular),
  which is exactly the step-3b gate.

Lift the `N==4` assert (`construct.rs:78`) as the last thing in 3b, once the
unpinned path is non-singular.

### 3.3 Single → multi-Gauss-point (cardinality 1 → G)

> **★ RUNG 4 LANDED (design delta — read first).** The build did NOT generalize
> `ElementGeometry` in place as the text below plans. `ElementGeometry` is
> consumed *by type* in `fbar.rs` (~30 sites) and `sensitivities.rs`, so making
> it const-generic `<N,G>` would ripple a signature change into `fbar.rs` —
> breaking the hard "no fbar.rs changes" isolation (§5, rung-8/fbar deferral).
> Instead, `ElementGeometry` stays non-generic (the single-point corner geometry
> for the Tet4-flavored consumers — F-bar / material adjoint / validity / mass,
> all byte-identical and untouched), and a NEW `GaussGeometry<const N, const G>`
> cache holds the per-Gauss-point `(grad_x_n, weight)` the three forward
> stiffness kernels integrate over. Two caches rather than one; for Tet4
> `gauss[0]` duplicates the `ElementGeometry` fields bit-for-bit. The rest of
> this section describes the per-GP *math*, which is unchanged; only the *home*
> of the per-GP data differs (a separate cache, not `ElementGeometry`'s fields).

`ElementGeometry` currently holds one `grad_x_n: SMatrix<f64,4,3>` + one
`volume` — a single centroid Gauss point, valid only because Tet4's
parametric gradients are constant. **For a straight-edged Tet10 the
isoparametric map collapses to affine, so `J`, `J⁻¹`, `detJ` are CONSTANT
across the element** (do not model the Jacobian as per-GP — that only happens
under the deferred rung-8 curved surface). What *does* vary per-GP is the shape
gradient `∇_xN^q = ∇_ξN(ξ_q)·J⁻¹` (because `∇_ξN(ξ_q)` is linear in ξ) and the
material `F(ξ_q)`. So generalise `ElementGeometry` to *G entries* (`[…;G]`) of
`(grad_x_n, weight)` — the 4 `grad_x_n` genuinely differ (cardinality 1→4) but
share one constant `detJ` (and, for the symmetric Stroud rule, equal weights).
Every kernel that today does
`deformation_gradient(&x_elem, &geom.grad_x_n)` + `materials[tet_id]` once
becomes a `for q in 0..G` accumulation calling `first_piola`/`tangent` per
GP. Tet4 monomorphises to `G=1` and must reproduce today's arithmetic
bit-for-bit (§3.1 traps).

### 3.4 Per-Gauss-point material sampling

The per-tet `Vec<M>` material cache (sampled at the centroid,
`materials_from_field`, `mesh/mod.rs:154`) is Tet4-specific. Tet10 samples
per Gauss point (`[M;G]` per tet, or a `materials[tet_id*G + q]` flattening);
the `MaterialField` is already position-valued, so this is a cache-shape
change, not a field change. The `Material` trait surface is unchanged.
(Note: the SDF-mesh interface centroid-tag misclassification band is a
*separate* pre-existing issue that Tet10 does not fix — do not attribute it
to this work.)

### 3.5 Contact and the gradient/adjoint path — narrower than they look

**Contact "just works" through Tet10 — but only once §3.2 lands.** Contact is
a **blind per-vertex loop over the position array** vs an analytic rigid SDF
(`contact/ipc.rs:296`, `contact/penalty.rs:942`); the only `ContactPair`
variant is `Vertex`, and `boundary_faces` is diagnostic/viz only (the trait
doc at `mesh/mod.rs:135` says "the Newton hot path never touches it"). So the
moment midside nodes are in the global DOF/position array, they are
automatically tested against the SDF and carry barrier force — *zero* contact
code changes. That moves the #676 net-force ratio (demand #1, step 6d), but is
**not** enough for correct *local* mechanics — which is why consistent contact
is its own rung 8 (the isolated-rung decision):
(a) node-collocated barrier on a quadratic face is *inconsistent* — under
uniform pressure a quadratic triangle loads midsides, not corners, so equal
per-node barriers give a lumpy/wrong local pressure (the net-force sum stays
robust). ⚠ **CORRECTION (round-3 meta-audit): a prior draft claimed "nodal
lumping" fixes this — it does NOT.** The consistent P2 load is `∫p·N_i dA`
(integrate a *traction*, corners ≈ 0 because `∫N_corner dA = 0`); the barrier
yields per-vertex *forces*, and the force→pressure inversion is singular at the
corners (corner area = 0). Redistributing point forces gives net-force
smoothing, **not** the consistent load — and the old "quadrature only matters
for varying `b(sd)`" excuse is false, since even *uniform* pressure needs the
surface integral. **The genuinely-consistent fix is the SURFACE-INTEGRATED
barrier `E = ∫ b(sd(x)) dA = Σ_q w_q b(sd(x(ξ_q)))` at face Gauss points**,
gradient distributed via the face shape functions. It is a real face contact
primitive (samples the SDF at non-nodal points), so it needs a new
`ContactPair` variant — **a downstream break** (`ContactPair` is public,
non-`#[non_exhaustive]`; coupling uses irrefutable `let Vertex{..}` matches
— `contact_readout.rs:154`, `single_step.rs:35`), requiring `#[non_exhaustive]`
+ downstream fixes.
It also **shares machinery with the isoparametric curved surface**, so both
build together as **rung 8** (the isolated-rung decision). (b) A midside
node gets `tributary_area = 0` → `pressure = NaN` → `peak_contact_pressure`
silently drops it — the pressure *readouts* regress from 3b until rung 8's
`boundary_vertex_areas` (`mesh/mod.rs:313`) 6-node upgrade (a known,
diagnostic-only degradation in the interim; #676 measures net force).

**★ Two things the real consistent-contact rung must handle (agent-verified):**
(i) **Friction is coupled to per-vertex contact.** `friction_blocks`
(`assembly.rs:216`) and the friction adjoint (`sensitivities.rs:303-313`,
`factor.rs:509-512`) read the contact gradient/Hessian assuming one vertex's
`λ=|force|`, `n̂=force/λ`; a face-distributed force breaks that, so reconcile
friction **or** guard face contact frictionless (the #676 gate is frictionless
— verified — so the guard is safe for validation). Step 8's FD list must
include friction, not just the barrier. (ii) **Hidden ordering dependency:**
the face contact Hessian's cross-node blocks are in the symbolic pattern only
once the element incidence reaches the full `0..N` node coupling — which lands
in **rung 4** (multi-GP stiffness fills the corner↔midside blocks), *not* 3b
(3b widens only to corner-corner + the free-DOF mass diagonal — see the
rung-3b code-verified note in §3.2). The `ContactHessian` type and scatter
already accept arbitrary `(row,col,block)` triplets (`contact/mod.rs:71`,
`assembly.rs:377-394`), and a boundary face's nodes are all co-element, so no
contact-specific sparsity change is needed — *provided rung 4 landed first*.
Order is already 3b→4→rung 8; just don't reorder.

**The gradient silent-wrong surface is real but *concentrated*, not diffuse.**
Most shape mismatches are caught at **compile time** — `tet_vertices →
[VertexId;4]`, `extract_element_dof_values → [f64;12]`, and `SMatrix<4,3>`
are type-level, so a widened Tet10 won't compile rather than silently bias.
The genuinely silent surface is specific: the literal `0..4` node loops, the
single-`geom.volume`/single-`grad_x_n` (missing GP loop), the per-tet single
material eval, and the `volume/4.0` mass split — wherever a *runtime*-widened
node list is iterated by a hardcoded `4`/single-GP. The FD gates compare each
adjoint against a re-solve FD of the *same* forward, so a Tet10 that stays
single-GP in **both** the forward and the adjoint produces wrong-but-matching
numbers that pass — that is the trap. Verified adjoint anchors and their
per-GP consumers:

- `assemble_material_residual_grad` (`sensitivities.rs:825`) — material
  channel RHS; shares the internal-force scatter skeleton.
- `internal_force_tangent_matvec` (`assembly.rs:472`) — a **third** tangent
  consumer (powers both the Dirichlet-reaction JVP *and* VJP), a separate
  per-GP site from the material RHS.
- The adjoint *operator* `A = factor_at_position` is per-GP-coupled for
  **every** channel (material, reaction, state) — only the load-θ RHS in
  `NewtonStepVjp` escapes per-GP, and it needs a Tet10 *consistent load
  vector* instead of "−1 per loaded node" (adjacent to the §3.2 load work).
- The **state/time-adjoint** channel carries a *second* Tet4 dependency: the
  lumped mass `volume/4.0` feeds `StateStepVjp` directly — a wrong Tet10 mass
  biases it even with the elastic path correct.

FD-gate every channel: `material_sensitivity`, `dirichlet_reaction_sensitivity`,
`state_sensitivity`.

---

## 4. Oracle-driven validation — the fail-fast strategy (reframed)

Unlike the F-bar arc (where a material swap was a cheap spike), Tet10
produces **no accuracy signal until most of the forward stack exists**. So
the build order (§5) reaches oracle validation as early as possible — but the
first draft's oracle framing had two errors the pressure-test caught:

- **★ There is no committed "Tet4 → 0.993 at ν=0.4" baseline.** That number
  is nowhere in the tests or src (grep-confirmed); it was an uncommitted
  spike. The Lamé oracle (`tests/concentric_lame_shells.rs`) asserts a
  *relative error* against a closed-form three-shell solution (`rel_err <
  0.20` fine-end, `< 0.30` sanity), on a Saint-Venant mean cavity
  displacement. **Step 6a below commits a real Tet4 baseline first** so the
  Tet10 target is measured against something that exists.
- **★ The #676 indentation oracle is the *slow* IPC route** — a
  multi-increment displacement ramp, release-only, `#[ignore]` in debug
  ("minutes release"), the very PERF-BLOCKED path the F-bar memo flagged. It
  is a real gate for demand #1 but **not** a cheap fail-fast one. Either
  accept a slow gate or build a cheaper contact micro-oracle (§5 step 6d).

The cheap, decisive gate is the **Lamé single-shell hollow sphere**
(`SoftScene::layered_silicone_sphere` + a single static `replay_step`,
`NullContact`, no IPC ramp — ~3 Newton iters). ν is a compile-time const
(`concentric_lame_shells.rs:294`) but both the FEM and analytic sides read
the same const family, so retargeting ν is a one-line edit that stays
consistent. Build order at the oracle:

1. **Commit the real Tet4 Lamé baseline** at ν=0.4 (retire 0.993). Record it
   as `e₄₀` (**Tet4** ν=0.4 rel-err, at the h/2 decision mesh) — the common
   mesh floor the ν=0.49 threshold subtracts against. (Tet10 ν=0.4 is a
   separate measurement, step 6b.)
2. **Tet10 at ν=0.4 → match-or-beat Tet4** on the same oracle. Bug-localizer:
   if Tet10 misses *here*, it's a forward/assembly bug, not an
   incompressibility limit.
3. **Tet10 at ν=0.49 → the decision gate — but PRE-REGISTERED, because the
   oracle as shipped cannot resolve it.** Three things must be fixed first
   (agent-verified): **(i)** run on the **uniform single-material sphere**
   (`uniform_middle_field`), *not* the three-shell — the three-shell's
   SDF interface-misclassification band (which Tet10 does *not* fix) is an
   element-order-independent error that swamps the locking signal; **(ii)**
   the shipped asserts are loose bands (`<0.20`/`<0.30`) that *straddle* the
   decision — assert a tight, pre-registered rel-err with the **decision at
   h/2** (~28k Tet10 DOF; the h/4 Tet10 mesh is ~226k DOF / GB-scale / OOM-risk,
   so it is a one-shot pre-push confirmation only — see §5 6c budget); **(iii)**
   raise
   `max_newton_iter` to ~150 (`fbar_locking.rs` needed that at ν=0.49 vs the
   Lamé gate's 50). **Convergence, not accuracy, is the real risk** — the
   oracle's own docstring says the ν→0.5 pressure-inflation mode makes
   "convergence collapse," so this is a **three-way** gate: **ACCEPT** (pure
   Tet10 adequate) iff Newton converges below budget **and** rel-err ≤
   `max(2·e₄₀, 0.10)`; **REJECT → Taylor-Hood** if rel-err > 0.20 **or** Newton
   stalls/exceeds budget; **gray zone 0.10–0.20 defaults to REJECT** unless
   h/2→h/4 shows it converging *downward* through 0.10. Pre-register these
   numbers in the test, don't eyeball a print stream after the fact.

**Both oracle meshes are conforming** (the BCC-stuffed SDF sphere is conformal
by the parity rule; the CFK hex grid shares `vid` indexing), so the
edge-enrichment helper can add shared midside nodes — but **not "for free":**
the enriched mesh is only *consumable* once §3.2 exists (the mesh→solver
connectivity is hardwired to 4 nodes today), and enrichment on the curved
sphere places midside nodes at *straight* edge midpoints (no curvature
capture — the isoparametric-surface piece is deferred, §7).

---

## 5. Gating ladder — build order (each step: commit → gating-review → next)

1. **`Element<10,4>` primitive.** Quadratic shape functions + 4-point
   symmetric Stroud rule (`a=(5+3√5)/20`, `b=(5−√5)/20`, weights `V/4` —
   constants verified degree-2-exact in closed form) + a real `gauss_points()`.
   **★ Pin the canonical 0-based edge→midside-node table as the single source
   of truth: `[(0,1),(1,2),(0,2),(0,3),(1,3),(2,3)] → local nodes [4,5,6,7,8,9]`**
   (independently re-derived from the spec + crate convention — exact match).
   **State the corner convention alongside it wherever cited: local node 0 =
   the `1−ξ−η−ζ` complement barycentric, nodes 1/2/3 = `ξ/η/ζ` (per `tet4.rs:26`)**
   — the table silently inverts if any code adopts node 0 = `ξ`.
   (This is the spec's *implicit* order `4ξ₄ξ, 4ξη, 4ξ₄η, 4ξ₄ζ, 4ξζ, 4ηζ`,
   which is written nowhere as a table and is 1-based in the spec — so an
   implementer who codes the Abaqus/textbook order gets a plausible-but-wrong
   element). The shape functions, `enrich_tet4_to_tet10`, the `MeshTopology`
   channel, multi-GP assembly, and 6-node boundary faces must **all cite this
   one table.** Unit-test standalone: partition of unity, Kronecker-delta at
   nodes, `shape_gradients` vs finite-difference, quadrature integrates
   degree-2 monomials exactly. Public API, zero solver integration → zero risk.
   *(Safe to land in isolation, ahead of the rest.)*
2. **Edge-enrichment helper** `enrich_tet4_to_tet10`. **★ Dedup by the sorted
   `(min,max)` global-vertex key (identity only) but assign the LOCAL slot 4–9
   from the step-1 canonical table — never from the sorted-key order.** The
   crate's dedup is uniformly `(min,max)`-sorted (`stuffing.rs:316`,
   `mesh/mod.rs:265`); if that same key also picks the local slot, each element
   gets a *different* edge→node permutation keyed on its accidental global
   numbering (not even a uniform swap) → silently wrong. Unit-test: shared
   edges dedup to one midside node, midpoints at edge centres, conforming
   output, **and local slots match the canonical table on an asymmetric tet.**
   (Consumable only after step 3b — it is a mesh producer.)
3. **★ Connectivity — split into 3a (safe plumbing) and 3b (atomic unpin-trio),
   per §3.2.**
   - **3a.** Additive `MeshTopology` 10-node channel + `enrich_tet4_to_tet10`
     wired into `positions()`/`n_vertices`, midside nodes left **auto-pinned**
     (`tet_vertices` stays 4 corners). Gate: **solve an enriched-and-midside-
     pinned golden fixture and assert `x_final.to_bits()` equals the un-enriched
     Tet4 solve** — not merely "constructs" (the whole 3a-as-safe-commit
     rationale rests on this bit-identity; it holds because Dirichlet DOFs are
     *condensed out*, so faer's fill-reducing permutation over the free block is
     unchanged *provided enrichment appends midside IDs after the corners*,
     preserving corner free-index numbering). Existing `to_bits()` goldens also
     stay green. Zero physics change.
   - **3b.** The inseparable trio **{free-DOF unpin + Tet10 HRZ mass +
     Hessian-incidence widening}** — forced together by the symbolic-pattern
     crash at `factor.rs:387`. Lift the `N==4` assert here. Gate: a uniform-Tet10
     mesh constructs with midside nodes **free**, a **positive** mass diagonal,
     and a **non-singular factor at a small dynamic dt** (not just `STATIC_DT` —
     a static gate cannot see a swamped negative mass; see the dynamics
     mass-gate below). Land 3b with, or just before, step 4.
4. **Multi-Gauss-point forward assembly (§3.3–3.4).** Real quadrature loop,
   per-GP geometry (cardinality 1→4, constant `detJ`), per-GP material. Gates:
   (a) **Tet4 byte-identity** — the `det.abs()/6.0` weight and edge-vector Jacobian
   preserved (§3.1 traps), proven against the golden bit-tests **extended to a
   Yeoh scene and a roller scene** (the current goldens are NH-only). (b) **A
   stiffness-magnitude reconciliation** — a single Tet10 element's production
   multi-GP tangent at rest (F=I) must equal an independent `BᵀDB` reference
   (NeoHookean `∂P/∂F(I)` is exactly linear isotropic elasticity), catching a
   stiffness-free element, a wrong assembly-level weight, or a mis-integrated
   corner↔midside block. This is a magnitude check only — node-*ordering* and
   element-primitive bugs remain the rung-5 / rung-1 gates' job.
5. **★ Tet10 element-correctness gates (before any integrated oracle). ✅ LANDED
   (#686).** Test-only, on the now-correct multi-GP element. Shipped: (a) the
   finite-rotation RBM gate and (b) the asymmetric quadratic-field ordering
   detector — the latter through the *production* node mapping
   (`element_node_ids` → assembler on a real `two_tet_shared_face`→Tet10 mesh),
   with an INDEPENDENT reference that maps slot→node by physical midpoint lookup
   (never reading `tet_midside_nodes`), so an enrich↔shape-gradient ordering
   drift diverges. (c) the rank/eigenspectrum gate was already landed in rung 1.
   Element-level companions (completeness + quadratic-F reproduction +
   constant-strain — the last documented ordering-blind) live in
   `element/tet10.rs`.
   **★ Design delta learned here: the assembler-based Tet10 gates — both rung-5
   production gates and the rung-4 `BᵀDB` tangent gate — now assert FINITENESS
   FIRST. A drift that inverts F (det ≤ 0 → `ln det F` = NaN) puts NaN in the
   assembled force, and `f64::max(m, NaN) == m` would otherwise mask it in the
   max-reduction, passing vacuously. The element-level companions need no such
   guard — they are pure kinematics that panic loudly on a singular inverse.**
   The original plan text for step 5 follows.
   (a) **Finite-rotation rigid-body** `x = c + Q·X` → zero internal force
   (stronger than translation-only; Neo-Hookean `P=0` for orthogonal `Q`).
   (b) **★ Asymmetric / quadratic-field patch test on an IRREGULAR element** —
   assert isoparametric completeness `Σᵢ Nᵢ(ξ)Xᵢ = X(ξ)` and exact reproduction
   of a *quadratic* displacement field (and an asymmetric shear `u=(aY,bZ,cX)`)
   at a non-symmetric interior ξ on a distorted tet. **This is the ONLY gate
   that catches a midside-ordering bug** — a symmetric uniform-stretch patch is
   reproduced by the corners alone and passes despite a permutation. (A uniform
   constant-strain patch is still worth keeping, but is machine-precision only
   *for straight edges* and breaks under the rung-8 curved surface.)
   (c) **Rank/eigenspectrum gate** — single-element 30×30 `K^e` at `F=I`,
   deformed, and distorted; assert **exactly 6 zero + 24 positive eigenvalues**
   (nalgebra `SymmetricEigen::eigenvalues` or an SVD null-count — both in-tree,
   no new dep). Catches spurious zero-energy modes and guards the zero-margin
   4-GP rule on slivers. **⚠ Necessary but NOT sufficient: a node-permutation is
   a similarity transform `PKᵉPᵀ`, so the spectrum is permutation-INVARIANT —
   this gate is blind to an ordering bug; (b) is the ordering detector.**
6. **★ ORACLE VALIDATION (fail-fast, §4).**
   - **6a.** Commit the real Tet4 Lamé baseline at ν=0.4 (retire 0.993); record `e₄₀`.
   - **6b.** Tet10 at ν=0.4 → match-or-beat Tet4 (bug-localizer anchor).
   - **6c.** Tet10 at ν=0.49 → **pre-registered three-way decision gate** on
     the **uniform** sphere, **decision at h/2** (~28k Tet10 DOF — on par with
     the proven Tet4 h/4 solve; **not h/4**, see budget below),
     `max_newton_iter≈150`: ACCEPT iff converged and rel-err ≤ `max(2·e₄₀, 0.10)`;
     REJECT → Taylor-Hood if rel-err > 0.20 **or** Newton stalls; gray zone →
     REJECT (§4). Measure `e₄₀` (6a) at the **same h/2** so the threshold stays
     differential. The locking signal is an *element property* visible at
     moderate mesh (Lamé convergence is already super-quadratic h/2→h/4), so
     h/2 resolves ACCEPT/REJECT; run **h/4 Tet10 as a ONE-SHOT pre-push
     mesh-stability confirmation only, never in CI.** **★ Budget reality: the
     h/4 Tet10 sphere is ~226k DOF** (66k edges dominate 9.5k corners), ~80–150×
     the Tet4 factor, **0.5–4 GB** (the ν=0.49 LU-fallback path doubles fill) →
     **tens of minutes, OOM risk on 7–16 GB CI runners** — do not reuse the
     Tet4 "57k-tet/multi-minute" figure. Add an **LM-retry / line-search-backtrack
     cap** so a stalling ν=0.49 solve fails the gate cleanly instead of thrashing
     the factor for an hour. Convergence, not accuracy, is the likely failure
     mode — the gate must not crash on it.
     **★ Load-consistency (weaker confound than 6d, but tighten it):** the
     cavity pressure is applied as an *equal per-node split* `p·A_v·n̂`,
     `A_v=4πR²/N_loaded` (`scene.rs:280-296`) — inconsistent for a P2 surface.
     Unlike 6d, the **total resultant is exactly node-density-independent**
     (`p·4πR²` for any split), so the error is self-equilibrated and the
     Saint-Venant *mean* suppresses it to **second order** — 6c is materially
     safer than 6d. But it is not provably below the tight threshold, so:
     apply the cavity pressure as the **consistent P2 boundary-face load**
     `∫p·N_i dA` (cheap here — prescribed pressure, no barrier), and select the
     loaded/pinned bands by **6-node boundary-face membership**, not the
     Tet4-tuned `0.5·cell_size` radial predicate (which injects borderline
     radial-edge midsides into both the load and `cavity_wall_mean`). Or, at
     minimum, A/B equal-split vs consistent-P2 on the uniform sphere and
     pre-register the bias bound.
   - **6d.** Demand #1: the #676 gate validates the element-stiffness cure on
     **net force only** (bare Tet10 with per-vertex contact already moves the
     ratio via displacement order; correct *local* mechanics come with rung 8).
     **★ Node-density control (required — the barrier is fixed-κ, non-area-
     weighted, `ipc.rs:37`).** `F_FEM(δ)` is node-density-dependent: Tet10
     ~doubles surface nodes, and a fixed-κ barrier's effective compliance scales
     with node count, shifting `δ_eff` by up to `O(d̂)` (`d̂/δ ≈ 5%` here) — a
     *contact-discretization* shift the gate would otherwise misattribute to
     element order. So the 6d comparison must either (a) run a Tet4 mesh refined
     to Tet10's surface-node count as the baseline (isolating the element-order
     shift), or (b) verify the barrier is near-rigid (`sd_v ≪ d̂`) at this κ so
     `δ_eff ≈ δ` independent of node count (note `contact_stability.rs` shows κ
     has a convergence ceiling — raising it is not free). Without this control
     the ~9% signal is confounded at the few-percent level. Still the slow IPC
     route — and **already CI-dark** (release-only, unregistered): accept it as
     pre-push, register it, or add a cheap contact micro-oracle.
7. **Gradient + dynamics path (§3.5).** Per-GP adjoint RHS + tangent, lift the
   fbar/`N==4` guards for Tet10 as each is made Tet10-correct, **FD-gradcheck**
   every channel — `material_sensitivity`, `dirichlet_reaction_sensitivity`,
   `state_sensitivity` (the per-vertex contact path is unchanged here — friction
   FD moves to rung 8 with the face barrier). `NewtonStepVjp` load-θ needs a
   consistent Tet10 load vector. **Add a Tet10 dynamics mass-gate** (a
   `state_sensitivity`-style small-dt scene: positive mass, SPD tangent,
   FD-checked `StateStepVjp`, + a free-fall total-mass conservation check) —
   every accuracy oracle is static and mass-blind, so without this a wrong HRZ
   mass ships undetected. Silent-wrong surface is *concentrated* (§3.5): the
   type system catches widening; FD gates catch the runtime `0..4`/single-GP
   sites and the mass. **→ At this point the Tet10 element foundation is
   complete and validated; rung 8 builds consistent contact ON it.**
8. **★ CONSISTENT-CONTACT RUNG (the isolated-rung decision — its own isolated rung on the
   proven foundation).** The genuinely-consistent scheme, done once, properly:
   **surface-integrated barrier `E=∫b(sd)dA` at face Gauss points** (gradient
   distributed via P2 face shape functions) **+ isoparametric curved surface**
   (project boundary midside nodes onto the true surface) **+
   `boundary_vertex_areas` 6-node** (restores pressure readouts) — they share
   the face-primitive machinery, so build them together. Requires a new
   `#[non_exhaustive]` `ContactPair::Face` variant + downstream fixes to the
   irrefutable `let Vertex{..}` matches in coupling; friction
   reconciled with the face-distributed force (FD-gate `contact_fd.rs` /
   `friction_*` here). Gate: uniform-pressure face → P2 load (corners ≈ 0) *by
   integration*; a varying-`b(sd)` face (flat face on the curved indenter)
   recovers the correct sub-face distribution; and #676 *local* patch mechanics
   (not just net force) tighten. This is the "obsessively-perfect contacts" work
   — isolated from the element bring-up so a failure localizes cleanly.

   *(Interim note: between 3b and rung 8, Tet10 runs with per-vertex contact —
   correct net force, but `peak_contact_pressure` readouts NaN-drop midside
   contacts. That is a known, diagnostic-only degradation, not a solve error;
   #676 measures net force, not peak pressure.)*

Per the standing covenant: per-crate cargo only, soft tests `--release`,
`xtask grade` before push, gating cold-reads before merge.

**★ Shipping / CI (verified, agent round 3):** Fast new tests (patch/rank/
goldens/ν=0.4) **auto-run** in tests-debug (it discovers every `tests/*.rs`).
But **release-only tests are hand-registered** in the `--test` list in
`.github/workflows/quality-gate.yml` — anything `#[cfg_attr(debug_assertions,
ignore)]` (the ν=0.49 gate, the dynamics mass-gate, #676) runs in **no
CI job** unless added there. **Note the #676 demand-#1 gate is *already*
CI-dark** (release-only, unregistered — a pre-push-only gate today); decide
explicitly whether the ν=0.49 **h/2 decision** solve (~28k DOF, CI-feasible)
and #676 become CI-run, while the ν=0.49 **h/4 confirmation** (~226k DOF,
GB-scale, OOM-risk) stays **pre-push one-shot, never CI**. Register whatever
becomes CI-run in the release `--test` list. Coverage is **skipped** for sim-soft (`grading_profile =
"integration-only"`), so there is *no unit-coverage net* — correctness rests
entirely on these integration gates. Any *new* dependency counts against
sim-soft's **L0** dep cap (~100); the rank-gate eigensolver adds none
(nalgebra is already in-tree). The `MeshTopology` supertrait is nominally a
public-API change (grade "API") but has **zero external implementors**, so the
blast radius is in-crate only.

---

## 6. Risks the build must actively manage

- **The 3b unpin-trio (highest).** `{free-DOF unpin + HRZ mass +
  Hessian-incidence}` is genuinely inseparable — unpinning without either the
  mass diagonal or the incidence slot crashes the factor at `factor.rs:387`.
  (The *plumbing* in 3a is safe to stage first; the earlier "one atomic rung of
  six" over-scoped this — geometry and boundary stage separately.)
- **★ Negative-mass landmine, invisible to every current gate.** Naive Tet10
  row-sum lumping gives negative corner masses → indefinite tangent → Cholesky
  failure at small dt. But at `STATIC_DT` the inertial term is ~4 orders below
  stiffness, so **all shipped oracles (Lamé, #676) are mass-blind** and would
  pass with a wrong mass. Use HRZ (§2) and gate it with the step-7 dynamics
  test — not a static construct check.
- **★ ν=0.49 is a convergence risk before it is an accuracy risk.** The Lamé
  docstring says the pressure-inflation mode makes "convergence collapse";
  pure Tet10 may stall before producing a number. The 6c gate is three-way
  (converged-accurate / converged-locked / didn't-converge), budget ~150 iters,
  and must route both failure modes to Taylor-Hood rather than crash.
- **Gradient silent-wrong (high, but concentrated).** The adjoint RHS/tangent
  and the mass must move to per-GP/HRZ in lockstep with the forward; a residual
  `0..4`/single-GP site does not error, it biases the gradient. FD gates are
  the only net for the runtime sites (the type system covers the rest).
- **Tet4 byte-identity.** Preserve the `det.abs()/6.0` weight and the
  edge-vector Jacobian (§3.1); the current goldens are NH-only — extend them to
  Yeoh + roller before step 4, or ULP drift there goes uncaught.
- **Friction × face-consistent contact.** Friction reads the per-vertex contact
  gradient with hard `λ=|force|` logic; the rung-8 face redistribution perturbs it.
  Reconcile or guard frictionless, and FD-gate friction in step 8 — not just
  the barrier.
- **Accuracy is unproven until step 6.** Treat "Tet10 fixes ν=0.49" as a
  hypothesis to falsify (the oracle's own docstring contra-indicates it), not a
  premise. Target the pre-registered rel-err threshold, not "→1.0".
- **Silent diagnostic regression (interim, accepted).** Pressure/`peak_contact_pressure`
  readouts NaN-drop midside contacts from 3b until rung 8's `boundary_vertex_areas`
  6-node upgrade — a known diagnostic-only degradation (#676 measures net force,
  not peak pressure), not a solve error.
- **★ Tet10 node-ordering silent-wrong.** The edge→midside-node convention must
  be pinned to the step-1 canonical table and cited by every piece; the
  `(min,max)` dedup key must not decide the local slot. The rank/eigenspectrum
  gate is permutation-INVARIANT and cannot catch this — only the step-5
  asymmetric/quadratic patch test on an irregular element does.
- **★ Fabricated-confidence claims (process risk).** The "nodal lumping fixes
  consistent contact" claim survived two rewrites before the round-3 meta-audit
  falsified it (barrier ≠ traction; singular P2 corner area). Any confident
  claim introduced by a review pass — especially a *scope reduction* justified
  by "cheaper and meets the same gate" — must be code/math-verified before it
  ships; the gate being satisfiable ≠ the claim being true.

---

## 7. Deferred follow-ons (named, not silent) + open questions

**Consistent contact is rung 8, not a deferral (the isolated-rung decision).** It is a
first-class rung of this plan — *sequenced after* the element foundation, not
dropped. The round-2 "nodal lumping" shortcut was falsified (§3.5); the real
work is the **surface-integrated barrier + isoparametric curved surface +
`boundary_vertex_areas` 6-node**, built together on the proven foundation
(§5 rung 8). Listed here only to record what rung 8 contains, not as a
someday-maybe:
- **Rung 8 = surface-integrated barrier `E=∫b(sd)dA` at face GPs** (makes
  contact *forces* consistent — needed even for a straight-edged face on the
  curved indenter, where `b(sd)` varies across the face) **+ curved surface**
  (geometric curvature) **+ `boundary_vertex_areas` 6-node** (pressure
  readouts) **+ a `#[non_exhaustive]` `ContactPair::Face` variant** + downstream
  fixes + friction reconciliation. The "obsessively-perfect contacts" work.

**Genuinely deferred (out of this plan entirely):**
- **`fbar.rs` on Tet10 / mixed-u-p.** F-bar's single-GP premise breaks; if a
  cure-on-top for ν=0.49 is needed it is Taylor-Hood P2-P1, a redesign, not a
  port (triggered only if step 6c REJECTs).

**Open questions carried into the build:**
- **Pure Tet10 vs Tet10 + Taylor-Hood at ν=0.49** — resolved empirically at
  step 6c.
- **Tet10 mass-lumping scheme** — decided at step 3b; must be a real scheme,
  not `volume/10`.
- **Mixed-element Tet4↔Tet10 interfaces** (band-`sdf` tagging, midside-node DOF
  elimination per `03-mixed.md`) — deferred; the initial rung is uniform-Tet10
  test meshes only.

---

*Companion state (the F-bar findings, the Lamé oracle, the recon anchors) is
mirrored in the session memory `project-tet10-fbar-element-upgrade`.*
