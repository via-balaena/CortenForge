# Sim-Soft Roadmap — Toward Professional-Grade Layered-Cavity FEM-Viz

**Status:** active (opened 2026-05-11)
**Predecessor docs:** [`VIEWER_DESIGN.md`](VIEWER_DESIGN.md) (cf-view Q1–Q8), [`V1_ROADMAP.md`](V1_ROADMAP.md) (workspace v1.0 cut), [`sim/docs/ARCHITECTURE.md`](../sim/docs/ARCHITECTURE.md), [`sim/L0/soft/EXAMPLE_INVENTORY.md`](../sim/L0/soft/EXAMPLE_INVENTORY.md)

---

## Target deliverable image

A rendered scene of a **3–5 layer silicone cavity being simulated penetrated by an object, with visible squishing** — at fidelity comparable to commercial FEM-viz tools (Abaqus/CAE, ANSYS Mechanical, FEBio+PostView, Houdini-with-FEM-cache).

Verbatim user statement (2026-05-11): *"i want to literally see a 3-5 layer silicone cavity being simulated penetrated by an object and see things squishing."*

Concrete decomposition:

- **Multi-layer silicone body** — 3–5 distinct layers, visually distinguishable (per-layer color or material-id scalar)
- **Cavity geometry** — production-target insertion cavity (row 25 open-mouth U-shape topology)
- **Indenter** — rigid primitive (sphere / capsule / scan-derived) physically pushing into the cavity
- **Visible squishing** — clear deformation of cavity walls and surrounding layers as the indenter advances (ramp animation, not single static frame)

---

## Current state inventory (as of 2026-05-11)

### Solver (sim-soft + mesh-* + cf-design)

- ✅ Multi-element FEM (Phase 2, PR #218)
- ✅ SDF→tet bridge: BCC + Isosurface Stuffing (Phase 3, PR #219)
- ✅ Multi-material per-tet MaterialField (Phase 4, PR #220)
- ✅ Quasi-static ramp solver (rows 21+22, PR #234)
- ✅ Yeoh hyperelastic foundation (F1–F4.0, PR #235; M=1 + M=2 shipped via F4.1 row 23 + row 24 v3 axial-zoned; M=3 plumbing in place but no row uses it until post-cast C₃)
- ✅ Rigid contact primitives (capsule, sphere, scan-derived)
- ✅ LayeredScalarField + BlendedScalarField composition primitives (PR #231)
- ✅ Bit-exact NH↔Yeoh additive decomposition
- ✅ faer Cholesky linear solver
- ⏳ faer LU fallback (Cholesky SPD pivot fails at deep penetration on capsule contacts; parked)
- ❌ Friction (Coulomb-Stribeck) — Phase 5.5, parked
- ❌ Self-contact (cavity walls touching themselves) — absent
- ❌ Damping (Rayleigh or numerical) — absent
- ❌ 3-param Yeoh post-cast C₃ — gated on Fork B physical cast
- ❌ Adaptive BCC cell refinement — uniform-cell only

### Viz (sim_soft::viz + mesh-offset)

- ✅ F1 analysis-mesh primitives (FEM-mesh raw output)
- ✅ F2.0 `design_slab_cut` (marching squares on design SDF)
- ✅ F2.1 `design_surface` (marching cubes on design SDF, with edge-cache dedup)
- ✅ F2.3a `design_surface_deformed` (with stress contours, amplify factor)
- ✅ F2.3b `design_scene` (body + contact merged with primitive_id scalar)
- ✅ F2.3c per-step ramp animation (PLY series, rows 22/23/24/25)
- ✅ F2.3d MC edge-cache dedup (6× vertex reduction)
- ❌ Smooth shading (today: flat-per-triangle)
- ❌ k-nearest-tet weighted scalar interp (today: scattered per-tet dots)
- ❌ Per-primitive bounds in design_scene (mesh-sdf far-field "wing" artifacts)
- ❌ Per-tet material-id scalar plumbing through design_surface_deformed
- ❌ Multi-scalar overlay (stress + displacement + material-id)
- ❌ Stress tensor invariants (von Mises, principal stresses) — currently exports `per_tet_psi` only
- ❌ Cutting planes (interactive cross-section)
- ❌ Wireframe / transparent / cutaway modes
- ❌ Known bug: missing-tet band on BCC+IS for CSG bodies (visible slits)

### cf-view (Bevy viewer)

- ✅ Single PLY rendering
- ✅ Camera orbit
- ❌ PLY sequence playback / timeline
- ❌ Animation controls (play/pause/step/scrub/speed/loop)
- ❌ Scalar-field toggle + color-map UI
- ❌ Camera bookmarks / zoom-to-fit
- ❌ Probe tool
- ❌ Light setup + shadows
- ❌ Per-object visibility toggle (show/hide body, indenter, etc.)
- ❌ MP4 / screen-recording export

### Validation

- ❌ Iter-1 physical cast (Fork B post-cast modulus calibration) — C1 north star
- ❌ Force-displacement curve vs measured
- ❌ Convergence study (mesh-refinement sanity check)
- ❌ Real-anatomy scan SDF integration (vN via mesh_sdf bridge)

---

## Five tracks

### Track A: Solver fidelity

| # | Leaf | Notes |
|---|------|-------|
| A1 | Missing-tet band fix (BCC+IS on CSG bodies) | Upstream blocker; visible slits in viz |
| A2 | faer LU fallback for SPD pivot failure | Unblocks 8mm penetration + finer cells |
| A3 | Adaptive BCC refinement near contact / cavity walls | Uniform cells waste resolution |
| A4 | Coulomb-Stribeck friction | Phase 5.5 parked; insertion mechanics |
| A5 | Damping (Rayleigh or numerical) | Smoother ramp dynamics |
| A6 | Self-contact (cavity walls touching themselves) | Required for deep penetration |
| A7 | 3-param Yeoh (post-cast C₃) | Gated on E1 |

### Track B: Material composition (the "3–5 layers")

| # | Leaf | Notes |
|---|------|-------|
| B1 | Per-tet material-id scalar plumbing through `design_surface_deformed` + `design_scene` | Materially distinguishable → visually distinguishable |
| B2 | LayeredScalarField robustness at finer cells | Gated on A2 |
| B3 | Per-layer material params (modulus, damping, friction coefficients) | Currently homogeneous within a layer |
| B4 | Per-layer mesh-resolution control | Finer cells in stiff / contact-prone layers |

### Track C: Viz quality

| # | Leaf | Notes |
|---|------|-------|
| C1 | Smooth shading: vertex normals from SDF gradient | Replaces flat-per-triangle |
| C2 | k-nearest-tet weighted scalar interp | Smooths per-tet scalar dots |
| C3 | Per-primitive bounds in `design_scene` | Fixes mesh-sdf "wing" artifacts |
| C4 | Multi-scalar overlay (stress + displacement + material-id) | FEM-viz table stakes |
| C5 | Cutting planes (interactive cross-section) | Indispensable for cavity inspection |
| C6 | Wireframe / transparent / cutaway modes | Standard FEM-viz controls |
| C7 | Stress tensor invariants (von Mises, principal stresses) | FEM-viz table stakes beyond raw `per_tet_psi` |

### Track D: cf-view player (interactive playback)

| # | Leaf | Notes |
|---|------|-------|
| D1 | PLY sequence playback (load indexed PLY directory + timeline) | Today needs ffmpeg/Blender/pyvista |
| D2 | Animation controls (play/pause/step/scrub/speed/loop) | Standard playback UI |
| D3 | Scalar-field toggle + color-map controls | Picker UI |
| D4 | Camera bookmarks / orbit / zoom-to-fit | Workflow QoL |
| D5 | Probe tool (click → scalar values) | FEM-viz table stakes |
| D6 | Light setup + shadows | Depth perception for layered geometry |
| D7 | MP4 / screen-recording export | Shareable artifacts |
| D8 | Per-object visibility toggle | Hide indenter to inspect cavity walls under deformation |

### Track E: Validation (sim matches reality)

| # | Leaf | Notes |
|---|------|-------|
| E1 | Iter-1 physical cast → Fork B modulus calibration | C1 north star |
| E2 | Force-displacement curve vs measured | Quantitative gate |
| E3 | Real-anatomy scan SDF integration | vN via mesh_sdf bridge |
| E4 | Convergence study (mesh-refinement sanity check) | Do finer cells give similar answers? Credibility gate |

---

## Convergence target (MVP image)

The minimum set of leaves required to produce **the target deliverable image** — defined as **visual only**: a clean, smooth, layered, animated squish at first-pass professional fidelity. Realistic moduli (E1) are **not** part of MVP — the user's verbatim ask ("see things squishing") is a visual gate, not a quantitative one. Validation lives in Slice S4.

- **A1** (clean geometry) + **A2** (solver converges to depth)
- **B1** (layers visible) + **B2** (finer cells)
- **C1** (smooth shading) + **C2** (smooth scalar interpolation)
- **D1** (sequence playback) + **D2** (basic controls)

≈ **8 sub-leaves across 4 tracks** (validation deferred). Each is ~3–10 commits.

Scope estimate (assumes current pace continues — baby-steps cadence + two-pass review):

- **MVP image** (visual only): 4–8 weeks of focused arcs
- **MVP + validated** (adds E1, E2): 6–12 weeks
- **Professional-grade across all tracks**: 4–8 months

## PR shape (horizontal slicing)

Ship as horizontal slices, **not** vertical track-completes. Each slice produces a visible step-change in output. MVP image completes cleanly at end of S2.

| Slice | Leaves | Outcome |
|-------|--------|---------|
| **S1 — MVP image, first pass** | A1 + B1 + C1 + D1 | First time you see a layered, animated squish (basic) |
| **S2 — MVP image, visually complete** | A2 + B2 + C2 + D2 | 8mm penetration + finer cells + smooth scalar + scrub controls — **MVP target image lands here** |
| **S3 — FEM-viz parity** | C4 + C5 + C7 + D3 + D5 | Multi-scalar + cutting planes + von Mises + color-map UI + probe tool |
| **S4 — validation + advanced solver** | E1 + E2 + E4 + A6 + A3 | Post-cast calibration + force-disp curve + convergence study + self-contact + adaptive refinement |
| **S5+ — polish & long tail** | C3, C6, D4, D6, D7, D8, A4, A5, A7, B3, B4, E3 | Workflow QoL + remaining solver fidelity + real-anatomy scans |

### Bold-choice note: front-load Track D

cf-view (Track D) is a small, high-leverage crate at the visibility boundary. Building D1+D2 early means every subsequent solver/viz improvement is **visible immediately**, without round-tripping through ffmpeg/Blender. S1 includes D1 and S2 includes D2 for this reason.

---

## Update protocol

This doc is **load-bearing** while the arc is active. Update on every slice ship:

1. Move the slice's leaves from ❌ to ✅ in the inventory.
2. Add a one-line entry to the "Slice ship log" below with date + commit SHA + PR #.
3. If a leaf splits or new sub-leaves emerge, edit the track table and add a note in the slice log.
4. If a leaf is dropped or re-scoped, mark it 🚫 with a one-line reason — do not delete; we want the audit trail.

Each slice must pass the user's standing gates before it ships:
- **A-grade-or-it-doesn't-ship** — `xtask grade` clean across affected crates.
- **Eyes-on-pixels review** — required for any viz/D-track work. Visual review IS the test for viz commits.
- **Two-pass review** — numbers pass (Claude) + visuals pass (user) before declaring complete.

When **MVP image** ships (end of S2), update the [target deliverable image](#target-deliverable-image) section with the actual rendered output path (PLY index + screenshot or MP4).

Archive trigger: move this doc to `docs/archive/` when it's no longer load-bearing for active work (judgment call — typically once the named slices have shipped and remaining items are absorbed into individual followup specs or just lived-with backlog).

### Slice ship log

_(empty — populated as slices ship)_

---

## Open questions

- **S1 sub-leaf ordering**: should A1 (missing-tet band) land before B1 (material-id plumbing), or in parallel? A1 unblocks visual quality on every row; B1 unblocks the "3–5 layers" axis specifically. Both feed S1's MVP image.
- **D1 implementation surface**: should PLY sequence playback be a new cf-view subcommand (`cf-view play <dir>`) or a flag on the existing single-file mode? Affects API stability.
- **Material-id colormap**: discrete (categorical, one color per layer) or continuous (gradient by stiffness)? Suggests discrete for clarity, but flag for user preference.
- **MVP image source row**: row 24 v3 (axial-zoned, has multi-layer composition) or a new row purpose-built for the canonical demo (open-cavity + 3–5 layers + indenter)?
