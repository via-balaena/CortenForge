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
- ✅ faer LU fallback (A2, shipped 2026-05-11 in commit `dda6cf7c`; unblocks row 21 v1.5 capsule-cap apex contact via `FactoredFreeTangent` enum routing both factor sites through `factor_free_tangent` — acceptance test at `scan-fit-3layer-sleeve-v15` confirms vertex 13271 trip resolves cleanly. Row 22/25 deeper-penetration walls remain validity-bound, NOT Llt — A2 does not unblock those)
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
- ✅ Smooth shading (C1, PR #238 — SDF-gradient analytical normals on `design_surface`, area-weighted normals on `design_surface_deformed` + `boundary_surface`)
- ❌ k-nearest-tet weighted scalar interp (today: scattered per-tet dots)
- ❌ Per-primitive bounds in design_scene (mesh-sdf far-field "wing" artifacts)
- ✅ Per-tet material-id scalar plumbing through design_surface_deformed (B1, PR #238 — row 25 emits `material_shell_id` + `material_zone_id` integer scalars)
- ❌ Multi-scalar overlay (stress + displacement + material-id)
- ❌ Stress tensor invariants (von Mises, principal stresses) — currently exports `per_tet_psi` only
- ❌ Cutting planes (interactive cross-section)
- ❌ Wireframe / transparent / cutaway modes
- ❌ Known bug: missing-tet band on BCC+IS for CSG bodies (visible slits)

### cf-view (Bevy viewer)

- ✅ Single PLY rendering
- ✅ Camera orbit
- ✅ PLY sequence playback (D1, PR #238 — `cf-view <dir>` auto-detects + loads + renders one frame from a lex-sorted directory; bottom panel shows `Frame N/M — <filename>`)
- ⏳ Animation controls — D1.3 ships keyboard step (`←` / `→` / `Home` / `End`); scrub timeline + play/pause/speed/loop still ahead (D2)
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
| A2 | faer LU fallback for SPD pivot failure | Unblocks row 21 v1.5 capsule-cap apex contact (parked at `project_sim_soft_row_21_v1_5_spec.md`); latent enabler for B2/B4 finer-cell layered-field cases. Does NOT unblock row 22/25 deeper penetration — those walls are validity-bound (NH/Yeoh `max_stretch_deviation`), upstream of any factorization |
| A3 | Adaptive BCC refinement near contact / cavity walls | Uniform cells waste resolution |
| A4 | Coulomb-Stribeck friction | Phase 5.5 parked; insertion mechanics |
| A5 | Damping (Rayleigh or numerical) | Smoother ramp dynamics |
| A6 | Self-contact (cavity walls touching themselves) | Required for deep penetration |
| A7 | 3-param Yeoh (post-cast C₃) | Gated on E1 |

### Track B: Material composition (the "3–5 layers")

| # | Leaf | Notes |
|---|------|-------|
| B1 | Per-tet material-id scalar plumbing through `design_surface_deformed` + `design_scene` | Materially distinguishable → visually distinguishable |
| B2 | LayeredScalarField robustness at finer cells | NOT gated on A2 on row 25's current geometry. Evidence experiment 2026-05-11: halving `CELL_SIZE = 0.004 → 0.002` on row 25 (~433k tets) ran 3 ramp steps cleanly through Llt (iters 8/13/17, residuals 2.85e-11/9.34e-11/3.44e-11) with zero LU fallback engagements. Pre-A2 docstring claim "finer cells trip SPD at first ramp step" was falsified. The parked 459K-tet 2 mm-cell spike that tripped Llt (row 22 v2 spec line 305) was on a different row's geometry; A2 may be load-bearing there but not here. Productionizing finer cells on row 25 is a re-bake-all-EXACT-and-REF_BITS task, banked as a B2 followup |
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

- **A1** (clean geometry) + **A2** (solver survives indefinite-tangent contact cases)
- **B1** (layers visible) + **B2** (finer cells)
- **C1** (smooth shading) + **C2** (smooth scalar interpolation)
- **D1** (sequence playback) + **D2** (basic controls)

≈ **8 sub-leaves across 4 tracks** (validation deferred). Each is ~3–10 commits.

### MVP image done-criteria (strawman)

The MVP image "lands" when a single rendered scene or short animation — from row 25 (`scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth`, the current production-target row) emitted via `cf-view` — passes ALL of these:

1. **Per-layer material distinction is visible without per-tet noise.** B1 ships `material_shell_id` + `material_zone_id` scalars; C2 smooths the scattered per-tet sampling pattern into continuous fields. The image must read as "this is three concentric silicone layers" at a glance, not "these are 38k dots colored by layer membership."
2. **Animated ramp plays cleanly in cf-view from rest to peak deformation.** D2 shipped scrub + play + speed + loop. With B2's finer cells the per-step deformation should be smooth (no visible mesh-resolution-bound popping or jitter between frames).
3. **Smooth shading reads as a continuous surface, not a polygonal mesh.** C1 shipped SDF-gradient analytical normals on the design-surface MC output. The deformed boundary mesh should look like a curved silicone surface, not flat-per-triangle facets.
4. **The squish is unambiguous.** A layperson seeing the animation should be able to articulate "the object is going into the cavity, and the cavity walls are deforming around it" without expert interpretation. The deformation magnitude should be visible at unamplified scale (no `DISPLACEMENT_SCALE = 10×` cheating) — i.e., the contact penetration produces real, eye-visible wall displacement at the chosen `PROBE_PENETRATION_FINAL`.
5. **Cavity AND indenter both visible in the same frame.** Row 25 emits `design_scene_deformed` which merges body + contact primitives. The indenter (cuboid plug in row 25's case) and the cavity walls must both be readable in the rendered frame — no compositing tricks, single PLY through `cf-view`.

**Out of MVP scope (not blockers):** quantitative force-vs-displacement curves, validated modulus values, multi-scalar overlays (stress + displacement + material-id together), interactive cutting planes for cavity inspection. All deferred to S3 (FEM-viz parity) or S4 (validation).

**Done-criteria revision protocol:** strawman is editable. When MVP image is closer to landing (post-B2 + post-C2), refine each criterion against the actual rendered output. Out-of-MVP items can move IN if they turn out to be load-bearing for the "visible squishing" framing.

Scope estimate (assumes current pace continues — baby-steps cadence + two-pass review):

- **MVP image** (visual only): 4–8 weeks of focused arcs
- **MVP + validated** (adds E1, E2): 6–12 weeks
- **Professional-grade across all tracks**: 4–8 months

## PR shape (horizontal slicing)

Ship as horizontal slices, **not** vertical track-completes. Each slice produces a visible step-change in output. MVP image completes cleanly at end of S2.

| Slice | Leaves | Outcome |
|-------|--------|---------|
| **S1 — MVP image, first pass** | A1 + B1 + C1 + D1 | First time you see a layered, animated squish (basic) |
| **S2 — MVP image, visually complete** | A2 + B2 + C2 + D2 | Indefinite-tangent contact survival + finer cells + smooth scalar + scrub controls — **MVP target image lands here**. Row 25 depth stays capped at 4 mm (validity-bound wall, separate followup); A2 covers row 21 v1.5 capsule-cap apex case |
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

- **S1 (partial)** — PR #238, 2026-05-11. Shipped: **D1** (cf-view auto-detect dir input + keyboard frame navigation), **B1** (per-tet `material_shell_id` + `material_zone_id` scalars through `design_surface` / `design_surface_deformed` / `boundary_surface`), **C1** (SDF-gradient analytical normals + area-weighted normals on deformed/boundary outputs). **A1** deferred — F2 architecturally masks the missing-tet band for design-mesh primitives; banked as Track A followup. Adjacent fixes that landed in the same slice: row 25's `iter_count==0` ramp-emit skip (cuboid-plug-into-open-mouth load case only engages pinned rim DOFs for steps 1–4), and cf-view's colormap detector noise tolerance (fp-noise negatives no longer flip integer scalars to Divergent). Commits: `61e41efc` (roadmap), `9ea098c6` (D1.1), `96443779` (D1.2), `1585e8f5 + b1415f86` (D1.2 polish), `e347020d` (D1.3), `66f297c5` (row 25 zero-iter), `7b8157ac` (detector noise), `b4afb911` (B1), `995c436d` (C1).
- **S2 (partial)** — A2 arc on dev, 2026-05-11. Shipped: **A2** (faer LU fallback at the two condensed-tangent factor sites in `CpuNewtonSolver`). New `pub(crate) FactoredFreeTangent { Llt(_), Lu(Box<_>) }` enum forwards `solve_in_place_with_conj`; happy path bit-identical to pre-A2 code, fall-through engages on `LltError::Numeric(NonPositivePivot)` and factors via Lu against a cached `SymbolicLu`. Acceptance test at new sibling crate `scan-fit-3layer-sleeve-v15` (row 21.5 in EXAMPLE_INVENTORY): pre-A2 the v1.5 capsule + capsule geometry was PARKED with documented Llt non-PD pivot at Newton iter 2-3 (vertex 13271 at 1 mm); post-A2 the fallback engages 4× at iters 2-5 (vertex 13271 at iter 2 matches the parked memo exactly), Newton converges in 92 iters to 9.05e-11. B2 + C2 + D2 still ahead for S2 completion. Commits: `415239b4` (A2 framing in roadmap), `b85e05a3` (SymbolicLu cache), `dda6cf7c` (Lu fallback + enum + unit tests), `3c116fda` (row 21.5 unpark).

---

## Open questions

- ~~**S1 sub-leaf ordering**: should A1 land before B1, or in parallel?~~ **Resolved in PR #238:** D1 first (visibility-boundary force-multiplier) → B1 (lowest-risk visible payoff) → C1 (single biggest visual win) → A1 deferred (deepest leaf, F2 masks the artifact for design-mesh primitives, ~multi-hour algorithmic investigation in BCC+IS).
- ~~**D1 implementation surface**: subcommand `cf-view play <dir>` or flag on existing single-file mode?~~ **Resolved in PR #238:** auto-detect on path type. `cf-view <path>` dispatches to single-frame or sequence mode based on `path.is_dir()`. No CLI restructure, no subcommand ceremony.
- **Material-id colormap**: discrete (categorical, one color per layer) or continuous (gradient by stiffness)? **Partly resolved in PR #238:** producer emits integer-valued f64 scalars; cf-view's Q5 detector picks Categorical when values are integer-clean, falls back to Sequential (viridis) when barycentric interpolation produces fractional values at layer boundaries — perceptually-ordered viridis happens to match the natural layer ordering well. A "pure categorical even with fractional samples" path would require nearest-tet sampling for named scalars; deferred unless visual review demands it.
- ~~**MVP image source row**: row 24 v3 or a new row?~~ **Resolved in PR #238:** **row 25** (`scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth`). Production-target geometry (open-mouth insertion cavity), already has 3-shell × 2-zone material composition + indenter + ramp animation. No new row needed.
- ~~**B2 trip-mode evidence**: does B2 actually need A2?~~ **Resolved 2026-05-11:** halved `CELL_SIZE = 0.004 → 0.002` on row 25 (~433k tets, 2.9× normal) and ran the ramp. **Outcome (c) — converged cleanly through Llt; zero LU fallback engagements through 3 ramp steps before the experiment was called for time** (each step ~10 min at this mesh density; full 8-step ramp would be ~2-3 hours). Iter counts 8/13/17 at depths 0.5/1.0/1.5 mm; residuals 2.85e-11/9.34e-11/3.44e-11. **B2 is NOT gated on A2 on row 25's current geometry** — A2 was incidental, not load-bearing. The pre-A2 docstring claim "finer cells trip SPD at first ramp step" on `CELL_SIZE` was falsified (likely stale prose from a prior row version, since corrected). The parked 459K-tet 2 mm-cell Llt trip (row 22 v2 spec line 305) was on a different row's geometry; A2 may still be load-bearing there but not on row 25. Implication: B2 as a roadmap leaf is now "productionize finer cells on row 25 by re-baking captured-bits at the larger mesh + re-running visuals pass" — a polish-shaped task, not a solver-fidelity-shaped task.
