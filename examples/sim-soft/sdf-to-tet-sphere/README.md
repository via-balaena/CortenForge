# sdf-to-tet-sphere

**`SdfMeshedTetMesh::from_sdf` on a solid `SphereSdf` — BCC + Labelle-Shewchuk Isosurface Stuffing (SIGGRAPH 2007 Theorem 1) on the canonical Phase 3 sphere scene (`R = 0.1` m, `cell_size = 0.02` m, `bbox = [-0.12, 0.12]³`). First triangle-mesh `cf-view` consumer in the sim-soft examples arc — rows 1+2 emitted point clouds, this row emits a closed triangulated boundary surface (1224 faces, 614 vertices, 1836 edges).** A per-vertex scalar `boundary_residual = |‖p‖ − R|` is attached to the PLY for cf-view colormap rendering — the bimodal distribution (warp-snapped ≈ 0 vs cut-point at `O(cell_size²) ≈ 4e-4`) visualizes both stuffing paths.

## What this example demonstrates

`SdfMeshedTetMesh::from_sdf(&dyn Sdf, &MeshingHints) -> Result<Self, MeshingError>` runs the full BCC + Isosurface Stuffing pipeline:

1. Build the BCC lattice spanning `hints.bbox` at `hints.cell_size`.
2. Sample the SDF at every lattice vertex in sequential `VertexId` order (`NaN` / `±inf` surfaces as `MeshingError::NonFiniteSdfValue` deterministically — smallest tripping `VertexId` reported).
3. Apply the SDF sign convention adapter (sim-soft negative-inside → paper positive-inside).
4. Warp lattice vertices in place near the surface (Decision M D-11 deterministic 14-edge walk).
5. Walk each BCC tet through the Labelle-Shewchuk stuffing case table, emitting sub-tets with cut-points along edges where the SDF sign changes.
6. Compute per-tet `QualityMetrics` (`signed_volume`, `aspect_ratio`, `dihedral_min`, `dihedral_max`).

The output is a tet mesh implementing the `Mesh` trait, ready for `CpuTet4NHSolver` consumption (downstream rows). This example exercises the meshing surface only — `material_field: None` selects the `MaterialField::skeleton_default` fallback inside `from_sdf`.

**Why these scene parameters**: `R = 0.1`, `cell_size = 0.02`, `bbox = [-0.12, 0.12]³` is the canonical Phase 3 sphere scene per `sdf_pipeline_determinism.rs` (III-1) and `sdf_quality_bounds.rs` (III-2 / III-4). The III-1 bit-equality contract guarantees every count and metric below is run-to-run deterministic on a fixed hardware/libm, so all counts are pinned EXACTLY — any future regression surfaces at the integer count layer (cheap to read, cheap to diagnose). If a future CI matrix expansion (different libm) trips a count, that is a real discovery; relax only after diagnosing.

**Why a triangle-mesh PLY (vs. row 1+2's point clouds)**: this is the first row in the arc producing a 3-D solid body; the closed boundary surface IS the canonical visual. cf-view renders it as a triangulated sphere with the per-vertex residual scalar colormapped; the bimodal distribution (warp-snapped ≈ 0 vs cut-point ≈ `O(cell_size²) ≈ 4e-4`) makes both stuffing paths visible.

## Numerical anchors

Each anchor is encoded as `assert!` / `assert_eq!` in `src/main.rs` under `verify_*`. Per [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md), a clean `cargo run --release` exit-0 IS the correctness signal; the visual pass below is optional pedagogy.

### Determinism

| Anchor | Bound |
|---|---|
| Second `from_sdf` call `equals_structurally` the first | strict |

### Counts (exact-pin per III-1)

| Anchor | Pinned value |
|---|---|
| `mesh.n_tets()` | 6768 |
| `mesh.n_vertices()` | 4634 |
| `referenced_vertices(&mesh).len()` | 1483 (< 4634 — orphan-rejection holds; 3151 BCC corners outside SDF) |
| Boundary face count | 1224 |
| Boundary vertex count | 614 (subset of referenced; 869 interior tet vertices) |
| Boundary edge count | 1836 (`= 3 · F / 2` by closed-manifold) |
| Euler χ = V − E + F | 2 (topological sphere) |

### Quality floors (Theorem 1)

| Anchor | Bound | Theorem 1 actual |
|---|---|---|
| Per-tet `signed_volume` | `> 0` strict (D-10 detector) | by construction |
| Per-tet `aspect_ratio` | `≥ 0.05` | ≈ 0.08 |
| Per-tet `dihedral_min` | `≥ 5°` | 9.32° |
| Per-tet `dihedral_max` | `≤ 175°` | 161.64° |

### Boundary geometry

| Anchor | Bound |
|---|---|
| Per-edge incidence across boundary faces | `== 2` (closed-manifold) |
| Per-face winding | `normal · centroid > 0` (outward, sphere centred at origin) |
| Per-vertex residual `|‖p‖ − R|` | `≤ cell_size = 0.02` (Eikonal band) |
| Warp-snapped bucket count (residual ≤ `1e-12`) | exact-pin: 38 |
| Cut-point bucket count (residual > `1e-12`) | exact-pin: 576 (`38 + 576 = 614` partitions the boundary vertex set) |

### Volume convergence (III-4 soft-bound)

| Anchor | Bound | Observed |
|---|---|---|
| `\|Σ signed_volume − (4/3) π R³\| / (4/3) π R³` | `≤ 0.15` | `0.0124` |

## Visuals

`out/sphere_boundary.ply` (binary LE, 614 verts + 1224 triangle faces) — closed triangulated boundary surface of the tet mesh, with one per-vertex scalar:

- `extras["boundary_residual"]` — `|‖p‖ − R|` per vertex; non-negative, sequential (auto-detected by cf-view).

Open in `cf-view`, the workspace's unified visual-review viewer:

```text
cargo run -p cf-viewer --release -- examples/sim-soft/sdf-to-tet-sphere/out/sphere_boundary.ply
```

cf-view auto-discovers the per-vertex scalar and selects it by default — so the launch view colour-maps the boundary by residual. What you should see:

- **Sphere boundary surface** centred at the origin, with visible triangle faces tracking the BCC + stuffing tessellation pattern.
- **Bimodal residual distribution** rendered as visible structure on the surface: warp-snapped vertices (residual ≤ `1e-12`, 38 of them) sit at original BCC lattice positions where the warp step pulled them exactly onto the analytic sphere; cut-point vertices (residual ∈ `(1e-12, ~4e-4]`, 576 of them) sit along stuffing-emitted edges where the SDF sign changes. Both populations are spatially distributed across the surface — the colour-mapping renders the split as visible texture, with the specific colour-to-bucket assignment depending on cf-view's auto-colormap behaviour.
- **No holes, no flips** — closed-manifold (every edge incident to two faces) and outward-wound (normals point radially out from the origin).

## Run

```text
cargo run -p example-sim-soft-sdf-to-tet-sphere --release
```

Output: `out/sphere_boundary.ply` (closed triangulated boundary surface). Stdout prints input fixture summary, all 11 anchor-group names, mesh counts, boundary surface counts (with bimodal-bucket breakdown), and volume statistics.

## Cross-references

- **Sister Tier 1 examples**: `sphere-sdf-eval` (row 1 — `Sdf` trait contract on `SphereSdf`) and `hollow-shell-sdf` (row 2 — `DifferenceSdf` composition). See `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 1.
- **`SdfMeshedTetMesh` impl**: `sim/L0/soft/src/sdf_bridge/sdf_meshed_tet_mesh.rs` — pipeline orchestration; `lattice.rs` (BCC); `stuffing.rs` (warp + dispatch).
- **III-1 / III-2 / III-4 internal fixtures**: `sim/L0/soft/tests/sdf_pipeline_determinism.rs` (bit-stability contract — `signed_volume`/`aspect_ratio`/`dihedral_min`/`dihedral_max` `to_bits()` equality), `sim/L0/soft/tests/sdf_quality_bounds.rs` (Theorem 1 floors + `(4/3) π R³` convergence at three resolutions).
- **`referenced_vertices` orphan filter**: `sim/L0/soft/src/mesh/mod.rs:214` — sorted-`BTreeSet` walk; downstream consumer is `SoftScene::layered_silicone_sphere` (filters spatial-predicate boundary conditions to drop unreferenced lattice corners).
- **Book reference**: Part 7 §00 §02 (Sdf trait), §00 §03 (BCC + Isosurface Stuffing pipeline), Ch 00 §02 (Mesh trait + structural-equality claim 3).
- **Cadence memos**:
  [`feedback_math_pass_first_handauthored`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md),
  [`feedback_one_at_a_time_review`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_one_at_a_time_review.md),
  [`feedback_thorough_review_before_commit`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_thorough_review_before_commit.md).
