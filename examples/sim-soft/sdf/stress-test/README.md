# sdf — Stress Test (sim-soft `Sdf` validation superset)

Headless validation of the sim-soft SDF surface — primitive field eval,
sharp-CSG composition, and SDF → tet meshing — against closed-form,
analytic, and topology oracles. No window, no Bevy — self-gating
assertions that abort (exit 101) on any mismatch, so `cargo xtask
run-validators` runs it red-or-green.

Folded from three former per-concept examples (`sphere-sdf-eval` row 1,
`hollow-shell-sdf` row 2, `sdf-to-tet-sphere` row 3), each now a module
preserving its hand-authored fixture and oracle checks verbatim. One
domain → one stress-test.

## Modules

### `sphere_eval` — `Sdf` trait contract on `SphereSdf`
Unit sphere `SphereSdf { radius: 1.0 }` at the origin. Asserts
`eval(p) = ‖p‖ − 1` and `grad(p) = p/‖p‖` at axis-aligned, off-axis, and
Pythagorean-triple anchors (bit-exact where dyadic, `1e-15` otherwise),
the documented `Vec3::z()` origin-singularity fallback, then sweeps an
11³ = 1331-point grid in `[−2, 2]³` verifying the identity at every
sample. Unique axis: a **finite-difference Eikonal diagnostic** —
FD-gradient magnitude in `[0, 1.05]`, `= 0` at the origin dip, `= 1`
on-axis, and the smoothed-curvature dip `≈ 0.874` in `[0.85, 0.90]`.

### `hollow_shell` — sharp-CSG `DifferenceSdf` combinator
`SphereSdf{1.0} \ SphereSdf{0.5}` via `DifferenceSdf`
(`φ = max(φ_a, −φ_b)`, book Part 7 §00 §01). A thick-walled hollow
shell: body interior between the two operand surfaces, cavity and
exterior both outside the body. Validates outer/cavity-surface `eval = 0`,
shell-interior/cavity/exterior signs, **inward-normal cavity gradients**
(`grad = −p/‖p‖` on the inner surface), and **branch-flip tie-break
determinism** (`≥` selects the outward a-branch), then sweeps a 49² = 2401
z = 0 slice checking the closed-form difference and re-derived
`active_branch` predicate at every sample.

### `sdf_to_tet` — `SdfMeshedTetMesh::from_sdf` (BCC + Isosurface Stuffing)
Solid `SphereSdf{0.1}` through the Labelle-Shewchuk stuffing pipeline
(SIGGRAPH 2007 Theorem 1) on the canonical Phase 3 scene
(`cell_size = 0.02`, `bbox = [−0.12, 0.12]³`). Exact-pins counts
(6768 tets / 4634 verts / 1483 referenced; 1224 boundary faces / 614
verts / 1836 edges per III-1 determinism), Theorem-1 quality floors
(`aspect_ratio ≥ 0.05`, dihedrals ∈ `[5°, 175°]`, strictly-positive
`signed_volume`), closed-manifold topology (per-edge incidence `== 2`,
**Euler χ = 2**, outward winding), and a `(4/3)πR³` volume-convergence
soft-bound (`≤ 0.15`, observed `0.0124`). Headline: exact integer pins
surface any future libm/hardware regression at the cheapest-to-diagnose
layer — a real discovery, not a threshold to relax blindly.

## Run

```
cargo run -p example-sdf-stress-test --release
```

Expected: each module prints its anchor-group summary (`Anchor groups (all
assertions exit-0 on success)`) and the binary exits 0 — a clean exit-0 IS the
correctness signal (there is no `PASS` token; a failed assert aborts with 101).

## Visual artifacts

Each module writes a human-inspectable PLY to `out/` for the `cf-viewer`
visual-review path (per-vertex scalars auto-discovered and colormapped):

```
cargo run -p cf-viewer --release -- examples/sim-soft/sdf/stress-test/out/sdf_grid.ply          # sphere_eval: signed_distance + gradient_magnitude over the 11³ grid
cargo run -p cf-viewer --release -- examples/sim-soft/sdf/stress-test/out/sdf_slice.ply         # hollow_shell: signed_distance + active_branch over the z=0 slice
cargo run -p cf-viewer --release -- examples/sim-soft/sdf/stress-test/out/sphere_boundary.ply   # sdf_to_tet: boundary_residual over the closed tet-mesh surface
```

## Cross-references

- **Inventory**: `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 1 (rows 1–3).
- **`SdfMeshedTetMesh` impl**: `sim/L0/soft/src/sdf_bridge/sdf_meshed_tet_mesh.rs`
  (pipeline), `lattice.rs` (BCC), `stuffing.rs` (warp + dispatch).
- **Internal fixtures**: `sim/L0/soft/tests/sdf_pipeline_determinism.rs`
  (III-1 bit-stability), `sdf_quality_bounds.rs` (Theorem-1 floors +
  volume convergence).
- **Book**: Part 7 §00 §01 (sharp-CSG difference), §02 (`Sdf` trait),
  §03 (BCC + Isosurface Stuffing).
