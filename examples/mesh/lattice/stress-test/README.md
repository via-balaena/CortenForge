# lattice — Stress Test (mesh-lattice validation superset)

Headless validation of the `mesh-lattice` generation paths against
closed-form and analytic oracles. No window, no Bevy — self-gating
assertions that abort (exit 101, or return `Err`) on any mismatch, so
`cargo xtask run-validators` runs it red-or-green.

Folded from five former per-crate examples (`mesh-lattice-tpms-gyroid`,
`-strut-cubic`, `-density-gradient`, `-shape-bounded`,
`-mesh-bounded-infill`), each now a module preserving its hand-authored
fixture and oracle checks verbatim. One domain → one stress-test.

## Modules

### `tpms_gyroid` — TPMS implicit-surface path
30 mm cube at density 0.5, `LatticeType::Gyroid`. Anchors the `gyroid`
free function at exact analytical points (bit-exact origin, one-cell
periodicity, `sin(π/2)` axis sample), the `density_to_threshold` TPMS
mapping, and `make_shell`, then runs `generate_lattice` and verifies
every marching-cubes vertex lands on the analytical shell surface
`|G(v)| ≈ half_thickness`. The unique closed-form-field oracle.

### `strut_cubic` — cubic strut path
25 mm cube, `LatticeType::Cubic`. Closed-form `generate_strut` /
`generate_strut_tapered` / `estimate_strut_volume` anchors (cylinder
`π·r²·L`, truncated cone), the `combine_struts` no-weld linearity, and
the **`BeamLatticeData`** 3MF beam-lattice precursor — quantize-dedup to
`6³ = 216` grid nodes, 540 beams, bit-exact `total_strut_length = 2700`.

### `density_gradient` — spatially-varying density
30 mm cube, octet-truss, `DensityMap::Gradient` climbing along z. The
only path exercising the full `DensityMap` enum (Uniform / Gradient /
Radial / Function / SurfaceDistance / StressField) and the special
`evaluate_with_distance`, plus the load-bearing proof that density
reaches beam geometry: four discrete cell-strata radii
`r1 = strut_thickness/2 · √density` and a top/bottom mean-r1 ratio ≈ √2.

### `shape_bounded` — analytical-SDF clipping
Gyroid trimmed to a radius-12 sphere via `with_shape_sdf`. Verifies the
`is_outside_shape` predicate (`sdf > 0` convention, `None`-default), that
the trim strictly reduces geometry, and the confinement invariant —
every output vertex within `radius + voxel_size + cushion` of the SDF's
zero level-set. Complementary to `mesh_bounded_infill`'s mesh-boundary
path: analytical SDF vs tessellated B-rep boundary.

### `mesh_bounded_infill` — composite FDM pipeline
50 mm cube input mesh → `generate_infill` producing the assembled
four-part decomposition: welded inward-offset hollow **shell** + interior
**lattice** + solid **caps** + node-to-wall **connections**. Verifies the
shell via the resolution-independent Euler invariant `2V − F = 8` (not a
fragile vertex count), the §Q-5 hollow-shell winding guard
(`shell_volume < cube_volume` catches the inner-wall sign flip), and the
mesh-SDF `interior_volume`. The 50 mm cube fixture lives in the
[`fixture`](src/mesh_bounded_infill/fixture.rs) submodule.

## Run

```
cargo run -p example-lattice-stress-test --release
```

Expected: each module prints its diagnostics + anchors and the binary
exits 0.

## Visual artifacts

Every module writes binary PLY files to the crate's `out/` for the
`cf-viewer` visual-review path: `gyroid_lattice.ply`,
`cubic_lattice.ply`, `density_gradient_lattice.ply`,
`sphere_gyroid.ply` + `sphere_gyroid_full.ply`, and (from
`mesh_bounded_infill`) `input.ply`, `shell.ply`, `lattice.ply`,
`composite.ply`:

```
cargo run -p cf-viewer --release -- examples/mesh/lattice/stress-test/out/gyroid_lattice.ply
```
