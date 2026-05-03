# Changelog

All notable changes to mesh-lattice will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### v0.9 candidates

These backlog candidates are gated on a real consumer driving them per
the platform's "examples drive gap-fixes" discipline. Each entry names
the consumer-arrival shape that re-opens the work. The platform-wide
list (sixteen candidates spanning four crates) lives in
[Part 10 of the mesh architecture study](../../docs/studies/mesh_architecture/src/100-roadmap.md);
the entries below are the mesh-lattice-specific subset.

- **Real Voronoi tessellation.** `LatticeType::Voronoi` is currently
  perturbed-cubic (a stand-in, not a Voronoi diagram). *Trigger*: a
  user reports cubic-grid artifacts in Voronoi output, OR an
  organic-feeling lattice is required for a layered-silicone-device
  payload. Effort: ~200-400 LOC of new Bowyer-Watson or
  Lloyd's-algorithm code.

- **Welded TPMS-lattice marching-cubes output.** `extract_isosurface`
  emits un-welded vertex-soup output (the locked signature
  `vertex_count == 3 × triangle_count`). `mesh-shell` already runs a
  weld pass via `mesh-repair::weld_vertices`; apply the same pattern.
  *Trigger*: a real consumer needs welded TPMS output for visual
  aesthetic OR file-size compression. Effort: ~30 LOC.

- **Demo non-default `BeamCap` variants (`Flat`, `Butt`).** The
  default `BeamCap::Round` is exercised by `mesh-lattice-strut-cubic`;
  `Flat` and `Butt` are unexercised at the example layer. *Trigger*:
  a user reports surprise that `BeamCap::Flat` / `BeamCap::Butt`
  aren't visually demonstrated. Augment `mesh-lattice-strut-cubic`
  with a sub-demo, OR add a separate example. Effort: ~80 LOC
  additive.

- **0%-infill early-return in `generate_infill` still uses the gap-a
  `shell = mesh.clone()` pattern.** The 100%-infill case is
  semantically defensible (solid = no shell distinction); the
  0%-infill case is semantically wrong (a hollow part SHOULD have
  an inward-offset shell, not a clone of the input mesh). Strict-(b)
  deferral from the F6 gap-a sub-arc. *Trigger*: a consumer reports
  the 0%-infill output looks wrong. Effort: ~30 LOC; extend the
  offset-mesh path to cover the early-return.

- **Non-convex inputs whose AABB inset includes outside-part regions**
  (e.g., torus-hole). The F6 gap-e cavity-SDF approach uses
  face-normal-of-closest-face with a sign convention tuned for the
  CCW-INWARD inner_offset. For a non-convex input where the AABB
  inset overlaps outside-part regions, the closest face for
  outside-part points is on the same side of the cavity, so the
  negated distance still reads negative — the lattice fills the
  outside-part region as if it were cavity. Convex inputs unaffected;
  documented in source. *Trigger*: a non-convex-input consumer
  arrives. Effort: ~50-100 LOC; extra explicit-shape-SDF intersection,
  or analytical-SDF clip on the input bounds.

- **`shell_thickness == 0` edge case.** Inherited from gap-a:
  `offset_mesh(mesh, -0, ...)` returns a near-degenerate inner_offset,
  fed to the cavity-SDF gives wrong signs across the interior, the
  lattice ends up empty or wrong. Pre-existing, not a regression.
  *Trigger*: a consumer needs `shell_thickness == 0` semantics
  (probably a hollow-only or solid-only path). Effort: ~20 LOC;
  explicit branch + clamp.

- **Explicit `LatticeParams::layer_height: f64` field.** The F6 gap-c
  cap thickness uses `cell_size / resolution` as a heuristic
  (FDM-typical 0.4-0.6 mm/layer across the three preset constructors).
  The `resolution` field is validated `>= 2` per
  `LatticeParams::validate`, so the division can never DIV0, but the
  heuristic is implicit. *Trigger*: a real consumer wants direct
  control of the layer-height heuristic. Effort: ~30 LOC additive.

- **Workspace-wide `#[non_exhaustive]` audit on v1.0.0-shipping
  public structs.** `LatticeResult` was marked `#[non_exhaustive]`
  during the v1.0.0 cut after a "Pure addition" semver claim was
  found to be technically wrong without the marker (adding a public
  field is strict-semver-breaking for struct-literal callers when
  the struct is exhaustive). Other public structs across
  mesh-{types, lattice, sdf, measure, io, offset, shell,
  printability} have not been audited; absent the marker, every
  future field addition becomes a v2.0.0-required breaking change
  for external struct-literal callers. *Trigger*: before broad
  external consumer adoption forces v2.0.0, OR a downstream
  consumer reports a struct field addition broke their build.
  Effort: ~30 LOC across ~5-8 files; low-risk additive markers
  with build/clippy verification per crate.

## [1.0.0] - 2026-05-03

### Added

- **`LatticeError::OffsetFailed(String)` error variant.** Wraps
  `mesh-offset` failures encountered during `generate_infill`'s
  inward-offset shell construction (F6 gap a). The wrapped string
  carries the upstream error context. Pure addition; existing
  exhaustive-match callers gain one new arm.

- **`LatticeResult::nodes: Vec<Point3<f64>>` field +
  `with_nodes(nodes)` builder.** Exposes the unique grid-node
  positions that participate in at least one emitted strut.
  Populated by strut-based lattices (Cubic / OctetTruss / Voronoi);
  empty for TPMS lattices (Gyroid / SchwarzP / Diamond), which have
  no graph-node concept since their geometry is the
  marching-cubes-extracted surface itself. Used by `generate_infill`
  to anchor the F6 gap-b lattice-to-shell bridging struts; also
  consumable by future graph-aware downstream tools (FEM bridges,
  optimization passes) that need explicit node positions without
  reverse-engineering them from strut vertex layouts. Pure addition.

- **`mesh-offset` and `mesh-sdf` workspace dependencies.** Required
  by `generate_infill`'s F6 fix arc:
    - `mesh-offset` provides the `offset_mesh` call that builds the
      inward-offset shell for cavity construction (gap a).
    - `mesh-sdf` provides the `SignedDistanceField::distance` and
      `closest_point` queries used to bound lattice nodes inside the
      cavity (gap e) and to anchor lattice-to-shell connections
      (gap b).

- **`LatticeResult` marked `#[non_exhaustive]`.** Future field
  additions to `LatticeResult` are now non-breaking for external
  consumers; struct-literal construction is no longer permitted
  outside the crate (use `LatticeResult::default()` + builder
  methods such as `with_nodes` instead). `LatticeError` and
  `LatticeType` were already `#[non_exhaustive]` pre-arc; this
  closes the gap surfaced when the `LatticeResult::nodes` addition
  was claimed as "Pure addition" — strictly true only with the
  marker.

### Fixed

#### F6 — `generate_infill` cavity correctness sub-arc

The F6 sub-arc closed five gaps in `generate_infill`'s shell + lattice
+ caps composition. Prior to v1.0.0, the function returned a result
that aggregated the input mesh, an unbounded lattice, and an
incorrectly-named "shell" field (a clone of the input). The fix arc
restored the documented contract: a hollow shell with inward-offset
inner surface, a lattice strictly bounded to the cavity interior,
solid caps closing the lattice along the build axis, and bridging
struts connecting near-shell lattice nodes to the inner shell wall.

The fixes ship together as the v1.0.0 cavity-correctness milestone.
Post-fix anchor verification in the
`mesh-lattice-mesh-bounded-infill` example's `main.rs` (under
`examples/mesh/`) locks the behavioural deltas for future
maintainers.

- **Gap a — real shell offset.** `generate_infill` now invokes
  `mesh_offset::offset_mesh(mesh, -shell_thickness, &OffsetConfig)`
  to build the inward-offset surface and combines it with the input
  mesh to form a hollow shell. Prior code set `shell = mesh.clone()`
  at the post-clamp non-degenerate case, returning a solid shell that
  contradicted the documented "outer surface combined with
  inward-offset inner surface" contract. Adds the
  `LatticeError::OffsetFailed` failure mode for upstream offset
  failures; the `0%-infill` early-return path retains the legacy
  `shell = mesh.clone()` pattern as a strict-(b) deferral (see v0.9
  candidates above).

- **Gap b — lattice-to-shell bridging struts.** Near-shell lattice
  nodes (within `2 * cell_size` of the inner shell wall) now emit a
  bridging strut from the node to its closest point on the
  inner-offset shell, anchored via `LatticeResult::nodes` and the
  inner shell's `SignedDistanceField::closest_point`. Without these
  bridges the lattice floated free of the shell, leaving a thin air
  gap that broke FDM/SLA print preparation. A cap-band filter
  excludes nodes whose closest point lies in the top/bottom cap-band
  z-range (avoids double-anchoring through the gap-c caps). Skipped
  for TPMS lattices via `lattice_result.nodes.is_empty()`, since
  TPMS surface-based geometry terminates at cell boundaries with
  implicit shell adjacency.

- **Gap c — solid caps at top and bottom.** `generate_infill` now
  emits axis-aligned outward-CCW closed-box cap geometry at the
  top and bottom interior z-bounds when `solid_caps = true`. The
  lattice iteration domain shrinks by `cap_thickness` along z so
  lattice struts do not intrude into the cap bands.
  `cap_thickness = cell_size / resolution` heuristically tracks
  FDM-typical 0.4-0.6 mm/layer across the three preset constructors;
  see the `layer_height` v0.9 candidate above for explicit-field
  follow-up. Cap-overlap edge case (`2 × cap_thickness >=
  interior_height_z`) returns `LatticeError::InteriorTooSmall`.

- **Gap d — proper signed-volume integrals.** `generate_infill`'s
  internal volume math now routes through
  `cf_geometry::IndexedMesh::volume()` (a thin
  `signed_volume().abs()` wrapper) instead of a
  byte-for-byte-duplicate private function. The deletion removes
  ~50 LOC of dead code and resolves an inconsistency where the
  duplicate silently skipped out-of-bounds face indices while the
  platform method panics fail-fast (no current production paths
  exercise OOB). Empirical witnesses on the cube fixture:
  `shell_volume` 71842.624 → 17195.179 mm³ (matches analytical
  wall target 50³ − 47.6³ = 17149.824 within ~45 mm³ of marching-cubes
  chamfer), `lattice_volume` 609.461907 → 39.906451 mm³ reflecting
  the un-welded marching-cubes vertex multiplicity, `interior_volume`
  53157.376 unchanged.

- **Gap e — SDF-bounded interior.** `generate_infill` now bounds
  lattice nodes to the inward-offset shell's cavity via a
  face-normal-signed `mesh-sdf` query on `inner_offset`, replacing
  the prior unbounded AABB-inset lattice that overflowed the cavity
  on non-rectangular inputs. The face-normal-of-closest-face approach
  (using `SignedDistanceField::closest_point` plus the closest
  face's outward normal, with sign negation to match the CCW-INWARD
  inner_offset orientation) avoids a 36% false-negative rate that
  ray-crossing parity-based `point_in_mesh` exhibits on the
  un-welded marching-cubes inner_offset where overlapping triangle
  clusters confuse parity counting. Convex inputs only; non-convex
  inputs whose AABB inset includes outside-part regions are tracked
  as a v0.9 candidate above.

## [0.7.0]

Initial release.
