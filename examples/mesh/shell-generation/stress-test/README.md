# shell-generation/stress-test

**The `mesh-shell` domain validation superset.** One headless validator
for `mesh_shell::ShellBuilder` — the wall-thickness shell generator that
underpins every casting and printing workflow — exercising both
wall-generation methods on the same 10mm cube geometry and self-gating
against closed-form oracles. Folded from the former
`shell-generation-fast` / `shell-generation-high-quality` pair (each now a
module).

## What it checks

Run it red-or-green with `cargo xtask run-validators` (or
`cargo run --release -p example-shell-generation-stress-test`). Each module
aborts (exit 101) on any oracle mismatch and writes its shell to `out/`
for the `cf-viewer` visual-review path.

| Module | Preset / method | Oracle |
|---|---|---|
| [`fast`](src/fast.rs) | `.fast()` — normal-based offset, open-topped box | Per-vertex offset table (magnitude == wall_thickness; direction from incident-triangle normals) + consistent-winding printable shell |
| [`high_quality`](src/high_quality.rs) | `.high_quality()` — SDF + marching cubes, closed cube | 24-sample wall-thickness uniformity within half a voxel + welded genus-0 printable shell |

## The load-bearing contrast: triangulation-dependent vs uniform walls

Both methods offset the same cube by a nominal 2mm wall, but they trade off
differently:

- **Normal method (`fast`)** duplicates each input vertex along its
  averaged normal, so inner↔outer stay 1:1 — but the *perpendicular* wall
  thickness skews by `1/√k` where `k` is the incident-triangle weight at a
  vertex (down to 0.667mm at a 3:1 diagonal vertex on this triangulation).
- **SDF method (`high_quality`)** marches the outer surface from the SDF
  level set, so perpendicular thickness is uniform (2mm ± half a voxel at
  every wall-interior sample) — at the cost of MC re-triangulation (no 1:1
  correspondence) and Steiner-Minkowski rounding at the cube's edges and
  corners.

## Winding: both methods are consistently wound

A printable shell must be watertight, manifold, and consistently oriented.
Both modules assert `has_consistent_winding == true` with an empty issues
list:

- The **SDF** path (closed input) generates no rim, so there is no
  inner/outer/rim seam to reconcile; its outer MC surface is per-face
  flipped by the 11.5.x engine fix.
- The **normal** path (open input) generates a rim to close the top. Its
  rim triangles are wound to **oppose** the reversed-inner and
  original-outer surface edges, so the assembled shell is a single
  consistently-oriented manifold.

> **History.** The normal path once shipped a rim-winding inconsistency:
> the rim quads matched, rather than opposed, the surface edges at the
> seam, so the shell was watertight + manifold + printable yet
> `has_consistent_winding == false` (one `InconsistentWinding` issue). That
> fixable platform quirk is now fixed in
> `mesh_shell::shell::rim::generate_rim` and guarded by the lib test
> `test_normal_shell_on_open_box_has_consistent_winding`; the `fast` module
> asserts the corrected behavior.

See `docs/studies/mesh_architecture/src/50-shell-and-print.md` for the
depth pass on the printable-shell pipeline, and `examples/mesh/README.md`
for the domain layout convention.
