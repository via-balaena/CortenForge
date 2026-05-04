# cf-viewer

Workspace-wide Bevy-CLI viewer for static-artifact visual review.
Replaces `f3d` (mesh-v1.0 examples) and MeshLab + filter-dialog chains
(sim-soft Tier 1) with a single `cf-view <path>` binary.

## What it does

Opens a PLY file in a Bevy 3D window with auto-discovered per-vertex
scalars colormapped by their value distribution. Mesh inputs render
as shaded triangle meshes; point-cloud inputs render as instanced
spheres. Side-panel dropdowns switch the active scalar and colormap
at runtime; orbit-zoom-pan camera; input geometry is auto-converted
from its up-axis convention to Bevy's Y-up internally.

## Quickstart

```text
cargo run -p cf-viewer -- examples/sim-soft/sphere-sdf-eval/out/sdf_grid.ply
```

The viewer auto-discovers per-vertex scalars in the PLY's `extras`
and selects the alphabetical-first by default; switch scalars via
the side-panel dropdown. The colormap auto-detects (any negative →
divergent; integer + < 16 unique → categorical; else sequential).

## CLI

```text
cf-view <path> [--scalar=<name>] [--colormap=<auto|divergent|sequential|categorical>] [--up=<+X|+Y|+Z>]
```

| Flag | Default | Effect |
| --- | --- | --- |
| `<path>` | required | Path to the PLY file. |
| `--scalar=<name>` | alphabetical-first | Pre-select a per-vertex scalar (must match an `extras` key). Runtime-overridable via the side-panel scalar dropdown. |
| `--colormap=<kind>` | `auto` | Pre-select colormap kind. `auto` defers to value-distribution detection. Runtime-overridable via the side-panel colormap dropdown. |
| `--up=<+X\|+Y\|+Z>` | `+Z` | Which input axis is up. `+Z` matches mesh-v1.0's build-plate convention + legacy `f3d --up=+Z`. NOT runtime-overridable — geometry is mapped to Bevy-Y-up at load. |

## Inputs

- **PLY only (v1)** — binary + ASCII, via `mesh-io::load_ply_attributed`. OBJ / STL deferred (no per-vertex-scalar support in those formats).
- **Per-vertex scalars** in `extras` are auto-discovered (any number; alphabetical-first picked on launch).
- **Per-face scalars** are NOT supported in v1 — `mesh-io::AttributedMesh.extras` is per-vertex only. Deferred until a real consumer asks.
- **Geometry-only PLYs** render with default shading; no scalars required (this is the dominant mesh-v1.0 retrofit case).

## See also

- [`docs/VIEWER_DESIGN.md`](../docs/VIEWER_DESIGN.md) — design plan with originating signal, locked decisions (Q1-Q8), open issues, and iteration log. Living document; will be deleted after the workspace fully migrates per `feedback_code_speaks`.
- [`examples/sim-soft/sphere-sdf-eval`](../examples/sim-soft/sphere-sdf-eval) — v1 reference consumer (1331-point PLY with two per-vertex scalars: `signed_distance` divergent + `gradient_magnitude` sequential) exercising auto-discovery + dropdown switching + colormap detection end-to-end.

## Tests

`cargo test -p cf-viewer` — 56 unit tests covering colormap detection, scalar extraction, CLI parsing, mesh conversion, and orbit-camera framing. The Bevy window itself is not tested in CI (no headless display server on the runners); local visual review against `examples/sim-soft/sphere-sdf-eval` is the v1 test surface for the rendering path.
