# `mesh-printability-trapped-volume`

Visual demo of the §6.3 `TrappedVolume` detector (Gap H of the v0.8 fix
arc).

## ⚠ f3d viewer callout — use MeshLab or ParaView with a clipping plane

`f3d`'s default rendering back-face-culls the REVERSED-wound inner
sphere (the cavity shell's normals point INTO the cavity, away from the
camera if you are outside the cube), so `f3d --up=+Z out/mesh.ply` shows
only the outer cube. **The cavity is not missing — `f3d` is hiding it
by design**, exactly the way the platform's directed-edge topology
sees it: the REVERSED inner shell is what tells a printer slicer "this
is a cavity, not a solid blob inside the cube".

To see the cavity:

- **MeshLab** (`meshlab out/mesh.ply`): use the slice plane (`Filters
  → Selection → Slice using a plane`) or simply enable `Render →
  Show Edges` and rotate; the inner sphere shows up as a wireframe.
- **ParaView** (`paraview out/mesh.ply`): apply a `Clip` filter at
  `x = 10` (cavity midplane) to peek inside.

## What this fixture is

A solid 20 × 20 × 20 mm cube with a sealed sphere cavity (radius 5 mm)
at its centre. Hand-authored as 12 outer cube triangles plus a UV-
tessellated sphere (32 segments × 16 stacks = 960 triangles) with
**REVERSED winding** so sphere normals point INTO the cavity. Both
shells are individually watertight + consistently wound and they share
no vertices — combined, every undirected edge appears in exactly two
faces.

## What the detector reports — multi-technology severity sweep

| Technology | TrappedVolume severity | Cavity-ceiling overhang | `is_printable()` |
|------------|------------------------|-------------------------|------------------|
| FDM        | `Info`                 | Critical (`90 > 75`)    | `false`          |
| SLA        | `Critical`             | Critical (`90 > 60`)    | `false`          |
| SLS        | `Critical`             | NOT flagged (`max=90°`) | `false`          |
| MJF        | `Critical`             | NOT flagged (`max=90°`) | `false`          |

`classify_trapped_volume_severity` (`validation.rs:1415`) is technology-
aware: a sealed cavity prints fine on FDM (extrusion doesn't trap
material) but is a hard failure on SLA (uncured resin traps), SLS, and
MJF (unsintered powder traps). The detector reports the same region on
all four techs but classifies severity per the `PrintTechnology` arm.

Each tech fails `is_printable()` but for a *different* reason — the
pedagogical point of the example.

## Why a sealed sphere cavity flags differently per tech

- **FDM**: sealed pockets print fine. Extrusion deposits material from
  the build plate up; once the cavity is closed, the empty interior
  stays empty. Severity `Info`.
- **SLA**: liquid resin fills the cavity during printing; once the
  outer shell closes, the interior is sealed with un-cured resin
  forever. Severity `Critical`.
- **SLS / MJF**: powder fills the cavity during build; once the shell
  closes, the interior is sealed with un-sintered powder forever.
  Severity `Critical`.

The detector is the validator; intent is the designer's. The validator
sees a sealed pocket and reports the consequence per technology.

## Multi-detector co-flag — cavity ceiling overhang

The cavity sphere's upper hemisphere has faces whose inward (cavity-
facing) normal points DOWNWARD (toward the cube floor). Peak observed
`overhang_angle ≈ 84°` on a 32 × 16 UV-tessellated sphere (the polar
tri's centroid normal is ~ 6° off vertical due to chord shrinkage).

- **FDM** (`max_overhang_angle = 45°`, predicate `> 45`): the upper-cap
  cluster (224 faces, area 45.5 mm²) flags Critical (`84 > 45 + 30 =
  75`).
- **SLA** (`max=30°`): a larger upper-cap cluster (288 faces, area
  69.2 mm²) flags Critical (`84 > 30 + 30 = 60`). SLA additionally
  fires a *Warning* `LongBridge` issue (160 of those same downward
  faces span 5.6 mm vs SLA's `max_bridge_span = 5.0 mm`) — a tertiary
  co-flag observed only at SLA's tighter span threshold; the example
  doesn't assert against this LongBridge cluster (it isn't load-
  bearing for the per-tech severity story) but the diagnostics
  surface it.
- **SLS** (`max=90°`): SLS's `requires_supports() == false` short-
  circuits `check_overhangs` at `validation.rs:284` before the per-
  face loop runs. Silent skip — no `DetectorSkipped` issue (per §6.2
  line 996).
- **MJF**: same as SLS.

This is the load-bearing co-flag of the example: a sealed cavity has a
ceiling, and the validator flags the ceiling under any tech with a
finite overhang threshold. **Validators see surface geometry, not
interior intent — sealed-cavity ceilings flag as overhang regardless of
designer intent.**

## How to run

```text
cargo run -p example-mesh-printability-trapped-volume --release
```

`--release` is required: SLA's voxel grid is `~ 802³ ≈ 515 MB` (well
under the §6.3 1 GB cap, but heavy in debug). Output is written to
`examples/mesh/printability-trapped-volume/out/`:

- `out/mesh.ply` — 490-vertex, 972-triangle ASCII PLY (12 outer cube
  tris + 960 sphere tris, REVERSED-wound).
- `out/issues.ply` — vertex-only ASCII PLY of the FDM iteration's
  centroids (the richest of the four runs): the trapped-volume
  centroid + cavity-ceiling overhang centroid + matching support-region
  centroid.

The fixture is hand-authored (no implicit-surface meshing); the sphere
tessellation is fully deterministic and reproducible across platforms.

## Numerical anchors

Per tech in `[FDM, SLA, SLS, MJF]`:

- `validation.trapped_volumes.len() == 1`
- Voxel-discretized cavity volume within `± 10 %` of analytical
  `(4/3) π r³ ≈ 523.6 mm³` (UV-tessellation chord shrinkage drops the
  meshed sphere by ~ 1.5 % to ~ 516 mm³; the 10 % band absorbs that
  plus voxel discretization plus cross-platform FP drift per §9.6)
- Cavity centroid within per-tech `voxel_size = min(min_feature_size,
  layer_height) / 2` of `(10, 10, 10)`. `voxel_size` is `0.1 mm` for
  FDM, `0.025 mm` for SLA, `0.05 mm` for SLS, `0.04 mm` for MJF.
- `validation.issues` contains exactly one `TrappedVolume` issue at the
  expected severity per the table above.
- FDM / SLA: `validation.overhangs.len() ≥ 1` (cavity ceiling, ≥ 1
  Critical `ExcessiveOverhang` issue).
- SLS / MJF: `validation.overhangs.len() == 0` (silent skip).
- All four techs: `validation.is_printable() == false`, but for
  different reasons.

## v0.8 spec deviation: `out/voxels.ply` deferred to v0.9

v0.8 spec §7.3 line 1665 calls for a third PLY artifact — a point-
cloud of the trapped voxel centres for visualizing the discretized
cavity shape. v0.8's `TrappedVolumeRegion` (`regions.rs:153`) exposes
only `center / volume / bounding_box / voxel_count`; the individual
voxel centres live in the detector's internal `VoxelGrid::states`
(`validation.rs:1442`) and are not surfaced through the public API.
Extending the API to expose them is v0.9 candidate scope; row #15's
mandate is example-only (no source change to `mesh-printability`), so
v0.8 ships the cavity centroid + bounding box in `issues.ply` and
defers the per-voxel point-cloud.

## v0.8 fix arc cross-references

- §6.3 — `TrappedVolume` detector (landed at row #14, commit `7539cc8e`)
- §7.3 — this example's spec
- §9.2.5 — voxel-grid memory cap stress fixture
- `examples/mesh/printability-thin-wall/` — sibling `TrappedVolume`
  consumer (axis-aligned cavity at row #14b, commit `0358cdf9`)
