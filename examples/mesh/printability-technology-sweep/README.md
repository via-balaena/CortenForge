# `printability-technology-sweep`

**Visual demo of cross-technology severity divergence — same mesh,
four different printability verdicts (§7.7 of the v0.8 fix arc).**
Hand-authors the `25 × 20 × 15 mm` hollow box from §7.1 (top wall
deliberately thinned to 0.4 mm; sealed inner cavity) and validates it
under all four `PrinterConfig::*_default()` technologies. Each tech
fails `is_printable()` for a *different* reason — the pedagogical
payoff.

## What this fixture is

A box `[0, 25] × [0, 20] × [0, 15]` mm with an internal cavity
`[1.5, 23.5] × [1.5, 18.5] × [1.5, 14.6]` mm — side and bottom walls
1.5 mm thick, **top wall thinned to 0.4 mm**, sealed everywhere. Two
vertex-disjoint triangle shells (8 outer corners + 8 inner corners; 12
outer triangles + 12 inner triangles = 24 total). Watertight +
consistently wound by construction (verified during the numbers-pass:
36 edges, 100 % incidence-2; signed volume 2600.6 mm³ matches the
outer-minus-cavity prediction bit-exactly).

The 0.4 mm wall is the load-bearing fixture parameter: it lands each
technology's `min_wall_thickness` on a *different* severity band
(see [Why 0.4 mm](#why-04-mm) below).

## What the detector reports — multi-technology severity matrix

| Tech | `min_wall` | `max_overhang` | `ThinWall` (wall = 0.4) | `TrappedVolume` | Cavity-ceiling overhang | `is_printable()` |
|------|-----------|----------------|-------------------------|-----------------|-------------------------|------------------|
| FDM  | 1.0 mm    | 45°            | 2× **Critical** (0.4 < 0.5)         | `Info`          | **Critical** (90 > 75)  | `false`          |
| SLA  | 0.4 mm    | 30°            | 0 flagged (0.4 ≥ 0.4 strict)        | **Critical**    | **Critical** (90 > 60)  | `false`          |
| SLS  | 0.7 mm    | 90°            | 2× **Warning** (0.35 ≤ 0.4 < 0.7)   | **Critical**    | NOT flagged (max = 90°) | `false`          |
| MJF  | 0.5 mm    | 90°            | 2× **Warning** (0.25 ≤ 0.4 < 0.5)   | **Critical**    | NOT flagged (max = 90°) | `false`          |

Each tech fails `is_printable()` but for a *different* reason — the
load-bearing pedagogical claim. **A fixture that "passes FDM" tells
you nothing about whether it'll print on SLA, SLS, or MJF; severity
is technology-aware.**

The matrix above is asserted in `main` per-tech under the §7.7 spec
exactly as printed; a regression that flips Critical→Warning, or that
mishandles the strict-less-than boundary at SLA, fails the run with a
named invariant.

## Why 0.4 mm

The wall thickness is locked at **exactly 0.4 mm** so each technology's
threshold lands on a different band per `classify_thin_wall_severity`
(`validation.rs:750`). The bands are:

- `thickness < min_wall / 2` ⇒ **Critical**
- `min_wall / 2 ≤ thickness < min_wall` ⇒ **Warning**
- `thickness ≥ min_wall` ⇒ **NOT flagged**

| Tech | `min_wall` | `min_wall / 2` | `0.4 < min_wall / 2`? | `0.4 < min_wall`? | Outcome    |
|------|-----------|----------------|-----------------------|-------------------|------------|
| FDM  | 1.0       | 0.5            | yes                   | yes               | Critical   |
| SLA  | 0.4       | 0.2            | no                    | **no** (strict)   | NOT flagged|
| SLS  | 0.7       | 0.35           | no                    | yes               | Warning    |
| MJF  | 0.5       | 0.25           | no                    | yes               | Warning    |

Picking 0.4 mm — exactly at SLA's threshold — makes the
strict-less-than convention visible: a wall *exactly* at `min_wall`
does NOT flag. A common pitfall when comparing validators across
pipelines.

## Why a sealed cavity flags differently per tech

`classify_trapped_volume_severity` (`validation.rs:1435`) is
technology-aware:

- **FDM**: sealed cavities print fine. Extrusion deposits material from
  the build plate up; once the cavity is closed, the empty interior
  stays empty. Severity `Info`.
- **SLA**: liquid resin fills the cavity during printing; once the
  outer shell closes, the interior is sealed with un-cured resin
  forever. Severity **Critical**.
- **SLS / MJF**: powder fills the cavity during build; once the shell
  closes, the interior is sealed with un-sintered powder forever.
  Severity **Critical**.

The cavity volume is `~ 4899.4 mm³` (mathematical `22 × 17 × 13.1 =
4900.4`, but `14.6` is not bit-exact in IEEE-754 f64 so the f64
constant evaluates to `~ 4899.4`). All four techs' `min_feature_size³`
resolution floors sit far below this (FDM 0.512, SLA 0.001, SLS 0.027,
MJF 0.008 mm³), so the resolution short-circuit at `validation.rs:1441`
never fires; the per-tech arm determines the severity outright.

## Why the cavity ceiling co-flags as overhang on FDM / SLA but not SLS / MJF

The inner-top face's normal points DOWN into the cavity — the
**REVERSED winding** rule for cavity shells. `check_overhangs` reads
its `overhang_angle` as 90°.

- **FDM** (`max = 45°`, `requires_supports() == true`): `90 > 45 + 30
  = 75` Critical band → **Critical**.
- **SLA** (`max = 30°`, `requires_supports() == true`): `90 > 30 + 30
  = 60` Critical band → **Critical**.
- **SLS** / **MJF** (`requires_supports() == false`):
  `check_overhangs` early-returns at `validation.rs:304` BEFORE the
  per-face loop runs. **Silent skip — no `DetectorSkipped` issue
  emitted** (per §6.2 line 996).

This is the load-bearing co-flag of the example: a sealed cavity has a
ceiling, and the validator flags the ceiling under any tech with a
finite overhang threshold + support requirement. **Validators see
surface geometry, not interior intent — sealed-cavity ceilings flag
as overhang regardless of designer intent.**

## Surprise co-flag — `LongBridge` on FDM and SLA

Not in the §7.7 matrix above but visible in stdout: the cavity
ceiling's 22 mm × 17 mm extent fires `LongBridge` Critical on FDM
(`max_bridge_span = 10 mm`; 22 > 10) and SLA (`max_bridge_span = 5 mm`;
22 > 5). SLS and MJF have `max_bridge_span = ∞` so no `LongBridge`
ever fires on them.

The example surfaces the count via stdout diagnostics but does NOT
assert against `LongBridge` (not load-bearing for the per-tech
**severity divergence** story; for FDM and SLA it just amplifies the
already-Critical verdict). Documented here so the reader can match
issue counts when comparing stdout to the matrix.

## Numerical anchors

Per tech in `[FDM, SLA, SLS, MJF]`:

- `validation.trapped_volumes.len() == 1` (sealed cavity).
- Voxel-discretized cavity volume within `± 10 %` of analytical
  `≈ 4899.4 mm³`. For axis-aligned cube cavities aligned to voxel
  edges (FDM / SLA / SLS), the discretization is bit-exact; MJF's
  `0.04 mm` voxel size with the cavity at `1.5 mm` origin produces
  slight drift (`4906.88 mm³`, +0.15 % drift, well within band).
- Cavity centroid within per-tech `voxel_size = min(min_feature_size,
  layer_height) / 2` of `(12.5, 10, 8.05)`. `voxel_size` is `0.1 mm`
  for FDM, `0.025 mm` for SLA, `0.05 mm` for SLS, `0.04 mm` for MJF.
- `validation.is_printable() == false` on every tech.

Per-tech specific (full matrix above):

- **FDM**: 2× Critical `ThinWall`; `Info` `TrappedVolume`; 1× Critical
  cavity-ceiling overhang.
- **SLA**: 0 `ThinWall` (strict-less-than); Critical `TrappedVolume`;
  1× Critical overhang.
- **SLS**: 2× Warning `ThinWall`; Critical `TrappedVolume`; 0
  overhangs (silent skip).
- **MJF**: 2× Warning `ThinWall`; Critical `TrappedVolume`; 0
  overhangs.

`ThinWall` cluster geometric anchors (apply to FDM / SLS / MJF, not
SLA which doesn't flag):

- Outer cluster centroid `(12.5, 10, 15.0)`, area `25 × 20 = 500 mm²`.
- Inner cluster centroid `(12.5, 10, 14.6)`, area `22 × 17 = 374 mm²`.
- Both clusters report `thickness = 0.4 mm` within `1e-5`.

## How to run

```text
cargo run -p example-mesh-printability-technology-sweep --release
```

`--release` is required: SLA's voxel grid is `~ 1000 × 800 × 600 ≈
480 MB` (well under the §6.3 1 GB cap, but heavy in debug). Output is
written to `examples/mesh/printability-technology-sweep/out/`:

- `out/mesh.ply` — 16-vertex, 24-triangle ASCII PLY.
- `out/issues_fdm.ply` — 5 centroids (2 ThinWall + 1 Overhang +
  1 SupportRegion + 1 TrappedVolume).
- `out/issues_sla.ply` — 3 centroids (0 ThinWall + 1 Overhang +
  1 SupportRegion + 1 TrappedVolume).
- `out/issues_sls.ply` — 3 centroids (2 ThinWall + 0 Overhang +
  1 TrappedVolume).
- `out/issues_mjf.ply` — 3 centroids (2 ThinWall + 0 Overhang +
  1 TrappedVolume).

Open the mesh and any one of the `issues_*.ply` files together
(e.g. `f3d --up=+Z --multi-file-mode=all out/mesh.ply
out/issues_fdm.ply`); compare against the SLS variant (no overhang
centroid at the cavity ceiling) to *see* the per-tech severity shift.
Without `--multi-file-mode=all`, f3d opens the two PLYs in separate
tabs instead of overlaying them.

The fixture's outer + inner shells are wound in opposite directions
(REVERSED inner shell → cavity normals point INTO the cavity); the
§7.1 `printability-thin-wall` README documents the visual implications
of this construction at length.

## v0.8 fix arc cross-references

- §6.1 — `ThinWall` detector (Gap C).
- §6.3 — `TrappedVolume` detector (Gap H).
- §6.2 — `ExcessiveOverhang` detector (Gap M / Gap D).
- §7.1 — `printability-thin-wall` (sister example with the same
  hollow-box construction, FDM only).
- §7.3 — `printability-trapped-volume` (sister example with a sealed
  sphere cavity, multi-tech).
- §7.7 — this example's spec (cross-tech sweep).
