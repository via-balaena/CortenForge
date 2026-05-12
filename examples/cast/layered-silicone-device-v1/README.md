# Cast Track F — Layered Silicone Device v1

Stage 2 F4 example wrapper. Builds a 3-layer [`CastSpec`][1] against
the layered-silicone-device v1.0 geometry and writes all cast
artifacts (mold STLs + procedure markdown) to `out/`.

## Run

```bash
cargo run --release -p example-cast-layered-silicone-device-v1
```

## Output

Writes to `out/` (relative to this crate's `Cargo.toml`):

| File                 | Purpose                                          |
| -------------------- | ------------------------------------------------ |
| `mold_layer_0.stl`   | Innermost shell mold cup (Ecoflex 00-30 pour).   |
| `mold_layer_1.stl`   | Middle shell mold cup (Dragon Skin 10A pour).    |
| `mold_layer_2.stl`   | Outermost shell mold cup (Ecoflex 00-30 pour).   |
| `plug.stl`           | Shared printed plug for the innermost cast.     |
| `procedure.md`       | Workshop-Markdown cast procedure (F3 generator). |

## Geometry

Capsule plug per the casting roadmap [Q2 resolution][2] (scan-derived
primary, capsule fallback). Body cumulative shells are
`capsule.offset(thickness) ∖ capsule` at 6 / 10 / 14 mm — same
thicknesses as row 25's `scan-fit-3layer-sleeve-yeoh-axial-zoned-
ramp-open-mouth` sim fixture.

| Param              | Value                                                |
| ------------------ | ---------------------------------------------------- |
| Plug radius        | 12 mm                                                |
| Plug half-height   | 20 mm (capsule total z-extent 64 mm)                 |
| Inner layer        | 6 mm Ecoflex 00-30                                   |
| Middle layer       | 4 mm Dragon Skin 10A (cumulative outer = 10 mm)      |
| Outer layer        | 4 mm Ecoflex 00-30 (cumulative outer = 14 mm)        |
| Bounding region    | 70 × 70 × 110 mm cuboid (printable on most FDM bays) |
| Marching-cubes cell | 2 mm                                                |

Expected pour masses: ~42 g innermost, ~43 g middle, ~57 g outermost
— **141 g total**, well under the 2 lb (907 g) per-pour budget.

## Scan-Derived Swap-In

For iter-2+ with a real scanned reference geometry, replace the
capsule with the scan's signed-distance field:

```rust
use mesh_io::load_stl;
use mesh_sdf::SignedDistanceField;
let scan_mesh = load_stl("scan.stl")?;
let scan_sdf = SignedDistanceField::from_mesh(&scan_mesh, ...);
let cavity = Solid::from_sdf(scan_sdf, scan_bounds);
```

The shell-construction CSG (`cavity.offset(thickness) ∖ cavity`) and
the rest of the wrapper is geometry-agnostic — only the
`build_spec()` plug + body lines need to change.

## Sanitization

Per the layered-silicone-device memo, the cavity geometry is
referred to as "scanned reference geometry" or the capsule
placeholder; no anatomical references appear in this crate.

[1]: ../../../design/cf-cast/src/spec.rs
[2]: ../../../docs/CASTING_ROADMAP.md
