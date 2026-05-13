# Layered Silicone Device v2 — Curve-Following Multi-Piece Cast

Closes the v2 cf-cast code-side arc from
[`docs/CURVE_FOLLOWING_DESIGN.md`](../../../docs/CURVE_FOLLOWING_DESIGN.md).
End-to-end demonstration of every v2 feature (Steps 5-10) on a
synthetic curved body that exercises the curve-following demold
geometry the v2 architecture was built for.

## What this produces

```
$ cargo run --release -p example-cast-layered-silicone-device-v2-scan-curve-following

out/
├── mold_layer_0_piece_0.stl  ← innermost Ecoflex shell, Negative-side piece
├── mold_layer_0_piece_1.stl  ← innermost Ecoflex shell, Positive-side piece
├── mold_layer_1_piece_0.stl  ← middle Dragon Skin shell, Negative-side piece
├── mold_layer_1_piece_1.stl  ← middle Dragon Skin shell, Positive-side piece
├── mold_layer_2_piece_0.stl  ← outer Ecoflex shell, Negative-side piece
├── mold_layer_2_piece_1.stl  ← outer Ecoflex shell, Positive-side piece
├── plug.stl                  ← shared inner-cavity plug
└── procedure.md              ← workshop procedure with v2 sections
```

8 files total for a 3-layer cast (vs v1's 4 files). The `_piece_0` /
`_piece_1` suffix maps to `PieceSide::Negative` / `PieceSide::Positive`
per `docs/CURVE_FOLLOWING_DESIGN.md` §"Output artifacts".

## v2 features exercised

| Step | Feature | Where it lights up |
|------|---------|--------------------|
| 5 | Per-piece SDF composition | `compose_piece_solid` (internal to `export_molds_v2`) |
| 6 | Marching cubes per piece + STL export | `CastSpec::export_molds_v2` |
| 7 | Per-piece printability + AABB checks | Each piece's `mesh-printability::validate_for_printing` |
| 8 | v2 procedure markdown | `CastSpec::write_procedure_v2` |
| 9 | Registration pins (cylindrical) | `Ribbon::with_registration(RegistrationKind::Pins(PinSpec::iter1()))` |
| 10 | Pour gate + air vent | `Ribbon::with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()))` |

## Geometry summary

- **Plug**: `Solid::pipe` along a 5-point centerline polyline,
  8 mm radius
- **Centerline**: gentle arc in XZ plane, ~80 mm long, max tangent
  rotation ~30°
- **Layers** (innermost-first):
  - Inner: Ecoflex 00-30, 6 mm thick (14 mm cumulative radius)
  - Middle: Dragon Skin 10A, 4 mm thick (18 mm cumulative radius)
  - Outer: Ecoflex 00-30, 4 mm thick (22 mm cumulative radius)
- **Bounding region**: 120 × 120 × 120 mm cube centered at origin
- **Ribbon split-normal**: `+Y` → pieces split top/bottom of the
  curved tube
- **Registration pins**: 2 per layer-piece-pair, 3 mm Ø × 10 mm long,
  25 mm offset from centerline
- **Pour gate**: 6 mm Ø at centerline base end, 40 mm long
- **Air vent**: 3 mm Ø at centerline tip end, 40 mm long

## Scan-driven swap-in

This example uses synthetic `Solid::pipe` geometry. To run on a
real cf-scan-prep-cleaned scan, see the module docstring in
[`src/main.rs`](src/main.rs) — replace the geometry construction
with `mesh_io::load_stl` + `mesh_sdf::SignedDistanceField::from_mesh`
+ `Solid::from_sdf`, and parse the centerline polyline from
`.prep.toml`'s `[centerline]` block. The CastSpec construction
(layers, materials, bounding region, cell size, printer config,
budget) is geometry-agnostic — the same wrapper drives both
pipelines.

## Workshop next steps

After running this example, the printable STLs + `procedure.md` are
ready for an FDM printer + the cast iter-1 workshop activity per
[`docs/CASTING_ROADMAP.md`](../../../docs/CASTING_ROADMAP.md) Stage 3.

For the iter-1 fixture (`sock_over_capsule.stl`, repo-excluded per
sanitization), follow the scan-driven swap-in pattern above with
the cf-scan-prep-produced `cleaned.stl` + `.prep.toml`.
