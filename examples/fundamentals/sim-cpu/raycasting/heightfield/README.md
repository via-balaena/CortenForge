# Heightfield — Rays on Terrain

An 8×8 grid of downward rays strikes a sinusoidal heightfield terrain.
Hit points form a dot cloud that traces the surface. `raycast_scene` casts
a height-field geom as MuJoCo's `mj_rayHfield` does: against the field's
triangles, two a cell, and its base and sides, the field centred on its
geom at the asset's half-extents and top.

**One concept:** A scene ray against a height field, as MuJoCo casts it.

## What you see

- **Green terrain mesh** — sinusoidal surface: `z = 0.3 + 0.3 sin(2πx/4) cos(2πy/4)`
- **64 cyan ray lines** descending from z=3 onto the terrain
- **Green dots** at each hit point, forming a grid that follows the terrain
- **Yellow arrows** — surface normals, tilting with the terrain slope

The dot cloud visually confirms that the rays track the surface — dots
sit right on the mesh, not floating above or below.

## Physics

The terrain is a `HeightFieldData` with 64×64 samples at cell_size ≈ 0.127m
(8m extent). The MJCF loads a placeholder hfield whose `size` (half-extents
4 × 4, top 0.6) is the terrain's, and the placeholder's data is replaced
programmatically with `HeightFieldData::from_fn` before geom spawning.

The field is centred on its geom: its extent and top come from the asset's
`size`, its grid from the data's counts. `from_fn` samples from the field's
−x, −y corner, so the example samples the terrain at `x − 4`, `y − 4`.
The heights stay between 0 and the top, as a field loaded from MJCF does.
Each ray meets the two triangles of the cells under it, and the nearest
crossing wins.

| Parameter | Value |
|-----------|-------|
| Grid | 64×64 samples |
| Extent | 8m × 8m, centred on the origin |
| Height | 0 to 0.6m |
| Ray grid | 8×8 (64 rays) |
| Ray origin z | 3.0m |

## Validation

Three automated checks run at startup:

| Check | Condition |
|-------|-----------|
| All 64 rays hit | 100% hit rate |
| Hit z matches terrain | max \|hit.z - terrain(x,y)\| < 0.01 |
| Normal matches gradient | dot(hit_normal, analytical_normal) > 0.95 |

## Run

```
cargo run -p example-raycasting-heightfield --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
