# Heightfield — Ray Marching on Terrain

An 8×8 grid of downward rays strikes a sinusoidal heightfield terrain.
Hit points form a dot cloud that traces the surface. Unlike convex
primitives (which use analytic intersection), heightfields use **ray
marching** — stepping along the ray until a surface crossing is detected,
then refining with binary search.

**One concept:** Ray marching on non-analytic geometry (heightfields, SDFs).

## What you see

- **Green terrain mesh** — sinusoidal surface: `z = 0.3 sin(2πx/4) cos(2πy/4)`
- **64 cyan ray lines** descending from z=3 onto the terrain
- **Green dots** at each hit point, forming a grid that follows the terrain
- **Yellow arrows** — surface normals, tilting with the terrain slope

The dot cloud visually confirms that ray marching tracks the surface
accurately — dots sit right on the mesh, not floating above or below.

## Physics

The terrain is a `HeightFieldData` with 64×64 samples at cell_size ≈ 0.127m
(8m extent). The MJCF loads a placeholder hfield, which is replaced
programmatically with `HeightFieldData::from_fn` before geom spawning.

Ray marching steps at half-cell intervals. When the ray crosses from above
to below the surface, 8 binary-search refinement steps narrow the hit to
sub-millimeter precision.

| Parameter | Value |
|-----------|-------|
| Grid | 64×64 samples |
| Extent | ~8m × 8m |
| Amplitude | 0.3m |
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
