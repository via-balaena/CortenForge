# Raycasting — Ray Queries and Shape Intersection

Four examples demonstrating ray intersection queries against all geometry
types. The raycast module is available as a direct API for visibility
checks, distance queries, and custom sensors. The Rangefinder sensor casts
MuJoCo's `mj_ray` (`sensor::ray_geom`), which reads a signed distance
field's shape through this module; a mesh's and a height field's rays are
ports of MuJoCo's `mj_rayMesh` and `mj_rayHfield` (`sensor::ray_mesh`).

## API

- **`raycast_shape`** — test a single shape. Provide the shape, its pose,
  and the ray. Returns `Option<RaycastHit>` with distance, hit point,
  and surface normal.
- **`raycast_scene`** — test all geoms in the model. Returns the closest
  hit with `geom_id`. Supports body exclusion and geom group filtering
  (matches MuJoCo's `mj_ray`).

## Examples

| Example | Concept | Checks |
|---------|---------|--------|
| [stress-test](stress-test/) | Headless: analytical validation of all shapes, normals, scene queries, edge cases | 20 |
| [basic-shapes](basic-shapes/) | `raycast_scene` on 6 primitives — hit dots + normal arrows | 5 |
| [scene-query](scene-query/) | LIDAR fan of 36 rays, body exclusion, geom group filtering | 4 |
| [heightfield](heightfield/) | Ray marching on sinusoidal terrain, dot cloud tracing the surface | 3 |

## Run

```
cargo run -p example-raycasting-stress-test --release
cargo run -p example-raycasting-basic-shapes --release
cargo run -p example-raycasting-scene-query --release
cargo run -p example-raycasting-heightfield --release
```
