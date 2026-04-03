# Raycasting — Ray Queries and Shape Intersection

**Status:** Draft
**Prerequisite:** `feature/raycast-consolidation` branch (raycast_scene API)
**API surface:**
- `sim_core::raycast_shape(shape, pose, origin, direction, max_distance) → Option<RaycastHit>`
- `sim_core::raycast_scene(model, data, origin, direction, max_distance, bodyexclude, geomgroup) → Option<SceneRayHit>`
- `sim_core::RaycastHit { distance, point, normal }`
- `sim_core::SceneRayHit { geom_id, hit }`

## Context

The raycast module supports ray intersection queries against all geometry types.
Used internally by the Rangefinder sensor but also available as a direct API for
visibility checks, distance queries, and custom sensors. After the raycast
consolidation, `raycast_shape` delegates to `cf_geometry::ray_cast` for all 10
shape types, and `raycast_scene` provides scene-level queries with body exclusion
and geom group filtering.

These examples demonstrate ray queries from the ground up: one shape at a time,
then terrain, then full-scene queries.

---

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | All subsystems | Headless: 20+ checks against analytical ray solutions |
| [basic-shapes](basic-shapes/) | `raycast_shape` + `RaycastHit` | 6 primitives, each struck by a ray — hit dot + normal arrow |
| [heightfield](heightfield/) | Ray marching on terrain | Rays rain down on sinusoidal terrain — hit dots track the surface |
| [scene-query](scene-query/) | `raycast_scene` + LIDAR fan | Fan of rays from a fixed eye — distance profile printed, nearest hits marked |

---

## 1. `basic-shapes/` — Shape-Level Ray Intersection

**One concept:** Call `raycast_shape` against a single primitive and inspect
the `RaycastHit` return (distance, point, normal).

### Scene

Six shapes arranged in a row along X, each at z=0: sphere, box, capsule,
cylinder, ellipsoid, plane (ground). All shapes are at rest (no joints, no
gravity). A ray fires from above (+Z) straight down (-Z) at each shape.

### MJCF sketch

```xml
<mujoco model="basic-shapes">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <geom name="sphere"   type="sphere"   pos="-4 0 0" size="0.5"/>
    <geom name="box"      type="box"      pos="-2 0 0" size="0.5 0.5 0.5"/>
    <geom name="capsule"  type="capsule"  pos=" 0 0 0" size="0.3 0.5"/>
    <geom name="cylinder" type="cylinder" pos=" 2 0 0" size="0.3 0.5"/>
    <geom name="ellipsoid" type="ellipsoid" pos=" 4 0 0" size="0.6 0.4 0.3"/>
    <geom name="ground"   type="plane"    size="6 2 0.01"/>
  </worldbody>
</mujoco>
```

### Visualization

Each step, for each shape:
1. Cast ray from `(shape_x, 0, 3)` in direction `(0, 0, -1)`.
2. Use `raycast_shape` (construct the `Shape` from geom data, or use
   `raycast_scene` with body exclude to isolate the target).
3. Draw the ray as a gizmo line (faint cyan).
4. Draw a small sphere at the hit point (bright green).
5. Draw a short arrow from the hit point along the surface normal (yellow).

### HUD

```
Sphere    dist=2.500  normal=(0, 0, 1)
Box       dist=2.500  normal=(0, 0, 1)
Capsule   dist=2.200  normal=(0, 0, 1)
Cylinder  dist=2.500  normal=(0, 0, 1)
Ellipsoid dist=2.700  normal≈(0, 0, 1)
Ground    dist=3.000  normal=(0, 0, 1)
```

### Validation (5 checks)

1. Each of 6 shapes returns `Some(hit)` — no misses.
2. Hit distances match analytical expectations within 1e-3.
3. Hit point lies on the shape surface (distance from shape center ≈ radius for
   sphere; z ≈ half_extent for box top face).
4. Normal is unit-length (‖normal‖ ∈ [0.999, 1.001]).
5. Normal points toward ray origin (dot(normal, ray_dir) < 0).

---

## 2. `heightfield/` — Ray Marching on Terrain

**One concept:** Ray intersection against a `HeightField` shape, which uses
ray marching (not analytic intersection).

### Scene

A heightfield with sinusoidal terrain: `z = 0.3 * sin(2πx/4) * cos(2πy/4)`.
A grid of rays fires downward from z=3, spaced evenly over the terrain. Hit
points form a dot cloud that traces the terrain surface.

### MJCF sketch

```xml
<mujoco model="heightfield">
  <option gravity="0 0 0" timestep="0.002"/>
  <asset>
    <hfield name="terrain" nrow="64" ncol="64" size="4 4 0.5 0"/>
  </asset>
  <worldbody>
    <geom name="terrain" type="hfield" hfield="terrain"/>
  </worldbody>
</mujoco>
```

The heightfield data is populated programmatically after model load by replacing
the `Arc<HeightFieldData>` entry:
```rust
use std::sync::Arc;
use cf_geometry::HeightFieldData;

// MuJoCo size = [4, 4, ...] → full extent = 8×8, 64 samples → cell_size = 8/63
let cell_size = 8.0 / 63.0;
let hfield = HeightFieldData::from_fn(64, 64, cell_size, |x, y| {
    0.3 * (std::f64::consts::TAU * x / 4.0).sin()
        * (std::f64::consts::TAU * y / 4.0).cos()
});
model.hfield_data[0] = Arc::new(hfield);
```

### Visualization

- Heightfield rendered as terrain mesh.
- An 8×8 grid of downward rays from z=3 shown as thin lines.
- Hit points rendered as small green spheres on the terrain surface.
- Normals drawn as short yellow arrows.

### HUD

```
Rays cast: 64
Hits:      64 (100%)
Max z-error vs terrain(x,y): 0.0012
```

### Validation (3 checks)

1. All 64 rays hit (100% hit rate).
2. For each hit, `|hit.point.z - terrain(hit.point.x, hit.point.y)| < 0.01`.
3. Surface normals are consistent with terrain gradient (dot with analytical
   gradient normal > 0.95).

---

## 3. `scene-query/` — Scene-Level Ray Fan (LIDAR)

**One concept:** Use `raycast_scene` to cast a fan of rays from one point and
find the nearest hit for each ray across all geoms in the scene.

### Scene

3-4 objects at various positions (sphere, box, capsule). A "sensor eye" site
at a fixed position casts a planar fan of 36 rays (10° apart, 360° sweep in
the XZ plane) using `raycast_scene`.

### MJCF sketch

```xml
<mujoco model="scene-query">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <geom name="sphere"  type="sphere"   pos="3 0 0"  size="0.5"/>
    <geom name="box"     type="box"      pos="0 0 3"  size="0.8 0.8 0.8"/>
    <geom name="capsule" type="capsule"  pos="-2 0 1" size="0.3 0.7"/>
    <geom name="ground"  type="plane"    pos="0 0 -1" size="5 5 0.01"/>
    <body name="eye" pos="0 0 0">
      <site name="lidar" pos="0 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

### Visualization

- All shapes rendered normally.
- 36 ray lines emanating from the eye, each colored by hit/miss:
  - Hit: cyan line from eye to hit point, green dot at hit.
  - Miss: faint grey line extending to max_distance.
- A distance profile ring: at each angle, a small marker at distance
  proportional to hit distance (closer = nearer to center), forming a polar
  distance plot around the eye.

### HUD

```
Rays: 36   Hits: 28   Misses: 8
Nearest: geom "sphere" at 2.50 m (angle 0°)
Farthest hit: geom "ground" at 4.12 m (angle 250°)
```

### Validation (4 checks)

1. Ray at angle 0° (toward sphere at x=3) hits with distance ≈ 2.5 (3.0 - 0.5).
2. `SceneRayHit.geom_id` matches the expected geom for known angles.
3. Rays aimed into open space return `None`.
4. Body-exclude filter: when excluding the sphere's body, the ray at 0° either
   misses or hits a farther object.

---

## 4. `stress-test/` — Headless Validation

**One concept:** Exhaustive numerical validation of ray intersection against
analytical solutions. No visual — headless only.

### Checks (20)

**Per-shape analytical intersection (10):**

1. **Sphere hit distance** — `center_dist - radius`. Ray from (0,0,0) in +Z at
   sphere center (0,0,5) r=1 → distance = 4.0.
2. **Sphere miss** — Ray in +X at sphere at (0,0,5) → `None`.
3. **Plane hit distance** — `(plane_point - origin) · normal / (dir · normal)`.
   Plane at z=3 normal +Z, ray from origin in +Z → distance = 3.0.
4. **Plane parallel miss** — Ray in +X at plane with normal +Z → `None`.
5. **Box slab intersection** — Box half_extents (1,1,1) at (0,0,5), ray +Z →
   distance = 4.0. Normal = (0,0,-1).
6. **Capsule sphere-swept-line** — Capsule half_length=1, radius=0.3 at (0,0,5),
   ray +Z → hits bottom cap sphere at 5-1-0.3 = 3.7.
7. **Cylinder flat cap** — Cylinder half_length=1, radius=0.5 at (0,0,5), ray +Z
   → hits bottom cap at z=4, distance=4.0.
8. **Ellipsoid scaled sphere** — Ellipsoid radii (1,1,2) at (0,0,5), ray +Z →
   hits at z=3, distance=3.0.
9. **Heightfield ray march** — Flat heightfield at z=0, ray from (0,0,3) in -Z →
   distance=3.0. Hit z ≈ 0.
10. **Convex mesh face test** — Tetrahedron hull, ray aimed at face center → hits.
    Ray aimed outside hull but inside bounding sphere → misses.

**Surface normal properties (3):**

11. **Normal is unit-length** — For all hits above: ‖normal‖ ∈ [0.999, 1.001].
12. **Normal faces ray** — For all hits above: dot(normal, ray_dir) < 0.
13. **Normal perpendicular to surface** — Sphere hit: normal = normalize(hit_point
    - center). Plane hit: normal = ±plane_normal. Box face hit: normal aligned
    with face axis.

**Scene-level queries (4):**

14. **Nearest hit** — Two spheres along +Z (r=1 at z=5 and z=10). Ray from origin
    in +Z → hits nearer sphere, distance=4.0, geom_id=0.
15. **Body exclude** — Exclude body of nearer sphere → hits farther sphere,
    distance=9.0, geom_id=1.
16. **Geom group filter** — Nearer sphere in group 1, farther in group 0. Filter
    to group 0 only → hits farther sphere.
17. **Scene miss** — Ray in -Z (away from all geoms) → `None`.

**Edge cases (3):**

18. **Ray origin inside sphere** — Origin at sphere center, ray +Z → returns
    exit point (distance = radius).
19. **Max distance clamp** — Sphere at z=10, max_distance=5 → `None` (hit at
    z=9 is beyond cutoff).
20. **Zero max distance** — Any shape, max_distance=0.0 → `None`.

---

## Key Ideas

- **`raycast_shape`** tests a single shape. You provide the shape, its pose, and
  the ray. Used for targeted queries against known geometry.
- **`raycast_scene`** tests all geoms in the model. You provide the ray and
  optional filters. Returns the closest hit with `geom_id`. Used for LIDAR,
  rangefinders, visibility.
- **`RaycastHit`** returns: distance along ray, hit point in world coordinates,
  and surface normal pointing outward from the surface.
- **Ray marching** is used for heightfields and SDFs. Analytic formulas are used
  for all convex primitives.
- **Geom group filter** and **body exclude** in `raycast_scene` match MuJoCo's
  `mj_ray` semantics.

## Color Convention

- Ray line: faint cyan (`rgba(0.2, 0.8, 1.0, 0.4)`)
- Hit point: bright green sphere
- Normal arrow: yellow
- Miss ray: faint grey
- Static ground: dark grey
