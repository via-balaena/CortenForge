# Mesh Collision Examples Spec

7 visual examples + 1 headless stress test covering every mesh collision pair
in the dispatch table (`sim-core/src/collision/mesh_collide.rs`). Each visual
example teaches exactly one concept about mesh collision. The stress test
validates all 7 pairs headlessly.

## Dispatch Table Coverage

| # | Example | Pair | Narrow-phase | Fixed this branch? |
|---|---------|------|--------------|--------------------|
| 1 | mesh-on-plane | mesh-plane | `collide_mesh_plane()` | No |
| 2 | mesh-sphere | mesh-sphere | `mesh_sphere_contact()` BVH | No |
| 3 | mesh-box | mesh-box | `mesh_box_contact()` BVH | No |
| 4 | mesh-capsule | mesh-capsule | `mesh_capsule_contact()` BVH | No |
| 5 | mesh-cylinder | mesh-cylinder | `gjk_epa_shape_pair()` | Yes |
| 6 | mesh-ellipsoid | mesh-ellipsoid | `gjk_epa_shape_pair()` | Yes |
| 7 | mesh-on-mesh | mesh-mesh | `gjk_epa_shape_pair()` (hull-hull) | Refactored |
| 8 | stress-test | all 7 | headless | — |

---

## Shared Conventions

### Mesh Ground Plane

Examples 2–6 use the same mesh ground: a thick box-mesh slab (2x2x2 m)
positioned at `pos="0 0 -1"` so the top face is at z=0. Inline
vertex/face data — 8 vertices, 12 triangles (space-separated, triangles only —
the MJCF parser splits on whitespace and requires length divisible by 3):

```xml
<mesh name="ground"
      vertex="-1 -1 -1  1 -1 -1  1 1 -1  -1 1 -1
              -1 -1  1  1 -1  1  1 1  1  -1 1  1"
      face="0 1 2  0 2 3  4 6 5  4 7 6
            0 4 5  0 5 1  2 6 7  2 7 3
            0 3 7  0 7 4  1 5 6  1 6 2"/>
```

This is a convex mesh (8 vertices, non-coplanar) so the engine builds a convex
hull automatically at load time via `compute_convex_hull()`. The hull is
required for GJK/EPA pairs (cylinder, ellipsoid, mesh-mesh). BVH pairs
(sphere, capsule, box) operate on triangles directly and don't need the hull.

**Hull constraint:** `convex_hull()` returns `None` for fewer than 4
non-coplanar points. All mesh assets in these examples must have >= 4
non-coplanar vertices.

### Physics Settings

All visual examples:
- `gravity="0 0 -9.81"`, `timestep="0.001"`
- Integrator: `RK4` (stable for settling)
- `density="1000"` for dynamic bodies
- `condim="3"` (frictionless normal + tangent) unless friction is the point

### Bevy Boilerplate

Each visual example follows the standard pattern:
- `PhysicsModel` / `PhysicsData` resources
- `spawn_model_geoms()` with `GeomMaterialOverride` array (tuple syntax `("name", handle)`)
- `spawn_example_camera(commands, target, distance, azimuth, elevation)`
- `ValidationHarness::new().report_at().print_every().display()`
- `OrbitCameraPlugin` for camera control
- `PhysicsHud` with `clear()`, `section()`, `scalar()` readouts

**Mesh rendering:** `spawn_model_geoms()` handles `type="mesh"` geoms
natively via `triangle_mesh_from_indexed()` — no special code needed.

### Naming

Package names: `example-mesh-collision-{name}` (e.g., `example-mesh-collision-plane`).
Follows existing convention: `example-{category}-{name}` (cf. `example-sensor-clock`,
`example-hinge-joint-pendulum`).

### Workspace Registration

Each example must be added as an explicit member in the root `Cargo.toml` —
workspace members are listed individually, not by glob.

---

## Example 1: `mesh-on-plane/`

**Concept:** Simplest mesh collision — a convex mesh resting on a ground plane.

**What you see:** A metallic tetrahedron sitting flat on its base face on an
infinite ground plane. The tetrahedron is defined by 4 vertices and 4 triangular
faces inline in MJCF. It drops from a small height, settles, and stays still.

### MJCF Sketch

```xml
<mujoco model="mesh-on-plane">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="tetra"
          vertex="0 0 0  1 0 0  0.5 0.866 0  0.5 0.289 0.816"
          face="0 2 1  0 1 3  1 2 3  0 3 2"/>
  </asset>
  <worldbody>
    <geom type="plane" size="2 2 0.01"/>
    <body name="tetra" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="tetra" density="1000"/>
    </body>
  </worldbody>
</mujoco>
```

The tetrahedron has a flat base triangle (vertices 0,1,2 at z=0 in local
frame). After settling, the body center should rest at the centroid height of
that base face above the plane.

### Validation (t=5s)

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = centroid height of base face (compute from vertices) | < 3mm |
| Settled | \|vz\| < 0.01 m/s | — |
| Contact force | avg force / weight in [0.95, 1.05] | 5% |
| Contacts exist | ncon >= 1 | — |

### Camera

`spawn_example_camera` — look at origin, distance ~2.0, slight elevation (~20 deg)
to see the tetrahedron sitting on the plane.

### Materials

- Tetrahedron: `MetalPreset::Anodized(COPPER_COLOR)` for contrast against grey plane
- Ground plane: rendered by Bevy's default plane geom (grey)

Available `MetalPreset` variants: `PolishedSteel`, `BrushedMetal`, `CastIron`,
`SpringWire`, `Anodized(Color)`. Use `Anodized` with a copper/warm color
(`Color::srgb(0.72, 0.45, 0.20)`) wherever the spec says "copper".

---

## Example 2: `mesh-sphere/`

**Concept:** Mesh-sphere collision with BVH-accelerated `mesh_sphere_contact()`.

**What you see:** A sphere resting in a shallow mesh bowl. The bowl is a
concave mesh (inverted truncated cone or similar) defined inline. The sphere
drops in, bounces slightly, then settles at the bottom. Demonstrates that BVH
triangle queries find the correct contact triangle even on non-trivial mesh
geometry.

### MJCF Sketch

Use the mesh ground slab (flat surface) for simplicity. A sphere drops onto
the mesh surface.

```xml
<asset>
  <mesh name="ground" vertex="..." face="..."/>  <!-- shared 2x2x2 slab -->
</asset>
<worldbody>
  <geom type="mesh" mesh="ground" pos="0 0 -1"/>
  <body name="ball" pos="0 0 0.3">
    <joint type="free"/>
    <geom type="sphere" size="0.1" density="1000"/>
  </body>
</worldbody>
```

### Validation (t=5s)

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = radius = 0.1 | < 3mm |
| Settled | \|vz\| < 0.01 m/s | — |
| Contact force / weight | in [0.95, 1.05] | 5% |

### Camera

Side view — distance ~1.5, elevation ~15 deg. Sphere visible against mesh surface.

### Materials

- Sphere: `MetalPreset::Anodized(COPPER_COLOR)`
- Mesh ground: `MetalPreset::BrushedMetal`

---

## Example 3: `mesh-box/`

**Concept:** Mesh-box collision via BVH-accelerated `mesh_box_contact()`.

**What you see:** A box (cuboid) dropped onto the mesh ground slab. It lands
on one face and rests flat. The box is an MJCF primitive (`type="box"`), the
ground is the mesh slab.

### MJCF Sketch

```xml
<asset>
  <mesh name="ground" vertex="..." face="..."/>
</asset>
<worldbody>
  <geom type="mesh" mesh="ground" pos="0 0 -1"/>
  <body name="box" pos="0 0 0.3">
    <joint type="free"/>
    <geom type="box" size="0.15 0.1 0.05" density="1000"/>
  </body>
</worldbody>
```

Box half-sizes: 0.15 x 0.1 x 0.05. Dropped from z=0.3. Should rest with
center at z = half_z = 0.05.

### Validation (t=5s)

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = 0.05 | < 3mm |
| Settled | \|vz\| < 0.01 m/s | — |
| Contact force / weight | in [0.95, 1.05] | 5% |

### Camera

Slight angle — distance ~1.5, azimuth ~30 deg, elevation ~20 deg. Shows the
box's rectangular profile against the mesh surface.

### Materials

- Box: `MetalPreset::PolishedSteel`
- Mesh ground: `MetalPreset::BrushedMetal`

---

## Example 4: `mesh-capsule/`

**Concept:** Mesh-capsule collision via BVH-accelerated `mesh_capsule_contact()`.

**What you see:** A capsule lying on its side on the mesh ground slab. The
curved barrel surface rests against the mesh triangles — visually distinct
from the upright cylinder in example 5. The capsule drops from a small height,
settles, and stays still.

### MJCF Sketch

```xml
<asset>
  <mesh name="ground" vertex="..." face="..."/>
</asset>
<worldbody>
  <geom type="mesh" mesh="ground" pos="0 0 -1"/>
  <body name="capsule" pos="0 0 0.2" quat="0.707 0.707 0 0">
    <joint type="free"/>
    <geom type="capsule" size="0.1 0.2" density="1000"/>
    <!-- size = radius half_length: total length = 2*(0.2+0.1) = 0.6 -->
    <!-- quat rotates 90° around X so capsule axis lies along Y -->
  </body>
</worldbody>
```

Capsule: radius=0.1, half_length=0.2. Rotated 90° so it lies on its side
(axis along Y). Rest height = radius = 0.1. Start at z=0.2 (slightly above
rest) to let it settle.

### Validation (t=5s)

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = radius = 0.1 | < 3mm |
| Settled | \|vz\| < 0.01 m/s | — |
| Contact force / weight | in [0.95, 1.05] | 5% |

### Camera

Side view — distance ~1.5, elevation ~10 deg. Capsule visible lying on
mesh surface, barrel curve clearly visible.

### Materials

- Capsule: `MetalPreset::Anodized(COPPER_COLOR)`
- Mesh ground: `MetalPreset::BrushedMetal`

---

## Example 5: `mesh-cylinder/`

**Concept:** Mesh-cylinder collision via GJK/EPA. This is the fix we just
landed — cylinders were previously approximated as capsules, giving wrong
contact geometry. With MULTICCD enabled, the flat cap produces 8 rim contact
points for rotational stability.

**What you see:** An upright cylinder on the mesh ground slab. It drops,
settles, and stays perfectly upright — no wobble, no drift. The flat bottom
cap makes full contact with the surface. This is the "money shot" for the
GJK/EPA cylinder fix.

### MJCF Sketch

```xml
<mujoco model="mesh-cylinder">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag multiccd="enable"/>
  </option>
  <asset>
    <mesh name="ground" vertex="..." face="..."/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="cylinder" pos="0 0 0.2">
      <joint type="free"/>
      <geom type="cylinder" size="0.1 0.2" density="1000"/>
      <!-- size = radius half_length -->
    </body>
  </worldbody>
</mujoco>
```

Cylinder: radius=0.1, half_length=0.2. Start at rest height z=0.2
(= half_length). MULTICCD enabled for flat-cap multi-contact.

### Validation (t=5s)

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = half_length = 0.2 | < 3mm |
| Settled | \|vz\| < 0.01 m/s | — |
| Contact force / weight | in [0.95, 1.05] | 5% |
| MULTICCD contacts | ncon >= 2 at steady state | — |
| Rotational stability | angular velocity magnitude < 0.1 rad/s | — |

### Camera

Slightly elevated view — distance ~1.5, elevation ~25 deg. Show the cylinder's
flat top face to emphasize it's a true cylinder (not a capsule).

### Materials

- Cylinder: `MetalPreset::PolishedSteel`
- Mesh ground: `MetalPreset::BrushedMetal`

### README Emphasis

The README should explain what changed: before this branch, mesh-cylinder used
a capsule approximation (rounded caps). Now it uses exact GJK/EPA, and MULTICCD
provides 8 rim contact points on the flat cap for stable flat-on-flat contact.

---

## Example 6: `mesh-ellipsoid/`

**Concept:** Mesh-ellipsoid collision via GJK/EPA. Previously approximated as
a sphere (using max radius), which gave wildly wrong rest heights for oblate
ellipsoids. Now uses exact GJK/EPA support functions.

**What you see:** An oblate ellipsoid (disc-shaped, like a puck) sitting on
the mesh ground slab. It rests at the correct height z = rz = 0.05, NOT at
z = 0.3 (which the old sphere approximation would produce). The flat
disc shape is visually obvious.

### MJCF Sketch

```xml
<mujoco model="mesh-ellipsoid">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="ground" vertex="..." face="..."/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="ellipsoid" pos="0 0 0.15">
      <joint type="free"/>
      <geom type="ellipsoid" size="0.3 0.3 0.05" density="1000"/>
      <!-- size = rx ry rz: oblate disc, rz << rx=ry -->
    </body>
  </worldbody>
</mujoco>
```

Ellipsoid radii: [0.3, 0.3, 0.05]. Start at z=0.15 (above rest height).
Should settle at z = rz = 0.05.

### Validation (t=5s)

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = rz = 0.05 | < 3mm |
| Not sphere approx | z < 0.1 (sphere approx would give ~0.3) | — |
| Settled | \|vz\| < 0.01 m/s | — |
| Contact force / weight | in [0.95, 1.05] | 5% |

### Camera

Top-down angled view — distance ~1.5, elevation ~35 deg. Show the disc
shape clearly. The oblate profile is the visual proof that this isn't a sphere.

### Materials

- Ellipsoid: `MetalPreset::Anodized(COPPER_COLOR)`
- Mesh ground: `MetalPreset::BrushedMetal`

### README Emphasis

The README should contrast old vs new: sphere approximation would place the
center at z=0.3 (max radius). GJK/EPA places it at z=0.05 (correct rz). A
6x error in rest height — that's the bug this branch fixed.

---

## Example 7: `mesh-on-mesh/`

**Concept:** Two convex meshes stacking via hull-hull GJK/EPA. Both objects
are `type="mesh"` — no primitives involved.

**What you see:** A mesh wedge (triangular prism) sitting on a mesh platform
(box slab). Both are defined inline with vertex/face data. The wedge drops
onto the platform and rests stably on its flat base face.

### MJCF Sketch

```xml
<mujoco model="mesh-on-mesh">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <asset>
    <!-- Platform: box mesh, 1x1x0.1 -->
    <mesh name="platform"
          vertex="-0.5 -0.5 -0.05  0.5 -0.5 -0.05  0.5 0.5 -0.05  -0.5 0.5 -0.05
                  -0.5 -0.5  0.05  0.5 -0.5  0.05  0.5 0.5  0.05  -0.5 0.5  0.05"
          face="0 1 2  0 2 3  4 6 5  4 7 6
                0 4 5  0 5 1  2 6 7  2 7 3
                0 3 7  0 7 4  1 5 6  1 6 2"/>
    <!-- Wedge: triangular prism, 6 vertices, 8 faces -->
    <mesh name="wedge"
          vertex="0 -0.15 0  0.3 -0.15 0  0.15 -0.15 0.25
                  0  0.15 0  0.3  0.15 0  0.15  0.15 0.25"
          face="0 1 2  3 5 4
                0 3 4  0 4 1  1 4 5  1 5 2  2 5 3  2 3 0"/>
  </asset>
  <worldbody>
    <geom type="plane" size="2 2 0.01"/>
    <geom type="mesh" mesh="platform" pos="0 0 0.05"/>
    <body name="wedge" pos="0 0 0.35">
      <joint type="free"/>
      <geom type="mesh" mesh="wedge" density="1000"/>
    </body>
  </worldbody>
</mujoco>
```

The platform is a static mesh (no joint) sitting on the plane. The wedge drops
onto the platform. Rest height = platform top (0.1) + wedge base-to-centroid
offset.

### Validation (t=5s)

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Settled | \|vz\| < 0.01 m/s | — |
| Above platform | z > 0.1 (platform top) | — |
| No interpenetration | z > platform_top + wedge_half_height - 0.005 | 5mm |
| Contact force / weight | in [0.90, 1.10] | 10% (hull approx) |

### Camera

Perspective view — distance ~2.0, azimuth ~45 deg, elevation ~25 deg. Show
both meshes and their stacking relationship.

### Materials

- Platform: `MetalPreset::BrushedMetal`
- Wedge: `MetalPreset::Anodized(COPPER_COLOR)`
- Ground plane: default grey

---

## Example 8: `stress-test/`

**Concept:** Headless validation of all 7 mesh collision pairs plus edge cases.
No Bevy, no window. Exits with code 0 on success, 1 on failure.

### Checks

| # | Group | Check | Tolerance |
|---|-------|-------|-----------|
| 1 | Mesh-Plane | Tetrahedron rest height matches centroid | < 3mm |
| 2 | Mesh-Plane | Contact force / weight ~ 1.0 | 5% |
| 3 | Mesh-Plane | ncon >= 1 at steady state | — |
| 4 | Mesh-Sphere | Sphere on mesh rests at z = radius | < 3mm |
| 5 | Mesh-Sphere | Contact force / weight ~ 1.0 | 5% |
| 6 | Mesh-Box | Box on mesh rests at z = half_z | < 3mm |
| 7 | Mesh-Box | Contact force / weight ~ 1.0 | 5% |
| 8 | Mesh-Capsule | Sideways capsule rests at z = radius | < 3mm |
| 9 | Mesh-Capsule | Contact force / weight ~ 1.0 | 5% |
| 10 | Mesh-Cylinder | Cylinder rests at z = half_length (GJK/EPA) | < 3mm |
| 11 | Mesh-Cylinder | Contact force / weight ~ 1.0 | 5% |
| 12 | Mesh-Cylinder | MULTICCD produces >= 2 contacts | — |
| 13 | Mesh-Ellipsoid | Oblate ellipsoid rests at z = rz = 0.05 | < 3mm |
| 14 | Mesh-Ellipsoid | z < 0.1 (rejects sphere approximation) | — |
| 15 | Mesh-Ellipsoid | Contact force / weight ~ 1.0 | 5% |
| 16 | Mesh-Mesh | Wedge on platform settled (vz < 0.01) | — |
| 17 | Mesh-Mesh | Wedge above platform surface | — |
| 18 | Mesh-Mesh | Contact force / weight ~ 1.0 | 10% |
| 19 | Edge: Separated | Two meshes far apart: ncon = 0 | exact |
| 20 | Edge: Single tri | Mesh with 1 triangle vs sphere: no crash, contact produced (BVH path — hull not required) | — |
| 21 | Edge: Swapped order | Primitive-mesh (geom1=sphere, geom2=mesh): same result as mesh-sphere | < 1mm |

### Dependencies

Stress test depends on `sim-core` and `sim-mjcf` only (no Bevy).

### Cargo.toml

```toml
[package]
name = "example-mesh-collision-stress-test"
# ... workspace fields ...
publish = false

[dependencies]
sim-core = { workspace = true }
sim-mjcf = { workspace = true }
```

### Implementation Note

Reuse the `settle_and_measure()` pattern from the integration tests
(`sim/L0/tests/integration/mesh_cylinder_ellipsoid.rs`). Each check is a
standalone MJCF model loaded, simulated for N steps, then validated.

---

## Session Plan

### Session 1: Foundation (examples 1-3)

1. **mesh-on-plane** — simplest case, introduces `type="mesh"` with inline data
2. **mesh-sphere** — first BVH-accelerated pair
3. **mesh-box** — second BVH pair, validates box-mesh contact

Review checkpoint: run all three, verify visuals + validation passes.

### Session 2: BVH + GJK/EPA (examples 4-6)

4. **mesh-capsule** — third BVH pair
5. **mesh-cylinder** — first GJK/EPA pair (branch highlight)
6. **mesh-ellipsoid** — second GJK/EPA pair (branch highlight)

Review checkpoint: run all three. Cylinder and ellipsoid are the critical ones.

### Session 3: Mesh-Mesh + Stress Test (examples 7-8)

7. **mesh-on-mesh** — hull-hull GJK/EPA
8. **stress-test** — headless validation of all 7 pairs + edge cases

Review checkpoint: stress test must exit 0. Final visual review of mesh-on-mesh.

---

## Resolved Questions

1. **Mesh ground vs plane ground:** Confirmed correct. Example 1 uses
   `type="plane"` ground (exercises the Plane-vs-Mesh path in
   `collide_with_mesh()` with normal negation on line 305). Examples 2-6
   use the mesh slab ground (exercises the Mesh-vs-Primitive path).
   Both dispatch paths are in `collide_with_mesh()` — the mesh check at
   `narrow.rs:87` fires before the plane check at line 92, so all
   mesh-plane pairs route through mesh dispatch, never plane dispatch.

2. **Convex hull minimum:** `convex_hull()` returns `None` for < 4
   non-coplanar points. The tetrahedron (4 vertices) is the minimum shape
   that produces a hull. Single-triangle meshes work only with BVH-path
   pairs (sphere, capsule, box). The stress test edge case (#20) is
   correctly scoped to mesh-sphere (BVH path).

3. **Tetrahedron for example 1:** Kept. A box-mesh is visually
   indistinguishable from `type="box"` — defeats the museum-exhibit purpose.
   The tetrahedron is the minimum convex hull (4 verts, 4 faces), unmistakably
   "hand-defined mesh." It earns example 1.

4. **Capsule sideways:** Changed to lying on side (axis along Y, rest height
   = radius = 0.1). Avoids visual repetition with the upright cylinder in
   example 5, shows the curved barrel contact, and is the capsule's natural
   resting pose.
