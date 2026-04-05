# Mesh-on-Plane — Simplest Mesh Collision

A convex tetrahedron defined by 4 vertices and 4 triangular faces, dropped onto
an infinite ground plane. The simplest possible mesh collision: one hand-defined
mesh against a `type="plane"` surface.

## What you see

- **Copper tetrahedron** sitting flat on its base face on a grey ground plane
- The tetrahedron drops from z=0.5, bounces slightly, then settles
- It rests motionless with its flat base triangle flush to the plane

## Physics

The tetrahedron is a regular tetrahedron (edge length 1), centered at its
volumetric centroid so that body position = center of mass. The base face
(vertices 0,1,2) sits at z = -0.204 in the local frame.

The narrow-phase path is `collide_mesh_plane()` — the mesh dispatcher
recognizes the plane geom and iterates over mesh triangles to find contact
points below the plane surface.

| Parameter | Value |
|-----------|-------|
| Mesh | 4 vertices, 4 faces (regular tetrahedron) |
| Centroid-to-base | sqrt(6)/12 = 0.204 m |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| Narrow-phase | `collide_mesh_plane()` |

## Validation

Four automated checks at t=5s:

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = 0.204 (centroid-to-base) | < 3mm |
| Settled | \|vz\| < 0.01 m/s | -- |
| Contact force / weight | 1.0 | 5% |
| Contacts exist | ncon >= 1 | -- |

## Run

```
cargo run -p example-mesh-collision-plane --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
