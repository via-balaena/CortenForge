# Mesh Collision

Seven visual examples + one headless stress test covering every mesh collision
pair in the dispatch table (`sim-core/src/collision/mesh_collide.rs`). Each
visual example teaches exactly one concept about mesh collision.

## Dispatch Table Coverage

| # | Example | Pair | Narrow-phase | Needs MULTICCD? |
|---|---------|------|--------------|-----------------|
| 1 | [mesh-on-plane](mesh-on-plane/) | mesh-plane | `collide_mesh_plane()` | No |
| 2 | [mesh-sphere](mesh-sphere/) | mesh-sphere | `mesh_sphere_contact()` BVH | No |
| 3 | [mesh-box](mesh-box/) | mesh-box | `gjk_epa_shape_pair()` | Yes |
| 4 | [mesh-capsule](mesh-capsule/) | mesh-capsule | `mesh_capsule_contact()` BVH | No |
| 5 | [mesh-cylinder](mesh-cylinder/) | mesh-cylinder | `gjk_epa_shape_pair()` | Yes |
| 6 | [mesh-ellipsoid](mesh-ellipsoid/) | mesh-ellipsoid | `gjk_epa_shape_pair()` | No |
| 7 | [mesh-on-mesh](mesh-on-mesh/) | mesh-mesh | `gjk_epa_shape_pair()` hull-hull | Yes |
| 8 | [stress-test](stress-test/) | all 7 | headless, 21 checks | — |

## Two Dispatch Paths

**BVH path** (sphere, capsule): The BVH finds candidate triangles near the
primitive, then each triangle is tested individually. Works on any mesh
topology — no convex hull required.

**GJK/EPA path** (box, cylinder, ellipsoid, mesh-mesh): Uses the mesh's convex
hull (computed at load time) as a single shape. Requires >= 4 non-coplanar
vertices. MULTICCD generates additional contact points for flat-on-flat
stability.

## Branch Highlights

Two GJK/EPA fixes landed on this branch:

- **Cylinder** — previously approximated as a capsule (rounded caps). Now uses
  exact GJK/EPA support functions. MULTICCD provides 8 rim contacts for flat-cap
  stability.
- **Ellipsoid** — previously approximated as a sphere (max radius). An oblate
  disc with rz=0.05 would rest at z=0.3 (6x error). Now rests at the correct
  z=0.05.
- **Box** — `triangle_box_contact()` was algorithmically incomplete (only tested
  triangle vertices inside box). Replaced with GJK/EPA on convex hull.

## Running

Visual examples (Bevy window):
```
cargo run -p example-mesh-collision-plane --release
cargo run -p example-mesh-collision-sphere --release
cargo run -p example-mesh-collision-box --release
cargo run -p example-mesh-collision-capsule --release
cargo run -p example-mesh-collision-cylinder --release
cargo run -p example-mesh-collision-ellipsoid --release
cargo run -p example-mesh-collision-mesh --release
```

Headless stress test (exit code 0 = pass):
```
cargo run -p example-mesh-collision-stress-test --release
```
