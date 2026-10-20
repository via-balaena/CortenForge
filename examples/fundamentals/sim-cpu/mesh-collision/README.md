# Mesh Collision

Seven visual examples + one headless stress test covering a mesh against a
plane and against each primitive and mesh (`sim-core/src/collision/narrow.rs`). Each
visual example teaches exactly one concept about mesh collision.

## Dispatch Table Coverage

| # | Example | Pair | Narrow-phase | MULTICCD on? |
|---|---------|------|--------------|--------------|
| 1 | [mesh-on-plane](mesh-on-plane/) | mesh-plane | `collide_mesh_plane()` | No |
| 2 | [mesh-sphere](mesh-sphere/) | mesh-sphere | `mjc_convex()` | No |
| 3 | [mesh-box](mesh-box/) | mesh-box | `mjc_convex()` | Yes |
| 4 | [mesh-capsule](mesh-capsule/) | mesh-capsule | `mjc_convex()` | No |
| 5 | [mesh-cylinder](mesh-cylinder/) | mesh-cylinder | `mjc_convex()` | Yes |
| 6 | [mesh-ellipsoid](mesh-ellipsoid/) | mesh-ellipsoid | `mjc_convex()` | No |
| 7 | [mesh-on-mesh](mesh-on-mesh/) | mesh-mesh | `mjc_convex()` | Yes |
| 8 | [stress-test](stress-test/) | all 7 | headless, 21 checks | — |

## Two Dispatch Paths

**Mesh against a plane**: `collide_with_mesh()` (`mesh_collide.rs`) calls
sim-core's own `collide_mesh_plane()`, which scans the mesh's vertices for
those below the plane or within the margin above it: up to three contacts.

**Mesh against a primitive or a mesh** (sphere, capsule, box, cylinder, ellipsoid,
mesh): MuJoCo's native convex collision, `mjc_convex()` in `mj_ccd.rs` (GJK,
EPA and MuJoCo's multi-contact). The mesh collides as the convex hull of its
vertices; a mesh with no hull computed is searched vertex by vertex.

MULTICCD (`<flag multiccd="enable"/>`) adds contacts:

- A box or a mesh against a mesh, neither with a margin: one pass clips the
  touching faces, up to 4 contacts.
- A capsule or a cylinder against a mesh (and a box or a mesh against a mesh
  with a margin): four more passes with the geoms turned by ±1e-3 rad, up to
  5 contacts.
- A sphere or an ellipsoid against a mesh: no extra pass, one contact.

## What the checks pin

- **Cylinder** — rests on its flat cap at z = half_length (stress-test check
  10), where a capsule of the same radius and half-length would rest a radius
  higher.
- **Ellipsoid** — an oblate disc with rz=0.05 rests at z=0.05 (check 13); a
  sphere of its largest radius would rest at z=0.3 (check 14 fails z ≥ 0.1).

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
