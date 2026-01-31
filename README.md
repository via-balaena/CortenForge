# CortenForge

> **Industrial-grade foundation, unlimited application.**
> From humanoid robots to vehicles to medical simulation.

[![Quality Gate](https://github.com/cortenforge/forge/actions/workflows/quality-gate.yml/badge.svg)](https://github.com/cortenforge/forge/actions/workflows/quality-gate.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

---

## What is CortenForge?

CortenForge is a Rust-native SDK for building physical things in software. It provides the computational foundation for:

- **Mesh Processing** - 27 crates: load, repair, transform, boolean ops, lattices, scanning
- **Parametric Geometry** - Bezier, B-spline, NURBS curves, arcs, helices
- **3D Routing** - A* pathfinding on voxel grids, multi-objective optimization
- **Physics Simulation** - MuJoCo-aligned rigid body dynamics, constraints, contact models, MJCF/URDF loading
- **Machine Learning** - Model training, inference, and dataset management (Burn)
- **Sensor Fusion** - Hardware-agnostic sensor types, stream synchronization

The same code runs in simulation and on hardware. Our code must be as reliable as the things built with it.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              LAYER 1: Bevy SDK                               │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                           cortenforge                                    ││
│  │  CfMeshPlugin · CfRoutingPlugin · CfSensorPlugin · CfSimPlugin · CfUi  ││
│  └─────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         LAYER 0: Pure Rust Foundation                        │
│                           (Zero Bevy Dependencies)                           │
├──────────────┬──────────────┬──────────────┬──────────────┬─────────────────┤
│   mesh/*     │  geometry/*  │   routing/*  │     sim/*    │ ml/* sensor/* + │
│   27 crates  │              │              │              │                 │
├──────────────┼──────────────┼──────────────┼──────────────┼─────────────────┤
│ mesh-types   │ curve-types  │ route-types  │ sim-types    │ ml-types        │
│ mesh-io      │              │ route-       │ sim-core     │ ml-models       │
│ mesh-repair  │              │  pathfind    │ sim-contact  │ ml-dataset      │
│ mesh-boolean │              │ route-       │ sim-constraint│ ml-training     │
│ mesh-lattice │              │  optimize    │ sim-mjcf     │ sensor-types    │
│ mesh-scan    │              │              │ sim-physics  │ sensor-fusion   │
│ ...          │              │              │ ...          │                 │
├──────────────┴──────────────┴──────────────┴──────────────┴─────────────────┤
│                              crates/cf-spatial                               │
│                  Voxel grids, occupancy maps, raycasting                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Layer 0: Pure Rust

All foundation crates have **zero Bevy dependencies**. They can be used in:
- CLI tools
- Web applications (WASM)
- Servers
- Embedded systems
- Other game engines
- Python bindings

### Layer 1: Bevy SDK

The `cortenforge` crate provides Bevy plugins that wrap Layer 0 functionality. This is the only place Bevy appears.

---

## Quick Start

### Pure Rust (Layer 0)

```rust
use mesh_types::IndexedMesh;
use mesh_io::load_stl;
use mesh_repair::{repair_mesh, RepairParams};

// Load and repair a mesh
let mesh = load_stl("model.stl")?;
let repaired = repair_mesh(&mesh, &RepairParams::default())?;
```

### Bevy SDK (Layer 1)

```rust
use bevy::prelude::*;
use cortenforge::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(CfMeshPlugin)
        .add_plugins(CfRoutingPlugin)
        .run();
}
```

---

## Crate Overview

**52 crates** across 7 domains, all Layer 0 (zero Bevy dependencies) except sim-bevy.

### Mesh Domain (`mesh/`) — 27 crates

| Crate | Description |
|-------|-------------|
| `mesh-types` | Core types: `Vertex`, `IndexedMesh`, `Triangle`, `AABB` |
| `mesh-io` | File I/O: STL, OBJ, PLY, 3MF, STEP |
| `mesh-repair` | Weld vertices, remove degenerates, fill holes |
| `mesh-transform` | RANSAC, PCA, alignment, orientation |
| `mesh-geodesic` | Geodesic distance computation |
| `mesh-sdf` | Signed distance field computation |
| `mesh-offset` | Mesh offset via SDF |
| `mesh-shell` | Shell generation for 3D printing |
| `mesh-zones` | Anatomical zone assignment |
| `mesh-from-curves` | Generate meshes from parametric curves |

### Geometry Domain (`geometry/`)

| Crate | Description |
|-------|-------------|
| `curve-types` | Bezier, B-spline, NURBS, arcs, circles, helices with Frenet frames |

### Routing Domain (`routing/`)

| Crate | Description |
|-------|-------------|
| `route-types` | Paths, routes, constraints, cost weights |
| `route-pathfind` | A* on voxel grids (6/26 connectivity), path smoothing |
| `route-optimize` | Clearance, curvature, shortening, Pareto optimization |

### Simulation Domain (`sim/`) — 14 crates

| Crate | Description |
|-------|-------------|
| `sim-types` | RigidBodyState, Pose, Twist, JointState, MassProperties |
| `sim-simd` | SIMD batch operations (Vec3x4/Vec3x8) |
| `sim-core` | MuJoCo-aligned pipeline: Model/Data, FK, CRBA, RNE, PGS solver |
| `sim-contact` | Compliant contact model, friction, domain randomization |
| `sim-constraint` | Joint types, motors, limits, equality constraints, solvers |
| `sim-sensor` | IMU, F/T, touch, rangefinder, magnetometer |
| `sim-deformable` | XPBD soft bodies (ropes, cloth, volumes) |
| `sim-muscle` | Hill-type muscle model |
| `sim-tendon` | Cable/tendon routing and actuation |
| `sim-mjcf` | MuJoCo XML/MJB format parser |
| `sim-urdf` | URDF parser, kinematic tree validation |
| `sim-physics` | Unified L0 API re-exporting all sim crates |
| `sim-conformance-tests` | MuJoCo conformance and integration tests |
| `sim-bevy` | Layer 1: Bevy visualization and debug gizmos |

See [sim/ARCHITECTURE.md](./sim/ARCHITECTURE.md) for pipeline details.

### ML Domain (`ml/`)

| Crate | Description |
|-------|-------------|
| `ml-types` | Inference types, dataset schemas, labels, metadata |
| `ml-models` | Neural network architectures + checkpoint persistence (Burn) |
| `ml-dataset` | Dataset loading, splitting, augmentation, warehousing |
| `ml-training` | Training loops, loss functions, box matching |

### Sensor Domain (`sensor/`)

| Crate | Description |
|-------|-------------|
| `sensor-types` | Hardware-agnostic sensor data: IMU, camera, LiDAR, F/T, GPS |
| `sensor-fusion` | Stream synchronization, temporal alignment, spatial transforms |

### Foundation (`crates/`)

| Crate | Description |
|-------|-------------|
| `cf-spatial` | Voxel grids, occupancy maps, raycasting, DDA traversal |

---

## Quality Standards

We maintain **A-grade standards** for all code, enforced by CI.

| Criterion | Standard |
|-----------|----------|
| Test Coverage | ≥75% (target: 90%) |
| Documentation | Zero warnings |
| Clippy | Zero warnings (pedantic + nursery) |
| Safety | Zero `unwrap`/`expect` in library code |
| Dependencies | Audited (cargo-deny, cargo-audit) |
| Bevy-free (Layer 0) | No bevy in dependency tree |
| Commits | Signed + conventional format |

See [STANDARDS.md](./STANDARDS.md) for full details.
See [CONTRIBUTING.md](./CONTRIBUTING.md) for the workflow.

```bash
# Check your work
cargo xtask grade <crate-name>

# Record completion
cargo xtask complete <crate-name>
```

---

## Development

```bash
# Clone
git clone https://github.com/cortenforge/forge.git
cd forge

# Build (auto-installs git hooks on first build)
cargo build --workspace

# Run tests
cargo test --workspace

# Run quality checks
cargo xtask check

# Grade a specific crate
cargo xtask grade mesh-types
```

Git hooks enforce formatting, clippy, and conventional commits automatically.

---

## License

Apache-2.0. See [LICENSE](./LICENSE).

---

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md).

**A-grade or it doesn't ship. No exceptions.**
