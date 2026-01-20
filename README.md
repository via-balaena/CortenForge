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
- **Physics Simulation** - Rigid body dynamics, constraints, contact models, URDF loading
- **Machine Learning** - Model training and inference (Burn)
- **Computer Vision** - Object detection, tracking, sensor fusion

The same code runs in simulation and on hardware. Our code must be as reliable as the things built with it.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              LAYER 1: Bevy SDK                               │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                           cortenforge                                    ││
│  │  CfMeshPlugin · CfRoutingPlugin · CfVisionPlugin · CfSimPlugin · CfUi  ││
│  └─────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         LAYER 0: Pure Rust Foundation                        │
│                           (Zero Bevy Dependencies)                           │
├──────────────┬──────────────┬──────────────┬──────────────┬─────────────────┤
│   mesh/*     │  geometry/*  │   routing/*  │     sim/*    │   ml/* + more   │
│   27 crates  │              │              │              │                 │
├──────────────┼──────────────┼──────────────┼──────────────┼─────────────────┤
│ mesh-types   │ curve-types  │ route-types  │ sim-types    │ ml-models       │
│ mesh-io      │   bezier     │ route-path   │ sim-core     │ ml-inference    │
│ mesh-repair  │   bspline    │ route-opt    │ sim-contact  │ ml-dataset      │
│ mesh-boolean │   nurbs      │              │ sim-const    │ ml-training     │
│ mesh-lattice │   arc/helix  │              │ sim-urdf     │ sensor-fusion   │
│ mesh-scan    │              │              │ sim-physics  │ vision-core     │
│ ...          │              │              │              │ cf-spatial      │
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
use mesh_repair::{repair, RepairConfig};

// Load and repair a mesh
let mesh = load_stl("model.stl")?;
let repaired = repair(&mesh, &RepairConfig::default())?;
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

**45+ crates** across 6 domains, all Layer 0 (zero Bevy dependencies).

### Mesh Domain (`mesh/`) — 27 crates

| Crate | Description |
|-------|-------------|
| `mesh-types` | Core types: `Vertex`, `IndexedMesh`, `Triangle`, `AABB` |
| `mesh-io` | File I/O: STL, OBJ, PLY, glTF |
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

### Simulation Domain (`sim/`)

| Crate | Description |
|-------|-------------|
| `sim-types` | RigidBodyState, Pose, Twist, JointState, MassProperties |
| `sim-core` | World container, integrators (Euler, Verlet, RK4), stepper |
| `sim-constraint` | Joint types (revolute, prismatic, spherical), motors, limits |
| `sim-contact` | MuJoCo-inspired contact model, friction (Coulomb, Stribeck) |
| `sim-urdf` | URDF parser, kinematic tree validation, mass property validation |
| `sim-physics` | Umbrella crate re-exporting all sim crates |

### ML Domain (`ml/`)

| Crate | Description |
|-------|-------------|
| `ml-models` | Neural network architectures (Burn) |
| `ml-inference` | Inference runtime |
| `ml-dataset` | Dataset management |
| `ml-training` | Training loops |

### Vision Domain (`vision/`)

| Crate | Description |
|-------|-------------|
| `vision-core` | Detection traits, frame types |
| `vision-capture` | Camera capture, recording |

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
