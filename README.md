# CortenForge

> **Industrial-grade foundation, unlimited application.**
> From humanoid robots to vehicles to medical simulation.

[![Quality Gate](https://github.com/cortenforge/forge/actions/workflows/quality-gate.yml/badge.svg)](https://github.com/cortenforge/forge/actions/workflows/quality-gate.yml)
[![Coverage](https://codecov.io/gh/cortenforge/forge/branch/main/graph/badge.svg)](https://codecov.io/gh/cortenforge/forge)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

---

## What is CortenForge?

CortenForge is an SDK for building physical things in software. It provides the computational foundation for:

- **Mesh Processing** - Load, repair, transform, offset, shell generation
- **Parametric Geometry** - Bezier curves, B-splines, tubular anatomy
- **3D Routing** - Pathfinding for wire harnesses, pipes, tendons
- **Machine Learning** - Model training and inference (Burn)
- **Computer Vision** - Object detection and tracking
- **Physics Simulation** - Rigid body dynamics (Avian)

Our code must be as reliable as the things built with it.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              LAYER 1: Bevy SDK                               │
│                                                                             │
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
├─────────────┬─────────────┬─────────────┬─────────────┬─────────────────────┤
│   mesh/*    │ geometry/*  │  routing/*  │    ml/*     │  vision/* · sim/*   │
├─────────────┼─────────────┼─────────────┼─────────────┼─────────────────────┤
│ mesh-types  │ curve-types │ route-types │ ml-models   │ vision-core         │
│ mesh-io     │ lumen-geo   │ route-path  │ ml-inference│ vision-capture      │
│ mesh-repair │             │ route-opt   │ ml-dataset  │ sim-core            │
│ mesh-offset │             │             │ ml-training │ sim-physics         │
│ mesh-shell  │             │             │             │                     │
│ mesh-zones  │             │             │             │                     │
│ mesh-sdf    │             │             │             │                     │
│ ...         │             │             │             │                     │
├─────────────┴─────────────┴─────────────┴─────────────┴─────────────────────┤
│                              crates/cf-spatial                               │
│                        (Voxel grids, spatial queries)                        │
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

### Mesh Domain (`mesh/`)

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
| `curve-types` | Bezier, B-spline, Catmull-Rom curves |
| `lumen-geometry` | Tubular anatomy (colon, vessels) |

### Routing Domain (`routing/`)

| Crate | Description |
|-------|-------------|
| `route-types` | Paths, routes, constraints |
| `route-pathfind` | A* on voxel grids, collision-free paths |
| `route-optimize` | Multi-objective route optimization |

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

### Simulation Domain (`sim/`)

| Crate | Description |
|-------|-------------|
| `sim-core` | Simulation configuration |
| `sim-physics` | Physics abstractions (Avian backend) |

### Foundation (`crates/`)

| Crate | Description |
|-------|-------------|
| `cf-spatial` | Voxel grids, occupancy maps, spatial queries |

---

## Quality Standards

We maintain **A-grade academic standards** for all code.

| Criterion | A Standard |
|-----------|------------|
| Test Coverage | ≥90% |
| Documentation | Zero warnings |
| Clippy | Zero warnings |
| Safety | Zero `unwrap`/`expect` in library code |
| Dependencies | Minimal, justified |
| Bevy-free (Layer 0) | No bevy in dependency tree |
| API Design | Idiomatic, intuitive |

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

# Build everything
cargo build --workspace

# Run tests
cargo test --workspace

# Run quality checks
cargo xtask check

# Grade a specific crate
cargo xtask grade mesh-types
```

---

## License

Apache-2.0. See [LICENSE](./LICENSE).

---

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md).

**A-grade or it doesn't ship. No exceptions.**
