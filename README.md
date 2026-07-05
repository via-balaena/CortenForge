# CortenForge

> A software development kit for the mechatronics and simulation space — composable Rust components for the full path from a physical scan to a simulated, designed, optimized, and manufactured system, and back.

[![Quality Gate](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml/badge.svg)](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Built with Rust](https://img.shields.io/badge/built%20with-Rust-dea584?logo=rust&logoColor=white)](https://www.rust-lang.org/)

## What it is

CortenForge is, at its core, a **software development kit**. It provides the building blocks spanning the full path **physical → digital → physical**: geometry, parametric design, meshing and digital fabrication, rigid- and soft-body physics, control and reinforcement learning, and sim-to-real calibration. Because those components are general, they serve a near-limitless range of adjacent applications — robotics, soft robotics, biomechanics, generative design and digital fabrication, custom-fit products, and embodied-AI research.

The kit is the product. To prove the components compose end to end, our capstone undertaking — **not yet built** — is a differentiable body-to-device co-design loop for person-specific assistive robotics, with a powered, RL-controlled exoskeleton as the demonstration. See **[MISSION.md](./MISSION.md)**.

## ⚠️ Disclaimer

CortenForge — including the **Cendrillon** application — is general-purpose research and engineering software, provided **AS IS** under the [Apache License 2.0](LICENSE), **without warranty of any kind**. **It is not a medical device** and makes no medical, therapeutic, or health claims. You use it — and make and use anything created with it — **entirely at your own risk**; you alone are responsible for choosing body-safe materials and for proper mixing, curing, and hygiene, and should always follow the manufacturer's safety data sheet (SDS). See **[DISCLAIMER.md](./DISCLAIMER.md)** for the full text.

## Stack

| Domain | Crates | Highlights |
|--------|--------|-----------|
| **Rigid-body physics** | sim-core, sim-mjcf, sim-urdf | MuJoCo-aligned dynamics (79/79 conformance), MJCF + URDF import, analytic derivatives |
| **Soft-body physics** | sim-soft | Hyperelastic FEM (NeoHookean / Yeoh), SDF→tet meshing, contact, differentiable |
| **ML / RL / Optimization** | sim-ml-chassis, sim-rl, sim-opt | VecEnv, autograd, CEM, REINFORCE, PPO, TD3, SAC, SA, parallel tempering |
| **Thermo environments** | sim-therm-env, sim-thermostat | Langevin thermostats, double wells, pairwise coupling, pluggable rewards |
| **Design** | cf-design, cf-geometry, cf-spatial | SDF primitives, smooth booleans, mechanism assembly, MJCF + STL export |
| **Mesh** | mesh-io, mesh-repair, mesh-sdf + 7 more | STL/OBJ/PLY/3MF I/O, repair, offset, shell, lattice, print validation |
| **Scan → fabrication** | cf-scan-prep, cf-cast, mesh-printability | Scan cleanup, multi-material mold generation, printability gating, procedure generation |

40+ crates. Pure Rust. Zero framework dependencies (Layer 0). Compiles to native, WASM, Python.

## Quick Start

### Use the SDK

Applications depend on a single crate — the **`cortenforge` facade** — and reach the whole SDK through it, so the internal crate structure can evolve behind one stable contract.

```toml
[dependencies]
# Publishing to crates.io is in progress; until then, depend on it via git:
cortenforge = { git = "https://github.com/via-balaena/CortenForge" }
```

```rust
// Two domain umbrellas expose the whole toolkit through one dependency:
use cortenforge::sim;   // rigid + soft physics, soft↔rigid coupling, RL/opt
use cortenforge::mesh;  // load / repair / measure / print meshes

// …alongside the design & fabrication path:
use cortenforge::{cf_design, cf_scan_prep_core, cf_cast, cf_cast_cli};
```

The facade is a headless capability map across three domains:

- **Simulation & co-design** — `cortenforge::sim`: rigid-body dynamics (`sim::core`), soft-body FEM (`sim::soft`), the differentiable soft↔rigid **coupling keystone** (`sim::coupling`), model I/O (`sim::mjcf` / `sim::urdf`), and the learning + optimization stack (`sim::ml_chassis` / `sim::rl` / `sim::opt`).
- **Mesh processing** — `cortenforge::mesh`: `mesh::io` (STL/OBJ/PLY/3MF), `mesh::repair`, `mesh::sdf`, `mesh::shell`, `mesh::measure`, `mesh::printability`, …
- **Design → fabrication** — the implicit-surface design kernel (`cf_design`), headless scan-prep (`cf_scan_prep_core`), and multi-material mold generation (`cf_cast` / `cf_cast_cli`).

Bevy/GUI/GPU crates are deliberately excluded, so every app compiling against the facade stays headless. See **[MISSION.md](./MISSION.md)**.

### Build from source

```bash
git clone https://github.com/via-balaena/CortenForge.git
cd CortenForge
cargo build --workspace
cargo xtask grade <crate-name>   # run the quality gate on a single crate
```

## Links

| | |
|--|--|
| **Website** | [cortenforge.com](https://cortenforge.com) |
| **Mission** | [MISSION.md](./MISSION.md) |
| **Research** | [Biological Navigation &mdash; X-Encoding Design Framework](https://cortenforge.com/research/) |
| **Architecture** | [sim/docs/ARCHITECTURE.md](./sim/docs/ARCHITECTURE.md) |
| **Standards** | [docs/STANDARDS.md](./docs/STANDARDS.md) |
| **Contributing** | [CONTRIBUTING.md](./CONTRIBUTING.md) |

## License

Apache-2.0
