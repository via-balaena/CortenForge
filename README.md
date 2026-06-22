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

```bash
git clone https://github.com/via-balaena/CortenForge.git
cd CortenForge
cargo build --workspace
cargo xtask grade <crate-name>
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
