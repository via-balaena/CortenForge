# CortenForge

> Simulation infrastructure for extracting and applying engineering principles from biological systems.

[![Quality Gate](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml/badge.svg)](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Built with Rust](https://img.shields.io/badge/built%20with-Rust-dea584?logo=rust&logoColor=white)](https://www.rust-lang.org/)

## Stack

| Domain | Crates | Highlights |
|--------|--------|-----------|
| **Physics** | sim-core, sim-mjcf, sim-urdf | MuJoCo-aligned dynamics (79/79 conformance), MJCF + URDF import |
| **Thermo Environments** | sim-therm-env, sim-thermostat | Langevin thermostats, double wells, pairwise coupling, pluggable rewards |
| **ML / RL / Optimization** | sim-ml-chassis, sim-rl, sim-opt | VecEnv, autograd, CEM, REINFORCE, PPO, TD3, SAC, SA, parallel tempering |
| **Design** | cf-design, cf-geometry | SDF primitives, smooth booleans, mechanism assembly, MJCF + STL export |
| **Mesh** | mesh-io, mesh-repair, mesh-sdf + 7 more | STL/OBJ/PLY/3MF I/O, repair, offset, shell, lattice, print validation |

20 crates. Pure Rust. Zero framework dependencies (Layer 0). Compiles to native, WASM, Python.

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
| **Research** | [Biological Navigation &mdash; X-Encoding Design Framework](https://cortenforge.com/research/) |
| **Architecture** | [sim/docs/ARCHITECTURE.md](./sim/docs/ARCHITECTURE.md) |
| **Standards** | [docs/STANDARDS.md](./docs/STANDARDS.md) |
| **Contributing** | [CONTRIBUTING.md](./CONTRIBUTING.md) |

## License

Apache-2.0
