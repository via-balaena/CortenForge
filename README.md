# CortenForge

> A Rust SDK for mechatronics and simulation — and the hydrogen farm machines we build with it to prove it works.

[![Quality Gate](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml/badge.svg)](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)
[![Built with Rust](https://img.shields.io/badge/built%20with-Rust-dea584?logo=rust&logoColor=white)](https://www.rust-lang.org/)

## What it is

Composable Rust components for the full path **physical → digital → physical**: geometry, parametric design, meshing and fabrication, rigid- and soft-body physics, control and reinforcement learning, sim-to-real. **The kit is the product.**

To prove the components compose end to end, we build a family of **hydrogen farm vehicles for the upper Midwest** — tractor, truck, trike — and the wind-to-hydrogen fuel chain that powers them, and answer one question: *how many acres per season can one farm run on its own wind?* **Not yet built.** See **[MISSION.md](./MISSION.md)**.

## ⚠️ Disclaimer

CortenForge — including the **Cendrillon** application — is general-purpose research and engineering software, provided **AS IS** under [MIT](./LICENSE-MIT) or [Apache-2.0](./LICENSE-APACHE) at your option, **without warranty of any kind**. **It is not a medical device** and makes no medical, therapeutic, or health claims. You use it — and make and use anything created with it — **entirely at your own risk**: you alone are responsible for deciding whether a design is fit for what you intend to do with it, and for the materials, fabrication, testing, and operation of anything you build. What you build with it can involve serious hazards — hydrogen and other compressed or flammable gases, pressure vessels, high-voltage systems, moving machinery, vehicles that are not certified for road use, and materials that contact the body. See **[DISCLAIMER.md](./DISCLAIMER.md)** for the full text.

## Stack

| Domain | Crates | Highlights |
|--------|--------|-----------|
| **Rigid-body physics** | sim-core, sim-mjcf, sim-urdf | MuJoCo-aligned dynamics, validated against the MuJoCo 3.4.0 conformance suite; MJCF + URDF import; analytic and finite-difference derivatives |
| **Soft-body physics** | sim-soft | Hyperelastic FEM (Neo-Hookean / Yeoh), SDF→tet meshing, contact, differentiable |
| **ML / RL / Optimization** | sim-ml-chassis, sim-rl, sim-opt | VecEnv, autograd, CEM, REINFORCE, PPO, TD3, SAC, parallel tempering |
| **Design** | cf-design, cf-geometry, cf-spatial | SDF primitives, smooth booleans, mechanism assembly, MJCF + STL export |
| **Mesh** | mesh-io, mesh-repair, mesh-sdf + 8 more | STL/OBJ/PLY/3MF I/O, repair, offset, shell, lattice, print validation |
| **Scan → fabrication** | cf-scan-prep-core, cf-cast, mesh-printability | Scan cleanup, multi-material mold generation, printability gating |

Pure Rust with no framework dependencies in the physics and mesh cores; the mold-CSG stage builds a vendored C++ kernel through CMake, so a first build of the facade needs CMake and a C++ compiler — `default-features = false, features = ["sim", "mesh"]` skips it. Layer-0 crates are checked against `wasm32-unknown-unknown` by the quality gate.

## Quick start

Applications depend on one crate — the **`cortenforge` facade** — so the internal structure can evolve behind a single import surface. It is headless by design: Bevy/GUI/GPU crates are excluded.

```toml
[dependencies]
# The crates.io listings lag this workspace — depend on the repository:
cortenforge = { git = "https://github.com/via-balaena/CortenForge" }
```

```rust
use cortenforge::sim;   // rigid + soft physics, soft↔rigid coupling, RL/opt
use cortenforge::mesh;  // load / repair / measure / print meshes
use cortenforge::{cf_design, cf_scan_prep_core, cf_cast};
```

```bash
git clone https://github.com/via-balaena/CortenForge.git
cd CortenForge
cargo build -p cortenforge       # the facade
cargo xtask grade <crate-name>   # the quality gate, one crate
```

## Links

| | |
|--|--|
| **Website** | [cortenforge.com](https://cortenforge.com) |
| **Mission** | [MISSION.md](./MISSION.md) |
| **Architecture** | [sim/docs/ARCHITECTURE.md](./sim/docs/ARCHITECTURE.md) |
| **Standards** | [docs/STANDARDS.md](./docs/STANDARDS.md) |
| **Contributing** | [CONTRIBUTING.md](./CONTRIBUTING.md) |

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](./LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](./LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
