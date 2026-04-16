# CortenForge

> **Simulation infrastructure for extracting and applying engineering principles from biological systems.**

[![Quality Gate](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml/badge.svg)](https://github.com/via-balaena/CortenForge/actions/workflows/quality-gate.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

---

## The Idea

Biology has spent hundreds of millions of years engineering solutions to problems that remain unsolved in human-built systems: robust control in turbulent environments, efficient navigation through noise, reliable signal processing at scale with finite energy. These solutions are embedded in organisms — from bacteria to falcons — as strategies that work under the same physics we need to engineer around.

CortenForge is the simulation platform that makes those strategies formally testable. We model biological systems under the physics that governs them, express the engineering principles they embody as quantitative hypotheses with pre-registered statistical gates, and extract the ones that survive as design rules an engineer can act on.

The result is a growing body of hardware-actionable engineering rules, each traceable to a biological origin, validated in simulation, and stated precisely enough that someone building real systems can change what they build tomorrow.

**[Read our current research &rarr;](https://cortenforge.com/research/)**

---

## Current Research: Thermodynamic Circuit Design

Our first application targets the **X-encoding problem** in thermodynamic computing — the unsolved question of how to inject inputs into stochastic circuits so they relax to the correct output. This problem is structurally isomorphic to biological navigation: an organism injecting intent into a noisy physical medium and achieving reliable convergence.

Five biological navigation principles tested on Ising chain models under Langevin dynamics. Three produced quantitative design rules. Two informative failures sharpen the boundary between what transfers and what doesn't.

| Rule | Biological Origin | What to Do | Effect |
|------|------------------|-----------|--------|
| **Noise Tuning** | E. coli chemotaxis | Operate at kT &asymp; 2.3 (weak coupling) or 4.3 (strong). &Delta;V/kT < 3.0. | &plusmn;30% off optimal kills performance. |
| **Injection Timing** | Ctenophore metachronal waves | Phase-lag &delta; &asymp; &pi;/5 between adjacent nodes (weak coupling). | 18–37% fidelity improvement. |
| **Scale-Invariance** | Octopus distributed control | Both rules hold from N=4 to N=16 without retuning. | No re-derivation needed when scaling. |

**The dividing line:** Statistical-mechanical questions (noise tuning, phase coordination, extensivity) transfer from biology to thermodynamic circuits. Dynamical-systems questions (topological invariants, bifurcation sensitivity) do not.

---

## Under the Hood

The biology research runs on a general-purpose engineering stack. 20 pure-Rust library crates, zero framework dependencies at Layer 0, compilable to native binaries, WASM, and Python modules.

```
Code -> geometry IS the collider -> simulate -> 3D print
```

**Physics Simulation** — MuJoCo-aligned rigid body dynamics: Newton, PGS, and CG contact solvers, implicit integration, Hill-type muscles, tendons, constraints. 79/79 MuJoCo conformance tests. Full MJCF and URDF import.

**Thermodynamic Environments** — Langevin thermostats, double-well potentials, pairwise coupling, oscillating fields. `ThermCircuitEnv` builder composes passive components into stochastic circuit models with pluggable reward functions.

**ML Chassis + RL + Optimization** — Algorithm-agnostic chassis (`Algorithm`, `VecEnv`, `Policy`, autograd, networks). RL baselines (CEM, REINFORCE, PPO, TD3, SAC). Gradient-free optimization (simulated annealing, parallel tempering).

**Geometric Design** — Implicit surface kernel: SDF primitives, smooth booleans, mechanism assembly. Designs export directly to MJCF for simulation and STL for 3D printing. One representation, zero export gaps.

**Mesh Processing** — 10 crates: STL/OBJ/PLY/3MF I/O, vertex welding, hole filling, SDF computation, mesh offset, shell generation, lattice structures, print validation.

**Architecture** — The same `IndexedMesh` is the simulation collider, the render mesh, and the 3D-printable artifact. Layer 0 is permanent foundation. Layer 1 (Bevy) is replaceable visualization.

### Quality Standards

A-grade or it doesn't ship. Pedantic clippy, &ge;75% coverage, zero `unwrap` in library code, audited dependencies, signed conventional commits. `cargo xtask grade <crate>` runs a 7-criterion check.

---

## Development

```bash
git clone https://github.com/via-balaena/CortenForge.git
cd CortenForge

cargo build --workspace

# Test a domain (prefer scoped over full workspace)
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests
cargo test -p sim-therm-env --release  # thermodynamic environments

cargo xtask grade <crate-name>  # 7-criterion quality check
```

---

## Links

| | |
|--|--|
| **Research** | [cortenforge.com/research](https://cortenforge.com/research/) |
| **Architecture** | [sim/docs/ARCHITECTURE.md](./sim/docs/ARCHITECTURE.md) |
| **Vision** | [docs/VISION.md](./docs/VISION.md) |
| **Standards** | [docs/STANDARDS.md](./docs/STANDARDS.md) |
| **Contributing** | [CONTRIBUTING.md](./CONTRIBUTING.md) |

---

Apache-2.0. See [LICENSE](./LICENSE).
