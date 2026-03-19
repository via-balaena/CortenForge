# CortenForge

> **An AI-native, simulation-first, manufacturing-ready platform for bio-inspired mechatronics.** Open-source, pure Rust, from design through physics simulation to 3D printing.

[![Quality Gate](https://github.com/cortenforge/forge/actions/workflows/quality-gate.yml/badge.svg)](https://github.com/cortenforge/forge/actions/workflows/quality-gate.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

---

## What is CortenForge?

CortenForge is a pure-Rust platform where AI (or humans) write code that
defines mechanisms — and that code produces geometry that is simultaneously
a physics collider, a 3D-printable artifact, and a simulation model. No
export steps, no approximation gaps.

```
Code → geometry IS the collider → simulate → 3D print
```

The same types flow through every layer. An `Arc<IndexedMesh>` loaded from
STL is the same mesh that sim-core collides against, that sim-bevy renders,
and that mesh-io exports back to STL. One truth.

**What's built:**

- **Geometric Kernel** — cf-geometry: unified shapes, GJK/EPA, BVH, ray casting, SDF grids — used by all domains
- **Physics Simulation** — MuJoCo-aligned rigid body dynamics: Newton/PGS/CG contact solvers, implicit integration, Hill-type muscles, tendons, constraints. 79/79 conformance tests pass.
- **Mesh Processing** — 10 crates: load, repair, SDF, offset, shell, lattices, print validation
**What's next:**

- **cf-design** — Implicit surface design kernel: define parts as math functions, compose with smooth booleans, assemble into mechanisms that map 1:1 to MJCF. See [CF_DESIGN_SPEC.md](./docs/CF_DESIGN_SPEC.md).

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              LAYER 1: Bevy SDK                              │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │  sim-bevy: Model/Data sync, debug gizmos, coordinate conversion        ││
│  └─────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         LAYER 0: Pure Rust Foundation                       │
│                           (Zero Bevy Dependencies)                          │
├──────────┬────────────┬──────────────┤
│  mesh/*  │ cf-geometry │    sim/*     │
│ 10 crates│ cf-spatial  │   7 crates   │
│          │ cf-design   │              │
├──────────┼────────────┼──────────────┤
│ mesh-io  │ Aabb       │ sim-core     │
│ mesh-    │ IndexedMesh│ sim-mjcf     │
│  repair  │ Shape(10)  │ sim-urdf     │
│ mesh-sdf │ GJK/EPA    │ sim-types    │
│ mesh-    │ BVH        │ sim-simd     │
│  lattice │ ConvexHull │ sim-bevy(L1) │
│ mesh-    │ SdfGrid    │ sim-tests¹   │
│  shell   │ Ray/RayHit │              │
└──────────┴────────────┴──────────────┘
```

### Layer 0: Pure Rust

All foundation crates have **zero Bevy dependencies**. They compile to native
binaries, WASM, Python modules (via PyO3), embedded targets, and other engines.

### Layer 1: Bevy Integration

sim-bevy provides Bevy visualization with debug gizmos and coordinate
conversion. This is the only place Bevy appears.

---

## Crate Overview

**21 library crates** across 3 domains.

### Foundation (`design/`)

| Crate | Description |
|-------|-------------|
| `cf-geometry` | Shared geometric kernel: Aabb, IndexedMesh, Shape (10 variants), ConvexHull, BVH, SdfGrid, GJK/EPA, ray casting, closest point queries. The canonical source of geometric types for all domains. |
| `cf-spatial` | Voxel grids, occupancy maps, raycasting, DDA traversal |

### Simulation Domain (`sim/`) — 7 crates

| Crate | Layer | Description |
|-------|-------|-------------|
| `sim-core` | L0 | MuJoCo-aligned pipeline: Model/Data, FK, CRBA, RNE, Newton/PGS/CG solvers, implicit integration, sleeping, Hill-type muscles, tendons, constraints. 79/79 conformance. |
| `sim-mjcf` | L0 | MuJoCo XML parser and model builder |
| `sim-urdf` | L0 | URDF parser, kinematic tree validation |
| `sim-types` | L0 | RigidBodyState, Pose, Twist, JointState, MassProperties |
| `sim-simd` | L0 | SIMD batch operations (Vec3x4/Vec3x8) |
| `sim-conformance-tests` | L0 | MuJoCo conformance and integration tests |
| `sim-bevy` | L1 | Bevy visualization, debug gizmos, coordinate conversion |

See [sim/docs/ARCHITECTURE.md](./sim/docs/ARCHITECTURE.md) for pipeline details and [sim/docs/ROADMAP_V1.md](./sim/docs/ROADMAP_V1.md) for the completed v1.0 roadmap.

### Mesh Domain (`mesh/`) — 10 crates

| Crate | Description |
|-------|-------------|
| `mesh-types` | Core mesh types, attribute storage (re-exports cf-geometry primitives) |
| `mesh-io` | File I/O: STL, OBJ, PLY, 3MF, STEP |
| `mesh-repair` | Weld vertices, remove degenerates, fill holes |
| `mesh-sdf` | Signed distance field computation |
| `mesh-offset` | Mesh offset via SDF |
| `mesh-shell` | Shell generation for 3D printing |
| `mesh-measure` | Dimensions, volume, surface area |
| `mesh-printability` | Print validation |
| `mesh-lattice` | Lattice structure generation |
| `mesh` | Umbrella re-export |

---

## Quality Standards

**A-grade or it doesn't ship.**

| Criterion | Standard |
|-----------|----------|
| Test Coverage | ≥75% (target: 90%) |
| Documentation | Zero warnings |
| Clippy | Zero warnings (pedantic + nursery) |
| Safety | Zero `unwrap`/`expect` in library code |
| Dependencies | Audited (cargo-deny, cargo-audit) |
| Bevy-free (Layer 0) | No bevy in dependency tree |
| Commits | Signed + conventional format |

See [STANDARDS.md](./docs/STANDARDS.md) for full details.

```bash
# Grade a crate
cargo xtask grade <crate-name>

# Run quality gate
cargo xtask check
```

---

## Development

```bash
# Clone
git clone https://github.com/cortenforge/forge.git
cd forge

# Build
cargo build --workspace

# Test a domain (prefer domain-scoped over full workspace)
cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests
cargo test -p mesh -p mesh-types -p mesh-io

# Full quality check
cargo xtask check
```

Git hooks enforce formatting, clippy, and conventional commits automatically.

---

## Key Documents

| Document | Purpose |
|----------|---------|
| [VISION.md](./docs/VISION.md) | North star — architecture philosophy, domain coverage, milestones |
| [CF_GEOMETRY_SPEC.md](./docs/CF_GEOMETRY_SPEC.md) | Shared geometric kernel spec (implemented) |
| [CF_DESIGN_SPEC.md](./docs/CF_DESIGN_SPEC.md) | Implicit surface design kernel spec (planned) |
| [STANDARDS.md](./docs/STANDARDS.md) | Quality criteria and grading rubric |
| [CONTRIBUTING.md](./CONTRIBUTING.md) | Development workflow |
| [sim/docs/ARCHITECTURE.md](./sim/docs/ARCHITECTURE.md) | Physics simulation pipeline details |

---

## License

Apache-2.0. See [LICENSE](./LICENSE).

---

**A-grade or it doesn't ship. No exceptions.**
