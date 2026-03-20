# CortenForge Vision

> An open-source SDK written in pure Rust for designing, simulating, and manufacturing bio-inspired mechanisms and robotics.

---

## What CortenForge Is

CortenForge is an **open-source, pure-Rust SDK for bio-inspired mechatronics** — spanning the full arc from mechanism design and mesh processing to MuJoCo-grade physics simulation and 3D-printable output. It demonstrates that the Rust programming language is not merely adequate but *ideal* for this domain: real-time simulation, geometric computing, implicit surface modeling, and deterministic builds.

It is not:
- A game engine (though it runs on one)
- A CAD program (though it has CAD-grade geometry)
- A physics simulator (though it simulates physics)

It is all of these composed into a coherent system where **the design geometry IS the simulation collider IS the 3D-printable artifact**.

---

## The Core Insight

The gap between design, simulation, and manufacturing is the central tax of mechatronics. Teams design in one tool, export to another for simulation, export again for manufacturing, and spend weeks reconciling the gaps.

CortenForge eliminates this by design:

```
┌─────────────────────────────────────────────────────────────────┐
│                    ZERO EXPORT GAPS                              │
│                                                                 │
│    cf-design                sim-core              mesh pipeline  │
│   ┌──────────┐           ┌──────────┐           ┌──────────┐   │
│   │ Implicit  │──Solid──▶│ MuJoCo-  │──Mesh───▶│ Repair,  │   │
│   │ Surfaces  │          │ aligned  │          │ Analyze, │   │
│   │ + Boolean │          │ Physics  │          │ Export   │   │
│   │ + Mech.   │          │          │          │          │   │
│   └──────────┘           └──────────┘           └──────────┘   │
│        │                                             │          │
│        │              SAME GEOMETRY                   │          │
│        └─────────────────────────────────────────────┘          │
│                              │                                  │
│                              ▼                                  │
│                     ┌──────────────┐                            │
│                     │  STL / 3MF   │                            │
│                     │  3D Printing │                            │
│                     └──────────────┘                            │
└─────────────────────────────────────────────────────────────────┘
```

A `cf_design::Solid` created from implicit surfaces and smooth booleans is directly usable as a simulation collider in `sim-core` and directly meshable to STL for 3D printing. No format conversion. No geometry approximation loss. One representation flows through the entire pipeline.

---

## Architecture Philosophy

### Layer 0: Pure Rust Foundation

**Zero Bevy dependencies. Zero framework lock-in.**

Every Layer 0 crate compiles to:
- Native binaries
- WASM for browsers
- Python modules via PyO3
- Embedded targets
- Other game engines

This is non-negotiable. Layer 0 is the **permanent foundation**. Bevy could be replaced tomorrow and Layer 0 would remain intact.

```
Layer 0 (Pure Rust) — 19 library crates
├── mesh/           10 crates — Industrial mesh processing
├── cf-geometry     Shared geometric kernel (shapes, queries, acceleration)
├── cf-spatial      Voxel grids, spatial indexing
├── cf-design       Implicit surface design kernel (AI-native mechanism design)
└── sim/            6 crates — MuJoCo-aligned physics
    ├── sim-types           Pose, Twist, MassProperties, JointState
    ├── sim-simd            SIMD batch operations (Vec3x4/Vec3x8)
    ├── sim-core            Model/Data, FK, CRBA, RNE, Newton/PGS/CG solvers,
    │                       implicit integration, muscles, deformables, constraints
    ├── sim-mjcf            MuJoCo XML parser
    ├── sim-urdf            URDF parser, kinematic tree validation
    └── sim-conformance-tests   79/79 MuJoCo conformance tests
```

### Layer 1: Bevy Integration

**ECS-native, not wrapped legacy code.**

Layer 1 is plugins that wire Layer 0 into Bevy's ECS. Components hold data. Systems process it. Resources manage state.

```
Layer 1 (Bevy) — 1 crate
└── sim-bevy        Model/Data sync, debug gizmos, coordinate conversion
```

When Bevy breaks (and it will), Layer 1 adapts. Layer 0 is untouched.

### The Simulation Question

**Simulation lives in BOTH layers.**

Why? Because simulation math is pure. `F = ma` doesn't need a game engine. But *visualizing* simulation, *interacting* with it, *recording* it — that's where Bevy excels.

A headless training run uses `sim-core` directly. A visualization demo uses `sim-bevy`. Same physics. Different interfaces.

`sim-core` is the densest crate in the workspace. It inlines the full MuJoCo-aligned pipeline: rigid body dynamics, contact solvers (Newton, PGS, CG), implicit integration (Euler, RK4, implicit fast), joint types with motors and limits, Hill-type muscles, XPBD deformables, sleeping with sparse LDL factorization, and equality constraints. This is deliberate — these subsystems are tightly coupled in MuJoCo's architecture, and separating them created artificial boundaries.

---

## Domain Coverage

### What We Build vs. What We Use

| Domain | Strategy | Status |
|--------|----------|--------|
| **Mesh Processing** | Build | 10 crates, A-grade |
| **Design** | Build (cf-design) | Phases 1–4 complete: implicit surfaces, smooth booleans, mechanism assembly, MJCF/STL export |
| **Geometry** | Build (cf-geometry) | Shared kernel — shapes, queries, acceleration structures |
| **Spatial** | Build (cf-spatial) + kiddo | Voxels ourselves, KD-trees from crate |
| **Rigid Physics** | Build (sim-core) | MuJoCo-aligned: Model/Data, CRBA, RNE, Newton/PGS/CG solvers, implicit integration, sleeping |
| **Soft Physics** | Build (sim-core) | Inlined: XPBD deformables, Hill-type muscles |
| **Collision** | Build (sim-core) | MuJoCo-aligned: analytical + GJK/EPA + BVH mesh |
| **Rendering** | Use Bevy | Not our domain |
| **UI** | Use bevy_egui | Not our domain |

### Domain Roadmap

```
COMPLETE (20 library crates):
├── Mesh Domain (10 crates)
│   mesh, mesh-types, mesh-io, mesh-repair, mesh-measure,
│   mesh-offset, mesh-shell, mesh-lattice, mesh-sdf, mesh-printability
├── Design Domain (3 crates)
│   cf-geometry, cf-spatial, cf-design (Phases 1–4: 656 tests)
└── Simulation Domain (7 crates: 6 L0 + 1 L1)
    sim-types, sim-simd, sim-core, sim-mjcf, sim-urdf,
    sim-conformance-tests, sim-bevy

NEXT PHASE:
├── cf-design Phase 5     Differentiable design optimization
├── PyO3 bindings         Python access to Layer 0
└── cortenforge           Bevy SDK umbrella (CfSimPlugin, etc.)

FUTURE:
├── B-Rep upgrade         Ship of Theseus swap of cf-design internals (if Fornjot matures)
├── Hardware bridges      ROS2, custom drivers, sim/real parity
├── GPU simulation        Batched sim via wgpu
└── surface-types         NURBS surfaces, patches (if needed beyond implicit)
```

---

## Quality Standard

**A-grade or it doesn't ship.**

This is not marketing. This is policy enforced by CI:

1. **Zero `unwrap()`/`expect()` in library code** - Errors are values
2. **≥75% test coverage (target: 90%)** - Measured and enforced
3. **Zero Clippy warnings** - pedantic + nursery enabled
4. **Doc examples for all public APIs** - Tested in CI
5. **Conventional commits** - Machine-readable history
6. **Semver compliance** - cargo-semver-checks on every PR
7. **Supply chain verified** - cargo-vet, cargo-audit, cargo-deny

Why this strict? Because CortenForge targets domains where correctness is non-negotiable. The standards exist so the code earns that trust.

---

## The Behemoth Comparison

### Siemens NX / Solid Edge (+ Parasolid)
What they have: Full parametric CAD, B-Rep kernel (Parasolid), assembly constraints, simulation integration, 40 years of development.

What we have: Shared geometric kernel (cf-geometry), implicit surface design kernel (cf-design) with smooth booleans and mechanism assembly, MuJoCo-aligned physics (sim-core, 79/79 conformance tests), industrial mesh processing (10 crates).

What we're building differently: Not B-Rep-first. Implicit surfaces for AI-native design of bio-inspired mechanisms. The design geometry IS the simulation collider IS the 3D-printable artifact. No export/import gap. See `CF_DESIGN_SPEC.md`.

B-Rep upgradability preserved via Ship of Theseus architecture — cf-design's `Solid` type is opaque, internals can be swapped to Fornjot B-Rep if/when needed.

### MuJoCo
What they have: 15 years of physics research, soft contacts, implicit integration, GPU acceleration, massive adoption in RL research.

What we have: A MuJoCo-aligned pipeline (7 crates) implementing Model/Data, FK, CRBA, RNE, Newton/PGS/CG contact solvers, implicit integration (Euler, RK4, implicit fast), MJCF/URDF loading, Hill-type muscles, XPBD deformables, sleeping with sparse LDL factorization, and 79/79 conformance tests passing.

What we need: Broader actuator coverage, full tendon/site transmissions, full MuJoCo XML parity across all edge cases, GPU-batched simulation.

The honest assessment: We can get useful CAD interop in a year. Full MuJoCo parity (all actuators, all constraint types) is ongoing work. See `sim/docs/ARCHITECTURE.md` and `sim/docs/MUJOCO_REFERENCE.md`.

---

## Target Applications

### Bio-Inspired Robotics (Primary)
- **Design:** Compliant mechanisms, actuator housings, bio-inspired body shells via implicit surfaces
- **Simulation:** Locomotion, manipulation, contact-rich tasks with muscles and tendons
- **Mesh:** Geometry repair, printability analysis, lattice infill for lightweight structures

### Humanoid and Legged Robots
- **Design:** Link geometry, gripper designs, mechanism assemblies
- **Simulation:** Walking, reaching, whole-body dynamics with MuJoCo-grade contact
- **Mesh:** Export to STL/3MF for rapid prototyping

### Medical Devices
- **Design:** Patient-specific implants, surgical tools via code-first implicit surfaces
- **Simulation:** Device mechanics, tissue interaction modeling
- **Mesh:** Mesh repair from scan data, printability validation

All three share the same foundation: design in cf-design, simulate in sim-core, manufacture through the mesh pipeline. CortenForge is the shared substrate.

---

## Growth Model

### Architecture That Welcomes Change

The crate structure is designed for AI-assisted development:

1. **Small, focused crates** - Each crate fits in context
2. **Clear boundaries** - Dependencies are explicit in Cargo.toml
3. **Comprehensive tests** - AI can verify changes work
4. **Documentation in code** - Doc comments are the source of truth

When AI capabilities improve:
- Larger context = more crates considered together
- Better reasoning = more complex algorithms implemented correctly
- Faster iteration = more experiments, faster convergence

The CI/CD pipeline is the immune system. It catches regressions regardless of who (or what) wrote the code.

### Breaking Changes Are Welcome

Bevy will break things. This is fine.

Layer 0 is untouched. Layer 1 adapts. The architecture absorbs change because the boundaries are clean.

When we need to rebuild Layer 1 from scratch? The AI does it in a day. The tests verify it works. Ship it.

### The Rebuild Scenario

"My AI literally just rebuilt everything ever written on my local home lab."

This is coming. The question is: what survives a rebuild?

**What survives:**
- Type definitions (they encode domain knowledge)
- Test cases (they encode expected behavior)
- Documentation (it encodes intent)
- Architecture decisions (INFRASTRUCTURE.md, this document)

**What gets rebuilt:**
- Implementation details
- Boilerplate
- Anything that can be derived from types + tests

CortenForge is structured so the *permanent* artifacts (types, tests, docs) are maximally information-dense, and the *transient* artifacts (implementations) can be regenerated.

---

## What "Done" Means

There is no "done." Cathedrals are never finished; they're maintained and extended.

But there are milestones:

### Milestone 1: Foundation ✅
- 20 library crates across 3 domains (mesh, design, simulation)
- MuJoCo-aligned physics with 79/79 conformance tests
- Implicit surface design kernel with mechanism assembly (cf-design Phases 1–4)
- Industrial mesh processing pipeline (10 crates)
- sim-bevy provides Bevy visualization with debug gizmos

### Milestone 2: Full Pipeline
- Design in cf-design — AI or human writes mechanism code
- Design geometry IS the simulation collider (zero export step)
- Simulate in sim-core
- Train policies in simulation
- Export to STL/3MF for 3D printing / manufacturing
- Iterate

### Milestone 3: Hardware Bridge
- Real sensor data flows through shared types
- Control commands go to real actuators
- Same code runs in sim and on hardware

### Milestone 4: Community
- Documentation complete enough for external contributors
- Examples for each major use case
- Python bindings stable
- First external production use

---

## Principles

1. **Layer 0 purity is sacred.** No exceptions. Ever.

2. **Tests are documentation.** If behavior isn't tested, it's not guaranteed.

3. **Types are contracts.** Get them right first. Implementation follows.

4. **Simple > Clever.** The next person reading this code might be an AI that needs to modify it.

5. **Fail fast, fail loud.** No silent corruption. Errors are values. Handle them.

6. **Own the core, use the rest.** We build what defines us. We use what doesn't.

7. **Quality over velocity.** A-grade takes longer. It's worth it.

---

## For Contributors

(Future section - when we're ready for external contributions)

The bar is high. This is intentional.

Before contributing:
1. Read STANDARDS.md
2. Read CONTRIBUTING.md
3. Read INFRASTRUCTURE.md
4. Run `cargo xtask check`
5. Understand why the rules exist

We're building something that will run on robots, in vehicles, in medical devices. The stakes justify the standards.

---

*This document is the north star. When in doubt, check here.*

*Last updated: 2026-03-19*
