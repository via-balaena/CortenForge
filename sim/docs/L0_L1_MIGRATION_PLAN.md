# L0/L1 Directory Structure Migration Plan

> **Purpose:** Restructure `sim/` to explicitly separate Layer 0 (Bevy-free) and Layer 1 (Bevy-dependent) crates.

---

## Current Structure

```
sim/
├── sim-contact/
├── sim-constraint/
├── sim-core/
├── sim-deformable/
├── sim-mjcf/
├── sim-muscle/
├── sim-physics/
├── sim-sensor/
├── sim-simd/
├── sim-tendon/
├── sim-types/
├── sim-urdf/
└── ARCHITECTURE.md
```

## Target Structure

```
sim/
├── L0/
│   ├── contact/       (sim-contact)
│   ├── constraint/    (sim-constraint)
│   ├── core/          (sim-core)
│   ├── deformable/    (sim-deformable)
│   ├── mjcf/          (sim-mjcf)
│   ├── muscle/        (sim-muscle)
│   ├── physics/       (sim-physics)
│   ├── sensor/        (sim-sensor)
│   ├── simd/          (sim-simd)
│   ├── tendon/        (sim-tendon)
│   ├── types/         (sim-types)
│   └── urdf/          (sim-urdf)
├── L1/
│   └── bevy/          (sim-bevy - new)
├── docs/
│   ├── L0_L1_MIGRATION_PLAN.md
│   └── SIM_BEVY_IMPLEMENTATION_PLAN.md (moved from /docs)
└── ARCHITECTURE.md
```

---

## Migration Steps

### Phase 1: Create Directory Structure

```bash
# Create new directories
mkdir -p sim/L0 sim/L1 sim/docs
```

### Phase 2: Move L0 Crates

Move each crate, dropping the `sim-` prefix from directory names (crate names remain unchanged in Cargo.toml):

```bash
git mv sim/sim-types      sim/L0/types
git mv sim/sim-simd       sim/L0/simd
git mv sim/sim-core       sim/L0/core
git mv sim/sim-contact    sim/L0/contact
git mv sim/sim-constraint sim/L0/constraint
git mv sim/sim-sensor     sim/L0/sensor
git mv sim/sim-deformable sim/L0/deformable
git mv sim/sim-muscle     sim/L0/muscle
git mv sim/sim-tendon     sim/L0/tendon
git mv sim/sim-mjcf       sim/L0/mjcf
git mv sim/sim-urdf       sim/L0/urdf
git mv sim/sim-physics    sim/L0/physics
```

### Phase 3: Update Workspace Cargo.toml

Update the root `Cargo.toml` workspace members:

```toml
[workspace]
members = [
    # Layer 0 - Bevy-free simulation
    "sim/L0/types",
    "sim/L0/simd",
    "sim/L0/core",
    "sim/L0/contact",
    "sim/L0/constraint",
    "sim/L0/sensor",
    "sim/L0/deformable",
    "sim/L0/muscle",
    "sim/L0/tendon",
    "sim/L0/mjcf",
    "sim/L0/urdf",
    "sim/L0/physics",

    # Layer 1 - Bevy integration
    "sim/L1/bevy",

    # ... other workspace members
]
```

### Phase 4: Update Inter-Crate Dependencies

Each L0 crate's `Cargo.toml` needs path updates. Example for `sim/L0/core/Cargo.toml`:

```toml
[dependencies]
sim-types = { path = "../types" }
sim-simd = { path = "../simd" }
sim-contact = { path = "../contact" }
sim-constraint = { path = "../constraint" }
```

Example for `sim/L0/physics/Cargo.toml` (depends on most L0 crates):

```toml
[dependencies]
sim-types = { path = "../types" }
sim-simd = { path = "../simd" }
sim-core = { path = "../core" }
sim-contact = { path = "../contact" }
sim-constraint = { path = "../constraint" }
sim-sensor = { path = "../sensor" }
sim-deformable = { path = "../deformable" }
sim-muscle = { path = "../muscle" }
sim-tendon = { path = "../tendon" }
sim-mjcf = { path = "../mjcf" }
sim-urdf = { path = "../urdf" }
```

### Phase 5: Create L1 Placeholder

```bash
mkdir -p sim/L1/bevy/src
```

Create `sim/L1/bevy/Cargo.toml`:

```toml
[package]
name = "sim-bevy"
version = "0.1.0"
edition = "2021"
license = "MIT OR Apache-2.0"
description = "Bevy visualization for CortenForge physics simulation"

[dependencies]
# L0 crates (relative paths go up to L1, then into L0)
sim-types = { path = "../../L0/types" }
sim-core = { path = "../../L0/core" }
sim-contact = { path = "../../L0/contact" }
sim-constraint = { path = "../../L0/constraint" }
sim-mjcf = { path = "../../L0/mjcf" }
sim-urdf = { path = "../../L0/urdf" }

# Bevy (L1 only!)
bevy = { version = "0.18", default-features = false, features = [
    "3d",
    "bevy_pbr",
    "bevy_gizmos",
    "bevy_winit",
    "bevy_state",
    "x11",
    "wayland",
] }
```

Create `sim/L1/bevy/src/lib.rs`:

```rust
//! Bevy visualization for CortenForge physics simulation.
//!
//! This crate is Layer 1 - it depends on Bevy and provides visualization
//! for the Layer 0 simulation crates.

pub mod prelude {
    // Re-exports will go here
}
```

### Phase 6: Move Documentation

```bash
git mv docs/SIM_BEVY_IMPLEMENTATION_PLAN.md sim/docs/
```

### Phase 7: Update ARCHITECTURE.md

Update the crate hierarchy diagram to reflect new paths:

```markdown
## Crate Hierarchy

```
sim/L0/
├── types      (L0)  ─────►  Pure data types, zero dependencies
├── simd       (L0)  ─────►  SIMD-optimized batch operations
├── contact    (L0)  ─────►  Compliant contact model, friction, solver
├── constraint (L0)  ─────►  Joint constraints, PGS/Newton/CG solvers
├── core       (L0)  ─────►  Integrators, world, stepper, collision detection
├── sensor     (L0)  ─────►  IMU, force/torque, touch, rangefinder
├── deformable (L0)  ─────►  XPBD soft bodies, cloth, ropes
├── muscle     (L0)  ─────►  Hill-type muscle models
├── tendon     (L0)  ─────►  Cable/tendon routing and actuation
├── mjcf       (L0)  ─────►  MuJoCo XML/binary format parser
├── urdf       (L0)  ─────►  URDF robot description parser
└── physics    (L0)  ─────►  Unified API combining all L0 crates

sim/L1/
└── bevy       (L1)  ─────►  Bevy integration layer
```
```

### Phase 8: Verify Build

```bash
cargo check --workspace
cargo test --workspace
cargo clippy --workspace -- -D warnings
```

### Phase 9: Commit

```bash
git add -A
git commit -m "refactor: restructure sim/ into L0/L1 layers

- Move all Bevy-free crates to sim/L0/
- Create sim/L1/ for Bevy-dependent crates
- Add sim/docs/ for simulation documentation
- Prepare structure for sim-bevy implementation

This enforces the architectural boundary between headless
simulation (L0) and visualization (L1).

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Dependency Path Reference

After migration, here's how paths work:

| From | To | Path |
|------|-----|------|
| L0 crate | Another L0 crate | `../other` |
| L1 crate | L0 crate | `../../L0/other` |
| L1 crate | Another L1 crate | `../other` |

---

## Validation Checklist

- [ ] All crates compile: `cargo check --workspace`
- [ ] All tests pass: `cargo test --workspace`
- [ ] No clippy warnings: `cargo clippy --workspace -- -D warnings`
- [ ] Documentation builds: `cargo doc --workspace --no-deps`
- [ ] No Bevy dependencies in any L0 crate
- [ ] ARCHITECTURE.md updated with new paths
- [ ] Workspace Cargo.toml updated

---

## Rollback Plan

If issues arise:

```bash
git reset --hard HEAD~1
```

Or selectively:

```bash
git mv sim/L0/types sim/sim-types
# ... repeat for each crate
```

---

## Notes

- **Crate names unchanged:** `sim-core`, `sim-bevy`, etc. remain the same. Only directory paths change.
- **No code changes:** This is purely a structural refactor. No Rust code modifications required (only Cargo.toml paths).
- **Git history preserved:** Using `git mv` preserves file history.
- **CI updates:** If CI references specific paths, those will need updating.
