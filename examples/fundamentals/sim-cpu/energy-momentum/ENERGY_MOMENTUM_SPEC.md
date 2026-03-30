# Energy-Momentum Examples Spec

**Status:** Draft — engine fix done, examples pending approval
**Date:** 2026-03-30
**Domain:** `examples/fundamentals/sim-cpu/energy-momentum/`

## Engine Fix (completed)

Tendon spring PE was missing from `mj_energy_pos()`. Added Phase 3: deadband
tendon spring energy (`0.5 * k * displacement^2`) with proper sleep filter
and `DISABLE_SPRING` gating. 3 new tests (T18–T20): classical spring,
deadband, zero stiffness. All pass.

## Goal

Demonstrate and validate energy and momentum conservation — the most
fundamental correctness properties of any physics engine. Each example
introduces exactly one new concept.

## Available APIs

- `ENABLE_ENERGY` flag (`<flag energy="enable"/>`)
- `data.energy_kinetic`, `data.energy_potential`, `data.total_energy()`
- `data.subtree_linvel[0]` — system COM velocity (linear momentum / total mass)
- `data.subtree_angmom[0]` — total angular momentum about system COM
- `data.subtree_mass[0]` — total system mass
- `ValidationHarness.track_energy()` — automated energy tracking

## Examples (1 concept each)

### 1. `free-flight/` — Conservation in Isolation

**New concept:** Energy and momentum conservation with zero external forces.

A free-floating rigid body in zero gravity with initial linear and angular
velocity. No contacts, no joints to world, no forces. All three conservation
laws must hold to machine precision.

**MJCF:** Single free body (box with asymmetric inertia), `gravity="0 0 0"`,
initial `qvel=[0.5, 0.3, 0, 0.2, 0.1, 0.4]`.

**What to show:**
- KE, linear momentum (mass * subtree_linvel[0]), angular momentum
  (subtree_angmom[0]) printed every 0.5s
- Drift as % deviation from initial values

**Pass/fail:**
- KE drift < 1e-10 over 10s
- Linear momentum drift < 1e-10 over 10s
- Angular momentum magnitude drift < 1e-8 over 10s

---

### 2. `pendulum-energy/` — Potential-Kinetic Exchange

**New concept:** Energy exchanges between kinetic and potential forms under
gravity, with total energy conserved.

An undamped hinge pendulum released from 45 degrees. KE is maximum at the
bottom, PE is maximum at the top. Total energy stays flat.

**MJCF:** Single hinge pendulum, `damping="0"`, `gravity="0 0 -9.81"`,
`<flag energy="enable"/>`. Initial qpos = pi/4.

**What to show:**
- KE and PE over time (oscillating, complementary)
- Total energy (flat line)
- HUD shows KE, PE, total, and KE/total ratio

**Pass/fail:**
- Total energy drift < 0.5% over 10s
- At bottom swing: KE > 0.9 * total
- At top swing: PE > 0.9 * total

---

### 3. `elastic-bounce/` — Conservation Through Contact

**New concept:** Energy conservation across a collision event.

A ball dropped from 0.5m onto a ground plane with `solref` tuned for
near-perfect restitution. Total energy (KE + PE) should be approximately
conserved through the bounce.

**MJCF:** Sphere (mass=1kg, r=0.05m) at height 0.5m. `gravity="0 0 -9.81"`.
Tuned `solref` for elastic bounce.

**What to show:**
- KE and PE tracked continuously
- Total energy printed every 0.5s
- Bounce height after first bounce vs drop height

**Pass/fail:**
- Total energy conserved to < 5% through first bounce
- Bounce height within 15% of drop height
- At least 3 visible bounces

---

### 4. `damped-decay/` — Energy Dissipation

**New concept:** Damping as the mechanism that removes energy from a system.

A damped hinge pendulum. Energy monotonically decreases over time — no
energy is ever gained. Contrast with the undamped case (example 2).

**MJCF:** Single hinge pendulum with `damping="0.5"`, initial qpos = pi/4.
`<flag energy="enable"/>`.

**What to show:**
- Total energy over time (exponentially decaying curve)
- Energy never increases step-to-step
- Final energy as % of initial

**Pass/fail:**
- Energy monotonically decreasing (no energy gain at any step)
- Final energy < 20% of initial after 10s
- Energy at t=5s < energy at t=0 (obvious dissipation)

---

### 5. `stress-test/` — Headless Validation

Comprehensive headless checks covering all conservation scenarios.
No rendering, no Bevy — pure sim-core.

**Checks:**

1. **Free-flight KE** — zero-gravity free body, KE drift < 1e-10 over 10s
2. **Free-flight linear momentum** — p = m*v constant to < 1e-10
3. **Free-flight angular momentum** — L magnitude constant to < 1e-8
4. **Free-flight velocity components** — each of 6 qvel constant to < 1e-10
5. **Pendulum energy conservation** — undamped, total drift < 0.5% over 10s
6. **Pendulum energy partition** — at bottom: KE > 0.9 * total
7. **Elastic bounce energy** — total energy through bounce < 5% loss
8. **Bounce height** — restitution > 0.85
9. **Damped monotonic** — energy never increases step-to-step
10. **Damped dissipation** — final energy < 20% of initial
11. **Energy flag disabled** — with flag off, energy fields stay 0.0
12. **Multi-body conservation** — 3 free bodies in zero-g, system KE conserved

## Concept Ladder

| # | Example | New concept | Builds on |
|---|---------|------------|-----------|
| 1 | free-flight | conservation (no forces) | — |
| 2 | pendulum-energy | PE ↔ KE exchange (gravity) | #1 |
| 3 | elastic-bounce | conservation through contact | #2 |
| 4 | damped-decay | dissipation (damping) | #2 |
| 5 | stress-test | headless validation of all | #1–#4 |

## Structure

```
energy-momentum/
  README.md
  free-flight/
    Cargo.toml
    src/main.rs         ~250 LOC
  pendulum-energy/
    Cargo.toml
    src/main.rs         ~250 LOC
  elastic-bounce/
    Cargo.toml
    src/main.rs         ~300 LOC
  damped-decay/
    Cargo.toml
    src/main.rs         ~250 LOC
  stress-test/
    Cargo.toml
    src/main.rs         ~400 LOC
```

5 examples, 12 stress-test checks.

## Naming Convention

| Example | Package Name |
|---------|-------------|
| free-flight | `example-energy-free-flight` |
| pendulum-energy | `example-energy-pendulum-energy` |
| elastic-bounce | `example-energy-elastic-bounce` |
| damped-decay | `example-energy-damped-decay` |
| stress-test | `example-energy-stress-test` |

## Dependencies

| Crate | Visual | Stress-test |
|-------|--------|-------------|
| sim-core | yes | yes |
| sim-mjcf | yes | yes |
| sim-bevy | yes | no |
| bevy | yes | no |
| nalgebra | if needed | yes |
