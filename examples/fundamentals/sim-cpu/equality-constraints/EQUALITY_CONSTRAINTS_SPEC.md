# Equality Constraints Examples Spec

**Status:** All 7 Bevy examples built and passing. Stress test 16/16.
**Date:** 2026-03-28
**Domain:** `examples/fundamentals/sim-cpu/equality-constraints/`

## Goal

Demonstrate the four equality constraint types — Connect, Weld, Distance,
and Joint — with one concept per example. Each example isolates a single
constraint configuration for clarity and easy debugging.

## Constraint Types

| Type | DOFs removed | What it does | Analogue |
|------|-------------|--------------|----------|
| **Connect** | 3 (translation) | Locks a point on body1 to body2 (or world). Rotation free. | Ball-and-socket joint |
| **Weld** | 6 (translation + rotation) | Locks full pose of body1 relative to body2 (or world). | Rigid glue / fixed joint |
| **Distance** | 1 (scalar) | Maintains fixed distance between two geom centers. | Rigid rod (no bending constraint) |
| **Joint** | 1 per constraint | Couples joint positions: `q1 = poly(q2)`. | Gear train, mimic joint |

All four are penalty-method constraints with Baumgarte stabilization,
controlled by `solref` (stiffness/damping) and `solimp` (impedance).
Small but non-zero constraint violation is expected — this is inherent
to the penalty approach (vs. Lagrange multipliers).

## Examples

### 1. `connect-to-world/` — Single Body Locked to World

A free body connected to the world at the origin. Swings like a
ball-and-socket pendulum — position locked, rotation free.

**Checks (3):** pivot error < 5mm, rotation free, energy bounded.

---

### 2. `connect-body-to-body/` — Chained Double Pendulum

Two free bodies chained via connect constraints. Link1 → world,
link2 → link1 tip. Double pendulum using only equality constraints.

**Checks (4):** pivot 1 < 5mm, pivot 2 < 5mm, both rotate, energy bounded.

---

### 3. `weld-to-world/` — Immovable Body

A free body welded to the world. Despite gravity, 6-DOF pose lock
keeps it fixed in space (position AND orientation).

**Checks (2):** displacement < 2mm, orientation < 0.01 rad.

---

### 4. `weld-body-to-body/` — Rigid Glue

Two free bodies welded together. They fall as a single rigid unit
under gravity and land on the ground.

**Checks (3):** relative pos < 2mm, relative angle < 0.05 rad, pair falls together.

---

### 5. `distance/` — Rigid Rod (1-DOF Scalar Constraint)

Two free spheres with a distance constraint maintaining 0.5m separation.
Lateral velocity kick creates tumbling orbital dynamics.

**Checks (4):** distance < 5mm, both move, mass ratio effect, energy bounded.

---

### 6. `joint-mimic/` — 1:1 Joint Coupling

Two hinge arms coupled 1:1 via `polycoef="0 1"`. Start at different
angles (0.5, -0.3 rad) to show convergence, then swing in unison.

**Checks (2):** mimic converges by 1s, tracks < 0.05 rad.

---

### 7. `joint-gear/` — 2:1 Gear Ratio

Two hinge arms with `polycoef="0 2"`. Motor drives j1 with sinusoidal
signal; j2 follows at double the angle.

**Checks (2):** gear ratio < 0.1 rad error, motor oscillates.

---

## Package Naming

| Example | Package name |
|---------|-------------|
| `connect-to-world/` | `example-equality-connect-to-world` |
| `connect-body-to-body/` | `example-equality-connect-body-to-body` |
| `weld-to-world/` | `example-equality-weld-to-world` |
| `weld-body-to-body/` | `example-equality-weld-body-to-body` |
| `distance/` | `example-equality-distance` |
| `joint-mimic/` | `example-equality-joint-mimic` |
| `joint-gear/` | `example-equality-joint-gear` |

## Implementation Notes

1. **Stiff solref for spatial constraints.** Use `solref="0.005 1.0"` for
   connect and `solref="0.002 1.0"` to `0.005` for weld to keep penalty
   error under 2mm. Softer `solref="0.05 1.0"` for joint-space constraints.

2. **Constraint violation is expected.** Penalty-method constraints have
   small but non-zero violation. This is NOT a bug — it's the physics.
   The examples display violation magnitude so users understand the
   tradeoff (stiffness vs stability).

3. **efc_force access.** Equality constraint rows occupy `efc_*[0..ne)`.
   For connect: 3 rows, weld: 6 rows, distance: 1 row, joint: 1 row.

4. **Free joints required.** Connect, weld, and distance operate on
   bodies — the bodies need free joints to have DOFs for the constraint
   to act on.

## Build Order

1. `connect-to-world/` — simplest possible constraint
2. `connect-body-to-body/` — adds body-to-body chaining
3. `weld-to-world/` — 6-DOF lock to world
4. `weld-body-to-body/` — 6-DOF relative lock
5. `distance/` — scalar constraint, different visual
6. `joint-mimic/` — joint-space coupling (1:1)
7. `joint-gear/` — joint-space coupling (2:1) + motor
