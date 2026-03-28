# Equality Constraints Examples Spec

**Status:** Draft
**Date:** 2026-03-28
**Domain:** `examples/fundamentals/sim-cpu/equality-constraints/`

## Goal

Demonstrate the three spatial equality constraint types — Connect, Weld,
and Distance — with clear visual behavior and quantitative validation.
Each example isolates one constraint type. A fourth example (joint-coupling)
covers the joint equality constraint, which couples DOFs rather than
locking spatial positions.

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

### 1. `connect/` — Ball-and-Socket (3-DOF Position Lock)

A free-floating body connected to the world at a fixed point via
`<connect>`. The body swings like a pendulum — position is locked at
the anchor, but rotation is free. A second body is connected to the
first body (body-to-body connect), creating a two-link pendulum chain
using only equality constraints (no joints).

**MJCF sketch:**
```xml
<worldbody>
  <!-- Link 1: free body connected to world at (0, 0, 2) -->
  <body name="link1" pos="0 0 2">
    <freejoint/>
    <geom name="rod1" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" mass="1"/>
    <geom name="ball1" type="sphere" pos="0 0 -0.5" size="0.06" mass="0.5"/>
  </body>

  <!-- Link 2: free body connected to link1 at link1's tip -->
  <body name="link2" pos="0 0 1.3">
    <freejoint/>
    <geom name="rod2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.025" mass="0.8"/>
    <geom name="ball2" type="sphere" pos="0 0 -0.4" size="0.05" mass="0.3"/>
  </body>
</worldbody>

<equality>
  <!-- Link1 pivot locked to world at body origin -->
  <connect body1="link1" anchor="0 0 0"/>
  <!-- Link2 top locked to link1 bottom -->
  <connect body1="link1" body2="link2" anchor="0 0 -0.5"/>
</equality>
```

**What you see:** Two capsule-sphere links swinging freely from a fixed
point — a double pendulum built entirely from equality constraints
instead of joints. The links rotate freely at each connection point
(ball-and-socket behavior).

**Concepts covered:** `<connect>` to world, `<connect>` body-to-body,
anchor point placement, penalty constraint violation, free joints +
equality constraints as alternative to hinge/ball joints.

**Pass/fail (report at 5s):**

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Pivot 1 anchored | link1 origin stays at (0,0,2) | < 5mm position error |
| Pivot 2 attached | link1 tip stays at link2 origin | < 5mm position error |
| Rotation free | link1 angular velocity non-zero | > 0.1 rad/s |
| Energy bounded | total energy doesn't grow | < 5% of initial |

**HUD:** constraint violations (mm), angular velocities, energy.

---

### 2. `weld/` — Rigid Glue (6-DOF Pose Lock)

Two free-floating bodies welded together — they move as a single rigid
unit when an external force is applied. A third body is welded to the
world (fully fixed in space despite gravity). Demonstrates the
difference between weld (locks rotation) and connect (allows rotation).

**MJCF sketch:**
```xml
<worldbody>
  <!-- Pair A: two bodies welded together, free to fall -->
  <body name="base" pos="-0.5 0 1.5">
    <freejoint/>
    <geom name="base_box" type="box" size="0.12 0.08 0.08" mass="1.0"/>
  </body>
  <body name="arm" pos="-0.5 0 1.2">
    <freejoint/>
    <geom name="arm_cap" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04" mass="0.5"/>
  </body>

  <!-- Body B: welded to world (immovable) -->
  <body name="fixed" pos="0.5 0 1.0">
    <freejoint/>
    <geom name="fixed_sphere" type="sphere" size="0.1" mass="1.0"/>
  </body>

  <!-- Ground for the falling pair to land on -->
  <geom name="ground" type="plane" size="2 2 0.01"/>
</worldbody>

<equality>
  <weld body1="base" body2="arm"/>
  <weld body1="fixed"/>
</equality>
```

**What you see:** Left side — a box+capsule welded pair falls together
as one rigid object and lands on the ground. Right side — a sphere
hangs in midair, fully fixed to the world despite gravity.

**Concepts covered:** `<weld>` body-to-body, `<weld>` to world, 6-DOF
pose lock (position + orientation), penalty stiffness visible as slight
flex on impact.

**Pass/fail (report at 5s):**

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Weld pose locked | base↔arm relative pose constant | position < 2mm, angle < 0.05 rad |
| Fixed in space | fixed body stays at (0.5, 0, 1) | < 2mm displacement |
| Pair falls together | base and arm z-velocities match | < 1% relative difference during freefall |
| Pair lands | base z stabilizes on ground | z within 5mm of rest height |

**HUD:** weld violation (mm, rad), fixed body displacement, energy.

---

### 3. `distance/` — Rigid Rod (1-DOF Scalar Constraint)

Two free-floating spheres with a distance constraint — they maintain
fixed separation like an invisible rigid rod. One sphere starts above
the other; gravity pulls them down while the constraint keeps them
exactly 0.5m apart. The spheres can rotate freely and orbit each other,
but the center-to-center distance is locked.

**MJCF sketch:**
```xml
<option gravity="0 0 -9.81" timestep="0.002">
  <flag energy="enable"/>
</option>

<worldbody>
  <geom name="ground" type="plane" size="2 2 0.01"/>

  <body name="sphere_a" pos="0 0 1.5">
    <freejoint/>
    <geom name="ga" type="sphere" size="0.08" mass="1.0"/>
  </body>
  <body name="sphere_b" pos="0 0 1.0">
    <freejoint/>
    <geom name="gb" type="sphere" size="0.06" mass="0.5"/>
  </body>
</worldbody>

<equality>
  <distance geom1="ga" geom2="gb" distance="0.5"/>
</equality>
```

Initial conditions: give sphere_a a lateral velocity (`qvel[0] = 1.0`)
so the pair tumbles and orbits rather than falling straight down.

**What you see:** Two spheres connected by an invisible rod — they fall,
bounce off the ground, and tumble while maintaining constant separation.
The heavier sphere stays lower; the lighter one orbits around it.

**Concepts covered:** `<distance>` constraint, geom-to-geom distance
lock, 1-DOF scalar constraint vs 3-DOF/6-DOF spatial constraints,
initial velocity for interesting dynamics.

**Pass/fail (report at 5s):**

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Distance maintained | \|p1 - p2\| stays at 0.5m | < 5mm deviation |
| Both move | sphere_a velocity non-zero | > 0.01 m/s |
| Different masses | sphere_b moves more than sphere_a | velocity ratio ≈ m_a/m_b |
| Energy bounded | total energy doesn't grow | < 5% of initial |

**HUD:** measured distance, target distance, distance error (mm), energy.

---

### 4. `joint-coupling/` — Gear Trains and Mimic Joints

Two articulated mechanisms demonstrating joint equality constraints:

**(A) Mimic (1:1):** Two hinge arms side by side with `polycoef="0 1"`.
Moving one arm forces the other to follow — a mimic joint. Both arms
start displaced at different angles; the constraint pulls them together.

**(B) Gear (2:1):** Two hinge arms with `polycoef="0 2"`. The second
joint moves at twice the rate of the first — a 2:1 gear ratio. A motor
actuator drives the first joint slowly; the second follows at double
speed.

**MJCF sketch:**
```xml
<worldbody>
  <!-- Mimic pair (left) -->
  <body name="arm_a1" pos="-0.8 0 1.5">
    <joint name="j_a1" type="hinge" axis="0 1 0" damping="0.5"/>
    <geom name="arm_a1" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
  </body>
  <body name="arm_a2" pos="-0.4 0 1.5">
    <joint name="j_a2" type="hinge" axis="0 1 0" damping="0.5"/>
    <geom name="arm_a2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
  </body>

  <!-- Gear pair (right) -->
  <body name="arm_b1" pos="0.4 0 1.5">
    <joint name="j_b1" type="hinge" axis="0 1 0" damping="0.5"/>
    <geom name="arm_b1" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
  </body>
  <body name="arm_b2" pos="0.8 0 1.5">
    <joint name="j_b2" type="hinge" axis="0 1 0" damping="0.5"/>
    <geom name="arm_b2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
  </body>
</worldbody>

<actuator>
  <motor name="drive" joint="j_b1" gear="1"/>
</actuator>

<equality>
  <joint joint1="j_a1" joint2="j_a2" polycoef="0 1" solref="0.05 1.0"/>
  <joint joint1="j_b1" joint2="j_b2" polycoef="0 2" solref="0.05 1.0"/>
</equality>
```

Control: sinusoidal signal on the motor (`ctrl[0] = 2.0 * sin(2πt)`).
Mimic pair: start with `qpos[j_a1] = 0.5, qpos[j_a2] = -0.3` to show
convergence.

**What you see:** Left — two arms that start at different angles, then
snap together and swing in unison (mimic). Right — two arms where the
right one swings at double the angle of the left (2:1 gear), driven by
a motor.

**Concepts covered:** `<joint>` equality constraint, `polycoef`
polynomial coupling, mimic joints (1:1), gear ratios (2:1), motor
driving through a constraint, softer `solref` for stability.

**Pass/fail (report at 5s):**

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Mimic converges | \|q_a1 - q_a2\| → 0 after 1s | < 0.05 rad |
| Mimic tracks | \|q_a1 - q_a2\| stays small | < 0.05 rad after convergence |
| Gear ratio | q_b2 ≈ 2 * q_b1 | < 0.1 rad error |
| Motor drives | j_b1 follows sinusoidal input | oscillation visible |

**HUD:** joint angles (all 4), mimic error, gear ratio error, motor torque.

---

## Package Naming

| Example | Package name |
|---------|-------------|
| `connect/` | `example-equality-connect` |
| `weld/` | `example-equality-weld` |
| `distance/` | `example-equality-distance` |
| `joint-coupling/` | `example-equality-joint-coupling` |

## Implementation Notes

1. **Softer solref recommended.** MuJoCo's default `solref=[0.02, 1.0]`
   is tuned for their implicit solver. Use `solref="0.05 1.0"` for
   stability with our explicit penalty integration.

2. **Constraint violation is expected.** Penalty-method constraints have
   small but non-zero violation. This is NOT a bug — it's the physics.
   The examples should display violation magnitude so users understand
   the tradeoff (stiffness vs stability).

3. **efc_force access.** Equality constraint rows occupy `efc_*[0..ne)`.
   For connect: 3 rows, weld: 6 rows, distance: 1 row, joint: 1 row.
   Display constraint forces in the HUD to show the penalty method
   at work.

4. **Free joints required.** Connect, weld, and distance operate on
   bodies — the bodies need free joints to have DOFs for the constraint
   to act on.

## Build Order

1. `connect/` — simplest spatial constraint (3 DOF)
2. `weld/` — extends to 6 DOF, reuses camera/material patterns
3. `distance/` — scalar constraint, different visual (no rod geom)
4. `joint-coupling/` — different mechanism (joint-space, not body-space)
