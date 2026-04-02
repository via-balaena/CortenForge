# Sensors Advanced — Remaining Sensor Types

**Status:** Draft
**Date:** 2026-04-02
**Parent:** `examples/COVERAGE_SPEC.md` §7
**Depends on:** Track 1A `sensors/` (9 examples), `joint-limits/`, `tendons/`

## Purpose

Cover every sensor type that lacks a dedicated example. The Track 1A
`sensors/` gallery covers 16 of the 31 non-plugin types. The Track 1B
`joint-limits/` and `tendons/` directories added coverage for JointLimitFrc,
TendonPos, TendonVel, and TendonLimitFrc. This directory fills the remaining
**11 sensor types** plus deeper coverage of 4 more (Frame*Axis, Magnetometer,
SubtreeCom) via the stress-test.

## Coverage Map

### Already covered (do NOT duplicate)

| Sensor Type | Covered In |
|-------------|-----------|
| Clock | sensors/clock |
| JointPos, JointVel | sensors/joint-pos-vel |
| FramePos, FrameQuat | sensors/frame-pos-quat |
| SubtreeCom | sensors/subtree-com (basic) |
| Gyro, Velocimeter | sensors/gyro-velocimeter |
| Accelerometer | sensors/accelerometer |
| Touch | sensors/touch |
| ActuatorFrc, JointActuatorFrc | sensors/actuator-force |
| GeomDist, GeomNormal, GeomFromTo | sensors/geom-distance |
| BallQuat, BallAngVel | ball-joint/ examples |
| JointLimitFrc | joint-limits/ |
| TendonPos, TendonVel | tendons/ |
| TendonLimitFrc | tendons/tendon-limits |

### Covered by this directory

| Sensor Type | Example | Pipeline Stage |
|-------------|---------|---------------|
| FrameLinVel, FrameAngVel | frame-velocity | Velocity |
| FrameLinAcc, FrameAngAcc | frame-acceleration | Acceleration |
| Force, Torque | force-torque | Acceleration |
| Rangefinder | rangefinder | Position |
| SubtreeLinVel | subtree-velocity | Velocity |
| SubtreeAngMom | subtree-angmom | Velocity |
| ActuatorPos, ActuatorVel | actuator-pos-vel | Position + Velocity |
| FrameXAxis, FrameYAxis, FrameZAxis | stress-test only | Position |
| Magnetometer | stress-test only | Position |

**Total after this directory:** 31/31 non-plugin sensor types covered
(User and Plugin are intentionally excluded — they require the plugin system).

---

## Directory Layout

```
sensors-advanced/
├── SENSORS_ADVANCED_SPEC.md    (this file)
├── README.md
├── frame-velocity/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── frame-acceleration/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── force-torque/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── rangefinder/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── subtree-velocity/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── subtree-angmom/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── actuator-pos-vel/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
└── stress-test/
    ├── Cargo.toml
    ├── README.md
    └── src/main.rs
```

---

## Example 1: `frame-velocity/` — FrameLinVel, FrameAngVel

### Concept

Frame velocity sensors measure linear and angular velocity at a site, body,
or geom frame. FrameLinVel gives the 3D linear velocity of the frame origin.
FrameAngVel gives the 3D angular velocity of the parent body. Both are
velocity-stage sensors (read from `cvel`).

### Physics setup

A horizontal rod (capsule, L=0.5 m, m=1.0 kg) on a hinge joint at the
world origin, axis = Z. A velocity-servo motor drives it at constant
ω = 2π rad/s (1 revolution per second). A site is attached at the rod tip
(pos="0.5 0 0" in body frame).

Two sensors:
- `<framelinvel name="tip_linvel" objtype="site" objname="tip"/>`
- `<frameangvel name="tip_angvel" objtype="site" objname="tip"/>`

### Analytical predictions

- **FrameAngVel:** [0, 0, 2π] rad/s constant (body spins about Z at ω).
- **FrameLinVel:** v = ω × r. The tip traces a circle of radius 0.5 m in
  the XY plane. |v| = ω × R = 2π × 0.5 = π ≈ 3.14159 m/s. Direction
  rotates in XY plane: v(t) = [-πsin(2πt), πcos(2πt), 0].

### HUD display

```
Frame Velocity
  ω (angvel)    [  0.000,  0.000,  6.283] rad/s
  v (linvel)    [ -1.234,  2.987,  0.000] m/s
  |v|            3.1416  m/s
  expected |v|   3.1416  m/s
  error          0.00%
```

### Validation checks (t=15s)

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| AngVel Z-component = 2π | mean |ω_z − 2π| | < 0.5% |
| AngVel XY ≈ 0 | max |ω_x|, |ω_y| | < 0.01 rad/s |
| LinVel magnitude = π | mean ||v| − π| | < 0.5% |
| LinVel Z ≈ 0 | max |v_z| | < 0.01 m/s |

---

## Example 2: `frame-acceleration/` — FrameLinAcc, FrameAngAcc

### Concept

Frame acceleration sensors measure linear and angular acceleration at a
frame. FrameLinAcc **includes gravity as pseudo-acceleration** — a body at
rest reads [0, 0, +g], identical to a real accelerometer. FrameAngAcc
measures angular acceleration (α = dω/dt). Both are acceleration-stage
sensors (read from `cacc`).

### Physics setup

A pendulum arm (capsule, L=0.5 m, m=1.0 kg) on a hinge joint, axis = Y.
Site at the arm tip with both sensors attached.

Two phases:
1. **Rest phase (t=0–5s):** Motor holds the arm horizontal (position servo,
   kp=500). Body is stationary. FrameLinAcc = [0, 0, +g]. FrameAngAcc = 0.
2. **Release phase (t=5–15s):** Motor disengages (ctrl=0, gainprm=0). Arm
   swings as a pendulum under gravity. FrameLinAcc gains centripetal and
   tangential components. FrameAngAcc = τ/I = mg(L/2)sin(θ)/I.

Two sensors:
- `<framelinacc name="tip_linacc" objtype="site" objname="tip"/>`
- `<frameangacc name="tip_angacc" objtype="site" objname="tip"/>`

### Analytical predictions

**Rest (t < 5s):**
- FrameLinAcc = [0, 0, +9.81] m/s² (gravity pseudo-acceleration).
- FrameAngAcc = [0, 0, 0] rad/s² (no angular acceleration).

**Swinging (t > 5s):**
- FrameAngAcc about Y = α = mgL/(2I) × sin(θ), where I = mL²/3 for a
  uniform rod about one end. So α = (3g/2L)sin(θ).
- FrameLinAcc has centripetal (ω²R toward hinge) + tangential (αR
  perpendicular) + gravity pseudo-acceleration components.

### HUD display

```
Frame Acceleration
  phase          REST (motor holding)
  a (linacc)    [  0.000,  0.000,  9.810] m/s²
  α (angacc)    [  0.000,  0.000,  0.000] rad/s²
  |a|            9.810  m/s²
  θ              0.000  rad
```

### Validation checks (t=15s)

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Rest: LinAcc ≈ [0,0,+g] | max ‖a − [0,0,g]‖ during t=1–4s | < 0.5% of g |
| Rest: AngAcc ≈ 0 | max ‖α‖ during t=1–4s | < 0.01 rad/s² |
| Swing: AngAcc sign flips with θ | sign(α_Y) = sign(sin(θ)) after release | 0 violations |
| Swing: |a| > g at bottom | centripetal adds to gravity | verified |

---

## Example 3: `force-torque/` — Force, Torque

### Concept

Force and Torque sensors measure the 3D internal (reaction) force and torque
at a site. They read from `cfrc_int` — the sum of inertial, constraint, and
Coriolis forces on the body, expressed at the sensor site. These are
acceleration-stage sensors.

For a static body, Force reads the reaction force (equal and opposite to
applied loads), and Torque reads the reaction moment.

### Physics setup

A cantilevered beam: horizontal capsule (L=0.5 m, m=1.0 kg) on a hinge
joint at the world origin, axis = Y. A strong position-servo motor holds
it horizontal (θ=0). A site at the hinge base has both sensors.

Two sensors:
- `<force name="base_force" site="base_site"/>`
- `<torque name="base_torque" site="base_site"/>`

### Analytical predictions (statics)

The beam is uniform, COM at L/2 from hinge.

- **Force:** Reaction supports weight. In world frame: F = [0, 0, +mg] =
  [0, 0, +9.81] N (upward Z). In site frame (aligned with world at θ=0):
  same values.
- **Torque:** Reaction resists gravity torque. τ = mg × (L/2) about Y axis.
  τ_Y = 1.0 × 9.81 × 0.25 = 2.4525 N·m.

Note: the sensor reads `cfrc_int` which is the total internal force on the
body (inertial + constraint). For a static body this equals the reaction
force from the joint constraint.

### HUD display

```
Force–Torque (static beam)
  Force       [  0.000,  0.000,  9.810] N
  Torque      [  0.000, -2.453,  0.000] N·m
  expected F  [  0.000,  0.000,  9.810] N
  expected τ  [  0.000, -2.453,  0.000] N·m
```

### Validation checks (t=15s)

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Force Z = mg | mean F_z during t=2–14s | < 1% of mg |
| Force XY ≈ 0 | max |F_x|, |F_y| | < 0.05 N |
| Torque magnitude = mgL/2 | mean |τ_Y| during t=2–14s | < 1% |
| Torque XZ ≈ 0 | max |τ_x|, |τ_z| | < 0.05 N·m |

---

## Example 4: `rangefinder/` — Rangefinder

### Concept

A rangefinder sensor casts a ray along the site's +Z axis and returns the
distance to the nearest geometry. Returns −1 when the ray hits nothing
within range. The sensor excludes geoms on its own parent body (no
self-intersection). Position-stage sensor.

### Physics setup

A body on a vertical slide joint (axis = Z) with a site pointing straight
down (site frame Z = world −Z). A ground plane sits at z=0. The body
oscillates via a sinusoidal position servo: z(t) = 1.0 + 0.5 sin(2πt/4)
(period 4s, range 0.5–1.5 m above ground).

A second site on the same body points horizontally (+X direction) toward
empty space — should always read −1.

Two sensors:
- `<rangefinder name="down" site="eye_down"/>`
- `<rangefinder name="side" site="eye_side"/>`

### Analytical predictions

- **down:** Distance = body z-position (height above ground plane). Tracks
  the sinusoidal motion: d(t) = 1.0 + 0.5 sin(2πt/4).
- **side:** −1 (no geometry in +X direction from body).

### HUD display

```
Rangefinder
  ↓ down         1.250  m
  → side         -1     (no hit)
  height (qpos)  1.250  m
  error          0.000  m
```

### Validation checks (t=15s)

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Down = height | max |rangefinder − qpos| over 15s | < 0.1% |
| Side = −1 | every frame reads exactly −1 | 0 violations |
| Range tracks oscillation | rangefinder min ≈ 0.5, max ≈ 1.5 | < 1% |

---

## Example 5: `subtree-velocity/` — SubtreeLinVel

### Concept

SubtreeLinVel gives the linear velocity of a subtree's center of mass. It
aggregates momentum across all bodies in the kinematic subtree rooted at the
sensor's body, then divides by total subtree mass. Velocity-stage sensor.
SubtreeCom (position-stage, covered in Track 1A) is included in the HUD for
context — both sensors read from the same subtree, showing position and
velocity of the composite COM.

### Physics setup

A 3-link chain (3 hinge joints, axis = Y) with different masses
(m1=2.0, m2=1.0, m3=0.5 kg) released from rest in gravity. The entire
chain free-falls as a unit. Sensor on body 1 (root of subtree) measures the
aggregate over all 3 links.

Two sensors:
- `<subtreecom name="sub_com" body="link1"/>`
- `<subtreelinvel name="sub_vel" body="link1"/>`

### Analytical predictions

- SubtreeLinVel: v_z(t) = −g×t (linear ramp, all links fall together).
  At t=1s: v_z = −9.81 m/s. At t=2s: v_z = −19.62 m/s.
- Horizontal components ≈ 0 (no horizontal forces, joints only swing in Y).
- SubtreeCom descends parabolically: Δz = −½gt².

### HUD display

```
Subtree Velocity (3-link chain, free fall)
  COM            [  0.000,  0.000,  0.834] m
  v_com          [  0.000,  0.000, -4.905] m/s
  expected v_z   -4.905  m/s
  error          0.00%
```

### Validation checks (t=15s)

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| v_z = −g×t | sample at t=2s: v_z ≈ −19.62 | < 0.5% |
| v_xy ≈ 0 | max |v_x|, |v_y| during 15s | < 0.01 m/s |
| COM descends | com_z(3s) < com_z(0s) | verified |

---

## Example 6: `subtree-angmom/` — SubtreeAngMom

### Concept

SubtreeAngMom gives the angular momentum of a kinematic subtree about its
own center of mass. In the absence of external torques (zero gravity, no
contact), angular momentum is exactly conserved — a fundamental invariant.
Velocity-stage sensor.

### Physics setup

A 3-link chain (3 hinge joints, axis = Y) with different masses
(m1=2.0, m2=1.0, m3=0.5 kg) in zero gravity (`gravity="0 0 0"`). Initial
joint velocities give the chain a spinning motion. No contacts, no
actuators — pure conservation test.

Two sensors:
- `<subtreeangmom name="sub_angmom" body="link1"/>`
- `<subtreecom name="sub_com" body="link1"/>` (for HUD context)

### Analytical predictions

- SubtreeAngMom: constant vector over the full 15s run. The individual
  links exchange spin and orbital angular momentum as the chain flexes,
  but the total about the subtree COM is invariant.
- |L| at t=0 = |L| at t=15s to machine precision.

### HUD display

```
Subtree Angular Momentum (3-link chain, zero-g)
  L (angmom)    [  0.000,  0.347,  0.000] kg*m^2/s
  |L|            0.347  kg*m^2/s
  |L| at t=0     0.347  kg*m^2/s
  drift          0.00e+00
```

### Validation checks (t=15s)

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| |L| conserved | max |ΔL|/|L₀| over 15s | < 1e-8 |
| L direction stable | max angle between L(t) and L(0) | < 1e-6 rad |

---

## Example 7: `actuator-pos-vel/` — ActuatorPos, ActuatorVel

### Concept

ActuatorPos and ActuatorVel sensors report the actuator's generalized
position and velocity — the quantity the actuator "sees" through its
transmission. For a joint-transmission actuator: ActuatorPos = gear × qpos,
ActuatorVel = gear × qvel. These sensors are useful for closed-loop control
where the actuator's own coordinate matters more than the joint coordinate.

### Physics setup

A hinge pendulum (L=0.5 m, m=1.0 kg) with a motor using joint transmission
(gear=2.0). The motor drives a sinusoidal position: ctrl(t) = 0.5 sin(2πt/3).

Two sensors:
- `<actuatorpos name="act_pos" actuator="motor"/>`
- `<actuatorvel name="act_vel" actuator="motor"/>`

Plus the corresponding joint sensors for comparison:
- `<jointpos name="jnt_pos" joint="hinge"/>`
- `<jointvel name="jnt_vel" joint="hinge"/>`

### Analytical predictions

- **ActuatorPos** = gear × qpos = 2.0 × qpos. The HUD shows both and
  verifies the ratio.
- **ActuatorVel** = gear × qvel = 2.0 × qvel. Same gear ratio applies to
  velocity.
- When qpos tracks the sinusoid: ActuatorPos oscillates with amplitude
  2.0 × 0.5 = 1.0 rad, ActuatorVel peaks at 2.0 × 0.5 × (2π/3) ≈ 2.09 rad/s.

### HUD display

```
Actuator Position & Velocity
  ActuatorPos    0.873  rad
  JointPos       0.437  rad
  ratio          2.000  (gear)
  ActuatorVel    1.047  rad/s
  JointVel       0.524  rad/s
  ratio          2.000  (gear)
```

### Validation checks (t=15s)

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| ActuatorPos = gear × JointPos | max |act_pos − 2×jnt_pos| | < 1e-10 |
| ActuatorVel = gear × JointVel | max |act_vel − 2×jnt_vel| | < 1e-10 |
| Gear ratio constant | ratio never deviates from 2.0 | < 1e-10 |

---

## Example 8: `stress-test/` — Headless Validation

### Concept

Headless (no Bevy, no window) validation of all sensor types covered in
this directory, plus Frame*Axis and Magnetometer which don't warrant visual
examples. Each check is a self-contained simulation with its own MJCF model.

### Checks (24)

**FrameLinVel (3 checks)**

1. **Spinning tip v = ωR:** Rod on hinge (ω=2π, R=0.5). Run 2s. Sample
   |FrameLinVel| at tip site. Expected: π m/s. Tolerance: 0.5%.

2. **Stationary body v=0:** Body held by motor. FrameLinVel ≈ 0. Tolerance:
   1e-8 m/s.

3. **Free-fall v = gt:** Free body dropped. FrameLinVel Z at t=1s = −9.81.
   Tolerance: 0.5%.

**FrameAngVel (2 checks)**

4. **Constant spin ω:** Rod on hinge (ω=2π). FrameAngVel Z = 2π. XY ≈ 0.
   Tolerance: 0.5%.

5. **At rest ω=0:** Stationary body. FrameAngVel = [0,0,0]. Tolerance:
   1e-8 rad/s.

**FrameLinAcc (3 checks)**

6. **At rest a = [0,0,+g]:** Stationary body on hinge. FrameLinAcc Z = +9.81.
   Tolerance: 0.5%.

7. **Free fall a ≈ 0 (weightless):** Free body dropped. FrameLinAcc ≈ 0
   (gravity pseudo-acceleration cancels with actual fall). Tolerance: 0.01
   m/s² (solver noise).

8. **Centripetal at bottom of swing:** Pendulum released from horizontal.
   At bottom: |a| > g (centripetal adds to gravity). Verified.

**FrameAngAcc (2 checks)**

9. **Constant ω → α=0:** Hinge with velocity servo at constant ω.
   FrameAngAcc ≈ 0. Tolerance: 0.1 rad/s².

10. **Torque pulse → α=τ/I:** Apply known torque to free hinge. Measure
    FrameAngAcc. Expected: α = τ/I. Tolerance: 1%.

**Force and Torque (3 checks)**

11. **Static beam F = [0,0,mg]:** Horizontal beam held by motor. Force Z =
    mg = 9.81 N. Tolerance: 1%.

12. **Static beam τ = mgL/2:** Same beam. Torque magnitude = 2.4525 N·m.
    Tolerance: 1%.

13. **Free body F ≈ 0:** Free-floating body in zero-g. No constraints → no
    internal force. Tolerance: 1e-8 N.

**Rangefinder (3 checks)**

14. **Distance = height:** Body on slide joint above plane. Rangefinder =
    qpos (height). Tolerance: 0.1%.

15. **No hit = −1:** Site points away from all geometry. Returns −1 exactly.

16. **Self-exclusion:** Site on a body with geoms. Ray passes through own
    geoms without registering a hit. Verified via setup where only own geoms
    are in the ray path.

**SubtreeLinVel (2 checks)**

17. **Free-fall v = gt:** 3-link chain dropped. SubtreeLinVel Z at t=1s =
    −9.81 m/s. Tolerance: 0.5%.

18. **Zero-g constant v:** Chain in zero-g with initial velocity.
    SubtreeLinVel constant over 5s. Tolerance: 1e-10 m/s.

**SubtreeAngMom (1 check)**

19. **Conservation in zero-g:** Chain in zero-g with initial angular
    velocity. |SubtreeAngMom| constant over 10s. Tolerance: 1e-8.

**ActuatorPos, ActuatorVel (2 checks)**

20. **Joint transmission:** ActuatorPos = gear × qpos exactly. Tolerance:
    1e-12.

21. **Velocity matches:** ActuatorVel = gear × qvel exactly. Tolerance:
    1e-12.

**FrameXAxis, FrameYAxis, FrameZAxis (1 check)**

22. **Axes match rotation matrix:** Attach all three axis sensors to a site
    on a spinning body. Verify each axis vector matches the corresponding
    column of the site's rotation matrix (from `site_xmat`). Tolerance:
    1e-10.

**Magnetometer (1 check)**

23. **Field in sensor frame:** Set `<option magnetic="0 0 -0.5"/>`. Attach
    magnetometer to a site on a rotating body. Verify the reading rotates
    with the body (magnitude preserved, direction tracks inverse rotation).
    |B| = 0.5 T constant. Tolerance: 1e-10.

**Sensor dimensions (1 check)**

24. **All dimensions correct:** Load a model with one of each sensor type.
    Verify `sensor_dim` matches expected: scalar (1D) for JointPos etc.,
    3D for FrameLinVel etc., 4D for FrameQuat/BallQuat. Exhaustive check
    over all types.

---

## Deviations from COVERAGE_SPEC.md §7

The original spec listed 7 visual examples + stress-test. Two examples are
now redundant:

- **`tendon-sensors`** (TendonPos, TendonVel) → fully covered by
  `tendons/` Track 1B examples (8 examples, 16 stress-test checks).
- **`limit-forces`** (JointLimitFrc, TendonLimitFrc) → fully covered by
  `joint-limits/` (JointLimitFrc) and `tendons/tendon-limits/`
  (TendonLimitFrc).

Replaced with **`actuator-pos-vel`** (ActuatorPos, ActuatorVel) which was
not covered anywhere. Added Frame*Axis and Magnetometer to the stress-test
for completeness.

Split **`subtree-momentum`** into **`subtree-velocity`** + **`subtree-angmom`**
to enforce one concept per example — the original mixed two concepts that
require different physics conditions (gravity for linear velocity, zero-g
for angular momentum conservation).

**Net result:** 7 visual examples + 1 stress-test (24 checks). Same goal
(100% sensor coverage), cleaner separation, no redundancy.

---

## Implementation Order

1. stress-test (headless, validates all physics before visual work)
2. frame-velocity (simplest visual — spinning rod)
3. frame-acceleration (same rod, different sensors)
4. force-torque (static beam — clean analytical comparison)
5. rangefinder (slide joint + ray)
6. subtree-velocity (free-falling chain)
7. subtree-angmom (zero-g conservation)
8. actuator-pos-vel (gear ratio demo)

## Naming Convention

Package names follow the existing sensor pattern:
- `example-sensor-adv-frame-velocity`
- `example-sensor-adv-frame-acceleration`
- `example-sensor-adv-force-torque`
- `example-sensor-adv-rangefinder`
- `example-sensor-adv-subtree-velocity`
- `example-sensor-adv-subtree-angmom`
- `example-sensor-adv-actuator-pos-vel`
- `example-sensor-adv-stress-test`
