# Joint Limits — Examples Spec

## Overview

4 examples demonstrating joint limit constraints with solver tuning. Joint
limits constrain range of motion via one-sided constraints. The stiffness and
damping of the limit response are tuned via `solref` and `solimp` — the same
solver parameters used for contacts.

All MJCF is embedded as `const` strings (no external files). Visual examples
use sim-bevy with the standard ValidationHarness + HUD pattern. Stress-test
is headless (no Bevy).

## Directory structure

```
examples/fundamentals/sim-cpu/joint-limits/
├── README.md
├── JOINT_LIMITS_EXAMPLES_SPEC.md    (this file)
├── hinge-limits/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── slide-limits/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── ball-cone/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
└── stress-test/
    ├── Cargo.toml
    ├── README.md
    └── src/main.rs
```

Cargo package names: `example-joint-limits-{hinge-limits,slide-limits,ball-cone,stress-test}`

## MJCF attribute reference

- `limited="true"` — enables limit on joint
- `range="lo hi"` — limit range (degrees for hinge if `angle="degree"`, radians if `angle="radian"`, meters for slide)
- `solreflimit="timeconst dampratio"` — solver reference for limits (default: `0.02 1.0`)
- `solimplimit="d0 dwidth width midpoint power"` — solver impedance for limits (default: `0.9 0.95 0.001 0.5 2.0`)
- Sensor: `<jointlimitfrc joint="name"/>` — reports constraint force when at limit, 0 otherwise

---

## Example 1: `hinge-limits/` — Hinge Limit Tuning

### Concept

A pendulum with `range="-45 45"` degrees released from 60° (beyond limit).
Three pendulums side by side with different solver tuning show the effect of
solref parameters on limit response.

### MJCF model

Single `<mujoco>` with `compiler angle="degree"`. Three pendulums spaced
along X-axis, each hanging from a fixed pivot via hinge joint:

| Pendulum | solreflimit | Behavior |
|----------|-------------|----------|
| Stiff (blue) | `"0.005 1.0"` | Short timeconst → high stiffness. Bounces off limit sharply. |
| Default (green) | (none — uses default `0.02 1.0`) | Standard response. Moderate bounce. |
| Soft (red) | `"0.08 1.0"` | Long timeconst → low stiffness. Visible penetration past limit before correction. |

Each pendulum:
- Parent body at `pos="X 0 1.5"` with a fixed geom (small sphere for pivot visual)
- Child body with hinge joint: `axis="0 1 0"`, `limited="true"`, `range="-45 45"`
- Rod geom (capsule, length ~0.6) + tip sphere for mass
- Initial `qpos` = 60° (1.047 rad) — starts beyond the +45° limit

Sensors: `<jointlimitfrc>` for each joint.

### HUD

```
Joint Limits — Hinge Limit Tuning
──────────────────────────────────
Stiff (blue):  angle= 43.2°  limit_frc= 12.4  solref=[0.005, 1.0]
Default (grn): angle= 44.8°  limit_frc=  3.1  solref=[0.02, 1.0]
Soft (red):    angle= 47.1°  limit_frc=  0.8  solref=[0.08, 1.0]

time= 2.34
```

### Validation checks (t=5s, 3 checks)

1. **Stiff limit force > soft limit force** — at equilibrium, stiffer spring
   produces higher restoring force for the same penetration
2. **All angles within range ± 5°** — no joint exceeds limit by more than
   solver penetration tolerance
3. **JointLimitFrc > 0 when at limit** — all three sensors report nonzero
   force while pressed against limit

### Camera

Position: `(0, -2.5, 1.5)`, looking at `(0, 0, 1.0)`. Distance ~2.5.
All three pendulums visible from front.

---

## Example 2: `slide-limits/` — Slide Limit with Motor Force

### Concept

A mass on a horizontal rail with `range="-0.3 0.3"`. A motor applies constant
force pushing the mass into the positive limit. Shows that the limit holds
against motor force, and the `JointLimitFrc` sensor reports the reaction.

### MJCF model

```xml
<compiler angle="radian"/>
<option gravity="0 0 -9.81" timestep="0.002"/>
```

- Ground plane for visual reference
- Rail body fixed at height ~0.3m, visual capsule along X-axis
- Slider body with slide joint: `axis="1 0 0"`, `limited="true"`, `range="-0.3 0.3"`
- Box geom (mass ~1.0) as the slider
- Motor actuator: `<motor joint="slider" gear="1"/>` with constant `ctrl[0] = 5.0`
  (applied in setup or via `data.ctrl`)
- `<jointlimitfrc joint="slider"/>` sensor

### HUD

```
Joint Limits — Slide Limit + Motor
───────────────────────────────────
position=  0.300  (range: -0.30 .. 0.30)
motor_force=  5.0
limit_force=  5.02
at_limit= YES

time= 3.12
```

### Validation checks (t=5s, 3 checks)

1. **Position clamped at limit** — `qpos ≤ 0.3 + 0.01` (within solver tolerance)
2. **JointLimitFrc ≈ motor force** — at equilibrium, limit force balances motor
   (within 20% tolerance to account for gravity/friction components)
3. **Limit force > 0** — sensor reports positive value when at limit

### Camera

Position: `(0, -1.5, 0.8)`, looking at `(0, 0, 0.3)`. Side view of rail.

---

## Example 3: `ball-cone/` — Ball Joint Cone Limit

### Concept

A ball joint with `range="0 30"` (30° cone limit). A rod hangs from a ball
joint and is given initial angular velocity in different directions to push it
beyond the cone. The cone constraint is rotationally symmetric — the limit
activates at the same angle regardless of azimuthal direction.

### MJCF model

```xml
<compiler angle="degree"/>
<option gravity="0 0 -9.81" timestep="0.002"/>
```

- Pivot body at `(0, 0, 1.5)` (fixed)
- Rod body with ball joint: `limited="true"`, `range="0 30"`
- Rod geom: capsule, length ~0.5
- Tip sphere for visual mass
- Initial qpos: quaternion representing ~40° tilt (beyond 30° limit) in XZ plane
- `<jointlimitfrc joint="ball"/>` sensor

The single rod demonstrates the cone constraint. The HUD shows the current
deflection angle and limit force.

### HUD

```
Joint Limits — Ball Joint Cone (30°)
─────────────────────────────────────
deflection= 29.4°  (limit: 30°)
limit_force= 8.72
cone_active= YES

time= 1.84
```

### Validation checks (t=5s, 3 checks)

1. **Deflection ≤ 30° + 3°** — angle never exceeds cone limit by more than
   solver tolerance
2. **JointLimitFrc > 0 when deflected past limit** — sensor activates at cone
   boundary
3. **Cone is symmetric** — deflection angle (computed from quaternion) stays
   near limit regardless of which azimuthal direction the rod swings

### Camera

Position: `(1.5, -1.5, 1.8)`, looking at `(0, 0, 1.2)`. Slightly above and
to the side, viewing the cone from a 3/4 angle.

---

## Example 4: `stress-test/` — Headless Validation

### Concept

Headless binary with 12 focused checks covering all joint limit behaviors.
No Bevy dependency. Each check uses a minimal, purpose-built MJCF model.

### Check list

| # | Check | Model | Validation |
|---|-------|-------|------------|
| 1 | Hinge limit activates | Pendulum, range=±45°, start at 60° | After 1000 steps, angle ≤ 45° + tolerance |
| 2 | Slide limit activates | Slider, range=±0.3, start at 0.5 | After 1000 steps, pos ≤ 0.3 + tolerance |
| 3 | Ball cone limit activates | Ball joint, range=0..30°, start at 45° | After 1000 steps, deflection ≤ 30° + tolerance |
| 4 | JointLimitFrc > 0 at limit | Hinge at limit | Sensor reads > 0 |
| 5 | JointLimitFrc == 0 interior | Hinge within range | Sensor reads 0.0 |
| 6 | Limit is one-sided | Hinge at upper limit, lower limit inactive | Only upper limit generates force |
| 7 | Solref stiffness scales force | Two hinges: solref 0.005 vs 0.08 | Stiff force > soft force |
| 8 | Solimp width controls penetration | Two hinges: solimp width 0.001 vs 0.1 | Wide solimp → more penetration |
| 9 | Motor cannot push past limit | Slide + motor into limit | Position stays at limit ± tolerance |
| 10 | Zero-width range (locked) | Hinge range="0 0" | Position stays at 0 ± tolerance |
| 11 | Limit force increases with penetration | Two hinges: 50° vs 70° initial (limit at 45°) | Higher initial overshoot → higher peak force |
| 12 | Ball cone azimuthal symmetry | Ball joint pushed in +X vs +Y directions | Similar limit forces (within 10%) |

### Pattern

Same as `contact-filtering/stress-test`:

```rust
fn main() {
    println!("=== Joint Limits — Stress Test ===\n");
    // Run check functions, accumulate pass/total
    // Print final TOTAL: N/N checks passed
}
```

Each check is a standalone function returning `(u32, u32)` (passed, total).

---

## Workspace registration

Add to root `Cargo.toml` members:

```toml
# Examples — fundamentals / sim-cpu / joint-limits
"examples/fundamentals/sim-cpu/joint-limits/stress-test",
"examples/fundamentals/sim-cpu/joint-limits/hinge-limits",
"examples/fundamentals/sim-cpu/joint-limits/slide-limits",
"examples/fundamentals/sim-cpu/joint-limits/ball-cone",
```

---

## Implementation notes (from codebase audit)

### Initial position setup

- **Hinge joints:** Use `ref="60"` with `compiler angle="degree"` to start at
  60°. The `ref` attribute IS angle-converted (builder/joint.rs:170), so qpos0
  ends up in radians internally.
- **Slide joints:** Use `ref="0.5"` to start at 0.5m. No conversion (translational).
- **Ball joints:** No `ref` attribute for ball type — qpos0 is always identity
  quaternion `[1,0,0,0]`. Must set `data.qpos` directly after `model.make_data()`
  to a tilted quaternion (e.g., `[cos(θ/2), sin(θ/2)*ax, 0, 0]`).
- **Keyframe qpos:** Raw values, NO angle conversion. Avoid for hinge angles
  in degree mode. Use `ref` or direct qpos manipulation instead.

### Data access patterns

- `data.qpos[model.jnt_qpos_adr[jnt_id]]` — read joint position (radians for hinge)
- `data.jnt_limit_frc[jnt_id]` — direct limit force (unsigned, assignment not accumulation)
- `data.sensor_scalar(&model, "sensor_name")` — sensor readback by name
- `data.ctrl[actuator_id] = value` — set motor control
- `model.joint_id("name")` — lookup joint index
- `data.step(&model).expect("step")` — advance one timestep
- `data.forward(&model).expect("fwd")` — recompute kinematics without integration

### Check 10 clarification

Zero-width range `range="0 0"` creates a locked joint. Both upper and lower
limits can activate simultaneously. `jnt_limit_frc` uses assignment (last writer
wins, mod.rs:461), so only one side's force is stored. Test **position stability**
(stays within tolerance of 0), not force values.

### Check 11 clarification

"Limit force increases with penetration" — compare peak `jnt_limit_frc` over
the first ~100 steps between two models with different initial overshoot.
Track `max_force` across steps, not instantaneous equilibrium force.

---

## Implementation order

1. stress-test (headless — validates all engine features before visual work)
2. hinge-limits (simplest visual — one joint type, three variants)
3. slide-limits (adds motor interaction)
4. ball-cone (3D constraint — most complex visual)
