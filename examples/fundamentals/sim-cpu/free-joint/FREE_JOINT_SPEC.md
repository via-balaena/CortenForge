# Free Joint Examples — Spec

**Status:** Draft — awaiting approval before implementation.

**Parent:** `examples/COVERAGE_SPEC.md` Track 1B, Layer 1, subdirectory 1 of 20.

**Existing coverage gap:** Free joints are the only joint type without a dedicated
example directory. The `energy-momentum/free-flight` example uses a free joint but
focuses on conservation laws — it never explains what a free joint *is*.

---

## Goal

Four examples that cover the **free joint as a joint type**: its 7/6 qpos/qvel
layout, quaternion integration on SO(3), torque-free rigid body dynamics, and
composition with equality constraints. Each example introduces exactly one concept.

---

## Directory Layout

```
fundamentals/sim-cpu/free-joint/
  tumble/             # Bevy visual — rotation only (zero-g)
  projectile/         # Bevy visual — translation only (gravity)
  spinning-toss/      # Bevy visual — full 6-DOF flight (translation + rotation)
  stress-test/        # Headless — 12 checks
  README.md
```

---

## Example 1: `tumble` — Torque-Free Precession

A box with asymmetric inertia (`Ixx != Iyy != Izz`) launched in zero gravity
with angular velocity only. The angular velocity vector precesses around the
angular momentum vector (Euler's equations), but `|L|` is exactly conserved.

**What it teaches:** The rotational subspace of a free joint — 4-component
quaternion in qpos, 3-component angular velocity in qvel, quaternion
integration via exponential map on SO(3).

### MJCF

```xml
<mujoco model="tumble">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="box" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.10 0.06 0.03"/>
      <geom name="body" type="box" size="0.25 0.18 0.12"
            contype="0" conaffinity="0" rgba="0.30 0.55 0.80 1"/>
      <!-- Painted corner to track rotation visually -->
      <geom name="marker" type="sphere" size="0.03"
            pos="0.25 0.18 0.12" contype="0" conaffinity="0"
            rgba="0.95 0.25 0.15 1"/>
    </body>
  </worldbody>

  <sensor>
    <subtreeangmom name="angmom" body="box"/>
  </sensor>
</mujoco>
```

### Initial Conditions

```
qvel = [0, 0, 0, 2.0, 0.5, 0.1]  # zero linear, angular = (2.0, 0.5, 0.1)
```

Angular velocity is primarily about the major axis (X) with small
perturbation in Y and Z. For asymmetric inertia, the intermediate axis is
unstable (Dzhanibekov effect), but here the dominant axis is the major axis
so the motion is stable precession.

### Analytical Values

- **Angular momentum** (body frame, t=0):
  L = I * omega = (0.10*2.0, 0.06*0.5, 0.03*0.1) = (0.200, 0.030, 0.003)
- **|L|** = sqrt(0.04 + 0.0009 + 0.000009) = 0.20225 kg*m^2/s
- **KE** = 0.5 * (Ix*wx^2 + Iy*wy^2 + Iz*wz^2) = 0.5*(0.4 + 0.015 + 0.0003) = 0.20765 J

Both |L| and KE are conserved exactly (zero gravity, no contacts, no damping).

### Bevy Setup

- **Camera:** `spawn_example_camera`, center at origin, distance 1.5, elevated
  slightly to see 3D tumble clearly.
- **Materials:**
  - body: `BrushedMetal` with steel-blue tint
  - marker: `PolishedSteel` bright red — makes rotation tracking easy
- **HUD:**
  - `omega_x`, `omega_y`, `omega_z` — watch precession in real time
  - `|L|` — should remain constant
  - `KE` — should remain constant
  - `|quat|` — quaternion norm (should stay 1.0)
  - `time`
- **Validation:** `report_at(15.0)`, checks:
  - KE drift < 1e-10 over 15s
  - |L| drift < 1e-8 over 15s
  - Quaternion norm drift < 1e-10 over 15s

---

## Example 2: `projectile` — Parabolic Trajectory

A sphere launched at 45 degrees in gravity. Exact analytical solution for the
entire trajectory. Demonstrates the translational subspace of a free joint.

**What it teaches:** The translational subspace — 3-component position in qpos,
3-component linear velocity in qvel, how gravity acts on the linear DOFs only.

### MJCF

```xml
<mujoco model="projectile">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <!-- Ground plane for visual reference (no collision with projectile) -->
    <geom name="ground" type="plane" size="5 5 0.01"
          rgba="0.3 0.3 0.3 0.5" contype="0" conaffinity="0"/>

    <body name="ball" pos="0 0 0.05">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.004 0.004 0.004"/>
      <geom name="sphere" type="sphere" size="0.05"
            contype="0" conaffinity="0" rgba="0.85 0.30 0.15 1"/>
    </body>
  </worldbody>
</mujoco>
```

### Initial Conditions

```
v0 = 5.0 m/s
angle = 45 degrees = pi/4
qvel = [v0*cos(45), 0, v0*sin(45), 0, 0, 0]
     = [3.5355, 0, 3.5355, 0, 0, 0]
```

### Analytical Values

- **Apex time:** t_apex = v0*sin(theta) / g = 3.5355 / 9.81 = 0.3604 s
- **Apex height:** h = v0^2 * sin^2(theta) / (2g) = 25*0.5 / 19.62 = 0.6371 m
- **Range time:** t_range = 2 * t_apex = 0.7208 s
- **Range:** R = v0^2 * sin(2*theta) / g = 25 / 9.81 = 2.5484 m
- **Total energy:** E = 0.5*m*v0^2 + m*g*z0 = 12.5 + 0.4905 = 12.9905 J (conserved)

### Bevy Setup

- **Camera:** center at (1.3, 0, 0.3) — midpoint of trajectory, distance 4.0,
  azimuth facing XZ plane, slight elevation.
- **Materials:**
  - ground: default grey (semi-transparent, visual only)
  - sphere: `PolishedSteel` with orange-red
- **HUD:**
  - `x`, `z` — position
  - `vx`, `vz` — velocity
  - `x_err`, `z_err` — deviation from analytical (x = v0*cos*t, z = z0 + v0*sin*t - 0.5*g*t^2)
  - `E_total` — should be conserved
  - `time`
- **Validation:** `report_at(1.5)` (well past landing), checks:
  - Apex height within 0.1% of analytical
  - Range within 0.1% of analytical
  - Total energy drift < 0.5% over flight

---

## Example 3: `spinning-toss` — Full 6-DOF Flight

A box with asymmetric inertia tossed upward at an angle with spin. The body
follows a parabolic arc (translation) while tumbling (rotation) — both
subspaces active simultaneously. This is what a real free-floating object does.

**What it teaches:** The full 6-DOF free joint — translation and rotation
coupled in one body. Gravity acts on the 3 linear DOFs only; the 3 angular
DOFs evolve independently (torque-free). The parabolic COM trajectory and the
tumbling rotation are decoupled.

### MJCF

```xml
<mujoco model="spinning-toss">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <!-- Ground plane for visual reference (no collision) -->
    <geom name="ground" type="plane" size="5 5 0.01"
          rgba="0.3 0.3 0.3 0.5" contype="0" conaffinity="0"/>

    <body name="brick" pos="0 0 0.5">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.5" diaginertia="0.06 0.03 0.015"/>
      <geom name="body" type="box" size="0.18 0.12 0.07"
            contype="0" conaffinity="0" rgba="0.40 0.55 0.75 1"/>
      <!-- Corner marker to track spin visually -->
      <geom name="marker" type="sphere" size="0.025"
            pos="0.18 0.12 0.07" contype="0" conaffinity="0"
            rgba="0.95 0.25 0.15 1"/>
    </body>
  </worldbody>

  <sensor>
    <subtreeangmom name="angmom" body="brick"/>
  </sensor>
</mujoco>
```

### Initial Conditions

```
qvel = [2.0, 0, 4.0, 3.0, 1.0, 0.5]
         ^lin_x  ^lin_z  ^ang_x ^ang_y ^ang_z
```

Tossed at ~63 degrees from horizontal with spin primarily about X.

### Analytical Values

Translation (COM follows exact projectile):
- **Apex time:** t_apex = vz/g = 4.0/9.81 = 0.4077 s
- **Apex height:** z_apex = z0 + vz^2/(2g) = 0.5 + 16/19.62 = 1.3155 m
- **Ground crossing:** t_gnd (solve z0 + vz*t - 0.5*g*t^2 = 0) = 0.9284 s
- **x at ground:** x = vx * t_gnd = 2.0 * 0.9284 = 1.8568 m

Rotation (decoupled from translation):
- **|L|** = sqrt((Ix*wx)^2 + (Iy*wy)^2 + (Iz*wz)^2)
          = sqrt((0.18)^2 + (0.03)^2 + (0.0075)^2)
          = sqrt(0.0324 + 0.0009 + 0.00005625)
          = 0.18274 kg*m^2/s — conserved exactly
- **KE_rot** = 0.5*(Ix*wx^2 + Iy*wy^2 + Iz*wz^2)
             = 0.5*(0.54 + 0.03 + 0.00375) = 0.28688 J — conserved exactly
- **Total energy** = KE_lin + KE_rot + PE — conserved exactly

Key insight: COM trajectory is identical to a point-mass projectile. Rotation
is identical to the zero-gravity tumble case. The two subspaces don't interact.

### Bevy Setup

- **Camera:** `spawn_example_camera`, center at (1.0, 0, 0.7) — midpoint of
  arc, distance 4.0, azimuth facing XZ plane, slight elevation.
- **Materials:**
  - body: `BrushedMetal` steel-blue tint
  - marker: `PolishedSteel` bright red — spin tracking
- **HUD:**
  - `x`, `z` — COM position
  - `x_err`, `z_err` — deviation from point-mass projectile
  - `|L|` — angular momentum magnitude (should be constant)
  - `E_total` — total energy (should be conserved)
  - `time`
- **Validation:** `report_at(2.0)` (well past ground crossing), checks:
  - Apex height within 0.1% of analytical
  - |L| conserved to < 1e-8 (rotation decoupled from gravity)
  - Total energy drift < 0.5%
  - COM x-position tracks point-mass projectile to < 0.1%

---

## Example 4: `stress-test` — Headless Validation

12 checks covering all free joint concepts. No Bevy, no rendering — pure
pass/fail. This is implemented and verified *before* the visual examples.

### Checks

Each check is a standalone function with its own MJCF model.

| # | Name | Model | What it checks | Tolerance |
|---|------|-------|----------------|-----------|
| 1 | Quaternion norm preserved | free body, zero-g, angular vel, 10s | `\|quat\| - 1.0` | < 1e-10 |
| 2 | Linear momentum conserved (zero-g) | free body, zero-g, lin+ang vel, 10s | `\|p_final - p_initial\|` | < 1e-10 |
| 3 | Angular momentum conserved (zero-g) | free body, zero-g, asymmetric inertia, 10s | `\|\|L_final\| - \|L_initial\|\|` | < 1e-8 |
| 4 | Projectile apex height | free body, gravity, 45deg launch | `\|z_apex - analytical\| / analytical` | < 0.1% |
| 5 | Projectile range | free body, gravity, 45deg launch | `\|x_range - analytical\| / analytical` | < 0.1% |
| 6 | qpos dim = 7, qvel dim = 6 | any free body | `nq`, `nv` from model | exact |
| 7 | Free joint has no limits | free body with `limited="true"` | joint_limited flag ignored, no constraint rows | exact |
| 8 | Gravity: linear acceleration = g | free body, gravity, at rest | `qacc[2]` after forward | < 1e-10 vs -9.81 |
| 9 | No interaction without contact | two free bodies, zero-g, no contact | each body velocity independent | < 1e-12 |
| 10 | Quaternion integration = exponential map | compare quat after 1 step vs manual exp map | `\|q_engine - q_analytical\|` | < 1e-12 |
| 11 | Energy conservation (zero-g) | free body, zero-g, KE + PE | `\|E_final - E_initial\|` | < 1e-10 |
| 12 | Gravity changes linear momentum at rate mg | free body, gravity, 1s | `\|p_z(1s) - (p_z(0) - m*g*1s)\|` | < 1e-6 |

### MJCF Models (shared where possible)

**Model A — Zero gravity, asymmetric inertia** (checks 1, 2, 3, 9, 11):
```xml
<mujoco model="free-zero-g">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="box" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.10 0.06 0.03"/>
      <geom type="box" size="0.15 0.1 0.08" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <sensor>
    <subtreeangmom name="angmom" body="box"/>
  </sensor>
</mujoco>
```

**Model B — Gravity** (checks 4, 5, 8, 12):
```xml
<mujoco model="free-gravity">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="ball" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.004 0.004 0.004"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
```

**Model C — Two independent bodies** (check 9):
```xml
<mujoco model="free-two-bodies">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>
  <worldbody>
    <body name="a" pos="0 0 0">
      <freejoint name="free_a"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="2 0 0">
      <freejoint name="free_b"/>
      <inertial pos="0 0 0" mass="3.0" diaginertia="0.05 0.05 0.05"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
```

### Check Details

**Check 1 — Quaternion norm:**
```
init: qvel = [0.5, 0.3, 0, 2.0, 0.5, 0.1]
step 10,000 times (10s)
qnorm = sqrt(qpos[3]^2 + qpos[4]^2 + qpos[5]^2 + qpos[6]^2)
PASS if |qnorm - 1.0| < 1e-10
```

**Check 2 — Linear momentum:**
```
init: mass=2.0, qvel = [0.5, 0.3, 0, 0, 0, 0]
p0 = [1.0, 0.6, 0.0]
step 10,000 times (10s)
p_final = mass * qvel[0..3]
PASS if |p_final - p0| < 1e-10
```

**Check 3 — Angular momentum:**
```
init: qvel = [0, 0, 0, 2.0, 0.5, 0.1]
forward() to populate sensor
L0_mag = |sensordata[angmom]|
step 10,000 times, forward()
L_final_mag = |sensordata[angmom]|
PASS if |L_final_mag - L0_mag| < 1e-8
```

**Check 4 — Projectile apex:**
```
init: v0=5.0, theta=pi/4, qvel = [v0*cos, 0, v0*sin, 0, 0, 0], qpos[2]=0
analytical apex = v0^2*sin^2(theta)/(2g) = 0.63710 m
step until qvel[2] crosses zero (rising->falling)
PASS if |z_apex - analytical| / analytical < 0.001
```

**Check 5 — Projectile range:**
```
same launch as check 4
analytical range = v0^2*sin(2*theta)/g = 2.5484 m
step until qpos[2] crosses back below 0
PASS if |x_range - analytical| / analytical < 0.001
```

**Check 6 — Dimensions:**
```
model = load(Model B)
PASS if model.nq == 7 && model.nv == 6
Also verify: joint_qpos(0).len() == 7, joint_qvel(0).len() == 6
```

**Check 7 — No limits:**
```
Parse a free joint with limited="true" and range="-1 1".
verify: model.jnt_limited[0] is irrelevant — the constraint assembly
generates zero limit rows for free joints. Load, step, and confirm
no limit-related constraint rows exist (nefc_limit == 0 or the joint
is not represented in the constraint Jacobian limit rows).

Alternative: set up a free body, apply large velocity, step —
position goes arbitrarily far. No clamping, no constraint force.
init: qvel = [100, 0, 0, 0, 0, 0], step 1000 times
PASS if qpos[0] > 90.0 (≈100.0) — no limit clamped it
```

**Check 8 — Gravity acceleration:**
```
model = load(Model B), data = make_data()
forward()
PASS if |qacc[2] - (-9.81)| < 1e-10
Also: qacc[0] == 0, qacc[1] == 0 (no lateral acceleration)
Also: qacc[3] == 0, qacc[4] == 0, qacc[5] == 0 (no angular acceleration)
```

**Check 9 — No interaction:**
```
model = load(Model C)
init: body_a qvel = [1.0, 0, 0, 0, 0, 0], body_b qvel = [0, 0, 0.5, 0, 0, 0]
step 5000 (5s)
PASS if |qvel_a[0] - 1.0| < 1e-12 (unchanged)
PASS if |qvel_b[2] - 0.5| < 1e-12 (unchanged)
```

**Check 10 — Quaternion integration matches exp map:**
```
model = load(Model A)
init: qvel = [0, 0, 0, 1.0, 0.5, 0.3]
dt = model.timestep (0.001)
omega = (1.0, 0.5, 0.3)
analytical: q_new = q_old * exp(0.5 * omega * dt)
            where exp = UnitQuaternion::from_axis_angle(omega/|omega|, |omega|*dt)
step 1 time
PASS if |qpos[3..7] - q_analytical| < 1e-12
```

**Check 11 — Energy conservation:**
```
model = load(Model A)
init: qvel = [0.5, 0.3, 0, 2.0, 0.5, 0.1]
forward(), E0 = KE + PE (PE = 0 in zero-g)
step 10,000 (10s)
PASS if |E_final - E0| < 1e-10
```

**Check 12 — Gravity changes momentum:**
```
model = load(Model B), mass = 1.0, g = 9.81
init: qvel = [0, 0, 0, 0, 0, 0] (at rest)
step 1000 (1s)
expected p_z = mass * (-g * 1.0) = -9.81
actual p_z = mass * qvel[2]
PASS if |actual - expected| < 1e-6
```

---

## Package Names

| Example | Crate name |
|---------|-----------|
| tumble | `example-free-joint-tumble` |
| projectile | `example-free-joint-projectile` |
| spinning-toss | `example-free-joint-spinning-toss` |
| stress-test | `example-free-joint-stress-test` |

---

## Implementation Order

1. **stress-test** — headless, verifies all engine behavior
2. **tumble** — simplest visual (rotation only, zero-g)
3. **projectile** — translation only, gravity
4. **spinning-toss** — full 6-DOF (translation + rotation, gravity)

---

## Differentiation from Existing Examples

| Existing example | What it covers | What free-joint/ adds |
|-----------------|----------------|----------------------|
| `energy-momentum/free-flight` | KE/momentum conservation | qpos/qvel layout, exp map, precession as visual phenomenon |
| `ball-joint/` | Spherical joint (3 DOF) | Full 6 DOF, translation + rotation, projectile |
| (none) | — | Full 6-DOF flight (spinning-toss): both subspaces active, decoupled |

---

## Validation Clock

All visual examples report at t=10-15s. The stress test runs fully synchronous
(no Bevy clock) and completes in <1s on release builds.
