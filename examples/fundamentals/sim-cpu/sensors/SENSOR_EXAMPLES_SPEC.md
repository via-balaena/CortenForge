# Sensor Examples Spec

## Context

The `examples/fundamentals/sim-cpu/` directory covers joint types (hinge, slide, ball)
but has zero sensor coverage. sim-core has 35+ sensor types across 3 pipeline stages,
all of which will likely need individual debugging when first exercised from examples.
This spec defines a `sensors/` subdirectory with one focused example per sensor type
(or natural pair), following the established baby-step pattern.

## Directory Structure

```
examples/fundamentals/sim-cpu/sensors/
  README.md                    # Index + link to ball-joint/ for BallQuat/BallAngVel
  joint-pos-vel/               # JointPos + JointVel
  frame-pos-quat/              # FramePos + FrameQuat
  clock/                       # Clock
  subtree-com/                 # SubtreeCom
  gyro-velocimeter/            # Gyro + Velocimeter
  accelerometer/               # Accelerometer
  touch/                       # Touch
  actuator-force/              # ActuatorFrc + JointActuatorFrc
  geom-distance/               # GeomDist + GeomNormal + GeomFromTo
```

9 examples + 1 README. BallQuat/BallAngVel already covered by `ball-joint/spherical-pendulum`.

## Validation Philosophy

Many sensor types are direct pass-throughs from backing data fields (JointPos reads
`qpos`, Clock reads `time`, etc.). Comparing sensor output against the backing field
is tautological — error is always exactly 0. These checks are still valuable as
**pipeline smoke tests** (proves sensors are enabled, addresses are correct, data flows
through), but each example must also include at least one **non-tautological check**:

- **Range check:** Verify the sensor value spans a meaningful range over the simulation
  (catches "all zeros because pipeline didn't run" failures).
- **Analytical cross-check:** Compare against a hand-computed expectation from the physics.
- **Cross-path check:** Compare two different code paths that should agree.

## Per-Example Specs

### 1. `joint-pos-vel/` — JointPos + JointVel

**Package:** `example-sensor-joint-pos-vel`

**Model:** Single hinge joint, pendulum. Zero damping, RK4. **Initial displacement:
45° from vertical.**

```xml
<mujoco model="joint_pos_vel">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <sensor>
    <jointpos name="pos" joint="hinge"/>
    <jointvel name="vel" joint="hinge"/>
  </sensor>
</mujoco>
```

**Initial condition:** `data.qpos[adr] = π/4` (45°).

**Validation (custom SensorValidation resource):**
- **Pipeline check:** `JointPos` sensor == `data.joint_qpos(model, jid)[0]` (max err < 1e-14)
- **Pipeline check:** `JointVel` sensor == `data.joint_qvel(model, jid)[0]` (max err < 1e-14)
- **Range check:** JointPos spans a range > 1.0 rad over the simulation (pendulum swings
  through ±45°). Catches "sensor stuck at zero" failure.
- **Sign-change check:** JointVel changes sign at least 10 times in 15s (pendulum oscillates).

**Harness trackers:** `track_energy(0.5)` (undamped sanity check)

**Display:** `t=X.Xs  pos=+0.XXX rad  vel=+0.XXX rad/s  sensor_err=X.Xe-XX`

---

### 2. `frame-pos-quat/` — FramePos + FrameQuat

**Package:** `example-sensor-frame-pos-quat`

**Model:** Hinge pendulum with a site on the tip body. **Initial displacement: 30°.**

```xml
<mujoco model="frame_pos_quat">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
      <site name="tip_site" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <sensor>
    <framepos name="tip_pos" site="tip_site"/>
    <framequat name="tip_quat" site="tip_site"/>
  </sensor>
</mujoco>
```

**Initial condition:** `data.qpos[adr] = π/6` (30°).

**Validation:**
- **Pipeline check:** `FramePos` sensor == `data.site_xpos[site_id]` components (max err < 1e-12)
- **FrameQuat:** Use **rotation-distance metric** `1 - |dot(q_sensor, q_data)| < 1e-12`
  instead of component-wise comparison (handles quaternion sign ambiguity: q and -q
  represent the same rotation; component-wise would show error ~2.0 on sign flip).
- **Range check:** FramePos X-component spans > 0.2m (tip swings laterally).
- **Analytical cross-check:** At t=0 after forward(), tip_pos should be at
  `(L*sin(30°), 0, -L*cos(30°))` = `(0.25, 0, -0.433)` within 1e-6.

**Display:** `t=X.Xs  pos=(x,y,z)  rot_err=X.Xe-XX  pos_err=X.Xe-XX`

---

### 3. `clock/` — Clock

**Package:** `example-sensor-clock`

**Model:** Minimal — a hinge pendulum (so there's something to watch). The clock
sensor reads `data.time`.

```xml
<mujoco model="clock">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <sensor>
    <clock name="clock"/>
  </sensor>
</mujoco>
```

**Initial condition:** `data.qpos[adr] = π/4` (45°) — gives the viewer something to watch.

**Validation:**
- **Pipeline check:** `Clock` sensor == `data.time` exactly (max err < 1e-15)
- **Monotonicity check:** Clock value increases every frame (never decreases or stalls)
- **Positive check:** At report time (t=15s), clock sensor > 14.0 (catches "time never advanced")

**Display:** `t=X.Xs  clock=X.XXXXX  error=X.Xe-XX`

---

### 4. `subtree-com/` — SubtreeCom

**Package:** `example-sensor-subtree-com`

**Model:** Two-body chain (shoulder hinge + elbow hinge). **Both joints displaced.**

```xml
<mujoco model="subtree_com">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="upper" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="upper_rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.4"/>
      <body name="lower" pos="0 0 -0.4">
        <joint name="elbow" type="hinge" axis="0 1 0" damping="0"/>
        <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.01 0.01 0.01"/>
        <geom name="lower_rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.4"/>
        <geom name="lower_tip" type="sphere" size="0.04" pos="0 0 -0.4"/>
      </body>
    </body>
  </worldbody>
  <sensor>
    <subtreecom name="com" body="upper"/>
  </sensor>
</mujoco>
```

**Initial condition:** `shoulder.qpos = π/6`, `elbow.qpos = π/4`.

**Validation:**
- **Pipeline check:** `SubtreeCom` sensor == `data.subtree_com[body_id]` (max err < 1e-12)
- **Range check:** COM X-component range > 0.1m over the simulation (both joints swing)
- **Analytical cross-check at t=0:** With known masses (1.0 kg each), positions from FK,
  compute expected COM = `(m1*p1 + m2*p2) / (m1 + m2)` and compare within 1e-6.

**Display:** `t=X.Xs  com=(x,y,z)  error=X.Xe-XX`

---

### 5. `gyro-velocimeter/` — Gyro + Velocimeter

**Package:** `example-sensor-gyro-velocimeter`

**Model:** Hinge pendulum with a site at the tip. **Initial displacement: 30°.**
Both sensors read in the **sensor's local frame** (not world frame).

```xml
<mujoco model="gyro_velocimeter">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <site name="imu" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <velocimeter name="vel" site="imu"/>
  </sensor>
</mujoco>
```

**Initial condition:** `data.qpos[adr] = π/6` (30°).

**Key physics:** For a Y-axis hinge with site rotating with the body:
- **Gyro** reads angular velocity in sensor frame. Since the rotation axis (Y) is
  invariant under Y-axis rotation, gyro always reads `[0, ω, 0]` regardless of θ.
  This is **exact** — no frame-rotation error. Cross-check: `gyro_y == qvel[0]`.
- **Velocimeter** reads linear velocity in sensor frame. The tip velocity in world
  frame is `ω × r`, but after rotating into the sensor frame: `v_sensor = [-ω*L, 0, 0]`
  (constant direction, magnitude proportional to ω). This is also **exact** for a
  single-axis hinge.

**Validation:**
- **Gyro Y-component:** == `data.joint_qvel(model, 0)[0]` (max err < 1e-10)
- **Gyro X,Z-components:** == 0.0 (max err < 1e-10)
- **Velocimeter X-component:** == `-L * data.joint_qvel(model, 0)[0]` where L=0.5 (max err < 1e-10)
- **Velocimeter Y,Z-components:** == 0.0 (max err < 1e-10)
- **Range check:** Gyro Y spans > 1.0 rad/s (pendulum swings with meaningful velocity)

**Display:** `t=X.Xs  gyro=(x,y,z)  veloc=(x,y,z)  ω=X.XX  gyro_err=X.Xe-XX`

---

### 6. `accelerometer/` — Accelerometer

**Package:** `example-sensor-accelerometer`

**Model:** A box on a free joint, starting slightly above the ground plane. It drops,
contacts the floor, and settles. This gives two phases: free-fall (accel ≈ 0) and
at-rest (accel ≈ [0, 0, +9.81]).

```xml
<mujoco model="accelerometer">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" contype="1" conaffinity="1"/>
    <body name="box" pos="0 0 0.5">
      <joint name="free" type="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="cube" type="box" size="0.1 0.1 0.1"
            contype="1" conaffinity="1"/>
      <site name="imu" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
```

**Initial condition:** Box center at z=0.5 (bottom face at z=0.4, well above floor).
Free-fall time ≈ `sqrt(2 * 0.4 / 9.81)` ≈ 0.29s.

**Validation:**
- **Free-fall phase (t < 0.2s):** `|accel_z|` < 0.5 m/s² (near-zero, not exactly zero
  due to numerical damping in the first few frames)
- **At-rest phase (t > 2s, skip_until=2.0):** accel_z ≈ +9.81 within 2%.
  accel_x, accel_y each < 0.1 m/s².
- **Positive check:** `|accel|` magnitude at rest within [9.5, 10.1] m/s².

**Display:** `t=X.Xs  accel=(x,y,z)  |a|=X.XX  expected=9.81`

---

### 7. `touch/` — Touch

**Package:** `example-sensor-touch`

**Model:** A sphere dropping onto a ground plane. Frictionless contact (`condim="1"`)
to avoid pyramidal friction approximation skewing the normal force reading. Default
`solref`/`solimp` provide implicit damping for settling.

```xml
<mujoco model="touch">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" condim="1"
          contype="1" conaffinity="1"/>
    <body name="ball_body" pos="0 0 0.5">
      <joint name="free" type="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.004 0.004 0.004"/>
      <geom name="ball" type="sphere" size="0.1" condim="1"
            contype="1" conaffinity="1"/>
      <site name="touch_site" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <touch name="touch" site="touch_site"/>
  </sensor>
</mujoco>
```

**Initial condition:** Ball center at z=0.5 (surface at z=0.4). Free-fall ≈ 0.29s.
With `condim="1"` (frictionless normal-only contact) and default `solref="0.02 1"`
(critically-damped spring), the ball should settle within ~2s.

**Validation:**
- **Free-fall phase (t < 0.2s):** touch == 0.0 exactly
- **At-rest phase (t > 3s, skip_until=3.0):** touch ≈ m*g = 9.81 N within 5%.
  Using `condim="1"` avoids the pyramidal friction approximation which would give
  ~75% of the true normal force (~25% error).
- **Non-negativity:** touch >= 0.0 every frame (positive data kind)

**Display:** `t=X.Xs  touch=X.XX N  contact={yes/no}  expected_rest=9.81 N`

---

### 8. `actuator-force/` — ActuatorFrc + JointActuatorFrc

**Package:** `example-sensor-actuator-force`

**Model:** Hinge joint with a motor actuator. A custom `set_control` system applies
constant `ctrl[0] = 5.0` every frame before physics stepping.

```xml
<mujoco model="actuator_force">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" gear="1"/>
  </actuator>
  <sensor>
    <actuatorfrc name="afrc" actuator="motor"/>
    <jointactuatorfrc name="jfrc" joint="hinge"/>
  </sensor>
</mujoco>
```

**Control injection:** Custom Bevy system `set_control` runs in Update, **chained before**
`step_physics_realtime`:
```rust
fn set_control(mut data: ResMut<PhysicsData>) {
    data.ctrl[0] = 5.0;
}
// In main: .add_systems(Update, (set_control, step_physics_realtime).chain())
```

**Validation:**
- **Pipeline check:** `ActuatorFrc` sensor == `data.actuator_force[0]` (max err < 1e-12)
- **Cross-path check:** `JointActuatorFrc` sensor == `ActuatorFrc` sensor (for single
  motor with gear=1, both code paths should agree; max err < 1e-12)
- **Analytical check:** `ActuatorFrc` ≈ 5.0 (ctrl * gainprm[0] * gear = 5 * 1 * 1).
  Tolerance 1e-6 (actuator computation may involve floating-point intermediate ops).
- **Non-zero check:** `|ActuatorFrc|` > 1.0 at all times after t=0.01s

**Display:** `t=X.Xs  ctrl=5.00  afrc=X.XX  jfrc=X.XX  afrc-jfrc=X.Xe-XX`

---

### 9. `geom-distance/` — GeomDist + GeomNormal + GeomFromTo

**Package:** `example-sensor-geom-distance`

**Model:** A box (GJK path, not analytic) and a fixed sphere at known positions. The
box is on a slide joint with an initial displacement and a spring, so it oscillates
and the distance changes over time. Uses `cutoff="10"` on all distance sensors to
avoid the cutoff=0 edge case.

> **Note:** Sphere-sphere would use the analytic fast path, bypassing GJK/EPA entirely.
> Using a box exercises the actual GJK narrowphase code.

```xml
<mujoco model="geom_distance">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <geom name="fixed" type="sphere" size="0.1" pos="0.5 0 0"/>
    <body name="moving" pos="-0.3 0 0">
      <joint name="slide" type="slide" axis="1 0 0" stiffness="10" damping="0"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
  <sensor>
    <distance name="dist" geom1="box" geom2="fixed" cutoff="10"/>
    <normal name="norm" geom1="box" geom2="fixed" cutoff="10"/>
    <fromto name="ft" geom1="box" geom2="fixed" cutoff="10"/>
  </sensor>
</mujoco>
```

**Initial condition:** `data.qpos[slide_adr] = 0.2` (displaces box to x=-0.1, so box
nearest face at x=0.0 and sphere surface at x=0.4 — gap = 0.4m). Spring at stiffness=10
pulls it back, creating oscillation.

**Validation:**
- **GeomDist analytical:** For a box at position `p_box` with half-extent 0.1 and sphere
  at `(0.5, 0, 0)` with radius 0.1: nearest point on box face is at `x = p_box + 0.1`,
  nearest point on sphere is at `x = 0.5 - 0.1 = 0.4`. Distance =
  `(0.4) - (p_box + 0.1) = 0.3 - p_box`. Compare sensor vs. analytical each frame.
  Tolerance: 1e-6 (GJK numerical precision).
- **GeomNormal:** For box-sphere along X-axis, normal should be `[1, 0, 0]` ± 1e-4
  (GJK may have slight off-axis error for box edge cases).
- **GeomFromTo:** Verify from-point is on box surface, to-point is on sphere surface.
- **Range check:** Distance varies by > 0.1m over the simulation (spring oscillation).

**Display:** `t=X.Xs  dist=X.XXX  normal=(x,y,z)  analytical=X.XXX  err=X.Xe-XX`

---

## Implementation Order

Build from simplest to most complex, so earlier examples validate infrastructure:

1. `clock/` — trivial, proves sensor pipeline works at all
2. `joint-pos-vel/` — simplest real sensors, direct state readback
3. `frame-pos-quat/` — FK-dependent, site attachment
4. `subtree-com/` — multi-body FK
5. `gyro-velocimeter/` — velocity stage
6. `accelerometer/` — acceleration stage (needs constraint solve + contact)
7. `touch/` — acceleration stage + contact force measurement
8. `actuator-force/` — acceleration stage + actuator pipeline + control injection
9. `geom-distance/` — GJK/EPA pathway, completely separate

## Potential Engine Issues

These may surface during implementation and should be fixed in the engine, not
worked around in examples:

- **cutoff=0 distance sensor bug:** `position.rs` line ~446 initializes `dist = cutoff`.
  When cutoff=0 (default), this means `dist = 0.0`, and the `dist_new < dist` check
  never updates for separated geoms (positive distance). Should initialize to `f64::MAX`
  when cutoff <= 0.0, matching MuJoCo's `mjMAXVAL` behavior. The spec works around
  this with `cutoff="10"` but the engine should be fixed.

## Skeleton Structure (per example)

```
example-name/
  Cargo.toml        # workspace deps, publish = false
  src/
    main.rs         # ~150-250 LOC
```

**Cargo.toml template:**
```toml
[package]
name = "example-sensor-{name}"
version.workspace = true
edition.workspace = true
license.workspace = true
publish = false

[dependencies]
sim-core = { workspace = true }
sim-mjcf = { workspace = true }
sim-bevy = { workspace = true }
bevy = { workspace = true, features = [
    "bevy_asset",
    "bevy_core_pipeline",
    "bevy_pbr",
    "bevy_render",
    "bevy_state",
    "bevy_winit",
    "multi_threaded",
    "tonemapping_luts",
    "zstd_rust",
] }

[lints]
workspace = true
```

## Code Pattern (all examples)

Each follows the established pattern from existing examples:

1. Doc comment header (sensor type, what it validates)
2. `#![allow(...)]` block (same as other examples)
3. Inline MJCF const
4. `main()` with println header, App::new(), plugins, harness, systems
5. `setup()` — load MJCF, **set initial qpos**, forward(), spawn_model_geoms with
   material overrides, camera
6. Custom `SensorValidation` resource + `sensor_diagnostics` system
7. Validation prints when `harness.reported()` fires
8. **System ordering:** `sensor_diagnostics` must run after `validation_system` in
   PostUpdate (use `.chain()` or explicit ordering)

## Files Modified

- `Cargo.toml` (workspace root) — add 9 workspace members
- `examples/EXAMPLES.md` — add sensors section to table
- `examples/COVERAGE_SPEC.md` — update sensors from STUB to Working (after each passes)

## Files Created

- `examples/fundamentals/sim-cpu/sensors/README.md`
- 9 × `Cargo.toml` + `src/main.rs` (18 files)

## Verification

Per example:
```bash
cargo run -p example-sensor-{name} --release
```
Wait for 15s report. All checks should PASS.

Batch smoke test (after all 9 are done):
```bash
cargo build -p example-sensor-clock \
  -p example-sensor-joint-pos-vel \
  -p example-sensor-frame-pos-quat \
  -p example-sensor-subtree-com \
  -p example-sensor-gyro-velocimeter \
  -p example-sensor-accelerometer \
  -p example-sensor-touch \
  -p example-sensor-actuator-force \
  -p example-sensor-geom-distance
```
