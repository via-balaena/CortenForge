# Tendon Examples Spec

8 examples covering the tendon system: fixed tendons (linear joint coupling),
spatial tendons (3D site routing with wrapping), tendon limits, pulleys,
tendon-driven actuators, and a headless stress-test.

**Branch:** `feature/examples-tendons`

**Engine status:** All v1.0 tendon items shipped (DT-28, DT-32, DT-33, DT-35,
DT-78). 40+ integration tests, MuJoCo 3.4.0 conformance.

---

## File Layout

```
examples/fundamentals/sim-cpu/tendons/
├── README.md
├── TENDON_EXAMPLES_SPEC.md          ← this file
├── fixed-coupling/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── spatial-path/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── sphere-wrap/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── cylinder-wrap/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── tendon-limits/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── pulley/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
├── tendon-actuator/
│   ├── Cargo.toml
│   ├── README.md
│   └── src/main.rs
└── stress-test/
    ├── Cargo.toml
    ├── README.md
    └── src/main.rs
```

**Workspace registration** — add to root `Cargo.toml` `members`:

```toml
# Examples — fundamentals / sim-cpu / tendons
"examples/fundamentals/sim-cpu/tendons/fixed-coupling",
"examples/fundamentals/sim-cpu/tendons/spatial-path",
"examples/fundamentals/sim-cpu/tendons/sphere-wrap",
"examples/fundamentals/sim-cpu/tendons/cylinder-wrap",
"examples/fundamentals/sim-cpu/tendons/tendon-limits",
"examples/fundamentals/sim-cpu/tendons/pulley",
"examples/fundamentals/sim-cpu/tendons/tendon-actuator",
"examples/fundamentals/sim-cpu/tendons/stress-test",
```

**Naming convention:** `example-tendon-{name}` in each `Cargo.toml`.

---

## Conventions

- Visual examples (1–7): sim-core + sim-mjcf + sim-bevy + bevy.
- Stress-test (8): sim-core + sim-mjcf only (headless).
- MJCF models embedded as `const MJCF: &str = r#"..."#` — no external XML files.
- Each visual example uses `OrbitCameraPlugin`, `PhysicsHud`, `ValidationHarness`.
- `spawn_model_geoms()` with `MetalPreset` colors for geometry.
- `spawn_example_camera()` with target/distance/azimuth/elevation.
- Tendon visualization via `TendonVisualization` resource, populated each frame
  from `data.wrap_xpos` / `data.ten_wrapadr` / `data.ten_wrapnum`.

---

## Example 1: `fixed-coupling`

**Concept:** A fixed tendon linearly couples two hinge joints. The tendon
length is `L = 1.0 * q1 + (-0.5) * q2`. A stiff spring on the tendon creates
a restoring force that mechanically couples the joints — when joint A moves,
joint B is pulled through the tendon.

**Scene:**

Two pendulums hanging from the world body, connected by a fixed tendon with
`stiffness="200"` and `damping="5"`. A motor on joint A drives it sinusoidally.
Joint B has no motor — it moves only through the tendon coupling force.

```xml
<mujoco model="fixed-coupling">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <geom type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>
    <body name="arm_a" pos="-0.3 0 0.8">
      <joint name="j_a" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"
            rgba="0.2 0.5 0.9 1" mass="0.5"/>
      <body name="tip_a" pos="0 0 -0.4">
        <geom type="sphere" size="0.04" rgba="0.2 0.5 0.9 1" mass="0.2"/>
      </body>
    </body>
    <body name="arm_b" pos="0.3 0 0.8">
      <joint name="j_b" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"
            rgba="0.9 0.3 0.2 1" mass="0.5"/>
      <body name="tip_b" pos="0 0 -0.4">
        <geom type="sphere" size="0.04" rgba="0.9 0.3 0.2 1" mass="0.2"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <fixed name="coupling" stiffness="200" damping="5">
      <joint joint="j_a" coef="1.0"/>
      <joint joint="j_b" coef="-0.5"/>
    </fixed>
  </tendon>

  <actuator>
    <motor name="drive_a" joint="j_a" gear="1"/>
  </actuator>

  <sensor>
    <tendonpos name="ten_pos" tendon="coupling"/>
    <tendonvel name="ten_vel" tendon="coupling"/>
    <jointpos name="angle_a" joint="j_a"/>
    <jointpos name="angle_b" joint="j_b"/>
  </sensor>
</mujoco>
```

**Control logic:** Sinusoidal motor on `j_a`:
`ctrl[0] = 2.0 * sin(2π * 0.5 * t)` (0.5 Hz, ±2 N·m).

**What you see:** Blue pendulum (A) swings under motor drive. Red pendulum (B)
follows at half the amplitude in the opposite direction, pulled by the tendon
spring. The coupling ratio is visible: when A moves +1 rad, B moves ~−0.5 rad.

**HUD:**

| Row | Content |
|-----|---------|
| Joint A | `{angle_a:+.1}°` |
| Joint B | `{angle_b:+.1}°` |
| Tendon L | `{ten_pos:.4}` (= 1.0·q_a − 0.5·q_b) |
| Tendon V | `{ten_vel:+.4}` |
| Manual L | `{1.0*q_a - 0.5*q_b:.4}` |
| Match | `✓` if |sensor − manual| < 1e-10 |

**Validation:** TendonPos sensor matches `1.0 * qpos[j_a] + (-0.5) * qpos[j_b]`
to machine precision every frame.

**Camera:** target=(0, 0.4, 0), distance=2.5, azimuth=0, elevation=0.3.

---

## Example 2: `spatial-path`

**Concept:** A spatial tendon routed through 3 sites on a 2-link arm. The
tendon length is the sum of Euclidean distances between consecutive sites.
No wrapping — pure straight-line segments.

**Scene:**

A 2-link planar arm (shoulder hinge + elbow hinge) with 3 sites: origin (on
upper arm near shoulder), midpoint (on upper arm near elbow), insertion (on
forearm near tip). The tendon path is visible as cyan line segments.

```xml
<mujoco model="spatial-path">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <geom type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>
    <body name="upper" pos="0 0 1.0">
      <joint name="shoulder" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5"
            rgba="0.6 0.6 0.7 1" mass="1.0"/>
      <site name="s_origin" pos="0.06 0 -0.05" size="0.015"/>
      <site name="s_mid" pos="0.06 0 -0.45" size="0.015"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0" damping="0.5"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"
              rgba="0.5 0.5 0.6 1" mass="0.5"/>
        <site name="s_insert" pos="0.05 0 -0.2" size="0.015"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <spatial name="path" stiffness="50" damping="2">
      <site site="s_origin"/>
      <site site="s_mid"/>
      <site site="s_insert"/>
    </spatial>
  </tendon>

  <sensor>
    <tendonpos name="ten_pos" tendon="path"/>
    <tendonvel name="ten_vel" tendon="path"/>
  </sensor>
</mujoco>
```

**Control logic:** None — arm swings under gravity. The shoulder and elbow
settle into a hanging configuration; the tendon spring provides a mild
restoring force.

**What you see:** A 2-link arm hangs and swings. Cyan tendon path (3 segments)
stretches and contracts as the arm moves. Site markers visible as small spheres.

**HUD:**

| Row | Content |
|-----|---------|
| Shoulder | `{q_shoulder:+.1}°` |
| Elbow | `{q_elbow:+.1}°` |
| Tendon L | `{ten_pos:.4}` |
| Manual L | `{d(s1,s2) + d(s2,s3):.4}` (Euclidean) |
| Match | `✓` if |sensor − manual| < 1e-8 |

**Tendon visualization:** Each frame, read `data.wrap_xpos` for the tendon's
path points (indexed by `data.ten_wrapadr[t]..+data.ten_wrapnum[t]`) and
populate `TendonVisualization`.

**Validation:** TendonPos matches sum of Euclidean segment distances to < 1e-8.

**Camera:** target=(0, 0.5, 0), distance=2.5, azimuth=0.3, elevation=0.2.

---

## Example 3: `sphere-wrap`

**Concept:** A spatial tendon wraps around a sphere. The tendon takes the
shortest path that doesn't penetrate the sphere, producing a great-circle arc
on the sphere surface. The wrap arc length changes with configuration, creating
a configuration-dependent moment arm.

**Scene:**

A single hinge joint rotates a body around a central sphere (radius 0.1). Two
sites — origin and insertion — are on opposite sides. The tendon wraps around
the sphere when the straight-line path would penetrate it.

```xml
<mujoco model="sphere-wrap">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <geom type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>

    <!-- Wrap sphere (fixed) -->
    <body name="wrap_body" pos="0 0 0.8">
      <geom name="wrap_sphere" type="sphere" size="0.1"
            rgba="0.8 0.8 0.2 0.3" contype="0" conaffinity="0"/>
    </body>

    <!-- Rotating arm with origin site -->
    <body name="arm" pos="0 0 0.8">
      <joint name="j1" type="hinge" axis="0 1 0" damping="0.3"/>
      <geom type="capsule" size="0.025" fromto="0 0 0 0.3 0 0"
            rgba="0.2 0.6 0.9 1" mass="0.5"/>
      <site name="s_origin" pos="0.25 0 0" size="0.015"/>
    </body>

    <!-- Fixed insertion site -->
    <body name="anchor" pos="-0.25 0 0.8">
      <site name="s_insert" pos="0 0 0" size="0.015"/>
    </body>
  </worldbody>

  <tendon>
    <spatial name="wrap_tendon" stiffness="100" damping="3">
      <site site="s_origin"/>
      <geom geom="wrap_sphere"/>
      <site site="s_insert"/>
    </spatial>
  </tendon>

  <sensor>
    <tendonpos name="ten_pos" tendon="wrap_tendon"/>
    <tendonvel name="ten_vel" tendon="wrap_tendon"/>
  </sensor>
</mujoco>
```

**Control logic:** None — arm swings under gravity past the sphere.

**What you see:** Arm rotates around the translucent yellow sphere. The cyan
tendon path hugs the sphere surface when wrapping is active — you see the arc
on the sphere. When the straight-line path is clear, the tendon goes straight.

**HUD:**

| Row | Content |
|-----|---------|
| Joint angle | `{q:+.1}°` |
| Tendon L | `{ten_pos:.4}` |
| Wrap active | `Yes` / `No` (based on `ten_wrapnum > 2`) |
| Wrap points | count from `data.ten_wrapnum[0]` |

**Tendon visualization:** Populate `TendonVisualization` from `wrap_xpos`.
When wrapping is active, the path includes intermediate points on the sphere
surface (tangent entry/exit + arc).

**Validation:** When wrapping is active, all wrap path points are at
distance ≥ sphere radius from sphere center (tendon doesn't penetrate).

**Camera:** target=(0, 0.4, 0), distance=2.0, azimuth=0.4, elevation=0.3.

---

## Example 4: `cylinder-wrap`

**Concept:** A spatial tendon wraps around a cylinder. Unlike sphere wrapping
(great-circle arc), cylinder wrapping follows a geodesic helix on the
cylinder surface. The Z-component of the path interpolates linearly.

**Scene:**

Two bodies on slide joints (one above, one below) connected by a spatial tendon
that wraps around a central cylinder (radius 0.08). As the slide joints move,
the helical wrap path changes length.

```xml
<mujoco model="cylinder-wrap">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <geom type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>

    <!-- Wrap cylinder (fixed, vertical) -->
    <body name="cyl_body" pos="0 0 0.6">
      <geom name="wrap_cyl" type="cylinder" size="0.08 0.25"
            rgba="0.2 0.8 0.3 0.3" contype="0" conaffinity="0"/>
    </body>

    <!-- Upper body with site -->
    <body name="upper" pos="-0.25 0.08 0.8">
      <joint name="j_up" type="hinge" axis="0 1 0" damping="0.5"
             range="-1.5 1.5" limited="true"/>
      <geom type="capsule" size="0.025" fromto="0 0 0 0.15 0 0"
            rgba="0.2 0.5 0.9 1" mass="0.5"/>
      <site name="s_up" pos="0.12 0 0" size="0.015"/>
    </body>

    <!-- Lower body with site -->
    <body name="lower" pos="0.25 -0.06 0.4">
      <joint name="j_down" type="hinge" axis="0 1 0" damping="0.5"
             range="-1.5 1.5" limited="true"/>
      <geom type="capsule" size="0.025" fromto="0 0 0 -0.15 0 0"
            rgba="0.9 0.3 0.2 1" mass="0.5"/>
      <site name="s_down" pos="-0.12 0 0" size="0.015"/>
    </body>
  </worldbody>

  <tendon>
    <spatial name="cyl_tendon" stiffness="80" damping="2">
      <site site="s_up"/>
      <geom geom="wrap_cyl"/>
      <site site="s_down"/>
    </spatial>
  </tendon>

  <sensor>
    <tendonpos name="ten_pos" tendon="cyl_tendon"/>
    <tendonvel name="ten_vel" tendon="cyl_tendon"/>
  </sensor>
</mujoco>
```

**Control logic:** None — bodies swing under gravity, tendon provides coupling.

**What you see:** Two arms on opposite sides of a translucent green cylinder.
The cyan tendon path wraps helically around the cylinder surface. As the arms
swing, the helix tightens or loosens.

**HUD:**

| Row | Content |
|-----|---------|
| Upper angle | `{q_up:+.1}°` |
| Lower angle | `{q_down:+.1}°` |
| Tendon L | `{ten_pos:.4}` |
| Wrap active | `Yes` / `No` |
| Wrap points | count |

**Tendon visualization:** Same pattern as sphere-wrap — `wrap_xpos` contains
the helical path points on the cylinder surface.

**Validation:** When wrapping, all wrap path points satisfy
`sqrt(x² + y²) ≥ cylinder_radius` in the cylinder's local frame.

**Camera:** target=(0, 0.3, 0), distance=2.5, azimuth=0.5, elevation=0.4.

---

## Example 5: `tendon-limits`

**Concept:** A spatial tendon with `limited="true"` and `range="lo hi"`. When
the tendon length exceeds the range, the solver generates a one-sided
constraint (same mechanism as joint limits). `TendonLimitFrc` reports the
constraint force.

**Scene:**

A 2-link arm with a spatial tendon. The tendon has `range="0.2 0.6"`.
A motor drives the shoulder joint, stretching the tendon toward the upper
limit. When the limit activates, the arm resists further extension.

```xml
<mujoco model="tendon-limits">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <geom type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>
    <body name="upper" pos="0 0 1.0">
      <joint name="shoulder" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5"
            rgba="0.6 0.6 0.7 1" mass="1.0"/>
      <site name="s1" pos="0.06 0 -0.05" size="0.015"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0" damping="0.5"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"
              rgba="0.5 0.5 0.6 1" mass="0.5"/>
        <site name="s2" pos="0.05 0 -0.2" size="0.015"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <spatial name="limited_tendon" limited="true" range="0.2 0.6"
            stiffness="0" damping="0" margin="0.01">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="drive" joint="shoulder" gear="5"/>
  </actuator>

  <sensor>
    <tendonpos name="ten_pos" tendon="limited_tendon"/>
    <tendonlimitfrc name="ten_lim_frc" tendon="limited_tendon"/>
  </sensor>
</mujoco>
```

**Control logic:** Ramp motor torque: `ctrl[0] = 0.5 * sin(2π * 0.3 * t)`
to push the arm back and forth through both limits.

**What you see:** Arm swings. When the tendon length hits 0.6 (upper limit) or
0.2 (lower limit), the motion stops — the constraint holds. The tendon path
changes color intensity based on proximity to limit.

**HUD:**

| Row | Content |
|-----|---------|
| Shoulder | `{q:+.1}°` |
| Tendon L | `{ten_pos:.4}` (range: 0.2–0.6) |
| Limit Frc | `{ten_lim_frc:+.4}` |
| At limit | `UPPER` / `LOWER` / `interior` |

**Validation:**
- TendonLimitFrc == 0 when `0.2 < L < 0.6` (interior).
- TendonLimitFrc > 0 when L ≥ 0.6 (upper limit active).
- TendonLimitFrc > 0 when L ≤ 0.2 (lower limit active).
- Tendon length never exceeds range ± solver penetration tolerance.

**Camera:** target=(0, 0.5, 0), distance=2.5, azimuth=0, elevation=0.2.

---

## Example 6: `pulley`

**Concept:** A spatial tendon with a `<pulley divisor="2"/>` element splitting
the path into two branches. Branch 1 contributes full length; branch 2
contributes length/divisor. This models mechanical advantage (like a block and
tackle).

**Scene:**

Three bodies arranged vertically. Branch 1: site on top body → site on middle
body. Pulley (divisor=2). Branch 2: site on middle body → site on bottom body.
Top and bottom bodies have slide joints.

```xml
<mujoco model="pulley">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <geom type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>

    <!-- Top: free to slide vertically -->
    <body name="top" pos="0 0 1.2">
      <joint name="j_top" type="slide" axis="0 0 1" damping="1.0"/>
      <geom type="box" size="0.08 0.08 0.04" rgba="0.2 0.5 0.9 1" mass="1.0"/>
      <site name="s1" pos="0 0 -0.04" size="0.015"/>
    </body>

    <!-- Middle: fixed (the pulley anchor) -->
    <body name="middle" pos="0 0 0.7">
      <geom type="box" size="0.06 0.06 0.03" rgba="0.8 0.8 0.2 1" mass="0.1"
            contype="0" conaffinity="0"/>
      <site name="s2" pos="0 0 0.03" size="0.015"/>
      <site name="s3" pos="0 0 -0.03" size="0.015"/>
    </body>

    <!-- Bottom: free to slide vertically -->
    <body name="bottom" pos="0 0 0.3">
      <joint name="j_bot" type="slide" axis="0 0 1" damping="1.0"/>
      <geom type="box" size="0.08 0.08 0.04" rgba="0.9 0.3 0.2 1" mass="2.0"/>
      <site name="s4" pos="0 0 0.04" size="0.015"/>
    </body>
  </worldbody>

  <tendon>
    <spatial name="pulley_tendon" stiffness="500" damping="10">
      <site site="s1"/>
      <site site="s2"/>
      <pulley divisor="2"/>
      <site site="s3"/>
      <site site="s4"/>
    </spatial>
  </tendon>

  <sensor>
    <tendonpos name="ten_pos" tendon="pulley_tendon"/>
    <tendonvel name="ten_vel" tendon="pulley_tendon"/>
  </sensor>
</mujoco>
```

**Control logic:** None — gravity pulls both masses. The top mass (1 kg) and
bottom mass (2 kg) settle to equilibrium with the pulley scaling the force.

**What you see:** Blue box (top) and red box (bottom) connected through the
yellow pulley anchor. The tendon path shows two branches (cyan lines). The
bottom mass moves half as far as the top mass due to the 2:1 mechanical
advantage.

**HUD:**

| Row | Content |
|-----|---------|
| Top pos | `{q_top:+.4} m` |
| Bot pos | `{q_bot:+.4} m` |
| Tendon L | `{ten_pos:.4}` |
| Branch 1 | `{d(s1,s2):.4}` |
| Branch 2 | `{d(s3,s4)/2:.4}` (÷ divisor) |
| Sum | `{branch1 + branch2:.4}` |

**Validation:** Tendon length = `d(s1,s2) + d(s3,s4)/2.0` (branch 2 scaled
by 1/divisor). Verify to < 1e-8.

**Camera:** target=(0, 0.35, 0), distance=3.0, azimuth=0, elevation=0.2.

---

## Example 7: `tendon-actuator`

**Concept:** An actuator with tendon transmission: `<general tendon="t1">`.
The actuator drives the tendon, which in turn drives the joints. For a spatial
tendon, the effective gear ratio is configuration-dependent (moment arm varies).
Contrast with a joint-transmission actuator where gear is constant.

**Scene:**

Two 2-link arms side by side:
- **Left arm:** actuator drives the tendon (`<general tendon="t1">`). The
  tendon is spatial (routed through sites on the arm).
- **Right arm:** actuator drives the joint directly (`<general joint="j1">`).

Both receive the same control signal. The tendon-driven arm shows
configuration-dependent force amplification.

```xml
<mujoco model="tendon-actuator">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <geom type="plane" size="2 2 0.01" rgba="0.3 0.3 0.3 1"/>

    <!-- Left arm: tendon-driven -->
    <body name="L_upper" pos="-0.4 0 1.0">
      <joint name="L_shoulder" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5"
            rgba="0.2 0.5 0.9 1" mass="1.0"/>
      <site name="L_s1" pos="0.06 0 -0.05" size="0.015"/>
      <body name="L_lower" pos="0 0 -0.5">
        <joint name="L_elbow" type="hinge" axis="0 1 0" damping="0.5"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"
              rgba="0.15 0.4 0.8 1" mass="0.5"/>
        <site name="L_s2" pos="0.05 0 -0.2" size="0.015"/>
      </body>
    </body>

    <!-- Right arm: joint-driven -->
    <body name="R_upper" pos="0.4 0 1.0">
      <joint name="R_shoulder" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5"
            rgba="0.9 0.3 0.2 1" mass="1.0"/>
      <body name="R_lower" pos="0 0 -0.5">
        <joint name="R_elbow" type="hinge" axis="0 1 0" damping="0.5"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4"
              rgba="0.8 0.25 0.15 1" mass="0.5"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <spatial name="L_tendon">
      <site site="L_s1"/>
      <site site="L_s2"/>
    </spatial>
  </tendon>

  <actuator>
    <general name="tendon_drive" tendon="L_tendon" gainprm="50 0 0"
             biasprm="0 0 0" gear="1"/>
    <general name="joint_drive" joint="R_shoulder" gainprm="50 0 0"
             biasprm="0 0 0" gear="1"/>
  </actuator>

  <sensor>
    <tendonpos name="L_ten_pos" tendon="L_tendon"/>
    <jointpos name="L_shoulder_pos" joint="L_shoulder"/>
    <jointpos name="R_shoulder_pos" joint="R_shoulder"/>
    <actuatorfrc name="L_act_frc" actuator="tendon_drive"/>
    <actuatorfrc name="R_act_frc" actuator="joint_drive"/>
  </sensor>
</mujoco>
```

**Control logic:** Same sinusoidal signal to both actuators:
`ctrl[0] = ctrl[1] = 0.5 * sin(2π * 0.3 * t)`.

**What you see:** Blue (tendon-driven) and red (joint-driven) arms side by
side. Both receive the same control input. The blue arm's response differs
because the tendon's moment arm changes with configuration — the effective
torque varies. The red arm has a constant gear ratio.

**HUD:**

| Row | Content |
|-----|---------|
| L shoulder | `{L_q:+.1}°` |
| R shoulder | `{R_q:+.1}°` |
| Tendon L | `{L_ten_pos:.4}` |
| L act force | `{L_act_frc:+.3}` |
| R act force | `{R_act_frc:+.3}` |
| Ctrl | `{ctrl:.3}` |

**Validation:** The two arms diverge — same input, different output — proving
the configuration-dependent gear ratio of tendon transmission.

**Camera:** target=(0, 0.5, 0), distance=3.5, azimuth=0, elevation=0.2.

---

## Example 8: `stress-test`

**Concept:** Headless validation of all tendon invariants. 16+ checks covering
fixed tendons, spatial tendons, wrapping, limits, pulleys, actuators, springs,
Jacobians, and sensors.

**Dependencies:** sim-core + sim-mjcf only (no Bevy).

**Structure:** Multiple embedded MJCF models + one `check()` function per
invariant. Main function aggregates results and exits with code 1 on failure.

### Models

| Const | Content | Used by |
|-------|---------|---------|
| `MODEL_FIXED` | Two hinges + fixed tendon `coef=[1.0, -0.5]` | Checks 1–3, 13 |
| `MODEL_SPATIAL` | 2-link arm + spatial tendon (3 sites, no wrap) | Checks 5, 14 |
| `MODEL_SPHERE` | Hinge + sphere wrap tendon | Check 6 |
| `MODEL_CYLINDER` | Two slides + cylinder wrap tendon | Check 7 |
| `MODEL_LIMITS` | Spatial tendon with `range="0.2 0.6"` | Checks 8–9 |
| `MODEL_PULLEY` | Spatial tendon with `<pulley divisor="2"/>` | Check 10 |
| `MODEL_ACTUATOR` | Motor with `tendon="t1"` transmission | Check 11 |
| `MODEL_SPRING` | Fixed tendon with `stiffness=100 damping=5` | Check 12 |
| `MODEL_MULTI` | Two fixed tendons on the same joint | Check 15 |
| `MODEL_MOMENT_ARM` | Spatial tendon on a hinge, two configurations | Check 16 |

### Checks

| # | Name | Invariant |
|---|------|-----------|
| 1 | Fixed tendon length | `ten_length[0] == 1.0 * qpos[j_a] + (-0.5) * qpos[j_b]` exactly |
| 2 | Fixed tendon velocity | `ten_velocity[0] == 1.0 * qvel[j_a] + (-0.5) * qvel[j_b]` exactly |
| 3 | TendonPos sensor | `sensor_data[ten_pos_adr] == data.ten_length[0]` |
| 4 | TendonVel sensor | `sensor_data[ten_vel_adr] == data.ten_velocity[0]` |
| 5 | Spatial length (no wrap) | `ten_length[0] == d(s1,s2) + d(s2,s3)` to < 1e-10 |
| 6 | Sphere wrap non-penetration | All wrap path points at distance ≥ `sphere_radius` from center |
| 7 | Cylinder wrap non-penetration | All wrap path points at distance ≥ `cylinder_radius` in local XY |
| 8 | Limit activates at range | `TendonLimitFrc > 0` when `ten_length ≥ range.1` |
| 9 | No limit force interior | `TendonLimitFrc == 0` when `range.0 < ten_length < range.1` |
| 10 | Pulley divisor scaling | `ten_length == d(s1,s2) + d(s3,s4) / divisor` to < 1e-10 |
| 11 | Tendon actuator force | `qfrc_actuator != 0` when ctrl != 0 with tendon transmission |
| 12 | Spring/damper passive force | `ten_force[0] == -stiffness * (L - L0) - damping * V` to < 1e-8 |
| 13 | Fixed Jacobian is constant | `ten_J[0]` is identical at two different configurations |
| 14 | Spatial Jacobian varies | `ten_J[0]` differs between two configurations |
| 15 | Multiple tendons compose | Two tendons on same joint: both have correct independent lengths |
| 16 | Moment arm varies | `ten_J[0][dof]` at config A ≠ `ten_J[0][dof]` at config B |

### Expected output

```
=== Tendons — Stress Test ===

-- 1. Fixed tendon length --
  [PASS] fixed_length: |error| = 0.00e+00 < 1.00e-12

-- 2. Fixed tendon velocity --
  [PASS] fixed_velocity: |error| = 0.00e+00 < 1.00e-12

...

============================================================
  TOTAL: 16/16 checks passed
  ALL PASS
```

---

## Progression

Build order (each depends on patterns established by predecessors):

1. **stress-test** — headless, validates all engine features first
2. **fixed-coupling** — simplest visual tendon
3. **spatial-path** — introduces tendon visualization from `wrap_xpos`
4. **sphere-wrap** — adds wrapping geometry
5. **cylinder-wrap** — second wrapping type
6. **tendon-limits** — constraint system
7. **pulley** — branched tendon path
8. **tendon-actuator** — integration with actuator system

---

## Acceptance Criteria

- [ ] All 8 examples compile: `cargo build -p example-tendon-{name}`
- [ ] Stress-test passes: `cargo run -p example-tendon-stress-test --release` → 16/16 PASS
- [ ] Each visual example runs for 15s without crash
- [ ] Each visual example shows tendon path (cyan lines) updating in real-time
- [ ] HUD displays sensor readback for all examples
- [ ] Each example has a README.md following the established pattern
- [ ] Workspace Cargo.toml includes all 8 members
- [ ] `cargo clippy -p example-tendon-{name} -- -D warnings` clean for all 8
- [ ] Parent README.md with example table + key ideas
