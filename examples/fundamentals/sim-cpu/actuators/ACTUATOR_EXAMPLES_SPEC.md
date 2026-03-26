# Actuator Examples Spec

**Status:** Draft — stress-tested, ready for review
**Date:** 2026-03-26
**Directory:** `examples/fundamentals/sim-cpu/actuators/`

## Goal

Cover every distinct actuator code path in sim-core with one baby-step example
per concept. Each example exercises a unique combination of gain type, bias type,
dynamics type, or transmission type — not just parameter variations.

## Scope

This spec covers standard (non-muscle) actuators. Muscle/HillMuscle examples
live in a separate `muscles/` directory per the coverage spec.

## Gap Being Closed

From `COVERAGE_SPEC.md` gap analysis:

| Category | Covered | **Uncovered (this spec)** |
|----------|---------|---------------------------|
| Dynamics | None | **Filter, FilterExact, Integrator** |
| Gain | Fixed | **Affine** |
| Bias | None | **Affine** |
| Transmission | Joint, Tendon | **Site, SliderCrank** |
| Features | — | **gear ratio, force clamping, ctrl clamping, activation state** |

Body/Adhesion transmission and JointInParent are noted as future candidates
(adhesion requires contacts; JointInParent only differs for ball joints).

---

## Examples (implementation order)

### 1. `motor` — Direct Torque Control

**The simplest actuator:** constant torque on a hinge joint. No dynamics, no
bias, no filtering — `ctrl` maps directly to joint torque.

**Concept:** `<motor>` shortcut → `GainType::Fixed(1)`, `BiasType::None`,
`ActuatorDynamics::None`, `ActuatorTransmission::Joint`.

**MJCF model:**
```xml
<mujoco model="motor">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="torque"/>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
```

**Setup:** Arm starts at rest (θ=0, hanging down). Apply constant `ctrl[0] = 2.0`
every frame.

**Analytical predictions:**
- Effective inertia: `I_eff = I_body + armature + m*d²` where `d` is CoM offset
  from pivot. `I_eff = 0.01 + 0.01 + 1.0 * 0.25² = 0.0825 kg·m²`.
- Without gravity: `α = τ / I_eff = 2.0 / 0.0825 ≈ 24.24 rad/s²`.
- With gravity: `α(θ) = (τ - m*g*d*sin(θ)) / I_eff`. At θ=0 (hanging):
  `α = (2.0 - 0) / 0.0825 = 24.24 rad/s²`. Gravity term is zero when hanging
  straight down (sin(0)=0).

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Force = ctrl | `actuator_force[0] == ctrl[0]` (gain=1, no bias) | exact (< 1e-15) |
| Initial acceleration | `α ≈ τ / I_eff` at t≈0 | < 2% |
| Quadratic position | `θ(t) ≈ ½αt²` for first 0.1s (gravity < 15% of τ) | < 3% |
| Sensor pipeline | `ActuatorFrc` sensor == `data.actuator_force[0]` | exact (< 1e-15) |

**Engine issues to watch for:**
- Inertia computation for offset CoM (parallel axis theorem).
- Armature contribution to effective inertia.

---

### 2. `position-servo` — PD Position Control

**A position actuator (PD servo) without activation dynamics.** The arm targets
a specified angle and converges via proportional-derivative control.

**Concept:** `<position>` shortcut → `GainType::Fixed(kp)`,
`BiasType::Affine(0, -kp, -kv)`, `ActuatorDynamics::None` (timeconst=0).
Force = `kp * (ctrl - q) - kv * qdot`.

**MJCF model:**
```xml
<mujoco model="position-servo">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>

  <actuator>
    <position name="servo" joint="hinge" kp="100" dampratio="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="servo"/>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
```

**Setup:** Arm starts hanging (θ=0). Set `ctrl[0] = π/4` (target: 45°).

**Analytical predictions:**
- Equation of motion: `I_eff * α = kp*(target - θ) - kv*ω - m*g*d*sin(θ)`
- With `dampratio=1`: critically damped → no overshoot.
- `kv = 2 * dampratio * sqrt(kp * I_eff) = 2 * sqrt(100 * 0.0825) ≈ 5.74 N·m·s/rad`.
  (Converted by `compute_actuator_params` from positive `biasprm[2]`.)
- Natural frequency: `ωn = sqrt(kp / I_eff) ≈ 34.8 rad/s`.
- 2% settling time: `ts ≈ 4 / ωn ≈ 0.115 s`.
- Steady-state offset from gravity: `θ_ss = target - m*g*d*sin(target)/kp`.
  Small for kp=100.

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Reaches target | `\|θ - target\| < 0.05 rad` after 0.5s | < 3° |
| No overshoot | θ never exceeds target (critically damped) | strict |
| Force formula | `force ≈ kp*(ctrl - q) - kv*qdot` | < 1% |
| Damping correct | kv matches `2*sqrt(kp*I_eff)` ± 5% | verify from biasprm |
| No activation state | `data.act` is empty (dyntype=None) | exact |

**Engine issues to watch for:**
- `dampratio` → `kv` conversion via `compute_actuator_params()` (Phase 5 Spec A,
  DT-56). Requires correct `acc0` computation.
- Gravity steady-state offset (position servo doesn't compensate gravity
  perfectly unless kp is very large).

---

### 3. `velocity-servo` — Velocity Tracking

**A velocity actuator that drives toward a target angular velocity.** The joint
accelerates until the velocity matches the control signal.

**Concept:** `<velocity>` shortcut → `GainType::Fixed(kv)`,
`BiasType::Affine(0, 0, -kv)`, `ActuatorDynamics::None`.
Force = `kv * (ctrl - qdot)`.

**MJCF model:**
```xml
<mujoco model="velocity-servo">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>

  <actuator>
    <velocity name="vel_servo" joint="hinge" kv="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="vel_servo"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
```

**Setup:** Gravity disabled for clean dynamics. Arm starts at rest. Set
`ctrl[0] = 2.0` (target: 2 rad/s).

**Analytical predictions:**
- Force = `kv * (ctrl - qdot)` = `1.0 * (2.0 - qdot)`.
- Equation: `I_eff * dω/dt = 1.0 * (2 - ω)`.
- First-order ODE: `ω(t) = 2.0 * (1 - e^(-t/τ_v))`.
- Time constant: `τ_v = I_eff / kv = 0.0825 / 1.0 = 0.0825 s`.
- 99% of target velocity reached at `t ≈ 4.6 * τ_v ≈ 0.38 s`.

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Reaches target ω | `\|ω - 2.0\| < 0.05 rad/s` after 0.5s | < 2.5% |
| Exponential approach | `ω(t)` matches `2*(1-e^(-t/τ))` | < 3% at t=τ |
| Force at start | `force ≈ kv * ctrl = 2.0 N·m` at t=0 | < 1% |
| Force at steady state | `force ≈ 0` when ω ≈ target | < 0.05 N·m |

**Engine issues to watch for:**
- Gravity disabled: verify `DISABLE_GRAVITY` flag or `gravity="0 0 0"` works
  correctly for this scenario.
- Time constant `τ_v ≈ 83ms` — transient visible over ~0.4s at 60 FPS.

---

### 4. `damper` — Viscous Damping

**A damper actuator that applies force proportional to velocity.** A spinning
arm decays exponentially under viscous drag.

**Concept:** `<damper>` shortcut → `GainType::Affine(0, 0, -kv)`,
`BiasType::None`, `ActuatorDynamics::None`.
Force = `-kv * velocity * ctrl`. Ctrl scales the damping (0 = off, 1 = full).

**MJCF model:**
```xml
<mujoco model="damper">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="wheel" pos="0 0 0">
      <joint name="spin" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.05 0.05 0.05"/>
      <geom name="disc" type="cylinder" size="0.2 0.02"
            euler="1.5708 0 0"/>
    </body>
  </worldbody>

  <actuator>
    <damper name="brake" joint="spin" kv="0.5" ctrlrange="0 1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="brake"/>
    <jointvel name="jvel" joint="spin"/>
  </sensor>
</mujoco>
```

**Setup:** Gravity disabled. Set initial `qvel[0] = 10.0` rad/s. Hold
`ctrl[0] = 1.0` (full damping).

**Analytical predictions:**
- `I_eff = 0.05 + 0.01 = 0.06 kg·m²` (diag inertia + armature).
- Force = `-kv * ω * ctrl = -0.5 * ω * 1.0`.
- `I_eff * dω/dt = -0.5ω` → `ω(t) = 10 * e^(-0.5t/0.06) = 10 * e^(-8.33t)`.
- Time constant: `τ = I_eff / kv = 0.06 / 0.5 = 0.12 s`.
- After `5τ = 0.6s`, velocity < 1% of initial.

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Exponential decay | `ω(t) / ω₀` matches `e^(-t/τ)` | < 3% at t=τ,2τ,3τ |
| Time constant | Velocity drops to 37% at t=τ | < 5% |
| Force proportional | `force ≈ -0.5 * ω` at each sample | < 2% |
| Affine gain path | `gaintype == Affine` (distinct from Fixed) | structural |
| Ctrl scaling | With ctrl=0.5: decay rate halves | < 5% |

**Engine issues to watch for:**
- Damper forces `ctrllimited=true` with default `ctrlrange=[0,1]`. Negative ctrl
  would reverse damping direction — verify clamping prevents this.
- `GainType::Affine` code path: `gain = gainprm[0] + gainprm[1]*length + gainprm[2]*velocity`.
  For damper: only `gainprm[2] = -kv` is non-zero.

---

### 5. `activation-filter` — FilterExact Activation Dynamics

**A position servo with first-order activation filtering.** Adding `timeconst`
to a position actuator introduces an activation state that smoothly tracks the
control signal, delaying the force response.

**Concept:** `<position>` with `timeconst > 0` → `ActuatorDynamics::FilterExact`.
Introduces `data.act[i]` state variable. Force uses activation (not ctrl) as
the effective setpoint. Contrast with example 2 (no dynamics).

**MJCF model:**
```xml
<mujoco model="activation-filter">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>

  <actuator>
    <position name="filtered_servo" joint="hinge"
              kp="100" dampratio="1" timeconst="0.1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="filtered_servo"/>
    <jointpos name="jpos" joint="hinge"/>
  </sensor>
</mujoco>
```

**Setup:** Arm starts hanging (θ=0). Step ctrl to `π/4` at t=0.

**Analytical predictions:**
- Activation dynamics: `d(act)/dt = (ctrl - act) / τ` with `τ = 0.1s`.
- FilterExact integration: `act(t) = ctrl * (1 - e^(-t/τ))`.
- The effective setpoint ramps gradually → arm responds more slowly than
  position-servo (example 2) which has no filter.
- At `t = τ = 0.1s`: `act ≈ 0.632 * ctrl ≈ 0.497 rad`.
- At `t = 3τ = 0.3s`: `act ≈ 0.95 * ctrl ≈ 0.747 rad`.

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Activation exists | `data.act.len() == 1` (one activation state) | exact |
| Filter response | `act(τ) ≈ 0.632 * ctrl` | < 5% |
| Filter response | `act(3τ) ≈ 0.950 * ctrl` | < 3% |
| Slower than no-filter | Position at t=0.2s < position-servo at t=0.2s | strict |
| act_dot correct | `act_dot ≈ (ctrl - act) / τ` | < 2% |
| FilterExact vs Euler | Exact integration: no timestep-dependent drift | verify |

**Engine issues to watch for:**
- FilterExact vs Filter: FilterExact uses `act += act_dot * τ * (1 - e^(-dt/τ))`
  which is exact for constant ctrl. Filter uses Euler: `act += act_dot * dt`.
  Verify FilterExact gives the right answer independent of timestep.
- `actearly` interaction: if actearly is true, force uses predicted next-step
  activation → reduces effective delay by one timestep.

---

### 6. `cylinder` — Pneumatic Cylinder

**A pneumatic/hydraulic cylinder actuator with Filter dynamics and affine bias.**
The cylinder has an effective area (pressure × area = force), first-order
pressure buildup, and bias forces representing internal friction and spring
return. A horizontal rail eliminates gravity so the cylinder only fights inertia.

**Concept:** `<cylinder>` shortcut → `GainType::Fixed(area)`,
`BiasType::Affine(bias[0], bias[1], bias[2])`, `ActuatorDynamics::Filter(τ)`.
Force = `area * activation + bias[0] + bias[1]*length + bias[2]*velocity`.
Demonstrates Filter dynamics (Euler-approximated, distinct from FilterExact in
example 5).

**MJCF model:**
```xml
<mujoco model="cylinder">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="cart" pos="0 0 0">
      <joint name="piston" type="slide" axis="1 0 0" armature="0.01"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <geom name="block" type="box" size="0.08 0.05 0.05"/>
    </body>
  </worldbody>

  <actuator>
    <cylinder name="pneumatic" joint="piston"
              timeconst="0.2" diameter="0.08"
              bias="0 -5 -2"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="pneumatic"/>
    <jointpos name="jpos" joint="piston"/>
    <jointvel name="jvel" joint="piston"/>
  </sensor>
</mujoco>
```

**Setup:** No gravity. Cart starts at rest (x=0). Apply `ctrl[0] = 1.0`
(full pressure) at t=0. The cylinder pushes the cart along X.

**Analytical predictions:**
- Area = `π/4 * 0.08² ≈ 0.005027 m²`.
- Bias: `b₀=0, b₁=-5, b₂=-2` (spring return + velocity damping).
- `m_eff = 1.0 + 0.01 = 1.01 kg` (mass + slide armature).
- Activation dynamics (Filter/Euler): `d(act)/dt = (ctrl - act) / τ`,
  `τ = 0.2s`. `act(t) ≈ 1 - e^(-t/0.2)`.
- Force = `area * act + 0 - 5*x - 2*vel`. The `-5*x` term acts as a spring
  centering force, the `-2*vel` term acts as damping. The system is a
  driven damped oscillator.
- At steady state (act=1, vel=0): `force = area - 5*x_eq = 0` →
  `x_eq = area/5 ≈ 0.001 m` (very small displacement). The spring return
  bias dominates.
- To see meaningful motion: the filter transient matters most. During the
  0.2s activation ramp, the cart accelerates, overshoots slightly due to
  inertia, then settles at x_eq.

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Activation exists | `data.act.len() == 1` | exact |
| Filter dynamics | `act(τ) ≈ 0.632` | < 5% |
| Force formula | `force ≈ area * act + bias · [1, len, vel]` | < 2% |
| Euler filter | Using Filter (not FilterExact) — slight dt-dependence | verify |
| Spring equilibrium | Cart settles near `x_eq = area/5 ≈ 0.001 m` | < 20% |
| Bias damping | Velocity decays to zero (damping term works) | vel < 0.01 by t=2s |
| Area from diameter | `area = π/4 * d²` matches `gainprm[0]` | < 1e-10 |

**Engine issues to watch for:**
- Filter (Euler) vs FilterExact: cylinder uses plain Filter (Euler integration),
  which has slight timestep-dependent error. This is by design (matches MuJoCo).
  Compare `act` at t=τ with FilterExact prediction — expect small discrepancy.
- Bias spring return (`bias[1] = -5`): creates a restoring force proportional
  to actuator length (= gear × joint position). Verify this feedback path works.

---

### 7. `integrator` — Integrator Dynamics

**A general actuator with integrator dynamics.** The control input is treated as
a rate: `d(act)/dt = ctrl`. The activation ramps linearly with constant ctrl,
giving position-like control from a velocity-like input.

**Concept:** `<general>` with `dyntype="integrator"` →
`ActuatorDynamics::Integrator`. Activation is the integral of ctrl over time.
Must use `actlimited="true"` to prevent unbounded growth.

**MJCF model:**
```xml
<mujoco model="integrator">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>

  <actuator>
    <general name="int_actuator" joint="hinge"
             dyntype="integrator" gainprm="50"
             biastype="affine" biasprm="0 0 -5"
             actlimited="true" actrange="-1.57 1.57"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="int_actuator"/>
    <jointpos name="jpos" joint="hinge"/>
  </sensor>
</mujoco>
```

**Setup:** Gravity disabled. Arm at rest. Apply `ctrl[0] = 1.0` for 0.5s, then
`ctrl[0] = 0.0` (hold), then `ctrl[0] = -1.0` for 0.5s (reverse).

**Analytical predictions:**
- `d(act)/dt = ctrl`. With ctrl=1.0: `act(t) = t` (linear ramp).
- After 0.5s: `act = 0.5`. After ctrl=0: act holds at 0.5 (integrator has memory).
- Force = `50 * act + 0 + 0 - 5*vel`. Acts like a position servo where the
  "target" (activation) is commanded by velocity input.
- Activation clamped to `[-π/2, π/2]` by actlimited.

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Linear ramp | `act(0.5) ≈ 0.5` with ctrl=1 | < 1% |
| Hold at zero ctrl | `act` constant when ctrl=0 | < 1e-6 drift/s |
| Reverse ramp | `act` decreases with ctrl=-1 | monotonic |
| Activation clamping | `act` never exceeds actrange | strict |
| Force = gain*act+bias | Verify force formula at each sample | < 2% |
| act_dot = ctrl | `data.act_dot[0] == ctrl[0]` | exact (< 1e-15) |

**Engine issues to watch for:**
- Without `actlimited`, activation grows unboundedly → force diverges.
  This is correct behavior but the example must demonstrate the safeguard.
- Integrator dynamics is rarely used in MuJoCo — this path may be less tested.

---

### 8. `gear-and-limits` — Gear Ratio and Force Clamping

**A motor with non-unity gear ratio and force/control limits.** Demonstrates
transmission scaling and saturation behavior.

**Concept:** `gear` parameter scales moment arm (and thus output torque).
`forcerange` clamps actuator force. `ctrlrange` clamps control input.
These are parameter-level features, not type-level, but they hit distinct
clamping code paths.

**MJCF model:** Two side-by-side arms — a baseline motor (gear=1, no limits) and
a geared motor with force clamping — for direct comparison.

```xml
<mujoco model="gear-and-limits">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm_a" pos="0 0.5 0">
      <joint name="hinge_a" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod_a" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip_a" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>

    <body name="arm_b" pos="0 -0.5 0">
      <joint name="hinge_b" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod_b" type="capsule" size="0.02" fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip_b" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="baseline" joint="hinge_a" gear="1"/>
    <motor name="geared_limited" joint="hinge_b" gear="5"
           forcerange="-3 3"/>
  </actuator>

  <sensor>
    <actuatorfrc name="force_a" actuator="baseline"/>
    <actuatorfrc name="force_b" actuator="geared_limited"/>
    <jointvel name="vel_a" joint="hinge_a"/>
    <jointvel name="vel_b" joint="hinge_b"/>
  </sensor>
</mujoco>
```

**Setup:** Gravity disabled. Two phases demonstrate gear scaling and force
saturation.

**Phase 1** — both get `ctrl = 1.0` (below force limit):
- Arm A: `actuator_force = 1.0`, `qfrc_actuator = 1 × 1.0 = 1.0 N·m`.
- Arm B: `actuator_force = 1.0` (< 3, not clamped), `qfrc_actuator = 5 × 1.0 = 5.0 N·m`.
- Arm B accelerates 5× faster (same inertia, 5× torque).

**Phase 2** — both get `ctrl = 5.0` (triggers force clamping on B):
- Arm A: `actuator_force = 5.0`, `qfrc_actuator = 1 × 5.0 = 5.0 N·m`.
- Arm B: `actuator_force = 5.0 → clamped to 3.0`, `qfrc_actuator = 5 × 3.0 = 15.0 N·m`.
- Force clamping occurs in actuator space (pre-gear), then gear multiplies.

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Gear scaling | Arm B accel = 5× Arm A accel (ctrl=1) | < 3% |
| Force identity | `actuator_force = ctrl * gain` (unclamped) | exact |
| Force clamping | `actuator_force_b = 3.0` when ctrl=5 | exact |
| Torque = gear × force | `qfrc_actuator = gear * clamped_force` | < 1e-10 |
| Actuator length | `actuator_length = gear * joint_pos` | exact |

**Engine issues to watch for:**
- Force clamping happens BEFORE gear scaling in the transmission phase.
  `actuator_force` is clamped, then `qfrc_actuator += gear * force`.
  This means forcerange limits the actuator-space force, not the joint-space
  torque. Important distinction.

---

### 9. `site-transmission` — Site-Based Wrench Actuator

**A general actuator that applies a wrench at a site on a 2-DOF arm.** Instead
of acting on a joint directly, the force is defined as a 6D wrench in the
site's local frame, projected to joint space via the site Jacobian. The key
demonstration: the moment arm on the shoulder joint is **configuration-
dependent** — it changes as the elbow bends.

**Why 2-DOF:** On a single hinge, any body-frame force produces a constant
generalized moment (the force and lever arm co-rotate). A second joint breaks
this symmetry — the end-effector position relative to the shoulder changes with
elbow angle, creating a varying moment arm on the shoulder DOF.

**Concept:** `<general site="...">` (no refsite) → `ActuatorTransmission::Site`,
Mode A. `gear` encodes the wrench direction: `[fx, fy, fz, tx, ty, tz]` in
site frame. Moment arm = `J_trans^T · R·gear_force + J_rot^T · R·gear_torque`
where R = site rotation matrix.

**MJCF model:**
```xml
<mujoco model="site-transmission">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Link 1: upper arm -->
    <body name="upper_arm" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.15" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="upper_rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.3"/>

      <!-- Link 2: forearm -->
      <body name="forearm" pos="0 0 -0.3">
        <joint name="elbow" type="hinge" axis="0 1 0" armature="0.005"/>
        <inertial pos="0 0 -0.1" mass="0.5" diaginertia="0.002 0.002 0.002"/>
        <geom name="lower_rod" type="capsule" size="0.015"
              fromto="0 0 0  0 0 -0.2"/>
        <geom name="hand" type="sphere" size="0.03" pos="0 0 -0.2"/>
        <site name="tip_site" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Motor on elbow to control configuration -->
    <position name="elbow_servo" joint="elbow" kp="20" dampratio="1"/>
    <!-- Site-transmission actuator: force at tip in site-local X -->
    <general name="site_force" site="tip_site"
             gear="1 0 0 0 0 0" gainprm="10"/>
  </actuator>

  <sensor>
    <actuatorfrc name="site_frc" actuator="site_force"/>
    <jointpos name="q_shoulder" joint="shoulder"/>
    <jointpos name="q_elbow" joint="elbow"/>
    <framepos name="site_pos" objtype="site" objname="tip_site"/>
  </sensor>
</mujoco>
```

**Setup:** Arm hangs straight down (q1=q2=0). The elbow servo slowly sweeps
q2 from 0 → π/2 (arm folds). The site-force actuator applies constant
`ctrl[1] = 1.0` throughout.

**Analytical predictions:**
- Arm fully extended (q2=0): site at `(0, 0, -0.5)`. Moment on shoulder
  = `-0.5` (full lever arm: 0.3 + 0.2 = 0.5m from shoulder).
- Arm folded 90° (q2=π/2): site at `(-0.2, 0, -0.3)`. Moment on shoulder
  changes to `-0.3` (horizontal reach reduced).
- Elbow moment is always `-0.2` (constant distance from elbow to tip,
  just like the 1-DOF case).
- `actuator_length = 0` always (site mode A).

| q2 (elbow) | Moment on shoulder | Moment on elbow |
|------------|-------------------|-----------------|
| 0° | -0.500 | -0.200 |
| 45° | -0.412 | -0.200 |
| 90° | -0.200 | -0.200 |

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Actuator length = 0 | Site mode A: length always 0 | exact |
| Shoulder moment varies | moment_j1(q2=0) ≈ -0.5, moment_j1(q2=π/2) ≈ -0.2 | < 3% |
| Elbow moment constant | moment_j2 ≈ -0.2 at all configurations | < 2% |
| Config-dependent | Shoulder torque from same ctrl changes with elbow angle | structural |
| Wrench in site frame | Force direction follows forearm rotation | structural |

**Engine issues to watch for:**
- `site_xmat` ordering: gear rotated by site rotation matrix.
  If site_xmat is transposed or column-major, wrench direction will be wrong.
- Jacobian must be computed at the site position, not the body CoM.
- Multi-DOF moment vector: the actuator_moment nv-vector must have correct
  entries for both shoulder and elbow DOFs simultaneously.

---

### 10. `slider-crank` — Mechanical Linkage Transmission

**A slider-crank mechanism that converts rotation into linear motion.** A crank
arm rotates about Y, driving a slider along X via a connecting rod. The
effective moment arm varies sinusoidally with crank angle, with dead centers
where the crank is aligned with the slider axis.

**Concept:** `<general cranksite="..." slidersite="..." cranklength="...">` →
`ActuatorTransmission::SliderCrank`. Length and moment computed from geometric
linkage equations using site positions and rod length.

**Geometry:** Standard textbook layout — crank rotates in the X-Z plane about Y,
slider translates along X. Crank radius `r = 0.1 m`, rod length `L = 0.2 m`,
slider pivot at `(0.25, 0, 0)`.

**MJCF model:**
```xml
<mujoco model="slider-crank">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Crank: rotates about Y-axis, arm extends along X -->
    <body name="crank" pos="0 0 0">
      <joint name="crank_joint" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0.05 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>
      <geom name="crank_arm" type="capsule" size="0.015"
            fromto="0 0 0  0.1 0 0"/>
      <site name="crank_pin" pos="0.1 0 0"/>
    </body>

    <!-- Slider: translates along X-axis, offset along X -->
    <body name="slider_body" pos="0.25 0 0">
      <joint name="slider_joint" type="slide" axis="1 0 0" armature="0.01"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <geom name="piston" type="box" size="0.04 0.02 0.02"/>
      <site name="slider_pin" pos="0 0 0"/>
    </body>
  </worldbody>

  <actuator>
    <general name="crank_motor" cranksite="crank_pin" slidersite="slider_pin"
             cranklength="0.2" gainprm="5" gear="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="crank_motor"/>
    <jointpos name="crank_angle" joint="crank_joint"/>
    <jointpos name="slider_pos" joint="slider_joint"/>
  </sensor>
</mujoco>
```

**Setup:** Gravity disabled. Apply `ctrl[0] = 1.0` to spin the crank.

**Analytical predictions:**
- Crank radius `r = 0.1 m`, rod length `L = 0.2 m`.
- `actuator_length = gear * (av - sqrt(det))` where:
  - `av = vec · axis` (projection of crank→slider vector onto slider X-axis)
  - `det = av² + L² - |vec|²`
- Moment arm (dl/dθ) varies with crank angle:
  - **Dead centers (θ=0, π):** crank aligned with slider axis → moment arm = 0.
    The crank pin velocity is perpendicular to the slider axis.
  - **Max advantage (θ ≈ π/2, 3π/2):** crank perpendicular to slider axis →
    peak moment arm ≈ 0.11. The crank pin velocity is parallel to slider axis.
- Length range: approximately -0.35 to -0.55 (varies by ~0.2 = 2r).
- Geometry stays valid: `det > 0` at all angles (verified numerically).

**Pass/fail checks:**
| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Length matches geometry | `actuator_length ≈ gear * kinematic_length(θ)` | < 1% |
| Variable moment arm | Moment arm changes magnitude with θ | structural |
| Dead center at θ≈0 | dl/dθ ≈ 0 when crank aligned with slider | < 0.005 |
| Max advantage at θ≈π/2 | Peak \|dl/dθ\| ≈ 0.10 | < 10% |
| Full revolution | Length traces a smooth periodic curve | structural |
| Gear scaling | Length and moment both scale by gear[0] | < 1e-10 |
| det > 0 always | No degenerate fallback triggered | structural |

**Engine issues to watch for:**
- Degenerate case: when `det ≤ 0` (rod too short to reach slider), the code
  falls back to `length = av`. The geometry is designed so this never triggers,
  but verify by checking `det > 0` at all sampled angles.
- Numerical precision near dead centers where moment arm → 0.
- The slider-crank code in `mj_transmission_slidercrank` uses full
  site Jacobians — this is the most complex transmission computation.
- The two joints (crank + slider) are coupled only through the actuator's
  moment vector, not through a kinematic constraint. The actuator distributes
  force to both DOFs based on the geometric derivatives.

---

## Code Path Coverage Matrix

| Example | Gain | Bias | Dynamics | Transmission |
|---------|------|------|----------|--------------|
| 1. motor | **Fixed** | **None** | **None** | **Joint** |
| 2. position-servo | Fixed | **Affine** | None | Joint |
| 3. velocity-servo | Fixed | Affine | None | Joint |
| 4. damper | **Affine** | None | None | Joint |
| 5. activation-filter | Fixed | Affine | **FilterExact** | Joint |
| 6. cylinder | Fixed | Affine | **Filter** | Joint |
| 7. integrator | Fixed | Affine | **Integrator** | Joint |
| 8. gear-and-limits | Fixed | None | None | Joint |
| 9. site-transmission | Fixed | None | None | **Site** |
| 10. slider-crank | Fixed | None | None | **SliderCrank** |

**Bold** = first time this code path is exercised.

## Implementation Order

The examples are ordered by complexity. Implement in sequence:

1. **motor** — Pure baseline. If this fails, everything else will too.
2. **position-servo** — Introduces Affine bias. Tests dampratio conversion.
3. **velocity-servo** — Same gain/bias types, different parameters.
4. **damper** — Introduces Affine gain. Different actuator "feel."
5. **activation-filter** — Introduces `data.act` state. First dynamics type.
6. **cylinder** — Second dynamics type (Filter vs FilterExact). Slide joint.
7. **integrator** — Third dynamics type. Tests `<general>` actuator.
8. **gear-and-limits** — Parameter variations. Two-arm comparison.
9. **site-transmission** — New transmission path. 2-DOF arm shows config-dependent moments.
10. **slider-crank** — Most complex. Geometric linkage math with dead centers.

## Future Candidates (not in this spec)

- **adhesion** — Body transmission. Requires contacts (crosses into collision
  domain). Better placed after contact-tuning examples exist.
- **joint-in-parent** — JointInParent transmission. Only differs from Joint for
  ball joints (torque in parent vs child frame). Subtle, specialized.
- **actearly** — The `actearly` flag (predict next-step activation) is a
  single-timestep effect. Hard to validate visually. Better as a unit test.
- **ctrl-interpolation** — `nsample`, `interp`, `delay` attributes. Runtime
  behavior not yet implemented (stored in model, no runtime effect).

## Notes

- All examples use the established pattern: inline MJCF, Bevy app, orbit
  camera, `ValidationHarness`, `PhysicsHud`, `Check`-based pass/fail report.
- Each example targets ~200–300 LOC.
- Validation runs for 15s, report prints at t=15.
- RK4 integrator for accuracy (matches sensor examples).
- Sensors included in each MJCF for pipeline verification (not just visual).
