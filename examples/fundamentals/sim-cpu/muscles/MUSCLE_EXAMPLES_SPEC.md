# Muscle Examples Spec

**Status:** Draft — awaiting review
**Date:** 2026-03-30
**Covers:** MuJoCo `<muscle>` (conformant) + CortenForge `HillMuscle` (extension)
**Prerequisite:** Phase 5 Spec C (landed), all existing muscle unit tests passing

---

## Scope

This spec defines the stress test and visual examples for the muscle
actuator system. Two distinct muscle models exist in the engine:

1. **MuJoCo `<muscle>`** — piecewise-quadratic FL/FV/FP curves,
   conformant with MuJoCo 3.5.0.
2. **CortenForge `HillMuscle`** — Gaussian FL, Hill hyperbolic FV,
   exponential FP, with pennation angle. Extension, not in MuJoCo.

Both share identical activation dynamics (Millard et al. 2013).

---

## Model: Two-Curve Forearm

All checks (unless noted) use the same physical setup: a forearm hinged
at the elbow, with a muscle actuator spanning the joint. This keeps the
model simple enough that every force can be computed analytically.

```
World
  └─ body "upper_arm" (fixed to world)
       └─ body "forearm" (hinge joint "elbow", axis Y)
            ├─ inertial: mass=1.0 kg, COM at (0, 0, -0.15)
            ├─ geom: capsule (visual)
            └─ muscle actuator on joint "elbow"
```

- **Hinge axis:** Y (rotation in XZ plane)
- **Joint range:** `[-1.57, 1.57]` (±90°)
- **Forearm length:** 0.3 m, COM at midpoint (0.15 m from pivot)
- **Gravity:** `[0, 0, -9.81]` (forearm hangs down at q=0)
- **Timestep:** 0.001 s
- **Integrator:** RK4

---

## Reference: Curve Equations

### Shared Activation Dynamics

Both `Muscle` and `HillMuscle` use `muscle_activation_dynamics()`:

```
ctrl_clamped = clamp(ctrl, 0, 1)
act_clamped  = clamp(act, 0, 1)
tau_act_eff  = dynprm[0] × (0.5 + 1.5 × act_clamped)
tau_deact_eff = dynprm[1] / (0.5 + 1.5 × act_clamped)
dctrl = ctrl_clamped - act   (unclamped act for derivative)

if tausmooth < 1e-10:
    tau = tau_act_eff  if dctrl > 0  else  tau_deact_eff    (hard switch)
else:
    x = clamp(0.5 × (dctrl / tausmooth + 1), 0, 1)
    S(x) = x³ × (6x² − 15x + 10)                          (quintic Hermite)
    tau = tau_deact_eff + (tau_act_eff − tau_deact_eff) × S(x)

act_dot = dctrl / max(1e-10, tau)
```

Default dynprm: `[tau_act=0.01, tau_deact=0.04, tausmooth=0.0]`

### MuJoCo Muscle Curves

**Active force-length** (`muscle_gain_length`): Piecewise-quadratic bump.
Peak 1.0 at L=1.0, zero outside [lmin, lmax].

```
a = 0.5 × (lmin + 1.0)
b = 0.5 × (1.0 + lmax)

FL(L) =
  L < lmin or L > lmax  → 0.0
  L ≤ a                 → 0.5 × ((L − lmin) / (a − lmin))²
  L ≤ 1.0               → 1.0 − 0.5 × ((1.0 − L) / (1.0 − a))²
  L ≤ b                 → 1.0 − 0.5 × ((L − 1.0) / (b − 1.0))²
  L ≤ lmax              → 0.5 × ((lmax − L) / (lmax − b))²
```

Default: lmin=0.5, lmax=1.6 → a=0.75, b=1.3.

**Force-velocity** (`muscle_gain_velocity`): Piecewise-quadratic.

```
y = fvmax − 1.0

FV(V) =
  V ≤ −1.0  → 0.0
  V ≤ 0.0   → (V + 1)²
  V ≤ y     → fvmax − (y − V)² / y
  V > y     → fvmax
```

Default: fvmax=1.4 (clamped to ≥1.0).

**Passive force** (`muscle_passive_force`): Half-quadratic + linear.

```
b = 0.5 × (1.0 + lmax)

FP(L) =
  L ≤ 1.0  → 0.0
  L ≤ b    → fpmax × 0.5 × ((L − 1.0) / (b − 1.0))²
  L > b    → fpmax × (0.5 + (L − b) / (b − 1.0))
```

Default: fpmax=0.3, lmax=1.6 → b=1.3.

**Force formula:**

```
L0 = (lr[1] − lr[0]) / (range[1] − range[0])
norm_len = range[0] + (actuator_length − lr[0]) / L0
norm_vel = actuator_velocity / (L0 × vmax)

gain = −F0 × FL(norm_len) × FV(norm_vel)
bias = −F0 × FP(norm_len)
force = gain × activation + bias
```

gainprm: `[range_lo, range_hi, F0, scale, lmin, lmax, vmax, fpmax, fvmax]`

### HillMuscle Curves (CortenForge Extension)

**Active force-length** (`hill_active_fl`): Asymmetric Gaussian.

```
w = w_asc (=0.45) if L ≤ 1.0 else w_desc (=0.56)

FL(L) =
  L < 0.5 or L > 1.6  → 0.0
  otherwise            → exp(−((L − 1.0) / w)²)
```

**Force-velocity** (`hill_force_velocity`): Hill hyperbolic.

```
a = 0.25        (curvature parameter)
fv_max = 1.5    (eccentric plateau)

FV(V) =
  V ≤ −1.0  → 0.0
  V ≤ 0.0   → (1 + V) / (1 − V/a)       (concentric: Hill hyperbola)
  V > 0.0   → 1 + (fv_max − 1) × V / (V + a)  (eccentric: plateau approach)
```

**Passive force-length** (`hill_passive_fl`): Exponential.

```
shape = 4.0

FP(L) =
  L ≤ 1.0  → 0.0
  L > 1.0  → (exp(shape × (L − 1.0)) − 1) / (exp(shape) − 1)
```

**Force formula (rigid tendon):**

```
fiber_length   = (MTU_length − L_slack) / cos(α)
fiber_velocity = MTU_velocity / cos(α)
norm_len = fiber_length / L_opt
norm_vel = fiber_velocity / (L_opt × vmax)

gain = −F0 × FL(norm_len) × FV(norm_vel) × cos(α)
bias = −F0 × FP(norm_len) × cos(α)
force = gain × activation + bias
```

gainprm: `[range_lo, range_hi, F0, scale, L_opt, L_slack, vmax, penn, stiff]`

Defaults: `[0.75, 1.05, -1.0, 200.0, 0.10, 0.20, 10.0, 0.0, 35.0]`

### Hardcoded Constants (Not Parameterized)

These are baked into the curve functions. The stress test must pin
exact values to detect accidental changes:

| Constant | Value | Location | Purpose |
|----------|-------|----------|---------|
| `w_asc` | 0.45 | `hill_active_fl` | Gaussian ascending width |
| `w_desc` | 0.56 | `hill_active_fl` | Gaussian descending width |
| `l_min` | 0.5 | `hill_active_fl` | FL range lower bound |
| `l_max` | 1.6 | `hill_active_fl` | FL range upper bound |
| `a` | 0.25 | `hill_force_velocity` | Hill curvature |
| `fv_max` | 1.5 | `hill_force_velocity` | Eccentric plateau |
| `shape` | 4.0 | `hill_passive_fl` | Exponential steepness |

---

## Stress Test Checks

### Group A: Activation Dynamics (shared — applies to both models)

**A1. Activation asymmetry**
- Setup: dynprm=[0.01, 0.04, 0.0], hard-switch mode
- Ramp ctrl=1 from act=0 until act≥0.9 (activation rise)
- Ramp ctrl=0 from act=1 until act≤0.1 (deactivation fall)
- **Check:** rise steps < fall steps (activation is faster than deactivation)
- **Tolerance:** rise steps must be strictly less (no tolerance needed)

**A2. Activation hard-switch exact values**
- Input: ctrl=0.8, act=0.3, dynprm=[0.01, 0.04, 0.0]
- act_clamped=0.3, tau_act=0.01×(0.5+0.45)=0.0095, dctrl=0.5>0
- **Check:** act_dot = 0.5 / 0.0095 = 52.631578947368421
- **Tolerance:** 1e-10

**A3. Activation deactivation exact value**
- Input: ctrl=0.2, act=0.6, dynprm=[0.01, 0.04, 0.0]
- act_clamped=0.6, tau_deact=0.04/(0.5+0.9)=0.028571..., dctrl=−0.4
- **Check:** act_dot = −0.4 / 0.028571... = −14.0
- **Tolerance:** 1e-10

**A4. Activation at act=0 boundary**
- Input: ctrl=0.5, act=0.0, dynprm=[0.01, 0.04, 0.0]
- tau_act=0.01×0.5=0.005, dctrl=0.5
- **Check:** act_dot = 0.5 / 0.005 = 100.0
- **Tolerance:** 1e-10

**A5. Activation at act=1 boundary**
- Input: ctrl=0.5, act=1.0, dynprm=[0.01, 0.04, 0.0]
- tau_deact=0.04/2.0=0.02, dctrl=−0.5
- **Check:** act_dot = −0.5 / 0.02 = −25.0
- **Tolerance:** 1e-10

**A6. Ctrl clamping**
- Input: ctrl=1.5, act=0.5 → should produce identical act_dot to ctrl=1.0, act=0.5
- **Check:** act_dot(1.5, 0.5) == act_dot(1.0, 0.5)
- **Tolerance:** exact (1e-15)

**A7. Smooth blend**
- Input: ctrl=0.55, act=0.5, dynprm=[0.01, 0.04, 0.2]
- **Check:** act_dot ≈ 2.7987 (within 1e-3)
- **Tolerance:** 1e-3

### Group B: MuJoCo Muscle Curve Pinning

**B1. MuJoCo FL curve — 5 pinned values**
- With lmin=0.5, lmax=1.6 (defaults):
  - `FL(0.5)` = 0.0 (at lmin boundary)
  - `FL(0.75)` = 0.5 (at midpoint a = 0.75)
  - `FL(1.0)` = 1.0 (peak)
  - `FL(1.3)` = 0.5 (at midpoint b = 1.3)
  - `FL(1.6)` = 0.0 (at lmax boundary)
- **Tolerance:** 1e-10

**B2. MuJoCo FL curve — outside range**
- `FL(0.3)` = 0.0, `FL(1.8)` = 0.0
- **Tolerance:** exact (0.0)

**B3. MuJoCo FV curve — 4 pinned values**
- With fvmax=1.4:
  - `FV(−1.0)` = 0.0 (max shortening)
  - `FV(−0.5)` = 0.25 (concentric: `(−0.5+1)² = 0.25`)
  - `FV(0.0)` = 1.0 (isometric)
  - `FV(0.4)` = 1.4 (eccentric plateau, since y=0.4 and V≥y)
- **Tolerance:** 1e-10

**B4. MuJoCo FP curve — 4 pinned values**
- With lmax=1.6, fpmax=0.3:
  - `FP(0.8)` = 0.0 (below optimal)
  - `FP(1.0)` = 0.0 (at optimal)
  - `FP(1.3)` = 0.15 (at midpoint b=1.3: fpmax × 0.5 = 0.15)
  - `FP(1.5)` = 0.15 + 0.3 × (1.5−1.3)/(1.3−1.0) = 0.35
- **Tolerance:** 1e-10

### Group C: HillMuscle Curve Pinning

**C1. Hill FL curve — 6 pinned values**
- `FL(0.49)` = 0.0 (below l_min)
- `FL(0.5)` > 0.0 (at l_min boundary, nonzero)
- `FL(0.8)` = exp(−((0.8−1.0)/0.45)²) = exp(−0.197531) ≈ 0.820754
- `FL(1.0)` = 1.0 (peak)
- `FL(1.2)` = exp(−((1.2−1.0)/0.56)²) = exp(−0.127551) ≈ 0.880163
- `FL(1.61)` = 0.0 (above l_max)
- **Tolerance:** 1e-5 for computed values, exact for boundary zeros

**C2. Hill FV curve — 5 pinned values**
- `FV(−1.0)` = 0.0 (max shortening)
- `FV(−0.5)` = (1−0.5)/(1+0.5/0.25) = 0.5/3.0 ≈ 0.166667
- `FV(0.0)` = 1.0 (isometric)
- `FV(0.5)` = 1 + (1.5−1)×0.5/(0.5+0.25) = 1 + 0.333... ≈ 1.333333
- `FV(1.0)` = 1 + 0.5×1.0/(1.0+0.25) = 1.4 (approaching plateau)
- **Tolerance:** 1e-5

**C3. Hill FP curve — 4 pinned values**
- `FP(0.9)` = 0.0 (below optimal)
- `FP(1.0)` = 0.0 (at optimal)
- `FP(1.25)` = (exp(4×0.25)−1)/(exp(4)−1) = (e¹−1)/(e⁴−1) ≈ 0.032758
- `FP(1.5)` = (exp(4×0.5)−1)/(exp(4)−1) = (e²−1)/(e⁴−1) ≈ 0.122564
- **Tolerance:** 1e-5

### Group D: Full-Pipeline Force (MuJoCo Muscle)

**D1. MuJoCo muscle — isometric at optimal length**
- Setup: `<muscle>` with F0=500, lmin=0.5, lmax=1.6, fvmax=1.4
- Joint at midrange so norm_len=1.0, velocity=0 so norm_vel=0
- ctrl=1.0, simulate until activation ≈ 1.0
- FL(1.0)=1.0, FV(0.0)=1.0
- **Check:** actuator_force ≈ −500 (gain×act + bias, passive FP=0 at L=1.0)
- **Tolerance:** 5% of F0

**D2. MuJoCo muscle — passive force at stretch**
- Setup: same muscle, joint pushed beyond optimal so norm_len > 1.0
- ctrl=0.0, activation ≈ 0 (no active force)
- **Check:** actuator_force < 0 (passive force resists stretch)
- **Check:** actuator_force magnitude increases with stretch

### Group E: Full-Pipeline Force (HillMuscle)

**E1. HillMuscle — isometric at optimal fiber length, zero pennation**
- Setup: `<general dyntype="hillmuscle">` with explicit F0=1000,
  L_opt=0.10, L_slack=0.20, penn=0.0
- Joint position such that MTU_length = L_slack + L_opt = 0.30
  → fiber_length = 0.10, norm_len = 1.0
- velocity=0, ctrl=1.0, simulate until activation ≈ 1.0
- FL(1.0)=1.0, FV(0.0)=1.0, cos(0)=1.0
- **Check:** actuator_force ≈ −1000
- **Tolerance:** 1% of F0

**E2. HillMuscle — pennation angle reduces force**
- Setup: same as E1 but penn=20° (0.349 rad)
- cos(20°) ≈ 0.9397
- **Check:** actuator_force ≈ −1000 × 0.9397 ≈ −939.7
- **Tolerance:** 1% of expected

**E3. HillMuscle — passive force at stretch**
- Setup: HillMuscle, ctrl=0.0 (activation≈0), joint position such that
  norm_len = 1.25
- Expected passive: FP(1.25) ≈ 0.032758, bias = −F0 × FP × cos(α)
- **Check:** actuator_force ≈ −F0 × 0.032758 (for α=0)
- **Tolerance:** 5%

**E4. HillMuscle — F0 auto-computation**
- Setup: HillMuscle with gainprm[2] = −1.0 (auto), scale=200
- After model build, F0 should be set to scale/acc0
- **Check:** model.actuator_gainprm[i][2] > 0 (auto-resolved)
- **Check:** model.actuator_gainprm[i][2] ≈ 200.0 / model.actuator_acc0[i]
- **Tolerance:** 1e-6

**E5. HillMuscle — activation dynamics identical to Muscle**
- Setup: Two actuators on same joint — one `<muscle>`, one
  `<general dyntype="hillmuscle">` — same dynprm
- ctrl=0.7 for both
- **Check:** act_dot[muscle] == act_dot[hillmuscle] (exact equality)

### Group F: Dynamic Behavior

**F1. Activation rise time constant**
- Setup: MuJoCo `<muscle>` with dynprm=[0.01, 0.04, 0.0]
- ctrl steps from 0→1 at t=0, track activation over time
- Time to reach 63.2% (1 − 1/e) of steady state
- **Check:** rise time within 20% of effective tau_act at midpoint
  activation. (Nonlinear due to activation-dependent tau, so not exact
  1st-order, but should be in the right ballpark.)

**F2. Forearm flexion — muscle lifts load**
- Setup: forearm hanging at q=0 (vertical down), HillMuscle on elbow
- ctrl=1.0 ramp
- **Check:** after 2s, elbow angle > 0 (forearm has flexed upward)
- **Check:** actuator_force is negative (muscle pulling)

**F3. Cocontraction — opposing muscles**
- Setup: Two HillMuscle actuators on same joint, opposite gear signs
  (agonist gear=1, antagonist gear=−1)
- Both ctrl=1.0 (full activation)
- **Check:** joint stays near equilibrium (net torque ≈ 0)
- **Check:** both actuator_forces are nonzero (both muscles active)
- **Tolerance:** joint displacement < 0.1 rad from initial

---

## Check Summary

| Group | Checks | What it proves |
|-------|--------|----------------|
| A: Activation dynamics | 7 | Shared dynamics correct, exact numerical values pinned |
| B: MuJoCo curves | 4 | Piecewise-quadratic FL/FV/FP pinned at key points |
| C: Hill curves | 3 | Gaussian FL, Hill FV, exponential FP pinned at key points |
| D: MuJoCo pipeline | 2 | Full pipeline: MJCF → model → force computation |
| E: Hill pipeline | 5 | Full pipeline: MJCF → model → force computation + pennation |
| F: Dynamic behavior | 3 | Time-domain behavior: rise time, flexion, cocontraction |
| **Total** | **24** | |

---

## Example Plan

### 1. `muscles/stress-test` — Headless validation (24 checks)

All 24 checks above. No Bevy, no visuals. `cargo run -p example-muscle-stress-test --release`.
Exit code 1 on any failure.

### 2. `muscles/activation` — Activation Dynamics Visual

Side-by-side comparison of activation rise and fall. Three arms with
different time constants. HUD shows activation, ctrl, and act_dot.
Demonstrates the asymmetry between activation and deactivation.

### 3. `muscles/force-length` — Force-Length Curve Visual

Single arm held at various joint angles (swept slowly). HUD shows
normalized fiber length and corresponding FL value for both MuJoCo
and HillMuscle curves. Demonstrates the Gaussian vs piecewise-quadratic
shape difference.

### 4. `muscles/forearm-flexion` — Forearm Lifting

Forearm hanging down, HillMuscle ramps to full activation. Camera shows
the arm curling upward. HUD shows activation, muscle force, and joint
angle over time.

### 5. `muscles/cocontraction` — Agonist-Antagonist

Two opposing muscles on same joint. Both activated. Joint stiffens at
equilibrium. HUD shows both forces, net torque, and joint angle.

---

## Implementation Order

1. **Stress test first.** Implements and validates all 24 checks.
   Any engine bugs surface here, not in visual examples.
2. **Visual examples second.** Built on a proven foundation.
   Each visual example is just a stress test check with a camera.

---

## MJCF Templates

### MuJoCo `<muscle>` Template

```xml
<mujoco model="mujoco-muscle-test">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <muscle name="bicep" joint="elbow" force="500"
            range="0.75 1.05" lmin="0.5" lmax="1.6"
            vmax="1.0" fpmax="0.3" fvmax="1.4"
            timeconst="0.01 0.04"/>
  </actuator>
</mujoco>
```

### HillMuscle `<general>` Template

```xml
<mujoco model="hill-muscle-test">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="upper_arm" pos="0 0 1">
      <body name="forearm" pos="0 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               range="-1.57 1.57" limited="true"/>
        <inertial pos="0 0 -0.15" mass="1.0"
                  diaginertia="0.015 0.015 0.001"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <general name="bicep_hill" joint="elbow"
             dyntype="hillmuscle"
             gainprm="0.75 1.05 1000 200 0.10 0.20 10.0 0.0 35.0"
             dynprm="0.01 0.04 0.0"/>
  </actuator>
</mujoco>
```

---

## Notes

- The stress test pins exact numerical values for all three curve
  families. If a hardcoded constant changes (e.g., `w_asc` from 0.45
  to 0.50), the pinned values will fail immediately. This is intentional.
- Groups D and E test the *full pipeline*: MJCF parsing → model build →
  parameter resolution (acc0, F0, lengthrange) → forward dynamics →
  force computation. They catch integration bugs that curve-level
  unit tests miss.
- Group F tests time-domain behavior — the muscle actually doing
  something physical. These are the checks that would catch a sign
  error or a broken transmission.
