# Bevy Examples Checklist

Visual verification of all sim-bevy examples.
Run each with `cargo run -p sim-bevy --example <name> --release`.

---

## Tier 1: Standalone physics (no sim-core)

- [x] **simple_pendulum** — Hand-rolled 2D pendulum with semi-implicit Euler.
  Red bob swings on a rod, gray ground reference line. Energy drift < 1%.

## Tier 2: sim-core Model/Data pipeline

- [x] **model_data_demo** — `Model::n_link_pendulum(1)` factory.
  Single red bob on rod, xipos-based sync, orbit camera. Energy ~0.2% drift.

- [x] **double_pendulum** — `Model::double_pendulum()` factory.
  Red + blue bobs with connecting rods, orbit camera, RK4 integrator.
  Links stay constant (L₁=L₂=1.0). Chaotic motion looks physically correct.

- [x] **nlink_pendulum** — `Model::n_link_pendulum(3)` factory.
  3 color-gradient bobs on a chain, orbit camera, RK4 + light damping.
  Smooth oscillation, stable 30s+. Proper rod inertia (m*L²/12).

- [x] **spherical_pendulum** — `Model::spherical_pendulum()` with ball joint.
  Golden bob with red trail, orbit camera, RK4 integrator. 3D precession,
  constraint sphere perfect (|r|=1.5000). Energy drift < 1%.

## Tier 3: MJCF loading + SimViewerPlugin

- [x] **falling_sphere** — `SimViewerPlugin` + MJCF sphere + ground plane.
  Sphere falls from Z=5, bounces, settles on ground. Contact physics correct.

- [x] **collision_shapes** — `SimViewerPlugin` + MJCF shapes in grid.
  All 8 shapes (sphere, box, capsule, cylinder, ellipsoid, tall box, small sphere,
  wide cylinder) fall and settle on ground plane. Capsule tilted 15° to topple.

- [x] **contact_debug** — `SimViewerPlugin` + debug gizmo overlay.
  4 spheres + 1 box fall and settle correctly. No floating bodies.

## Tier 4: Articulated MJCF + Actuation

Bridge from free-floating shapes to MJCF models with constrained joints and
motors. No ground contact — focus on articulated kinematics and control.

- [x] **mjcf_pendulum** — Simplest articulated MJCF model. First example where
  a joint is *not* a freejoint.

  *MJCF (inline):* 1 body, 1 hinge (axis Y), spring (stiffness=2,
  springref=0) + damper (0.3). Based on conformance `pendulum.xml` with
  visual-friendly sizes (0.8m downward-hanging link, scaled spring params).

  *Physics:* Hinge from MJCF, passive spring + damping, energy tracking.

  *Bevy:* `ModelDataPlugin` + wall-clock stepping. Capsule link +
  small sphere at pivot. Orbit camera. HUD: angle (deg), velocity, energy.

  *Acceptance:*
  - [x] Oscillates about equilibrium, amplitude decays
  - [x] Energy monotonically decreases (damped system)
  - [x] `J` shows joint axis gizmo (yellow line along Y)

  *New concepts:* hinge from MJCF, spring/damper passive forces, angle HUD.

- [x] **mjcf_cartpole** — Mixed joint types (slide + hinge), motor actuator,
  keyboard control. Classic control benchmark.

  *MJCF (inline):* Cart body with slide joint (limited ±3.6m, damping=8.0).
  Pole body with hinge (axis Y, damping=1.0). Motor on slider (gear=40,
  ctrl ∈ [-1, 1]). Pole is 1.0m, 0.5kg — tuned for human playability
  (high slider damping for snappy direction changes, moderate hinge damping
  so pole falls slowly but swings naturally). RK4 integrator, contact
  disabled. Visual rail geom.

  *Physics:* Slide joint, hinge joint, motor actuator, joint limits.

  *Bevy:* `ModelDataPlugin` + wall-clock stepping. Side view camera
  along -Y. Left/Right arrows → `data.ctrl[0] = ±1.0`, release → 0.0.
  HUD: cart pos, pole angle (deg), ctrl input, energy.

  *Acceptance:*
  - [x] Cart slides with arrow keys, stops at ±3.6m limits
  - [x] Pole swings freely, responds to cart acceleration
  - [x] Pole can be balanced manually with quick left/right taps
  - [x] `L` shows limit gizmos, `J` shows joint axes

  *New concepts:* slide joint, motor actuator + `data.ctrl`, keyboard →
  physics control loop, joint limits.

- [x] **mjcf_double_pendulum** — Serial chain from MJCF with coupling dynamics.

  *MJCF (inline):* 2 bodies, 2 hinges (axis Y). Upper: stiffness=5,
  damping=0.3, springref=0.2. Lower: damping=0.2. Based on conformance
  `double_pendulum.xml` with visual-friendly sizes.

  *Physics:* 2×2 mass matrix with off-diagonal coupling (CRBA), Coriolis
  terms (RNE), chain FK, chaotic motion.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` auto-step. Two color-coded
  capsule links (blue upper, red lower). Orbit camera at upper pivot.
  HUD: θ₁, θ₂, energy, time.

  *Acceptance:*
  - [x] Chaotic double-pendulum motion (qualitatively correct)
  - [x] Links stay connected (no constraint drift)
  - [x] Energy decreases monotonically (damped)

  *New concepts:* serial chain from MJCF (vs factory in Tier 2), validates
  MJCF → chain topology → FK end-to-end.

## Tier 5: Precision Mechanisms

Analytically verifiable mechanical systems. Every example has a closed-form
expected behavior — no subjective "does this look right?" judgments.

- [ ] **spring_mass_damper** — Mass on slide joint with spring, damper, and
  ground contact. The simplest articulated + contact example with an exact
  analytical answer.

  *MJCF (inline):* 1 body (mass block), 1 slide joint (Z axis), spring
  (stiffness=50) + damper (damping=2.0). Ground plane with contact enabled.
  Initial position above equilibrium (Z=1.5). RK4 integrator, dt=0.001.

  *Physics:* Slide joint, passive spring + damper, ground contact
  (box-plane), gravity. Analytically: ω_n = √(k/m), ζ = c/(2√(km)),
  ω_d = ω_n√(1−ζ²), x(t) = A·e^(−ζω_n·t)·cos(ω_d·t + φ).

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` wall-clock stepping.
  Side view camera. HUD: position, velocity, measured frequency,
  predicted frequency, error %. Trail showing oscillation envelope.

  *Acceptance:*
  - [ ] Mass oscillates vertically on spring
  - [ ] Measured frequency matches ω_d = ω_n√(1−ζ²) within 1%
  - [ ] Amplitude decay envelope matches e^(−ζω_n·t) within 2%
  - [ ] If dropped from above rest, bounces off ground then oscillates
  - [ ] Energy monotonically decreases
  - [ ] `R` resets to initial height

  *New concepts:* slide joint + spring + contact, analytical verification
  in HUD, measurable frequency/decay comparison.

- [ ] **2dof_arm** — 2-link planar robot arm with 2 motor actuators and
  keyboard control. FK is exact trigonometry.

  *MJCF (inline):* 2 bodies (upper arm L₁=1.0m, forearm L₂=0.8m). 2 hinge
  joints (axis Y). 2 motor actuators with gear ratios. Joint limits to
  prevent self-collision. Damping for stability. No gravity (horizontal
  plane) or with gravity (vertical plane — more interesting). RK4.

  *Physics:* 2 hinge joints, 2 motors, joint limits, CRBA mass matrix
  with off-diagonal coupling. FK: x = L₁cos(θ₁) + L₂cos(θ₁+θ₂),
  z = L₁sin(θ₁) + L₂sin(θ₁+θ₂). Workspace is annulus with
  r_inner = |L₁−L₂| = 0.2, r_outer = L₁+L₂ = 1.8.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` wall-clock stepping.
  Side view camera. Q/A = joint 1 ±, W/S = joint 2 ±. End-effector
  trail (gizmo line). HUD: θ₁, θ₂, end-effector (x,z) measured vs FK
  predicted, |error|, workspace radius.

  *Acceptance:*
  - [ ] Keyboard drives both joints smoothly
  - [ ] End-effector position matches FK formula within 1e-6
  - [ ] Trail traces correct workspace region
  - [ ] Joint limits prevent self-intersection
  - [ ] Mass matrix coupling visible (moving joint 1 affects joint 2)
  - [ ] `R` resets to home pose

  *New concepts:* multi-actuator control, FK verification against
  analytical formula, end-effector tracking, workspace visualization.

- [ ] **gyroscope** — Spinning disk on a pivot, free to precess under gravity.
  Precession rate is analytically exact.

  *MJCF (inline):* 2 bodies. Base (fixed or heavy). Disk body on a short
  rod with ball joint at pivot. Disk has high moment of inertia about spin
  axis. Initial angular velocity ω_spin about spin axis via qvel. Gravity
  enabled. RK4, dt=0.001.

  *Physics:* Ball joint (or free joint on rod), gravity torque, gyroscopic
  precession. Precession rate: ω_p = mgh/(I_spin · ω_spin). Nutation
  visible at lower spin speeds.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` wall-clock stepping.
  Orbit camera. HUD: spin rate, measured precession rate, predicted
  precession rate, error %, angular momentum magnitude (should be
  ~conserved).

  *Acceptance:*
  - [ ] Disk precesses around vertical axis (visible circular motion)
  - [ ] Measured precession period matches τ/(I_spin · ω_spin) within 5%
  - [ ] Angular momentum magnitude conserved within 1%
  - [ ] Higher spin speed → slower precession (inverse relationship)
  - [ ] `R` resets with initial spin

  *New concepts:* 3D rotational dynamics, angular momentum conservation,
  gyroscopic precession, analytical precession rate verification.

## Tier 6: Coupled & Constrained Systems

Equality constraints, tendons, and multi-body coupling. Proves the constraint
solver handles coupling between distinct kinematic chains.

- [ ] **coupled_pendulums** — Two pendulums linked by a tendon or equality
  constraint. Energy transfers between them in a measurable beating pattern.

  *MJCF (inline):* 2 independent pendulum bodies, each with hinge joint.
  Slightly different natural frequencies (different lengths or masses).
  Coupled by tendon or equality joint constraint. Light damping.

  *Physics:* 2 hinge joints, tendon/equality coupling, energy exchange.
  Beat frequency: f_beat = |f₁ − f₂| / 2. Energy sloshes between
  pendulums at beat frequency — analytically exact.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` wall-clock stepping.
  Front view camera. HUD: θ₁, θ₂, E₁, E₂, measured beat frequency,
  predicted beat frequency, error %.

  *Acceptance:*
  - [ ] Both pendulums oscillate
  - [ ] Energy visibly transfers between them (one slows as other speeds up)
  - [ ] Measured beat frequency matches |f₁ − f₂|/2 within 5%
  - [ ] Total energy conserved (minus damping losses)
  - [ ] Coupling strength affects beat amplitude

  *New concepts:* equality constraints or tendons in dynamic context,
  energy exchange between coupled systems, beat frequency verification.

- [ ] **crank_slider** — Motor-driven crank converts rotary motion to linear
  sliding. Classic mechanism with analytically exact piston position.

  *MJCF (inline):* 3 bodies. Crank arm (hinge joint, motor-driven).
  Connecting rod (body with equality connect constraints at both ends).
  Slider (slide joint, X axis). Motor on crank hinge with constant
  velocity or torque.

  *Physics:* Hinge joint (crank), slide joint (piston), equality connect
  constraints (rod endpoints), motor actuator. Piston position:
  x(θ) = r·cos(θ) + √(L² − r²·sin²(θ)) where r = crank radius,
  L = rod length, θ = crank angle.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` wall-clock stepping.
  Side view camera. HUD: crank angle θ, piston x measured vs x predicted,
  |error|, crank RPM.

  *Acceptance:*
  - [ ] Crank rotates continuously under motor drive
  - [ ] Piston slides back and forth in sync with crank
  - [ ] Measured piston position matches analytical formula within 1e-4
  - [ ] Rod stays connected at both ends (no constraint drift)
  - [ ] Smooth periodic motion at steady state
  - [ ] HUD error stays below threshold

  *New concepts:* equality connect constraints, closed-loop mechanism,
  motor-driven periodic motion, analytical position curve verification.

---

## Implementation order

Each validates infrastructure for the next:

1. ~~**mjcf_pendulum**~~ — articulated MJCF works ✓
2. ~~**mjcf_cartpole**~~ — actuators + control + limits ✓
3. ~~**mjcf_double_pendulum**~~ — chain from MJCF ✓
4. **spring_mass_damper** — slide + spring + contact, analytical verification
5. **2dof_arm** — multi-actuator + FK verification
6. **gyroscope** — 3D rotational dynamics + angular momentum
7. **coupled_pendulums** — equality constraints / tendons
8. **crank_slider** — closed-loop mechanism + connect constraints

Steps 4–5 are straightforward (existing joint types + motors, just new
examples with analytical checks). Step 6 tests 3D rotation more deeply.
Steps 7–8 are the hard ones — first use of equality constraints in examples.

## Feature progression

| Example | Bodies | Joints | Actuators | Contact | Verification |
|---------|--------|--------|-----------|---------|--------------|
| mjcf_pendulum | 1 | hinge | — | — | energy decay |
| mjcf_cartpole | 2 | slide+hinge | 1 motor | — | balance ctrl |
| mjcf_double_pend | 2 | 2×hinge | — | — | energy decay |
| spring_mass_damper | 1 | slide | — | box-plane | ω_d, ζ exact |
| 2dof_arm | 2 | 2×hinge | 2 motors | — | FK formula |
| gyroscope | 2 | ball | — | — | ω_p = τ/(Iω) |
| coupled_pendulums | 2 | 2×hinge | — | — | f_beat exact |
| crank_slider | 3 | hinge+slide | 1 motor | — | x(θ) exact |

---

## Status key
- [x] = Window opens, renders correctly, physics looks right
- [~] = Runs but has visual/physics issues (note what's wrong)
- [ ] = Not yet tested / not yet implemented
- [!] = Crashes or fails to run
