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

- [ ] **mjcf_pendulum** — Simplest articulated MJCF model. First example where
  a joint is *not* a freejoint.

  *MJCF (inline):* 1 body, 1 hinge (axis Y), spring (stiffness=10,
  springref=0.3) + damper (0.5). Based on conformance `pendulum.xml` with
  visual-friendly sizes.

  *Physics:* Hinge from MJCF, passive spring + damping, energy tracking.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` auto-step. Capsule link +
  small sphere at pivot. Orbit camera. HUD: angle (deg), velocity, energy.

  *Acceptance:*
  - [ ] Oscillates about springref, amplitude decays
  - [ ] Energy monotonically decreases (damped system)
  - [ ] `J` shows joint axis gizmo (yellow line along Y)

  *New concepts:* hinge from MJCF, spring/damper passive forces, angle HUD.

- [ ] **mjcf_cartpole** — Mixed joint types (slide + hinge), motor actuator,
  keyboard control. Classic control benchmark.

  *MJCF (inline):* Cart body with slide joint (limited ±1.8m, damping=0.5).
  Pole body with hinge (axis Y, damping=0.002). Motor on slider (gear=10,
  ctrl ∈ [-1, 1]). RK4 integrator, contact disabled. Visual rail geom.

  *Physics:* Slide joint, hinge joint, motor actuator, joint limits.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` auto-step. Side view camera
  along -Y. Left/Right arrows → `data.ctrl[0] = ±1.0`, release → 0.0.
  HUD: cart pos, pole angle (deg), ctrl input, energy.

  *Acceptance:*
  - [ ] Cart slides with arrow keys, stops at ±1.8m limits
  - [ ] Pole swings freely, responds to cart acceleration
  - [ ] Pole can be balanced manually with quick left/right taps
  - [ ] `L` shows limit gizmos, `J` shows joint axes

  *New concepts:* slide joint, motor actuator + `data.ctrl`, keyboard →
  physics control loop, joint limits.

- [ ] **mjcf_double_pendulum** — Serial chain from MJCF with coupling dynamics.

  *MJCF (inline):* 2 bodies, 2 hinges (axis Y). Upper: stiffness=5,
  damping=0.3, springref=0.2. Lower: damping=0.2. Based on conformance
  `double_pendulum.xml` with visual-friendly sizes.

  *Physics:* 2×2 mass matrix with off-diagonal coupling (CRBA), Coriolis
  terms (RNE), chain FK, chaotic motion.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` auto-step. Two color-coded
  capsule links (blue upper, red lower). Orbit camera at upper pivot.
  HUD: θ₁, θ₂, energy, time.

  *Acceptance:*
  - [ ] Chaotic double-pendulum motion (qualitatively correct)
  - [ ] Links stay connected (no constraint drift)
  - [ ] Energy decreases monotonically (damped)

  *New concepts:* serial chain from MJCF (vs factory in Tier 2), validates
  MJCF → chain topology → FK end-to-end.

## Tier 5: Multi-body kinematic trees + ground contact

The critical bridge: articulated bodies interacting with the ground plane.
This is what the humanoid needs and what no prior tier proves.

- [ ] **ragdoll** — Branching kinematic tree with free base + articulated limbs
  + ground contact. Passive (no actuators). The exact failure mode of the
  current humanoid — if this works, the humanoid's core physics work.

  *MJCF (inline):* 9 bodies. Torso with freejoint (nq=7/nv=6). Ball joints
  at shoulders and hips (4×3-DOF). Hinge joints at elbows and knees (4×1-DOF,
  limited). Ground plane with contact enabled. Damping throughout, friction
  on all geoms.

  *Physics:* Free + ball + hinge joints, ground contact (capsule-plane),
  joint limits, passive damping. No actuators.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` auto-step + 4 substeps.
  Orbit camera at (0, 0, 1). Debug keys: C/N/F/J/L. HUD: body count,
  contact count, energy, time.

  *Acceptance:*
  - [ ] Falls from Z=2, limbs flop realistically
  - [ ] Lands on ground — no interpenetration, no bouncing forever
  - [ ] Settles to rest within ~5s (damping dissipates energy)
  - [ ] Elbows/knees respect limits (no hyperextension)
  - [ ] `C` shows contact points at ground
  - [ ] No energy divergence

  *Why critical:* Simplified humanoid (9 bodies vs 13) with same joint mix
  (free + ball + hinge) and same failure mode (articulated tree + contact).
  Debug here, not on the full humanoid.

  *New concepts:* branching tree from MJCF, ground contact with articulated
  bodies, joint limits in dynamic context, substeps for contact stability.

- [ ] **planar_walker** — Actuated locomotion platform. Ground contact, joint
  limits, motor control. Last stepping stone before the humanoid.

  *MJCF (inline):* 7 bodies. Planar root (2 slides X/Z + 1 hinge Y — no
  freejoint). Symmetric legs: thigh → shin → foot, each with hinge (Y axis)
  + range limits. 6 motor actuators with gear ratios (hip=100, knee=50,
  ankle=20). Based on dm_control walker topology.

  *Physics:* Planar root (slide+slide+hinge), 6 actuated hinges with limits,
  6 motors with gear ratios, ground contact (capsule-plane), armature.

  *Bevy:* `SimViewerPlugin` + `ModelDataPlugin` auto-step + 4 substeps.
  Side view camera tracking torso X. Keyboard: Q/A W/S E/D (right hip/knee/
  ankle), U/J I/K O/L (left), Space (zero all), R (reset pose). HUD: 6 joint
  angles, torso height, contact count, ctrl values.

  *Acceptance:*
  - [ ] Stands on ground at initial pose
  - [ ] Motor torques move joints within limits
  - [ ] Falls realistically when unbalanced
  - [ ] Feet-ground contact is stable (no jitter, no sinking)
  - [ ] `R` resets to standing pose
  - [ ] Energy bounded even with active control

  *New concepts:* planar root (slide+slide+hinge), multi-actuator control
  with gear ratios, per-joint keyboard input, pose reset, locomotion platform.

## Tier 6: Complex articulated systems

- [~] **mjcf_humanoid** — `sim_mjcf::load_model()` of 13-body humanoid.
  Free+ball+hinge joints (nq=43, nv=34). Launches but energy diverges —
  ground plane has `contype="0" conaffinity="0"` (contact disabled), bodies
  fall freely.

  *Required fixes (after ragdoll works):*
  1. Enable ground contact: remove `contype="0" conaffinity="0"` from ground
  2. Add 4 substeps for contact stability
  3. Tune damping if needed
  4. Verify energy is bounded

---

## Implementation order

Each validates infrastructure for the next:

1. **mjcf_pendulum** — fast win, proves articulated MJCF works
2. **mjcf_cartpole** — proves actuators + control + limits
3. **mjcf_double_pendulum** — proves chain from MJCF
4. **ragdoll** — proves contact + articulated (critical gate)
5. **planar_walker** — proves actuated locomotion
6. **mjcf_humanoid fix** — enable ground contact, done

Steps 1–3 are straightforward (no new physics, just new examples). Step 4 is
the hard one — if contact + articulated breaks, debug here on 9 bodies rather
than 13. Steps 5–6 build on 4.

## Feature progression

| Example | Bodies | Joints | Actuators | Contact | Interactive |
|---------|--------|--------|-----------|---------|-------------|
| mjcf_pendulum | 1 | hinge | — | — | — |
| mjcf_cartpole | 2 | slide+hinge | 1 motor | — | keyboard |
| mjcf_double_pend | 2 | 2×hinge | — | — | — |
| ragdoll | 9 | free+4×ball+4×hinge | — | capsule-plane | debug viz |
| planar_walker | 7 | 2×slide+7×hinge | 6 motors | capsule-plane | 12-key |
| mjcf_humanoid | 13 | free+8×ball+4×hinge | — | (needs fix) | debug viz |

---

## Status key
- [x] = Window opens, renders correctly, physics looks right
- [~] = Runs but has visual/physics issues (note what's wrong)
- [ ] = Not yet tested / not yet implemented
- [!] = Crashes or fails to run
