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

## Tier 4: Complex articulated systems

- [~] **mjcf_humanoid** — `sim_mjcf::load_model()` of 13-body humanoid.
  Free+ball+hinge joints (nq=43, nv=34). Launches but energy diverges —
  no ground contact, bodies fall freely. Last example to get right.

---

## Status key
- [x] = Window opens, renders correctly, physics looks right
- [~] = Runs but has visual/physics issues (note what's wrong)
- [ ] = Not yet tested
- [!] = Crashes or fails to run
