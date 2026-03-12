# Bevy Examples Checklist

Visual verification of all sim-bevy examples.
Run each with `cargo run -p sim-bevy --example <name> --release`.

---

## Tier 1: Standalone physics (no sim-core)

- [x] **simple_pendulum** — Hand-rolled 2D pendulum with semi-implicit Euler.
  Red bob swings on a rod, gray ground reference line. Energy drift < 1%.

## Tier 2: sim-core Model/Data pipeline

- [ ] **model_data_demo** — Programmatic `Model::empty()` + manual fields.
  Single red bob, `step_model_data` + `sync_model_data_to_bevy`. Verifies the
  canonical Bevy integration pattern works end-to-end.

- [ ] **double_pendulum** — `Model::double_pendulum()` factory.
  Red + blue bobs with connecting rods. Chaotic motion, energy drift < 1%,
  link lengths must stay constant (FK integrity).

- [ ] **nlink_pendulum** — `Model::n_link_pendulum(5)` factory.
  5 color-gradient bobs (red to blue) on a chain. Complex chaotic dynamics,
  all link lengths constant, energy approximately conserved.

- [ ] **spherical_pendulum** — `Model::spherical_pendulum()` with ball joint.
  Golden bob on transparent constraint sphere with red trail. 3D precessing
  motion. Conserves both energy and vertical angular momentum.

## Tier 3: MJCF loading + SimViewerPlugin

- [ ] **mjcf_humanoid** — `sim_mjcf::load_model()` of 13-body humanoid.
  Free+ball+hinge joints (nq=45, nv=38). Body spheres should fall/ragdoll
  under gravity with joint damping. Most complex articulation test.

- [ ] **falling_sphere** — `SimViewerPlugin` + MJCF sphere + ground plane.
  Blue sphere at Z=5 falls onto gray ground. Tests freejoint + auto mesh
  spawning + orbit camera.

- [ ] **collision_shapes** — `SimViewerPlugin` + 8 MJCF shapes in grid.
  Sphere, box, capsule, cylinder, ellipsoid (2 rows x 4 cols) all falling.
  Tests all primitive shape mesh generation.

- [ ] **contact_debug** — `SimViewerPlugin` + 4 spheres + 1 box falling.
  Keyboard toggles: C=contacts, N=normals, F=forces, J=joints, L=limits.
  Tests debug gizmo overlay system.

---

## Status key
- [x] = Window opens, renders correctly, physics looks right
- [~] = Runs but has visual/physics issues (note what's wrong)
- [ ] = Not yet tested
- [!] = Crashes or fails to run
