# MuJoCo Conformance Verification

> **Goal:** Systematically verify that CortenForge's sim-* crates correctly implement MuJoCo semantics.

---

## Overview

CortenForge's simulation stack is "MuJoCo-inspired" but not a direct port. This document outlines verification approaches to ensure behavioral compatibility where it matters (sim-to-real transfer, model loading) while acknowledging intentional divergences.

---

## Verification Approaches

### 1. MuJoCo Conformance Tests

Run MuJoCo's own test suite against CortenForge implementations.

**Strategy:**
- Extract test cases from MuJoCo's C test suite
- Translate to Rust test harness
- Compare numerical outputs within tolerance

**Key test categories:**
| Category | MuJoCo Source | CortenForge Target |
|----------|---------------|-------------------|
| Forward dynamics | `test/engine_forward_test.cc` | sim-core |
| Contact physics | `test/engine_collision_test.cc` | sim-core |
| Constraint solver | `test/engine_core_smooth_test.cc` | sim-core (PGS + CG contact solvers) + sim-constraint (CGSolver, joints) |
| MJCF parsing | `test/xml_test.cc` | sim-mjcf |
| Sensor readings | `test/sensor_test.cc` | sim-core (pipeline sensors) + sim-sensor (standalone) |

**Implementation:**
```rust
// tests/mujoco_conformance/forward_dynamics.rs

#[test]
fn test_free_fall_matches_mujoco() {
    // MuJoCo reference: sphere drops 1m in ~0.45s under Earth gravity
    let model = sim_mjcf::load_model(SPHERE_DROP_MJCF).unwrap();
    let mut data = model.make_data();

    let dt = 0.002; // MuJoCo default timestep
    let mut t = 0.0;

    while t < 0.5 {
        data.step(&model).unwrap();
        t += dt;
    }

    let pos = &data.xpos[1]; // Body 1 (body 0 is world)
    // MuJoCo reference value (from mj_forward with same setup)
    let mujoco_z = 0.0123; // captured from MuJoCo run

    assert!((pos.z - mujoco_z).abs() < 1e-6, "Position diverged from MuJoCo reference");
}
```

**Status:** `[ ] Not started`

---

### 2. Systematic MJCF Documentation Comparison

Compare CortenForge's MJCF parser against MuJoCo's XML reference, element by element.

**MuJoCo XML Reference:** https://mujoco.readthedocs.io/en/stable/XMLreference.html

**Comparison matrix:**

#### Top-level elements
| Element | MuJoCo | sim-mjcf | Notes |
|---------|--------|----------|-------|
| `<mujoco>` | ✓ | ✓ | Root element |
| `<compiler>` | ✓ | ✓ | Partial: angle, coordinate |
| `<option>` | ✓ | ✓ | Full: timestep, gravity, integrator, solver, cone, jacobian, wind, flags, etc. |
| `<size>` | ✓ | ⚠️ | Memory hints, may not apply |
| `<visual>` | ✓ | ❌ | L1 concern (sim-bevy) |
| `<statistic>` | ✓ | ❌ | Auto-computed |
| `<default>` | ✓ | ✓ | Class inheritance system |
| `<custom>` | ✓ | ❌ | User data, low priority |
| `<extension>` | ✓ | ❌ | Plugin system, low priority |
| `<asset>` | ✓ | ⚠️ | Mesh, texture refs |
| `<worldbody>` | ✓ | ✓ | Body hierarchy |
| `<contact>` | ✓ | ✓ | Contact filtering |
| `<equality>` | ✓ | ✓ | Equality constraints |
| `<tendon>` | ✓ | ✓ | Fixed and spatial tendons |
| `<actuator>` | ✓ | ✓ | All 8 shortcut types (motor, position, velocity, damper, cylinder, adhesion, muscle, general) with MuJoCo-compatible gain/bias force model, GainType/BiasType dispatch, FilterExact dynamics |
| `<sensor>` | ✓ | ✓ | Various sensor types |
| `<keyframe>` | ✓ | ❌ | State snapshots |

#### Body/Geom/Joint attributes
| Attribute | MuJoCo | sim-mjcf | Notes |
|-----------|--------|----------|-------|
| `body/@pos` | ✓ | ✓ | Position |
| `body/@quat` | ✓ | ✓ | Orientation |
| `body/@euler` | ✓ | ✓ | Euler angles |
| `body/@axisangle` | ✓ | ⚠️ | Check implementation |
| `body/@xyaxes` | ✓ | ⚠️ | Check implementation |
| `body/@zaxis` | ✓ | ⚠️ | Check implementation |
| `geom/@type` | ✓ | ✓ | sphere, box, capsule, cylinder, ellipsoid, plane, mesh |
| `geom/@size` | ✓ | ✓ | Shape-dependent sizing |
| `geom/@fromto` | ✓ | ✓ | Capsule/cylinder shorthand |
| `geom/@contype` | ✓ | ✓ | Contact type bitmask |
| `geom/@conaffinity` | ✓ | ✓ | Contact affinity bitmask |
| `geom/@condim` | ✓ | ⚠️ | Contact dimensionality |
| `geom/@friction` | ✓ | ✓ | Sliding, torsional, rolling |
| `geom/@solref` | ✓ | ✓ | Solver reference params |
| `geom/@solimp` | ✓ | ✓ | Solver impedance params |
| `joint/@type` | ✓ | ✓ | hinge, slide, ball, free |
| `joint/@axis` | ✓ | ✓ | Joint axis |
| `joint/@range` | ✓ | ✓ | Position limits |
| `joint/@limited` | ✓ | ✓ | Enable limits |
| `joint/@damping` | ✓ | ✓ | Joint damping |
| `joint/@stiffness` | ✓ | ✓ | Joint spring |
| `joint/@armature` | ✓ | ⚠️ | Rotor inertia |

**Action items:**
- [ ] Create tracking issue for each ⚠️ item
- [ ] Document intentional ❌ omissions in `sim/docs/ARCHITECTURE.md`
- [ ] Add parsing tests for each ✓ attribute

**Status:** `[ ] Not started`

---

### 3. Real-World Model Loading ✅ COMPLETE

Load and validate models from established sources.

**Status:** Completed 2026-01-24 — model loading tests passing

**Implementation:** `sim/L0/tests/` (sim-conformance-tests) with git submodules for test assets.

Run tests: `cargo test -p sim-conformance-tests`

> **Note:** Conformance tests are being migrated to the Model/Data API.
> See `sim/L0/tests/integration/model_data_pipeline.rs` for the new test patterns.

#### MuJoCo Menagerie (Model Zoo)
https://github.com/google-deepmind/mujoco_menagerie

| Model | Category | Status | Notes |
|-------|----------|--------|-------|
| `franka_emika_panda` | Robot arm | [x] | 7-DOF manipulator |
| `universal_robots_ur5e` | Robot arm | [x] | Industrial arm |
| `kuka_iiwa_14` | Robot arm | [x] | 7-DOF arm |
| `unitree_go1` | Quadruped | [x] | Walking robot |
| `unitree_go2` | Quadruped | [x] | Walking robot |
| `unitree_h1` | Humanoid | [x] | Full humanoid |
| `unitree_g1` | Humanoid | [x] | Compact humanoid |
| `shadow_hand` | Dexterous hand | [x] | High-DOF hand |
| `robotiq_2f85` | Gripper | [x] | 2-finger gripper |
| `anymal_c` | Quadruped | [x] | ANYmal robot |
| `anymal_b` | Quadruped | [x] | ANYmal robot |
| `agility_digit` | Humanoid | [x] | Bipedal robot |
| `robotis_op3` | Humanoid | [x] | Small humanoid |
| `google_robot` | Mobile manip | [x] | Google robot |
| `google_barkour` | Quadruped | [x] | Barkour robot |
| `aloha` | Dual arm | [x] | Bimanual robot |

#### DeepMind Control Suite
https://github.com/google-deepmind/dm_control

| Domain | Task | Status | Notes |
|--------|------|--------|-------|
| `acrobot` | swingup | [x] | Double pendulum |
| `ball_in_cup` | catch | [x] | Ball manipulation |
| `cartpole` | balance | [x] | Classic control |
| `cheetah` | run | [x] | Planar runner |
| `finger` | spin | [x] | Object manipulation |
| `fish` | swim | [x] | 3D swimming |
| `hopper` | hop | [x] | Single-leg hopper |
| `humanoid` | walk | [x] | Full humanoid |
| `humanoid_CMU` | walk | [x] | CMU humanoid |
| `manipulator` | bring_ball | [x] | Arm with objects |
| `pendulum` | swingup | [x] | Single pendulum |
| `point_mass` | easy | [x] | 2D navigation |
| `reacher` | easy | [x] | Planar arm |
| `swimmer` | swimmer6 | [x] | Multi-link swimmer |
| `walker` | walk | [x] | Bipedal walker |
| `quadruped` | walk | [x] | 4-legged walker |
| `dog` | run | [x] | Realistic dog |
| `stacker` | stack | [x] | Block stacking |
| `lqr` | control | [x] | LQR benchmark |

**Test infrastructure:**
- Git submodules: `sim/L0/tests/assets/mujoco_menagerie/`, `sim/L0/tests/assets/dm_control/`
- Test crate: `sim/L0/tests/` (sim-conformance-tests)
- Macro-based tests verify body/joint counts match expectations

---

### 4. Numerical Trajectory Comparison

Run identical simulations in MuJoCo and CortenForge, compare trajectories.

**Methodology:**

1. **Reference generation:**
   - Run MuJoCo simulation (Python bindings)
   - Record state trajectory at each timestep
   - Export to JSON/binary format

2. **CortenForge simulation:**
   - Load same model
   - Run with identical parameters
   - Record state trajectory

3. **Comparison:**
   - Compute per-timestep error
   - Track error accumulation
   - Identify divergence points

**Reference capture script (Python):**
```python
import mujoco
import json

def capture_trajectory(model_path, duration=5.0, dt=0.002):
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    trajectory = []
    t = 0.0

    while t < duration:
        mujoco.mj_step(model, data)

        trajectory.append({
            "time": t,
            "qpos": data.qpos.tolist(),
            "qvel": data.qvel.tolist(),
            "qacc": data.qacc.tolist(),
        })

        t += dt

    return trajectory

# Generate reference for pendulum
traj = capture_trajectory("pendulum.xml")
with open("pendulum_reference.json", "w") as f:
    json.dump(traj, f)
```

**Comparison test (Rust):**
```rust
#[test]
fn test_pendulum_trajectory_matches_mujoco() {
    let reference: Vec<TrajectoryPoint> =
        serde_json::from_str(include_str!("pendulum_reference.json"))
            .expect("Failed to load reference");

    let model = sim_mjcf::load_model_from_file("pendulum.xml").unwrap();
    let mut data = model.make_data();

    let mut max_position_error = 0.0;
    let mut max_velocity_error = 0.0;

    for ref_point in &reference {
        // Step simulation
        data.step(&model).unwrap();

        // Compare positions (qpos is the joint-space state)
        for (ours, theirs) in data.qpos.iter().zip(ref_point.qpos.iter()) {
            let error = (ours - theirs).abs();
            max_position_error = max_position_error.max(error);
        }

        // Compare velocities
        for (ours, theirs) in data.qvel.iter().zip(ref_point.qvel.iter()) {
            let error = (ours - theirs).abs();
            max_velocity_error = max_velocity_error.max(error);
        }
    }

    // Allow small numerical differences (different floating point paths)
    assert!(max_position_error < 1e-4, "Position error too large: {}", max_position_error);
    assert!(max_velocity_error < 1e-3, "Velocity error too large: {}", max_velocity_error);
}
```

**Error tolerance guidelines:**
| Metric | Acceptable | Concerning | Failing |
|--------|------------|------------|---------|
| Position error (per step) | < 1e-6 | 1e-6 - 1e-4 | > 1e-4 |
| Position drift (1s) | < 1e-4 | 1e-4 - 1e-2 | > 1e-2 |
| Velocity error (per step) | < 1e-5 | 1e-5 - 1e-3 | > 1e-3 |
| Contact force error | < 5% | 5% - 20% | > 20% |

**Status:** `[ ] Not started`

---

## Intentional Divergences

Some differences from MuJoCo are by design:

| Area | MuJoCo | CortenForge | Rationale |
|------|--------|-------------|-----------|
| Memory model | Pre-allocated pools | Dynamic allocation | Rust idioms, safety |
| Threading | OpenMP | Rayon | Rust ecosystem |
| GPU | Custom CUDA | Future: wgpu | Cross-platform |
| Contact solver | Custom sparse | PGS + CG/PGD solver (mujoco_pipeline) | Maintainability |
| Visualization | Built-in | Separate L1 crate | Headless training |

---

## Test Infrastructure

### Directory structure
```
sim/L0/tests/
├── Cargo.toml
├── mujoco_conformance/
│   └── mod.rs              (migration stub — tests moving to integration/)
├── integration/
│   ├── mod.rs
│   ├── model_data_pipeline.rs   (Model/Data pipeline tests)
│   ├── collision_primitives.rs
│   ├── collision_plane.rs
│   ├── collision_edge_cases.rs
│   ├── collision_performance.rs
│   ├── cg_solver.rs
│   ├── collision_test_utils.rs
│   ├── equality_constraints.rs
│   ├── implicit_integration.rs
│   ├── rk4_integration.rs
│   ├── mjcf_sensors.rs
│   ├── musculoskeletal.rs
│   ├── passive_forces.rs
│   ├── sensors.rs
│   └── validation.rs
└── assets/
    ├── mujoco_menagerie/  (git submodule)
    └── dm_control/        (git submodule)
```

> **Note:** The `mujoco_conformance/` directory is a stub — conformance tests are being
> migrated to `integration/` using the Model/Data API. Forward dynamics, contact physics,
> and trajectory comparison tests are planned but not yet implemented.

### CI integration (planned — not yet created)

> **Note:** This workflow does not exist yet. The file `.github/workflows/mujoco_conformance.yml`
> and script `scripts/generate_mujoco_references.py` have not been created. The correct package
> for conformance tests is `sim-conformance-tests`, not `sim-core`. Current conformance tests
> run via `cargo test -p sim-conformance-tests` in the existing `quality-gate.yml` workflow.

```yaml
# .github/workflows/mujoco_conformance.yml (PLANNED)
name: MuJoCo Conformance

on: [push, pull_request]

jobs:
  conformance:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive

      - name: Install MuJoCo (for reference generation)
        run: pip install mujoco

      - name: Generate reference trajectories
        run: python scripts/generate_mujoco_references.py

      - name: Run conformance tests
        run: cargo test -p sim-conformance-tests
```

---

## Progress Tracking

| Approach | Priority | Status | Owner | Notes |
|----------|----------|--------|-------|-------|
| Conformance tests | High | Not started | - | Foundation for validation |
| Doc comparison | Medium | Not started | - | Guides implementation gaps |
| Model loading | High | ✅ Complete | - | Menagerie + DM Control models load (2026-01-24) |
| Trajectory comparison | High | Not started | - | Numerical correctness |

---

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo GitHub (tests)](https://github.com/google-deepmind/mujoco/tree/main/test)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- [DeepMind Control Suite](https://github.com/google-deepmind/dm_control)
