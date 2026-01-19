# Simulation Domain Architecture

## Overview

The `sim-*` crates provide a MuJoCo-inspired physics simulation stack designed for:

- **Robotics training** (reinforcement learning with sim-to-real transfer)
- **Headless simulation** (no Bevy dependency - Layer 0)
- **Deterministic execution** (fixed iteration counts, reproducible results)
- **Domain randomization** (tunable contact parameters for robust policies)

## Crate Hierarchy

```
sim-types     (L0)  ─┬─►  Pure data types, zero dependencies
                     │
sim-core      (L0)  ─┼─►  Integrators, world, stepper
                     │
sim-contact   (L0)  ─┴─►  Compliant contact model, friction, solver
                     │
sim-physics   (L0)  ─────►  [PLANNED] Unified API combining all
                     │
sim-bevy      (L1)  ─────►  [PLANNED] Bevy integration layer
```

## Current Implementation

### sim-types (v0.7.0)

Pure data structures with no physics logic:

| Type | Purpose |
|------|---------|
| `RigidBodyState` | Position, orientation, velocity (6-DOF) |
| `Pose` | Position + quaternion rotation |
| `Twist` | Linear + angular velocity |
| `MassProperties` | Mass, COM, inertia tensor |
| `Action` / `ExternalForce` | Control inputs |
| `JointState` / `JointCommand` | Articulated body support |
| `SimulationConfig` | Timestep, gravity, solver settings |

### sim-core (v0.7.0)

Numerical integration and world management:

| Component | Description |
|-----------|-------------|
| **Integrators** | Euler, Semi-Implicit Euler, Velocity Verlet, RK4 |
| **World** | Body/joint storage, entity management |
| **Stepper** | Simulation loop orchestration |

Key features:
- Symplectic integrators for energy conservation
- Action scheduling (forces, joint commands)
- Diagnostic queries (energy, momentum)

### sim-contact (v0.1.0)

MuJoCo-inspired compliant contact model:

```
F_normal = k * d^p + c * ḋ
F_friction = project_to_cone(F_tangent, μ * F_normal)
```

| Module | Contents |
|--------|----------|
| `contact.rs` | `ContactPoint`, `ContactManifold`, `ContactForce` |
| `friction.rs` | `FrictionCone`, `FrictionModel`, `RegularizedFriction` |
| `model.rs` | `ContactModel` (spring-damper computation) |
| `params.rs` | `ContactParams`, `DomainRandomization`, material presets |
| `solver.rs` | `ContactSolver` with deterministic fixed iterations |

Key features:
- Nonlinear stiffness (d^p) for stability at varying penetrations
- Regularized friction for implicit integration compatibility
- Domain randomization ranges for sim-to-real transfer
- Material presets: rubber, metal, plastic, wood, etc.

## Design Principles

### 1. Compliant vs Impulse-Based Contacts

We use **compliant (spring-damper) contacts** rather than impulse-based:

| Aspect | Compliant | Impulse-Based |
|--------|-----------|---------------|
| Force response | Smooth, continuous | Discontinuous at contact |
| Parameters | Physical (stiffness, damping) | Abstract (iterations, ERP) |
| Stability | Better for stiff contacts | Sensitive to timestep |
| Sim-to-real | Tunable to match real materials | Harder to transfer |

### 2. Determinism

All solvers use **fixed iteration counts** (not convergence-based):

```rust
ContactSolverConfig {
    velocity_iterations: 4,  // Always 4, regardless of convergence
    position_iterations: 2,
}
```

This ensures:
- Same inputs → same outputs (essential for RL)
- Predictable performance (fixed computational cost)
- Reproducible debugging

### 3. Domain Randomization

Contact parameters can be randomized for robust policy learning:

```rust
let randomizer = DomainRandomization::default();
// stiffness_range: (0.5, 1.5)  -- ±50% variation
// friction_range: (0.2, 1.0)   -- wide friction range
// etc.

let params = randomizer.sample(&base_params, rng_samples);
```

### 4. Layer 0 (No Bevy)

All `sim-*` crates are Layer 0:
- Zero Bevy dependencies
- Can run in headless training loops
- Can deploy to hardware
- Can integrate with other engines

## Gap Analysis: MuJoCo Parity

### Implemented ✓

- [x] Compliant contact force model
- [x] Nonlinear stiffness (d^p)
- [x] Spring-damper normal forces
- [x] Friction cone with projection
- [x] Regularized friction for smooth gradients
- [x] Domain randomization infrastructure
- [x] Deterministic solver
- [x] Basic collision detection (sphere-sphere, sphere-plane)

### Next Steps

- [ ] **Narrow-phase detection** for more shapes (box, convex hull, mesh)
- [ ] **Broad-phase culling** (integrate with `cf-spatial` BVH)
- [ ] **sim-core integration** (apply contact forces in stepper)
- [ ] **Articulated bodies** (joint limits, motors, constraints)
- [ ] **Implicit integration** (for very stiff contacts)
- [ ] **MuJoCo benchmark suite** (side-by-side trajectory comparison)

### Future Considerations

- **Soft body contacts** (deformable meshes)
- **Fluid coupling** (buoyancy, drag)
- **Cable/rope simulation** (position-based dynamics)

## Usage Example

```rust
use sim_core::{World, Stepper};
use sim_contact::{ContactModel, ContactParams, ContactSolver, ContactPoint};
use sim_types::{RigidBodyState, Pose, MassProperties};

// Create world
let mut world = World::default();
let sphere = world.add_body(
    RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 1.0))),
    MassProperties::sphere(1.0, 0.1),
);

// Create contact solver
let model = ContactModel::new(ContactParams::rubber_on_concrete());
let mut solver = ContactSolver::with_model(model);

// In simulation loop:
// 1. Detect contacts (narrow phase)
let contacts = detect_contacts(&world); // TODO: implement

// 2. Solve contact forces
let result = solver.solve(&contacts, |body_id, contact| {
    world.body(body_id).map(|b| b.state.twist.linear).unwrap_or_default()
});

// 3. Apply forces to bodies
for force_result in &result.forces {
    world.apply_force(force_result.contact.body_a, force_result.force.total());
    world.apply_force(force_result.contact.body_b, -force_result.force.total());
}

// 4. Step simulation
stepper.step(&mut world)?;
```

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#contact)
- [MuJoCo Technical Notes](https://mujoco.readthedocs.io/en/stable/computation.html)
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints"
