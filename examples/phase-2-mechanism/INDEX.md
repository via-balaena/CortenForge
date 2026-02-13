# Phase 2: Mechanism Design + Simulation

> **Parametric Geometry -> Rigid Body Simulation -> Physical Prototype**

Phase 2 introduces rigid body simulation alongside the mesh pipeline from
Phase 1. Three mechanical products that must work both in simulation and
in the real world. Simulated dynamics are verified against physical
prototypes -- if sim and reality diverge, we fix the sim.

---

## New Domains Introduced

| Domain | Purpose |
|--------|---------|
| sim-types | Core simulation data types (bodies, joints, states) |
| sim-core | Rigid body dynamics engine (integrator, broadphase, solver) |
| sim-constraint | Joint and contact constraint formulation |
| sim-mjcf | MuJoCo XML model import/export |
| sim-urdf | URDF model import/export |
| mesh-assembly | Multi-body mesh assembly with joint definitions |

**Prerequisites:** Phase 1 complete. All mesh pipeline crates are assumed
stable and available.

---

## Products

| Product | Description | Manufacturing | Spec |
|---------|-------------|---------------|------|
| Compliant Gripper | Two-finger flexure gripper with material compliance | FDM (flexible filament) | [SPEC.md](./compliant-gripper/SPEC.md) |
| Prosthetic Finger | Bio-inspired finger with tendon-routed actuation | FDM + cable | [SPEC.md](./prosthetic-finger/SPEC.md) |
| Four-Bar Walker | Passive walking toy (Strandbeest-style linkage) | FDM + metal pins | [SPEC.md](./four-bar-walker/SPEC.md) |

**Goal:** Introduce rigid body simulation and verify against physical
prototypes. Every mechanism must be simulatable in MuJoCo-compatible format
and must match measured physical behavior within stated tolerances.

---

## Crate Coverage

| Crate | Gripper | Finger | Walker |
|-------|---------|--------|--------|
| sim-types | x | x | x |
| sim-core | x | x | x |
| sim-constraint | x | x | x |
| sim-mjcf or sim-urdf | x | x | x |
| curve-types | x | x | x |
| mesh-from-curves | x | x | x |
| mesh-assembly | x | x | x |
| mesh-printability | x | x | x |

---

## Running

```bash
# Compliant gripper: generate geometry + simulate grasp
cargo run -p example-compliant-gripper

# Prosthetic finger: generate linkage + simulate flexion
cargo run -p example-prosthetic-finger

# Four-bar walker: generate linkage + simulate passive gait
cargo run -p example-four-bar-walker
```

---

## Shared Learnings

Things we expect to discover and codify during Phase 2:

- Joint modeling fidelity required for sim-to-real agreement on flexure mechanisms
- Contact model parameters (friction, restitution) that match FDM plastic behavior
- MJCF/URDF round-trip accuracy for parametric joint definitions
- Assembly constraints that are both geometrically valid and dynamically stable
- Minimum simulation timestep for stable linkage dynamics under gravity
- How compliant (flexible) elements map to rigid body constraint approximations

---

## Completion Criteria

Phase 2 is complete when all of the following hold:

- [ ] All three products simulate in CortenForge's rigid body engine
- [ ] Each product exports a valid MJCF or URDF model that loads in MuJoCo
- [ ] Simulated joint trajectories match physical prototype measurements within 10% RMS error
- [ ] The four-bar walker achieves stable passive walking in simulation for at least 10 gait cycles
- [ ] The compliant gripper closes around a test object without interpenetration or constraint explosion
- [ ] The prosthetic finger achieves full flexion/extension range matching the kinematic spec
- [ ] At least one product has been physically manufactured and tested against simulation predictions
- [ ] All mesh pipeline crates from Phase 1 continue to pass on Phase 2 geometries

---

*See also: [Product Roadmap](../PRODUCT_ROADMAP.md) | [Phase 1: Mesh Pipeline](../phase-1-mesh/INDEX.md) | [Phase 3: Perception](../phase-3-perception/INDEX.md)*
