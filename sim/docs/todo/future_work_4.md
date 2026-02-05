# Future Work 4 — Physics Completeness (Items #11–14)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 11. Deformable Body Pipeline Integration
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

*Transferred from [future_work_1.md](./future_work_1.md) #9.*

#### Current State
sim-deformable is a standalone 7,733-line crate (86 tests):

| Component | Location | Description |
|-----------|----------|-------------|
| `XpbdSolver` | `solver.rs:134` | XPBD constraint solver. `step(&mut self, body: &mut dyn DeformableBody, gravity: Vector3<f64>, dt: f64)` (`solver.rs:196`). Configurable substeps, damping, sleeping. |
| `DeformableBody` trait | `lib.rs:173` | Common interface for Cloth, SoftBody, CapsuleChain. |
| `Cloth` | `cloth.rs` | Triangle meshes with distance + dihedral bending constraints. `thickness` field (`cloth.rs:50`). Presets: cotton, silk, leather, rubber, paper, membrane. |
| `SoftBody` | `soft_body.rs` | Tetrahedral meshes with distance + volume constraints. Presets: rubber, gelatin, soft tissue, muscle, foam, stiff. |
| `CapsuleChain` | `capsule_chain.rs` | 1D particle chains with distance + bending constraints. `radius` field (`capsule_chain.rs:41`). Presets: rope, steel cable, hair, chain. |
| `Material` | `material.rs` | Young's modulus, Poisson's ratio, density, `friction` (`material.rs:94`). 14 presets. |
| `ConstraintType::Collision` | `constraints.rs:42-43` | Enum variant defined but unimplemented — no constraint implements it. |
| `FlexEdge` | `constraints.rs` | Stretch, shear, twist constraint variants. |

The crate has zero coupling to the MuJoCo pipeline. sim-physics re-exports it behind
the `deformable` feature flag (`physics/src/lib.rs:109-110`). `Material.friction` and
per-body `radius`/`thickness` fields are declared but unused by the collision system
(which doesn't exist yet).

#### Objective
Deformable bodies interact with rigid bodies through the same contact solver and
step in the same simulation loop.

#### Specification

**Collision detection** (greenfield — no infrastructure exists):

1. **Broadphase:** Deformable vertex AABBs vs rigid geom AABBs. Vertex AABBs are
   point + radius/thickness margin. Use `batch_aabb_overlap_4()` (`simd/src/batch_ops.rs:251`)
   for batched broadphase queries.
2. **Narrowphase:** Vertex-vs-geom closest point computation. Produces `Contact` structs
   identical to rigid-rigid contacts (same normal, depth, friction, solref/solimp).
3. **Friction:** Combine `Material.friction` (deformable side) with geom friction
   (rigid side) using the same combination rule as rigid-rigid contacts.

**Contact solver coupling:**

Deformable-rigid contacts feed into PGS (or CG) alongside rigid-rigid
contacts. Each deformable vertex is a 3-DOF point mass. Its inverse mass comes from
the XPBD solver's per-particle mass. Contact Jacobians for deformable vertices are
3x3 identity blocks (point mass — no rotational DOFs).

**Force feedback:**

Contact impulses from PGS apply to deformable vertex velocities directly and to
rigid body `qfrc_constraint` through the standard Jacobian transpose. XPBD
constraint projection runs after contact resolution within the same timestep.

**Substep iteration (XPBD/contact ordering):**

A single pass (contact solve -> XPBD) may leave contacts invalid because XPBD
constraint projection moves vertices after contacts are computed. For stiff
deformable bodies or deep penetrations, this causes jitter.

Options (to be chosen at implementation time):
- **Option A (simple):** Single pass, accept minor inaccuracy. Sufficient for cloth
  and rope where deformation is small relative to contact depth.
- **Option B (robust):** Iterate contact-detection + solve + XPBD for
  `n_substep_iterations` (default 1, configurable up to 4). Each iteration
  re-detects contacts at updated vertex positions. More expensive but handles
  stiff soft bodies contacting rigid surfaces.

The choice should be configurable per-model via an MJCF option.

**Pipeline integration in `Data::step()`:**

```
1. Rigid: forward kinematics, collision, forces
2. Deformable-rigid collision detection -> Contact list
3. Combined contact solve (rigid + deformable contacts)
4. Apply contact impulses to rigid bodies and deformable vertices
5. XpbdSolver::step() for each registered deformable body
6. (Optional) Repeat steps 2-5 for substep iterations > 1
7. Rigid: position integration
```

#### Acceptance Criteria
1. A rigid body resting on a deformable surface experiences the same contact forces as resting on a rigid surface of equivalent geometry.
2. XPBD internal constraints (distance, bending, volume) are satisfied after contact resolution — contact forces do not violate deformable material properties.
3. `Material.friction` is used in deformable-rigid contacts (not hardcoded).
4. `ConstraintType::Collision` is implemented in the constraint system.
5. Zero-deformable-body configurations have zero overhead (no broadphase, no substep).
6. Cloth draped over a rigid sphere reaches stable equilibrium without jitter.
7. Substep iteration count is configurable; default (1) works for cloth/rope use cases.

#### Files
- `sim/L0/deformable/src/` — modify (collision detection, `ConstraintType::Collision` implementation)
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (pipeline integration in `Data::step()`, deformable body registration)
- `sim/L0/simd/src/batch_ops.rs` — reference (`batch_aabb_overlap_4()`)

---

### 12. Analytical Derivatives (mjd_*)
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State
No analytical derivative infrastructure exists. The pipeline computes contact
Jacobians (`compute_contact_jacobian()` at line 7619) and body-point Jacobians
(`compute_body_jacobian_at_point()` at line 7534) for the constraint solver, but
these are not exposed as a general derivative API.

MuJoCo provides `mjd_transitionFD` (finite-difference derivatives of the full
transition function) and analytical derivatives for specific pipeline stages. These
are critical for model-based RL (iLQR, DDP), trajectory optimization, and system
identification.

#### Objective
Provide derivatives of the simulation transition function
`(q_{t+1}, v_{t+1}) = f(q_t, v_t, u_t)` with respect to state and control.

#### Specification

This item is intentionally sparse — the implementation strategy (analytical vs
finite-difference vs automatic differentiation) should be chosen at design time:

**Option A (pragmatic):** Finite-difference derivatives via `mjd_transitionFD`
pattern — step the simulation with perturbed inputs and compute centered
differences. Easy to implement, works with the existing pipeline, O(nv + nu) cost
per derivative computation.

**Option B (performant):** Analytical derivatives for each pipeline stage. Requires
deriving and implementing gradients through FK, contact, constraint solver, and
integration. Matches MuJoCo's `mjd_*` functions. Much more complex but O(1)
additional cost.

**Option C (modern):** Automatic differentiation via dual numbers or tape-based AD.
Would require making the pipeline generic over scalar type or using an AD library.

#### Acceptance Criteria
1. `d(q_{t+1})/d(q_t)`, `d(q_{t+1})/d(v_t)`, `d(q_{t+1})/d(u_t)` are computable.
2. Derivatives agree with finite differences to within numerical precision.
3. Derivative computation cost is documented (FD: ~O(nv * step_cost), analytical: ~O(step_cost)).

#### Files
- `sim/L0/core/src/` — new module (derivatives)

---

### 13. Full Implicit Integrator
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
`Integrator::ImplicitSpringDamper` implements diagonal-only implicit integration:

- Per-DOF stiffness K and damping D from joint properties
- Solves `(M + h*D + h^2*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)`
- Diagonal K and D — no off-diagonal coupling
- Tendon springs/dampers explicitly skipped in implicit mode
  (`mujoco_pipeline.rs:7403–7406`, comment: "Tendon springs/dampers couple multiple
  joints (non-diagonal K/D), so they cannot be absorbed into the existing diagonal
  implicit modification.")

MuJoCo's full implicit integrator handles the complete mass-matrix coupling
including off-diagonal stiffness and damping terms from tendons and other
multi-joint coupling elements.

#### Objective
Extend the implicit integrator to handle full (non-diagonal) stiffness and damping
matrices, including tendon spring/damper coupling across joints.

#### Specification

Replace diagonal K/D vectors with sparse K/D matrices. In `mj_fwd_passive()`,
compute off-diagonal entries from tendon Jacobians:

```
K_ij += J_tendon[i] * k_tendon * J_tendon[j]
D_ij += J_tendon[i] * d_tendon * J_tendon[j]
```

The implicit solve becomes `(M + h*D + h^2*K) * v = rhs` with sparse M+hD+h^2K.
Reuse the existing sparse Cholesky infrastructure from Task #2 (Phase 1).

#### Acceptance Criteria
1. With tendon springs, implicit integrator produces stable integration where
   explicit Euler diverges.
2. Diagonal-only case (no tendons) matches current `ImplicitSpringDamper` output
   (regression test).
3. Off-diagonal K/D terms are sparse — zero overhead for joints without coupling.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_passive()`,
  `mj_fwd_acceleration_implicit()`, implicit parameter storage)

---

### 14. Keyframes, Mocap Bodies, User Callbacks
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
None of these features exist in the pipeline. The only reference to "keyframe" in
the codebase is a doc comment in sim-constraint's equality module.

#### Objective
Support MJCF `<keyframe>` state snapshots for quick reset, `mocap` bodies for
externally driven poses, and (deferred) user callback hooks for custom logic
injection during simulation.

#### Specification

**Keyframes:**

Store named state snapshots in `Model`:

```rust
pub struct Keyframe {
    pub name: String,
    pub qpos: DVector<f64>,
    pub qvel: DVector<f64>,
    pub act: DVector<f64>,
    pub ctrl: DVector<f64>,
}
```

`Data::reset_to_keyframe(&mut self, &Model, keyframe_idx: usize)` restores
state. Parse from MJCF `<keyframe><key name="..." qpos="..."/></keyframe>`.

**Mocap bodies:**

Bodies with `mocap="true"` have externally driven position/orientation. They
participate in collision but are not integrated — their pose is set directly
via `data.mocap_pos[i]` and `data.mocap_quat[i]`.

```rust
// In Data:
pub mocap_pos: Vec<Vector3<f64>>,
pub mocap_quat: Vec<UnitQuaternion<f64>>,
```

Pipeline skips FK and integration for mocap bodies; collision uses the
mocap-set pose directly.

**User callbacks (deferred):**

MuJoCo's `mjcb_*` hooks (passive force, control, sensor, collision filter) are
useful but require careful API design for Rust's ownership model. Defer to a
follow-up — keyframes and mocap are higher priority.

#### Acceptance Criteria
1. `Data::reset_to_keyframe()` restores exact state from MJCF `<keyframe>`.
2. Mocap body pose is externally settable and affects collision but not integration.
3. Mocap bodies have zero mass contribution to the system.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (Model fields, Data fields,
  `Data::step()` skip for mocap, `reset_to_keyframe()`)
- `sim/L0/mjcf/src/parser.rs` — modify (parse `<keyframe>`, `mocap` attribute)
- `sim/L0/mjcf/src/model_builder.rs` — modify (build keyframe/mocap data)
