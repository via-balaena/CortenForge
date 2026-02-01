# Simulation — Future Work

## Priority Framework

Items are scored on three axes:

| Axis | Definition |
|------|-----------|
| **RL Impact** | How directly this unblocks reinforcement-learning training workflows (batch stepping, observation fidelity, GPU throughput). |
| **Correctness** | Whether this fixes a simulation bug, stub, or semantic mismatch that produces wrong results today. |
| **Effort** | Implementation size: **S** (< 200 LOC), **M** (200–800 LOC), **L** (800–2 000 LOC), **XL** (> 2 000 LOC). |

Priority is **RL Impact + Correctness**, tie-broken by inverse Effort. Items within a group
can be tackled in any order unless a prerequisite is noted.

| # | Item | RL Impact | Correctness | Effort | Prerequisites |
|---|------|-----------|-------------|--------|---------------|
| 1 | In-Place Cholesky | Low | Low | S | None |
| 2 | Sparse L^T D L | Medium | Low | M | #1 |
| 3 | CG Contact Solver | Medium | Low | L | None (#1, #2 help perf) |
| 4 | Tendon Pipeline | Low | High | L | None |
| 5 | Muscle Pipeline | Low | High | L | #4 |
| 6 | Sensor Completion | High | High | M | None |
| 7 | Integrator Rename | Low | Medium | S | None |
| 8 | True RK4 Integration | Low | Medium | M | None |
| 9 | Deformable Body Integration | Medium | Low | XL | None |
| 10 | Batched Simulation | High | Low | L | None |
| 11 | GPU Acceleration | High | Low | XL | #10 |

## Dependency Graph

```
   ┌───┐
   │ 1 │ In-Place Cholesky
   └─┬─┘
     │
     ▼
   ┌───┐         ┌───┐
   │ 2 │ Sparse  │ 3 │ CG Solver ←──(perf benefits from #1, #2; not hard deps)
   └───┘         └───┘

   ┌───┐
   │ 4 │ Tendon Pipeline
   └─┬─┘
     │
     ▼
   ┌───┐
   │ 5 │ Muscle Pipeline
   └───┘

   ┌───┐
   │ 6 │ Sensor Completion              (independent)
   └───┘

   ┌───┐   ┌───┐
   │ 7 │   │ 8 │ Integrator items        (independent)
   └───┘   └───┘

   ┌───┐
   │ 9 │ Deformable Body                (independent)
   └───┘

   ┌────┐
   │ 10 │ Batched Simulation
   └─┬──┘
     │
     ▼
   ┌────┐
   │ 11 │ GPU Acceleration
   └────┘
```

---

## Group A — Solver & Linear Algebra

### 1. In-Place Cholesky Factorization
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
`mj_fwd_acceleration_implicit()` (`mujoco_pipeline.rs:8331`) uses `std::mem::replace`
to swap `scratch_m_impl` out for Cholesky consumption, allocating an O(nv²) zero matrix
every step (`mujoco_pipeline.rs:8387`). For nv=100 this is 80 KB/step of unnecessary
allocation.

#### Objective
Zero-allocation Cholesky factorization that overwrites `scratch_m_impl` in place.

#### Specification

Implement `cholesky_in_place(m: &mut DMatrix<f64>) -> Result<(), StepError>`:

1. For each column j = 0..nv:
   - `L[j,j] = sqrt(M[j,j] - Σ(L[j,k]² for k < j))`
   - For each row i > j: `L[i,j] = (M[i,j] - Σ(L[i,k]·L[j,k] for k < j)) / L[j,j]`
2. Overwrite the lower triangle of `m` with L. The upper triangle is unused.
3. Return `Err(StepError::CholeskyFailed)` if any diagonal element is ≤ 0.

Implement `cholesky_solve_in_place(m: &DMatrix<f64>, x: &mut DVector<f64>)`:

1. Forward solve L·y = x (overwrite x with y).
2. Back solve L^T·z = y (overwrite x with z).

Both functions operate on borrowed data. No allocations, no ownership transfers,
no nalgebra `Cholesky` type.

#### Acceptance Criteria
1. `std::mem::replace` and `DMatrix::zeros(nv, nv)` are removed from `mj_fwd_acceleration_implicit()`.
2. `scratch_m_impl` is reused across steps via `copy_from(&data.qM)` as before.
3. Numeric results match nalgebra `Cholesky::solve` to ≤ 1e-12 relative error on the solved vector.
4. Zero heap allocations in the factorization/solve path (verifiable via `#[global_allocator]` counting test).
5. Existing implicit integration tests pass without modification.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_acceleration_implicit()`)

---

### 2. Sparse L^T D L Factorization
**Status:** Not started | **Effort:** M | **Prerequisites:** #1

#### Current State
Dense Cholesky in `mj_fwd_acceleration_implicit()` is O(nv³). For tree-structured
robots (humanoids, quadrupeds, manipulators), the mass matrix has banded sparsity —
DOF i couples only with its ancestor DOFs in the kinematic tree.

Data scaffolds exist in `Data` (`mujoco_pipeline.rs:1337-1344`): `qLD_diag` (diagonal
of D), `qLD_L` (sparse lower triangular entries per row), `qLD_valid` (validity flag).
These are initialized to defaults and never populated. The `TODO(FUTURE_WORK#7)` comment
at `mujoco_pipeline.rs:1321` cross-references this spec.

#### Objective
O(nv) factorization and solve for tree-structured robots.

#### Specification

Implement `mj_factor_sparse(model: &Model, data: &mut Data)`:

1. Walk the kinematic tree from leaves to root.
2. For each DOF i, accumulate composite inertia from child DOFs.
3. Compute `qLD_diag[i]` (diagonal pivot) and `qLD_L[i]` (off-diagonal entries
   for ancestor DOFs only).
4. Set `qLD_valid = true`.

Implement `mj_solve_sparse(data: &Data, x: &mut DVector<f64>)`:

1. Forward substitution: L·y = x, walking tree root-to-leaves.
2. Diagonal solve: D·z = y.
3. Back substitution: L^T·w = z, walking tree leaves-to-root.

Each row of L has at most `depth(i)` non-zero entries. Total work is O(Σ depth(i))
— O(nv) for balanced trees, O(nv·d) for chains of depth d.

#### Acceptance Criteria
1. For a humanoid (nv=30), factorization uses ~30 multiply-adds vs ~9,000 for dense (300× reduction).
2. Numeric results match dense Cholesky to ≤ 1e-12 relative error.
3. `mj_fwd_acceleration_implicit()` checks `data.qLD_valid` and dispatches to sparse solve when available, dense otherwise.
4. Sparse factorization is recomputed when the mass matrix changes (after CRBA).
5. Benchmark test demonstrating wall-clock speedup for nv ≥ 20 tree-structured models.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_factor_sparse()`, `mj_solve_sparse()`, dispatch in `mj_fwd_acceleration_implicit()`)

---

### 3. CG Contact Solver
**Status:** Not started | **Effort:** L | **Prerequisites:** None (#1, #2 improve M⁻¹ performance but are not hard dependencies)

#### Current State
`CGSolver` in `sim-constraint/src/cg.rs` (1,664 lines, `cg.rs:309`) implements
preconditioned conjugate gradient with Block Jacobi preconditioning. It solves joint
constraints via the `Joint` trait (`cg.rs:408`): `solve<J: Joint>(&mut self, joints: &[J], ...)`.
The pipeline uses PGS via `pgs_solve_contacts()` (`mujoco_pipeline.rs:6563`) for contact
constraints. CGSolver has zero callers in the pipeline.

The `Joint` trait expects rigid-body joint semantics (`parent()`, `child()`,
`parent_anchor()`, `joint_type()`). Contact constraints have none of these — they are
geometric collisions defined by normal, tangent frame, and friction coefficient. The
type systems are incompatible; no adapter exists.

#### Objective
CG-based contact constraint solver as an alternative to PGS, selectable per-model.

#### Specification

New function in `cg.rs`:

```rust
pub fn cg_solve_contacts(
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    model: &Model,
    data: &Data,
    efc_lambda: &mut HashMap<(usize, usize), [f64; 3]>,
    config: &CGSolverConfig,
) -> CGContactResult
```

Algorithm:

1. Assemble the Delassus matrix W = J M⁻¹ J^T from pre-computed contact Jacobians
   (from `compute_contact_jacobian()` at `mujoco_pipeline.rs:6383`).
2. Build the constraint RHS from penetration depths, approach velocities, and
   solref/solimp parameters (same physics as PGS).
3. Solve W·λ = rhs via preconditioned conjugate gradient with friction cone
   projection at each iteration (λ_n ≥ 0, |λ_t| ≤ μ·λ_n).
4. Each contact is one 3-row block (normal + 2 friction tangents). Block Jacobi
   preconditioner inverts these 3×3 diagonal blocks via Cholesky.

```rust
pub struct CGContactResult {
    pub forces: Vec<Vector3<f64>>,
    pub iterations_used: usize,
    pub residual_norm: f64,
    pub converged: bool,
}
```

New enum in `mujoco_pipeline.rs`:

```rust
pub enum SolverType { PGS, CG }
```

Stored as `solver_type: SolverType` in `Model`. Default: `PGS`. Parsed from MJCF
`<option solver="CG"/>`. `mj_fwd_constraint()` (`mujoco_pipeline.rs:7991`) dispatches
on `model.solver_type`.

**Fallback policy (testing concern):** If CG does not converge within
`config.max_iterations`, the step falls back to PGS and logs a warning. This
silent-fallback pattern means CG correctness bugs can hide behind PGS. The test
suite should include cases where CG is required to converge (no fallback) to
catch regressions. Consider a `SolverType::CGStrict` variant that returns
`Err(StepError::SolverFailed)` instead of falling back, for use in tests.

#### Acceptance Criteria
1. For any contact configuration where PGS converges, CG produces forces satisfying the same friction cone constraints (λ_n ≥ 0, |λ_t| ≤ μ·λ_n).
2. CG and PGS are interchangeable — switching `solver_type` does not change physical behavior, only solver performance.
3. Test suite includes ≥ 5 contact scenarios run with both PGS and CG, verifying force equivalence within tolerance.
4. Test suite includes `CGStrict` mode tests that fail on non-convergence rather than silently falling back.
5. Benchmark showing stable CG iteration count as contact count grows (expected crossover vs PGS at ~100 simultaneous contacts).

#### Files
- `sim/L0/constraint/src/cg.rs` — modify (new `cg_solve_contacts()`)
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`SolverType` enum, dispatch in `mj_fwd_constraint()`)
- `sim/L0/mjcf/src/parser.rs` — modify (`solver` attribute parsing)

---
## Group B — Pipeline Integration

### 4. Tendon Pipeline
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
sim-tendon is a standalone 3,919-line crate with path routing (`cable.rs`, `spatial.rs`,
`wrapping.rs`, `pulley.rs`, `fixed.rs`), a `TendonActuator` trait (`tendon/src/lib.rs:146`),
and zero coupling to the MuJoCo pipeline.

Model fields are fully declared (`mujoco_pipeline.rs:907-929`): `ntendon`,
`tendon_stiffness`, `tendon_damping`, `tendon_lengthspring`, `tendon_length0`,
`tendon_range`, `tendon_limited`, `tendon_num`, `tendon_adr`, `tendon_name`.

Data scaffolds exist (`mujoco_pipeline.rs:1366-1376`): `ten_length`, `ten_velocity`,
`ten_force`, `ten_J` — all initialized to defaults at `mujoco_pipeline.rs:1735-1738`
and never populated.

`mj_fwd_actuation()` (`mujoco_pipeline.rs:5475`) has an explicit placeholder at
`mujoco_pipeline.rs:5495-5498`:
```rust
ActuatorTransmission::Tendon | ActuatorTransmission::Site => {
    // Placeholder for tendon/site actuation
}
```

#### Objective
Wire tendon kinematics, passive forces, and actuation into the MuJoCo pipeline so
that tendon-driven actuators produce joint forces.

#### Specification

**`mj_fwd_tendon(model: &Model, data: &mut Data)`** — position-dependent:

1. For each tendon t = 0..ntendon:
   - Walk the tendon path (wrap objects from `tendon_adr[t]` to `tendon_adr[t] + tendon_num[t]`).
   - Compute `ten_length[t]` as the sum of segment lengths along the path.
   - Compute `ten_J[t]` (Jacobian mapping joint velocities to tendon length change)
     via finite differencing or analytic path derivatives.
2. Call site: after forward kinematics, before force computation.

**`mj_fwd_tendon_vel(model: &Model, data: &mut Data)`** — velocity-dependent:

1. For each tendon t: `ten_velocity[t] = ten_J[t].dot(&data.qvel)`.
2. Call site: after `mj_fwd_tendon()` and velocity computation.

**Tendon passive forces:**

1. Spring force: `f_spring = -tendon_stiffness[t] * (ten_length[t] - tendon_lengthspring[t])`.
2. Damping force: `f_damp = -tendon_damping[t] * ten_velocity[t]`.
3. Limit force: if `tendon_limited[t]` and length outside `tendon_range[t]`,
   apply Baumgarte-style restoring force using solref/solimp.
4. `ten_force[t] = f_spring + f_damp + f_limit`.
5. Map to joint forces: `qfrc_passive += ten_J[t]^T * ten_force[t]`.

**Tendon actuation** (fills the placeholder at `mujoco_pipeline.rs:5495`):

For `ActuatorTransmission::Tendon`: look up the target tendon via `actuator_trnid`.
Apply `ctrl * gear` as a force along the tendon, mapped to joints via:
`qfrc_actuator += ten_J[tendon_id]^T * (ctrl * gear)`.

#### Acceptance Criteria
1. `ten_length`, `ten_velocity`, `ten_force`, `ten_J` are populated every step for all tendons.
2. A tendon-driven actuator produces the same joint torque as a direct joint actuator with equivalent moment arm (within 1e-10 tolerance).
3. Tendon passive spring/damper forces appear in `qfrc_passive` and affect joint dynamics.
4. Tendon limit forces activate when length exceeds `tendon_range` and use solref/solimp parameters.
5. Zero-tendon models (`ntendon == 0`) have zero overhead.
6. The `ActuatorTransmission::Tendon` placeholder is replaced with working code.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_tendon()`, `mj_fwd_tendon_vel()`, tendon forces in `mj_fwd_passive()`, tendon actuation in `mj_fwd_actuation()`)
- `sim/L0/tendon/src/` — reference for path computation algorithms

---

### 5. Muscle Pipeline
**Status:** Not started | **Effort:** L | **Prerequisites:** #4 (tendon pipeline provides `ten_length`, `ten_velocity`, `ten_J`)

#### Current State
sim-muscle is a standalone 2,550-line crate with:
- `ActivationDynamics` (`activation.rs`): first-order `da/dt = (u - a) / τ(u, a)` with asymmetric time constants (τ_act ≈ 10-20 ms, τ_deact ≈ 40-80 ms).
- `HillMuscle` (`hill.rs`): contractile element (force-length, force-velocity), parallel elastic element, series elastic tendon. Presets: biceps, triceps, etc.
- Force-length/velocity curves (`curves.rs`).
- `MuscleKinematics` (`kinematics.rs`): pennation angle, fiber length from MTU length.

The pipeline declares `ActuatorDynamics::Muscle` (`mujoco_pipeline.rs:434`) and
`data.act` (`mujoco_pipeline.rs:1249`) but `mj_fwd_actuation()` has no muscle-specific
code path — it only handles `ActuatorTransmission::Joint` (direct force) and stubs
`::Tendon`/`::Site`.

#### Objective
Wire muscle activation dynamics and Hill-type force generation into the pipeline so
that muscle actuators produce physiologically realistic joint forces.

#### Specification

**Activation dynamics** (in `mj_fwd_actuation()` or a new `mj_fwd_activation()`):

For each actuator where `actuator_dyntype[i] == Muscle`:

1. Read excitation `u = ctrl[i]` (clamped to [0, 1]).
2. Update activation: `act[i] += dt * (u - act[i]) / τ(u, act[i])` where
   τ = τ_act if u > act[i], τ = τ_deact otherwise.
3. Clamp `act[i]` to [min_activation, 1.0].

**Force generation** (after tendon kinematics from #4):

1. Get muscle-tendon unit length from `ten_length[tendon_id]` (the tendon this
   muscle actuator drives) and velocity from `ten_velocity[tendon_id]`.
2. Compute fiber length and velocity via pennation angle geometry
   (from sim-muscle `MuscleKinematics`).
3. Compute active force: `F_active = act[i] * f_l(l_fiber) * f_v(v_fiber) * F_max`.
4. Compute passive force: `F_passive = f_pe(l_fiber) * F_max`.
5. Total muscle force: `F_muscle = (F_active + F_passive) * cos(pennation_angle)`.
6. Map to joint forces via tendon Jacobian: `qfrc_actuator += ten_J[tendon_id]^T * F_muscle`.

**Model fields needed** (new or repurposed):
- `actuator_muscle_config: Vec<Option<HillMuscleConfig>>` — per-actuator muscle parameters (F_max, optimal fiber length, tendon slack length, pennation angle, time constants).
- Parse from MJCF `<muscle>` element attributes.

#### Acceptance Criteria
1. `data.act` is updated every step for all muscle actuators using the first-order activation model.
2. Muscle force follows the Hill model: zero at zero activation, monotonically increasing with activation, exhibiting force-length and force-velocity relationships.
3. A muscle actuator at full activation with optimal fiber length produces `F_max * cos(pennation_angle)` force along the tendon.
4. Activation dynamics exhibit correct asymmetry: activation faster than deactivation.
5. Non-muscle actuators (`ActuatorDynamics::None/Filter/Integrator`) are unaffected.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`mj_fwd_actuation()` or new `mj_fwd_activation()`, muscle force computation)
- `sim/L0/muscle/src/` — reference for Hill model and activation dynamics
- `sim/L0/mjcf/src/parser.rs` — modify (parse `<muscle>` element)

---

### 6. Sensor Completion
**Status:** Not started | **Effort:** M | **Prerequisites:** None (some sensors benefit from #4 for tendon data)

#### Current State
The pipeline defines `MjSensorType` with 25+ variants (`mujoco_pipeline.rs:478`).
Many are fully implemented (JointPos, JointVel, Gyro, Accelerometer, FramePos, etc.).
The following are stubs or missing:

| Sensor | Location | Current Behavior | Correct Behavior |
|--------|----------|-----------------|-----------------|
| Force | `mujoco_pipeline.rs:5143-5150` | Returns `[0, 0, 0]` | Jacobian transpose mapping of `qfrc_constraint` to site frame |
| Torque | `mujoco_pipeline.rs:5153-5160` | Returns `[0, 0, 0]` | Jacobian transpose mapping of constraint torques to site frame |
| Touch | `mujoco_pipeline.rs:4897-4909` | `depth * 10000.0` hardcoded stiffness | Actual contact normal force from `efc_lambda` for contacts involving the sensor geom |
| Rangefinder | `mujoco_pipeline.rs:4911-4916` | Returns `sensor_cutoff` (max range) | Ray cast along site Z axis, return distance to nearest geom |
| Magnetometer | `mujoco_pipeline.rs:494` (enum only) | Falls to `_ => {}` (no output) | Magnetic field vector at site (requires field model or constant) |
| TendonPos | `mujoco_pipeline.rs:504` (enum only) | Falls to `_ => {}` | `ten_length[tendon_id]` (requires #4) |
| TendonVel | `mujoco_pipeline.rs:506` (enum only) | Falls to `_ => {}` | `ten_velocity[tendon_id]` (requires #4) |
| ActuatorPos | `mujoco_pipeline.rs:508` (enum only) | Falls to `_ => {}` | Actuator length (transmission-dependent) |
| ActuatorVel | `mujoco_pipeline.rs:510` (enum only) | Falls to `_ => {}` | Actuator velocity (transmission-dependent) |
| SubtreeAngMom | `mujoco_pipeline.rs:540` (enum only) | Falls to `_ => {}` | Angular momentum of subtree rooted at body |

For RL training where observations come from sensors, stubs returning zeros or
hardcoded values produce incorrect training signals.

#### Objective
Replace all sensor stubs with correct implementations so that `data.sensordata`
reflects true physical quantities.

#### Specification

**Force / Torque sensors** (highest RL impact):
1. Identify the site's parent body.
2. Compute the site Jacobian (translation + rotation) mapping joint velocities to site-frame motion.
3. Project `qfrc_constraint` through Jacobian transpose to get 3D force/torque at the site.

**Touch sensor:**
1. For each contact involving the sensor's geom, look up the solved contact force from `efc_lambda`.
2. Sum the normal force components: `force = Σ |λ_n|` for matching contacts.

**Rangefinder:**
1. Cast a ray from the site position along the site Z axis.
2. Test against all geoms (use existing collision infrastructure for ray-geom tests).
3. Return the minimum hit distance, or `sensor_cutoff` if no hit within range.

**Magnetometer:**
1. MuJoCo uses a constant global magnetic field `model.opt.magnetic` (3D vector).
2. Transform to site frame: `B_site = R_site^T * B_global`.

**TendonPos / TendonVel:** Read `ten_length[id]` / `ten_velocity[id]` (trivial once #4 lands; can stub with 0.0 until then).

**ActuatorPos / ActuatorVel:** Compute from transmission type — for joint actuators: joint position/velocity; for tendon actuators: tendon length/velocity.

**SubtreeAngMom:** Accumulate `I_i * ω_i` for all bodies in the subtree rooted at the sensor's body.

#### Acceptance Criteria
1. Force and Torque sensors return non-zero values when constraint forces are active.
2. Touch sensor returns actual contact force magnitude (not `depth * 10000`).
3. Rangefinder returns correct distance for a known geom placement (within 1e-6).
4. Magnetometer returns the global magnetic field rotated into the site frame.
5. TendonPos/TendonVel return `ten_length`/`ten_velocity` when tendon pipeline (#4) is active.
6. No sensor match arm falls through to `_ => {}` — all defined `MjSensorType` variants have explicit implementations (even if some return 0 with a documented reason).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (sensor readout functions at `mj_sensor_pos()`, `mj_sensor_vel()`, `mj_sensor_acc()`)

---
<!-- PLACEHOLDER: Group C — Integrator & Dynamics (Items 7–8) -->
<!-- PLACEHOLDER: Group D — Deformable Body (Item 9) -->
<!-- PLACEHOLDER: Group E — Scaling & Performance (Items 10–11) -->
<!-- PLACEHOLDER: Cleanup Tasks -->
<!-- PLACEHOLDER: Completed Work (Reference) -->
