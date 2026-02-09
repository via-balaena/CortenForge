# Future Work 5 — Quality of Life + Appendix (Items #15–17)

Part of [Simulation Phase 2 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

---

### 15. Newton Solver
**Status:** Not started | **Effort:** XL | **Prerequisites:** None

#### Current State
The standalone Newton solver implementation was deleted during Phase 3 crate
consolidation (commit `a5cef72`) — `sim/L0/constraint/src/newton.rs` (2,273
lines) was removed. The pipeline has PGS and CG (PGD+BB) solvers.

**MJCF-layer remnants still exist.** `MjcfSolverType::Newton` is defined in
`types.rs:95` and is the MuJoCo default. The parser accepts `solver="Newton"`
(`parser.rs:2201`). The model builder silently maps it to PGS:
`MjcfSolverType::PGS | MjcfSolverType::Newton => SolverType::PGS`
(`model_builder.rs:568`). Users who load standard MJCF models (which default to
Newton) get PGS without any warning.

MuJoCo's Newton solver uses analytical second-order derivatives of the constraint
cost and typically converges in 2–3 iterations, making it faster than PGS for
stiff problems. It is the recommended solver for most MuJoCo applications.

#### Objective
Implement a Newton contact solver in the pipeline, operating on the same
`assemble_contact_system()` infrastructure as PGS and CG. Update the MJCF
builder to route `MjcfSolverType::Newton` to the new solver instead of the
current silent PGS fallback.

#### Specification

Intentionally sparse — this is a large effort and should be fully specified at
implementation time. Key design points:

- Reuse `assemble_contact_system()` for Delassus matrix + RHS
- Newton direction: solve `H * delta = -g` where H is the Hessian of the
  constraint cost and g is the gradient
- Line search or trust region for step size
- Add `SolverType::Newton` variant (runtime) and wire `MjcfSolverType::Newton`
  to it in `model_builder.rs` (replacing the current silent PGS mapping)
- Projected Newton (respect `lambda_n >= 0` bounds)

#### Acceptance Criteria
1. Newton solver converges to same solution as PGS/CG (within tolerance).
2. Newton solver converges in fewer iterations than PGS for stiff contacts.
3. Falls back to PGS if Newton fails to converge.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (new solver function, SolverType
  variant, dispatch in `mj_fwd_constraint()`)

---

### 16. Sleeping / Body Deactivation
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
No sleep/deactivation system. Every body is simulated every step regardless of
whether it is stationary. MuJoCo deactivates bodies whose velocity is below a
threshold for a configurable duration, skipping their dynamics until an external
force or contact wakes them.

#### Objective
Bodies at rest are automatically deactivated, reducing computation for scenes with
many stationary objects.

#### Specification

Per-body sleep state:

```rust
pub body_sleep_time: Vec<f64>,   // time at zero velocity (in Model or Data)
pub body_asleep: Vec<bool>,      // deactivation flag
```

In `Data::step()`, after integration:
1. For each body, if `|v| < sleep_threshold` for `sleep_duration` steps, set
   `body_asleep = true`.
2. Asleep bodies skip FK, force computation, and integration.
3. Wake on: external force applied, contact with awake body, `ctrl` change on
   attached actuator.

#### Acceptance Criteria
1. Stationary bodies deactivate after configurable duration.
2. Contact with an active body wakes sleeping bodies.
3. Scene with 100 resting bodies and 1 active body runs faster than all-active.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (sleep tracking, skip logic)

---

### 17. SOR Relaxation for PGS
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
PGS uses plain Gauss-Seidel (relaxation factor omega = 1.0). MuJoCo's PGS uses SOR
with configurable omega for accelerated convergence.

#### Objective
Add SOR relaxation parameter to PGS solver.

#### Specification

In `pgs_solve_with_system()` (`mujoco_pipeline.rs:8049`), after the GS update:

```rust
lambda_new = (1 - omega) * lambda_old + omega * lambda_gs;
```

where `omega` is read from `Model.solver_sor` (default 1.0, parsed from MJCF
`<option sor="..."/>`).

#### Acceptance Criteria
1. omega = 1.0 matches current behavior (regression).
2. omega = 1.3 converges in fewer iterations for a stiff contact benchmark.
3. omega < 1.0 (under-relaxation) is stable for pathological cases.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — modify (`pgs_solve_with_system()`,
  Model field)

---

## Appendix: Deferred / Low Priority

Items acknowledged but not prioritized for Phase 2:

| Item | Reason for deferral |
|------|-------------------|
| `dampratio` attribute (Position actuators) | Builder-only change, no RL models use it. Deferred in Phase 1 #12. |
| Explicit Euler integrator | MuJoCo supports it but semi-implicit Euler is strictly superior. No use case. |
| Velocity Verlet integrator | Not in MuJoCo. Was in standalone system (deleted). No pipeline need. |
| ~~Implicit-fast (no Coriolis)~~ | ✅ Implemented in §13 as `Integrator::ImplicitFast`. |
| Planar/Cylindrical joints in pipeline | In sim-constraint standalone. No MJCF models use them. |
| `<custom>` element (user data) | Low priority — no simulation effect. |
| `<extension>` element (plugins) | Large design effort, no immediate need. |
| `<visual>` element | L1 concern (sim-bevy), not core physics. |
| `<statistic>` element | Auto-computed model stats. Informational only. |
| Sleeping bodies in deformable | Depends on #16. (#11 deformable pipeline ✅ complete) |
| Sparse mass matrix (deeper MuJoCo parity) | Phase 1 #1/#2 cover the main path. Full sparse pipeline is diminishing returns. |
| MuJoCo conformance test suite | Important but orthogonal to features — can be built incrementally. Without this, acceptance criteria for items #1–#17 rely on ad-hoc verification rather than systematic comparison against MuJoCo reference outputs. Consider bootstrapping a minimal conformance harness (load model, step N times, compare state vectors against MuJoCo ground truth) as infrastructure that benefits all items. |
| SIMD utilization (unused batch ops) | sim-simd exists; utilization will come naturally with #9/#10. |
| Tendon equality constraints | Standalone in sim-constraint. Pipeline tendons work; equality coupling is rare. Runtime warning at `mujoco_pipeline.rs:8493` fires when models include them — these constraints are silently ignored. |

---

## Cross-Reference: Phase 1 Mapping

| Phase 1 # | Phase 2 # | Notes |
|-----------|-----------|-------|
| #9 (Deformable Body) | #11 | Transferred verbatim ✅ Complete |
| #10 (Batched Simulation) | #9 | Transferred verbatim |
| #11 (GPU Acceleration) | #10 | Transferred verbatim |
