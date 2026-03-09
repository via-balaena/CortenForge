# Phase 11 — Derivatives: Session Plan

13 sessions, each self-contained. The umbrella spec
(`PHASE11_UMBRELLA.md`) coordinates across context boundaries.

Phase 11 completes the derivative pipeline for CortenForge's v1.0 surface:
analytical position derivatives (§58), sensor output Jacobians (DT-47),
inverse dynamics FD wrapper (DT-51), quaternion subtraction Jacobians
(DT-52), skip-stage forward optimization (DT-53), and muscle actuator
velocity derivatives (DT-54). The existing derivative infrastructure
(Phases A–D: `mjd_transition_fd`, `mjd_smooth_vel`, `mjd_transition_hybrid`,
~2,746 lines in `derivatives.rs`) provides the foundation — Phase 11
fills the remaining gaps to match MuJoCo's full `mjd_*` API surface.

Each spec follows a four-phase cycle: **rubric** -> **spec** ->
**implement** -> **review** (create review document, then execute it).
The review phase catches weak implementations, spec deviations, and
untracked deferred work before moving to the next spec.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt — the
documents and commits are the state, not the context window.

---

## Task Assignment

Every Phase 11 task is assigned to exactly one deliverable:

| Task | Deliverable | Status | Rationale |
|------|-------------|--------|-----------|
| DT-52 | T1 session (Session 2) | | `mjd_subQuat` — small quaternion utility (~50–100 lines), self-contained |
| DT-54 | T1 session (Session 2) | | Muscle velocity derivatives — extends existing `mjd_smooth_vel` (~100–150 lines) |
| DT-51 | T1 session (Session 3) | | `mjd_inverseFD` — FD wrapper following established `mjd_transition_fd` pattern (~200 lines) |
| DT-53 | T1 session (Session 3) | | `mj_forwardSkip` — skip-stage dispatch, mechanical pipeline guard (~200–300 lines) |
| §58 | Spec A | | `mjd_smooth_pos` analytical position derivatives — L effort, algorithmic (FK/RNE/passive chain rules) |
| DT-47 | Spec B | | Sensor derivatives (C, D matrices) — L effort, per-sensor-type Jacobian logic |

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: DT-52 + DT-54)          <- independent
    |
    +-- Session 3 (T1: DT-51 + DT-53)          <- independent
    |
    +-- Sessions 4-8 (Spec A: §58)              <- independent
    |       |
    |       v (soft dep: Spec B can use analytical position derivatives
    |         for sensor C matrix; FD fallback works without)
    |   Sessions 9-13 (Spec B: DT-47)
```

### Dependency edges

| From -> To | Type | Specific dependency |
|-----------|------|---------------------|
| Spec A (§58) -> Spec B (DT-47) | **Soft** | Spec B's sensor C matrix (`∂y/∂qpos`) can use §58's analytical position derivatives for the position-dependent chain rule. Without §58, FD position columns still work — just slower. |

All other deliverables are independent — T1 items (Sessions 2–3) and
Spec A can proceed in parallel. Spec B should ideally start after
Spec A to take advantage of analytical position derivatives, but is
not blocked by it.

### Why T1 items are independent of specs

The T1 items extend existing derivative infrastructure in self-contained
ways:

- DT-52 (`mjd_subQuat`) is a pure math utility — quaternion subtraction
  Jacobians. No dependency on position or sensor derivatives.
- DT-54 (muscle velocity derivatives) extends `mjd_smooth_vel` for a
  specific actuator type. Independent of position derivatives (§58) and
  sensor derivatives (DT-47).
- DT-51 (`mjd_inverseFD`) wraps the existing `mj_inverse()` with the
  established FD perturbation pattern from `mjd_transition_fd()`.
- DT-53 (`mj_forwardSkip`) adds a `skipstage` parameter to the forward
  pipeline. Used by the FD derivative loop but does not require
  analytical position or sensor derivatives.

---

## Session 1: Phase 11 Umbrella

- [x] Complete

```
Phase 11 Derivatives -- write the umbrella spec.

Read these in order:
1. sim/docs/ROADMAP_V1.md (Phase 11 section)
2. sim/docs/todo/future_work_14.md (§58 — analytical position derivatives)
3. sim/docs/todo/future_work_10f.md (DT-47, DT-51, DT-52, DT-53, DT-54)
4. sim/docs/todo/spec_fleshouts/phase11_derivatives/SESSION_PLAN.md
5. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
   (structural template — multi-spec pipeline depth pass)
6. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
   (structural template — T1 session + multi-spec, includes T1 handling pattern)
7. sim/L0/core/src/derivatives.rs (existing infrastructure — Phases A-D)

Phase 11 is a derivative pipeline completion pass: analytical position
derivatives (§58), sensor output Jacobians (DT-47), inverse dynamics
FD wrapper (DT-51), quaternion subtraction Jacobians (DT-52),
skip-stage forward optimization (DT-53), and muscle actuator velocity
derivatives (DT-54). All tasks converge on the derivative subsystem
in sim-core, building on the existing ~2,746-line Phases A-D
infrastructure.

Write PHASE11_UMBRELLA.md covering:
- Scope statement with conformance mandate
- Task Assignment table (mirror SESSION_PLAN.md assignments)
- Existing infrastructure summary (Phases A-D: what exists, what's missing)
- Sub-spec scope statements with MuJoCo C source citations:
  - Spec A: Analytical Position Derivatives (§58)
  - Spec B: Sensor Derivatives C/D Matrices (DT-47)
- T1 scope: DT-52 (mjd_subQuat) + DT-54 (muscle velocity derivatives),
  DT-51 (mjd_inverseFD) + DT-53 (mj_forwardSkip)
- Dependency Graph (Spec B soft-depends on Spec A)
- File Ownership Matrix (which spec touches each shared file)
- API Contracts (cross-spec boundaries: e.g., Spec A exposes
  mjd_smooth_pos that Spec B consumes for analytical sensor C matrix)
- Shared Convention Registry (derivative naming, matrix layout,
  perturbation patterns, tangent-space conventions)
- Cross-Spec Blast Radius
- Phase-Level Acceptance Criteria (PH11-AC1 through PH11-AC6)
- Out of Scope (explicit exclusions: full analytical contact derivatives
  DT-46, automatic differentiation DT-50, full position-analytical
  dFK/dq DT-45, sparse derivative storage DT-48, parallel FD DT-49)

Key differences from Phase 10 umbrella:
- Phase 11 builds on extensive existing infrastructure (Phases A-D)
  rather than creating new subsystems
- All tasks touch the same file (derivatives.rs) — high blast radius
  within the file, low blast radius across codebase
- Two soft-dependency chains (Spec B uses Spec A) vs Phase 10's
  hard-dependency chain
- T1 items are larger than Phase 10's (DT-51/DT-53 are M effort)
  but follow established patterns
- No trait architecture — pure function additions to existing module

Write to: sim/docs/todo/spec_fleshouts/phase11_derivatives/
MuJoCo conformance is the cardinal goal.
```

---

## Session 2: T1 items (DT-52 mjd_subQuat + DT-54 muscle velocity derivatives)

- [x] Complete

```
Phase 11 Derivatives -- implement T1 items (analytical extensions).

Read these first:
- sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md
  (Scope, Convention Registry sections)
- sim/docs/todo/future_work_10f.md (DT-52 at line 23, DT-54 at line 25)
- sim/L0/core/src/derivatives.rs (existing mjd_smooth_vel, mjd_quat_integrate)

Two deliverables in one session:

**DT-52 — `mjd_subQuat`: Quaternion Subtraction Jacobians:**
Compute the 3×3 Jacobians of quaternion subtraction (angle-axis
difference) with respect to each input quaternion:
- `d(q1 ⊖ q2)/dq1` — 3×3 matrix
- `d(q1 ⊖ q2)/dq2` — 3×3 matrix
Where `⊖` is the quaternion logarithm difference: `log(q2^{-1} * q1)`.
MuJoCo ref: `mjd_subQuat()` in `engine_derivative.c`.

Steps:
1. Implement `mjd_sub_quat(q1, q2) -> (Matrix3<f64>, Matrix3<f64>)`
2. Derive Jacobians through quaternion product + logarithm chain rule
3. Export from `lib.rs`
4. Tests: verify against finite-difference Jacobians for random quaternion
   pairs, identity case, small-angle limit, antipodal quaternions

**DT-54 — Muscle actuator velocity derivatives:**
Extend `mjd_smooth_vel()` to compute analytical velocity derivatives
for Hill-type muscle actuators. Currently muscle actuators are skipped
(captured via FD only).
MuJoCo ref: `mjd_actuator_vel()` in `engine_derivative.c` — muscle
force-velocity curve gradients.

Steps:
1. Implement piecewise FLV curve gradient for Hill-type muscle model
   (force-length and force-velocity partial derivatives)
2. Add `GainType::Muscle` / `GainType::HillMuscle` case to the
   actuator velocity derivative loop in `mjd_smooth_vel()`
3. Accumulate `∂qfrc_actuator/∂qvel` contributions via `J^T * dF/dv * J`
4. Tests: verify analytical muscle velocity derivatives match FD within
   tolerance for single-muscle and multi-muscle models

Both items extend existing analytical derivative infrastructure — no
new architectural patterns. Run `cargo test -p sim-core -p sim-mjcf
-p sim-conformance-tests` after each deliverable. MuJoCo conformance
is the cardinal goal.
```

---

## Session 3: T1 items (DT-51 mjd_inverseFD + DT-53 mj_forwardSkip)

- [x] Complete

```
Phase 11 Derivatives -- implement T1 items (FD infrastructure).

Read these first:
- sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md
  (Scope, Convention Registry sections)
- sim/docs/todo/future_work_10f.md (DT-51 at line 22, DT-53 at line 24)
- sim/L0/core/src/derivatives.rs (existing mjd_transition_fd pattern)

Two deliverables in one session:

**DT-51 — `mjd_inverseFD`: Inverse Dynamics Derivatives:**
Finite-difference wrapper around `mj_inverse()` computing Jacobians
of inverse dynamics outputs with respect to state:
- `DfDq`: `∂qfrc_inverse/∂qpos` (nv × nv)
- `DfDv`: `∂qfrc_inverse/∂qvel` (nv × nv)
- `DfDa`: `∂qfrc_inverse/∂qacc` (nv × nv)
MuJoCo ref: `mjd_inverseFD()` in `engine_derivative.c`.

Steps:
1. Define `InverseDynamicsDerivatives` struct with DfDq, DfDv, DfDa fields
2. Implement `mjd_inverse_fd(model, data, config) -> InverseDynamicsDerivatives`
   following the same perturbation pattern as `mjd_transition_fd`:
   - Save state → perturb qpos/qvel/qacc → call inverse() → measure
     qfrc_inverse → compute column → restore state
   - Position perturbations via `mj_integrate_pos_explicit()` (tangent space)
   - Support centered and forward FD via `DerivativeConfig`
3. Export from `lib.rs`
4. Tests: DfDa ≈ M (mass matrix) for simple models, verify all three
   matrices against known analytical results for pendulum/double-pendulum,
   centered vs forward FD convergence

**DT-53 — `mj_forwardSkip`: Skip-Stage Forward Optimization:**
Add a skip-stage variant of `forward()` that conditionally skips
position-dependent pipeline stages (FK, collision) when only
velocities or controls have been perturbed.
MuJoCo ref: `mj_forwardSkip(m, d, skipstage, skipsensor)` in
`engine_forward.c`.

Steps:
1. Define `MjStage` enum: `None`, `Pos`, `Vel` (matching MuJoCo's
   `mjSTAGE_NONE`, `mjSTAGE_POS`, `mjSTAGE_VEL`)
2. Implement `forward_skip(model, data, skipstage, skipsensor)`:
   - If skipstage < Pos: run fwd_position + sensor_pos + energy_pos
   - If skipstage < Vel: run fwd_velocity + sensor_vel + energy_vel
   - Always: run fwd_actuation + fwd_acceleration + fwd_constraint
   - If !skipsensor: run sensor_acc
3. Update `mjd_transition_fd()` to use `forward_skip(Vel)` for
   velocity/control perturbation columns (position data unchanged)
4. Export from `lib.rs`
5. Tests: verify forward_skip(None) == forward() (identical results),
   verify forward_skip(Vel) produces correct derivatives when only
   velocities perturbed, benchmark showing ~30-50% cost reduction
   for velocity/control FD columns

Both items extend FD infrastructure using established patterns. Run
`cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each deliverable. MuJoCo conformance is the cardinal goal.
```

---

## Session 4: Spec A rubric (Analytical Position Derivatives)

- [x] Complete (2d8ffa4)

```
Phase 11 Derivatives -- write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md

Spec A covers §58: `mjd_smooth_pos` — analytical position derivatives
for smooth dynamics (FK + RNE + passive forces). This replaces the
finite-difference position columns in `mjd_transition_hybrid()` with
analytical computation, enabling fully analytical transition Jacobians.

MuJoCo ref:
- `engine_derivative.c` → `mjd_smooth_pos()` — analytical derivatives
  of smooth forces with respect to generalized positions.
- Three components:
  1. FK position derivatives: ∂xpos/∂qpos, ∂xquat/∂qpos — chain rule
     through kinematic tree joint transforms
  2. RNE position derivatives: ∂qfrc_bias/∂qpos — gravity torques,
     centrifugal/Coriolis terms differentiated w.r.t. joint positions
  3. Passive force position derivatives: ∂qfrc_passive/∂qpos — joint
     spring forces, tendon spring forces
- Output stored in `qDeriv` (nv×nv) alongside velocity derivatives,
  or in separate position derivative array.
- Integration: `mjd_transition_hybrid()` replaces FD position columns
  with analytical columns from `mjd_smooth_pos()`.

Key areas to validate:
- Chain-rule structure through kinematic tree (parent-to-child propagation)
- Quaternion joint position derivatives (Ball/Free joints)
- Gravity torque position derivatives (body mass × position derivative)
- Coriolis/centrifugal position derivatives
- Spring force position derivatives (linear + quaternion geodesic)
- Tendon spring position derivatives (via tendon Jacobian)
- Integration into mjd_transition_hybrid: position columns fully analytical
- Performance: ≥1.5× speedup over FD position columns
- Accuracy: analytical matches FD within 1e-6 relative tolerance

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for mjd_smooth_pos computation
- Phase 2: Build SPEC_A_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase11_derivatives/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 5: Spec A spec (Analytical Position Derivatives)

- [x] Complete

```
Phase 11 Derivatives -- write Spec A spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_A_RUBRIC.md

Write SPEC_A.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- S1: FK position derivatives — ∂xpos/∂qpos, ∂xmat/∂qpos chain rule
  through kinematic tree. Per-joint-type differentiation (hinge, slide,
  ball, free). Parent-to-child propagation of position Jacobians.
- S2: RNE position derivatives — ∂qfrc_bias/∂qpos. Gravity torque
  derivatives (∂(J^T m g)/∂q), centrifugal/Coriolis derivatives.
  Backward pass through kinematic tree.
- S3: Passive force position derivatives — ∂qfrc_passive/∂qpos.
  Joint spring stiffness (diagonal for hinge/slide, quaternion geodesic
  for ball/free). Tendon spring via ∂(k · J^T · elongation)/∂q.
- S4: Assembly — combine FK, RNE, and passive position derivatives
  into `qDeriv` position columns (or separate storage).
  `mjd_smooth_pos(model, data)` public function signature.
- S5: Integration into hybrid transition — replace FD position columns
  in `mjd_transition_hybrid()` with analytical columns from S4.
  Verify transition A matrix matches FD within tolerance.
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase11_derivatives/
MuJoCo conformance is the cardinal goal.
```

---

## Session 6: Spec A implementation

- [x] Complete (276bed5)

```
Phase 11 Derivatives -- implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Start with FK position derivatives (chain rule through kinematic tree),
then RNE position derivatives, then passive force position derivatives,
then assembly into mjd_smooth_pos(), then integration into
mjd_transition_hybrid(). Verify that analytical position derivatives
match finite-difference position derivatives within 1e-6 relative
tolerance for models with hinge, slide, ball, and free joints.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 7: Spec A review — create review document

- [x] Complete

```
Phase 11 Derivatives -- create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_A.md

Copy the review template into this directory as SPEC_A_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_A_REVIEW.md
```

---

## Session 8: Spec A review — execute review

- [x] Complete

```
Phase 11 Derivatives -- execute Spec A review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_A.md
3. The implementation files listed in SPEC_A.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

Present the Review Verdict (section 10) to the user when done.
```

---

## Session 9: Spec B rubric (Sensor Derivatives)

- [ ] Complete

```
Phase 11 Derivatives -- write Spec B rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md

Spec B covers DT-47: Sensor derivatives (C, D matrices) for
`TransitionMatrices`. This populates the currently-`None` C and D
fields with sensor output Jacobians.

MuJoCo ref:
- `engine_derivative.c` → `mjd_transitionFD()` — computes C and D
  matrices via finite differencing of sensor outputs alongside state
  transition derivatives.
- C matrix: `∂sensordata_{t+1}/∂x_t` (nsensordata × (2*nv + na))
- D matrix: `∂sensordata_{t+1}/∂u_t` (nsensordata × nu)
- Sensor outputs are measured after each perturbation step — same
  FD loop as A/B matrices, just also recording sensordata deltas.
- MuJoCo also has `mjd_inverseFD()` sensor columns (DsDq, DsDv, DsDa)
  but those are separate from transition C/D.

Key areas to validate:
- C/D matrix computation via FD perturbation loop (extend existing
  mjd_transition_fd infrastructure)
- Per-sensor-type output handling (position-stage, velocity-stage,
  acceleration-stage sensors all captured by black-box step+measure)
- Sensor noise and cutoff interaction with derivatives
- Integration into both mjd_transition_fd() and mjd_transition_hybrid()
- Hybrid optimization: analytical sensor position derivatives (if §58
  Spec A is available) for C matrix velocity/activation columns
- Matrix dimensions: nsensordata × state_dim for C, nsensordata × nu for D
- Opt-in API: C/D only computed when requested (avoid cost when unused)

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for sensor derivative computation
- Phase 2: Build SPEC_B_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase11_derivatives/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 10: Spec B spec (Sensor Derivatives)

- [ ] Complete

```
Phase 11 Derivatives -- write Spec B spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_B_RUBRIC.md

Write SPEC_B.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- S1: DerivativeConfig extension — add `compute_sensor_derivatives: bool`
  flag (default false) to opt in to C/D computation. Avoids cost when
  only A/B matrices are needed.
- S2: FD sensor derivative loop — extend `mjd_transition_fd()` to
  record `sensordata` deltas during the existing perturbation loop.
  For each perturbation column (qpos, qvel, act, ctrl), also compute
  the sensordata difference and store in C/D matrix columns.
- S3: Hybrid sensor derivatives — extend `mjd_transition_hybrid()` to
  populate C/D. Velocity/activation columns can use analytical
  `qDeriv` chain rule through sensor Jacobians (if available).
  Position columns use FD (or analytical from §58 if Spec A landed).
- S4: TransitionMatrices population — wire C and D fields from
  `Option<DMatrix<f64>>` = None to populated matrices when
  `compute_sensor_derivatives` is true.
- S5: Validation — verify C/D matrices match pure FD sensor derivatives
  within tolerance. Test with position-stage (framepos), velocity-stage
  (framevelocity), and acceleration-stage (frameacc) sensors.
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase11_derivatives/
MuJoCo conformance is the cardinal goal.
```

---

## Session 11: Spec B implementation

- [ ] Complete

```
Phase 11 Derivatives -- implement Spec B.

Read these:
1. sim/docs/todo/spec_fleshouts/phase11_derivatives/PHASE11_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_B.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Start with DerivativeConfig extension, then FD sensor loop, then hybrid
sensor path, then TransitionMatrices wiring. Test with models that have
diverse sensor types (framepos, framevelocity, jointpos, actuatorfrc,
touch, etc.) and verify C/D matrices capture sensor output sensitivity
to state and control perturbations.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 12: Spec B review — create review document

- [ ] Complete

```
Phase 11 Derivatives -- create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_B.md

Copy the review template into this directory as SPEC_B_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_B_REVIEW.md
```

---

## Session 13: Spec B review — execute review

- [ ] Complete

```
Phase 11 Derivatives -- execute Spec B review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_B.md
3. The implementation files listed in SPEC_B.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

This is the final session of Phase 11. After completing, verify the Phase 11
aggregate ACs from the umbrella (PH11-AC1 through PH11-AC6) are satisfied.

Present the Review Verdict (section 10) to the user when done.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | Phase 11 Umbrella | Done | 33ec878 |
| 2 | T1: DT-52 (mjd_subQuat) + DT-54 (muscle velocity derivatives) | Done | 9feebfe |
| 3 | T1: DT-51 (mjd_inverseFD) + DT-53 (mj_forwardSkip) | Done | c68d9cb |
| 4 | Spec A rubric (§58 Analytical Position Derivatives) | Done | 2d8ffa4 |
| 5 | Spec A spec | Done | 65e8c04 |
| 6 | Spec A implementation | Done | 276bed5 |
| 7 | Spec A review — create document | Done | |
| 8 | Spec A review — execute | Done | |
| 9 | Spec B rubric (DT-47 Sensor Derivatives) | | |
| 10 | Spec B spec | | |
| 11 | Spec B implementation | | |
| 12 | Spec B review — create document | | |
| 13 | Spec B review — execute | | |
