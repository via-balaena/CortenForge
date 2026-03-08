# Phase 11 — Derivatives: Umbrella Spec

**Status:** Stress-tested
**Phase:** Roadmap Phase 11
**Tasks:** 6 (§58, DT-47, DT-51, DT-52, DT-53, DT-54)
**Deliverables:** 2 T1 sessions + 2 sub-specs (Spec A, Spec B)
**Test baseline:** 3,600+ domain tests (post-Phase 10)

---

## Scope

Phase 11 completes the derivative pipeline for CortenForge's v1.0 API surface.
**The goal is MuJoCo conformance** — after Phase 11, CortenForge provides the
full `mjd_*` derivative API: analytical position derivatives (`mjd_smooth_pos`),
sensor output Jacobians (C/D matrices in `TransitionMatrices`), inverse dynamics
derivatives (`mjd_inverseFD`), quaternion subtraction Jacobians (`mjd_subQuat`),
skip-stage forward optimization (`mj_forwardSkip`), and muscle actuator velocity
derivatives. Every function matches MuJoCo's `engine_derivative.c` formulation.

Phase 11 builds on substantial existing infrastructure (Phases A–D, ~2,746
lines in `derivatives.rs`):

| Existing (Phases A–D) | Phase 11 adds |
|----------------------|---------------|
| `mjd_transition_fd` — pure FD transition derivatives | `mjd_smooth_pos` — analytical position derivatives (§58) |
| `mjd_smooth_vel` — analytical velocity derivatives | Muscle velocity derivatives in `mjd_smooth_vel` (DT-54) |
| `mjd_transition_hybrid` — hybrid FD+analytical (velocity columns analytical, position columns FD) | Fully analytical position columns via §58 |
| `TransitionMatrices { A, B, C: None, D: None }` | Populated C/D sensor matrices (DT-47) |
| `Data::inverse()` — inverse dynamics | `mjd_inverse_fd` — FD wrapper for inverse dynamics derivatives (DT-51) |
| `mjd_quat_integrate` — quaternion integration derivatives | `mjd_sub_quat` — quaternion subtraction Jacobians (DT-52) |
| `Data::forward()` — full pipeline | `forward_skip` — skip-stage variant for FD cost reduction (DT-53) |

All tasks converge on the derivative subsystem in `sim-core`. The key
structural features are:
1. **Single-file focus:** All 6 tasks primarily modify `derivatives.rs` (with
   `forward/mod.rs` touched by DT-53 only). Low cross-file blast radius, high
   intra-file blast radius.
2. **Existing infrastructure is mature:** Phases A–D provide the FD loop,
   tangent-space conventions, `qDeriv` storage, integration derivatives, and
   the hybrid dispatch. Phase 11 fills gaps in this framework, not builds
   from scratch.
3. **One soft dependency:** Spec B (sensor C/D) benefits from Spec A's
   analytical position derivatives for the C matrix, but the FD fallback
   works without it.
4. **T1 items follow established patterns:** DT-51 follows `mjd_transition_fd`'s
   perturbation loop. DT-52 is a self-contained math utility. DT-53 adds
   conditional guards to the forward pipeline. DT-54 extends an existing
   `match` block in `mjd_actuator_vel`.

> **Conformance mandate for sub-spec authors:** Each sub-spec must cite the
> exact MuJoCo C source (function, file, line range) for every behavior it
> implements. Acceptance criteria must include expected values derived from
> MuJoCo's output — not from hand-calculation or intuition. The MuJoCo C
> source code is the single source of truth. When the MuJoCo docs and the
> C source disagree, the C source wins.

---

## Task Assignment

Every Phase 11 task is assigned to exactly one deliverable. No orphans, no
overlaps. Each task closes a specific MuJoCo derivative API gap.

| Task | Description | Deliverable | Status | Rationale |
|------|-------------|-------------|--------|-----------|
| DT-52 | `mjd_subQuat` — quaternion subtraction Jacobians | **T1** (Session 2) | Pending | M effort, self-contained math utility (~50–100 lines). Independent of other tasks. |
| DT-54 | Muscle actuator velocity derivatives — piecewise FLV curve gradients | **T1** (Session 2) | Pending | M effort, extends existing `mjd_actuator_vel` match block (~100–150 lines). |
| DT-51 | `mjd_inverseFD` — inverse dynamics derivatives | **T1** (Session 3) | Pending | M effort, follows established `mjd_transition_fd` perturbation pattern (~200 lines). |
| DT-53 | `mj_forwardSkip` — skip-stage forward optimization | **T1** (Session 3) | Pending | M effort, conditional pipeline guards + derivative loop integration (~70–100 lines). Existing `forward_pos_vel`/`forward_acc` split reduces scope. |
| §58 | `mjd_smooth_pos` — analytical position derivatives | **Spec A** | Pending | L effort, algorithmic. Chain-rule FK/RNE/passive position derivatives through kinematic tree. |
| DT-47 | Sensor derivatives (C, D matrices) for `TransitionMatrices` | **Spec B** | Pending | L effort, per-sensor-type Jacobian logic, extends FD and hybrid paths. |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo C functions it is porting, cite
them with source file and line ranges, and produce acceptance criteria with
MuJoCo-verified expected values.

**Spec A — Analytical Position Derivatives** (§58):
Analytical derivatives of smooth dynamics (FK + RNE + passive forces) with
respect to generalized positions. Replaces the finite-difference position
columns in `mjd_transition_hybrid()` with analytical computation, enabling
fully analytical transition Jacobians (~2× speedup for transition derivatives).

MuJoCo reference:
- `engine_derivative.c` → `mjd_smooth_pos()` — analytical derivatives of
  smooth forces with respect to positions.
- Three components:
  1. FK position derivatives: `∂xpos/∂qpos`, `∂xmat/∂qpos` — chain rule
     through kinematic tree joint transforms. Per-joint-type differentiation
     (hinge rotation matrix derivative, slide translation derivative, ball
     quaternion derivative, free transform derivative).
  2. RNE position derivatives: `∂qfrc_bias/∂qpos` — gravity torque
     derivatives and centrifugal/Coriolis position terms. Backward pass
     through kinematic tree.
  3. Passive force position derivatives: `∂qfrc_passive/∂qpos` — joint
     spring stiffness (diagonal for hinge/slide, quaternion geodesic for
     ball/free), tendon spring via `∂(k · J^T · elongation)/∂q`.
- Output stored in `Data.qDeriv_pos` (see Contract 1), used by
  `mjd_transition_hybrid()` to replace FD position columns.

Primary changes: `sim/L0/core/src/derivatives.rs` (new `mjd_smooth_pos()`
function + integration into `mjd_transition_hybrid()` position columns).

**Spec B — Sensor Derivatives (C, D matrices)** (DT-47):
Populates the currently-`None` C and D fields of `TransitionMatrices` with
sensor output Jacobians. C matrix = `∂sensordata_{t+1}/∂x_t`, D matrix =
`∂sensordata_{t+1}/∂u_t`.

MuJoCo reference:
- `engine_derivative.c` → `mjd_transitionFD()` — computes C and D matrices
  via finite differencing of sensor outputs alongside state transition
  derivatives. The same perturbation loop that computes A/B columns also
  records `sensordata` deltas for C/D.
- Sensor outputs are measured after each perturbation step — black-box FD
  captures all sensor types automatically (position-stage, velocity-stage,
  acceleration-stage sensors).
- Opt-in computation: C/D only populated when explicitly requested (avoids
  cost when only A/B are needed).

Primary changes: `sim/L0/core/src/derivatives.rs` (extend `mjd_transition_fd()`
and `mjd_transition_hybrid()` to compute C/D, add `compute_sensor_derivatives`
to `DerivativeConfig`, wire `TransitionMatrices.C` and `.D`).

**T1 — Analytical Extensions** (DT-52 + DT-54, Session 2):
Two self-contained analytical derivative additions.

DT-52 (`mjd_subQuat`): Quaternion subtraction Jacobians — two 3×3 matrices
for `d(q1 ⊖ q2)/dq1` and `d(q1 ⊖ q2)/dq2` where `⊖` is the quaternion
logarithm difference. MuJoCo ref: `mjd_subQuat()` in `engine_derivative.c`.

DT-54 (muscle velocity derivatives): Extend `mjd_actuator_vel()` to compute
analytical velocity derivatives for `GainType::Muscle` and
`GainType::HillMuscle` actuators. Currently skipped (comment: "Muscle
actuators are SKIPPED — velocity derivatives involve piecewise FLV curve
gradients"). MuJoCo ref: `mjd_actuator_vel()` muscle branch in
`engine_derivative.c`.

**T1 — FD Infrastructure** (DT-51 + DT-53, Session 3):
Two FD pipeline extensions.

DT-51 (`mjd_inverseFD`): Finite-difference wrapper around `Data::inverse()`.
Perturbs `qpos`/`qvel`/`qacc`, calls `inverse()`, measures `qfrc_inverse`
deltas. Produces three nv×nv Jacobian matrices: `DfDq`, `DfDv`, `DfDa`.
MuJoCo ref: `mjd_inverseFD()` in `engine_derivative.c`.

DT-53 (`mj_forwardSkip`): Skip-stage variant of `forward()` that
conditionally skips position-dependent pipeline stages (FK, collision) when
only velocities or controls have been perturbed. Leverages the existing
`forward_pos_vel()`/`forward_acc()` split in `forward/mod.rs` — adds
conditional guards, not a rewrite (~70–100 lines). Reduces FD cost by
~30–50% for velocity/control perturbation columns. MuJoCo ref:
`mj_forwardSkip()` in `engine_forward.c`. See Contract 3 for
`forward_skip` vs `step` semantics.

---

## Existing Infrastructure Summary

Phase 11 builds on the four-phase derivative infrastructure implemented
across earlier work:

### Phase A: Pure FD (`mjd_transition_fd`)
- **What:** Black-box perturbation through `step()`. Works with any integrator.
- **State:** Complete, tested, exported.
- **Phase 11 touches:** DT-47 extends the perturbation loop to capture
  sensor outputs. DT-53 replaces `step()` with `forward_skip() + integrate()`
  for velocity/control columns (see Contract 3 for semantics).

### Phase B: Analytical velocity derivatives (`mjd_smooth_vel`)
- **What:** `∂(qfrc_smooth)/∂qvel` via `mjd_passive_vel` + `mjd_actuator_vel`
  + `mjd_rne_vel`. Stored in `Data.qDeriv`.
- **State:** Complete, tested, exported. Muscle actuators skipped.
- **Phase 11 touches:** DT-54 adds the muscle case to `mjd_actuator_vel`.

### Phase C: Integration derivatives (`mjd_quat_integrate`)
- **What:** Quaternion chain rules for `∂qpos/∂qvel` and `∂qpos/∂qpos`
  through Ball/Free joints.
- **State:** Complete, tested, exported.
- **Phase 11 touches:** None directly. DT-52 (`mjd_subQuat`) is a related
  quaternion utility but operates on subtraction, not integration.

### Phase D: Hybrid FD+analytical (`mjd_transition_hybrid`)
- **What:** Analytical `qDeriv` for velocity columns of A, FD for position
  columns. ~2× speedup over pure FD.
- **State:** Complete, tested, exported. Position columns still use FD.
- **Phase 11 touches:** §58 replaces FD position columns with analytical
  columns from `mjd_smooth_pos()`.

### Key types
- `TransitionMatrices { A, B, C: Option, D: Option }` — output struct.
- `DerivativeConfig { eps, centered, use_analytical }` — configuration.
- `Data.qDeriv` — dense nv×nv velocity derivative matrix.
- `Data.deriv_Dcvel/Dcacc/Dcfrc` — scratch matrices for RNE chain rule.
- `IntegrationDerivatives` — per-step integration Jacobians (internal).

### Key conventions
- **Tangent space:** All state vectors use nv dimensions (not nq). Position
  perturbations via `mj_integrate_pos_explicit()`, output via
  `mj_differentiate_pos()`.
- **State layout:** `x[0..nv] = dq`, `x[nv..2*nv] = qvel`,
  `x[2*nv..2*nv+na] = act`.
- **Perturbation pattern:** Save state → perturb → compute → measure →
  restore. Centered (O(ε²)) or forward (O(ε)).
- **Solver dispatch:** Per-integrator `M⁻¹` or `(M−hD)⁻¹` or
  `(M+hD+h²K)⁻¹` solve for force-to-acceleration mapping.

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: DT-52 + DT-54)           <- independent
    |
    +-- Session 3 (T1: DT-51 + DT-53)           <- independent
    |
    +-- Sessions 4-8 (Spec A: §58)               <- independent
    |       |
    |       v (soft dep: Spec B can use analytical position derivatives
    |         for sensor C matrix; FD fallback works without)
    |   Sessions 9-13 (Spec B: DT-47)
```

### Dependency edges

| From -> To | Type | Specific dependency |
|-----------|------|---------------------|
| Spec A (§58) -> Spec B (DT-47) | **Soft** | Spec B's sensor C matrix (`∂sensordata/∂qpos`) can use §58's analytical position derivatives for hybrid sensor columns. Without §58, FD position columns still work — just slower. Spec B must work with or without §58. |

All other deliverables are independent — T1 items (Sessions 2–3) and
Spec A can proceed in parallel. Spec B should ideally start after
Spec A to take advantage of analytical position derivatives, but is
not blocked by it.

### Why T1 items are independent of specs

- **DT-52** (`mjd_subQuat`): Pure math utility. Computes quaternion
  subtraction Jacobians. No dependency on position or sensor derivatives.
- **DT-54** (muscle velocity derivatives): Extends `mjd_actuator_vel` for
  a specific actuator type. Independent of position derivatives (§58) and
  sensor derivatives (DT-47). The muscle `dforce/dV` computation only
  needs the existing `actuator_length`, `actuator_velocity`, and FLV curve
  parameters — all available pre-Phase 11.
- **DT-51** (`mjd_inverseFD`): Wraps existing `Data::inverse()` with the
  established FD perturbation pattern from `mjd_transition_fd()`. Uses
  `forward()`, not `forward_skip()`, so no dependency on DT-53.
- **DT-53** (`mj_forwardSkip`): Adds `forward_skip(skipstage, skipsensor)`
  to the forward pipeline. FD loops switch from `step()` to
  `forward_skip() + integrate()`. Can be integrated into `mjd_transition_fd`
  and `mjd_transition_hybrid` independently of §58 or DT-47.

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.
Single-owner files are not listed.

### `sim/L0/core/src/derivatives.rs` (~2,746 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (DT-52) | Add `mjd_sub_quat()` function (~50–100 lines) | Quaternion subtraction Jacobians available |
| 1 | T1 (DT-54) | Add muscle case in `mjd_actuator_vel()` match block (~100–150 lines) | Muscle velocity derivatives analytical |
| 2 | T1 (DT-51) | Add `mjd_inverse_fd()` function + `InverseDynamicsDerivatives` struct (~200 lines) | Inverse dynamics FD wrapper available |
| 2 | T1 (DT-53) | Update `mjd_transition_fd()` and `mjd_transition_hybrid()` to use `forward_skip() + integrate()` for velocity/control columns (~50 lines of integration code) | FD cost reduced for velocity/control columns |
| 3 | Spec A (§58) | Add `mjd_smooth_pos()` function (~400–500 lines), update `mjd_transition_hybrid()` to use analytical position columns | Position columns fully analytical |
| 4 | Spec B (DT-47) | Extend `mjd_transition_fd()` and `mjd_transition_hybrid()` to compute C/D sensor matrices, add `compute_sensor_derivatives` to `DerivativeConfig` (~200–300 lines) | TransitionMatrices.C and .D populated |

**Conflict risk: Medium.** All deliverables modify `derivatives.rs`. However,
they touch different sections: DT-52 adds a new standalone function, DT-54
adds to an existing match block, DT-51 adds a new function, DT-53 modifies
the perturbation loop, §58 adds a new function + modifies hybrid dispatch,
DT-47 extends the perturbation loop + config struct. Ordered execution
prevents conflicts.

### `sim/L0/core/src/forward/mod.rs` (~300 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (DT-53) | Add `forward_skip(skipstage, skipsensor)` method on `Data` (~30–50 lines) | Skip-stage forward available |

**Conflict risk: None.** Single modifier in Phase 11. Adds a new method
alongside existing `forward()`, `forward_core()`, `forward_skip_sensors()`.

### `sim/L0/core/src/lib.rs` (~290 lines)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | T1 (DT-52) | Add `mjd_sub_quat` to exports | Exported |
| 2 | T1 (DT-51) | Add `mjd_inverse_fd`, `InverseDynamicsDerivatives` to exports | Exported |
| 3 | Spec A (§58) | Add `mjd_smooth_pos` to exports | Exported |

**Conflict risk: None.** Additive changes to the `pub use derivatives::` block.

---

## API Contracts

Cross-spec API dependencies. Sub-specs must be written against these
contracts, not against the current (pre-Phase 11) codebase.

### Contract 1: `mjd_smooth_pos` signature (Spec A, consumed by Spec B)

**After Spec A — expected API:**
```rust
/// Compute ∂(qfrc_smooth)/∂qpos analytically and store in data.qDeriv_pos.
///
/// Populates position derivative storage with three analytical contributions:
///   1. ∂(passive)/∂qpos via FK → spring chain rule
///   2. ∂(actuator)/∂qpos via FK → length chain rule (position-dependent actuators)
///   3. −∂(bias)/∂qpos via RNE position derivatives (gravity torques)
pub fn mjd_smooth_pos(model: &Model, data: &mut Data);
```

**Position derivative storage:** `mjd_smooth_pos()` stores its result in a
new `Data.qDeriv_pos: DMatrix<f64>` field (nv × nv), paralleling how
`mjd_smooth_vel()` stores velocity derivatives in `Data.qDeriv`. This
matches MuJoCo's approach where `mjd_smooth_pos` writes to a caller-provided
`DfDq` buffer — in CortenForge the buffer lives on `Data` so that
`mjd_transition_hybrid()` can access it directly when replacing FD position
columns with analytical ones.

**New Data field required by Spec A:**
```rust
/// ∂(qfrc_smooth)/∂qpos — analytical position derivative matrix.
/// Populated by mjd_smooth_pos(). Dimensions: nv × nv.
/// Parallel to qDeriv which stores ∂(qfrc_smooth)/∂qvel.
pub qDeriv_pos: DMatrix<f64>,
```

**What Spec B may use:** If `mjd_smooth_pos` has been called and
`data.qDeriv_pos` is populated, Spec B's hybrid sensor path can use it for
analytical C matrix columns. If not available, Spec B falls back to FD for
all C columns.

**What Spec A guarantees:** After `mjd_smooth_pos(model, data)`,
`data.qDeriv_pos` is populated and matches FD within `1e-6` relative
tolerance. The function does not modify `qDeriv` (velocity derivatives) —
`qDeriv` and `qDeriv_pos` are independent storage.

### Contract 2: `DerivativeConfig` extension (Spec B)

**After Spec B — expected API:**
```rust
pub struct DerivativeConfig {
    pub eps: f64,
    pub centered: bool,
    pub use_analytical: bool,
    /// When true, compute sensor derivatives (C, D matrices).
    /// Default: false (skip sensor derivatives for performance).
    pub compute_sensor_derivatives: bool,  // NEW
}
```

**What changes:** `mjd_transition_fd()` and `mjd_transition_hybrid()` check
`config.compute_sensor_derivatives`. When `false` (default), `C` and `D`
remain `None`. When `true`, the perturbation loop also records `sensordata`
differences and populates C/D matrices.

**Backward compatibility:** Existing callers that construct
`DerivativeConfig::default()` get `compute_sensor_derivatives: false` — no
behavior change.

### Contract 3: `forward_skip` signature (DT-53, consumed by FD loops)

**After DT-53 — expected API:**
```rust
/// Forward dynamics with skip-stage optimization.
///
/// Conditionally skips position-dependent stages when only velocities
/// or controls have changed from the baseline state.
///
/// `skipstage`:
///   - `MjStage::None` (0): run full pipeline (equivalent to forward())
///   - `MjStage::Pos` (1): skip position stage (FK, collision)
///   - `MjStage::Vel` (2): skip position and velocity stages
///
/// `skipsensor`: when true, skip all sensor evaluation.
pub fn forward_skip(
    &mut self,
    model: &Model,
    skipstage: MjStage,
    skipsensor: bool,
) -> Result<(), StepError>;
```

**Critical semantic note — `forward_skip` vs `step`:**
`forward_skip()` is forward-only — it does **not** integrate. The existing
FD perturbation loop calls `scratch.step(model)?` which does
`forward() + integrate()`. `forward_skip()` replaces only the `forward()`
half of that sequence.

After DT-53, the FD loop changes from:
```rust
scratch.step(model)?;  // forward() + integrate()
```
to:
```rust
scratch.forward_skip(model, skipstage, true)?;  // forward only (skipping stages)
scratch.integrate(model);                        // integration (always runs)
```

This matches MuJoCo's approach: `mj_forwardSkip()` is a forward-only
function. The FD loop in `mjd_transitionFD()` calls `mj_step()` which
internally uses `mj_forwardSkip()` for the forward pass, then integrates.
In CortenForge the FD loop makes the two calls explicitly.

**What FD loops use:** When perturbing velocity or control columns,
the perturbation loop calls `forward_skip(MjStage::Pos, true)` +
`integrate()` — skips FK and collision (unchanged by velocity/control
perturbations), saving ~30–50% cost per column. For position columns,
`forward_skip(MjStage::None, true)` + `integrate()` runs the full pipeline
(equivalent to `step()` without the check/sleep/warmstart overhead).

### Contract 4: `InverseDynamicsDerivatives` struct (DT-51)

**After DT-51 — expected API:**
```rust
/// Finite-difference derivatives of inverse dynamics.
#[allow(non_snake_case)]
pub struct InverseDynamicsDerivatives {
    /// ∂qfrc_inverse/∂qpos (nv × nv)
    pub DfDq: DMatrix<f64>,
    /// ∂qfrc_inverse/∂qvel (nv × nv)
    pub DfDv: DMatrix<f64>,
    /// ∂qfrc_inverse/∂qacc (nv × nv) — approximately equals M (mass matrix)
    pub DfDa: DMatrix<f64>,
}

pub fn mjd_inverse_fd(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<InverseDynamicsDerivatives, StepError>;
```

This is internal to the T1 session. No other Phase 11 deliverable depends
on it. The function follows the same perturbation pattern as
`mjd_transition_fd()`.

### Contract 5: DT-54 muscle velocity derivative data dependencies

**What DT-54 changes:** Removes the `continue;` skip guards in
`mjd_actuator_vel()` (derivatives.rs lines 531–536 and 537–542) for
`GainType::Muscle | GainType::HillMuscle` and
`BiasType::Muscle | BiasType::HillMuscle`, replacing them with analytical
muscle velocity derivative logic.

**Muscle force model recap:**
For muscle actuators, `force = gain(L, V, act) · input + bias(L, V, act)`.
The velocity derivative `∂force/∂V` requires differentiating the force-
velocity (FV) component of the FLV curve:
- `dgain_dv`: derivative of the gain function w.r.t. actuator velocity.
  For `GainType::Muscle`: piecewise linear FV curve parameterized by
  `actuator_gainprm[i]`. For `GainType::HillMuscle`: Hill-type FV function
  from `actuator_gainprm[i]`.
- `dbias_dv`: derivative of the bias function w.r.t. actuator velocity.
  Same structure as `dgain_dv`, using `actuator_biasprm[i]`.

**Data dependencies (all available pre-Phase 11):**
- `data.actuator_length[i]` — current actuator length (for FL curve evaluation)
- `data.actuator_velocity[i]` — current actuator velocity (for FV curve evaluation)
- `data.act[actuator_act_adr[i]]` — activation state
- `model.actuator_gainprm[i]` — gain FLV curve parameters
- `model.actuator_biasprm[i]` — bias FLV curve parameters
- `model.actuator_gear[i]` — gear ratio
- `model.actuator_trntype[i]` / `model.actuator_trnid[i]` — transmission info

**Structural pattern:** After computing `dforce_dv` for muscle actuators,
the existing transmission `match` block (derivatives.rs lines 569–612)
applies identically — muscle actuators use the same `gear · moment · qDeriv`
dispatch as non-muscle actuators. Only the `dforce_dv` computation differs.

**MuJoCo reference:** `engine_derivative.c` → `mjd_actuator_vel()`, muscle
branch. The C source computes `dFdV` from the FV curve derivative, then
multiplies through the transmission exactly as for other actuator types.

---

## Shared Convention Registry

Conventions decided once here. Sub-specs reference this section instead of
inventing their own.

### 1. Derivative function naming

All new derivative functions follow the existing `mjd_` prefix convention:

| MuJoCo C function | CortenForge function | File |
|-------------------|---------------------|------|
| `mjd_smooth_pos` | `mjd_smooth_pos` | derivatives.rs |
| `mjd_subQuat` | `mjd_sub_quat` | derivatives.rs |
| `mjd_inverseFD` | `mjd_inverse_fd` | derivatives.rs |
| `mjd_actuator_vel` (muscle branch) | `mjd_actuator_vel` (extended) | derivatives.rs |
| `mj_forwardSkip` | `Data::forward_skip` | forward/mod.rs |

Convention: MuJoCo's camelCase names are converted to Rust snake_case.
The `mjd_` prefix is preserved (not `mj_d_`).

### 2. Matrix layout conventions

All derivative matrices use nalgebra `DMatrix<f64>`:

| Matrix | Dimensions | Convention |
|--------|-----------|------------|
| A (state transition) | `(2*nv+na) × (2*nv+na)` | Row = output state, col = input state |
| B (control influence) | `(2*nv+na) × nu` | Row = output state, col = control |
| C (sensor-state) | `nsensordata × (2*nv+na)` | Row = sensor output, col = input state |
| D (sensor-control) | `nsensordata × nu` | Row = sensor output, col = control |
| DfDq (inverse pos) | `nv × nv` | Row = qfrc_inverse DOF, col = qpos tangent |
| DfDv (inverse vel) | `nv × nv` | Row = qfrc_inverse DOF, col = qvel DOF |
| DfDa (inverse acc) | `nv × nv` | Row = qfrc_inverse DOF, col = qacc DOF |
| qDeriv (smooth vel) | `nv × nv` | Row = force DOF, col = velocity DOF |
| qDeriv_pos (smooth pos) | `nv × nv` | Row = force DOF, col = position tangent DOF |

### 3. Tangent-space convention (unchanged)

All position perturbations operate in tangent space (dimension nv, not nq):
- **Perturbation:** `mj_integrate_pos_explicit(model, qpos_out, qpos_ref, dq, 1.0)`
  where `dq[i] = ε` for the column being computed.
- **Measurement:** `mj_differentiate_pos(model, dq, qpos_ref, qpos_out, 1.0)`
  to map coordinate-space displacement back to tangent space.
- **Why:** Avoids quaternion singularities for Ball and Free joints. Matches
  MuJoCo's `mjd_transitionFD` convention.

### 4. FD perturbation pattern (unchanged)

```rust
// Save → perturb → compute → measure → restore
let saved = data.clone();
perturb(&mut data, i, eps);
data.step(model)?;  // current: forward() + integrate()
let y_plus = measure(&data);
restore(&mut data, &saved);
// Centered: repeat with -eps, column = (y+ - y-) / (2*eps)
// Forward: column = (y+ - y0) / eps
```

After DT-53, transition FD loops replace `step()` with the skip-stage variant:
```rust
data.forward_skip(model, skipstage, true)?;  // skips stages + sensors
data.integrate(model);                        // always runs
```

All new FD functions (DT-51, DT-47 extensions) follow the base pattern.
DT-51 (`mjd_inverse_fd`) uses `forward()` + `inverse()` (no integration,
no skip-stage) since inverse dynamics requires a complete forward pass.

### 5. Export pattern

New public functions are added to the `pub use derivatives::` block in
`lib.rs`. Internal helper functions remain `pub(crate)` or private.

### 6. `MjStage` enum (DT-53)

```rust
/// Pipeline stage for skip-stage forward dispatch.
/// Matches MuJoCo's `mjtStage` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MjStage {
    /// No stage completed — run full pipeline.
    None = 0,
    /// Position stage complete — skip FK, collision.
    Pos = 1,
    /// Velocity stage complete — skip FK, collision, velocity FK.
    Vel = 2,
}
```

The `PartialOrd` derivation enables `if skipstage < MjStage::Pos` guards
matching MuJoCo's integer comparison pattern.

**Location:** Define in `sim/L0/core/src/forward/mod.rs` alongside
`forward_skip()`. Export from `lib.rs`. This enum is pipeline-specific, not
derivative-specific — `derivatives.rs` imports it for use in FD loops.

---

## Cross-Spec Blast Radius

### Behavioral interactions between deliverables

| Interaction | Analysis |
|------------|----------|
| DT-54 (muscle vel derivs) + §58 (position derivs) | **No conflict.** DT-54 adds velocity derivatives for muscle actuators (writes to `qDeriv`). §58 adds position derivatives for smooth forces (writes to `qDeriv_pos` — separate storage, see Contract 1). DT-54 extends `mjd_actuator_vel()`, §58 adds a new `mjd_smooth_pos()`. |
| DT-53 (forwardSkip) + §58 (position derivs) | **Complementary.** DT-53 skips FK/collision for velocity FD columns (using `forward_skip() + integrate()` instead of `step()`). §58 replaces position FD columns with analytical. Together they eliminate most FD cost: position columns are analytical (§58), velocity columns skip FK (DT-53). Independent implementations. |
| DT-53 (forwardSkip) + DT-47 (sensor C/D) | **Compatible.** DT-47 extends the FD perturbation loop. DT-53 optimizes the `forward()` call within that loop. DT-47's sensor measurement happens after `forward()` (or `forward_skip()`), so skip-stage is transparent to sensor collection. |
| §58 (position derivs) + DT-47 (sensor C/D) | **Soft dependency.** Spec B's hybrid path can use §58's analytical position derivatives for sensor C matrix columns. Without §58, FD position columns still capture sensor derivatives correctly — just slower. Spec B must handle both cases. |
| DT-51 (inverseFD) + DT-53 (forwardSkip) | **Independent.** DT-51 uses `Data::forward()` + `Data::inverse()`, not `step()`. No skip-stage optimization applies because `inverse()` requires a complete forward pass. |
| DT-52 (subQuat) + everything else | **Independent.** Pure math utility with no coupling to the derivative pipeline. |

### Existing test impact (cross-deliverable)

| Test area | Touched by | Conflict risk |
|-----------|-----------|--------------|
| `derivatives.rs` integration tests (Steps 0–7) | DT-54 (adds muscle case), §58 (adds position tests) | **Low.** DT-54 adds new tests for muscle velocity derivatives. §58 adds new tests for position derivatives. Existing tests unchanged. |
| `fluid_derivatives.rs` tests (T1–T32) | None | **None.** Fluid velocity derivatives are unchanged by Phase 11. |
| Transition derivative conformance | §58 (analytical matches FD), DT-47 (C/D matrices populated), DT-53 (skip-stage produces identical results) | **Low.** All three must maintain existing A/B matrix values while adding new capabilities. Regression tests verify this. |
| Forward pipeline tests | DT-53 (forward_skip must match forward) | **Low.** `forward_skip(MjStage::None, false)` must produce bit-identical results to `forward()`. Regression test validates this. |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| T1 (DT-52 + DT-54) | 6–10 (subQuat FD validation, identity cases, muscle derivatives FD matching) | +6–10 |
| T1 (DT-51 + DT-53) | 6–10 (inverseFD DfDa ≈ M, forwardSkip == forward regression, FD speedup) | +6–10 |
| Spec A (§58) | 8–12 (position derivatives FD matching per joint type, transition speedup, regression) | +8–12 |
| Spec B (DT-47) | 6–10 (C/D matrix FD validation, per-sensor-type, opt-in flag, regression) | +6–10 |
| **Total** | **26–42** | **+26–42** |

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 11 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge's derivative API matches MuJoCo's `mjd_*` surface
for every function implemented in Phase 11.**

### PH11-AC1: All 6 tasks ship-complete
Every task in the assignment table (§58, DT-47, DT-51, DT-52, DT-53, DT-54)
has landed and its sub-spec ACs (or T1 criteria) are met. Every sub-spec AC
that asserts a numerical value has that value verified against MuJoCo's
actual output.

### PH11-AC2: No regression in existing test suite
All 3,600+ domain tests from the post-Phase 10 baseline pass. Zero test
failures attributable to Phase 11 changes.

### PH11-AC3: Quality gate passes
`cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` passes
(clippy clean, fmt clean, all domain tests green).

### PH11-AC4: Aggregate test growth
Domain test count increases by at least 26 (lower bound of per-deliverable
estimates). At least one MuJoCo conformance test (expected value from running
MuJoCo) per sub-spec.

### PH11-AC5: Derivative API coverage
After Phase 11, the following MuJoCo derivative features are covered:

| Feature | Pre-Phase 11 | Post-Phase 11 |
|---------|-------------|--------------|
| Position derivatives | FD only (in hybrid path) | Analytical `mjd_smooth_pos()` |
| Velocity derivatives | Analytical (non-muscle) | Analytical (all actuator types incl. muscle) |
| Transition A matrix | Hybrid: analytical velocity cols + FD position cols | Fully analytical (both velocity and position cols) |
| Transition C/D matrices | `None` (reserved but not populated) | Populated sensor Jacobians via FD/hybrid |
| Inverse dynamics derivatives | Not implemented | `mjd_inverse_fd()` with DfDq, DfDv, DfDa |
| Quaternion subtraction Jacobians | Not implemented | `mjd_sub_quat()` |
| Skip-stage forward | Not implemented | `forward_skip(skipstage, skipsensor)` |
| Transition derivative performance | ~2× speedup (hybrid vs pure FD) | ~4× speedup (fully analytical + skip-stage) |

### PH11-AC6: Cross-deliverable integrity
- DT-53's `forward_skip(MjStage::None, false)` produces bit-identical
  results to `forward()` on a reference model.
- DT-54's muscle velocity derivatives match FD within `1e-6` relative
  tolerance on a Hill-type muscle model.
- §58's analytical position derivatives match FD within `1e-6` relative
  tolerance on models with all joint types (hinge, slide, ball, free).
- DT-47's C/D matrices match pure FD sensor derivatives within `1e-6`
  relative tolerance.
- DT-51's DfDa approximates the mass matrix M within `1e-4` relative
  tolerance.

---

## Out of Scope

Explicitly excluded from Phase 11. Each exclusion states its conformance
impact.

- **DT-45** (full position-analytical derivatives `dFK/dq`, `dM/dq`) — T3,
  massive complexity. §58 implements the position derivatives needed for
  transition Jacobians. Full FK Jacobian and mass matrix position derivatives
  are a separate research effort. *Conformance impact: none — §58 covers
  the v1.0 surface.*

- **DT-46** (contact-analytical derivatives via implicit function theorem) —
  T3, research-grade. Contact transitions are captured by FD in the
  transition derivative loop. *Conformance impact: none — FD handles
  contact correctly.*

- **DT-50** (automatic differentiation — dual numbers / enzyme) — T3,
  requires scalar type genericity. No changes to the f64-based pipeline.
  *Conformance impact: none — orthogonal approach.*

- **DT-48** (sparse derivative storage) — T2, performance optimization for
  nv > 100. All matrices remain dense in Phase 11. *Conformance impact:
  none — dense matrices are numerically identical.*

- **DT-49** (parallel FD computation) — T2, performance optimization.
  Each perturbation column requires sequential `step()` (or `forward_skip()`).
  Parallelization deferred. *Conformance impact: none — sequential FD is
  numerically identical.*

- **DT-55** (`skipfactor` / factorization reuse) — T1, performance
  optimization for implicit integrators. Reuse Cholesky/LU factors across
  FD perturbation columns. *Conformance impact: none.*

- **Sensor analytical derivatives** — MuJoCo uses FD for sensor C/D
  matrices. Analytical sensor derivatives would require per-sensor-type
  chain rules. Phase 11's FD approach matches MuJoCo exactly.
  *Conformance impact: none.*

- **`mjd_transitionFD` sensor columns in inverseFD** — MuJoCo's
  `mjd_inverseFD` also computes sensor derivatives (DsDq, DsDv, DsDa).
  Phase 11's DT-51 covers force derivatives only. Sensor columns for
  inverse dynamics are a natural follow-up. *Conformance impact: minimal —
  inverse sensor derivatives are rarely used.*
