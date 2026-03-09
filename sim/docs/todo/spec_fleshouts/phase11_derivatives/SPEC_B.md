# Sensor Derivatives (C, D Matrices) — Spec

**Status:** Draft
**Phase:** Roadmap Phase 11 — Derivatives
**Effort:** L
**MuJoCo ref:** `mjd_transitionFD()` / `mjd_stepFD()` in `engine_derivative_fd.c`
**MuJoCo version:** 3.x (current main branch, verified 2026-03-09)
**Test baseline:** 3,600+ sim domain tests
**Prerequisites:**
- Phase A–D derivative infrastructure (landed, `derivatives.rs` ~4,276 lines)
- DT-53 `forward_skip` + `MjStage` enum (landed in `c68d9cb`)
- Spec A `mjd_smooth_pos` + analytical position columns (landed in `276bed5`)

**Independence:** This spec has a soft dependency on Spec A (§58 Analytical
Position Derivatives) per the umbrella dependency graph. Spec A has landed.
Shared file: `derivatives.rs` — Spec B extends the FD perturbation loop for
sensor C/D matrices and adds a field to `DerivativeConfig`. Spec A touched
position column assembly and `mjd_smooth_pos()`. No conflict — Spec B's
changes are additive to the perturbation loop and config struct.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical C and D matrices to MuJoCo's `mjd_transitionFD()`.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

Empirical verification against the MuJoCo C source confirms the umbrella
spec's scope with corrections discovered during rubric authoring (Session 9).

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "C matrix: `∂sensordata_{t+1}/∂x_t`" | Confirmed. `mjd_stepFD()` computes `DsDq` (ns × nv), `DsDv` (ns × nv), `DsDa` (ns × na) which assemble into C (ns × ndx where ndx=2*nv+na). | **In scope.** |
| "D matrix: `∂sensordata_{t+1}/∂u_t`" | Confirmed. `mjd_stepFD()` computes `DsDu` (ns × nu). | **In scope.** |
| "Sensor outputs measured after each perturbation step — black-box FD" | Confirmed. `getState()` copies `d->sensordata` alongside state after each `mj_stepSkip()`. | **In scope.** |
| "Opt-in computation: C/D only populated when explicitly requested" | Confirmed. `mjd_transitionFD()` takes nullable C/D pointers. When both NULL, `mjd_stepFD()` derives `skipsensor=1`. | **In scope.** Map to `DerivativeConfig.compute_sensor_derivatives: bool`. |
| "Hybrid optimization: analytical sensor position derivatives" | **NOT how MuJoCo works.** MuJoCo uses pure FD for ALL sensor derivative columns. No analytical sensor derivative path. | **Drop.** FD-only for C/D matches MuJoCo exactly. |
| "Per-sensor-type output handling" | NOT per-sensor-type. `mjd_stepFD()` treats sensordata as a flat `f64[nsensordata]` vector. FD differences the entire vector. | **Simplify.** No per-sensor-type logic — black-box FD captures all sensor types automatically. |
| "Sensor noise and cutoff interaction" | MuJoCo does NOT add noise during sensor eval. Cutoff clamping IS applied during pipeline — FD captures it naturally (creates zero-derivative regions where clamped). | **In scope.** No special handling needed. |
| "Integration into both mjd_transition_fd() and mjd_transition_hybrid()" | MuJoCo has only `mjd_transitionFD()` (pure FD). CortenForge hybrid path is an extension. For conformance, C/D must be FD-computed in both. | **In scope.** Both paths get C/D via FD. |
| Control clamping in D matrix columns | `mjd_stepFD()` checks `actuator_ctrllimited`/`actuator_ctrlrange`. Uses `clampedDiff` for asymmetric differencing. | **In scope.** Already fixed in Session 9 (EGT-11). Sensor D columns follow same clamping. |

**Final scope:**

1. `DerivativeConfig.compute_sensor_derivatives: bool` — opt-in flag (default false)
2. Extend `mjd_transition_fd()` to record `sensordata` deltas alongside state deltas
3. Extend `mjd_transition_hybrid()` to compute FD sensor C/D columns
4. Populate `TransitionMatrices.C` and `.D` when requested
5. Pass `skipsensor=false` to `forward_skip()` / `step()` when sensor derivatives are being computed
6. Handle `nsensordata == 0` edge case (no sensors → C/D remain None)
7. Validation: C/D matrices match standalone FD sensor derivatives within tolerance

---

## Problem Statement

CortenForge's `TransitionMatrices` struct has `C: Option<DMatrix<f64>>` and
`D: Option<DMatrix<f64>>` fields that are always `None`. MuJoCo's
`mjd_transitionFD()` populates these when the caller provides non-NULL `C`
and `D` pointers — computing `∂sensordata_{t+1}/∂x_t` (C) and
`∂sensordata_{t+1}/∂u_t` (D) via the same FD perturbation loop that
computes A and B.

This is a **conformance gap**: MuJoCo implements sensor derivatives in
`mjd_transitionFD()` (`engine_derivative_fd.c`); CortenForge does not.
Users doing state estimation (Kalman filters, observers) or output-feedback
control need C and D to linearize the full observation model
`y = h(x, u)` alongside the transition model `x⁺ = f(x, u)`.

---

## MuJoCo Reference

> **Source:** `engine_derivative_fd.c` in MuJoCo 3.x main branch.

### `mjd_transitionFD` — thin wrapper around `mjd_stepFD`

**Source:** `engine_derivative_fd.c`, `mjd_transitionFD()`

**Signature:**
```c
void mjd_transitionFD(const mjModel* m, mjData* d, mjtNum eps,
                      mjtByte flg_centered,
                      mjtNum* A, mjtNum* B, mjtNum* C, mjtNum* D);
```

**Algorithm:**
1. Stack-allocates transposed work matrices only when the corresponding
   output pointer is non-NULL:
   - `AT` (ndx × ndx) if `A` non-NULL
   - `BT` (nu × ndx) if `B` non-NULL
   - `CT` (ndx × ns) if `C` non-NULL — note: ndx columns, ns rows
   - `DT` (nu × ns) if `D` non-NULL
2. Splits `AT` into sub-blocks: `DyDq` (nv cols), `DyDv` (nv cols),
   `DyDa` (na cols). Splits `CT` into sub-blocks: `DsDq` (nv cols),
   `DsDv` (nv cols), `DsDa` (na cols).
3. Calls `mjd_stepFD(m, d, eps, flg_centered, DyDq, DyDv, DyDa, BT, DsDq, DsDv, DsDa, DT)`.
4. Transposes results to control-theory convention (row = output, col = input).

**Key insight:** The sensor opt-in flows from pointer nullability. When
`C=NULL` AND `D=NULL`, all `DsD*` pointers passed to `mjd_stepFD` are NULL,
so `mjd_stepFD` derives `skipsensor = !DsDq && !DsDv && !DsDa && !DsDu` = 1.

### `mjd_stepFD` — core FD perturbation loop with sensor recording

**Source:** `engine_derivative_fd.c`, `mjd_stepFD()`

**Signature:**
```c
void mjd_stepFD(const mjModel* m, mjData* d, mjtNum eps, mjtByte flg_centered,
                mjtNum* DyDq, mjtNum* DyDv, mjtNum* DyDa,  // state Jacobians
                mjtNum* DyDu,                               // B matrix transposed
                mjtNum* DsDq, mjtNum* DsDv, mjtNum* DsDa,  // sensor-state Jacobians
                mjtNum* DsDu);                              // sensor-control Jacobian
```

**Algorithm:**

1. **Dimensions:**
   ```c
   int nq = m->nq, nv = m->nv, na = m->na, nu = m->nu;
   int ns = m->nsensordata;
   int ndx = 2*nv + na;
   ```

2. **History guard:**
   ```c
   if (m->nhistory) mjERROR("delays are not supported");
   ```

3. **Sensor opt-in:**
   ```c
   int skipsensor = !DsDq && !DsDv && !DsDa && !DsDu;
   ```
   When all sensor Jacobian pointers are NULL, `skipsensor=1` → sensor
   evaluation is skipped during `mj_stepSkip()`, saving computation.

4. **Save full state:** Saves `time`, `qpos`, `qvel`, `act`, `ctrl`,
   `qacc_warmstart`, `mocap_pos`, `mocap_quat` via `mj_getState()` with
   `mjSTATE_FULLPHYSICS | mjSTATE_WARMSTART`.

5. **Allocate sensor buffers (when sensors requested):**
   ```c
   mjtNum *sensor=NULL, *sensor_plus=NULL, *sensor_minus=NULL;
   if (!skipsensor) {
       sensor       = mj_stackAllocNum(d, ns);
       sensor_plus  = mj_stackAllocNum(d, ns);
       sensor_minus = mj_stackAllocNum(d, ns);
   }
   ```

6. **Nominal step (always):**
   ```c
   mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor);
   // Save nominal next state
   getState(m, d, next, sensor);
   mj_setState(m, d, fullstate, restore_spec);  // restore
   ```

7. **Control columns (D matrix, nu columns):**
   ```c
   for (int i=0; i<nu; i++) {
       int limited = m->actuator_ctrllimited[i];
       // ... control clamping via clampedDiff ...
       // skipstage = mjSTAGE_VEL (position+velocity unchanged)
       mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
       getState(m, d, next_plus, sensor_plus);
       mj_setState(m, d, fullstate, restore_spec);
       // ... centered: repeat with -eps ...
       if (DyDu) clampedStateDiff(m, DyDu+i*ndx, ...);
       if (DsDu) clampedDiff(DsDu+i*ns, sensor, sensor_plus, sensor_minus, eps, ns);
   }
   ```

8. **Activation columns (A matrix, na columns):**
   ```c
   for (int i=0; i<na; i++) {
       // Perturb act[i] ± eps
       // skipstage = mjSTAGE_VEL
       mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
       getState(m, d, next_plus, sensor_plus);
       // ... state + sensor differencing ...
       if (DsDa) clampedDiff(DsDa+i*ns, ...);
   }
   ```

9. **Velocity columns (A matrix, nv columns):**
   ```c
   for (int i=0; i<nv; i++) {
       // Perturb qvel[i] ± eps
       // skipstage = mjSTAGE_POS (position unchanged)
       mj_stepSkip(m, d, mjSTAGE_POS, skipsensor);
       getState(m, d, next_plus, sensor_plus);
       // ... state + sensor differencing ...
       if (DsDv) clampedDiff(DsDv+i*ns, ...);
   }
   ```

10. **Position columns (A matrix, nv columns):**
    ```c
    for (int i=0; i<nv; i++) {
        // Perturb qpos in tangent space ± eps via mj_integratePos
        // skipstage = mjSTAGE_NONE (full pipeline needed)
        mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor);
        getState(m, d, next_plus, sensor_plus);
        // ... state + sensor differencing ...
        if (DsDq) clampedDiff(DsDq+i*ns, ...);
    }
    ```

### `getState` — captures state + sensor outputs

**Source:** `engine_derivative_fd.c`, `getState()`

```c
static void getState(const mjModel* m, const mjData* d,
                     mjtNum* state, mjtNum* sensor) {
    mj_getState(m, d, state, mjSTATE_PHYSICS);
    if (sensor) mju_copy(sensor, d->sensordata, m->nsensordata);
}
```

When `sensor` is non-NULL (i.e., sensors are being tracked), copies the
entire `sensordata` flat vector after each perturbation step.

### `mj_stepSkip` — forward + integrate with skip-stage + sensor control

**Source:** `engine_derivative_fd.c`, `mj_stepSkip()`

```c
void mj_stepSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
    mj_checkPos(m, d);
    mj_checkVel(m, d);
    mj_forwardSkip(m, d, skipstage, skipsensor);
    mj_checkAcc(m, d);
    if (mjENABLED(mjENBL_FWDINV)) mj_compareFwdInv(m, d);
    // Integrator dispatch (skipstage-aware):
    switch (m->opt.integrator) {
    case mjINT_EULER:    mj_EulerSkip(m, d, skipstage >= mjSTAGE_POS); break;
    case mjINT_RK4:      mj_RungeKutta(m, d, 4); break;
    case mjINT_IMPLICIT:
    case mjINT_IMPLICITFAST:
                         mj_implicitSkip(m, d, skipstage >= mjSTAGE_VEL); break;
    }
}
```

**Critical:** `mj_stepSkip` is `forwardSkip + checkAcc + integrator`. In
CortenForge, the equivalent is `forward_skip(skipstage, skipsensor) +
integrate()`. The existing `step()` includes additional check/sleep/warmstart
overhead that the FD loop doesn't need.

### `clampedDiff` — differencing with clamped boundaries

**Source:** `engine_derivative_fd.c`, `clampedDiff()`

```c
static void clampedDiff(mjtNum* dest, const mjtNum* center,
                        const mjtNum* plus, const mjtNum* minus,
                        mjtNum eps, int n) {
    // Uses forward, backward, or centered depending on which perturbations
    // were feasible (control clamping). plus=NULL means forward-nudge was
    // infeasible, minus=NULL means backward-nudge was infeasible.
}
```

Applied identically to both state differencing (`clampedStateDiff`) and
sensor differencing (`clampedDiff`). The CortenForge control clamping
infrastructure (fixed in Session 9) already handles this for state columns;
sensor columns follow the same pattern.

### Skip-stage assignments per perturbation type

| Perturbation type | MuJoCo `skipstage` | Rationale |
|-------------------|--------------------|-----------|
| Control (`ctrl`) | `mjSTAGE_VEL` | Position+velocity unchanged by control perturbation |
| Activation (`act`) | `mjSTAGE_VEL` | Position+velocity unchanged by activation perturbation |
| Velocity (`qvel`) | `mjSTAGE_POS` | Position unchanged by velocity perturbation |
| Position (`qpos`) | `mjSTAGE_NONE` | Full pipeline needed — position change affects everything |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Sensor derivative computation | `mjd_transitionFD()` computes C/D via `mjd_stepFD()` when C/D pointers non-NULL | C/D always `None` — not computed |
| Sensor opt-in mechanism | Nullable C/D pointer → `skipsensor` flag in `mj_stepSkip()` | No opt-in mechanism. `step()` always evaluates sensors. |
| FD sensor recording | `getState()` copies `d->sensordata` after each perturbed step | Not implemented — `extract_state()` captures only state, not sensors |
| Sensor skip during FD (no C/D) | `skipsensor=1` → sensors skipped → faster FD | Not implemented — `step()` always evaluates sensors regardless of whether C/D are needed |
| Control-clamped D columns | `clampedDiff` for sensor columns (same clamping as state columns) | State columns fixed (Session 9). Sensor columns not implemented. |
| `nsensordata == 0` | Allocates 0-length buffers, no-op sensor differencing. C/D are 0×ndx and 0×nu. | C/D = None. |
| `nhistory != 0` | `mjd_stepFD` errors: "delays are not supported" | Guard already added (Session 9). |
| Sensor noise | Not applied during pipeline. FD derivatives are noiseless. | Same — no noise in pipeline. |
| Sensor cutoff | Applied via `mj_sensor_postprocess`. Creates zero-derivative regions. | Same — cutoff applied in pipeline. |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| C/D opt-in | Nullable output pointers (`C=NULL` → skip) | `DerivativeConfig.compute_sensor_derivatives: bool` | Use `config.compute_sensor_derivatives` wherever MuJoCo checks pointer nullability |
| `mj_stepSkip()` | `forwardSkip + checkAcc + integrator` (single C function) | `forward_skip(skipstage, skipsensor) + integrate()` (two Rust calls) | For **sensor-only FD passes** (S3): use `scratch.forward_skip(model, skipstage, false)?; scratch.integrate(model)`. For **piggybacked FD columns** (S2, S3 existing FD): keep `scratch.step(model)?` (evaluates sensors by default — `skipsensor` optimization deferred, see Out of Scope). Both produce identical numerical results. |
| Matrix layout | Column-major in `mjd_stepFD`, transposed to row-major by `mjd_transitionFD` | nalgebra column-major. `column_mut(i).copy_from(&col)` writes columns directly — no manual transpose | Direct port — nalgebra column writes match `mjd_stepFD`'s column layout. No transpose step needed (nalgebra handles it). |
| `getState()` sensor capture | `mju_copy(sensor, d->sensordata, m->nsensordata)` | `scratch.sensordata.clone()` (DVector clone) | Use `scratch.sensordata.clone()` wherever MuJoCo uses `getState(,,,sensor)` |
| `extract_state()` | `mj_getState(m, d, state, mjSTATE_PHYSICS)` returns `[qpos, qvel, act]` in coordinate space. `clampedStateDiff` does tangent-space subtraction. | `extract_state(model, data, qpos_ref)` returns tangent-space state directly. | Direct port — `extract_state()` already handles tangent-space convention. Sensor extraction is independent (flat `f64` copy). |
| Sensor dimensions | `ns = m->nsensordata` | `model.nsensordata` | Direct port — no translation needed |
| `skipsensor` polarity | `skipsensor=1` means SKIP sensors | `skipsensor: bool` where `true` means SKIP | Direct port — same polarity. Pass `!config.compute_sensor_derivatives` for sensor control. |

---

## Architecture Decisions

### AD-1: FD loop integration strategy for hybrid path

**Problem:** The hybrid path (`mjd_transition_hybrid`) computes some A/B
columns analytically (velocity, simple B) and some via FD (position, complex
B, muscle activations). For sensor C/D, ALL columns must use FD (no
analytical sensor path). How should sensor FD columns be handled for the
analytically-computed A/B columns?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | **Integrated:** Piggyback sensor recording on existing FD steps. For analytical columns, add sensor-only FD passes. | Efficient — no redundant computation for existing FD columns. Matches MuJoCo's single-loop design. | Complex — must track which columns are FD vs analytical and add sensor-only passes for analytical ones. |
| 2 | **Separate:** After hybrid A/B, run a separate full FD pass for C/D only. | Simple — clean separation. | ~2× sensor FD cost (re-does FD steps for columns already computed). |

**Chosen:** Option 1 — Integrated. This matches MuJoCo's approach where a
single FD loop computes both state and sensor columns simultaneously. For
the hybrid path: FD columns (position, complex B, muscle activation) already
run perturbed steps — sensor recording piggybacks on these. Analytical columns
(velocity, simple B) need new sensor-only FD passes but can use skip-stage
optimization (`mjSTAGE_VEL` for velocity columns). The cost is additional
complexity in the hybrid function but eliminates redundant `step()` calls.

### AD-2: `nsensordata == 0` behavior

**Problem:** When the model has no sensors (`nsensordata == 0`) but the caller
requests `compute_sensor_derivatives = true`, what should C/D be?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Return `None` | Simple. "No sensors = nothing to compute." | Breaks the invariant "Some = was computed, None = not requested". Caller can't distinguish "not requested" from "no sensors". |
| 2 | Return `Some(DMatrix::zeros(0, ndx))` / `Some(DMatrix::zeros(0, nu))` | Matches MuJoCo (returns non-null 0-row matrices). Preserves Some/None semantics. | Slightly unusual — a 0-row matrix. |

**Chosen:** Option 2 — Return empty `Some` matrices. This matches MuJoCo's
behavior (non-null 0-row matrices) and preserves the Some/None API semantics:
`Some` = computation was requested and performed, `None` = not requested.

---

## Specification

### S1. DerivativeConfig extension

**File:** `sim/L0/core/src/derivatives.rs` (lines 142–173)
**MuJoCo equivalent:** Nullable `C`/`D` pointers in `mjd_transitionFD()` signature
**Design decision:** Use a bool field with `false` default for backward
compatibility. Matches the umbrella Contract 2. All existing callers using
`DerivativeConfig::default()` are unaffected. Struct-literal callers (13 of
28) need `compute_sensor_derivatives: false` added.

**After:**
```rust
pub struct DerivativeConfig {
    /// Perturbation magnitude for finite differences.
    /// Default: `1e-6` (centered differences).
    /// Must be in `(0, 1e-2]`. Typical range: `1e-8` to `1e-4`.
    pub eps: f64,

    /// Use centered differences (more accurate, 2x cost) vs forward differences.
    /// Centered: O(ε²) error. Forward: O(ε) error.
    /// Default: `true`.
    pub centered: bool,

    /// Use hybrid analytical+FD method (Phase D) when available.
    /// Default: `true`.
    pub use_analytical: bool,

    /// When true, compute sensor derivatives (C, D matrices) alongside
    /// state transition derivatives (A, B). Sensor outputs are captured
    /// via finite differencing of `sensordata` during the same perturbation
    /// loop that computes A/B.
    ///
    /// When false (default), C and D remain `None` in `TransitionMatrices`
    /// and sensor evaluation is skipped during FD perturbation steps,
    /// reducing computation cost.
    ///
    /// MuJoCo equivalent: passing non-NULL `C`/`D` pointers to
    /// `mjd_transitionFD()`.
    pub compute_sensor_derivatives: bool,
}

impl Default for DerivativeConfig {
    fn default() -> Self {
        Self {
            eps: 1e-6,
            centered: true,
            use_analytical: true,
            compute_sensor_derivatives: false,
        }
    }
}
```

**Migration for struct-literal callers:** Add `compute_sensor_derivatives: false`
to the 13 call sites that use struct literal syntax without `..Default::default()`.

| File | Call sites to update | Change |
|------|---------------------|--------|
| `derivatives.rs` | `fd_convergence_check` (2 sites) | Add `compute_sensor_derivatives: false` |
| `integration/derivatives.rs` | 7 early acceptance tests | Add `compute_sensor_derivatives: false` |
| `integration/implicit_integration.rs` | 4 sites | Add `compute_sensor_derivatives: false` |

### S2. FD sensor derivative loop in `mjd_transition_fd`

**File:** `sim/L0/core/src/derivatives.rs` (lines 227–378)
**MuJoCo equivalent:** `mjd_stepFD()` in `engine_derivative_fd.c`
**Design decision:** Extend the existing perturbation loop to capture
sensordata alongside state. This avoids a separate FD pass. The sensor
recording piggybacks on every `step()` call already being made, adding only
a `sensordata.clone()` per perturbation step. The loop already uses `step()`
which evaluates sensors — we just need to capture the output.

**Algorithm (changes to existing `mjd_transition_fd`):**

```rust
pub fn mjd_transition_fd(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError> {
    // ... existing validation ...

    let ns = model.nsensordata;
    let compute_sensors = config.compute_sensor_derivatives && ns > 0;

    // Phase 0 — Save nominal state and clone scratch.
    let mut scratch = data.clone();
    // ... existing qpos_0, qvel_0, etc. saves ...

    // Nominal step — capture baseline sensordata
    scratch.step(model)?;  // step() always evaluates sensors
    let y_0 = extract_state(model, &scratch, &qpos_0);
    let sensor_0 = if compute_sensors {
        Some(scratch.sensordata.clone())
    } else {
        None
    };
    // ... existing restore ...

    let mut A = DMatrix::zeros(nx, nx);
    let mut B = DMatrix::zeros(nx, nu);
    let mut C = if compute_sensors {
        Some(DMatrix::zeros(ns, nx))
    } else {
        None
    };
    let mut D = if compute_sensors {
        Some(DMatrix::zeros(ns, nu))
    } else {
        None
    };

    // Phase 1 — State perturbation (A matrix + C sensor-state columns)
    for i in 0..nx {
        apply_state_perturbation(model, &mut scratch, ...i, eps...);
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);
        let s_plus = if compute_sensors {
            Some(scratch.sensordata.clone())
        } else {
            None
        };

        if config.centered {
            apply_state_perturbation(model, &mut scratch, ...i, -eps...);
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);
            let s_minus = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            // State column (existing)
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            A.column_mut(i).copy_from(&col);

            // Sensor column (NEW)
            if let (Some(c_mat), Some(sp), Some(sm)) = (&mut C, &s_plus, &s_minus) {
                let scol = (sp - sm) / (2.0 * eps);
                c_mat.column_mut(i).copy_from(&scol);
            }
        } else {
            // Forward difference
            let col = (&y_plus - &y_0) / eps;
            A.column_mut(i).copy_from(&col);

            if let (Some(c_mat), Some(sp), Some(s0)) = (&mut C, &s_plus, &sensor_0) {
                let scol = (sp - s0) / eps;
                c_mat.column_mut(i).copy_from(&scol);
            }
        }
    }

    // Phase 2 — Control perturbation (B matrix + D sensor-control columns)
    // Same clamped-differencing pattern as existing B matrix loop.
    for j in 0..nu {
        let range = model.actuator_ctrlrange[j];
        let nudge_fwd = in_ctrl_range(ctrl_0[j], ctrl_0[j] + eps, range);
        let nudge_back = (config.centered || !nudge_fwd)
            && in_ctrl_range(ctrl_0[j] - eps, ctrl_0[j], range);

        let (y_plus, s_plus) = if nudge_fwd {
            // ... restore + perturb ctrl[j] += eps + step() ...
            scratch.step(model)?;
            let yp = Some(extract_state(model, &scratch, &qpos_0));
            let sp = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };
            (yp, sp)
        } else {
            (None, None)
        };

        let (y_minus, s_minus) = if nudge_back {
            // ... restore + perturb ctrl[j] -= eps + step() ...
            scratch.step(model)?;
            let ym = Some(extract_state(model, &scratch, &qpos_0));
            let sm = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };
            (ym, sm)
        } else {
            (None, None)
        };

        // State column (existing clamped differencing)
        let col = match (&y_plus, &y_minus) {
            (Some(yp), Some(ym)) => (yp - ym) / (2.0 * eps),
            (Some(yp), None) => (yp - &y_0) / eps,
            (None, Some(ym)) => (&y_0 - ym) / eps,
            (None, None) => DVector::zeros(nx),
        };
        B.column_mut(j).copy_from(&col);

        // Sensor column (NEW — same clamped differencing)
        if let Some(d_mat) = &mut D {
            let s0 = sensor_0.as_ref().unwrap();
            let scol = match (&s_plus, &s_minus) {
                (Some(sp), Some(sm)) => (sp - sm) / (2.0 * eps),
                (Some(sp), None) => (sp - s0) / eps,
                (None, Some(sm)) => (s0 - sm) / eps,
                (None, None) => DVector::zeros(ns),
            };
            d_mat.column_mut(j).copy_from(&scol);
        }
    }

    Ok(TransitionMatrices { A, B, C, D })
}
```

**Key points:**
- `step()` always evaluates sensors. The only change is capturing
  `scratch.sensordata.clone()` after each step when `compute_sensors` is true.
- Clamped control differencing for D columns follows the exact same
  `in_ctrl_range` / forward-backward-centered pattern as B columns.
- No new `step()` calls are added — sensor recording piggybacks on existing FD steps.
- When `compute_sensors` is false, zero overhead (no clone, no allocation).

### S3. FD sensor derivative loop in `mjd_transition_hybrid`

**File:** `sim/L0/core/src/derivatives.rs` (lines 2405–2934)
**MuJoCo equivalent:** No direct equivalent — MuJoCo has no hybrid path.
**Design decision:** The hybrid path computes A/B columns using a mix of
analytical and FD methods. For sensor C/D, ALL columns use FD. Three
categories of columns in the hybrid path:

| Column type | Current method | Has FD step? | Sensor handling |
|------------|---------------|-------------|-----------------|
| Velocity (A cols nv..2*nv) | Analytical | No | NEW: FD sensor-only pass with `skipstage=MjStage::Pos` |
| Position (A cols 0..nv) | FD (or analytical via Spec A) | Sometimes | Piggyback on existing FD step; or NEW FD sensor-only pass for analytical position columns |
| Activation (A cols 2*nv..2*nv+na) | Mixed (analytical for simple, FD for muscle) | Sometimes | Piggyback on FD steps; NEW FD sensor-only pass for analytical columns |
| Simple B (ctrl) | Analytical | No | NEW: FD sensor-only pass with `skipstage=MjStage::Vel` |
| Complex B (ctrl FD) | FD | Yes | Piggyback on existing FD step |

**Algorithm (changes to `mjd_transition_hybrid`):**

The changes are additive — all existing A/B computation remains unchanged.

```rust
pub fn mjd_transition_hybrid(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError> {
    // ... existing validation and setup ...
    let ns = model.nsensordata;
    let compute_sensors = config.compute_sensor_derivatives && ns > 0;

    // Existing: analytical velocity/activation/B columns for A/B ...
    // (unchanged — produces a_mat and b_mat as before)

    // ... existing nominal step + FD setup ...
    // Capture baseline sensordata alongside nominal y_0:
    let sensor_0 = if compute_sensors {
        Some(scratch.sensordata.clone())
    } else {
        None
    };

    let mut c_mat = if compute_sensors {
        Some(DMatrix::zeros(ns, nx))
    } else {
        None
    };
    let mut d_mat = if compute_sensors {
        Some(DMatrix::zeros(ns, nu))
    } else {
        None
    };

    // === Position columns of A (existing) ===
    if use_analytical_pos {
        // Analytical A position columns (unchanged)
        // ... mjd_smooth_pos, chain rule, fill a_mat ...

        // Sensor C position columns — need FD since analytical doesn't
        // capture sensor outputs.
        if compute_sensors {
            for i in 0..nv {
                // +eps perturbation
                apply_state_perturbation(model, &mut scratch, ..., i, eps, ...);
                scratch.forward_skip(model, MjStage::None, false)?;
                scratch.integrate(model);
                let s_plus = scratch.sensordata.clone();

                if config.centered {
                    // -eps perturbation
                    apply_state_perturbation(model, &mut scratch, ..., i, -eps, ...);
                    scratch.forward_skip(model, MjStage::None, false)?;
                    scratch.integrate(model);
                    let s_minus = scratch.sensordata.clone();

                    let scol = (&s_plus - &s_minus) / (2.0 * eps);
                    c_mat.as_mut().unwrap().column_mut(i).copy_from(&scol);
                } else {
                    let scol = (&s_plus - sensor_0.as_ref().unwrap()) / eps;
                    c_mat.as_mut().unwrap().column_mut(i).copy_from(&scol);
                }
            }
        }
    } else {
        // FD position columns (existing) — piggyback sensor recording
        for i in 0..nv {
            // ... existing: apply_state_perturbation + step + extract_state ...
            // ADD: capture sensordata after each step
            let s_plus = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            if config.centered {
                // ... existing: -eps perturbation + step + extract_state ...
                let s_minus = if compute_sensors {
                    Some(scratch.sensordata.clone())
                } else {
                    None
                };
                // ... existing A column ...
                if let (Some(c), Some(sp), Some(sm)) = (&mut c_mat, &s_plus, &s_minus) {
                    let scol = (sp - sm) / (2.0 * eps);
                    c.column_mut(i).copy_from(&scol);
                }
            } else {
                // ... existing A column ...
                if let (Some(c), Some(sp), Some(s0)) = (&mut c_mat, &s_plus, &sensor_0) {
                    let scol = (sp - s0) / eps;
                    c.column_mut(i).copy_from(&scol);
                }
            }
        }
    }

    // === Velocity columns: sensor-only FD ===
    // A velocity columns are analytical (no FD step). Sensor C velocity
    // columns need FD passes with skipstage=MjStage::Pos.
    if compute_sensors {
        for i in 0..nv {
            let state_col = nv + i;  // velocity block starts at nv
            apply_state_perturbation(model, &mut scratch, ..., state_col, eps, ...);
            scratch.forward_skip(model, MjStage::Pos, false)?;
            scratch.integrate(model);
            let s_plus = scratch.sensordata.clone();

            if config.centered {
                apply_state_perturbation(model, &mut scratch, ..., state_col, -eps, ...);
                scratch.forward_skip(model, MjStage::Pos, false)?;
                scratch.integrate(model);
                let s_minus = scratch.sensordata.clone();

                let scol = (&s_plus - &s_minus) / (2.0 * eps);
                c_mat.as_mut().unwrap().column_mut(state_col).copy_from(&scol);
            } else {
                let scol = (&s_plus - sensor_0.as_ref().unwrap()) / eps;
                c_mat.as_mut().unwrap().column_mut(state_col).copy_from(&scol);
            }
        }
    }

    // === Activation columns ===
    // Analytical activation columns: need sensor-only FD.
    // FD activation columns (muscle): piggyback sensor recording.
    // For analytical activation columns, add sensor-only FD passes
    // with skipstage=MjStage::Vel.
    if compute_sensors {
        // Sensor passes for analytically-computed activation columns
        for actuator_idx in 0..model.nu {
            let act_adr = model.actuator_act_adr[actuator_idx];
            let act_num = model.actuator_act_num[actuator_idx];
            if act_num == 0 { continue; }

            let is_muscle = matches!(
                model.actuator_dyntype[actuator_idx],
                ActuatorDynamics::Muscle | ActuatorDynamics::HillMuscle
            );

            for k in 0..act_num {
                let j = act_adr + k;
                let state_col = 2 * nv + j;

                if is_muscle {
                    // Muscle activation columns already use FD (piggyback below)
                    continue;
                }
                // Other non-analytical cases (User gain) already use FD
                let is_analytical = !is_muscle
                    && matches!(
                        model.actuator_gaintype[actuator_idx],
                        GainType::Fixed | GainType::Affine
                    )
                    && matches!(
                        model.actuator_dyntype[actuator_idx],
                        ActuatorDynamics::Integrator
                            | ActuatorDynamics::Filter
                            | ActuatorDynamics::FilterExact
                    );

                if !is_analytical {
                    continue; // FD column — sensor recording handled in FD fallback
                }

                // Sensor-only FD for analytical activation column
                apply_state_perturbation(model, &mut scratch, ..., state_col, eps, ...);
                scratch.forward_skip(model, MjStage::Vel, false)?;
                scratch.integrate(model);
                let s_plus = scratch.sensordata.clone();

                if config.centered {
                    apply_state_perturbation(model, &mut scratch, ..., state_col, -eps, ...);
                    scratch.forward_skip(model, MjStage::Vel, false)?;
                    scratch.integrate(model);
                    let s_minus = scratch.sensordata.clone();

                    let scol = (&s_plus - &s_minus) / (2.0 * eps);
                    c_mat.as_mut().unwrap().column_mut(state_col).copy_from(&scol);
                } else {
                    let scol = (&s_plus - sensor_0.as_ref().unwrap()) / eps;
                    c_mat.as_mut().unwrap().column_mut(state_col).copy_from(&scol);
                }
            }
        }
    }

    // Muscle activation FD fallback columns (existing) — piggyback sensor recording
    for &state_col in &act_fd_indices {
        apply_state_perturbation(model, &mut scratch, ..., state_col, eps, ...);
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);
        let s_plus = if compute_sensors {
            Some(scratch.sensordata.clone())
        } else {
            None
        };

        if config.centered {
            apply_state_perturbation(model, &mut scratch, ..., state_col, -eps, ...);
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);
            let s_minus = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            // Existing A column
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            a_mat.column_mut(state_col).copy_from(&col);

            // Sensor C column (piggybacked)
            if let (Some(c), Some(sp), Some(sm)) = (&mut c_mat, &s_plus, &s_minus) {
                let scol = (sp - sm) / (2.0 * eps);
                c.column_mut(state_col).copy_from(&scol);
            }
        } else {
            let col = (&y_plus - &y_0) / eps;
            a_mat.column_mut(state_col).copy_from(&col);

            if let (Some(c), Some(sp), Some(s0)) = (&mut c_mat, &s_plus, &sensor_0) {
                let scol = (sp - s0) / eps;
                c.column_mut(state_col).copy_from(&scol);
            }
        }
    }

    // === B matrix ===
    // Simple B (analytical): sensor-only FD with skipstage=MjStage::Vel
    // Complex B (FD): piggyback sensor recording
    if compute_sensors {
        // Sensor-only FD for analytical B columns
        for actuator_idx in 0..nu {
            // Skip if this actuator is already in ctrl_fd_indices
            if ctrl_fd_indices.contains(&actuator_idx) {
                continue; // Handled in FD fallback below
            }

            // This is an analytical B column — need sensor-only FD
            let range = model.actuator_ctrlrange[actuator_idx];
            let nudge_fwd = in_ctrl_range(ctrl_0[actuator_idx], ctrl_0[actuator_idx] + eps, range);
            let nudge_back = (config.centered || !nudge_fwd)
                && in_ctrl_range(ctrl_0[actuator_idx] - eps, ctrl_0[actuator_idx], range);

            let s_plus = if nudge_fwd {
                // Restore + perturb ctrl + forward_skip(Vel, false) + integrate
                scratch.qpos.copy_from(&qpos_0);
                scratch.qvel.copy_from(&qvel_0);
                scratch.act.copy_from(&act_0);
                scratch.ctrl.copy_from(&ctrl_0);
                scratch.ctrl[actuator_idx] += eps;
                scratch.qacc_warmstart.copy_from(&warmstart_0);
                scratch.time = time_0;
                scratch.forward_skip(model, MjStage::Vel, false)?;
                scratch.integrate(model);
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            let s_minus = if nudge_back {
                // Restore + perturb ctrl - eps + forward_skip(Vel, false) + integrate
                scratch.qpos.copy_from(&qpos_0);
                scratch.qvel.copy_from(&qvel_0);
                scratch.act.copy_from(&act_0);
                scratch.ctrl.copy_from(&ctrl_0);
                scratch.ctrl[actuator_idx] -= eps;
                scratch.qacc_warmstart.copy_from(&warmstart_0);
                scratch.time = time_0;
                scratch.forward_skip(model, MjStage::Vel, false)?;
                scratch.integrate(model);
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            let s0 = sensor_0.as_ref().unwrap();
            let scol = match (&s_plus, &s_minus) {
                (Some(sp), Some(sm)) => (sp - sm) / (2.0 * eps),
                (Some(sp), None) => (sp - s0) / eps,
                (None, Some(sm)) => (s0 - sm) / eps,
                (None, None) => DVector::zeros(ns),
            };
            d_mat.as_mut().unwrap().column_mut(actuator_idx).copy_from(&scol);
        }
    }

    // FD for non-analytical B columns (existing) — piggyback sensor recording
    for &j in &ctrl_fd_indices {
        let range = model.actuator_ctrlrange[j];
        let nudge_fwd = in_ctrl_range(ctrl_0[j], ctrl_0[j] + eps, range);
        let nudge_back = (config.centered || !nudge_fwd)
            && in_ctrl_range(ctrl_0[j] - eps, ctrl_0[j], range);

        let (y_plus, s_plus) = if nudge_fwd {
            scratch.qpos.copy_from(&qpos_0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.act.copy_from(&act_0);
            scratch.ctrl.copy_from(&ctrl_0);
            scratch.ctrl[j] += eps;
            scratch.qacc_warmstart.copy_from(&warmstart_0);
            scratch.time = time_0;
            scratch.step(model)?;
            let yp = Some(extract_state(model, &scratch, &qpos_0));
            let sp = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };
            (yp, sp)
        } else {
            (None, None)
        };

        let (y_minus, s_minus) = if nudge_back {
            scratch.qpos.copy_from(&qpos_0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.act.copy_from(&act_0);
            scratch.ctrl.copy_from(&ctrl_0);
            scratch.ctrl[j] -= eps;
            scratch.qacc_warmstart.copy_from(&warmstart_0);
            scratch.time = time_0;
            scratch.step(model)?;
            let ym = Some(extract_state(model, &scratch, &qpos_0));
            let sm = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };
            (ym, sm)
        } else {
            (None, None)
        };

        // State B column (existing clamped differencing)
        let col = match (&y_plus, &y_minus) {
            (Some(yp), Some(ym)) => (yp - ym) / (2.0 * eps),
            (Some(yp), None) => (yp - &y_0) / eps,
            (None, Some(ym)) => (&y_0 - ym) / eps,
            (None, None) => DVector::zeros(nx),
        };
        b_mat.column_mut(j).copy_from(&col);

        // Sensor D column (piggybacked — same clamped differencing)
        if let Some(d) = &mut d_mat {
            let s0 = sensor_0.as_ref().unwrap();
            let scol = match (&s_plus, &s_minus) {
                (Some(sp), Some(sm)) => (sp - sm) / (2.0 * eps),
                (Some(sp), None) => (sp - s0) / eps,
                (None, Some(sm)) => (s0 - sm) / eps,
                (None, None) => DVector::zeros(ns),
            };
            d.column_mut(j).copy_from(&scol);
        }
    }

    Ok(TransitionMatrices {
        A: a_mat,
        B: b_mat,
        C: c_mat,
        D: d_mat,
    })
}
```

**Key design properties:**
1. **No A/B numerical coupling:** Enabling sensor derivatives does NOT change
   A/B values. The same perturbation, same step — just also reading sensordata.
   For piggybacked FD columns, the step is identical. For sensor-only FD
   columns (analytical A/B columns), new steps are added but they only affect
   C/D — A/B columns remain analytical.
2. **Skip-stage optimization:** Sensor-only FD passes use the appropriate
   skip-stage: `MjStage::Pos` for velocity perturbations, `MjStage::Vel` for
   activation/control perturbations, `MjStage::None` for position perturbations.
3. **`forward_skip + integrate` pattern:** Sensor-only FD passes use
   `forward_skip(skipstage, false) + integrate()` instead of `step()`. The
   `skipsensor=false` ensures sensors are evaluated. The skip-stage reduces
   per-column cost.
4. **`step()` vs `forward_skip + integrate` safety in piggybacked columns:**
   Piggybacked FD columns use `step()` (which includes check/sleep/warmstart
   overhead) while sensor-only columns use `forward_skip + integrate`. This
   semantic difference is harmless: `warmstart_0` is restored before every
   perturbation via `apply_state_perturbation()` or explicit copy, so
   `step()`'s warmstart-save at the end of each call is overwritten before
   the next perturbation. The numerical results are identical.

### S4. TransitionMatrices wiring

**File:** `sim/L0/core/src/derivatives.rs` (lines 106–128)
**MuJoCo equivalent:** C/D output in `mjd_transitionFD()`
**Design decision:** Update docstrings to reflect that C/D are now populated
when `compute_sensor_derivatives` is true. The `Option` wrapper remains —
`None` means "not requested," `Some` means "computed."

**After:**
```rust
pub struct TransitionMatrices {
    /// State transition matrix `∂x_{t+1}/∂x_t`.
    /// Dimensions: `(2*nv + na) × (2*nv + na)`.
    pub A: DMatrix<f64>,

    /// Control influence matrix `∂x_{t+1}/∂u_t`.
    /// Dimensions: `(2*nv + na) × nu`.
    pub B: DMatrix<f64>,

    /// Sensor-state Jacobian `∂sensordata_{t+1}/∂x_t`.
    /// Dimensions: `nsensordata × (2*nv + na)`.
    /// `None` when `DerivativeConfig.compute_sensor_derivatives` is false (default).
    /// `Some(matrix)` when sensor derivatives are computed — even if
    /// `nsensordata == 0` (in which case the matrix has 0 rows).
    pub C: Option<DMatrix<f64>>,

    /// Sensor-control Jacobian `∂sensordata_{t+1}/∂u_t`.
    /// Dimensions: `nsensordata × nu`.
    /// `None` when `DerivativeConfig.compute_sensor_derivatives` is false (default).
    /// `Some(matrix)` when sensor derivatives are computed.
    pub D: Option<DMatrix<f64>>,
}
```

### S5. `mjd_transition` dispatch and `nsensordata == 0` handling

**File:** `sim/L0/core/src/derivatives.rs` (lines 2949–2961)
**MuJoCo equivalent:** `mjd_transitionFD()` edge cases
**Design decision:** The `mjd_transition` dispatch function passes `config`
through unchanged. Both `mjd_transition_fd` and `mjd_transition_hybrid` handle
`nsensordata == 0` identically: when `compute_sensor_derivatives` is true but
`nsensordata == 0`, set `compute_sensors = false` internally (triggering the
`C: None, D: None` early exit — or alternatively, set `C/D = Some(0-row matrix)`).

Per AD-2, when `nsensordata == 0` and `compute_sensor_derivatives == true`:
- Return `C: Some(DMatrix::zeros(0, nx))`, `D: Some(DMatrix::zeros(0, nu))`

This is handled by the `ns > 0` check in `compute_sensors`:
```rust
let compute_sensors = config.compute_sensor_derivatives && ns > 0;
```

When `ns == 0`, `compute_sensors = false`, so no sensor recording occurs.
But we need to return `Some(empty)` not `None`. So after the FD loop:

```rust
let (c_result, d_result) = if config.compute_sensor_derivatives {
    if ns > 0 {
        // C/D were populated by the FD loop
        (c_mat, d_mat)
    } else {
        // nsensordata == 0: return empty matrices
        (Some(DMatrix::zeros(0, nx)), Some(DMatrix::zeros(0, nu)))
    }
} else {
    (None, None)
};

Ok(TransitionMatrices { A, B, C: c_result, D: d_result })
```

---

## Acceptance Criteria

### AC1: C matrix dimensions *(runtime test — analytically derived)*
**Given:** 2-link pendulum with `jointpos` sensors on both joints (nsensordata=2, nv=2, na=0)
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`
**Assert:** C is `Some` with dimensions 2 × 4 (nsensordata × (2*nv + na))
**Field:** `TransitionMatrices.C`

### AC2: D matrix dimensions *(runtime test — analytically derived)*
**Given:** 2-link pendulum with motor actuator and `jointpos` sensor (nsensordata=1, nv=2, nu=1)
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`
**Assert:** D is `Some` with dimensions 1 × 1 (nsensordata × nu)
**Field:** `TransitionMatrices.D`

### AC3: Opt-in negative case *(runtime test — analytically derived)*
**Given:** Any model with sensors
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: false` (default)
**Assert:** C is `None` and D is `None`
**Field:** `TransitionMatrices.C`, `TransitionMatrices.D`

### AC4: FD C matrix accuracy *(runtime test — analytically derived)*
**Given:** 2-link pendulum with `jointpos` sensors, non-zero initial state
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`, centered
**Assert:** C matrix from `mjd_transition_fd` matches standalone per-column sensor FD (independent implementation) within `1e-6` relative tolerance
**Field:** `TransitionMatrices.C`

### AC5: FD D matrix accuracy *(runtime test — analytically derived)*
**Given:** 2-link pendulum with motor actuator and `jointpos` sensor, non-zero ctrl
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`, centered
**Assert:** D matrix from `mjd_transition_fd` matches standalone per-column sensor FD within `1e-6` relative tolerance
**Field:** `TransitionMatrices.D`

### AC6: Hybrid C/D match pure FD C/D *(runtime test — analytically derived)*
**Given:** 2-link pendulum with `jointpos` sensors and motor actuator
**After:** `mjd_transition_hybrid` with `compute_sensor_derivatives: true` AND `mjd_transition_fd` with same config
**Assert:** Hybrid C matches FD C within `1e-6` relative tolerance. Hybrid D matches FD D within `1e-6` relative tolerance.
**Field:** `TransitionMatrices.C`, `TransitionMatrices.D`

### AC7: A/B unchanged when C/D enabled *(runtime test — analytically derived)*
**Given:** 2-link pendulum with sensors
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: false` AND with `true`
**Assert:** A matrices are identical (bitwise). B matrices are identical (bitwise).
**Field:** `TransitionMatrices.A`, `TransitionMatrices.B`

### AC8: `nsensordata == 0` returns empty matrices *(runtime test — analytically derived)*
**Given:** Model with no sensors (`nsensordata == 0`)
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`
**Assert:** C is `Some(matrix)` with 0 rows and nx columns. D is `Some(matrix)` with 0 rows and nu columns.
**Field:** `TransitionMatrices.C`, `TransitionMatrices.D`

### AC9: Multi-sensor-type model *(runtime test — analytically derived)*
**Given:** Model with diverse sensors: `jointpos`, `jointvel`, `actuatorfrc` (position-stage, velocity-stage, acceleration-stage)
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`
**Assert:** C matrix has correct dimensions (nsensordata × nx). All sensor types are captured — C has non-zero columns for relevant perturbation types.
**Field:** `TransitionMatrices.C`

### AC10: Control-limited actuator D column *(runtime test — analytically derived)*
**Given:** Model with control-limited actuator at its upper bound (`ctrl[0] = ctrlrange.1`)
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`, centered
**Assert:** D column uses backward-only differencing (forward nudge is infeasible). D column is non-zero and correct direction.
**Field:** `TransitionMatrices.D`

### AC13: Structural C/D-to-A/B relationship for jointpos sensors *(runtime test — analytically derived)*
**Given:** 2-link pendulum with `jointpos` sensors on both joints, motor actuator
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`
**Assert:** For a `jointpos` sensor on joint `j`:
- `C[sensor_row, i] == A[joint_row, i]` for position columns `i = 0..nv`
  (jointpos measures `qpos[j]`, so `∂sensordata'/∂x = ∂qpos'/∂x = A` position rows)
- `D[sensor_row, k] == B[joint_row, k]` for all control columns
  (same chain: `∂sensordata'/∂ctrl = ∂qpos'/∂ctrl = B` position rows)
**Field:** `TransitionMatrices.C`, `TransitionMatrices.D`, `TransitionMatrices.A`, `TransitionMatrices.B`
**Rationale:** This structural relationship derives from MuJoCo's sensor→state mapping
and provides a verifiable expected value without running MuJoCo. For pure FD sensor
derivatives, this IS the conformance test — both MuJoCo and CortenForge use the same
FD algorithm, so FD-vs-FD agreement validates conformance. The C/D-to-A/B relationship
provides an independent structural cross-check.

### AC14: Cutoff-clamped sensor produces zero derivatives *(runtime test — analytically derived)*
**Given:** Model with `jointpos` sensor with `cutoff = 0.1`, joint at `qpos[0] = 0.5` (value exceeds cutoff)
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`
**Assert:** C row for the clamped sensor is all zeros (clamped value is constant → FD difference is 0)
**Field:** `TransitionMatrices.C`

### AC11: Backward compatibility — existing callers compile *(code review)*
**Assert:** All 28 `DerivativeConfig` construction sites compile after adding the new field. The 15 sites using `..Default::default()` require no change. The 13 struct-literal sites have `compute_sensor_derivatives: false` added.

### AC12: No regression in existing tests *(runtime test)*
**Given:** Full sim domain test suite
**After:** All changes applied
**Assert:** All existing tests pass. Zero failures attributable to Spec B changes.
**Field:** `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (C dimensions) | T1 | Direct |
| AC2 (D dimensions) | T1 | Direct |
| AC3 (opt-in negative) | T2 | Direct |
| AC4 (FD C accuracy) | T3 | Direct |
| AC5 (FD D accuracy) | T3 | Direct |
| AC6 (hybrid matches FD) | T4 | Direct |
| AC7 (A/B unchanged) | T5 | Direct |
| AC8 (nsensordata==0) | T6 | Direct + edge case |
| AC9 (multi-sensor-type) | T7 | Direct |
| AC10 (control-limited D) | T8 | Edge case |
| AC13 (C/D-to-A/B structural) | T11 | Direct |
| AC14 (cutoff-clamped zero) | T12 | Edge case |
| AC11 (backward compat) | — | Code review (manual) |
| AC12 (no regression) | T9 | Regression |

---

## Test Plan

### T1: Sensor derivative dimensions → AC1, AC2
**Model:** 2-link pendulum with `jointpos` sensors on both joints and 1 motor actuator.
Setup: `nsensordata=2`, `nv=2`, `na=0`, `nu=1`.
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`.
**Assert:**
- C is `Some` with shape (2, 4) — nsensordata × (2*nv + na)
- D is `Some` with shape (2, 1) — nsensordata × nu
**Expected value:** Analytically derived from dimension formulas.

### T2: Opt-in negative case → AC3
**Model:** Same as T1.
**After:** `mjd_transition_fd` with `DerivativeConfig::default()` (compute_sensor_derivatives = false).
**Assert:** `derivs.C.is_none()` and `derivs.D.is_none()`.
**Rationale:** Validates default behavior is unchanged. Matches existing test at `integration/derivatives.rs:206-207`.

### T3: C/D accuracy via independent FD validation → AC4, AC5
**Model:** 2-link pendulum with `jointpos` sensors and motor actuator.
Initial state: `qpos[0] = 0.3`, `qvel[1] = 0.5`, `ctrl[0] = 1.0`.
**After:** Compute C/D via `mjd_transition_fd(config)`. Then independently compute
sensor derivatives by manual per-column FD:
```rust
// For each state dimension i:
//   perturb x[i] ± eps, step, capture sensordata, difference
// For each control dimension j:
//   perturb ctrl[j] ± eps, step, capture sensordata, difference
```
**Assert:** Max relative error between `mjd_transition_fd` C/D and independent
manual FD C/D is < `1e-6` (floor = `1e-10`).
**Expected value:** Analytically derived (both methods use FD — they should agree
to machine precision given identical eps).

### T4: Hybrid C/D matches FD C/D → AC6
**Model:** 2-link pendulum with `jointpos` sensors and motor actuator.
Initial state: non-zero qpos, qvel, ctrl.
**After:** Compute via both `mjd_transition_fd` and `mjd_transition_hybrid`
with same eps, centered, and `compute_sensor_derivatives: true`.
**Assert:**
- `max_relative_error(hybrid.C, fd.C, 1e-10) < 1e-6`
- `max_relative_error(hybrid.D, fd.D, 1e-10) < 1e-6`
**Expected value:** Analytically derived — both use FD for sensor columns.

### T5: A/B unchanged when sensors enabled → AC7
**Model:** 2-link pendulum with sensors.
**After:** Compute `derivs_off = mjd_transition_fd(config_off)` where
`config_off.compute_sensor_derivatives = false`, and
`derivs_on = mjd_transition_fd(config_on)` where
`config_on.compute_sensor_derivatives = true` (same eps, centered).
**Assert:**
- `derivs_off.A == derivs_on.A` (bitwise — both call `step()` which evaluates sensors regardless)
- `derivs_off.B == derivs_on.B` (bitwise)
**Expected value:** Bitwise equality — enabling sensor capture adds no numerical perturbation.

### T6: `nsensordata == 0` edge case → AC8
**Model:** `Model::n_link_pendulum(2, 1.0, 0.1)` — no sensors.
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`.
**Assert:**
- C is `Some` with shape (0, 4) — 0 rows, nx columns
- D is `Some` with shape (0, 0) — 0 rows, 0 columns (nu=0 for pendulum)
**Expected value:** Analytically derived.

### T7: Multi-sensor-type model → AC9
**Model:** MJCF model with:
- 1 hinge joint + motor actuator
- `jointpos` sensor (position-stage, dim=1)
- `jointvel` sensor (velocity-stage, dim=1)
- `actuatorfrc` sensor (acceleration-stage, dim=1)
Total: nsensordata=3, nv=1, na=0, nu=1.
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`.
**Assert:**
- C is `Some` with shape (3, 2) — nsensordata × nx
- D is `Some` with shape (3, 1) — nsensordata × nu
- C has non-zero entries (all sensor types respond to state perturbations)
- D has non-zero entries (sensors respond to control perturbations through
  actuator force)
**Expected value:** Analytically derived. Exact non-zero pattern depends on
model configuration.

### T8: Control-limited actuator D column → AC10
**Model:** 1-joint model with control-limited actuator (`ctrlrange = [0, 1]`).
Set `ctrl[0] = 1.0` (at upper bound). Add `jointpos` sensor.
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`, centered.
**Assert:**
- D column is non-zero (backward-only differencing is used since forward
  nudge exceeds ctrlrange)
- D column has correct sign (negative perturbation → backward differencing)
**Expected value:** Analytically derived — backward-only FD uses `(y0 - y_minus) / eps`.

### T9: Regression — existing derivative tests pass → AC12
**After:** Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`.
**Assert:** All tests pass. Zero failures.
**Rationale:** Validates no regressions from DerivativeConfig field addition
or FD loop changes.

### T11: Structural C/D-to-A/B cross-check for jointpos → AC13
**Model:** 2-link pendulum with `jointpos` sensors on both joints and 1 motor actuator.
Initial state: `qpos[0] = 0.3`, `qvel[1] = 0.5`, `ctrl[0] = 1.0`.
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`.
**Assert:**
- For each `jointpos` sensor on joint `j` with sensor row `s`:
  - `C[s, i] == A[j, i]` for `i = 0..nv` (position columns, within 1e-12)
  - `D[s, k] == B[j, k]` for `k = 0..nu` (control columns, within 1e-12)
**Expected value:** Mathematical identity — `jointpos` directly measures `qpos[j]`,
so sensor derivatives equal state position derivatives. This relationship holds
identically for both MuJoCo's FD and ours — serving as a structural conformance check.

### T12: Cutoff-clamped sensor zero derivatives → AC14
**Model:** 1-hinge joint with motor actuator and `jointpos` sensor with
`cutoff = 0.1`. Set `qpos[0] = 0.5` (value far exceeds cutoff → clamped).
**After:** `mjd_transition_fd` with `compute_sensor_derivatives: true`.
**Assert:**
- C row for the clamped sensor is all zeros (clamped output is constant at
  `cutoff` regardless of state perturbation → FD difference is 0)
**Expected value:** Analytically derived — derivative of clamped function is 0
in clamped region. This is correct MuJoCo behavior captured naturally by FD.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| `nsensordata == 0` | No sensors → C/D should be empty `Some`, not `None`. MuJoCo returns non-null 0-row matrices. | T6 | AC8 |
| `nu == 0` (no controls) | D matrix should have 0 columns. No control loop runs. | T6 | AC8 |
| `na == 0` (no activations) | Activation block of C is absent (nx = 2*nv, not 2*nv+na). | T1, T3 | AC1, AC4 |
| Control-limited actuator at range boundary | Forward-only or backward-only FD for D column. `clampedDiff` behavior. | T8 | AC10 |
| Cutoff-clamped sensor | FD captures zero derivative in clamped region. MuJoCo behavior. | T12 | AC14 |
| Centered vs forward differences | Both differencing modes must produce correct C/D. | T3 | AC4, AC5 |
| Analytical A columns + sensor FD | Hybrid path: analytical position columns need separate sensor FD passes. | T4 | AC6 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T10 (forward-difference sensor derivatives) | C/D with `centered: false` | Validates forward-difference code path (O(eps) accuracy). Not covered by centered-only ACs but important for code coverage. |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `DerivativeConfig` struct layout | 3 fields: eps, centered, use_analytical | 4 fields: + compute_sensor_derivatives | Toward MuJoCo | 13 struct-literal callers | Add `compute_sensor_derivatives: false` to each |
| `TransitionMatrices.C/D` | Always `None` | `Some(matrix)` when `compute_sensor_derivatives: true` | Toward MuJoCo | Code pattern-matching on `C: None`/`D: None` | Update to handle `Some` case |
| `mjd_transition_fd` sensor evaluation | `step()` always evaluates sensors (but result discarded) | Same `step()` — now also captures sensordata when requested | Toward MuJoCo | None — transparent when disabled | None — transparent change |
| `mjd_transition_hybrid` FD cost | FD only for non-analytical columns | Additional FD steps for sensor-only columns when `compute_sensor_derivatives: true` | Toward MuJoCo | Performance-sensitive callers | Default is off — no cost when not requested |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/derivatives.rs` | Add `compute_sensor_derivatives` to `DerivativeConfig`, extend `mjd_transition_fd` and `mjd_transition_hybrid` FD loops for sensor recording, update `TransitionMatrices` docstrings, update `fd_convergence_check` struct literals | ~200 new + ~30 modified |
| `sim/L0/tests/integration/derivatives.rs` | New sensor derivative tests (T1-T10), update 7 struct-literal `DerivativeConfig` sites | ~250 new + ~7 modified |
| `sim/L0/tests/integration/implicit_integration.rs` | Update 4 struct-literal `DerivativeConfig` sites | ~4 modified |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_fd_produces_correct_dimensions` | `derivatives.rs:185` | Pass (unchanged) | Uses `DerivativeConfig::default()` — unaffected |
| `test_transition_matrices_c_d_none` | `derivatives.rs:205-207` | Pass (unchanged) | Asserts `C.is_none()` and `D.is_none()` with default config — still true |
| `test_centered_vs_forward` | `derivatives.rs:222-227` | Compile fix needed | Struct literal without `compute_sensor_derivatives` |
| `test_identity_at_origin` | `derivatives.rs:260` | Compile fix needed | Struct literal without `compute_sensor_derivatives` |
| `test_b_matrix_nonzero` | `derivatives.rs:350` | Compile fix needed | Struct literal without `compute_sensor_derivatives` |
| `test_contact_derivative_nonzero` | `derivatives.rs:397` | Compile fix needed | Struct literal without `compute_sensor_derivatives` |
| `test_eps_convergence` | `derivatives.rs:495-500` | Compile fix needed | 2 struct literals without `compute_sensor_derivatives` |
| `fd_convergence_check` | `derivatives.rs:3061-3069` | Compile fix needed | 2 struct literals without `compute_sensor_derivatives` |
| `test_implicit_*` (4 tests) | `implicit_integration.rs` | Compile fix needed | 4 struct literals without `compute_sensor_derivatives` |
| All other derivative tests | various | Pass (unchanged) | Use `DerivativeConfig::default()` or `..Default::default()` |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `forward/mod.rs:309` | `forward_skip(skipstage, skipsensor)` | Already supports `skipsensor` parameter — no changes needed |
| `lib.rs:265-270` | Exports `DerivativeConfig`, `TransitionMatrices` | Already exported — no new exports needed |
| `derivatives.rs:451` | `extract_state()` | Extracts state only — sensor extraction is independent |

---

## Execution Order

1. **S1: DerivativeConfig extension** → Compile all affected sites.
   Verify: `cargo build -p sim-core -p sim-conformance-tests` and
   `cargo test -p sim-core -p sim-conformance-tests` (all existing tests pass).

2. **S4: TransitionMatrices docstrings** → Update docs.
   Verify: No behavioral change. Docs-only.

3. **S2: FD sensor loop in mjd_transition_fd** → Core implementation.
   Verify: T1–T3, T5, T6, T8, T9 (all pure-FD tests).

4. **S3: FD sensor loop in mjd_transition_hybrid** → Hybrid integration.
   Verify: T4 (hybrid matches FD). T9 (regression).

5. **S5: nsensordata == 0 handling** → Edge case.
   Verify: T6 (0-row matrices).

Run `cargo test -p sim-core -p sim-conformance-tests` after each section.

---

## Performance Characterization

**Cost when `compute_sensor_derivatives = false` (default):** Zero additional
cost. No sensordata clones, no C/D allocation, no additional FD steps. The
`compute_sensors` bool gates all sensor-related allocation and capture.

**Note on `skipsensor` optimization:** MuJoCo's `mjd_stepFD` passes
`skipsensor=1` when C/D are not requested, which skips sensor evaluation
entirely during each FD perturbation step. CortenForge's `mjd_transition_fd`
(S2) continues using `step()` which always evaluates sensors — the only cost
saving when `compute_sensors=false` is avoiding sensordata clones, not
avoiding sensor evaluation. The full `skipsensor` optimization requires
migrating from `step()` to `forward_skip(skipstage, true) + integrate()`
for all FD columns, which is deferred to Out of Scope. The hybrid path's
sensor-only FD passes (S3) DO use `forward_skip(skipstage, false)` and
benefit from skip-stage optimization.

**Cost when `compute_sensor_derivatives = true`:**

- **`mjd_transition_fd`:** No additional `step()` calls. Each existing FD step
  adds one `sensordata.clone()` (ns floats). Overhead: `O(ns × (2·nx + 2·nu))`
  for centered FD (ns floats per column, nx + nu columns, 2 directions each).
  For a humanoid with ns=50, nv=27, nu=21: ~50 × (2·54 + 2·21) = 7,500
  additional float copies — negligible vs step() cost.

- **`mjd_transition_hybrid`:** Additional FD steps for sensor-only columns
  (columns that are analytical for A/B). In the worst case (all A/B columns
  analytical): 2·(2·nv + na + nu) additional `forward_skip + integrate` calls
  for centered FD. With skip-stage optimization, velocity and control columns
  skip position stage. For a humanoid: ~2·(54+21) = 150 additional
  `forward_skip + integrate` calls — significant but only when explicitly
  requested.

**Acceptable?** Yes. Cost is opt-in and zero when disabled. Users requesting
sensor derivatives accept the FD cost. The skip-stage optimization reduces
per-column cost. Future work (DT-49 parallel FD) could further reduce wall time.

---

## Out of Scope

- **Analytical sensor derivatives** — MuJoCo uses FD for all sensor C/D
  columns. Per-sensor-type analytical Jacobians (e.g., `∂jointpos/∂qpos` is
  identity) would be a CortenForge extension, not conformance. Deferred
  post-v1.0. *Conformance impact: none — FD matches MuJoCo exactly.*

- **Inverse dynamics sensor derivatives** (`DsDq`, `DsDv`, `DsDa` in
  `mjd_inverseFD`) — These are sensor columns of the inverse dynamics FD
  wrapper. Tracked as umbrella Out of Scope item. *Conformance impact:
  minimal — inverse sensor derivatives are rarely used.*

- **Parallel FD computation** (DT-49) — Each perturbation column is
  sequential. Parallelization deferred. *Conformance impact: none.*

- **`step()` → `forward_skip + integrate` migration for existing FD loops** —
  The rubric (EGT-8) recommended switching `mjd_transition_fd` from `step()`
  to `forward_skip + integrate` for skip-stage optimization. This is a
  performance improvement that could be done as part of Spec B but is not
  required for correctness. Sensor-only FD passes in the hybrid path (S3)
  use `forward_skip + integrate`, but existing FD loops in `mjd_transition_fd`
  (S2) continue using `step()`. The behavioral correctness is identical —
  `step()` evaluates sensors by default. Deferring the `step()` migration
  keeps S2 simpler and focused on sensor recording. *Conformance impact:
  none — `step()` and `forward_skip(None, false) + integrate()` produce
  identical results.*
