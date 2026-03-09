# Spec B — Sensor Derivatives (C, D Matrices): Spec Quality Rubric

Grades the Spec B spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical C and D matrices to MuJoCo's `mjd_transitionFD`."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest.

> **Conformance status:** Unlike Spec A (which was a CortenForge extension),
> Spec B is a **direct MuJoCo conformance task.** MuJoCo's `mjd_transitionFD()`
> and `mjd_stepFD()` compute C and D matrices alongside A and B. Our
> implementation must produce identical numerical results. The MuJoCo C source
> (`engine_derivative_fd.c`) is the single source of truth — not docs, not
> header files. Every behavior is port-from-source.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

Empirical verification against the MuJoCo C source confirms the umbrella
spec's scope with minor corrections.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "C matrix: `∂sensordata_{t+1}/∂x_t` (nsensordata × (2*nv + na))" | Confirmed. `mjd_stepFD()` computes `DsDq` (ns × nv), `DsDv` (ns × nv), `DsDa` (ns × na) which assemble into C (ns × ndx where ndx=2*nv+na). `mjd_transitionFD()` transposes from column-major to row-major. | **In scope.** Dimensions confirmed. |
| "D matrix: `∂sensordata_{t+1}/∂u_t` (nsensordata × nu)" | Confirmed. `mjd_stepFD()` computes `DsDu` (ns × nu). Transposes to row-major in `mjd_transitionFD()`. | **In scope.** |
| "Sensor outputs are measured after each perturbation step — black-box FD" | Confirmed. `getState()` in `engine_derivative_fd.c` copies `d->sensordata` alongside state after each `mj_stepSkip()`. | **In scope.** |
| "Opt-in computation: C/D only populated when explicitly requested" | Confirmed. `mjd_transitionFD()` takes `C` and `D` as nullable pointers. When NULL, sensor derivatives are skipped entirely. `mjd_stepFD()` derives `skipsensor = !DsDq && !DsDv && !DsDa && !DsDu` — when all are NULL, `mj_stepSkip()` is called with `skipsensor=1`, avoiding sensor evaluation cost. | **In scope.** Map to `DerivativeConfig.compute_sensor_derivatives: bool`. |
| "Hybrid optimization: analytical sensor position derivatives" | **NOT how MuJoCo works.** MuJoCo uses pure FD for ALL sensor derivative columns (position, velocity, activation, control). There is no analytical sensor derivative path. CortenForge's hybrid path already exists for A/B matrices, but sensor C/D should follow MuJoCo's pure FD approach. | **Drop analytical hybrid sensor path.** FD-only for C/D matches MuJoCo exactly. If future optimization is desired, defer to post-v1.0. |
| "Sensor noise and cutoff interaction with derivatives" | MuJoCo does NOT disable noise or cutoff during FD. Sensor noise in MuJoCo is metadata (`sensor_noise` field) — not applied during `mj_sensor_*`. Cutoff clamping IS applied via `mj_sensor_postprocess()` / `apply_cutoff()` during normal sensor evaluation. Since FD runs the full pipeline including sensors, cutoff affects the derivative (creates flat regions where clamped). This is correct MuJoCo behavior. | **In scope.** No special noise/cutoff handling needed — cutoff is captured by FD naturally. Document that cutoff creates zero-derivative regions. |
| "Integration into both mjd_transition_fd() and mjd_transition_hybrid()" | MuJoCo has only `mjd_transitionFD()` (pure FD). CortenForge's hybrid path is an extension. For conformance, C/D must be FD-computed in both paths. The hybrid path can use FD for C/D columns while using analytical for A/B columns. | **In scope.** Both `mjd_transition_fd` and `mjd_transition_hybrid` get C/D support, both using FD for sensor columns. |
| "Per-sensor-type output handling" | NOT per-sensor-type. `mjd_stepFD()` treats sensordata as a flat `f64[nsensordata]` vector. FD perturbs the input, runs the full pipeline (which evaluates all sensors), and differences the entire sensordata vector. No per-sensor-type logic needed. | **Simplify.** No per-sensor-type Jacobian logic — black-box FD captures all sensor types automatically. |
| Control clamping in `mjd_stepFD` for D matrix | `mjd_stepFD()` checks `actuator_ctrllimited` and `actuator_ctrlrange` — only nudges control within valid range. Uses `clampedDiff()` for forward/backward/centered selection based on range constraints. | **In scope.** Spec must handle control-limited actuators correctly. |

**Final scope:**

1. `DerivativeConfig.compute_sensor_derivatives: bool` — opt-in flag (default false)
2. Extend `mjd_transition_fd()` perturbation loop to record `sensordata` deltas alongside state deltas
3. Extend `mjd_transition_hybrid()` to compute FD sensor columns (C/D always use FD, A/B use analytical)
4. Populate `TransitionMatrices.C` and `.D` from `Option<DMatrix<f64>> = None` to `Some(matrix)` when requested
5. Pass `skipsensor=false` to `forward_skip()` / `step()` when sensor derivatives are being computed
6. Handle `nsensordata == 0` edge case (no sensors → C/D remain None)
7. Validation: C/D matrices match standalone FD sensor derivatives within tolerance

---

## Empirical Ground Truth

### EGT-1: MuJoCo `mjd_transitionFD` is a thin wrapper around `mjd_stepFD`

**MuJoCo version:** 3.x (current main branch, verified 2026-03-09).
**Source:** `engine_derivative_fd.c`

`mjd_transitionFD(m, d, eps, flg_centered, A, B, C, D)` does:
1. Stack-allocates transposed matrices (`AT`, `BT`, `CT`, `DT`) — only when
   the corresponding output pointer is non-NULL
2. Splits A transposed into `DyDq`/`DyDv`/`DyDa` sub-blocks; splits C
   transposed into `DsDq`/`DsDv`/`DsDa` sub-blocks
3. Calls `mjd_stepFD(m, d, eps, flg_centered, DyDq, DyDv, DyDa, BT, DsDq, DsDv, DsDa, DT)`
4. Transposes results back to control-theory (row-major) convention

**Key insight:** The sensor opt-in flows from `C`/`D` pointer nullability.
When `C=NULL` AND `D=NULL`, all `DsD*` pointers are NULL, so `mjd_stepFD`
sets `skipsensor=1`.

**CortenForge mapping:** Since we don't have nullable output pointers, we
use `config.compute_sensor_derivatives: bool` as the opt-in mechanism.
When false, skip sensor evaluation during FD (pass `skipsensor=true` to
`forward_skip`/`step`). When true, evaluate sensors and record deltas.

### EGT-2: `mjd_stepFD` perturbation loop with sensor recording

**Source:** `engine_derivative_fd.c` → `mjd_stepFD()`

The function takes 8 output Jacobian pointers (4 state × {q,v,a,u} + 4
sensor × {q,v,a,u}). Key details:

1. **Sensor buffers:** When sensors are requested, allocates `sensor` (baseline),
   `sensor_plus` (forward-perturbed), `sensor_minus` (backward-perturbed).
   Each is `ns = m->nsensordata` length.

2. **Nominal step:** Calls `mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor)` for
   the unperturbed nominal step. Saves sensor data: `getState(m, d, next, sensor)`.

3. **Per-column pattern (same for all 4 perturbation types):**
   ```c
   // Perturb +eps
   mj_stepSkip(m, d, skipstage, skipsensor);
   getState(m, d, next_plus, sensor_plus);
   mj_setState(m, d, fullstate, restore_spec);  // restore

   // Perturb -eps (if centered)
   mj_stepSkip(m, d, skipstage, skipsensor);
   getState(m, d, next_minus, sensor_minus);
   mj_setState(m, d, fullstate, restore_spec);  // restore

   // State difference
   if (DyDx) clampedStateDiff(m, DyDx+i*ndx, next, next_plus, next_minus, eps);
   // Sensor difference
   if (DsDx) clampedDiff(DsDx+i*ns, sensor, sensor_plus, sensor_minus, eps, ns);
   ```

4. **Skip stages per perturbation type:**
   - Control (u): `mjSTAGE_VEL` — position+velocity unchanged
   - Activation (a): `mjSTAGE_VEL` — position+velocity unchanged
   - Velocity (v): `mjSTAGE_POS` — position unchanged
   - Position (q): `mjSTAGE_NONE` — nothing can be skipped

5. **Control clamping:** For control columns, checks `actuator_ctrllimited`
   and `actuator_ctrlrange`. Only nudges if the perturbed value stays within
   range. Uses `clampedDiff` to handle forward-only, backward-only, or
   centered based on what's feasible.

### EGT-3: Sensor evaluation during FD — `mj_stepSkip` with `skipsensor`

**Source:** `engine_derivative_fd.c` → `mj_stepSkip()`

```c
void mj_stepSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
    mj_checkPos(m, d);
    mj_checkVel(m, d);
    mj_forwardSkip(m, d, skipstage, skipsensor);
    mj_checkAcc(m, d);
    // integrator dispatch...
}
```

When `skipsensor=0` (sensors needed for derivatives), `mj_forwardSkip`
evaluates `mj_sensor_pos`, `mj_sensor_vel`, and `mj_sensor_acc` at
appropriate pipeline stages. Sensor postprocessing (cutoff clamping) runs
as part of the sensor pipeline.

**CortenForge mapping:** Our `forward_skip(model, skipstage, skipsensor)`
already has the `skipsensor` parameter. Currently, the FD loops always
pass `skipsensor=true` (since C/D aren't computed). Spec B changes this:
when `config.compute_sensor_derivatives`, pass `skipsensor=false`.

### EGT-4: Matrix layout and transposition

**Source:** `engine_derivative_fd.c`

`mjd_stepFD` computes Jacobians in **column-major** layout (each column
corresponds to one perturbation direction). `mjd_transitionFD` then
transposes to **row-major** (control theory convention: row = output,
col = input).

Internal layout in `mjd_stepFD`:
- `DsDq[i*ns .. (i+1)*ns]` = column `i` of sensor-position Jacobian
- `DsDv[i*ns .. (i+1)*ns]` = column `i` of sensor-velocity Jacobian
- etc.

After `mju_transpose(C, CT, ndx, ns)` in `mjd_transitionFD`:
- `C[row * (2*nv+na) + col]` = `∂sensordata[row] / ∂state[col]`

**CortenForge mapping:** nalgebra `DMatrix` uses column-major storage.
`column_mut(i).copy_from(&col)` writes a column directly — no manual
transposition needed. The existing `mjd_transition_fd` already uses this
pattern for A and B matrices. C and D follow identically.

### EGT-5: `nsensordata == 0` edge case

When `nsensordata == 0` (no sensors defined), MuJoCo's behavior:
- `mjd_transitionFD` with `C` and `D` non-NULL allocates 0-byte buffers
  (which is effectively a no-op)
- `mjd_stepFD` allocates 0-length sensor arrays
- The perturbation loop still runs for state derivatives but sensor
  differencing operates on 0-length arrays (no-op)
- Result: C and D are 0×ndx and 0×nu matrices (valid but empty)

**CortenForge mapping:** When `nsensordata == 0` AND
`compute_sensor_derivatives == true`, return `Some(DMatrix::zeros(0, ndx))`
for C and `Some(DMatrix::zeros(0, nu))` for D. This maintains the
`Some`/`None` semantics: `Some` = "was computed", `None` = "not requested".
Alternative: return `None` when `nsensordata == 0` regardless of flag.
The spec should decide which — MuJoCo returns empty (non-null) matrices.

### EGT-6: Sensor noise is not applied during forward

MuJoCo's `sensor_noise` field is metadata for downstream consumers (e.g.,
state estimation, Kalman filters). It is **not** added to sensor values
during `mj_sensor_pos/vel/acc`. Therefore, FD sensor derivatives are
noiseless by construction — no special handling needed.

**Source verification:** Grepped `engine_sensor.c` — `sensor_noise` is
read only in `apply_noise()` which is NOT called during normal sensor
evaluation. It's a post-hoc addition for simulation output, not part of
the pipeline.

**CortenForge mapping:** Our `mj_sensor_postprocess` handles cutoff
clamping but does not add noise. Consistent with MuJoCo. No changes
needed for Spec B.

### EGT-7: Cutoff clamping creates flat derivative regions

Sensor cutoff (`sensor_cutoff > 0`) clamps sensor values via
`mj_sensor_postprocess` / `apply_cutoff()`. When the true sensor value
exceeds the cutoff, the clamped value is constant → FD difference is 0
→ C/D columns are 0 for those sensors at that operating point.

This is correct behavior — the derivative of a clamped function is 0 in
the clamped region. MuJoCo captures this naturally through FD.

### EGT-8: CortenForge codebase context — insertion points

**Files and functions to modify:**

| File | Function/Struct | Change | Line(s) |
|------|----------------|--------|---------|
| `derivatives.rs` | `DerivativeConfig` | Add `compute_sensor_derivatives: bool` field | 142-163 |
| `derivatives.rs` | `DerivativeConfig::default()` | Add `compute_sensor_derivatives: false` | 165-173 |
| `derivatives.rs` | `mjd_transition_fd()` | Save baseline sensordata, record sensor deltas in perturbation loop, populate C/D | 227-354 |
| `derivatives.rs` | `mjd_transition_hybrid()` | Add FD sensor column computation, populate C/D | 2368-2886 |
| `derivatives.rs` | `TransitionMatrices` docstrings | Update C/D docs to reflect populated state | 117-123 |
| `forward/mod.rs` | N/A (no changes) | `forward_skip()` already supports `skipsensor` parameter | 309-387 |
| `lib.rs` | N/A (no new exports) | `DerivativeConfig`, `TransitionMatrices` already exported | 265-270 |

**Critical finding: `mjd_transition_fd()` uses `step()`, NOT `forward_skip + integrate`**

Despite DT-53's Contract 3 (umbrella) stating that FD loops would switch
to `forward_skip() + integrate()`, this has NOT been done yet.
`mjd_transition_fd()` still calls `scratch.step(model)?` at lines 256,
286, 304, 326, 337. Similarly, `mjd_transition_hybrid()` uses
`scratch.step()` at lines 2601, 2687, 2714, 2731, 2757, 2774, 2860, 2870.

Only `mjd_inverse_fd()` uses `forward_skip()` (it was implemented as part
of DT-53's session). Spec B must decide:
1. **Keep `step()`**: Simpler. `step()` calls `forward()` which always
   evaluates sensors. For sensor derivatives, this works — sensordata is
   populated. For non-sensor columns, sensor eval is wasted cost.
2. **Switch to `forward_skip + integrate`**: Matches MuJoCo's
   `mj_stepSkip()` pattern. Enables skip-stage optimization AND sensor
   opt-in via `skipsensor` parameter. More MuJoCo-conformant.

Recommendation: switch to `forward_skip(skipstage, !compute_sensors) +
integrate()` in the perturbation loop. This:
- Enables skip-stage optimization for velocity/control columns
  (`skipstage=Pos/Vel` skips FK/collision)
- Enables sensor opt-in (`skipsensor=true` when C/D not requested)
- Matches `mj_stepSkip()` semantics exactly
- Is the natural completion of DT-53's intended integration

**Note:** `step()` includes check/sleep/warmstart overhead that FD loops
don't need. `forward_skip + integrate` is leaner and closer to MuJoCo's
`mj_stepSkip`.

**Critical: `mjd_transition_hybrid()` (lines 2368-2886)**
The hybrid path currently:
- Uses analytical `qDeriv` for velocity columns of A
- Uses FD for position columns of A (or analytical from Spec A)
- Uses analytical for simple B columns, FD for complex ones

For sensor C/D, ALL columns must use FD (no analytical sensor path).
This means the hybrid path needs a separate FD loop for sensor columns
when `compute_sensor_derivatives` is true. Two design options:
1. **Integrated:** Add sensor recording to every FD column the hybrid
   path already computes, plus add FD sensor-only columns for the
   analytical columns. More complex but avoids redundant computation.
2. **Separate:** After computing A/B (hybrid), run a separate pure FD
   pass for C/D only. Simpler code but ~2x cost for sensor columns.

The MuJoCo approach is option 1: a single FD loop computes both state
and sensor columns simultaneously. Spec should match this for efficiency.

**Critical: ~25 struct literal callers of `DerivativeConfig` will break**

Adding `compute_sensor_derivatives: bool` to `DerivativeConfig` will cause
compile errors at every site using struct literal syntax without
`..Default::default()`. Verified call sites:

| File | Approx. count | Pattern |
|------|---------------|---------|
| `sim/L0/core/src/derivatives.rs` (tests) | 6 | `DerivativeConfig { eps, centered, use_analytical }` |
| `sim/L0/tests/integration/derivatives.rs` | 15+ | Same struct literal pattern |
| `sim/L0/tests/integration/implicit_integration.rs` | 4 | Same struct literal pattern |

Migration: add `compute_sensor_derivatives: false` to each, or switch to
`DerivativeConfig { ..., ..Default::default() }` pattern. The spec should
choose and document the migration approach.

**Note:** No production callers (only test code) use struct literals.
`mjd_transition()` dispatches internally and constructs config via
`Default::default()`. So the breakage is test-only — high count but low
risk.

### EGT-9: Existing test infrastructure for derivatives

| Test file / area | Location | Count | Relevance |
|-----------------|----------|-------|-----------|
| `derivatives.rs` integration tests | lines 4100+ | ~50 | Test A/B matrices, will need C/D extension |
| `fluid_derivatives.rs` tests | separate file | ~32 | Unaffected — fluid velocity only |
| Phase A conformance (FD) | `derivatives.rs` | 8+ | Must not regress |
| Phase D conformance (hybrid) | `derivatives.rs` | 8+ | Must not regress |
| DT-51 tests (inverse FD) | `derivatives.rs` | 8 | Unaffected unless sensor extension is added |
| **C/D = None assertions** | `integration/derivatives.rs:206-207` | 1 | Asserts `derivs.C.is_none()` and `derivs.D.is_none()`. Uses `DerivativeConfig::default()` — will still pass if default has `compute_sensor_derivatives: false`. Validates opt-in negative case. |

### EGT-10: `mj_stepSkip` uses integrator-aware skip

**Source:** `engine_derivative_fd.c` → `mj_stepSkip()`

```c
void mj_stepSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
    mj_checkPos(m, d);
    mj_checkVel(m, d);
    mj_forwardSkip(m, d, skipstage, skipsensor);
    mj_checkAcc(m, d);
    if (mjENABLED(mjENBL_FWDINV)) mj_compareFwdInv(m, d);
    switch (m->opt.integrator) {
    case mjINT_EULER:    mj_EulerSkip(m, d, skipstage >= mjSTAGE_POS); break;
    case mjINT_RK4:      mj_RungeKutta(m, d, 4); break;
    case mjINT_IMPLICIT:
    case mjINT_IMPLICITFAST:
                         mj_implicitSkip(m, d, skipstage >= mjSTAGE_VEL); break;
    }
}
```

**Key:** `mj_stepSkip` is `forwardSkip + checkAcc + integrator`. CortenForge's
`step()` does `forward() + integrate()` (plus check/sleep/warmstart). For
sensor FD, the loop must use `forward_skip(skipstage, skipsensor=false)` +
`integrate()` to match `mj_stepSkip` semantics.

**CortenForge already has `step_skip`-equivalent infrastructure:** DT-53
implemented `forward_skip()` and the FD loops can call
`forward_skip(stage, sensor) + integrate()` to match `mj_stepSkip`.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does for sensor derivative
> computation — exact function names, field names, calling conventions,
> and edge cases. This is a direct conformance task: our C/D matrices
> must match MuJoCo's `mjd_transitionFD()` output.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function cited with source file and exact behavior: `mjd_transitionFD()` wrapper, `mjd_stepFD()` core loop, `getState()` sensor capture, `mj_stepSkip()` dispatch, `clampedDiff()` for control clamping. Skip-stage assignments per perturbation type documented (`mjSTAGE_VEL` for ctrl/act, `mjSTAGE_POS` for vel, `mjSTAGE_NONE` for pos). Sensor opt-in mechanism (pointer nullability → `skipsensor` flag) fully described. Edge cases addressed: `nsensordata == 0` (empty model), control-limited actuators (`actuator_ctrllimited` + `actuator_ctrlrange` → `clampedDiff` forward/backward/centered selection), cutoff clamping (flat derivative regions), `na == 0` (no activations). C code snippets included for non-obvious patterns (e.g., `clampedDiff`, `getState` sensor capture). Numerical expectations from MuJoCo output stated for at least one representative model. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage (e.g., missing control clamping details). |
| **B** | Correct at high level, but missing specifics (e.g., "sensors are captured during FD" without saying how `getState` works) — or based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or invented. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. The FD sensor
> recording is straightforward (save baseline sensordata, difference
> after perturbation) but must be specified for all four perturbation
> types with correct skip-stage assignments.

| Grade | Bar |
|-------|-----|
| **A+** | Complete Rust pseudocode for both `mjd_transition_fd` and `mjd_transition_hybrid` sensor extensions. All four perturbation types (position, velocity, activation, control) handled with correct `skipsensor=false` and appropriate skip-stage. Control clamping logic specified in Rust. Baseline sensordata capture, per-column sensor differencing, and C/D matrix assembly all written out. An implementer can type it in without reading MuJoCo source. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit (e.g., exact control clamping edge cases). |
| **B** | Algorithm structure is clear but some steps hand-waved or the hybrid path's sensor integration is underspecified. |
| **C** | Skeleton only — "record sensor deltas during FD loop." |

### P3. Convention Awareness

> Spec explicitly addresses CortenForge conventions where they differ
> from MuJoCo: nalgebra column-major storage vs MuJoCo row-major,
> `step()` vs `mj_stepSkip()`, `Option<DMatrix>` vs nullable pointers,
> `DerivativeConfig` vs function parameters.

| Grade | Bar |
|-------|-----|
| **A+** | Convention difference table present: MuJoCo nullable C/D pointers → CortenForge `compute_sensor_derivatives` bool. MuJoCo `mj_stepSkip()` → CortenForge `forward_skip() + integrate()`. MuJoCo column-major transposed → CortenForge nalgebra `column_mut()` (no manual transpose). MuJoCo `mj_getState(STATE_PHYSICS)` → CortenForge `extract_state()`. MuJoCo `mju_copy(sensor, d->sensordata, ns)` → CortenForge `scratch.sensordata.clone()`. Each porting rule verified to preserve numerical equivalence. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. For sensor derivatives,
> ACs must specify: (1) concrete model with specific sensors, (2) exact
> C/D matrix dimensions, (3) tolerance for FD-vs-FD validation or
> MuJoCo-vs-CortenForge conformance.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has: (1) concrete model (e.g., "2-link pendulum with jointpos + framevelocity sensors"), (2) exact expected dimensions (e.g., "C is 4×6 for nsensordata=4, nv=3, na=0"), (3) tolerance (`1e-6` relative for FD matching). At least one AC verifies C/D against standalone per-column FD (independent verification). At least one AC verifies `compute_sensor_derivatives=false` returns `C: None, D: None` (opt-in negative case). ACs cover `nsensordata==0`, control-limited actuator D columns, cutoff-clamped sensors (zero derivative region). Code-review ACs specify structural properties (field added, Default impl updated, backward-compatible callers). |
| **A** | ACs are testable. Some lack exact numerical expectations or edge-case coverage. |
| **B** | ACs are directionally correct but vague ("C matrix should be populated"). |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, and interactions. Each AC maps to
> at least one test. At least one MuJoCo conformance test per major code
> path.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory: `nsensordata==0`, `nu==0` (no controls → D is 0-column), `na==0` (no activations → activation block of C absent), control-limited actuators (forward-only FD for D columns at limits), cutoff-clamped sensors (zero derivative region), centered vs forward differences. Negative test: `compute_sensor_derivatives=false` → C/D remain None. At least one non-trivial model (multi-joint, multi-sensor-type). Regression test: A/B matrices unchanged when C/D computation is enabled (no numerical coupling). At least one MuJoCo-vs-CortenForge conformance test with expected values from running MuJoCo. |
| **A** | Good coverage. Minor edge-case gaps. Conformance tests present. |
| **B** | Happy path covered. Edge cases sparse. No conformance tests. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other
> Phase 11 deliverables are explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. Prerequisites stated: DT-53 (`forward_skip`) must be landed (commit hash cited), Spec A (`mjd_smooth_pos`) soft dependency documented (FD works without, hybrid A columns can use analytical while C/D stay FD). Cross-spec interactions: C/D computation in hybrid path must work regardless of whether Spec A's analytical position columns are used for A matrix. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and
> every existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. `DerivativeConfig` field addition documented with backward-compatibility analysis (existing callers use `DerivativeConfig::default()` or struct literal — struct literal callers will break, migration path stated). `TransitionMatrices.C`/`.D` change from always-None to conditionally-Some documented. Existing test impact: names specific test functions that construct `DerivativeConfig` or pattern-match on `TransitionMatrices` fields. Behavioral change: `mjd_transition_fd` now evaluates sensors during FD when `compute_sensor_derivatives=true` — performance impact quantified (adds sensor eval cost per FD column). No A/B matrix numerical change. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical
> terminology throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "sensor derivatives" vs "C/D matrices" used consistently. Every cross-reference accurate. Dimension formulas match between MuJoCo Reference, Specification, and AC sections. Edge case lists consistent across sections. `nsensordata` vs `ns` naming consistent. Skip-stage assignments consistent between algorithm description and MuJoCo reference. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. FD Loop Integration Design

> Spec B's core challenge is integrating sensor recording into two
> existing FD loops (pure FD and hybrid) without regressing A/B matrices
> and without duplicating perturbation steps. This criterion grades the
> integration design.

**Boundary with P2:** P2 grades algorithmic completeness (can an
implementer type it in?). P9 grades whether the integration design is
correct and efficient (does the sensor recording fit cleanly into the
existing loop structure without redundant computation or numerical
coupling?).

| Grade | Bar |
|-------|-----|
| **A+** | Both `mjd_transition_fd` and `mjd_transition_hybrid` integration designs specified. For each: (1) exactly which FD columns already exist that can carry sensor recording, (2) which analytical columns need new FD sensor-only columns, (3) how baseline sensordata is captured (before nominal step), (4) skip-stage assignment for each sensor column type matches EGT-2. Efficiency analysis: no redundant `step()` calls — sensor recording piggybacks on existing FD steps where possible. For hybrid path: explicit list of which A/B columns are analytical (need new FD sensor passes) vs which are already FD (piggyback). No A/B numerical coupling: enabling sensor derivatives does not change A/B values (same perturbation, same step, just also reading sensordata). |
| **A** | Integration design is sound. Minor efficiency gaps (e.g., some redundant steps). |
| **B** | Integration design is sketched but not fully specified for both paths. |
| **C** | Integration design not addressed — just "add sensor recording to FD loop." |

### P10. Opt-In API Design

> The opt-in mechanism (`compute_sensor_derivatives`) must be
> backward-compatible, zero-cost when disabled, and cleanly integrated
> into both code paths.

**Boundary with P7:** P7 grades blast radius (what breaks). P10 grades
whether the API design itself is clean — correct default, zero-cost
when off, ergonomic for callers.

| Grade | Bar |
|-------|-----|
| **A+** | `DerivativeConfig` field addition fully specified: name, type, default value, doc comment. Backward compatibility analysis: callers using `DerivativeConfig::default()` unaffected; callers using struct literal `DerivativeConfig { eps, centered, use_analytical }` will get a compile error (missing field) — list of such call sites identified and migration documented. When `compute_sensor_derivatives=false`: no sensor evaluation cost during FD (sensors skipped), C/D remain `None`, A/B unchanged. When `compute_sensor_derivatives=true` but `nsensordata==0`: C/D are `Some(empty matrix)` or `None` (decision stated with rationale). Callers that pattern-match `C: None` will need updating when C is Some — risk documented. |
| **A** | API design is clean and backward-compatible. Minor documentation gaps. |
| **B** | API design works but backward-compatibility analysis incomplete. |
| **C** | API design not fully specified — just "add a bool field." |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions
      (`mjd_stepFD`, `getState`, `clampedDiff`, `mj_stepSkip`), specific
      edge cases (`nsensordata==0`, control-limited actuators, cutoff
      clamping), and specific CortenForge insertion points
      (`mjd_transition_fd` lines 270-354, `DerivativeConfig` lines
      142-173). Two reviewers would agree on grades.

- [x] **Non-overlap:** P1 vs P9: P1 grades whether the MuJoCo reference
      is correctly described; P9 grades whether the integration design
      into CortenForge's loop structure is sound. P7 vs P10: P7 grades
      blast radius (what breaks); P10 grades API design quality
      (backward-compat, zero-cost-when-off, ergonomics). P2 vs P9: P2
      grades whether the algorithm can be typed in; P9 grades whether it
      fits cleanly into existing loops.

- [x] **Completeness:** 10 criteria cover: MuJoCo reference (P1),
      algorithm (P2), conventions (P3), ACs (P4), tests (P5),
      dependencies (P6), blast radius (P7), consistency (P8), loop
      integration (P9), API design (P10). The two domain-specific
      criteria (P9, P10) cover the task's unique dimensions: integrating
      into existing FD infrastructure and the opt-in API design.

- [x] **Gradeability:** P1→MuJoCo Reference section. P2→Specification
      (S1-S5). P3→Convention Notes. P4→Acceptance Criteria. P5→Test Plan.
      P6→Prerequisites/Execution Order. P7→Files Affected/Blast Radius.
      P8→Cross-cutting. P9→Specification (loop integration sections).
      P10→Specification (DerivativeConfig section).

- [x] **Conformance primacy:** P1 is tailored with specific MuJoCo C
      functions, edge cases, and skip-stage assignments. P4 requires
      MuJoCo-verified expected values. P5 requires conformance tests.
      P9 requires skip-stage assignments to match EGT-2.

- [x] **Empirical grounding:** EGT-1 through EGT-10 are filled in with
      verified MuJoCo source analysis and CortenForge codebase research.
      Every P1 A+ bar item traces to an EGT entry.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, S3, S4, S5) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Specification (S2 FD loop extension, S3 hybrid path extension) |
| P10 | Specification (S1 DerivativeConfig, S4 TransitionMatrices wiring) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. FD Loop Integration Design | | |
| P10. Opt-In API Design | | |

**Overall: — (Rev 2)**

> Scorecard is blank — this rubric grades the spec, which has not been
> written yet. Grades will be populated during spec review (Session 10).

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Initial umbrella claimed "hybrid analytical sensor derivatives" — MuJoCo uses pure FD for all sensor columns | EGT verification (mjd_stepFD source) | Scope Adjustment: dropped hybrid analytical sensor path. FD-only for C/D. | Rubric Rev 1 |
| R2 | P1 | Umbrella claimed "per-sensor-type Jacobian logic" — MuJoCo treats sensordata as flat vector, no per-type logic | EGT verification (mjd_stepFD source) | Scope Adjustment: simplified to flat sensordata differencing. | Rubric Rev 1 |
| R3 | P9 | Hybrid path integration design is complex: analytical A/B columns don't have FD sensor data. Need to address whether to add sensor-only FD passes for those columns or restructure the loop. | Codebase context analysis (EGT-8) | Added P9 criterion (FD Loop Integration Design) to grade this explicitly. | Rubric Rev 1 |
| R4 | P10 | `DerivativeConfig` struct literal callers will break when field is added — need backward-compat analysis | Codebase context analysis (EGT-8) | Added P10 criterion (Opt-In API Design) with explicit backward-compat bar. | Rubric Rev 1 |
| R5 | P1 | Control clamping in `mjd_stepFD` — `actuator_ctrllimited` + `actuator_ctrlrange` interaction was not in initial umbrella scope | EGT verification (mjd_stepFD source) | Added control clamping to P1 edge cases and P4 AC requirements. | Rubric Rev 1 |
| R6 | P1/P9 | `mj_stepSkip` includes integrator dispatch — not just `forward_skip` | EGT verification (EGT-10) | Added EGT-10 documenting `mj_stepSkip` = `forwardSkip + checkAcc + integrator`. P9 must address `forward_skip + integrate()` vs `step()` for correctness. | Rubric Rev 1 |
| R7 | P9 | `mjd_transition_fd` and `mjd_transition_hybrid` still use `step()` not `forward_skip + integrate` — DT-53 Contract 3 was aspirational | Stress test (codebase grep for `scratch.step`) | Updated EGT-8 with critical finding. P9 bar requires spec to address `step()` → `forward_skip + integrate` transition. | Rubric Rev 2 |
| R8 | P10/P7 | ~25 struct literal callers of `DerivativeConfig` in test files will break when field is added | Stress test (codebase grep for `DerivativeConfig {`) | Updated EGT-8 with exact call site counts. P10 bar requires migration plan. P7 bar requires naming affected test files. | Rubric Rev 2 |
| R9 | P9 | Hybrid path analytical columns (velocity, simple B) have no FD step — how to get sensor columns for those? Need explicit design decision in spec. | Stress test (analyzing hybrid path structure) | EGT-8 already had design options (integrated vs separate). P9 A+ bar strengthened to require explicit per-column-type analysis. | Rubric Rev 2 |
| R10 | P5/P7 | Existing test at `integration/derivatives.rs:206-207` asserts `derivs.C.is_none()` — validates opt-in negative case, not a regression risk | Stress test (grep for C/D pattern matching) | Added to EGT-9 test inventory. P5 bar already requires negative test for `compute_sensor_derivatives=false`. | Rubric Rev 2 |
