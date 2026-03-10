# Layer C Trajectory Comparison — Spec

**Status:** Draft
**Phase:** Roadmap Phase 12 — Conformance Test Suite
**Effort:** M
**MuJoCo ref:** `mj_step()` in `engine_forward.c`, `mj_Euler()` in `engine_forward.c`,
`mj_integratePos()` in `engine_util_misc.c`
**MuJoCo version:** 3.4.0
**Test baseline:** 3,600+ sim domain tests
**Prerequisites:**
- Phase 12 Session 4 — reference data generation infrastructure (landed in `c0142b2`)
- Phase 12 Spec A — Layer B per-stage reference tests (landed in `f6fa988`)

**Independence:** This spec depends on Spec A for shared utilities in `common.rs`
(`parse_npy`, `load_conformance_model`, `load_reference_f64`, `assert_array_eq`,
`assert_quat_eq`, tolerance constants). Spec B adds new functions to `common.rs`
and creates `layer_c.rs` — no modifications to existing functions.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for multi-step trajectory
> simulation. Every algorithm, every acceptance criterion, and every test is
> in service of this goal. The MuJoCo C source code is the single source of
> truth — not the MuJoCo documentation, not intuition about "what should
> happen," not a Rust-idiomatic reinterpretation. When in doubt, read the C
> source and match its behavior exactly.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "Covers contact and non-contact scenarios" | 8 canonical models with trajectory reference data exist: pendulum, double_pendulum, contact_scenario, actuated_system, tendon_model, sensor_model, equality_model, composite_model | **Expand** — test all 8 models. Reference data exists for all 8. |
| "step-aware growing tolerances" | Two tolerance regimes needed: hinge-only models (smooth, tight tolerance) and free-joint/contact model (chaotic, wide tolerance) | In scope — specify both regimes |
| "per-step per-field diagnostics" | Need to report first divergent step, field (qpos/qvel/qacc), DOF index, magnitude | In scope |
| "divergence diagnostics map back to pipeline stages" | Spec A's Layer B per-stage comparison can be used as secondary diagnostic | In scope as optional diagnostic (documented approach, not required infrastructure) |
| "sensordata per-step comparison" | Gen script only captures qpos/qvel/qacc per step, NOT sensordata | **Drop** — no reference data exists. Note as future enhancement. |
| "contact count structural check per step" | Gen script does not capture per-step contact counts | **Drop** — no reference data exists. Note as future enhancement. |
| Session prompt: "50-100 steps" for contact | All 8 models use 100 steps (contact_scenario included); composite_model uses 200 | **Align** with actual reference data: 100/200 steps per `reference_metadata.json` |

**Final scope:**

1. Trajectory comparison infrastructure (`TrajectoryComparison` helper + step-aware tolerance)
2. Non-contact trajectory tests (pendulum, double_pendulum) — 100 steps, tight tolerance
3. Contact trajectory test (contact_scenario) — 100 steps, wide tolerance
4. Actuated trajectory test (actuated_system) — 100 steps, ctrl=[1.0, 0.5]
5. Subsystem trajectory tests (tendon_model, sensor_model, equality_model) — 100 steps
6. Composite trajectory test (composite_model) — 200 steps, ctrl=[1.0], full pipeline
7. Divergence diagnostic reporting on failure

---

## Problem Statement

CortenForge has extensive per-feature integration tests (57 modules, 1,216+
tests) and Layer B per-stage reference comparison (43 tests comparing single
`forward()` output against MuJoCo 3.4.0). However, no test validates
**multi-step trajectory accuracy** — i.e., whether running CortenForge for N
steps produces the same trajectory (qpos, qvel, qacc at each step) as running
MuJoCo for N steps from the same initial state.

This is a **conformance gap**. Single-step comparison (Layer B) catches
per-stage bugs but misses bugs that only manifest through error accumulation
across steps — drift in integration, compounding CRBA/RNE errors, contact
event timing sensitivity, and activation state propagation. Trajectory
comparison (Layer C) catches these by comparing the full state trajectory
end-to-end.

Layer B currently shows 28 of 43 tests `#[ignore]`d due to xipos, collision,
and constraint gaps. These same root causes will cause trajectory divergence
from step 1 for most models. Layer C tests encode the correct MuJoCo
trajectory and will be `#[ignore]`d until upstream fixes land — but the tests
must exist with correct assertions to track progress toward conformance.

---

## MuJoCo Reference

### `mj_step()` pipeline — the trajectory loop driver

`mj_step()` in `engine_forward.c` (MuJoCo 3.4.0):

```c
void mj_step(const mjModel* m, mjData* d) {
    mj_markStack(d);
    mj_forward(m, d);    // Full forward: FK → CRBA → RNE → passive →
                         //   collision → constraint → actuation → sensors
    mj_checkAcc(m, d);   // NaN/divergence check on qacc
    mj_advance(m, d);    // Integration: qvel += dt*qacc; mj_integratePos(qpos, qvel, dt)
    mj_freeStack(d);
}
```

**Capture timing in `gen_conformance_reference.py` (lines 160–168):**

```python
for i in range(nsteps):
    mujoco.mj_step(model, data)
    qpos_traj[i] = data.qpos[:model.nq].copy()
    qvel_traj[i] = data.qvel[:model.nv].copy()
    qacc_traj[i] = data.qacc[:model.nv].copy()
```

After `mj_step()` returns:
- `qpos[step]` = integrated position (post-advance state)
- `qvel[step]` = integrated velocity (post-advance state)
- `qacc[step]` = acceleration computed by `mj_forward()` BEFORE advance.
  `mj_advance()` reads qacc to update qvel but does not write to qacc.
  The qacc at capture is from the forward pass at the state that existed
  at the START of this step.

### `mj_Euler()` — default integrator

All canonical models use `<option timestep="0.002"/>` without specifying an
integrator, so MuJoCo defaults to `mjINT_EULER`. `mj_Euler()` in
`engine_forward.c`:

```c
// Semi-implicit Euler: update velocity first, then position
qvel += dt * qacc;
mj_integratePos(m, d->qpos, d->qvel, dt);
```

CortenForge's Euler integration in `forward/mod.rs` → `integrate/mod.rs`
follows the same semi-implicit pattern: velocity update, then position
integration via `mj_integratePos()`.

### `mj_integratePos()` — quaternion-aware position integration

For hinge/slide joints: `qpos += dt * qvel` (scalar).

For free joints: translational components use `qpos[0..3] += dt * qvel[0..3]`.
Rotational quaternion (`qpos[3..7]`) is integrated on SO(3) using angular
velocity and renormalized. MuJoCo normalizes the quaternion after integration
to prevent drift. CortenForge does the same (quaternion normalization in
`integrate/mod.rs`).

### `mj_checkAcc()` — acceleration validation

Checks `qacc` for NaN/Inf after forward. On detection: logs warning, resets
state, re-runs `forward()` to recompute all derived quantities. Both MuJoCo
and CortenForge implement this check.

### Ctrl persistence

MuJoCo keeps `data.ctrl[i]` at the set value until explicitly overwritten.
The gen script sets ctrl once before the stepping loop (line 156–158) and
calls `mj_step()` N times without modifying ctrl. The Rust test must do the
same: set ctrl once, then step N times.

Ctrl values per model (from `reference_metadata.json`):
- `actuated_system`: ctrl = [1.0, 0.5] (motor + position servo)
- `composite_model`: ctrl = [1.0] (motor)
- All others: ctrl = null (nu=0 or ctrl stays at default zero)

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Step pipeline | `mj_step()`: forward → checkAcc → advance | `Data::step()`: forward → check_acc → integrate. Same order. |
| Default integrator | Euler (`mjINT_EULER`) when no integrator specified | Euler when no integrator in MJCF. Same default. |
| Euler integration | Semi-implicit: qvel += dt*qacc; integratePos(qpos, qvel, dt) | Same semi-implicit pattern in `integrate/mod.rs` |
| Free joint quaternion | `mj_integratePos()` integrates on SO(3), normalizes after | Same — quaternion integration + normalization in integrate |
| qacc after step | From forward pass (pre-advance). Advance does not write qacc. | Same — qacc is from forward, integrate does not overwrite. |
| ctrl persistence | Ctrl persists across steps until overwritten | Same — `data.ctrl` is a `DVector<f64>`, persists across `step()` calls |
| Timestep | `<option timestep="0.002"/>` for all canonical models | Parsed from MJCF. Same timestep. |
| Activation integration | `mj_next_activation()` updates `data.act` per step | Same — activation integrated in `integrate/mod.rs` before velocity update |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `qpos` | `d->qpos`: flat `mjtNum[nq]`. Free joint: `[x,y,z,w,qx,qy,qz]` | `data.qpos: DVector<f64>` (length nq). Free joint: same layout `[x,y,z,w,qx,qy,qz]` | Direct comparison. For free joint quaternion (indices 3..7): use sign-aware comparison (q ≡ -q). Positional indices 0..3: direct comparison. |
| `qvel` | `d->qvel`: flat `mjtNum[nv]`. Always length nv. | `data.qvel: DVector<f64>` (length nv) | Direct comparison, all DOFs. |
| `qacc` | `d->qacc`: flat `mjtNum[nv]`. Post-forward, pre-advance. | `data.qacc: DVector<f64>` (length nv). Post-forward, pre-integrate. Same semantics. | Direct comparison, all DOFs. |
| `ctrl` | `d->ctrl`: flat `mjtNum[nu]`. Set once, persists. | `data.ctrl: DVector<f64>` (length nu). `data.ctrl[i] = val`. | Direct port — no translation needed. |
| Step call | `mj_step(model, data)` — mutates data in place | `data.step(&model)` — mutates data in place | Direct port. |
| Reference file | `{model}_trajectory_{field}.npy`, shape (nsteps, nq/nv) | Loaded by `parse_npy()` into `(shape, Vec<f64>)` | Access step `s`, DOF `d`: `data[s * ndof + d]` where ndof = nq (qpos) or nv (qvel/qacc). |

---

## Architecture Decisions

### AD-1: Sweep-then-report vs panic-on-first-divergence

**Problem:** When a trajectory diverges, should the comparison panic at the
first mismatch or continue sweeping all steps to collect full diagnostics?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Panic on first divergence | Simple. Standard `assert!`. Early exit. | Only reports first failure — can't see if divergence is consistent or sporadic. |
| 2 | Full sweep, then report summary | Rich diagnostics. Shows first divergent step, worst-case step, accumulated error. | More complex. Runs full trajectory even when early divergence detected. |
| 3 | Sweep with early exit threshold | Stop after divergence exceeds 10× tolerance to avoid wasting time on clearly-diverged trajectories | Best of both — rich diagnostics for borderline cases, fast exit for gross failures |

**Chosen:** Option 2 — Full sweep, then report. The trajectory comparison
helper collects all divergences and reports a summary at the end. Rationale:
(a) these tests will mostly be `#[ignore]`d, so runtime cost is irrelevant;
(b) when un-ignored after fixes, the full diagnostic sweep helps identify
whether the fix resolved the root cause or just shifted the divergence point;
(c) implementation cost is low (~30 lines of bookkeeping).

### AD-2: Per-field vs uniform tolerance per step

**Problem:** Should all three fields (qpos, qvel, qacc) share the same
step-aware tolerance, or should each field have its own base tolerance?

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Uniform tolerance for all fields | Simpler. One tolerance formula. | qacc includes constraint solver output (iterative, ~1e-4 convergence), while qpos/qvel are integrated (tighter). Uniform tolerance must be as wide as the widest field. |
| 2 | Per-field base tolerance | Each field uses its algorithmically appropriate tolerance as the base. qacc tolerance wider than qpos/qvel. | More constants. Slightly more complex. |

**Chosen:** Option 2 — Per-field base tolerance. Rationale: qacc is the direct
output of the constraint solver (iterative, convergence at ~1e-4) + RNE
(accumulated FP at ~1e-10). Using TOL_INTEGRATION (1e-8) for qpos/qvel and
a wider tolerance for qacc avoids false positives from solver convergence
differences. The growth rate is shared across fields since all fields
accumulate at the same rate once the base is established.

---

## Specification

### S1. Trajectory comparison infrastructure

**File:** `sim/L0/tests/mujoco_conformance/common.rs` (additive), `sim/L0/tests/mujoco_conformance/mod.rs` (add `mod layer_c;`)
**MuJoCo equivalent:** N/A — test infrastructure, not simulation code
**Design decision:** A single `compare_trajectory()` function handles all 8
models. It takes the model name, ctrl values, step count, tolerance regime,
and free-joint flag as parameters. This avoids duplicating the step loop and
diagnostic reporting across 8 tests. The function returns nothing (panics on
failure via `assert!` at the end after collecting all divergences).

**Step-aware tolerance function:**

```rust
/// Compute step-aware tolerance: tol(step) = base * (1.0 + step as f64 * growth)
pub fn step_tolerance(base: f64, growth: f64, step: usize) -> f64 {
    base * (1.0 + step as f64 * growth)
}
```

**Two tolerance regimes:**

```rust
/// Hinge-only models: smooth dynamics, tight tolerance.
/// Base = TOL_INTEGRATION (1e-8), growth = 0.01 per step.
/// At step 100: tol = 1e-8 * (1 + 100*0.01) = 2e-8.
pub const TRAJ_BASE_SMOOTH: f64 = 1e-8;
pub const TRAJ_GROWTH_SMOOTH: f64 = 0.01;

/// Contact/free-joint models: chaotic sensitivity, wide tolerance.
/// Base = 1e-6, growth = 0.05 per step.
/// At step 100: tol = 1e-6 * (1 + 100*0.05) = 6e-6.
pub const TRAJ_BASE_CHAOTIC: f64 = 1e-6;
pub const TRAJ_GROWTH_CHAOTIC: f64 = 0.05;

/// qacc base tolerance multiplier: qacc includes constraint solver output
/// (iterative convergence at ~1e-4) so the base tolerance is wider than
/// qpos/qvel. Multiply the regime's base by this factor for qacc.
pub const TRAJ_QACC_FACTOR: f64 = 1e4;
// Smooth qacc base: 1e-8 * 1e4 = 1e-4
// Chaotic qacc base: 1e-6 * 1e4 = 1e-2
```

Tolerance derivation:
- **Smooth base (1e-8):** Matches `TOL_INTEGRATION` — the single-step
  integration tolerance from Layer B. For smooth dynamics (no contacts, no
  constraint solver), FP error accumulates linearly through the semi-implicit
  Euler integrator.
- **Smooth growth (0.01):** Linear FP accumulation in smooth dynamics. At
  100 steps, total tolerance is 2× the base — conservative for well-behaved
  systems.
- **Chaotic base (1e-6):** Contact activation/deactivation introduces
  discontinuous forces. Constraint solver is iterative with convergence at
  ~1e-4. The wider base accounts for this from step 1.
- **Chaotic growth (0.05):** Contact events cause cascading divergence.
  At 100 steps, tolerance grows to 6× the base.
- **qacc factor (1e4):** qacc = M⁻¹(qfrc_total). The constraint solver
  contributes at ~1e-4 to forces, which propagate through M⁻¹ to qacc.
  For smooth models the qacc base becomes 1e-4 (matching TOL_CONSTRAINT).
  For chaotic models the qacc base becomes 1e-2 (very wide — contact forces
  dominate).

**Divergence diagnostic struct:**

```rust
/// Record of a single trajectory divergence.
struct TrajectoryDivergence {
    step: usize,
    field: &'static str,  // "qpos", "qvel", or "qacc"
    dof: usize,
    expected: f64,
    actual: f64,
    diff: f64,
    tol: f64,
}
```

**Trajectory comparison function:**

```rust
/// Compare CortenForge trajectory against MuJoCo reference.
///
/// Runs `data.step(&model)` for `nsteps` times, comparing qpos/qvel/qacc
/// at each step against reference .npy data.
///
/// Arguments:
/// - `model_name`: canonical model name (e.g., "pendulum")
/// - `ctrl_values`: ctrl inputs to set before stepping (empty slice for unactuated)
/// - `nsteps`: number of steps (must match reference data shape[0])
/// - `base_tol`: base tolerance for qpos/qvel
/// - `growth`: tolerance growth rate per step
/// - `has_free_joint`: if true, qpos[3..7] uses quaternion sign-aware comparison
///
/// Panics with full diagnostic report if any divergence exceeds tolerance.
fn compare_trajectory(
    model_name: &str,
    ctrl_values: &[f64],
    nsteps: usize,
    base_tol: f64,
    growth: f64,
    has_free_joint: bool,
) {
    let (model, mut data) = common::load_conformance_model(model_name);

    // Set ctrl once before stepping (matches gen script behavior)
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }

    // Load reference trajectories
    let (qpos_shape, ref_qpos) = common::load_reference_f64(
        model_name, "trajectory", "qpos");
    let (qvel_shape, ref_qvel) = common::load_reference_f64(
        model_name, "trajectory", "qvel");
    let (_qacc_shape, ref_qacc) = common::load_reference_f64(
        model_name, "trajectory", "qacc");

    // Validate shapes
    let nq = qpos_shape[1];
    let nv = qvel_shape[1];
    assert_eq!(qpos_shape[0], nsteps,
        "[{model_name}] trajectory: qpos has {} steps, expected {nsteps}",
        qpos_shape[0]);
    assert_eq!(qvel_shape[0], nsteps,
        "[{model_name}] trajectory: qvel has {} steps, expected {nsteps}",
        qvel_shape[0]);
    assert_eq!(nq, model.nq,
        "[{model_name}] trajectory: nq mismatch");
    assert_eq!(nv, model.nv,
        "[{model_name}] trajectory: nv mismatch");

    let qacc_base = base_tol * TRAJ_QACC_FACTOR;
    let mut divergences: Vec<TrajectoryDivergence> = Vec::new();

    // Step loop: step N times, compare at each step
    for step in 0..nsteps {
        data.step(&model).unwrap_or_else(|e|
            panic!("[{model_name}] step {step} failed: {e}"));

        let tol_qpos = common::step_tolerance(base_tol, growth, step);
        let tol_qvel = common::step_tolerance(base_tol, growth, step);
        let tol_qacc = common::step_tolerance(qacc_base, growth, step);

        // Compare qpos
        for d in 0..nq {
            let expected = ref_qpos[step * nq + d];
            let actual = data.qpos[d];

            // Free joint quaternion: indices 3..7 need sign-aware comparison
            if has_free_joint && (3..7).contains(&d) {
                // Defer to per-quaternion comparison below
                continue;
            }

            let diff = (expected - actual).abs();
            if diff > tol_qpos {
                divergences.push(TrajectoryDivergence {
                    step, field: "qpos", dof: d,
                    expected, actual, diff, tol: tol_qpos,
                });
            }
        }

        // Free joint quaternion sign-aware comparison for qpos[3..7]
        if has_free_joint && nq >= 7 {
            let ref_q = [
                ref_qpos[step * nq + 3],
                ref_qpos[step * nq + 4],
                ref_qpos[step * nq + 5],
                ref_qpos[step * nq + 6],
            ];
            let cf_q = [
                data.qpos[3],
                data.qpos[4],
                data.qpos[5],
                data.qpos[6],
            ];
            let dist_pos = (0..4).map(|k| (cf_q[k] - ref_q[k]).abs())
                .fold(0.0f64, f64::max);
            let dist_neg = (0..4).map(|k| (cf_q[k] + ref_q[k]).abs())
                .fold(0.0f64, f64::max);
            let dist = dist_pos.min(dist_neg);
            if dist > tol_qpos {
                divergences.push(TrajectoryDivergence {
                    step, field: "qpos (quat)",
                    dof: 3, // quaternion block starts at index 3
                    expected: ref_q[0], actual: cf_q[0],
                    diff: dist, tol: tol_qpos,
                });
            }
        }

        // Compare qvel
        for d in 0..nv {
            let expected = ref_qvel[step * nv + d];
            let actual = data.qvel[d];
            let diff = (expected - actual).abs();
            if diff > tol_qvel {
                divergences.push(TrajectoryDivergence {
                    step, field: "qvel", dof: d,
                    expected, actual, diff, tol: tol_qvel,
                });
            }
        }

        // Compare qacc
        for d in 0..nv {
            let expected = ref_qacc[step * nv + d];
            let actual = data.qacc[d];
            let diff = (expected - actual).abs();
            if diff > tol_qacc {
                divergences.push(TrajectoryDivergence {
                    step, field: "qacc", dof: d,
                    expected, actual, diff, tol: tol_qacc,
                });
            }
        }
    }

    // Report all divergences
    if !divergences.is_empty() {
        let first = &divergences[0];
        let worst = divergences.iter()
            .max_by(|a, b| (a.diff / a.tol).partial_cmp(&(b.diff / b.tol))
                .unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        let n_steps_clean = first.step;

        let mut msg = format!(
            "\n[{model_name}] TRAJECTORY DIVERGENCE — {n_steps_clean}/{nsteps} \
             steps matched before first divergence.\n\n"
        );
        msg += &format!(
            "  First divergence: step {}, {}.dof[{}]: \
             expected {:.15e}, got {:.15e}, diff {:.3e}, tol {:.3e}\n",
            first.step, first.field, first.dof,
            first.expected, first.actual, first.diff, first.tol,
        );
        msg += &format!(
            "  Worst divergence: step {}, {}.dof[{}]: \
             expected {:.15e}, got {:.15e}, diff {:.3e}, tol {:.3e} ({:.1}× tol)\n",
            worst.step, worst.field, worst.dof,
            worst.expected, worst.actual, worst.diff, worst.tol,
            worst.diff / worst.tol,
        );
        msg += &format!(
            "  Total divergences: {} across {} steps\n",
            divergences.len(), nsteps,
        );
        panic!("{msg}");
    }
}
```

**AC coverage:** AC1 (infrastructure compiles), AC10 (diagnostic format).

### S2. Non-contact trajectory tests

**File:** `sim/L0/tests/mujoco_conformance/layer_c.rs`
**MuJoCo equivalent:** `mj_step()` × 100 on pendulum/double_pendulum
**Design decision:** These models have the simplest dynamics — single and
double hinge joints with gravity, no contacts, no actuators. Both use smooth
tolerance regime. If these diverge, there is a fundamental FK/CRBA/RNE/
integration bug. Both models have xipos-affected CRBA/RNE in Layer B
(all fail except contact_scenario), so trajectory divergence from step 1
is expected.

```rust
#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_pendulum() {
    compare_trajectory(
        "pendulum",
        &[],      // no ctrl
        100,      // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false,    // no free joint
    );
}

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_double_pendulum() {
    compare_trajectory(
        "double_pendulum",
        &[],      // no ctrl
        100,      // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false,    // no free joint
    );
}
```

**Models:** pendulum (nq=1, nv=1), double_pendulum (nq=2, nv=2).

**Expected outcome:** Both `#[ignore]`d. CRBA and RNE fail in Layer B for
these models (xipos not computed → wrong mass matrix → wrong qacc). Trajectory
diverges from step 1. `#[ignore]` comment cites "CRBA/RNE xipos cascade."

**AC coverage:** AC2 (pendulum), AC3 (double_pendulum).

### S3. Contact trajectory test

**File:** `sim/L0/tests/mujoco_conformance/layer_c.rs`
**MuJoCo equivalent:** `mj_step()` × 100 on contact_scenario
**Design decision:** contact_scenario is unique — it's the only model where
FK, CRBA, and RNE all pass in Layer B. This model has a free joint (nq=7,
nv=6). Divergence comes from collision position convention difference and
constraint Jacobian assembly, not from CRBA/RNE. Trajectory may match for
early steps when the sphere is in free flight, then diverge when constraint
forces contribute to qacc. Uses chaotic tolerance regime.

```rust
#[test]
#[ignore] // CONFORMANCE GAP: constraint forces wrong — contact.pos convention + efc_J assembly — Phase 3 collision/constraint
fn layer_c_trajectory_contact_scenario() {
    compare_trajectory(
        "contact_scenario",
        &[],      // no ctrl
        100,      // steps
        TRAJ_BASE_CHAOTIC,
        TRAJ_GROWTH_CHAOTIC,
        true,     // has free joint — qpos[3..7] quaternion
    );
}
```

**Free joint quaternion handling:** qpos has nq=7: indices 0..3 are
translational position (direct comparison), indices 3..7 are quaternion
[w, x, y, z] (sign-aware comparison — q ≡ -q). The `compare_trajectory()`
helper's `has_free_joint` flag enables this per-step.

**Expected outcome:** `#[ignore]`d. Layer B collision and constraint tests
both fail for contact_scenario. Even though FK/CRBA/RNE pass, the wrong
constraint forces will cause trajectory divergence. The divergent step may
be > 0 if the sphere is in free flight initially (constraint forces = 0
until contact activates). `#[ignore]` comment distinguishes this from
xipos-affected models.

**AC coverage:** AC4 (contact_scenario), AC9 (free joint quaternion handling).

### S4. Actuated trajectory test

**File:** `sim/L0/tests/mujoco_conformance/layer_c.rs`
**MuJoCo equivalent:** `mj_step()` × 100 on actuated_system with ctrl=[1.0, 0.5]
**Design decision:** Tests the ctrl → activation → force → dynamics →
integration chain. ctrl is set once before the step loop and persists
across all 100 steps (matching the gen script). The model has nu=2, na=1
(position servo has activation dynamics). Uses smooth tolerance regime
(hinge joints only, no contacts).

**CRITICAL: ctrl values must match exactly.** The gen script sets
`ctrl = [1.0, 0.5]` — these are exact IEEE 754 constants, no transcendental
functions, no floating-point ambiguity between Python and Rust.

```rust
#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_actuated_system() {
    compare_trajectory(
        "actuated_system",
        &[1.0, 0.5],  // motor + position servo
        100,           // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false,         // no free joint
    );
}
```

**Expected outcome:** `#[ignore]`d. CRBA/RNE fail in Layer B (xipos cascade).
Layer B actuation tests pass for this model, but wrong mass matrix and bias
forces dominate qacc. `#[ignore]` comment cites xipos, not actuation.

**AC coverage:** AC5 (actuated_system), AC8 (ctrl persistence).

### S5. Subsystem trajectory tests

**File:** `sim/L0/tests/mujoco_conformance/layer_c.rs`
**MuJoCo equivalent:** `mj_step()` × 100 on tendon_model, sensor_model, equality_model
**Design decision:** These models exercise specific subsystems (tendons,
sensors, equality constraints) that affect dynamics through passive forces,
constraint assembly, or don't affect dynamics directly (sensors are
observation-only). All use smooth tolerance regime (hinge joints only).

```rust
#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_tendon_model() {
    compare_trajectory(
        "tendon_model",
        &[],      // no ctrl
        100,      // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false,    // no free joint
    );
}

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 — CRBA/RNE xipos cascade — Phase 1 FK
fn layer_c_trajectory_sensor_model() {
    compare_trajectory(
        "sensor_model",
        &[],      // no ctrl
        100,      // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false,    // no free joint
    );
}

#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 + constraint Jacobian wrong — Phase 1 FK + Phase 3 constraint
fn layer_c_trajectory_equality_model() {
    compare_trajectory(
        "equality_model",
        &[],      // no ctrl
        100,      // steps
        TRAJ_BASE_SMOOTH,
        TRAJ_GROWTH_SMOOTH,
        false,    // no free joint
    );
}
```

**Expected outcomes:**
- tendon_model: `#[ignore]`d — CRBA/RNE xipos cascade. Layer B tendon tests
  pass (geometric computation unaffected by xipos), but wrong mass matrix
  dominates dynamics.
- sensor_model: `#[ignore]`d — CRBA/RNE xipos cascade. Sensors don't affect
  dynamics (observation-only), so trajectory divergence is purely from CRBA/RNE.
- equality_model: `#[ignore]`d — CRBA/RNE xipos cascade AND constraint
  Jacobian assembly divergence (dual cause). `#[ignore]` comment names both.

**AC coverage:** AC6 (tendon_model), AC7 (sensor_model, equality_model).

### S6. Composite trajectory test

**File:** `sim/L0/tests/mujoco_conformance/layer_c.rs`
**MuJoCo equivalent:** `mj_step()` × 200 on composite_model with ctrl=[1.0]
**Design decision:** The "does the whole thing work" test. composite_model
exercises contacts + actuators + sensors + tendons combined (nq=4, nv=4,
nu=1, ntendon=1, nsensordata=7). Uses chaotic tolerance regime because the
model has contacts. 200 steps (not 100) provides the longest trajectory in
the suite, maximizing divergence detection window.

```rust
#[test]
#[ignore] // CONFORMANCE GAP: qacc wrong from step 1 + constraint wrong — Phase 1 FK + Phase 3 collision/constraint
fn layer_c_trajectory_composite_model() {
    compare_trajectory(
        "composite_model",
        &[1.0],    // motor
        200,       // steps — longest trajectory
        TRAJ_BASE_CHAOTIC,
        TRAJ_GROWTH_CHAOTIC,
        false,     // no free joint (4 hinge joints)
    );
}
```

**Expected outcome:** `#[ignore]`d. CRBA/RNE xipos cascade + collision/
constraint divergence. Multiple root causes compound. `#[ignore]` comment
names both FK and collision/constraint as causes.

**AC coverage:** AC5 (composite full pipeline, long trajectory).

---

## Acceptance Criteria

### AC1: Infrastructure compiles *(code review)*

Layer C infrastructure is complete: `mod layer_c;` added to `mod.rs`,
`common.rs` has `step_tolerance()` + tolerance constants + `TrajectoryDivergence`
struct, `layer_c.rs` has `compare_trajectory()` function. `cargo test
-p sim-conformance-tests --test mujoco_conformance` compiles without errors.

### AC2: Pendulum trajectory *(runtime test — MuJoCo-verified)*

**Given:** pendulum model at qpos0, no ctrl
**After:** 100 × `data.step(&model)`
**Assert:** qpos, qvel match MuJoCo reference within step-aware tolerance
(base 1e-8, growth 0.01). qacc within qacc tolerance (base 1e-4, growth 0.01).
**Field:** `data.qpos`, `data.qvel`, `data.qacc` vs `pendulum_trajectory_*.npy`

*Expected: `#[ignore]`d — CRBA/RNE xipos cascade causes qacc divergence at step 1.*

### AC3: Double pendulum trajectory *(runtime test — MuJoCo-verified)*

**Given:** double_pendulum model at qpos0, no ctrl
**After:** 100 × `data.step(&model)`
**Assert:** qpos, qvel match MuJoCo reference within step-aware tolerance
(base 1e-8, growth 0.01). qacc within qacc tolerance (base 1e-4, growth 0.01).
**Field:** `data.qpos`, `data.qvel`, `data.qacc` vs `double_pendulum_trajectory_*.npy`

*Expected: `#[ignore]`d — CRBA/RNE xipos cascade causes qacc divergence at step 1.*

### AC4: Contact scenario trajectory *(runtime test — MuJoCo-verified)*

**Given:** contact_scenario model at qpos0, no ctrl
**After:** 100 × `data.step(&model)`
**Assert:** qpos (with sign-aware quaternion for indices 3..7), qvel match
MuJoCo reference within step-aware tolerance (base 1e-6, growth 0.05).
qacc within qacc tolerance (base 1e-2, growth 0.05).
**Field:** `data.qpos`, `data.qvel`, `data.qacc` vs `contact_scenario_trajectory_*.npy`

*Expected: `#[ignore]`d — constraint forces wrong. May match early steps if
sphere is in free flight initially.*

### AC5: Actuated system trajectory *(runtime test — MuJoCo-verified)*

**Given:** actuated_system model at qpos0, ctrl=[1.0, 0.5]
**After:** 100 × `data.step(&model)`
**Assert:** qpos, qvel match MuJoCo reference within step-aware tolerance
(base 1e-8, growth 0.01). qacc within qacc tolerance (base 1e-4, growth 0.01).
Ctrl persists across all 100 steps without modification.
**Field:** `data.qpos`, `data.qvel`, `data.qacc` vs `actuated_system_trajectory_*.npy`

*Expected: `#[ignore]`d — CRBA/RNE xipos cascade causes qacc divergence at step 1.*

### AC6: Subsystem trajectory — tendon *(runtime test — MuJoCo-verified)*

**Given:** tendon_model at qpos0, no ctrl
**After:** 100 × `data.step(&model)`
**Assert:** qpos, qvel, qacc match MuJoCo reference within smooth tolerance.
**Field:** `data.qpos`, `data.qvel`, `data.qacc` vs `tendon_model_trajectory_*.npy`

*Expected: `#[ignore]`d — CRBA/RNE xipos cascade.*

### AC7: Subsystem trajectory — sensor, equality *(runtime test — MuJoCo-verified)*

**Given:** sensor_model / equality_model at qpos0, no ctrl
**After:** 100 × `data.step(&model)` each
**Assert:** qpos, qvel, qacc match MuJoCo reference within smooth tolerance.
**Field:** `data.qpos`, `data.qvel`, `data.qacc` vs `{model}_trajectory_*.npy`

*Expected: both `#[ignore]`d — CRBA/RNE xipos cascade (+ constraint for equality_model).*

### AC8: Ctrl persistence across steps *(runtime test — MuJoCo-verified)*

**Given:** actuated_system model, ctrl=[1.0, 0.5] set once before stepping
**After:** 100 × `data.step(&model)` without modifying ctrl between steps
**Assert:** trajectory matches reference (same AC as AC5). If ctrl did not
persist, actuator forces would differ after step 1 and the trajectory would
diverge even if all pipeline stages were correct.
**Field:** Verified implicitly through AC5 — ctrl persistence is a precondition
for trajectory match.

### AC9: Free joint quaternion comparison *(runtime test — MuJoCo-verified)*

**Given:** contact_scenario model (nq=7, free joint)
**After:** 100 × `data.step(&model)`
**Assert:** qpos[3..7] compared with sign-aware logic (q ≡ -q) at each step.
qpos[0..3] compared directly. Quaternion sign flip does not cause false
divergence.
**Field:** `data.qpos[3..7]` — quaternion portion

*Verified through AC4 — contact_scenario trajectory test uses `has_free_joint=true`.*

### AC10: Diagnostic output format *(code review)*

On trajectory divergence, the diagnostic output includes:
1. Model name
2. Steps matched before first divergence (N/total)
3. First divergence: step, field, DOF, expected, actual, diff, tolerance
4. Worst divergence: step, field, DOF, expected, actual, diff, tolerance, factor (diff/tol)
5. Total divergence count

Format is specified in `compare_trajectory()` function. Matches the format in
S1 code specification.

### AC11: All trajectory tests compile and run *(runtime — CI gate)*

`cargo test -p sim-conformance-tests --test mujoco_conformance` passes.
All 8 trajectory tests are `#[ignore]`d. No compilation errors. No test
name conflicts with existing Layer A/B/D tests.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (infrastructure) | — | Code review |
| AC2 (pendulum trajectory) | T1 | Direct |
| AC3 (double_pendulum trajectory) | T2 | Direct |
| AC4 (contact_scenario trajectory) | T3 | Direct |
| AC5 (actuated trajectory + composite) | T4, T8 | Direct |
| AC6 (tendon trajectory) | T5 | Direct |
| AC7 (sensor + equality trajectory) | T6, T7 | Direct |
| AC8 (ctrl persistence) | T4 | Integration (implicit) |
| AC9 (free joint quaternion) | T3 | Edge case |
| AC10 (diagnostic format) | — | Code review |
| AC11 (CI gate) | all | Regression |

---

## Test Plan

### Tests T1–T8: Trajectory comparison (one per model)

Each test calls `compare_trajectory()` with model-specific parameters.
All tests follow the naming convention `layer_c_trajectory_{model}`.

| Test | Function name | Model | Steps | Tolerance regime | Ctrl | Free joint | AC |
|------|--------------|-------|-------|-----------------|------|------------|-----|
| T1 | `layer_c_trajectory_pendulum` | pendulum | 100 | smooth | — | no | AC2 |
| T2 | `layer_c_trajectory_double_pendulum` | double_pendulum | 100 | smooth | — | no | AC3 |
| T3 | `layer_c_trajectory_contact_scenario` | contact_scenario | 100 | chaotic | — | yes | AC4, AC9 |
| T4 | `layer_c_trajectory_actuated_system` | actuated_system | 100 | smooth | [1.0, 0.5] | no | AC5, AC8 |
| T5 | `layer_c_trajectory_tendon_model` | tendon_model | 100 | smooth | — | no | AC6 |
| T6 | `layer_c_trajectory_sensor_model` | sensor_model | 100 | smooth | — | no | AC7 |
| T7 | `layer_c_trajectory_equality_model` | equality_model | 100 | smooth | — | no | AC7 |
| T8 | `layer_c_trajectory_composite_model` | composite_model | 200 | chaotic | [1.0] | no | AC5 |

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Free joint quaternion sign ambiguity (q ≡ -q) | contact_scenario has qpos[3..7] quaternion. Without sign-aware comparison, q and -q (same orientation) would cause false divergence at every step. | T3 | AC9 |
| Ctrl persistence across 100+ steps | actuated_system and composite_model set ctrl once. If ctrl vector is zeroed or overwritten between steps, actuator forces disappear and trajectory diverges after step 1 even with correct pipeline stages. | T4, T8 | AC8 |
| Long trajectory (200 steps) | composite_model runs 200 steps — 2× the other models. Longer trajectory amplifies accumulation effects and tests tolerance growth formula at larger step counts. At step 200: smooth tol = 1e-8 × 3.0 = 3e-8; chaotic tol = 1e-6 × 11.0 = 1.1e-5. | T8 | AC5 |
| qacc includes constraint solver output | qacc is M⁻¹(qfrc_total) which includes iterative constraint solver forces (~1e-4 convergence). Using qpos/qvel tolerance for qacc would cause false failures from solver convergence differences. qacc needs separate wider tolerance. | T3, T7, T8 | AC4, AC7, AC5 |
| Zero ctrl for unactuated models | 6 of 8 models have nu=0 or leave ctrl at default zero. Empty ctrl slice must work correctly (no index-out-of-bounds, no actuator force pollution). | T1, T2, T3, T5, T6, T7 | AC2, AC3, AC4, AC6, AC7 |
| Step-1 vs step-N>1 divergence patterns | 7 models diverge at step 1 (xipos cascade). contact_scenario may diverge at step N>1 (constraint forces). The diagnostic must report the actual first divergent step to distinguish root causes. | All | AC10 |

### Supplementary Tests

None. Every test maps 1:1 to an (AC, model) pair via the traceability matrix.
No orphan tests.

---

## Reference File Coverage Matrix

Every trajectory reference file maps to exactly one test.

| Reference file | Test | AC |
|---------------|------|-----|
| `pendulum_trajectory_qpos.npy` | T1 (`layer_c_trajectory_pendulum`) | AC2 |
| `pendulum_trajectory_qvel.npy` | T1 | AC2 |
| `pendulum_trajectory_qacc.npy` | T1 | AC2 |
| `double_pendulum_trajectory_qpos.npy` | T2 (`layer_c_trajectory_double_pendulum`) | AC3 |
| `double_pendulum_trajectory_qvel.npy` | T2 | AC3 |
| `double_pendulum_trajectory_qacc.npy` | T2 | AC3 |
| `contact_scenario_trajectory_qpos.npy` | T3 (`layer_c_trajectory_contact_scenario`) | AC4 |
| `contact_scenario_trajectory_qvel.npy` | T3 | AC4 |
| `contact_scenario_trajectory_qacc.npy` | T3 | AC4 |
| `actuated_system_trajectory_qpos.npy` | T4 (`layer_c_trajectory_actuated_system`) | AC5 |
| `actuated_system_trajectory_qvel.npy` | T4 | AC5 |
| `actuated_system_trajectory_qacc.npy` | T4 | AC5 |
| `tendon_model_trajectory_qpos.npy` | T5 (`layer_c_trajectory_tendon_model`) | AC6 |
| `tendon_model_trajectory_qvel.npy` | T5 | AC6 |
| `tendon_model_trajectory_qacc.npy` | T5 | AC6 |
| `sensor_model_trajectory_qpos.npy` | T6 (`layer_c_trajectory_sensor_model`) | AC7 |
| `sensor_model_trajectory_qvel.npy` | T6 | AC7 |
| `sensor_model_trajectory_qacc.npy` | T6 | AC7 |
| `equality_model_trajectory_qpos.npy` | T7 (`layer_c_trajectory_equality_model`) | AC7 |
| `equality_model_trajectory_qvel.npy` | T7 | AC7 |
| `equality_model_trajectory_qacc.npy` | T7 | AC7 |
| `composite_model_trajectory_qpos.npy` | T8 (`layer_c_trajectory_composite_model`) | AC5 |
| `composite_model_trajectory_qvel.npy` | T8 | AC5 |
| `composite_model_trajectory_qacc.npy` | T8 | AC5 |

**Total trajectory reference files covered: 24. Orphan reference files: 0.**

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| None | N/A | N/A | N/A | N/A | N/A |

This spec adds new tests only. No production code is modified. No existing
test behavior changes.

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/tests/mujoco_conformance/layer_c.rs` | New file — 8 test functions + `compare_trajectory()` helper | +250–300 |
| `sim/L0/tests/mujoco_conformance/common.rs` | Add `step_tolerance()`, trajectory tolerance constants, `TrajectoryDivergence` struct | +30–40 |
| `sim/L0/tests/mujoco_conformance/mod.rs` | Add `mod layer_c;` declaration | +1 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| All Layer A tests (13) | `layer_a.rs` | Pass (unchanged) | Separate module, no shared mutable state |
| All Layer B tests (43) | `layer_b.rs` | Pass (unchanged) | Separate module, no shared mutable state |
| All Layer D tests (15) | `layer_d.rs` | Pass (unchanged) | Separate module, no shared mutable state |
| All integration tests (57 modules) | `integration/*.rs` | Pass (unchanged) | Separate test binary |
| `common.rs` additions | `common.rs` | N/A | Additive — new functions only, no modification to existing functions |

---

## Execution Order

1. **S1: Common infrastructure** — add `step_tolerance()`, tolerance constants,
   `TrajectoryDivergence` struct to `common.rs`. Add `mod layer_c;` to `mod.rs`.
   Create empty `layer_c.rs` with `use super::common;`. → compile check

2. **S2: Non-contact trajectory tests** (T1–T2) — simplest models, validates
   the step loop and comparison infrastructure. → `cargo test -p sim-conformance-tests --test mujoco_conformance`

3. **S3: Contact trajectory test** (T3) — validates free-joint quaternion
   comparison. → test

4. **S4: Actuated trajectory test** (T4) — validates ctrl persistence. → test

5. **S5: Subsystem trajectory tests** (T5–T7) — tendon, sensor, equality. → test

6. **S6: Composite trajectory test** (T8) — full pipeline, longest trajectory.
   Most integrative test — saved for last. → test

Priority order: infrastructure → simple models → complex models → full pipeline.

---

## Out of Scope

- **Per-step sensordata trajectory comparison** — gen script only captures
  qpos/qvel/qacc per step. No trajectory reference data exists for sensordata.
  Deferred as future enhancement to `gen_conformance_reference.py`. Conformance
  impact: minimal — sensordata is observation-only (does not affect dynamics).

- **Per-step contact count structural check** — gen script does not capture
  per-step contact counts. Deferred. Conformance impact: low — contact count
  changes are observable through trajectory divergence in qpos/qvel/qacc.

- **Layer B cross-reference diagnostic** — running per-stage Layer B comparison
  at the divergent step's state as a secondary diagnostic. Adds complexity for
  marginal benefit when `#[ignore]` comments already classify root causes.
  Documented as approach in rubric EGT-3 analysis; not implemented. Conformance
  impact: none — diagnostic aid, not conformance validation.

- **RK4 trajectory tests** — all canonical models default to Euler. RK4
  trajectory comparison requires models with explicit `integrator="RK4"` and
  separate reference data. Deferred. Conformance impact: RK4 is tested in
  existing integration tests (`rk4_integration.rs`).

- **Fixing upstream conformance failures** — Phase 12 writes tests, not fixes.
  xipos computation (Phase 1), collision position convention (Phase 3), and
  constraint Jacobian assembly (Phase 3) are root causes that Phase 12 tests
  expose but do not fix.

- **Flag combination trajectory tests** — testing trajectories with specific
  disable/enable flags (e.g., DISABLE_GRAVITY trajectory). Deferred.
  Conformance impact: low — single-flag golden tests exist in DT-97.
