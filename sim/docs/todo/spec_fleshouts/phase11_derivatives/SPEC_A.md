# Analytical Position Derivatives (`mjd_smooth_pos`) — Spec

**Status:** Draft
**Phase:** Roadmap Phase 11 — Derivatives
**Effort:** L
**MuJoCo ref:** No direct MuJoCo equivalent — see split mandate below
**MuJoCo version:** 3.x (current main branch, verified 2026-03-08)
**Test baseline:** 3,600+ sim domain tests
**Prerequisites:**
- Phase A–D derivative infrastructure (landed, `derivatives.rs` ~4,276 lines)
- DT-52 `mjd_sub_quat` (landed in `9feebfe`) — needed for Ball/Free spring position derivatives
- DT-53 `forward_skip` + `MjStage` enum (landed in `c68d9cb`) — used by hybrid path
- DT-54 muscle velocity derivatives (landed in `9feebfe`) — validates parallel structure

**Independence:** This spec is independent of Spec B (DT-47 Sensor Derivatives)
per the umbrella dependency graph. Shared file: `derivatives.rs` — Spec A adds
`mjd_smooth_pos()` and modifies `mjd_transition_hybrid()` position columns.
Spec B later extends the FD perturbation loop for sensor C/D matrices. No
conflict — Spec A touches position column assembly, Spec B touches sensor
measurement recording.

> **Conformance mandate (split):** `mjd_smooth_pos` does NOT exist in MuJoCo.
> MuJoCo computes ALL transition derivative position columns via finite
> differencing (`mjd_stepFD` in `engine_derivative_fd.c`). This spec is a
> **CortenForge extension** with a split conformance posture:
>
> 1. **Conformance subset:** The final transition A matrix (position columns)
>    MUST produce numerically equivalent results to the pure FD transition
>    derivatives — which match MuJoCo's output. The analytical computation
>    is an implementation strategy; the numerical result must be equivalent.
>    Validated by comparing analytical position columns against FD position
>    columns within `1e-6` relative tolerance on contact-free models.
>
> 2. **Extension subset:** The `mjd_smooth_pos()` function, `Data.qDeriv_pos`
>    field, and the chain-rule algorithm through passive/actuator/RNE are
>    CortenForge extensions. No MuJoCo C source to port — mathematical
>    correctness is validated against FD. The MuJoCo velocity derivative
>    functions (`mjd_passive_vel`, `mjd_actuator_vel`, `mjd_rne_vel`) serve
>    as structural templates, but the position math differs fundamentally.

---

## Scope Adjustment

Empirical verification against the MuJoCo C source discovered a critical
scope correction from the umbrella spec.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "`mjd_smooth_pos()` in `engine_derivative.c`" | **Function does NOT exist.** MuJoCo has no `mjd_smooth_pos`. All position columns use FD via `mjd_stepFD` with `mj_stepSkip(m, d, mjSTAGE_NONE, ...)`. Verified: `engine_derivative.c` contains only velocity derivative functions. `engine_derivative.h` declares `mjd_smooth_vel`, `mjd_actuator_vel`, `mjd_passive_vel`, `mjd_rne_vel_dense`, `mjd_subQuat`, `mjd_quatIntegrate` — no position derivative functions. `mujoco.h` lists only `mjd_transitionFD`, `mjd_inverseFD`, `mjd_subQuat`, `mjd_quatIntegrate`. | Reclassify as CortenForge extension. Split mandate. |
| "Three components: FK, RNE, passive" | Components are mathematically correct (chain rule through FK → forces). MuJoCo doesn't compute them analytically, but the math is standard rigid-body mechanics. Actuator position derivatives also needed. | **In scope** as extension with 3 sub-functions: `mjd_passive_pos`, `mjd_actuator_pos`, `mjd_rne_pos`. |
| "Output stored in `Data.qDeriv_pos`" | MuJoCo has no `qDeriv_pos` field. CortenForge extension. | **In scope.** New Data field (nv × nv). |
| "Integration into `mjd_transition_hybrid()`" | CortenForge's hybrid path already uses analytical velocity columns. Replacing FD position columns is a natural extension. | **In scope.** Final A matrix must match FD A matrix. |
| "≥1.5× speedup over FD position columns" | Performance goal, not conformance. | **In scope** as performance AC. |
| Actuator position derivatives | Not in original umbrella 3-component list. Actuator force depends on position through `actuator_length` → `gain(L)` and `bias(L)`. | **Added.** `mjd_actuator_pos` for position-dependent actuator forces. |

**Deferred passive sub-components** (not analytically implemented — models
with these features fall back to FD for position columns):

| Component | Why deferred | Conformance impact |
|-----------|-------------|-------------------|
| Fluid forces (`qfrc_fluid`) | Complex chain rule through `xmat` → drag coefficients. Analytical velocity derivatives exist (`mjd_fluid_vel`) but position derivatives require `∂xmat/∂qpos` chain rule through FK. | Models with non-zero fluid density/viscosity use FD position columns. |
| Gravity compensation (`qfrc_gravcomp`) | Force at `xipos[b]` through kinematic chain. Requires `∂xipos/∂qpos` FK Jacobian. | Models with `body_gravcomp ≠ 0` use FD position columns. |
| Flex bending forces (cotangent/Bridson) | Complex vertex-geometry derivatives through deformed mesh positions. | Models with flex elements use FD position columns. |
| Actuator moment-arm cross-term `(∂moment/∂qpos)·force` for Site/Body/SliderCrank transmissions | Requires FK Jacobian of site/body positions. Zero for Joint/JointInParent/Tendon transmissions. | Models with Site/Body/SliderCrank actuators use FD position columns. |
| Muscle/HillMuscle bias passive FL position derivative (`∂bias/∂L`) | Requires `muscle_passive_fl_deriv` / `hill_passive_fl_deriv` helpers (not yet implemented). Gain derivatives ARE implemented (active FL curve). | Models with Muscle/HillMuscle bias actuators use FD position columns. |

**Selective FD fallback strategy:** When a model has any deferred component,
`mjd_transition_hybrid()` falls back to FD for ALL position columns (not a
per-column mix). This is simpler and avoids partial-analytical/partial-FD
inconsistencies. The fallback is detected at the start of the hybrid function
by checking model flags (fluid density/viscosity, gravcomp bodies, flex count,
site/body/slidercrank actuators, muscle/hillmuscle bias actuators).

**Final scope:**

1. `mjd_smooth_pos(model, data)` — analytical position force derivatives
   stored in `Data.qDeriv_pos` (nv × nv). Contains
   `∂qfrc_smooth/∂qpos = ∂qfrc_passive/∂qpos + ∂qfrc_actuator/∂qpos − ∂RNEA(q,v,qacc)/∂qpos`.
2. Three sub-functions: `mjd_passive_pos`, `mjd_actuator_pos`, `mjd_rne_pos`
3. `Data.qDeriv_pos: DMatrix<f64>` (nv × nv) — new field
4. Integration into `mjd_transition_hybrid()` — replace FD position columns
   with analytical (for eligible models)
5. `IntegrationDerivatives.dqpos_dqpos` activated (remove `#[allow(dead_code)]`)
6. Selective FD fallback for models with deferred components
7. Validation: analytical position columns match FD within `1e-6` relative
   tolerance on contact-free, eligible models

---

## Problem Statement

CortenForge's `mjd_transition_hybrid()` computes velocity columns of the
transition A matrix analytically (via `mjd_smooth_vel` + integrator-specific
solve), achieving ~2× speedup over pure FD. However, **position columns
(columns 0..nv of A) still use finite differencing** — each column requires
a full `step()` call (or two for centered FD). For a model with nv = 27
(humanoid), this means 27–54 simulation steps just for position columns.

This is a **conformance-neutral efficiency gap**: the FD position columns
produce correct results (matching MuJoCo), but the computational cost
is unnecessarily high. Analytical position derivatives can replace the FD
loop with O(nbody · nv) matrix operations, eliminating the per-column
simulation steps.

The gap: `derivatives.rs:1448–1511` — the position column FD loop in
`mjd_transition_hybrid()`. After this spec, eligible models use analytical
position columns from `mjd_smooth_pos()`, reducing hybrid transition
derivative cost by an additional ~1.5–2× (total ~3–4× over pure FD).

---

## MuJoCo Reference

> **Split mandate context:** MuJoCo has no analytical position derivatives.
> This section documents (a) how MuJoCo computes position columns via FD
> (the validation oracle), and (b) the mathematical basis for the analytical
> extension (rigid-body chain rules).

### MuJoCo FD position columns (validation oracle)

**Source:** `engine_derivative_fd.c` → `mjd_stepFD()`, position column loop.

MuJoCo computes all transition derivative columns via FD:
```c
// For each position DOF i:
mj_integratePos(m, d->qpos, dpos, eps);    // tangent-space perturbation
mj_stepSkip(m, d, mjSTAGE_NONE, 1);        // full forward + integrate (skip sensors)
// measure output state, compute (y+ − y0) / eps or centered
```

Position perturbations use tangent-space integration (`mj_integratePos`)
which maps an nv-dimensional perturbation vector to nq-dimensional coordinate
space, handling quaternion joints correctly. This is the approach already
used by CortenForge's `apply_state_perturbation()` via
`mj_integrate_pos_explicit()`.

### Analytical basis: velocity derivatives as structural template

MuJoCo's `mjd_smooth_vel` (verified in `engine_derivative.c`) provides the
structural template:

```c
void mjd_smooth_vel(const mjModel* m, mjData* d, int flg_bias) {
    mju_zero(d->qDeriv, m->nD);
    mjd_actuator_vel(m, d);
    mjd_passive_vel(m, d);
    if (flg_bias) { mjd_rne_vel(m, d); }
}
```

CortenForge's `mjd_smooth_vel` (derivatives.rs:1020–1025) follows this
exactly. `mjd_smooth_pos` mirrors this pattern with position-specific math:

```rust
pub fn mjd_smooth_pos(model: &Model, data: &mut Data) {
    data.qDeriv_pos.fill(0.0);
    mjd_passive_pos(model, data);
    mjd_actuator_pos(model, data);
    mjd_rne_pos(model, data);
}
```

### Key mathematical differences: position vs velocity derivatives

Velocity derivatives are simpler because:
- Motion subspace S does NOT depend on qvel → `∂S/∂qvel = 0`
- Mass matrix M does NOT depend on qvel → `(∂M/∂qvel)·qacc = 0`
- Gravity torques are velocity-independent → `∂τ_gravity/∂qvel = 0`

Position derivatives are harder because:
- Motion subspace S depends on ancestor joint positions (body orientation
  rotates the joint axis in world frame) → `∂S/∂qpos ≠ 0`
- Mass matrix M depends on position through body transforms →
  `(∂M/∂qpos)·qacc ≠ 0` (see RNE at actual acceleration, below)
- Gravity torques depend on position through body COM → `∂τ_gravity/∂qpos ≠ 0`
- Body-frame inertias in world frame depend on position →
  `∂cinert/∂qpos ≠ 0`

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) | CortenForge (after) |
|----------|--------|-----------------------|---------------------|
| Position columns of A | FD via `mjd_stepFD` with `mj_stepSkip(mjSTAGE_NONE)` | FD via `step()` in `mjd_transition_hybrid` lines 1448–1511 | Analytical via `mjd_smooth_pos()` + chain rule (eligible models); FD fallback (ineligible models) |
| `∂qfrc_smooth/∂qpos` | Not computed (captured implicitly by FD) | Not computed | Computed analytically in `Data.qDeriv_pos` |
| `∂qfrc_constraint/∂qpos` | Captured by FD (re-runs collision detection) | Captured by FD | NOT computed analytically — same approximation as existing velocity columns (EGT-9 in rubric) |
| Transition A matrix values | Produced by pure FD | Hybrid: analytical velocity cols + FD position cols | Hybrid: analytical velocity cols + analytical position cols. Values match FD within `1e-6` on eligible contact-free models. |
| Sleeping bodies | FD captures sleeping state implicitly (step skips sleeping bodies) | Forward pass already handles sleeping; derivative functions receive post-forward-pass data | No special-casing needed. `mjd_smooth_pos` operates on data from a completed forward pass. If a body is sleeping, `data.qacc` and `data.cvel` reflect that state, and position derivatives are computed correctly from those values. |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Position derivative storage | No equivalent (FD only) | `Data.qDeriv_pos` (nv × nv dense) | CortenForge extension — no MuJoCo counterpart |
| Tangent space | `mj_integratePos` for perturbation, `mjSTAGE_NONE` for FD | `mj_integrate_pos_explicit` for perturbation | Direct port — same tangent-space approach |
| `qDeriv_pos` indexing | N/A | Row = force DOF, col = position tangent DOF (same as `qDeriv`) | Parallels `qDeriv` convention |
| SpatialVector layout | `[angular; linear]` (rotation; translation) | `[angular; linear]` (same) | Direct port — no translation needed |
| Quaternion convention | `(w, x, y, z)` | `(w, x, y, z)` (same) | Direct port |
| `subquat` output | Body-frame 3-vector (angular difference) | Body-frame 3-vector via `mjd_sub_quat` | Direct port — DT-52 matches MuJoCo |
| Constraint exclusion | FD captures everything (constraint + smooth) | Analytical computes `∂qfrc_smooth/∂qpos` only | Same approximation as velocity columns — validated by existing precedent |

---

## Architecture Decisions

### AD-1: Selective FD fallback for ineligible models

**Problem:** Some passive force components (fluid, gravcomp, flex bending)
and actuator transmission types (Site/Body/SliderCrank moment-arm cross-terms)
have complex position derivatives that would require significant FK Jacobian
infrastructure. Implementing all analytically would be a massive effort with
diminishing returns. But omitting them silently produces incorrect position
columns for models with those features.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Implement all analytically | Complete coverage | Massive scope increase (FK Jacobians for fluid, gravcomp, flex bending, moment-arm geometry). Research-grade complexity for flex bending. |
| 2 | Selective FD fallback (whole-model) | Clean boundary: eligible models get full speedup, ineligible models get correct FD results. Simple to implement. | No speedup for ineligible models. |
| 3 | Per-column mix (analytical where possible, FD for deferred terms) | Partial speedup for ineligible models | Complex, error-prone mixing. Risk of subtle inconsistencies. |
| 4 | Omit deferred terms, accept reduced accuracy | Simplest implementation | Silent conformance regression for affected models. Violates "A-grade or it doesn't ship." |

**Chosen:** Option 2 — selective FD fallback at model level. The eligibility
check is O(1) (inspect model flags at the start of `mjd_transition_hybrid`).
Eligible models (the common case: joint-actuated systems without fluid/gravcomp/flex)
get full analytical speedup. Ineligible models get correct FD results.

### AD-2: RNE at actual acceleration (not bias only)

**Problem:** The velocity derivative function `mjd_rne_vel` differentiates
RNEA at zero joint acceleration (bias forces only), because `M(q)` does
not depend on velocity. For position derivatives, `M(q)` DOES depend on
`q`, so `∂RNEA(q,v,0)/∂q` misses the `(∂M/∂q)·qacc` term.

**Chosen:** `mjd_rne_pos` differentiates `RNEA(q, v, qacc)` — NOT
`RNEA(q, v, 0)`. This captures the mass matrix position derivative through
the recursive structure: `(∂X_b/∂q)·cacc_full[parent]` and
`(∂S_b/∂q)·qacc_b`. Precondition: `data.qacc` must be populated (always
true after the forward pass).

---

## Specification

### S1. `mjd_passive_pos` — Passive force position derivatives

**File:** `sim/L0/core/src/derivatives.rs` (new function, ~100–150 lines)
**MuJoCo equivalent:** No direct equivalent — extension. Velocity template: `mjd_passive_vel` (lines 463–509)
**Design decision:** Mirrors `mjd_passive_vel` structure. Only computes
analytically tractable components: joint springs (hinge/slide/ball/free)
and tendon springs. Fluid, gravcomp, flex are deferred (see AD-1).

**Algorithm:**

```rust
pub(crate) fn mjd_passive_pos(model: &Model, data: &mut Data) {
    let nv = model.nv;
    if nv == 0 { return; }

    // 1. Joint spring stiffness: ∂qfrc_spring/∂qpos
    // Gated by DISABLE_SPRING (same as passive.rs:891)
    if !model.disable_spring {
        for jnt_id in 0..model.njnt {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let stiffness = model.jnt_stiffness[jnt_id];
            if stiffness == 0.0 { continue; }

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    // qfrc_spring[dof] -= k * (q - q_ref)
                    // ∂/∂qpos[dof] = -k  (diagonal)
                    data.qDeriv_pos[(dof_adr, dof_adr)] += -stiffness;
                }
                MjJointType::Ball => {
                    // qfrc_spring[dof..dof+3] -= k * subquat(q, q_ref)
                    // ∂/∂qpos = -k * ∂subquat/∂qa (3×3 tangent-space Jacobian)
                    let qpos_adr = model.jnt_qpos_adr[jnt_id];
                    let qa = [
                        data.qpos[qpos_adr], data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2], data.qpos[qpos_adr + 3],
                    ];
                    let qb = [
                        model.qpos_spring[qpos_adr], model.qpos_spring[qpos_adr + 1],
                        model.qpos_spring[qpos_adr + 2], model.qpos_spring[qpos_adr + 3],
                    ];
                    let (d_dqa, _d_dqb) = mjd_sub_quat(&qa, &qb);
                    // Accumulate: qDeriv_pos[dof..dof+3, dof..dof+3] += -k * d_dqa
                    for r in 0..3 {
                        for c in 0..3 {
                            data.qDeriv_pos[(dof_adr + r, dof_adr + c)] +=
                                -stiffness * d_dqa[(r, c)];
                        }
                    }
                }
                MjJointType::Free => {
                    // Translational part (dof..dof+3): same as Hinge/Slide (diagonal)
                    for i in 0..3 {
                        data.qDeriv_pos[(dof_adr + i, dof_adr + i)] += -stiffness;
                    }
                    // Rotational part (dof+3..dof+6): same as Ball
                    let qpos_adr = model.jnt_qpos_adr[jnt_id];
                    let qa = [
                        data.qpos[qpos_adr + 3], data.qpos[qpos_adr + 4],
                        data.qpos[qpos_adr + 5], data.qpos[qpos_adr + 6],
                    ];
                    let qb = [
                        model.qpos_spring[qpos_adr + 3], model.qpos_spring[qpos_adr + 4],
                        model.qpos_spring[qpos_adr + 5], model.qpos_spring[qpos_adr + 6],
                    ];
                    let (d_dqa, _d_dqb) = mjd_sub_quat(&qa, &qb);
                    for r in 0..3 {
                        for c in 0..3 {
                            data.qDeriv_pos[(dof_adr + 3 + r, dof_adr + 3 + c)] +=
                                -stiffness * d_dqa[(r, c)];
                        }
                    }
                }
            }
        }
    }

    // 2. Tendon spring stiffness: ∂qfrc_spring_tendon/∂qpos
    // tendon force = k * (bound - length), applied as J^T * force
    // ∂(J^T * force)/∂qpos ≈ J^T * k * J  (for fixed-geometry tendons where
    //   ∂J/∂qpos = 0 and ∂force/∂qpos = k * ∂length/∂qpos = k * J)
    // This ignores the cross-term (∂J^T/∂qpos) · force — which is zero for
    // fixed-point tendons (constant J) and small for spatial tendons.
    for t in 0..model.ntendon {
        let stiffness = model.tendon_stiffness[t];
        if stiffness == 0.0 { continue; }

        let length = data.ten_length[t];
        let lower = model.tendon_range[t].0;
        let upper = model.tendon_range[t].1;

        // Only active when outside deadband
        let active = (length < lower) || (length > upper);
        if !active { continue; }

        // ∂(J^T * k * (bound - length))/∂qpos ≈ J^T * (-k) * J
        // (negative because ∂length/∂qpos = J moves length AWAY from bound)
        let j = &data.ten_J[t];
        let scale = -stiffness;
        for r in 0..nv {
            if j[r] == 0.0 { continue; }
            for c in 0..nv {
                if j[c] == 0.0 { continue; }
                data.qDeriv_pos[(r, c)] += scale * j[r] * j[c];
            }
        }
    }
}
```

**What is NOT computed:**
- Fluid forces, gravity compensation, flex bending — deferred (AD-1)
- Implicit spring forces (ImplicitSpringDamper integrator handles these
  separately through the system matrix) — same as `mjd_passive_vel`

---

### S2. `mjd_actuator_pos` — Actuator force position derivatives

**File:** `sim/L0/core/src/derivatives.rs` (new function, ~80–120 lines)
**MuJoCo equivalent:** No direct equivalent — extension. Velocity template: `mjd_actuator_vel` (lines 532–651)
**Design decision:** Mirrors `mjd_actuator_vel` structure. The position
derivative of actuator force is through actuator length `L`:
`∂force/∂qpos = (∂gain/∂L · input + ∂bias/∂L) · ∂L/∂qpos`
where `∂L/∂qpos = moment` (the moment arm IS the length Jacobian).
Only the force chain-rule term is computed — the moment-arm cross-term
`(∂moment/∂qpos) · force` is deferred for non-Joint transmissions (AD-1).

**Algorithm:**

```rust
pub(crate) fn mjd_actuator_pos(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        // Get input (same as mjd_actuator_vel)
        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => data.ctrl[i],
            _ => data.act[model.actuator_act_adr[i]],
        };
        let length = data.actuator_length[i];

        // Compute ∂gain/∂L
        let dgain_dl = match model.actuator_gaintype[i] {
            GainType::Fixed => 0.0,     // gain = prm[0], no L dependence
            GainType::Affine => {
                // gain = prm[0] + prm[1]*L + prm[2]*V
                // ∂gain/∂L = prm[1]
                model.actuator_gainprm[i][1]
            }
            GainType::Muscle => {
                // gain = -F0 * FL(L_norm) * FV(V_norm)
                // ∂gain/∂L = -F0 * dFL/dL_norm * dL_norm/dL * FV(V_norm)
                let prm = &model.actuator_gainprm[i];
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2];
                let l0 = (lengthrange.1 - lengthrange.0)
                    / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (length - lengthrange.0)
                    / l0.max(1e-10);
                let norm_vel = data.actuator_velocity[i]
                    / (l0 * prm[6]).max(1e-10);
                let dfl_dnl = muscle_active_fl_deriv(norm_len, prm[4], prm[5]);
                let fv = muscle_gain_fv(norm_vel, prm[8]);
                let dnl_dl = 1.0 / l0.max(1e-10);
                -f0 * dfl_dnl * dnl_dl * fv
            }
            GainType::HillMuscle => {
                let prm = &model.actuator_gainprm[i];
                let f0 = prm[2];
                let optimal_fiber_length = prm[4];
                let tendon_slack_length = prm[5];
                let max_contraction_velocity = prm[6];
                let pennation_angle = prm[7];
                let cos_penn = pennation_angle.cos().max(1e-10);
                let fiber_length = (length - tendon_slack_length) / cos_penn;
                let norm_len = fiber_length
                    / optimal_fiber_length.max(1e-10);
                let norm_vel = data.actuator_velocity[i]
                    / (cos_penn * optimal_fiber_length * max_contraction_velocity)
                        .max(1e-10);
                let dfl_dnl = hill_active_fl_deriv(norm_len);
                let fv = hill_force_velocity(norm_vel);
                let dnl_dl = 1.0
                    / (cos_penn * optimal_fiber_length).max(1e-10);
                -f0 * dfl_dnl * dnl_dl * fv * cos_penn
            }
            GainType::User => continue,
        };

        // Compute ∂bias/∂L
        let dbias_dl = match model.actuator_biastype[i] {
            BiasType::None => 0.0,
            BiasType::Affine => {
                // bias = prm[0] + prm[1]*L + prm[2]*V
                // ∂bias/∂L = prm[1]
                model.actuator_biasprm[i][1]
            }
            BiasType::Muscle | BiasType::HillMuscle => {
                // Passive muscle force depends on length:
                //   Muscle: bias = -F0 * FP(L_norm)
                //   HillMuscle: bias = -F0 * FP_hill(L_norm) * cos(penn)
                // ∂bias/∂L is computable but deferred — requires
                // muscle_passive_fl_deriv / hill_passive_fl_deriv helpers.
                // Models with Muscle/HillMuscle bias actuators are excluded
                // from analytical position columns via the eligibility check
                // in S6. FD fallback produces correct results.
                0.0 // Safe: eligibility check ensures this code path is
                    // only reached when FD fallback handles position columns.
            }
            BiasType::User => continue,
        };

        // Combined: ∂force/∂L = ∂gain/∂L · input + ∂bias/∂L
        let dforce_dl = dgain_dl * input + dbias_dl;
        if dforce_dl.abs() < 1e-30 { continue; }

        // Transmission dispatch (same structure as mjd_actuator_vel)
        // ∂qfrc/∂qpos += moment · ∂force/∂L · moment^T
        //   = moment · dforce_dl · moment^T
        // (because ∂L/∂qpos = moment^T for joint/tendon transmissions)
        let gear = model.actuator_gear[i][0];
        let trnid = model.actuator_trnid[i][0];

        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint
            | ActuatorTransmission::JointInParent => {
                // moment = gear, ∂L/∂qpos[dof] = gear
                // ∂qfrc[dof]/∂qpos[dof] += gear² · ∂force/∂L
                let dof_adr = model.jnt_dof_adr[trnid];
                data.qDeriv_pos[(dof_adr, dof_adr)] +=
                    gear * gear * dforce_dl;
            }
            ActuatorTransmission::Tendon => {
                // moment = gear · J^T, ∂L/∂qpos = gear · J
                // ∂qfrc/∂qpos += gear² · ∂force/∂L · J^T · J
                let j = &data.ten_J[trnid];
                let scale = gear * gear * dforce_dl;
                for r in 0..model.nv {
                    if j[r] == 0.0 { continue; }
                    for c in 0..model.nv {
                        if j[c] == 0.0 { continue; }
                        data.qDeriv_pos[(r, c)] += scale * j[r] * j[c];
                    }
                }
            }
            ActuatorTransmission::Site
            | ActuatorTransmission::Body
            | ActuatorTransmission::SliderCrank => {
                // Moment arm depends on FK — cross-term deferred.
                // Force chain-rule term: moment · dforce_dl · moment^T
                let moment = &data.actuator_moment[i];
                for r in 0..model.nv {
                    if moment[r] == 0.0 { continue; }
                    for c in 0..model.nv {
                        if moment[c] == 0.0 { continue; }
                        data.qDeriv_pos[(r, c)] +=
                            dforce_dl * moment[r] * moment[c];
                    }
                }
                // NOTE: The cross-term (∂moment/∂qpos) · force is NOT
                // computed. This means the analytical position derivative
                // is approximate for these transmission types. For full
                // accuracy, models with these transmissions should use
                // the FD fallback (checked in AD-1 eligibility).
            }
        }
    }
}
```

**Helper functions needed:**
- `muscle_active_fl_deriv(norm_len, width, taper)` — derivative of the MuJoCo
  piecewise-quadratic FL curve w.r.t. normalized length. Returns `dFL/dL_norm`.
- `hill_active_fl_deriv(norm_len)` — derivative of the Hill-type FL curve.
- `muscle_gain_fv(norm_vel, fvmax)` — FV curve value (already exists for
  `mjd_actuator_vel`).
- `hill_force_velocity(norm_vel)` — Hill FV value (already exists).

These helpers parallel the velocity derivative helpers `muscle_gain_velocity_deriv`
and `hill_force_velocity_deriv` that already exist (derivatives.rs:2545–2585).

---

### S3. `mjd_rne_pos` — RNE position derivatives

**File:** `sim/L0/core/src/derivatives.rs` (new function, ~200–300 lines)
**MuJoCo equivalent:** No direct equivalent — extension. Velocity template: `mjd_rne_vel` (lines 780–1000)
**Design decision:** Follows the three-phase structure of `mjd_rne_vel`
(forward pass → backward pass → projection). Critical difference: position
derivatives are evaluated at actual acceleration `qacc` (not zero), capturing
the `(∂M/∂q)·qacc` term (AD-2). Uses new scratch Jacobians
`deriv_Dcvel_pos`, `deriv_Dcacc_pos`, `deriv_Dcfrc_pos` on Data (6×nv per body).

**Mathematical foundation:**

The RNE position derivative computes `−∂RNEA(q, v, qacc)/∂qpos`, which equals
`−(∂M/∂q)·qacc − ∂(C·v + g)/∂q`. This is stored in `qDeriv_pos`.

The RNEA recursion is:
```
Forward:  cvel[b] = X_b · cvel[parent] + S_b · qdot_b
          cacc[b] = X_b · cacc[parent] + cvel[b] ×_m (S_b · qdot_b) + S_b · qacc_b
Backward: cfrc[b] = I_b · cacc[b] + cvel[b] ×_f (I_b · cvel[b])
          cfrc[parent] += cfrc[b]
Project:  qfrc_bias[dof] = S_b^T · cfrc[b]
```

Differentiating w.r.t. `qpos` (tangent-space perturbation `δq`):

**Per-joint-type transform derivatives:**

For a joint `j` of body `b` with motion subspace `S_j`, the spatial transform
`X_b` is a function of `qpos` for joint `j`. The key derivative is
`∂(X_b · v)/∂q_j` for some spatial vector `v` being propagated:

- **Hinge (1 DOF):** Rotation about world-frame axis `a` by angle `θ`.
  ```
  ∂(X_hinge · v)/∂θ = S_hinge ×_m (X_hinge · v)
  ```
  where `S_hinge = [a; pos × a]` is the 6D motion subspace (world frame).
  The derivative is the spatial cross-product of the joint motion with the
  transformed vector. This is because an infinitesimal rotation `δθ` about
  axis `a` produces a velocity perturbation `S · δθ`, and the spatial
  transform derivative of `X · v` is `(S ×_m) · (X · v)`.

  In code:
  ```rust
  // For body b's own hinge joint, column dof_adr:
  let xv = /* X_b · cvel[parent] */ ;  // already computed as cvel[b] - S·qdot
  let s_col = joint_subspaces[jnt_id].column(0);
  Dcvel_pos[b][:, dof_adr] += spatial_cross_motion(s_col, xv);
  ```

- **Slide (1 DOF):** Translation along world-frame axis `a` by distance `d`.
  ```
  ∂(X_slide · v)/∂d = S_slide ×_m (X_slide · v)
  ```
  where `S_slide = [0; a]`. Same formula — slide motion subspace is pure
  translation, so the cross-product rotates the angular part by zero and
  shifts the linear part.

  In practice, for slide joints the transform derivative is simpler because
  `S_slide ×_m v = [0; a × ω]` where `ω` is the angular part of `v`. This
  only affects the linear part of the propagated vector.

- **Ball (3 DOF):** Rotation parameterized by quaternion `q` with tangent
  perturbation `δω ∈ R³`.
  ```
  ∂(X_ball · v)/∂ω_k = e_k ×_m (X_ball · v)    for k = 0,1,2
  ```
  where `e_k` are the unit vectors in the body's angular DOF space.
  Three columns, one per tangent direction. Each is a spatial cross-product
  of a unit angular velocity with the transformed vector.

  In code:
  ```rust
  for d in 0..3 {
      let s_col = joint_subspaces[jnt_id].column(d);  // e_k in spatial form
      let xv = /* X_b · cvel[parent] */;
      Dcvel_pos[b][:, dof_adr + d] += spatial_cross_motion(s_col, xv);
  }
  ```

- **Free (6 DOF):** Translation (3 DOFs) + rotation (3 DOFs).
  Translation DOFs (dof_adr..dof_adr+3): same as 3 independent slides.
  Rotation DOFs (dof_adr+3..dof_adr+6): same as Ball.
  ```rust
  for d in 0..6 {
      let s_col = joint_subspaces[jnt_id].column(d);
      let xv = /* X_b · cvel[parent] */;
      Dcvel_pos[b][:, dof_adr + d] += spatial_cross_motion(s_col, xv);
  }
  ```

**Key insight — common formula:** For ALL joint types, the transform position
derivative has the same form: `∂(X · v)/∂q_d = S[:, d] ×_m (X · v)`. This
is because the spatial transform derivative for any joint type is the
infinitesimal motion produced by a unit joint displacement. The formula
`spatial_cross_motion(S_col, X·v)` works uniformly across hinge, slide,
ball, and free joints.

**Algorithm (three phases):**

**Phase 1: Forward pass (root → leaves)**

```rust
pub(crate) fn mjd_rne_pos(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let nbody = model.nbody;
    if nv == 0 { return; }

    // Zero scratch Jacobians
    for b in 0..nbody {
        data.deriv_Dcvel_pos[b].fill(0.0);
        data.deriv_Dcacc_pos[b].fill(0.0);
        data.deriv_Dcfrc_pos[b].fill(0.0);
    }

    // Pre-compute joint motion subspaces
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    for body_id in 1..nbody {
        let parent_id = model.body_parent[body_id];

        // Propagate velocity Jacobian from parent
        let parent_dcvel = data.deriv_Dcvel_pos[parent_id].clone();
        data.deriv_Dcvel_pos[body_id].copy_from(&parent_dcvel);

        // For each joint of body b: transform derivative contribution
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        // Pre-compute X_body · cvel[parent] by subtracting ALL joints'
        // velocity contributions from cvel[b].
        //   cvel[b] = X_body · cvel[parent] + Σ_j S_j · qdot_j
        //   X_body · cvel[parent] = cvel[b] - Σ_j S_j · qdot_j
        // This must subtract ALL joints (not just one) for multi-joint
        // bodies. For single-joint bodies the result is the same.
        let mut xv = data.cvel[body_id];
        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let ndof = model.jnt_type[jnt_id].nv();
            for d in 0..ndof {
                for row in 0..6 {
                    xv[row] -= s[(row, d)] * data.qvel[dof_adr + d];
                }
            }
        }

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let ndof = model.jnt_type[jnt_id].nv();

            // Transform derivative: ∂(X_body · cvel[parent])/∂q_d
            //   = S[:, d] ×_m (X_body · cvel[parent])
            // Uses xv (parent velocity propagated through ALL joints)
            // because perturbing ANY joint rotates the body frame,
            // affecting the entire parent velocity propagation.
            for d in 0..ndof {
                let s_col = SpatialVector::new(
                    s[(0, d)], s[(1, d)], s[(2, d)],
                    s[(3, d)], s[(4, d)], s[(5, d)],
                );
                let cross = spatial_cross_motion(s_col, xv);
                for row in 0..6 {
                    data.deriv_Dcvel_pos[body_id][(row, dof_adr + d)] += cross[row];
                }
            }
        }

        // Propagate acceleration Jacobian from parent
        let parent_dcacc = data.deriv_Dcacc_pos[parent_id].clone();
        data.deriv_Dcacc_pos[body_id].copy_from(&parent_dcacc);

        // Pre-compute X_body · cacc_full[parent] by subtracting ALL joints'
        // Coriolis and acceleration contributions from cacc[b].
        //   cacc[b] = X_body · cacc[parent]
        //           + cvel[b] ×_m (Σ_j S_j · qdot_j) + Σ_j S_j · qacc_j
        //   X_body · cacc[parent] = cacc[b]
        //           - cvel[b] ×_m (Σ_j S_j · qdot_j) - Σ_j S_j · qacc_j
        // Must subtract ALL joints for multi-joint bodies.
        let mut xa = data.cacc[body_id];
        // Compute total joint velocity: v_joint_total = Σ_j S_j · qdot_j
        let mut v_joint_total = SpatialVector::zeros();
        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let ndof = model.jnt_type[jnt_id].nv();
            for d in 0..ndof {
                for row in 0..6 {
                    v_joint_total[row] += s[(row, d)] * data.qvel[dof_adr + d];
                }
            }
        }
        // Subtract Coriolis: cvel[b] ×_m v_joint_total
        let coriolis_total = spatial_cross_motion(data.cvel[body_id], v_joint_total);
        for row in 0..6 { xa[row] -= coriolis_total[row]; }
        // Subtract all joint accelerations: Σ_j S_j · qacc_j
        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let ndof = model.jnt_type[jnt_id].nv();
            for d in 0..ndof {
                let qacc_d = data.qacc[dof_adr + d];
                for row in 0..6 {
                    xa[row] -= s[(row, d)] * qacc_d;
                }
            }
        }

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let ndof = model.jnt_type[jnt_id].nv();

            // cacc_full[b] = X_body · cacc_full[parent]
            //              + cvel[b] ×_m (Σ_j S_j · qdot_j) + Σ_j S_j · qacc_j
            //
            // Position derivative has 4 terms:
            //
            // Term A: ∂(X_body · cacc_full[parent])/∂q_d
            //       = S[:, d] ×_m (X_body · cacc_full[parent])
            //       Uses xa (pre-computed above, subtracting ALL joints).
            //       This uses FULL parent acceleration (including qacc),
            //       capturing the (∂M/∂q)·qacc effect (AD-2).
            for d in 0..ndof {
                let s_col = SpatialVector::new(
                    s[(0, d)], s[(1, d)], s[(2, d)],
                    s[(3, d)], s[(4, d)], s[(5, d)],
                );
                // Term A: transform derivative of parent's full acceleration
                let cross_a = spatial_cross_motion(s_col, xa);
                for row in 0..6 {
                    data.deriv_Dcacc_pos[body_id][(row, dof_adr + d)] += cross_a[row];
                }
            }

            // Term B: X_body · Dcacc_pos[parent] — already propagated above.

            // Term C: ∂(cvel[b] ×_m (S_j · qdot_j))/∂qpos
            //       = Dcvel_pos[b] ×_m (S_j · qdot_j)
            //       (S_j · qdot_j is THIS JOINT's velocity contribution only.
            //        Each joint's Coriolis contribution is differentiated
            //        separately — the per-joint v_joint is correct here.)
            let mut v_joint = SpatialVector::zeros();
            for d in 0..ndof {
                for row in 0..6 {
                    v_joint[row] += s[(row, d)] * data.qvel[dof_adr + d];
                }
            }

            // Dcacc_pos[b] += mjd_cross_motion_vel(v_joint) · Dcvel_pos[b]
            // (reuse the existing mjd_cross_motion_vel helper — it computes
            //  ∂(cross_motion(v, s))/∂v, which here means
            //  ∂(cvel ×_m v_joint)/∂cvel · Dcvel_pos, giving the Coriolis
            //  position dependence through velocity change.)
            let mat = mjd_cross_motion_vel(&v_joint);
            let dcvel_ref = data.deriv_Dcvel_pos[body_id].clone();
            for c in 0..nv {
                let col = mat * dcvel_ref.column(c);
                for r in 0..6 {
                    data.deriv_Dcacc_pos[body_id][(r, c)] += col[r];
                }
            }

            // Term D: (∂S/∂qpos) · qacc_b
            // For hinge/slide: S depends on body orientation (world-frame axis).
            // When an ANCESTOR joint changes position, the body orientation
            // rotates, which rotates the joint axis. However, in CortenForge
            // (matching MuJoCo), joint_motion_subspace() returns S in the
            // CURRENT world frame — which is already position-dependent.
            // The ∂S/∂q term is captured by the fact that we recompute S at
            // the current position. For the body's OWN joint, S is local
            // (body-frame axis rotated to world) — the derivative ∂S/∂q_own
            // produces the same cross-product form as Term A applied to
            // S · qacc_b instead of cacc_full[parent].
            // For simplicity and correctness, we include this by also applying
            // the transform derivative to the joint acceleration contribution:
            for d in 0..ndof {
                let s_col = SpatialVector::new(
                    s[(0, d)], s[(1, d)], s[(2, d)],
                    s[(3, d)], s[(4, d)], s[(5, d)],
                );
                // ∂(S · qacc_b)/∂q_d = (∂S/∂q_d) · qacc_b
                // = S[:, d] ×_m (S · qacc_b)
                // Exact: same spatial cross-product identity as Term A.
                // An infinitesimal joint displacement δq_d rotates the body
                // frame by S[:, d]·δq_d, which rotates S·qacc by the same
                // amount. This is the Lie bracket [S_d, S·qacc].
                let mut s_qacc = SpatialVector::zeros();
                for dd in 0..ndof {
                    let qacc_dd = data.qacc[dof_adr + dd];
                    for row in 0..6 {
                        s_qacc[row] += s[(row, dd)] * qacc_dd;
                    }
                }
                let cross_d = spatial_cross_motion(s_col, s_qacc);
                for row in 0..6 {
                    data.deriv_Dcacc_pos[body_id][(row, dof_adr + d)] += cross_d[row];
                }
            }
        }
    }

    // Phase 2: Backward pass — body force position derivatives
    // Same structure as mjd_rne_vel backward pass (lines 875-920)
    for body_id in 1..nbody {
        let inertia = &data.cinert[body_id];
        let cvel = data.cvel[body_id];
        let i_v = inertia * cvel;

        // Dcfrc_pos[b] = I · Dcacc_pos[b]
        let dcacc = data.deriv_Dcacc_pos[body_id].clone();
        for c in 0..nv {
            let col = inertia * dcacc.column(c);
            for r in 0..6 {
                data.deriv_Dcfrc_pos[body_id][(r, c)] = col[r];
            }
        }

        // + crossForce_vel(I·v) · Dcvel_pos[b]
        let cross_vel_mat = mjd_cross_force_vel(&i_v);
        let dcvel = data.deriv_Dcvel_pos[body_id].clone();
        for c in 0..nv {
            let col = cross_vel_mat * dcvel.column(c);
            for r in 0..6 {
                data.deriv_Dcfrc_pos[body_id][(r, c)] += col[r];
            }
        }

        // + crossForce_frc(v) · I · Dcvel_pos[b]
        let cross_frc_mat = mjd_cross_force_frc(&cvel);
        for c in 0..nv {
            let i_col = inertia * dcvel.column(c);
            let result = cross_frc_mat * i_col;
            for r in 0..6 {
                data.deriv_Dcfrc_pos[body_id][(r, c)] += result[r];
            }
        }
    }

    // Accumulate to parent (leaves → root)
    for body_id in (1..nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            let child_dcfrc = data.deriv_Dcfrc_pos[body_id].clone();
            data.deriv_Dcfrc_pos[parent_id] += child_dcfrc;
        }
    }

    // Phase 3: Projection — qDeriv_pos[dof, :] -= S^T · Dcfrc_pos[body]
    for jnt_id in 0..model.njnt {
        let body_id = model.jnt_body[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let s = &joint_subspaces[jnt_id];
        let ndof = model.jnt_type[jnt_id].nv();

        for d in 0..ndof {
            for c in 0..nv {
                let mut val = 0.0;
                for row in 0..6 {
                    val += s[(row, d)] * data.deriv_Dcfrc_pos[body_id][(row, c)];
                }
                data.qDeriv_pos[(dof_adr + d, c)] -= val;
            }
        }
    }

    // Projection derivative: ∂(S^T · cfrc)/∂qpos includes
    // (∂S^T/∂q) · cfrc — the change in projection direction.
    // For the body's OWN joint:
    //   ∂(S^T)/∂q_d · cfrc[b] = -(S[:, d] ×_m)^T · cfrc[b]
    //   = -(cfrc[b] ×_f S[:, d])  (dual of spatial cross)
    // This captures how rotating the joint axis changes the projected force.
    for jnt_id in 0..model.njnt {
        let body_id = model.jnt_body[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let s = &joint_subspaces[jnt_id];
        let ndof = model.jnt_type[jnt_id].nv();
        let cfrc = data.cfrc[body_id];

        for d in 0..ndof {
            let s_col = SpatialVector::new(
                s[(0, d)], s[(1, d)], s[(2, d)],
                s[(3, d)], s[(4, d)], s[(5, d)],
            );
            // ∂(S[:, d']^T · cfrc)/∂q_d for all d'
            // = S[:, d']^T · (∂cfrc/∂q_d)  [already in Dcfrc_pos]
            //   + (∂S[:, d']/∂q_d)^T · cfrc
            // The second term for d' = d (own joint):
            let cross = spatial_cross_force(cfrc, s_col);
            for dd in 0..ndof {
                let s_dd = SpatialVector::new(
                    s[(0, dd)], s[(1, dd)], s[(2, dd)],
                    s[(3, dd)], s[(4, dd)], s[(5, dd)],
                );
                // (∂S[:, dd]/∂q_d)^T · cfrc
                // For own joint rotation: ∂S[:, dd]/∂q_d = S[:, d] ×_m S[:, dd]
                let ds = spatial_cross_motion(s_col, s_dd);
                let proj: f64 = (0..6).map(|row| ds[row] * cfrc[row]).sum();
                // This is equivalent to -S[:, dd]^T · cross (by adjoint)
                // but computed directly for clarity.
                data.qDeriv_pos[(dof_adr + dd, dof_adr + d)] -= proj;
            }
        }
    }
}
```

**Gravity contribution:** For bodies affected by gravity, the forward pass
inherits `cacc[world] = [0; 0; 0; -g_x; -g_y; -g_z]` (same as
`mjd_rne_vel`). Position derivatives of gravity torques emerge naturally
from Term A: `S[:, d] ×_m (X_b · cacc[parent])` in the forward pass — the
gravity "acceleration" propagates through the kinematic tree, and its
position derivative captures `∂(J^T · m · g)/∂q`.

**Direct gyroscopic term (Ball/Free):** The body-frame gyroscopic torque
`ω × (I·ω)` does NOT depend on position directly (it's in body frame
where I is constant). However, the mapping from generalized coordinates
to body-frame angular velocity does depend on position for ancestor
joints — this is captured by `Dcvel_pos` propagation through the
backward pass force derivatives. No separate gyroscopic position
derivative term is needed (unlike the velocity case where the direct
gyroscopic derivative at lines 942–999 handles body-frame coupling).

**Dimensional check:** Each phase preserves dimensions:
- `Dcvel_pos[b]`: 6 × nv (spatial velocity Jacobian w.r.t. position)
- `Dcacc_pos[b]`: 6 × nv (spatial acceleration Jacobian w.r.t. position)
- `Dcfrc_pos[b]`: 6 × nv (spatial force Jacobian w.r.t. position)
- Projection: `S^T (ndof × 6) · Dcfrc_pos (6 × nv) → ndof × nv` → fills
  `qDeriv_pos[dof_adr..+ndof, 0..nv]`

---

### S4. `mjd_smooth_pos` — Assembly function

**File:** `sim/L0/core/src/derivatives.rs` (new function, ~10 lines)
**MuJoCo equivalent:** Mirrors `mjd_smooth_vel` (lines 1020–1025)
**Design decision:** Simple dispatcher, same pattern as `mjd_smooth_vel`.
The `flg_bias` parameter from MuJoCo's `mjd_smooth_vel` is hardcoded to
true (position derivatives always include gravity/Coriolis contributions).

```rust
/// Compute ∂(qfrc_smooth)/∂qpos analytically and store in data.qDeriv_pos.
///
/// `qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias`
///
/// Populates position derivative storage with three analytical contributions:
///   1. ∂qfrc_passive/∂qpos via spring chain rules (joint + tendon)
///   2. ∂qfrc_actuator/∂qpos via gain/bias length derivatives
///   3. −∂RNEA(q,v,qacc)/∂qpos via RNE position chain rule
///      (includes −(∂M/∂q)·qacc from evaluating RNEA at actual acceleration)
///
/// **CortenForge extension** — MuJoCo has no analytical position derivatives.
/// Validated by comparing against FD position columns.
///
/// # Precondition
/// Forward pass must have completed (data.qacc populated).
pub fn mjd_smooth_pos(model: &Model, data: &mut Data) {
    data.qDeriv_pos.fill(0.0);
    mjd_passive_pos(model, data);
    mjd_actuator_pos(model, data);
    mjd_rne_pos(model, data);
}
```

---

### S5. `Data.qDeriv_pos` field and scratch Jacobians

**File:** `sim/L0/core/src/types/data.rs` (~line 577, after `qDeriv`)
**MuJoCo equivalent:** No equivalent — extension.
**Design decision:** Parallels `qDeriv` in every way: same dimensions (nv × nv),
same index convention (row = force DOF, col = position tangent DOF), same
lifecycle (transient, zeroed before each use by `mjd_smooth_pos`).

**New fields on Data:**

```rust
/// ∂(qfrc_smooth)/∂qpos — analytical position derivative matrix.
/// Populated by mjd_smooth_pos(). Dimensions: nv × nv.
/// Parallel to qDeriv which stores ∂(qfrc_smooth)/∂qvel.
/// CortenForge extension — MuJoCo has no equivalent.
#[allow(non_snake_case)]
pub qDeriv_pos: DMatrix<f64>,

/// Scratch Jacobian ∂(cvel)/∂(qpos) per body (length `nbody`, each 6 × nv).
/// Used by `mjd_rne_pos()` for position chain-rule propagation.
pub deriv_Dcvel_pos: Vec<DMatrix<f64>>,

/// Scratch Jacobian ∂(cacc_full)/∂(qpos) per body (length `nbody`, each 6 × nv).
/// Uses full acceleration (including qacc) — not just bias.
pub deriv_Dcacc_pos: Vec<DMatrix<f64>>,

/// Scratch Jacobian ∂(cfrc)/∂(qpos) per body (length `nbody`, each 6 × nv).
pub deriv_Dcfrc_pos: Vec<DMatrix<f64>>,
```

**Initialization in `make_data()` (model_init.rs:~730):**

```rust
qDeriv_pos: DMatrix::zeros(self.nv, self.nv),
deriv_Dcvel_pos: vec![DMatrix::zeros(6, self.nv); self.nbody],
deriv_Dcacc_pos: vec![DMatrix::zeros(6, self.nv); self.nbody],
deriv_Dcfrc_pos: vec![DMatrix::zeros(6, self.nv); self.nbody],
```

**No reset() changes needed:** Like the existing derivative scratch buffers
(`qDeriv`, `deriv_Dcvel`, etc.), the position derivative fields are transient
— zeroed at the start of each `mjd_smooth_pos()` call. The `reset()` method
does not zero existing derivative scratch buffers, and the new ones follow
the same convention.

---

### S6. Hybrid integration — analytical position columns

**File:** `sim/L0/core/src/derivatives.rs` (modify `mjd_transition_hybrid`, lines 1248–1657)
**MuJoCo equivalent:** No equivalent — extension. Current code: FD position columns at lines 1448–1511.
**Design decision:** Replace the FD position column loop (lines 1448–1511)
with analytical position columns using the chain rule:
```
∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq
∂v⁺/∂q = dvdq
∂act⁺/∂q = 0  (activation doesn't depend on position)
```

where `dvdq` is computed per integrator type from `qDeriv_pos`.

**Eligibility check (at top of `mjd_transition_hybrid`, after existing setup):**

```rust
// Check if model is eligible for analytical position columns.
// Ineligible models fall back to FD for position columns (same as before).
let use_analytical_pos = model.fluid_density == 0.0
    && model.fluid_viscosity == 0.0
    && model.ngravcomp_body == 0  // count of bodies with gravcomp != 0
    && model.nflex == 0
    && !has_site_body_slidercrank_actuators(model)
    && !has_muscle_bias_actuators(model);
```

**Helper functions:**
```rust
fn has_site_body_slidercrank_actuators(model: &Model) -> bool {
    model.actuator_trntype.iter().any(|t| matches!(t,
        ActuatorTransmission::Site
        | ActuatorTransmission::Body
        | ActuatorTransmission::SliderCrank
    ))
}

fn has_muscle_bias_actuators(model: &Model) -> bool {
    model.actuator_biastype.iter().any(|t| matches!(t,
        BiasType::Muscle | BiasType::HillMuscle
    ))
}
```

**When eligible — analytical position columns:**

```rust
if use_analytical_pos {
    // Compute analytical position force derivatives
    mjd_smooth_pos(model, &mut data_work);

    // Compute dvdq per integrator (parallels dvdv computation)
    let dvdq = match model.integrator {
        Integrator::Euler => {
            // dvdq = h · M⁻¹ · qDeriv_pos
            let mut dvdq = data_work.qDeriv_pos.clone();
            let (rowadr, rownnz, colind) = model.qld_csr();
            mj_solve_sparse_batch(
                rowadr, rownnz, colind,
                &data_work.qLD_data, &data_work.qLD_diag_inv,
                &mut dvdq,
            );
            dvdq *= h;
            dvdq
        }
        Integrator::ImplicitSpringDamper => {
            // dvdq = (M+hD+h²K)⁻¹ · h · qDeriv_pos
            // NO correction term (per EGT-7 in rubric):
            // The spring displacement term −h·K·δq provides its own
            // cancellation when differentiated w.r.t. position.
            //
            // Derivation: ISD step is (M+hD+h²K)·v⁺ = M·v + h·f_ext − h·K·δq
            // Differentiate w.r.t. q:
            //   (M+hD+h²K)·∂v⁺/∂q = h·∂f_ext/∂q − h·K
            // Since f_ext excludes implicit springs, ∂f_ext/∂q = qDeriv_pos + K:
            //   RHS = h·(qDeriv_pos + K) − h·K = h·qDeriv_pos
            let mut dvdq = DMatrix::zeros(nv, nv);
            for j in 0..nv {
                let mut rhs = DVector::zeros(nv);
                for i in 0..nv {
                    rhs[i] = h * data_work.qDeriv_pos[(i, j)];
                }
                cholesky_solve_in_place(&data_work.scratch_m_impl, &mut rhs);
                dvdq.column_mut(j).copy_from(&rhs);
            }
            dvdq
        }
        Integrator::ImplicitFast => {
            // dvdq = h · (M − h·D)⁻¹ · qDeriv_pos
            let mut dvdq = DMatrix::zeros(nv, nv);
            for j in 0..nv {
                let mut col = data_work.qDeriv_pos.column(j).clone_owned();
                cholesky_solve_in_place(&data_work.scratch_m_impl, &mut col);
                for i in 0..nv {
                    dvdq[(i, j)] = h * col[i];
                }
            }
            dvdq
        }
        Integrator::Implicit => {
            // dvdq = h · LU⁻¹ · qDeriv_pos
            let mut dvdq = DMatrix::zeros(nv, nv);
            for j in 0..nv {
                let mut col = data_work.qDeriv_pos.column(j).clone_owned();
                lu_solve_factored(
                    &data_work.scratch_m_impl,
                    &data_work.scratch_lu_piv,
                    &mut col,
                );
                for i in 0..nv {
                    dvdq[(i, j)] = h * col[i];
                }
            }
            dvdq
        }
        Integrator::RungeKutta4 => {
            // Already handled by early return to mjd_transition_fd
            unreachable!()
        }
    };

    // Chain rule: ∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq
    let dqdq = &integ.dqpos_dqpos + &integ.dqpos_dqvel * &dvdq;

    // Fill position columns of A
    a_mat.view_mut((0, 0), (nv, nv)).copy_from(&dqdq);    // ∂qpos/∂qpos
    a_mat.view_mut((nv, 0), (nv, nv)).copy_from(&dvdq);   // ∂qvel/∂qpos
    // ∂act/∂qpos = 0 (already zero from allocation)
} else {
    // Fallback: FD position columns (existing code, lines 1448–1511)
    // [existing FD loop preserved unchanged]
}
```

**`IntegrationDerivatives.dqpos_dqpos` activation:**
- Remove `#[allow(dead_code)]` from `dqpos_dqpos` field (line 1096)
- Remove the comment "Currently unused" (line 1094–1095)
- The field is now consumed by the analytical position column chain rule

**`dqpos_dqpos` per-joint-type values** (already computed by
`compute_integration_derivatives`, verified here for completeness):
- **Hinge/Slide:** `dqpos_dqpos = 1.0` (identity — position integrates
  linearly: `q⁺ = q + h·v⁺`)
- **Ball/Free (rotational DOFs):** `dqpos_dqpos = ∂(exp(ω·h)·q)/∂q` —
  the quaternion integration Jacobian from `mjd_quat_integrate`. For small
  `h·ω`, this is approximately `I₃ − h/2 · [ω]×` (3×3 block). The exact
  formula is `exp(−[ω·h]×)` (the rotation matrix for angle `−h·|ω|` about
  `ω/|ω|`).
- **Free (translational DOFs):** `dqpos_dqpos = 1.0` (same as Slide)

---

### S7. Export and model eligibility field

**File:** `sim/L0/core/src/lib.rs` (line ~265, derivatives export block)
**Design decision:** Export `mjd_smooth_pos` as public API.

```rust
pub use derivatives::{
    DerivativeConfig, InverseDynamicsDerivatives, TransitionMatrices,
    fd_convergence_check, max_relative_error, mjd_inverse_fd,
    mjd_passive_vel, mjd_quat_integrate, mjd_smooth_pos, mjd_smooth_vel,
    mjd_sub_quat, mjd_transition, mjd_transition_fd, mjd_transition_hybrid,
    validate_analytical_vs_fd,
};
```

**Model eligibility field:** Add `ngravcomp_body: usize` to Model if not
already present — count of bodies with `body_gravcomp != 0.0`. Computed
during model compilation. If already trackable via existing fields
(`model.body_gravcomp.iter().any(|g| *g != 0.0)`), no new field needed.

---

## Acceptance Criteria

### AC1: Hinge joint spring position derivative matches FD *(runtime test — analytically derived)*
**Given:** Single hinge joint, stiffness=10.0, springref=0.0, qpos=0.5rad, no gravity, no actuators, no contacts.
**After:** `mjd_smooth_pos(model, &mut data)`
**Assert:** `qDeriv_pos[(0,0)]` ≈ −10.0 ± 1e-12 (exact: spring derivative is −k).
**Field:** `Data.qDeriv_pos`

### AC2: Ball joint spring position derivative matches FD *(runtime test — FD-validated)*
**Given:** Single Ball joint, stiffness=5.0, springref=identity quaternion, qpos=small rotation (~0.3 rad about [1,1,1]/√3), no gravity, no actuators.
**After:** `mjd_smooth_pos(model, &mut data)`
**Assert:** `qDeriv_pos[0..3, 0..3]` matches FD `qDeriv_pos` within `1e-6` relative tolerance. FD computed by perturbing each tangent DOF ±ε and measuring `qfrc_spring` change.
**Field:** `Data.qDeriv_pos`

### AC3: Free joint spring position derivative matches FD *(runtime test — FD-validated)*
**Given:** Single Free joint, stiffness=8.0, springref at origin, qpos translated by [1,0,0] and rotated ~0.2 rad, no gravity.
**After:** `mjd_smooth_pos(model, &mut data)`
**Assert:** `qDeriv_pos[0..6, 0..6]` matches FD within `1e-6` relative tolerance. Translation block is −8·I₃, rotation block uses `mjd_sub_quat` Jacobian.
**Field:** `Data.qDeriv_pos`

### AC4: Tendon spring position derivative matches FD *(runtime test — FD-validated)*
**Given:** 2-hinge chain with tendon spanning both joints, tendon_stiffness=20.0, tendon_range=(0.5, 1.5), qpos set so tendon is outside deadband.
**After:** `mjd_smooth_pos(model, &mut data)`
**Assert:** `qDeriv_pos` tendon contribution matches FD within `1e-6` relative tolerance.
**Field:** `Data.qDeriv_pos`

### AC5: Affine actuator position derivative matches FD *(runtime test — FD-validated)*
**Given:** Single hinge joint with Affine gain (gainprm[1]=2.0 — length-dependent gain), ctrl=1.0, no gravity.
**After:** `mjd_smooth_pos(model, &mut data)`
**Assert:** `qDeriv_pos[(0,0)]` ≈ −gainprm[1] · ctrl · gear² (force-length chain rule via moment arm). Matches FD within `1e-6` relative tolerance.
**Field:** `Data.qDeriv_pos`

### AC6: RNE position derivatives match FD for multi-body hinge chain *(runtime test — FD-validated)*
**Given:** 3-link pendulum (3 hinge joints), gravity enabled, non-zero qvel and qacc (after a forward pass with ctrl=[1,1,1]).
**After:** `mjd_smooth_pos(model, &mut data)`
**Assert:** `qDeriv_pos` (full 3×3 matrix) matches FD within `1e-6` relative tolerance. The FD reference: perturb each qpos DOF ±ε, call `forward()` to recompute `qfrc_smooth`, measure difference.
**Field:** `Data.qDeriv_pos`

### AC7: Transition A matrix position columns match FD for eligible model *(runtime test — FD-validated)*
**Given:** 3-link pendulum (hinge joints), torque actuators, no contacts, Euler integrator.
**After:** `mjd_transition_hybrid(model, data, config)` with `use_analytical=true`
**Assert:** A matrix position columns (A[:, 0..nv]) match `mjd_transition_fd` within `1e-6` relative tolerance.
**Field:** `TransitionMatrices.A` columns 0..nv

### AC8: Transition A matrix — ISD integrator position columns *(runtime test — FD-validated)*
**Given:** 2-hinge pendulum with joint springs (stiffness=10), ImplicitSpringDamper integrator.
**After:** `mjd_transition_hybrid(model, data, config)`
**Assert:** Position columns match FD within `1e-6` relative tolerance. Specifically validates the ISD formula `dvdq = (M+hD+h²K)⁻¹ · h · qDeriv_pos` with NO correction term.
**Field:** `TransitionMatrices.A` columns 0..nv

### AC9: Transition A matrix — ImplicitFast/Implicit integrator position columns *(runtime test — FD-validated)*
**Given:** 2-hinge pendulum with damping, ImplicitFast integrator.
**After:** `mjd_transition_hybrid(model, data, config)`
**Assert:** Position columns match FD within `1e-6` relative tolerance.
**Field:** `TransitionMatrices.A` columns 0..nv

### AC10: Ball/Free joint transition position columns match FD *(runtime test — FD-validated)*
**Given:** Free-floating body (Free joint) + Ball joint child, non-zero state, Euler integrator.
**After:** `mjd_transition_hybrid(model, data, config)`
**Assert:** Position columns (all 9 = 3 free_linear + 3 free_angular + 3 ball) match FD within `1e-6` relative.
**Field:** `TransitionMatrices.A` columns 0..nv

### AC11: FD fallback for ineligible models *(runtime test)*
**Given:** Model with fluid density > 0 (or gravcomp or flex), same model otherwise as AC7.
**After:** `mjd_transition_hybrid(model, data, config)`
**Assert:** Position columns match `mjd_transition_fd` within FD tolerance. The hybrid path correctly falls back to FD.
**Field:** `TransitionMatrices.A` columns 0..nv

### AC12: Performance improvement *(runtime test)*
**Given:** 3-link pendulum (nv=3) or larger model.
**After:** Benchmark: time 100 calls to `mjd_transition_hybrid` with analytical position columns vs with FD position columns.
**Assert:** Analytical is ≥1.5× faster than FD-only hybrid path for position columns on models with nv ≥ 3.
**Field:** Wall-clock timing.

### AC13: No regression in existing tests *(runtime test)*
**Given:** Full domain test suite.
**After:** `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`
**Assert:** All existing tests pass. Zero failures.
**Field:** Test runner output.

### AC14: `dqpos_dqpos` no longer dead code *(code review)*
**Assert:** `#[allow(dead_code)]` removed from `IntegrationDerivatives.dqpos_dqpos`. The field is consumed by the analytical position column chain rule in `mjd_transition_hybrid`.

### AC15: `qDeriv_pos` field exists on Data *(code review)*
**Assert:** `Data.qDeriv_pos: DMatrix<f64>` field is present, initialized in `make_data()` to `zeros(nv, nv)`.

### AC16: Velocity columns unchanged *(runtime test)*
**Given:** Same model as AC7.
**After:** `mjd_transition_hybrid` with analytical position columns.
**Assert:** Velocity columns (A[:, nv..2*nv]) are identical to pre-Spec-A values. Position column changes do not affect velocity column computation.
**Field:** `TransitionMatrices.A` columns nv..2*nv

### AC17: Contact isolation — analytical matches FD on contact-free models *(runtime test — FD-validated)*
**Given:** 3-link pendulum, `DISABLE_CONTACT` or no geoms, non-zero state.
**After:** `mjd_transition_hybrid` vs `mjd_transition_fd`
**Assert:** Full A matrix matches within `1e-6` relative tolerance. On contact-free models, the constraint exclusion (EGT-9) has zero impact.
**Field:** `TransitionMatrices.A`

### AC18: Slide joint spring position derivative matches FD *(runtime test — analytically derived)*
**Given:** Single slide joint, stiffness=15.0, springref=0.0, qpos=0.3, no gravity, no actuators, no contacts.
**After:** `mjd_smooth_pos(model, &mut data)`
**Assert:** `qDeriv_pos[(0,0)]` ≈ −15.0 ± 1e-12 (exact: spring derivative is −k, same as hinge). Matches FD within `1e-6` relative tolerance.
**Field:** `Data.qDeriv_pos`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (hinge spring) | T1 | Direct |
| AC2 (ball spring) | T2 | Direct |
| AC3 (free spring) | T3 | Direct |
| AC4 (tendon spring) | T4 | Direct |
| AC5 (affine actuator) | T5 | Direct |
| AC6 (RNE multi-body) | T6 | Direct |
| AC7 (transition Euler) | T7 | Direct |
| AC8 (transition ISD) | T8 | Direct |
| AC9 (transition ImplicitFast) | T9 | Direct |
| AC10 (Ball/Free transition) | T10 | Direct |
| AC11 (FD fallback) | T11 | Direct |
| AC12 (performance) | T12 | Direct |
| AC13 (no regression) | T13 | Regression |
| AC14 (dqpos_dqpos not dead) | — | Code review (manual) |
| AC15 (qDeriv_pos field) | — | Code review (manual) |
| AC16 (velocity unchanged) | T7 (also checks velocity) | Regression |
| AC17 (contact isolation) | T14 | Direct |
| AC18 (slide spring) | T_supp6 | Direct |

---

## Test Plan

### T1: Hinge spring position derivative (analytical) → AC1
Single hinge joint, stiffness=10.0, springref=0.0, qpos=0.5.
Expected: `qDeriv_pos[(0,0)] = -10.0` (exact, diagonal stiffness).
Tolerance: 1e-12 (exact value, no approximation).

### T2: Ball spring position derivative (FD-validated) → AC2
Single Ball joint, stiffness=5.0, identity springref, small rotation.
FD reference: perturb each of 3 tangent DOFs by ±1e-6, measure `qfrc_spring`
change, compute 3×3 Jacobian. Compare analytical `qDeriv_pos[0..3, 0..3]`
against FD Jacobian.
Tolerance: 1e-6 relative (FD approximation).

### T3: Free joint spring position derivative (FD-validated) → AC3
Free joint with translation + rotation springs. Verify 6×6 block matches FD.
Tolerance: 1e-6 relative.

### T4: Tendon spring position derivative (FD-validated) → AC4
2-hinge chain with tendon spring outside deadband. Verify `J^T · k · J`
pattern in `qDeriv_pos`.
Tolerance: 1e-6 relative.

### T5: Affine actuator position derivative (FD-validated) → AC5
Hinge joint with Affine gain, gainprm[1]=2.0 (length-dependent), ctrl=1.0.
Verify `qDeriv_pos[(0,0)] ≈ -gainprm[1] · ctrl · gear²`.
Tolerance: 1e-6 relative.

### T6: RNE position derivatives — 3-link pendulum (FD-validated) → AC6
3-link hinge pendulum, gravity on, non-zero qvel and qacc.
FD reference: perturb each qpos DOF, call `forward()`, measure `qfrc_smooth`.
Compare full 3×3 `qDeriv_pos` against FD.
Tolerance: 1e-6 relative.

### T7: Transition A matrix — Euler — position columns match FD → AC7, AC16
3-link pendulum with torque actuators, Euler integrator.
Run `mjd_transition_hybrid` and `mjd_transition_fd`. Compare position columns
(0..nv) and verify velocity columns (nv..2*nv) are identical.
Tolerance: 1e-6 relative for position columns.

### T8: Transition A matrix — ISD — position columns match FD → AC8
2-hinge with springs, ISD integrator. Verify `dvdq = (M+hD+h²K)⁻¹ · h · qDeriv_pos`.
Tolerance: 1e-6 relative.

### T9: Transition A matrix — ImplicitFast — position columns match FD → AC9
2-hinge with damping, ImplicitFast integrator.
Tolerance: 1e-6 relative.

### T10: Transition A matrix — Ball/Free joints → AC10
Free body + Ball child, Euler integrator. 9 position columns.
Tolerance: 1e-6 relative.

### T11: FD fallback for ineligible model → AC11
Model with fluid density > 0 (make body have fluid). Verify hybrid path
uses FD for position columns (result matches `mjd_transition_fd`).
Tolerance: FD tolerance (machine precision for same FD path).

### T12: Performance benchmark → AC12
Wall-clock 100 iterations on a 3-link pendulum. Compare analytical position
columns vs FD. Assert ≥1.5× speedup.
(Note: may be `#[ignore]` in CI, run manually.)

### T13: Regression — all existing tests pass → AC13
`cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` green.

### T14: Contact isolation — analytical matches FD exactly → AC17
3-link pendulum, no contact geoms, non-zero state. Full A matrix comparison.
Tolerance: 1e-6 relative.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| World body (body_id == 0) | No parent, no joints. RNE forward pass must skip. | T6 (implicitly — world body is root) | AC6 |
| Zero-mass body | Zero gravity contribution, zero inertia. | T_supp1 | — |
| Disabled gravity (`DISABLE_GRAVITY`) | Gravity torque position derivatives should be zero. | T_supp2 | — |
| Disabled springs (`DISABLE_SPRING`) | `mjd_passive_pos` should skip spring computation. | T_supp3 | — |
| `nv = 0` (no joints) | All derivative functions should return immediately. | T_supp4 | — |
| Single-joint body (hinge) | Simplest non-trivial case. | T1, T7 | AC1, AC7 |
| Multi-joint body | Multiple joints on one body (rare but valid). | T_supp5 | — |
| Slide joint | Different from hinge: translation derivative. | T_supp6 | AC18 |
| FD convergence | Shrinking ε confirms analytical matches FD limit. | T_supp7 | — |
| ISD no correction term | Validate that NO `+h²·K` correction is needed. | T8 | AC8 |
| `dqpos_dqpos` values | Hinge/Slide = 1.0, Ball/Free = quaternion derivative. | T10 (implicitly) | AC10 |
| Sleeping bodies | Bodies with `disable_actuation` or sleeping flag. Position derivatives should still be correct (forward pass has already run). | T_supp8 | — |
| No position-dependent forces | Model with no springs, no actuators, gravity disabled. `qDeriv_pos` should be approximately zero. | T_supp9 | — |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T_supp1: Zero-mass body | Body with mass=0, verify no NaN or division by zero in RNE pos | Edge case robustness |
| T_supp2: Disabled gravity | `mjd_smooth_pos` with gravity disabled, verify gravity contribution is zero | Flag interaction |
| T_supp3: Disabled springs | `mjd_smooth_pos` with springs disabled, verify passive contribution is zero | Flag interaction |
| T_supp4: nv=0 model | All position derivative functions return without error | Degenerate case |
| T_supp5: Multi-joint body (FD-validated) | Body with 2 hinge joints on one body, non-zero qvel/qacc. Verify `qDeriv_pos` matches FD within `1e-6`. This validates the all-joints extraction of `X_body · cvel[parent]` and `X_body · cacc[parent]` (subtracting ALL joints, not just one). | Rare but critical for correctness |
| T_supp6: Slide joint spring position derivative → AC18 | Single slide joint with spring, stiffness=15.0, verify `qDeriv_pos[(0,0)] = -15.0` and matches FD | Joint type coverage (P4 requires one AC per joint type) |
| T_supp7: FD convergence | Run analytical vs FD at eps=1e-4, 1e-6, 1e-8; verify error decreases | Validates analytical correctness |
| T_supp8: Sleeping bodies | 2-body chain where one body has sleeping/disabled flag set. Verify `mjd_smooth_pos` produces correct derivatives (forward pass has already computed qacc, cvel, cacc). No special-casing needed — sleeping is a forward-pass concern, not a derivative concern. | Edge case (P1 A+ bar requires sleeping bodies) |
| T_supp9: Zero-derivative model (negative test) | Model with no springs (`DISABLE_SPRING`), no actuators (`nu=0`), gravity disabled (`DISABLE_GRAVITY`). After `mjd_smooth_pos`, verify `qDeriv_pos` is the zero matrix (all entries < 1e-15). RNE still contributes Coriolis position derivatives if qvel ≠ 0, so set qvel = 0 for true zero. | Negative case (P5 A+ bar requires "result ≈ zero matrix" test) |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Hybrid position columns (eligible models) | FD via `step()` loop (lines 1448–1511) | Analytical via `mjd_smooth_pos()` + chain rule | Already conformant → still conformant (same numerical result) | `mjd_transition_hybrid` callers | None — transparent change |
| Hybrid position columns (ineligible models) | FD via `step()` loop | FD via `step()` loop (unchanged) | Already conformant | Same | None |
| `IntegrationDerivatives.dqpos_dqpos` | Computed but unused (`#[allow(dead_code)]`) | Computed and consumed by position column chain rule | N/A — internal field | None — internal change | None |
| `Data` struct size | N fields | N+4 fields (`qDeriv_pos` + 3 scratch Jacobians) | N/A | Data allocation, clone cost | None — additive. Clone cost increase ~proportional to nv². |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/derivatives.rs` | New `mjd_smooth_pos()`, `mjd_passive_pos()`, `mjd_actuator_pos()`, `mjd_rne_pos()` functions. Modify `mjd_transition_hybrid()` to use analytical position columns. Helper functions for FL curve derivatives. Remove `#[allow(dead_code)]` from `dqpos_dqpos`. | +500–700 new, ~50 modified |
| `sim/L0/core/src/types/data.rs` | New `qDeriv_pos`, `deriv_Dcvel_pos`, `deriv_Dcacc_pos`, `deriv_Dcfrc_pos` fields. | +10 |
| `sim/L0/core/src/types/model_init.rs` | Initialize new Data fields in `make_data()`. | +5 |
| `sim/L0/core/src/lib.rs` | Export `mjd_smooth_pos`. | +1 |
| `sim/L0/tests/integration/derivatives.rs` | New tests T1–T14, T_supp1–T_supp9. | +300–400 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_transition_fd_*` | `derivatives.rs` tests | Pass (unchanged) | Pure FD path unchanged |
| `test_hybrid_*` | `derivatives.rs` tests | Pass (identical values) | Analytical position columns produce same result as FD |
| `test_smooth_vel_*` | `derivatives.rs` tests | Pass (unchanged) | Velocity derivatives unaffected |
| `test_sub_quat_*` | `derivatives.rs` tests | Pass (unchanged) | DT-52 unaffected |
| `test_inverse_fd_*` | `derivatives.rs` tests | Pass (unchanged) | DT-51 unaffected |
| `test_forward_skip_*` | `forward/` tests | Pass (unchanged) | DT-53 unaffected |
| `test_muscle_vel_deriv_*` | `derivatives.rs` tests | Pass (unchanged) | DT-54 unaffected |
| `test_fluid_deriv_*` | `fluid_derivatives.rs` | Pass (unchanged) | Fluid velocity derivatives unaffected |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `derivatives.rs:226–353` | `mjd_transition_fd` | Pure FD path — unchanged. May be used as validation reference. |
| `derivatives.rs:532–651` | `mjd_actuator_vel` | Velocity derivatives — independent of position derivatives. |
| `derivatives.rs:780–1000` | `mjd_rne_vel` | Velocity RNE — independent. New `mjd_rne_pos` is structurally similar but separate. |
| `derivatives.rs:1513–1553` | Muscle activation FD fallback | Activation columns — NOT affected by position column changes. |
| `derivatives.rs:1555–1649` | B matrix computation | Control influence — NOT affected by position column changes. |
| `forward/mod.rs:309+` | `forward_skip` | Pipeline function — not modified by position derivatives. |

---

## Execution Order

1. **S5 first** — Add `Data.qDeriv_pos` field and scratch Jacobians. Update
   `make_data()`. Verify compilation. This is the foundation — all other
   sections depend on it.

2. **S1 second** — Implement `mjd_passive_pos`. Test with T1 (hinge spring),
   T2 (ball spring), T3 (free spring), T4 (tendon spring). Verify each
   against FD.

3. **S2 third** — Implement `mjd_actuator_pos`. Test with T5 (affine actuator).
   Verify against FD. Implement FL curve derivative helpers.

4. **S3 fourth** — Implement `mjd_rne_pos`. This is the largest and most
   complex section. Test with T6 (3-link pendulum). Verify full `qDeriv_pos`
   against FD.

5. **S4 fifth** — Implement `mjd_smooth_pos` assembly. This is trivial
   (~10 lines) but validates the composition of S1+S2+S3.

6. **S6 sixth** — Integrate into `mjd_transition_hybrid`. Add eligibility
   check. Replace FD position loop with analytical path. Activate
   `dqpos_dqpos`. Test with T7–T10 (per-integrator), T11 (fallback),
   T14 (contact isolation).

7. **S7 last** — Export from `lib.rs`.

After each section lands, run `cargo test -p sim-core -p sim-mjcf
-p sim-conformance-tests` to verify no regressions.

---

## Performance Characterization

**Cost model:**
- **FD position columns:** `nv` simulation steps per column (forward FD) or
  `2·nv` for centered. Each step is O(nbody) forward dynamics. Total:
  O(nv · nbody) or O(nv² · nbody) amortized.
- **Analytical:** `mjd_smooth_pos` is O(nbody · nv) for the RNE chain rule
  (forward + backward pass over bodies, each touching 6×nv matrices).
  Plus O(nv²) for the integrator solve. Total: O(nbody · nv + nv²).
  No simulation steps.

**Expected speedup:**
- Position columns are roughly half the cost of `mjd_transition_hybrid`
  (the other half is velocity columns, already analytical).
- Eliminating nv simulation steps and replacing with O(nbody · nv) matrix
  ops gives ~1.5–2× speedup on the position column portion.
- **Total hybrid speedup: ~3–4× over pure FD** (velocity analytical +
  position analytical vs all FD).

**When analytical might NOT be faster:**
- Very small nv (nv ≤ 2) where FD overhead is minimal and analytical setup
  cost dominates. In practice, even nv=3 should show improvement.

**Measurement:** AC12 benchmarks wall-clock time on a reference model.

---

## Out of Scope

- **Full FK Jacobians `∂xpos/∂qpos`, `∂xmat/∂qpos`** (DT-45) — Spec A
  computes force derivatives, not FK output derivatives. The FK chain rule
  is used internally but not exposed as a standalone FK Jacobian API.
  *Conformance impact: none — Spec A covers the v1.0 surface.*

- **Contact force position derivatives `∂qfrc_constraint/∂qpos`** (DT-46) —
  Same approximation as existing velocity columns. FD captures contacts.
  *Conformance impact: none for contact-free models; bounded approximation
  for contact models, consistent with velocity-column precedent.*

- **Fluid force position derivatives** — Deferred via AD-1 (FD fallback).
  *Conformance impact: models with fluid use FD position columns (correct).*

- **Gravity compensation position derivatives** — Deferred via AD-1.
  *Conformance impact: models with gravcomp use FD position columns (correct).*

- **Flex bending/edge position derivatives** — Deferred via AD-1.
  *Conformance impact: models with flex use FD position columns (correct).*

- **Actuator moment-arm cross-term for Site/Body/SliderCrank** — Deferred
  via AD-1. *Conformance impact: models with these transmissions use FD
  position columns (correct).*

- **Muscle/HillMuscle bias passive FL position derivative** — Deferred.
  Gain position derivatives (active FL curve) ARE implemented; bias position
  derivatives (passive FL curve) are not. Models with Muscle/HillMuscle bias
  actuators use FD fallback via eligibility check.
  *Conformance impact: models with muscle bias use FD position columns (correct).*

- **Automatic differentiation** (DT-50) — orthogonal approach.

- **Sparse derivative storage** (DT-48) — all matrices remain dense.

- **Parallel FD** (DT-49) — sequential FD for fallback path.
