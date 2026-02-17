# Future Work 9 — Phase 3A: Noslip + Actuator/Dynamics + Tendon Equality (Items #33–37)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

All items are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers the noslip post-processor for PGS/CG, actuator/dynamics features, and
tendon equality constraints.

---

### 33. Noslip Post-Processor for PGS/CG Solvers
**Status:** Newton done, PGS/CG missing | **Effort:** S | **Prerequisites:** None

#### Current State

The noslip post-processor is **fully implemented for the Newton solver** as
`noslip_postprocess()` (`mujoco_pipeline.rs`, lines ~16521–16694, ~170 LOC).
Three integration tests verify it (`newton_solver.rs`, lines 656–777):
- `test_noslip_zero_iterations_is_noop` — regression test
- `test_noslip_produces_finite_results` — basic sanity
- `test_noslip_reduces_slip` — quantitative slip reduction on box-on-slope

The Newton implementation:
1. Identifies friction rows from `efc_type`/`efc_dim` (contact rows with `dim ≥ 3`,
   skipping the normal row 0).
2. Builds a friction-only Delassus submatrix `A_fric` with **no regularizer** on
   the diagonal (`1/A[i,i]` instead of `1/(A[i,i] + R[i])`).
3. Runs PGS iterations on friction rows with elliptic cone projection.
4. Writes back updated `efc_force` and recomputes `qfrc_constraint` and `qacc`.

**What's missing:** The PGS and CG solver paths bypass noslip entirely. The solver
dispatch at line ~18364 calls `noslip_postprocess()` only after
`NewtonResult::Converged`. The PGS/CG code path (which runs when Newton is not
selected, or Newton falls through) does not invoke noslip. This means models using
`solver="PGS"` or `solver="CG"` with `noslip_iterations > 0` silently ignore the
noslip setting.

MJCF parsing is complete: `noslip_iterations` and `noslip_tolerance` are read from
`<option>` and stored on `Model` (defaults: 0 iterations, 1e-6 tolerance).

#### Objective

Wire `noslip_postprocess()` into the PGS and CG solver paths so all three solver
types support noslip when `noslip_iterations > 0`.

#### Specification

##### S1. Refactor `noslip_postprocess()` to be solver-agnostic

The existing `noslip_postprocess()` function is already solver-agnostic internally —
it operates on `efc_force`, `efc_type`, `efc_dim`, and the Jacobians, none of which
are Newton-specific. The only coupling is the **call site**.

Verify that the function does not access any Newton-specific state (it should not).
If it does, factor out any Newton dependencies.

##### S2. Add noslip call after PGS/CG solve

In `mj_fwd_constraint()`, after the PGS or CG solver finishes (the code path that
handles penalty contacts + iterative solve), add:

```rust
// After PGS/CG solve completes and efc_force is populated:
if model.noslip_iterations > 0 {
    noslip_postprocess(model, data);
}
```

The noslip function already handles the `qacc` recomputation internally
(`qfrc_constraint = J^T · efc_force`, then `qacc = M^{-1} · (qfrc_smooth +
qfrc_constraint)`). No additional post-processing needed.

##### S3. Constraint row handling

The existing noslip implementation processes **contact friction rows only** (rows
where `efc_type == ContactElliptic || ContactNonElliptic` and `dim ≥ 3`, skipping
the normal row). MuJoCo's `mj_solNoSlip` also processes:
- Equality constraint rows (indices `0..ne`)
- Dry friction rows (indices `ne..ne+nf`) with interval clamping `[-floss, +floss]`

These are currently absent from our `noslip_postprocess()`. For PGS/CG conformance:
- **Phase 1 (this spec):** Keep current behavior — contact friction rows only.
  This covers the primary use case (suppress tangential slip in contact).
- **Phase 2 (stretch):** Add equality and dry friction row processing. Mark with
  TODO in code.

##### S4. Cone projection modes

Both pyramidal (`cone: 0`, MuJoCo default, §32 ✅) and elliptic (`cone: 1`)
friction cones are fully implemented. Pyramidal contacts skip noslip entirely
(no separate friction rows — §32), so noslip only operates on elliptic friction
rows. If noslip needs to support pyramidal in the future, `noslip_postprocess()`
must dispatch to the correct projection based on `model.cone`.

#### Acceptance Criteria

1. **PGS + noslip**: A box on a 20° incline with `solver="PGS"` and
   `noslip_iterations=10` shows reduced tangential slip compared to
   `noslip_iterations=0`. Measure `qvel` tangential component after 100 steps.
2. **CG + noslip**: Same benchmark with `solver="CG"` — noslip reduces slip.
3. **Newton regression**: Existing Newton noslip tests continue to pass
   (no behavior change for Newton path).
4. **`noslip_iterations=0` is no-op**: For all solver types, zero iterations
   produces identical output to the no-noslip code path.
5. **Normal forces unchanged**: After noslip, contact normal force components
   (`efc_force[normal_row]`) are identical to pre-noslip values (bit-exact).
6. **MuJoCo reference**: Compare `efc_force` after noslip against MuJoCo 3.4.0
   for a box-on-slope benchmark with `solver="PGS" noslip_iterations=10`.
   Tolerance: `1e-4` (PGS tolerance, same as main solve).

#### Implementation Notes

**Minimal diff.** The implementation is a 2–5 line change at the PGS/CG solver
call site. The `noslip_postprocess()` function is already written and tested.
The primary risk is verifying it works correctly with PGS/CG constraint assembly
output (which uses a slightly different `efc_*` layout than Newton).

**Verify `efc_type`/`efc_dim` layout.** The Newton path and PGS path may
populate `efc_type` and `efc_dim` differently (Newton uses
`assemble_unified_constraints`, PGS uses the penalty + PGS code path). Verify
that `noslip_postprocess()` correctly identifies friction rows in both layouts.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — add noslip call after PGS/CG solve
  (2–5 lines); verify `noslip_postprocess()` compatibility with PGS efc layout
- `sim/L0/tests/integration/` — new tests for PGS+noslip, CG+noslip

---

### 34. `actlimited` / `actrange` — Activation State Clamping
**Status:** Parsing done, runtime remaining | **Effort:** S–M | **Prerequisites:** None

#### Current State

**Parsing complete (Batch 1 defaults refactor):** `actlimited`, `actrange`,
`actearly`, `lengthrange`, and `group` are parsed on both `MjcfActuator` and
`MjcfActuatorDefaults` (`types.rs`, `parser.rs`). The defaults pipeline merges
and applies them via `merge_actuator_defaults()` / `apply_to_actuator()`
(`defaults.rs`). All 5 fields use `Option<T>` with `is_none()` guards.

**Not yet wired to runtime:** The parsed values are not forwarded to `Model`
fields and not enforced during activation dynamics. Only muscle activations
are clamped (hardcoded `[0, 1]`). All other stateful actuators (`dyntype=Integrator`,
`Filter`, `FilterExact`) can grow unbounded.

#### Objective

~~Parse `actlimited` and `actrange` from MJCF~~ ✅, store on Model, and clamp
activation state after dynamics integration.

#### Remaining Specification

1. ~~**MJCF parsing**~~ ✅ — All 5 activation fields parsed + defaultable.
2. **Model storage**: Add `actuator_actlimited: Vec<bool>` and
   `actuator_actrange: Vec<[f64; 2]>` fields to `Model`. Forward parsed values
   from `model_builder.rs`.
3. **Runtime clamping**: After activation dynamics integration
   (`mj_fwd_actuation` → `act_dot` → `act += dt * act_dot`), for each actuator
   where `actlimited[i] == true`, clamp `act[i]` to `actrange[i]`.
4. **Muscle special case**: Muscles currently hardcode `[0, 1]` clamping. If
   `actlimited=true` is set on a muscle, use `actrange` instead of `[0, 1]`.
   If `actlimited=false` (default), preserve the existing `[0, 1]` muscle clamp.
5. **MuJoCo behavior**: When `actlimited` is not set but the actuator has
   `dyntype != none`, MuJoCo does NOT clamp by default. We match this.

#### Acceptance Criteria

1. An integrator-type actuator with `actlimited="true" actrange="-1 1"` has
   activation clamped to `[-1, 1]`.
2. Without `actlimited`, integrator activations are unbounded (existing behavior).
3. Muscle actuators still default to `[0, 1]` clamping when `actlimited` is not
   explicitly set.
4. Position servo (`<position>`) with `actlimited="true"` clamps FilterExact
   activation state.
5. ~~Default class inheritance works for both attributes.~~ ✅

#### Files

- ~~`sim/L0/mjcf/src/model_builder.rs` — parse actlimited/actrange from MJCF~~ ✅
- `sim/L0/mjcf/src/model_builder.rs` — forward parsed actlimited/actrange to Model
- `sim/L0/core/src/mujoco_pipeline.rs` — Model fields, runtime clamping after
  activation integration

---

### 35. `gravcomp` — Body Gravity Compensation
**Status:** Not started | **Effort:** S–M | **Prerequisites:** None

#### Current State

Not parsed, not stored, not enforced. Silently ignored when present in MJCF. Test
assets (Apptronik Apollo) use `gravcomp="0"` which is silently dropped.

#### Objective

Parse `gravcomp` from `<body>`, store on Model, and subtract the specified fraction
of gravitational force from `qfrc_bias` during the RNE pass.

#### Specification

1. **MJCF parsing**: Parse `gravcomp` (float, default 0.0, range [0, 1]) from
   `<body>` elements. Value of 1.0 means full gravity compensation.
2. **Model storage**: Add `body_gravcomp: Vec<f64>` field to `Model`.
3. **Runtime effect**: In `mj_rne` (bias force computation), for each body with
   `gravcomp > 0`, subtract `gravcomp * m_subtree * gravity` from the body's
   gravitational contribution to `qfrc_bias`. MuJoCo implements this as: after
   computing the standard bias forces, iterate over bodies and subtract
   `gravcomp[i] * body_mass[i] * gravity` projected through the Jacobian to
   joint space.
4. **Subtree semantics**: MuJoCo's `gravcomp` applies to the body itself, not
   its subtree. Each body's gravity compensation is independent.

#### Acceptance Criteria

1. A body with `gravcomp="1"` experiences zero net gravitational bias force.
2. A body with `gravcomp="0"` (default) is unaffected.
3. A body with `gravcomp="0.5"` has half the gravitational bias force.
4. The Franka Panda model (which uses `gravcomp`) loads and simulates with
   correct bias forces.
5. Apptronik Apollo `gravcomp="0"` parses without warning (no behavioral change).

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse gravcomp
- `sim/L0/core/src/mujoco_pipeline.rs` — Model field, bias force adjustment

---

### 36. Body-Transmission Actuators (Adhesion)
**Status:** Parsing done, transmission missing | **Effort:** M | **Prerequisites:** None

#### Current State

**What exists:**
- **MJCF parsing**: `<actuator><adhesion>` is fully parsed. `MjcfActuatorType::Adhesion`
  sets `gaintype = Fixed` (with user-specified gain), `biastype = None`,
  `dyntype = None`, `ctrllimited = true`. The `body` attribute is parsed and stored
  as `actuator.body: Option<String>`. (`parser.rs`, lines ~1685–1689;
  `model_builder.rs`, lines ~1972–1976.)
- **Force generation**: Adhesion uses standard gain/bias: `force = gain * ctrl`.
  Clamped by `forcerange`. No activation dynamics. This already works through the
  standard `mj_fwd_actuation()` Phase 2 force computation.
- **Error on body transmission**: `model_builder.rs` line ~1794 returns
  `ModelConversionError` when `actuator.body` is `Some(...)`: "Actuator '...' uses
  body transmission '...' which is not yet supported." This is the **only** blocker.

**What's missing:**
- `ActuatorTransmission::Body(usize)` variant in the enum (currently: Joint/Tendon/Site).
- `mj_transmission_body()` — computes moment arm from contact normal Jacobians.
- Force application via `qfrc_actuator += moment_arm * force` in Phase 3 of
  `mj_fwd_actuation()`.

**Existing transmission patterns** (for reference):
- **Joint**: `qfrc_actuator[dof] += gear * force` (direct DOF application).
- **Tendon**: `apply_tendon_force()` with cached `ten_J` Jacobian.
- **Site**: `qfrc_actuator[dof] += actuator_moment[i][dof] * force` (pre-computed
  moment Jacobian from `mj_transmission_site()`).

Body transmission follows the **Site pattern** — pre-compute a moment arm vector
in a transmission function, then apply it in Phase 3.

#### Objective

Implement body transmission (`ActuatorTransmission::Body`) so adhesion actuators
load and function. The adhesion force pulls/pushes the target body toward/away from
surfaces via contact normal Jacobians.

#### Specification

##### S1. `ActuatorTransmission::Body` enum variant

Add a new variant to the transmission enum:

```rust
pub enum ActuatorTransmission {
    Joint,
    Tendon,
    Site,
    /// Body transmission — force applied via average of contact normal Jacobians.
    /// The usize is the target body index.
    Body(usize),
}
```

##### S2. `mj_transmission_body()` — moment arm computation

**Algorithm** (matches MuJoCo `engine_core_smooth.c`):

```rust
fn mj_transmission_body(
    model: &Model,
    data: &Data,
    actuator_id: usize,
) -> DVector<f64> {
    let body_id = match model.actuator_transmission[actuator_id] {
        ActuatorTransmission::Body(id) => id,
        _ => unreachable!(),
    };
    let gear = model.actuator_gear[actuator_id];
    let mut moment = DVector::zeros(model.nv);
    let mut count = 0usize;

    for c in 0..data.ncon {
        let contact = &data.contacts[c];
        let body1 = model.geom_body[contact.geom1];
        let body2 = model.geom_body[contact.geom2];

        if body1 != body_id && body2 != body_id {
            continue;
        }

        // Compute contact Jacobian difference (J_body1 - J_body2)
        // projected along contact normal (first axis of contact frame).
        // This gives the normal-direction relative velocity Jacobian.
        let j_normal = compute_contact_normal_jacobian(
            model, data, contact,
        );

        // Accumulate (sign convention: negative = attractive)
        moment += &j_normal;
        count += 1;
    }

    if count > 0 {
        // Average and negate: positive ctrl → attractive force
        moment *= -gear / (count as f64);
    }

    moment
}
```

The `compute_contact_normal_jacobian()` helper computes `J_diff · n` where
`J_diff = J(body1, contact_pos) - J(body2, contact_pos)` is the relative
velocity Jacobian at the contact point, and `n` is the contact normal.
This is the same Jacobian row used for the normal constraint direction.

**Key detail — negative sign:** The `-1/count` scaling means positive actuator
force produces a generalized force that pulls the body *toward* the surface
(attractive adhesion). This is MuJoCo's convention. The constraint solver
still clamps normal force ≥ 0 — adhesion does NOT allow negative constraint
forces. Instead, the adhesion generalized force in `qfrc_actuator` counteracts
gravity and other forces, keeping the body in contact.

##### S3. Call site in `mj_fwd_actuation()`

Call `mj_transmission_body()` during the **transmission computation phase** of
`mj_fwd_actuation()`, before the force application loop. Store the result in
`data.actuator_moment[actuator_id]`.

Then in Phase 3 (force application), handle the Body case like Site:

```rust
ActuatorTransmission::Body(_) => {
    for dof in 0..model.nv {
        let m = data.actuator_moment[i][dof];
        if m != 0.0 {
            data.qfrc_actuator[dof] += m * force;
        }
    }
}
```

Note: `actuator_length[i]` for body transmission is always 0 (adhesion has no
length, unlike tendons or joints).

##### S4. Model builder: remove error, resolve body name to ID

In `model_builder.rs`, replace the error at line ~1794 with body name resolution:

```rust
} else if let Some(ref body_name) = actuator.body {
    let body_id = self.resolve_body_id(body_name)?;
    transmission = ActuatorTransmission::Body(body_id);
    trnid = body_id;
}
```

##### S5. Interaction with `forcerange`

The adhesion model builder already sets up `forcerange` correctly (lines
~1876–1892). The scalar force `gain * ctrl` is clamped to `forcerange` in Phase 2
of `mj_fwd_actuation()` (line ~9975). No changes needed — the existing clamp
happens before the force is mapped through the moment arm.

The default adhesion setup uses `ctrl ∈ [0, 1]` with positive gain, so the scalar
force is always non-negative. The negative sign in the moment arm (S2) makes the
generalized force attractive.

##### S6. Contact count edge cases

- **No contacts**: If the target body has no active contacts, `moment` is zero,
  and the actuator produces no force. This is correct — adhesion requires contact.
- **Multiple contacts**: Forces are averaged across all contacts involving the body.
  This prevents the adhesion force from scaling with contact count.
- **Excluded contacts (gap zone)**: MuJoCo accumulates contributions from contacts
  in the margin gap zone into a separate `moment_exclude` buffer. Defer this to a
  stretch goal — it requires #27 (margin/gap runtime effect) to be implemented first.

#### Acceptance Criteria

1. **Loading**: `<actuator><adhesion body="box" gain="100"/>` loads without error
   for a model with a body named "box".
2. **Zero force when no contact**: With `ctrl=1.0` but the body not in contact,
   `qfrc_actuator` contribution from the adhesion actuator is zero.
3. **Attractive force**: A sphere resting on a plane with adhesion `ctrl=1.0`
   produces a downward `qfrc_actuator` (pulling sphere toward plane). Verify
   `qfrc_actuator` has the correct sign by checking that the sphere's equilibrium
   position is lower (more penetrated) than without adhesion.
4. **Force magnitude**: For a single contact, the generalized force equals
   `-(gain * ctrl) * J_normal`. Compare against MuJoCo 3.4.0 `qfrc_actuator`
   for a sphere-on-plane adhesion scenario. Tolerance: `1e-10`.
5. **Multiple contacts**: With 4 contacts (box on plane), moment arm is the
   average of 4 normal Jacobians. Compare against MuJoCo 3.4.0.
6. **Regression**: Models without adhesion actuators produce identical results
   (no force application code path is touched).
7. **`actuator_moment` populated**: After `forward()`, `data.actuator_moment[i]`
   for a body-transmission actuator is nonzero when contacts exist and zero
   when no contacts exist.

#### Implementation Notes

**Contact normal Jacobian reuse.** The constraint assembly already computes contact
normal Jacobians for the constraint system (`efc_J`). However, `mj_transmission_body()`
runs during `mj_fwd_actuation()`, which is called *before* constraint assembly in the
pipeline. So we cannot reuse `efc_J` — we must compute the Jacobian independently.
This matches MuJoCo's approach (transmission runs in the smooth phase, before
constraints).

Use the existing `mj_jac()` or `jac_body_com()` functions to compute the body
Jacobians at the contact point, then difference them and project along the contact
normal.

**Pipeline ordering.** MuJoCo's pipeline: `mj_fwd_position` → `mj_fwd_velocity` →
`mj_fwd_actuation` → `mj_fwd_acceleration` → `mj_fwd_constraint`. Body transmission
runs in `mj_fwd_actuation`, which means it uses contacts from the *previous* step
(contacts are detected in `mj_fwd_position` → `mj_collision`). This is the correct
order — contacts are already populated when `mj_fwd_actuation` runs.

**Gear parameter.** The adhesion shortcut sets `gear[0] = 1.0` by default. The
`gear` value scales the moment arm in `mj_transmission_body()`. For multi-axis
gear (not needed for adhesion), only `gear[0]` is used.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — add `Body(usize)` to
  `ActuatorTransmission`, implement `mj_transmission_body()`, add Body case
  to Phase 3 force application in `mj_fwd_actuation()`
- `sim/L0/mjcf/src/model_builder.rs` — replace error with body name resolution
  in `process_actuator()`
- `sim/L0/tests/integration/` — new test file `adhesion_actuator.rs` or add to
  existing `actuator_tests.rs`

---

### 37. Tendon Equality Constraints
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Pipeline emits a runtime warning and silently ignores `EqualityType::Tendon`
constraints. Any model using `<equality><tendon>` gets incorrect behavior — the
constraint is simply not enforced. Standalone `TendonConstraint`/`TendonNetwork`
exists in sim-constraint but is not wired into the pipeline.

#### Objective
Implement tendon equality constraints in the pipeline constraint assembly.

#### Specification

1. **Constraint type**: `tendon1.length - tendon2.length = polycoef(tendon1.length)`
   where `polycoef` is a polynomial (up to degree 4) mapping reference tendon length
   to target offset (MuJoCo semantics).
2. **Assembly**: In `mj_fwd_constraint()`, for each `EqualityType::Tendon`:
   - Compute constraint violation: `c = L1 - polycoef(L1) - L2`
   - Compute Jacobian: `J = J_tendon1 - dpolycoef/dL1 * J_tendon1 - J_tendon2`
   - Add row to equality constraint block (before contact constraints)
3. **Solver**: Equality constraints are unclamped in PGS/Newton (no projection needed).
4. **solref/solimp**: Apply per-constraint solver parameters for stabilization.

#### Acceptance Criteria
1. Two tendons coupled by `<equality><tendon>` maintain the specified length relationship.
2. `polycoef` polynomial evaluation matches MuJoCo for degree 0–4.
3. Removing the equality constraint reproduces uncoupled behavior (regression).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_constraint()`, equality block assembly
