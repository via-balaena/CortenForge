# Fix: Hybrid Derivative Moment Dispatch for Joint/Tendon Transmissions

**Date:** 2026-04-03
**Prereq:** Recon in `HYBRID_DERIVATIVE_BUGS_RECON.md`
**Status:** Ready for implementation

---

## Problem

The hybrid derivative path reads `data.actuator_moment[i]` when computing
B matrix columns (∂state/∂ctrl) and A activation columns (∂state/∂act).
For Joint and Tendon transmissions — the most common types — this vector
is **never populated** and remains zero. The result: hybrid B is all zeros
for motor/position/velocity/damper actuators, and activation columns are
all zeros for filter/integrator actuators on joints.

`mjd_actuator_vel` and `mjd_actuator_pos` already handle this correctly
by dispatching on `model.actuator_trntype[i]`. The B matrix and activation
column code paths missed this dispatch.

---

## Fix

### Site 1: B matrix — `hybrid.rs:2433-2495`

**Current code (broken):**
```rust
// hybrid.rs:2453
let moment = &data.actuator_moment[actuator_idx];
let mut dvdctrl = DVector::zeros(nv);
for dof in 0..nv {
    dvdctrl[dof] = h * gain * moment[dof];
}
```

**Fixed code:** Replace the moment lookup with a transmission-type dispatch
matching the pattern in `mjd_actuator_vel` (hybrid.rs:205-247):

```rust
let gear = model.actuator_gear[actuator_idx][0];
let trnid = model.actuator_trnid[actuator_idx][0];

let mut dvdctrl = DVector::zeros(nv);
match model.actuator_trntype[actuator_idx] {
    ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
        if trnid < model.njnt {
            let dof_adr = model.jnt_dof_adr[trnid];
            let nv_jnt = model.jnt_type[trnid].nv();
            // Joint: moment is one-hot with gear at dof_adr
            for k in 0..nv_jnt {
                dvdctrl[dof_adr + k] = h * gain * gear;
            }
        }
    }
    ActuatorTransmission::Tendon => {
        if trnid < model.ntendon {
            let j = &data.ten_J[trnid];
            // Tendon: moment = gear * J
            for dof in 0..nv {
                dvdctrl[dof] = h * gain * gear * j[dof];
            }
        }
    }
    ActuatorTransmission::Site
    | ActuatorTransmission::Body
    | ActuatorTransmission::SliderCrank => {
        let moment = &data.actuator_moment[actuator_idx];
        for dof in 0..nv {
            dvdctrl[dof] = h * gain * moment[dof];
        }
    }
}
```

The rest of the code (M^{-1} solve, chain rule, B fill) stays the same.

### Site 2: A activation columns — `hybrid.rs:1934-1939`

**Current code (broken):**
```rust
// hybrid.rs:1935
let moment = &data.actuator_moment[actuator_idx];
let mut dvdact = DVector::zeros(nv);
for dof in 0..nv {
    dvdact[dof] = h * gain * moment[dof];
}
```

**Fixed code:** Same dispatch pattern as Site 1:

```rust
let gear = model.actuator_gear[actuator_idx][0];
let trnid = model.actuator_trnid[actuator_idx][0];

let mut dvdact = DVector::zeros(nv);
match model.actuator_trntype[actuator_idx] {
    ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
        if trnid < model.njnt {
            let dof_adr = model.jnt_dof_adr[trnid];
            let nv_jnt = model.jnt_type[trnid].nv();
            for k in 0..nv_jnt {
                dvdact[dof_adr + k] = h * gain * gear;
            }
        }
    }
    ActuatorTransmission::Tendon => {
        if trnid < model.ntendon {
            let j = &data.ten_J[trnid];
            for dof in 0..nv {
                dvdact[dof] = h * gain * gear * j[dof];
            }
        }
    }
    ActuatorTransmission::Site
    | ActuatorTransmission::Body
    | ActuatorTransmission::SliderCrank => {
        let moment = &data.actuator_moment[actuator_idx];
        for dof in 0..nv {
            dvdact[dof] = h * gain * moment[dof];
        }
    }
}
```

### Helper extraction (optional)

Both sites share the same dispatch logic. If desired, extract a helper:

```rust
/// Build the moment-scaled force vector for an actuator.
///
/// For Joint/Tendon transmissions, constructs the moment inline from
/// gear and joint/tendon Jacobians. For Site/Body/SliderCrank, reads
/// the cached `data.actuator_moment` vector.
fn build_moment_scaled_force(
    model: &Model,
    data: &Data,
    actuator_idx: usize,
    scale: f64,  // h * gain
    nv: usize,
) -> DVector<f64> { ... }
```

This keeps both call sites as one-liners. Whether to extract is a
judgment call — two copies of the same match is acceptable for clarity.

---

## Verification

### Import needed

`validate_analytical_vs_fd` is not currently imported in the test file
but is exported from `sim_core`. Add it to the existing `use sim_core`
block.

### Unit tests to add (in `sim/L0/tests/integration/derivatives.rs`)

1. **`test_hybrid_vs_fd_B_joint_motor`** — 3-hinge pendulum with 2 motor
   actuators (Joint transmission). Compute hybrid B and FD B.
   Assert `max_relative_error(B_fd, B_hyb) < 1e-4`.

2. **`test_hybrid_vs_fd_B_tendon_actuator`** — model with tendon
   transmission actuator. Same comparison, same tolerance.

3. **`test_hybrid_vs_fd_act_col_joint_filter`** — pendulum with
   `<general joint="..." dyntype="filter">` actuator (Joint transmission
   with activation). Compute hybrid A and FD A. Compare the activation
   columns. Assert `max_relative_error < 1e-4` on the activation block.

4. **`test_validate_analytical_vs_fd_actuated`** — multi-actuator model
   with Joint transmission. Call `validate_analytical_vs_fd`. Assert
   both `err_A < 1e-3` and `err_B < 1e-3`.

### Stress test update

Update the derivatives stress test (`example-derivatives-stress-test`)
to re-enable the hybrid-vs-FD B comparison that was softened during
initial development:

- Check 12: compare hybrid B vs FD B directly (not just A)
- Tighten to `max_relative_error(B_fd, B_hyb) < 1e-4`

### Regression scope

```bash
cargo test -p sim-core -p sim-conformance-tests
```

No other crates affected — the fix is internal to `hybrid.rs`.

---

## Non-goals

- **No changes to `actuator_moment` population.** The forward pass
  correctly skips moment caching for Joint/Tendon because they don't
  need it for force application. The fix is in the derivative code, not
  the forward pass.
- **No changes to `mjd_actuator_vel` or `mjd_actuator_pos`.** These
  already dispatch correctly.
- **No changes to the FD path.** It's the reference truth.
- **Ball joint hybrid A (Bug 2)** is a separate issue with a separate spec.
