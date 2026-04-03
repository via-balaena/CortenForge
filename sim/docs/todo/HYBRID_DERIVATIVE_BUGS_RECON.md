# Hybrid Derivative Bugs — Reconnaissance

**Date:** 2026-04-03
**Found by:** Derivatives stress-test example development
**Status:** Recon complete — ready for fix spec

---

## Summary

Two bugs where `mjd_transition_hybrid` disagrees with `mjd_transition_fd`:

1. **B matrix for actuated systems** — relative error 1.0 — **ROOT CAUSE CONFIRMED**
2. **A matrix for ball joints** — relative error ~1.0 — needs deeper investigation

Both are untested by the existing 52-test suite. Both affect the
recommended fast path for control/optimization workflows (iLQR, MPC, DDP).

---

## Bug 1: `actuator_moment` not populated for Joint/Tendon transmissions

### Root cause (confirmed)

`data.actuator_moment` is a `Vec<DVector<f64>>` initialized to zero vectors
(`model_init.rs:460`). Only **Site**, **Body**, and **SliderCrank**
transmissions populate it during `forward()`. **Joint** and **Tendon**
transmissions never write to it — they apply forces inline:

```rust
// actuation.rs:665-670 — Joint transmission bypasses actuator_moment
ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
    data.qfrc_actuator[dof_adr] += gear * force;
}
```

The hybrid derivative code reads `actuator_moment` for ALL transmission
types, getting zeros for Joint/Tendon:

```rust
// hybrid.rs:2453 — B matrix, reads zero for Joint transmissions
let moment = &data.actuator_moment[actuator_idx];

// hybrid.rs:1935 — activation columns of A, same bug
let moment = &data.actuator_moment[actuator_idx];
```

Meanwhile, `mjd_actuator_vel` (hybrid.rs:206-210) and `mjd_actuator_pos`
correctly dispatch by transmission type — they use `gear` directly for
Joint transmissions. **The B matrix and activation column code paths missed
this dispatch.**

### Affected code paths (3 sites)

| File | Line | Context | Broken for Joint/Tendon? |
|------|------|---------|--------------------------|
| `hybrid.rs` | 2453 | B matrix: `∂state/∂ctrl` | **YES** — zeros |
| `hybrid.rs` | 1935 | A activation cols: `∂state/∂act` | **YES** — zeros |
| `hybrid.rs` | 232 | `mjd_actuator_vel`: `∂qfrc/∂qvel` | No — correctly dispatches |
| `hybrid.rs` | 960 | `mjd_actuator_pos`: `∂qfrc/∂qpos` | No — correctly dispatches |

### Correct pattern (from `mjd_actuator_vel`, hybrid.rs:205-247)

```rust
match model.actuator_trntype[i] {
    ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
        let dof_adr = model.jnt_dof_adr[trnid];
        // moment = gear, applied at dof_adr only
        data.qDeriv[(dof_adr, dof_adr)] += gear * gear * dforce_dv;
    }
    ActuatorTransmission::Tendon => {
        let j = &data.ten_J[trnid];
        // moment = gear * J^T
        let scale = gear * gear * dforce_dv;
        for r in 0..nv { for c in 0..nv {
            data.qDeriv[(r, c)] += scale * j[r] * j[c];
        }}
    }
    ActuatorTransmission::Site | ... => {
        let moment = &data.actuator_moment[i];
        // use cached moment vector
    }
}
```

### Fix approach

Add the same transmission-type dispatch to the B matrix (line 2453) and
activation column (line 1935) code paths. For Joint transmissions, build
the moment vector inline:

```rust
// For Joint: moment is a one-hot vector with gear at dof_adr
let mut moment = DVector::zeros(nv);
moment[dof_adr] = gear;
```

For Tendon: use `gear * ten_J[tendon_id]`.

### Reproduction

```rust
let model = multi_actuator(); // 3 hinges, 2 motors (Joint transmission)
let mut data = model.make_data();
data.qpos[0] = 0.3;
data.forward(&model).unwrap();

let fd = mjd_transition_fd(&model, &data, &config).unwrap();
let hyb = mjd_transition_hybrid(&model, &data, &config).unwrap();
max_relative_error(&fd.B, &hyb.B, 1e-10); // → (1.0, (0, 0))
// Hybrid B is all zeros; FD B has correct nonzero entries.
```

### Impact

- Every motor, position servo, velocity servo, and damper actuator using
  Joint transmission (the most common type) produces zero B columns in
  hybrid mode.
- `validate_analytical_vs_fd` catches this: `err_B = 1.0`.
- Any iLQR/MPC using `mjd_transition_hybrid` for actuated systems with
  Joint-transmission actuators gets zero control influence — the
  controller thinks controls have no effect.

---

## Bug 2: Hybrid A matrix wrong for ball joints

### Status: Deeper investigation needed

Error ~1.0 between hybrid and FD A matrices for ball joint with angular
velocity. The velocity Jacobian in `mjd_quat_integrate` returns `h*I`
(line 67 of integration.rs). The math comment derives this from
`η = log(exp(0)·exp(h·ω)) = h·ω`, but this measures tangent from
**identity**, not from the **nominal output** as the FD convention does.

The correct formula is `h * J_r^{-1}(h·ω)`, but at h=0.002 and
moderate ω, the correction is ~1e-6 — too small to explain error=1.0.

### What to investigate next

The dominant error source is still unidentified. Candidates:
1. `mjd_smooth_vel` not correctly computing `qDeriv` for ball joints
   (Coriolis/gyroscopic derivative `∂(ω × I·ω)/∂ω`).
2. `mjd_rne_vel` ball joint code path.
3. The position column FD in hybrid might have a quaternion tangent-space
   mapping issue.

**Propose:** Fix Bug 1 first (confirmed, simple), then use the improved
test infrastructure to isolate Bug 2.

---

## Existing test gaps

| Scenario | Covered? |
|----------|----------|
| Hybrid vs FD B for Joint-transmission actuators | **NO** |
| Hybrid vs FD B for Tendon-transmission actuators | **NO** |
| Hybrid vs FD B for Site-transmission actuators | **NO** |
| Hybrid vs FD A activation cols for Joint-transmission | **NO** |
| Hybrid vs FD A for ball joint with nonzero ω | **NO** |

---

## Files involved

| File | Lines | Issue |
|------|-------|-------|
| `hybrid.rs` | 2453 | B matrix reads zero `actuator_moment` for Joint |
| `hybrid.rs` | 1935 | Activation column reads zero `actuator_moment` for Joint |
| `integration.rs` | 64-67 | Velocity Jacobian `h*I` (minor, correct to O(h²·ω²)) |
| `actuation.rs` | 665-670 | Reference: how Joint forces are correctly applied |
| `hybrid.rs` | 205-247 | Reference: how `mjd_actuator_vel` correctly dispatches |
