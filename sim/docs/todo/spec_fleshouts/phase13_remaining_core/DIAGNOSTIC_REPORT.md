# Phase 13 Session 0 — Diagnostic Report

**Date:** 2026-03-10
**Branch:** `phase13-remaining-core`
**Model:** `flag_golden_test.xml` (2-link arm + free ball, nv=8, nq=9)

---

## Executive Summary

The ~1.69 qacc divergence reported in the Phase 12 gate assessment has been
reduced to ~2e-2 by Phase 12 Sessions 14-15 fixes. The remaining divergence
traces to a **single root cause**: `tendon_invweight0` is computed using a
diagonal-only approximation instead of the full `J · M⁻¹ · J^T` quadratic
form. This causes incorrect `diagApprox` → `R` → `D` for the tendon friction
loss constraint row, which propagates through the Newton solver to affect all
DOFs.

The Spec A/B grouping hypothesis is **validated with modification**:
- **Spec A** should fix the `tendon_invweight0` bug (DT-39) — this is the
  dominant root cause of the baseline divergence.
- **Spec B** (DT-128/129 solver items) remain as-specified — they affect
  convergence behavior but are not the primary divergence source.
- **FILTERPARENT** (~51.9 divergence) is an **independent collision filtering
  issue** — assigned to new DT-131 in Spec A scope (assembly-side).

---

## Model Constraint Inventory

The `flag_golden_test.xml` model produces **7 constraint rows** at initial
state (`forward()` with `ctrl=2.0`):

| Row | Type | Description | efc_id |
|-----|------|-------------|--------|
| 0 | Equality | Joint coupling: hinge2 = 0.5 * hinge1 | 0 |
| 1 | FrictionLoss | DOF friction: hinge1 frictionloss=0.5 | 0 |
| 2 | FrictionLoss | Tendon friction: tendon0 frictionloss=0.3 | 0 |
| 3 | ContactPyramidal | Ball-floor contact, facet 0 (+n+μf₁) | 0 |
| 4 | ContactPyramidal | Ball-floor contact, facet 1 (+n-μf₁) | 0 |
| 5 | ContactPyramidal | Ball-floor contact, facet 2 (+n+μf₂) | 0 |
| 6 | ContactPyramidal | Ball-floor contact, facet 3 (+n-μf₂) | 0 |

Constraint ordering: `ne=1` (equality), `nf=2` (friction loss), 4 contact
rows. Solver: Newton (default).

---

## Field-by-Field Comparison

### Fields that match (all rows, within 1e-6)

| Field | Status | Notes |
|-------|--------|-------|
| `dof_invweight0` | MATCH | All 8 DOFs match MuJoCo exactly |
| `body_invweight0` | MATCH | (inferred from dof_invweight0 correctness) |
| `qM` diagonal | MATCH | Mass matrix diagonal identical |
| `qacc_smooth` | MATCH | Unconstrained acceleration identical |
| `qfrc_smooth` | MATCH | Smooth forces identical |
| `efc_J` (all rows) | MATCH | Jacobians identical for all constraint rows |
| `efc_pos` (all rows) | MATCH | Constraint violations identical |
| Rows 0-1 `diagApprox` | MATCH | Equality + DOF friction rows correct |
| Rows 0-1 `R`, `D` | MATCH | Regularization correct for rows 0-1 |
| Rows 3-6 `R`, `D` | MATCH | Contact regularization correct (Rpy post-processing works) |
| Rows 3-6 `efc_force` | MATCH | Contact forces correct |

### Fields that diverge

| Row | Field | CortenForge | MuJoCo | Diff | Root Cause |
|-----|-------|-------------|--------|------|------------|
| 2 | `efc_diagApprox` | 5298.63 | 5388.92 | **90.29** | `tendon_invweight0` diagonal-only bug |
| 2 | `efc_R` | 588.74 | 598.77 | 10.03 | Cascades from diagApprox |
| 2 | `efc_D` | 1.699e-3 | 1.670e-3 | 2.85e-5 | Cascades from R |
| 2 | `efc_force` | 2.709e-3 | 2.694e-3 | 1.50e-5 | Cascades from R/D |
| 0 | `efc_force` | -7.573e-2 | -7.572e-2 | 1.36e-5 | Newton solver coupling with row 2 |
| 3-6 | `efc_diagApprox` | 4.0 | 8.0 | 4.0 | Cosmetic (see below) |

### Final qacc divergence per DOF

| DOF | CortenForge | MuJoCo | Diff |
|-----|-------------|--------|------|
| 0 (hinge1) | 90.764 | 90.766 | **2.14e-3** |
| 1 (hinge2) | 92.359 | 92.379 | **2.03e-2** |
| 2 (free tx) | 1.57e-5 | ~0 | 1.57e-5 |
| 3 (free ty) | -2.36e-5 | ~0 | 2.36e-5 |
| 4 (free tz) | 2.0095 | 2.0095 | <1e-15 |
| 5 (free rx) | 4.76e-4 | ~0 | 4.76e-4 |
| 6 (free ry) | 3.16e-4 | ~0 | 3.16e-4 |
| 7 (free rz) | 0.0 | 0.0 | 0.0 |

DOF 1 (hinge2) has the largest divergence because the tendon friction loss
row directly involves both hinge DOFs, and hinge2 has the smallest mass
(highest sensitivity to force errors).

---

## Root Cause #1: `tendon_invweight0` Diagonal-Only Approximation

**File:** `sim/L0/core/src/types/model_init.rs:1062-1074`
**Severity:** Primary root cause of baseline qacc divergence

### The Bug

```rust
// Current code (WRONG):
TendonType::Fixed => {
    let mut w = 0.0;
    for wr in adr..(adr + num) {
        if self.wrap_type[wr] == WrapType::Joint {
            let dof_adr = self.wrap_objid[wr];
            if dof_adr < self.nv {
                let coef = self.wrap_prm[wr];
                w += coef * coef * self.dof_invweight0[dof_adr];
            }
        }
    }
    self.tendon_invweight0[t] = w.max(MIN_VAL);
}
```

This computes: `Σ(coef_i² · (M⁻¹)[i,i])` — **diagonal only**.

MuJoCo computes: `J_tendon · M⁻¹ · J_tendon^T` — **full quadratic form**.

### Numeric Example

For the flag model tendon (coefs: hinge1=1.0, hinge2=-1.0):

| Formula | Value | Components |
|---------|-------|------------|
| CortenForge (diagonal) | 5298.63 | 1²×45.15 + 1²×5253.48 |
| MuJoCo (full) | 5388.92 | 1²×45.15 + 2×1×(-1)×(-45.15) + 1²×5253.48 |
| Missing term | 90.29 | 2 × c₀ × c₁ × M⁻¹[0,1] = 2×1×(-1)×(-45.15) |

The off-diagonal M⁻¹[0,1] = -45.15 (coupling between hinge1 and hinge2 due
to the serial kinematic chain) is entirely missing from the CortenForge
computation.

### Fix Required (Spec A, DT-39)

Build a tendon Jacobian vector `J_tendon` (nv×1) from the wrap coefficients,
solve `M · w = J_tendon` using the already-factored sparse LDL, and compute
`tendon_invweight0[t] = J_tendon^T · w`. This matches the approach already
used for `body_invweight0` and `dof_invweight0` in the same function.

### Cascade

```
tendon_invweight0 (wrong)
  → diagApprox for tendon friction loss row (row 2): 5298 vs 5389
    → impedance K, B, I, P (slightly wrong)
      → R (regularization): 588.7 vs 598.8
        → D = 1/R: also wrong
          → efc_force for row 2: wrong
            → Newton solver: row 2 force error couples to all other rows
              → efc_force for row 0 (equality): also wrong (1.36e-5)
                → qfrc_constraint = J^T · force: wrong
                  → qacc = qacc_smooth + M⁻¹ · qfrc_constraint: wrong
```

---

## Root Cause #2: Pyramidal `diagApprox` Factor-of-2 (Cosmetic)

**Rows 3-6:** CF=4.0, MJ=8.0. **Not a real bug.**

CortenForge computes: `(1+μ²) × bw_contact[0]` = (1+1)×2.0 = 4.0
MuJoCo stores: 8.0 (includes a factor-of-2 in intermediate representation)

However, **the final `efc_R` values match** for all pyramidal facet rows.
This is because the Rpy post-processing (`Rpy = 2μ²·R_first_facet`, line 789
of `assembly.rs`) overrides the per-row R computed from diagApprox. The
diagApprox intermediate is never used after R-scaling for pyramidal contacts.

**No fix needed.** The mismatch is in an intermediate value that does not
affect any downstream computation. The R/D/force/qacc all match for contact
rows.

---

## FILTERPARENT Analysis

**Gate assessment divergence:** ~51.9 (pre-Phase-12 fixes)
**Current actual divergence:** ~0.020 (same order as baseline)
**Classification:** NOT an independent bug — same tendon_invweight0 root cause

### Evidence

Running the golden flag tests post-Phase-12 reveals that FILTERPARENT's
divergence is ~0.020, virtually identical to baseline (~0.0021). The ~51.9
from the gate assessment was measured before Phase 12 Sessions 14-15 fixes
(invweight0 rewrite, contact ordering, etc.). Those fixes resolved the
collision filtering discrepancy.

**DT-131 is unnecessary.** FILTERPARENT does not require separate treatment.
It will be fixed by the same `tendon_invweight0` fix as all other tests.

### Updated gate assessment divergences (actual, post-Phase-12)

| Test | Gate Assessment | Current Actual |
|------|----------------|----------------|
| baseline | ~1.69 | 0.0021 |
| disable_constraint | ~25.9 | **PASS** |
| disable_equality | ~0.002 | 0.0069 |
| disable_frictionloss | ~3.99 | **PASS** |
| disable_filterparent | ~51.9 | 0.020 |
| disable_gravity | ~0.40 | 0.0011 |
| disable_clampctrl | ~2.48 | 0.0018 |
| disable_warmstart | ~1.69 | 0.0021 |

All failures are in the 0.001–0.02 range, consistent with a single root
cause (tendon_invweight0). Two tests already pass in scenarios where the
tendon friction row is absent.

---

## Predicted Fix Impact

### After Spec A (tendon_invweight0 fix)

Fixing `tendon_invweight0` to use full `J·M⁻¹·J^T` should:
1. Make row 2 `diagApprox` match exactly (5388.92)
2. Make row 2 `R`, `D`, `efc_force` match
3. Eliminate the Newton solver coupling error on row 0
4. Bring qacc divergence down to solver-precision level (<1e-10)

**Predicted golden flag test outcomes:**

All 24 failing tests have divergences in the 0.001–0.02 range, consistent
with a single root cause. Two tests already pass (`disable_constraint`,
`disable_frictionloss`) in scenarios where the tendon friction row is absent.

**Prediction: 26/26 pass after Spec A.** The `tendon_invweight0` fix is the
only remaining blocker. No secondary root causes detected.

### After Spec B (solver items)

If any tests still fail after Spec A due to solver convergence:
- DT-128 (early termination) ensures the solver converges to the same
  tolerance as MuJoCo
- DT-129 (warmstart projection) ensures initial force estimates match

---

## Spec A/B Grouping: Validated (Simplified)

The diagnostic validates the original hypothesis, but the scope is **simpler
than expected**:

| Spec | Items | Expected Impact |
|------|-------|-----------------|
| **A** | DT-39 (tendon_invweight0 fix), DT-23 (DOF friction params verify) | **All 26 tests** |
| **B** | DT-19 (QCQP verify), DT-128 (early termination), DT-129 (warmstart) | Correctness + convergence improvements; no blocking test failures |

**FILTERPARENT is NOT a separate issue.** DT-131 is unnecessary — the ~51.9
divergence from the gate assessment was a pre-Phase-12 artifact. Current
FILTERPARENT divergence is ~0.020, same root cause as baseline.

**DT-23** (per-DOF friction solver params) was not observed as a divergence
source. Row 1 (DOF friction loss) matches within tolerance. DT-23 should
still be verified during Spec A but is likely already correct for the flag
model's code paths.

---

## Diagnostic Artifacts

| Artifact | Location |
|----------|----------|
| Rust diagnostic test | `sim/L0/tests/integration/phase13_diagnostic.rs` |
| Python MuJoCo dump script | `sim/L0/tests/scripts/dump_constraint_diagnostic.py` |
| MuJoCo reference .npy files | `sim/L0/tests/assets/golden/flags/diagnostic/step0_pre/` |
| MuJoCo FILTERPARENT .npy files | `sim/L0/tests/assets/golden/flags/diagnostic/step0_filterparent/` |

---

## Next Steps

1. **Session 1:** Write Spec A rubric + spec, informed by this report
2. **Session 2:** Implement Spec A (primary: tendon_invweight0 full M⁻¹ solve)
3. **Session 4:** Run golden flag checkpoint after Spec A
4. If >2 tests still fail: investigate and update Spec B scope
