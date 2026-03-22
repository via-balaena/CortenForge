# Pyramidal Friction Instability on Convex-Convex Contacts

**Status:** Investigation complete. Fix not yet implemented.
**Date:** 2026-03-22
**Example:** `examples/sdf-physics/07-pair` (two SDF spheres stacked)
**Branch:** `examples-continued`

## Symptom

Two SDF spheres (radius 5mm) stacked vertically tunnel through each other
after ~1.2 seconds. The upper sphere drifts laterally with exponential growth
(doubling every ~17ms), then slides off and falls through.

## Root Cause Chain

### 1. Pyramidal facet force asymmetry (1 ULP)

Pyramidal friction (cone=0) creates two facets per friction direction:
```
J_pos = J_normal + mu * J_tangent
J_neg = J_normal - mu * J_tangent
```

The Delassus matrix `A = J * M^-1 * J^T` has subtly different entries for
pos vs neg facets because the angular Jacobian terms differ (IEEE 754
rounding in cross-product and lever-arm computations). The Newton solver
faithfully optimizes the QP with these slightly-different matrix entries,
producing forces that differ by ~1 ULP:

```
f_pos = 2.46702693094978498
f_neg = 2.46702693094978232
diff  = 2.66e-15
```

Verified at `sim/L0/tests/integration/collision_primitives.rs:sphere_stack_dynamic_sdf`
by printing `efc_J[(row, 7)]` and `efc_force[row]` at step 20.

### 2. Net lateral force from asymmetric facets

The contact Jacobian for DOF 7 (upper body y-linear) is exactly +1.0 for
the positive facet and -1.0 for the negative facet. With different forces:

```
qfrc_constraint[7] = (+1.0) * f_pos + (-1.0) * f_neg = 2.66e-15
```

This produces lateral acceleration: `qacc[7] = 2.66e-15 / 0.000655 = 4.06e-12`

### 3. Geometric amplification (unstable equilibrium)

Ball-on-ball is inherently unstable. Any lateral displacement tilts the
contact normal, reducing the vertical force component and adding a lateral
component. This creates a positive feedback loop:

```
tiny lateral drift -> tilted normal -> reduced vertical support + lateral push
-> more drift -> more tilt -> exponential growth
```

Growth rate: ~26/s (from contact stiffness, much faster than gravitational
instability alone at ~1/s). Time from 1e-15 seed to macroscopic failure:

```
t = ln(0.001 / 1e-15) / 26 = 1.06 seconds
```

This matches the observed failure at ~1.2s.

## Approaches Tried (and why they failed)

### A. Post-solve facet force averaging

**Approach:** After solver converges, average (f_pos + f_neg) / 2 for each
facet pair when the force ratio `|f_pos - f_neg| / (f_pos + f_neg)` is small.

**Result:** Successfully zeros the tangential force in `efc_force`. BUT:

1. **Newton's qacc carries the bias too.** Newton solves for qacc as a primal
   variable — it's not derived from efc_force. The qacc has the same lateral
   component as the asymmetric forces. Fixing efc_force alone doesn't fix qacc.

2. **Explicit qacc recomputation (`newton_solved = false`) introduces its own
   rounding.** The explicit path `qacc = M^-1 * (qfrc_smooth + qfrc_constraint)`
   uses the sparse LDL factorization. Even with qfrc_constraint[7] = 0 exactly,
   the sparse solve produces O(eps) lateral qacc from accumulated rounding.
   This delays failure to ~2.6s but doesn't prevent it.

3. **Direct delta correction (`qacc += delta/M_diag`) is fragile.** Computing
   the delta from qfrc_before and qfrc_after is correct in principle, but the
   delta is so tiny (1e-15) that any additional rounding in the correction
   pathway can produce a LARGER lateral acceleration than the original.

**Conclusion:** Post-processing the solver output cannot achieve the required
precision (< 1e-15 lateral force per step) because every computation path
through the mass matrix introduces its own O(eps) rounding.

### B. Increased force-ratio threshold

Tried thresholds of 1e-6, 0.01, and 0.1. Higher thresholds delay failure
(more aggressive averaging catches the growing asymmetry) but eventually
the geometric instability overcomes the threshold and enforcement stops.

## Correct Fix: Contact Normal Stabilization

The fix must happen at the **contact model level**, before the solver runs.

### The insight

The instability is geometric: the contact normal tilts as the sphere drifts,
creating a positive feedback loop. For analytical convex-convex contacts
(sphere-sphere), the contact normal is `(B_center - A_center) / |B - A|`.
This is correct but UNSTABLE — any lateral offset immediately changes the
normal direction.

### The fix

For the analytical path in `compute_shape_contact()` (file:
`sim/L0/core/src/sdf/shape.rs`), when both shapes return `effective_radius`,
the contact normal should be **projected to remove tangential drift**:

1. Compute the nominal separation axis (e.g., from initial contact geometry
   or from the gravity direction for stacking scenarios)
2. Project the computed normal onto this axis
3. Use the projected normal for the contact Jacobian

More concretely: the contact Jacobian's tangent frame should be constructed
to be **orthogonal to the true center-to-center direction**, not to a
potentially drifted direction. If the center-to-center direction has
accumulated floating-point lateral drift, the tangent frame should be
stabilized.

### Alternative approach: condim=1 for apex contacts

For sphere-on-sphere contacts where the contact is at the apex (top of lower
sphere), friction serves no physical purpose — there's no sliding to resist
at an unstable equilibrium. Setting condim=1 (frictionless) for such contacts
eliminates the tangential force channel entirely.

This is simpler but less general. The normal stabilization approach handles
all convex-convex stacking (including non-spherical shapes).

## Key Files

| File | Role |
|------|------|
| `sim/L0/core/src/sdf/shape.rs:52-85` | `compute_shape_contact()` — analytical contact path |
| `sim/L0/core/src/sdf/operations.rs:54-57` | `separation_direction()` — normal computation |
| `sim/L0/core/src/types/contact_types.rs:334-366` | `compute_tangent_frame()` — tangent vectors |
| `sim/L0/core/src/constraint/contact_assembly.rs:221-313` | `assemble_pyramidal_contact()` — facet Jacobians |
| `sim/L0/core/src/constraint/solver/pgs.rs:467-478` | Pyramidal facet classification |
| `sim/L0/core/src/constraint/solver/newton.rs` | Newton solver (produces qacc + efc_force) |
| `sim/L0/core/src/constraint/mod.rs:409-449` | Solver dispatch in mj_fwd_constraint |

## Diagnostic Data

### Per-step lateral drift (no fix, headless test at step 20)
```
row=4 type=ContactPyramidal J7=+1.0  f=2.46702693094978498  J7*f=+2.467e0
row=5 type=ContactPyramidal J7=-1.0  f=2.46702693094978232  J7*f=-2.467e0
total qfrc_constraint[7] = 2.66e-15
```

### Exponential drift timeline (original, no fix)
```
t=0.90: y=-0.000001  vy=-0.000036
t=1.00: y=-0.000021  vy=-0.000574
t=1.10: y=-0.000328  vy=-0.009013
t=1.20: y=-0.005291  vy=-0.145406
t=1.30: y=-0.083031  vy=-2.281543
t=1.40: y=-1.314121  vy=-35.738247
t=1.45: y=-4.864370  vy=-114.216230  (catastrophic tunneling)
```

### Contact normal tilt during failure
```
t=1.19: n=(0.00, -0.00, 1.00)  — normal starts pure vertical
t=1.28: n=(0.00, -0.01, 1.00)  — first visible tilt
t=1.39: n=(0.00, -0.13, 0.99)
t=1.43: n=(0.00, -0.32, 0.95)
t=1.47: n=(0.00, -0.94, 0.35)  — nearly horizontal, catastrophic
```
