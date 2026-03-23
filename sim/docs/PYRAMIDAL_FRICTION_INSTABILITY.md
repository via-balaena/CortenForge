# Pyramidal Friction Instability on Convex-Convex Contacts

**Status:** RESOLVED (2026-03-22). Two-part fix: normal stabilization + lever arm stabilization.
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

## Implemented Fix (two parts, both required)

### Part 1: Contact normal stabilization

`stabilize_direction()` in `sim/L0/core/src/sdf/operations.rs` zeros
direction-vector components below `1e-6 * max_component`, then re-normalizes.
Applied in the analytical path of `compute_shape_contact()` (shape.rs).

This prevents the contact normal from tilting due to sub-physical lateral
drift, which would give the constraint force a lateral component.

**Necessary but not sufficient.** Testing showed that normal stabilization
alone still produces 26/s exponential drift from a deeper mechanism (Part 2).

### Part 2: Lever arm stabilization

`stabilize_lever_arm()` in `sim/L0/core/src/constraint/jacobian.rs` projects
the lever arm (`r = contact_point - body_COM`) onto the contact normal axis
when its perpendicular component is sub-physical (< 1e-6 relative).

**The deep mechanism:** The Delassus matrix diagonal for pyramidal facet
pairs has a cross-term `(r × normal) · (r × tangent)` that differs between
pos and neg facets by an amount proportional to the lever arm's lateral
component. This is a mathematical asymmetry (not floating-point rounding).
When the lateral component grows from position drift, the Delassus asymmetry
grows, producing force asymmetry proportional to offset → exponential drift.

Projecting `r` onto the normal axis makes `r × normal = 0` (parallel
vectors), eliminating the cross-term entirely. The Delassus diagonal becomes
exactly equal for pos/neg facets: both are `μ²h²/I`. The solver produces
`f_pos = f_neg` exactly → zero net friction force → zero lateral acceleration.

**Key properties:**
- Works at the Jacobian level (the right abstraction layer)
- Preserves friction (no condim override needed)
- Applies to ALL contacts via threshold (not geometry-specific)
- For aligned contacts (sphere-on-flat, cube-on-cube): no-op (already aligned)
- For physically-offset contacts (inclined plane): threshold prevents activation
- Approximation error is negligible (proportional to drift / lever arm ≈ 1e-16)

### Result

Spheres stack stably for 15+ seconds with zero lateral drift (< 1e-9 at 9
decimal places). Friction is preserved (condim=3). Previously failed at
~1.2 seconds.

## Key Files

| File | Role |
|------|------|
| `sim/L0/core/src/constraint/jacobian.rs` | `stabilize_lever_arm()` + `compute_contact_jacobian()` — **the core fix** |
| `sim/L0/core/src/sdf/shape.rs` | `compute_shape_contact()` — analytical contact path, applies `stabilize_direction` |
| `sim/L0/core/src/sdf/operations.rs` | `stabilize_direction()` — zeroes sub-physical direction components |
| `sim/L0/core/src/types/contact_types.rs` | `compute_tangent_frame()` — tangent vectors |
| `sim/L0/core/src/constraint/contact_assembly.rs` | `assemble_pyramidal_contact()` — facet Jacobians |

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
