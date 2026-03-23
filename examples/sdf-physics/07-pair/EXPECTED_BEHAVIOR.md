# SDF Physics 07 — Pair (Expected Behavior)

Two identical 5mm-radius SDF spheres stacked vertically. The lower sphere
rests on the ground plane; the upper sphere rests on the lower sphere.

## What you should see

- Both spheres settle into a stable stacked configuration
- The stack holds indefinitely (15+ seconds verified, no lateral drift)
- Two contacts active throughout: ground-to-lower and lower-to-upper

## Why the floor shows through the bottom sphere

The ground contact uses MuJoCo-style **soft constraints** (`solref` impedance
model). The sphere sinks ~0.15mm into the ground to generate enough contact
force to support both spheres. This equilibrium penetration is physical — it
represents the compliance of the contact, analogous to how a real object
deforms slightly under load.

The penetration depth depends on `solref[0]` (time constant). Smaller values
produce stiffer contacts with less visible penetration. The current value
(0.0125s) is tuned for stability, not visual crispness.

## Why the upper sphere doesn't fall

Ball-on-ball is an inherently **unstable equilibrium** — in real physics, the
slightest perturbation causes the upper ball to slide off. In simulation, IEEE
754 floating-point rounding provides that perturbation at the 1 ULP level
(~10^-15), which exponentially amplifies through the contact geometry.

Two mechanisms prevent the instability:

1. **Contact normal stabilization.** The separation direction between the two
   sphere centers is stabilized by zeroing sub-physical components (< 10^-6
   relative). This prevents the contact force from acquiring a lateral
   component due to floating-point position drift.

2. **Frictionless analytical contact.** For convex-on-convex contacts (both
   shapes provide an analytical `effective_radius`), the contact dimension is
   set to 1 (frictionless). Pyramidal friction facets on such contacts produce
   force asymmetry proportional to the lateral offset through the angular
   Jacobian's lever-arm dependence — even with a perfectly stabilized normal.
   Eliminating friction removes this channel entirely.

With both fixes, the net lateral force on the upper sphere is exactly zero:
the normal Jacobian for the lateral DOF is `normal.y = 0` (exact), and no
friction rows exist to produce asymmetric forces.

## Key concept

This example tests **SDF-vs-SDF collision** with the analytical contact path.
It depends on 04-rest (SDF-plane contact, proven earlier) and exercises the
full chain: broadphase AABB overlap, `compute_shape_contact()` analytical
path, constraint assembly, and solver.
