# SDF Physics 07 — Pair (Expected Behavior)

Two identical 5mm-radius SDF spheres, one placed above the other. The lower
sphere rests on the ground plane; the upper sphere settles onto the lower one,
balances there briefly, then rolls off.

## What you should see

- The upper sphere settles onto the lower one (two contacts: ground-to-lower
  and lower-to-upper)
- Within about two seconds it rolls off and lands on the ground beside it
  (headless run of this model: still stacked at 1 s, both spheres on the
  ground at 2 s)
- The two spheres then roll apart along the ground, each with its own ground
  contact; nothing stops them, since the model has no rolling friction

## Why the floor shows through the bottom sphere

The ground contact uses MuJoCo-style **soft constraints** (`solref` impedance
model). The sphere sinks into the ground to generate enough contact force to
support its weight. This equilibrium penetration is physical — it represents
the compliance of the contact, analogous to how a real object deforms slightly
under load.

The penetration depth depends on `solref[0]` (time constant). Smaller values
produce stiffer contacts with less visible penetration. The current value
(0.0125s) is tuned for stability, not visual crispness.

## Why the upper sphere falls

Ball-on-ball is an inherently **unstable equilibrium** — in real physics, the
slightest perturbation causes the upper ball to slide off. In simulation, IEEE
754 floating-point rounding provides that perturbation at the 1 ULP level
(~10^-15), which grows exponentially through the contact geometry.

The contact rows are MuJoCo's (`mj_contactJacobian`), with no step that
projects a contact's lever arm onto its normal; an earlier version had one, and
held the stack. The SDF collision still zeroes sub-physical
lateral components of the separation direction (`sdf/operations.rs`); that
alone does not hold the stack.

## Key concept

This example tests **SDF-vs-SDF collision** with the analytical contact path.
It depends on 04-rest (SDF-plane contact, proven earlier) and exercises the
full chain: broadphase AABB overlap, `compute_shape_contact()` analytical
path, constraint assembly, and solver.

## Pass criteria (automated at t ≥ 5s)

- Both spheres on the ground: z within 1mm of the radius
- The spheres apart: their centres at least two radii apart horizontally
- Both z-velocities < 1 mm/s
- Contacts active (ncon > 0)
