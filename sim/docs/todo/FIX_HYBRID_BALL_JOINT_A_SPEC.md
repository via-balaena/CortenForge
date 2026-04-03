# Fix: Hybrid A Matrix for Ball Joints

**Date:** 2026-04-03
**Prereq:** Recon in `HYBRID_DERIVATIVE_BUGS_RECON.md`
**Status:** Needs deeper investigation before spec can be written

---

## Problem

`mjd_transition_hybrid` produces A matrices with ~1.0 relative error vs
`mjd_transition_fd` for ball joint models with nonzero angular velocity.
The error concentrates in the position-velocity coupling block
(A[0..nv, nv..2nv]) and propagates to velocity-position (A[nv..2nv, 0..nv]).

## Known contributing factor (minor)

`mjd_quat_integrate` (integration.rs:67) returns `h*I` for the velocity
Jacobian. The correct formula is `h * J_r^{-1}(h*ω)`. However, at
typical timesteps (h=0.002) and moderate ω, the correction is O(h²·ω²)
≈ 1e-6 — too small to explain the 1.0 error.

## Suspected dominant cause

`mjd_smooth_vel` → `mjd_rne_vel` may not correctly compute the
Coriolis/gyroscopic derivative `∂(ω × I·ω)/∂ω` for ball joints (3-DOF
rotational). The hybrid uses `dvdv = I + h * M^{-1} * qDeriv`, so wrong
`qDeriv` entries cascade into velocity columns of A.

## Investigation plan

1. Print the 3×3 ball-joint block of `qDeriv` after `mjd_smooth_vel`
2. Compare against FD of `∂(qfrc_smooth)/∂qvel` for the same ball joint
3. If they disagree, trace into `mjd_rne_vel` to find the faulty term
4. Fix the analytical computation
5. Then fix `mjd_quat_integrate` velocity Jacobian (minor, but correct)

## Blocked on

Completion of Bug 1 fix (moment dispatch) — so we can use the improved
test infrastructure and `validate_analytical_vs_fd` returning clean
results for hinge/actuator cases before isolating ball joint issues.
