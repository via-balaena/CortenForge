# Ball Joint Hybrid A Matrix — Resolved (Not a Bug)

**Date:** 2026-04-03
**Status:** RESOLVED — false alarm from error metric, not a code defect

---

## Original symptom

`mjd_transition_hybrid` and `mjd_transition_fd` produced A matrices with
~1.0 relative error for ball joint models with nonzero angular velocity
and extreme inertia ratios (diaginertia 0.1/0.1/0.001).

## Root cause

`max_relative_error` with `floor=1e-10` amplified FD noise on near-zero
entries. The third DOF (inertia=0.001) produces coupling entries of order
1e-10 in the dq/dv block. FD noise at this scale has opposite signs between
hybrid and FD, giving relative error ~1.0 on entries that are physically
negligible.

## Evidence (diagnostic test)

Block-by-block comparison for extreme inertia ball joint:

| Block | Floor=1e-10 | Floor=1e-6 |
|-------|-------------|------------|
| dq/dq (FD both) | 7.4e-4 | 7.4e-4 |
| **dq/dv (analytical hybrid)** | **9.99e-1** | **1.5e-4** |
| dv/dq (FD both) | 7.2e-10 | 7.2e-10 |
| dv/dv (analytical hybrid) | 2.0e-8 | 2.0e-8 |

The 0.999 error vanishes entirely with a reasonable floor. Entry-by-entry
inspection showed maximum absolute difference of 2.5e-10 — machine
epsilon level.

## Resolution

- Added `test_ball_joint_hybrid_vs_fd_a` to the test suite (standard +
  extreme inertia, floor=1e-6, tolerance=2e-3)
- No code changes needed in the derivatives engine
- The velocity Jacobian `h*I` in `mjd_quat_integrate` is correct for
  the FD tangent convention at typical timesteps
