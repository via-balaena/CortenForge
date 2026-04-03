# Stress Test — Headless Validation

Headless (no Bevy) validation of the full derivatives module. Tests dimensions,
eigenvalue analysis, FD convergence rates, hybrid-vs-FD agreement, sensor
derivatives, inverse dynamics, integrator coverage, quaternion handling,
actuator/activation dynamics, contact sensitivity, and config edge cases.
Exits with code 1 if any check fails.

## Checks

| # | Group | Check | Tolerance |
|---|-------|-------|-----------|
| 1 | Dimensions | A is (2*nv+na) x (2*nv+na) with activation | exact |
| 2 | Dimensions | B is (2*nv+na) x nu with motor | exact |
| 3 | Dimensions | Free joint: A is 12x12 (nv=6) | exact |
| 4 | Dimensions | Ball joint: A is 6x6 (nv=3) | exact |
| 5 | Eigenvalues | Upright pendulum has \|lambda\| > 1 (unstable) | > 1.0 |
| 6 | Eigenvalues | Downward pendulum has \|lambda\| <= 1 (stable) | <= 1+1e-6 |
| 7 | Eigenvalues | 1-DOF upright eigenvalues are real | imag < 1e-6 |
| 8 | FD convergence | Centered: halving eps quarters error (O(eps^2)) | ratio < 0.4 |
| 9 | FD convergence | Forward: halving eps halves error (O(eps)) | ratio < 0.7 |
| 10 | FD convergence | `fd_convergence_check()` returns true | exact |
| 11 | Hybrid vs FD | A agreement for 3-link pendulum | < 1e-4 |
| 12 | Hybrid vs FD | `validate_analytical_vs_fd()` A+B for actuated | < 1e-3 |
| 13 | Hybrid vs FD | `validate_analytical_vs_fd()` for unactuated | < 1e-3 |
| 14 | Sensors | C is Some when requested | exact |
| 15 | Sensors | D is Some when requested | exact |
| 16 | Sensors | C, D are None by default | exact |
| 17 | Inverse dynamics | DfDq, DfDv, DfDa dimensions nv x nv | exact |
| 18 | Inverse dynamics | DfDa ~ mass matrix M | < 1e-5 |
| 19 | Inverse dynamics | DfDv has damping on diagonal | qualitative |
| 20 | Integrators | Euler computes without error | no panic |
| 21 | Integrators | ImplicitSpringDamper computes without error | no panic |
| 22 | Integrators | ImplicitFast computes without error | no panic |
| 23 | Integrators | RK4 computes without error (pure FD fallback) | no panic |
| 24 | Quaternions | Ball joint A is 6x6 (tangent, not 8x8) | exact |
| 25 | Quaternions | Free joint A is 12x12 (tangent, not 14x14) | exact |
| 26 | Quaternions | Ball joint hybrid vs FD A agreement | < 2e-3 |
| 27 | Actuators | B column count equals nu | exact |
| 28 | Actuators | Activation filter entries in A diagonal | qualitative |
| 29 | Actuators | Zero-control matches passive dynamics | < 1e-10 |
| 30 | Contact | A changes with contact stiffness | qualitative |
| 31 | Contact | A differs between contact/no-contact | qualitative |
| 32 | Config | eps=1e-8 produces finite non-NaN matrices | no NaN/Inf |

## Run

```
cargo run -p example-derivatives-stress-test --release
```

Exit code 0 = all pass, exit code 1 = failure.
