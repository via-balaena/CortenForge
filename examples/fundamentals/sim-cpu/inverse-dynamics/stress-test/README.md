# Stress Test — Headless Validation

Headless (no Bevy) validation of all inverse dynamics and Jacobian
concepts. Runs automatically, prints PASS/FAIL for each check, exits
with code 1 if any check fails.

## Checks

| # | Section | Check | Tolerance |
|---|---------|-------|-----------|
| 1 | Formula | `qfrc_inverse == M*qacc + bias - passive - constraint` | 1e-10 |
| 2 | Round-trip | `qfrc_inverse == qfrc_applied + qfrc_actuator` | 1e-8 |
| 3 | Free-fall | `qfrc_inverse ~ 0` with no applied forces | 1e-8 |
| 4 | Gravity comp | `inv == bias` in static case | 1e-8 |
| 5 | Gravity comp | `|shoulder| > |elbow|` torque | qualitative |
| 6 | Gravity comp | Arm drift < 0.001 rad over 5s | 0.001 rad |
| 7 | Tracking | Inverse->forward replay < 0.02 rad | 0.02 rad |
| 8 | Jacobian | Dimensions are 3 x nv | exact |
| 9 | Jacobian | Zero-config analytical form | 1e-6 |
| 10 | Jacobian | Velocity match (J*qdot vs FD) < 1% | 1% relative |
| 11 | Torque | `|shoulder| > |elbow|` peak | qualitative |
| 12 | Torque | Smoothness (spike < 3x RMS) | ratio < 3.0 |
| 13 | Accumulators | `cacc` non-zero under gravity | > 1.0 |
| 14 | Accumulators | `cfrc_ext` reflects applied force | > 1.0 |

## Run

```
cargo run -p example-inverse-stress-test --release
```

Exit code 0 = all pass, exit code 1 = failure.
