# Stress Test — Headless Tendon Validation

Exhaustive headless validation of every tendon system invariant. No window,
no Bevy — pure sim-core assertions against analytical expectations.

## What it validates

| # | Check | Invariant |
|---|-------|-----------|
| 1 | Fixed tendon length | `L = Σ coef_i × qpos_i` — exact to machine precision |
| 2 | Fixed tendon velocity | `V = Σ coef_i × qvel_i` — exact to machine precision |
| 3 | TendonPos sensor | Sensor readback matches `data.ten_length` exactly |
| 4 | TendonVel sensor | Sensor readback matches `data.ten_velocity` exactly |
| 5 | Spatial length (no wrap) | Length equals sum of Euclidean site-to-site distances |
| 6 | Sphere wrap | All wrap path points at distance ≥ sphere radius from center |
| 7 | Cylinder wrap | All wrap path points at radial distance ≥ cylinder radius |
| 8 | Tendon limit activates | `TendonLimitFrc > 0` when tendon exceeds range |
| 9 | No limit force interior | `TendonLimitFrc == 0` when tendon length is within range |
| 10 | Pulley divisor | `L = d(s1,s2) + d(s3,s4) / divisor` — exact |
| 11 | Tendon actuator | Motor with tendon transmission produces nonzero `qfrc_actuator` |
| 12 | Spring/damper force | `F = −k(L − L₀) − bV` — exact to machine precision |
| 13 | Fixed Jacobian constant | Jacobian identical at two different configurations |
| 14 | Spatial Jacobian varies | Jacobian differs between two configurations |
| 15 | Multi-tendon compose | Two tendons on same joint have independent correct lengths |
| 16 | Moment arm varies | Spatial tendon Jacobian entry changes with joint angle |

## Expected output

```
TOTAL: 16/16 checks passed
ALL PASS
```

## Run

```
cargo run -p example-tendon-stress-test --release
```
