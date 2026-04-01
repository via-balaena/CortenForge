# Stress Test — Headless Joint Limits Validation

Exhaustive headless validation of joint limit constraints and solver tuning.
No window, no Bevy — pure sim-core assertions across purpose-built MJCF models.

## Concept

**Engine-level correctness.** Joint limits constrain range of motion via
one-sided constraint rows. When a joint reaches its limit, the solver generates
a constraint force whose stiffness and damping are controlled by `solref` and
`solimp`. Getting this wrong means joints blow past their range (false negative)
or generate spurious forces when interior (false positive). This stress test
exercises every limit path: hinge, slide, ball cone, sensor readback, solver
parameter tuning, and motor interaction.

## What it tests

| # | Check | Mechanism |
|---|-------|-----------|
| 1 | Hinge released beyond limit stays within range + tolerance | Hinge limit constraint |
| 2 | Slide released beyond limit stays within range + tolerance | Slide limit constraint |
| 3 | Ball joint beyond cone limit stays within cone + tolerance | Ball cone constraint |
| 4 | JointLimitFrc > 0 when at limit | Sensor readback at limit |
| 5 | JointLimitFrc == 0 when interior | Sensor readback inside range |
| 6 | Only violated side generates force (one-sided) | Upper vs lower independence |
| 7 | Stiff solref produces higher force than soft solref | Solref stiffness scaling |
| 8 | Wide solimp allows more penetration than narrow | Solimp width control |
| 9 | Motor cannot push past limit | Limit vs actuator force |
| 10 | Zero-width range holds position at zero | Locked joint (range="0 0") |
| 11 | Higher initial overshoot produces higher peak force | Force vs penetration depth |
| 12 | Ball cone limit force is azimuthally symmetric | Cone rotational symmetry |

## Expected output

```
TOTAL: 12/12 checks passed
ALL PASS
```

## Run

```
cargo run -p example-joint-limits-stress-test --release
```
