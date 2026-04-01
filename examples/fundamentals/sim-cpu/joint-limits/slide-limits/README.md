# Slide Limits — Motor Force vs Limit Constraint

A mass on a horizontal rail with `range="-0.3 0.3"` pushed by a constant
motor force into the positive limit. Demonstrates that the limit constraint
holds against actuator force and that the `JointLimitFrc` sensor reports the
reaction force.

## Concept

Slide joints translate along an axis. When limited, the solver generates a
one-sided constraint at the range boundary. A motor actuator applies a constant
force — at equilibrium, the limit constraint force exactly balances the motor.

## What you see

- A box sliding on a horizontal rail (capsule visual)
- Motor pushes the box toward the positive limit (+0.3m)
- The box reaches the limit and stays clamped
- HUD shows position, motor force, and limit reaction force in real time

## What to look for

- Position settles at ~0.3m (the upper limit)
- Limit force approximately equals the motor force at equilibrium
- Position never exceeds the limit by more than solver tolerance

## Run

```
cargo run -p example-joint-limits-slide-limits --release
```
