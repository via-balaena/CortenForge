# Ball Cone — Ball Joint Cone Limit

A rod hanging from a ball joint with a 30-degree cone limit. The rod starts
tilted 40 degrees beyond the cone boundary and is pushed back by the
constraint. The cone is rotationally symmetric — the same limit force applies
regardless of azimuthal direction.

## Concept

Ball joints have 3 rotational degrees of freedom. When limited, the constraint
restricts the total deflection angle (measured from the identity orientation)
to a cone. `range="0 30"` means the rod can deflect up to 30 degrees from
vertical in any direction.

## What you see

- A rod hanging from a pivot, swinging within a 30-degree cone
- Starts at ~40 degrees tilt (beyond the cone), gets pushed back
- The rod oscillates and eventually settles near the cone boundary
- HUD shows deflection angle and limit force in real time

## What to look for

- The rod never swings more than ~33 degrees from vertical (limit + solver tolerance)
- Limit force activates when deflection exceeds 30 degrees
- The swing pattern is symmetric — the cone doesn't favor any direction

## Run

```
cargo run -p example-joint-limits-ball-cone --release
```
