# Subtree Angular Momentum Sensor

Angular momentum conservation of a spinning body in zero gravity.

## What it demonstrates

- `<subtreeangmom>` gives the angular momentum of the subtree about its COM
- For a single rigid body: L = I_com * omega (exactly conserved)
- Zero gravity, zero damping, no external torques — |L| is invariant
- Velocity-stage sensor (computed from `cvel` and body inertias)

## Expected visual behavior

A rod with blue and red tip markers spins steadily about a horizontal axis
in zero gravity. The rotation is smooth and constant — no wobble, no
slowdown. The HUD shows |L| rock-steady to machine precision.

## Expected console output

```
t=  1.0s  |L|=0.12285714  L_y=0.12285714
t=  2.0s  |L|=0.12285714  L_y=0.12285714
...
```

|L| is constant every frame. L_y carries all the angular momentum (hinge
axis is Y).

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| \|L\| conserved | max drift over 15s | < 1e-8 relative |
| \|L\| nonzero | sensor reads a real value | > 1e-6 |
| L along Y axis | off-axis components negligible | < 1e-6 ratio |

## Run

```bash
cargo run -p example-sensor-adv-subtree-angmom --release
```
