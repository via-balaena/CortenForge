# Revolute Joint Pendulum

Single hinge pendulum defined in URDF. The arm hangs from the pivot under
gravity and oscillates. Validates the most common URDF joint type
(`revolute` → MJCF `hinge`) and verifies the oscillation period matches
the analytical prediction for a compound pendulum.

## What it tests

The URDF `revolute` joint converts to an MJCF `hinge` with `limited="true"`
and the range taken from `<limit lower upper>`. This is the most common
joint type in URDF robot descriptions.

## Physics

The arm has mass 1.0 kg with COM at 0.25m below the pivot. For a compound
pendulum at small angles:

```
I = m*L^2 + I_cm = 1.0*0.0625 + 0.02 = 0.0825
T = 2*pi*sqrt(I / (m*g*L)) = 2*pi*sqrt(0.0825 / 2.4525) = 1.152s
```

The measured period matches within 0.03%.

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | URDF loads with 1 joint | exact |
| 2 | Joint type is hinge | exact |
| 3 | Joint limited with range (-3.14, 3.14) | 0.01 |
| 4 | Period matches analytical T = 1.152s | 2% |
| 5 | Energy approximately conserved (no damping) | 1% drift |

## Run

```
cargo run -p example-urdf-revolute --release
```
