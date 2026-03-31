# Revolute Joint Pendulum

Single hinge pendulum defined in URDF. The arm hangs from an invisible
pivot and swings under gravity with a rod (cylinder) and tip mass (sphere).

## What you see

A pendulum swinging back and forth. The HUD shows angle, velocity, energy,
and energy drift. The ValidationHarness tracks period (against the
analytical compound-pendulum formula) and energy conservation.

## What it tests

The URDF `revolute` joint converts to an MJCF `hinge` with `limited="true"`
and the range taken from `<limit lower upper>`. This is the most common
joint type in URDF robot descriptions.

## Physics

The arm has mass 1.0 kg with COM at 1.0m below the pivot:

```
I_pivot = m*L^2 + I_cm = 1.0 + 0.01 = 1.01
T = 2*pi*sqrt(I / (m*g*L)) ≈ 2.05s
```

## Validation

| Check | Source |
|-------|--------|
| Period matches analytical | `track_period` (2% tolerance) |
| Energy conserved | `track_energy` (0.5% tolerance) |
| Revolute → hinge | `print_report` structural check |
| Joint is limited | `print_report` structural check |
| MJCF contains hinge | `print_report` structural check |

## Run

```
cargo run -p example-urdf-revolute --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
