# Force & Torque Sensors

Reaction force and moment at the base of a static cantilevered beam.

## What it demonstrates

- `<force>` measures 3D internal reaction force at a site (in site frame)
- `<torque>` measures 3D internal reaction moment at a site (in site frame)
- Both are acceleration-stage sensors (read from `cfrc_int`)
- For a static body: Force = reaction to gravity, Torque = reaction moment

## Expected visual behavior

A horizontal beam with a green tip held perfectly still by a strong motor.
Nothing moves — the beam stays level. The HUD shows steady force and torque
readings that match the analytical predictions exactly.

## Expected console output

```
t=  2.0s  |F|=9.8100  |tau|=3.9240
t=  3.0s  |F|=9.8100  |tau|=3.9240
...
```

Rock-steady readings matching mg and mgL/2.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| \|Force\| = mg | mean force magnitude after settling | < 1% |
| Force XY ≈ 0 | no lateral reaction forces | < 0.05 N |
| \|Torque\| = mgL/2 | mean torque magnitude after settling | < 1% |
| Torque XZ ≈ 0 | moment about hinge axis only | < 0.05 N·m |

## Run

```bash
cargo run -p example-sensor-adv-force-torque --release
```
