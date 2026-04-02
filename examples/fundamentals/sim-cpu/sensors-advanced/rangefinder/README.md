# Rangefinder Sensor

Ray-based distance measurement from an oscillating body to the ground.

## What it demonstrates

- `<rangefinder>` casts a ray along the site's +Z axis
- Returns distance to the nearest geometry (excludes own body's geoms)
- Returns −1 when the ray hits nothing within range
- Position-stage sensor (runs during forward kinematics)

## Expected visual behavior

A blue sphere bobs up and down above a dark ground surface. The sphere
oscillates smoothly between 0.5 m and 1.5 m height (4-second period).
The HUD tracks the rangefinder reading vs expected height in real time.

## Expected console output

```
t=  1.0s  down=0.8385  side=-1
t=  2.0s  down=1.4120  side=-1
t=  3.0s  down=0.8385  side=-1
t=  4.0s  down=0.4200  side=-1
```

The down reading oscillates sinusoidally. The side reading is always −1.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Down = height | max error over 15s | < 0.01 m |
| Side = −1 | every single frame | 0 violations |
| Range oscillates | min ≈ 0.42, max ≈ 1.42 | < 0.1 m |

## Run

```bash
cargo run -p example-sensor-adv-rangefinder --release
```
