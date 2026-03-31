# Prismatic Joint Slider

Mass on a horizontal spring using a prismatic (slide) joint from URDF. A
spring force is applied via `qfrc_applied` since URDF has no stiffness
attribute. Verifies the oscillation period matches T = 2*pi*sqrt(m/k).

## What it tests

The URDF `prismatic` joint converts to an MJCF `slide` with the correct
axis mapping and position limits. The axis attribute `<axis xyz="1 0 0"/>`
maps directly to the MJCF joint axis.

## Physics

The slider has mass 2.0 kg on a spring with k = 50 N/m. The horizontal
slide axis means gravity does not influence the oscillation.

```
T = 2*pi*sqrt(m/k) = 2*pi*sqrt(2/50) = 1.257s
```

The measured period matches within 0.05%.

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | URDF loads with 1 joint | exact |
| 2 | Joint type is slide | exact |
| 3 | Slide limits [-1, 1] | 0.001 |
| 4 | Axis maps to X | 0.01 |
| 5 | Period matches T = 2*pi*sqrt(m/k) | 2% |

## Run

```
cargo run -p example-urdf-prismatic --release
```
