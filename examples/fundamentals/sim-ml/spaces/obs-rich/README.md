# Rich Observation Space

Cart-pole model with a single large observation built from every extractor
type. The HUD breaks down the observation vector by source, showing which
slice of the tensor comes from which extractor.

## What you see

- A cart on a rail with a pole balanced on top (falls quickly)
- HUD showing the full observation vector broken into labeled segments:
  qpos, qvel, sensors (by name), xpos, energy, time, contact_count

## Expected behavior

- Pole topples within a few seconds (no controller)
- HUD updates every frame with live tensor values
- Each segment's element count matches the extractor's contribution
- Total dim equals the sum of all segments

## Validation

| Check | Expected | Tolerance |
|-------|----------|-----------|
| obs_dim | sum of all extractor dims | exact |
| Named sensors | match manual sensordata indexing | f32 epsilon |
| xpos | 3 floats per body | exact count |
| energy | 2 floats (kinetic, potential) | exact count |

## Run

```
cargo run -p example-ml-spaces-obs-rich --release
```
