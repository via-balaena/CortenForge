# Observation Extraction

Pendulum swinging freely from 45 degrees. The HUD shows extracted
tensor values (f32) alongside raw Data fields (f64) so you can see
the bridge's truncation in real time.

## What you see

- A pendulum swinging under gravity (no motor torque)
- HUD showing two columns: the raw f64 value from `data.qpos[0]`
  / `data.qvel[0]` and the f32 value from `obs_space.extract()`
- The error column shows the f64-to-f32 truncation magnitude

## Expected behavior

- Pendulum oscillates back and forth, amplitude decaying slightly
  from joint damping
- Tensor values track the raw data values closely (error < 1e-6)
- obs shape stays [2] every frame
- Two consecutive extract() calls on the same Data produce
  identical tensors (idempotency confirmed at report time)

## Validation

| Check | Expected | Tolerance |
|-------|----------|-----------|
| obs shape | [2] | exact |
| obs[0] vs qpos[0] | match as f32 | f32 epsilon |
| obs[1] vs qvel[0] | match as f32 | f32 epsilon |
| Idempotent extract | identical tensors | exact |

## Run

```
cargo run -p example-ml-spaces-obs-extract --release
```
