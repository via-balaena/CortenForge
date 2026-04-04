# batch-sim — Parallel Multi-Environment Simulation

Three examples exercising the `BatchSim` API: parallel stepping, selective
reset, and exhaustive invariant testing.

## Examples

| Example | Type | What it demonstrates |
|---------|------|---------------------|
| [`parameter-sweep/`](parameter-sweep/) | Visual | 8 pendulums with different damping, all stepping in parallel via `step_all()` |
| [`reset-subset/`](reset-subset/) | Visual | 12 landers converging on the right thrust — `reset_where(mask)` resets failures while successes stay landed |
| [`stress-test/`](stress-test/) | Headless | 10 checks: construction, independence, determinism, all reset variants, shared model, single-env parity |

## API surface covered

| Method | Example |
|--------|---------|
| `BatchSim::new(model, n)` | parameter-sweep |
| `step_all()` | parameter-sweep, reset-subset |
| `envs()` / `envs_mut()` | parameter-sweep, reset-subset |
| `env(i)` / `env_mut(i)` | reset-subset |
| `reset_where(mask)` | reset-subset |
| `reset(i)` | stress-test |
| `reset_all()` | stress-test |
| `len()` / `is_empty()` | stress-test |
| `model()` | stress-test |

## Shared infrastructure

Both visual examples use building blocks from `sim-bevy`:

- `sync_batch_geoms(&batch, &mut scenes)` — copies geom poses from `BatchSim` into `PhysicsScenes`
- `insert_batch_validation_dummies(commands, model)` — creates dummy resources for `ValidationHarness`
- `spawn_scene_geoms` / `spawn_scene_geoms_with` — spawns per-env visual geometry
- `physics_pos(x, y, z)` — Z-up to Y-up coordinate conversion
