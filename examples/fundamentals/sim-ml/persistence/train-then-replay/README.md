# Train-then-Replay — Policy Persistence

Trains CEM on the 2-DOF reaching arm headlessly (~20 epochs), saves the
**best-epoch** `PolicyArtifact` to disk as JSON, reconstructs a live
policy from the artifact, then visualizes a single arm executing the
best policy in Bevy. This is the proof that the persistence system works:
train once, save the best, reconstruct, replay — all in one binary.

See also: [Auto-Reset (CEM)](../../vec-env/auto-reset/) — the same CEM
training shown live with 50 arms.

## What you see

- **Console output first** — headless CEM training prints epoch progress,
  then the artifact file path and metadata (algorithm, epochs, reward,
  wall time, timestamp)
- **Single arm in Bevy** — after training completes, a Bevy window opens
  showing one two-link arm executing the learned policy. The arm should
  reach toward the green target sphere
- **HUD** (bottom-left) — shows training provenance (algorithm, epochs,
  final reward), artifact path, replay episode count, and average replay
  reward

## How it works

### Training phase (headless)

CEM trains on `reaching_2dof()` with 50 parallel environments for 20
epochs. Same algorithm, same hyperparameters as the auto-reset example.
No Bevy window — pure computation.

### Artifact creation

After training, the **best-epoch policy** is extracted via
`cem.best_artifact()` as a `PolicyArtifact` with full provenance metadata:

| Field | Content |
|-------|---------|
| version | 1 |
| descriptor | Linear, obs_dim=4, act_dim=2 |
| params | Trained weights as f64 |
| algorithm | "CEM" |
| task | "reaching-2dof" |
| epochs_trained | 20 |
| final_reward | ~-1.0 (varies by run) |
| wall_time_ms | Training duration |
| timestamp | ISO 8601 |

The artifact is saved to disk as JSON
(`trained_reaching_2dof.artifact.json`).

### Reconstruction

`PolicyArtifact::load(path)` reads the JSON, validates the version and
param count, and `to_policy()` reconstructs a live `Box<dyn Policy>`
from the descriptor and params. The reconstructed policy is identical to
the trained one — forward pass produces the same actions.

### Replay phase (visual)

A single arm executes the reconstructed best-epoch policy in Bevy. Pure
inference — no training, no noise. The arm follows the deterministic mean
action `tanh(W * obs_scaled + b)` using the best epoch's saved weights.

## Validation

| Check | Expected |
|-------|----------|
| Training converged | final_reward > -1.0 |
| Artifact saved to disk | file exists at printed path |
| Policy reconstructed | `to_policy()` succeeded |
| Replay policy active | average replay reward > -500 |

## Run

```
cargo run -p example-ml-persistence-train-then-replay --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
