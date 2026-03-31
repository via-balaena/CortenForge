# Activation Dynamics — Rise/Fall Asymmetry

Three identical forearms driven by muscles with different activation time
constants. All receive the same control signal simultaneously. The arms
contract together — but when the signal drops, the slow muscle holds
tension far longer than the fast one. This asymmetry between activation
rise and deactivation fall is the signature of Millard et al. (2013)
muscle dynamics.

## What you see

- **Three forearms** side by side, each with a tendon (blue at rest, red
  under tension)
  - **Left (green tip)** — Fast-twitch: tau_act=5ms, tau_deact=20ms
  - **Center (amber tip)** — Medium: tau_act=10ms, tau_deact=40ms
  - **Right (red tip)** — Slow-twitch: tau_act=50ms, tau_deact=200ms
- At t=0.5s, all three receive ctrl=1 — the arms curl up together
  (rise is fast for all three, so contraction looks simultaneous)
- At t=4.0s, ctrl drops to 0 — **watch the release**
  - The fast arm drops almost immediately
  - The medium arm follows shortly after
  - The slow arm visibly lingers at the flexed position before releasing
- Tendons shift from blue to red (contraction) and back to blue (release),
  with the slow tendon staying red longest

## Why the contraction looks the same

All three muscles reach full activation within ~200ms of ctrl onset. Even
the "slow" muscle (tau_act=50ms) reaches 90% in ~170ms — fast enough that
at the visual timescale (seconds), the arms appear to curl together. The
activation time constants span only one order of magnitude on the rise
side (5ms to 50ms).

The interesting physics is on the **release** side. Deactivation time
constants span the same order of magnitude (20ms to 200ms), but the
effective deactivation is further slowed by the activation-dependent
scaling:

```
tau_deact_eff = tau_deact / (0.5 + 1.5 * act)
```

At high activation (act near 1.0), the denominator is ~2.0, so deactivation
starts relatively fast. But as activation drops, the denominator approaches
0.5, making the tail of deactivation 4x slower than the start. This
creates a long tail that's much more visible than the fast rise.

## Physics

| Muscle | tau_act | tau_deact | Rise to 90% | Time at act=0.1 after release |
|--------|---------|-----------|-------------|-------------------------------|
| Fast   | 5 ms    | 20 ms     | ~19 ms      | ~60 ms                        |
| Medium | 10 ms   | 40 ms     | ~35 ms      | ~120 ms                       |
| Slow   | 50 ms   | 200 ms    | ~168 ms     | ~600 ms                       |

All other parameters are identical (scale=200, damping=0.3, same geometry).
The only difference is the activation filter bandwidth.

## Expected behavior

| Phase | Time | What happens |
|-------|------|-------------|
| Rest | 0–0.5s | All arms hang vertical, tendons blue |
| Contraction | 0.5–4.0s | Arms curl together (rise is fast for all) |
| Release | 4.0s+ | Arms release at different rates — fast first, slow last |
| Final rest | ~5–6s | All arms hanging, tendons blue |

## Validation

Four automated checks at t=8s:

| Check | Expected |
|-------|----------|
| **Fast rises before medium** | fast rise time < medium rise time |
| **Medium rises before slow** | medium rise time < slow rise time |
| **Slow > 5x fast rise time** | ratio of rise times > 5 |
| **All three reached 90%** | all muscles hit act > 0.9 |

## Run

```
cargo run -p example-muscle-activation --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
