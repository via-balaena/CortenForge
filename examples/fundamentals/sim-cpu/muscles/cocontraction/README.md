# Cocontraction — Agonist-Antagonist Muscle Pair

Two opposing muscles on the same elbow joint demonstrate the fundamental
principle of biological motor control: muscles can only pull, never push.
To control both position and stiffness, the nervous system activates
opposing muscles simultaneously — cocontraction.

## What you see

- **One forearm** with **two tendons** (one on each side of the arm)
  - **Agonist** (front, gear=+1) — pulls the elbow toward flexion
  - **Antagonist** (back, gear=-1) — pulls the elbow toward extension
- The demo cycles through four phases:

| Phase | Time | Agonist | Antagonist | What happens |
|-------|------|---------|------------|-------------|
| REST | 0–1s | off | off | Arm hangs vertical, both tendons blue |
| COCONTRACT | 1–4s | on | on | Both tendons red, joint stiffens at center |
| AGONIST | 4–7s | on | off | Front tendon red, arm swings ~60° one way |
| ANTAGONIST | 7–10s | off | on | Back tendon red, arm swings ~60° the other way |
| REST | 10s+ | off | off | Gravity returns arm to vertical, tendons blue |

## Physics

During cocontraction, both muscles produce substantial force (both tendons
turn red), but the net torque is near zero because they oppose each other.
The joint doesn't move, but it becomes **stiff** — resistant to perturbation.

When only one muscle activates, it produces torque unopposed (except by
gravity and damping). The arm swings to a partial angle where muscle torque
balances the gravity load. Because scale=50 gives F0 proportional to the
arm's inertia, the motion is smooth and controlled — no limit slamming.

Gravity provides the natural restoring force. When both muscles release,
the arm returns to vertical on its own. This is how real arms work — gravity
is part of the control system, not an obstacle.

```
net_torque = agonist_force x gear_ago + antagonist_force x gear_ant + gravity
           = F x (+1) + F x (-1) + mg*d*sin(theta)
           = mg*d*sin(theta)  (cocontraction: muscle forces cancel)
```

| Parameter | Value |
|-----------|-------|
| F0 (each muscle) | ~1.9 N (auto: scale/acc0) |
| Scale | 50 |
| Gear (agonist / antagonist) | +1 / -1 |
| Joint damping | 0.5 N-m-s/rad |
| Gravity torque (peak) | 1.47 N-m |
| Body mass | 1.0 kg |
| Forearm length | 0.3 m (CoM at 0.15 m) |

## Validation

Four automated checks at t=12s:

| Check | Expected |
|-------|----------|
| **Both active during cocontraction** | max \|force\| > 1 N for each muscle |
| **Joint stable during cocontraction** | angle range < 0.3 rad during t=2-3s |
| **Agonist-only moves joint** | angle > 0.1 rad at end of agonist phase |
| **Antagonist moves opposite** | angle difference > 0.1 between phases |

## Run

```
cargo run -p example-muscle-cocontraction --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
