# Joint Mimic — 1:1 Joint Coupling

Two hinge arms side by side, coupled by a `<joint>` equality constraint with
`polycoef="0 1"` — a **mimic joint**. Whatever angle arm1 is at, arm2 must
match. They start at different angles to show the constraint pulling them
together.

See also: [Joint Gear](../joint-gear/) — the same constraint type with a 2:1
ratio.

## What you see

- **Two orange capsule arms** — hanging from separate hinges, side by side
- At startup, arm1 is displaced to 0.5 rad and arm2 to -0.3 rad — they start
  apart
- Within the first second, the constraint snaps them together — both arms
  converge to the same angle
- After convergence, they swing in perfect unison under gravity, damping to
  rest together

## Physics

The `<joint>` equality constraint enforces a polynomial relationship between
two joint positions:

```
joint1 = polycoef[0] + polycoef[1] * joint2
```

With `polycoef="0 1"`, this becomes `j1 = j2` — a 1:1 mimic. The constraint
doesn't care about joint velocities directly; it penalizes the position error
`j1 - j2` and the velocity error via Baumgarte stabilization.

The softer `solref="0.05 1.0"` is intentional — joint-space constraints are
more stable with lower stiffness because they act on scalar DOFs with simple
dynamics (no quaternion complications).

| Parameter | Value |
|-----------|-------|
| Arm mass | 0.5 kg each |
| Arm length | 0.4 m |
| Joint damping | 0.5 N·m·s/rad |
| Initial angles | j1 = 0.5 rad, j2 = -0.3 rad |
| polycoef | 0, 1 (1:1 mimic) |
| solref | 0.05, 1.0 (soft) |

## Validation

Two automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Mimic converges** | Angle error < 0.1 rad by t=1s | converged by 1s |
| **Mimic tracks** | Angle error stays small after convergence | < 0.05 rad |

## Run

```
cargo run -p example-equality-joint-mimic --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
