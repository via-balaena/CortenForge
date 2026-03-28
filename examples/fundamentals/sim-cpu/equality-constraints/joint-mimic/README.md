# Joint Mimic — 1:1 Joint Coupling

Two hinge arms side by side, coupled by a `<joint>` equality constraint with
`polycoef="0 1"` — a **mimic joint**. Whatever angle arm1 is at, arm2 must
match. They start at different angles to show the constraint pulling them
together, then swing in unison.

See also: [Joint Gear](../joint-gear/) — the same constraint type with a 2:1
ratio.

## Where mimic joints are used

Mimic joints show up whenever you have **N joints but fewer than N actuators**:

- **Parallel grippers** — one motor drives one finger, the other mirrors it
  via a mimic constraint. Command one joint, both close symmetrically.
- **Humanoid hands** — finger joints coupled so proximal and distal knuckles
  flex together from a single actuator.
- **Linkage mechanisms** — four-bar linkages where one joint's angle determines
  another's.
- **Symmetric robots** — left/right legs or arms that should track each other
  during calibration or testing.

The key idea: the constraint reduces effective DOFs so one motor can drive
multiple joints in a coordinated way.

## What you see

- **Two orange capsule arms** — hanging from separate hinges, side by side
- At startup, arm1 is displaced to 1.2 rad (~70°) and arm2 to -1.0 rad (~57°
  opposite) — they start far apart, swinging out of phase
- Over the first few seconds the constraint gradually pulls them into sync —
  the arms go from opposing motion to matching motion
- After convergence, they swing together in unison indefinitely (no damping)

## Physics

The `<joint>` equality constraint enforces a polynomial relationship between
two joint positions:

```
joint1 = polycoef[0] + polycoef[1] * joint2
```

With `polycoef="0 1"`, this becomes `j1 = j2` — a 1:1 mimic. The constraint
penalizes the position error `j1 - j2` and the velocity error via Baumgarte
stabilization.

The soft `solref="0.8 0.5"` is intentional — it makes the synchronization
gradual enough to watch. A stiffer constraint would snap them together
instantly, which is useful in practice but not instructive.

| Parameter | Value |
|-----------|-------|
| Arm mass | 1.0 kg each |
| Arm length | 0.7 m |
| Joint damping | 0 (undamped) |
| Initial angles | j1 = 1.2 rad, j2 = -1.0 rad |
| polycoef | 0, 1 (1:1 mimic) |
| solref | 0.8, 0.5 (soft — visible convergence) |

## Validation

Two automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Mimic converges** | Angle error < 0.1 rad by t=3s | converged by 3s |
| **Mimic tracks** | Angle error stays small after convergence | < 0.1 rad |

## Run

```
cargo run -p example-equality-joint-mimic --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
