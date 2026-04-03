# Wake-on-Contact — Contact-Triggered Reactivation

Contact between an awake body and a sleeping body wakes the sleeper. This is
the primary wake trigger in the engine — collisions propagate activity through
the scene without any explicit user intervention.

## What you see

- A **steel blue box** sits on the ground plane, motionless — it started asleep
  via `sleep="init"` and is frozen in place (no gravity drift)
- An **orange ball** free-falls from 2 m above
- On impact (~0.6 s), the box instantly turns **orange** — the contact woke it
- Both bodies bounce, settle, and eventually turn **blue** as they re-sleep

## Physics

The wake-on-contact mechanism runs in `mj_wake_collision()` after narrowphase
collision detection:

```
for each contact:
    body1 = geom_body[contact.geom1]
    body2 = geom_body[contact.geom2]
    if one is Awake and the other is Asleep:
        wake the sleeping tree
```

The box uses `sleep="init"` — a policy that starts the body asleep from
timestep 0. While asleep, integration is skipped entirely: `qpos` never
changes, so the box sits at its initial position with zero velocity, zero
acceleration, and zero gravity drift. This is not approximate — sleeping bodies
are completely frozen.

The narrowphase skip optimization means two sleeping bodies (Asleep + Asleep)
generate zero contacts between them. But a sleeping body touching a Static body
(the ground plane) still generates contacts — it's only the Asleep-Asleep pair
that is skipped.

| Parameter | Value |
|-----------|-------|
| Box | 2 kg, 0.3 m cube, sleep="init" |
| Ball | 0.5 kg, radius 0.1 m |
| Drop height | 2.0 m (ball center) |
| Impact time | ~0.6 s |
| Sleep tolerance | 0.05 |
| Integrator | Euler, dt = 2 ms |

**Key distinction:** the box doesn't wake because it "felt" the impact force.
It wakes because the collision system detected a contact between an Awake body
(ball) and an Asleep body (box), and `mj_wake_collision` flipped the box's tree
to awake. The force is a consequence, not the cause.

## Validation

Five automated checks at t=15s:

| Check | Expected |
|-------|----------|
| Box starts asleep | sleep_state(box) == Asleep at t=0 |
| Ball starts awake | sleep_state(ball) == Awake at t=0 |
| Box frozen while asleep | Box z == 0.15 at t=0.3 (no gravity drift) |
| Box wakes on impact | sleep_state(box) == Awake after collision |
| Both re-sleep by t=14 | nbody_awake == 1 |

## Run

```
cargo run -p example-sleep-wake-contact --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
