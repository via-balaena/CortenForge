# SubtreeCom Sensor

Center-of-mass tracking for a multi-body subtree.

## What it demonstrates

- `<subtreecom>` reads the COM of all bodies in the subtree rooted at a given body
- Position-stage sensor, computed from body masses and FK positions
- Non-trivial with a 2-link chain (COM depends on both joint angles)
- No energy tracking — the undamped double pendulum is chaotic and
  energy drifts with any practical timestep

## Expected visual behavior

A two-link arm (shoulder + elbow hinges, both undamped) swings chaotically.
The shoulder starts at 30° and the elbow at 45°, creating interesting coupled
motion. Both links are capsule rods with a sphere tip on the lower link.

## Expected console output

```
t=  1.0s  com=(+0.XXXX,+0.0000,-0.XXXX)
t=  2.0s  com=(-0.XXXX,+0.0000,-0.XXXX)
...
```

The COM X-component swings widely as both joints move. Y stays at zero
(both hinges rotate in the XZ plane). Motion becomes chaotic over time.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| SubtreeCom == data | sensor matches `data.subtree_com[body_id]` | < 1e-12 |
| COM X range > 0.1m | chain actually swings (COM moves) | range > 0.1 |
| t=0 analytical COM | COM matches hand-computed FK: `(m1·p1 + m2·p2) / (m1+m2)` | < 1e-6 |

## Run

```bash
cargo run -p example-sensor-subtree-com --release
```
