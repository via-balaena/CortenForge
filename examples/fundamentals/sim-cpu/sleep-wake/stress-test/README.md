# Stress Test — Sleep/Wake Invariants

Headless validation of the entire sleep/wake subsystem: velocity threshold,
countdown mechanics, all three wake triggers, constraint island discovery,
policy flags, narrowphase optimization, and bookkeeping consistency.

## Checks (18)

| # | Check | What it validates |
|---|-------|-------------------|
| 1 | Sleep after threshold | Body sleeps when velocity drops below tolerance |
| 2 | Sleeping qvel = 0 | Sleeping DOFs have bitwise-zero velocity |
| 3 | Sleeping qacc = 0 | Sleeping DOFs have bitwise-zero acceleration |
| 4 | Sleeping qfrc_applied = 0 | Sleep transition zeroes applied forces |
| 5 | Countdown duration | At least MIN_AWAKE (10) sub-threshold steps before sleep |
| 6 | Wake on contact | Ball hitting init-sleep box wakes it |
| 7 | Wake on equality | Equality constraint couples sleeping tree to awake tree |
| 8 | Wake on xfrc_applied | Applied force wakes sleeping body |
| 9 | Wake cascade | Force on end box propagates wake through contact chain |
| 10 | No island for singletons | Isolated boxes are singletons (tree_island == -1) |
| 11 | Shared island | Stacked boxes share a constraint island |
| 12 | Island count | Two separate stacks produce nisland == 2 |
| 13 | Selective wake | Poking one stack wakes only that stack |
| 14 | Countdown reset | Velocity spike resets countdown timer to full |
| 15 | Sleep policy Never | `sleep="never"` body never sleeps |
| 16 | ENABLE_SLEEP disabled | Clearing the flag prevents all sleeping |
| 17 | Narrowphase skip | Two sleeping bodies generate zero mutual contacts |
| 18 | nbody_awake bookkeeping | Reported count matches manual tally |

## Run

```
cargo run -p example-sleep-wake-stress-test --release
```
