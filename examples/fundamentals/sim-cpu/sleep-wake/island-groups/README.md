# Island Groups — Independent Constraint Islands

Constraint islands are independent — disturbing one doesn't affect the other.
The engine discovers islands each step by flood-filling the contact/constraint
adjacency graph, and each island's sleep state is evaluated atomically.

## What you see

- **Two stacks** of 3 boxes each, 3 m apart, drop and settle into **blue**
  (asleep) columns
- At t=7 s, an upward impulse launches the top box of **Stack A** (left) — all
  three boxes in Stack A turn **orange** as the wake cascades through contacts
- **Stack B** (right) stays **blue** the entire time — it's a separate island,
  completely unaffected
- Stack A re-settles and turns blue again

## Physics

Island discovery runs every step via DFS flood-fill over a CSR adjacency graph
built from contacts, equality constraints, joint limits, and tendon limits:

```
Phase 1: Extract edges (tree pairs from contacts, equalities, ...)
Phase 2: Build CSR adjacency graph (rownnz, rowadr, colind)
Phase 3: Flood-fill DFS → connected components (islands)
Phase 4: Populate per-island arrays (trees, DOFs, constraint rows)
```

Only **awake** trees participate. When all bodies sleep, `nisland == 0` — there
are no islands to discover. When Stack A wakes, its 3 trees re-enter the island
graph and form island #0. Stack B's trees remain sleeping and absent.

The wake cascade within Stack A works through `mj_wake_collision`: the impulse
wakes a3 directly (via `xfrc_applied`), then a3's contact with a2 wakes a2,
then a2's contact with a1 wakes a1 — all within 3 timesteps.

| Parameter | Value |
|-----------|-------|
| Stacks | 2 stacks of 3 boxes, 1 kg each |
| Stack separation | 3.0 m center-to-center (2.7 m edge-to-edge) |
| Box size | 0.3 m cubes |
| Impulse | 60 N upward on a3 for 50 ms at t=7 s |
| Sleep tolerance | 0.05 |
| Integrator | Euler, dt = 2 ms |

**Key distinction:** islands are not permanent groups — they're recomputed every
step from the current contact state. A stack that breaks apart mid-air would
split into separate singletons until the pieces re-establish contact.

## Validation

Five automated checks at t=15s:

| Check | Expected |
|-------|----------|
| All asleep by t=6 | nbody_awake == 1 (world only) |
| Islands == 0 pre-disturb | nisland == 0 when all sleeping |
| Stack A wakes | All 3 Stack A bodies Awake at t=7.1 |
| Stack B stays asleep | All 3 Stack B bodies Asleep at t=7.1 |
| Stack A re-sleeps | All 3 Stack A bodies Asleep by t=14 |

## Run

```
cargo run -p example-sleep-wake-islands --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
