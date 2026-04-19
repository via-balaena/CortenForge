# Caching strategies

Three earlier sub-chapters each introduced a piece of per-Newton-iterate state that `sim-soft` persists across Newton iterations within a timestep: [Ch 01 §01](../01-ipc-internals/01-adaptive-kappa.md)'s adaptive $\kappa$ scalar, [Ch 03 §00](../03-self-contact/00-bvh.md)'s LBVH, and [Ch 04 §00](../04-multi-layer/00-sliding.md)'s interface-pair table. This sub-leaf extends each piece to persist *across* timesteps as well — three load-bearing caching techniques that together absorb the per-frame rebuild overhead that would dominate a naive "rebuild everything" pipeline at 30 FPS.

The aggregate across-frame save over a from-scratch pipeline is in the $3$–$10\times$ range, depending on how stable the scene is frame-to-frame. Scenes where most contact pairs persist across frames — the canonical cavity-probe equilibrium, a robot grasping a compliant object, a multi-layer assembly settling into contact — hit the high end of the range. Scenes with rapid pair turnover — high-speed cloth shedding, impact-dominated dynamics — hit the low end.

## BVH refit versus rebuild across frames

[Ch 03 §00 BVH](../03-self-contact/00-bvh.md) established the within-timestep refit-or-rebuild policy: inside a single timestep's Newton iterations, leaf AABBs are refit to new vertex positions in $O(n)$; topology changes force an $O(n \log n)$ Morton-sort rebuild. The across-frame caching extension applies the same rule at the timestep boundary.

**If topology did not change between timestep $t$ and $t+1$, the tree from $t$'s final Newton iterate is reused as $t+1$'s starting tree.** The timestep boundary is treated as just another refit, not a rebuild. For steady-state scenes with fixed topology, the $O(n \log n)$ rebuild runs once at simulation start and the entire simulation is refits from that point forward — the Morton-sort cost pays only for topology changes, not for each frame.

The invalidation rule is the same one [Ch 03 §00](../03-self-contact/00-bvh.md) named: topology change forces rebuild. In `sim-soft`'s canonical workload, topology changes come from [Part 3 Ch 02 adaptive refinement](../../30-discretization/02-adaptive.md) (stress-feedback-triggered, infrequent) and from [Part 7 Ch 04 live remesh](../../70-sdf-pipeline/04-live-remesh.md) (design-mode geometry edits, also infrequent). The across-frame rebuild cost amortizes over many quiet frames between topology events.

## Persistent inter-mesh pair lists

[Ch 04 §00 sliding](../04-multi-layer/00-sliding.md) introduced the **interface-pair table**: at a sliding inter-layer interface, the set of A-vs-B primitive pairs is known at scene-ingest time from the double-layer vertex map in [Part 3 Ch 03 §01 sliding-interfaces](../../30-discretization/03-interfaces/01-sliding.md), and the pairs persist for the simulation's duration. These are *expected-persistent* pairs — they are in contact by design, not as a transient detection.

Caching the persistent pairs as tracked state rather than re-detecting them at each frame saves both the broadphase candidate-generation cost *and* the narrow-phase confirmation cost on the persistent subset. The per-frame broadphase pass is consulted only for *transient* pairs — self-contact pairs that appear when a body folds onto itself, inter-mesh pairs at non-interface surfaces that approach each other dynamically. The persistent pairs skip both broadphase and narrow-phase and go straight to per-pair distance computation and barrier evaluation at the new configuration.

The payoff grows with interface area. Persistent-pair count scales linearly with interface area; on large multi-layer scenes (a full sleeve-liner assembly, a multi-ply composite) the persistent pairs can outnumber transient pairs by an order of magnitude or more. [Part 3 Ch 03 §01 sliding](../../30-discretization/03-interfaces/01-sliding.md)'s Phase H sizing budget already incorporates this growth; the persistent-pair caching is what keeps its wall-clock cost bounded within the real-time budget.

## Warm-start $\kappa$

[Ch 01 §01 adaptive-kappa](../01-ipc-internals/01-adaptive-kappa.md) established that $\kappa$ is a scalar adjusted per Newton iterate by the Li 2020 growth rule: if any active pair's gap crosses the safeguard threshold, $\kappa$ is doubled. Growth is monotone within a timestep — $\kappa$ only increases as the iterate progresses toward convergence.

Warm-starting $\kappa$ across timesteps: **$\kappa$ at the start of timestep $t+1$ is set to $\kappa$ at the end of timestep $t$**, not reset to the Li-2020-default initial value. The rationale: the contact configuration a scene settles into rarely changes drastically from one frame to the next; the $\kappa$ value that was stiff-enough to hold the safeguard threshold at timestep $t$ remains stiff-enough at $t+1$ for the same pair set, modulo small configurational drift. Resetting $\kappa$ to its conservative initial value each frame would force the growth rule to re-climb the same $\kappa$ ladder, costing several Newton iterations per frame.

The warm-start is valid because $\kappa$ is scalar — no sparsity pattern or pair-list invalidation is needed, and the scalar can be persisted directly across timesteps alongside the BVH and the persistent-pair list. The across-frame save is in reduced Newton-iteration count at the *start* of each frame, not in reduced per-iteration cost; [Ch 01 §01 adaptive-kappa](../01-ipc-internals/01-adaptive-kappa.md) forward-referenced this sub-leaf as the natural home for the across-timestep persistence because it's state management, not algorithm variance.

## Invalidation

The three caches share one invalidation trigger: **mesh topology change invalidates everything cacheable**. Topology change from [Part 3 Ch 02 adaptive refinement](../../30-discretization/02-adaptive.md), [Part 7 Ch 04 live remesh](../../70-sdf-pipeline/04-live-remesh.md), or scene re-ingest rebuilds the LBVH, regenerates the interface-pair table (primitives have been added, removed, or re-indexed), and resets $\kappa$ to its default initial value. Live remesh's warm-started state-transfer protocol from [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md) handles the cache-rebuild coordination at topology boundaries; the caching system exposes an invalidation hook and Part 7's remesh pipeline calls it.

**Configuration change alone — however large — does not invalidate the caches.** A scene where the probe moves substantially between frames still reuses the same BVH (leaf AABBs refit to new positions), the same interface-pair table (the pairs persist because they are a scene-ingest construction), and the same $\kappa$ (warm-started). The intersection-free invariant is preserved by the [per-step CCD line-search filter](../01-ipc-internals/02-ccd.md), which is independent of the caches and is re-run every iterate. Large configuration change increases Newton iteration count and may fire the adaptive-$\kappa$ growth rule repeatedly within a timestep, but it does not trigger cache invalidation; `sim-soft` is architected so topology is the rare event, not a per-frame concern.

## What this sub-leaf commits the book to

- **BVH, interface-pair tables, and $\kappa$ persist across timesteps by default.** Rebuilding these structures per frame is a naive baseline the real-time pipeline does not incur. Across-frame state-reuse is a first-class pipeline commitment, not an optional optimization.
- **The sole invalidation trigger is mesh topology change.** Configuration change — however large — refits the BVH and preserves all other cached state. Topology change (adaptive refinement, remesh, re-ingest) invalidates all three caches through a coordinated rebuild path.
- **Across-frame speedup estimate is $3$–$10\times$, scene-stability-dependent.** The estimate is per-technique-additive: BVH refit-vs-rebuild saves the Morton-sort ($O(n \log n) \to O(n)$) at each timestep boundary; persistent pair lists save both broadphase and narrow-phase on the persistent subset each frame; warm-start $\kappa$ saves several Newton iterations per frame for steady-state scenes.
- **Caching is a real-time commitment, not a performance afterthought.** [Part 11 Ch 03 Phase E](../../110-crate/03-build-order.md)'s 30 FPS / 5 Hz targets assume the caches are live; a from-scratch pipeline would not clear the targets on the canonical problem at the planned tet counts.
