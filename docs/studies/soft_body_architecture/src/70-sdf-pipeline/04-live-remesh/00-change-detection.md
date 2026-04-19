# Change detection

The [Ch 04 parent's Claim 2](../04-live-remesh.md) commits change detection to a content-hash-based classifier that runs across the opaque [`cf-design` ↔ `sim-soft` boundary](../00-sdf-primitive/02-cf-design.md). This leaf names the hash function, the probe-point policy, the two-stage classifier that distinguishes the three [`EditClass`](../04-live-remesh.md) variants, and why structural tree-diffing is rejected.

## The probe-point policy

The primary change hash is computed by evaluating the composed `SdfField` and each slot of the `MaterialField` at a fixed sparse set of probe points drawn from the `SdfField`'s `bbox`. The points are generated once at scene construction from a deterministic low-discrepancy sequence (Halton or Sobol) seeded by the bbox; they do not move across edits.

Probe-point count is a budget-vs-discrimination trade-off:

- **Too few points** — two SDF instances that differ only in a small local region hash identically, and the classifier misses real changes (false negative).
- **Too many points** — each edit's hash takes longer to compute, inflating the ≤50 ms parameter-only hot-path budget ([Ch 04 Claim 1](../04-live-remesh.md)).

`sim-soft` ships ~100 points as the default (tied to `resolution_hint`), which gives microsecond-scale hash computation and discrimination sensitivity at the same edit scale the designer would perceive. The point count is a tunable per-scene parameter that `cf-design` can raise for high-sensitivity designs or lower for high-throughput exploration.

## Two-stage classification

The change detector runs as a pre-classifier gate plus a two-stage classifier:

1. **Gate: full-boundary hash.** Evaluate `SdfField.eval(p)` and every `MaterialField` slot's `Field::sample(p)` at each probe point. Concatenate and hash with a fast non-cryptographic hash (xxHash or similar). If the hash matches the previous frame's, no change detected; return early and skip both the classifier and the re-solve entirely. Only when the hash differs does the classifier below run.
2. **Stage 1: topology hash.** Extract the iso-surface on a coarser grid (~30³, chosen to balance topology sensitivity against latency — ms-scale) via marching-cubes or surface-nets, count connected components, compute the genus (Euler characteristic), count cavity / enclosed-void regions. Hash the topology-descriptor tuple. If this topology hash matches the previous frame's, the edit is *topology-preserving* — the tet mesh's combinatorial structure does not need to change.
3. **Stage 2: field classifier.** If the topology hash matches, split the full-boundary hash into the `SdfField` part and the per-slot `MaterialField` parts. The part that changed identifies the edit class: if only the `SdfField` hash changed but topology held, the edit is `ParameterOnly` (geometry moved by less than one iso-surface cell); if one or more `MaterialField` slot hashes changed, the edit is `MaterialChanging` (whether or not the `SdfField` hash also moved — the material-re-sampling-plus-Hessian-refactor path subsumes the parameter-only pass). If the topology hash differs, classification short-circuits to `TopologyChanging` — no further discrimination is needed, because the topology-changing path is handled uniformly in [§02 state-transfer](02-state-transfer.md).

## Output: the `EditClass` enum

The classifier returns an `EditClass`:

```rust
pub enum EditClass {
    ParameterOnly,          // ≤50 ms budget; warm-start in §01
    MaterialChanging,       // ≤200 ms budget; re-sample + Hessian re-factor in §01
    TopologyChanging,       // ≤500 ms budget; full re-mesh + state transfer in §02
}
```

The no-change case is expressed as the gate returning early before the classifier runs — not as a fourth enum variant. This keeps `EditClass`'s three-variant shape consistent across [Ch 04 spine](../04-live-remesh.md), [Part 6 Ch 05 §03 FD-wrappers](../../60-differentiability/05-diff-meshing/03-fd-wrappers.md) and [Part 10 Ch 00 forward](../../100-optimization/00-forward.md), which all pattern-match against exactly those three variants.

Downstream: [§01 warm-start](01-warm-start.md) consumes `ParameterOnly` and `MaterialChanging`; [§02 state-transfer](02-state-transfer.md) consumes `TopologyChanging`.

## Why structural tree-diffing is rejected

An alternative to content-hashing is to walk `cf-design`'s composition tree and diff node-by-node (operator-type comparison, parameter-value comparison, child-subtree recursion). Structural diffing is more precise — it can distinguish between edits that produce identical iso-surfaces and edits that differ but happen to hash the same. But precision is not the governing concern here; opacity is.

The [Ch 00 §02 cf-design boundary](../00-sdf-primitive/02-cf-design.md) commits to `sim-soft` being agnostic to `cf-design`'s internal representation — `cf-design` may use a procedural tree, grid-sampled volume, neural SDF, or anything else behind the `Sdf` trait. Structural diffing would require `sim-soft` to understand each of those representations, which is exactly the coupling the trait-object abstraction is designed to prevent.

Content-hashing treats `cf-design` as opaque, adds one cheap function evaluation per probe point, and discriminates edits at the granularity that actually affects the simulation. The precision loss from hash collisions (two genuinely-different edits hashing identically) is recoverable — at worst, the gate returns early (reporting no change) and the next edit's hash picks it up; there is no silent simulation corruption, because the Newton loop runs deterministically on the mesh + material cache regardless of how the edit was classified.

## Hash collisions and failure modes

Hash collisions are the honest failure mode of content-based classification. Two mitigations:

- **Salt with the bounding-box hash.** The primary hash includes the bbox as a prefix — two fields with identical probe-point outputs but different bboxes (e.g., a translation that moves the whole field out of the probe grid) still hash differently.
- **Periodic full-hash verification.** At a lower cadence (every $N$ edits, configurable), the gate recomputes the hash over a denser probe set (~1k points) and compares to the expected. A mismatch between the coarse and dense hashes signals that the coarse gate has been returning early when it should not have; the classifier falls back to `TopologyChanging` to force a safe re-solve. This catches the long-tail case where a persistent hash collision has gone undetected.

Cryptographic hashing is not used — the collision resistance of a cryptographic hash does not help when the underlying probe-point evaluation is already sparse; the bottleneck is sampling density, not hash strength.

## What this sub-leaf commits the book to

- **Primary hash: ~100 probe points from a low-discrepancy sequence in the bbox.** Tunable per scene; microsecond-scale computation on the parameter-only hot path.
- **Two-stage classifier: topology hash → field hash.** Topology hash uses coarse-grid iso-surface + connected-components + genus; field hash splits geometry vs. material components of the full-boundary hash.
- **`EditClass` is a three-variant enum: ParameterOnly / MaterialChanging / TopologyChanging.** No-change short-circuits at the gate, before the classifier runs; a fourth "Unchanged" variant would break the three-variant shape Ch 04 spine and Parts 6 + 10 match against.
- **Structural tree-diffing is rejected.** The [Ch 00 §02 opaque-boundary commitment](../00-sdf-primitive/02-cf-design.md) requires content-based change detection.
- **Hash collisions mitigated by bbox-salting and periodic-dense verification, not by cryptographic strength.** The bottleneck is probe-point density, not hash quality.
