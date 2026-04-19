# Adaptive barrier width

[Li et al. 2020](../../appendices/00-references/00-ipc.md#li-2020) treats $\hat d$ as a global scalar — one value for the whole scene, chosen from the overall length scale and tuned to the thinnest-active-layer worst case. A global $\hat d$ small enough to satisfy [Ch 04 §02's $\hat d < \ell/2$ rule](../04-multi-layer/02-thin-material.md) at a $0.5$ mm liner forces every primitive in the scene — including the $10$ mm probe and the $2$ mm sleeve — to use the same small tolerance. The `sim-soft` real-time pipeline replaces this with a **per-primitive adaptive $\hat d$** schedule, the same scheme [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) exposes in its `SmoothCollisions` variant. Pairs in thick regions get a larger $\hat d_k$ matched to their local mesh scale; pairs at thin-layer interfaces get a smaller $\hat d_k$; and the total active-pair count shrinks because thick-region primitives whose operating gaps are well above the global-worst-case $\hat d$ are now culled at broadphase rather than carried through narrow-phase.

## The schedule

IPC Toolkit's `SmoothCollisions` stores per-primitive $\hat d$ arrays as `Eigen::VectorXd`:

- `vert_adaptive_dhat` — one entry per surface vertex.
- `edge_adaptive_dhat` — one entry per surface edge.
- `face_adaptive_dhat` — one entry per surface face (3D only).

Each entry is computed from the scene-level ratio parameter and the local contact distance. The implementation initializes all three arrays to the global $\hat d$ constant and then applies `assign_min` as candidate pairs are enumerated — meaning the stored value is overwritten only when a candidate-derived estimate is *smaller* than the current value. For a primitive $v$ with candidate collision pairs $\{c_j\}$, the final entry is:

$$ \hat d_v = \min\!\left(\hat d_\text{global},\ \min_j \text{ratio} \cdot \sqrt{d_{\text{contact},j}}\right) $$

where $\text{ratio}$ is the scalar value from `SmoothContactParameters::adaptive_dhat_ratio()` and $d_{\text{contact},j}$ is the contact-distance estimate for candidate pair $c_j$ at primitive $v$ (the local mesh-scale estimate of how close the primitives in the pair will approach). Because $\text{ratio}$ and $\sqrt{\cdot}$ are monotonic in positive arguments, the inner min simplifies to a min over gaps:

$$ \hat d_v = \min\!\left(\hat d_\text{global},\ \text{ratio} \cdot \sqrt{\min_j d_{\text{contact},j}}\right) $$

Equivalently: each primitive's $\hat d$ is the tighter of the global constant and a candidate-derived estimate from its closest candidate pair. A primitive that participates in no candidates retains $\hat d_\text{global}$ — so the global constant acts as an upper bound that the adaptive schedule can only tighten, never loosen.

At the pair level, `sim-soft` takes the tighter of the two primitives' local estimates for a pair of primitives $v_a, v_b$:

$$ \hat d_k = \min(\hat d_{v_a}, \hat d_{v_b}) $$

The min is a `sim-soft` correctness choice, not a claim about IPC Toolkit aggregation (the toolkit's per-pair dispatch for `SmoothCollisions` was not source-verified for this sub-chapter). The reasoning: at a thin-layer interface, the primitive whose local scale is smaller — the thin-layer primitive — produces the smaller per-primitive $\hat d_v$; the min-rule at the pair level lets that tighter estimate dominate, keeping the pair's barrier inside the [Ch 04 §02 $\hat d_k < \ell_k/2$ non-overlap condition](../04-multi-layer/02-thin-material.md). Max or geometric-mean aggregation would let a thick-region primitive's loose $\hat d_v$ widen the barrier at a thin-layer pair, breaking the correctness rule there.

The ratio parameter lives on the scene-level `SmoothContactParameters` struct; it is not per-primitive. The typical range is roughly $0.5$–$1.0$ for the problems `sim-soft` targets, with the setting chosen to satisfy the thinnest-layer correctness rule. Per-pair $\hat d_k$ overrides from the [Ch 04 §00 sliding](../04-multi-layer/00-sliding.md) interface-pair metadata are the escape hatch for scenes whose ratio-driven schedule does not cover a specific inter-layer pair.

## Why this shrinks the active-pair set

The naive reading is "adaptive $\hat d$ sometimes makes $\hat d_k$ larger at a pair than the global baseline, so more pairs are active, so more work." The opposite holds, because active-pair count is gated by the broadphase culling predicate $d < \hat d_k$, not by $\hat d_k$ alone:

- Under a **global $\hat d$** tuned to the thinnest-layer worst case (say $\hat d = 0.25$ mm for a $0.5$ mm liner scene), thick-region primitives whose operating gaps are $5$ mm or more are comfortably inactive ($5 \text{ mm} \gg 0.25 \text{ mm}$). Their barrier energy contribution is zero — but their broadphase *candidacy* is still computed each Newton iterate, and the candidate list is still generated and traversed through narrow-phase.
- Under the **adaptive schedule**, thick-region primitives get $\hat d_v \approx \text{ratio} \cdot \sqrt{5 \text{ mm}} \approx 1.6$ mm at $\text{ratio} = 0.7$. Their operating gap of $5$ mm is still $\gg \hat d_v$, so they stay inactive. What changes: the broadphase culling radius is now $\hat d_v$-local at each primitive rather than set globally. A candidate pair survives broadphase only when its gap is below the tighter of the two primitives' local $\hat d$; pairs whose primitives both happen to have small local $\hat d_v$ but whose gap exceeds that smaller $\hat d_v$ are filtered out earlier than they would be under the global-$\hat d$ baseline, reducing the candidate-pair count.

The practical effect: thick-region primitives that *cannot* become active at their local scale are filtered earlier. The candidate-pair list shrinks without changing which pairs actually contribute to the barrier energy. The reduction is scene-dependent — multi-scale scenes benefit most, uniform-thickness scenes benefit least — and the practitioner estimate for multi-scale configurations is a $2$–$5\times$ candidate-pair reduction over the global-$\hat d$ baseline. `sim-soft`'s multi-layer canonical problem is a multi-scale configuration; the adaptive schedule is what keeps the pair-count budget inside the Phase E rate targets.

## Connection to the thin-material regime

[Ch 04 §02 thin-material](../04-multi-layer/02-thin-material.md) established that the barrier's non-overlap condition is $\hat d_k < \ell_k/2$ at any pair $k$ inside a layer of thickness $\ell_k$. With per-primitive adaptive $\hat d$, this condition is automatically satisfied at thin-layer primitives: their local $d_{\text{contact}}$ is small (the inter-mesh gap between the thin layer's surfaces is small by construction), so $\hat d_v = \text{ratio} \cdot \sqrt{d_\text{contact}}$ is correspondingly small, and at a properly tuned ratio $\hat d_v < \ell_v/2$ holds without per-layer manual intervention.

The `sim-soft` scene-setup workflow: the designer sets one ratio value for the scene (with a conservative `sim-soft`-default around $0.5$ chosen to place the adaptive schedule safely inside the non-overlap rule for typical multi-layer canonical-problem geometry), the adaptive schedule produces per-primitive $\hat d$ values that satisfy the non-overlap rule everywhere, and no per-layer tuning is required. If a scene contains pathological thin-layer geometry that the default cannot cover, the ratio is a single scene-level scalar to tune down once. Per-pair $\hat d_k$ overrides on the [Ch 04 §00 sliding](../04-multi-layer/00-sliding.md) interface-pair metadata exist for the rare case where even a scene-wide ratio reduction is not specific enough.

## Attribution

The $\text{ratio} \cdot \sqrt{d_\text{contact}}$ schedule is IPC Toolkit-native. No published paper — Li 2020, C-IPC 2021, [GIPC](../../appendices/00-references/00-ipc.md#gipc-2024), [StiffGIPC](../../appendices/00-references/00-ipc.md#stiffgipc-2025), or the other 2022–2024 GPU-IPC works surveyed in [§00](00-gpu.md) — introduces this exact form as a primary contribution. The schedule is documented in the IPC Toolkit's `src/ipc/smooth_contact/smooth_collisions.{hpp,cpp}` source and exposed through the `adaptive_dhat_ratio()` accessor on `SmoothContactParameters`; no paper is cited at the call site. `sim-soft` ports it as a practitioner-heuristic choice grounded in the maintained reference implementation — same attribution framing as the $\hat d < \ell/2$ correctness rule in [Ch 04 §02](../04-multi-layer/02-thin-material.md).

## What this sub-leaf commits the book to

- **Per-primitive adaptive $\hat d$ is the barrier-tolerance schedule.** `sim-soft` ports the IPC Toolkit `SmoothCollisions` variant's per-primitive $\hat d_v$ arrays; the pair-level aggregation $\hat d_k = \min(\hat d_{v_a}, \hat d_{v_b})$ is a `sim-soft` correctness choice that preserves the [Ch 04 §02 $\hat d_k < \ell_k/2$ non-overlap rule](../04-multi-layer/02-thin-material.md) at thin-layer interfaces.
- **The schedule is $\hat d_v = \min(\hat d_\text{global},\ \text{ratio} \cdot \sqrt{\min_j d_{\text{contact},j}})$ per primitive**, with `assign_min` aggregation over candidate pairs and a scene-level ratio. The global constant is the upper bound; the adaptive rule can only tighten it. Ratio is the single scene-tunable scalar; per-pair overrides on [Ch 04 §00 sliding](../04-multi-layer/00-sliding.md) interface-pair metadata are the escape hatch.
- **The $2$–$5\times$ candidate-pair reduction comes from earlier broadphase culling at primitive-local $\hat d_v$**, not from changing which pairs contribute barrier energy. The reduction is scene-shape-dependent and most pronounced on multi-scale configurations.
- **[Ch 04 §02's $\hat d_k < \ell_k/2$ non-overlap rule](../04-multi-layer/02-thin-material.md) is satisfied automatically** by the adaptive schedule at a properly-tuned ratio. Thin-material scenes do not require per-layer manual tuning.
- **Attribution is IPC Toolkit-native.** The schedule is a production-code practitioner heuristic; no paper beyond the [IPC Toolkit anchor](../../appendices/00-references/00-ipc.md#ipc-toolkit) is cited for it.
