# contact/

The `contact/` module ships [Part 4](../../40-contact/00-why-ipc.md)'s contact solver: IPC barrier energy, continuous collision detection, smoothed Coulomb friction, broad-phase BVH, and self-contact pair generation. The parent [Ch 00 module-layout](../00-module-layout.md) and the [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) commit the whole crate to **contact as an energy term in the same total potential energy the solver minimizes** — not a separate LCP, not a post-hoc projection, not a penalty force scheduled alongside elasticity. This module supplies the energy; [`solver/`](04-solver.md) adds it to $U_n(x; \theta)$ and differentiates through it like any other scalar.

## What the module owns

| Sub-area | Responsibility | Where it is specified |
|---|---|---|
| IPC barrier | $b(d, \hat d) = -(d - \hat d)^2 \ln(d/\hat d)$ on $(0, \hat d)$, $C^2$ everywhere, divergent at $d \to 0^+$, zero + zero derivatives at $d = \hat d$ | [Part 4 Ch 01 §00](../../40-contact/01-ipc-internals/00-barrier.md) |
| Adaptive κ schedule | Per-step stiffness multiplier on the barrier; grown progressively when the gap closes fast | [Part 4 Ch 01 §01](../../40-contact/01-ipc-internals/01-adaptive-kappa.md) |
| Continuous collision detection | Per-step earliest-time-of-impact for point-triangle and edge-edge primitive pairs; interval arithmetic or analytic CCD | [Part 4 Ch 01 §02](../../40-contact/01-ipc-internals/02-ccd.md) |
| Friction | Smoothed Coulomb friction as an additional energy term, integrated with the barrier | [Part 4 Ch 02](../../40-contact/02-friction.md) |
| Self-contact broadphase | BVH over the mesh surface-triangle projection, proximity-pair generation, broad-phase culling | [Part 4 Ch 03](../../40-contact/03-self-contact.md) |
| IPC barrier VJP | Hand-written reverse-mode kernel for the contact energy's gradient; fused, not op-by-op | [Part 6 Ch 01 §02](../../60-differentiability/01-custom-vjps/02-contact-barrier.md) |

## Four claims

**1. IPC-only. Penalty and impulse are both rejected at the module level.** The [Part 4 Ch 00 why-IPC chapter](../../40-contact/00-why-ipc.md) carries the argument; this module does not ship fallback implementations of penalty or impulse contact. A scene that wants penalty contact is not a `sim-soft` scene — it is a scene for a different contact library. The rejection is not a political preference; it is the downstream consequence of the [Part 1 Ch 03 thesis's](../../10-physical/03-thesis.md) commitment that energy-based physics is differentiable by construction, and penalty / impulse are not.

**2. Friction is an energy term, not a conditional branch.** [Part 4 Ch 02 §01 smoothed-Coulomb](../../40-contact/02-friction/01-smoothed.md) derives the smooth-transition form $\mu_c \cdot \|v_T\| \cdot \tanh(\|v_T\|/\epsilon)$ (schematic; the rigorous form lives in Part 4) that resolves the stick-slip decision as a continuous energy rather than a branch on relative-tangential-velocity sign. This preserves the "one scalar objective, one Newton" property Part 5 Ch 00 Claim 1 commits to — no active-set iteration, no LCP — and is why the friction term is in `contact/` and not in a separate `friction/` module.

**3. Broadphase is a mesh-adjacency computation, not a spatial-hash over raw vertices.** The BVH is built over mesh surface triangles with adjacency taken directly from [`mesh/`](02-mesh.md); self-contact proximity queries walk the BVH and then check adjacency to avoid reporting neighbor-triangle pairs as colliding. [Part 4 Ch 03 §01 proximity](../../40-contact/03-self-contact/01-proximity.md) names this as the standard IPC self-contact recipe. A spatial-hash-over-vertices implementation would work functionally but would re-discover adjacency at each query; the mesh-adjacency design is cheaper and is the one implemented.

**4. Contact owns a hand-written VJP too.** Like [`element/`](01-element.md)'s FEM assembly VJP, the IPC barrier energy's gradient is a fused kernel registered against [`autograd/`](07-autograd.md)'s VJP API. Each active contact pair contributes a handful of elementary operations to the forward energy but dozens to the op-by-op reverse traversal (triangle-point distance derivatives, barrier chain rule, Kronecker structure across pairs); the fused kernel records one opaque node per barrier-evaluation batch, [Part 6 Ch 01 §02](../../60-differentiability/01-custom-vjps/02-contact-barrier.md). This matters because contact is precisely where a design gradient should be cheap — conforming-pressure rewards depend on how many pairs are active and where, and a naïve op-by-op tape over dense contact would make every acquisition-function evaluation in [Part 10](../../100-optimization/00-forward.md) pay.

## What the module does not carry

- **No solver state.** The Newton loop, the line search, the SPD Hessian, the sparse factorization — all [`solver/`](04-solver.md)'s. This module contributes $\Psi_\text{contact}(x)$ and its first two derivatives; it does not decide when or how they are minimized.
- **No multi-layer material awareness.** Contact between stacked sleeves ([Part 4 Ch 04](../../40-contact/04-multi-layer.md)) is driven by the surface-triangle proximity pairs the broadphase emits; the layer-sliding / no-penetration distinction is a downstream consequence of scene configuration (two separate bodies in contact) plus the standard IPC barrier, not a special-case code path.
- **No impact response.** Classical "impulse" concerns (restitution, normal-velocity-jump, Coulomb stick-slip as event) do not apply: `sim-soft` contact is a continuous energy minimization, and bounce / separation emerge from the inertial-plus-elastic-plus-contact energy balance, not from an impact event.

## What this commits downstream

- **[`solver/`](04-solver.md) adds this module's energy to $U_n$ without special-casing.** Contact appears as one more term in the scalar objective; the Newton tangent is $K + H_\text{contact} + M/\Delta t^2$ per [Part 5 Ch 00 Claim 2](../../50-time-integration/00-backward-euler.md), and the contact block's PSD structure is handled by the per-pair rank-1 positive contributions of the barrier Hessian.
- **[Part 4 Ch 05 real-time](../../40-contact/05-real-time.md)'s throughput targets bind on this module's GPU port in Phase E** — the wgpu kernels for CCD and barrier-evaluation live in [`gpu/`](06-gpu.md), but the CPU reference implementation here is the oracle they regress against.
- **[Part 11 Ch 04 regression tests](../04-testing/01-regression.md)** against MuJoCo flex evaluate contact-pressure curves on the canonical problem; discrepancies against flex-with-penalty-contact are expected (the IPC answer is different by design), and the regression pins the `sim-soft` answer, not the flex answer, as ground truth.
