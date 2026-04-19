# SDF operations — CSG, smooth blends, displacement

The [Ch 00 parent's Claim 2](../00-sdf-primitive.md) commits to a closed operator algebra on the `Sdf` trait sufficient for every geometric composition the canonical problem and its extensions need. This leaf writes out the operators, names their provenance, and is specific about where they are differentiable.

The algebra structure — set-theoretic operations, blending, and displacement as transformations of a defining real function — follows [Pasko, Adzhiev, Sourin & Savchenko 1995](../../appendices/00-references/03-diff-sim.md#pasko-1995)'s FRep framework. The concrete formulas for the smoothed variants are drawn from the optimization-community softmin idiom and the SDF-graphics practitioner lineage; attributions appear inline below.

## Sharp CSG

For SDFs $\phi_a, \phi_b : \mathbb{R}^3 \to \mathbb{R}$ with the convention that $\phi < 0$ is inside the shape and $\phi > 0$ is outside, the set-theoretic operators are:

| Operation | Formula | Convention |
|---|---|---|
| Union $A \cup B$ | $\min(\phi_a, \phi_b)$ | Point is inside iff inside at least one operand |
| Intersection $A \cap B$ | $\max(\phi_a, \phi_b)$ | Point is inside iff inside both operands |
| Difference $A \setminus B$ | $\max(\phi_a, -\phi_b)$ | Point is inside iff inside $A$ and outside $B$ |

These are $C^0$ everywhere — $\min$ and $\max$ are continuous — and $C^\infty$ away from the locus where the operands are equal. The kink locus for a union is the set where $\phi_a = \phi_b$, which geometrically is the crease where the two surfaces meet. Gradients evaluated strictly inside a single operand's interior or exterior are the gradient of that operand; gradients across the crease flip between the two operand gradients with no interpolation.

Sharp CSG is adequate for designs whose primitives meet at sharp edges (mechanical-feel parts, rectilinear features). When the designer wants a rounded meet, sharp CSG is inadequate — the composed SDF's surface normal is discontinuous at the crease — and the smoothed variants below take over.

## Smoothed variants

A smoothed union with blend radius $k > 0$ replaces the kink at the crease with a rounded fillet of approximate radius $k$. `sim-soft` exposes the log-sum-exp form as the default:

$$
\phi_\cup^k(p) = -k\, \log\!\left(e^{-\phi_a(p)/k} + e^{-\phi_b(p)/k}\right)
$$

This is the [softmin](https://en.wikipedia.org/wiki/Soft_minimum) / log-sum-exp trick from optimization literature, applied with a sign flip to return the smaller of the two operands: as $k \to 0^+$ the formula converges to $\min(\phi_a, \phi_b)$, and for $k > 0$ it is $C^\infty$ everywhere. The analogous smooth intersection replaces $e^{-\phi/k}$ with $e^{+\phi/k}$:

$$
\phi_\cap^k(p) = +k\, \log\!\left(e^{+\phi_a(p)/k} + e^{+\phi_b(p)/k}\right)
$$

and smooth difference is $\phi_\cap^k(\phi_a, -\phi_b)$.

The practitioner SDF-graphics tradition (Inigo Quilez's widely-referenced [SDF primitives notes](https://iquilezles.org/articles/distfunctions/)) offers polynomial-form smooth-unions with a bounded-support blend kernel, which have the advantage of being exactly $\min(\phi_a, \phi_b)$ outside a blend-band of width $k$ around the crease and the disadvantage of being only $C^1$ at the support-boundary where the polynomial piece meets the sharp-min piece. `sim-soft` ships the log-sum-exp default for $C^\infty$ smoothness and because its gradient is a simple weighted combination of the operand gradients — $\phi_\cup^k$'s gradient is the softmax-weighted average $\sum_i w_i \nabla \phi_i$ with $w_i = e^{-\phi_i/k} / \sum_j e^{-\phi_j/k}$. The polynomial forms are available as opt-ins for geometry-sensitive applications where bounded-support is architecturally useful.

The blend radius $k$ is a first-class design parameter. `cf-design` exposes it per-operation; `sim-soft`'s differentiability machinery treats $k$ as an element of the design-parameter vector $\theta$ and propagates gradients through $\partial \phi_\cup^k / \partial k$ the same way it handles primitive-radius or wall-thickness parameters. Moving a corner from sharp to rounded, or inflating a fillet radius, is a smooth design-space move.

## Spatial transforms

Translations, rotations, and uniform scalings are applied by inverse-transforming the query point rather than transforming the SDF's internal state. For a transform $T : \mathbb{R}^3 \to \mathbb{R}^3$ with inverse $T^{-1}$ that preserves distances (Euclidean isometries: translation, rotation):

$$
\phi_T(p) = \phi(T^{-1}(p))
$$

For a uniform scale by factor $s > 0$, the query inversion gives $\phi_s(p) = s \cdot \phi(p/s)$ — the extra factor of $s$ preserves the distance property. `sim-soft` requires transforms to preserve the distance property (or to produce at least a conservative lower bound on the true distance) because [Ch 01's meshing pipelines](../01-tet-strategies.md) rely on $|\phi|$ being a local distance lower bound for correctness guarantees. Non-uniform scaling and shears are rejected at the `cf-design`/`sim-soft` boundary for that reason; the designer who needs a non-uniform-scale effect composes it via independent primitive re-parameterization rather than via transform.

## Displacement

Displacement adds a scalar perturbation $d : \mathbb{R}^3 \to \mathbb{R}$ to the SDF:

$$
\phi_\text{disp}(p) = \phi(p) + d(p)
$$

Typical uses: surface texture (high-frequency $d$ with amplitude $\ll$ feature size), bump-map-like details. The perturbed field is no longer a true signed distance function — $|\phi_\text{disp}|$ is not a tight distance to the perturbed surface in general (it may over- or under-estimate the true distance depending on the direction and magnitude of $d$) — but it remains a valid implicit-surface definition and the meshing pipelines in [Ch 01](../01-tet-strategies.md) tolerate bounded-error distance functions (the isosurface extraction finds the $\phi = 0$ level set; the extracted surface is correct even when $|\phi|$ is not a tight distance value away from the surface).

`sim-soft`'s `Sdf::grad` method is expected to return a unit-length gradient for true distance fields and the unnormalized formal gradient for displacement-augmented fields. Consumers that need a surface normal (the meshing pipeline, the rendering path via [Part 9 Ch 05](../../90-visual/05-sim-bevy.md)) normalize as needed.

## The closed-algebra property

Every operator above takes `Sdf` trait implementations and returns an `Sdf` trait implementation. `cf-design`'s composition tree is therefore a tree of `Sdf`s at every internal node; the root is a single `Box<dyn Sdf>` that `sim-soft`'s `sdf_bridge/` module queries. No intermediate mesh is created, no precision is lost in operator chains, and a design with dozens of CSG+blend operators deep evaluates by recursive `eval()` calls — the stack depth equals the tree depth, and each leaf is a cheap primitive-SDF evaluation.

The differentiability of each operator is preserved by the Rust trait: `Sdf::grad` is an associated method, and each composition operator implements `grad` either analytically (the sharp CSGs via chain rule on the active operand, the log-sum-exp via the softmax-weighted sum above) or via forward-mode autodiff on `eval` inside `cf-design`. The per-operator analytic forms are the default; autodiff is the fallback for primitive types where deriving by hand is not worth the code cost.

## What this sub-leaf commits the book to

- **Sharp CSG uses $\min / \max / \max(-\cdot, \cdot)$ with the $\phi < 0$ inside convention.** $C^0$ at operand-equality loci, $C^\infty$ elsewhere.
- **Smoothed variants use log-sum-exp with blend radius $k$ as the default.** $C^\infty$ everywhere for $k > 0$; converges to the sharp form as $k \to 0^+$. Quilez-tradition polynomial forms are available as opt-ins.
- **Blend radius $k$ is a differentiable design parameter.** Part of $\theta$; `cf-design` exposes it; `sim-soft` propagates gradients through it.
- **Spatial transforms are query-point inversions.** Translation, rotation, and uniform scaling are supported; non-uniform scaling and shears are rejected to preserve the bounded-distance property [Ch 01](../01-tet-strategies.md) relies on.
- **Displacement adds scalar perturbations and may produce non-tight distance functions.** $|\phi_\text{disp}|$ is not a tight distance to the perturbed surface in general; the meshing pipelines consume the $\phi = 0$ level set directly and tolerate this. Rendering consumers normalize the gradient as needed.
- **The operator set is closed under SDF composition.** Every operator takes `Sdf` and returns `Sdf`; the `cf-design` composition tree is a tree of `Sdf`s at every node.
