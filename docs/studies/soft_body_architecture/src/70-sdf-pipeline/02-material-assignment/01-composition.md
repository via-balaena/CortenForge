# Multi-field composition

The [Ch 02 parent's Claim 2](../02-material-assignment.md) names the interface band as where multi-material composition bites: a tet straddling a stiffness transition samples a weighted-mean value that blurs the physical gradient, and the response is to flag such tets at mesh-build time and optionally [adaptively refine](../03-adaptive-refine.md) them. This leaf writes out the composition rules, the flagging criterion, and the handshake with [Part 3 Ch 03 multi-material interfaces](../../30-discretization/03-interfaces.md).

## Composition via SDF-distance-weighted blend

`cf-design` authors a graded material field by composing two (or more) scalar-parameter constants with a weight that is itself an SDF-derived field. The canonical pattern — stiff skin over soft core — looks like:

```text
stiffness_field(x) = w(phi_skin(x)) * mu_stiff + (1 - w(phi_skin(x))) * mu_soft
```

where `phi_skin` is the SDF for the skin's inner boundary (negative inside the skin, positive inside the core, zero at the boundary) and `w` is a smoothed step function (e.g., the classical $\operatorname{smoothstep}$ polynomial, or a Hermite sigmoid) that maps $\phi$ into $[0, 1]$ with transition-band half-width $w_\text{band}$. Outside the band ($|\phi| > w_\text{band}$), the step resolves cleanly to $0$ or $1$ and the blend snaps to one side's constant; inside the band, the result is a continuous interpolation. The step function is a blend-weight helper, distinct from the [smoothed-CSG operators from Ch 00 §01](../00-sdf-primitive/01-operations.md) which act on the SDFs themselves rather than on blend weights.

The composition uses scalar arithmetic on [`Field<Output = f64>`](../../20-materials/09-spatial-fields/00-sdf-valued.md) values — weighted sum with a smoothed-step weight — rather than the signed-distance-specific $\min / \max / \max(-\cdot, \cdot)$ operators from [Part 7 Ch 00 §01 SDF operations](../00-sdf-primitive/01-operations.md). The SDF operator algebra is for composing *geometric* SDFs; graded-material blending is a separate composition pattern that shares the authoring layer (`cf-design` exposes both) but not the operator set. No new machinery is needed in `sim-soft` to support graded-material composition; the per-sample `Field::sample` evaluation resolves the blend at mesh-build time.

## The interface-band flagging rule

When `sdf_bridge/` samples the `MaterialField` at each tet's evaluation point during [§00 sampling](00-sampling.md), it has access to the geometric SDFs that drove any blends (the designer authors them, [`cf-design`](../../110-crate/02-coupling/04-cf-design.md) passes them through). `sim-soft` flags a tet as an **interface tet** when:

$$ |\phi_\text{interface}(\mathbf{x}_c)| < L_e $$

where $\phi_\text{interface}$ is the SDF driving the material blend, $\mathbf{x}_c$ is the tet centroid, and $L_e$ is the tet's representative edge length (arithmetic mean of its six edge lengths — local notation, confined to this leaf). Equivalently: a tet is an interface tet when the nearest material-interface boundary is within one edge length of its centroid.

The check is cheap — one SDF evaluation per tet, reusing the [§00 sampling pass](00-sampling.md) — and produces a boolean flag in the per-tet material cache. The flag is not itself a trigger; it is metadata the downstream passes read.

## What the flag drives

The interface-tet flag is consumed by two downstream concerns:

- **[Adaptive h-refinement](../03-adaptive-refine.md).** When the refinement criterion [$r_e$](../03-adaptive-refine.md) evaluates the stress gradient across tet edges, an interface tet's additional refinement score is added — Ch 03's stress-based $r_e$ captures the fast-varying stress from material discontinuity, but the per-tet interface flag lets the refinement pass weight interface tets more aggressively when the ambient stress gradient is borderline. This is an opt-in Phase-H behavior per [Ch 03 Claim 2](../03-adaptive-refine.md).
- **Diagnostic reporting.** During [Part 11 Ch 04 testing](../../110-crate/04-testing.md), the number and distribution of interface tets is part of the mesh-quality diagnostic alongside the [Part 3 Ch 01 aspect-ratio and dihedral-angle statistics](../../30-discretization/01-mesh-quality.md). A mesh with many interface tets indicates the designer's transition-band half-width $w_\text{band}$ is tight relative to the mesh resolution; widening $w_\text{band}$ or increasing `resolution_hint` both reduce the count.

## Alternatives not pursued

- **Per-Gauss-point composition on Tet10.** Phase H Tet10 samples the `MaterialField` per Gauss point per [§00](00-sampling.md), which already captures within-element material gradients to quadrature-integration accuracy; no additional composition machinery is needed at the element level.
- **Mesh-interface-alignment as a required rule.** [Part 3 Ch 03 §00 bonded interfaces](../../30-discretization/03-interfaces/00-bonded.md) requires element edges to lie on *topological* material interfaces (where the material type changes discretely); the graded-blend case (where the material is continuous and only its value varies) does not require mesh alignment. Conflating the two would over-constrain the mesher; the interface-band flag is specifically for graded transitions that do not need alignment.
- **Runtime composition resolution.** The composition resolves at `Field::sample` evaluation time — once per mesh-build, not per-Newton-iteration. The Newton hot path sees the per-tet scalar parameters from the cache, not the composition tree.

## Interaction with material-only edits

A design edit that changes the blend weight or the SDF driving the blend (but leaves the geometric boundary unchanged) is a [material-changing edit](../04-live-remesh/00-change-detection.md) in [Ch 04](../04-live-remesh.md)'s classification. The interface-tet flag is re-computed during the re-sampling pass, alongside the material-parameter re-evaluation. A tet that was an interface tet before the edit may no longer be one (if the designer widened the blend band past the tet's centroid), and vice versa. The Hessian cache invalidation [Ch 04 §01 warm-start](../04-live-remesh/01-warm-start.md) performs for material-changing edits already covers the downstream consequence.

## What this sub-leaf commits the book to

- **Graded material composition is scalar arithmetic on `Field<Output = f64>` values, not SDF operator algebra.** Weighted sum with a smoothed-step weight (smoothstep / Hermite sigmoid), distinct from the $\min / \max$ CSG operators [Ch 00 §01](../00-sdf-primitive/01-operations.md) defines for geometric SDFs. The two composition patterns share the [`cf-design`](../../110-crate/02-coupling/04-cf-design.md) authoring layer but not the operator set.
- **Interface tets are flagged at mesh-build time.** A tet whose centroid is within one edge length of a material-interface SDF is flagged; the check is cheap and runs inside the [§00 sampling pass](00-sampling.md).
- **The flag drives adaptive refinement and diagnostics, not the solver directly.** [Ch 03 adaptive-refine](../03-adaptive-refine.md) may weight interface tets more aggressively; [Part 11 Ch 04 testing](../../110-crate/04-testing.md) reports the count. The Newton loop does not branch on the flag.
- **Graded transitions are distinct from topological multi-material interfaces.** [Part 3 Ch 03](../../30-discretization/03-interfaces.md) requires mesh-alignment for topological interfaces; graded transitions do not require it. The two handling paths are deliberately separate.
- **Composition resolves at mesh-build time.** The Newton hot path sees per-tet scalar parameters from the cache, not the composition tree.
