# F-bar method

The [mixed u-p sibling](01-mixed-up.md) cures locking via a second field and a saddle-point solve. F-bar cures it more cheaply: no extra unknowns, no saddle-point, no Schur complement — just a modification to the kinematic input that the constitutive law sees. The [Ch 05 parent](../05-incompressibility.md) calls this the "cheap Tet4 hack" and commits to shipping it as a fast path for moderate near-incompressibility ($\nu \leq 0.49$). This leaf writes the modification down and names the regime where it degenerates.

## Replacing the volumetric part of $F$

Decompose $F$ into its isochoric and volumetric parts as in [Ch 04's modified-form leaves](../04-hyperelastic/00-neo-hookean/02-incompressible.md): $F = \bar F \cdot J^{1/3}\, I$ with $\bar F = J^{-1/3}\, F$ and $\det \bar F = 1$. The deviatoric behavior of a hyperelastic law depends on $\bar F$ alone; the volumetric behavior on $J$ alone.

F-bar replaces the per-Gauss-point $J$ in the volumetric term with an *averaged* volume ratio $\bar J$ — averaged over a patch of neighboring Gauss points (or neighboring elements, for Tet4 where each element contributes one Gauss point). The kinematic input the constitutive law receives becomes:

$$ F^\star \;=\; \bar F \cdot \bar J^{1/3}\, I \;=\; \left(\frac{\bar J}{J}\right)^{1/3} F $$

The isochoric factor $\bar F$ is unchanged — deviatoric behavior is evaluated on the same per-Gauss-point kinematic the unmodified formulation would use. Only the volumetric scalar that feeds the volumetric-energy term is replaced.

$\bar J$ is an average: a simple choice is the arithmetic mean of $J$ over a patch of Gauss points adjacent to the current one. For Tet4 with one Gauss point per element, the patch is a set of neighboring tets — typically the star of tets around a shared node, or a cluster of topologically-adjacent tets. The exact averaging weights and patch definition depend on the variant; the shared structural feature is that $\bar J$ is a coarser-scale volume ratio than $J$.

## Why the patch average cures locking

The over-constraint in the single-field Tet4 formulation was "per-tet $J \approx 1$" as a constraint. Replacing the volumetric term's input with a *patch-averaged* $\bar J$ softens this to "patch-averaged $\bar J \approx 1$" — one constraint per patch rather than one per tet. The constraint count drops by the patch size, and the twelve-DOF Tet4 element has room to satisfy both the relaxed volumetric constraint and the deviatoric-mode boundary conditions.

The trade: individual tets can now have $\det F \neq 1$ locally, as long as the patch-averaged volumetric strain is zero. Physically this is a relaxation — the incompressibility constraint is no longer enforced per-element but rather in an averaged sense — and the accuracy loss is proportional to the within-patch variation of $J$. For moderate near-incompressibility ($\nu \leq 0.49$) on a well-refined mesh where the within-patch $J$ variation is small, the approximation is acceptable. For $\nu \to 0.499$ on coarse meshes where the within-patch variation dominates, the approximation degrades and mixed u-p becomes the better choice.

## Modified tangent and the same block structure

The F-bar modification changes the kinematic passed to the constitutive law; it does not change the number of unknowns or the block structure of the Newton tangent. The per-element stiffness block is still $12 \times 12$ (Tet4), the sparse CSR pattern is still determined by Tet4 topology, and the Cholesky factorization path is still [the factor-on-tape of Part 5 Ch 00](../../50-time-integration/00-backward-euler.md) — no saddle-point, no Schur complement.

The per-tet tangent is modified because $F^\star$ depends on $\bar J$, which in turn depends on neighboring tets' $J$ values. The chain rule picks up a coupling: $\partial F^\star / \partial u$ has a neighbor-contribution term that the unmodified Tet4 tangent does not. This extends the nonzero pattern of the stiffness matrix slightly beyond the Tet4-only adjacency (each tet's tangent now couples to its patch-neighboring tets), but the extension is bounded and pre-allocatable per the patch's connectivity. The [GPU autograd tape](../../80-gpu/03-gpu-autograd.md) carries the extended pattern along with the base Tet4 pattern; the CSR pre-allocation happens once per mesh.

## `Material`-agnostic wrapping

Like [mixed u-p](01-mixed-up.md), the F-bar modification is constitutive-law-agnostic. Any hyperelastic base law wrapped in `FBar<M>` uses the modified input $F^\star$ at its `first_piola` and `tangent` calls, and the impl itself needs no change — the modified kinematic flows through the existing trait surface.

```rust
pub struct FBar<M: Material> {
    base: M,
    patch: PatchScheme,  // how to average J over neighboring Gauss points
}

impl<M: Material> Material for FBar<M> { /* ... */ }
```

One difference from mixed u-p: F-bar needs neighbor information to compute $\bar J$, so the decorator's evaluation is not a per-Gauss-point operation but a per-patch operation. The [`element/`](../../110-crate/00-module-layout/01-element.md) layer's assembly loop runs a patch-averaging pass to compute $\bar J$ values once per Newton step, then the per-Gauss-point constitutive evaluation uses the cached $\bar J$ from the patch the Gauss point belongs to.

## Validity-domain widening and the $\nu \leq 0.49$ ceiling

Wrapping a base material in `FBar<M>` widens the `ValidityDomain`'s Poisson upper bound from $\nu < 0.45$ (the base law's standalone cap) to $\nu \leq 0.49$ — not to $\nu < 0.499$. The [Ch 00 validity descriptor](../00-trait-hierarchy/02-validity.md)'s composition-widening pattern reports the tightened upper bound; a user constructing `FBar<NeoHookean>` and trying to use it at $\nu = 0.499$ gets a runtime warning.

The ceiling is not arbitrary. At $\nu > 0.49$ the within-patch $J$ variation approaches the scale where patch-averaging is no longer an accurate approximation: the difference between "per-element $J$" and "patch-averaged $\bar J$" becomes comparable to the physical volumetric strains the material is undergoing, and the F-bar kinematic $F^\star$ differs measurably from the true $F$ in a way that corrupts the deviatoric evaluation. Above $\nu = 0.49$ the [mixed-u-p sibling](01-mixed-up.md) is the recommended path; F-bar is the cheaper alternative only within its declared regime.

## When to choose F-bar over mixed u-p

Two scenarios argue for F-bar.

- **Scenes that stay below $\nu = 0.49$.** Parts of the canonical problem sit at stiffer material grades (Dragon Skin 30A, carbon-black-loaded composites above percolation) where Poisson ratio is meaningfully below silicone's $\nu \approx 0.499$. For these regions F-bar's cheaper solver matches the accuracy of mixed u-p without the extra pressure field.
- **Phase-E real-time budgets where the saddle-point solve is the bottleneck.** The reduced-system Cholesky path in mixed u-p is SPD and efficient, but the Schur complement's coupling is a per-step setup cost. F-bar has no saddle-point and no Schur complement — its per-step Newton cost is closer to the unmodified Tet4 cost. For real-time scenes where the near-incompressibility regime is moderate, F-bar saves the difference.

Multi-material meshes can mix `FBar<M>` and `MixedUP<M>` regions per the [spatial material field machinery](../09-spatial-fields.md); the solver assembles heterogeneous wrapping without branching, because both decorators present as `Material` on the trait surface. A scene with a soft-silicone inner layer at $\nu = 0.499$ and a stiffer composite outer layer at $\nu = 0.48$ can use mixed u-p inside and F-bar outside.

## What this sub-leaf commits the book to

- **F-bar modifies the kinematic, not the formulation.** No extra unknowns, no saddle-point — just $\bar J$ (patch-averaged volume ratio) replacing per-Gauss-point $J$ in the volumetric term's input. The `Material` trait surface is unchanged; only the decorator's wrapping logic changes what $F$ the base impl sees.
- **The cure works by coarsening the constraint scale.** Per-element $J \approx 1$ becomes per-patch $\bar J \approx 1$; the over-constraint relaxes and the Tet4 deviatoric modes recover. The accuracy trade-off is bounded by within-patch $J$ variation.
- **Validity widens to $\nu \leq 0.49$ only.** Above that, mixed u-p is the recommended cure; the composed `ValidityDomain` surfaces the ceiling so the user does not silently ship an F-bar configuration outside its accuracy regime.
- **Per-element block size and CSR pattern are essentially unchanged.** Patch-neighbor coupling extends the nonzero pattern slightly but bounded-ly; pre-allocation and the [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) carry over without modification.
- **F-bar and mixed u-p are interchangeable per region in multi-material meshes.** Both decorators wrap `M: Material` and present as `Material`; the [spatial material field](../09-spatial-fields.md) assembles heterogeneous wrappings without branching in `element/`.
