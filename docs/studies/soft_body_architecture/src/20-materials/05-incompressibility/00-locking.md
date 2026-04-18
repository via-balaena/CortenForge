# Volumetric locking

The [Ch 05 parent](../05-incompressibility.md) named four treatments for near-incompressibility and claimed that naively setting $\lambda \gg \mu$ in a standard Tet4 discretization produces volumetric locking rather than near-incompressible behavior. This leaf makes the claim concrete: what locking is mechanically, why Tet4 specifically locks, what the locked solver returns, and what the [mixed u-p](01-mixed-up.md), [F-bar](02-f-bar.md), and [higher-order](03-higher-order.md) siblings each cure about it.

## The mechanism

The Lamé pair at $\nu \to 0.5$ has $\lambda = E\nu / [(1+\nu)(1 - 2\nu)]$ diverging while $\mu = E / 2(1 + \nu)$ stays bounded. For silicone at $\nu = 0.499$, $\lambda / \mu \approx 500$ — the volumetric-penalty stiffness dominates the deviatoric-shear stiffness by three orders of magnitude.

A [Tet4 element](../../30-discretization/00-element-choice/00-tet4.md) has four nodes, twelve displacement degrees of freedom, and one Gauss point per element (centroid integration is exact for the linear strain $\varepsilon$). Under a single Gauss-point rule, the volumetric strain $\mathrm{tr}\,\varepsilon$ is a single scalar per tet; the deviatoric strain has five independent components (a symmetric $3\times 3$ tensor minus the trace).

The per-element energy is approximately $\tfrac{1}{2}\, \mu\,\|\mathrm{dev}\,\varepsilon\|^2 + \tfrac{1}{2}\,\lambda\,(\mathrm{tr}\,\varepsilon)^2$ per Gauss point (linear-elastic expansion for the purpose of the locking argument; the hyperelastic energies reduce to this at small strain per [Ch 02](../02-linear.md)). When $\lambda / \mu$ is large, the solver minimizes the total energy by driving $\mathrm{tr}\,\varepsilon$ toward zero in every tet first, and only then relaxing the deviatoric modes.

The twelve-DOF Tet4 cannot, in general, configure itself so that $\mathrm{tr}\,\varepsilon = 0$ holds per tet while the deviatoric modes match an arbitrary boundary condition. The per-tet incompressibility constraint subtracts one equation per tet from the twelve-DOF space; for a globally incompressible motion, the number of per-tet constraints ($n_\text{tets}$) exceeds the number of displacement DOFs ($3\,n_\text{vertices}$) — typical 3D Delaunay meshes have $n_\text{tets} \approx 6\,n_\text{vertices}$, so constraints run at roughly twice the DOF count — and the linear system is over-constrained. The solver then cannot simultaneously satisfy all the volumetric constraints and the deviatoric-mode boundary conditions, and the minimizer is a configuration where neither set is fully satisfied — specifically, the deviatoric modes (the bending, the isochoric shape changes) are suppressed to keep the dominant volumetric constraints nearly met.

The mechanical signature: the mesh behaves as if it were much stiffer in isochoric-bending modes than the underlying material actually is. A cantilevered Tet4 beam with $\nu \to 0.5$ displaces approximately two orders of magnitude less than the analytical Euler–Bernoulli bending solution predicts — a canonical demonstration the [parent chapter](../05-incompressibility.md) names, and the one the [Ch 05 parent claims' "100×"](../05-incompressibility.md) number refers to.

## What the locked solver returns

Three observable failures that the [volume-loss failure in Part 1 Ch 02](../../10-physical/02-what-goes-wrong/00-volume-loss.md) catalogs under real canonical-problem scenes:

- **Rigid-looking bending.** A sleeve compressed onto a probe does not bulge sideways as an incompressible silicone would; it appears rigid in every mode that requires isochoric bending.
- **Visible mesh artifacts.** The suppression of deviatoric modes is not uniform across the mesh — it depends on the per-element constraint-versus-DOF balance, so the locking degree varies with mesh topology. The simulated deformation shows element-shape-dependent stiffness, visible as faceting and checkerboard patterns that a uniform material should not produce.
- **Volume loss (not volume conservation!).** Paradoxically, the locked mesh's reported $\det F$ values still drift from 1 because the Newton solver trades off the per-tet volumetric constraint against the deviatoric residual, and the trade-off is not tight. The [Part 1 Ch 02 volume-loss failure mode](../../10-physical/02-what-goes-wrong/00-volume-loss.md) is specifically this: the mesh is both unable to bend AND unable to conserve volume, because it cannot simultaneously satisfy the over-constrained system.

A naive user running a Tet4 simulation with $\nu = 0.49$ and a hyperelastic `Material` sees these failures and concludes that the hyperelastic model is wrong; the real problem is the discretization. [Ch 04's](../04-hyperelastic.md) closed-form energies are correct at the continuum level and match [linear elasticity](../02-linear.md) at small strain; the locking is a Tet4-plus-near-incompressibility interaction, not a constitutive-law bug.

## Why the parent chooses to fix it at the discretization layer

Three alternatives exist in principle, only one of which `sim-soft` takes.

**Ignoring incompressibility.** Set $\nu = 0.3$ for silicone, get a solver that converges but produces a model that does not match the physical material. This is what cheap soft-body pipelines ship and the [Part 1 Ch 02 volume-loss failure](../../10-physical/02-what-goes-wrong/00-volume-loss.md) catalogs as a root cause. `sim-soft` rejects this: silicone is $\nu \approx 0.499$, not $0.3$.

**Using a different element type.** Tet10 (quadratic tetrahedra) has more nodes per element and can satisfy the incompressibility constraint without locking; Hex8 can too, with a different mesh-generation pipeline. This is the [higher-order sub-leaf's](03-higher-order.md) treatment. `sim-soft` does not adopt it as the primary cure because Tet10 is substantially more expensive per element than Tet4-plus-a-cure and Hex8-from-SDF is unsolved ([Part 3 Ch 00](../../30-discretization/00-element-choice.md)).

**Augmenting the formulation at Tet4.** Replace the per-element energy with a mixed formulation — [mixed u-p](01-mixed-up.md), where pressure is an independent unknown — or a modified-kinematic formulation — [F-bar](02-f-bar.md), where the deformation gradient's volumetric part is element-averaged. Both keep Tet4 as the element type, cure locking, and stay within `sim-soft`'s [SDF → Tet4 pipeline](../../70-sdf-pipeline/01-tet-strategies.md) from [Part 7](../../70-sdf-pipeline/00-sdf-primitive.md). This is the primary cure.

## What this sub-leaf commits the book to

- **Locking is a discretization-level pathology, not a constitutive-law pathology.** The hyperelastic energies of [Ch 04](../04-hyperelastic.md) are correct; the Tet4-plus-near-incompressibility interaction is what produces the "rigid-looking bending" failure. Diagnostics should point at the element-plus-quadrature combination rather than at the material.
- **The locked configuration is both rigid and volume-non-conserving.** The over-constrained system cannot satisfy deviatoric and volumetric requirements simultaneously; the Newton solver's tradeoff produces a minimizer that fails both. The [volume-loss failure](../../10-physical/02-what-goes-wrong/00-volume-loss.md) is the canonical scene-level manifestation.
- **`sim-soft` cures locking at the formulation layer, not by switching element type.** Mixed u-p is the primary cure, F-bar is the fast path for moderate near-incompressibility, and higher-order elements are the last-resort alternative. The choice stays on Tet4 because the [SDF → Tet4 pipeline](../../70-sdf-pipeline/01-tet-strategies.md) is load-bearing for the [design-vs-experience dual-mode commitment](../../10-physical/03-thesis.md).
- **Naive $\nu = 0.3$ is explicitly rejected.** Setting a compressible Poisson ratio to avoid locking produces a model that does not match silicone; the [Part 1 Ch 02 volume-loss failure](../../10-physical/02-what-goes-wrong/00-volume-loss.md) catalogs the consequence. `sim-soft` surfaces a warning when a user constructs a hyperelastic `Material` at $\nu < 0.45$ for silicone-class material data, naming the discretization cure the user should reach for instead.
