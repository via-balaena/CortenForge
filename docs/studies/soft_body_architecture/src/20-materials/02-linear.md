# Linear elasticity — the baseline

Linear elasticity is the constitutive law every other law in Part 2 is measured against. It is the first law any FEM textbook teaches, the first law every FEM codebase implements, and — as [Part 1 Ch 02](../10-physical/02-what-goes-wrong.md) argued — the wrong law for the soft-body regime `sim-soft` targets. This chapter writes the law down explicitly, names the small-strain regime where it is correct, enumerates the specific failure modes outside that regime, and explains why it nonetheless ships in `sim-soft`'s `material/` module as a first-class `Material` implementation.

## The law

Linear elasticity makes three assumptions, each of which breaks at finite strain but is exact in the small-strain limit.

**Assumption 1 — infinitesimal strain.** The strain tensor is the symmetric gradient of the displacement field, with no higher-order terms:

$$ \varepsilon = \tfrac{1}{2}\left(\nabla u + (\nabla u)^T\right) $$

where $u = x - X$ is the displacement from reference to deformed. This is linear in $u$. The full nonlinear strain measure from [Ch 01](01-strain-measures.md) — Green strain $\mathbf{E} = \tfrac{1}{2}(F^T F - I)$ — equals $\varepsilon$ plus a term $\tfrac{1}{2}(\nabla u)^T (\nabla u)$ that linear elasticity drops. (Green strain is bolded $\mathbf{E}$ in this chapter's neighbourhood of Young's modulus $E$ to disambiguate; elsewhere the book follows the italic-$E$ convention disambiguated by context — see [`appendices/03-notation.md`](../appendices/03-notation.md).)

**Assumption 2 — linear stress–strain relation (Hooke's law).** Stress is a linear function of strain through the fourth-order elasticity tensor $\mathbb{C}$:

$$ \sigma = \mathbb{C} : \varepsilon $$

For isotropic materials — which is what `sim-soft`'s linear implementation supports — this collapses to two scalars, the Lamé parameters $\lambda$ and $\mu$:

$$ \sigma = \lambda\, (\mathrm{tr}\, \varepsilon)\, I + 2\mu\, \varepsilon $$

The Lamé parameters relate to Young's modulus $E$ and Poisson's ratio $\nu$ by $\mu = E / 2(1+\nu)$ and $\lambda = E\nu / (1+\nu)(1-2\nu)$. `sim-soft`'s `Material::new_linear` constructor takes $(E, \nu)$ and stores $\lambda, \mu$.

**Assumption 3 — no distinction between reference and deformed configuration.** The stress is expressed in the undeformed configuration; the elasticity tensor is constant in space and time; the equations of motion are solved on the reference mesh with the result reported as displacements from it. At finite strain, the stress measure matters (Cauchy vs. first Piola vs. second Piola), the elasticity tensor depends on the current configuration, and the reference mesh no longer matches the deformed geometry. Linear elasticity ignores all of this.

## What's right about it

**Exact in the small-strain limit.** For strains below ≈1%, the difference between $\varepsilon$ and $E$, and between the various stress measures, is negligible. At 0.1% strain on a steel part, linear elasticity and neo-Hookean agree to six digits. This is why structural-mechanics FEM — bridges, airframes, bolts — runs on linear elasticity with no loss of accuracy.

**The stiffness matrix is constant.** Because Assumption 2 is linear and Assumption 3 treats the reference geometry as fixed, the global stiffness matrix $K$ is assembled once and re-used every timestep. The solver loop reduces to $K\, u = f$ with a factorization that is computed once and back-substituted thereafter. This is ≈10× faster per timestep than Newton-on-hyperelastic, and for structural problems the speedup is free — the extra fidelity of hyperelastic would change no digits of the answer.

**Every FEM textbook and library starts here.** Linear elasticity is the common language. When the book cites FEBio, Abaqus, or SOFA and quotes numbers, the numbers are most often from a linear elastic simulation unless the paper specifies otherwise. Being able to reproduce a linear elastic result is the first interoperability check any new FEM implementation passes.

## What's wrong about it in the soft-body regime

**Geometric over-softening under stretch.** At 50% stretch ($\lambda = 1.5$), the linear-elasticity strain measure $\varepsilon$ underestimates the Green strain $E$ by ≈17%. The predicted stress is therefore 17% too low; the solver converges to a configuration that is 17% further stretched than the physical material would be. [Part 1 Ch 02's volume-loss failure](../10-physical/02-what-goes-wrong/00-volume-loss.md) is this error compounded over thousands of elements: the aggregate effect is a sleeve that appears to deflate under squeeze. The error grows as the square of strain — at 100% stretch the underestimate is ≈30%; at 200% stretch, ≈56%.

**Fictitious stress under pure rotation.** A rigid rotation $x = R X$ with $R \in SO(3)$ has $\nabla u \neq 0$ and therefore $\varepsilon \neq 0$, even though the material is not deformed at all. Linear elasticity predicts stress proportional to the rotation angle squared, purely from the rotation. A rotating soft cube under linear elasticity visibly inflates; this is the single clearest failure mode and the canonical motivation for [corotational elasticity](03-corotational.md). The fix is either to extract and divide out the rotation (corotational) or to use a rotation-invariant energy (hyperelastic).

**No volumetric incompressibility at finite strain.** Silicone's Poisson ratio at rest is ≈ 0.499, so $\lambda \gg \mu$. Linear elasticity with this ratio locks the finite-element mesh — the stiffness contribution from the volumetric term dominates the deviatoric and the tets become rigid in bending ([Part 2 Ch 05](05-incompressibility.md) covers the cure). Setting $\nu = 0.3$ to avoid locking gives the wrong material but preserves solver convergence; this is what cheap soft-body pipelines ship, and it produces a cube that visibly "deflates" under squeeze because the model is fundamentally compressible.

## Why sim-soft ships it anyway

Linear elasticity lives in `sim-soft`'s `material/` module as a first-class `Material` implementation for four reasons.

**Baseline for regression testing.** Every hyperelastic law must reduce to linear elasticity in the small-strain limit. A `Material` implementation of neo-Hookean that does not match the linear-elasticity reference to 6 digits on a 0.1%-strain test case has a bug. The [gradcheck suite](../110-crate/04-testing/03-gradcheck.md) includes a small-strain reduction test for every hyperelastic `Material` that uses the linear implementation as the oracle.

**Solver-plumbing prototype.** [Phase B](../110-crate/03-build-order.md) lands the Newton loop on elastic-only total potential energy. With a linear `Material`, the Newton loop converges in one iteration (because the energy is quadratic in $u$) and the solver-plumbing code — sparse assembly, [faer](https://github.com/sarah-quinones/faer-rs) factorization, gradient accumulation — can be debugged in isolation from constitutive-law complexity. Switching to neo-Hookean in the same Phase B is a type-parameter change, not a rewrite.

**Rigid backing.** Parts of the canonical problem ([Ch 00 in Part 1](../10-physical/00-canonical.md)) — the probe, mechanical fixtures — are modeled as rigid-elastic where deformation stays below 1%. Linear elasticity is correct in this regime and ≈10× cheaper than hyperelastic. Mixing `Material<Linear>` regions with `Material<NeoHookean>` regions in the same multi-material mesh ([Ch 09](09-spatial-fields.md)) is a composition, not a branch.

**Honest baseline for the Ch 02 failure catalog.** [Part 1 Ch 02's six failure modes](../10-physical/02-what-goes-wrong.md) are stated most sharply when each failure is reproducible. Shipping a linear `Material` and running the canonical problem against it makes the volume-loss failure, the fictitious-rotation-stress failure, and the locking failure all reproducible in `sim-soft` itself — so the book's claim that "hyperelastic fixes these" is testable rather than asserted. An honest baseline is worth more than an absent one.

## Validity domain declaration

Per [Ch 00](00-trait-hierarchy.md)'s validity-declaration requirement, the `Linear` `Material` impl declares its validity domain as: strain magnitude below 1%, rotation angle below 1°, Poisson ratio strictly below 0.45 (above which the locking failure dominates regardless of mesh). Outside this domain the gradcheck-verified error bound versus neo-Hookean is quadratic in strain and quadratic in rotation angle; `sim-soft` emits a runtime warning when a linear element exceeds its validity domain during a solve. The user can silence the warning for stress-testing, but the default is loud.
