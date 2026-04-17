# Volume loss under squeeze

A silicone cavity squeezed over a rigid probe should bulge — incompressible material displaces outward as radial compression takes the material's inside down to the probe's surface. A cavity that instead *shrinks* visually under the squeeze is the failure this leaf names. The visual signature is a sleeve that looks deflated while the probe is inserted. The scientific signature is an effective Poisson ratio that has collapsed at operating strain. The two framings describe the same defect.

## What fails, and where it shows up

Three off-the-shelf solver classes reproduce this failure on the [canonical problem](../00-canonical/00-formulation.md) within an afternoon of running.

**Linear FEM.** The linearized strain measure $\varepsilon = \tfrac{1}{2}(\nabla u + (\nabla u)^T)$ differs from the exact Green strain $\mathbf{E} = \tfrac{1}{2}(F^T F - I)$ by a term quadratic in $\nabla u$. At 50% stretch this omitted term is ≈17% of the strain; at 100% stretch, ≈30%. Under radial squeeze the solver predicts too little restoring stress and converges to a configuration that is too-far-compressed compared to the physical material ([Part 2 Ch 02](../../20-materials/02-linear.md) derives the numbers). Compounded across thousands of elements, the sleeve appears to have lost bulk volume.

**XPBD and mass-spring networks.** No finite-strain constitutive law at all. Volume is preserved — if at all — by explicit volumetric constraints on each tet, which are rate-limited and converge slowly; under interference-fit squeeze they saturate and the material passes through itself. Visually: interior of the cavity deflates, exterior bulges unpredictably or not at all.

**Penalty contact with reduced Poisson ratio.** Cheap pipelines ship $\nu = 0.3$ or $\nu = 0.4$ to avoid the [volumetric-locking](../../20-materials/05-incompressibility/00-locking.md) issue that a displacement-only formulation produces at $\nu \to 0.5$. The model is now fundamentally compressible; under squeeze the material compresses rather than displaces, and the cavity loses volume to match.

## Which reward terms it corrupts

- **Effective stiffness.** Lost volume appears at the solver as reduced resistance to squeeze — the transmitted axial force is too low for the displacement. The [stiffness-bound barrier](../01-reward/03-stiffness-bounds.md) fires at configurations the physical material would pass; the optimizer learns to prefer stiffer cavity designs than the problem actually requires.
- **Peak pressure.** Volume loss is spatially non-uniform — thin regions lose proportionally more volume than thick regions — so the pressure distribution shifts toward whichever region retains the most material. Peaks appear where the mesh is fine and troughs where it is coarse. [Peak-bound barriers](../01-reward/02-peak-bounds.md) fire inconsistently across mesh densities.
- **Pressure uniformity.** Indirectly — redistributed pressure from volume loss is not uniform. The [uniformity term](../01-reward/00-pressure-uniformity.md) reports non-physical spread.

## How sim-soft fixes it

Three commitments together.

- **[Neo-Hookean or Ogden hyperelastic constitutive law](../../20-materials/04-hyperelastic.md).** The hyperelastic energy density $\Psi(F)$ is written directly in the deformation gradient, so finite-strain kinematics are represented exactly and the 17%/30% linearization error vanishes.
- **[Near-incompressibility via mixed u-p formulation](../../20-materials/05-incompressibility/01-mixed-up.md).** The volumetric term is handled by an auxiliary pressure field that relaxes the displacement-only formulation's locking behavior at $\nu \to 0.5$, so the material runs at its real $\nu \approx 0.499$ without mesh locking.
- **[Poisson ratio from measurement](../04-material-data/00-ecoflex/00-mechanical.md).** The `MaterialField` carries $\nu \approx 0.499$ for every default silicone in the material database — not 0.3, not 0.4. Volume loss is not a free parameter to be traded for solver convergence; the material-database row is the contract with the physical silicone.

With these three in place, the canonical problem reproduces the real cavity's sideways bulging under squeeze, the effective stiffness matches what a force-displacement measurement on the printed part would show, and the reward terms fire only on configurations the physical material actually disprefers.

## Why the obvious cheap fix doesn't work

**"Just use a stiffer Poisson ratio."** Setting $\nu = 0.495$ in a linear-elastic formulation without mixed u-p approximates incompressibility but triggers [volumetric locking](../../20-materials/05-incompressibility/00-locking.md) on the tet mesh — the elements become too rigid in bending, the solver mis-predicts rim deformation ([see rim failure](04-rim.md)), and the fix trades one failure for another. The mixed u-p formulation is what lets $\nu \to 0.5$ without locking.

**"Scale the linear FEM's stiffness to match measured finite-strain data."** Tuning $E$ so the linear-elastic prediction matches measured force at one operating strain produces a single-point fit that fails at other strains — the linearization error is quadratic in strain, not constant. The result is a solver that agrees with one measurement and disagrees with every other one. Hyperelasticity is the compact fix.
