# Material parameter poverty

A silicone compliant cavity in the wild is not a uniform blob. Shore-graded stiffness varies across the wall to distribute pressure; fiber reinforcement is threaded through high-strain bands to control anisotropic stretch; carbon-black-loaded layers handle sensing and Joule heating; the viscoelastic spectrum depends on the base silicone and the filler loading. A simulator that represents the cavity as a single Young's modulus — no spatial variation, no fiber direction, no viscoelasticity, no conductivity — is representing a material that does not exist for any real engineered part. The visual signature is every soft object having the same plasticky feel. The scientific signature is a `Material` trait with one parameter where the real material has dozens. Same defect, two framings.

## What fails, and where it shows up

Two off-the-shelf patterns reproduce this failure on the [canonical problem](../00-canonical/00-formulation.md) and more sharply on any domain problem from the [related-problems map](../00-canonical/01-related-problems.md).

**Uniform-stiffness soft bodies.** One Young's modulus over the whole cavity. The material is isotropic, rate-independent, and has no electromagnetic properties. This is the default in mass-spring networks, in most XPBD implementations, and in many FEM codes configured for simplicity. A designer who wants to explore a softer-rim / stiffer-base cavity must hand-edit stiffness per-element, which makes stiffness a discrete variable per element rather than a smooth spatial field — and which defeats any optimizer looking for a continuous design parameter.

**No viscoelasticity, no anisotropy, no coupled-physics coupling.** Constitutive laws that only describe elastic response. The [wobble-failure sibling](01-wobble.md) is the visible consequence for viscoelasticity; material poverty extends the same critique to anisotropy (fiber reinforcement, which [Part 2 Ch 06](../../20-materials/06-anisotropic.md) handles) and to coupled physics (thermal, electrical, which [Part 2 Ch 08](../../20-materials/08-thermal-coupling.md) and the [carbon-black composite family](../04-material-data/02-carbon-black.md) handle).

## Which reward terms it corrupts

- **Every reward term at the same time.** A cavity with the wrong material optimizes to a different cavity shape than one with the right material. The optimizer converges to an optimum that is correct for the model but wrong for the physical part. [Sim-to-real correction](../../100-optimization/05-sim-to-real.md) cannot recover because the missing parameters have no handle — there is nothing in the simulation for a residual GP to correct against.
- **Specifically, stiffness-vs-compliance trade-off at the rim.** A real soft-robotics cavity often runs Shore 10A at the rim and Shore 30A at the base, specifically to produce uniform pressure under non-uniform loading. A uniform-stiffness solver cannot find this solution because the solution is not in its search space. The [uniformity term](../01-reward/00-pressure-uniformity.md) is bounded away from its optimum under the uniform-stiffness assumption.
- **Anisotropy for fiber-reinforced variants.** Directional stiffness — a fiber-reinforced sleeve that is stiff axially and compliant radially — is a different design problem with a different optimum. The [Holzapfel-Gasser-Ogden anisotropic model](../../20-materials/06-anisotropic/01-hgo.md) handles it; an isotropic `Material` cannot.
- **Carbon-black composites.** Spatially-varying electrical conductivity $\sigma_e(x)$ enables embedded sensing (capacitive mode below the [percolation threshold](../04-material-data/02-carbon-black/03-percolation.md), resistive above). A material model with no electrical axis cannot represent these designs; the reward cannot even be defined if the design space includes electrical behavior.

## How sim-soft fixes it

Four commitments together.

- **[Material trait hierarchy](../../20-materials/00-trait-hierarchy.md).** A `Material` trait is the abstraction; `NeoHookean`, `Ogden`, `MooneyRivlin`, `HGO`, and composition combinators (`Layered`, `PronyWrapper`, `ThermalCoupled`) are implementations. The trait surface is the same regardless of which law is plugged in; the solver does not branch on the law, only on the trait's methods.
- **[SDF-valued spatial material fields](../../20-materials/09-spatial-fields.md).** `MaterialField` is a trait object over reference-space SDFs for each scalar material parameter — stiffness, fiber direction, Prony-series weight, Prony-series relaxation time, $\sigma_e$, thermal conductivity. Per-element sampling at meshing time assigns values; per-parameter gradient flow through SDF evaluation makes the spatial field optimizable. Shore-graded sleeves and fiber-reinforced liners become continuous design parameters, not discrete per-element edits.
- **[Prony-series viscoelasticity](../../20-materials/07-viscoelastic.md).** The rate-dependent response of each base silicone from [the viscoelastic-spectrum leaves](../04-material-data/00-ecoflex/02-viscoelastic.md) enters the constitutive law as a per-material Prony fit, calibrated from DMA data.
- **[Anisotropic and carbon-black material families](../04-material-data.md).** Part 1 Ch 04 commits concrete rows for Ecoflex (four grades), Dragon Skin (three grades), and carbon-black-loaded composites with electrical/thermal axes. Every row is a `Material` implementation that plugs into the trait hierarchy.

Together these four make the material a compositional design variable rather than a scalar. A cavity's material is an `SdfField`-plus-`MaterialField` pair, not a single number; the optimizer searches over spatial fields, not just over global scalars.

## Why the obvious cheap fix doesn't work

**"Tune a uniform stiffness to match measured force at the canonical operating point.** The fit is single-point: the same uniform stiffness cannot match measured force at other operating points because the response is not globally linear. A single calibration yields a model that agrees at one design and disagrees at every other, which defeats the point of optimization. Fixing the trait hierarchy is the expansion that lets calibration data land at multiple operating points simultaneously.

**"Use several uniform-stiffness regions (hand-partition the cavity).** Better than one uniform region, but the partition boundary is a discontinuity and the stiffness step is not differentiable. The optimizer cannot search over partition boundary locations with gradient information. SDF-valued spatial fields replace the partition-switch with a smooth stiffness field that continuous optimization can walk.

**"Skip anisotropy, skip fiber reinforcement; the canonical problem does not require it.** The canonical problem deliberately does not require fiber reinforcement, but the [related-problems map](../00-canonical/01-related-problems.md) shows domain problems that do. Shipping a solver that cannot support those problems means the book's single-stack thesis fails: a fiber-reinforced-catheter engineer would need a different simulator, which is exactly what the [Part 1 Ch 03 thesis](../03-thesis.md) argues against.
