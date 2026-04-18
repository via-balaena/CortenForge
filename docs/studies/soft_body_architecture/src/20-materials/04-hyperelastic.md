# Hyperelastic family

Hyperelasticity is `sim-soft`'s default constitutive family, per the [Part 1 Ch 03 thesis commitment](../10-physical/03-thesis.md). This chapter covers the three named hyperelastic laws `sim-soft` ships, each one an `impl Material` over the trait surface from [Ch 00](00-trait-hierarchy.md), composable into the same mesh via [spatial material fields](09-spatial-fields.md). The three are not in competition; they cover different strain regimes and different calibration-data situations, and the right choice depends on the [material data](../10-physical/04-material-data.md) available for the part being simulated.

| Section | Parameters | Best for |
|---|---|---|
| [Neo-Hookean](04-hyperelastic/00-neo-hookean.md) | $\mu$, $\lambda$ (or $E$, $\nu$) | The default. Closed-form energy $\psi = \tfrac{\mu}{2}(I_1 - 3) - \mu \ln J + \tfrac{\lambda}{2}(\ln J)^2$; fast tangent; matches silicone stress–strain curves up to ≈100% strain before saturating |
| [Mooney-Rivlin](04-hyperelastic/01-mooney-rivlin.md) | $C_{10}$, $C_{01}$ | When the neo-Hookean fit to measured silicone data is not tight enough in the 50–150% strain range. Two-parameter form $\psi = C_{10}(I_1 - 3) + C_{01}(I_2 - 3)$ broadens the curve-fit window |
| [Ogden](04-hyperelastic/02-ogden.md) | $\{(\mu_i, \alpha_i)\}_{i=1}^{N}$ (typically $N = 2$ or $3$) | High-strain silicone, strain-hardening, materials with non-monotonic stress–strain shapes. Dominates [Ecoflex](../10-physical/04-material-data/00-ecoflex.md) curve fits above 100% strain; most expensive to evaluate |

Two claims Ch 04 rests on:

1. **Hyperelasticity is one axis, not three laws.** All three share the `Material` trait; switching is a type-parameter change, not a rewrite. The downstream machinery — solver, contact, readout, autograd — has no opinion about which hyperelastic law is in use. This is load-bearing for [multi-material meshes](09-spatial-fields.md), where different regions of the same mesh use different hyperelastic laws (stiff Dragon Skin core plus soft Ecoflex shell, for instance) and the solver assembles them without branching.
2. **Neo-Hookean is the default because default ≠ best.** Neo-Hookean fits silicone well enough across the canonical problem's strain regime (most of which sits below 100% stretch) and is the cheapest to evaluate among the three. Mooney-Rivlin and Ogden are not upgrades to be switched on by default; they are brought in when the test case pushes into regions where neo-Hookean's curve saturates and the aggregate error exceeds what the [reward function](../10-physical/01-reward.md) can tolerate. This is a specific instance of [Ch 00's validity-domain declaration](00-trait-hierarchy.md) — the right law is the one whose validity domain contains the operating regime.
