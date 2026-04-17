# Constitutive trait hierarchy

This chapter names the trait surface that the `material/` module exposes to the rest of `sim-soft`. The design goal is a surface thin enough to fit on one page and expressive enough that every constitutive law in Part 2 — linear through Ogden-plus-HGO-plus-Prony-plus-thermal — is a straight composition over it. The three sub-chapters below walk the trait surface itself, the composition rules that build every law in the book from it, and the validity regimes each law is honest about.

| Section | What it covers |
|---|---|
| [The core trait surface](00-trait-hierarchy/00-trait-surface.md) | `Material` trait — `energy(F) → ψ`, `first_piola(F) → P`, `tangent(F) → ∂P/∂F` as associated items; derived scalars (volumetric strain, von Mises) via blanket default methods |
| [Composition rules](00-trait-hierarchy/01-composition.md) | Viscoelasticity as `Viscoelastic<M: Material>` decorator; anisotropy as `Material` + `FiberDirection` associated type; thermal coupling as `Thermal<M: Material>` decorator reading from `sim-thermostat` |
| [Validity regimes and error bounds](00-trait-hierarchy/02-validity.md) | Each `Material` impl declares its validity domain — strain range, temperature range, rate regime — and a gradcheck-verified error bound outside that domain |

Two claims Ch 00 rests on:

1. **The trait surface is thin, the compositions are rich.** `Material` itself exposes four items — energy, first Piola stress, tangent, and a validity domain descriptor. Every other constitutive feature (viscoelastic history, thermal dependence, anisotropic fibers, spatial material fields) is a composition wrapping a base `Material` rather than a new trait. This keeps the hot path in [`element/`](../110-crate/00-module-layout/01-element.md) generic in one trait, not a combinatorial matrix of feature traits.
2. **Validity is declared, not hoped for.** Every `Material` impl names its validity domain in code, and the [gradcheck suite](../110-crate/04-testing/03-gradcheck.md) verifies the error bound outside it. A neo-Hookean implementation that claims validity up to 300% stretch but diverges at 150% fails the validity-declaration test, not a softer quality gate. This is how "A-grade or it doesn't ship" translates to constitutive laws.
