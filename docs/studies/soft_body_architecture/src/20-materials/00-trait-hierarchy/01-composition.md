# Composition rules

The [trait-surface sibling](00-trait-surface.md) pinned down the four-item `Material` trait. Three features of real constitutive laws — viscoelastic rate dependence, anisotropic fiber reinforcement, thermal coupling to `sim-thermostat`'s temperature field — are richer than the base trait can express in closed form at a single material point. This leaf writes down how each one composes over the base trait rather than growing it.

## Viscoelasticity as `Viscoelastic<M: Material>`

`Viscoelastic<M>` wraps any rate-independent base material and attaches a [Prony-series](../07-viscoelastic/00-prony.md) relaxation spectrum to it. The decorator carries:

```rust
pub struct Viscoelastic<M: Material> {
    base: M,
    prony: Vec<(f64 /* tau_i */, f64 /* g_i */)>,  // relaxation times and weights
}
```

The per-element hereditary state — one $3\times 3$ tensor per Prony mode per tet — lives in `element/`'s per-tet storage, not in the `Viscoelastic<M>` material instance. The material carries only the Prony parameters $(\tau_i, g_i)$, which are uniform across every element assigned to it. At every timestep the [Part 5 Ch 00 backward-Euler loop](../../50-time-integration/00-backward-euler.md) calls a history-advance method that evolves the hereditary state under the exponential-relaxation ODE and enters the summed hereditary contribution into the step's residual at assembly time.

The base `Material` trait itself stays pure in $F$. The viscoelastic correction enters the residual additively during assembly, not inside `first_piola`. This is the cleanest separation between state-free constitutive closed-form and history-bearing time integration; it keeps the base trait serializable and `Send + Sync`, and it keeps the [GPU autograd tape](../../80-gpu/03-gpu-autograd.md) pure in per-step inputs.

## Anisotropy as `Anisotropic<M: Material, R: FiberDirection>`

Anisotropic laws — [Holzapfel-Gasser-Ogden](../06-anisotropic/01-hgo.md) and its fiber-reinforced variants — need a preferred direction $a$ at each Gauss point in addition to the deformation gradient $F$. The decorator wraps any isotropic base with a fiber-direction resolver and adds an $I_4 = a \cdot C\, a$-dependent fiber-strain term to the base's energy:

```rust
pub struct Anisotropic<M: Material, R: FiberDirection> {
    base: M,
    k1: f64, k2: f64,        // HGO fiber-stiffening parameters
    fiber: R,                // resolver: position -> unit vector a
}

impl<M: Material, R: FiberDirection> Material for Anisotropic<M, R> { /* ... */ }

// HGO on silicone = Anisotropic<NeoHookean, _>:
type Hgo<R> = Anisotropic<NeoHookean, R>;
```

The HGO energy $\psi = \psi_\text{iso}(I_1) + \tfrac{k_1}{2 k_2}(e^{k_2 (I_4 - 1)^2} - 1)$ decomposes additively, so the decorator adds the $I_4$ term to the base's isotropic energy without blending — symmetric with how `Viscoelastic<M>` adds hereditary stress and `Thermal<M>` factors the deformation gradient. The fiber resolver `R` is a generic type parameter on the decorator, not on the base trait; the [direction-fields leaf](../06-anisotropic/02-direction-fields.md) enumerates what resolvers `sim-soft` ships (uniform, axisymmetric, SDF-valued from `cf-design`). This is what lets a mesh mix isotropic Ecoflex regions with anisotropic fiber-reinforced Dragon Skin regions in one [spatial material field](../09-spatial-fields.md) without branching in `element/`.

## Thermal coupling as `Thermal<M: Material>`

`Thermal<M>` is the thermal-coupling decorator:

```rust
pub struct Thermal<M: Material> {
    base: M,
    modulus_curve: Box<dyn Fn(f64 /* T */) -> (f64 /* mu */, f64 /* lambda */)>,
    expansion: f64,                        // alpha_thermal
    thermostat: Handle<ThermostatField>,   // ref to sim-thermostat's T field
}
```

The handle to `sim-thermostat`'s temperature field is what the decorator queries at each Gauss point to adjust the base material's Lamé pair and to factor the deformation gradient as $F = F_e\, F_\theta$ — an elastic part $F_e$ passed to `base.first_piola` and a thermal-expansion part $F_\theta = (1 + \alpha(T - T_0))\, I$ that the decorator applies before and unwinds after the base call. [Part 2 Ch 08 — thermal coupling](../08-thermal-coupling.md) covers the temperature-dependence curves; the coupling boundary with `sim-thermostat` is named in [Part 11 Ch 02](../../110-crate/02-coupling/01-thermostat.md).

Thermal coupling does not need per-element history — the temperature is a field the thermostat owns, not a history the material accumulates — so `Thermal<M>` is a pure decorator whose `first_piola` is a function of $F$ alone given the current thermostat read. Dissipative heating is the reverse channel — stress work the material emits back into the thermostat at each step — and is handled at the coupling boundary in [Part 2 Ch 08 dissipation](../08-thermal-coupling/02-dissipation.md), not inside `Thermal<M>`.

## Composition order is locked

The three decorators compose. The canonical order from outside to inside is `Thermal<Viscoelastic<Hgo<R>>>`: temperature-adjusts the Lamé pair, then applies hereditary Prony stress, then the anisotropic HGO energy. Reversing the order would let thermal expansion feed back into the Prony relaxation times in a way the book does not commit to, and would re-orient the fiber direction under thermal expansion in a way that contradicts the "fiber direction is fixed in the reference configuration" convention of [Ch 06](../06-anisotropic.md). The order is locked by the `Material::compose` builder in [`material/`](../../110-crate/00-module-layout/00-material.md)'s public API; stacking decorators in arbitrary orders is rejected at construction.

## What this sub-leaf commits the book to

- **Every feature beyond rate-independent instantaneous elasticity is a composition over the base trait.** No separate `RateDependentMaterial` or `ThermalMaterial` trait. Compositions nest; the outer trait remains `Material`.
- **Per-element history lives in `element/`, not on the `Material` trait.** The base trait stays pure in $F$; history-bearing behavior enters the residual at assembly time. This is what keeps the trait `Send + Sync` and keeps the GPU-autograd-tape playback path pure in per-step inputs.
- **Composition order is not a user choice.** The `Material::compose` builder enforces `Thermal<Viscoelastic<Anisotropic>>` outside-in as the canonical stack; other orders are rejected at construction so that decorator interactions (thermal expansion on relaxation times, thermal expansion on fiber orientation) stay in one documented combination rather than a combinatorial matrix.
