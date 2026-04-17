# Holzapfel-Gasser-Ogden

The Holzapfel-Gasser-Ogden (HGO) law is the canonical specialization of [Ch 06's fiber-reinforced template](00-fiber-reinforced.md) to an exponential fiber-stiffening form. It dominates the regime where reinforcing fibers — collagen, textile yarns, embedded glass or carbon — exhibit J-shaped stress-stretch behaviour: low compliance at small strain, sharp stiffening at moderate-to-large strain. `sim-soft` ships HGO as the default `Anisotropic<M, R>` specialization, with `Hgo<R>` aliased to `Anisotropic<NeoHookean, R>` per the [Ch 00 composition leaf](../00-trait-hierarchy/01-composition.md).

## Energy form

The HGO fiber energy is exponential in the [fiber invariant $I_4$](../01-strain-measures/03-invariants.md):

$$ \psi_\text{fiber}(I_4) = \frac{k_1}{2 k_2}\left( e^{k_2 (I_4 - 1)^2} - 1 \right) $$

with two parameters:

- $k_1$ — fiber stiffness, units of stress (Pa or kPa)
- $k_2$ — dimensionless fiber-stiffening exponent

Stacked on a neo-Hookean base, the full HGO energy is:

$$ \psi(F, a) = \frac{\mu}{2}(I_1 - 3) - \mu \ln J + \frac{\lambda}{2}(\ln J)^2 + \frac{k_1}{2 k_2}\left( e^{k_2 (I_4 - 1)^2} - 1 \right) $$

The base contributes the [neo-Hookean energy](../04-hyperelastic/00-neo-hookean/00-energy.md) including its $-\mu \ln J$ stress-free corrector and its $(\lambda/2)(\ln J)^2$ volumetric term; the decorator adds $\psi_\text{fiber}$ alone. No fiber-side corrector is needed — the [fiber-reinforced sibling](00-fiber-reinforced.md)'s structural-vanishing argument shows $P_\text{fiber}|_{I_4 = 1} = 0$ automatically via the $(I_4 - 1)$ factor inside $\psi'_\text{fiber}$.

## Why exponential

Polynomial forms $\psi_\text{fiber} = c (I_4 - 1)^n$ stiffen too slowly to capture the J-shape: stress grows polynomially in fiber stretch, missing the sharp upturn real fibers exhibit once they re-orient and uncrimp. The exponential $e^{k_2 (I_4 - 1)^2}$ matches this regime — the slope $\partial \sigma_\text{fiber} / \partial \lambda_f$ grows exponentially in $I_4 - 1$, reproducing the J-shaped stress-stretch curve that is the diagnostic signature of fiber-reinforced soft materials.

The two parameters separate roles. At small fiber strain the exponential is approximately $1 + k_2 (I_4 - 1)^2$, so $\psi_\text{fiber} \approx (k_1 / 2)(I_4 - 1)^2$ and $k_1$ sets the small-strain stiffness scale. At larger strain the exponential takes over, and $k_2$ controls how fast the stiffening accelerates. Calibration to a measured stress-stretch curve fits $k_1$ from the small-strain slope and $k_2$ from the curvature of the J-shape.

## First Piola and tangent

From the [fiber-reinforced sibling](00-fiber-reinforced.md), $P_\text{fiber} = 2 \psi'_\text{fiber}(I_4)\, F\, a\, a^T$ with:

$$ \psi'_\text{fiber}(I_4) = k_1 (I_4 - 1)\, e^{k_2 (I_4 - 1)^2} $$

so:

$$ P_\text{fiber} = 2 k_1 (I_4 - 1)\, e^{k_2 (I_4 - 1)^2}\, F\, a\, a^T $$

vanishing at $I_4 = 1$. The fiber tangent involves $\psi''_\text{fiber}(I_4)$:

$$ \psi''_\text{fiber}(I_4) = k_1\, e^{k_2 (I_4 - 1)^2}\, \left( 1 + 2 k_2 (I_4 - 1)^2 \right) $$

which is everywhere positive: HGO is convex in $I_4$, and the fiber-tangent contribution is positive in the deformed fiber direction $F a$ across the full tension-only range $I_4 \geq 1$ — including at $I_4 = 1$ exactly, where $\psi'_\text{fiber}$ vanishes but $\psi''_\text{fiber}$ does not. The full per-element tangent is the sum of the base's isotropic tangent and the HGO fiber tangent; the [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) carries over without modification.

Tension-only is enforced by setting $\psi_\text{fiber} = 0$ for $I_4 < 1$ ([fiber-reinforced sibling](00-fiber-reinforced.md)'s switch-with-smoothing convention). Without the switch, $\psi_\text{fiber}$ at $I_4 = 0.5$ would be a nonzero positive value — the exponential's $(I_4 - 1)^2$ argument is positive regardless of the sign of $I_4 - 1$, which would unphysically resist compression of the fiber direction. This is a known HGO-impl gotcha; the switch is essential, not optional.

## Two-fiber families

Two-fiber-family HGO — the standard form for materials with two distinct fiber families (woven sleeves, biological vessel walls) — adds two exponential terms with shared $(k_1, k_2)$:

$$ \psi_\text{fiber}^{(2f)}(I_4^{(1)}, I_4^{(2)}) = \sum_{i=1,2} \frac{k_1}{2 k_2}\left( e^{k_2 (I_4^{(i)} - 1)^2} - 1 \right) $$

Materials with two physically distinct fiber families — different fiber types or asymmetric weave — would carry per-direction $(k_1^{(i)}, k_2^{(i)})$ pairs; the trait surface accommodates this via the resolver's compile-time branch on the output type. For the canonical problem's symmetric reinforcement patterns the shared-parameter form is sufficient.

## Numerical regime guard

At large fiber stretch, both $\psi_\text{fiber}$ and its derivatives grow exponentially; for $k_2$ values typical of stiff fibrous reinforcement, the magnitudes outrun the physical regime the parameters were fit on long before any IEEE-754 overflow. `sim-soft`'s HGO impl exposes a `max_fiber_stretch: f64` cap on $\lambda_f = \sqrt{I_4}$ that triggers a [`GradientEstimate::Noisy`](../../60-differentiability/05-diff-meshing.md) flag when exceeded; the optimizer treats outputs from clamped evaluations as untrusted rather than letting unphysical tangent magnitudes corrupt the autograd tape.

The cap is not a model improvement — it is a regime-mismatch guard. Real soft parts that approach the cap are operating outside the regime HGO was fit on, and the right response is to flag the result for review rather than silently extrapolate. The cap value is per-material, calibrated alongside $k_1$ and $k_2$.

## Construction

```rust
pub type Hgo<R> = Anisotropic<NeoHookean, R>;

impl<R: FiberDirection> Hgo<R> {
    pub fn from_neo_hookean(base: NeoHookean, k1: f64, k2: f64, fiber: R) -> Self {
        assert!(k1 > 0.0, "HGO k1 must be positive");
        assert!(k2 > 0.0, "HGO k2 must be positive");
        Anisotropic { base, k1, k2, fiber }
    }
}
```

The base struct definition lives in the [Ch 00 composition leaf](../00-trait-hierarchy/01-composition.md); the impl block above adds the convenience constructor. Mooney-Rivlin or Ogden bases are also constructible via direct `Anisotropic<MooneyRivlin, R>` or `Anisotropic<Ogden, R>`; the type alias `Hgo<R>` is shorthand for the dominant configuration only.

## What this sub-leaf commits the book to

- **HGO is the default `Anisotropic<M, R>` specialization.** Exponential fiber-stiffening $\psi_\text{fiber} = (k_1/2k_2)(e^{k_2(I_4-1)^2} - 1)$, two parameters $(k_1, k_2)$ stored on the decorator as locked in [Ch 00 composition](../00-trait-hierarchy/01-composition.md). `Hgo<R>` aliases `Anisotropic<NeoHookean, R>` as the convenience name for the dominant configuration.
- **The exponential captures J-shaped stiffening real fibers exhibit at moderate-to-large stretch.** Polynomial forms underfit; HGO's two parameters separate small-strain stiffness ($k_1$) from large-strain stiffening rate ($k_2$).
- **Stress-free reference and convexity in $I_4$ are automatic.** $\psi'_\text{fiber}(1) = 0$ via the $(I_4 - 1)$ factor; $\psi''_\text{fiber}(I_4) > 0$ everywhere.
- **Tension-only switch is essential.** Without it the symmetric $(I_4 - 1)^2$ exponent would unphysically resist compression. The switch and smoothing live in the fiber-reinforced sibling, not in the HGO impl itself.
- **Two-fiber HGO sums two exponential terms with shared $(k_1, k_2)$.** Per-direction parameters are an extension via the resolver type; the standard form covers symmetric reinforcement patterns.
- **Large-stretch evaluation is guarded, not modeled away.** `max_fiber_stretch` triggers `GradientEstimate::Noisy` at the cap; the optimizer flags rather than silently extrapolates.
