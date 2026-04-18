# Validity regimes and error bounds

Every `Material` impl in Part 2 is correct on some regime and wrong outside it. Linear elasticity is correct at infinitesimal strain; corotational extends correctness to moderate stretch plus arbitrary rotation; neo-Hookean extends further to around 100% stretch; Ogden extends further still. None is universally correct. The [trait-surface sibling](00-trait-surface.md) committed a `validity()` method on every `Material`; this leaf writes down what the method returns, what the gradcheck suite verifies against it, and what `sim-soft` does when an element exceeds its declared regime at runtime.

## The descriptor

```rust
pub struct ValidityDomain {
    /// Maximum principal stretch deviation, |lambda_stretch - 1|,
    /// at which the declared error bound against the next-more-accurate
    /// reference law still holds.
    pub max_stretch_deviation: f64,

    /// Maximum rotation angle in radians. f64::INFINITY for rotation-invariant laws.
    pub max_rotation: f64,

    /// Allowed Poisson ratio range. Upper bound below 0.5 for laws with no
    /// near-incompressibility cure; wider when composed with a mixed-u-p or
    /// F-bar locking-fix decorator from Ch 05.
    pub poisson_range: (f64, f64),

    /// Allowed temperature range when wrapped by Thermal<M>.
    pub temperature_range: Option<(f64, f64)>,

    /// Strain-rate regime for rate-dependent laws. None for rate-independent.
    pub strain_rate_range: Option<(f64, f64)>,

    /// Behavior under element inversion (det F <= 0).
    pub inversion: InversionHandling,
}
```

Six slots. The stretch, rotation, and Poisson bounds and the inversion handler are populated by every base law; the temperature and strain-rate slots are `None` for a rate-independent thermally-uncoupled base impl and filled in when a `Thermal<M>` or `Viscoelastic<M>` decorator wraps it. A composed law like `Thermal<Viscoelastic<Hgo<R>>>` returns a descriptor with all six slots populated without growing the base trait surface.

## Per-law declarations

Each impl sets the descriptor from its own derivation. The table below names representative bounds for the laws Pass 1 has written; each law's full statement lives in its own sub-chapter.

| Law | Stretch / strain bound | Rotation | Poisson | Source |
|---|---|---|---|---|
| Linear | ~1% strain | ~1° | < 0.45 | [Ch 02](../02-linear.md#validity-domain-declaration) |
| Corotational | stretch in [0.8, 1.25] | unrestricted | < 0.45 | [Ch 03](../03-corotational.md#validity-domain-declaration) |
| Neo-Hookean | ~100% stretch | unrestricted | < 0.499 with mixed u-p | [Ch 04 — NH](../04-hyperelastic/00-neo-hookean.md) |
| Mooney-Rivlin | ~150% stretch | unrestricted | < 0.499 with mixed u-p | [Ch 04 — MR](../04-hyperelastic/01-mooney-rivlin.md) |
| Ogden ($N=2$ or $3$) | into the strain-hardening regime per the Ecoflex fit | unrestricted | < 0.499 with mixed u-p | [Ch 04 — Ogden](../04-hyperelastic/02-ogden.md) |

[Ch 05's incompressibility treatments](../05-incompressibility.md) widen the Poisson-range upper bound on any hyperelastic impl by feeding into a mixed-u-p or F-bar composite; the descriptor is updated by the composition builder when the locking-fix decorator is wrapped. The Poisson upper bound the user sees on a composed law is therefore the one the composition produces, not the base impl's standalone bound.

## Gradcheck-verified error bounds

The declared domain is only trustworthy if a test exercises it. Every `Material` impl is run against a next-more-accurate reference — linear against neo-Hookean, corotational against neo-Hookean, neo-Hookean against Ogden, Mooney-Rivlin against Ogden — at canonical-problem Gauss-point strain states sampled inside and outside the declared domain. The [Part 11 Ch 04 gradcheck suite](../../110-crate/04-testing/03-gradcheck.md) asserts:

- **Inside the domain.** Error in $\psi$, $P$, and tangent below the impl's declared tolerance (six digits in the small-strain limit for linear versus neo-Hookean; a few percent at the declared stretch bound for the hyperelastic family).
- **Outside the domain.** Error grows with the expected asymptotic — quadratic in excess strain for linear against hyperelastic, quadratic in excess stretch for corotational against hyperelastic. An impl whose out-of-domain error does not match the declared asymptotic fails the validity test, even if its in-domain numbers are fine.

The asymptotic check is what separates a validity declaration from a hand-wave. A law that claims correctness up to 25% stretch and silently diverges at 30% rather than degrading quadratically has a bug the test catches at the asymptotic, not at the numerics.

## Runtime warnings

`sim-soft` emits a runtime warning the first time a Gauss point in a solve exceeds its material's declared domain. The warning is rate-limited (once per material per solve) and carries the element index and the specific axis (stretch, rotation angle, temperature, strain rate) that exceeded the declaration.

The warning is loud by default. A user running an intentional stress test can silence it via an explicit opt-in acknowledgement on the `Material` instance; the default for every solve run through the [Newton loop](../../50-time-integration/00-backward-euler.md) is that out-of-domain evaluations are a bug the simulation surfaces rather than hides.

## What this sub-leaf commits the book to

- **Every `Material` impl declares its regime in code, not in prose.** The `validity()` method is required by the trait surface; there is no impl in `sim-soft` that ships without a populated `ValidityDomain`.
- **Gradcheck tests the asymptotic, not just the numbers.** An impl whose out-of-domain error diverges instead of degrading at the claimed rate is a bug, not a domain limitation.
- **Out-of-domain evaluations surface at runtime.** The simulation does not silently produce values outside the regime the impl declared; the user sees a warning or an opt-in acknowledgement.
- **The validity descriptor is how "A-grade or it doesn't ship" becomes testable for constitutive laws.** A candidate `Material` that either under-declares (to avoid warnings) or over-declares (to claim range it doesn't have) fails the Phase B gradcheck and does not land in `material/`.
