# Temperature-dependent modulus

The [Ch 08 parent's Claim 1](../08-thermal-coupling.md) commits to temperature as a material parameter, not a kinematic one — the temperature field modulates the wrapped material's Lamé parameters without changing the constitutive form. This leaf writes the per-Gauss-point modulus query, the linear-around-reference parameterization the [`Thermal<M>` decorator](../00-trait-hierarchy/01-composition.md) ships as default, and the $\partial P / \partial T$ tangent contribution the Newton solver picks up.

## Why temperature affects modulus

Polymer chain mobility is temperature-dependent. At higher temperatures, chains move more freely and the cross-link network resists deformation less; the bulk material softens. At lower temperatures, chains are stiffer and the material hardens. For silicones in the operating temperature window of soft-robotics applications, the response is monotonic in $T$ and well-approximated as linear over a several-degree band around a reference temperature.

The constitutive form (neo-Hookean, Mooney-Rivlin, Ogden, HGO) is unchanged by temperature: the material behaves like the same hyperelastic law, just with rescaled coefficients. This is what lets the [`Thermal<M>` decorator](../00-trait-hierarchy/01-composition.md) be a pure decorator over any `M: Material` rather than a separate constitutive family.

## Linear modulus parameterization

For Lamé pair $(\mu, \lambda)$ with reference values $(\mu_0, \lambda_0)$ at reference temperature $T_0$, the linear-around-reference form is:

$$ \mu(T) = \mu_0\, \left(1 - c_\mu\, (T - T_0)\right) $$

$$ \lambda(T) = \lambda_0\, \left(1 - c_\lambda\, (T - T_0)\right) $$

with $c_\mu, c_\lambda$ the per-Lamé-parameter modulus temperature coefficients (units of inverse temperature). For typical silicones the coefficients are positive and small, capturing the modest softening per degree above the reference; for cold-regime regions where the material approaches its glass transition the linear form fails and a more general curve is needed. The letter $c$ here is local to the modulus-curve formula and distinct from the thermal stretch factor $\beta(T)$ introduced in [01-expansion](01-expansion.md), which uses $\beta$ for the multiplicative thermal-stretch scalar.

The [Ch 00 composition leaf](../00-trait-hierarchy/01-composition.md)'s `Thermal<M>` decorator stores the modulus curve as a closure `Box<dyn Fn(f64 /* T */) -> (f64 /* mu */, f64 /* lambda */)>`. The default impl ships the linear form above; user-supplied curves (tabulated, polynomial, Arrhenius-style for wide ranges) replace the closure at construction time without changing the decorator's interface to the rest of the solver.

## Per-Gauss-point evaluation

At every Newton iteration of every step, the decorator queries `sim-thermostat`'s temperature field at each per-element Gauss point's reference position $x_\text{ref}$:

$$ T_g = T(x_\text{ref}^{(g)}, t_n) $$

evaluates $(\mu(T_g), \lambda(T_g))$ via the stored modulus curve, and passes the temperature-adjusted Lamé pair to the wrapped base material's `first_piola(F)` call. The base computes its constitutive response with these per-Gauss-point parameters; the decorator does not interfere with the base's evaluation otherwise.

The temperature value $T_g$ is treated as constant within a single mechanical Newton step: the [thermostat substep](../../110-crate/02-coupling/01-thermostat.md) runs separately from the mechanical solver's substep, and the temperature field at $t_n$ is the static input the mechanical step sees. The Newton iteration's nonlinearity is purely in $F$; the temperature-dependence enters as a static modulus rescaling.

## $\partial P / \partial T$ tangent contribution

When the [bidirectional coupling](02-dissipation.md) is live, the residual depends on $T$ as well as $F$, and the [Newton tangent](../../50-time-integration/00-backward-euler.md) picks up an additional term $\partial P / \partial T$ that flows through the modulus curve. By chain rule:

$$ \frac{\partial P}{\partial T}\bigg|_\text{mod} = \frac{\partial P}{\partial \mu}\, \frac{d\mu}{dT} + \frac{\partial P}{\partial \lambda}\, \frac{d\lambda}{dT} $$

$\partial P / \partial \mu$ and $\partial P / \partial \lambda$ are the constitutive law's parameter sensitivities — quantities the base material can expose via a parameter-derivative trait method, or that the decorator can compute by finite differencing the base's `first_piola` call. For the [linear elastic baseline](../02-linear.md) and the [neo-Hookean energy](../04-hyperelastic/00-neo-hookean/00-energy.md), the closed-form expressions are simple polynomials in $F$.

The contribution enters the Newton residual when the temperature change between the previous and current step is non-zero — at thermostat substeps that move $T$, the mechanical solver's residual sees a stress shift even at fixed $F$. The [Phase B gradcheck](../../110-crate/03-build-order.md) verifies that the $\partial P / \partial T$ contribution agrees with finite-difference numerical perturbations of the temperature field. The full $\partial P / \partial T$ also includes the [thermal-expansion contribution](01-expansion.md); both flow into the same tangent slot.

## Validity-domain widening

`Thermal<M>` widens the [composed `ValidityDomain`](../00-trait-hierarchy/02-validity.md)'s `temperature_range` slot from "isothermal at the base material's reference $T_0$" to the linear-modulus-curve's validity band. Beyond that band the user must supply a non-linear modulus curve; the decorator's default validity flag is set to "out-of-band — verify the modulus curve covers the operating range."

The Poisson cap from the base material is inherited unchanged — temperature affects the magnitudes of the Lamé pair but not their ratio (under the linear approximation), so the near-incompressibility structure of the wrapped base law carries over.

## What this sub-leaf commits the book to

- **Temperature modulates the wrapped material's Lamé pair, not its constitutive form.** $(\mu(T), \lambda(T))$ via a closure stored on `Thermal<M>`; default linear-around-reference form $\mu(T) = \mu_0\, (1 - c_\mu\, (T - T_0))$.
- **Per-Gauss-point evaluation, static within a Newton step.** Thermostat's temperature field at $t_n$ is the input; the mechanical solver iterates in $F$ with the modulus held fixed for the step.
- **Newton tangent picks up $\partial P / \partial T$ when the bidirectional coupling is live.** Chain-rule through the modulus curve plus the [thermal-expansion contribution](01-expansion.md); gradcheck-verified at Phase B.
- **The user-supplied curve replaces the linear default at construction.** Tabulated, polynomial, or Arrhenius-style curves slot into the same `Box<dyn Fn(f64) -> (f64, f64)>` closure without changing the decorator's interface.
- **Validity widens the temperature range; the Poisson cap is inherited from the base.** The linear approximation has a band; outside the band, the user must supply a curve covering the wider range.
