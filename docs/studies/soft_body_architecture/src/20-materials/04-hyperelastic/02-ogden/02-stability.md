# Numerical stability

The [N-term sibling](00-n-term.md) named two hot-path cost centers — non-integer `pow` evaluations per principal direction per term, and a per-Gauss-point eigen-decomposition — and named one ill-conditioning site — the repeated-eigenvalue configuration where the principal-stretch tangent's off-diagonal blocks diverge. This leaf states how `sim-soft` handles each, what the gradcheck suite asserts against the handling, and the regime where the impl falls back to an invariant-based equivalent form rather than push through the degeneracy.

## Non-integer `pow` on the hot path

Evaluating $\lambda^{\alpha}$ for non-integer $\alpha$ is several-fold slower than integer-power inlining on modern CPUs. `f64::powf` is a transcendental-function library call — roughly the cost of a few dozen `mul`/`add` operations, per the standard hardware microarchitectures. Per Gauss point, a naive $N$-term Ogden evaluation needs $3N$ `powf` calls for the first Piola (each principal direction × each Ogden term) plus $3N$ more for the tangent's $\partial P / \partial \lambda$ pieces — $6N$ total before caching — plus the eigen-decomposition. The optimizations below collapse the $6N$ to $3N$ via a shared-power arithmetic identity.

`sim-soft`'s `impl Material for Ogden` uses three arithmetic-cost optimizations:

- **Integer-exponent branching.** If every $\alpha_i$ in the configured $(\mu_i, \alpha_i)$ vector is a small integer (uncommon for measured silicone but common for unit tests and published fits), the impl branches to `powi` with hand-unrolled multiplications for $\alpha_i \in \{1, 2, 3, 4, -1, -2\}$. The calibration reader records the integer-vs-non-integer flag at `from_measured` construction; the runtime branch has no per-Gauss-point overhead.
- **Shared $\lambda^{\alpha - 1}$ and $\lambda^{\alpha - 2}$.** The first-Piola uses $\lambda^{\alpha_i - 1}$, the tangent uses $\lambda^{\alpha_i - 2}$, and both share the $\lambda^{\alpha_i}$ evaluation. One `powf(lambda, alpha - 2)` gives $\lambda^{\alpha_i - 2}$; multiplying by $\lambda$ gives $\lambda^{\alpha_i - 1}$; multiplying again gives $\lambda^{\alpha_i}$. Three powers at the cost of one transcendental call per $(i, j)$ pair.
- **`EvaluationScratch`-cached principal stretches and eigenvectors.** Same pattern as [neo-Hookean's](../00-neo-hookean/03-impl.md) caching of $F^{-T}$ and $\ln J$ — the eigen-decomposition runs once per Gauss-point visit and the `pow` results are computed once; the first-Piola and tangent calls at the same Gauss point share the scratch by reference.

These keep Ogden's per-Gauss-point arithmetic well-defined but do not make it cheap; the [Mooney-Rivlin tradeoffs sub-leaf](../01-mooney-rivlin/02-tradeoffs.md) names the regime where paying this cost is justified.

## Isotropic-point degeneracy

At the reference configuration $F = I$ the principal stretches are $\lambda_1 = \lambda_2 = \lambda_3 = 1$ and the eigenvectors of $U$ are degenerate — any orthonormal basis serves. The principal-stretch derivatives $\partial \lambda_j / \partial F$ that enter the tangent are ill-conditioned: the eigenvector resolution is under-determined in the degenerate subspace, and the usual closed-form expression for $\partial \lambda_j / \partial F$ in terms of the eigenvectors has $0/0$ components where two stretches coincide.

The same problem arises at any near-repeated configuration — $\lambda_1 \approx \lambda_2$ with $\lambda_3$ different, or $\lambda_2 \approx \lambda_3$, etc. A rigid rotation plus small stretch is close to the isotropic point; a uniaxial-dominated deformation has two near-coincident lateral stretches.

`sim-soft` handles the degeneracy via a two-level strategy. Let $\delta_{\min} = \min_{i \neq j} |\lambda_i - \lambda_j|$ be the smallest principal-stretch gap.

- **Small gap, $\delta_{\min} > \sqrt{\varepsilon_\text{mach}} \approx 1.5 \times 10^{-8}$.** The closed-form principal-stretch derivatives are well-conditioned enough to use directly. This is the common case in any non-near-isotropic configuration; the condition-number penalty scales like $1 / \delta_{\min}$ and stays bounded by a modest factor.
- **Near-degenerate gap, $\delta_{\min} \leq \sqrt{\varepsilon_\text{mach}}$.** The impl switches to the invariant-based equivalent form of the Ogden energy. At exactly the isotropic point, every Ogden term collapses to $(\mu_i / \alpha_i)(3\lambda^{\alpha_i} - 3)$ where $\lambda$ is the common principal stretch; the first Piola and tangent have well-defined limits that can be computed from the isotropic expression directly, bypassing the eigenvector resolution. For near-degenerate configurations slightly off the isotropic point, a Taylor expansion around the isotropic limit — with the small gap $\delta_{\min}$ as the expansion parameter — gives a closed-form tangent that agrees with the full principal-stretch form to leading order in the gap.

The threshold $\sqrt{\varepsilon_\text{mach}}$ is a standard choice: below it, the numerical representation of the principal-stretch difference is dominated by round-off, and any division by it amplifies noise catastrophically; above it, the closed form is safe.

At the reference configuration exactly — which is what the [Phase B gradcheck](../../../110-crate/04-testing/03-gradcheck.md)'s small-strain reduction test sees — the isotropic fallback path is what produces the six-digit match against linear elasticity. The fallback path agrees with the general-case path to machine precision at the degeneracy by construction.

## Inversion handling

`Ogden` declares `InversionHandling::RequireOrientation` on its [`ValidityDomain`](../../00-trait-hierarchy/02-validity.md), matching neo-Hookean and Mooney-Rivlin. If $\det F \leq 0$ reaches any principal-stretch evaluation — which would leave at least one $\lambda_j$ negative or zero, and $\lambda_j^{\alpha_i}$ for non-integer $\alpha_i$ is then either complex or undefined — the impl panics with the element index and Gauss point, and downstream diagnostics surface the panic as an IPC-barrier failure rather than a constitutive bug.

The volumetric term $U_\text{vol}(J)$ diverges as $J \to 0^+$ in both the compressible and modified forms — the standard $\tfrac{\lambda}{2}(\ln J)^2$ choice goes to $+\infty$ — so the energy itself guarantees an infinite barrier against inversion in the closed-form sense. IPC is what prevents finite-step Newton iterations from crossing into negative-$\det F$ territory in practice; the constitutive law is not expected to handle the post-inversion regime.

## Gradcheck hooks

The [gradcheck suite](../../../110-crate/04-testing/03-gradcheck.md) runs the same four assertions on `Ogden` it runs on every `Material`, with two Ogden-specific additions:

- **Rotation invariance.** Structural from the principal-stretch parameterization; test is a sanity check.
- **Small-strain reduction.** Six-digit match against linear elasticity at $F = I + 10^{-3}\,G$. Exercises the isotropic-fallback path, because $F$ is near the isotropic point.
- **First Piola = $\partial \psi / \partial F$.** Finite-difference against central-difference of energy. At typical-deformation configurations, not at the reference.
- **Tangent = $\partial P / \partial F$.** Finite-difference against central-difference of first Piola. Same typical-deformation configurations.
- **Degeneracy crossing.** A sweep from $F$ clearly non-degenerate through a near-isotropic configuration back to non-degenerate, asserting that $\psi, P, \mathbb{C}$ are continuous (no jumps at the $\sqrt{\varepsilon_\text{mach}}$ threshold) and that first Piola and tangent agree with their finite-difference references on both sides and at the transition.
- **$N$-term sum-rule reduction.** At small strain, verify that the evaluated small-strain shear modulus matches $\tfrac{1}{2}\sum_i \mu_i \alpha_i$ to six digits — catches calibration-versus-impl drift where the fit enforced the sum rule but the impl's evaluated tangent does not match.

The degeneracy-crossing test is where a bug in the fallback-threshold handling shows up. Passing it is a precondition for `Ogden` landing in [`material/`](../../../110-crate/00-module-layout/00-material.md).

## What this sub-leaf commits the book to

- **Non-integer `pow` is a hot-path cost, not a pathology.** The impl uses integer-exponent branching where applicable and caches $\lambda^{\alpha_i - 2}, \lambda^{\alpha_i - 1}, \lambda^{\alpha_i}$ from one transcendental call per $(i, j)$ pair. Beyond that, the cost is real and the [Mooney-Rivlin tradeoffs sub-leaf](../01-mooney-rivlin/02-tradeoffs.md) is where the decision to pay it sits.
- **Isotropic-point degeneracy is handled by a threshold-triggered fallback path.** Below $\sqrt{\varepsilon_\text{mach}}$ gap, the impl uses an invariant-equivalent form that has well-defined limits; above it, the closed-form principal-stretch derivatives. The threshold and the cross-over continuity are gradcheck-tested.
- **Inversion is not the constitutive law's job.** `InversionHandling::RequireOrientation`, panic on $\det F \leq 0$, IPC barrier is what actually prevents the configuration from arising.
- **The gradcheck suite's degeneracy-crossing test is Ogden-specific.** The other hyperelastic laws don't need it because their invariant evaluation has no eigenvector-resolution ambiguity; Ogden does, and the test is what catches fallback-threshold bugs.
