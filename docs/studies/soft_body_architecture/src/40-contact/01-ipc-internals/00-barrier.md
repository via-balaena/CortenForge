# Barrier function

The IPC barrier is a scalar potential $b(d, \hat d)$ of a single gap distance $d$ and a tolerance $\hat d$. It is the object [Ch 00 guarantees](../00-why-ipc/02-ipc-guarantees.md) rest on — non-penetration, $C^2$ smoothness, and differentiability all follow from this one function's three design properties. This sub-leaf writes the barrier down, derives its first two derivatives, verifies the boundary conditions at $d = \hat d$, and explains why this specific form was chosen over simpler alternatives.

## The formula

For a contact pair with gap $d > 0$ and tolerance $\hat d > 0$:

$$ b(d, \hat d) = \begin{cases} -(d - \hat d)^2 \ln(d / \hat d) & 0 < d < \hat d \\ 0 & d \ge \hat d \end{cases} $$

This is the barrier introduced in [Li et al. 2020](../../appendices/00-references/00-ipc.md#li-2020). The product $-(d-\hat d)^2 \ln(d/\hat d)$ is positive on the active band because both factors are negative — $-(d-\hat d)^2 \le 0$ (minus a square) and $\ln(d/\hat d) < 0$ for $d < \hat d$, so their product is non-negative. The barrier diverges to $+\infty$ as $d \to 0^+$ because $\ln(d/\hat d) \to -\infty$ while the $(d - \hat d)^2$ factor approaches $\hat d^2$.

## First and second derivatives

Direct differentiation of $b$ on $(0, \hat d)$:

$$ b'(d) = -2(d - \hat d) \ln(d/\hat d) - (d - \hat d)^2 / d = (\hat d - d)\left[2 \ln(d/\hat d) - \hat d/d + 1\right] $$

$$ b''(d) = -2 \ln(d/\hat d) + \frac{\hat d}{d}\left(\frac{\hat d}{d} + 2\right) - 3 $$

The derivations are direct applications of the product and chain rules; both expressions are smooth on the open interval $(0, \hat d)$.

## Boundary conditions at $d = \hat d$

Evaluating at $d = \hat d$ (so $\ln(d/\hat d) = 0$ and $\hat d/d = 1$):

- $b(\hat d) = -(0)^2 \cdot 0 = 0$
- $b'(\hat d) = 0 \cdot [0 - 1 + 1] = 0$
- $b''(\hat d) = 0 + 1 \cdot (1 + 2) - 3 = 0$

All three vanish at the outer edge of the tolerance band. Combined with the zero value of $b$ outside the band, this makes $b$ a $C^2$ function globally: the energy, the force ($-b'$), and the stiffness contribution ($b''$) all connect smoothly across the active/inactive boundary.

This is the single most load-bearing fact of the barrier's construction. [Ch 00 Guarantee 2](../00-why-ipc/02-ipc-guarantees.md) depends on it — the $C^2$ property of the total potential energy is exactly this $C^2$ property of the barrier composed with the element-level $d(x)$ distance function. Changing the barrier to any form that does not satisfy $b(\hat d) = b'(\hat d) = b''(\hat d) = 0$ breaks that guarantee.

## Divergence as $d \to 0^+$

For the non-penetration guarantee, the other end of the interval matters. Expanding near $d = 0$: $\ln(d/\hat d) \to -\infty$ and $(d - \hat d)^2 \to \hat d^2$, so $b(d) \to +\infty$ at a logarithmic rate. The force $-b'(d) = (d - \hat d)[2 \ln(d/\hat d) - \hat d/d + 1]$ also diverges, dominated by the $\hat d/d$ term in the bracket: in the limit the force scales like $1/d$ (with coefficient $\hat d^2$), not like $1/d^2$ or a higher inverse power. This is the distinguishing feature of the logarithmic barrier: energy divergence is logarithmic (slow), force divergence is $1/d$ (moderate), and the Hessian diverges like $1/d^2$. Contrast with a $1/d$-style barrier where energy itself diverges like $1/d$ and force like $1/d^2$ — much stiffer conditioning, much harder on the Newton solver's inner linear inversion.

## Why this specific form

Three properties are simultaneously required and this barrier is the simplest function achieving all three:

**(1) Divergence at $d = 0$** prevents any Newton iterate from crossing into penetration — the potential is unbounded above as the gap closes, so no descent direction can cross $d = 0$. Any barrier that grows without bound at the origin satisfies this; logarithmic is the slowest-growing such function, which is what keeps the Hessian condition number manageable.

**(2) $C^2$ matching at $d = \hat d$** is required for the Newton tangent to be continuous as pairs enter and leave the active set. The three conditions $b(\hat d) = b'(\hat d) = b''(\hat d) = 0$ are the exact polynomial-matching conditions for smooth truncation. A simpler form like $-\ln(d/\hat d)$ would diverge correctly at zero but has $b(\hat d) = 0$, $b'(\hat d) = -1/\hat d \ne 0$ — the force is non-zero at the tolerance boundary, which introduces a jump in force as pairs enter the band.

**(3) Compact support on $(0, \hat d]$** keeps the per-pair cost bounded regardless of how far apart the bodies are. Pairs with $d \ge \hat d$ contribute zero and can be culled entirely from the energy sum and Hessian assembly — the [Part 4 Ch 03 BVH](../03-self-contact/00-bvh.md) only has to report pairs within $\hat d$, not all pairs in the scene. Compare a barrier with support on $(0, \infty)$ like $1/d$: every pair in the scene contributes to the energy at every Newton iterate, and the cost becomes quadratic in the number of surface primitives.

The quadratic pre-factor $(d - \hat d)^2$ is what unifies the boundary conditions — a logarithmic barrier alone gives divergence (property 1) but fails properties 2 and 3; a polynomial barrier gives smooth matching (property 2) but no divergence. The product $(d - \hat d)^2 \ln(d/\hat d)$ achieves all three because the polynomial factor vanishes at $\hat d$ fast enough to zero out the log's non-zero derivative there, and the logarithmic factor gives the required divergence at $0$.

## What this sub-leaf commits the book to

- **The barrier form is fixed.** `sim-soft` ships the Li 2020 barrier form and does not parameterize the barrier shape. The only per-pair parameters are $\hat d$ ([Part 4 Ch 05](../05-real-time/01-barrier-width.md), adaptive per pair) and the global stiffness $\kappa$ ([next sub-chapter](01-adaptive-kappa.md), adaptive per Newton iteration). The function of a single pair's gap is not tunable.
- **The boundary conditions $b = b' = b'' = 0$ at $d = \hat d$ are load-bearing, not cosmetic.** Changing the barrier form in a way that violates any of these three requires re-establishing the $C^2$ property of the total potential that every downstream chapter assumes.
- **The per-pair cost is bounded by construction.** The barrier's compact support on $(0, \hat d]$ is what makes the [Part 4 Ch 03 BVH broadphase](../03-self-contact/00-bvh.md)'s $\hat d$-radius query the right broadphase — pairs outside that radius are known to contribute zero.
