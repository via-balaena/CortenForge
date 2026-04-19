# Penalty method failure modes

Penalty contact is the simplest possible formulation: overlap between two bodies generates a restoring force proportional to the overlap depth. The [Part 1 Ch 02 popping sub-leaf](../../10-physical/02-what-goes-wrong/02-popping.md) named this as the first of three off-the-shelf solver classes that reproduce popping on the canonical problem; the [parent Ch 00](../00-why-ipc.md) claimed penalty is structurally wrong for `sim-soft` because its force law is discontinuous at the contact boundary. This sub-leaf unpacks that structural wrongness — why the tradeoff between stiffness and stability is unavoidable under penalty, why the discontinuity at the contact boundary kills both Newton and autograd, and why "turn the stiffness up" is not a path forward.

## The force law and its derivative

Standard penalty contact writes the contact force along the outward surface normal as:

$$ F_\text{pen}(d) = \kappa_\text{pen} \max(0, \hat d - d) $$

for gap $d$, contact tolerance $\hat d$, and penalty stiffness $\kappa_\text{pen}$. The force is continuous in $d$ — zero for $d \ge \hat d$, rising linearly as the gap closes — but its derivative has a jump discontinuity at $d = \hat d$:

$$ \frac{dF_\text{pen}}{dd} = \begin{cases} -\kappa_\text{pen} & d < \hat d \\ 0 & d > \hat d \end{cases} $$

The Hessian contribution from a contact pair is therefore discontinuous across the contact/no-contact boundary. This one fact is every downstream failure.

## Newton fails at the boundary

The [Part 5 Ch 00 backward-Euler Newton loop](../../50-time-integration/00-backward-euler.md) iterates on $H \Delta x = -r$ where $H$ is the Hessian of the total potential and $r$ is its gradient. When a contact pair is near $d = \hat d$, a trial Newton step may move the gap across the boundary; under penalty, the Hessian entry for that pair either appears or vanishes depending on which side the step lands. The Newton iteration sees two different linear systems at two consecutive iterates and oscillates without converging. Line search cannot fix this because the Newton direction itself is computed with a Hessian that is wrong on the other side of the boundary: Armijo backtracking may satisfy the merit-function decrease condition with a shorter step, but the direction the iterate is following was derived under an inconsistent Hessian, and successive iterations keep reversing across the kink without reaching a converged state.

## The stiffness-vs-stability tradeoff

Penalty contact has one tuning knob, $\kappa_\text{pen}$. To enforce non-penetration to a tolerance $\delta_\text{pen}$, the stiffness must satisfy $\kappa_\text{pen} \gtrsim F_\text{max} / \delta_\text{pen}$ for the largest expected contact force $F_\text{max}$. Larger $F_\text{max}$ or tighter $\delta_\text{pen}$ demands larger $\kappa_\text{pen}$. But the effective contact frequency is $\omega_c = \sqrt{\kappa_\text{pen} / m_\text{eff}}$ for effective mass $m_\text{eff}$; under explicit integration the stability limit is $\Delta t < 2/\omega_c$, so $\Delta t$ must shrink as $\sqrt{1/\kappa_\text{pen}}$. Under implicit integration the tradeoff moves from stability to conditioning — the Hessian condition number grows linearly in $\kappa_\text{pen}$, and both Newton's convergence rate and the conjugate-gradient inner-iteration count degrade.

Neither path escapes the tradeoff. Increasing $\kappa_\text{pen}$ to enforce tighter non-penetration buys either smaller timesteps (explicit) or more inner iterations per Newton step (implicit), and in both cases at finite $\kappa_\text{pen}$ there remains a finite penetration depth at any finite load. The asymptotic limit $\kappa_\text{pen} \to \infty$ recovers a hard non-penetration constraint — an indicator function that is zero for $d \ge \hat d$ and infinite for $d < \hat d$. This is not the IPC barrier, which is a smooth logarithmic potential on $(0, \hat d)$; the two limiting objects share the non-penetration property but are mathematically distinct and have different gradient and Hessian structure in the band above the boundary. Penalty does not become IPC in the large-stiffness limit; it becomes an unsolvable hard constraint.

## Energy-conservation clarification

Pure penalty contact *is* a conservative potential: the force derives from $\Psi_\text{pen}(d) = \tfrac{1}{2} \kappa_\text{pen} \max(0, \hat d - d)^2$. Energy is not lost under penalty; it is stored in overlap. The popping artefact is therefore not a conservation failure — it is a discontinuity failure. The overlap energy is released on separation; the problem is that the release happens across a non-smooth Hessian transition that produces ringing oscillation at the contact's characteristic frequency. This distinction is worth stating because "non-conservative contact" is a common folk framing of penalty's pathology, and the fix (smoothness of the potential's second derivative) is not what conservation failure would demand (added dissipation).

## Why autograd breaks at the same point

Claim 1 of the [parent chapter](../00-why-ipc.md) was that the visible failures are the same algorithmic property that breaks differentiability. Under penalty, the total potential energy is $C^1$ but only piecewise $C^2$ — value and gradient (force) are continuous at $d = \hat d$, but the Hessian has a finite jump from $\kappa_\text{pen}$ (inside the band) to zero (outside). The implicit-function adjoint from [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md) requires $\partial r/\partial x$ (equivalently, the Hessian of $U$) to be continuous at the converged state $x^\ast$; under penalty this fails whenever the converged state has any contact pair at $d = \hat d$. Gradient-based optimization through a penalty-contact simulator is therefore limited to configurations that are robustly in-contact or robustly out-of-contact; the transition band is a forbidden zone. The [reward function](../../10-physical/01-reward.md) reads contact pressure across exactly that transition band — pressure rises from zero as contact engages — so gradient optimization has no reach at the boundary where it is most needed.

## What this sub-leaf commits the book to

- **`sim-soft` does not ship a penalty contact implementation, even as a baseline.** The [Part 4 Ch 01 barrier](../01-ipc-internals/00-barrier.md) is the only contact force law. Penalty appears only in comparative prose here and in [Part 1 Ch 02 popping](../../10-physical/02-what-goes-wrong/02-popping.md).
- **The discontinuity-at-boundary diagnosis is load-bearing.** When future chapters compare IPC against alternatives, the comparison is "smooth barrier beats discontinuous force law," never "strong penalty beats weak penalty."
- **The $\kappa_\text{pen} \to \infty$ limit is a comparative device, not an implementation path.** Penalty does not become IPC in practice; it becomes numerically unsolvable. The limit argument is correct as a sanity check and wrong as a roadmap.
