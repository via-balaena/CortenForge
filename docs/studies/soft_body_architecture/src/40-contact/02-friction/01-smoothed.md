# Smoothed Coulomb

The [previous sub-leaf](00-coulomb.md) identified the set-valued stick point as the mathematical obstacle to consuming Coulomb friction in a Newton solver. This sub-leaf writes the smoothing that IPC uses, following [Li et al. 2020](../../appendices/00-references/00-ipc.md#li-2020). The object that enters the total potential is a *position-space* dissipative potential — a smooth function of the current configuration $x$, not a rate-of-dissipation function of velocity. The distinction matters: the backward-Euler Newton loop minimizes a function of $x$, not $\dot x$, so the friction contribution must be a function of $x$ to compose with it. Writing the smoothing in position space, with velocity appearing only implicitly through the timestep $h$, is the single design decision that makes the rest of this chapter possible.

## Lagged local frame and tangential displacement

At a contact pair with a lagged tangent basis $T^n$ (a $3 \times 2$ matrix whose columns span the contact-point tangent plane from the prior Newton iterate or prior timestep; the lagging is [explained in the next sub-leaf](02-ipc-friction.md)) and lagged normal-force magnitude $\lambda^n \ge 0$ (the subscript is dropped in this single-pair context; [Ch 02's integration sub-leaf](02-ipc-friction.md) restores pair indexing as $\lambda_k^n$), define the relative tangential displacement since the start of the current timestep:

$$ u_T(x) = (T^n)^\top \big( x - x^t \big) $$

where $x^t$ is the configuration at the start of the step. This is a 2-vector in the tangent plane — a pure displacement, not a velocity. The relative tangential velocity $\dot u_T \approx u_T(x)/h$ is implicit in the ratio to the timestep $h$.

## The smoothing kernel and its antiderivative

Li 2020's smoothing kernel is a $C^1$ piecewise function $f_1: \mathbb{R}_{\ge 0} \to [0, 1]$:

$$ f_1(y) = \begin{cases} -y^2/(\epsilon_v^2 h^2) + 2y/(\epsilon_v h) & 0 \le y \le h\epsilon_v \\ 1 & y \ge h \epsilon_v \end{cases} $$

Here $\epsilon_v$ is a velocity threshold in m/s, so $h\epsilon_v$ has units of displacement and serves as the width of the smooth transition zone $[0, h\epsilon_v]$ in displacement space. The IPC default is $\epsilon_v = 10^{-3} L_\text{bbox}$ m/s for bounding-box length $L_\text{bbox}$.

Direct verification at the seam $y = h\epsilon_v$: $f_1(h\epsilon_v) = -1 + 2 = 1$; $f_1'(h\epsilon_v) = -2/(\epsilon_v h) + 2/(\epsilon_v h) = 0$. Both sides meet with matching value and first derivative, so $f_1$ is $C^1$ globally. The second derivative has a jump from $-2/(\epsilon_v^2 h^2)$ to $0$ at the seam — $f_1$ is not $C^2$, by design. A $C^2$ kernel would introduce more smoothing and slow Newton convergence; Li 2020 identifies $C^1$ as the sweet spot.

The position-space friction potential uses the *antiderivative* $f_0$ with $f_0' = f_1$. Because $f_1$ is $C^1$, $f_0$ is $C^2$, which is the property the Newton loop cares about — the Hessian of the friction potential exists and is continuous. On $[0, h\epsilon_v]$:

$$ f_0(y) = C - \frac{y^3}{3 \epsilon_v^2 h^2} + \frac{y^2}{\epsilon_v h} $$

The additive constant $C$ is physically irrelevant (gradients are unchanged); Li 2020 normalizes so that $f_0(h\epsilon_v) = h\epsilon_v$, which gives $C = h\epsilon_v/3$. For $y \ge h\epsilon_v$, $f_0'(y) = 1$, so $f_0(y) = y$.

## The friction potential

The per-pair friction potential is:

$$ D(x) = \mu_c \, \lambda^n \, f_0(\|u_T(x)\|) $$

Single-pair notation throughout this sub-leaf; [the integration sub-leaf](02-ipc-friction.md) restores the pair index as $D_k$ and gives the backward-Euler-scaled sum $h^2 \sum_k D_k$ that enters $U_\text{total}$.

Direct gradient of the single pair's $D$:

$$ \nabla_x D = \mu_c \lambda^n \, f_1(\|u_T\|) \, T^n \, \frac{u_T}{\|u_T\|} $$

For $\|u_T\| \ge h\epsilon_v$, $f_1 = 1$ and the gradient reproduces the Coulomb slip force $-\mu_c \lambda^n \, T^n u_T/\|u_T\|$ (with the sign coming from the potential's minus-sign convention in force = $-\nabla D$). For $\|u_T\| \in (0, h\epsilon_v)$, $f_1 \in (0, 1)$ and the force magnitude ramps from $0$ at the origin up to the full $\mu_c \lambda^n$ at the seam.

## Behaviour at the origin

At the origin, $f_1(0) = 0$ and $f_1'(0) = 2/(\epsilon_v h) > 0$. Combined with $f_0'(0) = f_1(0) = 0$ and $f_0''(0) = f_1'(0) = 2/(\epsilon_v h)$, Taylor-expanding $f_0(\|u\|)$ near $u = 0$:

$$ f_0(\|u\|) \approx \frac{h\epsilon_v}{3} + \frac{\|u\|^2}{h\epsilon_v} + O(\|u\|^3) $$

This is a smooth quadratic bowl in the components of $u$ — the norm non-smoothness at the origin is absorbed because $f_0$ vanishes to quadratic order there. The friction potential $D$ is therefore $C^2$ at $u_T = 0$ despite the inner $\|\cdot\|$ factor. Away from the origin, $f_0$ and the norm are both smooth, and the product is $C^2$ as well.

In the stick regime, an applied tangent load $F_\text{ext}$ with $\|F_\text{ext}\| \le \mu_c \lambda^n$ balances the smoothed friction at an equilibrium displacement that satisfies $f_1(\|u_T^\ast\|) = \|F_\text{ext}\|/(\mu_c \lambda^n)$. Solving the inner-piece quadratic for $\xi^\ast = \|u_T^\ast\|/(h\epsilon_v)$ gives $\xi^\ast = 1 - \sqrt{1 - \|F_\text{ext}\|/(\mu_c \lambda^n)}$, so the stick equilibrium lies within the transition zone $[0, h\epsilon_v]$: at zero load, $\xi^\ast = 0$ (exact stick at origin); at the Coulomb threshold $\|F_\text{ext}\| = \mu_c \lambda^n$, $\xi^\ast = 1$ (equilibrium at the seam). For small loads, the linear approximation $\|u_T^\ast\| \approx \|F_\text{ext}\| h\epsilon_v/(2 \mu_c \lambda^n)$ holds. Shrinking $\epsilon_v$ tightens the approximation: $\epsilon_v \to 0$ recovers exact stick, at the cost of Hessian condition number $\sim 1/(\epsilon_v h)$. The default $\epsilon_v = 10^{-3} L_\text{bbox}$ m/s puts the transition zone at sub-millimeter displacements for meter-scale bounding boxes — well below any visible slip the canonical problem requires.

## What this sub-leaf commits the book to

- **The friction potential is position-space, not velocity-space.** `sim-soft`'s `FrictionPair` term contributes $D(x)$ as a function of $x$ to the minimized incremental potential. A rate-form dissipation function $R(\dot u)$ would be a different object requiring a different integrator composition; the choice is made here and propagated downstream.
- **The Li 2020 smoothing is the shipped form, not an alternative.** The kernel $f_1$ and its antiderivative $f_0$ are fixed; user-facing tuning is $\epsilon_v$ (transition width) and $\mu_c$ (per-pair friction coefficient). The smoothing kernel is not parameterized at the material or API level.
- **The energy is $C^2$, the force is $C^1$.** This is what the [Ch 01 energy sub-leaf](../01-ipc-internals/03-energy.md)'s claim that $U_\text{total}$ is $C^2$ rests on. A $C^2$ friction energy contributes a continuous Hessian to the Newton assembly; the Hessian itself is not differentiable at the seam $\|u_T\| = h\epsilon_v$, but continuity is sufficient for Newton's convergence. The [Part 6 Ch 01 custom-VJP](../../60-differentiability/01-custom-vjps.md) layer inherits $C^2$ friction the same way it inherits $C^2$ barrier.
