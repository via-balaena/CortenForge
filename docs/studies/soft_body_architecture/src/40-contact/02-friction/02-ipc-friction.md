# Integration with IPC

The [previous sub-leaf](01-smoothed.md) wrote the per-pair friction potential $D_k(x)$ as a $C^2$ function of the current configuration, conditional on a tangent basis $T^n$ and a normal-force magnitude $\lambda^n$ (superscript $n$ marks the lagged iterate). This sub-leaf closes the loop: where $T^n$ and $\lambda^n$ come from, why they are *lagged* rather than recomputed every Newton iteration, and how the friction term joins the total potential alongside the barrier. Once those three connections are stated, the [four IPC components from Ch 01](../01-ipc-internals/03-energy.md) — [barrier](../01-ipc-internals/00-barrier.md), [adaptive $\kappa$](../01-ipc-internals/01-adaptive-kappa.md), [CCD line search](../01-ipc-internals/02-ccd.md), and energy-based composition — augmented with the friction term from this chapter, compose into one minimization that the Newton loop consumes unchanged.

## The total potential with friction

The [energy-based formulation](../01-ipc-internals/03-energy.md) stated the backward-Euler minimization with a placeholder $D(v(x))$ for friction. Filling in the Li 2020 form:

$$ U_\text{total}(x) = \tfrac{1}{2}(x - \hat x)^\top M (x - \hat x) + h^2 \Psi_\text{elastic}(x) + h^2 \kappa B(x) + h^2 \sum_{k \in \mathcal C} D_k(x) $$

Each friction term is $D_k(x) = \mu_c \, \lambda_k^n \, f_0(\|u_T^k(x)\|)$ with $u_T^k(x) = (T_k^n)^\top (x - x^t)$, written in each pair's own lagged tangent frame. The subscript $k$ indexes the pair; the superscript $n$ marks lagged values — constants within one Newton iteration, not functions of $x$. The sum $\sum_{k \in \mathcal C}$ is over the same active-contact set $\mathcal C$ the barrier term sums over.

## How $\lambda$ comes from the barrier

At each active pair, Li 2020's normal-force magnitude is the barrier's derivative contribution to the contact reaction, evaluated at the current gap:

$$ \lambda_k = -\frac{\kappa}{h^2} \, \frac{\partial b(d_k, \hat d)}{\partial d_k} $$

This is Li 2020's equation 9. The factor $\kappa/h^2$ inverts the $h^2\kappa$ scaling of the barrier term in $U_\text{total}$; the minus sign converts the barrier-force sign convention to a positive normal-force magnitude (the barrier's derivative $b'(d)$ is negative inside the active band, per the [barrier sub-leaf](../01-ipc-internals/00-barrier.md)). The friction coefficient $\mu_c$ does not depend on $\kappa$; the coupling to $\kappa$ is entirely through $\lambda_k$'s linear dependence. The lagged value $\lambda_k^n$ used in the friction potential is this expression evaluated at the prior iterate's gap $d_k(x^n)$.

This is the single place in the solver where friction and barrier are not independent. Everywhere else, the IPC components are composable — each could be replaced in isolation without touching the others — but $\lambda_k$ links friction magnitude to barrier stiffness through $b'(d_k)$ evaluated at the pair's current gap. The adaptive-$\kappa$ schedule's step-size changes therefore flow through into friction magnitude on the next lagged update.

## Why lagging, not live updates

If $T$ and $\lambda$ were recomputed as functions of $x$ every Newton iteration — tangent basis from the current pair geometry, $\lambda$ from $b'(d(x))$ at the current gap — the friction potential $D_k(x)$ would depend on $x$ through three nested paths: the tangential displacement $u_T(T(x), x)$, the basis $T(x)$ directly, and the normal force $\lambda_k(x)$. The Hessian would couple all three paths, introducing off-diagonal blocks between friction and barrier and destroying the sparsity pattern the [BSR-3 assembly](../../80-gpu/01-sparse-matrix/01-bsr.md) relies on. Worse, the tangent basis $T(x)$ has discontinuities at closest-feature transitions (when the closest point on a triangle moves from the interior to an edge to a vertex, or when an edge-edge pair switches to a different edge), and propagating those jumps through the Hessian would break the $C^2$ property the Newton loop requires.

Lagging breaks this cycle: $T_k^n$ and $\lambda_k^n$ are held constant from the prior iterate (within a timestep) or the prior timestep (initial value within a step). Under lagging, $D_k(x)$ depends on $x$ only through the single smooth path $u_T^k(x) = (T_k^n)^\top(x - x^t)$, and the $C^2$ argument from the [previous sub-leaf](01-smoothed.md) applies without qualification. After each Newton solve, $T_k^n$ and $\lambda_k^n$ are refreshed from the new iterate and the next Newton solve runs against the updated lagged values; the outer loop over lagged values is the "lagged iteration" of Li 2020 Section 5.4–5.5.

## Scope caveat: lagging has no convergence proof

Li 2020 is explicit that the lagged iteration "does not have convergence guarantees" (Section 5.5), and that "in our large-deformation frictional examples we apply just a single lagging iteration" as the practical default. The barrier term preserves its non-penetration and $C^2$ guarantees under lagging — lagging is a property of friction alone — but the outer-loop fixed-point for $T_k^n, \lambda_k^n$ is empirical rather than proven. `sim-soft` inherits this scope: friction is correct at the per-Newton-iterate level, the lagged outer loop is run for a fixed number of passes (default: 1 in production, 3–5 in high-accuracy modes), and the book does not claim friction is solved to the same bound as the barrier. The [Part 6 Ch 01 implicit-function theorem](../../60-differentiability/01-custom-vjps.md) on which differentiability rests treats the converged state as if lagging had reached fixed point; practitioners who need tighter friction convergence run more lagging passes and budget the cost accordingly.

## What composes

Substituting the Li 2020 friction form into $U_\text{total}$ gives the full incremental potential in one line:

$$ U_\text{total}(x) = \underbrace{\tfrac{1}{2}(x-\hat x)^\top M(x - \hat x)}_\text{inertia} + \underbrace{h^2 \Psi_\text{elastic}(x)}_\text{material} + \underbrace{h^2 \kappa B(x)}_\text{barrier} + \underbrace{h^2 \sum_k \mu_c \lambda_k^n f_0(\|u_T^k(x)\|)}_\text{friction} $$

The CCD line search ([Ch 01 CCD sub-leaf](../01-ipc-internals/02-ccd.md)) filters Newton steps against the barrier's non-penetration constraint; a step that crosses multiple pairs' transition seams is still a valid step because $f_1$ is $C^1$, so CCD's step-bound logic cares only about barrier-controlled penetration, not friction. The adaptive-$\kappa$ schedule ([Ch 01 adaptive-$\kappa$](../01-ipc-internals/01-adaptive-kappa.md)) updates $\kappa$ each Newton iterate; friction sees the updated $\kappa$ through $\lambda_k$'s linear dependence on the next lagged refresh.

This is the decisive claim of Part 4: contact and friction are not special cases in the Newton solver. They are two more energies in the sum, with well-defined gradients and Hessians assembled into the same sparse structure as elasticity and inertia. The same machinery solves them all.

## What this sub-leaf commits the book to

- **Friction's tangent basis and normal force are lagged, not live.** `sim-soft`'s `FrictionPair::update_lagged(x)` is called between Newton solves, not within them. The `max_lagging_iterations` knob defaults to 1 (Li 2020's production default) and can be raised to 3–5 for high-accuracy modes; the lagging strategy is not user-tunable at the API level beyond this iteration count.
- **$\lambda_k$ is derived from the barrier, not stored separately.** The friction term reads $b'(d_k)$ at each active pair via the same element-level distance-function interface the barrier uses. There is no independent normal-force computation and no separate friction-vs-barrier reconciliation code path.
- **Barrier and friction compose in one potential, assembled into one sparse Hessian.** `sim-soft`'s Newton loop does not distinguish friction blocks from barrier blocks from elastic blocks at assembly time; all three go through the same [Part 8 BSR-3 assembly](../../80-gpu/01-sparse-matrix/01-bsr.md) with the same $3 \times 3$ block size. The contact API boundary is `ContactBody` returning an energy, a gradient, and a Hessian; friction plugs into the same trait as barrier and is indistinguishable to the solver.
