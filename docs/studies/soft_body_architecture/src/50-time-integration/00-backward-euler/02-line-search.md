# Line search backtracking

Newton's method is locally quadratic-convergent inside the basin of attraction and undefined outside it. For soft-body simulation with IPC contact, every timestep's starting iterate — the inertial predictor $\widehat x_n = x_{n-1} + \Delta t\, v_{n-1}$ — sits outside the basin with non-trivial frequency. Contact pairs that were inactive at $x_{n-1}$ enter the active set partway through the step, friction modes switch between stick and slip, and the elastic tangent transits indefinite regions under large deformation. A globalization strategy is not optional; [01-newton's](01-newton.md) unmodified Newton step would diverge on a substantial fraction of Phase B's canonical-problem starting iterates.

`sim-soft`'s globalization is **Armijo backtracking line search on $U_n$, composed with the [IPC CCD step-length clip](../../40-contact/01-ipc-internals/02-ccd.md)**. One subroutine, two concerns, three distinct failure signals.

## Armijo on a scalar objective

At iterate $x^{(k)}$ with Newton direction $\Delta x^{(k)} = -(H^{(k)})^{-1} g^{(k)}$, the Armijo condition is

$$ U_n(x^{(k)} + \alpha \Delta x^{(k)}) \le U_n(x^{(k)}) + c_1\, \alpha\, (g^{(k)})^T \Delta x^{(k)} $$

with $c_1 \in (0, 1)$ a user-tunable slack parameter (small, conventionally $10^{-4}$). The left-hand side is the value of the total potential after a step of length $\alpha$; the right-hand side is the value at $x^{(k)}$ plus a fraction of the predicted linear decrease. Backtracking: start $\alpha = 1$, check the condition, if it fails set $\alpha \leftarrow \alpha/2$ and retry. Accept the first $\alpha$ that satisfies it.

Two properties make Armijo-on-$U_n$ the correct merit-function choice for this solver, as opposed to the alternative merit function $\|r_n\|^2/2$ with $r_n = \nabla U_n$:

- **Framing consistency with the minimization commitment.** [Spine Claim 1](../00-backward-euler.md) commits to the per-step problem as minimization of $U_n$, and $U_n$ is also the physically meaningful scalar (the total potential energy the iteration is descending). Monitoring $U_n$'s per-halving decrease gives a direct measurement of physical progress toward equilibrium, and it admits the standard Wolfe-condition and trust-region extensions textbook optimization ships with. Under the SPD-projected Hessian from [01-newton](01-newton.md) step 3, $\|r_n\|^2/2$ would also work as a merit function — the Newton direction descends both $U_n$ and $\|r_n\|^2/2$ under SPD $H$ — but the $\|r_n\|^2/2$ choice decouples the line-search merit from the physical problem, loses the minimization-framing interpretability per bracket, and becomes clumsier to extend with curvature conditions.
- **Small-$\alpha$ acceptance is unconditional.** Because $H^{(k)}$ is the SPD-projected Newton Hessian, $\Delta x^{(k)} = -(H^{(k)})^{-1} g^{(k)}$ satisfies $(g^{(k)})^T \Delta x^{(k)} < 0$ — the Newton direction strictly decreases $U_n$ at first order. Expanding $U_n(x^{(k)} + \alpha \Delta x^{(k)})$ to first order gives $U_n(x^{(k)}) + \alpha (g^{(k)})^T \Delta x^{(k)} + O(\alpha^2)$; since $(g^{(k)})^T \Delta x^{(k)} < 0$ and $c_1 < 1$, the Armijo inequality is satisfied for all $\alpha$ small enough — the $(1 - c_1) \alpha |g^T \Delta x|$ slack absorbs the $O(\alpha^2)$ curvature term. Backtracking therefore terminates in finite halvings whenever the Newton direction is a descent direction on $U_n$, which the SPD projection guarantees.

The first property is what makes Armijo-on-$U_n$ the right choice under the minimization framing; the second is what makes the backtracking loop itself finite. Together they are what makes the line-search loop trivially terminating in the nominal regime; termination under contact-pair insertions, sudden friction-mode switches, and similar discrete changes to $U_n$ is where the next concern enters.

## CCD clip is a line-search pre-condition

A Newton step's $\alpha = 1$ can send a node across a contact barrier — from $d > \hat d$ (out of IPC's active set) to $d < 0$ (interpenetration). IPC's total potential is $+\infty$ at $d \le 0$, so a full Newton step that tunnels through the barrier lands at an infinite $U_n$, the Armijo check always rejects, and backtracking reduces to the first $\alpha$ that barely avoids the barrier — wasted work and near-zero step progress per Newton iterate.

The [continuous collision detection](../../40-contact/01-ipc-internals/02-ccd.md) check prevents the waste by computing, before Armijo runs, the time-of-impact $\text{toi} \in [0, 1]$ along the line segment $x^{(k)} \to x^{(k)} + \Delta x^{(k)}$ — the largest step fraction at which no pair has crossed zero gap. The line-search routine caps $\alpha$ at

$$ \alpha \le 0.5 \cdot \text{toi} $$

before the Armijo loop starts. The 0.5 factor is IPC's standard safety margin — it keeps the post-step iterate strictly inside the barrier's active band, so the subsequent Armijo evaluations see a finite, well-behaved $U_n$. With the CCD cap in place, Armijo backtracking settles on a step that satisfies both descent-on-$U_n$ and no-barrier-crossing in one subroutine.

This composition is what [spine page Claim 4](../00-backward-euler.md) calls "line search is also an IPC safety mechanism." The same backtracking loop handles both concerns, and a failed line search — one that exhausts its retry budget without satisfying Armijo — carries a specific signal about which concern caused the failure.

## Three failure signals

The line-search subroutine returns a triple $(\alpha, \text{success}, \text{reason})$ where $\text{reason} \in \{\text{ok}, \text{armijo-stuck}, \text{ccd-clipped-to-zero}\}$.

- **`armijo-stuck`:** backtracking exhausted its retry budget without decreasing $U_n$ adequately. This is the classical Newton-line-search failure mode and typically signals that $x^{(k)}$ sits outside the basin of attraction of any equilibrium reachable from here at the current $\Delta t$. The calling [Newton loop](01-newton.md) responds by signalling [Ch 02 adaptive-dt](../02-adaptive-dt.md) to halve the timestep and restart the step from $x_{n-1}, v_{n-1}$.
- **`ccd-clipped-to-zero`:** CCD returned $\text{toi} < \epsilon_\text{ccd}$ — the full Newton step was heading into a barrier essentially immediately. This signals that the current iterate is at a pathological boundary configuration where the Newton direction is incompatible with the contact constraints. Response is the same as `armijo-stuck` — halve $\Delta t$ and restart. [Ch 02](../02-adaptive-dt.md) treats both failures as one trigger because the remediation is identical.
- **`ok`:** Armijo satisfied, $\alpha$ returned to the Newton loop for the update step.

The backtracking retry budget is a small integer — the `sim-soft` default tracks the IPC Toolkit's default and is tunable in the solver config; exceeding it is the `armijo-stuck` signal. The number itself is not load-bearing — what matters is that the failure path exists as a named branch rather than an infinite loop, so the adaptive layer above can react to it.

The energy-monitor fallback [described in Ch 02](../02-adaptive-dt/00-energy.md) is the *backstop* for the rare case where CCD is disabled (e.g., for a debug run) and the line search allows an energy-violating step through undetected. In the nominal Phase B configuration with CCD enabled, `armijo-stuck` and `ccd-clipped-to-zero` are the only paths by which the timestep shrinks.

## Why not separate contact-and-elasticity line searches

An alternative globalization splits the line search across physics: one inner loop backtracks on elasticity's potential, a separate outer loop handles contact as a projection step. This split is what [projective dynamics](../01-projective.md) and XPBD do, and it has a legitimate cost advantage — each inner loop can use a cheaper linear solver tailored to its sub-problem.

It is rejected for `sim-soft` because the coupling [Ch 00 spine Claim 4](../00-backward-euler.md) argues is load-bearing: contact and elasticity negotiate step length *through the same Armijo test on the same combined $U_n$*. A split globalization solves each sub-problem correctly but loses the global descent guarantee on the sum — the elasticity sub-step can move a vertex into the contact barrier's active band, which the contact projection then pushes back out, which the elasticity sub-step then re-enters, and the loop oscillates. This is the [popping failure](../../10-physical/02-what-goes-wrong/02-popping.md) from Part 1 Ch 02 leaking back in at the line-search level; [Part 4 Ch 00](../../40-contact/00-why-ipc.md)'s argument for unified contact-and-elasticity is structurally the same at the step-length scale as at the potential scale.

## What this commits

- One line-search subroutine handling both Armijo-on-$U_n$ and CCD-clip. No split between contact-safety and descent-monotonicity checks.
- Newton-loop fallback to [Ch 02 adaptive-dt](../02-adaptive-dt.md) is triggered by line-search failure (either `armijo-stuck` or `ccd-clipped-to-zero`), not by an independent stability monitor.
- The [CCD routine from Part 4 Ch 01](../../40-contact/01-ipc-internals/02-ccd.md) runs inside every Newton step inside every timestep. Its per-step cost is amortized against the Newton iteration count — a well-globalized iteration converges in a handful of Newton iterates per step in the nominal regime, so CCD runs a handful of times per step rather than once per timestep.
- The `ok` / `armijo-stuck` / `ccd-clipped-to-zero` return signal is the contract this leaf exposes to the Newton loop. Ch 02's adaptive layer reads this signal to decide whether to halve $\Delta t$ and restart.
