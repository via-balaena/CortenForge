# Fixed-point iteration across substeps

The fixed-point scheme resolves the inconsistency a one-shot handshake produces: each simulator advances with stale boundary data from the other, so the forces and motions each produces may disagree with the coupled-system response where both sides are mutually consistent. The scheme's mechanic is an outer loop — advance each simulator, exchange new boundary data, re-advance with the updated values, repeat until the boundary stabilises within tolerance or a retry cap is hit.

## The scheme

Let tick $n$ denote a handshake boundary at time $t_n$ (handshakes happen at the `sim-soft` rate — 16 ms in experience mode, 1 ms in design mode). Write $\mathbf{b}_\text{rigid}$ for the rigid-boundary data (pose + spatial velocity of each participating rigid body, as exported in [§00 mujoco](00-mujoco.md)) and $\mathbf{f}_\text{contact}$ for the contact-reaction wrench tuple `sim-soft` returns. $\mathbf{b}$ for "boundary" avoids collision with MuJoCo's generalized-coordinate $q$-vector. Between $t_n$ and $t_{n+1}$, with initial interface data $(\mathbf{b}^{(0)}_\text{rigid}, \mathbf{f}^{(0)}_\text{contact})$ carried from tick $n$, the coupling iterates for $k = 0, 1, 2, \ldots$:

1. Advance `sim-soft` from $t_n$ to $t_{n+1}$ with the rigid boundary data $\mathbf{b}^{(k)}_\text{rigid}$ (sim-soft's CCD treats the rigid body as a moving obstacle using the supplied velocity for linear extrapolation across the step). This produces contact reactions $\mathbf{f}^{(k+1)}_\text{contact}$ from the converged Newton step on the extrapolated rigid trajectory.
2. Advance `sim-core` from $t_n$ to $t_{n+1}$ — typically several native rigid-body steps (see §"Stepping-rate mismatch" below) — with the contact wrench held at $\mathbf{f}^{(k+1)}_\text{contact}$ throughout. This produces new rigid boundary data $\mathbf{b}^{(k+1)}_\text{rigid}$ at $t_{n+1}$.
3. Stop if the boundary-data change $\|\mathbf{b}^{(k+1)}_\text{rigid} - \mathbf{b}^{(k)}_\text{rigid}\|$ and $\|\mathbf{f}^{(k+1)}_\text{contact} - \mathbf{f}^{(k)}_\text{contact}\|$ both fall below tolerance, or if $k$ reaches the retry cap; otherwise set $k \leftarrow k + 1$ and repeat.

After stopping, commit both simulators' end-of-step state for tick $n+1$ and proceed.

## Convergence criterion

The stopping tolerance combines a pose-change norm on the rigid side and a force-change norm on the soft side. `sim-soft` accepts the rigid-boundary fluctuation as consistent when the aggregate pose change over an iterate falls below $\epsilon_b$ (order-of-magnitude starting budget: millimetre translation, milliradian rotation — Phase F measurements calibrate the actual threshold) and the contact-wrench change falls below $\epsilon_f$ (a small fraction of the estimated peak reaction-force magnitude). Both norms aggregate in $L^2$ over all interfacing rigid bodies in the scene.

Two retry-budget disciplines apply together. First, a hard iterate cap (budget for how many outer iterates any one tick may consume) prevents a slowly-drifting interface from blocking real-time. Second, if the cap is hit without converging, the step commits the last iterate and flags a `handshake-uncontracted` signal (analogous to the [`armijo-stuck` signal from Part 5 Ch 00 §02 line-search](../00-backward-euler/02-line-search.md)); sustained signalling triggers a timestep shrink via the [adaptive-$\Delta t$ path](../02-adaptive-dt.md) to smaller handshakes that contract more reliably. Shorter handshakes admit less fluctuation per iterate, so a design-mode run with $\Delta t = 1$ ms is expected to contract in fewer iterates than a 16 ms experience-mode run — the Phase F benchmark will confirm.

The scheme is a Gauss-Seidel-type fixed-point iteration on the two-simulator interface residual; [Gravouil and Combescure 2001](../../appendices/00-references/06-co-simulation.md#gravouil-combescure-2001) is the canonical anchor for this class of coupling with differing sub-domain timesteps, and [Kübler and Schiehlen 2000](../../appendices/00-references/06-co-simulation.md#kubler-schiehlen-2000) identifies the algebraic-loop conditions under which the fixed-point converges at all. For `sim-soft`'s contact-dominated regime the relevant stability concern is whether the rigid body moves far enough during one handshake to cross the contact's current barrier-support region; the [CCD-clipping shrinkage](../02-adaptive-dt/01-ccd-shrink.md) on the soft side and `sim-core`'s own step-size discipline jointly bound this, and the `handshake-uncontracted` signal feeds the adaptive-$\Delta t$ path when they do not.

## Iteration count in practice

For the canonical problem — a rigid probe pressed at controlled depth into a compliant cavity — 1–3 outer iterates per handshake is a reasonable starting budget. The first iterate propagates the previous tick's rigid state into `sim-soft` and the resulting contact force into `sim-core`; a second iterate refines both sides with the updated partner data; a third exists as headroom for sharp transients (probe tip first entering contact, a rigid impact event). This is a practitioner starting budget rather than a theorem — the [Gomes et al. 2018](../../appendices/00-references/06-co-simulation.md#gomes-2018) co-simulation survey covers the broader taxonomy of fixed-point and quasi-Newton coupling schemes, and actual iterate counts depend on the stiffness ratio between the two simulators' response, the contact area engaged, and the handshake period. The Phase F deliverable ([Part 11 Ch 03 build order](../../110-crate/03-build-order.md#the-committed-order)) includes the measured iterate-count distribution on the canonical problem, which calibrates the retry cap.

## Stepping-rate mismatch

The two simulators' native timesteps differ. `sim-core` defaults to MuJoCo's 2 ms rigid-body step; contact-heavy scenes reduce this toward 0.5–1 ms. `sim-soft` runs 16 ms in experience mode, 1 ms in design mode. The coupling resolves the mismatch by making the `sim-soft` step the handshake period and letting `sim-core` advance several of its own steps within it:

- Experience mode (16 ms handshake, 2 ms rigid step): `sim-core` advances 8 consecutive rigid-body steps within one handshake.
- Experience mode with sub-ms rigid step (0.5 ms, contact-heavy): 32 consecutive rigid-body steps.
- Design mode (1 ms handshake, 2 ms rigid step): one `sim-core` step would overshoot the handshake. MuJoCo has no internal substep mechanism — each `mj_step()` advances exactly one configured timestep — so design mode configures `sim-core`'s step at ≤1 ms so its native steps subdivide the handshake period evenly (e.g., one 1 ms step, or two 0.5 ms steps, per handshake).

Between handshakes, `sim-core` uses the last-received contact wrench as a constant external force over its native step sequence. This is the standard partitioned-analysis convention — the coupling payload is held piecewise-constant across the subordinate simulator's steps ([Felippa and Park 1980](../../appendices/00-references/06-co-simulation.md#felippa-park-1980)) — and the zeroth-order extrapolation incurs a local truncation error bounded by $O(\dot{\mathbf{f}}\,\Delta t^2)$ in impulse transferred per handshake. For 16 ms handshakes in experience mode this error is expected to sit below the dynamic-range fluctuation of the reward terms in [Part 1 Ch 01](../../10-physical/01-reward.md) and does not require higher-order interpolation; if Phase F measurements show it becomes load-bearing, the fix is to feed `sim-core` a linear-ramp wrench across the handshake at the cost of interpolating the wrench over `sim-core`'s step sequence (per-step interpolation overhead, no extra outer iterates).

"Substep" is not MuJoCo vocabulary — MuJoCo advances one integration step per `mj_step()` call, with no internal substep mechanism. This sub-leaf uses "consecutive steps between handshakes" to avoid confusion with soft-body engines that do have substep machinery.

## Alternatives rejected

**One-shot explicit coupling (no iteration).** Each handshake calls each simulator exactly once with the other's prior state, then commits. Simpler, cheaper, and adequate for weakly-coupled pairs — but `sim-soft`'s contact reaction is stiff (IPC barrier), and a one-shot handshake admits lag-induced drift whenever the rigid body's response to the current tick's contact force changes the contact configuration at next tick. The fixed-point correction is cheap enough (a few iterates) that one-shot is the wrong local optimum.

**Global Newton on the combined system.** Treat the two simulators as one system and run Newton on the joint state. This is the monolithic scheme the [spine rejects on architectural grounds](../03-coupling.md) — it requires absorbing `sim-core`'s pipeline into `sim-soft`'s Newton solver or vice versa, which is off-platform.

**Quasi-Newton / IQN-ILS acceleration.** [Gomes et al. 2018](../../appendices/00-references/06-co-simulation.md#gomes-2018) documents interface quasi-Newton schemes (IQN-ILS and relatives) that build a low-rank Jacobian approximation across iterates and accelerate convergence in high-iterate-count regimes. For `sim-soft`'s 1–3-iterate budget these add bookkeeping with little payoff; they are a Phase H optimisation if the measured iterate count on the canonical problem under Phase F runs higher than expected.

**Synchronised fine-grained handshakes.** Run both simulators at the finer simulator's native rate (e.g., `sim-soft` at `sim-core`'s 0.5 ms). Eliminates the step mismatch but pays the cost of running `sim-soft` up to 32× more frequently than its native budget, for no additional fidelity on the quasi-static canonical regime.

## What this commits

- The coupling scheme is outer-loop Gauss-Seidel fixed-point iteration on the two-simulator interface residual; each simulator retains its native solver internally.
- Convergence is judged framewise on the boundary-data change between successive iterates with retry-budget fallback and an adaptive-$\Delta t$ shrink as the escape hatch.
- Iterate count is 1–3 in steady state; the Phase F benchmark measures the actual distribution on the canonical problem to calibrate the retry cap.
- `sim-core` advances 8–32 native rigid-body steps within one experience-mode handshake; one step per handshake in design mode with `sim-core`'s step tuned to match.
- The handshake's contact wrench is held piecewise-constant across `sim-core`'s steps as the zeroth-order coupling payload, consistent with standard partitioned analysis.
