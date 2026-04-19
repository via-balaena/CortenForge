# Why fully implicit

The scheme choice is a binding commitment for everything downstream: [Ch 01 Newton iteration](01-newton.md) iterates on whatever time-discretization this leaf picks, [Ch 02 adaptive timestep](../02-adaptive-dt.md) shrinks whatever step length this leaf declares nominal, and [Part 6 Ch 02's IFT adjoint](../../60-differentiability/02-implicit-function.md) re-uses the exact Hessian this leaf's scheme produces in forward. Picking explicit would propagate a different set of requirements through three downstream parts, so the scheme is picked here, once, and narrowly. This leaf argues the *implicit* part of "backward Euler with Newton"; [01-newton.md](01-newton.md) argues the *backward-Euler* and *Newton* parts.

## The explicit stability bound

Explicit time integration on an elastic continuum is conditionally stable: the timestep cannot exceed a mesh-and-material bound set by the fastest wave that fits in an element. For a linear elastic medium discretized with characteristic element size $h$ and material wave speed $c$, the Courant–Friedrichs–Lewy (CFL) condition requires

$$ \Delta t_\text{explicit} \lesssim \frac{h}{c}, \qquad c = \sqrt{E/\rho} $$

with $E$ the elastic modulus and $\rho$ the mass density. The inequality comes from requiring that the numerical domain of dependence contains the physical one — information travels at most one element per step, otherwise the discrete scheme cannot represent the true solution and the iteration diverges. The same argument extends to geometrically nonlinear hyperelastic materials with $E$ replaced by a local tangent modulus, which for neo-Hookean silicone is within a small factor of the small-strain value until stretches become extreme.

The bound is tight in practice. For Dragon Skin 20A ($E \approx 1$ MPa, $\rho \approx 1050$ kg/m³ — see the [mechanical-data leaf](../../10-physical/04-material-data/01-dragonskin.md)) at a representative element of $h = 3$ mm, $c \approx 30$ m/s and $\Delta t_\text{explicit} \lesssim 100$ μs. For stiffer or finer-meshed variants the bound tightens proportionally; for softer Ecoflex grades at the same mesh it relaxes by roughly $\sqrt{E_\text{dragonskin}/E_\text{ecoflex}} \approx 4$, landing near 400 μs. The canonical frame-rate budget is 16 ms (60 Hz), so the explicit scheme would need dozens to hundreds of substeps per frame at these reference values, with the count rising further at finer mesh resolutions or stiffer material grades.

That substep count is not automatically disqualifying — explicit schemes are cheap per-step (no linear solve), and some regimes profitably absorb the substep count in exchange. The regime this study operates in does not: the [Part 4 IPC contact](../../40-contact/00-why-ipc.md) commitment requires a $C^2$ total potential evaluated together with elasticity inside one Newton iteration, which is incompatible with stepping elasticity explicitly and contact implicitly (see Claim 1 in the [parent spine page](../00-backward-euler.md)). Once contact needs implicit treatment for non-negotiable architectural reasons, the question shifts from "can we afford implicit elasticity" to "can we afford a split scheme that carries a different solver for each physics." The answer is covered under [Alternatives rejected](../00-backward-euler.md#alternatives-rejected) in the spine page.

## Backward Euler is A-stable

Backward Euler on the semi-discrete ODE $M \ddot x = -\nabla \Psi(x)$ writes

$$ M\, \frac{v_n - v_{n-1}}{\Delta t} = -\nabla \Psi(x_n), \qquad x_n = x_{n-1} + \Delta t\, v_n $$

and is unconditionally A-stable for linear stiffness — the stability region covers the entire left half of the complex $\lambda \Delta t$ plane, so no linearized wave mode can destabilize the iteration regardless of $\Delta t$. The nonlinear hyperelastic case does not exhibit the linear-case's closed-form A-stability, but it also does not exhibit timestep-dependent blow-up under the Lipschitz-tangent regularity that [Part 4 IPC barrier smoothness](../../40-contact/01-ipc-internals/00-barrier.md) separately requires and that neo-Hookean and Mooney-Rivlin elasticity satisfy away from the compression-to-zero limit. Explicit schemes, by contrast, exhibit linear-case blow-up under $\Delta t > h/c$ and the corresponding stiffness-scaled pathology in the nonlinear case.

Concretely, `sim-soft` runs at $\Delta t = 16$ ms as the nominal graphics-aligned step, with [Ch 02 adaptive shrinking](../02-adaptive-dt.md) taking over when a Newton iteration fails to converge. At that step size the scheme is roughly two orders of magnitude over the explicit bound at the stiff-material end of the ladder — no explicit scheme, tuned, could close that gap. The unconditional stability is not free; it is paid for in Newton iteration cost per step ([Ch 01](01-newton.md)) and in the factorization + back-substitution cost that a linear solve imposes. But the cost comes out favorably for the scene scales `sim-soft` targets: a handful of Newton iterates with one sparse Cholesky factor per iterate beats hundreds of explicit substeps by a wide margin in wall-clock, with the margin widening as material stiffness increases. Concrete benchmarks land with the [Phase B build-order deliverable](../../110-crate/03-build-order.md#the-committed-order).

## Numerical damping as feature, not bug

Backward Euler introduces artificial numerical damping at high frequencies. For a linear oscillator at natural frequency $\omega$, the per-step amplitude decay factor is approximately $1/(1 + (\omega \Delta t)^2/2)$ for small $\omega \Delta t$, and tends to zero as $\omega \Delta t \to \infty$. High-frequency modes are not conserved: they decay. In a fully energy-conservative Hamiltonian system this would be a flaw — symplectic schemes like Verlet conserve the high-frequency content exactly, which is why they dominate long-time molecular-dynamics simulation.

The soft-body regime is not that regime. Friction ([Part 4 Ch 02](../../40-contact/02-friction.md)), viscoelasticity ([Part 2 Ch 07](../../20-materials/07-viscoelastic.md)), and dissipative contact (IPC barrier + friction) together mean the physical system is not Hamiltonian — there is no exact energy to conserve, because real silicone loses energy per cycle. Backward Euler's numerical damping is roughly aligned with the physical dissipation regime: high-frequency mesh-basis modes above the resolved physical frequency get damped out, which works in the same direction as (though is not a substitute for) the [Prony-series viscoelastic damping](../../20-materials/07-viscoelastic/00-prony.md) the constitutive law ships. The [Part 1 Ch 02 wobble failure](../../10-physical/02-what-goes-wrong/01-wobble.md) specifically targets missing *physical* viscoelastic damping and is closed only by the Prony fit — not by numerical damping — but backward Euler's damping of unresolved high-frequency content means that the scheme does not manufacture additional oscillatory artifacts beyond what the physical model predicts.

The damping is not free either — at excessive step sizes it damps the *physical* low-frequency modes too, softening the simulated response below what the constitutive law prescribes. Keeping $\Delta t$ at or below the frame-rate scale (16 ms at 60 Hz, 8 ms at 120 Hz) stays within the regime where the damping acts on high-frequency artifacts only, and the [Ch 02 adaptive layer](../02-adaptive-dt.md) prevents the step from ballooning beyond that.

## What this closes off

Three scheme families drop out by this leaf's argument:

- **Explicit (Verlet, RK4, central-difference):** the CFL bound above makes them untenable at the substep counts Phase B's mesh resolutions demand. The [Part 4 contact coupling](../../40-contact/00-why-ipc.md) independently rules them out at the physics level even at acceptable substep counts.
- **Semi-implicit / IMEX splitting:** explicit elasticity + implicit contact, or vice versa. Rejected on the physics side — see Claim 4 in the [spine page](../00-backward-euler.md) — because the elasticity-and-contact coupling through the Newton Hessian is the mechanism that kills [popping from Part 1 Ch 02](../../10-physical/02-what-goes-wrong/02-popping.md), and splitting re-introduces that failure.
- **Higher-order BDF (BDF2, BDF3):** rejected in the [spine page's alternatives section](../00-backward-euler.md#alternatives-rejected) on cost-vs-benefit grounds, not on stability. BDF2 is A-stable and would be a reasonable choice for a study where constitutive-law fidelity was the accuracy ceiling; it is not `sim-soft`'s regime.

[Ch 01 projective dynamics](../01-projective.md) is the structural-incompatibility case — it shares the implicit framing but rejects the full Newton iteration and the IFT re-use story, and so gets its own chapter rather than being merged into the alternatives list above.

## What this commits

- $\Delta t_\text{nominal} = 16$ ms as the graphics-aligned default, with [Ch 02 adaptive halving](../02-adaptive-dt.md) on Newton failure. No explicit fallback path.
- High-frequency numerical damping is expected and wanted. Any [Part 11 Ch 04 regression test](../../110-crate/04-testing/01-regression.md) checking energy conservation has to account for this — the total energy drift per step is not a bug signal unless it exceeds the Prony-series dissipation by a meaningful margin.
- The implicit-scheme commitment is upstream of every Newton-iteration detail in [Ch 01](01-newton.md) and every line-search detail in [02-line-search.md](02-line-search.md) — those two leaves take this leaf's scheme choice as given and build the solver on it.
