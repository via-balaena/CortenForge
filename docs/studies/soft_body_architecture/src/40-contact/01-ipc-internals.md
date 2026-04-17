# IPC internals

This chapter covers how IPC actually works, one sub-chapter per named component. The high-level shape is: IPC adds a contact term to the total potential energy the solver already minimizes, so contact becomes one more energy in the Newton loop from [Part 5 Ch 00 — Backward Euler](../50-time-integration/00-backward-euler.md). The four components below are what make that one sentence correct.

| Section | Component | Role |
|---|---|---|
| [Barrier function](01-ipc-internals/00-barrier.md) | $b(d, \hat d) = -(d - \hat d)^2 \ln(d / \hat d)$ for $0 < d < \hat d$, zero otherwise. $C^2$ continuous; diverges as $d \to 0^+$; zero energy and zero force at $d = \hat d$ | The energy that replaces penalty force with a smooth divergent potential |
| [Adaptive stiffness schedule](01-ipc-internals/01-adaptive-kappa.md) | Scalar multiplier $\kappa$ on the barrier; grown progressively across Newton iterations if the gap closes too fast | Lets the barrier be steep enough to enforce non-penetration without making the first Newton step diverge |
| [Continuous collision detection](01-ipc-internals/02-ccd.md) | Per-step search for the earliest time a pair of primitives (point-triangle, edge-edge) first reach distance $\hat d$; uses interval arithmetic or analytic CCD | Replaces discrete snapshot collision tests — guarantees no primitive crosses another between timesteps |
| [Energy-based formulation](01-ipc-internals/03-energy.md) | Contact contribution $E_\text{contact}(x) = \kappa \sum_{i \in \text{pairs}} b(d_i(x), \hat d)$ added to total potential; gradient is analytical via the chain rule on $d(x)$ | What makes IPC a drop-in energy term for the Newton-on-potential solver |

Two claims Ch 01 rests on:

1. **IPC is four pieces composed.** Each is independently replaceable — a different smooth barrier shape, a different stiffness schedule, a different CCD algorithm, a different energy integration — but the composition is what makes the method work. The [energy-based formulation sub-chapter](01-ipc-internals/03-energy.md) is where the composition closes: once contact is expressed as an added energy term, every piece of the solver below it (Newton loop, sparse factorization, autograd VJP) inherits correctness automatically.
2. **The barrier's specific form is not arbitrary.** $b(d, \hat d) = -(d - \hat d)^2 \ln(d/\hat d)$ has exactly the three properties IPC needs: (a) it diverges as $d \to 0^+$ so non-penetration is enforced by infinite energy rather than by a projection; (b) it is $C^2$ everywhere on $(0, \hat d]$ so the Newton tangent is well-defined; (c) it has $b(\hat d) = 0$, $b'(\hat d) = 0$, and $b''(\hat d) = 0$, so turning contact on and off at the tolerance boundary does not introduce a discontinuity in energy or force. Swapping the barrier for any function that lacks one of these properties breaks something specific downstream — the Newton solver, the autograd VJP, or the warm-start across Newton iterations.
