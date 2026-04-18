# Milestone — first working sim-soft

At the end of Phase D, `sim-soft` is a working end-to-end system: a compliant cavity conforms to a rigid probe, the IPC barrier holds contact without penetration, the solver converges to equilibrium in a few Newton iterations, and the reward landscape from [Part 1 Ch 01](../10-physical/01-reward.md) has a gradient that passes a 5-digit finite-difference gradcheck. The system runs on CPU with no GPU code. The design space is a synthetic 2-parameter cut — small enough that finite differences are tractable as a debugging tool, rich enough that the reward is not trivially linear.

This is the book's "first working" milestone per [Part 11 Ch 03](../110-crate/03-build-order.md#the-committed-order). It is the first moment `sim-soft` stops being a design study and starts being a running system. Every subsequent milestone improves something that already works.

| Section | What the milestone guarantees |
|---|---|
| [Neo-Hookean + IPC on CPU](02-first-working/00-cpu.md) | Forward solve converges on the canonical problem; per-step wall time lands inside the Phase D CPU budget; IPC holds no-penetration |
| [Gradcheck passing](02-first-working/01-gradcheck.md) | IFT gradient on fixed-topology mesh agrees with central-difference to 5 digits; gradcheck suite runs on every PR from this phase onward |

Three scope limits this milestone honors without apology: no GPU (Phase E), no SDF authoring live (Phase G), no visual layer beyond a diagnostic wireframe (Phase I). A Phase D demo looks like numbers converging in a terminal, not a rendered gel. [Part 11 Ch 03](../110-crate/03-build-order.md#the-committed-order) defends the scope; the thesis's fused visual-and-physical standard is not abandoned here, it is sequenced.
