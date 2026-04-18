# Canonical formulation

This chapter pins down the problem the rest of the book keeps returning to: a compliant cavity, closed over an inserted probe, optimized so the interior contact pressure is uniform. The geometry is small enough to draw in a sentence and small enough to simulate on a single GPU, but the problem itself exercises every subsystem the book develops — hyperelasticity at finite strain, IPC contact with friction, near-incompressibility, implicit time integration, differentiability through the steady-state conformity map, and real-time rendering of translucent silicone with per-vertex stress readout.

The canonical problem is not the only problem. It is the one the book uses as its standing test case, the one Parts 2–10 point back to when asking "does this subsystem help, and how much," and the one Part 12's milestones are written against. Domain problems a reader actually cares about (a compliant gripper, a catheter sheath, a wearable seal, or a soft pulsatile pump) differ in geometry and loading but reuse the same stack; the related-problems chapter below names the mapping.

| Section | What it covers |
|---|---|
| [Compliant cavity–probe conformity](00-canonical/00-formulation.md) | Geometry, loading, boundary conditions, state variables, nondimensional groups |
| [Related engineering problems](00-canonical/01-related-problems.md) | Seals, compliant grippers, catheter sheaths, soft-robotic suction cups — one problem, many hats |
| [Why this is a good canonical test case](00-canonical/02-why-canonical.md) | Which of the book's subsystems it exercises, which it deliberately does not, and where it falls silent |

Two claims for the rest of Part 1 to rest on:

1. **The problem is load-bearing for subsystem selection.** Every constitutive law, contact formulation, time integrator, and GPU kernel the book recommends is justified in part by whether it improves the canonical problem's solution. "Improves" means one of: measurable drop in the conformity reward (Ch 01), fix of a Ch 02 failure, or enabling a differentiable path through the solver that was not previously available.
2. **The problem is not the application.** The compliant-cavity-on-probe abstraction is intentionally generic: it matches a sealing gland, a soft gripper's finger, a catheter insertion sheath, a pulsatile compression sleeve, and many other engineered parts. The book does not commit to a single application. Parts 2–11 commit to the physics; Part 12's roadmap commits to the build order; the application layer is downstream of all of it.
