# What existing solvers get wrong

Before the book proposes the stack it proposes, it catalogs what is wrong with the stacks that already exist. Each failure below is a regression test for anything `sim-soft` ships. If the new solver cannot strictly improve on MuJoCo flex, XPBD cloth, mass-spring networks, or linear-FEM pipelines like SOFA on every row in this table, there is no reason to have built it.

The six failures below are not a complete bestiary of soft-body simulation defects — rate-dependent yielding, self-contact failure in multi-layer sleeves, and thermal drift during long runs all deserve their own rows and get them in later parts. The six here are the ones that show up on the canonical problem from Ch 00 under realistic loading, with off-the-shelf solvers as they ship today. Each is reproducible in an afternoon by any reader with a solver in each listed category and the patience to watch the result.

| Failure mode | Seen in | Breaks which reward term |
|---|---|---|
| [Volume loss under squeeze](02-what-goes-wrong/00-volume-loss.md) | Linear FEM, XPBD, penalty contact under stiff constraints | Effective stiffness, peak pressure, uniformity (indirectly — lost volume redistributes unevenly) |
| [Wobble and unphysical jelly feel](02-what-goes-wrong/01-wobble.md) | Position-based dynamics, mass-spring, under-damped explicit integrators | Time-to-equilibrium on the conformity reward; steady-state is not reached |
| [Popping and snap-through](02-what-goes-wrong/02-popping.md) | Penalty contact, SDF-fallback contact, impulse-based | Coverage (spurious loss of contact), uniformity (instantaneous spikes at the pop) |
| [Missing friction](02-what-goes-wrong/03-no-friction.md) | Real-time engines defaulting to frictionless, ad-hoc Coulomb without stick | Sleeve slides off probe under axial load; no stable conformity configuration exists |
| [Poor rim / edge deformation](02-what-goes-wrong/04-rim.md) | Coarse meshes, linear tets, pipelines without adaptive refinement | Peak pressure (localized spike at the rim), uniformity near the entry band |
| [Material parameter poverty](02-what-goes-wrong/05-material-poverty.md) | Uniform single-stiffness soft bodies, no viscoelasticity, no anisotropy | Every reward term, because the wrong material cannot be optimized to the right behavior |

Two claims Ch 02 rests on:

1. **Each failure corrupts the reward.** The third column above is the key one. A defect that is visible but does not corrupt the reward is a cosmetic complaint and does not belong in this chapter; a defect that corrupts the reward is a physical error that propagates through Part 10's optimization loop as bias the optimizer cannot see past. All six failures in the table are of the second kind.
2. **Each failure has a corresponding subsystem in Parts 2–5.** Volume loss → near-incompressibility (Part 2 Ch 05) and hyperelastic constitutive laws (Part 2 Ch 04). Wobble → Prony-series viscoelasticity (Part 2 Ch 07) and implicit integration (Part 5). Popping → IPC barrier + CCD (Part 4). Missing friction → smoothed Coulomb integrated with IPC (Part 4 Ch 02). Rim deformation → adaptive refinement (Part 3 Ch 02) and Tet10 elements (Part 3 Ch 00). Material poverty → the material trait hierarchy and SDF-valued spatial fields (Part 2 Chs 00, 09). The rest of the book is not a theoretical taxonomy; it is these six rows, each one answered in turn.

The thesis chapter next door (Ch 03) argues the stronger form of this claim: these six failures are not separate from visual quality, they *are* the visual quality problem, and fixing them physically fixes them visually for free.
