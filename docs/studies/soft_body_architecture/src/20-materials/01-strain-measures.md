# Strain measures

Before naming constitutive laws, Part 2 names the arguments those laws take. Every constitutive law in this Part is a function of some strain measure derived from the deformation gradient $F$; different laws prefer different measures for different reasons. This chapter fixes the definitions once so later chapters can refer to $F$, $C$, $E$, and the isotropic invariants $I_1, I_2, I_3$ without redefining them each time.

| Section | Strain measure | Used by |
|---|---|---|
| [Deformation gradient $F$](01-strain-measures/00-F.md) | $F_{ij} = \partial x_i / \partial X_j$ — the primary kinematic object, mapping reference to deformed configuration | Everything downstream; neo-Hookean energy directly; Piola stress |
| [Right Cauchy-Green $C$](01-strain-measures/01-C.md) | $C = F^T F$ — symmetric, rotation-invariant, positive-definite; the natural argument for invariant-based energies | Mooney-Rivlin, Ogden, HGO; all invariant-formulated hyperelastic laws |
| [Green strain $E$](01-strain-measures/02-E.md) | $E = \tfrac{1}{2}(C - I)$ — the finite-strain generalization of the infinitesimal strain tensor | Saint Venant–Kirchhoff; some corotational formulations; biomech literature by default |
| [Invariants $I_1, I_2, I_3$](01-strain-measures/03-invariants.md) | $I_1 = \mathrm{tr}\, C$, $I_2 = \tfrac{1}{2}((\mathrm{tr}\, C)^2 - \mathrm{tr}\, C^2)$, $I_3 = \det C = J^2$ — rotation-invariant scalar summaries | Ogden (through principal stretches); Mooney-Rivlin; anisotropic laws use $I_4 = a \cdot C a$ as an extra |

Two claims Ch 01 rests on:

1. **These are inputs, not outputs.** A constitutive law takes $F$ (or something derived from $F$) and produces stress; the derived strain measures and invariants are plumbing each law writes in terms of. Mixing them up — writing a law in terms of $E$ when $C$ would be clearer, using $F$ directly when an invariant form would enforce rotation-invariance structurally — is a common source of bugs in hand-rolled FEM code. The `Material` trait signature ([Ch 00](00-trait-hierarchy.md)) is deliberately in terms of $F$ so each implementation writes its derivation once against the primary kinematic.
2. **Rotation-invariance comes from invariants.** Any isotropic hyperelastic energy must be a function of $I_1, I_2, I_3$ (or equivalents like principal stretches) — this is a physical requirement, not a convenience. An energy written as $\psi(F)$ that is not expressible as $\psi(I_1, I_2, I_3)$ is not frame-indifferent, predicts fictitious stress under pure rotation, and fails the [rotation-invariance gradcheck](../110-crate/04-testing/03-gradcheck.md) every `Material` implementation is required to pass. This is the structural reason [corotational formulations](03-corotational.md) are a workaround rather than a first-class constitutive law.
