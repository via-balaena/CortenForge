# FEBio — biomechanics FEM

Repo: [`github.com/febiosoftware/FEBio`](https://github.com/febiosoftware/FEBio) · MIT · v4.12 (2026-02-25) · Primary paper: Maas, Ellis, Ateshian, Weiss 2012 "FEBio: Finite Elements for Biomechanics," *Journal of Biomechanical Engineering* 134(1):011005 (venue + year + authors per common citation; DOI soft-deferred to Pass 2) · Accessed: 2026-04-20

FEBio is "a nonlinear finite element solver that is specifically designed for biomechanical applications" (repo README). Extensive hyperelastic constitutive catalog — Arruda-Boyce, Mooney-Rivlin, Neo-Hookean (coupled + uncoupled), Ogden (coupled + uncoupled), Holzapfel-Gasser-Ogden, Gent, Holmes-Mow, Veronda-Westmann, Porous Neo-Hookean, Large-Poisson's-Ratio Ligament, transversely isotropic, orthotropic — plus viscoelastic, biphasic, biphasic-solute, and triphasic/multiphasic formulations for tissue mechanics (all per User Manual v4.9 §4). Implicit Dynamics Solver (§8.4) for time-dependent problems.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `partial`. The biomechanics variant of axis 1: extensive hyperelastic models, viscoelastic, biphasic; implicit integration; measured-data biomechanics calibration. Contact uses Sliding Interfaces / Tied Interfaces / Rigid Wall Interfaces — the specific algorithmic method (mortar vs penalty vs augmented Lagrangian) is not independently verified in the 2026-04-20 fetch (section titles only, not body). **Not IPC-grade non-penetration**. Missing the IPC component of axis 1.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `—`. Offline biomechanics research focus; no integrated shader pipeline. Visualization via ParaView or PostView (separate tools).
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `—`. Offline CPU. No real-time design goal; biomechanics research runs accept minutes-to-hours per simulation.
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `—`. No AD. Biomechanics inverse problems are handled via external wrappers (parameter-sweep + optimization libraries), not built-in autograd.

## What `sim-soft` inherits or learns

FEBio is the biomechanics reference for **axis 1's material-model breadth**. [Part 2 Ch 04 — hyperelastic](../../20-materials/04-hyperelastic.md), [Part 2 Ch 06 — anisotropic](../../20-materials/06-anisotropic.md), and [Part 2 Ch 07 — viscoelastic](../../20-materials/07-viscoelastic.md) trace constitutive-model choices to the same literature FEBio implements. `sim-soft`'s relationship is as an **algorithmic reference** for the material-model catalog (specific formulations re-implemented in Rust from the same papers) and as a **regression baseline** for hyperelastic correctness on shared biomechanics test scenes. Where `sim-soft` diverges: [IPC-grade contact](../../40-contact/00-why-ipc.md) (FEBio's sliding interfaces are constraint-based), GPU execution, and differentiability.

## Citation status

- Repo + license + release cadence (v4.12 on 2026-02-25) confirmed from README.
- Primary paper: Maas, Ellis, Ateshian, Weiss 2012 "FEBio: Finite Elements for Biomechanics" — author + title + year per common citation; *J. Biomech. Eng.* 134(1):011005 venue + page numbers **soft-deferred to Pass 2** (not independently fetched).
- Contact-formulation specific method — soft-deferred to Pass 2 (section titles "Sliding Interfaces / Tied Interfaces / Rigid Wall Interfaces" confirmed; algorithmic-method body — mortar vs penalty vs augmented Lagrangian — not fetched).
- Hyperelastic catalog — confirmed from User Manual v4.9 §4 (extensive list cross-checked).
