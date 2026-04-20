# Reference paper list

Every inline name-year citation in the book links to a `#lastname-year` anchor inside one of the nine leaves below. The leaves are organized by topic rather than by date; the topic is the axis readers will most often scan by. Each leaf is an anchored index — one anchor per cited work, a one-to-two-line description of what the work is and what the book uses it for — not a bibliographic treatment. Pass 3 expands each entry to full bibliographic form (venue, DOI, extracted-result summary) and, where relevant, a deeper extract of the specific result the book cites.

The [Part 0 Ch 03 "no hallucinated citations" commitment](../00-context/03-how-produced.md) is binding on this appendix. An entry appears here only if the referenced work was fetched, read, and traced to the specific claim the book uses it for. Pass 1's job is scaffolding — anchors for every inline citation across Parts 1–12 resolve, and each anchor carries enough description that a reader can judge relevance at a glance.

| Leaf | What it covers |
|---|---|
| [IPC papers](00-references/00-ipc.md) | Incremental Potential Contact — the 2020 source paper and its real-time follow-ups. The foundation the [Part 4](../40-contact/00-why-ipc.md) commitment rests on |
| [Hyperelasticity papers](00-references/01-hyperelastic.md) | Neo-Hookean, Mooney-Rivlin, Ogden, Holzapfel-Gasser-Ogden constitutive-law references; silicone curve-fit studies (Ecoflex and Dragon Skin) |
| [Adjoint method papers](00-references/02-adjoint.md) | The implicit-function-theorem adjoint, Revolve checkpointing, SDE adjoints for stochastic forward trajectories; the machinery [Part 6](../60-differentiability/00-what-autograd-needs.md) consumes |
| [Differentiable sim papers](00-references/03-diff-sim.md) | Differentiable meshing (DiffMC, neural marching cubes, DMTet, differentiable Delaunay) and differentiable-physics frameworks. [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md)'s open-problem references live here |
| [Rendering papers](00-references/04-rendering.md) | Subsurface scattering, anisotropic BRDFs, differentiable rendering. [Part 9](../90-visual/00-sss.md)'s visual-layer commitments trace back to these |
| [Time-integration papers](00-references/05-time-integration.md) | Projective Dynamics, Position-Based Dynamics, XPBD, and the 2017 extensions (quasi-Newton, ADMM). The alternatives [Part 5 Ch 01](../50-time-integration/01-projective.md) rejects |
| [Co-simulation papers](00-references/06-co-simulation.md) | Partitioned and fixed-point coupling references for [Part 5 Ch 03](../50-time-integration/03-coupling.md). Felippa-Park 1980 partitioned analysis plus modern multi-timestep and fixed-point survey references |
| [Sparse-solver papers](00-references/07-solvers.md) | CG, MINRES, AMG, preconditioners, and mixed-precision iterative refinement. The [Part 8 Ch 02](../80-gpu/02-sparse-solvers.md) GPU iterative-solver cluster anchors here |
| [Optimization-loop papers](00-references/08-optimization.md) | Bayesian optimization (EI, UCB, gradient-enhanced GPs, TuRBO), information-theoretic active learning (MES, PES, BALD, cost-aware), preference learning (probit-GP, Bradley–Terry, dueling bandits), surrogate uncertainty (deep ensembles, calibration), and sim-to-real calibration (Kennedy–O'Hagan, domain randomization, residual physics). [Part 10](../100-optimization/00-forward.md)'s optimization-loop cluster anchors here |

The anchor convention is `{#lastname-year}` for single-author or first-author-plus-al works, `{#lastname-lastname-year}` for two-author works. Disambiguation suffixes (e.g. `li-2020-sde` vs. `li-2020`) distinguish separate works by the same author in the same year.
