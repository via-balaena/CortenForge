# Open research questions

Every other chapter in Part 12 names a capability the Phase A–I roadmap commits to ship. This chapter names what the roadmap does *not* commit to: problems the book has identified as load-bearing on its ambition but unsolved at the field level, or scoped out of the committed phases, or contingent on resources the simulation crate cannot assume. Naming these is the point; the book's commitments are only trustworthy to the degree that the gaps around them are trustworthy too. The tone is deliberate: this is the chapter where the book admits what it has not bought.

Five open problems, ranked by priority for the project as a whole — not by research-community prominence.

## 1. Differentiable meshing

**Status.** Open research frontier. No known solution composes cleanly with a real-time differentiable IPC FEM at the tet counts `sim-soft` targets.

**Why the roadmap does not solve it.** [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md) is the book's long-form treatment of the gap. Five approaches have been published — deep marching cubes (soft topology), neural marching cubes (amortized discrete choice), DMTet, differentiable Delaunay (Rakotosaona / Son), and finite-difference wrappers — and none is a production-ready drop-in for a fully-volumetric differentiable FEM at 30k tets. `sim-soft` commits to a three-part compromise: topology-fixed-per-episode, between-episode re-meshing with warm-started state transfer, and FD wrappers at topology boundaries. The compromise is a compromise; it is not a solution.

**What would a solution look like.** Two concrete options from [Part 6 Ch 05's research-frontier section](../60-differentiability/05-diff-meshing.md#the-research-frontier):

- **Option A — DMTet-style volumetric meshing extended to tet-mesh FEM.** A temperature annealing schedule, phantom elements weight-scaled to negligible FEM residual contribution, all plugged into the chassis VJP registration via an additional input slot for the temperature parameter. Open questions: do the phantom elements destabilize the IPC barrier? Does the annealing schedule compose with Revolve checkpointing from [Part 6 Ch 04](../60-differentiability/04-checkpointing.md)? Is the FEM-reward gradient signal informative enough to drive the temperature annealing directly, or is a prescribed schedule of optimizer progress a more reliable construction?
- **Option B — topology-aware gradient via an analytically-integrated Dirac contribution.** A "change-of-topology adjoint" with a known coefficient derived from the SDF geometry at the crossing, integrated in a Monte-Carlo estimator analogous to visibility-boundary treatments in differentiable rendering. Has not been published for FEM.

Either direction is multi-year research. `sim-soft` flags both as open and keeps its commitment honest by flagging SDF-parameter gradients as `GradientEstimate::Noisy { variance }` at topology boundaries rather than hiding FD noise behind a shim.

**Why this is priority 1.** Every other commitment in the book is differentiable by construction. The meshing gap is the *only* place where the thesis's differentiability-is-first-class claim has to be qualified. Closing it would elevate `sim-soft` from "differentiable everywhere except through the SDF" to "differentiable everywhere," which is a sharper and more defensible claim.

## 2. Post-Phase-I physical-print loop

**Status.** Known to be needed, scoped out of Phases A–I, not research-frontier — the components exist, but the integration requires resources out of scope for the simulation crate.

**Why the roadmap does not deliver it.** The [full design-print-rate loop from Part 10 Ch 06](../100-optimization/06-full-loop.md) closes around physical printing: `MeasurementReport` ingestion, residual GP online updates per [Part 10 Ch 05](../100-optimization/05-sim-to-real.md), batch-rating workflow, cross-print drift detection. Each of these requires a specific printer, a set of measurement instruments (force sensor, contact-pressure film, integrating-sphere spectrophotometer), a measurement protocol, and a designer in the loop. The simulation crate does not commit to acquiring any of these. [Part 10 Ch 05's "What this does NOT commit"](../100-optimization/05-sim-to-real.md#what-this-does-not-commit) names the scope boundary explicitly — no sensor drivers ship with `sim-soft`, no cross-printer transfer learning is promised, no adversarial bias bounds are guaranteed.

**What closing it would require.** Concretely:

- A reference printer and material configuration the project treats as an anchor — e.g., an Ecoflex- or Dragon-Skin-compatible direct-ink-write or silicone-casting setup.
- A measurement bench: force sensor, Tekscan or equivalent pressure array, integrating-sphere spectrophotometer for the diffusion-profile calibration in [Part 10 Ch 05](../100-optimization/05-sim-to-real.md).
- A `MeasurementReport` JSON schema (this part is ready to specify today) and the ingest pipeline in `sim-opt`.
- A designer-rating UI or CLI, driven by the preference GP from [Part 10 Ch 03](../100-optimization/03-preference.md).
- An anchor-print schedule for printer-drift change-detection per [Part 10 Ch 05](../100-optimization/05-sim-to-real.md).

**Why this is priority 2.** Without it, `sim-soft` is an excellent differentiable simulator. With it, `sim-soft` is the engineering-workflow tool the thesis promised. The difference is not technical — the technical components exist by Phase I — it is resource-scoped. The priority reflects the distance from Phase I close to meaningful production use.

**How to think of this item.** Not research. Engineering under a budget line that the simulation crate cannot allocate. Documenting it here makes sure the seam between what `sim-soft` ships and what a full engineering workflow needs is a named seam, not a silent one.

## 3. Direct sparse factorization on GPU

**Status.** Known hard problem; scoped to iterative solvers on GPU by Phase E; direct-on-GPU deferred indefinitely.

**Why the roadmap defers it.** Phase E's [GPU port](../80-gpu/02-sparse-solvers.md) is preconditioned conjugate gradient on GPU. Direct sparse factorization on GPU — Cholesky or LU — is a well-known hard problem: the irregular fill-in pattern and the load-balancing challenge across warps/wavefronts make it difficult to match CPU sparse direct solvers' efficiency on GPU. Published GPU direct solvers target structured matrices from specific PDE discretizations and do not generalize easily to the arbitrary-tet-mesh matrices `sim-soft` produces.

Iterative solvers work fine for Phase E's rate targets, and [Part 8 Ch 02](../80-gpu/02-sparse-solvers.md) preserves the CPU 10–30× factor-on-tape amortization on GPU via preconditioner-plus-warm-start re-use. The concern lives at a different intersection:

- **Very-large-scene scaling.** Beyond roughly a hundred thousand tets, preconditioned CG's iteration count grows as the stiffness matrix condition number worsens at high aspect ratios; a direct solver's back-substitution cost would not grow the same way. For the canonical problem at ~30k tets the iterative path is fine and the 10–30× preconditioner-on-tape speedup holds. For multi-character or full-assembly scenes (hundreds of thousands of tets), CG iteration count can dominate and the preconditioner-on-tape amortization weakens.
- **Pathological-RHS workloads.** In the rare regime where backward passes demand many wildly-different RHS vectors per forward solve, CPU's cheap back-substitution beats GPU's per-RHS preconditioned CG even with warm-start. Nothing in the canonical design space puts us there; applications that would want dense Hessian extraction rather than Hessian-vector products might.

**What would improve the situation.** Algebraic multigrid (AMG) as the GPU preconditioner rather than Jacobi or block-Jacobi — per [Part 8 Ch 02's AMG sub-chapter](../80-gpu/02-sparse-solvers/02-amg.md). AMG with a mesh-aware smoother is closer to direct-solver behavior on the adjoint backward pass and scales better in DOFs. AMG is an option explored in Phase E; it is not committed to being live at Phase E close.

**Why this is priority 3.** The canonical problem does not bottleneck on this. Full-assembly-scale scenes would, and those are a post-Phase-I ambition rather than a Phase A–I deliverable. The item is named because it is where the Phase E rate model stops generalizing, and the reader should know where that breakpoint lives.

## 4. Latent-space BayesOpt for high-dimensional design

**Status.** Active research area; practical implementations exist; none is a perfect fit for the canonical-problem design space.

**Why the roadmap defers it.** The [high-dim BayesOpt sub-chapter](../100-optimization/02-bayesopt/02-high-dim.md) commits to this as a Part 10 concern. The canonical design space at 10–50 parameters is comfortably inside modern BayesOpt's sweet spot; gradient-enhanced GPs with ARD kernels handle it. Beyond a few hundred parameters, the standard GP scales poorly — the kernel matrix fill grows quadratically in evaluations, and ARD length-scales become under-identified.

Three directions the field has pursued, none a clean fit:

- **Random embeddings (REMBO, ALEBO).** Project the design space into a low-dimensional subspace and run BayesOpt there. Works if the objective has low effective dimensionality; the canonical problem's reward is reasonably strong in its material-field parameters *and* its geometric parameters, and a random-subspace projection often misses one.
- **Trust-region methods (TuRBO and relatives).** Run BayesOpt in local trust regions and shift regions as the posterior concentrates. More robust in high dimensions. Has not been composed with gradient-enhanced GPs in a production-ready way.
- **VAE-learned latent spaces.** Train a variational autoencoder on the design-space distribution and run BayesOpt in the latent space. Works well when there is a dataset of reasonable designs to train on; at the start of a canonical-problem run, there isn't.

**What the roadmap commits to.** Phases A–I ship BayesOpt at the canonical 10–50 scale with gradient-enhanced GPs. The high-dimensional regime (hundreds of parameters) is named in the [high-dim sub-chapter](../100-optimization/02-bayesopt/02-high-dim.md) and deferred to post-Phase-I, when the density of accumulated design data makes a VAE-learned latent space tractable.

**Why this is priority 4.** The canonical problem does not need it. Larger design spaces — many-primitive composition trees, per-element material fields, spatially varying fiber-direction fields — do. It is named because scaling the canonical problem to richer design spaces runs into this wall first.

## 5. Principled topology-aware gradients

**Status.** Research frontier. Overlaps with #1 but distinct.

**Why this is a separate item.** Differentiable meshing (item 1) is about making the mesh itself a differentiable function of its SDF parameters. Topology-aware gradients (this item) is about *integrating the topology change into the adjoint* when the mesh does change — Option B in [Part 6 Ch 05's frontier section](../60-differentiability/05-diff-meshing.md#the-research-frontier). The two can be pursued independently: item 1 is a structural change to the mesh data structure, this item is a change to the adjoint derivation. In principle, a clean solution to this item obviates the FD-wrapper regime at topology boundaries without requiring DMTet-style phantom elements at all.

**What would unlock it.** A derivation of the Dirac-contribution coefficient at a topology-change boundary from the SDF geometry alone; an unbiased Monte-Carlo estimator that exercises it; a regression test showing FD-free gradient variance matching or beating the FD-wrapper variance from [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md). The visibility-boundary treatment in differentiable rendering is the closest published parallel; porting it to FEM topology changes is unpublished work.

**Why this is priority 5.** The FD-wrapper regime is a workable compromise. Closing this item would elevate the compromise to an exact answer, but the compromise is not currently a blocker for the canonical design space at 10–50 parameters. Priority reflects opportunity rather than urgency.

## What this chapter commits the roadmap to

None of the five is a Phase A–I deliverable. That is the chapter's point. The commitments Part 12 Chs 02–06 make are sharpened, not weakened, by naming what they do not include. Two practical consequences:

- **The roadmap's pace is not held hostage to these five items.** Phases A–I can land on their own schedule; if any of the five resolves during the phases, that is a bonus, not a requirement. The book does not promise to move the research frontier; it promises to build a platform that absorbs a frontier move cleanly when one lands.
- **The seam between "what the book commits to" and "what the project aspires to post-Phase-I" is a named seam.** A reader arriving at Phase I close should understand that `sim-soft` is fast, differentiable-except-through-SDF, SDF-authored, visually ceiling-class, and *not yet* a closed-loop sim-to-real design-print-rate engineering tool. The distance from Phase I to that tool is the post-Phase-I physical-print loop (item 2), and the distance can be short or long depending on resource allocation that the simulation crate does not make.

[Part 6 Ch 05](../60-differentiability/05-diff-meshing.md) closed with "naming the gap is what makes the rest of the commitment trustworthy." This chapter closes with the same claim, extended to the whole roadmap. The items are on this list because they matter; they are ranked because ranking them exposes which ones most constrain the project's ambition; the book is the spec, and the spec names its limits.
