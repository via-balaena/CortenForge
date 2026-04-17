# Differentiability through meshing — the open problem

This is the book's honest chapter. Every other chapter in Part 6 names a solved problem with a known gradient construction — reverse-mode autograd for policy networks ([Ch 00](00-what-autograd-needs.md)), hand-derived VJPs for fused physics kernels ([Ch 01](01-custom-vjps.md)), the implicit function theorem for equilibrium configurations ([Ch 02](02-implicit-function.md)), the adjoint method for trajectory objectives ([Ch 03](03-time-adjoint.md)), Revolve checkpointing for long-horizon adjoints ([Ch 04](04-checkpointing.md)). All of them assume the mesh is fixed. This chapter names what breaks when the mesh itself is a design variable, why the existing research has not closed the gap, and what `sim-soft` does as a result.

## The gap

The [thesis](../10-physical/03-thesis.md) commits `sim-soft` to two things that do not compose for free.

**Commitment 1 — differentiability is first-class.** Every design parameter $\theta$ — material stiffness, fiber direction, contact friction coefficient, boundary traction, reward-term weight — must admit a gradient $\partial L / \partial \theta$ that the optimization loop in [Part 10](../100-optimization/00-forward.md) consumes. The book has paid for that commitment in the previous four chapters.

**Commitment 2 — geometry is authored as SDFs, live-remeshed.** [Part 7](../70-sdf-pipeline/00-sdf-primitive.md) commits to SDF-valued geometry as the universal design primitive; [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md) commits to live re-meshing under design edits. The designer edits an SDF, the tet mesh re-derives, the solve continues from the warm-started previous state.

The problem is at their intersection. If the SDF parameters $\theta_{\text{sdf}}$ are design variables — SDF blend radii, control-point positions of a spline-based boundary, SDF-valued material-field parameters — then the mesh is a *function* of $\theta_{\text{sdf}}$, and computing $\partial L / \partial \theta_{\text{sdf}}$ requires propagating gradients *through* the meshing step. That step is, for every meshing algorithm that ships today, not differentiable.

## Why marching cubes is not differentiable

[Marching cubes](https://dl.acm.org/doi/10.1145/37401.37422) (Lorensen & Cline 1987) and its variants are the default SDF-to-surface algorithm. The construction is: evaluate the SDF on a regular grid, and for each grid cell, choose a *mesh topology pattern* (one of 15 canonical cases for tet meshing, 256 for cube meshing) based on the sign pattern of the SDF at the cell corners; then place vertices on cell edges by linear interpolation of the SDF values bracketing zero.

The vertex-placement step *is* differentiable — if a cell is classified with a fixed topology, moving the SDF values smoothly moves the vertex along the edge, smoothly. The gradient of a vertex position w.r.t. an SDF corner value is a one-line expression.

The *topology-classification step is not*. When the SDF value at a corner crosses zero, the cell's topology jumps — a vertex appears or disappears, an edge is added or removed, a tet subdivision pattern changes. The mesh-level function $x_{\text{mesh}}(\theta_{\text{sdf}})$ has a step discontinuity at every corner where the SDF passes through zero. The derivative is a Dirac delta at those points and zero elsewhere. No amount of chain rule can rescue this: the function is literally discontinuous.

The same pathology afflicts [Delaunay tetrahedralization](../70-sdf-pipeline/01-tet-strategies/01-delaunay.md) (edge flips are discrete), [fTetWild](../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md) (envelope-based mesh simplification makes discrete local topology choices), and every other meshing pipeline in the scientific-computing ecosystem. Meshing is a *combinatorial* algorithm with a numerical front-end; the numerical part is smooth, the combinatorial part is not.

## What the research has tried

Five approaches have been published for closing or working around the gap.

**DiffMC — differentiable marching cubes.** [Wei et al. 2023](../appendices/00-references/03-diff-sim.md#wei-2023) replaces the discrete topology choice with a soft classification: each cell's topology is a convex combination of the 15 canonical patterns, weighted by a softmax over the SDF corner values. Vertices exist everywhere with variable mass, and the topology emerges as the softmax sharpens. This makes the full pipeline end-to-end differentiable, with one significant cost: the "mesh" produced in the soft regime has phantom vertices and unphysical element connectivity, and training has to include a temperature-annealing schedule to recover a usable mesh at the end. For inverse-rendering tasks where the mesh is consumed by a differentiable renderer, this is acceptable. For FEM, where phantom vertices translate into phantom mass and spurious stiffness entries, it is not obviously usable.

**Neural marching cubes.** [Chen & Zhang 2021](../appendices/00-references/03-diff-sim.md#chen-zhang-2021) trains a network to place vertices and choose topologies that a downstream loss likes, amortizing the discrete choice into the network weights. The network is differentiable by construction; the discrete topology choice happens inside the network forward pass and is effectively a Gumbel-softmax or a hard argmax with straight-through estimator. This works well for reconstruction tasks where the ground-truth mesh is known; for FEM, where "the right mesh" is not known in advance — it is whatever mesh produces the best solver convergence and stress-resolution for the downstream physics — the training signal is not obvious.

**DMTet — deep marching tetrahedra.** [Shen et al. 2021](../appendices/00-references/03-diff-sim.md#shen-2021) parameterizes a deformable tetrahedral grid whose SDF values live on the grid vertices; the iso-surface is extracted by a differentiable marching-tetrahedra variant that handles the six-tet partition of the cube with a small fixed set of topologies. Gradient flow through vertex positions is clean; topology changes are rare and happen at grid-cell boundaries. DMTet has been adopted in 3D-generation pipelines (NVIDIA GET3D, Magic3D) and is arguably the most production-ready of the differentiable-meshing approaches — but it is still a surface mesher, not a volumetric tet mesher, and it inherits the "phantom elements at topology boundaries" problem in attenuated form.

**Differentiable Delaunay.** [Wang et al. 2023](../appendices/00-references/03-diff-sim.md#wang-2023) and subsequent work formulate Delaunay triangulation as a limit of a regularized LP problem; gradients flow through the LP's dual variables. This is mathematically clean but scales poorly — the LP has to be re-solved on every geometry update, and the constants are prohibitive at the tet counts `sim-soft` targets (30k+). Production deployment in FEM has not been demonstrated.

**Finite-difference wrappers.** The brute-force baseline: re-mesh for $\theta + \delta$ and $\theta - \delta$, solve both, central-difference the result. Cost is $2n_\theta$ full solves per gradient evaluation; noise is $O(\delta^2)$ plus mesh-topology-change noise at the boundaries where the topology flips between $\theta \pm \delta$. For $n_\theta \lesssim 10$ this is acceptable; for high-dimensional design spaces it is not. FD wrappers are the fallback, not the plan.

## What `sim-soft` does

Given that none of the five options is a clean solution for a real-time differentiable FEM, `sim-soft` commits to a three-part compromise that is honest about the limitation.

**Topology-fixed-per-episode.** Inside one optimization episode — one rollout from initial condition to terminal state, with the gradient flowing back through the time adjoint — the mesh topology is held fixed. All gradients within the episode flow through the [IFT](02-implicit-function.md) and [time adjoint](03-time-adjoint.md) on a fixed-topology mesh. This covers the common case: optimize material parameters, boundary conditions, contact friction, reward weights on a given geometry. The gradient is clean.

**Between-episode re-meshing with warm-started state transfer.** When the SDF changes enough that the mesh topology should change — the designer moves a control point, the blending radius crosses a threshold where a hole opens or closes — the mesh is *re-derived* between episodes via the [SDF → tet pipeline](../70-sdf-pipeline/00-sdf-primitive.md), and the solver state is warm-started via the [state transfer](../70-sdf-pipeline/04-live-remesh/02-state-transfer.md) machinery in Part 7. The *gradient does not flow through the re-mesh*. The optimizer treats the new mesh as a fresh episode starting point.

**Finite-difference wrappers at topology boundaries.** When a gradient w.r.t. an SDF parameter is actually requested — $\partial L / \partial \theta_{\text{sdf}}$ is a term in the loss — the wrapper evaluates $L(\theta_{\text{sdf}} \pm \delta)$ with two full re-meshed-and-resolved episodes and central-differences. The $\delta$ is tuned per parameter to avoid the topology-flip region when possible; when the parameter straddles a topology-change threshold, the gradient estimate is flagged as noisy and the outer optimizer is notified. This is the compromise the book is honest about: gradients through $\theta_{\text{sdf}}$ are affordable only at the scale of tens of parameters, not thousands.

The practical consequence is that `sim-soft`'s *differentiability commitment applies to everything that is not the SDF* — materials, contact, reward terms, boundary conditions, policy — cleanly and at scale. The SDF itself is partially differentiable, with a known noise source at topology boundaries, and is parameterized to avoid dense gradient flow where possible. [Part 7 Ch 00](../70-sdf-pipeline/00-sdf-primitive.md) documents the specific SDF parameterization conventions that keep topology changes rare.

## The research frontier

What would a clean solution look like? Two shapes.

**Option A — `sim-soft` gets DMTet-style volumetric differentiable meshing.** Extend DMTet's surface construction to a full volumetric tet mesh with per-tet material properties carried across topology changes. Phantom elements at boundaries are weight-scaled so they contribute negligibly to the FEM residual while remaining in the gradient path. Open questions: do the phantom elements destabilize the IPC barrier? Does the annealing schedule compose with Revolve checkpointing? Can the chassis's VJP registration accommodate a non-trivial forward pass with a learned parameter (the temperature)?

**Option B — `sim-soft` gets a principled topology-aware gradient.** A Dirac-delta contribution at topology-change boundaries is not a bug; it encodes the correct change-of-configuration-space information. If that delta can be integrated analytically — as a "change-of-topology adjoint" with a known coefficient derived from the SDF geometry at the crossing — the gradient is no longer noise, it is a well-defined generalized-function contribution. Integration in a Monte-Carlo estimator (similar to the visibility-boundary treatment in [Li et al. 2018](../appendices/00-references/04-rendering.md#li-2018)) would give unbiased estimates. This has not been published for FEM.

Either direction is multi-year research. `sim-soft` commits to the three-part compromise above and flags both options as open. [Part 12 Ch 07](../120-roadmap/07-open-questions.md) names this as the book's highest-priority unsolved problem.

## Where this leaves the book

The thesis's commitment to differentiability as first-class is honored *everywhere except through the SDF*. That is a sharp, reproducible, checked boundary — gradcheck passes on everything except the FD-wrapped SDF gradients; the FD-wrapped ones are flagged in the chassis API with a `GradientEstimate::Noisy { variance }` variant so downstream optimizers can weight them appropriately. A less honest book would claim the gap does not exist, hide the FD wrapper inside a smoothed shim, and let noise corrupt the outer optimizer silently. `sim-soft` refuses that route: the gap is named, the compromise is named, the research frontier is named, and the cost of the compromise (affordable SDF design spaces are small) is named.

This chapter is where the book's physical-and-visual-are-one ambition meets the limits of what the field has built. The rest of Part 6 buys a very sharp differentiability; Ch 05 is where the book admits what it has not bought. Naming the gap is what makes the rest of the commitment trustworthy.
