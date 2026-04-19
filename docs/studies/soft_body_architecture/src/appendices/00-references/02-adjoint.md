# Adjoint method papers

The adjoint machinery in [Part 6](../../60-differentiability/00-what-autograd-needs.md) — implicit-function-theorem gradients at equilibrium, backward time-adjoint integration, checkpointing at long horizons, stochastic adjoints under Brownian forcing — is the book's largest single literature-anchored subsystem. This leaf indexes the works Pass 1 cites by anchor.

Pass 1 populates only anchors referenced inline by the book. Pass 3 will expand each entry with venue, DOI, and the specific result the book uses, plus adjacent references (Pontryagin, Bliss, the control-theory origins of the adjoint) that Pass 1 does not need.

## Griewank & Walther 2000 {#griewank-walther-2000}

*Algorithm 799: Revolve — An Implementation of Checkpointing for the Reverse or Adjoint Mode of Computational Differentiation.* ACM Transactions on Mathematical Software.

The original Revolve algorithm — binomial checkpoint placement that achieves $O(\log T)$ memory and $O(T \log T)$ compute with provably optimal constants for fixed checkpoint budgets. Cited inline from the [Part 6 Ch 04 table of revolve-algorithm techniques](../../60-differentiability/04-checkpointing.md). The algorithm is the standard reference that adolC, Dolfin-adjoint, and JAX's `jax.checkpoint` multi-level scheduling descend from; `sim-soft` uses it at the time-adjoint layer to recover linear-in-$T$ memory at the price of $\log T$ extra forward re-solves.

## Li et al. 2020 (SDE adjoint) {#li-2020-sde}

*Scalable Gradients for Stochastic Differential Equations.* AISTATS 2020.

The reference for backward-in-time adjoint integration through an SDE, delivering unbiased gradients for trajectories driven by Brownian forcing. Cited inline from [Part 6 Ch 03 (time adjoint)](../../60-differentiability/03-time-adjoint.md) as the basis for stochastic-adjoint machinery [`sim-thermostat`](../../110-crate/02-coupling/01-thermostat.md)'s thermal-fluctuation-coupled simulations require. Distinct from the Li et al. 2020 IPC paper ([`#li-2020` in the IPC leaf](00-ipc.md#li-2020)); the disambiguation suffix prevents collision.

## Kidger et al. 2021 {#kidger-2021}

*Efficient and Accurate Gradients for Neural SDEs.* NeurIPS 2021.

Companion reference on the Stratonovich vs. Itô choice for SDE adjoints and the resulting estimator-variance implications. Cited inline from [Part 6 Ch 03 (time adjoint)](../../60-differentiability/03-time-adjoint.md) alongside Li et al. 2020 SDE as the pair the book points at for stochastic-adjoint background. Relevant to `sim-soft` specifically because the [custom thermo-RL project's](../../10-physical/00-canonical.md) exploration noise sits inside the forward trajectory.

## Implicit differentiation through fixed points

Two anchor references for the "differentiate through an implicit solve by implicit-function theorem rather than by unrolling" pattern, cited inline from [Part 6 Ch 00 (what physics-specific autograd needs)](../../60-differentiability/00-what-autograd-needs/01-physics-specific.md) as the differentiable-programming community's landing of the same construction `sim-soft`'s [Ch 02 IFT derivation](../../60-differentiability/02-implicit-function.md) uses at the backward-Euler-step level.

### Bai et al. 2019 {#bai-2019}

*Deep Equilibrium Models.* NeurIPS 2019. arXiv:1909.01377.

Introduces the deep-equilibrium (DEQ) architecture — an infinite-depth weight-tied network whose forward pass is a fixed-point solve $z^\ast = f_\theta(z^\ast, x)$ and whose backward pass uses the implicit function theorem to compute $\partial z^\ast / \partial \theta$ with a single linear solve rather than by unrolling the fixed-point iteration. The algebraic form matches the backward-Euler IFT gradient `sim-soft` uses, modulo the replacement of the neural-network fixed-point operator by the energy-minimization residual.

### Blondel et al. 2022 {#blondel-2022}

*Efficient and Modular Implicit Differentiation.* NeurIPS 2022. arXiv:2105.15183.

Generalizes DEQ-style implicit differentiation into a modular JAX library ([JAXopt](https://jaxopt.github.io)) that composes arbitrary forward solvers with IFT-derived backward passes. The relevance to `sim-soft` is architectural, not library-level: Blondel et al. name "solver and backward are separate concerns, glued by one linear solve" as the core pattern, which is the same architectural claim [Ch 02](../../60-differentiability/02-implicit-function.md) makes for sim-soft's Newton-plus-IFT composition. `sim-soft` does not import JAXopt; it writes the composition directly on `sim-ml-chassis`'s tape.

## FEM assembly adjoint

Three anchor references for the "analytical adjoint of FEM assembly is a single pass over the connectivity list" result, cited inline from [Part 6 Ch 01 §01 FEM assembly VJP](../../60-differentiability/01-custom-vjps/01-fem-assembly.md). The composition itself is not a named theorem — it is the chain rule applied to a standard FEM assembly forward — so the references cover the forward structure (Hughes) and the adjoint-method tradition in which the discrete composition sits (Plessix; Mitusch et al. for its automated-AD embodiment in Firedrake/FEniCS).

### Hughes 2000 {#hughes-2000}

*The Finite Element Method: Linear Static and Dynamic Finite Element Analysis.* Dover reprint of the 1987 Prentice Hall original. Thomas J. R. Hughes.

Standard reference for the FEM forward that the Part 6 Ch 01 §01 adjoint composes against. Covers shape functions, the strain-displacement matrix $B$, element stiffness $K^e = B^T \mathbb{C} B V^e$, and the scatter-and-add assembly over element connectivity. The adjoint in sim-soft follows by the chain rule applied to this forward; Hughes does not discuss the adjoint explicitly but anchors the notation and the discrete structure the adjoint uses.

### Plessix 2006 {#plessix-2006}

*A review of the adjoint-state method for computing the gradient of a functional with geophysical applications.* Geophysical Journal International, 167:495–503. DOI [10.1111/j.1365-246X.2006.03006.x](https://doi.org/10.1111/j.1365-246X.2006.03006.x).

Review of the discrete-adjoint method as practised in geophysical inverse problems. Section 3 documents the per-element-assembly adjoint composition as the standard discretization of the continuous adjoint equations, with explicit treatment of the scatter-and-add structure's adjoint (a gather-and-sum). Cited inline from Part 6 Ch 01 §01 as the closest published reference for the per-element adjoint composition the sub-leaf writes out.

### Mitusch, Funke, Dokken 2019 {#mitusch-2019}

*dolfin-adjoint 2018.1: automated adjoints for FEniCS and Firedrake.* Journal of Open Source Software, 4(38):1292. DOI [10.21105/joss.01292](https://doi.org/10.21105/joss.01292).

The pyadjoint library (Python front-end for Firedrake/FEniCS) that automates the per-element-assembly adjoint composition over FEM variational forms. Cited inline from Part 6 Ch 01 §01 as a representative of the automated-FEM-adjoint library class; sim-soft's hand-written per-element adjoint is the same composition, implemented per element type rather than JIT-compiled per variational form.

## Pass 3 anchors (not yet inline-cited)

Reserved slots the Pass 3 bibliography will populate: the original Pontryagin derivation (for completeness), Giles & Pierce's review of the adjoint approach in CFD, modern neural-ODE adjoint papers (Chen et al. 2018), and control-theory-community IFT adjoint work from the shape-optimization literature. None is anchored here yet because no inline citation currently references them.
