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

## Pass 3 anchors (not yet inline-cited)

Reserved slots the Pass 3 bibliography will populate: the original Pontryagin derivation (for completeness), Giles & Pierce's review of the adjoint approach in CFD, modern neural-ODE adjoint papers (Chen et al. 2018), and control-theory-community IFT adjoint work from the shape-optimization literature. None is anchored here yet because no inline citation currently references them.
