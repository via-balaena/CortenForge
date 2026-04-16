# Introduction

> Via Balaena LLC / CortenForge — Foundational Research Document v1
>
> Status: Pre-experimental hypothesis generation
> Date: April 2026
> Classification: Open research — intended for public release

This document records the intellectual origin of a research program connecting biological locomotion control theory to the unsolved x-encoding problem in thermodynamic computing. It is written as a living document: a record of where the reasoning started, what has been established, and what remains open. It is not a paper. It is the foundation from which papers will be written.

The central claim is this: the full spectrum of biological strategies for navigating chaotic physical media — from bacteria to peregrine falcons — constitutes a map of design principles for injecting logical inputs into nonequilibrium stochastic physical systems. Each regime of the biological spectrum corresponds to a distinct engineering strategy. The map is not complete. But enough of it has been identified to constitute a serious research program.

## Thermodynamic Computing and the Y Problem

Thermodynamic computing is a paradigm in which computation is performed not by enforcing deterministic logic states, but by allowing a physical system to relax toward thermodynamic equilibrium, reading the resulting probability distribution as the output. The key entities are:

- **The energy function F**: defines the shape of the target probability distribution over output states
- **Y**: the target distribution itself — the answer the system is supposed to produce
- **The physical substrate**: a stochastic system (resistor-inductor-capacitor networks, analog probabilistic circuits, or future purpose-built thermodynamic chips) whose natural dynamics under Langevin noise are governed by F

The state of the art as of 2025-2026 includes Normal Computing's CN101 — the world's first taped-out thermodynamic semiconductor chip — and Extropic's XTR-0 development platform, both of which demonstrate that thermodynamic sampling units (TSUs) can perform AI inference tasks including matrix inversion and Gaussian sampling at energy efficiencies orders of magnitude beyond conventional GPUs.

The Y problem, to first approximation, is solved. Energy-based models provide a formal language for defining Y. Boltzmann statistics guarantee that a system in thermal equilibrium will sample from Y. The physics of the output is understood.

## The X Problem — The Unsolved Root

The unsolved problem is X: the input encoding.

In a classical digital computer, encoding an input X is trivial — you set a voltage high or low. The physical act of encoding is decoupled from the physics of computation. In a thermodynamic computer, there is no such decoupling. The input X must be encoded as the initial conditions or boundary conditions of a physical stochastic system. The system then relaxes — through a nonequilibrium trajectory governed by Langevin dynamics — toward (hopefully) the correct distribution Y.

The problem is that this relaxation trajectory is sensitive to how X is injected. An imperfect encoding perturbs the energy landscape, potentially steering the system toward the wrong basin of attraction. At small scale, this can be managed empirically. At scale — as the circuit grows in size and the noise profile shifts — there is no formal theory telling an engineer how to inject X such that the relaxation remains correct. Every scaling step requires empirical re-tuning.

This is not a materials problem or a fabrication problem. It is a theory problem: we lack a formal design language for X-encoding in nonequilibrium stochastic systems. Until that language exists, thermodynamic computing cannot be engineered the way digital computing is engineered — from first principles, with predictable scaling behavior.

## The Deeper Root: The Gap in Nonequilibrium Statistical Physics

The theory problem traces to a gap in physics. Equilibrium statistical physics is well-understood. Landauer's bound gives the minimum energy cost of erasing a bit. Boltzmann statistics describe the equilibrium distribution. But thermodynamic computers operate far from equilibrium — they must complete a computation quickly, which means they cannot afford to wait for true equilibrium to be reached.

The stochastic thermodynamics of far-from-equilibrium systems is a field in active development. Thermodynamic uncertainty relations now provide bounds on computation speed, noise level, and energy cost. But a constructive theory — one that tells you how to build a nonequilibrium system that reliably encodes X and converges to Y — does not yet exist.

The analogy: we know the speed limit (Landauer's bound, uncertainty relations), but we have no road map.

## The Core Hypothesis

The hypothesis is that biological evolution has already solved versions of the X-encoding problem across a continuous spectrum of operating regimes, and that the biological solutions can be formally mapped to engineering principles for thermodynamic circuit design.

Specifically: every organism that navigates a chaotic physical medium — fluid, air, a stochastic chemical gradient — is solving a version of the same problem. It must inject "intent" (a direction, a target, a behavioral goal) into a noisy physical system (its own body in a turbulent medium) and achieve reliable convergence to the desired output state, at some throughput level, with finite energy. The physics of the problem is structurally isomorphic to X-encoding in thermodynamic computing.

The biological world has explored this design space for hundreds of millions of years. We should read the solutions it found.
