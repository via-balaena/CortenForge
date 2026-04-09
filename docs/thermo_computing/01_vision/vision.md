# Vision (the endpoint)

> Extracted from `MASTER_PLAN.md` §1 during the 2026-04-09 doc-tree refactor.

Build a unified **design → simulate → fabricate** pipeline for *mechanical
thermodynamic computing devices* — physical systems whose probabilistic
behavior is the computation. The same parameters drive the geometry (cf-design),
the physics (sim-core + a Langevin thermostat), the energy-based model
(ml-bridge autograd), and the controller (ml-bridge RL).

The probabilistic computing element is **not** a transistor. It is a bistable
mechanical structure — a buckling beam, a snap-through latch, a Brownian
ratchet — whose state is set by thermal noise plus local field. These elements
are real, they're studied seriously in stochastic thermodynamics (Landauer,
Bennett, Sekimoto), and **they are 3D-printable**. Extropic builds p-bits out
of transistors because they have a fab. CortenForge has cf-design and a 3D
printer. That's a different — and arguably more interesting — angle.

## Why CortenForge is the right substrate

- **sim-core** already does articulated rigid-body dynamics with constraint
  solving at the device scale. This is where the bistable elements live.
- **External force injection** (`qfrc_applied`, `xfrc_applied`) and
  **damping infrastructure** (`dof_damping`, `jnt_damping`, etc.) already
  exist and are MuJoCo-conformant. The fluctuation-dissipation machinery is
  half-built without anyone planning for it.
- **cf-design** parameterizes the geometry of bistable elements with
  implicit surfaces and mechanism primitives. The same parameters can be made
  differentiable.
- **ml-bridge autograd** (Phases 1–6c) gives us a Rust-native gradient engine
  for training energy functions.
- **ml-bridge RL** (CEM/TD3/REINFORCE + policy persistence) can consume the
  resulting stochastic environment.
- **3D printing** closes the loop in a way no software-only stack can.

## Headline research claim (aspirational)

> One Rust-native pipeline where geometry, physics, energy model, and control
> policy share a single parameter set — and where the simulated device matches
> its 3D-printed twin within validated tolerances.

(See §2's *Synthesis* for the longer version: same claim, framed in terms of
the energy function the four subsystems share.)

Nobody has this. LAMMPS can't design. cf-design can't sample. PyTorch EBMs
aren't physical. Extropic can't fab cheaply. The wedge is real.
