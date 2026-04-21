# What nobody has shipped yet — the gap

[§01](01-convergence.md) argued that the four axes from [§00](00-definitions.md) have converged onto one architecture in 2026. This sub-chapter argues that no working stack has integrated that architecture yet — and that the missing piece is integration work, not undiscovered research. That distinction is what makes `sim-soft` a design study rather than a PhD.

## The missing combination

The specific thing nobody has shipped is this combination:

- In **Rust**, with the safety and toolchain properties that come with it,
- On top of **[Bevy](../../90-visual/05-sim-bevy.md)**, with the FEM mesh as the render mesh and per-vertex physics attributes feeding the shader directly,
- On **GPU** via [wgpu compute kernels](../../80-gpu/00-wgpu-layout.md) for the hot paths — [sparse CG](../../80-gpu/02-sparse-solvers/00-cg.md), per-element neo-Hookean assembly, [IPC barrier evaluation](../../40-contact/05-real-time.md),
- With [full differentiability](../../60-differentiability/00-what-autograd-needs.md) via the implicit function theorem and the time-adjoint method, on the same tape that records the forward kernel calls,
- Driven by [SDF-authored geometry and material fields](../../70-sdf-pipeline/00-sdf-primitive.md) from `cf-design`, with [live re-meshing under design edits](../../70-sdf-pipeline/04-live-remesh.md),
- Using [hyperelastic constitutive laws](../../20-materials/04-hyperelastic.md) (neo-Hookean + Mooney-Rivlin + Ogden + anisotropy + viscoelasticity) calibrated to measured material data,
- With [IPC-grade non-penetration contact](../../40-contact/00-why-ipc.md) and smoothed friction integrated into the same energy minimization as the elastic solve.

Every line of the list is a capability [Ch 02](../02-sota.md) finds demonstrated in at least one existing solver. No line is net-new or requires an undiscovered algorithm. The *combination* is the gap.

## Why the gap has the shape it does

Each category of existing stack stopped at a different side of the 2015-era tradeoff:

- **Games-branch stacks** ([Flex](../02-sota/05-nvidia-flex.md), [Jelly](../02-sota/06-houdini-jelly.md), [MuJoCo flex](../02-sota/08-mujoco-flex.md)) chose real-time first and accepted a loss on physical correctness: PBD or penalty contact over IPC-grade non-penetration, no hyperelastic default, no differentiability.
- **Science-branch stacks** ([PolyFEM](../02-sota/00-polyfem.md), [FEBio](../02-sota/07-febio.md), [IPC Toolkit](../02-sota/03-ipc-toolkit.md)) chose physical correctness first and accepted offline CPU execution with limited-to-no differentiability and no real-time path.
- **Differentiable-physics stacks** ([Warp](../02-sota/01-warp.md), [Genesis](../02-sota/02-genesis.md), [DiffTaichi](../02-sota/04-difftaichi.md)) chose GPU throughput and differentiability but stopped short of IPC-grade contact, or hedged on fully-bundled hyperelastic + friction + near-incompressible material coverage.

These three categories exhaust the design space as historically occupied. The gap is not a slice any one category is close to crossing on its own — it requires picking up the pieces each category left behind and building one stack that carries all three. That is what [`sim-soft`](../../110-crate/00-module-layout.md) is.

## Integration, not research

Each axis from [§00](00-definitions.md) has at least one reference implementation:

- **Physically correct:** [PolyFEM](../02-sota/00-polyfem.md) demonstrates the full four-component combination (hyperelastic + IPC + implicit + measured-data); [FEBio](../02-sota/07-febio.md) demonstrates a biomechanics variant (hyperelastic + viscoelastic + implicit + measured-data, with mortar/penalty contact in place of IPC). The physical-correctness axis is demonstrated.
- **Visually great:** Houdini Jelly is the production proof of what per-vertex-attribute + shader integration looks like at the ceiling; the technique is demonstrated even though Jelly itself is offline and closed-source.
- **Real-time:** NVIDIA Warp's published neo-Hookean benchmarks demonstrate GPU FEM at 60+ FPS on tens-of-thousands-of-tets scenes; NVIDIA Flex demonstrates GPU soft-body at production game-engine frame rates. Real-time is solved.
- **Differentiable:** Warp, Genesis, and DiffTaichi each demonstrate end-to-end differentiable physics with adjoint-via-IFT or adjoint-state on the tape.

No axis requires a technique that has not yet been invented. `sim-soft`'s contribution is what happens when those four separately-demonstrated capabilities are built into one solver with consistent traits ([Part 11 Ch 01](../../110-crate/01-traits.md)), one sparse-linalg stack ([faer on CPU, wgpu on GPU per Part 11 Ch 03](../../110-crate/03-build-order.md#the-committed-order)), and one tape that records forward kernel calls and replays them for gradients without picking up a Python bridge or a C++ dependency.

## The honest post-Phase-I qualifier

"Integration, not research" is a load-bearing claim and it earns its load by being narrow. The [Phase A–I build-order](../../110-crate/03-build-order.md#the-committed-order) commits to closing the *integration* gap by Phase I — the visual layer completes the fusion, and every earlier phase has a shipping criterion that closes one boundary. What Phase A–I does *not* commit to closing are the research-frontier items the book is explicit about:

- **[Differentiability through meshing](../../60-differentiability/05-diff-meshing.md) under topology-crossing SDF edits.** [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) names this as an open problem; the [γ-locked `GradientEstimate::Noisy { variance }` return variant](../../100-optimization/00-forward.md) is the architecture's honest admission that ~5% of gradient calls do not get the clean adjoint treatment. Closing the topology-crossing gap at full 5-digit gradcheck discipline is research, not integration, and is deferred [post-Phase-I per Part 12 Ch 07](../../120-roadmap/07-open-questions.md).
- **The physical-print feedback loop.** [Part 10 Ch 05](../../100-optimization/05-sim-to-real.md)'s `SimToRealCorrection` and [Part 10 Ch 06](../../100-optimization/06-full-loop.md)'s design-print-rate loop consume [`MeasurementReport`](../../100-optimization/05-sim-to-real.md) streams; the physical-print loop is the [highest-priority post-Phase-I engineering item](../../120-roadmap/06-optimization.md), not a Phase A–I commitment, and its inclusion in the eventual ceiling is conditional on printing-pipeline work the book does not itself architect.
- **Research-frontier open questions.** [Part 12 Ch 07](../../120-roadmap/07-open-questions.md) enumerates the contestable commitments — differentiable meshing, IPC real-time at very large scale, the SDF-to-FEM pipeline under live design edits at full gradient fidelity — that the book names as open, not as ready-to-ship.

The distinction matters. Claiming `sim-soft` closes every adjacent research question would be the kind of overclaim [§01](01-convergence.md) warned against: conflating the existence of an architectural skeleton with the delivery of every consequence the skeleton might one day support. The ceiling [§00](00-definitions.md) defines is tight: four axes, four testable criteria, one integrated solver at Phase I. The research frontier is named separately, deferred honestly, and not folded into the integration claim.
