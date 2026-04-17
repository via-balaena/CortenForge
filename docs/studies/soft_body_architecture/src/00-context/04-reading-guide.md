# Reading guide by goal

This chapter is the book's table of contents sorted by who is reading it. The canonical [`SUMMARY`](../SUMMARY.md) is sorted by subject — materials, contact, time integration, and so on — which is the right order for depth-first reading. But most readers do not read the book depth-first; they read it looking for a specific thing. The paths below are suggested orders for the most common kinds of reader.

## If you are a Rust engineer building something soft

Start with the [thesis](../10-physical/03-thesis.md) (Part 1 Ch 03) for why the stack is shaped the way it is — the whole book's design decisions trace back to that chapter, and without it the module layout in Part 11 looks arbitrary.

Then jump to [Part 11 — Crate Architecture](../110-crate/00-module-layout.md) for the shape of `sim-soft` at the module and trait level, and [Part 12 — Roadmap](../120-roadmap/00-dependencies.md) for the build order and the [Part 12 Ch 02 — first-working milestone](../120-roadmap/02-first-working.md) for what "minimum viable" means. If you are going to touch the contact layer, [Part 4 Ch 00 — Why IPC](../40-contact/00-why-ipc.md) and [Part 5 Ch 00 — Backward Euler](../50-time-integration/00-backward-euler.md) are required reading; if the GPU layer, [Part 8 Ch 00 — wgpu layout](../80-gpu/00-wgpu-layout.md).

Likely skip on the first pass: Parts 3 (discretization), 7 (SDF pipeline), 9 (visual), 10 (optimization loop). These fill in later as their sub-systems become relevant to what you are building.

## If you are an ML researcher interested in differentiable physics

Start with [Part 6 — Differentiability](../60-differentiability/00-what-autograd-needs.md) for the specific things differentiable FEM needs that generic autograd does not do. Then [Part 4 — Contact](../40-contact/00-why-ipc.md), because IPC's smoothness is what makes the whole stack differentiable in the first place. Then [Part 10 — Optimization Loop](../100-optimization/00-forward.md) for how the gradient is consumed by a design optimizer.

The [Part 1 Ch 03 thesis](../10-physical/03-thesis.md) is worth reading at some point but is less central for this reader; the ML reader's question is usually *is this stack differentiable in the way I need, and how fast,* and the answer to both is in Parts 6 and 8.

Likely skip: Parts 1 (canonical problem), 9 (visual), 11 (crate architecture), unless implementation is on the table.

## If you are a game developer interested in real-time soft body

Start with [Part 1 Ch 03 — the thesis](../10-physical/03-thesis.md) for the claim that games physics and science physics have stopped being separate branches — this is the part most likely to read as contrarian from a games-industry vantage, and Ch 03 makes the case.

Then [Part 4 Ch 05 — Making IPC real-time](../40-contact/05-real-time.md) for the specific techniques (adaptive barrier width, GPU-parallel CCD, warm-started friction) that bring IPC into the frame budget. [Part 8 — GPU Implementation](../80-gpu/00-wgpu-layout.md) is central. [Part 9 — Visual Layer](../90-visual/00-sss.md) will be the most natively familiar. The SOTA survey's [NVIDIA Flex](02-sota/05-nvidia-flex.md) and [Houdini Jelly](02-sota/06-houdini-jelly.md) entries are where the comparison to the current games state of the art lives.

Likely skip: Part 6 (differentiability) unless the game uses learned behaviors; Part 7 (SDF pipeline) unless the game is procedural.

## If you are an FEM or biomechanics researcher

Start with [Part 2 — Material Models](../20-materials/00-trait-hierarchy.md), then [Part 3 — Discretization](../30-discretization/00-element-choice.md), then [Part 5 — Time Integration](../50-time-integration/00-backward-euler.md). This is closest to a graduate-course FEM trajectory and will read as familiar.

[Part 4 — Contact](../40-contact/00-why-ipc.md) is where the book may diverge from a biomech reader's expectation — `sim-soft` commits to IPC rather than the mortar / penalty / augmented-Lagrangian contact formulations common in biomech FEM. The [Part 1 Ch 03 thesis](../10-physical/03-thesis.md) and [Part 4 Ch 00 — Why IPC beats penalty and impulse](../40-contact/00-why-ipc.md) are where the reasoning lives.

Likely skip on a first pass: Part 8 (GPU) and Part 9 (visual) — the physics is solver-agnostic and biomech researchers typically come from CPU-implicit-backward-Euler backgrounds.

## If you are a soft roboticist

Start with [Part 1 — The Physical Problem](../10-physical/00-canonical.md) because the canonical problem is a generic soft-robotics setup and the language will be familiar. Then [Part 2 Ch 06 — Anisotropic hyperelasticity](../20-materials/06-anisotropic.md) for fiber-reinforced and textured elastomer materials, which is where soft robotics tends to want expressiveness the received engineering-grade FEM lacks.

Then [Part 10 — Optimization Loop](../100-optimization/00-forward.md) for the design-print-rate loop that soft robotics typically lacks on the simulation side. [Part 11 Ch 00 — module layout](../110-crate/00-module-layout.md) for the module architecture; [Part 7 Ch 00 — SDF as primitive](../70-sdf-pipeline/00-sdf-primitive.md) for the SDF authoring path that connects [`cf-design`](../110-crate/02-coupling/04-cf-design.md) to `sim-soft`.

## If you are a sim-to-real researcher

Start with [Part 10 Ch 05 — Sim-to-real bias correction](../100-optimization/05-sim-to-real.md) for how this stack handles the gap. Then [Part 1 Ch 04 — Real material data](../10-physical/04-material-data.md) for the constitutive data provenance — sim-to-real accuracy is bounded by the quality of the constitutive parameters fit to real materials, and the book is explicit about where those numbers come from. [Part 2 Ch 07 — Viscoelasticity](../20-materials/07-viscoelastic.md) and [Part 2 Ch 08 — Thermal coupling](../20-materials/08-thermal-coupling.md) are where the rate-dependent and thermal-drift corrections live.

## If you want the one-chapter summary

Read [Part 1 Ch 03 — the thesis](../10-physical/03-thesis.md) and stop. Every downstream commitment in the book is traceable to that chapter. The rest of the book is the consequences.

## If you disagree with the thesis

[Part 12 Ch 07 — Open research questions](../120-roadmap/07-open-questions.md) lists the places where the book's commitments are most contestable — differentiable meshing, IPC real-time at large scale, the SDF-to-FEM pipeline under live design edits. Reading that chapter first will tell you whether the book's disagreements are already the same as yours, or whether there is a new argument for you to make.
