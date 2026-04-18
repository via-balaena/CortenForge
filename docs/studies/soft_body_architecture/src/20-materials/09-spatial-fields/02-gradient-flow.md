# Gradient flow through SDF evaluation

The [Ch 09 parent's Claim 2](../09-spatial-fields.md) commits to differentiability per-sample: the optimizer adjusts SDF parameters $\theta$ (layer boundary positions, fillet radii, blend weights, fiber-direction parameters) and sees the [reward](../../10-physical/01-reward.md) move via gradients flowing through every per-element material evaluation. This leaf writes the chain, names the easy part (per-sample gradient through SDF evaluation) and the hard part (gradient through meshing), and commits to the FD-wrapper handling at topology-changing transitions.

## The gradient chain

For a scalar reward $R(\theta)$, the chain from SDF parameters $\theta$ to reward goes through every per-element material evaluation:

$$ \frac{\partial R}{\partial \theta} = \sum_e \frac{\partial R}{\partial \mathbf{p}_e}\, \frac{\partial \mathbf{p}_e}{\partial \theta} $$

with $\mathbf{p}_e$ the per-element scalar parameter vector ($\mu_e, \lambda_e, \alpha_e, \ldots$) at element $e$. The first factor — reward sensitivity to per-element parameters — flows through the [Newton-converged equilibrium](../../50-time-integration/00-backward-euler.md) and the [implicit-function-theorem adjoint](../../60-differentiability/02-implicit-function.md): standard physics-side autograd that the [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) caches the artifacts of. The second factor — per-element parameters' sensitivity to SDF parameters — is the SDF-evaluation gradient this leaf is about.

## Per-sample SDF gradient

For each element $e$ at sample point $x_\text{ref}^{(e)}$ ([per-element sampling sibling](01-sampling.md)), the per-element parameter $p_e$ is:

$$ p_e = \texttt{Sdf::sample}(x_\text{ref}^{(e)};\, \theta) $$

The gradient $\partial p_e / \partial \theta$ flows through the SDF's internal computation by reverse-mode autograd. SDF evaluation is a pipeline of scalar arithmetic operations (CSG min/max, smooth blends, distance computations) on the SDF parameters $\theta$ — every one of those operations is differentiable except at the measure-zero seams of `min`/`max`, which are smoothed in the [Part 7 Ch 00 SDF primitive operations](../../70-sdf-pipeline/00-sdf-primitive/01-operations.md). The reverse-mode pass through the SDF evaluation is plain autograd; no custom VJP is needed.

For unit-vector SDFs ([HGO fiber direction](../06-anisotropic/02-direction-fields.md)), the gradient picks up the normalization step ($\partial(\hat v / \|v\|) / \partial v$ projects out the radial component); for tensor-valued SDFs, the gradient picks up the symmetric-or-PSD projection. The chain at each typed `Sdf<T>` instance handles the per-type Jacobian; the consumer-side gradient flow is uniform.

## Easy part versus hard part

The per-sample gradient is the **easy part** — every link is differentiable, every operation is in the autograd-supported set, no custom VJP needed beyond the typed normalization/projection above. This is what the parent's Claim 2 calls "differentiability is per-sample" and what makes spatial material fields a clean fit for the [forward-map optimizer](../../100-optimization/00-forward.md).

The **hard part** — gradient through meshing — is [Part 6 Ch 05's open problem](../../60-differentiability/05-diff-meshing.md), and is not unique to material fields. Whenever the SDF parameter $\theta$ change moves the geometry envelope (not just the material assignment), re-meshing is triggered and the topology of the FEM mesh changes; the gradient through topology is the FD-wrapper case. For material-only changes (the geometry envelope is fixed and only the material SDFs vary), no re-meshing happens and the gradient flows through cleanly.

The [`EditResult`](../../70-sdf-pipeline/04-live-remesh/00-change-detection.md) classification at change-detection time decides which path: a material-only edit takes the clean per-sample gradient path; a topology-changing edit takes the FD-wrapper path with the [`GradientEstimate::Noisy`](../../60-differentiability/05-diff-meshing.md) flag set, and the stochastic-adjoint upgrade scheduled for [Phase H](../../110-crate/03-build-order.md) per [Part 6 Ch 03 sub-chapter 02](../../60-differentiability/03-time-adjoint/02-stochastic.md) is the planned long-term fix for the FD-wrapper coarse-grain.

## Discontinuous transitions

The [per-element sampling sibling](01-sampling.md) handles sub-element material discontinuities by requiring the meshing step to align element boundaries with the material-interface SDFs — no element straddles a discontinuity, every element samples cleanly. The gradient consequence: when the design parameter $\theta$ being differentiated is itself the position of a material-interface SDF, moving $\theta$ moves the interface, which forces re-meshing — the same FD-wrapper path as the geometry-change case above. Material-discontinuity-position parameters are a topology-changing parameter class.

## What this sub-leaf commits the book to

- **Per-sample SDF gradient is plain reverse-mode autograd, no custom VJP needed.** Smoothed `min`/`max` from [Part 7 Ch 00 operations](../../70-sdf-pipeline/00-sdf-primitive/01-operations.md) keeps every link differentiable; typed-SDF normalization/projection handles the per-type Jacobian.
- **Material-only design changes take the clean gradient path.** Geometry envelope unchanged → no re-meshing → per-sample gradient flows through.
- **Geometry-changing or material-interface-position changes take the FD-wrapper path.** [`EditResult`](../../70-sdf-pipeline/04-live-remesh/00-change-detection.md) classifies; topology-changing edits set [`GradientEstimate::Noisy`](../../60-differentiability/05-diff-meshing.md), with stochastic-adjoint upgrade scheduled for [Phase H](../../110-crate/03-build-order.md) per [Part 6 Ch 03](../../60-differentiability/03-time-adjoint/02-stochastic.md).
- **The hard part is meshing, not material.** Material-field gradient is the easy side of [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md)'s open problem; the open part is the meshing-gradient.
- **Mesh-aligned material interfaces avoid sub-element discontinuities.** Per the [meshing-step requirement](../../30-discretization/03-interfaces.md), no element straddles a material discontinuity; every per-element sample is on one side of every interface.
