# Oldroyd-B for large deformation

The [Prony series sibling](00-prony.md) inherits the small-strain linearity assumption: the convolutional integral $\int_0^t e^{-(t-s)/\tau} \dot P_\text{base}(F(s)) ds$ treats stored history as additive across timesteps in a fixed reference frame. At large deformation — large rotations, large stretches, large rates — that assumption breaks: the rotation of the deformation gradient between $t$ and $t-s$ rotates the stress contributions in a way the linear convolution does not capture, and the resulting total stress is no longer frame-indifferent. Oldroyd-B is the standard frame-indifferent extension: a tensor evolution equation built on the upper-convected derivative that handles arbitrary rotations and stretches without losing objectivity.

## Why Prony fails at large deformation

Frame indifference (objectivity) requires that the constitutive law commute with rigid-body rotations: under a superposed rotation $F \to R(t) F$, the first Piola stress must transform as $P \to R(t) P$. Prony's convolutional form does not satisfy this — the stored stresses $Q_i^n$ from previous timesteps are tensors written in the $t_n$ configuration, and adding them to $P_\text{base}(F^{n+1})$ at time $t_{n+1}$ assumes they have been rigidly transported, which the linear ODE does not enforce.

For small-strain regimes (rotations small, stretches close to identity), the discrepancy is negligible: the configuration changes slowly enough that the rotation is essentially absent. For large-rotation regimes — a sleeve folding on itself, a tube under finite torsion, an inflation past 2× — the convective rotation between adjacent timesteps becomes the dominant kinematic, and Prony's linearized history misattributes stress directions in a way that breaks objectivity.

## Upper-convected derivative

The upper-convected derivative of a contravariant tensor field $T$ in the spatial frame is:

$$ \overset{\nabla}{T} = \dot T - L\, T - T\, L^T $$

where $L = \dot F\, F^{-1}$ is the spatial velocity gradient. The construction subtracts the part of $\dot T$ that comes from the material configuration's rotation and stretch, leaving only the intrinsic constitutive rate. Under any superposed rigid rotation $R(t)$, $\overset{\nabla}{T}$ transforms tensorially the same way $T$ does — it is objective by construction, which is the property Prony's $\dot Q_i$ lacks.

The lower-convected derivative $\overset{\Delta}{T} = \dot T + L^T T + T L$ is the dual choice for covariant tensors; the Oldroyd-A model ([Oldroyd 1950](../../appendices/00-references/01-hyperelastic.md#oldroyd-1950)) uses the lower-convected derivative directly, and the [Johnson-Segalman model](../../appendices/00-references/01-hyperelastic.md#johnson-segalman-1977) parameterizes the interpolation between upper- and lower-convected. Oldroyd-B is the upper-convected case from the same [Oldroyd 1950](../../appendices/00-references/01-hyperelastic.md#oldroyd-1950) paper and is the standard for tensor stresses that transform contravariantly under stretch.

## The Oldroyd-B-style stress evolution

For each viscoelastic mode $i$, Oldroyd-B replaces Prony's linear stress ODE with a UCD-based tensor evolution in the Cauchy-stress frame:

$$ \overset{\nabla}{Q}_i + \frac{Q_i}{\tau_i} = 2\, G_i\, D $$

where $D = (L + L^T)/2$ is the symmetric rate-of-deformation tensor and $G_i$ is the over-stress shear modulus for mode $i$, fixed at construction from the dimensionless Prony weight $g_i$ and a representative small-strain shear modulus of the wrapped base material. The total Cauchy stress is $\sigma_\text{total} = \sigma_\text{base}(F) + \sum_i Q_i$ in the spatial frame; first Piola conversion $P = J\, \sigma_\text{total}\, F^{-T}$ recovers the per-element residual format the [Ch 00 trait surface](../00-trait-hierarchy/00-trait-surface.md) commits to.

The UCD term is what makes this frame-indifferent: under a superposed rotation, $L$ picks up the rotation rate, $D$ stays symmetric, and $\overset{\nabla}{Q}_i$ transforms tensorially with $Q_i$ — the equation rotates with the configuration without injecting spurious stress. This is the property the Prony convolution loses at large deformation.

The exact form `sim-soft` ships is the standard Oldroyd-B model from the polymer-rheology literature, generalized to $N$ modes; alternative UCD-family choices (Oldroyd-A, Johnson-Segalman, [Phan-Thien–Tanner](../../appendices/00-references/01-hyperelastic.md#phan-thien-tanner-1977)) exist for specific material classes but are not on the Phase A–I roadmap.

## Implementation as a parallel decorator

Oldroyd-B is a regime-specialization, not the default. `sim-soft` exposes it as a parallel decorator — `OldroydB<M>` — that wraps a base material the same way [`Viscoelastic<M>`](../00-trait-hierarchy/01-composition.md) does, with the same Prony-parameter shape but a UCD-form per-step ODE update:

```rust
pub struct OldroydB<M: Material> {
    base: M,
    prony: Vec<(f64 /* tau_i */, f64 /* g_i */)>,
}

impl<M: Material> Material for OldroydB<M> { /* UCD-form per-step update */ }
```

Both decorators store the same $(\tau_i, g_i)$ pairs and the same per-element $Q_i$ history layout in [`element/`](../../110-crate/00-module-layout/01-element.md); only the per-step ODE update differs. `Viscoelastic<M>` uses the [implicit-exponential Prony rule](00-prony.md); `OldroydB<M>` uses a backward-Euler discretization of the UCD-form evolution, with the velocity gradient $L^{n+1, k}$ from the current Newton iterate. Both implement `Material`; the trait surface and assembly contract are unchanged, so downstream consumers (the [Newton loop](../../50-time-integration/00-backward-euler.md), the autograd tape, the [tangent](02-tangent.md)) handle both variants without branching.

The user opts into Oldroyd-B by constructing `OldroydB<M>` instead of `Viscoelastic<M>` at material setup time. Mixed meshes can use both: a soft-rotation region wrapped in `Viscoelastic<NeoHookean>` and a large-stretch region in `OldroydB<NeoHookean>` are assembled into the same FEM system without solver-level branching.

## High Weissenberg number regime guard

Oldroyd-B has its own pathologies. The high Weissenberg number problem (Weissenberg number $\text{Wi} = \tau\, \dot\gamma$, the ratio of relaxation time to shear rate) describes a class of stability failures at $\text{Wi} \gtrsim O(1)$ — exponential growth of stress modes that defeat naive backward-Euler integration. Mitigations in the literature (log-conformation reformulation, square-root tensor representation) are extensive and out of `sim-soft`'s default scope; the [`GradientEstimate::Noisy`](../../60-differentiability/05-diff-meshing.md) flag is set when the Weissenberg number exceeds a per-material threshold so the optimizer treats those evaluations as untrusted rather than trusting potentially-divergent stresses.

The default validity-domain widening for `OldroydB<M>` is the strain-rate range, raised relative to Prony's small-strain ceiling but capped at the Weissenberg-number guard.

## What this sub-leaf commits the book to

- **Oldroyd-B is the large-deformation viscoelastic specialization, not the default.** Constructed as `OldroydB<M>` in parallel with `Viscoelastic<M>` (Prony); same Prony parameter shape and per-element history layout, different per-step ODE.
- **Frame indifference is the defining feature.** The upper-convected derivative makes the stress evolution objective under arbitrary rotations and stretches; this is what Prony's linear convolution loses at large deformation.
- **Cauchy-frame evolution, Piola conversion at assembly.** The UCD is naturally written in the spatial frame; the decorator converts to first Piola at the end of each per-element call to honour the [Ch 00 trait surface](../00-trait-hierarchy/00-trait-surface.md)'s contract.
- **High Weissenberg number is guarded, not solved.** Stable Oldroyd-B integration in the $\text{Wi} \gtrsim O(1)$ regime requires log-conformation or similar reformulations beyond `sim-soft`'s default scope; the per-material threshold and the noisy-gradient flag handle the boundary.
- **The UCD-family choice is locked to Oldroyd-B.** Alternative members (Oldroyd-A, Johnson-Segalman, Phan-Thien–Tanner) exist but are not on the Phase A–I roadmap.
