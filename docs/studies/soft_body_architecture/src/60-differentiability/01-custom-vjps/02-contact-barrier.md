# Contact barrier VJP

The IPC barrier $b(d, \hat d) = -(d - \hat d)^2 \ln(d / \hat d)$ on $(0, \hat d)$, zero elsewhere, per [Part 4 Ch 01 §00](../../40-contact/01-ipc-internals/00-barrier.md). Its closed-form first derivative $b'(d) = (\hat d - d)\left[2 \ln(d/\hat d) - \hat d/d + 1\right]$ and second derivative $b''(d) = -2 \ln(d/\hat d) + (\hat d/d)(\hat d/d + 2) - 3$ are derived in the same sub-leaf. The per-contact-pair VJP takes upstream $\bar b = \partial L / \partial b$ and returns $\bar d = \bar b \cdot b'(d)$. This sub-leaf writes the backward pattern, names two distinct numerical-precision regimes the naïve evaluation has to navigate, and specifies what stabilization machinery `sim-soft` adopts from the [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) ecosystem.

## Forward and backward in closed form

The per-pair forward is a scalar-in, scalar-out function of the gap $d$ — the distance function $d(x)$ is part of [Part 4 Ch 03 proximity detection](../../40-contact/03-self-contact/01-proximity.md) and lands in the tape as part of the composed contact energy, not inside this VJP. The barrier VJP receives $d$ as its input and produces $b(d, \hat d)$ as its output; the backward receives $\bar b$ and produces $\bar d = \bar b\, b'(d)$. The per-pair Hessian contribution to the Newton linear system picks up $b''(d)$ through the outer composition (the contact energy's Hessian assembly, not inside this VJP).

Both $b'(d)$ and $b''(d)$ are closed forms in $d, \hat d, \ln(d/\hat d)$, and $1/d$. The IPC Toolkit implements them directly in `src/ipc/barrier/barrier.cpp` using `std::log` and the written-out algebra — no log1p recentering, no Taylor expansion at the junction, no rearrangement beyond what the symbolic derivatives give. `sim-soft` adopts the identical derivative formulas. What the sim-soft VJP adds over "call the same closed form the toolkit calls" is the *registration surface* of [§00](00-registration.md) — the barrier evaluation becomes an opaque tape node with a captured barrier-variant tag and per-pair tolerance $\hat d$, rather than a free function called from outside the tape.

## Two distinct fragility regimes

The closed-form derivatives are mathematically smooth on $(0, \hat d)$ but evaluated in double precision they have two distinct fragility regimes.

**Near $d = \hat d$: catastrophic cancellation.** Both $(\hat d - d)$ and the bracket $[2 \ln(d/\hat d) - \hat d/d + 1]$ vanish to leading order in $\epsilon \equiv d - \hat d$. Expanding in small $\epsilon$: $\ln(d/\hat d) \approx \epsilon/\hat d - \epsilon^2/(2\hat d^2) + \ldots$, $\hat d/d \approx 1 - \epsilon/\hat d + \epsilon^2/\hat d^2 + \ldots$, so the bracket expands as $3\epsilon/\hat d - 2\epsilon^2/\hat d^2 + O(\epsilon^3)$ — linear in $\epsilon$, with the constant-order terms ($2\cdot 0 - 1 + 1$) cancelling exactly only when all three contributions are representable at the same scale. In the interior of the active band the direct evaluation is fine; within a few ULPs of $d = \hat d$, the bracket computes by subtracting two near-unity terms whose difference agrees to roughly $\epsilon/\hat d$ precision relative to 1, so the result's relative precision degrades as $\epsilon$ shrinks. The final product $(\hat d - d) \cdot [\text{near-zero bracket}]$ then carries less accuracy than either factor had in isolation. This is standard floating-point catastrophic-cancellation behavior ([Goldberg 1991](https://doi.org/10.1145/103162.103163)), not IPC-specific; Li 2020 does not discuss it explicitly, and IPC Toolkit's source does not branch on it.

**Near $d \to 0^+$: Hessian ill-conditioning.** By design $b''(d) \propto 1/d^2$ diverges as the gap closes — that is the barrier's entire mechanism for preventing penetration. The Hessian terms the Newton solver needs are therefore unbounded-above near contact; their condition number grows without bound as $d$ shrinks. Floating-point precision is bounded, so the Hessian's contribution to the Newton linear system becomes progressively ill-conditioned as $d / \hat d$ shrinks. [GIPC (Huang et al. 2024)](../../appendices/00-references/00-ipc.md#gipc-2024) documents this directly as a motivation for their analytic-eigensystem reformulation — the barrier Hessian becomes progressively ill-conditioned as contact closes, and the per-pair eigendecomposition GIPC ships restores the assembly's conditioning without changing the underlying barrier. The mathematical values are well-defined inside floating-point range; the linear-solve precision at which downstream machinery consumes them is not.

The two regimes are different problems. The junction fragility is a *forward*-evaluation precision issue — the derivative values themselves lose accuracy. The $d \to 0^+$ regime is a *downstream conditioning* issue — the Hessian values are computed accurately, but they push the Newton linear system toward numerical pathology.

## Stabilization techniques the ecosystem uses

The IPC ecosystem has three distinct stabilization strategies; `sim-soft` picks from them based on the regime.

**Barrier-variant choice.** [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) ships five barrier variants in `src/ipc/barrier/barrier.{hpp,cpp}`: (1) `ClampedLogBarrier`, the Li 2020 original $-(d-\hat d)^2 \ln(d/\hat d)$; (2) `NormalizedClampedLogBarrier`, which rescales to $-(d/\hat d - 1)^2 \ln(d/\hat d)$; (3) `ClampedLogSqBarrier`, which replaces $\ln$ with $\ln^2$ as $(d-\hat d)^2 \ln^2(d/\hat d)$; (4) `CubicBarrier`, a pure polynomial $-\tfrac{2}{3\hat d}(d-\hat d)^3$ with no log at all; and (5) `TwoStageBarrier`, a piecewise variant — logarithmic for $d < \hat d / 2$, quadratic for $\hat d / 2 \le d < \hat d$ — combining divergent enforcement near contact with smooth truncation near the junction. The quadratic-log and cubic variants trade the logarithmic divergence for slower growth — for stiff scenes, the cubic variant gives a bounded Hessian at the cost of weaker enforcement, which can matter more than the divergence guarantee.

**Analytic Hessian eigensystem.** [GIPC (Huang et al. 2024)](../../appendices/00-references/00-ipc.md#gipc-2024) derives a closed-form eigendecomposition of the per-pair barrier Hessian from simplicial contact-measure geometry — the per-pair Hessian is written as an analytic eigenbasis-plus-eigenvalue pair rather than a numerical eigensolve. That replaces the ill-conditioned direct assembly with a computation that stays well-conditioned in the eigenbasis. It is the current GPU-IPC state of the art; [StiffGIPC (Huang et al. 2025)](../../appendices/00-references/00-ipc.md#stiffgipc-2025) extends it with a multilevel Schwarz preconditioner for stiff materials.

**Trust-double-precision with conditional guards.** Direct evaluation of $b, b', b''$ with no algebraic rearrangement, supplemented by $d \ge \hat d \Rightarrow b = 0$ and $d \le 0 \Rightarrow$ unreachable-by-invariant guards. This is what [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) does in the reference implementation — `std::log` directly, no log1p, no Taylor-expanded fallback. The toolkit trusts that the junction cancellation and the Hessian conditioning are manageable at the precision regime Newton runs at with adaptive barrier stiffness from [Part 4 Ch 01 §01](../../40-contact/01-ipc-internals/01-adaptive-kappa.md) and adaptive barrier width from [Part 4 Ch 05 §01](../../40-contact/05-real-time/01-barrier-width.md).

## sim-soft's commitment

`sim-soft`'s per-pair VJP ships the closed-form derivatives of the chosen barrier variant, evaluated directly in double precision the way [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) does. The barrier-variant choice is a [Phase B build-order](../../110-crate/03-build-order.md) decision, not an architectural commitment — the default is `ClampedLogBarrier` (Li 2020 original, matches the formula used throughout [Part 4](../../40-contact/01-ipc-internals.md)); the fallback is `CubicBarrier` or `TwoStageBarrier` if the [gradcheck suite](../../110-crate/04-testing/03-gradcheck.md) or the condition-number regression test surfaces precision issues on the canonical problem. The GIPC analytic-eigensystem reformulation is noted as a Phase E GPU-port option — the port pressure at that layer is different from the CPU Newton's, and GIPC's eigensystem is specifically what made GPU-IPC tractable.

The registration shape per the [§00 trait](00-registration.md):

```rust
struct BarrierVjp { variant: BarrierVariant }

struct BarrierState {
    dhat: f64,          // per-pair tolerance (adaptive per Part 4 Ch 05 §01)
    variant: BarrierVariant,
}

impl CustomVjp for BarrierVjp {
    type State = BarrierState;
    fn forward(&self, state: &BarrierState, inputs: &[f64]) -> Vec<f64> {
        // inputs = [d]; compute b(d, state.dhat) per state.variant; return [b].
    }
    fn backward(&self, state: &BarrierState, inputs: &[f64],
                _outputs: &[f64], grad_outputs: &[f64]) -> Vec<f64> {
        // return [grad_outputs[0] * b_prime(inputs[0], state.dhat, state.variant)].
    }
}
```

The per-pair VJP is rarely registered as a standalone tape node. The dominant consumer is the [IFT wrapper](../02-implicit-function.md) — the backward-Euler step's opaque tape node — which composes the contact-energy gradient across all active pairs internally. The per-pair VJP is a utility called by that wrapper's backward, same pattern as the [FEM assembly VJP](01-fem-assembly.md). Standalone registration exists for material-identification workflows in [Part 10](../../100-optimization/04-active-learning.md) that want pair-wise barrier gradients exposed; the registration surface supports both.

## What this sub-leaf commits the book to

- **The closed-form derivatives are not algebraically rearranged.** `sim-soft` evaluates $b, b', b''$ directly as [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) does. The earlier speculative framing — log1p-style recentering, Taylor expansion at the junction — is dropped: IPC Toolkit's reference implementation does neither, and no paper in the IPC line (Li 2020, C-IPC 2021, GIPC 2024, StiffGIPC 2025) documents those as stabilization techniques. The junction fragility is a floating-point cancellation ([Goldberg 1991](https://doi.org/10.1145/103162.103163)) the toolkit accepts; the $d \to 0^+$ Hessian conditioning is a GPU-layer concern ([GIPC](../../appendices/00-references/00-ipc.md#gipc-2024)), not a CPU-VJP one.
- **Barrier-variant choice is a Phase B tuning knob, not an architectural commitment.** Default is `ClampedLogBarrier` (Li 2020 original). Fallbacks to `CubicBarrier` or `TwoStageBarrier` are unlocked by the [gradcheck suite](../../110-crate/04-testing/03-gradcheck.md) finding precision pathologies on the canonical problem. The book does not fix the variant for the reader; the engine exposes the full IPC-Toolkit-compatible menu (five variants per `src/ipc/barrier/barrier.{hpp,cpp}`).
- **GIPC's analytic-eigensystem reformulation is a Phase E GPU-port option, not a CPU commitment.** The CPU path trusts direct evaluation; the GPU port may adopt GIPC's eigensystem if the `sim-soft` GPU Newton's conditioning requires it. That decision is staged with the [Phase E wgpu kernels](../../80-gpu/04-chassis-extension.md), not anchored in this chapter.
- **Per-pair $\hat d$ comes from the scene-level aggregation rule in [Part 4 Ch 01 §00](../../40-contact/01-ipc-internals/00-barrier.md).** The VJP's `State` captures the resolved per-pair $\hat d_k$ (the `sim-soft` $\min$-aggregation of per-primitive $\hat d_{v_a}, \hat d_{v_b}$, per the [notation appendix](../../appendices/03-notation.md)) at the forward-evaluation site; adaptive updates across Newton iterations re-register the pair.
