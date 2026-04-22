# FEM assembly VJP

The global stiffness matrix $K$ is assembled per [Part 3 Ch 00](../../30-discretization/00-element-choice.md) as $K = \sum_e K^e$ with $K^e = B_e^T\, \mathbb{C}_e\, B_e\, V^e$ — a sum over thousands of per-element contributions. Every $K^e$ entry depends on the element's nodal positions $x_e$ (via the strain-displacement matrix $B_e$ and, for nonlinear materials, the material tangent $\mathbb{C}_e = \partial P / \partial F$ evaluated at $F_e(x_e)$) and on the element's material parameters $\mathbf{p}_e$ (via $\mathbb{C}_e$). The reverse-mode gradient $\partial L / \partial x_e$ and $\partial L / \partial \mathbf{p}_e$ given an upstream $\bar K = \partial L / \partial K$ is what the assembly VJP returns. This sub-leaf writes out the adjoint composition, names the cost, and states why autodiffing through the assembly with the chassis tape instead would be strictly worse.

## The forward is a sum over elements; the backward is too

The assembly's forward structure is a scatter-and-add: each element contributes a dense $12 \times 12$ block (Tet4; $30 \times 30$ for Tet10) into the sparse global matrix at the row/column positions given by the element's connectivity. Globally $K = \sum_e A_e^T\, K^e\, A_e$ where $A_e$ is the element-to-global scatter matrix (permutation-and-embedding; not stored as a dense matrix in practice but as a connectivity index list).

By linearity of differentiation, the adjoint of a sum of contributions is the sum of adjoints of contributions. Given $\bar K$, the per-element local adjoint is $\bar K^e = A_e\, \bar K\, A_e^T$ — extract the $12 \times 12$ (or $30 \times 30$) block from the sparse $\bar K$ at the element's connectivity positions. Then for each element independently:

$$ \bar x_e = \frac{\partial K^e}{\partial x_e} : \bar K^e, \qquad \bar{\mathbf{p}}_e = \frac{\partial K^e}{\partial \mathbf{p}_e} : \bar K^e $$

where `:` is the appropriate tensor contraction. The global nodal gradient $\bar x$ and material gradient $\bar{\mathbf{p}}$ are assembled back via the same connectivity: $\bar x = \sum_e A_e^T\, \bar x_e$. No cross-element coupling in the adjoint — the forward's locality transfers to the backward.

The per-element contraction composes three stages by chain rule. Writing it for the total-Lagrangian hyperelastic convention $\mathbb{C}_e = \partial P / \partial F$ that [Part 2 Ch 00](../../20-materials/00-trait-hierarchy.md) commits to — with $B_e$ the deformation-gradient-displacement matrix relating nodal positions $x_e$ to the flattened deformation gradient $F_e$:

$$ \bar K^e \longrightarrow \bar{\mathbb{C}}_e = V^e\, B_e\, \bar K^e\, B_e^T \longrightarrow \bar F_e = (\partial \mathbb{C}_e / \partial F_e)^T : \bar{\mathbb{C}}_e \longrightarrow \bar x_e = B_e^T\, \bar F_e $$

(with a parallel path through $\mathbf{p}_e$ for material-parameter gradients; the $\partial \mathbb{C}_e / \partial \mathbf{p}_e$ branch comes out of the same material-tangent callback). The last step uses $\partial F_e / \partial x_e = B_e$ because $F_e$ is linear in $x_e$ through the shape-function gradients; for Tet4 this identity is exact by the derivation in [Part 3 Ch 00 — Tet4](../../30-discretization/00-element-choice/00-tet4.md).

The composition is the standard reverse-mode contraction of the forward assembly's chain rule. Applied discretely to a FEM mesh, it is what every adjoint-FEM library does per element — including [Dolfin-adjoint / pyadjoint](https://www.dolfin-adjoint.org/) ([Mitusch, Funke, Dokken 2019](../../appendices/00-references/02-adjoint.md#mitusch-2019)), [Firedrake's adjoint](https://www.firedrakeproject.org/adjoint.html), and hand-rolled FEM-adjoint implementations across the geophysics-inversion community cited in [Plessix 2006](../../appendices/00-references/02-adjoint.md#plessix-2006). The derivation is the chain rule applied to a standard FEM assembly; the result is not a named theorem but a textbook composition whose forward half sits in [Hughes 2000](../../appendices/00-references/02-adjoint.md#hughes-2000).

## Cost: same order as the forward

The per-element forward does $O(d^2)$ work where $d$ is the per-element degree-of-freedom count (12 for Tet4, 30 for Tet10) — the dense $B_e^T \mathbb{C}_e B_e$ multiply dominates. The per-element backward does $O(d^2)$ work in the opposite direction — the same three factors contracted against the upstream $\bar K^e$. Summed over $N_e$ elements, both are $O(N_e\, d^2)$ with the same constant to leading order. The ratio forward:backward is on the order of 1:1 — the hand-derived adjoint costs about the same as the forward, modulo the constant factor from traversing the connectivity once more.

This is the result the [§00 registration API](00-registration.md) needs as its first concrete-case justification. The chassis-tape alternative is what the contrast surfaces.

## Why the chassis tape is the wrong level

A scalar chassis tape would record one node per scalar multiply inside each element's $B_e^T \mathbb{C}_e B_e V^e$ computation. Per Tet4 element, the $12 \times 12$ block contains 144 scalar entries, each the result of roughly $O(d)$ scalar multiplies (the middle matrix's inner dimension) — so a per-scalar tape records on the order of thousands of nodes per element. At a canonical-problem-sized mesh ($N_e \sim 10^4$ elements), one assembly produces a tape on the order of $10^7$ nodes. At roughly 32 bytes per node (back-of-envelope for the chassis's current `Node` layout — `f64` value plus tagged `BackwardOp`, whose largest variant carries two `u32` parents and two `f64` local derivatives), that is on the order of 300 MB of tape for a single $K$ assembly.

Tape *size* is the easy half of the story; tape *dispatch* is the harder half. Every one of those $10^7$ nodes' backward rule dispatches through an enum match and a gradient-slot read-modify-write. For a scalar tape that stores everything in a flat `Vec<Node>` — the chassis's current shape — the per-node constant is dominated by cache-line behavior rather than by arithmetic, so backward traversal cost scales with the number of nodes, not with the FLOPs. Practitioner observation at this mesh scale (shared across differentiable-physics toolchains including NVIDIA Warp, DiffTaichi, and Genesis; not formalized as a named complexity result) is that once tapes reach $10^6$-node size, per-node dispatch overhead is the dominant term, and recording at per-scalar granularity becomes the wrong granularity choice.

A tensor-level tape (Burn, PyTorch, JAX) compresses many scalar ops into one block-matrix node and keeps tape dispatch manageable. The chassis does not have a tensor-level tape — that is exactly the tradeoff [Ch 00 §00](../00-what-autograd-needs/00-generic.md) names as deliberate: scalar-f64 granularity is the right granularity for policy-network work, and the FEM assembly is the case that it does not serve. The hand-derived per-element VJP is the right granularity for this side: one opaque tape node per assembly, one backward closure that walks the connectivity list once and emits $\bar x$ and $\bar{\mathbf{p}}$ in a single pass.

## Registration shape

The assembly VJP fits the [§00 `VjpOp` trait](00-registration.md). All per-invocation state lives as struct fields on the impl — the per-element $B_e$ matrices (shape-function-gradient derivatives; reference-geometry-fixed; computed once at mesh-load time and reused across the episode), the element connectivity, the material-tangent callback $\partial P / \partial F$ that the [hyperelastic trait](../../20-materials/00-trait-hierarchy.md) already exposes, and the parent `Var` indices the tape routes cotangents to:

```rust
struct FemAssemblyVjp {
    parents:      Vec<u32>,                   // parent Var indices on the tape
    elements:     ElementType,
    b_matrices:   Vec<BMatrix>,               // shape per element-type + material convention
    connectivity: Arc<[ElementConnectivity]>,
    volumes:      Arc<[f64]>,                 // per-element reference volume V^e
    material:     Arc<dyn HyperelasticTangent>, // trait from Part 2 Ch 00
}
```

The caller computes $K$ externally from the current $x$ and $\mathbf{p}$ using the cached $B_e$'s (no tape recording) and passes the result to `Tape::push_custom` as the forward value. The `VjpOp::vjp` method receives $\bar K$ as the upstream cotangent and writes $\bar x$ and $\bar{\mathbf{p}}$ into the parent cotangent slots via the single-pass composition above. The captured state is tape-lifetime-scoped per [§00](00-registration.md); the `Box<dyn VjpOp>` drops when the tape drops.

In practice, the assembly VJP is rarely the outermost tape node. It is called from *inside* the [backward-Euler step's IFT wrapper](../02-implicit-function.md), which itself is the opaque tape node the outer graph sees. The FEM assembly adjoint is a utility that the IFT backward invokes when it needs $\partial r / \partial x$ or $\partial r / \partial \theta$ to close the adjoint linear system. The registration surface supports both usage shapes — a standalone tape-visible assembly node, or an internal utility — with the same trait and the same composition pattern.

## What this sub-leaf commits the book to

- **Per-element adjoint formulas are hand-written, per element type.** Tet4 ships in Phase A; Tet10 in Phase C (per the [build order](../../110-crate/03-build-order.md)); Hex8 or higher-order is deferred to post-Phase-E. Each type gets an `ElementAdjoint::backward` method that owns its closed-form composition; none of them route through the chassis tape's per-scalar ops.
- **Shape-function derivatives $B_e$ are cached at mesh-load, not recomputed per step.** Reference-geometry-fixed for Tet4; for higher-order elements with quadrature-point evaluation the $B_e$ *family* is cached (one matrix per quadrature point), not the full-reference-matrix-per-current-configuration. This matches the material-tangent API the [hyperelastic trait surface](../../20-materials/00-trait-hierarchy.md) uses.
- **Cost commitment: backward is same-order as forward.** Regression-tested by the [gradcheck suite](../../110-crate/04-testing/03-gradcheck.md) against central finite differences at each supported element type, and benchmarked in Phase B as a ratio of wall-clock-backward to wall-clock-forward. Divergence in that ratio by more than a small constant factor is a regression that fails the release gate.
- **The assembly VJP is an internal utility by default.** Tape-registration is optional: the IFT wrapper ([Ch 02](../02-implicit-function.md)) invokes it as a function, not as a tape-visible op. External callers who want the standalone assembly gradient (material-identification workflows in [Part 10 Ch 04](../../100-optimization/04-active-learning.md), for instance) can register it directly through [`push_custom`](00-registration.md). Both entry points use the same closed-form composition.
