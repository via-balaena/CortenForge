//! Tape-based reverse-mode automatic differentiation.
//!
//! A minimal, CPU-only autograd engine. Builds a computation graph during
//! the forward pass, then walks it in reverse to compute gradients via the
//! chain rule. Designed for transparency — every operation's gradient rule
//! is visible, every node is inspectable.
//!
//! # Storage model
//!
//! Nodes carry [`Tensor<f64>`](crate::tensor::Tensor) values; a "scalar" is
//! shape `[]`. The scalar API ([`Tape::param`], [`Tape::value`],
//! [`Tape::grad`], and the nine primitive ops) lifts and unwraps rank-0
//! tensors transparently — RL consumers that work in scalar `f64` keep their
//! call sites untouched. Physics consumers push arbitrary-shape tensors via
//! [`Tape::param_tensor`] / [`Tape::push_custom`] and custom [`VjpOp`]s.
//!
//! # Architecture
//!
//! ```text
//! Tape   — columnar: parallel Vec<Tensor<f64>> for values and grads,
//!          Vec<BackwardOp> for backward rules, Vec<bool> for leaf mask.
//! Var    — lightweight index into the tape (Copy, 4 bytes).
//! ```
//!
//! The tape is topologically sorted by construction: each operation can only
//! reference previously created nodes. `backward()` is a simple reverse
//! iteration — no sort, no BFS, no graph traversal.
//!
//! # Supported operations
//!
//! Nine elementwise primitives, strict same-shape: [`Tape::add`],
//! [`Tape::sub`], [`Tape::mul`], [`Tape::neg`], [`Tape::tanh`],
//! [`Tape::relu`], [`Tape::square`], [`Tape::ln`], [`Tape::exp`].
//!
//! Three fused compositions: [`Tape::affine`], [`Tape::sum`], [`Tape::mean`]
//! — built from primitives, gradients follow automatically.
//!
//! Custom VJPs via [`Tape::push_custom`] + [`VjpOp`]: the extension point
//! for ops chassis can't express as primitive compositions (e.g. sim-soft
//! physics-backed Newton replay, faer factorizations, sparse matvecs).
//!
//! # What this module does NOT do
//!
//! - No GPU tensors — CPU `f64` only. (GPU path enters chassis behind the
//!   `gpu` feature flag per B.5.)
//! - No graph optimizations — transparency over performance.
//! - No broadcasting in primitive ops — strict same-shape. Broadcast-style
//!   ops register as `VjpOp`s when a consumer needs them.
//! - No operator overloading — explicit method calls on `&mut Tape`.

use std::fmt::Debug;

use crate::tensor::Tensor;

// ── VjpOp trait ───────────────────────────────────────────────────────────

/// A custom vector-Jacobian product op, plugged into the tape via
/// [`Tape::push_custom`].
///
/// Built-in primitives (add, mul, tanh, ...) do not use this trait — they
/// dispatch through `BackwardOp::Unary` / `BackwardOp::Binary` for
/// zero-allocation inspection and `Copy`-free storage (those variants are
/// module-private implementation detail). `VjpOp` is the extension point
/// for ops chassis can't express as compositions of primitives — sim-soft
/// physics-backed Newton replay, faer factorizations, sparse matvecs.
///
/// Implementors stash whatever primal data the VJP needs (input tensors,
/// cached factorizations, integer tables) at construction time. Parent
/// tape indices are tracked by the tape itself: callers pass parents as
/// `&[Var]` to [`Tape::push_custom`], and the backward pass feeds
/// `parent_cotans` to [`vjp`](Self::vjp) in the same order — matching
/// JAX's `custom_vjp` and `PyTorch`'s `autograd.Function`, where the
/// framework owns parent tracking rather than the op.
///
/// # Contract
///
/// - [`vjp`](Self::vjp) receives a `cotangent` (the gradient flowing into
///   this node's output) and `parent_cotans`, a mutable slice of the same
///   length as the `parents` slice passed to [`Tape::push_custom`]. Slot
///   `k` corresponds to `parents[k]`; duplicates are permitted (fan-in
///   with shared parents). Slots are zero-initialized by the caller with
///   shapes matching the parent values. Implementors add (not overwrite)
///   their contributions — `parent_cotans[k] += ∂f/∂x_k · cot`.
/// - [`op_id`](Self::op_id) returns a stable string identity. Used today
///   for debug/logging; futureproofs potential tape serialization.
///
/// `Send + Sync` are supertrait bounds (per B.1.a); any `dyn VjpOp` is
/// automatically `Send + Sync`.
pub trait VjpOp: Send + Sync + Debug {
    /// Stable identity for this op class. Typically a module-qualified
    /// string literal, e.g. `"sim_soft::autograd::CheckpointedStep"`.
    fn op_id(&self) -> &'static str;

    /// Compute parent cotangents and accumulate them into `parent_cotans`.
    ///
    /// `parent_cotans.len()` matches the `parents` slice passed to
    /// [`Tape::push_custom`]; slot `k` corresponds to `parents[k]`. Slots
    /// are zero-initialized by the caller with shapes matching the parent
    /// values. Implementors add (not overwrite) their contributions —
    /// `parent_cotans[k] += ∂f/∂x_k · cot`.
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]);
}

// ── Types ─────────────────────────────────────────────────────────────────

/// Index into a [`Tape`]. Lightweight and `Copy`.
///
/// Created by [`Tape::param`] / [`Tape::constant`] / [`Tape::param_tensor`] /
/// [`Tape::constant_tensor`] for leaf values, or by any operation method
/// (e.g. [`Tape::add`]) or [`Tape::push_custom`] for intermediate values.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Var(u32);

/// Unary gradient rule. Each variant stashes the primal data the VJP needs.
#[derive(Debug)]
enum UnaryRule {
    /// vjp: `-cot`
    Neg,
    /// vjp: `(1 - out²) ⊙ cot`
    Tanh { out: Tensor<f64> },
    /// vjp: `mask ⊙ cot`, where `mask[i] = 1 if input[i] > 0 else 0`
    Relu { mask: Tensor<f64> },
    /// vjp: `2·input ⊙ cot`
    Square { input: Tensor<f64> },
    /// vjp: `cot / input`
    Ln { input: Tensor<f64> },
    /// vjp: `out ⊙ cot`
    Exp { out: Tensor<f64> },
}

/// Binary gradient rule. Each variant stashes the primal data the VJP needs.
#[derive(Debug)]
enum BinaryRule {
    /// vjp: `cot → lhs`, `cot → rhs`
    Add,
    /// vjp: `cot → lhs`, `-cot → rhs`
    Sub,
    /// vjp: `rhs_val ⊙ cot → lhs`, `lhs_val ⊙ cot → rhs`
    Mul {
        lhs_val: Tensor<f64>,
        rhs_val: Tensor<f64>,
    },
}

/// How a node propagates gradient to its parents.
///
/// Built-in primitives are inspectable sum-type variants; custom ops go
/// through the [`VjpOp`] trait object. Hybrid layout per B.1.a: primitives
/// stay allocation-free and pattern-matchable, only custom ops pay one
/// allocation per node.
#[derive(Debug)]
enum BackwardOp {
    /// Leaf node (parameter or constant) — no parents.
    Leaf,
    /// One parent. Dispatch via `rule`.
    Unary { parent: u32, rule: UnaryRule },
    /// Two parents. Dispatch via `rule`.
    Binary {
        lhs: u32,
        rhs: u32,
        rule: BinaryRule,
    },
    /// Custom VJP. Used for ops that can't be expressed as primitive
    /// compositions (e.g. sim-soft physics ops). Parents are tracked by
    /// the tape at [`Tape::push_custom`] time, not by the op — keeping
    /// the two sources of truth merged.
    Custom {
        parents: Box<[Var]>,
        op: Box<dyn VjpOp>,
    },
}

/// The computation tape. One per forward pass, discarded after backward.
///
/// # Usage
///
/// ```
/// use sim_ml_chassis::autograd::{Tape, Var};
///
/// let mut tape = Tape::new();
/// let x = tape.param(3.0);
/// let y = tape.param(2.0);
/// let z = tape.mul(x, y);   // z = x * y = 6
/// tape.backward(z);
///
/// assert!((tape.grad(x) - 2.0).abs() < 1e-12); // dz/dx = y = 2
/// assert!((tape.grad(y) - 3.0).abs() < 1e-12); // dz/dy = x = 3
/// ```
pub struct Tape {
    values: Vec<Tensor<f64>>,
    grads: Vec<Tensor<f64>>,
    backward: Vec<BackwardOp>,
    leaf_mask: Vec<bool>,
}

// ── Elementwise helpers (private) ─────────────────────────────────────────

/// Lift an `f64` to a shape-`[]` `Tensor<f64>`.
fn scalar_tensor(v: f64) -> Tensor<f64> {
    Tensor::from_slice(&[v], &[])
}

fn ew_neg(a: &Tensor<f64>) -> Tensor<f64> {
    let data: Vec<f64> = a.as_slice().iter().map(|&x| -x).collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_tanh(a: &Tensor<f64>) -> Tensor<f64> {
    let data: Vec<f64> = a.as_slice().iter().map(|&x| x.tanh()).collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_square(a: &Tensor<f64>) -> Tensor<f64> {
    let data: Vec<f64> = a.as_slice().iter().map(|&x| x * x).collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_ln(a: &Tensor<f64>) -> Tensor<f64> {
    // Positivity enforced by the caller.
    let data: Vec<f64> = a.as_slice().iter().map(|&x| x.ln()).collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_exp(a: &Tensor<f64>) -> Tensor<f64> {
    let data: Vec<f64> = a.as_slice().iter().map(|&x| x.exp()).collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_relu(a: &Tensor<f64>) -> Tensor<f64> {
    let data: Vec<f64> = a.as_slice().iter().map(|&x| x.max(0.0)).collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_relu_mask(a: &Tensor<f64>) -> Tensor<f64> {
    let data: Vec<f64> = a
        .as_slice()
        .iter()
        .map(|&x| if x > 0.0 { 1.0 } else { 0.0 })
        .collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_add(a: &Tensor<f64>, b: &Tensor<f64>) -> Tensor<f64> {
    assert!(
        a.shape() == b.shape(),
        "Tape: shape mismatch in add: lhs {:?} vs rhs {:?}",
        a.shape(),
        b.shape(),
    );
    let data: Vec<f64> = a
        .as_slice()
        .iter()
        .zip(b.as_slice().iter())
        .map(|(&x, &y)| x + y)
        .collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_sub(a: &Tensor<f64>, b: &Tensor<f64>) -> Tensor<f64> {
    assert!(
        a.shape() == b.shape(),
        "Tape: shape mismatch in sub: lhs {:?} vs rhs {:?}",
        a.shape(),
        b.shape(),
    );
    let data: Vec<f64> = a
        .as_slice()
        .iter()
        .zip(b.as_slice().iter())
        .map(|(&x, &y)| x - y)
        .collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_mul(a: &Tensor<f64>, b: &Tensor<f64>) -> Tensor<f64> {
    assert!(
        a.shape() == b.shape(),
        "Tape: shape mismatch in mul: lhs {:?} vs rhs {:?}",
        a.shape(),
        b.shape(),
    );
    let data: Vec<f64> = a
        .as_slice()
        .iter()
        .zip(b.as_slice().iter())
        .map(|(&x, &y)| x * y)
        .collect();
    Tensor::from_slice(&data, a.shape())
}

fn ew_add_assign(dst: &mut Tensor<f64>, src: &Tensor<f64>) {
    assert!(
        dst.shape() == src.shape(),
        "Tape: shape mismatch in grad accumulate: dst {:?} vs src {:?}",
        dst.shape(),
        src.shape(),
    );
    for (d, &s) in dst.as_mut_slice().iter_mut().zip(src.as_slice().iter()) {
        *d += s;
    }
}

// ── Tape construction ─────────────────────────────────────────────────────

impl Tape {
    /// Create a new empty tape.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            values: Vec::new(),
            grads: Vec::new(),
            backward: Vec::new(),
            leaf_mask: Vec::new(),
        }
    }

    /// Append a node and return its `Var` index.
    fn push(&mut self, value: Tensor<f64>, backward: BackwardOp, is_leaf: bool) -> Var {
        let idx = self.values.len();
        assert!(
            idx < u32::MAX as usize,
            "Tape overflow: more than {} nodes",
            u32::MAX,
        );
        let grad = Tensor::zeros(value.shape());
        self.values.push(value);
        self.grads.push(grad);
        self.backward.push(backward);
        self.leaf_mask.push(is_leaf);
        #[allow(clippy::cast_possible_truncation)] // guarded by assert above
        Var(idx as u32)
    }

    // ── scalar API (shape-[] sugar) ──────────────────────────────────────

    /// Create a scalar parameter — a shape-`[]` leaf whose gradient is kept
    /// after backward.
    #[must_use]
    pub fn param(&mut self, value: f64) -> Var {
        self.push(scalar_tensor(value), BackwardOp::Leaf, true)
    }

    /// Create a scalar constant — a shape-`[]` leaf whose gradient is
    /// discarded.
    #[must_use]
    pub fn constant(&mut self, value: f64) -> Var {
        self.push(scalar_tensor(value), BackwardOp::Leaf, false)
    }

    /// Read the scalar forward value of a node.
    ///
    /// # Panics
    ///
    /// Panics if the node's value is not shape-`[]`. Use
    /// [`value_tensor`](Self::value_tensor) for non-scalar nodes.
    #[must_use]
    pub fn value(&self, v: Var) -> f64 {
        let t = &self.values[v.0 as usize];
        assert!(
            t.shape().is_empty(),
            "Tape::value requires shape [], got {:?}; use value_tensor instead",
            t.shape(),
        );
        t.as_slice()[0]
    }

    /// Read the scalar gradient of a node (populated after
    /// [`backward`](Self::backward)).
    ///
    /// # Panics
    ///
    /// Panics if the node's value is not shape-`[]`. Use
    /// [`grad_tensor`](Self::grad_tensor) for non-scalar nodes.
    #[must_use]
    pub fn grad(&self, v: Var) -> f64 {
        let t = &self.grads[v.0 as usize];
        assert!(
            t.shape().is_empty(),
            "Tape::grad requires shape [], got {:?}; use grad_tensor instead",
            t.shape(),
        );
        t.as_slice()[0]
    }

    // ── tensor API ───────────────────────────────────────────────────────

    /// Create a tensor parameter — a leaf whose gradient is kept after
    /// backward. Any shape, including shape-`[]` scalars.
    #[must_use]
    pub fn param_tensor(&mut self, value: Tensor<f64>) -> Var {
        self.push(value, BackwardOp::Leaf, true)
    }

    /// Create a tensor constant — a leaf whose gradient is discarded. Any
    /// shape, including shape-`[]` scalars.
    #[must_use]
    pub fn constant_tensor(&mut self, value: Tensor<f64>) -> Var {
        self.push(value, BackwardOp::Leaf, false)
    }

    /// Read the forward-value tensor of a node (any shape).
    #[must_use]
    pub fn value_tensor(&self, v: Var) -> &Tensor<f64> {
        &self.values[v.0 as usize]
    }

    /// Read the gradient tensor of a node (any shape; populated after
    /// [`backward`](Self::backward)).
    #[must_use]
    pub fn grad_tensor(&self, v: Var) -> &Tensor<f64> {
        &self.grads[v.0 as usize]
    }

    // ── custom VJP ───────────────────────────────────────────────────────

    /// Append a custom op to the tape.
    ///
    /// - `parents` — the tape nodes whose cotangents `op.vjp` will accumulate
    ///   into, in declaration order. Duplicates are permitted (fan-in with
    ///   shared parents). The slice is copied onto the tape, so callers do
    ///   not need to keep it alive past this call.
    /// - `value` — the op's forward output, any shape.
    /// - `op` — the VJP implementation.
    ///
    /// The extension point for ops that can't be expressed as primitive
    /// compositions. See [`VjpOp`] for the contract.
    #[must_use]
    pub fn push_custom(&mut self, parents: &[Var], value: Tensor<f64>, op: Box<dyn VjpOp>) -> Var {
        let parents: Box<[Var]> = Box::from(parents);
        self.push(value, BackwardOp::Custom { parents, op }, false)
    }
}

// ── Elementwise primitives (9 ops) ────────────────────────────────────────
//
// All primitives are strict same-shape elementwise. Broadcasting is not
// supported in primitive ops — see module doc.

impl Tape {
    /// `a + b` (elementwise). Gradients: `da = cot`, `db = cot`.
    ///
    /// # Panics
    ///
    /// Panics if `a` and `b` have different shapes.
    #[must_use]
    pub fn add(&mut self, a: Var, b: Var) -> Var {
        let val = ew_add(&self.values[a.0 as usize], &self.values[b.0 as usize]);
        self.push(
            val,
            BackwardOp::Binary {
                lhs: a.0,
                rhs: b.0,
                rule: BinaryRule::Add,
            },
            false,
        )
    }

    /// `a − b` (elementwise). Gradients: `da = cot`, `db = -cot`.
    ///
    /// # Panics
    ///
    /// Panics if `a` and `b` have different shapes.
    #[must_use]
    pub fn sub(&mut self, a: Var, b: Var) -> Var {
        let val = ew_sub(&self.values[a.0 as usize], &self.values[b.0 as usize]);
        self.push(
            val,
            BackwardOp::Binary {
                lhs: a.0,
                rhs: b.0,
                rule: BinaryRule::Sub,
            },
            false,
        )
    }

    /// `a × b` (elementwise). Gradients: `da = b ⊙ cot`, `db = a ⊙ cot`.
    ///
    /// # Panics
    ///
    /// Panics if `a` and `b` have different shapes.
    #[must_use]
    pub fn mul(&mut self, a: Var, b: Var) -> Var {
        let lhs_val = self.values[a.0 as usize].clone();
        let rhs_val = self.values[b.0 as usize].clone();
        let val = ew_mul(&lhs_val, &rhs_val);
        self.push(
            val,
            BackwardOp::Binary {
                lhs: a.0,
                rhs: b.0,
                rule: BinaryRule::Mul { lhs_val, rhs_val },
            },
            false,
        )
    }

    /// `−a` (elementwise). Gradient: `da = -cot`.
    #[must_use]
    pub fn neg(&mut self, a: Var) -> Var {
        let val = ew_neg(&self.values[a.0 as usize]);
        self.push(
            val,
            BackwardOp::Unary {
                parent: a.0,
                rule: UnaryRule::Neg,
            },
            false,
        )
    }

    /// `tanh(a)` (elementwise). Gradient: `da = (1 − tanh²(a)) ⊙ cot`.
    #[must_use]
    pub fn tanh(&mut self, a: Var) -> Var {
        let out = ew_tanh(&self.values[a.0 as usize]);
        self.push(
            out.clone(),
            BackwardOp::Unary {
                parent: a.0,
                rule: UnaryRule::Tanh { out },
            },
            false,
        )
    }

    /// `a²` (elementwise). Gradient: `da = 2·a ⊙ cot`.
    #[must_use]
    pub fn square(&mut self, a: Var) -> Var {
        let input = self.values[a.0 as usize].clone();
        let out = ew_square(&input);
        self.push(
            out,
            BackwardOp::Unary {
                parent: a.0,
                rule: UnaryRule::Square { input },
            },
            false,
        )
    }

    /// `ln(a)` (elementwise). Gradient: `da = cot / a`.
    ///
    /// # Panics
    ///
    /// Panics if any element of `a` is non-positive (logarithm undefined).
    #[must_use]
    pub fn ln(&mut self, a: Var) -> Var {
        let input = self.values[a.0 as usize].clone();
        for &x in input.as_slice() {
            assert!(x > 0.0, "Tape::ln: input must be positive, got {x}");
        }
        let out = ew_ln(&input);
        self.push(
            out,
            BackwardOp::Unary {
                parent: a.0,
                rule: UnaryRule::Ln { input },
            },
            false,
        )
    }

    /// `exp(a)` (elementwise). Gradient: `da = exp(a) ⊙ cot`.
    #[must_use]
    pub fn exp(&mut self, a: Var) -> Var {
        let out = ew_exp(&self.values[a.0 as usize]);
        self.push(
            out.clone(),
            BackwardOp::Unary {
                parent: a.0,
                rule: UnaryRule::Exp { out },
            },
            false,
        )
    }

    /// `max(0, a)` (elementwise). Gradient: `da = mask ⊙ cot`, where
    /// `mask[i] = 1 if a[i] > 0 else 0`.
    #[must_use]
    pub fn relu(&mut self, a: Var) -> Var {
        let va = &self.values[a.0 as usize];
        let out = ew_relu(va);
        let mask = ew_relu_mask(va);
        self.push(
            out,
            BackwardOp::Unary {
                parent: a.0,
                rule: UnaryRule::Relu { mask },
            },
            false,
        )
    }
}

// ── Fused operations ──────────────────────────────────────────────────────

impl Tape {
    /// Sum a slice of variables. Derived from repeated [`add`](Self::add);
    /// all inputs must share the same shape.
    ///
    /// # Panics
    ///
    /// Panics if `vars` is empty, or if the inputs have mismatched shapes.
    #[must_use]
    pub fn sum(&mut self, vars: &[Var]) -> Var {
        assert!(!vars.is_empty(), "Tape::sum: empty input");
        let mut acc = vars[0];
        for &v in &vars[1..] {
            acc = self.add(acc, v);
        }
        acc
    }

    /// Mean of a slice of scalar variables (shape `[]`). Gradient: each
    /// input gets `1/n · cot`.
    ///
    /// # Panics
    ///
    /// Panics if `vars` is empty, or if any input is non-scalar (same-shape
    /// multiply against the scalar `1/n` requires all-scalar inputs).
    #[must_use]
    pub fn mean(&mut self, vars: &[Var]) -> Var {
        assert!(!vars.is_empty(), "Tape::mean: empty input");
        let s = self.sum(vars);
        #[allow(clippy::cast_precision_loss)] // n < 2^52
        let inv_n = self.constant(1.0 / vars.len() as f64);
        self.mul(s, inv_n)
    }

    /// Affine transform: `z[i] = Σ_j W[i*cols + j] · x[j] + b[i]`.
    ///
    /// `w` is row-major `[rows × cols]`, `x` is `[cols]`, `b` is `[rows]`;
    /// all inputs are scalar `Var`s (shape `[]`). Returns `[rows]` output
    /// variables.
    ///
    /// Built from [`mul`](Self::mul), [`sum`](Self::sum), and
    /// [`add`](Self::add) — gradients are automatically correct via
    /// composition.
    ///
    /// # Panics
    ///
    /// Panics if `w.len() != rows * cols`, `x.len() != cols`, or
    /// `b.len() != rows`.
    #[must_use]
    pub fn affine(
        &mut self,
        w: &[Var],
        x: &[Var],
        b: &[Var],
        rows: usize,
        cols: usize,
    ) -> Vec<Var> {
        assert!(
            w.len() == rows * cols,
            "Tape::affine: w.len() ({}) != rows ({rows}) * cols ({cols})",
            w.len(),
        );
        assert!(
            x.len() == cols,
            "Tape::affine: x.len() ({}) != cols ({cols})",
            x.len(),
        );
        assert!(
            b.len() == rows,
            "Tape::affine: b.len() ({}) != rows ({rows})",
            b.len(),
        );
        (0..rows)
            .map(|i| {
                let products: Vec<Var> =
                    (0..cols).map(|j| self.mul(w[i * cols + j], x[j])).collect();
                let dot = self.sum(&products);
                self.add(dot, b[i])
            })
            .collect()
    }
}

// ── Backward pass ─────────────────────────────────────────────────────────

fn unary_vjp(rule: &UnaryRule, cot: &Tensor<f64>) -> Tensor<f64> {
    match rule {
        UnaryRule::Neg => ew_neg(cot),
        UnaryRule::Tanh { out } => {
            assert!(
                out.shape() == cot.shape(),
                "Tape::backward: tanh shape mismatch"
            );
            let data: Vec<f64> = out
                .as_slice()
                .iter()
                .zip(cot.as_slice().iter())
                .map(|(&o, &c)| o.mul_add(-o, 1.0) * c)
                .collect();
            Tensor::from_slice(&data, cot.shape())
        }
        UnaryRule::Relu { mask } => ew_mul(mask, cot),
        UnaryRule::Square { input } => {
            assert!(
                input.shape() == cot.shape(),
                "Tape::backward: square shape mismatch"
            );
            let data: Vec<f64> = input
                .as_slice()
                .iter()
                .zip(cot.as_slice().iter())
                .map(|(&i, &c)| 2.0 * i * c)
                .collect();
            Tensor::from_slice(&data, cot.shape())
        }
        UnaryRule::Ln { input } => {
            assert!(
                input.shape() == cot.shape(),
                "Tape::backward: ln shape mismatch"
            );
            let data: Vec<f64> = input
                .as_slice()
                .iter()
                .zip(cot.as_slice().iter())
                .map(|(&i, &c)| c / i)
                .collect();
            Tensor::from_slice(&data, cot.shape())
        }
        UnaryRule::Exp { out } => ew_mul(out, cot),
    }
}

fn binary_vjp(rule: &BinaryRule, cot: &Tensor<f64>) -> (Tensor<f64>, Tensor<f64>) {
    match rule {
        BinaryRule::Add => (cot.clone(), cot.clone()),
        BinaryRule::Sub => (cot.clone(), ew_neg(cot)),
        BinaryRule::Mul { lhs_val, rhs_val } => (ew_mul(rhs_val, cot), ew_mul(lhs_val, cot)),
    }
}

impl Tape {
    /// Compute gradients of `output` w.r.t. all parameters.
    ///
    /// Walks the tape in reverse (which is topological order by
    /// construction). After this call, use [`grad`](Self::grad) /
    /// [`grad_tensor`](Self::grad_tensor) to read gradients.
    ///
    /// The output node's cotangent is seeded with ones (shape-matching
    /// the output). Existing gradients at ancestor nodes accumulate — this
    /// matches the prior scalar-tape "second call accumulates" semantics.
    /// Create a fresh `Tape` for each independent forward pass.
    pub fn backward(&mut self, output: Var) {
        let out_idx = output.0 as usize;

        // Seed output cotangent with ones. Overwrites any prior value at
        // out_idx (matches the prior scalar-tape `grads[out] = 1.0` seed).
        let out_len = self.values[out_idx].len();
        let out_shape_vec: Vec<usize> = self.values[out_idx].shape().to_vec();
        let ones_data = vec![1.0_f64; out_len];
        self.grads[out_idx] = Tensor::from_slice(&ones_data, &out_shape_vec);

        for i in (0..=out_idx).rev() {
            // Skip nodes whose cotangent is all-zero — they are not
            // ancestors of output, or their contribution was exactly zero.
            // Local clone releases the shared borrow of self.grads so the
            // match arms can mutate grads[parent] freely.
            let cot_i = self.grads[i].clone();
            if cot_i.as_slice().iter().all(|&x| x == 0.0) {
                continue;
            }

            match &self.backward[i] {
                BackwardOp::Leaf => {}
                BackwardOp::Unary { parent, rule } => {
                    let contrib = unary_vjp(rule, &cot_i);
                    ew_add_assign(&mut self.grads[*parent as usize], &contrib);
                }
                BackwardOp::Binary { lhs, rhs, rule } => {
                    let (lhs_contrib, rhs_contrib) = binary_vjp(rule, &cot_i);
                    let lhs_idx = *lhs as usize;
                    let rhs_idx = *rhs as usize;
                    // Fan-in to the same parent (e.g. `x + x`): accumulate
                    // both contributions into the single parent slot.
                    ew_add_assign(&mut self.grads[lhs_idx], &lhs_contrib);
                    ew_add_assign(&mut self.grads[rhs_idx], &rhs_contrib);
                }
                BackwardOp::Custom { parents, op } => {
                    // Materialize parents into an owned vec so the final
                    // flush loop can run after NLL releases the
                    // self.backward[i] borrow following op.vjp.
                    let parents: Vec<Var> = parents.to_vec();
                    // Allocate zero-initialized parent cotangent slots
                    // shape-matched to the parent values. The op fills
                    // them via its VJP; we then flush into grads[parent].
                    let mut parent_cotans: Vec<Tensor<f64>> = parents
                        .iter()
                        .map(|v| Tensor::zeros(self.values[v.0 as usize].shape()))
                        .collect();
                    op.vjp(&cot_i, &mut parent_cotans);
                    for (k, v) in parents.iter().enumerate() {
                        ew_add_assign(&mut self.grads[v.0 as usize], &parent_cotans[k]);
                    }
                }
            }
        }
    }
}

impl Default for Tape {
    fn default() -> Self {
        Self::new()
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]
mod tests {
    use super::*;

    const FD_EPS: f64 = 1e-7;
    const FD_TOL: f64 = 1e-5;

    // ── Finite-difference helper ──────────────────────────────────

    /// Verify that the autograd gradient of a scalar function matches
    /// central finite differences.
    fn assert_grad_fd(x: f64, build_graph: impl Fn(&mut Tape, Var) -> Var) {
        // Analytical gradient via autograd.
        let mut tape = Tape::new();
        let p = tape.param(x);
        let out = build_graph(&mut tape, p);
        tape.backward(out);
        let analytical = tape.grad(p);

        // Numerical gradient via central finite differences.
        let f_plus = {
            let mut fd_tape = Tape::new();
            let fd_param = fd_tape.param(x + FD_EPS);
            let fd_out = build_graph(&mut fd_tape, fd_param);
            fd_tape.value(fd_out)
        };
        let f_minus = {
            let mut fd_tape = Tape::new();
            let fd_param = fd_tape.param(x - FD_EPS);
            let fd_out = build_graph(&mut fd_tape, fd_param);
            fd_tape.value(fd_out)
        };
        let numerical = (f_plus - f_minus) / (2.0 * FD_EPS);

        let err = (analytical - numerical).abs();
        assert!(
            err < FD_TOL,
            "FD mismatch: analytical={analytical}, numerical={numerical}, err={err}, x={x}",
        );
    }

    /// Multi-parameter FD check.
    fn assert_grads_fd(params: &[f64], build_graph: impl Fn(&mut Tape, &[Var]) -> Var) {
        let mut tape = Tape::new();
        let pvars: Vec<Var> = params.iter().map(|&v| tape.param(v)).collect();
        let out = build_graph(&mut tape, &pvars);
        tape.backward(out);
        let analytical: Vec<f64> = pvars.iter().map(|&v| tape.grad(v)).collect();

        for (i, &analytic_i) in analytical.iter().enumerate() {
            let eval_at = |delta: f64| {
                let mut fd_tape = Tape::new();
                let vs: Vec<Var> = params
                    .iter()
                    .enumerate()
                    .map(|(j, &val)| fd_tape.param(if j == i { val + delta } else { val }))
                    .collect();
                let fd_out = build_graph(&mut fd_tape, &vs);
                fd_tape.value(fd_out)
            };
            let numerical = (eval_at(FD_EPS) - eval_at(-FD_EPS)) / (2.0 * FD_EPS);
            let err = (analytic_i - numerical).abs();
            assert!(
                err < FD_TOL,
                "FD mismatch param[{i}]: analytical={analytic_i}, numerical={numerical}, err={err}",
            );
        }
    }

    // ── Per-op FD tests ───────────────────────────────────────────

    #[test]
    fn add_gradient_matches_fd() {
        assert_grads_fd(&[2.0, 3.0], |t, p| t.add(p[0], p[1]));
    }

    #[test]
    fn sub_gradient_matches_fd() {
        assert_grads_fd(&[2.0, 3.0], |t, p| t.sub(p[0], p[1]));
    }

    #[test]
    fn mul_gradient_matches_fd() {
        assert_grads_fd(&[2.0, 3.0], |t, p| t.mul(p[0], p[1]));
    }

    #[test]
    fn neg_gradient_matches_fd() {
        assert_grad_fd(2.5, Tape::neg);
    }

    #[test]
    fn tanh_gradient_matches_fd() {
        for &x in &[-2.0, -0.5, 0.0, 0.5, 2.0] {
            assert_grad_fd(x, Tape::tanh);
        }
    }

    #[test]
    fn relu_gradient_matches_fd() {
        for &x in &[-2.0, -0.1, 0.1, 1.5, 3.0] {
            assert_grad_fd(x, Tape::relu);
        }
    }

    #[test]
    fn square_gradient_matches_fd() {
        for &x in &[-3.0, 0.0, 1.5] {
            assert_grad_fd(x, Tape::square);
        }
    }

    #[test]
    fn ln_gradient_matches_fd() {
        for &x in &[0.1, 1.0, 5.0] {
            assert_grad_fd(x, Tape::ln);
        }
    }

    #[test]
    fn exp_gradient_matches_fd() {
        for &x in &[-1.0, 0.0, 2.0] {
            assert_grad_fd(x, Tape::exp);
        }
    }

    // ── Composition test ──────────────────────────────────────────

    #[test]
    fn chain_gradient_matches_fd() {
        assert_grads_fd(&[0.7, -1.2], |t, p| {
            let ab = t.mul(p[0], p[1]);
            let a2 = t.square(p[0]);
            let sum = t.add(ab, a2);
            t.tanh(sum)
        });
    }

    // ── Fused op tests ────────────────────────────────────────────

    #[test]
    fn sum_gradient_matches_fd() {
        assert_grads_fd(&[1.0, 2.0, 3.0], Tape::sum);
    }

    #[test]
    fn mean_gradient_matches_fd() {
        assert_grads_fd(&[1.0, 2.0, 3.0], Tape::mean);
    }

    #[test]
    fn affine_gradient_matches_fd() {
        let params = vec![
            0.3, -0.5, 0.8, -0.2, 0.4, -0.1, // W: 2×3 row-major
            0.1, -0.05, // b: 2
            1.0, -0.5, 2.0, // x: 3
        ];
        assert_grads_fd(&params, |t, p| {
            let w = &p[0..6];
            let b = &p[6..8];
            let x = &p[8..11];
            let z = t.affine(w, x, b, 2, 3);
            t.sum(&z)
        });
    }

    // ── Edge case tests ───────────────────────────────────────────

    #[test]
    fn constant_has_zero_grad() {
        let mut tape = Tape::new();
        let c = tape.constant(5.0);
        let p = tape.param(3.0);
        let out = tape.mul(c, p);
        tape.backward(out);

        assert!((tape.grad(p) - 5.0).abs() < 1e-12);
        assert!((tape.grad(c) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn fan_out_accumulates() {
        let mut tape = Tape::new();
        let x = tape.param(3.0);
        let out = tape.add(x, x);
        tape.backward(out);
        assert!((tape.grad(x) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn fan_out_mul_accumulates() {
        let mut tape = Tape::new();
        let x = tape.param(3.0);
        let out = tape.mul(x, x);
        tape.backward(out);
        assert!((tape.grad(x) - 6.0).abs() < 1e-12);
    }

    #[test]
    fn backward_only_reaches_ancestors() {
        let mut tape = Tape::new();
        let a = tape.param(1.0);
        let b = tape.param(2.0);
        let c = tape.param(3.0); // not connected to output
        let out = tape.add(a, b);
        tape.backward(out);

        assert!((tape.grad(a) - 1.0).abs() < 1e-12);
        assert!((tape.grad(b) - 1.0).abs() < 1e-12);
        assert!(tape.grad(c).abs() < 1e-12);
    }

    #[test]
    fn value_reads_forward_result() {
        let mut tape = Tape::new();
        let a = tape.param(3.0);
        let b = tape.param(4.0);
        let c = tape.add(a, b);
        assert!((tape.value(c) - 7.0).abs() < 1e-12);
    }

    // ── Tensor-API tests ──────────────────────────────────────────

    #[test]
    fn param_tensor_roundtrip() {
        let mut tape = Tape::new();
        let t = Tensor::from_slice(&[1.0, 2.0, 3.0], &[3]);
        let v = tape.param_tensor(t.clone());
        assert_eq!(tape.value_tensor(v), &t);
        assert_eq!(tape.grad_tensor(v), &Tensor::zeros(&[3]));
    }

    #[test]
    fn scalar_sugar_agrees_with_tensor_api() {
        let mut tape = Tape::new();
        let scalar_v = tape.param(2.5);
        // Scalar API reads scalar f64; tensor API reads shape-[] Tensor.
        assert_eq!(tape.value(scalar_v), 2.5);
        let t = tape.value_tensor(scalar_v);
        assert_eq!(t.shape(), &[] as &[usize]);
        assert_eq!(t.as_slice(), &[2.5]);
    }

    #[test]
    fn tensor_add_elementwise_gradient() {
        // f(a) = sum(a + a) = 2·sum(a) → df/da = [2, 2, 2]
        let mut tape = Tape::new();
        let a_val = Tensor::from_slice(&[1.0, 2.0, 3.0], &[3]);
        let a = tape.param_tensor(a_val);
        let y = tape.add(a, a);
        // Reduce tensor to scalar for backward: build constant ones and mul
        // elementwise, then sum via iterating (no tensor-reduce primitive).
        // Simpler: use mul with shape [3] vs [3] then sum — but sum needs
        // scalar inputs. Use a custom scalarize via tensor reduction op:
        // extract values and do summation by chaining scalar ops.
        // Skip reducer: just test via grad_tensor directly — backward seeds
        // y with ones, which corresponds to sum(y) as the output.
        tape.backward(y);
        assert_eq!(
            tape.grad_tensor(a),
            &Tensor::from_slice(&[2.0, 2.0, 2.0], &[3])
        );
    }

    #[test]
    fn tensor_mul_elementwise_gradient() {
        // f(a, b) = a ⊙ b (seed=ones). ∂f/∂a = b, ∂f/∂b = a.
        let mut tape = Tape::new();
        let a = tape.param_tensor(Tensor::from_slice(&[2.0, 3.0], &[2]));
        let b = tape.param_tensor(Tensor::from_slice(&[5.0, 7.0], &[2]));
        let y = tape.mul(a, b);
        tape.backward(y);
        assert_eq!(tape.grad_tensor(a), &Tensor::from_slice(&[5.0, 7.0], &[2]));
        assert_eq!(tape.grad_tensor(b), &Tensor::from_slice(&[2.0, 3.0], &[2]));
    }

    #[test]
    #[should_panic(expected = "shape mismatch")]
    fn tensor_add_panics_on_shape_mismatch() {
        let mut tape = Tape::new();
        let a = tape.param_tensor(Tensor::from_slice(&[1.0, 2.0], &[2]));
        let b = tape.param_tensor(Tensor::from_slice(&[1.0, 2.0, 3.0], &[3]));
        let _ = tape.add(a, b);
    }

    #[test]
    #[should_panic(expected = "requires shape []")]
    fn scalar_value_panics_on_non_scalar() {
        let mut tape = Tape::new();
        let v = tape.param_tensor(Tensor::from_slice(&[1.0, 2.0], &[2]));
        let _ = tape.value(v);
    }

    #[test]
    #[should_panic(expected = "requires shape []")]
    fn scalar_grad_panics_on_non_scalar() {
        let mut tape = Tape::new();
        let v = tape.param_tensor(Tensor::from_slice(&[1.0, 2.0], &[2]));
        let _ = tape.grad(v);
    }

    // ── Custom VjpOp test ─────────────────────────────────────────

    /// Test op: `y = k · x`. ∂y/∂x = k (elementwise scale). Parents are
    /// tracked at `push_custom` time, not by the op itself.
    #[derive(Debug)]
    struct ScaleOp {
        k: f64,
    }

    impl ScaleOp {
        fn new(k: f64) -> Self {
            Self { k }
        }
    }

    impl VjpOp for ScaleOp {
        fn op_id(&self) -> &'static str {
            "test::Scale"
        }

        fn vjp(&self, cot: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
            for (dst, &c) in parent_cotans[0]
                .as_mut_slice()
                .iter_mut()
                .zip(cot.as_slice().iter())
            {
                *dst += self.k * c;
            }
        }
    }

    #[test]
    fn custom_vjp_scale_by_k() {
        // y = 3 · x; x = [1, 2]; ∂(sum y)/∂x = [3, 3].
        let mut tape = Tape::new();
        let x = tape.param_tensor(Tensor::from_slice(&[1.0, 2.0], &[2]));
        // Forward: y_val = 3 · x
        let y_data: Vec<f64> = tape
            .value_tensor(x)
            .as_slice()
            .iter()
            .map(|&v| 3.0 * v)
            .collect();
        let y_val = Tensor::from_slice(&y_data, &[2]);
        let y = tape.push_custom(&[x], y_val, Box::new(ScaleOp::new(3.0)));
        tape.backward(y);
        assert_eq!(tape.grad_tensor(x), &Tensor::from_slice(&[3.0, 3.0], &[2]));
    }

    #[test]
    fn custom_vjp_op_id() {
        let op = ScaleOp::new(2.0);
        assert_eq!(op.op_id(), "test::Scale");
    }

    #[test]
    fn custom_vjp_composes_with_primitives() {
        // f(x) = tanh(2 · x). Scale by 2 is a custom VJP; tanh is a
        // primitive. ∂f/∂x = 2 · (1 - tanh²(2x)).
        let mut tape = Tape::new();
        let x = tape.param_tensor(Tensor::from_slice(&[0.5], &[1]));
        let scaled_data = vec![2.0 * 0.5];
        let scaled = tape.push_custom(
            &[x],
            Tensor::from_slice(&scaled_data, &[1]),
            Box::new(ScaleOp::new(2.0)),
        );
        let y = tape.tanh(scaled);
        tape.backward(y);
        let expected = 2.0 * (1.0 - (2.0 * 0.5_f64).tanh().powi(2));
        let got = tape.grad_tensor(x).as_slice()[0];
        assert!(
            (got - expected).abs() < 1e-12,
            "custom∘primitive grad: expected {expected}, got {got}",
        );
    }
}
