//! Tape-based reverse-mode automatic differentiation.
//!
//! A minimal, RL-focused autograd engine. Builds a computation graph during
//! the forward pass, then walks it in reverse to compute gradients via the
//! chain rule. Designed for transparency — every operation's gradient rule
//! is visible, every node is inspectable.
//!
//! # Architecture
//!
//! ```text
//! Tape   — flat Vec<Node>, owns all values and gradients
//! Var    — lightweight index into the tape (Copy, 4 bytes)
//! Node   — one operation: its forward value + backward rule
//! ```
//!
//! The tape is topologically sorted by construction: each operation can only
//! reference previously created nodes. `backward()` is a simple reverse
//! iteration — no sort, no BFS, no graph traversal.
//!
//! # Supported operations
//!
//! Nine scalar primitives: [`Tape::add`], [`Tape::sub`], [`Tape::mul`],
//! [`Tape::neg`], [`Tape::tanh`], [`Tape::relu`], [`Tape::square`],
//! [`Tape::ln`], [`Tape::exp`].
//!
//! Three fused operations: [`Tape::affine`] (matrix-vector multiply + bias),
//! [`Tape::sum`], [`Tape::mean`].
//!
//! # What this module does NOT do
//!
//! - No GPU tensors — CPU `f64` only.
//! - No graph optimizations — transparency over performance.
//! - No batched tape — loop per sample (physics dominates wall-clock time).
//! - No operator overloading — explicit method calls on `&mut Tape`.

// ── Types ─────────────────────────────────────────────────────────────────

/// Index into a [`Tape`]. Lightweight and `Copy`.
///
/// Created by [`Tape::param`] or [`Tape::constant`] for leaf values, or by
/// any operation method (e.g. [`Tape::add`]) for intermediate values.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Var(u32);

/// One node in the computation graph.
struct Node {
    value: f64,
    backward: BackwardOp,
}

/// How a node propagates gradient to its parents.
///
/// For most operations the local gradient is a constant computed at forward
/// time. `tanh` is special: its local gradient depends on the output value
/// (`1 − out²`), so it stores `out` explicitly.
enum BackwardOp {
    /// Leaf node (parameter or constant) — no parents.
    Leaf,
    /// One parent. `grad[parent] += local * grad[self]`.
    Unary { parent: u32, local: f64 },
    /// Two parents. `grad[lhs] += dl * grad[self]`, `grad[rhs] += dr * grad[self]`.
    Binary {
        lhs: u32,
        rhs: u32,
        dl: f64,
        dr: f64,
    },
    /// `tanh(parent)`. Local gradient = `1 − out²`, computed during backward.
    Tanh { parent: u32, out: f64 },
}

/// The computation tape. One per forward pass, discarded after backward.
///
/// # Usage
///
/// ```
/// use sim_ml_bridge::autograd::{Tape, Var};
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
    nodes: Vec<Node>,
    grads: Vec<f64>,
    leaf_mask: Vec<bool>,
}

// ── Tape construction ─────────────────────────────────────────────────────

impl Tape {
    /// Create a new empty tape.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            nodes: Vec::new(),
            grads: Vec::new(),
            leaf_mask: Vec::new(),
        }
    }

    /// Append a node and return its `Var` index.
    fn push(&mut self, value: f64, backward: BackwardOp, is_leaf: bool) -> Var {
        let idx = self.nodes.len();
        assert!(
            idx < u32::MAX as usize,
            "Tape overflow: more than {} nodes",
            u32::MAX,
        );
        self.nodes.push(Node { value, backward });
        self.grads.push(0.0);
        self.leaf_mask.push(is_leaf);
        #[allow(clippy::cast_possible_truncation)] // guarded by assert above
        Var(idx as u32)
    }

    /// Create a parameter — a leaf whose gradient is kept after backward.
    #[must_use]
    pub fn param(&mut self, value: f64) -> Var {
        self.push(value, BackwardOp::Leaf, true)
    }

    /// Create a constant — a leaf whose gradient is discarded.
    #[must_use]
    pub fn constant(&mut self, value: f64) -> Var {
        self.push(value, BackwardOp::Leaf, false)
    }

    /// Read the forward value of a node.
    #[must_use]
    pub fn value(&self, v: Var) -> f64 {
        self.nodes[v.0 as usize].value
    }

    /// Read the gradient of a node (populated after [`backward`](Self::backward)).
    #[must_use]
    pub fn grad(&self, v: Var) -> f64 {
        self.grads[v.0 as usize]
    }
}

// ── Scalar operations (8 primitives) ──────────────────────────────────────

impl Tape {
    /// `a + b`. Gradients: `da = 1`, `db = 1`.
    #[must_use]
    pub fn add(&mut self, a: Var, b: Var) -> Var {
        let val = self.value(a) + self.value(b);
        self.push(
            val,
            BackwardOp::Binary {
                lhs: a.0,
                rhs: b.0,
                dl: 1.0,
                dr: 1.0,
            },
            false,
        )
    }

    /// `a − b`. Gradients: `da = 1`, `db = −1`.
    #[must_use]
    pub fn sub(&mut self, a: Var, b: Var) -> Var {
        let val = self.value(a) - self.value(b);
        self.push(
            val,
            BackwardOp::Binary {
                lhs: a.0,
                rhs: b.0,
                dl: 1.0,
                dr: -1.0,
            },
            false,
        )
    }

    /// `a × b`. Gradients: `da = b`, `db = a`.
    #[must_use]
    pub fn mul(&mut self, a: Var, b: Var) -> Var {
        let va = self.value(a);
        let vb = self.value(b);
        self.push(
            va * vb,
            BackwardOp::Binary {
                lhs: a.0,
                rhs: b.0,
                dl: vb,
                dr: va,
            },
            false,
        )
    }

    /// `−a`. Gradient: `da = −1`.
    #[must_use]
    pub fn neg(&mut self, a: Var) -> Var {
        let val = -self.value(a);
        self.push(
            val,
            BackwardOp::Unary {
                parent: a.0,
                local: -1.0,
            },
            false,
        )
    }

    /// `tanh(a)`. Gradient: `da = 1 − tanh²(a)`.
    #[must_use]
    pub fn tanh(&mut self, a: Var) -> Var {
        let out = self.value(a).tanh();
        self.push(out, BackwardOp::Tanh { parent: a.0, out }, false)
    }

    /// `a²`. Gradient: `da = 2a`.
    #[must_use]
    pub fn square(&mut self, a: Var) -> Var {
        let va = self.value(a);
        self.push(
            va * va,
            BackwardOp::Unary {
                parent: a.0,
                local: 2.0 * va,
            },
            false,
        )
    }

    /// `ln(a)`. Gradient: `da = 1/a`.
    ///
    /// # Panics
    ///
    /// Panics if `a ≤ 0` (logarithm undefined).
    #[must_use]
    pub fn ln(&mut self, a: Var) -> Var {
        let va = self.value(a);
        assert!(va > 0.0, "Tape::ln: input must be positive, got {va}");
        self.push(
            va.ln(),
            BackwardOp::Unary {
                parent: a.0,
                local: 1.0 / va,
            },
            false,
        )
    }

    /// `exp(a)`. Gradient: `da = exp(a)`.
    #[must_use]
    pub fn exp(&mut self, a: Var) -> Var {
        let out = self.value(a).exp();
        self.push(
            out,
            BackwardOp::Unary {
                parent: a.0,
                local: out,
            },
            false,
        )
    }

    /// `max(0, a)`. Gradient: `da = if a > 0 { 1 } else { 0 }`.
    #[must_use]
    pub fn relu(&mut self, a: Var) -> Var {
        let va = self.value(a);
        let (out, local) = if va > 0.0 { (va, 1.0) } else { (0.0, 0.0) };
        self.push(out, BackwardOp::Unary { parent: a.0, local }, false)
    }
}

// ── Fused operations ──────────────────────────────────────────────────────

impl Tape {
    /// Sum a slice of variables. Gradient: each input gets `1.0`.
    ///
    /// # Panics
    ///
    /// Panics if `vars` is empty.
    #[must_use]
    pub fn sum(&mut self, vars: &[Var]) -> Var {
        assert!(!vars.is_empty(), "Tape::sum: empty input");
        let mut acc = vars[0];
        for &v in &vars[1..] {
            acc = self.add(acc, v);
        }
        acc
    }

    /// Mean of a slice of variables. Gradient: each input gets `1/n`.
    ///
    /// # Panics
    ///
    /// Panics if `vars` is empty.
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
    /// `w` is row-major `[rows × cols]`, `x` is `[cols]`, `b` is `[rows]`.
    /// Returns `[rows]` output variables.
    ///
    /// Built from [`mul`](Self::mul), [`sum`](Self::sum), and [`add`](Self::add)
    /// — gradients are automatically correct via composition.
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

impl Tape {
    /// Compute gradients of `output` w.r.t. all parameters.
    ///
    /// Walks the tape in reverse (which is topological order by construction).
    /// After this call, use [`grad`](Self::grad) to read parameter gradients.
    ///
    /// Calling `backward` a second time accumulates into existing gradients.
    /// Create a fresh `Tape` for each independent forward pass.
    pub fn backward(&mut self, output: Var) {
        let out_idx = output.0 as usize;
        self.grads[out_idx] = 1.0;

        for i in (0..=out_idx).rev() {
            let g = self.grads[i];
            // Skip nodes with zero gradient — they are not ancestors of output
            // or their gradient contribution was zero.
            if g == 0.0 {
                continue;
            }

            match self.nodes[i].backward {
                BackwardOp::Leaf => {}
                BackwardOp::Unary { parent, local } => {
                    self.grads[parent as usize] += local * g;
                }
                BackwardOp::Binary { lhs, rhs, dl, dr } => {
                    self.grads[lhs as usize] += dl * g;
                    self.grads[rhs as usize] += dr * g;
                }
                BackwardOp::Tanh { parent, out } => {
                    self.grads[parent as usize] += out.mul_add(-out, 1.0) * g;
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
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]
mod tests {
    use super::*;

    const FD_EPS: f64 = 1e-7;
    const FD_TOL: f64 = 1e-5;

    // ── Finite-difference helper ──────────────────────────────────

    /// Verify that the autograd gradient of a scalar function matches
    /// central finite differences.
    ///
    /// `build_graph` takes a tape and a param `Var`, builds the forward
    /// graph, and returns the output `Var`.
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

    /// Multi-parameter FD check. `build_graph` takes a tape and a slice
    /// of param `Var`s, returns the output `Var`.
    fn assert_grads_fd(params: &[f64], build_graph: impl Fn(&mut Tape, &[Var]) -> Var) {
        // Analytical gradients.
        let mut tape = Tape::new();
        let pvars: Vec<Var> = params.iter().map(|&v| tape.param(v)).collect();
        let out = build_graph(&mut tape, &pvars);
        tape.backward(out);
        let analytical: Vec<f64> = pvars.iter().map(|&v| tape.grad(v)).collect();

        // Numerical gradients.
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
        // Test at several points including near saturation.
        for &x in &[-2.0, -0.5, 0.0, 0.5, 2.0] {
            assert_grad_fd(x, Tape::tanh);
        }
    }

    #[test]
    fn relu_gradient_matches_fd() {
        // Positive, negative, and near-zero.
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
        // f(a, b) = tanh(a * b + a²)
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
        // 2×3 affine: z[2] = W[2×3] · x[3] + b[2]
        // Params: W (6 values) + b (2 values) + x (3 values) = 11
        // We test gradients w.r.t. all 11.
        let params = vec![
            // W: 2×3 row-major
            0.3, -0.5, 0.8, -0.2, 0.4, -0.1, // b: 2
            0.1, -0.05, // x: 3
            1.0, -0.5, 2.0,
        ];
        assert_grads_fd(&params, |t, p| {
            let w = &p[0..6];
            let b = &p[6..8];
            let x = &p[8..11];
            let z = t.affine(w, x, b, 2, 3);
            // Reduce to scalar: sum of outputs.
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

        // dout/dp = c = 5
        assert!((tape.grad(p) - 5.0).abs() < 1e-12);
        // constant still accumulates gradient internally (it's just not
        // semantically a "parameter"). The leaf_mask distinguishes them
        // for future pruning, but grad() returns the raw value.
        assert!((tape.grad(c) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn fan_out_accumulates() {
        // f(x) = x + x = 2x → df/dx = 2
        let mut tape = Tape::new();
        let x = tape.param(3.0);
        let out = tape.add(x, x);
        tape.backward(out);
        assert!((tape.grad(x) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn fan_out_mul_accumulates() {
        // f(x) = x * x = x² → df/dx = 2x
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
        assert!(tape.grad(c).abs() < 1e-12); // c is not an ancestor
    }

    #[test]
    fn value_reads_forward_result() {
        let mut tape = Tape::new();
        let a = tape.param(3.0);
        let b = tape.param(4.0);
        let c = tape.add(a, b);
        assert!((tape.value(c) - 7.0).abs() < 1e-12);
    }
}
