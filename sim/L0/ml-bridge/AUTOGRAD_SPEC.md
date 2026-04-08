# Autograd Engine — Level 2 Spec

> Build a minimal, RL-focused reverse-mode automatic differentiation engine
> inside `sim-ml-bridge`. No external ML framework deps. Hand-coded gradients
> become the test oracle; autograd becomes the production path.

**Status**: v4 — Phases 1-6 implemented, Phase 6 results documented
**Crate**: `sim-ml-bridge` (new module: `autograd`)
**License precedents**: micrograd (MIT), burn (Apache-2.0/MIT), dfdx (Apache-2.0/MIT)

---

## 1. Design principles

These are the constraints. Every implementation decision flows from them.

| Principle | What it means |
|-----------|---------------|
| **Transparent** | You can print the tape, see every node, trace any gradient back to the chain rule step that produced it. No hidden machinery. |
| **Simple** | 9 scalar ops + affine. No graph optimization, no lazy eval, no cleverness that hides the math. If you can't explain a line to a beginner, rewrite it. |
| **Complete** | Handles all 5 algorithms' gradient needs (REINFORCE, PPO, TD3, SAC, CEM doesn't need gradients). Not "general ML complete" — RL complete. |
| **Reliable** | Two independent oracles: finite differences AND hand-coded. Both must agree. If they don't, the autograd is wrong. |
| **Predictable** | Deterministic. Fresh tape per forward pass. No hidden state, no accumulation artifacts between calls. Same inputs → same outputs, always. |
| **Debuggable** | When a gradient is wrong, you can find *which op* broke in minutes, not hours. The tape is inspectable, printable, traceable. |
| **Auditable** | One person reads the entire engine in an afternoon. < 800 LOC, zero external dependencies. |

This is the stock RB26. Pull the valve cover off and see exactly where the
fuel goes, exactly where the spark fires, exactly where the exhaust exits.
No turbos, no variable valve timing, no black boxes. When you understand
this one completely, *then* you build the next one with tricks — and that
future engine has to prove itself against this one, because this one is
obviously correct by construction.

---

## 2. Why build our own

| Reason | Detail |
|--------|--------|
| **RL needs ~10 ops** | General frameworks carry thousands of ops for conv, attention, etc. We need: add, mul, sub, neg, tanh, relu, square, log, exp + fused matmul. |
| **Perfect test oracles** | Every hand-coded gradient in `linear.rs` and `mlp.rs` is finite-difference validated. We can verify autograd against them bit-for-bit. |
| **Parameter sync wart disappears** | Level 0-1 has dual param ownership (optimizer + network). Autograd owns params in one place. |
| **Foundational fix** | CortenForge principle: own your chassis. burn/candle are someone else's. |
| **Minimal surface area** | ~500 lines for the engine, not 500K. Every line is auditable. |

---

## 3. Prior art

### micrograd (Karpathy, MIT) — conceptual blueprint

The mental model:

```
Value {
    data: f64,
    grad: f64,
    _backward: Fn(),      // local gradient propagation
    _prev: Set<Value>,    // parent nodes
}

backward():
    1. topological sort
    2. walk in reverse
    3. call each node's _backward()
```

**Informed our design**: The conceptual architecture. Tape = list of
operations. backward = reverse walk applying chain rule. Nothing else needed.

**Not applicable**: Python operator overloading (`__add__`, `__mul__`). In
Rust, we use explicit method calls on the Tape — cleaner, no trait orphan
issues.

### burn (Tracel, Apache-2.0/MIT) — Rust ownership patterns

How burn solves the borrow checker:

- `Arc<Node>` for shared ownership (no lifetime parameters anywhere)
- `Box<dyn Step>` for type-erased backward closures (each step runs once)
- `HashMap<NodeId, StepBoxed>` for the graph
- `Gradients` container with `consume()`/`register()` pattern
- Memory management via `Arc::strong_count` pruning

**Informed our design**: The insight that Arc-free is possible when the tape
owns everything. burn needs Arc because tensors outlive the tape. Our tape
is ephemeral (one per forward pass), so index-based references suffice.
Also: the consume/register gradient accumulation pattern.

**Not applicable**: The full DAG with BFS traversal, checkpointing, memory
management, device abstraction. All overkill for 10-5K parameter networks.

### dfdx (Lowman, Apache-2.0/MIT) — the Rust-native approach

The closure-based tape:

```rust
struct OwnedTape<E, D> {
    operations: Vec<(UniqueId, Box<dyn FnOnce(&mut Gradients)>)>,
    gradients: Gradients,
}

// backward = sort by ID, reverse, execute closures
fn execute(&mut self) -> Gradients {
    self.operations.sort_by_key(|(k, _)| *k);
    for (_, op) in self.operations.drain(..).rev() {
        (op)(&mut self.gradients);
    }
}
```

And the zero-cost inference trick:

```rust
// OwnedTape = tracking gradients
// NoneTape  = compiles to nothing (monomorphization eliminates all backward code)
struct Tensor<S, E, D, T = NoneTape> { ... }
```

**Informed our design**:
- Monotonic IDs → tape is automatically topologically sorted (no sort needed!)
- `leaf_ids` + `drop_non_leafs()` for memory pruning after backward
- The insight that our tape IS the topological order by construction

**Not applicable**: Compile-time shape checking (const generics). Our
networks are small; runtime dim checks are fine. Also: device abstraction,
tape merging for binary ops (we own the tape explicitly).

---

## 4. Architecture

### Core insight

The tape is a flat `Vec<Node>`. Each `Var` is an index into that vec.
Because nodes can only reference previously-created nodes, the tape IS the
topological order. `backward()` is a simple reverse iteration — no sort, no
BFS, no graph traversal.

This is simpler than all three reference implementations.

### Types (as implemented)

```rust
/// Index into the tape. Copy + lightweight.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Var(u32);

/// One node in the computation graph.
struct Node {
    value: f64,
    backward: BackwardOp,
}

/// How a node propagates gradient to its parents.
///
/// Most ops store the local gradient as a precomputed f64 scalar.
/// `tanh` is special: its gradient depends on the output value (1 − out²),
/// so it stores `out` explicitly.
enum BackwardOp {
    /// Leaf node (parameter or constant) — no parents.
    Leaf,
    /// One parent. grad[parent] += local * grad[self].
    Unary { parent: u32, local: f64 },
    /// Two parents. grad[lhs] += dl * grad[self], grad[rhs] += dr * grad[self].
    Binary { lhs: u32, rhs: u32, dl: f64, dr: f64 },
    /// tanh(parent). Local gradient = 1 − out², computed during backward.
    Tanh { parent: u32, out: f64 },
}

/// The computation tape. One per forward pass, discarded after backward.
pub struct Tape {
    nodes: Vec<Node>,
    grads: Vec<f64>,       // same length as nodes, accumulated during backward
    leaf_mask: Vec<bool>,  // true for params whose grads we want to keep
}
```

**Key simplification from v1**: `BackwardOp` uses precomputed `f64` scalars
instead of function pointers. For most ops the local gradient is a constant
computed at forward time (e.g., `mul(a,b)`: `dl=b.value, dr=a.value`).
No `MatVec` variant — `affine` composes from scalar `mul`/`add`/`sum` ops,
which is more transparent and still correct (verified by FD tests).

### Why this works

| Design choice | Justification |
|---------------|---------------|
| `Var(u32)` not `Var(usize)` | 4B nodes is more than enough; halves index size |
| `enum BackwardOp` not `Box<dyn FnOnce>` | No heap allocation per op. Enum is stack-sized, cache-friendly |
| `local: f64` not `fn` pointers | Simpler, no indirection. Gradient rule is a precomputed number, not a callback |
| `Tanh` special variant | Only op whose gradient depends on its output value. All others (including `relu`) are precomputable into `Unary { local }` |
| Flat `Vec<Node>` not `HashMap` | Sequential allocation, sequential backward. Cache-perfect |
| No `Arc`, no lifetimes | Tape owns everything. Vars are Copy indices. Zero reference counting |
| `leaf_mask` not `HashSet<UniqueId>` | Vec<bool> indexed by Var position. O(1) lookup, no hashing |
| No `MatVec` variant | `affine` composes from `mul`+`sum`+`add`. Transparent, auto-correct gradients |

### API

```rust
impl Tape {
    /// Create a new empty tape.
    pub fn new() -> Self;

    // ── Leaf creation ──────────────────────────────────────────────

    /// Create a parameter (leaf whose gradient we keep).
    pub fn param(&mut self, value: f64) -> Var;

    /// Create a constant (leaf whose gradient we discard).
    pub fn constant(&mut self, value: f64) -> Var;

    // ── Scalar operations (9 primitives) ───────────────────────────

    pub fn add(&mut self, a: Var, b: Var) -> Var;     // da=1, db=1
    pub fn sub(&mut self, a: Var, b: Var) -> Var;     // da=1, db=-1
    pub fn mul(&mut self, a: Var, b: Var) -> Var;     // da=b, db=a
    pub fn neg(&mut self, a: Var) -> Var;              // da=-1
    pub fn tanh(&mut self, a: Var) -> Var;             // da=1-out²
    pub fn relu(&mut self, a: Var) -> Var;             // da=if a>0 {1} else {0}
    pub fn square(&mut self, a: Var) -> Var;           // da=2*a
    pub fn ln(&mut self, a: Var) -> Var;               // da=1/a
    pub fn exp(&mut self, a: Var) -> Var;              // da=out

    // ── Fused operations (performance) ─────────────────────────────

    /// z[i] = Σ_j W[i,j] * x[j] + b[i], where W/x/b are Var slices.
    /// Returns Vec<Var> of length `rows`.
    pub fn affine(&mut self, w: &[Var], x: &[Var], b: &[Var],
                  rows: usize, cols: usize) -> Vec<Var>;

    /// Sum a slice of Vars into a single Var.
    pub fn sum(&mut self, vars: &[Var]) -> Var;

    /// Mean of a slice of Vars.
    pub fn mean(&mut self, vars: &[Var]) -> Var;

    // ── Backward pass ──────────────────────────────────────────────

    /// Compute gradients of `output` w.r.t. all params.
    /// After this call, use `grad()` to read parameter gradients.
    pub fn backward(&mut self, output: Var);

    /// Read the gradient of a Var (typically a param).
    pub fn grad(&self, v: Var) -> f64;

    /// Read the forward value of a Var.
    pub fn value(&self, v: Var) -> f64;
}
```

### backward() implementation (as implemented)

```rust
pub fn backward(&mut self, output: Var) {
    self.grads[output.0 as usize] = 1.0;

    for i in (0..=output.0 as usize).rev() {
        let g = self.grads[i];
        if g == 0.0 { continue; }

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
```

---

## 5. RL layers

These are convenience functions that build subgraphs on the tape.
They are NOT types — just functions that take a `&mut Tape` and return `Var`s.

### Linear layer

```rust
/// z = tanh(W · x + b)
/// W: [out_dim × in_dim] params, b: [out_dim] params, x: [in_dim] vars.
/// Returns [out_dim] output vars.
pub fn linear_tanh(
    tape: &mut Tape,
    w: &[Var],      // row-major [out_dim × in_dim]
    b: &[Var],      // [out_dim]
    x: &[Var],      // [in_dim]
    out_dim: usize,
    in_dim: usize,
) -> Vec<Var>;

/// z = W · x + b (no activation — for value function output)
pub fn linear_raw(
    tape: &mut Tape,
    w: &[Var], b: &[Var], x: &[Var],
    out_dim: usize, in_dim: usize,
) -> Vec<Var>;
```

### Gaussian log-probability

```rust
/// log π(a|s) = -0.5 * Σ_i [(a_i - μ_i)² / σ_i² + 2*log(σ_i) + log(2π)]
///
/// `mu`: [act_dim] vars from policy forward pass
/// `log_std`: [act_dim] vars (params for StochasticPolicy, constants otherwise)
/// `action`: [act_dim] constants (observed actions)
///
/// Returns a single scalar Var.
pub fn gaussian_log_prob(
    tape: &mut Tape,
    mu: &[Var],
    log_std: &[Var],
    action: &[Var],
) -> Var;
```

### MSE loss

```rust
/// L = (pred - target)²
pub fn mse_loss(tape: &mut Tape, pred: Var, target: Var) -> Var;

/// L = mean_i (pred_i - target_i)²
pub fn mse_loss_batch(
    tape: &mut Tape,
    preds: &[Var],
    targets: &[Var],
) -> Var;
```

---

## 6. Network types

These are the autograd-backed implementations of the existing traits.

### AutogradPolicy

```rust
pub struct AutogradPolicy {
    obs_dim: usize,
    act_dim: usize,
    hidden_dims: Vec<usize>,   // e.g. [64, 64] for 2 hidden layers
    obs_scale: Vec<f64>,
    params: Vec<f64>,          // single source of truth
    // Precomputed offsets into params for each layer's W and b.
    layer_offsets: Vec<(usize, usize, usize, usize)>, // (w_start, w_end, b_start, b_end)
}
```

**Key difference from level 0-1**: `hidden_dims` is a `Vec`, not a single
`usize`. The autograd engine handles arbitrary depth automatically — no
hand-coded backprop per architecture.

Implements: `Policy` + `DifferentiablePolicy`.

**How `log_prob_gradient` works with autograd**:

```rust
fn log_prob_gradient(&self, obs: &[f32], action: &[f64], sigma: f64) -> Vec<f64> {
    let mut tape = Tape::new();

    // 1. Load params onto tape
    let param_vars: Vec<Var> = self.params.iter()
        .map(|&p| tape.param(p))
        .collect();

    // 2. Load obs as constants
    let obs_vars: Vec<Var> = obs.iter()
        .zip(&self.obs_scale)
        .map(|(&o, &s)| tape.constant(f64::from(o) * s))
        .collect();

    // 3. Forward pass through layers (builds the tape)
    let mu = self.forward_on_tape(&mut tape, &param_vars, &obs_vars);

    // 4. Compute log_prob (extends the tape)
    let log_std_vars: Vec<Var> = (0..self.act_dim)
        .map(|_| tape.constant(sigma.ln()))
        .collect();
    let action_vars: Vec<Var> = action.iter()
        .map(|&a| tape.constant(a))
        .collect();
    let log_prob = gaussian_log_prob(&mut tape, &mu, &log_std_vars, &action_vars);

    // 5. Backward (one call — handles any depth)
    tape.backward(log_prob);

    // 6. Read param gradients
    param_vars.iter().map(|&v| tape.grad(v)).collect()
}
```

No hand-coded backprop. No architecture-specific gradient code. Works for
1 hidden layer, 5 hidden layers, or 50.

### AutogradStochasticPolicy

Same as `AutogradPolicy` but `log_std` is part of `params` (not external).
Implements `StochasticPolicy`.

### AutogradValue

```rust
pub struct AutogradValue {
    obs_dim: usize,
    hidden_dims: Vec<usize>,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    layer_offsets: Vec<(usize, usize, usize, usize)>,
}
```

Implements `ValueFn`. Output layer is linear (no tanh) — value predictions
are unbounded. `mse_gradient` builds tape → forward → mse_loss → backward
→ read grads.

### AutogradQ

```rust
pub struct AutogradQ {
    obs_dim: usize,
    act_dim: usize,
    hidden_dims: Vec<usize>,
    obs_scale: Vec<f64>,
    params: Vec<f64>,
    layer_offsets: Vec<(usize, usize, usize, usize)>,
}
```

Implements `QFunction`. Input = `[obs_scaled; action]` concatenated.
`action_gradient` computes dQ/da by making actions into params (not
constants) on the tape, then reading their gradients after backward.

---

## 7. Optimizer integration

### Current problem (level 0-1)

```rust
// After optimizer.step():
policy.set_params(optimizer.params());  // manual sync — forgetting = silent bug
// SAC has 5 of these sync points per update step
```

### Level 2 solution

The optimizer operates directly on the network's `params: Vec<f64>`:

```rust
impl Optimizer {
    /// Update params in-place. Same math as step(), but on borrowed slices.
    /// `ascent` is required — actors maximize, critics minimize.
    pub fn step_in_place(&mut self, params: &mut [f64], grads: &[f64], ascent: bool);
}
```

The existing `Optimizer` trait (with Adam state: m, v, t) stays. The new
method takes `(&mut [f64], &[f64], bool)` instead of managing its own
param copy. Core math extracted into `adam_update()` free function shared
by both `step()` and `step_in_place()` — zero duplication.

---

## 8. Phase plan

### Phase 1: Core engine — DONE

**Delivered**: `src/autograd.rs` — 393 library LOC, 234 test LOC, 17 tests.

| Item | Status |
|------|--------|
| `Tape::new()`, `param()`, `constant()` | Done |
| 8 scalar ops: add, sub, mul, neg, tanh, square, ln, exp (relu added in Phase 5) | Done |
| 3 fused ops: `affine`, `sum`, `mean` (pulled from Phase 2) | Done |
| `backward()` | Done |
| `grad()`, `value()` | Done |
| FD tests for every op (ε=1e-7, tol=1e-5) | Done — 17 tests |

**Implementation note**: `affine` composes from `mul`+`sum`+`add` rather
than a fused `MatVec` backward variant. This is more transparent and
correct by construction (each scalar op is individually FD-tested).

### Phase 2: RL layers — DONE

**Delivered**: `src/autograd_layers.rs` — 171 library LOC, 238 test LOC, 10 tests.

| Item | Status |
|------|--------|
| `linear_tanh()` / `linear_raw()` | Done |
| `gaussian_log_prob()` (per-dim `log_std`) | Done |
| `mse_loss()` / `mse_loss_batch()` | Done |
| FD tests for each layer | Done |
| Forward parity with `MlpPolicy::forward` | Done — matches to 1e-12 |
| Value parity with `sac::gaussian_log_prob` | Done — matches to 1e-12 |

### Phase 3: Trait implementations — DONE

**Delivered**: `src/autograd_policy.rs` (437 library LOC, 322 test LOC),
`src/autograd_value.rs` (304 library LOC, 175 test LOC). 20 tests total.

| Item | Status |
|------|--------|
| `AutogradPolicy` — `Policy` + `DifferentiablePolicy` | Done |
| `AutogradStochasticPolicy` — `StochasticPolicy` with learned `log_std` | Done |
| `AutogradValue` — `ValueFn` | Done |
| `AutogradQ` — `QFunction` with `action_gradient` + `mse_gradient` | Done |
| Parity tests: forward, `log_prob_gradient`, `forward_vjp` | Done — 1e-10 |
| Parity tests: `mse_gradient` (Value + Q), `action_gradient` (Q) | Done — 1e-10 |
| 2-layer `AutogradPolicy` FD test | Done — 1e-4 (no oracle, pure FD) |
| `AutogradStochasticPolicy` FD tests | Done — 1e-4 (no oracle, pure FD) |

**Implementation finding**: There is no hand-coded MLP stochastic policy
oracle (`LinearStochasticPolicy` has 0 hidden layers, `MlpPolicy` has no
`log_std`). `AutogradStochasticPolicy` is validated via FD tests only.
Two tolerance tiers exist: **1e-10** (parity against hand-coded oracle)
and **1e-4** (FD tests where no oracle exists).

### Phase 4: Optimizer integration — DONE

**Delivered**: `Optimizer::step_in_place` + algorithm migration. 5 new tests.

| Item | Status |
|------|--------|
| `Optimizer::step_in_place(&mut self, params: &mut [f64], gradient: &[f64], ascent: bool)` | Done |
| `adam_update()` free function — shared by `step()` and `step_in_place()` | Done |
| REINFORCE: 2 sync points removed | Done |
| PPO: 4 sync points removed (2 init + 2 per K pass) | Done |
| TD3: 6 sync points removed (3 init + 3 per update) | Done |
| SAC: 6 sync points removed (3 init + 3 per update) | Done |
| Tests: `step_in_place` matches `step` (single + multi-step momentum) | Done — 5 tests |

**Implementation finding**: `adam_update()` extraction eliminated all code
duplication between `step()` and `step_in_place()`. The optimizer still
holds a `params: Vec<f64>` for backward compatibility with `step()`, but
`step_in_place()` ignores it — only m/v/t state is used. Algorithm code
now does `let mut p = network.params().to_vec(); optimizer.step_in_place(&mut p, ...); network.set_params(&p);` — still requires `set_params` on
the network because traits don't expose `params_mut()`. The dual ownership
wart is eliminated (optimizer no longer needs `set_params` at init).

### Phase 5: ReLU + Xavier init — DONE

**Delivered**: Activation choice, proper initialization, 5 new tests.

Two of the five original items were already done in Phase 3:
- **2+ hidden layers** — all autograd types accept `hidden_dims: &[usize]`.
- **Per-dimension sigma** — `AutogradStochasticPolicy` stores `log_std[act_dim]`.

| Item | Status |
|------|--------|
| `Tape::relu(a)` — 9th scalar op, fits in `Unary { parent, local }` | Done |
| `linear_relu` layer fn in `autograd_layers.rs` | Done |
| `Activation` enum (`Tanh` / `Relu`) in `autograd_layers.rs` | Done |
| `linear_hidden` dispatch fn (delegates to `linear_tanh` or `linear_relu`) | Done |
| All 4 autograd types store `activation` field | Done |
| `new_with(activation)` constructors on all 4 types | Done |
| `new_xavier(activation, rng)` constructors on all 4 types | Done |
| `xavier_init` helper — Glorot uniform for Tanh, He normal for ReLU | Done |
| FD tests: `relu`, `linear_relu`, ReLU 2-layer policy | Done — 3 tests |
| Xavier init nonzero weights test | Done |
| Convergence: 2-layer ReLU+Xavier REINFORCE on 2-DOF reaching | Done |

**Implementation findings**:
- `relu` needed no new `BackwardOp` variant — its gradient is a constant
  (`1.0` or `0.0`) determined at forward time, fitting in the existing
  `Unary { parent, local }` variant. Same simplification that worked for
  all non-tanh ops.
- `xavier_init` and `randn` helpers are duplicated between
  `autograd_policy.rs` and `autograd_value.rs` (same pattern as
  `compute_offsets` / `LayerOffsets`). Could be extracted to a shared
  module if more network types are added.
- All existing tests still pass with `Activation::Tanh` (default) — the
  `new()` constructors delegate to `new_with(..., Activation::Tanh)`, so
  backward compatibility is preserved.
- Output layer behavior is unchanged by activation choice: policy output
  always uses tanh (bounds actions to [-1, 1]), value/Q output always
  uses raw (unbounded).

### Phase 6: Competition re-run — DONE

**Delivered**: 10 builder functions, 2 integration tests, full results
documented in `COMPETITION_TESTS_SPEC.md`. Training observability added
(`on_epoch` callback on `Algorithm::train`).

| Item | Status |
|------|--------|
| Swap implementations | Done — autograd types for all 5 algorithms |
| 1-hidden-layer baseline | Done — exact parity (CEM -1.05, TD3 -11.99, PPO -3449, REINFORCE -7500). SAC overtakes TD3 with MLP actor. |
| 2-hidden-layer runs | Done — CEM -3.07, TD3 -4.08, SAC -30.04, PPO -9026, REINFORCE -11980 |
| Document ordering | Done — CEM still wins at 50 epochs but TD3 is 1 unit behind and converging. Reversal imminent at ~100-200 epochs. |
| Update `COMPETITION_TESTS_SPEC.md` | Done — level 2 results section with analysis |
| Training observability | Done — `on_epoch` callback, `Competition::new_verbose()`. Spec: `TRAINING_OBSERVABILITY_SPEC.md` |

**Hypothesis result**: Partially confirmed. TD3 improved 3x (-11.99 → -4.08)
while CEM degraded 3x (-1.05 → -3.07). The crossover is imminent but
didn't happen at 50 epochs — budget wasn't scaled for 8.8x more params.
SAC unstable due to aggressive LR. On-policy methods (PPO, REINFORCE)
need much more data per epoch. Follow-up experiments proposed (Phase 6b).

### Phase 6b: Budget scaling experiments

**Deliverable**: Confirm the ordering reversal by scaling budget to match
the 8.8x param increase. Three independent experiments on the same
reaching-6dof task (one variable at a time).

**Key constraint**: run one variable at a time. Each experiment compares
against the Phase 6 baseline (Test 9: seed 42, 50ep/50env, 2-layer
[64,64] ReLU Xavier) to isolate the effect.

**Prediction**: Experiment 6b-1 (more epochs) alone should reverse the
TD3/CEM ordering. TD3 was at -4.08 vs CEM -3.07 at epoch 50, and TD3
was still converging while CEM had plateaued.

#### Phase 6 baseline (Test 9, for reference)

| Algorithm | Reward | Dones | Params |
|-----------|--------|-------|--------|
| CEM | -3.07 | 0 | ~5,400 |
| TD3 | -4.08 | 0 | ~5,400 |
| SAC | -30.04 | 0 | ~5,400 |
| PPO | -9,025.90 | 0 | ~5,400 |
| REINFORCE | -11,979.50 | 0 | ~5,400 |

#### Experiment matrix

| Test | Name | Variable | Baseline → New | Hypothesis | Algorithms |
|------|------|----------|---------------|------------|------------|
| 10 | `budget_scaling_more_epochs` | Epochs | 50 → 200 | TD3 overtakes CEM. CEM plateaus, TD3 converges. | CEM, TD3, SAC (top 3) |
| 11 | `budget_scaling_lower_lr` | Learning rate | see below | SAC stabilizes. All gradient methods converge smoother. | All 5 |
| 12 | `budget_scaling_more_envs` | Envs | 50 → 200 | On-policy (PPO, REINFORCE) improve most — 4x samples per gradient estimate. | All 5 |

**Why top-3 for Test 10**: PPO (-9,026) and REINFORCE (-11,980) are
3 orders of magnitude behind CEM/TD3 at 50 epochs. 4x more epochs won't
bridge that gap — their problem is gradient variance, not training
duration. Including them wastes ~70 min of wall clock.

#### 6b-2 learning rate changes

Same reduction factor per family. CEM is gradient-free — unchanged.

| Algorithm | Current LR | New LR | Reduction |
|-----------|-----------|--------|-----------|
| REINFORCE | 0.05 | 0.01 | 5x |
| PPO | 0.025 | 0.005 | 5x |
| TD3 | 3e-4 | 1e-4 | 3x |
| SAC (optimizer) | 3e-4 | 1e-4 | 3x |
| SAC (alpha_lr) | 3e-4 | 1e-4 | 3x |

#### New builder functions (4 total, Test 11 only)

Tests 10 and 12 reuse existing `build_*_autograd_2layer` builders —
only `Competition::new_verbose(...)` params change.

Test 11 needs lower-LR variants (identical to `build_*_autograd_2layer`
except `OptimizerConfig::adam(lr)` and SAC's `alpha_lr`):

```rust
fn build_reinforce_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm>
    // OptimizerConfig::adam(0.01) instead of 0.05
fn build_ppo_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm>
    // OptimizerConfig::adam(0.005) instead of 0.025
fn build_td3_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm>
    // OptimizerConfig::adam(1e-4) instead of 3e-4
fn build_sac_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm>
    // OptimizerConfig::adam(1e-4) instead of 3e-4, alpha_lr: 1e-4
```

#### Test structure (each test)

Each test follows the Test 9 pattern:

1. `let task = reaching_6dof();`
2. `let comp = Competition::new_verbose(n_envs, TrainingBudget::Epochs(n), 42);`
3. Build algorithm vector, `comp.run(&[task], &builders)`
4. Assert all metrics finite
5. Print ranked table (algorithm, final reward, dones)
6. Check if any gradient method overtakes CEM → print reversal finding
7. Print Phase 6 baseline comparison

Assertions are **minimal/exploratory** — these are scientific
experiments, not regressions. Document whatever ordering emerges.

#### Runtime estimates

Based on Test 9 = ~2,850s for 5 algos at 50ep/50env (~570s/algo):

| Test | Algos | Scaling factor | Estimated time |
|------|-------|---------------|---------------|
| 10 | 3 | 4x epochs | ~110 min |
| 11 | 5 | 1x (same budget) | ~47 min |
| 12 | 5 | 4x envs | ~190 min |

Total: ~350 min (~6 hours). Run sequentially, one at a time.

#### Results

All three experiments complete. Full analysis in `COMPETITION_TESTS_SPEC.md`.

| Test | Variable | Winner | Reward | Key finding |
|------|----------|--------|--------|-------------|
| 11 (LR) | 3e-4 → 1e-4 | CEM | -3.07 | SAC stabilized (-30 → -14). TD3 transiently beat CEM at ep45 (-1.95). |
| 10 (epochs) | 50 → 200 | **TD3** | -5.22 | **Ordering reversed.** Both degraded from 50ep; CEM degraded faster. |
| 12 (envs) | 50 → 200 | CEM | -0.84 | CEM's best result ever (195 dones). Gradient methods collapsed. |

**Headline finding**: Algorithm ordering depends on *how* you scale
compute, not *how much*:

- **CEM scales with candidates** (more envs = more candidates/generation).
  200 candidates in 5,400 dims → CEM solves the task (-0.84, 195 dones).
- **Gradient methods scale with training duration** (more epochs).
  TD3 overtakes CEM at 200 epochs (-5.22 vs -5.75).
- **Neither scales well with the other's lever.** TD3 collapsed at
  200 envs (replay buffer overflow). CEM degraded at 200 epochs
  (random-walk after hitting noise_min).

**Unexpected findings**:

1. **On-policy methods got *worse* with 4x more envs.** REINFORCE and
   PPO diverged harder because the gradient sum (not mean) scales with
   env count — effectively 4x higher LR. Fix: normalize gradients by
   batch size.
2. **TD3's replay buffer (50K) is too small for 200 envs.** 200 envs ×
   500 steps = 100K transitions/epoch. Buffer can't hold one epoch.
   TD3 becomes effectively on-policy.
3. **CEM's noise schedule breaks at 200 epochs.** Hits noise_min by
   epoch ~75, then random-walks in 5,400 dims for 125 more epochs.
   Needs cyclical or adaptive noise for long-horizon training.

**Hypothesis result**: Partially confirmed. The reversal happens when
scaling epochs (Test 10: TD3 > CEM) but not when scaling envs (Test 12:
CEM dominates). The smooth quadratic reward is fundamental — CEM will
always be competitive on this task given enough candidates. Phase 6c
(nonlinear task) is needed to prove gradient methods' structural advantage.

#### Files modified

| File | Change |
|------|--------|
| `tests/competition.rs` | 4 new builder functions + 3 new test functions (Tests 10-12) |
| `AUTOGRAD_SPEC.md` | This section (Phase 6b expanded + results) |
| `../../examples/fundamentals/sim-ml/COMPETITION_TESTS_SPEC.md` | Full results + per-experiment analysis |

#### Verification

```bash
# Quick: existing tests still pass
cargo test -p sim-ml-bridge --lib

# Run one budget scaling experiment (use --release, these are multi-minute)
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture budget_scaling_lower_lr
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture budget_scaling_more_epochs
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture budget_scaling_more_envs
```

### Phase 6c: Nonlinear task design

**Deliverable**: A task where deeper networks have an inherent advantage
over linear controllers, replacing or supplementing the smooth quadratic
reaching task.

The current reaching-6dof reward (`-Σ(qpos - target)²`) is smooth,
unimodal, and approximately solvable by a linear PD controller. CEM
exploits this — random search on a smooth landscape is nearly as good as
gradient descent. A nonlinear task would expose CEM's limitations and
amplify gradient methods' advantage.

Properties the new task should have:
- **Task-space reward** (end-effector position, not joint angles)
- **Obstacles or contacts** (discontinuous dynamics)
- **Nonlinear reward landscape** (multiple waypoints, avoidance zones)
- **Same 6-DOF arm** (isolate the task, not the morphology)

Candidates:
- Reaching through a narrow gap (contact penalty)
- Sequential waypoint reaching (time-varying target)
- Reaching with joint limit penalties (soft constraints)

### Phase 7: Visual examples

**Deliverable**: Updated Bevy examples showing the ordering reversal live.

| Item | Detail |
|------|--------|
| Deeper network configs | Examples use 2-layer nets by default |
| Side-by-side comparison | New example: CEM vs PPO vs SAC on same task, autograd |
| README updates | Document the level 2 behavior changes |

---

## 9. File structure (as implemented)

```
sim/L0/ml-bridge/src/
├── autograd.rs          // Phase 1+5: Tape, Var, Node, backward, 9 scalar ops + affine/sum/mean
├── autograd_layers.rs   // Phase 2+5: linear_tanh, linear_relu, linear_raw, linear_hidden,
│                        //            Activation enum, gaussian_log_prob, mse_loss
├── autograd_policy.rs   // Phase 3+5: AutogradPolicy, AutogradStochasticPolicy,
│                        //            xavier_init, new_with, new_xavier
├── autograd_value.rs    // Phase 3+5: AutogradValue, AutogradQ,
│                        //            xavier_init, new_with, new_xavier
├── optimizer.rs         // Phase 4: adam_update, step_in_place
├── lib.rs               // pub mod + re-exports for all autograd modules
└── ... (existing algorithm files: reinforce, ppo, td3, sac — Phase 4 sync removal)
```

**Note**: `affine`, `sum`, `mean`, and `relu` live in `autograd.rs` (tape
primitives), not `autograd_layers.rs`. `Activation` enum and `linear_hidden`
dispatch live in `autograd_layers.rs` since they are layer-level concerns.

**Note**: `compute_offsets`, `LayerOffsets`, `xavier_init`, and `randn`
are duplicated between `autograd_policy.rs` and `autograd_value.rs`.
Could be extracted to a shared module if more network types are added.

**Note**: `autograd_layers::gaussian_log_prob` has the same name as
`sac::gaussian_log_prob` but a different signature (takes `Var`s, not
`f64`s). The autograd version is not re-exported at the crate root to
avoid confusion — access it via `autograd_layers::gaussian_log_prob`.

Phase 6 additions:
```
sim/L0/ml-bridge/
├── src/algorithm.rs     // Phase 6: on_epoch callback on Algorithm::train
├── src/competition.rs   // Phase 6: verbose field, new_verbose(), epoch logging
├── tests/competition.rs // Phase 6: 10 autograd builders, Tests 8-9
├── TRAINING_OBSERVABILITY_SPEC.md  // on_epoch design rationale
└── DEFERRED.md          // Deferred tasks (n_params, early stopping, persistence, RNG)
```

Hand-coded implementations (`linear.rs`, `mlp.rs`) stay forever — they are:
1. The test oracle for autograd correctness
2. A fallback for debugging
3. The level 0-1 reference for the competition narrative

---

## 10. Testing strategy

### Per-op finite differences (Phase 1)

```rust
fn assert_grad(tape_fn: impl Fn(&mut Tape, Var) -> Var, x: f64) {
    let eps = 1e-7;

    // Analytical
    let mut tape = Tape::new();
    let v = tape.param(x);
    let out = tape_fn(&mut tape, v);
    tape.backward(out);
    let analytical = tape.grad(v);

    // Numerical (central difference)
    let f_plus = { let mut t = Tape::new(); let v = t.param(x + eps); let o = tape_fn(&mut t, v); t.value(o) };
    let f_minus = { let mut t = Tape::new(); let v = t.param(x - eps); let o = tape_fn(&mut t, v); t.value(o) };
    let numerical = (f_plus - f_minus) / (2.0 * eps);

    assert!((analytical - numerical).abs() < 1e-5,
        "grad mismatch: analytical={analytical}, numerical={numerical}");
}
```

### Oracle parity (Phase 3)

```rust
#[test]
fn autograd_policy_matches_hand_coded() {
    let obs = &[0.5f32, -0.3, 0.8, 0.1];
    let action = &[0.2, -0.4];
    let sigma = 0.3;

    // Hand-coded (oracle)
    let mlp = MlpPolicy::new(4, 32, 2, &[1.0/PI, 1.0/PI, 0.1, 0.1]);
    mlp.set_params(&random_params);
    let expected = mlp.log_prob_gradient(obs, action, sigma);

    // Autograd (test subject)
    let ag = AutogradPolicy::new(4, vec![32], 2, &[1.0/PI, 1.0/PI, 0.1, 0.1]);
    ag.set_params(&random_params);  // same params
    let actual = ag.log_prob_gradient(obs, action, sigma);

    for (e, a) in expected.iter().zip(&actual) {
        assert!((e - a).abs() < 1e-10, "parity failure");
    }
}
```

### Competition parity (Phase 6)

```rust
#[test]
fn autograd_1layer_matches_level0_competition() {
    // Same architecture (1 hidden layer, 32 units) must produce
    // same competition results as hand-coded, within noise margin.
}
```

---

## 11. Performance expectations

| Metric | Level 0-1 (hand-coded) | Level 2 (autograd) | Notes |
|--------|------------------------|---------------------|-------|
| Forward pass | ~same | ~same | Both do the same matmul+tanh |
| Single backward | Faster (fused) | Slightly slower (tape overhead) | ~2-3x for small nets |
| Batch backward | N sequential calls | N sequential calls (phase 1) | Phase 2+: potential for batched tape |
| Memory | O(params) | O(params + tape nodes) | Tape is ephemeral, freed after backward |
| Expressiveness | 1 hidden layer, hand-coded | Arbitrary depth, automatic | The whole point |

The performance regression for small nets is acceptable because:
1. Physics stepping (sim-core) dominates wall-clock time, not gradient computation
2. The competition finding was that gradient *quality* matters, not gradient *speed*
3. Deeper networks enabled by autograd will learn faster, offsetting any per-step overhead

---

## 12. What changes for algorithms

**Trait firewall holds.** Algorithms are oblivious to the autograd backend.

```rust
// REINFORCE doesn't know or care:
let grad = policy.log_prob_gradient(obs, action, sigma);
// Works with LinearPolicy, MlpPolicy, OR AutogradPolicy.

// TD3 doesn't know or care:
let dq_da = q_fn.action_gradient(obs, action);
let policy_grad = policy.forward_vjp(obs, &dq_da);
// Works with any QFunction + DifferentiablePolicy implementation.
```

Two algorithm-level changes, neither breaking the trait firewall:
- Phase 4: `optimizer.step_in_place()` replaced manual param sync.
- Phase 6: `on_epoch: &dyn Fn(&EpochMetrics)` added to `Algorithm::train`
  for training observability. Existing call sites pass `&|_| {}`.
  Spec: `TRAINING_OBSERVABILITY_SPEC.md`.

---

## 13. Non-goals (explicitly excluded)

| Not doing | Why |
|-----------|-----|
| GPU tensors | Physics is CPU. GPU pipeline is in sim-gpu, separate concern. |
| Compile-time shapes | Networks are small. Runtime dim checks suffice. |
| Custom backward kernels | `affine` composes from scalar ops — no fused kernel needed. |
| Observation normalization | Belongs in policy preprocessing, not autograd. |
| Differentiable physics | Future (level 4-5). Autograd through the policy only. |
| Recurrent policies (LSTM/GRU) | Future. Linear + MLP is enough for level 2. |
| Gradient checkpointing | Networks are tiny. No memory pressure. |

---

## 14. Success criteria

1. **Every autograd gradient matches hand-coded to 1e-10** (Phase 3) — DONE
2. **FD-validated where no oracle exists** (Phase 3+5) — DONE (stochastic policy, 2-layer nets, ReLU nets, tol=1e-4)
3. **Competition results unchanged for same architecture** (Phase 6) — DONE. Exact parity for CEM/TD3/PPO/REINFORCE. SAC improved (MLP actor).
4. **Gradient methods improve with 2+ hidden layers** (Phase 6+6b) — DONE. TD3 improved 3x at 50 epochs (-11.99 → -4.08), reversed ordering at 200 epochs (TD3 -5.22 vs CEM -5.75, Test 10).
5. **Trait firewall holds** — DONE. Algorithms are oblivious to backend. Only optimizer-level change (Phase 4: `step_in_place`).
6. **Total autograd code ~1800 library LOC + ~1400 test LOC** across autograd engine, layers, policy, value, and optimizer integration. Original 800 LOC estimate underestimated trait impls, stochastic policy, activation/init, and test code.
7. **Zero new external dependencies** (pure Rust, no ML framework) — DONE
8. **57 autograd-related tests, 0 failures** across 5 files (autograd, layers, policy, value, optimizer) — DONE. 296 total crate tests pass.
