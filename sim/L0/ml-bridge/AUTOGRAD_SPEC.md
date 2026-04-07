# Autograd Engine — Level 2 Spec

> Build a minimal, RL-focused reverse-mode automatic differentiation engine
> inside `sim-ml-bridge`. No external ML framework deps. Hand-coded gradients
> become the test oracle; autograd becomes the production path.

**Status**: Draft v1
**Crate**: `sim-ml-bridge` (new module: `autograd`)
**License precedents**: micrograd (MIT), burn (Apache-2.0/MIT), dfdx (Apache-2.0/MIT)

---

## 1. Design principles

These are the constraints. Every implementation decision flows from them.

| Principle | What it means |
|-----------|---------------|
| **Transparent** | You can print the tape, see every node, trace any gradient back to the chain rule step that produced it. No hidden machinery. |
| **Simple** | 8 scalar ops + affine. No graph optimization, no lazy eval, no cleverness that hides the math. If you can't explain a line to a beginner, rewrite it. |
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
| **RL needs ~10 ops** | General frameworks carry thousands of ops for conv, attention, etc. We need: add, mul, sub, neg, tanh, square, log, exp + fused matmul. |
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

### Types

```rust
/// Index into the tape. Copy + lightweight.
#[derive(Clone, Copy)]
pub struct Var(u32);

/// One node in the computation graph.
struct Node {
    value: f64,
    /// How to propagate gradient to parents.
    /// Called with (this node's grad, &mut all grads).
    backward: BackwardFn,
}

/// Backward function — propagates gradient to parent nodes.
/// `grad` is this node's accumulated gradient.
/// Writes into parent slots in the gradient vec.
enum BackwardFn {
    /// No parents (leaf / constant).
    Leaf,
    /// One parent.
    Unary { parent: u32, f: fn(f64, f64, &Node) -> f64 },
    /// Two parents.
    Binary { lhs: u32, rhs: u32, f: fn(f64, f64, &Node, &Node) -> (f64, f64) },
    /// Matrix-vector multiply: multiple parents.
    MatVec { w_start: u32, x_start: u32, rows: u32, cols: u32 },
}

/// The computation tape. One per forward pass, discarded after backward.
pub struct Tape {
    nodes: Vec<Node>,
    grads: Vec<f64>,       // same length as nodes, accumulated during backward
    leaf_mask: Vec<bool>,  // true for params whose grads we want to keep
}
```

### Why this works

| Design choice | Justification |
|---------------|---------------|
| `Var(u32)` not `Var(usize)` | 4B nodes is more than enough; halves index size |
| `enum BackwardFn` not `Box<dyn FnOnce>` | No heap allocation per op. Enum is stack-sized, cache-friendly |
| Flat `Vec<Node>` not `HashMap` | Sequential allocation, sequential backward. Cache-perfect |
| No `Arc`, no lifetimes | Tape owns everything. Vars are Copy indices. Zero reference counting |
| `leaf_mask` not `HashSet<UniqueId>` | Vec<bool> indexed by Var position. O(1) lookup, no hashing |

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

    // ── Scalar operations (8 primitives) ───────────────────────────

    pub fn add(&mut self, a: Var, b: Var) -> Var;     // da=1, db=1
    pub fn sub(&mut self, a: Var, b: Var) -> Var;     // da=1, db=-1
    pub fn mul(&mut self, a: Var, b: Var) -> Var;     // da=b, db=a
    pub fn neg(&mut self, a: Var) -> Var;              // da=-1
    pub fn tanh(&mut self, a: Var) -> Var;             // da=1-out²
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

### backward() implementation

```rust
pub fn backward(&mut self, output: Var) {
    // Seed: d(output)/d(output) = 1
    self.grads[output.0 as usize] = 1.0;

    // Reverse walk — tape is already topologically sorted by construction
    for i in (0..=output.0 as usize).rev() {
        let g = self.grads[i];
        if g == 0.0 { continue; }  // skip unreached nodes

        match &self.nodes[i].backward {
            BackwardFn::Leaf => {},
            BackwardFn::Unary { parent, f } => {
                self.grads[*parent as usize] += f(g, self.nodes[i].value, &self.nodes[*parent as usize]);
            },
            BackwardFn::Binary { lhs, rhs, f } => {
                let (dl, dr) = f(g, self.nodes[i].value, &self.nodes[*lhs as usize], &self.nodes[*rhs as usize]);
                self.grads[*lhs as usize] += dl;
                self.grads[*rhs as usize] += dr;
            },
            BackwardFn::MatVec { .. } => { /* fused kernel */ },
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
    /// Update params in-place. Reads gradients from the most recent backward pass.
    pub fn step_autograd(&mut self, params: &mut [f64], grads: &[f64]);
}
```

Or: the network types expose `&mut [f64]` to the optimizer, and the
optimizer writes directly. No copy, no sync, no silent bugs.

The existing `Optimizer` struct (with Adam state) stays. We add a method
that takes `(&mut [f64], &[f64])` instead of managing its own param copy.

---

## 8. Phase plan

### Phase 1: Core engine (~200 LOC)

**Deliverable**: `src/autograd.rs` — Tape, Var, Node, backward.

| Item | Detail |
|------|--------|
| `Tape::new()`, `param()`, `constant()` | Leaf creation |
| 8 scalar ops | add, sub, mul, neg, tanh, square, ln, exp |
| `backward()` | Reverse walk with gradient accumulation |
| `grad()`, `value()` | Read results |
| Tests | Finite-difference test for every op (central differences, ε=1e-7, tol=1e-5) |

**Validation**: Every op tested against `(f(x+ε) - f(x-ε)) / 2ε`.

### Phase 2: Fused ops + RL layers (~150 LOC)

**Deliverable**: `affine`, `sum`, `mean` fused ops + `linear_tanh`,
`linear_raw`, `gaussian_log_prob`, `mse_loss` layer functions.

| Item | Detail |
|------|--------|
| `affine()` | Fused W·x+b with correct gradient propagation |
| `linear_tanh()` / `linear_raw()` | Layer builders |
| `gaussian_log_prob()` | Full log-prob with per-dim sigma |
| `mse_loss()` / `mse_loss_batch()` | Loss functions |
| Tests | FD tests for each layer. Forward parity with hand-coded `MlpPolicy::forward`. |

### Phase 3: Trait implementations (~300 LOC)

**Deliverable**: `AutogradPolicy`, `AutogradStochasticPolicy`,
`AutogradValue`, `AutogradQ` — all implementing the existing traits.

| Item | Detail |
|------|--------|
| `AutogradPolicy` | `Policy` + `DifferentiablePolicy` for arbitrary depth |
| `AutogradStochasticPolicy` | `StochasticPolicy` with learned `log_std` |
| `AutogradValue` | `ValueFn` with autograd MSE gradient |
| `AutogradQ` | `QFunction` with autograd action_gradient + MSE gradient |
| **Parity tests** | For 1-hidden-layer configs, compare every gradient method against hand-coded `MlpPolicy`/`MlpValue`/`MlpQ` outputs. Must match to 1e-10. |

This is the critical phase. The hand-coded implementations are the oracle.

### Phase 4: Optimizer integration (~50 LOC)

**Deliverable**: `Optimizer::step_in_place(&mut self, params: &mut [f64], grads: &[f64])`.

| Item | Detail |
|------|--------|
| New optimizer method | Works on borrowed slices, not owned params |
| Algorithm updates | Remove all `set_params(optimizer.params())` sync calls |
| Tests | Same Adam behavior, no sync bugs |

### Phase 5: Deeper networks + new capabilities (~100 LOC)

**Deliverable**: Capabilities that were impossible at level 0-1.

| Item | Detail |
|------|--------|
| 2+ hidden layers | `hidden_dims: vec![64, 64]` just works — no new backprop code |
| ReLU activation | `tape.relu(a)` — one new op, backward = `if a > 0 { grad } else { 0 }` |
| Per-dimension sigma | `log_std` as `[act_dim]` params, not scalar — anisotropic exploration |
| Xavier initialization | `AutogradPolicy::new_xavier()` — proper init for deep nets |
| Tests | FD tests for new ops. Training convergence on 2-DOF reaching with 2-layer net. |

### Phase 6: Competition re-run

**Deliverable**: Updated competition test results with autograd backends.

| Item | Detail |
|------|--------|
| Swap implementations | Competition tests use `AutogradPolicy`/`AutogradValue`/`AutogradQ` |
| 1-hidden-layer baseline | Must match level 0-1 results (same architecture = same performance) |
| 2-hidden-layer runs | The real test — gradient methods should improve dramatically |
| Document ordering | Does it reverse? Where? By how much? |
| Update `COMPETITION_TESTS_SPEC.md` | Level 2 results section |

**Hypothesis**: With autograd + 2 hidden layers:
- PPO and SAC overtake CEM on 2-DOF
- CEM still competitive on low-param configs
- TD3 becomes viable (batch backward removes the per-sample noise)
- REINFORCE improves but still worst gradient method (no baseline/clipping)

### Phase 7: Visual examples

**Deliverable**: Updated Bevy examples showing the ordering reversal live.

| Item | Detail |
|------|--------|
| Deeper network configs | Examples use 2-layer nets by default |
| Side-by-side comparison | New example: CEM vs PPO vs SAC on same task, autograd |
| README updates | Document the level 2 behavior changes |

---

## 9. File structure

```
sim/L0/ml-bridge/src/
├── autograd.rs          // Phase 1: Tape, Var, Node, backward, scalar ops
├── autograd_layers.rs   // Phase 2: affine, linear_tanh, gaussian_log_prob, mse_loss
├── autograd_policy.rs   // Phase 3: AutogradPolicy, AutogradStochasticPolicy
├── autograd_value.rs    // Phase 3: AutogradValue, AutogradQ
├── lib.rs               // Add: pub mod autograd, autograd_layers, etc.
└── ... (existing files unchanged)
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

**Nothing.** That's the whole point of the trait firewall.

```rust
// REINFORCE doesn't know or care:
let grad = policy.log_prob_gradient(obs, action, sigma);
// Works with LinearPolicy, MlpPolicy, OR AutogradPolicy.

// TD3 doesn't know or care:
let dq_da = q_fn.action_gradient(obs, action);
let policy_grad = policy.forward_vjp(obs, &dq_da);
// Works with any QFunction + DifferentiablePolicy implementation.
```

The only algorithm-level change is in Phase 4: removing the manual
`set_params(optimizer.params())` sync calls, because autograd policies
own their params directly.

---

## 13. Non-goals (explicitly excluded)

| Not doing | Why |
|-----------|-----|
| GPU tensors | Physics is CPU. GPU pipeline is in sim-gpu, separate concern. |
| Compile-time shapes | Networks are small. Runtime dim checks suffice. |
| Custom backward kernels | Fused `affine` is the only optimization needed. |
| Observation normalization | Belongs in policy preprocessing, not autograd. |
| Differentiable physics | Future (level 4-5). Autograd through the policy only. |
| Recurrent policies (LSTM/GRU) | Future. Linear + MLP is enough for level 2. |
| Gradient checkpointing | Networks are tiny. No memory pressure. |

---

## 14. Success criteria

1. **Every autograd gradient matches hand-coded to 1e-10** (Phase 3)
2. **Competition results unchanged for same architecture** (Phase 6)
3. **Gradient methods improve with 2+ hidden layers** (Phase 6)
4. **Zero algorithm code changes** (trait firewall holds)
5. **Total autograd code < 800 LOC** (engine + layers + trait impls)
6. **Zero new external dependencies** (pure Rust, no ML framework)
