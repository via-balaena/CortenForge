# Training Observability Spec

> Make `Algorithm::train` transparent. Every epoch should be visible as it
> happens, not hidden behind a multi-minute black box.

**Status**: Draft
**Crate**: `sim-ml-bridge`
**Motivation**: Competition tests (Phase 6) run 5 algorithms × 50 epochs
each. With no logging, you stare at a blank terminal for 10+ minutes and
have no idea if training is converging, diverging, or stuck.

---

## 1. Design principle

ML training is uniquely opaque. The feedback loop is minutes, not
milliseconds. Observability compensates for that — you should be able to
watch the reward curve form in real time and kill a run early if it's
clearly going nowhere.

The autograd spec says "Transparent" is principle #1. That extends to
training, not just the tape.

---

## 2. The change

Add a per-epoch callback to `Algorithm::train`.

### Before

```rust
pub trait Algorithm: Send {
    fn name(&self) -> &'static str;
    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
    ) -> Vec<EpochMetrics>;
}
```

### After

```rust
pub trait Algorithm: Send {
    fn name(&self) -> &'static str;
    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics>;
}
```

One new parameter. Every algorithm calls `on_epoch(&metrics)` at the end
of each epoch, after collecting metrics but before the next epoch starts.
The caller decides what to do — log it, ignore it, aggregate it.

---

## 3. Why a callback, not alternatives

| Alternative | Why not |
|-------------|---------|
| `eprintln!` inside each algorithm | Duplicated 5×, can't silence, caller has no control |
| Iterator-based train (`-> impl Iterator<Item = EpochMetrics>`) | Requires restructuring all algorithm internals to yield mid-loop |
| `TrainingLogger` trait object | Over-engineered for one hook; callback is the minimal useful abstraction |
| Verbose flag on Algorithm | Bakes logging into the algorithm instead of giving control to the caller |

The callback is zero-cost when unused (`&|_| {}`), gives the caller full
control, and requires no new types or traits.

---

## 4. Competition runner changes

`Competition` gains a `verbose` field. When true, it wraps the callback
with algorithm-level framing (start/end messages, total wall time).

### Before

```rust
pub struct Competition {
    n_envs: usize,
    budget: TrainingBudget,
    seed: u64,
}
```

### After

```rust
pub struct Competition {
    n_envs: usize,
    budget: TrainingBudget,
    seed: u64,
    verbose: bool,
}
```

New constructor:

```rust
impl Competition {
    /// Create a competition runner with epoch-level logging.
    pub const fn new_verbose(n_envs: usize, budget: TrainingBudget, seed: u64) -> Self {
        Self { n_envs, budget, seed, verbose: true }
    }
}
```

`new()` stays unchanged (`verbose: false`), preserving existing behavior
for unit tests and non-competition callers.

### Verbose output format

```
[CEM] training on reaching-6dof (5382 params)...
  epoch  0: reward=-142.30, dones=0, 84ms
  epoch  1: reward= -98.71, dones=0, 79ms
  ...
  epoch 49: reward=  -1.05, dones=49, 81ms
[CEM] done — reward=-1.05, 49 dones, 3841ms total

[REINFORCE] training on reaching-6dof (5382 params)...
  epoch  0: reward=-9142.50, dones=0, 312ms
  ...
```

Algorithm name comes from `algorithm.name()` (`&'static str`, Copy —
captured before the mutable `train` borrow). Param count comes from
the policy's `n_params()` — not available through the `Algorithm` trait,
so we report it only when available (Competition knows the builder
output).

**Implementation note**: `algorithm.name()` returns `&'static str` which
is `Copy`. We capture it before calling `train(&mut self, ...)` to avoid
a borrow conflict between the closure and the mutable self reference:

```rust
let name = algorithm.name();
let metrics = algorithm.train(&mut env, self.budget, self.seed, &|m| {
    eprintln!(
        "  epoch {:>2}: reward={:>10.2}, dones={}, {}ms",
        m.epoch, m.mean_reward, m.done_count, m.wall_time_ms
    );
});
```

---

## 5. Affected files

### Trait + runner (logic changes)

| File | Change |
|------|--------|
| `src/algorithm.rs` | Add `on_epoch` parameter to `Algorithm::train` |
| `src/competition.rs` | Add `verbose` field, `new_verbose()`, callback wiring |

### Algorithm implementations (mechanical — add callback invocation)

| File | Impl | Change |
|------|------|--------|
| `src/cem.rs` | `Cem` | Add `on_epoch` param, call `on_epoch(&metrics)` at end of epoch loop |
| `src/reinforce.rs` | `Reinforce` | Same |
| `src/ppo.rs` | `Ppo` | Same |
| `src/td3.rs` | `Td3` | Same |
| `src/sac.rs` | `Sac` | Same |
| `src/competition.rs` | `MockAlgorithm` | Restructure from `map().collect()` to `for` loop + callback |

### Call site migration (mechanical — add `&|_| {}`)

| File | Call sites | Change |
|------|-----------|--------|
| `src/cem.rs` | 2 (tests) | Add `&\|_\| {}` |
| `src/reinforce.rs` | 2 (tests) | Add `&\|_\| {}` |
| `src/ppo.rs` | 3 (tests) | Add `&\|_\| {}` |
| `src/td3.rs` | 2 (tests) | Add `&\|_\| {}` |
| `src/sac.rs` | 2 (tests) | Add `&\|_\| {}` |
| `src/autograd_policy.rs` | 1 (convergence test) | Add `&\|_\| {}` |
| `src/competition.rs` | 1 (runner) + 4 (unit tests via MockAlgorithm) | Callback from verbose logic / `&\|_\| {}` |

### Competition tests (use verbose)

| File | Change |
|------|--------|
| `tests/competition.rs` | Tests 8-9: `Competition::new_verbose(...)` instead of `new(...)` |

**Total**: 2 logic changes, 6 mechanical impl changes, ~17 mechanical
call site changes. Zero new files, zero new dependencies.

---

## 6. What NOT to change

- **Bevy examples**: They implement their own training loops using shared
  components directly. They don't go through `Algorithm::train`. No change
  needed.
- **The `EpochMetrics` struct**: Already has everything we need (epoch,
  mean_reward, done_count, total_steps, wall_time_ms, extra). No new
  fields.
- **Algorithm internals**: The training loop logic stays identical. We
  just call the callback with the metrics struct that's already being
  built.

---

## 7. Implementation order

1. Change `Algorithm` trait in `algorithm.rs`
2. Update all 6 implementations (add param + call callback)
3. Update all ~17 call sites (add `&|_| {}`)
4. Add `verbose` field + `new_verbose()` to `Competition`
5. Wire verbose callback in `Competition::run`
6. Switch Tests 8-9 to `new_verbose()`
7. Verify: `cargo test -p sim-ml-bridge --lib` (296 tests)
8. Verify: `cargo clippy -p sim-ml-bridge --tests -- -D warnings`

---

## 8. Future extensions (not in scope)

These are natural next steps that the callback pattern enables, but we
don't build them now:

- **Early stopping**: Caller wraps the callback with a condition check,
  sets a flag. Algorithm checks the flag (requires a `bool` return or
  `Arc<AtomicBool>` — future work).
- **Progress bars**: Caller passes a `indicatif` progress bar update as
  the callback.
- **Metric streaming**: Callback writes to a channel for real-time
  plotting.
- **Bevy example logging**: If Bevy examples ever use `Algorithm::train`,
  they get observability for free.

---

## 9. Success criteria

1. All 296 existing tests pass unchanged (callback is `&|_| {}`)
2. Competition Tests 8-9 print epoch-level progress when run
3. Zero new types, zero new dependencies
4. The pattern is established: any future algorithm automatically gets
   observability by implementing the callback contract
