# sim-ml-bridge — Deferred Tasks

Items worth doing but not blocking current work. Each entry has enough
context to pick up cold.

---

## Observability

- **`Algorithm::n_params()`** — Add param count to the `Algorithm` trait
  so `Competition` verbose logging can print it in the start message
  (e.g., `[CEM] training on reaching-6dof (5382 params)...`). Currently
  omitted because the trait doesn't expose it. Straightforward: one new
  method, 5 impls delegate to their policy's `n_params()`.
  *Origin*: TRAINING_OBSERVABILITY_SPEC.md §4 deviation.

- **Early stopping via callback** — Extend `on_epoch` to support
  stopping training early (e.g., divergence detected, reward plateau).
  Options: return `ControlFlow<(), ()>` from the callback, or pass an
  `Arc<AtomicBool>` stop flag. The current callback pattern makes this
  a clean extension — no trait redesign needed.
  *Origin*: TRAINING_OBSERVABILITY_SPEC.md §8.

---

## Competition

- **`MlpStochasticPolicy`** — Hand-coded MLP stochastic policy (like
  `MlpPolicy` but with learned `log_std`). Would let SAC compete at
  level 0-1 MLP without autograd. Low priority now that
  `AutogradStochasticPolicy` exists, but would complete the level 0-1
  oracle set.
  *Origin*: competition.rs known limitation (Tests 1-7).

---

## Persistence

- **Save/load trained weights** — After a multi-minute competition run,
  there's no way to save the best policy for later inspection or
  reuse. Algorithms already expose `policy.params()` — serialization
  can wrap that without changing the trait. Consider serde on the
  network types or a simple `params.bin` dump.

---

## Reproducibility

- **Pin RNG algorithm** — Replace `StdRng` with an explicit algorithm
  (e.g., `ChaCha8Rng`) across all algorithms and builders. `StdRng` is
  documented as not reproducible across `rand` versions. `Cargo.lock`
  pins it in practice, but an explicit algorithm guards against silent
  result changes on dependency bumps. Mechanical find-and-replace.
