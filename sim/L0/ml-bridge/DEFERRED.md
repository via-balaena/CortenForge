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

---

## Competition

- **`MlpStochasticPolicy`** — Hand-coded MLP stochastic policy (like
  `MlpPolicy` but with learned `log_std`). Would let SAC compete at
  level 0-1 MLP without autograd. Low priority now that
  `AutogradStochasticPolicy` exists, but would complete the level 0-1
  oracle set.
  *Origin*: competition.rs known limitation (Tests 1-7).
