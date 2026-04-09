# 2026-04-09 (part 8) — Item 6: `model.timestep` variability across steps

> Extracted from `MASTER_PLAN.md` §7 part 08 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Recon item 6 under sharpen-the-axe. Affects whether the
  thermostat reads `h = model.timestep` fresh inside `cb_passive` every
  step or precomputes the scale at install time.
- **Field declaration**: `pub timestep: f64` at
  `sim/L0/core/src/types/model.rs:828`. Plain public field, no setter
  shadowing it. (A second `timestep` field exists at `model.rs:1105`
  on `LengthRangeOpt`, but that's a length-range computation option
  unrelated to the main `Model::timestep`.)
- **Catalog of every write site in the workspace** (categorized):
  1. **Defaults / construction** (set once, never touched again):
     - `sim/L0/types/src/config.rs:32,56,65,76` — Config presets
       (240Hz, 60Hz, 1000Hz, 30Hz).
     - `sim/L0/core/src/types/model.rs:1123` —
       `LengthRangeOpt::default` (different field).
     - `sim/L0/core/src/types/model_init.rs:330` — Default model init
       value (500Hz, 0.002s).
     - `sim/L0/core/src/types/model_factories.rs:132,240,331` — Model
       factory presets (240Hz).
     - `sim/L0/mjcf/src/types.rs:513` — MJCF default.
     - `sim/L0/mjcf/src/builder/init.rs:144` — MJCF builder init.
     - `sim/L0/mjcf/src/builder/mod.rs:772` — Builder copying option
       into model.
     - `sim/L0/mjcf/src/parser.rs:246` — MJCF parser writing into
       option struct.
  2. **Test fixtures setting timestep before any stepping starts**:
     - `sim/L0/core/src/forward/fiber.rs` × 6 (all
       `model.timestep = 0.001`).
     - `sim/L0/core/src/constraint/jacobian.rs` × 2.
     - `sim/L0/core/src/jacobian/mod.rs` × 7.
  3. **Validation tests setting an invalid value to assert rejection**:
     - `sim/L0/types/src/config.rs:485,488,491` — `-0.01`, `0.0`, NaN.
     - `sim/L0/mjcf/src/validation.rs:804,811,818,825` — same pattern.
     - `sim/L0/tests/integration/split_step.rs:126` —
       `model.timestep = 0.0` to test the `step1()` error path
       (already seen in item 3 part 5).
  4. **Internal sub-component sync** (NOT `Model::timestep` itself):
     - `sim/L0/core/src/forward/muscle.rs:134` — `lr_model.timestep =
       opt.timestep;` writes a temporary length-range solver model,
       not the user's main model.
- **Crucial finding**: **There is no site in the codebase where
  `Model::timestep` is mutated *between* steps during a running
  simulation.** Every write is either init/default, test-fixture
  pre-setup, or validation-test invalid-value injection. The field is
  *de facto constant after model init*. The codebase convention is
  "set once, leave alone."
- **No compile-time enforcement, however**. The field is `pub` and
  freely mutable. Any user with `&mut Model` can rewrite it between
  steps. The codebase simply doesn't do this. There is no documented
  policy *because there is no usage pattern that needs one*.
- **Implication for the thermostat**: The cb_passive callback receives
  `&Model` (immutable) and `&mut Data`. It can read `model.timestep`
  for free every step. Two schemes for how to use it:
  - **Scheme A — read fresh every step**: thermostat reads
    `h = model.timestep` inside `cb_passive`, computes
    `(2γkT/h)^{1/2}` per draw. Current API sketch matches this.
    - Pros: Robust to any user manipulation (legal but
      unconventional). Constructor stays simple — no `h` parameter.
      Matches the cb_passive idiom (the callback already gets
      `&Model`; using it costs nothing). One extra `f64` read +
      sqrt per DOF per step.
    - Cons: One sqrt per DOF per step that *could* be precomputed.
  - **Scheme B — cache at install time**: thermostat captures `h` in
    its constructor (or `install()`) and stores the per-DOF scale
    `sigma_i = (2γ_i kT/h)^{1/2}` as a `DVector<f64>`. The hot path
    becomes one multiply per DOF per step, no sqrt.
    - Pros: One sqrt per DOF saved per step.
    - Cons: Becomes silently wrong if the user mutates
      `model.timestep` after install. Constructor needs `h` as input,
      OR `install()` needs to read it from the model — adds an
      `install`-time read that has its own ordering hazard ("what if
      the user installs the thermostat *before* setting timestep?").
- **Performance accounting**: Phase 1 is `nv=1` and runs 10⁵ steps.
  At Scheme A's cost of one sqrt + one read per DOF per step, that's
  10⁵ sqrts total — order of microseconds on any modern CPU,
  negligible compared to the constraint solver + RNG draws. Even at
  Phase 4 (8 coupled bistable elements, maybe `nv ~ 8-16`), the cost
  is 10⁶-ish sqrts over a 10⁵-step run — still negligible. The
  precompute is a true micro-optimization at every plausible phase
  scale.
- **Recommendation: Scheme A — read fresh every step**. Reasoning:
  1. The cost is negligible at every plausible phase scale (Phase
     1–7). Optimizing it is premature; the cost of being wrong is
     not.
  2. Robustness against user mutation is free at this cost. A user
     who legally rewrites `model.timestep` between steps gets correct
     physics from the thermostat, not silently wrong physics.
  3. Constructor stays minimal: `LangevinThermostat::new(gamma,
     k_b_t, seed)` — three parameters, no `h`. Simpler API,
     simpler tests.
  4. No install-order hazard. The thermostat doesn't care when the
     timestep is set, only that it's set when `cb_passive` fires.
  5. Matches the cb_passive idiom: the callback gets `&Model` for a
     reason; using it is the correct pattern. Caching would defeat
     the purpose of the callback receiving the model.
  6. **Loud failure beats silent failure** — same reasoning as item
     2 part 3 and item 4 part 6. Scheme B's failure mode (silent
     wrong physics after a `model.timestep` mutation) is exactly
     the class sharpen-the-axe forbids.
- **Confidence**: high. This is a low-stakes decision (the costs of
  both schemes are well below noise floor), but the principled
  choice still points to A.
- **Implication for the spec**: Document that the thermostat reads
  `h = model.timestep` inside `cb_passive` every step, and that this
  is by design — it's robust to legal-but-unusual user mutation of
  `model.timestep` between steps, and the cost is negligible. State
  that the codebase convention is "set once, leave alone" (no
  in-tree write site mutates timestep mid-simulation), but that the
  thermostat doesn't depend on the convention — it would still be
  correct if the convention were broken.
- **DECISION (user confirmed)**: **Scheme A — read fresh every step.**
  User cited Scheme B's silent-failure potential as the deciding
  factor — same reasoning as item 2 part 3, item 4 part 6, and the
  reasoning in this entry. Loud failure beats silent failure
  consistently across the recon line.
- **Item 6 RESOLVED.** Thermostat reads `h = model.timestep` inside
  `cb_passive` every step. Constructor signature is
  `LangevinThermostat::new(gamma, k_b_t, seed)` — no `h` parameter.
  No code, no spec, no Cargo.toml touched.
- **Next action**: Phase-1-blocking
  recon item 7 — statistical-test infrastructure conventions in
  `sim/L0/tests/`. Question: how does the existing test suite
  structure conformance/numerical tests, and what assertion
  patterns + tolerance idioms are in use? Determines where the
  Phase 1 equipartition test lives, what test crate it joins, and
  what shape its assertions take.

