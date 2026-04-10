# 2026-04-09 (part 9) — Item 7: test infrastructure conventions

> Extracted from `MASTER_PLAN.md` §7 part 09 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Recon item 7 under sharpen-the-axe. Determines test
  crate location, file structure, model-construction idiom, naming
  convention, tolerance idiom, and where the new equipartition test
  lives.
- **Test crate**: `sim-conformance-tests` at `sim/L0/tests/`
  (`sim/L0/tests/Cargo.toml`). One workspace test crate, two
  `[[test]]` targets:
  - `mujoco_conformance` (path `mujoco_conformance/mod.rs`) — golden
    conformance tests against MuJoCo reference outputs.
  - `integration` (path `integration/mod.rs`) — every other sim
    integration test (collision, equality constraints, integrators,
    sensors, actuators, callbacks, etc.).

  Deps: `sim-mjcf`, `sim-urdf`, `sim-core`, `sim-types`, `nalgebra`,
  `approx`. **No `rand` / `rand_chacha` / `rand_distr` in the test
  crate's deps.** Adding statistical tests will need to add these
  (or pull them transitively via the `sim-thermostat` crate from
  item 8).
- **Convention 1 — model construction is by MJCF XML, not direct Rust
  construction**. Every callback test (`tests/integration/callbacks.rs`)
  and almost every passive test (`tests/integration/passive_forces.rs`)
  builds models like:
  ```rust
  let xml = r#"
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>...</worldbody>
  </mujoco>"#;
  let mut model = sim_mjcf::load_model(xml).expect("load");
  ```
  This is the dominant idiom; direct `Model::n_link_pendulum(...)`
  construction (used in `split_step.rs`) is the exception. The
  thermostat's 1-DOF damped harmonic oscillator should be expressed
  as MJCF XML to match convention.
- **Convention 2 — naming**. `^fn test_` prefix is dominant: 582
  matches across 23 files in `tests/integration/`. The non-prefixed
  style (`fn passive_callback_adds_force()` in callbacks.rs) is an
  outlier in a small subset of files. **Thermostat tests should use
  `test_` prefix** to match the dominant convention.
- **Convention 3 — tolerance assertions use `approx::assert_relative_eq!`**.
  20+ files import `approx::assert_relative_eq` and use the
  `epsilon = X` form:
  ```rust
  assert_relative_eq!(data.qpos[0], 0.5, epsilon = 0.05);
  ```
  Some files mix in plain `(actual - expected).abs() < epsilon`
  patterns for one-off tolerance checks (e.g., `callbacks.rs:39`).
  Both are accepted; `assert_relative_eq!` is the dominant idiom.
- **Convention 4 — doc comments per test explaining intent**. Every
  test in `callbacks.rs` and `passive_forces.rs` has a `///` doc
  comment explaining what is being verified, often including the
  MuJoCo physics reference being tested. Convention.
- **Adjacent test files for the thermostat — natural neighbors**:
  - `tests/integration/callbacks.rs` (DT-79) — already tests
    `cb_passive`, `cb_control`, `cb_contactfilter`, `cb_sensor`. The
    natural home for *infrastructure* tests on the thermostat
    (callback fires once per step, RNG advances by expected count,
    install/clear lifecycle, etc.).
  - `tests/integration/passive_forces.rs` — tests existing passive
    physics (springs, damping, friction loss). The natural physics
    neighborhood; the equipartition test could conceptually live
    here, though it's substantial enough to deserve its own file.
  - **Recommended new file**: `tests/integration/langevin_thermostat.rs`,
    registered in `tests/integration/mod.rs` as
    `pub mod langevin_thermostat;`. Holds the equipartition test +
    callback-firing-count test + reproducibility-from-seed test +
    parameter sweeps. Big enough as a test surface to deserve its
    own file; small enough to live as one file rather than a
    subdirectory.
- **CORRECTION to part 5 (item 3)**: Part 5 claimed *"The thermostat
  will be the workspace's first `cb_passive` consumer ... nothing in
  `sim/`, `examples/`, or any test currently uses it."* That was
  **partially wrong**. `tests/integration/callbacks.rs` already has:
  - `passive_callback_adds_force` (line 11) — unit test that
    `cb_passive` fires and writes to `qfrc_passive`.
  - `clone_with_callbacks` (line 148) — unit test that cloning a
    Model preserves the passive callback.
  - `none_callbacks_no_effect` (line 117) — sanity check.

  These are **unit tests of the cb_passive API itself**, not
  production consumers. The corrected claim: the thermostat will be
  the workspace's **first stateful `cb_passive` consumer**, and its
  first production consumer. The unit tests in callbacks.rs prove
  the API works; they do not exercise the *stateful* + *Mutex<RNG>*
  patterns the thermostat needs. Worth flagging for accuracy in
  future references back to part 5.
- **Genuine new finding 1 — no existing statistical-sampling tests
  in the workspace**. Verified by greps for `n_samples`,
  `sample_mean`, `0..100000`, `mean +/=`, `chi-square`, etc. The
  `0..10000` loops in `passive_forces.rs:358,672`,
  `model_data_pipeline.rs:280`, `validation.rs:1480`, and
  `collision_primitives.rs:1743` are all *time-evolution* loops
  (run the simulation forward), not Monte Carlo sample loops.
  No test in `sim/L0/tests/` averages a quantity over many samples
  and compares against a theoretical statistical value. **The
  thermostat's equipartition test will be the workspace's first
  statistical-sampling validation test.**
  - Implication: there is **no convention to inherit** for
    statistical assertions. The thermostat must establish one. The
    Phase 1 spec needs to think about whether `approx::assert_relative_eq!`
    is the right tool (probably not — it tests relative error, not
    sampling error) or whether to roll a small helper like
    `assert_within_sampling_error!(actual, expected, n_samples,
    expected_relative_std)` that captures the statistical-tolerance
    intuition. This is a Phase 1 spec design question, not a recon
    question.
  - Implication: the thermostat test sets the *style* for all
    future statistical validation in the thermo line — Phase 2
    (free + articulated body equipartition), Phase 3 (Kramers
    rate), Phase 4 (Ising/Gibbs joint distribution), Phase 5 (EBM
    target match). All of them will need similar statistical
    assertion infrastructure. Worth designing it deliberately at
    Phase 1, not stumbling into it.
- **Genuine new finding 2 — `Model::clone()` preserves passive
  callbacks AND shares captured state across clones**. This is the
  most important finding from item 7 and it surfaces a Phase 1 spec
  API design question. **`callbacks.rs::clone_with_callbacks`
  (lines 148-186) proves it explicitly**:
  ```rust
  let called = Arc::new(AtomicBool::new(false));
  let called_clone = Arc::clone(&called);
  model.set_passive_callback(move |_model, data| {
      called_clone.store(true, Ordering::SeqCst);
      data.qfrc_passive[0] += 1.0;
  });
  let model2 = model.clone();           // ← clone here
  data.forward(&model2).expect("forward");
  assert!(called.load(Ordering::SeqCst));  // ← shared Arc fired
  ```
  The captured `Arc<AtomicBool>` is observed to be set via the
  cloned model's callback firing. Therefore the closure is shared
  (or at least its captured state is) between the original and the
  clone. **Implication for the thermostat**: a user who builds a
  Model, installs a `LangevinThermostat` (which captures
  `Arc<LangevinThermostat>`), then clones the Model N times for N
  parallel envs gets:
  - **Shared `Arc<LangevinThermostat>` across all clones**.
  - **Shared `Mutex<ChaCha8Rng>` across all clones**.
  - **Shared RNG stream** — sample 1 goes to env 0, sample 2 to
    env 5, sample 3 to env 2, etc., based on whatever order the
    parallel envs happen to call the callback. **This is wrong
    physics** — each env's noise stream should be statistically
    independent, not interleaved with all the others.
  - **Mutex contention** on every `cb_passive` invocation across
    every parallel env. Performance disaster at high env counts.
- **Why this is a Phase 1 spec question, not a Phase 1 *implementation*
  problem**: Phase 1 itself is single-env (1-DOF DHO equipartition).
  No clone, no BatchSim, no problem. But the API the thermostat
  exposes in Phase 1 will set the pattern that BatchSim users in
  Phase 2+ will follow. If we ship a `LangevinThermostat::install`
  that silently breaks under clone, BatchSim users will hit it
  cold. Sharpen-the-axe says we surface this *now* and design the
  API with the answer in mind, even if Phase 1 doesn't exercise it.
- **Three possible API responses (sketches only — not deciding
  here)**:
  - **(A)** Document loudly that the user must install a *fresh*
    `LangevinThermostat` on each cloned model, *after* cloning.
    Pure documentation, no code change. Risk: silent footgun,
    exactly the failure mode sharpen-the-axe forbids.
  - **(B)** Provide a paired API:
    `LangevinThermostat::install_per_env(Arc<Model>, n_envs: usize,
    seed_base: u64) -> Vec<Arc<Model>>` that builds N models from
    one prototype, installs N independent thermostats with seeds
    derived from `seed_base`, returns the N models. Forces correct
    usage at the API level. Cost: more API surface, requires Arc
    rewrap.
  - **(C)** Provide a `LangevinThermostat::for_clone(seed: u64) ->
    LangevinThermostat` helper that the user calls explicitly after
    each clone, plus loud documentation. Middle ground.
  - **(D)** Investigate whether `set_passive_callback` could be
    redesigned to take a *factory* (`impl Fn() -> Box<dyn Fn...>`)
    that runs at clone time and produces a fresh closure per clone.
    Most invasive — touches sim-core's callback infrastructure.
    Probably out of scope for Phase 1; flag for the thermostat-and-
    BatchSim spec interaction.

  This list is *not* a decision; it's the option space the spec
  needs to consider. Item 8 (where the thermostat lives) and the
  Phase 1 spec design will weigh these against each other.

  > **Update (chassis design Decision 3, 2026-04-09)**: This
  > clone-footgun option space is now **resolved at the chassis
  > level**, not at the Phase 1 spec level. The chassis design
  > chose **option (B)** — `PassiveStack::install_per_env(prototype,
  > n, build_one)` — as the named API for BatchSim setups, with a
  > defensive `model.clear_passive_callback()` inside the loop
  > that makes the API bulletproof against the install-then-batch
  > order. Option (D) (modify sim-core) is named in the chassis
  > design as the *eventual* right answer if a future user
  > demands truly impossible-to-misuse semantics, but is
  > Phase-1-out-of-scope. See `THERMO_CHASSIS_DESIGN.md`
  > Decision 3 for the full trade-off analysis.
- **Caveat (caveat 3) from part 4 status updated**: Previously
  framed as "BatchSim parallel envs need their own
  `LangevinThermostat` instance — not a shared one — to keep RNG
  streams independent and avoid lock contention. The API should
  make the wrong choice hard." After item 7's finding, it's
  upgraded from "user discipline issue" to "easy-to-trigger
  silent footgun under the natural BatchSim workflow (clone the
  model)." The Phase 1 spec must address it explicitly, not just
  mention it.
- **Item 7 RESOLVED on its actual question** (test conventions
  catalogued, file structure clear, naming and tolerance idioms
  documented, the thermostat tests have a clear home). **Two
  side-findings flagged**: the statistical-test precedent void,
  and the clone-preserves-callback footgun. Neither blocks Phase
  1 implementation; both are inputs to the Phase 1 spec design.
- **Did NOT yet draft**: any code, any spec, any Cargo.toml
  changes.
- **Next action**: Phase-1-blocking recon item 8 — where the
  thermostat module lives (sim-core submodule vs. sibling L0 crate
  vs. inside a new crate). With item 4's "keep sim-core rand-free"
  constraint already set as heavy, the leading candidate is a new
  L0 crate (`sim-thermostat`). Item 8 will weigh this against any
  counter-evidence (e.g., does sim-core's `cb_passive` API expose
  enough surface for an external crate to install a callback?
  needs verification).

