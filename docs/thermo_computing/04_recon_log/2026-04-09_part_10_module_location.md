# 2026-04-09 (part 10) — Item 8: thermostat module location

> Extracted from `MASTER_PLAN.md` §7 part 10 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Recon item 8 under sharpen-the-axe. Item 4 already
  set "keep sim-core rand-free" as a heavy constraint and named the
  leading candidate (a sibling L0 crate `sim-thermostat`). Item 8
  exists to verify the public-API compatibility, name the location
  precisely, and surface any counter-evidence.
- **Question**: Does sim-core's `cb_passive` API expose enough
  *public* surface for an *external* crate to install a callback,
  with no need to access sim-core-private types?
- **Verification — every required type and method is `pub`**:
  - `Model::set_passive_callback<F>(&mut self, f: F)` at
    `sim/L0/core/src/types/model.rs:1237-1243` — public, generic,
    bound `F: Fn(&Self, &mut Data) + Send + Sync + 'static`. The
    setter constructs the internal `Callback(Arc<dyn Fn...>)`
    wrapper itself; **external crates never need to mention
    `CbPassive` or `Callback` types directly**.
  - `Model::clear_passive_callback(&mut self)` at `model.rs:1246`
    — public.
  - `Model` re-exported at `sim/L0/core/src/lib.rs:219` →
    `sim_core::Model`.
  - `Data` re-exported at `lib.rs:192` → `sim_core::Data`.
  - `DVector<f64>` re-exported via
    `pub use nalgebra::{DMatrix, DVector, ...}` at `lib.rs:257` →
    `sim_core::DVector`.
  - `Data::qfrc_passive` is `pub` (item 5 part 7).
  - `Model::timestep` is `pub` (item 6 part 8).

  **An external `sim-thermostat` crate can do everything it needs
  through `sim_core`'s public API with no friction.** No
  sim-core-private types are required, no sim-core code changes
  are required, no `pub(crate)` widening is required.

- **Type-level confirmation of the closure bound**: The thermostat's
  `Arc<LangevinThermostat>` (where `LangevinThermostat` contains
  `gamma: DVector<f64>`, `k_b_t: f64`, `rng: Mutex<ChaCha8Rng>`) is:
  - `Send + Sync`: `DVector<f64>` is Send + Sync; `f64` is
    Send + Sync; `Mutex<ChaCha8Rng>` is Send + Sync because
    `ChaCha8Rng: Send` (its state is plain bytes). ✓
  - `'static`: it owns all its state, no borrows. ✓
  - The closure `move |m, d| me.apply(m, d)` (where `me:
    Arc<LangevinThermostat>`) therefore satisfies
    `Fn(&Model, &mut Data) + Send + Sync + 'static`. ✓
- **Wrapper-type confirmation of the clone footgun**: Looking at
  `set_passive_callback`'s body:
  ```rust
  self.cb_passive = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
  ```
  The closure is wrapped in `Arc<dyn Fn...>` *inside* sim-core. So
  `Model::clone()` clones the outer `Callback` struct, which clones
  the inner `Arc`, which is a pointer-copy — both Models point at
  the same closure object. The closure's `move`-captured
  `Arc<LangevinThermostat>` is therefore physically shared between
  original and clone, which means the `Mutex<ChaCha8Rng>` is shared.
  **The clone footgun from item 7 is now confirmed at the type
  level**, not just observed via the test in `clone_with_callbacks`.
- **Existing sim/L0 sibling layout** (verified by directory listing):
  ```
  sim/L0/types       → sim-types
  sim/L0/simd        → sim-simd
  sim/L0/core        → sim-core
  sim/L0/gpu         → sim-gpu
  sim/L0/mjcf        → sim-mjcf
  sim/L0/urdf        → sim-urdf
  sim/L0/tests       → sim-conformance-tests
  sim/L0/ml-bridge   → sim-ml-bridge
  ```
  **8 existing sim/L0 sibling crates**. Naming convention is
  single-word lowercase (`types`, `simd`, `core`, `gpu`, `mjcf`,
  `urdf`) with hyphen for compound names (`ml-bridge`,
  `conformance-tests`). Package names are `sim-{dirname}`.
  **A new `sim/L0/thermostat/` → `sim-thermostat` fits cleanly.**
- **Two schemes compared** (per "Two schemes, then choose"):
  - **Scheme α — sim-core submodule**: Add a `thermostat`
    submodule at `sim/L0/core/src/thermostat.rs` (or alongside
    `forward/passive.rs`). Promote `rand`, `rand_chacha`,
    `rand_distr` from dev-deps to production deps in
    `sim/L0/core/Cargo.toml`. Re-export `LangevinThermostat` from
    `sim/L0/core/src/lib.rs`.
    - **Pros**: One fewer crate. Colocation with the integration
      point (`cb_passive`, `qfrc_passive`, `forward/passive.rs`).
      Discoverable: a reader of sim-core sees the thermostat
      naturally.
    - **Cons**: Promotes `rand` + `rand_chacha` + `rand_distr` to
      production deps of sim-core. **sim-core's identity changes
      from "deterministic physics anchor" to "physics anchor with
      stochastic infrastructure."** Every consumer of sim-core
      (sim-mjcf, sim-urdf, sim-ml-bridge, sim-conformance-tests,
      sim-bevy, every example, every downstream user) inherits the
      `rand` dep tree transitively, *whether or not they use the
      thermostat*. Violates item 4's heavy constraint.
  - **Scheme β — sibling L0 crate `sim-thermostat`**: New crate at
    `sim/L0/thermostat/` with `Cargo.toml` declaring `sim-core`,
    `nalgebra`, `rand`, `rand_chacha`, `rand_distr` as deps.
    `LangevinThermostat::install` takes `&mut sim_core::Model` and
    calls `model.set_passive_callback(...)`. sim-core stays
    rand-free in production.
    - **Pros**: sim-core's identity is preserved
      (deterministic-by-default, rand-free). Dep tree property
      "if you don't depend on sim-thermostat, your sim is rand-free"
      is observable from the dep graph alone — the kind of
      readability-from-types property that beginners benefit from.
      Aligns with the workspace's existing 8-crate sibling layout
      and the user's stated "narrow crates over monoliths"
      preference. Future companion features (Phase 2 free-body
      thermostat, Phase 3 bistable element library, Phase 4
      coupled-array thermostat) get a natural home alongside
      `LangevinThermostat`. Item 4's constraint is honored.
    - **Cons**: One more crate to maintain (Cargo.toml, lib.rs,
      lints, dev-deps). Discoverability slightly worse — a reader
      of sim-core won't see the thermostat without browsing
      siblings. (Mitigated by a one-line "see also" comment in
      sim-core's `forward/passive.rs:719` near the `cb_passive`
      invocation site, and by the spec / master plan.)
- **Recommendation: Scheme β — `sim-thermostat` sibling crate**.
  Confidence: high. Reasoning condensed:
  1. Item 4's heavy constraint is unambiguous: keep sim-core
     rand-free. Putting the thermostat in sim-core directly
     violates it.
  2. The public-API surface verification proves there is **zero
     technical friction** to the sibling-crate approach. No
     `pub(crate)` widening, no callback infrastructure changes,
     no sim-core code edits required.
  3. The R34 / narrow-crates / readability-from-types philosophy
     points the same direction (item 4 part 6 already documented
     this).
  4. The "one fewer crate" pro of Scheme α is real but small —
     the workspace already has 8 sibling sim-L0 crates, so the
     marginal cost of a 9th is well-amortized.
  5. The discoverability con of Scheme β is mitigable with one
     "see also" comment in sim-core and proper documentation in
     the master plan + Phase 1 spec.
- **Concrete location**: New crate at `sim/L0/thermostat/` with
  package name `sim-thermostat`. Files at Phase 1 implementation
  time:
  ```
  sim/L0/thermostat/
  ├── Cargo.toml         # deps: sim-core, nalgebra, rand,
  │                      #       rand_chacha, rand_distr
  └── src/
      └── lib.rs         # pub struct LangevinThermostat
                         # impl LangevinThermostat { new, install,
                         #                           clear, ... }
  ```
  **One file is enough at Phase 1 scale**. Phase 2+ may add
  submodules (`free_body.rs`, `articulated.rs`) as needed; the
  crate-as-collection-point gives them a home.
- **Test location — flagged for the Phase 1 spec, not decided here**:
  Three options for where the thermostat tests live:
  - **(α)** Inside `sim-thermostat/tests/` as integration tests of
    the sibling crate. Pro: tests with code, deps isolated. Con:
    diverges from the "all sim integration tests in
    sim-conformance-tests" convention.
  - **(β)** In `sim-conformance-tests` as
    `tests/integration/langevin_thermostat.rs` (per item 7
    recommendation). Pro: matches convention. Con:
    `sim-conformance-tests` gains `sim-thermostat` as a dep (and
    transitively `rand`/`rand_chacha`/`rand_distr`).
  - **(γ)** Hybrid: API/lifecycle/seeding tests in
    `sim-thermostat/tests/`, equipartition + cross-physics
    integration tests in `sim-conformance-tests`. Pro: each test
    lives where it makes most sense. Con: two homes, slightly
    worse discoverability.

  This is a Phase 1 spec design question, not a recon question.
  Item 8 is resolved on the *code* location; the *test* location
  goes in the spec.
- **DECISION (user confirmed)**: **Scheme β — `sim-thermostat`
  sibling crate at `sim/L0/thermostat/`**. Item 4's "keep sim-core
  rand-free" constraint is honored; the public-API surface is
  sufficient (no sim-core changes required); the sibling layout
  matches the existing 8 sim/L0 crates.
- **Item 8 RESOLVED.** Thermostat code will live in
  `sim/L0/thermostat/` as a new sibling L0 crate (`sim-thermostat`),
  depending on `sim-core`, `nalgebra`, `rand`, `rand_chacha`,
  `rand_distr`. sim-core stays rand-free in production.
- **Did NOT yet draft**: any code, any spec, any Cargo.toml
  changes, any new directories.
- **Phase-1-blocking recon round complete.** Item
  list status:
  - Item 2 — RESOLVED (part 3, then superseded by part 4).
  - Item 3 — RESOLVED (part 4 + part 5).
  - Item 4 — RESOLVED (part 6).
  - Item 5 — RESOLVED (part 7).
  - Item 6 — RESOLVED (part 8).
  - Item 7 — RESOLVED (part 9).
  - Item 8 — RESOLVED (this entry).

  Once item 8 is confirmed, the next action is **drafting the
  Phase 1 spec** (`docs/thermo_computing/PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`).
  All inputs are now collected. The spec needs to address:
  1. The base algorithm (Euler-Maruyama Langevin) and the
     equipartition validation gate.
  2. Composition section (field disjointness with RL writers,
     per part 5).
  3. Test section (callback-firing-count + equipartition + seed
     reproducibility, per part 7).
  4. **Statistical-test infrastructure design** — how the
     equipartition test asserts sample-mean-within-tolerance, and
     what shape the helper macro/fn takes (per part 9 finding 1).
  5. **Clone footgun resolution** — which of options A-D from
     part 9 finding 2 the spec adopts, and how the API enforces
     it (per part 9 finding 2).
  6. Cadence section (plain `step()`, no split-step required,
     per part 5).
  7. PRNG choice (`ChaCha8Rng + StandardNormal`, per part 6) and
     the constructor signature (`new(gamma, k_b_t, seed)`, per
     part 8).
  8. Module location (`sim/L0/thermostat/`, per this entry).
  9. Phase 5+ caveats from part 4 (FD perturbation gating,
     plugin ordering, BatchSim per-env instances).
  10. The validation parameter set (M=1, k_spring=1, γ=0.1,
      k_B·T=1, h=0.001, 10⁵ steps after burn-in) and the
      γ + T sweep grid.

  > **Update (chassis design round, 2026-04-09)**: Items 1-3
  > and 6-10 above are now answered by `THERMO_CHASSIS_DESIGN.md`
  > (Decisions 1-6) and the existing recon log entries. Items 4
  > and 5 — the two "new design surfaces" — are also resolved at
  > the chassis level: item 4 by Decision 5 (`WelfordOnline` +
  > `assert_within_n_sigma`), item 5 by Decision 3
  > (`PassiveStack::install_per_env` with defensive clear). The
  > Phase 1 spec inherits answers to all 10 items from the
  > chassis design round; the only design decision left for the
  > spec is choosing the validation parameter set fix
  > (α/β/γ — see the corrected Phase 1 validation paragraph in
  > §The Gap above and Decision 5 in the chassis doc).

