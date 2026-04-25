# L0 Architectural Plan

**Date:** 2026-04-25
**Companion to:** `L0_current_state.md` (the diagnosis)
**Status:** Specification. No code written yet. Designed to be read once and executed without ambiguity.

---

## 1. Problem in one paragraph

The "L0" label denotes a single architectural layer ("Bevy-free, WASM-buildable, pure compute substrate"). The codebase contains 13 crates wearing that label whose transitive release-dep counts span 20 to 165. The grader enforces the L0 invariant on the default-features dependency graph only; STANDARDS.md aspires to several stronger guarantees that are not enforced anywhere; CI is sized for capacity that GitHub's free runners don't have. Each individual decision that produced this state was reasonable in scope. The cumulative effect is that "L0" no longer denotes a property; it denotes a directory. Building soft-body Phases 1–5 on top of this guarantees the leaks compound.

The fix is not "remove the bevy_ecs feature." The fix is to make the architectural property real and enforced, then bring the codebase into compliance with it.

---

## 2. Architectural target

### 2.1 Four tiers, replacing the binary L0/L1

The current "L0 vs L1 (sim-bevy)" split is too coarse. A `sim-types` crate (22 deps, pure data) and a `sim-mjcf` crate (150 deps, file format support) cannot share a layer label and have that label mean anything. The proposed structure introduces a finer split:

| Tier | Definition | Max release deps | Banned-prefix exclusions | Crates |
|---|---|---:|---|---|
| **L0** | Pure compute substrate. Math, types, algorithms, in-memory data structures. No file I/O, no graphics, no GPU compute, no game-engine integration. WASM-buildable. | **60** | `bevy*`, `winit`, `wgpu*`, `image*`, `zip*`, `zstd*`, `criterion`, `plotters*`, `mesh-io`, `sim-mjcf`, `sim-urdf` | `sim-types`, `sim-simd`, `sim-core`, `sim-thermostat`, `sim-ml-chassis` (no bevy feat), `sim-opt`, `sim-soft` (no gpu-probe feat), `cf-spatial`, `cf-geometry` |
| **L0-io** | Format parsers, asset loaders, GPU compute kernels. Allowed format-specific heavy chains (image, zip, mesh-io) and wgpu compute (NOT bevy). WASM-buildable not required. | **180** | `bevy*`, `winit` | `sim-mjcf`, `sim-urdf`, `mesh-io`, `sim-gpu` |
| **L0-integration** | Composes L0 + L0-io into higher-level abstractions. Bevy-free. Inherits L0-io weight legitimately. | **180** | `bevy*`, `winit` | `sim-rl`, `sim-therm-env`, `sim-conformance-tests` |
| **L1** | Visualization, ECS integration, interactive runtime. Bevy/winit/wgpu allowed unconditionally. | unbounded | (none) | `sim-bevy`, `cortenforge`, future `sim-ml-chassis-bevy` |

**How the dep-count maximums were derived (not arbitrary):**
- L0 = 60: forces real architectural cleanup, not just admission of the status quo. Current L0-pure crates are 20–48 (sim-types, sim-simd, sim-core, sim-thermostat, sim-opt, sim-ml-chassis default). sim-soft default is 78 — purely from the nalgebra/simba math ecosystem, no architectural debt — and is the one crate that needs explicit accommodation (see "tier-up features" below). 60 gives the L0-pure crates ~20 deps of headroom for normal Rust dependency growth without admitting heavy chains.
- L0-io = 180: matches current heaviest legitimate L0-io crate (sim-mjcf at 157 under --all-features) plus 20 deps of headroom. Tighter than the original "200" because 200 was admission-of-status-quo; 180 is "current weight + small buffer."
- L0-integration = 180: matches the heaviest current crate (sim-therm-env at 163) + headroom.

If a crate exceeds its tier max, the grader fails the build with a message naming the violating crate and its current count. The fix is either (a) trim deps, or (b) explicitly reclassify to a higher tier — both visible in PR review.

**The key invariants:**
- **L0 is binary-clean for embedded/WASM/server consumers.** No surprises, no opt-in features that pull L1-typical chains (except declared tier-up features below).
- **L0-io is the heavy-but-headless layer.** Format parsing, GPU compute, asset loading. Consumers who don't want the weight don't depend on L0-io crates.
- **L0-integration is the composition layer.** Inherits L0-io weight legitimately. Distinct from L1 because it doesn't require Bevy.
- **L1 is graphics/runtime.** What it always was.

**sim-gpu reclassified to L0-io.** The original plan put sim-gpu in L0 with a "MEDIUM (wgpu by design)" hand-wave. That's incorrect — wgpu is L1-typical and sim-gpu's purpose ("GPU-accelerated SDF collision") makes wgpu a load-bearing release dep. sim-gpu is L0-io: bevy-free, headless-server compatible, but allowed wgpu under its tier rules.

This is not a directory move. The crates stay where they are physically. The layer is a *property* attached to each crate via Cargo.toml metadata, enforced by the grader. Directory reorganization is optional and deferred (§7).

### 2.1a Tier-up features (the sim-soft gpu-probe pattern)

Some L0 crates legitimately want an opt-in feature that temporarily upgrades their tier. sim-soft's `gpu-probe` is the canonical example: under default features, sim-soft is L0-pure (78 deps, nalgebra ecosystem only); under `--features gpu-probe`, it pulls wgpu/bytemuck/pollster (120 deps) and behaves as L0-io for that build configuration.

The grader handles this via explicit declaration:

```toml
[package.metadata.cortenforge]
tier = "L0"
# Features that promote the crate to a higher tier when enabled.
# Each entry: feature_name = target_tier_name. The grader applies the
# target tier's banned-prefix list and dep-count max when the feature
# is enabled (e.g., during --all-features grading).
tier_up_features = { gpu-probe = "L0-io" }
```

Grader logic (per crate):
1. Default-features graph → check against declared tier ("L0").
2. `--all-features` graph → check against declared tier OR any tier reachable via `tier_up_features` (whichever is stricter for the dep set).
3. Without `tier_up_features` declaration, `--all-features` is checked against the declared tier — exactly catching leaks like sim-ml-chassis's bevy_ecs feature.

This makes opt-in heavy features explicit: a leak (bevy_ecs) and an intentional tier-up (gpu-probe) become structurally distinguishable. The bevy_ecs leak fails grading because no `tier_up_features` declaration exists for it; the only fixes are (a) extract to an L1 crate (the chosen surgery) or (b) declare `bevy = "L1"` and accept the consequences.

### 2.2 The dev-dep invariant

Dev-deps must not pull cross-tier weight into the test compile graph. Specifically:

- **L0 crates' dev-deps must stay in L0.** A `sim-thermostat` test compile must not pull `sim-mjcf`. If a test needs an MJCF model fixture, the fixture is constructed in-line (Rust `MjcfModel::new()` calls), not loaded from XML.
- **L0-io crates' dev-deps may include L0 + L0-io.** Internal coherence allowed.
- **`criterion` is not a workspace-level dev-dep.** Crates with benchmarks declare it directly with the bench in a separate `-benches` crate (see §4.3).

This invariant solves the +130 dev-dep blowup pattern that dominates current test-compile times.

---

## 3. The four required surgeries

Each surgery is independently shippable and testable. They can be sequenced (recommended) or parallelized within the branch.

### 3.1 Extract `sim-ml-chassis-bevy` (the bevy_ecs leak)

**Current state:** `sim-ml-chassis` has `bevy = ["dep:bevy_ecs"]` opt-in feature. Default graph: clean. `--all-features` graph: pulls 9 bevy_* sub-crates. Grader doesn't catch.

**Target state:** `sim-ml-chassis` has no `bevy` feature, no `bevy_ecs` dep. New crate `sim-ml-chassis-bevy` lives at L1, depends on `sim-ml-chassis` + `bevy_ecs`, exposes the integration types.

**Consumer audit (verified 2026-04-25):** `rg 'features.*=.*\[.*"bevy"' --type toml` finds **7 example crates** currently consuming `sim-ml-chassis = { features = ["bevy"] }` — all under `examples/fundamentals/sim-ml/`. The feature is load-bearing for the example layer, not orphan code. Migration is required, not optional.

**Mechanics:**
1. Identify which symbols in `sim-ml-chassis` source are `#[cfg(feature = "bevy")]`-gated. Move them to the new crate.
2. New crate Cargo.toml:
   ```toml
   [package]
   name = "sim-ml-chassis-bevy"
   description = "Bevy ECS integration for sim-ml-chassis"
   # ...workspace inheritance...
   [dependencies]
   sim-ml-chassis = { workspace = true }
   bevy_ecs = "0.18"

   [package.metadata.cortenforge]
   tier = "L1"
   ```
3. Workspace `Cargo.toml`: register the new crate. Path: `sim/L1/ml-chassis-bevy/` (since the directory implies tier today; physical location can change later under §7).
4. Migrate the 7 example crates: each switches `sim-ml-chassis = { features = ["bevy"] }` to `sim-ml-chassis = { workspace = true }` + `sim-ml-chassis-bevy = { workspace = true }`. Update `use` paths to import bevy-aware types from `sim_ml_chassis_bevy::` instead of `sim_ml_chassis::`.
5. Delete `bevy = [...]` from `sim-ml-chassis/Cargo.toml` features. Delete `bevy_ecs` from its `[dependencies]`. Delete `#[cfg(feature = "bevy")]` blocks (they're now in the L1 crate).
6. Verify: `cargo tree -p sim-ml-chassis --all-features --prefix none --format '{p}' | grep '^bevy'` returns empty.

**Tier label nuance:** Calling the new crate "L1" is a slight stretch — `bevy_ecs` is just an ECS, not the full Bevy renderer/winit/wgpu chain. It's "Bevy-the-engine" only nominally. Two acceptable framings:
- (a) L1 means "Bevy-aware in any form" — the cleaner taxonomy. We keep the binary distinction at the L0/L1 boundary.
- (b) Introduce a separate "L0-bevy-ecs" tier for crates that pull bevy_ecs but not the renderer. More precise but adds complexity for one crate.
- Recommend (a). The marginal precision of (b) doesn't pay for itself unless we end up with multiple bevy_ecs-only adapters.

**Risk:** medium, not "none." 7 example crates need coordinated migration; if any are missed, they'll fail to compile after the bevy feature is deleted. Mitigation: migrate examples in commit before the feature deletion, verify all build, then delete in subsequent commit. If an example is missed, the build break is immediate and obvious — easy to recover.

### 3.2 Sever sim-mjcf dev-deps from L0-pure crates

**Current state:** `sim-thermostat`, `sim-opt`, `sim-ml-chassis` have `sim-mjcf` as dev-dep, used for `cfg(test)` MJCF model fixtures. This drags the entire image+zip+mesh chain into their test compiles (~100+ extra crates, +130 in ml-chassis case). The "cyclic-sim-rl-dev-dep workaround" comment at `sim-ml-chassis/Cargo.toml:37-44` documents that this was deliberate, with reason.

**Target state:** None of `sim-thermostat`, `sim-opt`, `sim-ml-chassis` depend on `sim-mjcf` in any form. Test fixtures use in-line `MjcfModel` construction (or trivial `Model` mocks where MJCF isn't needed).

**Verified architecture (2026-04-25):** `sim-mjcf::load_model(xml)` returns `sim_core::Model` (defined at `sim/L0/core/src/types/model.rs`). `Model` already has factory APIs at `sim/L0/core/src/types/model_factories.rs` and `model_init.rs`. **The dependency to remove is on the parser (sim-mjcf), not the type (Model).** Tests can build `Model` directly via sim-core factories without going through MJCF XML parsing at all. The crates already depend on sim-core, so no new dep is added.

**Use-site enumeration (verified):**
- `sim-thermostat`: 7+ call sites in `stack.rs`, `ising_learner.rs`, `langevin.rs`, `lib.rs` doc-test. Each calls `sim_mjcf::load_model(xml)` with custom XML per test.
- `sim-ml-chassis`: 9+ call sites in `rollout.rs`, `test_stock_tasks.rs`, `space.rs`, `task.rs`, `vec_env.rs`, `env.rs`. Heavy use of shared XML constants (`MJCF_2DOF`, `MJCF_6DOF`, `pendulum_xml()`).
- `sim-opt`: 0 direct uses found. The `sim-mjcf` dev-dep may be transitive-only (via sim-ml-chassis dev-dep). Verify before changing.

**Mechanics:**
1. **Build a fixture module in sim-core** (`sim/L0/core/src/test_fixtures/mod.rs`, `#[cfg(any(test, feature = "test-fixtures"))]`) that exposes builder functions producing the canonical test models: `pendulum()`, `dof_2()`, `dof_6()`, `simple_sphere()`, etc. Implementation uses sim-core's existing factory APIs — pure Rust, no XML parsing.
2. **For each affected crate:** add `sim-core = { workspace = true, features = ["test-fixtures"] }` (the feature is already a workspace dep, just enables the fixture module).
3. **Per call site:** replace `sim_mjcf::load_model(xml).unwrap()` with `sim_core::test_fixtures::pendulum()` (or the appropriate fixture). Tests that built ad-hoc XML need their model expressed in factory-API form once, then the fixture is reusable across the workspace.
4. **Tests that specifically validate MJCF parsing semantics** (rare in these crates — most use MJCF as a model source, not a parser test): move to `sim-mjcf`'s own test module or to `sim-rl` if they need both.
5. **Cyclic-sim-rl-dev-dep workaround dissolves.** No dep on sim-rl needed because no dep on sim-mjcf needed.
6. **Delete `sim-mjcf` from `[dev-dependencies]`** of `sim-thermostat`, `sim-ml-chassis`, and `sim-opt` (after verifying sim-opt's actual usage).

**Cost:** Two phases.
- *Fixture authoring* (one-time, in sim-core): ~150–300 LOC for the canonical fixture set. This is the load-bearing investment; everything downstream becomes ~5 LOC per call site.
- *Per-crate migration*: ~5 LOC per call site × ~17 sites = ~85 LOC of changes. Plus removing dev-dep lines.
- Total: ~250–400 LOC across the affected files. Significantly more than my original "50–200 per crate" estimate (which was naive).

**Risk:** medium-high. Three concrete risks: (i) the fixture API needs to be expressive enough to cover existing test variations (some tests use weird XML edge cases — verify per call site that the factory API can produce the same Model); (ii) coverage shifts if existing tests were incidentally exercising sim-mjcf parsing — track per-crate coverage delta; (iii) the canonical fixture set needs design review (what's the right level of abstraction — `pendulum()` vs `body_with_joints(joint_count: usize)`?). Recommend doing the fixture-design pass before the migration to avoid rework.

### 3.3 Carve `sim-conformance-tests` out of L0-pure deps

**Current state:** `sim-conformance-tests` has 152 release deps. Lives at `sim/L0/tests/`. The "tests" suffix suggests it's a test crate, but it has a release graph (it's a regular Cargo package with `[dependencies]`). It legitimately depends on sim-mjcf for MuJoCo conformance testing, which is its purpose.

**Target state:** This crate is L0-integration tier, not L0. The label change is the entire surgery — no code moves. The grader, once tier-aware, will simply assign it the higher dep-count budget.

**Mechanics:**
1. Add `cf_layer = "L0-integration"` to `[package.metadata]` in `Cargo.toml`.
2. Grader reads metadata (§5).
3. No code change.

This is a one-line surgery whose value is honesty.

### 3.4 Feature-gate `mesh-io` 3MF support

**Current state:** `mesh-io` unconditionally depends on `zip` (used only in `threemf.rs`). All consumers of mesh-io pay the zip+zstd+cc compile cost regardless of whether they touch 3MF.

**Target state:** `zip` is gated behind a `threemf` feature. Default: off. Consumers that need 3MF (printability, packaging) opt in.

**Consumer audit (verified 2026-04-25):** `rg -l 'threemf|3mf'` finds:
- `cf-design/src/solid.rs` — real consumer, uses 3MF for design output
- `mesh/tests/api_regression.rs` — regression test exercising 3MF
- mesh-io internals (the module itself + fuzz target)
So **two real consumer call sites need migration**: cf-design and the mesh regression test.

**Mechanics:**
1. `mesh-io/Cargo.toml`:
   ```toml
   [features]
   default = []
   threemf = ["dep:zip"]
   [dependencies]
   zip = { workspace = true, optional = true }
   ```
2. `mesh-io/src/lib.rs`: `#[cfg(feature = "threemf")] pub mod threemf;`
3. Migrate the 2 known consumers:
   - `cf-design`: add `mesh-io = { workspace = true, features = ["threemf"] }` to its Cargo.toml.
   - `mesh` (the umbrella crate, if it re-exports): same.
4. Verify default-features build of mesh-io has no zip in graph: `cargo tree -p mesh-io --no-default-features --prefix none --format '{p}' | grep -E '^(zip|zstd)'` returns empty.
5. Verify default-features build of every L0/L0-io/L0-integration crate has no zip in graph (since the contagion vector is severed at mesh-io, this should clear automatically).

**Cost:** ~15 LOC across mesh-io + 2 consumer Cargo.toml edits.

**Risk:** low. Feature gates are well-understood. The two consumers are known; if a third exists that grep missed, CI catches via test/clippy failure.

---

## 4. Secondary surgeries (improvements without redefining structure)

These are not required for the core architectural fix but should ride along.

### 4.1 sim-mjcf texture/HField feature gates

**Current state:** sim-mjcf unconditionally depends on `image` for textures and HFields. Consumers that don't load textured/HField models pay the cost.

**Target state:** `textures` and `hfields` are features in sim-mjcf, default-on (preserving compat). Consumers can opt out via `default-features = false`.

**Mechanics:**
1. `sim-mjcf/Cargo.toml`:
   ```toml
   [features]
   default = ["textures", "hfields"]
   textures = ["dep:image"]
   hfields = ["dep:image"]
   [dependencies]
   image = { version = "0.25", optional = true }
   ```
2. `#[cfg(feature = "textures")]` gates around `builder/mesh.rs:295` and texture-loading code paths.
3. `#[cfg(feature = "hfields")]` gates around `builder/mod.rs:1134` and HField code paths.
4. Texture/HField references in MJCF input → if feature off, return parse error with clear message.

**Value:** Future embedded/headless consumers of sim-mjcf can disable these for a leaner build. Current consumers see no behavior change (defaults preserve compat).

### 4.2 Move criterion benches out of test compiles

**Current state:** `criterion` is workspace dev-dep. Crates with benchmarks list it in `[dev-dependencies]`, which means every `cargo test` invocation compiles criterion + plotters + tinytemplate + ciborium chain (~9 crates).

**Target state:** Per-crate `-benches` companion crate at `sim/L0/<crate>/benches-crate/` (or similar) that owns the benchmark code and the criterion dep. The main crate has no criterion dep.

**Mechanics (per crate with benches):**
1. New crate `sim-ml-chassis-benches` with:
   ```toml
   [package]
   name = "sim-ml-chassis-benches"
   publish = false
   [dependencies]
   sim-ml-chassis = { workspace = true }
   criterion = { workspace = true }
   [[bench]]
   name = "bridge_benchmarks"
   harness = false
   ```
2. Move `sim-ml-chassis/benches/bridge_benchmarks.rs` → `sim-ml-chassis-benches/benches/bridge_benchmarks.rs`.
3. Remove `criterion` from `sim-ml-chassis/Cargo.toml` `[dev-dependencies]`.
4. Remove `[[bench]]` block from `sim-ml-chassis/Cargo.toml`.
5. Bench invocation changes from `cargo bench -p sim-ml-chassis` to `cargo bench -p sim-ml-chassis-benches`. Document in README/contributing notes.

**Value:** sim-ml-chassis test compile drops by ~9 crates. Same pattern applies to sim-simd, sim-core (if they have criterion benches; verify). Composable cost reduction.

**Trade-off:** workspace crate count grows by 3–5 (one per current bench-having crate). Acceptable — workspace already has 224 crates per the soft-body refactor.

**UX cost:** bench invocation changes from `cargo bench -p sim-ml-chassis` to `cargo bench -p sim-ml-chassis-benches`. Real friction for someone who runs benches regularly. Mitigation: document the convention in CONTRIBUTING.md and add a top-level `cargo bench --workspace` runs all bench crates correctly already (Cargo handles bench-target aggregation across workspace members).

---

## 5. Enforcement: extending the grader

Without enforcement, every fix above will silently un-fix itself within a year. The grader changes are the most load-bearing part of this plan.

### 5.1 Tier metadata

Each crate declares its tier in `Cargo.toml`:

```toml
[package.metadata.cortenforge]
tier = "L0"  # one of: "L0", "L0-io", "L0-integration", "L1", "Example", "Xtask"
# Optional: features that promote this crate to a stricter (heavier) tier
# when enabled. See §2.1a for the gpu-probe rationale.
tier_up_features = { gpu-probe = "L0-io" }
```

**Scope decision:** tier metadata is **required for all `sim-*`, `mesh-*`, `cf-*`, and `cortenforge*` crates** in the workspace (not just sim-*). The grader errors if missing for those prefixes, allowing the broader workspace policy to extend later. `examples/` and `xtask` already have implicit tier classification handled separately.

**Plan B (if Cargo.toml metadata feels wrong):** alternatives considered:
- (a) **Directory-driven tiers** — `sim/L0/<crate>/` is L0 by path. Rejected: forces directory moves on every reclassification, and current dir layout is wrong for the new tiers.
- (b) **Workspace-root `tiers.toml` file** — list crates per tier in one place. Rejected: review burden moves out of the affected Cargo.toml and into a global file; harder to grep "what tier am I editing?"
- (c) **Cargo.toml metadata** (chosen) — locality wins. Reviewers see the tier at the same time as deps changes.

### 5.2 New criterion: `Layer Integrity`

Replaces and subsumes the current `Bevy-free` criterion (criterion 6). Implementation in `xtask/src/grade.rs`.

**For each crate:**
1. Read tier from `Cargo.toml` metadata. Error if missing.
2. Run `cargo tree -p <crate> --prefix none --format '{p}'` under three configs:
   - `--no-default-features`
   - default features
   - `--all-features`
3. Run `cargo tree -p <crate> -e normal,dev --prefix none --format '{p}'` under the same three for dev-graph (test compile graph).
4. For each (config × graph_kind) combination:
   - Count unique transitive crates.
   - Check against tier's max:
     | Tier | Release max | Test max |
     |---|---:|---:|
     | L0 | 80 | 100 |
     | L0-io | 200 | 220 |
     | L0-integration | 200 | 220 |
     | L1 | unbounded | unbounded |
   - Check against tier's banned-prefix list:
     | Tier | Banned in any graph |
     |---|---|
     | L0 | `bevy*`, `winit`, `wgpu*`, `image*`, `zip*`, `zstd*`, `sim-mjcf`, `sim-urdf`, `mesh-io`, `criterion`, `plotters*` |
     | L0-io | `bevy*`, `winit` |
     | L0-integration | `bevy*`, `winit` |
     | L1 | (none) |

   *Note: this banned list for L0-io / L0-integration was corrected in the
   step-3 commit to drop `wgpu*`. The original §5.2 table cell included
   `wgpu*` as a typo: §2.1's tier definition, §2.1a's tier-up-feature
   rationale, the sim-gpu reclassification rationale, and §5.2's own
   "why this works" example all explicitly permit wgpu in L0-io
   (sim-gpu's GPU-accelerated SDF collision and sim-soft's gpu-probe
   tier-up). Banning wgpu in L0-io would contradict every other paragraph
   in the plan and fail sim-gpu and sim-soft+gpu-probe by design.*

5. Grade: A if all pass, F if any fail. Failure message names the violating dep + which config/graph it was found in.

**Rollout in two phases (CRITICAL — don't skip):**
1. **Warning phase.** Implement the criterion, but emit findings as warnings (visible in grader output, not failing the build). All known violations stay listed but don't block. This lets us land the surgeries (§3) one at a time, each clearing some warnings.
2. **Hard-gate phase.** After all surgeries land and the warning list is empty, flip the criterion to hard-gate. New PRs fail on any violation.

If we skip the warning phase and go straight to hard-gate, the grader will fail on every L0 crate the day the criterion lands, blocking all PRs until every surgery is complete. That serializes work that should be parallelizable per-surgery.

**Why this works:** every leak we found in the recon would fail this check.
- bevy_ecs in sim-ml-chassis under `--all-features`: caught by L0 release-graph banned-prefix check (no `tier_up_features` declaration for the bevy feature).
- gpu-probe in sim-soft under `--all-features`: PASSES because sim-soft declares `tier_up_features = { gpu-probe = "L0-io" }`, and L0-io permits wgpu. Distinguishes intentional opt-in from accidental leak.
- sim-mjcf as dev-dep in sim-thermostat: caught by L0 dev-graph banned-prefix check.
- 165-dep transitive blowup in sim-thermostat: caught by L0 dev-graph 100-max.
- 152-dep sim-conformance-tests: passes after tier reclassification to L0-integration.

### 5.3 New criterion: `WASM compatibility (L0 only)`

STANDARDS.md §6 lists "Can be compiled for `--target wasm32-unknown-unknown` without bevy" as an L0 requirement. The CI WASM job currently tolerates failures. The grader doesn't check.

**Implementation:**
1. For tier == L0 only, run `cargo check -p <crate> --target wasm32-unknown-unknown --no-default-features 2>&1`.
2. Grade: A if exit 0, F otherwise. Failure message includes the rustc error.
3. L0-io / L0-integration / L1: N/A (skipped, marked NA in report).

**One-time cost:** L0-pure crates that don't currently build for WASM need fixes. Likely few — `sim-types`, `sim-simd`, `sim-core` are clean candidates; others may need conditional compilation or feature splits.

### 5.4 Existing criteria: keep, possibly tighten

- **Coverage**: unchanged.
- **Documentation**: unchanged.
- **Clippy**: unchanged.
- **Safety**: unchanged.
- **Dependencies (justification scan)**: keep, but the new "Layer Integrity" criterion subsumes the transitive-bloat concern. Could drop the "10+ deps = heavy" informational text since it's no longer load-bearing.

---

## 6. CI rework

The current CI has structural mismatches with both the grader (redundancy) and the runner constraint (capacity). Restructure:

### 6.1 Drop `--all-features` from the test matrix

`cargo test --all-features` against 23 crates on free runners is the immediate cause of the 38-min wall time. The grader, post-§5.2, will already enforce that all-features doesn't pull cross-tier weight; the test matrix doesn't need to verify the same property by recompilation.

**Target test-job invocation:**
```yaml
- name: Run tests (default features)
  run: |
    cargo test \
      -p sim-types -p sim-simd -p sim-core ...
- name: Run tests (specific feature combos for crates that have them)
  run: |
    cargo test --features gpu-probe -p sim-soft
    # Add per-feature combos here as features are introduced
```

**Math behind the savings estimate:**
- Current `cargo test --all-features` against 23 crates: 38–60+ min observed (when bevy_ecs + image + zip + criterion chains all compile).
- xtask Grade currently runs `cargo test` (default features) per crate as part of grading: total job time 14 min for 11 L0 + cf-* + mesh-* crates. That implies ~30–60s per crate of default-features test execution + compile.
- Post-refactor, the test matrix runs default features only on the same 23 crates. Linear extrapolation: 23 × ~40s ≈ 15 min, plus a separate ~2 min step for explicit feature-combo tests (gpu-probe, etc.).
- Total estimated: 15–20 min vs current 38–60 min. **Roughly 2–3x faster.**

The savings come specifically from **not paying the bevy_ecs/image/zip/criterion compile cost on every consumer's test build.** For crates whose default-features graph is small, the savings per-crate are large (sim-ml-chassis, sim-thermostat). For crates whose default == --all-features (most others), there's no savings — but no penalty either. Net: substantial.

### 6.2 Slow-tier separation

Some tests are inherently slow (sim-conformance-tests, anything with `#[ignore]`'d heavy fixtures). Move them to a separate `slow-tests` job with longer timeout, ubuntu-only (skip macos/windows for slow tests):

```yaml
slow-tests:
  runs-on: ubuntu-latest
  steps:
    - run: |
        cargo test -p sim-conformance-tests -- --include-ignored
        # Other heavy suites here
```

The Quality Gate `needs:` list keeps both `test` and `slow-tests`.

### 6.3 Stop recompiling what xtask Grade already compiles

xtask Grade runs `cargo test` (default features) per crate as part of grading. The CI test matrix re-runs the same per-crate tests under `--all-features`. Either:
- (a) Drop the test matrix entirely and rely on xtask Grade's per-crate test invocation, plus a slow-tier job for cross-OS smoke testing (ubuntu only), or
- (b) Keep the matrix but only for cross-OS validation: limit to maybe 3–5 crates that are platform-sensitive (FP determinism, mmap behavior, file-path handling), not all 23.

Recommend (b). It validates the actual cross-OS concern (post-bug-finding from the artifact round-trip incident) without re-paying the full test cost.

### 6.4 WASM job becomes hard gate

Currently `::warning::` only. After §5.3 lands, WASM compat for L0 is enforced by the grader, so the CI WASM job can either:
- (a) Be removed (grader covers it), or
- (b) Stay as belt-and-suspenders, but flip to error (`exit 1`) on failure.

Recommend (a) — grader is single source of truth.

---

## 7. Workspace reorganization (DEFERRED, optional)

The current directory structure is `sim/L0/<crate>/`. Under the new tier system, crates have different tiers. Optional reorganization:

```
sim/L0/types, simd, core, thermostat, opt, ml-chassis, soft
sim/L0-io/mjcf, urdf
sim/L0-integration/rl, therm-env, conformance-tests
sim/L1/bevy, ml-chassis-bevy
```

**Cost:** Cargo `path = "..."` updates everywhere, all `use` paths unchanged (workspace dep names don't depend on directory paths, only `path` references do).

**Decision:** Defer until after §5 enforcement is green. Directory reflects tier; tier is metadata-driven; reorg is documentation, not function.

If skipped, document: "directory `sim/L0/` contains crates of multiple tiers; tier is declared in each Cargo.toml metadata block."

---

## 8. Sequenced execution plan

This branch (`feature/phase-0-track-a-bundle`, retitled) executes the work in the order that minimizes regression risk. Each step is a commit boundary; CI runs at end-of-branch only (per the user's CI economics constraint). Pre-squash tag at end is optional — this branch isn't a study with chapter-hash anchors, but the audit trail value is high enough that tagging is recommended regardless.

| Order | Step | Surgery | Verify by | LOC est. |
|---:|---|---|---|---:|
| 1 | Land Phase 0 commit 2 (validate wiring) | Original Phase 0 §2 Change 2 | Local: `cargo xtask grade sim-mjcf` A | ~10 |
| 2 | Add tier metadata to all sim-*/mesh-*/cf-* crates | §5.1 | Manual review of metadata block per crate | ~80 (Cargo.toml only) |
| 3 | Implement `Layer Integrity` criterion in **WARNING MODE** | §5.2 phase 1 | New unit tests; criterion runs and reports findings but doesn't fail the grade | ~350 |
| 4 | Implement `WASM compatibility` criterion in **WARNING MODE** | §5.3 phase 1 | Reports per-crate WASM build status; doesn't fail | ~80 |
| 5 | Surgery: §3.4 (mesh-io threemf feature) | mesh-io + 2 consumer Cargo.toml | `cargo tree -p mesh-io --no-default-features` shows no zip; cf-design opts in correctly | ~15 |
| 6 | Build sim-core test fixtures (§3.2 prerequisite) | New `sim-core::test_fixtures` module | Module unit-tested; existing tests in sim-core unaffected | ~250 |
| 7 | Surgery: §3.2 (sever sim-mjcf dev-deps) | Per-crate test rewriting using sim-core fixtures | Layer Integrity warning count drops for sim-thermostat, sim-opt, sim-ml-chassis | ~100 (per crate × 3) |
| 8 | Surgery: §3.1 (extract sim-ml-chassis-bevy + migrate 7 examples) | New L1 crate + example migrations | All 7 examples build; Layer Integrity warning gone for sim-ml-chassis | ~200 |
| 9 | Surgery: §3.3 (sim-conformance-tests tier reclass) | Tier metadata only | Layer Integrity warning gone | 1 line |
| 10 | Optional: §4.1 (sim-mjcf texture/HField gates) | sim-mjcf restructuring | Default behavior unchanged; `--no-default-features` builds lean | ~100 |
| 11 | Optional: §4.2 (criterion benches extraction) | New -benches crates | sim-ml-chassis test compile drops criterion | ~80 |
| 12 | **Flip Layer Integrity + WASM criteria to HARD GATE** | §5.2 phase 2 + §5.3 phase 2 | Run `cargo xtask grade-all`; all green | ~10 |
| 13 | CI rework: §6 (drop --all-features matrix, slow-tier separation) | quality-gate.yml changes | One full CI run on this branch validates all surgeries + new CI architecture | ~80 (yaml) |
| 14 | Update STANDARDS.md to match new criteria | Doc | Manual review | ~150 |
| 15 | Pre-squash tag (`feature/l0-foundation-pre-squash`), squash-merge PR | — | — | — |

**Total work estimate:** ~1500–1700 LOC across ~15 commits. Realistic session-count: **5–8 focused sessions** depending on per-step depth. Step 6 (fixture authoring) and step 8 (example migration) are the largest individual steps; the rest are small.

**CI cost estimate post-merge:** Test matrix drops from ~30–60 min to ~15–20 min (per §6.1). xtask Grade remains ~14 min. Total Quality Gate fast-tier ~20 min total vs current 60+ min worst case. **Roughly 2–3x faster, with stronger enforcement.**

**Within-branch CI policy:** zero CI runs until step 13 lands. Local verification per step:
- `cargo xtask grade <affected crate>` after each surgery to confirm no regression
- `cargo build --workspace` after step 8 to confirm example migration didn't break anything
- One full CI run at end of branch to validate the integrated whole

---

## 9. Risks and decision points

### 9.1 Risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| §3.2 (sever dev-deps) breaks unrelated tests via coverage shift | medium | medium | Per-crate review of test conversions; coverage delta checked per crate via xtask grade |
| §3.1 (extract bevy adapter) misses a downstream consumer | low | low | `rg` for `features.*=.*\[.*"bevy"' --type toml` before cutting; downstream is currently empty |
| §4.1 (texture/HField gates) breaks tests that load textured models | medium | low | Defaults preserve current behavior; opt-out is the lean path |
| Grader extension (§5.2) over-restricts and trips passing crates | medium | medium | Implement criterion + dry-run on existing codebase before flipping to hard gate; tune banned-prefix list per dry-run output |
| WASM criterion (§5.3) reveals L0 crates that have never built for WASM | high | medium | Triage per-crate: (a) fix to be WASM-compatible, (b) feature-gate the offending dep, or (c) reclassify the crate to a non-WASM tier |
| New tier max numbers (80 / 200) are wrong on first try | high | low | Easy to tune in code; first failure pass is informative, not destructive |

### 9.2 Decision points (user calls)

- **Tier max numbers.** Recommended 80 / 200 / 200 based on current weight distribution + reasonable headroom. Could be tighter (e.g., L0 max 60) if you want stricter enforcement; could be looser if you want less churn. Default to recommended unless you have a specific number in mind.
- **CI rework aggressiveness (§6.3).** (a) drop test matrix entirely and rely on xtask Grade + slow-tier; (b) keep matrix limited to 3–5 platform-sensitive crates. Recommend (b). Calls a small but real concern: a few crates have legitimate cross-OS behavior worth validating; for the rest, ubuntu coverage is sufficient.
- **Workspace reorganization (§7).** Defer (recommend) or include in this branch. Defer keeps PR scope bounded; including makes the directory reflect the tier system from the start.
- **Optional surgeries (§4).** Both worth doing; recommend including in this branch since CI cost is paid once.

---

## 10. Out of scope

- L1 architecture (`sim-bevy`, `cortenforge`). Not audited. Possible follow-up after this lands.
- mesh-* layer. `mesh-io` is touched (§3.4) because it's load-bearing for L0; the rest of mesh-* and the broader mesh layer's structure are out of scope.
- cf-* design layer. Not audited.
- xtask itself's structure (currently 2700+ lines in `grade.rs`). Adding ~400 LOC for new criteria; further refactoring of xtask is its own concern.
- Soft-body Phases 1–5. The whole point of this work is to make those phases sit on a clean foundation. They begin after this PR merges.

---

## 11. Why this is the right plan

Three claims that justify this plan over alternatives:

**Claim 1: The leaks are not the disease; the unenforced invariants are.** Fixing only the bevy_ecs leak (or only the dev-dep pollution, or only the CI runtime) addresses individual symptoms while leaving the soil that grew them intact. The grader extension (§5.2) is the most load-bearing part of this plan because it makes the architectural property a property, not a guideline.

**Claim 2: The tier expansion is honest, not arbitrary.** The current "L0 vs L1" binary forced a `sim-types` (22 deps) and a `sim-mjcf` (150 deps) into the same conceptual bin. The data shows two distinct populations; the architecture should reflect what's actually there. L0-io and L0-integration are not new constraints — they're labels for crates that already exist with their current weight. The L0 tier becomes meaningful by *removing* crates from it, not by adding new ones.

**Claim 3: The sequence minimizes regression risk.** Grader extensions land in warning mode first, so each refactor has a green/red signal as it happens without blocking other PRs. Optional surgeries land late so the core fix doesn't depend on them. CI rework lands at end so a single CI run validates the whole branch. The hard-gate flip happens only after all warnings clear. This isn't novel pattern for this codebase — the soft-body architecture study used multi-PR sequencing with similar discipline (α/β.1/β.1-followup/β.2/γ/δ each merged separately). The branch-batched form here is dictated by the user's CI economics constraint (38+ min per push makes per-PR sequencing prohibitively expensive); the underlying discipline (incremental verification, fail-safe rollouts, late binding of optional work) is the same.

---

## Appendix A: Self-Review Changes (2026-04-25)

This document was reviewed by the same architect after the initial draft, with explicit verification of factual claims via `rg` and `cargo tree`. The following corrections were made:

| § | What changed | Why |
|---|---|---|
| 2.1 | Added "How dep-count maximums were derived" paragraph; tightened L0 max from 80 → 60; tightened L0-io and L0-integration from 200 → 180 | Original numbers were "current weight + headroom" — admission of status quo. New numbers are principled, force real cleanup or explicit reclassification |
| 2.1 | Reclassified `sim-gpu` from L0 to L0-io | sim-gpu's purpose ("GPU-accelerated SDF collision") makes wgpu a load-bearing release dep. Calling it L0 was hand-waving; it belongs at L0-io |
| 2.1a (new) | Added "Tier-up features" section + metadata schema for `tier_up_features` | The original plan had no answer for the sim-soft gpu-probe pattern — feature that intentionally pulls heavier deps. Without an explicit mechanism, gpu-probe would fail Layer Integrity, contradicting the stated intent that it's "by design" |
| 3.1 | Acknowledged 7 example crates currently use `sim-ml-chassis features=["bevy"]` (verified via rg); risk revised from "none" to "medium"; added migration step | Original "no expected risk" was unverified. Real consumers exist and need migration |
| 3.1 | Added explicit discussion of L1 tier label for the new bevy_ecs adapter (L1 vs new L0-bevy-ecs tier) | bevy_ecs ≠ visualization. Plan B is explicit |
| 3.2 | Pivoted mechanism from "in-line MjcfModel construction" to "use sim-core::Model factories" | Verified that `Model` lives in sim-core, not sim-mjcf. Tests already need sim-core; just removing the sim-mjcf parser dep, not the Model type |
| 3.2 | Added explicit use-site enumeration with file references; revised cost from "50–200 LOC per crate" to "250–400 LOC total" with two-phase breakdown | Original cost estimate was naive; verified call sites show heavier work in fixture authoring + per-call-site migration |
| 3.4 | Acknowledged cf-design and mesh test as real consumers (verified via rg); added migration step | Original "few consumers" was unverified — actually two known consumers need explicit opt-in |
| 5.1 | Clarified scope: sim-*/mesh-*/cf-*/cortenforge* (not just sim-*); added Plan B alternatives | Scope was ambiguous; alternatives weren't acknowledged |
| 5.2 | Added critical "two-phase rollout" section: warning mode first, then hard gate | Original plan would have failed every L0 crate's grade the day the criterion landed, blocking all PRs until every surgery completed |
| 5.2 | Added gpu-probe-passes example to "why this works" list | Demonstrates that the tier-up mechanism distinguishes intentional opt-in from accidental leaks |
| 6.1 | Added "Math behind the savings estimate" paragraph | Original "10–15 min" estimate was unjustified; now derived from xtask Grade timing data |
| 8 | Reordered execution plan to interleave warning-mode rollout with surgeries; added step 12 for hard-gate flip; revised LOC and session estimates upward | Original ordering would have caused mid-branch CI failures; new ordering preserves green-build invariant during the work |
| 8 | Added explicit within-branch CI policy (no CI runs until step 13) | User's CI economics constraint wasn't operationalized in the plan |
| 8 | Pre-squash tag note softened ("recommended" not "standard") | This branch isn't a chapter-anchor study; pre-squash is recommended for audit value, not strictly required |
| 11 (Claim 3) | Removed false claim that this matches "the same pattern the soft-body architecture study used" — replaced with honest framing acknowledging the branch-batched form is novel for this codebase, dictated by CI economics | Soft-body study was multi-PR; this is one branch. Discipline is the same; pattern isn't |
| 4.2 | Added UX-cost paragraph for bench-invocation change | Original framing dismissed the friction; it's small but real |

**What did NOT change after review (and why these claims hold):**

- **Diagnosis (§1).** The "rules don't constrain what we want them to constrain" framing is the load-bearing diagnosis. Recon evidence supports it.
- **Four-tier system (§2.1, in concept).** The data shows two distinct dep-weight populations under the L0 label; the architecture catching up to reality is correct. Numbers tightened, structure preserved.
- **Grader-as-most-load-bearing-change (§5).** Without enforcement, every fix silently un-fixes itself. Review didn't move this conviction.
- **Four required surgeries (§3).** Each addresses a distinct concrete symptom. None is removable without leaving the corresponding leak unfixed.
- **CI rework (§6).** Dropping `--all-features` from the matrix is the right move once the grader enforces all-features cleanliness via Layer Integrity.

---

## Appendix B: invariants summary (for quick reference)

After this plan executes:

1. **L0 release graph contains no bevy/winit/wgpu/image/zip/criterion/sim-mjcf/mesh-io.** Enforced by grader.
2. **L0 dev graph contains no bevy/winit/wgpu/image/zip/criterion/sim-mjcf/mesh-io.** Enforced by grader.
3. **L0 builds for `wasm32-unknown-unknown` with `--no-default-features`.** Enforced by grader.
4. **Every crate has an explicit tier declaration in `Cargo.toml`.** Enforced by grader (missing = error).
5. **Tier release-dep counts respect declared maximums.** Enforced by grader.
6. **CI test matrix runs default-features only on 23 crates; specific feature combos as separate steps.** Enforced by workflow.
7. **No workspace dev-dep on `criterion`.** Per-crate `-benches` companion crates own bench-only deps.
8. **`mesh-io` zip dep is feature-gated behind `threemf`.** Default mesh-io graph is zip-free.
9. **`sim-mjcf` texture/HField support is opt-in via `textures` / `hfields` features (defaults on).** Embedded consumers can disable.
10. **STANDARDS.md describes what the grader actually enforces.** No aspirational gap.
