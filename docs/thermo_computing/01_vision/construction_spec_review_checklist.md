# Construction Spec Review — Edit Checklist

**Goal.** Apply the 2026-04-15 review findings to `physics_aware_ml_construction.md` so a fresh author can execute §10 commits 1-5 without further research.

**Branch.** `chassis-rl-split-spec-review`
**Target file.** `docs/thermo_computing/01_vision/physics_aware_ml_construction.md`
**Delete this checklist** before the chassis/rl split PR merges.

Two phases, two commits:
- **Phase A** — 4 blockers (decisions). Land as one commit.
- **Phase B** — 12 cruft items (hygiene). Land as a second commit.

---

## Phase A — Blockers

### [ ] A1. Strike §6 entirely (sim-thermostat SR extraction is obsolete)

**Why.** Ch 42 §6 sub-decision (f) intentionally kept the SR MJCF + constants duplicated in `sim/L0/opt/tests/d2c_sr_rematch*.rs`. The chassis/rl split does not revisit that decision.

**Edits:**
- Delete all of §6 (§6.1 through §6.5, ~470 lines).
- Replace with a one-paragraph pointer (see below).
- Delete §1.1 row for `sim/L0/thermostat/src/tasks.rs`.
- Delete §1.5 rows for `sim/L0/thermostat/Cargo.toml`, `src/lib.rs`, `src/tasks.rs`, `tests/stochastic_resonance_task.rs`.
- Delete §10 commit 5 row (`feat(sim-thermostat): expose stochastic_resonance as TaskConfig`). Renumber remaining rows.

**Replacement pointer (paste where §6 was):**

> ## 6. SR task packaging — deferred
>
> The construction spec originally proposed extracting `stochastic_resonance()` into `sim_thermostat::tasks`. Ch 42 §6 sub-decision (f) intentionally kept the SR MJCF + constants duplicated in the three rematch fixtures under `sim/L0/opt/tests/`. The chassis/rl split does not revisit that decision. If a future PR wants SR-as-`TaskConfig`, see `project_thermo_rl_loop_vision.md` for the motivation.

---

### [ ] A2. Strike §6.4 + §6.4.1 (`TaskConfig::from_build_fn` already exists upstream)

**Why.** `TaskConfig::from_build_fn` already exists at `sim/L0/ml-bridge/src/task.rs:153-170` with a different signature (generic `F: Fn(usize, u64) -> Result<VecEnv, EnvError> + Send + Sync + 'static`, returns `Self`, no obs_scale validation). Spec §6.4's proposed `Result<Self>` version would conflict.

**Edits (most already handled by A1 deleting §6 wholesale):**
- Confirm §6.4 and §6.4.1 are gone with A1.
- Delete item 3 from §1.2 (`task.rs gains TaskConfig::from_build_fn`).
- Delete the "Add `TaskConfig::from_build_fn` per §6.4 with the two unit tests from §6.4.1" clause from §10 commit 2's description.
- Add a one-line note after §3.3's `pub use task::{...}` block:

> Note: `TaskConfig::from_build_fn` already exists upstream (added during the ml-chassis-refactor study, PR #190). No new signature work in this split — the function moves with `task.rs` unchanged.

---

### [ ] A3. Verify no `build_vec_env(n_envs)` single-arg callsites survive

**Why.** `TaskConfig::build_vec_env` now takes `(n_envs: usize, seed: u64)` per `task.rs:120`. Spec had three wrong-arity callsites, all inside §6 (now struck).

**Edits:**
- After A1, grep the remaining spec for `build_vec_env(`. Expected: zero hits. If any survive, update arity to `build_vec_env(n, 0)` for deterministic callers.

---

### [ ] A4. Add §8.4 — sim-opt import rewrites

**Why.** sim-opt currently imports `sim_ml_bridge::` in 4 source files and 3 test fixtures. The §8 rewrite table does not list them. A fresh author running the find-and-replace verbatim would leave sim-opt broken.

**Edits:**
- Add a new §8.4 subsection after §8.3 with the table below.
- Add commit 5 to the §10 table: `refactor(sim-opt): re-home on sim-ml-chassis + sim-rl`.

**§8.4 table to paste:**

```
### 8.4 sim-opt sites

| File | Actual imports (verified 2026-04-15) | Target |
|---|---|---|
| `sim/L0/opt/Cargo.toml` | `sim-ml-bridge` dep | `sim-ml-chassis` regular + `sim-rl` dev-dep (Cem for rematch fixtures) |
| `sim/L0/opt/src/lib.rs` | doc mentions of `sim-ml-bridge` at lines 7, 27, 29, 32 | rewrite prose to `sim-ml-chassis` / `sim-rl` as appropriate |
| `sim/L0/opt/src/algorithm.rs:20` | `Algorithm, ArtifactError, CURRENT_VERSION, EpochMetrics, Policy, PolicyArtifact, TrainingBudget, TrainingCheckpoint, VecEnv, collect_episodic_rollout` | `sim_ml_chassis::` |
| `sim/L0/opt/src/algorithm.rs:375` (test mod) | `LinearPolicy, reaching_2dof` | `sim_ml_chassis::` |
| `sim/L0/opt/src/richer_sa.rs:50,492,511` | same shape as algorithm.rs | `sim_ml_chassis::` |
| `sim/L0/opt/src/parallel_tempering.rs:64,484,499` | same shape | `sim_ml_chassis::` |
| `sim/L0/opt/src/analysis.rs:17` | `Algorithm, Competition, CompetitionResult, EnvError, TaskConfig` | `sim_ml_chassis::` (all chassis types) |
| `sim/L0/opt/src/analysis.rs:634` (test mod) | mixed | chassis for primitives; `sim_rl::` for any algorithm struct |
| `sim/L0/opt/tests/d2c_sr_rematch.rs:42` | `ActionSpace, Algorithm, Cem, CemHyperparams, Competition, LinearPolicy, ObservationSpace, Policy, TaskConfig, TrainingBudget, VecEnv` | `sim_rl::` (Cem forces sim-rl route; re-exports cover the rest) |
| `sim/L0/opt/tests/d2c_sr_rematch_richer_sa.rs:61` | same shape | `sim_rl::` |
| `sim/L0/opt/tests/d2c_sr_rematch_pt.rs:52` | same shape | `sim_rl::` |

Commit 5 (`refactor(sim-opt): re-home on sim-ml-chassis + sim-rl`) lands these. Verification: `cargo test -p sim-opt --release` on non-ignored tests must stay green; `cargo xtask grade sim-opt` = A.
```

**Phase A commit gate.** After A1-A4 complete and the spec still reads top-to-bottom: commit as `docs(physics-aware-ml-construction): close 4 blockers from 2026-04-15 review`.

---

## Phase B — Cruft

Each item is a pure deletion or replacement. Work top to bottom.

- [ ] **B1.** §1.1 — strike sim-opt rows that describe files that were never created: `annealing.rs`, `tempering.rs`, `cma_es.rs`, `metropolis.rs`, `builders.rs`, `tests/d2c_rematch.rs`. (sim-opt's actual source layout is `algorithm.rs`, `richer_sa.rs`, `parallel_tempering.rs`, `analysis.rs`.)

- [ ] **B2.** §1.4 — the "commit 2's transitional `sim-ml-chassis` dep in `sim-ml-bridge/Cargo.toml`" paragraph is ~7 sentences. Cut to 2: what's added, when it's dropped.

- [ ] **B3.** §3.6 — the "Why this relocation is load-bearing" paragraph is ~5 sentences. Cut to 3: chassis-only consumer → would need sim-rl dep for a math helper → keep it in chassis.

- [ ] **B4.** §5.3 — strike the entire `SimulatedAnnealing::train` skeleton block (~120 lines of commented pseudocode + randn helper + borrow-checker commentary). Replace with:

  > **Shipped.** Simulated Annealing is implemented at `sim/L0/opt/src/algorithm.rs::Sa::train` (PR #188, `3d6f9ad8`). Richer-proposal SA with Rechenberg 1/5 is at `sim/L0/opt/src/richer_sa.rs::RicherSa::train` (PR #190, `c7aabcc7`). Parallel Tempering with K=4 geometric ladder is at `sim/L0/opt/src/parallel_tempering.rs::Pt::train` (PR #190, `8f8e006f`). The chassis/rl split does not re-derive any of this; it only rewrites sim-opt's `use sim_ml_bridge::*` lines.

- [ ] **B5.** §5.4 — strike the stub blocks for `tempering.rs`, `cma_es.rs`, `metropolis.rs`. PT is shipped; CMA-ES and MH are not scoped for this PR.

- [ ] **B6.** §5.5 — strike `sim-opt/src/builders.rs`. No live consumer; sim-opt's rematch fixtures construct algorithms inline.

- [ ] **B7.** §4.4 — strike `sim-rl/src/builders.rs`. Same reason: no live consumer. The rematch tests build algorithms with direct `::new` calls.

- [ ] **B8.** §7 — strike the entire D2c rematch code block. Replace section body with:

  > ## 7. D2c rematch — shipped
  >
  > Shipped via `sim/L0/opt/tests/d2c_sr_rematch{,_richer_sa,_pt}.rs` under PRs #188 / #190. D2c-SR question closed at the study level per `docs/studies/ml_chassis_refactor/src/53-robustness-check-prereg.md` §6.4. See `physics_aware_ml_pivot.md` "D2c rematch — shipped and closed" for the full reframing.

- [ ] **B9.** §10 — strike rows for old commits 5, 6, 7, 8. Renumber to 5 total. After A4, row 5 is the new sim-opt re-home commit.

- [ ] **B10.** §12 — strike "Parallel Tempering implementation. Stub only." (PT is fully implemented). CMA-ES, MH, DE lines stay. MLP/autograd baseline line stays. `sim-ml-bridge` backwards-compat line stays.

- [ ] **B11.** §13 — update memory update list: rename `project_sim_ml_bridge.md` → `project_sim_ml_split.md`, flip `project_sim_ml_pivot.md`'s "chassis/rl split is pending" line to "shipped", sync `MEMORY.md`'s Codebase Structure line. Remove any reference to `project_thermo_rl_loop_vision.md` edits (already recorded).

- [ ] **B12.** §8.3 — the `ML_COMPETITION_SPEC.md` historical-banner proposal is fine as-is; just verify the banner text still reads clean after A1 (should, it doesn't reference §6).

**Phase B commit gate.** After B1-B12: commit as `docs(physics-aware-ml-construction): cruft pass post-blocker edits`.

---

## Decisions log

Record anything surprising that comes up during editing. Future-you will thank you.

- _(empty — add entries as we edit)_

---

## Done criteria

- [ ] Phase A committed
- [ ] Phase B committed
- [ ] Spec reads clean top-to-bottom (one sitting, no "wait, this contradicts §X" moments)
- [ ] `grep -n 'build_vec_env(' docs/thermo_computing/01_vision/physics_aware_ml_construction.md` returns zero single-arg hits
- [ ] `grep -n 'sim_ml_bridge\|sim-ml-bridge' docs/thermo_computing/01_vision/physics_aware_ml_construction.md` returns only the explicitly-historical mentions we want kept
- [ ] This checklist file deleted in the chassis/rl split execution PR
