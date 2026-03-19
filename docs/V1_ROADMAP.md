# V1.0 Roadmap

> What stands between 0.7.0 and a clean, shippable 1.0.

**Goal:** Every item here is either a doc fix, a test gap, or a housekeeping
task. The code architecture and scope are already v1.0-quality. This list is
about making the packaging match.

---

## 1. Refresh VISION.md

VISION.md is heavily stale after the repo cleanup. It references removed
domains, non-existent crates, and wrong counts throughout.

**Stale references (exhaustive):**
- Line 56: `sensor_fusion::StreamSynchronizer` and `route_pathfind::VoxelAStar`
- Lines 79–81: Layer 0 diagram lists `sensor/*`, `ml/*`, `routing/*`
- Lines 96–97: Layer 1 diagram lists `CfSensorPlugin`, `CfMlPlugin`
- Lines 113–120: Sim diagram lists 6 non-existent crates (`sim-constraint`,
  `sim-sensor`, `sim-deformable`, `sim-muscle`, `sim-tendon`, `sim-physics`)
- Lines 144–146: Domain Coverage table lists Sensors, ML Pipeline, Routing
- Lines 158–165: Domain Roadmap claims "52+ crates COMPLETE" with counts for
  removed domains; claims "27 mesh crates" (actual: 10), "14 sim crates" (actual: 7)
- Line 150: Soft Physics row references `sim-deformable`, `sim-muscle`
- Lines 205–207: Behemoth Comparison references `ml-types`, `ml-dataset`, etc.
- Line 333: Milestone 2 references `sensor-types`
- Lines 241–266: Target Applications reference sensor/routing capabilities
- Line 391: "Last updated: 2026-03-15" (cleanup was 2026-03-19)

**Work:**
- Rewrite to reflect the current 3-pillar structure (design, mesh, sim)
- Update all diagrams, tables, counts, milestones
- Remove all references to removed domains and non-existent crates
- Keep the north-star tone — just make it match reality

**Size:** ~1–2 hours (more extensive than originally estimated)

---

## 2. Clean up sim/docs/todo/

18 `future_work_*.md` files + 14 spec fleshout directories + 1 loose spec file
(`DT16_DT90_SPEC.md`) + `POST_V1_ROADMAP.md` + `index.md`. A newcomer opening
this directory sees planning artifacts that signal "in progress," not "finished."

**Work:**
- Consolidate the 18 `future_work_*.md` files into `POST_V1_ROADMAP.md` (or a
  single `FUTURE_WORK.md`). One file, organized by theme, not chronologically
- Move completed spec fleshouts (phases 3–6) into `sim/docs/todo/archived/`
- Keep active/future phases (7–13) as-is
- File the loose `DT16_DT90_SPEC.md` into a directory or archived/
- Update `index.md` to reflect the new structure

**Size:** ~2 hours

---

## 3. Sweep stale references across all docs

The cleanup removed 9 crates but stale references survive in many active files
beyond VISION.md. This is a codebase-wide sweep.

**Files with confirmed stale references:**

| File | Stale content |
|------|---------------|
| `CLAUDE.md` lines 20–21 | `cargo test` commands for ml-types, ml-dataset, ml-training, route-types, route-pathfind, route-optimize |
| `docs/FUTURE.md` line 47 | `sensor-fusion transforms` |
| `docs/INFRASTRUCTURE.md` line 344 | `sensor-fusion transforms` |
| `docs/CF_GEOMETRY_SPEC.md` (~15 occurrences) | Migration plans referencing `route-types`, `route-pathfind`, `route-optimize` |
| `docs/archive/COMPLETION_LOG.md` | Lists 52 crates; 11 no longer exist |
| `.github/settings.yml` line 19 | `sensor-fusion` |
| `Cargo.toml` lines 153–154 | TODO comments for `avian3d` / `bevy_egui` Bevy 0.18 compat |

**Work:**
- Remove ml/route/sensor test commands from CLAUDE.md
- Remove `sensor-fusion` from FUTURE.md and INFRASTRUCTURE.md
- Update or archive CF_GEOMETRY_SPEC.md route-* references (either strip the
  migration plans for deleted crates, or add a header noting they were removed)
- Add a cleanup-delta note at top of COMPLETION_LOG.md
- Remove stale entry from .github/settings.yml
- Resolve or remove Cargo.toml TODO comments

**Size:** ~1 hour

---

## 4. Clean up stale source comments

Three sim source files have comments referencing removed crate names. The code
is correct — only the comments are stale.

| File | Stale comment |
|------|---------------|
| `sim/L0/tests/integration/flex_unified.rs` | References `sim-deformable` crate |
| `sim/L0/core/src/forward/actuation.rs` | "Inlined from sim-muscle" |
| `sim/L0/tests/integration/mjcf_sensors.rs` | References `sim-sensor` crate |

**Work:**
- Update comments to reference current location (e.g., "muscle model inlined
  in sim-core" instead of "inlined from sim-muscle")
- Quick grep for any other `sim-constraint`, `sim-deformable`, `sim-muscle`,
  `sim-tendon`, `sim-physics`, `sim-sensor` references in .rs files

**Size:** ~15 min

---

## 5. cf-design test hardening

cf-design has ~23K lines across 31 files. It has inline `#[cfg(test)]` modules
in many files, which is a start. But for a crate that is the differentiating
technology, test coverage should match the sim domain's standard.

**Work:**
- Audit current test coverage (`cargo tarpaulin -p cf-design`)
- Identify critical paths with no coverage: dual contouring, adaptive DC,
  mechanism builder, MJCF export, STL export, parameter gradients
- Add integration tests for the design-to-mesh pipeline (solid → mesh → validate)
- Add integration tests for mechanism-to-MJCF pipeline
- Target: ≥75% line coverage (matching the A-grade standard)

**Size:** ~1–2 sessions

---

## 6. README polish

The README says "21 library crates" — actual count is 20 (10 mesh + 3 design +
7 sim). cf-design is listed under "What's next" but Phases 1–4 are implemented.

**Work:**
- Fix crate count to 20
- Move cf-design from "What's next" to "What's built" (Phases 1–4 are done)
- Update cf-design description to reflect current state vs. planned Phase 5

**Size:** ~15 min

---

## 7. Version bump to 1.0.0

Once items 1–6 are done, bump the workspace version.

**Work:**
- Update `version` in root Cargo.toml from `0.7.0` to `1.0.0`
- Update any version references in docs
- Tag the release

**Size:** ~5 min (after everything else is done)

---

## Non-goals for 1.0

These are good ideas but explicitly out of scope for the 1.0 cut:

- **cf-design Phase 5 (Differentiable Design)** — new feature work, not a 1.0
  prerequisite. Ship 1.0 with Phases 1–4 stable, Phase 5 comes in 1.1
- **PyO3 bindings** — future work
- **WASM examples** — nice to have, not blocking
- **Consolidating sim/docs/ spec fleshouts into fewer files** — moving completed
  ones to archived/ is sufficient
- **coverage-diff tooling** (docs/FUTURE.md) — enhancement, not 1.0
- **B-Rep upgrade path** — explicitly future
