# Repo Cleanup — v1.0 Shape-Up

**Date:** 2026-03-19
**Branch:** chore/workspace-trim-tier1
**Status:** Complete

## Motivation

CortenForge has a clear product: design bio-inspired mechanisms → simulate
them → mesh them → validate for 3D printing. But the repo doesn't
communicate this at a glance. The root directory has 14 markdown files, 9
directories (7 containing crates), and 3 entire domains (`ml/`, `routing/`,
`sensor/`) with zero consumers. A newcomer has to read documentation to
understand what the project does; the directory structure should tell them.

This spec cleans the repo so its shape matches its purpose.

## Goal State

After this work, the root directory looks like this:

```
Cargo.toml
Cargo.lock
claude.md
CONTRIBUTING.md
LICENSE
README.md
clippy.toml
deny.toml
rustfmt.toml

design/           ← design kernel (3 crates)
docs/             ← all project documentation
examples/         ← 6 working showcases
mesh/             ← mesh pipeline (10 crates + umbrella)
sim/              ← physics simulation (7 crates)
xtask/            ← build tooling
```

Someone opens the repo and sees: **design + mesh + sim + examples**. That's
the product. Everything else is supporting infrastructure.

## Changes

### 1. Remove Tier 2 Orphan Domains

Delete `ml/`, `routing/`, `sensor/` — 9 crates, zero consumers.

| Directory | Crates | Internal Deps Only |
|-----------|--------|--------------------|
| `ml/` | ml-types, ml-models, ml-dataset, ml-training | ml-types ↔ sensor-types |
| `routing/` | route-types, route-pathfind, route-optimize | route-types → pathfind → optimize |
| `sensor/` | sensor-types, sensor-fusion | sensor-types → fusion |

**Verification (stress-tested 2026-03-19):** No crate outside these 9
depends on any of them. Checked via cargo metadata dependency graph, `use`
import grep across all .rs files, and Cargo.toml inspection. The only hits
outside these directories are doc comments in `cf-geometry` mentioning
`route-types` as a former duplicate — no code dependency.

Git history preserves everything.

**Root `Cargo.toml` edits:**
- Remove 9 crates from `[workspace] members`
- Remove 9 path entries from `[workspace.dependencies]`
- Remove 7 workspace deps that become orphaned:

| Dependency | Version | Only Used By |
|------------|---------|--------------|
| `burn` | 0.16 | ml-models, ml-training |
| `burn-ndarray` | 0.16 | ml-models (dev), ml-training (dev) |
| `burn-wgpu` | 0.16 | **Already orphaned** — declared but unused |
| `pathfinding` | 4.0 | route-pathfind |
| `glam` | 0.30 | sensor-fusion |
| `tracing-subscriber` | 0.3 | **Already orphaned** — declared but unused |
| `cortenforge` | path | **Already orphaned** — path doesn't exist |

**CI workflow edits:**

| File | Lines | What to Remove |
|------|-------|----------------|
| `quality-gate.yml` | 149–150 | `sensor-types`, `ml-types` from WASM_CRATES |
| `quality-gate.yml` | 395–400 | `sensor-types`, `sensor-fusion`, `ml-types`, `ml-models`, `ml-dataset`, `ml-training` from LAYER0_CRATES |
| `scheduled.yml` | 82–83 | `sensor-types`, `ml-types` from mutation testing matrix |

**Crate count:** 36 → 27

### 2. Rename `crates/` → `design/`

`crates/` currently holds `cf-design`, `cf-geometry`, `cf-spatial` — all
design kernel crates. The name `crates/` is generic and says nothing. Rename
to `design/` so it mirrors `mesh/` and `sim/`.

```
design/
  cf-design/
  cf-geometry/
  cf-spatial/
```

**Cargo.toml edits (4 files, 9 occurrences):**

| File | Changes |
|------|---------|
| `Cargo.toml` (root) | 3 member paths + 3 `[workspace.dependencies]` path entries |
| `sim/L1/bevy/Cargo.toml` | `cf-geometry` path: `../../../crates/cf-geometry` → `../../../design/cf-geometry` |
| `mesh/mesh-types/Cargo.toml` | `cf-geometry` path: `../../crates/cf-geometry` → `../../design/cf-geometry` |
| `mesh/mesh-repair/Cargo.toml` | `cf-geometry` path: `../../crates/cf-geometry` → `../../design/cf-geometry` |

**Markdown edits (6 files, 13 occurrences):**

| File | Occurrences |
|------|-------------|
| `CF_DESIGN_SPEC.md` | 5 (crate path references) |
| `CF_GEOMETRY_SPEC.md` | 2 |
| `WORKSPACE_TRIM_SPEC.md` | 1 (historical — moves to archive) |
| `MIGRATION_CHECKLIST.md` | 2 (historical — moves to archive) |
| `REPO_CLEANUP_SPEC.md` | 2 (this spec — update at completion) |
| `sim/docs/todo/spec_fleshouts/mesh_collision_fixes/SPEC.md` | 1 |

**No .rs or .yml files reference `crates/` paths.**

**xtask hardcoded path lists (runtime-breaking):**

| File | Line | Current Value | Fix |
|------|------|---------------|-----|
| `xtask/src/complete.rs` | 117 | `find_crate_path` searches `["crates", "mesh", "geometry", "routing", "ml", "vision", "sim"]` | Change `"crates"` → `"design"`, remove `"routing"`, `"ml"` |
| `xtask/src/grade.rs` | 289 | Same hardcoded search list | Same fix |

Without this fix, `cargo xtask grade cf-design` and `cargo xtask complete
cf-design` will fail to find the crate after the rename.

### 3. Move Documentation to `docs/`

Move 10 markdown files from root into `docs/`, with an `archive/`
subdirectory for historical documents.

**Stays in root (standard convention):**
- `README.md`
- `CONTRIBUTING.md`
- `LICENSE`
- `CLAUDE.md` (tooling config)

**Moves to `docs/`:**

| File | Destination | Cross-references to Update |
|------|-------------|----------------------------|
| `VISION.md` | `docs/VISION.md` | README.md (1), WORKSPACE_TRIM_SPEC.md (1, archive) |
| `STANDARDS.md` | `docs/STANDARDS.md` | **~40 references** — see detail below |
| `INFRASTRUCTURE.md` | `docs/INFRASTRUCTURE.md` | VISION.md (2), Cargo.toml (1), MIGRATION_CHECKLIST.md (2, archive), 3 CI workflows (4), xtask (4), .github/CODEOWNERS (1) |
| `SECURITY.md` | `docs/SECURITY.md` | MIGRATION_CHECKLIST.md (1, archive), .github/CODEOWNERS (1) |
| `FUTURE.md` | `docs/FUTURE.md` | WORKSPACE_TRIM_SPEC.md (1, archive) |
| `UPSTREAM.md` | `docs/UPSTREAM.md` | quality-gate.yml (2) |
| `CF_DESIGN_SPEC.md` | `docs/CF_DESIGN_SPEC.md` | README.md (2), VISION.md (2), sim/docs/ (4) |
| `CF_GEOMETRY_SPEC.md` | `docs/CF_GEOMETRY_SPEC.md` | README.md (1), WORKSPACE_TRIM_SPEC.md (2, archive) |

**Moves to `docs/archive/`:**

| File | Cross-references to Update |
|------|----------------------------|
| `MIGRATION_CHECKLIST.md` | WORKSPACE_TRIM_SPEC.md (1, also archive) |
| `COMPLETION_LOG.md` | CONTRIBUTING.md (2), MIGRATION_CHECKLIST.md (1, archive), **xtask/src/complete.rs:195** (hardcoded path — will break) |
| `WORKSPACE_TRIM_SPEC.md` | sim/docs/MUJOCO_GAP_ANALYSIS.md (2), sim/docs/ARCHITECTURE.md (1) |

**STANDARDS.md cross-reference detail (highest-touch file, ~38 total):**

| Category | Files | Count |
|----------|-------|-------|
| Surviving .rs doc comments | cf-spatial, mesh-io, mesh-types | 3 |
| Deleted .rs doc comments (ml/sensor) | 6 crates (deleted in step 1) | 6 |
| COMPLETION.md files in mesh crates | 7 files | 7 |
| templates/COMPLETION.md.template | 1 (deleted in step 4) | 1 |
| xtask source code | complete.rs (3), grade.rs (2), check.rs (1), main.rs (1) | 7 |
| CI workflows | quality-gate.yml (1), release.yml (1) | 2 |
| .github/CODEOWNERS | 1 | 1 |
| sim/docs/ | SIM_BEVY_IMPLEMENTATION_PLAN.md (1), STRUCTURAL_REFACTOR_RUBRIC.md (2), future_work_3.md (1), future_work_4.md (1) | 5 |
| Other docs | CONTRIBUTING.md (6), README.md (2), VISION.md (1), COMPLETION_LOG.md (1), MIGRATION_CHECKLIST.md (2) | 12 |

**Runtime-breaking changes in xtask:**

| File | Line | Issue |
|------|------|-------|
| `complete.rs` | 195 | `let log_path = "COMPLETION_LOG.md"` — must update to `"docs/archive/COMPLETION_LOG.md"` |
| `complete.rs` | 147 | Template generates `[STANDARDS.md](../../STANDARDS.md)` in new COMPLETION.md files — relative path will be wrong after move |
| `complete.rs` | 219–225 | Template for new COMPLETION_LOG.md generates `[STANDARDS.md](./STANDARDS.md)` — stale after move |

### 4. Delete `scripts/`, `templates/`, `requirements/`

| Directory | Contents | References Found | Disposition |
|-----------|----------|------------------|-------------|
| `scripts/` | `local-quality-check.sh` (230 lines) | CONTRIBUTING.md (11 lines of usage examples), sim/docs/todo LEGACY_CRATE_CLEANUP.md (1) | **Delete.** References 20+ deleted crates. Subsumed by `cargo xtask check`. |
| `templates/` | `COMPLETION.md.template` | None outside itself | **Delete.** Not used by xtask code. `xtask/src/complete.rs` generates the template inline. |
| `requirements/` | `REQ-MESH-001.yaml`, `schema.yaml`, `README.md` | INFRASTRUCTURE.md (4 references) | **Delete.** One requirement file, no code consumers. |

**CONTRIBUTING.md rewrites:**
- Lines 44–58: Replace 11 lines of `./scripts/local-quality-check.sh`
  usage examples with `cargo xtask check` guidance.
- Lines 178–191: "Project Structure" section lists `routing/`, `ml/`,
  `sensor/`, and `cortenforge/` — all deleted or nonexistent. Rewrite to
  reflect the new `design/`, `mesh/`, `sim/`, `examples/` layout.

## What This Does NOT Touch

- **Mesh crates** — 10 + umbrella, all have consumers, all tested
- **Sim crates** — 7 crates, active pipeline
- **Design kernel crates** — 3 crates, renamed directory only
- **Examples** — 6 showcases, unchanged
- **xtask** — build tooling (update hardcoded paths in complete.rs + grade.rs)

## Session Plan

### Session 1: Tier 2 Removal + Directory Restructure ✅

**Completed:** 2026-03-19 — commit `e51322b`

All Cargo.toml + file moves that must land together for the workspace to
compile.

1. ✅ Delete `ml/`, `routing/`, `sensor/` (9 crate directories)
2. ✅ Rename `crates/` → `design/`
3. ✅ Edit root `Cargo.toml`:
   - Remove 9 members + 9 path deps
   - Remove 7 orphaned workspace deps (burn, burn-ndarray, burn-wgpu,
     pathfinding, glam, tracing-subscriber, cortenforge)
   - Update 3 member paths + 3 dep paths (`crates/` → `design/`)
4. ✅ Edit 3 crate Cargo.toml files for cf-geometry path change:
   - `sim/L1/bevy/Cargo.toml`
   - `mesh/mesh-types/Cargo.toml`
   - `mesh/mesh-repair/Cargo.toml`
5. ✅ Update xtask hardcoded path lists:
   - `xtask/src/complete.rs:117` — `"crates"` → `"design"`, remove `"routing"`, `"ml"`
   - `xtask/src/grade.rs:289` — same fix
6. ✅ Delete `scripts/`, `templates/`, `requirements/`
7. ✅ Remove `.github/CODEOWNERS` Tier 2 entries:
   - `/sensor/sensor-types/` and `/ml/ml-types/` ownership lines
8. ✅ Delete `MUJOCO_LOG.TXT` (MuJoCo test output file, not project artifact)
9. ✅ Verify:
   ```
   cargo build -p cf-design -p cf-geometry -p cf-spatial -p mesh -p sim-core
   cargo test -p cf-design -p cf-geometry -p cf-spatial
   ```
   Build clean. 1283 tests passed (cf-design 698, cf-geometry 441, cf-spatial 144).

### Session 2: Documentation Restructure ✅

**Completed:** 2026-03-19 — commits `ef21de0`, `7ddbc18`, `0c46724`

Move markdown files, fix all cross-references. Three commits:

**Commit A** (`ef21de0`) — Moves only (`git mv`):
1. ✅ Create `docs/` and `docs/archive/`
2. ✅ `git mv` 8 files to `docs/`, 3 to `docs/archive/`

**Commit B** (`7ddbc18`) — Reference fixups (33 files, ~50 edits):
- ✅ `README.md` — substantial rewrite: remove Routing/ML/Sensor sections,
  update architecture diagram, crate count 28→21, `crates/`→`design/`,
  update all doc links
- ✅ `CONTRIBUTING.md` — update STANDARDS.md links (6), COMPLETION_LOG.md
  links (2), replace `scripts/` usage with xtask, rewrite project structure
- ✅ `.github/CODEOWNERS` — update 3 doc path entries
- ✅ `.github/workflows/` — update doc refs in 3 files
- ✅ `xtask/src/complete.rs` — STANDARDS.md refs (3), COMPLETION_LOG.md
  path (runtime-breaking), COMPLETION.md + COMPLETION_LOG template paths
- ✅ `xtask/src/grade.rs` (2), `check.rs` (1), `main.rs` (1) — STANDARDS.md
- ✅ `xtask/build.rs` (2), `setup.rs` (2) — INFRASTRUCTURE.md
- ✅ `.rs` doc comments — cf-spatial, mesh-io, mesh-types (3 files)
- ✅ 7 mesh `COMPLETION.md` files — STANDARDS.md relative path
- ✅ `Cargo.toml` (root) — INFRASTRUCTURE.md comment
- ✅ `sim/docs/` — CF_DESIGN_SPEC.md (5), WORKSPACE_TRIM_SPEC.md (3),
  STANDARDS.md (1 link in SIM_BEVY_IMPLEMENTATION_PLAN.md)
- ✅ `docs/INFRASTRUCTURE.md` — remove 4 stale `requirements/` references
- ✅ `LEGACY_CRATE_CLEANUP.md` — remove `scripts/` reference
- ✅ `design/cf-geometry/` — aabb.rs, mesh.rs, sphere.rs: remove route-types
  and route-pathfind refs

**Commit C** (`0c46724`) — Follow-up: remaining stale refs in spec docs:
- ✅ `docs/CF_DESIGN_SPEC.md` — 5× `crates/cf-design/` → `design/cf-design/`
- ✅ `docs/CF_GEOMETRY_SPEC.md` — 2× `crates/cf-geometry/` → `design/cf-geometry/`
- ✅ `sim/docs/` — STRUCTURAL_REFACTOR_RUBRIC.md (2), future_work_3.md (1),
  future_work_4.md (1): STANDARDS.md text mentions updated

**Verified:** grep for each moved filename confirms zero stale references
in active (non-archive) files. xtask builds clean, clippy passes on all
5 affected crates.

### Session 3: CI + Final Verification ✅

**Completed:** 2026-03-19 — commits `70e5c5c`, (close-out)

**Commit A** (`70e5c5c`) — CI cleanup + last stale ref:
1. ✅ `quality-gate.yml` — removed `sensor-types`, `ml-types` from WASM_CRATES;
   removed `sensor-types`, `sensor-fusion`, `ml-types`, `ml-models`,
   `ml-dataset`, `ml-training` from LAYER0_CRATES
2. ✅ `scheduled.yml` — removed `sensor-types`, `ml-types` from mutation
   testing matrix
3. ✅ Doc path comments already correct (fixed in Session 2)
4. ✅ Fixed last stale `crates/` → `design/` ref in
   `sim/docs/todo/spec_fleshouts/mesh_collision_fixes/SPEC.md:176`

**Verification:**
5. ✅ Build: `mesh`, `cf-design`, `cf-geometry`, `cf-spatial`, `sim-core` — clean
6. ✅ Build: all 6 example crates — clean
7. ✅ Test: 9 mesh crates — all pass
8. ✅ Test: 3 design crates (1283 tests) — all pass
9. ✅ Clippy: `mesh`, `cf-design` — zero warnings
10. ✅ Dangling reference sweep: zero stale hits in active files

**Commit B** — Close-out:
11. ✅ `git mv REPO_CLEANUP_SPEC.md docs/archive/REPO_CLEANUP_SPEC.md`
12. ✅ Updated spec: marked complete, recorded final state

## Stress Test Results (2026-03-19)

**Round 1** — verified core assumptions. **Round 2** — found 18
discrepancies (2 high, 4 medium, 12 minor); all resolved in this revision.

All assumptions verified against the codebase:

- **Tier 2 isolation**: CONFIRMED. Zero cargo deps, zero `use` imports,
  zero example references outside ml/routing/sensor. Only hits are doc
  comments in cf-geometry mentioning route-types as a former duplicate.
- **Orphaned workspace deps**: 4 become orphaned by this removal (burn,
  burn-ndarray, pathfinding, glam). 3 are already orphaned (burn-wgpu,
  tracing-subscriber, cortenforge).
- **`crates/` path references**: 4 Cargo.toml files (build-critical) +
  6 markdown files (documentation). Zero .rs files. Zero CI files.
- **xtask hardcoded path lists**: `complete.rs:117` and `grade.rs:289`
  search `["crates", ...]` — **runtime-breaking** after rename. Must
  change `"crates"` → `"design"` and remove `"routing"`, `"ml"`.
- **STANDARDS.md cross-refs**: ~38 total. 6 in crates being deleted,
  7 in xtask code, 7 in COMPLETION.md files, 3 in surviving .rs doc
  comments, 5 in sim/docs/, remainder in other docs/CI.
- **Runtime-breaking paths**: `complete.rs:195` (COMPLETION_LOG.md path),
  `complete.rs:147` (COMPLETION.md template STANDARDS.md path),
  `complete.rs:219–225` (COMPLETION_LOG template STANDARDS.md path).
- **README.md scope**: Needs domain section removal (routing/ml/sensor),
  architecture diagram update, crate count update, and layout references
  — not just link updates.
- **CONTRIBUTING.md**: 11 lines of `local-quality-check.sh` examples +
  project structure section (lines 178–191) lists deleted directories.
- **CI workflows**: 10 Tier 2 crate entries to remove across 2 files.
  No `crates/` path references in CI. `release.yml` needs no changes.
- **CODEOWNERS**: 2 Tier 2 ownership lines (`/sensor/sensor-types/`,
  `/ml/ml-types/`) + 3 doc path entries to update.
- **Root artifacts**: `MUJOCO_LOG.TXT` is a test output file, not a
  project artifact — delete in Session 1.
- **Cargo.toml exclude**: `exclude = ["docs"]` — `templates` entry removed
  in Session 1 (directory deleted). `docs` exclude is intentional for the
  new dir.

## Final State

```
27 crates across 3 domains + examples + tooling
Root: 4 markdown files, 6 directories
```

| Domain | Crates | Directory |
|--------|--------|-----------|
| Design kernel | cf-design, cf-geometry, cf-spatial | `design/` |
| Mesh pipeline | mesh (umbrella), mesh-types, mesh-io, mesh-repair, mesh-sdf, mesh-offset, mesh-shell, mesh-measure, mesh-printability, mesh-lattice | `mesh/` |
| Physics sim | sim-types, sim-simd, sim-core, sim-mjcf, sim-urdf, sim-conformance-tests, sim-bevy | `sim/` |
| Examples | 6 example crates | `examples/` |
| Tooling | xtask | `xtask/` |
