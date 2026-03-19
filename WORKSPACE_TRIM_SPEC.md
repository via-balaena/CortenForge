# Workspace Trim — Tier 1

**Date:** 2026-03-19
**Branch:** `chore/workspace-trim-tier1`
**Status:** Session 2 complete (a43267c) — Session 3 next

## Motivation

CortenForge committed to an SDF/implicit-surface-first architecture
(CF_DESIGN_SPEC.md §3, decision 2026-03-15). A workspace audit found that
19 of 59 crates are orphaned — implemented but not depended on by any other
crate or example. They compile in isolation but aren't integration-tested
against the actual pipeline, meaning they bit-rot silently and add cognitive
load without delivering capability.

Git history preserves everything. When a capability is needed, it should be
rebuilt against the architecture as it exists at that time.

## Scope

**Tier 1 (this PR):** Remove orphaned crates that have zero consumers.
**Tier 2 (deferred):** ML, sensor, and routing domains remain. They're
self-contained, don't pollute the core pipeline, and represent distinct
future capabilities.

## Crates to Remove (19)

### Orphaned Mesh Crates (16)

These mesh crates are not depended on by any other crate or example. Their
only consumer is the `mesh` umbrella re-export, which is not sufficient
justification to keep them.

| Crate | What It Did |
|-------|-------------|
| `mesh-boolean` | Triangle-level CSG boolean ops |
| `mesh-geodesic` | Surface distance (Dijkstra) |
| `mesh-thickness` | Wall thickness analysis (ray casting) |
| `mesh-transform` | PCA, RANSAC plane fitting |
| `mesh-slice` | Layer slicing / print preview |
| `mesh-scan` | Point cloud processing, denoising |
| `mesh-template` | Parametric template fitting |
| `mesh-remesh` | Isotropic remeshing |
| `mesh-decimate` | Quadric error simplification |
| `mesh-subdivide` | Loop / midpoint subdivision |
| `mesh-from-curves` | Tube/sweep/loft from curves |
| `mesh-zones` | Zone assignment / segmentation |
| `mesh-region` | Region selection / material zones |
| `mesh-assembly` | Multi-part assembly management |
| `mesh-morph` | RBF and FFD deformation |
| `mesh-registration` | ICP / landmark alignment |

### GPU Crates (2)

| Crate | What It Did |
|-------|-------------|
| `mesh-gpu` | GPU-accelerated SDF via WGPU |
| `sim-gpu` | GPU-accelerated batched physics via WGPU |

### Geometry (1)

| Crate | What It Did |
|-------|-------------|
| `curve-types` | Bezier, NURBS, B-spline, Arc, Helix |

**Why curve-types:** cf-design already has its own Catmull-Rom spline
implementation in `PipeSpline`. The curve-types crate duplicates this
capability and has zero consumers.

## Surviving Mesh Crates (10 + umbrella)

| Crate | Role | Consumers |
|-------|------|-----------|
| `mesh-types` | Foundation types | All mesh crates, sim-mjcf, examples |
| `mesh-io` | STL/OBJ/PLY/3MF/STEP I/O | 5 examples, sim-mjcf |
| `mesh-repair` | Validation & repair | mesh-shell, mesh-pipeline example |
| `mesh-sdf` | Mesh-to-SDF conversion | mesh-offset |
| `mesh-offset` | SDF-based offset | mesh-shell |
| `mesh-shell` | Shell generation for printing | mesh-pipeline example |
| `mesh-measure` | Dimensions, volume, area | mesh-pipeline example |
| `mesh-printability` | Print validation | mesh-pipeline example |
| `mesh-lattice` | Infill generation (TPMS, struts) | mesh-pipeline example |
| `mesh` | Umbrella re-export | — |

## Workspace Dependencies to Clean Up

| Dependency | Action | Reason |
|------------|--------|--------|
| `kiddo` | Remove | Only used by mesh-scan + mesh-registration |
| `smallvec` | Remove | Only used by mesh-boolean |
| `indexmap` | Remove | No workspace consumers found |
| `thiserror` in cf-geometry | Remove from crate | Declared but unused |

## Stress Test Results (2026-03-19)

All assumptions verified against the codebase:

- **Zero surviving crates** depend on any removed crate
- **Zero examples** reference any removed crate
- **kiddo** — confirmed only mesh-scan + mesh-registration
- **smallvec** — confirmed only mesh-boolean
- **indexmap** — confirmed zero consumers in entire workspace
- **thiserror in cf-geometry** — confirmed: declared in Cargo.toml, zero
  usage in source (no `#[derive(Error)]`, no `#[error]`, no `use thiserror`)
- **58+ references** in 16 non-Cargo files needed updating (see Session 2)

## Session Plan

### Session 1: Core Removal (atomic — must be one commit)

All Cargo.toml + code changes that must land together for the workspace to
compile.

**Delete 19 crate directories + 1 empty parent:**
```
mesh/mesh-boolean/       mesh/mesh-geodesic/      mesh/mesh-thickness/
mesh/mesh-transform/     mesh/mesh-slice/         mesh/mesh-scan/
mesh/mesh-template/      mesh/mesh-remesh/        mesh/mesh-decimate/
mesh/mesh-subdivide/     mesh/mesh-from-curves/   mesh/mesh-zones/
mesh/mesh-region/        mesh/mesh-assembly/      mesh/mesh-morph/
mesh/mesh-registration/  mesh/mesh-gpu/           geometry/curve-types/
sim/L0/gpu/
```

**Delete `geometry/` entirely** — `curve-types` is its only child. The
commented-out `lumen-geometry` member does not exist on disk.

**Edit root `Cargo.toml`:**
- Remove 19 crates from `[workspace] members`
- Remove 19 path entries from `[workspace.dependencies]`
- Remove `kiddo`, `smallvec`, `indexmap` from `[workspace.dependencies]`
- Remove commented-out `geometry/lumen-geometry` line (directory doesn't exist)

**Edit `mesh/mesh/Cargo.toml`:**
- Remove 16 mesh dependencies + `gpu` feature/optional dep

**Edit `mesh/mesh/src/lib.rs`:**
- Remove 16 re-exports (`pub use mesh_* as *`)
- Remove corresponding doc sections (module docs, Quick Start if needed)
- Remove `mesh_transform::Transform3D` from prelude
- Inline test `test_module_reexports` only uses `types`, `repair`, `shell` — safe

**Edit `mesh/mesh/tests/api_regression.rs`** (major — 532 lines, ~60% removed):
- **Tier 1** (foundation): Keep — only uses `mesh-types` ✓
- **Tier 2** (core ops): Keep repair + io tests. **Remove** `decimate_params_presets`
  and `decimate_mesh_operation` (mesh-decimate)
- **Tier 3** (advanced): Keep `offset_mesh_operation`. **Remove**
  `boolean_config_presets`, `boolean_operations_return_result`,
  `multi_boolean_operations` (mesh-boolean), `slice_mesh_operation` (mesh-slice)
- **Tier 4** (analysis): Keep all — uses `printability`, `sdf`, `repair` ✓
- **Tier 5** (specialized): Keep `lattice_structure`. **Remove**
  `point_cloud_operations` (mesh-scan), `morph_params` (mesh-morph),
  `remesh_operation` (mesh-remesh), `registration_icp` (mesh-registration)
- **Error handling**: Keep `printability_empty_mesh_error`. **Remove**
  `boolean_empty_mesh_error` (mesh-boolean)
- Update tier doc comments to reflect surviving modules

**Edit `crates/cf-geometry/Cargo.toml`:**
- Remove unused `thiserror` dependency

**Verify:**
```
cargo build -p mesh -p cf-geometry
cargo test -p mesh
cargo test -p mesh-io -p mesh-repair -p mesh-sdf -p mesh-offset \
  -p mesh-shell -p mesh-measure -p mesh-printability -p mesh-lattice
cargo build -p example-hello-solid -p example-bio-shapes \
  -p example-tendon-finger -p example-mesh-pipeline
```

---

### Session 2: CI, Tooling, and Documentation ✅ (a43267c)

Updated all non-Cargo references to deleted crates. The pre-session audit
found 58 references in 9 files; the actual sweep found additional references
in 7 more files (16 total).

**Files edited (16):**

| File | Changes |
|------|---------|
| `.github/workflows/quality-gate.yml` | Removed 3 WASM_CRATES + 16 LAYER0_CRATES entries |
| `.github/workflows/scheduled.yml` | Removed mesh-boolean from mutation testing matrix |
| `xtask/src/setup.rs` | Updated example commit message |
| `xtask/build.rs` | Updated example commit message (same pattern) |
| `README.md` | Updated counts (28 crates/6 domains), architecture table, mesh/sim listings, removed Geometry Domain section |
| `INFRASTRUCTURE.md` | Updated 2 mesh-boolean references (example commit, formal verification) |
| `CONTRIBUTING.md` | Removed curve-types, sim-gpu exception, mesh-geodesic example, geometry/ directory |
| `STANDARDS.md` | Removed curve-types from Layer 0 list |
| `CF_GEOMETRY_SPEC.md` | Updated 8 locations: Aabb counts, dependency profile, session exits, curve-types removal notes |
| `FUTURE.md` | Removed mesh-boolean from formal verification targets |
| `VISION.md` | Removed sim-gpu from crate listing, roadmap, milestone |
| `sim/docs/ARCHITECTURE.md` | Replaced sim-gpu section with removal note, updated feature flags |
| `sim/docs/MUJOCO_CONFORMANCE.md` | Updated GPU comparison row |
| `sim/docs/MUJOCO_REFERENCE.md` | Replaced GPU path section with removal note |
| `sim/docs/MUJOCO_GAP_ANALYSIS.md` | Updated GPU rows, replaced implementation notes section |
| `sim/L0/core/src/integrate/mod.rs` | Updated doc comments (sim-gpu → GPU backends) |

**Preserved as historical (not edited):**
- `MIGRATION_CHECKLIST.md` — mesh monolith decomposition record
- `COMPLETION_LOG.md` — A-grade completion dates
- `sim/docs/todo/` — archived specs (future_work_3.md, V1_CLEANUP.md, etc.)

**Verification results:**
- YAML syntax: valid (both files)
- `cargo build -p xtask`: compiles clean
- Dangling reference sweep: 0 hits in active files. Only hits are in
  this spec, CF_GEOMETRY_SPEC.md removal notes, historical docs, and
  3 removal notes in sim/docs/ that say "sim-gpu was removed"

---

### Session 3: Final Verification + Close

**Full build + clippy:**
```
cargo build -p mesh -p cf-geometry -p cf-design -p cf-spatial
cargo test -p mesh -p mesh-io -p mesh-repair -p mesh-sdf -p mesh-offset \
  -p mesh-shell -p mesh-measure -p mesh-printability -p mesh-lattice
cargo build -p example-hello-solid -p example-bio-shapes \
  -p example-tendon-finger -p example-pendulum-sim \
  -p example-design-to-sim -p example-mesh-pipeline
cargo clippy -p mesh -- -D warnings
```

**Update this spec:**
- Mark status as complete
- Record final crate count (was 59, now 40)

**Archive or delete `WORKSPACE_TRIM_SPEC.md`** after merge.

## What This Does NOT Touch

- **ML domain** (ml-types, ml-models, ml-dataset, ml-training) — Tier 2
- **Sensor domain** (sensor-types, sensor-fusion) — Tier 2
- **Routing domain** (route-types, route-pathfind, route-optimize) — Tier 2
- **Simulation domain** (all sim-* crates except sim-gpu) — active pipeline
- **Design kernel** (cf-design, cf-geometry, cf-spatial) — active pipeline
- **Examples** — all 6 survive unchanged
