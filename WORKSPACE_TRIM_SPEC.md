# Workspace Trim — Tier 1

**Date:** 2026-03-19
**Branch:** `chore/workspace-trim-tier1`
**Status:** Session 1 complete (745777d) — Session 2 next

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
- **58 references** in 9 non-Cargo files need updating (see Session 2)

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

### Session 2: CI, Tooling, and Documentation

Update all non-Cargo references to deleted crates (58 references in 9 files).

**CI Workflows:**

**Edit `.github/workflows/quality-gate.yml`:**
- Remove 3 entries from WASM_CRATES array (~lines 148-150):
  `mesh-geodesic`, `mesh-transform`, `mesh-slice`
- Remove 16 entries from LAYER0_CRATES array (~lines 392-413):
  `mesh-geodesic`, `mesh-transform`, `mesh-zones`, `mesh-from-curves`,
  `mesh-thickness`, `mesh-slice`, `mesh-decimate`, `mesh-subdivide`,
  `mesh-remesh`, `mesh-region`, `mesh-assembly`, `mesh-boolean`,
  `mesh-registration`, `mesh-morph`, `mesh-scan`, `mesh-template`

**Edit `.github/workflows/scheduled.yml`:**
- Remove `mesh-boolean` from mutation testing matrix (~line 82)

**Edit `xtask/src/setup.rs`:**
- Update example commit message (~line 143)

**Documentation:**

**Edit `README.md`** (10 references):
- Update architecture table (~lines 58, 65)
- Update crate list (~lines 103, 116-124, 131)

**Edit `INFRASTRUCTURE.md`** (4 references):
- Update example commit messages (~line 190)
- Update sim-gpu Layer 0 exception note (~line 223)
- Update critical algorithms section (~line 376)
- Update geodesic example (~line 310)

**Edit `CONTRIBUTING.md`** (2 references):
- Update Layer 0 crate list (~line 218)
- Update sim-gpu exception note (~line 223)

**Edit `STANDARDS.md`** (1 reference):
- Remove curve-types from Layer 0 list (~line 386)

**Edit `CF_GEOMETRY_SPEC.md`** (7 references):
- Update private Aabb references (~lines 29-30, 173-174)
- Update dependency profile comparison (~lines 61, 73)
- Update session exit conditions (~lines 912, 916, 919)
- Add note: "curve-types removed in workspace trim (2026-03-19)"

**Verify:**
- YAML syntax valid (parse check)
- `cargo build -p xtask`
- Dangling reference sweep:
  `grep -r 'mesh-boolean\|mesh-geodesic\|mesh-thickness\|mesh-transform\|mesh-slice\|mesh-scan\|mesh-template\|mesh-remesh\|mesh-decimate\|mesh-subdivide\|mesh-from-curves\|mesh-zones\|mesh-region\|mesh-assembly\|mesh-morph\|mesh-registration\|mesh-gpu\|curve-types\|sim-gpu' --include='*.md' --include='*.yml' --include='*.rs'`
- Only hits should be in this spec file and historical notes in
  `CF_GEOMETRY_SPEC.md`

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
