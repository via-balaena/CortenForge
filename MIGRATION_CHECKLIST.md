# CortenForge Migration Checklist

> **Philosophy**: Breaking changes first. Foundation before framework. Pure Rust before Bevy.

> **Vision**: Industrial-grade foundation, unlimited application. From humanoid robots to vehicles to medical simulation.

> **Standard**: A-grade or it doesn't ship. No exceptions.

---

## Reference

- **Original mesh code (archived)**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/`
- **Old CortenForge (archived)**: `/Users/jonhillesheim/forge/archive/CortenForge-old/`
- **New workspace**: `/Users/jonhillesheim/forge/cortenforge/`
- **Mesh crates location**: `/Users/jonhillesheim/forge/cortenforge/mesh/`
- **Quality standard**: A-grade per STANDARDS.md

---

## Detailed Gap Analysis

### Original mesh-repair Modules (40 modules in archive)

The original `mesh-repair` crate contained approximately 40 modules.
Here's what has been migrated vs what remains:

#### ✅ MIGRATED (10 crates, A-grade complete)

| Original Module | New Crate | Status |
|-----------------|-----------|--------|
| `types.rs` (Mesh, Vertex, Triangle) | `mesh-types` | A-GRADE |
| `io.rs` (STL, OBJ, PLY, 3MF) | `mesh-io` | A-GRADE (STL, OBJ, PLY, 3MF) |
| `adjacency.rs` | `mesh-repair` | A-GRADE |
| `validate.rs` | `mesh-repair` | A-GRADE |
| `repair.rs` (weld, degenerate, duplicate, unreferenced) | `mesh-repair` | A-GRADE |
| `fitting.rs` (RANSAC) | `mesh-transform` | A-GRADE |
| `transform.rs` (PCA) | `mesh-transform` | A-GRADE |
| SDF computation | `mesh-sdf` | A-GRADE |
| Marching cubes / offset | `mesh-offset` | A-GRADE |
| Region grow / zone | `mesh-zones` | A-GRADE |
| Geodesic distances | `mesh-geodesic` | A-GRADE |
| Tube from polyline | `mesh-from-curves` | A-GRADE |
| Template fitting | `mesh-template` | A-GRADE |

#### ❌ NOT YET MIGRATED (requires new crates)

| Original Module | Planned Crate | Priority | Notes |
|-----------------|---------------|----------|-------|
| `holes.rs` | `mesh-repair` extension | HIGH | Hole detection and filling |
| `winding.rs` | `mesh-repair` extension | HIGH | Fix face winding order |
| `components.rs` | `mesh-repair` extension | HIGH | Connected component analysis |
| `intersect.rs` | `mesh-repair` extension | HIGH | Self-intersection detection |
| **mesh-shell/** (entire crate) | `mesh-shell` | **CRITICAL** | MAIN GOAL - shell generation |
| `decimate.rs` | `mesh-decimate` | MEDIUM | ✅ A-GRADE COMPLETE |
| `subdivide.rs` | `mesh-subdivide` | MEDIUM | ✅ A-GRADE COMPLETE |
| `remesh.rs` | `mesh-remesh` | MEDIUM | ✅ A-GRADE COMPLETE |
| `measure.rs` | `mesh-measure` | MEDIUM | ✅ A-GRADE COMPLETE |
| `thickness.rs` | `mesh-thickness` | MEDIUM | ✅ A-GRADE COMPLETE |
| `slice.rs` | `mesh-slice` | MEDIUM | ✅ A-GRADE COMPLETE |
| `boolean.rs` | `mesh-boolean` | LOWER | ✅ A-GRADE COMPLETE |
| `morph.rs` | `mesh-morph` | LOWER | RBF/FFD deformation |
| `registration.rs` | `mesh-registration` | LOWER | ✅ A-GRADE COMPLETE |
| `scan.rs`, `multiscan.rs` | `mesh-scan` | LOWER | Scan processing |
| `pointcloud.rs` | `mesh-scan` | LOWER | Point cloud reconstruction |
| `printability.rs` | `mesh-printability` | LOWER | ✅ A-GRADE COMPLETE |
| `lattice.rs` | `mesh-lattice` | LOWER | Infill generation |
| `assembly.rs` | `mesh-assembly` | LOWER | ✅ A-GRADE COMPLETE |
| `region.rs` | `mesh-region` | LOWER | ✅ A-GRADE COMPLETE |
| `template.rs` | `mesh-template` | LOWER | ✅ A-GRADE COMPLETE |
| **mesh-gpu/** (entire crate) | `mesh-gpu` | LOWER | ✅ A-GRADE COMPLETE |

#### ✅ I/O FORMATS (ALL COMPLETE)

| Format | Original | New | Notes |
|--------|----------|-----|-------|
| STL | ✅ Binary + ASCII | ✅ Binary + ASCII | Complete |
| OBJ | ✅ Load/Save | ✅ Load/Save | Complete |
| PLY | ✅ ASCII + Binary | ✅ ASCII + Binary | Complete |
| 3MF | ✅ With materials, beam lattice | ✅ Geometry only | Complete (no materials/lattice) |
| STEP | ✅ (feature-gated) | ✅ (feature-gated) | Complete via truck ecosystem |

---

## Quick Status

### Completed Crates (32)

#### Mesh Domain (26 crates)
| Crate | Tests | Status |
|-------|-------|--------|
| mesh-types | 35 unit, 33 doc | A-GRADE |
| mesh-io | 31 unit, 17 doc | A-GRADE |
| mesh-geodesic | 22 tests | A-GRADE |
| mesh-transform | 43 unit, 4 doc | A-GRADE |
| mesh-repair (core + extensions) | 74 unit, 21 doc | A-GRADE |
| mesh-sdf | 38 tests | A-GRADE |
| mesh-offset | 8 tests | A-GRADE |
| mesh-zones | 9 tests | A-GRADE |
| mesh-from-curves | 5 tests | A-GRADE |
| **mesh-shell** | 28 unit, 5 doc | **A-GRADE** |
| mesh-measure | 11 unit, 3 doc | A-GRADE |
| mesh-thickness | 14 unit, 2 doc | A-GRADE |
| mesh-slice | 28 unit, 6 doc | A-GRADE |
| mesh-decimate | 21 unit, 2 doc | A-GRADE |
| mesh-subdivide | 21 unit, 3 doc | A-GRADE |
| mesh-remesh | 19 unit, 3 doc | A-GRADE |
| mesh-region | 71 unit, 6 doc | A-GRADE |
| mesh-assembly | 54 unit, 13 doc | A-GRADE |
| mesh-printability | 32 unit, 4 doc | A-GRADE |
| mesh-boolean | 93 unit, 10 doc | A-GRADE |
| mesh-registration | 46 unit, 12 doc | A-GRADE |
| mesh-morph | 57 unit, 32 doc | A-GRADE |
| mesh-scan | 195 unit, 56 doc | A-GRADE |
| mesh-lattice | 84 unit, 28 doc | A-GRADE |
| mesh-template | 102 unit, 46 doc | A-GRADE |
| mesh-gpu | 35 unit, 9 integration | A-GRADE |

#### Sensor Domain (2 crates)
| Crate | Tests | Status |
|-------|-------|--------|
| sensor-types | 80+ unit, 20+ doc | A-GRADE |
| sensor-fusion | 74 unit, 8 doc | A-GRADE |

#### ML Domain (4 crates)
| Crate | Tests | Status |
|-------|-------|--------|
| ml-types | 90+ unit, 15+ doc | A-GRADE |
| ml-models | 45+ unit, 10+ doc | A-GRADE |
| ml-dataset | 63 unit, 11 doc | A-GRADE |
| ml-training | 89 unit, 10 doc | A-GRADE |

### Implementation Order

```
HIGH PRIORITY (COMPLETE):
├── mesh-repair extensions (holes, winding, components, intersect) ✅
└── mesh-shell ✅ MAIN GOAL ACHIEVED

MEDIUM PRIORITY (COMPLETE):
├── mesh-decimate, mesh-subdivide, mesh-remesh ✅
└── mesh-measure, mesh-thickness, mesh-slice ✅

LOWER PRIORITY (in progress):
├── mesh-region ✅
├── mesh-assembly ✅
├── mesh-printability ✅

LOWER PRIORITY (as needed):
├── mesh-boolean ✅
├── mesh-registration ✅
├── mesh-morph ✅
├── mesh-scan ✅
├── mesh-lattice ✅
├── mesh-template ✅
├── mesh-gpu ✅
└── curve-types

SEPARATE TRACKS (independent):
├── cf-spatial + routing/* (for routing features)
├── ml/* + vision/* + sim/* (for ML/vision features)
└── cortenforge (Bevy SDK - after Layer 0 is solid)
```

---

## Quality Infrastructure (COMPLETE - INDUSTRIAL-SCALE)

> **See INFRASTRUCTURE.md for the full constraint specification.**

### Quality Documents

- [x] **CONTRIBUTING.md** - The culture document ("A-grade or it doesn't ship")
- [x] **STANDARDS.md** - Full quality criteria (Seven A-grade criteria)
- [x] **COMPLETION_LOG.md** - Project-wide completion tracking
- [x] **INFRASTRUCTURE.md** - Full constraint specification (industrial-scale)
- [x] **SECURITY.md** - Security policy, signed commits, vulnerability reporting

### Enforcement Tools

- [x] **xtask/** - Rust-based quality enforcement tool
  - `cargo xtask check` - Run all quality checks
  - `cargo xtask grade <crate>` - Grade a crate against A-grade standard
  - `cargo xtask complete <crate>` - Record A-grade completion
  - `cargo xtask ci` - Full CI suite
  - `cargo xtask setup` - Manually install git hooks (auto-installed on build)
  - `cargo xtask uninstall` - Remove git hooks
  - Git hooks auto-install via `xtask/build.rs` on first build

### Configuration Files

- [x] **rustfmt.toml** - Formatting configuration
- [x] **clippy.toml** - Linting configuration
- [x] **deny.toml** - Dependency policy (cargo-deny)
- [x] **.cargo/config.toml** - Enables `cargo xtask` alias

### CI/CD (Industrial-Scale)

- [x] **.github/workflows/quality-gate.yml** - GitHub Actions quality gate
  - Format, Clippy, Tests (3 platforms + ARM64)
  - Coverage (≥90% threshold)
  - Safety scan (unwrap/expect detection)
  - Security scan (cargo-audit)
  - Dependency policy (cargo-deny)
  - Semver checks (cargo-semver-checks)
  - SBOM generation (CycloneDX)
  - Layer 0 Bevy-free verification
  - WASM compatibility check
- [x] **.github/workflows/release.yml** - Release automation
  - Multi-platform builds
  - SBOM attached to releases
  - Auto-generated changelog from conventional commits
  - Signed releases
- [x] **.github/workflows/scheduled.yml** - Weekly maintenance
  - Fresh security advisories
  - Dependency freshness check
  - Mutation testing (cargo-mutants)
  - Benchmark baseline
  - Supply chain verification (cargo-vet)
  - MSRV check

### Pre-Commit Hooks (Auto-Installed)

- [x] **pre-commit** - Format + Clippy + Safety scan
- [x] **commit-msg** - Conventional commit enforcement
- [x] **Auto-install** - Hooks installed automatically via `xtask/build.rs` on first build

### Traceability Infrastructure (ISO 26262 / IEC 62304 / DO-178C Ready)

- [x] **requirements/** - Structured requirements directory
- [x] **requirements/schema.yaml** - Requirement schema definition
- [x] **requirements/mesh/REQ-MESH-001.yaml** - Example requirement with full traceability

### Test Infrastructure (COMPLETE)

- [x] **Umbrella crate** - `mesh/mesh/` re-exports all 26 mesh-* crates with prelude
- [x] **Fuzz testing** - `mesh/mesh-io/fuzz/` with 4 targets (STL, OBJ, PLY, 3MF)
- [x] **Property-based testing** - `mesh/mesh-repair/tests/proptest_mesh.rs` (9 tests)
- [x] **Visual regression** - `mesh/mesh-repair/tests/visual_regression.rs` (7 tests)
- [x] **Thingi10k conformance** - `mesh/mesh-io/tests/thingi10k_conformance.rs`
- [x] **Benchmarks** - 7 benchmark files across crates (io, repair, decimate, remesh, shell, boolean, gpu)
- [x] **Mutation testing** - Weekly cargo-mutants runs on critical crates

### Intentionally Not Migrated

- **mesh-cli** - Apps own their CLI; no SDK-level CLI
- **Pipeline config** - Redundant with modular crate architecture; apps define their own workflows

### Quality Checklist (Per Crate)

Before marking a crate complete:

- [ ] `cargo test -p <crate>` - All tests pass
- [ ] `cargo clippy -p <crate> -- -D warnings` - Zero warnings
- [ ] `RUSTDOCFLAGS="-D warnings" cargo doc -p <crate> --no-deps` - Zero doc warnings
- [ ] No `unwrap()` or `expect()` in library code
- [ ] ≥90% test coverage
- [ ] Doc examples for all public functions
- [ ] Error types use `thiserror`

---

## Pre-Flight (COMPLETE)

### Decisions Locked In

- [x] Mono-repo with path dependencies
- [x] Avian for physics (Bevy-native, ECS-first)
- [x] Layer 0 crates = pure Rust, zero Bevy deps
- [x] Layer 1 = CortenForge Bevy plugins
- [x] No CLI in SDK (apps own their CLI)
- [x] skate-mesh = separate repo
- [x] CortenForge Studio = future (post-1.0)
- [x] Use community crates where appropriate (`pathfinding`, `parry3d`, `kiddo`)
- [x] Build domain-specific crates ourselves (curves, routing, voxel grids)
- [x] anatomy/, electronics/, vehicle/ = documented for future, not built now

### Crate Name Reservations

**Already owned:**
- [x] `cortenforge`
- [x] `mesh-repair`
- [x] `mesh-shell`
- [x] `cortenforge-sim-core`
- [x] `cortenforge-vision-core`
- [x] `lumen-geometry`

**Mesh Domain (check availability before publish):**
- [ ] `mesh-types` - CHECK AVAILABILITY
- [ ] `mesh-io` - CHECK AVAILABILITY
- [ ] `mesh-transform` - CHECK AVAILABILITY
- [ ] `mesh-offset` - CHECK AVAILABILITY
- [ ] `mesh-zones` - CHECK AVAILABILITY
- [ ] `mesh-geodesic` - CHECK AVAILABILITY
- [ ] `mesh-from-curves` - CHECK AVAILABILITY

---

## Workspace Foundation (COMPLETE)

- [x] Create `forge/` directory
- [x] Create workspace `Cargo.toml` with all workspace dependencies
- [x] Create `.gitignore`
- [x] Create root `README.md` with architecture overview
- [x] Create `LICENSE` (Apache-2.0)
- [x] Create directory structure (crates/, mesh/, geometry/, routing/, ml/, vision/, sim/, cortenforge/, docs/)

---

## Mesh Foundation - Completed Crates

### mesh-types (COMPLETE)

The foundation everything else depends on.

- [x] Create `mesh/mesh-types/Cargo.toml`
- [x] Create `src/lib.rs` with module structure
- [x] Create `src/vertex.rs` - `Vertex`, `VertexAttributes`, `VertexColor`
- [x] Create `src/mesh.rs` - `IndexedMesh` with core methods
- [x] Create `src/triangle.rs` - `Triangle` with `normal()`, `area()`, `centroid()`
- [x] Create `src/bounds.rs` - `Aabb` with spatial methods
- [x] Create `src/traits.rs` - `MeshTopology`, `MeshBounds`
- [x] Write unit tests (35 tests passing)
- [x] Write doc tests (33 tests passing)

**Status**: A-GRADE COMPLETE

### mesh-io (COMPLETE)

File I/O, depends only on mesh-types.

- [x] Create `mesh/mesh-io/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/stl.rs` - STL load/save (binary + ASCII)
- [x] Create `src/obj.rs` - OBJ load/save
- [x] Create `src/ply.rs` - PLY load/save (ASCII + binary)
- [x] Create `src/threemf.rs` - 3MF load/save
- [x] Create `src/error.rs` - `IoError` enum
- [x] Write round-trip tests

**Status**: A-GRADE COMPLETE

### mesh-geodesic (COMPLETE)

Geodesic distance computation. Pure graph algorithm.

- [x] Create `mesh/mesh-geodesic/Cargo.toml`
- [x] Create `src/lib.rs` - `compute_distances`, `compute_distances_multi`
- [x] Create `src/dijkstra.rs` - Internal implementation
- [x] Write tests with known distances

**Status**: A-GRADE COMPLETE

### mesh-transform (COMPLETE)

Geometric transformations.

- [x] Create `mesh/mesh-transform/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/ransac.rs` - `ransac_plane()`, `RansacConfig`
- [x] Create `src/pca.rs` - `pca_axes()`, `PcaResult`
- [x] Create `src/transform.rs` - `Transform3D` with all operations
- [x] Write tests (43 tests passing)
- [x] Write doc tests (4 passing)

**Status**: A-GRADE COMPLETE

### mesh-repair - Core (COMPLETE)

**Source**: `mesh-pre-refactor/mesh-repair/src/`

- [x] Create `mesh/mesh-repair/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/adjacency.rs` - `MeshAdjacency`
- [x] Create `src/validate.rs` - `validate_mesh()`, `MeshReport`
- [x] Create `src/repair.rs` - `weld_vertices`, `remove_degenerates`, `remove_duplicates`
- [x] Create `src/error.rs` - `RepairError`
- [x] Write tests

**Status**: CORE COMPLETE (extensions below)

### mesh-sdf (COMPLETE)

Signed distance field operations.

- [x] Create `mesh/mesh-sdf/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/grid.rs` - `ScalarGrid`
- [x] Create `src/compute.rs` - `compute_sdf()`
- [x] Create `src/aabb_tree.rs` - BVH acceleration
- [x] Write tests

**Status**: A-GRADE COMPLETE

### mesh-offset (COMPLETE)

Mesh offset operations using SDF + marching cubes.

- [x] Create `mesh/mesh-offset/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/offset.rs` - `offset_mesh()`, `OffsetConfig`
- [x] Create `src/grid.rs` - Grid utilities
- [x] Create `src/marching_cubes.rs` - Isosurface extraction
- [x] Create `src/error.rs`
- [x] Write tests

**Status**: A-GRADE COMPLETE

### mesh-zones (COMPLETE)

Zone assignment for meshes.

- [x] Create `mesh/mesh-zones/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/zone.rs` - `Zone`, `ZoneConfig`
- [x] Create `src/blend.rs` - `BlendFunction`
- [x] Create `src/assign.rs` - Zone assignment
- [x] Write tests

**Status**: A-GRADE COMPLETE

### mesh-from-curves (COMPLETE)

Generate meshes from curves (tubes, sweeps).

- [x] Create `mesh/mesh-from-curves/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/tube.rs` - `tube_from_polyline()`, `TubeConfig`
- [x] Create `src/frame.rs` - `parallel_transport_frames()`, `Frame`
- [x] Create `src/error.rs`
- [x] Write tests

**Status**: A-GRADE COMPLETE

### Layer Independence (COMPLETE)

- [x] All mesh-* crates build without Bevy deps
- [x] `cargo tree` shows no Bevy/wgpu/winit
- [x] All tests pass: 240+ tests passing

---

# HIGH PRIORITY

## mesh-repair Extensions (COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/`

These extensions are needed by mesh-shell and other downstream crates.

- [x] Create `src/holes.rs` - Hole detection and filling
- [x] Create `src/winding.rs` - Fix face winding order
- [x] Create `src/components.rs` - Connected component analysis
- [x] Create `src/intersect.rs` - Self-intersection detection
- [x] Write tests for each module

| Module | Priority | Complexity | Dependencies | Status |
|--------|----------|------------|--------------|--------|
| `holes.rs` | HIGH | Medium | adjacency | A-GRADE COMPLETE |
| `winding.rs` | HIGH | Medium | adjacency | A-GRADE COMPLETE |
| `components.rs` | HIGH | Medium | adjacency | A-GRADE COMPLETE |
| `intersect.rs` | HIGH | High | spatial | A-GRADE COMPLETE |

---

## mesh-shell (COMPLETE) ← MAIN GOAL ACHIEVED

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-shell/`

Shell generation - THE MAIN MISSING PIECE for 3D printing workflows.

### Tasks
- [x] Create `mesh/mesh-shell/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/builder.rs` - `ShellBuilder` fluent API
- [x] Create `src/shell/generate.rs` - `generate_shell()` (inner + outer surfaces)
- [x] Create `src/shell/rim.rs` - `generate_rim()` (connect boundaries)
- [x] Create `src/shell/validation.rs` - Shell validation
- [x] Create `src/error.rs`
- [x] Write tests (28 unit tests passing)

### Feature Details

| Feature | Priority | Complexity | Dependencies | Status |
|---------|----------|------------|--------------|--------|
| `ShellBuilder` | HIGH | Medium | mesh-offset, mesh-sdf | A-GRADE COMPLETE |
| `generate_shell()` | HIGH | High | mesh-offset | A-GRADE COMPLETE |
| `generate_rim()` | HIGH | High | - | A-GRADE COMPLETE |
| Shell validation | MEDIUM | Medium | mesh-repair | A-GRADE COMPLETE |
| Boundary analysis | HIGH | Medium | adjacency | A-GRADE COMPLETE |
| `ShellParams` | HIGH | Low | - | A-GRADE COMPLETE |
| `WallGenerationMethod` | MEDIUM | Low | - | A-GRADE COMPLETE |

### Architecture Implemented
- Uses existing `mesh-sdf` for SDF computation
- Uses existing `mesh-offset` for marching cubes extraction
- New code focuses on: rim generation, shell stitching, validation
- Two wall generation methods: Normal-based (fast) and SDF-based (consistent thickness)

---

# MEDIUM PRIORITY

## Mesh Topology Operations

### mesh-decimate (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/decimate.rs`

- [x] Create `mesh/mesh-decimate/Cargo.toml`
- [x] Edge collapse (QEM-based decimation)
- [x] Target ratio (reduce to X% of triangles)
- [x] Target count (reduce to N triangles)
- [x] Boundary preservation
- [x] Progress callback

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Edge collapse | HIGH | High | QEM-based decimation | A-GRADE |
| Target ratio | HIGH | Low | Reduce to X% of triangles | A-GRADE |
| Target count | HIGH | Low | Reduce to N triangles | A-GRADE |
| Boundary preservation | MEDIUM | Medium | Keep mesh edges intact | A-GRADE |
| Progress callback | LOW | Low | For UI feedback | A-GRADE |

**Tests**: 21 unit tests, 2 doc tests

### mesh-subdivide (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/subdivide.rs`

- [x] Create `mesh/mesh-subdivide/Cargo.toml`
- [x] Loop subdivision
- [x] Midpoint subdivision (non-smoothing)
- [x] Iteration count control

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Loop subdivision | HIGH | Medium | Standard smoothing | A-GRADE |
| Midpoint subdivision | HIGH | Low | Non-smoothing split | A-GRADE |
| Iteration count | HIGH | Low | Control subdivision level | A-GRADE |

**Tests**: 21 unit tests, 3 doc tests

### mesh-remesh (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/remesh.rs`

- [x] Create `mesh/mesh-remesh/Cargo.toml`
- [x] Isotropic remeshing (split, collapse, flip, smooth)
- [x] Feature edge preservation
- [x] Boundary preservation
- [x] Builder-pattern API (`RemeshParams`)

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Isotropic remesh | HIGH | High | Uniform edge lengths | A-GRADE |
| Edge split/collapse/flip | HIGH | High | Core operations | A-GRADE |
| Tangential smoothing | HIGH | Medium | Vertex relaxation | A-GRADE |
| Feature edge detection | MEDIUM | Medium | Preserve sharp edges | A-GRADE |
| Boundary preservation | HIGH | Medium | Keep boundaries intact | A-GRADE |

**Tests**: 19 unit tests, 3 doc tests

---

## Analysis & Measurement

### mesh-measure (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/measure.rs`

- [x] Create `mesh/mesh-measure/Cargo.toml`
- [x] Volume calculation (signed volume method)
- [x] Surface area (sum of triangle areas)
- [x] Center of mass (volume-weighted centroid)
- [x] Compactness measure (sphericity ratio)
- [x] `MeshStats` comprehensive stats struct

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Volume calculation | HIGH | Low | Signed volume method | A-GRADE |
| Surface area | HIGH | Low | Sum of triangle areas | A-GRADE |
| Center of mass | HIGH | Low | Volume-weighted centroid | A-GRADE |
| Compactness | MEDIUM | Low | Sphericity ratio | A-GRADE |
| MeshStats struct | HIGH | Low | All measurements | A-GRADE |

**Tests**: 11 unit tests, 3 doc tests

### mesh-thickness (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/thickness.rs`

- [x] Create `mesh/mesh-thickness/Cargo.toml`
- [x] Ray-based thickness (cast rays to find walls)
- [x] Thin region detection
- [x] Per-vertex thickness values
- [x] `ThicknessParams`, `ThicknessResult` structs

| Feature | Priority | Complexity | Dependencies | Notes | Status |
|---------|----------|------------|--------------|-------|--------|
| Ray-based thickness | HIGH | High | spatial | Cast rays to find walls | A-GRADE |
| Thin region detection | HIGH | Medium | - | Find areas below threshold | A-GRADE |
| Thickness visualization | MEDIUM | Low | - | Per-vertex thickness values | A-GRADE |
| `ThicknessParams` | HIGH | Low | - | Min thickness, ray count | A-GRADE |
| `ThicknessResult` struct | HIGH | Low | - | Full analysis results | A-GRADE |

**Tests**: 14 unit tests, 2 doc tests

### mesh-slice (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/slice.rs`

- [x] Create `mesh/mesh-slice/Cargo.toml`
- [x] `slice_mesh()` (plane-mesh intersection)
- [x] `slice_at_heights()` (multi-height slicing)
- [x] `Contour` / `Layer` / `SliceStack` types
- [x] Layer statistics (area, perimeter)
- [x] Contour chaining and ordering

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| `slice_mesh()` | HIGH | Medium | Plane-mesh intersection | A-GRADE |
| `slice_at_heights()` | HIGH | Medium | Multi-height slicing | A-GRADE |
| `Contour` / `Layer` types | HIGH | Low | Represent slice results | A-GRADE |
| `SliceStack` | HIGH | Low | Multi-layer container | A-GRADE |
| Layer statistics | MEDIUM | Low | Area, perimeter | A-GRADE |
| Contour chaining | HIGH | Medium | Connect segments | A-GRADE |

**Tests**: 28 unit tests, 6 doc tests

---

# LOWER PRIORITY

## Advanced Operations

### mesh-boolean (A-GRADE COMPLETE)

**Source**: Built from scratch with reference to archive

- [x] Create `mesh/mesh-boolean/Cargo.toml`
- [x] Union (A ∪ B)
- [x] Intersection (A ∩ B)
- [x] Difference (A - B)
- [x] Coplanar handling
- [x] BVH-accelerated intersection detection
- [x] Edge insertion for clean boundaries
- [x] Multi-mesh parallel tree operations
- [x] GPU acceleration (feature-gated)

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Union | HIGH | Very High | A ∪ B | A-GRADE |
| Intersection | HIGH | Very High | A ∩ B | A-GRADE |
| Difference | HIGH | Very High | A - B | A-GRADE |
| Coplanar handling | HIGH | High | Handle touching faces | A-GRADE |
| Edge insertion | MEDIUM | High | Clean intersection boundaries | A-GRADE |
| Multi-mesh union | MEDIUM | Medium | Parallel tree reduction | A-GRADE |
| GPU acceleration | LOW | High | wgpu-based (feature-gated) | A-GRADE |
| Config presets | MEDIUM | Low | for_scans, for_cad, strict | A-GRADE |

**Tests**: 93 unit tests, 10 doc tests

### mesh-morph (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/morph.rs`

- [x] Create `mesh/mesh-morph/Cargo.toml`
- [x] RBF morphing (radial basis function deformation)
- [x] FFD morphing (free-form deformation)
- [x] Constraint types (point, displacement, weighted)

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| RBF morphing | MEDIUM | High | TPS, Gaussian, Multiquadric, InverseMultiquadric kernels | A-GRADE |
| FFD morphing | LOW | High | Bernstein polynomial lattice deformation | A-GRADE |
| Constraint types | MEDIUM | Low | Point, displacement, weighted, vertex-index | A-GRADE |
| `MorphParams` | MEDIUM | Low | Builder API with algorithm selection | A-GRADE |
| `MorphOutput` | MEDIUM | Low | Quality metrics (displacement, stretch, compression) | A-GRADE |
| Vertex masking | LOW | Low | Selective deformation | A-GRADE |

**Tests**: 57 unit tests, 32 doc tests

### mesh-registration (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/registration.rs`

- [x] Create `mesh/mesh-registration/Cargo.toml`
- [x] ICP alignment (iterative closest point) with KD-tree acceleration
- [x] Landmark-based registration with weighted correspondences
- [x] Kabsch algorithm for optimal rigid transform (SVD-based)
- [x] `RigidTransform` type (rotation, translation, scale)

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| ICP alignment | MEDIUM | High | Iterative closest point | A-GRADE |
| Landmark-based | MEDIUM | Medium | Known correspondence | A-GRADE |
| Kabsch algorithm | MEDIUM | Medium | SVD-based optimal transform | A-GRADE |
| `RigidTransform` | MEDIUM | Low | Rotation + translation + scale | A-GRADE |
| KD-tree acceleration | MEDIUM | Medium | O(n log n) nearest neighbor | A-GRADE |

**Tests**: 46 unit tests, 12 doc tests
**Coverage**: 95.19%

---

## Scan Processing

### mesh-scan (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/scan.rs`, `multiscan.rs`, `pointcloud.rs`

- [x] Create `mesh/mesh-scan/Cargo.toml`
- [x] Denoising (Laplacian, Taubin, Bilateral smoothing)
- [x] Outlier removal (statistical k-NN based)
- [x] Spike removal
- [x] Multi-scan alignment (via mesh-registration ICP)
- [x] Multi-scan merging with overlap handling
- [x] Point cloud I/O (XYZ format)
- [x] Normal estimation (PCA-based)
- [x] Ball pivoting surface reconstruction

| Feature | Priority | Complexity | Dependencies | Notes | Status |
|---------|----------|------------|--------------|-------|--------|
| Point cloud structure | MEDIUM | Low | nalgebra | CloudPoint, PointCloud | A-GRADE |
| Point cloud I/O | MEDIUM | Medium | - | XYZ file format | A-GRADE |
| Normal estimation | MEDIUM | Medium | kiddo | PCA-based, KD-tree | A-GRADE |
| Outlier removal | MEDIUM | Medium | kiddo | Statistical k-NN | A-GRADE |
| Spike removal | MEDIUM | Medium | - | Curvature-based | A-GRADE |
| Laplacian smoothing | MEDIUM | Medium | - | Weighted averaging | A-GRADE |
| Taubin smoothing | MEDIUM | Medium | - | Shrink-free variant | A-GRADE |
| Bilateral filtering | MEDIUM | Medium-High | - | Feature-preserving | A-GRADE |
| Multi-scan alignment | LOW | Medium | mesh-registration | ICP wrapper | A-GRADE |
| Multi-scan merging | LOW | Medium-High | mesh-repair | Overlap handling | A-GRADE |
| Ball pivoting | LOW | High | kiddo | Surface reconstruction | A-GRADE |

**Tests**: 195 unit tests, 56 doc tests
**Dependencies**: 5 (mesh-types, mesh-repair, mesh-registration, nalgebra, kiddo)

---

## Manufacturing

### mesh-printability (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/printability.rs`

- [x] Create `mesh/mesh-printability/Cargo.toml`
- [x] Overhang detection
- [x] Support region analysis
- [x] Auto-orientation
- [x] Printer config (FDM, SLA, SLS, MJF)
- [x] Thin wall region detection
- [x] Basic manifold check (watertight detection)

| Feature | Priority | Complexity | Dependencies | Notes | Status |
|---------|----------|------------|--------------|-------|--------|
| Overhang detection | MEDIUM | Medium | - | Find unsupported areas | A-GRADE |
| Support region analysis | MEDIUM | Medium | - | Where supports needed | A-GRADE |
| Auto-orientation | LOW | High | - | Optimize print orientation | A-GRADE |
| Printer config | MEDIUM | Low | - | FDM/SLA/SLS/MJF settings | A-GRADE |
| Build volume check | MEDIUM | Low | - | Check if mesh fits printer | A-GRADE |
| Manifold check | MEDIUM | Medium | - | Detect open/non-manifold edges | A-GRADE |

**Tests**: 32 unit tests, 4 doc tests

### mesh-lattice (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/lattice.rs`

- [x] Create `mesh/mesh-lattice/Cargo.toml`
- [x] Lattice generation (internal structure)
- [x] Infill patterns (gyroid, grid, etc.)
- [x] Density mapping
- [x] TPMS surfaces (Gyroid, Schwarz-P, Diamond, Neovius, IWP)
- [x] Marching cubes isosurface extraction
- [x] Strut-based lattice generation
- [x] Beam lattice data structures (3MF compatible)

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Lattice generation | LOW | High | Cubic, OctetTruss, TPMS, Voronoi | A-GRADE |
| Infill patterns | LOW | Medium | Gyroid, grid, rectilinear, honeycomb | A-GRADE |
| Density mapping | LOW | Medium | Uniform, gradient, radial, custom | A-GRADE |
| TPMS functions | LOW | Medium | Gyroid, Schwarz-P, Diamond, Neovius, IWP | A-GRADE |
| Marching cubes | LOW | High | Isosurface extraction from scalar fields | A-GRADE |
| Strut generation | LOW | Medium | Cylindrical struts with caps | A-GRADE |
| Beam lattice | LOW | Low | 3MF-compatible data structures | A-GRADE |

**Tests**: 84 unit tests, 28 doc tests

### mesh-assembly (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/assembly.rs`

- [x] Create `mesh/mesh-assembly/Cargo.toml`
- [x] Multi-part assembly with hierarchical transforms
- [x] Interference detection (bounding box based)
- [x] Clearance analysis
- [x] Bill of materials with CSV export
- [x] 3MF export with transforms (feature-gated)
- [x] Assembly validation (orphan refs, circular deps, connections)

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Multi-part assembly | LOW | Medium | Manage multiple meshes | A-GRADE |
| Hierarchical transforms | LOW | Medium | Parent-child relationships | A-GRADE |
| Interference detection | LOW | Medium | Check part collisions | A-GRADE |
| Clearance analysis | LOW | Medium | Gap checking | A-GRADE |
| Bill of materials | LOW | Low | Part list with CSV export | A-GRADE |
| 3MF export | LOW | Medium | Feature-gated export | A-GRADE |
| Connection types | LOW | Low | Snap-fit, press-fit, etc. | A-GRADE |

**Tests**: 54 unit tests, 13 doc tests

---

## GPU Acceleration

### mesh-gpu (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-gpu/`

- [x] Create `mesh/mesh-gpu/Cargo.toml`
- [x] `GpuContext` (WGPU device management)
- [x] GPU SDF computation (3-68x speedup for small meshes)
- [x] Automatic CPU fallback via `try_compute_sdf_gpu()`

| Feature | Priority | Complexity | Dependencies | Notes | Status |
|---------|----------|------------|--------------|-------|--------|
| `GpuContext` | HIGH | High | wgpu | Device management | A-GRADE |
| GPU SDF computation | HIGH | High | wgpu, mesh-types | 3-68x speedup for small meshes | A-GRADE |
| `SdfPipeline` | HIGH | High | wgpu | Reusable pipeline | A-GRADE |
| `MeshBuffers` / `SdfGridBuffers` | HIGH | Medium | wgpu, bytemuck | GPU buffer management | A-GRADE |
| Automatic fallback | MEDIUM | Low | - | Try GPU, fall back to CPU | A-GRADE |

**Tests**: 35 unit tests, 9 integration tests
**Note**: GPU SDF is beneficial for small-medium meshes. Surface nets and collision remain CPU-only as they are faster.

---

## Advanced Geometry

### mesh-region (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/region.rs`

- [x] Create `mesh/mesh-region/Cargo.toml`
- [x] Region growing from seed faces
- [x] Material zones with properties (density, thermal, strength)
- [x] Selection by criteria (curvature, normal, distance)
- [x] Flood fill selection
- [x] Region painting (expand/contract)
- [x] Region merging and splitting
- [x] Region statistics

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Region growing | LOW | Medium | Grow from seed faces | A-GRADE |
| Material zones | LOW | Medium | Assign materials with properties | A-GRADE |
| Selection criteria | LOW | Medium | Normal, curvature, distance | A-GRADE |
| Flood fill | LOW | Low | Select connected regions | A-GRADE |
| Region painting | LOW | Medium | Expand/contract by distance | A-GRADE |
| Region merging | LOW | Low | Combine adjacent regions | A-GRADE |
| Region statistics | LOW | Low | Area, centroid, face count | A-GRADE |

**Tests**: 71 unit tests, 6 doc tests

### mesh-template (A-GRADE COMPLETE)

**Source**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/mesh-repair/src/template.rs`

- [x] Create `mesh/mesh-template/Cargo.toml`
- [x] FitTemplate struct with control regions
- [x] RegionDefinition enum (Point, Vertices, Faces, Bounds, Sphere, Cylinder, MeasurementPlane)
- [x] ControlRegion struct with weights and preservation
- [x] FitParams with target scan, landmarks, measurements
- [x] FitResult with stage tracking
- [x] Three-stage fitting pipeline (ICP alignment, RBF morphing, measurement scaling)
- [x] Measurement types (Circumference, Width, Height, Depth)

| Feature | Priority | Complexity | Notes | Status |
|---------|----------|------------|-------|--------|
| Template fitting | LOW | High | Fit template to scan via ICP | A-GRADE |
| Control regions | LOW | Medium | Point, sphere, cylinder, bounds, measurement plane | A-GRADE |
| Landmark deformation | LOW | High | RBF-based morphing via mesh-morph | A-GRADE |
| Measurement extraction | LOW | Medium | Circumference, width, height, depth | A-GRADE |
| Measurement adjustment | LOW | Medium | Scale regions to match targets | A-GRADE |

**Tests**: 102 unit tests, 46 doc tests
**Dependencies**: 5 (mesh-types, mesh-morph, mesh-registration, nalgebra, thiserror)

### geometry/curve-types (NOT STARTED)

- [ ] Create `geometry/curve-types/Cargo.toml`
- [ ] `Curve3D` trait
- [ ] `TubularCurve` trait
- [ ] Polyline (already implicit in mesh-from-curves)
- [ ] Bezier curves (cubic)
- [ ] B-splines
- [ ] NURBS (CAD-style)
- [ ] Arc/Circle

| Feature | Priority | Notes |
|---------|----------|-------|
| Polyline | HIGH | Already implicit in mesh-from-curves |
| Bezier curves | MEDIUM | Cubic Bezier |
| B-splines | LOW | General splines |
| NURBS | LOW | CAD-style curves |
| Arc/Circle | MEDIUM | Circular arcs |

**Note**: Only needed if mesh-from-curves or other crates need mathematical curves beyond polylines.

### lumen-geometry (ARCHIVED)

Medical/anatomical geometry. Moved to archive at `/Users/jonhillesheim/forge/archive/lumen-geometry/`.
Has its own git repo - integrate when needed.

- [ ] Move from archive to `geometry/lumen-geometry/`
- [ ] Add to workspace members
- [ ] Update to use curve-types traits

---

# SEPARATE TRACKS

These are independent of the mesh work and can be done in parallel when needed.

---

## Sensor and ML Architecture

> **Philosophy**: Sim ↔ Real parity. Train in simulation, deploy to hardware. Same types, same inference path.
>
> **Burn-native**: Deep integration with Burn ML framework, not thin abstractions.
> **Bevy-native**: Layer 1 will be pure ECS, not wrapped legacy code.

### Key Design Decisions

1. **No separate vision-types crate** - `Frame`, `DetectionResult`, `BoundingBox`, etc. are ML inference types, not general vision types. They live in `ml-types`.
2. **Burn-native** - No generic `Detector` trait. Models return Burn tensors directly.
3. **WGPU default** - GPU acceleration by default, NdArray as fallback feature.
4. **Sim ↔ Real** - Same sensor types work for Bevy simulation AND real hardware.

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              LAYER 1: Bevy SDK                               │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                           cortenforge                                    ││
│  │    CfMeshPlugin · CfSensorPlugin · CfMlPlugin · CfSimPlugin · CfUi     ││
│  └─────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         LAYER 0: Pure Rust Foundation                        │
│                           (Zero Bevy Dependencies)                           │
├─────────────────────────┬───────────────────────────┬───────────────────────┤
│       sensor/*          │          ml/*             │       mesh/*          │
├─────────────────────────┼───────────────────────────┼───────────────────────┤
│ sensor-types            │ ml-types                  │ (26 crates complete)  │
│ sensor-fusion           │ ml-models                 │                       │
│                         │ ml-dataset                │                       │
│                         │ ml-training               │                       │
└─────────────────────────┴───────────────────────────┴───────────────────────┘
```

### Data Flow: Sim ↔ Real Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          TRAINING LOOP                                       │
│                                                                             │
│   Simulation (Bevy)              Real World Hardware                        │
│        │                              │                                     │
│        ▼                              ▼                                     │
│   Synthetic Sensor Data         Real Sensor Data                            │
│   (sensor-types)                (sensor-types)                              │
│        │                              │                                     │
│        └──────────┬───────────────────┘                                     │
│                   ▼                                                         │
│         Mixed Dataset (ml-dataset)                                          │
│                   │                                                         │
│                   ▼                                                         │
│         Training (ml-training)                                              │
│                   │                                                         │
│                   ▼                                                         │
│         Trained Model (ml-models checkpoint)                                │
└─────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         INFERENCE + FEEDBACK (CI/CD)                         │
│                                                                             │
│   Deploy to Real World ──► Run Inference ──► Collect New Data               │
│                                                    │                        │
│                                                    ▼                        │
│                                           Label (human/auto)                │
│                                                    │                        │
│                                                    ▼                        │
│                                            Back to Dataset                  │
│                                           (TFX-style pipeline)              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Dependency Graph

```
sensor-types (foundation - raw sensor data)
      │
      ├──────────────────────────────┐
      ▼                              ▼
sensor-fusion                   ml-types (includes Frame, DetectionResult, labels, metadata)
                                     │
                    ┌────────────────┼────────────────┐
                    ▼                ▼                ▼
               ml-models        ml-dataset      (uses sensor-types)
                    │                │
                    └────────┬───────┘
                             ▼
                        ml-training
```

---

## Sensor Domain (2 crates)

### sensor-types (A-GRADE COMPLETE) ← FOUNDATION

**Purpose**: Hardware-agnostic sensor data types. Used by:
- Real hardware drivers (ROS2, custom drivers)
- Bevy simulation plugins (simulated sensors)
- ML models (input data)
- Dataset storage (serialized sensor readings)

**Source**: New crate (no direct archive equivalent)

**Design Note**: These are RAW sensor types. Processing/inference types (`Frame`, `DetectionResult`) go in `ml-types`.

- [x] Create `sensor/sensor-types/Cargo.toml`
- [x] Create `src/lib.rs` with module structure
- [x] Create `src/time.rs` - `Timestamp`, `TimeRange`, `Duration`
- [x] Create `src/frame.rs` - `CoordinateFrame`, `Pose3D`
- [x] Create `src/imu.rs` - `ImuReading` (acceleration, angular velocity)
- [x] Create `src/camera.rs` - `CameraFrame`, `CameraIntrinsics`, `CameraExtrinsics`
- [x] Create `src/depth.rs` - `DepthMap`, `DepthPixel`
- [x] Create `src/lidar.rs` - `LidarScan`, `LidarPoint`
- [x] Create `src/force.rs` - `ForceTorqueReading`
- [x] Create `src/encoder.rs` - `EncoderReading`, `JointState`
- [x] Create `src/gps.rs` - `GpsReading`, `GpsAccuracy`
- [x] Create `src/bundle.rs` - `SensorBundle` (multi-sensor container)
- [x] Create `src/error.rs` - `SensorError`
- [x] Write unit tests (80+ tests)
- [x] Write doc tests (20+ tests)

**Tests**: 80+ unit tests, 20+ doc tests

**Dependencies**: `serde`, `thiserror`

**Status**: A-GRADE COMPLETE

---

### sensor-fusion (A-GRADE COMPLETE)

**Purpose**: Real-time stream synchronization, temporal alignment, spatial transforms, multi-rate buffering. Critical for combining multiple sensors into a coherent input for ML.

**Source**: New crate (no direct archive equivalent)

**Use Cases**:
- Align IMU (1kHz) with camera (30Hz) for visual-inertial odometry
- Transform LiDAR points from sensor frame to body frame
- Buffer and interpolate sensor readings for consistent timestamps
- Fuse depth + RGB into RGBD

- [x] Create `sensor/sensor-fusion/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/sync.rs` - `StreamSynchronizer`, `SyncPolicy`, `SyncResult`
- [x] Create `src/buffer.rs` - `StreamBuffer<T>`, time-ordered circular buffer
- [x] Create `src/transform.rs` - `Transform3D`, `TransformChain` (rigid body transforms)
- [x] Create `src/interpolation.rs` - `Interpolator`, `InterpolationMethod` (linear, nearest, previous, next)
- [x] Create `src/error.rs` - `FusionError`
- [x] Write unit tests (74 tests)
- [x] Write doc tests (8 tests)

| Feature | Description |
|---------|-------------|
| `StreamSynchronizer` | Align readings from multiple sensors to common timebase |
| `SyncPolicy::Nearest` | Use temporally nearest reading (no interpolation) |
| `SyncPolicy::Interpolate` | Linear interpolation between readings |
| `SyncPolicy::DropUnmatched` | Skip streams without valid readings |
| `StreamBuffer<T>` | Time-ordered circular buffer for streaming data |
| `Transform3D` | Rigid body transform (rotation + translation) |
| `TransformChain` | Named chain of transforms for sensor calibration |
| `Interpolator` | Temporal interpolation with multiple methods |

**Tests**: 74 unit tests, 8 doc tests

**Dependencies**: `sensor-types`, `glam`, `serde`, `thiserror`

**Status**: A-GRADE COMPLETE

---

## ML Domain (4 crates)

> **Design Decision**: Burn-native. No generic `Detector` trait abstraction. Models return tensors directly.
> Backend default: WGPU primary, NdArray as fallback feature flag.
>
> **IMPORTANT**: `ml-types` includes BOTH dataset schemas AND inference types (`Frame`, `DetectionResult`, etc.)
> We merged vision-types into ml-types because all "vision" types are ML inference types.

### ml-types (A-GRADE COMPLETE)

**Purpose**: ALL ML-related types:
1. **Inference types**: `Frame`, `DetectionResult`, `BoundingBox`, `SegmentationMask`, `KeypointSet`
2. **Dataset schemas**: `DetectionLabel`, `LabelSource`, `CaptureMetadata`, `RunManifest`
3. **Preprocessing**: `ImageStats`

**Source**:
- `/Users/jonhillesheim/forge/archive/CortenForge-old/crates/data_contracts/` (labels, metadata)
- `/Users/jonhillesheim/forge/archive/CortenForge-old/crates/vision_core/src/interfaces.rs` (Frame, DetectionResult)

**Design Note**: We merged what was planned as `vision-types` into `ml-types` because `Frame`, `DetectionResult`, etc. are ML inference types with no use outside ML context.

- [x] Create `ml/ml-types/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/frame.rs` - `Frame` (processed image ready for inference)
- [x] Create `src/detection.rs` - `DetectionResult`, `BoundingBox`
- [x] Create `src/segmentation.rs` - `SegmentationMask`, `InstanceMask`
- [x] Create `src/keypoints.rs` - `KeypointSet`, `Keypoint`, `Skeleton`
- [x] Create `src/tracking.rs` - `TrackingState`, `TrackId`
- [x] Create `src/label.rs` - `DetectionLabel`, `LabelSource`, `ClassLabel`
- [x] Create `src/metadata.rs` - `CaptureMetadata`, `FrameMetadata`
- [x] Create `src/manifest.rs` - `RunManifest`, `DatasetManifest`, schema versioning
- [x] Create `src/stats.rs` - `ImageStats` (mean, std, aspect for normalization)
- [x] Create `src/validation.rs` - Label validation utilities
- [x] Create `src/error.rs` - `MlTypesError`
- [x] Write unit tests (90+ tests)
- [x] Write doc tests (15+ tests)

**Tests**: 90+ unit tests, 15+ doc tests

**Status**: A-GRADE COMPLETE

**Inference Types** (from archived vision_core):

| Type | Description | Source |
|------|-------------|--------|
| `Frame` | Processed image ready for ML | `vision_core::interfaces::Frame` |
| `DetectionResult` | Inference output (boxes + scores) | `vision_core::interfaces::DetectionResult` |
| `BoundingBox` | Normalized box `[x0, y0, x1, y1]` in 0..1 | New |
| `SegmentationMask` | Per-pixel class labels | New |
| `InstanceMask` | Per-pixel instance IDs | New |
| `KeypointSet` | Body/object keypoints | New |
| `Keypoint` | Single keypoint with confidence | New |
| `Skeleton` | Keypoint connectivity graph | New |
| `TrackingState` | Object tracking across frames | New |
| `TrackId` | Unique track identifier | New |

**Dataset Schema Types** (from archived data_contracts):

| Type | Description | Source |
|------|-------------|--------|
| `DetectionLabel` | Ground truth box + class | `data_contracts::capture::DetectionLabel` |
| `LabelSource` | Provenance enum: `SimAuto`, `Human`, `Model` | `data_contracts::capture::LabelSource` |
| `ClassLabel` | Class ID + name | New |
| `CaptureMetadata` | Per-frame metadata | `data_contracts::capture::CaptureMetadata` |
| `FrameMetadata` | Lightweight frame info | New |
| `RunManifest` | Dataset run metadata | `data_contracts::manifest::RunManifest` |
| `DatasetManifest` | Multi-run dataset metadata | New |
| `ImageStats` | Normalization stats (mean, std, aspect) | `data_contracts::preprocess::ImageStats` |

**Frame struct details** (from archive):
```rust
pub struct Frame {
    pub id: u64,
    pub timestamp: f64,          // Sim or capture timestamp (seconds)
    pub rgba: Option<Vec<u8>>,   // Optional raw RGBA8 data
    pub size: (u32, u32),        // Image dimensions (width, height)
    pub path: Option<PathBuf>,   // Optional on-disk location for lazy loading
}
```

**DetectionResult struct details** (from archive):
```rust
pub struct DetectionResult {
    pub frame_id: u64,
    pub positive: bool,          // Any detection above threshold?
    pub confidence: f32,         // Overall confidence
    pub boxes: Vec<[f32; 4]>,    // Normalized boxes [x0,y0,x1,y1] in 0..1
    pub scores: Vec<f32>,        // Per-box scores aligned with boxes
}
```

**DetectionLabel struct details** (from archive):
```rust
pub struct DetectionLabel {
    pub center_world: [f32; 3],           // 3D world position
    pub bbox_px: Option<[f32; 4]>,        // Pixel coordinates
    pub bbox_norm: Option<[f32; 4]>,      // Normalized 0..1
    pub source: Option<LabelSource>,      // SimAuto, Human, or Model
    pub source_confidence: Option<f32>,   // Confidence if Model-generated
}
```

**Dependencies**: `sensor-types` (for `CameraFrame` → `Frame` conversion), `serde`, `thiserror`

**Status**: NOT STARTED

---

### ml-models (A-GRADE COMPLETE)

**Purpose**: Burn model architectures + checkpoint persistence. Self-contained models that know how to save/load themselves.

**Source**:
- `/Users/jonhillesheim/forge/archive/CortenForge-old/crates/models/src/lib.rs`
- `/Users/jonhillesheim/forge/archive/CortenForge-old/crates/inference/src/factory.rs` (checkpoint loading)

**Design Note**: We merged `ml-inference` into `ml-models`. A model should know how to serialize/deserialize itself.

- [x] Create `ml/ml-models/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/classifier.rs` - `LinearClassifier`, `LinearClassifierConfig`
- [x] Create `src/multibox.rs` - `MultiboxModel`, `MultiboxModelConfig`
- [x] Create `src/checkpoint.rs` - `CheckpointFormat`, `CheckpointMetadata`
- [x] Create `src/backend.rs` - Backend utilities
- [x] Create `src/error.rs` - `ModelError`
- [x] Write unit tests (45+ tests)
- [x] Write doc tests (10+ tests)

**Tests**: 45+ unit tests, 10+ doc tests

**Status**: A-GRADE COMPLETE

**LinearClassifier** (from archive):
```rust
#[derive(Debug, Module)]
pub struct LinearClassifier<B: Backend> {
    linear1: nn::Linear<B>,  // 4 → hidden
    linear2: nn::Linear<B>,  // hidden → 1
}

pub struct LinearClassifierConfig {
    pub hidden: usize,  // default: 64
}

// Forward: [B, 4] → relu → [B, 1] (logit)
```

**MultiboxModel** (from archive):
```rust
#[derive(Debug, Module)]
pub struct MultiboxModel<B: Backend> {
    stem: nn::Linear<B>,           // input_dim → hidden
    blocks: Vec<nn::Linear<B>>,    // depth layers: hidden → hidden
    box_head: nn::Linear<B>,       // hidden → max_boxes * 4
    score_head: nn::Linear<B>,     // hidden → max_boxes
    max_boxes: usize,
    input_dim: usize,
}

pub struct MultiboxModelConfig {
    pub hidden: usize,           // default: 128
    pub depth: usize,            // default: 2
    pub max_boxes: usize,        // default: 64
    pub input_dim: Option<usize>, // default: 4, or 12 with features
}

// forward(): [B, D] → [B, max_boxes] scores
// forward_multibox(): [B, D] → ([B, max_boxes, 4] boxes, [B, max_boxes] scores)
```

| Model | Description | Input → Output |
|-------|-------------|----------------|
| `LinearClassifier` | 2-layer feedforward | `[B, 4]` → `[B, 1]` (binary score) |
| `MultiboxModel` | Multi-box detection | `[B, D]` → `([B, N, 4], [B, N])` (boxes, scores) |

**Features**:
- `backend-wgpu` (default) - GPU acceleration via WGPU
- `backend-ndarray` - CPU fallback

**Backend type aliases** (compile-time selection):
```rust
#[cfg(feature = "backend-wgpu")]
pub type DefaultBackend = burn_wgpu::Wgpu<f32>;

#[cfg(all(not(feature = "backend-wgpu"), feature = "backend-ndarray"))]
pub type DefaultBackend = burn_ndarray::NdArray<f32>;
```

**Dependencies**: `burn`, `burn-wgpu` (optional), `burn-ndarray` (optional), `ml-types`, `thiserror`

**Status**: NOT STARTED

---

### ml-dataset (A-GRADE COMPLETE)

**Purpose**: Full dataset lifecycle - load, validate, split, augment, warehouse, record, visualize, prune.

**Source**:
- `/Users/jonhillesheim/forge/archive/CortenForge-old/crates/burn_dataset/`
- `/Users/jonhillesheim/forge/archive/CortenForge-old/crates/capture_utils/`

- [x] Create `ml/ml-dataset/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/sample.rs` - `DatasetSample`, `ResizeMode`
- [x] Create `src/loader.rs` - `DatasetLoader`, `LoaderConfig`
- [x] Create `src/splits.rs` - `split_dataset()`, `SplitConfig`, `DatasetSplits`
- [x] Create `src/augment.rs` - `AugmentConfig`, `AugmentPipeline`
- [x] Create `src/summary.rs` - `DatasetSummary` (statistics)
- [x] Create `src/warehouse.rs` - `WarehouseManifest`, `ShardMetadata`, `ShardBuilder`
- [x] Create `src/error.rs` - `DatasetError`
- [x] Write unit tests (63 tests)
- [x] Write doc tests (11 tests)

**Tests**: 63 unit tests, 11 doc tests

**Status**: A-GRADE COMPLETE

**DatasetSample** (from archive):
```rust
pub struct DatasetSample {
    pub frame_id: u64,
    pub image_chw: Vec<f32>,  // Image in CHW layout, normalized to [0, 1]
    pub width: u32,
    pub height: u32,
    pub boxes: Vec<[f32; 4]>, // Normalized bounding boxes
}
```

**ShardMetadata** (from archive):
```rust
pub struct ShardMetadata {
    pub id: String,
    pub relative_path: String,
    pub shard_version: u32,
    pub samples: usize,
    pub width: u32,
    pub height: u32,
    pub channels: u32,
    pub max_boxes: usize,
    pub checksum_sha256: Option<String>,
    pub dtype: ShardDType,  // F32 or F16
    pub endianness: Endianness,
}
```

| Feature | Description | Source |
|---------|-------------|--------|
| `load_run_dataset()` | Load samples from filesystem | `burn_dataset::capture` |
| `index_runs()` | Index all runs in a directory | `burn_dataset::capture` |
| `split_runs_stratified()` | Train/val split by box count | `burn_dataset::splits` |
| `TransformPipeline` | Augmentation chain (resize, normalize) | `burn_dataset::aug` |
| `WarehouseManifest` | Shard-based storage manifest | `burn_dataset::warehouse` |
| `WarehouseLoaders` | Load shards, iterate samples | `burn_dataset::warehouse` |
| `BurnBatch` | Burn-compatible batch tensors | `burn_dataset::batch` |
| `JsonRecorder` | Write frame metadata to JSON | `capture_utils` |
| `generate_overlays()` | Render detection boxes on images | `capture_utils` |
| `prune_run()` | Filter/copy dataset by criteria | `capture_utils` |

**Features**:
- `burn-runtime` - Enable Burn batch iteration (`BurnBatch`, `BatchIter`)

**Dependencies**: `ml-types`, `sensor-types`, `burn` (optional), `serde`, `serde_json`, `image`, `thiserror`

**Status**: NOT STARTED

---

### ml-training (A-GRADE COMPLETE)

**Purpose**: Training loops, loss functions, optimization. Takes model + dataset, produces trained checkpoint.

**Source**: `/Users/jonhillesheim/forge/archive/CortenForge-old/crates/training/`

- [x] Create `ml/ml-training/Cargo.toml`
- [x] Create `src/lib.rs`
- [x] Create `src/config.rs` - `TrainingConfig`, `OptimizerConfig`, `LearningRateSchedule`
- [x] Create `src/loss.rs` - `box_loss`, `classification_loss`, `detection_loss`, `compute_iou`, `giou_loss`
- [x] Create `src/matching.rs` - `BoxMatcher`, `MatchResult`, `MatchStrategy`
- [x] Create `src/trainer.rs` - `Trainer`, `TrainingState`
- [x] Create `src/metrics.rs` - `EpochMetrics`, `TrainingMetrics`
- [x] Create `src/error.rs` - `TrainingError`
- [x] Write unit tests (89 tests)
- [x] Write doc tests (10 tests)

**Tests**: 89 unit tests, 10 doc tests (1 ignored)

**Status**: A-GRADE COMPLETE

**CollatedBatch** (from archive):
```rust
pub struct CollatedBatch<B: Backend> {
    pub boxes: Tensor<B, 3>,     // [batch, max_boxes, 4]
    pub box_mask: Tensor<B, 2>,  // [batch, max_boxes] - 1.0 where valid
    pub features: Tensor<B, 2>,  // [batch, feature_dim] - image stats
}
```

**TrainConfig** (derived from archive TrainArgs, without clap):
```rust
pub struct TrainConfig {
    pub epochs: usize,
    pub batch_size: usize,
    pub learning_rate: f32,
    pub max_boxes: usize,
    pub lambda_box: f32,     // Loss weight for box regression
    pub lambda_obj: f32,     // Loss weight for objectness
    pub checkpoint_dir: PathBuf,
}
```

**Loss computation** (from archive):
- Box regression: L1 loss on matched predictions only
- Objectness: BCE loss with sigmoid on predicted scores
- Matching: Greedy IoU-based matching (assign each GT to best pred)

| Feature | Description | Source |
|---------|-------------|--------|
| `collate()` | Batch collation with padding | `training::dataset` |
| `collate_from_burn_batch()` | Collate from warehouse batch | `training::dataset` |
| `run_train()` | Training loop with checkpoints | `training::util` |
| `build_greedy_targets()` | GT-pred matching by IoU | `training::util` |
| `iou_xyxy()` | IoU computation for boxes | `training::util` |
| `TrainConfig` | Training hyperparameters | `training::util::TrainArgs` (without clap) |

**Features**:
- `backend-wgpu` (default) - GPU training
- `backend-ndarray` - CPU fallback

**Dependencies**: `ml-models`, `ml-dataset`, `ml-types`, `burn`, `thiserror`

**Status**: NOT STARTED

---

## Migration Order

```
Phase 1: Foundation Types ✅ COMPLETE
├── 1. sensor-types     ✅ A-GRADE (raw sensor data)
└── 2. ml-types         ✅ A-GRADE (inference types + dataset schemas)

Phase 2: ML Core ✅ COMPLETE
├── 3. ml-models        ✅ A-GRADE (Burn architectures + checkpoint)
├── 4. ml-dataset       ✅ A-GRADE (dataset lifecycle)
└── 5. ml-training      ✅ A-GRADE (training loops)

Phase 3: Sensor Fusion ✅ COMPLETE
└── 6. sensor-fusion    ✅ A-GRADE (stream sync, transforms)

Phase 4: Layer 1 Bevy SDK (after Layer 0 solid)
└── 7. cortenforge      (CfSensorPlugin, CfMlPlugin, CfSimPlugin) - NOT STARTED
```

**Total: 6 new crates COMPLETE** (sensor-types, sensor-fusion, ml-types, ml-models, ml-dataset, ml-training)

---

## Spatial Foundation

### cf-spatial (NOT STARTED)

- [ ] Create `crates/cf-spatial/Cargo.toml`
- [ ] `VoxelGrid` struct
- [ ] `OccupancyMap` (sparse storage)
- [ ] Raycasting
- [ ] Sphere overlap queries

---

## Routing Foundation

### route-types (NOT STARTED)
- [ ] Create `routing/route-types/Cargo.toml`
- [ ] `Path3D`, `Waypoint`
- [ ] `Route`, `RouteBundle`
- [ ] Constraints

### route-pathfind (NOT STARTED)
- [ ] Create `routing/route-pathfind/Cargo.toml`
- [ ] Voxel A* pathfinding
- [ ] Path smoothing
- [ ] Clearance inflation

### route-optimize (NOT STARTED)
- [ ] Create `routing/route-optimize/Cargo.toml`
- [ ] Multi-objective optimization
- [ ] Local search refinement

---

## Layer 1 - CortenForge Bevy SDK

### cortenforge crate (NOT STARTED)

Only start after Layer 0 is solid.

- [ ] `CfMeshPlugin` - Mesh asset loading, repair systems
- [ ] `CfSensorPlugin` - Simulated sensors, sensor components
- [ ] `CfVisionPlugin` - Capture camera, GPU readback, frame buffer
- [ ] `CfMlPlugin` - Async inference scheduling, detector resources
- [ ] `CfSimPlugin` - Camera controller, autopilot, recorder systems
- [ ] `CfUiPlugin` - Detection overlays, debug visualization

**Bevy-specific systems migrated from archive:**
- Camera systems (`setup_camera`, `camera_controller`, `pov_toggle_system`)
- Capture plugin (`CapturePlugin` → `CfVisionPlugin`)
- Inference runtime (`InferenceRuntimePlugin` → `CfMlPlugin`)
- Sim config (`SimPlugin`, `SimConfig`, `SimRunMode`)
- Autopilot (`AutoDrive`, `DatagenInit`)
- Recorder (`RecorderConfig`, `RecorderState`)

---

## Workspace Cargo.toml Updates

When adding new crates, update `/Users/jonhillesheim/forge/cortenforge/Cargo.toml`:

```toml
[workspace]
members = [
    # Mesh (complete - 26 crates)
    "mesh/mesh-types",
    "mesh/mesh-io",
    # ... all 26 mesh crates ...

    # Sensor (2 crates)
    "sensor/sensor-types",
    "sensor/sensor-fusion",

    # ML (4 crates - includes inference types, no separate vision crate)
    "ml/ml-types",
    "ml/ml-models",
    "ml/ml-dataset",
    "ml/ml-training",

    # Geometry
    "geometry/curve-types",

    # Spatial
    "crates/cf-spatial",

    # Routing
    "routing/route-types",
    "routing/route-pathfind",
    "routing/route-optimize",

    # SDK (Layer 1)
    "cortenforge",
]

[workspace.dependencies]
# Sensor
sensor-types = { path = "sensor/sensor-types" }
sensor-fusion = { path = "sensor/sensor-fusion" }

# ML (no vision-types - merged into ml-types)
ml-types = { path = "ml/ml-types" }
ml-models = { path = "ml/ml-models" }
ml-dataset = { path = "ml/ml-dataset" }
ml-training = { path = "ml/ml-training" }

# External - ML
burn = "0.15"
burn-wgpu = "0.15"
burn-ndarray = "0.15"
```

---

## Notes

1. **Original code archived**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/`
2. **Old CortenForge archived**: `/Users/jonhillesheim/forge/archive/CortenForge-old/` (ML/vision crates)
3. **lumen-geometry archived**: `/Users/jonhillesheim/forge/archive/lumen-geometry/` (separate git repo)
4. Focus on clean, modular APIs that can be composed together
5. Each crate should be usable independently (Layer 0 = no Bevy deps)
6. **Sim ↔ Real parity**: Same sensor-types work for simulation AND real hardware
7. **Burn-native**: Deep Burn integration, not generic ML traits
8. **CI/CD ready**: Dataset and training crates support TFX-style pipelines
9. **No vision-types crate**: `Frame`, `DetectionResult`, etc. are ML inference types → live in `ml-types`
10. **6 new crates total**: sensor-types, sensor-fusion, ml-types, ml-models, ml-dataset, ml-training

---

*Last updated: 2026-01-18 (mesh domain complete, sensor/ml domains complete - 32 A-grade crates total)*
