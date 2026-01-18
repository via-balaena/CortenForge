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

### Completed Crates (26)
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

## Quality Infrastructure (COMPLETE)

### Quality Documents

- [x] **CONTRIBUTING.md** - The culture document ("A-grade or it doesn't ship")
- [x] **STANDARDS.md** - Full quality criteria (Seven A-grade criteria)
- [x] **COMPLETION_LOG.md** - Project-wide completion tracking

### Enforcement Tools

- [x] **xtask/** - Rust-based quality enforcement tool
  - `cargo xtask check` - Run all quality checks
  - `cargo xtask grade <crate>` - Grade a crate against A-grade standard
  - `cargo xtask complete <crate>` - Record A-grade completion
  - `cargo xtask ci` - Full CI suite

### Configuration Files

- [x] **rustfmt.toml** - Formatting configuration
- [x] **clippy.toml** - Linting configuration
- [x] **deny.toml** - Dependency policy (cargo-deny)
- [x] **.cargo/config.toml** - Enables `cargo xtask` alias

### CI/CD

- [x] **.github/workflows/quality-gate.yml** - GitHub Actions quality gate

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

## ML Foundation

### ml-models (NOT STARTED)
- [ ] Extract from CortenForge/crates/models/

### ml-inference (NOT STARTED)
- [ ] Extract from CortenForge/crates/inference/

### ml-dataset (NOT STARTED)
- [ ] Extract from CortenForge/crates/burn_dataset/

### ml-training (NOT STARTED)
- [ ] Extract from CortenForge/crates/training/

---

## Vision & Sim Foundation

### vision-core (NOT STARTED)
- [ ] Extract from CortenForge/crates/vision_core/

### vision-capture (NOT STARTED)
- [ ] Extract capture utilities

### sim-core (NOT STARTED)
- [ ] Extract from CortenForge/crates/sim_core/

### sim-physics (NOT STARTED)
- [ ] Physics abstractions

---

## Layer 1 - CortenForge Bevy SDK

### cortenforge crate (NOT STARTED)

Only start after Layer 0 is solid.

- [ ] `CfMeshPlugin`
- [ ] `CfRoutingPlugin`
- [ ] `CfVisionPlugin`
- [ ] `CfSimPlugin`
- [ ] `CfUiPlugin`

---

## Workspace Cargo.toml Updates

When adding new crates, update `/Users/jonhillesheim/forge/cortenforge/Cargo.toml`:

```toml
[workspace]
members = [
    # ... existing ...

    # New mesh crates
    "mesh/mesh-shell",
    "mesh/mesh-decimate",
    "mesh/mesh-subdivide",
    "mesh/mesh-remesh",
    "mesh/mesh-measure",
    "mesh/mesh-thickness",
    "mesh/mesh-slice",
    "mesh/mesh-boolean",
    "mesh/mesh-morph",
    "mesh/mesh-registration",
    "mesh/mesh-scan",
    "mesh/mesh-printability",
    "mesh/mesh-lattice",
    "mesh/mesh-assembly",
    "mesh/mesh-region",
    "mesh/mesh-template",
    "mesh/mesh-gpu",

    # Geometry
    "geometry/curve-types",
    "geometry/lumen-geometry",

    # Spatial
    "crates/cf-spatial",

    # Routing
    "routing/route-types",
    "routing/route-pathfind",
    "routing/route-optimize",

    # ML
    "ml/ml-models",
    "ml/ml-inference",
    "ml/ml-dataset",
    "ml/ml-training",

    # Vision/Sim
    "vision/vision-core",
    "vision/vision-capture",
    "sim/sim-core",
    "sim/sim-physics",

    # SDK
    "cortenforge",
]

[workspace.dependencies]
# Add internal crate dependencies as they're created
mesh-shell = { path = "mesh/mesh-shell" }
mesh-decimate = { path = "mesh/mesh-decimate" }
mesh-subdivide = { path = "mesh/mesh-subdivide" }
# ... etc
```

---

## Notes

1. **Original code archived**: `/Users/jonhillesheim/forge/archive/mesh-pre-refactor/`
2. **Old CortenForge archived**: `/Users/jonhillesheim/forge/archive/CortenForge-old/` (ML/vision crates)
3. **lumen-geometry archived**: `/Users/jonhillesheim/forge/archive/lumen-geometry/` (separate git repo)
4. Focus on clean, modular APIs that can be composed together
5. Each crate should be usable independently (Layer 0 = no Bevy deps)

---

*Last updated: 2026-01-18 (mesh-gpu complete - 26 crates total, MESH DOMAIN COMPLETE)*
