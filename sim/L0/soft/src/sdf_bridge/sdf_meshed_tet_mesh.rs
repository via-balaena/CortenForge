//! `SdfMeshedTetMesh` — placeholder mesher's [`Mesh`] impl.
//!
//! Walks the [`MeshingHints`] bbox on a uniform cubic lattice with the
//! same z-major-y-x outer order and 6-tet Kuhn per-cell decomposition
//! as [`super::lattice::build_lattice_tets`]; for each Kuhn tet,
//! evaluates the [`Sdf`] at the four cube-corner positions, dispatches
//! through [`super::marching_tet`]'s 16-case table, post-hoc swaps
//! left-handed sub-tets, and drops degenerate sub-tets below
//! [`super::marching_tet::EPSILON_VOLUME`]. Vertex deduplication runs
//! through a single `BTreeMap` keyed by sorted-pair-of-grid-corners,
//! correctly merging the space diagonal shared by all 6 Kuhn tets in
//! each cube and any face / axis-aligned edge shared between adjacent
//! cells.
//!
//! The complete determinism contract per scope memo §3 Decision M:
//!
//! - Outer iteration: `for iz in 0..nz { for iy in 0..ny { for ix in
//!   0..nx { ... }}}`. Same as lattice.rs and cf-design's marching
//!   cubes.
//! - Per-cell Kuhn-tet emission order: fixed by `KUHN_TETS`.
//! - Per-Kuhn-tet sub-tet emission order: the order listed in the
//!   per-case array of `SUB_TETS_BY_CASE` (one fixed sequence per
//!   case index).
//! - Vertex cache (D-9): `BTreeMap<((i32,i32,i32),(i32,i32,i32)),
//!   VertexId>` with the key being a sorted-lexicographic pair of the
//!   two grid-corner integer coordinates. Encodes any edge between
//!   any two grid corners uniformly — the 12 axis-aligned cube edges,
//!   the 6 face diagonals, and the 1 space diagonal that the 6-tet
//!   Kuhn decomposition produces. The space diagonal in particular is
//!   shared by all 6 tets within a cube, so its cut point MUST dedup
//!   to a single [`VertexId`]; sorted-pair canonicalization makes the
//!   key independent of which Kuhn tet first encounters the edge.
//!   `BTreeMap` (not `HashMap`) — defense-in-depth alignment with the
//!   walking-skeleton D-3 pattern of avoiding `HashMap` on numeric
//!   paths. The current cache is only accessed via point lookups, so
//!   `HashMap` would also be content-deterministic, but `BTreeMap`
//!   stays robust against future code that might iterate the cache.
//! - Cut-position computation (canonicalized): when a cache entry is
//!   first inserted, the position is computed using the sorted-pair
//!   `(min, max)` grid corners as the (a, b) endpoints, NOT the
//!   per-tet endpoint encounter order — same `t = -v_a / (v_b - v_a)`
//!   formula evaluates bit-equal regardless of which Kuhn tet's MT
//!   walk first traverses the edge.
//! - Vertex IDs: assigned as `vertices.len()` at first insertion;
//!   deterministic in the `BTreeMap` walk-by-key sense — but for any
//!   given input, the same edges are encountered in the same outer
//!   order, so the assignment matches run-to-run.
//!
//! The post-hoc orientation swap (last-two-slot swap when signed
//! volume is negative) plus the degenerate-tet filter
//! (`signed_volume < EPSILON_VOLUME` after the swap) together
//! guarantee that every emitted sub-tet has signed volume strictly
//! above `EPSILON_VOLUME`, holding the right-handed-by-construction
//! contract per Decision H. Any structural algorithm bug producing
//! left-handed sub-tets (e.g. a wrong vertex set in the case table)
//! would still be caught by Phase 3's downstream III-2 strict
//! `signed_volume > 0` assertion at the post-filter mesh — the swap
//! cannot mask topology bugs because it preserves the four-vertex
//! set, only reorders.
//!
//! Empty-mesh detection runs after the cell walk: if no sub-tet
//! survived the degenerate filter, return [`MeshingError::EmptyMesh`]
//! (typical cause: misconfigured `bbox` placed entirely outside the
//! SDF<0 region). Non-finite SDF values surface as
//! [`MeshingError::NonFiniteSdfValue`] at sampling time, before any
//! mesh state is constructed.

use std::collections::BTreeMap;

use super::MeshingHints;
use super::lattice::{CORNER_OFFSETS, KUHN_TETS};
use super::marching_tet::{self, EPSILON_VOLUME, VertexSlot};
use super::sdf::Sdf;
use crate::Vec3;
use crate::mesh::{Mesh, MeshAdjacency, QualityMetrics, TetId, VertexId, quality};

/// Failure modes of [`SdfMeshedTetMesh::from_sdf`].
///
/// Right-handedness violations are NOT a [`MeshingError`] variant per
/// scope memo §3 Decision H — degenerate sub-tets are silently
/// filtered (D-10) and right-handed-by-construction is the algorithm
/// contract; structural bugs surface via downstream Phase 3 invariants
/// (III-2 strict `signed_volume > 0`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MeshingError {
    /// No sub-tet survived MT-clip and the degenerate filter, typically
    /// because the `bbox` is positioned entirely outside the SDF<0
    /// region (the SDF is positive at every grid corner).
    EmptyMesh,
    /// The SDF returned `NaN` or `±∞` at a lattice grid point.
    /// Indicates a malformed [`Sdf`] impl rather than a meshing
    /// failure; surfaced before any mesh state is built.
    NonFiniteSdfValue,
}

/// SDF-meshed tetrahedral mesh — Phase 3's third [`Mesh`] impl.
///
/// Construct via [`SdfMeshedTetMesh::from_sdf`]. Fields are private;
/// the only construction path runs through the placeholder mesher,
/// which guarantees right-handed orientation by the post-hoc swap +
/// degenerate filter (Decision M D-10) and dedup of grid-shared
/// edges by the sorted-pair `BTreeMap` (D-9).
#[derive(Clone, Debug)]
pub struct SdfMeshedTetMesh {
    vertices: Vec<Vec3>,
    tets: Vec<[VertexId; 4]>,
    adj: MeshAdjacency,
    q: QualityMetrics,
}

/// Sorted-pair-of-grid-corners key for the vertex cache (D-9).
///
/// `(g, g)` (same corner twice) keys a grid-corner vertex; `(g_min,
/// g_max)` with `g_min < g_max` keys an edge cut. Sorted-pair
/// canonicalization is what makes the key deterministic for diagonal
/// edges shared between Kuhn tets within a cube.
type GridKey = ((i32, i32, i32), (i32, i32, i32));

impl SdfMeshedTetMesh {
    /// Mesh the SDF inside region across the [`MeshingHints`] bbox.
    ///
    /// Walks the bbox on a uniform cubic lattice with spacing
    /// `hints.cell_size`, decomposes each cell into 6 Kuhn tets, and
    /// applies marching-tetrahedra clipping at the SDF zero crossing.
    /// Returns `Err` on either an empty inside region or a non-finite
    /// SDF return value — see [`MeshingError`].
    ///
    /// # Errors
    ///
    /// - [`MeshingError::EmptyMesh`] if the inside region produced no
    ///   non-degenerate sub-tets.
    /// - [`MeshingError::NonFiniteSdfValue`] if the SDF returned `NaN`
    ///   or `±∞` at any lattice corner.
    pub fn from_sdf(sdf: &dyn Sdf, hints: &MeshingHints) -> Result<Self, MeshingError> {
        let cell_size = hints.cell_size;
        let extent = hints.bbox.max - hints.bbox.min;
        let nx = axis_cell_count(extent.x, cell_size);
        let ny = axis_cell_count(extent.y, cell_size);
        let nz = axis_cell_count(extent.z, cell_size);

        let stride_y = nx + 1;
        let stride_z = (nx + 1) * (ny + 1);

        let (grid_position, grid_sdf) = sample_grid_sdf(sdf, hints, [nx, ny, nz])?;

        let mut vertices: Vec<Vec3> = Vec::new();
        let mut tets: Vec<[VertexId; 4]> = Vec::new();
        let mut vertex_cache: BTreeMap<GridKey, VertexId> = BTreeMap::new();

        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let (corner_grid, corner_sdf) =
                        cell_corner_state(ix, iy, iz, &grid_sdf, stride_y, stride_z);

                    for &kuhn in &KUHN_TETS {
                        process_kuhn_tet(
                            kuhn,
                            &corner_grid,
                            &corner_sdf,
                            &grid_position,
                            &grid_sdf,
                            stride_y,
                            stride_z,
                            &mut vertices,
                            &mut tets,
                            &mut vertex_cache,
                        );
                    }
                }
            }
        }

        if tets.is_empty() {
            return Err(MeshingError::EmptyMesh);
        }

        let q = quality::compute_metrics(&vertices, &tets);
        Ok(Self {
            vertices,
            tets,
            adj: MeshAdjacency,
            q,
        })
    }
}

impl Mesh for SdfMeshedTetMesh {
    fn n_tets(&self) -> usize {
        self.tets.len()
    }

    fn n_vertices(&self) -> usize {
        self.vertices.len()
    }

    fn tet_vertices(&self, tet: TetId) -> [VertexId; 4] {
        let idx = tet as usize;
        assert!(
            idx < self.tets.len(),
            "tet ID {idx} out of bounds for {n}-tet SdfMeshedTetMesh",
            n = self.tets.len(),
        );
        self.tets[idx]
    }

    fn positions(&self) -> &[Vec3] {
        &self.vertices
    }

    fn adjacency(&self) -> &MeshAdjacency {
        &self.adj
    }

    fn quality(&self) -> &QualityMetrics {
        &self.q
    }

    // n_tets returns usize; TetId is u32. Mesher output stays well
    // below u32::MAX for any plausible scene (canonical III-1 has
    // ~10k tets pre-filter).
    #[allow(clippy::cast_possible_truncation)]
    fn equals_structurally(&self, other: &dyn Mesh) -> bool {
        if self.n_tets() != other.n_tets() {
            return false;
        }
        if self.n_vertices() != other.n_vertices() {
            return false;
        }
        for tet_id in 0..self.n_tets() as TetId {
            if self.tet_vertices(tet_id) != other.tet_vertices(tet_id) {
                return false;
            }
        }
        true
    }
}

// Round-to-nearest cell count: integer-multiple inputs (e.g. III-1's
// `0.24 / 0.02 = 12`) hit exactly; non-aligned bboxes drift by at
// most half a cell_size at the lattice extent. The .max(1.0) floor
// makes degenerate / inverted bboxes collapse to a single cell rather
// than zero per Decision H.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn axis_cell_count(extent: f64, cell_size: f64) -> usize {
    (extent / cell_size).round().max(1.0) as usize
}

// Cast loses precision only at ix/iy/iz > 2^53, far beyond any
// plausible lattice scale (canonical III-1 scene is 13 corners/axis).
#[allow(clippy::cast_precision_loss)]
fn grid_offset(ix: usize, iy: usize, iz: usize, cell_size: f64) -> Vec3 {
    Vec3::new(ix as f64, iy as f64, iz as f64) * cell_size
}

// Lattice indices fit in i32 for any plausible scene; the BTreeMap
// key needs a signed integer type (so sorted-pair `(min, max)`
// canonicalizes via `<` on tuples).
#[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
fn cell_corner_state(
    ix: usize,
    iy: usize,
    iz: usize,
    grid_sdf: &[f64],
    stride_y: usize,
    stride_z: usize,
) -> ([(i32, i32, i32); 8], [f64; 8]) {
    let mut corner_grid = [(0_i32, 0_i32, 0_i32); 8];
    let mut corner_sdf = [0.0_f64; 8];
    for (k, &(dx, dy, dz)) in CORNER_OFFSETS.iter().enumerate() {
        let cx = ix + dx;
        let cy = iy + dy;
        let cz = iz + dz;
        corner_grid[k] = (cx as i32, cy as i32, cz as i32);
        corner_sdf[k] = grid_sdf[cz * stride_z + cy * stride_y + cx];
    }
    (corner_grid, corner_sdf)
}

fn slot_to_key(slot: VertexSlot, parent_grid: &[(i32, i32, i32); 4]) -> GridKey {
    if slot < 4 {
        let g = parent_grid[slot as usize];
        (g, g)
    } else {
        let (a_slot, b_slot) = marching_tet::edge_endpoints(slot);
        let g_a = parent_grid[a_slot as usize];
        let g_b = parent_grid[b_slot as usize];
        if g_a < g_b { (g_a, g_b) } else { (g_b, g_a) }
    }
}

// Cast back to usize is safe because `cell_corner_state` produced the
// i32 key from in-range usize lattice indices; no negative coordinate
// reaches this path.
#[allow(clippy::cast_sign_loss)]
fn compute_position(
    key: GridKey,
    grid_position: &[Vec3],
    grid_sdf: &[f64],
    stride_y: usize,
    stride_z: usize,
) -> Vec3 {
    let (g_a, g_b) = key;
    let idx_a = (g_a.2 as usize) * stride_z + (g_a.1 as usize) * stride_y + (g_a.0 as usize);
    let p_a = grid_position[idx_a];
    if g_a == g_b {
        return p_a;
    }
    let idx_b = (g_b.2 as usize) * stride_z + (g_b.1 as usize) * stride_y + (g_b.0 as usize);
    let p_b = grid_position[idx_b];
    let v_a = grid_sdf[idx_a];
    let v_b = grid_sdf[idx_b];
    let t = marching_tet::cut_parameter(v_a, v_b);
    p_a + t * (p_b - p_a)
}

// Sample the SDF at every grid corner in z-major-y-x order matching
// `lattice.rs`'s vertex layout convention. Returns the position and
// value arrays jointly so corner-index lookups index into both.
fn sample_grid_sdf(
    sdf: &dyn Sdf,
    hints: &MeshingHints,
    counts: [usize; 3],
) -> Result<(Vec<Vec3>, Vec<f64>), MeshingError> {
    let [nx, ny, nz] = counts;
    let n_grid = (nx + 1) * (ny + 1) * (nz + 1);
    let mut grid_position: Vec<Vec3> = Vec::with_capacity(n_grid);
    let mut grid_sdf: Vec<f64> = Vec::with_capacity(n_grid);
    for iz in 0..=nz {
        for iy in 0..=ny {
            for ix in 0..=nx {
                let p = hints.bbox.min + grid_offset(ix, iy, iz, hints.cell_size);
                let v = sdf.eval(p);
                if !v.is_finite() {
                    return Err(MeshingError::NonFiniteSdfValue);
                }
                grid_position.push(p);
                grid_sdf.push(v);
            }
        }
    }
    Ok((grid_position, grid_sdf))
}

// MT-clip a single Kuhn tet, dedup vertices via the shared cache, run
// the post-hoc orientation swap + degenerate filter, and append any
// surviving sub-tets to the output mesh.
#[allow(clippy::too_many_arguments)]
fn process_kuhn_tet(
    kuhn: [u8; 4],
    corner_grid: &[(i32, i32, i32); 8],
    corner_sdf: &[f64; 8],
    grid_position: &[Vec3],
    grid_sdf: &[f64],
    stride_y: usize,
    stride_z: usize,
    vertices: &mut Vec<Vec3>,
    tets: &mut Vec<[VertexId; 4]>,
    vertex_cache: &mut BTreeMap<GridKey, VertexId>,
) {
    let parent_grid: [(i32, i32, i32); 4] = [
        corner_grid[kuhn[0] as usize],
        corner_grid[kuhn[1] as usize],
        corner_grid[kuhn[2] as usize],
        corner_grid[kuhn[3] as usize],
    ];
    let parent_sdf: [f64; 4] = [
        corner_sdf[kuhn[0] as usize],
        corner_sdf[kuhn[1] as usize],
        corner_sdf[kuhn[2] as usize],
        corner_sdf[kuhn[3] as usize],
    ];

    let case = marching_tet::case_index(parent_sdf);
    if case == 0 {
        return;
    }

    for sub_tet_template in marching_tet::sub_tets_for_case(case) {
        let mut sub_vids = [0_u32; 4];
        for (k, &slot) in sub_tet_template.iter().enumerate() {
            let key = slot_to_key(slot, &parent_grid);
            sub_vids[k] = get_or_insert_vid(
                key,
                grid_position,
                grid_sdf,
                stride_y,
                stride_z,
                vertices,
                vertex_cache,
            );
        }

        let p = [
            vertices[sub_vids[0] as usize],
            vertices[sub_vids[1] as usize],
            vertices[sub_vids[2] as usize],
            vertices[sub_vids[3] as usize],
        ];
        let sv_unswapped = marching_tet::signed_volume(p[0], p[1], p[2], p[3]);

        // Post-hoc orientation correction (memo §3 D-10 contract).
        // Swapping slots 2 and 3 flips the determinant sign while
        // preserving the same 4-vertex set, so the topological
        // coverage of the inside region is unchanged.
        let (final_vids, sv) = if sv_unswapped < 0.0 {
            (
                [sub_vids[0], sub_vids[1], sub_vids[3], sub_vids[2]],
                -sv_unswapped,
            )
        } else {
            (sub_vids, sv_unswapped)
        };

        // Degenerate-tet filter (D-10): drops zero-volume and FP-near-
        // zero sub-tets that arise from lattice-on-sphere coincidences.
        if sv < EPSILON_VOLUME {
            continue;
        }
        tets.push(final_vids);
    }
}

// Single-entry vertex-cache lookup: returns the existing VertexId on a
// hit, or computes the cut/grid-corner position deterministically and
// inserts on a miss.
fn get_or_insert_vid(
    key: GridKey,
    grid_position: &[Vec3],
    grid_sdf: &[f64],
    stride_y: usize,
    stride_z: usize,
    vertices: &mut Vec<Vec3>,
    vertex_cache: &mut BTreeMap<GridKey, VertexId>,
) -> VertexId {
    if let Some(&existing) = vertex_cache.get(&key) {
        return existing;
    }
    let position = compute_position(key, grid_position, grid_sdf, stride_y, stride_z);
    // Mesher output stays well below u32::MAX vertices for any plausible
    // scene (canonical III-1 scene has < 11k vertices). Phase H may
    // revisit if scenes ever exceed u32::MAX vertices.
    #[allow(clippy::cast_possible_truncation)]
    let new_id = vertices.len() as VertexId;
    vertices.push(position);
    vertex_cache.insert(key, new_id);
    new_id
}

#[cfg(test)]
mod tests {
    // Tests panic on unexpected `Err` via expect/expect_err — that IS
    // the test failure signal in the inline-test convention. The hand-
    // built scenes here either succeed by construction or are
    // explicitly chosen to drive a specific error variant; any
    // deviation is a regression to investigate.
    #![allow(clippy::expect_used)]

    use super::super::Aabb3;
    use super::super::sdf::{Sdf, SphereSdf};
    use super::*;

    /// Constant-positive SDF: every grid corner is outside, MT-clip
    /// produces no sub-tets. Used to drive the [`MeshingError::EmptyMesh`]
    /// path.
    struct AlwaysOutsideSdf;
    impl Sdf for AlwaysOutsideSdf {
        fn eval(&self, _p: Vec3) -> f64 {
            1.0
        }
        fn grad(&self, _p: Vec3) -> Vec3 {
            Vec3::z()
        }
    }

    /// Constant-negative SDF: every grid corner is inside, every Kuhn
    /// tet is case 15 → emitted verbatim. Used to drive the
    /// "case-15-only" path that should reproduce the lattice generator
    /// output volume-equivalently.
    struct AlwaysInsideSdf;
    impl Sdf for AlwaysInsideSdf {
        fn eval(&self, _p: Vec3) -> f64 {
            -1.0
        }
        fn grad(&self, _p: Vec3) -> Vec3 {
            Vec3::z()
        }
    }

    /// Half-space SDF cutting along `x = threshold`. Inside region is
    /// the slab `x ≤ threshold`. Yields a known analytic volume that
    /// stress-tests the full MT case-mix at a single planar interface.
    struct HalfSpaceXSdf {
        threshold: f64,
    }
    impl Sdf for HalfSpaceXSdf {
        fn eval(&self, p: Vec3) -> f64 {
            p.x - self.threshold
        }
        fn grad(&self, _p: Vec3) -> Vec3 {
            Vec3::x()
        }
    }

    /// SDF that returns `NaN` to drive the
    /// [`MeshingError::NonFiniteSdfValue`] path.
    struct NaNSdf;
    impl Sdf for NaNSdf {
        fn eval(&self, _p: Vec3) -> f64 {
            f64::NAN
        }
        fn grad(&self, _p: Vec3) -> Vec3 {
            Vec3::z()
        }
    }

    fn unit_cube_hints() -> MeshingHints {
        MeshingHints {
            bbox: Aabb3::new(Vec3::zeros(), Vec3::new(1.0, 1.0, 1.0)),
            cell_size: 1.0,
        }
    }

    fn total_signed_volume(mesh: &SdfMeshedTetMesh) -> f64 {
        mesh.q.signed_volume.iter().sum()
    }

    #[test]
    fn empty_inside_region_returns_empty_mesh_error() {
        // SDF positive at every grid corner → every Kuhn tet is case 0
        // → no sub-tets emitted → EmptyMesh.
        let result = SdfMeshedTetMesh::from_sdf(&AlwaysOutsideSdf, &unit_cube_hints());
        assert_eq!(
            result.expect_err("constant-positive SDF should produce EmptyMesh"),
            MeshingError::EmptyMesh,
        );
    }

    #[test]
    fn nan_sdf_returns_non_finite_value_error() {
        // NaN at the very first sampled corner short-circuits before
        // any mesh state is built. Detector for the early-return path.
        let result = SdfMeshedTetMesh::from_sdf(&NaNSdf, &unit_cube_hints());
        assert_eq!(
            result.expect_err("NaN SDF should produce NonFiniteSdfValue"),
            MeshingError::NonFiniteSdfValue,
        );
    }

    #[test]
    fn all_inside_path_emits_six_kuhn_tets_per_cell() {
        // SDF negative at every grid corner → every Kuhn tet is case
        // 15 (all inside) → emitted verbatim. Single-cell unit cube
        // → 6 sub-tets, total volume = 1.0.
        let mesh = SdfMeshedTetMesh::from_sdf(&AlwaysInsideSdf, &unit_cube_hints())
            .expect("AlwaysInsideSdf on the unit cube should mesh");
        assert_eq!(mesh.n_tets(), 6);
        // 8 grid corners on a single cube; no cuts because no edge
        // crosses zero.
        assert_eq!(mesh.n_vertices(), 8);
        assert!((total_signed_volume(&mesh) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn all_inside_path_every_sub_tet_is_right_handed() {
        // The case-15 path is the trivial verifier of the post-hoc
        // orientation contract: all 6 emitted Kuhn tets are RH by
        // construction (lattice.rs guarantees), and the post-hoc swap
        // never fires. signed_volume strict positivity is the
        // load-bearing assertion downstream III-2 will tighten further.
        let mesh = SdfMeshedTetMesh::from_sdf(&AlwaysInsideSdf, &unit_cube_hints())
            .expect("AlwaysInsideSdf on the unit cube should mesh");
        for &v in &mesh.q.signed_volume {
            assert!(v > EPSILON_VOLUME, "sub-tet has degenerate volume {v:e}");
        }
    }

    #[test]
    fn half_space_clip_volume_matches_analytic_expectation() {
        // HalfSpaceXSdf at x_threshold = 0.5 cuts a unit cube (grid
        // 0..=2 × 0..=2 × 0..=2 with cell_size 0.5) exactly in half.
        // Inside region volume = 0.5. Stress-tests the full MT
        // case-mix at a single planar interface: each Kuhn tet
        // straddling x = 0.5 hits one of the 1-/2-/3-inside cases,
        // and the BTreeMap edge cache must dedup the cut points
        // shared between adjacent cells along the cut plane.
        let hints = MeshingHints {
            bbox: Aabb3::new(Vec3::zeros(), Vec3::new(1.0, 1.0, 1.0)),
            cell_size: 0.5,
        };
        let mesh = SdfMeshedTetMesh::from_sdf(&HalfSpaceXSdf { threshold: 0.5 }, &hints)
            .expect("half-space SDF on the test bbox should mesh");
        let vol: f64 = mesh.q.signed_volume.iter().sum();
        assert!((vol - 0.5).abs() < 1e-12, "expected 0.5, got {vol}");
        // Right-handedness post-filter — strict bound.
        for &v in &mesh.q.signed_volume {
            assert!(v > EPSILON_VOLUME, "sub-tet has degenerate volume {v:e}");
        }
    }

    #[test]
    fn diagonal_edge_cuts_dedup_to_unique_vertex_ids_per_unique_edge() {
        // Direct D-9 detector. Place a sphere SDF at the cube origin
        // with radius `0.5` so cube corner 0 is inside (SDF = -0.5)
        // and corners 1..7 are outside (SDF ∈ [0.5, sqrt(3) - 0.5]).
        // All 6 Kuhn tets in the cube fall into case 1 (only their
        // local v_0 — cube corner 0 — is inside) and emit a corner
        // sub-tet from cube corner 0 to 3 cuts along the cube edges
        // leaving it. The 7 cube edges leaving corner 0 are
        // {0-1, 0-2, 0-3, 0-4, 0-5, 0-6, 0-7}; they include 3
        // axis-aligned (0-1, 0-3, 0-4), 3 face-diagonal (0-2, 0-5,
        // 0-7), and 1 space-diagonal (0-6) edges. The space diagonal
        // is shared by ALL 6 Kuhn tets in the cube.
        //
        // With D-9 correct (sorted-pair-grid-corners): 1 grid-corner
        // vertex + 7 unique cut vertices = 8 total. With a
        // hypothetical (cell, axis) keying regression: only the 3
        // axis-aligned edges dedup; each face diagonal duplicates
        // 2× (one per Kuhn tet sharing it) and the space diagonal
        // duplicates 6× → 1 + 3 + 2 + 2 + 6 + 2 = 16 total. The
        // exact `n_vertices == 8` assertion distinguishes pass-by-
        // correctness from pass-by-coincidence (a buggy keying
        // would visibly inflate the count to 16, not silently match
        // 8).
        let mesh = SdfMeshedTetMesh::from_sdf(&SphereSdf { radius: 0.5 }, &unit_cube_hints())
            .expect("sphere SDF cutting only corner 0 should mesh");
        assert_eq!(mesh.n_tets(), 6);
        assert_eq!(
            mesh.n_vertices(),
            8,
            "expected 1 grid corner + 7 unique cut points; \
             count inflation indicates D-9 (sorted-pair-grid-corners) \
             keying regressed",
        );
    }

    #[test]
    fn quality_metrics_lengths_match_tet_count() {
        let mesh = SdfMeshedTetMesh::from_sdf(&AlwaysInsideSdf, &unit_cube_hints())
            .expect("AlwaysInsideSdf on the unit cube should mesh");
        let n = mesh.n_tets();
        assert_eq!(mesh.q.signed_volume.len(), n);
        assert_eq!(mesh.q.aspect_ratio.len(), n);
        assert_eq!(mesh.q.dihedral_min.len(), n);
        assert_eq!(mesh.q.dihedral_max.len(), n);
    }

    #[test]
    fn equals_structurally_holds_under_repeat_construction() {
        // Two consecutive constructions on the same SDF + hints should
        // produce structurally identical meshes (same n_tets,
        // n_vertices, per-tet vertex IDs in the same order). Local
        // version of the III-1 same-process check; integration test
        // strengthens to bit-equal positions + QualityMetrics fields.
        let hints = MeshingHints {
            bbox: Aabb3::new(Vec3::zeros(), Vec3::new(1.0, 1.0, 1.0)),
            cell_size: 0.5,
        };
        let a = SdfMeshedTetMesh::from_sdf(&HalfSpaceXSdf { threshold: 0.5 }, &hints)
            .expect("half-space SDF on the test bbox should mesh");
        let b = SdfMeshedTetMesh::from_sdf(&HalfSpaceXSdf { threshold: 0.5 }, &hints)
            .expect("half-space SDF on the test bbox should mesh");
        assert!(a.equals_structurally(&b));
    }

    #[test]
    fn sphere_inside_bbox_produces_non_empty_mesh() {
        // SphereSdf radius 0.1 inside a [-0.12, 0.12]^3 bbox at
        // cell_size 0.04 (3 cells/axis, 4³ = 64 corners). Tractable
        // smoke test for the canonical sphere scene; III-1 integration
        // tightens to cell_size 0.02.
        let mesh = SdfMeshedTetMesh::from_sdf(
            &SphereSdf { radius: 0.1 },
            &MeshingHints {
                bbox: Aabb3::new(Vec3::new(-0.12, -0.12, -0.12), Vec3::new(0.12, 0.12, 0.12)),
                cell_size: 0.04,
            },
        )
        .expect("sphere SDF inside bbox should mesh");
        assert!(mesh.n_tets() > 0);
        // Right-handedness post-filter — every sub-tet survives.
        for &v in &mesh.q.signed_volume {
            assert!(v > EPSILON_VOLUME);
        }
        // Sanity: total volume bounded by enclosing cube (0.24³) and
        // greater than zero.
        let vol: f64 = mesh.q.signed_volume.iter().sum();
        assert!(vol > 0.0);
        assert!(vol < 0.24_f64.powi(3));
    }
}
