//! Uniform-cubic lattice generator with 6-tet Kuhn decomposition.
//!
//! Phase 3 commit 4 deliverable per scope memo §8 step 4. Walks the
//! [`MeshingHints`] bbox on a uniform cubic grid and emits 6 Kuhn-style
//! tets per cell, all sharing the canonical (corner 0 → corner 6)
//! space diagonal so face-shared triangulations align across adjacent
//! cells (memo §2 + §3 Decision A; matches cf-design's
//! `CORNER_OFFSETS` convention at `design/cf-design/src/mesher.rs:186-195`).
//! 5-tet decomposition rejected because adjacent cubes' orientations
//! would flip per [Wikipedia: Marching tetrahedra](https://en.wikipedia.org/wiki/Marching_tetrahedra).
//!
//! Determinism: outer iteration is z-major-y-then-x (mirroring
//! cf-design's marching-cubes order); within each cell the 6 Kuhn
//! tets are emitted in a fixed per-cell order. No SDF, no MT-clip —
//! those land in commit 5.

use super::MeshingHints;
use crate::{Vec3, VertexId};

/// Integer grid-cell index inside the lattice.
///
/// `ix` / `iy` / `iz` are cell coordinates with each axis ranging
/// `0..n_axis`, where `n_axis` is the number of cells along that axis.
#[derive(Clone, Copy, Debug)]
pub(super) struct LatticeCell {
    pub(super) ix: usize,
    pub(super) iy: usize,
    pub(super) iz: usize,
}

/// Cell-local corner offsets, matching cf-design's `CORNER_OFFSETS`
/// convention (`design/cf-design/src/mesher.rs:186-195`). Corner 0 →
/// corner 6 is the canonical (0,0,0) → (1,1,1) space diagonal that all
/// 6 Kuhn tets pivot around, so adjacent cells agree on face
/// triangulations. Shared with the SDF mesher so the meshing path
/// emits tets in the same per-cell vertex ordering as the lattice
/// generator.
pub(super) const CORNER_OFFSETS: [(usize, usize, usize); 8] = [
    (0, 0, 0), // corner 0
    (1, 0, 0), // corner 1
    (1, 1, 0), // corner 2
    (0, 1, 0), // corner 3
    (0, 0, 1), // corner 4
    (1, 0, 1), // corner 5
    (1, 1, 1), // corner 6
    (0, 1, 1), // corner 7
];

/// Six Kuhn-decomposition tets per cube, indexed by cell-local corner
/// ID. Every tet contains corners 0 and 6 — the canonical
/// (0,0,0)→(1,1,1) space diagonal — so adjacent cells agree on the
/// face triangulation along every shared face. All 6 entries are
/// right-handed (positive signed volume): for the three permutations
/// of (x,y,z) with sign(σ) = -1, the two intermediate vertices are
/// swapped to flip orientation back to right-handed without altering
/// the per-cell tet decomposition. Shared with the SDF mesher so its
/// MT-clip walks the same Kuhn tets in the same per-cell order.
pub(super) const KUHN_TETS: [[u8; 4]; 6] = [
    // sign(σ) = +1, identity (x,y,z): 0 → +x → +xy → 6.
    [0, 1, 2, 6],
    // sign(σ) = -1, (x,z,y): 0 → +x → +xz → 6, intermediates swapped.
    [0, 5, 1, 6],
    // sign(σ) = -1, (y,x,z): 0 → +y → +xy → 6, intermediates swapped.
    [0, 2, 3, 6],
    // sign(σ) = +1, (y,z,x): 0 → +y → +yz → 6.
    [0, 3, 7, 6],
    // sign(σ) = +1, (z,x,y): 0 → +z → +xz → 6.
    [0, 4, 5, 6],
    // sign(σ) = -1, (z,y,x): 0 → +z → +yz → 6, intermediates swapped.
    [0, 7, 4, 6],
];

/// Build the full lattice's tet decomposition for `hints`.
///
/// Returns `(positions, tets)` where `positions` are world-space
/// grid-corner points indexed by [`VertexId`] and `tets` are 4-vertex
/// indices for each emitted tet. Cell counts along each axis are
/// `((max - min) / cell_size).round().max(1) as usize`; the lattice
/// extent runs from `bbox.min` to `bbox.min + (nx, ny, nz) *
/// cell_size`. Outer iteration order is z-major-y-then-x; each cell
/// emits 6 Kuhn tets in fixed per-cell order.
///
/// Caller invariant: `bbox.min[i] <= bbox.max[i]` componentwise and
/// `cell_size > 0`. Phase 3 does not validate per memo Decision H;
/// degenerate inputs produce a 1-cell lattice rather than panicking.
pub(super) fn build_lattice_tets(hints: &MeshingHints) -> (Vec<Vec3>, Vec<[VertexId; 4]>) {
    let cell_size = hints.cell_size;
    let extent = hints.bbox.max - hints.bbox.min;
    let nx = axis_cell_count(extent.x, cell_size);
    let ny = axis_cell_count(extent.y, cell_size);
    let nz = axis_cell_count(extent.z, cell_size);

    // Vertex ID layout: x-fastest, y-medium, z-slowest. Neighbouring
    // corners along +x have consecutive IDs, matching cf-design's
    // grid indexing convention.
    let stride_x = 1_usize;
    let stride_y = nx + 1;
    let stride_z = (nx + 1) * (ny + 1);

    let n_vertices = stride_z * (nz + 1);
    let mut positions = Vec::with_capacity(n_vertices);
    for iz in 0..=nz {
        for iy in 0..=ny {
            for ix in 0..=nx {
                positions.push(hints.bbox.min + grid_offset(ix, iy, iz, cell_size));
            }
        }
    }

    let mut tets = Vec::with_capacity(nx * ny * nz * 6);
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let cell = LatticeCell { ix, iy, iz };
                let corners = corner_vertex_ids(cell, stride_x, stride_y, stride_z);
                for kuhn in &KUHN_TETS {
                    tets.push([
                        corners[kuhn[0] as usize],
                        corners[kuhn[1] as usize],
                        corners[kuhn[2] as usize],
                        corners[kuhn[3] as usize],
                    ]);
                }
            }
        }
    }
    (positions, tets)
}

// Round-to-nearest is the cleanest convention for inputs that are
// integer multiples of cell_size (the III-1 scene aligns to 12
// cells/axis exactly); for non-aligned bboxes the lattice extent may
// differ from `bbox.max` by at most half a cell_size. The .max(1.0)
// floor makes degenerate / inverted bboxes collapse to a single cell
// rather than zero per Decision H.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn axis_cell_count(extent: f64, cell_size: f64) -> usize {
    (extent / cell_size).round().max(1.0) as usize
}

// Cast loses precision only at ix/iy/iz > 2^53, far beyond any
// plausible lattice scale (Phase 3 canonical scene is 13³ corners).
#[allow(clippy::cast_precision_loss)]
fn grid_offset(ix: usize, iy: usize, iz: usize, cell_size: f64) -> Vec3 {
    Vec3::new(ix as f64, iy as f64, iz as f64) * cell_size
}

// Lattice fits in u32: III-1 canonical scene has 13³ = 2197
// vertices, well below u32::MAX. Phase H may need u64 if a scene
// exceeds 4 G vertices; revisit at that scale.
#[allow(clippy::cast_possible_truncation)]
fn corner_vertex_ids(
    cell: LatticeCell,
    stride_x: usize,
    stride_y: usize,
    stride_z: usize,
) -> [VertexId; 8] {
    let base = cell.iz * stride_z + cell.iy * stride_y + cell.ix * stride_x;
    let mut out = [0_u32; 8];
    for (k, &(dx, dy, dz)) in CORNER_OFFSETS.iter().enumerate() {
        let id = base + dz * stride_z + dy * stride_y + dx * stride_x;
        out[k] = id as u32;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::super::Aabb3;
    use super::*;
    use approx::assert_relative_eq;
    use std::collections::BTreeMap;

    fn cube_bbox(min: f64, max: f64) -> Aabb3 {
        Aabb3::new(Vec3::new(min, min, min), Vec3::new(max, max, max))
    }

    fn signed_volume(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
        // Independent of mesh::quality::compute_metrics so the test's
        // verdict can't agree-by-bug with the kernel under audit.
        let a = p1 - p0;
        let b = p2 - p0;
        let c = p3 - p0;
        a.dot(&b.cross(&c)) / 6.0
    }

    #[test]
    fn single_cell_emits_eight_vertices_and_six_kuhn_tets() {
        let hints = MeshingHints {
            bbox: cube_bbox(0.0, 0.5),
            cell_size: 0.5,
        };
        let (positions, tets) = build_lattice_tets(&hints);
        assert_eq!(positions.len(), 8);
        assert_eq!(tets.len(), 6);
    }

    #[test]
    fn every_tet_in_single_cell_is_right_handed() {
        let hints = MeshingHints {
            bbox: cube_bbox(0.0, 0.5),
            cell_size: 0.5,
        };
        let (positions, tets) = build_lattice_tets(&hints);
        for [a, b, c, d] in &tets {
            let v = signed_volume(
                positions[*a as usize],
                positions[*b as usize],
                positions[*c as usize],
                positions[*d as usize],
            );
            assert!(v > 0.0, "left-handed Kuhn tet emitted: v = {v}");
        }
    }

    #[test]
    fn every_tet_in_multi_cell_lattice_is_right_handed() {
        let hints = MeshingHints {
            bbox: cube_bbox(-0.05, 0.15),
            cell_size: 0.05,
        };
        let (positions, tets) = build_lattice_tets(&hints);
        for [a, b, c, d] in &tets {
            let v = signed_volume(
                positions[*a as usize],
                positions[*b as usize],
                positions[*c as usize],
                positions[*d as usize],
            );
            assert!(v > 0.0, "left-handed Kuhn tet emitted: v = {v}");
        }
    }

    #[test]
    fn six_kuhn_tets_per_cell_share_canonical_space_diagonal_endpoints() {
        // Multi-cell lattice — for each cell, the 6 emitted tets all
        // share their first (cell-local corner 0) and last (cell-local
        // corner 6) global VIDs. This is the canonical (corner 0 →
        // corner 6) space-diagonal property the Kuhn decomposition
        // pivots around; without it, adjacent cells could disagree on
        // face triangulations.
        let hints = MeshingHints {
            bbox: cube_bbox(0.0, 0.2),
            cell_size: 0.1,
        };
        let (_positions, tets) = build_lattice_tets(&hints);
        for cell_tets in tets.chunks(6) {
            let v_start = cell_tets[0][0];
            let v_end = cell_tets[0][3];
            for tet in cell_tets {
                assert_eq!(
                    tet[0], v_start,
                    "cell-local corner 0 (canonical diagonal start) inconsistent across cell's 6 Kuhn tets",
                );
                assert_eq!(
                    tet[3], v_end,
                    "cell-local corner 6 (canonical diagonal end) inconsistent across cell's 6 Kuhn tets",
                );
            }
        }
    }

    #[test]
    fn full_lattice_volume_equals_bbox_volume() {
        // 2x2x2 cells with cell_size 0.1 → 8 cubes × cell_size³ = 0.008.
        let hints = MeshingHints {
            bbox: cube_bbox(0.0, 0.2),
            cell_size: 0.1,
        };
        let (positions, tets) = build_lattice_tets(&hints);
        let total: f64 = tets
            .iter()
            .map(|[a, b, c, d]| {
                signed_volume(
                    positions[*a as usize],
                    positions[*b as usize],
                    positions[*c as usize],
                    positions[*d as usize],
                )
            })
            .sum();
        assert_relative_eq!(total, 0.008, epsilon = 1e-15);
    }

    #[test]
    fn non_cubic_lattice_volume_and_counts() {
        // 3x2x4 cells × cell_size³: independent axis counts.
        let hints = MeshingHints {
            bbox: Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.15, 0.10, 0.20)),
            cell_size: 0.05,
        };
        let (positions, tets) = build_lattice_tets(&hints);
        let total: f64 = tets
            .iter()
            .map(|[a, b, c, d]| {
                signed_volume(
                    positions[*a as usize],
                    positions[*b as usize],
                    positions[*c as usize],
                    positions[*d as usize],
                )
            })
            .sum();
        // 3 * 2 * 4 * 0.05³ = 24 * 1.25e-4 = 3e-3.
        assert_relative_eq!(total, 0.003, epsilon = 1e-14);
        assert_eq!(tets.len(), 3 * 2 * 4 * 6);
        assert_eq!(positions.len(), 4 * 3 * 5);
    }

    #[test]
    fn face_shared_triangulations_align_in_2x2x2_lattice() {
        // Manifold/watertight property: in a 2x2x2 Kuhn lattice,
        // exactly 48 triangles lie on the bbox surface (count 1) and
        // 72 are interior (count 2 — either shared between 2 tets in
        // the same cube OR shared between 2 cubes meeting at a cube-
        // face). If adjacent cells emitted different diagonals on a
        // shared cube-face, those 24 cube-shared triangles would
        // split into 48 cube-disjoint triangles each at count 1,
        // lifting count-1 from 48 to 96 — so `count_1 == 48` is the
        // precise alignment detector. A weaker `count == 1 || count
        // == 2` assertion would pass under misalignment and fail to
        // gate Decision A's load-bearing claim.
        //
        // Counts: 8 cubes × 6 tets × 4 faces = 192 face slots.
        // Outer: 6 bbox sides × 2×2 cube-faces × 2 tris = 48 (count
        // 1). Cube-shared: 3 axis planes × 2×2 cube-faces × 2 tris
        // = 24 (count 2). Cube-internal: 8 cubes × 6 internal
        // tet-faces = 48 (count 2). Total: 120 unique tris; 48*1 +
        // 72*2 = 192 ✓.
        let hints = MeshingHints {
            bbox: cube_bbox(0.0, 0.2),
            cell_size: 0.1,
        };
        let (_positions, tets) = build_lattice_tets(&hints);
        let mut counts: BTreeMap<[VertexId; 3], usize> = BTreeMap::new();
        for [a, b, c, d] in &tets {
            for face in &[[*a, *b, *c], [*a, *b, *d], [*a, *c, *d], [*b, *c, *d]] {
                let mut key = *face;
                key.sort_unstable();
                *counts.entry(key).or_insert(0) += 1;
            }
        }
        let count_1 = counts.values().filter(|&&c| c == 1).count();
        let count_2 = counts.values().filter(|&&c| c == 2).count();
        let count_other = counts.values().filter(|&&c| c != 1 && c != 2).count();
        assert_eq!(count_other, 0, "every face must appear 1 or 2 times");
        assert_eq!(
            count_1, 48,
            "expected 48 boundary tris in 2x2x2 lattice; \
             excess count-1 entries indicate face-shared diagonals \
             disagreeing between adjacent cells",
        );
        assert_eq!(
            count_2, 72,
            "expected 72 interior tris each shared between 2 tets",
        );
    }

    #[test]
    fn iteration_order_is_z_major_y_then_x() {
        // 3x1x1 lattice, 18 tets total. Cell (ix=0) emits tets 0..6,
        // (ix=1) emits 6..12, (ix=2) emits 12..18 — proves x is the
        // innermost (fastest-varying) axis in the cell loop.
        let hints = MeshingHints {
            bbox: Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.3, 0.1, 0.1)),
            cell_size: 0.1,
        };
        let (positions, tets) = build_lattice_tets(&hints);
        // Each cell-block's first tet has cell-local corner 0 at
        // (ix * cell_size, 0, 0) — the cell's minimum corner.
        let expected_xs = [0.0_f64, 0.1, 0.2];
        for (ix, &expected_x) in expected_xs.iter().enumerate() {
            let first_tet = tets[ix * 6];
            let corner_0_position = positions[first_tet[0] as usize];
            let expected = Vec3::new(expected_x, 0.0, 0.0);
            assert_relative_eq!(corner_0_position, expected, epsilon = 1e-15);
        }
    }

    #[test]
    fn iii_1_canonical_scene_produces_expected_lattice_size() {
        // Memo §1 III-1 scene: bbox = [-0.12, 0.12]^3, cell_size 0.02
        // → 12 cells/axis, 13^3 = 2197 vertices, 12^3 * 6 = 10368
        // tets. Sanity check the round-to-nearest cell count handles
        // integer-multiple inputs without drift.
        let hints = MeshingHints {
            bbox: cube_bbox(-0.12, 0.12),
            cell_size: 0.02,
        };
        let (positions, tets) = build_lattice_tets(&hints);
        assert_eq!(positions.len(), 13 * 13 * 13);
        assert_eq!(tets.len(), 12 * 12 * 12 * 6);
    }
}
