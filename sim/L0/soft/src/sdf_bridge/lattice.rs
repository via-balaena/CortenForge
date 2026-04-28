//! BCC (body-centred cubic) lattice generator for the SDF→tet bridge.
//!
//! Produces the Sommerville-tet tessellation of the BCC lattice
//! `BCC = Z³ ∪ (Z³ + (½, ½, ½))` within a caller-supplied bounding
//! box, scaled by [`MeshingHints::cell_size`]. References:
//! Labelle-Shewchuk SIGGRAPH 2007 §3 and Figure 2; Sommerville [1923].
//!
//! Each cube of the cubic sublattice emits **12 owned Sommerville
//! tetrahedra** toward its `+x`, `+y`, `+z` face-adjacent neighbours.
//! Canonical lex-smaller-cube-centre ownership avoids duplicate
//! emission across face-adjacent cubes; `-x`, `-y`, `-z` directions
//! are emitted by the corresponding negative neighbour. 24 Sommerville
//! tets touch each cube counting both directions; ownership halves to
//! 12 per cube unique.
//!
//! Each Sommerville tet has edge lengths `a` (long, axis-aligned) and
//! `a · √3/2` (short, diagonal), with dihedral angles 60° (at the four
//! short edges) and 90° (at the two long edges), where `a = cell_size`.
//! The four vertices are 2 odd (cube centres in the two face-adjacent
//! cubes, joined by an axis-aligned long edge) plus 2 even (cube
//! corners on the shared face, joined by a perpendicular axis-aligned
//! long edge); the four short cross-edges connect each odd vertex to
//! each even vertex.
//!
//! Vertex IDs are assigned in `(sublattice_tag, k, j, i)` order — even
//! sublattice first (`tag = 0`), then odd (`tag = 1`); within each, `k`
//! changes slowest and `i` fastest. This matches scope-memo Decision M
//! D-11's deterministic iteration order for the warp step (commit 5).

use crate::Vec3;
use crate::mesh::VertexId;

use super::MeshingHints;

/// Lattice index for a BCC vertex.
///
/// `BCC = Z³ ∪ (Z³ + (½, ½, ½))` per Labelle-Shewchuk SIGGRAPH 2007 §3
/// (Figure 2). The integer triple `(i, j, k)` indexes both sublattices;
/// `sublattice_tag` disambiguates them. Lattice points land at
/// world-space positions:
///
/// - `(i, j, k) · cell_size` for `sublattice_tag = 0` (even — cube
///   corner);
/// - `((i + ½), (j + ½), (k + ½)) · cell_size` for `sublattice_tag = 1`
///   (odd — cube centre).
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct BccVertexId {
    /// Lattice integer index along x.
    pub i: i32,
    /// Lattice integer index along y.
    pub j: i32,
    /// Lattice integer index along z.
    pub k: i32,
    /// Sublattice tag — `0` = even (cube corner), `1` = odd (cube
    /// centre). Constructors inside this module never produce other
    /// values.
    pub sublattice_tag: u8,
}

/// One tet vertex's offset relative to a cube's lower-corner integer
/// indices `(cx, cy, cz)`, packed for table use.
#[derive(Clone, Copy)]
struct TetVertexOffset {
    di: i32,
    dj: i32,
    dk: i32,
    sublattice_tag: u8,
}

impl TetVertexOffset {
    const fn new(di: i32, dj: i32, dk: i32, sublattice_tag: u8) -> Self {
        Self {
            di,
            dj,
            dk,
            sublattice_tag,
        }
    }
}

/// 12 Sommerville tets emitted by each cube — 4 per +x / +y / +z
/// face-adjacent neighbour direction. Each entry is canonicalised for
/// right-handedness (`signed_volume > 0` strictly, verified by
/// `every_tet_has_strictly_positive_signed_volume` below).
///
/// Layout per entry: vertex 0 is the cube's own centre `(0,0,0,1)`,
/// vertex 1 is the neighbour cube's centre, vertices 2 + 3 are the two
/// even corners on the shared face. Vertex 2 / 3 ordering canonicalised
/// so the 4-vertex tet has positive signed volume `(p1 - p0) · ((p2 -
/// p0) × (p3 - p0)) / 6 > 0`.
const BCC_TETS: [[TetVertexOffset; 4]; 12] = {
    use TetVertexOffset as V;
    [
        // +x direction (O = (1, 0, 0, tag=1)). Shared face at
        // x = cx + 1; long even-edges on that face are 2 along ±y
        // (at z = cz, cz+1) and 2 along ±z (at y = cy, cy+1).
        [
            V::new(0, 0, 0, 1),
            V::new(1, 0, 0, 1),
            V::new(1, 0, 0, 0),
            V::new(1, 1, 0, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(1, 0, 0, 1),
            V::new(1, 1, 1, 0),
            V::new(1, 0, 1, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(1, 0, 0, 1),
            V::new(1, 0, 1, 0),
            V::new(1, 0, 0, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(1, 0, 0, 1),
            V::new(1, 1, 0, 0),
            V::new(1, 1, 1, 0),
        ],
        // +y direction (O = (0, 1, 0, tag=1)).
        [
            V::new(0, 0, 0, 1),
            V::new(0, 1, 0, 1),
            V::new(1, 1, 0, 0),
            V::new(0, 1, 0, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(0, 1, 0, 1),
            V::new(0, 1, 1, 0),
            V::new(1, 1, 1, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(0, 1, 0, 1),
            V::new(0, 1, 0, 0),
            V::new(0, 1, 1, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(0, 1, 0, 1),
            V::new(1, 1, 1, 0),
            V::new(1, 1, 0, 0),
        ],
        // +z direction (O = (0, 0, 1, tag=1)).
        [
            V::new(0, 0, 0, 1),
            V::new(0, 0, 1, 1),
            V::new(0, 0, 1, 0),
            V::new(1, 0, 1, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(0, 0, 1, 1),
            V::new(1, 1, 1, 0),
            V::new(0, 1, 1, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(0, 0, 1, 1),
            V::new(0, 1, 1, 0),
            V::new(0, 0, 1, 0),
        ],
        [
            V::new(0, 0, 0, 1),
            V::new(0, 0, 1, 1),
            V::new(1, 0, 1, 0),
            V::new(1, 1, 1, 0),
        ],
    ]
};

/// Body-centred cubic lattice tessellated into Sommerville tetrahedra
/// over a caller-supplied bounding box.
///
/// `positions` and `tets` form a complete pair suitable for a
/// `Mesh`-trait impl: every tet has strictly positive signed volume by
/// construction (each entry of [`BCC_TETS`] is canonicalised), and the
/// geometry invariants (edge lengths `cell_size` and `cell_size · √3/2`;
/// dihedrals 60° and 90°) are unit-tested in this module.
#[derive(Clone, Debug)]
pub struct BccLattice {
    /// Vertex positions in world units (metres). Length `2 · nx · ny ·
    /// nz`. Even sublattice first (IDs `0..n_per_sublattice`), then odd
    /// (IDs `n_per_sublattice..2 · n_per_sublattice`); within each
    /// sublattice, IDs are assigned in `(k, j, i)` order with `k`
    /// slowest.
    pub positions: Vec<Vec3>,
    /// Per-tet 4-vertex incidence, indexed by sequential `TetId`.
    /// Right-handed by construction (positive signed volume for every
    /// entry).
    pub tets: Vec<[VertexId; 4]>,
    /// Inclusive-low integer x-index of the lattice extent.
    pub i_min: i32,
    /// Inclusive-low integer y-index.
    pub j_min: i32,
    /// Inclusive-low integer z-index.
    pub k_min: i32,
    /// Vertex count along x. Cube count along x is `nx - 1`.
    pub nx: u32,
    /// Vertex count along y.
    pub ny: u32,
    /// Vertex count along z.
    pub nz: u32,
    /// Lattice spacing in world units (metres). Forwarded from
    /// [`MeshingHints::cell_size`].
    pub cell_size: f64,
}

impl BccLattice {
    /// Build the Sommerville-tet tessellation of the BCC lattice
    /// spanning `hints.bbox` at spacing `hints.cell_size`.
    ///
    /// The cube range is `[i_min, i_max - 1]` inclusive along each
    /// axis, where `i_min = floor(bbox.min.x / cell_size)` and `i_max =
    /// ceil(bbox.max.x / cell_size)` (and similarly for y, z). Vertices
    /// span the inclusive range `[i_min, i_max]` in each axis; the
    /// rightmost cube along each axis emits tets that reference centres
    /// at `i = i_max` (resp. `j_max`, `k_max`), so the vertex range is
    /// one wider than the cube range along each axis.
    ///
    /// # Panics
    ///
    /// Panics if `cell_size <= 0`, if `bbox.min` is not componentwise
    /// `≤ bbox.max`, or if the resulting cube count along any axis is
    /// zero (degenerate bbox tighter than `cell_size` and exactly
    /// aligned to integer multiples of `cell_size`). Callers must
    /// supply bounding boxes that yield at least one cube along every
    /// axis; Phase 3's invariant tests use `bbox` strictly containing
    /// the SDF support, so the assertion never fires under normal use.
    // Casts are bounded by the bbox / cell_size ratio; sim-soft's
    // canonical Phase 3 sphere parameters yield i32-safe ranges (≪
    // 2³¹). Pathological inputs trip the cube-count assertion below
    // before any out-of-range arithmetic; widening for production
    // safety would require checked-cast machinery disproportionate to
    // the read surface.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    #[must_use]
    pub fn new(hints: &MeshingHints) -> Self {
        assert!(
            hints.cell_size > 0.0,
            "cell_size must be > 0; got {}",
            hints.cell_size,
        );
        let bb = &hints.bbox;
        assert!(
            bb.min.x <= bb.max.x && bb.min.y <= bb.max.y && bb.min.z <= bb.max.z,
            "bbox.min must be componentwise ≤ bbox.max",
        );

        let cell = hints.cell_size;
        let i_min = (bb.min.x / cell).floor() as i32;
        let j_min = (bb.min.y / cell).floor() as i32;
        let k_min = (bb.min.z / cell).floor() as i32;
        let i_max = (bb.max.x / cell).ceil() as i32;
        let j_max = (bb.max.y / cell).ceil() as i32;
        let k_max = (bb.max.z / cell).ceil() as i32;

        // Vertex range is `[i_min, i_max]` inclusive (one wider than
        // the cube range along each axis); the rightmost cube
        // (cx = i_max - 1) references its +x neighbour's centre at
        // i = i_max etc.
        assert!(
            i_max > i_min && j_max > j_min && k_max > k_min,
            "bbox + cell_size yields zero cubes along some axis (bbox.min={:?}, bbox.max={:?}, cell_size={})",
            bb.min,
            bb.max,
            cell,
        );

        // i_max > i_min ⇒ (i_max - i_min + 1) ≥ 2 ⇒ positive i32 ⇒
        // safe to cast to u32. Same reasoning for ny, nz.
        let nx = (i_max - i_min + 1) as u32;
        let ny = (j_max - j_min + 1) as u32;
        let nz = (k_max - k_min + 1) as u32;
        let n_per_sublattice = (nx as usize) * (ny as usize) * (nz as usize);

        let mut positions = Vec::with_capacity(2 * n_per_sublattice);
        // Vertex order: even sublattice first, then odd; within each
        // sublattice (k, j, i) with k slowest, i fastest. Sequential
        // pushes implicitly assign VertexIds matching D-11.
        for tag in 0u8..=1 {
            let half = if tag == 0 { 0.0 } else { 0.5 };
            for kk in 0..nz {
                for jj in 0..ny {
                    for ii in 0..nx {
                        // Stay in f64 throughout — `i_min + ii` is
                        // representable exactly because both fit in
                        // i32 by construction, and f64 addition is
                        // exact below `2^53`.
                        positions.push(Vec3::new(
                            (f64::from(i_min) + f64::from(ii) + half) * cell,
                            (f64::from(j_min) + f64::from(jj) + half) * cell,
                            (f64::from(k_min) + f64::from(kk) + half) * cell,
                        ));
                    }
                }
            }
        }

        let n_cubes_x = nx - 1;
        let n_cubes_y = ny - 1;
        let n_cubes_z = nz - 1;
        let n_tets = 12 * (n_cubes_x as usize) * (n_cubes_y as usize) * (n_cubes_z as usize);
        let mut tets: Vec<[VertexId; 4]> = Vec::with_capacity(n_tets);

        // Cube iteration: (cz, cy, cx) outer-to-inner. Per cube, emit
        // 12 tets in BCC_TETS order. Lookups via the closed-form
        // `vertex_id_in_range` are total because every offset in
        // BCC_TETS lies within `{0, 1}³` of `(cube_i, cube_j, cube_k)`
        // and the cube range is one short of the vertex range along
        // each axis.
        for cz in 0..n_cubes_z {
            for cy in 0..n_cubes_y {
                for cx in 0..n_cubes_x {
                    for tet_template in &BCC_TETS {
                        let mut tet_vids: [VertexId; 4] = [0; 4];
                        for (slot, off) in tet_template.iter().enumerate() {
                            // off.di / off.dj / off.dk are 0 or 1, so
                            // (cx + off.di) etc. fits in u32.
                            let i_off = cx + off.di as u32;
                            let j_off = cy + off.dj as u32;
                            let k_off = cz + off.dk as u32;
                            tet_vids[slot] = vertex_id_in_range(
                                i_off,
                                j_off,
                                k_off,
                                off.sublattice_tag,
                                nx,
                                ny,
                                nz,
                            );
                        }
                        tets.push(tet_vids);
                    }
                }
            }
        }

        Self {
            positions,
            tets,
            i_min,
            j_min,
            k_min,
            nx,
            ny,
            nz,
            cell_size: cell,
        }
    }

    /// World-space position of an arbitrary BCC lattice vertex,
    /// regardless of whether it lies inside this lattice's vertex
    /// range.
    ///
    /// Even-sublattice (`tag = 0`) vertex at `(i, j, k) · cell_size`;
    /// odd-sublattice (`tag = 1`) vertex at `((i + ½), (j + ½), (k +
    /// ½)) · cell_size`.
    #[must_use]
    pub fn position_of(&self, bcc: BccVertexId) -> Vec3 {
        let half = if bcc.sublattice_tag == 0 { 0.0 } else { 0.5 };
        Vec3::new(
            (f64::from(bcc.i) + half) * self.cell_size,
            (f64::from(bcc.j) + half) * self.cell_size,
            (f64::from(bcc.k) + half) * self.cell_size,
        )
    }

    /// Sequential `VertexId` for a given BCC index, or `None` if the
    /// index falls outside this lattice's vertex range or its
    /// `sublattice_tag` is greater than `1`.
    ///
    /// Used by commit 5's warp step to walk the 14 incident edges of
    /// every lattice vertex; out-of-range neighbours (lattice boundary)
    /// come back as `None` and are skipped.
    #[must_use]
    pub fn vertex_id_of(&self, bcc: BccVertexId) -> Option<VertexId> {
        if bcc.sublattice_tag > 1 {
            return None;
        }
        // Reject negative offsets via try_from; reject above-range via
        // the `>= self.n_` checks. Both bounds are required for the
        // closed-form linearisation in `vertex_id_in_range` to land
        // inside `[0, n_per_sublattice)`.
        let i_off = u32::try_from(bcc.i - self.i_min).ok()?;
        let j_off = u32::try_from(bcc.j - self.j_min).ok()?;
        let k_off = u32::try_from(bcc.k - self.k_min).ok()?;
        if i_off >= self.nx || j_off >= self.ny || k_off >= self.nz {
            return None;
        }
        Some(vertex_id_in_range(
            i_off,
            j_off,
            k_off,
            bcc.sublattice_tag,
            self.nx,
            self.ny,
            self.nz,
        ))
    }

    /// Inverse of [`vertex_id_of`]: recovers the [`BccVertexId`] for a
    /// sequential `VertexId` in this lattice.
    ///
    /// # Panics
    ///
    /// Panics if `vid` is outside the lattice (i.e., `vid >= 2 · nx ·
    /// ny · nz`).
    // i32 widening from the sublattice offsets is safe: each offset is
    // already bounded by `self.nx / ny / nz`, themselves capped at the
    // i32-safe lattice extent established in `BccLattice::new`. The
    // `as i32` cast on `i_off`, `j_off`, `k_off` cannot wrap.
    #[allow(clippy::cast_possible_wrap)]
    #[must_use]
    pub fn bcc_id_of(&self, vid: VertexId) -> BccVertexId {
        let n_per = self.nx * self.ny * self.nz;
        assert!(
            vid < 2 * n_per,
            "VertexId {vid} is outside the lattice (2 · n_per_sublattice = {})",
            2 * n_per,
        );
        let (sublattice_tag, in_layer) = if vid < n_per {
            (0u8, vid)
        } else {
            (1u8, vid - n_per)
        };
        let plane = self.nx * self.ny;
        let k_off = in_layer / plane;
        let rem = in_layer % plane;
        let j_off = rem / self.nx;
        let i_off = rem % self.nx;
        BccVertexId {
            i: (i_off as i32) + self.i_min,
            j: (j_off as i32) + self.j_min,
            k: (k_off as i32) + self.k_min,
            sublattice_tag,
        }
    }
}

/// Closed-form `VertexId` for a lattice point given its already-offset
/// indices. Total over the constructor's index range (validated by
/// `debug_assert!`s); used by [`BccLattice::new`] in the tet-emission
/// inner loop and by [`BccLattice::vertex_id_of`] after the bounds
/// check.
fn vertex_id_in_range(
    i_off: u32,
    j_off: u32,
    k_off: u32,
    tag: u8,
    nx: u32,
    ny: u32,
    nz: u32,
) -> u32 {
    debug_assert!(i_off < nx);
    debug_assert!(j_off < ny);
    debug_assert!(k_off < nz);
    debug_assert!(tag <= 1);
    let n_per = nx * ny * nz;
    let in_layer = k_off * nx * ny + j_off * nx + i_off;
    if tag == 0 { in_layer } else { n_per + in_layer }
}

#[cfg(test)]
mod tests {
    // Tests use bounded sentinel values whose conversions among u32 /
    // i32 / usize are guaranteed in-range by the test harness (the
    // canonical small lattice has 54 positions and `nx = 3`); the
    // explicit `try_from` + `expect` is a runtime safety belt that
    // would never fire under any test invocation.
    #![allow(clippy::expect_used)]
    use super::*;
    use crate::sdf_bridge::Aabb3;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    /// Canonical small lattice: 2 cubes per axis, `cell_size` = 1, bbox
    /// `[0, 2]³`. 8 cubes total → 96 tets; 3³ × 2 = 54 vertices.
    fn small_unit_lattice() -> BccLattice {
        let hints = MeshingHints {
            bbox: Aabb3::new(Vec3::zeros(), Vec3::new(2.0, 2.0, 2.0)),
            cell_size: 1.0,
            material_field: None,
        };
        BccLattice::new(&hints)
    }

    /// Closed-form signed volume — duplicated locally so the test
    /// doesn't depend on `mesh::quality::compute_metrics`'s
    /// already-validated formula via cross-module path.
    fn signed_volume(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
        let m = nalgebra::Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
        m.determinant() / 6.0
    }

    #[test]
    fn small_lattice_has_expected_vertex_and_tet_counts() {
        let l = small_unit_lattice();
        assert_eq!(l.nx, 3);
        assert_eq!(l.ny, 3);
        assert_eq!(l.nz, 3);
        assert_eq!(l.positions.len(), 54);
        assert_eq!(l.tets.len(), 96);
    }

    #[test]
    fn vertex_ids_round_trip_through_bcc_id() {
        let l = small_unit_lattice();
        let n_total = u32::try_from(l.positions.len()).expect("lattice fits in u32");
        for vid in 0..n_total {
            let bcc = l.bcc_id_of(vid);
            let round = l.vertex_id_of(bcc);
            assert_eq!(round, Some(vid), "round-trip failed at vid={vid}");
        }
    }

    #[test]
    fn vertex_id_assignment_matches_iteration_order() {
        let l = small_unit_lattice();
        assert_eq!(
            l.bcc_id_of(0),
            BccVertexId {
                i: l.i_min,
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 0,
            }
        );
        assert_eq!(
            l.bcc_id_of(1),
            BccVertexId {
                i: l.i_min + 1,
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 0,
            }
        );
        let n_per = l.nx * l.ny * l.nz;
        assert_eq!(
            l.bcc_id_of(n_per),
            BccVertexId {
                i: l.i_min,
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 1,
            }
        );
    }

    #[test]
    fn vertex_id_of_out_of_range_returns_none() {
        let l = small_unit_lattice();
        let nx_i32 = i32::try_from(l.nx).expect("nx fits in i32");
        assert_eq!(
            l.vertex_id_of(BccVertexId {
                i: l.i_min - 1,
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 0,
            }),
            None,
        );
        assert_eq!(
            l.vertex_id_of(BccVertexId {
                i: l.i_min + nx_i32,
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 0,
            }),
            None,
        );
        assert_eq!(
            l.vertex_id_of(BccVertexId {
                i: l.i_min,
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 2,
            }),
            None,
        );
    }

    #[test]
    fn position_of_even_corners_align_with_grid() {
        let l = small_unit_lattice();
        assert_relative_eq!(l.positions[0], Vec3::zeros(), epsilon = 1e-15);
        let n_per = (l.nx as usize) * (l.ny as usize) * (l.nz as usize);
        assert_relative_eq!(
            l.positions[n_per],
            Vec3::new(0.5, 0.5, 0.5),
            epsilon = 1e-15,
        );
    }

    #[test]
    fn every_tet_has_strictly_positive_signed_volume() {
        // Right-handedness: BCC_TETS canonicalisation is sound only if
        // every emitted tet has signed_volume > 0 across all 12
        // BCC_TETS entries instantiated at every cube. Catches a
        // regression where any const-table entry has its even-pair
        // ordering flipped.
        let l = small_unit_lattice();
        for (tet_id, tet) in l.tets.iter().enumerate() {
            let p = tet.map(|vid| l.positions[vid as usize]);
            let v = signed_volume(p[0], p[1], p[2], p[3]);
            assert!(v > 0.0, "tet {tet_id} has non-positive signed volume {v}");
            // Cell-size-cubed scaling: each Sommerville tet has volume
            // cell_size³ / 12. With cell_size = 1, every tet has volume
            // 1/12.
            assert_relative_eq!(v, 1.0 / 12.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn every_tet_has_correct_edge_lengths() {
        // Sommerville-tet invariant: 2 long edges of length cell_size
        // (axis-aligned) and 4 short edges of length cell_size · √3/2
        // (diagonal). Catches a regression where any BCC_TETS entry
        // accidentally references a non-Sommerville vertex set.
        let l = small_unit_lattice();
        let long_expected = l.cell_size;
        let short_expected = l.cell_size * (3.0_f64.sqrt() / 2.0);
        let edges_per_tet = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
        for (tet_id, tet) in l.tets.iter().enumerate() {
            let p = tet.map(|vid| l.positions[vid as usize]);
            let mut n_long = 0;
            let mut n_short = 0;
            for (a, b) in edges_per_tet {
                let len = (p[b] - p[a]).norm();
                let is_long = (len - long_expected).abs() < 1e-12;
                let is_short = (len - short_expected).abs() < 1e-12;
                assert!(
                    is_long || is_short,
                    "tet {tet_id} edge ({a},{b}) has length {len}; expected long={long_expected} or short={short_expected}",
                );
                if is_long {
                    n_long += 1;
                } else {
                    n_short += 1;
                }
            }
            assert_eq!(
                n_long, 2,
                "tet {tet_id} has {n_long} long edges (expected 2)"
            );
            assert_eq!(
                n_short, 4,
                "tet {tet_id} has {n_short} short edges (expected 4)"
            );
        }
    }

    #[test]
    fn every_tet_has_correct_dihedral_angles() {
        // Sommerville-tet invariant: 2 dihedrals of 90° (at the two
        // long edges) and 4 dihedrals of 60° (at the four short
        // edges). Per Labelle-Shewchuk paper §3.
        let l = small_unit_lattice();
        let target_60 = std::f64::consts::FRAC_PI_3;
        for (tet_id, tet) in l.tets.iter().enumerate() {
            let p = tet.map(|vid| l.positions[vid as usize]);
            let dihedrals = tet_dihedrals(p[0], p[1], p[2], p[3]);
            let mut n_60 = 0;
            let mut n_90 = 0;
            for (idx, d) in dihedrals.iter().enumerate() {
                let is_60 = (d - target_60).abs() < 1e-12;
                let is_90 = (d - FRAC_PI_2).abs() < 1e-12;
                assert!(
                    is_60 || is_90,
                    "tet {tet_id} dihedral {idx} is {d} rad; expected 60° or 90°",
                );
                if is_60 {
                    n_60 += 1;
                } else {
                    n_90 += 1;
                }
            }
            assert_eq!(
                n_60, 4,
                "tet {tet_id} has {n_60} 60° dihedrals (expected 4)"
            );
            assert_eq!(
                n_90, 2,
                "tet {tet_id} has {n_90} 90° dihedrals (expected 2)"
            );
        }
    }

    #[test]
    fn total_lattice_volume_equals_cube_count_times_cell_volume() {
        // Closed-form check: for an `n_cubes_x × n_cubes_y × n_cubes_z`
        // cube range with cell_size `a`, total Sommerville volume =
        // `(n_cubes) × a³`. Catches missing-tet, double-counted-tet,
        // and misaligned-cube-range bugs in one shot.
        let l = small_unit_lattice();
        let total_vol: f64 = l
            .tets
            .iter()
            .map(|tet| {
                let p = tet.map(|vid| l.positions[vid as usize]);
                signed_volume(p[0], p[1], p[2], p[3])
            })
            .sum();
        let n_cubes = f64::from(l.nx - 1) * f64::from(l.ny - 1) * f64::from(l.nz - 1);
        let expected = n_cubes * l.cell_size.powi(3);
        assert_relative_eq!(total_vol, expected, epsilon = 1e-12);
    }

    #[test]
    fn lattice_scales_with_cell_size_and_offset() {
        // Reproduce the bbox/cell_size combo III-1 will use: r = 0.1 m
        // sphere, bbox = `[-0.12, 0.12]³`, cell_size = 0.02.
        let hints = MeshingHints {
            bbox: Aabb3::new(Vec3::new(-0.12, -0.12, -0.12), Vec3::new(0.12, 0.12, 0.12)),
            cell_size: 0.02,
            material_field: None,
        };
        let l = BccLattice::new(&hints);
        // i_min = floor(-0.12/0.02) = -6; i_max = ceil(0.12/0.02) = 6.
        // nx = i_max - i_min + 1 = 13. n_cubes_x = 12.
        assert_eq!(l.i_min, -6);
        assert_eq!(l.nx, 13);
        assert_eq!(l.tets.len(), 12 * 12 * 12 * 12);
        for (tet_id, tet) in l.tets.iter().enumerate() {
            let p = tet.map(|vid| l.positions[vid as usize]);
            let v = signed_volume(p[0], p[1], p[2], p[3]);
            assert!(v > 0.0, "tet {tet_id} has non-positive volume {v}");
            assert_relative_eq!(v, l.cell_size.powi(3) / 12.0, epsilon = 1e-15);
        }
    }

    #[test]
    fn iteration_order_is_z_major_y_x() {
        // Pin the contract: tets are emitted in (cz, cy, cx) cube
        // order with cx fastest. The first emitted tet's vertex 1
        // (the +x neighbour centre) sits at i = i_min + 1; the first
        // tet of the last cube along x (cube index = n_cubes_x - 1)
        // sits at i = i_min + n_cubes_x.
        let l = small_unit_lattice();
        let first_neighbour = l.bcc_id_of(l.tets[0][1]);
        assert_eq!(
            first_neighbour,
            BccVertexId {
                i: l.i_min + 1,
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 1,
            }
        );
        let n_cubes_x = l.nx - 1;
        let last_in_first_row_idx = 12 * (n_cubes_x as usize - 1);
        let last_neighbour = l.bcc_id_of(l.tets[last_in_first_row_idx][1]);
        assert_eq!(
            last_neighbour,
            BccVertexId {
                i: l.i_min + i32::try_from(n_cubes_x).expect("n_cubes_x fits in i32"),
                j: l.j_min,
                k: l.k_min,
                sublattice_tag: 1,
            }
        );
    }

    /// Closed-form dihedral angles at the 6 edges of a tet. Returned
    /// in edge order `[(0,1), (0,2), (0,3), (1,2), (1,3), (2,3)]`
    /// matching `mesh::quality::dihedral_min_max`.
    fn tet_dihedrals(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> [f64; 6] {
        [
            dihedral(p0, p1, p2, p3),
            dihedral(p0, p2, p1, p3),
            dihedral(p0, p3, p1, p2),
            dihedral(p1, p2, p0, p3),
            dihedral(p1, p3, p0, p2),
            dihedral(p2, p3, p0, p1),
        ]
    }

    fn dihedral(edge_a: Vec3, edge_b: Vec3, opp1: Vec3, opp2: Vec3) -> f64 {
        let e = (edge_b - edge_a).normalize();
        let u = opp1 - edge_a;
        let v = opp2 - edge_a;
        let u_perp = u - e * u.dot(&e);
        let v_perp = v - e * v.dot(&e);
        let cos_theta = u_perp.dot(&v_perp) / (u_perp.norm() * v_perp.norm());
        cos_theta.clamp(-1.0, 1.0).acos()
    }
}
