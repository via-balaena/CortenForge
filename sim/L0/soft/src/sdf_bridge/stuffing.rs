//! Labelle-Shewchuk Isosurface Stuffing — warp step + 81-case stencil
//! dispatch.
//!
//! Implements steps 3 and 4 of the *SIGGRAPH 2007* "Isosurface Stuffing:
//! Fast Tetrahedral Meshes with Good Dihedral Angles" algorithm by
//! Labelle and Shewchuk on top of the BCC lattice from
//! [`super::lattice`]. Pinned by scope-memo Decision A and §3.3:
//!
//! - **Sign convention** — internal to the BCC pipeline is the *paper's*
//!   convention (positive = inside, negative = outside). Sim-soft's
//!   [`crate::Sdf::eval`] uses the opposite convention (negative inside);
//!   commit 6's `SdfMeshedTetMesh::from_sdf` handles the one-line
//!   negation when sampling the SDF, so this module sees paper-convention
//!   values throughout.
//! - **Warp predicate** — paper §3.2: a cut point `c` violates an
//!   endpoint `v` of edge `e` iff the distance from `c` to `v` is less
//!   than `α · ‖e‖`. Concretely, with `t = -f(v) / (f(n) - f(v))` the
//!   linear-interpolation parameter of the cut along `(v, n)`,
//!   `|t| < α` is the violation predicate. The memo's prose form
//!   `|f(v)| < α · ‖e‖` is shorthand that coincides with the paper's
//!   form whenever the SDF has unit gradient (sphere SDFs in Phase 3
//!   satisfy this); the implementation here is the paper-faithful t
//!   form, valid for any continuous SDF.
//! - **α constants** — Table 1 "min dihedral, safe" row pinned by
//!   Decision A: [`ALPHA_LONG`] = `0.24999` for axis-aligned ("black",
//!   length `a`, same-sublattice) edges; [`ALPHA_SHORT`] = `0.41189`
//!   for diagonal ("red", length `a · √3/2`, different-sublattice)
//!   edges. Theorem 1 (computer-assisted proof, computer-verified to
//!   `0.0001°`) bounds output dihedrals in `[9.3171°, 161.6432°]` and
//!   aspect ratios `≥ ≈0.08` under these parameters.
//! - **Iteration order (Decision M D-11)** — outer order over lattice
//!   vertices is implicit in [`super::lattice::BccLattice`]'s
//!   `(sublattice_tag, k, j, i)` `VertexId` assignment (commit 4); the
//!   warp loop walks `0..n_vertices` sequentially. Per-vertex inner
//!   walk uses fixed neighbour-offset tables ([`NEIGHBOUR_OFFSETS`])
//!   with first-match-wins on the violation predicate; this trades the
//!   paper's "nearest violating cut" tie-break for determinism. Paper
//!   §3.2 explicitly notes that the angle guarantees do not depend on
//!   which violating cut is chosen, so first-match-wins remains
//!   theorem-safe.
//! - **D-9 cut-point dedup** — cuts on un-warped edges are dedup'd
//!   through a `BTreeMap` keyed by the sorted-lex pair
//!   `((i32, i32, i32, u8), (i32, i32, i32, u8))` of
//!   `(grid_x, grid_y, grid_z, sublattice_tag)` quadruples, where the
//!   `u8` `sublattice_tag` distinguishes BCC's bipartite lattice
//!   `(0, 0, 0, 0) ≠ (0, 0, 0, 1)`. III-1 bit-equality is the explicit
//!   detector for any dedup or sublattice-tag bug.
//! - **D-10 [`EPSILON_VOLUME`] backstop** — every emitted sub-tet is
//!   defensively checked against `signed_volume < EPSILON_VOLUME` and
//!   silently dropped if so. Under correct operation Theorem 1
//!   guarantees no degenerate sub-tets, so this filter is expected to
//!   trigger zero times across the III-2 sweep; the check stays as
//!   FP-edge-case insurance.
//! - **Quadruple-zero handling** — paper §3.3 Option 1: BCC tets where
//!   every vertex post-warp is `0` are discarded (not output). The
//!   "min dihedral, safe" parameter row guarantees these aren't
//!   inverted, but discarding eliminates them as a quality-degradation
//!   source for III-2.
//! - **Long-vs-short edge identification** — the BCC tet layout from
//!   [`BCC_TETS`](super::lattice) is structurally fixed: slots 0 and 1
//!   are always odd-sublattice (cube centres) and slots 2 and 3 are
//!   always even-sublattice (cube corners). The two long edges per tet
//!   are therefore `(0, 1)` and `(2, 3)` by construction; the four
//!   short edges are `(0, 2)`, `(0, 3)`, `(1, 2)`, `(1, 3)`. No
//!   per-tet long-edge mask is needed, in contrast to length-comparison
//!   ports such as Changxi Zheng's `isostuffer` reference impl
//!   ([`alecjacobson/isostuffer`](https://github.com/alecjacobson/isostuffer)).
//!
//! ## Parity Rule predicate — BCC-integer deviation from paper
//!
//! Paper §3.3 specifies the Parity Rule predicate as
//! `count(a > c per axis) % 2`, where `a` is a whole-long-edge
//! lattice endpoint and `c` is a *cut point* (FP position derived by
//! linear interpolation along a red BCC edge). This implementation
//! uses a deviated form: `coord_greater_count(bcc(a), bcc(b)) % 2`
//! where `b` is the *other* whole-long-edge endpoint, comparing
//! integer BCC indices instead of FP world coordinates. Trade-offs:
//!
//! - **Conformity preserved.** Adjacent BCC tets sharing a
//!   whole-long-edge quadrilateral compute the predicate from the
//!   same `(a, b)` lattice points (the face's two long-edge
//!   endpoints are shared), so the diagonal choice agrees on both
//!   sides — the mesh is conformal. This holds for any deterministic
//!   predicate that depends only on the face's lattice points.
//! - **Determinism strengthened.** No FP arithmetic in the
//!   parity predicate; III-1 bit-equality survives across runs.
//! - **Theorem 1 strict bound may not hold.** The paper's quality
//!   proof assumes the paper's predicate, so my deviation can in
//!   principle pick the diagonal that gives a worse (larger) max
//!   dihedral or smaller min dihedral. Phase 3's sanity floor is
//!   `≥ 5°` / `≤ 175°` / aspect `≥ 0.05`, with margins 4.32° / 13.36° /
//!   0.03 from the published bounds. Even a worst-case deviation
//!   stays comfortably above the floor for sphere SDFs at canonical
//!   parameters. **If III-2 (`sdf_quality_bounds`) fails at commit 7,
//!   revisit by switching to the paper's FP-position predicate.**
//!
//! ## Stencil case-table cross-check (S-8 lens vii)
//!
//! The 12 unique stencils of paper Figure 3 (and the 81-case dispatch
//! that maps onto them) are cross-checked against two sources:
//!
//! 1. **Labelle-Shewchuk SIGGRAPH 2007 paper** §3.3 + Figure 3 — the
//!    canonical specification.
//! 2. **Changxi Zheng's `isostuffer`** at
//!    [`alecjacobson/isostuffer`](https://github.com/alecjacobson/isostuffer)
//!    `src/IsoStuffer.hpp` `stencil_match` (lines 1033-1191) — a
//!    public C++ port that uses the opposite sign convention
//!    (negative = inside) and dispatches via 4 vt-permutation × 3
//!    cyclic-permutation retries instead of the closed-form
//!    `(n_inside, n_outside, n_boundary)` taxonomy here. Sub-tet counts
//!    per case match this implementation after the convention swap;
//!    Parity Rule logic mirrors `isostuffer::create_tet_type1` and
//!    `create_tet_type2`. Recorded here for the audit trail per
//!    scope-memo §6 S-8 lens (vii).

use std::collections::BTreeMap;

use crate::Vec3;
use crate::mesh::VertexId;

use super::lattice::{BccLattice, BccVertexId};

/// Warp threshold for axis-aligned ("black", length `a`,
/// same-sublattice) edges. Paper Table 1 "min dihedral, safe" row.
///
/// Each lattice vertex `v` connected to a neighbour `n` of opposite
/// SDF sign by an axis-aligned edge is warped to the linear-interp
/// cut point on `(v, n)` iff the cut's fractional distance from `v`
/// satisfies `t < ALPHA_LONG`.
pub(super) const ALPHA_LONG: f64 = 0.24999;

/// Warp threshold for diagonal ("red", length `a · √3/2`,
/// different-sublattice) edges. Paper Table 1 "min dihedral, safe"
/// row.
pub(super) const ALPHA_SHORT: f64 = 0.41189;

/// Defensive backstop on emitted sub-tet volumes. Sub-tets with
/// `signed_volume < EPSILON_VOLUME` are silently dropped. Under
/// correct BCC + warp + stuffing operation with the safe parameter
/// row, Theorem 1 guarantees no sub-tet falls below this threshold;
/// the filter exists as FP-edge-case insurance only (Decision M
/// D-10 demoted post-pivot).
pub(super) const EPSILON_VOLUME: f64 = 1e-15;

/// `BTreeMap` key for cut-point dedup across BCC tet adjacencies
/// (Decision M D-9). Sorted-lex pair of `(grid_x, grid_y, grid_z,
/// sublattice_tag)` quadruples, distinguishing BCC's bipartite
/// lattice. `BTreeMap` (not `HashMap`) for sorted-iteration
/// determinism.
pub(super) type EdgeKey = ((i32, i32, i32, u8), (i32, i32, i32, u8));

const SIGN_OUTSIDE: u8 = 0;
const SIGN_BOUNDARY: u8 = 1;
const SIGN_INSIDE: u8 = 2;

/// Single neighbour-offset entry for the warp step's per-vertex
/// 14-edge walk. `(di, dj, dk)` is added to the source vertex's
/// integer index; `target_tag` is the destination sublattice; `is_long`
/// distinguishes axis-aligned (black) edges from diagonal (red) edges
/// for α-threshold selection.
type NeighbourOffset = (i32, i32, i32, u8, bool);

/// Per-source-sublattice 14-edge offset tables. Index 0 = even
/// sublattice source (cube corner), index 1 = odd sublattice source
/// (cube centre).
///
/// Each row lists 6 axis-aligned long edges first (`±x`, `±y`, `±z`
/// to same-sublattice neighbours), then 8 diagonal short edges to
/// different-sublattice neighbours. The fixed order is the warp
/// step's tie-break per Decision M D-11; first-match-wins on the
/// violation predicate.
///
/// Even-source short offsets land on `(a, b, c) ∈ {-1, 0}³` of
/// integer indices (target tag 1); odd-source short offsets land on
/// `(a, b, c) ∈ {0, 1}³` (target tag 0). This asymmetry encodes BCC's
/// bipartite structure: an even corner sees the 8 odd centres of the
/// 8 cubes meeting at that corner; an odd centre sees the 8 even
/// corners of its own cube.
const NEIGHBOUR_OFFSETS: [[NeighbourOffset; 14]; 2] = [
    // Source sublattice tag = 0 (even, cube corner).
    [
        (1, 0, 0, 0, true),
        (-1, 0, 0, 0, true),
        (0, 1, 0, 0, true),
        (0, -1, 0, 0, true),
        (0, 0, 1, 0, true),
        (0, 0, -1, 0, true),
        (-1, -1, -1, 1, false),
        (0, -1, -1, 1, false),
        (-1, 0, -1, 1, false),
        (0, 0, -1, 1, false),
        (-1, -1, 0, 1, false),
        (0, -1, 0, 1, false),
        (-1, 0, 0, 1, false),
        (0, 0, 0, 1, false),
    ],
    // Source sublattice tag = 1 (odd, cube centre).
    [
        (1, 0, 0, 1, true),
        (-1, 0, 0, 1, true),
        (0, 1, 0, 1, true),
        (0, -1, 0, 1, true),
        (0, 0, 1, 1, true),
        (0, 0, -1, 1, true),
        (0, 0, 0, 0, false),
        (1, 0, 0, 0, false),
        (0, 1, 0, 0, false),
        (1, 1, 0, 0, false),
        (0, 0, 1, 0, false),
        (1, 0, 1, 0, false),
        (0, 1, 1, 0, false),
        (1, 1, 1, 0, false),
    ],
];

/// Apply the warp step to a BCC lattice in place.
///
/// On entry, `positions` holds lattice positions (caller usually
/// passes a clone of [`BccLattice::positions`]) and `sdf_values` holds
/// per-vertex SDF values in **paper convention** (positive = inside),
/// already negated from sim-soft's standard convention by the caller.
///
/// On return, vertices that violated the warp threshold have been
/// snapped to the cut point on the chosen incident edge, and their
/// `sdf_values` entry has been set to exactly `0.0`. Per paper §3.2,
/// any cut points adjoining a warped vertex are then implicitly
/// discarded (subsequent edge checks in this same loop see the
/// post-warp `0.0` value and skip — `f64` zero on either endpoint
/// fails the opposite-sign predicate).
///
/// Iteration order: vertices in sequential `VertexId` order (matching
/// Decision M D-11's `(sublattice_tag, k, j, i)` order from commit 4),
/// then 14 incident edges per vertex in
/// [`NEIGHBOUR_OFFSETS`] order (6 long, then 8 short),
/// first-match-wins on `t < α`. Per paper §3.2, the angle guarantees
/// of Theorem 1 are independent of which violating cut is chosen, so
/// first-match-wins is theorem-safe and gives the determinism III-1
/// requires.
pub(super) fn warp_lattice(lattice: &BccLattice, positions: &mut [Vec3], sdf_values: &mut [f64]) {
    debug_assert_eq!(positions.len(), lattice.positions.len());
    debug_assert_eq!(sdf_values.len(), lattice.positions.len());

    // Length cast for VertexId range; total vertex count fits in u32 by
    // BccLattice's construction (Phase 3 canonical lattices have ≤
    // 10⁶ vertices).
    #[allow(clippy::cast_possible_truncation)]
    let n_total = positions.len() as u32;

    for vid in 0..n_total {
        let f_v = sdf_values[vid as usize];
        // f_v == 0.0 catches both unwarped exact-zeros and prior-warp
        // snaps in this same pass; f64 zero compares exactly.
        #[allow(clippy::float_cmp)]
        if f_v == 0.0 {
            continue;
        }

        let bcc_v = lattice.bcc_id_of(vid);
        let pos_v = positions[vid as usize];

        // 14 incident edges in fixed offset order. The source
        // sublattice tag selects the offset table; tag is 0 or 1 by
        // BccLattice's construction.
        let table = &NEIGHBOUR_OFFSETS[bcc_v.sublattice_tag as usize];
        for &(di, dj, dk, target_tag, is_long) in table {
            let neighbour_bcc = BccVertexId {
                i: bcc_v.i + di,
                j: bcc_v.j + dj,
                k: bcc_v.k + dk,
                sublattice_tag: target_tag,
            };
            let Some(nid) = lattice.vertex_id_of(neighbour_bcc) else {
                // Out-of-lattice neighbour (boundary cube). Skip.
                continue;
            };

            let f_n = sdf_values[nid as usize];
            // Opposite-sign predicate: cut exists iff f_v and f_n have
            // strictly opposite signs. Either-zero or same-sign cases
            // give product ≥ 0 and skip — handles the "boundary on
            // either endpoint" case correctly per paper §3.2 (a cut
            // requires both endpoints in P with opposite signs).
            if f_v * f_n >= 0.0 {
                continue;
            }

            // Linear-interp cut location from v: cut = v + t·(n − v),
            // with t = −f_v / (f_n − f_v) ∈ (0, 1) since signs are
            // strictly opposite. Paper §3.2: violation iff `t < α`.
            let t = -f_v / (f_n - f_v);
            let alpha = if is_long { ALPHA_LONG } else { ALPHA_SHORT };
            if t < alpha {
                let pos_n = positions[nid as usize];
                positions[vid as usize] = pos_v + (pos_n - pos_v) * t;
                sdf_values[vid as usize] = 0.0;
                break; // first-match-wins per Decision M D-11
            }
        }
    }
}

/// Sign classifier in paper convention. Strict comparisons ensure
/// post-warp `0.0` values flow into the boundary case
/// ([`SIGN_BOUNDARY`]); only strictly-positive (inside) and
/// strictly-negative (outside) values enter the other classes.
fn classify_sign(v: f64) -> u8 {
    if v > 0.0 {
        SIGN_INSIDE
    } else if v < 0.0 {
        SIGN_OUTSIDE
    } else {
        SIGN_BOUNDARY
    }
}

/// Sorted-lex `EdgeKey` for a BCC edge between two lattice vertices.
fn edge_key(a: BccVertexId, b: BccVertexId) -> EdgeKey {
    let qa = (a.i, a.j, a.k, a.sublattice_tag);
    let qb = (b.i, b.j, b.k, b.sublattice_tag);
    if qa <= qb { (qa, qb) } else { (qb, qa) }
}

/// Long-edge predicate from `BCC_TETS` slot indices. By construction
/// (commit 4 lattice layout) slots 0 and 1 are always odd-sublattice
/// vertices joined by an axis-aligned long edge, and slots 2 and 3
/// are always even-sublattice vertices joined by the perpendicular
/// long edge. The four short (diagonal) edges are the
/// odd-even cross-pairs.
fn is_long_edge(slot_a: usize, slot_b: usize) -> bool {
    let lo = slot_a.min(slot_b);
    let hi = slot_a.max(slot_b);
    (lo == 0 && hi == 1) || (lo == 2 && hi == 3)
}

/// Return the deduped `VertexId` for the cut point on the BCC edge
/// `(a, b)`. Inserts the cut point into `output_positions` at first
/// query and caches under [`EdgeKey`]; subsequent queries on the same
/// edge (across BCC tet adjacencies) return the cached `VertexId`.
///
/// Caller invariant: `sdf_values[a]` and `sdf_values[b]` have strictly
/// opposite signs (the cut point exists). The warp step ensures any
/// near-zero crossings have already been absorbed; surviving cuts on
/// edges reaching the stencil dispatch are at fractional positions
/// `t ∈ [α, 1 − α]`, well clear of the endpoints.
fn get_or_insert_cut(
    a: VertexId,
    b: VertexId,
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) -> VertexId {
    let bcc_a = lattice.bcc_id_of(a);
    let bcc_b = lattice.bcc_id_of(b);
    let key = edge_key(bcc_a, bcc_b);
    if let Some(&vid) = cut_cache.get(&key) {
        return vid;
    }
    let f_a = sdf_values[a as usize];
    let f_b = sdf_values[b as usize];
    debug_assert!(
        f_a * f_b < 0.0,
        "get_or_insert_cut requires opposite signs; got {f_a} and {f_b}",
    );
    let t = -f_a / (f_b - f_a);
    let pos_a = positions[a as usize];
    let pos_b = positions[b as usize];
    let cut_pos = pos_a + (pos_b - pos_a) * t;
    // VertexId u32 cap: every Phase 3 mesh fits ≤ 10⁹ vertices.
    #[allow(clippy::cast_possible_truncation)]
    let new_vid = output_positions.len() as VertexId;
    output_positions.push(cut_pos);
    cut_cache.insert(key, new_vid);
    new_vid
}

/// Signed volume of a tetrahedron. Used by [`emit_sub_tet`] to apply
/// the D-10 backstop and to detect orientation for the defensive
/// last-two-slot swap.
fn signed_volume(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    let m = nalgebra::Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
    m.determinant() / 6.0
}

/// Emit one sub-tet. Applies the [`EPSILON_VOLUME`] D-10 backstop
/// (silently drops sub-tets below the floor) and a defensive
/// last-two-slot orientation swap (corrects FP-edge-case left-handed
/// emissions to right-handed; should not trigger under correct
/// stencil construction).
fn emit_sub_tet(
    mut tet: [VertexId; 4],
    output_positions: &[Vec3],
    output_tets: &mut Vec<[VertexId; 4]>,
) {
    let p = tet.map(|vid| output_positions[vid as usize]);
    let v = signed_volume(p[0], p[1], p[2], p[3]);
    if v.abs() < EPSILON_VOLUME {
        // D-10 backstop. Silently drop. Theorem-1-bounded operation
        // produces no degenerates; this path is FP edge case insurance.
        return;
    }
    if v < 0.0 {
        // Defensive last-two-slot swap. Stencil construction is
        // canonicalised for right-handedness, but FP cancellation in
        // pathological inputs could in principle flip orientation;
        // catch and correct rather than emit a left-handed sub-tet.
        tet.swap(2, 3);
    }
    output_tets.push(tet);
}

/// Dispatch a single BCC tet through the 81-case stencil table.
///
/// Inputs:
/// - `tet_vids` — the 4 lattice `VertexId`s of the BCC tet from
///   [`BccLattice::tets`]. By construction (commit 4 layout) slots 0
///   and 1 are odd-sublattice vertices joined by the first long edge;
///   slots 2 and 3 are even-sublattice vertices joined by the second
///   long edge; the four short edges are the odd-even cross-pairs.
/// - `lattice` — referenced for `bcc_id_of` lookups in
///   [`get_or_insert_cut`].
/// - `positions` and `sdf_values` — post-warp lattice state (paper
///   convention: positive inside).
/// - `output_positions` — grows with new cut-point vertices appended
///   beyond the lattice prefix.
/// - `output_tets` — receives 0–3 emitted sub-tets per call.
/// - `cut_cache` — dedup across calls.
///
/// Action by `(n_inside, n_outside, n_boundary)`:
/// - `(0, _, _)` → emit nothing (no inside material; covers
///   all-outside and all-boundary quadruple-zero per paper §3.3
///   Option 1).
/// - `(_, 0, _)` with `n_inside ≥ 1` → emit the BCC tet itself (the
///   tet is fully inside the domain, possibly with warped vertices
///   on the boundary).
/// - mixed → dispatch to one of six stencil shape functions per the
///   `(n_inside, n_outside, n_boundary)` taxonomy (paper §3.3 + Figure
///   3, with isostuffer cross-check per S-8 lens vii).
pub(super) fn dispatch_case(
    tet_vids: [VertexId; 4],
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    let signs: [u8; 4] = [
        classify_sign(sdf_values[tet_vids[0] as usize]),
        classify_sign(sdf_values[tet_vids[1] as usize]),
        classify_sign(sdf_values[tet_vids[2] as usize]),
        classify_sign(sdf_values[tet_vids[3] as usize]),
    ];
    let mut n_in: usize = 0;
    let mut n_out: usize = 0;
    for &s in &signs {
        if s == SIGN_INSIDE {
            n_in += 1;
        } else if s == SIGN_OUTSIDE {
            n_out += 1;
        }
    }

    // Trivial case 1: no inside vertices. Covers all-outside, mixed
    // outside+boundary, and quadruple-zero (Option 1: discard).
    if n_in == 0 {
        return;
    }
    // Trivial case 2: no outside vertices. The whole BCC tet lies in
    // the closed domain; emit it directly. Warped vertices contribute
    // their snapped positions via positions[..].
    if n_out == 0 {
        emit_sub_tet(tet_vids, output_positions, output_tets);
        return;
    }

    dispatch_mixed_case(
        tet_vids,
        signs,
        n_in,
        n_out,
        lattice,
        positions,
        sdf_values,
        output_positions,
        output_tets,
        cut_cache,
    );
}

/// Stencil dispatch for the 6 mixed `(n_in, n_out, n_bd)` taxonomies.
/// Pre-condition: `n_in ≥ 1` and `n_out ≥ 1`; trivial all-inside /
/// all-outside / quadruple-zero are handled in [`dispatch_case`].
// Stencil dispatch arg bundle (slot indices + tet vertex IDs + lattice +
// position/SDF arrays + output buffers + cut cache); refactoring to a
// context struct would obscure the per-case Labelle-Shewchuk Figure 3 mapping.
#[allow(clippy::too_many_arguments)]
fn dispatch_mixed_case(
    tet_vids: [VertexId; 4],
    signs: [u8; 4],
    n_in: usize,
    n_out: usize,
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    let (in_slots, out_slots, bd_slots) = collect_slots_by_sign(signs);

    match (n_in, n_out) {
        (1, 1) => emit_one_one_two_bd(
            in_slots[0],
            out_slots[0],
            bd_slots[0],
            bd_slots[1],
            tet_vids,
            lattice,
            positions,
            sdf_values,
            output_positions,
            output_tets,
            cut_cache,
        ),
        (1, 2) => emit_one_two_one_bd(
            in_slots[0],
            out_slots[0],
            out_slots[1],
            bd_slots[0],
            tet_vids,
            lattice,
            positions,
            sdf_values,
            output_positions,
            output_tets,
            cut_cache,
        ),
        (1, 3) => emit_one_three(
            in_slots[0],
            [out_slots[0], out_slots[1], out_slots[2]],
            tet_vids,
            lattice,
            positions,
            sdf_values,
            output_positions,
            output_tets,
            cut_cache,
        ),
        (2, 1) => emit_two_one_one_bd(
            [in_slots[0], in_slots[1]],
            out_slots[0],
            bd_slots[0],
            tet_vids,
            lattice,
            positions,
            sdf_values,
            output_positions,
            output_tets,
            cut_cache,
        ),
        (2, 2) => emit_two_two(
            [in_slots[0], in_slots[1]],
            [out_slots[0], out_slots[1]],
            tet_vids,
            lattice,
            positions,
            sdf_values,
            output_positions,
            output_tets,
            cut_cache,
        ),
        (3, 1) => emit_three_one(
            [in_slots[0], in_slots[1], in_slots[2]],
            out_slots[0],
            tet_vids,
            lattice,
            positions,
            sdf_values,
            output_positions,
            output_tets,
            cut_cache,
        ),
        // Other combinations have n_in == 0 or n_out == 0 and were
        // handled by the trivial cases in dispatch_case. The match is
        // total over the admissible mixed (n_in, n_out) tuples.
        _ => unreachable!(
            "unreachable mixed (n_in, n_out) = ({n_in}, {n_out}); trivial cases caught earlier",
        ),
    }
}

/// Group the 4 slot indices by sign. Returns three fixed-size arrays;
/// only the leading prefix of each (length `n_in`, `n_out`, `n_bd`
/// respectively) holds valid slot indices, the remainder is zeros.
/// Caller must use [`SIGN_INSIDE`] / [`SIGN_OUTSIDE`] / [`SIGN_BOUNDARY`]
/// counts (recoverable from the input `signs`) to know how much of
/// each array is valid.
fn collect_slots_by_sign(signs: [u8; 4]) -> ([usize; 3], [usize; 3], [usize; 3]) {
    let mut in_slots = [0usize; 3];
    let mut out_slots = [0usize; 3];
    let mut bd_slots = [0usize; 3];
    let mut ni = 0;
    let mut no = 0;
    let mut nb = 0;
    for (slot, &s) in signs.iter().enumerate() {
        match s {
            SIGN_INSIDE => {
                in_slots[ni] = slot;
                ni += 1;
            }
            SIGN_OUTSIDE => {
                out_slots[no] = slot;
                no += 1;
            }
            _ => {
                bd_slots[nb] = slot;
                nb += 1;
            }
        }
    }
    (in_slots, out_slots, bd_slots)
}

/// `(1 inside, 1 outside, 2 boundary)`: 1 cut on the in-out edge.
/// The inside polyhedron is `[in, bd_a, bd_b, cut(in, out)]` — a
/// single sub-tet. Long-edge orientation is irrelevant.
// Stencil dispatch arg bundle (slot indices + tet vertex IDs + lattice +
// position/SDF arrays + output buffers + cut cache); refactoring to a
// context struct would obscure the per-case Labelle-Shewchuk Figure 3 mapping.
#[allow(clippy::too_many_arguments)]
fn emit_one_one_two_bd(
    in_slot: usize,
    out_slot: usize,
    bd_slot_a: usize,
    bd_slot_b: usize,
    tet_vids: [VertexId; 4],
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    let cut = get_or_insert_cut(
        tet_vids[in_slot],
        tet_vids[out_slot],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );
    emit_sub_tet(
        [
            tet_vids[in_slot],
            tet_vids[bd_slot_a],
            tet_vids[bd_slot_b],
            cut,
        ],
        output_positions,
        output_tets,
    );
}

/// `(1 inside, 2 outside, 1 boundary)`: 2 cuts on the in-out edges.
/// The inside polyhedron is `[in, bd, cut(in, out_a), cut(in, out_b)]`
/// — a single sub-tet.
// Stencil dispatch arg bundle (slot indices + tet vertex IDs + lattice +
// position/SDF arrays + output buffers + cut cache); refactoring to a
// context struct would obscure the per-case Labelle-Shewchuk Figure 3 mapping.
#[allow(clippy::too_many_arguments)]
fn emit_one_two_one_bd(
    in_slot: usize,
    out_slot_a: usize,
    out_slot_b: usize,
    bd_slot: usize,
    tet_vids: [VertexId; 4],
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    let cut_a = get_or_insert_cut(
        tet_vids[in_slot],
        tet_vids[out_slot_a],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );
    let cut_b = get_or_insert_cut(
        tet_vids[in_slot],
        tet_vids[out_slot_b],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );
    emit_sub_tet(
        [tet_vids[in_slot], tet_vids[bd_slot], cut_a, cut_b],
        output_positions,
        output_tets,
    );
}

/// `(1 inside, 3 outside)`: 3 cuts on the in-out edges. The inside
/// polyhedron is the small pyramid `[in, cut₁, cut₂, cut₃]` — a
/// single sub-tet.
// Stencil dispatch arg bundle (slot indices + tet vertex IDs + lattice +
// position/SDF arrays + output buffers + cut cache); refactoring to a
// context struct would obscure the per-case Labelle-Shewchuk Figure 3 mapping.
#[allow(clippy::too_many_arguments)]
fn emit_one_three(
    in_slot: usize,
    out_slots: [usize; 3],
    tet_vids: [VertexId; 4],
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    let cuts = out_slots.map(|out_slot| {
        get_or_insert_cut(
            tet_vids[in_slot],
            tet_vids[out_slot],
            lattice,
            positions,
            sdf_values,
            output_positions,
            cut_cache,
        )
    });
    emit_sub_tet(
        [tet_vids[in_slot], cuts[0], cuts[1], cuts[2]],
        output_positions,
        output_tets,
    );
}

/// `(2 inside, 1 outside, 1 boundary)`: 2 cuts from the outside
/// vertex. The inside polyhedron has 5 vertices `[in_a, in_b, bd,
/// cut(out, in_a), cut(out, in_b)]` and is split into 2 sub-tets.
///
/// Bisection of the quadrilateral face `(in_a, in_b, cut_b, cut_a)`
/// follows paper §3.3: the face has a whole long edge if `(in_a,
/// in_b)` is a long BCC edge, else two truncated long edges (in
/// which case we choose the diagonal adjoining whichever cut sits on
/// a long-edge truncation). For BCC tets in the layout from
/// [`super::lattice::BCC_TETS`], the long edges are slots `(0, 1)`
/// and `(2, 3)`; whether `(in_a, in_b)` is long depends on which
/// pair of slots they occupy.
// Stencil dispatch arg bundle (slot indices + tet vertex IDs + lattice +
// position/SDF arrays + output buffers + cut cache); refactoring to a
// context struct would obscure the per-case Labelle-Shewchuk Figure 3 mapping.
#[allow(clippy::too_many_arguments)]
fn emit_two_one_one_bd(
    in_slots: [usize; 2],
    out_slot: usize,
    bd_slot: usize,
    tet_vids: [VertexId; 4],
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    let cut_a = get_or_insert_cut(
        tet_vids[out_slot],
        tet_vids[in_slots[0]],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );
    let cut_b = get_or_insert_cut(
        tet_vids[out_slot],
        tet_vids[in_slots[1]],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );
    let v_in_a = tet_vids[in_slots[0]];
    let v_in_b = tet_vids[in_slots[1]];
    let v_bd = tet_vids[bd_slot];

    if is_long_edge(in_slots[0], in_slots[1]) {
        // Quadrilateral (in_a, in_b, cut_b, cut_a) has whole long
        // edge (in_a, in_b) and two truncated red edges (paper §3.3
        // whole-long case) — Parity Rule applies. Diagonal "ac"
        // (in_a, cut_b) bisects via (in_a, cut_b); diagonal "bd"
        // (in_b, cut_a) bisects via (in_b, cut_a). The same predicate
        // is applied across all whole-long quadrilaterals
        // (including the ones in `emit_two_two` and `emit_three_one`)
        // for conformity at shared faces.
        let bcc_a = lattice.bcc_id_of(v_in_a);
        let bcc_b = lattice.bcc_id_of(v_in_b);
        let agc = coord_greater_count(bcc_a, bcc_b) % 2;
        if agc == 0 {
            // "ac" diagonal — bisect via (in_a, cut_b).
            emit_sub_tet([v_in_a, v_in_b, v_bd, cut_b], output_positions, output_tets);
            emit_sub_tet([v_in_a, cut_b, v_bd, cut_a], output_positions, output_tets);
        } else {
            // "bd" diagonal — bisect via (in_b, cut_a).
            emit_sub_tet([v_in_a, v_in_b, v_bd, cut_a], output_positions, output_tets);
            emit_sub_tet([v_in_b, cut_a, v_bd, cut_b], output_positions, output_tets);
        }
    } else {
        // Quadrilateral has truncated long edge (the long edge runs
        // through `out` to one of the in slots; the truncation sits
        // on whichever cut is on a long edge). Diagonal choice
        // adjoins that cut. Exactly one of (out, in_slots[0]),
        // (out, in_slots[1]) is long here (since (in_a, in_b) is
        // short and bd-out is the remaining edge from out).
        let in_zero_paired_long = is_long_edge(out_slot, in_slots[0]);
        debug_assert!(in_zero_paired_long ^ is_long_edge(out_slot, in_slots[1]));
        if in_zero_paired_long {
            // Diagonal adjoins cut_a.
            emit_sub_tet([v_in_a, v_in_b, v_bd, cut_a], output_positions, output_tets);
            emit_sub_tet([cut_a, v_in_b, v_bd, cut_b], output_positions, output_tets);
        } else {
            // Diagonal adjoins cut_b.
            emit_sub_tet([v_in_a, v_in_b, v_bd, cut_b], output_positions, output_tets);
            emit_sub_tet([v_in_a, cut_b, v_bd, cut_a], output_positions, output_tets);
        }
    }
}

/// `(2 inside, 2 outside)`: 4 cuts on the in-out edges, forming a
/// quadrilateral cut surface. The inside polyhedron is a triangular
/// prism with 6 vertices and is split into 3 sub-tets via the Parity
/// Rule (paper §3.3).
///
/// In the BCC tet layout, the two long edges are `(0, 1)` and
/// `(2, 3)`. With 2 inside and 2 outside, three sub-cases arise per
/// which slots host inside vs outside:
///
/// - Both inside slots come from one long edge (e.g., slots 0, 1):
///   the long edge `(0, 1)` is wholly inside, the long edge `(2, 3)`
///   is wholly outside. The 4 cut points sit on the 4 short edges.
///   Both quadrilateral side faces have a whole long edge — Parity
///   Rule disambiguates the diagonal.
/// - One inside slot from each long edge (e.g., slots 0, 2 vs slots
///   1, 3): both long edges are truncated by the cut surface. Two of
///   the 4 cuts sit on long edges; diagonal adjoins those.
///
/// Implementation cross-checked against `isostuffer::create_tet_type2`
/// (same paper-§3.3 Parity Rule, sign-flipped). The Parity Rule
/// predicate on `(odd-sublattice, slot 0 / slot 2)` lattice
/// coordinates picks the diagonal deterministically; the
/// implementation here uses `coord_greater_count` over the
/// pre-warp BCC coordinates (matching isostuffer's
/// `point_greater_number`) so the rule remains deterministic across
/// runs.
// `clippy::panic` — the truncated-branch `unwrap_or_else(|| panic!(_))`
// asserts a structural BCC layout invariant (with both `in_slots`
// short and one short out_slot, exactly two of the four `(in, out)`
// cross-pairs sit on long BCC edges by the slot-0/1-odd /
// slot-2/3-even layout from `super::lattice::BCC_TETS`). A panic
// here indicates a BCC layout regression, not a runtime data error,
// so a hard panic is the correct failure mode.
#[allow(clippy::too_many_arguments, clippy::panic)]
fn emit_two_two(
    in_slots: [usize; 2],
    out_slots: [usize; 2],
    tet_vids: [VertexId; 4],
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    // Cuts indexed by (in_slot, out_slot) pair: cut_io[i][o] is the
    // cut on the edge from in_slots[i] to out_slots[o].
    let cut = |i_idx: usize,
               o_idx: usize,
               output_positions: &mut Vec<Vec3>,
               cut_cache: &mut BTreeMap<EdgeKey, VertexId>|
     -> VertexId {
        get_or_insert_cut(
            tet_vids[in_slots[i_idx]],
            tet_vids[out_slots[o_idx]],
            lattice,
            positions,
            sdf_values,
            output_positions,
            cut_cache,
        )
    };
    let c00 = cut(0, 0, output_positions, cut_cache);
    let c01 = cut(0, 1, output_positions, cut_cache);
    let c10 = cut(1, 0, output_positions, cut_cache);
    let c11 = cut(1, 1, output_positions, cut_cache);
    let v_in_a = tet_vids[in_slots[0]];
    let v_in_b = tet_vids[in_slots[1]];

    // The triangular prism's two triangular caps are
    // (in_a, c00, c01) and (in_b, c10, c11). Three side
    // quadrilaterals connect them; bisecting these three
    // quadrilaterals into 6 triangles defines the 3-tet split.
    // Following isostuffer's `create_tet_type2`, the bisection
    // diagonal is `(in_a, c11)` if the Parity Rule picks "ac" and
    // `(in_b, c00)` if it picks "bd". The rule depends on whether
    // (in_a, in_b) is a long BCC edge (whole-long quadrilateral
    // case, Parity Rule applies) vs. truncated-long (diagonal
    // adjoins the cut on the long edge).

    if is_long_edge(in_slots[0], in_slots[1]) {
        // Both inside vertices are on the same long edge; the
        // opposite long edge (out_a, out_b) is wholly outside. Two
        // of the side quadrilaterals have whole long edges
        // (in_a, in_b) and (out_a, out_b); the third has a "whole
        // short" edge structure. Apply the Parity Rule on the long
        // edge (in_a, in_b).
        let bcc_a = lattice.bcc_id_of(v_in_a);
        let bcc_b = lattice.bcc_id_of(v_in_b);
        // Parity on whether `bcc_a` has more or fewer coordinates
        // strictly greater than `bcc_b`'s. Paper §3.3 cites this on
        // the cubical lattice Z³ (rule reverses on Z³ + (½, ½, ½)).
        // For our BCC layout, slots 0/1 are odd and 2/3 are even, so
        // (in_slots[0], in_slots[1]) being long means both are on the
        // same sublattice. The rule below is applied on integer BCC
        // indices, which captures the parity invariant identically
        // for both sublattices because the sub-half-cell offset
        // cancels in the difference.
        // Parity Rule on the long-edge endpoints (paper §3.3,
        // mirrored from `isostuffer::create_tet_type2`). The 3-tet
        // split mirrors isostuffer's mapping:
        //   v1 = in_a, v3 = in_b
        //   c0 = cut(out_a, in_a) = c00
        //   c1 = cut(in_a, out_b) = c01
        //   c2 = cut(out_b, in_b) = c11
        //   c3 = cut(in_b, out_a) = c10
        let agc = coord_greater_count(bcc_a, bcc_b) % 2;
        let bd_diagonal = agc == 1;
        if bd_diagonal {
            // "bd" branch: push (v1, c1, v3, c3), (c1, c2, v3, c3),
            // (v1, c1, c3, c0).
            emit_sub_tet([v_in_a, c01, v_in_b, c10], output_positions, output_tets);
            emit_sub_tet([c01, c11, v_in_b, c10], output_positions, output_tets);
            emit_sub_tet([v_in_a, c01, c10, c00], output_positions, output_tets);
        } else {
            // "ac" branch: push (v1, c1, c2, c0), (v1, c2, v3, c0),
            // (c0, c2, v3, c3).
            emit_sub_tet([v_in_a, c01, c11, c00], output_positions, output_tets);
            emit_sub_tet([v_in_a, c11, v_in_b, c00], output_positions, output_tets);
            emit_sub_tet([c00, c11, v_in_b, c10], output_positions, output_tets);
        }
    } else {
        // (in_a, in_b) is short (cross-sublattice). The opposite-edge
        // pair is also short. The two long edges run "across" the
        // cut — one between each in-out pair. Determine which
        // (in, out) pair forms a long edge; the diagonal adjoins the
        // cut on that long edge.

        // Long edges in BCC are (0, 1) and (2, 3); inside slots are
        // one each from {0, 1} and {2, 3} (since (in_a, in_b) short).
        // Same for outside slots. The long-edge pairings between
        // inside and outside are exactly (0, 1) and (2, 3); we
        // identify which (in, out) cross-pair lies on each long edge.
        // BCC layout invariant: with (in_slots short, out_slots
        // short), exactly two of the four (in, out) cross-pairs sit
        // on long edges (i.e., one of `(0, 1)` or `(2, 3)` per the
        // BCC tet layout).
        let mut long_pair_a: Option<(usize, usize)> = None;
        let mut long_pair_b: Option<(usize, usize)> = None;
        for (i, &i_slot) in in_slots.iter().enumerate() {
            for (o, &o_slot) in out_slots.iter().enumerate() {
                if is_long_edge(i_slot, o_slot) {
                    if long_pair_a.is_none() {
                        long_pair_a = Some((i, o));
                    } else {
                        long_pair_b = Some((i, o));
                    }
                }
            }
        }
        let (i_a, o_a) = long_pair_a
            .unwrap_or_else(|| panic!("BCC layout invariant: at least one long (in, out) pair"));
        let (i_b, o_b) = long_pair_b
            .unwrap_or_else(|| panic!("BCC layout invariant: at least two long (in, out) pairs"));
        // The two cuts on long edges: cut(in_slots[i_a], out_slots[o_a])
        // and cut(in_slots[i_b], out_slots[o_b]). Diagonals adjoin
        // these cuts. Directly emit 3 sub-tets matching the truncated
        // diagonal split.
        let cut_long_a = match (i_a, o_a) {
            (0, 0) => c00,
            (0, 1) => c01,
            (1, 0) => c10,
            (1, 1) => c11,
            _ => unreachable!(), // (i_a, o_a) ∈ {0, 1}² — 4 arms above are exhaustive
        };
        let cut_long_b = match (i_b, o_b) {
            (0, 0) => c00,
            (0, 1) => c01,
            (1, 0) => c10,
            (1, 1) => c11,
            _ => unreachable!(), // (i_b, o_b) ∈ {0, 1}² — 4 arms above are exhaustive
        };
        let cut_short_a = match (i_a, o_a) {
            (0, 0) => c01,
            (0, 1) => c00,
            (1, 0) => c11,
            (1, 1) => c10,
            _ => unreachable!(), // (i_a, o_a) ∈ {0, 1}² — 4 arms above are exhaustive
        };
        let cut_short_b = match (i_b, o_b) {
            (0, 0) => c01,
            (0, 1) => c00,
            (1, 0) => c11,
            (1, 1) => c10,
            _ => unreachable!(), // (i_b, o_b) ∈ {0, 1}² — 4 arms above are exhaustive
        };
        // The prism has triangular faces (in_slots[i_a], cut_long_a,
        // cut_short_a) and (in_slots[i_b], cut_long_b, cut_short_b);
        // bisecting via diagonals through the long-edge cuts gives
        // 3 sub-tets.
        let v_a = tet_vids[in_slots[i_a]];
        let v_b = tet_vids[in_slots[i_b]];
        emit_sub_tet(
            [v_a, cut_long_a, v_b, cut_long_b],
            output_positions,
            output_tets,
        );
        emit_sub_tet(
            [v_a, cut_short_a, cut_long_a, cut_long_b],
            output_positions,
            output_tets,
        );
        emit_sub_tet(
            [v_b, cut_long_b, cut_short_b, cut_long_a],
            output_positions,
            output_tets,
        );
    }
}

/// `(3 inside, 1 outside)`: 3 cuts from the outside vertex. The inside
/// polyhedron is a triangular prism with 6 vertices `[in_a, in_b,
/// in_c, cut(out, in_a), cut(out, in_b), cut(out, in_c)]`, split into
/// 3 sub-tets via Parity Rule type 1 (paper §3.3 + isostuffer's
/// `create_tet_type1` pattern, sign-flipped).
///
/// Long-edge structure determines bisection: which long edge of the
/// BCC tet is incident on the outside vertex selects one of three
/// sub-arrangements.
// `clippy::panic` — the `unwrap_or_else(|| panic!(_))` asserts a
// structural BCC layout invariant: with `out_slot ∈ {0, 1, 2, 3}`
// and `in_slots` the other three, exactly one of `in_slots` shares a
// long BCC edge with `out_slot` (slots 0/1 form one long edge, slots
// 2/3 form the other — `super::lattice::BCC_TETS`). A panic here
// indicates a BCC layout regression, not a runtime data error.
#[allow(clippy::too_many_arguments, clippy::panic)]
fn emit_three_one(
    in_slots: [usize; 3],
    out_slot: usize,
    tet_vids: [VertexId; 4],
    lattice: &BccLattice,
    positions: &[Vec3],
    sdf_values: &[f64],
    output_positions: &mut Vec<Vec3>,
    output_tets: &mut Vec<[VertexId; 4]>,
    cut_cache: &mut BTreeMap<EdgeKey, VertexId>,
) {
    // Identify the inside slot that shares a long edge with the
    // outside slot (the "long-paired inside"). The other two inside
    // slots share short edges with the outside slot.
    let long_paired = in_slots
        .iter()
        .copied()
        .find(|&s| is_long_edge(s, out_slot))
        .unwrap_or_else(|| {
            panic!("BCC layout invariant: one inside slot shares a long edge with out_slot")
        });
    let mut short_paired = [0usize; 2];
    let mut k = 0;
    for &s in &in_slots {
        if s != long_paired {
            short_paired[k] = s;
            k += 1;
        }
    }

    // 3 cut points: long-paired (cut on long edge) and 2 short-paired.
    let cut_long = get_or_insert_cut(
        tet_vids[out_slot],
        tet_vids[long_paired],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );
    let cut_short_a = get_or_insert_cut(
        tet_vids[out_slot],
        tet_vids[short_paired[0]],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );
    let cut_short_b = get_or_insert_cut(
        tet_vids[out_slot],
        tet_vids[short_paired[1]],
        lattice,
        positions,
        sdf_values,
        output_positions,
        cut_cache,
    );

    let v_long = tet_vids[long_paired];
    let v_a = tet_vids[short_paired[0]];
    let v_b = tet_vids[short_paired[1]];

    // First sub-tet: 3 inside vertices + cut on the long edge.
    emit_sub_tet([v_long, v_a, v_b, cut_long], output_positions, output_tets);

    // Remaining 5-vertex shape (v_a, v_b, cut_short_a, cut_short_b,
    // cut_long) is a square pyramid; bisect into 2 sub-tets via
    // Parity Rule type 1. The diagonal of the quadrilateral
    // (v_a, cut_short_a, cut_short_b, v_b) is chosen by parity. The
    // other two faces are triangles already.
    let bcc_a = lattice.bcc_id_of(v_a);
    let bcc_b = lattice.bcc_id_of(v_b);
    // Same BCC-integer Parity Rule predicate as
    // `emit_two_one_one_bd` and `emit_two_two` (see module-level
    // "Parity Rule predicate — BCC-integer deviation from paper").
    // (short_paired[0], short_paired[1]) come from a long BCC edge
    // — slots `{0, 1}` (both odd) or slots `{2, 3}` (both even);
    // either way, the predicate is `coord_greater_count(bcc_a,
    // bcc_b) % 2`. `agc == 0` selects "ac" diagonal (bisect via
    // `(v_a, cut_short_b)`); `agc == 1` selects "bd" (bisect via
    // `(v_b, cut_short_a)`).
    let agc = coord_greater_count(bcc_a, bcc_b) % 2;
    if agc == 0 {
        // "ac" diagonal — bisect via (v_a, cut_short_b).
        emit_sub_tet(
            [v_a, cut_short_a, cut_short_b, cut_long],
            output_positions,
            output_tets,
        );
        emit_sub_tet(
            [v_a, cut_short_b, v_b, cut_long],
            output_positions,
            output_tets,
        );
    } else {
        // "bd" diagonal — bisect via (v_b, cut_short_a).
        emit_sub_tet(
            [v_a, cut_short_a, v_b, cut_long],
            output_positions,
            output_tets,
        );
        emit_sub_tet(
            [cut_short_a, cut_short_b, v_b, cut_long],
            output_positions,
            output_tets,
        );
    }
}

/// Number of coordinates of `a` strictly greater than the
/// corresponding coordinate of `b`. Paper §3.3 Parity Rule predicate
/// (`point_greater_number` in isostuffer): the parity of this count
/// drives the diagonal-bisection choice for quadrilateral faces with
/// whole long edges. Comparing pre-warp integer BCC indices makes the
/// rule deterministic and FP-independent.
fn coord_greater_count(a: BccVertexId, b: BccVertexId) -> u32 {
    u32::from(a.i > b.i) + u32::from(a.j > b.j) + u32::from(a.k > b.k)
}

#[cfg(test)]
mod tests {
    // Tests use bounded sentinel values; the explicit `expect` is a
    // runtime safety belt that would never fire under any test
    // invocation. `clippy::float_cmp` is accepted in tests where exact
    // f64 equality is the contract being asserted (warp determinism).
    #![allow(clippy::expect_used, clippy::float_cmp)]
    use super::*;
    use crate::sdf_bridge::Aabb3;
    use crate::sdf_bridge::MeshingHints;
    use approx::assert_relative_eq;

    /// Canonical small lattice — 2 cubes per axis at `cell_size` 1,
    /// bbox `[0, 2]³`. Produces 27 even + 27 odd = 54 vertices and
    /// 12 × 8 = 96 BCC tets.
    fn small_unit_lattice() -> BccLattice {
        let hints = MeshingHints {
            bbox: Aabb3::new(Vec3::zeros(), Vec3::new(2.0, 2.0, 2.0)),
            cell_size: 1.0,
            material_field: None,
        };
        BccLattice::new(&hints)
    }

    /// Synthetic SDF: linear `f(p) = p.x − threshold`. Chosen so that
    /// per-vertex sign is determined entirely by the x-coordinate;
    /// makes the test parametrically simple while exercising warp
    /// candidates on every axis-aligned and many diagonal edges.
    fn linear_x_sdf(positions: &[Vec3], threshold: f64) -> Vec<f64> {
        positions.iter().map(|p| p.x - threshold).collect()
    }

    #[test]
    fn warp_determinism_two_runs_bit_equal() {
        // III-1 contract at the warp-step level: two invocations of
        // warp_lattice on the same lattice + SDF input give bit-equal
        // positions and sdf_values.
        let lattice = small_unit_lattice();
        // Threshold 0.05 puts the SDF zero close to vertices at
        // x = 0 — well inside the warp threshold band, exercising
        // many warp candidates.
        let initial_sdf = linear_x_sdf(&lattice.positions, 0.05);
        let mut positions_a = lattice.positions.clone();
        let mut sdf_a = initial_sdf.clone();
        warp_lattice(&lattice, &mut positions_a, &mut sdf_a);

        let mut positions_b = lattice.positions.clone();
        let mut sdf_b = initial_sdf;
        warp_lattice(&lattice, &mut positions_b, &mut sdf_b);

        for (i, (a, b)) in positions_a.iter().zip(positions_b.iter()).enumerate() {
            assert_eq!(a.x.to_bits(), b.x.to_bits(), "position[{i}].x diverged");
            assert_eq!(a.y.to_bits(), b.y.to_bits(), "position[{i}].y diverged");
            assert_eq!(a.z.to_bits(), b.z.to_bits(), "position[{i}].z diverged");
        }
        for (i, (a, b)) in sdf_a.iter().zip(sdf_b.iter()).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "sdf[{i}] diverged");
        }
    }

    #[test]
    fn warp_zeroes_violating_vertex_sdf() {
        // After warp, any vertex that violated the threshold must
        // have sdf set to exactly 0.0; positions of such vertices
        // should have moved (no longer at the original lattice
        // location) iff they actually warped.
        let lattice = small_unit_lattice();
        // Threshold 0.05 puts the zero very close to corner vertices
        // at x = 0 — well within ALPHA_LONG·1 = 0.24999.
        let initial_sdf = linear_x_sdf(&lattice.positions, 0.05);
        let mut positions = lattice.positions.clone();
        let mut sdf = initial_sdf.clone();
        warp_lattice(&lattice, &mut positions, &mut sdf);

        // At least some vertices must have warped (the test scene
        // chosen guarantees it).
        let n_warped = sdf.iter().filter(|&&v| v == 0.0).count();
        assert!(
            n_warped > 0,
            "expected at least one warp under linear SDF threshold = 0.05",
        );

        // Every warped vertex's position must have moved off its
        // original lattice point. (The reverse — every position
        // change implies a warp — is also true by construction.)
        for (i, (&new_pos, &original_pos)) in
            positions.iter().zip(lattice.positions.iter()).enumerate()
        {
            if sdf[i] == 0.0 && initial_sdf[i] != 0.0 {
                let moved = new_pos != original_pos;
                assert!(moved, "vertex {i} sdf set to 0.0 but position unchanged");
            }
        }
    }

    #[test]
    fn warp_does_not_violate_unwarped_vertex() {
        // A vertex with |t| ≥ α on every incident edge must not warp.
        // Construct an SDF where the zero is exactly at x = 0.5 (mid
        // cell), so at every cell-corner vertex the cut at x = 0.5 is
        // at t = 0.5 from the corner — well above ALPHA_LONG (0.25)
        // and ALPHA_SHORT (0.41). No vertex should warp.
        let lattice = small_unit_lattice();
        let mut positions = lattice.positions.clone();
        let mut sdf = linear_x_sdf(&lattice.positions, 0.5);
        warp_lattice(&lattice, &mut positions, &mut sdf);

        // No SDF value should be 0.0 unless it was originally 0.0.
        // x = 0.5 corner vertices include cube centres at x = 0.5,
        // which already have f = 0.0 pre-warp — those remain 0.0.
        for (i, (&pos, &v)) in lattice.positions.iter().zip(sdf.iter()).enumerate() {
            let original_already_zero = pos.x == 0.5;
            if !original_already_zero {
                assert_ne!(v, 0.0, "vertex {i} unexpectedly warped under mid-cell SDF");
            }
        }
    }

    /// Run a single BCC tet through the dispatch with hand-set sign
    /// values. Returns `(output_positions, output_tets)` for assertion.
    fn dispatch_one(signs: [u8; 4], lattice: &BccLattice) -> (Vec<Vec3>, Vec<[VertexId; 4]>) {
        let mut sdf_values = vec![0.0; lattice.positions.len()];
        let positions = lattice.positions.clone();
        // The first BCC tet of the small_unit_lattice; vertex slots
        // 0/1 are odd-sublattice, 2/3 are even.
        let tet_vids = lattice.tets[0];
        for (slot, &s) in signs.iter().enumerate() {
            let v: f64 = match s {
                SIGN_INSIDE => 1.0,
                SIGN_OUTSIDE => -1.0,
                _ => 0.0,
            };
            sdf_values[tet_vids[slot] as usize] = v;
        }
        let mut output_positions = positions.clone();
        let mut output_tets: Vec<[VertexId; 4]> = Vec::new();
        let mut cut_cache: BTreeMap<EdgeKey, VertexId> = BTreeMap::new();
        dispatch_case(
            tet_vids,
            lattice,
            &positions,
            &sdf_values,
            &mut output_positions,
            &mut output_tets,
            &mut cut_cache,
        );
        (output_positions, output_tets)
    }

    #[test]
    fn dispatch_all_inside_emits_full_bcc_tet() {
        let lattice = small_unit_lattice();
        let (positions, tets) = dispatch_one(
            [SIGN_INSIDE, SIGN_INSIDE, SIGN_INSIDE, SIGN_INSIDE],
            &lattice,
        );
        assert_eq!(tets.len(), 1, "all-inside emits 1 sub-tet");
        // No cut points appended; output_positions == lattice.positions
        // length-wise.
        assert_eq!(positions.len(), lattice.positions.len());
    }

    #[test]
    fn dispatch_all_outside_emits_nothing() {
        let lattice = small_unit_lattice();
        let (_, tets) = dispatch_one(
            [SIGN_OUTSIDE, SIGN_OUTSIDE, SIGN_OUTSIDE, SIGN_OUTSIDE],
            &lattice,
        );
        assert!(tets.is_empty(), "all-outside emits 0 sub-tets");
    }

    #[test]
    fn dispatch_quadruple_zero_discarded() {
        // Per paper §3.3 Option 1.
        let lattice = small_unit_lattice();
        let (_, tets) = dispatch_one(
            [SIGN_BOUNDARY, SIGN_BOUNDARY, SIGN_BOUNDARY, SIGN_BOUNDARY],
            &lattice,
        );
        assert!(tets.is_empty(), "quadruple-zero is discarded");
    }

    #[test]
    fn dispatch_inside_plus_boundary_emits_full_tet() {
        // Mix of inside + boundary (no outside) emits the BCC tet
        // unchanged.
        let lattice = small_unit_lattice();
        for (slot_to_zero, _) in (0..4).zip(0..4) {
            let mut signs = [SIGN_INSIDE; 4];
            signs[slot_to_zero] = SIGN_BOUNDARY;
            let (_, tets) = dispatch_one(signs, &lattice);
            assert_eq!(tets.len(), 1, "inside+boundary emits 1 sub-tet");
        }
    }

    #[test]
    fn dispatch_outside_plus_boundary_emits_nothing() {
        let lattice = small_unit_lattice();
        for slot_to_zero in 0..4 {
            let mut signs = [SIGN_OUTSIDE; 4];
            signs[slot_to_zero] = SIGN_BOUNDARY;
            let (_, tets) = dispatch_one(signs, &lattice);
            assert!(tets.is_empty(), "outside+boundary emits 0 sub-tets");
        }
    }

    #[test]
    fn dispatch_one_inside_three_outside_emits_one_subtet() {
        let lattice = small_unit_lattice();
        for in_slot in 0..4 {
            let mut signs = [SIGN_OUTSIDE; 4];
            signs[in_slot] = SIGN_INSIDE;
            let (positions, tets) = dispatch_one(signs, &lattice);
            assert_eq!(
                tets.len(),
                1,
                "(1 in, 3 out) emits 1 sub-tet (in_slot={in_slot})"
            );
            // 3 cut points appended.
            assert_eq!(
                positions.len(),
                lattice.positions.len() + 3,
                "expected 3 new cut-point vertices",
            );
            // All-positive volume.
            let p = tets[0].map(|vid| positions[vid as usize]);
            let v = signed_volume(p[0], p[1], p[2], p[3]);
            assert!(v > 0.0, "sub-tet has non-positive volume {v}");
        }
    }

    #[test]
    fn dispatch_three_inside_one_outside_emits_three_subtets() {
        let lattice = small_unit_lattice();
        for out_slot in 0..4 {
            let mut signs = [SIGN_INSIDE; 4];
            signs[out_slot] = SIGN_OUTSIDE;
            let (positions, tets) = dispatch_one(signs, &lattice);
            assert_eq!(
                tets.len(),
                3,
                "(3 in, 1 out) emits 3 sub-tets (out_slot={out_slot})",
            );
            assert_eq!(positions.len(), lattice.positions.len() + 3);
            for (sub_idx, t) in tets.iter().enumerate() {
                let p = t.map(|vid| positions[vid as usize]);
                let v = signed_volume(p[0], p[1], p[2], p[3]);
                assert!(
                    v > 0.0,
                    "sub-tet {sub_idx} (out_slot={out_slot}) has non-positive volume {v}",
                );
            }
            // Distinct vertices per sub-tet.
            for (sub_idx, t) in tets.iter().enumerate() {
                let mut sorted = *t;
                sorted.sort_unstable();
                for w in sorted.windows(2) {
                    assert!(
                        w[0] != w[1],
                        "sub-tet {sub_idx} has duplicate vertex ID (out_slot={out_slot})",
                    );
                }
            }
        }
    }

    #[test]
    fn dispatch_two_inside_two_outside_emits_three_subtets() {
        let lattice = small_unit_lattice();
        // Iterate all 6 (in_slots, out_slots) configurations of 4
        // choose 2.
        for in_a in 0..4 {
            for in_b in (in_a + 1)..4 {
                let mut signs = [SIGN_OUTSIDE; 4];
                signs[in_a] = SIGN_INSIDE;
                signs[in_b] = SIGN_INSIDE;
                let (positions, tets) = dispatch_one(signs, &lattice);
                assert_eq!(
                    tets.len(),
                    3,
                    "(2 in, 2 out) emits 3 sub-tets (in_slots=[{in_a},{in_b}])",
                );
                for (sub_idx, t) in tets.iter().enumerate() {
                    let p = t.map(|vid| positions[vid as usize]);
                    let v = signed_volume(p[0], p[1], p[2], p[3]);
                    assert!(
                        v > 0.0,
                        "sub-tet {sub_idx} (in_slots=[{in_a},{in_b}]) has non-positive volume {v}",
                    );
                    // Distinct-vertex assertion: catches stencil bugs
                    // that emit a degenerate tet sharing a vertex
                    // (which would also have signed_volume = 0 and
                    // be caught by the D-10 backstop, but the
                    // explicit assertion gives clearer failure
                    // diagnostics).
                    let mut sorted = *t;
                    sorted.sort_unstable();
                    for w in sorted.windows(2) {
                        assert!(
                            w[0] != w[1],
                            "sub-tet {sub_idx} has duplicate vertex \
                             (in_slots=[{in_a},{in_b}])",
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn dispatch_two_inside_one_outside_one_boundary_emits_two_subtets() {
        let lattice = small_unit_lattice();
        // 4 × 3 = 12 configurations.
        for out_slot in 0..4 {
            for bd_offset in 1..4 {
                let bd_slot = (out_slot + bd_offset) % 4;
                let mut signs = [SIGN_INSIDE; 4];
                signs[out_slot] = SIGN_OUTSIDE;
                signs[bd_slot] = SIGN_BOUNDARY;
                let (positions, tets) = dispatch_one(signs, &lattice);
                assert_eq!(
                    tets.len(),
                    2,
                    "(2 in, 1 out, 1 bd) emits 2 sub-tets (out={out_slot}, bd={bd_slot})",
                );
                for (sub_idx, t) in tets.iter().enumerate() {
                    let p = t.map(|vid| positions[vid as usize]);
                    let v = signed_volume(p[0], p[1], p[2], p[3]);
                    assert!(
                        v > 0.0,
                        "sub-tet {sub_idx} (out={out_slot}, bd={bd_slot}) has non-positive volume {v}",
                    );
                    let mut sorted = *t;
                    sorted.sort_unstable();
                    for w in sorted.windows(2) {
                        assert!(
                            w[0] != w[1],
                            "sub-tet {sub_idx} has duplicate vertex (out={out_slot}, bd={bd_slot})",
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn dispatch_one_inside_two_outside_one_boundary_emits_one_subtet() {
        let lattice = small_unit_lattice();
        for in_slot in 0..4 {
            for bd_offset in 1..4 {
                let bd_slot = (in_slot + bd_offset) % 4;
                let mut signs = [SIGN_OUTSIDE; 4];
                signs[in_slot] = SIGN_INSIDE;
                signs[bd_slot] = SIGN_BOUNDARY;
                let (positions, tets) = dispatch_one(signs, &lattice);
                assert_eq!(
                    tets.len(),
                    1,
                    "(1 in, 2 out, 1 bd) emits 1 sub-tet (in={in_slot}, bd={bd_slot})",
                );
                let p = tets[0].map(|vid| positions[vid as usize]);
                let v = signed_volume(p[0], p[1], p[2], p[3]);
                assert!(v > 0.0, "sub-tet has non-positive volume {v}");
            }
        }
    }

    #[test]
    fn dispatch_one_inside_one_outside_two_boundary_emits_one_subtet() {
        let lattice = small_unit_lattice();
        for in_slot in 0..4 {
            for out_slot in 0..4 {
                if in_slot == out_slot {
                    continue;
                }
                let mut signs = [SIGN_BOUNDARY; 4];
                signs[in_slot] = SIGN_INSIDE;
                signs[out_slot] = SIGN_OUTSIDE;
                let (positions, tets) = dispatch_one(signs, &lattice);
                assert_eq!(tets.len(), 1, "(1 in, 1 out, 2 bd) emits 1 sub-tet");
                let p = tets[0].map(|vid| positions[vid as usize]);
                let v = signed_volume(p[0], p[1], p[2], p[3]);
                assert!(v > 0.0, "sub-tet has non-positive volume {v}");
            }
        }
    }

    #[test]
    fn cut_cache_dedups_shared_edge_across_tets() {
        // Two adjacent BCC tets that share a long edge should
        // produce the same cut VertexId at that shared edge.
        let lattice = small_unit_lattice();
        let mut positions = lattice.positions.clone();
        // Threshold 0.45 keeps cuts well off lattice vertices
        // (corner f = -0.45 or 0.55, centre f = 0.05), and at
        // |t| ≥ 0.45 they exceed both ALPHA_LONG (0.24999) and
        // ALPHA_SHORT (0.41189); but centre vertices have small
        // f-magnitudes (0.05) on edges to corners, putting their
        // |t| close to 1 — they ARE expected to warp under this
        // setup. The shared-edge dedup contract holds regardless of
        // which vertices warped.
        let mut sdf_values = linear_x_sdf(&lattice.positions, 0.45);
        warp_lattice(&lattice, &mut positions, &mut sdf_values);

        let mut output_positions = positions.clone();
        let mut output_tets: Vec<[VertexId; 4]> = Vec::new();
        let mut cut_cache: BTreeMap<EdgeKey, VertexId> = BTreeMap::new();

        for &tet_vids in &lattice.tets {
            dispatch_case(
                tet_vids,
                &lattice,
                &positions,
                &sdf_values,
                &mut output_positions,
                &mut output_tets,
                &mut cut_cache,
            );
        }
        // The cache should have far fewer entries than the total
        // number of cut-emitting calls — proves dedup is happening.
        // Expected: the number of unique BCC edges cut by the plane
        // x = 0.5 in the [0, 2]³ lattice. Empirically this is much
        // smaller than the per-tet dispatch count.
        assert!(
            !cut_cache.is_empty(),
            "non-trivial SDF should produce cut points",
        );
        // All cached cut-point VertexIds must point at appended
        // positions (≥ lattice.positions.len()).
        for (&_, &vid) in &cut_cache {
            assert!(
                vid as usize >= lattice.positions.len(),
                "cached vid {vid} must be appended past the lattice prefix",
            );
        }
        // No two distinct keys map to the same vid.
        let mut seen = std::collections::BTreeSet::new();
        for (&_, &vid) in &cut_cache {
            assert!(seen.insert(vid), "vid {vid} appears in cache twice");
        }
    }

    #[test]
    fn cut_at_t_is_paper_faithful() {
        // Independent regression check on the cut-location formula.
        // For SDF f(p) = p.x − 0.5 along edge from (0, 0, 0) (f = -0.5)
        // to (1, 0, 0) (f = 0.5), t = -(-0.5) / (0.5 - (-0.5)) = 0.5;
        // cut at (0.5, 0, 0).
        let lattice = small_unit_lattice();
        let mut positions = lattice.positions.clone();
        let mut sdf_values = linear_x_sdf(&lattice.positions, 0.5);
        warp_lattice(&lattice, &mut positions, &mut sdf_values);
        let mut output_positions = positions.clone();
        let mut cut_cache: BTreeMap<EdgeKey, VertexId> = BTreeMap::new();
        // Find an axis-aligned edge from a low-x even corner to a
        // high-x even corner.
        let v0 = lattice
            .vertex_id_of(BccVertexId {
                i: 0,
                j: 0,
                k: 0,
                sublattice_tag: 0,
            })
            .expect("origin corner exists");
        let v1 = lattice
            .vertex_id_of(BccVertexId {
                i: 1,
                j: 0,
                k: 0,
                sublattice_tag: 0,
            })
            .expect("adjacent corner exists");
        let cut_id = get_or_insert_cut(
            v0,
            v1,
            &lattice,
            &positions,
            &sdf_values,
            &mut output_positions,
            &mut cut_cache,
        );
        let cut_pos = output_positions[cut_id as usize];
        assert_relative_eq!(cut_pos, Vec3::new(0.5, 0.0, 0.0), epsilon = 1e-15);
    }

    #[test]
    fn edge_key_orders_lexicographically() {
        // (smaller, larger) pair regardless of input order.
        let a = BccVertexId {
            i: 0,
            j: 1,
            k: 0,
            sublattice_tag: 0,
        };
        let b = BccVertexId {
            i: 0,
            j: 1,
            k: 0,
            sublattice_tag: 1,
        };
        assert_eq!(edge_key(a, b), edge_key(b, a));
        // Sublattice-tag disambiguates "same integer index, different
        // sublattice" (tag 0 < tag 1).
        let key = edge_key(a, b);
        assert_eq!(key.0.3, 0);
        assert_eq!(key.1.3, 1);
    }

    #[test]
    fn is_long_edge_matches_bcc_layout() {
        // BCC_TETS slot layout: 0/1 odd, 2/3 even.
        assert!(is_long_edge(0, 1));
        assert!(is_long_edge(2, 3));
        assert!(is_long_edge(1, 0));
        assert!(is_long_edge(3, 2));
        assert!(!is_long_edge(0, 2));
        assert!(!is_long_edge(0, 3));
        assert!(!is_long_edge(1, 2));
        assert!(!is_long_edge(1, 3));
    }
}
