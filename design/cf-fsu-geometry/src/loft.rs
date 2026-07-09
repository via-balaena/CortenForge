//! General mesh-loft primitive — the anatomy-free core of the *bushing*.
//!
//! A **bushing** is defined by its two contact regions: one patch painted on
//! each of the two bodies it sits between, joined by a smooth *perimeter wall*
//! stitched between their rims (the free, no-contact surface). The loft is what
//! connects the two patches into one closed compliant interface.
//!
//! This module is deliberately anatomy-free — it operates on a bare
//! [`IndexedMesh`] and a face-id selection, with no notion of vertebrae,
//! endplates, or discs. The FSU disc is only its *first* consumer (it picks the
//! two endplate patches and lofts them); a general bushing GUI is the second.
//! When that second consumer lands this module is expected to graduate to a
//! standalone `mesh-loft` crate.
//!
//! # Ladder
//!
//! * **B0 (this file):** [`extract_patch`] — from a mesh and a face-id
//!   selection, produce the patch sub-mesh plus its ordered, winding-oriented
//!   boundary-rim loop(s).
//! * B1: stitch two rims into the perimeter wall.
//! * B2: assemble top patch + wall + bottom patch → one closed watertight mesh.
//! * B3+: smooth only the wall; pick anatomical patches; GUI painting.

use std::collections::{HashMap, HashSet};

use cf_geometry::IndexedMesh;
use mesh_repair::MeshAdjacency;
use nalgebra::Point3;

/// A contact patch extracted from a source mesh.
///
/// The patch carries its own compact vertex array (unused source vertices are
/// dropped and the faces re-indexed), the ordered boundary-rim loop(s) that
/// bound it, and the map back to the source vertices it came from.
#[derive(Debug, Clone)]
pub struct Patch {
    /// The patch sub-mesh: the selected faces, re-indexed against a compact
    /// vertex array containing only the vertices those faces reference.
    pub mesh: IndexedMesh,

    /// Ordered boundary-rim loops, each a list of indices into
    /// [`Patch::mesh`]'s vertices.
    ///
    /// Each loop is **winding-oriented**: consecutive entries are the tail and
    /// head of a directed boundary half-edge `a → b` exactly as it is wound in
    /// its single owning face, and the last entry connects back to the first.
    /// So the loop runs counter-clockwise as seen from the patch's outward
    /// (face-normal) side — the orientation the wall stitch (B1) needs. A
    /// simple painted patch (a disc of the surface) has exactly one loop; a
    /// patch with an interior hole has more.
    pub rims: Vec<Vec<u32>>,

    /// For each patch vertex, the index it held in the source mesh:
    /// `source_vertex[new] = old`. Lets a caller pin the patch back onto the
    /// body it was painted on (the disc's contact bond) or carry attributes.
    pub source_vertex: Vec<u32>,
}

/// Extract the patch defined by `face_ids` from `mesh`.
///
/// Produces the re-indexed sub-mesh of the selected faces plus its ordered,
/// winding-oriented boundary-rim loop(s). Face ids out of range are skipped.
///
/// A boundary edge is one that appears in exactly one selected face; the rim is
/// the closed chain of those edges. Detection reuses the tested
/// [`MeshAdjacency`] boundary set; this function adds the winding orientation
/// (the directed half-edge `a → b` as wound in its owning face) that the
/// undirected adjacency does not carry.
///
/// # Assumptions
///
/// The selection is expected to be an edge-manifold surface patch (each
/// boundary vertex has a single outgoing boundary half-edge). Non-manifold
/// pinches — where one vertex bounds the rim twice — are not resolved: the last
/// half-edge wins and the affected loop may be truncated.
#[must_use]
pub fn extract_patch(mesh: &IndexedMesh, face_ids: &[usize]) -> Patch {
    // --- Re-index the selected faces against a compact vertex array. ---
    // First-seen order over the selection gives deterministic, stable indices.
    let mut old_to_new: HashMap<u32, u32> = HashMap::new();
    let mut source_vertex: Vec<u32> = Vec::new();
    let mut vertices = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    // Runs in lockstep with `vertices.len()`; kept as a `u32` so re-indexing
    // never casts a `usize` length down (vertex indices are `u32` by contract).
    let mut next_new: u32 = 0;

    for &fid in face_ids {
        let Some(face) = mesh.face(fid) else { continue };
        let mut new_face = [0u32; 3];
        for (slot, &old) in new_face.iter_mut().zip(face.iter()) {
            let new = *old_to_new.entry(old).or_insert_with(|| {
                let idx = next_new;
                next_new += 1;
                source_vertex.push(old);
                vertices.push(mesh.vertices[old as usize]);
                idx
            });
            *slot = new;
        }
        faces.push(new_face);
    }

    let rims = extract_rims(&faces);
    let mesh = IndexedMesh::from_parts(vertices, faces);
    Patch {
        mesh,
        rims,
        source_vertex,
    }
}

/// Trace the ordered, winding-oriented boundary-rim loops of a face set.
fn extract_rims(faces: &[[u32; 3]]) -> Vec<Vec<u32>> {
    let adjacency = MeshAdjacency::build(faces);
    let boundary: HashSet<(u32, u32)> = adjacency.boundary_edges().collect();
    if boundary.is_empty() {
        return Vec::new();
    }

    // Orient each boundary edge by the winding of its single owning face:
    // `next[a] = b` for the directed half-edge a → b. `tails` records the
    // half-edge tails in face order so loop seeding is deterministic.
    let mut next: HashMap<u32, u32> = HashMap::new();
    let mut tails: Vec<u32> = Vec::new();
    for &[a, b, c] in faces {
        for (from, to) in [(a, b), (b, c), (c, a)] {
            if boundary.contains(&normalize_edge(from, to)) {
                next.insert(from, to);
                tails.push(from);
            }
        }
    }

    let mut visited: HashSet<u32> = HashSet::new();
    let mut rims: Vec<Vec<u32>> = Vec::new();
    for &start in &tails {
        if visited.contains(&start) {
            continue;
        }
        let mut loop_vertices = vec![start];
        visited.insert(start);
        let mut current = start;
        while let Some(&head) = next.get(&current) {
            if head == start {
                break; // loop closed
            }
            if !visited.insert(head) {
                break; // malformed / non-manifold pinch — stop this loop
            }
            loop_vertices.push(head);
            current = head;
        }
        if loop_vertices.len() >= 3 {
            rims.push(loop_vertices);
        }
    }
    rims
}

/// Normalize an edge so the smaller vertex index comes first — matches
/// [`MeshAdjacency`]'s undirected edge key.
#[inline]
const fn normalize_edge(v0: u32, v1: u32) -> (u32, u32) {
    if v0 < v1 { (v0, v1) } else { (v1, v0) }
}

/// Stitch two boundary-rim loops into the bushing's perimeter wall — the free
/// (no-contact) surface bridging the top patch to the bottom patch (B1).
///
/// Both rims index into the shared `verts` array (the assembled bushing's
/// vertex list); the returned triangles reference the same indices, so the wall
/// drops straight into the assembly (B2) alongside the two patch face sets.
///
/// The stitch is correspondence-robust: it aligns the rims by nearest start
/// vertex, picks the winding direction of `rim_b` that matches `rim_a` (the two
/// patches face *away* from each other, so their rims are typically listed in
/// opposite angular order — a naive index-parallel bridge would twist), then
/// advances along both by normalized arc length so unequal vertex counts
/// distribute cleanly. The result is a closed triangle band of exactly
/// `rim_a.len() + rim_b.len()` faces whose only boundary edges are the two rims.
///
/// Returns an empty vec if either rim has fewer than three vertices or is
/// geometrically degenerate (zero perimeter).
#[must_use]
pub fn stitch_rims(verts: &[Point3<f64>], rim_a: &[u32], rim_b: &[u32]) -> Vec<[u32; 3]> {
    let (na, nb) = (rim_a.len(), rim_b.len());
    if na < 3 || nb < 3 {
        return Vec::new();
    }

    let pa: Vec<Point3<f64>> = rim_a.iter().map(|&v| verts[v as usize]).collect();
    let pb: Vec<Point3<f64>> = rim_b.iter().map(|&v| verts[v as usize]).collect();

    // Align `rim_b` to start at the vertex nearest `rim_a[0]`, then choose the
    // traversal direction whose next step stays closest to `rim_a`'s next step.
    let k = (0..nb)
        .min_by(|&i, &j| dist(&pa[0], &pb[i]).total_cmp(&dist(&pa[0], &pb[j])))
        .unwrap_or(0);
    let fwd = dist(&pa[1], &pb[(k + 1) % nb]);
    let rev = dist(&pa[1], &pb[(k + nb - 1) % nb]);
    let reverse = rev < fwd;

    // `rim_b` rotated to start at `k`, forward or reversed to match `rim_a`.
    let b_idx: Vec<u32> = (0..nb)
        .map(|m| {
            let src = if reverse {
                (k + nb - m) % nb
            } else {
                (k + m) % nb
            };
            rim_b[src]
        })
        .collect();
    let b_pos: Vec<Point3<f64>> = b_idx.iter().map(|&v| verts[v as usize]).collect();

    let ta = cumulative_params(&pa);
    let tb = cumulative_params(&b_pos);
    if ta.is_empty() || tb.is_empty() {
        return Vec::new(); // degenerate (zero perimeter)
    }

    // Greedy loop-band triangulation: advance whichever rim is behind in
    // normalized arc length, emitting one triangle per advance. `na + nb`
    // advances → `na + nb` triangles closing the band.
    let mut faces = Vec::with_capacity(na + nb);
    let (mut i, mut j) = (0usize, 0usize);
    while i < na || j < nb {
        let advance_a = if i >= na {
            false
        } else if j >= nb {
            true
        } else {
            ta[i + 1] <= tb[j + 1]
        };
        if advance_a {
            faces.push([rim_a[i], b_idx[j % nb], rim_a[(i + 1) % na]]);
            i += 1;
        } else {
            faces.push([rim_a[i % na], b_idx[j], b_idx[(j + 1) % nb]]);
            j += 1;
        }
    }
    faces
}

/// Euclidean distance between two points.
#[inline]
fn dist(a: &Point3<f64>, b: &Point3<f64>) -> f64 {
    (a - b).norm()
}

/// Cumulative normalized arc length of a closed loop: length `n + 1`, with
/// `[0] = 0`, `[n] = 1.0` (the perimeter including the closing edge back to the
/// start). Returns empty if the loop has zero perimeter.
fn cumulative_params(positions: &[Point3<f64>]) -> Vec<f64> {
    let n = positions.len();
    let mut cum = Vec::with_capacity(n + 1);
    cum.push(0.0);
    let mut total = 0.0;
    for i in 0..n {
        total += dist(&positions[i], &positions[(i + 1) % n]);
        cum.push(total);
    }
    if total <= 0.0 {
        return Vec::new();
    }
    for c in &mut cum {
        *c /= total;
    }
    cum
}

#[cfg(test)]
#[allow(
    // Tests legitimately use `.unwrap()`/indexing to assert structure; the
    // workspace deny-lints on these are relaxed for the test module.
    clippy::unwrap_used,
    clippy::indexing_slicing,
    // Synthetic-circle fixtures cast small loop counts to `f64` for angles and
    // to `u32` for rim indices — both fit trivially at test sizes.
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation
)]
mod tests {
    use std::f64::consts::TAU;

    use super::*;
    use nalgebra::Point3;

    /// A unit square (4 corners, 2 CCW triangles in the z=0 plane) plus a stray
    /// vertex and triangle that share the square's `1–2` edge but are *not*
    /// selected — so the test also exercises sub-mesh compaction.
    fn square_with_stray() -> (IndexedMesh, Vec<usize>) {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0), // 0
            Point3::new(1.0, 0.0, 0.0), // 1
            Point3::new(1.0, 1.0, 0.0), // 2
            Point3::new(0.0, 1.0, 0.0), // 3
            Point3::new(2.0, 2.0, 0.0), // 4 — stray, must be dropped
        ];
        let faces = vec![
            [0, 1, 2], // square, selected
            [0, 2, 3], // square, selected
            [1, 4, 2], // stray, NOT selected
        ];
        (IndexedMesh::from_parts(vertices, faces), vec![0, 1])
    }

    /// Signed area of a 2D polygon in the z=0 plane (shoelace). Positive = CCW.
    fn signed_area_xy(mesh: &IndexedMesh, loop_vertices: &[u32]) -> f64 {
        let n = loop_vertices.len();
        let mut area = 0.0;
        for i in 0..n {
            let p = mesh.vertices[loop_vertices[i] as usize];
            let q = mesh.vertices[loop_vertices[(i + 1) % n] as usize];
            area += p.x * q.y - q.x * p.y;
        }
        area * 0.5
    }

    #[test]
    fn extract_square_patch_compacts_and_traces_rim() {
        let (mesh, selection) = square_with_stray();
        let patch = extract_patch(&mesh, &selection);

        // Sub-mesh compaction: the stray vertex 4 is dropped.
        assert_eq!(patch.mesh.vertex_count(), 4);
        assert_eq!(patch.mesh.face_count(), 2);
        assert_eq!(patch.source_vertex, vec![0, 1, 2, 3]);

        // Exactly one rim, the 4-corner square perimeter.
        assert_eq!(patch.rims.len(), 1);
        let rim = &patch.rims[0];
        assert_eq!(rim.len(), 4);
        let unique: HashSet<u32> = rim.iter().copied().collect();
        assert_eq!(unique, HashSet::from([0, 1, 2, 3]));

        // Every consecutive pair is a real boundary edge of the patch.
        let adjacency = MeshAdjacency::build(&patch.mesh.faces);
        let boundary: HashSet<(u32, u32)> = adjacency.boundary_edges().collect();
        for i in 0..rim.len() {
            let edge = normalize_edge(rim[i], rim[(i + 1) % rim.len()]);
            assert!(boundary.contains(&edge), "rim edge {edge:?} not a boundary");
        }

        // Winding is CCW (positive area) — the orientation B1's stitch needs.
        assert!(signed_area_xy(&patch.mesh, rim) > 0.0);
    }

    #[test]
    fn single_triangle_patch_has_triangular_rim() {
        let mesh = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let patch = extract_patch(&mesh, &[0]);
        assert_eq!(patch.rims.len(), 1);
        assert_eq!(patch.rims[0].len(), 3);
        assert!(signed_area_xy(&patch.mesh, &patch.rims[0]) > 0.0);
    }

    #[test]
    fn empty_selection_yields_empty_patch() {
        let (mesh, _) = square_with_stray();
        let patch = extract_patch(&mesh, &[]);
        assert_eq!(patch.mesh.vertex_count(), 0);
        assert_eq!(patch.mesh.face_count(), 0);
        assert!(patch.rims.is_empty());
    }

    #[test]
    fn out_of_range_face_ids_are_skipped() {
        let (mesh, _) = square_with_stray();
        // 99 is out of range; only face 0 is real.
        let patch = extract_patch(&mesh, &[0, 99]);
        assert_eq!(patch.mesh.face_count(), 1);
        assert_eq!(patch.mesh.vertex_count(), 3);
    }

    const CYL_R: f64 = 2.0;
    const CYL_H: f64 = 3.0;

    /// One circle of `n` vertices at height `z`, radius [`CYL_R`], wound CCW
    /// (`ccw = true`) or CW when viewed from +z.
    fn circle(n: usize, z: f64, ccw: bool) -> Vec<Point3<f64>> {
        (0..n)
            .map(|i| {
                let t = TAU * (i as f64) / (n as f64);
                let a = if ccw { t } else { -t };
                Point3::new(CYL_R * a.cos(), CYL_R * a.sin(), z)
            })
            .collect()
    }

    /// Two coaxial circles as a shared vertex array plus the two rim loops.
    /// The bottom rim is wound *opposite* the top when `opposite` — the
    /// realistic bushing case (patches face away, so rims list in reverse
    /// angular order), which a naive index-parallel stitch would twist.
    fn cylinder_rims(
        n_top: usize,
        n_bot: usize,
        opposite: bool,
    ) -> (Vec<Point3<f64>>, Vec<u32>, Vec<u32>) {
        let mut verts = circle(n_top, CYL_H, true);
        verts.extend(circle(n_bot, 0.0, !opposite));
        let rim_a: Vec<u32> = (0..n_top as u32).collect();
        let rim_b: Vec<u32> = (n_top as u32..(n_top + n_bot) as u32).collect();
        (verts, rim_a, rim_b)
    }

    /// Area of the triangle `[a, b, c]` (indices into `verts`).
    fn tri_area(verts: &[Point3<f64>], f: [u32; 3]) -> f64 {
        let (a, b, c) = (
            verts[f[0] as usize],
            verts[f[1] as usize],
            verts[f[2] as usize],
        );
        (b - a).cross(&(c - a)).norm() * 0.5
    }

    /// Normalized edges of a rim loop (each consecutive pair, closing the loop).
    fn rim_edges(rim: &[u32]) -> HashSet<(u32, u32)> {
        (0..rim.len())
            .map(|i| normalize_edge(rim[i], rim[(i + 1) % rim.len()]))
            .collect()
    }

    /// Assert the stitched wall is a clean, untwisted band: right face count,
    /// edge-manifold, boundary == exactly the two rims, and total area within
    /// tolerance of the ideal cylinder wall (a twist inflates the area).
    fn assert_clean_wall(verts: &[Point3<f64>], rim_a: &[u32], rim_b: &[u32], faces: &[[u32; 3]]) {
        assert_eq!(faces.len(), rim_a.len() + rim_b.len(), "face count");

        // No degenerate triangles.
        for &f in faces {
            assert!(tri_area(verts, f) > 1e-9, "degenerate triangle {f:?}");
        }

        // Edge-manifold, and its only boundary edges are the two rims.
        let adjacency = MeshAdjacency::build(faces);
        assert_eq!(adjacency.non_manifold_edge_count(), 0, "non-manifold wall");
        let boundary: HashSet<(u32, u32)> = adjacency.boundary_edges().collect();
        let mut expected = rim_edges(rim_a);
        expected.extend(rim_edges(rim_b));
        assert_eq!(boundary, expected, "wall boundary is not exactly the rims");

        // Total area near the ideal cylinder lateral area — a twisted stitch
        // (long diagonal triangles crossing the axis) blows this up.
        let area: f64 = faces.iter().map(|&f| tri_area(verts, f)).sum();
        let ideal = TAU * CYL_R * CYL_H;
        assert!(
            (area - ideal).abs() / ideal < 0.05,
            "wall area {area} not within 5% of ideal {ideal}"
        );
    }

    #[test]
    fn stitch_opposite_wound_circles_is_clean_cylinder() {
        // The realistic case: rims wound opposite. Must not twist.
        let (verts, rim_a, rim_b) = cylinder_rims(16, 16, true);
        let faces = stitch_rims(&verts, &rim_a, &rim_b);
        assert_clean_wall(&verts, &rim_a, &rim_b, &faces);
    }

    #[test]
    fn stitch_same_wound_circles_is_clean_cylinder() {
        // Direction detection must not wrongly reverse already-aligned rims.
        let (verts, rim_a, rim_b) = cylinder_rims(16, 16, false);
        let faces = stitch_rims(&verts, &rim_a, &rim_b);
        assert_clean_wall(&verts, &rim_a, &rim_b, &faces);
    }

    #[test]
    fn stitch_unequal_counts_is_clean_cylinder() {
        // Unequal vertex counts must still bridge cleanly (arc-length advance).
        let (verts, rim_a, rim_b) = cylinder_rims(16, 9, true);
        let faces = stitch_rims(&verts, &rim_a, &rim_b);
        assert_clean_wall(&verts, &rim_a, &rim_b, &faces);
    }

    #[test]
    fn stitch_degenerate_rims_return_empty() {
        let verts = circle(3, 0.0, true);
        // A two-vertex "rim" is not a loop.
        assert!(stitch_rims(&verts, &[0, 1], &[0, 1, 2]).is_empty());
    }
}
