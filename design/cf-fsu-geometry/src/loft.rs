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

#[cfg(test)]
#[allow(
    // Tests legitimately use `.unwrap()`/indexing to assert structure; the
    // workspace deny-lints on these are relaxed for the test module.
    clippy::unwrap_used,
    clippy::indexing_slicing
)]
mod tests {
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
}
