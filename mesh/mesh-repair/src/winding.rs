//! Normal consistency and winding order: measurement and correction.
//!
//! This module both **reports** inconsistent face winding ([`winding_census`],
//! [`count_inconsistent_faces`]) and **fixes** it ([`fix_winding_order`],
//! [`flip_winding`]). Consistent winding is required for correct normal
//! computation, for rendering, and for any inside/outside test derived from
//! face normals — see [`WindingCensus`] for which question each instrument
//! actually answers.
//!
//! # Winding Convention
//!
//! CortenForge uses **counter-clockwise (CCW) winding** when viewed from the front.
//! Normals point outward by the right-hand rule.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Point3};
//! use mesh_repair::winding::fix_winding_order;
//!
//! let mut mesh = IndexedMesh::new();
//! // ... add vertices and faces ...
//!
//! // Fix inconsistent winding
//! fix_winding_order(&mut mesh).unwrap();
//! ```

#![allow(
    // `usize` → `u32` casts are safe at mesh sizes the crate targets:
    // face / vertex indices fit in `u32` by mesh-types contract.
    clippy::cast_possible_truncation
)]

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;
use std::collections::VecDeque;
use tracing::{debug, info};

use crate::adjacency::MeshAdjacency;
use crate::error::RepairResult;

/// Fix winding order so all faces have consistent orientation.
///
/// Uses BFS flood fill from an arbitrary start face in each connected component.
/// For each face, ensures that shared edges are traversed in opposite directions.
///
/// This function handles disconnected meshes by processing each component separately.
///
/// # Arguments
///
/// * `mesh` - The mesh to repair
///
/// # Returns
///
/// Ok(()) if successful.
///
/// # Errors
///
/// Currently always returns `Ok(())` — the function operates in-place
/// on `mesh.faces` and the only failure modes are upstream
/// adjacency-graph construction failures, which `MeshAdjacency::build`
/// handles internally without surfacing through this signature. The
/// `RepairResult<()>` shape is preserved for forward compatibility
/// (future winding strategies that consult per-vertex normals or
/// signed-volume diagnostics may surface error variants).
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Point3};
/// use mesh_repair::winding::fix_winding_order;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
/// mesh.vertices.push(Point3::new(0.5, -1.0, 0.0));
///
/// // Two triangles with inconsistent winding
/// mesh.faces.push([0, 1, 2]); // CCW
/// mesh.faces.push([0, 1, 3]); // Wrong: same direction as first on shared edge
///
/// fix_winding_order(&mut mesh).unwrap();
/// // Now both triangles have consistent winding
/// ```
pub fn fix_winding_order(mesh: &mut IndexedMesh) -> RepairResult<()> {
    if mesh.faces.is_empty() {
        return Ok(());
    }

    let adjacency = MeshAdjacency::build(&mesh.faces);
    let face_count = mesh.faces.len();

    // Track which faces have been visited globally
    let mut global_visited: HashSet<u32> = HashSet::new();
    let mut to_flip: HashSet<u32> = HashSet::new();
    let mut component_count = 0;
    let mut total_flipped = 0;

    // Process all faces, starting new components as needed
    for start_face in 0..face_count {
        let start_face = start_face as u32;

        // Skip already visited faces
        if global_visited.contains(&start_face) {
            continue;
        }

        // Start a new component
        component_count += 1;
        let mut component_flips: HashSet<u32> = HashSet::new();
        let mut queue: VecDeque<u32> = VecDeque::new();

        queue.push_back(start_face);
        global_visited.insert(start_face);

        while let Some(face_idx) = queue.pop_front() {
            let face = mesh.faces[face_idx as usize];

            // Check all three edges of this face
            for edge_idx in 0..3 {
                let v0 = face[edge_idx];
                let v1 = face[(edge_idx + 1) % 3];

                // Find neighbor faces sharing this edge
                if let Some(neighbors) = adjacency.faces_for_edge(v0, v1) {
                    for &neighbor_idx in neighbors {
                        let neighbor_idx = neighbor_idx as u32;
                        if neighbor_idx == face_idx {
                            continue;
                        }

                        if global_visited.contains(&neighbor_idx) {
                            continue;
                        }

                        global_visited.insert(neighbor_idx);

                        // Check edge direction in neighbor
                        let neighbor_face = mesh.faces[neighbor_idx as usize];
                        let neighbor_dir = edge_direction_in_face(&neighbor_face, v0, v1);

                        // Current face traverses edge as v0 -> v1
                        // For consistent winding, neighbor should traverse as v1 -> v0
                        // (opposite direction on the shared edge)
                        // If neighbor has same direction, one of them needs flipping
                        // Since current face is "correct", flip the neighbor
                        // (Edge not found shouldn't happen, defaults to no flip)
                        let should_flip = neighbor_dir.unwrap_or(false);

                        let actual_flip = if component_flips.contains(&face_idx) {
                            // Current face was itself flipped, so invert the decision
                            !should_flip
                        } else {
                            should_flip
                        };

                        if actual_flip {
                            component_flips.insert(neighbor_idx);
                        }

                        queue.push_back(neighbor_idx);
                    }
                }
            }
        }

        // Add this component's flips to the global set
        total_flipped += component_flips.len();
        to_flip.extend(component_flips);
    }

    // Apply flips (swap indices 1 and 2)
    for &face_idx in &to_flip {
        let face = &mut mesh.faces[face_idx as usize];
        face.swap(1, 2);
    }

    if total_flipped > 0 {
        info!(
            "Fixed winding order: flipped {} faces across {} component(s)",
            total_flipped, component_count
        );
    } else {
        debug!(
            "Winding order already consistent across {} component(s)",
            component_count
        );
    }

    Ok(())
}

/// Check if edge (a, b) appears in face in the same direction (a -> b).
/// Returns `Some(true)` if same direction, `Some(false)` if opposite, `None` if edge not found.
fn edge_direction_in_face(face: &[u32; 3], a: u32, b: u32) -> Option<bool> {
    for i in 0..3 {
        let v0 = face[i];
        let v1 = face[(i + 1) % 3];

        if v0 == a && v1 == b {
            return Some(true); // Same direction
        }
        if v0 == b && v1 == a {
            return Some(false); // Opposite direction
        }
    }
    None
}

/// Reverse the winding of every triangle in place: `[a, b, c] → [a, c, b]`.
///
/// Per-face operation that swaps indices 1 and 2 of every face. Vertex
/// positions and counts are preserved exactly; only triangle orientation
/// flips. The operation is its own inverse — calling `flip_winding` twice
/// returns the mesh to its original state.
///
/// # When to use this vs `fix_winding_order`
///
/// `fix_winding_order` is adjacency-based: it builds the edge-to-face map
/// and BFS-traverses each connected component, flipping individual faces
/// to make shared edges traverse in opposite directions. It only works
/// when faces share edges by index. On a **soup mesh** (every triangle
/// disconnected, e.g. marching-cubes output before welding) it is a no-op
/// because BFS visits zero neighbors from any starting face.
///
/// `flip_winding` is the right tool for soup meshes. It also works as a
/// universal "reverse the orientation of every face" operation on any
/// mesh — for example, when a producer emits inside-out output and the
/// caller wants to flip the entire surface in one pass without first
/// having to detect per-face inconsistency.
///
/// # Example
///
/// ```
/// use mesh_repair::flip_winding;
/// use mesh_types::{IndexedMesh, Point3};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// flip_winding(&mut mesh);
/// assert_eq!(mesh.faces[0], [0, 2, 1]);
///
/// flip_winding(&mut mesh);
/// assert_eq!(mesh.faces[0], [0, 1, 2]); // self-inverse
/// ```
pub fn flip_winding(mesh: &mut IndexedMesh) {
    for face in &mut mesh.faces {
        face.swap(1, 2);
    }
}

/// Count the number of faces that would need to be flipped.
///
/// This is a non-mutating version that just reports how many faces
/// have inconsistent winding.
///
/// # What this number is, and is not
///
/// It is the count of faces that disagree with **an arbitrary seed face** —
/// BFS starts at the lowest-index unvisited face of each component and
/// propagates that face's orientation outward. So it answers *"how many
/// faces would `fix_winding_order` swap?"*, which is a question about the
/// repair, not about the mesh: on a component of `n` faces with a single
/// misoriented face, this returns `1` when the seed lands in the majority
/// and `n - 1` when the seed lands on the flipped face itself. Both
/// describe the same defect.
///
/// It is also a no-op on a **soup mesh** (no two faces share vertex
/// indices), for the reason given on [`flip_winding`] — BFS visits zero
/// neighbours, so it returns `0` for a mesh whose winding was never
/// examined at all.
///
/// For a seed-independent measure that distinguishes "consistent" from
/// "never checked", use [`winding_census`].
///
/// # Arguments
///
/// * `mesh` - The mesh to analyze
///
/// # Returns
///
/// The number of faces that would be flipped by `fix_winding_order`.
#[must_use]
pub fn count_inconsistent_faces(mesh: &IndexedMesh) -> usize {
    if mesh.faces.is_empty() {
        return 0;
    }

    let adjacency = MeshAdjacency::build(&mesh.faces);
    let face_count = mesh.faces.len();

    let mut global_visited: HashSet<u32> = HashSet::new();
    let mut total_flipped = 0;

    for start_face in 0..face_count {
        let start_face = start_face as u32;

        if global_visited.contains(&start_face) {
            continue;
        }

        let mut component_flips: HashSet<u32> = HashSet::new();
        let mut queue: VecDeque<u32> = VecDeque::new();

        queue.push_back(start_face);
        global_visited.insert(start_face);

        while let Some(face_idx) = queue.pop_front() {
            let face = mesh.faces[face_idx as usize];

            for edge_idx in 0..3 {
                let v0 = face[edge_idx];
                let v1 = face[(edge_idx + 1) % 3];

                if let Some(neighbors) = adjacency.faces_for_edge(v0, v1) {
                    for &neighbor_idx in neighbors {
                        let neighbor_idx = neighbor_idx as u32;
                        if neighbor_idx == face_idx || global_visited.contains(&neighbor_idx) {
                            continue;
                        }

                        global_visited.insert(neighbor_idx);

                        let neighbor_face = mesh.faces[neighbor_idx as usize];
                        let neighbor_dir = edge_direction_in_face(&neighbor_face, v0, v1);
                        let should_flip = neighbor_dir.unwrap_or(false);

                        let actual_flip = if component_flips.contains(&face_idx) {
                            !should_flip
                        } else {
                            should_flip
                        };

                        if actual_flip {
                            component_flips.insert(neighbor_idx);
                        }

                        queue.push_back(neighbor_idx);
                    }
                }
            }
        }

        total_flipped += component_flips.len();
    }

    total_flipped
}

/// Per-edge orientation-consistency census — the **local** winding instrument.
///
/// # Why this exists next to `is_inside_out`
///
/// [`crate::validate_mesh`] reports `is_inside_out`, which is a **signed-volume**
/// test: it sums origin-apex tetrahedra over the faces and asks whether the
/// total came out negative.
///
/// ⚠ **That sum is translation-invariant only while the surface is consistently
/// oriented.** Flip one face and the invariance is gone: that face's term then
/// grows with the mesh's distance from the world origin, so the flag becomes a
/// function of *where the mesh sits*. Measured, on the octahedron fixture in
/// this file's tests, flipping a single face:
///
/// | z-offset | 0 | 1 | 2 | 3 | 4 | 10 | 970 |
/// |---|---|---|---|---|---|---|---|
/// | `is_inside_out` | false | false | false | false | **true** | **true** | **true** |
///
/// (Producer: `signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin`.)
///
/// So `is_inside_out` is **not** a sound reading of local winding in either
/// direction. Near the origin it misses a local flip; far from it, a local flip
/// can set the flag that a reader will interpret as "globally reversed". Meshes
/// kept in a native anatomical frame sit far from the origin, which is exactly
/// where the second failure lives.
///
/// This census asks the local question directly and answers it with no
/// reference to the origin, or to any coordinate at all — it reads
/// `mesh.faces` and never touches `mesh.vertices`. On a consistently oriented
/// surface every interior edge is traversed in **opposite** directions by its
/// two incident faces; an edge traversed the *same* way by both is a local
/// orientation defect.
///
/// The two instruments are orthogonal and neither subsumes the other. A global
/// flip **preserves** `inconsistent_edges` exactly (it maps `forward` to
/// `2 - forward`, so `forward != 1` is unchanged) while negating the signed
/// volume — so a consistently-but-inward-wound mesh reads clean here and fires
/// there.
///
/// # What this does NOT answer
///
/// A clean census is **necessary, not sufficient**, for a surface to be safe to
/// sign. It is combinatorial, so it is blind to every geometric defect:
/// self-intersection, near-degenerate slivers whose pseudo-normals vanish (see
/// `mesh_sdf::TriMeshDistance::health`, which measures that consequence
/// directly), and — the sharpest case — **two disjoint closed shells wound
/// oppositely**. The census compares only faces that share an edge, so it never
/// compares the two shells; it reports zero inconsistent edges, and
/// `is_inside_out` reports clean too whenever the larger shell dominates the
/// volume. Per-component orientation is the instrument for that one; see
/// [`crate::find_connected_components`].
///
/// # Seed-independence
///
/// Unlike [`count_inconsistent_faces`], nothing here depends on a traversal
/// order or a starting face — each edge is classified from its own two faces.
/// A single flipped triangle reports exactly its own three edges **when all
/// three are interior**; where one of its edges is a boundary or non-manifold
/// edge it reports correspondingly fewer, because there is no second face to
/// disagree with.
///
/// # What is deliberately excluded
///
/// * **Non-manifold edges** (more than two incident faces) are counted in
///   `non_manifold_edges` and excluded from `interior_edges` /
///   `inconsistent_edges`. "Both faces agree" has no unique meaning when there
///   are three of them, and this is an input class on which a pseudo-normal
///   sign oracle may become undefined — so it is reported, not folded into a
///   number that would read clean.
/// * **Degenerate faces** (a face listing the same vertex twice) are counted in
///   `degenerate_faces` and skipped whole. Such a face traverses one edge in
///   both directions by itself; counting its corners would fabricate a
///   two-faced interior edge out of a single face.
///
/// # Vacuity
///
/// A mesh with no interior edges has nothing to check, and reports
/// `inconsistent_edges == 0` for that reason rather than because its winding
/// was verified. That covers triangle soup, a single triangle, a fully-open
/// surface, and a mesh whose only shared edges are non-manifold. Use
/// [`Self::has_judgeable_edges`] before reading a zero as good news.
///
/// # Example
///
/// ```
/// use mesh_repair::{winding_census, flip_winding};
/// use mesh_types::{IndexedMesh, Point3};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
/// mesh.vertices.push(Point3::new(0.5, 0.5, 1.0));
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([0, 3, 1]);
/// mesh.faces.push([1, 3, 2]);
/// mesh.faces.push([2, 3, 0]);
///
/// let census = winding_census(&mesh);
/// assert_eq!(census.interior_edges, 6);
/// assert_eq!(census.inconsistent_edges, 0);
/// assert!(census.has_judgeable_edges());
///
/// // A global flip stays locally consistent — every edge still has one
/// // traversal each way.
/// flip_winding(&mut mesh);
/// assert_eq!(winding_census(&mesh).inconsistent_edges, 0);
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct WindingCensus {
    /// Edges with exactly two incident faces — the edges this census can judge.
    ///
    /// Faces listing a vertex twice are skipped whole (`degenerate_faces`), so
    /// every surviving face touches each of its edges exactly once and this is
    /// a genuine count of *distinct incident faces*, not of traversals.
    pub interior_edges: usize,
    /// Interior edges traversed in the **same** direction by both incident
    /// faces. Zero on a consistently oriented surface.
    ///
    /// ⚠ Two *identical* faces (same three indices, same order) also land here:
    /// both traverse each shared edge the same way. That is a duplicate-face
    /// defect rather than an orientation one — `crate::validate_mesh`'s
    /// `duplicate_face_count` distinguishes them, and `crate::repair_mesh`
    /// removes duplicates.
    pub inconsistent_edges: usize,
    /// Distinct faces incident to at least one inconsistent edge.
    ///
    /// Bounded by `inconsistent_edges` within a factor of ~3 either way, so it
    /// is a coarse locality hint, not a concentration metric.
    pub faces_on_inconsistent_edges: usize,
    /// Edges with exactly one incident face — holes and open boundaries.
    pub boundary_edges: usize,
    /// Edges with more than two incident faces. Excluded from the judgement
    /// above; see the type docs.
    pub non_manifold_edges: usize,
    /// Faces listing the same vertex index twice. Skipped whole, and excluded
    /// from every edge counter above; see the type docs.
    pub degenerate_faces: usize,
}

impl WindingCensus {
    /// Whether any interior edge is traversed the same way by both its faces.
    ///
    /// This direction is **sound as "a defect exists"**: `true` means two faces
    /// genuinely walk a shared edge the same way. It does not by itself
    /// distinguish a flipped face from a duplicated one — see
    /// `inconsistent_edges`. The negation is only meaningful when
    /// [`Self::has_judgeable_edges`] holds.
    #[must_use]
    pub const fn has_inconsistent_winding(&self) -> bool {
        self.inconsistent_edges > 0
    }

    /// Whether **any** edge was in a position to be judged — that is, whether
    /// `interior_edges` is non-zero.
    ///
    /// `false` means the mesh is soup (or a single triangle): a zero
    /// `inconsistent_edges` from such a mesh reports that no edge was
    /// examined, not that the winding is good.
    ///
    /// ⚠ **`true` is not a coverage claim.** A surface that is 99 % boundary
    /// edges with a single interior edge reports `true`, having judged one
    /// edge. This method answers non-vacuity only. If you need to know how
    /// much of the surface the census actually saw, compare `interior_edges`
    /// against `boundary_edges` yourself — the counters are there precisely
    /// so that judgement stays with the caller.
    #[must_use]
    pub const fn has_judgeable_edges(&self) -> bool {
        self.interior_edges > 0
    }
}

impl std::fmt::Display for WindingCensus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "winding: {} of {} interior edges inconsistent ({} faces) | \
             boundary-edges {} non-manifold-edges {} degenerate-faces {}{}",
            self.inconsistent_edges,
            self.interior_edges,
            self.faces_on_inconsistent_edges,
            self.boundary_edges,
            self.non_manifold_edges,
            self.degenerate_faces,
            if self.has_judgeable_edges() {
                ""
            } else {
                " | INCONCLUSIVE: no edge has two incident faces"
            },
        )
    }
}

/// Census a mesh's **local** orientation consistency, edge by edge.
///
/// See [`WindingCensus`] for what the counters mean, what is excluded, and why
/// this is not the same question as `is_inside_out`. Report-only: this function
/// classifies, it never refuses and never mutates.
///
/// Runs in one pass over the faces: `O(faces)` time, `O(edges)` space.
///
/// # Example
///
/// ```
/// use mesh_repair::winding_census;
/// use mesh_types::{IndexedMesh, Point3};
///
/// // Two triangles sharing edge (0,1), the second one flipped: both traverse
/// // 0 -> 1, so the shared edge is inconsistent.
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
/// mesh.vertices.push(Point3::new(0.5, -1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
/// mesh.faces.push([0, 1, 3]);
///
/// let census = winding_census(&mesh);
/// assert_eq!(census.inconsistent_edges, 1);
/// assert_eq!(census.faces_on_inconsistent_edges, 2);
/// assert_eq!(census.boundary_edges, 4);
/// ```
#[must_use]
pub fn winding_census(mesh: &IndexedMesh) -> WindingCensus {
    // Per undirected edge: how many incident faces traverse it low -> high,
    // how many traverse it high -> low, and which faces they are. A
    // consistently oriented interior edge has exactly one of each.
    #[derive(Default)]
    struct EdgeUse {
        forward: usize,
        backward: usize,
        /// The first two incident faces. Only ever read for edges that turn
        /// out to be interior — exactly two faces — so a fixed pair suffices
        /// and the census stays allocation-free per edge.
        faces: [usize; 2],
    }

    let mut edges: HashMap<(u32, u32), EdgeUse> = HashMap::new();
    let mut degenerate_faces = 0;

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        // ★ Skip a face listing the same vertex twice AS A WHOLE, not corner by
        // corner. Such a face walks one undirected edge in BOTH directions by
        // itself — `[a, a, b]` contributes `a -> b` and `b -> a` to the single
        // edge `a-b` — so counting its corners would credit that edge with two
        // traversals from ONE face and classify it as a two-faced interior
        // edge. Skipping the whole face is what makes the invariant below hold:
        //
        //   for every surviving edge, traversal count == distinct incident faces
        //
        // because a face with three distinct indices touches each of its three
        // edges exactly once. The match on `forward + backward` is only a valid
        // reading of "how many faces meet here" while that invariant holds.
        if face[0] == face[1] || face[1] == face[2] || face[0] == face[2] {
            degenerate_faces += 1;
            continue;
        }
        for corner in 0..3 {
            let a = face[corner];
            let b = face[(corner + 1) % 3];
            let use_ = edges.entry((a.min(b), a.max(b))).or_default();
            let seen = use_.forward + use_.backward;
            if a < b {
                use_.forward += 1;
            } else {
                use_.backward += 1;
            }
            if seen < 2 {
                use_.faces[seen] = face_idx;
            }
        }
    }

    let mut census = WindingCensus {
        degenerate_faces,
        ..WindingCensus::default()
    };
    let mut damaged_faces: HashSet<usize> = HashSet::new();

    for use_ in edges.values() {
        match use_.forward + use_.backward {
            1 => census.boundary_edges += 1,
            2 => {
                census.interior_edges += 1;
                // Opposite traversal is one each way; anything else means both
                // faces walked the edge the same direction.
                if use_.forward != 1 {
                    census.inconsistent_edges += 1;
                    damaged_faces.extend(use_.faces);
                }
            }
            _ => census.non_manifold_edges += 1,
        }
    }

    census.faces_on_inconsistent_edges = damaged_faces.len();
    census
}

#[cfg(test)]
#[allow(
    // Tests legitimately use `.unwrap()` and `panic!()` for control
    // flow (asserting an Option is Some, asserting an unreachable
    // arm). Workspace lints flip these to deny in production code,
    // but the test mod is exempt.
    clippy::unwrap_used,
    clippy::panic
)]
mod tests {
    use super::*;
    use crate::validate_mesh;
    use mesh_types::Point3;

    #[test]
    fn test_already_consistent() {
        // Tetrahedron with consistent winding
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.5, 1.0));

        // All faces consistently oriented (every edge traversed once each
        // way). Inward-facing, not outward as this comment once claimed —
        // `validate_mesh` reports `is_inside_out` for this face order.
        mesh.faces.push([0, 1, 2]); // Bottom
        mesh.faces.push([0, 3, 1]); // Front
        mesh.faces.push([1, 3, 2]); // Right
        mesh.faces.push([2, 3, 0]); // Left

        let result = fix_winding_order(&mut mesh);
        assert!(result.is_ok());
        // May or may not flip depending on starting face, but should be consistent
    }

    #[test]
    fn test_fix_inconsistent() {
        // Two triangles sharing an edge, one with wrong winding
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, -1.0, 0.0));

        mesh.faces.push([0, 1, 2]); // CCW
        mesh.faces.push([0, 1, 3]); // Wrong: should be [1, 0, 3] for consistent winding

        fix_winding_order(&mut mesh).unwrap();

        // Check that edge (0,1) is now traversed in opposite directions
        let f0 = mesh.faces[0];
        let f1 = mesh.faces[1];

        let dir0 = edge_direction_in_face(&f0, 0, 1);
        let dir1 = edge_direction_in_face(&f1, 0, 1);

        // They should be opposite
        match (dir0, dir1) {
            (Some(d0), Some(d1)) => assert_ne!(d0, d1),
            _ => panic!("Edge should exist in both faces"),
        }
    }

    #[test]
    fn test_fix_disconnected_components() {
        // Two disconnected components, each with inconsistent winding
        let mut mesh = IndexedMesh::new();

        // Component 1: Two triangles sharing edge (0,1)
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, -1.0, 0.0));
        mesh.faces.push([0, 1, 2]); // CCW
        mesh.faces.push([0, 1, 3]); // Wrong winding

        // Component 2: Two triangles sharing edge (4,5), disconnected from component 1
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(11.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(10.5, -1.0, 0.0));
        mesh.faces.push([4, 5, 6]); // CCW
        mesh.faces.push([4, 5, 7]); // Wrong winding

        fix_winding_order(&mut mesh).unwrap();

        // Check both components have consistent winding
        // Component 1: edge (0,1) should be opposite in faces 0 and 1
        let f0 = mesh.faces[0];
        let f1 = mesh.faces[1];
        let dir0 = edge_direction_in_face(&f0, 0, 1);
        let dir1 = edge_direction_in_face(&f1, 0, 1);
        match (dir0, dir1) {
            (Some(d0), Some(d1)) => assert_ne!(d0, d1, "Component 1 winding inconsistent"),
            _ => panic!("Edge should exist in both faces of component 1"),
        }

        // Component 2: edge (4,5) should be opposite in faces 2 and 3
        let f2 = mesh.faces[2];
        let f3 = mesh.faces[3];
        let dir2 = edge_direction_in_face(&f2, 4, 5);
        let dir3 = edge_direction_in_face(&f3, 4, 5);
        match (dir2, dir3) {
            (Some(d2), Some(d3)) => assert_ne!(d2, d3, "Component 2 winding inconsistent"),
            _ => panic!("Edge should exist in both faces of component 2"),
        }
    }

    #[test]
    fn test_empty_mesh() {
        let mut mesh = IndexedMesh::new();
        let result = fix_winding_order(&mut mesh);
        assert!(result.is_ok());
    }

    #[test]
    fn test_single_face() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let result = fix_winding_order(&mut mesh);
        assert!(result.is_ok());
        // Single face should remain unchanged
    }

    #[test]
    fn test_count_inconsistent() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, -1.0, 0.0));

        mesh.faces.push([0, 1, 2]); // CCW
        mesh.faces.push([0, 1, 3]); // Wrong winding

        let count = count_inconsistent_faces(&mesh);
        assert_eq!(count, 1);
    }

    /// Anchor the per-face index swap on a single triangle. `[a, b, c]`
    /// must become `[a, c, b]` exactly; vertex array must remain
    /// byte-identical (positions are not touched).
    #[test]
    fn test_flip_winding_swaps_indices_one_and_two() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        let original_vertices = mesh.vertices.clone();
        mesh.faces.push([0, 1, 2]);

        flip_winding(&mut mesh);
        assert_eq!(mesh.faces[0], [0, 2, 1]);
        assert_eq!(mesh.vertices, original_vertices);
    }

    /// `flip_winding` is its own inverse — applying it twice returns the
    /// mesh to its original state. Anchored as the load-bearing
    /// idempotence-under-double-application property.
    #[test]
    fn test_flip_winding_is_self_inverse() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.5, 1.0));
        // Tetrahedron, four faces with mixed winding so no symmetry hides
        // the property.
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 3, 1]);
        mesh.faces.push([1, 3, 2]);
        mesh.faces.push([2, 3, 0]);
        let original_faces = mesh.faces.clone();

        flip_winding(&mut mesh);
        assert_ne!(mesh.faces, original_faces, "first flip should change faces");
        flip_winding(&mut mesh);
        assert_eq!(
            mesh.faces, original_faces,
            "double flip should restore original face order",
        );
    }

    /// Empty mesh (no faces, no vertices) must not panic and must remain
    /// empty. Edge case for the `for face in &mut mesh.faces` loop.
    #[test]
    fn test_flip_winding_empty_mesh() {
        let mut mesh = IndexedMesh::new();
        flip_winding(&mut mesh);
        assert_eq!(mesh.vertices.len(), 0);
        assert_eq!(mesh.faces.len(), 0);
    }

    /// Soup mesh (every triangle disconnected — no shared edges by index)
    /// is the load-bearing use case `flip_winding` exists for.
    /// `fix_winding_order` is a no-op on soup; `flip_winding` flips every
    /// face independently. Anchored as the contract that distinguishes
    /// the two operations.
    #[test]
    fn test_flip_winding_works_on_soup_mesh() {
        let mut mesh = IndexedMesh::new();
        // 3 disconnected triangles — 9 verts total, no shared indices.
        for i in 0u32..3 {
            let off = f64::from(i) * 10.0;
            mesh.vertices.push(Point3::new(off, 0.0, 0.0));
            mesh.vertices.push(Point3::new(off + 1.0, 0.0, 0.0));
            mesh.vertices.push(Point3::new(off, 1.0, 0.0));
            mesh.faces.push([3 * i, 3 * i + 1, 3 * i + 2]);
        }

        flip_winding(&mut mesh);
        assert_eq!(mesh.faces[0], [0, 2, 1]);
        assert_eq!(mesh.faces[1], [3, 5, 4]);
        assert_eq!(mesh.faces[2], [6, 8, 7]);
    }

    // ── winding_census ────────────────────────────────────────────────
    //
    // The instrument these gates exercise exists because `is_inside_out`
    // (signed volume) answers a different question, and answers it in a
    // coordinate-dependent way once any face is flipped — see
    // `signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin`. The pair
    // `..._is_blind_to_a_global_flip...` /
    // `..._counts_three_edges_per_flipped_face_wherever_it_sits` pins the
    // orthogonality in BOTH directions. Each gate below is built so it can
    // fail for the reason it names.

    /// A closed octahedron whose eight faces are consistently oriented:
    /// every one of the twelve edges is traversed once each way. Verified as
    /// such by the first assertion of each gate that uses it, so a gate
    /// measuring a defect always starts from a clean baseline.
    ///
    /// ⚠ **Why not the tetrahedron this started as.** The census's headline
    /// claim is that it is seed-independent where [`count_inconsistent_faces`]
    /// is not: one flipped face reports three edges, while the seed-relative
    /// count reports `1` or `n - 1` depending on where BFS began. A closed
    /// triangulation has `F = 2V - 4`, so `F` is 4, 6, 8, …, and **`F = 4` is
    /// the one size where `n - 1 == 3`** — on a tetrahedron the two
    /// instruments return the same number and no assertion can separate them.
    /// Substituting `count_inconsistent_faces` into the gate below therefore
    /// left it green.
    ///
    /// Eight faces is not the minimum: a triangular bipyramid (`V = 5`,
    /// `F = 6`) also separates them. The octahedron is simply the fixture
    /// chosen; the property that matters is `F != 4`.
    fn consistent_octahedron() -> IndexedMesh {
        consistent_octahedron_at(0.0)
    }

    /// The same octahedron translated along z, for the gate that measures how
    /// `is_inside_out` responds to distance from the world origin.
    fn consistent_octahedron_at(z: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        for v in [
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, -1.0),
        ] {
            mesh.vertices.push(Point3::new(v.0, v.1, v.2 + z));
        }
        // Upper cap fan around vertex 4, then lower cap fan around vertex 5.
        for f in [
            [0, 2, 4],
            [2, 1, 4],
            [1, 3, 4],
            [3, 0, 4],
            [2, 0, 5],
            [1, 2, 5],
            [3, 1, 5],
            [0, 3, 5],
        ] {
            mesh.faces.push(f);
        }
        mesh
    }

    /// Baseline: a consistently oriented closed surface reports twelve
    /// interior edges, none inconsistent, and says so conclusively.
    #[test]
    fn winding_census_is_clean_on_a_consistent_closed_surface() {
        let mesh = consistent_octahedron();
        let census = winding_census(&mesh);
        assert_eq!(census.interior_edges, 12, "octahedron has 12 edges");
        assert_eq!(census.inconsistent_edges, 0);
        assert_eq!(census.faces_on_inconsistent_edges, 0);
        assert_eq!(census.boundary_edges, 0, "closed surface has no boundary");
        assert_eq!(census.non_manifold_edges, 0);
        assert_eq!(census.degenerate_faces, 0);
        assert!(census.has_judgeable_edges());
        assert!(!census.has_inconsistent_winding());
        // The fixture is outward-facing. Asserted directly rather than as
        // "unchanged", so this file pins the POLARITY of `is_inside_out`
        // somewhere — otherwise inverting `check_inside_out` leaves every
        // gate here green.
        assert!(
            !validate_mesh(&mesh).is_inside_out,
            "the fixture is wound outward (signed volume +4/3)",
        );

        // Euler characteristic, from an edge count computed WITHOUT the
        // census. An earlier version of this assert took E from
        // `census.interior_edges` — which the assert above had already pinned
        // to 12 — so it reduced to `6 + 8 - 12 == 2` and no production change
        // could falsify it, while its comment called it an independent check.
        // A cross-check must not route through the artifact under test.
        let mut distinct_edges: HashSet<(u32, u32)> = HashSet::new();
        for face in &mesh.faces {
            for corner in 0..3 {
                let (a, b) = (face[corner], face[(corner + 1) % 3]);
                distinct_edges.insert((a.min(b), a.max(b)));
            }
        }
        assert_eq!(
            mesh.vertices.len() + mesh.faces.len() - distinct_edges.len(),
            2,
            "V - E + F = 2 for a closed genus-0 surface",
        );
        assert_eq!(
            distinct_edges.len(),
            census.interior_edges,
            "and the census must agree with that independent edge count",
        );
    }

    /// ★ The gate that replaces a false claim with a measurement.
    ///
    /// This file used to assert that `is_inside_out` is "structurally
    /// incapable" of seeing a locally flipped face. It is not. The
    /// signed-volume sum is translation-invariant only while the surface is
    /// consistently oriented; once one face is flipped, that face's term grows
    /// with distance from the world origin and eventually dominates.
    ///
    /// The transition is measured here rather than asserted in prose. It
    /// matters because meshes kept in a native anatomical frame sit ~1e3 units
    /// from the origin — the far end of this sweep, not the near end.
    #[test]
    fn signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin() {
        let mut saw_missed = false;
        let mut saw_caught = false;

        for (z, expected_after_flip) in [
            (0.0, false),
            (1.0, false),
            (2.0, false),
            (3.0, false),
            (4.0, true),
            (10.0, true),
            (970.0, true),
        ] {
            let clean = consistent_octahedron_at(z);
            assert!(
                !validate_mesh(&clean).is_inside_out,
                "z={z}: the unflipped fixture is outward at every offset — \
                 translation-invariance holds while winding is consistent",
            );

            let mut flipped = consistent_octahedron_at(z);
            let before = flipped.faces[0];
            flipped.faces[0].swap(1, 2);
            assert_ne!(flipped.faces[0], before, "z={z}: the flip must land");

            assert_eq!(
                validate_mesh(&flipped).is_inside_out,
                expected_after_flip,
                "z={z}: one flipped face, signed-volume flag",
            );
            // The census is coordinate-free: same answer at every offset.
            assert_eq!(
                winding_census(&flipped).inconsistent_edges,
                3,
                "z={z}: the local census does not depend on where the mesh sits",
            );

            if expected_after_flip {
                saw_caught = true;
            } else {
                saw_missed = true;
            }
        }

        // Non-vacuity: the sweep must actually straddle the transition, or it
        // is not evidence of coordinate-dependence at all.
        assert!(
            saw_missed && saw_caught,
            "the sweep must contain offsets on BOTH sides of the transition",
        );
    }

    /// ★ The load-bearing gate. One flipped face reports **exactly its own
    /// three edges, wherever it sits** — where the signed-volume test reports
    /// nothing at all and the seed-relative face count reports two different
    /// numbers for the same defect.
    ///
    /// The three-way contrast is the whole claim:
    ///
    /// | | flip face 0 | flip face 3 |
    /// |---|---|---|
    /// | `winding_census` | 3 edges | 3 edges |
    /// | `count_inconsistent_faces` | 7 | 1 |
    /// | `is_inside_out` | unmoved | unmoved |
    ///
    /// The middle row is why the census exists: BFS seeds at face 0, so
    /// flipping face 0 makes the *other seven* look wrong, while flipping any
    /// other face makes only that face look wrong. Both describe one flipped
    /// triangle. Only the census answers with the defect rather than with the
    /// repair.
    ///
    /// Every one of the eight positions is swept — an earlier version sampled
    /// two, and both happened to lie in the upper cap fan, so the name
    /// "wherever it sits" outran the test.
    #[test]
    fn winding_census_counts_three_edges_per_flipped_face_wherever_it_sits() {
        for flipped in 0..consistent_octahedron().faces.len() {
            let mut mesh = consistent_octahedron();

            // The baseline must be clean, or a non-zero reading below would
            // not be attributable to the flip.
            assert_eq!(winding_census(&mesh).inconsistent_edges, 0);
            let volume_flag_before = validate_mesh(&mesh).is_inside_out;

            // Inject the defect, and assert the injection landed.
            let before = mesh.faces[flipped];
            mesh.faces[flipped].swap(1, 2);
            assert_ne!(
                mesh.faces[flipped], before,
                "the flip must actually change face {flipped}",
            );

            let census = winding_census(&mesh);
            assert_eq!(
                census.inconsistent_edges, 3,
                "face {flipped}: a flipped triangle disagrees with its \
                 neighbours on exactly its own three edges, regardless of \
                 where in the face list it sits",
            );
            assert_eq!(
                census.faces_on_inconsistent_edges, 4,
                "face {flipped}: the flipped face plus its three neighbours",
            );
            assert_eq!(census.interior_edges, 12, "topology is unchanged by a swap");
            assert!(census.has_inconsistent_winding());

            // THE GAP THIS INSTRUMENT EXISTS FOR: signed volume is unmoved.
            //
            // ⚠ That is a property of THIS FIXTURE, not a general one: the
            // octahedron is origin-centred, where the flipped face's term is
            // small relative to the enclosed volume. Translate it and the flag
            // does move — see
            // `signed_volume_sees_a_local_flip_once_the_mesh_is_off_origin`.
            assert_eq!(
                validate_mesh(&mesh).is_inside_out,
                volume_flag_before,
                "face {flipped}: on this origin-centred fixture is_inside_out \
                 must not move for a local flip",
            );

            // And the seed-relative count gives a DIFFERENT answer for each
            // flip position, while the census above gave the same one twice.
            assert_eq!(
                count_inconsistent_faces(&mesh),
                if flipped == 0 { 7 } else { 1 },
                "face {flipped}: BFS seeds at face 0, so flipping the seed \
                 makes every other face look wrong and flipping a non-seed \
                 face makes only itself look wrong",
            );
        }
    }

    /// The other direction of the same orthogonality: a **globally** flipped
    /// mesh is locally consistent everywhere — every edge still has one
    /// traversal each way — while signed volume flips sign.
    ///
    /// Together with the gate above this pins both instruments: neither
    /// subsumes the other, and a caller needs both readings.
    #[test]
    fn winding_census_is_blind_to_a_global_flip_where_signed_volume_is_not() {
        let mut mesh = consistent_octahedron();
        let before_volume_flag = validate_mesh(&mesh).is_inside_out;
        assert_eq!(winding_census(&mesh).inconsistent_edges, 0);

        let faces_before = mesh.faces.clone();
        flip_winding(&mut mesh);
        // Without this, a `flip_winding` that became a no-op would leave the
        // census assertion below passing for the wrong reason.
        assert_ne!(mesh.faces, faces_before, "the global flip must land");

        assert_eq!(
            winding_census(&mesh).inconsistent_edges,
            0,
            "a global flip is still a consistent orientation, just the other one",
        );
        assert_ne!(
            validate_mesh(&mesh).is_inside_out,
            before_volume_flag,
            "signed volume must negate under a global flip — this is the case \
             is_inside_out DOES catch and the census does not",
        );
    }

    /// Soup has no interior edges, so it has nothing to judge. The census
    /// says so; `count_inconsistent_faces` returns a clean-looking `0` for
    /// the same mesh, which is the failure mode `has_judgeable_edges` exists to
    /// make visible.
    ///
    /// This matters on the STL path specifically: an STL is soup until it is
    /// welded, so a winding census run before the weld can never report a
    /// defect no matter how badly wound the mesh is.
    #[test]
    fn winding_census_reports_soup_as_inconclusive_not_clean() {
        let mut mesh = IndexedMesh::new();
        // Three disconnected triangles, deliberately mixed winding — no
        // shared vertex indices, so no edge has two faces.
        for i in 0u32..3 {
            let off = f64::from(i) * 10.0;
            mesh.vertices.push(Point3::new(off, 0.0, 0.0));
            mesh.vertices.push(Point3::new(off + 1.0, 0.0, 0.0));
            mesh.vertices.push(Point3::new(off, 1.0, 0.0));
            if i == 1 {
                mesh.faces.push([3 * i, 3 * i + 2, 3 * i + 1]);
            } else {
                mesh.faces.push([3 * i, 3 * i + 1, 3 * i + 2]);
            }
        }

        let census = winding_census(&mesh);
        assert_eq!(census.interior_edges, 0, "soup shares no edges");
        assert_eq!(census.boundary_edges, 9, "3 triangles x 3 unshared edges");
        assert!(
            !census.has_judgeable_edges(),
            "nothing was checked, so a zero must not read as a clean bill",
        );
        assert_eq!(census.inconsistent_edges, 0, "vacuously, not verifiably");

        // The instrument that reads this as simply fine:
        assert_eq!(
            count_inconsistent_faces(&mesh),
            0,
            "BFS finds no neighbours on soup, so it reports clean",
        );
    }

    /// An edge with three incident faces has no unique "do both agree?"
    /// answer. It is reported in its own counter and kept out of the
    /// interior/inconsistent judgement rather than folded into a number that
    /// would read clean.
    #[test]
    fn winding_census_excludes_non_manifold_edges_from_the_judgement() {
        let mut mesh = IndexedMesh::new();
        for i in 0..5 {
            mesh.vertices.push(Point3::new(f64::from(i), 0.0, 0.0));
        }
        // Three faces all sharing edge (0,1) — a T-junction fin.
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([1, 0, 3]);
        mesh.faces.push([0, 1, 4]);

        let census = winding_census(&mesh);
        assert_eq!(census.non_manifold_edges, 1, "edge (0,1) has three faces");
        assert_eq!(
            census.interior_edges, 0,
            "the non-manifold edge must not be judged as interior",
        );
        assert_eq!(census.inconsistent_edges, 0);
        assert_eq!(census.boundary_edges, 6, "each fin contributes two");
        assert!(!census.has_judgeable_edges());
    }

    /// ★★ A face listing the same vertex twice is skipped **whole**, and must
    /// contribute to no edge counter at all.
    ///
    /// ⚠ This gate previously asserted the opposite. `[0, 0, 1]` walks the
    /// single edge `0-1` in both directions by itself, so counting its corners
    /// credited that edge with two traversals and classified it as a
    /// **two-faced interior edge** — from one face. The old gate asserted
    /// `interior_edges == 1` and called it "surprising, not load-bearing".
    ///
    /// It was load-bearing. `has_judgeable_edges()` exists to distinguish "no
    /// inconsistent edges" from "nothing was checked", and a mesh of such
    /// slivers defeated it — see the sibling gate below.
    #[test]
    fn winding_census_skips_a_degenerate_face_whole() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.faces.push([0, 0, 1]);

        let census = winding_census(&mesh);
        assert_eq!(census.degenerate_faces, 1, "the face is counted");
        assert_eq!(
            (census.interior_edges, census.boundary_edges),
            (0, 0),
            "and contributes NO edges — one face cannot make an interior edge",
        );
        assert_eq!(census.non_manifold_edges, 0);
        assert_eq!(census.inconsistent_edges, 0);
        assert!(
            !census.has_judgeable_edges(),
            "nothing was judged, and the census must say so",
        );
    }

    /// ★★ The consequence the gate above exists to prevent: a mesh of
    /// degenerate slivers must not read as a clean **closed** surface.
    ///
    /// Before the whole-face skip this returned `interior_edges 3`,
    /// `boundary_edges 0`, `inconsistent_edges 0` and
    /// `has_judgeable_edges() == true` — a conclusive clean bill on a mesh
    /// where not one cross-face comparison had happened. Found by an
    /// adversarial review, not by the gates.
    #[test]
    fn winding_census_does_not_fabricate_a_closed_surface_from_slivers() {
        let mut mesh = IndexedMesh::new();
        for i in 0..4 {
            mesh.vertices.push(Point3::new(f64::from(i), 0.0, 0.0));
        }
        mesh.faces.push([0, 0, 1]);
        mesh.faces.push([1, 1, 2]);
        mesh.faces.push([2, 2, 3]);

        let census = winding_census(&mesh);
        assert_eq!(census.degenerate_faces, 3);
        assert_eq!(census.interior_edges, 0, "no edge has two incident faces");
        assert!(
            !census.has_judgeable_edges(),
            "a mesh nothing was checked on must never read as judged: {census}",
        );
    }

    /// ★★ And the same root cause in the other direction: a degenerate sliver
    /// beside a real face must not fabricate a **non-manifold** edge.
    ///
    /// `non_manifold_edges` is the counter a consumer reads to decide whether
    /// a pseudo-normal sign is defined at all, so inflating it condemns
    /// surfaces that are sound. Before the whole-face skip, edge `0-1` here
    /// collected two traversals from the sliver plus one from the real face
    /// and was reported non-manifold, though only two faces touch it.
    #[test]
    fn winding_census_does_not_fabricate_a_non_manifold_edge_from_a_sliver() {
        let mut mesh = IndexedMesh::new();
        for i in 0..3 {
            mesh.vertices.push(Point3::new(f64::from(i), 0.0, 0.0));
        }
        mesh.faces.push([0, 0, 1]);
        mesh.faces.push([0, 1, 2]);

        let census = winding_census(&mesh);
        assert_eq!(
            census.non_manifold_edges, 0,
            "only two faces touch edge 0-1, and one of them is degenerate: {census}",
        );
        assert_eq!(census.degenerate_faces, 1);
        assert_eq!(
            census.boundary_edges, 3,
            "the one real face contributes its three edges, all unshared",
        );
    }
}
