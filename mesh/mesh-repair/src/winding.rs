//! Normal consistency and winding order correction.
//!
//! This module provides tools for fixing inconsistent face winding in meshes.
//! Consistent winding is required for correct normal computation and rendering.
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
/// test: it integrates the divergence theorem over the whole surface and asks
/// whether the total came out negative. That detects a mesh whose winding was
/// reversed *everywhere*. It is structurally incapable of seeing a handful of
/// locally flipped faces on an otherwise-correct mesh — a few flipped cap fans
/// contribute a small negative term that a correctly-wound body outweighs, and
/// the global sign never changes.
///
/// This census asks the local question instead. On a consistently oriented
/// surface every interior edge is traversed in **opposite** directions by its
/// two incident faces. An edge traversed the *same* way by both is a local
/// orientation defect, whatever the global volume says. The two instruments are
/// orthogonal and neither subsumes the other: a globally flipped mesh is
/// perfectly consistent edge-by-edge (`inconsistent_edges == 0`) while
/// `is_inside_out` fires, and a locally flipped fan does the reverse.
///
/// # Seed-independence
///
/// Unlike [`count_inconsistent_faces`], nothing here depends on a traversal
/// order or a starting face — each edge is classified from its own two faces.
/// A single flipped triangle always reports exactly its own three edges.
///
/// # What is deliberately excluded
///
/// * **Non-manifold edges** (more than two incident faces) are counted in
///   `non_manifold_edges` and excluded from `interior_edges` /
///   `inconsistent_edges`. "Both faces agree" has no unique meaning when there
///   are three of them, and this is exactly the input class on which a
///   pseudo-normal sign oracle is undefined by contract — so it is reported,
///   not folded into a number that would read clean.
/// * **Degenerate edges** (a face listing the same vertex twice) are counted in
///   `degenerate_edges` and excluded. They are not edges.
///
/// # Vacuity
///
/// A mesh with no interior edges — triangle soup, or a single triangle — has
/// nothing to check, and reports `inconsistent_edges == 0` for that reason
/// rather than because its winding was verified. Use [`Self::is_conclusive`]
/// before reading a zero as good news.
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
/// assert!(census.is_conclusive());
///
/// // A global flip stays locally consistent — every edge still has one
/// // traversal each way.
/// flip_winding(&mut mesh);
/// assert_eq!(winding_census(&mesh).inconsistent_edges, 0);
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct WindingCensus {
    /// Edges with exactly two incident faces — the edges this census can judge.
    pub interior_edges: usize,
    /// Interior edges traversed in the **same** direction by both incident
    /// faces. Zero on a consistently oriented surface.
    pub inconsistent_edges: usize,
    /// Distinct faces incident to at least one inconsistent edge.
    ///
    /// Locality: one flipped triangle in an otherwise-clean mesh touches four
    /// faces (itself and its three neighbours), whereas a flipped region of `n`
    /// faces touches only its boundary. A large `inconsistent_edges` with a
    /// small face count means the damage is concentrated.
    pub faces_on_inconsistent_edges: usize,
    /// Edges with exactly one incident face — holes and open boundaries.
    pub boundary_edges: usize,
    /// Edges with more than two incident faces. Excluded from the judgement
    /// above; see the type docs.
    pub non_manifold_edges: usize,
    /// Face corners whose two endpoints are the same vertex index. Excluded;
    /// see the type docs.
    pub degenerate_edges: usize,
}

impl WindingCensus {
    /// Whether any interior edge is traversed the same way by both its faces.
    ///
    /// This direction is **sound**: `true` means a local orientation defect was
    /// observed. The negation is only meaningful when [`Self::is_conclusive`]
    /// holds.
    #[must_use]
    pub const fn has_inconsistent_winding(&self) -> bool {
        self.inconsistent_edges > 0
    }

    /// Whether there was anything to judge — at least one interior edge.
    ///
    /// `false` means the mesh is soup (or a single triangle): a zero
    /// `inconsistent_edges` from such a mesh reports that no edge was
    /// examined, not that the winding is good.
    #[must_use]
    pub const fn is_conclusive(&self) -> bool {
        self.interior_edges > 0
    }
}

impl std::fmt::Display for WindingCensus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "winding: {} of {} interior edges inconsistent ({} faces) | \
             boundary {} non-manifold {} degenerate {}{}",
            self.inconsistent_edges,
            self.interior_edges,
            self.faces_on_inconsistent_edges,
            self.boundary_edges,
            self.non_manifold_edges,
            self.degenerate_edges,
            if self.is_conclusive() {
                ""
            } else {
                " | INCONCLUSIVE: no interior edges (soup?)"
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
/// Runs in one pass over the faces, `O(faces)` time and edges of space.
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
        faces: Vec<usize>,
    }

    let mut edges: HashMap<(u32, u32), EdgeUse> = HashMap::new();
    let mut degenerate_edges = 0;

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        for corner in 0..3 {
            let a = face[corner];
            let b = face[(corner + 1) % 3];
            if a == b {
                degenerate_edges += 1;
                continue;
            }
            let use_ = edges.entry((a.min(b), a.max(b))).or_default();
            if a < b {
                use_.forward += 1;
            } else {
                use_.backward += 1;
            }
            use_.faces.push(face_idx);
        }
    }

    let mut census = WindingCensus {
        degenerate_edges,
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
                    damaged_faces.extend(use_.faces.iter().copied());
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
    // (signed volume) is a GLOBAL test: it cannot see a locally flipped
    // region on an otherwise-correct mesh. Each gate below is built so it
    // can fail for the reason it names, and the pair
    // `..._is_blind_to_a_global_flip...` / `..._counts_exactly_the_three_
    // edges...` pins the orthogonality in BOTH directions.

    /// A closed tetrahedron whose four faces are consistently oriented:
    /// every one of the six edges is traversed once each way. Verified as
    /// such by the first assertion of each gate that uses it, so a gate
    /// measuring a defect always starts from a clean baseline.
    ///
    /// ⚠ Consistent, but **inward**-facing: `validate_mesh` reports
    /// `is_inside_out == true` for this face order (measured — an earlier
    /// draft of these gates asserted the opposite and failed). Consistency is
    /// what the census judges and polarity is what `is_inside_out` judges;
    /// this fixture deliberately separates the two, so gates here assert how
    /// the global flag *moves*, never what it equals.
    fn consistent_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 3, 1]);
        mesh.faces.push([1, 3, 2]);
        mesh.faces.push([2, 3, 0]);
        mesh
    }

    /// Baseline: a consistently oriented closed surface reports six interior
    /// edges, none inconsistent, and says so conclusively.
    #[test]
    fn winding_census_is_clean_on_a_consistent_closed_surface() {
        let census = winding_census(&consistent_tetrahedron());
        assert_eq!(census.interior_edges, 6, "tetrahedron has 6 edges");
        assert_eq!(census.inconsistent_edges, 0);
        assert_eq!(census.faces_on_inconsistent_edges, 0);
        assert_eq!(census.boundary_edges, 0, "closed surface has no boundary");
        assert_eq!(census.non_manifold_edges, 0);
        assert_eq!(census.degenerate_edges, 0);
        assert!(census.is_conclusive());
        assert!(!census.has_inconsistent_winding());
    }

    /// ★ The load-bearing gate. One flipped face must report **exactly its
    /// own three edges** — seed-independently — where the signed-volume test
    /// reports nothing at all.
    ///
    /// The contrast with `count_inconsistent_faces` is the point, not a
    /// footnote: that function answers "how many faces would the repair
    /// swap?", which depends on which face BFS happened to seed from. Here
    /// it seeds on the flipped face itself and therefore reports the other
    /// three as the wrong ones. Both numbers describe one flipped triangle;
    /// only the edge count says which three edges.
    #[test]
    fn winding_census_counts_exactly_the_three_edges_of_one_flipped_face() {
        let mut mesh = consistent_tetrahedron();

        // The baseline must be clean, or a non-zero reading below would not
        // be attributable to the flip. (The fixture is consistently oriented
        // but INWARD-facing — measured, see `consistent_tetrahedron` — which
        // is why the signed-volume claim below is stated as "unchanged by the
        // flip" rather than as a fixed polarity.)
        assert_eq!(winding_census(&mesh).inconsistent_edges, 0);
        let volume_flag_before = validate_mesh(&mesh).is_inside_out;

        // Inject the defect, and assert the injection landed.
        let before = mesh.faces[0];
        mesh.faces[0].swap(1, 2);
        assert_ne!(
            mesh.faces[0], before,
            "the flip must actually change face 0"
        );

        let census = winding_census(&mesh);
        assert_eq!(
            census.inconsistent_edges, 3,
            "a flipped triangle disagrees with its neighbours on exactly its \
             own three edges",
        );
        assert_eq!(
            census.faces_on_inconsistent_edges, 4,
            "the flipped face plus its three neighbours",
        );
        assert_eq!(census.interior_edges, 6, "topology is unchanged by a swap");
        assert!(census.has_inconsistent_winding());

        // THE GAP THIS INSTRUMENT EXISTS FOR: signed volume is unmoved. One
        // flipped face out of four does not outweigh the other three, so the
        // global flag reads exactly what it read before the defect existed.
        assert_eq!(
            validate_mesh(&mesh).is_inside_out,
            volume_flag_before,
            "is_inside_out is a global test and must not move for a local \
             flip — if it ever starts moving here, the claim that it cannot \
             see local flips is what needs revisiting, not this gate",
        );

        // And the seed-relative count disagrees with the edge count, as
        // documented: BFS seeds at face 0, which IS the flipped face.
        assert_eq!(
            count_inconsistent_faces(&mesh),
            3,
            "seeded on the flipped face, the repair would swap the other three",
        );
    }

    /// The other direction of the same orthogonality: a **globally** flipped
    /// mesh is locally consistent everywhere — every edge still has one
    /// traversal each way — while signed volume flips sign.
    ///
    /// Together with the gate above this pins both instruments: neither
    /// subsumes the other, and a caller needs both readings.
    #[test]
    fn winding_census_is_blind_to_a_global_flip_where_signed_volume_is_not() {
        let mut mesh = consistent_tetrahedron();
        let before_volume_flag = validate_mesh(&mesh).is_inside_out;
        assert_eq!(winding_census(&mesh).inconsistent_edges, 0);

        flip_winding(&mut mesh);

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
    /// the same mesh, which is the failure mode `is_conclusive` exists to
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
            !census.is_conclusive(),
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
        assert!(!census.is_conclusive());
    }

    /// A face listing the same vertex twice contributes a zero-length corner.
    /// It is counted and excluded — not silently dropped.
    #[test]
    fn winding_census_counts_degenerate_corners_separately() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.faces.push([0, 0, 1]);

        let census = winding_census(&mesh);
        assert_eq!(census.degenerate_edges, 1, "the 0 -> 0 corner");
        assert_eq!(census.inconsistent_edges, 0);
    }
}
