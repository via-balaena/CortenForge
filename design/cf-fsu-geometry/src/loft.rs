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
use nalgebra::{Point3, Vector3};

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

/// An assembled bushing: the closed loft mesh plus the boundary between its
/// exact contact caps and its free perimeter wall.
#[derive(Debug, Clone)]
pub struct Bushing {
    /// The closed, outward-oriented bushing mesh.
    pub mesh: IndexedMesh,

    /// The first `contact_count` vertices are the two contact caps (the exact
    /// endplate surfaces — pinned when the wall is smoothed); every vertex at or
    /// after this index is a free wall-interior vertex introduced by the ring
    /// subdivision. With a single-segment wall there are no free vertices and
    /// `contact_count == mesh.vertex_count()`.
    pub contact_count: u32,
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

/// Reorder `rim_b` to correspond to `rim_a`: rotate it to start at the vertex
/// nearest `pa0` (= `rim_a[0]`), then reverse it if that better matches the
/// direction of `rim_a`'s first step toward `pa1` (= `rim_a[1]`). Shared by the
/// wall stitch and the ring-subdivision positioning so both agree on how the
/// two rims line up.
fn orient_rim_b(
    pa0: &Point3<f64>,
    pa1: &Point3<f64>,
    rim_b: &[u32],
    verts: &[Point3<f64>],
) -> Vec<u32> {
    let nb = rim_b.len();
    let pb: Vec<Point3<f64>> = rim_b.iter().map(|&v| verts[v as usize]).collect();
    let k = (0..nb)
        .min_by(|&i, &j| dist(pa0, &pb[i]).total_cmp(&dist(pa0, &pb[j])))
        .unwrap_or(0);
    let fwd = dist(pa1, &pb[(k + 1) % nb]);
    let rev = dist(pa1, &pb[(k + nb - 1) % nb]);
    let reverse = rev < fwd;
    (0..nb)
        .map(|m| {
            let src = if reverse {
                (k + nb - m) % nb
            } else {
                (k + m) % nb
            };
            rim_b[src]
        })
        .collect()
}

/// Resample a closed loop of positions to exactly `r` points spaced uniformly
/// by arc length, starting at the loop's first point. Used to give an interior
/// wall ring a matched vertex count when the two rims differ in resolution.
/// Returns the input unchanged if it has zero perimeter or `r == 0`.
// The step/count casts are exact: loop counts are tiny and lossless in `f64`.
#[allow(clippy::cast_precision_loss)]
fn resample_loop(positions: &[Point3<f64>], count: usize) -> Vec<Point3<f64>> {
    let len = positions.len();
    if len == 0 || count == 0 {
        return positions.to_vec();
    }
    let mut cum = vec![0.0];
    let mut total = 0.0;
    for i in 0..len {
        total += dist(&positions[i], &positions[(i + 1) % len]);
        cum.push(total);
    }
    if total <= 0.0 {
        return positions.to_vec();
    }
    let mut out = Vec::with_capacity(count);
    let mut seg = 0usize;
    for step in 0..count {
        let target = (step as f64 / count as f64) * total;
        while seg + 1 < cum.len() && cum[seg + 1] < target {
            seg += 1;
        }
        let seg_len = cum[seg + 1] - cum[seg];
        let frac = if seg_len > 0.0 {
            (target - cum[seg]) / seg_len
        } else {
            0.0
        };
        let start = positions[seg % len];
        let end = positions[(seg + 1) % len];
        out.push(start + (end - start) * frac);
    }
    out
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
    let b_idx = orient_rim_b(&pa[0], &pa[1], rim_b, verts);
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

/// Assemble a top patch, the perimeter wall, and a bottom patch into one closed
/// [`Bushing`] mesh (B2 / B3a).
///
/// The two patches are concatenated into a shared vertex array (top first, then
/// bottom), the bottom patch's faces are **flipped** so its normals point out
/// the opposite side, and the perimeter wall seals the two outer rims. A final
/// orientation pass flips the whole mesh if it came out inside-out, so the
/// result always has outward-facing normals (`signed_volume > 0`).
///
/// `wall_segments` is the number of stacked wall rings between the two rims.
/// With `1` the wall is a single triangle band directly between the rims (no
/// free vertices). With `n > 1` the wall gains `n - 1` interior rings of **free
/// vertices** (linearly interpolated between the rims), so the free surface can
/// be smoothed or bulged while the contact caps stay exact — see [`Bushing`].
///
/// # Preconditions
///
/// Each patch must be a single-boundary surface disc (exactly one rim; the
/// outer rim `rims[0]` is used) and the two patches must be given in the **same
/// winding convention** — both wound so their normals point the same absolute
/// direction, as [`extract_patch`] yields when both regions are painted with a
/// consistent surface orientation. The bottom flip then makes the two caps face
/// away from each other and the wall seals them. A patch with an interior hole
/// (more than one rim) needs one wall per rim and is out of scope here.
///
/// If either patch has no rim the caps are still concatenated but the mesh is
/// left open (no wall) — the watertight check is the caller's readout.
#[must_use]
// Vertex/ring counts fit in `u32` by the `IndexedMesh` index contract and the
// ring fraction is a tiny lossless integer ratio, so no cast truncates or loses
// precision at any real mesh size.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
pub fn assemble_bushing(top: &Patch, bottom: &Patch, wall_segments: usize) -> Bushing {
    let offset = top.mesh.vertices.len() as u32;
    let contact_count = offset + bottom.mesh.vertices.len() as u32;

    let mut vertices = top.mesh.vertices.clone();
    vertices.extend_from_slice(&bottom.mesh.vertices);

    let mut faces = top.mesh.faces.clone();
    // Bottom faces: reverse winding (flip normal) and shift into the shared array.
    for &[a, b, c] in &bottom.mesh.faces {
        faces.push([c + offset, b + offset, a + offset]);
    }

    if let (Some(rim_a), Some(rim_b)) = (top.rims.first(), bottom.rims.first()) {
        let rim_b_shifted: Vec<u32> = rim_b.iter().map(|&v| v + offset).collect();
        let segments = wall_segments.max(1);
        if segments == 1 {
            faces.extend(stitch_rims(&vertices, rim_a, &rim_b_shifted));
        } else {
            build_ringed_wall(&mut vertices, &mut faces, rim_a, &rim_b_shifted, segments);
        }
    }

    let mut mesh = IndexedMesh::from_parts(vertices, faces);
    // Guarantee outward orientation: flip every face if the solid is inside-out.
    if mesh.is_inside_out() {
        for face in &mut mesh.faces {
            face.swap(1, 2);
        }
    }
    Bushing {
        mesh,
        contact_count,
    }
}

/// Build a subdivided wall of `segments` stacked rings between `rim_a` and
/// `rim_b` (both indexing `vertices`). Appends `segments - 1` interior rings of
/// free vertices (each with `rim_a.len()` points, interpolated toward the
/// arc-length-resampled bottom rim) and the wall faces. Interior rings are
/// stitched to their neighbours as equal-count quad grids; the last interior
/// ring joins `rim_b` via [`stitch_rims`] so unequal rim counts still seal.
// Ring index arithmetic and the ring fraction are lossless at any real mesh size.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
fn build_ringed_wall(
    vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<[u32; 3]>,
    rim_a: &[u32],
    rim_b: &[u32],
    segments: usize,
) {
    let na = rim_a.len();
    let pa: Vec<Point3<f64>> = rim_a.iter().map(|&v| vertices[v as usize]).collect();
    // Bottom rim oriented to match rim_a, then resampled to `na` points so each
    // interior ring vertex `i` interpolates rim_a[i] → sample_b[i] (no twist).
    let b_oriented = orient_rim_b(&pa[0], &pa[1], rim_b, vertices);
    let b_pos: Vec<Point3<f64>> = b_oriented.iter().map(|&v| vertices[v as usize]).collect();
    let sample_b = resample_loop(&b_pos, na);

    let interior_base = vertices.len() as u32;
    for ring in 1..segments {
        let t = ring as f64 / segments as f64;
        for i in 0..na {
            vertices.push(pa[i] + (sample_b[i] - pa[i]) * t);
        }
    }

    // Vertex indices of ring `r` (ring 0 = rim_a; interior rings appended above).
    let ring_indices = |r: usize| -> Vec<u32> {
        if r == 0 {
            rim_a.to_vec()
        } else {
            (0..na)
                .map(|i| interior_base + ((r - 1) * na + i) as u32)
                .collect()
        }
    };

    // Equal-count quad-grid stitch between consecutive na-point rings. The
    // winding matches `stitch_rims`' equal-count output so the whole wall — and
    // its join to the caps — stays consistently oriented.
    for r in 0..segments - 1 {
        let top = ring_indices(r);
        let bot = ring_indices(r + 1);
        for i in 0..na {
            let (t0, t1) = (top[i], top[(i + 1) % na]);
            let (b0, b1) = (bot[i], bot[(i + 1) % na]);
            faces.push([t0, b0, t1]);
            faces.push([t1, b0, b1]);
        }
    }

    // Last interior ring → bottom rim: `stitch_rims` handles unequal counts.
    let last = ring_indices(segments - 1);
    let wall = stitch_rims(vertices, &last, rim_b);
    faces.extend(wall);
}

/// Taubin-smooth **only** the free wall vertices of a `bushing`, leaving the
/// contact caps exact (B3b).
///
/// The first `bushing.contact_count` vertices — the two endplate caps — are
/// pinned: they still anchor their free neighbours (so the wall stays attached
/// to the exact contact rims) but never move, keeping `rendered === contacts`
/// on the surfaces that matter. Every free wall-interior vertex is relaxed by a
/// Taubin low-pass filter (alternating shrink `λ` and expand `μ` passes) so the
/// free surface loses faceting/creases without the volume shrinkage plain
/// Laplacian smoothing would cause.
///
/// Returns the number of iterations applied (`0` if `iterations == 0`, the mesh
/// has no faces, or there are no free vertices to move).
pub fn smooth_wall(bushing: &mut Bushing, iterations: usize, lambda: f64, mu: f64) -> usize {
    let first_free = bushing.contact_count as usize;
    let mesh = &mut bushing.mesh;
    if iterations == 0 || mesh.faces.is_empty() || first_free >= mesh.vertices.len() {
        return 0;
    }

    // 1-ring adjacency: `neighbors[i]` = vertices sharing an edge with `i`.
    let n = mesh.vertices.len();
    let mut neighbors: Vec<Vec<u32>> = vec![Vec::new(); n];
    for &[a, b, c] in &mesh.faces {
        push_neighbor(&mut neighbors, a, b);
        push_neighbor(&mut neighbors, b, c);
        push_neighbor(&mut neighbors, c, a);
    }

    let mut buffer = mesh.vertices.clone();
    for _ in 0..iterations {
        taubin_pass(&mesh.vertices, &neighbors, first_free, lambda, &mut buffer);
        std::mem::swap(&mut mesh.vertices, &mut buffer);
        taubin_pass(&mesh.vertices, &neighbors, first_free, mu, &mut buffer);
        std::mem::swap(&mut mesh.vertices, &mut buffer);
    }
    iterations
}

/// Add the undirected edge `(a, b)` to the 1-ring adjacency, deduping linearly
/// (triangulated-surface valence is small, so the `contains` scan is bounded).
fn push_neighbor(neighbors: &mut [Vec<u32>], a: u32, b: u32) {
    if a == b {
        return;
    }
    if !neighbors[a as usize].contains(&b) {
        neighbors[a as usize].push(b);
    }
    if !neighbors[b as usize].contains(&a) {
        neighbors[b as usize].push(a);
    }
}

/// One Laplacian umbrella pass with weight `weight`: each free vertex
/// (index `>= first_free`) moves `weight × (neighbour_centroid - vertex)`;
/// pinned vertices and vertices with no neighbours pass through unchanged.
// Neighbour counts are tiny and lossless in `f64`.
#[allow(clippy::cast_precision_loss)]
fn taubin_pass(
    src: &[Point3<f64>],
    neighbors: &[Vec<u32>],
    first_free: usize,
    weight: f64,
    dst: &mut [Point3<f64>],
) {
    for i in 0..src.len() {
        if i < first_free || neighbors[i].is_empty() {
            dst[i] = src[i];
            continue;
        }
        let mut sum = Vector3::zeros();
        for &nb in &neighbors[i] {
            sum += src[nb as usize].coords;
        }
        let centroid = sum / neighbors[i].len() as f64;
        dst[i] = src[i] + (centroid - src[i].coords) * weight;
    }
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
    use mesh_repair::{TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU};
    use nalgebra::{Point3, Vector3};

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

    /// A unit-square patch at height `z`, wound CCW from +z (normal +z) — the
    /// same convention for top and bottom, as [`assemble_bushing`] expects.
    fn square_patch(z: f64) -> Patch {
        let mesh = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, z),
                Point3::new(1.0, 0.0, z),
                Point3::new(1.0, 1.0, z),
                Point3::new(0.0, 1.0, z),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        );
        extract_patch(&mesh, &[0, 1])
    }

    /// Assert the mesh is a consistently-oriented closed 2-manifold: every
    /// directed half-edge is unique (no two faces traverse `a→b` the same way)
    /// and every edge is traversed in both directions (its reverse is present).
    fn assert_consistently_oriented_closed(mesh: &IndexedMesh) {
        let mut directed: HashSet<(u32, u32)> = HashSet::new();
        for &[a, b, c] in &mesh.faces {
            for edge in [(a, b), (b, c), (c, a)] {
                assert!(directed.insert(edge), "duplicate half-edge {edge:?}");
            }
        }
        for &(a, b) in &directed {
            assert!(directed.contains(&(b, a)), "edge {a}->{b} has no reverse");
        }
    }

    #[test]
    fn assemble_two_squares_is_closed_puck() {
        let top = square_patch(1.0);
        let bottom = square_patch(0.0);
        let puck = assemble_bushing(&top, &bottom, 1).mesh;

        // 2 top caps + 2 bottom caps + 8 wall = 12 tris over 8 verts (a box).
        assert_eq!(puck.vertex_count(), 8);
        assert_eq!(puck.face_count(), 12);

        // Watertight: no boundary edges, no non-manifold edges.
        let adjacency = MeshAdjacency::build(&puck.faces);
        assert_eq!(adjacency.boundary_edge_count(), 0, "not watertight");
        assert_eq!(adjacency.non_manifold_edge_count(), 0, "non-manifold");

        // Consistently oriented, outward (positive), unit-box volume.
        assert_consistently_oriented_closed(&puck);
        let vol = puck.signed_volume();
        assert!(vol > 0.0, "inside-out: volume {vol}");
        assert!((vol - 1.0).abs() < 1e-9, "volume {vol} != unit box");
    }

    #[test]
    fn assemble_recovers_when_caps_assemble_inward() {
        // Pass the caps z-swapped (lower square as "top", upper as "bottom") —
        // a valid same-convention pair whose natural assembly is consistently
        // oriented but inside-out. The orientation pass must recover a positive,
        // still-consistent puck, so the result is robust to which cap is which.
        let puck = assemble_bushing(&square_patch(0.0), &square_patch(1.0), 1).mesh;
        let vol = puck.signed_volume();
        assert!(vol > 0.0, "orientation pass did not recover: volume {vol}");
        assert!((vol - 1.0).abs() < 1e-9, "volume {vol} != unit box");
        assert_consistently_oriented_closed(&puck);
    }

    #[test]
    fn subdivided_wall_adds_free_rings_and_stays_closed() {
        let top = square_patch(1.0);
        let bottom = square_patch(0.0);
        let bushing = assemble_bushing(&top, &bottom, 3);
        let mesh = &bushing.mesh;

        // 8 cap verts + (segments-1)=2 interior rings × 4 = 8 free verts.
        assert_eq!(mesh.vertex_count(), 16);
        assert_eq!(bushing.contact_count, 8);
        // 4 caps + wall: two 4-quad grids (16 tris) + final 4×4 band (8) = 24.
        assert_eq!(mesh.face_count(), 28);

        // Still a valid closed solid.
        let adjacency = MeshAdjacency::build(&mesh.faces);
        assert_eq!(adjacency.boundary_edge_count(), 0, "not watertight");
        assert_eq!(adjacency.non_manifold_edge_count(), 0, "non-manifold");
        assert_consistently_oriented_closed(mesh);
        let vol = mesh.signed_volume();
        assert!((vol - 1.0).abs() < 1e-9, "volume {vol} != unit box");

        // The contact caps are byte-identical to the input patches.
        for i in 0..4 {
            assert_eq!(mesh.vertices[i], top.mesh.vertices[i]);
            assert_eq!(mesh.vertices[i + 4], bottom.mesh.vertices[i]);
        }
        // Interior ring verts sit strictly between the caps in z (free wall).
        for v in &mesh.vertices[8..16] {
            assert!(
                v.z > 0.0 && v.z < 1.0,
                "interior ring z {} not between caps",
                v.z
            );
        }
    }

    /// Sum over free vertices of the distance to their 1-ring neighbour
    /// centroid — a discrete-Laplacian roughness that Taubin smoothing lowers.
    fn wall_roughness(mesh: &IndexedMesh, first_free: usize) -> f64 {
        let n = mesh.vertices.len();
        let mut neighbors: Vec<HashSet<u32>> = vec![HashSet::new(); n];
        for &[a, b, c] in &mesh.faces {
            for (x, y) in [(a, b), (b, c), (c, a)] {
                neighbors[x as usize].insert(y);
                neighbors[y as usize].insert(x);
            }
        }
        let mut roughness = 0.0;
        for (i, ring) in neighbors.iter().enumerate().skip(first_free) {
            if ring.is_empty() {
                continue;
            }
            let mut sum = Vector3::zeros();
            for &j in ring {
                sum += mesh.vertices[j as usize].coords;
            }
            let centroid = sum / ring.len() as f64;
            roughness += (mesh.vertices[i].coords - centroid).norm();
        }
        roughness
    }

    #[test]
    fn smooth_wall_relaxes_free_verts_and_pins_caps() {
        let top = square_patch(1.0);
        let bottom = square_patch(0.0);
        let mut bushing = assemble_bushing(&top, &bottom, 4);
        let first_free = bushing.contact_count as usize; // 8

        // Corrugate the free wall so it is far from a Taubin fixpoint.
        for (offset, v) in bushing.mesh.vertices[first_free..].iter_mut().enumerate() {
            v.x += if offset % 2 == 0 { 0.3 } else { -0.3 };
        }
        let caps_before: Vec<Point3<f64>> = bushing.mesh.vertices[..first_free].to_vec();
        let free_before: Vec<Point3<f64>> = bushing.mesh.vertices[first_free..].to_vec();
        let rough_before = wall_roughness(&bushing.mesh, first_free);

        let iters = smooth_wall(&mut bushing, 10, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU);
        assert_eq!(iters, 10);

        // Caps are pinned: byte-identical after smoothing.
        assert_eq!(&bushing.mesh.vertices[..first_free], caps_before.as_slice());

        // Free verts actually moved, and the wall got smoother.
        assert_ne!(&bushing.mesh.vertices[first_free..], free_before.as_slice());
        let rough_after = wall_roughness(&bushing.mesh, first_free);
        assert!(
            rough_after < rough_before * 0.5,
            "roughness {rough_after} not < half of {rough_before}"
        );

        // Still a valid closed solid.
        let adjacency = MeshAdjacency::build(&bushing.mesh.faces);
        assert_eq!(adjacency.boundary_edge_count(), 0);
        assert!(bushing.mesh.signed_volume() > 0.0);
    }

    #[test]
    fn smooth_wall_is_noop_without_free_verts() {
        let mut bushing = assemble_bushing(&square_patch(1.0), &square_patch(0.0), 1);
        let before = bushing.mesh.vertices.clone();
        let iters = smooth_wall(&mut bushing, 10, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU);
        assert_eq!(iters, 0);
        assert_eq!(bushing.mesh.vertices, before);
    }
}
