//! Analysis-mesh primitives (F1): display geometry comes from the analysis
//! tet mesh's own connectivity (boundary surface + marching-tetrahedra slab cuts).

use std::collections::{BTreeMap, HashMap};

use mesh_types::{AttributedMesh, IndexedMesh, Point3};

use crate::Vec3;
use crate::material::Material;
use crate::mesh::{Mesh, VertexId};

use super::error::VizError;
use super::plane::Plane;
use super::scalar_transfer::{
    is_categorical_scalar_name, validate_per_tet_scalars, volume_weighted_per_vertex_avg,
};

/// Extract the closed boundary surface of a tet mesh as a triangulated
/// [`AttributedMesh`] with per-vertex scalars.
///
/// The boundary faces come from the precomputed
/// [`Mesh::boundary_faces`] cache (right-handed outward winding, populated
/// at mesh construction). Per-tet scalars in `per_tet_scalars` are
/// projected to per-vertex and surfaced as entries in the output mesh's
/// `extras` map under the same names. Continuous scalars use volume-
/// weighted averaging over incident tets; categorical scalars (suffix
/// `_id`) use the largest-volume incident tet (C2.1 nearest-tet
/// semantics) so display-vertex values stay integer for the cf-view
/// colormap detector's Categorical pick.
///
/// All vertices of `mesh` are emitted, including interior vertices
/// (which appear as unreferenced vertices in the PLY — most readers
/// tolerate this). Compaction to the boundary-vertex subset is a future
/// optimization.
///
/// # Errors
///
/// Returns [`VizError::PerTetScalarLengthMismatch`] if any value vec in
/// `per_tet_scalars` has length other than `mesh.n_tets()`.
///
/// # Panics
///
/// Does not panic on caller input (an invalid `per_tet_scalars` is returned
/// as [`VizError`]); the internal `.expect()` is unreachable by construction.
//
// `cast_possible_truncation` allowed: per-vertex scalar values are
// projected to f32 for PLY emit (the format's per-vertex extra slot
// is f32). The truncation IS the documented behavior. `expect_used`
// allowed on `insert_extra`: the validate-up-front + by-construction
// length contract makes the call infallible (failure would mean the
// per_vertex Vec we just built is a different length than
// mesh.n_vertices() — internally inconsistent and unreachable).
// `too_many_lines` allowed: the C2 categorical pre-pass + per-scalar
// dispatch loop pushes the body past clippy's 100-line default; the
// alternative — extracting `winner_tet_per_vertex` to a helper —
// inflates the API surface for a single internal use.
#[allow(
    clippy::cast_possible_truncation,
    clippy::expect_used,
    clippy::too_many_lines
)]
pub fn boundary_surface<M: Material>(
    mesh: &dyn Mesh<M>,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    validate_per_tet_scalars(mesh, per_tet_scalars)?;

    let mut geometry = IndexedMesh::new();
    for &p in mesh.positions() {
        geometry.vertices.push(Point3::new(p.x, p.y, p.z));
    }
    for &face in mesh.boundary_faces() {
        geometry.faces.push(face);
    }

    // C2 categorical path: pick a single "winner" tet per analysis-mesh
    // vertex (the largest-volume incident tet) so categorical scalars
    // emit the exact integer of that representative tet rather than the
    // volume-weighted-average-over-incident-tets fractional value that
    // continuous scalars use. Only pay the pre-pass when at least one
    // categorical scalar is in flight; pure-continuous calls (the
    // pre-C2.1 default) skip it entirely.
    let any_categorical = per_tet_scalars
        .keys()
        .any(|&n| is_categorical_scalar_name(n));
    let winner_tet_per_vertex: Option<Vec<Option<u32>>> = if any_categorical {
        let n_vertices = mesh.n_vertices();
        let signed_volumes = &mesh.quality().signed_volume;
        let mut best_vol = vec![0.0_f64; n_vertices];
        let mut best_tet: Vec<Option<u32>> = vec![None; n_vertices];
        for (t, &sv) in signed_volumes.iter().enumerate() {
            let vol = sv.abs();
            let tet_id = t as u32;
            let [v0, v1, v2, v3] = mesh.tet_vertices(tet_id);
            for v in [v0, v1, v2, v3] {
                if vol > best_vol[v as usize] {
                    best_vol[v as usize] = vol;
                    best_tet[v as usize] = Some(tet_id);
                }
            }
        }
        Some(best_tet)
    } else {
        None
    };

    let mut attr = AttributedMesh::new(geometry);
    for (&name, &per_tet) in per_tet_scalars {
        let per_vertex: Vec<f64> = if is_categorical_scalar_name(name) {
            // Categorical: emit the per-tet value of the largest-volume
            // incident tet at each vertex (C2 nearest-tet, k=1).
            // Orphan vertices (no incident tets) fall back to 0.0 to
            // match the continuous-path orphan convention.
            let lookup = winner_tet_per_vertex
                .as_ref()
                .expect("populated above when any categorical scalar is present");
            lookup
                .iter()
                .map(|&maybe_t| maybe_t.map_or(0.0, |t| per_tet[t as usize]))
                .collect()
        } else {
            volume_weighted_per_vertex_avg(mesh, per_tet)
        };
        let f32_values: Vec<f32> = per_vertex.iter().map(|&v| v as f32).collect();
        attr.insert_extra(name.to_string(), f32_values)
            .expect("per-vertex length matches mesh.n_vertices() by construction");
    }
    // C1: area-weighted vertex normals from the tet-mesh boundary
    // triangulation. No SDF on this code path (the boundary surface is
    // extracted from the analysis tet mesh, not a scalar field), so the
    // analytical-gradient variant used in `design_surface` isn't an
    // option — the area-weighted alternative is still smooth-shaded
    // and replaces the flat-per-triangle default cf-view would
    // otherwise compute.
    attr.compute_normals();
    Ok(attr)
}

/// Intersect a tet mesh with an axis-aligned plane via marching-tetrahedra.
///
/// Returns the cross-section as an [`AttributedMesh`] of triangles
/// with per-vertex scalars sampled from the analysis tet mesh.
/// Continuous scalars linearly interpolate along each crossed edge
/// from the volume-weighted-per-vertex field; categorical scalars
/// (suffix `_id`) take the value of the tet whose centroid is closest
/// in 3D space to the cut vertex among the tets that touch the cut
/// edge (C2.1 position-based nearest-tet semantics) — preserves
/// integer labels for the cf-view colormap detector's Categorical pick.
///
/// Each tet contributes 0, 1, or 2 cross-section triangles depending
/// on its sign pattern with respect to the plane (5 cases by
/// symmetry; see implementation comments). Cross-points along each
/// edge are deduplicated by sorted-vertex-pair edge key, so a shared
/// edge between two tets produces one cross-vertex.
///
/// All emitted triangles are wound so the right-hand-rule normal
/// points in the +`plane.axis` direction (cf-view's default lighting
/// is single-sided; this matches the [`Mesh::boundary_faces`]
/// outward-winding convention).
///
/// # Errors
///
/// - [`VizError::InvalidPlaneAxis`] if `plane.axis >= 3`.
/// - [`VizError::PerTetScalarLengthMismatch`] if any scalar's length
///   != `mesh.n_tets()`.
pub fn slab_cut<M: Material>(
    mesh: &dyn Mesh<M>,
    plane: Plane,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    slab_cut_at_positions(mesh, mesh.positions(), plane, per_tet_scalars)
}

/// Deformed-mesh sibling of [`slab_cut`].
///
/// Same marching-tetrahedra cut, but each rest position is offset by
/// `displacement_per_vertex[v] * amplify` before classifying tets and
/// interpolating cross-edges. Renders the cross-section of the
/// **deformed** tet mesh — sliced cavity walls, contact-zone bulges,
/// and any other deformation that's invisible from a closed outer
/// boundary surface.
///
/// Per-tet scalar projection still uses the rest-position scalar
/// pipeline (volume-weighted-per-vertex for continuous; closest-
/// centroid-to-cut-vertex for categorical `_id` scalars; see
/// [`slab_cut`]) — only the geometric cut uses deformed positions.
/// Pass `amplify > 1` to make sub-mm deformations visually obvious
/// on cm-scale bodies, matching the [`design_surface_deformed`](super::design_surface_deformed)
/// convention.
///
/// # Errors
///
/// All [`slab_cut`] errors plus [`VizError::PerVertexLengthMismatch`]
/// if `displacement_per_vertex`'s length differs from
/// `mesh.n_vertices()`.
pub fn slab_cut_deformed<M: Material>(
    mesh: &dyn Mesh<M>,
    plane: Plane,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
    displacement_per_vertex: &[Vec3],
    amplify: f64,
) -> Result<AttributedMesh, VizError> {
    let n_vertices = mesh.n_vertices();
    if displacement_per_vertex.len() != n_vertices {
        return Err(VizError::PerVertexLengthMismatch {
            expected: n_vertices,
            actual: displacement_per_vertex.len(),
        });
    }

    let rest_positions = mesh.positions();
    let deformed_positions: Vec<Vec3> = rest_positions
        .iter()
        .zip(displacement_per_vertex.iter())
        .map(|(p, d)| p + d * amplify)
        .collect();

    slab_cut_at_positions(mesh, &deformed_positions, plane, per_tet_scalars)
}

/// Marching-tet core shared by [`slab_cut`] (rest) and
/// [`slab_cut_deformed`] (displaced). `positions` is the position
/// slice the SD computation and cross-edge interpolation read; `mesh`
/// supplies validation, tet connectivity, and the per-tet scalar
/// projection (rest-volume-weighted per-vertex for continuous;
/// closest-centroid-to-cut-vertex for categorical `_id` scalars).
//
// `cast_possible_truncation` allowed for the f64 → f32 PLY emit path
// and the usize → u32 vertex index path (VertexId is u32 by crate
// convention; `mesh.n_tets() < u32::MAX` is a γ-locked mesh invariant).
// `expect_used` allowed on `insert_extra` — the cross-vertex count we
// count up matches the cut_scalars[i] vec length by construction.
// `similar_names` allowed because `i_ac`/`i_bc`/`i_ad`/`i_bd` are the
// algorithm's natural names for the four cross-edges of the 2-2 tet
// sign-pattern case (above-{a,b}, below-{c,d}); renaming would obscure
// the geometry. `unreachable` allowed on the `_ => unreachable!()`
// match arm in the (na, nb) dispatch — invariant-pinned: a tet has
// exactly 4 vertices, so na + nb == 4 and the 5 explicit arms
// (4,0)/(0,4)/(1,3)/(3,1)/(2,2) exhaust the space.
#[allow(
    clippy::cast_possible_truncation,
    clippy::expect_used,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::unreachable
)]
fn slab_cut_at_positions<M: Material>(
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    plane: Plane,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    if plane.axis >= 3 {
        return Err(VizError::InvalidPlaneAxis { axis: plane.axis });
    }
    validate_per_tet_scalars(mesh, per_tet_scalars)?;

    // C2 categorical/continuous split. Continuous scalars pre-project
    // to a volume-weighted per-vertex average so the marching-tet
    // inner loop can linearly interpolate along each crossed edge.
    // Categorical scalars (suffix `_id`) hand a `Some(per_tet)` slice
    // to `intersect_edge`, which samples the cut vertex from the tet
    // whose centroid is closest in 3D space to the cut vertex among
    // the tets that have touched it so far — preserves the integer-
    // label invariant the cf-view colormap detector reads as
    // Categorical (`tab10`) AND keeps the categorical boundary aligned
    // with the 3D centroid-Voronoi midline (smooth) rather than the
    // analysis-mesh largest-volume topology (jagged stair-step at
    // every shell-boundary face). The empty Vec slots in
    // `per_vertex_by_scalar` at categorical indices are intentionally
    // never read (gated by `per_tet_by_scalar[i].is_some()` in
    // `intersect_edge`).
    let scalar_names: Vec<&str> = per_tet_scalars.keys().copied().collect();
    let per_vertex_by_scalar: Vec<Vec<f64>> = scalar_names
        .iter()
        .map(|&name| {
            if is_categorical_scalar_name(name) {
                Vec::new()
            } else {
                volume_weighted_per_vertex_avg(mesh, per_tet_scalars[&name])
            }
        })
        .collect();
    let per_tet_by_scalar: Vec<Option<&[f64]>> = scalar_names
        .iter()
        .map(|&name| {
            if is_categorical_scalar_name(name) {
                Some(per_tet_scalars[&name])
            } else {
                None
            }
        })
        .collect();

    // Per-vertex signed distance: positive = above (axis side),
    // non-positive = below. Vertices exactly on the plane (sd == 0)
    // classify as below, which makes the cross-edge interpolation at
    // t = 0 collapse onto the mesh vertex (harmless degeneracy).
    let sd: Vec<f64> = positions
        .iter()
        .map(|p| p[plane.axis] - plane.value)
        .collect();

    let mut state = SlabCutState::new(scalar_names.len());

    let n_tets = mesh.n_tets();
    for t in 0..n_tets {
        let tet_id = t as u32;
        let [v0, v1, v2, v3] = mesh.tet_vertices(tet_id);
        let tet_centroid = (positions[v0 as usize]
            + positions[v1 as usize]
            + positions[v2 as usize]
            + positions[v3 as usize])
            * 0.25;
        let verts = [v0, v1, v2, v3];
        let mut above = [0u32; 4];
        let mut na = 0usize;
        let mut below = [0u32; 4];
        let mut nb = 0usize;
        for &v in &verts {
            if sd[v as usize] > 0.0 {
                above[na] = v;
                na += 1;
            } else {
                below[nb] = v;
                nb += 1;
            }
        }
        match (na, nb) {
            (4, 0) | (0, 4) => {}
            (1, 3) => {
                let s = above[0];
                let i0 = state.intersect_edge(
                    s,
                    below[0],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                let i1 = state.intersect_edge(
                    s,
                    below[1],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                let i2 = state.intersect_edge(
                    s,
                    below[2],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                state.push_face([i0, i1, i2], plane.axis);
            }
            (3, 1) => {
                let s = below[0];
                let i0 = state.intersect_edge(
                    s,
                    above[0],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                let i1 = state.intersect_edge(
                    s,
                    above[1],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                let i2 = state.intersect_edge(
                    s,
                    above[2],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                state.push_face([i0, i1, i2], plane.axis);
            }
            (2, 2) => {
                // Quad cross-section. Above = {a, b}, below = {c, d}.
                // Cross-edges: a-c, b-c, b-d, a-d. Going around the
                // quad's perimeter: i_ac → i_bc → i_bd → i_ad → i_ac.
                // Triangulate via the i_ac → i_bd diagonal.
                let i_ac = state.intersect_edge(
                    above[0],
                    below[0],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                let i_bc = state.intersect_edge(
                    above[1],
                    below[0],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                let i_bd = state.intersect_edge(
                    above[1],
                    below[1],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                let i_ad = state.intersect_edge(
                    above[0],
                    below[1],
                    tet_id,
                    tet_centroid,
                    positions,
                    &per_vertex_by_scalar,
                    &per_tet_by_scalar,
                    &sd,
                );
                state.push_face([i_ac, i_bc, i_bd], plane.axis);
                state.push_face([i_ac, i_bd, i_ad], plane.axis);
            }
            _ => unreachable!("tet has 4 vertices; (na, nb) sums to 4"),
        }
    }

    let mut geometry = IndexedMesh::new();
    geometry.vertices = state.cut_positions;
    geometry.faces = state.cut_faces;

    let mut attr = AttributedMesh::new(geometry);
    for (i, &name) in scalar_names.iter().enumerate() {
        let f32_values: Vec<f32> = state.cut_scalars[i].iter().map(|&v| v as f32).collect();
        attr.insert_extra(name.to_string(), f32_values)
            .expect("per-cross-vertex length matches cross-vertex count by construction");
    }
    Ok(attr)
}

/// Internal scratch state for [`slab_cut`]: cross-vertex positions,
/// per-scalar value vecs, dedup map, and emitted faces.
///
/// C2 categorical scalars sample from a single "owner" tet per cut
/// vertex. The owner is the tet whose centroid is closest in 3D space
/// to the cut vertex; later tet-iteration encounters of the same
/// shared edge replace the owner only if their centroid is closer.
/// Position-based winner (rather than topology-based — e.g. largest
/// volume) keeps the categorical boundary aligned with the 3D
/// centroid-Voronoi midline between adjacent tets of differing labels,
/// avoiding the analysis-mesh stair-step and speckle artifacts a
/// volume-only rule produces at layer interfaces on a regular
/// BCC + IS mesh. Matches the position-based pick `design_surface`
/// uses via [`nearest_tet_centroid_idx`].
///
/// Continuous scalars are determined by the edge endpoints alone
/// (linear interp of the volume-weighted-per-vertex average) and are
/// invariant under owner promotion.
struct SlabCutState {
    edge_to_idx: HashMap<(VertexId, VertexId), u32>,
    cut_positions: Vec<Point3<f64>>,
    /// Outer Vec indexed by scalar; inner Vec indexed by cross-vertex.
    cut_scalars: Vec<Vec<f64>>,
    cut_faces: Vec<[u32; 3]>,
    /// Tet ID of the categorical "owner" for each cut vertex (parallel
    /// to `cut_positions`). The owner is the tet whose centroid is
    /// closest in 3D space to the cut vertex among the tets that have
    /// touched the cut vertex so far.
    cut_owner_tet: Vec<u32>,
    /// Squared distance from the cut vertex to the current owner tet's
    /// centroid (parallel to `cut_positions`). Tracked so dedup-hit
    /// promotion checks don't have to recompute the centroid distance
    /// for the stored owner.
    cut_owner_dist2: Vec<f64>,
}

impl SlabCutState {
    fn new(n_scalars: usize) -> Self {
        Self {
            edge_to_idx: HashMap::new(),
            cut_positions: Vec::new(),
            cut_scalars: (0..n_scalars).map(|_| Vec::new()).collect(),
            cut_faces: Vec::new(),
            cut_owner_tet: Vec::new(),
            cut_owner_dist2: Vec::new(),
        }
    }

    // `cast_possible_truncation` allowed: cross_vertex count ≤
    // 4×n_tets which fits in u32 by the same γ-locked invariant
    // mesh.n_tets() < u32::MAX assumes (VertexId is u32 crate-wide).
    // `too_many_arguments` allowed: the tet-id / tet-centroid /
    // per_tet_by_scalar trio is the C2 categorical-owner channel and
    // is essential context for both new-vertex emission and dedup-hit
    // owner promotion; folding it into a struct would inflate the
    // call-site noise without clarifying the contract.
    #[allow(clippy::cast_possible_truncation, clippy::too_many_arguments)]
    fn intersect_edge(
        &mut self,
        va: VertexId,
        vb: VertexId,
        tet_id: u32,
        tet_centroid: crate::Vec3,
        positions: &[crate::Vec3],
        per_vertex_by_scalar: &[Vec<f64>],
        per_tet_by_scalar: &[Option<&[f64]>],
        sd: &[f64],
    ) -> u32 {
        let (lo, hi) = if va < vb { (va, vb) } else { (vb, va) };
        if let Some(&idx) = self.edge_to_idx.get(&(lo, hi)) {
            // Dedup hit: promote the categorical owner if this tet's
            // centroid is closer to the cut vertex in 3D space than
            // the current owner's. Continuous scalars depend only on
            // the edge endpoints, so they need no update.
            let cross_idx = idx as usize;
            let cross = self.cut_positions[cross_idx];
            let cross_v = crate::Vec3::new(cross.x, cross.y, cross.z);
            let dist2 = (cross_v - tet_centroid).norm_squared();
            if dist2 < self.cut_owner_dist2[cross_idx] {
                self.cut_owner_tet[cross_idx] = tet_id;
                self.cut_owner_dist2[cross_idx] = dist2;
                for (i, &maybe) in per_tet_by_scalar.iter().enumerate() {
                    if let Some(per_tet) = maybe {
                        self.cut_scalars[i][cross_idx] = per_tet[tet_id as usize];
                    }
                }
            }
            return idx;
        }
        let sd_a = sd[va as usize];
        let sd_b = sd[vb as usize];
        // Caller guarantees `va` and `vb` are on opposite sides (one
        // sd > 0, the other sd <= 0), so the denominator is non-zero.
        let t = sd_a / (sd_a - sd_b);
        let p_a = positions[va as usize];
        let p_b = positions[vb as usize];
        let cross = p_a + (p_b - p_a) * t;
        let new_idx = self.cut_positions.len() as u32;
        self.cut_positions
            .push(Point3::new(cross.x, cross.y, cross.z));
        self.cut_owner_tet.push(tet_id);
        self.cut_owner_dist2
            .push((cross - tet_centroid).norm_squared());
        for (i, &maybe) in per_tet_by_scalar.iter().enumerate() {
            if let Some(per_tet) = maybe {
                // Categorical: take the owner tet's per-tet value
                // (k=1 nearest-tet semantics — ties between tets of
                // differing categorical labels at a shared cut
                // vertex resolve to the tet whose centroid is closer
                // in 3D space to the cut vertex).
                self.cut_scalars[i].push(per_tet[tet_id as usize]);
            } else {
                // Continuous: linear interp from vol-weighted
                // per-vertex avg across the edge endpoints (matches
                // the pre-C2.1 path bit-exact).
                let per_vertex = &per_vertex_by_scalar[i];
                let psi_a = per_vertex[va as usize];
                let psi_b = per_vertex[vb as usize];
                self.cut_scalars[i].push((1.0 - t).mul_add(psi_a, t * psi_b));
            }
        }
        self.edge_to_idx.insert((lo, hi), new_idx);
        new_idx
    }

    fn push_face(&mut self, tri: [u32; 3], plane_axis: usize) {
        self.cut_faces
            .push(align_winding_to_axis(tri, &self.cut_positions, plane_axis));
    }
}

/// Reorder a triangle's vertices so its right-hand-rule normal has
/// non-negative projection onto `plane_axis`. Mirrors the
/// `Mesh::boundary_faces` outward-winding contract for cut polygons.
fn align_winding_to_axis(
    tri: [u32; 3],
    cut_positions: &[Point3<f64>],
    plane_axis: usize,
) -> [u32; 3] {
    let p0 = cut_positions[tri[0] as usize];
    let p1 = cut_positions[tri[1] as usize];
    let p2 = cut_positions[tri[2] as usize];
    let a = p1 - p0;
    let b = p2 - p0;
    let cross_on_axis = match plane_axis {
        0 => a.y.mul_add(b.z, -(a.z * b.y)),
        1 => a.z.mul_add(b.x, -(a.x * b.z)),
        _ => a.x.mul_add(b.y, -(a.y * b.x)),
    };
    if cross_on_axis >= 0.0 {
        tri
    } else {
        [tri[0], tri[2], tri[1]]
    }
}
