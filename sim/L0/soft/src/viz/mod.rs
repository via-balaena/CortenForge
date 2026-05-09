//! FEM-grade visualization helpers for soft-body meshes.
//!
//! Two tet-mesh-native primitives, both geometry-agnostic:
//!
//! - [`boundary_surface`] extracts the precomputed boundary surface
//!   from [`Mesh::boundary_faces`] and emits it as a triangulated 3D
//!   surface coloured by per-vertex scalars (volume-weighted averaged
//!   from per-tet scalars).
//! - [`slab_cut`] intersects the tet mesh with an axis-aligned plane
//!   via marching-tetrahedra and emits the cross-section polygon as
//!   a triangle mesh, with per-vertex scalars linearly interpolated
//!   from the same volume-weighted per-vertex field.
//!
//! Both helpers consume any [`Mesh<M>`] sim-soft can produce and emit
//! [`AttributedMesh`] with multiple per-vertex scalars (each input
//! per-tet scalar surfaces as one entry in the output `extras`
//! `BTreeMap`). PLY emit is the caller's responsibility — pair with
//! `mesh_io::save_ply_attributed` downstream (mesh-io is not a
//! sim-soft dep, so the link is plain text rather than intra-doc).
//!
//! # Architecture
//!
//! Sim-soft's BCC + Isosurface Stuffing pipeline produces a tet mesh
//! that already encodes the body's geometry. The viz primitives here
//! USE that connectivity directly rather than re-meshing from a
//! lower-information representation. A pre-pivot Delaunay-of-centroids
//! architecture (banked at `examples/sim-soft/spade-delaunay-spike/`)
//! was falsified across 8 fix attempts because it threw away the tet
//! connectivity and tried to re-derive geometry from centroid clouds
//! plus filters; each fix patched one geometry-specific symptom and
//! exposed another. The lesson: when a structured mesh exists, its
//! connectivity is the right primitive.
//!
//! See `project_sim_soft_viz_arc.md` (memory) for the full
//! architecture story including the spike findings.
//!
//! # Per-vertex scalar averaging
//!
//! Per-tet scalars (e.g. strain-energy density `psi_j_per_m3` from
//! the Newton solver) project to per-vertex scalars via volume-
//! weighted averaging: each tet contributes its scalar to each of its
//! 4 vertices weighted by the tet's volume, then the per-vertex value
//! is the volume-weighted mean over all incident tets. Volume
//! weighting (over uniform averaging) avoids small boundary-fitting
//! tets dominating the average on Isosurface-Stuffed organic meshes
//! — uniform looks identical on near-uniform BCC meshes but degrades
//! on the cavity-fitting layered-silicone production target.

use std::collections::{BTreeMap, HashMap};

use mesh_types::{AttributedMesh, IndexedMesh, Point3};

use crate::material::Material;
use crate::mesh::{Mesh, VertexId};

/// Error surface for viz primitives.
#[derive(Clone, Debug)]
pub enum VizError {
    /// A per-tet scalar's length does not equal `mesh.n_tets()`.
    PerTetScalarLengthMismatch {
        /// Name of the offending scalar (`BTreeMap` key).
        name: String,
        /// Required length (= `mesh.n_tets()`).
        expected: usize,
        /// Length actually supplied.
        actual: usize,
    },
    /// Cutting plane axis index out of range (must be 0/1/2 = x/y/z).
    InvalidPlaneAxis {
        /// Bad axis value supplied.
        axis: usize,
    },
}

impl std::fmt::Display for VizError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PerTetScalarLengthMismatch {
                name,
                expected,
                actual,
            } => write!(
                f,
                "per-tet scalar `{name}` has length {actual}, expected {expected} (= mesh.n_tets())"
            ),
            Self::InvalidPlaneAxis { axis } => {
                write!(f, "plane axis must be 0/1/2 (x/y/z), got {axis}")
            }
        }
    }
}

impl std::error::Error for VizError {}

/// Axis-aligned cutting plane for [`slab_cut`].
///
/// `axis` is 0 (x), 1 (y), or 2 (z); `value` is the plane's
/// position along that axis. The plane normal points in the +`axis`
/// direction; "above" the plane means `position[axis] > value`.
///
/// Generalizing to arbitrary planes (point + normal) is a future
/// enhancement; F1.1 ships the axis-aligned form because every
/// existing call site cuts along an axis and the simpler API
/// dodges premature plane-frame plumbing.
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    /// Cutting-plane axis (0 = x, 1 = y, 2 = z).
    pub axis: usize,
    /// Plane position along `axis`.
    pub value: f64,
}

/// Extract the closed boundary surface of a tet mesh as a triangulated
/// [`AttributedMesh`] with per-vertex scalars.
///
/// The boundary faces come from the precomputed
/// [`Mesh::boundary_faces`] cache (right-handed outward winding, populated
/// at mesh construction). Per-tet scalars in `per_tet_scalars` are
/// projected to per-vertex via volume-weighted averaging and surface as
/// entries in the output mesh's `extras` map under the same names.
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
//
// `cast_possible_truncation` allowed: per-vertex scalar values are
// projected to f32 for PLY emit (the format's per-vertex extra slot
// is f32). The truncation IS the documented behavior. `expect_used`
// allowed on `insert_extra`: the validate-up-front + by-construction
// length contract makes the call infallible (failure would mean the
// per_vertex Vec we just built is a different length than
// mesh.n_vertices() — internally inconsistent and unreachable).
#[allow(clippy::cast_possible_truncation, clippy::expect_used)]
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

    let mut attr = AttributedMesh::new(geometry);
    for (&name, &per_tet) in per_tet_scalars {
        let per_vertex = volume_weighted_per_vertex_avg(mesh, per_tet);
        let f32_values: Vec<f32> = per_vertex.iter().map(|&v| v as f32).collect();
        attr.insert_extra(name.to_string(), f32_values)
            .expect("per-vertex length matches mesh.n_vertices() by construction");
    }
    Ok(attr)
}

/// Intersect a tet mesh with an axis-aligned plane via marching-tetrahedra.
///
/// Returns the cross-section as an [`AttributedMesh`] of triangles
/// with per-vertex scalars linearly interpolated from the
/// volume-weighted per-vertex field.
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
//
// `cast_possible_truncation` allowed for the f64 → f32 PLY emit
// path and the usize → u32 vertex index path (VertexId is u32 by
// crate convention; `mesh.n_tets() < u32::MAX` is a γ-locked mesh
// invariant). `expect_used` allowed on `insert_extra` — see the
// boundary_surface justification; the cross-vertex count we count
// up matches the cut_scalars[i] vec length by construction.
// `similar_names` allowed because `i_ac`/`i_bc`/`i_ad`/`i_bd` are
// the algorithm's natural names for the four cross-edges of the
// 2-2 tet sign-pattern case (above-{a,b}, below-{c,d}); renaming
// would obscure the geometry.
#[allow(
    clippy::cast_possible_truncation,
    clippy::expect_used,
    clippy::similar_names
)]
pub fn slab_cut<M: Material>(
    mesh: &dyn Mesh<M>,
    plane: Plane,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    if plane.axis >= 3 {
        return Err(VizError::InvalidPlaneAxis { axis: plane.axis });
    }
    validate_per_tet_scalars(mesh, per_tet_scalars)?;

    let positions = mesh.positions();
    let n_tets = mesh.n_tets();

    // Project each per-tet scalar to per-vertex up-front so the
    // marching-tet interior loop can interpolate cheaply along each
    // crossed edge.
    let scalar_names: Vec<&str> = per_tet_scalars.keys().copied().collect();
    let per_vertex_by_scalar: Vec<Vec<f64>> = scalar_names
        .iter()
        .map(|&name| volume_weighted_per_vertex_avg(mesh, per_tet_scalars[&name]))
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

    for t in 0..n_tets {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t as u32);
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
                let i0 = state.intersect_edge(s, below[0], positions, &per_vertex_by_scalar, &sd);
                let i1 = state.intersect_edge(s, below[1], positions, &per_vertex_by_scalar, &sd);
                let i2 = state.intersect_edge(s, below[2], positions, &per_vertex_by_scalar, &sd);
                state.push_face([i0, i1, i2], plane.axis);
            }
            (3, 1) => {
                let s = below[0];
                let i0 = state.intersect_edge(s, above[0], positions, &per_vertex_by_scalar, &sd);
                let i1 = state.intersect_edge(s, above[1], positions, &per_vertex_by_scalar, &sd);
                let i2 = state.intersect_edge(s, above[2], positions, &per_vertex_by_scalar, &sd);
                state.push_face([i0, i1, i2], plane.axis);
            }
            (2, 2) => {
                // Quad cross-section. Above = {a, b}, below = {c, d}.
                // Cross-edges: a-c, b-c, b-d, a-d. Going around the
                // quad's perimeter: i_ac → i_bc → i_bd → i_ad → i_ac.
                // Triangulate via the i_ac → i_bd diagonal.
                let i_ac =
                    state.intersect_edge(above[0], below[0], positions, &per_vertex_by_scalar, &sd);
                let i_bc =
                    state.intersect_edge(above[1], below[0], positions, &per_vertex_by_scalar, &sd);
                let i_bd =
                    state.intersect_edge(above[1], below[1], positions, &per_vertex_by_scalar, &sd);
                let i_ad =
                    state.intersect_edge(above[0], below[1], positions, &per_vertex_by_scalar, &sd);
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

/// Volume-weighted projection of a per-tet scalar to per-vertex.
///
/// `per_tet_scalar.len()` must equal `mesh.n_tets()`; callers are
/// expected to validate this first via [`validate_per_tet_scalars`]
/// (the public viz fns do).
//
// `cast_possible_truncation` allowed: TetId is u32 by crate
// convention so `mesh.n_tets() < u32::MAX` is a γ-locked invariant.
#[allow(clippy::cast_possible_truncation)]
fn volume_weighted_per_vertex_avg<M: Material>(
    mesh: &dyn Mesh<M>,
    per_tet_scalar: &[f64],
) -> Vec<f64> {
    let n_vertices = mesh.n_vertices();
    let signed_volumes = &mesh.quality().signed_volume;
    let mut accum_val_vol = vec![0.0_f64; n_vertices];
    let mut accum_vol = vec![0.0_f64; n_vertices];
    for t in 0..mesh.n_tets() {
        // signed_volume comes out positive for right-handed tets per
        // pipeline Decision H; .abs() is defensive.
        let vol = signed_volumes[t].abs();
        let weighted = per_tet_scalar[t] * vol;
        let [v0, v1, v2, v3] = mesh.tet_vertices(t as u32);
        for v in [v0, v1, v2, v3] {
            accum_val_vol[v as usize] += weighted;
            accum_vol[v as usize] += vol;
        }
    }
    (0..n_vertices)
        .map(|v| {
            if accum_vol[v] > 0.0 {
                accum_val_vol[v] / accum_vol[v]
            } else {
                // Orphan vertex (no incident tets) — shouldn't occur
                // on connected meshes; return 0 so the scalar slot
                // stays valid.
                0.0
            }
        })
        .collect()
}

fn validate_per_tet_scalars<M: Material>(
    mesh: &dyn Mesh<M>,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<(), VizError> {
    let expected = mesh.n_tets();
    for (&name, &values) in per_tet_scalars {
        if values.len() != expected {
            return Err(VizError::PerTetScalarLengthMismatch {
                name: name.to_string(),
                expected,
                actual: values.len(),
            });
        }
    }
    Ok(())
}

/// Internal scratch state for [`slab_cut`]: cross-vertex positions,
/// per-scalar value vecs, dedup map, and emitted faces.
struct SlabCutState {
    edge_to_idx: HashMap<(VertexId, VertexId), u32>,
    cut_positions: Vec<Point3<f64>>,
    /// Outer Vec indexed by scalar; inner Vec indexed by cross-vertex.
    cut_scalars: Vec<Vec<f64>>,
    cut_faces: Vec<[u32; 3]>,
}

impl SlabCutState {
    fn new(n_scalars: usize) -> Self {
        Self {
            edge_to_idx: HashMap::new(),
            cut_positions: Vec::new(),
            cut_scalars: (0..n_scalars).map(|_| Vec::new()).collect(),
            cut_faces: Vec::new(),
        }
    }

    // `cast_possible_truncation` allowed: cross_vertex count ≤
    // 4×n_tets which fits in u32 by the same γ-locked invariant
    // mesh.n_tets() < u32::MAX assumes (VertexId is u32 crate-wide).
    #[allow(clippy::cast_possible_truncation)]
    fn intersect_edge(
        &mut self,
        va: VertexId,
        vb: VertexId,
        positions: &[crate::Vec3],
        per_vertex_by_scalar: &[Vec<f64>],
        sd: &[f64],
    ) -> u32 {
        let (lo, hi) = if va < vb { (va, vb) } else { (vb, va) };
        if let Some(&idx) = self.edge_to_idx.get(&(lo, hi)) {
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
        for (i, per_vertex) in per_vertex_by_scalar.iter().enumerate() {
            let psi_a = per_vertex[va as usize];
            let psi_b = per_vertex[vb as usize];
            self.cut_scalars[i].push((1.0 - t).mul_add(psi_a, t * psi_b));
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

#[cfg(test)]
#[allow(
    // Tests use unwrap/expect/panic freely on values whose
    // unwrap-safety is the test's invariant; they're not user code.
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::match_wildcard_for_single_variants,
    // `cast_precision_loss` on len() for averaging in the
    // winding-regression test; len is small (handful of vertices).
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use crate::material::MaterialField;
    use crate::mesh::{HandBuiltTetMesh, SingleTetMesh};

    /// Single-tet boundary surface: 4 vertices, 4 boundary faces, all
    /// per-vertex scalars equal the (single) per-tet scalar.
    #[test]
    fn boundary_surface_single_tet() {
        let field = MaterialField::skeleton_default();
        let mesh = SingleTetMesh::new(&field);
        let psi = [42.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = boundary_surface(&mesh, &scalars).unwrap();
        assert_eq!(attr.geometry.vertices.len(), 4);
        assert_eq!(attr.geometry.faces.len(), 4);
        let psi_extra = attr.extras.get("psi").expect("psi extra missing");
        assert_eq!(psi_extra.len(), 4);
        for &v in psi_extra {
            assert!((v - 42.0).abs() < 1e-9);
        }
    }

    /// Two-tet shared-face mesh: 5 vertices, 6 boundary faces (8 total
    /// face slots − 2 interior shared faces). Per-vertex psi for the
    /// 2 shared-face vertices is the volume-weighted average of both
    /// tets' values; for the 3 unique vertices it's the host tet's
    /// value.
    #[test]
    fn boundary_surface_two_tet_shared_face() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let psi: [f64; 2] = [1.0, 3.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = boundary_surface(&mesh, &scalars).unwrap();
        assert_eq!(attr.geometry.vertices.len(), 5);
        assert_eq!(attr.geometry.faces.len(), 6);
        let psi_extra = attr.extras.get("psi").expect("psi extra missing");
        assert_eq!(psi_extra.len(), 5);
    }

    /// Per-tet scalar with wrong length surfaces as the typed error.
    #[test]
    fn boundary_surface_rejects_wrong_length_scalar() {
        let field = MaterialField::skeleton_default();
        let mesh = SingleTetMesh::new(&field);
        let psi = [1.0_f64, 2.0]; // length 2, mesh has 1 tet
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let err = boundary_surface(&mesh, &scalars).unwrap_err();
        match err {
            VizError::PerTetScalarLengthMismatch {
                name,
                expected,
                actual,
            } => {
                assert_eq!(name, "psi");
                assert_eq!(expected, 1);
                assert_eq!(actual, 2);
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }

    /// `slab_cut` on `SingleTetMesh` (canonical decimeter tet at the
    /// origin with edge 0.1) cut by the x = 0.05 plane. The plane
    /// crosses through 1 vertex (`v_1` = (0.1, 0, 0)) on the +x side
    /// and 3 vertices on the -x side, producing a single triangle.
    #[test]
    fn slab_cut_single_tet_one_three_case() {
        let field = MaterialField::skeleton_default();
        let mesh = SingleTetMesh::new(&field);
        let psi = [7.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = slab_cut(
            &mesh,
            Plane {
                axis: 0,
                value: 0.05,
            },
            &scalars,
        )
        .unwrap();
        assert_eq!(attr.geometry.faces.len(), 1);
        assert_eq!(attr.geometry.vertices.len(), 3);

        let psi_extra = attr.extras.get("psi").expect("psi extra missing");
        // All cross-points sit on edges between mesh vertices, all of
        // which carry psi = 7.0 (single-tet → uniform per-vertex).
        // Linear interp between two equal values is the same value.
        for &v in psi_extra {
            assert!((v - 7.0).abs() < 1e-9);
        }
    }

    /// Winding regression: every cut triangle's normal projects
    /// non-negatively onto the cut plane axis.
    #[test]
    fn slab_cut_winding_aligned_with_axis() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let psi: [f64; 2] = [1.0, 1.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        for axis in 0..3 {
            // Pick a value that's mid-mesh by inspecting positions.
            let value = mesh.positions().iter().map(|p| p[axis]).sum::<f64>()
                / mesh.positions().len() as f64;
            let attr = slab_cut(&mesh, Plane { axis, value }, &scalars).unwrap();
            for face in &attr.geometry.faces {
                let p0 = attr.geometry.vertices[face[0] as usize];
                let p1 = attr.geometry.vertices[face[1] as usize];
                let p2 = attr.geometry.vertices[face[2] as usize];
                let a = p1 - p0;
                let b = p2 - p0;
                let cross_on_axis = match axis {
                    0 => a.y.mul_add(b.z, -(a.z * b.y)),
                    1 => a.z.mul_add(b.x, -(a.x * b.z)),
                    _ => a.x.mul_add(b.y, -(a.y * b.x)),
                };
                assert!(
                    cross_on_axis >= 0.0,
                    "cut polygon at axis={axis} value={value} has inward-facing normal: \
                     cross_on_axis={cross_on_axis}",
                );
            }
        }
    }

    /// Plane axis out of range surfaces as the typed error.
    #[test]
    fn slab_cut_rejects_invalid_axis() {
        let field = MaterialField::skeleton_default();
        let mesh = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let err = slab_cut(
            &mesh,
            Plane {
                axis: 3,
                value: 0.0,
            },
            &scalars,
        )
        .unwrap_err();
        match err {
            VizError::InvalidPlaneAxis { axis } => assert_eq!(axis, 3),
            other => panic!("unexpected error: {other:?}"),
        }
    }
}
