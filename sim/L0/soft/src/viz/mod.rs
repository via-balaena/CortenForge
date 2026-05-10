//! FEM-grade visualization helpers for soft-body meshes.
//!
//! Three primitives, organised by which mesh drives the display geometry:
//!
//! ## Analysis-mesh primitives (F1)
//!
//! Display geometry comes from the analysis tet mesh's connectivity.
//! The boundary or cross-section IS the polyhedral approximation the
//! solver actually used.
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
//! ## Design-mesh primitives (F2)
//!
//! Display geometry comes from the design SDF (the user-authored
//! `Solid` / `Sdf` that drove the analysis-mesh build). The analysis
//! mesh is consulted only for scalar values; the geometry is exact at
//! any analysis-mesh coarseness.
//!
//! - [`design_slab_cut`] intersects the design SDF with an axis-aligned
//!   plane via marching-squares-filled and emits the cross-section
//!   polygon. Per-vertex scalars come from barycentric point-in-tet
//!   interpolation against the analysis mesh's volume-weighted
//!   per-vertex field; falls back to nearest-tet (clipped barycentric)
//!   for SDF-isosurface points just outside the analysis mesh's
//!   polyhedral boundary.
//!
//! Pick F1 when the analysis-mesh discretization IS the story (e.g.,
//! debugging mesh quality, sliver-tet inspection). Pick F2 when the
//! design intent is the story (smooth boundaries, sharp creases at
//! axis-aligned faces, organic-scan body geometry rendered the way the
//! user authored it). On the canonical row-20 layered-silicone-device
//! cavity-fit case, F2's `design_slab_cut` produces a clean smooth
//! circle-minus-square cross-section while F1's `slab_cut` shows a
//! chunky polyhedral approximation with sliver-tet zigzag along
//! curved boundaries.
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
use nalgebra::{Matrix3, Point3 as NaPoint3};

use crate::Vec3;
use crate::material::Material;
use crate::mesh::{Mesh, VertexId};
use crate::sdf_bridge::{Aabb3, Sdf};

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
    /// Marching-squares grid resolution must be strictly positive.
    InvalidResolution {
        /// Bad resolution value supplied (in plane units, m).
        value: f64,
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
            Self::InvalidResolution { value } => {
                write!(f, "marching-squares resolution must be > 0, got {value}")
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
    clippy::similar_names,
    // The `_ => unreachable!()` arm in the (na, nb) match is invariant-
    // pinned: a tet has exactly 4 vertices, so na + nb == 4 and the 5
    // explicit arms (4,0)/(0,4)/(1,3)/(3,1)/(2,2) exhaust the space.
    // The `unreachable!()` cannot fire under any well-formed input.
    clippy::unreachable
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

/// Intersect a design SDF with an axis-aligned plane and return a
/// triangulated cross-section [`AttributedMesh`] with per-vertex
/// scalars interpolated from `analysis_mesh`.
///
/// Decouples display geometry from analysis geometry per the F2 viz
/// arc: the cross-section comes from marching-squares-filled on `sdf`
/// (sharp at axis-aligned faces, smooth on curved boundaries, exact
/// at any analysis-mesh coarseness), while scalar values come from
/// barycentric point-in-tet interpolation against `analysis_mesh`'s
/// volume-weighted per-vertex field.
///
/// # Comparison vs [`slab_cut`]
///
/// [`slab_cut`] intersects the analysis tet mesh directly and inherits
/// the polyhedral approximation (sliver-tet zigzag on curved
/// boundaries, sawtooth at sharp edges from any cell's diagonal pick).
/// `design_slab_cut` queries `sdf` instead, so curved boundaries (e.g.
/// a sphere body) are smooth and axis-aligned boundaries (e.g. a
/// carved cube cavity) are sharp regardless of the analysis mesh's
/// coarseness. `analysis_mesh` is consulted only for scalar values.
///
/// # Algorithm
///
/// 1. Sample `sdf` on a 2D grid in the cutting plane at `resolution`
///    cell size, spanning `bounds` projected onto the (u, v) axes
///    perpendicular to `plane.axis`.
/// 2. Marching-squares-filled: for each grid cell, classify the 4
///    corners by SDF sign (strictly negative = inside) and emit
///    interior triangles via 16-case dispatch. Saddle cases
///    (2-opposite-inside, masks `0b0101` / `0b1010`) resolve via
///    `sdf.eval(cell_center)` so opposite-corner islands bridge or
///    split as the SDF dictates.
/// 3. For each emitted display vertex, find the enclosing analysis
///    tet via barycentric point-in-tet on a slab-filtered tet subset
///    (only tets whose AABB along `plane.axis` straddles `plane.value`
///    can possibly contain a point on the plane). Interpolate
///    `volume_weighted_per_vertex_avg` scalars to the display vertex
///    via the barycentrics (same per-vertex projection [`slab_cut`]
///    uses; `volume_weighted_per_vertex_avg` is private to this module).
/// 4. Fall back to nearest-tet (clipped + renormalized barycentrics)
///    for vertices that fall just outside every analysis tet — common
///    where the design SDF's smooth surface lies outside the analysis
///    mesh's polyhedral envelope.
///
/// CCW (u, v) winding picks (uu, vv) such that uu × vv = +`plane.axis`,
/// matching [`slab_cut`]'s outward-winding convention so cf-view's
/// single-sided lighting renders consistently.
///
/// # Errors
///
/// - [`VizError::InvalidPlaneAxis`] if `plane.axis >= 3`.
/// - [`VizError::InvalidResolution`] if `resolution` is not strictly
///   positive (zero, negative, or `NaN`).
/// - [`VizError::PerTetScalarLengthMismatch`] if any scalar's length
///   != `analysis_mesh.n_tets()`.
//
// `cast_possible_truncation` allowed for f64 → f32 (PLY emit format)
// and usize → u32 (VertexId is u32 crate-wide). `cast_precision_loss`
// allowed for grid index → f64 (grid count is bounded above by world
// extent / resolution, which fits in f64 well below 2^53). `expect_used`
// allowed on `insert_extra` — by construction each scalar Vec's length
// equals the display vertex count we counted up. `unreachable` allowed
// in the (na, nb)/(mask) match — 4 sign bits exhaust 16 cases.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::too_many_lines,
    clippy::similar_names
)]
pub fn design_slab_cut<M: Material>(
    sdf: &dyn Sdf,
    analysis_mesh: &dyn Mesh<M>,
    plane: Plane,
    bounds: &Aabb3,
    resolution: f64,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    if plane.axis >= 3 {
        return Err(VizError::InvalidPlaneAxis { axis: plane.axis });
    }
    if resolution <= 0.0 || resolution.is_nan() {
        return Err(VizError::InvalidResolution { value: resolution });
    }
    validate_per_tet_scalars(analysis_mesh, per_tet_scalars)?;

    let (uu, vv) = uv_axes(plane.axis);
    let grid = SdfGrid::sample(sdf, plane, bounds, resolution, uu, vv);

    let scalar_names: Vec<&str> = per_tet_scalars.keys().copied().collect();
    let per_vertex_by_scalar: Vec<Vec<f64>> = scalar_names
        .iter()
        .map(|&name| volume_weighted_per_vertex_avg(analysis_mesh, per_tet_scalars[&name]))
        .collect();

    let analysis_positions = analysis_mesh.positions();
    let slab_tets: Vec<u32> = (0..analysis_mesh.n_tets() as u32)
        .filter(|&t| {
            let [v0, v1, v2, v3] = analysis_mesh.tet_vertices(t);
            let coords = [v0, v1, v2, v3].map(|v| analysis_positions[v as usize][plane.axis]);
            let lo = coords.iter().copied().fold(f64::INFINITY, f64::min);
            let hi = coords.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            lo <= plane.value && plane.value <= hi
        })
        .collect();

    let mut state = DesignCutState::new();
    grid.sweep(sdf, &mut state);

    let mut display_scalars: Vec<Vec<f64>> = scalar_names
        .iter()
        .map(|_| Vec::with_capacity(state.positions.len()))
        .collect();
    for &p in &state.positions {
        let probe = Vec3::new(p[0], p[1], p[2]);
        let scalars = sample_analysis_at_point(
            probe,
            analysis_mesh,
            analysis_positions,
            &slab_tets,
            &per_vertex_by_scalar,
        );
        for (i, val) in scalars.iter().enumerate() {
            display_scalars[i].push(*val);
        }
    }

    let mut geometry = IndexedMesh::new();
    geometry.vertices = state
        .positions
        .iter()
        .map(|p| Point3::new(p[0], p[1], p[2]))
        .collect();
    geometry.faces = state.faces;

    let mut attr = AttributedMesh::new(geometry);
    for (i, &name) in scalar_names.iter().enumerate() {
        let f32_values: Vec<f32> = display_scalars[i].iter().map(|&v| v as f32).collect();
        attr.insert_extra(name.to_string(), f32_values)
            .expect("per-display-vertex length matches positions count by construction");
    }
    Ok(attr)
}

/// Pair of in-plane axis indices `(uu, vv)` such that the right-hand-rule
/// cross product `e_uu × e_vv = +e_axis`. CCW polygons in the (u, v)
/// frame thus have outward face normals along `+axis`, matching the
/// analysis-mesh slab-cut + boundary-surface winding convention.
const fn uv_axes(axis: usize) -> (usize, usize) {
    match axis {
        0 => (1, 2),
        1 => (2, 0),
        _ => (0, 1),
    }
}

/// 2D scalar grid of SDF samples in the cutting plane, plus the
/// supporting frame data needed for per-cell case dispatch.
struct SdfGrid {
    plane: Plane,
    uu: usize,
    vv: usize,
    nu: usize,
    nv: usize,
    u_min: f64,
    v_min: f64,
    du: f64,
    dv: f64,
    sdf_values: Vec<f64>,
}

impl SdfGrid {
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_sign_loss,
        clippy::cast_possible_truncation,
        clippy::similar_names
    )]
    fn sample(
        sdf: &dyn Sdf,
        plane: Plane,
        bounds: &Aabb3,
        resolution: f64,
        uu: usize,
        vv: usize,
    ) -> Self {
        let u_min = bounds.min[uu];
        let u_max = bounds.max[uu];
        let v_min = bounds.min[vv];
        let v_max = bounds.max[vv];
        let nu_minus_1 = ((u_max - u_min) / resolution).ceil().max(1.0) as usize;
        let nv_minus_1 = ((v_max - v_min) / resolution).ceil().max(1.0) as usize;
        let nu = nu_minus_1 + 1;
        let nv = nv_minus_1 + 1;
        let du = (u_max - u_min) / nu_minus_1 as f64;
        let dv = (v_max - v_min) / nv_minus_1 as f64;

        let mut sdf_values = vec![0.0_f64; nu * nv];
        for iv in 0..nv {
            for iu in 0..nu {
                let p = Self::corner_pos_static(plane, uu, vv, u_min, v_min, du, dv, iu, iv);
                sdf_values[iv * nu + iu] = sdf.eval(NaPoint3::new(p[0], p[1], p[2]));
            }
        }

        Self {
            plane,
            uu,
            vv,
            nu,
            nv,
            u_min,
            v_min,
            du,
            dv,
            sdf_values,
        }
    }

    #[allow(clippy::cast_precision_loss, clippy::too_many_arguments)]
    fn corner_pos_static(
        plane: Plane,
        uu: usize,
        vv: usize,
        u_min: f64,
        v_min: f64,
        du: f64,
        dv: f64,
        iu: usize,
        iv: usize,
    ) -> [f64; 3] {
        let mut p = [0.0_f64; 3];
        p[plane.axis] = plane.value;
        p[uu] = (iu as f64).mul_add(du, u_min);
        p[vv] = (iv as f64).mul_add(dv, v_min);
        p
    }

    fn corner_pos(&self, iu: usize, iv: usize) -> [f64; 3] {
        Self::corner_pos_static(
            self.plane, self.uu, self.vv, self.u_min, self.v_min, self.du, self.dv, iu, iv,
        )
    }

    const fn corner_key(&self, iu: usize, iv: usize) -> usize {
        iv * self.nu + iu
    }

    fn corner_sdf(&self, iu: usize, iv: usize) -> f64 {
        self.sdf_values[self.corner_key(iu, iv)]
    }

    #[allow(clippy::cast_precision_loss, clippy::similar_names)]
    fn cell_center(&self, iu: usize, iv: usize) -> [f64; 3] {
        let mut p = [0.0_f64; 3];
        p[self.plane.axis] = self.plane.value;
        let iu_f = iu as f64 + 0.5;
        let iv_f = iv as f64 + 0.5;
        p[self.uu] = iu_f.mul_add(self.du, self.u_min);
        p[self.vv] = iv_f.mul_add(self.dv, self.v_min);
        p
    }

    /// Sweep all interior cells, dispatching marching-squares-filled
    /// cases into `state`'s vertex / face accumulators.
    fn sweep(&self, sdf: &dyn Sdf, state: &mut DesignCutState) {
        for iv in 0..self.nv - 1 {
            for iu in 0..self.nu - 1 {
                self.dispatch_cell(iu, iv, sdf, state);
            }
        }
    }

    #[allow(clippy::too_many_lines, clippy::unreachable, clippy::similar_names)]
    fn dispatch_cell(&self, iu: usize, iv: usize, sdf: &dyn Sdf, state: &mut DesignCutState) {
        let s00 = self.corner_sdf(iu, iv);
        let s10 = self.corner_sdf(iu + 1, iv);
        let s11 = self.corner_sdf(iu + 1, iv + 1);
        let s01 = self.corner_sdf(iu, iv + 1);

        // Strict `< 0` (not `<= 0`) avoids spurious triangles at SDF=0
        // surfaces that align with grid corners (e.g. axis-aligned cube
        // faces in row-20 sphere-minus-cube CSG): with `<= 0` a corner
        // on the cube face plus an adjacent corner inside the cube
        // interior produces a pentagon whose triangle pokes INTO the
        // cube cavity. Strict `<` classifies surface corners as outside,
        // so cells fully inside subtractive primitives emit no triangles.
        let mask = u8::from(s00 < 0.0)
            | (u8::from(s10 < 0.0) << 1)
            | (u8::from(s11 < 0.0) << 2)
            | (u8::from(s01 < 0.0) << 3);

        match mask {
            0b0000 => {}
            0b0001 => {
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                state.faces.push([i_c00, i_ea, i_ed]);
            }
            0b0010 => {
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                state.faces.push([i_c10, i_eb, i_ea]);
            }
            0b0100 => {
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                state.faces.push([i_c11, i_ec, i_eb]);
            }
            0b1000 => {
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                state.faces.push([i_c01, i_ed, i_ec]);
            }
            0b0011 => {
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                state.faces.push([i_c00, i_c10, i_eb]);
                state.faces.push([i_c00, i_eb, i_ed]);
            }
            0b0110 => {
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                state.faces.push([i_c10, i_c11, i_ec]);
                state.faces.push([i_c10, i_ec, i_ea]);
            }
            0b1100 => {
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                state.faces.push([i_c11, i_c01, i_ed]);
                state.faces.push([i_c11, i_ed, i_eb]);
            }
            0b1001 => {
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                state.faces.push([i_c00, i_ea, i_ec]);
                state.faces.push([i_c00, i_ec, i_c01]);
            }
            0b0101 => {
                // Saddle: corners 00 + 11 inside. Resolve via center.
                let center = self.cell_center(iu, iv);
                let center_in = sdf.eval(NaPoint3::new(center[0], center[1], center[2])) < 0.0;
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                if center_in {
                    state.faces.push([i_c00, i_ea, i_eb]);
                    state.faces.push([i_c00, i_eb, i_c11]);
                    state.faces.push([i_c00, i_c11, i_ec]);
                    state.faces.push([i_c00, i_ec, i_ed]);
                } else {
                    state.faces.push([i_c00, i_ea, i_ed]);
                    state.faces.push([i_c11, i_ec, i_eb]);
                }
            }
            0b1010 => {
                // Saddle: corners 10 + 01 inside.
                let center = self.cell_center(iu, iv);
                let center_in = sdf.eval(NaPoint3::new(center[0], center[1], center[2])) < 0.0;
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                if center_in {
                    state.faces.push([i_c10, i_eb, i_ec]);
                    state.faces.push([i_c10, i_ec, i_c01]);
                    state.faces.push([i_c10, i_c01, i_ed]);
                    state.faces.push([i_c10, i_ed, i_ea]);
                } else {
                    state.faces.push([i_c10, i_eb, i_ea]);
                    state.faces.push([i_c01, i_ed, i_ec]);
                }
            }
            0b1110 => {
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                state.faces.push([i_ea, i_c10, i_c11]);
                state.faces.push([i_ea, i_c11, i_c01]);
                state.faces.push([i_ea, i_c01, i_ed]);
            }
            0b1101 => {
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_ea = state.emit_edge(self, iu, iv, iu + 1, iv);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                state.faces.push([i_c00, i_ea, i_eb]);
                state.faces.push([i_c00, i_eb, i_c11]);
                state.faces.push([i_c00, i_c11, i_c01]);
            }
            0b1011 => {
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_eb = state.emit_edge(self, iu + 1, iv, iu + 1, iv + 1);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                state.faces.push([i_c00, i_c10, i_eb]);
                state.faces.push([i_c00, i_eb, i_ec]);
                state.faces.push([i_c00, i_ec, i_c01]);
            }
            0b0111 => {
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_ec = state.emit_edge(self, iu + 1, iv + 1, iu, iv + 1);
                let i_ed = state.emit_edge(self, iu, iv + 1, iu, iv);
                state.faces.push([i_c00, i_c10, i_c11]);
                state.faces.push([i_c00, i_c11, i_ec]);
                state.faces.push([i_c00, i_ec, i_ed]);
            }
            0b1111 => {
                let i_c00 = state.emit_corner(self, iu, iv);
                let i_c10 = state.emit_corner(self, iu + 1, iv);
                let i_c11 = state.emit_corner(self, iu + 1, iv + 1);
                let i_c01 = state.emit_corner(self, iu, iv + 1);
                state.faces.push([i_c00, i_c10, i_c11]);
                state.faces.push([i_c00, i_c11, i_c01]);
            }
            _ => unreachable!("4 sign bits exhaust 16 cases"),
        }
    }
}

/// Vertex / face accumulator for [`design_slab_cut`]. Dedupes corner
/// vertices by flat-grid index and cross-edge vertices by sorted
/// `(corner_a, corner_b)` key, sharing one positions [`Vec`].
struct DesignCutState {
    positions: Vec<[f64; 3]>,
    faces: Vec<[u32; 3]>,
    corner_to_idx: HashMap<usize, u32>,
    edge_to_idx: HashMap<(usize, usize), u32>,
}

impl DesignCutState {
    fn new() -> Self {
        Self {
            positions: Vec::new(),
            faces: Vec::new(),
            corner_to_idx: HashMap::new(),
            edge_to_idx: HashMap::new(),
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn emit_corner(&mut self, grid: &SdfGrid, iu: usize, iv: usize) -> u32 {
        let key = grid.corner_key(iu, iv);
        if let Some(&idx) = self.corner_to_idx.get(&key) {
            return idx;
        }
        let idx = self.positions.len() as u32;
        self.positions.push(grid.corner_pos(iu, iv));
        self.corner_to_idx.insert(key, idx);
        idx
    }

    #[allow(clippy::cast_possible_truncation, clippy::similar_names)]
    fn emit_edge(
        &mut self,
        grid: &SdfGrid,
        iu_a: usize,
        iv_a: usize,
        iu_b: usize,
        iv_b: usize,
    ) -> u32 {
        let ka = grid.corner_key(iu_a, iv_a);
        let kb = grid.corner_key(iu_b, iv_b);
        let key = if ka < kb { (ka, kb) } else { (kb, ka) };
        if let Some(&idx) = self.edge_to_idx.get(&key) {
            return idx;
        }
        let idx = self.positions.len() as u32;
        let sa = grid.sdf_values[ka];
        let sb = grid.sdf_values[kb];
        // Caller invariant: only called when sa and sb straddle 0
        // (one strictly negative, the other non-negative). Denominator
        // sa - sb is non-zero by construction.
        let t = sa / (sa - sb);
        let pa = grid.corner_pos(iu_a, iv_a);
        let pb = grid.corner_pos(iu_b, iv_b);
        self.positions.push([
            (pb[0] - pa[0]).mul_add(t, pa[0]),
            (pb[1] - pa[1]).mul_add(t, pa[1]),
            (pb[2] - pa[2]).mul_add(t, pa[2]),
        ]);
        self.edge_to_idx.insert(key, idx);
        idx
    }
}

/// Find the analysis-mesh tet enclosing `probe` and return barycentric-
/// interpolated per-vertex scalars; falls back to nearest-tet (clipped
/// + renormalized barycentrics) for points just outside every tet.
//
// `cast_possible_truncation` allowed for VertexId casts (u32 crate-wide).
#[allow(clippy::cast_possible_truncation, clippy::similar_names)]
fn sample_analysis_at_point<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    slab_tets: &[u32],
    per_vertex_by_scalar: &[Vec<f64>],
) -> Vec<f64> {
    // Tolerance is dimensionless (barycentric coordinates sum to 1.0);
    // -1e-9 forgives pure floating-point round-off without admitting
    // extrapolation into neighbouring tets.
    let tol_bary = -1e-9_f64;
    let mut best_outside_t: Option<u32> = None;
    let mut best_outside_min_b = f64::NEG_INFINITY;

    for &t in slab_tets {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let m = Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
        if m.determinant().abs() < 1e-30 {
            continue;
        }
        let Some(inv) = m.try_inverse() else {
            continue;
        };
        let b = inv * (probe - p0);
        let b0 = 1.0 - b.x - b.y - b.z;
        let bs = [b0, b.x, b.y, b.z];
        let min_b = bs.iter().copied().fold(f64::INFINITY, f64::min);
        if min_b >= tol_bary {
            return per_vertex_by_scalar
                .iter()
                .map(|pv| {
                    bs[3].mul_add(
                        pv[v3 as usize],
                        bs[2].mul_add(
                            pv[v2 as usize],
                            bs[1].mul_add(pv[v1 as usize], bs[0] * pv[v0 as usize]),
                        ),
                    )
                })
                .collect();
        }
        if min_b > best_outside_min_b {
            best_outside_min_b = min_b;
            best_outside_t = Some(t);
        }
    }

    if let Some(t) = best_outside_t {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let m = Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
        if let Some(inv) = m.try_inverse() {
            let b = inv * (probe - p0);
            let b0 = 1.0 - b.x - b.y - b.z;
            let mut bs = [b0.max(0.0), b.x.max(0.0), b.y.max(0.0), b.z.max(0.0)];
            let sum: f64 = bs.iter().sum();
            if sum > 0.0 {
                for v in &mut bs {
                    *v /= sum;
                }
                return per_vertex_by_scalar
                    .iter()
                    .map(|pv| {
                        bs[3].mul_add(
                            pv[v3 as usize],
                            bs[2].mul_add(
                                pv[v2 as usize],
                                bs[1].mul_add(pv[v1 as usize], bs[0] * pv[v0 as usize]),
                            ),
                        )
                    })
                    .collect();
            }
        }
    }
    per_vertex_by_scalar.iter().map(|_| 0.0).collect()
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

    /// `design_slab_cut` of a sphere at z = 0 produces a triangulated
    /// disk whose total area approximates π r² within discretization
    /// tolerance. SDF query path validates marching-squares-filled.
    #[test]
    fn design_slab_cut_disk_area_approximates_pi_r_squared() {
        use crate::SphereSdf;

        let radius = 0.05_f64;
        let sphere = SphereSdf { radius };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        // 80×80 grid over [−0.10, +0.10]² gives ~3% relative area error.
        let resolution = 0.0025_f64;
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = design_slab_cut(
            &sphere,
            &analysis,
            Plane {
                axis: 2,
                value: 0.0,
            },
            &bounds,
            resolution,
            &scalars,
        )
        .unwrap();

        let area: f64 = attr
            .geometry
            .faces
            .iter()
            .map(|face| {
                let p0 = attr.geometry.vertices[face[0] as usize];
                let p1 = attr.geometry.vertices[face[1] as usize];
                let p2 = attr.geometry.vertices[face[2] as usize];
                let a = p1 - p0;
                let b = p2 - p0;
                0.5 * a.cross(&b).norm()
            })
            .sum();

        let expected = std::f64::consts::PI * radius * radius;
        let rel_err = (area - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "disk area {area} vs expected {expected} (rel_err {rel_err})",
        );
    }

    /// Plane outside the SDF's interior produces an empty mesh — no
    /// triangles, no panics, no errors.
    #[test]
    fn design_slab_cut_empty_outside_sdf_interior() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        // Plane at z = 0.08 is strictly outside the sphere of radius 0.05.
        let attr = design_slab_cut(
            &sphere,
            &analysis,
            Plane {
                axis: 2,
                value: 0.08,
            },
            &bounds,
            0.005,
            &scalars,
        )
        .unwrap();

        assert_eq!(attr.geometry.faces.len(), 0);
        assert_eq!(attr.geometry.vertices.len(), 0);
        let psi_extra = attr.extras.get("psi").expect("psi extra missing");
        assert_eq!(psi_extra.len(), 0);
    }

    /// Plane axis out of range surfaces as the typed error.
    #[test]
    fn design_slab_cut_rejects_invalid_axis() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let err = design_slab_cut(
            &sphere,
            &analysis,
            Plane {
                axis: 7,
                value: 0.0,
            },
            &bounds,
            0.005,
            &scalars,
        )
        .unwrap_err();
        match err {
            VizError::InvalidPlaneAxis { axis } => assert_eq!(axis, 7),
            other => panic!("unexpected error: {other:?}"),
        }
    }

    /// Resolution must be strictly positive — zero, negative, and `NaN`
    /// all surface as the typed error.
    #[test]
    fn design_slab_cut_rejects_invalid_resolution() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        for bad_res in [0.0_f64, -0.001, f64::NAN] {
            let err = design_slab_cut(
                &sphere,
                &analysis,
                Plane {
                    axis: 2,
                    value: 0.0,
                },
                &bounds,
                bad_res,
                &scalars,
            )
            .unwrap_err();
            assert!(
                matches!(err, VizError::InvalidResolution { .. }),
                "bad_res = {bad_res}: expected InvalidResolution, got {err:?}",
            );
        }
    }

    /// Per-tet scalar with wrong length surfaces as the typed error.
    #[test]
    fn design_slab_cut_rejects_wrong_length_scalar() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        // SingleTetMesh has 1 tet; supply length 2 to trip the gate.
        let psi = [1.0_f64, 2.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let err = design_slab_cut(
            &sphere,
            &analysis,
            Plane {
                axis: 2,
                value: 0.0,
            },
            &bounds,
            0.005,
            &scalars,
        )
        .unwrap_err();
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

    /// Saddle-case (2-opposite-corner) resolution: the test uses a
    /// pair of disjoint spheres synthesised via `min(sphere_a, sphere_b)`.
    /// At a cell where opposite corners are inside (one in each sphere)
    /// and adjacent corners are outside, the cell-center SDF is positive
    /// (between spheres → outside), so the saddle resolves to TWO
    /// disjoint triangles, not a connecting hexagon. Verifies the
    /// 0b0101 / 0b1010 dispatch's center-eval path.
    #[test]
    fn design_slab_cut_saddle_resolves_via_center_eval() {
        // Two spheres centered at (-d, 0, 0) and (+d, 0, 0), both
        // radius r, with d > r so they don't overlap.
        struct TwoSpheres {
            r: f64,
            d: f64,
        }
        impl Sdf for TwoSpheres {
            fn eval(&self, p: NaPoint3<f64>) -> f64 {
                let a = (p - NaPoint3::new(-self.d, 0.0, 0.0)).norm() - self.r;
                let b = (p - NaPoint3::new(self.d, 0.0, 0.0)).norm() - self.r;
                a.min(b)
            }
            fn grad(&self, _p: NaPoint3<f64>) -> Vec3 {
                Vec3::z()
            }
        }
        let two_spheres = TwoSpheres { r: 0.02, d: 0.04 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = design_slab_cut(
            &two_spheres,
            &analysis,
            Plane {
                axis: 2,
                value: 0.0,
            },
            &bounds,
            0.005,
            &scalars,
        )
        .unwrap();

        // Total area should be ~2 × π × r² for two disks.
        let area: f64 = attr
            .geometry
            .faces
            .iter()
            .map(|face| {
                let p0 = attr.geometry.vertices[face[0] as usize];
                let p1 = attr.geometry.vertices[face[1] as usize];
                let p2 = attr.geometry.vertices[face[2] as usize];
                let a = p1 - p0;
                let b = p2 - p0;
                0.5 * a.cross(&b).norm()
            })
            .sum();
        let expected = 2.0 * std::f64::consts::PI * 0.02 * 0.02;
        let rel_err = (area - expected).abs() / expected;
        assert!(
            rel_err < 0.10,
            "two-disk area {area} vs expected {expected} (rel_err {rel_err})",
        );
    }
}
