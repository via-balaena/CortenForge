//! FEM-grade visualization helpers for soft-body meshes.
//!
//! Six primitives, organised by which mesh drives the display geometry:
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
//! - [`design_surface`] extracts the iso-0 surface of a design SDF as a
//!   triangulated 3D mesh via marching cubes. Same scalar-transfer
//!   convention as `design_slab_cut`. Replaces [`boundary_surface`] on
//!   bodies where the design intent (smooth sphere, organic 3D scan)
//!   matters more than the analysis-mesh discretization.
//! - [`design_surface_deformed`] same as [`design_surface`] but each
//!   surface vertex is offset by the per-vertex displacement field
//!   interpolated from the analysis tet mesh, scaled by an `amplify`
//!   factor. Renders the deformed configuration with stress contours
//!   — the canonical commercial-FEM-viz convention. Pass `amplify > 1`
//!   to make sub-mm deformations visually obvious on cm-scale bodies.
//! - [`design_scene`] combines a body's [`design_surface`] with
//!   marching-cubes-only renderings of each contact primitive
//!   (rigid; no scalar field) into one [`AttributedMesh`] with a
//!   categorical `primitive_id` scalar. cf-view toggles between
//!   `primitive_id` (body-vs-contact split) and the body's continuous
//!   scalars (deformation/strain with contact primitives as uniform-
//!   zero solids).
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
//! All helpers consume any [`Mesh<M>`] sim-soft can produce and emit
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
    /// A per-vertex displacement field's length does not equal
    /// `analysis_mesh.n_vertices()`.
    PerVertexLengthMismatch {
        /// Required length (= `analysis_mesh.n_vertices()`).
        expected: usize,
        /// Length actually supplied.
        actual: usize,
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
            Self::PerVertexLengthMismatch { expected, actual } => write!(
                f,
                "per-vertex field has length {actual}, expected {expected} \
                 (= analysis_mesh.n_vertices())"
            ),
        }
    }
}

impl std::error::Error for VizError {}

/// Axis-aligned cutting plane for [`slab_cut`] and [`design_slab_cut`].
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
// equals the display vertex count we counted up. `unreachable` (in the
// inner `dispatch_cell` match) is invariant-pinned: 4 sign bits exhaust
// 16 cases.
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

/// Extract the iso-0 surface of a design SDF as a triangulated 3D
/// [`AttributedMesh`] with per-vertex scalars interpolated from
/// `analysis_mesh`.
///
/// Decouples display geometry from analysis geometry per the F2 viz
/// arc: the boundary surface comes from marching cubes on `sdf` (smooth
/// curved boundaries on organic shapes, sharp axis-aligned faces on
/// CSG primitives, exact at any analysis-mesh coarseness), while
/// scalar values come from barycentric point-in-tet interpolation
/// against `analysis_mesh`'s volume-weighted per-vertex field.
///
/// # Comparison vs [`boundary_surface`]
///
/// [`boundary_surface`] extracts the analysis tet mesh's precomputed
/// boundary faces and inherits the BCC lattice's polyhedral
/// approximation (visible sliver-tet zigzag along curved surfaces).
/// `design_surface` queries `sdf` through marching cubes instead, so
/// the body's outer shell on a sphere-like geometry is smooth, and on
/// an organic 3D-scanned body the surface follows the scan's actual
/// shape rather than the BCC lattice's approximation. `analysis_mesh`
/// is consulted only for scalar values.
///
/// # Algorithm
///
/// 1. Sample `sdf` on a 3D grid spanning `bounds` at `resolution` cell
///    size (delegated to [`mesh_offset::ScalarGrid`]).
/// 2. Run [`mesh_offset::marching_cubes`] at iso-value 0 to extract
///    the closed boundary surface as a triangulated [`IndexedMesh`].
/// 3. For each emitted display vertex, find the enclosing analysis
///    tet via barycentric point-in-tet on the per-call uniform-grid
///    spatial index (`TetGrid` keyed by tet AABB; 3×3×3 cell window
///    per probe, see Performance below). Interpolate
///    `volume_weighted_per_vertex_avg` scalars to the display vertex
///    via the barycentrics; fall back to nearest-tet (clipped +
///    renormalized barycentrics) for vertices just outside every
///    analysis tet.
///
/// # Performance
///
/// Per-probe scalar lookup uses an internal uniform-grid spatial
/// index of analysis-tet AABBs (`avg_tet_edge × 0.25` cell size,
/// 3×3×3 cell window per probe). For row 20's geometry (51 k tets,
/// ~300 k display vertices) the scalar interp step runs in ~280 ms;
/// total `design_surface` cost is dominated by the SDF grid fill
/// (~50 ms) and is well under 1 s. Pre-perf naive `O(n_display ×
/// n_tets)` walk took ~75 s on the same input; the spatial index
/// gave a ~250× speedup.
///
/// # Errors
///
/// - [`VizError::InvalidResolution`] if `resolution` is not strictly
///   positive (zero, negative, or `NaN`).
/// - [`VizError::PerTetScalarLengthMismatch`] if any scalar's length
///   != `analysis_mesh.n_tets()`.
//
// `cast_possible_truncation` allowed for the f64 → f32 PLY emit path
// and usize → u32 vertex/tet index path (`VertexId` and `TetId` are
// `u32` by crate convention). `expect_used` allowed on `insert_extra`:
// the validate-up-front contract makes the per-vertex scalar Vec
// length equal the surface vertex count by construction.
#[allow(
    clippy::cast_possible_truncation,
    clippy::expect_used,
    clippy::similar_names
)]
pub fn design_surface<M: Material>(
    sdf: &dyn Sdf,
    analysis_mesh: &dyn Mesh<M>,
    bounds: &Aabb3,
    resolution: f64,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};

    if resolution <= 0.0 || resolution.is_nan() {
        return Err(VizError::InvalidResolution { value: resolution });
    }
    validate_per_tet_scalars(analysis_mesh, per_tet_scalars)?;

    // 1. Build grid + sample SDF at every corner.
    let mut grid = ScalarGrid::from_bounds(
        NaPoint3::new(bounds.min.x, bounds.min.y, bounds.min.z),
        NaPoint3::new(bounds.max.x, bounds.max.y, bounds.max.z),
        resolution,
        0,
    );
    let (nx, ny, nz) = grid.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, sdf.eval(p));
            }
        }
    }

    // 2. Marching cubes on the iso-0 surface.
    let surface = marching_cubes(&grid, &MarchingCubesConfig::at_iso_value(0.0));

    // 3. Pre-compute volume-weighted per-vertex scalars on analysis mesh.
    let scalar_names: Vec<&str> = per_tet_scalars.keys().copied().collect();
    let per_vertex_by_scalar: Vec<Vec<f64>> = scalar_names
        .iter()
        .map(|&name| volume_weighted_per_vertex_avg(analysis_mesh, per_tet_scalars[&name]))
        .collect();

    // 4. Per-display-vertex barycentric scalar interp via uniform-grid
    //    spatial accelerator. Each probe queries a 3×3×3 cell window
    //    around its grid cell — covers tets whose AABB contains the
    //    probe AND nearby tets the probe is just outside (the design
    //    SDF's smooth surface and the analysis mesh's polyhedral
    //    surface don't coincide; nearest-tet-with-clipped-bary fallback
    //    needs to see neighboring cells to find the closest candidate).
    //    Total work: ~27 × tets-per-cell per probe vs n_tets in the
    //    pre-perf naive walk. On row 20 (51 k tets, 0.01 m cell, 305 k
    //    display vertices), naive ran ~80 s; spatial-grid ran ~few s.
    let analysis_positions = analysis_mesh.positions();
    let tet_grid = TetGrid::build(analysis_mesh, analysis_positions);

    let mut display_scalars: Vec<Vec<f64>> = scalar_names
        .iter()
        .map(|_| Vec::with_capacity(surface.vertices.len()))
        .collect();
    // Reused per-probe candidate buffer (avoids 305 k allocations on
    // row 20). Capacity sized for ~3×3×3 cells × ~20 tets/cell upper
    // bound; the Vec grows as needed.
    let mut candidates: Vec<u32> = Vec::with_capacity(512);
    for v in &surface.vertices {
        let probe = Vec3::new(v.x, v.y, v.z);
        candidates.clear();
        tet_grid.append_candidates_with_neighbors(probe, &mut candidates);
        let scalars = sample_analysis_at_point(
            probe,
            analysis_mesh,
            analysis_positions,
            &candidates,
            &per_vertex_by_scalar,
        );
        for (i, val) in scalars.iter().enumerate() {
            display_scalars[i].push(*val);
        }
    }

    // 5. Wrap as AttributedMesh.
    let mut attr = AttributedMesh::new(surface);
    for (i, &name) in scalar_names.iter().enumerate() {
        let f32_values: Vec<f32> = display_scalars[i].iter().map(|&v| v as f32).collect();
        attr.insert_extra(name.to_string(), f32_values)
            .expect("per-display-vertex length matches surface vertex count by construction");
    }

    // 6. C1 (per `docs/SIM_SOFT_ROADMAP.md` Track C): smooth-shading
    //    normals from the SDF gradient. The gradient of a signed-
    //    distance field at a zero-crossing is exactly the unit outward
    //    surface normal — the analytical alternative to area-weighted
    //    normals computed from the triangulated MC output. cf-view's
    //    `build_face_mesh` stored-normals path routes these straight
    //    through without per-triangle expansion, replacing the flat-
    //    per-triangle faceting the F2 viz arc shipped with.
    //
    //    Falls back to `+ẑ` if the gradient is degenerate (norm < tol);
    //    matches the trait's documented singularity convention.
    attr.normals = Some(
        attr.geometry
            .vertices
            .iter()
            .map(|v| {
                let g = sdf.grad(NaPoint3::new(v.x, v.y, v.z));
                let n_sq = g.z.mul_add(g.z, g.x.mul_add(g.x, g.y * g.y));
                if n_sq > 1e-30 {
                    g / n_sq.sqrt()
                } else {
                    Vec3::z()
                }
            })
            .collect(),
    );

    Ok(attr)
}

/// Same output as [`design_surface`] but each display vertex is
/// offset by the per-vertex displacement field interpolated from the
/// analysis tet mesh, scaled by `amplify`.
///
/// Renders the body's deformed configuration with stress contours,
/// the canonical commercial-FEM-viz convention. For tiny deformations
/// (sub-mm displacements on cm-scale bodies, e.g. row 20's static
/// fit pose), pass `amplify = 5.0..50.0` to make the deformation
/// visually obvious; pass `amplify = 1.0` for the true deformed shape.
///
/// # Algorithm
///
/// 1. Call [`design_surface`] for the rest geometry + scalar field.
/// 2. Rebuild the analysis-tet spatial index (the same uniform-grid
///    `TetGrid` `design_surface` uses internally).
/// 3. For each surface vertex, find the enclosing analysis tet via
///    barycentric on the spatial-index 3×3×3 cell window and
///    interpolate `displacement_per_vertex` to the surface vertex.
/// 4. Offset the surface vertex position by `displacement * amplify`.
///
/// The double-pass (one inside `design_surface` for scalars, one here
/// for displacement) costs ~2× the single-pass equivalent on row 20
/// (~600 ms vs ~300 ms); per-row scalar lookup is the dominant cost
/// in both cases. Acceptable for now; collapse into a single-pass
/// implementation if profiling shows organic-scan rows need it.
///
/// # Errors
///
/// All [`design_surface`] errors plus
/// [`VizError::PerVertexLengthMismatch`] if `displacement_per_vertex`'s
/// length differs from `analysis_mesh.n_vertices()`.
//
// `cast_possible_truncation` allowed: VertexId/TetId are u32 crate-wide
// (γ-locked invariant) so usize→u32 casts in the bary search fit.
// `similar_names` allowed: `bs[0..3]` + `v0..v3` are the tet bary +
// vertex naming convention shared with sample_analysis_at_point.
#[allow(
    clippy::too_many_arguments,
    clippy::cast_possible_truncation,
    clippy::similar_names
)]
pub fn design_surface_deformed<M: Material>(
    sdf: &dyn Sdf,
    analysis_mesh: &dyn Mesh<M>,
    bounds: &Aabb3,
    resolution: f64,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
    displacement_per_vertex: &[Vec3],
    amplify: f64,
) -> Result<AttributedMesh, VizError> {
    let n_vertices = analysis_mesh.n_vertices();
    if displacement_per_vertex.len() != n_vertices {
        return Err(VizError::PerVertexLengthMismatch {
            expected: n_vertices,
            actual: displacement_per_vertex.len(),
        });
    }

    // 1. Get rest geometry + scalar field via design_surface (also
    //    validates resolution and per_tet_scalars lengths).
    let mut attr = design_surface(sdf, analysis_mesh, bounds, resolution, per_tet_scalars)?;

    // 2. Rebuild the analysis-tet spatial index. Cost on row 20 is
    //    ~35 ms — negligible vs the per-vertex lookup pass.
    let analysis_positions = analysis_mesh.positions();
    let tet_grid = TetGrid::build(analysis_mesh, analysis_positions);

    // 3. Per-vertex displacement lookup + apply.
    let mut candidates: Vec<u32> = Vec::with_capacity(512);
    for v in &mut attr.geometry.vertices {
        let probe = Vec3::new(v.x, v.y, v.z);
        candidates.clear();
        tet_grid.append_candidates_with_neighbors(probe, &mut candidates);
        let displacement = sample_displacement_at_point(
            probe,
            analysis_mesh,
            analysis_positions,
            &candidates,
            displacement_per_vertex,
        );
        v.x += displacement.x * amplify;
        v.y += displacement.y * amplify;
        v.z += displacement.z * amplify;
    }

    // C1: the rest-state normals `design_surface` stamped via the SDF
    //    gradient no longer match the displaced surface — replace with
    //    area-weighted normals computed from the deformed triangle
    //    geometry. Loses the analytical SDF-gradient smoothness for
    //    the deformed case, but stays correct as the body squishes
    //    against the indenter; still smooth-shaded vs the F2 viz
    //    arc's flat-per-triangle default.
    attr.compute_normals();

    Ok(attr)
}

/// Combine [`design_surface`] of `body_sdf` with contact primitives.
///
/// Each entry in `contact_sdfs` is rendered geometry-only via marching
/// cubes (rigid bodies have no scalar field) and merged into a single
/// [`AttributedMesh`] with a categorical `primitive_id` scalar
/// (`0.0` = body, `1.0..=N` = each contact in `contact_sdfs` order).
///
/// Other scalars from `per_tet_scalars` (e.g. `displacement_magnitude`,
/// `material_id`, `psi_j_per_m3`) are computed for the body via the
/// usual barycentric interp and padded with `0.0` for contact-primitive
/// vertices — contact primitives are rigid and have no FEM scalar
/// field, so the uniform-zero padding is the most honest fill.
///
/// In cf-view, toggle the Scalar dropdown to `primitive_id` for a
/// categorical body-vs-contact split (canonical commercial-FEM-viz
/// scene convention); toggle to a continuous scalar (e.g.
/// `displacement_magnitude`) to see the body's field with the
/// contact primitives as uniform-zero solids.
///
/// # Errors
///
/// All [`design_surface`] errors. The contact primitives are rendered
/// geometry-only (no scalar interp), so per-tet scalar validation
/// applies to the body's pass only.
//
// `cast_precision_loss` allowed: `(idx + 1) as f32` for primitive_id
// is safe up to ~16 M contacts (well beyond any practical use).
// `cast_possible_truncation` allowed: vertex/face index casts use
// the same VertexId/TetId u32 invariant as the rest of the module.
#[allow(
    clippy::too_many_arguments,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::expect_used
)]
pub fn design_scene<M: Material>(
    body_sdf: &dyn Sdf,
    contact_sdfs: &[&dyn Sdf],
    analysis_mesh: &dyn Mesh<M>,
    bounds: &Aabb3,
    resolution: f64,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};

    // 1. Body via design_surface (handles validation).
    let mut attr = design_surface(body_sdf, analysis_mesh, bounds, resolution, per_tet_scalars)?;
    let body_vert_count = attr.geometry.vertices.len();
    let mut primitive_id: Vec<f32> = vec![0.0_f32; body_vert_count];

    // 2. Contacts via marching cubes only (rigid; no scalar interp).
    for (contact_idx, &contact_sdf) in contact_sdfs.iter().enumerate() {
        let id_value = (contact_idx + 1) as f32;
        let mut grid = ScalarGrid::from_bounds(
            NaPoint3::new(bounds.min.x, bounds.min.y, bounds.min.z),
            NaPoint3::new(bounds.max.x, bounds.max.y, bounds.max.z),
            resolution,
            0,
        );
        let (nx, ny, nz) = grid.dimensions();
        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let p = grid.position(ix, iy, iz);
                    grid.set(ix, iy, iz, contact_sdf.eval(p));
                }
            }
        }
        let surface = marching_cubes(&grid, &MarchingCubesConfig::at_iso_value(0.0));

        let base_v = attr.geometry.vertices.len() as u32;
        for v in &surface.vertices {
            attr.geometry.vertices.push(*v);
            primitive_id.push(id_value);
        }
        for face in &surface.faces {
            attr.geometry
                .faces
                .push([face[0] + base_v, face[1] + base_v, face[2] + base_v]);
        }
    }

    // 3. Pad existing extras with 0.0 for contact-primitive vertices.
    let total_vertices = attr.geometry.vertices.len();
    let extra_keys: Vec<String> = attr.extras.keys().cloned().collect();
    for key in extra_keys {
        if let Some(values) = attr.extras.get_mut(&key) {
            while values.len() < total_vertices {
                values.push(0.0_f32);
            }
        }
    }

    // 4. Insert primitive_id as the new categorical scalar.
    attr.insert_extra("primitive_id".to_string(), primitive_id)
        .expect("primitive_id length matches total vertex count by construction");

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
    candidate_tets: &[u32],
    per_vertex_by_scalar: &[Vec<f64>],
) -> Vec<f64> {
    // Tolerance is dimensionless (barycentric coordinates sum to 1.0);
    // -1e-9 forgives pure floating-point round-off without admitting
    // extrapolation into neighbouring tets.
    let tol_bary = -1e-9_f64;
    let mut best_outside_t: Option<u32> = None;
    let mut best_outside_min_b = f64::NEG_INFINITY;

    for &t in candidate_tets {
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

/// Uniform 3D grid of analysis-tet IDs, keyed by tet AABB intersection.
///
/// Cuts the per-probe scalar lookup from `O(n_tets)` to ~27 ×
/// `tets_per_cell`, where `tets_per_cell` is small (~3-5 on row-20-
/// shaped BCC + IS meshes at the empirically-tuned `avg_tet_edge ×
/// 0.25` cell size — see [`TetGrid::build`] for the cell-size choice
/// rationale).
///
/// Built once per [`design_surface`] call; cost is `O(n_tets × cells_
/// per_tet)` to register each tet in every cell its AABB overlaps.
struct TetGrid {
    /// Grid origin (mesh-AABB minimum corner, lightly padded).
    origin: Vec3,
    /// Uniform cell side length in world units.
    cell_size: f64,
    /// Cells along each axis.
    nx: usize,
    ny: usize,
    nz: usize,
    /// Flat-indexed cell storage: `cells[iz * nx * ny + iy * nx + ix]`
    /// holds the tet IDs whose AABB intersects that cell.
    cells: Vec<Vec<u32>>,
}

impl TetGrid {
    /// Build a grid from `mesh`'s tet AABBs.
    //
    // `cast_possible_truncation` + `cast_sign_loss` allowed: tet/cell
    // counts and floor/ceil indices are bounded by world extent /
    // cell_size, well below `u32::MAX` (γ-locked TetId invariant).
    // `cast_precision_loss` allowed: index → f64 in cell-size pick.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    fn build<M: Material>(mesh: &dyn Mesh<M>, positions: &[Vec3]) -> Self {
        let n_tets = mesh.n_tets();

        // Mesh AABB.
        let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in positions {
            min = min.inf(p);
            max = max.sup(p);
        }
        // Slight pad so probes exactly on the AABB boundary still hit a
        // valid cell after `floor`. Smaller than any meaningful tet edge.
        let pad = 1e-9_f64;
        min -= Vec3::new(pad, pad, pad);
        max += Vec3::new(pad, pad, pad);

        // Cell size: empirically tuned at `avg_tet_edge × 0.25` (see
        // the explicit comment + measurement just below the avg_edge
        // computation). Sample one edge per tet for the average.
        let mut total_edge = 0.0_f64;
        for t in 0..n_tets as u32 {
            let [v0, v1, _v2, _v3] = mesh.tet_vertices(t);
            total_edge += (positions[v1 as usize] - positions[v0 as usize]).norm();
        }
        let avg_edge = if n_tets > 0 {
            total_edge / n_tets as f64
        } else {
            1.0
        };
        // Empirically tuned on row 20 (51 k tets, ~50 k display vertices
        // per cardinal cross-section). `avg_edge × 0.25` minimises total
        // build + lookup time on uniform BCC + IS meshes; coarser cells
        // overload per-cell tet lists, finer cells overload the build.
        let cell_size = (avg_edge * 0.25).max(1e-6);

        let extent = max - min;
        let nx = ((extent.x / cell_size).ceil() as usize).max(1);
        let ny = ((extent.y / cell_size).ceil() as usize).max(1);
        let nz = ((extent.z / cell_size).ceil() as usize).max(1);
        let mut cells: Vec<Vec<u32>> = vec![Vec::new(); nx * ny * nz];

        for t in 0..n_tets as u32 {
            let [v0, v1, v2, v3] = mesh.tet_vertices(t);
            let p0 = positions[v0 as usize];
            let p1 = positions[v1 as usize];
            let p2 = positions[v2 as usize];
            let p3 = positions[v3 as usize];
            let tet_min = p0.inf(&p1).inf(&p2).inf(&p3);
            let tet_max = p0.sup(&p1).sup(&p2).sup(&p3);

            let ix0 = (((tet_min.x - min.x) / cell_size).floor() as usize).min(nx - 1);
            let iy0 = (((tet_min.y - min.y) / cell_size).floor() as usize).min(ny - 1);
            let iz0 = (((tet_min.z - min.z) / cell_size).floor() as usize).min(nz - 1);
            let ix1 = (((tet_max.x - min.x) / cell_size).ceil() as usize)
                .min(nx)
                .saturating_sub(1)
                .max(ix0);
            let iy1 = (((tet_max.y - min.y) / cell_size).ceil() as usize)
                .min(ny)
                .saturating_sub(1)
                .max(iy0);
            let iz1 = (((tet_max.z - min.z) / cell_size).ceil() as usize)
                .min(nz)
                .saturating_sub(1)
                .max(iz0);

            for iz in iz0..=iz1 {
                for iy in iy0..=iy1 {
                    for ix in ix0..=ix1 {
                        cells[iz * nx * ny + iy * nx + ix].push(t);
                    }
                }
            }
        }

        Self {
            origin: min,
            cell_size,
            nx,
            ny,
            nz,
            cells,
        }
    }

    /// Append tet IDs registered in `probe`'s grid cell plus the 26
    /// neighbors (3×3×3 window) into `out` (caller clears and reuses
    /// the buffer to avoid per-probe allocations). The neighbor
    /// expansion catches tets whose AABB the probe is just outside —
    /// needed for the nearest-tet fallback in
    /// [`sample_analysis_at_point`] when the design SDF surface
    /// doesn't coincide with the analysis mesh's polyhedral envelope.
    /// Duplicates are not removed; per-probe linear walk tolerates them
    /// (`sample_analysis_at_point` is idempotent under repeated tets).
    //
    // `cast_possible_truncation` + `cast_sign_loss` + `cast_possible_wrap`
    // allowed: grid dimensions are bounded by world extent / cell_size
    // which fits well below i64::MAX in any practical deployment; the
    // i64 work only exists to make the dx/dy/dz=-1..=1 boundary check
    // expressible without underflow on the lower-corner cell.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_possible_wrap
    )]
    fn append_candidates_with_neighbors(&self, probe: Vec3, out: &mut Vec<u32>) {
        let ix = ((probe.x - self.origin.x) / self.cell_size).floor() as i64;
        let iy = ((probe.y - self.origin.y) / self.cell_size).floor() as i64;
        let iz = ((probe.z - self.origin.z) / self.cell_size).floor() as i64;

        for dz in -1_i64..=1 {
            let nz = iz + dz;
            if nz < 0 || nz >= self.nz as i64 {
                continue;
            }
            for dy in -1_i64..=1 {
                let ny = iy + dy;
                if ny < 0 || ny >= self.ny as i64 {
                    continue;
                }
                for dx in -1_i64..=1 {
                    let nx = ix + dx;
                    if nx < 0 || nx >= self.nx as i64 {
                        continue;
                    }
                    let key = nz as usize * self.nx * self.ny + ny as usize * self.nx + nx as usize;
                    out.extend_from_slice(&self.cells[key]);
                }
            }
        }
    }
}

/// Find the analysis-mesh tet enclosing `probe` and return the
/// barycentric-interpolated per-vertex displacement; same enclosing-
/// tet-or-nearest-fallback logic as [`sample_analysis_at_point`] but
/// returning a [`Vec3`] instead of a per-scalar [`Vec`]. Used by
/// [`design_surface_deformed`].
//
// `cast_possible_truncation` allowed for VertexId casts (u32 crate-wide).
#[allow(clippy::cast_possible_truncation, clippy::similar_names)]
fn sample_displacement_at_point<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    candidate_tets: &[u32],
    displacement_per_vertex: &[Vec3],
) -> Vec3 {
    let tol_bary = -1e-9_f64;
    let mut best_outside_t: Option<u32> = None;
    let mut best_outside_min_b = f64::NEG_INFINITY;

    for &t in candidate_tets {
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
            return bs[0] * displacement_per_vertex[v0 as usize]
                + bs[1] * displacement_per_vertex[v1 as usize]
                + bs[2] * displacement_per_vertex[v2 as usize]
                + bs[3] * displacement_per_vertex[v3 as usize];
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
                return bs[0] * displacement_per_vertex[v0 as usize]
                    + bs[1] * displacement_per_vertex[v1 as usize]
                    + bs[2] * displacement_per_vertex[v2 as usize]
                    + bs[3] * displacement_per_vertex[v3 as usize];
            }
        }
    }
    Vec3::zeros()
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

    /// `design_surface` of a sphere produces a triangulated boundary
    /// whose total area approximates 4πR² within discretization
    /// tolerance. Validates marching cubes on the iso-0 surface +
    /// scalar transfer integration.
    #[test]
    fn design_surface_sphere_area_approximates_4_pi_r_squared() {
        use crate::SphereSdf;

        let radius = 0.05_f64;
        let sphere = SphereSdf { radius };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        // 40^3 grid over [-0.10, +0.10]^3 gives ~3% relative area error.
        let resolution = 0.005_f64;
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();

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

        let expected = 4.0 * std::f64::consts::PI * radius * radius;
        let rel_err = (area - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "sphere area {area} vs expected {expected} (rel_err {rel_err})",
        );
    }

    /// C1: `design_surface` populates per-vertex normals from the SDF
    /// gradient. On a sphere the analytical outward normal is the
    /// unit position vector `p / |p|`, so each emitted vertex on the
    /// iso-0 surface should have a stored normal aligned with its
    /// position (within MC discretization tolerance — vertices are
    /// linearly interpolated along grid edges, so they sit slightly
    /// off the true sphere).
    #[test]
    fn design_surface_sphere_populates_radial_normals() {
        use crate::SphereSdf;

        let radius = 0.05_f64;
        let sphere = SphereSdf { radius };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let resolution = 0.005_f64;
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();

        let normals = attr.normals.as_ref().expect("normals populated by C1");
        assert_eq!(normals.len(), attr.geometry.vertices.len());

        // Every normal should be a unit vector and align with its
        // vertex's outward direction. MC edge-linear-interp puts
        // vertices ~`resolution` off the true sphere, so allow a few
        // degrees of misalignment.
        for (v, n) in attr.geometry.vertices.iter().zip(normals.iter()) {
            assert!(
                (n.norm() - 1.0).abs() < 1e-9,
                "normal not unit-length: |n| = {}",
                n.norm(),
            );
            let r = nalgebra::Vector3::new(v.x, v.y, v.z);
            let r_unit = r / r.norm();
            let dot = n.dot(&r_unit);
            assert!(
                dot > 0.95,
                "normal {n:?} not aligned with radial direction {r_unit:?} (dot = {dot})",
            );
        }
    }

    /// Plane outside the SDF's interior produces an empty mesh — no
    /// triangles, no panics, no errors.
    #[test]
    fn design_surface_empty_when_bounds_outside_sdf_interior() {
        use crate::SphereSdf;

        // Sphere at origin radius 0.05; bounds well outside (entirely +x
        // octant far from origin) so the SDF is everywhere positive in
        // the grid → no iso-0 crossings.
        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(0.20, 0.20, 0.20), Vec3::new(0.30, 0.30, 0.30));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let attr = design_surface(&sphere, &analysis, &bounds, 0.005, &scalars).unwrap();

        assert_eq!(attr.geometry.faces.len(), 0);
        let psi_extra = attr.extras.get("psi").expect("psi extra missing");
        assert_eq!(psi_extra.len(), attr.geometry.vertices.len());
    }

    /// Resolution must be strictly positive — zero, negative, and `NaN`
    /// all surface as the typed error.
    #[test]
    fn design_surface_rejects_invalid_resolution() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        for bad_res in [0.0_f64, -0.001, f64::NAN] {
            let err = design_surface(&sphere, &analysis, &bounds, bad_res, &scalars).unwrap_err();
            assert!(
                matches!(err, VizError::InvalidResolution { .. }),
                "bad_res = {bad_res}: expected InvalidResolution, got {err:?}",
            );
        }
    }

    /// Per-tet scalar with wrong length surfaces as the typed error.
    #[test]
    fn design_surface_rejects_wrong_length_scalar() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        // SingleTetMesh has 1 tet; supply length 2 to trip the gate.
        let psi = [1.0_f64, 2.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let err = design_surface(&sphere, &analysis, &bounds, 0.005, &scalars).unwrap_err();
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

    /// Zero-displacement case: `design_surface_deformed` with all-zero
    /// displacement matches `design_surface` bit-for-bit on positions.
    #[test]
    fn design_surface_deformed_zero_displacement_matches_rest() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let resolution = 0.005_f64;
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let displacement = vec![Vec3::zeros(); analysis.n_vertices()];
        let amplify = 1.0_f64;

        let rest = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
        let deformed = design_surface_deformed(
            &sphere,
            &analysis,
            &bounds,
            resolution,
            &scalars,
            &displacement,
            amplify,
        )
        .unwrap();

        assert_eq!(
            rest.geometry.vertices.len(),
            deformed.geometry.vertices.len()
        );
        for (a, b) in rest
            .geometry
            .vertices
            .iter()
            .zip(deformed.geometry.vertices.iter())
        {
            assert!((a.x - b.x).abs() < 1e-12);
            assert!((a.y - b.y).abs() < 1e-12);
            assert!((a.z - b.z).abs() < 1e-12);
        }
    }

    /// Uniform displacement translates every surface vertex by exactly
    /// `displacement * amplify`. Validates that the barycentric interp
    /// (and the clipped+renormalized fallback) preserves uniform fields.
    /// Uses an offset sphere fitted inside `SingleTetMesh`'s AABB so
    /// every surface vertex falls within the tet-grid spatial index's
    /// search window (otherwise fallback returns zeros).
    #[test]
    fn design_surface_deformed_uniform_displacement_translates_uniformly() {
        struct OffsetSphere {
            center: Vec3,
            radius: f64,
        }
        impl Sdf for OffsetSphere {
            fn eval(&self, p: NaPoint3<f64>) -> f64 {
                (Vec3::new(p.x, p.y, p.z) - self.center).norm() - self.radius
            }
            fn grad(&self, _p: NaPoint3<f64>) -> Vec3 {
                Vec3::z()
            }
        }
        // Sphere centered inside SingleTetMesh's AABB ([0, 0.1]^3),
        // small enough to fit entirely within the tet's neighborhood.
        let sphere = OffsetSphere {
            center: Vec3::new(0.04, 0.04, 0.04),
            radius: 0.015,
        };
        let bounds = Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.1, 0.1, 0.1));
        let resolution = 0.0025_f64;
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let uniform = Vec3::new(0.001, 0.002, 0.003);
        let displacement = vec![uniform; analysis.n_vertices()];

        for amplify in [1.0_f64, 2.0, 10.0] {
            let rest = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
            let deformed = design_surface_deformed(
                &sphere,
                &analysis,
                &bounds,
                resolution,
                &scalars,
                &displacement,
                amplify,
            )
            .unwrap();
            assert!(
                !rest.geometry.vertices.is_empty(),
                "rest surface should be non-empty"
            );

            let expected = uniform * amplify;
            for (a, b) in rest
                .geometry
                .vertices
                .iter()
                .zip(deformed.geometry.vertices.iter())
            {
                assert!(
                    (b.x - a.x - expected.x).abs() < 1e-9,
                    "amplify={amplify}: x diff {} expected {}",
                    b.x - a.x,
                    expected.x
                );
                assert!((b.y - a.y - expected.y).abs() < 1e-9);
                assert!((b.z - a.z - expected.z).abs() < 1e-9);
            }
        }
    }

    /// Mismatched displacement length surfaces as the typed error.
    #[test]
    fn design_surface_deformed_rejects_wrong_displacement_length() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        // SingleTetMesh has 4 vertices; supply length 2 to trip the gate.
        let bad_displacement = vec![Vec3::zeros(); 2];

        let err = design_surface_deformed(
            &sphere,
            &analysis,
            &bounds,
            0.005,
            &scalars,
            &bad_displacement,
            1.0,
        )
        .unwrap_err();
        match err {
            VizError::PerVertexLengthMismatch { expected, actual } => {
                assert_eq!(expected, 4);
                assert_eq!(actual, 2);
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }

    /// Empty `contact_sdfs` produces output equal to `design_surface`
    /// on geometry + scalar field, plus a `primitive_id` scalar that's
    /// uniform `0.0` across the body.
    #[test]
    fn design_scene_empty_contacts_matches_design_surface() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let resolution = 0.005_f64;
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let body_only = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
        let scene = design_scene(&sphere, &[], &analysis, &bounds, resolution, &scalars).unwrap();

        assert_eq!(
            body_only.geometry.vertices.len(),
            scene.geometry.vertices.len()
        );
        assert_eq!(body_only.geometry.faces.len(), scene.geometry.faces.len());

        let primitive_id = scene
            .extras
            .get("primitive_id")
            .expect("primitive_id missing");
        assert_eq!(primitive_id.len(), scene.geometry.vertices.len());
        for &id in primitive_id {
            assert!(
                (id - 0.0).abs() < 1e-9,
                "expected body primitive_id 0.0, got {id}"
            );
        }
    }

    /// Two contact primitives produce `primitive_id` values 0.0 (body),
    /// 1.0 (first contact), 2.0 (second contact), each in a contiguous
    /// vertex range. Other scalars are padded with 0.0 for contacts.
    #[test]
    fn design_scene_two_contacts_categorical_primitive_id() {
        use crate::SphereSdf;

        // Body sphere + two non-overlapping contact spheres at offsets.
        struct OffsetSphere {
            cx: f64,
            r: f64,
        }
        impl Sdf for OffsetSphere {
            fn eval(&self, p: NaPoint3<f64>) -> f64 {
                let dx = p.x - self.cx;
                dx.mul_add(dx, p.y.mul_add(p.y, p.z * p.z)).sqrt() - self.r
            }
            fn grad(&self, _p: NaPoint3<f64>) -> Vec3 {
                Vec3::z()
            }
        }

        let body = SphereSdf { radius: 0.05 };
        let contact_a = OffsetSphere { cx: 0.07, r: 0.015 };
        let contact_b = OffsetSphere {
            cx: -0.07,
            r: 0.015,
        };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let scene = design_scene(
            &body,
            &[&contact_a as &dyn Sdf, &contact_b as &dyn Sdf],
            &analysis,
            &bounds,
            0.005,
            &scalars,
        )
        .unwrap();

        let primitive_id = scene
            .extras
            .get("primitive_id")
            .expect("primitive_id missing");

        // All three categorical values present.
        let mut counts = [0_usize; 3];
        for &id in primitive_id {
            let bucket = if (id - 0.0).abs() < 0.5 {
                0
            } else if (id - 1.0).abs() < 0.5 {
                1
            } else if (id - 2.0).abs() < 0.5 {
                2
            } else {
                panic!("primitive_id {id} out of expected range 0..=2");
            };
            counts[bucket] += 1;
        }
        assert!(counts[0] > 0, "expected body vertices (id=0)");
        assert!(counts[1] > 0, "expected contact_a vertices (id=1)");
        assert!(counts[2] > 0, "expected contact_b vertices (id=2)");

        // psi extra is body-only-meaningful — padded 0.0 for contacts.
        // Total length matches vertex count.
        let psi_extra = scene.extras.get("psi").expect("psi missing");
        assert_eq!(psi_extra.len(), scene.geometry.vertices.len());
    }
}
