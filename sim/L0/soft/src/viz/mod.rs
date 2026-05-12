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
//! - [`slab_cut_deformed`] same as [`slab_cut`] but each rest position
//!   is offset by the per-vertex displacement field, scaled by an
//!   `amplify` factor. Renders the cross-section of the deformed tet
//!   mesh — sliced cavity walls + contact-zone bulges that are
//!   invisible from a closed outer boundary surface.
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
//! - [`design_scene_deformed`] same as [`design_scene`] but the body
//!   pass uses [`design_surface_deformed`] — quasi-static ramp
//!   animations of a soft body with a moving rigid plug land here.
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

/// C2.2 IDW hyperparameter — top-k count for the continuous-scalar
/// kNN-IDW sampler [`idw_k_nearest_tet_centroids`].
///
/// k = 8 matches the typical tets-per-cell on a BCC analysis mesh
/// (six BCC tets per cubic cell, plus boundary-overflow from the
/// 3×3×3 `TetGrid` window) — the smoothing kernel is effectively
/// "one Voronoi neighborhood." Smaller k narrows the smoothing
/// (k = 1 collapses to nearest-tet, identical to the C2.1 categorical
/// path); larger k widens it but admits centroids from further away
/// that aren't physically relevant to the probe.
const IDW_K: usize = 8;

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
/// on cm-scale bodies, matching the [`design_surface_deformed`]
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

/// Intersect a design SDF with an axis-aligned plane and return a
/// triangulated cross-section [`AttributedMesh`] with per-vertex
/// scalars interpolated from `analysis_mesh`.
///
/// Decouples display geometry from analysis geometry per the F2 viz
/// arc: the cross-section comes from marching-squares-filled on `sdf`
/// (sharp at axis-aligned faces, smooth on curved boundaries, exact
/// at any analysis-mesh coarseness), while scalar values are sampled
/// per display vertex from `analysis_mesh`'s per-tet field (barycentric
/// interp of the volume-weighted-per-vertex field for continuous
/// scalars; single nearest-tet-centroid for categorical `_id` scalars;
/// see Algorithm below).
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
/// 3. For each emitted display vertex, sample from `analysis_mesh`'s
///    per-tet field over a slab-filtered tet subset (only tets whose
///    AABB along `plane.axis` straddles `plane.value` can possibly
///    contain a point on the plane). Continuous scalars: barycentric
///    point-in-tet on the volume-weighted-per-vertex field (same
///    per-vertex projection [`slab_cut`] uses). Categorical scalars
///    (suffix `_id`): single nearest-tet-centroid via
///    `nearest_tet_centroid_idx` — preserves integer labels for the
///    Categorical colormap pick.
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

    // C2.1 categorical/continuous split. Continuous scalars pre-
    // project to vol-weighted-per-vertex so `sample_analysis_at_point`
    // can barycentric-interpolate inside the enclosing slab tet;
    // categorical scalars (suffix `_id`) skip the pre-projection and
    // share a single `nearest_tet_centroid_idx` lookup per display
    // vertex against `slab_tets`, emitting the exact integer of the
    // nearest tet. Mirrors the dispatch in `design_surface` (the C2.2
    // kNN-IDW continuous-path swap is deliberately scope-limited to
    // `design_surface` — `design_slab_cut` doesn't build a `TetGrid`
    // and adding one is a perf-vs-benefit trade banked for C2.x).
    let scalar_names: Vec<&str> = per_tet_scalars.keys().copied().collect();
    let is_categorical: Vec<bool> = scalar_names
        .iter()
        .map(|&n| is_categorical_scalar_name(n))
        .collect();
    let any_categorical = is_categorical.iter().any(|&b| b);
    let cont_per_vertex_by_scalar: Vec<Vec<f64>> = scalar_names
        .iter()
        .zip(is_categorical.iter())
        .filter_map(|(&name, &cat)| {
            if cat {
                None
            } else {
                Some(volume_weighted_per_vertex_avg(
                    analysis_mesh,
                    per_tet_scalars[&name],
                ))
            }
        })
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
        let cont_vals = sample_analysis_at_point(
            probe,
            analysis_mesh,
            analysis_positions,
            &slab_tets,
            &cont_per_vertex_by_scalar,
        );
        let nearest = if any_categorical {
            nearest_tet_centroid_idx(probe, analysis_mesh, analysis_positions, &slab_tets)
        } else {
            None
        };
        let mut next_cont = 0_usize;
        for (i, &name) in scalar_names.iter().enumerate() {
            let val = if is_categorical[i] {
                nearest.map_or(0.0, |t| per_tet_scalars[&name][t as usize])
            } else {
                let v = cont_vals[next_cont];
                next_cont += 1;
                v
            };
            display_scalars[i].push(val);
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
/// scalar values are sampled per probe from `analysis_mesh`'s per-tet
/// field (Shepard kNN-IDW for continuous scalars, single nearest-tet-
/// centroid for categorical `_id` scalars; see Algorithm below).
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
/// 3. For each emitted display vertex, query a per-call uniform-grid
///    spatial index (`TetGrid` keyed by tet AABB; 3×3×3 cell window
///    per probe, see Performance below) for nearby analysis tets.
///    Continuous scalars sample via Shepard kNN-IDW (top-k=8 by
///    centroid distance, weights `1/(d²+ε)`; see
///    `idw_k_nearest_tet_centroids`). Categorical scalars (suffix
///    `_id`) sample via single nearest-tet-centroid (k=1; see
///    `nearest_tet_centroid_idx`) shared across every categorical
///    scalar this probe — preserves integer labels for the cf-view
///    colormap detector's Categorical (`tab10`) pick. Both paths
///    share the same 3×3×3 candidate pool.
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
// `too_many_lines` allowed: the C2 categorical/continuous split adds
// a partition pass + nearest-tet branch alongside the existing
// barycentric path, pushing the body past clippy's 100-line default;
// extracting either half to a helper would inflate the API surface
// for a single internal use.
#[allow(
    clippy::cast_possible_truncation,
    clippy::expect_used,
    clippy::similar_names,
    clippy::too_many_lines
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

    // 3. C2 categorical/continuous split. Continuous scalars sample
    //    via Shepard kNN-IDW (`idw_k_nearest_tet_centroids`) per probe
    //    directly on per-tet values; categorical scalars (suffix
    //    `_id`) sample via single nearest-tet-centroid
    //    (`nearest_tet_centroid_idx`, k=1) per probe, emitting the
    //    exact integer of that tet. Both paths share the same
    //    `TetGrid` 3×3×3 cell window. Preserves the cf-view colormap
    //    detector's Categorical pick (`tab10`) at layer boundaries
    //    instead of falling back to Sequential viridis on fractional
    //    samples.
    let scalar_names: Vec<&str> = per_tet_scalars.keys().copied().collect();
    let is_categorical: Vec<bool> = scalar_names
        .iter()
        .map(|&n| is_categorical_scalar_name(n))
        .collect();
    let any_categorical = is_categorical.iter().any(|&b| b);
    // Continuous slots only, in `scalar_names` order with categorical
    // slots skipped. C2.2 samples each probe directly from per-tet
    // values via kNN-IDW (no precomputed vol-weighted-per-vertex
    // intermediate), so we just collect the underlying `&[f64]`
    // references — `idw_k_nearest_tet_centroids` does the smoothing
    // at sample time.
    let cont_per_tet_refs: Vec<&[f64]> = scalar_names
        .iter()
        .zip(is_categorical.iter())
        .filter_map(|(&name, &cat)| {
            if cat {
                None
            } else {
                Some(per_tet_scalars[&name])
            }
        })
        .collect();

    // 4. Per-display-vertex scalar sampling via uniform-grid spatial
    //    accelerator. Each probe queries a 3×3×3 cell window around
    //    its grid cell — covers tets whose AABB contains the probe
    //    AND nearby tets the probe is just outside. On row 20 (51 k
    //    tets, 0.01 m cell, 305 k display vertices) the naive
    //    O(n_tets) walk ran ~80 s; spatial-grid lookup runs in seconds.
    //    Categorical scalars share a single `nearest_tet_centroid_idx`
    //    lookup per probe (C2.1 k=1); continuous scalars share a
    //    single `idw_k_nearest_tet_centroids` pass per probe (C2.2
    //    Shepard's IDW over the top-k=8 nearest centroids in the
    //    same 3×3×3 candidate pool).
    let analysis_positions = analysis_mesh.positions();
    let tet_grid = TetGrid::build(analysis_mesh, analysis_positions);

    // C2.2 IDW eps floor: scale-invariant weight clamp that collapses
    // the IDW kernel to nearest-tet behavior when the probe lies
    // essentially on a centroid (avoids 1/0 blow-up while staying
    // negligible at any geometric distance above the floor).
    // `IDW_K` is module-level (see top of file).
    let idw_eps = (tet_grid.cell_size * 1e-3).powi(2);

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

        // Continuous: Shepard-style kNN-IDW (k=8) directly on per-tet
        // values. Wider smoothing radius than the pre-C2.2 single-
        // enclosing-tet barycentric path; smooths the patchwork
        // gradient-jump-at-tet-faces pattern visible on ψ fields.
        // Empty `cont_per_tet_refs` (all-categorical case) returns
        // an empty `cont_vals` (the helper short-circuits at
        // `n_scalars == 0`).
        let cont_vals = idw_k_nearest_tet_centroids(
            probe,
            analysis_mesh,
            analysis_positions,
            &candidates,
            &cont_per_tet_refs,
            IDW_K,
            idw_eps,
        );

        // Categorical: single nearest-tet-centroid lookup shared
        // across every categorical scalar this probe. `None` only
        // when `candidates` is empty (the probe lies outside every
        // analysis tet's 3×3×3 cell neighborhood) — fall back to
        // `0.0`, matching the orphan convention elsewhere in the
        // module.
        let nearest = if any_categorical {
            nearest_tet_centroid_idx(probe, analysis_mesh, analysis_positions, &candidates)
        } else {
            None
        };

        // Recombine into `scalar_names` order. `next_cont` advances
        // in lockstep with continuous slots so categorical positions
        // don't consume an entry from `cont_vals`.
        let mut next_cont = 0_usize;
        for (i, &name) in scalar_names.iter().enumerate() {
            let val = if is_categorical[i] {
                nearest.map_or(0.0, |t| per_tet_scalars[&name][t as usize])
            } else {
                let v = cont_vals[next_cont];
                next_cont += 1;
                v
            };
            display_scalars[i].push(val);
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
/// `material_shell_id`, `psi_j_per_m3`) are computed for the body via
/// [`design_surface`]'s dispatched scalar pipeline (kNN-IDW for
/// continuous, nearest-tet for categorical `_id` scalars) and padded
/// with `0.0` for contact-primitive vertices — contact primitives are
/// rigid and have no FEM scalar field, so the uniform-zero padding is
/// the most honest fill.
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
pub fn design_scene<M: Material>(
    body_sdf: &dyn Sdf,
    contact_sdfs: &[&dyn Sdf],
    analysis_mesh: &dyn Mesh<M>,
    bounds: &Aabb3,
    resolution: f64,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<AttributedMesh, VizError> {
    // 1. Body via design_surface (handles validation; stamps SDF-gradient
    //    analytical normals on body vertices per C1).
    let mut attr = design_surface(body_sdf, analysis_mesh, bounds, resolution, per_tet_scalars)?;
    // 2. Append contacts + extras pad + primitive_id stamp (shared with
    //    `design_scene_deformed`).
    append_contact_primitives(&mut attr, contact_sdfs, bounds, resolution);
    Ok(attr)
}

/// Deformed-body sibling of [`design_scene`].
///
/// The body pass uses [`design_surface_deformed`], so the body
/// renders in its deformed configuration while the contact primitives
/// render at their rest pose. Quasi-static intrusion ramps with a
/// moving rigid plug land here — one PLY per ramp step with both the
/// squishing body and the descending plug.
///
/// # Algorithm
///
/// 1. Body via [`design_surface_deformed`] (validation + displacement
///    + area-weighted normals on the deformed surface).
/// 2. Each contact primitive via marching cubes only (rigid; no
///    scalar interp). Contact normals come from each contact SDF's
///    gradient at the iso-0 surface (analytical, matching the C1
///    convention).
/// 3. Existing scalars are padded with `0.0` for contact-primitive
///    vertices; a categorical `primitive_id` scalar is stamped
///    (`0.0` = body, `1.0..=N` = each contact in input order).
///
/// # Errors
///
/// All [`design_surface_deformed`] errors. Contact primitives are
/// rendered geometry-only, so per-tet scalar validation applies to
/// the body pass only.
//
// `too_many_arguments` allowed: 8 args matches the F2 design-mesh
// convention (sdf + analysis_mesh + bounds + resolution + scalars +
// displacement + amplify, plus the `contact_sdfs` slot
// `design_scene` adds). Splitting into a config struct would
// double the call-site noise without clarifying the contract.
#[allow(clippy::too_many_arguments)]
pub fn design_scene_deformed<M: Material>(
    body_sdf: &dyn Sdf,
    contact_sdfs: &[&dyn Sdf],
    analysis_mesh: &dyn Mesh<M>,
    bounds: &Aabb3,
    resolution: f64,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
    displacement_per_vertex: &[Vec3],
    amplify: f64,
) -> Result<AttributedMesh, VizError> {
    let mut attr = design_surface_deformed(
        body_sdf,
        analysis_mesh,
        bounds,
        resolution,
        per_tet_scalars,
        displacement_per_vertex,
        amplify,
    )?;
    append_contact_primitives(&mut attr, contact_sdfs, bounds, resolution);
    Ok(attr)
}

/// Append marching-cubes renderings of each contact primitive into
/// `attr`, pad existing extras with `0.0` over the new vertex range,
/// and stamp a categorical `primitive_id` extra (`0.0` = body,
/// `1.0..=N` = each contact in input order).
///
/// If `attr.normals` is already populated by the body pass, extends
/// the normal vec with analytical unit normals computed from each
/// contact SDF's gradient at the iso-0 surface (falls back to `+ẑ`
/// on degenerate gradient, matching the [`design_surface`]
/// convention). Keeps the slot-length invariant the PLY writer
/// validates.
//
// `cast_precision_loss` allowed: `(idx + 1) as f32` for primitive_id
// is safe up to ~16 M contacts (well beyond any practical use).
// `cast_possible_truncation` allowed: vertex index casts use the
// same VertexId/TetId u32 invariant as the rest of the module.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::expect_used
)]
fn append_contact_primitives(
    attr: &mut AttributedMesh,
    contact_sdfs: &[&dyn Sdf],
    bounds: &Aabb3,
    resolution: f64,
) {
    use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};

    let body_vert_count = attr.geometry.vertices.len();
    let mut primitive_id: Vec<f32> = vec![0.0_f32; body_vert_count];
    let extend_normals = attr.normals.is_some();

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

        if extend_normals {
            if let Some(normals) = attr.normals.as_mut() {
                normals.reserve(surface.vertices.len());
                for v in &surface.vertices {
                    let g = contact_sdf.grad(NaPoint3::new(v.x, v.y, v.z));
                    let n_sq = g.z.mul_add(g.z, g.x.mul_add(g.x, g.y * g.y));
                    let n = if n_sq > 1e-30 {
                        g / n_sq.sqrt()
                    } else {
                        Vec3::z()
                    };
                    normals.push(n);
                }
            }
        }
    }

    let total_vertices = attr.geometry.vertices.len();
    let extra_keys: Vec<String> = attr.extras.keys().cloned().collect();
    for key in extra_keys {
        if let Some(values) = attr.extras.get_mut(&key) {
            while values.len() < total_vertices {
                values.push(0.0_f32);
            }
        }
    }

    attr.insert_extra("primitive_id".to_string(), primitive_id)
        .expect("primitive_id length matches total vertex count by construction");
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
    // `cast_precision_loss` + `cast_sign_loss` + `cast_possible_truncation`
    // allowed: usize ↔ f64 round-trip for the 2D grid sweep
    // (`(u_max - u_min) / resolution as usize`, then `nu_minus_1 as f64`
    // for the back-conversion to step size). Grid dimensions stay
    // within u32 range at any canonical bounds + resolution.
    // `similar_names` allowed: `nu`/`nv`, `u_min`/`v_min`, `du`/`dv`,
    // `uu`/`vv` are the algorithm's natural in-plane axis names;
    // renaming would obscure the 2D-sweep symmetry.
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

    // `cast_precision_loss` allowed: `(iu as f64).mul_add(du, u_min)`
    // converts a grid index to its world-space coordinate; usize → f64
    // on indices well below 2^53.
    // `too_many_arguments` allowed: this is the `static` helper
    // [`corner_pos`] forwards to during `&self` construction, where
    // `&self` doesn't yet exist (see `sample`). Bundling into a config
    // struct would add wrapping cost for two callers with no clarity
    // gain.
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

    // `cast_precision_loss` allowed: `iu as f64 + 0.5` converts a
    // cell-index to its 0.5-offset cell-center coordinate; the indices
    // are well below 2^53.
    // `similar_names` allowed: `iu_f`/`iv_f` are the f64 versions of
    // the integer indices, following the existing
    // `uu`/`vv`/`du`/`dv`/`u_min`/`v_min` 2-axis naming convention.
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

    // `too_many_lines` allowed: marching-squares dispatch enumerates
    // the 16 corner-sign configurations explicitly (a single 4-bit
    // case-table); condensing the per-case emit would obscure the
    // 1:1 mapping with the marching-squares lookup table.
    // `unreachable` allowed: the case-table dispatch is exhaustive
    // over the 4-bit corner pattern; the catch-all arm is invariant-
    // pinned and cannot fire on a well-formed cell.
    // `similar_names` allowed: `s00`/`s10`/`s11`/`s01` are the
    // canonical names for the 4 corner SDF values (uv-quadrant
    // indexing); these match the marching-squares literature.
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

    // `cast_possible_truncation` allowed: `self.positions.len() as u32`
    // packs the new vertex index — VertexId is u32 by the mesh-types
    // convention, and cross-section vertex counts stay comfortably
    // below u32::MAX at any canonical resolution.
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

    // `cast_possible_truncation` allowed: same `positions.len() as u32`
    // vertex-ID pack as `emit_corner`; vertex counts stay within u32.
    // `similar_names` allowed: `iu_a`/`iv_a`/`iu_b`/`iv_b` are the
    // endpoint coordinates of a cell edge; the `_a`/`_b` suffixes
    // mirror the corner-naming pattern at the call sites.
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

/// C2 categorical-scalar predicate: a per-tet scalar whose **name** ends
/// in `_id` is treated as a categorical integer label (e.g.
/// `material_shell_id`, `material_zone_id`, `primitive_id`) and routed
/// through the nearest-tet path that preserves the integer; anything
/// else (`psi_j_per_m3`, `displacement_magnitude`, …) is treated as a
/// continuous field and routed through the per-primitive continuous-
/// scalar path — volume-weighted-per-vertex (on [`boundary_surface`]),
/// linear-along-edge from the volume-weighted-per-vertex field (on
/// [`slab_cut`] / [`slab_cut_deformed`] / [`design_slab_cut`]), or
/// Shepard kNN-IDW (on [`design_surface`] and its deformed/scene
/// siblings; see [`idw_k_nearest_tet_centroids`]).
///
/// Continuous-scalar smoothing breaks the categorical invariant —
/// averaging a `_id = 1` tet with a `_id = 2` tet yields `1.5`, which
/// the cf-view colormap detector reads as Sequential rather than
/// Categorical (it falls back to viridis at every layer boundary).
/// Routing categorical scalars to nearest-tet (k=1) keeps display-vertex
/// samples exactly integer, preserving the tab10 categorical render.
///
/// Suffix-by-name is the lightest convention that matches the existing
/// producer code (`material_shell_id`, `material_zone_id`,
/// `primitive_id`); per-scalar metadata on [`mesh_types::AttributedMesh`]
/// is a cleaner long-term answer if `_id` ever becomes a footgun, and is
/// banked as a C2 followup.
#[inline]
#[must_use]
fn is_categorical_scalar_name(name: &str) -> bool {
    name.ends_with("_id")
}

/// Pick the tet from `candidate_tets` whose centroid is closest to
/// `probe`, returning its tet ID. Returns `None` when `candidate_tets`
/// is empty.
///
/// Used by the C2 categorical sampling path on [`design_surface`]
/// (and its deformed/scene siblings) and on [`design_slab_cut`]: one
/// nearest-tet lookup per probe is shared across every categorical
/// scalar in flight (all categorical scalars sample from the same
/// nearest tet, so the centroid search amortizes). [`slab_cut`] /
/// [`slab_cut_deformed`] use a related position-based pick via
/// `SlabCutState`'s `cut_owner_dist2` tracking rather than calling
/// this helper directly — the cut-vertex's edge-star tets are visited
/// one-at-a-time as the marching-tet loop iterates, so the winner
/// promotion happens incrementally on edge dedup.
//
// `cast_possible_truncation` allowed: VertexId/TetId are u32 by crate
// convention so `mesh.n_tets() < u32::MAX` is a γ-locked invariant.
#[allow(clippy::cast_possible_truncation)]
fn nearest_tet_centroid_idx<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    candidate_tets: &[u32],
) -> Option<u32> {
    let mut best: Option<u32> = None;
    let mut best_d2 = f64::INFINITY;
    for &t in candidate_tets {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let centroid = (positions[v0 as usize]
            + positions[v1 as usize]
            + positions[v2 as usize]
            + positions[v3 as usize])
            * 0.25;
        let d2 = (probe - centroid).norm_squared();
        if d2 < best_d2 {
            best_d2 = d2;
            best = Some(t);
        }
    }
    best
}

/// C2.2 continuous-scalar sampler: Shepard-style k-nearest-tet IDW
/// (inverse-distance-weighted average) at an arbitrary 3D probe point.
///
/// Returns one f64 per scalar in `per_tet_scalars` order. Picks the
/// `k` tets from `candidate_tets` with smallest centroid-to-probe
/// distance and computes the weighted average:
/// `value = Σ (w_i * per_tet[t_i]) / Σ w_i` where
/// `w_i = 1 / (d_i² + eps)` and `d_i` is the centroid-to-probe
/// distance for tet `t_i`. The `eps` floor avoids weight blow-up when
/// the probe coincides with a centroid (the limit collapses to
/// nearest-tet behavior, which matches the C2.1 categorical path).
///
/// Wider smoothing radius than the pre-C2.2 vol-weighted-per-vertex +
/// barycentric path: that path averaged over each MC vertex's one
/// enclosing analysis tet, producing piecewise-linear-per-tet output
/// with gradient jumps at tet faces (visible as a "patchwork" pattern
/// on the ψ field of `design_surface` output). kNN-IDW averages over
/// k tets regardless of enclosing-tet boundaries, producing visibly
/// smoother gradients on the design-mesh display surface.
///
/// `candidate_tets` is expected to be a `TetGrid` 3×3×3 cell window
/// around the probe; empty candidates → returns `vec![0.0; n_scalars]`
/// (orphan-probe convention shared with the rest of the module).
//
// `cast_possible_truncation` allowed: VertexId/TetId are u32 by crate
// convention; `mesh.n_tets() < u32::MAX` is a γ-locked invariant.
#[allow(clippy::cast_possible_truncation)]
fn idw_k_nearest_tet_centroids<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    candidate_tets: &[u32],
    per_tet_scalars: &[&[f64]],
    k: usize,
    eps: f64,
) -> Vec<f64> {
    let n_scalars = per_tet_scalars.len();
    if candidate_tets.is_empty() || n_scalars == 0 {
        return vec![0.0; n_scalars];
    }

    // (squared_distance_to_centroid, tet_id) for each candidate.
    let mut by_dist2: Vec<(f64, u32)> = candidate_tets
        .iter()
        .map(|&t| {
            let [v0, v1, v2, v3] = mesh.tet_vertices(t);
            let centroid = (positions[v0 as usize]
                + positions[v1 as usize]
                + positions[v2 as usize]
                + positions[v3 as usize])
                * 0.25;
            ((probe - centroid).norm_squared(), t)
        })
        .collect();

    // Sort ascending by squared distance. For BCC + 3×3×3 TetGrid
    // window the candidate count is ~150 and k is ~8; full sort is
    // cheap and simpler than a partial-sort heap.
    by_dist2.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let k_eff = k.min(by_dist2.len());

    let mut total_weight = 0.0_f64;
    let mut sums = vec![0.0_f64; n_scalars];
    for &(d2, t) in &by_dist2[..k_eff] {
        let w = 1.0 / (d2 + eps);
        total_weight += w;
        for (i, &per_tet) in per_tet_scalars.iter().enumerate() {
            sums[i] = w.mul_add(per_tet[t as usize], sums[i]);
        }
    }

    if total_weight > 0.0 {
        sums.iter().map(|&s| s / total_weight).collect()
    } else {
        // All weights numerically zero — logically unreachable: the
        // empty-candidates case returns at the top of the fn, and
        // `1 / (d² + eps)` is strictly positive for any finite `d²`
        // with `eps > 0`. Keep the branch as a defensive zero-fill
        // so the caller's per-scalar Vec stays correctly sized.
        vec![0.0; n_scalars]
    }
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
    clippy::cast_precision_loss,
    // `float_cmp` on values that have just been `.round()`'d to an
    // integer in the C2 categorical tests; comparing the rounded
    // result against a small integer literal is exact by
    // construction (integer-valued f64 has no fp-noise window).
    clippy::float_cmp
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

    /// C2 categorical (suffix `_id`): every per-vertex value must be
    /// EXACTLY integer. Pre-C2.1 the volume-weighted-per-vertex average
    /// at the 3 shared-face vertices of `two_tet_shared_face` would
    /// have produced a fractional value mid-way between the two zones;
    /// the C2 nearest-tet (k=1, largest-volume incident tet) path
    /// emits exactly one of the two input integers at every vertex.
    #[test]
    fn boundary_surface_categorical_id_stays_integer() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let zone_id: [f64; 2] = [3.0, 7.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("material_zone_id", &zone_id);

        let attr = boundary_surface(&mesh, &scalars).unwrap();
        let zone_extra = attr
            .extras
            .get("material_zone_id")
            .expect("zone extra missing");
        assert_eq!(zone_extra.len(), 5);

        for &v in zone_extra {
            let rounded = v.round();
            assert!(
                (v - rounded).abs() < 1e-9,
                "categorical value {v} is fractional — C2 nearest-tet \
                 path should keep every value integer",
            );
            assert!(
                rounded == 3.0 || rounded == 7.0,
                "categorical value {rounded} is not one of the input \
                 integers {{3, 7}}",
            );
        }
    }

    /// C2 mixed scalars: when continuous (`psi`) and categorical
    /// (`material_zone_id`) are emitted in the same call, the
    /// continuous slot must be BIT-EXACT identical to a continuous-
    /// only call (so the C2.1 split doesn't perturb the volume-
    /// weighted-per-vertex pipeline), and the categorical slot stays
    /// integer.
    #[test]
    fn boundary_surface_mixed_continuous_and_categorical_preserves_continuous() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let psi: [f64; 2] = [1.0, 3.0];
        let zone_id: [f64; 2] = [3.0, 7.0];

        // Continuous-only baseline.
        let mut cont_only: BTreeMap<&str, &[f64]> = BTreeMap::new();
        cont_only.insert("psi", &psi);
        let baseline = boundary_surface(&mesh, &cont_only).unwrap();

        // Mixed.
        let mut mixed: BTreeMap<&str, &[f64]> = BTreeMap::new();
        mixed.insert("psi", &psi);
        mixed.insert("material_zone_id", &zone_id);
        let attr = boundary_surface(&mesh, &mixed).unwrap();

        // Continuous slot bit-exact preserved.
        let psi_baseline = baseline.extras.get("psi").expect("psi baseline");
        let psi_mixed = attr.extras.get("psi").expect("psi mixed");
        assert_eq!(
            psi_baseline, psi_mixed,
            "continuous `psi` slot must be bit-exact identical when a \
             categorical scalar is added to the call",
        );

        // Categorical slot stays integer.
        let zone = attr
            .extras
            .get("material_zone_id")
            .expect("zone extra missing");
        for &v in zone {
            assert!(
                (v - v.round()).abs() < 1e-9,
                "categorical value {v} fractional",
            );
        }
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

    /// C2 categorical (suffix `_id`): every cut-vertex value must be
    /// EXACTLY integer. Pre-C2.1 the linear interp along each crossed
    /// edge between vol-weighted-per-vertex averages would have
    /// produced fractional values at any cut vertex on an edge shared
    /// between the two zones; the C2 nearest-tet (k=1, largest-volume
    /// tet wins on edge dedup) path emits one of the input integers
    /// at every cut vertex.
    #[test]
    fn slab_cut_categorical_id_stays_integer() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let zone_id: [f64; 2] = [3.0, 7.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("material_zone_id", &zone_id);

        // z = 0.05 crosses both tets — tet 0 (corner) in a (1, 3)
        // pattern (v3 above), tet 1 (apex jut) in a (2, 2) pattern
        // (v3 + v4 above). Both contribute cut vertices on the
        // shared edges (v1, v3) and (v2, v3), which dedup to one cut
        // vertex per shared edge whose owner is the larger-volume tet.
        let attr = slab_cut(
            &mesh,
            Plane {
                axis: 2,
                value: 0.05,
            },
            &scalars,
        )
        .unwrap();
        let zone = attr
            .extras
            .get("material_zone_id")
            .expect("zone extra missing");
        assert!(
            !zone.is_empty(),
            "z = 0.05 must produce a non-empty cross-section on \
             two_tet_shared_face",
        );
        for &v in zone {
            let rounded = v.round();
            assert!(
                (v - rounded).abs() < 1e-9,
                "categorical cut-vertex value {v} is fractional — \
                 C2 nearest-tet path should keep every value integer",
            );
            assert!(
                rounded == 3.0 || rounded == 7.0,
                "categorical cut-vertex value {rounded} is not one \
                 of the input integers {{3, 7}}",
            );
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

    /// `slab_cut_deformed` with zero displacement matches `slab_cut`
    /// bit-exact (positions identical, scalars identical, face count
    /// identical). Mismatched displacement length surfaces as the
    /// typed error.
    #[test]
    fn slab_cut_deformed_zero_displacement_matches_rest() {
        let field = MaterialField::skeleton_default();
        let mesh = SingleTetMesh::new(&field);
        let psi = [7.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let n_v = mesh.n_vertices();
        let zero_disp = vec![Vec3::zeros(); n_v];
        let plane = Plane {
            axis: 0,
            value: 0.05,
        };

        let rest = slab_cut(&mesh, plane, &scalars).unwrap();
        let deformed = slab_cut_deformed(&mesh, plane, &scalars, &zero_disp, 1.0).unwrap();

        assert_eq!(
            rest.geometry.vertices.len(),
            deformed.geometry.vertices.len()
        );
        assert_eq!(rest.geometry.faces.len(), deformed.geometry.faces.len());
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

        // Mismatched displacement length trips the typed error.
        let bad_disp = vec![Vec3::zeros(); 2];
        let err = slab_cut_deformed(&mesh, plane, &scalars, &bad_disp, 1.0).unwrap_err();
        match err {
            VizError::PerVertexLengthMismatch { expected, actual } => {
                assert_eq!(expected, 4);
                assert_eq!(actual, 2);
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }

    /// Uniform displacement translates the cross-section by
    /// `displacement * amplify` along each axis, leaving topology
    /// (vertex count, face count) and the in-plane shape intact.
    #[test]
    fn slab_cut_deformed_uniform_displacement_translates_cross_section() {
        let field = MaterialField::skeleton_default();
        let mesh = SingleTetMesh::new(&field);
        let psi = [7.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let n_v = mesh.n_vertices();
        // Translate in +z so the x = 0.05 cross-section is unchanged
        // in (x, y) but shifted in z by displacement * amplify.
        let uniform = Vec3::new(0.0, 0.0, 0.001);
        let displacement = vec![uniform; n_v];
        let amplify = 5.0_f64;

        let plane = Plane {
            axis: 0,
            value: 0.05,
        };
        let rest = slab_cut(&mesh, plane, &scalars).unwrap();
        let deformed = slab_cut_deformed(&mesh, plane, &scalars, &displacement, amplify).unwrap();

        assert_eq!(rest.geometry.faces.len(), deformed.geometry.faces.len());
        assert_eq!(
            rest.geometry.vertices.len(),
            deformed.geometry.vertices.len()
        );
        let expected_dz = uniform.z * amplify;
        for (a, b) in rest
            .geometry
            .vertices
            .iter()
            .zip(deformed.geometry.vertices.iter())
        {
            assert!((b.x - a.x).abs() < 1e-9);
            assert!((b.y - a.y).abs() < 1e-9);
            assert!(
                (b.z - a.z - expected_dz).abs() < 1e-9,
                "expected dz={expected_dz}, got {}",
                b.z - a.z
            );
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

    /// C2 categorical (suffix `_id`): every MC display-vertex value
    /// must be EXACTLY integer. Pre-C2.1 the barycentric interp inside
    /// the enclosing analysis tet — whose 4 vertices carry vol-weighted
    /// averages that differ at the shared-face vertices — would have
    /// produced fractional values at every MC vert; the C2 nearest-
    /// tet-centroid (k=1, shared across all categorical scalars per
    /// probe) path emits exactly one of the input integers.
    ///
    /// The sphere isosurface at radius 0.015 centered at the tet 1
    /// centroid (0.045, 0.045, 0.045) sits fully inside the analysis
    /// mesh's combined AABB (0..0.1 each axis), so every MC vertex has
    /// candidate analysis tets in its 3×3×3 cell window.
    #[test]
    fn design_surface_categorical_id_stays_integer() {
        use crate::SphereSdf;

        // Sphere offset from origin so the iso-0 surface lies inside
        // the analysis mesh's interior (origin-anchored sphere would
        // collapse to a single point on the analysis-mesh boundary
        // and produce zero MC verts).
        struct OffsetSphere {
            center: Vec3,
            inner: SphereSdf,
        }
        impl crate::sdf_bridge::Sdf for OffsetSphere {
            fn eval(&self, p: nalgebra::Point3<f64>) -> f64 {
                let shifted = nalgebra::Point3::new(
                    p.x - self.center.x,
                    p.y - self.center.y,
                    p.z - self.center.z,
                );
                self.inner.eval(shifted)
            }
            fn grad(&self, p: nalgebra::Point3<f64>) -> Vec3 {
                let shifted = nalgebra::Point3::new(
                    p.x - self.center.x,
                    p.y - self.center.y,
                    p.z - self.center.z,
                );
                self.inner.grad(shifted)
            }
        }

        let sphere = OffsetSphere {
            center: Vec3::new(0.045, 0.045, 0.045),
            inner: SphereSdf { radius: 0.015 },
        };
        let bounds = Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.1, 0.1, 0.1));
        let resolution = 0.005_f64;
        let field = MaterialField::skeleton_default();
        let analysis = HandBuiltTetMesh::two_tet_shared_face(&field);
        let shell_id: [f64; 2] = [3.0, 7.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("material_shell_id", &shell_id);

        let attr = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
        let shell = attr
            .extras
            .get("material_shell_id")
            .expect("shell extra missing");
        assert!(
            !shell.is_empty(),
            "sphere isosurface should produce a non-empty MC mesh \
             within the analysis-mesh AABB",
        );
        for &v in shell {
            let rounded = v.round();
            assert!(
                (v - rounded).abs() < 1e-9,
                "categorical MC-vertex value {v} is fractional — C2 \
                 nearest-tet path should keep every value integer",
            );
            assert!(
                rounded == 3.0 || rounded == 7.0,
                "categorical MC-vertex value {rounded} is not one of \
                 the input integers {{3, 7}}",
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

    /// `design_scene` populates normals over the entire vertex range
    /// (body + contacts). The PLY writer enforces this invariant; the
    /// pre-helper-refactor code only stamped body normals.
    #[test]
    fn design_scene_normals_cover_body_and_contacts() {
        use crate::SphereSdf;

        struct OffsetSphere {
            cx: f64,
            r: f64,
        }
        impl Sdf for OffsetSphere {
            fn eval(&self, p: NaPoint3<f64>) -> f64 {
                let dx = p.x - self.cx;
                dx.mul_add(dx, p.y.mul_add(p.y, p.z * p.z)).sqrt() - self.r
            }
            fn grad(&self, p: NaPoint3<f64>) -> Vec3 {
                let dx = p.x - self.cx;
                let d = Vec3::new(dx, p.y, p.z);
                let n = d.norm();
                if n > 1e-12 { d / n } else { Vec3::z() }
            }
        }

        let body = SphereSdf { radius: 0.05 };
        let contact = OffsetSphere { cx: 0.07, r: 0.015 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let scene = design_scene(
            &body,
            &[&contact as &dyn Sdf],
            &analysis,
            &bounds,
            0.005,
            &scalars,
        )
        .unwrap();

        let normals = scene.normals.as_ref().expect("normals slot missing");
        assert_eq!(
            normals.len(),
            scene.geometry.vertices.len(),
            "normals must cover every vertex (body + contacts)"
        );
        for n in normals {
            let nsq = n.x.mul_add(n.x, n.y.mul_add(n.y, n.z * n.z));
            assert!((nsq - 1.0).abs() < 1e-6, "non-unit normal: {nsq}");
        }
    }

    /// `design_scene_deformed` with empty `contact_sdfs` produces output
    /// equal to `design_surface_deformed` on geometry + scalars, plus a
    /// `primitive_id` extra that's uniform `0.0` across the body.
    #[test]
    fn design_scene_deformed_empty_contacts_matches_deformed_surface() {
        use crate::SphereSdf;

        let sphere = SphereSdf { radius: 0.05 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let resolution = 0.005_f64;
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let n_v = analysis.n_vertices();
        let uniform = Vec3::new(0.001, 0.0, 0.0);
        let displacement = vec![uniform; n_v];
        let amplify = 5.0_f64;

        let body_only = design_surface_deformed(
            &sphere,
            &analysis,
            &bounds,
            resolution,
            &scalars,
            &displacement,
            amplify,
        )
        .unwrap();
        let scene = design_scene_deformed(
            &sphere,
            &[],
            &analysis,
            &bounds,
            resolution,
            &scalars,
            &displacement,
            amplify,
        )
        .unwrap();

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
            assert!(id.abs() < 1e-9, "expected body primitive_id 0.0, got {id}");
        }
    }

    /// `design_scene_deformed` with a contact primitive: body verts get
    /// `primitive_id = 0.0` and are displaced by `displacement * amplify`;
    /// contact verts get `primitive_id = 1.0` and are NOT displaced
    /// (contacts are rigid; their geometry comes from the contact SDF's
    /// rest pose, set per-call by the caller). Normals slot covers the
    /// full merged vertex range.
    #[test]
    fn design_scene_deformed_one_contact_separates_body_from_contact() {
        use crate::SphereSdf;

        struct OffsetSphere {
            cx: f64,
            r: f64,
        }
        impl Sdf for OffsetSphere {
            fn eval(&self, p: NaPoint3<f64>) -> f64 {
                let dx = p.x - self.cx;
                dx.mul_add(dx, p.y.mul_add(p.y, p.z * p.z)).sqrt() - self.r
            }
            fn grad(&self, p: NaPoint3<f64>) -> Vec3 {
                let dx = p.x - self.cx;
                let d = Vec3::new(dx, p.y, p.z);
                let n = d.norm();
                if n > 1e-12 { d / n } else { Vec3::z() }
            }
        }

        let body = SphereSdf { radius: 0.05 };
        let contact = OffsetSphere { cx: 0.07, r: 0.015 };
        let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
        let field = MaterialField::skeleton_default();
        let analysis = SingleTetMesh::new(&field);
        let psi = [1.0_f64];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("psi", &psi);

        let n_v = analysis.n_vertices();
        let displacement = vec![Vec3::zeros(); n_v];

        let scene = design_scene_deformed(
            &body,
            &[&contact as &dyn Sdf],
            &analysis,
            &bounds,
            0.005,
            &scalars,
            &displacement,
            1.0,
        )
        .unwrap();

        let primitive_id = scene
            .extras
            .get("primitive_id")
            .expect("primitive_id missing");
        let mut n_body = 0_usize;
        let mut n_contact = 0_usize;
        for &id in primitive_id {
            if id.abs() < 0.5 {
                n_body += 1;
            } else if (id - 1.0).abs() < 0.5 {
                n_contact += 1;
            } else {
                panic!("unexpected primitive_id {id}");
            }
        }
        assert!(n_body > 0, "body vertices missing");
        assert!(n_contact > 0, "contact vertices missing");

        let psi_extra = scene.extras.get("psi").expect("psi missing");
        assert_eq!(psi_extra.len(), scene.geometry.vertices.len());

        let normals = scene.normals.as_ref().expect("normals slot missing");
        assert_eq!(
            normals.len(),
            scene.geometry.vertices.len(),
            "normals must cover body + contact"
        );
    }

    // ---- C2.2 idw_k_nearest_tet_centroids unit tests ----

    /// Constant per-tet field → output equals that constant at any
    /// probe regardless of distance distribution. Trivially follows
    /// from `Σ w_i * c / Σ w_i = c`; pinned as a baseline so future
    /// kernel changes don't accidentally bias constant fields.
    #[test]
    fn idw_k_nearest_constant_field_returns_constant() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let positions = mesh.positions();
        let per_tet: [f64; 2] = [42.5, 42.5];
        let candidates: Vec<u32> = vec![0, 1];
        for probe in [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.05, 0.05, 0.05),
            Vec3::new(1.0, -2.0, 3.0),
        ] {
            let out = idw_k_nearest_tet_centroids(
                probe,
                &mesh,
                positions,
                &candidates,
                &[&per_tet],
                8,
                1e-12,
            );
            assert_eq!(out.len(), 1);
            assert!(
                (out[0] - 42.5).abs() < 1e-12,
                "constant field at probe {probe:?}: got {} expected 42.5",
                out[0],
            );
        }
    }

    /// Single candidate tet → output equals that tet's per-tet value
    /// regardless of probe-to-centroid distance. The IDW reduces to
    /// `(w * v) / w = v` when only one tet contributes.
    #[test]
    fn idw_k_nearest_single_candidate_returns_that_tet_value() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let positions = mesh.positions();
        let per_tet: [f64; 2] = [42.0, 99.0];
        // Probe arbitrary, well away from either centroid.
        let probe = Vec3::new(10.0, 10.0, 10.0);
        // Only tet 1 in the candidate set.
        let candidates: Vec<u32> = vec![1];
        let out = idw_k_nearest_tet_centroids(
            probe,
            &mesh,
            positions,
            &candidates,
            &[&per_tet],
            8,
            1e-12,
        );
        assert!(
            (out[0] - 99.0).abs() < 1e-12,
            "single-tet IDW must return that tet's value, got {}",
            out[0],
        );
    }

    /// Probe at the midpoint between two tet centroids with scalars
    /// `0.0` and `1.0` → IDW returns `0.5` (equal weights cancel).
    /// Pins the equal-distance branch of the Shepard kernel.
    #[test]
    fn idw_k_nearest_symmetric_two_candidates_returns_average() {
        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let positions = mesh.positions();
        // Tet 0 centroid = (0.025, 0.025, 0.025); tet 1 centroid =
        // mean of v1=(0.1,0,0), v2=(0,0.1,0), v3=(0,0,0.1),
        // v4=(0.08,0.08,0.08) = (0.045, 0.045, 0.045). Midpoint:
        // (0.035, 0.035, 0.035) is equidistant from both centroids.
        let probe = Vec3::new(0.035, 0.035, 0.035);
        let per_tet: [f64; 2] = [0.0, 1.0];
        let candidates: Vec<u32> = vec![0, 1];
        let out = idw_k_nearest_tet_centroids(
            probe,
            &mesh,
            positions,
            &candidates,
            &[&per_tet],
            8,
            1e-12,
        );
        assert!(
            (out[0] - 0.5).abs() < 1e-12,
            "equidistant probe IDW must average to 0.5, got {}",
            out[0],
        );
    }

    /// `design_slab_cut` C2.1 gap-fix: a categorical `_id` scalar on
    /// the design-SDF cross-section must produce integer-only output,
    /// matching the rest of the C2.1 design-X family. Pre-fix the
    /// vol-weighted-per-vertex + barycentric path would produce
    /// fractional values at any cross-vertex inside an analysis tet
    /// whose 4 vertices carry different integer-derived averages.
    #[test]
    fn design_slab_cut_categorical_id_stays_integer() {
        // Use the analysis-mesh boundary directly as the design SDF —
        // the cross-section at z=0.05 then samples shell_id from the
        // two analysis tets via nearest_tet_centroid_idx.
        struct MeshBoxSdf;
        impl crate::sdf_bridge::Sdf for MeshBoxSdf {
            fn eval(&self, p: nalgebra::Point3<f64>) -> f64 {
                let lo = nalgebra::Vector3::new(0.0, 0.0, 0.0);
                let hi = nalgebra::Vector3::new(0.1, 0.1, 0.1);
                let q = nalgebra::Vector3::new(
                    (lo.x - p.x).max(p.x - hi.x),
                    (lo.y - p.y).max(p.y - hi.y),
                    (lo.z - p.z).max(p.z - hi.z),
                );
                q.x.max(q.y).max(q.z)
            }
            fn grad(&self, _p: nalgebra::Point3<f64>) -> Vec3 {
                Vec3::z()
            }
        }
        let sdf = MeshBoxSdf;

        let field = MaterialField::skeleton_default();
        let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
        let shell_id: [f64; 2] = [3.0, 7.0];
        let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
        scalars.insert("material_shell_id", &shell_id);
        let bounds = Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.1, 0.1, 0.1));

        let attr = design_slab_cut(
            &sdf,
            &mesh,
            Plane {
                axis: 2,
                value: 0.05,
            },
            &bounds,
            0.01,
            &scalars,
        )
        .unwrap();
        let shell = attr
            .extras
            .get("material_shell_id")
            .expect("shell extra missing");
        assert!(
            !shell.is_empty(),
            "z=0.05 design slab cut must produce a non-empty cross-section",
        );
        for &v in shell {
            let rounded = v.round();
            assert!(
                (v - rounded).abs() < 1e-9,
                "categorical design-slab-cut value {v} is fractional — \
                 C2.1 gap-fix should keep every value integer",
            );
            assert!(
                rounded == 3.0 || rounded == 7.0,
                "categorical design-slab-cut value {rounded} is not one \
                 of the input integers {{3, 7}}",
            );
        }
    }
}
