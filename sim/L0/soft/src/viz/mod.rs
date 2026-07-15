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
//!   from [`Mesh::boundary_faces`](crate::mesh::Mesh::boundary_faces) and emits it as a triangulated 3D
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
//!   (rigid; no scalar field) into one [`AttributedMesh`](mesh_types::AttributedMesh) with a
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
//! All helpers consume any [`Mesh<M>`](crate::mesh::Mesh) sim-soft can produce and emit
//! [`AttributedMesh`](mesh_types::AttributedMesh) with multiple per-vertex scalars (each input
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
//! architecture was falsified across 8 fix attempts because it threw
//! away the tet connectivity and tried to re-derive geometry from
//! centroid clouds plus filters; each fix patched one geometry-specific
//! symptom and exposed another. The lesson: when a structured mesh
//! exists, its connectivity is the right primitive.
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

mod analysis;
mod design;
mod error;
mod plane;
mod scalar_transfer;

#[cfg(test)]
mod tests;

pub use analysis::{boundary_surface, slab_cut, slab_cut_deformed};
pub use design::{
    design_scene, design_scene_deformed, design_slab_cut, design_surface, design_surface_deformed,
};
pub use error::VizError;
pub use plane::Plane;
