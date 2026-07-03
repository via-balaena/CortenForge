//! Design-mesh primitives (F2): display geometry comes from the design SDF
//! (marching-squares slab cuts + marching-cubes surfaces/scenes); the analysis
//! mesh is consulted only for scalar values.

use std::collections::{BTreeMap, HashMap};

use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use nalgebra::Point3 as NaPoint3;

use crate::Vec3;
use crate::material::Material;
use crate::mesh::Mesh;
use crate::sdf_bridge::{Aabb3, Sdf};

use super::error::VizError;
use super::plane::Plane;
use super::scalar_transfer::{
    IDW_K, TetGrid, idw_k_nearest_tet_centroids, is_categorical_scalar_name,
    nearest_tet_centroid_idx, sample_analysis_at_point, sample_displacement_at_point,
    validate_per_tet_scalars, volume_weighted_per_vertex_avg,
};

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
/// # Comparison vs [`slab_cut`](super::slab_cut)
///
/// [`slab_cut`](super::slab_cut) intersects the analysis tet mesh directly and inherits
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
///    per-vertex projection [`slab_cut`](super::slab_cut) uses). Categorical scalars
///    (suffix `_id`): single nearest-tet-centroid via
///    `nearest_tet_centroid_idx` — preserves integer labels for the
///    Categorical colormap pick.
/// 4. Fall back to nearest-tet (clipped + renormalized barycentrics)
///    for vertices that fall just outside every analysis tet — common
///    where the design SDF's smooth surface lies outside the analysis
///    mesh's polyhedral envelope.
///
/// CCW (u, v) winding picks (uu, vv) such that uu × vv = +`plane.axis`,
/// matching [`slab_cut`](super::slab_cut)'s outward-winding convention so cf-view's
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
/// # Comparison vs [`boundary_surface`](super::boundary_surface)
///
/// [`boundary_surface`](super::boundary_surface) extracts the analysis tet mesh's precomputed
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
