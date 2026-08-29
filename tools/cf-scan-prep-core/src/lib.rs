//! `cf-scan-prep-core` — the headless, Bevy-free mesh-editing core for scan
//! preprocessing.
//!
//! This is the pure-compute half of the `cf-scan-prep` tool, extracted so a
//! second frontend (CortenForge Studio's Slint+wgpu scan editor) can drive
//! the exact same algorithms the Bevy tool does, with no behavior drift. The
//! Bevy tool keeps the ECS/egui/rendering shell and calls into here; Studio
//! calls into here too.
//!
//! Everything here operates on [`mesh_types::IndexedMesh`] + nalgebra types.
//! Nothing here depends on Bevy, egui, or any renderer.

mod cap;
mod centerline;
mod clip;
mod prep_toml;
mod reconstruct;
mod save;
mod simplify;
mod transform;
mod trim;

// The crate's public surface is FLAT — `cf-scan-prep` and Studio both consume
// it as `use cf_scan_prep_core::*`, so the split must not move a single name.
// Every item re-exports at the root, exactly where it was.
pub use cap::{
    DetectedCapLoop, MeshEdgeKey, PLANE_INTERSECTION_ON_PLANE_EPS_M, auto_cap_open_boundaries,
    build_detected_cap_loop, detect_boundary_loops, emit_centroid_fan_cap, fit_plane_to_points,
    floor_loop_index, intersect_plane_with_mesh, orient_cap_normal_outward, point_in_triangle_2d,
    polygon_area_3d, polygon_centroid_3d, project_loop_to_plane_2d, triangulate_polygon_2d_earclip,
};
pub use centerline::{
    CENTERLINE_MAIN_BODY_AREA_FRACTION, CENTERLINE_REORIENT_PASSES, MIN_SLAB_AREA_M2, SlabSample,
    build_polyline_with_boundary_trim, compute_centerline_polyline, compute_slab_sample,
    correct_tangents_for_end_regions, local_polyline_tangents, smooth_polyline,
};
pub use clip::{clip_mesh_against_plane, clip_mesh_against_plane_eq, lerp_point};
pub use prep_toml::{
    PrepAabbBlock, PrepCapLoop, PrepCapsBlock, PrepCenterlineBlock, PrepCenterlineTrimBlock,
    PrepOutputBlock, PrepReconstructSubBlock, PrepRotationBlock, PrepScanPrepBlock,
    PrepSimplifyBlock, PrepSmoothingBlock, PrepToml, PrepTransformBlock, PrepTranslationBlock,
    SIMPLIFY_ALGORITHM_NAME, SIMPLIFY_ALGORITHM_VERSION, SMOOTHING_ALGORITHM_NAME,
    build_prep_toml_string,
};
pub use reconstruct::{
    AppliedReconstruct, RECONSTRUCT_ANGLE_BINS, RECONSTRUCT_RING_COUNT, RECONSTRUCT_TAPER_AT_FLOOR,
    ReconstructShape, ReconstructedFloorPlane, STABLE_INWARD_TANGENT_LOOKBACK_M,
    apply_reconstruction, compute_reconstructed_floor_plane_physics, find_floor_loop_index,
    perpendicular_basis_for, sample_radial_profile, sample_radial_profile_linear_fit,
    sample_radius_at_angle, stable_inward_tangent,
};
pub use save::{
    atomic_write_save, build_cleaned_mesh, chrono_like_timestamp, iso8601_utc_from_unix_seconds,
    unix_days_to_ymd,
};
pub use simplify::{
    CLEANUP_DEGENERATE_AREA_M2, CLEANUP_MIN_COMPONENT_FACES, CleanupReport,
    SIMPLIFY_TARGET_DEFAULT, SIMPLIFY_TARGET_MAX, SIMPLIFY_TARGET_MIN, SIMPLIFY_WELD_EPSILON_M,
    SimplifyResult, cleanup_cleaned_mesh_for_disk, human_count, mesh_looks_unwelded, simplify_mesh,
};
pub use transform::{
    auto_center_in_place, auto_pca_in_place, bake_vertex_with_pivot, compute_pca_orientation,
    rotated_aabb_around_centroid_physics_mm, scale_vertices_in_place,
};
pub use trim::{
    point_along_polyline_at_arc_distance, polyline_arc_length_m, trim_centerline_polyline,
    trim_mesh_along_centerline,
};

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level for production
    // safety; allow them inside tests so assertions can pull values out of
    // `Option` / `Result` returns without multi-line `match` ceremony.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    // The flat `lib.rs` imported these for its production code; after the
    // module split the facade holds no code, so the test module carries the
    // foreign types it uses directly.
    use anyhow::Result;
    use mesh_repair::holes;
    use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
    use nalgebra::{UnitQuaternion, Vector3};

    fn one_triangle_at(scale: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(scale, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, scale, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, scale));
        mesh.faces.push([0, 1, 2]);
        mesh
    }
    /// Build a regular grid-subdivided square (in the XY plane) with
    /// `cells × cells` quads, each split into 2 triangles. Vertices are
    /// shared (proper indexed mesh; no STL-style 3N unsharing). Used
    /// for the simplify-on-known-shape test — small enough to be fast
    /// in CI while large enough that meshopt has something to collapse.
    fn grid_square(cells: usize) -> IndexedMesh {
        let mut mesh = IndexedMesh::with_capacity((cells + 1) * (cells + 1), cells * cells * 2);
        for j in 0..=cells {
            for i in 0..=cells {
                #[allow(clippy::cast_precision_loss)]
                let x = i as f64 / cells as f64;
                #[allow(clippy::cast_precision_loss)]
                let y = j as f64 / cells as f64;
                mesh.vertices.push(mesh_types::Point3::new(x, y, 0.0));
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let stride = (cells + 1) as u32;
        for j in 0..cells {
            for i in 0..cells {
                #[allow(clippy::cast_possible_truncation)]
                let row = j as u32 * stride;
                #[allow(clippy::cast_possible_truncation)]
                let next_row = (j as u32 + 1) * stride;
                #[allow(clippy::cast_possible_truncation)]
                let col = i as u32;
                let a = row + col;
                let b = row + col + 1;
                let c = next_row + col + 1;
                let d = next_row + col;
                mesh.faces.push([a, b, c]);
                mesh.faces.push([a, c, d]);
            }
        }
        mesh
    }
    /// Build a unit-cube indexed mesh (corners at 0/1 on each axis,
    /// 8 verts + 12 triangles, CCW-outward winding). Hand-crafted
    /// fixture used by the [`intersect_plane_with_mesh`] tests; the
    /// general-purpose body-shape fixture helpers live alongside the
    /// algorithm tests further down.
    fn make_unit_cube_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::with_capacity(8, 12);
        // Corner ordering: bit 0 = x, bit 1 = y, bit 2 = z.
        for i in 0..8 {
            mesh.vertices.push(Point3::new(
                f64::from(i & 1),
                f64::from((i >> 1) & 1),
                f64::from((i >> 2) & 1),
            ));
        }
        // 6 faces × 2 triangles, CCW from outside.
        let faces: [[u32; 3]; 12] = [
            // -Z (bottom)
            [0, 2, 1],
            [1, 2, 3],
            // +Z (top)
            [4, 5, 6],
            [5, 7, 6],
            // -Y (front)
            [0, 1, 4],
            [1, 5, 4],
            // +Y (back)
            [2, 6, 3],
            [3, 6, 7],
            // -X (left)
            [0, 4, 2],
            [2, 4, 6],
            // +X (right)
            [1, 3, 5],
            [3, 7, 5],
        ];
        for f in faces {
            mesh.faces.push(f);
        }
        mesh
    }
    /// Build a closed Z-axis frustum (cylinder if `radius_base ==
    /// radius_tip`; cone if `radius_tip == 0`) with `n_rings` axial
    /// rings × `n_segs` angular segments per ring, plus triangle-fan
    /// caps at z = ±height/2. Watertight; CCW-outward winding;
    /// centered on z-axis.
    ///
    /// `apply_noise(ring_idx, seg_idx, theta) -> f64` is added to
    /// the ring's interpolated radius per vertex. Use a closure
    /// returning `0.0` for a clean surface.
    fn make_closed_frustum_mesh(
        n_rings: usize,
        n_segs: usize,
        radius_base: f64,
        radius_tip: f64,
        height: f64,
        apply_noise: impl Fn(usize, usize, f64) -> f64,
    ) -> IndexedMesh {
        assert!(
            n_rings >= 2 && n_segs >= 3,
            "frustum needs >= 2 rings, >= 3 segs"
        );
        let vert_count = n_rings * n_segs + 2;
        let face_count = 2 * (n_rings - 1) * n_segs + 2 * n_segs;
        let mut mesh = IndexedMesh::with_capacity(vert_count, face_count);

        // Side-wall vertices: index = ring * n_segs + seg.
        for r in 0..n_rings {
            #[allow(clippy::cast_precision_loss)]
            let t = (r as f64) / ((n_rings - 1) as f64);
            let z = -height / 2.0 + t * height;
            let ring_radius = radius_base * (1.0 - t) + radius_tip * t;
            for k in 0..n_segs {
                #[allow(clippy::cast_precision_loss)]
                let theta = (k as f64) * std::f64::consts::TAU / (n_segs as f64);
                let radius = ring_radius + apply_noise(r, k, theta);
                mesh.vertices
                    .push(Point3::new(radius * theta.cos(), radius * theta.sin(), z));
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let bottom_center_idx = (n_rings * n_segs) as u32;
        let top_center_idx = bottom_center_idx + 1;
        mesh.vertices.push(Point3::new(0.0, 0.0, -height / 2.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, height / 2.0));

        // Side-wall: 2 CCW-outward triangles per quad.
        for r in 0..(n_rings - 1) {
            for k in 0..n_segs {
                #[allow(clippy::cast_possible_truncation)]
                let i00 = (r * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i01 = (r * n_segs + (k + 1) % n_segs) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i10 = ((r + 1) * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i11 = ((r + 1) * n_segs + (k + 1) % n_segs) as u32;
                mesh.faces.push([i00, i01, i11]);
                mesh.faces.push([i00, i11, i10]);
            }
        }
        // Bottom cap (normal -Z): CW when viewed from +Z above.
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = k as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = ((k + 1) % n_segs) as u32;
            mesh.faces.push([bottom_center_idx, i1, i0]);
        }
        // Top cap (normal +Z): CCW when viewed from +Z above.
        let top_ring_base = (n_rings - 1) * n_segs;
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = (top_ring_base + k) as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = (top_ring_base + (k + 1) % n_segs) as u32;
            mesh.faces.push([top_center_idx, i0, i1]);
        }
        mesh
    }
    /// Like [`make_closed_frustum_mesh`] but with **non-uniform
    /// angular sampling** per ring — vertices are placed at the
    /// user-provided `thetas` (in `[0, 2π)`, monotonic; same set
    /// repeated for every ring). Pure cylinder shape (`radius_base
    /// == radius_tip`), no noise.
    ///
    /// Used to verify density-independence: with `thetas` densely
    /// sampled on one side of the circle and sparsely on the other,
    /// the BOUNDARY shape is still the same circle but the vertex
    /// density is asymmetric — the polygon centroid is invariant
    /// (regression test for the iter-1 failure mode), while the
    /// prior vertex-centroid statistic would have been biased
    /// toward the dense side.
    fn make_density_biased_cylinder_mesh(
        n_rings: usize,
        thetas: &[f64],
        radius: f64,
        height: f64,
    ) -> IndexedMesh {
        let n_segs = thetas.len();
        assert!(
            n_rings >= 2 && n_segs >= 3,
            "density-biased cylinder needs >= 2 rings, >= 3 thetas"
        );
        let vert_count = n_rings * n_segs + 2;
        let face_count = 2 * (n_rings - 1) * n_segs + 2 * n_segs;
        let mut mesh = IndexedMesh::with_capacity(vert_count, face_count);

        for r in 0..n_rings {
            #[allow(clippy::cast_precision_loss)]
            let t = (r as f64) / ((n_rings - 1) as f64);
            let z = -height / 2.0 + t * height;
            for &theta in thetas {
                mesh.vertices
                    .push(Point3::new(radius * theta.cos(), radius * theta.sin(), z));
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let bottom_center_idx = (n_rings * n_segs) as u32;
        let top_center_idx = bottom_center_idx + 1;
        mesh.vertices.push(Point3::new(0.0, 0.0, -height / 2.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, height / 2.0));

        for r in 0..(n_rings - 1) {
            for k in 0..n_segs {
                #[allow(clippy::cast_possible_truncation)]
                let i00 = (r * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i01 = (r * n_segs + (k + 1) % n_segs) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i10 = ((r + 1) * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i11 = ((r + 1) * n_segs + (k + 1) % n_segs) as u32;
                mesh.faces.push([i00, i01, i11]);
                mesh.faces.push([i00, i11, i10]);
            }
        }
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = k as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = ((k + 1) % n_segs) as u32;
            mesh.faces.push([bottom_center_idx, i1, i0]);
        }
        let top_ring_base = (n_rings - 1) * n_segs;
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = (top_ring_base + k) as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = (top_ring_base + (k + 1) % n_segs) as u32;
            mesh.faces.push([top_center_idx, i0, i1]);
        }
        mesh
    }
    /// Translate every vertex of a mesh by `offset` (utility for
    /// building off-axis fixtures — apply to a centered fixture to
    /// shift the whole body laterally).
    fn translate_mesh(mesh: &mut IndexedMesh, offset: Vector3<f64>) {
        for v in &mut mesh.vertices {
            v.coords += offset;
        }
    }
    /// Rotate every vertex of a mesh by `rotation` around origin
    /// (utility for building rotated-axis fixtures — apply to a
    /// Z-axis centered fixture to test XYZ-independence with a
    /// non-Z body axis).
    fn rotate_mesh(mesh: &mut IndexedMesh, rotation: UnitQuaternion<f64>) {
        for v in &mut mesh.vertices {
            v.coords = rotation * v.coords;
        }
    }
    /// A unit square at the origin (CCW in XY). 4 vertices, 2-triangle
    /// triangulation expected from `triangulate_polygon_2d_earclip`.
    fn unit_square_2d() -> Vec<(f64, f64)> {
        vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    }
    /// Build the STL-style (per-triangle unshared) version of an
    /// IndexedMesh — `mesh_io::load_stl` produces this layout because
    /// binary STL stores each triangle's 3 vertices independently
    /// without index sharing. cf-scan-prep's hygiene pass is supposed
    /// to weld these back to the shared layout downstream
    /// `simplify_decoder` requires. Test helper, not production code.
    fn unshare_vertices(mesh: &IndexedMesh) -> IndexedMesh {
        let n_faces = mesh.faces.len();
        let mut out = IndexedMesh::with_capacity(n_faces * 3, n_faces);
        #[allow(clippy::cast_possible_truncation)]
        for face in &mesh.faces {
            let base = out.vertices.len() as u32;
            for &idx in face {
                out.vertices.push(mesh.vertices[idx as usize]);
            }
            out.faces.push([base, base + 1, base + 2]);
        }
        out
    }

    // ----- scale_vertices_in_place -----------------------------------
    /// `factor = 1.0` is a no-op fast path — load with `--stl-units m`
    /// must leave vertices bit-identical (avoids subtle FP drift on
    /// re-saved meter-scale STLs).
    #[test]
    fn scale_vertices_identity_is_bit_exact_no_op() {
        let mut mesh = one_triangle_at(1.234_567_891_234_567);
        let original = mesh.vertices.clone();
        scale_vertices_in_place(&mut mesh, 1.0);
        assert_eq!(mesh.vertices, original);
    }
    /// `factor = 0.001` (the `mm → m` case) walks every vertex and
    /// scales each component. Asserts on the three components of the
    /// scaled triangle independently to catch axis-confusion bugs.
    #[test]
    fn scale_vertices_mm_to_meters_scales_each_component() {
        let mut mesh = one_triangle_at(80.0); // 80 mm
        scale_vertices_in_place(&mut mesh, 0.001);
        assert!((mesh.vertices[0].x - 0.080).abs() < 1e-12);
        assert!((mesh.vertices[1].y - 0.080).abs() < 1e-12);
        assert!((mesh.vertices[2].z - 0.080).abs() < 1e-12);
    }

    // ----- auto_center_in_place (CSP.3.5) ----------------------------
    /// `auto_center_in_place` moves a scan offset far from the
    /// physics origin to a centroid-at-origin position, and reports
    /// the offset it applied. Simulates the "scanner pointed down,
    /// captured geometry at negative z" workflow the CSP.3.5
    /// followup addresses.
    #[test]
    fn auto_center_in_place_moves_centroid_to_origin() {
        // A unit-cube-like mesh sitting at physics centroid
        // (0.05, 0.00, -0.170) — far from origin, like a scanner
        // dropped it there.
        let mut mesh = IndexedMesh::with_capacity(8, 0);
        for &(x, y, z) in &[
            (0.05 - 0.04, 0.00 - 0.03, -0.170 - 0.06),
            (0.05 + 0.04, 0.00 - 0.03, -0.170 - 0.06),
            (0.05 + 0.04, 0.00 + 0.03, -0.170 - 0.06),
            (0.05 - 0.04, 0.00 + 0.03, -0.170 - 0.06),
            (0.05 - 0.04, 0.00 - 0.03, -0.170 + 0.06),
            (0.05 + 0.04, 0.00 - 0.03, -0.170 + 0.06),
            (0.05 + 0.04, 0.00 + 0.03, -0.170 + 0.06),
            (0.05 - 0.04, 0.00 + 0.03, -0.170 + 0.06),
        ] {
            mesh.vertices.push(Point3::new(x, y, z));
        }
        let offset = auto_center_in_place(&mut mesh);
        // Offset = -centroid → expect (-0.05, 0, +0.170).
        assert!((offset.x - (-0.05)).abs() < 1e-12);
        assert!(offset.y.abs() < 1e-12);
        assert!((offset.z - 0.170).abs() < 1e-12);
        // Post-centering AABB centroid should be ~origin.
        let post_centroid = mesh.aabb().center();
        assert!(post_centroid.coords.norm() < 1e-12);
    }
    /// `auto_center_in_place` is a no-op (returns zero offset) for
    /// a mesh whose centroid is already within 1 µm of the origin.
    /// Guards against FP-drift on already-centered fixtures. Uses a
    /// symmetric ±1 mm cube — its AABB centroid lands at bit-exact
    /// origin, so the threshold guard fires.
    #[test]
    fn auto_center_in_place_no_op_when_already_centered() {
        let mut mesh = IndexedMesh::with_capacity(2, 0);
        mesh.vertices.push(Point3::new(-0.001, -0.001, -0.001));
        mesh.vertices.push(Point3::new(0.001, 0.001, 0.001));
        let mesh_before = mesh.clone();
        let offset = auto_center_in_place(&mut mesh);
        assert!(offset.norm() < 1e-12, "near-zero offset, got {offset:?}");
        // Vertices unchanged.
        assert_eq!(mesh.vertices, mesh_before.vertices);
    }
    /// `auto_center_in_place` is a no-op on empty meshes (defensive
    /// against the load-failure overlay path — never hit, but cheap
    /// to guarantee).
    #[test]
    fn auto_center_in_place_handles_empty_mesh() {
        let mut mesh = IndexedMesh::with_capacity(0, 0);
        let offset = auto_center_in_place(&mut mesh);
        assert_eq!(offset, Vector3::zeros());
        assert!(mesh.vertices.is_empty());
    }

    // ----- auto_pca_in_place (CSP.4a) --------------------------------
    /// Mesh elongated along physics +X: after auto-PCA-in-place, the
    /// long axis aligns with +Z (the principal-axis vertices' X
    /// components rotate into Z). Confirms the bake-in-place
    /// behavior matches the rotation `compute_pca_orientation`
    /// returns.
    #[test]
    fn auto_pca_in_place_rotates_long_x_axis_to_plus_z() {
        // 11 points along +X (centroid at origin), no spread on Y/Z.
        let mut mesh = IndexedMesh::with_capacity(11, 0);
        for i in -5..=5 {
            mesh.vertices
                .push(Point3::new(f64::from(i) * 0.01, 0.0, 0.0));
        }
        let pre_x_range_max = mesh
            .vertices
            .iter()
            .map(|v| v.x.abs())
            .fold(0.0_f64, f64::max);
        assert!((pre_x_range_max - 0.05).abs() < 1e-12);

        let q = auto_pca_in_place(&mut mesh).expect("PCA should succeed");

        // After rotation, the long axis runs along +Z (or -Z; the
        // sign-pick keeps the rotation short). The X spread
        // collapses to ~zero; the Z spread inherits the original X
        // spread.
        let post_z_range_max = mesh
            .vertices
            .iter()
            .map(|v| v.z.abs())
            .fold(0.0_f64, f64::max);
        let post_x_range_max = mesh
            .vertices
            .iter()
            .map(|v| v.x.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            (post_z_range_max - 0.05).abs() < 1e-9,
            "post Z extent: {post_z_range_max}"
        );
        assert!(
            post_x_range_max < 1e-9,
            "post X extent should collapse: {post_x_range_max}"
        );

        // Quaternion identity-check: not identity (we actually rotated).
        let vec_sq = q.i * q.i + q.j * q.j + q.k * q.k;
        assert!(vec_sq > 1e-6, "expected non-identity rotation");
    }
    /// Already-PCA-aligned mesh (principal axis = +Z, vertices in
    /// the YZ plane with the long axis on Z): auto-PCA is the
    /// identity, vertex positions stay bit-exact.
    #[test]
    fn auto_pca_in_place_skips_vertex_walk_on_near_identity() {
        // 11 points along +Z, centroid at origin. PCA principal axis
        // = +Z already; rotation_between(+Z, +Z) = identity.
        let mut mesh = IndexedMesh::with_capacity(11, 0);
        for i in -5..=5 {
            mesh.vertices
                .push(Point3::new(0.0, 0.0, f64::from(i) * 0.01));
        }
        let mesh_before = mesh.clone();

        let q = auto_pca_in_place(&mut mesh).expect("PCA should succeed");
        // Quaternion should be near-identity.
        assert!(
            (q.w - 1.0).abs() < 1e-9,
            "expected near-identity q.w, got {}",
            q.w
        );
        // Vertices bit-exactly preserved (the near-identity skip
        // path didn't walk them).
        assert_eq!(mesh.vertices, mesh_before.vertices);
    }
    /// Degenerate input (all vertices coincident → no principal
    /// axis): auto-PCA returns `None`, vertices unchanged. Pairs
    /// with `compute_pca_orientation`'s `None` path.
    #[test]
    fn auto_pca_in_place_returns_none_on_degenerate() {
        let mut mesh = IndexedMesh::with_capacity(10, 0);
        for _ in 0..10 {
            mesh.vertices.push(Point3::origin());
        }
        let mesh_before = mesh.clone();
        let q = auto_pca_in_place(&mut mesh);
        assert!(q.is_none(), "expected None on degenerate input");
        assert_eq!(mesh.vertices, mesh_before.vertices);
    }

    // ----- human_count ------------------------------------------------
    /// Small counts pass through as bare integers (no suffix). The
    /// `1`/`999` boundary cases pin the `< 1000` threshold.
    #[test]
    fn human_count_small_returns_bare_integer() {
        assert_eq!(human_count(0), "0");
        assert_eq!(human_count(1), "1");
        assert_eq!(human_count(999), "999");
    }
    /// Thousands use `k` suffix with one decimal. `18_432` matches the
    /// spec mockup's `"18.4k"` example exactly.
    #[test]
    fn human_count_thousands_use_k_suffix() {
        assert_eq!(human_count(1_000), "1.0k");
        assert_eq!(human_count(18_432), "18.4k");
    }
    /// Millions use `M` suffix with two decimals. `3_352_068` matches
    /// the iter-1 fixture's face count (`sock_over_capsule.stl`, 3.35M
    /// faces) — banked in MEMORY.md's Resume-here block as the spec's
    /// canonical perf-calibration value.
    #[test]
    fn human_count_millions_use_m_suffix() {
        assert_eq!(human_count(1_000_000), "1.00M");
        assert_eq!(human_count(3_352_068), "3.35M");
    }

    // ----- SIMPLIFY_TARGET_MIN / MAX ---------------------------------
    /// Slider bounds spec-pin (1k–1M) per spec §Panel specifications §2.
    /// Changing either bound silently shifts the slider's logarithmic
    /// midpoint (where the 200k default sits in the track), so worth
    /// a regression test.
    #[test]
    fn simplify_slider_bounds_match_spec() {
        assert_eq!(SIMPLIFY_TARGET_MIN, 1_000);
        assert_eq!(SIMPLIFY_TARGET_MAX, 1_000_000);
    }

    // ----- simplify_mesh end-to-end (small fixture) ------------------
    #[test]
    fn mesh_looks_unwelded_flags_raw_stl_soup() {
        // Raw STL load: 3 verts per triangle → 3× faces. Flagged.
        assert!(mesh_looks_unwelded(600_000, 200_000));
        // Welded mesh: V ≈ F/2. Not flagged.
        assert!(!mesh_looks_unwelded(100_000, 200_000));
        // Exactly at the 2× threshold → flagged.
        assert!(mesh_looks_unwelded(400_000, 200_000));
        // Empty mesh → never flagged (avoids div-by-zero / false alarm).
        assert!(!mesh_looks_unwelded(0, 0));
    }
    /// `simplify_mesh` on a 20×20-cell grid (800 faces) targeting 100
    /// faces returns a mesh with face count ≤ target. meshopt doesn't
    /// guarantee hitting the target exactly (topology may force fewer
    /// collapses), so the assertion is upper-bounded.
    #[test]
    fn simplify_mesh_reduces_face_count_within_target() {
        let original = grid_square(20);
        assert_eq!(original.faces.len(), 800);

        let result = simplify_mesh(&original, 100);
        assert!(
            result.mesh.faces.len() <= 100,
            "simplified face count {} should not exceed target 100",
            result.mesh.faces.len(),
        );
        // Strictly smaller than original (some progress was made).
        assert!(
            result.mesh.faces.len() < original.faces.len(),
            "simplify must reduce face count (got {} from original {})",
            result.mesh.faces.len(),
            original.faces.len(),
        );
        // Elapsed time is finite + non-negative (sanity on the timer).
        assert!(result.elapsed_secs >= 0.0);
        assert!(result.elapsed_secs.is_finite());
    }
    /// Regression for the iter-1-eyes-on-pixels crash: meshopt's C++
    /// side asserts `target_index_count <= index_count` in
    /// `meshopt_simplifyEdge` and SIGABRTs the process if violated.
    /// `simplify_mesh` defensively early-returns the input unchanged
    /// when `target_face_count >= original.faces.len()` so the FFI is
    /// never invoked with assertion-violating params.
    ///
    /// Covers the "drag slider right past current face count + Apply"
    /// workflow (e.g., after a prior Apply reduces to 100 faces, then
    /// the user drags the slider to 200 and clicks Apply again). The
    /// `handle_simplify_actions` system pre-checks this case and
    /// surfaces a user-facing status message, but the guard inside
    /// `simplify_mesh` itself is the bulletproof layer.
    #[test]
    fn simplify_mesh_above_input_face_count_is_no_op() {
        let original = grid_square(20);
        assert_eq!(original.faces.len(), 800);

        // Target >> input — would SIGABRT without the guard.
        let result = simplify_mesh(&original, 10_000);

        assert_eq!(
            result.mesh.faces.len(),
            original.faces.len(),
            "guard must return input unchanged when target >= current",
        );
        assert_eq!(result.mesh.vertices.len(), original.vertices.len());

        // Exactly-equal target also exits the guard (>=, not >).
        let result_eq = simplify_mesh(&original, 800);
        assert_eq!(result_eq.mesh.faces.len(), 800);
    }

    // ----- compute_pca_orientation ----------------------------------
    /// Empty vertex slice returns `None` — PCA needs at least one
    /// point to define a centroid, three to define a principal axis.
    /// Pinned so a misuse at the call site surfaces visibly rather
    /// than panicking inside `symmetric_eigen` on a zero matrix.
    #[test]
    fn pca_orientation_too_few_vertices_returns_none() {
        let v: Vec<Point3<f64>> = vec![];
        assert!(compute_pca_orientation(&v).is_none());
        let v = vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)];
        assert!(compute_pca_orientation(&v).is_none());
    }
    /// Coincident vertices have zero covariance → no principal axis.
    /// Returns `None` rather than producing an arbitrary rotation
    /// from numerical noise.
    #[test]
    fn pca_orientation_degenerate_coincident_returns_none() {
        let v = vec![Point3::origin(); 10];
        assert!(compute_pca_orientation(&v).is_none());
    }
    /// Vertices stretched along physics-frame `+X` produce a rotation
    /// that maps `+X` to `+Z`. The shortest such rotation is `-90°`
    /// about `+Y`; via `from_axis_angle` that's quaternion
    /// `(cos(-45°), 0, sin(-45°), 0)`.
    #[test]
    fn pca_orientation_long_x_axis_rotates_x_to_z() {
        // 11 points along +X with no spread on Y or Z. Length 1.0 m
        // along X, zero variance on Y / Z → principal axis = +X
        // (or -X; sign-pick flips to +X for positive z-target).
        let v: Vec<Point3<f64>> = (0..11)
            .map(|i| Point3::new(f64::from(i) * 0.1, 0.0, 0.0))
            .collect();
        let q = compute_pca_orientation(&v).unwrap();

        // Apply the rotation to +X — should land at +Z.
        let rotated = q.transform_vector(&Vector3::x());
        assert!((rotated.x).abs() < 1e-9, "rotated.x = {}", rotated.x);
        assert!((rotated.y).abs() < 1e-9, "rotated.y = {}", rotated.y);
        assert!((rotated.z - 1.0).abs() < 1e-9, "rotated.z = {}", rotated.z);
    }
    /// Vertices already long along `+Z` produce the identity
    /// rotation — no work needed. Pins the "no-op fast path" against
    /// silently producing a small rotation from FP noise.
    #[test]
    fn pca_orientation_long_z_axis_yields_identity() {
        let v: Vec<Point3<f64>> = (0..11)
            .map(|i| Point3::new(0.0, 0.0, f64::from(i) * 0.1))
            .collect();
        let q = compute_pca_orientation(&v).unwrap();
        assert!((q.w - 1.0).abs() < 1e-9, "q.w = {}", q.w);
        assert!(q.i.abs() < 1e-9, "q.i = {}", q.i);
        assert!(q.j.abs() < 1e-9, "q.j = {}", q.j);
        assert!(q.k.abs() < 1e-9, "q.k = {}", q.k);
    }
    /// Sign-pick check: the principal eigenvector for vertices
    /// stretched along `-Z` could come out as `+Z` or `-Z`; the
    /// helper flips to `+Z` (the side closer to the `+Z` target) so
    /// the resulting rotation is identity rather than a 180° flip.
    /// Sigil for "no rotation needed when the long axis ALREADY
    /// points up, even if the eigenvector signs out the other way."
    #[test]
    fn pca_orientation_sign_pick_prefers_positive_z() {
        // 11 points along ±Z spanning 1 m, symmetric about origin.
        // Centroid = origin; principal eigenvector axis = ±Z line.
        let v: Vec<Point3<f64>> = (0..11)
            .map(|i| Point3::new(0.0, 0.0, (f64::from(i) - 5.0) * 0.1))
            .collect();
        let q = compute_pca_orientation(&v).unwrap();
        // Identity — sign-pick collapsed both ±Z choices onto +Z.
        assert!((q.w - 1.0).abs() < 1e-9, "q.w = {}", q.w);
    }
    /// Cuboid mass distribution with the long axis along physics
    /// `+Y` produces a rotation that maps `+Y` to `+Z` — i.e. the
    /// 90°-about-X rotation behind the existing `[Snap +Y -> +Z]`
    /// button. Sanity check that PCA picks up dominant variance
    /// along a non-axis-of-largest-index direction.
    #[test]
    fn pca_orientation_long_y_axis_rotates_y_to_z() {
        // Cuboid: 0.05 m × 1.0 m × 0.05 m (long along +Y). Use 9
        // points covering corners + center to mimic mass spread.
        let v = vec![
            Point3::new(-0.025, -0.5, -0.025),
            Point3::new(0.025, -0.5, -0.025),
            Point3::new(-0.025, 0.5, -0.025),
            Point3::new(0.025, 0.5, -0.025),
            Point3::new(-0.025, -0.5, 0.025),
            Point3::new(0.025, -0.5, 0.025),
            Point3::new(-0.025, 0.5, 0.025),
            Point3::new(0.025, 0.5, 0.025),
            Point3::origin(),
        ];
        let q = compute_pca_orientation(&v).unwrap();
        let rotated = q.transform_vector(&Vector3::y());
        assert!((rotated.x).abs() < 1e-9, "rotated.x = {}", rotated.x);
        assert!((rotated.y).abs() < 1e-9, "rotated.y = {}", rotated.y);
        assert!((rotated.z - 1.0).abs() < 1e-9, "rotated.z = {}", rotated.z);
    }

    // ----- rotated_aabb_around_centroid_physics_mm -------------------
    /// Identity rotation: `rotated_aabb_around_centroid_physics_mm`
    /// returns the raw AABB scaled to mm with no axis permutation.
    #[test]
    fn rotated_aabb_identity_preserves_bounds_in_mm() {
        let raw = Aabb::from_corners(
            mesh_types::Point3::new(-0.05, -0.10, -0.15),
            mesh_types::Point3::new(0.05, 0.10, 0.15),
        );
        let (min_mm, max_mm) =
            rotated_aabb_around_centroid_physics_mm(UnitQuaternion::identity(), &raw);

        assert!((min_mm.x + 50.0).abs() < 1e-9);
        assert!((min_mm.y + 100.0).abs() < 1e-9);
        assert!((min_mm.z + 150.0).abs() < 1e-9);
        assert!((max_mm.x - 50.0).abs() < 1e-9);
        assert!((max_mm.y - 100.0).abs() < 1e-9);
        assert!((max_mm.z - 150.0).abs() < 1e-9);
    }
    /// 90° rotation about physics X axis: physics +Y becomes physics
    /// +Z; physics +Z becomes physics -Y. The rotated bbox's
    /// Y-extent equals the original Z-extent and vice-versa (signs
    /// shift accordingly). Test load: `(±0.05, ±0.10, ±0.15)` (mm:
    /// `±50, ±100, ±150`) rotated `+90°` about X:
    ///
    /// - X unchanged → `±50` mm.
    /// - Original Y `±100` becomes... actually the +90°-about-X
    ///   rotation maps `(0, 1, 0) -> (0, 0, 1)` and
    ///   `(0, 0, 1) -> (0, -1, 0)`. So the rotated bbox's Y range
    ///   is `±150` (= original Z range), and the rotated Z range is
    ///   `±100` (= original Y range).
    #[test]
    fn rotated_aabb_90deg_about_x_swaps_y_z_extents() {
        let raw = Aabb::from_corners(
            mesh_types::Point3::new(-0.05, -0.10, -0.15),
            mesh_types::Point3::new(0.05, 0.10, 0.15),
        );
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::x_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let (min_mm, max_mm) = rotated_aabb_around_centroid_physics_mm(rot, &raw);

        // X unchanged.
        assert!((min_mm.x + 50.0).abs() < 1e-6);
        assert!((max_mm.x - 50.0).abs() < 1e-6);
        // Y now spans `±150` (was Z).
        assert!((min_mm.y + 150.0).abs() < 1e-6);
        assert!((max_mm.y - 150.0).abs() < 1e-6);
        // Z now spans `±100` (was Y, sign-flipped by the +90° rotation
        // but symmetric bbox -> same bounds).
        assert!((min_mm.z + 100.0).abs() < 1e-6);
        assert!((max_mm.z - 100.0).abs() < 1e-6);
    }
    /// Off-center AABB under centroid-pivot rotation: the AABB CENTER
    /// stays at the centroid (in mm) regardless of rotation; only the
    /// extents rotate. Test load: AABB from `(0.10, -0.05, 0.20)` to
    /// `(0.20, 0.05, 0.40)` — centroid at `(0.15, 0.0, 0.30)`. Under
    /// `+90°` about Y the half-extents `(50, 50, 100)` mm in
    /// `(x, y, z)` rotate to `(100, 50, 50)` mm. After translating the
    /// rotated centered AABB back to centroid (in mm `(150, 0, 300)`),
    /// the world AABB is centroid ± rotated_half_extents.
    ///
    /// **This test is THE pivot-vs-origin discriminator** — under the
    /// old origin-pivot helper, the rotated AABB would be far from the
    /// raw bbox; here we pin it to stay locked at the centroid.
    #[test]
    fn rotated_aabb_off_center_pivots_around_centroid_not_origin() {
        let raw = Aabb::from_corners(
            mesh_types::Point3::new(0.10, -0.05, 0.20),
            mesh_types::Point3::new(0.20, 0.05, 0.40),
        );
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let (min_mm, max_mm) = rotated_aabb_around_centroid_physics_mm(rot, &raw);

        // Centroid in mm: (150, 0, 300).
        // Half-extents before rotation (mm): (50, 50, 100) in (x,y,z).
        // +90° about Y sends original X-half-extent (50) to Z and
        // original Z-half-extent (100) to -X (-100 → magnitude 100 in
        // X). So rotated half-extents: (100, 50, 50) in (x,y,z).
        // World AABB: centroid ± rotated_half_extents.
        assert!(
            (min_mm.x - (150.0 - 100.0)).abs() < 1e-6,
            "min.x = {}",
            min_mm.x
        );
        assert!(
            (max_mm.x - (150.0 + 100.0)).abs() < 1e-6,
            "max.x = {}",
            max_mm.x
        );
        assert!(
            (min_mm.y - (0.0 - 50.0)).abs() < 1e-6,
            "min.y = {}",
            min_mm.y
        );
        assert!(
            (max_mm.y - (0.0 + 50.0)).abs() < 1e-6,
            "max.y = {}",
            max_mm.y
        );
        assert!(
            (min_mm.z - (300.0 - 50.0)).abs() < 1e-6,
            "min.z = {}",
            min_mm.z
        );
        assert!(
            (max_mm.z - (300.0 + 50.0)).abs() < 1e-6,
            "max.z = {}",
            max_mm.z
        );
        // Center of the rotated world AABB == centroid in mm.
        let center_x = 0.5 * (min_mm.x + max_mm.x);
        let center_y = 0.5 * (min_mm.y + max_mm.y);
        let center_z = 0.5 * (min_mm.z + max_mm.z);
        assert!((center_x - 150.0).abs() < 1e-6);
        assert!(center_y.abs() < 1e-6);
        assert!((center_z - 300.0).abs() < 1e-6);
    }

    // ----- bake_vertex_with_pivot -----------------------------------
    /// Identity rotation + zero translation: bake is the identity
    /// transform (returns the input point unchanged regardless of
    /// centroid). Confirms the formula `R*(v-c)+c+t` collapses to `v`
    /// at the trivial case.
    #[test]
    fn bake_vertex_with_pivot_identity_is_passthrough() {
        let v = Point3::new(0.5, -0.3, 1.2);
        let c = Point3::new(0.1, 0.2, 0.3);
        let baked = bake_vertex_with_pivot(&v, UnitQuaternion::identity(), &c, Vector3::zeros());
        assert!((baked.x - v.x).abs() < 1e-12);
        assert!((baked.y - v.y).abs() < 1e-12);
        assert!((baked.z - v.z).abs() < 1e-12);
    }
    /// Centroid itself is fixed under any pivot rotation (rotation
    /// around centroid leaves the centroid in place by definition).
    /// Translation shifts it linearly.
    #[test]
    fn bake_vertex_with_pivot_centroid_is_fixed_point() {
        let c = Point3::new(0.15, 0.0, 0.30);
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let t = Vector3::new(0.05, -0.10, 0.20);
        let baked = bake_vertex_with_pivot(&c, rot, &c, t);
        // baked = R*(c-c)+c+t = c+t
        assert!((baked.x - (c.x + t.x)).abs() < 1e-12);
        assert!((baked.y - (c.y + t.y)).abs() < 1e-12);
        assert!((baked.z - (c.z + t.z)).abs() < 1e-12);
    }
    /// 180° rotation around an off-origin centroid reflects every
    /// point through the centroid. Sigil: an off-origin vertex
    /// reflected through a non-origin pivot does NOT land at the
    /// origin-reflected position — the pivot matters.
    #[test]
    fn bake_vertex_with_pivot_180_reflects_through_centroid() {
        let v = Point3::new(0.20, 0.0, 0.30);
        let c = Point3::new(0.15, 0.0, 0.30);
        let rot =
            UnitQuaternion::from_axis_angle(&nalgebra::Vector3::y_axis(), std::f64::consts::PI);
        let baked = bake_vertex_with_pivot(&v, rot, &c, Vector3::zeros());
        // Reflection of v through c on the X axis: c.x - (v.x - c.x) = 0.10.
        // (Y axis 180° rotation also flips Z, but z component of v - c
        // is 0, so z stays at c.z = 0.30.)
        assert!((baked.x - 0.10).abs() < 1e-12, "baked.x = {}", baked.x);
        assert!(baked.y.abs() < 1e-12, "baked.y = {}", baked.y);
        assert!((baked.z - 0.30).abs() < 1e-12, "baked.z = {}", baked.z);
    }

    // ----- Cap algorithms (plane fit / normal / centerline) ---------
    /// `fit_plane_to_points` on a perfectly planar XY-plane loop:
    /// normal should be ±Z; R² should be 1.0; centroid at origin.
    #[test]
    fn plane_fit_on_xy_loop_returns_z_normal_and_unit_r_squared() {
        let points = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
        ];
        let (centroid, normal, r_sq) = fit_plane_to_points(&points);

        assert!(centroid.x.abs() < 1e-9);
        assert!(centroid.y.abs() < 1e-9);
        assert!(centroid.z.abs() < 1e-9);
        // Normal is ±Z (SVD doesn't pick a sign).
        assert!(normal.x.abs() < 1e-9, "normal x: {}", normal.x);
        assert!(normal.y.abs() < 1e-9, "normal y: {}", normal.y);
        assert!(
            (normal.z.abs() - 1.0).abs() < 1e-9,
            "normal z: {}",
            normal.z
        );
        // Planar loop → R² ≈ 1.
        assert!((r_sq - 1.0).abs() < 1e-9, "r² = {r_sq}");
    }
    /// `fit_plane_to_points` on a non-planar (twisted) loop has
    /// R² < 1. Four points lifted slightly off the XY plane in
    /// alternating directions; the best-fit plane is still ~XY but
    /// the residual is non-zero.
    #[test]
    fn plane_fit_on_twisted_loop_has_r_squared_below_one() {
        let points = vec![
            Point3::new(1.0, 0.0, 0.05),
            Point3::new(0.0, 1.0, -0.05),
            Point3::new(-1.0, 0.0, 0.05),
            Point3::new(0.0, -1.0, -0.05),
        ];
        let (_, _, r_sq) = fit_plane_to_points(&points);
        assert!(r_sq < 1.0);
        assert!(
            r_sq > 0.5,
            "non-trivial twist; r² should still be high: {r_sq}"
        );
    }
    /// `fit_plane_to_points` on < 3 input points returns the
    /// fallback (R² = 0, normal = +Z). Degenerate input shouldn't
    /// panic.
    #[test]
    fn plane_fit_degenerate_input_returns_fallback() {
        let too_few = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let (_, _, r_sq) = fit_plane_to_points(&too_few);
        assert_eq!(r_sq, 0.0);
    }
    /// `orient_cap_normal_outward` flips the normal so it points
    /// away from the side containing more mesh vertices. Build a
    /// mesh with all vertices above the cap plane (z > 0); the
    /// outward normal should be -Z (pointing away from mesh-side).
    #[test]
    fn orient_cap_normal_flips_to_point_away_from_mesh_majority() {
        let mut mesh = IndexedMesh::with_capacity(5, 0);
        for v in &[
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
            (-1.0, 0.0, 2.0),
            (0.0, -1.0, 2.0),
        ] {
            mesh.vertices.push(Point3::new(v.0, v.1, v.2));
        }
        let plane_centroid = Point3::origin();
        // Try with normal pointing +Z (toward the mesh majority).
        let oriented =
            orient_cap_normal_outward(&mesh, plane_centroid, Vector3::new(0.0, 0.0, 1.0));
        // Outward = away from majority = -Z.
        assert!(
            oriented.z < 0.0,
            "expected outward normal to point -Z; got {:?}",
            oriented,
        );
    }
    /// Cap-face 3D winding emission anchor for
    /// `auto_cap_open_boundaries` (see also the sister anchor on the
    /// `build_cleaned_mesh` path inside
    /// `build_cleaned_mesh_projects_loop_verts_onto_fit_plane`). The
    /// function must emit cap triangles whose 3D cross-product normals
    /// align with `orient_cap_normal_outward`'s returned OUTWARD
    /// direction. Without this assertion the emission could (and
    /// historically did, until the B arc fix at commit `99f2c512`)
    /// produce inward normals: cf-view's flat-shaded render path
    /// would compute inward face normals from the winding and shade
    /// cap faces DARK; mesh-io's `save_stl` would write inverted
    /// facet normals to disk for 3rd-party STL tools (Meshlab,
    /// ParaView, slicers).
    ///
    /// ⚠ This anchor used to close with "all in-tree SDF consumers
    /// tolerate either winding … so the bug was visualization-only".
    /// The three oracles it named do tolerate either winding
    /// (`FloodFillSign` is topological-reachability-based;
    /// `TriMeshDistance` is unsigned; cf-cap-planes uses `.dot().abs()`)
    /// — but they are not all of them, so the conclusion did not
    /// follow. `mesh_sdf::PseudoNormalSign` takes its sign from face
    /// winding, and production paths compose it throughout the tree —
    /// mesh-offset's `offset_mesh`, cf-fsu-geometry's `oracle`,
    /// cf-device-geometry's `build_cached_scan_sdf`, cf-cast-cli's
    /// `derive_spec_and_ribbon` and cf-sim-research's
    /// `run_sdf_bridge_spike` among them. Inward caps reaching any of
    /// them invert the field's sign rather than its shading, so the
    /// blast radius was never bounded to rendering. The assertion below
    /// is what keeps that moot.
    ///
    /// Fixture: 5-vertex square pyramid with the base OPEN (4 side
    /// triangles, no base triangulation). Apex at (0, 0, +1); base
    /// at z=0 with 4 corner verts. The open base is the boundary loop.
    /// `orient_cap_normal_outward` checks vertex distribution — 4 verts
    /// at z=0 (no sign), 1 at z=+1 (above) → returns -plane_normal →
    /// outward = -Z. After cap, every cap-face cross product must
    /// point -Z (outward, away from apex).
    #[test]
    fn auto_cap_open_boundaries_emits_outward_cap_normals() {
        // 5 verts + 4 side triangles + 2 cap triangles = 6 faces post-cap.
        let mut mesh = IndexedMesh::with_capacity(5, 6);
        // Base square (z=0) + apex (z=+1).
        mesh.vertices.push(Point3::new(-1.0, -1.0, 0.0)); // 0
        mesh.vertices.push(Point3::new(1.0, -1.0, 0.0)); // 1
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2
        mesh.vertices.push(Point3::new(-1.0, 1.0, 0.0)); // 3
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4 apex
        // 4 side triangles (CCW from outside — outward normals point
        // sideways + upward). Base is open.
        mesh.faces.push([0, 1, 4]);
        mesh.faces.push([1, 2, 4]);
        mesh.faces.push([2, 3, 4]);
        mesh.faces.push([3, 0, 4]);
        let initial_faces = mesh.faces.len();
        let capped = auto_cap_open_boundaries(&mut mesh);
        assert_eq!(capped, 1, "expected exactly 1 cap loop to be capped");
        let cap_faces = &mesh.faces[initial_faces..];
        assert!(
            !cap_faces.is_empty(),
            "auto-cap should append ≥1 cap triangle",
        );
        // Every cap face's 3D cross-product normal must point -Z
        // (outward, away from the apex above). Without the B-arc
        // fix the normals pointed +Z (inward toward apex), shading
        // dark under Bevy lighting in cf-view.
        for face in cap_faces {
            let v0 = mesh.vertices[face[0] as usize];
            let v1 = mesh.vertices[face[1] as usize];
            let v2 = mesh.vertices[face[2] as usize];
            let e1 = v1.coords - v0.coords;
            let e2 = v2.coords - v0.coords;
            let normal = e1.cross(&e2);
            assert!(
                normal.z < 0.0,
                "cap-face normal must point -Z (outward, away from apex); \
                 face {face:?} produced normal {normal:?}",
            );
        }
    }
    /// Plane bisecting a unit cube perpendicular to +Z at `z=0.5`
    /// produces exactly one closed loop describing the unit square
    /// cross-section at z=0.5. **Contract test for
    /// `intersect_plane_with_mesh`.**
    ///
    /// The unit cube's 4 vertical edges each contribute one corner;
    /// each side face is split into 2 triangles whose internal
    /// DIAGONAL also crosses the plane, contributing 4 extra
    /// (collinear-on-the-square's-edges) intersection points. So
    /// the returned loop has 8 vertices around a square boundary,
    /// not the "minimal" 4 — this is correct algorithmic behavior
    /// (the polygon centroid is invariant to such mesh-diagonal
    /// subdivisions, verified by `polygon_centroid_3d_density_independent`).
    /// Test asserts the polygon's CENTROID + AREA, not vertex count.
    #[test]
    fn intersect_plane_unit_cube_bisecting_z_returns_square() {
        let mesh = make_unit_cube_mesh();
        let plane_pt = Point3::new(0.5, 0.5, 0.5);
        let plane_n = Vector3::new(0.0, 0.0, 1.0);
        let loops = intersect_plane_with_mesh(&plane_pt, &plane_n, &mesh);
        assert_eq!(
            loops.len(),
            1,
            "expected exactly 1 loop; got {}",
            loops.len()
        );
        let loop0 = &loops[0];
        assert!(
            loop0.len() >= 4,
            "expected ≥ 4 verts on square cross-section; got {}",
            loop0.len()
        );
        // Every vertex sits at z=0.5 and on the boundary of the
        // unit square (x or y coordinate equals 0 or 1).
        for p in loop0 {
            assert!((p.z - 0.5).abs() < 1e-9, "vert {p:?} not on z=0.5 plane");
            let on_boundary = (p.x.abs() < 1e-9 || (p.x - 1.0).abs() < 1e-9)
                || (p.y.abs() < 1e-9 || (p.y - 1.0).abs() < 1e-9);
            assert!(
                on_boundary,
                "vert {p:?} not on unit-square boundary at z=0.5"
            );
        }
        // Polygon-level geometric properties: area = 1.0, centroid
        // = (0.5, 0.5, 0.5).
        let area = polygon_area_3d(loop0, &plane_n);
        assert!((area - 1.0).abs() < 1e-9, "area should be 1.0; got {area}");
        let c = polygon_centroid_3d(loop0, &plane_n).expect("non-degenerate square");
        assert!((c.x - 0.5).abs() < 1e-9, "centroid x: {c:?}");
        assert!((c.y - 0.5).abs() < 1e-9, "centroid y: {c:?}");
        assert!((c.z - 0.5).abs() < 1e-9, "centroid z: {c:?}");
    }
    /// Plane parked outside the mesh's z-range produces no loops.
    /// Defensive test for the "no triangles crossed" path.
    #[test]
    fn intersect_plane_outside_mesh_returns_empty() {
        let mesh = make_unit_cube_mesh();
        let plane_pt = Point3::new(0.0, 0.0, 10.0);
        let plane_n = Vector3::new(0.0, 0.0, 1.0);
        let loops = intersect_plane_with_mesh(&plane_pt, &plane_n, &mesh);
        assert!(loops.is_empty(), "expected no loops; got {}", loops.len());
    }
    /// Defensive paths: empty mesh OR zero-magnitude normal return
    /// `Vec::new()` without panicking.
    #[test]
    fn intersect_plane_degenerate_input_returns_empty() {
        let empty = IndexedMesh::with_capacity(0, 0);
        let plane_pt = Point3::origin();
        let plane_n = Vector3::new(0.0, 0.0, 1.0);
        assert!(intersect_plane_with_mesh(&plane_pt, &plane_n, &empty).is_empty());

        let cube = make_unit_cube_mesh();
        let zero_n = Vector3::zeros();
        assert!(intersect_plane_with_mesh(&plane_pt, &zero_n, &cube).is_empty());
    }
    /// Polygon centroid of an axis-aligned unit square in the
    /// xy-plane sits at the square's geometric center. Contract
    /// test for [`polygon_centroid_3d`].
    #[test]
    fn polygon_centroid_3d_unit_square_at_center() {
        let square = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let n = Vector3::new(0.0, 0.0, 1.0);
        let c = polygon_centroid_3d(&square, &n).expect("non-degenerate square");
        assert!((c.x - 0.5).abs() < 1e-12, "x centroid: {c:?}");
        assert!((c.y - 0.5).abs() < 1e-12, "y centroid: {c:?}");
        assert!(c.z.abs() < 1e-12, "z centroid: {c:?}");
    }
    /// **Load-bearing test for density-independence.** A unit
    /// square traversed with 4 vertices vs. the SAME square with
    /// 4 extra collinear midpoints (8 verts on the boundary, same
    /// SHAPE) produces the same centroid. This is the geometric
    /// property that makes the centerline algorithm density-
    /// independent: adding redundant boundary samples does not
    /// shift the polygon centroid, unlike the failed vertex-centroid
    /// statistic (which would average toward the dense side).
    #[test]
    fn polygon_centroid_3d_density_independent() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let sparse = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Same boundary, traversed with midpoints between each
        // pair of corners (8 verts total, all on the boundary).
        let dense = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.5, 0.0),
        ];
        let c_sparse = polygon_centroid_3d(&sparse, &n).unwrap();
        let c_dense = polygon_centroid_3d(&dense, &n).unwrap();
        let diff = (c_sparse.coords - c_dense.coords).norm();
        assert!(
            diff < 1e-12,
            "density-asymmetric boundary shifted centroid: sparse={c_sparse:?}, dense={c_dense:?}, diff={diff}"
        );
    }
    /// Polygon centroid is invariant to traversal winding (CW vs
    /// CCW with respect to `plane_normal`): the signed-area sum in
    /// the numerator AND denominator both flip sign, canceling out.
    /// Defensive test — the centerline algorithm doesn't control the
    /// winding of loops produced by [`intersect_plane_with_mesh`].
    #[test]
    fn polygon_centroid_3d_winding_invariant() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let ccw = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let cw = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ];
        let c_ccw = polygon_centroid_3d(&ccw, &n).unwrap();
        let c_cw = polygon_centroid_3d(&cw, &n).unwrap();
        let diff = (c_ccw.coords - c_cw.coords).norm();
        assert!(
            diff < 1e-12,
            "winding changed centroid: ccw={c_ccw:?}, cw={c_cw:?}"
        );
    }
    /// `polygon_centroid_3d` returns `None` for degenerate inputs:
    /// < 3 vertices OR collinear vertices (zero projected area).
    #[test]
    fn polygon_centroid_3d_degenerate_returns_none() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let two = [Point3::origin(), Point3::new(1.0, 0.0, 0.0)];
        assert!(polygon_centroid_3d(&two, &n).is_none());
        let collinear = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        assert!(polygon_centroid_3d(&collinear, &n).is_none());
    }
    /// `polygon_area_3d` of a unit square is 1.0; winding-invariant.
    #[test]
    fn polygon_area_3d_unit_square() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let ccw = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        assert!((polygon_area_3d(&ccw, &n) - 1.0).abs() < 1e-12);
        let cw = [ccw[0], ccw[3], ccw[2], ccw[1]];
        assert!((polygon_area_3d(&cw, &n) - 1.0).abs() < 1e-12);
    }
    /// `compute_centerline_polyline` on a closed Z-axis cylinder
    /// produces polyline points along the Z axis (every centroid
    /// at (0, 0, z)) with monotonically increasing z. **Contract
    /// test for the per-slab area-weighted polygon centroid
    /// algorithm.** XYZ-independence (non-Z body axes) is
    /// separately covered by [`centerline_algorithm_xyz_independent`].
    #[test]
    fn centerline_along_closed_cylinder_z_axis_follows_z() {
        let mesh = make_closed_frustum_mesh(10, 16, 0.1, 0.1, 2.0, |_, _, _| 0.0);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 10);
        assert!(
            polyline.len() >= 5,
            "expected ≥5 polyline pts; got {}",
            polyline.len()
        );
        for p in &polyline {
            assert!(p.x.abs() < 1e-9, "polyline x drifted: {p:?}");
            assert!(p.y.abs() < 1e-9, "polyline y drifted: {p:?}");
        }
        let zs: Vec<f64> = polyline.iter().map(|p| p.z).collect();
        for window in zs.windows(2) {
            assert!(
                window[1] > window[0],
                "polyline z should monotonically increase: {zs:?}",
            );
        }
    }
    /// `compute_centerline_polyline` on empty mesh / zero-direction
    /// input returns an empty Vec without panicking.
    #[test]
    fn centerline_degenerate_input_returns_empty() {
        let empty = IndexedMesh::with_capacity(0, 0);
        let polyline = compute_centerline_polyline(&empty, Vector3::new(1.0, 0.0, 0.0), 10);
        assert!(polyline.is_empty());

        let mut tiny = IndexedMesh::with_capacity(1, 0);
        tiny.vertices.push(Point3::new(0.0, 0.0, 0.0));
        let zero_dir = compute_centerline_polyline(&tiny, Vector3::zeros(), 10);
        assert!(zero_dir.is_empty());
    }
    /// A tapering frustum (radius 1.0 at z=−0.5 → 0.05 at z=+0.5,
    /// 20 rings × 16 segs, with asymmetric per-vertex noise) has a
    /// centerline that stays on its true axis at EVERY slab,
    /// including the narrow-tip end. **Load-bearing test for the
    /// per-slab area-weighted polygon centroid algorithm vs. the
    /// failed vertex-centroid statistic** — vertex-centroid would
    /// be biased toward the noise direction at small-cross-section
    /// slabs; polygon-centroid is geometry-only, so the noise's
    /// effect averages out around the slab boundary.
    #[test]
    fn centerline_tapered_cylinder_tip_stays_on_axis() {
        const N_RINGS: usize = 20;
        const N_SEGS: usize = 16;
        const RADIUS_BASE: f64 = 1.0;
        const RADIUS_TIP: f64 = 0.05;
        const NOISE_MAG: f64 = 0.005; // 0.5 % of the base radius

        let mesh = make_closed_frustum_mesh(
            N_RINGS,
            N_SEGS,
            RADIUS_BASE,
            RADIUS_TIP,
            1.0,
            |ring, seg, _theta| {
                // Asymmetric noise: amplitude tied to (ring + seg)
                // so different rings have different noise patterns.
                NOISE_MAG * (((ring * 7 + seg * 13) % 11) as f64 / 11.0 - 0.5)
            },
        );

        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), N_RINGS);
        assert!(
            polyline.len() >= N_RINGS - 2,
            "expected ~N_RINGS polyline pts; got {}",
            polyline.len()
        );

        // Every polyline point should be on the body axis (x ≈ 0,
        // y ≈ 0) within the per-slab noise envelope. The polygon
        // centroid is unbiased w.r.t. uniform-around-the-ring noise;
        // residual drift comes from second-order chord-area
        // asymmetry (bounded by ~ NOISE_MAG for a 16-segment
        // polygon).
        for (i, p) in polyline.iter().enumerate() {
            assert!(
                p.x.abs() < NOISE_MAG,
                "x drift at slab {i}: {p:?} (NOISE_MAG = {NOISE_MAG})"
            );
            assert!(
                p.y.abs() < NOISE_MAG,
                "y drift at slab {i}: {p:?} (NOISE_MAG = {NOISE_MAG})"
            );
        }
    }
    /// **Spec test #1 (contract)** — a perfectly symmetric closed
    /// cylinder along Z produces a centerline pinned to the Z axis
    /// at every slab. Tightest tolerance of the suite.
    #[test]
    fn centerline_algorithm_axisymmetric_cylinder_along_axis() {
        let mesh = make_closed_frustum_mesh(20, 32, 0.05, 0.05, 0.2, |_, _, _| 0.0);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(polyline.len(), 30);
        for (i, p) in polyline.iter().enumerate() {
            assert!(p.x.abs() < 1e-9, "x drift at slab {i}: {p:?}");
            assert!(p.y.abs() < 1e-9, "y drift at slab {i}: {p:?}");
        }
    }
    /// **Spec test #2 (regression for iter-1 failure mode)** — a
    /// closed cylinder translated by Δx = 5mm off the world Z axis
    /// produces a centerline pinned to the BODY'S axis (i.e. at
    /// (5mm, 0, z)), NOT to the world Z axis. This is the iteration-5
    /// failure mode the algorithm switch fixes.
    #[test]
    fn centerline_algorithm_offset_cylinder_tracks_body_axis() {
        const OFFSET_X: f64 = 0.005;
        let mut mesh = make_closed_frustum_mesh(20, 32, 0.05, 0.05, 0.2, |_, _, _| 0.0);
        translate_mesh(&mut mesh, Vector3::new(OFFSET_X, 0.0, 0.0));
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(polyline.len(), 30);
        for (i, p) in polyline.iter().enumerate() {
            assert!(
                (p.x - OFFSET_X).abs() < 1e-9,
                "x should track OFFSET_X at slab {i}: {p:?}"
            );
            assert!(p.y.abs() < 1e-9, "y should be 0 at slab {i}: {p:?}");
        }
    }
    /// **Spec test #3 (load-bearing: density-independence)** — a
    /// cylinder sampled with 80% of its angular vertices in the
    /// right semi-circle and 20% in the left semi-circle has the
    /// SAME centerline as a uniformly-sampled cylinder of the same
    /// radius. This is the property that escapes ALL five prior
    /// failed algorithms (Kasa, Kasa+prior, PCA, vertex-centroid,
    /// AABB-midpoint were each biased by sampling asymmetry).
    ///
    /// Vertex-centroid baseline (FOR THE OLD ALGORITHM, would have
    /// FAILED): mean x of the boundary samples for the dense fixture
    /// is ~ +0.022 m (vs. radius 0.05 → 44% bias), which would have
    /// pulled the prior centerline off-axis by ~22mm on a 50mm-radius
    /// body. The new algorithm produces sub-mm residual.
    #[test]
    fn centerline_algorithm_density_independent() {
        const RADIUS: f64 = 0.05;
        // 80% (24/30) of thetas densely packed in the right
        // semi-circle (theta ∈ (-π/2, π/2)); 20% (6/30) sparsely
        // in the left semi-circle. Boundary shape is still the
        // same circle.
        let mut thetas: Vec<f64> = Vec::new();
        for k in 0..24 {
            #[allow(clippy::cast_precision_loss)]
            let f = (k as f64 + 0.5) / 24.0;
            thetas.push(-std::f64::consts::FRAC_PI_2 + f * std::f64::consts::PI);
        }
        for k in 0..6 {
            #[allow(clippy::cast_precision_loss)]
            let f = (k as f64 + 0.5) / 6.0;
            thetas.push(std::f64::consts::FRAC_PI_2 + f * std::f64::consts::PI);
        }
        let mesh = make_density_biased_cylinder_mesh(20, &thetas, RADIUS, 0.2);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(polyline.len(), 30);
        // Residual chord-area asymmetry: ~(1 - cos(π / n_dense))
        // for the dense side × similar on sparse side. With 24 vs 6
        // segments, residual x-drift is at most a few percent of
        // RADIUS (sub-mm at RADIUS=50mm). The vertex-centroid
        // statistic on the SAME fixture would drift by ~50% of
        // RADIUS — this tolerance is 100× tighter.
        let tol_x = 0.05 * RADIUS;
        for (i, p) in polyline.iter().enumerate() {
            assert!(
                p.x.abs() < tol_x,
                "density-biased x drift at slab {i}: {p:?} (tol = {tol_x})"
            );
            assert!(
                p.y.abs() < tol_x,
                "density-biased y drift at slab {i}: {p:?} (tol = {tol_x})"
            );
        }
    }
    /// **Spec test #4 (degenerate-slab interpolation)** — a needle
    /// cone (radius 1.0 at z=−0.5 → 0.00001 at z=+0.5) has its
    /// tip-end slabs fall below MIN_SLAB_AREA_M2 (π × 1e-5² ≈
    /// 3e-10 m² < 1e-8 m²). Those degenerate slabs must be filled
    /// in by linear interpolation / axial extrapolation; the
    /// polyline length should match `n_slices` exactly (no gaps)
    /// and the filled-in points should stay on the body axis.
    #[test]
    fn centerline_algorithm_degenerate_slabs_filled_by_interpolation() {
        let mesh = make_closed_frustum_mesh(20, 32, 1.0, 1e-5, 1.0, |_, _, _| 0.0);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(
            polyline.len(),
            30,
            "polyline length should equal n_slices even with degenerate slabs"
        );
        // All points (interior + filled-in tip) should be on the
        // body axis. The interpolation from non-degenerate
        // neighbors stays on axis since those neighbors are on axis
        // for a symmetric cone.
        for (i, p) in polyline.iter().enumerate() {
            assert!(p.x.abs() < 1e-9, "x drift at slab {i}: {p:?}");
            assert!(p.y.abs() < 1e-9, "y drift at slab {i}: {p:?}");
        }
    }
    /// **Spec test #5 (XYZ-independence)** — a closed cylinder
    /// whose body axis has been rotated 30° around Y produces a
    /// centerline that follows the ROTATED body axis, not the
    /// world Z axis. The algorithm uses `spine_hint` (not world
    /// directions) for slicing, so any spine_hint direction works.
    /// See `project_scans_axis_orientation` memo.
    #[test]
    fn centerline_algorithm_xyz_independent() {
        let mut mesh = make_closed_frustum_mesh(20, 32, 0.05, 0.05, 0.2, |_, _, _| 0.0);
        let axis_angle_deg = 30.0;
        let rotation = UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            axis_angle_deg * std::f64::consts::PI / 180.0,
        );
        rotate_mesh(&mut mesh, rotation);
        let rotated_axis = rotation * Vector3::new(0.0, 0.0, 1.0);
        let polyline = compute_centerline_polyline(&mesh, rotated_axis, 30);
        assert_eq!(polyline.len(), 30);
        // Each polyline point should lie on the line through origin
        // in direction `rotated_axis`. Cross-product magnitude with
        // the axis direction tells us the perpendicular distance —
        // should be ~ 0 for an on-axis point.
        for (i, p) in polyline.iter().enumerate() {
            let perp = p.coords.cross(&rotated_axis).norm();
            assert!(
                perp < 1e-9,
                "perp distance from rotated axis at slab {i}: {perp} (point {p:?})"
            );
        }
    }

    // ----- Centerline trim (CSP.4b) ----------------------------------
    /// `trim_mesh_along_centerline` with both trims at 0 is a
    /// pure-clone no-op (returns the input unchanged). Pins the
    /// fast-path guard so future refactors don't accidentally walk
    /// vertices unnecessarily for un-trimmed saves.
    #[test]
    fn trim_mesh_along_centerline_no_op_when_both_trims_zero() {
        let mesh = one_triangle_at(0.01);
        let centerline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)];
        let trimmed = trim_mesh_along_centerline(&mesh, &centerline, 0.0, 0.0);
        assert_eq!(trimmed.vertices, mesh.vertices);
        assert_eq!(trimmed.faces, mesh.faces);
    }
    /// Degenerate centerline (< 2 points) is a no-op. Defensive
    /// against the "Cap → Scan not yet clicked" path (CapState
    /// centerline empty); the save handler skips trim entirely in
    /// that case, but the function itself guards too.
    #[test]
    fn trim_mesh_along_centerline_no_op_when_centerline_too_short() {
        let mesh = one_triangle_at(0.01);
        let centerline = vec![Point3::new(0.0, 0.0, 0.0)];
        let trimmed = trim_mesh_along_centerline(&mesh, &centerline, 10.0, 10.0);
        assert_eq!(trimmed.vertices, mesh.vertices);
        assert_eq!(trimmed.faces, mesh.faces);
    }
    /// Trim from the floor end of a +Z-axis centerline drops
    /// vertices beyond the trim plane. Vertical "pole" of 11 points
    /// from z=0 to z=0.1 (100 mm), centerline tip→floor along +Z,
    /// trim floor by 30 mm → keep points with z <= 0.07.
    #[test]
    fn trim_mesh_along_centerline_floor_end_clips_high_z_vertices() {
        // Build a simple vertical column of triangles (pole), 11
        // segments stacked from z=0 to z=0.1. Each "stack-rung" is
        // a degenerate-ish 3-vertex triangle for testing.
        let mut mesh = IndexedMesh::with_capacity(11, 9);
        for i in 0..11 {
            let z = f64::from(i) * 0.01;
            mesh.vertices.push(Point3::new(0.0, 0.0, z));
        }
        // Connect into a chain of "triangles" with shared verts.
        // (Geometrically degenerate but valid topology; serves the
        // trim test.)
        for i in 0..9 {
            mesh.faces.push([i as u32, (i + 1) as u32, (i + 2) as u32]);
        }
        // Centerline from tip (z=0) to floor (z=0.1). Trim floor
        // by 30 mm → clip plane at z = 0.07.
        let centerline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.1)];
        let trimmed = trim_mesh_along_centerline(&mesh, &centerline, 0.0, 30.0);
        // All surviving vertices' z <= 0.07 (the trim plane).
        for v in &trimmed.vertices {
            assert!(v.z <= 0.07 + 1e-9, "kept vertex z={} above trim plane", v.z);
        }
        // Some vertices survived.
        assert!(!trimmed.vertices.is_empty(), "all dropped by trim");
    }
    /// `trim_centerline_polyline` keeps the polyline whole when no
    /// trim is requested. Pins the no-op identity.
    #[test]
    fn trim_centerline_polyline_no_op_when_both_zero() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.0, 0.0, 0.1),
        ];
        let trimmed = trim_centerline_polyline(&polyline, 0.0, 0.0);
        assert_eq!(trimmed, polyline);
    }
    /// `trim_centerline_polyline` cuts the start + end of the
    /// polyline at the requested mm distances. 100 mm polyline,
    /// trim 30 mm tip + 20 mm floor → result is 50 mm long, from
    /// the 30 mm mark to the 80 mm mark.
    #[test]
    fn trim_centerline_polyline_clips_both_ends_at_arc_length() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),   //  0 mm
            Point3::new(0.0, 0.0, 0.025), // 25 mm
            Point3::new(0.0, 0.0, 0.05),  // 50 mm
            Point3::new(0.0, 0.0, 0.075), // 75 mm
            Point3::new(0.0, 0.0, 0.1),   // 100 mm
        ];
        let trimmed = trim_centerline_polyline(&polyline, 30.0, 20.0);
        // First trimmed point: z = 0.030 (interpolated between 25
        // and 50 mm originals).
        assert!((trimmed.first().unwrap().z - 0.030).abs() < 1e-9);
        // Last trimmed point: z = 0.080.
        assert!((trimmed.last().unwrap().z - 0.080).abs() < 1e-9);
        // Total arc length collapsed from 100 mm to ~50 mm.
        let new_total_mm: f64 = trimmed
            .windows(2)
            .map(|w| (w[1].coords - w[0].coords).norm() * 1000.0)
            .sum();
        assert!((new_total_mm - 50.0).abs() < 1e-6);
    }
    /// `trim_centerline_polyline` returns empty when the trim
    /// consumes the entire polyline (`trim_tip + trim_floor >=
    /// total arc length`). The save handler's
    /// `centerline.is_empty()` check then omits the `[centerline]`
    /// block from the TOML.
    #[test]
    fn trim_centerline_polyline_returns_empty_when_fully_consumed() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.1), // 100 mm total
        ];
        // Trim 60 + 60 = 120 mm > 100 mm total.
        let trimmed = trim_centerline_polyline(&polyline, 60.0, 60.0);
        assert!(trimmed.is_empty());
    }
    /// CSP.4b.3 regression test — the polyline-walker primitive
    /// that drives both the trim algorithm + the live overlay.
    /// Pre-CSP.4b.3 trim/overlay code extrapolated linearly from
    /// the first segment's tangent, which diverged from the actual
    /// curve as the trim distance grew. This test builds a
    /// L-shaped polyline (sharp 90° bend at mid-arc) and asserts
    /// that walking past the bend lands on the SECOND-leg
    /// position, NOT linearly extrapolated from the first leg's
    /// tangent.
    #[test]
    fn point_along_polyline_walks_through_a_bend() {
        // L-shaped polyline:
        //   (0,0,0) → (0,0,0.05) → (0.05,0,0.05)
        // Total arc length: 0.05 + 0.05 = 0.10 m (100 mm).
        // First leg (+Z, 50 mm); second leg (+X, 50 mm); 90° bend
        // at index 1.
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.05, 0.0, 0.05),
        ];

        // 25 mm in: on the first leg. Expect (0, 0, 0.025), tangent = +Z.
        let (p, t) = point_along_polyline_at_arc_distance(&polyline, 0.025).unwrap();
        assert!((p.x - 0.0).abs() < 1e-12);
        assert!((p.z - 0.025).abs() < 1e-12);
        assert!((t.z - 1.0).abs() < 1e-9);
        assert!(t.x.abs() < 1e-9);

        // 75 mm in: 25 mm into the SECOND leg. Expect (0.025, 0,
        // 0.05), tangent = +X. The pre-CSP.4b.3 buggy code would
        // have walked 75 mm along the first leg's +Z tangent and
        // landed at (0, 0, 0.075) — way off the actual polyline.
        let (p, t) = point_along_polyline_at_arc_distance(&polyline, 0.075).unwrap();
        assert!(
            (p.x - 0.025).abs() < 1e-12,
            "x post-bend should be 0.025, got {}",
            p.x
        );
        assert!(
            (p.z - 0.05).abs() < 1e-12,
            "z post-bend should be 0.05, got {}",
            p.z
        );
        assert!(
            (t.x - 1.0).abs() < 1e-9,
            "tangent post-bend should be +X, got {t:?}"
        );
        assert!(t.z.abs() < 1e-9);
    }
    /// `point_along_polyline_at_arc_distance` returns `None` for a
    /// polyline with < 2 points (no segment to walk). Pairs with
    /// the trim algorithm's "skip if centerline too short" path.
    #[test]
    fn point_along_polyline_returns_none_when_too_short() {
        let single = vec![Point3::origin()];
        assert!(point_along_polyline_at_arc_distance(&single, 0.0).is_none());
        let empty: Vec<Point3<f64>> = Vec::new();
        assert!(point_along_polyline_at_arc_distance(&empty, 0.1).is_none());
    }
    /// `polyline_arc_length_m` sums Euclidean segment lengths.
    #[test]
    fn polyline_arc_length_sums_segments() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.05, 0.0, 0.05),
            Point3::new(0.05, 0.0, 0.0),
        ];
        // 50 + 50 + 50 mm = 0.15 m.
        let total = polyline_arc_length_m(&polyline);
        assert!((total - 0.15).abs() < 1e-12);
    }
    /// CSP.4e.2.3 — `smooth_polyline` pins the endpoints and
    /// flattens an interior zigzag. Fixture: 5-point polyline
    /// where the middle point is offset perpendicular to the
    /// otherwise-straight line. After 3 iterations of 3-tap
    /// moving average, the middle point should be pulled
    /// substantially back toward the straight line; endpoints
    /// `polyline[0]` and `polyline[N-1]` should be unchanged.
    #[test]
    fn smooth_polyline_flattens_interior_pins_endpoints() {
        let raw = vec![
            Point3::new(0.0, 0.0, 0.0), // endpoint
            Point3::new(0.01, 0.0, 0.0),
            Point3::new(0.02, 0.005, 0.0), // wobble +5 mm
            Point3::new(0.03, 0.0, 0.0),
            Point3::new(0.04, 0.0, 0.0), // endpoint
        ];
        let smoothed = smooth_polyline(&raw, 3);
        assert_eq!(smoothed.len(), raw.len());
        // Endpoints bit-exact preserved.
        assert_eq!(smoothed[0], raw[0]);
        assert_eq!(smoothed[4], raw[4]);
        // Middle point's y substantially reduced (started at
        // 5 mm; after 3 iterations of 3-tap should be < 2 mm).
        assert!(
            smoothed[2].y.abs() < raw[2].y.abs(),
            "middle should smooth toward 0, got y = {}",
            smoothed[2].y,
        );
        assert!(
            smoothed[2].y.abs() < 0.002,
            "after 3 iterations 5 mm wobble should drop below 2 mm, got {}",
            smoothed[2].y,
        );
    }
    /// `smooth_polyline` is a no-op for < 3-point input (no
    /// interior to smooth) and for `iterations == 0`.
    #[test]
    fn smooth_polyline_no_op_when_too_short_or_zero_iters() {
        let two_pt = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        assert_eq!(smooth_polyline(&two_pt, 5), two_pt);
        let three_pt = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ];
        assert_eq!(smooth_polyline(&three_pt, 0), three_pt);
    }

    // ----- simplify_mesh post-process --------------------------------
    /// `simplify_mesh` post-process strips unreferenced vertices, so
    /// every vertex in the output is touched by at least one face. The
    /// converse (every face references valid vertex indices) is
    /// trivially true since `remove_unreferenced_vertices` preserves
    /// face validity by construction.
    #[test]
    fn simplify_mesh_strips_unreferenced_vertices() {
        let original = grid_square(20);
        let result = simplify_mesh(&original, 50);

        let vertex_count = result.mesh.vertices.len();
        let mut touched = vec![false; vertex_count];
        for face in &result.mesh.faces {
            for &idx in face {
                touched[idx as usize] = true;
            }
        }
        let untouched = touched.iter().filter(|t| !**t).count();
        assert_eq!(
            untouched, 0,
            "simplified mesh should have no unreferenced vertices",
        );
    }

    // ----- triangulate_polygon_2d_earclip ----------------------------
    /// Ear-clipping a unit square produces exactly 2 triangles
    /// covering the full polygon area (4 area units of 0.5 each).
    /// Validates the basic ear-clip + signed-area path.
    #[test]
    fn earclip_unit_square_produces_two_triangles() {
        let verts = unit_square_2d();
        let tris = triangulate_polygon_2d_earclip(&verts);
        assert_eq!(tris.len(), 2);
        // Total signed area of the triangulation should equal the
        // square's area (1.0). Sums absolute signed area per tri.
        let total_area: f64 = tris
            .iter()
            .map(|t| {
                let a = verts[t[0] as usize];
                let b = verts[t[1] as usize];
                let c = verts[t[2] as usize];
                0.5 * ((b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)).abs()
            })
            .sum();
        assert!(
            (total_area - 1.0).abs() < 1e-12,
            "expected total area 1.0, got {total_area}",
        );
    }
    /// Reversing a CCW polygon to CW must still produce a valid
    /// 2-triangle triangulation. Pins the `signed_area < 0 -> reverse`
    /// branch of the ear-clip.
    #[test]
    fn earclip_cw_polygon_is_handled() {
        let mut verts = unit_square_2d();
        verts.reverse();
        let tris = triangulate_polygon_2d_earclip(&verts);
        assert_eq!(tris.len(), 2);
    }

    // ----- stable_inward_tangent --------------------------------------
    /// 2-point polyline → look-back walks the only segment fully
    /// and returns the head→tail (then negated) direction. The
    /// `lookback_m` exceeding the polyline arc-length falls
    /// through to the head endpoint, NOT an error.
    #[test]
    fn stable_inward_tangent_falls_back_to_last_segment_on_2_point_polyline() {
        let polyline = [Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 0.0)];
        // Cut endpoint = (0,0,0); look-back at (0,0,1). Inward
        // direction (cut → body) = +Z.
        let dir = stable_inward_tangent(&polyline, 0.020).expect("tangent");
        assert!((dir - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }
    /// On a long polyline the look-back point sits at exactly
    /// `lookback_m` arc-length from the cut endpoint (interpolated
    /// within the relevant segment). Pin the resulting direction
    /// against a known-good handcomputed answer.
    #[test]
    fn stable_inward_tangent_walks_back_lookback_arc_length() {
        // 4-point centerline along +Z, segments of length 0.010 m.
        // Total arc length = 0.030 m.
        let polyline = [
            Point3::new(0.0, 0.0, 0.030), // body end
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, 0.010),
            Point3::new(0.0, 0.0, 0.000), // cut end
        ];
        // Look back 0.015 m → between segments [2] and [1] (i.e.
        // halfway through the segment from (0,0,0.010) to
        // (0,0,0.020)). Look-back point = (0,0,0.015).
        let dir = stable_inward_tangent(&polyline, 0.015).expect("tangent");
        // dir = (lookback - cut) / norm = (0,0,0.015) / 0.015 = +Z.
        assert!((dir - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }
    /// Look-back exceeding the polyline arc-length walks the
    /// whole polyline and returns the head→tail unit vector.
    /// Important so very-short trim-cut polylines don't crash.
    #[test]
    fn stable_inward_tangent_uses_full_polyline_when_lookback_exceeds_arc_length() {
        // Total arc length = 0.005 m; ask for 0.020 m look-back.
        let polyline = [Point3::new(0.0, 0.0, 0.005), Point3::new(0.0, 0.0, 0.000)];
        let dir = stable_inward_tangent(&polyline, 0.020).expect("tangent");
        assert!((dir - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }
    /// Polygons with `n < 3` vertices produce no triangles (early
    /// return). Otherwise an `n=3` polygon should produce exactly
    /// 1 triangle.
    #[test]
    fn earclip_handles_degenerate_vertex_counts() {
        assert!(triangulate_polygon_2d_earclip(&[]).is_empty());
        assert!(triangulate_polygon_2d_earclip(&[(0.0, 0.0)]).is_empty());
        assert!(triangulate_polygon_2d_earclip(&[(0.0, 0.0), (1.0, 0.0)]).is_empty());
        let single = triangulate_polygon_2d_earclip(&[(0.0, 0.0), (1.0, 0.0), (0.5, 1.0)]);
        assert_eq!(single.len(), 1);
    }
    /// A convex pentagon (5 vertices) produces 3 triangles
    /// (`n - 2 = 3` for any simple polygon).
    #[test]
    fn earclip_convex_pentagon_produces_three_triangles() {
        // Regular-ish pentagon at unit radius.
        let verts = (0..5)
            .map(|i| {
                let theta = i as f64 * std::f64::consts::TAU / 5.0;
                (theta.cos(), theta.sin())
            })
            .collect::<Vec<_>>();
        let tris = triangulate_polygon_2d_earclip(&verts);
        assert_eq!(tris.len(), 3);
    }
    /// `iso8601_utc_from_unix_seconds` produces a parseable, sensible
    /// timestamp. Pins the date math against a known unix epoch:
    /// `1640995200` = `2022-01-01T00:00:00Z`.
    #[test]
    fn iso8601_timestamp_matches_known_unix_epoch() {
        let s = iso8601_utc_from_unix_seconds(1_640_995_200);
        assert_eq!(s, "2022-01-01T00:00:00Z");
    }
    #[test]
    fn compute_reconstructed_floor_plane_matches_apply_reconstruction_geometry() {
        // Synthetic 40 mm centerline along +Z; floor end at z = 0.
        // The reconstruction workflow is: trim removes the last
        // `applied_floor_mm` of arc length, then `apply_reconstruction`
        // extends the body by `applied_floor_mm` along the OUTWARD
        // direction from the trimmed endpoint. The trim-then-extend
        // pair restores the original body length with cleaner floor
        // geometry, so `bottom_center` lands at the original
        // polyline's floor endpoint (z = 0.000 in this fixture).
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.040),
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, 0.010),
            Point3::new(0.0, 0.0, 0.000),
        ];
        let rf =
            compute_reconstructed_floor_plane_physics(&polyline, 0.0, 10.0).expect("Some plane");
        // Trim drops last 10 mm → trimmed_last = z=0.010; outward
        // extension by 10 mm in -Z → bottom_center z = 0.000.
        assert!((rf.centroid_m.x).abs() < 1e-9);
        assert!((rf.centroid_m.y).abs() < 1e-9);
        assert!(
            (rf.centroid_m.z).abs() < 1e-9,
            "bottom_center.z = {} (expected 0.000)",
            rf.centroid_m.z,
        );
        // Outward normal = -inward (+Z) = -Z.
        assert!((rf.normal.x).abs() < 1e-9);
        assert!((rf.normal.y).abs() < 1e-9);
        assert!((rf.normal.z - (-1.0)).abs() < 1e-9);
    }
    #[test]
    fn compute_reconstructed_floor_plane_iter1_reproducer() {
        // Reproduces the iter-1 numerical hypothesis check: the
        // synthetic offset along the cap normal should be ~ extension
        // along the tangent ≈ 40 mm (since recorded cap_centroid is
        // the pre-reconstruction boundary fit-plane at the trim cut,
        // and bottom_center is 40 mm beyond it along the centerline
        // tangent). Pin the math on a tangent-mostly-Z polyline so
        // future refactors of stable_inward_tangent don't silently
        // drift the recorded plane back toward the old stale position.
        let polyline = vec![
            Point3::new(0.001, -0.002, 0.060),
            Point3::new(0.001, -0.002, 0.040),
            Point3::new(0.001, -0.002, 0.020),
            Point3::new(0.001, -0.002, 0.000),
        ];
        let rf =
            compute_reconstructed_floor_plane_physics(&polyline, 0.0, 20.0).expect("Some plane");
        // Trim drops last 20 mm → trimmed_last = z=0.020; outward
        // (= -Z) extension by 20 mm → bottom_center z = 0.000.
        assert!((rf.centroid_m.z).abs() < 1e-9);
        // The override's centroid sits 20 mm below the trim cut
        // (trimmed_last z = 0.020) along the tangent. cf-device-design's
        // candidate-A clip would otherwise hit the body at trim_cut +
        // recorded-plane-offset; this assertion is the load-bearing
        // contract that the override moves it to bottom_center.
        let trim_cut_z = 0.020;
        let override_minus_trim = rf.centroid_m.z - trim_cut_z;
        assert!((override_minus_trim - (-0.020)).abs() < 1e-9);
    }
    #[test]
    fn compute_reconstructed_floor_plane_returns_none_for_zero_floor_trim() {
        let polyline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.010)];
        assert!(compute_reconstructed_floor_plane_physics(&polyline, 0.0, 0.0).is_none());
    }
    #[test]
    fn compute_reconstructed_floor_plane_returns_none_for_short_polyline() {
        let polyline = vec![Point3::new(0.0, 0.0, 0.0)];
        assert!(compute_reconstructed_floor_plane_physics(&polyline, 0.0, 10.0).is_none());
        let polyline2 = vec![Point3::new(0.0, 0.0, 0.020), Point3::new(0.0, 0.0, 0.010)];
        // applied_floor_mm = 15 > polyline length (10 mm) → trim
        // consumes everything → None.
        assert!(compute_reconstructed_floor_plane_physics(&polyline2, 0.0, 15.0).is_none());
    }
    /// CSP.3b — `cleanup_cleaned_mesh_for_disk` welds STL-style
    /// unshared vertices into the shared-index layout that
    /// downstream `simplify_decoder` requires. Builds a 4×4
    /// grid-square (32 faces — comfortably above the 10-face
    /// small-component floor), STL-unshares it via the test helper,
    /// runs cleanup, and confirms the welded result has the shared-
    /// vertex topology the iter-1 fixture's downstream consumer was
    /// missing (`tools/cf-device-design/src/main.rs:419-425`).
    #[test]
    fn cleanup_welds_stl_style_unshared_vertices() {
        let shared = grid_square(4); // 32 faces, 25 shared verts
        let shared_vert_count = shared.vertices.len();
        let shared_face_count = shared.faces.len();
        assert!(shared_face_count >= CLEANUP_MIN_COMPONENT_FACES);
        let mut unshared = unshare_vertices(&shared);
        // STL-unshared form: 3 verts per face.
        assert_eq!(unshared.vertices.len(), shared_face_count * 3);

        // Smoothing disabled — this test asserts the weld/degenerate
        // hygiene path, not the smoothing pass.
        let report = cleanup_cleaned_mesh_for_disk(&mut unshared, 0);
        assert!(
            report.welded > 0,
            "weld_vertices report welded=0: {report:?}",
        );
        // Welded back down to the shared form (25 verts) with all
        // faces surviving.
        assert_eq!(unshared.vertices.len(), shared_vert_count);
        assert_eq!(unshared.faces.len(), shared_face_count);
    }
    /// CSP.3b — small-component strip drops scanner-noise islands
    /// below `CLEANUP_MIN_COMPONENT_FACES`. Builds a 20-face main
    /// component + a single 1-face island; cleanup keeps the main
    /// component + drops the island.
    #[test]
    fn cleanup_drops_small_component_islands() {
        // Main component: 5×5 grid (50 faces).
        let mut mesh = grid_square(5);
        let main_faces_before = mesh.faces.len();
        assert!(main_faces_before >= CLEANUP_MIN_COMPONENT_FACES);
        // Add a stray 1-face island far from the main grid (so
        // welding doesn't accidentally merge it in).
        let v0 = mesh.vertices.len() as u32;
        mesh.vertices.push(Point3::new(100.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(101.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(100.5, 1.0, 0.0));
        mesh.faces.push([v0, v0 + 1, v0 + 2]);

        let report = cleanup_cleaned_mesh_for_disk(&mut mesh, 0);
        assert!(
            report.small_components >= 1,
            "small_components report: {report:?}",
        );
        // The 1-face island is gone; main component faces survive.
        assert_eq!(mesh.faces.len(), main_faces_before);
    }
    /// CSP.3b — `CleanupReport::total()` is zero for an
    /// already-clean input mesh. Pins the "no work needed → status
    /// message stays clean" path so future cleanup pipeline
    /// additions don't silently start surfacing spurious counts on
    /// pristine inputs.
    #[test]
    fn cleanup_report_total_zero_on_clean_input() {
        // 5×5 grid is already in shared-index form, no degenerates,
        // single component, no unreferenced verts.
        let mut mesh = grid_square(5);
        let report = cleanup_cleaned_mesh_for_disk(&mut mesh, 0);
        assert_eq!(report.total(), 0, "clean mesh produced cleanup: {report:?}");
    }
    /// Smoothing with non-zero iterations counts toward
    /// [`CleanupReport::total()`] AND actually moves vertices.
    /// Pins the slider's `iterations > 0` path so a future
    /// refactor doesn't silently disconnect the slider from
    /// the smoothing call.
    #[test]
    fn cleanup_applies_taubin_smoothing_when_iterations_positive() {
        let mut mesh = grid_square(5);
        let original_verts = mesh.vertices.clone();
        let report = cleanup_cleaned_mesh_for_disk(&mut mesh, 5);
        assert_eq!(
            report.smoothing, 5,
            "smoothing iters not threaded: {report:?}"
        );
        // grid_square produces vertices ON a plane; Taubin should
        // not significantly displace them (Laplacian of a planar
        // mesh is ~zero), but some boundary verts will drift
        // toward the interior. At least one vertex should differ.
        let any_moved = mesh
            .vertices
            .iter()
            .enumerate()
            .any(|(i, v)| (v.coords - original_verts[i].coords).norm() > 1e-12);
        assert!(
            any_moved,
            "Taubin smoothing with 5 iters did not move any vertex on grid_square(5)",
        );
    }
    /// Atomic-write produces both expected files when both writes
    /// succeed. Uses a tempfile-style temp dir constructed from
    /// `std::env::temp_dir()` + a unique stem so tests don't collide.
    #[test]
    fn atomic_write_save_lands_both_files() -> Result<()> {
        let tmp_root = std::env::temp_dir().join(format!(
            "cf-scan-prep-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0),
        ));
        std::fs::create_dir_all(&tmp_root)?;
        let stl_path = tmp_root.join("scan.cleaned.stl");
        let toml_path = tmp_root.join("scan.prep.toml");
        let mesh = one_triangle_at(0.001);
        atomic_write_save(&mesh, &stl_path, &toml_path, "key = \"value\"\n")?;
        assert!(stl_path.exists(), "cleaned STL should land at final path");
        assert!(toml_path.exists(), ".prep.toml should land at final path");
        // No `.tmp` files left over.
        assert!(!stl_path.with_extension("stl.tmp").exists());
        assert!(!toml_path.with_extension("toml.tmp").exists());
        // Cleanup.
        let _ = std::fs::remove_dir_all(&tmp_root);
        Ok(())
    }

    // ----- reconstruction path: fixtures ------------------------------

    /// Frustum with a **closed tip cap and an OPEN base** — the shape
    /// [`apply_reconstruction`] expects: exactly one boundary loop, at
    /// the cut. Ring `r` sits at `z = r/(n_rings-1) * height` with
    /// radius lerping `radius_base -> radius_tip`, so the reference
    /// zone above the cut obeys the exact linear law
    /// `r(s) = radius_base + ((radius_tip - radius_base)/height) * s`.
    /// That law is the independent oracle for the Extrapolate fit.
    fn open_base_frustum(
        n_rings: usize,
        n_segs: usize,
        radius_base: f64,
        radius_tip: f64,
        height: f64,
    ) -> IndexedMesh {
        assert!(n_rings >= 2 && n_segs >= 3);
        let mut mesh = IndexedMesh::with_capacity(n_rings * n_segs + 1, 2 * n_rings * n_segs);
        for r in 0..n_rings {
            #[allow(clippy::cast_precision_loss)]
            let t = (r as f64) / ((n_rings - 1) as f64);
            let z = t * height;
            let ring_radius = radius_base * (1.0 - t) + radius_tip * t;
            for k in 0..n_segs {
                #[allow(clippy::cast_precision_loss)]
                let theta = (k as f64) * std::f64::consts::TAU / (n_segs as f64);
                mesh.vertices.push(Point3::new(
                    ring_radius * theta.cos(),
                    ring_radius * theta.sin(),
                    z,
                ));
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let tip_idx = (n_rings * n_segs) as u32;
        mesh.vertices.push(Point3::new(0.0, 0.0, height));
        for r in 0..(n_rings - 1) {
            for k in 0..n_segs {
                #[allow(clippy::cast_possible_truncation)]
                let i00 = (r * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i01 = (r * n_segs + (k + 1) % n_segs) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i10 = ((r + 1) * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i11 = ((r + 1) * n_segs + (k + 1) % n_segs) as u32;
                mesh.faces.push([i00, i01, i11]);
                mesh.faces.push([i00, i11, i10]);
            }
        }
        let top_base = (n_rings - 1) * n_segs;
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = (top_base + k) as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = (top_base + (k + 1) % n_segs) as u32;
            mesh.faces.push([tip_idx, i0, i1]);
        }
        mesh
    }

    /// Centerline running tip -> cut, so `centerline.last()` is the cut
    /// at `z = 0` and the inward tangent is `+Z`.
    fn axis_centerline(height: f64, n: usize) -> Vec<Point3<f64>> {
        #[allow(clippy::cast_precision_loss)]
        (0..n)
            .map(|i| {
                let t = (i as f64) / ((n - 1) as f64);
                Point3::new(0.0, 0.0, height * (1.0 - t))
            })
            .collect()
    }

    /// XY distance from the axis — the radius of a reconstruction ring
    /// vertex, computed without reusing the production basis.
    fn axial_radius(p: Point3<f64>) -> f64 {
        (p.x * p.x + p.y * p.y).sqrt()
    }

    // ----- perpendicular_basis_for ------------------------------------

    /// `(u, v, n)` must be a right-handed orthonormal frame. Checked
    /// against the definition (dot/cross identities), not against the
    /// Gram-Schmidt steps the implementation happens to use.
    #[test]
    fn perpendicular_basis_is_orthonormal_and_right_handed() {
        for n in [
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 2.0, 3.0).normalize(),
            Vector3::new(-0.3, 0.4, -0.86602540378).normalize(),
        ] {
            let (u, v) = perpendicular_basis_for(n);
            assert!((u.norm() - 1.0).abs() < 1e-12, "u not unit for {n:?}");
            assert!((v.norm() - 1.0).abs() < 1e-12, "v not unit for {n:?}");
            assert!(u.dot(&n).abs() < 1e-12, "u not perpendicular to {n:?}");
            assert!(v.dot(&n).abs() < 1e-12, "v not perpendicular to {n:?}");
            assert!(u.dot(&v).abs() < 1e-12, "u,v not orthogonal for {n:?}");
            assert!(
                (u.cross(&v) - n).norm() < 1e-12,
                "frame is left-handed for {n:?}"
            );
        }
    }

    /// The `n.x.abs() < 0.9` guard exists so the Gram-Schmidt seed is
    /// never parallel to `n`. `n = +X` takes the else-branch; without
    /// the guard the subtraction would leave the zero vector and
    /// `normalize()` would produce NaN.
    #[test]
    fn perpendicular_basis_stays_finite_when_n_is_the_x_axis() {
        let (u, v) = perpendicular_basis_for(Vector3::new(1.0, 0.0, 0.0));
        assert!(u.iter().all(|c| c.is_finite()), "u went non-finite: {u:?}");
        assert!(v.iter().all(|c| c.is_finite()), "v went non-finite: {v:?}");
        assert!((u.norm() - 1.0).abs() < 1e-12);
        assert!(u.dot(&Vector3::new(1.0, 0.0, 0.0)).abs() < 1e-12);
    }

    // ----- find_floor_loop_index --------------------------------------

    fn loop_of(n: usize) -> holes::BoundaryLoop {
        #[allow(clippy::cast_possible_truncation)]
        holes::BoundaryLoop {
            vertices: (0..n as u32).collect(),
        }
    }

    /// The rim is the LARGEST loop, and largest must win over both
    /// earlier and later candidates.
    #[test]
    fn floor_loop_is_the_largest_loop_not_the_first() {
        let loops = [loop_of(12), loop_of(30), loop_of(11)];
        let mesh = IndexedMesh::with_capacity(0, 0);
        let got = find_floor_loop_index(&loops, &mesh, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(got, Some(1));
    }

    /// CSP.4e.2 regression: on an unsimplified scan the mesh carries
    /// hundreds of 3-vertex scanner stragglers. Picking one of those
    /// generated the degenerate "white vertical line" column. Loops
    /// under `MIN_RIM_LOOP_VERTS` (10) must be refused outright — even
    /// when they are the only loops present.
    #[test]
    fn floor_loop_refuses_noise_stragglers_rather_than_picking_one() {
        let loops = [loop_of(3), loop_of(9), loop_of(5)];
        let mesh = IndexedMesh::with_capacity(0, 0);
        assert_eq!(
            find_floor_loop_index(&loops, &mesh, Point3::new(0.0, 0.0, 0.0)),
            None
        );
    }

    /// A degenerate loop (< 3 vertices) is not a loop at all; it must
    /// be skipped even though nothing else qualifies on count alone.
    #[test]
    fn floor_loop_skips_invalid_loops_and_takes_the_valid_one() {
        let loops = [loop_of(2), loop_of(10)];
        let mesh = IndexedMesh::with_capacity(0, 0);
        assert_eq!(
            find_floor_loop_index(&loops, &mesh, Point3::new(0.0, 0.0, 0.0)),
            Some(1)
        );
    }

    // ----- sample_radius_at_angle -------------------------------------

    /// A flat profile is flat everywhere, including between bins and
    /// outside `[0, TAU)`.
    #[test]
    fn radius_at_angle_of_a_flat_profile_is_that_radius_everywhere() {
        let radii = [2.5_f64; RECONSTRUCT_ANGLE_BINS];
        for angle in [0.0, 0.37, 2.0, -1.1, std::f64::consts::TAU - 1e-9] {
            assert!((sample_radius_at_angle(&radii, angle) - 2.5).abs() < 1e-12);
        }
    }

    /// Halfway between two bin centres the result is their mean —
    /// linear interpolation, computed here from the bin values rather
    /// than from the implementation's `t`.
    #[test]
    fn radius_at_angle_interpolates_linearly_between_adjacent_bins() {
        let mut radii = [0.0_f64; RECONSTRUCT_ANGLE_BINS];
        radii[0] = 1.0;
        radii[1] = 3.0;
        #[allow(clippy::cast_precision_loss)]
        let bin_span = std::f64::consts::TAU / RECONSTRUCT_ANGLE_BINS as f64;
        assert!((sample_radius_at_angle(&radii, 0.0) - 1.0).abs() < 1e-12);
        assert!((sample_radius_at_angle(&radii, 0.5 * bin_span) - 2.0).abs() < 1e-12);
        assert!((sample_radius_at_angle(&radii, bin_span) - 3.0).abs() < 1e-12);
    }

    /// The profile is periodic: the last bin interpolates back into the
    /// first, and a negative angle names the same direction as its
    /// positive coterminal.
    #[test]
    fn radius_at_angle_wraps_across_the_seam_and_normalizes_negatives() {
        let mut radii = [0.0_f64; RECONSTRUCT_ANGLE_BINS];
        radii[0] = 4.0;
        radii[RECONSTRUCT_ANGLE_BINS - 1] = 2.0;
        #[allow(clippy::cast_precision_loss)]
        let bin_span = std::f64::consts::TAU / RECONSTRUCT_ANGLE_BINS as f64;
        #[allow(clippy::cast_precision_loss)]
        let seam = (RECONSTRUCT_ANGLE_BINS - 1) as f64 * bin_span + 0.5 * bin_span;
        assert!(
            (sample_radius_at_angle(&radii, seam) - 3.0).abs() < 1e-12,
            "seam did not interpolate bin[23] -> bin[0]"
        );
        for a in [0.1_f64, 1.3, 3.0] {
            let neg = sample_radius_at_angle(&radii, a - std::f64::consts::TAU);
            let pos = sample_radius_at_angle(&radii, a);
            assert!((neg - pos).abs() < 1e-12, "negative angle differed at {a}");
        }
    }

    // ----- sample_radial_profile --------------------------------------

    /// On a right cylinder every angular bin sees the same radius.
    #[test]
    fn radial_profile_of_a_cylinder_is_the_cylinder_radius_in_every_bin() {
        let mesh = open_base_frustum(6, 24, 0.02, 0.02, 0.060);
        let (u, v) = perpendicular_basis_for(Vector3::new(0.0, 0.0, 1.0));
        let radii = sample_radial_profile(
            &mesh,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            u,
            v,
            0.030,
        );
        for (i, r) in radii.iter().enumerate() {
            assert!((r - 0.02).abs() < 1e-9, "bin {i} was {r}, expected 0.02");
        }
    }

    /// Only the slab `0 < s <= reference_zone` may contribute. Vertices
    /// below the cut and beyond the zone are decoys with a wildly
    /// different radius; if either leaked in, the medians would move.
    #[test]
    fn radial_profile_ignores_geometry_outside_the_reference_zone() {
        let mut mesh = open_base_frustum(6, 24, 0.02, 0.02, 0.060);
        let baseline = sample_radial_profile(
            &mesh,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.030,
        );
        // Deleting EITHER half of the zone guard also admits the frustum's
        // own out-of-zone rings (5 more samples at r = 0.02), so the decoys
        // must outnumber those too or the median swallows the leak and this
        // test passes with the guard gone. Six per side does it; three did
        // not, and the surviving mutant is what said so.
        for k in 0..24 {
            #[allow(clippy::cast_precision_loss)]
            let th = (k as f64) * std::f64::consts::TAU / 24.0;
            for j in 0..6 {
                #[allow(clippy::cast_precision_loss)]
                let d = (j as f64) * 0.002;
                // below the cut (s < 0)
                mesh.vertices
                    .push(Point3::new(0.5 * th.cos(), 0.5 * th.sin(), -0.005 - d));
                // beyond the reference zone (s > 0.030)
                mesh.vertices
                    .push(Point3::new(0.5 * th.cos(), 0.5 * th.sin(), 0.050 + d));
            }
        }
        let after = sample_radial_profile(
            &mesh,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.030,
        );
        assert_eq!(
            baseline, after,
            "vertices outside the reference zone changed the profile"
        );
    }

    /// Per-bin **median**, not mean: one wild vertex inside the zone
    /// must not move its bin. A mean would land near 0.18 here.
    #[test]
    fn radial_profile_median_absorbs_a_single_outlier_vertex() {
        let mut mesh = open_base_frustum(6, 24, 0.02, 0.02, 0.060);
        mesh.vertices.push(Point3::new(0.5, 0.0, 0.012));
        let radii = sample_radial_profile(
            &mesh,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.030,
        );
        assert!(
            (radii[0] - 0.02).abs() < 1e-9,
            "outlier moved bin 0 to {} — median degraded to a mean?",
            radii[0]
        );
    }

    // ----- sample_radial_profile_linear_fit ---------------------------

    /// The fixture's radius is exactly linear in `s`, so the per-bin
    /// regression must recover the fixture's own coefficients. Both are
    /// derived from the frustum's geometry, independently of the
    /// least-squares code under test.
    #[test]
    fn linear_fit_recovers_the_frustums_analytic_taper() {
        let (r_base, r_tip, height) = (0.020_f64, 0.030_f64, 0.060_f64);
        // 120 segments over 24 bins: every bin keeps >= 2 samples even when
        // floating-point rounding drops an angle into its neighbour. At
        // n_segs == RECONSTRUCT_ANGLE_BINS some bins come up short and take
        // the flat fallback, which is a different branch (covered separately).
        let mesh = open_base_frustum(6, 120, r_base, r_tip, height);
        let expected_slope = (r_tip - r_base) / height;
        let (intercepts, slopes) = sample_radial_profile_linear_fit(
            &mesh,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.050,
        );
        for i in 0..RECONSTRUCT_ANGLE_BINS {
            assert!(
                (slopes[i] - expected_slope).abs() < 1e-9,
                "bin {i} slope {} != analytic {expected_slope}",
                slopes[i]
            );
            assert!(
                (intercepts[i] - r_base).abs() < 1e-9,
                "bin {i} intercept {} != analytic {r_base}",
                intercepts[i]
            );
        }
    }

    /// The linear-fit fixture obeys one law everywhere, so admitting
    /// out-of-zone geometry would leave the fit unchanged and a deleted
    /// zone guard would go unnoticed. These decoys sit OFF that line —
    /// below the cut and past the zone — so any leak drags the
    /// regression away from the frustum's analytic coefficients.
    #[test]
    fn linear_fit_ignores_geometry_outside_the_reference_zone() {
        let (r_base, r_tip, height) = (0.020_f64, 0.030_f64, 0.060_f64);
        let mut mesh = open_base_frustum(6, 120, r_base, r_tip, height);
        for k in 0..120 {
            #[allow(clippy::cast_precision_loss)]
            let th = (k as f64) * std::f64::consts::TAU / 120.0;
            for j in 0..4 {
                #[allow(clippy::cast_precision_loss)]
                let d = (j as f64) * 0.002;
                mesh.vertices
                    .push(Point3::new(0.4 * th.cos(), 0.4 * th.sin(), -0.004 - d));
                mesh.vertices
                    .push(Point3::new(0.4 * th.cos(), 0.4 * th.sin(), 0.052 + d));
            }
        }
        let expected_slope = (r_tip - r_base) / height;
        let (intercepts, slopes) = sample_radial_profile_linear_fit(
            &mesh,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.050,
        );
        for i in 0..RECONSTRUCT_ANGLE_BINS {
            assert!(
                (slopes[i] - expected_slope).abs() < 1e-9,
                "bin {i} slope {} moved off the analytic {expected_slope}",
                slopes[i]
            );
            assert!(
                (intercepts[i] - r_base).abs() < 1e-9,
                "bin {i} intercept {} moved off the analytic {r_base}",
                intercepts[i]
            );
        }
    }

    /// A bin with fewer than two samples cannot support a regression;
    /// it falls back to the overall median with zero slope rather than
    /// dividing by a degenerate denominator.
    #[test]
    fn linear_fit_falls_back_to_flat_for_bins_with_too_few_samples() {
        let mut mesh = IndexedMesh::with_capacity(4, 0);
        // Two samples in bin 0 (angle 0), one lone sample near angle pi.
        mesh.vertices.push(Point3::new(0.02, 0.0, 0.005));
        mesh.vertices.push(Point3::new(0.02, 0.0, 0.010));
        mesh.vertices.push(Point3::new(-0.09, 0.0, 0.008));
        let (intercepts, slopes) = sample_radial_profile_linear_fit(
            &mesh,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.030,
        );
        let overall = 0.02_f64; // median of [0.02, 0.02, 0.09]
        let lone_bin = RECONSTRUCT_ANGLE_BINS / 2;
        assert!(
            (slopes[lone_bin]).abs() < 1e-15,
            "single-sample bin got a slope: {}",
            slopes[lone_bin]
        );
        assert!(
            (intercepts[lone_bin] - overall).abs() < 1e-12,
            "single-sample bin did not fall back to the overall median: {}",
            intercepts[lone_bin]
        );
    }

    // ----- apply_reconstruction ---------------------------------------

    /// A centerline too short to define a tangent cannot drive an
    /// extrusion. The documented contract is "fall back to flat-cap",
    /// so the result must match a plain `auto_cap_open_boundaries` —
    /// compared against that function directly, not against a count.
    #[test]
    fn reconstruction_with_a_degenerate_centerline_falls_back_to_flat_cap() {
        let mesh = open_base_frustum(4, 16, 0.02, 0.02, 0.040);
        let mut expected = mesh.clone();
        auto_cap_open_boundaries(&mut expected);

        let short = vec![Point3::new(0.0, 0.0, 0.0)];
        let got =
            apply_reconstruction(mesh.clone(), &short, 10.0, 20.0, ReconstructShape::Constant);
        assert_eq!(got.vertices.len(), expected.vertices.len());
        assert_eq!(got.faces.len(), expected.faces.len());
        assert!(
            detect_boundary_loops(&got).is_empty(),
            "result is not closed"
        );
    }

    /// A non-positive extension is the same "nothing to reconstruct"
    /// branch, and must not emit a zero-height extrusion.
    #[test]
    fn reconstruction_with_a_non_positive_extension_falls_back_to_flat_cap() {
        let mesh = open_base_frustum(4, 16, 0.02, 0.02, 0.040);
        let mut expected = mesh.clone();
        auto_cap_open_boundaries(&mut expected);

        let cl = axis_centerline(0.040, 5);
        let got = apply_reconstruction(mesh.clone(), &cl, 0.0, 20.0, ReconstructShape::Constant);
        assert_eq!(got.vertices.len(), expected.vertices.len());
        assert_eq!(got.faces.len(), expected.faces.len());
    }

    /// The happy path adds exactly `K` rings of `L` vertices plus one
    /// fan centroid, and `2·L·K + L` faces — counts derived from the
    /// documented algorithm, and the result must be watertight.
    #[test]
    fn reconstruction_adds_the_documented_ring_geometry_and_closes_the_mesh() {
        let n_segs = 16;
        let mesh = open_base_frustum(5, n_segs, 0.02, 0.02, 0.050);
        let v0 = mesh.vertices.len();
        let f0 = mesh.faces.len();
        let cl = axis_centerline(0.050, 6);

        let got = apply_reconstruction(mesh, &cl, 8.0, 25.0, ReconstructShape::Constant);

        assert_eq!(
            got.vertices.len(),
            v0 + RECONSTRUCT_RING_COUNT * n_segs + 1,
            "expected K*L + 1 new vertices"
        );
        assert_eq!(
            got.faces.len(),
            f0 + 2 * n_segs * RECONSTRUCT_RING_COUNT + n_segs,
            "expected 2*L*K + L new faces"
        );
        assert!(
            detect_boundary_loops(&got).is_empty(),
            "reconstructed mesh still has an open boundary"
        );
    }

    /// The new floor sits `applied_floor_mm` beyond the cut, along the
    /// direction opposite the inward tangent. Getting this wrong is the
    /// 2.73 mm mid-body plane the `[caps]` override exists to prevent.
    #[test]
    fn reconstruction_puts_the_new_floor_at_the_requested_depth() {
        let n_segs = 16;
        let mesh = open_base_frustum(5, n_segs, 0.02, 0.02, 0.050);
        let v0 = mesh.vertices.len();
        let cl = axis_centerline(0.050, 6);

        let got = apply_reconstruction(mesh, &cl, 8.0, 25.0, ReconstructShape::Constant);

        let centroid = got.vertices[v0 + RECONSTRUCT_RING_COUNT * n_segs];
        assert!(
            (centroid.z - (-0.008)).abs() < 1e-9,
            "floor centroid at z={}, expected -0.008",
            centroid.z
        );
        assert!(centroid.x.abs() < 1e-9 && centroid.y.abs() < 1e-9);
    }

    /// Taper's contract is `(1 - RECONSTRUCT_TAPER_AT_FLOOR)` of the
    /// canonical radius at the floor. Compared against the Constant
    /// run's own bottom ring, so the assertion needs no hard-coded
    /// radius.
    #[test]
    fn taper_pinches_the_floor_ring_by_the_documented_fraction() {
        let n_segs = 16;
        let cl = axis_centerline(0.050, 6);
        let bottom_ring = |shape| {
            let mesh = open_base_frustum(5, n_segs, 0.02, 0.02, 0.050);
            let v0 = mesh.vertices.len();
            let got = apply_reconstruction(mesh, &cl, 8.0, 25.0, shape);
            let start = v0 + (RECONSTRUCT_RING_COUNT - 1) * n_segs;
            (start..start + n_segs)
                .map(|i| axial_radius(got.vertices[i]))
                .collect::<Vec<_>>()
        };
        let constant = bottom_ring(ReconstructShape::Constant);
        let taper = bottom_ring(ReconstructShape::Taper);
        for (i, (c, t)) in constant.iter().zip(taper.iter()).enumerate() {
            let expected = c * (1.0 - RECONSTRUCT_TAPER_AT_FLOOR);
            assert!(
                (t - expected).abs() < 1e-9,
                "vertex {i}: taper {t} != {expected} (constant {c})"
            );
        }
    }

    /// Extrapolate must continue the reference zone's trend below the
    /// cut. The fixture's taper is analytic, so the floor radius is
    /// predicted in closed form — `r_base + slope·(-extension)` — with
    /// no reference to the regression code.
    #[test]
    fn extrapolate_continues_the_measured_taper_below_the_cut() {
        let (r_base, r_tip, height) = (0.020_f64, 0.030_f64, 0.060_f64);
        // Densely sampled for the same reason as the linear-fit test: with
        // fewer segments than bins, the empty bins fall back to a flat
        // profile and the interpolated floor radius mixes the two branches.
        let n_segs = 120;
        let extension_m = 0.008_f64;
        let mesh = open_base_frustum(7, n_segs, r_base, r_tip, height);
        let v0 = mesh.vertices.len();
        let cl = axis_centerline(height, 7);

        let got = apply_reconstruction(mesh, &cl, 8.0, 50.0, ReconstructShape::Extrapolate);

        let slope = (r_tip - r_base) / height;
        let expected = slope.mul_add(-extension_m, r_base);
        let start = v0 + (RECONSTRUCT_RING_COUNT - 1) * n_segs;
        for i in start..start + n_segs {
            let r = axial_radius(got.vertices[i]);
            assert!(
                (r - expected).abs() < 1e-6,
                "floor radius {r} != analytic extrapolation {expected}"
            );
        }
        assert!(
            expected < r_base,
            "fixture chosen wrong: extrapolation must narrow below the cut"
        );
    }

    /// The K rings smoothstep from the noisy rim toward the canonical
    /// profile — that gradual fade IS the anti-lip behaviour, so it has
    /// to be asserted on an INTERMEDIATE ring. On a clean cylinder the
    /// rim already equals the canonical radius and every blend curve
    /// gives the same answer, so the rim here is widened to 1.5x while
    /// the reference zone above the cut is left alone. Ring 2 is chosen
    /// because smoothstep(0.25) = 0.15625 while a linear blend would
    /// give 0.25 — at ring 4 the two curves cross and prove nothing.
    #[test]
    fn extrusion_rings_smoothstep_from_the_rim_toward_the_canonical_profile() {
        let n_segs = 16;
        let radius = 0.020_f64;
        let mut mesh = open_base_frustum(5, n_segs, radius, radius, 0.050);
        // Widen ONLY the rim (ring 0, the boundary loop at z = 0).
        for v in mesh.vertices.iter_mut().take(n_segs) {
            v.x *= 1.5;
            v.y *= 1.5;
        }
        let v0 = mesh.vertices.len();
        let cl = axis_centerline(0.050, 6);

        let got = apply_reconstruction(mesh, &cl, 8.0, 25.0, ReconstructShape::Constant);

        let t_k = 2.0 / RECONSTRUCT_RING_COUNT as f64;
        let blend = t_k * t_k * (3.0 - 2.0 * t_k);
        let expected = (1.5 * radius).mul_add(1.0 - blend, radius * blend);
        let start = v0 + n_segs; // ring k = 2
        for i in start..start + n_segs {
            let r = axial_radius(got.vertices[i]);
            assert!(
                (r - expected).abs() < 1e-9,
                "ring 2 radius {r} != smoothstep blend {expected}"
            );
        }
    }

    /// An aggressive reference-zone trend extrapolated over a long
    /// extension projects through zero to a NEGATIVE radius. Unclamped,
    /// the ring vertices would pass through the centerline and come out
    /// the far side — the mesh self-crosses its own axis. The clamp
    /// pins the floor to the axis instead. Fixture: slope 1.0 mm/mm
    /// with a 40 mm extension, so the raw extrapolation is -0.020.
    #[test]
    fn extrapolate_clamps_a_runaway_taper_at_the_centerline() {
        let n_segs = 120;
        let mesh = open_base_frustum(6, n_segs, 0.020, 0.060, 0.040);
        let v0 = mesh.vertices.len();
        let cl = axis_centerline(0.040, 6);

        let got = apply_reconstruction(mesh, &cl, 40.0, 35.0, ReconstructShape::Extrapolate);

        let start = v0 + (RECONSTRUCT_RING_COUNT - 1) * n_segs;
        for i in start..start + n_segs {
            let r = axial_radius(got.vertices[i]);
            assert!(
                r < 1e-9,
                "floor ring vertex sits {r} off the axis; the negative \
                 extrapolation was not clamped and the ring inverted"
            );
        }
    }

    // ----- build_detected_cap_loop ------------------------------------

    /// Spec §6: loops with >= 8 vertices are included by default,
    /// smaller ones are treated as acceptable holes / scanner artifacts.
    #[test]
    fn detected_cap_loop_includes_large_loops_and_defers_small_ones() {
        let mesh = open_base_frustum(4, 16, 0.02, 0.02, 0.040);
        let loops = detect_boundary_loops(&mesh);
        assert_eq!(loops.len(), 1, "fixture should have exactly one open loop");
        let big = build_detected_cap_loop(&mesh, &loops[0]);
        assert!(big.include, "a 16-vertex rim should default to included");

        let small = holes::BoundaryLoop {
            vertices: loops[0].vertices.iter().copied().take(5).collect(),
        };
        assert!(
            !build_detected_cap_loop(&mesh, &small).include,
            "a 5-vertex loop should not default to included"
        );
    }

    /// The cap normal points AWAY from the body. The fixture's rim is
    /// at `z = 0` with the whole mesh above it, so outward is `-Z`.
    #[test]
    fn detected_cap_loop_normal_points_away_from_the_mesh_body() {
        let mesh = open_base_frustum(4, 16, 0.02, 0.02, 0.040);
        let loops = detect_boundary_loops(&mesh);
        let cap = build_detected_cap_loop(&mesh, &loops[0]);
        assert!(
            cap.plane_normal.z < -0.99,
            "outward normal was {:?}, expected ~-Z",
            cap.plane_normal
        );
        assert!(
            cap.plane_centroid.z.abs() < 1e-9,
            "rim centroid should sit on z=0, got {}",
            cap.plane_centroid.z
        );
    }
}
