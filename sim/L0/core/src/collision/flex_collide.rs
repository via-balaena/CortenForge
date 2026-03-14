//! Flex collision dispatcher — internal collision (adjacent elements) and
//! flex-flex cross-object pair enumeration.
//!
//! Narrowphase primitives live in `flex_narrow`, self-collision modes in
//! `flex_self`.

use super::flex_narrow::{make_contact_flex_flex, make_contact_flex_self, sphere_triangle_contact};
use super::flex_self::collide_element_pair;
use crate::mid_phase::{Bvh, BvhPrimitive};
use crate::types::{Data, Model};
use cf_geometry::Aabb;
use nalgebra::{Point3, Vector3};

// Re-export public items so existing import paths in collision/mod.rs continue
// to work without modification.
pub use super::flex_narrow::{make_contact_flex_rigid, narrowphase_sphere_geom};
pub use super::flex_self::{
    mj_collide_flex_self_bvh, mj_collide_flex_self_narrow, mj_collide_flex_self_sap,
};

// =============================================================================
// S6: Internal collision (adjacent elements)
// =============================================================================

/// Internal collision: contacts between adjacent elements within a flex.
///
/// For each adjacent element pair, tests non-shared vertices of each element
/// against faces of the other element. Generates vertex-face contacts where
/// the vertex sphere penetrates the opposing face within margin.
pub fn mj_collide_flex_internal(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 {
        return;
    }

    // Iterate unique adjacent pairs
    for local_e in 0..elem_count {
        let e1 = elem_start + local_e;
        let adr = model.flex_elem_adj_adr[e1];
        let num = model.flex_elem_adj_num[e1];
        for &e2 in &model.flex_elem_adj[adr..adr + num] {
            // Only process each pair once (e1 < e2)
            if e2 <= e1 {
                continue;
            }
            // Both elements must belong to the same flex
            if model.flexelem_flexid[e2] != f {
                continue;
            }

            // Get vertex sets for both elements
            let verts1 = elem_vertices(model, e1);
            let verts2 = elem_vertices(model, e2);

            // Test non-shared vertices of e1 against faces of e2
            for &v in &verts1 {
                if !verts2.contains(&v) {
                    test_vertex_against_element_faces(model, data, v, &verts2, dim, f, f);
                }
            }
            // Test non-shared vertices of e2 against faces of e1
            for &v in &verts2 {
                if !verts1.contains(&v) {
                    test_vertex_against_element_faces(model, data, v, &verts1, dim, f, f);
                }
            }
        }
    }
}

/// Get vertex indices for an element.
pub(super) fn elem_vertices(model: &Model, elem: usize) -> Vec<usize> {
    let adr = model.flexelem_dataadr[elem];
    let num = model.flexelem_datanum[elem];
    model.flexelem_data[adr..adr + num].to_vec()
}

/// Test a vertex sphere against the faces of an element.
///
/// For dim=2 (triangle): one face (the triangle itself).
/// For dim=3 (tetrahedron): four triangular faces.
/// For dim=1 (edge): skip (no faces for vertex-face test).
///
/// `flex_id_vertex`: flex ID of the vertex being tested.
/// `flex_id_elem`: flex ID of the element whose faces are tested against.
/// When `flex_id_vertex == flex_id_elem`, produces self-collision contacts;
/// when different, produces flex-flex contacts.
pub(super) fn test_vertex_against_element_faces(
    model: &Model,
    data: &mut Data,
    vertex: usize,
    elem_verts: &[usize],
    dim: usize,
    flex_id_vertex: usize,
    flex_id_elem: usize,
) {
    let vpos = data.flexvert_xpos[vertex];
    let radius = model.flexvert_radius[vertex];
    let margin = model.flex_margin[flex_id_vertex] + model.flex_margin[flex_id_elem];
    let gap = model.flex_gap[flex_id_vertex] + model.flex_gap[flex_id_elem];
    let includemargin = margin - gap;

    match dim {
        2 => {
            // Triangle face: elem_verts = [v0, v1, v2]
            if elem_verts.len() < 3 {
                return;
            }
            let tri = [
                data.flexvert_xpos[elem_verts[0]],
                data.flexvert_xpos[elem_verts[1]],
                data.flexvert_xpos[elem_verts[2]],
            ];
            if let Some((depth, normal, contact_pos)) = sphere_triangle_contact(vpos, radius, &tri)
            {
                if depth > -includemargin {
                    let nearest = nearest_vertex(vpos, elem_verts, data);
                    let contact = if flex_id_vertex == flex_id_elem {
                        make_contact_flex_self(model, vertex, nearest, contact_pos, normal, depth)
                    } else {
                        make_contact_flex_flex(model, vertex, nearest, contact_pos, normal, depth)
                    };
                    data.contacts.push(contact);
                    data.ncon += 1;
                }
            }
        }
        3 => {
            // Tetrahedron: 4 triangular faces
            if elem_verts.len() < 4 {
                return;
            }
            let faces = [
                [elem_verts[0], elem_verts[1], elem_verts[2]],
                [elem_verts[0], elem_verts[1], elem_verts[3]],
                [elem_verts[0], elem_verts[2], elem_verts[3]],
                [elem_verts[1], elem_verts[2], elem_verts[3]],
            ];
            for face in &faces {
                let tri = [
                    data.flexvert_xpos[face[0]],
                    data.flexvert_xpos[face[1]],
                    data.flexvert_xpos[face[2]],
                ];
                if let Some((depth, normal, contact_pos)) =
                    sphere_triangle_contact(vpos, radius, &tri)
                {
                    if depth > -includemargin {
                        let nearest = nearest_vertex(vpos, face, data);
                        let contact = if flex_id_vertex == flex_id_elem {
                            make_contact_flex_self(
                                model,
                                vertex,
                                nearest,
                                contact_pos,
                                normal,
                                depth,
                            )
                        } else {
                            make_contact_flex_flex(
                                model,
                                vertex,
                                nearest,
                                contact_pos,
                                normal,
                                depth,
                            )
                        };
                        data.contacts.push(contact);
                        data.ncon += 1;
                    }
                }
            }
        }
        _ => {} // dim=1: no faces for vertex-face test
    }
}

/// Find the nearest vertex (by Euclidean distance) from a set.
///
/// Returns `candidates[0]` if the set is empty (caller guarantees non-empty).
pub(super) fn nearest_vertex(pos: Vector3<f64>, candidates: &[usize], data: &Data) -> usize {
    candidates
        .iter()
        .copied()
        .min_by(|&a, &b| {
            let da = (data.flexvert_xpos[a] - pos).norm_squared();
            let db = (data.flexvert_xpos[b] - pos).norm_squared();
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(candidates[0])
}

// =============================================================================
// Flex-flex cross-object collision (Spec D)
// =============================================================================

/// Flex-flex element pair enumeration with BVH midphase.
///
/// Builds BVH from the larger flex's elements, queries with each element
/// of the smaller flex. No adjacency filtering (cross-object elements
/// cannot be adjacent). Dispatches to `collide_element_pair()` for each
/// AABB-overlapping pair.
pub fn mj_collide_flex_pair(model: &Model, data: &mut Data, f1: usize, f2: usize) {
    let elem_start1 = model.flex_elemadr[f1];
    let elem_count1 = model.flex_elemnum[f1];
    let elem_start2 = model.flex_elemadr[f2];
    let elem_count2 = model.flex_elemnum[f2];

    if elem_count1 == 0 || elem_count2 == 0 {
        return;
    }

    // Mixed dimensions: use the minimum dim for narrowphase dispatch.
    let dim1 = model.flex_dim[f1];
    let dim2 = model.flex_dim[f2];
    let dim = dim1.min(dim2);

    // Build BVH from the flex with more elements (better tree balance)
    let (bvh_start, bvh_count, query_start, query_count, bvh_flex, query_flex) =
        if elem_count2 >= elem_count1 {
            (elem_start2, elem_count2, elem_start1, elem_count1, f2, f1)
        } else {
            (elem_start1, elem_count1, elem_start2, elem_count2, f1, f2)
        };

    // Build BVH from larger flex's elements
    let primitives: Vec<BvhPrimitive> = (0..bvh_count)
        .map(|i| {
            let e = bvh_start + i;
            let verts = elem_vertices(model, e);
            let aabb = element_aabb(&verts, data);
            BvhPrimitive {
                aabb,
                index: i,
                data: 0,
            }
        })
        .collect();

    let bvh = Bvh::build(primitives);

    // Query each element of smaller flex against BVH
    for i in 0..query_count {
        let e_query = query_start + i;
        let verts_query = elem_vertices(model, e_query);
        let aabb_query = element_aabb(&verts_query, data);

        for candidate_idx in bvh.query(&aabb_query) {
            let e_bvh = bvh_start + candidate_idx;

            // No adjacency filtering — cross-object elements are never adjacent
            // No duplicate filtering — each (query, bvh) pair is unique

            collide_element_pair(model, data, e_query, e_bvh, dim, query_flex, bvh_flex);
        }
    }
}

/// Compute AABB for an element from its vertex positions.
fn element_aabb(verts: &[usize], data: &Data) -> Aabb {
    let first = data.flexvert_xpos[verts[0]];
    let mut min_v = first;
    let mut max_v = first;

    for &v in &verts[1..] {
        let p = data.flexvert_xpos[v];
        min_v = Vector3::new(min_v.x.min(p.x), min_v.y.min(p.y), min_v.z.min(p.z));
        max_v = Vector3::new(max_v.x.max(p.x), max_v.y.max(p.y), max_v.z.max(p.z));
    }

    Aabb {
        min: Point3::from(min_v),
        max: Point3::from(max_v),
    }
}

// =============================================================================
// Spec E tests — flex-vs-complex-geom narrowphase
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod spec_e_tests {
    use super::super::flex_narrow::narrowphase_sphere_geom;
    use crate::heightfield::HeightFieldData;
    use crate::mesh::TriangleMeshData;
    use crate::sdf::SdfCollisionData;
    use crate::types::{GeomType, Model};
    use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    /// Build a minimal Model with one geom at the given type and identity pose.
    fn make_single_geom_model(geom_type: GeomType) -> Model {
        let mut model = Model::empty();
        model.ngeom = 1;
        model.geom_type = vec![geom_type];
        model.geom_body = vec![0];
        model.geom_pos = vec![Vector3::zeros()];
        model.geom_quat = vec![UnitQuaternion::identity()];
        model.geom_size = vec![Vector3::new(0.5, 0.5, 0.5)];
        model.geom_mesh = vec![None];
        model.geom_hfield = vec![None];
        model.geom_sdf = vec![None];
        model.geom_contype = vec![1];
        model.geom_conaffinity = vec![1];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001)];
        model.geom_condim = vec![3];
        model.geom_solref = vec![[0.02, 1.0]];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.geom_priority = vec![0];
        model.geom_solmix = vec![1.0];
        model.geom_margin = vec![0.0];
        model.geom_gap = vec![0.0];
        model.geom_group = vec![0];
        model.geom_rgba = vec![[1.0; 4]];
        model.geom_fluid = vec![[0.0; 12]];
        model.geom_name = vec![None];
        model.geom_rbound = vec![1.0];
        model
    }

    /// Unit cube mesh (half-size 0.5) with convex hull.
    fn cube_mesh_with_hull() -> Arc<TriangleMeshData> {
        let h = 0.5;
        let vertices = vec![
            Point3::new(-h, -h, -h),
            Point3::new(-h, -h, h),
            Point3::new(h, -h, h),
            Point3::new(h, -h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, h, h),
            Point3::new(h, h, h),
            Point3::new(h, h, -h),
        ];
        let indices = vec![
            0, 1, 2, 0, 2, 3, // -Y face
            4, 6, 5, 4, 7, 6, // +Y face
            0, 4, 5, 0, 5, 1, // -X face
            3, 2, 6, 3, 6, 7, // +X face
            0, 3, 7, 0, 7, 4, // -Z face
            1, 5, 6, 1, 6, 2, // +Z face
        ];
        let mut mesh = TriangleMeshData::new(vertices, indices);
        mesh.compute_convex_hull(None);
        Arc::new(mesh)
    }

    /// Single-triangle mesh (no convex hull).
    fn single_triangle_mesh() -> Arc<TriangleMeshData> {
        let vertices = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let indices = vec![0, 1, 2];
        Arc::new(TriangleMeshData::new(vertices, indices))
    }

    // --- T1: Flex vertex sphere vs mesh with convex hull (AC1) ---

    #[test]
    fn t1_flex_vs_mesh_with_hull() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        let mesh = cube_mesh_with_hull();
        assert!(mesh.convex_hull().is_some());
        model.mesh_data = vec![mesh];
        model.geom_mesh = vec![Some(0)];

        // Vertex at z=0.45, radius=0.1. Sphere bottom at 0.35, hull top at 0.5.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.10 && depth < 0.20,
            "depth={depth}, expected ~0.15"
        );
        assert!(normal.z > 0.5, "normal.z={}, expected >0.5", normal.z);
    }

    // --- T2: Flex vertex sphere vs heightfield (AC2) ---

    #[test]
    fn t2_flex_vs_hfield() {
        let mut model = make_single_geom_model(GeomType::Hfield);

        // 5×5 flat heightfield at height=0.5, half-extent 2.0
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex at z=0.45, radius=0.1. Sphere bottom at 0.35, terrain at 0.5.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.05 && depth < 0.25,
            "depth={depth}, expected ~0.15"
        );
        assert!(normal.z > 0.9, "normal.z={}, expected ~1.0", normal.z);
    }

    // --- T3: Flex vertex sphere vs SDF (AC3) ---

    #[test]
    fn t3_flex_vs_sdf() {
        let mut model = make_single_geom_model(GeomType::Sdf);

        // Sphere SDF, radius 1.0, centered at origin, 32 resolution
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];

        // Vertex at z=0.95, radius=0.1. Distance to SDF surface = 0.05.
        // Penetration = 0.1 - 0.05 = 0.05.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.95),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.01 && depth < 0.15,
            "depth={depth}, expected ~0.05"
        );
        assert!(normal.z > 0.5, "normal.z={}, expected >0.5", normal.z);
    }

    // --- T4: Missing geom data guards (AC4, AC5, AC6) ---

    #[test]
    fn t4_missing_data_no_panic() {
        // Mesh with no data
        let model_mesh = make_single_geom_model(GeomType::Mesh);
        assert!(model_mesh.geom_mesh[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_mesh,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Mesh with no data should return None");

        // Hfield with no data
        let model_hfield = make_single_geom_model(GeomType::Hfield);
        assert!(model_hfield.geom_hfield[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_hfield,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Hfield with no data should return None");

        // SDF with no data
        let model_sdf = make_single_geom_model(GeomType::Sdf);
        assert!(model_sdf.geom_sdf[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_sdf,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "SDF with no data should return None");
    }

    // --- T5: Vertex above surface — no contact (AC7) ---

    #[test]
    fn t5_above_surface_no_contact() {
        // Mesh: vertex well above
        let mut model = make_single_geom_model(GeomType::Mesh);
        model.mesh_data = vec![cube_mesh_with_hull()];
        model.geom_mesh = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above mesh should return None");

        // Hfield: vertex well above
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above hfield should return None");

        // SDF: vertex well above
        let mut model = make_single_geom_model(GeomType::Sdf);
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above SDF should return None");
    }

    // --- T6: Per-triangle BVH fallback (AC10) ---

    #[test]
    fn t6_flex_vs_mesh_per_triangle_fallback() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        let mesh = single_triangle_mesh();
        assert!(
            mesh.convex_hull().is_none(),
            "No hull on single-triangle mesh"
        );
        model.mesh_data = vec![mesh];
        model.geom_mesh = vec![Some(0)];

        // Triangle in Z=0 plane. Vertex at z=-0.05, radius=0.1.
        // Sphere top at z=0.05, triangle at z=0.0 → penetrating.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, -0.05),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, _normal, _contact_pos) = result.expect("BVH fallback should generate contact");
        assert!(depth > 0.0, "depth={depth}, expected >0");
    }

    // --- TS1: Zero-radius vertex vs mesh ---

    #[test]
    fn ts1_zero_radius_vs_mesh() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        model.mesh_data = vec![cube_mesh_with_hull()];
        model.geom_mesh = vec![Some(0)];

        // Zero-radius sphere inside cube → should not panic
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        // Zero radius means no penetration extent — result may be None or Some
        // depending on GJK/EPA; the key requirement is no panic.
        let _ = result;
    }

    // --- TS2: Hfield vertex on surface boundary ---

    #[test]
    fn ts2_hfield_vertex_on_surface() {
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex at z=0.55, radius=0.05 → sphere bottom at 0.50 = terrain height
        // Boundary: penetration ≈ 0. Should not panic regardless of result.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.55),
            0.05,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        let _ = result; // No panic is the key requirement
    }

    // --- TS3: SDF vertex outside bounding box ---

    #[test]
    fn ts3_sdf_vertex_outside_bbox() {
        let mut model = make_single_geom_model(GeomType::Sdf);
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];

        // Vertex far outside SDF bounding box
        let result = narrowphase_sphere_geom(
            Vector3::new(100.0, 100.0, 100.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(
            result.is_none(),
            "Vertex outside SDF bbox should return None"
        );
    }

    // --- TS4: Single-cell (2×2) hfield ---

    #[test]
    fn ts4_single_cell_hfield() {
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 2; // Minimum valid hfield: 2×2 grid = 1 cell
        let hf_size: [f64; 4] = [1.0, 1.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex penetrating
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact on 2×2 hfield");
        assert!(depth > 0.0, "depth={depth}");
        assert!(normal.z > 0.5, "normal.z={}", normal.z);
    }
}
