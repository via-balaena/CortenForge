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
                && depth > -includemargin
            {
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
                    && depth > -includemargin
                {
                    let nearest = nearest_vertex(vpos, face, data);
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
