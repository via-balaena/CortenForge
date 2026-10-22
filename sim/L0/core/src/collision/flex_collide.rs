//! Flex collision dispatcher — internal collision (adjacent elements).
//!
//! Narrowphase primitives live in `flex_narrow`, self-collision modes in
//! `flex_self`.

use super::flex_narrow::{make_contact_flex_flex, make_contact_flex_self, sphere_triangle_contact};
use crate::types::{Data, Model};
use nalgebra::Vector3;

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
