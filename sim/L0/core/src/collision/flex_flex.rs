//! A flex against a flex, as MuJoCo collides them
//! (`engine_collision_driver.c:346-412`, `:2002-2098`).
//!
//! With the midphase, the two flexes' hierarchies are descended together
//! ([`collide_tree_flexes`]), each pair of leaves to the narrow phase;
//! without it, every element of the first against every element of the
//! second. Two elements meet unless their boxes are further apart than the
//! margin or a vertex of one is on a body a vertex of the other is on: two
//! cables' elements as capsules, any other two through the convex route.
//! Each contact lists the first flex's element first, its normal toward the
//! second's.

use nalgebra::Vector3;

use super::midphase::{collide_tree_flexes, filter_box};
use super::mj_ccd::{CcdObj, mjc_convex_elem};
use super::mj_primitive::{Frame, RawContact, raw_capsule_capsule};
use super::mj_triangle::make_capsule;
use super::order::{ListPlace, flex_place};
use super::{
    ContactParams, assign_friction, assign_imp, assign_margin, assign_ref, contact_param_flexes,
};
use crate::types::{Contact, DISABLE_MIDPHASE, Data, Model, disabled, make_frame};

/// Collide flex `flex1` with flex `flex2`, a pair the broadphase passed, and
/// list each contact's place (`mj_collision`,
/// `engine_collision_driver.c:346-412`): through the midphase when it is on
/// and both have hierarchies, else each pair of elements in turn. `hits` and
/// `stack` are scratch, reused across pairs.
pub(super) fn collide_flex_flex(
    model: &Model,
    data: &mut Data,
    places: &mut Vec<ListPlace>,
    (flex1, flex2): (usize, usize),
    (hits, stack): (&mut Vec<(usize, usize)>, &mut Vec<(usize, usize)>),
) {
    let start = data.contacts.len();
    hits.clear();
    if !disabled(model, DISABLE_MIDPHASE)
        && collide_tree_flexes(model, data, flex1, flex2, hits, stack)
    {
        for &(e1, e2) in hits.iter() {
            collide_elems(model, data, (flex1, e1), (flex2, e2));
        }
    } else {
        for e1 in 0..model.flex_elemnum[flex1] {
            for e2 in 0..model.flex_elemnum[flex2] {
                collide_elems(model, data, (flex1, e1), (flex2, e2));
            }
        }
    }
    // MuJoCo sorts a midphase pair's contacts by element (`contactcompare`,
    // `:226-256`, stably), the order taken in turn already has; `flex_place`
    // keys each contact so.
    for contact in &data.contacts[start..] {
        places.push(flex_place(model, contact));
    }
    data.ncon = data.contacts.len();
}

/// `mj_collideElems` (`:2002-2098`): element `e1` of flex `f1` against
/// element `e2` of flex `f2`, a flex against itself without its margin or
/// gap. Nothing where the elements' boxes are further apart than the margin,
/// or a vertex of one is on a body a vertex of the other is on.
pub(super) fn collide_elems(
    model: &Model,
    data: &mut Data,
    (f1, e1): (usize, usize),
    (f2, e2): (usize, usize),
) {
    let margin = if f1 == f2 {
        0.0
    } else {
        assign_margin(model, model.flex_margin[f1] + model.flex_margin[f2])
    };
    let (global1, global2) = (model.flex_elemadr[f1] + e1, model.flex_elemadr[f2] + e2);
    if filter_box(
        &data.flexelem_aabb[global1],
        &data.flexelem_aabb[global2],
        margin,
    ) {
        return;
    }
    let (verts1, verts2) = (elem_verts(model, global1), elem_verts(model, global2));
    let body = |v: usize| model.flexvert_bodyid[v];
    if verts1
        .iter()
        .any(|&v1| verts2.iter().any(|&v2| body(v1) == body(v2)))
    {
        return;
    }
    let (p1, p2) = (positions(data, verts1), positions(data, verts2));
    let raws = if model.flex_dim[f1] == 1 && model.flex_dim[f2] == 1 {
        let (pos1, mat1, size1) = make_capsule(&p1[0], &p1[1], model.flex_radius[f1]);
        let (pos2, mat2, size2) = make_capsule(&p2[0], &p2[1], model.flex_radius[f2]);
        let capsule1 = Frame {
            pos: &pos1,
            mat: &mat1,
        };
        let capsule2 = Frame {
            pos: &pos2,
            mat: &mat2,
        };
        raw_capsule_capsule(margin, capsule1, &size1, capsule2, &size2)
    } else {
        let center =
            |global: usize| -> [f64; 3] { std::array::from_fn(|k| data.flexelem_aabb[global][k]) };
        let mut elem1 = CcdObj::flex(&p1, center(global1), model.flex_radius[f1], margin);
        let mut elem2 = CcdObj::flex(&p2, center(global2), model.flex_radius[f2], margin);
        mjc_convex_elem(model, &mut elem1, &mut elem2, margin, None)
    };
    if raws.is_empty() {
        return;
    }
    let params = contact_param_flexes(model, f1, f2);
    let gap = if f1 == f2 { 0.0 } else { params.1 };
    for raw in &raws {
        let sides = ([Some(e1), Some(e2)], [None, None]);
        let contact = flex_contact(model, raw, (f1, f2), sides, &params, margin - gap);
        data.contacts.push(contact);
    }
}

/// Element `global`'s vertices (global ids).
fn elem_verts(model: &Model, global: usize) -> &[usize] {
    let adr = model.flexelem_dataadr[global];
    &model.flexelem_data[adr..adr + model.flexelem_datanum[global]]
}

/// The positions of vertices `verts`.
fn positions(data: &Data, verts: &[usize]) -> Vec<[f64; 3]> {
    verts
        .iter()
        .map(|&v| {
            let x = data.flexvert_xpos[v];
            [x.x, x.y, x.z]
        })
        .collect()
}

/// A contact of flex `f1`, side 0, and flex `f2`, side 1, at each side's
/// element or vertex `(elem, vert)` (flex-local), with `params`
/// (`mj_contactParam(-1, -1, f1, f2)`) and `includemargin`, as
/// `mj_setContact` sets it (`:1402-1428`).
fn flex_contact(
    model: &Model,
    raw: &RawContact,
    (f1, f2): (usize, usize),
    (elem, vert): ([Option<usize>; 2], [Option<usize>; 2]),
    (condim, _, solref, solimp, mu): &ContactParams,
    includemargin: f64,
) -> Contact {
    let mu = assign_friction(model, mu);
    let dim = match condim {
        1 => 1,
        4 => 4,
        6 => 6,
        _ => 3,
    };
    let (normal, frame) = make_frame(&Vector3::from(raw.normal), &Vector3::zeros());
    Contact {
        pos: Vector3::from(raw.pos),
        normal,
        depth: -raw.dist,
        geom: [None, None],
        flex: [Some(f1), Some(f2)],
        elem,
        vert,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref: assign_ref(model, solref),
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp: assign_imp(model, solimp),
        frame,
        regularized_mu: 0.0,
    }
}
