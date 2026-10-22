//! A flex against a flex, and against itself, as MuJoCo collides them
//! (`engine_collision_driver.c:346-412`, `:428-475`, `:1809-1861`,
//! `:2002-2098`).
//!
//! Two flexes: with the midphase, their hierarchies are descended together
//! ([`collide_tree_flexes`]), each pair of leaves to the narrow phase;
//! without it, every element of the first against every element of the
//! second. A flex against itself, after every pair: its active elements
//! through its own hierarchy, by sweep and prune, or each pair in turn, as
//! its `selfcollide` and the midphase say. Two elements meet unless their
//! boxes are further apart than the margin (none against itself) or a vertex
//! of one is on a body a vertex of the other is on: two cables' elements as
//! capsules, any other two through the convex route. Each contact lists the
//! first element first, its normal toward the second.

use nalgebra::Vector3;

use nalgebra::Matrix3;

use super::broadphase::sweep_and_prune;
use super::midphase::{collide_tree_flexes, filter_box};
use super::mj_ccd::{CcdObj, mjc_convex_elem};
use super::mj_primitive::{Frame, RawContact, raw_capsule_capsule, raw_sphere_capsule};
use super::mj_triangle::{make_capsule, plane_vertex, raw_sphere_triangle};
use super::order::{ListPlace, flex_place};
use super::{
    ContactParams, assign_friction, assign_imp, assign_margin, assign_ref, contact_param_flexes,
};
use crate::types::{Contact, DISABLE_MIDPHASE, Data, FlexSelfCollide, Model, disabled, make_frame};

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

/// Collide flex `f` with itself and list each contact's place (`mj_collision`,
/// `engine_collision_driver.c:428-475`): a flex not rigid whose contype
/// meets its own conaffinity, its internal collisions when `flex_internal`
/// is set, then, unless its `selfcollide` is none, its active elements
/// against each other: with the midphase and a hierarchy, through the
/// hierarchy (`bvh`, or `auto` for a tetrahedral flex) or by sweep and prune
/// (`sap`, or `auto` otherwise); else (`narrow`, or without the midphase)
/// each pair in turn. The contacts keep the order made. `hits` and `stack`
/// are scratch.
pub(super) fn collide_flex_self(
    model: &Model,
    data: &mut Data,
    places: &mut Vec<ListPlace>,
    f: usize,
    (hits, stack): (&mut Vec<(usize, usize)>, &mut Vec<(usize, usize)>),
) {
    if model.flex_rigid[f] || model.flex_contype[f] & model.flex_conaffinity[f] == 0 {
        return;
    }
    let start = data.contacts.len();
    if model.flex_internal[f] {
        collide_flex_internal(model, data, f);
    }
    if model.flex_selfcollide[f] != FlexSelfCollide::None {
        collide_active_elements(model, data, f, (hits, stack));
    }
    for contact in &data.contacts[start..] {
        places.push(flex_place(model, contact));
    }
    data.ncon = data.contacts.len();
}

/// `mj_collideFlexInternal` (`:1745-1806`): flex `f`'s element–vertex pairs
/// (`flex_evpair`: each border element of a cable or a cloth and the vertex
/// of its neighbour across a shared fragment), each to [`collide_elem_vert`];
/// then, for a tetrahedral flex, each tetrahedron's four faces against their
/// opposite vertices ([`plane_vertex`]), each contact the element's first,
/// condim 1, the margin and gap left out.
fn collide_flex_internal(model: &Model, data: &mut Data, f: usize) {
    let adr = model.flex_evpairadr[f];
    for &[e, v] in &model.flex_evpair[adr..adr + model.flex_evpairnum[f]] {
        collide_elem_vert(model, data, f, e, v);
    }
    if model.flex_dim[f] != 3 {
        return;
    }
    let params = contact_param_flexes(model, f, f);
    let radius = model.flex_radius[f];
    let vertadr = model.flex_vertadr[f];
    for e in 0..model.flex_elemnum[f] {
        let global = model.flex_elemadr[f] + e;
        let verts = elem_verts(model, global);
        let x: Vec<[f64; 3]> = positions(data, verts);
        for [t0, t1, t2, v] in [[0, 1, 2, 3], [0, 2, 3, 1], [0, 3, 1, 2], [1, 3, 2, 0]] {
            let Some(raw) = plane_vertex([&x[t0], &x[t1], &x[t2]], &x[v], radius) else {
                continue;
            };
            let sides = ([Some(e), None], [None, Some(verts[v] - vertadr)]);
            let mut contact = flex_contact(model, &raw, (f, f), sides, &params, 0.0);
            contact.dim = 1;
            data.contacts.push(contact);
        }
    }
}

/// `mj_collideElemVert` (`:2101-2180`): element `e` of flex `f` against its
/// vertex `v` (flex-local), a sphere of the flex's radius: nothing where the
/// vertex's box, its radius and the margin wide, is clear of the element's;
/// else, at no margin, a cable's element as a capsule, a cloth's as a
/// triangle of the flex's radius. Each contact lists the vertex first, the
/// margin and gap left out. (A tetrahedral flex has no such pairs, its
/// compiler making them for cables and cloths only, `user_mesh.cc:4644-4645`,
/// so MuJoCo's convex route for one, `:2143-2146`, is not ported.)
fn collide_elem_vert(model: &Model, data: &mut Data, f: usize, e: usize, v: usize) {
    let margin = assign_margin(model, model.flex_margin[f]);
    let radius = model.flex_radius[f];
    let vert = data.flexvert_xpos[model.flex_vertadr[f] + v];
    let global = model.flex_elemadr[f] + e;
    let aabb = &data.flexelem_aabb[global];
    let rbound = margin + radius;
    if aabb[0] - aabb[3] > vert.x + rbound
        || aabb[1] - aabb[4] > vert.y + rbound
        || aabb[2] - aabb[5] > vert.z + rbound
        || aabb[0] + aabb[3] < vert.x - rbound
        || aabb[1] + aabb[4] < vert.y - rbound
        || aabb[2] + aabb[5] < vert.z - rbound
    {
        return;
    }
    let x = positions(data, elem_verts(model, global));
    let raws: Vec<RawContact> = match x.as_slice() {
        [v1, v2] => {
            let (pos, mat, size) = make_capsule(v1, v2, radius);
            let sphere = Frame {
                pos: &vert,
                mat: &Matrix3::identity(),
            };
            let capsule = Frame {
                pos: &pos,
                mat: &mat,
            };
            raw_sphere_capsule(0.0, sphere, radius, capsule, &size)
                .into_iter()
                .collect()
        }
        [t1, t2, t3] => {
            raw_sphere_triangle(0.0, &[vert.x, vert.y, vert.z], radius, [t1, t2, t3], radius)
                .into_iter()
                .collect()
        }
        _ => Vec::new(),
    };
    if raws.is_empty() {
        return;
    }
    let params = contact_param_flexes(model, f, f);
    for raw in &raws {
        let sides = ([None, Some(e)], [Some(v), None]);
        data.contacts
            .push(flex_contact(model, raw, (f, f), sides, &params, 0.0));
    }
}

/// Flex `f`'s active elements against each other, by its `selfcollide` mode
/// (`:441-472`).
fn collide_active_elements(
    model: &Model,
    data: &mut Data,
    f: usize,
    (hits, stack): (&mut Vec<(usize, usize)>, &mut Vec<(usize, usize)>),
) {
    let mode = model.flex_selfcollide[f];
    let hierarchy = model.flex_bvhadr.get(f).copied().flatten().is_some();
    if !disabled(model, DISABLE_MIDPHASE) && mode != FlexSelfCollide::Narrow && hierarchy {
        if mode == FlexSelfCollide::Bvh || (mode == FlexSelfCollide::Auto && model.flex_dim[f] == 3)
        {
            hits.clear();
            collide_tree_flexes(model, data, f, f, hits, stack);
            for &(e1, e2) in hits.iter() {
                collide_elems(model, data, (f, e1), (f, e2));
            }
        } else {
            collide_flex_sap(model, data, f);
        }
    } else {
        let active = active_elements(model, f);
        for (i, &e1) in active.iter().enumerate() {
            for &e2 in &active[i + 1..] {
                collide_elems(model, data, (f, e1), (f, e2));
            }
        }
    }
}

/// Flex `f`'s active elements (flex-local), in order.
fn active_elements(model: &Model, f: usize) -> Vec<usize> {
    (0..model.flex_elemnum[f])
        .filter(|&e| model.flex_elem_active(f, e))
        .collect()
}

/// `mj_collideFlexSAP` (`:1809-1861`): flex `f`'s active elements, by their
/// boxes, swept and pruned along the longest axis of its hierarchy's root
/// box (x unless another is longer, then y unless z is), at most 1,000,000
/// pairs, each pair to the narrow phase. (From 65,536 active elements
/// MuJoCo's `mj_SAP` refuses its input and `mj_collideFlexSAP` raises an
/// error; this sweeps them.)
fn collide_flex_sap(model: &Model, data: &mut Data, f: usize) {
    let active = active_elements(model, f);
    let Some(root) = model.flex_bvhadr.get(f).copied().flatten() else {
        return;
    };
    if active.len() < 2 {
        return;
    }
    let adr = model.flex_elemadr[f];
    let aamm: Vec<[f64; 6]> = active
        .iter()
        .map(|&e| {
            let b = &data.flexelem_aabb[adr + e];
            [
                b[0] - b[3],
                b[1] - b[4],
                b[2] - b[5],
                b[0] + b[3],
                b[1] + b[4],
                b[2] + b[5],
            ]
        })
        .collect();
    let r = &data.bvh_aabb_dyn[root - model.nbvhstatic];
    let axis = if r[3] > r[4] && r[3] > r[5] {
        0
    } else if r[4] > r[5] {
        1
    } else {
        2
    };
    let mut pairs = sweep_and_prune(&aamm, axis);
    pairs.truncate(1_000_000);
    for (i, j) in pairs {
        collide_elems(model, data, (f, active[i]), (f, active[j]));
    }
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
