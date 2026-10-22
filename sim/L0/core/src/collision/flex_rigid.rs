//! A body's geoms against a flex, as MuJoCo collides them
//! (`engine_collision_driver.c:346-412`, `:1658-1710`, `:1866-1998`).
//!
//! With the midphase, the body's hierarchy and the flex's are descended
//! together ([`collide_tree_flex`]), a world-welded body's planes first
//! against every vertex; without it, each geom the bitmask passes against
//! every element, a plane against every vertex. A geom meets an element by
//! type: a sphere, a capsule or a box against a cable's element through the
//! raw capsule functions, against a triangle through the raw triangle ones,
//! a height field through its prisms, any other through the convex route.
//! Each contact lists the geom first and the flex's element or vertex
//! second, its normal from the geom toward the flex.
//!
//! A signed distance field is the exception (registry `D-FLEX-SDF`): MuJoCo
//! collides its mesh's hull with each element through the convex route;
//! sim-core's distance field has no hull, so each vertex, a sphere of the
//! flex's radius, is tested against the field, as the plane's vertices are.

use nalgebra::{Point3, Vector3};
use sim_types::Pose;

use super::broadphase::BodyGeoms;
use super::midphase::{collide_tree_flex, filter_sphere_box};
use super::mj_ccd::{CcdObj, mjc_convex_elem, mjc_hfield_elem};
use super::mj_primitive::{
    Frame, Mat9, RawContact, raw_capsule_box, raw_capsule_capsule, raw_sphere_capsule,
};
use super::mj_triangle::{
    make_capsule, raw_box_triangle, raw_capsule_triangle, raw_sphere_triangle,
};
use super::order::{ListPlace, flex_place};
use super::{
    assign_friction, assign_imp, assign_margin, assign_ref, contact_param_geom_flex,
    finite_rotation,
};
use crate::sdf::sdf_sphere_contact;
use crate::types::{Contact, DISABLE_MIDPHASE, Data, GeomType, Model, disabled, make_frame};

/// Collide body `body`'s geoms with flex `flex`, a pair the broadphase
/// passed, and list each contact's place (`mj_collision`,
/// `engine_collision_driver.c:346-412`): through the midphase when it is on
/// and both have hierarchies, else each geom in turn. `hits` and `stack` are
/// scratch, reused across pairs.
pub(super) fn collide_body_flex(
    model: &Model,
    data: &mut Data,
    geoms: &BodyGeoms,
    places: &mut Vec<ListPlace>,
    (body, flex): (usize, usize),
    (hits, stack): (&mut Vec<(usize, usize)>, &mut Vec<(usize, usize)>),
) {
    let start = data.contacts.len();
    hits.clear();
    if !disabled(model, DISABLE_MIDPHASE) && collide_tree_flex(model, data, body, flex, hits, stack)
    {
        // `mj_collideTree`'s planes, before its descent and without the
        // geom's bitmask (`:658-665`), and the distance fields.
        for &g in geoms.of(body) {
            match model.geom_type[g] {
                GeomType::Plane if model.body_weldid[body] == 0 => {
                    collide_plane_flex(model, data, g, flex);
                }
                GeomType::Sdf if !filter_bitmask(model, g, flex) => {
                    collide_sdf_flex(model, data, g, flex);
                }
                _ => {}
            }
        }
        for &(g, e) in hits.iter() {
            collide_geom_elem(model, data, g, flex, e);
        }
    } else {
        for &g in geoms.of(body) {
            if filter_bitmask(model, g, flex) {
                continue;
            }
            match model.geom_type[g] {
                GeomType::Plane => collide_plane_flex(model, data, g, flex),
                GeomType::Sdf => collide_sdf_flex(model, data, g, flex),
                _ => {
                    for e in 0..model.flex_elemnum[flex] {
                        collide_geom_elem(model, data, g, flex, e);
                    }
                }
            }
        }
    }
    // MuJoCo sorts a midphase pair's contacts by geom, then element or
    // vertex (`contactcompare`, `:226-256`, stably), the order taken in
    // turn already has; `flex_place` keys each contact so.
    for contact in &data.contacts[start..] {
        places.push(flex_place(model, contact));
    }
    data.ncon = data.contacts.len();
}

/// `filterBitmask` for geom `g` and flex `f` (`:99-102`): whether neither's
/// type meets the other's affinity.
pub(super) fn filter_bitmask(model: &Model, g: usize, f: usize) -> bool {
    model.geom_contype[g] & model.flex_conaffinity[f] == 0
        && model.flex_contype[f] & model.geom_conaffinity[g] == 0
}

/// `mj_collidePlaneFlex` (`:1658-1710`): each vertex of flex `f` within
/// the margin and the flex's radius of plane `g`, its distance from the
/// plane less the radius, at its position moved toward the plane by half
/// that and the radius, the plane's normal.
fn collide_plane_flex(model: &Model, data: &mut Data, g: usize, f: usize) {
    let radius = model.flex_radius[f];
    let pos = data.geom_xpos[g];
    let nrm = data.geom_xmat[g].column(2).into_owned();
    let margin = assign_margin(model, model.geom_margin[g] + model.flex_margin[f]);
    let adr = model.flex_vertadr[f];
    for i in 0..model.flex_vertnum[f] {
        let v = data.flexvert_xpos[adr + i];
        let dif = v - pos;
        let dist = dif.x * nrm.x + dif.y * nrm.y + dif.z * nrm.z;
        if dist > margin + radius {
            continue;
        }
        let dist = dist - radius;
        let scl = -dist * 0.5 - radius;
        let raw = RawContact {
            dist,
            pos: [v.x + scl * nrm.x, v.y + scl * nrm.y, v.z + scl * nrm.z],
            normal: [nrm.x, nrm.y, nrm.z],
        };
        let contact = geom_flex_contact(model, &raw, (g, f), (None, Some(i)), margin);
        data.contacts.push(contact);
    }
}

/// `mj_collideGeomElem` (`:1866-1998`): geom `g` against element `e` of
/// flex `f`. Without the midphase, nothing where the geom's bounding sphere
/// grown by the margin is clear of the element's box; nothing where a
/// vertex of the element is on the geom's body.
fn collide_geom_elem(model: &Model, data: &mut Data, g: usize, f: usize, e: usize) {
    let margin = assign_margin(model, model.geom_margin[g] + model.flex_margin[f]);
    let eglobal = model.flex_elemadr[f] + e;
    let pos = data.geom_xpos[g];
    if disabled(model, DISABLE_MIDPHASE)
        && filter_sphere_box(
            &pos,
            model.geom_rbound[g] + margin,
            &data.flexelem_aabb[eglobal],
        )
    {
        return;
    }
    let dim = model.flex_dim[f];
    let adr = model.flexelem_dataadr[eglobal];
    let verts = &model.flexelem_data[adr..=adr + dim];
    let body = model.geom_body[g];
    if verts.iter().any(|&v| model.flexvert_bodyid[v] == body) {
        return;
    }
    let vertices: Vec<[f64; 3]> = verts
        .iter()
        .map(|&v| {
            let x = data.flexvert_xpos[v];
            [x.x, x.y, x.z]
        })
        .collect();
    let raws = geom_elem_raw(model, data, (g, f, e), &vertices, margin);
    for raw in &raws {
        let contact = geom_flex_contact(model, raw, (g, f), (Some(e), None), margin);
        data.contacts.push(contact);
    }
}

/// The contacts of geom `g` and element `e` of flex `f` (its `vertices`),
/// by the geom's type and the flex's dimension
/// (`engine_collision_driver.c:1898-1966`), each from the geom toward the
/// element.
fn geom_elem_raw(
    model: &Model,
    data: &Data,
    (g, f, e): (usize, usize, usize),
    vertices: &[[f64; 3]],
    margin: f64,
) -> Vec<RawContact> {
    let pos = data.geom_xpos[g];
    let mat = data.geom_xmat[g];
    let size = model.geom_size[g];
    let radius = model.flex_radius[f];
    let mat9: Mat9 = std::array::from_fn(|i| mat[(i / 3, i % 3)]);
    let geom_type = model.geom_type[g];
    let p = [pos.x, pos.y, pos.z];
    match (geom_type, vertices) {
        (GeomType::Sphere | GeomType::Capsule | GeomType::Box, [v1, v2]) => {
            let (cpos, cmat, csize) = make_capsule(v1, v2, radius);
            let geom = Frame {
                pos: &pos,
                mat: &mat,
            };
            let capsule = Frame {
                pos: &cpos,
                mat: &cmat,
            };
            match geom_type {
                GeomType::Sphere => raw_sphere_capsule(margin, geom, size.x, capsule, &csize)
                    .into_iter()
                    .collect(),
                GeomType::Capsule => raw_capsule_capsule(margin, geom, &size, capsule, &csize),
                // The box second: its normals turned round (`:1916-1924`).
                _ => raw_capsule_box(margin, capsule, &csize, geom, &size)
                    .into_iter()
                    .map(|c| RawContact {
                        normal: [-c.normal[0], -c.normal[1], -c.normal[2]],
                        ..c
                    })
                    .collect(),
            }
        }
        (GeomType::Hfield, _) => {
            let Some(id) = model.geom_hfield[g] else {
                return Vec::new();
            };
            let center: [f64; 3] =
                std::array::from_fn(|k| data.flexelem_aabb[model.flex_elemadr[f] + e][k]);
            mjc_hfield_elem(
                &model.hfield_data[id],
                &model.hfield_size[id],
                &p,
                &mat9,
                vertices,
                &center,
                radius,
                margin,
                (model.ccd_iterations, model.ccd_tolerance),
            )
        }
        (GeomType::Sphere, [t1, t2, t3]) => {
            raw_sphere_triangle(margin, &p, size.x, [t1, t2, t3], radius)
                .into_iter()
                .collect()
        }
        (GeomType::Box, [t1, t2, t3]) => raw_box_triangle(
            margin,
            &p,
            &mat9,
            &[size.x, size.y, size.z],
            [t1, t2, t3],
            radius,
        ),
        (GeomType::Capsule, [t1, t2, t3]) => {
            raw_capsule_triangle(margin, &p, &mat9, [size.x, size.y], [t1, t2, t3], radius)
        }
        _ => {
            let Some(mut geom) = CcdObj::of_geom(model, g, &pos, &mat, margin) else {
                return Vec::new();
            };
            let center: [f64; 3] =
                std::array::from_fn(|k| data.flexelem_aabb[model.flex_elemadr[f] + e][k]);
            let mut elem = CcdObj::flex(vertices, center, radius, margin);
            let fix =
                (model.flex_dim[f] == 2).then_some((geom_type, p, &mat9, [size.x, size.y, size.z]));
            mjc_convex_elem(model, &mut geom, &mut elem, margin, fix)
        }
    }
}

/// Each vertex of flex `f`, a sphere of the flex's radius, against signed
/// distance field `g`, within the margin: sim-core's route where MuJoCo
/// takes the field's mesh hull through the convex route (registry
/// `D-FLEX-SDF`). The contact is on the field's surface, its normal the
/// field's gradient there, its distance the vertex's from the surface less
/// the radius; nothing for a vertex on the geom's body.
fn collide_sdf_flex(model: &Model, data: &mut Data, g: usize, f: usize) {
    let Some(shape) = model.geom_shape[g] else {
        return;
    };
    let sdf = model.shape_data[shape].sdf_grid();
    let Some(rotation) = finite_rotation(&data.geom_xmat[g]) else {
        return;
    };
    let pose = Pose::from_position_rotation(Point3::from(data.geom_xpos[g]), rotation);
    let margin = assign_margin(model, model.geom_margin[g] + model.flex_margin[f]);
    let radius = model.flex_radius[f];
    let adr = model.flex_vertadr[f];
    for i in 0..model.flex_vertnum[f] {
        if model.flexvert_bodyid[adr + i] == model.geom_body[g] {
            continue;
        }
        let v = data.flexvert_xpos[adr + i];
        let Some(hit) = sdf_sphere_contact(sdf, &pose, Point3::from(v), radius + margin) else {
            continue;
        };
        let raw = RawContact {
            dist: margin - hit.penetration,
            pos: [hit.point.x, hit.point.y, hit.point.z],
            normal: [hit.normal.x, hit.normal.y, hit.normal.z],
        };
        let contact = geom_flex_contact(model, &raw, (g, f), (None, Some(i)), margin);
        data.contacts.push(contact);
    }
}

/// A contact of geom `g`, side 0, and flex `f`, side 1, at its element or
/// vertex `(elem, vert)` (flex-local), with MuJoCo's parameters
/// (`mj_contactParam(g, -1, -1, f)`, `mj_setContact`, `:1305-1428`): the
/// geom's and the flex's combined, the margin less their gaps included.
fn geom_flex_contact(
    model: &Model,
    raw: &RawContact,
    (g, f): (usize, usize),
    (elem, vert): (Option<usize>, Option<usize>),
    margin: f64,
) -> Contact {
    let (condim, gap, solref, solimp, mu) = contact_param_geom_flex(model, g, f);
    let mu = assign_friction(model, &mu);
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
        geom: [Some(g), None],
        flex: [None, Some(f)],
        elem: [None, elem],
        vert: [None, vert],
        friction: mu[0],
        dim,
        includemargin: margin - gap,
        mu,
        solref: assign_ref(model, &solref),
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp: assign_imp(model, &solimp),
        frame,
        regularized_mu: 0.0,
    }
}
