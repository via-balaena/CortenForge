//! MuJoCo's collision functions against a flex element, ported statement for
//! statement: a sphere, a box and a capsule against a triangle with a
//! radius (`mjraw_SphereTriangle`, `mjraw_BoxTriangle`,
//! `mjraw_CapsuleTriangle`, `engine_collision_primitive.c:490-766`), and the
//! capsule a cable's element is (`mj_makeCapsule`,
//! `engine_collision_driver.c:1432-1449`), which the sphere, capsule and box
//! functions of `mj_primitive` meet. Each contact's normal points from the
//! geom to the element.

use nalgebra::{Matrix3, Vector3};

use super::mj_primitive::{
    MAX_CONPAIR, Mat9, RawContact, add3, dot3, mat_t_vec, mat_vec, normalize3, scl3, sub3,
};
use crate::constraint::impedance::MJ_MINVAL;
use crate::linalg::mju_dot;
use crate::quat::{quat_to_mat, quat_z2vec};

/// `mji_cross`.
fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `mji_addScl3`: `v1 + v2 · s`.
fn add_scl(v1: &[f64; 3], v2: &[f64; 3], s: f64) -> [f64; 3] {
    [v1[0] + v2[0] * s, v1[1] + v2[1] * s, v1[2] + v2[2] * s]
}

/// `mju_sign` (`engine_util_misc.c:1318-1326`): −1, 1, or 0 (NaN too).
fn sign(x: f64) -> f64 {
    if x < 0.0 {
        -1.0
    } else if x > 0.0 {
        1.0
    } else {
        0.0
    }
}

/// `areaSign` (`engine_collision_primitive.c:493-495`): the sign of the
/// planar triangle `(p1, p2, p3)`'s signed area.
fn area_sign(p1: &[f64; 2], p2: &[f64; 2], p3: &[f64; 2]) -> f64 {
    sign((p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]))
}

/// `pointSegment` (`:499-521`): the point of segment `(u, v)` nearest `p`,
/// and its distance.
fn point_segment(p: &[f64; 2], u: &[f64; 2], v: &[f64; 2]) -> ([f64; 2], f64) {
    let uv = [v[0] - u[0], v[1] - u[1]];
    let up = [p[0] - u[0], p[1] - u[1]];
    let length = mju_dot(&uv, &uv);
    // `mju_max(mjMINVAL, length)`.
    let a = mju_dot(&uv, &up)
        / if MJ_MINVAL >= length {
            MJ_MINVAL
        } else {
            length
        };
    let res = if a <= 0.0 {
        *u
    } else if a >= 1.0 {
        *v
    } else {
        [u[0] + uv[0] * a, u[1] + uv[1] * a]
    };
    // MuJoCo's arithmetic (`mju_sqrt` of the sum), not `hypot`'s.
    #[allow(clippy::imprecise_flops)]
    let distance = ((res[0] - p[0]) * (res[0] - p[0]) + (res[1] - p[1]) * (res[1] - p[1])).sqrt();
    (res, distance)
}

/// `mjraw_SphereTriangle` (`:525-608`): the sphere at `centre` of radius
/// `rs` against the triangle `(t1, t2, t3)` of radius `rt`, within
/// `margin`: the nearest point of the triangle, inside it or on an edge, to
/// the centre.
pub(super) fn raw_sphere_triangle(
    margin: f64,
    centre: &[f64; 3],
    rs: f64,
    [t1, t2, t3]: [&[f64; 3]; 3],
    rt: f64,
) -> Option<RawContact> {
    let rbound = margin + rs + rt;

    // t1 the origin: the triangle (O, A, B), the centre S; the plane's
    // normal N.
    let s_rel = sub3(centre, t1);
    let a_rel = sub3(t2, t1);
    let b_rel = sub3(t3, t1);
    let mut plane = cross(&a_rel, &b_rel);
    normalize3(&mut plane);

    // The centre's signed distance to the plane; too far, none.
    let dst_s = dot3(&plane, &s_rel);
    if dst_s.abs() > rbound {
        return None;
    }

    // P, the centre projected on the plane, and the plane's axes V1 (along
    // A) and V2: in them the triangle (o, a, b) and P (p).
    let p_rel = add_scl(&s_rel, &plane, -dst_s);
    let mut v1 = a_rel;
    let len_a = normalize3(&mut v1);
    let mut v2 = cross(&plane, &a_rel);
    normalize3(&mut v2);
    let o2 = [0.0, 0.0];
    let a2 = [len_a, 0.0];
    let b2 = [dot3(&v1, &b_rel), dot3(&v2, &b_rel)];
    let p2 = [dot3(&v1, &p_rel), dot3(&v2, &p_rel)];

    let sign1 = area_sign(&p2, &o2, &a2);
    let sign2 = area_sign(&p2, &a2, &b2);
    let sign3 = area_sign(&p2, &b2, &o2);
    // MuJoCo compares the signs exactly.
    #[allow(clippy::float_cmp)]
    let nearest = if sign1 == sign2 && sign2 == sign3 {
        p_rel
    } else {
        // The nearest point on the edges (o, a), (a, b), (b, o).
        let (x0, d0) = point_segment(&p2, &o2, &a2);
        let (x1, d1) = point_segment(&p2, &a2, &b2);
        let (x2, d2) = point_segment(&p2, &b2, &o2);
        let best = if d0 < d1 && d0 < d2 {
            x0
        } else if d1 < d2 {
            x1
        } else {
            x2
        };
        // `mji_scl3`, then `mji_addToScl3`.
        let mut on_edge = scl3(&v1, best[0]);
        for k in 0..3 {
            on_edge[k] += v2[k] * best[1];
        }
        on_edge
    };

    // The normal from the centre to that point, and its separation.
    let mut normal = sub3(&nearest, &s_rel);
    let separation = normalize3(&mut normal);
    if separation > rbound {
        return None;
    }
    let dist = separation - rs - rt;
    Some(RawContact {
        dist,
        pos: add_scl(centre, &normal, rs + dist / 2.0),
        normal,
    })
}

/// `mjraw_BoxTriangle` (`:611-700`): box at `pos` turned by the row-major
/// `mat`, half-sizes `size`, against the triangle of radius `rt`: each
/// triangle vertex against the box's nearest face, then each box corner as
/// a sphere of radius 0 against the triangle.
pub(super) fn raw_box_triangle(
    margin: f64,
    pos: &[f64; 3],
    mat: &Mat9,
    size: &[f64; 3],
    vertices: [&[f64; 3]; 3],
    rt: f64,
) -> Vec<RawContact> {
    let mut contacts = Vec::new();
    for vert in vertices {
        let local = mat_t_vec(mat, &sub3(vert, pos));

        // The face the vertex is deepest under (or nearest outside).
        let mut maxaxis = 0;
        let mut maxval = local[0].abs() - size[0];
        for j in 1..3 {
            let val = local[j].abs() - size[j];
            if val > maxval {
                maxval = val;
                maxaxis = j;
            }
        }
        if maxval - rt > margin {
            continue;
        }
        if (0..3).any(|j| local[j].abs() > size[j] + margin + rt) {
            continue;
        }
        if contacts.len() < MAX_CONPAIR {
            let mut nrm_local = [0.0; 3];
            nrm_local[maxaxis] = if local[maxaxis] > 0.0 { 1.0 } else { -1.0 };
            let normal = mat_vec(mat, &nrm_local);
            let dist = maxval - rt;
            let offset = rt + dist * 0.5;
            contacts.push(RawContact {
                dist,
                pos: add_scl(vert, &normal, -offset),
                normal,
            });
        }
    }
    for i in 0..8 {
        if contacts.len() >= MAX_CONPAIR {
            break;
        }
        let local = [
            if i & 1 != 0 { size[0] } else { -size[0] },
            if i & 2 != 0 { size[1] } else { -size[1] },
            if i & 4 != 0 { size[2] } else { -size[2] },
        ];
        let corner = add3(&mat_vec(mat, &local), pos);
        if let Some(contact) = raw_sphere_triangle(margin, &corner, 0.0, vertices, rt) {
            contacts.push(contact);
        }
    }
    contacts
}

/// `mjraw_CapsuleTriangle` (`:703-766`): capsule at `pos`, its axis `mat`'s
/// third column, `size` its radius and half-length, against the triangle of
/// radius `rt`: its two end spheres, then each triangle vertex against the
/// inside of its segment.
pub(super) fn raw_capsule_triangle(
    margin: f64,
    pos: &[f64; 3],
    mat: &Mat9,
    size: [f64; 2],
    vertices: [&[f64; 3]; 3],
    rt: f64,
) -> Vec<RawContact> {
    let [radius, len] = size;
    let axis = [mat[2], mat[5], mat[8]];
    let p1 = add_scl(pos, &axis, -len);
    let p2 = add_scl(pos, &axis, len);
    let mut contacts = Vec::new();
    for end in [&p1, &p2] {
        contacts.extend(raw_sphere_triangle(margin, end, radius, vertices, rt));
        if contacts.len() >= MAX_CONPAIR {
            return contacts;
        }
    }
    for vert in vertices {
        let ab = sub3(&p2, &p1);
        let t = dot3(&sub3(vert, &p1), &ab) / (4.0 * len * len);
        if t <= MJ_MINVAL || t >= 1.0 - MJ_MINVAL {
            continue;
        }
        let closest = add_scl(&p1, &ab, t);
        let mut normal = sub3(vert, &closest);
        let dst = normalize3(&mut normal);
        if dst > radius + rt + margin {
            continue;
        }
        // `mji_add3`, `mji_addToScl3`, then `mju_scl3` by 0.5.
        let mut midway = add3(&closest, vert);
        for k in 0..3 {
            midway[k] += normal[k] * (radius - rt);
        }
        contacts.push(RawContact {
            dist: dst - radius - rt,
            pos: scl3(&midway, 0.5),
            normal,
        });
        if contacts.len() >= MAX_CONPAIR {
            return contacts;
        }
    }
    contacts
}

/// `planeVertex` (`engine_collision_driver.c:1714-1740`): vertex `v`, a
/// sphere of `radius`, against the plane of the face `t0`, `t1`, `t2`, whose
/// normal is `(t1 − t0) × (t2 − t0)` made unit: nothing where `v` is two
/// radii or more behind the plane; else its distance minus `v`'s along the
/// normal less two radii, its normal the plane's turned round, its position
/// `v` moved along the normal by minus half `v`'s distance.
pub(super) fn plane_vertex(
    [t0, t1, t2]: [&[f64; 3]; 3],
    v: &[f64; 3],
    radius: f64,
) -> Option<RawContact> {
    let (e1, e2, ev) = (sub3(t1, t0), sub3(t2, t0), sub3(v, t0));
    let mut nrm = cross(&e1, &e2);
    normalize3(&mut nrm);
    let dst = dot3(&ev, &nrm);
    if dst <= -2.0 * radius {
        return None;
    }
    Some(RawContact {
        dist: -dst - 2.0 * radius,
        pos: add_scl(v, &nrm, -0.5 * dst),
        normal: scl3(&nrm, -1.0),
    })
}

/// `mj_makeCapsule` (`engine_collision_driver.c:1432-1449`): the capsule a
/// cable's element is, between its vertices `v1` and `v2`: its position
/// halfway, its z axis from `v2` to `v1` (`mju_quatZ2Vec`), its radius the
/// flex's and its half-length half the distance.
pub(super) fn make_capsule(
    v1: &[f64; 3],
    v2: &[f64; 3],
    radius: f64,
) -> (Vector3<f64>, Matrix3<f64>, Vector3<f64>) {
    let mut dif = sub3(v1, v2);
    let half = 0.5 * normalize3(&mut dif);
    let pos = scl3(&add3(v1, v2), 0.5);
    let mat = quat_to_mat(&quat_z2vec(&dif));
    (
        Vector3::from(pos),
        Matrix3::from_row_slice(&mat),
        Vector3::new(radius, half, 0.0),
    )
}

#[cfg(test)]
// Test code: a malformed golden is a failed test.
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use nalgebra::{Matrix3, Vector3};
    use serde_json::Value;

    use super::super::mj_ccd::{CcdObj, mjc_convex_elem, mjc_hfield_elem};
    use super::super::mj_primitive::{
        Frame, RawContact, raw_capsule_box, raw_capsule_capsule, raw_sphere_capsule,
    };
    use super::{make_capsule, raw_box_triangle, raw_capsule_triangle, raw_sphere_triangle};
    use crate::heightfield::HeightFieldData;
    use crate::quat::quat_z2vec;
    use crate::types::contact_types::make_frame;
    use crate::types::{GeomType, Model};

    /// MuJoCo's `mjGEOM_HFIELD`, `mjGEOM_SPHERE`, `mjGEOM_CAPSULE`,
    /// `mjGEOM_ELLIPSOID`, `mjGEOM_CYLINDER` and `mjGEOM_BOX`.
    const HFIELD: u64 = 1;
    const SPHERE: u64 = 2;
    const CAPSULE: u64 = 3;
    const ELLIPSOID: u64 = 4;
    const CYLINDER: u64 = 5;
    const BOX: u64 = 6;

    fn golden() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../tests/assets/golden/flex/flex_primitives.json"
        );
        let text = std::fs::read_to_string(path).expect("the flex primitive golden reads");
        serde_json::from_str(&text).expect("the flex primitive golden parses")
    }

    fn vec3(v: &Value) -> [f64; 3] {
        let a: Vec<f64> = v
            .as_array()
            .expect("array")
            .iter()
            .map(|x| x.as_f64().expect("number"))
            .collect();
        [a[0], a[1], a[2]]
    }

    fn floats(v: &Value) -> Vec<f64> {
        v.as_array()
            .expect("array")
            .iter()
            .map(|x| x.as_f64().expect("number"))
            .collect()
    }

    /// The raw function MuJoCo's driver calls for the case's geom and
    /// element (`engine_collision_driver.c:1890-1963`), on its inputs.
    fn ours(case: &Value) -> Vec<RawContact> {
        let pos = vec3(&case["pos"]);
        let mat_rows = floats(&case["mat"]);
        let mat: [f64; 9] = std::array::from_fn(|i| mat_rows[i]);
        let size = vec3(&case["size"]);
        let vertices: Vec<[f64; 3]> = case["vertices"]
            .as_array()
            .expect("vertices")
            .iter()
            .map(vec3)
            .collect();
        let radius = case["radius"].as_f64().expect("radius");
        let margin = case["margin"].as_f64().expect("margin");
        let kind = case["type"].as_u64().expect("type");
        if let [t1, t2, t3] = vertices.as_slice() {
            let triangle = [t1, t2, t3];
            return match kind {
                SPHERE => raw_sphere_triangle(margin, &pos, size[0], triangle, radius)
                    .into_iter()
                    .collect(),
                BOX => raw_box_triangle(margin, &pos, &mat, &size, triangle, radius),
                CAPSULE => {
                    raw_capsule_triangle(margin, &pos, &mat, [size[0], size[1]], triangle, radius)
                }
                _ => panic!("geom type {kind}"),
            };
        }
        let (cpos, cmat, csize) = make_capsule(&vertices[0], &vertices[1], radius);
        let (gpos, gmat) = (Vector3::from(pos), Matrix3::from_row_slice(&mat));
        let geom = Frame {
            pos: &gpos,
            mat: &gmat,
        };
        let capsule = Frame {
            pos: &cpos,
            mat: &cmat,
        };
        let gsize = Vector3::from(size);
        match kind {
            SPHERE => raw_sphere_capsule(margin, geom, size[0], capsule, &csize)
                .into_iter()
                .collect(),
            CAPSULE => raw_capsule_capsule(margin, geom, &gsize, capsule, &csize),
            // The box second: its normals turned round (`:1916-1924`).
            BOX => raw_capsule_box(margin, capsule, &csize, geom, &gsize)
                .into_iter()
                .map(|c| RawContact {
                    normal: [-c.normal[0], -c.normal[1], -c.normal[2]],
                    ..c
                })
                .collect(),
            _ => panic!("geom type {kind}"),
        }
    }

    /// Each case's contacts as MuJoCo's, bit for bit: a sphere, a box and a
    /// capsule against a triangle with a radius (inside it, past an edge or
    /// a vertex, under it, in the margin, apart, and a degenerate triangle),
    /// and against a cable's element, the capsule `mj_makeCapsule` makes
    /// (`gen_flex_primitive_reference.py`). The normal is compared as
    /// `mj_setContact` leaves it, normalised by `mju_makeFrame`.
    #[test]
    fn triangle_and_cable_element_contacts_as_mujoco() {
        for case in golden()["cases"].as_array().expect("cases") {
            let name = case["name"].as_str().expect("name");
            let ours = ours(case);
            let want = case["contacts"].as_array().expect("contacts");
            assert_eq!(ours.len(), want.len(), "{name}: contacts");
            for (k, (c, w)) in ours.iter().zip(want).enumerate() {
                let dist = w["dist"].as_f64().expect("dist");
                assert_eq!(
                    c.dist.to_bits(),
                    dist.to_bits(),
                    "{name} {k}: dist {} {dist}",
                    c.dist
                );
                let (pos, normal) = (vec3(&w["pos"]), vec3(&w["normal"]));
                for (i, (ours, want)) in c.pos.iter().zip(pos).enumerate() {
                    assert_eq!(ours.to_bits(), want.to_bits(), "{name} {k}: pos {i}");
                }
                let (n, _) = make_frame(&Vector3::from(c.normal), &Vector3::zeros());
                for (i, (ours, want)) in n.iter().zip(normal).enumerate() {
                    assert_eq!(ours.to_bits(), want.to_bits(), "{name} {k}: normal {i}");
                }
            }
        }
    }

    /// The convex route's contacts (`mjc_ConvexElem`, `mjc_HFieldElem`)
    /// for the case's geom and element, on its inputs.
    fn convex(case: &Value) -> Vec<RawContact> {
        let kind = case["type"].as_u64().expect("type");
        let pos = vec3(&case["pos"]);
        let mat_rows = floats(&case["mat"]);
        let mat: [f64; 9] = std::array::from_fn(|i| mat_rows[i]);
        let size = vec3(&case["size"]);
        let vertices: Vec<[f64; 3]> = case["vertices"]
            .as_array()
            .expect("vertices")
            .iter()
            .map(vec3)
            .collect();
        let center = vec3(&case["center"]);
        let radius = case["radius"].as_f64().expect("radius");
        let margin = case["margin"].as_f64().expect("margin");
        let ccd = (
            usize::try_from(case["ccd"][0].as_u64().expect("iterations")).expect("fits"),
            case["ccd"][1].as_f64().expect("tolerance"),
        );
        if kind == HFIELD {
            let field = &case["hfield"];
            let count = |k: &str| usize::try_from(field[k].as_u64().expect(k)).expect("fits");
            let (nrow, ncol) = (count("nrow"), count("ncol"));
            let hsize = floats(&field["size"]);
            let hsize = [hsize[0], hsize[1], hsize[2], hsize[3]];
            let heights = floats(&field["data"])
                .iter()
                .map(|h| h * hsize[2])
                .collect();
            // A grid's extent, far below 2^52, exact as a float.
            #[allow(clippy::cast_precision_loss)]
            let cell = 2.0 * hsize[0] / (ncol - 1) as f64;
            let hfield = HeightFieldData::new(heights, ncol, nrow, cell);
            return mjc_hfield_elem(
                &hfield, &hsize, &pos, &mat, &vertices, &center, radius, margin, ccd,
            );
        }
        let geom_type = match kind {
            SPHERE => GeomType::Sphere,
            CAPSULE => GeomType::Capsule,
            ELLIPSOID => GeomType::Ellipsoid,
            CYLINDER => GeomType::Cylinder,
            BOX => GeomType::Box,
            _ => panic!("geom type {kind}"),
        };
        let mut model = Model::empty();
        (model.ccd_iterations, model.ccd_tolerance) = ccd;
        let mut geom = CcdObj::new(geom_type, pos, mat, size, margin, None).expect("a convex geom");
        let mut elem = CcdObj::flex(&vertices, center, radius, margin);
        let fix = (vertices.len() == 3).then_some((geom_type, pos, &mat, size));
        mjc_convex_elem(&model, &mut geom, &mut elem, margin, fix)
    }

    /// The convex route as MuJoCo's, bit for bit: an ellipsoid and a
    /// cylinder against a triangle (their normals fixed by `mjc_fixNormal`:
    /// inside and outside the ellipsoid, a cylinder's round wall, and near
    /// its cap left as `mjc_ccd` gives it), a sphere, a capsule, a box, an
    /// ellipsoid and a cylinder against a tetrahedron, an ellipsoid and a
    /// cylinder against a cable's element (`mjc_ConvexElem`), and a height
    /// field against a triangle and a tetrahedron, one within a margin
    /// (`mjc_HFieldElem`, its prism set up as `mjc_ConvexHField`'s).
    #[test]
    fn convex_route_contacts_as_mujoco() {
        for case in golden()["convex"].as_array().expect("convex") {
            let name = case["name"].as_str().expect("name");
            let ours = convex(case);
            let want = case["contacts"].as_array().expect("contacts");
            assert_eq!(ours.len(), want.len(), "{name}: contacts");
            for (k, (c, w)) in ours.iter().zip(want).enumerate() {
                let dist = w["dist"].as_f64().expect("dist");
                assert_eq!(
                    c.dist.to_bits(),
                    dist.to_bits(),
                    "{name} {k}: dist {} {dist}",
                    c.dist
                );
                let (pos, normal) = (vec3(&w["pos"]), vec3(&w["normal"]));
                for (i, (ours, want)) in c.pos.iter().zip(pos).enumerate() {
                    assert_eq!(ours.to_bits(), want.to_bits(), "{name} {k}: pos {i}");
                }
                let (n, _) = make_frame(&Vector3::from(c.normal), &Vector3::zeros());
                for (i, (ours, want)) in n.iter().zip(normal).enumerate() {
                    assert_eq!(ours.to_bits(), want.to_bits(), "{name} {k}: normal {i}");
                }
            }
        }
    }

    /// `mju_quatZ2Vec` as MuJoCo's, bit for bit: along +z, along −z (half a
    /// turn about x), too short to turn, and general vectors.
    #[test]
    fn quat_z2vec_as_mujoco() {
        for case in golden()["quat_z2vec"].as_array().expect("cases") {
            let vec = vec3(&case["vec"]);
            let want = floats(&case["quat"]);
            let ours = quat_z2vec(&vec);
            for (i, (ours, want)) in ours.iter().zip(&want).enumerate() {
                assert_eq!(ours.to_bits(), want.to_bits(), "{vec:?}: {i}");
            }
        }
    }
}
