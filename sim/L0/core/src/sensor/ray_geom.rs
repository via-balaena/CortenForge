//! MuJoCo's `mju_rayGeom` (`engine_ray.c:972-998`) on the types it takes:
//! along a ray from `pnt` in the direction `vec`, the distance to a geom's
//! surface, in lengths of `vec`, the nearest crossing at or past the start,
//! or −1 where the ray misses. The touch sensor's zone
//! (`engine_sensor.c:922-927`). Each step is MuJoCo's, in its order, so a
//! ray grazing a zone's edge goes the way MuJoCo's does.

use nalgebra::{Matrix3, Point3, UnitVector3, Vector3};

use crate::constraint::impedance::MJ_MINVAL;
use crate::types::{Data, GeomType, Model};

/// MuJoCo's `mj_ray` (`engine_ray.c:1308-1351`) as a site's rangefinder casts
/// it (`engine_sensor.c:478-479`: no group filter, static geoms included):
/// from `pnt` along `vec`, the distance in lengths of `vec` to the nearest
/// geom that `bodyexclude` does not hold and whose alpha is not 0
/// (`ray_eliminate`, `:68-99`; sim-mjcf keeps no materials, so a material's
/// alpha is not read), or −1. A primitive's distance is [`ray_geom`]'s. A
/// mesh's, a height field's and a signed distance field's are
/// `raycast_geom`'s along the unit direction, divided by `|vec|`, not
/// MuJoCo's `mj_rayMesh`, `mj_rayHfield` and `mj_raySdf` (ledger L267); each
/// is cast no farther than the geom's bounding sphere reaches. A plane is read
/// as unbounded, as MuJoCo reads one of size 0: sim-mjcf does not keep a
/// plane's written size until Rigid-loading's L14, and MuJoCo bounds the hit
/// by it.
pub fn mj_ray(
    model: &Model,
    data: &Data,
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
    bodyexclude: usize,
) -> f64 {
    let mut dist = -1.0;
    for g in 0..model.ngeom {
        if model.geom_body[g] == bodyexclude || model.geom_rgba[g][3] == 0.0 {
            continue;
        }
        let newdist = match model.geom_type[g] {
            GeomType::Mesh | GeomType::Hfield | GeomType::Sdf => {
                let norm = vec.norm();
                let reach = (pnt - data.geom_xpos[g]).norm() + model.geom_rbound[g];
                crate::raycast::raycast_geom(
                    model,
                    data,
                    g,
                    Point3::from(*pnt),
                    UnitVector3::new_unchecked(vec / norm),
                    reach,
                )
                .map_or(-1.0, |hit| hit.distance / norm)
            }
            GeomType::Plane => ray_geom(
                &data.geom_xpos[g],
                &data.geom_xmat[g],
                &Vector3::zeros(),
                pnt,
                vec,
                GeomType::Plane,
            ),
            geom_type => ray_geom(
                &data.geom_xpos[g],
                &data.geom_xmat[g],
                &model.geom_size[g],
                pnt,
                vec,
                geom_type,
            ),
        };
        if newdist >= 0.0 && (newdist < dist || dist < 0.0) {
            dist = newdist;
        }
    }
    dist
}

/// `mju_rayGeom`: the distance along the ray to the geom of `geom_type`,
/// `size`, centred at `pos` and turned by `mat`, or −1. MuJoCo raises an
/// error for a mesh, a height field or a signed distance field; sim-mjcf
/// gives a site none of them, and here they read −1.
pub fn ray_geom(
    pos: &Vector3<f64>,
    mat: &Matrix3<f64>,
    size: &Vector3<f64>,
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
    geom_type: GeomType,
) -> f64 {
    let size = [size.x, size.y, size.z];
    match geom_type {
        GeomType::Plane => ray_plane(pos, mat, &size, pnt, vec),
        GeomType::Sphere => ray_sphere(pos, size[0] * size[0], pnt, vec),
        GeomType::Capsule => ray_capsule(pos, mat, &size, pnt, vec),
        GeomType::Ellipsoid => ray_ellipsoid(pos, mat, &size, pnt, vec),
        GeomType::Cylinder => ray_cylinder(pos, mat, &size, pnt, vec),
        GeomType::Box => ray_box(pos, mat, &size, pnt, vec),
        GeomType::Mesh | GeomType::Hfield | GeomType::Sdf => -1.0,
    }
}

/// `ray_map`: the ray's start and direction in the geom's frame.
fn ray_map(
    pos: &Vector3<f64>,
    mat: &Matrix3<f64>,
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> ([f64; 3], [f64; 3]) {
    let local = |v: [f64; 3]| {
        [0, 1, 2].map(|i| mat[(0, i)] * v[0] + mat[(1, i)] * v[1] + mat[(2, i)] * v[2])
    };
    (
        local([pnt.x - pos.x, pnt.y - pos.y, pnt.z - pos.z]),
        local([vec.x, vec.y, vec.z]),
    )
}

/// `ray_quad`: the real roots of `a·x² + 2·b·x + c = 0`, the lower first
/// (both −1 where there are none or `a` is below mjMINVAL), and the least of
/// them not below 0, or −1.
// b² − a·c is the discriminant of a·x² + 2·b·x + c, as MuJoCo writes it.
#[allow(clippy::suspicious_operation_groupings)]
fn ray_quad(a: f64, b: f64, c: f64) -> (f64, [f64; 2]) {
    let det = b * b - a * c;
    if det < 0.0 || a < MJ_MINVAL {
        return (-1.0, [-1.0, -1.0]);
    }
    let det = det.sqrt();
    let x = [(-b - det) / a, (-b + det) / a];
    let nearest = if x[0] >= 0.0 {
        x[0]
    } else if x[1] >= 0.0 {
        x[1]
    } else {
        -1.0
    };
    (nearest, x)
}

/// `ray_plane`: its front face only, within its size where that is above 0.
fn ray_plane(
    pos: &Vector3<f64>,
    mat: &Matrix3<f64>,
    size: &[f64; 3],
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> f64 {
    let (lp, lv) = ray_map(pos, mat, pnt, vec);
    if lv[2] > -MJ_MINVAL {
        return -1.0;
    }
    let x = -lp[2] / lv[2];
    if x < 0.0 {
        return -1.0;
    }
    let p0 = lp[0] + x * lv[0];
    let p1 = lp[1] + x * lv[1];
    if (size[0] <= 0.0 || p0.abs() <= size[0]) && (size[1] <= 0.0 || p1.abs() <= size[1]) {
        x
    } else {
        -1.0
    }
}

/// `ray_sphere`: a sphere of radius `√dist_sqr` about `pos` (the bounding
/// sphere of the other types too).
fn ray_sphere(pos: &Vector3<f64>, dist_sqr: f64, pnt: &Vector3<f64>, vec: &Vector3<f64>) -> f64 {
    let dif = [pnt.x - pos.x, pnt.y - pos.y, pnt.z - pos.z];
    let a = vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
    let b = vec.x * dif[0] + vec.y * dif[1] + vec.z * dif[2];
    let c = dif[0] * dif[0] + dif[1] * dif[1] + dif[2] * dif[2] - dist_sqr;
    ray_quad(a, b, c).0
}

/// `ray_capsule`: its round side between its flat ends, then each cap's
/// half sphere, the nearest crossing of the three.
fn ray_capsule(
    pos: &Vector3<f64>,
    mat: &Matrix3<f64>,
    size: &[f64; 3],
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> f64 {
    let ssz = size[0] + size[1];
    if ray_sphere(pos, ssz * ssz, pnt, vec) < 0.0 {
        return -1.0;
    }
    let (lp, lv) = ray_map(pos, mat, pnt, vec);
    let mut x = -1.0;

    let a = lv[0] * lv[0] + lv[1] * lv[1];
    let b = lv[0] * lp[0] + lv[1] * lp[1];
    let c = lp[0] * lp[0] + lp[1] * lp[1] - size[0] * size[0];
    let (sol, _) = ray_quad(a, b, c);
    if sol >= 0.0 && (lp[2] + sol * lv[2]).abs() <= size[1] && (x < 0.0 || sol < x) {
        x = sol;
    }

    let mut ldif = [lp[0], lp[1], lp[2] - size[1]];
    let a = lv[0] * lv[0] + lv[1] * lv[1] + lv[2] * lv[2];
    let b = lv[0] * ldif[0] + lv[1] * ldif[1] + lv[2] * ldif[2];
    let c = ldif[0] * ldif[0] + ldif[1] * ldif[1] + ldif[2] * ldif[2] - size[0] * size[0];
    for xi in ray_quad(a, b, c).1 {
        if xi >= 0.0 && lp[2] + xi * lv[2] >= size[1] && (x < 0.0 || xi < x) {
            x = xi;
        }
    }

    ldif[2] = lp[2] + size[1];
    let b = lv[0] * ldif[0] + lv[1] * ldif[1] + lv[2] * ldif[2];
    let c = ldif[0] * ldif[0] + ldif[1] * ldif[1] + ldif[2] * ldif[2] - size[0] * size[0];
    for xi in ray_quad(a, b, c).1 {
        if xi >= 0.0 && lp[2] + xi * lv[2] <= -size[1] && (x < 0.0 || xi < x) {
            x = xi;
        }
    }
    x
}

/// `ray_ellipsoid`: the ray scaled by the inverse squared half axes.
fn ray_ellipsoid(
    pos: &Vector3<f64>,
    mat: &Matrix3<f64>,
    size: &[f64; 3],
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> f64 {
    let (lp, lv) = ray_map(pos, mat, pnt, vec);
    let s = size.map(|h| 1.0 / (h * h));
    let a = s[0] * lv[0] * lv[0] + s[1] * lv[1] * lv[1] + s[2] * lv[2] * lv[2];
    let b = s[0] * lv[0] * lp[0] + s[1] * lv[1] * lp[1] + s[2] * lv[2] * lp[2];
    let c = s[0] * lp[0] * lp[0] + s[1] * lp[1] * lp[1] + s[2] * lp[2] * lp[2] - 1.0;
    ray_quad(a, b, c).0
}

/// `ray_cylinder`: each flat end within its radius, then its round side
/// between them, the nearest crossing.
fn ray_cylinder(
    pos: &Vector3<f64>,
    mat: &Matrix3<f64>,
    size: &[f64; 3],
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> f64 {
    let ssz = size[0] * size[0] + size[1] * size[1];
    if ray_sphere(pos, ssz, pnt, vec) < 0.0 {
        return -1.0;
    }
    let (lp, lv) = ray_map(pos, mat, pnt, vec);
    let mut x = -1.0;
    if lv[2].abs() > MJ_MINVAL {
        for side in [-1.0, 1.0] {
            let sol = (side * size[1] - lp[2]) / lv[2];
            if sol >= 0.0 {
                let p0 = lp[0] + sol * lv[0];
                let p1 = lp[1] + sol * lv[1];
                if p0 * p0 + p1 * p1 <= size[0] * size[0] && (x < 0.0 || sol < x) {
                    x = sol;
                }
            }
        }
    }
    let a = lv[0] * lv[0] + lv[1] * lv[1];
    let b = lv[0] * lp[0] + lv[1] * lp[1];
    let c = lp[0] * lp[0] + lp[1] * lp[1] - size[0] * size[0];
    let (sol, _) = ray_quad(a, b, c);
    if sol >= 0.0 && (lp[2] + sol * lv[2]).abs() <= size[1] && (x < 0.0 || sol < x) {
        x = sol;
    }
    x
}

/// `ray_box`: each face within its rectangle, the nearest crossing.
fn ray_box(
    pos: &Vector3<f64>,
    mat: &Matrix3<f64>,
    size: &[f64; 3],
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> f64 {
    // Each axis's face, and the two axes its rectangle spans.
    const IFACE: [[usize; 2]; 3] = [[1, 2], [0, 2], [0, 1]];
    let ssz = size[0] * size[0] + size[1] * size[1] + size[2] * size[2];
    if ray_sphere(pos, ssz, pnt, vec) < 0.0 {
        return -1.0;
    }
    let (lp, lv) = ray_map(pos, mat, pnt, vec);
    let mut x = -1.0;
    for (i, [j, k]) in IFACE.into_iter().enumerate() {
        if lv[i].abs() > MJ_MINVAL {
            for side in [-1.0, 1.0] {
                let sol = (side * size[i] - lp[i]) / lv[i];
                if sol >= 0.0 {
                    let p0 = lp[j] + sol * lv[j];
                    let p1 = lp[k] + sol * lv[k];
                    if p0.abs() <= size[j] && p1.abs() <= size[k] && (x < 0.0 || sol < x) {
                        x = sol;
                    }
                }
            }
        }
    }
    x
}

#[cfg(test)]
mod tests {
    // Test code: a malformed golden is a failed test.
    #![allow(clippy::expect_used, clippy::panic)]

    use nalgebra::{Matrix3, Vector3};
    use serde_json::Value;

    use super::ray_geom;
    use crate::types::GeomType;

    fn vector(v: &Value) -> Vec<f64> {
        v.as_array()
            .expect("an array")
            .iter()
            .map(|x| x.as_f64().expect("a number"))
            .collect()
    }

    /// Each of the touch golden's `rays` (`gen_touch_reference.py`: 300 a
    /// type, from inside a geom and outside it, along its own axes and not;
    /// and 24 on the edges of its tests) meets its geom at MuJoCo's
    /// distance, or misses it as MuJoCo's does (-1), bit for bit.
    #[test]
    fn rays_meet_each_geom_as_mujoco() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../tests/assets/golden/touch/touch.json"
        );
        let text = std::fs::read_to_string(path).expect("the touch golden reads");
        let golden: Value = serde_json::from_str(&text).expect("the touch golden parses");
        let rays = golden["rays"].as_array().expect("rays");
        assert_eq!(rays.len(), 1824);
        let mut differ = Vec::new();
        for (i, ray) in rays.iter().enumerate() {
            let geom_type = match ray["type"].as_str().expect("a type") {
                "plane" => GeomType::Plane,
                "sphere" => GeomType::Sphere,
                "capsule" => GeomType::Capsule,
                "ellipsoid" => GeomType::Ellipsoid,
                "cylinder" => GeomType::Cylinder,
                "box" => GeomType::Box,
                other => panic!("ray {i}: a type {other}"),
            };
            let three = |key: &str| Vector3::from_column_slice(&vector(&ray[key]));
            let mat = Matrix3::from_row_slice(&vector(&ray["mat"]));
            let ours = ray_geom(
                &three("pos"),
                &mat,
                &three("size"),
                &three("pnt"),
                &three("vec"),
                geom_type,
            );
            let theirs = ray["x"].as_f64().expect("a distance");
            if ours.to_bits() != theirs.to_bits() {
                differ.push(format!(
                    "ray {i} ({geom_type:?}): {ours:e}, MuJoCo {theirs:e}"
                ));
            }
        }
        assert!(differ.is_empty(), "{}", differ.join("\n"));
    }
}
