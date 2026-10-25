//! MuJoCo's rays through triangles (`engine_ray.c`): a height field's
//! (`mj_rayHfield`, `:561-738`), through `ray_triangle` (`:131-200`). Each
//! step is MuJoCo's, in its order, so a ray grazing a triangle's edge goes
//! the way MuJoCo's does.

use nalgebra::{Matrix3, Vector3};

use super::ray_geom::{ray_box, ray_map};
use crate::constraint::impedance::MJ_MINVAL;
use crate::heightfield::HeightFieldData;
use crate::quat::normalize3;

/// `mju_dot3`.
fn dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `mju_sub3`.
fn sub3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `mju_cross`.
fn cross3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `mjMIN`: `a` where it is below `b`, else `b`.
fn mj_min(a: f64, b: f64) -> f64 {
    if a < b { a } else { b }
}

/// `mjMAX`: `a` where it is above `b`, else `b`.
fn mj_max(a: f64, b: f64) -> f64 {
    if a > b { a } else { b }
}

/// `mju_round`: the nearer whole number, the upper one at a half.
fn mju_round(x: f64) -> f64 {
    let (lower, upper) = (x.floor(), x.ceil());
    if x - lower < upper - x { lower } else { upper }
}

/// The basis `b0`, `b1` of the plane normal to the ray `lvec` that
/// [`ray_triangle`] projects onto (`engine_ray.c:609-620`): `b0` is (1, 1, 1)
/// with the ray's largest part set to 0 (the first of equals), `b1` that less
/// its part along the ray, normalised, and then `b0` is `b1 × lvec`,
/// normalised.
fn ray_basis(lvec: &[f64; 3]) -> ([f64; 3], [f64; 3]) {
    let mut b0 = [1.0; 3];
    if lvec[0].abs() >= lvec[1].abs() && lvec[0].abs() >= lvec[2].abs() {
        b0[0] = 0.0;
    } else if lvec[1].abs() >= lvec[2].abs() {
        b0[1] = 0.0;
    } else {
        b0[2] = 0.0;
    }
    let scl = -dot3(lvec, &b0) / dot3(lvec, lvec);
    let mut b1 = [
        b0[0] + scl * lvec[0],
        b0[1] + scl * lvec[1],
        b0[2] + scl * lvec[2],
    ];
    normalize3(&mut b1);
    let mut b0 = cross3(&b1, lvec);
    normalize3(&mut b0);
    (b0, b1)
}

/// `ray_triangle`: along the ray from `lpnt` in the direction `lvec`, the
/// distance in lengths of `lvec` to the plane of the triangle `v`, where the
/// ray passes through the triangle as seen along it (each corner's offset
/// from the start on `b0` and `b1`, [`ray_basis`]'s), or −1. The distance is
/// below 0 for a triangle behind the start; the callers keep one at or past
/// 0.
fn ray_triangle(
    v: &[[f64; 3]; 3],
    lpnt: &[f64; 3],
    lvec: &[f64; 3],
    b0: &[f64; 3],
    b1: &[f64; 3],
) -> f64 {
    let dif: [[f64; 3]; 3] = std::array::from_fn(|i| sub3(&v[i], lpnt));
    let planar: [[f64; 2]; 3] = std::array::from_fn(|i| [dot3(b0, &dif[i]), dot3(b1, &dif[i])]);
    let all_above = |k: usize| planar.iter().all(|p| p[k] > 0.0);
    let all_below = |k: usize| planar.iter().all(|p| p[k] < 0.0);
    if all_above(0) || all_below(0) || all_above(1) || all_below(1) {
        return -1.0;
    }

    // The start within the triangle as seen along the ray: A t = b, with A's
    // columns the first two corners less the third, b the third negated.
    let a = [
        planar[0][0] - planar[2][0],
        planar[1][0] - planar[2][0],
        planar[0][1] - planar[2][1],
        planar[1][1] - planar[2][1],
    ];
    let b = [-planar[2][0], -planar[2][1]];
    let det = a[0] * a[3] - a[1] * a[2];
    if det.abs() < MJ_MINVAL {
        return -1.0;
    }
    let t0 = (a[3] * b[0] - a[1] * b[1]) / det;
    let t1 = (-a[2] * b[0] + a[0] * b[1]) / det;
    if t0 < 0.0 || t1 < 0.0 || t0 + t1 > 1.0 {
        return -1.0;
    }

    let nrm = cross3(&sub3(&v[0], &v[2]), &sub3(&v[1], &v[2]));
    let denom = dot3(lvec, &nrm);
    if denom.abs() < MJ_MINVAL {
        return -1.0;
    }
    -dot3(&sub3(lpnt, &v[2]), &nrm) / denom
}

/// `mj_rayHfield`: along a ray from `pnt` in the direction `vec`, the
/// distance in lengths of `vec` to the height field `hfield` of half-extents,
/// top and base `size`, centred at `xpos` and turned by `xmat`, or −1. Its
/// base box (`size[3]` deep, below `xpos`) first; then, where the ray crosses
/// its top box (`size[2]` high, above), each triangle of the cells under the
/// ray's path through that box, a cell more each way, the cells split along
/// (c, r)–(c + 1, r + 1); and the top box's four sides, each where the
/// ray's crossing is below the field's edge there; the nearest.
///
/// `hfield` is the field as sim-core holds it, MuJoCo's elevations scaled by
/// `size[2]`, its rows from −y (as `collision::mj_ccd::mjc_convex_hfield`
/// reads it). The sides' test reads MuJoCo's elevation unscaled and in single
/// precision: the height over `size[2]`, rounded to single precision, which
/// is that elevation wherever the height is one scaled. sim-mjcf does not yet
/// load a field's elevations as MuJoCo's (normalised, the rows flipped, in
/// single precision: book Rigid-loading L45). A field of one row or column,
/// which sim-mjcf refuses (ledger L208) and whose sides MuJoCo's test reads
/// past its data for, reads its base box's distance.
pub(super) fn mj_ray_hfield(
    hfield: &HeightFieldData,
    size: &[f64; 4],
    xpos: &Vector3<f64>,
    xmat: &Matrix3<f64>,
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> f64 {
    let (nrow, ncol) = (hfield.depth(), hfield.width());
    // The field's z axis, its frame's third column.
    let up = [xmat[(0, 2)], xmat[(1, 2)], xmat[(2, 2)]];
    let base_size = [size[0], size[1], size[3] * 0.5];
    let base_pos = Vector3::new(
        xpos.x - up[0] * size[3] * 0.5,
        xpos.y - up[1] * size[3] * 0.5,
        xpos.z - up[2] * size[3] * 0.5,
    );
    let top_size = [size[0], size[1], size[2] * 0.5];
    let top_pos = Vector3::new(
        xpos.x + up[0] * size[2] * 0.5,
        xpos.y + up[1] * size[2] * 0.5,
        xpos.z + up[2] * size[2] * 0.5,
    );

    let mut x = ray_box(&base_pos, xmat, &base_size, pnt, vec).0;
    let (top_intersect, all) = ray_box(&top_pos, xmat, &top_size, pnt, vec);
    if top_intersect < 0.0 || nrow < 2 || ncol < 2 {
        return x;
    }

    let (lpnt, lvec) = ray_map(xpos, xmat, pnt, vec);
    let (b0, b1) = ray_basis(&lvec);

    // The ray's path through the top box: from its first crossing to the
    // farthest face it crosses, or from its start where it starts inside.
    let mut seg = [0.0, top_intersect];
    for &a in &all {
        if a > seg[1] {
            seg = [top_intersect, a];
        }
    }

    // A grid's extents are far below 2^52, exact as floats.
    #[allow(clippy::cast_precision_loss)]
    let (cols, rows) = ((ncol - 1) as f64, (nrow - 1) as f64);
    let dx = (2.0 * size[0]) / cols;
    let dy = (2.0 * size[1]) / rows;
    let sx = seg.map(|s| (lpnt[0] + s * lvec[0] + size[0]) / dx);
    let sy = seg.map(|s| (lpnt[1] + s * lvec[1] + size[1]) / dy);

    // The cells under the path, a cell more each way, within the grid. A
    // cast to an integer saturates, and takes a NaN to 0, where C's leaves
    // it undefined.
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let (cmin, cmax, rmin, rmax) = (
        ((mj_min(sx[0], sx[1]).floor() as i64) - 1).max(0),
        ((mj_max(sx[0], sx[1]).ceil() as i64) + 1).min(ncol as i64 - 1),
        ((mj_min(sy[0], sy[1]).floor() as i64) - 1).max(0),
        ((mj_max(sy[0], sy[1]).ceil() as i64) + 1).min(nrow as i64 - 1),
    );

    // The height at column `c`, row `r` (within the grid), and the grid
    // point there.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let height = |c: i64, r: i64| hfield.get(c as usize, r as usize).unwrap_or(0.0);
    #[allow(clippy::cast_precision_loss)]
    let corner = |c: i64, r: i64| {
        [
            dx * c as f64 - size[0],
            dy * r as f64 - size[1],
            height(c, r),
        ]
    };
    for r in rmin..rmax {
        for c in cmin..cmax {
            for v in [
                [corner(c, r), corner(c + 1, r), corner(c + 1, r + 1)],
                [corner(c, r), corner(c + 1, r + 1), corner(c, r + 1)],
            ] {
                let sol = ray_triangle(&v, &lpnt, &lvec, &b0, &b1);
                if sol >= 0.0 && (x < 0.0 || sol < x) {
                    x = sol;
                }
            }
        }
    }

    // MuJoCo's elevation at column `c`, row `r`: the height unscaled, in
    // single precision.
    #[allow(clippy::cast_possible_truncation)]
    let elevation =
        |c: usize, r: usize| f64::from((hfield.get(c, r).unwrap_or(0.0) / size[2]) as f32);
    // A rounded index within the grid (`y0` is whole and within it); a cast
    // saturates and takes a NaN to 0.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let index = |y: f64| mju_round(y) as usize;
    for (i, &a) in all.iter().enumerate().take(4) {
        if a >= 0.0 && (a < x || x < 0.0) {
            let z = (lpnt[2] + a * lvec[2]) / size[2];
            // The side's edge between the grid points on either side of the
            // crossing: along y on the −x and +x sides, along x on the others.
            let (y, y0, z0, z1) = if i < 2 {
                let y = (lpnt[1] + a * lvec[1] + size[1]) / dy;
                let y0 = mj_max(0.0, mj_min(rows - 1.0, y.floor()));
                let c = if i == 1 { ncol - 1 } else { 0 };
                (
                    y,
                    y0,
                    elevation(c, index(y0)),
                    elevation(c, index(y0 + 1.0)),
                )
            } else {
                let y = (lpnt[0] + a * lvec[0] + size[0]) / dx;
                let y0 = mj_max(0.0, mj_min(cols - 1.0, y.floor()));
                let r = if i == 3 { nrow - 1 } else { 0 };
                (
                    y,
                    y0,
                    elevation(index(y0), r),
                    elevation(index(y0 + 1.0), r),
                )
            };
            if z < z0 * (y0 + 1.0 - y) + z1 * (y - y0) {
                x = a;
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

    use super::mj_ray_hfield;
    use crate::heightfield::HeightFieldData;

    fn golden() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../tests/assets/golden/ray/ray.json"
        );
        let text = std::fs::read_to_string(path).expect("the ray golden reads");
        serde_json::from_str(&text).expect("the ray golden parses")
    }

    fn numbers(v: &Value) -> Vec<f64> {
        v.as_array()
            .expect("an array")
            .iter()
            .map(|x| x.as_f64().expect("a number"))
            .collect()
    }

    fn vector(v: &Value) -> Vector3<f64> {
        Vector3::from_column_slice(&numbers(v))
    }

    /// The ray golden's `hfield` rays (`gen_ray_reference.py`: 400 a field
    /// over five fields, from above, below, beside and inside them, unturned
    /// and turned, and 16 on the edges of its tests) meet each field at
    /// MuJoCo's distance, or miss it as MuJoCo's does (−1), bit for bit: each
    /// field built from MuJoCo's elevations (`hfield_data`) scaled by its top,
    /// as sim-mjcf does not load them yet (book Rigid-loading L45).
    #[test]
    fn rays_meet_each_height_field_as_mujoco() {
        let golden = golden();
        let fields = golden["hfield_fields"].as_object().expect("fields");
        let built: Vec<(&String, HeightFieldData, [f64; 4])> = fields
            .iter()
            .map(|(name, field)| {
                let size = numbers(&field["size"]);
                let count =
                    |v: &Value| usize::try_from(v.as_u64().expect("a count")).expect("a count");
                let (nrow, ncol) = (count(&field["nrow"]), count(&field["ncol"]));
                let heights = numbers(&field["data"])
                    .iter()
                    .map(|e| e * size[2])
                    .collect();
                (
                    name,
                    HeightFieldData::new(heights, ncol, nrow, 1.0),
                    [size[0], size[1], size[2], size[3]],
                )
            })
            .collect();
        let rays = golden["hfield"].as_array().expect("rays");
        assert_eq!(rays.len(), 5 * 400 + 16, "every ray read");
        let mut differ = Vec::new();
        let mut hits = 0;
        for (i, ray) in rays.iter().enumerate() {
            let name = ray["field"].as_str().expect("a field");
            let (_, hfield, size) = built
                .iter()
                .find(|(n, _, _)| n.as_str() == name)
                .expect("the ray's field");
            let ours = mj_ray_hfield(
                hfield,
                size,
                &vector(&ray["pos"]),
                &Matrix3::from_row_slice(&numbers(&ray["mat"])),
                &vector(&ray["pnt"]),
                &vector(&ray["vec"]),
            );
            let theirs = ray["x"].as_f64().expect("a distance");
            hits += usize::from(theirs >= 0.0);
            if ours.to_bits() != theirs.to_bits() {
                differ.push(format!("ray {i} ({name}): {ours:e}, MuJoCo {theirs:e}"));
            }
        }
        assert!(hits > 0 && hits < rays.len(), "hits and misses: {hits}");
        assert!(
            differ.is_empty(),
            "{} differ:\n{}",
            differ.len(),
            differ.join("\n")
        );
    }
}
