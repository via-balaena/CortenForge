//! MuJoCo's rays through triangles (`engine_ray.c`): a height field's
//! (`mj_rayHfield`, `:561-738`) and a mesh's (`mj_rayMesh`, `:951-968`, down
//! its hierarchy, `mju_rayTree`, `:770-881`), each through `ray_triangle`
//! (`:131-200`). Each step is MuJoCo's, in its order, so that given MuJoCo's
//! inputs a ray grazing a triangle's edge goes the way MuJoCo's does.

use nalgebra::{Matrix3, Vector3};

use super::ray_geom::{ray_box, ray_map};
use crate::constraint::impedance::MJ_MINVAL;
use crate::heightfield::HeightFieldData;
use crate::mesh::TriangleMeshData;
use crate::quat::normalize3;
use crate::types::Model;

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
/// [`ray_triangle`] projects onto (`engine_ray.c:614-627`, `:797-809`): `b0` is (1, 1, 1)
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
/// load a field's elevations as MuJoCo's (normalised, the rows flipped: book
/// Rigid-loading L45); where a height is not one scaled, the sides' edge is
/// that rounding of it, up to an ulp of single precision from the triangles'
/// edge. A field of one row or column (which sim-mjcf refuses, ledger L208)
/// has no triangles; along its one row or column, a ray's place is 0, and
/// the side test's second elevation, past its row or column, is multiplied
/// by 0, as in MuJoCo (here it reads 0).
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
    if top_intersect < 0.0 {
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
    // cast to an integer, and the cell more, saturate, and the cast takes a
    // NaN to 0, where C's leave them undefined.
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let (cmin, cmax, rmin, rmax) = (
        (mj_min(sx[0], sx[1]).floor() as i64)
            .saturating_sub(1)
            .max(0),
        (mj_max(sx[0], sx[1]).ceil() as i64)
            .saturating_add(1)
            .min(ncol as i64 - 1),
        (mj_min(sy[0], sy[1]).floor() as i64)
            .saturating_sub(1)
            .max(0),
        (mj_max(sy[0], sy[1]).ceil() as i64)
            .saturating_add(1)
            .min(nrow as i64 - 1),
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
    // single precision; 0 past the grid, where a field of one row or column
    // reads it, times 0.
    #[allow(clippy::cast_possible_truncation)]
    let elevation =
        |c: usize, r: usize| f64::from((hfield.get(c, r).unwrap_or(0.0) / size[2]) as f32);
    // A rounded index (`y0` is whole; one past the grid on a field of one row
    // or column, which `elevation` reads as 0); a cast saturates and takes a
    // NaN to 0.
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

/// `mju_raySlab` (`engine_ray.c:741-766`): whether the ray from `src` in the
/// direction `dir`, both in the frame of the box `aabb` (`[cx, cy, cz, hx,
/// hy, hz]`), crosses it at or past its start: the latest of its entries
/// into the box's three slabs, from 0, before the earliest of its exits.
fn ray_slab(aabb: &[f64; 6], src: &[f64; 3], dir: &[f64; 3]) -> bool {
    let (mut tmin, mut tmax) = (0.0, f64::INFINITY);
    for d in 0..3 {
        let invdir = 1.0 / dir[d];
        let t1 = (aabb[d] - aabb[d + 3] - src[d]) * invdir;
        let t2 = (aabb[d] + aabb[d + 3] - src[d]) * invdir;
        let (minval, maxval) = if t1 < t2 { (t1, t2) } else { (t2, t1) };
        tmin = if tmin > minval { tmin } else { minval };
        tmax = if tmax < maxval { tmax } else { maxval };
    }
    tmin < tmax
}

/// The half-sizes of the box MuJoCo gives a mesh's geom (its `geom_size`,
/// `user_objects.cc:3765-3769`): along each axis, the larger magnitude of
/// the mesh's bounds.
fn mesh_box(mesh: &TriangleMeshData) -> [f64; 3] {
    let (lo, hi) = mesh.aabb();
    // `std::max(std::abs(a), std::abs(b))`.
    std::array::from_fn(|i| {
        let (a, b) = (lo[i].abs(), hi[i].abs());
        if a < b { b } else { a }
    })
}

/// `mj_rayMesh`: along a ray from `pnt` in the direction `vec`, the distance
/// in lengths of `vec` to mesh `mesh` of a geom centred at `xpos` and turned
/// by `xmat`, or −1. None where the ray misses the box MuJoCo gives the geom
/// (each half-size the larger magnitude of the mesh's bounds along that
/// axis, `user_objects.cc:3765-3769`); else `mju_rayTree`'s: down the mesh's
/// hierarchy (`mesh_bvhadr`), each node whose box the ray crosses
/// (`mju_raySlab`), the child pushed last taken first, each leaf's face
/// through [`ray_triangle`]; the nearest.
///
/// The mesh is as sim-core holds it: its vertices in double precision, where
/// MuJoCo's ray reads them in single precision (`mesh_vert`; the same for
/// vertices a float holds; ledger L281), and in its written frame, where
/// MuJoCo turns a mesh to its principal frame (book Rigid-loading L35). Its
/// box and its hierarchy's are over those vertices, where MuJoCo's are over
/// its double-precision ones before it rounds them (ledger L281), and its
/// hierarchy's split can differ where a face has no area (registry
/// D-MESH-SPLIT): a ray through a vertex or across an edge can then go
/// otherwise. A mesh with no hierarchy (no face, or a model whose
/// hierarchies were not built, [`Model::compute_body_bvhs`]) reads −1. A
/// model whose mesh was replaced after its hierarchy was built reads
/// through the old hierarchy, a leaf naming a face the mesh no longer has
/// passed over, until it is derived again.
pub(super) fn mj_ray_mesh(
    model: &Model,
    mesh: usize,
    xpos: &Vector3<f64>,
    xmat: &Matrix3<f64>,
    pnt: &Vector3<f64>,
    vec: &Vector3<f64>,
) -> f64 {
    let data = &model.mesh_data[mesh];
    if ray_box(xpos, xmat, &mesh_box(data), pnt, vec).0 < 0.0 {
        return -1.0;
    }
    let Some(adr) = model.mesh_bvhadr.get(mesh).copied().flatten() else {
        return -1.0;
    };

    let (lpnt, lvec) = ray_map(xpos, xmat, pnt, vec);
    let (b0, b1) = ray_basis(&lvec);
    let (vertices, faces) = (data.vertices(), data.triangles());
    let mut x = -1.0;
    let mut stack = vec![0];
    while let Some(node) = stack.pop() {
        if !ray_slab(&model.bvh_aabb[adr + node], &lpnt, &lvec) {
            continue;
        }
        if let Some(face) = model.bvh_nodeid[adr + node] {
            let Some(face) = faces.get(face) else {
                continue;
            };
            let v = face.map(|i| vertices[i as usize].coords.into());
            let sol = ray_triangle(&v, &lpnt, &lvec, &b0, &b1);
            if sol >= 0.0 && (x < 0.0 || sol < x) {
                x = sol;
            }
            continue;
        }
        stack.extend(model.bvh_child[adr + node].into_iter().flatten());
    }
    x
}

#[cfg(test)]
mod tests {
    // Test code: a malformed golden is a failed test.
    #![allow(clippy::expect_used, clippy::panic)]

    use nalgebra::{Matrix3, Vector3};
    use serde_json::Value;

    use super::{mj_ray_hfield, mj_ray_mesh};
    use crate::heightfield::HeightFieldData;
    use crate::types::Model;

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
    /// over seven fields, one of them a single row and one a single column,
    /// from above, below, beside and inside them, unturned and turned; 17 on
    /// the edges of its tests; and 480 across the sides of four fields at
    /// their edges' height, to within 2 ulps) meet each field at MuJoCo's
    /// distance, or miss it as MuJoCo's does (−1), bit for bit: each field
    /// built from MuJoCo's elevations (`hfield_data`) scaled by its top, as
    /// sim-mjcf does not load them yet (book Rigid-loading L45).
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
        assert_eq!(rays.len(), 7 * 400 + 17 + 4 * 120, "every ray read");
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

    /// The ray golden's `mesh` rays (`gen_ray_reference.py`: 300 a mesh over
    /// its eight meshes, the mesh unturned and turned, and 14 on the edges of
    /// three of the meshes MuJoCo keeps in their written frame) meet each mesh at
    /// MuJoCo's distance, or miss it as MuJoCo's does (−1), bit for bit: each
    /// mesh built from MuJoCo's vertices and faces as it compiles them
    /// (`mesh_models`), its hierarchy sim-core's (`each_mesh_hierarchy_is_mujocos`
    /// compares it), the geom at MuJoCo's pose. The drawn rays aim at points,
    /// not at vertices or edges, so they do not tell a tree or a box that
    /// differs from MuJoCo's at a vertex or an edge (ledger L281, registry
    /// D-MESH-SPLIT).
    #[test]
    fn rays_meet_each_mesh_as_mujoco() {
        let golden = golden();
        let models: Vec<(&String, Model)> = golden["mesh_models"]
            .as_object()
            .expect("mesh_models")
            .iter()
            .map(|(name, mesh)| {
                let face: Vec<usize> = mesh["face"]
                    .as_array()
                    .expect("faces")
                    .iter()
                    .map(|i| usize::try_from(i.as_u64().expect("a vertex")).expect("a vertex"))
                    .collect();
                (name, Model::one_mesh(&numbers(&mesh["vert"]), &face))
            })
            .collect();
        let rays = golden["mesh"].as_array().expect("rays");
        assert_eq!(rays.len(), 8 * 300 + 14, "every ray read");
        let mut differ = Vec::new();
        let mut hits = 0;
        for (i, ray) in rays.iter().enumerate() {
            let name = ray["mesh"].as_str().expect("a mesh");
            let (_, model) = models
                .iter()
                .find(|(n, _)| n.as_str() == name)
                .expect("the ray's mesh");
            let ours = mj_ray_mesh(
                model,
                0,
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

    /// The box `mj_ray_mesh` gives a mesh's geom is MuJoCo's `geom_size` bit
    /// for bit for each of the ray golden's meshes whose vertices and turn
    /// MuJoCo keeps as written, the pyramid among them, whose larger bound in
    /// magnitude along z (1.5) is not its half-extent (1.0) (the others MuJoCo
    /// turns, and builds its box over its double-precision vertices: ledger
    /// L281).
    #[test]
    fn a_mesh_geom_box_is_mujocos() {
        let golden = golden();
        let mut compared = 0;
        for (name, mesh) in golden["mesh_models"].as_object().expect("mesh_models") {
            if !mesh["vertices_as_written"].as_bool().expect("a flag") {
                continue;
            }
            let face: Vec<usize> = mesh["face"]
                .as_array()
                .expect("faces")
                .iter()
                .map(|i| usize::try_from(i.as_u64().expect("a vertex")).expect("a vertex"))
                .collect();
            let model = Model::one_mesh(&numbers(&mesh["vert"]), &face);
            let ours = super::mesh_box(&model.mesh_data[0]);
            let theirs = numbers(&mesh["size"]);
            assert!(
                ours.iter()
                    .zip(&theirs)
                    .all(|(a, b)| a.to_bits() == b.to_bits()),
                "{name}: {ours:?}, MuJoCo {theirs:?}"
            );
            compared += 1;
        }
        assert_eq!(compared, 5, "the meshes with their vertices as written");
    }
}
