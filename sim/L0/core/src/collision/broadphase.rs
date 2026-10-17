//! MuJoCo's broadphase (`mj_broadphase`, `engine_collision_driver.c:1154-1299`):
//! the pairs of bodies whose geoms the narrow phase tests.
//!
//! The world body, when it holds geoms, and each body welded to the world
//! that holds a plane, pair with every other body. The rest are bounded along
//! the three rows of the matrix `mju_eig3` makes of the covariance of the
//! geoms' (and flex vertices') positions (its columns are the eigenvectors,
//! so a row is not, in general, one), each body by its geoms' bounding
//! spheres, and swept and pruned along the first row with the bounds cast to
//! `f32` (`mj_SAP`, `:1021-1114`): bounds
//! equal in `f32` sort in the order the bodies are numbered, a lower body's
//! upper bound ahead of a higher body's lower bound, so two bodies that only
//! touch along that axis do not pair.

use crate::constraint::impedance::MJ_MINVAL;
use crate::quat::{mul_quat, normalize4, quat_to_mat};
use crate::types::{
    DISABLE_FILTERPARENT, Data, ENABLE_OVERRIDE, ENABLE_SLEEP, GeomType, Model, SleepState,
    disabled, enabled,
};

/// Each body's geoms in index order, from `geom_body`: the ranges
/// `body_geom_adr` and `body_geom_num` give where a body's geoms are numbered
/// together, as MuJoCo's always are and sim-mjcf's are. A model built in code
/// may number them otherwise: `Model::add_ground_plane` appends a world geom
/// after the other bodies' and widens the world's range over them.
pub(super) struct BodyGeoms {
    start: Vec<usize>,
    geoms: Vec<usize>,
}

impl BodyGeoms {
    pub(super) fn new(model: &Model) -> Self {
        let mut start = vec![0; model.nbody + 1];
        for &body in &model.geom_body {
            start[body + 1] += 1;
        }
        for body in 0..model.nbody {
            start[body + 1] += start[body];
        }
        let mut next = start.clone();
        let mut geoms = vec![0; model.geom_body.len()];
        for (geom, &body) in model.geom_body.iter().enumerate() {
            geoms[next[body]] = geom;
            next[body] += 1;
        }
        Self { start, geoms }
    }

    /// Body `body`'s geoms, in index order.
    pub(super) fn of(&self, body: usize) -> &[usize] {
        &self.geoms[self.start[body]..self.start[body + 1]]
    }
}

/// The pairs of bodies MuJoCo's broadphase passes on, each `(lower,
/// higher)`, in signature order; a pair may repeat, as in MuJoCo's list.
pub(super) fn body_pairs(model: &Model, data: &Data, geoms: &BodyGeoms) -> Vec<(usize, usize)> {
    let nbody = model.nbody;
    let filterparent = !disabled(model, DISABLE_FILTERPARENT);
    let sleep_filter = enabled(model, ENABLE_SLEEP) && data.nbody_awake < nbody;
    let asleep = |b: usize| sleep_filter && data.body_sleep_state[b] == SleepState::Asleep;
    let weld = |b: usize| model.body_weldid[b];
    let parent_weld = |b: usize| model.body_weldid[model.body_parent[model.body_weldid[b]]];

    // A body's contype and conaffinity: its geoms' together.
    let mut contype = vec![0; nbody];
    let mut conaffinity = vec![0; nbody];
    for g in 0..model.ngeom {
        contype[model.geom_body[g]] |= model.geom_contype[g];
        conaffinity[model.geom_body[g]] |= model.geom_conaffinity[g];
    }
    let can_collide = |b: usize| contype[b] != 0 || conaffinity[b] != 0;
    let mut pairs = Vec::new();
    let mut add_pair = |b1: usize, b2: usize| {
        if contype[b1] & conaffinity[b2] != 0 || contype[b2] & conaffinity[b1] != 0 {
            pairs.push((b1.min(b2), b1.max(b2)));
        }
    };

    // The world body with geoms, and world-welded bodies holding a plane,
    // with every body not welded to the world or asleep.
    for b1 in 0..nbody {
        if !can_collide(b1) {
            continue;
        }
        let world_like = (b1 == 0 && !geoms.of(0).is_empty())
            || (weld(b1) == 0 && has_plane(model, geoms.of(b1)));
        if !world_like {
            continue;
        }
        for b2 in 0..nbody {
            if can_collide(b2)
                && !filter_body_pair(
                    (0, 0, true),
                    (weld(b2), parent_weld(b2), asleep(b2)),
                    filterparent,
                )
            {
                add_pair(b1, b2);
            }
        }
    }

    // The covariance of the positions of geoms and flex vertices off the
    // world body; none, and only the pairs above.
    let mut points: Vec<[f64; 3]> = Vec::new();
    for g in (0..model.ngeom).filter(|&g| model.geom_body[g] != 0) {
        let p = data.geom_xpos[g];
        points.push([p.x, p.y, p.z]);
    }
    for v in (0..model.nflexvert).filter(|&v| model.flexvert_bodyid[v] != 0) {
        let p = data.flexvert_xpos[v];
        points.push([p.x, p.y, p.z]);
    }
    if points.is_empty() {
        return pairs;
    }
    #[allow(clippy::cast_precision_loss)] // a point count
    let inverse = 1.0 / points.len() as f64;
    let mut center = [0.0; 3];
    for p in &points {
        for i in 0..3 {
            center[i] += p[i];
        }
    }
    for c in &mut center {
        *c *= inverse;
    }
    let mut cov = [0.0; 9];
    for p in &points {
        update_cov(&mut cov, p, &center);
    }
    for c in &mut cov {
        *c *= inverse;
    }
    let frame = eig3(&cov);

    // The bodies (from 1) that can collide, each bounded along the frame's
    // axes by its geoms' spheres and margins.
    let bodies: Vec<usize> = (1..nbody).filter(|&b| can_collide(b)).collect();
    if bodies.len() > 1 {
        let aamm: Vec<[f64; 6]> = bodies
            .iter()
            .map(|&b| body_aamm(model, data, geoms.of(b), &frame))
            .collect();
        for (i1, i2) in sweep_and_prune(&aamm) {
            let (b1, b2) = (bodies[i1], bodies[i2]);
            if !filter_body_pair(
                (weld(b1), parent_weld(b1), asleep(b1)),
                (weld(b2), parent_weld(b2), asleep(b2)),
                filterparent,
            ) {
                add_pair(b1, b2);
            }
        }
    }

    pairs.sort_unstable();
    pairs
}

/// Whether the broadphase drops a pair of bodies, each given as its weld
/// body, its weld body's parent's weld body, and whether it is asleep
/// (`filterBodyPair`, `engine_collision_driver.c:160-186`): one weld body;
/// both asleep; one asleep and the other welded to the world; or, with the
/// parent filter on and neither welded to the world, one the other's parent.
fn filter_body_pair(
    (weld1, parent1, asleep1): (usize, usize, bool),
    (weld2, parent2, asleep2): (usize, usize, bool),
    filterparent: bool,
) -> bool {
    weld1 == weld2
        || (asleep1 && asleep2)
        || (asleep1 && weld2 == 0)
        || (asleep2 && weld1 == 0)
        || (filterparent && weld1 != 0 && weld2 != 0 && (weld1 == parent2 || weld2 == parent1))
}

fn has_plane(model: &Model, geoms: &[usize]) -> bool {
    geoms.iter().any(|&g| model.geom_type[g] == GeomType::Plane)
}

/// A geom's bounding radius as MuJoCo's: 0 for a plane (sim-core's is
/// infinite; ledger L173).
pub(super) fn rbound(model: &Model, geom: usize) -> f64 {
    if model.geom_type[geom] == GeomType::Plane {
        0.0
    } else {
        model.geom_rbound[geom]
    }
}

/// `updateCov` (`engine_collision_driver.c:1118-1135`).
fn update_cov(cov: &mut [f64; 9], p: &[f64; 3], center: &[f64; 3]) {
    let d = [p[0] - center[0], p[1] - center[1], p[2] - center[2]];
    let (d00, d01, d02) = (d[0] * d[0], d[0] * d[1], d[0] * d[2]);
    let (d11, d12, d22) = (d[1] * d[1], d[1] * d[2], d[2] * d[2]);
    for (c, add) in cov
        .iter_mut()
        .zip([d00, d01, d02, d01, d11, d12, d02, d12, d22])
    {
        *c += add;
    }
}

/// A body's bounds along the three rows of `frame`, `[min; 3, max; 3]`: over
/// its geoms, each centre's projection less and plus its bounding radius and
/// margin (`makeAAMM`, `engine_collision_driver.c:853-932`).
fn body_aamm(model: &Model, data: &Data, geoms: &[usize], frame: &[f64; 9]) -> [f64; 6] {
    let mut aamm = [0.0; 6];
    for (i, &geom) in geoms.iter().enumerate() {
        let margin = if enabled(model, ENABLE_OVERRIDE) {
            0.5 * model.o_margin
        } else {
            model.geom_margin[geom]
        };
        let p = data.geom_xpos[geom];
        let r = rbound(model, geom);
        let mut geom_aamm = [0.0; 6];
        for j in 0..3 {
            let center = p.x * frame[3 * j] + p.y * frame[3 * j + 1] + p.z * frame[3 * j + 2];
            geom_aamm[j] = center - r - margin;
            geom_aamm[j + 3] = center + r + margin;
        }
        if i == 0 {
            aamm = geom_aamm;
        } else {
            for j in 0..3 {
                // mju_min and mju_max.
                if geom_aamm[j] < aamm[j] {
                    aamm[j] = geom_aamm[j];
                }
                if geom_aamm[j + 3] > aamm[j + 3] {
                    aamm[j + 3] = geom_aamm[j + 3];
                }
            }
        }
    }
    aamm
}

/// The pairs `(i, j)` of `aamm` whose bounds overlap, swept along axis 0 with
/// its bounds cast to `f32` and sorted stably, the minima before the maxima
/// of each entry, and pruned on axes 1 and 2 (`mj_SAP`,
/// `engine_collision_driver.c:1021-1114`).
fn sweep_and_prune(aamm: &[[f64; 6]]) -> Vec<(usize, usize)> {
    // (value, entry, is the maximum)
    let mut sweep: Vec<(f32, usize, bool)> = Vec::with_capacity(2 * aamm.len());
    for (i, a) in aamm.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)] // MuJoCo sorts on f32
        {
            sweep.push((a[0] as f32, i, false));
            sweep.push((a[3] as f32, i, true));
        }
    }
    // MuJoCo's `SAPcmp`: less, equal, or else greater, compared exactly.
    #[allow(clippy::float_cmp)]
    sweep.sort_by(|a, b| {
        if a.0 < b.0 {
            std::cmp::Ordering::Less
        } else if a.0 == b.0 {
            std::cmp::Ordering::Equal
        } else {
            std::cmp::Ordering::Greater
        }
    });
    let mut active: Vec<usize> = Vec::new();
    let mut pairs = Vec::new();
    for &(_, i, is_max) in &sweep {
        if is_max {
            if let Some(k) = active.iter().position(|&a| a == i) {
                active.remove(k);
            }
            continue;
        }
        for &a in &active {
            let (p, q) = (&aamm[a], &aamm[i]);
            if p[1] > q[4] || q[1] > p[4] || p[2] > q[5] || q[2] > p[5] {
                continue;
            }
            pairs.push((a, i));
        }
        active.push(i);
    }
    pairs
}

/// MuJoCo's `mju_eig3` (`engine_util_solve.c:889-983`): the eigenvectors of
/// the symmetric `mat` by Jacobi rotations accumulated in a quaternion, the
/// eigenvalues sorted in decreasing order, as the columns of a row-major
/// matrix (the quaternion's, `mju_quat2Mat`). The broadphase takes its rows
/// as its axes, as MuJoCo's `mj_broadphase` does: a row is not, in general,
/// an eigenvector (`diag(0, 0, 2/3)` gives row 0 (0, −1, 0) and column 0
/// (0, 0, 1)).
fn eig3(mat: &[f64; 9]) -> [f64; 9] {
    const EIG_EPS: f64 = MJ_MINVAL * 1000.0;
    let mut quat = [1.0, 0.0, 0.0, 0.0];
    let mut eigval = [0.0; 3];
    for _ in 0..500 {
        let eigvec = quat_to_mat(&quat);
        let d = mul_mat_mat3(&mul_mat_t_mat3(&eigvec, mat), &eigvec);
        eigval = [d[0], d[4], d[8]];
        let (rk, ck, rotk) = if d[1].abs() > d[2].abs() && d[1].abs() > d[5].abs() {
            (0, 1, 2)
        } else if d[2].abs() > d[5].abs() {
            (0, 2, 1)
        } else {
            (1, 2, 0)
        };
        if d[3 * rk + ck].abs() < EIG_EPS {
            break;
        }
        let tau = (d[4 * ck] - d[4 * rk]) / (2.0 * d[3 * rk + ck]);
        let t = if tau >= 0.0 {
            1.0 / (tau + (1.0 + tau * tau).sqrt())
        } else {
            -1.0 / (-tau + (1.0 + tau * tau).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        if c > 1.0 - EIG_EPS {
            break;
        }
        let mut rotation = [0.0; 4];
        let s = (0.5 - 0.5 * c).sqrt();
        rotation[rotk + 1] = if tau >= 0.0 { -s } else { s };
        if rotk == 1 {
            rotation[rotk + 1] = -rotation[rotk + 1];
        }
        rotation[0] = (1.0 - rotation[rotk + 1] * rotation[rotk + 1]).sqrt();
        normalize4(&mut rotation);
        quat = mul_quat(&quat, &rotation);
        normalize4(&mut quat);
    }
    // Decreasing eigenvalues (bubble sort over 0, 1, 0).
    for j in 0..3 {
        let j1 = j % 2;
        if eigval[j1] + EIG_EPS < eigval[j1 + 1] {
            eigval.swap(j1, j1 + 1);
            // MuJoCo's literal, 4 ulps above `FRAC_1_SQRT_2`.
            #[allow(clippy::approx_constant)]
            let mut rotation = [0.707_106_781_186_548, 0.0, 0.0, 0.0];
            rotation[(j1 + 2) % 3 + 1] = rotation[0];
            quat = mul_quat(&quat, &rotation);
            normalize4(&mut quat);
        }
    }
    quat_to_mat(&quat)
}

/// `mji_mulMatTMat3`: `mat1ᵀ · mat2`, row-major.
fn mul_mat_t_mat3(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
    let mut r = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            r[3 * i + j] = a[i] * b[j] + a[3 + i] * b[3 + j] + a[6 + i] * b[6 + j];
        }
    }
    r
}

/// `mji_mulMatMat3`: `mat1 · mat2`, row-major.
fn mul_mat_mat3(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
    let mut r = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
        }
    }
    r
}

/// Whether MuJoCo's bounding-sphere filter drops the geom pair
/// (`mj_filterSphere`, `engine_collision_driver.c:139-156`): with neither a
/// plane, the centres farther apart than both radii and `margin`; with a
/// plane, the other geom's centre farther above it than its radius and
/// `margin`.
pub(super) fn filter_sphere(
    model: &Model,
    data: &Data,
    geom1: usize,
    geom2: usize,
    margin: f64,
) -> bool {
    let (r1, r2) = (rbound(model, geom1), rbound(model, geom2));
    if r1 > 0.0 && r2 > 0.0 {
        let d = data.geom_xpos[geom1] - data.geom_xpos[geom2];
        let bound = r1 + r2 + margin;
        return d.x * d.x + d.y * d.y + d.z * d.z > bound * bound;
    }
    let plane = |g: usize| model.geom_type[g] == GeomType::Plane;
    (plane(geom1) && r2 > 0.0 && plane_geom_dist(data, geom1, geom2) > margin + r2)
        || (plane(geom2) && r1 > 0.0 && plane_geom_dist(data, geom2, geom1) > margin + r1)
}

/// The signed distance of `geom`'s centre above `plane` (`planeGeomDist`,
/// `engine_collision_driver.c:71-78`).
fn plane_geom_dist(data: &Data, plane: usize, geom: usize) -> f64 {
    let n = data.geom_xmat[plane].column(2);
    let d = data.geom_xpos[geom] - data.geom_xpos[plane];
    d.x * n[0] + d.y * n[1] + d.z * n[2]
}
