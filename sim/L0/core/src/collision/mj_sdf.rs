//! MuJoCo 3.5.0's collision of a geom with a signed distance field geom that
//! has no plugin, the field the octree MuJoCo's compiler makes over the
//! geom's mesh (`mjc_SDF`, `engine_collision_sdf.c:203-788`), ported
//! statement for statement: each geom type's distance and gradient
//! (`geomDistance`, `geomGradient`), the three ways `mjc_SDF` combines two
//! fields (`mjc_distance`, `mjc_gradient`), the gradient descent from
//! Halton starting points (`stepGradient`) and the contact on the surface
//! midway between the two (`addContact`); with the pose arithmetic MuJoCo
//! does it in (`mju_mat2Quat`, `mju_negPose`, `mju_mulPose`, `mapPose`) and
//! `mju_Halton`.
//!
//! The pair is taken in MuJoCo's order: the field last (the higher
//! `mjtGeom`), two fields as given. An octree read finds no leaf for a point
//! so far from the mesh that it rounds outside the octree's box, where
//! MuJoCo stops with "Node not found" ([`MeshOctree::distance`]): a starting
//! point whose descent reads one gives no contact.

use nalgebra::{Matrix3, Vector3};

use super::mj_primitive::{Mat9, add3, dot3, mat_t_vec, mat_vec, normalize3, scl3};
use crate::constraint::impedance::MJ_MINVAL;
use crate::mesh::MeshOctree;
use crate::quat::{mul_quat, neg_quat, normalize4, quat_to_mat, rot_vec_quat};
use crate::types::{GeomType, Model, make_frame};

/// `mjMAXVAL` (`mjmodel.h:26`).
const MJ_MAXVAL: f64 = 1e10;

/// `mju_max`: `a` where `a >= b`, else `b`.
fn max(a: f64, b: f64) -> f64 {
    if a >= b { a } else { b }
}

/// `mju_min`: `a` where `a <= b`, else `b`.
fn min(a: f64, b: f64) -> f64 {
    if a <= b { a } else { b }
}

/// `mju_clip`.
fn clip(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// `mju_norm3`.
fn norm3(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// `mju_Halton` (`engine_util_misc.c:1859-1873`): the `index`th point of
/// van der Corput's sequence in `base`.
fn halton(index: usize, base: usize) -> f64 {
    #[allow(clippy::cast_precision_loss)] // MuJoCo's int to double: a base and a digit
    let as_f64 = |n: usize| n as f64;
    let mut n0 = index;
    let b = as_f64(base);
    let mut f = 1.0 / b;
    let mut hn = 0.0;
    while n0 > 0 {
        let n1 = n0 / base;
        let r = n0 - n1 * base;
        hn += f * as_f64(r);
        f /= b;
        n0 = n1;
    }
    hn
}

/// `mju_mat2Quat` (`engine_util_spatial.c:187-221`): the quaternion of the
/// row-major rotation `mat`, from its largest component, normalized.
fn mat2quat(mat: &Mat9) -> [f64; 4] {
    let mut q = [0.0; 4];
    if mat[0] + mat[4] + mat[8] > 0.0 {
        q[0] = 0.5 * (1.0 + mat[0] + mat[4] + mat[8]).sqrt();
        q[1] = 0.25 * (mat[7] - mat[5]) / q[0];
        q[2] = 0.25 * (mat[2] - mat[6]) / q[0];
        q[3] = 0.25 * (mat[3] - mat[1]) / q[0];
    } else if mat[0] > mat[4] && mat[0] > mat[8] {
        q[1] = 0.5 * (1.0 + mat[0] - mat[4] - mat[8]).sqrt();
        q[0] = 0.25 * (mat[7] - mat[5]) / q[1];
        q[2] = 0.25 * (mat[1] + mat[3]) / q[1];
        q[3] = 0.25 * (mat[2] + mat[6]) / q[1];
    } else if mat[4] > mat[8] {
        q[2] = 0.5 * (1.0 - mat[0] + mat[4] - mat[8]).sqrt();
        q[0] = 0.25 * (mat[2] - mat[6]) / q[2];
        q[1] = 0.25 * (mat[1] + mat[3]) / q[2];
        q[3] = 0.25 * (mat[5] + mat[7]) / q[2];
    } else {
        q[3] = 0.5 * (1.0 - mat[0] - mat[4] + mat[8]).sqrt();
        q[0] = 0.25 * (mat[3] - mat[1]) / q[3];
        q[1] = 0.25 * (mat[2] + mat[6]) / q[3];
        q[2] = 0.25 * (mat[5] + mat[7]) / q[3];
    }
    normalize4(&mut q);
    q
}

/// `mju_rotVecQuat` (`engine_util_spatial.c:28-53`): `vec` rotated by
/// `quat`, zero for a zero `vec` (where `mji_rotVecQuat`, [`rot_vec_quat`],
/// can give a negative zero).
// MuJoCo tests for a zero vector exactly, so the comparison is too.
#[allow(clippy::float_cmp)]
fn rot_vec_quat_zero(vec: &[f64; 3], quat: &[f64; 4]) -> [f64; 3] {
    if vec[0] == 0.0 && vec[1] == 0.0 && vec[2] == 0.0 {
        return [0.0; 3];
    }
    rot_vec_quat(vec, quat)
}

/// `mju_negPose` (`engine_util_spatial.c:337-344`): the inverse of the pose.
fn neg_pose(pos: &[f64; 3], quat: &[f64; 4]) -> ([f64; 3], [f64; 4]) {
    let quatres = neg_quat(quat);
    let posres = scl3(&rot_vec_quat(pos, &quatres), -1.0);
    (posres, quatres)
}

/// `mju_mulPose` (`engine_util_spatial.c:323-334`): pose 1 then pose 2, the
/// quaternion normalized.
fn mul_pose(
    pos1: &[f64; 3],
    quat1: &[f64; 4],
    pos2: &[f64; 3],
    quat2: &[f64; 4],
) -> ([f64; 3], [f64; 4]) {
    let mut quatres = mul_quat(quat1, quat2);
    normalize4(&mut quatres);
    let posres = add3(&rot_vec_quat(pos2, quat1), pos1);
    (posres, quatres)
}

/// `mapPose` (`engine_collision_sdf.c:535-542`): frame 1 in frame 2, as
/// `(pos2, quat2)⁻¹ ∘ (pos1, quat1)`, the turn as a matrix.
fn map_pose(
    xpos1: &[f64; 3],
    xquat1: &[f64; 4],
    xpos2: &[f64; 3],
    xquat2: &[f64; 4],
) -> ([f64; 3], Mat9) {
    let (negpos, negquat) = neg_pose(xpos2, xquat2);
    let (pos12, quat12) = mul_pose(&negpos, &negquat, xpos1, xquat1);
    (pos12, quat_to_mat(&quat12))
}

/// A mesh geom as `geomDistance` reads it in a pair with a field
/// (`engine_collision_sdf.c:281-295`).
#[derive(Clone, Copy)]
pub(super) struct MeshField<'a> {
    /// The octree MuJoCo reads: mesh number `g1`'s, `g1` the mesh GEOM's id
    /// (`mjc_SDF` sets `instance[1] = g1`, `:724`, and `geomDistance` reads
    /// `mesh_octnum[i]`, `:282`; ledger L442). `None` where that mesh has
    /// none, and where `g1` is past the model's meshes (MuJoCo reads past
    /// `mesh_octnum` there).
    octree: Option<&'a MeshOctree>,
    model: &'a Model,
    /// The geom's own mesh, which `mj_rayMesh` casts.
    mesh: usize,
    pos: &'a Vector3<f64>,
    mat: &'a Matrix3<f64>,
}

/// One of a pair's two fields, as `geomDistance` and `geomGradient` read it.
#[derive(Clone, Copy)]
pub(super) enum Field<'a> {
    /// A signed distance field geom with no plugin: its mesh's octree
    /// (`oct_distance(m, x, geom_dataid)`).
    Octree(&'a MeshOctree),
    /// A sphere, a capsule, an ellipsoid, a cylinder or a box, and its
    /// `geom_size`.
    Primitive(GeomType, [f64; 3]),
    /// A mesh geom.
    Mesh(MeshField<'a>),
}

impl MeshField<'_> {
    /// `mj_rayMesh` from the point `x` (in the geom's frame) moved to the
    /// world, along the vector to the world's origin; and MuJoCo's test of
    /// "outside", `dist > r`: the hit's distance in lengths of that vector
    /// against the vector's length (`engine_collision_sdf.c:290`, `:418`).
    fn ray_toward_origin(&self, x: &[f64; 3]) -> (Vector3<f64>, Vector3<f64>, f64, bool) {
        let mat: Mat9 = std::array::from_fn(|i| self.mat[(i / 3, i % 3)]);
        let a = add3(&mat_vec(&mat, x), &[self.pos.x, self.pos.y, self.pos.z]);
        let dir = [-a[0], -a[1], -a[2]];
        let r = norm3(&dir);
        let a = Vector3::from(a);
        let dir = Vector3::from(dir);
        let dist = crate::sensor::mj_ray_mesh(self.model, self.mesh, self.pos, self.mat, &a, &dir);
        (a, dir, dist, dist > r)
    }
}

impl Field<'_> {
    /// `geomDistance` (`engine_collision_sdf.c:218-301`) at `x`, in the
    /// field's frame.
    // MuJoCo's `mju_sqrt(x·x + y·y)`, not `hypot`.
    #[allow(clippy::imprecise_flops)]
    fn distance(&self, x: &[f64; 3]) -> Option<f64> {
        let mut a = [0.0; 3];
        let mut b = [0.0; 3];
        Some(match *self {
            Self::Octree(octree) => octree.distance(*x)?,
            Self::Mesh(mesh) => {
                if let Some(octree) = mesh.octree {
                    octree.distance(*x)?
                } else {
                    let (a, dir, dist, outside) = mesh.ray_toward_origin(x);
                    if outside {
                        let dir = dir * -1.0;
                        -crate::sensor::mj_ray_mesh(
                            mesh.model, mesh.mesh, mesh.pos, mesh.mat, &a, &dir,
                        )
                    } else {
                        dist
                    }
                }
            }
            Self::Primitive(geom_type, size) => match geom_type {
                GeomType::Sphere => norm3(x) - size[0],
                GeomType::Box => {
                    a[0] = x[0].abs() - size[0];
                    a[1] = x[1].abs() - size[1];
                    a[2] = x[2].abs() - size[2];
                    if a[0] >= 0.0 || a[1] >= 0.0 || a[2] >= 0.0 {
                        b[0] = max(a[0], 0.0);
                        b[1] = max(a[1], 0.0);
                        b[2] = max(a[2], 0.0);
                        return Some(norm3(&b) + min(max(a[0], max(a[1], a[2])), 0.0));
                    }
                    radial_field(&mut b, &a, x, &size);
                    let t = [-a[0] / b[0].abs(), -a[1] / b[1].abs(), -a[2] / b[2].abs()];
                    -min(t[0], min(t[1], t[2])) * norm3(&b)
                }
                GeomType::Capsule => {
                    a[0] = x[0];
                    a[1] = x[1];
                    a[2] = x[2] - clip(x[2], -size[1], size[1]);
                    norm3(&a) - size[0]
                }
                GeomType::Ellipsoid => {
                    a[0] = x[0] / size[0];
                    a[1] = x[1] / size[1];
                    a[2] = x[2] / size[2];
                    b[0] = a[0] / size[0];
                    b[1] = a[1] / size[1];
                    b[2] = a[2] / size[2];
                    let k0 = norm3(&a);
                    let k1 = norm3(&b);
                    k0 * (k0 - 1.0) / k1
                }
                GeomType::Cylinder => {
                    a[0] = (x[0] * x[0] + x[1] * x[1]).sqrt() - size[0];
                    a[1] = x[2].abs() - size[1];
                    b[0] = max(a[0], 0.0);
                    b[1] = max(a[1], 0.0);
                    // `mju_norm(b, 2)`: `mju_dot`'s two-entry tail, added to 0.
                    min(max(a[0], a[1]), 0.0) + (0.0 + (b[0] * b[0] + b[1] * b[1])).sqrt()
                }
                // `mjc_SDF` is not given a plane, a height field or a mesh
                // as a primitive (`engine_collision_driver.c:41-52`).
                GeomType::Plane | GeomType::Hfield | GeomType::Mesh | GeomType::Sdf => {
                    unreachable!("geomDistance of a {geom_type:?} as a primitive")
                }
            },
        })
    }

    /// `geomGradient` (`engine_collision_sdf.c:304-427`) at `x`, in the
    /// field's frame.
    // MuJoCo's names (`a`, `b`, `c`, `e`, `k`, `l`, `j`), its ternaries for
    // `k`, `l` and `j`, and its `mju_sqrt(x·x + y·y)`, not `hypot`.
    #[allow(
        clippy::many_single_char_names,
        clippy::bool_to_int_with_if,
        clippy::imprecise_flops
    )]
    fn gradient(&self, x: &[f64; 3]) -> Option<[f64; 3]> {
        let mut gradient = [0.0; 3];
        let mut a = [0.0; 3];
        let mut b = [0.0; 3];
        match *self {
            Self::Octree(octree) => return octree.gradient(*x),
            Self::Mesh(mesh) => {
                if let Some(octree) = mesh.octree {
                    return octree.gradient(*x);
                }
                let (_, _, _, outside) = mesh.ray_toward_origin(x);
                let s = if outside { 1.0 } else { -1.0 };
                gradient = [s, s, s];
            }
            Self::Primitive(geom_type, size) => match geom_type {
                GeomType::Sphere => {
                    gradient = *x;
                    let c = norm3(x);
                    gradient[0] *= 1.0 / c;
                    gradient[1] *= 1.0 / c;
                    gradient[2] *= 1.0 / c;
                }
                GeomType::Box => {
                    a[0] = x[0].abs() - size[0];
                    a[1] = x[1].abs() - size[1];
                    a[2] = x[2].abs() - size[2];
                    let k = if a[0] > a[1] { 0 } else { 1 };
                    let l = if a[2] > a[k] { 2 } else { k };
                    if a[l] < 0.0 {
                        radial_field(&mut gradient, &a, x, &size);
                    } else {
                        b[0] = max(a[0], 0.0);
                        b[1] = max(a[1], 0.0);
                        b[2] = max(a[2], 0.0);
                        let c = norm3(&b);
                        for i in 0..3 {
                            gradient[i] = if a[i] > 0.0 {
                                b[i] / c * x[i] / x[i].abs()
                            } else {
                                0.0
                            };
                        }
                    }
                }
                GeomType::Capsule => {
                    a[0] = x[0];
                    a[1] = x[1];
                    a[2] = x[2] - clip(x[2], -size[1], size[1]);
                    let c = norm3(&a);
                    gradient = [a[0] / c, a[1] / c, a[2] / c];
                }
                GeomType::Ellipsoid => {
                    a[0] = x[0] / size[0];
                    a[1] = x[1] / size[1];
                    a[2] = x[2] / size[2];
                    b[0] = a[0] / size[0];
                    b[1] = a[1] / size[1];
                    b[2] = a[2] / size[2];
                    let k0 = norm3(&a);
                    let k1 = norm3(&b);
                    let inv_k0 = 1.0 / k0;
                    let inv_k1 = 1.0 / k1;
                    let gk0 = [b[0] * inv_k0, b[1] * inv_k0, b[2] * inv_k0];
                    let gk1 = [
                        b[0] * inv_k1 / (size[0] * size[0]),
                        b[1] * inv_k1 / (size[1] * size[1]),
                        b[2] * inv_k1 / (size[2] * size[2]),
                    ];
                    let df_dk0 = (2.0 * k0 - 1.0) * inv_k1;
                    let df_dk1 = k0 * (k0 - 1.0) * inv_k1 * inv_k1;
                    for i in 0..3 {
                        gradient[i] = gk0[i] * df_dk0 - gk1[i] * df_dk1;
                    }
                    normalize3(&mut gradient);
                }
                GeomType::Cylinder => {
                    let c = (x[0] * x[0] + x[1] * x[1]).sqrt();
                    let e = x[2].abs();
                    a[0] = c - size[0];
                    a[1] = e - size[1];
                    let grada = [
                        x[0] / max(c, 1.0 / MJ_MAXVAL),
                        x[1] / max(c, 1.0 / MJ_MAXVAL),
                        x[2] / max(e, 1.0 / MJ_MAXVAL),
                    ];
                    let j = if a[0] > a[1] { 0 } else { 1 };
                    if a[j] < 0.0 {
                        gradient[0] = if j == 0 { grada[0] } else { 0.0 };
                        gradient[1] = if j == 0 { grada[1] } else { 0.0 };
                        gradient[2] = if j == 1 { grada[2] } else { 0.0 };
                    } else {
                        b[0] = max(a[0], 0.0);
                        b[1] = max(a[1], 0.0);
                        let bnorm =
                            max((0.0 + (b[0] * b[0] + b[1] * b[1])).sqrt(), 1.0 / MJ_MAXVAL);
                        gradient[0] = grada[0] * b[0] / bnorm;
                        gradient[1] = grada[1] * b[0] / bnorm;
                        gradient[2] = grada[2] * b[1] / bnorm;
                    }
                }
                GeomType::Plane | GeomType::Hfield | GeomType::Mesh | GeomType::Sdf => {
                    unreachable!("geomGradient of a {geom_type:?} as a primitive")
                }
            },
        }
        Some(gradient)
    }
}

/// `radialField3d` (`engine_collision_sdf.c:205-216`): from inside a box,
/// the unit vector along `-size / a` (each half size over the point's depth
/// below that face), signed by the point's octant.
fn radial_field(field: &mut [f64; 3], a: &[f64; 3], x: &[f64; 3], size: &[f64; 3]) {
    field[0] = -size[0] / a[0];
    field[1] = -size[1] / a[1];
    field[2] = -size[2] / a[2];
    normalize3(field);
    for i in 0..3 {
        if x[i] < 0.0 {
            field[i] = -field[i];
        }
    }
}

/// How `mjc_distance` combines the two fields (`mjtSDFType`).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Combine {
    /// The larger of the two (`mjSDFTYPE_INTERSECTION`).
    Intersection,
    /// The first less the second (`mjSDFTYPE_MIDSURFACE`).
    Midsurface,
    /// `A + B + |max(A, B)|` (`mjSDFTYPE_COLLISION`).
    Collision,
}

/// Two fields as `mjc_SDF` builds its `mjSDF`: the field geom's first, the
/// other's second, a point `x` in the first's frame read in the second's at
/// `relmat · x + relpos`.
struct Pair<'a> {
    fields: [Field<'a>; 2],
    relpos: [f64; 3],
    relmat: Mat9,
}

impl Pair<'_> {
    /// `x` in the second field's frame.
    fn second(&self, x: &[f64; 3]) -> [f64; 3] {
        add3(&mat_vec(&self.relmat, x), &self.relpos)
    }

    /// `mjc_distance` (`engine_collision_sdf.c:433-463`).
    fn distance(&self, combine: Combine, x: &[f64; 3]) -> Option<f64> {
        let y = self.second(x);
        let a = self.fields[0].distance(x)?;
        let b = self.fields[1].distance(&y)?;
        Some(match combine {
            Combine::Intersection => max(a, b),
            Combine::Midsurface => a - b,
            Combine::Collision => a + b + max(a, b).abs(),
        })
    }

    /// `mjc_gradient` (`engine_collision_sdf.c:467-517`).
    // MuJoCo's names (`x`, `y`, `a`, `b`, `s`).
    #[allow(clippy::many_single_char_names)]
    fn gradient(&self, combine: Combine, x: &[f64; 3]) -> Option<[f64; 3]> {
        let y = self.second(x);
        Some(match combine {
            Combine::Intersection => {
                let first = self.fields[0].distance(x)? > self.fields[1].distance(&y)?;
                if first {
                    self.fields[0].gradient(x)?
                } else {
                    mat_t_vec(&self.relmat, &self.fields[1].gradient(&y)?)
                }
            }
            Combine::Midsurface => {
                let mut grad1 = self.fields[0].gradient(x)?;
                normalize3(&mut grad1);
                let mut grad2 = mat_t_vec(&self.relmat, &self.fields[1].gradient(&y)?);
                normalize3(&mut grad2);
                let mut gradient = [
                    grad1[0] - grad2[0],
                    grad1[1] - grad2[1],
                    grad1[2] - grad2[2],
                ];
                normalize3(&mut gradient);
                gradient
            }
            Combine::Collision => {
                let a = self.fields[0].distance(x)?;
                let b = self.fields[1].distance(&y)?;
                let grad1 = self.fields[0].gradient(x)?;
                let grad2 = mat_t_vec(&self.relmat, &self.fields[1].gradient(&y)?);
                let mut gradient = add3(&grad1, &grad2);
                let larger = if a > b { grad1 } else { grad2 };
                let s = if max(a, b) > 0.0 { 1.0 } else { -1.0 };
                for i in 0..3 {
                    gradient[i] += larger[i] * s;
                }
                gradient
            }
        })
    }

    /// `stepGradient` (`engine_collision_sdf.c:589-634`): up to `niter`
    /// steps down the combined field's gradient from `x`, each a
    /// backtracking line search (Armijo's condition, `c` 0.1; the gradient's
    /// factor 1 first, halved while above 1e-4, so 2⁻¹⁴ last); stops early
    /// where a step raises the field, or with mjMAXVAL where the gradient is
    /// not finite or past mjMAXVAL. The field at the last point tried.
    fn step_gradient(&self, combine: Combine, x: &mut [f64; 3], niter: usize) -> Option<f64> {
        const C: f64 = 0.1;
        const RHO: f64 = 0.5;
        const AMIN: f64 = 1e-4;
        let mut dist = MJ_MAXVAL;
        for _ in 0..niter {
            let mut alpha = 2.0;
            let grad = self.gradient(combine, x)?;
            // NaN or past mjMAXVAL either way: outside the range.
            if grad.iter().any(|g| !(-MJ_MAXVAL..=MJ_MAXVAL).contains(g)) {
                return Some(MJ_MAXVAL);
            }
            let x0 = *x;
            let dist0 = self.distance(combine, &x0)?;
            let mut wolfe = -C * alpha * dot3(&grad, &grad);
            loop {
                alpha *= RHO;
                wolfe *= RHO;
                for i in 0..3 {
                    x[i] = x0[i] + -alpha * grad[i];
                }
                dist = self.distance(combine, x)?;
                if !(alpha > AMIN && dist - dist0 > wolfe) {
                    break;
                }
            }
            if dist0 < dist {
                return Some(dist);
            }
        }
        Some(dist)
    }
}

/// A contact as `mjc_SDF` writes it: its distance (at most 0), its position,
/// and its frame's normal (from the first geom to the field) and first
/// tangent as `mju_makeFrame` completes them.
pub struct FieldContact {
    pub dist: f64,
    pub pos: [f64; 3],
    pub normal: [f64; 3],
    pub tangent: [f64; 3],
}

/// A geom of the pair as `mjc_SDF` reads it: its position, turn (row-major),
/// `geom_aabb` (centre and half sizes in its frame) and field.
pub(super) struct SdfGeom<'a> {
    pub(super) pos: [f64; 3],
    pub(super) mat: Mat9,
    pub(super) aabb: [f64; 6],
    pub(super) field: Field<'a>,
}

/// `mjc_SDF` (`engine_collision_sdf.c:653-788`): `g1` against the field
/// `g2`. Where the two geoms' boxes, in `g1`'s frame, overlap, `initpoints`
/// points spread over the overlap by Halton's sequence (bases 2, 3, 5), each
/// taken to `g2`'s frame, descend `iterations` steps on the two fields'
/// collision function, then one on their intersection; each that ends
/// inside both (its distance at most 0) and not within mjMINVAL of an
/// earlier one gives a contact, its normal the gradient of the midsurface
/// (`g2`'s field less `g1`'s) negated and turned to the world, its position
/// the point in the world moved half its distance along the normal.
///
/// Past 50 contacts (`mjMAXCONPAIR`), reachable with `initpoints` above 50,
/// MuJoCo writes its 51st found point past its 50-entry array and stops
/// with an error (registry D-SDF-INITPOINTS); every contact is listed here.
// MuJoCo's names (`offset2`, `offset12`, `rotation2`, `rotation12`).
#[allow(clippy::similar_names)]
pub(super) fn mjc_sdf(
    g1: &SdfGeom<'_>,
    g2: &SdfGeom<'_>,
    initpoints: usize,
    iterations: usize,
) -> Vec<FieldContact> {
    let (pos1, mat1, size1) = (&g1.pos, &g1.mat, &g1.aabb);
    let (pos2, mat2, size2) = (&g2.pos, &g2.mat, &g2.aabb);
    let mut aabb1 = [
        MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, -MJ_MAXVAL, -MJ_MAXVAL, -MJ_MAXVAL,
    ];
    let mut aabb2 = aabb1;
    let mut aabb = aabb1;

    let quat1 = mat2quat(mat1);
    let quat2 = mat2quat(mat2);
    let (offset2, rotation2) = map_pose(pos1, &quat1, pos1, &quat1);
    let (offset21, rotation21) = map_pose(pos2, &quat2, pos1, &quat1);
    let (offset12, rotation12) = map_pose(pos1, &quat1, pos2, &quat2);

    for i in 0..8 {
        let corner = |s: &[f64; 6], k: usize, bit: usize| {
            if i & bit != 0 {
                s[k] + s[k + 3]
            } else {
                s[k] - s[k + 3]
            }
        };
        let vec1 = [
            corner(size1, 0, 1),
            corner(size1, 1, 2),
            corner(size1, 2, 4),
        ];
        let vec2 = [
            corner(size2, 0, 1),
            corner(size2, 1, 2),
            corner(size2, 2, 4),
        ];
        let vec2 = add3(&mat_vec(&rotation21, &vec2), &offset21);
        for k in 0..3 {
            aabb1[k] = min(aabb1[k], vec1[k]);
            aabb1[3 + k] = max(aabb1[3 + k], vec1[k]);
            aabb2[k] = min(aabb2[k], vec2[k]);
            aabb2[3 + k] = max(aabb2[3 + k], vec2[k]);
        }
    }
    for k in 0..3 {
        aabb[k] = max(aabb1[k], aabb2[k]);
        aabb[3 + k] = min(aabb1[3 + k], aabb2[3 + k]);
    }
    if aabb[3] < aabb[0] || aabb[4] < aabb[1] || aabb[5] < aabb[2] {
        return Vec::new();
    }

    let pair = Pair {
        fields: [g2.field, g1.field],
        relpos: offset21,
        relmat: rotation21,
    };
    let mut points: Vec<[f64; 3]> = Vec::new();
    let mut contacts = Vec::new();
    for j in 0..initpoints {
        let mut x = [
            aabb[0] + (aabb[3] - aabb[0]) * halton(j, 2),
            aabb[1] + (aabb[4] - aabb[1]) * halton(j, 3),
            aabb[2] + (aabb[5] - aabb[2]) * halton(j, 5),
        ];
        let y = add3(&mat_vec(&rotation2, &x), &offset2);
        x = add3(&mat_vec(&rotation12, &y), &offset12);
        let Some(contact) = start(&pair, &mut x, iterations, pos2, &quat2, &points) else {
            continue;
        };
        points.push(x);
        contacts.push(contact);
    }
    contacts
}

/// One starting point `x` of `mjc_SDF`: its two descents, then
/// `addContact` (`engine_collision_sdf.c:559-585`, with `isknown`,
/// `:548-555`, against `points`): the contact, or `None` where it is
/// outside (its distance above 0), already known, or an octree read found
/// no leaf.
fn start(
    pair: &Pair<'_>,
    x: &mut [f64; 3],
    iterations: usize,
    pos2: &[f64; 3],
    quat2: &[f64; 4],
    points: &[[f64; 3]],
) -> Option<FieldContact> {
    pair.step_gradient(Combine::Collision, x, iterations)?;
    let dist = pair.step_gradient(Combine::Intersection, x, 1)?;
    if dist > 0.0 || points.iter().any(|p| dist3(x, p) < MJ_MINVAL) {
        return None;
    }
    let norm = scl3(&pair.gradient(Combine::Midsurface, x)?, -1.0);
    let world = rot_vec_quat_zero(&norm, quat2);
    let (normal, [tangent, _]) = make_frame(&Vector3::from(world), &Vector3::zeros());
    let normal = [normal.x, normal.y, normal.z];
    let vec = scl3(&normal, -dist / 2.0);
    let pos = add3(&add3(&rot_vec_quat_zero(x, quat2), pos2), &vec);
    Some(FieldContact {
        dist,
        pos,
        normal,
        tangent: [tangent.x, tangent.y, tangent.z],
    })
}

/// `mju_dist3`.
fn dist3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// The field of geom `g` in a pair with a loaded signed distance field
/// (`mjc_SDF`'s `instance` and `geomtype`): a loaded field's octree, a
/// primitive's size, or a mesh's octree as MuJoCo reads it (by the geom's
/// id, [`MeshField`]); `None` for a geom with no such field (a field with
/// a shape, a mesh geom with no mesh, a field whose mesh has no octree).
pub(super) fn field_of<'a>(
    model: &'a Model,
    g: usize,
    pos: &'a Vector3<f64>,
    mat: &'a Matrix3<f64>,
) -> Option<Field<'a>> {
    let size = model.geom_size[g];
    Some(match model.geom_type[g] {
        GeomType::Sdf => {
            if model.geom_shape[g].is_some() {
                return None;
            }
            Field::Octree(model.mesh_data[model.geom_mesh[g]?].octree()?)
        }
        GeomType::Mesh => Field::Mesh(MeshField {
            octree: model.mesh_data.get(g).and_then(|mesh| mesh.octree()),
            model,
            mesh: model.geom_mesh[g]?,
            pos,
            mat,
        }),
        t @ (GeomType::Sphere
        | GeomType::Capsule
        | GeomType::Ellipsoid
        | GeomType::Cylinder
        | GeomType::Box) => Field::Primitive(t, [size.x, size.y, size.z]),
        GeomType::Plane | GeomType::Hfield => return None,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

    use super::{Combine, Field, Mat9, Pair, halton, map_pose, mat2quat};
    use crate::types::GeomType;

    /// MuJoCo 3.5.0's own compiled functions on the same inputs
    /// (`tests/data/sdf_fields.txt`, written by
    /// `sim/L0/tests/scripts/gen_sdf_field_reference.py`, which says how).
    /// They take square roots and the four operations only, so every value
    /// but a NaN is held to the bit on every platform (not measured off
    /// macOS arm64); a NaN, whose sign Rust leaves to the platform (an
    /// invalid operation gives `0x7ff8…` on arm64, `0xfff8…` on x86-64), is
    /// held to be a NaN.
    const GOLDEN: &str = include_str!("../../tests/data/sdf_fields.txt");

    fn doubles(words: &[&str]) -> Vec<f64> {
        words
            .iter()
            .map(|w| f64::from_bits(u64::from_str_radix(w, 16).unwrap()))
            .collect()
    }

    /// A double's bits, a NaN's the canonical NaN's.
    fn bit(x: f64) -> u64 {
        if x.is_nan() {
            f64::NAN.to_bits()
        } else {
            x.to_bits()
        }
    }

    fn bits<const N: usize>(v: &[f64; N]) -> [u64; N] {
        v.map(bit)
    }

    fn lines(kind: &str) -> impl Iterator<Item = Vec<&'static str>> + '_ {
        GOLDEN
            .lines()
            .filter(move |l| l.split_whitespace().next() == Some(kind))
            .map(|l| l.split_whitespace().skip(1).collect())
    }

    /// `mju_Halton` for each index below 70 in each base `mjc_SDF` takes.
    #[test]
    fn halton_is_mujocos() {
        let mut n = 0;
        for w in lines("halton") {
            let (index, base): (usize, usize) = (w[0].parse().unwrap(), w[1].parse().unwrap());
            let theirs = doubles(&w[2..])[0];
            assert_eq!(bit(halton(index, base)), bit(theirs), "{index} {base}");
            n += 1;
        }
        assert_eq!(n, 210, "halton lines");
    }

    /// `mju_mat2Quat` of rotations reaching each of its four branches.
    #[test]
    fn mat2quat_is_mujocos() {
        let mut n = 0;
        for w in lines("mat2quat") {
            let v = doubles(&w);
            let mat: Mat9 = v[..9].try_into().unwrap();
            let quat: [f64; 4] = v[9..].try_into().unwrap();
            assert_eq!(bits(&mat2quat(&mat)), bits(&quat), "{mat:?}");
            n += 1;
        }
        assert_eq!(n, 40, "mat2quat lines");
    }

    /// `mapPose` as `mju_negPose`, `mju_mulPose` and `mju_quat2Mat` give it.
    // MuJoCo's names (`pos2`, `pos12`).
    #[allow(clippy::similar_names)]
    #[test]
    fn map_pose_is_mujocos() {
        let mut n = 0;
        for w in lines("mappose") {
            let v = doubles(&w);
            let (pos1, quat1): ([f64; 3], [f64; 4]) =
                (v[0..3].try_into().unwrap(), v[3..7].try_into().unwrap());
            let (pos2, quat2): ([f64; 3], [f64; 4]) =
                (v[7..10].try_into().unwrap(), v[10..14].try_into().unwrap());
            let (pos12, mat12) = map_pose(&pos1, &quat1, &pos2, &quat2);
            let theirs_pos: [f64; 3] = v[14..17].try_into().unwrap();
            let theirs_mat: Mat9 = v[17..26].try_into().unwrap();
            assert_eq!(
                (bits(&pos12), bits(&mat12)),
                (bits(&theirs_pos), bits(&theirs_mat)),
                "{w:?}"
            );
            n += 1;
        }
        assert_eq!(n, 30, "mappose lines");
    }

    /// `geomDistance` and `geomGradient` of each primitive (`single`), and
    /// `mjc_distance` and `mjc_gradient` of two primitives under each other
    /// combination, at each golden point.
    #[test]
    fn each_field_is_mujocos() {
        let geoms: Vec<Field<'static>> = lines("geom")
            .map(|w| {
                let geom_type = match w[1] {
                    "sphere" => GeomType::Sphere,
                    "capsule" => GeomType::Capsule,
                    "ellipsoid" => GeomType::Ellipsoid,
                    "cylinder" => GeomType::Cylinder,
                    "box" => GeomType::Box,
                    other => panic!("geom type {other}"),
                };
                Field::Primitive(geom_type, doubles(&w[2..]).try_into().unwrap())
            })
            .collect();
        assert_eq!(geoms.len(), 7, "geom lines");
        let mut failures = Vec::new();
        let mut counts = [0; 4];
        for w in lines("single") {
            let id: usize = w[0].parse().unwrap();
            let v = doubles(&w[1..]);
            let x: [f64; 3] = v[..3].try_into().unwrap();
            let theirs = (v[3], [v[4], v[5], v[6]]);
            let (dist, grad) = (
                geoms[id].distance(&x).unwrap(),
                geoms[id].gradient(&x).unwrap(),
            );
            if (bit(dist), bits(&grad)) != (bit(theirs.0), bits(&theirs.1)) {
                failures.push(format!(
                    "single {id} at {x:?}: ({dist}, {grad:?}) / {theirs:?}"
                ));
            }
            counts[0] += 1;
        }
        for w in lines("field") {
            let (id0, id1): (usize, usize) = (w[1].parse().unwrap(), w[2].parse().unwrap());
            let v = doubles(&w[3..]);
            let x: [f64; 3] = v[12..15].try_into().unwrap();
            let theirs = (v[15], [v[16], v[17], v[18]]);
            let (kind, slot) = match w[0] {
                "intersection" => (Combine::Intersection, 1),
                "midsurface" => (Combine::Midsurface, 2),
                "collision" => (Combine::Collision, 3),
                other => panic!("field kind {other}"),
            };
            counts[slot] += 1;
            let pair = Pair {
                fields: [geoms[id0], geoms[id1]],
                relpos: v[0..3].try_into().unwrap(),
                relmat: v[3..12].try_into().unwrap(),
            };
            let ours = (pair.distance(kind, &x), pair.gradient(kind, &x));
            let (dist, grad) = (ours.0.unwrap(), ours.1.unwrap());
            if (bit(dist), bits(&grad)) != (bit(theirs.0), bits(&theirs.1)) {
                failures.push(format!(
                    "{} {id0} {id1} at {x:?}: ({dist}, {grad:?}) / ({}, {:?})",
                    w[0], theirs.0, theirs.1
                ));
            }
        }
        assert_eq!(counts, [522, 126, 126, 125], "field lines by kind");
        assert!(
            failures.is_empty(),
            "{} differ:\n{}",
            failures.len(),
            failures.join("\n")
        );
    }
}
