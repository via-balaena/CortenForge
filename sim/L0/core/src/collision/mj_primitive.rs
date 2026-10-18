//! MuJoCo's collision functions for a sphere against a sphere, a capsule, a
//! cylinder and a box, ported statement for statement
//! (`engine_collision_primitive.c:25-385`, `engine_collision_box.c:20-92`),
//! with MuJoCo's helpers (`mju_normalize3`, `mju_clip`, `mju_clampVec`, the
//! `mji_*` products). Each function takes the sphere first, as MuJoCo's do
//! (its `mjtGeom` is the lower), and makes at most one contact, its normal
//! from the sphere to the other geom.

use nalgebra::{Matrix3, Vector3};

use crate::constraint::impedance::MJ_MINVAL;

/// A contact as MuJoCo's functions make it: its distance (negative in
/// penetration), its position, and its normal from the first geom to the
/// second.
pub(super) struct RawContact {
    pub(super) dist: f64,
    pub(super) pos: [f64; 3],
    pub(super) normal: [f64; 3],
}

/// A frame's position and turn, as MuJoCo reads `geom_xpos` and the
/// row-major `geom_xmat`.
#[derive(Clone, Copy)]
pub(super) struct Frame<'a> {
    pub(super) pos: &'a Vector3<f64>,
    pub(super) mat: &'a Matrix3<f64>,
}

impl Frame<'_> {
    fn pos(&self) -> [f64; 3] {
        [self.pos.x, self.pos.y, self.pos.z]
    }

    /// The frame's z axis, MuJoCo's `{mat[2], mat[5], mat[8]}`.
    fn z(&self) -> [f64; 3] {
        let m = self.mat;
        [m[(0, 2)], m[(1, 2)], m[(2, 2)]]
    }

    /// `mji_mulMatVec3(res, mat, v)`.
    fn mul(&self, v: &[f64; 3]) -> [f64; 3] {
        let m = self.mat;
        std::array::from_fn(|r| m[(r, 0)] * v[0] + m[(r, 1)] * v[1] + m[(r, 2)] * v[2])
    }

    /// `mji_mulMatTVec3(res, mat, v)`.
    fn mul_t(&self, v: &[f64; 3]) -> [f64; 3] {
        let m = self.mat;
        std::array::from_fn(|c| m[(0, c)] * v[0] + m[(1, c)] * v[1] + m[(2, c)] * v[2])
    }
}

/// `mju_dot3`.
fn dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `mji_sub3`.
fn sub3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `mji_add3`.
fn add3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// `mji_scl3`.
fn scl3(v: &[f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

/// `mju_normalize3`: `v` made unit, or (1, 0, 0) where its norm is below
/// mjMINVAL; its norm before.
fn normalize3(v: &mut [f64; 3]) -> f64 {
    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if norm < MJ_MINVAL {
        *v = [1.0, 0.0, 0.0];
    } else {
        let inv = 1.0 / norm;
        for x in v.iter_mut() {
            *x *= inv;
        }
    }
    norm
}

/// `mju_clip`.
fn clip(x: f64, min: f64, max: f64) -> f64 {
    if x < min {
        min
    } else if x > max {
        max
    } else {
        x
    }
}

/// `mjraw_PlaneSphere` (`engine_collision_primitive.c:28-50`): the plane's
/// normal is its frame's z axis; listed where the centre is within `margin`
/// plus the radius above it, the contact halfway between the sphere's
/// surface and the plane.
pub(super) fn raw_plane_sphere(
    margin: f64,
    plane: Frame<'_>,
    center: &[f64; 3],
    radius: f64,
) -> Option<RawContact> {
    let normal = plane.z();
    let cdist = dot3(&sub3(center, &plane.pos()), &normal);
    if cdist > margin + radius {
        return None;
    }
    let dist = cdist - radius;
    let pos = add3(center, &scl3(&normal, -dist / 2.0 - radius));
    Some(RawContact { dist, pos, normal })
}

/// `mjraw_SphereSphere` (`engine_collision_primitive.c:245-276`): listed
/// where the centres are within the radii and `margin` (inclusive). The
/// normal runs from the first centre to the second; with the centres
/// within mjMINVAL, it is the first frame's z axis crossed with the
/// second's, (1, 0, 0) where they are parallel. The contact is midway
/// between the surfaces.
pub(super) fn raw_sphere_sphere(
    margin: f64,
    sphere1: Frame<'_>,
    radius1: f64,
    center2: &[f64; 3],
    frame2: Frame<'_>,
    radius2: f64,
) -> Option<RawContact> {
    let pos1 = sphere1.pos();
    let dif = sub3(&pos1, center2);
    let cdist_sqr = dot3(&dif, &dif);
    let min_dist = margin + radius1 + radius2;
    if cdist_sqr > min_dist * min_dist {
        return None;
    }
    let dist = cdist_sqr.sqrt() - radius1 - radius2;
    let mut normal = sub3(center2, &pos1);
    if normalize3(&mut normal) < MJ_MINVAL {
        let (z1, z2) = (sphere1.z(), frame2.z());
        normal = [
            z1[1] * z2[2] - z1[2] * z2[1],
            z1[2] * z2[0] - z1[0] * z2[2],
            z1[0] * z2[1] - z1[1] * z2[0],
        ];
        normalize3(&mut normal);
    }
    let pos = add3(&scl3(&normal, radius1 + dist / 2.0), &pos1);
    Some(RawContact { dist, pos, normal })
}

/// `mjraw_SphereCapsule` (`engine_collision_primitive.c:288-303`): the
/// sphere against the point of the capsule's segment nearest its centre,
/// as two spheres. `size` is the capsule's radius and half-length.
pub(super) fn raw_sphere_capsule(
    margin: f64,
    sphere: Frame<'_>,
    radius: f64,
    capsule: Frame<'_>,
    size: &Vector3<f64>,
) -> Option<RawContact> {
    let len = size.y;
    let axis = capsule.z();
    let x = clip(dot3(&axis, &sub3(&sphere.pos(), &capsule.pos())), -len, len);
    let nearest = add3(&scl3(&axis, x), &capsule.pos());
    raw_sphere_sphere(margin, sphere, radius, &nearest, capsule, size.x)
}

/// `mjc_SphereCylinder` (`engine_collision_primitive.c:315-385`): with the
/// centre beside the side (strictly within the half-height), the sphere
/// against the axis point level with it as two spheres; above or below a
/// cap (strictly within the radius), against the cap as a plane, the normal
/// turned round; with the centre inside, whichever of the two is
/// shallower; else against the rim as a sphere of radius 0. `size` is the
/// cylinder's radius and half-height.
pub(super) fn sphere_cylinder(
    margin: f64,
    sphere: Frame<'_>,
    radius: f64,
    cylinder: Frame<'_>,
    size: &Vector3<f64>,
) -> Option<RawContact> {
    let (cyl_radius, height) = (size.x, size.y);
    let axis = cylinder.z();
    let vec = sub3(&sphere.pos(), &cylinder.pos());
    let x = dot3(&axis, &vec);
    let a_proj = scl3(&axis, x);
    let p_proj = sub3(&vec, &a_proj);
    let p_proj_sqr = dot3(&p_proj, &p_proj);

    let mut collide_side = x.abs() < height;
    let mut collide_cap = p_proj_sqr < cyl_radius * cyl_radius;
    if collide_side && collide_cap {
        // The centre is inside: the shallower way out.
        let dist_cap = height - x.abs();
        let dist_radius = cyl_radius - p_proj_sqr.sqrt();
        if dist_cap < dist_radius {
            collide_side = false;
        } else {
            collide_cap = false;
        }
    }

    if collide_side {
        let level = add3(&a_proj, &cylinder.pos());
        return raw_sphere_sphere(margin, sphere, radius, &level, cylinder, cyl_radius);
    }

    if collide_cap {
        // The cap as a plane: the top's frame is the cylinder's; the
        // bottom's has its x and z axes turned round.
        let m = cylinder.mat;
        let flipped = Matrix3::new(
            -m[(0, 0)],
            m[(0, 1)],
            -m[(0, 2)],
            -m[(1, 0)],
            m[(1, 1)],
            -m[(1, 2)],
            -m[(2, 0)],
            m[(2, 1)],
            -m[(2, 2)],
        );
        let (offset, mat) = if x > 0.0 {
            (height, m)
        } else {
            (-height, &flipped)
        };
        let p = cylinder.pos();
        let cap_pos = Vector3::new(
            p[0] + offset * axis[0],
            p[1] + offset * axis[1],
            p[2] + offset * axis[2],
        );
        let cap_frame = Frame { pos: &cap_pos, mat };
        // The plane's normal points out of the cap, at the sphere; the
        // sphere is the first geom, so the contact's normal is the reverse
        // (`mjGEOM_PLANE < mjGEOM_SPHERE < mjGEOM_CYLINDER`).
        return raw_plane_sphere(margin, cap_frame, &sphere.pos(), radius).map(|mut c| {
            c.normal = scl3(&c.normal, -1.0);
            c
        });
    }

    // The rim: the point of the rim nearest the centre, as a sphere of
    // radius 0 (the centre is off the axis here, so `p_proj_sqr` > 0).
    let rim = add3(
        &add3(
            &scl3(&axis, if x > 0.0 { height } else { -height }),
            &scl3(&p_proj, cyl_radius / p_proj_sqr.sqrt()),
        ),
        &cylinder.pos(),
    );
    raw_sphere_sphere(margin, sphere, radius, &rim, cylinder, 0.0)
}

/// `mjraw_SphereBox` (`engine_collision_box.c:39-92`): the sphere's centre
/// in the box's frame, clamped to the box along each axis of positive
/// half-size (`mju_clampVec`); listed where the clamped point is within the
/// radius and `margin` of the centre (inclusive). With the centre inside the
/// box (the clamped point within mjMINVAL of it), out through the nearest
/// face, the first of the nearest in the order −x, +x, −y, +y, −z, +z.
/// The contact is midway between the sphere's deepest point and the box.
pub(super) fn raw_sphere_box(
    margin: f64,
    sphere: Frame<'_>,
    radius: f64,
    box_frame: Frame<'_>,
    size: &Vector3<f64>,
) -> Option<RawContact> {
    let half = [size.x, size.y, size.z];
    let center = box_frame.mul_t(&sub3(&sphere.pos(), &box_frame.pos()));
    let mut clamped = center;
    for i in 0..3 {
        if half[i] > 0.0 {
            if clamped[i] < -half[i] {
                clamped[i] = -half[i];
            } else if clamped[i] > half[i] {
                clamped[i] = half[i];
            }
        }
    }
    let mut deepest = center;
    let mut tmp = sub3(&clamped, &center);
    let mut dist = normalize3(&mut tmp);
    if dist - radius > margin {
        return None;
    }

    let (pos, normal) = if dist <= MJ_MINVAL {
        let mut closest = (half[0] + half[1] + half[2]) * 2.0;
        let mut k = 0;
        for i in 0..6 {
            let face = if i % 2 == 1 {
                half[i / 2]
            } else {
                -half[i / 2]
            };
            let gap = (face - center[i / 2]).abs();
            if closest > gap {
                closest = gap;
                k = i;
            }
        }
        let mut nearest = [0.0; 3];
        nearest[k / 2] = if k % 2 == 1 { -1.0 } else { 1.0 };
        let shift = (radius - closest) / 2.0;
        let pos: [f64; 3] = std::array::from_fn(|i| center[i] + nearest[i] * shift);
        dist = -closest;
        (pos, box_frame.mul(&nearest))
    } else {
        for i in 0..3 {
            deepest[i] += tmp[i] * radius;
        }
        let pos: [f64; 3] = std::array::from_fn(|i| 0.0 + clamped[i] * 0.5 + deepest[i] * 0.5);
        (pos, box_frame.mul(&tmp))
    };

    let pos = add3(&box_frame.mul(&pos), &box_frame.pos());
    Some(RawContact {
        dist: dist - radius,
        pos,
        normal,
    })
}

/// `mjc_SphereSphere` (`engine_collision_primitive.c:280-285`).
pub(super) fn sphere_sphere(
    margin: f64,
    sphere1: Frame<'_>,
    radius1: f64,
    sphere2: Frame<'_>,
    radius2: f64,
) -> Option<RawContact> {
    raw_sphere_sphere(margin, sphere1, radius1, &sphere2.pos(), sphere2, radius2)
}

impl RawContact {
    /// The contact between `geom1` and `geom2`, MuJoCo's first geom and
    /// second, at `margin`.
    pub(super) fn contact(
        &self,
        model: &crate::types::Model,
        geom1: usize,
        geom2: usize,
        margin: f64,
    ) -> crate::types::Contact {
        super::narrow::make_contact_from_geoms(
            model,
            Vector3::from(self.pos),
            Vector3::from(self.normal),
            -self.dist,
            geom1,
            geom2,
            margin,
        )
    }
}
