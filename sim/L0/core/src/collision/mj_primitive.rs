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
    pub(super) fn pos(&self) -> [f64; 3] {
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

/// `mjraw_SphereSphere` (`engine_collision_primitive.c:245-276`), spheres
/// at `center1` and `center2` whose frames are turned as `frame1` and
/// `frame2` (a capsule's or a cylinder's point is not its frame's origin):
/// listed where the centres are within the radii and `margin` (inclusive).
/// The normal runs from the first centre to the second; with the centres
/// within mjMINVAL, it is the first frame's z axis crossed with the
/// second's, (1, 0, 0) where they are parallel. The contact is midway
/// between the surfaces.
pub(super) fn raw_sphere_sphere(
    margin: f64,
    center1: &[f64; 3],
    frame1: Frame<'_>,
    radius1: f64,
    center2: &[f64; 3],
    frame2: Frame<'_>,
    radius2: f64,
) -> Option<RawContact> {
    let pos1 = *center1;
    let dif = sub3(&pos1, center2);
    let cdist_sqr = dot3(&dif, &dif);
    let min_dist = margin + radius1 + radius2;
    if cdist_sqr > min_dist * min_dist {
        return None;
    }
    let dist = cdist_sqr.sqrt() - radius1 - radius2;
    let mut normal = sub3(center2, &pos1);
    if normalize3(&mut normal) < MJ_MINVAL {
        let (z1, z2) = (frame1.z(), frame2.z());
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
    raw_sphere_sphere(
        margin,
        &sphere.pos(),
        sphere,
        radius,
        &nearest,
        capsule,
        size.x,
    )
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
        return raw_sphere_sphere(
            margin,
            &sphere.pos(),
            sphere,
            radius,
            &level,
            cylinder,
            cyl_radius,
        );
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
    raw_sphere_sphere(margin, &sphere.pos(), sphere, radius, &rim, cylinder, 0.0)
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
    center: &[f64; 3],
    radius: f64,
    box_frame: Frame<'_>,
    size: &Vector3<f64>,
) -> Option<RawContact> {
    let half = [size.x, size.y, size.z];
    let center = box_frame.mul_t(&sub3(center, &box_frame.pos()));
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
    raw_sphere_sphere(
        margin,
        &sphere1.pos(),
        sphere1,
        radius1,
        &sphere2.pos(),
        sphere2,
        radius2,
    )
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

/// `mjraw_CapsuleCapsule` (`engine_collision_primitive.c:389-481`): the
/// nearest points of the two segments as two spheres where the axes are
/// not parallel (`|det|` at least mjMINVAL); else each segment's ends
/// against the other segment, the first segment's ends first, up to two
/// contacts, which may be the same point twice. `size` is a capsule's
/// radius and half-length.
pub(super) fn raw_capsule_capsule(
    margin: f64,
    capsule1: Frame<'_>,
    size1: &Vector3<f64>,
    capsule2: Frame<'_>,
    size2: &Vector3<f64>,
) -> Vec<RawContact> {
    let (pos1, pos2) = (capsule1.pos(), capsule2.pos());
    let axis1 = scl3(&capsule1.z(), size1.y);
    let axis2 = scl3(&capsule2.z(), size2.y);
    let dif = sub3(&pos1, &pos2);
    let ma = dot3(&axis1, &axis1);
    let mb = -dot3(&axis1, &axis2);
    let mc = dot3(&axis2, &axis2);
    let u = -dot3(&axis1, &dif);
    let v = dot3(&axis2, &dif);
    #[allow(clippy::suspicious_operation_groupings)] // MuJoCo's determinant, ma·mc − mb²
    let det = ma * mc - mb * mb;
    let collide = |point1: &[f64; 3], point2: &[f64; 3]| {
        raw_sphere_sphere(margin, point1, capsule1, size1.x, point2, capsule2, size2.x)
    };

    // The segments' points at x1 and x2, along the scaled axes.
    let point1 = |x1: f64| add3(&scl3(&axis1, x1), &pos1);
    let point2 = |x2: f64| add3(&scl3(&axis2, x2), &pos2);

    if det.abs() >= MJ_MINVAL {
        let mut x1 = (mc * u - mb * v) / det;
        let mut x2 = (ma * v - mb * u) / det;
        if x1 > 1.0 {
            x1 = 1.0;
            x2 = (v - mb) / mc;
        } else if x1 < -1.0 {
            x1 = -1.0;
            x2 = (v + mb) / mc;
        }
        if x2 > 1.0 {
            x2 = 1.0;
            x1 = clip((u - mb) / ma, -1.0, 1.0);
        } else if x2 < -1.0 {
            x2 = -1.0;
            x1 = clip((u + mb) / ma, -1.0, 1.0);
        }
        return collide(&point1(x1), &point2(x2)).into_iter().collect();
    }

    // Parallel axes: the first segment's ends, then the second's while
    // fewer than two contacts are found.
    let mut contacts: Vec<RawContact> = Vec::new();
    let end1 = add3(&pos1, &axis1);
    contacts.extend(collide(&end1, &point2(clip((v - mb) / mc, -1.0, 1.0))));
    let end1 = sub3(&pos1, &axis1);
    contacts.extend(collide(&end1, &point2(clip((v + mb) / mc, -1.0, 1.0))));
    if contacts.len() >= 2 {
        return contacts;
    }
    let end2 = add3(&pos2, &axis2);
    contacts.extend(collide(&point1(clip((u - mb) / ma, -1.0, 1.0)), &end2));
    if contacts.len() >= 2 {
        return contacts;
    }
    let end2 = sub3(&pos2, &axis2);
    contacts.extend(collide(&point1(clip((u + mb) / ma, -1.0, 1.0)), &end2));
    contacts
}

/// `mjraw_CapsuleBox` (`engine_collision_box.c:119-591`, without its
/// `j == 2` block, `:312-393`, whose values nothing reads): in the box's
/// frame, the capsule's segment point nearest the box, from its ends
/// against the faces and the segment against the twelve edges; then a
/// second point along the segment where the segment lies along a face or
/// an edge, as far as the box reaches; each point a sphere of the
/// capsule's radius against the box (`mjraw_SphereBox`). `size1` is the
/// capsule's radius and half-length, `size2` the box's half-sizes.
#[allow(clippy::too_many_lines)] // MuJoCo's function, statement for statement
pub(super) fn raw_capsule_box(
    margin: f64,
    capsule: Frame<'_>,
    size1: &Vector3<f64>,
    box_frame: Frame<'_>,
    size2: &Vector3<f64>,
) -> Vec<RawContact> {
    let half = [size2.x, size2.y, size2.z];
    let halflength = size1.y;
    // The second point's position along the segment from the first; below
    // -3, none.
    let mut secondpos = -4.0;

    let pos = box_frame.mul_t(&sub3(&capsule.pos(), &box_frame.pos()));
    let axis = box_frame.mul_t(&capsule.z());
    let halfaxis = scl3(&axis, halflength);
    let mut axisdir = 0;
    for (k, h) in halfaxis.iter().enumerate() {
        if *h > 0.0 {
            axisdir += 1 << k;
        }
    }

    let bestdistmax = margin + 2.0 * (size1.x + halflength + half[0] + half[1] + half[2]);
    let mut bestdist = bestdistmax;
    let mut bestsegmentpos = 0.0;
    // -3 or -1: a capsule end (−1 or +1) nearest a face; 0..8: an edge, by
    // the segment's and the edge's clamps (`s1 * 3 + s2`).
    let mut cltype: i32 = -4;
    let mut clface: Option<usize> = None;
    let mut clcorner: i32 = 0;
    let mut cledge: usize = 0;
    let mut bestboxpos = 0.0;

    // Each end of the segment against the faces, where at most one of its
    // coordinates is past the box.
    for (i, end) in [(-1_i32, -1.0), (1, 1.0)] {
        let mut tmp1: [f64; 3] = std::array::from_fn(|k| pos[k] + halfaxis[k] * end);
        let tmp2 = tmp1;
        let (mut clamped, mut face) = (0, None);
        for j in 0..3 {
            if tmp1[j] < -half[j] {
                clamped += 1;
                face = Some(j);
                tmp1[j] = -half[j];
            } else if tmp1[j] > half[j] {
                clamped += 1;
                face = Some(j);
                tmp1[j] = half[j];
            }
        }
        if clamped > 1 {
            continue;
        }
        let gap = sub3(&tmp1, &tmp2);
        let dist = dot3(&gap, &gap);
        if dist < bestdist {
            bestdist = dist;
            bestsegmentpos = end;
            cltype = -2 + i;
            clface = face;
        }
    }

    // The segment against each edge along axis j, from corner i.
    for j in 0..3 {
        for i in 0..8_i32 {
            if i & (1 << j) != 0 {
                continue;
            }
            let mut tmp3: [f64; 3] =
                std::array::from_fn(|k| if i & (1 << k) != 0 { half[k] } else { -half[k] });
            tmp3[j] = 0.0;
            let dif = sub3(&tmp3, &pos);
            let ma = half[j] * half[j];
            let mb = -half[j] * halfaxis[j];
            let mc = size1.y * size1.y;
            let u = -half[j] * dif[j];
            let v = dot3(&halfaxis, &dif);
            #[allow(clippy::suspicious_operation_groupings)] // MuJoCo's determinant
            let det = ma * mc - mb * mb;
            if det.abs() < MJ_MINVAL {
                continue;
            }
            let idet = 1.0 / det;
            let mut x1 = (mc * u - mb * v) * idet;
            let mut x2 = (ma * v - mb * u) * idet;
            // 1: the middle of the edge (s1) or the segment (s2); 0 or 2 an end.
            let (mut s1, mut s2) = (1, 1);
            if x1 > 1.0 {
                x1 = 1.0;
                s1 = 2;
                x2 = (v - mb) * (1.0 / mc);
            } else if x1 < -1.0 {
                x1 = -1.0;
                s1 = 0;
                x2 = (v + mb) * (1.0 / mc);
            }
            if x2 > 1.0 {
                x2 = 1.0;
                s2 = 2;
                x1 = (u - mb) * (1.0 / ma);
                if x1 > 1.0 {
                    x1 = 1.0;
                    s1 = 2;
                } else if x1 < -1.0 {
                    x1 = -1.0;
                    s1 = 0;
                }
            } else if x2 < -1.0 {
                x2 = -1.0;
                s2 = 0;
                x1 = (u + mb) * (1.0 / ma);
                if x1 > 1.0 {
                    x1 = 1.0;
                    s1 = 2;
                } else if x1 < -1.0 {
                    x1 = -1.0;
                    s1 = 0;
                }
            }
            let mut dif = sub3(&tmp3, &pos);
            for k in 0..3 {
                dif[k] += halfaxis[k] * -x2;
            }
            dif[j] += half[j] * x1;
            let dist = dot3(&dif, &dif);
            let clamps = s1 * 3 + s2;
            // MuJoCo's −mjMINVAL, for an axis numerically parallel to the box.
            if dist < bestdist - MJ_MINVAL {
                bestdist = dist;
                bestsegmentpos = x2;
                bestboxpos = x1;
                // Below 6: the edge's lower end or its middle is nearest.
                clcorner = i + (1 << j) * (clamps / 6);
                cledge = j;
                cltype = clamps;
            }
        }
    }

    if cltype == -4 {
        return Vec::new();
    }

    let reach = |ax: usize| 2.0 * half[ax] / halfaxis[ax].abs();
    if cltype >= 0 && cltype / 3 != 1 {
        // A corner of the box is nearest: with the segment along a face or
        // an edge from it, a second point.
        let mut c1 = axisdir ^ clcorner;
        if c1 != 0 && c1 != 7 {
            let (mul, de, dp) = if c1 == 3 || c1 == 5 || c1 == 6 {
                c1 = 7 - c1;
                (-1.0, 1.0 + bestsegmentpos, 1.0 - bestsegmentpos)
            } else {
                (1.0, 1.0 - bestsegmentpos, 1.0 + bestsegmentpos)
            };
            let corner_axes = match c1 {
                1 => Some((0, 1, 2)),
                2 => Some((1, 2, 0)),
                4 => Some((2, 0, 1)),
                _ => None,
            };
            if let Some((ax, ax1, ax2)) = corner_axes {
                if axis[ax] * axis[ax] > 0.5 {
                    // Along the edge.
                    secondpos = de;
                    let e1 = reach(ax);
                    if e1 < secondpos {
                        secondpos = e1;
                    }
                    secondpos *= mul;
                } else {
                    // Along a face.
                    secondpos = dp;
                    for e1 in [reach(ax1), reach(ax2)] {
                        if e1 < secondpos {
                            secondpos = e1;
                        }
                    }
                    secondpos *= -mul;
                }
            }
        }
    } else if cltype >= 0 {
        // The middle of an edge is nearest: with the segment crossing it
        // (not in a T), a second point.
        let mut c1 = axisdir ^ clcorner;
        c1 &= 7 - (1 << cledge);
        if c1 == 1 || c1 == 2 || c1 == 4 {
            let ax = cledge;
            let (mut ax1, mut ax2) = match cledge {
                0 => (1, 2),
                1 => (2, 0),
                _ => (0, 1),
            };
            if axis[ax1].abs() > axis[ax2].abs() {
                ax1 = ax2;
            }
            ax2 = 3 - ax - ax1;
            let mul = if c1 & (1 << ax2) != 0 {
                secondpos = 1.0 - bestsegmentpos;
                1.0
            } else {
                secondpos = 1.0 + bestsegmentpos;
                -1.0
            };
            let e1 = reach(ax2);
            if e1 < secondpos {
                secondpos = e1;
            }
            let e2 = if ((axisdir & (1 << ax)) != 0) == ((c1 & (1 << ax2)) != 0) {
                1.0 - bestboxpos
            } else {
                1.0 + bestboxpos
            };
            let e1 = half[ax] * e2 / halfaxis[ax].abs();
            if e1 < secondpos {
                secondpos = e1;
            }
            secondpos *= mul;
        }
    } else if let Some(face) = clface {
        // A capsule end is nearest a face (not inside the box): the other
        // end's way, as far as the segment stays over the box.
        let mul = if cltype == -3 { 1.0 } else { -1.0 };
        secondpos = 2.0;
        let tmp1: [f64; 3] = std::array::from_fn(|k| pos[k] + halfaxis[k] * -mul);
        for i in (0..3).filter(|&i| i != face) {
            for bound in [half[i], -half[i]] {
                let e1 = (bound - tmp1[i]) / halfaxis[i] * mul;
                if e1 > 0.0 && e1 < secondpos {
                    secondpos = e1;
                }
            }
        }
        secondpos *= mul;
    }

    // Each point, back in the world, as a sphere against the box.
    let sphere_at = |along: f64| {
        let local: [f64; 3] = std::array::from_fn(|k| pos[k] + halfaxis[k] * along);
        add3(&box_frame.mul(&local), &box_frame.pos())
    };
    let mut contacts: Vec<RawContact> = raw_sphere_box(
        margin,
        &sphere_at(bestsegmentpos),
        size1.x,
        box_frame,
        size2,
    )
    .into_iter()
    .collect();
    if secondpos > -3.0 {
        contacts.extend(raw_sphere_box(
            margin,
            &sphere_at(secondpos + bestsegmentpos),
            size1.x,
            box_frame,
            size2,
        ));
    }
    contacts
}
