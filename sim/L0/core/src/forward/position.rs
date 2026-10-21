//! Forward kinematics and position-stage computations.
//!
//! Computes body poses, site poses, geom poses, subtree centers of mass,
//! and AABBs from joint positions. Corresponds to the position-stage of
//! MuJoCo's `engine_core_smooth.c`.

use crate::collision::narrow::GEOM_EPSILON;
use crate::constraint::impedance::MJ_MINVAL;
use crate::dynamics::{compute_body_spatial_inertia, mju_dof_com};
use crate::quat;
use crate::quat::mul_mat_vec3;
use crate::tendon::mj_fwd_tendon;
use crate::types::{Data, ENABLE_SLEEP, MjJointType, Model, SameFrame, SleepState};
use nalgebra::{DVector, Matrix3, Matrix6, Quaternion, UnitQuaternion, Vector3};

/// Forward kinematics: compute body poses from qpos.
///
/// This traverses the kinematic tree from root to leaves, computing
/// the world-frame position and orientation of each body; then wakes the
/// trees the user changed (`island::mj_wake`), then computes the geoms',
/// sites' and subtree centres of mass, and the tendons.
pub fn mj_fwd_position(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // Body 0 (world) is always at origin
    data.xpos[0] = Vector3::zeros();
    data.xquat[0] = UnitQuaternion::identity();
    data.xmat[0] = Matrix3::identity();

    // Process bodies in order (assumes topological sort: parent before child),
    // sleeping ones included.
    for body_id in 1..model.nbody {
        let (pos, xquat) = body_pose(model, data, body_id);
        let pos = Vector3::from(pos);
        let quat =
            UnitQuaternion::new_unchecked(Quaternion::new(xquat[0], xquat[1], xquat[2], xquat[3]));

        // §16.15: qpos change detection — if this body was sleeping and its
        // computed pose differs from the stored pose (exact bitwise match),
        // mark the tree for waking. This detects external qpos modifications.
        if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
            let tree = model.body_treeid[body_id];
            if tree < data.tree_qpos_dirty.len() {
                // Compare new vs stored (exact floating-point equality)
                let pos_changed = pos != data.xpos[body_id];
                let quat_changed =
                    quat.into_inner().coords != data.xquat[body_id].into_inner().coords;
                if pos_changed || quat_changed {
                    data.tree_qpos_dirty[tree] = true;
                }
            }
        }

        // Store computed pose (shared path for both mocap and regular bodies)
        data.xpos[body_id] = pos;
        data.xquat[body_id] = quat;
        data.xmat[body_id] = mat3(&quat::quat_to_mat(&xquat));

        // Compute inertial frame position (shared)
        let (xipos, _, ximat) = local_to_global(
            data,
            body_id,
            &model.body_ipos[body_id],
            &model.body_iquat[body_id],
            sameframe(&model.body_sameframe, body_id),
        );
        data.xipos[body_id] = xipos;
        data.ximat[body_id] = ximat;

        // Compute body spatial inertia in world frame (cinert, shared).
        // MuJoCo does NOT zero mass for mocap bodies, and neither do we.
        let h = data.xipos[body_id] - pos; // COM offset from body origin in world frame
        data.cinert[body_id] = compute_body_spatial_inertia(
            model.body_mass[body_id],
            model.body_inertia[body_id],
            &data.ximat[body_id],
            h,
        );
    }

    // World body has zero inertia
    data.cinert[0] = Matrix6::zeros();

    // Wake what the user changed (§16.4) after the bodies' poses, which find
    // a sleeping pose changed, and before the geoms', sites' and tendons', as
    // MuJoCo's `mj_kinematics` runs `mj_wake` between `mj_kinematics1` and
    // `mj_kinematics2` (`engine_core_smooth.c:236-241`), so a tendon on a
    // tree this pass wakes is computed; with sleep disabled it wakes every
    // sleeping tree.
    if crate::island::mj_wake(model, data) {
        crate::island::mj_update_sleep_arrays(model, data);
    }

    // Update geom poses
    for geom_id in 0..model.ngeom {
        let (xpos, _, xmat) = local_to_global(
            data,
            model.geom_body[geom_id],
            &model.geom_pos[geom_id],
            &model.geom_quat[geom_id],
            sameframe(&model.geom_sameframe, geom_id),
        );
        data.geom_xpos[geom_id] = xpos;
        data.geom_xmat[geom_id] = xmat;
    }

    // Update site poses
    for site_id in 0..model.nsite {
        let (xpos, xquat, xmat) = local_to_global(
            data,
            model.site_body[site_id],
            &model.site_pos[site_id],
            &model.site_quat[site_id],
            sameframe(&model.site_sameframe, site_id),
        );
        data.site_xpos[site_id] = xpos;
        data.site_xmat[site_id] = xmat;
        data.site_xquat[site_id] =
            UnitQuaternion::new_unchecked(Quaternion::new(xquat[0], xquat[1], xquat[2], xquat[3]));
    }

    // Before the tendons: MuJoCo's `mj_comPos` precedes `mj_tendon`
    // (`engine_forward.c:140-145`), and a spatial tendon's Jacobian reads
    // `subtree_com` and `cdof`.
    mj_com_pos(model, data);

    // Tendon kinematics (after site poses and the subtree COM)
    mj_fwd_tendon(model, data);
}

/// The subtree masses and centres of mass, then each dof's motion axis
/// (`cdof`) in the world's orientation about its tree's subtree centre of
/// mass, `subtree_com[body_rootid[b]]`: MuJoCo's `mj_comPos`
/// (`engine_core_smooth.c:245-350`). The axes are the awake bodies', as
/// MuJoCo's: a sleeping body keeps those of its last awake pass (for a tree
/// that starts asleep, the reset's forward pass, `Model::start_sleep`).
fn mj_com_pos(model: &Model, data: &mut Data) {
    // Initialize with each body's own mass and COM
    for body_id in 0..model.nbody {
        data.subtree_mass[body_id] = model.body_mass[body_id];
        data.subtree_com[body_id] = model.body_mass[body_id] * data.xipos[body_id];
    }

    // Backward pass: accumulate children's contributions to parents
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        // Copy child data first to satisfy borrow checker
        let child_mass = data.subtree_mass[body_id];
        let child_weighted_com = data.subtree_com[body_id];
        data.subtree_mass[parent_id] += child_mass;
        data.subtree_com[parent_id] += child_weighted_com;
    }

    // Convert weighted sum to actual COM as `mj_comPos` normalizes it
    // (`engine_core_smooth.c:274-284`): a subtree with less than mjMINVAL of
    // mass takes the body's own centre of mass; another is scaled by the
    // reciprocal of its mass.
    for body_id in 0..model.nbody {
        if data.subtree_mass[body_id] < MJ_MINVAL {
            data.subtree_com[body_id] = data.xipos[body_id];
        } else {
            data.subtree_com[body_id] *= 1.0 / data.subtree_mass[body_id];
        }
    }

    // Each dof's motion axis (`mju_dofCom`, `:298-350`): a hinge's axis, a
    // ball's or a free joint's rotation the columns of the body's `xmat`,
    // each about the joint's anchor; a slide's axis, a free joint's
    // translation the world's axes.
    let sleep_filter = model.enableflags & ENABLE_SLEEP != 0 && data.nbody_awake < model.nbody;
    let nbody = if sleep_filter {
        data.nbody_awake
    } else {
        model.nbody
    };
    for b in 1..nbody {
        let body = if sleep_filter {
            data.body_awake_ind[b]
        } else {
            b
        };
        let joints = model.body_jnt_adr[body]..model.body_jnt_adr[body] + model.body_jnt_num[body];
        for jnt in joints {
            let dof = model.jnt_dof_adr[jnt];
            let offset = data.subtree_com[model.body_rootid[body]] - data.xanchor[jnt];
            let xmat = data.xmat[body];
            let column = |k: usize| xmat.column(k).into_owned();
            match model.jnt_type[jnt] {
                MjJointType::Free => {
                    for k in 0..3 {
                        data.cdof[dof + k] = mju_dof_com(&Vector3::ith(k, 1.0), None);
                        data.cdof[dof + 3 + k] = mju_dof_com(&column(k), Some(&offset));
                    }
                }
                MjJointType::Ball => {
                    for k in 0..3 {
                        data.cdof[dof + k] = mju_dof_com(&column(k), Some(&offset));
                    }
                }
                MjJointType::Slide => data.cdof[dof] = mju_dof_com(&data.xaxis[jnt], None),
                MjJointType::Hinge => {
                    data.cdof[dof] = mju_dof_com(&data.xaxis[jnt], Some(&offset));
                }
            }
        }
    }
}

/// The world pose of body `body_id` (`[x, y, z]`, `[w, x, y, z]`) from its
/// parent's, its offset (or its mocap pose) and its joints, in MuJoCo 3.5.0's
/// arithmetic (`mj_kinematics1`, `engine_core_smooth.c:40-187`). Writes its
/// joints' `xanchor` and `xaxis`: each joint's anchor and axis in the frame
/// before it turns, as MuJoCo's.
///
/// A hinge or a slide moves by `qpos − qpos0`; a hinge or a ball turns about
/// its anchor, so a `jnt_pos` off the body origin moves the origin. A free
/// joint, alone on its body, sets the pose from `qpos`.
fn body_pose(model: &Model, data: &mut Data, body_id: usize) -> ([f64; 3], [f64; 4]) {
    let jnt_adr = model.body_jnt_adr[body_id];
    let jnt_num = model.body_jnt_num[body_id];
    let (xpos, mut xquat) = if jnt_num == 1 && model.jnt_type[jnt_adr] == MjJointType::Free {
        let pose = free_pose(&data.qpos, model.jnt_qpos_adr[jnt_adr]);
        data.xanchor[jnt_adr] = Vector3::from(pose.0);
        data.xaxis[jnt_adr] = model.jnt_axis[jnt_adr];
        pose
    } else {
        let (body_pos, body_quat) = match model.body_mocapid.get(body_id) {
            Some(&Some(mocap)) => {
                let mut quat = quat_array(&data.mocap_quat[mocap]);
                quat::normalize4(&mut quat);
                (data.mocap_pos[mocap].into(), quat)
            }
            _ => (
                model.body_pos[body_id].into(),
                quat_array(&model.body_quat[body_id]),
            ),
        };
        let parent = model.body_parent[body_id];
        let (mut xpos, mut xquat) = if parent == 0 {
            (body_pos, body_quat)
        } else {
            (
                add3(
                    &mul_mat_vec3(&data.xmat[parent], &body_pos),
                    &data.xpos[parent].into(),
                ),
                quat::mul_quat(&quat_array(&data.xquat[parent]), &body_quat),
            )
        };
        for jnt_id in jnt_adr..jnt_adr + jnt_num {
            let qadr = model.jnt_qpos_adr[jnt_id];
            let jnt_axis: [f64; 3] = model.jnt_axis[jnt_id].into();
            let jnt_pos: [f64; 3] = model.jnt_pos[jnt_id].into();
            let xaxis = quat::rot_vec_quat(&jnt_axis, &xquat);
            let xanchor = add3(&quat::rot_vec_quat(&jnt_pos, &xquat), &xpos);
            match model.jnt_type[jnt_id] {
                MjJointType::Slide => {
                    let displacement = data.qpos[qadr] - model.qpos0[qadr];
                    for i in 0..3 {
                        xpos[i] += xaxis[i] * displacement;
                    }
                }
                jnt_type @ (MjJointType::Hinge | MjJointType::Ball) => {
                    let local = if jnt_type == MjJointType::Ball {
                        quat::qpos_quat(&data.qpos, qadr)
                    } else {
                        quat::axis_angle_to_quat(&jnt_axis, data.qpos[qadr] - model.qpos0[qadr])
                    };
                    xquat = quat::mul_quat(&xquat, &local);
                    let offset = quat::rot_vec_quat(&jnt_pos, &xquat);
                    xpos = [
                        xanchor[0] - offset[0],
                        xanchor[1] - offset[1],
                        xanchor[2] - offset[2],
                    ];
                }
                // MuJoCo refuses a free joint that shares its body; this
                // one sets the pose from `qpos` as a lone one does.
                MjJointType::Free => (xpos, xquat) = free_pose(&data.qpos, qadr),
            }
            data.xanchor[jnt_id] = Vector3::from(xanchor);
            data.xaxis[jnt_id] = Vector3::from(xaxis);
        }
        (xpos, xquat)
    };
    quat::normalize4(&mut xquat);
    (xpos, xquat)
}

/// A free joint's pose from `qpos[qadr..qadr + 7]`, its quaternion normalized.
fn free_pose(qpos: &DVector<f64>, qadr: usize) -> ([f64; 3], [f64; 4]) {
    (
        [qpos[qadr], qpos[qadr + 1], qpos[qadr + 2]],
        quat::qpos_quat(qpos, qadr + 3),
    )
}

/// The world position, orientation and rotation matrix of a frame at `pos`,
/// `quat` in body `body_id`'s frame (MuJoCo `mj_local2Global`,
/// `engine_core_util.c:844-885`): the position and the matrix are copied
/// from the body's frame or its inertial frame where `same` says the frame
/// is that one, as MuJoCo does. The quaternion, which MuJoCo does not store,
/// is computed whatever `same` says, as MuJoCo's `framequat` sensor computes
/// a site's.
fn local_to_global(
    data: &Data,
    body_id: usize,
    pos: &Vector3<f64>,
    quat: &UnitQuaternion<f64>,
    same: SameFrame,
) -> (Vector3<f64>, [f64; 4], Matrix3<f64>) {
    let xpos = match same {
        SameFrame::Body => data.xpos[body_id],
        SameFrame::Inertia => data.xipos[body_id],
        SameFrame::None | SameFrame::BodyRot | SameFrame::InertiaRot => Vector3::from(add3(
            &mul_mat_vec3(&data.xmat[body_id], &(*pos).into()),
            &data.xpos[body_id].into(),
        )),
    };
    let xquat = quat::mul_quat(&quat_array(&data.xquat[body_id]), &quat_array(quat));
    let xmat = match same {
        SameFrame::Body | SameFrame::BodyRot => data.xmat[body_id],
        SameFrame::Inertia | SameFrame::InertiaRot => data.ximat[body_id],
        SameFrame::None => mat3(&quat::quat_to_mat(&xquat)),
    };
    (xpos, xquat, xmat)
}

/// Entry `i` of a `*_sameframe` array; [`SameFrame::None`], which computes
/// the frame, for a model built without [`Model::compute_simple_bodies`].
fn sameframe(flags: &[SameFrame], i: usize) -> SameFrame {
    flags.get(i).copied().unwrap_or_default()
}

/// A row-major 3×3 matrix.
fn mat3(m: &[f64; 9]) -> Matrix3<f64> {
    Matrix3::new(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8])
}

/// `[w, x, y, z]` of a unit quaternion.
fn quat_array(q: &UnitQuaternion<f64>) -> [f64; 4] {
    [q.w, q.i, q.j, q.k]
}

fn add3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Find closest point on line segment AB to point P.
#[inline]
pub fn closest_point_segment(a: Vector3<f64>, b: Vector3<f64>, p: Vector3<f64>) -> Vector3<f64> {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = ab.dot(&ab);

    if ab_len_sq < GEOM_EPSILON {
        return a; // Degenerate segment
    }

    let t = (ap.dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
    a + ab * t
}
