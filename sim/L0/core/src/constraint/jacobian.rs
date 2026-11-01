//! The contact constraint's Jacobian rows, as MuJoCo 3.5.0's
//! `mj_contactJacobian` and `mj_instantiateContact` build them
//! (`engine_core_constraint.c:907-1087`).

use nalgebra::{DMatrix, Vector3};

use crate::constraint::chain::{body_pair_chain, union_chains};
use crate::constraint::impedance::MJ_MINVAL;
use crate::jacobian::mj_jac;
use crate::types::{Contact, Data, Model};

/// Whether MuJoCo takes its special case for the contact, one body a side
/// (a geom's, or a flex vertex's own; `engine_core_constraint.c:911-930`):
/// every side but a flex element's. sim-core has no interpolated flex.
fn one_body_a_side(contact: &Contact) -> bool {
    contact.elem.iter().all(Option::is_none)
}

/// The bodies of a contact's side, in MuJoCo's order (`:936-978`): a geom's
/// body, a flex vertex's own body, or an element's vertices' bodies, the
/// other side's vertex left out where the element has it
/// (`mj_elemBodyWeight`, `:142-182`). With their vertices, global, for an
/// element; `None` for a geom or a vertex.
fn side_bodies(model: &Model, contact: &Contact, side: usize) -> Vec<(usize, Option<usize>)> {
    if let Some(geom) = contact.geom[side] {
        return vec![(model.geom_body[geom], None)];
    }
    let Some(f) = contact.flex[side] else {
        return Vec::new();
    };
    let vertadr = model.flex_vertadr[f];
    if let Some(v) = contact.vert[side] {
        return vec![(model.flexvert_bodyid[vertadr + v], None)];
    }
    let Some(e) = contact.elem[side] else {
        return Vec::new();
    };
    let global = model.flex_elemadr[f] + e;
    let adr = model.flexelem_dataadr[global];
    let skip = contact.vert[1 - side].map(|v| vertadr + v);
    model.flexelem_data[adr..adr + model.flexelem_datanum[global]]
        .iter()
        .filter(|&&v| Some(v) != skip)
        .map(|&v| (model.flexvert_bodyid[v], Some(v)))
        .collect()
}

/// A contact side's bodies with MuJoCo's weights (`:936-978`): a geom's
/// body or a flex vertex's own body 1; an element's vertices' bodies each
/// the inverse of its distance to the contact point (at least `mjMINVAL`),
/// the weights scaled to sum to 1 (`mj_elemBodyWeight`, `:142-182`; MuJoCo
/// stops below an `mjMINVAL` sum, which a contact within 10^15 of its
/// element never reaches).
pub fn side_weights(
    model: &Model,
    data: &Data,
    contact: &Contact,
    side: usize,
) -> Vec<(usize, f64)> {
    let bodies = side_bodies(model, contact, side);
    if bodies.iter().all(|(_, v)| v.is_none()) {
        return bodies.into_iter().map(|(b, _)| (b, 1.0)).collect();
    }
    let p = contact.pos;
    let mut weighted: Vec<(usize, f64)> = bodies
        .iter()
        .map(|&(b, v)| {
            let x = v.map_or(p, |v| data.flexvert_xpos[v]);
            // `mju_dist3`, then `mju_max(mjMINVAL, dist)`.
            let d = [p.x - x.x, p.y - x.y, p.z - x.z];
            let dist = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            (b, 1.0 / if MJ_MINVAL >= dist { MJ_MINVAL } else { dist })
        })
        .collect();
    // `mju_sum`, then `mju_scl` by its inverse.
    let sum = weighted.iter().fold(0.0, |s, &(_, w)| s + w);
    let inv = 1.0 / sum;
    for (_, w) in &mut weighted {
        *w *= inv;
    }
    weighted
}

/// The contact's Jacobian difference at its point, translational and
/// rotational (3×nv each), the second side's less the first's: MuJoCo's
/// special case `mj_jacDifPair` (`J₂ − J₁`, `engine_core_util.c:429-500`),
/// or its general case `mj_jacSum` (`:503-573`) over the sides' weighted
/// bodies, the first side's weights negated: the first body's Jacobian
/// times its weight, then each other's times its weight added in turn.
fn jacobian_difference(
    model: &Model,
    data: &Data,
    contact: &Contact,
) -> (DMatrix<f64>, DMatrix<f64>) {
    let p = &contact.pos;
    if one_body_a_side(contact) {
        let [Some(b1), Some(b2)] = contact.bodies(model) else {
            return (DMatrix::zeros(3, model.nv), DMatrix::zeros(3, model.nv));
        };
        let (trans1, rot1) = mj_jac(model, data, b1, p);
        let (trans2, rot2) = mj_jac(model, data, b2, p);
        return (trans2 - trans1, rot2 - rot1);
    }
    let weighted = side_weights(model, data, contact, 0)
        .into_iter()
        .map(|(b, w)| (b, -w))
        .chain(side_weights(model, data, contact, 1));
    let (mut trans, mut rot) = (DMatrix::zeros(3, model.nv), DMatrix::zeros(3, model.nv));
    for (k, (b, w)) in weighted.enumerate() {
        let (tb, rb) = mj_jac(model, data, b, p);
        if k == 0 {
            trans = tb * w;
            rot = rb * w;
        } else {
            trans += tb * w;
            rot += rb * w;
        }
    }
    (trans, rot)
}

/// A contact's dof chain at the sparse size: its two sides' bodies' merged
/// chain (`mj_jacDifPair`, `engine_core_constraint.c:913-929`), or with a
/// flex element on a side every body's chain merged (`mj_jacSum`,
/// `engine_core_util.c:516-551`).
pub fn contact_chain(model: &Model, contact: &Contact) -> Vec<usize> {
    if one_body_a_side(contact) {
        return match contact.bodies(model) {
            [Some(b1), Some(b2)] => body_pair_chain(model, b1, b2),
            _ => Vec::new(),
        };
    }
    (0..2)
        .flat_map(|side| side_bodies(model, contact, side))
        .fold(Vec::new(), |chain, (b, _)| {
            union_chains(&chain, &body_pair_chain(model, b, b))
        })
}

/// `rows` of `j` from row `row0` on: each row of `frame` times `jacdif`
/// (3×nv), as `mju_mulMatMat` multiplies them (a zero frame entry skipped).
fn frame_rows(j: &mut DMatrix<f64>, row0: usize, frame: &[Vector3<f64>], jacdif: &DMatrix<f64>) {
    for (r, f) in frame.iter().enumerate() {
        for c in 0..jacdif.ncols() {
            let mut v = 0.0;
            for k in 0..3 {
                if f[k] != 0.0 {
                    v += jacdif[(k, c)] * f[k];
                }
            }
            j[(row0 + r, c)] = v;
        }
    }
}

/// A contact's normal row (1×nv): the normal times the translational
/// Jacobian difference at the contact point, as MuJoCo's adhesion
/// transmission projects an excluded contact's (`mju_mulMatMat(jac,
/// con->frame, jacdifp, 1, 3, NV)`, `engine_core_smooth.c:1685-1694`).
#[must_use]
pub fn compute_contact_normal_row(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64> {
    let (jacdif, _) = jacobian_difference(model, data, contact);
    let mut j = DMatrix::zeros(1, model.nv);
    frame_rows(&mut j, 0, &[contact.normal], &jacdif);
    j
}

/// A contact's `dim`×nv Jacobian, in the contact frame: MuJoCo 3.5.0's.
///
/// The translational Jacobians of the contact point on the two sides'
/// bodies ([`mj_jac`]) are differenced, second minus first
/// (`mj_jacDifPair`, `engine_core_util.c:429-500`), or with a flex element
/// on a side summed over each side's bodies with their weights (`mj_jacSum`,
/// `:503-573`; an element's vertices weighted by their inverse distance to
/// the point, `engine_core_constraint.c:142-182`), and turned into the
/// contact frame (`mju_mulMatMat`): row 0 the normal, rows 1-2 the tangents.
/// With `dim` above 3 the rotational Jacobians' difference gives the
/// torsional row (the normal) and, at `dim` 6, the rolling rows (the
/// tangents). Pyramidal and elliptic rows are built from these rows by the
/// assembly.
#[must_use]
pub fn compute_contact_jacobian(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64> {
    let dim = contact.dim;
    let (trans, rot) = jacobian_difference(model, data, contact);
    let frame = [contact.normal, contact.frame[0], contact.frame[1]];
    let mut j = DMatrix::zeros(dim, model.nv);
    frame_rows(&mut j, 0, &frame[..dim.min(3)], &trans);
    if dim > 3 {
        frame_rows(&mut j, 3, &frame[..dim - 3], &rot);
    }
    j
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::similar_names)]
mod contact_jac_free_joint_tests {
    use super::*;
    use crate::constraint::impedance::{DEFAULT_SOLIMP, DEFAULT_SOLREF};
    use crate::forward::mj_fwd_position;
    use crate::jacobian::mj_jac;
    use crate::types::{MjJointType, Model};
    use approx::assert_relative_eq;
    use nalgebra::{DVector, UnitQuaternion, Vector3};
    use std::f64::consts::{FRAC_PI_4, FRAC_PI_6};

    // =========================================================================
    // Helpers
    // =========================================================================

    /// Single free-joint body with minimal geom fields for contact tests.
    ///
    /// Returns identity orientation with FK already run. Caller can write a
    /// non-identity quaternion into `data.qpos[3..7]` and re-run
    /// `mj_fwd_position` for non-trivial orientations.
    fn make_free_body_contact_model() -> (Model, Data) {
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 6];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)];
        model.body_name = vec![Some("world".into()), Some("b1".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 7;
        model.nv = 6;
        model.jnt_type = vec![MjJointType::Free];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("j1".into())];

        model.dof_body = vec![1; 6];
        model.dof_jnt = vec![0; 6];
        model.dof_parent = vec![None, Some(0), Some(1), Some(2), Some(3), Some(4)];
        model.dof_armature = vec![0.0; 6];
        model.dof_damping = vec![0.0; 6];
        model.dof_frictionloss = vec![0.0; 6];

        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        // Geom fields: compute_contact_jacobian only reads geom_body, but
        // mj_fwd_position reads geom_pos/geom_quat to compute geom world poses.
        model.ngeom = 2;
        model.geom_body = vec![0, 1]; // geom 0 → world, geom 1 → free body
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.body_geom_adr = vec![0, 1];
        model.body_geom_num = vec![1, 1];

        // Identity quaternion for the free joint
        model.qpos0 = DVector::zeros(7);
        model.qpos0[3] = 1.0; // w component
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_dof_patterns();

        let mut data = model.make_data();
        mj_fwd_position(&model, &mut data);

        (model, data)
    }

    /// Two free-joint bodies (no kinematic chain between them) with geom fields.
    ///
    /// nbody=3, njnt=2, nv=12, nq=14, ngeom=3. Both free bodies are children of
    /// the world body. Caller sets orientations in qpos and calls mj_fwd_position.
    fn make_two_free_body_contact_model() -> (Model, Data) {
        let mut model = Model::empty();

        // --- Topology: 3 bodies (world + 2 free), no kinematic chain ---
        model.nbody = 3;
        model.body_parent = vec![0, 0, 0];
        model.body_rootid = vec![0, 1, 2];
        model.body_jnt_adr = vec![0, 0, 1];
        model.body_jnt_num = vec![0, 1, 1];
        model.body_dof_adr = vec![0, 0, 6];
        model.body_dof_num = vec![0, 6, 6];
        model.body_pos = vec![Vector3::zeros(); 3];
        model.body_quat = vec![UnitQuaternion::identity(); 3];
        model.body_ipos = vec![Vector3::zeros(); 3];
        model.body_iquat = vec![UnitQuaternion::identity(); 3];
        model.body_mass = vec![0.0, 1.0, 1.0];
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.01, 0.01, 0.01),
            Vector3::new(0.01, 0.01, 0.01),
        ];
        model.body_name = vec![Some("world".into()), Some("b1".into()), Some("b2".into())];
        model.body_subtreemass = vec![2.0, 1.0, 1.0];

        // --- 2 free joints ---
        model.njnt = 2;
        model.nq = 14; // 7 + 7
        model.nv = 12; // 6 + 6
        model.jnt_type = vec![MjJointType::Free; 2];
        model.jnt_body = vec![1, 2];
        model.jnt_qpos_adr = vec![0, 7];
        model.jnt_dof_adr = vec![0, 6];
        model.jnt_axis = vec![Vector3::z(); 2];
        model.jnt_pos = vec![Vector3::zeros(); 2];
        model.jnt_limited = vec![false; 2];
        model.jnt_range = vec![(0.0, 0.0); 2];
        model.jnt_stiffness = vec![0.0; 2];
        model.jnt_springref = vec![0.0; 2];
        // Free(7) + Free(7) = 14 qpos_spring entries
        model.qpos_spring = vec![
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // joint 0
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // joint 1
        ];
        model.jnt_damping = vec![0.0; 2];
        model.jnt_solref = vec![[0.02, 1.0]; 2];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.jnt_name = vec![Some("j1".into()), Some("j2".into())];

        // --- DOF metadata (12 DOFs: 6 per free joint) ---
        model.dof_body = vec![1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2];
        model.dof_jnt = vec![0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1];
        model.dof_parent = vec![
            None,
            Some(0),
            Some(1),
            Some(2),
            Some(3),
            Some(4),
            None,
            Some(6),
            Some(7),
            Some(8),
            Some(9),
            Some(10),
        ];
        model.dof_armature = vec![0.0; 12];
        model.dof_damping = vec![0.0; 12];
        model.dof_frictionloss = vec![0.0; 12];

        // --- No sites needed for contact tests ---
        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        // --- Geoms: compute_contact_jacobian reads geom_body; mj_fwd_position
        // reads geom_pos/geom_quat to compute geom world poses ---
        model.ngeom = 3;
        model.geom_body = vec![0, 1, 2];
        model.geom_pos = vec![Vector3::zeros(); 3];
        model.geom_quat = vec![UnitQuaternion::identity(); 3];
        model.body_geom_adr = vec![0, 1, 2];
        model.body_geom_num = vec![1, 1, 1];

        // --- qpos0: identity quaternions for both free joints ---
        // qpos layout: [x1,y1,z1, w1,i1,j1,k1, x2,y2,z2, w2,i2,j2,k2]
        model.qpos0 = DVector::zeros(14);
        model.qpos0[3] = 1.0; // body 1: identity quaternion w-component
        model.qpos0[10] = 1.0; // body 2: identity quaternion w-component
        model.timestep = 0.001;

        // --- Ancestor metadata (required by compute_dof_patterns → make_data) ---
        model.body_ancestor_joints = vec![vec![]; 3];
        model.body_ancestor_mask = vec![vec![]; 3];
        model.compute_ancestors();
        model.compute_dof_patterns();

        let data = model.make_data();
        (model, data)
    }

    // =========================================================================
    // 5a — contact_jac_free_body_frame_vs_world_frame
    // =========================================================================

    /// Verify that free-joint angular DOF columns use body-frame axes (R·eᵢ),
    /// not world-frame unit vectors. At 45° about Z, R·e₀ ≠ e₀, so world-frame
    /// would produce measurably wrong values.
    #[test]
    fn contact_jac_free_body_frame_vs_world_frame() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1), // pos
            Vector3::new(0.0, 0.0, 1.0), // normal: +Z
            0.01,                        // depth
            0,                           // geom1 (world geom)
            1,                           // geom2 (free body geom)
            0.5,                         // friction → dim=3
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 3);
        assert_eq!(j.ncols(), 6);

        let rot = data.xquat[1].to_rotation_matrix();
        let r = contact.pos - data.xpos[1];
        let normal = contact.normal;

        // Verify angular DOF columns (3, 4, 5) use body-frame axes
        for i in 0..3 {
            let omega = rot * Vector3::ith(i, 1.0);
            let expected = normal.dot(&omega.cross(&r));
            assert_relative_eq!(j[(0, 3 + i)], expected, epsilon = 1e-12);
        }

        // Sanity: at 45° about Z, R·e₀ ≠ e₀ — world-frame would give wrong answer
        let body_e0 = rot * Vector3::x();
        assert!(
            (body_e0 - Vector3::x()).norm() > 0.1,
            "R·e₀ should differ from e₀ at 45°"
        );
    }

    // =========================================================================
    // 5b — contact_jac_all_rows_match_mj_jac_projection
    // =========================================================================

    /// Verify all 3 translational rows (normal + 2 tangents) of the contact
    /// Jacobian match the relative body Jacobian projected along each direction.
    #[test]
    fn contact_jac_all_rows_match_mj_jac_projection() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);

        let body1 = model.geom_body[contact.geom[0].unwrap()]; // world (body 0)
        let body2 = model.geom_body[contact.geom[1].unwrap()]; // free body

        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);
        let (jacp1, _) = mj_jac(&model, &data, body1, &contact.pos);

        let directions = [contact.normal, contact.frame[0], contact.frame[1]];
        let nv = model.nv;
        for (row, dir) in directions.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5c — angular_jac_free_body_frame (torsional + rolling, condim=6)
    // =========================================================================

    /// Verify all 6 rows (normal + 2 tangent + torsional + 2 rolling) of a
    /// condim=6 contact match the relative translational and rotational
    /// Jacobians from mj_jac.
    #[test]
    fn angular_jac_free_body_frame() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,   // sliding friction
            0.01,  // torsional friction
            0.005, // rolling friction
            6_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 6);

        let body1 = model.geom_body[contact.geom[0].unwrap()];
        let body2 = model.geom_body[contact.geom[1].unwrap()];

        let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);
        let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);

        let nv = model.nv;

        // Rows 0–2 (normal + tangents): relative translational Jacobian projection
        let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (row, dir) in trans_dirs.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }

        // Row 3 (torsional): relative rotational Jacobian projected along normal
        // Rows 4–5 (rolling): relative rotational Jacobian projected along tangents
        let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (i, dir) in rot_dirs.iter().enumerate() {
            let row = 3 + i;
            for dof in 0..nv {
                let col2 = Vector3::new(jacr2[(0, dof)], jacr2[(1, dof)], jacr2[(2, dof)]);
                let col1 = Vector3::new(jacr1[(0, dof)], jacr1[(1, dof)], jacr1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5d — contact_jac_two_free_bodies (strongest: both bodies non-identity)
    // =========================================================================

    /// Two free-joint bodies at different non-identity orientations, condim=6
    /// contact between them. Validates all 6 rows × 12 DOFs against mj_jac.
    #[test]
    fn contact_jac_two_free_bodies() {
        let (model, mut data) = make_two_free_body_contact_model();

        // Body 1: 45° about Z
        let q1 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q1.w;
        data.qpos[4] = q1.i;
        data.qpos[5] = q1.j;
        data.qpos[6] = q1.k;
        // Body 2: 30° about X
        let q2 = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_6);
        data.qpos[10] = q2.w;
        data.qpos[11] = q2.i;
        data.qpos[12] = q2.j;
        data.qpos[13] = q2.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            1, // geom1 on body 1
            2, // geom2 on body 2
            0.5,
            0.01,
            0.005,
            6_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 12);

        let body1 = model.geom_body[contact.geom[0].unwrap()]; // body 1
        let body2 = model.geom_body[contact.geom[1].unwrap()]; // body 2

        let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);
        let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);

        let nv = model.nv;

        // Rows 0–2: relative translational Jacobian
        let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (row, dir) in trans_dirs.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }

        // Rows 3–5: relative rotational Jacobian
        let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (i, dir) in rot_dirs.iter().enumerate() {
            let row = 3 + i;
            for dof in 0..nv {
                let col2 = Vector3::new(jacr2[(0, dof)], jacr2[(1, dof)], jacr2[(2, dof)]);
                let col1 = Vector3::new(jacr1[(0, dof)], jacr1[(1, dof)], jacr1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5e — contact_jac_free_identity_unchanged (regression)
    // =========================================================================

    /// At identity orientation, body-frame axes coincide with world-frame axes.
    /// Verify the contact Jacobian produces the same values as both the old
    /// world-frame formula and the mj_jac projection.
    #[test]
    fn contact_jac_free_identity_unchanged() {
        let (model, data) = make_free_body_contact_model();

        // Confirm identity orientation
        let rot = data.xquat[1].to_rotation_matrix();
        for i in 0..3 {
            let body_axis = rot * Vector3::ith(i, 1.0);
            let world_axis = Vector3::ith(i, 1.0);
            assert_relative_eq!(body_axis, world_axis, epsilon = 1e-15);
        }

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);

        // Cross-check against mj_jac projection
        let body2 = model.geom_body[contact.geom[1].unwrap()];
        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);

        let directions = [contact.normal, contact.frame[0], contact.frame[1]];
        let nv = model.nv;
        for (row, dir) in directions.iter().enumerate() {
            for dof in 0..nv {
                let col = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let expected = dir.dot(&col);
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5f — contact_jac_free_frictionless (condim=1)
    // =========================================================================

    /// Frictionless contact (condim=1, dim=1): only the normal row, no tangent,
    /// torsional, or rolling rows. Verify angular DOFs use body-frame axes.
    #[test]
    fn contact_jac_free_frictionless() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z (non-identity to make body-frame meaningful)
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::z(),
            0.01,
            0,
            1,
            0.0, // no sliding friction
            0.0, // no torsional
            0.0, // no rolling
            1_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 1);
        assert_eq!(j.ncols(), 6);

        // Cross-check the single normal row against mj_jac projection
        let body2 = model.geom_body[contact.geom[1].unwrap()];
        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);
        let normal = contact.normal;

        for dof in 0..model.nv {
            let col = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
            let expected = normal.dot(&col);
            assert_relative_eq!(j[(0, dof)], expected, epsilon = 1e-12);
        }
    }
}
