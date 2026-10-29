//! Quaternion and orientation helpers for MJCF element processing.
//!
//! Converts between MJCF's (w, x, y, z) quaternion convention and nalgebra's
//! internal representation, and resolves an element's orientation — its
//! quaternion or one of the four alternatives (euler, axisangle, xyaxes,
//! zaxis) — as MuJoCo 3.5.0's compiler does: a transcription of
//! `ResolveOrientation` (`user_objects.cc:239-346`) and the `user_util.cc`
//! helpers it calls, in plain arithmetic, each product and sum in the C
//! source's order.

use std::f64::consts::PI;

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};

use crate::error::MjcfError;
use crate::types::{AngleUnit, MjcfCompiler, MjcfSite};

/// MuJoCo's compiler's `mjEPS` (`user_util.h:29`).
pub const MJ_EPS: f64 = 1e-14;

/// `mjuu_normvec` (`user_util.cc:149-169`): `v` made unit, unless its norm
/// lies within mjEPS of 1, and the norm returned; where its squared norm is
/// below mjEPS, `v` is left as it is and 0 returned.
pub fn normvec<const N: usize>(v: &mut [f64; N]) -> f64 {
    let mut nrm = 0.0;
    for x in v.iter() {
        nrm += x * x;
    }
    if nrm < MJ_EPS {
        return 0.0;
    }
    let nrm = nrm.sqrt();
    if (nrm - 1.0).abs() > MJ_EPS {
        for x in v.iter_mut() {
            *x /= nrm;
        }
    }
    nrm
}

/// `mjuu_z2quat` (`user_util.cc:395-408`): the turn (w, x, y, z) of the z
/// axis onto `vec`, about their cross product made unit, or about x where
/// `normvec` returns 0 for it (a norm below 1e-7).
pub fn z2quat(vec: &[f64; 3]) -> [f64; 4] {
    let z = [0.0, 0.0, 1.0];
    let mut axis = [
        z[1] * vec[2] - z[2] * vec[1],
        z[2] * vec[0] - z[0] * vec[2],
        z[0] * vec[1] - z[1] * vec[0],
    ];
    let s = normvec(&mut axis);
    if s < 1e-10 {
        axis = [1.0, 0.0, 0.0];
    }
    let ang = s.atan2(vec[2]);
    [
        (ang / 2.0).cos(),
        axis[0] * (ang / 2.0).sin(),
        axis[1] * (ang / 2.0).sin(),
        axis[2] * (ang / 2.0).sin(),
    ]
}

/// `mjuu_mulquat` (`user_util.cc:245-253`): `qa · qb`, made unit by `normvec`.
pub fn mulquat(qa: &[f64; 4], qb: &[f64; 4]) -> [f64; 4] {
    let mut r = [
        qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3],
        qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2],
        qa[0] * qb[2] - qa[1] * qb[3] + qa[2] * qb[0] + qa[3] * qb[1],
        qa[0] * qb[3] + qa[1] * qb[2] - qa[2] * qb[1] + qa[3] * qb[0],
    ];
    normvec(&mut r);
    r
}

/// `mjuu_frame2quat` (`user_util.cc:412-448`): the quaternion of the frame
/// whose columns are `x`, `y` and `z`, taken from its largest component, made
/// unit by `normvec`.
fn frame2quat(x: &[f64; 3], y: &[f64; 3], z: &[f64; 3]) -> [f64; 4] {
    // mat[c][r], as MuJoCo indexes it.
    let mat = [x, y, z];
    let mut q = [0.0; 4];
    if mat[0][0] + mat[1][1] + mat[2][2] > 0.0 {
        q[0] = 0.5 * (1.0 + mat[0][0] + mat[1][1] + mat[2][2]).sqrt();
        q[1] = 0.25 * (mat[1][2] - mat[2][1]) / q[0];
        q[2] = 0.25 * (mat[2][0] - mat[0][2]) / q[0];
        q[3] = 0.25 * (mat[0][1] - mat[1][0]) / q[0];
    } else if mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2] {
        q[1] = 0.5 * (1.0 + mat[0][0] - mat[1][1] - mat[2][2]).sqrt();
        q[0] = 0.25 * (mat[1][2] - mat[2][1]) / q[1];
        q[2] = 0.25 * (mat[1][0] + mat[0][1]) / q[1];
        q[3] = 0.25 * (mat[2][0] + mat[0][2]) / q[1];
    } else if mat[1][1] > mat[2][2] {
        q[2] = 0.5 * (1.0 - mat[0][0] + mat[1][1] - mat[2][2]).sqrt();
        q[0] = 0.25 * (mat[2][0] - mat[0][2]) / q[2];
        q[1] = 0.25 * (mat[1][0] + mat[0][1]) / q[2];
        q[3] = 0.25 * (mat[2][1] + mat[1][2]) / q[2];
    } else {
        q[3] = 0.5 * (1.0 - mat[0][0] - mat[1][1] + mat[2][2]).sqrt();
        q[0] = 0.25 * (mat[0][1] - mat[1][0]) / q[3];
        q[1] = 0.25 * (mat[2][0] + mat[0][2]) / q[3];
        q[2] = 0.25 * (mat[2][1] + mat[1][2]) / q[3];
    }
    normvec(&mut q);
    q
}

/// An alternative MuJoCo 3.5.0's `ResolveOrientation` refuses
/// (`user_objects.cc:239-346`): its attribute, and MuJoCo's reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OrientationError {
    /// The alternative's attribute.
    pub attribute: &'static str,
    /// MuJoCo's reason, as its compiler words it.
    pub reason: &'static str,
}

impl OrientationError {
    /// The refusal of an `element`'s alternative.
    pub fn on(self, element: &str) -> MjcfError {
        MjcfError::invalid_value(element, self.attribute, self.reason)
    }
}

/// `q` (w, x, y, z), already unit by MuJoCo's arithmetic, as a `UnitQuaternion`.
fn unit(q: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::new_unchecked(Quaternion::new(q[0], q[1], q[2], q[3]))
}

/// Convert an MJCF quaternion `[w, x, y, z]` to a `UnitQuaternion`.
pub fn quat_from_wxyz(q: nalgebra::Vector4<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]))
}

/// A site's orientation as MuJoCo's compiler holds it before composing the
/// site's frame (`user_objects.cc:3976-3985`): its alternative where it has
/// one, else its quaternion as written, not yet made unit.
pub fn site_quat_as_written(
    site: &MjcfSite,
    compiler: &MjcfCompiler,
) -> Result<UnitQuaternion<f64>, OrientationError> {
    if let Some(q) = alternative(
        site.euler,
        site.axisangle,
        site.xyaxes,
        site.zaxis,
        compiler,
    )? {
        return Ok(unit(q));
    }
    let quat = site.quat.unwrap_or(Vector4::new(1.0, 0.0, 0.0, 0.0));
    Ok(unit([quat[0], quat[1], quat[2], quat[3]]))
}

/// A site's orientation made unit as MuJoCo's compiler makes it (`normvec`,
/// `user_objects.cc:3988`): left as it is where its norm lies within mjEPS of
/// 1 (`quat_from_wxyz` divides by the norm whatever it is).
pub fn site_orientation(
    site: &MjcfSite,
    compiler: &MjcfCompiler,
) -> Result<UnitQuaternion<f64>, OrientationError> {
    let q = site_quat_as_written(site, compiler)?.into_inner();
    let mut q = [q.w, q.i, q.j, q.k];
    normvec(&mut q);
    Ok(unit(q))
}

/// Convert a `UnitQuaternion` back to MJCF `Vector4<f64>` (w, x, y, z order).
pub fn quat_to_wxyz(q: &UnitQuaternion<f64>) -> Vector4<f64> {
    let qi = q.into_inner();
    Vector4::new(qi.w, qi.i, qi.j, qi.k)
}

/// An angle in radians: MuJoCo's `x / 180 * mjPI` where the compiler's unit is
/// degrees (`user_objects.cc:254`, `:309`).
fn radians(x: f64, compiler: &MjcfCompiler) -> f64 {
    if compiler.angle == AngleUnit::Degree {
        x / 180.0 * PI
    } else {
        x
    }
}

/// MuJoCo's turn by euler angles `euler_rad` (radians) about the axes `seq`
/// names (`ResolveOrientation`, `user_objects.cc:304-343`): from the identity,
/// each axis's turn (cos(e/2), sin(e/2) on the axis) post-multiplied for a
/// lowercase letter (moving axes) and pre-multiplied for an uppercase one
/// (fixed axes), each product made unit (`mulquat`), then the result.
pub fn euler_seq_to_quat(euler_rad: Vector3<f64>, seq: &str) -> UnitQuaternion<f64> {
    let mut quat = [1.0, 0.0, 0.0, 0.0];
    for (ch, &e) in seq.chars().zip(euler_rad.iter()) {
        let sa = (e / 2.0).sin();
        let mut qrot = [(e / 2.0).cos(), 0.0, 0.0, 0.0];
        match ch.to_ascii_lowercase() {
            'x' => qrot[1] = sa,
            'y' => qrot[2] = sa,
            // 'z', the parser having refused any other letter
            _ => qrot[3] = sa,
        }
        quat = if ch.is_ascii_lowercase() {
            mulquat(&quat, &qrot)
        } else {
            mulquat(&qrot, &quat)
        };
    }
    normvec(&mut quat);
    unit(quat)
}

/// An element's alternative orientation as MuJoCo 3.5.0's `ResolveOrientation`
/// makes it (`user_objects.cc:239-346`), or `None` where it has none. A
/// document gives an element at most one (the reader refuses two, and an
/// element's or class's own replace the ones it inherits:
/// `defaults::alternative_over`); where code sets several, euler > axisangle >
/// xyaxes > zaxis.
fn alternative(
    euler: Option<Vector3<f64>>,
    axisangle: Option<Vector4<f64>>,
    xyaxes: Option<[f64; 6]>,
    zaxis: Option<Vector3<f64>>,
    compiler: &MjcfCompiler,
) -> Result<Option<[f64; 4]>, OrientationError> {
    let refuse = |attribute, reason| Err(OrientationError { attribute, reason });
    if let Some(euler) = euler {
        let e = euler.map(|x| radians(x, compiler));
        let q = euler_seq_to_quat(e, &compiler.eulerseq).into_inner();
        return Ok(Some([q.w, q.i, q.j, q.k]));
    }
    if let Some(aa) = axisangle {
        let ang2 = radians(aa.w, compiler) / 2.0;
        let mut axis = [aa.x, aa.y, aa.z];
        if normvec(&mut axis) < MJ_EPS {
            return refuse("axisangle", "axisangle too small");
        }
        return Ok(Some([
            ang2.cos(),
            ang2.sin() * axis[0],
            ang2.sin() * axis[1],
            ang2.sin() * axis[2],
        ]));
    }
    if let Some(xy) = xyaxes {
        let mut x = [xy[0], xy[1], xy[2]];
        let mut y = [xy[3], xy[4], xy[5]];
        if normvec(&mut x) < MJ_EPS {
            return refuse("xyaxes", "xaxis too small");
        }
        let d = x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        y[0] -= x[0] * d;
        y[1] -= x[1] * d;
        y[2] -= x[2] * d;
        if normvec(&mut y) < MJ_EPS {
            return refuse("xyaxes", "yaxis too small");
        }
        let mut z = [
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ];
        // 0 where an axis's sum of squares overflowed: `normvec` then divided it
        // by an infinite norm, to 0.
        if normvec(&mut z) < MJ_EPS {
            return refuse("xyaxes", "cross(xaxis, yaxis) too small");
        }
        return Ok(Some(frame2quat(&x, &y, &z)));
    }
    if let Some(zaxis) = zaxis {
        let mut z = [zaxis.x, zaxis.y, zaxis.z];
        if normvec(&mut z) < MJ_EPS {
            return refuse("zaxis", "zaxis too small");
        }
        return Ok(Some(z2quat(&z)));
    }
    Ok(None)
}

/// A body's, geom's or frame's orientation as MuJoCo 3.5.0's compiler makes it
/// (`mjCBody::Compile`, `user_objects.cc:2387-2402`; `mjCGeom::Compile`,
/// `:3673-3724`; `mjCFrame::Compile`, `:2734-2746`): its alternative where it
/// has one, else its quaternion made unit by `normvec`.
pub fn resolve_orientation(
    quat: Vector4<f64>,
    euler: Option<Vector3<f64>>,
    axisangle: Option<Vector4<f64>>,
    xyaxes: Option<[f64; 6]>,
    zaxis: Option<Vector3<f64>>,
    compiler: &MjcfCompiler,
) -> Result<UnitQuaternion<f64>, OrientationError> {
    if let Some(q) = alternative(euler, axisangle, xyaxes, zaxis, compiler)? {
        return Ok(unit(q));
    }
    let mut q = [quat[0], quat[1], quat[2], quat[3]];
    normvec(&mut q);
    Ok(unit(q))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn test_euler_seq_to_quat_intrinsic_xyz() {
        // Default MuJoCo sequence: intrinsic xyz → Rx * Ry * Rz
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "xyz");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = rx * ry * rz;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "intrinsic xyz failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_to_quat_extrinsic_xyz() {
        // Extrinsic XYZ → pre-multiply: Rz * Ry * Rx (= intrinsic zyx)
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "XYZ");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = rz * ry * rx;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "extrinsic XYZ failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_to_quat_mixed_case() {
        // Mixed: XYz → extrinsic X, extrinsic Y, intrinsic z
        // q = identity
        // ch='X' (extrinsic): q = Rx * q = Rx
        // ch='Y' (extrinsic): q = Ry * q = Ry * Rx
        // ch='z' (intrinsic): q = q * Rz = Ry * Rx * Rz
        let euler = Vector3::new(0.1, 0.2, 0.3);
        let q = euler_seq_to_quat(euler, "XYz");

        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let expected = ry * rx * rz;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "mixed XYz failed: got {q:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_zyx_matches_different_order() {
        // Intrinsic zyx should match nalgebra's from_euler_angles(roll, pitch, yaw)
        // which is extrinsic XYZ
        let euler = Vector3::new(0.3, 0.2, 0.1); // z, y, x angles
        let q = euler_seq_to_quat(euler, "zyx");

        // Intrinsic zyx: Rz * Ry * Rx
        let rz = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3);
        let ry = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let rx = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1);
        let expected = rz * ry * rx;

        assert!(
            (q.into_inner() - expected.into_inner()).norm() < 1e-12,
            "intrinsic zyx failed"
        );
    }

    // -- Body orientation tests (from monolith) --

    #[test]
    fn test_body_axisangle_orientation() {
        use crate::builder::load_model;
        // Body with axisangle should produce correct quaternion.
        // axisangle="0 0 1 90" with default degree -> 90deg around Z.
        let model = load_model(
            r#"
            <mujoco model="axisangle_test">
                <worldbody>
                    <body name="b" axisangle="0 0 1 90">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // 90deg around Z: quaternion = [cos(45deg), 0, 0, sin(45deg)]
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.body_quat[1]; // body 0 is world, body 1 is "b"
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "axisangle 90deg Z: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_body_axisangle_radian() {
        use crate::builder::load_model;
        // With angle="radian", axisangle angle is in radians.
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
                <mujoco model="axisangle_rad">
                    <compiler angle="radian"/>
                    <worldbody>
                        <body name="b" axisangle="0 0 1 {pi_2}">
                            <geom type="sphere" size="0.1" mass="1.0"/>
                        </body>
                    </worldbody>
                </mujoco>
                "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), pi_2);
        let got = model.body_quat[1];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "axisangle radian: got {got:?}, expected {expected:?}"
        );
    }

    #[test]
    fn test_euler_seq_zyx_body_orientation() {
        use crate::builder::load_model;
        // With eulerseq="ZYX" and angle="radian", body euler should use ZYX sequence.
        let model = load_model(
            r#"
            <mujoco model="eulerseq_test">
                <compiler angle="radian" eulerseq="ZYX"/>
                <worldbody>
                    <body name="b" euler="0.3 0.2 0.1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX");
        let got = model.body_quat[1];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "ZYX euler: got {got:?}, expected {expected:?}"
        );
    }

    // -- A body's xyaxes and zaxis (ledger L249). Expected values are MuJoCo
    // 3.5.0's `body_quat`, `body_pos` and `site_pos`/`site_quat`; within 1e-12,
    // as they pass through sin, cos and sqrt (MuJoCo's arithmetic is not yet
    // ours here). MuJoCo's 0.7071067811865476 is `FRAC_1_SQRT_2`.

    use std::f64::consts::FRAC_1_SQRT_2;

    fn assert_quat(got: &UnitQuaternion<f64>, mujoco: [f64; 4], what: &str) {
        let g = got.into_inner();
        let g = [g.w, g.i, g.j, g.k];
        for k in 0..4 {
            assert!(
                (g[k] - mujoco[k]).abs() < 1e-12,
                "{what}: got {g:?}, MuJoCo {mujoco:?}"
            );
        }
    }

    fn assert_vec(got: &Vector3<f64>, mujoco: [f64; 3], what: &str) {
        for k in 0..3 {
            assert!(
                (got[k] - mujoco[k]).abs() < 1e-12,
                "{what}: got {got:?}, MuJoCo {mujoco:?}"
            );
        }
    }

    fn body_quat_of(xml: &str) -> Vec<UnitQuaternion<f64>> {
        crate::builder::load_model(xml)
            .expect("should load")
            .body_quat
    }

    #[test]
    fn a_bodys_xyaxes_and_zaxis_turn_it_as_mujocos_do() {
        let one = |attr: &str| {
            body_quat_of(&format!(
                r#"<mujoco><worldbody><body name="b" {attr}><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#
            ))[1]
        };
        assert_quat(
            &one(r#"xyaxes="0 1 0 -1 0 0""#),
            [FRAC_1_SQRT_2, 0.0, 0.0, 0.707_106_781_186_547_5],
            "xyaxes 0 1 0 -1 0 0",
        );
        assert_quat(
            &one(r#"zaxis="1 0 0""#),
            [FRAC_1_SQRT_2, 0.0, 0.707_106_781_186_547_5, 0.0],
            "zaxis 1 0 0",
        );
        assert_quat(
            &one(r#"xyaxes="1 2 3 -1 0.5 0.2""#),
            [
                0.746_648_941_770_3,
                0.275_945_608_244_638_2,
                -0.356_005_394_554_691_43,
                0.489_519_701_433_707_8,
            ],
            "xyaxes 1 2 3 -1 0.5 0.2",
        );
        assert_quat(
            &one(r#"zaxis="0.3 -0.7 1.1""#),
            [
                0.954_509_979_525_392_3,
                0.274_069_719_620_169,
                0.117_458_451_265_786_7,
                0.0,
            ],
            "zaxis 0.3 -0.7 1.1",
        );
    }

    #[test]
    fn a_bodys_xyaxes_and_zaxis_in_a_turned_frame_are_composed_with_it() {
        let in_frame = |attr: &str| {
            body_quat_of(&format!(
                r#"<mujoco><worldbody><frame euler="30 40 50"><body name="b" {attr}><joint type="hinge"/><geom type="sphere" size="0.1"/></body></frame></worldbody></mujoco>"#
            ))[1]
        };
        assert_quat(
            &in_frame(r#"zaxis="0.3 -0.7 1.1""#),
            [
                0.627_728_704_244_400_1,
                0.504_388_678_536_701_7,
                0.407_035_323_864_856_97,
                0.431_127_568_079_190_95,
            ],
            "zaxis in a frame",
        );
        assert_quat(
            &in_frame(r#"xyaxes="1 2 3 -1 0.5 0.2""#),
            [
                0.329_880_457_753_313_87,
                0.746_881_588_259_864,
                -0.180_987_292_478_980_27,
                0.548_261_230_320_014,
            ],
            "xyaxes in a frame",
        );
    }

    #[test]
    fn a_fused_bodys_zaxis_turns_its_site() {
        let model = crate::builder::load_model(
            r#"<mujoco><compiler fusestatic="true"/><worldbody>
                <body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/>
                    <body name="b" pos="0.1 0 0" zaxis="1 0 0"><site name="s" pos="0 0 0.1"/></body>
                </body>
            </worldbody></mujoco>"#,
        )
        .expect("should load");
        assert_eq!(model.nbody, 2, "b is fused into p");
        assert_vec(&model.site_pos[0], [0.2, 0.0, 0.0], "site_pos");
        assert_quat(
            &model.site_quat[0],
            [FRAC_1_SQRT_2, 0.0, 0.707_106_781_186_547_5, 0.0],
            "site_quat",
        );
    }

    #[test]
    fn a_fused_bodys_xyaxes_moves_its_child_whose_own_zaxis_is_kept() {
        let model = crate::builder::load_model(
            r#"<mujoco><compiler fusestatic="true"/><worldbody>
                <body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/>
                    <body name="b" pos="0.1 0 0" xyaxes="0 1 0 -1 0 0">
                        <body name="c" pos="0 0.2 0" zaxis="0 1 0"><joint type="hinge"/><geom type="sphere" size="0.1"/></body>
                    </body>
                </body>
            </worldbody></mujoco>"#,
        )
        .expect("should load");
        assert_eq!(model.nbody, 3, "b is fused into p");
        assert_vec(&model.body_pos[2], [-0.1, 0.0, 0.0], "c's body_pos");
        assert_quat(
            &model.body_quat[2],
            [0.500_000_000_000_000_1, -0.5, -0.499_999_999_999_999_9, 0.5],
            "c's body_quat",
        );
    }
}
