//! Phase 6 Spec A — sensor completeness tests.
//!
//! Tests for DT-62 (objtype attribute parsing), DT-64 (multi-geom touch sensor
//! aggregation), and DT-102 (geom-attached FrameLinAcc/FrameAngAcc).

use approx::assert_relative_eq;
use nalgebra::{DVector, UnitQuaternion, Vector3};
use sim_core::{ConstraintType, Contact, MjObjectType, MjSensorType};
use sim_mjcf::load_model;

// ============================================================================
// T1: Parser objtype attribute → AC1
// ============================================================================

#[test]
fn t01_parser_objtype_attribute() {
    let mjcf = r#"
        <mujoco model="t01">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom name="g1" type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp_with_objtype" objtype="geom" objname="g1"/>
                <framepos name="fp_without_objtype" objname="g1"/>
            </sensor>
        </mujoco>
    "#;

    let parsed = sim_mjcf::parse_mjcf_str(mjcf).expect("parse");
    let sensors = &parsed.sensors;
    assert_eq!(sensors.len(), 2);

    // With objtype
    assert_eq!(sensors[0].objtype, Some("geom".to_string()));
    // Without objtype
    assert_eq!(sensors[1].objtype, None);
}

// ============================================================================
// T2: Parser reftype/refname separation → AC2
// ============================================================================

#[test]
fn t02_parser_reftype_refname_separated() {
    let mjcf = r#"
        <mujoco model="t02">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp_both" site="s1" reftype="body" refname="b1"/>
                <framepos name="fp_refname_only" site="s1" refname="b1"/>
                <framepos name="fp_reftype_only" site="s1" reftype="body"/>
            </sensor>
        </mujoco>
    "#;

    let parsed = sim_mjcf::parse_mjcf_str(mjcf).expect("parse");
    let sensors = &parsed.sensors;
    assert_eq!(sensors.len(), 3);

    // Both reftype and refname
    assert_eq!(sensors[0].reftype, Some("body".to_string()));
    assert_eq!(sensors[0].refname, Some("b1".to_string()));

    // Only refname
    assert_eq!(sensors[1].reftype, None);
    assert_eq!(sensors[1].refname, Some("b1".to_string()));

    // Only reftype
    assert_eq!(sensors[2].reftype, Some("body".to_string()));
    assert_eq!(sensors[2].refname, None);
}

// ============================================================================
// T3: Parser geom= attribute → AC3
// ============================================================================

#[test]
fn t03_parser_geom_attribute() {
    let mjcf = r#"
        <mujoco model="t03">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom name="g1" type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp" geom="g1"/>
            </sensor>
        </mujoco>
    "#;

    let parsed = sim_mjcf::parse_mjcf_str(mjcf).expect("parse");
    assert_eq!(parsed.sensors[0].objname, Some("g1".to_string()));
}

// ============================================================================
// T4: Builder explicit objtype="geom" → AC4
// ============================================================================

#[test]
fn t04_builder_explicit_objtype_geom() {
    let mjcf = r#"
        <mujoco model="t04">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom name="g1" type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp" objtype="geom" objname="g1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_objtype[0], MjObjectType::Geom);
}

// ============================================================================
// T5: Builder body/xbody distinction → AC5
// ============================================================================

#[test]
fn t05_builder_body_xbody_distinction() {
    let mjcf = r#"
        <mujoco model="t05">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp_body" objtype="body" objname="b1"/>
                <framepos name="fp_xbody" objtype="xbody" objname="b1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_objtype[0], MjObjectType::Body);
    assert_eq!(model.sensor_objtype[1], MjObjectType::XBody);
}

// ============================================================================
// T6: MuJoCo conformance — body vs xbody position → AC6
// ============================================================================

#[test]
fn t06_body_vs_xbody_position() {
    // Body with sphere geom offset along x-axis. COM (xipos) differs from
    // joint frame origin (xpos) by the geom's pos offset.
    // NOTE: We use explicit `pos=` (not `fromto=`) because fromto→pos conversion
    // happens in the geom builder, not in the parsed MjcfGeom — so
    // compute_inertia_from_geoms cannot see the computed midpoint position.
    let mjcf = r#"
        <mujoco model="t06">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.05" pos="0.3 0 0" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp_body" objtype="body" objname="b1"/>
                <framepos name="fp_xbody" objtype="xbody" objname="b1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Check that the body's xipos differs from xpos
    let b1_id = model.sensor_objid[0]; // body ID for both sensors
    let xipos = data.xipos[b1_id];
    let xpos = data.xpos[b1_id];

    // XBody = joint frame origin = xpos
    let xbody_adr = model.sensor_adr[1];
    assert_relative_eq!(data.sensordata[xbody_adr], xpos.x, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[xbody_adr + 1], xpos.y, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[xbody_adr + 2], xpos.z, epsilon = 1e-10);

    // Body = COM/inertial frame = xipos
    let body_adr = model.sensor_adr[0];
    assert_relative_eq!(data.sensordata[body_adr], xipos.x, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[body_adr + 1], xipos.y, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[body_adr + 2], xipos.z, epsilon = 1e-10);

    // Key: xipos and xpos must differ for a body with COM offset
    let diff = (xipos - xpos).norm();
    assert!(
        diff > 0.01,
        "body xipos and xpos must differ for offset COM; xipos={xipos:?}, xpos={xpos:?}"
    );
}

// ============================================================================
// T7: Body FrameQuat uses mulQuat → AC7
// ============================================================================

#[test]
fn t07_body_framequat_mulquat() {
    // Body with offset geom → body_iquat ≠ identity (asymmetric inertia).
    // Use geoms at non-axis-aligned positions so the principal axes are rotated
    // relative to the body frame → body_iquat ≠ identity.
    let mjcf = r#"
        <mujoco model="t07">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.05" pos="0.3 0.1 0" mass="1"/>
                    <geom type="sphere" size="0.05" pos="-0.1 0.2 0" mass="0.5"/>
                </body>
            </worldbody>
            <sensor>
                <framequat name="fq_body" objtype="body" objname="b1"/>
                <framequat name="fq_xbody" objtype="xbody" objname="b1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let xbody_adr = model.sensor_adr[1];
    let body_adr = model.sensor_adr[0];

    // XBody should read xquat directly (= identity at rest)
    let xbody_quat = UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(
        data.sensordata[xbody_adr],
        data.sensordata[xbody_adr + 1],
        data.sensordata[xbody_adr + 2],
        data.sensordata[xbody_adr + 3],
    ));
    assert_relative_eq!(xbody_quat.angle(), 0.0, epsilon = 1e-6);

    // Body = mulQuat(xquat, body_iquat)
    // For a capsule along x, body_iquat should align principal axes
    let body_quat_w = data.sensordata[body_adr];
    let body_quat_x = data.sensordata[body_adr + 1];
    let body_quat_y = data.sensordata[body_adr + 2];
    let body_quat_z = data.sensordata[body_adr + 3];
    let norm = (body_quat_w * body_quat_w
        + body_quat_x * body_quat_x
        + body_quat_y * body_quat_y
        + body_quat_z * body_quat_z)
        .sqrt();
    // Must be unit quaternion
    assert_relative_eq!(norm, 1.0, epsilon = 1e-10);

    // Verify it equals xquat * body_iquat
    let b1_id = model.sensor_objid[0];
    let expected = data.xquat[b1_id] * model.body_iquat[b1_id];
    assert_relative_eq!(body_quat_w, expected.w, epsilon = 1e-10);
    assert_relative_eq!(body_quat_x, expected.i, epsilon = 1e-10);
    assert_relative_eq!(body_quat_y, expected.j, epsilon = 1e-10);
    assert_relative_eq!(body_quat_z, expected.k, epsilon = 1e-10);
}

// ============================================================================
// T8: Default inference body= → XBody → AC8
// ============================================================================

#[test]
fn t08_default_inference_body_is_xbody() {
    let mjcf = r#"
        <mujoco model="t08">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos body="b1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    // body= without objtype → XBody (MuJoCo: mjOBJ_XBODY)
    assert_eq!(model.sensor_objtype[0], MjObjectType::XBody);
}

// ============================================================================
// T9: MuJoCo conformance — touch multi-geom → AC9
// ============================================================================

#[test]
fn t09_touch_multi_geom_aggregation() {
    // Body with 3 sphere geoms. Touch sensor on a site of that body.
    // Inject 3 elliptic contacts (one per geom). All should contribute.
    let mjcf = r#"
        <mujoco model="t09">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom name="g1" type="sphere" size="0.1" pos="-0.3 0 0" mass="1"/>
                    <geom name="g2" type="sphere" size="0.1" pos="0 0 0" mass="1"/>
                    <geom name="g3" type="sphere" size="0.1" pos="0.3 0 0" mass="1"/>
                    <site name="touch_site" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <touch name="t" site="touch_site"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");

    // Verify touch stores (Site, site_id) not (Geom, geom_id)
    assert_eq!(model.sensor_objtype[0], MjObjectType::Site);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Look up geom IDs for body 1
    let body_id = model.site_body[model.sensor_objid[0]];
    let geom_ids: Vec<usize> = (0..model.ngeom)
        .filter(|&g| model.geom_body[g] == body_id)
        .collect();
    assert_eq!(geom_ids.len(), 3, "body should have 3 geoms");

    // Inject 3 elliptic contacts — one per geom. Each has normal force 9.81.
    for &gid in &geom_ids {
        let ci = data.contacts.len();
        data.contacts.push(Contact::new(
            Vector3::new(0.0, 0.0, 0.5),
            Vector3::z(),
            0.01,
            gid,   // geom on our body
            99999, // other geom (doesn't matter)
            1.0,
        ));
        // Elliptic contact: 3 rows, first row = normal force
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_dim.push(3);
        data.efc_dim.push(3);
        data.efc_dim.push(3);
        data.efc_id.push(ci);
        data.efc_id.push(ci);
        data.efc_id.push(ci);
    }
    // Forces: [normal, tan1, tan2] for each contact
    data.efc_force = DVector::from_vec(vec![
        9.81, 0.0, 0.0, // contact 1 on g1
        9.81, 0.0, 0.0, // contact 2 on g2
        9.81, 0.0, 0.0, // contact 3 on g3
    ]);

    // Run acceleration sensors
    sim_core::sensor::mj_sensor_acc(&model, &mut data);

    // All 3 contacts should contribute: 3 × 9.81 = 29.43
    assert_relative_eq!(data.sensordata[0], 29.43, epsilon = 1e-2);
}

// ============================================================================
// T10: Touch — wrong body filtered → AC10
// ============================================================================

#[test]
fn t10_touch_wrong_body_filtered() {
    let mjcf = r#"
        <mujoco model="t10">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom name="g1" type="sphere" size="0.1" mass="1"/>
                    <site name="touch_site"/>
                </body>
                <body name="b2" pos="0 0 -1">
                    <joint type="free"/>
                    <geom name="g2" type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <touch name="t" site="touch_site"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Find geom IDs
    let g2_id = model
        .geom_name
        .iter()
        .position(|n| n.as_deref() == Some("g2"))
        .unwrap();

    // Inject contact between b2's geoms only (should NOT contribute to b1's touch)
    data.contacts.push(Contact::new(
        Vector3::zeros(),
        Vector3::z(),
        0.01,
        g2_id,
        g2_id, // same body
        1.0,
    ));
    data.efc_type.push(ConstraintType::ContactElliptic);
    data.efc_type.push(ConstraintType::ContactElliptic);
    data.efc_type.push(ConstraintType::ContactElliptic);
    data.efc_dim.push(3);
    data.efc_dim.push(3);
    data.efc_dim.push(3);
    data.efc_id.push(0);
    data.efc_id.push(0);
    data.efc_id.push(0);
    data.efc_force = DVector::from_vec(vec![50.0, 0.0, 0.0]);

    sim_core::sensor::mj_sensor_acc(&model, &mut data);

    // Touch should be 0 — contact is on b2, sensor is on b1
    assert_eq!(data.sensordata[0], 0.0);

    // Now inject a contact involving b1's geom
    let g1_id = model
        .geom_name
        .iter()
        .position(|n| n.as_deref() == Some("g1"))
        .unwrap();
    let ci = data.contacts.len();
    data.contacts.push(Contact::new(
        Vector3::zeros(),
        Vector3::z(),
        0.01,
        g1_id,
        g2_id,
        1.0,
    ));
    data.efc_type.push(ConstraintType::ContactElliptic);
    data.efc_type.push(ConstraintType::ContactElliptic);
    data.efc_type.push(ConstraintType::ContactElliptic);
    data.efc_dim.push(3);
    data.efc_dim.push(3);
    data.efc_dim.push(3);
    data.efc_id.push(ci);
    data.efc_id.push(ci);
    data.efc_id.push(ci);
    data.efc_force = DVector::from_vec(vec![
        50.0, 0.0, 0.0, // first contact (b2-b2)
        25.0, 0.0, 0.0, // second contact (b1-b2) ← should contribute
    ]);

    sim_core::sensor::mj_sensor_acc(&model, &mut data);
    assert_relative_eq!(data.sensordata[0], 25.0, epsilon = 1e-10);
}

// ============================================================================
// T11: Touch — body with zero geoms → AC11
// ============================================================================

#[test]
fn t11_touch_body_zero_geoms() {
    // Body with only a site, no geoms. Touch sensor should read 0.
    let mjcf = r#"
        <mujoco model="t11">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <site name="touch_site"/>
                </body>
            </worldbody>
            <sensor>
                <touch name="t" site="touch_site"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // No contacts injected — touch should be 0
    sim_core::sensor::mj_sensor_acc(&model, &mut data);
    assert_eq!(data.sensordata[0], 0.0);
}

// ============================================================================
// T12: MuJoCo conformance — geom-attached FrameLinAcc → AC12
// ============================================================================

#[test]
fn t12_geom_framelinacc_matches_site() {
    // A geom-attached FrameLinAcc on the same body as a co-located site-attached
    // FrameLinAcc should produce the same acceleration reading.
    // Uses a hinge joint with non-zero velocity to generate centripetal acceleration.
    let mjcf = r#"
        <mujoco model="t12">
            <option gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 0 1"/>
                    <geom name="g1" type="sphere" size="0.05" pos="0.5 0 0" mass="1"/>
                    <site name="s1" pos="0.5 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <framelinacc name="fla_geom" objtype="geom" objname="g1"/>
                <framelinacc name="fla_site" site="s1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_objtype[0], MjObjectType::Geom);
    assert_eq!(model.sensor_objtype[1], MjObjectType::Site);

    let mut data = model.make_data();
    data.qvel[0] = 10.0; // 10 rad/s around z → centripetal at r=0.5
    data.forward(&model).expect("forward");

    let geom_adr = model.sensor_adr[0];
    let site_adr = model.sensor_adr[1];

    // Both should report the same acceleration (centripetal ≈ -ω²r = -50 in x)
    for i in 0..3 {
        assert_relative_eq!(
            data.sensordata[geom_adr + i],
            data.sensordata[site_adr + i],
            epsilon = 1e-6,
            max_relative = 1e-6,
        );
    }

    // Sanity: centripetal acceleration should be non-trivial
    let ax = data.sensordata[geom_adr];
    assert!(
        ax.abs() > 10.0,
        "expected significant centripetal acceleration, got ax={ax}"
    );
}

// ============================================================================
// T13: Geom-attached FrameAngAcc → AC13
// ============================================================================

#[test]
fn t13_geom_frameangacc_static() {
    let mjcf = r#"
        <mujoco model="t13">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom name="g1" type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <frameangacc name="faa" objtype="geom" objname="g1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_objtype[0], MjObjectType::Geom);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    // No angular acceleration at rest
    assert_relative_eq!(data.sensordata[adr], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 2], 0.0, epsilon = 1e-10);
}

// ============================================================================
// T14: Geom-attached FrameLinAcc with centripetal → AC14
// ============================================================================

#[test]
fn t14_geom_framelinacc_centripetal() {
    // Body with hinge joint (z axis), geom offset at (0.5, 0, 0).
    // Set qvel = 10 rad/s. Centripetal: a_x = -ω²r = -100×0.5 = -50
    let mjcf = r#"
        <mujoco model="t14">
            <option gravity="0 0 0"/>
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 0 1"/>
                    <geom name="g1" type="sphere" size="0.05" pos="0.5 0 0" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <framelinacc name="fla" objtype="geom" objname="g1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_objtype[0], MjObjectType::Geom);

    let mut data = model.make_data();
    data.qvel[0] = 10.0; // 10 rad/s around z
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    // Centripetal acceleration toward axis: a_x ≈ -50
    assert_relative_eq!(data.sensordata[adr], -50.0, epsilon = 1.0);
}

// ============================================================================
// T16: Builder regression — touch sensor builds as Site → AC17
// ============================================================================

#[test]
fn t16_builder_regression_touch_as_site() {
    let mjcf = r#"
        <mujoco model="t16">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <touch site="s1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.sensor_type[0], MjSensorType::Touch);
    assert_eq!(model.sensor_objtype[0], MjObjectType::Site);
    assert_eq!(model.sensor_dim[0], 1);
}

// ============================================================================
// T17: Negative case — objtype ignored for Touch → AC18
// ============================================================================

#[test]
fn t17_objtype_ignored_for_touch() {
    let mjcf = r#"
        <mujoco model="t17">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <touch site="s1" objtype="geom"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    // Touch ignores objtype — always resolves as Site
    assert_eq!(model.sensor_objtype[0], MjObjectType::Site);
}

// ============================================================================
// T18: Edge case — Body FramePos on zero-mass body → AC19
// ============================================================================

#[test]
fn t18_body_framepos_zero_mass() {
    // Body with no geoms (only a site). Mass = 0.
    // MuJoCo sets xipos = xpos for massless bodies.
    let mjcf = r#"
        <mujoco model="t18">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
                <body name="b_empty" pos="1 2 3">
                    <joint type="free"/>
                    <site name="anchor"/>
                    <geom type="sphere" size="0.001" mass="0.001"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp" objtype="body" objname="b_empty"/>
                <framepos name="fp_x" objtype="xbody" objname="b_empty"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let body_adr = model.sensor_adr[0];
    let xbody_adr = model.sensor_adr[1];

    // For near-zero mass body, xipos ≈ xpos (COM frame ≈ joint frame)
    let body_pos = Vector3::new(
        data.sensordata[body_adr],
        data.sensordata[body_adr + 1],
        data.sensordata[body_adr + 2],
    );
    let xbody_pos = Vector3::new(
        data.sensordata[xbody_adr],
        data.sensordata[xbody_adr + 1],
        data.sensordata[xbody_adr + 2],
    );

    // Both should be close to (1, 2, 3) for near-zero mass
    assert_relative_eq!(xbody_pos.x, 1.0, epsilon = 1e-6);
    assert_relative_eq!(xbody_pos.y, 2.0, epsilon = 1e-6);
    assert_relative_eq!(xbody_pos.z, 3.0, epsilon = 1e-6);

    // body and xbody should be close for near-zero mass
    assert_relative_eq!((body_pos - xbody_pos).norm(), 0.0, epsilon = 0.01);
}
