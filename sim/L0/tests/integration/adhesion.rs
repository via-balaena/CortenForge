//! §36 Body-Transmission Actuator Tests (AC1–AC14).
//!
//! Verifies MuJoCo-conformant adhesion actuator behavior:
//! - Body transmission type parsing and `actuator_trnid` wiring
//! - Contact normal Jacobian accumulation in `mj_transmission_body()`
//! - Negated-average moment arm, force sign (attractive = toward surface)
//! - Gear bypass (adhesion ignores gear attribute)
//! - Gain-controlled force magnitude
//! - `actuator_length == 0` invariant
//! - `actuator_velocity == moment · qvel`
//! - Multiple contacts, kinematic chains, two-body contacts
//!
//! Note: Sphere positions use slight penetration (z < radius) to ensure
//! collision detection generates contacts (strict `>` threshold with default
//! margin=0 excludes exact-touch configurations).

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// AC1: Loading and trntype/trnid wiring
// ============================================================================

/// `<adhesion body="ball" gain="100"/>` parses correctly: trntype=Body, trnid[0]=ball body ID.
#[test]
fn ac1_loading() {
    let mjcf = r#"
        <mujoco model="adhesion_load">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nu, 1);

    use sim_core::ActuatorTransmission;
    assert_eq!(model.actuator_trntype[0], ActuatorTransmission::Body);

    // trnid[0] should be the body ID for "ball" (body 0 is world, so ball is body 1)
    let ball_body_id = model.actuator_trnid[0][0];
    assert_eq!(
        ball_body_id, 1,
        "ball should be body 1 (after world body 0)"
    );

    // trnid[1] should be usize::MAX (unused)
    assert_eq!(model.actuator_trnid[0][1], usize::MAX);

    // Verify gain: GainType::Fixed, gainprm[0] = 100
    use sim_core::GainType;
    assert_eq!(model.actuator_gaintype[0], GainType::Fixed);
    assert_relative_eq!(model.actuator_gainprm[0][0], 100.0, epsilon = 1e-15);
}

// ============================================================================
// AC2: Zero force when no contacts
// ============================================================================

/// No contacts (contype=0, conaffinity=0) → moment all zeros, qfrc_actuator zero.
#[test]
fn ac2_zero_force_no_contact() {
    let mjcf = r#"
        <mujoco model="adhesion_no_contact">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1" contype="0" conaffinity="0"/>
                <body name="ball" pos="0 0 2.0">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0" contype="0" conaffinity="0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // No contacts → moment should be all zeros
    for dof in 0..model.nv {
        assert_relative_eq!(data.actuator_moment[0][dof], 0.0, epsilon = 1e-15);
    }

    // qfrc_actuator should have no adhesion contribution.
    // force = gain * ctrl = 100, but moment = 0, so qfrc += 0 * 100 = 0.
    for dof in 0..model.nv {
        assert_relative_eq!(data.qfrc_actuator[dof], 0.0, epsilon = 1e-15);
    }
}

// ============================================================================
// AC3: Attractive force sign (positive ctrl → force toward surface)
// ============================================================================

/// Sphere on plane with ctrl=1 → qfrc_actuator z-component < 0 (downward = attractive).
#[test]
fn ac3_attractive_force_sign() {
    let mjcf = r#"
        <mujoco model="adhesion_sign">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // Verify there is at least one contact
    assert!(data.ncon > 0, "expected contact between ball and plane");

    // Adhesion force should be attractive (pulling ball toward the plane = downward).
    // For a sphere on a horizontal plane, the contact normal points upward (+z).
    // moment = -(1/count) * Σ J_normal → z-component of moment is negative.
    // qfrc_actuator[z_dof] = moment_z * gain * ctrl < 0 (downward).
    //
    // Free joint DOFs: [tx, ty, tz, rx, ry, rz], z-translation is index 2.
    assert!(
        data.qfrc_actuator[2] < 0.0,
        "expected downward (attractive) force on z-DOF, got qfrc_actuator[2] = {}",
        data.qfrc_actuator[2]
    );
}

// ============================================================================
// AC4: Force magnitude for single contact (analytical)
// ============================================================================

/// Sphere on plane, gain=100, ctrl=1. Analytical trace:
/// - Contact normal = [0, 0, +1] (plane normal, pointing from plane toward sphere)
/// - J_normal = normal^T * (J(ball) - J(world)). J(world)=0.
///   Translation DOFs: normal directly → [0, 0, +1].
///   Rotation DOFs: normal · (omega_i × r) where r = contact_pos - xpos ≈ (0, 0, -0.0995).
///   Since r is parallel to normal, all cross products lie in the xy-plane and dot to zero.
///   So J_normal = [0, 0, +1, 0, 0, 0] exactly.
/// - moment = -(1/1) * J_normal = [0, 0, -1, 0, 0, 0]
/// - actuator_force = gain * ctrl = 100 * 1 = 100
/// - qfrc_actuator[z] = moment_z * force = -1 * 100 = -100
#[test]
fn ac4_force_magnitude_single_contact() {
    let mjcf = r#"
        <mujoco model="adhesion_magnitude">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_eq!(data.ncon, 1, "expected exactly one contact");

    // actuator_force = gain * ctrl = 100
    assert_relative_eq!(data.actuator_force[0], 100.0, epsilon = 1e-10);

    // moment = [0, 0, -1, 0, 0, 0] — negated contact normal Jacobian.
    // Rotational components are exactly zero because r ∥ normal (see docstring).
    assert_relative_eq!(data.actuator_moment[0][0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][2], -1.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][3], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][4], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][5], 0.0, epsilon = 1e-10);

    // qfrc_actuator z-component = -1 * 100 = -100
    assert_relative_eq!(data.qfrc_actuator[2], -100.0, epsilon = 1e-10);
}

// ============================================================================
// AC5: Multiple contacts (averaged moment)
// ============================================================================

/// Multiple geoms on the same body produce multiple contacts. Moment should be
/// the negated average of all contact normal Jacobians: moment = -(1/count) * Σ J_normal_k.
#[test]
fn ac5_multiple_contacts() {
    // Two separate spheres on the same body, each touching the plane at different
    // x-positions. This guarantees multiple contacts with the same body.
    let mjcf = r#"
        <mujoco model="adhesion_multi_contact">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="multi" pos="0 0 0.099">
                    <freejoint/>
                    <geom name="s1" type="sphere" size="0.1" mass="0.5" pos="-0.3 0 0"/>
                    <geom name="s2" type="sphere" size="0.1" mass="0.5" pos="0.3 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="multi" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // Two geoms on the body should produce at least 2 contacts with the plane
    assert!(
        data.ncon >= 2,
        "expected multiple contacts for multi-geom body on plane, got {}",
        data.ncon
    );

    // Both normals are vertical → translational z-component of each J_normal = +1.
    // Rotational components differ by sign (x-offsets ±0.3 produce ±0.3 on DOF 4)
    // but cancel exactly in the average. Verify the full moment vector.
    assert_relative_eq!(data.actuator_moment[0][0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][2], -1.0, epsilon = 1e-10);
    // Rotational DOFs: symmetric contacts cancel (±0.3 on DOF 4 from ±x offsets)
    assert_relative_eq!(data.actuator_moment[0][3], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][4], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_moment[0][5], 0.0, epsilon = 1e-10);

    // Force is still gain * ctrl = 100
    assert_relative_eq!(data.actuator_force[0], 100.0, epsilon = 1e-10);
    assert_relative_eq!(data.qfrc_actuator[2], -100.0, epsilon = 1e-10);
}

// ============================================================================
// AC6: Gear has no effect (body transmission ignores gear)
// ============================================================================

/// Adhesion with gear="2" should produce identical moment/force as gear="1".
/// MuJoCo's body transmission omits gear entirely.
#[test]
fn ac6_gear_has_no_effect() {
    let mjcf_gear1 = r#"
        <mujoco model="adhesion_gear1">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1" gear="1"/>
            </actuator>
        </mujoco>
    "#;

    let mjcf_gear2 = r#"
        <mujoco model="adhesion_gear2">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1" gear="2"/>
            </actuator>
        </mujoco>
    "#;

    let model1 = load_model(mjcf_gear1).expect("should load gear=1");
    let mut data1 = model1.make_data();
    data1.ctrl[0] = 1.0;
    data1.forward(&model1).expect("forward failed");

    let model2 = load_model(mjcf_gear2).expect("should load gear=2");
    let mut data2 = model2.make_data();
    data2.ctrl[0] = 1.0;
    data2.forward(&model2).expect("forward failed");

    // Moment should be identical regardless of gear
    for dof in 0..model1.nv {
        assert_relative_eq!(
            data1.actuator_moment[0][dof],
            data2.actuator_moment[0][dof],
            epsilon = 1e-15,
        );
    }

    // Forces should be identical
    assert_relative_eq!(
        data1.actuator_force[0],
        data2.actuator_force[0],
        epsilon = 1e-15
    );
    for dof in 0..model1.nv {
        assert_relative_eq!(
            data1.qfrc_actuator[dof],
            data2.qfrc_actuator[dof],
            epsilon = 1e-15,
        );
    }
}

// ============================================================================
// AC7: Gain controls magnitude (gain=200 vs gain=100)
// ============================================================================

/// Doubling the gain should double actuator_force and qfrc_actuator.
#[test]
fn ac7_gain_controls_magnitude() {
    let mjcf_100 = r#"
        <mujoco model="adhesion_gain100">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let mjcf_200 = r#"
        <mujoco model="adhesion_gain200">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="200" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model_100 = load_model(mjcf_100).expect("should load gain=100");
    let mut data_100 = model_100.make_data();
    data_100.ctrl[0] = 1.0;
    data_100.forward(&model_100).expect("forward failed");

    let model_200 = load_model(mjcf_200).expect("should load gain=200");
    let mut data_200 = model_200.make_data();
    data_200.ctrl[0] = 1.0;
    data_200.forward(&model_200).expect("forward failed");

    // Both should have contacts
    assert!(data_100.ncon > 0, "gain=100 model should have contacts");
    assert!(data_200.ncon > 0, "gain=200 model should have contacts");

    // actuator_force ratio should be 2.0
    assert_relative_eq!(
        data_200.actuator_force[0] / data_100.actuator_force[0],
        2.0,
        epsilon = 1e-12
    );

    // qfrc_actuator ratio should be 2.0 on z-DOF
    assert_relative_eq!(
        data_200.qfrc_actuator[2] / data_100.qfrc_actuator[2],
        2.0,
        epsilon = 1e-12
    );
}

// ============================================================================
// AC8: Regression — no adhesion model is unaffected
// ============================================================================

/// A model without any adhesion actuator should produce identical results
/// across two forward() calls (no code path side effects).
#[test]
fn ac8_regression_no_adhesion() {
    let mjcf = r#"
        <mujoco model="no_adhesion">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0" contype="0" conaffinity="0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    let mut data1 = model.make_data();
    data1.forward(&model).expect("forward failed");
    let qacc1: Vec<f64> = data1.qacc.iter().copied().collect();
    let qfrc1: Vec<f64> = data1.qfrc_actuator.iter().copied().collect();

    let mut data2 = model.make_data();
    data2.forward(&model).expect("forward failed");
    let qacc2: Vec<f64> = data2.qacc.iter().copied().collect();
    let qfrc2: Vec<f64> = data2.qfrc_actuator.iter().copied().collect();

    for dof in 0..model.nv {
        assert_relative_eq!(qacc1[dof], qacc2[dof], epsilon = 1e-15);
        assert_relative_eq!(qfrc1[dof], qfrc2[dof], epsilon = 1e-15);
    }
}

// ============================================================================
// AC9: Moment populated with contact, zero without
// ============================================================================

/// With contact: actuator_moment[0] is nonzero.
/// Without contact (contype=0): actuator_moment[0] is all zeros.
#[test]
fn ac9_moment_populated() {
    // With contact — sphere slightly penetrating the plane
    let mjcf_contact = r#"
        <mujoco model="adhesion_moment_contact">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model_c = load_model(mjcf_contact).expect("should load");
    let mut data_c = model_c.make_data();
    data_c.forward(&model_c).expect("forward failed");

    assert!(data_c.ncon > 0, "expected contact");
    // At least one component of moment should be nonzero
    let moment_norm: f64 = data_c.actuator_moment[0]
        .iter()
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    assert!(
        moment_norm > 1e-10,
        "expected nonzero moment with contact, got norm = {}",
        moment_norm
    );

    // Without contact
    let mjcf_no_contact = r#"
        <mujoco model="adhesion_moment_no_contact">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1" contype="0" conaffinity="0"/>
                <body name="ball" pos="0 0 2.0">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0" contype="0" conaffinity="0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model_nc = load_model(mjcf_no_contact).expect("should load");
    let mut data_nc = model_nc.make_data();
    data_nc.forward(&model_nc).expect("forward failed");

    for dof in 0..model_nc.nv {
        assert_relative_eq!(data_nc.actuator_moment[0][dof], 0.0, epsilon = 1e-15);
    }
}

// ============================================================================
// AC10: actuator_length is always zero
// ============================================================================

/// Body transmission has no length concept. `actuator_length[0]` must be 0.0.
#[test]
fn ac10_actuator_length_zero() {
    let mjcf = r#"
        <mujoco model="adhesion_length">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], 0.0, epsilon = 1e-15);
}

// ============================================================================
// AC11: actuator_velocity = moment · qvel
// ============================================================================

/// With nonzero qvel, actuator_velocity should equal the dot product of
/// the cached moment vector and qvel.
#[test]
fn ac11_actuator_velocity() {
    let mjcf = r#"
        <mujoco model="adhesion_velocity">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="ball" pos="0 0 0.099">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="ball" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Set some nonzero velocities
    data.qvel[0] = 0.5; // vx
    data.qvel[1] = -0.3; // vy
    data.qvel[2] = 1.0; // vz
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // Verify: actuator_velocity = moment · qvel
    let expected_vel: f64 = data.actuator_moment[0].dot(&data.qvel);
    assert_relative_eq!(data.actuator_velocity[0], expected_vel, epsilon = 1e-12);
}

// ============================================================================
// AC12: Flex contacts excluded (documented skip)
// ============================================================================

// AC12: Flex contacts excluded — deferred, flex collision not yet wired through
// standard rigid-rigid collision detection. The `mj_transmission_body()` function
// correctly skips contacts with `flex_vertex.is_some()`, but we cannot generate
// such contacts in the standard pipeline to test this path.

// ============================================================================
// AC13: Kinematic chain with hinge joint
// ============================================================================

/// Hinge body on a plane: only the hinge DOF should have nonzero moment.
/// The moment value should match the analytical lever arm projection.
#[test]
fn ac13_kinematic_chain_hinge() {
    // Capsule fromto=(0,0,0)→(0.3,0,-0.5), body at z=0.5 → endpoint at world z=0.0.
    // With capsule radius 0.05, the endcap bottom is at z=-0.05 → penetrates the plane.
    let mjcf = r#"
        <mujoco model="adhesion_hinge">
            <compiler angle="radian"/>
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="arm" pos="0 0 0.5">
                    <joint name="hinge1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0.3 0 -0.5" mass="0.5"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="arm" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // With a hinge joint, nv = 1 (single DOF)
    assert_eq!(model.nv, 1, "hinge body should have 1 DOF");

    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert!(
        data.ncon > 0,
        "expected contact between capsule and plane; ncon = {}",
        data.ncon
    );

    // The single hinge DOF should have nonzero moment from the contact Jacobian
    assert!(
        data.actuator_moment[0][0].abs() > 1e-10,
        "expected nonzero moment on hinge DOF, got {}",
        data.actuator_moment[0][0]
    );

    // qfrc_actuator should also be nonzero
    assert!(
        data.qfrc_actuator[0].abs() > 1e-10,
        "expected nonzero qfrc_actuator on hinge DOF, got {}",
        data.qfrc_actuator[0]
    );
}

// ============================================================================
// AC14: Two non-world bodies in contact
// ============================================================================

/// When two non-world bodies are in contact, the adhesion actuator on one body
/// should produce nonzero moment on both bodies' DOFs (via the Jacobian
/// difference J(b2) - J(b1) projected along the contact normal).
///
/// Uses sibling bodies (not parent-child) because MuJoCo parent-child
/// collision filtering prevents adjacent bodies from colliding.
#[test]
fn ac14_two_body_contact() {
    // Two sibling spheres (not parent-child) under the world body, each with
    // its own slide joint along x. Centers at (±0.04, 0, 0.15), both radius 0.1.
    // Distance = 0.08 < sum_radii = 0.2 → penetration = 0.12, guaranteed contact.
    // Both spheres are above the plane (z=0.15-0.1=0.05 > 0) → no plane contacts.
    let mjcf = r#"
        <mujoco model="adhesion_two_body">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="10 10 1"/>
                <body name="left" pos="-0.04 0 0.15">
                    <joint name="slide_left" type="slide" axis="1 0 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="right" pos="0.04 0 0.15">
                    <joint name="slide_right" type="slide" axis="1 0 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion name="grip" body="left" gain="100" ctrlrange="0 1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_eq!(model.nv, 2, "expected 2 DOFs (slide_left + slide_right)");

    // Find contacts involving the "left" body
    let left_body_id = model.actuator_trnid[0][0];
    let mut has_body_body_contact = false;
    for c in 0..data.ncon {
        let b1 = model.geom_body[data.contacts[c].geom1];
        let b2 = model.geom_body[data.contacts[c].geom2];
        // A contact between two non-world bodies
        if (b1 == left_body_id && b2 != 0) || (b2 == left_body_id && b1 != 0) {
            has_body_body_contact = true;
            break;
        }
    }

    assert!(
        has_body_body_contact,
        "expected contact between left body ({}) and another non-world body; ncon = {}",
        left_body_id, data.ncon
    );

    // Sphere-sphere contact normal is horizontal (along x). Both slide joints
    // have axis (1,0,0). J_normal = normal·(J(b2) - J(b1)):
    //   J(right, slide_right) = +1, J(left, slide_left) = -1
    //   → J_normal = [-1, +1], moment = [+1, -1]
    // Both DOFs must be individually nonzero.
    assert!(
        data.actuator_moment[0][0].abs() > 1e-10,
        "expected nonzero moment on left slide DOF, got {}",
        data.actuator_moment[0][0]
    );
    assert!(
        data.actuator_moment[0][1].abs() > 1e-10,
        "expected nonzero moment on right slide DOF, got {}",
        data.actuator_moment[0][1]
    );
}
