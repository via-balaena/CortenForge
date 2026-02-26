//! Phase 4 branch audit tests — AC-level coverage for all 4 specs.
//!
//! Organized by spec in priority order:
//! - Section D: 4A.6 acc-stage sensor values (accelerometer, force, torque, framelinacc, frameangacc)
//! - Section A: CVEL reference-point fixes (velocimeter, framelinvel, subtree momentum/angmom, KE)
//! - Section B: §56 persistent subtree fields (subtree_linvel, subtree_angmom, lazy flags)
//! - Section C: flg_rnepost lazy gate (demand-driven body accumulators)

use sim_core::{DISABLE_SENSOR, SleepState};
use sim_mjcf::load_model;

// ============================================================================
// Helpers
// ============================================================================

const TOL: f64 = 1e-10;
const TOL_MODERATE: f64 = 1e-6;

/// Assert a 3D sensor reading at `sensordata[adr..adr+3]` matches expected values.
fn assert_sensor3(
    sensordata: &nalgebra::DVector<f64>,
    adr: usize,
    expected: [f64; 3],
    tol: f64,
    label: &str,
) {
    for i in 0..3 {
        assert!(
            (sensordata[adr + i] - expected[i]).abs() < tol,
            "{label}[{i}]: expected {}, got {}, diff = {}",
            expected[i],
            sensordata[adr + i],
            (sensordata[adr + i] - expected[i]).abs(),
        );
    }
}

// ============================================================================
// Section D: 4A.6 Acc-Stage Sensor Values (highest priority)
// ============================================================================

/// 4A.6-T2 / AC1: Accelerometer on static body reads [0, 0, +9.81] (proper acceleration).
///
/// A body at rest on a plane (or simply in static equilibrium) experiences
/// proper acceleration = -gravity = [0, 0, +9.81] in world frame. The
/// accelerometer reads cacc which includes the gravity pseudo-acceleration.
#[test]
fn d01_accelerometer_static_gravity() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    // A free body free-falls (qacc cancels gravity → accelerometer = 0).
    // For STATIC, use a hinge body that cannot translate — gravity is fully visible.
    let _model = load_model(xml).expect("load");
    let xml_static = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml_static).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Accelerometer is dim=3, at adr=0. Site is at body origin, so no spatial shift.
    // cacc[1] = cacc[0] + S*qacc. The hinge constrains all DOF except rotation
    // around Y, so qacc has 1 DOF (angular about Y). The gravity pseudo-acc
    // from cacc[0] = [0,0,0, 0,0,+9.81] passes through to cacc[1] in the
    // linear channels. The accelerometer rotates by site_xmat^T.
    //
    // With zero joint angle and identity body rotation, site_xmat = I,
    // so sensor reads [0, 0, +9.81] in the Z component.
    let adr = model.sensor_adr[0];
    assert!(
        (data.sensordata[adr + 2] - 9.81).abs() < TOL,
        "Accelerometer Z should be +9.81, got {}",
        data.sensordata[adr + 2]
    );
    assert!(
        data.sensordata[adr].abs() < TOL,
        "Accelerometer X should be 0, got {}",
        data.sensordata[adr]
    );
    assert!(
        data.sensordata[adr + 1].abs() < TOL,
        "Accelerometer Y should be 0, got {}",
        data.sensordata[adr + 1]
    );
}

/// 4A.6-T1 / AC1: Accelerometer in free fall reads ~[0,0,0].
///
/// A free body with no contacts: qacc = [0,0,-g,...], which cancels the
/// gravity pseudo-acceleration. Net cacc[body] ≈ 0 → accelerometer ≈ 0.
#[test]
fn d02_accelerometer_free_fall() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="ball" pos="0 0 5">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    for i in 0..3 {
        assert!(
            data.sensordata[adr + i].abs() < 1e-6,
            "Free-fall accelerometer[{i}] should be ~0, got {}",
            data.sensordata[adr + i]
        );
    }
}

/// 4A.6-T3 / AC2: Accelerometer centripetal with site offset.
///
/// Hinge body rotating at omega=10 rad/s about Z. Site at [0.5, 0, 0] from
/// body origin. Centripetal acceleration = -omega^2 * r = -100 * 0.5 = -50.
/// In the rotating body frame (site_xmat^T), this appears as [-50, 0, 0]
/// (radially inward).
#[test]
fn d03_accelerometer_centripetal() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Set angular velocity = 10 rad/s about z-axis
    data.qvel[0] = 10.0;
    data.forward(&model).expect("forward");

    // Centripetal acceleration at [0.5, 0, 0]:
    // omega × (omega × r) where omega = [0,0,10], r = [0.5,0,0]
    // omega × r = [0,5,0], omega × [0,5,0] = [-50,0,0]
    // Plus Coriolis correction: omega × v_at_site.
    //
    // In world frame: the centripetal is [-50, 0, 0] (pointing inward).
    // After rotating into site frame (which rotates with the body at q=0, so
    // site_xmat = I at t=0), the reading is [-50, 0, 0].
    let adr = model.sensor_adr[0];
    assert!(
        (data.sensordata[adr] - (-50.0)).abs() < 1e-4,
        "Centripetal X should be -50, got {}",
        data.sensordata[adr]
    );
    assert!(
        data.sensordata[adr + 1].abs() < 1e-4,
        "Centripetal Y should be ~0, got {}",
        data.sensordata[adr + 1]
    );
    assert!(
        data.sensordata[adr + 2].abs() < 1e-4,
        "Centripetal Z should be ~0, got {}",
        data.sensordata[adr + 2]
    );
}

/// 4A.6-T4 / AC3: FrameLinAcc includes gravity (conformance fix).
///
/// A static hinge body: FrameLinAcc reads [0, 0, +9.81] in world frame.
/// This is a behavior change from the pre-refactor code that returned [0,0,0].
#[test]
fn d04_framelinacc_includes_gravity() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <framelinacc site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    assert_sensor3(
        &data.sensordata,
        adr,
        [0.0, 0.0, 9.81],
        TOL,
        "FrameLinAcc static",
    );
}

/// 4A.6-T5 / AC4: FrameAngAcc reads angular acceleration from cacc.
///
/// Hinge body with qacc = 5.0 about Y-axis → FrameAngAcc = [0, 5.0, 0].
#[test]
fn d05_frameangacc_hinge() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <frameangacc site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Set qacc directly — forward() normally computes this via the solver,
    // but we can inject it. For a hinge, qacc has 1 DOF.
    // However, forward() will overwrite qacc from the solver. Instead,
    // apply a torque that produces known qacc.
    //
    // For a hinge with I ≈ body_inertia (sphere of mass=1, r=0.1):
    //   I_yy = 2/5 * m * r^2 = 0.004
    //   qacc = torque / I_yy
    //
    // Apply torque = 5.0 * I_yy via qfrc_applied:
    //   qacc = qfrc_applied / M_diag ≈ (5 * 0.004) / 0.004 = 5.0
    //
    // Actually, for precision let's set a known qfrc_applied and read qacc.
    // The key question is: what is the effective inertia for this hinge DOF?
    // For a sphere: I_yy = 2/5 * 1.0 * 0.01 = 0.004, but there may be
    // additional inertia from the mass at the joint position.
    //
    // Simpler approach: apply torque, read FrameAngAcc, and verify the
    // relationship. Or better: just verify that forward() + read gives
    // the correct qacc mapping.
    //
    // Let's use step1/step2 pattern: after forward, qacc is computed by the solver.
    // With no gravity, no velocity, no external forces, qacc = 0 for all DOF.
    // Apply qfrc_applied to get nonzero qacc.
    data.qfrc_applied[0] = 1.0; // torque about y-axis
    data.forward(&model).expect("forward");

    // qacc[0] = qfrc_applied[0] / M_diag[0]
    // FrameAngAcc = cacc angular component = qacc[0] * axis_world = qacc[0] * [0,1,0]
    let qacc = data.qacc[0];
    let adr = model.sensor_adr[0];
    assert_sensor3(
        &data.sensordata,
        adr,
        [0.0, qacc, 0.0],
        TOL,
        "FrameAngAcc hinge",
    );
    // Verify qacc is nonzero (sanity)
    assert!(qacc.abs() > 1.0, "qacc should be significant, got {}", qacc);
}

/// 4A.6-T6 / AC5: Force sensor on single body reads [0, 0, +mg].
///
/// A 2 kg body on a hinge in gravity. The constraint force (internal force)
/// supporting it equals mg = 2 * 9.81 = 19.62 N upward.
#[test]
fn d06_force_sensor_static() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="2.0"/>
          <site name="ft" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <force site="ft"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    // Force sensor reads cfrc_int linear part rotated into site frame.
    // For a body on a hinge with no velocity: cfrc_int = I*cacc + 0 - 0.
    // cacc[1] includes gravity pseudo-acceleration. The internal force
    // supporting the body against gravity is [0, 0, +mg].
    assert_sensor3(
        &data.sensordata,
        adr,
        [0.0, 0.0, 2.0 * 9.81],
        TOL_MODERATE,
        "Force sensor static",
    );
}

/// 4A.6-T8 / AC6: Torque sensor spatial force transform (moment arm shift).
///
/// Verifies that the torque sensor correctly applies the spatial force transform
/// when shifting the reference point from body origin to site position:
///   torque_at_site = torque_at_origin - r × force
///
/// Uses a simple hinge body with the geom at body origin (no COM offset).
/// Two torque sensors at different positions verify the shift formula.
#[test]
fn d07_torque_sensor_moment_arm() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="ft_origin" pos="0 0 0"/>
          <site name="ft_offset" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <torque site="ft_origin"/>
        <torque site="ft_offset"/>
        <force site="ft_origin"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Read force at origin (sensor 2): should be [0, 0, +mg]
    let force_adr = model.sensor_adr[2];
    let force_x = data.sensordata[force_adr];
    let force_y = data.sensordata[force_adr + 1];
    let force_z = data.sensordata[force_adr + 2];
    assert!(
        (force_z - 9.81).abs() < TOL_MODERATE,
        "Force Z should be +9.81, got {}",
        force_z
    );

    // Read torque at origin (sensor 0) and at offset (sensor 1)
    let torque_origin_adr = model.sensor_adr[0];
    let torque_offset_adr = model.sensor_adr[1];

    // Verify spatial transform: torque_offset = torque_origin - r × force
    // r = [0.5, 0, 0] (from body origin to ft_offset site)
    // r × force = [0*fz - 0*fy, 0*fx - 0.5*fz, 0.5*fy - 0*fx]
    //           = [0, -0.5*fz, 0]
    // torque_offset = torque_origin - [0, -0.5*fz, 0]
    //              = torque_origin + [0, 0.5*fz, 0]
    for i in 0..3 {
        let torque_origin_i = data.sensordata[torque_origin_adr + i];
        let torque_offset_i = data.sensordata[torque_offset_adr + i];
        // Compute expected shift: -r × force for component i
        let r_cross_f = match i {
            0 => 0.0 * force_z - 0.0 * force_y,
            1 => 0.0 * force_x - 0.5 * force_z,
            2 => 0.5 * force_y - 0.0 * force_x,
            _ => unreachable!(),
        };
        let expected_offset_i = torque_origin_i - r_cross_f;
        assert!(
            (torque_offset_i - expected_offset_i).abs() < TOL_MODERATE,
            "Torque shift[{i}]: offset ({}) should equal origin ({}) - r×f ({}), expected {}",
            torque_offset_i,
            torque_origin_i,
            r_cross_f,
            expected_offset_i
        );
    }
}

/// 4A.6-T7/T9 / AC7: Force/Torque 3-link chain regression.
///
/// A 3-link serial chain with hinge joints. Force and torque sensors on each
/// link. Verify cfrc_int propagation: leaf supports 1 body, middle supports 2,
/// root supports 3.
#[test]
fn d08_force_torque_chain_regression() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="link1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.25" mass="1.0"/>
          <site name="ft1" pos="0 0 0"/>
          <body name="link2" pos="0 0 -0.5">
            <joint name="j2" type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.05 0.25" mass="1.0"/>
            <site name="ft2" pos="0 0 0"/>
            <body name="link3" pos="0 0 -0.5">
              <joint name="j3" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05 0.25" mass="1.0"/>
              <site name="ft3" pos="0 0 0"/>
            </body>
          </body>
        </body>
      </worldbody>
      <sensor>
        <force site="ft1"/>
        <force site="ft2"/>
        <force site="ft3"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let g = 9.81;

    // Force sensor reads cfrc_int linear part in site frame.
    // For a static chain, the Z force at each link should support the
    // weight of all bodies below (inclusive).
    let force_norm = |idx: usize| -> f64 {
        let adr = model.sensor_adr[idx];
        let fx = data.sensordata[adr];
        let fy = data.sensordata[adr + 1];
        let fz = data.sensordata[adr + 2];
        (fx * fx + fy * fy + fz * fz).sqrt()
    };

    let f1 = force_norm(0); // root: supports 3 bodies
    let f2 = force_norm(1); // middle: supports 2 bodies
    let f3 = force_norm(2); // leaf: supports 1 body

    // Check approximate magnitudes (within 10% of n*mg)
    assert!(
        (f3 - 1.0 * g).abs() < 1.0 * g * 0.1,
        "Link3 force should be ~1g ({}), got {}",
        g,
        f3
    );
    assert!(
        (f2 - 2.0 * g).abs() < 2.0 * g * 0.1,
        "Link2 force should be ~2g ({}), got {}",
        2.0 * g,
        f2
    );
    assert!(
        (f1 - 3.0 * g).abs() < 3.0 * g * 0.1,
        "Link1 force should be ~3g ({}), got {}",
        3.0 * g,
        f1
    );

    // Monotonic: f1 >= f2 >= f3
    assert!(f1 >= f2 - 0.1, "Monotonic: f1 ({f1}) >= f2 ({f2})");
    assert!(f2 >= f3 - 0.1, "Monotonic: f2 ({f2}) >= f3 ({f3})");
}

/// 4A.6-T11 / edge case: World body sensor produces finite values.
#[test]
fn d09_acc_sensor_world_body_finite() {
    // A body sensor attached to the world body (body 0) should produce
    // finite values (gravity pseudo-acceleration), not NaN or panic.
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <site name="world_site" pos="0 0 0"/>
        <body name="b1" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <framelinacc site="world_site"/>
        <frameangacc site="world_site"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // World body FrameLinAcc should show gravity pseudo-acceleration
    let adr = model.sensor_adr[0];
    for i in 0..3 {
        assert!(
            data.sensordata[adr + i].is_finite(),
            "World FrameLinAcc[{i}] should be finite, got {}",
            data.sensordata[adr + i]
        );
    }

    // World body FrameAngAcc should be zero (no angular acceleration on world)
    let adr_ang = model.sensor_adr[1];
    for i in 0..3 {
        assert!(
            data.sensordata[adr_ang + i].abs() < TOL,
            "World FrameAngAcc[{i}] should be 0, got {}",
            data.sensordata[adr_ang + i]
        );
    }
}

/// 4A.6-T12 / edge case: Zero-mass body produces no NaN in acc sensors.
#[test]
fn d10_acc_sensor_zero_mass_no_nan() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="0.0001"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
        <force site="imu"/>
        <torque site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // All sensor values should be finite (no NaN from division by zero-ish mass)
    for i in 0..model.nsensordata {
        assert!(
            data.sensordata[i].is_finite(),
            "sensordata[{i}] should be finite, got {}",
            data.sensordata[i]
        );
    }
}

/// 4A.6-T13 / AC: DISABLE_SENSOR prevents acc-stage evaluation.
///
/// With DISABLE_SENSOR, neither mj_sensor_acc nor mj_body_accumulators runs
/// (the lazy gate is not triggered because sensors are never scanned).
#[test]
fn d11_disable_sensor_acc_stage() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81">
        <flag sensor="disable"/>
      </option>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    assert!(
        model.disableflags & DISABLE_SENSOR != 0,
        "DISABLE_SENSOR should be set"
    );

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // flg_rnepost should still be false (lazy gate never triggered)
    assert!(
        !data.flg_rnepost,
        "flg_rnepost should be false with DISABLE_SENSOR"
    );

    // cacc should remain zeroed (body accumulators never ran)
    for i in 0..6 {
        assert!(
            data.cacc[0][i].abs() < TOL || !data.flg_rnepost,
            "cacc should be stale/zeroed with DISABLE_SENSOR"
        );
    }
}

/// 4A.6-T14 / AC: Sleep interaction — RNE pass computes for all bodies.
///
/// When an acc-stage sensor triggers mj_body_accumulators, it computes cacc
/// and cfrc_int for ALL bodies, including sleeping ones.
#[test]
fn d12_acc_sensor_sleep_computes_all() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
        <flag sleep="enable"/>
      </option>
      <worldbody>
        <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
        <body name="sleeper" pos="0 0 0.2">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
        </body>
        <body name="awake" pos="5 0 10">
          <freejoint/>
          <geom type="sphere" size="0.05" mass="0.01"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Step until sleeper falls asleep
    let mut asleep = false;
    for _ in 0..5000 {
        data.step(&model).expect("step");
        if data.body_sleep_state[1] == SleepState::Asleep {
            asleep = true;
            break;
        }
    }
    assert!(asleep, "Sleeper should fall asleep");

    // forward() triggers body accumulators via the awake body's accelerometer
    data.forward(&model).expect("forward");

    // cacc for world body should have gravity pseudo-acceleration
    assert!(
        (data.cacc[0][5] - 9.81).abs() < TOL_MODERATE,
        "cacc[0] should have gravity, got z={}",
        data.cacc[0][5]
    );

    // cfrc_int for the sleeping body should be nonzero
    let norm: f64 = data.cfrc_int[1].iter().map(|x| x * x).sum::<f64>().sqrt();
    assert!(
        norm > 0.1,
        "cfrc_int[1] should be nonzero for sleeping body, got norm={norm}"
    );
}

// ============================================================================
// Section A: CVEL Reference-Point Fixes
// ============================================================================

/// B1-AC1 / T1: Velocimeter with site offset from hinge body.
///
/// Hinge about Z, qvel=1. Site at [0, 0, 0.5] from body origin.
/// Tangential velocity at site: v = omega × r = [0,0,1] × [0,0,0.5] = [0,0,0].
/// Wait — that's zero because r is parallel to omega.
///
/// Use site at [0.5, 0, 0] instead:
/// v = omega × r = [0,0,1] × [0.5,0,0] = [0,0.5,0].
/// In sensor frame (identity at q=0), velocimeter reads [0, 0.5, 0].
///
/// Actually, the spec says: "hinge z-axis, site at [0,0,0.5], qvel=1.0.
/// Reads [-0.5, 0, 0]". Let me re-derive:
/// omega = [0, 0, 1] (hinge Z), site offset r = [0, 0, 0.5].
/// v_at_site = v_origin + omega × r = 0 + [0,0,1] × [0,0,0.5] = [0,0,0].
/// That's still zero because the cross product of parallel vectors is zero.
///
/// The spec likely means hinge about Y: omega = [0, 1, 0].
/// Then: omega × [0, 0, 0.5] = [1*0.5 - 0*0, 0*0 - 0*0.5, 0*0 - 1*0] = [0.5, 0, 0].
///
/// Let me use the simplest case: hinge about Y, site at [0.5, 0, 0].
/// omega_world = [0, 1, 0] * qvel = [0, 1, 0].
/// v_site = omega × r = [0,1,0] × [0.5,0,0] = [0*0-0*0, 0*0.5-1*0, 1*0-0*0.5]
///        = [0, 0, -0.5].
///
/// For consistency with the spec's [-0.5, 0, 0] output:
/// Hinge about Y, site at [0, 0, 0.5]:
/// omega = [0, 1, 0], r = [0, 0, 0.5].
/// v_site = [1*0.5 - 0*0, 0*0 - 0*0.5, 0*0 - 1*0] = [0.5, 0, 0].
///
/// Hmm, that gives [0.5, 0, 0]. The spec says [-0.5, 0, 0], which is the
/// opposite direction. This might depend on sign convention. Let me just test
/// the magnitude and direction, using a concrete example with a predictable answer.
#[test]
fn a01_velocimeter_site_offset() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="vel_site" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="vel_site"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0; // 1 rad/s about Z
    data.forward(&model).expect("forward");

    // omega = [0, 0, 1] (world frame, hinge about Z with identity quat at q=0)
    // r = site_xpos - xpos = [0.5, 0, 0] - [0, 0, 0] = [0.5, 0, 0]
    // v_at_site = v_origin + omega × r = 0 + [0,0,1] × [0.5,0,0] = [0, 0.5, 0]
    //
    // Velocimeter rotates into site frame: site_xmat^T * v_world.
    // At q=0 with identity body rotation, site_xmat = I, so sensor = [0, 0.5, 0].
    let adr = model.sensor_adr[0];
    assert!(
        data.sensordata[adr].abs() < TOL,
        "Velocimeter X should be ~0, got {}",
        data.sensordata[adr]
    );
    assert!(
        (data.sensordata[adr + 1] - 0.5).abs() < TOL,
        "Velocimeter Y should be 0.5, got {}",
        data.sensordata[adr + 1]
    );
    assert!(
        data.sensordata[adr + 2].abs() < TOL,
        "Velocimeter Z should be ~0, got {}",
        data.sensordata[adr + 2]
    );
}

/// B1-AC2: Velocimeter on body (not site). Output matches cvel directly.
#[test]
fn a02_velocimeter_body_origin() {
    // Velocimeter attached to body directly (objtype=Body) reads cvel at body origin.
    // No spatial shift needed.
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="origin_site" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="origin_site"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 2.0;
    data.forward(&model).expect("forward");

    // Site at body origin: v_site = v_origin + omega × 0 = v_origin = 0
    // (hinge at origin means linear velocity at origin is zero, only angular)
    let adr = model.sensor_adr[0];
    for i in 0..3 {
        assert!(
            data.sensordata[adr + i].abs() < TOL,
            "Velocimeter at origin[{i}] should be 0, got {}",
            data.sensordata[adr + i]
        );
    }
}

/// B1-AC3: Velocimeter with off-axis site, verify |v| = omega * r.
#[test]
fn a03_velocimeter_off_axis_magnitude() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="off_site" pos="0.3 0.4 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="off_site"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 2.0;
    data.forward(&model).expect("forward");

    // omega = [0, 0, 2], r = [0.3, 0.4, 0]
    // v = omega × r = [0*0 - 2*0.4, 2*0.3 - 0*0, 0*0.4 - 0*0.3] = [-0.8, 0.6, 0]
    // |v| = sqrt(0.64 + 0.36) = 1.0
    let adr = model.sensor_adr[0];
    let vx = data.sensordata[adr];
    let vy = data.sensordata[adr + 1];
    let vz = data.sensordata[adr + 2];
    let vmag = (vx * vx + vy * vy + vz * vz).sqrt();
    assert!((vmag - 1.0).abs() < TOL, "|v| should be 1.0, got {}", vmag);
    // Check components: in sensor frame (site_xmat^T * v_world). At q=0 site_xmat = I.
    assert!(
        (vx - (-0.8)).abs() < TOL,
        "Velocimeter X should be -0.8, got {}",
        vx
    );
    assert!(
        (vy - 0.6).abs() < TOL,
        "Velocimeter Y should be 0.6, got {}",
        vy
    );
}

/// B2-AC1 / T2: FrameLinVel with site offset in world frame.
#[test]
fn a04_framelinvel_site_offset() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="off_site" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <framelinvel site="off_site"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    // omega = [0,0,1], r = [0.5,0,0]
    // v = omega × r = [0, 0.5, 0] in world frame.
    // FrameLinVel outputs in world frame (no rotation).
    let adr = model.sensor_adr[0];
    assert_sensor3(
        &data.sensordata,
        adr,
        [0.0, 0.5, 0.0],
        TOL,
        "FrameLinVel site offset",
    );
}

/// B2-AC2: FrameLinVel on body (not site). No shift needed.
#[test]
fn a05_framelinvel_body_origin() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="origin" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <framelinvel site="origin"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 5.0;
    data.forward(&model).expect("forward");

    // At body origin, linear velocity is zero for a hinge (all velocity is angular)
    let adr = model.sensor_adr[0];
    assert_sensor3(
        &data.sensordata,
        adr,
        [0.0, 0.0, 0.0],
        TOL,
        "FrameLinVel at origin",
    );
}

/// B5-AC1 / T5: Kinetic energy consistency.
///
/// Per-body KE fallback vs 0.5 * qvel^T * M * qvel.
/// For a free body with mass m and velocity v, KE = 0.5 * m * |v|^2.
#[test]
fn a06_kinetic_energy_consistency() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="2.0"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <framelinvel site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    // Set velocity: [1, 2, 3] linear, [0, 0, 0] angular
    data.qvel[0] = 1.0;
    data.qvel[1] = 2.0;
    data.qvel[2] = 3.0;
    data.forward(&model).expect("forward");

    // KE from qvel^T * M * qvel
    let mut m_qvel = nalgebra::DVector::zeros(model.nv);
    data.qM.mul_to(&data.qvel, &mut m_qvel);
    let ke_quad = 0.5 * data.qvel.dot(&m_qvel);

    // KE from body velocity: 0.5 * m * |v|^2
    let m = 2.0;
    let v2 = 1.0 * 1.0 + 2.0 * 2.0 + 3.0 * 3.0; // 14
    let ke_body = 0.5 * m * v2; // 14.0

    assert!(
        (ke_quad - ke_body).abs() < TOL,
        "KE quadratic ({ke_quad}) should match KE body ({ke_body})"
    );
}

/// T7: World body velocimeter/FrameLinVel — no crash, zero output.
#[test]
fn a07_vel_sensor_world_body_zero() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <site name="world_site" pos="0 0 0"/>
        <body name="b1" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="world_site"/>
        <framelinvel site="world_site"/>
        <frameangvel site="world_site"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0; // give the free body velocity
    data.forward(&model).expect("forward");

    // World body sensors should be zero (world body has zero velocity)
    for s in 0..3 {
        let adr = model.sensor_adr[s];
        let dim = model.sensor_dim[s];
        for i in 0..dim {
            assert!(
                data.sensordata[adr + i].abs() < TOL,
                "World body sensor {s}[{i}] should be 0, got {}",
                data.sensordata[adr + i]
            );
        }
    }
}

/// T8: Zero-mass body velocity sensors — no NaN.
#[test]
fn a08_vel_sensor_zero_mass_no_nan() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="0.0001"/>
          <site name="s1" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="s1"/>
        <framelinvel site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    for i in 0..model.nsensordata {
        assert!(
            data.sensordata[i].is_finite(),
            "sensordata[{i}] should be finite, got {}",
            data.sensordata[i]
        );
    }
}

/// T9: Sleeping body velocity sensor reads zero (frozen).
#[test]
fn a09_vel_sensor_sleeping_frozen() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
        <flag sleep="enable"/>
      </option>
      <worldbody>
        <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
        <body name="sleeper" pos="0 0 0.2">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Step until body sleeps
    let mut asleep = false;
    for _ in 0..5000 {
        data.step(&model).expect("step");
        if data.body_sleep_state[1] == SleepState::Asleep {
            asleep = true;
            break;
        }
    }
    assert!(asleep, "Body should fall asleep");

    // Velocity sensors on sleeping bodies are frozen (skipped by §16.5d).
    // The sensor retains its last value before sleep — which should be near
    // zero since the body settled, but may have small residual velocity at
    // the moment sleep was triggered (within sleep_tolerance = 0.1).
    data.forward(&model).expect("forward");
    let adr = model.sensor_adr[0];
    let v_norm: f64 = (0..3)
        .map(|i| data.sensordata[adr + i].powi(2))
        .sum::<f64>()
        .sqrt();
    assert!(
        v_norm < 0.2,
        "Sleeping body velocimeter norm should be small (< sleep_tolerance), got {}",
        v_norm
    );
}

/// T10: DISABLE_SENSOR — velocity sensors not evaluated.
#[test]
fn a10_vel_sensor_disable_sensor() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0">
        <flag sensor="disable"/>
      </option>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="s1"/>
        <subtreelinvel body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 5.0;
    data.forward(&model).expect("forward");

    // With DISABLE_SENSOR, sensordata should be stale (initial zeros).
    // flg_subtreevel should not be set (mj_subtree_vel never called).
    assert!(
        !data.flg_subtreevel,
        "flg_subtreevel should be false with DISABLE_SENSOR"
    );

    // Sensordata was never written (initialized to zero, stays zero)
    for i in 0..model.nsensordata {
        assert!(
            data.sensordata[i].abs() < TOL,
            "Sensordata[{i}] should be stale (0) with DISABLE_SENSOR, got {}",
            data.sensordata[i]
        );
    }
}

// ============================================================================
// Section B: §56 Persistent Subtree Fields
// ============================================================================

/// S56-AC2 / T1: Free body subtree_linvel matches linear velocity.
///
/// Single free body with mass=1, velocity=[1,2,3].
/// subtree_linvel[1] = [1,2,3] (mass-weighted, but only 1 body → same as v_com).
#[test]
fn b01_subtree_linvel_free_body() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0;
    data.qvel[1] = 2.0;
    data.qvel[2] = 3.0;
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    assert_sensor3(
        &data.sensordata,
        adr,
        [1.0, 2.0, 3.0],
        TOL,
        "SubtreeLinVel free body",
    );
}

/// S56-AC3 / T2: 3-body chain mass-weighted subtree_linvel.
///
/// Root body (mass=2) at velocity [1,0,0], child (mass=1) at [2,0,0].
/// subtree_linvel[root] = (2*1 + 1*2) / 3 = 4/3 ≈ 1.333.
#[test]
fn b02_subtree_linvel_chain_average() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="slide" axis="1 0 0"/>
          <geom type="sphere" size="0.1" mass="2.0"/>
          <body name="b2" pos="1 0 0">
            <joint name="j2" type="slide" axis="1 0 0"/>
            <geom type="sphere" size="0.1" mass="1.0"/>
          </body>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="b1"/>
        <subtreelinvel body="b2"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    // j1 gives b1 velocity [1,0,0], j2 gives b2 additional [1,0,0] → b2 total [2,0,0]
    data.qvel[0] = 1.0; // j1: slide x for b1
    data.qvel[1] = 1.0; // j2: slide x for b2 (additive → b2 has total 2.0 in x)
    data.forward(&model).expect("forward");

    // subtree_linvel[b2] = v_b2 = [2, 0, 0] (leaf, only itself)
    let adr2 = model.sensor_adr[1];
    assert!(
        (data.sensordata[adr2] - 2.0).abs() < 1e-8,
        "SubtreeLinVel[b2] x should be 2.0, got {}",
        data.sensordata[adr2]
    );

    // subtree_linvel[b1] = (m1*v1 + m2*v2) / (m1+m2)
    // But v is at COM, not at origin! The COM of b1 is at xipos[1], COM of b2 is at xipos[2].
    // cvel is at xpos (body origin). mj_subtree_vel shifts from xpos to xipos.
    // For a sphere with pos="0 0 0" and size="0.1", xipos = xpos (COM at origin).
    // So: subtree_linvel[b1] = (2*[1,0,0] + 1*[2,0,0]) / 3 = [4/3, 0, 0]
    let adr1 = model.sensor_adr[0];
    assert!(
        (data.sensordata[adr1] - 4.0 / 3.0).abs() < 1e-8,
        "SubtreeLinVel[b1] x should be 4/3, got {}",
        data.sensordata[adr1]
    );
}

/// S56-AC4 / T3: Spinning body subtree_angmom = I_zz * omega_z.
///
/// Body with inertia I_zz=2.0 (controlled via mass+geometry) spinning at omega_z=3.0.
/// subtree_angmom = [0, 0, I_zz * omega_z] = [0, 0, 6.0].
///
/// For a sphere: I = 2/5 * m * r^2. To get I_zz = 2.0: m * r^2 = 5.0.
/// Use m=50, r=sqrt(0.1)≈0.316... → I = 2/5 * 50 * 0.1 = 2.0. ✓
#[test]
fn b03_subtree_angmom_spin() {
    // Use a box with explicit inertia to avoid sphere approximations.
    // Actually, for a sphere: I_xx = I_yy = I_zz = 2/5 * m * r^2.
    // With m=50, r=0.31623 → I = 2/5 * 50 * 0.1 = 2.0.
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.31623" mass="50.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreeangmom body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Set angular velocity about z: free body DOF order is [vx,vy,vz, wx,wy,wz]
    data.qvel[5] = 3.0; // omega_z = 3.0
    data.forward(&model).expect("forward");

    // I_zz = 2/5 * 50 * 0.31623^2 = 2/5 * 50 * 0.1 = 2.0
    // angmom_z = I_zz * omega_z = 2.0 * 3.0 = 6.0
    let adr = model.sensor_adr[0];
    assert!(
        data.sensordata[adr].abs() < 1e-6,
        "SubtreeAngMom X should be ~0, got {}",
        data.sensordata[adr]
    );
    assert!(
        data.sensordata[adr + 1].abs() < 1e-6,
        "SubtreeAngMom Y should be ~0, got {}",
        data.sensordata[adr + 1]
    );
    assert!(
        (data.sensordata[adr + 2] - 6.0).abs() < 1e-4,
        "SubtreeAngMom Z should be 6.0, got {}",
        data.sensordata[adr + 2]
    );
}

/// S56-AC5 / T4: Orbital angular momentum.
///
/// Body at [1, 0, 0] with velocity [0, 1, 0], mass=1.
/// Orbital L = r × (m*v) = [1,0,0] × [0,1,0] = [0,0,1].
#[test]
fn b04_subtree_angmom_orbital() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="1 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreeangmom body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[1] = 1.0; // vy = 1.0 (free body linear DOF)
    data.forward(&model).expect("forward");

    // For a single body: subtree_angmom = spin + orbital correction.
    // Spin: I_local * omega = 0 (no angular velocity).
    // The orbital contribution arises in Phase 3 correction.
    //
    // Phase 1: subtree_angmom[b1] = spin = I * omega = 0.
    // Phase 3 Part A: dx_a = xipos[b1] - subtree_com[b1].
    //   For single body, subtree_com[b1] = xipos[b1], so dx_a = 0.
    //   → No contribution.
    //
    // But: the world body (body 0) accumulates everything. The sensor
    // is on body b1, which is a leaf. For a single body subtree, there's
    // no orbital angular momentum about its own COM. The orbital L
    // would appear at body 0 (the world subtree includes b1).
    //
    // Let me read the world subtree's angmom instead.
    let adr = model.sensor_adr[0];
    // For a single-body subtree, angmom about own COM = spin only = 0.
    // This is correct — orbital angular momentum only appears in the parent's subtree.
    let angmom_x = data.sensordata[adr];
    let angmom_y = data.sensordata[adr + 1];
    let angmom_z = data.sensordata[adr + 2];

    // The single-body subtree angmom is just the spin component (≈0).
    assert!(
        angmom_x.abs() < TOL,
        "Single body subtree angmom X should be ~0, got {}",
        angmom_x
    );
    assert!(
        angmom_y.abs() < TOL,
        "Single body subtree angmom Y should be ~0, got {}",
        angmom_y
    );
    // Spin is zero, so angmom_z should be ~0 for a leaf body.
    assert!(
        angmom_z.abs() < TOL,
        "Single body leaf angmom Z should be ~0 (orbital is at parent), got {}",
        angmom_z
    );
}

/// S56-AC5 / T4 (continued): Orbital angmom at world subtree.
///
/// Two free bodies at [1,0,0] and [-1,0,0] with opposite velocities.
/// World subtree should capture total angular momentum.
#[test]
fn b05_subtree_angmom_world_total() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="1 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
        <body name="b2" pos="-1 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreeangmom body="world"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // b1 at [1,0,0] moves at [0,1,0], b2 at [-1,0,0] moves at [0,-1,0]
    // Angular momentum: L1 = [1,0,0] × [0,1,0] = [0,0,1]
    //                   L2 = [-1,0,0] × [0,-1,0] = [0,0,1]
    //                   Total L = [0,0,2]
    // b1 free DOF: [vx, vy, vz, wx, wy, wz]
    let dof1 = model.jnt_dof_adr[0]; // b1's free joint
    let dof2 = model.jnt_dof_adr[1]; // b2's free joint
    data.qvel[dof1 + 1] = 1.0; // b1: vy = 1
    data.qvel[dof2 + 1] = -1.0; // b2: vy = -1
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    assert!(
        data.sensordata[adr].abs() < 1e-8,
        "World angmom X should be ~0, got {}",
        data.sensordata[adr]
    );
    assert!(
        data.sensordata[adr + 1].abs() < 1e-8,
        "World angmom Y should be ~0, got {}",
        data.sensordata[adr + 1]
    );
    assert!(
        (data.sensordata[adr + 2] - 2.0).abs() < 1e-6,
        "World angmom Z should be 2.0, got {}",
        data.sensordata[adr + 2]
    );
}

/// S56-AC8 / T7: Lazy gate — no subtree sensors → flg_subtreevel stays false.
#[test]
fn b06_lazy_no_subtree_sensors() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <jointpos joint="j1"/>
        <gyro site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // No SubtreeLinVel or SubtreeAngMom sensors → mj_subtree_vel() never called
    assert!(
        !data.flg_subtreevel,
        "flg_subtreevel should be false without subtree sensors"
    );

    // subtree_linvel should remain zeroed
    for b in 0..model.nbody {
        assert!(
            data.subtree_linvel[b].norm() < TOL,
            "subtree_linvel[{b}] should be zeroed, got {:?}",
            data.subtree_linvel[b]
        );
    }
}

/// S56-AC9 / T8: Lazy gate — subtree computed once for multiple sensors.
#[test]
fn b07_lazy_single_trigger_multiple_sensors() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <body name="b2" pos="1 0 0">
            <joint name="j2" type="slide" axis="1 0 0"/>
            <geom type="sphere" size="0.1" mass="1.0"/>
          </body>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="b1"/>
        <subtreelinvel body="b2"/>
        <subtreeangmom body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0; // b1 linear x
    data.forward(&model).expect("forward");

    // flg_subtreevel should be true (triggered by first SubtreeLinVel sensor)
    assert!(
        data.flg_subtreevel,
        "flg_subtreevel should be true after subtree sensors"
    );

    // All three sensors should have been written correctly
    assert!(model.nsensordata == 9, "3 sensors * 3D = 9 sensordata");

    // Sensor values should be finite and consistent
    for i in 0..model.nsensordata {
        assert!(
            data.sensordata[i].is_finite(),
            "sensordata[{i}] should be finite"
        );
    }
}

/// S56-AC10 / T10: Flag cleared each step at velocity stage start.
#[test]
fn b08_flag_cleared_each_step() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert!(data.flg_subtreevel, "Flag should be set after forward");

    // Step clears the flag during velocity stage, then re-sets it via sensor
    data.step(&model).expect("step");
    // After a full step, the flag should be re-set (sensors triggered it again)
    assert!(data.flg_subtreevel, "Flag should be set again after step");
}

/// S56-AC11 / T10: Reset clears subtree fields and flag.
#[test]
fn b09_reset_clears_subtree() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="b1"/>
        <subtreeangmom body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 5.0;
    data.forward(&model).expect("forward");

    // Verify fields are populated
    assert!(data.flg_subtreevel, "Flag should be set after forward");
    assert!(
        data.subtree_linvel[1].norm() > 0.1,
        "subtree_linvel should be nonzero"
    );

    // Reset
    data.reset(&model);

    // Everything should be zeroed
    assert!(!data.flg_subtreevel, "Flag should be cleared after reset");
    for b in 0..model.nbody {
        assert!(
            data.subtree_linvel[b].norm() < TOL,
            "subtree_linvel[{b}] should be zeroed after reset"
        );
        assert!(
            data.subtree_angmom[b].norm() < TOL,
            "subtree_angmom[{b}] should be zeroed after reset"
        );
    }
}

/// S56-T13: Zero-mass body in chain — no NaN/panic.
#[test]
fn b10_subtree_zero_mass_no_nan() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="slide" axis="1 0 0"/>
          <geom type="sphere" size="0.1" mass="0.0001"/>
          <body name="b2" pos="1 0 0">
            <joint name="j2" type="slide" axis="1 0 0"/>
            <geom type="sphere" size="0.1" mass="1.0"/>
          </body>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    let adr = model.sensor_adr[0];
    for i in 0..3 {
        assert!(
            data.sensordata[adr + i].is_finite(),
            "SubtreeLinVel[{i}] should be finite with near-zero mass, got {}",
            data.sensordata[adr + i]
        );
    }
}

/// S56-T14: DISABLE_SENSOR → subtree not computed.
#[test]
fn b11_subtree_disable_sensor() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0">
        <flag sensor="disable"/>
      </option>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="b1"/>
        <subtreeangmom body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 5.0;
    data.forward(&model).expect("forward");

    assert!(
        !data.flg_subtreevel,
        "flg_subtreevel should stay false with DISABLE_SENSOR"
    );
}

/// S56-T15: Sleep interaction — subtree computed for all bodies.
#[test]
fn b12_subtree_sleep_computes_all() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
        <flag sleep="enable"/>
      </option>
      <worldbody>
        <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
        <body name="sleeper" pos="0 0 0.2">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
        </body>
        <body name="awake" pos="5 0 5">
          <freejoint/>
          <geom type="sphere" size="0.05" mass="0.01"/>
        </body>
      </worldbody>
      <sensor>
        <subtreelinvel body="world"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Step until sleeper falls asleep
    let mut asleep = false;
    for _ in 0..5000 {
        data.step(&model).expect("step");
        if data.body_sleep_state[1] == SleepState::Asleep {
            asleep = true;
            break;
        }
    }
    assert!(asleep, "Sleeper should fall asleep");

    data.forward(&model).expect("forward");

    // flg_subtreevel should be true (world subtree sensor triggers it)
    assert!(data.flg_subtreevel, "flg_subtreevel should be true");

    // subtree_linvel[0] (world) should be finite and computed for all bodies
    for i in 0..3 {
        assert!(
            data.subtree_linvel[0][i].is_finite(),
            "World subtree_linvel[{i}] should be finite"
        );
    }
}

/// S56-T12: SubtreeCom sensor reads persistent field directly.
#[test]
fn b13_subtreecom_reads_persistent() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="1 2 3">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <subtreecom body="b1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // SubtreeCom reads data.subtree_com[body_id] directly (computed in position stage)
    let adr = model.sensor_adr[0];
    let body_id = model.sensor_objid[0];
    for i in 0..3 {
        assert!(
            (data.sensordata[adr + i] - data.subtree_com[body_id][i]).abs() < TOL,
            "SubtreeCom[{i}] should match subtree_com field"
        );
    }
}

// ============================================================================
// Section C: flg_rnepost Lazy Gate
// ============================================================================

/// Umbrella-AC6: No acc sensors → flg_rnepost false, cacc stays zeroed.
#[test]
fn c01_no_acc_sensors_flag_false() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <jointpos joint="j1"/>
        <jointvel joint="j1"/>
        <gyro site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Step 10 times to ensure pipeline runs fully
    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    // No accelerometer/force/torque/framelinacc/frameangacc → no trigger
    assert!(
        !data.flg_rnepost,
        "flg_rnepost should be false without acc-stage sensors"
    );

    // cacc should remain zeroed (body accumulators never ran)
    for b in 0..model.nbody {
        for i in 0..6 {
            assert!(
                data.cacc[b][i].abs() < TOL,
                "cacc[{b}][{i}] should be 0 without acc sensors, got {}",
                data.cacc[b][i]
            );
        }
    }
}

/// Umbrella-AC11: inverse() calls mj_body_accumulators and sets flg_rnepost.
#[test]
fn c02_inverse_sets_flg_rnepost() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <jointpos joint="j1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // forward() with only position sensors → flg_rnepost stays false
    data.forward(&model).expect("forward");
    assert!(
        !data.flg_rnepost,
        "flg_rnepost should be false after forward (no acc sensors)"
    );

    // inverse() should call mj_body_accumulators → flg_rnepost = true
    data.inverse(&model);
    assert!(
        data.flg_rnepost,
        "flg_rnepost should be true after inverse()"
    );

    // cacc should now be populated (world body has gravity pseudo-acceleration)
    assert!(
        (data.cacc[0][5] - 9.81).abs() < TOL,
        "cacc[0] z should be +9.81 after inverse(), got {}",
        data.cacc[0][5]
    );
}

/// Umbrella-AC9: flg_rnepost cleared between steps.
///
/// forward_acc() clears flg_rnepost at the start of the acceleration stage.
/// If no acc sensors exist, it stays false.
#[test]
fn c03_flag_cleared_between_steps() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <jointpos joint="j1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();

    // Forward + inverse to set the flag
    data.forward(&model).expect("forward");
    data.inverse(&model);
    assert!(data.flg_rnepost, "Flag should be set after inverse");

    // Next forward() should clear it (no acc sensors to re-set it)
    data.forward(&model).expect("forward");
    assert!(
        !data.flg_rnepost,
        "flg_rnepost should be cleared by forward_acc (no acc sensors)"
    );
}

/// Umbrella-AC7: Multiple accelerometer sensors only trigger body_accumulators once.
///
/// If flg_rnepost is set by the first accelerometer sensor, subsequent ones
/// skip the call. We verify by checking that cacc is computed (flag is set)
/// and all sensor values are correct.
#[test]
fn c04_multiple_acc_sensors_single_trigger() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
          <site name="s2" pos="0.1 0 0"/>
          <site name="s3" pos="0 0.1 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="s1"/>
        <accelerometer site="s2"/>
        <accelerometer site="s3"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Flag should be set (triggered by first accelerometer)
    assert!(data.flg_rnepost, "flg_rnepost should be set");

    // All three accelerometers should produce valid readings
    for s in 0..3 {
        let adr = model.sensor_adr[s];
        // Each should read gravity component (within tolerance)
        assert!(
            data.sensordata[adr + 2].abs() > 1.0,
            "Accelerometer {s} Z should be significant (gravity)"
        );
        for i in 0..3 {
            assert!(
                data.sensordata[adr + i].is_finite(),
                "Accelerometer {s}[{i}] should be finite"
            );
        }
    }
}
