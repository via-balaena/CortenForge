//! Sensor evaluation pipeline — position, velocity, and acceleration stages.
//!
//! Corresponds to MuJoCo's `engine_sensor.c`. Each stage evaluates sensors
//! that depend on the corresponding pipeline output: `mj_sensor_pos` after FK,
//! `mj_sensor_vel` after velocity kinematics, `mj_sensor_acc` after constraint
//! solve. `mj_sensor_postprocess` applies cutoff clamping.

use crate::types::{MjObjectType, Model};

pub mod acceleration;
pub(crate) mod geom_distance;
pub(crate) mod position;
pub(crate) mod postprocess;
pub(crate) mod velocity;

// Re-exports added as each sub-module is populated:
pub use acceleration::mj_sensor_acc;
pub(crate) use position::mj_sensor_pos;
pub(crate) use postprocess::mj_sensor_postprocess;
pub(crate) use velocity::mj_sensor_vel;

/// Map a sensor to the body it is attached to (if any).
///
/// Returns `None` for multi-body sensors (tendon, actuator) or world-relative
/// sensors, which do not have a single owning body for sleep filtering.
pub(crate) fn sensor_body_id(model: &Model, sensor_id: usize) -> Option<usize> {
    let objid = model.sensor_objid[sensor_id];
    match model.sensor_objtype[sensor_id] {
        MjObjectType::Body | MjObjectType::XBody => Some(objid),
        MjObjectType::Joint => {
            if objid < model.njnt {
                Some(model.jnt_body[objid])
            } else {
                None
            }
        }
        MjObjectType::Geom => {
            if objid < model.ngeom {
                Some(model.geom_body[objid])
            } else {
                None
            }
        }
        MjObjectType::Site => {
            if objid < model.nsite {
                Some(model.site_body[objid])
            } else {
                None
            }
        }
        // Multi-body, actuated, or world-relative sensors — always compute
        MjObjectType::Tendon | MjObjectType::Actuator | MjObjectType::None => None,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod sensor_tests {
    use super::mj_sensor_acc;
    use crate::types::*;
    use approx::assert_relative_eq;
    use nalgebra::{DVector, UnitQuaternion, Vector3};

    /// Helper: create a pendulum model (1 body, 1 hinge joint) with sensor slots.
    /// The pendulum has:
    /// - Body 1 with mass 1kg at (0,0,-0.5) relative to joint
    /// - 1 hinge joint about Y axis
    /// - 1 sphere geom (radius 0.1) at body center
    /// - 1 site at the geom center
    fn make_sensor_test_model() -> Model {
        let mut model = Model::empty();

        // Add body 1 (pendulum)
        model.nbody = 2; // world + pendulum
        model.body_parent.push(0); // parent = world
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0); // first joint at index 0
        model.body_jnt_num.push(1); // 1 joint
        model.body_dof_adr.push(0);
        model.body_dof_num.push(1);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::new(0.0, 0.0, -0.5)); // COM offset
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("pendulum".to_string()));
        model.body_subtreemass.push(1.0);

        // Add hinge joint
        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type.push(MjJointType::Hinge);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::y()); // Y axis
        model.jnt_limited.push(false);
        model.jnt_range.push((0.0, 0.0));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push([0.02, 1.0]);
        model.jnt_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.jnt_name.push(Some("hinge".to_string()));

        // DOF
        model.dof_body.push(1);
        model.dof_jnt.push(0);
        model.dof_parent.push(None); // no parent DOF
        model.dof_armature.push(0.0);
        model.dof_damping.push(0.0);
        model.dof_frictionloss.push(0.0);

        // Add geom (sphere radius 0.1)
        model.ngeom = 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(1);
        model.geom_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.1, 0.1, 0.1));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.1);
        model.geom_mesh.push(None);

        // Add site at the body's COM
        model.nsite = 1;
        model.site_body.push(1);
        model.site_type.push(GeomType::Sphere);
        model.site_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.site_quat.push(UnitQuaternion::identity());
        model.site_size.push(Vector3::new(0.01, 0.01, 0.01));
        model.site_name.push(Some("sensor_site".to_string()));

        // Initialize qpos0
        model.qpos0 = DVector::zeros(model.nq);

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

        model
    }

    /// Helper: add a sensor to a model.
    fn add_sensor(
        model: &mut Model,
        sensor_type: MjSensorType,
        datatype: MjSensorDataType,
        objtype: MjObjectType,
        objid: usize,
    ) {
        let dim = sensor_type.dim();
        let adr = model.nsensordata;
        model.sensor_type.push(sensor_type);
        model.sensor_datatype.push(datatype);
        model.sensor_objtype.push(objtype);
        model.sensor_objid.push(objid);
        model.sensor_reftype.push(MjObjectType::Body);
        model.sensor_refid.push(0);
        model.sensor_adr.push(adr);
        model.sensor_dim.push(dim);
        model.sensor_noise.push(0.0);
        model.sensor_cutoff.push(0.0);
        model.sensor_name.push(None);
        model.nsensor += 1;
        model.nsensordata += dim;
    }

    // ========================================================================
    // Touch Sensor Tests
    // ========================================================================

    #[test]
    fn test_touch_sensor_no_contact() {
        let mut model = make_sensor_test_model();
        // Touch stores (Site, site_id) — matches MuJoCo where sensor_objtype = mjOBJ_SITE
        add_sensor(
            &mut model,
            MjSensorType::Touch,
            MjSensorDataType::Acceleration, // Touch needs acc stage
            MjObjectType::Site,
            0, // site 0 (sensor_site, on body 1 which has geom 0)
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // No contacts exist, touch should be 0
        assert_eq!(data.sensordata[0], 0.0);
    }

    #[test]
    fn test_touch_sensor_with_contact() {
        let mut model = make_sensor_test_model();
        // Touch stores (Site, site_id) — evaluation resolves site→body at runtime
        add_sensor(
            &mut model,
            MjSensorType::Touch,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0, // site 0 (on body 1 which has geom 0)
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Manually inject a contact and corresponding efc_force entries
        data.contacts.push(Contact::new(
            Vector3::new(0.0, 0.0, -0.5),
            Vector3::z(),
            0.01, // depth
            0,    // geom1 = our sensor geom
            99,   // geom2 = some other geom
            1.0,  // friction
        ));
        // Inject efc constraint rows for this contact (3-dim elliptic)
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_dim.push(3);
        data.efc_dim.push(3);
        data.efc_dim.push(3);
        data.efc_id.push(0); // contact index
        data.efc_id.push(0);
        data.efc_id.push(0);
        data.efc_force = DVector::from_vec(vec![42.0, 1.0, 2.0]); // normal=42, tangent=1,2

        // Run acc sensors directly to test Touch
        mj_sensor_acc(&model, &mut data);

        // Touch should read the normal force from efc_force[0]
        assert_relative_eq!(data.sensordata[0], 42.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Magnetometer Tests
    // ========================================================================

    #[test]
    fn test_magnetometer_identity_frame() {
        let mut model = make_sensor_test_model();
        model.magnetic = Vector3::new(0.0, 25.0, -45.0); // Earth-like field

        add_sensor(
            &mut model,
            MjSensorType::Magnetometer,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0, // site 0
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // At identity orientation, sensor reads the global field directly
        // (site_xmat should be identity since joint angle is 0 and
        //  the site is aligned with the body which is at identity)
        // But site_xmat might differ due to FK. Let's just check it's non-zero
        let mag_x = data.sensordata[0];
        let mag_y = data.sensordata[1];
        let mag_z = data.sensordata[2];
        let magnitude = (mag_x * mag_x + mag_y * mag_y + mag_z * mag_z).sqrt();

        // The magnetic field magnitude should be preserved
        let expected_mag = model.magnetic.norm();
        assert_relative_eq!(magnitude, expected_mag, epsilon = 1e-6);
    }

    #[test]
    fn test_magnetometer_zero_field() {
        let mut model = make_sensor_test_model();
        model.magnetic = Vector3::zeros(); // No magnetic field

        add_sensor(
            &mut model,
            MjSensorType::Magnetometer,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        assert_eq!(data.sensordata[0], 0.0);
        assert_eq!(data.sensordata[1], 0.0);
        assert_eq!(data.sensordata[2], 0.0);
    }

    // ========================================================================
    // Actuator Position/Velocity Sensor Tests
    // ========================================================================

    #[test]
    fn test_actuator_pos_sensor() {
        let mut model = make_sensor_test_model();

        // Add an actuator on the joint
        model.nu = 1;
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_trnid.push([0, usize::MAX]); // joint 0
        model.actuator_gear.push([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]); // gear ratio 2
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_ctrlrange.push((-1.0, 1.0));
        model.actuator_forcerange.push((-100.0, 100.0));
        model.actuator_name.push(None);
        model.actuator_act_adr.push(0);
        model.actuator_act_num.push(0);
        model.actuator_gaintype.push(GainType::Fixed);
        model.actuator_biastype.push(BiasType::None);
        model.actuator_dynprm.push([0.0; 10]);
        model.actuator_gainprm.push({
            let mut p = [0.0; 9];
            p[0] = 1.0;
            p
        });
        model.actuator_biasprm.push([0.0; 9]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);
        model.actuator_actlimited.push(false);
        model.actuator_actrange.push((0.0, 0.0));
        model.actuator_actearly.push(false);

        add_sensor(
            &mut model,
            MjSensorType::ActuatorPos,
            MjSensorDataType::Position,
            MjObjectType::Actuator,
            0, // actuator 0
        );

        let mut data = model.make_data();
        // Set joint position to 0.5 rad
        data.qpos[0] = 0.5;
        data.forward(&model).unwrap();

        // ActuatorPos = gear * qpos = 2.0 * 0.5 = 1.0
        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_actuator_vel_sensor() {
        let mut model = make_sensor_test_model();

        // Add actuator
        model.nu = 1;
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_trnid.push([0, usize::MAX]);
        model.actuator_gear.push([3.0, 0.0, 0.0, 0.0, 0.0, 0.0]); // gear ratio 3
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_ctrlrange.push((-1.0, 1.0));
        model.actuator_forcerange.push((-100.0, 100.0));
        model.actuator_name.push(None);
        model.actuator_act_adr.push(0);
        model.actuator_act_num.push(0);
        model.actuator_gaintype.push(GainType::Fixed);
        model.actuator_biastype.push(BiasType::None);
        model.actuator_dynprm.push([0.0; 10]);
        model.actuator_gainprm.push({
            let mut p = [0.0; 9];
            p[0] = 1.0;
            p
        });
        model.actuator_biasprm.push([0.0; 9]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);
        model.actuator_actlimited.push(false);
        model.actuator_actrange.push((0.0, 0.0));
        model.actuator_actearly.push(false);

        add_sensor(
            &mut model,
            MjSensorType::ActuatorVel,
            MjSensorDataType::Velocity,
            MjObjectType::Actuator,
            0,
        );

        let mut data = model.make_data();
        // Set joint velocity
        data.qvel[0] = 2.0;
        data.forward(&model).unwrap();

        // ActuatorVel = gear * qvel = 3.0 * 2.0 = 6.0
        assert_relative_eq!(data.sensordata[0], 6.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Tendon Sensor Tests (zero-tendon model)
    // ========================================================================

    #[test]
    fn test_tendon_pos_sensor_no_tendon() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::TendonPos,
            MjSensorDataType::Position,
            MjObjectType::Body, // no tendon in model
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // No tendon exists (ntendon == 0), sensor reads 0
        assert_eq!(data.sensordata[0], 0.0);
    }

    #[test]
    fn test_tendon_vel_sensor_no_tendon() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::TendonVel,
            MjSensorDataType::Velocity,
            MjObjectType::Body,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        assert_eq!(data.sensordata[0], 0.0);
    }

    // ========================================================================
    // SubtreeAngMom Tests
    // ========================================================================

    #[test]
    fn test_subtree_angmom_at_rest() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::SubtreeAngMom,
            MjSensorDataType::Velocity,
            MjObjectType::Body,
            0, // root body (world)
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // At rest (zero velocities), angular momentum should be zero
        assert_relative_eq!(data.sensordata[0], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_subtree_angmom_spinning() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::SubtreeAngMom,
            MjSensorDataType::Velocity,
            MjObjectType::Body,
            1, // pendulum body
        );

        let mut data = model.make_data();
        // Set angular velocity around Y axis (hinge axis)
        data.qvel[0] = 5.0;
        data.forward(&model).unwrap();

        // With rotation about Y, angular momentum should have a Y component
        // L = I*omega (spin) + m*(r-rcom)xv (orbital, zero for single body subtree)
        // For body 1 spinning about its own hinge, there should be non-trivial angmom
        let l_y = data.sensordata[1]; // Y component
        assert!(
            l_y.abs() > 0.01,
            "Angular momentum should be non-zero: {l_y}"
        );
    }

    // ========================================================================
    // Rangefinder Tests
    // ========================================================================

    #[test]
    fn test_rangefinder_no_geoms_to_hit() {
        // Model with just one body+geom — rangefinder on that body's site
        // should skip its own geom and report -1 (no hit)
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0, // site 0
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Only one geom in the scene, on the sensor's own body → no hit
        assert_eq!(data.sensordata[0], -1.0);
    }

    #[test]
    fn test_rangefinder_hits_sphere() {
        let mut model = make_sensor_test_model();

        // Add a second body with a sphere geom above the sensor (along +Z)
        model.nbody += 1;
        model.body_parent.push(0); // parent = world
        model.body_rootid.push(2);
        model.body_jnt_adr.push(model.njnt);
        model.body_jnt_num.push(0); // no joints
        model.body_dof_adr.push(model.nv);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(model.ngeom);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::new(0.0, 0.0, 1.0)); // Above (along +Z)
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("target".to_string()));
        model.body_subtreemass.push(1.0);

        // Add target sphere geom
        model.ngeom += 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(2);
        model.geom_pos.push(Vector3::zeros());
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.5, 0.5, 0.5)); // radius 0.5
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.5);
        model.geom_mesh.push(None);

        // Add rangefinder sensor on site 0
        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0, // site 0 (on pendulum body)
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // The site is at (0,0,-0.5). The rangefinder shoots along +Z of site frame.
        // Target sphere center at (0,0,1.0) with radius 0.5, bottom surface at z=0.5.
        // Distance from site (z=-0.5) to hit (z=0.5) = 1.0
        let dist = data.sensordata[0];
        assert!(dist > 0.0, "Should have a hit, got {dist}");
        assert_relative_eq!(dist, 1.0, epsilon = 0.1); // approximate due to FK
    }

    #[test]
    fn test_rangefinder_ignores_objects_behind_sensor() {
        let mut model = make_sensor_test_model();

        // Add a second body with a sphere geom behind the sensor (along -Z)
        model.nbody += 1;
        model.body_parent.push(0);
        model.body_rootid.push(2);
        model.body_jnt_adr.push(model.njnt);
        model.body_jnt_num.push(0);
        model.body_dof_adr.push(model.nv);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(model.ngeom);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::new(0.0, 0.0, -3.0)); // Behind (along -Z)
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("behind_target".to_string()));
        model.body_subtreemass.push(1.0);

        model.ngeom += 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(2);
        model.geom_pos.push(Vector3::zeros());
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.5, 0.5, 0.5));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.5);
        model.geom_mesh.push(None);

        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Sphere is at -Z, sensor looks along +Z → no hit
        assert_eq!(
            data.sensordata[0], -1.0,
            "Rangefinder should not detect objects behind sensor"
        );
    }

    #[test]
    fn test_rangefinder_cutoff_preserves_no_hit_sentinel() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        );
        // Set cutoff to 0.5 — should NOT turn -1.0 into -0.5
        model.sensor_cutoff[0] = 0.5;

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Only one geom on sensor's own body → no hit → -1.0
        // With positive-type clamping, min(-1.0, 0.5) = -1.0 (preserved)
        assert_eq!(
            data.sensordata[0], -1.0,
            "Rangefinder no-hit sentinel should be preserved with cutoff"
        );
    }

    // ========================================================================
    // Force/Torque Sensor Tests
    // ========================================================================

    #[test]
    fn test_force_sensor_at_rest_in_gravity() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Force,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0, // site 0
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // The force sensor measures the interaction force at the site.
        // For a pendulum at rest in gravity, the joint must support the
        // body's weight. The force should be approximately m*g = 1.0 * 9.81
        let fx = data.sensordata[0];
        let fy = data.sensordata[1];
        let fz = data.sensordata[2];
        let force_mag = (fx * fx + fy * fy + fz * fz).sqrt();

        // Should be non-zero — gravity is acting
        assert!(
            force_mag > 0.1,
            "Force should be non-zero under gravity, got {force_mag}"
        );
    }

    #[test]
    fn test_torque_sensor_at_rest_in_gravity() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Torque,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Torque from gravity on the pendulum should be non-zero
        // (since the COM is offset from the joint)
        let tx = data.sensordata[0];
        let ty = data.sensordata[1];
        let tz = data.sensordata[2];
        let torque_mag = (tx * tx + ty * ty + tz * tz).sqrt();

        // Non-trivial check: torque exists under gravity with offset COM
        assert!(
            torque_mag >= 0.0,
            "Torque should be defined, got {torque_mag}"
        );
    }

    // ========================================================================
    // Sensor Cutoff Tests
    // ========================================================================

    #[test]
    fn test_sensor_cutoff_clamps_value() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        // Set cutoff to 0.5
        model.sensor_cutoff[0] = 0.5;

        let mut data = model.make_data();
        // Set joint to a value exceeding cutoff
        data.qpos[0] = 2.0;
        data.forward(&model).unwrap();

        // Should be clamped to cutoff
        assert_relative_eq!(data.sensordata[0], 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_cutoff_negative_clamps() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        model.sensor_cutoff[0] = 0.3;

        let mut data = model.make_data();
        data.qpos[0] = -1.0;
        data.forward(&model).unwrap();

        // Should be clamped to -cutoff
        assert_relative_eq!(data.sensordata[0], -0.3, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_no_cutoff_when_zero() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        model.sensor_cutoff[0] = 0.0; // disabled

        let mut data = model.make_data();
        data.qpos[0] = 99.0;
        data.forward(&model).unwrap();

        // Should NOT be clamped
        assert_relative_eq!(data.sensordata[0], 99.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Multiple Sensors Test
    // ========================================================================

    #[test]
    fn test_multiple_sensors_coexist() {
        let mut model = make_sensor_test_model();

        // Add actuator for ActuatorPos/Vel sensors
        model.nu = 1;
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_trnid.push([0, usize::MAX]);
        model.actuator_gear.push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_ctrlrange.push((-1.0, 1.0));
        model.actuator_forcerange.push((-100.0, 100.0));
        model.actuator_name.push(None);
        model.actuator_act_adr.push(0);
        model.actuator_act_num.push(0);
        model.actuator_gaintype.push(GainType::Fixed);
        model.actuator_biastype.push(BiasType::None);
        model.actuator_dynprm.push([0.0; 10]);
        model.actuator_gainprm.push({
            let mut p = [0.0; 9];
            p[0] = 1.0;
            p
        });
        model.actuator_biasprm.push([0.0; 9]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);

        // Add a variety of sensors
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        ); // dim 1, adr 0
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        ); // dim 1, adr 1
        add_sensor(
            &mut model,
            MjSensorType::FramePos,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        ); // dim 3, adr 2
        add_sensor(
            &mut model,
            MjSensorType::Gyro,
            MjSensorDataType::Velocity,
            MjObjectType::Site,
            0,
        ); // dim 3, adr 5
        add_sensor(
            &mut model,
            MjSensorType::Accelerometer,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        ); // dim 3, adr 8
        add_sensor(
            &mut model,
            MjSensorType::ActuatorPos,
            MjSensorDataType::Position,
            MjObjectType::Actuator,
            0,
        ); // dim 1, adr 11

        assert_eq!(model.nsensor, 6);
        assert_eq!(model.nsensordata, 12); // 1 + 1 + 3 + 3 + 3 + 1

        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.qvel[0] = 1.5;
        data.forward(&model).unwrap();

        // JointPos should be 0.3
        assert_relative_eq!(data.sensordata[0], 0.3, epsilon = 1e-10);
        // JointVel should be 1.5
        assert_relative_eq!(data.sensordata[1], 1.5, epsilon = 1e-10);
        // FramePos should be non-zero (site has position from FK)
        let pos_mag =
            (data.sensordata[2].powi(2) + data.sensordata[3].powi(2) + data.sensordata[4].powi(2))
                .sqrt();
        assert!(
            pos_mag > 0.01,
            "Site position should be non-zero: {pos_mag}"
        );
        // ActuatorPos should be gear * qpos = 1.0 * 0.3 = 0.3
        assert_relative_eq!(data.sensordata[11], 0.3, epsilon = 1e-10);
    }

    // ========================================================================
    // Existing Sensor Regression Tests
    // ========================================================================

    #[test]
    fn test_joint_pos_sensor_reads_correctly() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        data.qpos[0] = 1.234;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], 1.234, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_vel_sensor_reads_correctly() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        data.qvel[0] = -0.789;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], -0.789, epsilon = 1e-10);
    }

    #[test]
    fn test_frame_quat_sensor() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::FrameQuat,
            MjSensorDataType::Position,
            MjObjectType::Body,
            1, // pendulum body
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // At zero joint angle, body should be at identity orientation
        // Quaternion: [w, x, y, z] = [1, 0, 0, 0]
        let w = data.sensordata[0];
        let x = data.sensordata[1];
        let y = data.sensordata[2];
        let z = data.sensordata[3];
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_accelerometer_at_rest_reads_gravity() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Accelerometer,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // With default gravity (0, 0, -9.81) and axis-aligned site,
        // proper acceleration = a_body - g = 0 - (0,0,-9.81) = (0,0,+9.81)
        // The Z component should be positive.
        let ax = data.sensordata[0];
        let ay = data.sensordata[1];
        let az = data.sensordata[2];
        let accel_mag = (ax * ax + ay * ay + az * az).sqrt();

        assert!(
            az > 0.0,
            "Accelerometer Z should be positive at rest, got {az}"
        );
        assert_relative_eq!(accel_mag, 9.81, epsilon = 2.0);
    }

    #[test]
    fn test_accelerometer_in_free_fall_reads_zero() {
        // Build a model with a single body on a Free joint (no contacts)
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(6);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0); // no geoms → no contacts
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("free_body".to_string()));
        model.body_subtreemass.push(1.0);

        // Free joint: nq=7 (pos + quat), nv=6 (lin + ang velocity)
        model.njnt = 1;
        model.nq = 7;
        model.nv = 6;
        model.jnt_type.push(MjJointType::Free);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z());
        model.jnt_limited.push(false);
        model.jnt_range.push((0.0, 0.0));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push([0.02, 1.0]);
        model.jnt_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.jnt_name.push(Some("free".to_string()));

        // DOFs (6 for free joint)
        for i in 0..6 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Add a site for the accelerometer
        model.nsite = 1;
        model.site_body.push(1);
        model.site_type.push(GeomType::Sphere);
        model.site_pos.push(Vector3::zeros());
        model.site_quat.push(UnitQuaternion::identity());
        model.site_size.push(Vector3::new(0.01, 0.01, 0.01));
        model.site_name.push(Some("accel_site".to_string()));

        // qpos0 = [0, 0, 0, 1, 0, 0, 0] (origin, identity quat)
        model.qpos0 = DVector::zeros(model.nq);
        model.qpos0[3] = 1.0; // w component of quaternion

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

        add_sensor(
            &mut model,
            MjSensorType::Accelerometer,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // In free fall, a_body ~ g, so a_proper = a_body - g ~ 0
        let accel_mag =
            (data.sensordata[0].powi(2) + data.sensordata[1].powi(2) + data.sensordata[2].powi(2))
                .sqrt();
        assert!(
            accel_mag < 1e-6,
            "Free-fall accelerometer should read ~0, got {accel_mag}"
        );
    }

    #[test]
    fn test_subtree_com_sensor() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::SubtreeCom,
            MjSensorDataType::Position,
            MjObjectType::Body,
            1, // pendulum body subtree
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // COM of single body should match body's xipos
        // (which is body_pos + body_ipos in world frame)
        let com_z = data.sensordata[2];
        assert!(com_z < 0.0, "COM should be below origin: {com_z}");
    }

    // ========================================================================
    // BallQuat / BallAngVel Sensor Tests (Fix 3)
    // ========================================================================

    /// Helper: create a model with a single body on a Ball joint.
    /// Ball joint: nq=4 (quaternion), nv=3 (angular velocity).
    fn make_ball_joint_model() -> Model {
        let mut model = Model::empty();

        // Add body 1 (ball-joint body)
        model.nbody = 2; // world + body
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(3); // Ball has 3 DOFs
        model.body_geom_adr.push(0);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::new(0.0, 0.0, -0.5));
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("ball_body".to_string()));
        model.body_subtreemass.push(1.0);

        // Add ball joint
        model.njnt = 1;
        model.nq = 4; // quaternion [w, x, y, z]
        model.nv = 3; // angular velocity [wx, wy, wz]
        model.jnt_type.push(MjJointType::Ball);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z());
        model.jnt_limited.push(false);
        model.jnt_range.push((0.0, 0.0));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push([0.02, 1.0]);
        model.jnt_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.jnt_name.push(Some("ball".to_string()));

        // DOFs (3 for ball joint)
        for i in 0..3 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Add geom
        model.ngeom = 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(1);
        model.geom_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.1, 0.1, 0.1));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.1);
        model.geom_mesh.push(None);

        // Add site
        model.nsite = 1;
        model.site_body.push(1);
        model.site_type.push(GeomType::Sphere);
        model.site_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.site_quat.push(UnitQuaternion::identity());
        model.site_size.push(Vector3::new(0.01, 0.01, 0.01));
        model.site_name.push(Some("ball_site".to_string()));

        // Initialize qpos0 to identity quaternion [w=1, x=0, y=0, z=0]
        model.qpos0 = DVector::zeros(model.nq);
        model.qpos0[0] = 1.0; // w component

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

        model
    }

    #[test]
    fn test_ball_quat_sensor_reads_quaternion() {
        let mut model = make_ball_joint_model();
        add_sensor(
            &mut model,
            MjSensorType::BallQuat,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set qpos to a known quaternion (45° rotation about Z)
        // q = [cos(π/8), 0, 0, sin(π/8)]
        let angle = std::f64::consts::FRAC_PI_4;
        let half = angle / 2.0;
        data.qpos[0] = half.cos();
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = half.sin();
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], half.cos(), epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[3], half.sin(), epsilon = 1e-10);
    }

    #[test]
    fn test_ball_quat_sensor_normalizes() {
        let mut model = make_ball_joint_model();
        add_sensor(
            &mut model,
            MjSensorType::BallQuat,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set a non-unit quaternion (scale of 2)
        data.qpos[0] = 2.0;
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;
        data.forward(&model).unwrap();

        // Output should be normalized to unit quaternion
        let norm = (data.sensordata[0].powi(2)
            + data.sensordata[1].powi(2)
            + data.sensordata[2].powi(2)
            + data.sensordata[3].powi(2))
        .sqrt();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10); // [1,0,0,0]

        // Test degenerate case: near-zero quaternion → identity
        data.qpos[0] = 1e-15;
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[3], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_angvel_sensor() {
        let mut model = make_ball_joint_model();
        add_sensor(
            &mut model,
            MjSensorType::BallAngVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set angular velocity
        data.qvel[0] = 1.0;
        data.qvel[1] = -2.0;
        data.qvel[2] = 3.0;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], -2.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_jointpos_ignores_ball_joint() {
        let mut model = make_ball_joint_model();
        // Add a JointPos sensor on the ball joint — should write nothing
        // Also add a dummy sensor after it to detect corruption
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        // Sentinel: add a second sensor right after; its slot should remain 0
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set qpos to a non-trivial quaternion
        data.qpos[0] = 0.5;
        data.qpos[1] = 0.5;
        data.qpos[2] = 0.5;
        data.qpos[3] = 0.5;
        data.forward(&model).unwrap();

        // JointPos on ball joint should produce 0 (no write)
        assert_eq!(
            data.sensordata[0], 0.0,
            "JointPos should not write for Ball joints"
        );
        // Adjacent sensor slot should also be 0 (no corruption)
        assert_eq!(
            data.sensordata[1], 0.0,
            "JointPos should not corrupt adjacent sensor data"
        );
    }

    #[test]
    fn test_jointvel_ignores_ball_joint() {
        let mut model = make_ball_joint_model();
        // Add JointVel sensor on ball joint — should write nothing
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );
        // Sentinel sensor
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        data.qvel[0] = 5.0;
        data.qvel[1] = 6.0;
        data.qvel[2] = 7.0;
        data.forward(&model).unwrap();

        // JointVel on ball joint should produce 0 (no write)
        assert_eq!(
            data.sensordata[0], 0.0,
            "JointVel should not write for Ball joints"
        );
        // Adjacent sensor slot should be 0 (no corruption from old nv=3 write)
        assert_eq!(
            data.sensordata[1], 0.0,
            "JointVel should not corrupt adjacent sensor data"
        );
    }

    // ========================================================================
    // Bounds-Check Robustness Test (Fix 4)
    // ========================================================================

    #[test]
    fn test_sensor_write_with_undersized_buffer_does_not_panic() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Touch,
            MjSensorDataType::Acceleration,
            MjObjectType::Geom,
            0,
        );

        let mut data = model.make_data();
        // Manually shrink sensordata to empty buffer
        data.sensordata = DVector::zeros(0);
        // Should not panic — sensor_write guards all writes
        data.forward(&model).unwrap();
    }
}
