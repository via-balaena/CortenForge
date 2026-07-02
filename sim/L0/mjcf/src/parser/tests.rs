//! Unit tests for the MJCF parser.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::attrs::parse_vector3;
use super::parse_mjcf_str;
use crate::error::MjcfError;
use crate::types::{
    AngleUnit, CompositeType, InertiaFromGeom, MjcfActuatorType, MjcfConeType, MjcfGeomType,
    MjcfIntegrator, MjcfJacobianType, MjcfJointType, MjcfOption, MjcfSensorType, MjcfSolverType,
    MjcfTendonType, SpatialPathElement,
};
use approx::assert_relative_eq;

#[test]
fn test_parse_simple_model() {
    let xml = r#"
            <mujoco model="test_model">
                <worldbody>
                    <body name="base">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.name, "test_model");
    assert_eq!(model.worldbody.children.len(), 1);

    let base = &model.worldbody.children[0];
    assert_eq!(base.name, "base");
    assert_eq!(base.geoms.len(), 1);
    assert_eq!(base.geoms[0].geom_type, Some(MjcfGeomType::Sphere));
}

#[test]
fn test_parse_option() {
    let xml = r#"
            <mujoco model="test">
                <option timestep="0.001" gravity="0 0 -10"/>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_relative_eq!(model.option.timestep, 0.001, epsilon = 1e-10);
    assert_relative_eq!(model.option.gravity.z, -10.0, epsilon = 1e-10);
}

#[test]
fn test_parse_joint() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge" axis="0 1 0" limited="true" range="-1.57 1.57" damping="0.5"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let body = &model.worldbody.children[0];
    assert_eq!(body.joints.len(), 1);

    let joint = &body.joints[0];
    assert_eq!(joint.name, "joint1");
    assert_eq!(joint.joint_type, Some(MjcfJointType::Hinge));
    assert_eq!(joint.limited, Some(true));
    assert_eq!(joint.range, Some((-1.57, 1.57)));
    assert_relative_eq!(joint.damping.unwrap(), 0.5, epsilon = 1e-10);
    assert_relative_eq!(joint.axis.unwrap().y, 1.0, epsilon = 1e-10);
}

#[test]
fn test_parse_inertial() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <inertial pos="0 0 0.1" mass="2.0" diaginertia="0.1 0.2 0.3"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let body = &model.worldbody.children[0];
    let inertial = body.inertial.as_ref().expect("should have inertial");

    assert_relative_eq!(inertial.mass, 2.0, epsilon = 1e-10);
    assert_relative_eq!(inertial.pos.z, 0.1, epsilon = 1e-10);

    let diag = inertial.diaginertia.expect("should have diaginertia");
    assert_relative_eq!(diag.x, 0.1, epsilon = 1e-10);
    assert_relative_eq!(diag.y, 0.2, epsilon = 1e-10);
    assert_relative_eq!(diag.z, 0.3, epsilon = 1e-10);
}

#[test]
fn test_parse_nested_bodies() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="base" pos="0 0 0.1">
                        <geom type="box" size="0.1 0.1 0.1"/>
                        <body name="child" pos="0 0 0.2">
                            <joint name="j1" type="hinge"/>
                            <geom type="sphere" size="0.05"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let base = &model.worldbody.children[0];
    assert_eq!(base.name, "base");
    assert_eq!(base.children.len(), 1);

    let child = &base.children[0];
    assert_eq!(child.name, "child");
    assert_eq!(child.joints.len(), 1);
    assert_relative_eq!(child.pos.z, 0.2, epsilon = 1e-10);
}

#[test]
fn test_parse_actuators() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="joint1" gear="100"/>
                    <position name="servo1" joint="joint1" kp="10"/>
                </actuator>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.actuators.len(), 2);

    let motor = &model.actuators[0];
    assert_eq!(motor.name, "motor1");
    assert_eq!(motor.actuator_type, MjcfActuatorType::Motor);
    assert_eq!(motor.joint, Some("joint1".to_string()));
    assert_relative_eq!(motor.gear[0], 100.0, epsilon = 1e-10);

    let servo = &model.actuators[1];
    assert_eq!(servo.name, "servo1");
    assert_eq!(servo.actuator_type, MjcfActuatorType::Position);
    assert_relative_eq!(servo.kp, 10.0, epsilon = 1e-10);
}

#[test]
fn test_parse_geom_fromto() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <geom type="capsule" fromto="0 0 0 0 0 0.5" size="0.05"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let geom = &model.worldbody.children[0].geoms[0];
    assert_eq!(geom.geom_type.unwrap(), MjcfGeomType::Capsule);

    let fromto = geom.fromto.expect("should have fromto");
    assert_relative_eq!(fromto[5], 0.5, epsilon = 1e-10);
}

#[test]
fn test_unknown_joint_type() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint type="invalid"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

    let result = parse_mjcf_str(xml);
    assert!(matches!(result, Err(MjcfError::UnknownJointType(_))));
}

#[test]
fn test_parse_vector3() {
    let v = parse_vector3("1.0 2.0 3.0").expect("should parse");
    assert_relative_eq!(v.x, 1.0, epsilon = 1e-10);
    assert_relative_eq!(v.y, 2.0, epsilon = 1e-10);
    assert_relative_eq!(v.z, 3.0, epsilon = 1e-10);

    // With extra whitespace
    let v = parse_vector3("  1   2   3  ").expect("should parse");
    assert_relative_eq!(v.x, 1.0, epsilon = 1e-10);
}

// ========================================================================
// Option parsing tests
// ========================================================================

#[test]
fn test_parse_option_all_attributes() {
    let xml = r#"
            <mujoco model="test">
                <option
                    timestep="0.005"
                    integrator="RK4"
                    solver="PGS"
                    iterations="200"
                    tolerance="1e-6"
                    ls_iterations="100"
                    noslip_iterations="10"
                    ccd_iterations="25"
                    sdf_iterations="15"
                    sdf_initpoints="60"
                    cone="elliptic"
                    jacobian="sparse"
                    impratio="2.0"
                    gravity="0 0 -10.5"
                    wind="1 2 0"
                    magnetic="0.1 0.2 0.3"
                    density="1.2"
                    viscosity="0.01"
                    nconmax="500"
                    njmax="1000"
                    o_margin="0.002"
                />
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let opt = &model.option;

    // Core simulation
    assert_relative_eq!(opt.timestep, 0.005, epsilon = 1e-10);
    assert_eq!(opt.integrator, MjcfIntegrator::RK4);

    // Solver configuration
    assert_eq!(opt.solver, MjcfSolverType::PGS);
    assert_eq!(opt.iterations, 200);
    assert_relative_eq!(opt.tolerance, 1e-6, epsilon = 1e-15);
    assert_eq!(opt.ls_iterations, 100);
    assert_eq!(opt.noslip_iterations, 10);
    assert_eq!(opt.ccd_iterations, 25);
    assert_eq!(opt.sdf_iterations, 15);
    assert_eq!(opt.sdf_initpoints, 60);

    // Contact configuration
    assert_eq!(opt.cone, MjcfConeType::Elliptic);
    assert_eq!(opt.jacobian, MjcfJacobianType::Sparse);
    assert_relative_eq!(opt.impratio, 2.0, epsilon = 1e-10);

    // Physics environment
    assert_relative_eq!(opt.gravity.z, -10.5, epsilon = 1e-10);
    assert_relative_eq!(opt.wind.x, 1.0, epsilon = 1e-10);
    assert_relative_eq!(opt.wind.y, 2.0, epsilon = 1e-10);
    assert_relative_eq!(opt.magnetic.x, 0.1, epsilon = 1e-10);
    assert_relative_eq!(opt.density, 1.2, epsilon = 1e-10);
    assert_relative_eq!(opt.viscosity, 0.01, epsilon = 1e-10);

    // Constraint limits
    assert_eq!(opt.nconmax, 500);
    assert_eq!(opt.njmax, 1000);

    // Overrides
    assert_relative_eq!(opt.o_margin, 0.002, epsilon = 1e-10);
}

#[test]
fn test_sdf_option_defaults() {
    // When sdf_iterations and sdf_initpoints are not specified, defaults apply.
    let xml = r#"
            <mujoco model="test">
                <option timestep="0.001"/>
                <worldbody/>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(
        model.option.sdf_iterations, 10,
        "sdf_iterations default should be 10"
    );
    assert_eq!(
        model.option.sdf_initpoints, 40,
        "sdf_initpoints default should be 40"
    );
}

#[test]
fn test_sdf_option_nondefault_parsing() {
    // Verify non-default sdf_iterations and sdf_initpoints parse correctly.
    let xml = r#"
            <mujoco model="test">
                <option sdf_iterations="20" sdf_initpoints="80"/>
                <worldbody/>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.option.sdf_iterations, 20);
    assert_eq!(model.option.sdf_initpoints, 80);
}

#[test]
fn test_sdf_options_reach_model() {
    // Verify sdf_iterations and sdf_initpoints flow through the full
    // parse → build → Model pipeline.
    use crate::builder::model_from_mjcf;

    // Non-default values
    let xml = r#"
            <mujoco model="test">
                <option sdf_iterations="7" sdf_initpoints="50"/>
                <worldbody>
                    <body>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let mjcf = parse_mjcf_str(xml).expect("should parse");
    let model = model_from_mjcf(&mjcf, None).expect("should build");
    assert_eq!(model.sdf_iterations, 7, "sdf_iterations should reach Model");
    assert_eq!(
        model.sdf_initpoints, 50,
        "sdf_initpoints should reach Model"
    );

    // Default values
    let xml_defaults = r#"
            <mujoco model="test">
                <worldbody>
                    <body>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let mjcf_d = parse_mjcf_str(xml_defaults).expect("should parse");
    let model_d = model_from_mjcf(&mjcf_d, None).expect("should build");
    assert_eq!(
        model_d.sdf_iterations, 10,
        "default sdf_iterations should be 10"
    );
    assert_eq!(
        model_d.sdf_initpoints, 40,
        "default sdf_initpoints should be 40"
    );
}

/// T5: AC5 — ccd_iterations non-default value reaches Model
#[test]
fn test_ccd_iterations_nondefault_reaches_model() {
    use crate::builder::model_from_mjcf;

    let xml = r#"
            <mujoco model="test">
                <option ccd_iterations="10"/>
                <worldbody>
                    <body>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let mjcf = parse_mjcf_str(xml).expect("should parse");
    let model = model_from_mjcf(&mjcf, None).expect("should build");
    assert_eq!(
        model.ccd_iterations, 10,
        "ccd_iterations should reach Model"
    );
}

/// T6: AC6 — ccd_tolerance non-default value reaches Model
#[test]
fn test_ccd_tolerance_nondefault_reaches_model() {
    use crate::builder::model_from_mjcf;

    let xml = r#"
            <mujoco model="test">
                <option ccd_tolerance="1e-8"/>
                <worldbody>
                    <body>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let mjcf = parse_mjcf_str(xml).expect("should parse");
    let model = model_from_mjcf(&mjcf, None).expect("should build");
    assert!(
        (model.ccd_tolerance - 1e-8).abs() < 1e-15,
        "ccd_tolerance should reach Model, got {}",
        model.ccd_tolerance
    );
}

/// T7: AC7 — ccd_iterations default is 35
#[test]
fn test_ccd_iterations_default_is_35() {
    use crate::builder::model_from_mjcf;

    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let mjcf = parse_mjcf_str(xml).expect("should parse");
    let model = model_from_mjcf(&mjcf, None).expect("should build");
    assert_eq!(
        model.ccd_iterations, 35,
        "default ccd_iterations should be 35"
    );
    assert!(
        (model.ccd_tolerance - 1e-6).abs() < 1e-15,
        "default ccd_tolerance should be 1e-6"
    );
}

#[test]
fn test_parse_compiler_all_attributes() {
    let xml = r#"
            <mujoco model="test">
                <compiler
                    angle="radian"
                    eulerseq="ZYX"
                    meshdir="meshes/"
                    texturedir="textures/"
                    assetdir="assets/"
                    autolimits="false"
                    inertiafromgeom="true"
                    boundmass="0.01"
                    boundinertia="0.001"
                    balanceinertia="true"
                    settotalmass="10.0"
                    strippath="true"
                    discardvisual="true"
                    fusestatic="true"
                    coordinate="local"
                    fitaabb="true"
                    usethread="false"
                    alignfree="true"
                />
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let c = &model.compiler;

    assert_eq!(c.angle, AngleUnit::Radian);
    assert_eq!(c.eulerseq, "ZYX");
    assert_eq!(c.meshdir.as_deref(), Some("meshes/"));
    assert_eq!(c.texturedir.as_deref(), Some("textures/"));
    assert_eq!(c.assetdir.as_deref(), Some("assets/"));
    assert!(!c.autolimits);
    assert_eq!(c.inertiafromgeom, InertiaFromGeom::True);
    assert_relative_eq!(c.boundmass, 0.01, epsilon = 1e-10);
    assert_relative_eq!(c.boundinertia, 0.001, epsilon = 1e-10);
    assert!(c.balanceinertia);
    assert_relative_eq!(c.settotalmass, 10.0, epsilon = 1e-10);
    assert!(c.strippath);
    assert!(c.discardvisual);
    assert!(c.fusestatic);
    assert!(c.fitaabb);
    assert!(!c.usethread);
    assert!(c.alignfree);
}

#[test]
fn test_parse_compiler_defaults() {
    let xml = r#"
            <mujoco model="test">
                <compiler/>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let c = &model.compiler;

    assert_eq!(c.angle, AngleUnit::Degree);
    assert_eq!(c.eulerseq, "xyz");
    assert!(c.meshdir.is_none());
    assert!(c.texturedir.is_none());
    assert!(c.assetdir.is_none());
    assert!(c.autolimits);
    assert_eq!(c.inertiafromgeom, InertiaFromGeom::Auto);
    assert_relative_eq!(c.boundmass, 0.0, epsilon = 1e-10);
    assert_relative_eq!(c.boundinertia, 0.0, epsilon = 1e-10);
    assert!(!c.balanceinertia);
    assert_relative_eq!(c.settotalmass, -1.0, epsilon = 1e-10);
    assert!(!c.strippath);
    assert!(!c.discardvisual);
    assert!(!c.fusestatic);
}

#[test]
fn test_parse_compiler_coordinate_global_rejected() {
    let xml = r#"
            <mujoco model="test">
                <compiler coordinate="global"/>
                <worldbody/>
            </mujoco>
        "#;

    let result = parse_mjcf_str(xml);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("coordinate='global'"),
        "Error should mention coordinate='global', got: {err}"
    );
}

#[test]
fn test_parse_compiler_invalid_eulerseq() {
    let xml = r#"
            <mujoco model="test">
                <compiler eulerseq="ab"/>
                <worldbody/>
            </mujoco>
        "#;

    let result = parse_mjcf_str(xml);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("eulerseq"),
        "Error should mention eulerseq, got: {err}"
    );
}

#[test]
fn test_parse_compiler_with_children() {
    // Compiler element with children should be parsed (attrs) and children skipped
    let xml = r#"
            <mujoco model="test">
                <compiler angle="radian">
                    <lengthrange/>
                </compiler>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.compiler.angle, AngleUnit::Radian);
}

#[test]
fn test_parse_option_integrator_types() {
    for (name, expected) in [
        ("Euler", MjcfIntegrator::Euler),
        ("RK4", MjcfIntegrator::RK4),
        ("implicit", MjcfIntegrator::Implicit),
        ("implicitfast", MjcfIntegrator::ImplicitFast),
        ("implicitspringdamper", MjcfIntegrator::ImplicitSpringDamper),
    ] {
        let xml = format!(
            r#"
                <mujoco model="test">
                    <option integrator="{name}"/>
                    <worldbody/>
                </mujoco>
            "#
        );

        let model = parse_mjcf_str(&xml).expect("should parse");
        assert_eq!(model.option.integrator, expected, "integrator: {name}");
    }
}

#[test]
fn test_parse_option_solver_types() {
    for (name, expected) in [
        ("PGS", MjcfSolverType::PGS),
        ("CG", MjcfSolverType::CG),
        ("Newton", MjcfSolverType::Newton),
    ] {
        let xml = format!(
            r#"
                <mujoco model="test">
                    <option solver="{name}"/>
                    <worldbody/>
                </mujoco>
            "#
        );

        let model = parse_mjcf_str(&xml).expect("should parse");
        assert_eq!(model.option.solver, expected, "solver: {name}");
    }
}

#[test]
fn test_parse_option_with_flag() {
    let xml = r#"
            <mujoco model="test">
                <option timestep="0.002">
                    <flag
                        contact="disable"
                        gravity="disable"
                        constraint="enable"
                        warmstart="disable"
                        energy="enable"
                        island="enable"
                    />
                </option>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let flag = &model.option.flag;

    assert!(!flag.contact);
    assert!(!flag.gravity);
    assert!(flag.constraint);
    assert!(!flag.warmstart);
    assert!(flag.energy);
    assert!(flag.island);
}

#[test]
fn test_parse_option_self_closing_with_defaults() {
    let xml = r#"
            <mujoco model="test">
                <option timestep="0.001"/>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let opt = &model.option;

    // Specified value
    assert_relative_eq!(opt.timestep, 0.001, epsilon = 1e-10);

    // Defaults
    assert_eq!(opt.integrator, MjcfIntegrator::Euler);
    assert_eq!(opt.solver, MjcfSolverType::Newton);
    assert_eq!(opt.iterations, 100);
    assert_eq!(opt.cone, MjcfConeType::Pyramidal);
    assert!(opt.flag.contact);
    assert!(opt.flag.gravity);
}

#[test]
fn test_parse_option_override_arrays() {
    let xml = r#"
            <mujoco model="test">
                <option
                    o_solimp="0.9 0.95 0.001 0.5 2"
                    o_solref="0.02 1"
                    o_friction="1 0.005 0.0001 0.0001 0.0001"
                />
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let opt = &model.option;

    let solimp = opt.o_solimp.expect("should have o_solimp");
    assert_relative_eq!(solimp[0], 0.9, epsilon = 1e-10);
    assert_relative_eq!(solimp[1], 0.95, epsilon = 1e-10);
    assert_relative_eq!(solimp[2], 0.001, epsilon = 1e-10);

    let solref = opt.o_solref.expect("should have o_solref");
    assert_relative_eq!(solref[0], 0.02, epsilon = 1e-10);
    assert_relative_eq!(solref[1], 1.0, epsilon = 1e-10);

    let friction = opt.o_friction.expect("should have o_friction");
    assert_relative_eq!(friction[0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(friction[1], 0.005, epsilon = 1e-10);
}

#[test]
fn test_option_default_values() {
    let xml = r#"
            <mujoco model="test">
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let opt = &model.option;

    // Check MuJoCo defaults
    assert_relative_eq!(opt.timestep, 0.002, epsilon = 1e-10);
    assert_relative_eq!(opt.gravity.z, -9.81, epsilon = 1e-10);
    assert_eq!(opt.integrator, MjcfIntegrator::Euler);
    assert_eq!(opt.solver, MjcfSolverType::Newton);
    assert_eq!(opt.iterations, 100);
    assert_relative_eq!(opt.tolerance, 1e-8, epsilon = 1e-15);
    assert_eq!(opt.ls_iterations, 50);
    assert_eq!(opt.ccd_iterations, 35);
    assert_eq!(opt.cone, MjcfConeType::Pyramidal);
    assert_eq!(opt.jacobian, MjcfJacobianType::Dense);
    assert_relative_eq!(opt.impratio, 1.0, epsilon = 1e-10);
    assert_relative_eq!(opt.density, 0.0, epsilon = 1e-10);
    assert!(opt.flag.contact);
    assert!(opt.flag.gravity);
    assert!(opt.flag.warmstart);
}

#[test]
fn test_option_helper_methods() {
    let mut opt = MjcfOption::default();

    // Gravity enabled by default
    assert!(opt.gravity_enabled());
    assert!(opt.contacts_enabled());

    // Disable via flag
    opt.flag.gravity = false;
    assert!(!opt.gravity_enabled());

    opt.flag.contact = false;
    assert!(!opt.contacts_enabled());

    // Effective margin
    assert!(opt.effective_margin().is_none()); // Default is negative
    opt.o_margin = 0.001;
    assert_eq!(opt.effective_margin(), Some(0.001));
}

// ========================================================================
// Default parsing tests
// ========================================================================

#[test]
fn test_parse_default_joint() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <joint damping="0.5" stiffness="10.0" armature="0.01"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    let joint_defaults = default.joint.as_ref().expect("should have joint defaults");
    assert_relative_eq!(joint_defaults.damping.unwrap(), 0.5, epsilon = 1e-10);
    assert_relative_eq!(joint_defaults.stiffness.unwrap(), 10.0, epsilon = 1e-10);
    assert_relative_eq!(joint_defaults.armature.unwrap(), 0.01, epsilon = 1e-10);
}

#[test]
fn test_parse_default_geom() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <geom type="box" density="500" friction="0.8 0.01 0.001" rgba="1 0 0 1"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    let geom_defaults = default.geom.as_ref().expect("should have geom defaults");
    assert_eq!(geom_defaults.geom_type, Some(MjcfGeomType::Box));
    assert_relative_eq!(geom_defaults.density.unwrap(), 500.0, epsilon = 1e-10);
    let friction = geom_defaults.friction.unwrap();
    assert_relative_eq!(friction.x, 0.8, epsilon = 1e-10);
    let rgba = geom_defaults.rgba.unwrap();
    assert_relative_eq!(rgba.x, 1.0, epsilon = 1e-10);
    assert_relative_eq!(rgba.y, 0.0, epsilon = 1e-10);
}

#[test]
fn test_parse_default_actuator() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <motor gear="100" ctrlrange="-1 1" kp="50" kv="5" ctrllimited="true"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    let actuator_defaults = default
        .actuator
        .as_ref()
        .expect("should have actuator defaults");
    assert_relative_eq!(actuator_defaults.gear.unwrap()[0], 100.0, epsilon = 1e-10);
    assert_eq!(actuator_defaults.ctrlrange, Some((-1.0, 1.0)));
    assert_relative_eq!(actuator_defaults.kp.unwrap(), 50.0, epsilon = 1e-10);
    assert_relative_eq!(actuator_defaults.kv.unwrap(), 5.0, epsilon = 1e-10);
    assert_eq!(actuator_defaults.ctrllimited, Some(true));
}

#[test]
fn test_parse_default_tendon() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <tendon stiffness="1000" damping="10" width="0.01" limited="true" range="0 0.5"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    let tendon_defaults = default
        .tendon
        .as_ref()
        .expect("should have tendon defaults");
    assert_relative_eq!(tendon_defaults.stiffness.unwrap(), 1000.0, epsilon = 1e-10);
    assert_relative_eq!(tendon_defaults.damping.unwrap(), 10.0, epsilon = 1e-10);
    assert_relative_eq!(tendon_defaults.width.unwrap(), 0.01, epsilon = 1e-10);
    assert_eq!(tendon_defaults.limited, Some(true));
    assert_eq!(tendon_defaults.range, Some((0.0, 0.5)));
}

#[test]
fn test_parse_default_sensor() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <sensor noise="0.01" cutoff="100"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    let sensor_defaults = default
        .sensor
        .as_ref()
        .expect("should have sensor defaults");
    assert_relative_eq!(sensor_defaults.noise.unwrap(), 0.01, epsilon = 1e-10);
    assert_relative_eq!(sensor_defaults.cutoff.unwrap(), 100.0, epsilon = 1e-10);
}

#[test]
fn test_parse_nested_defaults() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <joint damping="0.1"/>
                    <default class="arm">
                        <joint damping="0.5" armature="0.01"/>
                        <geom rgba="0.8 0.2 0.2 1"/>
                    </default>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 2);

    // Root default
    let root = model
        .defaults
        .iter()
        .find(|d| d.class.is_empty())
        .expect("root");
    assert!(root.parent_class.is_none());
    let root_joint = root.joint.as_ref().expect("root joint");
    assert_relative_eq!(root_joint.damping.unwrap(), 0.1, epsilon = 1e-10);

    // Arm class
    let arm = model
        .defaults
        .iter()
        .find(|d| d.class == "arm")
        .expect("arm");
    assert_eq!(arm.parent_class, Some(String::new()));
    let arm_joint = arm.joint.as_ref().expect("arm joint");
    assert_relative_eq!(arm_joint.damping.unwrap(), 0.5, epsilon = 1e-10);
    assert_relative_eq!(arm_joint.armature.unwrap(), 0.01, epsilon = 1e-10);
    let arm_geom = arm.geom.as_ref().expect("arm geom");
    assert_relative_eq!(arm_geom.rgba.unwrap().x, 0.8, epsilon = 1e-10);
}

#[test]
fn test_parse_default_self_closing() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <joint damping="0.5"/>
                    <geom density="500"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    assert!(default.joint.is_some());
    assert!(default.geom.is_some());
}

#[test]
fn test_parse_default_mesh() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <mesh scale="0.001 0.001 0.001"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    let mesh_defaults = default.mesh.as_ref().expect("should have mesh defaults");
    let scale = mesh_defaults.scale.unwrap();
    assert_relative_eq!(scale.x, 0.001, epsilon = 1e-10);
    assert_relative_eq!(scale.y, 0.001, epsilon = 1e-10);
    assert_relative_eq!(scale.z, 0.001, epsilon = 1e-10);
}

#[test]
fn test_parse_default_site() {
    let xml = r#"
            <mujoco model="test">
                <default>
                    <site type="sphere" size="0.02" rgba="0 1 0 1"/>
                </default>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.defaults.len(), 1);

    let default = &model.defaults[0];
    let site_defaults = default.site.as_ref().expect("should have site defaults");
    assert_eq!(site_defaults.site_type, Some("sphere".to_string()));
    let size = site_defaults.size.as_ref().unwrap();
    assert_relative_eq!(size[0], 0.02, epsilon = 1e-10);
    let rgba = site_defaults.rgba.unwrap();
    assert_relative_eq!(rgba.y, 1.0, epsilon = 1e-10);
}

// ========================================================================
// Asset parsing tests (mesh)
// ========================================================================

#[test]
fn test_parse_mesh_asset_from_file() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="robot_body" file="meshes/body.stl"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.meshes.len(), 1);

    let mesh = &model.meshes[0];
    assert_eq!(mesh.name, "robot_body");
    assert_eq!(mesh.file, Some("meshes/body.stl".to_string()));
    // Default scale is None (not explicitly set)
    assert!(mesh.scale.is_none());
}

#[test]
fn test_parse_mesh_asset_with_scale() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="scaled_mesh" file="model.obj" scale="0.001 0.001 0.001"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.meshes.len(), 1);

    let mesh = &model.meshes[0];
    assert_eq!(mesh.name, "scaled_mesh");
    let scale = mesh.scale.expect("scale should be set");
    assert_relative_eq!(scale.x, 0.001, epsilon = 1e-10);
    assert_relative_eq!(scale.y, 0.001, epsilon = 1e-10);
    assert_relative_eq!(scale.z, 0.001, epsilon = 1e-10);
}

#[test]
fn test_parse_mesh_asset_with_embedded_vertices() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="triangle" vertex="0 0 0 1 0 0 0 1 0"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.meshes.len(), 1);

    let mesh = &model.meshes[0];
    assert_eq!(mesh.name, "triangle");
    assert!(mesh.file.is_none());
    assert!(mesh.has_embedded_data());
    assert_eq!(mesh.vertex_count(), 3);

    let vertices = mesh.vertex.as_ref().unwrap();
    assert_eq!(vertices.len(), 9); // 3 vertices * 3 components
    assert_relative_eq!(vertices[0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(vertices[3], 1.0, epsilon = 1e-10);
    assert_relative_eq!(vertices[7], 1.0, epsilon = 1e-10);
}

#[test]
fn test_parse_multiple_mesh_assets() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="mesh1" file="a.stl"/>
                    <mesh name="mesh2" file="b.stl" scale="2 2 2"/>
                    <mesh name="mesh3" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.meshes.len(), 3);

    assert_eq!(model.meshes[0].name, "mesh1");
    assert_eq!(model.meshes[1].name, "mesh2");
    assert_eq!(model.meshes[2].name, "mesh3");

    assert_relative_eq!(
        model.meshes[1].scale.expect("scale should be set").x,
        2.0,
        epsilon = 1e-10
    );
    assert_eq!(model.meshes[2].vertex_count(), 4);
}

// T9: maxhullvert parsing → AC9
#[test]
fn test_parse_maxhullvert() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="box" maxhullvert="10" vertex="0 0 0 1 0 0 1 1 0 0 1 0 0 0 1 1 0 1 1 1 1 0 1 1"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.meshes[0].maxhullvert, Some(10));
}

// T10: maxhullvert validation error → AC10
#[test]
fn test_parse_maxhullvert_invalid() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="box" maxhullvert="2" vertex="0 0 0 1 0 0 1 1 0 0 1 0 0 0 1 1 0 1 1 1 1 0 1 1"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;
    let err = parse_mjcf_str(xml).expect_err("should fail");
    let msg = format!("{err}");
    assert!(
        msg.contains("maxhullvert must be larger than 3"),
        "error should mention maxhullvert, got: {msg}"
    );
}

// T12: Hull available after model build → AC12
#[test]
fn test_hull_available_after_build() {
    // Use a tetrahedron mesh (4 vertices — minimum for a hull)
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="tet" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 3 1 2 3 0 2 3"/>
                </asset>
                <worldbody>
                    <body name="b">
                        <geom type="mesh" mesh="tet"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let model = crate::load_model(xml).expect("should build");
    assert!(
        model.mesh_data[0].convex_hull().is_some(),
        "hull should be computed at build time"
    );
}

// T18: maxhullvert=-1 parse → AC9
#[test]
fn test_parse_maxhullvert_minus_one() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="box" maxhullvert="-1" vertex="0 0 0 1 0 0 1 1 0 0 1 0 0 0 1 1 0 1 1 1 1 0 1 1"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(
        model.meshes[0].maxhullvert, None,
        "maxhullvert=-1 should map to None"
    );
}

#[test]
fn test_parse_geom_with_mesh_type() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="cube" vertex="0 0 0 1 0 0 1 1 0 0 1 0 0 0 1 1 0 1 1 1 1 0 1 1"/>
                </asset>
                <worldbody>
                    <body name="link">
                        <geom type="mesh" mesh="cube"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.meshes.len(), 1);
    assert_eq!(model.worldbody.children.len(), 1);

    let body = &model.worldbody.children[0];
    assert_eq!(body.geoms.len(), 1);

    let geom = &body.geoms[0];
    assert_eq!(geom.geom_type.unwrap(), MjcfGeomType::Mesh);
    assert_eq!(geom.mesh, Some("cube".to_string()));
}

#[test]
fn test_model_mesh_lookup() {
    let xml = r#"
            <mujoco model="test">
                <asset>
                    <mesh name="mesh_a" file="a.stl"/>
                    <mesh name="mesh_b" file="b.stl"/>
                </asset>
                <worldbody/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");

    assert!(model.mesh("mesh_a").is_some());
    assert!(model.mesh("mesh_b").is_some());
    assert!(model.mesh("nonexistent").is_none());

    let mesh_a = model.mesh("mesh_a").unwrap();
    assert_eq!(mesh_a.file, Some("a.stl".to_string()));
}

// ========================================================================
// Equality constraint parsing tests
// ========================================================================

#[test]
fn test_parse_connect_constraint_basic() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="body1" pos="0 0 0">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="body2" pos="1 0 0">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <equality>
                    <connect body1="body1" body2="body2" anchor="0.5 0 0"/>
                </equality>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.equality.connects.len(), 1);

    let connect = &model.equality.connects[0];
    assert_eq!(connect.body1, "body1");
    assert_eq!(connect.body2, Some("body2".to_string()));
    assert_relative_eq!(connect.anchor.x, 0.5, epsilon = 1e-10);
    assert_relative_eq!(connect.anchor.y, 0.0, epsilon = 1e-10);
    assert_relative_eq!(connect.anchor.z, 0.0, epsilon = 1e-10);
    assert_eq!(connect.active, None);
}

#[test]
fn test_parse_connect_constraint_with_name() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1"/>
                    <body name="link2"/>
                </worldbody>
                <equality>
                    <connect name="ball_joint" body1="link1" body2="link2"/>
                </equality>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let connect = &model.equality.connects[0];
    assert_eq!(connect.name, Some("ball_joint".to_string()));
}

#[test]
fn test_parse_connect_constraint_to_world() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="floating_body"/>
                </worldbody>
                <equality>
                    <connect body1="floating_body" anchor="0 0 1"/>
                </equality>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let connect = &model.equality.connects[0];
    assert_eq!(connect.body1, "floating_body");
    assert!(connect.body2.is_none()); // Defaults to world
    assert_relative_eq!(connect.anchor.z, 1.0, epsilon = 1e-10);
}

#[test]
fn test_parse_connect_constraint_with_solver_params() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="body1"/>
                    <body name="body2"/>
                </worldbody>
                <equality>
                    <connect body1="body1" body2="body2"
                             solref="0.02 1"
                             solimp="0.9 0.95 0.001 0.5 2"/>
                </equality>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let connect = &model.equality.connects[0];

    let solref = connect.solref.expect("should have solref");
    assert_relative_eq!(solref[0], 0.02, epsilon = 1e-10);
    assert_relative_eq!(solref[1], 1.0, epsilon = 1e-10);

    let solimp = connect.solimp.expect("should have solimp");
    assert_relative_eq!(solimp[0], 0.9, epsilon = 1e-10);
    assert_relative_eq!(solimp[1], 0.95, epsilon = 1e-10);
    assert_relative_eq!(solimp[2], 0.001, epsilon = 1e-10);
}

#[test]
fn test_parse_connect_constraint_inactive() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="body1"/>
                    <body name="body2"/>
                </worldbody>
                <equality>
                    <connect body1="body1" body2="body2" active="false"/>
                </equality>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let connect = &model.equality.connects[0];
    assert_eq!(connect.active, Some(false));
}

#[test]
fn test_parse_multiple_connect_constraints() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="a"/>
                    <body name="b"/>
                    <body name="c"/>
                </worldbody>
                <equality>
                    <connect name="c1" body1="a" body2="b"/>
                    <connect name="c2" body1="b" body2="c"/>
                    <connect name="c3" body1="c" anchor="0 0 0"/>
                </equality>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.equality.connects.len(), 3);
    assert_eq!(model.equality.connects[0].name, Some("c1".to_string()));
    assert_eq!(model.equality.connects[1].name, Some("c2".to_string()));
    assert_eq!(model.equality.connects[2].name, Some("c3".to_string()));
}

#[test]
fn test_parse_connect_constraint_missing_body1() {
    let xml = r#"
            <mujoco model="test">
                <worldbody/>
                <equality>
                    <connect body2="body2"/>
                </equality>
            </mujoco>
        "#;

    let result = parse_mjcf_str(xml);
    assert!(result.is_err());
    let err = result.unwrap_err();
    assert!(matches!(err, MjcfError::MissingAttribute { .. }));
}

#[test]
fn test_parse_empty_equality() {
    let xml = r#"
            <mujoco model="test">
                <worldbody/>
                <equality/>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert!(model.equality.is_empty());
}

#[test]
fn test_equality_len() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="a"/>
                    <body name="b"/>
                </worldbody>
                <equality>
                    <connect body1="a" body2="b"/>
                    <connect body1="b"/>
                </equality>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.equality.len(), 2);
    assert!(!model.equality.is_empty());
}

// ========================================================================
// Cylinder, Muscle, and Adhesion actuator parsing tests
// ========================================================================

#[test]
fn test_parse_cylinder_actuator() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="slide"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <cylinder name="cyl1" joint="joint1" area="0.002" timeconst="0.5" bias="1 2 3"/>
                </actuator>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.actuators.len(), 1);

    let cyl = &model.actuators[0];
    assert_eq!(cyl.name, "cyl1");
    assert_eq!(cyl.actuator_type, MjcfActuatorType::Cylinder);
    assert_eq!(cyl.joint, Some("joint1".to_string()));
    assert_relative_eq!(cyl.area, 0.002, epsilon = 1e-10);
    assert_eq!(cyl.timeconst, Some(0.5));
    assert_relative_eq!(cyl.bias[0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(cyl.bias[1], 2.0, epsilon = 1e-10);
    assert_relative_eq!(cyl.bias[2], 3.0, epsilon = 1e-10);
}

#[test]
fn test_parse_cylinder_actuator_with_diameter() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="slide"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <cylinder name="cyl1" joint="joint1" diameter="0.05"/>
                </actuator>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let cyl = &model.actuators[0];
    assert!(cyl.diameter.is_some());
    assert_relative_eq!(cyl.diameter.unwrap(), 0.05, epsilon = 1e-10);
}

#[test]
fn test_parse_muscle_actuator() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge"/>
                        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <muscle name="muscle1" joint="joint1" timeconst="0.02 0.06" range="0.8 1.1" force="500" scale="300" lmin="0.4" lmax="1.8" vmax="2.0" fpmax="1.5" fvmax="1.3"/>
                </actuator>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.actuators.len(), 1);

    let muscle = &model.actuators[0];
    assert_eq!(muscle.name, "muscle1");
    assert_eq!(muscle.actuator_type, MjcfActuatorType::Muscle);
    assert_eq!(muscle.joint, Some("joint1".to_string()));

    // Muscle-specific attributes
    assert_relative_eq!(muscle.muscle_timeconst.0, 0.02, epsilon = 1e-10);
    assert_relative_eq!(muscle.muscle_timeconst.1, 0.06, epsilon = 1e-10);
    assert_relative_eq!(muscle.range.0, 0.8, epsilon = 1e-10);
    assert_relative_eq!(muscle.range.1, 1.1, epsilon = 1e-10);
    assert_relative_eq!(muscle.force, 500.0, epsilon = 1e-10);
    assert_relative_eq!(muscle.scale, 300.0, epsilon = 1e-10);
    assert_relative_eq!(muscle.lmin, 0.4, epsilon = 1e-10);
    assert_relative_eq!(muscle.lmax, 1.8, epsilon = 1e-10);
    assert_relative_eq!(muscle.vmax, 2.0, epsilon = 1e-10);
    assert_relative_eq!(muscle.fpmax, 1.5, epsilon = 1e-10);
    assert_relative_eq!(muscle.fvmax, 1.3, epsilon = 1e-10);
}

#[test]
fn test_parse_muscle_actuator_defaults() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="joint1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <muscle name="muscle1" joint="joint1"/>
                </actuator>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let muscle = &model.actuators[0];

    // Check MuJoCo default values
    assert_relative_eq!(muscle.muscle_timeconst.0, 0.01, epsilon = 1e-10);
    assert_relative_eq!(muscle.muscle_timeconst.1, 0.04, epsilon = 1e-10);
    assert_relative_eq!(muscle.range.0, 0.75, epsilon = 1e-10);
    assert_relative_eq!(muscle.range.1, 1.05, epsilon = 1e-10);
    assert_relative_eq!(muscle.force, -1.0, epsilon = 1e-10); // Negative triggers auto computation
    assert_relative_eq!(muscle.scale, 200.0, epsilon = 1e-10);
    assert_relative_eq!(muscle.lmin, 0.5, epsilon = 1e-10);
    assert_relative_eq!(muscle.lmax, 1.6, epsilon = 1e-10);
    assert_relative_eq!(muscle.vmax, 1.5, epsilon = 1e-10);
    assert_relative_eq!(muscle.fpmax, 1.3, epsilon = 1e-10);
    assert_relative_eq!(muscle.fvmax, 1.2, epsilon = 1e-10);
}

#[test]
fn test_parse_adhesion_actuator() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="gripper">
                        <geom type="box" size="0.05 0.05 0.01"/>
                    </body>
                </worldbody>
                <actuator>
                    <adhesion name="grip1" body="gripper" gain="100" ctrlrange="0 1"/>
                </actuator>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.actuators.len(), 1);

    let adhesion = &model.actuators[0];
    assert_eq!(adhesion.name, "grip1");
    assert_eq!(adhesion.actuator_type, MjcfActuatorType::Adhesion);
    assert_eq!(adhesion.body, Some("gripper".to_string()));
    assert_relative_eq!(adhesion.gain, 100.0, epsilon = 1e-10);
    assert_eq!(adhesion.ctrlrange, Some((0.0, 1.0)));
}

#[test]
fn test_parse_mixed_actuator_types() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1">
                        <joint name="j1" type="hinge"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                    <body name="link2">
                        <joint name="j2" type="slide"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                    <body name="gripper">
                        <geom type="box" size="0.05 0.05 0.01"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="motor1" joint="j1" gear="50"/>
                    <position name="pos1" joint="j1" kp="100"/>
                    <cylinder name="cyl1" joint="j2" area="0.001"/>
                    <muscle name="mus1" joint="j1" force="200"/>
                    <adhesion name="adh1" body="gripper" gain="50"/>
                </actuator>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.actuators.len(), 5);

    assert_eq!(model.actuators[0].actuator_type, MjcfActuatorType::Motor);
    assert_eq!(model.actuators[1].actuator_type, MjcfActuatorType::Position);
    assert_eq!(model.actuators[2].actuator_type, MjcfActuatorType::Cylinder);
    assert_eq!(model.actuators[3].actuator_type, MjcfActuatorType::Muscle);
    assert_eq!(model.actuators[4].actuator_type, MjcfActuatorType::Adhesion);
}

#[test]
fn test_actuator_type_as_str() {
    assert_eq!(MjcfActuatorType::Motor.as_str(), "motor");
    assert_eq!(MjcfActuatorType::Position.as_str(), "position");
    assert_eq!(MjcfActuatorType::Velocity.as_str(), "velocity");
    assert_eq!(MjcfActuatorType::General.as_str(), "general");
    assert_eq!(MjcfActuatorType::Muscle.as_str(), "muscle");
    assert_eq!(MjcfActuatorType::Cylinder.as_str(), "cylinder");
    assert_eq!(MjcfActuatorType::Damper.as_str(), "damper");
    assert_eq!(MjcfActuatorType::Adhesion.as_str(), "adhesion");
}

// ========================================================================
// Tendon parsing tests
// ========================================================================

#[test]
fn test_parse_spatial_tendon() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1">
                        <site name="s1" pos="0 0 0"/>
                    </body>
                    <body name="link2">
                        <site name="s2" pos="0 0 0.5"/>
                    </body>
                    <body name="link3">
                        <site name="s3" pos="0 0 1"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="cable1" stiffness="1000" damping="10" width="0.005">
                        <site site="s1"/>
                        <site site="s2"/>
                        <site site="s3"/>
                    </spatial>
                </tendon>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.tendons.len(), 1);

    let tendon = &model.tendons[0];
    assert_eq!(tendon.name, "cable1");
    assert_eq!(tendon.tendon_type, MjcfTendonType::Spatial);
    assert_relative_eq!(tendon.stiffness.unwrap(), 1000.0, epsilon = 1e-10);
    assert_relative_eq!(tendon.damping.unwrap(), 10.0, epsilon = 1e-10);
    assert_relative_eq!(tendon.width.unwrap(), 0.005, epsilon = 1e-10);
    assert_eq!(tendon.path_elements.len(), 3);
    assert_eq!(
        tendon.path_elements[0],
        SpatialPathElement::Site {
            site: "s1".to_string()
        }
    );
    assert_eq!(
        tendon.path_elements[1],
        SpatialPathElement::Site {
            site: "s2".to_string()
        }
    );
    assert_eq!(
        tendon.path_elements[2],
        SpatialPathElement::Site {
            site: "s3".to_string()
        }
    );
}

#[test]
fn test_parse_fixed_tendon() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link1">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <body name="link2">
                        <joint name="j2" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <fixed name="coupling" limited="true" range="0 0.5">
                        <joint joint="j1" coef="1"/>
                        <joint joint="j2" coef="-1"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.tendons.len(), 1);

    let tendon = &model.tendons[0];
    assert_eq!(tendon.name, "coupling");
    assert_eq!(tendon.tendon_type, MjcfTendonType::Fixed);
    assert_eq!(tendon.limited, Some(true));
    assert_eq!(tendon.range, Some((0.0, 0.5)));
    assert_eq!(
        tendon.joints,
        vec![("j1".to_string(), 1.0), ("j2".to_string(), -1.0)]
    );
}

#[test]
fn test_parse_multiple_tendons() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <site name="s1" pos="0 0 0"/>
                        <site name="s2" pos="0 0 0.5"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <tendon>
                    <spatial name="t1">
                        <site site="s1"/>
                        <site site="s2"/>
                    </spatial>
                    <fixed name="t2">
                        <joint joint="j1" coef="2"/>
                    </fixed>
                </tendon>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.tendons.len(), 2);
    assert_eq!(model.tendons[0].name, "t1");
    assert_eq!(model.tendons[0].tendon_type, MjcfTendonType::Spatial);
    assert_eq!(model.tendons[1].name, "t2");
    assert_eq!(model.tendons[1].tendon_type, MjcfTendonType::Fixed);
}

#[test]
fn test_parse_tendon_with_rgba() {
    let xml = r#"
            <mujoco model="test">
                <worldbody/>
                <tendon>
                    <spatial name="colored" rgba="1 0 0 1"/>
                </tendon>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let tendon = &model.tendons[0];
    let rgba = tendon.rgba.unwrap();
    assert_relative_eq!(rgba.x, 1.0, epsilon = 1e-10);
    assert_relative_eq!(rgba.y, 0.0, epsilon = 1e-10);
    assert_relative_eq!(rgba.z, 0.0, epsilon = 1e-10);
    assert_relative_eq!(rgba.w, 1.0, epsilon = 1e-10);
}

// ========================================================================
// Sensor parsing tests
// ========================================================================

#[test]
fn test_parse_joint_sensors() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <jointpos name="j1_pos" joint="j1"/>
                    <jointvel name="j1_vel" joint="j1" noise="0.01"/>
                </sensor>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.sensors.len(), 2);

    let pos_sensor = &model.sensors[0];
    assert_eq!(pos_sensor.name, "j1_pos");
    assert_eq!(pos_sensor.sensor_type, MjcfSensorType::Jointpos);
    assert_eq!(pos_sensor.objname, Some("j1".to_string()));
    assert_relative_eq!(pos_sensor.noise, 0.0, epsilon = 1e-10);

    let vel_sensor = &model.sensors[1];
    assert_eq!(vel_sensor.name, "j1_vel");
    assert_eq!(vel_sensor.sensor_type, MjcfSensorType::Jointvel);
    assert_relative_eq!(vel_sensor.noise, 0.01, epsilon = 1e-10);
}

#[test]
fn test_parse_imu_sensors() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <site name="imu_site" pos="0 0 0"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <accelerometer name="accel" site="imu_site"/>
                    <gyro name="gyro" site="imu_site" noise="0.001"/>
                </sensor>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.sensors.len(), 2);

    let accel = &model.sensors[0];
    assert_eq!(accel.name, "accel");
    assert_eq!(accel.sensor_type, MjcfSensorType::Accelerometer);
    assert_eq!(accel.objname, Some("imu_site".to_string()));

    let gyro = &model.sensors[1];
    assert_eq!(gyro.name, "gyro");
    assert_eq!(gyro.sensor_type, MjcfSensorType::Gyro);
    assert_relative_eq!(gyro.noise, 0.001, epsilon = 1e-10);
}

#[test]
fn test_parse_force_sensors() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <site name="ft_site" pos="0 0 0"/>
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <force name="force1" site="ft_site"/>
                    <torque name="torque1" site="ft_site"/>
                    <touch name="contact1" site="ft_site"/>
                </sensor>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    assert_eq!(model.sensors.len(), 3);

    assert_eq!(model.sensors[0].sensor_type, MjcfSensorType::Force);
    assert_eq!(model.sensors[1].sensor_type, MjcfSensorType::Torque);
    assert_eq!(model.sensors[2].sensor_type, MjcfSensorType::Touch);
}

#[test]
fn test_parse_sensor_with_cutoff() {
    let xml = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="link">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <jointpos name="filtered" joint="j1" cutoff="50"/>
                </sensor>
            </mujoco>
        "#;

    let model = parse_mjcf_str(xml).expect("should parse");
    let sensor = &model.sensors[0];
    assert_relative_eq!(sensor.cutoff, 50.0, epsilon = 1e-10);
}

#[test]
fn test_sensor_type_enum() {
    // Position sensors
    assert_eq!(
        MjcfSensorType::from_str("jointpos"),
        Some(MjcfSensorType::Jointpos)
    );
    assert_eq!(
        MjcfSensorType::from_str("tendonpos"),
        Some(MjcfSensorType::Tendonpos)
    );

    // Velocity sensors
    assert_eq!(
        MjcfSensorType::from_str("jointvel"),
        Some(MjcfSensorType::Jointvel)
    );

    // IMU sensors
    assert_eq!(
        MjcfSensorType::from_str("accelerometer"),
        Some(MjcfSensorType::Accelerometer)
    );
    assert_eq!(MjcfSensorType::from_str("gyro"), Some(MjcfSensorType::Gyro));

    // Force sensors
    assert_eq!(
        MjcfSensorType::from_str("force"),
        Some(MjcfSensorType::Force)
    );
    assert_eq!(
        MjcfSensorType::from_str("torque"),
        Some(MjcfSensorType::Torque)
    );
    assert_eq!(
        MjcfSensorType::from_str("touch"),
        Some(MjcfSensorType::Touch)
    );

    // Unknown
    assert_eq!(MjcfSensorType::from_str("invalid"), None);
}

#[test]
fn test_sensor_dimensionality() {
    // Scalar sensors
    assert_eq!(MjcfSensorType::Jointpos.dim(), 1);
    assert_eq!(MjcfSensorType::Jointvel.dim(), 1);

    // 3D sensors
    assert_eq!(MjcfSensorType::Accelerometer.dim(), 3);
    assert_eq!(MjcfSensorType::Gyro.dim(), 3);
    assert_eq!(MjcfSensorType::Force.dim(), 3);

    // 4D sensors (quaternion)
    assert_eq!(MjcfSensorType::Framequat.dim(), 4);
    assert_eq!(MjcfSensorType::Ballquat.dim(), 4);
}

// ── Section merging tests ──────────────────────────────────────

#[test]
fn test_duplicate_actuator_sections_merge() {
    let model = parse_mjcf_str(
        r#"
            <mujoco model="merge_test">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j1" type="hinge"/>
                        <joint name="j2" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <motor joint="j1" name="m1"/>
                </actuator>
                <actuator>
                    <motor joint="j2" name="m2"/>
                </actuator>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(
        model.actuators.len(),
        2,
        "both actuator sections should merge"
    );
    assert_eq!(model.actuators[0].name, "m1");
    assert_eq!(model.actuators[1].name, "m2");
}

#[test]
fn test_duplicate_worldbody_sections_merge() {
    let model = parse_mjcf_str(
        r#"
            <mujoco model="merge_wb">
                <worldbody>
                    <body name="a">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(
        model.worldbody.children.len(),
        2,
        "worldbody children should merge"
    );
    assert_eq!(model.worldbody.children[0].name, "a");
    assert_eq!(model.worldbody.children[1].name, "b");
}

#[test]
fn test_duplicate_asset_sections_merge() {
    let model = parse_mjcf_str(
        r#"
            <mujoco model="merge_asset">
                <asset>
                    <mesh name="m1" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>
                </asset>
                <asset>
                    <mesh name="m2" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>
                </asset>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(model.meshes.len(), 2, "mesh assets should merge");
}

#[test]
fn test_duplicate_sensor_sections_merge() {
    let model = parse_mjcf_str(
        r#"
            <mujoco model="merge_sensor">
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <joint name="j1" type="hinge"/>
                        <joint name="j2" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <jointpos joint="j1"/>
                </sensor>
                <sensor>
                    <jointpos joint="j2"/>
                </sensor>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(model.sensors.len(), 2, "sensor sections should merge");
}

// ========================================================================
// DT-16: density attribute on <flex> silently ignored
// ========================================================================

#[test]
fn dt16_flex_density_attribute_silently_ignored() {
    // DT-16 regression: density="500" on <flex> should be ignored (non-conformant).
    // The MjcfFlex.density field stays at default 1000.0.
    let model = parse_mjcf_str(
        r#"
            <mujoco model="dt16_density">
                <deformable>
                    <flex name="test" dim="1" density="500">
                        <vertex pos="0 0 0  1 0 0"/>
                        <element data="0 1"/>
                    </flex>
                </deformable>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(model.flex.len(), 1);
    assert_relative_eq!(model.flex[0].density, 1000.0, epsilon = 1e-15);
}

#[test]
fn dt16_flex_mass_attribute_works() {
    // DT-16 positive: mass="1.5" on <flex> is the conformant API.
    let model = parse_mjcf_str(
        r#"
            <mujoco model="dt16_mass">
                <deformable>
                    <flex name="test" dim="1" mass="1.5">
                        <vertex pos="0 0 0  1 0 0"/>
                        <element data="0 1"/>
                    </flex>
                </deformable>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(model.flex.len(), 1);
    assert_eq!(model.flex[0].mass, Some(1.5));
}

// ========================================================================
// DT-90: flex friction parsed as Vector3 (tangential, torsional, rolling)
// ========================================================================

#[test]
fn dt90_flex_friction_three_values() {
    // DT-90: <contact friction="0.8 0.01 0.002"/> → all 3 components stored.
    let model = parse_mjcf_str(
        r#"
            <mujoco model="dt90_friction3">
                <deformable>
                    <flex name="test" dim="1">
                        <contact friction="0.8 0.01 0.002"/>
                        <vertex pos="0 0 0  1 0 0"/>
                        <element data="0 1"/>
                    </flex>
                </deformable>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(model.flex.len(), 1);
    let f = model.flex[0].friction;
    assert_relative_eq!(f.x, 0.8, epsilon = 1e-15);
    assert_relative_eq!(f.y, 0.01, epsilon = 1e-15);
    assert_relative_eq!(f.z, 0.002, epsilon = 1e-15);
}

#[test]
fn dt90_flex_friction_one_value_fills_defaults() {
    // DT-90: <contact friction="0.5"/> → tangential set, torsional/rolling at defaults.
    let model = parse_mjcf_str(
        r#"
            <mujoco model="dt90_friction1">
                <deformable>
                    <flex name="test" dim="1">
                        <contact friction="0.5"/>
                        <vertex pos="0 0 0  1 0 0"/>
                        <element data="0 1"/>
                    </flex>
                </deformable>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(model.flex.len(), 1);
    let f = model.flex[0].friction;
    assert_relative_eq!(f.x, 0.5, epsilon = 1e-15);
    assert_relative_eq!(f.y, 0.005, epsilon = 1e-15); // MuJoCo torsional default
    assert_relative_eq!(f.z, 0.0001, epsilon = 1e-15); // MuJoCo rolling default
}

#[test]
fn test_compiler_last_writer_wins() {
    let model = parse_mjcf_str(
        r#"
            <mujoco model="compiler_lww">
                <compiler angle="degree"/>
                <compiler angle="radian"/>
                <worldbody/>
            </mujoco>
            "#,
    )
    .unwrap();
    assert_eq!(
        model.compiler.angle,
        AngleUnit::Radian,
        "last compiler should win"
    );
}

#[test]
fn test_parse_composite_in_worldbody() {
    let xml = r#"
            <mujoco model="composite_test">
                <worldbody>
                    <composite type="cable" count="3 1 1" curve="l 0 0" size="1">
                        <joint kind="main" damping="0.01"/>
                        <geom type="capsule" size=".005"/>
                    </composite>
                </worldbody>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).unwrap();
    assert_eq!(
        model.worldbody.composites.len(),
        1,
        "should parse one composite"
    );
    let comp = &model.worldbody.composites[0];
    assert_eq!(comp.comp_type, CompositeType::Cable);
    assert_eq!(comp.count, [3, 1, 1]);
    assert!(comp.joint.is_some());
    assert!(comp.geom.is_some());
}

#[test]
fn test_parse_composite_in_body() {
    let xml = r#"
            <mujoco model="composite_test_body">
                <worldbody>
                    <body name="parent" pos="1 0 0">
                        <composite type="cable" count="3 1 1" curve="l 0 0" size="1">
                            <joint kind="main" damping="0.01"/>
                            <geom type="capsule" size=".005"/>
                        </composite>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).unwrap();
    assert_eq!(model.worldbody.children.len(), 1);
    let parent = &model.worldbody.children[0];
    assert_eq!(parent.name, "parent");
    assert_eq!(
        parent.composites.len(),
        1,
        "should parse one composite on parent body"
    );
}

// T2: Parse `<extension>` with instances and config → AC3
#[test]
fn t2_parse_extension_with_instances_and_config() {
    let xml = r#"
            <mujoco model="plugin_test">
                <extension>
                    <plugin plugin="mujoco.elasticity.cable">
                        <instance name="cable1">
                            <config key="twist" value="1e-4"/>
                            <config key="bend" value="1e-2"/>
                        </instance>
                    </plugin>
                </extension>
                <worldbody>
                    <body name="b"/>
                </worldbody>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse extension");
    assert_eq!(model.extensions.len(), 1);
    let ext = &model.extensions[0];
    assert_eq!(ext.plugins.len(), 1);
    assert_eq!(ext.plugins[0].plugin, "mujoco.elasticity.cable");
    assert_eq!(ext.plugins[0].instances.len(), 1);
    let inst = &ext.plugins[0].instances[0];
    assert_eq!(inst.name, "cable1");
    assert_eq!(inst.config.len(), 2);
    assert_eq!(inst.config[0].key, "twist");
    assert_eq!(inst.config[0].value, "1e-4");
    assert_eq!(inst.config[1].key, "bend");
    assert_eq!(inst.config[1].value, "1e-2");
}

// T4: Plugin sub-element on body → AC5
#[test]
fn t4_plugin_sub_element_on_body() {
    let xml = r#"
            <mujoco model="body_plugin_test">
                <worldbody>
                    <body name="b1">
                        <plugin plugin="test.passive"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse body plugin");
    let b1 = &model.worldbody.children[0];
    assert!(b1.plugin.is_some());
    let pref = b1.plugin.as_ref().unwrap();
    assert_eq!(pref.plugin, "test.passive");
    assert!(pref.instance.is_none());
    assert!(pref.config.is_empty());
}

// T4 variant: Plugin sub-element on actuator
#[test]
fn t4_plugin_sub_element_on_actuator() {
    let xml = r#"
            <mujoco model="actuator_plugin_test">
                <worldbody>
                    <body name="b1">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general joint="j1">
                        <plugin plugin="test.actuator"/>
                    </general>
                </actuator>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse actuator plugin");
    assert_eq!(model.actuators.len(), 1);
    assert!(model.actuators[0].plugin.is_some());
    let pref = model.actuators[0].plugin.as_ref().unwrap();
    assert_eq!(pref.plugin, "test.actuator");
}

// T4 variant: Plugin sub-element on sensor
#[test]
fn t4_plugin_sub_element_on_sensor() {
    let xml = r#"
            <mujoco model="sensor_plugin_test">
                <worldbody>
                    <body name="b1">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <user dim="3" objtype="body" objname="b1">
                        <plugin plugin="test.sensor"/>
                    </user>
                </sensor>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse sensor plugin");
    assert_eq!(model.sensors.len(), 1);
    assert!(model.sensors[0].plugin.is_some());
    let pref = model.sensors[0].plugin.as_ref().unwrap();
    assert_eq!(pref.plugin, "test.sensor");
}

// T2 variant: Plugin ref with instance attribute
#[test]
fn t2_plugin_ref_with_instance() {
    let xml = r#"
            <mujoco model="instance_ref_test">
                <extension>
                    <plugin plugin="test.cable">
                        <instance name="cable1">
                            <config key="stiffness" value="100"/>
                        </instance>
                    </plugin>
                </extension>
                <worldbody>
                    <body name="b1">
                        <plugin plugin="test.cable" instance="cable1"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse instance ref");
    let b1 = &model.worldbody.children[0];
    assert!(b1.plugin.is_some());
    let pref = b1.plugin.as_ref().unwrap();
    assert_eq!(pref.plugin, "test.cable");
    assert_eq!(pref.instance.as_deref(), Some("cable1"));
}

// T13: Plugin with empty config (supplementary)
#[test]
fn t13_extension_plugin_empty_config() {
    let xml = r#"
            <mujoco model="empty_config">
                <extension>
                    <plugin plugin="test.simple">
                        <instance name="inst1"/>
                    </plugin>
                </extension>
                <worldbody>
                    <body name="b"/>
                </worldbody>
            </mujoco>
        "#;
    let model = parse_mjcf_str(xml).expect("should parse");
    let inst = &model.extensions[0].plugins[0].instances[0];
    assert_eq!(inst.name, "inst1");
    assert!(inst.config.is_empty());
}

// T3: Duplicate config key rejected → AC4
#[test]
fn t3_duplicate_config_key_rejected() {
    let xml = r#"
            <mujoco model="dup_config">
                <extension>
                    <plugin plugin="test.plugin">
                        <instance name="inst1">
                            <config key="gain" value="100"/>
                            <config key="gain" value="200"/>
                        </instance>
                    </plugin>
                </extension>
                <worldbody>
                    <body name="b"/>
                </worldbody>
            </mujoco>
        "#;
    let result = parse_mjcf_str(xml);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("duplicate config key"),
        "expected 'duplicate config key' error, got: {err}"
    );
}
