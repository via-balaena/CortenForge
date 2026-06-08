//! Structural invariants of `parse_leg_chain` — pins the shape of the parsed
//! leg-chain [`Model`] so a structural parse regression that happens to preserve
//! the knee-sweep moment arms (and so slips past `opensim_cross_check`) is still
//! caught. These are exactly the invariants the A2 hip-unweld will deliberately
//! change, so this test is the explicit record of "what A1 parses".
//!
//! (Re-homed from the deleted `cf-msk-lib::general_ir_fk`, whose machine-zero
//! FK-vs-oracle half became tautological once the FK *is* the oracle; this
//! structural half belongs with the parser.)

use cf_osim::parse_leg_chain;

fn model() -> cf_msk_lib::Model {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    parse_leg_chain(&std::fs::read_to_string(path).expect("read gait2392.osim"))
}

#[test]
fn parse_leg_chain_builds_the_thigh_knee_shank_chain() {
    let model = model();
    assert_eq!(
        model
            .bodies
            .iter()
            .map(|b| b.name.as_str())
            .collect::<Vec<_>>(),
        ["pelvis", "femur_r", "tibia_r", "talus_r"]
    );
    assert_eq!(model.bodies[0].parent, None);
    assert_eq!(model.bodies[1].parent, Some(0));
    assert_eq!(model.bodies[2].parent, Some(1));
    assert_eq!(model.bodies[3].parent, Some(2));

    // A3: the ankle is added so the tibia has a distal endpoint — the talus
    // `location_in_parent` (in the tibia frame) is the dialable tibia length.
    assert!(
        (model.bodies[3].location_in_parent - nalgebra::Vector3::new(0.0, -0.42506489, 0.0)).norm()
            < 1e-9,
        "talus offset = tibia length"
    );

    // A2: the hip is UNWELDED — the femur carries the hip CustomJoint: three
    // rotation DOFs (flexion/adduction/rotation) + three (zero) translation axes.
    let femur = &model.bodies[1].joint;
    assert_eq!(
        femur.iter().filter(|a| a.rotation).count(),
        3,
        "hip rotations"
    );
    assert_eq!(
        femur.iter().filter(|a| !a.rotation).count(),
        3,
        "hip translations"
    );

    // tibia: three rotation axes (one driven flexion + two zero) + three
    // translation axes (two coupled splines + one zero) — the gait2392 knee.
    let tibia = &model.bodies[2].joint;
    assert_eq!(tibia.iter().filter(|a| a.rotation).count(), 3);
    assert_eq!(tibia.iter().filter(|a| !a.rotation).count(), 3);

    // talus: the ankle hinge (one driven rotation about the oblique talocrural
    // axis + two zero constant rotations) + three (zero) constant translations.
    let talus = &model.bodies[3].joint;
    assert_eq!(talus.iter().filter(|a| a.rotation).count(), 3);
    assert_eq!(talus.iter().filter(|a| !a.rotation).count(), 3);

    // Coordinates: the three hip DOFs (proximal), then the knee, then the ankle.
    let coords: Vec<&str> = model.coordinates.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(
        coords,
        [
            "hip_flexion_r",
            "hip_adduction_r",
            "hip_rotation_r",
            "knee_angle_r",
            "ankle_angle_r"
        ]
    );
}
