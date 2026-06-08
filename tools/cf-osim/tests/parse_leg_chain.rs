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
fn parse_leg_chain_builds_the_knee_chain() {
    let model = model();
    assert_eq!(
        model
            .bodies
            .iter()
            .map(|b| b.name.as_str())
            .collect::<Vec<_>>(),
        ["pelvis", "femur_r", "tibia_r"]
    );
    assert_eq!(model.bodies[0].parent, None);
    assert_eq!(model.bodies[1].parent, Some(0));
    assert_eq!(model.bodies[2].parent, Some(1));
    // A1: the hip is welded at neutral (femur placed by the hip offset, no DOFs).
    // A2 unwelds this — when it does, this assertion is the deliberate change site.
    assert!(model.bodies[1].joint.is_empty(), "hip is welded at neutral");
    // tibia: three rotation axes (one driven flexion + two zero) + three
    // translation axes (two coupled splines + one zero) — the gait2392 knee.
    let tibia = &model.bodies[2].joint;
    assert_eq!(tibia.iter().filter(|a| a.rotation).count(), 3);
    assert_eq!(tibia.iter().filter(|a| !a.rotation).count(), 3);
    // Only the knee's coordinate is free for A1 (the hip is welded).
    assert_eq!(model.coordinates.len(), 1);
    assert_eq!(model.coordinates[0].name, "knee_angle_r");
}
