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

/// Every leg-chain body carries its real gait2392 mass distribution. The femur and
/// tibia are read directly (diagonal, zero products); the `talus_r` body is the
/// LUMPED FOOT (talus + calcn + toes composed about the talus frame), since OpenSim
/// locks subtalar/mtp and the twin has one foot body. These are the values the
/// forward-dynamics gate used to inject — pinning them here is what lets the gate
/// stop injecting.
#[test]
fn parse_leg_chain_reads_real_segment_inertias() {
    let model = model();
    let iner = |name: &str| {
        model
            .bodies
            .iter()
            .find(|b| b.name == name)
            .unwrap()
            .inertia
            .expect("body carries inertia")
    };

    // femur_r / tibia_r: direct from the .osim (diagonal, products 0).
    let femur = iner("femur_r");
    assert!((femur.mass - 8.98403823076288).abs() < 1e-9);
    assert!((femur.com - nalgebra::Vector3::new(0.0, -0.195031, 0.0)).norm() < 1e-9);
    assert!(
        (femur.tensor[0] - 0.170220714339411).abs() < 1e-9,
        "femur Ixx"
    );
    assert!(
        (femur.tensor[1] - 0.0446209639530494).abs() < 1e-9,
        "femur Iyy"
    );
    assert!(
        (femur.tensor[2] - 0.179500857839617).abs() < 1e-9,
        "femur Izz"
    );
    assert_eq!(&femur.tensor[3..], &[0.0, 0.0, 0.0], "femur products zero");

    let tibia = iner("tibia_r");
    assert!((tibia.mass - 3.58100089669871).abs() < 1e-9);
    assert!(
        (tibia.tensor[1] - 0.00481356680103835).abs() < 1e-9,
        "tibia Iyy"
    );

    // pelvis (welded root) still carries its mass for IR completeness.
    assert!(iner("pelvis").mass > 0.0);

    // talus_r = the lumped foot composite. Mass is the exact sum of the three foot
    // bodies; the CoM + full tensor are the gait2392 foot composed about the talus
    // frame. The expected values below come from an INDEPENDENT numpy re-implementation
    // (`gen_forward_dynamics.py::foot_composite`, which composes OpenSim's per-body
    // getInertia/transforms) — re-derived here from the raw .osim — so this pins
    // cf-osim's Rust composition against a separate derivation, not against itself.
    let foot = iner("talus_r");
    assert!(
        (foot.mass - (0.0965880214888392 + 1.20735026861049 + 0.209209654544826)).abs() < 1e-12,
        "foot composite mass = talus + calcn + toes"
    );
    assert!(
        (foot.com - nalgebra::Vector3::new(0.06139466, -0.01859489, 0.00592779)).norm() < 1e-6,
        "foot composite CoM, got {:?}",
        foot.com
    );
    let expect = [
        0.0029441428792817,
        0.0082382950109367,
        0.0084486759399523,
        0.0006723861759138,
        0.0003127036922726,
        -6.893180220689e-5,
    ];
    for (k, (&got, &want)) in foot.tensor.iter().zip(&expect).enumerate() {
        assert!(
            (got - want).abs() < 1e-6,
            "foot composite tensor[{k}] = {got}, want {want}"
        );
    }
}
