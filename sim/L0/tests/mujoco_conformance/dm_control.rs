//! DeepMind Control Suite model loading tests.
//!
//! Tests loading benchmark models from the DeepMind Control Suite:
//! https://github.com/google-deepmind/dm_control
//!
//! These models are the standard benchmarks for reinforcement learning research
//! and represent well-tested MJCF files.

use std::path::PathBuf;

/// Get the path to the dm_control suite directory.
fn dm_control_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("assets")
        .join("dm_control")
        .join("dm_control")
        .join("suite")
}

/// Check if dm_control assets are available (submodule initialized).
fn dm_control_available() -> bool {
    dm_control_path().join("cartpole.xml").exists()
}

macro_rules! dm_control_test {
    ($name:ident, $model_file:expr, $expected_bodies:expr, $expected_joints:expr) => {
        #[test]
        fn $name() {
            if !dm_control_available() {
                eprintln!(
                    "Skipping {}: dm_control submodule not initialized",
                    stringify!($name)
                );
                eprintln!("Run: git submodule update --init --recursive");
                return;
            }

            let model_file = dm_control_path().join($model_file);
            assert!(
                model_file.exists(),
                "Model file not found: {}",
                model_file.display()
            );

            let result = sim_mjcf::load_mjcf_file(&model_file);

            match result {
                Ok(model) => {
                    println!("Loaded: {}", model.name);
                    println!("  Bodies: {}", model.bodies.len());
                    println!("  Joints: {}", model.joints.len());
                    println!("  Actuators: {}", model.actuators.len());

                    // Verify minimum expected counts
                    assert!(
                        model.bodies.len() >= $expected_bodies,
                        "Expected at least {} bodies, got {}",
                        $expected_bodies,
                        model.bodies.len()
                    );
                    assert!(
                        model.joints.len() >= $expected_joints,
                        "Expected at least {} joints, got {}",
                        $expected_joints,
                        model.joints.len()
                    );
                }
                Err(e) => {
                    panic!("Failed to load {}: {}", $model_file, e);
                }
            }
        }
    };
}

// =============================================================================
// Classic Control Tasks
// =============================================================================

dm_control_test!(
    test_pendulum,
    "pendulum.xml",
    1, // pendulum body
    1  // hinge joint
);

dm_control_test!(
    test_cartpole,
    "cartpole.xml",
    2, // cart + pole
    2  // slide + hinge
);

dm_control_test!(
    test_acrobot,
    "acrobot.xml",
    2, // upper + lower link
    2  // 2 hinge joints
);

dm_control_test!(
    test_point_mass,
    "point_mass.xml",
    1, // point mass
    2  // 2D slide joints
);

// =============================================================================
// Locomotion Tasks
// =============================================================================

dm_control_test!(
    test_hopper,
    "hopper.xml",
    4, // torso + leg segments
    3  // hip, knee, ankle
);

dm_control_test!(
    test_walker,
    "walker.xml",
    7, // torso + 2 legs
    6  // 2 legs * 3 joints
);

dm_control_test!(
    test_cheetah,
    "cheetah.xml",
    7, // torso + legs
    6  // leg joints
);

dm_control_test!(
    test_swimmer,
    "swimmer.xml",
    3, // 3 segments (swimmer3)
    2  // 2 joints
);

dm_control_test!(
    test_humanoid,
    "humanoid.xml",
    13, // full humanoid
    21  // many joints
);

dm_control_test!(
    test_humanoid_cmu,
    "humanoid_CMU.xml",
    15, // CMU humanoid (more detailed)
    25  // more joints
);

dm_control_test!(
    test_quadruped,
    "quadruped.xml",
    13, // body + 4 legs
    12  // 4 legs * 3 joints
);

dm_control_test!(
    test_dog, "dog.xml", 1, // At least base
    1  // At least one joint
);

// =============================================================================
// Manipulation Tasks
// =============================================================================

dm_control_test!(
    test_reacher,
    "reacher.xml",
    3, // base + arm segments
    2  // 2 joints
);

dm_control_test!(
    test_finger,
    "finger.xml",
    4, // finger segments + spinner
    3  // finger joints
);

dm_control_test!(
    test_ball_in_cup,
    "ball_in_cup.xml",
    3, // arm + cup + ball
    2  // arm joints
);

dm_control_test!(
    test_manipulator,
    "manipulator.xml",
    5, // arm segments + objects
    4  // arm joints
);

dm_control_test!(
    test_stacker,
    "stacker.xml",
    1, // At least base
    1  // At least one joint
);

// =============================================================================
// Other Tasks
// =============================================================================

dm_control_test!(
    test_fish, "fish.xml", 4, // body + fins
    5  // swimming DOF
);

dm_control_test!(
    test_lqr, "lqr.xml", 1, // single body
    1  // linear joint
);

// =============================================================================
// Summary test
// =============================================================================

#[test]
fn test_dm_control_summary() {
    if !dm_control_available() {
        eprintln!("Skipping dm_control summary: submodule not initialized");
        return;
    }

    let models = [
        ("pendulum.xml", "Pendulum"),
        ("cartpole.xml", "Cartpole"),
        ("acrobot.xml", "Acrobot"),
        ("hopper.xml", "Hopper"),
        ("walker.xml", "Walker"),
        ("cheetah.xml", "Cheetah"),
        ("humanoid.xml", "Humanoid"),
        ("reacher.xml", "Reacher"),
        ("finger.xml", "Finger"),
        ("fish.xml", "Fish"),
        ("swimmer.xml", "Swimmer"),
        ("ball_in_cup.xml", "Ball in Cup"),
    ];

    println!("\n=== DeepMind Control Suite Loading Summary ===\n");

    let mut passed = 0;
    let mut failed = 0;

    for (file, name) in models {
        let model_file = dm_control_path().join(file);
        match sim_mjcf::load_mjcf_file(&model_file) {
            Ok(model) => {
                println!(
                    "✓ {} - {} bodies, {} joints",
                    name,
                    model.bodies.len(),
                    model.joints.len()
                );
                passed += 1;
            }
            Err(e) => {
                println!("✗ {} - {}", name, e);
                failed += 1;
            }
        }
    }

    println!("\nResults: {} passed, {} failed", passed, failed);
}
