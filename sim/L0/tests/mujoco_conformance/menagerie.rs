//! MuJoCo Menagerie model loading tests.
//!
//! Tests loading real-world robot models from the MuJoCo Menagerie:
//! https://github.com/google-deepmind/mujoco_menagerie
//!
//! These tests verify that sim-mjcf can parse production-quality MJCF files
//! used by the robotics research community.

use std::path::PathBuf;

/// Get the path to the mujoco_menagerie assets directory.
fn menagerie_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("assets")
        .join("mujoco_menagerie")
}

/// Check if menagerie assets are available (submodule initialized).
fn menagerie_available() -> bool {
    menagerie_path().join("franka_emika_panda").exists()
}

macro_rules! menagerie_test {
    ($name:ident, $model_path:expr, $expected_bodies:expr, $expected_joints:expr) => {
        #[test]
        fn $name() {
            if !menagerie_available() {
                eprintln!(
                    "Skipping {}: mujoco_menagerie submodule not initialized",
                    stringify!($name)
                );
                eprintln!("Run: git submodule update --init --recursive");
                return;
            }

            let model_file = menagerie_path().join($model_path);
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

                    // Verify minimum expected counts (models may have more)
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
                    panic!("Failed to load {}: {}", $model_path, e);
                }
            }
        }
    };
}

// =============================================================================
// Robot Arms
// =============================================================================

menagerie_test!(
    test_franka_panda,
    "franka_emika_panda/panda.xml",
    8, // 7 links + base
    7  // 7 DOF arm
);

menagerie_test!(
    test_franka_panda_hand,
    "franka_emika_panda/hand.xml",
    3, // hand + 2 fingers
    2  // 2 finger joints
);

menagerie_test!(
    test_universal_robots_ur5e,
    "universal_robots_ur5e/ur5e.xml",
    7, // 6 links + base
    6  // 6 DOF arm
);

menagerie_test!(
    test_kuka_iiwa,
    "kuka_iiwa_14/iiwa14.xml",
    8, // 7 links + base
    7  // 7 DOF arm
);

menagerie_test!(
    test_robotis_op3,
    "robotis_op3/op3.xml",
    1, // At least base
    1  // At least one joint
);

// =============================================================================
// Quadrupeds
// =============================================================================

menagerie_test!(
    test_unitree_go1,
    "unitree_go1/go1.xml",
    13, // body + 4 legs * 3 links
    12  // 4 legs * 3 joints
);

menagerie_test!(
    test_unitree_go2,
    "unitree_go2/go2.xml",
    13, // body + 4 legs * 3 links
    12  // 4 legs * 3 joints
);

menagerie_test!(
    test_anymal_c,
    "anybotics_anymal_c/anymal_c.xml",
    13, // body + 4 legs * 3 links
    12  // 4 legs * 3 joints
);

menagerie_test!(
    test_anymal_b,
    "anybotics_anymal_b/anymal_b.xml",
    13, // body + 4 legs * 3 links
    12  // 4 legs * 3 joints
);

// =============================================================================
// Humanoids
// =============================================================================

menagerie_test!(
    test_unitree_h1,
    "unitree_h1/h1.xml",
    15, // Many body segments
    19  // Full body DOF
);

menagerie_test!(
    test_unitree_g1,
    "unitree_g1/g1.xml",
    10, // Many body segments
    10  // Full body DOF
);

menagerie_test!(
    test_agility_digit,
    "agility_digit/digit.xml",
    1, // At least base
    1  // At least one joint
);

// =============================================================================
// Dexterous Hands
// =============================================================================

menagerie_test!(
    test_shadow_hand,
    "shadow_hand/right_hand.xml",
    20, // Many finger segments
    20  // High DOF hand
);

menagerie_test!(
    test_robotiq_2f85,
    "robotiq_2f85/2f85.xml",
    3, // Base + fingers
    2  // Finger joints
);

// =============================================================================
// Other Robots
// =============================================================================

menagerie_test!(
    test_google_barkour,
    "google_barkour_v0/barkour_v0.xml",
    1, // At least base
    1  // At least one joint
);

menagerie_test!(
    test_google_robot,
    "google_robot/robot.xml",
    1, // At least base
    1  // At least one joint
);

menagerie_test!(
    test_aloha,
    "aloha/aloha.xml",
    1, // At least base
    1  // At least one joint
);

// =============================================================================
// Summary test
// =============================================================================

#[test]
fn test_menagerie_summary() {
    if !menagerie_available() {
        eprintln!("Skipping menagerie summary: submodule not initialized");
        return;
    }

    let models = [
        ("franka_emika_panda/panda.xml", "Franka Panda"),
        ("universal_robots_ur5e/ur5e.xml", "UR5e"),
        ("unitree_go1/go1.xml", "Unitree Go1"),
        ("unitree_h1/h1.xml", "Unitree H1"),
        ("shadow_hand/right_hand.xml", "Shadow Hand"),
    ];

    println!("\n=== MuJoCo Menagerie Loading Summary ===\n");

    let mut passed = 0;
    let mut failed = 0;

    for (path, name) in models {
        let model_file = menagerie_path().join(path);
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

    // This test is informational - don't fail on individual model failures
    // Individual model tests above will fail if there are parsing issues
}
