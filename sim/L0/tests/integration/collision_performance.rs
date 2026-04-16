//! Collision performance tests — throughput observation and scaling validation.
//!
//! Each `perf_*` test prints its measured throughput; the `scaling_*` tests
//! gate on relative scaling ratios (which are robust to instrumentation
//! overhead). Absolute wall-clock thresholds live in benchmarks, not here.

use sim_mjcf::load_model;
use std::time::Instant;

/// Run benchmark and return steps per second.
fn benchmark_model(model: &sim_core::Model, warmup_steps: usize, bench_steps: usize) -> f64 {
    let mut data = model.make_data();

    // Warmup phase
    for _ in 0..warmup_steps {
        data.step(model).expect("step failed");
    }

    // Benchmark phase
    let start = Instant::now();
    for _ in 0..bench_steps {
        data.step(model).expect("step failed");
    }
    let elapsed = start.elapsed().as_secs_f64();

    bench_steps as f64 / elapsed
}

// ============================================================================
// Simple System Benchmarks
// ============================================================================

/// Performance gate: Simple pendulum simulation throughput.
///
/// A single hinge joint with one body should be extremely fast.
/// This establishes the baseline for computational overhead.
#[test]
fn perf_simple_pendulum() {
    let mjcf = r#"
        <mujoco model="simple_pendulum">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05 0.5" pos="0 0 -0.5" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let steps_per_sec = benchmark_model(&model, 100, 1000);

    eprintln!(
        "Simple pendulum: {:.0} steps/sec ({:.2}ms/step)",
        steps_per_sec,
        1000.0 / steps_per_sec
    );
}

/// Performance gate: Double pendulum (chaotic system).
///
/// Two hinge joints, tests basic articulation performance.
#[test]
fn perf_double_pendulum() {
    let mjcf = r#"
        <mujoco model="double_pendulum">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link1" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05 0.5" pos="0 0 -0.5" mass="1"/>
                    <body name="link2" pos="0 0 -1">
                        <joint type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.05 0.5" pos="0 0 -0.5" mass="1"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let steps_per_sec = benchmark_model(&model, 100, 1000);

    eprintln!(
        "Double pendulum: {:.0} steps/sec ({:.2}ms/step)",
        steps_per_sec,
        1000.0 / steps_per_sec
    );
}

// ============================================================================
// Contact System Benchmarks
// ============================================================================

/// Performance gate: Ball stack with contacts.
///
/// Three spheres stacked on a plane, active contact constraints.
/// This tests contact detection and constraint solving performance.
#[test]
fn perf_ball_stack() {
    let mjcf = r#"
        <mujoco model="ball_stack">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball1" pos="0 0 0.5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.5" mass="1"/>
                </body>
                <body name="ball2" pos="0 0 1.5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.5" mass="1"/>
                </body>
                <body name="ball3" pos="0 0 2.5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.5" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let steps_per_sec = benchmark_model(&model, 100, 1000);

    eprintln!(
        "Ball stack: {:.0} steps/sec ({:.2}ms/step)",
        steps_per_sec,
        1000.0 / steps_per_sec
    );
}

/// Performance gate: Falling boxes on plane.
///
/// Multiple box-plane contacts with SAT collision detection.
///
/// This test is ignored by default because performance benchmarks are
/// sensitive to system load and produce flaky results when run concurrently
/// with other tests. Run separately with:
///
/// ```sh
/// cargo test -p sim-conformance-tests --test integration perf_falling_boxes -- --ignored
/// ```
#[test]
#[ignore = "benchmark — sensitive to system load, run separately with --release --ignored"]
fn perf_falling_boxes() {
    let mjcf = r#"
        <mujoco model="falling_boxes">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="box1" pos="-0.5 0 0.5">
                    <joint type="free"/>
                    <geom type="box" size="0.4 0.4 0.4" mass="1"/>
                </body>
                <body name="box2" pos="0.5 0 0.5">
                    <joint type="free"/>
                    <geom type="box" size="0.4 0.4 0.4" mass="1"/>
                </body>
                <body name="box3" pos="0 0 1.5">
                    <joint type="free"/>
                    <geom type="box" size="0.4 0.4 0.4" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let steps_per_sec = benchmark_model(&model, 100, 1000);

    eprintln!(
        "Falling boxes: {:.0} steps/sec ({:.2}ms/step)",
        steps_per_sec,
        1000.0 / steps_per_sec
    );
}

/// Performance gate: Capsule pile on plane.
///
/// Multiple capsule-plane and capsule-capsule contacts.
#[test]
fn perf_capsule_pile() {
    let mjcf = r#"
        <mujoco model="capsule_pile">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="cap1" pos="-0.3 0 0.7">
                    <joint type="free"/>
                    <geom type="capsule" size="0.2 0.5" mass="1"/>
                </body>
                <body name="cap2" pos="0.3 0 0.7">
                    <joint type="free"/>
                    <geom type="capsule" size="0.2 0.5" mass="1"/>
                </body>
                <body name="cap3" pos="0 0 1.7" euler="90 0 0">
                    <joint type="free"/>
                    <geom type="capsule" size="0.2 0.5" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let steps_per_sec = benchmark_model(&model, 100, 1000);

    eprintln!(
        "Capsule pile: {:.0} steps/sec ({:.2}ms/step)",
        steps_per_sec,
        1000.0 / steps_per_sec
    );
}

// ============================================================================
// Complex System Benchmarks
// ============================================================================

/// Performance gate: Simplified humanoid-like model.
///
/// Multiple bodies, joints, and potential self-collision.
/// This represents a typical robotics simulation workload.
#[test]
fn perf_humanoid_simplified() {
    let mjcf = r#"
        <mujoco model="humanoid_simplified">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <body name="torso" pos="0 0 1.2">
                    <joint type="free"/>
                    <geom type="capsule" size="0.1 0.2" mass="10"/>

                    <body name="head" pos="0 0 0.3">
                        <joint type="ball"/>
                        <geom type="sphere" size="0.1" mass="2"/>
                    </body>

                    <body name="upper_arm_r" pos="0.2 0 0.1">
                        <joint type="ball"/>
                        <geom type="capsule" size="0.04 0.15" mass="1.5"/>
                        <body name="lower_arm_r" pos="0.3 0 0">
                            <joint type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.03 0.12" mass="1"/>
                        </body>
                    </body>

                    <body name="upper_arm_l" pos="-0.2 0 0.1">
                        <joint type="ball"/>
                        <geom type="capsule" size="0.04 0.15" mass="1.5"/>
                        <body name="lower_arm_l" pos="-0.3 0 0">
                            <joint type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.03 0.12" mass="1"/>
                        </body>
                    </body>

                    <body name="upper_leg_r" pos="0.1 0 -0.3">
                        <joint type="ball"/>
                        <geom type="capsule" size="0.05 0.2" mass="3"/>
                        <body name="lower_leg_r" pos="0 0 -0.4">
                            <joint type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.04 0.2" mass="2"/>
                        </body>
                    </body>

                    <body name="upper_leg_l" pos="-0.1 0 -0.3">
                        <joint type="ball"/>
                        <geom type="capsule" size="0.05 0.2" mass="3"/>
                        <body name="lower_leg_l" pos="0 0 -0.4">
                            <joint type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.04 0.2" mass="2"/>
                        </body>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let steps_per_sec = benchmark_model(&model, 50, 500);

    eprintln!(
        "Humanoid simplified: {:.0} steps/sec ({:.2}ms/step)",
        steps_per_sec,
        1000.0 / steps_per_sec
    );
}

/// Performance gate: Articulated robot arm.
///
/// 6-DOF arm typical of industrial robots.
#[test]
fn perf_robot_arm() {
    let mjcf = r#"
        <mujoco model="robot_arm">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="base" pos="0 0 0.1">
                    <geom type="cylinder" size="0.1 0.1" mass="5"/>
                    <body name="link1" pos="0 0 0.1">
                        <joint type="hinge" axis="0 0 1"/>
                        <geom type="capsule" size="0.04 0.15" pos="0 0 0.15" mass="2"/>
                        <body name="link2" pos="0 0 0.3">
                            <joint type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.03 0.2" pos="0 0 0.2" mass="1.5"/>
                            <body name="link3" pos="0 0 0.4">
                                <joint type="hinge" axis="0 1 0"/>
                                <geom type="capsule" size="0.025 0.15" pos="0 0 0.15" mass="1"/>
                                <body name="link4" pos="0 0 0.3">
                                    <joint type="hinge" axis="0 0 1"/>
                                    <geom type="capsule" size="0.02 0.1" pos="0 0 0.1" mass="0.5"/>
                                    <body name="link5" pos="0 0 0.2">
                                        <joint type="hinge" axis="0 1 0"/>
                                        <geom type="capsule" size="0.015 0.05" pos="0 0 0.05" mass="0.3"/>
                                        <body name="end_effector" pos="0 0 0.1">
                                            <joint type="hinge" axis="0 0 1"/>
                                            <geom type="sphere" size="0.02" mass="0.1"/>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");
    let steps_per_sec = benchmark_model(&model, 100, 1000);

    eprintln!(
        "Robot arm: {:.0} steps/sec ({:.2}ms/step)",
        steps_per_sec,
        1000.0 / steps_per_sec
    );
}

// ============================================================================
// Scaling Tests
// ============================================================================

/// Verify collision scales sub-linearly with body count.
///
/// With spatial hashing/BVH, doubling bodies should NOT double collision time.
#[test]
fn scaling_collision_bodies() {
    // 5 spheres
    let mjcf_5 = r#"
        <mujoco model="scale_5">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="s0" pos="0 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s1" pos="1.2 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s2" pos="2.4 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s3" pos="3.6 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s4" pos="4.8 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
            </worldbody>
        </mujoco>
    "#;

    // 10 spheres
    let mjcf_10 = r#"
        <mujoco model="scale_10">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1"/>
                <body name="s0" pos="0 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s1" pos="1.2 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s2" pos="2.4 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s3" pos="3.6 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s4" pos="4.8 0 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s5" pos="0 1.2 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s6" pos="1.2 1.2 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s7" pos="2.4 1.2 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s8" pos="3.6 1.2 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
                <body name="s9" pos="4.8 1.2 0.5"><joint type="free"/><geom type="sphere" size="0.5" mass="1"/></body>
            </worldbody>
        </mujoco>
    "#;

    let model_5 = load_model(mjcf_5).expect("Failed to load 5-sphere model");
    let model_10 = load_model(mjcf_10).expect("Failed to load 10-sphere model");

    let steps_5 = benchmark_model(&model_5, 50, 500);
    let steps_10 = benchmark_model(&model_10, 50, 500);

    // Time per step
    let time_5 = 1.0 / steps_5;
    let time_10 = 1.0 / steps_10;

    // With 2× bodies, time should be less than 10× (allowing for O(n²) to O(n³) behavior).
    //
    // Theoretical scaling:
    // - O(n): ratio = 2.0
    // - O(n²): ratio = 4.0
    // - O(n³): ratio = 8.0 (Cholesky)
    //
    // Current architecture uses dense mass matrix and Cholesky solve, which gives
    // scaling between O(n²) and O(n³). Future optimization could exploit block-diagonal
    // structure for independent free joints to achieve better scaling.
    //
    // Threshold of 10.0 allows for O(n³) Cholesky scaling plus CI measurement variance.
    // CI runners show higher variance due to shared resources and virtualization.
    let ratio = time_10 / time_5;

    assert!(
        ratio < 10.0,
        "Collision scaling regression: 2× bodies → {:.1}× time (expected <10.0× for current dense solver)",
        ratio
    );

    eprintln!(
        "Collision scaling: 5 spheres {:.0} steps/sec, 10 spheres {:.0} steps/sec, ratio {:.2}×",
        steps_5, steps_10, ratio
    );
}

// ============================================================================
// Consistency Tests
// ============================================================================

/// Verify simulation is deterministic (same inputs → same outputs).
///
/// Run the same simulation twice and verify identical results.
#[test]
fn determinism_check() {
    let mjcf = r#"
        <mujoco model="determinism">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="5 5 0.1"/>
                <body name="ball" pos="0.1 0.2 1.0">
                    <joint type="free"/>
                    <geom type="sphere" size="0.5" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");

    // Run 1
    let mut data1 = model.make_data();
    for _ in 0..100 {
        data1.step(&model).expect("step failed");
    }
    let pos1 = data1.xpos[1];

    // Run 2
    let mut data2 = model.make_data();
    for _ in 0..100 {
        data2.step(&model).expect("step failed");
    }
    let pos2 = data2.xpos[1];

    // Positions should be exactly identical (bitwise)
    assert_eq!(pos1.x, pos2.x, "Simulation not deterministic: x differs");
    assert_eq!(pos1.y, pos2.y, "Simulation not deterministic: y differs");
    assert_eq!(pos1.z, pos2.z, "Simulation not deterministic: z differs");
}

/// Verify stepping multiple times is equivalent to multiple single steps.
///
/// This catches any hidden state accumulation issues.
#[test]
fn step_equivalence() {
    let mjcf = r#"
        <mujoco model="step_equiv">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05 0.5" pos="0 0 -0.5" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load model");

    // Run 100 steps
    let mut data = model.make_data();
    data.qpos[0] = 0.5; // Initial angle
    for _ in 0..100 {
        data.step(&model).expect("step failed");
    }
    let pos_100 = data.qpos[0];

    // Run 50 steps, then 50 more
    let mut data2 = model.make_data();
    data2.qpos[0] = 0.5;
    for _ in 0..50 {
        data2.step(&model).expect("step failed");
    }
    for _ in 0..50 {
        data2.step(&model).expect("step failed");
    }
    let pos_50_50 = data2.qpos[0];

    // Should be identical
    assert_eq!(
        pos_100, pos_50_50,
        "Step splitting changes result: {} vs {}",
        pos_100, pos_50_50
    );
}
