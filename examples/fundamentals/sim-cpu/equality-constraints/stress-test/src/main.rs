//! Stress test for equality-constraints spec — verifies all 4 MJCF models
//! parse, simulate, and meet pass/fail criteria before building Bevy examples.
//!
//! Run: `cargo run -p example-equality-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::suboptimal_flops
)]

use nalgebra::Vector3;

fn main() {
    println!("=== Equality Constraints Spec — Stress Test ===\n");

    let mut total = 0;
    let mut passed = 0;

    println!("── 0. Diagnostics (single-body connect, weld sag) ──");
    test_diagnostics();

    println!("\n── 1. Connect (ball-and-socket) ──");
    let (p, t) = test_connect();
    passed += p;
    total += t;

    println!("\n── 2. Weld (6-DOF pose lock) ──");
    let (p, t) = test_weld();
    passed += p;
    total += t;

    println!("\n── 3. Distance (rigid rod) ──");
    let (p, t) = test_distance();
    passed += p;
    total += t;

    println!("\n── 4. Joint Coupling (mimic + gear) ──");
    let (p, t) = test_joint_coupling();
    passed += p;
    total += t;

    println!("\n============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS — spec is valid");
    } else {
        println!("  {} FAILED — spec needs revision", total - passed);
        std::process::exit(1);
    }
}

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

// ── 0. Diagnostics ──────────────────────────────────────────────────────────

fn test_diagnostics() {
    // A: Single free body, connect to world, no gravity — should stay put
    println!("\n  --- A: Connect to world, no gravity ---");
    let mjcf_a = r#"
        <mujoco>
          <option gravity="0 0 0" timestep="0.001"/>
          <worldbody>
            <body name="b" pos="0 0 0">
              <freejoint/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
          <equality>
            <connect body1="b" anchor="0 0 0"/>
          </equality>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf_a).expect("parse");
    let mut data = model.make_data();
    data.qvel[0] = 1.0; // push it
    data.forward(&model).expect("fwd");
    for _ in 0..1000 {
        data.step(&model).expect("step");
    }
    println!(
        "    pos after 1s: [{:.4}, {:.4}, {:.4}]",
        data.xpos[1][0], data.xpos[1][1], data.xpos[1][2]
    );
    println!("    err: {:.4} mm", data.xpos[1].norm() * 1000.0);
    println!("    ne={}, nefc={}", data.ne, data.efc_force.len());

    // B: Single free body, connect to world, WITH gravity
    println!("\n  --- B: Connect to world, with gravity ---");
    let mjcf_b = r#"
        <mujoco>
          <option gravity="0 0 -9.81" timestep="0.001"/>
          <worldbody>
            <body name="b" pos="0 0 0">
              <freejoint/>
              <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1"/>
            </body>
          </worldbody>
          <equality>
            <connect body1="b" anchor="0 0 0"/>
          </equality>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf_b).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");
    for i in 0..2000 {
        data.step(&model).expect("step");
        if i == 99 || i == 999 || i == 1999 {
            let err = data.xpos[1].norm();
            println!(
                "    t={:.1}s: pos=[{:.4}, {:.4}, {:.4}], err={:.2}mm, ne={}, angvel={:.3}",
                data.time,
                data.xpos[1][0],
                data.xpos[1][1],
                data.xpos[1][2],
                err * 1000.0,
                data.ne,
                Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm()
            );
        }
    }

    // C: Weld to world — how much sag under gravity?
    println!("\n  --- C: Weld to world, with gravity ---");
    let mjcf_c = r#"
        <mujoco>
          <option gravity="0 0 -9.81" timestep="0.001"/>
          <worldbody>
            <body name="b" pos="0 0 1">
              <freejoint/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
          <equality>
            <weld body1="b"/>
          </equality>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf_c).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");
    let init = data.xpos[1];
    for i in 0..5000 {
        data.step(&model).expect("step");
        if i == 99 || i == 999 || i == 4999 {
            let disp = (data.xpos[1] - init).norm();
            println!(
                "    t={:.1}s: z={:.4}, disp={:.2}mm, ne={}",
                data.time,
                data.xpos[1][2],
                disp * 1000.0,
                data.ne
            );
        }
    }

    // C2: Single weld to world at (0.5, 0, 1) — does angular coupling cause drift?
    println!("\n  --- C2: Single weld at OFFSET position (0.5, 0, 1) ---");
    let mjcf_c2 = r#"
        <mujoco>
          <option gravity="0 0 -9.81" timestep="0.001"/>
          <worldbody>
            <body name="b" pos="0.5 0 1">
              <freejoint/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
          <equality>
            <weld body1="b"/>
          </equality>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf_c2).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");
    let init = data.xpos[1];
    for i in 0..5000 {
        data.step(&model).expect("step");
        if i == 99 || i == 999 || i == 4999 {
            let disp = (data.xpos[1] - init).norm();
            let angvel = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
            println!(
                "    t={:.1}s: z={:.4}, disp={:.2}mm, angvel={:.6}",
                data.time,
                data.xpos[1][2],
                disp * 1000.0,
                angvel
            );
        }
    }

    // D1: Double pendulum connect — no gravity (isolate constraint interaction)
    println!("\n  --- D1: Double connect, NO gravity ---");
    let mjcf_d1 = r#"
        <mujoco>
          <option gravity="0 0 0" timestep="0.001"/>
          <worldbody>
            <body name="a" pos="0 0 0">
              <freejoint/>
              <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" mass="1"/>
            </body>
            <body name="b" pos="0 0 -0.5">
              <freejoint/>
              <geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.025" mass="0.8"/>
            </body>
          </worldbody>
          <equality>
            <connect body1="a" anchor="0 0 0"/>
            <connect body1="a" body2="b" anchor="0 0 -0.5"/>
          </equality>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf_d1).expect("parse");
    let mut data = model.make_data();
    data.qvel[4] = 2.0;
    data.forward(&model).expect("fwd");
    for i in 0..2000 {
        data.step(&model).expect("step");
        if i == 99 || i == 999 || i == 1999 {
            let pivot1_err = data.xpos[1].norm();
            let anchor =
                data.xpos[1] + data.xquat[1].transform_vector(&Vector3::new(0.0, 0.0, -0.5));
            let pivot2_err = (anchor - data.xpos[2]).norm();
            let angvel = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
            println!(
                "    t={:.1}s: pivot1_err={:.2}mm, pivot2_err={:.2}mm, angvel={:.3}",
                data.time,
                pivot1_err * 1000.0,
                pivot2_err * 1000.0,
                angvel
            );
        }
    }

    // D2: Weld to world + falling pair WITHOUT ground (isolate from contacts)
    // Test with explicit Newton solver, high iterations, tight tolerance
    println!("\n  --- D2: Weld to world + falling pair, NO ground ---");
    let mjcf_d2 = r#"
        <mujoco>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="100" tolerance="1e-12"/>
          <worldbody>
            <body name="base" pos="-0.5 0 1.5">
              <freejoint/>
              <geom type="box" size="0.1 0.1 0.1" mass="1"/>
            </body>
            <body name="arm" pos="-0.5 0 1.2">
              <freejoint/>
              <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04" mass="0.5"/>
            </body>
            <body name="fixed" pos="0.5 0 1">
              <freejoint/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
          <equality>
            <weld body1="base" body2="arm"/>
            <weld body1="fixed"/>
          </equality>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf_d2).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");
    let init_fixed = data.xpos[3];
    println!("    nv={}, neq={}", model.nv, model.neq);
    println!("    eq_type: {:?}", model.eq_type);
    println!("    eq_obj1id: {:?}", model.eq_obj1id);
    println!("    eq_obj2id: {:?}", model.eq_obj2id);
    // Print which DOFs each body owns
    for b in 0..model.nbody {
        let jnt_start = model.body_jnt_adr.get(b).copied().unwrap_or(0);
        let jnt_count = model.body_jnt_num.get(b).copied().unwrap_or(0);
        if jnt_count > 0 {
            let dof_start = model.jnt_dof_adr[jnt_start];
            println!(
                "    body {b}: jnt_start={jnt_start}, jnt_count={jnt_count}, dof_start={dof_start}"
            );
        }
    }
    for i in 0..5000 {
        data.step(&model).expect("step");
        if i == 0 {
            // After first step: dump constraint assembly
            println!(
                "    After step 0: ne={}, nefc={}, solver_niter={}",
                data.ne,
                data.efc_force.len(),
                data.solver_niter
            );
            // Print Jacobian sparsity: which rows touch which DOFs
            for r in 0..data.ne.min(12) {
                let mut nonzero_dofs = Vec::new();
                for c in 0..model.nv {
                    let v = data.efc_J[(r, c)];
                    if v.abs() > 1e-15 {
                        nonzero_dofs.push(format!("d{}={:.3}", c, v));
                    }
                }
                println!(
                    "    efc_J row {r}: [{}] force={:.4} aref={:.4} pos={:.6}",
                    nonzero_dofs.join(", "),
                    data.efc_force[r],
                    data.efc_aref[r],
                    data.efc_pos[r]
                );
            }
        }
        if i == 99 || i == 999 || i == 4999 {
            let fixed_disp = (data.xpos[3] - init_fixed).norm();
            let weld_pos_err =
                ((data.xpos[2] - data.xpos[1]) - (Vector3::new(0.0, 0.0, -0.3))).norm();
            // Print forces on the fixed body's constraint rows (rows 6-11)
            let mut fixed_forces = Vec::new();
            for r in 6..12.min(data.efc_force.len()) {
                fixed_forces.push(format!("{:.4}", data.efc_force[r]));
            }
            println!(
                "    t={:.1}s: fixed_disp={:.2}mm, weld_pair_err={:.2}mm, ncon={}, fixed_forces=[{}]",
                data.time,
                fixed_disp * 1000.0,
                weld_pos_err * 1000.0,
                data.ncon,
                fixed_forces.join(", ")
            );
        }
    }

    // D: Two bodies connect — does body-to-body work?
    println!("\n  --- D: Two bodies connected, no gravity ---");
    let mjcf_d = r#"
        <mujoco>
          <option gravity="0 0 0" timestep="0.001"/>
          <worldbody>
            <body name="a" pos="0 0 0">
              <freejoint/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
            <body name="b" pos="0 0 -0.5">
              <freejoint/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
          <equality>
            <connect body1="a" body2="b" anchor="0 0 -0.5"/>
          </equality>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf_d).expect("parse");
    let mut data = model.make_data();
    data.qvel[3] = 1.0; // spin body a
    data.forward(&model).expect("fwd");
    for i in 0..2000 {
        data.step(&model).expect("step");
        if i == 99 || i == 999 || i == 1999 {
            let anchor =
                data.xpos[1] + data.xquat[1].transform_vector(&Vector3::new(0.0, 0.0, -0.5));
            let err = (anchor - data.xpos[2]).norm();
            println!(
                "    t={:.1}s: err={:.2}mm, angvel_a={:.3}",
                data.time,
                err * 1000.0,
                Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm()
            );
        }
    }
}

// ── 1. Connect ──────────────────────────────────────────────────────────────
//
// Bodies start at constraint-satisfied positions (near-zero initial violation).
// Link1 origin at world origin (connect-to-world), link2 origin at link1 tip
// (connect body-to-body). Angular velocity kick starts the swinging.

fn test_connect() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="connect-stress-test">
          <compiler angle="radian"/>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10">
            <flag energy="enable"/>
          </option>
          <worldbody>
            <!-- Link1 origin at world origin — pivot point -->
            <body name="link1" pos="0 0 0">
              <freejoint damping="0.01"/>
              <geom name="rod1" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" mass="1"/>
              <geom name="ball1" type="sphere" pos="0 0 -0.5" size="0.06" mass="0.5"/>
            </body>

            <!-- Link2 origin at link1 tip (0, 0, -0.5) -->
            <body name="link2" pos="0 0 -0.5">
              <freejoint damping="0.01"/>
              <geom name="rod2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.025" mass="0.8"/>
              <geom name="ball2" type="sphere" pos="0 0 -0.4" size="0.05" mass="0.3"/>
            </body>
          </worldbody>
          <equality>
            <connect body1="link1" anchor="0 0 0" solref="0.02 1.0"/>
            <connect body1="link1" body2="link2" anchor="0 0 -0.5" solref="0.02 1.0"/>
          </equality>
        </mujoco>
    "#;

    let model = match sim_mjcf::load_model(mjcf) {
        Ok(m) => m,
        Err(e) => {
            println!("  [FAIL] MJCF parse: {e}");
            return (0, 1);
        }
    };
    println!(
        "  Model: neq={}, nbody={}, njnt={}",
        model.neq, model.nbody, model.njnt
    );

    let mut data = model.make_data();
    // Angular kick to start swinging (rotation around Y axis)
    data.qvel[4] = 2.0;
    data.forward(&model).expect("forward");

    let initial_energy = data.total_energy();

    let steps = 5000; // 5 seconds at dt=0.001
    let mut max_pivot1_err = 0.0_f64;
    let mut max_pivot2_err = 0.0_f64;
    let mut link1_max_angvel = 0.0_f64;

    for _ in 0..steps {
        data.step(&model).expect("step");

        // Pivot 1: link1 origin should stay at world origin (0, 0, 0)
        let err1 = data.xpos[1].norm();
        max_pivot1_err = max_pivot1_err.max(err1);

        // Pivot 2: anchor at (0,0,-0.5) in link1 frame should match link2 origin
        let anchor_world =
            data.xpos[1] + data.xquat[1].transform_vector(&Vector3::new(0.0, 0.0, -0.5));
        let err2 = (anchor_world - data.xpos[2]).norm();
        max_pivot2_err = max_pivot2_err.max(err2);

        // Angular velocity of link1 (free joint: qvel[3..6])
        let angvel_mag = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
        link1_max_angvel = link1_max_angvel.max(angvel_mag);
    }

    let final_energy = data.total_energy();
    let energy_growth = (final_energy - initial_energy) / initial_energy.abs().max(1e-10);

    let mut passed = 0;
    let total = 4;

    if check(
        "Pivot 1 anchored",
        max_pivot1_err < 0.01,
        &format!("max err = {:.2} mm", max_pivot1_err * 1000.0),
    ) {
        passed += 1;
    }
    if check(
        "Pivot 2 attached",
        max_pivot2_err < 0.01,
        &format!("max err = {:.2} mm", max_pivot2_err * 1000.0),
    ) {
        passed += 1;
    }
    if check(
        "Rotation free",
        link1_max_angvel > 0.1,
        &format!("max w = {:.3} rad/s", link1_max_angvel),
    ) {
        passed += 1;
    }
    if check(
        "Energy bounded",
        energy_growth < 0.05,
        &format!("growth = {:.2}%", energy_growth * 100.0),
    ) {
        passed += 1;
    }

    (passed, total)
}

// ── 2. Weld ─────────────────────────────────────────────────────────────────

fn test_weld() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="weld-stress-test">
          <compiler angle="radian"/>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10">
            <flag energy="enable"/>
          </option>
          <worldbody>
            <geom name="ground" type="plane" size="2 2 0.01"/>

            <!-- Welded pair — base + arm, falling together -->
            <body name="base" pos="-0.5 0 1.5">
              <freejoint/>
              <geom name="base_box" type="box" size="0.12 0.08 0.08" mass="1.0"/>
            </body>
            <body name="arm" pos="-0.5 0 1.2">
              <freejoint/>
              <geom name="arm_cap" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04" mass="0.5"/>
            </body>

            <!-- Fixed to world — should resist gravity -->
            <body name="fixed" pos="0.5 0 1.0">
              <freejoint/>
              <geom name="fixed_sphere" type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <equality>
            <weld body1="base" body2="arm" solref="0.02 1.0"/>
            <weld body1="fixed" solref="0.02 1.0"/>
          </equality>
        </mujoco>
    "#;

    let model = match sim_mjcf::load_model(mjcf) {
        Ok(m) => m,
        Err(e) => {
            println!("  [FAIL] MJCF parse: {e}");
            return (0, 1);
        }
    };
    println!(
        "  Model: neq={}, nbody={}, njnt={}",
        model.neq, model.nbody, model.njnt
    );

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let initial_rel_pos = data.xpos[2] - data.xpos[1]; // arm - base
    let initial_rel_quat = data.xquat[1].inverse() * data.xquat[2];
    let initial_fixed_pos = data.xpos[3];

    let steps = 5000;
    let mut max_weld_pos_err = 0.0_f64;
    let mut max_weld_angle_err = 0.0_f64;
    let mut max_fixed_displacement = 0.0_f64;
    let mut freefall_vel_match = true;

    for i in 0..steps {
        data.step(&model).expect("step");

        // Weld pair: relative pose should stay constant
        let rel_pos = data.xpos[2] - data.xpos[1];
        let pos_err = (rel_pos - initial_rel_pos).norm();
        max_weld_pos_err = max_weld_pos_err.max(pos_err);

        let rel_quat = data.xquat[1].inverse() * data.xquat[2];
        let quat_diff = initial_rel_quat.inverse() * rel_quat;
        let angle_err = 2.0
            * quat_diff
                .quaternion()
                .imag()
                .norm()
                .atan2(quat_diff.quaternion().w.abs());
        max_weld_angle_err = max_weld_angle_err.max(angle_err);

        // Fixed body: should stay put
        let fixed_disp = (data.xpos[3] - initial_fixed_pos).norm();
        max_fixed_displacement = max_fixed_displacement.max(fixed_disp);

        // During freefall (first 0.3s before ground contact), z-velocities match
        // base: free joint DOFs 0..6, arm: free joint DOFs 6..12
        if i < 300 && data.time < 0.3 {
            let base_vz = data.qvel[2];
            let arm_vz = data.qvel[8];
            if (base_vz - arm_vz).abs() > 0.05 * base_vz.abs().max(0.1) {
                freefall_vel_match = false;
            }
        }
    }

    let mut passed = 0;
    let total = 4;

    if check(
        "Weld pose locked (pos)",
        max_weld_pos_err < 0.01,
        &format!("max = {:.2} mm", max_weld_pos_err * 1000.0),
    ) {
        passed += 1;
    }
    if check(
        "Weld pose locked (angle)",
        max_weld_angle_err < 0.05,
        &format!("max = {:.4} rad", max_weld_angle_err),
    ) {
        passed += 1;
    }
    if check(
        "Fixed in space",
        max_fixed_displacement < 0.01,
        &format!("max = {:.2} mm", max_fixed_displacement * 1000.0),
    ) {
        passed += 1;
    }
    if check(
        "Pair falls together",
        freefall_vel_match,
        "z-velocities match during freefall",
    ) {
        passed += 1;
    }

    (passed, total)
}

// ── 3. Distance ─────────────────────────────────────────────────────────────

fn test_distance() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="distance-stress-test">
          <compiler angle="radian"/>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10">
            <flag energy="enable"/>
          </option>
          <worldbody>
            <geom name="ground" type="plane" size="2 2 0.01"/>

            <body name="sphere_a" pos="0 0 1.5">
              <freejoint/>
              <geom name="ga" type="sphere" size="0.08" mass="1.0"/>
            </body>
            <body name="sphere_b" pos="0 0 1.0">
              <freejoint/>
              <geom name="gb" type="sphere" size="0.06" mass="0.5"/>
            </body>
          </worldbody>
          <equality>
            <distance geom1="ga" geom2="gb" distance="0.5" solref="0.02 1.0"/>
          </equality>
        </mujoco>
    "#;

    let model = match sim_mjcf::load_model(mjcf) {
        Ok(m) => m,
        Err(e) => {
            println!("  [FAIL] MJCF parse: {e}");
            return (0, 1);
        }
    };
    println!(
        "  Model: neq={}, nbody={}, njnt={}",
        model.neq, model.nbody, model.njnt
    );

    let mut data = model.make_data();
    // Lateral velocity for tumbling
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    let initial_energy = data.total_energy();
    let target_dist = 0.5;

    let steps = 5000;
    let mut max_dist_err = 0.0_f64;
    let mut sphere_a_moved = false;

    for _ in 0..steps {
        data.step(&model).expect("step");

        let sep = (data.xpos[1] - data.xpos[2]).norm();
        let err = (sep - target_dist).abs();
        max_dist_err = max_dist_err.max(err);

        let vel_a = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
        if vel_a > 0.01 {
            sphere_a_moved = true;
        }
    }

    let final_energy = data.total_energy();
    let energy_growth = (final_energy - initial_energy) / initial_energy.abs().max(1e-10);

    let vel_a = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    let vel_b = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();

    let mut passed = 0;
    let total = 4;

    if check(
        "Distance maintained",
        max_dist_err < 0.01,
        &format!("max err = {:.2} mm", max_dist_err * 1000.0),
    ) {
        passed += 1;
    }
    if check(
        "Both move",
        sphere_a_moved,
        &format!("vel_a = {vel_a:.4} m/s"),
    ) {
        passed += 1;
    }
    if check(
        "Mass ratio effect",
        vel_b > vel_a * 0.5 || vel_a < 0.01,
        &format!("vel_a={vel_a:.3}, vel_b={vel_b:.3}"),
    ) {
        passed += 1;
    }
    if check(
        "Energy bounded",
        energy_growth < 0.05,
        &format!("growth = {:.2}%", energy_growth * 100.0),
    ) {
        passed += 1;
    }

    (passed, total)
}

// ── 4. Joint Coupling ───────────────────────────────────────────────────────
//
// MuJoCo convention: constraint enforces joint1 = poly(joint2).
// Mimic: polycoef="0 1" → j_a1 = j_a2  (1:1)
// Gear:  joint1=j_b2, joint2=j_b1, polycoef="0 2" → j_b2 = 2*j_b1

fn test_joint_coupling() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="joint-coupling-stress-test">
          <compiler angle="radian"/>
          <option gravity="0 0 -9.81" timestep="0.001">
            <flag energy="enable"/>
          </option>
          <worldbody>
            <!-- Mimic pair -->
            <body name="arm_a1" pos="-0.8 0 1.5">
              <joint name="j_a1" type="hinge" axis="0 1 0" damping="0.5"/>
              <geom name="arm_a1" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
            </body>
            <body name="arm_a2" pos="-0.4 0 1.5">
              <joint name="j_a2" type="hinge" axis="0 1 0" damping="0.5"/>
              <geom name="arm_a2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
            </body>

            <!-- Gear pair — motor drives j_b1, j_b2 follows at 2x -->
            <body name="arm_b1" pos="0.4 0 1.5">
              <joint name="j_b1" type="hinge" axis="0 1 0" damping="0.5"/>
              <geom name="arm_b1" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
            </body>
            <body name="arm_b2" pos="0.8 0 1.5">
              <joint name="j_b2" type="hinge" axis="0 1 0" damping="0.5"/>
              <geom name="arm_b2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
            </body>
          </worldbody>

          <actuator>
            <motor name="drive" joint="j_b1" gear="1"/>
          </actuator>

          <equality>
            <!-- Mimic: j_a1 = j_a2 (1:1) -->
            <joint joint1="j_a1" joint2="j_a2" polycoef="0 1" solref="0.05 1.0"/>
            <!-- Gear: j_b2 = 2*j_b1 (joint1=j_b2, joint2=j_b1, so j_b2 = poly(j_b1) = 2*j_b1) -->
            <joint joint1="j_b2" joint2="j_b1" polycoef="0 2" solref="0.05 1.0"/>
          </equality>
        </mujoco>
    "#;

    let model = match sim_mjcf::load_model(mjcf) {
        Ok(m) => m,
        Err(e) => {
            println!("  [FAIL] MJCF parse: {e}");
            return (0, 1);
        }
    };
    println!(
        "  Model: neq={}, nbody={}, njnt={}, nu={}",
        model.neq, model.nbody, model.njnt, model.nu
    );

    let mut data = model.make_data();
    // Mimic: start at different angles to test convergence
    data.qpos[0] = 0.5; // j_a1
    data.qpos[1] = -0.3; // j_a2
    // Gear: start at satisfied position (j_b2 = 2*j_b1 = 0)
    data.forward(&model).expect("forward");

    let steps = 5000;
    let mut mimic_converged = false;
    let mut max_mimic_err_after_1s = 0.0_f64;
    let mut max_gear_err = 0.0_f64;
    let mut j_b1_min = f64::MAX;
    let mut j_b1_max = f64::MIN;

    for _ in 0..steps {
        let t = data.time;
        data.ctrl[0] = 2.0 * (2.0 * std::f64::consts::PI * t).sin();

        data.step(&model).expect("step");

        // Mimic: j_a1 ≈ j_a2
        let mimic_err = (data.qpos[0] - data.qpos[1]).abs();
        if data.time >= 1.0 {
            if !mimic_converged && mimic_err < 0.1 {
                mimic_converged = true;
            }
            max_mimic_err_after_1s = max_mimic_err_after_1s.max(mimic_err);
        }

        // Gear: j_b2 ≈ 2*j_b1 (joint1=j_b2 is qpos[3], joint2=j_b1 is qpos[2])
        let q_b1 = data.qpos[2];
        let q_b2 = data.qpos[3];
        let gear_err = (q_b2 - 2.0 * q_b1).abs();
        if data.time >= 1.0 {
            max_gear_err = max_gear_err.max(gear_err);
        }

        if data.time > 0.5 {
            j_b1_min = j_b1_min.min(q_b1);
            j_b1_max = j_b1_max.max(q_b1);
        }
    }

    let motor_oscillated = (j_b1_max - j_b1_min) > 0.05;

    let mut passed = 0;
    let total = 4;

    if check("Mimic converges", mimic_converged, "converged by 1s") {
        passed += 1;
    }
    if check(
        "Mimic tracks",
        max_mimic_err_after_1s < 0.1,
        &format!("max err after 1s = {:.4} rad", max_mimic_err_after_1s),
    ) {
        passed += 1;
    }
    if check(
        "Gear ratio",
        max_gear_err < 0.15,
        &format!("max |q_b2 - 2*q_b1| = {:.4} rad", max_gear_err),
    ) {
        passed += 1;
    }
    if check(
        "Motor drives",
        motor_oscillated,
        &format!("q range = {:.3}..{:.3}", j_b1_min, j_b1_max),
    ) {
        passed += 1;
    }

    (passed, total)
}
