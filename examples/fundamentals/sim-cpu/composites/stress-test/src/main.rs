//! Stress test — headless validation of cable composite body generation.
//!
//! 8 checks covering: body count, joint count/types, contact exclusions,
//! gravity hang, segment convergence, length preservation, curve shapes,
//! multi-cable independence.
//!
//! Run: `cargo run -p example-composite-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]

use sim_core::{MjJointType, Model};

// ── Helpers ──────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn load(xml: &str) -> Model {
    sim_mjcf::load_model(xml).expect("MJCF should parse")
}

fn step_seconds(model: &Model, data: &mut sim_core::Data, seconds: f64) {
    let steps = (seconds / model.timestep).round() as usize;
    for _ in 0..steps {
        data.step(model).expect("step");
    }
}

// ── MJCF fragments ──────────────────────────────────────────────────────────

fn cable_mjcf(count: u32, prefix: &str, initial: &str, curve: &str, size: &str) -> String {
    format!(
        r#"
        <mujoco model="cable-test">
          <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
          <worldbody>
            <body name="anchor" pos="0 0 1">
              <composite type="cable" prefix="{prefix}" count="{count} 1 1"
                         initial="{initial}" curve="{curve}" size="{size}"
                         offset="0 0 0">
                <joint kind="main" damping="0.05"/>
                <geom type="capsule" size="0.01" rgba="0.8 0.2 0.1 1"
                      density="500"/>
              </composite>
            </body>
          </worldbody>
        </mujoco>
        "#
    )
}

// ── Check 1: Body count ─────────────────────────────────────────────────────

fn check_1_body_count() -> (u32, u32) {
    let xml = cable_mjcf(11, "", "ball", "l 0 0", "1");
    let model = load(&xml);

    // world + anchor + 10 cable bodies = 12
    // count=11 vertices → 10 cable bodies
    let expected = 12;
    let p = check(
        "Body count",
        model.nbody == expected,
        &format!("nbody={}, expected={expected}", model.nbody),
    );
    (u32::from(p), 1)
}

// ── Check 2: Joint count and types ──────────────────────────────────────────

fn check_2_joint_count_and_types() -> (u32, u32) {
    let xml = cable_mjcf(8, "", "ball", "l 0 0", "1");
    let model = load(&xml);

    // count=8 → 7 cable bodies → 7 ball joints (initial="ball")
    let count_ok = model.njnt == 7;
    let all_ball = (0..model.njnt).all(|i| model.jnt_type[i] == MjJointType::Ball);

    let p1 = check(
        "Joint count",
        count_ok,
        &format!("njnt={}, expected=7", model.njnt),
    );
    let p2 = check(
        "All joints are ball",
        all_ball,
        &format!(
            "types: {:?}",
            (0..model.njnt)
                .map(|i| model.jnt_type[i])
                .collect::<Vec<_>>()
        ),
    );
    (u32::from(p1) + u32::from(p2), 2)
}

// ── Check 3: Contact exclusions ─────────────────────────────────────────────

fn check_3_contact_exclusions() -> (u32, u32) {
    let xml = cable_mjcf(6, "", "none", "l 0 0", "1");
    let model = load(&xml);
    let mut data = model.make_data();

    // count=6 → 5 cable bodies → 4 adjacent exclude pairs
    let exclude_count = model.contact_excludes.len();
    let p1 = check(
        "Exclude pair count",
        exclude_count == 4,
        &format!("excludes={exclude_count}, expected=4"),
    );

    // Step and verify no adjacent cable bodies appear in contacts
    step_seconds(&model, &mut data, 2.0);

    let mut adjacent_contact = false;
    for contact in data.contacts.iter().take(data.ncon) {
        let (b1, b2) = contact.bodies(&model);
        let key = (b1.min(b2), b1.max(b2));
        if model.contact_excludes.contains(&key) {
            adjacent_contact = true;
            break;
        }
    }
    let p2 = check(
        "No excluded-pair contacts",
        !adjacent_contact,
        &format!("ncon={}, adjacent_found={adjacent_contact}", data.ncon),
    );
    (u32::from(p1) + u32::from(p2), 2)
}

// ── Check 4: Cable hangs below anchor ───────────────────────────────────────

fn check_4_hangs_below_anchor() -> (u32, u32) {
    // Horizontal cable with high damping, long settling time.
    let xml = r#"
        <mujoco model="hang-test">
          <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
          <worldbody>
            <body name="anchor" pos="0 0 1.5">
              <composite type="cable" prefix="" count="11 1 1"
                         initial="none" curve="l 0 0" size="1">
                <joint kind="main" damping="0.5"/>
                <geom type="capsule" size="0.01" rgba="0.8 0.2 0.1 1"
                      density="500"/>
              </composite>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = load(xml);
    let mut data = model.make_data();

    step_seconds(&model, &mut data, 10.0);

    // Anchor at z=1.5. First cable body (index 2) is pinned at anchor.
    // The pinned body's z may equal anchor_z. All bodies from index 3+
    // should be at or below anchor z after settling (gravity pulls down).
    let anchor_z = data.xpos[2][2]; // pinned cable body z = anchor z
    let mut all_below = true;
    let mut min_z = f64::MAX;
    let mut max_z = f64::MIN;

    for i in 3..model.nbody {
        let z = data.xpos[i][2];
        if z > anchor_z + 1e-3 {
            all_below = false;
        }
        min_z = min_z.min(z);
        max_z = max_z.max(z);
    }

    let p = check(
        "Non-pinned cable bodies below anchor",
        all_below,
        &format!("anchor_z={anchor_z:.3}, cable z range=[{min_z:.3}, {max_z:.3}]"),
    );
    (u32::from(p), 1)
}

// ── Check 5: Convergence with more segments ─────────────────────────────────

fn check_5_convergence() -> (u32, u32) {
    // Three cables with 5, 10, 20 segments (count = 6, 11, 21)
    // Use higher damping and longer settling for convergence
    let counts = [6u32, 11, 21];
    let mut tips = Vec::new();

    for &count in &counts {
        let xml = format!(
            r#"
            <mujoco model="converge-{count}">
              <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
              <worldbody>
                <body name="anchor" pos="0 0 1">
                  <composite type="cable" prefix="" count="{count} 1 1"
                             initial="none" curve="l 0 0" size="1">
                    <joint kind="main" damping="0.5"/>
                    <geom type="capsule" size="0.01" rgba="0.8 0.2 0.1 1"
                          density="500"/>
                  </composite>
                </body>
              </worldbody>
            </mujoco>
            "#
        );
        let model = load(&xml);
        let mut data = model.make_data();
        step_seconds(&model, &mut data, 20.0);

        // Last cable body = model.nbody - 1
        let tip = data.xpos[model.nbody - 1];
        tips.push(tip);
    }

    // |tip_10 - tip_20| < |tip_5 - tip_20|
    let err_5_vs_20 = (tips[0] - tips[2]).norm();
    let err_10_vs_20 = (tips[1] - tips[2]).norm();

    let p = check(
        "10-seg closer to 20-seg than 5-seg",
        err_10_vs_20 < err_5_vs_20,
        &format!("err_5v20={err_5_vs_20:.6}, err_10v20={err_10_vs_20:.6}"),
    );
    (u32::from(p), 1)
}

// ── Check 6: Cable preserves total length ───────────────────────────────────

fn check_6_length_preservation() -> (u32, u32) {
    let xml = cable_mjcf(11, "", "none", "l 0 0", "1");
    let model = load(&xml);
    let mut data = model.make_data();

    step_seconds(&model, &mut data, 5.0);

    // Sum distances between consecutive cable bodies (indices 2..nbody).
    // Body positions represent vertices 0..N-1. The last edge (vertex N-1 to N)
    // extends along the last body's local x-axis by edge_length, so we add it.
    let mut inter_body_sum = 0.0;
    for i in 3..model.nbody {
        let seg = data.xpos[i] - data.xpos[i - 1];
        inter_body_sum += seg.norm();
    }
    // Last edge: size / (count - 1) = 1.0 / 10 = 0.1
    let edge_length = 1.0 / 10.0;
    let total_length = inter_body_sum + edge_length;

    let expected = 1.0;
    let error_pct = ((total_length - expected) / expected).abs() * 100.0;

    let p = check(
        "Total length preserved",
        error_pct < 2.0,
        &format!(
            "length={total_length:.4} (inter-body={inter_body_sum:.4} + last_edge={edge_length:.4}), \
             expected={expected:.1}, error={error_pct:.2}%"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 7: Cosine curve initial shape ─────────────────────────────────────

fn check_7_curve_shape() -> (u32, u32) {
    // curve="s c 0" → sin in x, cos in y, zero in z
    // size="1 0.3 2" → length=1, amplitude=0.3, frequency=2
    let xml = cable_mjcf(21, "", "ball", "s c 0", "1 0.3 2");
    let model = load(&xml);
    let mut data = model.make_data();

    // Forward kinematics only — don't step, read initial shape
    data.forward(&model).expect("forward");

    // Cable bodies: indices 2..nbody (0=world, 1=anchor)
    let cable_start = 2;
    let cable_end = model.nbody;
    let n_cable = cable_end - cable_start;

    // Check y-positions are not all zero (cosine envelope present)
    let mut has_nonzero_y = false;
    for i in cable_start..cable_end {
        if data.xpos[i][1].abs() > 0.01 {
            has_nonzero_y = true;
            break;
        }
    }
    let p1 = check(
        "Cosine envelope present (y not all zero)",
        has_nonzero_y,
        &format!("n_cable={n_cable}"),
    );

    // Check smoothness: no consecutive-segment angle > 30°
    let mut max_angle_deg = 0.0_f64;
    for i in (cable_start + 1)..(cable_end - 1) {
        let v1 = data.xpos[i] - data.xpos[i - 1];
        let v2 = data.xpos[i + 1] - data.xpos[i];
        let n1 = v1.norm();
        let n2 = v2.norm();
        if n1 > 1e-12 && n2 > 1e-12 {
            let cos_angle = v1.dot(&v2) / (n1 * n2);
            let angle = cos_angle.clamp(-1.0, 1.0).acos().to_degrees();
            max_angle_deg = max_angle_deg.max(angle);
        }
    }
    let p2 = check(
        "Smooth curve (max angle < 30°)",
        max_angle_deg < 30.0,
        &format!("max_angle={max_angle_deg:.1}°"),
    );

    (u32::from(p1) + u32::from(p2), 2)
}

// ── Check 8: Multiple cables don't interfere ────────────────────────────────

fn check_8_multi_cable_independence() -> (u32, u32) {
    // Cable A alone
    let xml_a = r#"
        <mujoco model="solo-A">
          <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
          <worldbody>
            <body name="anchor" pos="0 0 1">
              <composite type="cable" prefix="A" count="6 1 1"
                         initial="none" curve="l 0 0" size="0.8">
                <joint kind="main" damping="0.05"/>
                <geom type="capsule" size="0.01" rgba="0.8 0.2 0.1 1"
                      density="500"/>
              </composite>
            </body>
          </worldbody>
        </mujoco>
    "#;

    // Cable A + Cable B together
    let xml_ab = r#"
        <mujoco model="dual-AB">
          <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>
          <worldbody>
            <body name="anchor_A" pos="0 0 1">
              <composite type="cable" prefix="A" count="6 1 1"
                         initial="none" curve="l 0 0" size="0.8">
                <joint kind="main" damping="0.05"/>
                <geom type="capsule" size="0.01" rgba="0.8 0.2 0.1 1"
                      density="500"/>
              </composite>
            </body>
            <body name="anchor_B" pos="2 0 1">
              <composite type="cable" prefix="B" count="8 1 1"
                         initial="none" curve="l 0 0" size="0.8">
                <joint kind="main" damping="0.05"/>
                <geom type="capsule" size="0.01" rgba="0.2 0.8 0.1 1"
                      density="500"/>
              </composite>
            </body>
          </worldbody>
        </mujoco>
    "#;

    let model_a = load(xml_a);
    let mut data_a = model_a.make_data();
    step_seconds(&model_a, &mut data_a, 3.0);

    let model_ab = load(xml_ab);
    let mut data_ab = model_ab.make_data();
    step_seconds(&model_ab, &mut data_ab, 3.0);

    // Compare cable A body positions: solo vs dual
    // Solo: cable A bodies at indices 2..7 (world=0, anchor=1, 5 cable bodies)
    // Dual: cable A bodies at indices 2..7 (world=0, anchor_A=1, 5 cable A bodies)
    //       cable B bodies at 8..15 (anchor_B=7, 7 cable B bodies)
    let mut max_diff = 0.0_f64;
    let cable_a_count = model_a.nbody - 2; // 5 cable bodies
    for i in 0..cable_a_count {
        let solo_idx = i + 2;
        let dual_idx = i + 2;
        let diff = (data_a.xpos[solo_idx] - data_ab.xpos[dual_idx]).norm();
        max_diff = max_diff.max(diff);
    }

    let p = check(
        "Cable A independent of cable B",
        max_diff < 1e-10,
        &format!("max_pos_diff={max_diff:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Composites — Stress Test ===\n");

    let checks: &[(&str, fn() -> (u32, u32))] = &[
        ("Body count", check_1_body_count),
        ("Joint count and types", check_2_joint_count_and_types),
        ("Contact exclusions", check_3_contact_exclusions),
        ("Cable hangs below anchor", check_4_hangs_below_anchor),
        ("Convergence with segments", check_5_convergence),
        ("Length preservation", check_6_length_preservation),
        ("Cosine curve shape", check_7_curve_shape),
        ("Multi-cable independence", check_8_multi_cable_independence),
    ];

    let mut total = 0u32;
    let mut passed = 0u32;

    for (i, (label, func)) in checks.iter().enumerate() {
        println!("-- {}. {} --", i + 1, label);
        let (p, t) = func();
        passed += p;
        total += t;
        println!();
    }

    println!("============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
