//! Stress test — headless validation of all contact filtering invariants.
//!
//! 12 checks covering: contype/conaffinity bitmask rule (AND + OR),
//! cross-layer filtering, parent-child auto-exclusion, world body exemption,
//! explicit `<exclude>` suppression, and `<pair>` bitmask bypass with condim
//! override.
//!
//! Run: `cargo run -p example-contact-filtering-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Model A — Bitmask basics (checks 1–3).
/// Two spheres + ground plane.
/// "collider": default contype=1, conaffinity=1 → collides with ground.
/// "ghost": contype=0, conaffinity=0 → collides with nothing.
const MODEL_A: &str = r#"
<mujoco model="bitmask-basics">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="collider" pos="-0.5 0 1">
      <freejoint name="collider_free"/>
      <geom name="collider_geom" type="sphere" size="0.1" mass="1.0"/>
    </body>
    <body name="ghost" pos="0.5 0 1">
      <freejoint name="ghost_free"/>
      <geom name="ghost_geom" type="sphere" size="0.1" mass="1.0"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model A2 — OR-rule verification (check 4).
/// Sphere with contype=0, conaffinity=1.
/// Ground: contype=1, conaffinity=1.
/// Rule: (0 & 1) || (1 & 1) = 0 || 1 = true → collides.
const MODEL_A2: &str = r#"
<mujoco model="or-rule">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 1">
      <freejoint name="ball_free"/>
      <geom name="ball_geom" type="sphere" size="0.1" mass="1.0"
            contype="0" conaffinity="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model B — Selective bitmask (checks 5–6).
/// Three spheres near the ground:
/// - sphere_a: contype=1, conaffinity=3 (bits 0+1)
/// - sphere_b: contype=2, conaffinity=2 (bit 1)
/// - sphere_c: contype=4, conaffinity=4 (bit 2)
///
/// A↔B: (1 & 2) || (2 & 3) = 0 || 2 = true  → collide
/// B↔C: (2 & 4) || (4 & 2) = 0 || 0 = false → no collide
const MODEL_B: &str = r#"
<mujoco model="selective-bitmask">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="sphere_a" pos="0 0 0.15">
      <freejoint name="a_free"/>
      <geom name="a_geom" type="sphere" size="0.1" mass="1.0"
            contype="1" conaffinity="3"/>
    </body>
    <body name="sphere_b" pos="0 0 0.35">
      <freejoint name="b_free"/>
      <geom name="b_geom" type="sphere" size="0.1" mass="1.0"
            contype="2" conaffinity="2"/>
    </body>
    <body name="sphere_c" pos="0.5 0 0.15">
      <freejoint name="c_free"/>
      <geom name="c_geom" type="sphere" size="0.1" mass="1.0"
            contype="4" conaffinity="4"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model C — Parent-child auto-exclusion (checks 7–8).
/// Non-world parent body (via freejoint) with a hinge child.
/// Parent and child geoms overlap spatially.
/// Parent-child filter: body_parent\[child\] = parent, both non-zero → excluded.
/// Ground plane (body 0) is exempt from parent-child filter → collides with child.
const MODEL_C: &str = r#"
<mujoco model="parent-child">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="parent_body" pos="0 0 1">
      <freejoint name="parent_free"/>
      <geom name="parent_geom" type="box" size="0.2 0.2 0.2" mass="1.0"/>
      <body name="child_body" pos="0 0 0">
        <joint name="child_hinge" type="hinge" axis="0 1 0"/>
        <geom name="child_geom" type="box" size="0.15 0.15 0.15" mass="0.5"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model D — Explicit exclusion (checks 9–10).
/// Two overlapping free bodies with `<exclude>`.
/// Third body with no exclusion collides normally.
const MODEL_D: &str = r#"
<mujoco model="exclude-pairs">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="box_a" pos="0 0 0.5">
      <freejoint name="a_free"/>
      <geom name="a_geom" type="box" size="0.15 0.15 0.15" mass="1.0"/>
    </body>
    <body name="box_b" pos="0 0 0.7">
      <freejoint name="b_free"/>
      <geom name="b_geom" type="box" size="0.15 0.15 0.15" mass="1.0"/>
    </body>
    <body name="box_c" pos="0.5 0 0.5">
      <freejoint name="c_free"/>
      <geom name="c_geom" type="box" size="0.15 0.15 0.15" mass="1.0"/>
    </body>
  </worldbody>
  <contact>
    <exclude body1="box_a" body2="box_b"/>
  </contact>
</mujoco>
"#;

/// Model D2 — Same as D but with exclude body order reversed.
/// Verifies bidirectionality (check 10).
const MODEL_D2: &str = r#"
<mujoco model="exclude-reversed">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="box_a" pos="0 0 0.5">
      <freejoint name="a_free"/>
      <geom name="a_geom" type="box" size="0.15 0.15 0.15" mass="1.0"/>
    </body>
    <body name="box_b" pos="0 0 0.7">
      <freejoint name="b_free"/>
      <geom name="b_geom" type="box" size="0.15 0.15 0.15" mass="1.0"/>
    </body>
  </worldbody>
  <contact>
    <exclude body1="box_b" body2="box_a"/>
  </contact>
</mujoco>
"#;

/// Model E — Explicit pair override (checks 11–12).
/// Two geoms with contype=1, conaffinity=0 → bitmask fails:
///   (1 & 0) || (1 & 0) = false.
/// An explicit `<pair>` bypasses bitmask and sets condim=1 (frictionless).
const MODEL_E: &str = r#"
<mujoco model="pair-override">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="left" pos="-0.05 0 1">
      <freejoint name="left_free"/>
      <geom name="left_geom" type="sphere" size="0.1" mass="1.0"
            contype="1" conaffinity="0"/>
    </body>
    <body name="right" pos="0.05 0 1">
      <freejoint name="right_free"/>
      <geom name="right_geom" type="sphere" size="0.1" mass="1.0"
            contype="1" conaffinity="0"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="left_geom" geom2="right_geom" condim="1" friction="0 0 0 0 0"/>
  </contact>
</mujoco>
"#;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

// ── Check 1: contype=0 falls through ground ──────────────────────────────

fn check_1_ghost_falls_through() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let ghost_id = model.body_id("ghost").expect("ghost body");

    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    let z = data.xpos[ghost_id].z;
    let p = check(
        "contype=0 falls through ground",
        z < -1.0,
        &format!("ghost z = {z:.4} (expected < -1.0)"),
    );
    (u32::from(p), 1)
}

// ── Check 2: Default bitmask collides ────────────────────────────────────

fn check_2_default_collides() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let collider_id = model.body_id("collider").expect("collider body");

    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    let z = data.xpos[collider_id].z;
    let radius = 0.1;
    let tol = 0.02;
    let p = check(
        "Default bitmask collides",
        (z - radius).abs() < tol,
        &format!("collider z = {z:.4} (expected ≈ {radius})"),
    );
    (u32::from(p), 1)
}

// ── Check 3: ncon=0 for ghost ────────────────────────────────────────────

fn check_3_ghost_no_contacts() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let ghost_geom = model.geom_id("ghost_geom").expect("ghost_geom");

    // Step a few times so collision detection runs
    for _ in 0..50 {
        data.step(&model).expect("step");
    }

    let ghost_contacts = data.contacts_involving_geom(ghost_geom);
    let p = check(
        "ncon=0 for ghost",
        ghost_contacts == 0,
        &format!("contacts involving ghost geom = {ghost_contacts}"),
    );
    (u32::from(p), 1)
}

// ── Check 4: OR rule — contype=0 still collides if conaffinity matches ──

fn check_4_or_rule() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A2).expect("parse");
    let mut data = model.make_data();

    let ball_id = model.body_id("ball").expect("ball body");

    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    let z = data.xpos[ball_id].z;
    let radius = 0.1;
    let tol = 0.02;
    let p = check(
        "OR rule — contype=0 collides if conaffinity matches",
        (z - radius).abs() < tol,
        &format!("ball z = {z:.4} (expected ≈ {radius}, rule: (0&1)||(1&1) = true)"),
    );
    (u32::from(p), 1)
}

// ── Check 5: Cross-layer bitmask collides ────────────────────────────────

fn check_5_cross_layer_collides() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_B).expect("parse");
    let mut data = model.make_data();

    let a_geom = model.geom_id("a_geom").expect("a_geom");
    let b_geom = model.geom_id("b_geom").expect("b_geom");

    // Spheres A (z=0.15) and B (z=0.35) are stacked, both radius 0.1.
    // Gap = 0.35 - 0.15 = 0.20, sum of radii = 0.20 → touching.
    // Let them settle under gravity.
    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    let ab_contacts = data.contacts_between_geoms(a_geom, b_geom);
    let p = check(
        "Cross-layer bitmask collides",
        ab_contacts > 0,
        &format!(
            "A↔B contacts = {ab_contacts} (A.ct=1,ca=3 vs B.ct=2,ca=2: (1&2)||(2&3) = 0||2 = true)"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 6: Disjoint layers no contact ──────────────────────────────────

fn check_6_disjoint_no_contact() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_B).expect("parse");
    let mut data = model.make_data();

    let b_geom = model.geom_id("b_geom").expect("b_geom");
    let c_geom = model.geom_id("c_geom").expect("c_geom");

    // B and C start at different X positions (0 vs 0.5), so they don't
    // initially overlap. Move them together to force potential overlap.
    let b_body = model.body_id("sphere_b").expect("sphere_b");
    let c_body = model.body_id("sphere_c").expect("sphere_c");

    // Set both at same position via qpos (freejoint: 3 pos + 4 quat)
    let b_qadr = model.jnt_qpos_adr[model.body_jnt_adr[b_body]];
    let c_qadr = model.jnt_qpos_adr[model.body_jnt_adr[c_body]];
    data.qpos[b_qadr] = 0.25; // x
    data.qpos[b_qadr + 1] = 0.0;
    data.qpos[b_qadr + 2] = 0.15;
    data.qpos[c_qadr] = 0.25; // same x → overlapping
    data.qpos[c_qadr + 1] = 0.0;
    data.qpos[c_qadr + 2] = 0.15;

    data.forward(&model).expect("forward");
    data.step(&model).expect("step");

    let bc_contacts = data.contacts_between_geoms(b_geom, c_geom);
    let p = check(
        "Disjoint layers no contact",
        bc_contacts == 0,
        &format!(
            "B↔C contacts = {bc_contacts} (B.ct=2,ca=2 vs C.ct=4,ca=4: (2&4)||(4&2) = 0||0 = false)"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 7: Parent-child auto-excluded ──────────────────────────────────

fn check_7_parent_child_excluded() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_C).expect("parse");
    let mut data = model.make_data();

    let parent_body = model.body_id("parent_body").expect("parent_body");
    let child_body = model.body_id("child_body").expect("child_body");

    // Confirm parent-child relationship
    let child_parent = model.body_parent[child_body];
    assert_eq!(
        child_parent, parent_body,
        "child's parent should be parent_body"
    );
    assert_ne!(parent_body, 0, "parent_body should not be world body");

    data.forward(&model).expect("forward");
    data.step(&model).expect("step");

    let pc_contacts = data.contacts_between_bodies(&model, parent_body, child_body);
    let p = check(
        "Parent-child auto-excluded",
        pc_contacts == 0,
        &format!(
            "parent↔child contacts = {pc_contacts} (both non-world, body_parent[{child_body}] = {parent_body})"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 8: World body exempt from auto-exclusion ───────────────────────

fn check_8_world_body_exempt() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_C).expect("parse");
    let mut data = model.make_data();

    let parent_body = model.body_id("parent_body").expect("parent_body");
    let ground_geom = model.geom_id("ground").expect("ground");

    // Drop the parent body so it reaches the ground plane
    let jnt_idx = model.body_jnt_adr[parent_body];
    let qadr = model.jnt_qpos_adr[jnt_idx];
    data.qpos[qadr + 2] = 0.25; // low enough to hit ground

    for _ in 0..200 {
        data.step(&model).expect("step");
    }

    // Check for contacts between ground (body 0) and the parent or child.
    // body_parent[parent_body] = 0 (world), but world is exempt from
    // parent-child filter, so ground plane should collide.
    let ground_contacts = data.contacts_involving_geom(ground_geom);
    let p = check(
        "World body exempt from auto-exclusion",
        ground_contacts > 0,
        &format!(
            "contacts involving ground = {ground_contacts} (world body exempt from parent-child filter)"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 9: Exclude suppresses contact ──────────────────────────────────

fn check_9_exclude_suppresses() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_D).expect("parse");
    let mut data = model.make_data();

    let body_a = model.body_id("box_a").expect("box_a");
    let body_b = model.body_id("box_b").expect("box_b");
    let body_c = model.body_id("box_c").expect("box_c");

    // box_a (z=0.5) and box_b (z=0.7) overlap (half-size 0.15 each,
    // gap = 0.2, sum of half-sizes = 0.3 > 0.2 → overlapping).
    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    let ab_contacts = data.contacts_between_bodies(&model, body_a, body_b);
    let ac_contacts = data.contacts_between_bodies(&model, body_a, body_c);

    // box_a ↔ box_c should be 0 because they're at different X (0 vs 0.5)
    // and don't overlap. We mainly care that ab = 0.
    let p = check(
        "Exclude suppresses contact",
        ab_contacts == 0,
        &format!("box_a↔box_b contacts = {ab_contacts} (excluded), box_a↔box_c = {ac_contacts}"),
    );
    (u32::from(p), 1)
}

// ── Check 10: Exclude is bidirectional ───────────────────────────────────

fn check_10_exclude_bidirectional() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_D2).expect("parse");
    let mut data = model.make_data();

    let body_a = model.body_id("box_a").expect("box_a");
    let body_b = model.body_id("box_b").expect("box_b");

    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    let ab_contacts = data.contacts_between_bodies(&model, body_a, body_b);
    let p = check(
        "Exclude is bidirectional",
        ab_contacts == 0,
        &format!(
            "box_a↔box_b contacts = {ab_contacts} (exclude body1=box_b body2=box_a, reversed order)"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 11: Pair bypasses bitmask ──────────────────────────────────────

fn check_11_pair_bypasses_bitmask() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_E).expect("parse");
    let mut data = model.make_data();

    let left_geom = model.geom_id("left_geom").expect("left_geom");
    let right_geom = model.geom_id("right_geom").expect("right_geom");

    // Spheres at x=-0.05 and x=0.05, radius 0.1 each → overlapping.
    // Bitmask: (1&0)||(1&0) = false, but <pair> bypasses bitmask.
    data.forward(&model).expect("forward");
    data.step(&model).expect("step");

    let lr_contacts = data.contacts_between_geoms(left_geom, right_geom);
    let p = check(
        "Pair bypasses bitmask",
        lr_contacts > 0,
        &format!("left↔right contacts = {lr_contacts} (bitmask would reject, <pair> bypasses)"),
    );
    (u32::from(p), 1)
}

// ── Check 12: Pair overrides condim ──────────────────────────────────────

fn check_12_pair_condim() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_E).expect("parse");
    let mut data = model.make_data();

    let left_geom = model.geom_id("left_geom").expect("left_geom");
    let right_geom = model.geom_id("right_geom").expect("right_geom");

    data.forward(&model).expect("forward");
    data.step(&model).expect("step");

    // Find the contact between left and right, check its dim
    let pair_contact = data.contacts.iter().find(|c| {
        (c.geom1 == left_geom && c.geom2 == right_geom)
            || (c.geom1 == right_geom && c.geom2 == left_geom)
    });

    let (found, dim) = pair_contact.map_or((false, 0), |c| (true, c.dim));

    let p = check(
        "Pair overrides condim",
        found && dim == 1,
        &format!(
            "contact found = {found}, dim = {dim} (expected 1 = frictionless from <pair condim=\"1\">)"
        ),
    );
    (u32::from(p), 1)
}

// ── Main ──────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Contact Filtering — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        (
            "contype=0 falls through ground",
            check_1_ghost_falls_through,
        ),
        ("Default bitmask collides", check_2_default_collides),
        ("ncon=0 for ghost", check_3_ghost_no_contacts),
        (
            "OR rule — contype=0 collides if conaffinity matches",
            check_4_or_rule,
        ),
        ("Cross-layer bitmask collides", check_5_cross_layer_collides),
        ("Disjoint layers no contact", check_6_disjoint_no_contact),
        ("Parent-child auto-excluded", check_7_parent_child_excluded),
        (
            "World body exempt from auto-exclusion",
            check_8_world_body_exempt,
        ),
        ("Exclude suppresses contact", check_9_exclude_suppresses),
        ("Exclude is bidirectional", check_10_exclude_bidirectional),
        ("Pair bypasses bitmask", check_11_pair_bypasses_bitmask),
        ("Pair overrides condim", check_12_pair_condim),
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
