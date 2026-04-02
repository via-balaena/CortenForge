//! Stress test — headless validation of all tendon system invariants.
//!
//! 16 checks covering: fixed tendon length/velocity, sensor readback,
//! spatial tendon length, sphere/cylinder wrap non-penetration, tendon limits,
//! pulley divisor scaling, tendon-driven actuators, spring/damper passive forces,
//! fixed vs spatial Jacobian properties, multi-tendon composition, and
//! configuration-dependent moment arms.
//!
//! Run: `cargo run -p example-tendon-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::suboptimal_flops
)]

use nalgebra::Vector3;

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Model A — Fixed tendon coupling two hinges (checks 1–4, 13).
/// Coefficients [1.0, −0.5]. Includes TendonPos and TendonVel sensors.
const MODEL_FIXED: &str = r#"
<mujoco model="fixed-tendon">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="a" pos="-0.3 0 1">
      <joint name="j_a" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
    </body>
    <body name="b" pos="0.3 0 1">
      <joint name="j_b" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t1">
      <joint joint="j_a" coef="1.0"/>
      <joint joint="j_b" coef="-0.5"/>
    </fixed>
  </tendon>
  <sensor>
    <tendonpos name="tp" tendon="t1"/>
    <tendonvel name="tv" tendon="t1"/>
  </sensor>
</mujoco>
"#;

/// Model B — Spatial tendon through 3 sites on a 2-link arm (checks 5, 14).
/// No wrapping — pure straight-line segments.
const MODEL_SPATIAL: &str = r#"
<mujoco model="spatial-tendon">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="upper" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
      <site name="s1" pos="0.06 0 -0.05"/>
      <site name="s2" pos="0.06 0 -0.45"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
        <site name="s3" pos="0.05 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1">
      <site site="s1"/>
      <site site="s2"/>
      <site site="s3"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model C — Sphere wrapping (check 6).
/// Sites on opposite sides of the sphere so the straight-line path crosses
/// through it, triggering wrapping. Zero gravity.
const MODEL_SPHERE: &str = r#"
<mujoco model="sphere-wrap">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="wrap_body" pos="0 0 0.8">
      <geom name="wrap_sphere" type="sphere" size="0.1"
            contype="0" conaffinity="0"/>
    </body>
    <body name="arm" pos="0 0 0.8">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.025" fromto="0 0 0 0.25 0 0" mass="0.5"/>
      <site name="s_origin" pos="0.2 0 0"/>
    </body>
    <body name="anchor" pos="-0.2 0 0.8">
      <site name="s_insert" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1">
      <site site="s_origin"/>
      <geom geom="wrap_sphere"/>
      <site site="s_insert"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model D — Cylinder wrapping (check 7).
/// Two bodies on opposite sides of a cylinder. Zero gravity.
const MODEL_CYLINDER: &str = r#"
<mujoco model="cylinder-wrap">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="cyl_body" pos="0 0 0.6">
      <geom name="wrap_cyl" type="cylinder" size="0.08 0.3"
            contype="0" conaffinity="0"/>
    </body>
    <body name="b1" pos="-0.3 0.1 0.8">
      <joint name="j1" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.02" mass="0.5"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="b2" pos="0.3 -0.05 0.4">
      <joint name="j2" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.02" mass="0.5"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1">
      <site site="s1"/>
      <geom geom="wrap_cyl"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model E — Tendon limits (checks 8, 9).
/// Spatial tendon with limited=true, range="0.2 0.8". Motor on shoulder.
const MODEL_LIMITS: &str = r#"
<mujoco model="tendon-limits">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="upper" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0" damping="1.0"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
      <site name="s1" pos="0.06 0 -0.05"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0" damping="1.0"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
        <site name="s2" pos="0.05 0 -0.35"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1" limited="true" range="0.2 0.9">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>
  <sensor>
    <tendonlimitfrc name="tlf" tendon="t1"/>
  </sensor>
  <actuator>
    <motor name="drive_sh" joint="shoulder" gear="5"/>
    <motor name="drive_el" joint="elbow" gear="5"/>
  </actuator>
</mujoco>
"#;

/// Model F — Pulley with divisor=2 (check 10).
/// Three bodies vertically: top (slide), middle (fixed), bottom (slide).
/// Branch 1: s1→s2, Branch 2: s3→s4 scaled by 1/2.
const MODEL_PULLEY: &str = r#"
<mujoco model="pulley">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="top" pos="0 0 1">
      <joint name="j_top" type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.02" mass="0.5"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="mid" pos="0 0 0.5">
      <geom type="sphere" size="0.01" mass="0.1" contype="0" conaffinity="0"/>
      <site name="s2" pos="0 0 0"/>
      <site name="s3" pos="0 0 0"/>
    </body>
    <body name="bot" pos="0 0 0">
      <joint name="j_bot" type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.02" mass="0.5"/>
      <site name="s4" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1">
      <site site="s1"/>
      <site site="s2"/>
      <pulley divisor="2"/>
      <site site="s3"/>
      <site site="s4"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Model G — Tendon-driven actuator (check 11).
/// Motor with tendon="t1" transmission on a 2-link arm.
const MODEL_ACTUATOR: &str = r#"
<mujoco model="tendon-actuator">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="upper" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
      <site name="s1" pos="0.06 0 -0.05"/>
      <body name="lower" pos="0 0 -0.5">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
        <site name="s2" pos="0.05 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>
  <actuator>
    <motor name="a1" tendon="t1" gear="50"/>
  </actuator>
</mujoco>
"#;

/// Model H — Fixed tendon with spring and damper (check 12).
/// stiffness=100, damping=5, coef=1.0, springlength="0" (rest at L=0).
const MODEL_SPRING: &str = r#"
<mujoco model="tendon-spring">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t1" stiffness="100" damping="5" springlength="0">
      <joint joint="j" coef="1.0"/>
    </fixed>
  </tendon>
</mujoco>
"#;

/// Model I — Two fixed tendons on the same joint (check 15).
/// t1: coef=2.0, t2: coef=-3.0.
const MODEL_MULTI: &str = r#"
<mujoco model="multi-tendon">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t1">
      <joint joint="j" coef="2.0"/>
    </fixed>
    <fixed name="t2">
      <joint joint="j" coef="-3.0"/>
    </fixed>
  </tendon>
</mujoco>
"#;

/// Model J — Spatial tendon on a hinge for moment arm check (check 16).
/// One site fixed in world, one on the rotating arm, so the tendon Jacobian
/// (moment arm) changes with joint angle.
const MODEL_MOMENT_ARM: &str = r#"
<mujoco model="moment-arm">
  <option gravity="0 0 0" timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <site name="s_world" pos="0 0 1.05"/>
    <body name="arm" pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0.3 0 0" mass="0.5"/>
      <site name="s_arm" pos="0.25 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t1">
      <site site="s_world"/>
      <site site="s_arm"/>
    </spatial>
  </tendon>
</mujoco>
"#;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn dist(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    (a - b).norm()
}

// ── Check 1: Fixed tendon length ───────────────────────────────────────────

/// L = 1.0 * qpos[j_a] + (-0.5) * qpos[j_b] — exact for fixed tendons.
fn check_1_fixed_length() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FIXED).expect("parse");
    let mut data = model.make_data();

    // Step to get non-zero joint positions
    for _ in 0..200 {
        data.step(&model).expect("step");
    }

    let ja = model.joint_id("j_a").expect("j_a");
    let jb = model.joint_id("j_b").expect("j_b");
    let qa = data.qpos[model.jnt_qpos_adr[ja]];
    let qb = data.qpos[model.jnt_qpos_adr[jb]];
    let expected = 1.0 * qa + (-0.5) * qb;
    let actual = data.ten_length[0];
    let err = (actual - expected).abs();

    let p = check(
        "Fixed tendon length",
        err < 1e-12,
        &format!("|error| = {err:.2e} < 1.00e-12"),
    );
    (u32::from(p), 1)
}

// ── Check 2: Fixed tendon velocity ─────────────────────────────────────────

/// V = 1.0 * qvel[j_a] + (-0.5) * qvel[j_b] — exact for fixed tendons.
fn check_2_fixed_velocity() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FIXED).expect("parse");
    let mut data = model.make_data();

    for _ in 0..200 {
        data.step(&model).expect("step");
    }

    let ja = model.joint_id("j_a").expect("j_a");
    let jb = model.joint_id("j_b").expect("j_b");
    let va = data.qvel[model.jnt_dof_adr[ja]];
    let vb = data.qvel[model.jnt_dof_adr[jb]];
    let expected = 1.0 * va + (-0.5) * vb;
    let actual = data.ten_velocity[0];
    let err = (actual - expected).abs();

    let p = check(
        "Fixed tendon velocity",
        err < 1e-12,
        &format!("|error| = {err:.2e} < 1.00e-12"),
    );
    (u32::from(p), 1)
}

// ── Check 3: TendonPos sensor matches data.ten_length ──────────────────────

fn check_3_sensor_pos() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FIXED).expect("parse");
    let mut data = model.make_data();

    for _ in 0..200 {
        data.step(&model).expect("step");
    }

    let sensor_val = data.sensor_scalar(&model, "tp").expect("sensor tp");
    let direct_val = data.ten_length[0];
    let err = (sensor_val - direct_val).abs();

    let p = check(
        "TendonPos sensor matches ten_length",
        err == 0.0,
        &format!("sensor = {sensor_val:.10}, direct = {direct_val:.10}, diff = {err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 4: TendonVel sensor matches data.ten_velocity ────────────────────

fn check_4_sensor_vel() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FIXED).expect("parse");
    let mut data = model.make_data();

    for _ in 0..200 {
        data.step(&model).expect("step");
    }

    let sensor_val = data.sensor_scalar(&model, "tv").expect("sensor tv");
    let direct_val = data.ten_velocity[0];
    let err = (sensor_val - direct_val).abs();

    let p = check(
        "TendonVel sensor matches ten_velocity",
        err == 0.0,
        &format!("sensor = {sensor_val:.10}, direct = {direct_val:.10}, diff = {err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 5: Spatial tendon length (no wrap) ───────────────────────────────

/// For a spatial tendon with only sites (no wrapping), the length equals the
/// sum of Euclidean distances between consecutive sites.
fn check_5_spatial_length() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SPATIAL).expect("parse");
    let mut data = model.make_data();

    // Step to get a non-trivial configuration
    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    let s1 = model.site_id("s1").expect("s1");
    let s2 = model.site_id("s2").expect("s2");
    let s3 = model.site_id("s3").expect("s3");
    let p1 = data.site_xpos[s1];
    let p2 = data.site_xpos[s2];
    let p3 = data.site_xpos[s3];
    let expected = dist(&p1, &p2) + dist(&p2, &p3);
    let actual = data.ten_length[0];
    let err = (actual - expected).abs();

    let p = check(
        "Spatial length = Σ segment distances",
        err < 1e-10,
        &format!("|error| = {err:.2e} < 1.00e-10 (L = {actual:.6})"),
    );
    (u32::from(p), 1)
}

// ── Check 6: Sphere wrap non-penetration ───────────────────────────────────

/// All wrap path points must lie at distance ≥ sphere radius from center.
fn check_6_sphere_non_penetration() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SPHERE).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let tid = 0;
    let adr = data.ten_wrapadr[tid];
    let num = data.ten_wrapnum[tid];

    // Find the wrapping geom (wrap_obj >= 0)
    let mut min_dist = f64::MAX;
    let mut geom_id = None;
    for i in 0..num {
        let obj = data.wrap_obj[adr + i];
        if obj >= 0 {
            let gid = obj as usize;
            geom_id = Some(gid);
            let center = data.geom_xpos[gid];
            let point = data.wrap_xpos[adr + i];
            let d = dist(&point, &center);
            if d < min_dist {
                min_dist = d;
            }
        }
    }

    let (pass, detail) = if let Some(gid) = geom_id {
        let radius = model.geom_size[gid].x;
        let margin = min_dist - radius;
        (
            margin >= -1e-6,
            format!(
                "min_dist = {min_dist:.8}, radius = {radius:.4}, margin = {margin:.2e} (wrap_points = {num})"
            ),
        )
    } else {
        (false, format!("No wrapping detected (wrapnum = {num})"))
    };

    let p = check("Sphere wrap: no penetration", pass, &detail);
    (u32::from(p), 1)
}

// ── Check 7: Cylinder wrap non-penetration ─────────────────────────────────

/// All wrap path points must lie at radial distance ≥ cylinder radius in the
/// cylinder's local XY plane.
fn check_7_cylinder_non_penetration() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_CYLINDER).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let tid = 0;
    let adr = data.ten_wrapadr[tid];
    let num = data.ten_wrapnum[tid];

    let mut min_radial = f64::MAX;
    let mut geom_id = None;
    for i in 0..num {
        let obj = data.wrap_obj[adr + i];
        if obj >= 0 {
            let gid = obj as usize;
            geom_id = Some(gid);
            let center = data.geom_xpos[gid];
            let mat = data.geom_xmat[gid];
            let p_local = mat.transpose() * (data.wrap_xpos[adr + i] - center);
            let radial = p_local.x.hypot(p_local.y);
            if radial < min_radial {
                min_radial = radial;
            }
        }
    }

    let (pass, detail) = if let Some(gid) = geom_id {
        let radius = model.geom_size[gid].x;
        let margin = min_radial - radius;
        (
            margin >= -1e-6,
            format!(
                "min_radial = {min_radial:.8}, radius = {radius:.4}, margin = {margin:.2e} (wrap_points = {num})"
            ),
        )
    } else {
        (false, format!("No wrapping detected (wrapnum = {num})"))
    };

    let p = check("Cylinder wrap: no penetration", pass, &detail);
    (u32::from(p), 1)
}

// ── Check 8: Tendon limit activates at range ───────────────────────────────

/// Drive the arm so the tendon stretches beyond the upper limit. The limit
/// constraint should produce a positive TendonLimitFrc.
fn check_8_limit_activates() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_LIMITS).expect("parse");
    let mut data = model.make_data();

    // Drive shoulder to extend the arm and stretch the tendon beyond range
    let mut max_frc = 0.0_f64;
    for step in 0..2000 {
        // Ramp up motor torque to stretch tendon
        data.ctrl[0] = if step < 500 { 0.0 } else { -10.0 };
        data.ctrl[1] = if step < 500 { 0.0 } else { 5.0 };
        data.step(&model).expect("step");
        max_frc = max_frc.max(data.ten_limit_frc[0]);
    }

    let p = check(
        "Tendon limit activates",
        max_frc > 0.0,
        &format!("peak TendonLimitFrc = {max_frc:.6}"),
    );
    (u32::from(p), 1)
}

// ── Check 9: No limit force when interior ──────────────────────────────────

/// At the initial configuration (tendon within range), the limit force must
/// be exactly zero.
fn check_9_no_limit_interior() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_LIMITS).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let length = data.ten_length[0];
    let range = model.tendon_range[0];
    let frc = data.ten_limit_frc[0];

    let interior = length > range.0 && length < range.1;
    let p = check(
        "No limit force when interior",
        interior && frc == 0.0,
        &format!(
            "L = {length:.4}, range = [{:.2}, {:.2}], interior = {interior}, frc = {frc:.6}",
            range.0, range.1
        ),
    );
    (u32::from(p), 1)
}

// ── Check 10: Pulley divisor scaling ───────────────────────────────────────

/// Total length = d(s1,s2) + d(s3,s4) / divisor.
fn check_10_pulley_scaling() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_PULLEY).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let s1 = model.site_id("s1").expect("s1");
    let s2 = model.site_id("s2").expect("s2");
    let s3 = model.site_id("s3").expect("s3");
    let s4 = model.site_id("s4").expect("s4");
    let branch1 = dist(&data.site_xpos[s1], &data.site_xpos[s2]);
    let branch2 = dist(&data.site_xpos[s3], &data.site_xpos[s4]);
    let divisor = 2.0;
    let expected = branch1 + branch2 / divisor;
    let actual = data.ten_length[0];
    let err = (actual - expected).abs();

    let p = check(
        "Pulley divisor scaling",
        err < 1e-10,
        &format!(
            "|error| = {err:.2e} (branch1 = {branch1:.4}, branch2/2 = {:.4}, total = {actual:.4})",
            branch2 / divisor
        ),
    );
    (u32::from(p), 1)
}

// ── Check 11: Tendon actuator produces force ───────────────────────────────

/// A tendon-driven actuator with nonzero ctrl should produce nonzero
/// qfrc_actuator on the driven DOFs.
fn check_11_tendon_actuator() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_ACTUATOR).expect("parse");
    let mut data = model.make_data();

    data.ctrl[0] = 1.0;
    data.step(&model).expect("step");

    // Check any DOF has nonzero actuator force (tendon may couple to elbow too)
    let mut max_frc = 0.0_f64;
    for d in 0..model.nv {
        let f = data.qfrc_actuator[d].abs();
        if f > max_frc {
            max_frc = f;
        }
    }

    let p = check(
        "Tendon actuator produces force",
        max_frc > 1e-10,
        &format!("max |qfrc_actuator| = {max_frc:.6} (ctrl = 1.0)"),
    );
    (u32::from(p), 1)
}

// ── Check 12: Spring/damper passive force ──────────────────────────────────

/// ten_force = -stiffness * (L - L0) - damping * V.
/// With springlength="0" → L0 = 0.
fn check_12_spring_damper() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SPRING).expect("parse");
    let mut data = model.make_data();

    // Set a known configuration
    let jid = model.joint_id("j").expect("j");
    data.qpos[model.jnt_qpos_adr[jid]] = 0.5;
    data.qvel[model.jnt_dof_adr[jid]] = 0.2;
    data.forward(&model).expect("forward");

    let length = data.ten_length[0]; // = 1.0 * 0.5 = 0.5
    let velocity = data.ten_velocity[0]; // = 1.0 * 0.2 = 0.2
    let stiffness = 100.0;
    let damping = 5.0;
    let l0 = 0.0; // springlength="0"
    let expected = -stiffness * (length - l0) - damping * velocity;
    let actual = data.ten_force[0];
    let err = (actual - expected).abs();

    let p = check(
        "Spring/damper passive force",
        err < 1e-8,
        &format!(
            "|error| = {err:.2e} (L = {length:.4}, V = {velocity:.4}, F = {actual:.4}, expected = {expected:.4})"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 13: Fixed Jacobian is constant ───────────────────────────────────

/// The Jacobian of a fixed tendon depends only on coefficients, not
/// configuration. It must be identical at two different joint positions.
fn check_13_fixed_jacobian_constant() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FIXED).expect("parse");
    let mut data = model.make_data();

    // Config A: initial position (qpos = 0)
    data.forward(&model).expect("forward");
    let j_a: Vec<f64> = data.ten_J[0].iter().copied().collect();

    // Config B: different position
    let ja = model.joint_id("j_a").expect("j_a");
    let jb = model.joint_id("j_b").expect("j_b");
    data.qpos[model.jnt_qpos_adr[ja]] = 0.8;
    data.qpos[model.jnt_qpos_adr[jb]] = -0.3;
    data.forward(&model).expect("forward");
    let j_b: Vec<f64> = data.ten_J[0].iter().copied().collect();

    let max_diff: f64 = j_a
        .iter()
        .zip(j_b.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);

    let p = check(
        "Fixed Jacobian is constant",
        max_diff == 0.0,
        &format!("max |J_A - J_B| = {max_diff:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 14: Spatial Jacobian varies ──────────────────────────────────────

/// The Jacobian of a spatial tendon is configuration-dependent. It must differ
/// between two distinct joint configurations.
fn check_14_spatial_jacobian_varies() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SPATIAL).expect("parse");
    let mut data = model.make_data();

    // Config A: initial
    data.forward(&model).expect("forward");
    let j_a: Vec<f64> = data.ten_J[0].iter().copied().collect();

    // Config B: different
    let sh = model.joint_id("shoulder").expect("shoulder");
    let el = model.joint_id("elbow").expect("elbow");
    data.qpos[model.jnt_qpos_adr[sh]] = 0.8;
    data.qpos[model.jnt_qpos_adr[el]] = -0.5;
    data.forward(&model).expect("forward");
    let j_b: Vec<f64> = data.ten_J[0].iter().copied().collect();

    let max_diff: f64 = j_a
        .iter()
        .zip(j_b.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);

    let p = check(
        "Spatial Jacobian varies with config",
        max_diff > 1e-6,
        &format!("max |J_A - J_B| = {max_diff:.6} > 1e-6"),
    );
    (u32::from(p), 1)
}

// ── Check 15: Multiple tendons compose correctly ───────────────────────────

/// Two fixed tendons on the same joint should each have independent correct
/// lengths: t1 = 2.0 * q, t2 = -3.0 * q.
fn check_15_multi_tendon() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_MULTI).expect("parse");
    let mut data = model.make_data();

    for _ in 0..200 {
        data.step(&model).expect("step");
    }

    let jid = model.joint_id("j").expect("j");
    let q = data.qpos[model.jnt_qpos_adr[jid]];

    let t1_id = model.tendon_id("t1").expect("t1");
    let t2_id = model.tendon_id("t2").expect("t2");
    let err1 = (data.ten_length[t1_id] - 2.0 * q).abs();
    let err2 = (data.ten_length[t2_id] - (-3.0 * q)).abs();

    let p = check(
        "Multiple tendons compose",
        err1 < 1e-12 && err2 < 1e-12,
        &format!(
            "t1 err = {err1:.2e}, t2 err = {err2:.2e} (q = {q:.4}, L1 = {:.4}, L2 = {:.4})",
            data.ten_length[t1_id], data.ten_length[t2_id]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 16: Moment arm varies with configuration ─────────────────────────

/// For a spatial tendon on a hinge, the Jacobian entry (moment arm) must
/// differ between two configurations.
fn check_16_moment_arm_varies() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_MOMENT_ARM).expect("parse");
    let mut data = model.make_data();
    let jid = model.joint_id("j").expect("j");
    let dof = model.jnt_dof_adr[jid];

    // Config A: qpos = 0
    data.forward(&model).expect("forward");
    let ma_a = data.ten_J[0][dof];

    // Config B: qpos = 1.0 rad
    data.qpos[model.jnt_qpos_adr[jid]] = 1.0;
    data.forward(&model).expect("forward");
    let ma_b = data.ten_J[0][dof];

    let diff = (ma_a - ma_b).abs();
    let p = check(
        "Moment arm varies with config",
        diff > 1e-6,
        &format!("J[dof] at q=0: {ma_a:.6}, at q=1: {ma_b:.6}, diff = {diff:.6}"),
    );
    (u32::from(p), 1)
}

// ── Main ──────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Tendons — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("Fixed tendon length", check_1_fixed_length),
        ("Fixed tendon velocity", check_2_fixed_velocity),
        ("TendonPos sensor readback", check_3_sensor_pos),
        ("TendonVel sensor readback", check_4_sensor_vel),
        ("Spatial length (no wrap)", check_5_spatial_length),
        (
            "Sphere wrap non-penetration",
            check_6_sphere_non_penetration,
        ),
        (
            "Cylinder wrap non-penetration",
            check_7_cylinder_non_penetration,
        ),
        ("Tendon limit activates", check_8_limit_activates),
        ("No limit force when interior", check_9_no_limit_interior),
        ("Pulley divisor scaling", check_10_pulley_scaling),
        ("Tendon actuator produces force", check_11_tendon_actuator),
        ("Spring/damper passive force", check_12_spring_damper),
        (
            "Fixed Jacobian is constant",
            check_13_fixed_jacobian_constant,
        ),
        (
            "Spatial Jacobian varies with config",
            check_14_spatial_jacobian_varies,
        ),
        ("Multiple tendons compose", check_15_multi_tendon),
        ("Moment arm varies with config", check_16_moment_arm_varies),
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
