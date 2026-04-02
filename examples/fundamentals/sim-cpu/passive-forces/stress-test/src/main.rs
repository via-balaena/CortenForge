//! Stress test — headless validation of all passive force invariants.
//!
//! 18 checks covering: inertia-box fluid drag (terminal velocity, v² scaling,
//! zero medium, viscous linearity), ellipsoid fluid model (activation, shape
//! dependence, interaction coefficient), wind (stationary body, cancellation,
//! direction), spring mechanics (restoring force, springref, gravity equilibrium),
//! damper mechanics (dissipation, proportionality), ImplicitSpringDamper
//! suppression, and DISABLE_SPRING / DISABLE_DAMPER flags.
//!
//! Run: `cargo run -p example-passive-stress-test --release`

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

/// Model A — Inertia-box fluid drag (checks 1–2).
/// Single free sphere in a dense medium with zero viscosity.
/// Pure quadratic drag: F_drag = 0.5 * rho * bx * by * v^2.
const MODEL_A: &str = r#"
<mujoco model="inertia-box-drag">
  <option gravity="0 0 -9.81" timestep="0.001"
          density="1.2" viscosity="0"/>
  <worldbody>
    <body name="sphere" pos="0 0 100">
      <freejoint name="sphere_free"/>
      <geom name="sphere_geom" type="sphere" size="0.15" mass="2.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model B — Zero medium (check 3).
/// density=0, viscosity=0 → no fluid forces.
const MODEL_B: &str = r#"
<mujoco model="zero-medium">
  <option gravity="0 0 -9.81" timestep="0.002"
          density="0" viscosity="0"/>
  <worldbody>
    <body name="sphere" pos="0 0 10">
      <freejoint name="sphere_free"/>
      <geom type="sphere" size="0.15" mass="2.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model C — Pure viscous drag (check 4).
/// density=0, viscosity > 0 → Stokes drag only (linear in v).
const MODEL_C: &str = r#"
<mujoco model="viscous-only">
  <option gravity="0 0 0" timestep="0.002"
          density="0" viscosity="1.0"/>
  <worldbody>
    <body name="sphere" pos="0 0 0">
      <freejoint name="sphere_free"/>
      <geom type="sphere" size="0.15" mass="2.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model D — Ellipsoid fluid model (checks 5–7).
/// Per-geom fluidshape="ellipsoid" activates the advanced 5-component model.
const MODEL_D: &str = r#"
<mujoco model="ellipsoid-drag">
  <option gravity="0 0 -9.81" timestep="0.002"
          density="50.0" viscosity="0.1"/>
  <worldbody>
    <body name="sphere" pos="0 0 10">
      <freejoint name="sphere_free"/>
      <geom name="sphere_geom" type="sphere" size="0.15" mass="2.0"
            fluidshape="ellipsoid"/>
    </body>
    <body name="cylinder" pos="2 0 10">
      <freejoint name="cyl_free"/>
      <geom name="cyl_geom" type="cylinder" size="0.3 0.05" mass="2.0"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model E — Wind on stationary body (checks 8, 10).
/// Body at rest, wind blows in +X. Fluid force should have x-component.
const MODEL_E: &str = r#"
<mujoco model="wind-stationary">
  <option gravity="0 0 0" timestep="0.002"
          density="1.2" viscosity="1.5e-5" wind="10 0 0"/>
  <worldbody>
    <body name="sphere" pos="0 0 0">
      <freejoint name="sphere_free"/>
      <geom type="sphere" size="0.2" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model E2 — Wind in +Y (check 10).
const MODEL_E2: &str = r#"
<mujoco model="wind-y">
  <option gravity="0 0 0" timestep="0.002"
          density="1.2" viscosity="1.5e-5" wind="0 10 0"/>
  <worldbody>
    <body name="sphere" pos="0 0 0">
      <freejoint name="sphere_free"/>
      <geom type="sphere" size="0.2" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model F — Spring restoring force (checks 11–12).
/// Horizontal slide joint with stiffness, no gravity.
const MODEL_F: &str = r#"
<mujoco model="spring-force">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <body name="slider" pos="0 0 0">
      <joint name="slide_j" type="slide" axis="1 0 0"
             stiffness="100" damping="0" springref="0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model F2 — Springref != qpos0 (check 12).
/// springref=0.5, initial qpos=0 → spring pulls toward 0.5.
const MODEL_F2: &str = r#"
<mujoco model="springref-offset">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <body name="slider" pos="0 0 0">
      <joint name="slide_j" type="slide" axis="1 0 0"
             stiffness="100" damping="0" springref="0.5"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model G — Spring + gravity equilibrium (check 13).
/// Vertical slide (Z-axis) with spring. Equilibrium: q = springref - mg/k.
/// (negative because gravity is -Z, spring restores toward springref)
const MODEL_G: &str = r#"
<mujoco model="spring-gravity">
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="slider" pos="0 0 1">
      <joint name="slide_j" type="slide" axis="0 0 1"
             stiffness="200" damping="5.0" springref="0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model H — Damper dissipation (check 14).
/// Hinge with damping, no spring. Energy should decrease monotonically.
const MODEL_H: &str = r#"
<mujoco model="damper-dissipation">
  <option gravity="0 0 -9.81" timestep="0.001">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="arm" pos="0 0 2">
      <joint name="hinge_j" type="hinge" axis="0 1 0"
             stiffness="0" damping="2.0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.04" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model I — Damper force proportionality (check 15).
/// Hinge with known damping, no spring, no gravity.
const MODEL_I: &str = r#"
<mujoco model="damper-proportional">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge_j" type="hinge" axis="0 1 0"
             stiffness="0" damping="5.0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.04" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model J — ImplicitSpringDamper suppression (check 16).
/// Spring + damper + fluid. In implicitspringdamper mode, qfrc_spring and
/// qfrc_damper should be zero, but qfrc_fluid should be nonzero.
const MODEL_J: &str = r#"
<mujoco model="implicit-suppression">
  <option gravity="0 0 -9.81" timestep="0.001"
          integrator="implicitspringdamper" density="1.2" viscosity="0.01"/>
  <worldbody>
    <body name="slider" pos="0 0 1">
      <joint name="slide_j" type="slide" axis="0 0 1"
             stiffness="100" damping="5.0" springref="0"/>
      <geom type="sphere" size="0.15" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model K — DISABLE_SPRING (check 17).
/// Spring + damper, spring disabled via flag.
const MODEL_K: &str = r#"
<mujoco model="disable-spring">
  <option gravity="0 0 0" timestep="0.002">
    <flag spring="disable"/>
  </option>
  <worldbody>
    <body name="slider" pos="0 0 0">
      <joint name="slide_j" type="slide" axis="1 0 0"
             stiffness="100" damping="5.0" springref="0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model L — DISABLE_DAMPER (check 18).
/// Spring + damper, damper disabled via flag.
const MODEL_L: &str = r#"
<mujoco model="disable-damper">
  <option gravity="0 0 0" timestep="0.002">
    <flag damper="disable"/>
  </option>
  <worldbody>
    <body name="slider" pos="0 0 0">
      <joint name="slide_j" type="slide" axis="1 0 0"
             stiffness="100" damping="5.0" springref="0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

/// Compute equivalent box side length from body inertia along one axis.
/// bx = sqrt((Iy + Iz - Ix) / m * 6), etc.
fn inertia_box_dim(inertia: &nalgebra::Vector3<f64>, mass: f64, axis: usize) -> f64 {
    let i1 = inertia[(axis + 1) % 3];
    let i2 = inertia[(axis + 2) % 3];
    let ia = inertia[axis];
    ((i1 + i2 - ia).max(1e-10) / mass * 6.0).sqrt()
}

// ── Check 1: Terminal velocity magnitude ───────────────────────────────────

fn check_01_terminal_velocity() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let body_id = model.body_id("sphere").expect("sphere body");
    let mass = model.body_mass[body_id];
    let inertia = &model.body_inertia[body_id];
    let rho = model.density;
    let g = 9.81;

    // Equivalent box face area for Z-axis drag
    let bx = inertia_box_dim(inertia, mass, 0);
    let by = inertia_box_dim(inertia, mass, 1);
    let v_t_analytical = (2.0 * mass * g / (rho * bx * by)).sqrt();

    // Simulate for 20 seconds (20000 steps at dt=0.001)
    for _ in 0..20_000 {
        data.step(&model).expect("step");
    }

    // Free joint: DOF 0-2 = translational, DOF 3-5 = rotational
    // Z velocity is DOF index 2 (but for free joint, dof_adr + 2 is the z vel)
    let dof_adr = model.body_dof_adr[body_id];
    let vz = -data.qvel[dof_adr + 2]; // negate: falling is negative z
    let err_pct = ((vz - v_t_analytical) / v_t_analytical * 100.0).abs();

    let p = check(
        "Terminal velocity magnitude",
        err_pct < 5.0,
        &format!("measured={vz:.4} m/s, analytical={v_t_analytical:.4} m/s, err={err_pct:.2}%"),
    );
    (u32::from(p), 1)
}

// ── Check 2: Drag proportional to v² ──────────────────────────────────────

fn check_02_drag_v_squared() -> (u32, u32) {
    // Instead of simulating to a specific velocity, directly set velocity and
    // read the instantaneous drag force. This avoids timing / drift issues.
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");

    let body_id = model.body_id("sphere").expect("sphere body");
    let dof_adr = model.body_dof_adr[body_id];

    // Measure drag at v1 and v2 = 2*v1. Expect F(2v)/F(v) = 4 (quadratic).
    let v1 = 5.0;
    let v2 = 10.0;

    let mut data1 = model.make_data();
    data1.qvel[dof_adr + 2] = -v1; // downward
    data1.forward(&model).expect("forward");
    let f1 = data1.qfrc_fluid[dof_adr + 2]; // positive = upward drag

    let mut data2 = model.make_data();
    data2.qvel[dof_adr + 2] = -v2;
    data2.forward(&model).expect("forward");
    let f2 = data2.qfrc_fluid[dof_adr + 2];

    // Quadratic drag: F ∝ v². F(2v)/F(v) should be 4.0.
    // (viscosity=0, so only quadratic term contributes)
    let ratio = if f1.abs() > 1e-15 { f2 / f1 } else { 0.0 };
    let err_pct = ((ratio - 4.0) / 4.0 * 100.0).abs();

    let p = check(
        "Drag proportional to v² (quadratic)",
        err_pct < 5.0,
        &format!(
            "F(v={v1})={f1:.6}, F(v={v2})={f2:.6}, ratio={ratio:.4}, expected=4.0, err={err_pct:.2}%"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 3: Zero medium = zero drag ──────────────────────────────────────

fn check_03_zero_medium() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_B).expect("parse");
    let mut data = model.make_data();

    // Step a few times so body has velocity
    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    data.forward(&model).expect("forward");

    let max_fluid = data
        .qfrc_fluid
        .iter()
        .map(|v| v.abs())
        .fold(0.0f64, f64::max);
    let p = check(
        "Zero medium = zero drag",
        max_fluid == 0.0,
        &format!("max |qfrc_fluid| = {max_fluid:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 4: Viscous drag linear in velocity ──────────────────────────────

fn check_04_viscous_linearity() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_C).expect("parse");

    // Test at two velocities: v and 2v
    let dof_adr = model.body_dof_adr[model.body_id("sphere").expect("sphere")];
    let v1 = 1.0;
    let v2 = 2.0;

    // Measure force at v1
    let mut data1 = model.make_data();
    data1.qvel[dof_adr + 2] = v1; // z velocity
    data1.forward(&model).expect("forward");
    let f1 = data1.qfrc_fluid[dof_adr + 2];

    // Measure force at v2
    let mut data2 = model.make_data();
    data2.qvel[dof_adr + 2] = v2;
    data2.forward(&model).expect("forward");
    let f2 = data2.qfrc_fluid[dof_adr + 2];

    // Linear: doubling v should double force. Ratio f2/f1 ≈ 2.0.
    let ratio = if f1.abs() > 1e-15 { f2 / f1 } else { 0.0 };
    let err_pct = ((ratio - 2.0) / 2.0 * 100.0).abs();

    let p = check(
        "Viscous drag linear in velocity",
        err_pct < 5.0,
        &format!("f(v)={f1:.6}, f(2v)={f2:.6}, ratio={ratio:.4}, err={err_pct:.2}%"),
    );
    (u32::from(p), 1)
}

// ── Check 5: Ellipsoid activates on fluidshape ────────────────────────────

fn check_05_ellipsoid_activates() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_D).expect("parse");
    let mut data = model.make_data();

    // Step so the sphere gains velocity
    for _ in 0..100 {
        data.step(&model).expect("step");
    }
    data.forward(&model).expect("forward");

    let dof_adr = model.body_dof_adr[model.body_id("sphere").expect("sphere")];
    let fluid_z = data.qfrc_fluid[dof_adr + 2];

    // Also verify geom_fluid[0] was set to 1.0 (ellipsoid activated)
    let geom_id = model.geom_id("sphere_geom").expect("sphere_geom");
    let interaction_coef = model.geom_fluid[geom_id][0];

    let p = check(
        "Ellipsoid activates on fluidshape",
        fluid_z.abs() > 1e-6 && (interaction_coef - 1.0).abs() < 1e-10,
        &format!("qfrc_fluid_z={fluid_z:.6}, interaction_coef={interaction_coef:.1}"),
    );
    (u32::from(p), 1)
}

// ── Check 6: Shape dependence ─────────────────────────────────────────────

fn check_06_shape_dependence() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_D).expect("parse");
    let mut data = model.make_data();

    let sphere_body = model.body_id("sphere").expect("sphere");
    let cyl_body = model.body_id("cylinder").expect("cylinder");
    let sphere_dof = model.body_dof_adr[sphere_body];
    let cyl_dof = model.body_dof_adr[cyl_body];

    // Give both the same downward velocity
    data.qvel[sphere_dof + 2] = -5.0;
    data.qvel[cyl_dof + 2] = -5.0;
    data.forward(&model).expect("forward");

    // Flat cylinder (r=0.3, h=0.05) has larger frontal area than sphere (r=0.15)
    // when falling in Z → higher drag magnitude
    let sphere_drag = data.qfrc_fluid[sphere_dof + 2].abs();
    let cyl_drag = data.qfrc_fluid[cyl_dof + 2].abs();

    let p = check(
        "Shape dependence — cylinder > sphere drag",
        cyl_drag > sphere_drag,
        &format!("sphere |drag|={sphere_drag:.4}, cylinder |drag|={cyl_drag:.4}"),
    );
    (u32::from(p), 1)
}

// ── Check 7: Interaction coefficient scaling ──────────────────────────────

fn check_07_interaction_scaling() -> (u32, u32) {
    let mut model = sim_mjcf::load_model(MODEL_D).expect("parse");
    let geom_id = model.geom_id("sphere_geom").expect("sphere_geom");
    let body_id = model.body_id("sphere").expect("sphere");
    let dof_adr = model.body_dof_adr[body_id];

    // Measure force with interaction_coef = 1.0
    model.geom_fluid[geom_id][0] = 1.0;
    let mut data1 = model.make_data();
    data1.qvel[dof_adr + 2] = -5.0;
    data1.forward(&model).expect("forward");
    let f1 = data1.qfrc_fluid[dof_adr + 2];

    // Measure force with interaction_coef = 0.5
    model.geom_fluid[geom_id][0] = 0.5;
    let mut data2 = model.make_data();
    data2.qvel[dof_adr + 2] = -5.0;
    data2.forward(&model).expect("forward");
    let f2 = data2.qfrc_fluid[dof_adr + 2];

    let ratio = if f1.abs() > 1e-15 { f2 / f1 } else { 0.0 };
    let err_pct = ((ratio - 0.5) / 0.5 * 100.0).abs();

    let p = check(
        "Interaction coefficient scaling",
        err_pct < 1.0,
        &format!("force@1.0={f1:.6}, force@0.5={f2:.6}, ratio={ratio:.4}, err={err_pct:.2}%"),
    );
    (u32::from(p), 1)
}

// ── Check 8: Wind on stationary body ──────────────────────────────────────

fn check_08_wind_stationary() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_E).expect("parse");
    let mut data = model.make_data();

    // Body at rest, wind = [10, 0, 0]. Forward computes fluid forces.
    data.forward(&model).expect("forward");

    let dof_adr = model.body_dof_adr[model.body_id("sphere").expect("sphere")];
    let fx = data.qfrc_fluid[dof_adr]; // x-component

    let p = check(
        "Wind on stationary body",
        fx.abs() > 1e-6,
        &format!("qfrc_fluid_x={fx:.6} (wind=[10,0,0], body at rest)"),
    );
    (u32::from(p), 1)
}

// ── Check 9: Wind cancellation ────────────────────────────────────────────

fn check_09_wind_cancellation() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_E).expect("parse");
    let mut data = model.make_data();

    // Wind = [10, 0, 0]. Set body velocity to match wind.
    // Free joint DOF order: [vx, vy, vz, wx, wy, wz]
    let dof_adr = model.body_dof_adr[model.body_id("sphere").expect("sphere")];
    data.qvel[dof_adr] = 10.0; // vx = wind_x

    data.forward(&model).expect("forward");

    // Effective velocity = body_vel - wind = [0, 0, 0] → zero drag
    // Note: translational x-component should be ~0. Other components may have
    // small residuals from wind in other axes or numerical noise.
    let fx = data.qfrc_fluid[dof_adr];

    let p = check(
        "Wind cancellation",
        fx.abs() < 1e-6,
        &format!("qfrc_fluid_x={fx:.2e} (body vx=10, wind=[10,0,0], effective=0)"),
    );
    (u32::from(p), 1)
}

// ── Check 10: Wind direction ──────────────────────────────────────────────

fn check_10_wind_direction() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_E2).expect("parse");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");

    let dof_adr = model.body_dof_adr[model.body_id("sphere").expect("sphere")];
    let fx = data.qfrc_fluid[dof_adr]; // x-component
    let fy = data.qfrc_fluid[dof_adr + 1]; // y-component

    // Wind is [0, 10, 0] → force should be primarily in Y, not X
    let p = check(
        "Wind direction — Y wind → Y force",
        fy.abs() > 1e-6 && fy.abs() > fx.abs() * 100.0,
        &format!("fx={fx:.2e}, fy={fy:.6} (wind=[0,10,0])"),
    );
    (u32::from(p), 1)
}

// ── Check 11: Spring restoring force ──────────────────────────────────────

fn check_11_spring_force() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_F).expect("parse");
    let mut data = model.make_data();

    // Displace slide joint by 0.1 from springref=0
    let jid = model.joint_id("slide_j").expect("slide_j");
    let qadr = model.jnt_qpos_adr[jid];
    let dof_adr = model.jnt_dof_adr[jid];
    data.qpos[qadr] = 0.1;
    data.qvel[dof_adr] = 0.0; // zero velocity → no damper force

    data.forward(&model).expect("forward");

    // F_spring = -k * (q - springref) = -100 * (0.1 - 0) = -10.0
    let f_spring = data.qfrc_spring[dof_adr];
    let expected = -10.0;
    let err = (f_spring - expected).abs();

    let p = check(
        "Spring restoring force",
        err < 1e-10,
        &format!("qfrc_spring={f_spring:.10}, expected={expected:.1}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 12: Springref != qpos0 ─────────────────────────────────────────

fn check_12_springref_offset() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_F2).expect("parse");
    let mut data = model.make_data();

    // qpos=0 (initial), springref=0.5 → spring pulls toward +0.5
    // F_spring = -100 * (0 - 0.5) = +50.0 (positive = toward springref)
    let jid = model.joint_id("slide_j").expect("slide_j");
    let dof_adr = model.jnt_dof_adr[jid];

    data.forward(&model).expect("forward");

    let f_spring = data.qfrc_spring[dof_adr];

    let p = check(
        "Springref != qpos0 — pulls toward springref",
        f_spring > 0.0 && (f_spring - 50.0).abs() < 1e-10,
        &format!("qfrc_spring={f_spring:.6} (expected +50.0: -100*(0-0.5))"),
    );
    (u32::from(p), 1)
}

// ── Check 13: Spring + gravity equilibrium ────────────────────────────────

fn check_13_spring_gravity_equilibrium() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_G).expect("parse");
    let mut data = model.make_data();

    let jid = model.joint_id("slide_j").expect("slide_j");
    let qadr = model.jnt_qpos_adr[jid];
    let mass = 1.0;
    let g = 9.81;
    let k = 200.0;

    // Equilibrium: spring force balances gravity.
    // -k * (q_eq - springref) = -m*g  (slide axis is +Z, gravity is -Z)
    // q_eq = springref - mg/k = 0 - 1.0*9.81/200 = -0.04905
    let q_eq_expected = -mass * g / k;

    // Simulate long enough to settle (damping = 5.0 helps)
    for _ in 0..20_000 {
        data.step(&model).expect("step");
    }

    let q_eq = data.qpos[qadr];
    let err_pct = ((q_eq - q_eq_expected) / q_eq_expected * 100.0).abs();

    let p = check(
        "Spring + gravity equilibrium",
        err_pct < 1.0,
        &format!("q_eq={q_eq:.6}, expected={q_eq_expected:.6}, err={err_pct:.3}%"),
    );
    (u32::from(p), 1)
}

// ── Check 14: Damper dissipation ──────────────────────────────────────────

fn check_14_damper_dissipation() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_H).expect("parse");
    let mut data = model.make_data();

    // Displace pendulum so it has energy
    let jid = model.joint_id("hinge_j").expect("hinge_j");
    let qadr = model.jnt_qpos_adr[jid];
    data.qpos[qadr] = 0.8; // ~45 degrees

    // Track total energy every 100 steps. Should be monotonically decreasing.
    let mut prev_energy = f64::MAX;
    let mut monotonic = true;
    let mut samples = 0;

    for i in 0..5000 {
        data.step(&model).expect("step");
        if i % 100 == 99 {
            let total_e = data.energy_kinetic + data.energy_potential;
            if total_e > prev_energy + 1e-10 {
                monotonic = false;
            }
            prev_energy = total_e;
            samples += 1;
        }
    }

    let p = check(
        "Damper dissipation — energy monotonically decreases",
        monotonic && samples > 10,
        &format!("monotonic={monotonic}, samples={samples}, final_E={prev_energy:.6}"),
    );
    (u32::from(p), 1)
}

// ── Check 15: Damper force proportional to velocity ───────────────────────

fn check_15_damper_proportional() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_I).expect("parse");
    let mut data = model.make_data();

    let jid = model.joint_id("hinge_j").expect("hinge_j");
    let dof_adr = model.jnt_dof_adr[jid];
    let damping = 5.0;

    // Set known velocity
    let qvel = 2.5;
    data.qvel[dof_adr] = qvel;

    data.forward(&model).expect("forward");

    // qfrc_damper = -damping * qvel = -5.0 * 2.5 = -12.5
    let f_damper = data.qfrc_damper[dof_adr];
    let expected = -damping * qvel;
    let err = (f_damper - expected).abs();

    let p = check(
        "Damper force proportional to velocity",
        err < 1e-10,
        &format!("qfrc_damper={f_damper:.10}, expected={expected:.1}, err={err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 16: ImplicitSpringDamper suppression ────────────────────────────

fn check_16_implicit_suppression() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_J).expect("parse");
    let mut data = model.make_data();

    // Give the slider some velocity so damper would normally produce force
    let jid = model.joint_id("slide_j").expect("slide_j");
    let qadr = model.jnt_qpos_adr[jid];
    let dof_adr = model.jnt_dof_adr[jid];
    data.qpos[qadr] = 0.5; // displaced from springref=0
    data.qvel[dof_adr] = 1.0; // nonzero velocity

    data.forward(&model).expect("forward");

    let f_spring = data.qfrc_spring[dof_adr];
    let f_damper = data.qfrc_damper[dof_adr];
    let f_fluid = data.qfrc_fluid[dof_adr];
    let f_passive = data.qfrc_passive[dof_adr];

    // In ImplicitSpringDamper mode: spring and damper are handled implicitly,
    // so qfrc_spring and qfrc_damper remain zero. But fluid still computed.
    let spring_zero = f_spring.abs() < 1e-15;
    let damper_zero = f_damper.abs() < 1e-15;
    let fluid_nonzero = f_fluid.abs() > 1e-10;

    let p = check(
        "ImplicitSpringDamper suppression",
        spring_zero && damper_zero && fluid_nonzero,
        &format!(
            "spring={f_spring:.2e}, damper={f_damper:.2e}, fluid={f_fluid:.6}, passive={f_passive:.6}"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 17: DISABLE_SPRING ──────────────────────────────────────────────

fn check_17_disable_spring() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_K).expect("parse");
    let mut data = model.make_data();

    let jid = model.joint_id("slide_j").expect("slide_j");
    let qadr = model.jnt_qpos_adr[jid];
    let dof_adr = model.jnt_dof_adr[jid];

    // Displace from springref and give velocity
    data.qpos[qadr] = 0.3;
    data.qvel[dof_adr] = 1.0;

    data.forward(&model).expect("forward");

    let f_spring = data.qfrc_spring[dof_adr];
    let f_damper = data.qfrc_damper[dof_adr];

    let p = check(
        "DISABLE_SPRING — spring=0, damper computed",
        f_spring.abs() < 1e-15 && f_damper.abs() > 0.1,
        &format!("spring={f_spring:.2e}, damper={f_damper:.6}"),
    );
    (u32::from(p), 1)
}

// ── Check 18: DISABLE_DAMPER ──────────────────────────────────────────────

fn check_18_disable_damper() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_L).expect("parse");
    let mut data = model.make_data();

    let jid = model.joint_id("slide_j").expect("slide_j");
    let qadr = model.jnt_qpos_adr[jid];
    let dof_adr = model.jnt_dof_adr[jid];

    // Displace from springref and give velocity
    data.qpos[qadr] = 0.3;
    data.qvel[dof_adr] = 1.0;

    data.forward(&model).expect("forward");

    let f_spring = data.qfrc_spring[dof_adr];
    let f_damper = data.qfrc_damper[dof_adr];

    let p = check(
        "DISABLE_DAMPER — damper=0, spring computed",
        f_damper.abs() < 1e-15 && f_spring.abs() > 0.1,
        &format!("spring={f_spring:.6}, damper={f_damper:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Main ──────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Passive Forces — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("Terminal velocity magnitude", check_01_terminal_velocity),
        ("Drag proportional to v²", check_02_drag_v_squared),
        ("Zero medium = zero drag", check_03_zero_medium),
        (
            "Viscous drag linear in velocity",
            check_04_viscous_linearity,
        ),
        (
            "Ellipsoid activates on fluidshape",
            check_05_ellipsoid_activates,
        ),
        ("Shape dependence", check_06_shape_dependence),
        (
            "Interaction coefficient scaling",
            check_07_interaction_scaling,
        ),
        ("Wind on stationary body", check_08_wind_stationary),
        ("Wind cancellation", check_09_wind_cancellation),
        ("Wind direction", check_10_wind_direction),
        ("Spring restoring force", check_11_spring_force),
        ("Springref != qpos0", check_12_springref_offset),
        (
            "Spring + gravity equilibrium",
            check_13_spring_gravity_equilibrium,
        ),
        ("Damper dissipation", check_14_damper_dissipation),
        (
            "Damper force proportional to velocity",
            check_15_damper_proportional,
        ),
        (
            "ImplicitSpringDamper suppression",
            check_16_implicit_suppression,
        ),
        ("DISABLE_SPRING", check_17_disable_spring),
        ("DISABLE_DAMPER", check_18_disable_damper),
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
