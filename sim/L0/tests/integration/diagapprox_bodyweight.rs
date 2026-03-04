//! DT-39: Body-weight diagonal approximation tests.
//!
//! Validates that `compute_invweight0()` correctly computes:
//! - `body_invweight0[b] = [1/subtree_mass, 1/subtree_inertia_trace]`
//! - `dof_invweight0[d]` selects translational or rotational based on joint type
//! - `tendon_invweight0[t] = Σ(coef² * dof_invweight0[dof])` for fixed tendons
//!
//! Also validates that `diagapprox_bodyweight` mode produces reasonable
//! approximations compared to the exact M⁻¹ solve.
//!
//! MuJoCo ref: `setInertia()` in `engine_setconst.c`, `mj_diagApprox()` in
//! `engine_core_constraint.c`.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Test 1: body_invweight0 — single body
// ============================================================================

#[test]
fn invweight0_single_body() {
    let mjcf = r#"
    <mujoco model="dt39_single">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0"/>
                <geom type="box" size="0.1 0.1 0.1" mass="2.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // World body (index 0): subtreemass includes all children
    assert!(
        model.body_invweight0[0][0] > 0.0,
        "world translational invweight should be > 0"
    );
    assert!(
        model.body_invweight0[0][1] > 0.0,
        "world rotational invweight should be > 0"
    );

    // Body 1: mass=2.0, subtreemass=2.0 (no children)
    // invweight0[1][0] = 1/2.0 = 0.5
    assert_relative_eq!(model.body_invweight0[1][0], 0.5, epsilon = 1e-6);

    // Body 1 inertia trace: for a uniform box 0.2×0.2×0.2, mass=2.0:
    // Ix = m/12 * (h² + d²) = 2/12 * (0.04 + 0.04) = 0.01333...
    // Iy = Iz = same for a cube → trace = 3 * 0.01333... = 0.04
    // invweight0[1][1] = 1/0.04 = 25.0
    let inertia_trace = model.body_inertia[1].x + model.body_inertia[1].y + model.body_inertia[1].z;
    assert_relative_eq!(
        model.body_invweight0[1][1],
        1.0 / inertia_trace,
        epsilon = 1e-6
    );
}

// ============================================================================
// Test 2: body_invweight0 — chain with subtree accumulation
// ============================================================================

#[test]
fn invweight0_chain_subtree_accumulation() {
    let mjcf = r#"
    <mujoco model="dt39_chain">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="3.0"/>
                <body name="b2" pos="0.5 0 0">
                    <joint name="J2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // Body 2 (leaf): subtreemass = 1.0
    assert_relative_eq!(model.body_invweight0[2][0], 1.0 / 1.0, epsilon = 1e-6);

    // Body 1: subtreemass = 3.0 + 1.0 = 4.0
    assert_relative_eq!(model.body_invweight0[1][0], 1.0 / 4.0, epsilon = 1e-6);

    // World body: subtreemass = total = 4.0
    assert_relative_eq!(model.body_invweight0[0][0], 1.0 / 4.0, epsilon = 1e-6);

    // Subtree inertia also accumulates
    let trace1 = model.body_inertia[1].x + model.body_inertia[1].y + model.body_inertia[1].z;
    let trace2 = model.body_inertia[2].x + model.body_inertia[2].y + model.body_inertia[2].z;
    // body 1 subtree inertia trace = trace1 + trace2
    assert_relative_eq!(
        model.body_invweight0[1][1],
        1.0 / (trace1 + trace2),
        epsilon = 1e-6
    );
}

// ============================================================================
// Test 3: dof_invweight0 — translational vs rotational selection
// ============================================================================

#[test]
fn dof_invweight0_joint_type_dispatch() {
    let mjcf = r#"
    <mujoco model="dt39_dof">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J_slide" type="slide" axis="1 0 0"/>
                <geom type="sphere" size="0.1" mass="2.0"/>
            </body>
            <body name="b2" pos="1 0 1">
                <joint name="J_hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="2.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // Slide joint (translational DOF): should use body_invweight0[body][0]
    let slide_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J_slide"))
        .unwrap();
    let slide_dof = model.jnt_dof_adr[slide_jnt];
    let slide_body = model.dof_body[slide_dof];
    assert_relative_eq!(
        model.dof_invweight0[slide_dof],
        model.body_invweight0[slide_body][0], // translational
        epsilon = 1e-12
    );

    // Hinge joint (rotational DOF): should use body_invweight0[body][1]
    let hinge_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J_hinge"))
        .unwrap();
    let hinge_dof = model.jnt_dof_adr[hinge_jnt];
    let hinge_body = model.dof_body[hinge_dof];
    assert_relative_eq!(
        model.dof_invweight0[hinge_dof],
        model.body_invweight0[hinge_body][1], // rotational
        epsilon = 1e-12
    );
}

// ============================================================================
// Test 4: dof_invweight0 — free joint mixed DOFs
// ============================================================================

#[test]
fn dof_invweight0_free_joint_mixed() {
    let mjcf = r#"
    <mujoco model="dt39_free">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <freejoint name="J_free"/>
                <geom type="sphere" size="0.1" mass="5.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    let body_id = 1usize;
    let dof_adr = model.jnt_dof_adr[0];

    // Free joint: DOFs 0,1,2 = translational, DOFs 3,4,5 = rotational
    for i in 0..3 {
        assert_relative_eq!(
            model.dof_invweight0[dof_adr + i],
            model.body_invweight0[body_id][0], // translational
            epsilon = 1e-12
        );
    }
    for i in 3..6 {
        assert_relative_eq!(
            model.dof_invweight0[dof_adr + i],
            model.body_invweight0[body_id][1], // rotational
            epsilon = 1e-12
        );
    }
}

// ============================================================================
// Test 5: tendon_invweight0 — fixed tendon
// ============================================================================

#[test]
fn tendon_invweight0_fixed_tendon() {
    let mjcf = r#"
    <mujoco model="dt39_tendon">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="2.0"/>
            </body>
            <body name="b2" pos="1 0 1">
                <joint name="J2" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="3.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1">
                <joint joint="J1" coef="2.0"/>
                <joint joint="J2" coef="3.0"/>
            </fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // tendon_invweight0[0] = 2.0² * dof_invweight0[J1_dof] + 3.0² * dof_invweight0[J2_dof]
    let j1_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J1"))
        .unwrap();
    let j2_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J2"))
        .unwrap();
    let dof1 = model.jnt_dof_adr[j1_jnt];
    let dof2 = model.jnt_dof_adr[j2_jnt];

    let expected = 4.0 * model.dof_invweight0[dof1] + 9.0 * model.dof_invweight0[dof2];
    assert_relative_eq!(model.tendon_invweight0[0], expected, epsilon = 1e-10);
}

// ============================================================================
// Test 6: diagapprox_bodyweight mode produces non-zero values
// ============================================================================

#[test]
fn diagapprox_bodyweight_mode_produces_values() {
    let mjcf = r#"
    <mujoco model="dt39_bw_mode">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0" limited="true" range="-1 1"/>
                <geom type="sphere" size="0.1" mass="2.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let mut model = load_model(mjcf).expect("should load");
    model.diagapprox_bodyweight = true; // Enable bodyweight mode
    let mut data = model.make_data();

    // Put joint at limit to trigger a constraint row
    data.qpos[model.jnt_qpos_adr[0]] = -1.5; // violates lower limit
    data.forward(&model).expect("forward failed");

    // Should have at least one constraint row
    assert!(
        !data.efc_diagApprox.is_empty(),
        "diagApprox should have entries for violated limit"
    );

    // The bodyweight diagApprox should be positive (= dof_invweight0[0])
    let diag = data.efc_diagApprox[0];
    assert!(
        diag > 0.0,
        "bodyweight diagApprox should be > 0, got {diag}"
    );

    // It should equal dof_invweight0[0] for a joint limit
    assert_relative_eq!(diag, model.dof_invweight0[0], epsilon = 1e-10);
}

// ============================================================================
// Test 7: bodyweight vs exact comparison — same order of magnitude
// ============================================================================

#[test]
fn diagapprox_bodyweight_vs_exact_order_of_magnitude() {
    let mjcf = r#"
    <mujoco model="dt39_compare">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0" limited="true" range="-1 1"/>
                <geom type="box" size="0.1 0.1 0.1" mass="2.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    // Exact mode
    let model_exact = load_model(mjcf).expect("should load");
    let mut data_exact = model_exact.make_data();
    data_exact.qpos[model_exact.jnt_qpos_adr[0]] = -1.5;
    data_exact.forward(&model_exact).expect("forward failed");

    // Bodyweight mode
    let mut model_bw = load_model(mjcf).expect("should load");
    model_bw.diagapprox_bodyweight = true;
    let mut data_bw = model_bw.make_data();
    data_bw.qpos[model_bw.jnt_qpos_adr[0]] = -1.5;
    data_bw.forward(&model_bw).expect("forward failed");

    // Both should have the same number of constraint rows
    assert_eq!(
        data_exact.efc_diagApprox.len(),
        data_bw.efc_diagApprox.len()
    );

    // For a simple 1-DOF hinge limit, exact and bodyweight should agree
    // within an order of magnitude (bodyweight is approximate).
    for i in 0..data_exact.efc_diagApprox.len() {
        let exact = data_exact.efc_diagApprox[i];
        let bw = data_bw.efc_diagApprox[i];
        assert!(
            exact > 0.0 && bw > 0.0,
            "Both diagApprox values should be positive: exact={exact}, bw={bw}"
        );
        let ratio = (exact / bw).max(bw / exact);
        assert!(
            ratio < 100.0,
            "diagApprox exact ({exact}) and bodyweight ({bw}) should be within 100x, ratio={ratio}"
        );
    }
}

// ============================================================================
// Test 8: default mode is exact (backward compatibility)
// ============================================================================

#[test]
fn diagapprox_default_is_exact() {
    let mjcf = r#"
    <mujoco model="dt39_default">
        <option gravity="0 0 0" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");
    assert!(
        !model.diagapprox_bodyweight,
        "default diagapprox_bodyweight should be false"
    );
}
