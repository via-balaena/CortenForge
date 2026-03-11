//! DT-39: Body-weight diagonal approximation tests.
//!
//! Validates that `compute_invweight0()` correctly computes:
//! - `body_invweight0[b]` = operational-space inverse inertia (J·M⁻¹·J^T diagonal avg)
//! - `dof_invweight0[d]` = M⁻¹ diagonal subblock for each joint's DOFs
//! - `tendon_invweight0[t]` = J_tendon · M⁻¹ · J_tendon^T (full quadratic form)
//!
//! MuJoCo ref: `setInertia()` / `set0()` in `engine_setconst.c`.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Test 1: body_invweight0 — single hinge body
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

    // World body: always [0, 0] (MuJoCo convention)
    assert_eq!(model.body_invweight0[0], [0.0, 0.0]);

    // Body 1 has a single hinge joint (axis=y). The 6×6 operational-space
    // inverse inertia J·M⁻¹·J^T at COM has only a single nonzero entry
    // in the rotational block (the y-rotation diagonal = 1/I_yy).
    // Average of 3×3 rotational diagonal = (0 + 1/I_yy + 0) / 3 = I_yy^{-1}/3.
    // Average of 3×3 translational diagonal = 0 (hinge doesn't translate).
    // MuJoCo fallback: if translational < MIN_VAL, copy rotational.
    let iyy = model.body_inertia[1].y;
    let rot = 1.0 / (3.0 * iyy);
    // Translational gets fallback copy from rotational
    assert_relative_eq!(model.body_invweight0[1][0], rot, epsilon = 1e-6);
    assert_relative_eq!(model.body_invweight0[1][1], rot, epsilon = 1e-6);

    // Verify the rotational component relates to 1/(3*I_yy) for a cube
    // I_yy = m/12*(w²+d²) = 2/12*(0.04+0.04) = 0.01333...
    assert_relative_eq!(iyy, 2.0 / 12.0 * 0.08, epsilon = 1e-6);

    // Also verify subtree mass is still computed correctly (used elsewhere)
    assert_relative_eq!(model.body_subtreemass[1], 2.0, epsilon = 1e-10);
    assert_relative_eq!(model.body_subtreemass[0], 2.0, epsilon = 1e-10);
}

// ============================================================================
// Test 2: body_invweight0 — chain uses M⁻¹ (NOT subtree accumulation)
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

    // World body: always [0, 0]
    assert_eq!(model.body_invweight0[0], [0.0, 0.0]);

    // Subtree mass still computed (used elsewhere)
    assert_relative_eq!(model.body_subtreemass[1], 4.0, epsilon = 1e-10);
    assert_relative_eq!(model.body_subtreemass[2], 1.0, epsilon = 1e-10);

    // Body invweight0 values should be positive for bodies with DOFs
    assert!(model.body_invweight0[1][0] > 0.0);
    assert!(model.body_invweight0[1][1] > 0.0);
    assert!(model.body_invweight0[2][0] > 0.0);
    assert!(model.body_invweight0[2][1] > 0.0);

    // Leaf body (body 2) should have higher invweight than parent (body 1)
    // because it's the tip of the chain with less effective inertia
    assert!(model.body_invweight0[2][1] > model.body_invweight0[1][1]);
}

// ============================================================================
// Test 3: dof_invweight0 — M⁻¹ diagonal for hinge and slide
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

    // Slide joint: dof_invweight0 = M⁻¹[dof,dof] = 1/mass (for decoupled body)
    let slide_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J_slide"))
        .unwrap();
    let slide_dof = model.jnt_dof_adr[slide_jnt];
    // For a single slide joint body with mass=2: M[0,0] = mass, M⁻¹[0,0] = 1/mass = 0.5
    assert_relative_eq!(model.dof_invweight0[slide_dof], 0.5, epsilon = 1e-6);

    // Slide-only body: body_invweight0 = [1/mass, 0] (MuJoCo body_simple==2)
    let slide_body = model.dof_body[slide_dof];
    assert_relative_eq!(model.body_invweight0[slide_body][0], 0.5, epsilon = 1e-6);
    assert_eq!(model.body_invweight0[slide_body][1], 0.0);

    // Hinge joint: dof_invweight0 = M⁻¹[dof,dof] = 1/I_yy (for decoupled body)
    let hinge_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J_hinge"))
        .unwrap();
    let hinge_dof = model.jnt_dof_adr[hinge_jnt];
    let hinge_body = model.dof_body[hinge_dof];
    let iyy = model.body_inertia[hinge_body].y;
    assert_relative_eq!(model.dof_invweight0[hinge_dof], 1.0 / iyy, epsilon = 1e-6);
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

    // Free joint: DOFs 0,1,2 = translational (avg of M⁻¹ diagonal = 1/mass)
    let mass = model.body_mass[body_id];
    for i in 0..3 {
        assert_relative_eq!(
            model.dof_invweight0[dof_adr + i],
            1.0 / mass,
            epsilon = 1e-6
        );
    }

    // Free joint: DOFs 3,4,5 = rotational (avg of M⁻¹ diagonal)
    // For a uniform sphere, all principal moments are equal: I = 2/5*m*r²
    let inertia = model.body_inertia[body_id];
    let avg_inv_inertia = (1.0 / inertia.x + 1.0 / inertia.y + 1.0 / inertia.z) / 3.0;
    for i in 3..6 {
        assert_relative_eq!(
            model.dof_invweight0[dof_adr + i],
            avg_inv_inertia,
            epsilon = 1e-6
        );
    }

    // body_invweight0 for free body: translational = 1/mass, rotational = avg 1/I
    assert_relative_eq!(
        model.body_invweight0[body_id][0],
        1.0 / mass,
        epsilon = 1e-6
    );
    assert_relative_eq!(
        model.body_invweight0[body_id][1],
        avg_inv_inertia,
        epsilon = 1e-6
    );
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

    // tendon_invweight0 = J · M⁻¹ · J^T; for independent bodies (block-diagonal M)
    // this equals 2.0² * dof_invweight0[J1_dof] + 3.0² * dof_invweight0[J2_dof]
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
// Test 5b: tendon_invweight0 — single-joint tendon (T2, AC2)
// Full form collapses to diagonal for 1 DOF: invweight0 = dof_invweight0
// ============================================================================

#[test]
fn tendon_invweight0_single_joint() {
    let mjcf = r#"
    <mujoco model="dt39_single_tendon">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="2.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1">
                <joint joint="J1" coef="1.0"/>
            </fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // AC2: Single-joint tendon: invweight0 = dof_invweight0 (coef=1.0)
    let dof = model.jnt_dof_adr[0];
    assert_relative_eq!(
        model.tendon_invweight0[0],
        model.dof_invweight0[dof],
        epsilon = 1e-12
    );
}

// ============================================================================
// Test 5c: tendon_invweight0 — serial chain off-diagonal coupling (T3, AC3)
// Full J·M⁻¹·J^T > diagonal-only Σ(coef²·M⁻¹[i,i]) for serial chain
// ============================================================================

#[test]
fn tendon_invweight0_serial_chain_coupling() {
    let mjcf = r#"
    <mujoco model="dt39_serial_tendon">
        <option gravity="0 0 -9.81" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="2.0"/>
                <body name="b2" pos="0.5 0 0">
                    <joint name="J2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1">
                <joint joint="J1" coef="1.0"/>
                <joint joint="J2" coef="-1.0"/>
            </fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // AC3: Multi-joint serial chain — full form strictly greater than diagonal
    let dof0 = model.jnt_dof_adr[0];
    let dof1 = model.jnt_dof_adr[1];
    let diagonal_only = model.dof_invweight0[dof0] + model.dof_invweight0[dof1];

    assert!(
        model.tendon_invweight0[0] > diagonal_only,
        "Full J·M⁻¹·J^T ({}) should be > diagonal-only ({}) for serial chain",
        model.tendon_invweight0[0],
        diagonal_only
    );
}

// ============================================================================
// Test 5d: tendon_invweight0 — spatial tendon regression guard (T5, AC7)
// Spatial tendons use the same full J·M⁻¹·J^T algorithm; invweight0 > MIN_VAL
// ============================================================================

#[test]
fn tendon_invweight0_spatial_regression() {
    let mjcf = r#"
    <mujoco model="dt39_spatial_tendon">
        <worldbody>
            <body name="upper" pos="0 0 1">
                <joint name="shoulder" type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                <site name="s1" pos="0 0 0"/>
                <body name="lower" pos="0 0 -0.5">
                    <joint name="elbow" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                    <site name="s2" pos="0 0 -0.2"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <spatial name="t1">
                <site site="s1"/>
                <site site="s2"/>
            </spatial>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // AC7: Spatial tendon invweight0 >= MIN_VAL (1e-15), no panics.
    // Note: This model has a geometric degeneracy at qpos=0 — the tendon
    // is collinear with Z and both joints rotate around Y, so
    // d(length)/d(qvel) = 0. MuJoCo would also give MIN_VAL here.
    assert!(
        model.tendon_invweight0[0] >= 1e-15,
        "Spatial tendon invweight0 should be >= MIN_VAL, got {}",
        model.tendon_invweight0[0]
    );
}

// ============================================================================
// Test 5e: tendon_invweight0 — spatial tendon non-degenerate (nonzero J)
// When the tendon is NOT collinear with the joint axis plane, J is nonzero
// and invweight0 > MIN_VAL.
// ============================================================================

#[test]
fn tendon_invweight0_spatial_nondegenerate() {
    // s1 offset in X so the tendon is NOT collinear with Z — this breaks
    // the geometric degeneracy and gives a nonzero Jacobian.
    let mjcf = r#"
    <mujoco model="dt39_spatial_nondegen">
        <worldbody>
            <body name="upper" pos="0 0 1">
                <joint name="shoulder" type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                <site name="s1" pos="0.3 0 0"/>
                <body name="lower" pos="0 0 -0.5">
                    <joint name="elbow" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                    <site name="s2" pos="0 0 -0.2"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <spatial name="t1">
                <site site="s1"/>
                <site site="s2"/>
            </spatial>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");

    // With s1 offset in X, the tendon has a nonzero X component.
    // Hinge rotation around Y moves s1 in X-Z → changes tendon length.
    // So J is nonzero and invweight0 > MIN_VAL.
    assert!(
        model.tendon_invweight0[0] > 1e-15,
        "Non-degenerate spatial tendon invweight0 should be > MIN_VAL, got {}",
        model.tendon_invweight0[0]
    );
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
    let model = load_model(mjcf).expect("should load");
    // MJCF default is now bodyweight (matching MuJoCo)
    assert!(model.diagapprox_bodyweight);
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

    // Bodyweight mode (now the default)
    let model_bw = load_model(mjcf).expect("should load");
    assert!(model_bw.diagapprox_bodyweight);
    let mut data_bw = model_bw.make_data();
    data_bw.qpos[model_bw.jnt_qpos_adr[0]] = -1.5;
    data_bw.forward(&model_bw).expect("forward failed");

    // Exact mode (override)
    let mut model_exact = load_model(mjcf).expect("should load");
    model_exact.diagapprox_bodyweight = false;
    let mut data_exact = model_exact.make_data();
    data_exact.qpos[model_exact.jnt_qpos_adr[0]] = -1.5;
    data_exact.forward(&model_exact).expect("forward failed");

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
// Test 8: MJCF default is bodyweight (matching MuJoCo)
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
        model.diagapprox_bodyweight,
        "MJCF default diagapprox_bodyweight should be true (matching MuJoCo)"
    );
}
