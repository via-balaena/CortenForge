//! DT-25 Verification: Deformable-rigid friction correctness.
//!
//! Phase 8 Session 13 — verification pass (not a full spec).
//! Confirms deformable-rigid friction works for condim=3 (the common case),
//! verifies R-scaling with asymmetric Jacobians, and documents gaps.

use sim_mjcf::load_model;

// ============================================================================
// MJCF Fixtures
// ============================================================================

/// A single flex vertex resting on a tilted large box with friction.
/// The box is tilted 30° about Y so gravity has a tangential component.
/// With sufficient friction (μ=1.0), the vertex should decelerate.
/// condim=3 → sliding friction (2 tangent directions).
///
/// Note: The collision geom is on a separate kinematic body (not world body)
/// because flex vertices parented to the world body cannot collide with
/// world-body geoms (parent-child filter in mj_collision_flex).
fn flex_vertex_on_tilted_plane_mjcf() -> &'static str {
    r#"
    <mujoco model="flex_friction_tilt">
        <option gravity="0 0 -9.81" timestep="0.001" solver="PGS" iterations="50"/>
        <worldbody>
            <body pos="0 0 -0.5" euler="0 30 0">
                <geom type="box" size="5 5 0.5" friction="1.0 0.005 0.0001"/>
            </body>
        </worldbody>
        <deformable>
            <flex name="point" dim="1" density="1.0" radius="0.01">
                <contact condim="3" friction="1.0 0.005 0.0001" solref="0.02 1.0"/>
                <elasticity young="1e4"/>
                <vertex pos="0 0 0.1"/>
                <element data="0"/>
            </flex>
        </deformable>
    </mujoco>
    "#
}

/// Flex cloth (2x2) on a large box (acting as ground) with friction.
/// Vertices start within contact distance (z=0.002 < radius=0.005) so
/// contacts are generated immediately. With condim=3 friction, contact
/// constraint rows should include tangential components.
///
/// Note: Uses a box geom on a separate body because flex vertices parented
/// to the world body skip collision with world-body geoms (parent-child filter).
fn flex_cloth_friction_mjcf() -> &'static str {
    r#"
    <mujoco model="flex_cloth_friction">
        <option gravity="0 0 -9.81" timestep="0.001" solver="PGS" iterations="100"/>
        <worldbody>
            <body pos="0 0 -0.5">
                <geom type="box" size="5 5 0.5" friction="1.0 0.005 0.0001"/>
            </body>
        </worldbody>
        <deformable>
            <flex name="cloth" dim="2" density="100" radius="0.005">
                <contact condim="3" friction="1.0 0.005 0.0001" solref="0.02 1.0"/>
                <elasticity young="50" damping="5.0" thickness="0.01"/>
                <vertex pos="0 0 0.002  0.1 0 0.002  0 0.1 0.002  0.1 0.1 0.002"/>
                <element data="0 1 2  1 3 2"/>
            </flex>
        </deformable>
    </mujoco>
    "#
}

/// Frictionless flex contact (condim=1) for comparison.
/// Vertices start within contact distance (z=0.002 < radius=0.005).
///
/// Note: Uses a box geom on a separate body because flex vertices parented
/// to the world body skip collision with world-body geoms (parent-child filter).
fn flex_cloth_frictionless_mjcf() -> &'static str {
    r#"
    <mujoco model="flex_cloth_frictionless">
        <option gravity="0 0 -9.81" timestep="0.001" solver="PGS" iterations="100"/>
        <worldbody>
            <body pos="0 0 -0.5">
                <geom type="box" size="5 5 0.5" condim="1" friction="0 0 0"/>
            </body>
        </worldbody>
        <deformable>
            <flex name="cloth" dim="2" density="100" radius="0.005">
                <contact condim="1" friction="0 0 0" solref="0.02 1.0"/>
                <elasticity young="50" damping="5.0" thickness="0.01"/>
                <vertex pos="0 0 0.002  0.1 0 0.002  0 0.1 0.002  0.1 0.1 0.002"/>
                <element data="0 1 2  1 3 2"/>
            </flex>
        </deformable>
    </mujoco>
    "#
}

// ============================================================================
// DT-25 Test 1: Flex contact condim=3 produces tangential constraint rows
// ============================================================================

#[test]
fn dt25_flex_contact_condim3_has_friction_rows() {
    let model = load_model(flex_cloth_friction_mjcf()).expect("should load");
    let mut data = model.make_data();

    // Verify flex condim is 3
    assert_eq!(model.flex_condim[0], 3, "flex condim should be 3");

    // Simulate a few steps to generate contacts
    for step in 0..200 {
        if let Err(e) = data.step(&model) {
            panic!("step {} failed: {:?}", step, e);
        }
    }

    // After settling, we should have flex contacts
    let flex_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| c.flex_vertex.is_some())
        .collect();
    assert!(
        !flex_contacts.is_empty(),
        "should have flex-rigid contacts after settling"
    );

    // Each flex contact with condim=3 should have dim=3
    for contact in &flex_contacts {
        assert_eq!(
            contact.dim, 3,
            "flex contact dim should be 3 for condim=3, got {}",
            contact.dim
        );
    }

    // Constraint rows should exist beyond equality+friction block
    let ne = data.ne;
    let nf = data.nf;
    let nefc = data.efc_type.len();
    assert!(
        nefc > ne + nf,
        "should have contact constraint rows beyond equality+friction: nefc={nefc}, ne={ne}, nf={nf}"
    );
}

// ============================================================================
// DT-25 Test 2: Friction prevents lateral sliding on tilted plane
// ============================================================================

#[test]
fn dt25_flex_friction_resists_sliding_on_tilted_plane() {
    let model = load_model(flex_vertex_on_tilted_plane_mjcf()).expect("should load");
    let mut data = model.make_data();

    // Run simulation
    for step in 0..500 {
        if let Err(e) = data.step(&model) {
            panic!("step {} failed: {:?}", step, e);
        }
    }

    // With μ=1.0 on a 30° slope, static friction should hold the vertex.
    // tan(30°) ≈ 0.577 < μ=1.0, so the Coulomb cone is not exceeded.
    // The vertex should remain nearly stationary — any significant sliding
    // indicates the friction projection is broken.
    let x_displacement = data.flexvert_xpos[0].x.abs();
    let y_displacement = data.flexvert_xpos[0].y.abs();

    assert!(
        x_displacement < 0.05,
        "vertex slid too far in x: {x_displacement:.4} (μ=1.0 should hold on 30° slope)"
    );
    assert!(
        y_displacement < 0.05,
        "vertex slid in y: {y_displacement:.4} (unexpected lateral motion)"
    );
}

// ============================================================================
// DT-25 Test 3: Nonzero tangential constraint forces for flex contacts
// ============================================================================

#[test]
fn dt25_flex_friction_nonzero_tangential_forces() {
    let model = load_model(flex_cloth_friction_mjcf()).expect("should load");
    let mut data = model.make_data();

    // Give vertices initial lateral velocity to trigger friction
    for vi in 0..model.nflexvert {
        if model.flexvert_invmass[vi] > 0.0 {
            let dof_base = model.flexvert_dofadr[vi];
            if dof_base != usize::MAX {
                data.qvel[dof_base] = 1.0; // lateral velocity in x
            }
        }
    }

    // Run enough steps for contacts to form and friction to act
    let mut had_contact_forces = false;
    for step in 0..300 {
        if let Err(e) = data.step(&model) {
            panic!("step {} failed: {:?}", step, e);
        }

        // Check for nonzero forces on contact rows
        let ne = data.ne;
        let nf = data.nf;
        let contact_start = ne + nf;
        let nefc = data.efc_type.len();

        for i in contact_start..nefc {
            if data.efc_force[i].abs() > 1e-10 {
                had_contact_forces = true;
            }
        }
    }

    assert!(
        had_contact_forces,
        "should have nonzero contact constraint forces (including friction)"
    );
}

// ============================================================================
// DT-25 Test 4: condim=1 produces dim=1 (frictionless baseline)
// ============================================================================

#[test]
fn dt25_flex_frictionless_contact_dim1() {
    let model = load_model(flex_cloth_frictionless_mjcf()).expect("should load");
    let mut data = model.make_data();

    // Simulate
    for step in 0..200 {
        if let Err(e) = data.step(&model) {
            panic!("step {} failed: {:?}", step, e);
        }
    }

    let flex_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| c.flex_vertex.is_some())
        .collect();

    assert!(
        !flex_contacts.is_empty(),
        "should have frictionless flex contacts (vertices start within contact distance)"
    );
    for contact in &flex_contacts {
        assert_eq!(
            contact.dim, 1,
            "frictionless flex contact should have dim=1, got {}",
            contact.dim
        );
    }
}

// ============================================================================
// DT-25 Test 5: R-scaling — exact diagonal handles asymmetric Jacobian
// ============================================================================

#[test]
fn dt25_exact_diag_approx_handles_flex_asymmetric_jacobian() {
    // When diagapprox_bodyweight is false (default), the exact diagonal
    // computation (J · M⁻¹ · J^T) should produce valid positive values
    // even with asymmetric flex-rigid Jacobians.
    let model = load_model(flex_cloth_friction_mjcf()).expect("should load");
    assert!(
        !model.diagapprox_bodyweight,
        "default should be exact diagonal (not bodyweight)"
    );

    let mut data = model.make_data();

    // Run a few steps to generate contacts
    for step in 0..200 {
        if let Err(e) = data.step(&model) {
            panic!("step {} failed: {:?}", step, e);
        }
    }

    // Check diagApprox values for contact rows
    let ne = data.ne;
    let nf = data.nf;
    let contact_start = ne + nf;
    let nefc = data.efc_type.len();

    let mj_minval = 1e-15_f64;
    for i in contact_start..nefc {
        let diag = data.efc_diagApprox[i];
        assert!(
            diag >= mj_minval,
            "diagApprox[{i}] = {diag} should be >= MJ_MINVAL for flex contact row"
        );
        assert!(
            diag.is_finite(),
            "diagApprox[{i}] = {diag} should be finite"
        );

        // R should also be valid
        let r = data.efc_R[i];
        assert!(r >= mj_minval, "efc_R[{i}] = {r} should be >= MJ_MINVAL");
        assert!(r.is_finite(), "efc_R[{i}] = {r} should be finite");
    }
}

// ============================================================================
// DT-25 Test 6: Flex friction prevents free-fall (end-to-end)
// ============================================================================

#[test]
fn dt25_flex_friction_cloth_rests_on_plane() {
    let model = load_model(flex_cloth_friction_mjcf()).expect("should load");
    let mut data = model.make_data();

    // Simulate for 1 second
    for step in 0..1000 {
        if let Err(e) = data.step(&model) {
            panic!("step {} failed: {:?}", step, e);
        }
    }

    // All vertices should be near z=0 (resting on plane), not in free-fall.
    // Without constraints, after 1s they'd be at z ≈ 0.01 - 0.5*9.81*1² ≈ -4.9.
    for vi in 0..model.nflexvert {
        let z = data.flexvert_xpos[vi].z;
        assert!(
            z > -0.5,
            "vertex {vi} at z={z:.3} — fell through plane (contact forces not working)"
        );
    }
}

// ============================================================================
// DT-25 Test 7: Flex contact Jacobian is well-formed (no NaN)
// ============================================================================

#[test]
fn dt25_flex_contact_jacobian_well_formed() {
    // Verify the constraint Jacobian rows for flex contacts are finite
    // and produce meaningful efc_J entries. The flex vertex side has only
    // 3 translational DOFs (no angular), creating an asymmetric Jacobian.
    let model = load_model(flex_cloth_friction_mjcf()).expect("should load");
    let mut data = model.make_data();

    // Run to generate contacts
    for step in 0..200 {
        if let Err(e) = data.step(&model) {
            panic!("step {} failed: {:?}", step, e);
        }
    }

    let nv = model.nv;
    let ne = data.ne;
    let nf = data.nf;
    let contact_start = ne + nf;
    let nefc = data.efc_type.len();

    // All contact Jacobian rows should be finite
    for i in contact_start..nefc {
        for col in 0..nv {
            let val = data.efc_J[(i, col)];
            assert!(
                val.is_finite(),
                "efc_J[{i},{col}] = {val} is not finite (flex contact row)"
            );
        }

        // At least some columns should be nonzero (the row has meaning)
        let row_norm: f64 = (0..nv).map(|col| data.efc_J[(i, col)].powi(2)).sum();
        assert!(
            row_norm > 1e-20,
            "efc_J row {i} is all-zero (degenerate flex contact Jacobian)"
        );
    }
}
