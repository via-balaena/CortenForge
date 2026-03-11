//! Phase 13 Spec C — Composite body generation tests.
//!
//! Tests T1–T12 from SPEC_C.md: cable body chain generation,
//! naming conventions, contact excludes, sites, deprecated type errors,
//! uservert, validation, and pipeline integration.

use sim_core::{ElementType, GeomType, MjJointType};

/// Helper: load a model from an MJCF string using the full pipeline.
fn load_model(xml: &str) -> sim_core::Model {
    sim_mjcf::load_model(xml).unwrap_or_else(|e| panic!("Failed to load model: {e}"))
}

/// Helper: attempt to load a model, returning the error string if it fails.
fn try_load_model(xml: &str) -> Result<sim_core::Model, String> {
    sim_mjcf::load_model(xml).map_err(|e| e.to_string())
}

// ============================================================================
// T1: Cable basic generation → AC1, AC2, AC3, AC8, AC9, AC10
// ============================================================================

#[test]
fn t1_cable_basic_generation() {
    let xml = r#"
        <mujoco model="cable_test">
            <worldbody>
                <composite type="cable" count="5 1 1" curve="s 0 0" size="1" offset="0 0 1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC1: body count — world + 4 cable bodies = 5
    assert_eq!(model.nbody, 5, "Expected 5 bodies (world + 4 cable)");

    // AC1: joint count — 4 ball joints (one per cable body, initial="ball" default)
    assert_eq!(model.njnt, 4, "Expected 4 joints");

    // AC1: geom count — 4 geoms (one per cable body)
    assert_eq!(model.ngeom, 4, "Expected 4 geoms");

    // AC2: body naming
    assert_eq!(model.id2name(ElementType::Body, 1).unwrap(), "B_first");
    assert_eq!(model.id2name(ElementType::Body, 2).unwrap(), "B_1");
    assert_eq!(model.id2name(ElementType::Body, 3).unwrap(), "B_2");
    assert_eq!(model.id2name(ElementType::Body, 4).unwrap(), "B_last");

    // AC3: parent chain
    assert_eq!(model.body_parent[1], 0, "B_first parent = world");
    assert_eq!(model.body_parent[2], 1, "B_1 parent = B_first");
    assert_eq!(model.body_parent[3], 2, "B_2 parent = B_1");
    assert_eq!(model.body_parent[4], 3, "B_last parent = B_2");

    // AC8: contact excludes — 3 pairs for 4 bodies
    assert_eq!(
        model.contact_excludes.len(),
        3,
        "Expected 3 contact exclude pairs"
    );

    // AC9: boundary sites — 2 sites total
    assert_eq!(model.nsite, 2, "Expected 2 boundary sites");

    // AC10: geom type is capsule
    for i in 0..4 {
        assert_eq!(
            model.geom_type[i],
            GeomType::Capsule,
            "Geom {i} should be capsule"
        );
    }
}

// ============================================================================
// T2: Cable initial="none" → AC4
// ============================================================================

#[test]
fn t2_cable_initial_none() {
    let xml = r#"
        <mujoco model="cable_none">
            <worldbody>
                <composite type="cable" count="4 1 1" initial="none" curve="l 0 0" size="1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC4: no joint on first body, ball joints on others → njnt = 2
    assert_eq!(model.njnt, 2, "Expected 2 joints (no joint on B_first)");

    // Verify joint types are all ball
    for i in 0..model.njnt {
        assert_eq!(
            model.jnt_type[i],
            MjJointType::Ball,
            "Joint {i} should be ball"
        );
    }
}

// ============================================================================
// T3: Cable initial="free" → AC5
// ============================================================================

#[test]
fn t3_cable_initial_free() {
    let xml = r#"
        <mujoco model="cable_free">
            <worldbody>
                <composite type="cable" count="4 1 1" initial="free" curve="l 0 0" size="1">
                    <joint kind="main" damping="0.5"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC5: njnt = 3 — free joint on first, ball joints on others
    assert_eq!(model.njnt, 3, "Expected 3 joints");

    // First joint is free
    assert_eq!(
        model.jnt_type[0],
        MjJointType::Free,
        "First joint should be free"
    );

    // Free joint has damping = 0
    assert_eq!(model.dof_damping[0], 0.0, "Free joint damping should be 0");

    // Other joints are ball
    for i in 1..model.njnt {
        assert_eq!(
            model.jnt_type[i],
            MjJointType::Ball,
            "Joint {i} should be ball"
        );
    }
}

// ============================================================================
// T4: Deprecated type errors → AC6
// ============================================================================

#[test]
fn t4_deprecated_type_errors() {
    let deprecated = [
        ("particle", "replicate"),
        ("grid", "flex"),
        ("rope", "cable"),
        ("loop", "flexcomp"),
        ("cloth", "shell"),
    ];

    for (comp_type, replacement) in &deprecated {
        let xml = format!(
            r#"
            <mujoco model="deprecated_{comp_type}">
                <worldbody>
                    <composite type="{comp_type}" count="5 1 1" size="1">
                        <geom type="capsule" size=".005"/>
                    </composite>
                </worldbody>
            </mujoco>
            "#
        );
        let result = try_load_model(&xml);
        assert!(
            result.is_err(),
            "Deprecated type '{comp_type}' should error"
        );
        let err = result.unwrap_err();
        assert!(
            err.contains("deprecated"),
            "Error for '{comp_type}' should mention 'deprecated': {err}"
        );
        assert!(
            err.contains(replacement),
            "Error for '{comp_type}' should mention replacement '{replacement}': {err}"
        );
    }
}

// ============================================================================
// T5: Cable with prefix → AC7
// ============================================================================

#[test]
fn t5_cable_with_prefix() {
    let xml = r#"
        <mujoco model="cable_prefix">
            <worldbody>
                <composite type="cable" prefix="R" count="3 1 1" curve="l 0 0" size="1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC7: all element names start with prefix "R"
    assert_eq!(model.id2name(ElementType::Body, 1).unwrap(), "RB_first");
    assert_eq!(model.id2name(ElementType::Body, 2).unwrap(), "RB_last");
    assert_eq!(model.id2name(ElementType::Joint, 0).unwrap(), "RJ_first");
    assert_eq!(model.id2name(ElementType::Joint, 1).unwrap(), "RJ_last");
    assert_eq!(model.id2name(ElementType::Geom, 0).unwrap(), "RG0");
    assert_eq!(model.id2name(ElementType::Geom, 1).unwrap(), "RG1");
}

// ============================================================================
// T6: Cable in nested body → AC12
// ============================================================================

#[test]
fn t6_cable_in_nested_body() {
    let xml = r#"
        <mujoco model="cable_nested">
            <worldbody>
                <body name="parent" pos="1 0 0">
                    <composite type="cable" count="3 1 1" curve="l 0 0" size="1">
                        <joint kind="main" damping="0.01"/>
                        <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                    </composite>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC12: B_first is a child of "parent" body, not world
    let b_first_id = model.name2id(ElementType::Body, "B_first").unwrap();
    let parent_id = model.body_parent[b_first_id];
    let parent_name = model.id2name(ElementType::Body, parent_id).unwrap();
    assert_eq!(parent_name, "parent", "B_first parent should be 'parent'");
}

// ============================================================================
// T7: Cable minimum count (+ dual sites) → AC13
// ============================================================================

#[test]
fn t7_cable_minimum_count() {
    let xml = r#"
        <mujoco model="cable_min">
            <worldbody>
                <composite type="cable" count="2 1 1" curve="l 0 0" size="1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC13: single body (world + 1 cable body)
    assert_eq!(model.nbody, 2, "Expected 2 bodies (world + 1 cable)");
    assert_eq!(model.njnt, 1, "Expected 1 joint");
    assert_eq!(model.ngeom, 1, "Expected 1 geom");
    assert_eq!(model.contact_excludes.len(), 0, "Expected 0 excludes");

    // Both S_first and S_last on the same body
    assert_eq!(model.nsite, 2, "Expected 2 sites on single body");
}

// ============================================================================
// T8: Invalid geom type → AC14
// ============================================================================

#[test]
fn t8_invalid_geom_type() {
    let xml = r#"
        <mujoco model="cable_bad_geom">
            <worldbody>
                <composite type="cable" count="3 1 1" curve="l 0 0" size="1">
                    <joint kind="main"/>
                    <geom type="sphere" size=".005"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let result = try_load_model(xml);
    assert!(result.is_err(), "Sphere geom type should be rejected");
}

// ============================================================================
// T9: Multiple composites (supplementary)
// ============================================================================

#[test]
fn t9_multiple_composites() {
    let xml = r#"
        <mujoco model="cable_multi">
            <worldbody>
                <composite type="cable" prefix="A" count="3 1 1" curve="l 0 0" size="1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
                <composite type="cable" prefix="B" count="4 1 1" curve="l 0 0" size="2" offset="0 1 0">
                    <joint kind="main" damping="0.02"/>
                    <geom type="capsule" size=".01" rgba=".2 .8 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // A: 2 cable bodies, B: 3 cable bodies = 5 + world = 6
    assert_eq!(model.nbody, 6, "Expected 6 bodies (world + 2 + 3)");

    // Verify both prefixes exist
    assert!(model.name2id(ElementType::Body, "AB_first").is_some());
    assert!(model.name2id(ElementType::Body, "AB_last").is_some());
    assert!(model.name2id(ElementType::Body, "BB_first").is_some());
    assert!(model.name2id(ElementType::Body, "BB_last").is_some());
}

// ============================================================================
// T10: Cable with cylinder geom → AC10
// ============================================================================

#[test]
fn t10_cable_cylinder_geom() {
    let xml = r#"
        <mujoco model="cable_cyl">
            <worldbody>
                <composite type="cable" count="3 1 1" curve="l 0 0" size="1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="cylinder" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC10: geoms are cylinder type
    for i in 0..model.ngeom {
        assert_eq!(
            model.geom_type[i],
            GeomType::Cylinder,
            "Geom {i} should be cylinder"
        );
    }
}

// ============================================================================
// T11: Cable with user-specified vertices → AC15
// ============================================================================

#[test]
fn t11_cable_uservert() {
    let xml = r#"
        <mujoco model="cable_uservert">
            <worldbody>
                <composite type="cable" vertex="0 0 0 1 0 0 2 0 1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(xml);

    // AC15: 3 vertices → 2 cable bodies
    assert_eq!(model.nbody, 3, "Expected 3 bodies (world + 2 cable)");
    assert_eq!(model.njnt, 2, "Expected 2 joints");
}

// ============================================================================
// T12: Validation error cases → AC16, AC17, AC18
// ============================================================================

#[test]
fn t12_validation_errors() {
    // (a) count="1 1 1" → too few
    let xml_a = r#"
        <mujoco><worldbody>
            <composite type="cable" count="1 1 1" curve="l 0 0" size="1">
                <joint kind="main"/><geom type="capsule" size=".005"/>
            </composite>
        </worldbody></mujoco>
    "#;
    assert!(try_load_model(xml_a).is_err(), "count=1 should error");

    // (b) count="5 5 1" → multi-dimensional
    let xml_b = r#"
        <mujoco><worldbody>
            <composite type="cable" count="5 5 1" curve="l 0 0" size="1">
                <joint kind="main"/><geom type="capsule" size=".005"/>
            </composite>
        </worldbody></mujoco>
    "#;
    let err_b = try_load_model(xml_b).unwrap_err();
    assert!(
        err_b.contains("one-dimensional"),
        "Multi-dim error: {err_b}"
    );

    // (c) both count and vertex → mutual exclusion
    let xml_c = r#"
        <mujoco><worldbody>
            <composite type="cable" count="5 1 1" vertex="0 0 0 1 0 0" curve="l 0 0" size="1">
                <joint kind="main"/><geom type="capsule" size=".005"/>
            </composite>
        </worldbody></mujoco>
    "#;
    let err_c = try_load_model(xml_c).unwrap_err();
    assert!(
        err_c.contains("vertex and count") || err_c.contains("Cannot specify both"),
        "Mutual exclusion error: {err_c}"
    );

    // (d) initial="xyz" → invalid initial
    let xml_d = r#"
        <mujoco><worldbody>
            <composite type="cable" count="3 1 1" initial="xyz" curve="l 0 0" size="1">
                <joint kind="main"/><geom type="capsule" size=".005"/>
            </composite>
        </worldbody></mujoco>
    "#;
    let err_d = try_load_model(xml_d).unwrap_err();
    assert!(
        err_d.contains("initial") || err_d.contains("xyz"),
        "Invalid initial error: {err_d}"
    );

    // (e) size="0 0 0" (no uservert) → too small
    let xml_e = r#"
        <mujoco><worldbody>
            <composite type="cable" count="3 1 1" size="0 0 0" curve="l 0 0">
                <joint kind="main"/><geom type="capsule" size=".005"/>
            </composite>
        </worldbody></mujoco>
    "#;
    let err_e = try_load_model(xml_e).unwrap_err();
    assert!(
        err_e.contains("size") || err_e.contains("small"),
        "Zero size error: {err_e}"
    );
}
