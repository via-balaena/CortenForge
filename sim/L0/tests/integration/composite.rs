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
                <composite type="cable" count="4 1 1" initial="none" curve="s 0 0" size="1">
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
                <composite type="cable" count="4 1 1" initial="free" curve="s 0 0" size="1">
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
                <composite type="cable" prefix="R" count="3 1 1" curve="s 0 0" size="1">
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
                    <composite type="cable" count="3 1 1" curve="s 0 0" size="1">
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
                <composite type="cable" count="2 1 1" curve="s 0 0" size="1">
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
                <composite type="cable" count="3 1 1" curve="s 0 0" size="1">
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
                <composite type="cable" prefix="A" count="3 1 1" curve="s 0 0" size="1">
                    <joint kind="main" damping="0.01"/>
                    <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
                </composite>
                <composite type="cable" prefix="B" count="4 1 1" curve="s 0 0" size="2" offset="0 1 0">
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
                <composite type="cable" count="3 1 1" curve="s 0 0" size="1">
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
            <composite type="cable" count="1 1 1" curve="s 0 0" size="1">
                <joint kind="main"/><geom type="capsule" size=".005"/>
            </composite>
        </worldbody></mujoco>
    "#;
    assert!(try_load_model(xml_a).is_err(), "count=1 should error");

    // (b) count="5 5 1" → multi-dimensional
    let xml_b = r#"
        <mujoco><worldbody>
            <composite type="cable" count="5 5 1" curve="s 0 0" size="1">
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
            <composite type="cable" count="5 1 1" vertex="0 0 0 1 0 0" curve="s 0 0" size="1">
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
            <composite type="cable" count="3 1 1" initial="xyz" curve="s 0 0" size="1">
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
            <composite type="cable" count="3 1 1" size="0 0 0" curve="s 0 0">
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

// ============================================================================
// T13: Cable sites take no class (supplementary)
// ============================================================================

/// A cable's boundary sites take no class of the document's: MuJoCo 3.5.0
/// builds them from the composite's own default (`user_composite.cc:112`,
/// `:437`), and on the oracle they are spheres of size 0.005 in group 3
/// under a root default and under the holder's childclass, each of which
/// makes the holder's own site a box of (0.03, 0.04, 0.05) in group 0.
#[test]
fn t13_cable_sites_take_no_class() {
    let box_site = r#"<site type="box" size="0.03 0.04 0.05"/>"#;
    for (default, holder) in [
        (box_site.to_string(), ""),
        (
            format!(r#"<default class="k">{box_site}</default>"#),
            r#" childclass="k""#,
        ),
    ] {
        let xml = format!(
            r#"
            <mujoco><default>{default}</default><worldbody>
                <body name="holder"{holder}>
                    <site name="own"/>
                    <composite type="cable" count="4 1 1" curve="s" size="1" initial="none">
                        <joint kind="main" damping="0.1"/>
                        <geom type="capsule" size=".01"/>
                    </composite>
                </body>
            </worldbody></mujoco>"#
        );
        let model = load_model(&xml);
        let own = model.site_id("own").expect("own");
        assert_eq!(model.site_type[own], GeomType::Box, "{xml}");
        assert_eq!(model.site_group[own], 0, "{xml}");
        for name in ["S_first", "S_last"] {
            let site = model.site_id(name).expect(name);
            assert_eq!(model.site_type[site], GeomType::Sphere, "{name}: {xml}");
            assert_eq!(model.site_group[site], 3, "{name}: {xml}");
            assert_eq!(
                model.site_size[site],
                nalgebra::Vector3::new(0.005, 0.005, 0.005),
                "{name}: {xml}"
            );
        }
    }
}

// ============================================================================
// The curve keywords, as MuJoCo 3.5.0's `shape_map` (book L07; ledger L260)
// ============================================================================

/// A cable of `count` vertices with `curve` and `size`, prefixed `C`.
fn curve_cable(count: u32, curve: &str, size: &str) -> String {
    format!(
        r#"<mujoco><worldbody>
            <composite type="cable" prefix="C" count="{count} 1 1" curve="{curve}" size="{size}">
                <geom type="capsule" size="0.01"/>
            </composite>
        </worldbody></mujoco>"#
    )
}

#[test]
fn cable_curve_keywords_as_mujoco() {
    // MuJoCo 3.5.0 (measured on the unfused oracle): `s` is the straight
    // line, `sin(s)` and `cos(s)` the sine and the cosine. Before: `s` was
    // the sine, `l` the line and `c` the cosine, and `sin(s)` was refused.
    let model = load_model(&curve_cable(6, "s 0 sin(s)", "1 0.2 1"));
    let b1 = model.name2id(ElementType::Body, "CB_1").expect("CB_1");
    let first = model
        .name2id(ElementType::Body, "CB_first")
        .expect("CB_first");
    // The vertices pass through f32, as MuJoCo's do (book L08).
    let pos = model.body_pos[b1];
    assert!((pos.x - 0.231_990_648_333_280_79).abs() < 1e-15, "{pos}");
    assert!(pos.y.abs() < 1e-15 && pos.z.abs() < 1e-15, "{pos}");
    let q = model.body_quat[first];
    let want = [
        0.682_294_608_073_227_5,
        -0.682_294_608_073_227_5,
        -0.185_671_935_935_942_9,
        -0.185_671_935_935_942_9,
    ];
    for (got, want) in [q.w, q.i, q.j, q.k].into_iter().zip(want) {
        assert!((got - want).abs() < 1e-15, "{q}");
    }
    // `cos(s) sin(s) 0` is read (MuJoCo loads it).
    let model = load_model(&curve_cable(6, "cos(s) sin(s) 0", "1 0.5 1"));
    assert_eq!(model.nbody, 6);
}

#[test]
fn a_curve_word_not_mujocos_is_refused() {
    // MuJoCo: "The curve array contains an invalid shape".
    for curve in ["l 0 0", "S 0 0", "zero 0 0", "s c 0"] {
        let err = try_load_model(&curve_cable(5, curve, "1")).unwrap_err();
        assert!(
            err.contains("invalid keyword") && err.contains("curve"),
            "{curve}: {err}"
        );
    }
    // MuJoCo: "The curve array must have a maximum of 3 components". Before:
    // a fourth word was dropped.
    let err = try_load_model(&curve_cable(5, "s 0 0 0", "1")).unwrap_err();
    assert!(err.contains("curve"), "{err}");
}

#[test]
fn space_after_the_last_curve_word_is_read_without_it() {
    // D-CURVE-WHITESPACE: MuJoCo 3.5.0 refuses `"s 0 0 "` ("maximum of 3
    // components") and `" "` ("invalid shape"): its read loop re-reads a stale
    // token after the last word. sim-mjcf reads them as `"s 0 0"` and `""`.
    let trimmed = load_model(&curve_cable(5, "s 0 0", "1"));
    let spaced = load_model(&curve_cable(5, "s 0 0 ", "1"));
    assert_eq!(trimmed.body_pos, spaced.body_pos);
    let empty = load_model(&curve_cable(5, "", "1"));
    let space = load_model(&curve_cable(5, " ", "1"));
    assert_eq!(empty.body_pos, space.body_pos);
    // Space before or between words MuJoCo reads too (measured).
    let model = load_model(&curve_cable(5, " s\t 0 0", "1"));
    assert_eq!(model.body_pos, trimmed.body_pos);
}

// ============================================================================
// Cable vertices through f32, and the 1- and 2-body cables (book L08)
// ============================================================================

#[test]
fn cable_vertices_round_through_f32() {
    // MuJoCo 3.5.0 stores a cable's vertices as floats (`user_composite.h`
    // `uservert`), measured: B_1 at x 0.3333333432674408, a third as a float.
    // Before: 0.3333333333333333.
    let model = load_model(&curve_cable(4, "s 0 0", "1"));
    let b1 = model.name2id(ElementType::Body, "CB_1").expect("CB_1");
    assert_eq!(model.body_pos[b1].x, f64::from((1.0_f64 / 3.0) as f32));
    assert_eq!(model.body_pos[b1].x, 0.333_333_343_267_440_8);
}

#[test]
fn two_body_cable_matches_mujoco_generator() {
    // D-CABLE-TWO-BODY: MuJoCo 3.5.0 refuses this document ("body 'B_1' not
    // found in bodypair 0"): its first body is also the second-last, and the
    // exclude it adds names B_1, which it names B_last. The reference is
    // MuJoCo's own expansion (MjSpec) with that one name corrected, written
    // at 17 digits and reloaded; its qpos and qvel at steps 1, 10, 100, 500,
    // measured on the unfused oracle (A9 §3.2, §3.5).
    let model = load_model(
        r#"<mujoco model="cable_uservert"><worldbody>
            <composite type="cable" vertex="0 0 0 1 0 0 2 0 1">
                <joint kind="main" damping="0.01"/>
                <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
            </composite></worldbody></mujoco>"#,
    );
    let first = model
        .name2id(ElementType::Body, "B_first")
        .expect("B_first");
    let last = model.name2id(ElementType::Body, "B_last").expect("B_last");
    assert_eq!(model.nbody, 3);
    assert_eq!(model.contact_excludes.len(), 1);
    assert!(
        model.contact_excludes.contains(&(first, last))
            || model.contact_excludes.contains(&(last, first))
    );
    let golden: [(usize, [f64; 8], [f64; 6]); 4] = [
        (
            1,
            [
                0.999_999_999_751_946,
                8.953_939_595_722_174e-19,
                5.529_945_652_121_103e-21,
                -2.227_348_174_714_429_7e-5,
                0.999_999_999_705_911_1,
                -3.769_642_584_006_882e-19,
                6.284_181_367_740_443e-19,
                2.425_237_540_270_774_7e-5,
            ],
            [
                8.953_939_596_462_529e-16,
                5.529_945_652_578_346e-18,
                -0.022_273_481_748_985_975,
                -3.769_642_584_376_418e-16,
                6.284_181_368_356_479e-16,
                0.024_252_375_405_085_198,
            ],
        ),
        (
            10,
            [
                0.999_999_253_068_526_3,
                6.499_068_096_296_155e-18,
                2.424_632_405_444_334_4e-19,
                -0.001_222_236_633_906_899_4,
                0.999_999_119_628_955_8,
                -3.254_940_872_205_717e-18,
                4.390_088_113_566_97e-18,
                0.001_326_929_279_528_090_3,
            ],
            [
                3.801_822_354_839_432e-16,
                4.243_494_806_032_707e-17,
                -0.221_930_447_818_991_4,
                -4.036_741_762_827_343e-16,
                2.300_527_761_263_919_6e-16,
                0.240_492_891_165_991_98,
            ],
        ),
        (
            100,
            [
                0.994_428_437_101_387_6,
                9.658_378_725_269_55e-17,
                1.581_522_154_775_960_5e-17,
                -0.105_413_867_608_072_89,
                0.994_933_276_017_639_5,
                5.831_972_834_501_562e-17,
                6.576_940_518_347_792e-17,
                0.100_537_437_170_475_92,
            ],
            [
                1.251_485_727_082_517_2e-15,
                1.196_772_447_386_961_7e-16,
                -1.990_439_464_273_854,
                2.307_465_664_822_542_6e-15,
                1.029_664_681_881_032_7e-15,
                1.646_146_460_520_004_2,
            ],
        ),
        (
            500,
            [
                0.444_813_913_413_780_4,
                -3.713_766_464_164_956e-17,
                1.249_043_912_490_699_5e-16,
                -0.895_623_013_568_498_6,
                0.883_496_203_927_752_7,
                -1.216_184_458_225_102e-16,
                1.038_383_506_741_981_1e-16,
                -0.468_438_317_866_131_1,
            ],
            [
                -5.519_423_639_833_345_4e-15,
                1.147_397_098_257_190_5e-15,
                -6.212_247_956_365_928,
                -6.700_500_083_138_36e-15,
                -3.409_192_451_329_129_7e-15,
                7.775_807_386_523_438_5,
            ],
        ),
    ];
    let mut data = model.make_data();
    let mut step = 0;
    for (at, qpos, qvel) in golden {
        while step < at {
            data.step(&model).expect("step");
            step += 1;
        }
        for (i, want) in qpos.iter().enumerate() {
            assert!(
                (data.qpos[i] - want).abs() < 1e-12,
                "step {at} qpos[{i}] {}",
                data.qpos[i]
            );
        }
        for (i, want) in qvel.iter().enumerate() {
            assert!(
                (data.qvel[i] - want).abs() < 1e-10,
                "step {at} qvel[{i}] {}",
                data.qvel[i]
            );
        }
    }
}

#[test]
fn one_body_cable_frame_is_defined() {
    // D-CABLE-ONE-BODY: MuJoCo 3.5.0 builds a 1-body cable's frame from an
    // uninitialised tangent (`user_composite.cc`, AddCableBody `tnext`, set
    // only when the body is not the last), so its quaternion depends on what
    // memory holds (A9 measured NaN; finite garbage in a later run), and makes
    // one site, S_first, at the far end. sim-mjcf takes that tangent as zero:
    // the body's x-axis is the edge, and it makes S_first at 0 and S_last at
    // the far end. Edges along the six axes and two oblique ones, written as
    // floats exactly.
    for v in [
        "1 0 0",
        "-1 0 0",
        "0 1 0",
        "0 -1 0",
        "0 0 1",
        "0 0 -1",
        "0.5 0.25 0.75",
        "-0.25 1 -0.5",
    ] {
        let model = load_model(&format!(
            r#"<mujoco><worldbody><composite type="cable" vertex="0 0 0 {v}">
                <geom type="capsule" size=".005"/>
            </composite></worldbody></mujoco>"#
        ));
        assert_eq!(model.nbody, 2, "{v}");
        assert_eq!(model.nsite, 2, "{v}");
        let edge: Vec<f64> = v.split(' ').map(|t| t.parse().expect("number")).collect();
        let edge = nalgebra::Vector3::new(edge[0], edge[1], edge[2]).normalize();
        let q = model.body_quat[1];
        assert!((q.quaternion().norm() - 1.0).abs() < 1e-12, "{v}: {q}");
        let x_axis = q * nalgebra::Vector3::x();
        assert!((x_axis - edge).norm() < 1e-12, "{v}: {x_axis}");
        let mut data = model.make_data();
        for _ in 0..500 {
            data.step(&model).expect("step");
        }
        assert!(data.qpos.iter().all(|x| x.is_finite()), "{v}");
    }
}
