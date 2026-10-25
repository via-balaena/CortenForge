//! Phase 10 Spec D — Flex-Flex Cross-Object Collision Tests.
//!
//! Tests T1–T14 from SPEC_D.md. Verifies broadphase bitmask filtering,
//! contact parameter combination (priority, solmix, condim), margin/gap
//! additivity, flex_rigid non-gating, multi-flex all-pairs enumeration,
//! sentinel-safe downstream pipeline, and self/flex-rigid regression.

use approx::assert_relative_eq;
use sim_core::validation::max_abs_or_nan;
use sim_mjcf::load_model;

/// Helper: count flex-flex contacts (a flex on each side, two different
/// flexes).
fn count_flex_flex_contacts(data: &sim_core::Data) -> usize {
    data.contacts
        .iter()
        .filter(|c| matches!(c.flex, [Some(f1), Some(f2)] if f1 != f2))
        .count()
}

/// Two overlapping 3×3 dim=2 flexcomp grids for basic collision tests.
fn two_overlapping_grids_mjcf(ct1: i32, ca1: i32, ct2: i32, ca2: i32) -> String {
    format!(
        r#"
        <mujoco model="flex_flex">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="{ct1}" conaffinity="{ca1}" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  2 0 0  0 1 0  1 1 0  2 1 0  0 2 0  1 2 0  2 2 0"/>
                    <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="{ct2}" conaffinity="{ca2}" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.5 0.5 0  1.5 0.5 0  2.5 0.5 0  0.5 1.5 0  1.5 1.5 0  2.5 1.5 0  0.5 2.5 0  1.5 2.5 0  2.5 2.5 0"/>
                    <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
                </flex>
            </deformable>
        </mujoco>
        "#,
        ct1 = ct1,
        ca1 = ca1,
        ct2 = ct2,
        ca2 = ca2
    )
}

// ============================================================================
// T1: Incompatible bitmask → zero flex-flex contacts → AC1
// ============================================================================

#[test]
fn t01_incompatible_bitmask_zero_contacts() {
    let mjcf = two_overlapping_grids_mjcf(1, 1, 2, 2);
    let model = load_model(&mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let ff_count = count_flex_flex_contacts(&data);
    assert_eq!(
        ff_count, 0,
        "incompatible bitmask: (1&2)=0, (2&1)=0 → zero flex-flex contacts"
    );
}

// ============================================================================
// T2: Compatible bitmask → contacts with correct encoding → AC2, AC9
// ============================================================================

#[test]
fn t02_compatible_bitmask_generates_contacts() {
    let mjcf = two_overlapping_grids_mjcf(1, 3, 2, 1);
    let model = load_model(&mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let ff_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| matches!(c.flex, [Some(f1), Some(f2)] if f1 != f2))
        .collect();

    // AC2: contact count, MuJoCo 3.5.0's for these grids (the golden
    // `flex_self.json`'s case `coplanar_grids`, the same grids with a body
    // for each vertex).
    assert_eq!(ff_contacts.len(), 31, "MuJoCo's 31 flex-flex contacts");

    // AC9: contact encoding
    for c in &ff_contacts {
        assert_eq!(c.geom, [None; 2], "a flex-flex contact has no geom");
        assert!(
            c.elem.iter().all(Option::is_some),
            "an element on each side"
        );
        assert_eq!(c.vert, [None; 2], "no vertex");
        let [Some(fid1), Some(fid2)] = c.flex else {
            panic!("a flex on each side: {:?}", c.flex);
        };
        assert_ne!(
            fid1, fid2,
            "a flex-flex contact's sides are on different flexes"
        );
    }
}

// ============================================================================
// T3: Priority winner takes all → AC3
// ============================================================================

#[test]
fn t03_priority_winner_takes_all() {
    let mjcf = r#"
        <mujoco model="priority_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" priority="1"
                             solref="0.05 2.0" friction="0.5 0.005 0.0001" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" priority="0"
                             solref="0.02 1.0" friction="0.3 0.005 0.0001" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let ff_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| matches!(c.flex, [Some(f1), Some(f2)] if f1 != f2))
        .collect();

    assert!(
        !ff_contacts.is_empty(),
        "must generate flex-flex contacts for priority test"
    );

    for c in &ff_contacts {
        // Priority 1 (flex1) wins over priority 0 (flex2)
        assert_relative_eq!(c.solref[0], 0.05, epsilon = 1e-12);
        assert_relative_eq!(c.solref[1], 2.0, epsilon = 1e-12);
        assert_relative_eq!(c.mu[0], 0.5, epsilon = 1e-12);
    }
}

// ============================================================================
// T4: Solmix weighted blend → AC4
// ============================================================================

#[test]
fn t04_solmix_weighted_blend() {
    let mjcf = r#"
        <mujoco model="solmix_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1"
                             solmix="0.5" solref="0.02 1.0" friction="0.8 0.005 0.0001" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1"
                             solmix="1.5" solref="0.04 2.0" friction="0.2 0.005 0.0001" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let ff_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| matches!(c.flex, [Some(f1), Some(f2)] if f1 != f2))
        .collect();

    assert!(
        !ff_contacts.is_empty(),
        "must generate flex-flex contacts for solmix test"
    );

    // mix = 0.5 / (0.5 + 1.5) = 0.25
    // solref[0] = 0.25 * 0.02 + 0.75 * 0.04 = 0.035
    // solref[1] = 0.25 * 1.0 + 0.75 * 2.0 = 1.75
    // friction = max(0.8, 0.2) = 0.8
    for c in &ff_contacts {
        assert_relative_eq!(c.solref[0], 0.035, epsilon = 1e-12);
        assert_relative_eq!(c.solref[1], 1.75, epsilon = 1e-12);
        assert_relative_eq!(c.mu[0], 0.8, epsilon = 1e-12);
    }
}

// ============================================================================
// T5: Margin/gap additive → AC5
// ============================================================================

#[test]
fn t05_margin_gap_additive() {
    let mjcf = r#"
        <mujoco model="margin_gap_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.05" gap="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.03" gap="0.02"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let ff_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| matches!(c.flex, [Some(f1), Some(f2)] if f1 != f2))
        .collect();

    assert!(
        !ff_contacts.is_empty(),
        "must generate contacts for margin test"
    );

    // includemargin = (0.05 + 0.03) - (0.01 + 0.02) = 0.05
    for c in &ff_contacts {
        assert_relative_eq!(c.includemargin, 0.05, epsilon = 1e-12);
    }
}

// ============================================================================
// T6: flex_rigid does NOT gate flex-flex → AC6
// ============================================================================

#[test]
fn t06_flex_rigid_does_not_gate_flex_flex() {
    // Baseline: same geometry with both flexes unpinned (density > 0, no pin).
    let mjcf_unpinned = r#"
        <mujoco model="unpinned_baseline">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f_normal" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model_unpinned = load_model(mjcf_unpinned).expect("load unpinned");
    assert!(
        !model_unpinned.flex_rigid[0],
        "baseline flex must not be rigid"
    );
    let mut data_unpinned = model_unpinned.make_data();
    data_unpinned.forward(&model_unpinned).expect("forward");
    let unpinned_count = count_flex_flex_contacts(&data_unpinned);
    assert!(
        unpinned_count > 0,
        "baseline must produce flex-flex contacts"
    );

    // Test: flex1 fully pinned (invmass=0 → flex_rigid[0]=true)
    let mjcf_pinned = r#"
        <mujoco model="rigid_flex_test">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f_rigid" dim="2" density="0" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                    <pin id="0 1 2 3"/>
                </flex>
                <flex name="f_normal" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model_pinned = load_model(mjcf_pinned).expect("load pinned");
    assert!(
        model_pinned.flex_rigid[0],
        "first flex must be rigid (all pinned)"
    );
    assert!(!model_pinned.flex_rigid[1], "second flex must not be rigid");

    let mut data_pinned = model_pinned.make_data();
    data_pinned.forward(&model_pinned).expect("forward");

    let pinned_count = count_flex_flex_contacts(&data_pinned);
    assert_eq!(
        pinned_count, unpinned_count,
        "flex_rigid=true must NOT gate flex-flex: pinned count ({}) must equal unpinned count ({})",
        pinned_count, unpinned_count,
    );
}

// ============================================================================
// T7: Three-flex all-pairs enumeration → AC7
// ============================================================================

#[test]
fn t07_three_flex_all_pairs_enumeration() {
    let mjcf = r#"
        <mujoco model="three_flex">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f0" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.5 0.5 0  1.5 0.5 0  0.5 1.5 0  1.5 1.5 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Collect flex pairs from contacts
    let mut pairs: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
    for c in &data.contacts {
        if let [Some(fid1), Some(fid2)] = c.flex
            && fid1 != fid2
        {
            pairs.insert((fid1.min(fid2), fid1.max(fid2)));
        }
    }

    // 3 flex objects → C(3,2) = 3 pairs: (0,1), (0,2), (1,2)
    assert!(
        pairs.contains(&(0, 1)),
        "must have contacts between flex 0 and flex 1"
    );
    assert!(
        pairs.contains(&(0, 2)),
        "must have contacts between flex 0 and flex 2"
    );
    assert!(
        pairs.contains(&(1, 2)),
        "must have contacts between flex 1 and flex 2"
    );
    assert_eq!(pairs.len(), 3, "exactly 3 unique flex pairs");
}

// ============================================================================
// T8: Condim max combination → AC8
// ============================================================================

#[test]
fn t08_condim_max_combination() {
    // Test that condim combination uses max: condim=1 vs condim=4 → max=4.
    let mjcf = r#"
        <mujoco model="condim_test">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" condim="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" condim="4"
                             friction="1.0 0.005 0.0001" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let ff_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| matches!(c.flex, [Some(f1), Some(f2)] if f1 != f2))
        .collect();

    assert!(
        !ff_contacts.is_empty(),
        "must generate flex-flex contacts for condim test"
    );

    // max(1, 4) = 4 → dim = 4
    for c in &ff_contacts {
        assert_eq!(c.dim, 4, "condim must be max(1, 4) = 4");
    }
}

// ============================================================================
// T9: Full forward step — no panic, active constraint forces → AC10, AC15
// ============================================================================

/// Two 3×3 grids (known to produce contacts from T2) with gravity and
/// pinned top-row vertices, so they stay overlapping under gravity.
const FULL_STEP_SCENE: &str = r#"
        <mujoco model="full_step_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.05"/>
                    <elasticity young="1000" damping="10"/>
                    <vertex pos="0 0 0  1 0 0  2 0 0  0 1 0  1 1 0  2 1 0  0 2 0  1 2 0  2 2 0"/>
                    <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
                    <pin id="0 1 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.05"/>
                    <elasticity young="1000" damping="10"/>
                    <vertex pos="0.5 0.5 0  1.5 0.5 0  2.5 0.5 0  0.5 1.5 0  1.5 1.5 0  2.5 1.5 0  0.5 2.5 0  1.5 2.5 0  2.5 2.5 0"/>
                    <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
                    <pin id="0 1 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

#[test]
fn t09_full_forward_step_no_panic() {
    let model = load_model(FULL_STEP_SCENE).expect("load");
    let mut data = model.make_data();

    // Step 1: forward() to generate contacts and verify they exist
    data.forward(&model).expect("forward");
    let ff_count = count_flex_flex_contacts(&data);
    assert!(
        ff_count > 0,
        "overlapping flexes with large margin must produce flex-flex contacts"
    );

    // Step 2: full step exercises the entire pipeline including:
    // - Collision (flex-flex contacts generated)
    // - Constraint assembly (Jacobian for flex-flex contacts)
    // - Constraint solve
    // - Force distribution (acceleration.rs — S7a fix)
    // - Sleep/wake detection (sleep.rs — S7b fix)
    // - Island construction (island/mod.rs — S7d fix)
    // Must not panic at any sentinel geom_body[] access.
    data.step(&model).expect("step 1");
    data.step(&model).expect("step 2");
    data.step(&model).expect("step 3");

    // AC10: constraint forces flow through the full pipeline.
    // Check after the last step's forward() which is embedded in step().
    let has_constraint_force = max_abs_or_nan(&data.qfrc_constraint) > 1e-15;
    assert!(
        has_constraint_force,
        "flex-flex contacts must produce non-zero qfrc_constraint"
    );

    // Note: spec AC10 also requires cfrc_ext non-zero for flex vertex bodies.
    // cfrc_ext is lazy-populated by mj_body_accumulators(), which only runs
    // when triggered by acceleration/force/torque sensors. Without sensors in
    // this model, cfrc_ext stays zero. The qfrc_constraint assertion above
    // proves the full constraint pipeline (collision → Jacobian → assembly →
    // solve → force distribution) works for flex-flex contacts.
}

// ============================================================================
// T12: Flex-rigid regression → AC13
// ============================================================================

#[test]
fn t12_flex_rigid_regression() {
    let mjcf = r#"
        <mujoco model="flex_rigid_regression">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom type="plane" size="5 5 0.1" pos="0 0 -0.1"/>
            </worldbody>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Flex-rigid contacts should work normally after S7 changes.
    // Contact.bodies() for flex-rigid gives (geom_body, flexvert_bodyid).
    for c in &data.contacts {
        if c.geom[0].is_some() && c.vert[1].is_some() {
            // Flex-rigid contact: bodies() should return valid indices
            let [b1, b2] = c.bodies(&model).map(Option::unwrap);
            assert!(b1 < model.nbody, "rigid geom body must be valid");
            assert!(b2 < model.nbody, "flex body must be valid");
        }
    }

    // Full step should not panic
    data.step(&model).expect("step");
}

// ============================================================================
// T13: contact_param_flexes unit test → supplementary
// ============================================================================

#[test]
fn t13_contact_param_flexes_unit() {
    // Test the parameter combination function via actual flex-flex contacts.
    // Use overlapping 2×2 grids with large margins to ensure contacts are
    // generated for coplanar triangles (depth ≈ 0 needs margin > gap).
    let mjcf = r#"
        <mujoco model="param_unit">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1"
                             solref="0.02 1.0" solimp="0.9 0.95 0.001 0.5 2.0"
                             friction="0.5 0.005 0.0001" gap="0.01" margin="0.05"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1"
                             solref="0.04 2.0" solimp="0.8 0.99 0.002 0.6 3.0"
                             friction="0.3 0.003 0.0002" gap="0.02" margin="0.05"/>
                    <elasticity young="50"/>
                    <vertex pos="0.25 0.25 0  1.25 0.25 0  0.25 1.25 0  1.25 1.25 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.nflex, 2);

    // Verify model arrays loaded correctly
    assert_relative_eq!(model.flex_gap[0], 0.01, epsilon = 1e-12);
    assert_relative_eq!(model.flex_gap[1], 0.02, epsilon = 1e-12);

    // Generate contacts
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Filter to flex-flex contacts
    let ff_contacts: Vec<_> = data
        .contacts
        .iter()
        .filter(|c| matches!(c.flex, [Some(f1), Some(f2)] if f1 != f2))
        .collect();

    assert!(
        !ff_contacts.is_empty(),
        "co-located triangles must produce flex-flex contacts"
    );

    // Equal priority (both default=0), solmix both default=1.0
    // mix = 1.0 / (1.0 + 1.0) = 0.5
    // solref = 0.5 * [0.02, 1.0] + 0.5 * [0.04, 2.0] = [0.03, 1.5]
    // solimp = 0.5 * [0.9,0.95,0.001,0.5,2.0] + 0.5 * [0.8,0.99,0.002,0.6,3.0]
    //        = [0.85, 0.97, 0.0015, 0.55, 2.5]
    // friction = max(0.5, 0.3) = 0.5
    // includemargin = (0.05 + 0.05) - (0.01 + 0.02) = 0.07
    for c in &ff_contacts {
        assert_relative_eq!(c.solref[0], 0.03, epsilon = 1e-12);
        assert_relative_eq!(c.solref[1], 1.5, epsilon = 1e-12);
        assert_relative_eq!(c.solimp[0], 0.85, epsilon = 1e-12);
        assert_relative_eq!(c.solimp[1], 0.97, epsilon = 1e-12);
        assert_relative_eq!(c.solimp[2], 0.0015, epsilon = 1e-12);
        assert_relative_eq!(c.solimp[3], 0.55, epsilon = 1e-12);
        assert_relative_eq!(c.solimp[4], 2.5, epsilon = 1e-12);
        assert_relative_eq!(c.mu[0], 0.5, epsilon = 1e-12);
        assert_relative_eq!(c.includemargin, 0.07, epsilon = 1e-12);
    }
}

// ============================================================================
// T14: Zero-element flex early return → supplementary
// ============================================================================

#[test]
fn t14_zero_element_flex_no_panic() {
    // A flex with zero elements should not panic in flex-flex dispatch.
    // MJCF always generates elements, so we load a normal model, zero out one
    // flex's element count, and derive its element pairs and hierarchy again
    // (`Model::recompute_derived`'s table).
    let mjcf = r#"
        <mujoco model="zero_elem">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody/>
            <deformable>
                <flex name="f1" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0 0 0  1 0 0  0 1 0  1 1 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
                <flex name="f2" dim="2" density="1000" radius="0.01">
                    <contact contype="1" conaffinity="1" margin="0.01"/>
                    <elasticity young="50"/>
                    <vertex pos="0.5 0.5 0  1.5 0.5 0  0.5 1.5 0  1.5 1.5 0"/>
                    <element data="0 1 2  1 3 2"/>
                </flex>
            </deformable>
        </mujoco>
    "#;

    let mut model = load_model(mjcf).expect("load");

    // Verify baseline: both flexes have elements and produce contacts
    let mut data = model.make_data();
    data.forward(&model).expect("forward baseline");
    let baseline_count = count_flex_flex_contacts(&data);
    assert!(
        baseline_count > 0,
        "baseline must produce flex-flex contacts"
    );

    // Zero out flex 1's element count
    model.flex_elemnum[1] = 0;
    model.compute_flex_topology();
    model.compute_flex_bvhs();
    let mut data = model.make_data();
    data.forward(&model).expect("forward zero-elem");

    // Must not panic, and must produce zero flex-flex contacts
    let zero_count = count_flex_flex_contacts(&data);
    assert_eq!(
        zero_count, 0,
        "zero-element flex must produce no flex-flex contacts"
    );
}

/// [`sim_core::Data::contacts_between_bodies`] counts a contact with a flex
/// element on a side for no pair of bodies: two overlapping grids' contacts,
/// each between an element of each, count for none, the world's included.
#[test]
fn contacts_between_bodies_count_no_pair_for_two_flexes_elements() {
    let grids = two_overlapping_grids_mjcf(1, 1, 1, 1);
    let alone = r#"margin="0.01" selfcollide="none"/>"#;
    let xml = grids.replace(r#"margin="0.01"/>"#, alone);
    assert_eq!(
        xml.matches(alone).count(),
        2,
        "each grid without self-collision"
    );
    let model = load_model(&xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert!(data.ncon > 0, "the grids touch");
    for c in &data.contacts {
        assert!(
            c.elem.iter().all(Option::is_some),
            "an element on each side"
        );
    }
    assert_eq!(data.contacts_between_bodies(&model, 0, 0), 0);
    let pairs: usize = (0..model.nflexvert)
        .flat_map(|a| (0..model.nflexvert).map(move |b| (a, b)))
        .map(|(a, b)| {
            data.contacts_between_bodies(&model, model.flexvert_bodyid[a], model.flexvert_bodyid[b])
        })
        .sum();
    assert_eq!(pairs, 0, "no contact is between two vertex bodies");
}
