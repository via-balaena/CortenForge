//! A flex's edge rows come from a flex equality, as MuJoCo's (`mjEQ_FLEX`,
//! `engine_core_constraint.c:616-643`): one row per non-rigid edge, typed as
//! an equality row with the equality's id, `solref` and `solimp`, and the
//! edge's `flexedge_invweight0` as its diagonal approximation
//! (`engine_setconst.c:716-750`). The grid asks for its equality with
//! `<edge equality="true"/>`, as a MuJoCo flexcomp does.
//!
//! Checked against the rule, and the grid against MuJoCo 3.5.0, whose own
//! document it is: 16 edges, 2 of them rigid, 14 rows (the unfused oracle,
//! Rigid-loading PR RL-8c).

use sim_core::{ConstraintType, EqualityType, Model, StepError, mj_solve_m};
use sim_mjcf::load_model;

/// MuJoCo's `mjMINVAL`.
const MJ_MINVAL: f64 = 1e-15;

/// A 3×3 cloth grid with its first three vertices pinned: 16 edges, the 2
/// between pinned vertices rigid (A13 §3.3).
const PINNED_GRID: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <flexcomp name="cloth" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" mass="0.9">
      <edge equality="true"/>
      <pin id="0 1 2"/>
    </flexcomp>
  </worldbody>
</mujoco>"#;

fn flex_equality(model: &Model) -> usize {
    (0..model.neq)
        .find(|&eq| model.eq_type[eq] == EqualityType::Flex)
        .expect("the grid's flex equality")
}

fn flex_rows(model: &Model, data: &sim_core::Data) -> Vec<usize> {
    (0..data.efc_type.len())
        .filter(|&r| {
            data.efc_type[r] == ConstraintType::Equality
                && model.eq_type[data.efc_id[r]] == EqualityType::Flex
        })
        .collect()
}

/// The grid's equality makes 14 rows, one per non-rigid edge; with the
/// equality inactive, none.
#[test]
fn flex_rows_only_with_edge_equality() {
    let mut model = load_model(PINNED_GRID).expect("load");
    assert_eq!(model.nflexedge, 16);
    assert_eq!(model.flexedge_rigid.iter().filter(|&&r| r).count(), 2);
    let eq = flex_equality(&model);
    assert_eq!(model.eq_obj1id[eq], 0, "names flex 0");
    assert!(model.flex_edgeequality[0]);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_eq!(flex_rows(&model, &data).len(), 14);

    model.eq_active[eq] = false;
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_eq!(flex_rows(&model, &data).len(), 0);
}

/// Each flex row's diagonal approximation is made as its edge's
/// `flexedge_invweight0`: `(1/m₁ + 1/m₂)/2` when both vertex bodies are
/// simple with slides only, else `J·M⁻¹·Jᵀ` at `qpos0`. `mj_makeImpedance`
/// makes the row's R from it, then remakes the diagApprox from that R
/// (`engine_core_constraint.c:1484`, `:1575-1578`).
#[test]
fn flex_edge_diag_approx_is_invweight0() {
    let model = load_model(PINNED_GRID).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let rows = flex_rows(&model, &data);
    let edges: Vec<usize> = (0..model.nflexedge)
        .filter(|&e| !model.flexedge_rigid[e])
        .collect();
    assert_eq!(rows.len(), edges.len());
    let (mut simple, mut general) = (0, 0);
    for (&r, &e) in rows.iter().zip(&edges) {
        let [v1, v2] = model.flexedge_vert[e];
        let (b1, b2) = (model.flexvert_bodyid[v1], model.flexvert_bodyid[v2]);
        let want = if model.body_simple[b1] == 2 && model.body_simple[b2] == 2 {
            simple += 1;
            (1.0 / model.body_mass[b1] + 1.0 / model.body_mass[b2]) / 2.0
        } else {
            general += 1;
            // The edge's Jacobian at qpos0, where the data still is.
            let mut j = vec![0.0; model.nv];
            let adr = model.flexedge_J_rowadr[e];
            for k in adr..adr + model.flexedge_J_rownnz[e] {
                j[model.flexedge_J_colind[k]] = data.flexedge_J[k];
            }
            let mut w = j.clone();
            mj_solve_m(&model, &data, &mut w);
            j.iter().zip(&w).map(|(a, b)| a * b).sum()
        };
        assert!(
            (model.flexedge_invweight0[e] - want).abs() <= 1e-12 * want.abs(),
            "edge {e}: flexedge_invweight0 {} vs {want}",
            model.flexedge_invweight0[e]
        );
        let imp = data.efc_imp[r];
        assert!(imp > 0.0 && imp < 1.0, "row {r}: impedance {imp}");
        let r_want = ((1.0 - imp) * model.flexedge_invweight0[e] / imp).max(MJ_MINVAL);
        assert_eq!(
            data.efc_R[r].to_bits(),
            r_want.to_bits(),
            "row {r}, edge {e}: R"
        );
        assert_eq!(
            data.efc_diagApprox[r].to_bits(),
            (r_want * imp / (1.0 - imp)).to_bits(),
            "row {r}, edge {e}: diagApprox"
        );
    }
    assert!(simple > 0, "the grid has edges between free vertices");
    eprintln!("edges: {simple} simple, {general} general");
}

/// Every row of a flex equality joins its own trees (MuJoCo `findEdges`,
/// `engine_island.c:327-333`): an unpinned grid's vertices, each its own
/// tree, form one island.
#[test]
fn flex_rows_join_every_edge_s_trees() {
    let model = load_model(&PINNED_GRID.replace(r#"<pin id="0 1 2"/>"#, "")).expect("load");
    assert_eq!(model.ntree, 9, "one tree per vertex");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_eq!(data.nisland, 1);
    assert!(data.tree_island[..model.ntree].iter().all(|&i| i == 0));
}

/// With sleep enabled, the grid's active flex equality is refused by the
/// first step, as MuJoCo's position stage raises on it (`mj_wakeEquality`,
/// `engine_sleep.c:401-403`; `sleep_parity`'s
/// `flex_equality_with_sleep_refused` holds MuJoCo's calls). Made inactive,
/// it steps: the box beside the grid falls asleep, and the grid makes no rows.
#[test]
fn a_flex_equality_with_sleep_enabled_is_refused() {
    let xml = PINNED_GRID
        .replace(
            r#"<option gravity="0 0 -9.81" timestep="0.001"/>"#,
            r#"<option gravity="0 0 -9.81" timestep="0.001"><flag sleep="enable"/></option>"#,
        )
        .replace(
            "</worldbody>",
            r#"  <geom type="plane" size="5 5 0.1" pos="0 0 -2"/>
    <body name="box" pos="3 0 -1.9">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>"#,
        );
    let mut model = load_model(&xml).expect("load");
    let eq = flex_equality(&model);
    assert_eq!(
        model.make_data().step(&model),
        Err(StepError::FlexEqualityWithSleep { eq })
    );
    model.eq_active[eq] = false;
    let box_body = (0..model.nbody)
        .find(|&b| model.body_name[b].as_deref() == Some("box"))
        .expect("the box");
    let box_tree = model.body_treeid[box_body];
    let mut data = model.make_data();
    let mut asleep_steps = 0;
    for _ in 0..2000 {
        data.step(&model).expect("step");
        if !data.tree_awake[box_tree] {
            asleep_steps += 1;
            assert!(flex_rows(&model, &data).is_empty(), "the grid's rows");
        }
    }
    assert!(asleep_steps > 0, "the box falls asleep");
}

/// A flex's rows follow its equality's `solref` and `solimp`: with them set
/// to (0.05, 0.5) and (0.7, 0.8, 0.002, 0.5, 2), each row's impedance lies
/// in [0.7, 0.8] and its reference acceleration is `−B·vel − K·imp·pos` with
/// `K = 1/(0.8²·0.05²·0.5²)` and `B = 2/(0.8·0.05)`.
#[test]
fn flex_rows_take_the_equality_s_solref() {
    let mut model = load_model(PINNED_GRID).expect("load");
    let eq = flex_equality(&model);
    model.eq_solref[eq] = [0.05, 0.5];
    model.eq_solimp[eq] = [0.7, 0.8, 0.002, 0.5, 2.0];
    let mut data = model.make_data();
    for _ in 0..20 {
        data.step(&model).expect("step");
    }
    data.forward(&model).expect("forward");
    let dmax = model.eq_solimp[eq][1];
    let (k, b) = (
        1.0 / (dmax * dmax * 0.05 * 0.05 * 0.5 * 0.5),
        2.0 / (dmax * 0.05),
    );
    let rows = flex_rows(&model, &data);
    assert_eq!(rows.len(), 14);
    let mut moving = 0;
    for r in rows {
        assert!(
            (0.7..=0.8).contains(&data.efc_imp[r]),
            "row {r}: impedance {} outside the equality's [0.7, 0.8]",
            data.efc_imp[r]
        );
        let want = -b * data.efc_vel[r] - k * data.efc_imp[r] * data.efc_pos[r];
        assert!(
            (data.efc_aref[r] - want).abs() <= 1e-9 * want.abs().max(1.0),
            "row {r}: aref {}, from the solref {want}",
            data.efc_aref[r]
        );
        if want.abs() > 1e-6 {
            moving += 1;
        }
    }
    assert!(moving > 0, "some edge is stretched or moving");
}

/// A flex has edge rows only where the document asks for them, as MuJoCo's
/// (`flex_edgeequality`, `user_model.cc:3432-3446`): the grid without its
/// `<edge equality="true"/>` has no equality and no row.
#[test]
fn flex_edge_rows_only_where_the_doc_asks() {
    let xml = PINNED_GRID.replace("      <edge equality=\"true\"/>\n", "");
    assert_ne!(xml, PINNED_GRID);
    let model = load_model(&xml).expect("load");
    assert_eq!(model.neq, 0);
    assert_eq!(model.flex_edgeequality, vec![false]);
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_eq!(flex_rows(&model, &data).len(), 0);
}

/// A 3×3 cloth whose vertices are bodies, the first row of them without
/// joints, under gravity with contacts off; its equality takes `solref`
/// (0.05, 1), not the flex's.
const CLOTH: &str = r#"<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body name="g_v0" pos="0.0 0.0 1"><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v1" pos="0.1 0.0 1"><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v2" pos="0.2 0.0 1"><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v3" pos="0.0 0.1 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v4" pos="0.1 0.1 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v5" pos="0.2 0.1 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v6" pos="0.0 0.2 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v7" pos="0.1 0.2 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v8" pos="0.2 0.2 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
  </worldbody>
  <deformable>
    <flex name="g" dim="2" body="g_v0 g_v1 g_v2 g_v3 g_v4 g_v5 g_v6 g_v7 g_v8" element="0 1 3 1 4 3 1 2 4 2 5 4 3 4 6 4 7 6 4 5 7 5 8 7">
      <contact contype="0" conaffinity="0"/>
    </flex>
  </deformable>
  <equality>
    <flex flex="g" solref="0.05 1"/>
  </equality>
</mujoco>"#;

/// The cloth steps as MuJoCo 3.5.0 steps it (the unfused oracle,
/// `scripts/gen_flex_equality_reference.py`): with its equality, 14 rows (16
/// edges, the 2 between the fixed vertices rigid) and MuJoCo's `qpos` after
/// 500 steps; without, no row and the free vertices falling as MuJoCo's.
#[test]
fn flex_edge_rows_match_mujoco_3_5_0() {
    let equality = "  <equality>\n    <flex flex=\"g\" solref=\"0.05 1\"/>\n  </equality>\n";
    assert!(CLOTH.contains(equality));
    let fall = -1.228_702_499_999_998_7;
    let cases: [(&str, String, usize, [f64; 18]); 2] = [
        (
            "with the equality",
            CLOTH.to_string(),
            14,
            [
                0.002_176_400_717_777_742,
                -0.201_370_929_487_046_8,
                -0.042_444_310_711_155_58,
                0.002_134_527_337_477_689_2,
                -0.201_085_658_778_568_15,
                -0.043_236_522_490_743_66,
                0.002_034_208_074_451_764_2,
                -0.200_935_516_028_789_5,
                -0.043_749_565_292_276_65,
                0.002_774_621_480_602_038_3,
                -0.407_603_005_114_603_4,
                -0.068_233_752_839_581_84,
                0.002_700_410_197_226_646_8,
                -0.406_623_159_553_067_4,
                -0.071_033_480_911_615_02,
                0.002_683_665_244_978_826,
                -0.406_394_344_230_245_1,
                -0.071_757_588_741_525_78,
            ],
        ),
        (
            "without",
            CLOTH.replace(equality, ""),
            0,
            [
                0.0, 0.0, fall, 0.0, 0.0, fall, 0.0, 0.0, fall, 0.0, 0.0, fall, 0.0, 0.0, fall,
                0.0, 0.0, fall,
            ],
        ),
    ];
    for (label, xml, rows, mujoco) in cases {
        let model = load_model(&xml).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert_eq!(flex_rows(&model, &data).len(), rows, "{label}");
        for _ in 0..500 {
            data.step(&model).expect("step");
        }
        assert!(
            data.qpos.iter().all(|q| q.is_finite()),
            "{label}: {:?}",
            data.qpos
        );
        let off = data
            .qpos
            .iter()
            .zip(mujoco)
            .map(|(q, m)| (q - m).abs())
            .fold(0.0, f64::max);
        assert!(off < 1e-12, "{label}: {:?}, MuJoCo {mujoco:?}", data.qpos);
    }
}
