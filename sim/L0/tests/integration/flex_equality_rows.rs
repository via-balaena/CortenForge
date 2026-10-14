//! A flex's edge rows come from a flex equality, as MuJoCo's (`mjEQ_FLEX`,
//! `engine_core_constraint.c:616-643`): one row per non-rigid edge, typed as
//! an equality row with the equality's id, `solref` and `solimp`, and the
//! edge's `flexedge_invweight0` as its diagonal approximation
//! (`engine_setconst.c:716-750`). Until the parser reads `<equality><flex>`
//! (Rigid-loading L31) the builder gives every flex an active one.
//!
//! Checked against the rule: MuJoCo refuses this crate's `<deformable>`
//! documents.

use sim_core::{ConstraintType, EqualityType, Model, mj_solve_m};
use sim_mjcf::load_model;

/// A 3×3 cloth grid with its first three vertices pinned: 16 edges, the 2
/// between pinned vertices rigid (A13 §3.3).
const PINNED_GRID: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <deformable>
    <flexcomp name="cloth" type="grid" count="3 3 1" spacing="0.1" dim="2" mass="0.9">
      <pin id="0 1 2"/>
    </flexcomp>
  </deformable>
</mujoco>"#;

fn flex_equality(model: &Model) -> usize {
    (0..model.neq)
        .find(|&eq| model.eq_type[eq] == EqualityType::Flex)
        .expect("the builder's flex equality")
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

/// Each flex row's diagonal approximation is its edge's
/// `flexedge_invweight0`: `(1/m₁ + 1/m₂)/2` when both vertex bodies are
/// simple with slides only, else `J·M⁻¹·Jᵀ` at `qpos0`.
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
        assert_eq!(
            data.efc_diagApprox[r].to_bits(),
            model.flexedge_invweight0[e].to_bits(),
            "row {r}, edge {e}"
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
