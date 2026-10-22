//! A flex against a flex and against itself, against MuJoCo 3.5.0.
//!
//! MuJoCo pairs two flexes in its broadphase, descends their hierarchies
//! together, each pair of leaves to `mj_collideElems`, and sorts the pair's
//! contacts by element; without the midphase, every element of the first
//! against every element of the second. After every pair, each flex's
//! active elements against each other, through its hierarchy, by sweep and
//! prune, or each pair in turn. Two cables' elements meet as capsules, any
//! other two through the convex route (`engine_collision_driver.c:264-482`,
//! `:622-847`, `:1809-1861`, `:2002-2098`). The golden is
//! `assets/golden/flex/flex_self.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_flex_self_reference.py`, which describes its
//! cases).

use serde_json::Value;
use sim_mjcf::load_model;

use super::flex_golden::{differences, floats};

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_self.json"))
        .expect("flex self golden parses")
}

/// Each case of the golden as [`differences`] compares it.
#[test]
fn flexes_collide_with_each_other_as_mujoco() {
    let off = differences(&golden());
    assert!(
        off.is_empty(),
        "{} values differ:\n{}",
        off.len(),
        off.join("\n")
    );
}

/// A flex whose every vertex is pinned is rigid and does not collide with
/// itself (`engine_collision_driver.c:432`): the golden's folded strip, its
/// elements within reach of each other, collides with itself unpinned, and
/// not once every vertex is pinned.
#[test]
fn a_rigid_flex_does_not_collide_with_itself() {
    let strip = |pins: &str| {
        format!(
            r#"<mujoco><option gravity="0 0 0"><flag equality="disable"/></option><worldbody/>
              <deformable><flex name="s" dim="2" radius="0.01" mass="2">
                <contact selfcollide="narrow" internal="false"/>
                <vertex pos="0 0 0  0 0.05 0  0.05 0 0  0.05 0.05 0  0.1 0 0  0.1 0.05 0  0.15 0 0
                             0.15 0.05 0  0.2 0 0  0.2 0.05 0  0.212 0.012 0.012  0.212 0.062 0.012
                             0.162 0.012 0.012  0.162 0.062 0.012  0.112 0.012 0.012
                             0.112 0.062 0.012  0.062 0.012 0.012  0.062 0.062 0.012
                             0.012 0.012 0.012  0.012 0.062 0.012"/>
                <element data="0 2 1  2 3 1  2 4 3  4 5 3  4 6 5  6 7 5  6 8 7  8 9 7  8 10 9
                               10 11 9  10 12 11  12 13 11  12 14 13  14 15 13  14 16 15
                               16 17 15  16 18 17  18 19 17"/>
                {pins}
              </flex></deformable></mujoco>"#
        )
    };
    let contacts = |xml: &str| {
        let model = load_model(xml).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        (model.flex_rigid[0], data.ncon)
    };
    let (rigid, ncon) = contacts(&strip(""));
    assert!(!rigid && ncon > 0, "unpinned: {ncon} contacts with itself");
    let all = (0..20).map(|v| v.to_string()).collect::<Vec<_>>().join(" ");
    let (rigid, ncon) = contacts(&strip(&format!(r#"<pin id="{all}"/>"#)));
    assert!(rigid, "every vertex pinned: rigid");
    assert_eq!(ncon, 0, "a rigid flex: no contact with itself");
}

/// An elliptic cone of condim 4 or 6 on a flex's contacts at the dense size
/// with islands (registry `D-FLEX-NO-TREE`): neither side has a rotational
/// dof, so the torsional and rolling rows reach no tree, and MuJoCo 3.5.0
/// stops (`gen_flex_self_reference.py`'s `mujoco_cone_stops`). sim-core puts
/// those rows in their contact's island and steps on: each golden case
/// recorded without islands, run with them, keeps every contact's rows in
/// one island, and its acceleration within 1e-8 of MuJoCo's without islands.
#[test]
fn an_elliptic_cone_s_rows_stay_in_their_contact_s_island() {
    let golden = golden();
    let mut cases = 0;
    for case in golden["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        if !name.ends_with("_no_islands") {
            continue;
        }
        cases += 1;
        let ours = case["ours"].as_str().expect("ours");
        let with_islands = ours.replace(r#" island="disable""#, "");
        assert_ne!(with_islands, ours, "{name}: the island flag");
        let model = load_model(&with_islands).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert!(data.nisland > 0, "{name}: islands");
        for row in 0..data.efc_type.len() {
            assert!(data.efc_island[row] >= 0, "{name}: row {row} in an island");
            let same = row > 0
                && (data.efc_type[row], data.efc_id[row])
                    == (data.efc_type[row - 1], data.efc_id[row - 1]);
            if same {
                assert_eq!(
                    data.efc_island[row],
                    data.efc_island[row - 1],
                    "{name}: row {row} with its contact"
                );
            }
        }
        let want = floats(&case["qacc"]);
        for (i, (a, b)) in data.qacc.iter().zip(&want).enumerate() {
            assert!(
                (a - b).abs() <= 1e-8 * b.abs().max(1.0),
                "{name}: qacc[{i}] {a:e}, MuJoCo without islands {b:e}"
            );
        }
    }
    assert_eq!(cases, 3, "the golden's cases without islands");
}
