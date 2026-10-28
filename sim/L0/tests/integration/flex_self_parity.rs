//! A flex against a flex and against itself, against MuJoCo 3.5.0.
//!
//! MuJoCo pairs two flexes in its broadphase, descends their hierarchies
//! together, each pair of leaves to `mj_collideElems`, and sorts the pair's
//! contacts by element; without the midphase, every element of the first
//! against every element of the second. After every pair, each flex's
//! internal collisions, then its active elements against each other,
//! through its hierarchy, by sweep and prune, or each pair in turn. Two
//! cables' elements meet as capsules, any other two through the convex route
//! (`engine_collision_driver.c:264-482`, `:622-847`, `:1714-1806`,
//! `:1809-1861`, `:2002-2180`). The golden is
//! `assets/golden/flex/flex_self.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_flex_self_reference.py`, which describes its
//! cases).

use serde_json::Value;
use sim_mjcf::load_model;

use super::flex_golden::{differences, floats};
use super::flex_test_utils::{triples, uniform, vertex_bodies};

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_self.json"))
        .expect("flex self golden parses")
}

/// The golden of a body's geoms against a flex (`flex_rigid_parity.rs`'s).
fn rigid_golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_rigid.json"))
        .expect("flex rigid golden parses")
}

/// The case `name` of `golden`.
fn case<'a>(golden: &'a Value, name: &str) -> &'a Value {
    golden["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("the golden's {name}"))
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

/// A rigid flex does not collide with itself (`engine_collision_driver.c:
/// 439`): the golden's folded strip, its elements within reach of each other,
/// collides with itself as a flex of bodies with three slides, and not once
/// it is rigid. A flex of jointless bodies is not rigid (MuJoCo's rigid flex
/// is one on a single body, which sim-mjcf refuses), so the jointless strip
/// is set rigid on the loaded model.
#[test]
fn a_rigid_flex_does_not_collide_with_itself() {
    let strip = |pinned: &[usize]| {
        let (bodies, names) = vertex_bodies(
            "s",
            &triples(
                "0 0 0  0 0.05 0  0.05 0 0  0.05 0.05 0  0.1 0 0  0.1 0.05 0  0.15 0 0
                 0.15 0.05 0  0.2 0 0  0.2 0.05 0  0.212 0.012 0.012  0.212 0.062 0.012
                 0.162 0.012 0.012  0.162 0.062 0.012  0.112 0.012 0.012
                 0.112 0.062 0.012  0.062 0.012 0.012  0.062 0.062 0.012
                 0.012 0.012 0.012  0.012 0.062 0.012",
            ),
            &uniform(2.0, 20),
            pinned,
            None,
        );
        format!(
            r#"<mujoco><option gravity="0 0 0"><flag equality="disable"/></option><worldbody>{bodies}</worldbody>
              <deformable><flex name="s" dim="2" radius="0.01" body="{names}"
                  element="0 2 1  2 3 1  2 4 3  4 5 3  4 6 5  6 7 5  6 8 7  8 9 7  8 10 9
                           10 11 9  10 12 11  12 13 11  12 14 13  14 15 13  14 16 15
                           16 17 15  16 18 17  18 19 17">
                <contact selfcollide="narrow" internal="false"/>
              </flex></deformable></mujoco>"#
        )
    };
    let contacts = |xml: &str, rigid: bool| {
        let mut model = load_model(xml).expect("load");
        assert!(!model.flex_rigid[0], "loaded: no rigid flex");
        model.flex_rigid[0] = rigid;
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        data.ncon
    };
    let ncon = contacts(&strip(&[]), false);
    assert!(ncon > 0, "three slides each: {ncon} contacts with itself");
    let all: Vec<usize> = (0..20).collect();
    assert_eq!(
        contacts(&strip(&all), true),
        0,
        "a rigid flex: no contact with itself"
    );
}

/// An elliptic cone of condim 4, 5 or 6 on a flex's contacts at the dense
/// size with islands (registry `D-FLEX-NO-TREE`): where neither side turns,
/// the torsional and rolling rows reach no tree, and MuJoCo 3.5.0 stops
/// (`gen_flex_self_reference.py`'s `mujoco_cone_stops`,
/// `gen_flex_rigid_reference.py`'s `mujoco_stops`). sim-core puts those rows
/// in their contact's island and steps on: each golden case recorded without
/// islands (a flex's own contacts, two flexes', two such pairs apart, and a
/// world plane's under a cloth), run with them, keeps every contact's rows
/// in one island, and its acceleration within 1e-8 of MuJoCo's without
/// islands.
#[test]
fn an_elliptic_cone_s_rows_stay_in_their_contact_s_island() {
    let (golden, rigid) = (golden(), rigid_golden());
    let cases: Vec<&Value> = golden["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .filter(|c| c["name"].as_str().expect("name").ends_with("_no_islands"))
        .chain([case(&rigid, "plane_cloth_elliptic_condim4_no_islands")])
        .collect();
    assert_eq!(cases.len(), 6, "the goldens' cases without islands");
    for case in cases {
        let name = case["name"].as_str().expect("name");
        let ours = case["ours"].as_str().expect("ours");
        let with_islands = ours.replace(r#" island="disable""#, "");
        assert_ne!(with_islands, ours, "{name}: the island flag");
        let model = load_model(&with_islands).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let islands = match name {
            "two_pairs_elliptic_condim4_no_islands" => 2,
            "plane_cloth_elliptic_condim4_no_islands" => 9,
            _ => 1,
        };
        assert_eq!(data.nisland, islands, "{name}: islands");
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
}

/// A pinned vertex is a body of its own in sim-mjcf, on its parent body in
/// MuJoCo (registry `D-FLEX-PINNED-BODY`): two elements each holding a
/// pinned vertex, or a world geom and such an element, collide here, where
/// MuJoCo's same-body skip drops them. On each of the golden's pinned scenes
/// sim-core lists every contact MuJoCo lists, and adds some, each between two
/// such sides.
#[test]
fn pinned_vertices_are_their_own_bodies() {
    let golden = golden();
    let scenes = golden["pinned"].as_array().expect("pinned scenes");
    assert_eq!(scenes.len(), 3, "the golden's pinned scenes");
    for scene in scenes {
        let name = scene["name"].as_str().expect("name");
        let model = load_model(scene["ours"].as_str().expect("ours")).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let id = |o: Option<usize>| o.map_or(-1, |x| i64::try_from(x).expect("fits"));
        let ours: Vec<Vec<i64>> = data
            .contacts
            .iter()
            .map(|c| {
                [c.geom, c.flex, c.elem, c.vert]
                    .iter()
                    .flat_map(|s| s.map(id))
                    .collect()
            })
            .collect();
        let mujoco: Vec<Vec<i64>> = scene["contacts"]
            .as_array()
            .expect("contacts")
            .iter()
            .map(|c| {
                c.as_array()
                    .expect("sides")
                    .iter()
                    .map(|x| x.as_i64().expect("id"))
                    .collect()
            })
            .collect();
        for c in &mujoco {
            assert!(ours.contains(c), "{name}: MuJoCo's contact {c:?}");
        }
        let pinned = |f: usize, v: usize| {
            scene["pins"][f.to_string()]
                .as_array()
                .is_some_and(|p| p.iter().any(|x| x.as_u64() == u64::try_from(v).ok()))
        };
        let pinned_side = |c: &sim_core::Contact, k: usize| match (c.geom[k], c.flex[k], c.elem[k])
        {
            (Some(g), _, _) => model.geom_body[g] == 0,
            (None, Some(f), Some(e)) => {
                let global = model.flex_elemadr[f] + e;
                let adr = model.flexelem_dataadr[global];
                model.flexelem_data[adr..adr + model.flexelem_datanum[global]]
                    .iter()
                    .any(|&v| pinned(f, v - model.flex_vertadr[f]))
            }
            _ => false,
        };
        let added: Vec<&sim_core::Contact> = data
            .contacts
            .iter()
            .zip(&ours)
            .filter(|(_, sides)| !mujoco.contains(sides))
            .map(|(c, _)| c)
            .collect();
        assert!(!added.is_empty(), "{name}: contacts MuJoCo skips");
        for c in added {
            assert!(
                pinned_side(c, 0) && pinned_side(c, 1),
                "{name}: an added contact between two pinned sides, {c:?}"
            );
        }
    }
}

/// A flex's condim of 2 is taken as 3 (registry `D-FLEX-CONDIM`), where
/// MuJoCo 3.5.0 makes contacts of dimension 2
/// (`gen_flex_self_reference.py`'s `mujoco_condim_kept`): the golden's bent
/// cable's internal contacts, and the rigid golden's sphere, of condim 1, on
/// a cloth.
#[test]
fn a_flex_condim_of_2_is_taken_as_3() {
    let (golden, rigid) = (golden(), rigid_golden());
    let cable = case(&golden, "cable_v_internal")["ours"]
        .as_str()
        .expect("ours");
    let sphere = case(&rigid, "sphere_cloth")["ours"].as_str().expect("ours");
    let flex = r#"selfcollide="none" internal="#;
    let geom = r#"<geom type="sphere" size="0.03"/>"#;
    assert_eq!(cable.matches(flex).count(), 1, "the cable's contact");
    assert_eq!(sphere.matches(flex).count(), 1, "the cloth's contact");
    assert_eq!(sphere.matches(geom).count(), 1, "the sphere");
    let documents = [
        (
            "the cable",
            cable.replace(flex, &format!(r#"condim="2" {flex}"#)),
        ),
        (
            "the sphere",
            sphere
                .replace(flex, &format!(r#"condim="2" {flex}"#))
                .replace(geom, r#"<geom type="sphere" size="0.03" condim="1"/>"#),
        ),
    ];
    for (name, xml) in documents {
        let model = load_model(&xml).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert!(data.ncon > 0, "{name}: contacts");
        assert!(
            data.contacts.iter().all(|c| c.dim == 3),
            "{name}: dimension 3"
        );
    }
}
