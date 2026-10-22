//! What MuJoCo 3.5.0's compiler makes for a flex's collisions, and what
//! `mj_flex` makes of it each step: each flex's radius, element layers,
//! element–vertex pairs and bounding-volume hierarchy, then each element's
//! box and each hierarchy node's box with every dof moved.
//!
//! The golden is `assets/golden/flex/flex_data.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_flex_data_reference.py`, which
//! describes its cases): MuJoCo's flexcomps' vertices and elements, which
//! each case writes in sim-mjcf's `<deformable><flex>` form.

use serde_json::Value;
use sim_core::Model;
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_data.json"))
        .expect("flex data golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn ints(v: &Value) -> Vec<i64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_i64().expect("integer"))
        .collect()
}

fn count(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("count")).expect("count")
}

/// An index as MuJoCo stores it, -1 for none.
fn id(x: Option<usize>) -> i64 {
    x.map_or(-1, |x| i64::try_from(x).expect("index"))
}

/// The case's flexes in sim-mjcf's form, after the same free ball.
fn ours(case: &Value) -> Model {
    let mut xml = String::from(
        r#"<mujoco><option gravity="0 0 0"><flag equality="disable"/></option><worldbody>
<body name="ball" pos="1 1 1"><freejoint/><geom type="sphere" size="0.05"/></body>
</worldbody><deformable>"#,
    );
    for (k, flex) in case["flexes"]
        .as_array()
        .expect("flexes")
        .iter()
        .enumerate()
    {
        let vertices: Vec<String> = flex["vertices"]
            .as_array()
            .expect("vertices")
            .iter()
            .map(|v| {
                let p = floats(v);
                format!("{:?} {:?} {:?}", p[0], p[1], p[2])
            })
            .collect();
        let elements: Vec<String> = ints(&flex["elements"]).iter().map(i64::to_string).collect();
        xml += &format!(
            r#"<flex name="f{k}" dim="{}" radius="{:?}"><contact selfcollide="none" internal="false" activelayers="{}"/><vertex pos="{}"/><element data="{}"/></flex>"#,
            flex["dim"],
            flex["radius"].as_f64().expect("radius"),
            flex["activelayers"],
            vertices.join("  "),
            elements.join(" "),
        );
    }
    xml += "</deformable></mujoco>";
    load_model(&xml).expect("load")
}

/// Each flex's radius, its elements' layers, its element–vertex pairs and
/// its hierarchy (its nodes' children, leaf elements and depths, numbered
/// within it, and its first node past the bodies'), as MuJoCo's compiler
/// makes them (`user_mesh.cc:4423-4671`): a cable, then a cloth, a 4 × 4 × 4
/// grid of tetrahedra (layers 0 to 3, only layer 0 active), and the same
/// grid with two layers active. A model derived again comes out the same,
/// and with a ground plane added its flexes keep their hierarchies.
#[test]
fn flex_layers_pairs_and_hierarchies_as_mujoco() {
    for case in golden()["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        let model = ours(case);
        // Derived again, the same, with no node added twice: in full, then
        // the flexes' hierarchies alone once more.
        let mut again = model.clone();
        again.recompute_derived().expect("recompute");
        again.compute_flex_bvhs();
        let flex_data = |m: &Model| {
            (
                m.bvh_child.clone(),
                m.bvh_nodeid.clone(),
                m.bvh_depth.clone(),
                (m.nbvhstatic, m.nbvhdynamic),
                (m.flex_bvhadr.clone(), m.flex_bvhnum.clone()),
                m.flex_elemlayer.clone(),
                (
                    m.flex_evpair.clone(),
                    m.flex_evpairadr.clone(),
                    m.flex_evpairnum.clone(),
                ),
            )
        };
        assert_eq!(
            flex_data(&again),
            flex_data(&model),
            "{name}: derived again"
        );
        // A ground plane joins the world's hierarchy; each flex's nodes
        // follow the bodies' again, the same, and the model steps.
        let mut grounded = model.clone();
        grounded.add_ground_plane();
        let tail = |m: &Model| {
            let s = m.nbvhstatic;
            (
                m.bvh_child[s..].to_vec(),
                m.bvh_nodeid[s..].to_vec(),
                m.bvh_depth[s..].to_vec(),
                m.flex_bvhnum.clone(),
                m.flex_bvhadr
                    .iter()
                    .map(|a| a.map(|a| a - s))
                    .collect::<Vec<_>>(),
            )
        };
        assert_eq!(tail(&grounded), tail(&model), "{name}: with a ground plane");
        let mut data = grounded.make_data();
        data.forward(&grounded)
            .expect("forward with a ground plane");
        assert_eq!(model.nflexelem, count(&case["nflexelem"]), "{name}");
        assert_eq!(
            model.nbvhdynamic,
            count(&case["nbvhdynamic"]),
            "{name}: nodes"
        );
        for (f, flex) in case["flexes"]
            .as_array()
            .expect("flexes")
            .iter()
            .enumerate()
        {
            let at = format!("{name} flex {f}");
            assert_eq!(
                model.flex_radius[f].to_bits(),
                flex["radius"].as_f64().expect("radius").to_bits(),
                "{at}: radius"
            );
            let (ea, en) = (model.flex_elemadr[f], model.flex_elemnum[f]);
            let layer: Vec<i64> = model.flex_elemlayer[ea..ea + en]
                .iter()
                .map(|&l| id(Some(l)))
                .collect();
            assert_eq!(layer, ints(&flex["elemlayer"]), "{at}: layers");
            let (pa, pn) = (model.flex_evpairadr[f], model.flex_evpairnum[f]);
            let pairs: Vec<i64> = model.flex_evpair[pa..pa + pn]
                .iter()
                .flatten()
                .map(|&x| id(Some(x)))
                .collect();
            assert_eq!(pairs, ints(&flex["evpair"]), "{at}: pairs");
            let bn = model.flex_bvhnum[f];
            assert_eq!(bn, count(&flex["bvhnum"]), "{at}: nodes");
            let Some(ba) = model.flex_bvhadr[f] else {
                assert!(
                    flex.get("bvhadr_past_static").is_none(),
                    "{at}: a hierarchy"
                );
                continue;
            };
            assert_eq!(
                ba - model.nbvhstatic,
                count(&flex["bvhadr_past_static"]),
                "{at}: first node"
            );
            let child: Vec<i64> = model.bvh_child[ba..ba + bn]
                .iter()
                .flatten()
                .map(|&c| id(c))
                .collect();
            assert_eq!(child, ints(&flex["bvh_child"]), "{at}: children");
            let nodeid: Vec<i64> = model.bvh_nodeid[ba..ba + bn]
                .iter()
                .map(|&e| id(e))
                .collect();
            assert_eq!(nodeid, ints(&flex["bvh_nodeid"]), "{at}: leaves");
            let depth: Vec<i64> = model.bvh_depth[ba..ba + bn]
                .iter()
                .map(|&d| id(Some(d)))
                .collect();
            assert_eq!(depth, ints(&flex["bvh_depth"]), "{at}: depths");
        }
    }
}

/// With every dof moved, each element's box over its vertices grown by its
/// flex's radius, and each hierarchy node's box (a leaf its element's, a
/// node its children's together), as MuJoCo's `mj_flex`
/// (`engine_core_smooth.c:610-662`), bit for bit. With the midphase
/// disabled the nodes' boxes are left as they were.
#[test]
fn flex_element_and_hierarchy_boxes_as_mujoco() {
    for case in golden()["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        let mut model = ours(case);
        let mut data = model.make_data();
        let moved = floats(&case["moved_qpos"]);
        assert_eq!(data.qpos.len(), moved.len(), "{name}: qpos");
        data.qpos.as_mut_slice().copy_from_slice(&moved);
        data.forward(&model).expect("forward");
        let elements: Vec<f64> = data.flexelem_aabb.iter().flatten().copied().collect();
        let nodes: Vec<f64> = data.bvh_aabb_dyn.iter().flatten().copied().collect();
        for (what, ours, want) in [
            ("element boxes", &elements, floats(&case["flexelem_aabb"])),
            ("node boxes", &nodes, floats(&case["bvh_aabb_dyn"])),
        ] {
            assert_eq!(ours.len(), want.len(), "{name}: {what}");
            for (k, (a, b)) in ours.iter().zip(&want).enumerate() {
                assert_eq!(
                    a.to_bits(),
                    b.to_bits(),
                    "{name}: {what}[{k}] {a}, MuJoCo {b}"
                );
            }
        }

        model.disableflags |= sim_core::DISABLE_MIDPHASE;
        let before = data.bvh_aabb_dyn.clone();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(model.qpos0.as_slice());
        data.forward(&model).expect("forward");
        assert_eq!(data.bvh_aabb_dyn, before, "{name}: midphase disabled");
        assert_ne!(
            data.flexelem_aabb
                .iter()
                .flatten()
                .copied()
                .collect::<Vec<f64>>(),
            elements,
            "{name}: the element boxes move all the same"
        );
    }
}
