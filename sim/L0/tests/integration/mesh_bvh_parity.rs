//! Each mesh's bounding-volume hierarchy over its faces against MuJoCo
//! 3.5.0's (`mjCMesh::SetBoundingVolume`, `user_mesh.cc:847-870`; laid out
//! after the bodies', `user_model.cc:2134-2145`), as sim-mjcf loads a model.
//!
//! The golden is `assets/golden/ray/ray.json`'s `mesh_models`, from
//! `scripts/gen_ray_reference.py` on the unfused oracle: each a model of one
//! mesh geom on the world. Those MuJoCo keeps in their written frame (a box, a
//! box whose faces are 4 by 4 squares, an octahedron) are compared here
//! through the loader, every hierarchy field of the model bit for bit; the
//! others MuJoCo turns to their principal frame, which sim-mjcf does not
//! (book Rigid-loading L35), and sim-core's unit test
//! `each_mesh_hierarchy_is_mujocos` builds them from MuJoCo's vertices.

use serde_json::Value;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/ray/ray.json")).expect("ray golden parses")
}

/// MuJoCo's index, -1 for none, as an `Option`.
fn index(v: &Value) -> Option<usize> {
    usize::try_from(v.as_i64().expect("an index")).ok()
}

fn indices(v: &Value) -> Vec<Option<usize>> {
    v.as_array().expect("an array").iter().map(index).collect()
}

#[test]
fn a_mesh_hierarchy_loads_as_mujocos() {
    let golden = golden();
    let mut compared = Vec::new();
    for (name, case) in golden["mesh_models"].as_object().expect("mesh_models") {
        if !case["authored_frame"].as_bool().expect("a flag") {
            continue;
        }
        let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
        assert_eq!(
            Some(model.nbvhstatic),
            index(&case["nbvhstatic"]),
            "{name}: nbvhstatic"
        );
        assert_eq!(
            model.body_bvhnum,
            indices(&case["body_bvhnum"])
                .into_iter()
                .flatten()
                .collect::<Vec<_>>(),
            "{name}: body_bvhnum"
        );
        assert_eq!(
            model.mesh_bvhadr,
            indices(&case["mesh_bvhadr"]),
            "{name}: mesh_bvhadr"
        );
        assert_eq!(
            model.mesh_bvhnum,
            indices(&case["mesh_bvhnum"])
                .into_iter()
                .flatten()
                .collect::<Vec<_>>(),
            "{name}: mesh_bvhnum"
        );
        let children: Vec<[Option<usize>; 2]> = case["bvh_child"]
            .as_array()
            .expect("bvh_child")
            .iter()
            .map(|c| [index(&c[0]), index(&c[1])])
            .collect();
        assert_eq!(model.bvh_child, children, "{name}: bvh_child");
        assert_eq!(
            model.bvh_nodeid,
            indices(&case["bvh_nodeid"]),
            "{name}: bvh_nodeid"
        );
        assert_eq!(
            model.bvh_depth,
            indices(&case["bvh_depth"])
                .into_iter()
                .flatten()
                .collect::<Vec<_>>(),
            "{name}: bvh_depth"
        );
        let boxes = case["bvh_aabb"].as_array().expect("bvh_aabb");
        assert_eq!(model.bvh_aabb.len(), boxes.len(), "{name}: nodes");
        for (k, (ours, theirs)) in model.bvh_aabb.iter().zip(boxes).enumerate() {
            let theirs: Vec<f64> = theirs
                .as_array()
                .expect("a box")
                .iter()
                .map(|x| x.as_f64().expect("a number"))
                .collect();
            assert!(
                ours.iter()
                    .zip(&theirs)
                    .all(|(a, b)| a.to_bits() == b.to_bits()),
                "{name} node {k}: box {ours:?}, MuJoCo {theirs:?}"
            );
        }
        compared.push(name.as_str());
    }
    assert_eq!(
        compared,
        ["box", "box4", "octahedron"],
        "the meshes compared"
    );
}
