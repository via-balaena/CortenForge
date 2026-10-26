//! Deep documents and trees: loaded on the loader's own stack, refused past
//! the depths it takes, and walked without recursing.
//!
//! A file of its own, so a stack overflow aborts this binary only.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{
    MjcfBody, MjcfError, MjcfJoint, MjcfModel, load_model, load_model_from_file, model_from_mjcf,
    validate, validate_tendons,
};

/// `n` bodies nested in the worldbody (the root `<mujoco>` at depth 1, the
/// worldbody at 2, body `k` at `2 + k`), each holding a `<geom/>` when
/// `geoms`; the innermost self-closing when `self_closing`.
fn nested_bodies(n: usize, geoms: bool, self_closing: bool) -> String {
    let mut xml = String::from("<mujoco><worldbody>");
    for k in 0..n {
        let last = k + 1 == n;
        if last && self_closing {
            xml.push_str("<body/>");
        } else {
            xml.push_str("<body>");
            if geoms {
                xml.push_str(r#"<geom size="0.1"/>"#);
            }
        }
    }
    for k in 0..n {
        if !(k == 0 && self_closing) {
            xml.push_str("</body>");
        }
    }
    xml.push_str("</worldbody></mujoco>");
    xml
}

#[test]
fn the_deepest_document_mujoco_loads_loads_from_a_small_thread() {
    // 496 nested bodies each with a geom: the deepest MuJoCo 3.5.0 loads. Its
    // parse alone took 3.3 MiB of stack in this profile (research A4 §1.4).
    let xml = nested_bodies(496, true, false);
    let model = std::thread::Builder::new()
        .stack_size(256 << 10)
        .spawn(move || load_model(&xml).map(|m| m.nbody))
        .unwrap()
        .join()
        .unwrap();
    assert_eq!(model.unwrap(), 497);
}

fn depth_limit(xml: &str) -> Option<usize> {
    match load_model(xml) {
        Ok(_) => None,
        Err(MjcfError::TooDeep { limit, .. }) => Some(limit),
        Err(other) => panic!("{other}"),
    }
}

#[test]
fn a_document_nests_as_deep_as_mujocos_reader_takes() {
    // With content: the innermost body at depth 498 loads, at 499 is refused.
    assert_eq!(depth_limit(&nested_bodies(496, false, false)), None);
    assert_eq!(depth_limit(&nested_bodies(497, false, false)), Some(498));
    // Self-closing at depth 499 loads; one at 500 sits in a body with content
    // at 499, refused first.
    assert_eq!(depth_limit(&nested_bodies(497, false, true)), None);
    assert_eq!(depth_limit(&nested_bodies(498, false, true)), Some(498));
}

#[test]
fn the_depth_rule_reads_the_document_after_its_includes_are_spliced() {
    // 300 nested bodies, the innermost including a file of 300 more. MuJoCo
    // 3.5.0 checks each file and loads it (nbody 601, measured); the loader
    // checks the spliced document, 600 deep.
    let dir = tempfile::tempdir().unwrap();
    let part = format!(
        "<mujoco>{}{}</mujoco>",
        "<body>".repeat(300),
        "</body>".repeat(300)
    );
    std::fs::write(dir.path().join("deep_part.xml"), part).unwrap();
    let main = format!(
        r#"<mujoco><worldbody>{}<include file="deep_part.xml"/>{}</worldbody></mujoco>"#,
        "<body>".repeat(300),
        "</body>".repeat(300)
    );
    let path = dir.path().join("main.xml");
    std::fs::write(&path, main).unwrap();
    let err = load_model_from_file(&path).unwrap_err();
    assert!(
        matches!(err, MjcfError::TooDeep { limit: 498, .. }),
        "{err}"
    );
}

/// A chain of `n` bodies built in code without recursing, the innermost
/// holding a joint "deep".
fn chain(n: usize) -> MjcfModel {
    let mut inner = MjcfBody::new(format!("b{}", n - 1))
        .with_joint(MjcfJoint::hinge("deep", nalgebra::Vector3::z()));
    for k in (0..n - 1).rev() {
        let mut outer = MjcfBody::new(format!("b{k}"));
        outer.children.push(inner);
        inner = outer;
    }
    MjcfModel::new("chain").with_body(inner)
}

/// `f` on a thread with a stack that holds a deep tree's clone and drop.
fn on_big_stack<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    std::thread::Builder::new()
        .stack_size(256 << 20)
        .spawn(f)
        .unwrap()
        .join()
        .unwrap()
}

#[test]
fn a_tree_built_in_code_deeper_than_4096_is_refused() {
    // 100,000 deep: past the limit, and too deep for the loader's own stack
    // to clone or walk, so each entry point refuses it before either.
    on_big_stack(|| {
        let model = chain(100_000);
        for err in [
            model_from_mjcf(&model, None).unwrap_err(),
            validate(&model).unwrap_err(),
            validate_tendons(&model).unwrap_err(),
        ] {
            assert!(
                matches!(err, MjcfError::TooDeep { limit: 4096, .. }),
                "{err}"
            );
        }
        // At the limit the depth guard passes (the worldbody's children at 1).
        assert!(validate(&chain(4096)).is_ok());
    });
}

#[test]
fn a_body_and_a_joint_are_found_at_the_bottom_of_a_deep_tree_from_a_small_thread() {
    // Built and dropped on a big stack (both recurse), searched from 256 KiB.
    on_big_stack(|| {
        let model = chain(100_000);
        std::thread::scope(|scope| {
            std::thread::Builder::new()
                .stack_size(256 << 10)
                .spawn_scoped(scope, || {
                    assert_eq!(
                        model.body("b99999").map(|b| b.name.as_str()),
                        Some("b99999")
                    );
                    assert_eq!(model.joint("deep").map(|j| j.name.as_str()), Some("deep"));
                    assert!(model.body("absent").is_none());
                })
                .unwrap()
                .join()
                .unwrap();
        });
    });
}

#[test]
fn all_bodies_lists_every_level_breadth_first() {
    let model = MjcfModel::new("m").with_body(
        MjcfBody::new("a")
            .with_child(MjcfBody::new("b").with_child(MjcfBody::new("c")))
            .with_child(MjcfBody::new("d")),
    );
    let names: Vec<&str> = model.all_bodies().iter().map(|b| b.name.as_str()).collect();
    assert_eq!(names, ["a", "b", "d", "c"]);
}

#[test]
fn a_wide_document_is_not_a_deep_one() {
    // 600 sibling bodies: more elements than the depth limit, none nested.
    let xml = format!(
        "<mujoco><worldbody>{}</worldbody></mujoco>",
        r#"<body><geom size="0.1"/></body>"#.repeat(600)
    );
    assert_eq!(load_model(&xml).unwrap().nbody, 601);
}

#[test]
fn a_cable_that_nests_past_4096_below_its_body_is_refused_after_expansion() {
    // 4097 vertices make 4096 bodies, each within the cable's own limit; in a
    // body they nest to 4097, refused by the tree guard once expanded.
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body name="holder">
                    <composite type="cable" count="4097 1 1" size="1 0 0">
                        <geom type="capsule" size="0.01"/>
                    </composite>
                </body>
            </worldbody>
        </mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(err, MjcfError::TooDeep { limit: 4096, .. }),
        "{err}"
    );
}

#[test]
fn a_cable_too_long_to_nest_is_refused_before_it_is_built() {
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body name="rope">
                    <composite type="cable" count="5000 1 1" size="1 0 0">
                        <geom type="capsule" size="0.01"/>
                    </composite>
                </body>
            </worldbody>
        </mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(err, MjcfError::TooDeep { limit: 4096, .. }),
        "{err}"
    );
    assert!(err.to_string().contains("(body 'rope')"), "{err}");
}
