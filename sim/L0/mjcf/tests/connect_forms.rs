//! A `<connect>` in MuJoCo 3.5.0's two forms, `body1` with `anchor` or
//! `site1` with `site2`, and the forms it refuses (book L12).
//!
//! The census compares the loadable documents' dynamics with MuJoCo's.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use sim_core::{ConstraintType, EqualityType};
use sim_mjcf::{MjcfError, load_model};

/// A site-form connect holds its two sites' positions together: its rows'
/// residual is `site_xpos[site1] − site_xpos[site2]`, MuJoCo's
/// (`engine_core_constraint.c:438-443`), bit for bit, with no anchor matched
/// at `qpos0` (MuJoCo's first row −0.726…, measured on this model). The model
/// keeps it as the sites' bodies and the sites' positions in them
/// (D-CONNECT-SITE); the second connect is to a site of the world.
#[test]
fn a_site_form_connect_holds_its_sites_together() {
    let model = load_model(
        r#"<mujoco><worldbody>
<body name="a" pos="0 0 1"><freejoint/><geom size="0.1" mass="1"/><site name="sa" pos="0.1 0.2 0.3"/></body>
<body name="b" pos="1 0 1" quat="0.9 0.1 0.2 0.3"><freejoint/><geom size="0.1" mass="1"/>
<site name="sb" pos="-0.3 0 0.1" quat="0 1 0 0"/></body>
<site name="sw" pos="0 0 2"/></worldbody>
<equality><connect site1="sa" site2="sb"/><connect site1="sa" site2="sw"/></equality></mujoco>"#,
    )
    .expect("loads");
    assert_eq!(
        model.eq_type,
        [EqualityType::Connect, EqualityType::Connect]
    );
    assert_eq!(model.eq_obj1id, [1, 1]);
    assert_eq!(model.eq_obj2id, [2, 0]);
    let site = |name: &str| {
        model
            .site_name
            .iter()
            .position(|n| n.as_deref() == Some(name))
            .unwrap()
    };
    let (sa, sb, sw) = (site("sa"), site("sb"), site("sw"));
    assert_eq!(model.eq_data[0][..3], *model.site_pos[sa].as_slice());
    assert_eq!(model.eq_data[0][3..6], *model.site_pos[sb].as_slice());
    assert_eq!(model.eq_data[1][3..6], *model.site_pos[sw].as_slice());

    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let rows: Vec<usize> = (0..data.efc_type.len())
        .filter(|&r| data.efc_type[r] == ConstraintType::Equality)
        .collect();
    assert_eq!(rows.len(), 6);
    for (k, &r) in rows.iter().enumerate() {
        let (i, j) = if k < 3 { (sa, sb) } else { (sa, sw) };
        let want = data.site_xpos[i][k % 3] - data.site_xpos[j][k % 3];
        assert_eq!(data.efc_pos[r].to_bits(), want.to_bits(), "row {k}");
    }
    assert!(
        data.efc_pos[rows[0]] < -0.7,
        "no anchor is matched at qpos0"
    );
}

/// The forms MuJoCo refuses (measured): a site and a body attribute together,
/// and either form incomplete, where this names the attribute missing.
#[test]
fn a_connect_in_no_form_is_refused() {
    let refusal = |connect: &str| {
        load_model(&format!(
            r#"<mujoco><worldbody><body name="a"><freejoint/><geom size="0.1"/><site name="sa"/></body>
<site name="sw"/></worldbody><equality>{connect}</equality></mujoco>"#
        ))
        .expect_err(connect)
    };
    for connect in [
        r#"<connect site1="sa" site2="sw" anchor="0 0 0"/>"#,
        r#"<connect site1="sa" site2="sw" body2="a"/>"#,
    ] {
        let err = refusal(connect).to_string();
        assert!(
            err.contains("body and site semantics cannot be mixed"),
            "{connect}: {err}"
        );
    }
    for (connect, missing) in [
        ("<connect/>", "body1"),
        (r#"<connect body1="a"/>"#, "anchor"),
        (r#"<connect anchor="0 0 0"/>"#, "body1"),
        (r#"<connect body2="a" anchor="0 0 0"/>"#, "body1"),
        (r#"<connect site1="sa"/>"#, "site2"),
        (r#"<connect site2="sw"/>"#, "site1"),
    ] {
        let err = refusal(connect);
        assert!(
            matches!(&err, MjcfError::MissingAttribute { element, attribute, .. }
                if element == "connect" && attribute == missing),
            "{connect}: {err}"
        );
    }
}

/// The site form names each site's BODY, not the site: here the sites' ids
/// (2 and 3, after two sites before them) are not their bodies' (1 and 2).
#[test]
fn a_site_form_connect_names_its_sites_bodies() {
    let model = load_model(
        r#"<mujoco><worldbody><site name="w"/>
<body name="a" pos="0 0 1"><freejoint/><geom size="0.1" mass="1"/><site name="x"/><site name="sa" pos="0.1 0 0"/></body>
<body name="b" pos="1 0 1"><freejoint/><geom size="0.1" mass="1"/><site name="sb"/></body></worldbody>
<equality><connect site1="sa" site2="sb"/></equality></mujoco>"#,
    )
    .expect("loads");
    let site = |name: &str| {
        model
            .site_name
            .iter()
            .position(|n| n.as_deref() == Some(name))
            .unwrap()
    };
    assert_eq!((site("sa"), site("sb")), (2, 3));
    assert_eq!((model.eq_obj1id[0], model.eq_obj2id[0]), (1, 2));
}

/// A site-form connect's residual, as `site_xpos[a] − site_xpos[b]` sim-core
/// computes, bit for bit (it places sites as MuJoCo's `mj_local2Global`).
fn residual_is_the_sites(xml: &str, a: &str, b: &str) {
    let model = load_model(xml).expect("loads");
    let site = |name: &str| {
        model
            .site_name
            .iter()
            .position(|n| n.as_deref() == Some(name))
            .unwrap()
    };
    let (i, j) = (site(a), site(b));
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let rows: Vec<usize> = (0..data.efc_type.len())
        .filter(|&r| data.efc_type[r] == ConstraintType::Equality)
        .collect();
    assert_eq!(rows.len(), 3);
    for (k, &r) in rows.iter().enumerate() {
        let want = data.site_xpos[i][k] - data.site_xpos[j][k];
        assert_eq!(data.efc_pos[r].to_bits(), want.to_bits(), "{a}-{b} row {k}");
    }
}

/// A site MuJoCo places by its body's frame (within 1e-6 of its origin) or
/// by its inertial frame reads that frame's position, not its own: a
/// residual differing by the offset (measured, 3.6e-7 and 2.9e-7) before
/// the build set each point from the site's frame.
#[test]
fn a_site_form_connect_reads_a_site_placed_by_its_bodys_frame() {
    residual_is_the_sites(
        r#"<mujoco><worldbody>
<body name="a" pos="0 0 1" quat="0.9 0.1 0.2 0.3"><freejoint/><geom size="0.1" mass="1"/><site name="sa" pos="5e-7 0 0"/></body>
<site name="sw" pos="0.2 0.1 1.3"/></worldbody>
<equality><connect site1="sa" site2="sw"/></equality></mujoco>"#,
        "sa",
        "sw",
    );
    residual_is_the_sites(
        r#"<mujoco><worldbody>
<body name="b" pos="0 0 1" quat="0.9 0.1 0.2 0.3"><freejoint/><inertial pos="0.1 0 0" quat="0.7071068 0 0.7071068 0" mass="1" diaginertia="0.01 0.02 0.03"/><geom size="0.05" mass="0"/><site name="sb" pos="0.1000004 0 0" quat="0.7071068 0 0.7071068 0"/></body>
<site name="sw" pos="0.2 0.1 1.3"/></worldbody>
<equality><connect site1="sb" site2="sw"/></equality></mujoco>"#,
        "sb",
        "sw",
    );
}

/// An equality whose two objects are one is refused, as MuJoCo refuses
/// every kind of it ("element 'a' is repeated in equality constraint 0",
/// measured), the world standing for an unwritten `body2`.
#[test]
fn an_equality_on_one_object_twice_is_refused() {
    for (equality, name) in [
        (r#"<connect site1="sa" site2="sa"/>"#, "sa"),
        (r#"<connect body1="a" body2="a" anchor="0 0 0"/>"#, "a"),
        (r#"<connect body1="world" anchor="0 0 0"/>"#, "world"),
        (r#"<weld body1="a" body2="a"/>"#, "a"),
        (r#"<joint joint1="j1" joint2="j1"/>"#, "j1"),
        (r#"<tendon tendon1="t1" tendon2="t1"/>"#, "t1"),
    ] {
        let err = load_model(&format!(
            r#"<mujoco><worldbody><body name="a"><freejoint/><geom size="0.1"/><site name="sa"/></body>
<body name="b" pos="1 0 0"><joint name="j1" type="hinge"/><joint name="j2" type="slide"/><geom size="0.1"/></body></worldbody>
<tendon><fixed name="t1"><joint joint="j1" coef="1"/></fixed></tendon><equality>{equality}</equality></mujoco>"#
        ))
        .expect_err(equality)
        .to_string();
        assert!(
            err.contains(&format!(
                "element '{name}' is repeated in equality constraint 0"
            )),
            "{equality}: {err}"
        );
    }
}

/// Under `fusestatic`, a site-form connect holds the sites it names. MuJoCo
/// 3.5.0 resolves a site's name to its index from before the fusion, which
/// moving a fused body's sites renumbers (`user_model.cc:4346`, `:4371`), so
/// here its connect holds `sw` and `sb` (measured: `eq_obj1id` 0 = `sw`) and
/// its body `a` falls free (D-FUSESTATIC-SITE-INDEX).
#[test]
fn a_fused_models_site_connect_holds_the_sites_it_names() {
    let xml = r#"<mujoco><compiler fusestatic="true"/><worldbody>
<body name="a" pos="0 0 1"><freejoint/><geom size="0.1" mass="1"/><site name="sa" pos="0.1 0 0"/></body>
<body name="w" pos="0.5 0 2"><site name="sw" pos="0 0 0"/></body>
<body name="b" pos="1 0 1"><freejoint/><geom size="0.1" mass="1"/><site name="sb" pos="-0.1 0 0"/></body>
</worldbody>
<equality><connect site1="sa" site2="sb"/></equality></mujoco>"#;
    let model = load_model(xml).expect("loads");
    assert_eq!(model.nbody, 3, "w is fused into the world");
    residual_is_the_sites(xml, "sa", "sb");
}

/// An empty site name is refused as no site's: MuJoCo binds an unnamed site
/// to it (measured), D-EMPTY-NAME. A connect mixing forms names the site
/// attribute it holds.
#[test]
fn an_empty_or_mixed_connect_is_refused() {
    let err = load_model(
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><freejoint/><geom size="0.1" mass="1"/><site pos="0.1 0 0"/></body>
<site name="sw" pos="0 0 1.3"/></worldbody><equality><connect site1="" site2="sw"/></equality></mujoco>"#,
    )
    .expect_err("empty site1");
    assert!(err.to_string().contains("unknown site ''"), "{err}");
    let err = load_model(
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom size="0.1"/></body><site name="sw"/></worldbody>
<equality><connect site2="sw" body1="a"/></equality></mujoco>"#,
    )
    .expect_err("mixed");
    assert!(
        matches!(&err, MjcfError::InvalidValue { attribute, .. } if attribute == "site2"),
        "{err}"
    );
}
