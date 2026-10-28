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
