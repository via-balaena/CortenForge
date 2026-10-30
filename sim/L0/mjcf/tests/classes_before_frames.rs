//! An element's class applied where MuJoCo 3.5.0 applies it, when the
//! element is read (book L16): before a `<frame>` composes its pose, before
//! `discardvisual` and `fusestatic` look at it, and never to the bodies,
//! geoms, joints and sites a `<composite>` makes, which MuJoCo builds from
//! the composite's own default. Each value is MuJoCo 3.5.0's for the same
//! document (measured).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// The expected values are MuJoCo's, printed to every digit it gives, some of
// them near a named constant they are not.
#![allow(clippy::unreadable_literal, clippy::approx_constant)]

use approx::assert_relative_eq;
use nalgebra::Vector3;
use sim_mjcf::load_model;

/// Geom `g`'s quaternion as (w, x, y, z).
fn geom_quat(model: &sim_core::Model, g: usize) -> [f64; 4] {
    let q = model.geom_quat[g].into_inner();
    [q.w, q.i, q.j, q.k]
}

fn assert_quat(got: [f64; 4], want: [f64; 4]) {
    for (g, w) in got.iter().zip(want) {
        assert_relative_eq!(*g, w, epsilon = 1e-12);
    }
}

fn assert_vec3(got: Vector3<f64>, want: [f64; 3]) {
    assert_relative_eq!(got, Vector3::from(want), epsilon = 1e-12);
}

#[test]
fn a_class_pos_is_composed_by_its_frame() {
    let model = load_model(
        r#"<mujoco><default><geom pos="0.5 0 0"/></default><worldbody><body><frame pos="0 0 1"><geom size=".1"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_vec3(model.geom_pos[0], [0.5, 0.0, 1.0]);
}

#[test]
fn a_class_fromto_in_a_frame_is_moved_by_it() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><geom type="capsule" fromto="0.5 0 0 0.5 0 0.2"/></default></default><worldbody><body><frame pos="0 0 1"><geom class="c" size=".1"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_vec3(model.geom_pos[0], [0.5, 0.0, 1.1]);
    assert_quat(geom_quat(&model, 0), [6.123233995736766e-17, 1.0, 0.0, 0.0]);
    assert_vec3(model.geom_size[0], [0.1, 0.1, 0.0]);
}

/// Ledger L243's geom half: the segment is resolved in the frame, then the
/// frame is composed, so the turn about the segment is MuJoCo's.
#[test]
fn a_fromto_box_in_a_turned_frame_is_resolved_then_turned() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree"/><worldbody><body><frame euler="30 40 50"><geom type="box" size=".05 .01 .01" fromto="0.1 0 0 0.1 0.2 0.3"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_vec3(
        model.geom_pos[0],
        [
            0.08697612022024478,
            0.06059370323863397,
            0.17681503793303865,
        ],
    );
    assert_quat(
        geom_quat(&model, 0),
        [
            -0.1170489771899163,
            0.8558630002470293,
            0.5009047804146062,
            -0.05378162051057231,
        ],
    );
    assert_vec3(model.geom_size[0], [0.05, 0.05, 0.18027756377319948]);
}

#[test]
fn a_fromto_capsule_in_a_turned_frame_is_resolved_then_turned() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree"/><worldbody><body><frame euler="30 40 50"><geom type="capsule" size=".05" fromto="0.1 0 0 0.1 0.2 0.3"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_quat(
        geom_quat(&model, 0),
        [
            -0.1170489771899163,
            0.8558630002470293,
            0.5009047804146062,
            -0.05378162051057231,
        ],
    );
    assert_vec3(model.geom_size[0], [0.05, 0.18027756377319948, 0.0]);
}

/// Ledger L245: a class's `fromto` is the geom's own, in its frame.
#[test]
fn a_class_fromto_in_a_turned_frame_is_resolved_then_turned() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree"/><default><default class="c"><geom type="capsule" fromto="0 0 0 0 0 0.4"/></default></default><worldbody><body><frame euler="0 90 0"><geom class="c" size=".05"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_vec3(model.geom_pos[0], [0.2, 0.0, 4.4408920985006264e-17]);
    assert_quat(
        geom_quat(&model, 0),
        [
            4.329780281177467e-17,
            0.7071067811865476,
            4.329780281177466e-17,
            -0.7071067811865475,
        ],
    );
    assert_vec3(model.geom_size[0], [0.05, 0.2, 0.0]);
}

/// Ledger L332: a geom's own alternative orientation and its class's are
/// layered before a frame or a fused body turns the geom (one alternative per
/// element, the element's own over its class's).
#[test]
fn a_geoms_own_alternative_beats_its_class_alternative_in_a_frame() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree"/><default><default class="c"><geom euler="0 0 90"/></default><default class="q"><geom quat="0 1 0 0"/></default></default><worldbody><body><frame pos="0 0 1"><geom class="c" size=".1" axisangle="1 0 0 90"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_quat(
        geom_quat(&model, 0),
        [0.7071067811865476, 0.7071067811865475, 0.0, 0.0],
    );
}

#[test]
fn a_class_alternative_in_a_turned_frame_is_turned() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree"/><default><default class="c"><geom euler="0 0 90"/></default><default class="q"><geom quat="0 1 0 0"/></default></default><worldbody><body><frame euler="90 0 0"><geom class="c" size=".1"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_quat(
        geom_quat(&model, 0),
        [0.5000000000000001, 0.5, -0.4999999999999999, 0.5],
    );
}

#[test]
fn a_class_quat_in_a_turned_frame_is_turned() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree"/><default><default class="c"><geom euler="0 0 90"/></default><default class="q"><geom quat="0 1 0 0"/></default></default><worldbody><body><frame euler="90 0 0"><geom class="q" size=".1"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_quat(
        geom_quat(&model, 0),
        [-0.7071067811865475, 0.7071067811865476, 0.0, 0.0],
    );
}

/// A class's alternative replaces the geom's own `quat` (MuJoCo's reader
/// keeps one alternative per element; a `quat` is not one).
#[test]
fn a_class_alternative_replaces_an_own_quat_in_a_turned_frame() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree"/><default><default class="c"><geom euler="0 0 90"/></default><default class="q"><geom quat="0 1 0 0"/></default></default><worldbody><body><frame euler="90 0 0"><geom class="c" size=".1" quat="0 0 1 0"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_quat(
        geom_quat(&model, 0),
        [0.5000000000000001, 0.5, -0.4999999999999999, 0.5],
    );
}

#[test]
fn a_class_alternative_in_a_turned_fused_body_is_turned() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree" fusestatic="true"/><default><default class="c"><geom euler="0 0 90"/></default><default class="q"><geom quat="0 1 0 0"/></default></default><worldbody><body name="p"><joint/><geom size=".1"/><body euler="90 0 0"><geom class="c" size=".1"/></body></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.nbody, 2);
    assert_quat(
        geom_quat(&model, 1),
        [0.5000000000000001, 0.5, -0.4999999999999999, 0.5],
    );
}

#[test]
fn a_geoms_own_alternative_beats_its_class_alternative_in_a_turned_fused_body() {
    let model = load_model(
        r#"<mujoco><compiler angle="degree" fusestatic="true"/><default><default class="c"><geom euler="0 0 90"/></default><default class="q"><geom quat="0 1 0 0"/></default></default><worldbody><body name="p"><joint/><geom size=".1"/><body euler="90 0 0"><geom class="c" size=".1" axisangle="1 0 0 90"/></body></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.nbody, 2);
    assert_quat(geom_quat(&model, 1), [2.220446049250313e-16, 1.0, 0.0, 0.0]);
}

/// `discardvisual` drops a geom whose class makes it visual only.
#[test]
fn discardvisual_sees_a_geoms_class() {
    let model = load_model(
        r#"<mujoco><compiler discardvisual="true"/><default><default class="v"><geom contype="0" conaffinity="0"/></default></default><worldbody><body><joint/><geom size=".1"/><geom class="v" size=".2"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.ngeom, 1);
}

/// A geom fused into its parent keeps the class its body's `childclass` gave.
#[test]
fn fusestatic_keeps_a_fused_bodys_childclass() {
    let model = load_model(
        r#"<mujoco><compiler fusestatic="true"/><default><default class="f"><geom friction="0.3"/></default></default><worldbody><body name="p"><joint/><geom size=".1"/><body childclass="f" pos="0 0 .5"><geom size=".1"/></body></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.nbody, 2);
    assert_vec3(model.geom_friction[0], [1.0, 0.005, 0.0001]);
    assert_vec3(model.geom_friction[1], [0.3, 0.005, 0.0001]);
}

#[test]
fn a_cable_takes_no_main_class() {
    let model = load_model(
        r#"<mujoco><default><joint armature="0.2"/><geom friction="0.3"/></default><worldbody><body><composite type="cable" count="4 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.dof_armature, vec![0.0; 6]);
    for g in 0..model.ngeom {
        assert_relative_eq!(model.geom_friction[g].x, 1.0);
    }
}

#[test]
fn a_cable_takes_no_childclass() {
    let model = load_model(
        r#"<mujoco><default><default class="u"><joint armature="0.2"/><geom friction="0.3"/></default></default><worldbody><body childclass="u"><composite type="cable" count="4 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.dof_armature, vec![0.0; 6]);
    for g in 0..model.ngeom {
        assert_relative_eq!(model.geom_friction[g].x, 1.0);
    }
}

/// Ledger L308: a cable's geom is its template over MuJoCo's built-in geom,
/// not over the main class.
#[test]
fn a_cable_geom_is_its_template_over_the_built_in_geom() {
    let model = load_model(
        r#"<mujoco><default><geom friction="2 0.3 0.4" solref="0.03 0.5" condim="1"/></default><worldbody><body><composite type="cable" count="4 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01" friction="0.5" solref="0.05"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_vec3(model.geom_friction[0], [0.5, 0.005, 0.0001]);
    assert_relative_eq!(model.geom_solref[0][0], 0.05);
    assert_relative_eq!(model.geom_solref[0][1], 1.0);
    assert_eq!(model.geom_condim[0], 3);
}

/// Ledger L240: a cable's boundary sites take no main class either.
#[test]
fn a_cables_sites_take_no_main_class() {
    let model = load_model(
        r#"<mujoco><default><site size="0.02 0.03 0.04" group="1"/></default><worldbody><body><composite type="cable" count="4 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(
        model.site_name,
        vec![Some("S_first".into()), Some("S_last".into())]
    );
    for s in 0..2 {
        assert_vec3(model.site_size[s], [0.005, 0.005, 0.005]);
        assert_eq!(model.site_group[s], 3);
    }
}

/// Ledger L240 (a): a cable's last boundary site sits at its last body's
/// length (0.3333333134651184, from MuJoCo's `f32` vertices), at the cable's
/// far end.
#[test]
fn a_cables_last_site_is_at_its_far_end() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="h"><composite prefix="c" type="cable" count="4 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(
        model.site_name,
        vec![Some("cS_first".into()), Some("cS_last".into())]
    );
    assert_vec3(model.site_pos[0], [0.0, 0.0, 0.0]);
    assert_vec3(model.site_pos[1], [0.3333333134651184, 0.0, 0.0]);
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    assert_vec3(data.site_xpos[1], [1.0, 0.0, 0.0]);
}

/// Ledger L240 (b): a composite's own `<site>` gives its boundary sites their
/// size (over MuJoCo's 0.005, element-wise), group and color.
#[test]
fn a_cables_site_gives_its_boundary_sites() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="h"><composite prefix="c" type="cable" count="4 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/><site size="0.02" group="1" rgba="1 0 0 1"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap();
    for s in 0..2 {
        assert_vec3(model.site_size[s], [0.02, 0.005, 0.005]);
        assert_eq!(model.site_group[s], 1);
        for (got, want) in model.site_rgba[s].iter().zip([1.0, 0.0, 0.0, 1.0]) {
            assert_relative_eq!(*got, want);
        }
    }
}

#[test]
fn a_cables_site_size_of_three_is_taken_whole() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="h"><composite prefix="c" type="cable" count="4 1 1" curve="s" size="1" initial="none"><geom type="capsule" size=".01"/><site size="0.02 0.03 0.04"/></composite></body></worldbody></mujoco>"#,
    )
    .unwrap();
    for s in 0..2 {
        assert_vec3(model.site_size[s], [0.02, 0.03, 0.04]);
        assert_eq!(model.site_group[s], 3);
    }
}
