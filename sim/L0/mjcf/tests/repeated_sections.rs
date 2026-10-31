//! A document's `<compiler>` and `<option>`, written more than once, and the one `<flag>` each
//! `<option>` may hold, as MuJoCo 3.5.0 reads them (`xml_native_reader.cc`, the Compiler and
//! Option loops): each reads onto the same settings and sets only what it writes. Each expected
//! value was measured on MuJoCo 3.5.0.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use sim_core::{DISABLE_CONTACT, DISABLE_GRAVITY, ENABLE_ENERGY, ENABLE_SLEEP, Model};
use sim_mjcf::load_model;

fn load(xml: &str) -> Model {
    load_model(xml).unwrap_or_else(|e| panic!("refused: {e}"))
}

/// A hinge with a range, under `sections`.
fn with(sections: &str) -> String {
    format!(
        r#"<mujoco>{sections}<worldbody><body><joint type="hinge" range="-1 1"/><geom size="0.1"/></body></worldbody></mujoco>"#
    )
}

#[test]
fn a_later_compiler_keeps_an_earlier_ones_settings() {
    // Before: a later <compiler> reset every setting it did not write, so
    // `angle="radian"` was lost and the range read in degrees.
    let m = load(&with(
        r#"<compiler angle="radian"/><compiler autolimits="true"/>"#,
    ));
    assert_eq!(m.jnt_range[0], (-1.0, 1.0));
    // Ledger L248 (dm_control's rodent): `angle`, then `meshdir`. MuJoCo:
    // the body's quat is (0.9848134698792882, 0.17361575258114187, 0, 0).
    let m = load(
        r#"<mujoco><compiler angle="radian"/><compiler meshdir="./"/>
        <worldbody><body euler="0.349 0 0"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    let q = m.body_quat[1];
    assert!((q.w - 0.984_813_469_879_288_2).abs() < 1e-15, "{q}");
    assert!((q.i - 0.173_615_752_581_141_87).abs() < 1e-15, "{q}");
}

#[test]
fn a_later_option_keeps_an_earlier_ones_settings_and_flags() {
    let m = load(&with(
        r#"<option timestep="0.001"/><option><flag energy="enable"/></option>"#,
    ));
    assert_eq!(m.timestep, 0.001);
    assert_ne!(m.enableflags & ENABLE_ENERGY, 0);
    // Ledger L75: a second <option> kept the sleep flag (before: cleared it).
    let m = load(&with(
        r#"<option><flag sleep="enable"/></option><option timestep="0.001"/>"#,
    ));
    assert_eq!(m.timestep, 0.001);
    assert_ne!(m.enableflags & ENABLE_SLEEP, 0);
    let m = load(&with(r#"<option gravity="0 0 -1"/><option wind="1 0 0"/>"#));
    assert_eq!(m.gravity.z, -1.0);
    assert_eq!(m.wind.x, 1.0);
}

#[test]
fn a_flag_in_each_of_two_options_merges() {
    let m = load(&with(
        r#"<option><flag gravity="disable"/></option><option><flag contact="disable"/></option>"#,
    ));
    assert_eq!(m.disableflags, DISABLE_GRAVITY | DISABLE_CONTACT);
    let m = load(&with(
        r#"<option><flag gravity="disable"/></option><option><flag gravity="enable"/></option>"#,
    ));
    assert_eq!(m.disableflags, 0);
}

#[test]
fn a_later_options_prefix_lies_over_an_earlier_ones() {
    // Book L09's Also line: o_solref 0.05 0.5, then 0.07, is 0.07 0.5; an
    // o_friction of three, then one, keeps the second and third.
    let m = load(&with(
        r#"<option o_solref="0.05 0.5" o_friction="0.7 0.8 0.1"/><option o_solref="0.07" o_friction="0.3"/>"#,
    ));
    assert_eq!(m.o_solref, [0.07, 0.5]);
    assert_eq!(m.o_friction, [0.3, 0.8, 0.1, 0.0001, 0.0001]);
    // o_solimp likewise (MuJoCo: 0.5 0.9 0.01 0.4 3, measured).
    let m = load(&with(
        r#"<option o_solimp="0.8 0.9 0.01 0.4 3"/><option o_solimp="0.5"/>"#,
    ));
    assert_eq!(m.o_solimp, [0.5, 0.9, 0.01, 0.4, 3.0]);
}

#[test]
fn a_later_options_disabled_actuator_groups_add_to_an_earlier_ones() {
    // MuJoCo ORs each group into the same mask: 1 2, then 3, is 0b1110.
    let m = load(&with(
        r#"<option actuatorgroupdisable="1 2"/><option actuatorgroupdisable="3"/>"#,
    ));
    assert_eq!(m.disableactuator, 0b1110);
    let m = load(&with(
        r#"<option actuatorgroupdisable="1 2"/><option timestep="0.001"/>"#,
    ));
    assert_eq!(m.disableactuator, 0b0110);
}

#[test]
fn a_compiler_with_a_closing_tag_keeps_an_earlier_ones_settings() {
    // Found untested by 0a1d7075's mutants: `<compiler>…</compiler>` takes
    // its own path through the parser. MuJoCo (measured): range -1 1.
    let m = load(&with(
        r#"<compiler angle="radian"></compiler><compiler autolimits="true"></compiler>"#,
    ));
    assert_eq!(m.jnt_range[0], (-1.0, 1.0));
}

/// Loads `compilers` over a unit tetrahedron mesh, `tet.obj`, found only in `b/`.
fn tetrahedron_found_in_b(compilers: &str) -> Result<Model, sim_mjcf::MjcfError> {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir(dir.path().join("a")).unwrap();
    std::fs::create_dir(dir.path().join("b")).unwrap();
    std::fs::write(
        dir.path().join("b/tet.obj"),
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n",
    )
    .unwrap();
    let main = dir.path().join("main.xml");
    std::fs::write(
        &main,
        format!(
            r#"<mujoco>{compilers}<asset><mesh name="t" file="tet.obj"/></asset>
            <worldbody><body><freejoint/><geom type="mesh" mesh="t"/></body></worldbody></mujoco>"#
        ),
    )
    .unwrap();
    sim_mjcf::load_model_from_file(&main)
}

#[test]
fn a_later_compilers_assetdir_lies_over_an_earlier_meshdir() {
    // MuJoCo writes assetdir into meshdir and texturedir as it reads it, then
    // reads meshdir over it (`xml_native_reader.cc`, Compiler). Measured:
    // meshdir a, then assetdir b, finds the mesh in b (body mass
    // 166.66666666666666); assetdir b then meshdir a, in two elements or in
    // one, looks in a ("Error opening file 'a/tet.obj'"). Before: a later
    // assetdir lost to an earlier meshdir.
    let m = tetrahedron_found_in_b(r#"<compiler meshdir="a"/><compiler assetdir="b"/>"#)
        .unwrap_or_else(|e| panic!("refused: {e}"));
    assert!(
        (m.body_mass[1] - 166.666_666_666_666_66).abs() < 1e-9,
        "{}",
        m.body_mass[1]
    );
    assert!(tetrahedron_found_in_b(r#"<compiler assetdir="b"/><compiler meshdir="a"/>"#).is_err());
    assert!(tetrahedron_found_in_b(r#"<compiler assetdir="b" meshdir="a"/>"#).is_err());
}

/// Every unnamed top-level `<default>` is MuJoCo 3.5.0's one main class
/// (`mjXReader::Default` takes `mjs_getSpecDefault` for each): each reads onto
/// it, the later over the earlier, and an empty one changes nothing. (A block
/// named `main` is not read onto it here: ledger L239.)
#[test]
fn every_top_level_default_reads_onto_the_main_class() {
    for empty in [
        r#"<mujoco><default><geom contype="2"/></default><default/><worldbody><geom size=".1"/></worldbody></mujoco>"#,
        r#"<mujoco><default><geom contype="2"/></default><default></default><worldbody><geom size=".1"/></worldbody></mujoco>"#,
    ] {
        assert_eq!(load(empty).geom_contype[0], 2);
    }
    let m = load(
        r#"<mujoco><default><geom contype="2"/></default><default><joint damping="3"/></default><worldbody><body><joint/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert_eq!(m.geom_contype[0], 2);
    assert_eq!(m.dof_damping[0], 3.0);
    let m = load(
        r#"<mujoco><default><geom contype="2" conaffinity="5"/></default><default><geom contype="4"/></default><worldbody><geom size=".1"/></worldbody></mujoco>"#,
    );
    assert_eq!(m.geom_contype[0], 4);
    assert_eq!(m.geom_conaffinity[0], 5);
}

/// MuJoCo 3.5.0 keeps one actuator default per class, which each actuator
/// element (`<position>`, `<motor>`, …) writes in place, top-level blocks in
/// order: a later block's `<motor>` resets the gain to 1 and keeps its gear
/// (measured: a `<position>`'s gain 1, bias −1, gear 5).
#[test]
fn a_later_top_level_blocks_actuator_element_writes_the_one_default() {
    let m = load(
        r#"<mujoco><default><position kp="10"/></default><default><motor gear="5"/></default><worldbody><body><joint name="j"/><geom size=".1"/></body></worldbody><actuator><position joint="j"/><motor joint="j"/></actuator></mujoco>"#,
    );
    assert_eq!(m.actuator_gainprm[0][0], 1.0);
    assert_eq!(m.actuator_biasprm[0][1], -1.0);
    assert_eq!(m.actuator_gear[0][0], 5.0);
}
