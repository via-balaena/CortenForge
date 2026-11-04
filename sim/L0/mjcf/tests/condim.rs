//! A geom's condim checked as MuJoCo 3.5.0's compiler checks it (book L25):
//! a value other than 1, 3, 4 or 6 is refused ('invalid condim in geom',
//! `mjCGeom::Compile`, `user_objects.cc:3651-3654`), whether written on the
//! geom, by its class, the root class, a composite's template, inside a
//! frame or on a body `fusestatic` fuses. Each document was run on MuJoCo
//! 3.5.0 as written (2026-11-04):
//! every refused one is refused there for the same geom, every loaded one
//! loads there with the same condims.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::load_model;

/// `(case, document, this crate's error)`; MuJoCo's is 'invalid condim in
/// geom' for the same geom in each.
const GEOM_CONDIMS: &[(&str, &str, &str)] = &[
    (
        "0",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="0"/></body></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "2 through a class",
        r#"<mujoco><default><default class="c"><geom condim="2"/></default></default><worldbody><body name="b"><freejoint/><geom name="g" class="c" size=".1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "5",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="5"/></body></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "7",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="7"/></body></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "-1",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="-1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "the root class's: the world's geom first",
        r#"<mujoco><default><geom condim="2"/></default><worldbody><body name="b"><freejoint/><geom name="g" size=".1"/></body><geom name="floor" type="plane" size="1 1 .1"/></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'floor' in body 'world')",
    ),
    (
        "a cable's template",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><composite type="cable" prefix="c" count="4 1 1" curve="s 0 0" size="1"><geom type="capsule" size=".02" condim="2"/></composite></body></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'cG0' in body 'cB_first')",
    ),
    (
        "on a body fusestatic fuses",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s"><geom name="g" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom (geom 'g' in body 'world')",
    ),
];

#[test]
fn condim_checked_as_mujoco() {
    let mut wrong = Vec::new();
    for &(case, xml, expected) in GEOM_CONDIMS {
        match load_model(xml) {
            Err(e) if e.to_string() == expected => {}
            Err(e) => wrong.push(format!("{case}: {e}")),
            Ok(_) => wrong.push(format!("{case}: loads")),
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// Condims 1, 3, 4 and 6 load as written, an unwritten one as 3, and a class
/// no geom uses is not checked: MuJoCo 3.5.0 loads this document with the
/// same five condims.
#[test]
fn a_geom_s_valid_condims_load_as_written() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><geom condim="2"/></default></default><worldbody><body name="b"><freejoint/><geom size=".1" condim="1"/><geom size=".1" condim="3"/><geom size=".1" condim="4"/><geom size=".1" condim="6"/><geom size=".1"/></body></worldbody></mujoco>"#,
    )
    .expect("loads");
    assert_eq!(model.geom_condim, [1, 3, 4, 6, 3]);
}
