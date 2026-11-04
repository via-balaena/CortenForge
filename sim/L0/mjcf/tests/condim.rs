//! A geom's and a pair's condim checked as MuJoCo 3.5.0's compiler checks
//! them (book L25): a value other than 1, 3, 4 or 6 is refused ('invalid
//! condim in geom', `mjCGeom::Compile`, `user_objects.cc:3651-3654`; 'invalid
//! condim in contact pair', `mjCPair::Compile`, `:5666-5669`), whether written
//! on the element, by its class, the root class, a composite's template,
//! inside a frame or on a body `fusestatic` fuses. Each geom and pair
//! condim document was run on MuJoCo 3.5.0 as written (2026-11-04): every
//! refused one is refused there for the same element, every loaded one loads
//! there with the same condims. A pair naming no geom, which MuJoCo binds to
//! an unnamed geom, is refused here (registry `D-PAIR-NO-GEOM`).
//!
//! A flex's condim, which MuJoCo does not check, is refused outside 1 to 6
//! (ledger L234, registry `D-FLEX-CONDIM-RANGE`); MuJoCo loads each of its
//! documents (run 2026-11-04).

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
        "invalid condim in geom (geom 'g' in body 's')",
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

/// `(case, document, this crate's error)`; MuJoCo's is 'invalid condim in
/// contact pair' for the same pair in each (`mjCPair::Compile`,
/// `user_objects.cc:5666-5669`), whether the pair, its class or the root
/// class wrote it.
const PAIR_CONDIMS: &[(&str, &str, &str)] = &[
    (
        "0",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="0"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "2",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "2 through the root class",
        r#"<mujoco><default><pair condim="2"/></default><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "2 through a class",
        r#"<mujoco><default><default class="c"><pair condim="2"/></default></default><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair class="c" geom1="g" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "5",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="5"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "7",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="7"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "-1",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="-1"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "the second pair's",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor"/><pair name="p" geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair (pair 'p')",
    ),
];

#[test]
fn a_pair_s_condim_checked_as_mujoco() {
    let mut wrong = Vec::new();
    for &(case, xml, expected) in PAIR_CONDIMS {
        match load_model(xml) {
            Err(e) if e.to_string() == expected => {}
            Err(e) => wrong.push(format!("{case}: {e}")),
            Ok(_) => wrong.push(format!("{case}: loads")),
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// A pair's condims 1, 4 and 6 load as written, and a pair class no pair
/// uses is not checked: MuJoCo 3.5.0 loads this document with the same
/// three.
#[test]
fn a_pair_s_valid_condims_load_as_written() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><pair condim="2"/></default></default><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body><body name="c"><freejoint/><geom name="h" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="1"/><pair geom1="h" geom2="floor" condim="4"/><pair geom1="g" geom2="h" condim="6"/></contact></mujoco>"#,
    )
    .expect("loads");
    let condims: Vec<i32> = model.contact_pairs.iter().map(|p| p.condim).collect();
    assert_eq!(condims, [1, 4, 6]);
}

/// A pair whose `geom1` is empty or not written is refused (registry
/// `D-PAIR-NO-GEOM`), where MuJoCo 3.5.0 binds it to an unnamed geom of
/// the model (measured on these documents: the body's unnamed sphere, or
/// the geom written `name=""`); an empty one after the keyframes, so that
/// MuJoCo's own fault beside it is named.
#[test]
fn a_pair_naming_no_geom_is_refused() {
    let empty = load_model(
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body></worldbody><contact><pair geom1="" geom2="g"/></contact></mujoco>"#,
    )
    .expect_err("an empty geom name");
    assert_eq!(empty.to_string(), "unknown geom '' (pair #0)");
    let empty_geom2 = load_model(
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body></worldbody><contact><pair geom1="floor" geom2=""/></contact></mujoco>"#,
    )
    .expect_err("an empty geom2");
    assert_eq!(empty_geom2.to_string(), "unknown geom '' (pair #0)");
    let unwritten = load_model(
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body></worldbody><contact><pair geom2="g"/></contact></mujoco>"#,
    )
    .expect_err("no geom1");
    assert!(
        unwritten
            .to_string()
            .starts_with("required attribute missing: 'geom1' on <pair>"),
        "{unwritten}"
    );
    // A later body's size 0, which MuJoCo 3.5.0 names here (measured),
    // having bound the pair, is named first.
    let before_a_size = load_model(
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body><body name="c"><freejoint/><geom name="z" size="0"/></body></worldbody><contact><pair geom1="" geom2="g"/></contact></mujoco>"#,
    )
    .expect_err("an empty geom name beside a size 0");
    assert_eq!(
        before_a_size.to_string(),
        "invalid size on <geom>: size 0 must be positive (geom 'z' in body 'c')"
    );
    // A geom written `name=""` is unnamed here as in MuJoCo, which binds the
    // pair to it (measured).
    let written_empty = load_model(
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom name="" size=".2"/></body><body name="c"><freejoint/><geom name="z" size="0"/></body></worldbody><contact><pair geom1="" geom2="g"/></contact></mujoco>"#,
    )
    .expect_err("an empty geom name beside a geom named empty");
    assert_eq!(
        written_empty.to_string(),
        "invalid size on <geom>: size 0 must be positive (geom 'z' in body 'c')"
    );
    // With every geom named, MuJoCo refuses the pair too, where it compiles
    // pairs ('geom '' not found', measured): before the later size here.
    let all_named = load_model(
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body><body name="c" pos="1 0 1"><freejoint/><geom name="z" size="0"/></body></worldbody><contact><pair geom1="" geom2="ga"/></contact></mujoco>"#,
    )
    .expect_err("an empty geom name, every geom named");
    assert_eq!(all_named.to_string(), "unknown geom '' (pair #0)");
}

/// `(condim, document, this crate's error)`: a sphere on a flex sheet whose
/// contact condim is outside 1 to 6, refused (registry
/// `D-FLEX-CONDIM-RANGE`). MuJoCo 3.5.0 loads each and stops at the first
/// contact that takes the value (measured: 7 at the first step; 0 and -1 run
/// here as the sphere's 3).
const FLEX_CONDIMS: &[(&str, &str, &str)] = &[
    (
        "0",
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="0"/></flex></deformable></mujoco>"#,
        "invalid condim on <flex>: 0 is outside 1 to 6 (flex 'f')",
    ),
    (
        "7",
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="7"/></flex></deformable></mujoco>"#,
        "invalid condim on <flex>: 7 is outside 1 to 6 (flex 'f')",
    ),
    (
        "-1",
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="-1"/></flex></deformable></mujoco>"#,
        "invalid condim on <flex>: -1 is outside 1 to 6 (flex 'f')",
    ),
];

#[test]
fn a_flex_s_condim_outside_1_to_6_is_refused() {
    let mut wrong = Vec::new();
    for &(case, xml, expected) in FLEX_CONDIMS {
        match load_model(xml) {
            Err(e) if e.to_string() == expected => {}
            Err(e) => wrong.push(format!("{case}: {e}")),
            Ok(_) => wrong.push(format!("{case}: loads")),
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// The same document with a flex condim of 1 to 6 loads as written, as
/// MuJoCo 3.5.0 loads it (measured).
const FLEX_CONDIMS_LOADED: &[(i32, &str)] = &[
    (
        1,
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="1"/></flex></deformable></mujoco>"#,
    ),
    (
        2,
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="2"/></flex></deformable></mujoco>"#,
    ),
    (
        3,
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="3"/></flex></deformable></mujoco>"#,
    ),
    (
        4,
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="4"/></flex></deformable></mujoco>"#,
    ),
    (
        5,
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="5"/></flex></deformable></mujoco>"#,
    ),
    (
        6,
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="6"/></flex></deformable></mujoco>"#,
    ),
];

#[test]
fn a_flex_s_condims_1_to_6_load_as_written() {
    for &(condim, xml) in FLEX_CONDIMS_LOADED {
        let model = load_model(xml).expect("loads");
        assert_eq!(model.flex_condim, [condim], "{condim}");
    }
}
