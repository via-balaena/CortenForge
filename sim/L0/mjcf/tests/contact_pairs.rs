//! A `<pair>`'s values as MuJoCo 3.5.0's compiler gives them: a value the
//! pair or its class writes lies over MuJoCo's pair default
//! (`mjs_defaultPair`, `user_init.c:288-297`: condim 3, friction 1 1 0.005
//! 0.0001 0.0001, solref 0.02 1, solimp 0.9 0.95 0.001 0.5 2, margin, gap and
//! solreffriction 0), and an unwritten one is that default whatever its
//! geoms write (ledger L147, L241). Each document was run on MuJoCo 3.5.0 as
//! written (2026-11-04) and these are its values.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::load_model;

/// The one pair's values: condim, friction, solref, solreffriction, solimp,
/// margin, gap.
type Values = (i32, [f64; 5], [f64; 2], [f64; 2], [f64; 5], f64, f64);

const DEFAULTS: Values = (
    3,
    [1.0, 1.0, 0.005, 0.0001, 0.0001],
    [0.02, 1.0],
    [0.0, 0.0],
    [0.9, 0.95, 0.001, 0.5, 2.0],
    0.0,
    0.0,
);

fn pair_values(xml: &str) -> Values {
    let model = load_model(xml).expect("loads");
    assert_eq!(model.contact_pairs.len(), 1);
    let p = &model.contact_pairs[0];
    (
        p.condim,
        p.friction,
        p.solref,
        p.solreffriction,
        p.solimp,
        p.margin,
        p.gap,
    )
}

#[test]
fn a_pair_s_unwritten_values_are_mujoco_s_defaults() {
    let cases = [
        (
            "two geoms of condim 1",
            r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="ga" size=".1" condim="1"/></body><body name="b" pos="1 0 0"><freejoint/><geom name="gb" size=".1" condim="1"/></body></worldbody><contact><pair geom1="ga" geom2="gb" /></contact></mujoco>"#,
        ),
        (
            "two geoms of condim 6",
            r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="ga" size=".1" condim="6"/></body><body name="b" pos="1 0 0"><freejoint/><geom name="gb" size=".1" condim="6"/></body></worldbody><contact><pair geom1="ga" geom2="gb" /></contact></mujoco>"#,
        ),
        (
            "a geom of condim 1 and higher priority",
            r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="ga" size=".1" condim="1" priority="2"/></body><body name="b" pos="1 0 0"><freejoint/><geom name="gb" size=".1" condim="6"/></body></worldbody><contact><pair geom1="ga" geom2="gb" /></contact></mujoco>"#,
        ),
        (
            "geoms writing every value",
            r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="ga" size=".1" friction="0.3 0.02 0.003" solref="0.05 0.7" solimp="0.8 0.95 0.01 0.4 3" margin="0.1" gap="0.05" solmix="2"/></body><body name="b" pos="1 0 0"><freejoint/><geom name="gb" size=".1" friction="0.9 0.004 0.0005" solref="0.03 1.2" solimp="0.7 0.9 0.002 0.6 2" margin="0.02" gap="0.01"/></body></worldbody><contact><pair geom1="ga" geom2="gb" /></contact></mujoco>"#,
        ),
        (
            "geoms writing every value, one of higher priority",
            r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="ga" size=".1" friction="0.3 0.02 0.003" solref="0.05 0.7" solimp="0.8 0.95 0.01 0.4 3" margin="0.1" gap="0.05" solmix="2" priority="1"/></body><body name="b" pos="1 0 0"><freejoint/><geom name="gb" size=".1" friction="0.9 0.004 0.0005" solref="0.03 1.2" solimp="0.7 0.9 0.002 0.6 2" margin="0.02" gap="0.01"/></body></worldbody><contact><pair geom1="ga" geom2="gb" /></contact></mujoco>"#,
        ),
    ];
    for (case, xml) in cases {
        assert_eq!(pair_values(xml), DEFAULTS, "{case}");
    }
}

/// A class's values lie over the defaults, and the pair's over its class's.
#[test]
fn a_pair_s_class_values_lie_over_the_defaults() {
    let partial = pair_values(
        r#"<mujoco><default><default class="c"><pair friction="0.6 0.5" solimp="0.85"/></default></default><worldbody><body name="a"><freejoint/><geom name="ga" size=".1" friction="0.3 0.02 0.003" solref="0.05 0.7" solimp="0.8 0.95 0.01 0.4 3" margin="0.1" gap="0.05" solmix="2"/></body><body name="b" pos="1 0 0"><freejoint/><geom name="gb" size=".1" friction="0.9 0.004 0.0005" solref="0.03 1.2" solimp="0.7 0.9 0.002 0.6 2" margin="0.02" gap="0.01"/></body></worldbody><contact><pair geom1="ga" geom2="gb" class="c" friction="0.2"/></contact></mujoco>"#,
    );
    assert_eq!(
        partial,
        (
            3,
            [0.2, 0.5, 0.005, 0.0001, 0.0001],
            [0.02, 1.0],
            [0.0, 0.0],
            [0.85, 0.95, 0.001, 0.5, 2.0],
            0.0,
            0.0
        )
    );
    let root = pair_values(
        r#"<mujoco><default><pair condim="4" friction="0.6 0.5 0.004 0.0003 0.0002" solref="0.06 0.9" solimp="0.85 0.9 0.003 0.4 2" margin="0.03" gap="0.01" solreffriction="0.07 0.8"/></default><worldbody><body name="a"><freejoint/><geom name="ga" size=".1" friction="0.3 0.02 0.003" solref="0.05 0.7" solimp="0.8 0.95 0.01 0.4 3" margin="0.1" gap="0.05" solmix="2"/></body><body name="b" pos="1 0 0"><freejoint/><geom name="gb" size=".1" friction="0.9 0.004 0.0005" solref="0.03 1.2" solimp="0.7 0.9 0.002 0.6 2" margin="0.02" gap="0.01"/></body></worldbody><contact><pair geom1="ga" geom2="gb" /></contact></mujoco>"#,
    );
    assert_eq!(
        root,
        (
            4,
            [0.6, 0.5, 0.004, 0.0003, 0.0002],
            [0.06, 0.9],
            [0.07, 0.8],
            [0.85, 0.9, 0.003, 0.4, 2.0],
            0.03,
            0.01
        )
    );
}

/// Each `<pair>` is kept, one on the geoms of an earlier pair too, in either
/// geom order or another `<contact>` block (book L26, ledger L348), and
/// each makes its own contacts: MuJoCo 3.5.0's pairs and, after one forward
/// pass, its contacts, `(geom1, geom2, dim)` in its order (measured).
#[test]
fn repeated_pairs_are_each_kept_and_each_collides() {
    let cases: [(&str, &str, &[i32], &[(&str, &str, usize)]); 3] = [
        (
            "two, in reversed geom order",
            r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 0.09"><freejoint/><geom name="ga" type="sphere" size="0.1"/></body><body name="b" pos="0.19 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="gb" condim="3"/><pair geom1="gb" geom2="ga" condim="6"/></contact></mujoco>"#,
            &[3, 6],
            &[
                ("floor", "ga", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("ga", "gb", 3),
                ("ga", "gb", 6),
            ],
        ),
        (
            "three on the floor",
            r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 0.09"><freejoint/><geom name="ga" type="sphere" size="0.1"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="floor" condim="1"/><pair geom1="ga" geom2="floor" condim="4"/><pair geom1="floor" geom2="ga" condim="6"/></contact></mujoco>"#,
            &[1, 4, 6],
            &[
                ("floor", "ga", 1),
                ("floor", "ga", 4),
                ("floor", "ga", 6),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
            ],
        ),
        (
            "across blocks, an empty one between",
            r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 0.09"><freejoint/><geom name="ga" type="sphere" size="0.1"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="floor" condim="3"/></contact><contact/><contact><pair geom1="ga" geom2="floor" condim="6"/></contact></mujoco>"#,
            &[3, 6],
            &[
                ("floor", "ga", 3),
                ("floor", "ga", 6),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
            ],
        ),
    ];
    for (case, xml, condims, contacts) in cases {
        let model = load_model(xml).expect(case);
        let got: Vec<i32> = model.contact_pairs.iter().map(|p| p.condim).collect();
        assert_eq!(got, condims, "{case}: pairs");
        let mut data = model.make_data();
        data.forward(&model).expect(case);
        let name = |g: Option<usize>| model.geom_name[g.expect("a geom")].clone().expect("named");
        let got: Vec<(String, String, usize)> = data
            .contacts
            .iter()
            .map(|c| (name(c.geom[0]), name(c.geom[1]), c.dim))
            .collect();
        let want: Vec<(String, String, usize)> = contacts
            .iter()
            .map(|&(a, b, d)| (a.to_string(), b.to_string(), d))
            .collect();
        assert_eq!(got, want, "{case}: contacts");
    }
}

/// A pair names the lower body's geom first, and the pairs are stored by
/// their bodies, stably, as MuJoCo 3.5.0 compiles them
/// (`mjCPair::ResolveReferences`, `user_objects.cc:5645-5657`;
/// `user_model.cc:4969-4972`): MuJoCo's pairs for this document (measured),
/// where two pairs on one body and the world keep the order written.
#[test]
fn pairs_are_stored_in_mujoco_s_order() {
    let model = load_model(
        r#"<mujoco><option gravity="0 0 0"/><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 0.3"><freejoint/><geom name="ga" type="sphere" size="0.1"/><geom name="ga2" type="sphere" size="0.05" pos="0 0 0.1"/></body><body name="b" pos="0.15 0.05 0.3"><freejoint/><geom name="gb" type="sphere" size="0.1"/></body><body name="c" pos="0.3 -0.02 0.09"><freejoint/><geom name="gc" type="sphere" size="0.1"/></body></worldbody><contact><pair geom1="gb" geom2="ga" condim="1"/><pair geom1="gc" geom2="floor" condim="4"/><pair geom1="ga2" geom2="floor" condim="6"/><pair geom1="gc" geom2="gb" condim="6"/><pair geom1="ga" geom2="floor" condim="3"/></contact></mujoco>"#,
    )
    .expect("loads");
    let name = |g: usize| model.geom_name[g].clone().expect("named");
    let got: Vec<(String, String, i32)> = model
        .contact_pairs
        .iter()
        .map(|p| (name(p.geom1), name(p.geom2), p.condim))
        .collect();
    let want: Vec<(String, String, i32)> = [
        ("floor", "ga2", 6),
        ("floor", "ga", 3),
        ("floor", "gc", 4),
        ("ga", "gb", 1),
        ("gb", "gc", 6),
    ]
    .iter()
    .map(|&(a, b, c)| (a.to_string(), b.to_string(), c))
    .collect();
    assert_eq!(got, want);
}

/// Under `fusestatic`, a pair holds the geoms it names (registry
/// `D-FUSESTATIC-PAIR-GEOM`). MuJoCo 3.5.0 binds it through its geom names'
/// indices from before the fusion, which moving a fused body's geoms
/// renumbers: for this document it holds `gs0` and the floor, both the
/// world's, and stops at the first step ('contact 0 is between two static
/// bodies', measured).
#[test]
fn a_fused_model_s_pair_holds_the_geoms_it_names() {
    let model = load_model(
        r#"<mujoco><compiler fusestatic="true"/><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="m1" pos="0 0 .09"><freejoint/><geom name="g1" size=".1"/></body><body name="s0" pos="2 0 0"><geom name="gs0" size=".1"/></body></worldbody><contact><pair geom1="g1" geom2="floor" condim="1"/></contact></mujoco>"#,
    )
    .expect("loads");
    let name = |g: usize| model.geom_name[g].clone().expect("named");
    let got: Vec<(String, String)> = model
        .contact_pairs
        .iter()
        .map(|p| (name(p.geom1), name(p.geom2)))
        .collect();
    assert_eq!(got, [("floor".to_string(), "g1".to_string())]);
    let mut data = model.make_data();
    data.step(&model).expect("steps");
}

/// Two geoms of one body, or two of the world, are kept in the order a pair
/// writes them, MuJoCo 3.5.0 swapping only where the first geom's body is the
/// higher; and 30 pairs on three bodies' geoms and the floor, written in
/// turn, are stored body by body, each body's in the order written, as
/// MuJoCo's stable sort keeps them (measured on both documents).
#[test]
fn pairs_of_one_body_and_many_interleaved_are_stored_as_mujoco_stores_them() {
    let model = load_model(
        r#"<mujoco><worldbody><geom name="w1" type="plane" size="1 1 .1"/><geom name="w2" type="box" size=".1 .1 .1" pos="3 0 0"/><body name="b" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/><geom name="gb" size=".1" pos=".5 0 0"/></body></worldbody><contact><pair geom1="gb" geom2="ga"/><pair geom1="w2" geom2="w1"/><pair geom1="ga" geom2="w1"/></contact></mujoco>"#,
    )
    .expect("loads");
    let name = |g: usize| model.geom_name[g].clone().expect("named");
    let got: Vec<(String, String)> = model
        .contact_pairs
        .iter()
        .map(|p| (name(p.geom1), name(p.geom2)))
        .collect();
    let want: Vec<(String, String)> = [("w2", "w1"), ("w1", "ga"), ("gb", "ga")]
        .iter()
        .map(|&(a, b)| (a.to_string(), b.to_string()))
        .collect();
    assert_eq!(got, want);
    let model = load_model(
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body><body name="b" pos="1 0 1"><freejoint/><geom name="gb" size=".1"/></body><body name="c" pos="2 0 1"><freejoint/><geom name="gc" size=".1"/></body></worldbody><contact><pair geom1="ga" geom2="floor" margin="0.001"/><pair geom1="gb" geom2="floor" margin="0.002"/><pair geom1="gc" geom2="floor" margin="0.003"/><pair geom1="ga" geom2="floor" margin="0.004"/><pair geom1="gb" geom2="floor" margin="0.005"/><pair geom1="gc" geom2="floor" margin="0.006"/><pair geom1="ga" geom2="floor" margin="0.007"/><pair geom1="gb" geom2="floor" margin="0.008"/><pair geom1="gc" geom2="floor" margin="0.009"/><pair geom1="ga" geom2="floor" margin="0.010"/><pair geom1="gb" geom2="floor" margin="0.011"/><pair geom1="gc" geom2="floor" margin="0.012"/><pair geom1="ga" geom2="floor" margin="0.013"/><pair geom1="gb" geom2="floor" margin="0.014"/><pair geom1="gc" geom2="floor" margin="0.015"/><pair geom1="ga" geom2="floor" margin="0.016"/><pair geom1="gb" geom2="floor" margin="0.017"/><pair geom1="gc" geom2="floor" margin="0.018"/><pair geom1="ga" geom2="floor" margin="0.019"/><pair geom1="gb" geom2="floor" margin="0.020"/><pair geom1="gc" geom2="floor" margin="0.021"/><pair geom1="ga" geom2="floor" margin="0.022"/><pair geom1="gb" geom2="floor" margin="0.023"/><pair geom1="gc" geom2="floor" margin="0.024"/><pair geom1="ga" geom2="floor" margin="0.025"/><pair geom1="gb" geom2="floor" margin="0.026"/><pair geom1="gc" geom2="floor" margin="0.027"/><pair geom1="ga" geom2="floor" margin="0.028"/><pair geom1="gb" geom2="floor" margin="0.029"/><pair geom1="gc" geom2="floor" margin="0.030"/></contact></mujoco>"#,
    )
    .expect("loads");
    let name = |g: usize| model.geom_name[g].clone().expect("named");
    let got: Vec<(String, String, f64)> = model
        .contact_pairs
        .iter()
        .map(|p| (name(p.geom1), name(p.geom2), p.margin))
        .collect();
    let margins = [
        0.001, 0.004, 0.007, 0.01, 0.013, 0.016, 0.019, 0.022, 0.025, 0.028, 0.002, 0.005, 0.008,
        0.011, 0.014, 0.017, 0.02, 0.023, 0.026, 0.029, 0.003, 0.006, 0.009, 0.012, 0.015, 0.018,
        0.021, 0.024, 0.027, 0.03,
    ];
    let want: Vec<(String, String, f64)> = margins
        .iter()
        .enumerate()
        .map(|(k, &m)| {
            (
                "floor".to_string(),
                ["ga", "gb", "gc"][k / 10].to_string(),
                m,
            )
        })
        .collect();
    assert_eq!(got, want);
}

/// Two concentric spheres leave the contact normal to the collision
/// function, which takes the geoms in the pair's order. Under a pair naming
/// the higher body's sphere first, MuJoCo 3.5.0's normal is +x, and after 50
/// steps the first body is at x = -0.17956204793888816 and the second at
/// x = 0.17956204793888816 (measured).
#[test]
fn a_pair_naming_the_higher_body_first_collides_as_mujoco() {
    let model = load_model(
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="g1" type="sphere" size="0.1"/></body><body name="b"><freejoint/><geom name="g2" type="sphere" size="0.1"/></body></worldbody><contact><pair geom1="g2" geom2="g1"/></contact></mujoco>"#,
    )
    .expect("loads");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_eq!(data.contacts.len(), 1);
    let normal = data.contacts[0].normal;
    assert!(
        (normal.x - 1.0).abs() < 1e-12 && normal.y.abs() < 1e-12 && normal.z.abs() < 1e-12,
        "{normal:?}"
    );
    for _ in 0..50 {
        data.step(&model).expect("step");
    }
    let x = 0.179_562_047_938_888_16;
    assert!(
        (data.qpos[0] + x).abs() < 1e-9 && (data.qpos[7] - x).abs() < 1e-9,
        "{} {}",
        data.qpos[0],
        data.qpos[7]
    );
}
