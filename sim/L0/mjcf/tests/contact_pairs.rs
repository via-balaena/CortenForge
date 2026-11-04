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
            r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 0.09"><freejoint/><geom name="ga" type="sphere" size="0.1"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="gb" condim="3"/><pair geom1="gb" geom2="ga" condim="6"/></contact></mujoco>"#,
            &[3, 6],
            &[
                ("floor", "ga", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
                ("floor", "gb", 3),
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
