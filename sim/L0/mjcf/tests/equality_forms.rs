//! Equalities as MuJoCo 3.5.0 builds and steps them (`engine_core_constraint.c`
//! `mj_instantiateEquality`): a joint equality's joints measured from their
//! `qpos0`, in MuJoCo's arithmetic (ledger L446), and one on a ball or free
//! joint refused (ledger L465); a connect's site form, its sites kept as MuJoCo
//! keeps them (ledger L445); a weld's site form, and the forms of a connect and
//! a weld MuJoCo refuses (ledger L445, L315); an attribute another equality
//! element takes, refused as MuJoCo's schema refuses it, and the scalar-joint
//! refusal after the faults the crate named before it.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-25). Where a test expects otherwise, it names the
//! registry row that says why.

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

use sim_core::{ConstraintType, EqualityType, MjObjectType};
use sim_mjcf::load_model;

/// What MuJoCo 3.5.0 makes of a document: its equalities' types, objects,
/// object types and data in its numbering (`eq_type`, `eq_obj1id`,
/// `eq_obj2id`, `eq_objtype`, `eq_data`), after `mj_forward` the constraint
/// force, the equality rows' errors, the islands and the sensors
/// (`qfrc_constraint`, `efc_pos`, `nisland`, `efc_island`, `sensordata`),
/// and the position and velocity after ten steps.
struct Want {
    eq_type: &'static [i64],
    eq_obj1id: &'static [i64],
    eq_obj2id: &'static [i64],
    eq_objtype: &'static [i64],
    eq_data: &'static [f64],
    qfrc_constraint: &'static [f64],
    efc_eq_pos: &'static [f64],
    nisland: i64,
    efc_island: &'static [i64],
    sensordata: &'static [f64],
    qpos: &'static [f64],
    qvel: &'static [f64],
}

/// What a document should do.
enum Expect {
    /// Load as MuJoCo 3.5.0's model, and step as it does.
    Loads(Want),
    /// Be refused, as MuJoCo 3.5.0 refuses it, in words holding these: MuJoCo's,
    /// or this crate's own for the same fault; or, in a document with two
    /// faults, for the other one, which this crate names first (ledger L353).
    Refused(&'static str),
}

const CASES: &[(&str, &str, Expect)] = &[
    // a coupled pair, the follower with a ref
    (
        "j01_ref_follower",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" ref="0.3"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[0],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[0.00015332705691739292, -0.00015332705691739292],
            efc_eq_pos: &[0.0],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.015511572143334096, 0.010275904491612024],
            qvel: &[0.9341434388863462, 0.9341641467649968],
        }),
    ),
    // a coupled pair, the leader with a ref
    (
        "j02_ref_leader",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0" ref="0.3"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[0],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[0.00015332705691739292, -0.00015332705691739292],
            efc_eq_pos: &[0.0],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.010275584387351107, 0.015511892247595013],
            qvel: &[0.9341434388863463, 0.9341641467649968],
        }),
    ),
    // both with a ref, a quadratic coupling
    (
        "j03_ref_both_quadratic",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" ref="0.2"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0" ref="-0.4"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0.1 0.5 0.2 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[0],
            eq_data: &[0.1, 0.5, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[2.2952733237137926, -1.1476366618568963],
            efc_eq_pos: &[-0.1],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.03321529868176048, -0.006310259479674224],
            qvel: &[2.0650025219038737, 0.375752516971259],
        }),
    ),
    // one joint with a ref, held at it plus data[0]
    (
        "j04_single_ref",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" ref="0.2"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" polycoef="0.1 0 0 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[-1],
            eq_objtype: &[0],
            eq_data: &[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[2.6601890278008793, 0.0],
            efc_eq_pos: &[-0.1],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.03333394674142943, 0.010278608035144864],
            qvel: &[1.8882056851211717, 0.9344140904414143],
        }),
    ),
    // no ref: MuJoCo's arithmetic for the quadratic
    (
        "j05_no_ref",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0.1 0.5 0.2 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[0],
            eq_data: &[0.1, 0.5, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[2.2952733237137926, -1.1476366618568963],
            efc_eq_pos: &[-0.1],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.029724640177771825, 0.0006710575283030919],
            qvel: &[2.0650025219038737, 0.375752516971259],
        }),
    ),
    // two slides with a ref
    (
        "j06_slide_ref",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="slide" axis="0 1 0" ref="0.05"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="slide" axis="0 1 0" ref="-0.02"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0 2 0 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[0],
            eq_data: &[0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[0.0, 0.0],
            efc_eq_pos: &[0.0],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.05, -0.02],
            qvel: &[0.0, 0.0],
        }),
    ),
    // a quartic coupling, both with a ref
    (
        "j10_quartic_refs",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" ref="0.1"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0" ref="0.25"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0 1 0.1 0.05 0.02"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[0],
            eq_data: &[0.0, 1.0, 0.1, 0.05, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[0.00015332705691739292, -0.00015332705691739292],
            efc_eq_pos: &[0.0],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.012023707110485879, 0.014636464198359914],
            qvel: &[0.9346402515852694, 0.9336725427965507],
        }),
    ),
    // one slide with a ref
    (
        "j11_single_ref_slide",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="slide" axis="0 1 0" ref="0.05"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[-1],
            eq_objtype: &[0],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[0.0, 0.0],
            efc_eq_pos: &[0.0],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.05, 0.010278608035144864],
            qvel: &[0.0, 0.9344140904414143],
        }),
    ),
    // a site connect: a world site and a free body's
    (
        "c01_connect_sites_world_free",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><connect site1="w" site2="s"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                -500.0000000000008,
                0.0,
                2.491588688946001,
                0.0,
                0.2491588688946001,
                0.0,
            ],
            efc_eq_pos: &[-0.19999999999999998, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.2408590882165985,
                0.0,
                0.9984370807566013,
                0.9999701069773799,
                0.0,
                0.007732085853616833,
                0.0,
            ],
            qvel: &[
                -3.742642148938513,
                0.0,
                -0.14184647580355914,
                0.0,
                1.4083459979396975,
                0.0,
            ],
        }),
    ),
    // turned sites: a connect reads their positions only
    (
        "c02_connect_sites_rotated",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1" quat="0.9238795 0.3826834 0 0"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0.02 0" quat="0.8660254 0 0.5 0"/></body></worldbody><equality><connect site1="w" site2="s"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                -479.7104888467015,
                51.44755576649281,
                2.2546333402852454,
                0.04509266680570491,
                0.22546333402852456,
                4.449454200284748,
            ],
            efc_eq_pos: &[-0.19999999999999998, -0.02, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.24281532913557033,
                0.005773202441763079,
                0.9983828031722428,
                0.9981979972363846,
                0.004740174870229698,
                0.007176438537186543,
                0.05938676439571947,
            ],
            qvel: &[
                -3.645985898694489,
                0.3598299522632523,
                -0.14675362666031236,
                0.8522146938425353,
                1.3140297942407195,
                7.29407540478378,
            ],
        }),
    ),
    // a site on a body whose inertial frame is not its own
    (
        "c03_connect_site_inertial_offset",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" pos="0.05 0.01 0" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><connect site1="w" site2="s"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                -498.1471850214545,
                -27.792224678195296,
                1.2829458684126944,
                0.0,
                0.12829458684126946,
                2.7792224678195296,
            ],
            efc_eq_pos: &[-0.19999999999999998, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.2408450244761825,
                -0.0021048881586921545,
                0.9987926307976674,
                0.9999217934424246,
                -0.0013923228352826526,
                0.006000484113114274,
                -0.010884053767577244,
            ],
            qvel: &[
                -3.743813064758247,
                -0.13282135099519124,
                -0.1095819972216957,
                -0.25191826396436734,
                1.0922142291338088,
                -1.3374373092784182,
            ],
        }),
    ),
    // two free bodies' sites
    (
        "c04_connect_two_moving",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s2" pos="-0.1 0 0" quat="0.9238795 0.3826834 0 0"/></body></worldbody><equality><connect site1="s1" site2="s2"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                125.0000000000002,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -125.0000000000002,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            efc_eq_pos: &[-0.09999999999999998, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.014783589295672072,
                0.0,
                0.9978418000000002,
                1.0,
                0.0,
                0.0,
                0.0,
                0.28521641070432796,
                0.0,
                0.9978418000000002,
                1.0,
                0.0,
                0.0,
                0.0,
            ],
            qvel: &[
                0.9353691395801967,
                0.0,
                -0.1962,
                0.0,
                0.0,
                0.0,
                -0.9353691395801971,
                0.0,
                -0.1962,
                0.0,
                0.0,
                0.0,
            ],
        }),
    ),
    // a site placed by a <frame>
    (
        "c06_connect_site_in_frame",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><frame pos="0 0.02 0" quat="0.8660254 0 0.5 0"><site name="s" pos="-0.1 0 0"/></frame></body></worldbody><equality><connect site1="w" site2="s"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                -207.08278986123358,
                3.8141511643628654,
                21.66991488047865,
                0.10308311666662817,
                -16.8503999729011,
                3.950948240256585,
            ],
            efc_eq_pos: &[-0.25000000032774194, -0.02, -0.08660254056766581],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.26952832625652445,
                0.002102338354613113,
                1.007006061042496,
                0.9615311624424026,
                0.009564259651871779,
                -0.2692742943990931,
                0.05345748745820832,
            ],
            qvel: &[
                -2.3734945566206918,
                0.22971237305573156,
                0.7735737484907764,
                1.6547600169582073,
                -32.96424272221456,
                6.599965583066249,
            ],
        }),
    ),
    // a site at the end of a hinge chain, to a world site
    (
        "c10_connect_site_hinge_chain",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0.4 0 1"/><body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><body name="b" pos="0.2 0 0"><joint type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><site name="s" pos="0.2 0 0"/></body></body></worldbody><equality><connect site1="s" site2="w"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[1],
            eq_obj2id: &[0],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[-0.3381050492127906, -0.1690525246063953],
            efc_eq_pos: &[0.0, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[0.01487462031433574, -0.029694703234460912],
            qvel: &[1.3369844758748635, -2.670325391069699],
        }),
    ),
    // two sleeping bodies held by sites beside an awake one: no rows, the equality asleep through its sites' bodies
    (
        "c11_connect_sites_asleep",
        r#"<mujoco model="m"><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="c" pos="2 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="x"/></body><body name="a" pos="0 0 1" sleep="init"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="sa" pos="0.1 0 0"/></body><body name="b" pos="0.3 0 1" sleep="init"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="sb" pos="-0.1 0 0"/></body></worldbody><equality><connect site1="sa" site2="sb"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[1],
            eq_obj2id: &[2],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0,
            ],
            efc_eq_pos: &[],
            nisland: 0,
            efc_island: &[],
            sensordata: &[],
            qpos: &[
                2.0,
                0.0,
                0.9978418000000002,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.3,
                0.0,
                1.0,
                1.0,
                0.0,
                0.0,
                0.0,
            ],
            qvel: &[
                0.0, 0.0, -0.1962, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0,
            ],
        }),
    ),
    // sensors on a body a site connect holds: the connect's force on its site's body
    (
        "c12_connect_sites_sensors",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/><site name="m" pos="0.05 0.02 0"/></body></worldbody><equality><connect site1="w" site2="s"/></equality><sensor><accelerometer site="m"/><force site="m"/><torque site="m"/><framelinacc objtype="site" objname="m"/></sensor></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                -500.0000000000008,
                0.0,
                2.491588688946001,
                0.0,
                0.2491588688946001,
                0.0,
            ],
            efc_eq_pos: &[-0.19999999999999998, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[
                -499.9999999999999,
                0.0,
                -1.1020488431876232,
                9.094947017729282e-13,
                0.0,
                3.907985046680551e-14,
                -7.815970093361102e-16,
                2.59237076249974e-15,
                1.8189894035458565e-14,
                -499.9999999999999,
                0.0,
                -1.1020488431876232,
            ],
            qpos: &[
                0.2408590882165985,
                0.0,
                0.9984370807566013,
                0.9999701069773799,
                0.0,
                0.007732085853616833,
                0.0,
            ],
            qvel: &[
                -3.742642148938513,
                0.0,
                -0.14184647580355914,
                0.0,
                1.4083459979396975,
                0.0,
            ],
        }),
    ),
    // a site weld: a world site and a free body's
    (
        "w01_weld_sites_world_free",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="w" site2="s"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                -500.0000000000008,
                0.0,
                5.310491088136937,
                0.0,
                0.1462936585381578,
                0.0,
            ],
            efc_eq_pos: &[-0.19999999999999998, 0.0, 0.0, 0.0, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.24086433329388923,
                0.0,
                0.9992622022818202,
                0.9999939234902786,
                0.0,
                0.00348611280926635,
                0.0,
            ],
            qvel: &[
                -3.741682865047363,
                0.0,
                -0.057301909761929795,
                0.0,
                0.5438694672684516,
                0.0,
            ],
        }),
    ),
    // turned sites: the weld holds each site's turn
    (
        "w02_weld_sites_rotated",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1" quat="0.9238795 0.3826834 0 0"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0.02 0" quat="0.8660254 0 0.5 0"/></body></worldbody><equality><weld site1="w" site2="s"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                -495.20852633706403,
                -26.042631685328903,
                81.63958957659659,
                1.6771445252725323,
                -3.841966335404378,
                1.050761768187229,
            ],
            efc_eq_pos: &[
                -0.19999999999999998,
                -0.02,
                0.0,
                0.33141355970379927,
                -0.4619397711104239,
                0.1913417087442235,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.24003095813559264,
                -0.007766289044949564,
                1.0125986546044345,
                0.9903017838101918,
                0.10804800494256668,
                -0.08714712428077218,
                -0.005777918299421512,
            ],
            qvel: &[
                -3.890777164470035,
                -0.7434654399664773,
                0.9995546618884475,
                13.943884094494754,
                -13.569244291160217,
                -2.438859578207057,
            ],
        }),
    ),
    // a site weld's torquescale
    (
        "w03_weld_sites_torquescale",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1" quat="0.9238795 0.3826834 0 0"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0" quat="0.8660254 0 0.5 0"/></body></worldbody><equality><weld site1="w" site2="s" torquescale="0.3"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3],
            qfrc_constraint: &[
                -500.0000000000008,
                -4.603668001510291,
                14.576487496072142,
                0.6576910893839905,
                -0.19183392968122703,
                -0.20191526322413506,
            ],
            efc_eq_pos: &[
                -0.19999999999999998,
                0.0,
                0.0,
                0.09942406791113978,
                -0.13858193133312716,
                0.057402512623267045,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.24085922155699085,
                -0.0010054005998153296,
                1.0010416763953038,
                0.9983777128231218,
                0.05642143014312139,
                -0.005808988765556304,
                -0.004982008450257632,
            ],
            qvel: &[
                -3.74260783735667,
                -0.09266832444754229,
                0.09474060080619802,
                8.785383229831158,
                -1.0628527502031764,
                -0.8757708321327664,
            ],
        }),
    ),
    // two free bodies' turned sites
    (
        "w04_weld_two_moving",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s1" pos="0.1 0 0" quat="0.9238795 0.3826834 0 0"/></body><body name="b" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s2" pos="-0.1 0 0" quat="0.8660254 0 0.5 0"/></body></worldbody><equality><weld site1="s1" site2="s2"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                125.0000000000002,
                6.750155989720945e-14,
                1.350031197944189e-13,
                -0.8459262108380257,
                3.388423351632337,
                1.598043549710002,
                -125.0000000000002,
                -6.750155989720945e-14,
                -1.350031197944189e-13,
                0.8459262108380257,
                -3.388423351632364,
                -1.5980435497099887,
            ],
            efc_eq_pos: &[
                -0.09999999999999998,
                0.0,
                0.0,
                0.33141355970379927,
                -0.4619397711104239,
                0.1913417087442235,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.01533768817362632,
                4.023373846307846e-07,
                0.9978194859474621,
                0.9958200520383869,
                -0.05583331695433597,
                0.06623217561599427,
                0.028954508963435835,
                0.2846623118263737,
                -4.023373846309534e-07,
                0.9978641140525378,
                0.9959366504360663,
                0.053417510958810276,
                -0.06807200017551783,
                -0.024959179338651036,
            ],
            qvel: &[
                1.0143167147816823,
                0.00100568160735739,
                -0.19880632975350349,
                -7.434610396333362,
                8.957013902590317,
                4.306634935738879,
                -1.0143167147816825,
                -0.0010056816073574005,
                -0.1935936702464965,
                6.783858192926638,
                -9.484845448728848,
                -3.1618509248311795,
            ],
        }),
    ),
    // a site weld's solref
    (
        "w10_weld_sites_solref",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1" quat="0.9238795 0.3826834 0 0"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="w" site2="s" solref="0.05 1"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                -80.00000000000031,
                0.0,
                5.310491088136934,
                0.2999286633519893,
                0.1462936585381576,
                2.7755575615628914e-17,
            ],
            efc_eq_pos: &[
                -0.19999999999999998,
                0.0,
                0.0,
                0.38268341623423263,
                0.0,
                0.0,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.28621248719048314,
                1.2168937096570103e-06,
                0.9991154655152222,
                0.9996246133391057,
                0.027079837431832125,
                0.0041609079184150724,
                -4.070139009594471e-05,
            ],
            qvel: &[
                -1.100930912594699,
                0.0002039946418371859,
                -0.07572969469858519,
                4.351654313723001,
                0.7134110405259178,
                -0.01419786021848551,
            ],
        }),
    ),
    // a turned site at the end of a hinge chain, to a turned world site
    (
        "w11_weld_site_hinge_chain",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0.4 0 1" quat="0.8660254 0 0.5 0"/><body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><body name="b" pos="0.2 0 0"><joint type="hinge" axis="0 0 1"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><site name="s" pos="0.2 0 0" quat="0.9238795 0.3826834 0 0"/></body></body></worldbody><equality><weld site1="s" site2="w"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[1],
            eq_obj2id: &[0],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[0.6581308930401093, 0.35720540606133966],
            efc_eq_pos: &[
                0.0,
                0.0,
                0.0,
                0.33141355970379927,
                -0.4619397711104239,
                0.1913417087442235,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[0.0066506345248086626, 0.01007433180311042],
            qvel: &[0.4160570488952215, 0.6296855351581452],
        }),
    ),
    // a site on a body whose inertial frame is turned
    (
        "w12_weld_sites_inertial_offset",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1" quat="0.9238795 0.3826834 0 0"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" pos="0.05 0.01 0" euler="0.3 0.2 0.1" mass="1"/><site name="s" pos="-0.1 0 0" quat="0.8660254 0 0.5 0"/></body></worldbody><equality><weld site1="w" site2="s"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                -496.24879682634844,
                -56.26804760474455,
                96.55395308742727,
                2.316575098630291,
                -6.631152119340601,
                0.5213404712196938,
            ],
            efc_eq_pos: &[
                -0.19999999999999998,
                0.0,
                0.0,
                0.33141355970379927,
                -0.4619397711104239,
                0.1913417087442235,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.2404992297103405,
                -0.005890173697924724,
                1.0093942086344707,
                0.9941805527800524,
                0.09072310210298654,
                -0.05084662055295514,
                -0.028088581258296857,
            ],
            qvel: &[
                -3.8003581880550907,
                -0.47106433133711045,
                0.8032612543147889,
                12.345307471561851,
                -8.76468693202755,
                -3.8885223631336463,
            ],
        }),
    ),
    // two sleeping bodies held by sites beside an awake one
    (
        "w13_weld_sites_asleep",
        r#"<mujoco model="m"><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="c" pos="2 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="x"/></body><body name="a" pos="0 0 1" sleep="init"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="sa" pos="0.1 0 0" quat="0.9238795 0.3826834 0 0"/></body><body name="b" pos="0.3 0 1" sleep="init"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="sb" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="sa" site2="sb"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[1],
            eq_obj2id: &[2],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0,
            ],
            efc_eq_pos: &[],
            nisland: 0,
            efc_island: &[],
            sensordata: &[],
            qpos: &[
                2.0,
                0.0,
                0.9978418000000002,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.3,
                0.0,
                1.0,
                1.0,
                0.0,
                0.0,
                0.0,
            ],
            qvel: &[
                0.0, 0.0, -0.1962, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0,
            ],
        }),
    ),
    // sensors on a body a site weld holds: the weld's force and torque on its site's body
    (
        "w14_weld_sites_sensors",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/><site name="m" pos="0.05 0.02 0"/></body></worldbody><equality><weld site1="w" site2="s"/></equality><sensor><accelerometer site="m"/><force site="m"/><torque site="m"/><framelinacc objtype="site" objname="m"/></sensor></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                -500.0000000000008,
                0.0,
                5.310491088136937,
                0.0,
                0.1462936585381578,
                0.0,
            ],
            efc_eq_pos: &[-0.19999999999999998, 0.0, 0.0, 0.0, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[
                -499.9999999999999,
                0.0,
                3.2004863976827496,
                9.094947017729282e-13,
                0.0,
                3.730349362740526e-14,
                -7.460698725481052e-16,
                0.38475545027553965,
                1.8189894035458565e-14,
                -499.9999999999999,
                0.0,
                3.2004863976827496,
            ],
            qpos: &[
                0.24086433329388923,
                0.0,
                0.9992622022818202,
                0.9999939234902786,
                0.0,
                0.00348611280926635,
                0.0,
            ],
            qvel: &[
                -3.741682865047363,
                0.0,
                -0.057301909761929795,
                0.0,
                0.5438694672684516,
                0.0,
            ],
        }),
    ),
    // a body-form connect: eq_objtype body
    (
        "c05_connect_body_form",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><connect body1="a" anchor="-0.1 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[1],
            eq_obj2id: &[0],
            eq_objtype: &[1],
            eq_data: &[
                -0.1,
                0.0,
                0.0,
                0.19999999999999998,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            qfrc_constraint: &[0.0, 0.0, 2.4551229946524002, 0.0, 0.24551229946524003, 0.0],
            efc_eq_pos: &[0.0, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.29999346133323423,
                0.0,
                0.9983891819539331,
                0.9999688406554172,
                0.0,
                0.00789415722293211,
                0.0,
            ],
            qvel: &[
                -0.0011657652430356936,
                0.0,
                -0.1462028928351946,
                0.0,
                1.4418955206286581,
                0.0,
            ],
        }),
    ),
    // a body-form weld: eq_objtype body
    (
        "w08_weld_body_form",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><weld body1="a"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[1],
            eq_obj2id: &[0],
            eq_objtype: &[1],
            eq_data: &[0.0, 0.0, 0.0, -0.3, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                2.410866813855059,
                0.0,
                8.105739955843479,
                0.0,
                0.009286301801511224,
                0.0,
            ],
            efc_eq_pos: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.30055558770490914,
                0.0,
                0.9997146338455062,
                0.9999999567963345,
                0.0,
                0.0002939512364660452,
                0.0,
            ],
            qvel: &[
                0.05134236486937464,
                0.0,
                -0.022886191933568445,
                0.0,
                0.05337419544375159,
                0.0,
            ],
        }),
    ),
    // two free bodies' site connect at the sparse size: the sites' bodies' chain
    (
        "c04s_connect_two_moving_sparse",
        r#"<mujoco model="m"><option timestep="0.002" jacobian="sparse"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s2" pos="-0.1 0 0" quat="0.9238795 0.3826834 0 0"/></body></worldbody><equality><connect site1="s1" site2="s2"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[
                125.0000000000002,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -125.0000000000002,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            efc_eq_pos: &[-0.09999999999999998, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.014783589295672077,
                0.0,
                0.9978418000000002,
                1.0,
                0.0,
                0.0,
                0.0,
                0.28521641070432796,
                0.0,
                0.9978418000000002,
                1.0,
                0.0,
                0.0,
                0.0,
            ],
            qvel: &[
                0.935369139580197,
                0.0,
                -0.1962,
                0.0,
                0.0,
                0.0,
                -0.9353691395801967,
                0.0,
                -0.1962,
                0.0,
                0.0,
                0.0,
            ],
        }),
    ),
    // a hinge chain's site connect at the sparse size, its site ids not its bodies'
    (
        "c10s_connect_site_hinge_chain_sparse",
        r#"<mujoco model="m"><option timestep="0.002" jacobian="sparse"/><worldbody><site name="w" pos="0.4 0 1"/><body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><body name="b" pos="0.2 0 0"><joint type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><site name="s" pos="0.2 0 0"/></body></body></worldbody><equality><connect site1="s" site2="w"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[0],
            eq_obj1id: &[1],
            eq_obj2id: &[0],
            eq_objtype: &[6],
            eq_data: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qfrc_constraint: &[-0.33810504921285967, -0.16905252460642983],
            efc_eq_pos: &[0.0, 0.0, 0.0],
            nisland: 1,
            efc_island: &[0, 0, 0],
            sensordata: &[],
            qpos: &[0.014874620314335736, -0.02969470323446091],
            qvel: &[1.3369844758748635, -2.6703253910696985],
        }),
    ),
    // two free bodies' site weld at the sparse size
    (
        "w04s_weld_two_moving_sparse",
        r#"<mujoco model="m"><option timestep="0.002" jacobian="sparse"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s1" pos="0.1 0 0" quat="0.9238795 0.3826834 0 0"/></body><body name="b" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s2" pos="-0.1 0 0" quat="0.8660254 0 0.5 0"/></body></worldbody><equality><weld site1="s1" site2="s2"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[
                125.0000000000002,
                6.750155989720945e-14,
                -2.700062395888378e-13,
                -0.8459262108380264,
                3.3884233516323996,
                1.5980435497100092,
                -125.0000000000002,
                -6.750155989720945e-14,
                2.700062395888378e-13,
                0.8459262108380264,
                -3.388423351632346,
                -1.5980435497099954,
            ],
            efc_eq_pos: &[
                -0.09999999999999998,
                0.0,
                0.0,
                0.33141355970379927,
                -0.4619397711104239,
                0.1913417087442235,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[
                0.01533768817362632,
                4.0233738463272093e-07,
                0.9978194859474621,
                0.9958200520383869,
                -0.05583331695433596,
                0.06623217561599425,
                0.02895450896343582,
                0.2846623118263737,
                -4.023373846318635e-07,
                0.9978641140525378,
                0.9959366504360663,
                0.053417510958810276,
                -0.0680720001755178,
                -0.024959179338651036,
            ],
            qvel: &[
                1.0143167147816825,
                0.0010056816073574816,
                -0.19880632975350349,
                -7.434610396333362,
                8.957013902590317,
                4.306634935738879,
                -1.0143167147816823,
                -0.001005681607357434,
                -0.19359367024649657,
                6.783858192926639,
                -9.48484544872885,
                -3.161850924831181,
            ],
        }),
    ),
    // a hinge chain's site weld at the sparse size, its site ids not its bodies'
    (
        "w11s_weld_site_hinge_chain_sparse",
        r#"<mujoco model="m"><option timestep="0.002" jacobian="sparse"/><worldbody><site name="w" pos="0.4 0 1" quat="0.8660254 0 0.5 0"/><body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><body name="b" pos="0.2 0 0"><joint type="hinge" axis="0 0 1"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/><site name="s" pos="0.2 0 0" quat="0.9238795 0.3826834 0 0"/></body></body></worldbody><equality><weld site1="s" site2="w"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[1],
            eq_obj1id: &[1],
            eq_obj2id: &[0],
            eq_objtype: &[6],
            eq_data: &[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[0.6581308930401093, 0.35720540606133966],
            efc_eq_pos: &[
                0.0,
                0.0,
                0.0,
                0.33141355970379927,
                -0.4619397711104239,
                0.1913417087442235,
            ],
            nisland: 1,
            efc_island: &[0, 0, 0, 0, 0, 0],
            sensordata: &[],
            qpos: &[0.006650634524808663, 0.01007433180311042],
            qvel: &[0.41605704889522155, 0.6296855351581453],
        }),
    ),
    // two slides with a ref, falling along their axes
    (
        "j14_slides_ref_moving",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="slide" axis="0 0.6 0.8" ref="0.05"/><geom type="box" size="0.05 0.05 0.05" mass="1"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="slide" axis="0 0 1" ref="-0.02"/><geom type="box" size="0.05 0.05 0.05" mass="2"/></body></worldbody><equality><joint joint1="j1" joint2="j2" polycoef="0.01 0.5 0 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[1],
            eq_objtype: &[0],
            eq_data: &[0.01, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[24.302382513661243, -12.151191256830622],
            efc_eq_pos: &[-0.01],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.05143693549173666, -0.022949073872934173],
            qvel: &[0.05998472426599434, -0.2504361810664986],
        }),
    ),
    // one slide with a ref held as it falls
    (
        "j15_single_slide_ref_moving",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="slide" axis="0 0.6 0.8" ref="0.05"/><geom type="box" size="0.05 0.05 0.05" mass="1"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="slide" axis="0 0 1"/><geom type="box" size="0.05 0.05 0.05" mass="2"/></body></worldbody><equality><joint joint1="j1" polycoef="0.02 0 0 0 0"/></equality></mujoco>"#,
        Expect::Loads(Want {
            eq_type: &[2],
            eq_obj1id: &[0],
            eq_obj2id: &[-1],
            eq_objtype: &[0],
            eq_data: &[0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            qfrc_constraint: &[57.45560000000019, 0.0],
            efc_eq_pos: &[-0.02],
            nisland: 1,
            efc_island: &[0],
            sensordata: &[],
            qpos: &[0.05586702707475185, -0.0021582000000000003],
            qvel: &[0.37121134502910863, -0.1962],
        }),
    ),
    // a ball joint coupled (MuJoCo's compiler couples hinges and slides only, L465), naming the joint refused
    (
        "j07_ball_follower",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // a free joint as the leader
    (
        "j08_free_leader",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><freejoint name="j2"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality></mujoco>"#,
        Expect::Refused(
            "invalid joint2 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // a ball joint held alone
    (
        "j12_ball_single",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1"/></equality></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // a site weld with a relpose: MuJoCo's semantics mixed
    (
        "w05_weld_sites_relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="w" site2="s" relpose="0.1 0 0 0.9238795 0.3826834 0 0"/></equality></mujoco>"#,
        Expect::Refused("body and site semantics cannot be mixed"),
    ),
    // a weld with body1 and site1
    (
        "w06_weld_body_and_site",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><weld body1="a" site1="w"/></equality></mujoco>"#,
        Expect::Refused("body and site semantics cannot be mixed"),
    ),
    // a site weld with an anchor
    (
        "w09_weld_sites_anchor",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="w" site2="s" anchor="0.1 0 0"/></equality></mujoco>"#,
        Expect::Refused("body and site semantics cannot be mixed"),
    ),
    // a weld with site1 alone: this names the attribute missing, as a connect's form does
    (
        "w07_weld_site1_only",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="s"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'site2' on <weld>"),
    ),
    // a weld of one site with itself
    (
        "w15_weld_one_site_twice",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="s" site2="s"/></equality></mujoco>"#,
        Expect::Refused("element 's' is repeated in equality constraint 0"),
    ),
    // a connect's torquescale, which MuJoCo's schema does not take
    (
        "c07_connect_torquescale",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><connect body1="a" anchor="-0.1 0 0" torquescale="0.5"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'torquescale' on <connect>"),
    ),
    // a connect with body1 and site2
    (
        "c08_connect_body_and_site",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><connect body1="a" site2="w" anchor="-0.1 0 0"/></equality></mujoco>"#,
        Expect::Refused("body and site semantics cannot be mixed"),
    ),
    // a ball joint coupled to itself: the repeated check first, as MuJoCo's
    (
        "j13_ball_coupled_to_itself",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j1"/></equality></mujoco>"#,
        Expect::Refused("element 'j1' is repeated in equality constraint 0"),
    ),
    // a ball joint coupled beside an exclude naming an unknown body: the exclude first, as MuJoCo and main name it
    (
        "k01_ball_equality_and_exclude",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality><contact><exclude body1="a" body2="nope"/></contact></mujoco>"#,
        Expect::Refused("unknown body 'nope'"),
    ),
    // a connect's relpose, which MuJoCo's schema gives a weld only
    (
        "s01_connect_relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><connect body1="a" anchor="-0.1 0 0" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'relpose' on <connect>"),
    ),
    // a site connect's relpose
    (
        "s02_connect_sites_relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><connect site1="w" site2="s" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'relpose' on <connect>"),
    ),
    // a connect's polycoef
    (
        "s03_connect_polycoef",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><connect body1="a" anchor="-0.1 0 0" polycoef="0 1"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'polycoef' on <connect>"),
    ),
    // a weld's polycoef
    (
        "s04_weld_polycoef",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><weld body1="a" polycoef="0 1"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'polycoef' on <weld>"),
    ),
    // a joint equality's torquescale
    (
        "s05_joint_torquescale",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" torquescale="0.5"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'torquescale' on <joint>"),
    ),
    // a tendon equality's torquescale
    (
        "s06_tendon_torquescale",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><tendon><fixed name="t1"><joint joint="j1" coef="1"/></fixed><fixed name="t2"><joint joint="j2" coef="1"/></fixed></tendon><equality><tendon tendon1="t1" tendon2="t2" torquescale="0.5"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'torquescale' on <tendon>"),
    ),
    // a joint equality's relpose
    (
        "s07_joint_relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'relpose' on <joint>"),
    ),
    // a weld's joint1
    (
        "s08_weld_joint1",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/></body></worldbody><equality><weld body1="a" joint1="x"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'joint1' on <weld>"),
    ),
    // a flex equality's polycoef
    (
        "s09_flex_polycoef",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><flexcomp name="f" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" radius="0.01"/></worldbody><equality><flex flex="f" polycoef="0 1"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'polycoef' on <flex>"),
    ),
    // a flexvert equality's body1
    (
        "s10_flexvert_body1",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><flexcomp name="f" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" radius="0.01"/></worldbody><equality><flexvert flex="f" body1="x"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'body1' on <flexvert>"),
    ),
    // a connect's torquescale beside a partial form: MuJoCo's schema first
    (
        "c13_connect_torquescale_and_partial",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><connect site1="s" torquescale="0.5"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'torquescale' on <connect>"),
    ),
    // a weld's partial form beside a torquescale that is no number: the form first, read before it
    (
        "w16_weld_partial_bad_torquescale",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site1="s" torquescale="x"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'site2' on <weld>"),
    ),
    // a weld with no attribute
    (
        "n01_weld_empty",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'body1' on <weld>"),
    ),
    // a weld with body2 alone
    (
        "n02_weld_body2_only",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld body2="a"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'body1' on <weld>"),
    ),
    // a weld with site2 alone
    (
        "n03_weld_site2_only",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld site2="s"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'site1' on <weld>"),
    ),
    // a weld with an anchor alone
    (
        "n04_weld_anchor_only",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld anchor="0 0 0"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'body1' on <weld>"),
    ),
    // a weld with a relpose alone
    (
        "n05_weld_relpose_only",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'body1' on <weld>"),
    ),
    // a weld with body2 and an anchor
    (
        "n06_weld_body2_and_anchor",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld body2="a" anchor="0.1 0 0"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'body1' on <weld>"),
    ),
    // a ball joint's coupling beside a sensor's negative noise: the coupling first, as MuJoCo's (the late block's order)
    (
        "q1_ball_eq_and_sensor_noise",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality><sensor><jointpos joint="j2" noise="-1"/></sensor></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // beside a free joint below the top level: the coupling first
    (
        "q2_ball_eq_and_nested_free",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/><body name="c" pos="0 0 -0.3"><freejoint/><geom size="0.05"/></body></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // beside a tendon's negative armature: the coupling first
    (
        "q3_ball_eq_and_tendon_armature",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality><tendon><spatial armature="-1"><site site="s1"/><site site="s2"/></spatial></tendon></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // beside a massless moving body: the coupling first
    (
        "q4_ball_eq_and_massless",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/><body name="c" pos="0 0 -0.3"><joint type="hinge" axis="1 0 0"/></body></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // before a rigid flex's equality: the coupling first, one equality at a time as MuJoCo's
    (
        "q5_ball_eq_then_rigid_flex",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body><flexcomp name="f" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" radius="0.01" rigid="true"/></worldbody><equality><joint joint1="j1" joint2="j2"/><flex flex="f"/></equality></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // before an unknown flex's equality: the coupling first
    (
        "q6_ball_eq_then_unknown_flex",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body><flexcomp name="f" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" radius="0.01" rigid="true"/></worldbody><equality><joint joint1="j1" joint2="j2"/><flex flex="nope"/></equality></mujoco>"#,
        Expect::Refused(
            "invalid joint1 on <joint>: only HINGE and SLIDE joint allowed in constraint",
        ),
    ),
    // two foreign attributes: the first written, as MuJoCo's schema names it
    (
        "s11_connect_polycoef_then_relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><equality><connect body1="a" anchor="0 0 0" polycoef="0 1" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'polycoef' on <connect>"),
    ),
    // torquescale written before relpose on a connect
    (
        "s12_connect_torquescale_then_relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><equality><connect body1="a" anchor="0 0 0" torquescale="1" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        Expect::Refused("unrecognized attribute 'torquescale' on <connect>"),
    ),
    // a connect with site1 alone: this names the attribute missing
    (
        "c09_connect_site1_only",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><connect site1="s"/></equality></mujoco>"#,
        Expect::Refused("required attribute missing: 'site2' on <connect>"),
    ),
];

/// 0 on macOS arm64, where MuJoCo's values were printed, and 1e-9 of an
/// array's largest entry elsewhere, where the platform's libm gives other
/// last bits.
const TOLERANCE: f64 = if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
    0.0
} else {
    1e-9
};

fn close(ours: &[f64], mujoco: &[f64]) -> bool {
    if ours.len() != mujoco.len() {
        return false;
    }
    // Equal bits pass; otherwise both entries are finite and within the
    // tolerance of the array's largest finite entry, and two zeros have one
    // sign.
    let scale = mujoco
        .iter()
        .filter(|v| v.is_finite())
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter().zip(mujoco).all(|(a, b)| {
        a.to_bits() == b.to_bits()
            || (a.is_finite()
                && b.is_finite()
                && (a - b).abs() <= TOLERANCE * scale
                && !(*a == 0.0 && *b == 0.0))
    })
}

/// MuJoCo's number for an equality's type (`mjtEq`).
const fn type_number(t: EqualityType) -> i64 {
    match t {
        EqualityType::Connect => 0,
        EqualityType::Weld => 1,
        EqualityType::Joint => 2,
        EqualityType::Tendon => 3,
        EqualityType::Flex => 4,
        EqualityType::Distance => 5,
    }
}

/// MuJoCo's number for an equality's object type (`mjtObj`): the unknown
/// one (0) for a type MuJoCo leaves it unset on.
const fn objtype_number(t: MjObjectType) -> i64 {
    match t {
        MjObjectType::Body => 1,
        MjObjectType::Site => 6,
        _ => 0,
    }
}

/// An object's id as MuJoCo numbers it: -1 for none.
fn id(x: usize) -> i64 {
    if x == usize::MAX {
        -1
    } else {
        i64::try_from(x).unwrap()
    }
}

/// The fields of the model `doc` loads to that are not MuJoCo 3.5.0's
/// `want`; `load`, `make_data`, `forward` or `step` where that fails, so that
/// one case's failure leaves the others compared.
fn differing(doc: &str, want: &Want) -> Vec<&'static str> {
    let Ok(m) = load_model(doc) else {
        return vec!["load"];
    };
    let Ok(mut d) = m.try_make_data() else {
        return vec!["make_data"];
    };
    if d.forward(&m).is_err() {
        return vec!["forward"];
    }
    let efc_eq_pos: Vec<f64> = (0..d.efc_pos.len())
        .filter(|&i| d.efc_type[i] == ConstraintType::Equality)
        .map(|i| d.efc_pos[i])
        .collect();
    let qfrc = d.qfrc_constraint.as_slice().to_vec();
    let nisland = i64::try_from(d.nisland).unwrap();
    let efc_island: Vec<i64> = d.efc_island.iter().map(|&i| i64::from(i)).collect();
    let sensordata = d.sensordata.as_slice().to_vec();
    for _ in 0..10 {
        if d.step(&m).is_err() {
            return vec!["step"];
        }
    }
    // A joint equality's tenth entry is its degree-10 coefficient here, 0
    // where none is written, where MuJoCo stores its default 1 (registry
    // `D-POLYCOEF-DEGREE`): it is compared as 0.
    let want_data: Vec<f64> = want
        .eq_data
        .chunks(11)
        .zip(&m.eq_type)
        .flat_map(|(row, t)| {
            let mut row = row.to_vec();
            if *t == EqualityType::Joint {
                row[10] = 0.0;
            }
            row
        })
        .collect();
    let eq_data: Vec<f64> = m.eq_data.iter().flatten().copied().collect();
    let types: Vec<i64> = m.eq_type.iter().map(|&t| type_number(t)).collect();
    let obj1: Vec<i64> = m.eq_obj1id.iter().map(|&x| id(x)).collect();
    let obj2: Vec<i64> = m.eq_obj2id.iter().map(|&x| id(x)).collect();
    let objtype: Vec<i64> = m.eq_objtype.iter().map(|&t| objtype_number(t)).collect();
    [
        ("eq_type", types == want.eq_type),
        ("eq_obj1id", obj1 == want.eq_obj1id),
        ("eq_obj2id", obj2 == want.eq_obj2id),
        ("eq_objtype", objtype == want.eq_objtype),
        ("eq_data", close(&eq_data, &want_data)),
        ("qfrc_constraint", close(&qfrc, want.qfrc_constraint)),
        ("efc_eq_pos", close(&efc_eq_pos, want.efc_eq_pos)),
        ("nisland", nisland == want.nisland),
        ("efc_island", efc_island == want.efc_island),
        ("sensordata", close(&sensordata, want.sensordata)),
        ("qpos", close(d.qpos.as_slice(), want.qpos)),
        ("qvel", close(d.qvel.as_slice(), want.qvel)),
    ]
    .into_iter()
    .filter(|&(_, same)| !same)
    .map(|(field, _)| field)
    .collect()
}

/// Each document MuJoCo 3.5.0 loads loads here as its model, and steps as it
/// does.
#[test]
fn each_equality_loads_and_steps_as_mujocos() {
    let mut bad = Vec::new();
    for (name, doc, expect) in CASES {
        if let Expect::Loads(want) = expect {
            let fields = differing(doc, want);
            if !fields.is_empty() {
                bad.push(format!("{name}: {fields:?}"));
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// Each document MuJoCo 3.5.0 refuses is refused here, in the words its
/// case gives.
#[test]
fn each_equality_mujoco_refuses_is_refused() {
    let mut bad = Vec::new();
    for (name, doc, expect) in CASES {
        if let Expect::Refused(words) = expect {
            match load_model(doc) {
                Ok(_) => bad.push(format!("{name}: loads")),
                Err(e) if !e.to_string().contains(words) => bad.push(format!("{name}: {e}")),
                Err(_) => {}
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}
