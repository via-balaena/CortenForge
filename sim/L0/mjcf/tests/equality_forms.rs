//! Equalities as MuJoCo 3.5.0 builds and steps them (`engine_core_constraint.c`
//! `mj_instantiateEquality`): a joint equality's joints measured from their
//! `qpos0`, in MuJoCo's arithmetic (ledger L446), and one on a ball or free
//! joint refused (ledger L465); a connect's site form, its sites kept as MuJoCo
//! keeps them (ledger L445).
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
    /// Be refused, in words holding MuJoCo 3.5.0's.
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
    // a ball joint coupled (MuJoCo's compiler couples hinges and slides only, L465)
    (
        "j07_ball_follower",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality></mujoco>"#,
        Expect::Refused("only HINGE and SLIDE joint allowed in constraint"),
    ),
    // a free joint as the leader
    (
        "j08_free_leader",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><freejoint name="j2"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality></mujoco>"#,
        Expect::Refused("only HINGE and SLIDE joint allowed in constraint"),
    ),
    // a ball joint held alone
    (
        "j12_ball_single",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/></body></worldbody><equality><joint joint1="j1"/></equality></mujoco>"#,
        Expect::Refused("only HINGE and SLIDE joint allowed in constraint"),
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
/// `want`.
fn differing(doc: &str, want: &Want) -> Vec<&'static str> {
    let m = load_model(doc).unwrap();
    let mut d = m.make_data();
    d.forward(&m).unwrap();
    let efc_eq_pos: Vec<f64> = (0..d.efc_pos.len())
        .filter(|&i| d.efc_type[i] == ConstraintType::Equality)
        .map(|i| d.efc_pos[i])
        .collect();
    let qfrc = d.qfrc_constraint.as_slice().to_vec();
    let nisland = i64::try_from(d.nisland).unwrap();
    let efc_island: Vec<i64> = d.efc_island.iter().map(|&i| i64::from(i)).collect();
    let sensordata = d.sensordata.as_slice().to_vec();
    for _ in 0..10 {
        d.step(&m).unwrap();
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

/// Each document MuJoCo 3.5.0 refuses is refused here, in words holding
/// MuJoCo's.
#[test]
fn each_equality_mujoco_refuses_is_refused_in_its_words() {
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
