//! A sphere, capsule, cylinder, ellipsoid or box that names a mesh, fitted
//! to it as MuJoCo 3.5.0 fits one (book Rigid-loading L35, ledger L337):
//! `mjCMesh::FitGeom` (`user_mesh.cc:979-1057`) replaces the places its type
//! fits, from the mesh's inertia box or, under `fitaabb`, its box, and
//! scales all three by `fitscale`; `mjCGeom::Compile` then takes the mesh's
//! frame into the geom's, about the fit's centre (`user_objects.cc:3727-3756`),
//! and checks the fitted size (`:3757`).
//!
//! Every expected value and reason was printed by MuJoCo 3.5.0 on the census's
//! unfused oracle (2026-11-13; the parent class's `fitscale`, the replaced
//! `<inertial>` and the negative mass 2026-11-14). Each geom is turned by a
//! written `quat`, made unit by a square root alone, so its bits are the same
//! on every platform.
//! MuJoCo keeps a fitted geom's mesh in its `geom_dataid`; sim-mjcf's
//! `geom_mesh` holds none for it (its doc says so).

// The values are MuJoCo's, printed to the last digit, and compared to it.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unreadable_literal,
    clippy::float_cmp
)]

use sim_core::{GeomType, Model};
use sim_mjcf::{MjcfError, load_model};

/// A geom as MuJoCo stores it: its type, its mesh (MuJoCo's `geom_dataid`
/// for a mesh geom), `geom_pos`, `geom_quat` (w, x, y, z), `geom_size`,
/// `geom_aabb` and `geom_rbound`.
struct Geom {
    kind: GeomType,
    mesh: Option<usize>,
    pos: [f64; 3],
    quat: [f64; 4],
    size: [f64; 3],
    aabb: [f64; 6],
    rbound: f64,
}

/// A body after the world as MuJoCo stores it: `body_mass`, `body_inertia`,
/// `body_ipos` and `body_iquat` (w, x, y, z).
struct Body {
    mass: f64,
    inertia: [f64; 3],
    ipos: [f64; 3],
    iquat: [f64; 4],
}

/// A document MuJoCo loads, and its `nmesh`, geoms and bodies.
struct Loads {
    what: &'static str,
    xml: &'static str,
    nmesh: usize,
    geoms: &'static [Geom],
    bodies: &'static [Body],
}

const LOADS: &[Loads] = &[
    Loads {
        what: "a sphere, from the mesh's inertia box",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="sphere" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Sphere,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.6492066606155911, 0.0, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.6492066606155911,
                0.6492066606155911,
                0.6492066606155911,
            ],
            rbound: 0.6492066606155911,
        }],
        bodies: &[Body {
            mass: 1146.1395787053682,
            inertia: [193.2250529602566, 193.2250529602566, 193.2250529602566],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a sphere, from the mesh's box (`fitaabb`)",
        xml: r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="sphere" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Sphere,
            mesh: None,
            pos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [1.7346546012032606, 0.0, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                1.7346546012032606,
                1.7346546012032606,
                1.7346546012032606,
            ],
            rbound: 1.7346546012032606,
        }],
        bodies: &[Body {
            mass: 21863.900717279736,
            inertia: [26315.62340819788, 26315.62340819788, 26315.62340819788],
            ipos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a capsule, from the mesh's inertia box",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="capsule" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Capsule,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.4498234907850218, 0.8230612548842187, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.4498234907850218,
                0.4498234907850218,
                1.2728847456692405,
            ],
            rbound: 1.2728847456692405,
        }],
        bodies: &[Body {
            mass: 1427.6510489755055,
            inertia: [684.2141463588432, 684.2141463588432, 136.721945123074],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a capsule, from the mesh's box (`fitaabb`)",
        xml: r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="capsule" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Capsule,
            mesh: None,
            pos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [1.0684717245507858, 0.6661828766524749, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                1.0684717245507858,
                1.0684717245507858,
                1.7346546012032606,
            ],
            rbound: 1.7346546012032606,
        }],
        bodies: &[Body {
            mass: 9888.077790917385,
            inertia: [9399.30486808989, 9399.30486808989, 5060.9563020937],
            ipos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a cylinder, from the mesh's inertia box",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="cylinder" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Cylinder,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.4498234907850218, 1.0479730002767296, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.4498234907850218,
                0.4498234907850218,
                1.0479730002767296,
            ],
            rbound: 1.1404335062470905,
        }],
        bodies: &[Body {
            mass: 1332.3374183941658,
            inertia: [555.1420516637412, 555.1420516637412, 134.79335794291742],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a cylinder, from the mesh's box (`fitaabb`)",
        xml: r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="cylinder" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Cylinder,
            mesh: None,
            pos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [1.0684717245507858, 1.7346546012032606, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                1.0684717245507858,
                1.0684717245507858,
                1.7346546012032606,
            ],
            rbound: 2.037316473118542,
        }],
        bodies: &[Body {
            mass: 12442.823714202492,
            inertia: [16031.543341346107, 16031.543341346107, 7102.561779744156],
            ipos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "an ellipsoid, from the mesh's inertia box",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="ellipsoid" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Ellipsoid,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.25999429786814693, 0.6396526837018968, 1.0479730002767296],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.25999429786814693,
                0.6396526837018968,
                1.0479730002767296,
            ],
            rbound: 1.0479730002767296,
        }],
        bodies: &[Body {
            mass: 730.0401616754544,
            inertia: [220.09294086683562, 170.22265331125223, 69.60970767744834],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "an ellipsoid, from the mesh's box (`fitaabb`)",
        xml: r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="ellipsoid" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Ellipsoid,
            mesh: None,
            pos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.4697409950902548, 1.0684717245507858, 1.7346546012032606],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.4697409950902548,
                1.0684717245507858,
                1.7346546012032606,
            ],
            rbound: 1.7346546012032606,
        }],
        bodies: &[Body {
            mass: 3646.893819737066,
            inertia: [3027.4021018500434, 2355.6623315643546, 993.6242502161379],
            ipos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a box, from the mesh's inertia box",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Box,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.25999429786814693, 0.6396526837018968, 1.0479730002767296],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.25999429786814693,
                0.6396526837018968,
                1.0479730002767296,
            ],
            rbound: 1.2549900398011131,
        }],
        bodies: &[Body {
            mass: 1394.2740046350602,
            inertia: [700.5775895717821, 541.8355340140755, 221.5745812809552],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a box, from the mesh's box (`fitaabb`)",
        xml: r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Box,
            mesh: None,
            pos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.4697409950902548, 1.0684717245507858, 1.7346546012032606],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.4697409950902548,
                1.0684717245507858,
                1.7346546012032606,
            ],
            rbound: 2.0907690006570685,
        }],
        bodies: &[Body {
            mass: 6965.054140109251,
            inertia: [9636.52018472456, 7498.306086476928, 3162.804219957532],
            ipos: [-0.5177395922582934, 0.19340068407591032, 1.5599973389676782],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a box scaled by its `fitscale`",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" fitscale="2" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Box,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.5199885957362939, 1.2793053674037935, 2.095946000553459],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.5199885957362939,
                1.2793053674037935,
                2.095946000553459,
            ],
            rbound: 2.5099800796022262,
        }],
        bodies: &[Body {
            mass: 11154.192037080482,
            inertia: [22418.48286629703, 17338.737088450416, 7090.386600990566],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a cylinder scaled by its class's `fitscale`",
        xml: r#"<mujoco><compiler /><default><geom fitscale="0.5"/></default><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="cylinder" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Cylinder,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.2249117453925109, 0.5239865001383648, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.2249117453925109,
                0.2249117453925109,
                0.5239865001383648,
            ],
            rbound: 0.5702167531235453,
        }],
        bodies: &[Body {
            mass: 166.54217729927072,
            inertia: [17.348189114491912, 17.348189114491912, 4.2122924357161695],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a sphere's written second and third sizes, scaled",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="sphere" mesh="m" size="0.5 0.2 0.3" fitscale="2" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Sphere,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [1.2984133212311821, 0.4, 0.6],
            aabb: [
                0.0,
                0.0,
                0.0,
                1.2984133212311821,
                1.2984133212311821,
                1.2984133212311821,
            ],
            rbound: 1.2984133212311821,
        }],
        bodies: &[Body {
            mass: 9169.116629642946,
            inertia: [6183.201694728211, 6183.201694728211, 6183.201694728211],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a box fitted to a shell mesh's inertia box",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" inertia="shell"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Box,
            mesh: None,
            pos: [-0.35847953216374273, 0.4748538011695907, 1.2286549707602337],
            quat: [
                0.8136391122033646,
                0.24044901166140575,
                -0.2272270307623693,
                0.4780622808535425,
            ],
            size: [0.3345330703092596, 0.8077392740335413, 1.3383573684217474],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.3345330703092596,
                0.8077392740335413,
                1.3383573684217474,
            ],
            rbound: 1.598610507770907,
        }],
        bodies: &[Body {
            mass: 2893.1592369540863,
            inertia: [2356.6162797554994, 1835.336145429241, 737.1336822185202],
            ipos: [-0.35847953216374273, 0.4748538011695907, 1.2286549707602337],
            iquat: [
                0.8136391122033646,
                0.24044901166140575,
                -0.2272270307623693,
                0.4780622808535425,
            ],
        }],
    },
    Loads {
        what: "an untyped geom whose class names a mesh, a fitted sphere",
        xml: r#"<mujoco><compiler /><default><geom mesh="m"/></default><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Sphere,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.6492066606155911, 0.0, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.6492066606155911,
                0.6492066606155911,
                0.6492066606155911,
            ],
            rbound: 0.6492066606155911,
        }],
        bodies: &[Body {
            mass: 1146.1395787053682,
            inertia: [193.2250529602566, 193.2250529602566, 193.2250529602566],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "an ellipsoid with its mass written",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="ellipsoid" mesh="m" mass="2" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Ellipsoid,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.25999429786814693, 0.6396526837018968, 1.0479730002767296],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.25999429786814693,
                0.6396526837018968,
                1.0479730002767296,
            ],
            rbound: 1.0479730002767296,
        }],
        bodies: &[Body {
            mass: 2.0,
            inertia: [0.6029611860304196, 0.4663377776931844, 0.19070103627639579],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a world geom fitted, beside a body's sphere",
        xml: r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><geom type="box" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/><body pos="0 0 1"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[
            Geom {
                kind: GeomType::Box,
                mesh: None,
                pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
                quat: [
                    0.8195037416774446,
                    0.2285226966383506,
                    -0.23198912081885698,
                    0.4715634022265704,
                ],
                size: [0.25999429786814693, 0.6396526837018968, 1.0479730002767296],
                aabb: [
                    0.0,
                    0.0,
                    0.0,
                    0.25999429786814693,
                    0.6396526837018968,
                    1.0479730002767296,
                ],
                rbound: 1.2549900398011131,
            },
            Geom {
                kind: GeomType::Sphere,
                mesh: None,
                pos: [0.0, 0.0, 0.0],
                quat: [1.0, 0.0, 0.0, 0.0],
                size: [0.1, 0.0, 0.0],
                aabb: [0.0, 0.0, 0.0, 0.1, 0.1, 0.1],
                rbound: 0.1,
            },
        ],
        bodies: &[Body {
            mass: 4.188790204786391,
            inertia: [
                0.01675516081914557,
                0.01675516081914557,
                0.01675516081914557,
            ],
            ipos: [0.0, 0.0, 0.0],
            iquat: [1.0, 0.0, 0.0, 0.0],
        }],
    },
    Loads {
        what: "a fitted geom under `discardvisual`, kept with its mesh",
        xml: r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Box,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.25999429786814693, 0.6396526837018968, 1.0479730002767296],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.25999429786814693,
                0.6396526837018968,
                1.0479730002767296,
            ],
            rbound: 1.2549900398011131,
        }],
        bodies: &[Body {
            mass: 1394.2740046350602,
            inertia: [700.5775895717821, 541.8355340140755, 221.5745812809552],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a capsule fitted to a flat mesh's inertia box",
        xml: r#"<mujoco><compiler /><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Capsule,
            mesh: None,
            pos: [0.0, 0.0, 0.0],
            quat: [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            size: [0.5500000007450584, 0.7249999996274705, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.5500000007450584,
                0.5500000007450584,
                1.275000000372529,
            ],
            rbound: 1.275000000372529,
        }],
        bodies: &[Body {
            mass: 2074.8910538596847,
            inertia: [1004.7026380419161, 1004.7026380419161, 292.74574600151954],
            ipos: [0.0, 0.0, 0.0],
            iquat: [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
        }],
    },
    Loads {
        what: "a cylinder scaled by its parent class's `fitscale`",
        xml: r#"<mujoco><compiler /><default><default class="a"><geom fitscale="2"/><default class="b"><geom type="cylinder"/></default></default></default><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom class="b" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Cylinder,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.8996469815700436, 2.095946000553459, 0.0],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.8996469815700436,
                0.8996469815700436,
                2.095946000553459,
            ],
            rbound: 2.280867012494181,
        }],
        bodies: &[Body {
            mass: 10658.699347153326,
            inertia: [17764.54565323972, 17764.54565323972, 4313.387454173358],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
    Loads {
        what: "a negative `<inertial>` a fitted geom replaces (`inertiafromgeom`)",
        xml: r#"<mujoco><compiler inertiafromgeom="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="-1" diaginertia="1 1 1"/><geom type="box" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
        nmesh: 1,
        geoms: &[Geom {
            kind: GeomType::Box,
            mesh: None,
            pos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            quat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
            size: [0.25999429786814693, 0.6396526837018968, 1.0479730002767296],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.25999429786814693,
                0.6396526837018968,
                1.0479730002767296,
            ],
            rbound: 1.2549900398011131,
        }],
        bodies: &[Body {
            mass: 1394.2740046350602,
            inertia: [700.5775895717821, 541.8355340140755, 221.5745812809552],
            ipos: [-0.2605263157894736, 0.4894736842105262, 1.1131578947368421],
            iquat: [
                0.8195037416774446,
                0.2285226966383506,
                -0.23198912081885698,
                0.4715634022265704,
            ],
        }],
    },
];

/// A box fitted to a convex mesh a mesh geom names too. Under `convex` this
/// crate sums a mesh's mass properties over its own hull, MuJoCo over qhull's
/// (ledger L408, registry `D-MESH-CONVEX-MASS`), so the fit, from the hull's
/// inertia box, is MuJoCo's to [`HULL_ROUNDING`] and not to the bit.
const CONVEX: Loads = Loads {
    what: "a box fitted to a convex mesh a mesh geom names too",
    xml: r#"<mujoco><compiler /><asset><mesh name="m" inertia="convex" vertex="0 0 0  2 0 0  2 1 0  1 1 0  1 2 0  0 2 0  0 0 1  2 0 1  2 1 1  1 1 1  1 2 1  0 2 1" face="0 2 1  0 3 2  0 4 3  0 5 4  6 7 8  6 8 9  6 9 10  6 10 11  0 1 7  0 7 6  1 2 8  1 8 7  2 3 9  2 9 8  3 4 10  3 10 9  4 5 11  4 11 10  5 0 6  5 6 11"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
    nmesh: 1,
    geoms: &[
        Geom {
            kind: GeomType::Box,
            mesh: None,
            pos: [0.046867167919799585, 1.232581453634085, 1.2045112781954885],
            quat: [
                -0.07376164447585432,
                0.5802279577563578,
                -0.1713399838806634,
                0.7928034729779392,
            ],
            size: [0.5000000000000002, 0.8340133280065236, 1.0522085616183023],
            aabb: [
                0.0,
                0.0,
                0.0,
                0.5000000000000002,
                0.8340133280065236,
                1.0522085616183023,
            ],
            rbound: 1.4327320365076557,
        },
        Geom {
            kind: GeomType::Mesh,
            mesh: Some(0),
            pos: [0.9047619047619048, 0.9047619047619045, 0.4999999999999999],
            quat: [
                0.2705980500730989,
                0.6532814824381896,
                -0.2705980500730989,
                0.6532814824381896,
            ],
            size: [0.5000000000000021, 1.2795265564328053, 1.4142135623731007],
            aabb: [
                1.1102230246251565e-16,
                0.2188663846529797,
                0.0,
                0.500000000000002,
                1.0606601717798256,
                1.4142135623731007,
            ],
            rbound: 1.9715953460628808,
        },
    ],
    bodies: &[Body {
        mass: 7010.223856928951,
        inertia: [5932.954438287173, 5471.923016035907, 2884.56579391699],
        ipos: [0.4751889505372758, 1.0689107286373982, 0.8527693760810493],
        iquat: [
            0.03322769458928426,
            0.7549522435613364,
            -0.3103689234083521,
            0.5767271119310506,
        ],
    }],
};

/// The most a value of [`CONVEX`] may be from MuJoCo's, times the larger of 1
/// and MuJoCo's magnitude: the largest measured, 7.8e-16 (a stored vertex),
/// 2026-11-14.
const HULL_ROUNDING: f64 = 1e-14;

/// `(what, document, MuJoCo's reason)`: a fitted geom's mass is checked on
/// its fitted volume, which a written mass's sign decides
/// (`user_objects.cc:3780-3807`, after the fit).
const NEGATIVE_MASS: (&str, &str, &str) = (
    "a fitted box of negative mass",
    r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" mass="-1" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#,
    "mass, inertia or density are negative in geom",
);

/// A visual geom fitted to a mesh only it names, which `discardvisual`
/// discards with the mesh. Its body's mass leaves out the discarded geom
/// here, where MuJoCo's keeps it (ledger L336), so the body is not compared.
const DISCARDED: Loads = Loads {
    what: "a fitted visual geom `discardvisual` discards, with its mesh",
    xml: r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" contype="0" conaffinity="0" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/><geom size="0.1"/></body></worldbody></mujoco>"#,
    nmesh: 0,
    geoms: &[Geom {
        kind: GeomType::Sphere,
        mesh: None,
        pos: [0.0, 0.0, 0.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        size: [0.1, 0.0, 0.0],
        aabb: [0.0, 0.0, 0.0, 0.1, 0.1, 0.1],
        rbound: 0.1,
    }],
    bodies: &[Body {
        mass: 1398.4627948398465,
        inertia: [703.9142393640352, 546.6660382817762, 226.3754735576572],
        ipos: [
            -0.25974596604853323,
            0.48800757257603217,
            1.1098236731164606,
        ],
        iquat: [
            0.8240170757834305,
            0.226929837580949,
            -0.2295958100894164,
            0.4656011937508263,
        ],
    }],
};

/// `(what, document, MuJoCo's reason)`, each refused by MuJoCo.
const REFUSED: &[(&str, &str, &str)] = &[
    (
        "a capsule fitted to a flat mesh's box, its half length not above 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        "size 1 must be positive",
    ),
    (
        "a box scaled by a `fitscale` of 0",
        r#"<mujoco><compiler /><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body pos="0 0 1"><freejoint/><geom type="box" mesh="m" fitscale="0"/></body></worldbody></mujoco>"#,
        "size 0 must be positive",
    ),
];

fn bits<const N: usize>(values: [f64; N]) -> [u64; N] {
    values.map(f64::to_bits)
}

/// Each of `ours` within [`HULL_ROUNDING`] of MuJoCo's `want`, scaled by the
/// larger of 1 and its magnitude (a `NaN` is not).
fn within<const N: usize>(ours: [f64; N], want: [f64; N]) -> bool {
    ours.iter()
        .zip(want)
        .all(|(&a, b)| (a - b).abs() <= HULL_ROUNDING * b.abs().max(1.0))
}

/// Each field of `model`'s geoms that is not within [`HULL_ROUNDING`] of
/// MuJoCo's; the type and mesh exactly.
fn geom_distances(model: &Model, geoms: &[Geom]) -> Vec<String> {
    let mut wrong = Vec::new();
    let mut check = |field: String, same: bool| {
        if !same {
            wrong.push(field);
        }
    };
    check(format!("ngeom {}", model.ngeom), model.ngeom == geoms.len());
    for (g, want) in geoms.iter().enumerate().take(model.ngeom) {
        let q = model.geom_quat[g];
        check(
            format!("geom {g} type {:?}", model.geom_type[g]),
            model.geom_type[g] == want.kind,
        );
        check(
            format!("geom {g} mesh {:?}", model.geom_mesh[g]),
            model.geom_mesh[g] == want.mesh,
        );
        check(
            format!("geom {g} pos {:?}", model.geom_pos[g]),
            within(model.geom_pos[g].into(), want.pos),
        );
        check(
            format!("geom {g} quat {q:?}"),
            within([q.w, q.i, q.j, q.k], want.quat),
        );
        check(
            format!("geom {g} size {:?}", model.geom_size[g]),
            within(model.geom_size[g].into(), want.size),
        );
        check(
            format!("geom {g} aabb {:?}", model.geom_aabb[g]),
            within(model.geom_aabb[g], want.aabb),
        );
        check(
            format!("geom {g} rbound {}", model.geom_rbound[g]),
            within([model.geom_rbound[g]], [want.rbound]),
        );
    }
    wrong
}

/// Each field of `model`'s bodies after the world that is not within
/// [`HULL_ROUNDING`] of MuJoCo's.
fn body_distances(model: &Model, bodies: &[Body]) -> Vec<String> {
    let mut wrong = Vec::new();
    let mut check = |field: String, same: bool| {
        if !same {
            wrong.push(field);
        }
    };
    check(
        format!("nbody {}", model.nbody),
        model.nbody == bodies.len() + 1,
    );
    for (k, want) in bodies
        .iter()
        .enumerate()
        .take(model.nbody.saturating_sub(1))
    {
        let b = k + 1;
        let q = model.body_iquat[b];
        check(
            format!("body {b} mass {}", model.body_mass[b]),
            within([model.body_mass[b]], [want.mass]),
        );
        check(
            format!("body {b} inertia {:?}", model.body_inertia[b]),
            within(model.body_inertia[b].into(), want.inertia),
        );
        check(
            format!("body {b} ipos {:?}", model.body_ipos[b]),
            within(model.body_ipos[b].into(), want.ipos),
        );
        check(
            format!("body {b} iquat {q:?}"),
            within([q.w, q.i, q.j, q.k], want.iquat),
        );
    }
    wrong
}

/// Each field of `model`'s geoms that is not MuJoCo's, bit for bit.
fn geom_differences(model: &Model, geoms: &[Geom]) -> Vec<String> {
    let mut wrong = Vec::new();
    let mut check = |field: String, same: bool| {
        if !same {
            wrong.push(field);
        }
    };
    check(format!("ngeom {}", model.ngeom), model.ngeom == geoms.len());
    for (g, want) in geoms.iter().enumerate().take(model.ngeom) {
        let q = model.geom_quat[g];
        check(
            format!("geom {g} type {:?}", model.geom_type[g]),
            model.geom_type[g] == want.kind,
        );
        check(
            format!("geom {g} mesh {:?}", model.geom_mesh[g]),
            model.geom_mesh[g] == want.mesh,
        );
        check(
            format!("geom {g} pos {:?}", model.geom_pos[g]),
            bits(model.geom_pos[g].into()) == bits(want.pos),
        );
        check(
            format!("geom {g} quat {q:?}"),
            bits([q.w, q.i, q.j, q.k]) == bits(want.quat),
        );
        check(
            format!("geom {g} size {:?}", model.geom_size[g]),
            bits(model.geom_size[g].into()) == bits(want.size),
        );
        check(
            format!("geom {g} aabb {:?}", model.geom_aabb[g]),
            bits(model.geom_aabb[g]) == bits(want.aabb),
        );
        check(
            format!("geom {g} rbound {}", model.geom_rbound[g]),
            model.geom_rbound[g].to_bits() == want.rbound.to_bits(),
        );
    }
    wrong
}

/// Each field of `model`'s bodies after the world that is not MuJoCo's, bit
/// for bit.
fn body_differences(model: &Model, bodies: &[Body]) -> Vec<String> {
    let mut wrong = Vec::new();
    let mut check = |field: String, same: bool| {
        if !same {
            wrong.push(field);
        }
    };
    check(
        format!("nbody {}", model.nbody),
        model.nbody == bodies.len() + 1,
    );
    for (k, want) in bodies
        .iter()
        .enumerate()
        .take(model.nbody.saturating_sub(1))
    {
        let b = k + 1;
        let q = model.body_iquat[b];
        check(
            format!("body {b} mass {}", model.body_mass[b]),
            model.body_mass[b].to_bits() == want.mass.to_bits(),
        );
        check(
            format!("body {b} inertia {:?}", model.body_inertia[b]),
            bits(model.body_inertia[b].into()) == bits(want.inertia),
        );
        check(
            format!("body {b} ipos {:?}", model.body_ipos[b]),
            bits(model.body_ipos[b].into()) == bits(want.ipos),
        );
        check(
            format!("body {b} iquat {q:?}"),
            bits([q.w, q.i, q.j, q.k]) == bits(want.iquat),
        );
    }
    wrong
}

/// Each document MuJoCo fits, loaded with MuJoCo's geoms and bodies.
#[test]
fn a_primitive_naming_a_mesh_is_fitted_as_mujoco_fits_it() {
    let mut missed = Vec::new();
    for case in LOADS {
        match load_model(case.xml) {
            Ok(model) => {
                let mut wrong = geom_differences(&model, case.geoms);
                wrong.extend(body_differences(&model, case.bodies));
                if model.nmesh != case.nmesh {
                    wrong.push(format!("nmesh {}", model.nmesh));
                }
                if !wrong.is_empty() {
                    missed.push(format!("{}: {}", case.what, wrong.join("; ")));
                }
            }
            Err(e) => missed.push(format!("{}: {e}", case.what)),
        }
    }
    assert!(
        missed.is_empty(),
        "{} of {} not MuJoCo's:\n  {}",
        missed.len(),
        LOADS.len(),
        missed.join("\n  ")
    );
}

/// Each fitted size MuJoCo refuses, refused with its reason.
#[test]
fn a_fitted_size_mujoco_refuses_is_refused() {
    let mut missed = Vec::new();
    for &(what, xml, reason) in REFUSED {
        match load_model(xml) {
            Err(MjcfError::InvalidValue {
                element,
                attribute,
                reason: r,
                ..
            }) if element == "geom" && attribute == "size" && r == reason => {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(m) => missed.push(format!("{what}: loads, size {:?}", m.geom_size[0])),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

/// A fitted geom of negative mass, refused with MuJoCo's reason, at the geom.
#[test]
fn a_fitted_geom_of_negative_mass_is_refused() {
    let (what, xml, reason) = NEGATIVE_MASS;
    match load_model(xml) {
        Err(e) if e.to_string() == format!("{reason} (geom #0 in body #1)") => {}
        other => panic!("{what}: {other:?}"),
    }
}

/// A capsule fitted to a flat mesh's box, its half length not above 0, in a
/// static child `fusestatic` fuses into its parent, after a sibling body's
/// two geoms: MuJoCo's document and reason (review round 1, RA-4).
const FUSED: (&str, &str, &str) = (
    "a fitted capsule in a fused static child, after a sibling's geoms",
    r#"<mujoco><compiler fitaabb="true" fusestatic="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a" pos="0 0 1"><freejoint/><geom size="0.1"/><body name="c" pos="1 0 0"><joint type="hinge"/><geom size="0.1"/><geom size="0.1"/></body><body name="b" pos="0.1 0 0"><geom type="capsule" mesh="f"/></body></body></worldbody></mujoco>"#,
    "size 1 must be positive",
);

/// A fused static child's fitted geom refused with MuJoCo's reason where it
/// was written, as this crate names a geom `fusestatic` moved: MuJoCo's
/// geom 3, in body `b`.
#[test]
fn a_fused_childs_fitted_geom_is_refused_where_it_was_written() {
    let (what, xml, reason) = FUSED;
    match load_model(xml) {
        Err(e)
            if e.to_string()
                == format!("invalid size on <geom>: {reason} (geom #3 in body 'b')") => {}
        other => panic!("{what}: {other:?}"),
    }
}

/// A plane and an ellipsoid fitted to a cube mesh only it names, 0.01 into
/// the plane on a vertical slide; then the same ellipsoid written out, with
/// MuJoCo's fitted size, position and orientation (review round 1, RA-1).
const PLANE_ELLIPSOID: [(&str, &str); 2] = [
    (
        "fitted",
        r#"<mujoco><asset><mesh name="c" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><geom type="plane" size="0 0 1"/><body pos="0 0 0.49"><joint type="slide" axis="0 0 1"/><geom type="ellipsoid" mesh="c"/></body></worldbody></mujoco>"#,
    ),
    (
        "written",
        r#"<mujoco><worldbody><geom type="plane" size="0 0 1"/><body pos="0 0 0.49"><joint type="slide" axis="0 0 1"/><geom type="ellipsoid" size="0.5 0.5 0.5"/></body></worldbody></mujoco>"#,
    ),
];

/// MuJoCo's one contact for the written ellipsoid: its distance and position.
const PLANE_ELLIPSOID_CONTACT: (f64, [f64; 3]) =
    (-0.010000000000000009, [0.0, 0.0, -0.0050000000000000044]);

/// A fitted ellipsoid meets a plane as the ellipsoid it was fitted to: one
/// contact, MuJoCo's for the written one bit for bit (registry
/// `D-FIT-ELLIPSOID-PLANE`, wrong-value). MuJoCo 3.5.0 lists two more for the
/// fitted one, at the cube's corners below the plane: its plane–convex
/// collision takes a geom that holds a mesh's index for a mesh
/// (`engine_collision_convex.c:1074-1104`), and a fitted geom keeps its mesh's.
#[test]
fn a_fitted_ellipsoid_meets_a_plane_as_the_ellipsoid_it_was_fitted_to() {
    let (dist, pos) = PLANE_ELLIPSOID_CONTACT;
    for (what, xml) in PLANE_ELLIPSOID {
        let model = load_model(xml).unwrap();
        let mut data = model.make_data();
        data.forward(&model).unwrap();
        let got: Vec<_> = data.contacts[..data.ncon]
            .iter()
            .map(|c| ((-c.depth).to_bits(), bits([c.pos.x, c.pos.y, c.pos.z])))
            .collect();
        assert_eq!(got, [(dist.to_bits(), bits(pos))], "{what}");
    }
}

/// A box fitted to a convex mesh, from the mesh's hull: MuJoCo's geoms and
/// body to the hulls' rounding.
#[test]
fn a_box_fitted_to_a_convex_mesh_is_fitted_from_its_hull() {
    let model = load_model(CONVEX.xml).unwrap();
    let mut wrong = geom_distances(&model, CONVEX.geoms);
    wrong.extend(body_distances(&model, CONVEX.bodies));
    if model.nmesh != CONVEX.nmesh {
        wrong.push(format!("nmesh {}", model.nmesh));
    }
    assert!(wrong.is_empty(), "{}: {}", CONVEX.what, wrong.join("; "));
}

/// A convex mesh only a fitted geom names: [`CONVEX`]'s document without its
/// mesh geom. MuJoCo 3.5.0 crashes loading it (ledger L409), so it can have
/// no parity-census golden; it is made here at run time rather than written
/// out, which keeps it out of the census. sim-mjcf fits the geom from the
/// mesh's convex hull, as MuJoCo fits it where a mesh geom names the mesh too.
#[test]
fn a_convex_mesh_only_a_fitted_geom_names_fits_as_beside_a_mesh_geom() {
    let mesh_geom = r#"<geom type="mesh" mesh="m"/>"#;
    assert_eq!(CONVEX.xml.matches(mesh_geom).count(), 1, "one mesh geom");
    let model = load_model(&CONVEX.xml.replace(mesh_geom, "")).unwrap();
    // The fitted geom only: the body's mass is that geom's alone here.
    let wrong = geom_distances(&model, &CONVEX.geoms[..1]);
    assert!(wrong.is_empty(), "{}", wrong.join("; "));
}

/// A fitted visual geom `discardvisual` discards goes, with the mesh only it
/// names; the body's other geom is MuJoCo's.
#[test]
fn a_fitted_visual_geom_discardvisual_discards_goes_with_its_mesh() {
    let model = load_model(DISCARDED.xml).unwrap();
    let mut wrong = geom_differences(&model, DISCARDED.geoms);
    if model.nmesh != DISCARDED.nmesh {
        wrong.push(format!("nmesh {}", model.nmesh));
    }
    assert!(wrong.is_empty(), "{}: {}", DISCARDED.what, wrong.join("; "));
}
