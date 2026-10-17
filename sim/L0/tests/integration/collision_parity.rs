//! Collision against MuJoCo 3.5.0.
//!
//! The golden is `assets/golden/collision/collision.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_collision_reference.py`, which describes
//! its cases).

use serde_json::Value;
use sim_core::{Contact, Data, Model};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/collision/collision.json"))
        .expect("collision golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn count(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("count")).expect("count")
}

/// The contact between the geoms of MuJoCo's `want`, its normal turned to
/// MuJoCo's geom order (MuJoCo lists the lower geom type first).
fn matching<'a>(data: &'a Data, want: &Value) -> (&'a Contact, f64) {
    let geoms = floats(&want["geom"]);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let (g1, g2) = (geoms[0] as usize, geoms[1] as usize);
    let c = data
        .contacts
        .iter()
        .find(|c| (c.geom1, c.geom2) == (g1, g2) || (c.geom1, c.geom2) == (g2, g1))
        .unwrap_or_else(|| panic!("no contact between geoms {g1} and {g2}"));
    (c, if c.geom1 == g1 { 1.0 } else { -1.0 })
}

/// A body resting on a slab whose top face is at z = 0 (a box, or a convex
/// mesh of eight vertices): its height after every 50th step from 1 mm
/// above, within `z_tol` of MuJoCo's; and at a turned pose 1 mm in, the
/// contact's position, normal and depth within `contact_tol`.
fn rests_as_mujoco(name: &str, z_tol: f64, contact_tol: f64) {
    let case = &golden()["rest"][name];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos[2] = case["z0"].as_f64().expect("z0");
    let every = count(&case["every"]);
    let mut want = floats(&case["z"]).into_iter();
    for k in 0..count(&case["nstep"]) {
        data.step(&model).expect("step");
        if k % every == every - 1 {
            let (ours, theirs) = (data.qpos[2], want.next().expect("a sample"));
            assert!(
                (ours - theirs).abs() <= z_tol,
                "{name}: z after step {k}: {ours}, MuJoCo {theirs}"
            );
        }
    }

    let mut data = model.make_data();
    for (q, v) in data.qpos.iter_mut().zip(floats(&case["pose"])) {
        *q = v;
    }
    data.forward(&model).expect("forward");
    for want in case["contacts"].as_array().expect("contacts") {
        let (c, sign) = matching(&data, want);
        let (pos, normal) = (floats(&want["pos"]), floats(&want["normal"]));
        for i in 0..3 {
            assert!(
                (c.pos[i] - pos[i]).abs() <= contact_tol,
                "{name}: contact pos {:?}, MuJoCo {pos:?}",
                c.pos
            );
            assert!(
                (sign * c.normal[i] - normal[i]).abs() <= contact_tol,
                "{name}: contact normal {:?} (sign {sign}), MuJoCo {normal:?}",
                c.normal
            );
        }
        let dist = want["dist"].as_f64().expect("dist");
        assert!(
            (-c.depth - dist).abs() <= contact_tol,
            "{name}: depth {}, MuJoCo's dist {dist}",
            c.depth
        );
    }
}

// The pairs below go through GJK/EPA (MuJoCo's through mjc_Convex), whose
// contact was the first geom's support against the normal: a corner of the
// slab's bottom face, or the body's top. Each body fell through its slab.
// MuJoCo's EPA is not ported, so contacts on a curved surface agree within
// the two polytopes' convergence, not bitwise. The bounds sit a decade or
// more above what was measured.

/// A sphere-shaped ellipsoid on a box slab. Measured: height 1.4e-6, contact
/// 1.6e-14.
#[test]
fn ellipsoid_rests_on_box_as_mujoco() {
    rests_as_mujoco("box_ellipsoid", 1e-5, 1e-12);
}

/// A cylinder on a box slab. Upright on its cap, the touching set is a disc
/// over which both engines' single contact moves; at the turned pose, a rim
/// point. Measured: height 6.6e-5, contact 2.0e-4.
#[test]
fn cylinder_rests_on_box_as_mujoco() {
    rests_as_mujoco("box_cylinder", 1e-3, 1e-3);
}

/// A box, a cylinder and a three-axis ellipsoid on a convex mesh slab,
/// through GJK/EPA on the mesh's hull. Measured: height 1.6e-9, 2.9e-4 and
/// 1.7e-9; contact 1.5e-9, 2.0e-4 and 4.3e-6.
#[test]
fn box_cylinder_ellipsoid_rest_on_convex_mesh_slab_as_mujoco() {
    rests_as_mujoco("mesh_box", 1e-8, 1e-8);
    rests_as_mujoco("mesh_cylinder", 1e-3, 1e-3);
    rests_as_mujoco("mesh_ellipsoid3", 1e-8, 1e-5);
}
