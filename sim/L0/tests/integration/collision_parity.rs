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

/// Our contacts after `forward` against MuJoCo's list: the same geoms in the
/// same order, each normal, tangent and distance within `tol`, the same
/// dimension.
fn lists_as_mujoco(name: &str, tol: f64) {
    let case = &golden()["order"][name];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let want = case["contacts"].as_array().expect("contacts");
    let ours: Vec<[usize; 2]> = data.contacts.iter().map(|c| [c.geom1, c.geom2]).collect();
    let theirs: Vec<[usize; 2]> = want
        .iter()
        .map(|w| {
            let g = floats(&w["geom"]);
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            [g[0] as usize, g[1] as usize]
        })
        .collect();
    assert_eq!(ours, theirs, "{name}: contact geoms in order");
    for (k, (c, w)) in data.contacts.iter().zip(want).enumerate() {
        let normal = floats(&w["normal"]);
        let tangents = floats(&w["tangents"]);
        for i in 0..3 {
            assert!(
                (c.normal[i] - normal[i]).abs() <= tol,
                "{name}: contact {k} normal {:?}, MuJoCo {normal:?}",
                c.normal
            );
            for (t, ours) in c.frame.iter().enumerate() {
                assert!(
                    (ours[i] - tangents[3 * t + i]).abs() <= tol,
                    "{name}: contact {k} tangent {t} {ours:?}, MuJoCo {:?}",
                    &tangents[3 * t..3 * t + 3]
                );
            }
        }
        let dist = w["dist"].as_f64().expect("dist");
        assert!(
            (-c.depth - dist).abs() <= tol,
            "{name}: contact {k} depth {}, MuJoCo's dist {dist}",
            c.depth
        );
        assert_eq!(c.dim, count(&w["dim"]), "{name}: contact {k} dim");
    }
}

/// A sphere (geom 1) on a box (geom 0): MuJoCo lists the lower geom type
/// first (`mj_collideGeoms`, `engine_collision_driver.c:1475-1480`), so the
/// sphere is `geom1` and the normal points from it into the box.
#[test]
fn o_sphere_box_order() {
    lists_as_mujoco("sphere_box", 1e-12);
}

/// MuJoCo's list order: body pairs by signature, a `<pair>` ahead of the
/// broadphase's contacts at its signature, and within a pair of bodies that
/// are not both single-geom, the midphase's order by the listed geoms
/// (`engine_collision_driver.c:226-256`, `:290-370`), or with the midphase
/// disabled, the first body's geoms in turn against the second's.
#[test]
fn contacts_in_mujoco_body_pair_order() {
    for name in ["bodies", "bodies_no_midphase", "pairs", "pairs_no_midphase"] {
        lists_as_mujoco(name, 1e-12);
    }
}

/// A 3 × 3 flex grid on a plane between two spheres: MuJoCo lists the
/// plane's contacts with the spheres first, then its contacts with the
/// flex's vertices (a body–flex pair's signature is past every body pair's
/// of the same first body). sim-mjcf does not load MuJoCo's `<flexcomp>` in
/// a body (ledger L152), so the same flex is built in sim-mjcf's
/// `<deformable><flex>` form from MuJoCo's vertex positions and elements.
#[test]
fn flex_contacts_follow_the_body_contacts_as_mujoco() {
    let case = &golden()["order"]["flex"];
    let join = |v: Vec<String>| v.join(" ");
    let pos = join(
        floats(&case["flexvert_xpos"])
            .iter()
            .map(f64::to_string)
            .collect(),
    );
    let elem = join(
        floats(&case["flex_elem"])
            .iter()
            .map(f64::to_string)
            .collect(),
    );
    let xml = format!(
        r#"<mujoco><option><flag gravity="disable"/></option><worldbody>
<body name="ground"><geom name="floor" type="plane" size="2 2 0.1"/></body>
<body name="ball" pos="0.8 0 0.045"><freejoint/><geom name="ball" type="sphere" size="0.05"/></body>
<body name="ball2" pos="-0.8 0 0.045"><freejoint/><geom name="ball2" type="sphere" size="0.05"/></body>
</worldbody><deformable><flex name="cloth" dim="2" radius="0.01" mass="0.1">
<contact selfcollide="none"/><vertex pos="{pos}"/><element data="{elem}"/></flex></deformable></mujoco>"#
    );
    let model: Model = load_model(&xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let want = case["contacts"].as_array().expect("contacts");
    assert_eq!(data.ncon, want.len(), "contacts");
    for (k, (c, w)) in data.contacts.iter().zip(want).enumerate() {
        let (geom, vert) = (floats(&w["geom"]), floats(&w["vert"]));
        let ours = match c.flex_vertex {
            Some(v) => format!("plane {} – vertex {v}", c.geom1),
            None => format!("geoms ({}, {})", c.geom1, c.geom2),
        };
        let theirs = if vert[1] >= 0.0 {
            format!("plane {} – vertex {}", geom[0], vert[1])
        } else {
            format!("geoms ({}, {})", geom[0], geom[1])
        };
        assert_eq!(ours, theirs, "contact {k}");
        let dist = w["dist"].as_f64().expect("dist");
        assert!(
            (-c.depth - dist).abs() <= 1e-12,
            "contact {k}: depth {}, MuJoCo's dist {dist}",
            c.depth
        );
    }
}

/// The frames' bound: `make_frame` is MuJoCo's arithmetic, so the frames
/// differ only where the collision functions' normals or the kinematics'
/// axes do. Measured: 0, 1.1e-16 (the diagonal's sphere–sphere normal) and 0.
const FRAME_TOL: f64 = 1e-15;

/// Our contact frames after `forward` against MuJoCo's: the same contacts in
/// order, the normal and both tangents within `tol` (MuJoCo's `mju_makeFrame`,
/// which `mj_setContact` runs on every contact).
fn frames_as_mujoco(name: &str, tol: f64) {
    let case = &golden()["frame"][name];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let want = case["contacts"].as_array().expect("contacts");
    assert_eq!(data.ncon, want.len(), "{name}: contacts");
    for (k, (c, w)) in data.contacts.iter().zip(want).enumerate() {
        let geom = floats(&w["geom"]);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let listed = (geom[0] as usize, geom[1] as usize);
        assert_eq!((c.geom1, c.geom2), listed, "{name}: contact {k} geoms");
        let frame = floats(&w["frame"]);
        for (part, ours) in [c.normal, c.frame[0], c.frame[1]].iter().enumerate() {
            for i in 0..3 {
                let theirs = frame[3 * part + i];
                assert!(
                    (ours[i] - theirs).abs() <= tol,
                    "{name}: contact {k} frame row {part} {ours:?}, MuJoCo {:?}",
                    &frame[3 * part..3 * part + 3]
                );
            }
        }
    }
}

/// Two spheres along x: MuJoCo's first tangent is (0, 1, 0)
/// (`mju_makeFrame`'s default for a normal whose y lies in (−0.5, 0.5)).
#[test]
fn f_sphere_sphere_x() {
    frames_as_mujoco("sphere_sphere_x", FRAME_TOL);
}

/// Two spheres along a diagonal whose normal's y is past 0.5: MuJoCo's
/// first tangent starts from (0, 0, 1).
#[test]
fn f_sphere_sphere_diagonal() {
    frames_as_mujoco("sphere_sphere_diagonal", FRAME_TOL);
}

/// A horizontal capsule on a plane: `mjc_PlaneCapsule` passes the capsule's
/// axis as the first tangent (`engine_collision_primitive.c:80-85`).
#[test]
fn f_plane_capsule() {
    frames_as_mujoco("plane_capsule", FRAME_TOL);
}

/// MuJoCo's contacts after `forward` and how many constraint rows they make:
/// each contact listed at `dist ≤ margin` by its collision function, with no
/// rows where its distance is at or past `margin − gap` or the model has no
/// dofs (`mj_setContact`, `mj_instantiateContact`).
fn includes_as_mujoco(name: &str) {
    let case = &golden()["inclusion"][name];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let want = case["contacts"].as_array().expect("contacts");
    assert_eq!(data.ncon, want.len(), "{name}: contacts listed");
    for (k, (c, w)) in data.contacts.iter().zip(want).enumerate() {
        let dist = w["dist"].as_f64().expect("dist");
        assert!(
            (-c.depth - dist).abs() <= 1e-12,
            "{name}: contact {k} depth {}, MuJoCo's dist {dist}",
            c.depth
        );
        let pos = floats(&w["pos"]);
        for i in 0..3 {
            assert!(
                (c.pos[i] - pos[i]).abs() <= 1e-12,
                "{name}: contact {k} pos {:?}, MuJoCo {pos:?}",
                c.pos
            );
        }
        assert_eq!(
            c.is_excluded(),
            w["exclude"].as_u64() == Some(1),
            "{name}: contact {k} excluded"
        );
    }
    assert_eq!(data.efc_type.len(), count(&case["nefc"]), "{name}: rows");
}

/// A sphere resting on a plane at distance 0: MuJoCo lists the contact
/// (inclusive) and excludes it (`dist ≥ includemargin` = 0), so no rows.
#[test]
fn i_sphere_touching_plane() {
    includes_as_mujoco("sphere_touching_plane");
}

/// An upright cylinder and a box resting on a plane at distance 0: each
/// touching point listed and excluded (ledger L41's zero-distance contacts).
#[test]
fn i_cylinder_and_box_touching_plane() {
    includes_as_mujoco("cylinder_touching_plane");
    includes_as_mujoco("box_touching_plane");
}

/// A box tilted 10 degrees within a plane's margin 0.1 and gap 0.08: four
/// corners listed, the two in the gap with no rows (`mjc_PlaneBox` per
/// corner).
#[test]
fn i_box_plane_gap() {
    includes_as_mujoco("box_plane_gap");
}

/// An explicit pair of two static spheres: listed, but with no dofs no rows.
#[test]
fn i_static_pair() {
    includes_as_mujoco("static_pair");
}

/// Census document 830708de: a box dropped on a plane, tumbling under the
/// census's first excitation, lands on one corner, then two, then three:
/// MuJoCo lists each corner by its own distance (`mjc_PlaneBox`), where the
/// box's whole bottom face was listed once its deepest corner was within the
/// margin. The contact count after every step, and the state within 1e-12.
#[test]
fn box_plane_lists_each_corner_as_mujoco() {
    let case = &golden()["box_plane_tumble"];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    for (i, v) in data.qvel.iter_mut().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        {
            *v = 0.1 * (1.0 + (i % 3) as f64);
        }
    }
    for (k, want) in case["rows"].as_array().expect("rows").iter().enumerate() {
        data.step(&model).expect("step");
        assert_eq!(data.ncon, count(&want["ncon"]), "step {k}: contacts");
        for (field, ours) in [("qpos", &data.qpos), ("qvel", &data.qvel)] {
            for (i, (a, b)) in ours.iter().zip(floats(&want[field])).enumerate() {
                assert!(
                    (a - b).abs() <= 1e-12,
                    "step {k}: {field}[{i}] {a}, MuJoCo {b}"
                );
            }
        }
    }
}

/// An adhesion actuator on a tilted box with two corners active, one in the
/// gap and one past the margin: MuJoCo's body transmission averages the
/// active contacts' rows (the normal row, or the pyramid's rows at
/// 0.5/(dim − 1) each) and the excluded contact's normal Jacobian, negated
/// (`mjTRN_BODY`, `engine_core_smooth.c:1621-1731`); ledger L124.
#[test]
fn adhesion_transmission_counts_contacts_as_mujoco() {
    for cone in ["pyramidal", "elliptic"] {
        let case = &golden()["adhesion"][cone];
        let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert_eq!(data.ncon, count(&case["ncon"]), "{cone}: contacts");
        assert_eq!(data.efc_type.len(), count(&case["nefc"]), "{cone}: rows");
        let want = floats(&case["moment"]);
        for (j, (a, b)) in data.actuator_moment[0].iter().zip(&want).enumerate() {
            assert!(
                (a - b).abs() <= 1e-12,
                "{cone}: moment[{j}] {a}, MuJoCo {b}"
            );
        }
    }
}

/// Census document 31833131's two touching spheres, 1.1e-16 into each other
/// after 7 steps: MuJoCo's broadphase sorts the bodies' bounds along its
/// first axis cast to `f32` (`mj_SAP`), where they meet, so it pairs them
/// only with a margin wide enough to part the bounds in `f32`. MuJoCo's
/// contact count from that state at geom margins 0, 1e-9 and 1e-6.
#[test]
fn broadphase_culls_touching_spheres_as_mujoco() {
    let case = &golden()["touching_spheres"];
    for (margin, label) in [(0.0, "0.0"), (1e-9, "1e-09"), (1e-6, "1e-06")] {
        let mut model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        for m in &mut model.geom_margin {
            *m = margin;
        }
        let mut data = model.make_data();
        for (q, v) in data.qpos.iter_mut().zip(floats(&case["qpos"])) {
            *q = v;
        }
        for (q, v) in data.qvel.iter_mut().zip(floats(&case["qvel"])) {
            *q = v;
        }
        data.forward(&model).expect("forward");
        assert_eq!(
            data.ncon,
            count(&case["ncon"][label]),
            "margin {margin}: contacts"
        );
    }
}
