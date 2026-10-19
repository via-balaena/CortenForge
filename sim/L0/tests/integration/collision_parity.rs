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

/// A capsule (radius 0.03, half-length 0.05) on the box slab, as
/// [`rests_as_mujoco`]: capsule–box is ported (`mjraw_CapsuleBox`), so its
/// height after every 50th step and its turned pose's contact are MuJoCo's
/// bit for bit (measured: 0).
#[test]
fn capsule_rests_on_box_as_mujoco() {
    rests_as_mujoco("box_capsule", 0.0, 0.0);
}

// The pairs below go through GJK/EPA (MuJoCo's through mjc_Convex), whose
// contact was the first geom's support against the normal: a corner of the
// slab's bottom face, or the body's top. Each body fell through its slab.
// MuJoCo's EPA is not ported, so contacts on a curved surface agree within
// the two polytopes' convergence, not bitwise. Each bound is 2.3 to 62
// times what was measured, which each test's doc gives.

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
/// And three explicit pairs within one body (two spheres, two capsules, a
/// capsule with a sphere), each declared higher geom first: MuJoCo keeps a
/// same-body pair in its declared order (the compiler puts the lower body's
/// geom first only between two bodies), so (2, 1), (4, 3), (1, 3).
#[test]
fn contacts_in_mujoco_body_pair_order() {
    for name in [
        "bodies",
        "bodies_no_midphase",
        "pairs",
        "pairs_no_midphase",
        "pairs_same_body",
    ] {
        lists_as_mujoco(name, 1e-12);
    }
}

/// MuJoCo's list for an `order` flex case against ours: sim-mjcf does not
/// load MuJoCo's `<flexcomp>` in a body (ledger L152), so the same flex is
/// built in sim-mjcf's `<deformable><flex>` form from MuJoCo's vertex
/// positions and elements, beside `bodies`, the case's other bodies.
fn flex_lists_as_mujoco(name: &str, bodies: &str) {
    let case = &golden()["order"][name];
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
{bodies}
</worldbody><deformable><flex name="cloth" dim="2" radius="0.01" mass="0.1">
<contact selfcollide="none"/><vertex pos="{pos}"/><element data="{elem}"/></flex></deformable></mujoco>"#
    );
    let model: Model = load_model(&xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let want = case["contacts"].as_array().expect("contacts");
    assert_eq!(data.ncon, want.len(), "{name}: contacts");
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
        assert_eq!(ours, theirs, "{name}: contact {k}");
        let dist = w["dist"].as_f64().expect("dist");
        assert!(
            (-c.depth - dist).abs() <= 1e-12,
            "{name}: contact {k}: depth {}, MuJoCo's dist {dist}",
            c.depth
        );
    }
}

/// A 3 × 3 flex grid on a plane between two spheres: MuJoCo lists the
/// plane's contacts with the spheres first, then its contacts with the
/// flex's vertices (a body–flex pair's signature is past every body pair's
/// of the same first body).
#[test]
fn flex_contacts_follow_the_body_contacts_as_mujoco() {
    flex_lists_as_mujoco(
        "flex",
        r#"<body name="ground"><geom name="floor" type="plane" size="2 2 0.1"/></body>
<body name="ball" pos="0.8 0 0.045"><freejoint/><geom name="ball" type="sphere" size="0.05"/></body>
<body name="ball2" pos="-0.8 0 0.045"><freejoint/><geom name="ball2" type="sphere" size="0.05"/></body>"#,
    );
}

/// The flex beside a static body holding two planes and a far sphere:
/// MuJoCo lists the first plane's vertex contacts (vertices 0 to 8), then
/// the second's (6 to 8), each geom's in turn; ours, which collides a flex
/// vertex by vertex, places them so.
#[test]
fn flex_contacts_follow_each_plane_in_turn_as_mujoco() {
    flex_lists_as_mujoco(
        "flex_two_planes",
        r#"<body name="ground"><geom name="floor" type="plane" size="2 2 0.1"/>
<geom name="ramp" type="plane" size="2 2 0.1" pos="0.06 0 0" quat="0.99 0 -0.1 0"/>
<geom name="far" type="sphere" size="0.05" pos="3 3 3"/></body>
<body name="ball" pos="0.8 0 0.045"><freejoint/><geom name="ball" type="sphere" size="0.05"/><geom name="ball_b" type="box" size="0.03 0.03 0.03" pos="0.1 0 0"/></body>
<body name="ball2" pos="-0.8 0 0.045"><freejoint/><geom name="ball2" type="sphere" size="0.05"/></body>"#,
    );
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

/// An upright ellipsoid at distance 0, and two in a plane's margin 0.125 and
/// gap 0.0625, at distance 0 and at exactly the margin: each listed, as
/// MuJoCo's `mjc_PlaneConvex` lists at `dist ≤ margin`
/// (`engine_collision_convex.c:1056`), the first and the last excluded.
/// Before, the plane–ellipsoid test was strict and listed neither.
#[test]
fn i_ellipsoid_touching_plane_and_at_the_margin() {
    includes_as_mujoco("ellipsoid_touching_plane");
    includes_as_mujoco("ellipsoid_at_margin");
}

/// Three upright cylinders turned by 1e-12, 3e-11 and 3e-9 about x, 0.001
/// into a plane: their rim points as MuJoCo's, which takes the rim's
/// direction from the axis while its length squared is at least mjMINVAL²
/// (`mjc_PlaneCylinder`, `engine_collision_primitive.c:115`). Before, below a
/// length of 1e-10 the cylinder's x axis was taken, so the first two
/// cylinders' points were turned 90° about the axis (0.1 off).
#[test]
fn i_cylinders_nearly_upright() {
    includes_as_mujoco("cylinders_nearly_upright");
}

/// Two cylinders lying on their sides, at distance 0 and at exactly a geom
/// margin of 0.25: both caps' rim points listed (`dist ≤ margin`, the far
/// cap's too); a cube turned 45° about x and one on its corner, each within
/// its margin, listed corner by corner (`mjc_PlaneBox`).
#[test]
fn i_plane_edges() {
    includes_as_mujoco("plane_edges");
}

/// Two cubes turned 45° about x within a margin 0.3, one turned so that two
/// corners' distances along the plane's normal cancel exactly: `mjc_PlaneBox`
/// takes a corner on the plane's side of the centre at `ldist ≤ 0`, so the
/// one with `ldist` exactly 0 counts, and stops at four
/// (`engine_collision_primitive.c:196-239`): four corners each.
#[test]
fn i_cube_zero_ldist() {
    includes_as_mujoco("cube_zero_ldist");
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
/// (`mjTRN_BODY`, `engine_core_smooth.c:1621-1731`); ledger L124. Under each
/// cone with condim 3, and with condim 1, 4 and 6 (on both geoms): a
/// condim-1 contact's normal row, and the pyramid's rows, whose weights sum
/// to the normal row only at 0.5/(dim − 1); and under each cone with
/// friction loss on the box's free joint, whose six rows come ahead of the
/// contact rows.
#[test]
fn adhesion_transmission_counts_contacts_as_mujoco() {
    let cases = golden()["adhesion"].clone();
    for (cone, case) in cases.as_object().expect("cases") {
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

/// Two height fields, one reaching farther above its origin than below and
/// one farther below: each bounding radius is MuJoCo's, from its size,
/// sqrt(x² + y² + max(top², base²)) (`mjCGeom::GetRBound`). Measured: equal.
/// The scene's plane and pyramid mesh are not compared: a plane's radius is
/// infinite here, MuJoCo's 0 (ledger L173), and the mesh's is about its
/// authored frame, where MuJoCo's is about the mesh's centre of mass (book
/// Rigid-loading L35 adopts MuJoCo's mesh frame).
#[test]
fn height_field_bounds_are_mujocos() {
    let case = &golden()["bounds"];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let want = floats(&case["rbound"]);
    for g in [1, 2] {
        assert_eq!(model.geom_type[g], sim_core::GeomType::Hfield);
        assert_eq!(
            model.geom_rbound[g].to_bits(),
            want[g].to_bits(),
            "geom {g}: rbound {}, MuJoCo {}",
            model.geom_rbound[g],
            want[g]
        );
    }
}

/// A pyramid mesh, its apex 0.01 into a plane. The geom's origin is 4.49
/// above the plane in MuJoCo's mesh frame (the centroid) and 5.99 in
/// sim-mjcf's (the authored frame; book Rigid-loading L35): farther than the
/// half-diagonal of the pyramid's box (3.20), within its bounding radius
/// about the origin (MuJoCo's 4.64, ours 6.10), so the bounding-sphere filter
/// passes the pair, as MuJoCo's. Our contacts after `forward` against
/// MuJoCo's: the same geoms in order, each distance within 1e-13 (measured
/// 6.2e-15). The position is not compared: MuJoCo's is midway between the
/// apex and the plane, ours at the apex (book P51).
#[test]
fn a_mesh_apex_on_a_plane_is_listed_as_mujoco() {
    let case = &golden()["bounds"];
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
    assert_eq!(ours, theirs, "contact geoms in order");
    for (k, (c, w)) in data.contacts.iter().zip(want).enumerate() {
        let dist = w["dist"].as_f64().expect("dist");
        assert!(
            (-c.depth - dist).abs() <= 1e-13,
            "contact {k}: depth {}, MuJoCo's dist {dist}",
            c.depth
        );
    }
}

/// A box mesh authored with its origin at a corner, its base 0.001 into a
/// plane, the plane in the world body and in a static body after the mesh's
/// (so each of the narrow phase's two argument orders is reached): the
/// plane–mesh contacts are spaced as MuJoCo's, whose `addplanemesh` skips a
/// vertex nearer the first contact than 0.3 of `geom_rbound`
/// (`engine_collision_convex.c:1018`) in its mesh frame, at the centre of
/// mass along the principal axes. sim-mjcf keeps the authored frame (book
/// Rigid-loading L35), so the spacing takes the half-diagonal of the mesh's
/// box, which here is MuJoCo's radius (2.077) within 1e-12: three of the
/// base's four corners, the same count and geoms as MuJoCo's, each distance
/// within 1e-13. Which three is not compared: the corners tie for the
/// deepest, which ours takes in vertex order and MuJoCo's in its hull's (book
/// P51's limitation row), nor the height (MuJoCo's contact is midway, ours at
/// the corner; book P51). Before, the spacing took our `geom_rbound`, about
/// the authored origin (4.153), and listed two.
#[test]
fn plane_mesh_contacts_are_spaced_as_mujoco() {
    let cases = golden()["plane_mesh"].clone();
    for (name, case) in cases.as_object().expect("cases") {
        let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mesh = usize::try_from(case["mesh_geom"].as_u64().expect("mesh_geom")).expect("geom");
        let rbound = case["rbound"].as_f64().expect("rbound");
        // `geom_aabb` is the box's centre offset, then its half-extents.
        let aabb = model.geom_aabb[mesh];
        let half = nalgebra::Vector3::new(aabb[3], aabb[4], aabb[5]).norm();
        assert!(
            (half - rbound).abs() <= 1e-12,
            "{name}: half-diagonal {half}, MuJoCo's radius {rbound}"
        );
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let want = case["contacts"].as_array().expect("contacts");
        assert_eq!(data.ncon, want.len(), "{name}: contacts");
        for (k, (c, w)) in data.contacts.iter().zip(want).enumerate() {
            let g = floats(&w["geom"]);
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let theirs = [g[0] as usize, g[1] as usize];
            assert_eq!([c.geom1, c.geom2], theirs, "{name} contact {k}: geoms");
            let dist = w["dist"].as_f64().expect("dist");
            assert!(
                (-c.depth - dist).abs() <= 1e-13,
                "{name} contact {k}: depth {}, MuJoCo's dist {dist}",
                c.depth
            );
        }
    }
}

/// One of each primitive (sphere, capsule, cylinder, ellipsoid, box): each
/// bounding radius is MuJoCo's bit for bit (`mjCGeom::GetRBound`,
/// `user_objects.cc:3428-3441`). The cylinder's is `sqrt(r² + h²)` as
/// MuJoCo's computes it; before, `hypot` gave 0.5385164807134504 where
/// MuJoCo's is 0.5385164807134505 (r 0.2, h 0.5).
#[test]
fn primitive_bounding_radii_are_mujocos_bit_for_bit() {
    let case = &golden()["primitive_rbound"];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let want = floats(&case["rbound"]);
    assert_eq!(
        model
            .geom_rbound
            .iter()
            .map(|r| r.to_bits())
            .collect::<Vec<_>>(),
        want.iter().map(|r| r.to_bits()).collect::<Vec<_>>(),
        "rbound {:?}, MuJoCo {want:?}",
        model.geom_rbound
    );
}

/// The geom-distance sensors (distance, normal, fromto) of two overlapping
/// boxes, in both argument orders: MuJoCo's within 1e-12 (the review
/// measured 1.1e-16). `fromto` is rebuilt from the contact point, the
/// witnesses' midpoint since P40, and its distance and normal (its two ends
/// the two witnesses), which no test compared with MuJoCo's before.
#[test]
fn geom_distance_sensors_of_overlapping_boxes_as_mujoco() {
    let cases = golden()["geom_distance"].clone();
    for (name, case) in cases.as_object().expect("cases") {
        let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let want = floats(&case["sensordata"]);
        assert_eq!(data.sensordata.len(), want.len(), "{name}: sensordata");
        for (k, (a, b)) in data.sensordata.iter().zip(&want).enumerate() {
            assert!(
                (a - b).abs() <= 1e-12,
                "{name}: sensordata[{k}] {a}, MuJoCo {b}"
            );
        }
    }
}

/// The geom-distance sensors of separated pairs, a box, an ellipsoid and a
/// cylinder each turned, 4 mm apart, in both argument orders, and two poses
/// of a random search (an ellipsoid and a box whose distance subalgorithm
/// meets a minor of exactly 0; an ellipsoid and a cylinder whose GJK runs
/// out of its 35 iterations): their distance comes from GJK alone
/// (`gjk_distance`), MuJoCo's within 1e-10. Measured: the distance 1.4e-16
/// at worst, the normal and fromto 1.2e-11 (ellipsoid–cylinder).
#[test]
fn geom_distance_sensors_of_separated_pairs_as_mujoco() {
    let cases = golden()["geom_distance_apart"].clone();
    for (name, case) in cases.as_object().expect("cases") {
        let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let want = floats(&case["sensordata"]);
        assert_eq!(data.sensordata.len(), want.len(), "{name}: sensordata");
        for (k, (a, b)) in data.sensordata.iter().zip(&want).enumerate() {
            assert!(
                (a - b).abs() <= 1e-10,
                "{name}: sensordata[{k}] {a}, MuJoCo {b}"
            );
        }
    }
}

/// Our geom-distance sensors at a golden case's `qpos`, after a forward.
fn sensordata_at(case: &Value) -> Vec<f64> {
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    for (q, v) in data.qpos.iter_mut().zip(floats(&case["qpos"])) {
        *q = v;
    }
    data.forward(&model).expect("forward");
    data.sensordata.iter().copied().collect()
}

/// The geom-distance sensors of seeded random separated pairs of boxes,
/// ellipsoids and cylinders, each turned and posed through `qpos`, 12 per
/// pair of types: the distance within 1e-8 of MuJoCo's and the normal and
/// fromto within 1e-5, and a box against a box all within 1e-14. Measured:
/// the distance 1.2e-9 at worst (cylinder–cylinder), the normal and fromto
/// 6.7e-6 (box–ellipsoid); box–box 1.6e-16 and 2.2e-15. Where the curved
/// pairs' witnesses part from MuJoCo's is not isolated (ledger L191). The
/// bounds are this seed's: on 1,200 pairs of another seed the fix-diff review
/// measured 18 past them (a box–box normal 5.7e-14 off at a distance of
/// 5.5e-4, a cylinder–ellipsoid distance 2.1e-6, an ellipsoid–ellipsoid
/// normal 1.8e-5).
#[test]
fn geom_distance_sensors_of_random_separated_pairs_as_mujoco() {
    let cases = golden()["geom_distance_random"].clone();
    for (name, case) in cases.as_object().expect("cases") {
        let ours = sensordata_at(case);
        let want = floats(&case["sensordata"]);
        assert_eq!(ours.len(), want.len(), "{name}: sensordata");
        let (distance, rest) = if name.starts_with("box_box_") {
            (1e-14, 1e-14)
        } else {
            (1e-8, 1e-5)
        };
        for (k, (a, b)) in ours.iter().zip(&want).enumerate() {
            let bound = if k == 0 { distance } else { rest };
            assert!(
                (a - b).abs() <= bound,
                "{name}: sensordata[{k}] {a}, MuJoCo {b}"
            );
        }
    }
}

/// Registry `D-GJK-S3D`: three box pairs apart whose distance MuJoCo's
/// sensor reads 0 (its GJK keeps a stale weight in `S3D`'s fourth face,
/// `engine_collision_gjk.c:633-643`, and its `mjc_ccd` takes the pair as
/// touching), chosen where ours with that weight kept reads 0 too. Ours reads
/// the pair's exact distance (the generator's, from the boxes' corners and
/// edges) within 1e-12. Measured: 4.2e-17, 0 and 1.1e-16.
#[test]
fn separated_boxes_read_apart_where_mujocos_sensor_reads_0() {
    let cases = golden()["s3d_stale"].clone();
    for (name, case) in cases.as_object().expect("cases") {
        assert_eq!(
            floats(&case["sensordata"])[0],
            0.0,
            "{name}: MuJoCo's sensor"
        );
        let exact = case["exact_distance"].as_f64().expect("exact_distance");
        let ours = sensordata_at(case)[0];
        assert!(
            exact > 0.0 && (ours - exact).abs() <= 1e-12,
            "{name}: ours {ours}, the exact distance {exact}"
        );
    }
}

/// A three-axis ellipsoid (0.05, 0.04, 0.03) 1 mm above the box slab, the
/// slab's margin 0.01: MuJoCo lists one contact from the start, 1 mm apart,
/// and so does ours, from the margin zone's GJK distance. Its distance
/// within 1e-6 of MuJoCo's, its normal within 1e-5 and its position within
/// 1e-3; the height after every 50th of 1,000 steps within 1e-5. Measured:
/// 2.5e-7, 5.2e-7, 1.0e-4 (MuJoCo's contact is that far from the midpoint
/// of the ellipsoid's lowest point and the face, ours 7.5e-9) and 1.6e-6.
#[test]
fn g_ellipsoid_box_margin() {
    let case = &golden()["margin"];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos[2] = case["z0"].as_f64().expect("z0");
    data.forward(&model).expect("forward");
    let contacts = case["contacts"].as_array().expect("contacts");
    assert_eq!(data.ncon, contacts.len(), "contacts at the start");
    for want in contacts {
        let (c, sign) = matching(&data, want);
        let (pos, normal) = (floats(&want["pos"]), floats(&want["normal"]));
        for i in 0..3 {
            assert!(
                (c.pos[i] - pos[i]).abs() <= 1e-3,
                "contact pos {:?}, MuJoCo {pos:?}",
                c.pos
            );
            assert!(
                (sign * c.normal[i] - normal[i]).abs() <= 1e-5,
                "contact normal {:?} (sign {sign}), MuJoCo {normal:?}",
                c.normal
            );
        }
        let dist = want["dist"].as_f64().expect("dist");
        assert!(
            (-c.depth - dist).abs() <= 1e-6,
            "depth {}, MuJoCo's dist {dist}",
            c.depth
        );
    }

    let every = count(&case["every"]);
    let mut want = floats(&case["z"]).into_iter();
    for k in 0..count(&case["nstep"]) {
        data.step(&model).expect("step");
        if k % every == every - 1 {
            let (ours, theirs) = (data.qpos[2], want.next().expect("a sample"));
            assert!(
                (ours - theirs).abs() <= 1e-5,
                "z after step {k}: {ours}, MuJoCo {theirs}"
            );
        }
    }
}

/// A plane, two height fields and one of each primitive: each geom's box,
/// `geom_aabb`, is MuJoCo's bit for bit (`mjCGeom::ComputeAABB`,
/// `user_objects.cc:3573-3626`): a plane's is the half-space below it, to
/// 1e10 each way; a cylinder's half-height is its own; a height field's is
/// from its size, its base to its top.
#[test]
fn geom_boxes_are_mujocos_bit_for_bit() {
    let case = &golden()["geom_aabb"];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let bits = |a: &[f64]| a.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    let want: Vec<Vec<u64>> = case["aabb"]
        .as_array()
        .expect("aabb")
        .iter()
        .map(|a| bits(&floats(a)))
        .collect();
    let ours: Vec<Vec<u64>> = model.geom_aabb.iter().map(|a| bits(a)).collect();
    assert_eq!(ours, want, "geom_aabb {:?}", model.geom_aabb);
}

/// A golden list of integers, -1 for none.
fn indices(v: &Value) -> Vec<Option<usize>> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| usize::try_from(x.as_i64().expect("integer")).ok())
        .collect()
}

/// A golden list of geom pairs.
fn pairs(v: &Value) -> Vec<[usize; 2]> {
    v.as_array()
        .expect("pairs")
        .iter()
        .map(|p| {
            let p: Vec<usize> = indices(p).into_iter().flatten().collect();
            [p[0], p[1]]
        })
        .collect()
}

/// A world body with a plane, a body of six geoms (one visual) in a turned
/// inertial frame, a body of five spheres, three at one place along the
/// split axis and two at another, a body of visual geoms and one with none:
/// the bodies'
/// bounding-volume hierarchies are MuJoCo's (`mjCBody::ComputeBVH`,
/// `user_objects.cc:2323-2335`, `:396-543`). The same nodes, children,
/// geoms and depths; each box within 1e-12 of MuJoCo's, relative past 1;
/// the margins bitwise. The inertial frames the boxes are in are compared
/// first, within 1e-15.
#[test]
fn body_hierarchies_are_mujocos() {
    let case = &golden()["midphase"]["bodies"];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let near = |a: f64, b: f64, tol: f64| (a - b).abs() <= tol * b.abs().max(1.0);
    for (b, (p, q)) in case["body_ipos"]
        .as_array()
        .expect("body_ipos")
        .iter()
        .zip(case["body_iquat"].as_array().expect("body_iquat"))
        .enumerate()
    {
        let (p, q) = (floats(p), floats(q));
        let (ours_p, ours_q) = (model.body_ipos[b], model.body_iquat[b]);
        let ours_q = [ours_q.w, ours_q.i, ours_q.j, ours_q.k];
        for i in 0..3 {
            assert!(
                near(ours_p[i], p[i], 1e-15),
                "body {b}: ipos {ours_p:?}, MuJoCo {p:?}"
            );
        }
        for i in 0..4 {
            assert!(
                near(ours_q[i], q[i], 1e-15),
                "body {b}: iquat {ours_q:?}, MuJoCo {q:?}"
            );
        }
    }
    assert_eq!(
        model
            .body_margin
            .iter()
            .map(|m| m.to_bits())
            .collect::<Vec<_>>(),
        floats(&case["body_margin"])
            .iter()
            .map(|m| m.to_bits())
            .collect::<Vec<_>>(),
        "body_margin {:?}",
        model.body_margin
    );
    assert_eq!(
        model.body_bvhadr,
        indices(&case["body_bvhadr"]),
        "body_bvhadr"
    );
    let counts: Vec<usize> = indices(&case["body_bvhnum"])
        .into_iter()
        .flatten()
        .collect();
    assert_eq!(model.body_bvhnum, counts, "body_bvhnum");
    let children: Vec<[Option<usize>; 2]> = case["bvh_child"]
        .as_array()
        .expect("bvh_child")
        .iter()
        .map(|c| {
            let c = indices(c);
            [c[0], c[1]]
        })
        .collect();
    assert_eq!(model.bvh_child, children, "bvh_child");
    assert_eq!(model.bvh_nodeid, indices(&case["bvh_nodeid"]), "bvh_nodeid");
    let depths: Vec<usize> = indices(&case["bvh_depth"]).into_iter().flatten().collect();
    assert_eq!(model.bvh_depth, depths, "bvh_depth");
    let boxes = case["bvh_aabb"].as_array().expect("bvh_aabb");
    assert_eq!(model.bvh_aabb.len(), boxes.len(), "nodes");
    for (k, (ours, theirs)) in model.bvh_aabb.iter().zip(boxes).enumerate() {
        let theirs = floats(theirs);
        for i in 0..6 {
            assert!(
                near(ours[i], theirs[i], 1e-12),
                "node {k}: box {ours:?}, MuJoCo {theirs:?}"
            );
        }
    }
}

/// Scenes of bodies of several geoms, so every pair of bodies goes through
/// the midphase, each body's inertial frame given (the hierarchies' boxes
/// are in it, and sim-mjcf's principal axes are not MuJoCo's on many
/// bodies, book Rigid-loading L03 and ledger L195; measured on these rafts,
/// their frames inferred: the same six leaf pairs in another order). The
/// geom pairs the contact filter is called with, in order, are MuJoCo's,
/// with the midphase on and with it off (`DISABLE_MIDPHASE`), each in
/// MuJoCo's type order, and the contacts MuJoCo's (the same geoms in order,
/// each distance within 1e-12). In `sparse` the midphase calls the filter
/// once, without it 33 times: a pair whose boxes overlap but whose bounding
/// spheres do not is dropped at the leaves, and the one contact, 0.05 apart
/// within a margin of 0.1, is kept by the bodies' margins. In `rafts` it
/// reaches six leaf pairs, in its order. In `far_frames` the two bodies'
/// inertial frames are 10 either side of their overlapping geoms. In
/// `singles` two single-geom bodies whose bounding spheres are apart are
/// collided without the midphase, as MuJoCo's, so the filter is called for
/// them. In `pile` (turned bodies of up to seven geoms, a child and a
/// grandchild, a static body, a single-geom body, an `<exclude>` and a
/// `<pair>`) the midphase calls the filter 13 times, without it 168; its
/// geom with contype and conaffinity 0 is in no hierarchy, so the filter,
/// which keeps every pair, is called for it only without the midphase, and
/// its contact is listed only then. `twins` holds two pairs of equal bodies,
/// and `twins_spheres` two equal bodies whose roots' boxes have the same
/// surface, where MuJoCo descends the second.
#[test]
fn contact_filter_calls_are_mujocos_with_and_without_the_midphase() {
    use std::sync::{Arc, Mutex};

    let cases = golden()["midphase"]["filter"].clone();
    for (name, case) in cases.as_object().expect("cases") {
        for (key, disable) in [("midphase", false), ("no_midphase", true)] {
            let mut model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
            if disable {
                model.disableflags |= sim_core::DISABLE_MIDPHASE;
            }
            let calls = Arc::new(Mutex::new(Vec::new()));
            let log = Arc::clone(&calls);
            model.set_contactfilter_callback(move |_, _, g1, g2| {
                log.lock().expect("log").push([g1, g2]);
                true
            });
            let mut data = model.make_data();
            data.forward(&model).expect("forward");
            let want = &case[key];
            let theirs = pairs(&want["calls"]);
            assert_eq!(
                *calls.lock().expect("log"),
                theirs,
                "{name} {key}: the filter's calls"
            );
            let contacts = want["contacts"].as_array().expect("contacts");
            let ours: Vec<[usize; 2]> = data.contacts.iter().map(|c| [c.geom1, c.geom2]).collect();
            let listed: Vec<[usize; 2]> = contacts
                .iter()
                .map(|w| {
                    let g: Vec<usize> = indices(&w["geom"]).into_iter().flatten().collect();
                    [g[0], g[1]]
                })
                .collect();
            assert_eq!(ours, listed, "{name} {key}: contact geoms in order");
            for (k, (c, w)) in data.contacts.iter().zip(contacts).enumerate() {
                let dist = w["dist"].as_f64().expect("dist");
                assert!(
                    (-c.depth - dist).abs() <= 1e-12,
                    "{name} {key} contact {k}: depth {}, MuJoCo's dist {dist}",
                    c.depth
                );
            }
        }
    }
}

/// The `pile` scene with its bodies' hierarchies taken away, as in a model
/// built by hand and never derived: each pair of bodies collides each geom of
/// one with each of the other, so the filter's calls and the contacts are
/// MuJoCo's without the midphase.
#[test]
fn bodies_without_hierarchies_collide_every_pair() {
    use std::sync::{Arc, Mutex};

    let case = &golden()["midphase"]["filter"]["pile"];
    let mut model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    assert!(
        model.body_bvhadr.iter().any(Option::is_some),
        "pile has hierarchies"
    );
    for adr in &mut model.body_bvhadr {
        *adr = None;
    }
    let calls = Arc::new(Mutex::new(Vec::new()));
    let log = Arc::clone(&calls);
    model.set_contactfilter_callback(move |_, _, g1, g2| {
        log.lock().expect("log").push([g1, g2]);
        true
    });
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let want = &case["no_midphase"];
    assert_eq!(
        *calls.lock().expect("log"),
        pairs(&want["calls"]),
        "the filter's calls"
    );
    let listed: Vec<[usize; 2]> = want["contacts"]
        .as_array()
        .expect("contacts")
        .iter()
        .map(|w| {
            let g: Vec<usize> = indices(&w["geom"]).into_iter().flatten().collect();
            [g[0], g[1]]
        })
        .collect();
    let ours: Vec<[usize; 2]> = data.contacts.iter().map(|c| [c.geom1, c.geom2]).collect();
    assert_eq!(ours, listed, "contact geoms in order");
}

/// The user's contact filter in place of the geoms' bitmask, as MuJoCo's
/// `mjcb_contactfilter` (`engine_collision_driver.c:1475-1507`): with a
/// filter that keeps every pair, the geom pairs it is called with, in order
/// and in MuJoCo's type order, and the contacts are MuJoCo's, and without a
/// filter the contacts are too. In `bitmask` two touching spheres whose
/// contype and conaffinity reject each other collide under the filter only;
/// in `flat` height fields on free bodies over a plane and over a height
/// field in the world, which MuJoCo has no collision function for, are
/// never given to the filter, and nothing collides.
#[test]
fn the_contact_filter_replaces_the_bitmask_as_mujocos() {
    use std::sync::{Arc, Mutex};

    let contacts_of = |data: &Data| -> Vec<([usize; 2], f64)> {
        data.contacts
            .iter()
            .map(|c| ([c.geom1, c.geom2], -c.depth))
            .collect()
    };
    let theirs_of = |v: &Value| -> Vec<([usize; 2], f64)> {
        v.as_array()
            .expect("contacts")
            .iter()
            .map(|c| {
                let g: Vec<usize> = indices(&c["geom"]).into_iter().flatten().collect();
                ([g[0], g[1]], c["dist"].as_f64().expect("dist"))
            })
            .collect()
    };
    let same =
        |name: &str, what: &str, ours: &[([usize; 2], f64)], theirs: &[([usize; 2], f64)]| {
            assert_eq!(
                ours.iter().map(|c| c.0).collect::<Vec<_>>(),
                theirs.iter().map(|c| c.0).collect::<Vec<_>>(),
                "{name}: {what} contact geoms in order"
            );
            for (k, (a, b)) in ours.iter().zip(theirs).enumerate() {
                assert!(
                    (a.1 - b.1).abs() <= 1e-12,
                    "{name}: {what} contact {k}: {} against {}",
                    a.1,
                    b.1
                );
            }
        };
    let cases = golden()["contact_filter"].clone();
    for (name, case) in cases.as_object().expect("cases") {
        let mut model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        same(
            name,
            "unfiltered",
            &contacts_of(&data),
            &theirs_of(&case["unfiltered"]),
        );

        let calls = Arc::new(Mutex::new(Vec::new()));
        let log = Arc::clone(&calls);
        model.set_contactfilter_callback(move |_, _, g1, g2| {
            log.lock().expect("log").push([g1, g2]);
            true
        });
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert_eq!(
            *calls.lock().expect("log"),
            pairs(&case["calls"]),
            "{name}: the filter's calls"
        );
        same(
            name,
            "filtered",
            &contacts_of(&data),
            &theirs_of(&case["contacts"]),
        );
    }
}

/// Forward `model` from `qpos` and compare its contacts with MuJoCo's `want`:
/// the same geoms in order, each distance, position and normal within 1e-12.
fn contacts_as_mujoco(name: &str, model: &Model, qpos: &[f64], want: &Value) {
    let mut data = model.make_data();
    data.qpos.as_mut_slice().copy_from_slice(qpos);
    data.forward(model).expect("forward");
    let want = want.as_array().expect("contacts");
    let ours: Vec<[usize; 2]> = data.contacts.iter().map(|c| [c.geom1, c.geom2]).collect();
    let theirs: Vec<[usize; 2]> = want
        .iter()
        .map(|w| {
            let g: Vec<usize> = indices(&w["geom"]).into_iter().flatten().collect();
            [g[0], g[1]]
        })
        .collect();
    assert_eq!(ours, theirs, "{name}: contact geoms in order");
    for (k, (c, w)) in data.contacts.iter().zip(want).enumerate() {
        let near = |a: f64, b: f64| (a - b).abs() <= 1e-12;
        let dist = w["dist"].as_f64().expect("dist");
        assert!(
            near(-c.depth, dist),
            "{name} contact {k}: dist {}, MuJoCo {dist}",
            -c.depth
        );
        let (pos, normal) = (floats(&w["pos"]), floats(&w["normal"]));
        for i in 0..3 {
            assert!(
                near(c.pos[i], pos[i]),
                "{name} contact {k}: pos {:?}, MuJoCo {pos:?}",
                c.pos
            );
            assert!(
                near(c.normal[i], normal[i]),
                "{name} contact {k}: normal {:?}, MuJoCo {normal:?}",
                c.normal
            );
        }
    }
}

/// A sphere against a sphere, a capsule, a cylinder and a box, each pair on
/// two free bodies, the sphere's body first in the file and second: the
/// contacts are MuJoCo's within 1e-12 (`mjraw_SphereSphere`,
/// `mjraw_SphereCapsule`, `mjc_SphereCylinder`,
/// `engine_collision_primitive.c:245-385`; `mjraw_SphereBox`,
/// `engine_collision_box.c:39-92`), at named poses on the cases those
/// functions treat apart (coincident centres, upright and turned, whose
/// normal is the cross of the two z axes or x; a centre on a capsule's or a
/// cylinder's axis, inside it, where the cylinder takes the side or the cap
/// by the shallower; inside a box, out through the nearest face; a
/// cylinder's side, either cap and rim; shallow overlaps, each contact
/// midway between the surfaces; a sphere exactly touching a sphere and a
/// box's face, listed at distance 0, MuJoCo's tests being inclusive), at
/// edge poses on the functions' exact boundaries (a centre on a cylinder's
/// rim, equidistant from its side and a cap, at a flat cylinder's centre,
/// touching a cap or at the margin exactly; on a box's edge or corner, at a
/// tie inside it, 1e-16 outside it; size ratios of 100; a capsule of
/// half-length 1e-300) and at 40 seeded random poses per pair, a sixth of
/// them degenerate, at geom margins 0 and 0.02.
#[test]
fn sphere_pairs_collide_as_mujoco() {
    pairs_collide_as_mujoco(&golden()["sphere_pairs"]);
}

/// A golden case of named and random poses of two free bodies
/// (`sphere_pairs`, `capsule_pairs`): every pose's contacts are MuJoCo's
/// ([`contacts_as_mujoco`]).
fn pairs_collide_as_mujoco(case: &Value) {
    for (name, c) in case["named"].as_object().expect("named") {
        let model: Model = load_model(c["xml"].as_str().expect("xml")).expect("load");
        contacts_as_mujoco(name, &model, &floats(&c["qpos"]), &c["contacts"]);
    }
    for (other, set) in case["random"].as_object().expect("random") {
        let models: std::collections::BTreeMap<&str, Model> = set["xml"]
            .as_object()
            .expect("xml")
            .iter()
            .map(|(variant, xml)| {
                let model = load_model(xml.as_str().expect("xml")).expect("load");
                (variant.as_str(), model)
            })
            .collect();
        for (k, pose) in set["poses"].as_array().expect("poses").iter().enumerate() {
            let variant = pose["variant"].as_str().expect("variant");
            contacts_as_mujoco(
                &format!("{other} pose {k} ({variant})"),
                &models[variant],
                &floats(&pose["qpos"]),
                &pose["contacts"],
            );
        }
    }
}

/// A capsule against a capsule and a box, each pair on two free bodies, the
/// capsule's body first in the file and second: the contacts are MuJoCo's
/// within 1e-12 (`mjraw_CapsuleCapsule`,
/// `engine_collision_primitive.c:389-481`; `mjraw_CapsuleBox`,
/// `engine_collision_box.c:119-591`), at named poses on the cases those
/// functions treat apart (capsules end to end and side by side, where the
/// parallel axes give two contacts, the same point twice end to end; offset
/// along their axes; crossing through one point, whose normal is the cross
/// of the two axes, and apart; in a T; skew; a capsule lying on a box's
/// face, standing on it, tilted on it, across an edge, along one, which
/// gives a second contact, over a corner, inside the box, over a turned
/// box, an end inside a box of half-size 1, where MuJoCo takes no second
/// point), at edge poses on the functions' exact boundaries (a capsule in a
/// box's face plane, a short one exactly in it, one wholly inside, an end
/// exactly on a side plane, along an edge line; capsules antiparallel,
/// nearly parallel, crossing at an end, a half-length of 1e-300) and at 40
/// seeded random poses per pair, a sixth of them with the axis along the
/// other capsule's or a box axis, at geom margins 0 and 0.02.
#[test]
fn capsule_pairs_collide_as_mujoco() {
    pairs_collide_as_mujoco(&golden()["capsule_pairs"]);
}

/// A box against a box on two free bodies, the first box's body first in
/// the file and second: the contacts are MuJoCo's within 1e-12
/// (`mjc_BoxBox`, `engine_collision_box.c:604-1339`, and the driver's
/// box–box filter, `engine_collision_driver.c:1543-1612`), at named poses
/// (stacked aligned, turned 45° and tilted; an edge on a face, an edge
/// across an edge, a corner on a face; side by side; one inside the other;
/// touching exactly, four contacts at distance 0; equal aligned boxes whose
/// x and y face axes tie exactly, where MuJoCo takes x; a deep overlap of
/// equally turned boxes whose every contact the filter drops, book Q74), at
/// edge poses at geom margins 0 and 0.02 (equal boxes tied on two and three
/// axes, vertex to vertex; an edge parallel to a face, edges crossing and
/// touching; a needle turned 90° across an edge; a corner on and into an
/// edge, into a corner and down onto a face; plates and slats crossed; an
/// edge, a corner, crossed edges and a vertex near touching), at
/// four poses of one cube against another's corner at margin 0.01, and at
/// 40 seeded random poses, a sixth of them turned alike or a quarter turn
/// apart, at geom margins 0 and 0.02.
#[test]
fn box_pairs_collide_as_mujoco() {
    pairs_collide_as_mujoco(&golden()["box_pairs"]);
}

/// Census document e6ea68ac, two equal boxes overlapping and aligned, after
/// two steps of the census's first excitation: MuJoCo's `mjc_BoxBox` lists 4
/// contacts there and 5 with the first box's quaternion's w one ulp lower,
/// and so does ours, each contact within 1e-12 (registry `D-BOXBOX-TIE`: our
/// state there is ulps from MuJoCo's, so the census verdict falls at the
/// third step).
#[test]
fn box_box_contact_count_turns_on_one_ulp_as_mujocos() {
    let case = &golden()["box_box_tie"];
    let model: Model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    // The contacts are the pose's; `contacts_as_mujoco` forwards from `qpos`.
    for (name, state) in case["states"].as_object().expect("states") {
        contacts_as_mujoco(name, &model, &floats(&state["qpos"]), &state["contacts"]);
    }
}
