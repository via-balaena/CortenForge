//! A body's geoms against a flex, against MuJoCo 3.5.0.
//!
//! MuJoCo pairs a body with a flex in its broadphase, descends the two
//! hierarchies together (a world-welded body's planes first against every
//! vertex) or, without the midphase, takes each geom against every element,
//! and collides a geom with an element by type: raw functions for a sphere,
//! a capsule or a box against a cable's element or a triangle, a height
//! field's prisms, else the convex route (`engine_collision_driver.c:
//! 264-482`, `:622-847`, `:1658-1710`, `:1866-1998`). The golden is
//! `assets/golden/flex/flex_rigid.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_flex_rigid_reference.py`, which describes its
//! cases). A signed distance field is sim-core's own route (registry
//! `D-FLEX-SDF`), tested against the sphere it samples.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde_json::Value;
use sim_core::{DISABLE_ISLAND, Data, GeomType, Model, SdfGrid, ShapeSphere, SleepState};
use sim_mjcf::load_model;

use super::flex_golden::{differences, usizes};

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_rigid.json"))
        .expect("flex rigid golden parses")
}

/// Each case of the golden as [`differences`] compares it.
#[test]
fn a_body_s_geoms_collide_with_a_flex_as_mujoco() {
    let off = differences(&golden());
    assert!(
        off.is_empty(),
        "{} values differ:\n{}",
        off.len(),
        off.join("\n")
    );
}

/// A cloth of nine vertices 5 mm over a world plane, its first vertex pinned,
/// at the dense size: the pinned vertex's contact with the plane has no dof
/// on either side, so with islands its rows are in no island (MuJoCo stops
/// on such a row: registry `D-FLEX-NO-TREE`). Their Jacobian is zero, so their force
/// cannot depend on the other rows': each takes the force and state the
/// whole problem's solve (islands disabled) gives it.
#[test]
fn rows_in_no_island_take_the_whole_solve_s_force() {
    let model = load_model(
        r#"<mujoco><option><flag equality="disable"/></option>
          <worldbody><geom type="plane" size="1 1 0.1"/></worldbody>
          <deformable><flex name="c" dim="2" radius="0.01" mass="0.9">
            <contact selfcollide="none"/>
            <vertex pos="0 0 0.005  0.05 0 0.005  0.1 0 0.005  0 0.05 0.005  0.05 0.05 0.005
                         0.1 0.05 0.005  0 0.1 0.005  0.05 0.1 0.005  0.1 0.1 0.005"/>
            <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
            <pin id="0"/>
          </flex></deformable>
        </mujoco>"#,
    )
    .expect("load");
    assert!(!model.is_sparse(), "the dense size");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let outside: Vec<usize> = (0..data.efc_type.len())
        .filter(|&row| data.efc_island[row] < 0)
        .collect();
    assert!(
        outside.iter().any(|&row| data.efc_force[row] != 0.0),
        "rows in no island, one with a force"
    );

    let mut whole_model = model.clone();
    whole_model.disableflags |= DISABLE_ISLAND;
    let mut whole = whole_model.make_data();
    whole
        .forward(&whole_model)
        .expect("forward without islands");
    assert_eq!(whole.efc_type, data.efc_type, "the same rows");
    for &row in &outside {
        assert_eq!(
            data.efc_force[row].to_bits(),
            whole.efc_force[row].to_bits(),
            "row {row}'s force"
        );
        assert_eq!(
            data.efc_state[row], whole.efc_state[row],
            "row {row}'s state"
        );
    }
}

/// A box of 0.1 kg asleep (`sleep="init"`) on the floor, a flex triangle of
/// 0.375 kg falling onto it from 2 mm (registry `D-FLEX-WAKE`): the
/// triangle's contact wakes the box, as a geom's would, and after 200 steps
/// the triangle rests on its top. MuJoCo 3.5.0 wakes no tree for a contact
/// with a flex side (`mj_wakeCollision`, `engine_sleep.c:290-293`): with the
/// box declared after the flex the box stays asleep and the triangle passes
/// through it, with the box first MuJoCo stops (`gen_flex_rigid_reference.py`'s
/// `mujoco_stops` asserts both).
#[test]
fn a_flex_contact_wakes_a_sleeping_body() {
    let model = load_model(
        r#"<mujoco><option timestep="0.002"><flag equality="disable" sleep="enable"/></option>
          <worldbody>
            <geom type="plane" size="1 1 0.1"/>
            <body name="b" pos="0 0 0.05" sleep="init"><freejoint/>
              <inertial pos="0 0 0" mass="0.1" diaginertia="2e-5 3e-5 4e-5"/>
              <geom type="box" size="0.05 0.05 0.05"/>
            </body>
          </worldbody>
          <deformable><flex name="t" dim="2" radius="0.01" mass="0.375">
            <contact selfcollide="none" internal="false"/>
            <vertex pos="-0.02 -0.02 0.112  0.02 -0.02 0.112  0 0.02 0.112"/>
            <element data="0 1 2"/>
          </flex></deformable>
        </mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let b = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("b"))
        .expect("the box's body");
    assert_eq!(
        data.body_sleep_state[b],
        SleepState::Asleep,
        "asleep at first"
    );
    for step in 1..=200 {
        data.step(&model)
            .unwrap_or_else(|e| panic!("step {step}: {e:?}"));
    }
    assert_eq!(data.body_sleep_state[b], SleepState::Awake, "woken");
    let top = data.xpos[b].z + 0.05;
    let lowest = data
        .flexvert_xpos
        .iter()
        .map(|x| x.z)
        .fold(f64::INFINITY, f64::min);
    assert!(
        lowest > top,
        "resting on the box: lowest vertex {lowest}, top {top}"
    );
}

/// A cloth of nine vertices (a 3×3 grid, 5 cm apart, radius 1 cm, margin
/// 5 mm) whose middle sits 5 cm above the centre of a world sphere of radius
/// 6.5 cm, the sphere's own attributes `bits`; a far box of the world's
/// lets the world body pair with the flex whatever the sphere's bits.
fn cloth_over_a_sphere(midphase: bool, bits: &str) -> (Model, Data) {
    let flag = if midphase {
        ""
    } else {
        r#"<flag midphase="disable"/>"#
    };
    let xml = format!(
        r#"<mujoco>
          <option gravity="0 0 0">{flag}</option>
          <worldbody>
            <geom type="sphere" size="0.065" pos="0 0 -0.05" {bits}/>
            <geom type="box" size="0.01 0.01 0.01" pos="1 1 1"/>
          </worldbody>
          <deformable>
            <flex name="c" dim="2" radius="0.01">
              <contact selfcollide="none" internal="false" margin="0.005"/>
              <vertex pos="-0.05 -0.05 0  0 -0.05 0  0.05 -0.05 0  -0.05 0 0  0 0 0  0.05 0 0
                           -0.05 0.05 0  0 0.05 0  0.05 0.05 0"/>
              <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
            </flex>
          </deformable>
        </mujoco>"#
    );
    let mut model = load_model(&xml).expect("load");
    let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 0.065, 32, 0.02));
    model
        .shape_data
        .push(Arc::new(ShapeSphere::new(grid, 0.065)));
    model.nshape = model.shape_data.len();
    model.geom_type[0] = GeomType::Sdf;
    model.geom_shape[0] = Some(model.nshape - 1);
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    (model, data)
}

/// A signed distance field against a flex (registry `D-FLEX-SDF`): MuJoCo
/// collides the field's mesh hull with each element through the convex
/// route; sim-core's field has no hull, so each vertex, a sphere of the
/// flex's radius, meets the field within the margin. A cloth over a field
/// sphere: a contact for each vertex within the radius and margin of the
/// surface (its middle, 1.5 cm in, and the four at its edges' middles,
/// 0.57 cm off), none at its corners (2.2 cm off), each listing the field
/// and the vertex, none an element; its distance the vertex's from the
/// sphere less the radius (the margin not in it), to the field's grid; the
/// same list, bit for bit, with the midphase on and off; none where the
/// field's bitmask misses the flex's.
#[test]
fn a_signed_distance_field_meets_each_flex_vertex() {
    let (model, data) = cloth_over_a_sphere(true, "");
    let verts: Vec<usize> = data.contacts.iter().filter_map(|c| c.vert[1]).collect();
    assert_eq!(verts, [1, 3, 4, 5, 7], "the vertices within reach");
    for c in &data.contacts {
        assert_eq!(c.geom, [Some(0), None], "the field first");
        assert_eq!(c.flex, [None, Some(0)]);
        assert_eq!(c.elem, [None, None], "no element");
        let v = data.flexvert_xpos[model.flex_vertadr[0] + c.vert[1].expect("a vertex")];
        let want = (v - Vector3::new(0.0, 0.0, -0.05)).norm() - 0.065 - 0.01;
        assert!(
            (-c.depth - want).abs() < 2e-4,
            "dist {}, the sphere's {want}",
            -c.depth
        );
    }
    let (_, all_to_all) = cloth_over_a_sphere(false, "");
    let key = |d: &Data| -> Vec<(Option<usize>, u64, [u64; 3])> {
        d.contacts
            .iter()
            .map(|c| (c.vert[1], c.depth.to_bits(), c.pos.map(f64::to_bits).into()))
            .collect()
    };
    assert_eq!(key(&data), key(&all_to_all), "the midphase on and off");
    for midphase in [true, false] {
        let (_, missed) = cloth_over_a_sphere(midphase, r#"contype="2" conaffinity="2""#);
        assert!(
            missed.contacts.is_empty(),
            "midphase {midphase}: the bitmask"
        );
    }
}

/// A signed distance field on a body asleep (`sleep="init"`) under a cloth
/// falling onto it from 5 mm beyond reach (registry `D-FLEX-WAKE`): the
/// first contacts list a vertex, not an element, and wake the body in the
/// step that makes them.
#[test]
fn a_flex_vertex_contact_wakes_a_sleeping_field() {
    let mut model = load_model(
        r#"<mujoco><option timestep="0.002"><flag equality="disable" sleep="enable"/></option>
          <worldbody>
            <body name="b" pos="0 0 -0.085" sleep="init"><freejoint/>
              <geom type="sphere" size="0.065"/>
            </body>
          </worldbody>
          <deformable><flex name="c" dim="2" radius="0.01">
            <contact selfcollide="none" internal="false" margin="0.005"/>
            <vertex pos="-0.05 -0.05 0  0 -0.05 0  0.05 -0.05 0  -0.05 0 0  0 0 0  0.05 0 0
                         -0.05 0.05 0  0 0.05 0  0.05 0.05 0"/>
            <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
          </flex></deformable>
        </mujoco>"#,
    )
    .expect("load");
    let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 0.065, 32, 0.02));
    model
        .shape_data
        .push(Arc::new(ShapeSphere::new(grid, 0.065)));
    model.nshape = model.shape_data.len();
    model.geom_type[0] = GeomType::Sdf;
    model.geom_shape[0] = Some(model.nshape - 1);
    let mut data = model.make_data();
    let b = model.geom_body[0];
    assert_eq!(
        data.body_sleep_state[b],
        SleepState::Asleep,
        "asleep at first"
    );
    data.forward(&model).expect("forward");
    for step in 1..=100 {
        data.step(&model)
            .unwrap_or_else(|e| panic!("step {step}: {e:?}"));
        if !data.contacts.is_empty() {
            break;
        }
    }
    assert!(
        !data.contacts.is_empty() && data.contacts.iter().all(|c| c.vert[1].is_some()),
        "vertex contacts"
    );
    assert_eq!(data.body_sleep_state[b], SleepState::Awake, "woken");
}

/// A geom turned by a NaN against a flex: each call returns, with the
/// midphase on and off, for a height field, a box, an ellipsoid, a cylinder
/// and a signed distance field (which collides as nothing there, its turn
/// not a quaternion).
#[test]
fn a_non_finite_turn_against_a_flex_returns() {
    use nalgebra::{Quaternion, UnitQuaternion};
    let cloth = r#"<deformable><flex name="c" dim="2" radius="0.01">
      <contact selfcollide="none" internal="false"/>
      <vertex pos="0 0 0.005  0.05 0 0.005  0.1 0 0.005  0 0.05 0.005  0.05 0.05 0.005  0.1 0.05 0.005
                   0 0.1 0.005  0.05 0.1 0.005  0.1 0.1 0.005"/>
      <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
    </flex></deformable>"#;
    let geoms = [
        (
            "hfield",
            r#"<asset><hfield name="h" nrow="3" ncol="3" size="1 1 0.05 0.05" elevation="1 1 1 1 1 1 1 1 1"/></asset>"#,
            r#"<geom type="hfield" hfield="h"/>"#,
        ),
        (
            "box",
            "",
            r#"<geom type="box" size="1 1 0.01" pos="0 0 -0.005"/>"#,
        ),
        (
            "ellipsoid",
            "",
            r#"<geom type="ellipsoid" size="0.1 0.1 0.02" pos="0.05 0.05 -0.015"/>"#,
        ),
        (
            "cylinder",
            "",
            r#"<geom type="cylinder" size="0.1 0.02" pos="0.05 0.05 -0.015"/>"#,
        ),
        (
            "sdf",
            "",
            r#"<geom type="sphere" size="0.1" pos="0.05 0.05 -0.09"/>"#,
        ),
    ];
    for (name, asset, geom) in geoms {
        for midphase in [true, false] {
            let flag = if midphase {
                ""
            } else {
                r#"midphase="disable""#
            };
            let xml = format!(
                r#"<mujoco><option><flag equality="disable" {flag}/></option>{asset}
                   <worldbody>{geom}</worldbody>{cloth}</mujoco>"#
            );
            let mut model = load_model(&xml).expect("load");
            if name == "sdf" {
                let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 0.1, 16, 0.02));
                model.shape_data.push(Arc::new(ShapeSphere::new(grid, 0.1)));
                model.nshape = model.shape_data.len();
                model.geom_type[0] = GeomType::Sdf;
                model.geom_shape[0] = Some(model.nshape - 1);
            }
            let mut data = model.make_data();
            data.forward(&model).expect("forward, unturned");
            assert!(
                data.ncon > 0,
                "{name} midphase {midphase}: the cloth touches the geom"
            );
            model.geom_quat[0] =
                UnitQuaternion::new_unchecked(Quaternion::new(f64::NAN, 0.0, 0.0, 0.0));
            model.geom_sameframe[0] = Default::default();
            // Each call returns; its result is not asserted.
            let _ = data.forward(&model);
            if name == "sdf" {
                assert_eq!(data.ncon, 0, "{name} midphase {midphase}: nothing");
            }
        }
    }
}

/// A flex's `<contact passive>` (registry `D-FLEX-PASSIVE-READ`): MuJoCo
/// 3.5.0's reader overwrites the flag with a stale variable whenever the
/// attribute is given (`xml_native_reader.cc:1457-1523`), here the
/// `internal` flag's value, so the golden's passive cases,
/// `internal="false" passive="true"`, read as not passive there (the
/// generator sets the flag on MuJoCo's compiled model). sim-mjcf reads the
/// attribute as written.
#[test]
fn a_flex_s_passive_attribute_reads_as_written() {
    let golden = golden();
    let cases: Vec<&Value> = golden["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .filter(|c| c.get("flex_passive_read").is_some())
        .collect();
    assert!(!cases.is_empty(), "the golden's passive cases");
    for case in cases {
        let name = case["name"].as_str().expect("name");
        let model = load_model(case["ours"].as_str().expect("ours")).expect("load");
        assert_eq!(
            model.flex_passive,
            [false, true],
            "{name}: ours, as written"
        );
        assert_eq!(
            usizes(&case["flex_passive_read"]),
            [0, 0],
            "{name}: MuJoCo's reader, its internal flag"
        );
    }
}
