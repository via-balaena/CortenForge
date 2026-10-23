//! The touch sensor against MuJoCo 3.5.0: for each contact of its site's
//! body, in contact order, MuJoCo adds the normal force `mj_contactForce`
//! decodes for that contact (a pyramidal contact's facets summed first,
//! `mju_decodePyramid`), skipping one not above 0 and one whose ray, from
//! the contact along the normal away from the sensor's body, misses the
//! site's zone (`mju_rayGeom`, `engine_sensor.c:887-930`).
//!
//! The golden is `assets/golden/touch/touch.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_touch_reference.py`, which describes its
//! cases).

use serde_json::Value;
use sim_core::sensor::mj_sensor_acc;
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/touch/touch.json"))
        .expect("touch golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

/// A free box resting on a plane (four contacts) beside a resting sphere
/// (one), each with a touch sensor on a site enclosing it, at condim 1, 3,
/// 4 and 6 under the pyramidal cone and condim 3 under the elliptic one: at
/// 20 of MuJoCo's states, with MuJoCo's `efc_force` written in place of the
/// solve's and the acceleration sensors rerun, both readings are MuJoCo's
/// bit for bit, so the sensor is compared apart from the solve; and at a
/// 21st, the sphere's contact rows' forces negated, whose normal force below
/// 0 MuJoCo does not add (its touch reads 0). Before, the
/// sensor added each pyramidal facet to its running total: at 268 such
/// states, the box's reading differed at 129, 156 and 169 for condim 3, 4
/// and 6, and at none for condim 1 or the elliptic cone.
#[test]
fn touch_sums_each_contact_as_mujoco() {
    let g = golden();
    for (name, case) in g["cases"].as_object().expect("cases") {
        readings_are_mujocos(name, case);
    }
}

/// Touch sensors whose zones take some of their bodies' contacts and not
/// others (the golden's `zones`): on a sphere resting on a plane, sites
/// enclosing it, about its contact, below it, beside the ray, turned, and an
/// ellipsoid flat across the ray by its size; two on the plane's body; under
/// part of a box's four contacts; on a sphere and a box it rests on, where
/// the box's ray turns. At each of MuJoCo's states, with its `efc_force`,
/// every reading is MuJoCo's bit for bit, 0 where no ray meets the zone.
#[test]
fn touch_takes_only_the_contacts_whose_rays_meet_its_zone() {
    let g = golden();
    for (name, case) in g["zones"].as_object().expect("zones") {
        readings_are_mujocos(name, case);
    }
}

/// The zone test at its edges (the golden's `zone_edges`), each state's
/// `efc_force` written over MuJoCo's: a contact on a sphere zone's surface
/// and a box zone's face (its ray meets them at distance 0, which counts);
/// normal forces of 1e-16 and 1e-15, below and at mjMINVAL, where
/// `mju_normalize3` takes the ray as (1, 0, 0) and as the normal; and an
/// explicit pair of two geoms of the sensor's body, which is then the
/// contact's second side as well as its first and turns the ray. Every
/// reading MuJoCo's, bit for bit.
#[test]
fn touch_at_the_edges_of_its_zone() {
    let g = golden();
    for (name, case) in g["zone_edges"].as_object().expect("zone edges") {
        readings_are_mujocos(name, case);
    }
}

/// At each of the case's MuJoCo states: the contacts and rows MuJoCo's, and
/// with MuJoCo's `efc_force` written in place of the solve's (and each
/// contact's position, where the golden writes one) and the acceleration
/// sensors rerun, every reading MuJoCo's bit for bit.
fn readings_are_mujocos(name: &str, case: &Value) {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    for state in case["states"].as_array().expect("states") {
        let step = &state["step"];
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&state["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&state["qvel"]));
        data.qacc_warmstart
            .as_mut_slice()
            .copy_from_slice(&floats(&state["qacc_warmstart"]));
        data.time = state["time"].as_f64().expect("time");
        data.forward(&model).expect("forward");
        let force = floats(&state["efc_force"]);
        assert_eq!(
            data.contacts.len(),
            usize::try_from(state["ncon"].as_u64().expect("ncon")).expect("ncon"),
            "{name} step {step}: contacts"
        );
        assert_eq!(
            data.efc_force.len(),
            force.len(),
            "{name} step {step}: rows"
        );
        data.efc_force.as_mut_slice().copy_from_slice(&force);
        if let Some(positions) = state["contact_pos"].as_array() {
            for (contact, pos) in data.contacts.iter_mut().zip(positions) {
                contact.pos = nalgebra::Vector3::from_column_slice(&floats(pos));
            }
        }
        mj_sensor_acc(&model, &mut data);
        let want = floats(&state["sensordata"]);
        assert_eq!(data.sensordata.len(), want.len(), "{name}: sensors");
        for (k, (a, b)) in data.sensordata.iter().zip(&want).enumerate() {
            assert!(
                a.to_bits() == b.to_bits(),
                "{name} step {step}: touch {k}: {a}, MuJoCo {b}"
            );
        }
    }
}

/// Each site's size, position and orientation as MuJoCo compiles them (the
/// golden's `site_sizes`; a touch sensor's zone is its site): the size
/// 0.005 in each place, then the numbers its default classes write, the
/// parent's first, then its own, each in the first places; a segment
/// (`fromto`) on a capsule, cylinder, ellipsoid or box giving the position,
/// the orientation and the length. Sizes and positions bit for bit; an
/// orientation, which `mjuu_z2quat` takes through `atan2`, `cos` and `sin`,
/// within 1e-15 of MuJoCo's.
#[test]
fn site_sizes_are_mujocos() {
    let g = golden();
    let case = &g["site_sizes"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let want = case["site_size"].as_object().expect("site sizes");
    assert_eq!(want.len(), model.nsite);
    for (name, size) in want {
        let site = model.site_id(name).expect("a site of that name");
        let ours = model.site_size[site].map(f64::to_bits);
        let theirs: Vec<u64> = floats(size).into_iter().map(f64::to_bits).collect();
        assert_eq!(ours.as_slice(), theirs.as_slice(), "{name}: size");
        let ours = model.site_pos[site].map(f64::to_bits);
        let theirs: Vec<u64> = floats(&case["site_pos"][name])
            .into_iter()
            .map(f64::to_bits)
            .collect();
        assert_eq!(ours.as_slice(), theirs.as_slice(), "{name}: pos");
        let q = model.site_quat[site];
        let ours = [q.w, q.i, q.j, q.k];
        let theirs = floats(&case["site_quat"][name]);
        for (a, b) in ours.iter().zip(&theirs) {
            assert!(
                (a - b).abs() <= 1e-15,
                "{name}: quat {ours:?}, MuJoCo {theirs:?}"
            );
        }
    }
}

/// A site's `fromto` where MuJoCo 3.5.0's compiler refuses it
/// (`mjCSite::Compile`, `user_objects.cc:3939-3955`), measured on the
/// oracle: on a sphere; beside a `pos` other than 0, the site's own or its
/// class's; with its two points too close. Beside a `pos` of 0 it loads.
#[test]
fn a_site_fromto_is_refused_where_mujoco_refuses_it() {
    let doc = |site: &str, default: &str| {
        format!(
            r#"<mujoco><default>{default}</default><worldbody><body name="b"><freejoint/>
               <geom size="0.1"/>{site}</body></worldbody></mujoco>"#
        )
    };
    let refused = [
        doc(
            r#"<site type="sphere" size="0.02" fromto="0 0 0 0 0 0.4"/>"#,
            "",
        ),
        doc(
            r#"<site type="capsule" size="0.02" pos="0 0 0.1" fromto="0 0 0 0 0 0.4"/>"#,
            "",
        ),
        doc(
            r#"<site class="c" fromto="0 0 0 0 0 0.4"/>"#,
            r#"<default class="c"><site type="capsule" size="0.02" pos="0.1 0 0"/></default>"#,
        ),
        doc(
            r#"<site type="capsule" size="0.02" fromto="0 0 0 0 0 1e-15"/>"#,
            "",
        ),
    ];
    for xml in &refused {
        assert!(load_model(xml).is_err(), "loads: {xml}");
    }
    let zero_pos = doc(
        r#"<site type="capsule" size="0.02" pos="0 0 0" fromto="0 0 0 0 0 0.4"/>"#,
        "",
    );
    let model = load_model(&zero_pos).expect("a pos of 0 beside fromto loads");
    assert_eq!(model.site_pos[0], nalgebra::Vector3::new(0.0, 0.0, 0.2));
}
