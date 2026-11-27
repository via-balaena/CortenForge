//! Integration test: a height field's scene ray as MuJoCo 3.5.0's `mj_ray`
//! casts it.
//!
//! `raycast_scene` casts a height-field geom through MuJoCo's `mj_rayHfield`
//! (sim-core's `sensor::ray_mesh` port): the field centred on its geom, its
//! extent and top from the asset's `size`, with MuJoCo's normal (ledger L280,
//! where it read the grid from the geom's origin). Checked here:
//! 1. MJCF with an `<hfield>` asset builds a Model with the field's data.
//! 2. Rays MuJoCo cast meet each field as MuJoCo's do: the geom, the
//!    distance and the normal.
//! 3. A field's data replaced in code (`model.hfield_data[0]`, as the
//!    `raycasting/heightfield` example replaces it) is cast at the asset's
//!    size, centred on its geom.

use std::sync::Arc;

use nalgebra::{Point3, UnitVector3, Vector3};
use sim_core::{HeightFieldData, raycast_scene};
use sim_mjcf::load_model;

const HFIELD_MJCF: &str = r#"
<mujoco model="hfield-raycast">
  <option gravity="0 0 0" timestep="0.002"/>
  <asset>
    <hfield name="terrain" nrow="4" ncol="4" size="4 4 0.5 0.1"
            elevation="0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0"/>
  </asset>
  <worldbody>
    <geom name="terrain" type="hfield" hfield="terrain"/>
  </worldbody>
</mujoco>
"#;

/// MJCF with hfield asset loads successfully and model has hfield data.
#[test]
fn hfield_mjcf_loads() {
    let model = load_model(HFIELD_MJCF).expect("MJCF should parse");

    assert_eq!(model.nhfield, 1, "expected 1 heightfield asset");
    assert!(
        !model.hfield_data.is_empty(),
        "hfield_data should be populated"
    );

    // The geom should reference this hfield
    let hfield_geom = model
        .geom_hfield
        .iter()
        .position(|h| h.is_some())
        .expect("at least one geom should reference an hfield");
    assert_eq!(
        model.geom_type[hfield_geom],
        sim_core::GeomType::Hfield,
        "geom type should be Hfield"
    );
}

/// The documents MuJoCo's rays were cast on: ledger L280's flat field of
/// half-extents 1 × 1; a field turned and moved on the world; a field turned
/// in a turned and moved body; two fields, the second turned a quarter, beside
/// a sphere.
const DOCS: &[(&str, &str)] = &[
    (
        "l280_flat",
        r#"<mujoco model="l280_flat">
  <asset>
    <hfield name="h" nrow="3" ncol="3" size="1 1 0.5 0.1" elevation="0 0 0 0 0 0 0 0 0"/>
  </asset>
  <worldbody>
    <geom type="hfield" hfield="h"/>
  </worldbody>
</mujoco>"#,
    ),
    (
        "turned_6x6",
        r#"<mujoco model="turned_6x6">
  <asset>
    <hfield name="h" nrow="6" ncol="6" size="1.5 1 0.6 0.15" elevation="0.1 0.7 0.3 0.9 0.5 0.1 0.5 0.1 0.7 0.3 0.9 0.5 0.9 0.5 0.1 0.7 0.3 0.9 0.3 0.9 0.5 0.1 0.7 0.3 0.7 0.3 0.9 0.5 0.1 0.7 0.1 0.7 0.3 0.9 0.5 0.1"/>
  </asset>
  <worldbody>
    <geom type="hfield" hfield="h" pos="0.3 -0.2 0.1" euler="20 -10 35"/>
  </worldbody>
</mujoco>"#,
    ),
    (
        "on_body",
        r#"<mujoco model="on_body">
  <asset>
    <hfield name="h" nrow="5" ncol="4" size="1 1.5 0.3 0.1" elevation="0.1 0.7 0.3 0.9 0.5 0.1 0.7 0.3 0.9 0.5 0.1 0.7 0.3 0.9 0.5 0.1 0.7 0.3 0.9 0.5"/>
  </asset>
  <worldbody>
    <body pos="1 2 0.5" euler="0 15 -30"><geom type="hfield" hfield="h" pos="0.2 0 -0.1" euler="5 0 0"/></body>
  </worldbody>
</mujoco>"#,
    ),
    (
        "two_fields_sphere",
        r#"<mujoco model="two_fields_sphere">
  <asset>
    <hfield name="a" nrow="4" ncol="4" size="1 1 0.5 0.1" elevation="0.1 0.7 0.3 0.9 0.5 0.1 0.7 0.3 0.9 0.5 0.1 0.7 0.3 0.9 0.5 0.1"/>
    <hfield name="b" nrow="3" ncol="5" size="1 0.8 0.3 0.1" elevation="0.1 0.7 0.3 0.9 0.5 0.5 0.1 0.7 0.3 0.9 0.9 0.5 0.1 0.7 0.3"/>
  </asset>
  <worldbody>
    <geom type="hfield" hfield="a" pos="-1.2 0 0"/><geom type="hfield" hfield="b" pos="1.2 0 0.05" euler="0 0 90"/><geom type="sphere" size="0.2" pos="-1.2 0 1.2"/>
  </worldbody>
</mujoco>"#,
    ),
];

/// A ray MuJoCo 3.5.0 cast through `mj_ray` (no group filter, static geoms
/// included, no body excluded) on the census's oracle after `mj_forward`:
/// its start and unit direction, and MuJoCo's answer, the geom it met (−1 for
/// none), the distance (−1 for none) and the normal (zero for none), each as
/// its bits.
struct Ray {
    doc: &'static str,
    pnt: [u64; 3],
    vec: [u64; 3],
    geom: i64,
    x: u64,
    normal: [u64; 3],
}

/// Ledger L280's four rays down from z 1 over its flat field, at (−0.5, −0.5),
/// (0.5, 0.5), (1.5, 1.5) and (−1.5, −1.5): MuJoCo meets the first two at 1 and
/// misses the others; then, on the turned field, a ray from above meeting a
/// triangle, one from beside meeting a side, one from below meeting the base,
/// one from inside the top box meeting a triangle and one from above missing;
/// on the field in a body, a triangle and a side; on the two fields, a
/// triangle of each. Each the first of its kind in the start set's order
/// (Rigid-loading RL-11b2).
const RAYS: &[Ray] = &[
    // l280_flat ray 196: l280
    Ray {
        doc: "l280_flat",
        pnt: [0xbfe0000000000000, 0xbfe0000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x3ff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    },
    // l280_flat ray 197: l280
    Ray {
        doc: "l280_flat",
        pnt: [0x3fe0000000000000, 0x3fe0000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x3ff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    },
    // l280_flat ray 198: l280
    Ray {
        doc: "l280_flat",
        pnt: [0x3ff8000000000000, 0x3ff8000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: -1,
        x: 0xbff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
    },
    // l280_flat ray 199: l280
    Ray {
        doc: "l280_flat",
        pnt: [0xbff8000000000000, 0xbff8000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: -1,
        x: 0xbff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
    },
    // turned_6x6 ray 30: triangle
    Ray {
        doc: "turned_6x6",
        pnt: [0xbfe0349a38ec6222, 0xc0002fa236ae6b18, 0x3fe6132e4391aba5],
        vec: [0x3fc63a1a7e0b7388, 0x3fd58e869f82dce1, 0xbfed9d033a6cb460],
        geom: 0,
        x: 0x3ff4333331ffffff,
        normal: [0x3fe174993f013f40, 0xbfe4180af7d3467c, 0x3fe1c3738a782198],
    },
    // turned_6x6 ray 351: side
    Ray {
        doc: "turned_6x6",
        pnt: [0xbff2c39e252a8f67, 0xbff39bf360f97323, 0xbfdc67fc1381597a],
        vec: [0xbfa9a15d36c40fa6, 0x3fe4f69164fb1fff, 0x3fe81fcfa33b2a9e],
        geom: 0,
        x: 0x3fe661af5f5cf5ce,
        normal: [0xbfe9d08bcb56899a, 0xbfdf61a6a66271d9, 0xbfd51c1ba5e5816c],
    },
    // turned_6x6 ray 300: base
    Ray {
        doc: "turned_6x6",
        pnt: [0x3ff0ee9f5a32e12e, 0xbfcf91a3ee6f14bc, 0xbfeec73f286463e9],
        vec: [0xbfe550444c85e7d0, 0xbfc6a515ebf8311d, 0x3fe730210ecddf0d],
        geom: 0,
        x: 0x3ff1f1a568986b68,
        normal: [0x3fc63a1a7e0b7385, 0x3fd58e869f82dce1, 0xbfed9d033a6cb45e],
    },
    // turned_6x6 ray 400: triangle
    Ray {
        doc: "turned_6x6",
        pnt: [0x3ff1ea6ce26e1432, 0x3fb78b786bfe3374, 0x3fec00e460ca064f],
        vec: [0xbf559eb68c6f1648, 0xbfeed5241a75a760, 0xbfd12101d22333b2],
        geom: 0,
        x: 0x3fa118e3ada3209b,
        normal: [0x3fe174993c4fe98a, 0xbfe4180af266d89a, 0x3fe1c373934017f9],
    },
    // turned_6x6 ray 201: miss
    Ray {
        doc: "turned_6x6",
        pnt: [0x3ff39559ce05c375, 0x3fcc5afff1576f94, 0x3ffa72e586cfbef6],
        vec: [0x3fefb78c4abc007a, 0xbf9a2a3a26b88610, 0xbfc0ab10353d7f70],
        geom: -1,
        x: 0xbff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
    },
    // on_body ray 30: triangle
    Ray {
        doc: "on_body",
        pnt: [0xbfb35f5aa7d43e70, 0x3ff1c1e2e5a78a50, 0x3ff981f11aef0dba],
        vec: [0xbfcb9d6818cd9ca1, 0x3fb35298f381f53c, 0xbfef272639f5605b],
        geom: 0,
        x: 0x3fe8ae147aa3d70e,
        normal: [0x3fa04757ebe21f54, 0x3fcfaca00fe63822, 0x3feefcf2a9b89ccb],
    },
    // on_body ray 353: side
    Ray {
        doc: "on_body",
        pnt: [0x3fee8f54ef67b54d, 0x400d5594507b4142, 0x3fe0e96aec2b8bf6],
        vec: [0x3fe1ec239b63a97f, 0xbfea740297955837, 0x3fabca8f884d62ac],
        geom: 0,
        x: 0x3fb6ca94ce9577c6,
        normal: [0xbfeac4bdd6e3fd73, 0x3fe0000000000001, 0x3fccb0bf0b6b7109],
    },
    // two_fields_sphere ray 30: triangle
    Ray {
        doc: "two_fields_sphere",
        pnt: [0xc000cccccccccccd, 0xbfeccccccccccccd, 0x3ff3333333333333],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x3ff0e66661cccccc,
        normal: [0x3fd3e1fcd84887b8, 0xbfddd2fb6c30c594, 0x3fea82a6aabb57a0],
    },
    // two_fields_sphere ray 480: triangle
    Ray {
        doc: "two_fields_sphere",
        pnt: [0x3ffeb851eb851eb8, 0xbfecccccccccccce, 0x3ff0cccccccccccd],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 1,
        x: 0x3fe7d70a3c7ae148,
        normal: [0xbfc6a0379dd941e2, 0x3fd219c5cf79e9b6, 0x3fee2af49621195a],
    },
];

/// A run's values against MuJoCo's: bit for bit on macOS arm64, where the
/// oracle ran; elsewhere within 1e-9 of the array's largest entry, as another
/// libm moves the last bits of a turned frame. A `NaN` fails either way.
fn close(ours: &[f64], mujoco: &[u64]) -> bool {
    let want: Vec<f64> = mujoco.iter().map(|&b| f64::from_bits(b)).collect();
    if ours.len() != want.len() || ours.iter().chain(&want).any(|v| v.is_nan()) {
        return false;
    }
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        return ours.iter().map(|v| v.to_bits()).eq(mujoco.iter().copied());
    }
    let scale = want.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter()
        .zip(&want)
        .all(|(a, b)| (a - b).abs() <= 1e-9 * scale)
}

/// Each of MuJoCo's rays meets the geom MuJoCo's met, at MuJoCo's distance,
/// with MuJoCo's normal, or misses as MuJoCo's does.
#[test]
fn rays_meet_height_fields_as_mujocos_mj_ray() {
    let bits3 = |b: [u64; 3]| {
        Vector3::new(
            f64::from_bits(b[0]),
            f64::from_bits(b[1]),
            f64::from_bits(b[2]),
        )
    };
    let mut bad = Vec::new();
    for (name, xml) in DOCS {
        let model = load_model(xml).expect("the document loads");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        for ray in RAYS.iter().filter(|r| r.doc == *name) {
            let pnt = Point3::from(bits3(ray.pnt));
            let dir = UnitVector3::new_unchecked(bits3(ray.vec));
            let (geom, x, normal) = raycast_scene(&model, &data, pnt, dir, 1e9, None, None).map_or(
                (-1, -1.0, [0.0; 3]),
                |h| {
                    let n = h.hit.normal;
                    (
                        i64::try_from(h.geom_id).expect("a geom id"),
                        h.hit.distance,
                        [n.x, n.y, n.z],
                    )
                },
            );
            if geom != ray.geom || !close(&[x], &[ray.x]) || !close(&normal, &ray.normal) {
                bad.push(format!(
                    "{name} from {pnt:?}: geom {geom} at {x:e} normal {normal:?}, MuJoCo geom {} at {:e}",
                    ray.geom,
                    f64::from_bits(ray.x)
                ));
            }
        }
    }
    assert_eq!(
        (RAYS.len(), RAYS.iter().filter(|r| r.geom >= 0).count()),
        (13, 10),
        "every ray read, and every hit"
    );
    assert!(bad.is_empty(), "{} differ:\n{}", bad.len(), bad.join("\n"));
}

/// A flat field replaced in code is cast at the asset's size, centred on its
/// geom: 32 × 32 heights of 0.2 under the asset's half-extents 4 × 4 are met
/// straight down at the field's centre and near its −x, −y corner, 2.8 below
/// a start 3 up, with the normal +z; a ray past its +x, +y corner, which the
/// grid read from the geom's origin would hold, misses.
#[test]
fn a_replaced_flat_field_is_cast_centred_at_its_assets_size() {
    let mut model = load_model(HFIELD_MJCF).expect("MJCF should parse");
    model.hfield_data[0] = Arc::new(HeightFieldData::flat(32, 32, 8.0 / 31.0, 0.2));
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let down = UnitVector3::new_normalize(-Vector3::z());
    for (x, y) in [(0.0, 0.0), (-3.5, -3.5)] {
        let hit = raycast_scene(
            &model,
            &data,
            Point3::new(x, y, 3.0),
            down,
            20.0,
            None,
            None,
        )
        .expect("the ray meets the field");
        assert!(
            (hit.hit.distance - 2.8).abs() < 1e-12,
            "({x}, {y}): distance {}",
            hit.hit.distance
        );
        assert_eq!(hit.hit.normal, Vector3::z(), "({x}, {y})");
    }
    let past = raycast_scene(
        &model,
        &data,
        Point3::new(4.5, 4.5, 3.0),
        down,
        20.0,
        None,
        None,
    );
    assert!(past.is_none(), "past the corner: {past:?}");
}

/// A sinusoidal field replaced in code, as the `raycasting/heightfield`
/// example replaces it: heights `0.25 + 0.2 sin(2πx/4) cos(2πy/4)` sampled at
/// the asset's grid, centred on the geom (`x`, `y` from −4 to 4), within its
/// top of 0.5. A ray down meets the terrain at its centre, and a quarter period
/// off it, within 0.02 of the function.
#[test]
fn a_replaced_sinusoidal_field_is_cast_centred_at_its_assets_size() {
    let mut model = load_model(HFIELD_MJCF).expect("MJCF should parse");
    let terrain = |x: f64, y: f64| {
        0.25 + 0.2
            * (std::f64::consts::TAU * x / 4.0).sin()
            * (std::f64::consts::TAU * y / 4.0).cos()
    };
    let cell = 8.0 / 31.0;
    model.hfield_data[0] = Arc::new(HeightFieldData::from_fn(32, 32, cell, |x, y| {
        terrain(x - 4.0, y - 4.0)
    }));
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let down = UnitVector3::new_normalize(-Vector3::z());
    for (x, y) in [(0.0, 0.0), (-1.0, 0.0)] {
        let hit = raycast_scene(
            &model,
            &data,
            Point3::new(x, y, 3.0),
            down,
            20.0,
            None,
            None,
        )
        .expect("the ray meets the field");
        let expected = terrain(x, y);
        assert!(
            (hit.hit.point.z - expected).abs() < 0.02,
            "({x}, {y}): z {} where the terrain is {expected}",
            hit.hit.point.z
        );
    }
}
