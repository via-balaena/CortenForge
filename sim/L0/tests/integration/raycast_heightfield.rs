//! Integration test: a height field's scene ray as MuJoCo 3.5.0's `mj_ray`
//! casts it.
//!
//! `raycast_scene` casts a height-field geom through MuJoCo's `mj_rayHfield`
//! (sim-core's `sensor::ray_mesh` port): the field centred on its geom, its
//! extent and top from the asset's `size`, with MuJoCo's normal (ledger L280,
//! where it read the grid from the geom's origin). Checked here:
//! 1. MJCF with an `<hfield>` asset builds a Model with the field's data.
//! 2. Rays MuJoCo cast meet each field as MuJoCo's do: the geom, the
//!    distance, the normal and the hit's point.
//! 3. A field's data replaced in code (`model.hfield_data[0]`, as the
//!    `raycasting/heightfield` example replaces it) is cast at the asset's
//!    size, centred on its geom, its own spacing unread, and rays meet its
//!    heights below 0 and above its top as MuJoCo's meet such data.

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
/// a sphere; a field turned a half about x, upside down; a field with one
/// raised flat cell.
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
    (
        "raised_cell",
        r#"<mujoco model="raised_cell">
  <asset>
    <hfield name="h" nrow="3" ncol="3" size="1 1 0.5 0.1" elevation="1 0 0 0 0.5 0.5 0 0.5 0.5"/>
  </asset>
  <worldbody>
    <geom type="hfield" hfield="h"/>
  </worldbody>
</mujoco>"#,
    ),
    (
        "turned_180",
        r#"<mujoco model="turned_180">
  <asset>
    <hfield name="h" nrow="5" ncol="5" size="1 1 0.4 0.1" elevation="0.1 0.7 0.3 0.9 0.5 0.5 0.1 0.7 0.3 0.9 0.9 0.5 0.1 0.7 0.3 0.3 0.9 0.5 0.1 0.7 0.7 0.3 0.9 0.5 0.1"/>
  </asset>
  <worldbody>
    <geom type="hfield" hfield="h" pos="0 0 0.5" euler="180 0 0"/>
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
/// misses the others; two rays starting on that field's surface, which MuJoCo
/// meets at −0 going down and at 0 going up. On the turned field, a ray from
/// above meeting a triangle, one from beside meeting a side, one from below
/// meeting the base, one from inside the top box meeting a triangle and one
/// from above missing; on the field in a body, a triangle and a side; on the
/// two fields, a triangle of each; on the upside-down field, two rays meeting
/// a side and one meeting the base box's side with the top box missed,
/// MuJoCo's normal holding a −0; two rays starting on the raised cell, met by
/// its triangle at 0 going down and at −0 going up. A −0 is told from 0 only
/// where `close` compares bits, on macOS arm64. Each triangle's ray meets its
/// cell at least a tenth of a cell from each of the cell's edges, its two
/// coordinates in the cell at least a tenth apart, off the diagonal the
/// cell's two triangles share, where a frame one ulp off sent an earlier
/// pinned ray to the other triangle.
const RAYS: &[Ray] = &[
    // l280_flat: L280's ray from z 1, a hit
    Ray {
        doc: "l280_flat",
        pnt: [0xbfe0000000000000, 0xbfe0000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x3ff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    },
    // l280_flat: L280's ray from z 1, a hit
    Ray {
        doc: "l280_flat",
        pnt: [0x3fe0000000000000, 0x3fe0000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x3ff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    },
    // l280_flat: L280's ray from z 1, a miss
    Ray {
        doc: "l280_flat",
        pnt: [0x3ff8000000000000, 0x3ff8000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: -1,
        x: 0xbff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
    },
    // l280_flat: L280's ray from z 1, a miss
    Ray {
        doc: "l280_flat",
        pnt: [0xbff8000000000000, 0xbff8000000000000, 0x3ff0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: -1,
        x: 0xbff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
    },
    // l280_flat: starts on the surface, down, met at -0.0
    Ray {
        doc: "l280_flat",
        pnt: [0x3fd3333333333333, 0x3fc999999999999a, 0x0000000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x8000000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    },
    // l280_flat: starts on the surface, up, met at 0.0
    Ray {
        doc: "l280_flat",
        pnt: [0x3fd3333333333333, 0x3fc999999999999a, 0x0000000000000000],
        vec: [0x3fe3333333333333, 0x0000000000000000, 0x3fe999999999999a],
        geom: 0,
        x: 0x0000000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    },
    // turned_6x6: triangle, down
    Ray {
        doc: "turned_6x6",
        pnt: [0xbfe3d212bc557c88, 0xbffdccc7e31a768b, 0x3fe744dc8075635a],
        vec: [0x3fc63a1a7e0b7388, 0x3fd58e869f82dce1, 0xbfed9d033a6cb460],
        geom: 0,
        x: 0x3ff0999998666665,
        normal: [0x3fe174993f013f40, 0xbfe4180af7d3467c, 0x3fe1c3738a782198],
    },
    // turned_6x6: side, from beside
    Ray {
        doc: "turned_6x6",
        pnt: [0xbff2c39e252a8f67, 0xbff39bf360f97323, 0xbfdc67fc1381597a],
        vec: [0xbfa9a15d36c40fa6, 0x3fe4f69164fb1fff, 0x3fe81fcfa33b2a9e],
        geom: 0,
        x: 0x3fe661af5f5cf5ce,
        normal: [0xbfe9d08bcb56899a, 0xbfdf61a6a66271d9, 0xbfd51c1ba5e5816c],
    },
    // turned_6x6: base, from below
    Ray {
        doc: "turned_6x6",
        pnt: [0x3ff0ee9f5a32e12e, 0xbfcf91a3ee6f14bc, 0xbfeec73f286463e9],
        vec: [0xbfe550444c85e7d0, 0xbfc6a515ebf8311d, 0x3fe730210ecddf0d],
        geom: 0,
        x: 0x3ff1f1a568986b68,
        normal: [0x3fc63a1a7e0b7385, 0x3fd58e869f82dce1, 0xbfed9d033a6cb45e],
    },
    // turned_6x6: triangle, from inside the top box
    Ray {
        doc: "turned_6x6",
        pnt: [0x3fd4d5d7f898b712, 0x3fe8170659944b96, 0x3fea38b20c300c0c],
        vec: [0x3fc846d68a64fbd0, 0xbfe2e64650cad1b8, 0xbfe9196a0fc43208],
        geom: 0,
        x: 0x3fbb24147cd15b25,
        normal: [0xbfea64aad6f2499e, 0xbfb1dcc00228210c, 0x3fe1f4b1884ea36d],
    },
    // turned_6x6: miss, from above
    Ray {
        doc: "turned_6x6",
        pnt: [0x3ff39559ce05c375, 0x3fcc5afff1576f94, 0x3ffa72e586cfbef6],
        vec: [0x3fefb78c4abc007a, 0xbf9a2a3a26b88610, 0xbfc0ab10353d7f70],
        geom: -1,
        x: 0xbff0000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
    },
    // on_body: triangle, down
    Ray {
        doc: "on_body",
        pnt: [0x3fb34f732d493f80, 0x3ff5e6024326316b, 0x3ff94af9eb2c851a],
        vec: [0xbfcb9d6818cd9ca1, 0x3fb35298f381f53c, 0xbfef272639f5605b],
        geom: 0,
        x: 0x3fea99999770a3db,
        normal: [0x3fa04757ebe21f54, 0x3fcfaca00fe63822, 0x3feefcf2a9b89ccb],
    },
    // on_body: side, from beside
    Ray {
        doc: "on_body",
        pnt: [0x3fee8f54ef67b54d, 0x400d5594507b4142, 0x3fe0e96aec2b8bf6],
        vec: [0x3fe1ec239b63a97f, 0xbfea740297955837, 0x3fabca8f884d62ac],
        geom: 0,
        x: 0x3fb6ca94ce9577c6,
        normal: [0xbfeac4bdd6e3fd73, 0x3fe0000000000001, 0x3fccb0bf0b6b7109],
    },
    // two_fields_sphere: triangle, down, geom 0
    Ray {
        doc: "two_fields_sphere",
        pnt: [0xc000cccccccccccd, 0xbfe6666666666666, 0x3ff3333333333333],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x3fee33332c666665,
        normal: [0x3fd3e1fcd84887b8, 0xbfddd2fb6c30c594, 0x3fea82a6aabb57a0],
    },
    // two_fields_sphere: triangle, down, geom 1
    Ray {
        doc: "two_fields_sphere",
        pnt: [0x3ffc28f5c28f5c28, 0xbfecccccccccccce, 0x3ff0cccccccccccd],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 1,
        x: 0x3fe8cccccbd70a3d,
        normal: [0xbfc6a0373eef10dd, 0x3fd219c614598ba7, 0x3fee2af4903f54f5],
    },
    // turned_180: side, from beside, a -0 in its normal
    Ray {
        doc: "turned_180",
        pnt: [0xbff4df2c74367752, 0xbfe834aeee6ceb04, 0x3fe0006228d0b4db],
        vec: [0x3fdea50f09b221a7, 0x3fe945b559aaa8ea, 0xbfd889e0fe2d8147],
        geom: 0,
        x: 0x3fe45950c4a6046b,
        normal: [0xbff0000000000000, 0x8000000000000000, 0x0000000000000000],
    },
    // turned_180: side, from inside the top box, a -0 in its normal
    Ray {
        doc: "turned_180",
        pnt: [0xbfef1d441560dc76, 0xbfcf6284aeb1f628, 0x3fdc44104e53c24b],
        vec: [0xbfbb484a4c413e6d, 0x3fef7ac2b9284649, 0xbfc28203a0634e53],
        geom: 0,
        x: 0x3fd09f0c1b9cff26,
        normal: [0xbff0000000000000, 0x8000000000000000, 0x0000000000000000],
    },
    // turned_180: the base box's -x side, from beside, the top box missed, met at 0.5
    Ray {
        doc: "turned_180",
        pnt: [0xbff8000000000000, 0xbfb999999999999a, 0x3fe199999999999a],
        vec: [0x3ff0000000000000, 0x0000000000000000, 0x0000000000000000],
        geom: 0,
        x: 0x3fe0000000000000,
        normal: [0xbff0000000000000, 0x8000000000000000, 0x0000000000000000],
    },
    // raised_cell: starts on a raised flat cell, down, met at 0.0
    Ray {
        doc: "raised_cell",
        pnt: [0x3fe0000000000000, 0xbfe0000000000000, 0x3fd0000000000000],
        vec: [0x0000000000000000, 0x0000000000000000, 0xbff0000000000000],
        geom: 0,
        x: 0x0000000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    },
    // raised_cell: starts on a raised flat cell, up and along, met at -0.0
    Ray {
        doc: "raised_cell",
        pnt: [0x3fe0000000000000, 0xbfe0000000000000, 0x3fd0000000000000],
        vec: [0x3fe3333333333333, 0x0000000000000000, 0x3fe999999999999a],
        geom: 0,
        x: 0x8000000000000000,
        normal: [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
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
/// with MuJoCo's normal, or misses as MuJoCo's does; a hit's point is the
/// start plus the direction times MuJoCo's distance.
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
            let hit = raycast_scene(&model, &data, pnt, dir, 1e9, None, None);
            let (geom, x, normal) = hit.map_or((-1, -1.0, [0.0; 3]), |h| {
                let n = h.hit.normal;
                (
                    i64::try_from(h.geom_id).expect("a geom id"),
                    h.hit.distance,
                    [n.x, n.y, n.z],
                )
            });
            let point_off = hit.is_some_and(|h| {
                let want = pnt + dir.into_inner() * f64::from_bits(ray.x);
                !close(
                    &[h.hit.point.x, h.hit.point.y, h.hit.point.z],
                    &[want.x, want.y, want.z].map(f64::to_bits),
                )
            });
            if geom != ray.geom
                || !close(&[x], &[ray.x])
                || !close(&normal, &ray.normal)
                || point_off
            {
                bad.push(format!(
                    "{name} from {pnt:?}: geom {geom} at {x:e} normal {normal:?}, MuJoCo geom {} at {:e}{}",
                    ray.geom,
                    f64::from_bits(ray.x),
                    if point_off { " (the point off)" } else { "" }
                ));
            }
        }
    }
    assert_eq!(
        (RAYS.len(), RAYS.iter().filter(|r| r.geom >= 0).count()),
        (20, 17),
        "every ray read, and every hit"
    );
    assert!(bad.is_empty(), "{} differ:\n{}", bad.len(), bad.join("\n"));
}

/// The model a replaced field is cast in: [`HFIELD_MJCF`]'s, its field's data
/// replaced by `field`, after a forward.
fn with_field(field: HeightFieldData) -> (sim_core::Model, sim_core::Data) {
    let mut model = load_model(HFIELD_MJCF).expect("MJCF should parse");
    model.hfield_data[0] = Arc::new(field);
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    (model, data)
}

/// A ray from `from` along `dir` through `raycast_scene`, cast no farther
/// than `max`.
fn cast(
    model: &sim_core::Model,
    data: &sim_core::Data,
    from: [f64; 3],
    dir: Vector3<f64>,
    max: f64,
) -> Option<sim_core::SceneRayHit> {
    raycast_scene(
        model,
        data,
        Point3::from(from),
        UnitVector3::new_normalize(dir),
        max,
        None,
        None,
    )
}

/// A flat field replaced in code is cast at the asset's size, centred on its
/// geom: 32 × 32 heights of 0.2 under the asset's half-extents 4 × 4 are met
/// straight down at the field's centre and near its −x, −y corner, 2.8 below
/// a start 3 up, with the normal +z, cast no farther than 20 or without
/// bound; a ray past its +x, +y corner, which the grid read from the geom's
/// origin would hold, misses.
#[test]
fn a_replaced_flat_field_is_cast_centred_at_its_assets_size() {
    let (model, data) = with_field(HeightFieldData::flat(32, 32, 8.0 / 31.0, 0.2));
    for (x, y) in [(0.0, 0.0), (-3.5, -3.5)] {
        for max in [20.0, f64::INFINITY] {
            let hit = cast(&model, &data, [x, y, 3.0], -Vector3::z(), max)
                .expect("the ray meets the field");
            assert!(
                (hit.hit.distance - 2.8).abs() < 1e-12,
                "({x}, {y}) within {max}: distance {}",
                hit.hit.distance
            );
            assert_eq!(hit.hit.normal, Vector3::z(), "({x}, {y})");
        }
    }
    let past = cast(&model, &data, [4.5, 4.5, 3.0], -Vector3::z(), 20.0);
    assert!(past.is_none(), "past the corner: {past:?}");
}

/// A replaced field's own spacing is not read: 32 × 32 heights of 0.2 a cell
/// of 0.1 apart, 3.1 across, still span the asset's 8 × 8, met at (3.5, 3.5)
/// and (−3.5, 3.5).
#[test]
fn a_replaced_fields_own_spacing_is_not_read() {
    let (model, data) = with_field(HeightFieldData::flat(32, 32, 0.1, 0.2));
    for (x, y) in [(3.5, 3.5), (-3.5, 3.5)] {
        let hit =
            cast(&model, &data, [x, y, 3.0], -Vector3::z(), 20.0).expect("the ray meets the field");
        assert!(
            (hit.hit.distance - 2.8).abs() < 1e-12,
            "({x}, {y}): distance {}",
            hit.hit.distance
        );
    }
}

/// A replaced field's heights below 0 meet rays as MuJoCo's meet such data
/// (its field's data set to −0.4, times its top of 0.5, on the oracle): a ray
/// down from 3 meets the base box's top, 3.0 away, with the normal +z, and a
/// level ray at z 0.3 misses.
#[test]
fn a_replaced_fields_height_below_zero_meets_rays_as_mujocos() {
    let (model, data) = with_field(HeightFieldData::flat(32, 32, 8.0 / 31.0, -0.2));
    let hit = cast(&model, &data, [0.0, 0.0, 3.0], -Vector3::z(), 20.0)
        .expect("the ray meets the base box");
    assert!(
        (hit.hit.distance - 3.0).abs() < 1e-12,
        "distance {}",
        hit.hit.distance
    );
    assert_eq!(hit.hit.normal, Vector3::z());
    let level = cast(&model, &data, [-5.0, 0.0, 0.3], Vector3::x(), 20.0);
    assert!(level.is_none(), "level at z 0.3: {level:?}");
}

/// A replaced field's heights above its top meet rays as MuJoCo's meet such
/// data (a 32 × 32 field's data set to 1.6, times its top of 0.5, on the
/// oracle): heights of 0.8 are met by a ray down from 3, 2.2 away (MuJoCo
/// 2.199999988079071, its data single precision); a level ray at z 0.7
/// misses, one at z 0.3 meets the top box's −x side, 1 in, with the normal −x;
/// a ray up from (0, 0, 0.7), under the surface and above the top box,
/// misses; a ray from (−4.5, 0, 1) along (1, 0, −0.1), through the surface at
/// x −2.5, meets the +x side 8.542394277952756 away, with the normal +x.
#[test]
fn a_replaced_fields_height_above_the_top_meets_rays_as_mujocos() {
    let (model, data) = with_field(HeightFieldData::flat(32, 32, 8.0 / 31.0, 0.8));
    let down =
        cast(&model, &data, [0.0, 0.0, 3.0], -Vector3::z(), 20.0).expect("the ray meets the field");
    assert!(
        (down.hit.distance - 2.2).abs() < 1e-12,
        "distance {}",
        down.hit.distance
    );
    let above = cast(&model, &data, [-5.0, 0.0, 0.7], Vector3::x(), 20.0);
    assert!(above.is_none(), "level at z 0.7: {above:?}");
    let side =
        cast(&model, &data, [-5.0, 0.0, 0.3], Vector3::x(), 20.0).expect("the ray meets the side");
    assert!(
        (side.hit.distance - 1.0).abs() < 1e-12,
        "distance {}",
        side.hit.distance
    );
    assert_eq!(side.hit.normal, -Vector3::x());
    let up = cast(&model, &data, [0.0, 0.0, 0.7], Vector3::z(), 20.0);
    assert!(up.is_none(), "up from z 0.7: {up:?}");
    let through = cast(
        &model,
        &data,
        [-4.5, 0.0, 1.0],
        Vector3::new(1.0, 0.0, -0.1),
        20.0,
    )
    .expect("the ray meets the far side");
    assert!(
        (through.hit.distance - 8.542_394_277_952_756).abs() < 1e-12,
        "distance {}",
        through.hit.distance
    );
    assert_eq!(through.hit.normal, Vector3::x());
}

/// A hit exactly `max_distance` away is not returned; one a float nearer than
/// `max_distance` is, as `raycast_scene`'s doc says ("nearer than").
#[test]
fn a_hit_exactly_at_max_distance_is_not_returned() {
    let (model, data) = with_field(HeightFieldData::flat(32, 32, 8.0 / 31.0, 0.2));
    let d = cast(&model, &data, [0.0, 0.0, 3.0], -Vector3::z(), 20.0)
        .expect("the ray meets the field")
        .hit
        .distance;
    assert!(
        cast(&model, &data, [0.0, 0.0, 3.0], -Vector3::z(), d).is_none(),
        "at {d}"
    );
    let past = f64::from_bits(d.to_bits() + 1);
    assert!(
        cast(&model, &data, [0.0, 0.0, 3.0], -Vector3::z(), past).is_some(),
        "within {past}"
    );
}

/// A sinusoidal field replaced in code, as the `raycasting/heightfield`
/// example replaces it: heights `0.25 + 0.2 sin(2πx/4) cos(2πy/4)` sampled at
/// the asset's grid, centred on the geom (`x`, `y` from −4 to 4), within its
/// top of 0.5. A ray down meets the terrain at its centre, and a quarter period
/// off it, within 0.02 of the function.
#[test]
fn a_replaced_sinusoidal_field_is_cast_centred_at_its_assets_size() {
    let terrain = |x: f64, y: f64| {
        0.25 + 0.2
            * (std::f64::consts::TAU * x / 4.0).sin()
            * (std::f64::consts::TAU * y / 4.0).cos()
    };
    let (model, data) = with_field(HeightFieldData::from_fn(32, 32, 8.0 / 31.0, |x, y| {
        terrain(x - 4.0, y - 4.0)
    }));
    for (x, y) in [(0.0, 0.0), (-1.0, 0.0)] {
        let hit =
            cast(&model, &data, [x, y, 3.0], -Vector3::z(), 20.0).expect("the ray meets the field");
        let expected = terrain(x, y);
        assert!(
            (hit.hit.point.z - expected).abs() < 0.02,
            "({x}, {y}): z {} where the terrain is {expected}",
            hit.hit.point.z
        );
    }
}
