//! Integration test: MJCF heightfield loading → data replacement → raycast.
//!
//! Validates the full pipeline that the visual heightfield example depends on:
//! 1. MJCF with `<hfield>` asset parses and builds a Model with hfield data
//! 2. Replacing `model.hfield_data[0]` with a custom `HeightFieldData` works
//! 3. `raycast_scene` hits the replaced terrain at the expected position

use std::sync::Arc;

use nalgebra::{Point3, UnitVector3, Vector3};
use sim_core::{HeightFieldData, raycast_scene};
use sim_mjcf::load_model;

const HFIELD_MJCF: &str = r#"
<mujoco model="hfield-raycast">
  <option gravity="0 0 0" timestep="0.002"/>
  <asset>
    <hfield name="terrain" nrow="4" ncol="4" size="4 4 0.5 0"
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

/// Replace hfield data with a flat surface and raycast against it.
#[test]
fn hfield_replace_and_raycast_flat() {
    let mut model = load_model(HFIELD_MJCF).expect("MJCF should parse");

    // Replace with a flat heightfield at z=0
    let flat = HeightFieldData::flat(32, 32, 0.25, 0.0);
    model.hfield_data[0] = Arc::new(flat);

    let mut data = model.make_data();
    let _ = data.forward(&model);

    // Ray from above the center of the field, pointing down
    // Field extent: 32 cells × 0.25 = 8m, center at (4, 4)
    let origin = Point3::new(4.0, 4.0, 3.0);
    let dir = UnitVector3::new_normalize(-Vector3::z());

    let hit = raycast_scene(&model, &data, origin, dir, 20.0, None, None);
    let hit = hit.expect("ray should hit the flat heightfield");

    assert!(
        (hit.hit.distance - 3.0).abs() < 0.02,
        "expected dist ≈ 3.0, got {:.4}",
        hit.hit.distance
    );
    assert!(
        hit.hit.point.z.abs() < 0.02,
        "expected hit.z ≈ 0, got {:.4}",
        hit.hit.point.z
    );
}

/// Replace hfield data with sinusoidal terrain and verify hit matches the function.
#[test]
fn hfield_replace_and_raycast_sinusoidal() {
    let mut model = load_model(HFIELD_MJCF).expect("MJCF should parse");

    let cell_size = 0.25;
    let terrain_fn = |x: f64, y: f64| -> f64 {
        0.3 * (std::f64::consts::TAU * x / 4.0).sin() * (std::f64::consts::TAU * y / 4.0).cos()
    };

    let hfield = HeightFieldData::from_fn(32, 32, cell_size, terrain_fn);
    model.hfield_data[0] = Arc::new(hfield);

    let mut data = model.make_data();
    let _ = data.forward(&model);

    // Cast a ray at the field center (4, 4) where terrain(4, 4) = 0.3 * sin(2π) * cos(2π) = 0
    let origin_center = Point3::new(4.0, 4.0, 3.0);
    let dir = UnitVector3::new_normalize(-Vector3::z());

    let hit_center = raycast_scene(&model, &data, origin_center, dir, 20.0, None, None)
        .expect("center ray should hit");
    let expected_z = terrain_fn(4.0, 4.0);
    assert!(
        (hit_center.hit.point.z - expected_z).abs() < 0.02,
        "center: expected z ≈ {expected_z:.4}, got {:.4}",
        hit_center.hit.point.z
    );

    // Cast at (3, 4) where terrain = 0.3 * sin(3π/2) * cos(2π) = -0.3
    let origin_off = Point3::new(3.0, 4.0, 3.0);
    let hit_off = raycast_scene(&model, &data, origin_off, dir, 20.0, None, None)
        .expect("off-center ray should hit");
    let expected_z_off = terrain_fn(3.0, 4.0);
    assert!(
        (hit_off.hit.point.z - expected_z_off).abs() < 0.05,
        "off-center: expected z ≈ {expected_z_off:.4}, got {:.4}",
        hit_off.hit.point.z
    );
}
