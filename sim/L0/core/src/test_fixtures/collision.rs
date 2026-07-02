//! Minimal geom fixtures for narrow-phase collision unit tests.
//!
//! Narrow-phase colliders (`collision::pair_*`, `collision::plane`, …) take a
//! `&Model` only to read per-geom contact parameters (`friction`, `condim`,
//! `solref`/`solimp`, `margin`/`gap`). [`collision_geoms`] builds a bodyless
//! `Model` with `ngeom` default-parameterized geoms; each test then overrides
//! `geom_type`/`geom_size` per geom and passes explicit poses/sizes to the
//! collider under test.

use crate::types::{GeomType, Model};
use nalgebra::{UnitQuaternion, Vector3};

/// Build a `Model` carrying `ngeom` geoms with MuJoCo-default contact params.
///
/// Defaults: unit sphere, friction `[1.0, 0.005, 0.0001]`, condim 3, zero
/// margin/gap. Tests override `geom_type`/`geom_size` per geom.
#[must_use]
pub fn collision_geoms(ngeom: usize) -> Model {
    let mut model = Model::empty();
    model.ngeom = ngeom;
    model.geom_type = vec![GeomType::Sphere; ngeom];
    model.geom_body = vec![0; ngeom];
    model.geom_pos = vec![Vector3::zeros(); ngeom];
    model.geom_quat = vec![UnitQuaternion::identity(); ngeom];
    model.geom_size = vec![Vector3::new(1.0, 1.0, 1.0); ngeom];
    model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); ngeom];
    model.geom_condim = vec![3; ngeom];
    model.geom_contype = vec![1; ngeom];
    model.geom_conaffinity = vec![1; ngeom];
    model.geom_margin = vec![0.0; ngeom];
    model.geom_gap = vec![0.0; ngeom];
    model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; ngeom];
    model.geom_solref = vec![[0.02, 1.0]; ngeom];
    model.geom_name = vec![None; ngeom];
    model.geom_rbound = vec![1.0; ngeom];
    model.geom_mesh = vec![None; ngeom];
    model.geom_priority = vec![0; ngeom];
    model.geom_solmix = vec![1.0; ngeom];
    model
}
