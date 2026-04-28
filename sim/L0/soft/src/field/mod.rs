//! `Field<T>` — typed spatial fields over reference space.
//!
//! Phase 4 introduces typed spatial fields for per-element material
//! parameters per Part 2 Ch 09 [`00-sdf-valued.md`][s] and Part 7 Ch 02
//! [`00-sampling.md`][a]. The `Field<T>` trait is the consumer-side API
//! that the per-tet centroid sample pass walks at mesh-build time;
//! [`MaterialField`](crate::MaterialField) (commit 3,
//! `material/material_field.rs`) aggregates one `Box<dyn Field<f64>>`
//! per scalar material parameter slot. Phase 4 ships `T = f64` only;
//! `T = Vec3` for HGO fiber direction lands in Phase H per scope memo
//! Decision A.
//!
//! The trait is parallel to (not a sub-trait of) the geometric
//! [`Sdf`](crate::sdf_bridge::Sdf): `Sdf` is the signed-distance surface
//! (scalar with `grad`), `Field` is the typed sample-only surface for
//! material parameters.
//!
//! Naming: book Part 2 §09 §00 spells the trait `Field<Output = T>`
//! (associated type); the code uses `Field<T>` (generic parameter) for
//! 2024-edition Rust idiom per scope memo §3 Decision A and §2 R-3
//! reconciliation. The two forms are surface-equivalent for every
//! consumer in this crate.
//!
//! [s]: ../../../../docs/studies/soft_body_architecture/src/20-materials/09-spatial-fields/00-sdf-valued.md
//! [a]: ../../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment/00-sampling.md

use crate::Vec3;

pub mod constant;
pub mod layered;

pub use constant::ConstantField;
pub use layered::{BlendedScalarField, LayeredScalarField};

/// Typed spatial field over reference space.
///
/// One method, [`Field::sample`], returning the field value `T` at a
/// reference-space point `x_ref`. Implementations must be deterministic
/// (same input, same output) — Phase 2 invariant I-5 carry-forward per
/// scope memo §3 Decision N.
///
/// `Send + Sync` so trait objects (`Box<dyn Field<f64>>` inside the
/// upcoming `MaterialField`) can thread across rayon-style parallel
/// builds when those eventually land. Phase 4's mesher walks single-
/// threaded; the bound is forward-looking.
pub trait Field<T>: Send + Sync {
    /// Field value at reference-space point `x_ref`.
    fn sample(&self, x_ref: Vec3) -> T;
}
