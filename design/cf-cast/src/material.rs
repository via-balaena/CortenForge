//! Caller-supplied material data for cast layers.
//!
//! [`MoldingMaterial`] carries the workshop-procedure data each
//! [`crate::CastLayer`] needs (display name + bulk density + optional
//! cure-protocol key) without coupling cf-cast to `sim-soft`'s FEM dep
//! tree (`faer` + `sim-ml-chassis` + …).
//!
//! Callers that already depend on `sim-soft` for the design-side
//! simulation build a [`MoldingMaterial`] from a
//! `sim_soft::SiliconeMaterial` anchor at one site:
//!
//! ```ignore
//! use cf_cast::MoldingMaterial;
//! use sim_soft::material::silicone_table::{
//!     ConstructionSource, ECOFLEX_00_30, SiliconeMaterial,
//! };
//!
//! let silicone: SiliconeMaterial = ECOFLEX_00_30;
//! let molding = MoldingMaterial {
//!     display_name: "Ecoflex 00-30".to_string(),
//!     density_kg_m3: silicone.density,
//!     anchor_key: match silicone.source {
//!         ConstructionSource::Anchor { name } => Some(name),
//!         _ => None,
//!     },
//! };
//! ```
//!
//! Density flows runtime — there is no compile-time copy of
//! `silicone_table.rs` inside cf-cast, so anchor updates in `sim-soft`
//! propagate without source drift.
//!
//! The cf-cast-local cure-protocol table (Stage 2 F3) keys on
//! [`MoldingMaterial::anchor_key`] to look up mix ratio + pot life +
//! cure time per material; entries with `anchor_key = None`
//! (interpolated / measured / non-Smooth-On materials) surface as a
//! "consult Smooth-On TDS for cure protocol" placeholder in the
//! generated procedure markdown.

/// Per-layer material data — what the caller pours into one cast
/// layer.
///
/// Field semantics:
///
/// - [`display_name`]: human-readable name surfaced in F3 procedure
///   markdown and per-layer error messages. Owned `String` so
///   recipe-augmented names ("Ecoflex 00-30 + 50 % Slacker") are
///   first-class without lifetime gymnastics.
/// - [`density_kg_m3`]: bulk density in kilograms per cubic metre.
///   F2 multiplies SDF-negative voxel volume by this to compute pour
///   mass and enforce the per-silicone mass budget.
/// - [`anchor_key`]: optional `'static` identifier matching a
///   `sim_soft::SiliconeMaterial` anchor name (`"ECOFLEX_00_30"`,
///   `"DRAGON_SKIN_10A"`, …) so cf-cast's local cure-protocol table
///   can resolve mix ratio + pot life + cure time. `None` for
///   materials without a published anchor (post-cast measurement,
///   anchor-bounded interpolation, third-party silicones).
///
/// [`display_name`]: Self::display_name
/// [`density_kg_m3`]: Self::density_kg_m3
/// [`anchor_key`]: Self::anchor_key
#[derive(Debug, Clone)]
pub struct MoldingMaterial {
    /// Human-readable name for procedure markdown + error messages.
    pub display_name: String,
    /// Bulk density in kg/m³ (drives F2 pour-mass calculation).
    pub density_kg_m3: f64,
    /// Optional key into cf-cast's cure-protocol table; matches a
    /// `sim_soft::SiliconeMaterial::source = Anchor { name }` value.
    pub anchor_key: Option<&'static str>,
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::MoldingMaterial;

    #[test]
    fn molding_material_construction_holds_fields_verbatim() {
        // Pinning that the public-field struct preserves every value
        // bit-equally — F2 and F3 will assume density and anchor_key
        // are stored as supplied, with no normalization.
        let m = MoldingMaterial {
            display_name: "Ecoflex 00-30".to_string(),
            density_kg_m3: 1070.0,
            anchor_key: Some("ECOFLEX_00_30"),
        };
        assert_eq!(m.display_name, "Ecoflex 00-30");
        assert!((m.density_kg_m3 - 1070.0).abs() < f64::EPSILON);
        assert_eq!(m.anchor_key, Some("ECOFLEX_00_30"));
    }

    #[test]
    fn molding_material_accepts_none_anchor_for_measured_paths() {
        // Post-cast Path-3 / interpolated Path-2 materials carry
        // anchor_key = None; F3 will fall back to a TDS-lookup
        // placeholder for these.
        let m = MoldingMaterial {
            display_name: "Ecoflex 00-30 + 50 % Slacker".to_string(),
            density_kg_m3: 1055.0,
            anchor_key: None,
        };
        assert_eq!(m.anchor_key, None);
    }
}
