//! `cast.toml` schema + deserialization + semantic validation.
//!
//! See `project_scan_to_cast_bridge_design.md` for the schema
//! rationale. This module is pure (no I/O); higher-level orchestration
//! lives in [`crate::run`].

use std::path::PathBuf;

use anyhow::{Result, bail, ensure};
use serde::Deserialize;

use cf_cast::DEFAULT_MASS_BUDGET_KG;

/// Top-level `cast.toml` schema.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CastConfig {
    /// Reference to the cf-scan-prep output that this cast consumes.
    pub scan: ScanConfig,
    /// Cast-level numerical defaults + overrides.
    #[serde(default)]
    pub cast: CastDefaults,
    /// Slice 9 — optional reference to a `<scan>.design.toml`
    /// produced by cf-device-design. When set, the layer stack is
    /// **derived** from the design (the cast.toml's `[[layers]]`
    /// must be empty in that case). When absent, the cast.toml's
    /// own `[[layers]]` is authoritative — backward compatible with
    /// pre-slice-9 cast TOMLs. See
    /// `docs/INSERTION_SIM_STATE.md`'s slice-9 update for the
    /// rationale + mapping rules.
    #[serde(default)]
    pub design: Option<DesignSourceConfig>,
    /// Innermost-first per-layer thickness + material. Required iff
    /// `[design]` is absent; must be empty iff `[design]` is set.
    /// See `CastConfig::validate` for the cross-field gate.
    #[serde(default)]
    pub layers: Vec<LayerConfig>,
    /// Plug-anchor pin override (default = `PlugPinSpec::iter1()`).
    /// Absence of the table means "enabled with iter1 defaults".
    /// Set `enabled = false` to disable plug-anchor pins entirely.
    #[serde(default)]
    pub plug_pins: PlugPinConfig,
    /// Pour gate + air vent override (default = `PourGateSpec::iter1()`,
    /// enabled). Absence of the table means "enabled with iter1
    /// defaults". Set `enabled = false` to disable.
    #[serde(default)]
    pub pour_gate: PourGateConfig,
    /// Inter-piece registration pin override (default =
    /// `PinSpec::iter1()` + enabled). Absence of the table means
    /// "enabled with iter1 defaults". Set `enabled = false` to
    /// disable.
    #[serde(default)]
    pub registration_pins: RegistrationConfig,
}

/// Slice 9 — `[design]` block. Points cf-cast-cli at the
/// cf-device-design output for this scan, so the engineered layer
/// stack feeds the cast directly (no hand-duplication between
/// `.design.toml` and `cast.toml`'s `[[layers]]`).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DesignSourceConfig {
    /// Path (absolute or relative to the cast TOML's directory) to
    /// the cf-device-design `.design.toml`. The file's
    /// `schema_version` must be ≤
    /// [`design_ref::DESIGN_TOML_SCHEMA_VERSION_READ`](crate::design_ref::DESIGN_TOML_SCHEMA_VERSION_READ).
    pub path: PathBuf,
}

/// `[scan]` block — points the bridge at the cf-scan-prep output.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScanConfig {
    /// Path (absolute or relative to the cast TOML's directory) to
    /// the cf-scan-prep cleaned STL.
    pub cleaned_stl: PathBuf,
    /// Path (absolute or relative to the cast TOML's directory) to
    /// the cf-scan-prep `.prep.toml`. The `[centerline].points_m`
    /// block must be present and non-empty.
    pub prep_toml: PathBuf,
}

/// `[cast]` block — numerical defaults + overrides shared across
/// layers. Every field has a workspace-default so the block can be
/// omitted entirely for workshop-default behavior.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CastDefaults {
    /// Marching-cubes sampling cell size (meters). v2 example
    /// defaults to 3 mm; this bridge inherits.
    #[serde(default = "default_mesh_cell_size_m")]
    pub mesh_cell_size_m: f64,
    /// Per-pour silicone mass budget (kg). Defaults to
    /// [`cf_cast::DEFAULT_MASS_BUDGET_KG`] (2 lb).
    #[serde(default = "default_mass_budget_kg")]
    pub mass_budget_kg: f64,
    /// Bounding-region pad relative to the scan AABB (meters per
    /// axis). The bounding cuboid envelopes the outer-most layer
    /// body plus this margin.
    #[serde(default = "default_bounding_margin_m")]
    pub bounding_margin_m: f64,
    /// Ribbon split-normal unit vector (will be normalized before
    /// `SplitNormal::new`). Default `[0, 0, -1]` per v2 example —
    /// XZ-plane centerlines bisect LEFT/RIGHT for balanced pieces.
    #[serde(default = "default_split_normal")]
    pub split_normal: [f64; 3],
    /// Per-piece minimum wall thickness for the F4 gate (mm).
    /// Default `0.1` per v2 example (FDM-default 0.4 mm extrusion
    /// width tolerates sub-mm seam stair-stepping).
    #[serde(default = "default_piece_min_wall_mm")]
    pub piece_min_wall_mm: f64,
    /// Output directory (relative to the cast TOML's directory or
    /// absolute). Default `"out"`.
    #[serde(default = "default_output_dir")]
    pub output_dir: PathBuf,
}

impl Default for CastDefaults {
    fn default() -> Self {
        Self {
            mesh_cell_size_m: default_mesh_cell_size_m(),
            mass_budget_kg: default_mass_budget_kg(),
            bounding_margin_m: default_bounding_margin_m(),
            split_normal: default_split_normal(),
            piece_min_wall_mm: default_piece_min_wall_mm(),
            output_dir: default_output_dir(),
        }
    }
}

fn default_mesh_cell_size_m() -> f64 {
    0.003
}
fn default_mass_budget_kg() -> f64 {
    DEFAULT_MASS_BUDGET_KG
}
fn default_bounding_margin_m() -> f64 {
    0.020
}
fn default_split_normal() -> [f64; 3] {
    [0.0, 0.0, -1.0]
}
fn default_piece_min_wall_mm() -> f64 {
    0.1
}
fn default_output_dir() -> PathBuf {
    PathBuf::from("out")
}

/// `[[layers]]` entry — one cast layer's thickness + material.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayerConfig {
    /// Cured shell thickness on top of the inward boundary (meters).
    /// For layer 0 this is the thickness above the scan surface; for
    /// layer N > 0 this is the thickness above layer N-1's outer
    /// surface (per the v2 example's plug-stack convention).
    pub thickness_m: f64,
    /// Cure-table anchor key — one of `"ECOFLEX_00_10"`,
    /// `"ECOFLEX_00_20"`, `"ECOFLEX_00_30"`, `"ECOFLEX_00_50"`,
    /// `"DRAGON_SKIN_10A"`, `"DRAGON_SKIN_15"`, `"DRAGON_SKIN_20A"`,
    /// `"DRAGON_SKIN_30A"`. The cf-cast `cure` table resolves the
    /// anchor to a [`cf_cast::CureProtocol`]; this bridge looks up
    /// `density_kg_m3` from its own per-anchor table, or honors a
    /// per-layer `density_kg_m3` override below.
    pub material: String,
    /// Optional density override (kg/m³). Required iff the anchor
    /// is not present in [`cf_cast::cure::lookup`] OR the workshop
    /// wants a different density than the bridge's per-anchor default
    /// table. When `material` is a known cf-cast cure anchor and this
    /// field is absent, the density falls back to the per-anchor
    /// default in [`crate::density_for_anchor`].
    #[serde(default)]
    pub density_kg_m3: Option<f64>,
    /// Human-readable name for procedure markdown + report stdout.
    /// Defaults to a title-cased version of the anchor key (e.g.,
    /// `"ECOFLEX_00_30"` → `"Ecoflex 00-30"`).
    #[serde(default)]
    pub display_name: Option<String>,
}

/// `[plug_pins]` block — plug-anchor pin override. Maps to
/// [`cf_cast::PlugPinKind`].
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlugPinConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::PlugPinKind::None`] to the ribbon (no plug-anchor
    /// pin geometry).
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Override [`cf_cast::PlugPinSpec::pin_length_m`]. The v2
    /// example overrides the iter1 20 mm default to 28 mm to clear
    /// the bounding cup wall on every layer; bridge users with
    /// thicker shells may need more.
    #[serde(default)]
    pub pin_length_m: Option<f64>,
    /// Override [`cf_cast::PlugPinSpec::include_dome_pin`]. Defaults
    /// to `None` (= use iter1's `false`).
    #[serde(default)]
    pub include_dome_pin: Option<bool>,
}

impl Default for PlugPinConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            pin_length_m: None,
            include_dome_pin: None,
        }
    }
}

/// `[pour_gate]` block — pour-gate + apex vent toggle. Defaults to
/// `enabled = true` with [`cf_cast::PourGateSpec::iter1`] geometry.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PourGateConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::PourGateKind::None`] (no pour gate, no vent).
    #[serde(default = "default_true")]
    pub enabled: bool,
}

impl Default for PourGateConfig {
    fn default() -> Self {
        Self { enabled: true }
    }
}

/// `[registration_pins]` block — inter-piece pin toggle. Defaults to
/// `enabled = true` with [`cf_cast::PinSpec::iter1`] geometry.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegistrationConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::RegistrationKind::None`] (pieces clamp by hand).
    #[serde(default = "default_true")]
    pub enabled: bool,
}

impl Default for RegistrationConfig {
    fn default() -> Self {
        Self { enabled: true }
    }
}

fn default_true() -> bool {
    true
}

impl CastConfig {
    /// Parse a [`CastConfig`] from a `cast.toml` string.
    ///
    /// # Errors
    ///
    /// Returns the `toml` parse error wrapped in `anyhow`.
    pub fn from_toml_str(text: &str) -> Result<Self> {
        let cfg: Self = toml::from_str(text)?;
        Ok(cfg)
    }

    /// Run semantic validation against this config. Pure (no I/O).
    ///
    /// Validates non-empty layers, finite + positive thicknesses,
    /// positive mesh cell size + bounding margin, finite + positive
    /// piece minimum wall, normalizable split-normal, and per-layer
    /// material resolvable to a density (anchor lookup OR explicit
    /// override).
    ///
    /// # Errors
    ///
    /// Returns the first failing invariant; chains are short so the
    /// CLI's stderr error includes the field name + value.
    /// Slice 9 cross-field gate — checks `[design]` ↔ `[[layers]]`
    /// mutual exclusion + non-emptiness BEFORE any design.toml lift.
    /// Caller pattern:
    ///
    /// 1. `cfg.validate_layer_source()?;` — enforce the gate.
    /// 2. If `cfg.design.is_some()`, lift `design.layers` → `cfg.layers`.
    /// 3. `cfg.validate_after_layer_source()?;` — per-layer + numeric
    ///    + split-normal + plug-pin checks on the now-populated layers.
    ///
    /// The split exists because step 1's "layers must be empty when
    /// `[design]` is set" disagrees with step 3's "layers must be
    /// non-empty". Splitting the two passes makes each one's
    /// invariant unambiguous.
    ///
    /// # Errors
    ///
    /// - `[design]` is set AND `[[layers]]` is non-empty (two sources
    ///   of truth — the user dialed both).
    /// - `[design]` is absent AND `[[layers]]` is empty (no layer
    ///   source at all).
    pub fn validate_layer_source(&self) -> Result<()> {
        if self.design.is_some() {
            ensure!(
                self.layers.is_empty(),
                "cast.toml: `[design]` is set, so `[[layers]]` must be empty \
                 (the layer stack is derived from the design.toml). Remove the \
                 `[[layers]]` entries or drop `[design]`."
            );
        } else {
            ensure!(
                !self.layers.is_empty(),
                "cast.toml: `[[layers]]` is empty AND `[design]` is absent — \
                 neither layer source is set. Either add `[[layers]]` entries \
                 or point `[design].path` at a cf-device-design `.design.toml`."
            );
        }
        Ok(())
    }

    /// Pre-slice-9 single-pass validate, kept for callers that don't
    /// participate in the design-lift dance. Internally calls
    /// [`validate_layer_source`](Self::validate_layer_source) (the
    /// cross-field gate) then [`validate_after_layer_source`](Self::validate_after_layer_source).
    /// The split lets the run pipeline interleave the design TOML
    /// lift between them; callers that don't need that interleave
    /// can keep using `validate()` unchanged.
    ///
    /// # Errors
    ///
    /// Forwards either sub-method's error.
    pub fn validate(&self) -> Result<()> {
        self.validate_layer_source()?;
        self.validate_after_layer_source()
    }

    /// Per-layer + numeric + split-normal + plug-pin invariants.
    /// Slice 9 split: callers that lifted layers from a design.toml
    /// run this AFTER the lift; callers without a `[design]` block
    /// reach it via [`validate`](Self::validate) directly.
    ///
    /// # Errors
    ///
    /// Surfaces the first failing invariant (field name + value in
    /// the chain).
    pub fn validate_after_layer_source(&self) -> Result<()> {
        // After the layer-source gate this assert holds (either
        // `[design]` was just lifted into `layers`, or there was no
        // `[design]` and `layers` was non-empty from the start).
        // `ensure!` rather than a bare assert because hand-built
        // `CastConfig`s in tests sometimes call this method out of
        // order.
        ensure!(
            !self.layers.is_empty(),
            "cast.toml: validate_after_layer_source called with empty layers — \
             call validate_layer_source() first (or lift from a design TOML)"
        );
        for (i, layer) in self.layers.iter().enumerate() {
            ensure!(
                layer.thickness_m.is_finite() && layer.thickness_m > 0.0,
                "layer[{}].thickness_m = {} must be finite and > 0",
                i,
                layer.thickness_m
            );
            if let Some(d) = layer.density_kg_m3 {
                ensure!(
                    d.is_finite() && d > 0.0,
                    "layer[{}].density_kg_m3 = {} must be finite and > 0",
                    i,
                    d
                );
            }
            if crate::derive::density_for_anchor(&layer.material).is_none()
                && layer.density_kg_m3.is_none()
            {
                bail!(
                    "layer[{}] material '{}' is not a recognized cf-cast cure anchor and no density_kg_m3 override is provided",
                    i,
                    layer.material
                );
            }
        }

        ensure!(
            self.cast.mesh_cell_size_m.is_finite() && self.cast.mesh_cell_size_m > 0.0,
            "cast.mesh_cell_size_m = {} must be finite and > 0",
            self.cast.mesh_cell_size_m
        );
        ensure!(
            self.cast.mass_budget_kg.is_finite() && self.cast.mass_budget_kg > 0.0,
            "cast.mass_budget_kg = {} must be finite and > 0",
            self.cast.mass_budget_kg
        );
        ensure!(
            self.cast.bounding_margin_m.is_finite() && self.cast.bounding_margin_m >= 0.0,
            "cast.bounding_margin_m = {} must be finite and >= 0",
            self.cast.bounding_margin_m
        );
        ensure!(
            self.cast.piece_min_wall_mm.is_finite() && self.cast.piece_min_wall_mm > 0.0,
            "cast.piece_min_wall_mm = {} must be finite and > 0",
            self.cast.piece_min_wall_mm
        );

        let n = self.cast.split_normal;
        ensure!(
            n.iter().all(|c| c.is_finite()),
            "cast.split_normal = {:?} has non-finite component",
            n
        );
        let norm_sq = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
        ensure!(
            norm_sq > 0.0,
            "cast.split_normal = {:?} has zero magnitude",
            n
        );

        if let Some(len) = self.plug_pins.pin_length_m {
            ensure!(
                len.is_finite() && len > 0.0,
                "plug_pins.pin_length_m = {} must be finite and > 0",
                len
            );
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn minimal_config_text() -> &'static str {
        r#"
[scan]
cleaned_stl = "scan.cleaned.stl"
prep_toml = "scan.cleaned.prep.toml"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"
"#
    }

    #[test]
    fn parses_minimal_config_with_defaults() {
        let cfg = CastConfig::from_toml_str(minimal_config_text()).unwrap();
        cfg.validate().unwrap();
        assert_eq!(cfg.layers.len(), 1);
        assert_eq!(cfg.layers[0].material, "ECOFLEX_00_30");
        // Defaults from CastDefaults.
        assert!((cfg.cast.mesh_cell_size_m - 0.003).abs() < 1e-12);
        assert!((cfg.cast.mass_budget_kg - DEFAULT_MASS_BUDGET_KG).abs() < 1e-12);
        assert!((cfg.cast.bounding_margin_m - 0.020).abs() < 1e-12);
        assert_eq!(cfg.cast.split_normal, [0.0, 0.0, -1.0]);
        assert!((cfg.cast.piece_min_wall_mm - 0.1).abs() < 1e-12);
        assert_eq!(cfg.cast.output_dir, PathBuf::from("out"));
        // Block defaults.
        assert!(cfg.plug_pins.enabled);
        assert!(cfg.pour_gate.enabled);
        assert!(cfg.registration_pins.enabled);
    }

    #[test]
    fn parses_full_three_layer_config() {
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[cast]
mesh_cell_size_m = 0.004
mass_budget_kg = 0.500
bounding_margin_m = 0.030
split_normal = [0.0, 1.0, 0.0]
piece_min_wall_mm = 0.5
output_dir = "iter1_out"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"
display_name = "Inner Ecoflex"

[[layers]]
thickness_m = 0.004
material = "DRAGON_SKIN_10A"

[[layers]]
thickness_m = 0.004
material = "ECOFLEX_00_30"

[plug_pins]
pin_length_m = 0.028
include_dome_pin = false

[pour_gate]
enabled = true

[registration_pins]
enabled = false
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert_eq!(cfg.layers.len(), 3);
        assert_eq!(cfg.layers[0].display_name.as_deref(), Some("Inner Ecoflex"));
        assert!((cfg.cast.mesh_cell_size_m - 0.004).abs() < 1e-12);
        assert_eq!(cfg.cast.output_dir, PathBuf::from("iter1_out"));
        assert_eq!(cfg.plug_pins.pin_length_m, Some(0.028));
        assert!(!cfg.registration_pins.enabled);
    }

    #[test]
    fn rejects_empty_layers() {
        // `layers = []` is at the top level — keep it BEFORE the
        // `[scan]` table opener so the TOML parser doesn't try to
        // attach it to `[scan]`'s schema.
        let text = r#"
layers = []

[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg.validate().expect_err("empty layers must fail");
        assert!(err.to_string().contains("empty"), "unexpected error: {err}");
    }

    #[test]
    fn rejects_unknown_material_without_density_override() {
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[[layers]]
thickness_m = 0.006
material = "UNKNOWN_GRADE"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg
            .validate()
            .expect_err("unknown material without override must fail");
        let s = err.to_string();
        assert!(s.contains("UNKNOWN_GRADE"), "unexpected error: {s}");
        assert!(s.contains("density_kg_m3"), "unexpected error: {s}");
    }

    #[test]
    fn accepts_unknown_material_with_density_override() {
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[[layers]]
thickness_m = 0.006
material = "CUSTOM_GRADE"
density_kg_m3 = 1080.0
display_name = "Custom Silicone"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
    }

    #[test]
    fn rejects_zero_split_normal() {
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[cast]
split_normal = [0.0, 0.0, 0.0]

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg.validate().expect_err("zero split_normal must fail");
        assert!(
            err.to_string().contains("zero magnitude"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn rejects_negative_thickness() {
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[[layers]]
thickness_m = -0.001
material = "ECOFLEX_00_30"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg.validate().expect_err("negative thickness must fail");
        assert!(
            err.to_string().contains("thickness_m"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn rejects_unknown_fields() {
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"
typo_field = "boom"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"
"#;
        let err =
            CastConfig::from_toml_str(text).expect_err("unknown_fields must fail at parse time");
        assert!(
            err.to_string().contains("typo_field"),
            "unexpected error: {err}"
        );
    }
}
