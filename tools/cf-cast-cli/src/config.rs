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
    /// Per-layer gasket mold override (default = enabled with
    /// `GasketSpec::iter1()` + Ecoflex 00-30 material). Absence of
    /// the table means "enabled with iter1 defaults". Set
    /// `enabled = false` to disable. S3 of the seam-gasket-mold arc.
    #[serde(default)]
    pub gasket: GasketConfig,
    /// Seam-plane flange override (default = enabled with
    /// [`cf_cast::FlangeSpec::iter1`] geometry). Absence of the table
    /// means "enabled with iter1 defaults". Set `enabled = false` to
    /// disable. S2 of the seam-flange arc per recon §F-6.
    #[serde(default)]
    pub flange: FlangeConfig,
    /// Symmetric dowel-hole registration override (default = enabled
    /// with [`cf_cast::dowel_hole::DowelHoleSpec::iter1`] geometry).
    /// Absence of the table means "enabled with iter1 defaults". Set
    /// `enabled = false` to disable. §M-S2 of the unified-mating-plane
    /// arc per [[project-cf-cast-unified-mating-plane-recon]].
    #[serde(default)]
    pub dowel_hole: DowelHoleConfig,
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
    /// Mold cup-wall thickness (meters). The bounding region is the
    /// outer-most layer body grown outward by this much (Option A —
    /// `outermost.offset(wall_thickness_m)`); the cup wall therefore
    /// follows the device contour rather than enveloping it in a
    /// cuboid. Default `0.005` (5 mm) — FDM single-perimeter +
    /// 30 % infill is rigid enough at this thickness for the iter-1
    /// silicone pour pressures. SLA prints + thinner-extrusion FDM
    /// setups can dial down; deeper pours / stiffer rind layers
    /// dial up.
    #[serde(default = "default_wall_thickness_m")]
    pub wall_thickness_m: f64,
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
    /// Feature flag for the **scan-mesh-direct plug_layer_0** path
    /// (S1 of `docs/CF_CAST_SCAN_MESH_DIRECT_RECON.md`).
    ///
    /// When `true` AND `cavity_inset_m == 0`, the layer-0 plug STL is
    /// emitted directly from the cf-scan-prep cleaned scan mesh
    /// instead of through the `pinned_floor_shell → SDF → marching
    /// cubes` pipeline. Layers 1+ continue on the SDF/MC path until
    /// S2 of the recon extends scan-mesh-direct to the offset cases.
    ///
    /// Off by default — the SDF/MC path is bit-preserved for every
    /// cast.toml that doesn't opt in.
    ///
    /// **Safety guard**: the flag is silently ignored when
    /// `cavity_inset_m > 0` because the scan mesh has no inward
    /// offset baked in and a non-zero `cavity_inset_m` would emit a
    /// plug oversized by the inset amount. cf-cast-cli derives
    /// `cavity_inset_m` from the design / inline-layers source and
    /// gates the opt-in accordingly.
    #[serde(default)]
    pub scan_mesh_direct_plug_layer_0: bool,
}

impl Default for CastDefaults {
    fn default() -> Self {
        Self {
            mesh_cell_size_m: default_mesh_cell_size_m(),
            mass_budget_kg: default_mass_budget_kg(),
            wall_thickness_m: default_wall_thickness_m(),
            split_normal: default_split_normal(),
            piece_min_wall_mm: default_piece_min_wall_mm(),
            output_dir: default_output_dir(),
            scan_mesh_direct_plug_layer_0: false,
        }
    }
}

fn default_mesh_cell_size_m() -> f64 {
    0.003
}
fn default_mass_budget_kg() -> f64 {
    DEFAULT_MASS_BUDGET_KG
}
fn default_wall_thickness_m() -> f64 {
    0.005
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
    /// Slice 9.5 — Smooth-On Slacker mass fraction (0.0–1.0) to mix
    /// into the base silicone for this layer. Informational only:
    /// surfaced in procedure.md's `## Slacker Recipe` section as a
    /// pour-side recipe line, but doesn't drive cf-cast geometry.
    ///
    /// Lifted from `.design.toml`'s per-layer `slacker_fraction` when
    /// the design source is active; defaults to `None` for the
    /// inline-layers path (cast.toml currently has no surface for
    /// this — Slacker is a cf-device-design concept, surfaced in cf-
    /// cast-cli's procedure markdown for workshop completeness).
    #[serde(default)]
    pub slacker_fraction: Option<f64>,
}

/// `[plug_pins]` block — plug-floor-lock toggle. Maps to
/// [`cf_cast::PlugPinKind`].
///
/// Post-S4 of the FDM-friendly geometry arc the only user-facing
/// knob is `enabled`. The pre-S4 `pin_length_m` + `include_dome_pin`
/// per-field overrides retired:
/// - `pin_length_m`: pinned by [`cf_cast::PrismaticPinSpec::plug_lock_default`]
///   per recon-1 §G-6 / §G-8 typed-range defaults; S7 workshop-
///   physical calibration narrows numeric values rather than
///   exposing per-cast `cast.toml` overrides.
/// - `include_dome_pin`: dome-end variant is out of scope for the
///   iter-3 default path per §G-1 (the plug-floor lock is a single
///   cap-plane-end feature).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlugPinConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::PlugPinKind::None`] to the ribbon (no plug-floor
    /// lock geometry; workshop user hand-positions the plug during
    /// pour + cure).
    #[serde(default = "default_true")]
    pub enabled: bool,
}

impl Default for PlugPinConfig {
    fn default() -> Self {
        Self { enabled: true }
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

/// `[gasket]` block — per-layer gasket mold toggle + material pick.
/// Maps to [`cf_cast::GasketKind`].
///
/// S3 of the seam-gasket-mold arc per recon §G-7. Defaults to
/// `enabled = true` with [`cf_cast::GasketSpec::iter1`] geometry +
/// Ecoflex 00-30 material. Workshop user may flip
/// `material = "DRAGON_SKIN_10A"` at S6 iter-3 pour if the Ecoflex
/// pour compresses too freely. Cross-section is pinned trapezoidal
/// at iter1 (recon §G-7 default; S2 picked); no per-cast TOML
/// override surfaced — `draft_angle_deg` recalibration belongs to a
/// downstream S6 / S7 arc, not iter-3.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GasketConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::GasketKind::None`] (no per-layer gasket molds; cup
    /// halves hand-clamped without a silicone seal).
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Gasket material — Smooth-On silicone product key.
    /// Recognized values: `"ECOFLEX_00_30"` (iter1 default),
    /// `"DRAGON_SKIN_10A"`. Maps to [`cf_cast::GasketMaterial`].
    /// Absent in the TOML → falls back to the iter1 default
    /// (Ecoflex 00-30 per recon §G-1).
    #[serde(default)]
    pub material: Option<String>,
}

impl Default for GasketConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            material: None,
        }
    }
}

/// `[flange]` block — seam-plane clampable flange toggle + geometry
/// overrides. Maps to [`cf_cast::FlangeKind`].
///
/// S2 of the seam-flange arc per recon §F-6. Defaults to
/// `enabled = true` with [`cf_cast::FlangeSpec::iter1`] geometry
/// (15 mm width × 4 mm thickness per half × 2 mm inner offset). Per-
/// field overrides are surfaced as optionals; absent → falls back to
/// the iter1 default for that field. The cross-field invariant
/// `inner_offset_m > GasketSpec.channel_width_m / 2` is enforced at
/// [`CastConfig::validate_after_layer_source`] time when both the
/// gasket and flange are enabled (recon §F-4 "gasket-disjoint
/// invariant").
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlangeConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::FlangeKind::None`] (no seam-plane flange; cup
    /// halves clamped via whatever contoured surface they have).
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Lateral extent (meters) from `inner_offset_m` outward in the
    /// seam plane. `None` → falls back to
    /// [`cf_cast::FlangeSpec::iter1`]'s 15 mm default.
    #[serde(default)]
    pub width_m: Option<f64>,
    /// Half-thickness (meters) perpendicular to the seam plane.
    /// Closed flange-zone thickness ≈ 2 × this. `None` → 4 mm per
    /// half (iter1 default).
    #[serde(default)]
    pub thickness_m: Option<f64>,
    /// Lateral gap (meters) between body cavity perimeter and
    /// flange inner edge. Must exceed half the gasket channel width
    /// when the gasket is enabled (recon §F-4). `None` → 2 mm
    /// (iter1 default).
    #[serde(default)]
    pub inner_offset_m: Option<f64>,
}

impl Default for FlangeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            width_m: None,
            thickness_m: None,
            inner_offset_m: None,
        }
    }
}

/// `[dowel_hole]` block — symmetric dowel-hole registration toggle
/// + geometry overrides. Maps to [`cf_cast::dowel_hole::DowelHoleKind`].
///
/// §M-S2 of [[project-cf-cast-unified-mating-plane-recon]]. Defaults
/// to `enabled = true` with
/// [`cf_cast::dowel_hole::DowelHoleSpec::iter1`] (3 mm diameter ×
/// 4 holes × 5 mm depth × 8 mm outboard offset × 0.1 mm clearance).
/// Per-field overrides surfaced as optionals; absent → falls back to
/// the iter1 default for that field.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DowelHoleConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::dowel_hole::DowelHoleKind::None`] (no dowel holes).
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Dowel diameter (meters). `None` → 3 mm (iter1 default).
    #[serde(default)]
    pub diameter_m: Option<f64>,
    /// Radial clearance between dowel and hole wall (meters).
    /// `None` → 0.1 mm (iter1 default).
    #[serde(default)]
    pub clearance_m: Option<f64>,
    /// Hole depth PER HALF (meters). `None` → 5 mm (iter1 default).
    #[serde(default)]
    pub depth_m: Option<f64>,
    /// Number of dowels arc-length-equal-spaced around the silhouette.
    /// `None` → 4 (iter1 default).
    #[serde(default)]
    pub count: Option<u32>,
    /// Radial offset from the body silhouette curve to the dowel
    /// centerline (meters). `None` → 8 mm (iter1 default). Must satisfy
    /// the §M-5-b cross-field invariants in the recon.
    #[serde(default)]
    pub silhouette_outboard_offset_m: Option<f64>,
}

impl Default for DowelHoleConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            diameter_m: None,
            clearance_m: None,
            depth_m: None,
            count: None,
            silhouette_outboard_offset_m: None,
        }
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

    /// Slice 9 cross-field gate — pre-lift check that exactly one of
    /// `[design]` / `[[layers]]` is set. Pure (no I/O); the actual
    /// design-TOML lift happens in `crate::run` between this method
    /// and [`validate_after_layer_source`](Self::validate_after_layer_source).
    ///
    /// Caller pattern (see `crate::run`):
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
            self.cast.wall_thickness_m.is_finite() && self.cast.wall_thickness_m > 0.0,
            "cast.wall_thickness_m = {} must be finite and > 0",
            self.cast.wall_thickness_m
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

        // S4 of the FDM-friendly geometry arc retired the pre-S4
        // `plug_pins.pin_length_m` override + the wall-thickness ↔
        // pin-length cross-field gate. The gate guarded against the
        // pre-S4 cylindrical plug-shaft poking out past the
        // cup-piece outer face (cup-wall penetration); the post-S4
        // plug-floor lock is INTERIOR to the cavity (recessed
        // socket, NOT a through-hole — see recon-1 §G-1), so the
        // workshop-meaningful constraint is now socket-recess depth
        // ≤ wall thickness − 1 mm cup material at the cap-plane
        // outer face. With the spec-pinned 4 mm half-length default
        // and the workshop's 5 mm `cast.wall_thickness_m` floor,
        // the constraint is satisfied by construction; the gate is
        // omitted until S7 calibration surfaces a workshop-physical
        // need.

        // S2 of the seam-flange arc per recon §F-6. Per-field
        // finiteness + positivity + cross-field gasket-disjoint
        // invariant (recon §F-4). Skipped when flange disabled.
        if self.flange.enabled {
            for (label, v) in [
                ("flange.width_m", self.flange.width_m),
                ("flange.thickness_m", self.flange.thickness_m),
                ("flange.inner_offset_m", self.flange.inner_offset_m),
            ] {
                if let Some(value) = v {
                    ensure!(
                        value.is_finite() && value > 0.0,
                        "cast.toml: {label} = {value} must be finite and > 0"
                    );
                }
            }
            // Cross-field gate (recon §F-4): when both gasket + flange
            // are enabled, the flange's lateral inner edge MUST sit
            // outside the gasket strip (which traces body_dist=0 with
            // half-width = GasketSpec::iter1().cross_section_width_m
            // / 2). The post-S2 gasket arc doesn't surface a
            // channel-width TOML override, so the iter1 width (1.5 mm
            // → half = 0.75 mm) is the authoritative half-width. The
            // flange `inner_offset_m` is the bare gap from
            // body_dist=0 to the flange's inner edge; it must
            // STRICTLY exceed the half-width so there's lateral air
            // between gasket and flange.
            if self.gasket.enabled {
                let iter1 = cf_cast::FlangeSpec::iter1();
                let inner_offset = self
                    .flange
                    .inner_offset_m
                    .unwrap_or(iter1.flange_inner_offset_m);
                let gasket_half_width = cf_cast::GasketSpec::iter1().cross_section_width_m / 2.0;
                ensure!(
                    inner_offset > gasket_half_width,
                    "cast.toml: flange.inner_offset_m = {inner_offset} must exceed half \
                     the gasket channel width ({gasket_half_width}) when both [gasket] and \
                     [flange] are enabled (recon §F-4 gasket-disjoint invariant). Either \
                     widen the flange inner offset, narrow the gasket, or disable one of \
                     the two blocks."
                );
            }
        }

        // §M-S2 cross-field gate (recon §M-5-b): when both flange +
        // dowel_hole are enabled, the dowel hole must fit inside the
        // flange band with FDM-floor wall thickness on both sides
        // (inboard toward gasket channel + outboard toward flange
        // outer edge). The iter1 defaults satisfy this with comfortable
        // 4.4 mm margins; per-field overrides in TOML can violate it,
        // so gate at parse time. Cold-read finding 2026-05-27.
        if self.dowel_hole.enabled && self.flange.enabled {
            let flange_iter1 = cf_cast::FlangeSpec::iter1();
            let dowel_iter1 = cf_cast::dowel_hole::DowelHoleSpec::iter1();
            let flange_inner_offset = self
                .flange
                .inner_offset_m
                .unwrap_or(flange_iter1.flange_inner_offset_m);
            let flange_width = self.flange.width_m.unwrap_or(flange_iter1.flange_width_m);
            let dowel_diameter = self.dowel_hole.diameter_m.unwrap_or(dowel_iter1.diameter_m);
            let dowel_clearance = self
                .dowel_hole
                .clearance_m
                .unwrap_or(dowel_iter1.clearance_m);
            let dowel_offset = self
                .dowel_hole
                .silhouette_outboard_offset_m
                .unwrap_or(dowel_iter1.silhouette_outboard_offset_m);
            let hole_outer_radius = dowel_diameter / 2.0 + dowel_clearance;
            // 1 mm FDM-friendly wall floor (the workshop's iter-1
            // print pipeline can resolve down to ~0.4 mm bead but
            // <1 mm walls are squish/strip risks).
            const FDM_WALL_FLOOR_M: f64 = 0.001;
            let inboard_wall = dowel_offset - hole_outer_radius - flange_inner_offset;
            ensure!(
                inboard_wall >= FDM_WALL_FLOOR_M,
                "cast.toml: dowel-hole inboard wall thickness = \
                 {inboard_wall_mm:.3} mm violates the {floor_mm:.1} mm FDM \
                 floor (silhouette_outboard_offset_m {dowel_offset:.4} − \
                 hole_outer_radius {hole_outer_radius:.4} − \
                 flange.inner_offset_m {flange_inner_offset:.4}). The dowel \
                 hole would pierce the gasket channel. Move dowels outboard \
                 (raise dowel_hole.silhouette_outboard_offset_m), shrink the \
                 dowel (dowel_hole.diameter_m), or narrow the flange inner \
                 offset (flange.inner_offset_m).",
                inboard_wall_mm = inboard_wall * 1000.0,
                floor_mm = FDM_WALL_FLOOR_M * 1000.0,
            );
            let outboard_wall = flange_width - dowel_offset - hole_outer_radius;
            ensure!(
                outboard_wall >= FDM_WALL_FLOOR_M,
                "cast.toml: dowel-hole outboard wall thickness = \
                 {outboard_wall_mm:.3} mm violates the {floor_mm:.1} mm FDM \
                 floor (flange.width_m {flange_width:.4} − \
                 silhouette_outboard_offset_m {dowel_offset:.4} − \
                 hole_outer_radius {hole_outer_radius:.4}). The dowel hole \
                 would breach the flange outer perimeter. Move dowels \
                 inboard (lower dowel_hole.silhouette_outboard_offset_m) or \
                 widen the flange (flange.width_m).",
                outboard_wall_mm = outboard_wall * 1000.0,
                floor_mm = FDM_WALL_FLOOR_M * 1000.0,
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
        // Post-S4: the wall-thickness ↔ pin-length cross-field
        // gate that previously made all-defaults configs fail
        // validation was retired with the plug-shaft cup-wall
        // penetration mechanism (see config.rs validate_after_layer_source
        // §"S4 of the FDM-friendly geometry arc" comment). This
        // test scopes itself to field-by-field parsing of defaults;
        // the all-defaults validation gate is verified below.
        let cfg = CastConfig::from_toml_str(minimal_config_text()).unwrap();
        assert_eq!(cfg.layers.len(), 1);
        assert_eq!(cfg.layers[0].material, "ECOFLEX_00_30");
        // Defaults from CastDefaults.
        assert!((cfg.cast.mesh_cell_size_m - 0.003).abs() < 1e-12);
        assert!((cfg.cast.mass_budget_kg - DEFAULT_MASS_BUDGET_KG).abs() < 1e-12);
        assert!((cfg.cast.wall_thickness_m - 0.005).abs() < 1e-12);
        assert_eq!(cfg.cast.split_normal, [0.0, 0.0, -1.0]);
        assert!((cfg.cast.piece_min_wall_mm - 0.1).abs() < 1e-12);
        assert_eq!(cfg.cast.output_dir, PathBuf::from("out"));
        // Block defaults.
        assert!(cfg.plug_pins.enabled);
        assert!(cfg.pour_gate.enabled);
        // S3 seam-gasket-mold arc default: enabled + Ecoflex (None →
        // GasketMaterial::Ecoflex0030 in derive).
        assert!(cfg.gasket.enabled);
        assert!(cfg.gasket.material.is_none());
        // S2 seam-flange arc default: enabled + all iter1 overrides
        // None (derive::resolve_flange_spec falls back to
        // FlangeSpec::iter1() per field).
        assert!(cfg.flange.enabled);
        assert!(cfg.flange.width_m.is_none());
        assert!(cfg.flange.thickness_m.is_none());
        assert!(cfg.flange.inner_offset_m.is_none());
    }

    #[test]
    fn parses_gasket_block_disabled() {
        // S3: `[gasket] enabled = false` parses cleanly + the
        // master toggle propagates. Material absent (None) is the
        // documented fallback to iter1 default at derive time.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[gasket]
enabled = false
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        assert!(!cfg.gasket.enabled);
        assert!(cfg.gasket.material.is_none());
    }

    #[test]
    fn parses_gasket_block_dragon_skin_override() {
        // S3: workshop iter-3 fallback path — material override to
        // Dragon Skin 10A when Ecoflex compresses too freely per
        // recon §G-1. derive::resolve_gasket_material handles the
        // string → enum lift.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[gasket]
material = "DRAGON_SKIN_10A"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        assert!(cfg.gasket.enabled);
        assert_eq!(cfg.gasket.material.as_deref(), Some("DRAGON_SKIN_10A"));
    }

    #[test]
    fn parses_flange_block_disabled() {
        // S2 seam-flange arc: `[flange] enabled = false` parses cleanly
        // + master toggle propagates. Overrides absent (None) → iter1
        // defaults at derive time.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[flange]
enabled = false
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!(!cfg.flange.enabled);
        assert!(cfg.flange.width_m.is_none());
    }

    #[test]
    fn parses_flange_block_with_overrides() {
        // S2: partial override — width + thickness picked at non-iter1
        // values, inner_offset left as iter1 default. Validates the
        // optional-per-field design (recon §F-6).
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[flange]
width_m = 0.020
thickness_m = 0.005
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!(cfg.flange.enabled);
        assert!((cfg.flange.width_m.unwrap() - 0.020).abs() < 1e-12);
        assert!((cfg.flange.thickness_m.unwrap() - 0.005).abs() < 1e-12);
        assert!(cfg.flange.inner_offset_m.is_none());
    }

    #[test]
    fn rejects_flange_inner_offset_overlapping_gasket() {
        // S2 cross-field gate (recon §F-4): with both gasket + flange
        // enabled, `flange.inner_offset_m` MUST exceed half the gasket
        // channel width (1.5 mm / 2 = 0.75 mm at iter1). A 0.5 mm
        // override falls below the threshold → rejected at validate
        // time.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[flange]
inner_offset_m = 0.0005
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg
            .validate()
            .expect_err("flange inner_offset below gasket half-width must fail");
        let s = err.to_string();
        assert!(s.contains("inner_offset_m"), "unexpected error: {s}");
        assert!(s.contains("gasket"), "unexpected error: {s}");
    }

    #[test]
    fn accepts_flange_inner_offset_overlapping_when_gasket_disabled() {
        // S2 cross-field gate scope: with the gasket disabled there's
        // no gasket strip to clear, so a small `inner_offset_m`
        // override is allowed. Pins the gate's "skip when gasket off"
        // branch.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[gasket]
enabled = false

[flange]
inner_offset_m = 0.0005
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!(!cfg.gasket.enabled);
        assert!(cfg.flange.enabled);
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
wall_thickness_m = 0.030
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
enabled = true

[pour_gate]
enabled = true
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert_eq!(cfg.layers.len(), 3);
        assert_eq!(cfg.layers[0].display_name.as_deref(), Some("Inner Ecoflex"));
        assert!((cfg.cast.mesh_cell_size_m - 0.004).abs() < 1e-12);
        assert_eq!(cfg.cast.output_dir, PathBuf::from("iter1_out"));
        assert!(cfg.plug_pins.enabled);
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
        // Post-S4 the wall-thickness ↔ pin-length cross-field gate
        // is gone (the plug-floor lock is interior to the cavity,
        // not a through-shaft), so this test no longer needs
        // `[plug_pins] enabled = false` to scope itself to material-
        // override validation. Keep the override on for parity with
        // production iter-1 cast.toml.
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
    fn rejects_non_positive_wall_thickness() {
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[cast]
wall_thickness_m = 0.0

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg.validate().expect_err("zero wall_thickness must fail");
        assert!(
            err.to_string().contains("wall_thickness_m"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn accepts_all_defaults_post_s4() {
        // Post-S4 of the FDM-friendly geometry arc the wall-thickness
        // ↔ pin-length cross-field gate is retired (the plug-floor
        // lock is interior to the cavity, NOT a through-shaft, so
        // no cup-wall penetration). The pre-S4 tests
        // `rejects_wall_thinner_than_default_pin_length` /
        // `accepts_wall_thickness_matching_pin_length_override`
        // were retired with the cross-field gate. This test pins
        // the post-S4 contract: all-defaults validates clean (no
        // gate fires).
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
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
