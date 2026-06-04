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
    /// defaults". Set `enabled = false` to disable; `gate_radius_m` is the
    /// per-silicone sprue-size knob (widens gate + funnel together — bump it for a
    /// thick silicone). See [[project-cf-cast-per-silicone-sprue-funnel]].
    #[serde(default)]
    pub pour_gate: PourGateConfig,
    /// Per-layer gasket mold override (**default = disabled** since the S4.5
    /// demand-flange default flip — the demand flange self-seals, so the gasket is
    /// opt-in). Set `enabled = true` (with `GasketSpec::iter1()` geometry + Ecoflex
    /// 00-30) for a gasketed seal — typically alongside `kind = "plate"` or a
    /// widened demand land. S3 of the seam-gasket-mold arc.
    #[serde(default)]
    pub gasket: GasketConfig,
    /// Seam-plane flange override (default = enabled, `kind = "demand"` —
    /// the scalloped demand flange, recon §4/§7.6). Absence of the table
    /// means "enabled with demand-flange iter1 defaults". Set `enabled = false`
    /// to disable, or `kind = "plate"` for the legacy uniform band. S2 of the
    /// seam-flange arc per recon §F-6 + S4.5.
    #[serde(default)]
    pub flange: FlangeConfig,
    /// Symmetric dowel-hole registration override (default = enabled
    /// with [`cf_cast::dowel_hole::DowelHoleSpec::iter1`] geometry).
    /// Absence of the table means "enabled with iter1 defaults". Set
    /// `enabled = false` to disable. §M-S2 of the unified-mating-plane
    /// arc per [[project-cf-cast-unified-mating-plane-recon]].
    #[serde(default)]
    pub dowel_hole: DowelHoleConfig,
    /// M5 through-bolt clamp pattern override (default = enabled with
    /// [`cf_cast::bolt_pattern::BoltPatternSpec::iter1`] geometry).
    /// Absence of the table means "enabled with iter1 defaults". Set
    /// `enabled = false` to disable. §B of
    /// [[project-cf-cast-flange-continuity-bolt-pattern-recon]].
    #[serde(default)]
    pub bolt_pattern: BoltPatternConfig,
    /// Interior-canal feature override (default = DISABLED). When
    /// `enabled = true`, the layer-0 plug's scan-derived surface gets
    /// parametric grip rings + a frenulum D-section pinch + frenulum-
    /// gated texture + a terminal suction bulb composed on top (the
    /// Canal Interior arc, Candidate A). Baseline girth is unchanged —
    /// tightness stays owned by the layer/inset machinery. Mutually
    /// exclusive with `scan_mesh_direct_plug_layer_0` (both rewrite the
    /// layer-0 plug). See [`cf_cast::CanalSpec`].
    #[serde(default)]
    pub canal: CanalConfig,
}

impl CastConfig {
    /// Build a config for the guided-wizard path: a cleaned scan + its
    /// `.prep.toml` + a `.design.toml` (the layer stack), with the
    /// **validated workshop production recipe** — the settings the
    /// base_mold class of part actually casts cleanly with (mirrors the
    /// out-of-tree `cast.base_mold.canaloff.toml`, which is validated):
    ///
    /// - `planar_seam = true` — the cup halves print mating-face-down, so
    ///   the seam must be flat (the hard flat-mating-face constraint); also
    ///   a prerequisite for the apex pour.
    /// - `split_normal = [1, 0, 0]` — split into left/right halves across
    ///   the part's width. The scan stands apex-up after step-2 PCA
    ///   orientation, so the default top/bottom `[0,0,-1]` split is wrong.
    /// - `pour_gate.apex_axial = true` — a single axial pour bore at the
    ///   dome apex (the organic-parts pour), not the iter-1 V-shape.
    /// - plug-floor lock pins on; gasket off (the demand flange self-seals);
    ///   canal off (the scan-specific frenulum canal is a separate opt-in,
    ///   not a wizard default — base_mold casts cleanly without it).
    ///
    /// `mesh_cell_size_m` is the marching-cubes resolution (time ↔ quality).
    /// The layer stack is taken from the design, so `[[layers]]` stays empty
    /// (the design-set ⟺ empty-layers gate holds by construction).
    ///
    /// Paths are relative to the cast run's `base_dir` (typically the scan
    /// folder), matching how `run_with_config` resolves them.
    #[must_use]
    pub fn for_design(
        cleaned_stl: PathBuf,
        prep_toml: PathBuf,
        design_toml: PathBuf,
        mesh_cell_size_m: f64,
    ) -> Self {
        Self {
            scan: ScanConfig {
                cleaned_stl,
                prep_toml,
            },
            cast: CastDefaults {
                mesh_cell_size_m,
                planar_seam: true,
                split_normal: [1.0, 0.0, 0.0],
                ..CastDefaults::default()
            },
            design: Some(DesignSourceConfig { path: design_toml }),
            layers: Vec::new(),
            plug_pins: PlugPinConfig::default(), // enabled by default
            pour_gate: PourGateConfig {
                apex_axial: true,
                ..PourGateConfig::default()
            },
            gasket: GasketConfig::default(),
            flange: FlangeConfig::default(),
            dowel_hole: DowelHoleConfig::default(),
            bolt_pattern: BoltPatternConfig::default(),
            canal: CanalConfig::default(),
        }
    }
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
    /// Collapse the cup seam to a single FLAT VERTICAL plane instead of
    /// the curve-following ribbon (S1 of `CF_CAST_ORGANIC_PARTS_RECON.md`).
    /// For organic/curved parts where the cup halves print
    /// mating-face-down and need a guaranteed-flat seam to seal. Off by
    /// default — the curve-following seam is bit-preserved for every
    /// cast.toml that doesn't opt in. Re-level the scan in cf-scan-prep
    /// first so the vertical cut bisects cleanly.
    #[serde(default)]
    pub planar_seam: bool,
    /// When `planar_seam` is on, FIT the flat seam to the body (apex-anchored,
    /// balance-swept — item A §4.1) so it follows a leaning part and bisects the
    /// dome evenly instead of skimming it into a sliver. **Default `true`** —
    /// the fit is the better seam (the flange/bolt/dowel silhouette is built in
    /// the fitted plane, so it builds manifold at any orientation). Set `false`
    /// to force the legacy binormal-flatten seam (the escape hatch); needs the
    /// scan's `.prep.toml [caps]` for the apex anchor, else falls back to the
    /// binormal seam automatically.
    #[serde(default = "default_true")]
    pub planar_seam_fit: bool,
    /// §MA-S1b: flatten the cavity floor with exact post-MC CSG (dip-fill
    /// union + bump-cut subtract at the cap plane) so the floor↔socket↔
    /// seam junction is crisp instead of marching-cubes-rounded. Needs
    /// `planar_seam` + a pour-end cap hint (the scan `.prep.toml [caps]`);
    /// a no-op otherwise. Off by default — opt-in per part, because the
    /// flattening footprint is the cap-plane cross-section's convex hull
    /// (correct for convex cavity floors, over-covers non-convex ones).
    #[serde(default)]
    pub flat_cavity_floor: bool,
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
            planar_seam: false,
            planar_seam_fit: true,
            flat_cavity_floor: false,
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
    /// Organic-parts opt-in: when `true`, use the single AXIAL pour
    /// bore at the dome apex on the seam ([`cf_cast::PourGateLayout::ApexAxial`])
    /// instead of the iter-1 V-shape — and the funnel un-bends to a
    /// straight nipple, and the bolt pattern brackets the pour bore
    /// rather than dropping nearby bolts. No vent is modeled (the
    /// workshop hand-drills tiny carbide vents at the high spots).
    /// Default `false` (existing casts byte-identical). Organic-parts
    /// arc §4.3, 2026-05-29.
    #[serde(default)]
    pub apex_axial: bool,
    /// Pour-leg (sprue) radius (meters) — **the per-silicone sprue-size knob**.
    /// `None` → [`cf_cast::PourGateSpec::iter1`]'s 5 mm (Ø10 mm bore). Bump it for a
    /// thick silicone (e.g. Dragon Skin) that pours slowly. The pour THROAT depends on
    /// the layout: **apex-axial** (the integral split funnel) has a lumen continuous
    /// into the bore — no inserted nipple — so the throat IS the bore Ø (Ø10 mm at the
    /// default); **V-at-dome** (the legacy separate funnel) derives a narrower nipple
    /// throat (≈ Ø6.5 mm = bore − 3.5 mm wall) off this. Widening the apex bore also
    /// re-shapes the demand-flange seal land at the apex (the solver handles the
    /// bracket spacing; re-eyeball the apex). See
    /// [[project-cf-cast-per-silicone-sprue-funnel]].
    #[serde(default)]
    pub gate_radius_m: Option<f64>,
    /// Vent-leg radius (meters). `None` → iter1's 3 mm (Ø6 mm). Used by the V-shape
    /// layout; the apex-axial layout models no vent (hand-drilled).
    #[serde(default)]
    pub vent_radius_m: Option<f64>,
}

impl Default for PourGateConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            apex_axial: false,
            gate_radius_m: None,
            vent_radius_m: None,
        }
    }
}

/// `[gasket]` block — per-layer gasket mold toggle + material pick.
/// Maps to [`cf_cast::GasketKind`].
///
/// S3 of the seam-gasket-mold arc per recon §G-7. **Defaults to `enabled = false`
/// since the S4.5 demand-flange default flip:** the default flange is now the
/// demand flange, whose continuous seal-ring land seals PLA-on-PLA on its own, so a
/// silicone gasket is opt-in (a `kind = "plate"` seal, or a deliberately-widened
/// demand land — the demand land's 0.5 mm default would otherwise pinch the gasket,
/// recon §F-4). When enabled it uses [`cf_cast::GasketSpec::iter1`] geometry +
/// Ecoflex 00-30 (flip `material = "DRAGON_SKIN_10A"` if the Ecoflex pour
/// compresses too freely). Cross-section is pinned trapezoidal at iter1.
// Default (enabled=false since the S4.5 demand-flange flip; material=None) is the
// per-field type default, so it derives — see the struct doc for the rationale.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GasketConfig {
    /// Master toggle. **Default `false`** (the demand flange self-seals; see the
    /// struct doc). When `true`, per-layer gasket molds are emitted; when `false`,
    /// the bridge passes [`cf_cast::GasketKind::None`].
    #[serde(default)]
    pub enabled: bool,
    /// Gasket material — Smooth-On silicone product key.
    /// Recognized values: `"ECOFLEX_00_30"` (iter1 default),
    /// `"DRAGON_SKIN_10A"`. Maps to [`cf_cast::GasketMaterial`].
    /// Absent in the TOML → falls back to the iter1 default
    /// (Ecoflex 00-30 per recon §G-1).
    #[serde(default)]
    pub material: Option<String>,
}

/// `[flange]` block — seam-plane clampable flange toggle + `kind` selector +
/// geometry overrides. Maps to [`cf_cast::FlangeKind`].
///
/// S2 of the seam-flange arc per recon §F-6 + S4.5 demand flange (recon §4).
/// Defaults to `enabled = true` with `kind = "demand"` (the scalloped seal-ring +
/// per-fastener bosses — the end-state print target, recon §7.6); set
/// `kind = "plate"` for the legacy uniform band
/// ([`cf_cast::FlangeSpec::iter1`]: 20 mm width × 4 mm thickness per half × 2 mm
/// inner offset). Per-field overrides are surfaced as optionals (Plate fields:
/// `width_m`/`inner_offset_m`; Demand fields: `land_width_m`/`land_inner_offset_m`/
/// `web_width_m`/`boss_wall_margin_m`; `thickness_m` is shared); absent → the
/// matching iter1 default. The cross-field gasket-disjoint invariant (recon §F-4)
/// is enforced at [`CastConfig::validate_after_layer_source`] time when both the
/// gasket and a flange are enabled, **kind-aware**: it checks the active flange's
/// inner edge against the gasket half-width — the Plate `inner_offset_m` for
/// `kind = "plate"`, the Demand `land_inner_offset_m` for the default demand kind.
/// (The demand land already seals PLA-on-PLA, so the default demand land + a gasket
/// is rejected as redundant unless the land is widened past the half-width.)
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlangeConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::FlangeKind::None`] (no seam-plane flange; cup
    /// halves clamped via whatever contoured surface they have).
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Flange kind: `"plate"` (uniform band) or `"demand"` (scalloped seal-ring +
    /// per-fastener bosses, generated to fit the placed fasteners — recon §4).
    /// `None` → `"demand"` (S4.5/3 default flip, recon §7.6 — the demand flange is
    /// the end-state / print target). Set `kind = "plate"` to opt back into the
    /// legacy uniform band.
    #[serde(default)]
    pub kind: Option<String>,
    /// **Plate kind.** Lateral extent (meters) from `inner_offset_m` outward in the
    /// seam plane. `None` → falls back to
    /// [`cf_cast::FlangeSpec::iter1`]'s 20 mm default.
    #[serde(default)]
    pub width_m: Option<f64>,
    /// Half-thickness (meters) perpendicular to the seam plane (BOTH kinds).
    /// Closed flange-zone thickness ≈ 2 × this. `None` → 4 mm per
    /// half (iter1 default).
    #[serde(default)]
    pub thickness_m: Option<f64>,
    /// **Plate kind.** Lateral gap (meters) between body cavity perimeter and
    /// flange inner edge. Must exceed half the gasket channel width
    /// when the gasket is enabled (recon §F-4). `None` → 2 mm
    /// (iter1 default).
    #[serde(default)]
    pub inner_offset_m: Option<f64>,
    /// **Demand kind.** Continuous seal-land width (meters). `None` → 6 mm
    /// ([`cf_cast::DemandFlangeSpec::iter1`]).
    #[serde(default)]
    pub land_width_m: Option<f64>,
    /// **Demand kind.** Seal-land inboard start (meters) from the body perimeter.
    /// `None` → 0.5 mm (gasket-None hug-tight, recon §4.1 N2).
    #[serde(default)]
    pub land_inner_offset_m: Option<f64>,
    /// **Demand kind.** Tadpole spoke (web) width (meters). `None` → 4 mm.
    #[serde(default)]
    pub web_width_m: Option<f64>,
    /// **Demand kind.** PLA wall a boss keeps beyond the fastener footprint
    /// (meters): `boss_r = footprint + this`. `None` → 2 mm.
    #[serde(default)]
    pub boss_wall_margin_m: Option<f64>,
}

impl Default for FlangeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            kind: None,
            width_m: None,
            thickness_m: None,
            inner_offset_m: None,
            land_width_m: None,
            land_inner_offset_m: None,
            web_width_m: None,
            boss_wall_margin_m: None,
        }
    }
}

/// `[dowel_hole]` block — symmetric dowel-hole registration toggle
/// + geometry overrides. Maps to [`cf_cast::dowel_hole::DowelHoleKind`].
///
/// §M-S2 of [[project-cf-cast-unified-mating-plane-recon]]. Defaults
/// to `enabled = true` with
/// [`cf_cast::dowel_hole::DowelHoleSpec::iter1`] (3 mm diameter ×
/// 5 mm depth per half × 0.1 mm radial clearance). The hole COUNT and
/// seam-loop position are **not** config knobs — the seam-placement
/// solver derives them per layer (dowels at the body's long-axis
/// extremes, typically 2; the fixed `count`/`offset` fields were
/// removed in S5c). Per-field overrides surfaced as optionals; absent
/// → falls back to the iter1 default for that field.
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
}

impl Default for DowelHoleConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            diameter_m: None,
            clearance_m: None,
            depth_m: None,
        }
    }
}

/// `[bolt_pattern]` block — M5 through-bolt clamp pattern toggle
/// plus geometry overrides. Maps to
/// [`cf_cast::bolt_pattern::BoltPatternKind`].
///
/// §B of [[project-cf-cast-flange-continuity-bolt-pattern-recon]].
/// Defaults to `enabled = true` with
/// [`cf_cast::bolt_pattern::BoltPatternSpec::iter1`] (5.5 mm M5
/// clearance). Bolt *count* and radial offset are no longer config
/// knobs — the seam-placement solver derives them per layer (even
/// spacing at ≤30 mm pitch, variable offset from the flange band +
/// washer clearance). Per-field overrides surfaced as optionals;
/// absent → falls back to the iter1 default for that field.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoltPatternConfig {
    /// Master toggle. When `false`, the bridge passes
    /// [`cf_cast::bolt_pattern::BoltPatternKind::None`] (no bolt holes).
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Bolt clearance hole diameter (meters). `None` → 5.5 mm
    /// (M5 ISO 273 medium fit).
    #[serde(default)]
    pub clearance_diameter_m: Option<f64>,
}

impl Default for BoltPatternConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            clearance_diameter_m: None,
        }
    }
}

/// `[canal]` block — interior-canal feature toggle + geometry
/// overrides. Maps to [`cf_cast::CanalSpec`].
///
/// Canal Interior arc (Candidate A, 2026-05-28). Defaults to
/// `enabled = false` — absence of the table (or `enabled = false`)
/// leaves the layer-0 plug exactly as today (scan-derived negative).
/// When enabled, fields left unset fall back to
/// [`cf_cast::CanalSpec::iter1`]. Set `suction_bulge_m = 0.0` to drop
/// the suction bulb, `texture_amplitude_m = 0.0` to drop ribs, etc.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CanalConfig {
    /// Master toggle. `false` (default) → plug unchanged from the
    /// scan-derived baseline.
    #[serde(default)]
    pub enabled: bool,
    /// Frenulum direction in the cast world frame (asymmetry axis).
    /// `None` → [0, 1, 0] (iter1 default).
    #[serde(default)]
    pub frenulum_dir: Option<[f64; 3]>,
    /// Frenulum-gated texture rib amplitude (meters). `None` → 1.5 mm.
    /// `Some(0.0)` disables texture.
    #[serde(default)]
    pub texture_amplitude_m: Option<f64>,
    /// Texture rib pitch (meters). `None` → 8 mm.
    #[serde(default)]
    pub texture_pitch_m: Option<f64>,
    /// Frenulum-side D-section pinch depth (meters). `None` → 1.5 mm.
    /// `Some(0.0)` disables the asymmetry.
    #[serde(default)]
    pub dsection_depth_m: Option<f64>,
    /// Terminal suction-bulb outward bulge (meters). `None` → 3 mm.
    /// `Some(0.0)` disables the bulb (and its wall-thinning risk).
    #[serde(default)]
    pub suction_bulge_m: Option<f64>,
    /// Marching-cubes cell size for the layer-0 plug only (meters).
    /// `None` → 0.5 mm. Finer than the global `mesh_cell_size_m` so the
    /// ~1.5 mm texture survives; cups stay coarse. See the S0 probe.
    #[serde(default)]
    pub plug_mesh_cell_size_m: Option<f64>,
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
        // The apex pour bore is projected onto the seam plane so it splits the
        // flange into open half-troughs; that only works when the seam is a flat
        // plane (`[cast].planar_seam`). Without it the bore lands on the
        // per-segment curve-following seam, can't re-register, and the sprue
        // becomes a blind hole on one half. Require the flat seam.
        ensure!(
            !self.pour_gate.apex_axial || self.cast.planar_seam,
            "cast.toml: [pour_gate].apex_axial requires [cast].planar_seam = true \
             (the apex bore must lie on a flat seam to split the flange cleanly)"
        );
        // The §MA-S1b cavity-floor flattening slab is seam-clipped to the flat
        // seam plane; `build_cavity_floor_slab_transforms` silently no-ops
        // without `planar_seam` (and without a pour-end cap hint). Fail fast on
        // the config-time half of that dependency — the same fail-fast pattern
        // as `apex_axial` above — so `flat_cavity_floor = true` can never be a
        // silent no-op. (The cap hint comes from the scan `.prep.toml [caps]`,
        // not the cast TOML, so it stays a runtime concern, not a config gate.)
        ensure!(
            !self.cast.flat_cavity_floor || self.cast.planar_seam,
            "cast.toml: [cast].flat_cavity_floor requires [cast].planar_seam = true \
             (the floor slab is seam-clipped to a flat seam plane; without it the \
             flattening is silently skipped)"
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

        // Pour-gate radius knobs (the per-silicone sprue-size lever): finite +
        // positive. The funnel derives its nipple bore off gate_radius_m, so a
        // too-small gate surfaces as the funnel's F4 ThinWall gate (the realistic
        // use is WIDENING for a thick silicone). See
        // [[project-cf-cast-per-silicone-sprue-funnel]].
        if self.pour_gate.enabled {
            for (label, v) in [
                ("pour_gate.gate_radius_m", self.pour_gate.gate_radius_m),
                ("pour_gate.vent_radius_m", self.pour_gate.vent_radius_m),
            ] {
                if let Some(value) = v {
                    ensure!(
                        value.is_finite() && value > 0.0,
                        "cast.toml: {label} = {value} must be finite and > 0"
                    );
                }
            }
        }

        // S2 of the seam-flange arc per recon §F-6. Per-field
        // finiteness + positivity + cross-field gasket-disjoint
        // invariant (recon §F-4). Skipped when flange disabled.
        if self.flange.enabled {
            // Reject an unknown flange kind early with a typo-friendly message.
            if let Some(kind) = self.flange.kind.as_deref() {
                ensure!(
                    matches!(kind, "plate" | "demand"),
                    "cast.toml: flange.kind = {kind:?} is unknown. Recognized: \
                     \"plate\", \"demand\"."
                );
            }
            for (label, v) in [
                ("flange.width_m", self.flange.width_m),
                ("flange.thickness_m", self.flange.thickness_m),
                ("flange.inner_offset_m", self.flange.inner_offset_m),
                ("flange.land_width_m", self.flange.land_width_m),
                (
                    "flange.land_inner_offset_m",
                    self.flange.land_inner_offset_m,
                ),
                ("flange.web_width_m", self.flange.web_width_m),
                ("flange.boss_wall_margin_m", self.flange.boss_wall_margin_m),
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
            // inner edge is the bare gap from body_dist=0 to where the
            // flange begins; it must STRICTLY exceed the half-width so
            // there's lateral air between gasket and flange.
            //
            // The check is **kind-aware** (S4.5/3 default flip — the default flange
            // is now `demand`): validate whichever inner offset the ACTIVE flange
            // kind builds, resolved the same way derive.rs maps it (`Some("plate")
            // => Plate, _ => Demand`). The demand seal land defaults to 0.5 mm
            // (DemandFlangeSpec::iter1) which is INSIDE the 0.75 mm half-width, so a
            // gasket + a default demand flange is rejected here — the demand land
            // already seals PLA-on-PLA, so the gasket is redundant; widen
            // `land_inner_offset_m` past the half-width to keep both, or disable one.
            if self.gasket.enabled {
                let gasket_half_width = cf_cast::GasketSpec::iter1().cross_section_width_m / 2.0;
                let (label, inner_offset) = match self.flange.kind.as_deref() {
                    Some("plate") => (
                        "flange.inner_offset_m",
                        self.flange
                            .inner_offset_m
                            .unwrap_or(cf_cast::FlangeSpec::iter1().flange_inner_offset_m),
                    ),
                    // None or "demand" → the demand flange (the default).
                    _ => (
                        "flange.land_inner_offset_m",
                        self.flange
                            .land_inner_offset_m
                            .unwrap_or(cf_cast::DemandFlangeSpec::iter1().land_inner_offset_m),
                    ),
                };
                ensure!(
                    inner_offset > gasket_half_width,
                    "cast.toml: {label} = {inner_offset} must exceed half the gasket \
                     channel width ({gasket_half_width}) when both [gasket] and [flange] \
                     are enabled (recon §F-4 gasket-disjoint invariant). The demand \
                     flange's seal land already seals PLA-on-PLA, so a gasket is \
                     redundant — either disable [gasket], widen the inner offset past \
                     the half-width, or use `kind = \"plate\"`."
                );
            }
        }

        // §B cross-field gate: bolt-pattern requires a flange to clamp
        // through. The former inboard/outboard wall + washer-cup-wall-step
        // offset validators were deleted in S5c — the seam-placement
        // solver supersedes them: its `d_floor = wall + washer + margin`
        // + flange-band feasibility guarantee by construction what those
        // gates checked (recon §7.5 D2). A too-narrow flange now surfaces
        // as the solver's `cross_layer_snap` "dropped N" warn (Risk #1)
        // rather than a config-parse failure.
        if self.bolt_pattern.enabled {
            ensure!(
                self.flange.enabled,
                "cast.toml: [bolt_pattern] is enabled but [flange] is \
                 disabled. M5 through-bolts clamp through the flange — \
                 without a flange there is no material to bolt. Either \
                 enable [flange] or disable [bolt_pattern]."
            );
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn for_design_builds_a_valid_design_sourced_config() {
        let c = CastConfig::for_design(
            PathBuf::from("base_mold.cleaned.stl"),
            PathBuf::from("base_mold.prep.toml"),
            PathBuf::from("base_mold.design.toml"),
            0.0015,
        );
        assert_eq!(c.scan.cleaned_stl, PathBuf::from("base_mold.cleaned.stl"));
        assert_eq!(c.scan.prep_toml, PathBuf::from("base_mold.prep.toml"));
        assert!(c.design.is_some(), "design-sourced");
        assert!(
            c.layers.is_empty(),
            "layers come from the design, not inline"
        );
        assert!((c.cast.mesh_cell_size_m - 0.0015).abs() < 1e-12);
        // The design-set ⟺ empty-layers cross-field gate holds (the pre-lift
        // check; layers are lifted from the design by the run, then
        // validate_after_layer_source applies).
        c.validate_layer_source()
            .expect("for_design satisfies the layer-source gate");
        // It must also pass the full validate() once we lift a dummy layer —
        // in particular the apex_axial ⟹ planar_seam coupling.
        let mut lifted = c.clone();
        lifted.design = None;
        lifted.layers = vec![LayerConfig {
            thickness_m: 0.005,
            material: "ECOFLEX_00_30".to_string(),
            density_kg_m3: None,
            display_name: None,
            slacker_fraction: None,
        }];
        lifted
            .validate()
            .expect("for_design's geometry recipe is self-consistent");
    }

    #[test]
    fn for_design_matches_the_validated_canaloff_recipe() {
        // The out-of-tree cast.base_mold.canaloff.toml is the validated
        // production recipe that casts base_mold cleanly. for_design must
        // emit the same cast/pour_gate/plug_pins/gasket/canal settings (only
        // the cell size + the canal frenulum_dir, a no-op while off, differ).
        let c = CastConfig::for_design(
            PathBuf::from("base_mold.cleaned.stl"),
            PathBuf::from("base_mold.prep.toml"),
            PathBuf::from("base_mold.design.toml"),
            0.0015,
        );
        assert!(c.cast.planar_seam, "planar seam (flat mating face)");
        assert!(!c.cast.flat_cavity_floor);
        assert_eq!(c.cast.split_normal, [1.0, 0.0, 0.0], "left/right split");
        assert!((c.cast.wall_thickness_m - 0.005).abs() < 1e-12);
        assert!((c.cast.piece_min_wall_mm - 0.1).abs() < 1e-12);
        assert!(!c.cast.scan_mesh_direct_plug_layer_0);
        assert!(c.plug_pins.enabled, "plug-floor lock pins on");
        assert!(c.pour_gate.enabled);
        assert!(c.pour_gate.apex_axial, "apex axial pour bore");
        assert!(!c.gasket.enabled, "gasket off (demand flange self-seals)");
        assert!(!c.canal.enabled, "canal off (scan-specific opt-in)");
    }

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
        // S4.5/3 default flip: the gasket now defaults DISABLED (the default
        // demand flange self-seals; the gasket is opt-in).
        assert!(!cfg.gasket.enabled);
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
        // string → enum lift. `enabled = true` is now explicit (the gasket
        // defaults off since the S4.5 demand-flange flip).
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[gasket]
enabled = true
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

# §B requires flange to clamp through; disable both together when
# disabling flange.
[bolt_pattern]
enabled = false
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!(!cfg.flange.enabled);
        assert!(cfg.flange.width_m.is_none());
        assert!(!cfg.bolt_pattern.enabled);
    }

    #[test]
    fn parses_flange_block_with_overrides() {
        // S2: partial override — width + thickness picked at non-iter1
        // values, inner_offset left as iter1 default. Validates the
        // optional-per-field design (recon §F-6). `kind = "plate"` because
        // width_m/inner_offset_m are PLATE fields (the default kind is now
        // demand, S4.5/3) — and with the default gasket on, the Plate 2 mm
        // inner offset clears the 0.75 mm gasket half-width.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[flange]
kind = "plate"
width_m = 0.020
thickness_m = 0.005
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!(cfg.flange.enabled);
        assert_eq!(cfg.flange.kind.as_deref(), Some("plate"));
        assert!((cfg.flange.width_m.unwrap() - 0.020).abs() < 1e-12);
        assert!((cfg.flange.thickness_m.unwrap() - 0.005).abs() < 1e-12);
        assert!(cfg.flange.inner_offset_m.is_none());
    }

    #[test]
    fn rejects_explicit_gasket_with_default_demand_flange_land() {
        // S4.5/3 + review-fix: the default flange is `demand` with a 0.5 mm seal
        // land, INSIDE the 0.75 mm gasket half-width. The gasket now defaults OFF,
        // so a user who EXPLICITLY re-enables it on the default demand land hits the
        // kind-aware gasket-disjoint gate: the 0.5 mm land would pinch the gasket
        // (and the land already seals on its own), so the combo is rejected.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[gasket]
enabled = true
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg.validate().expect_err(
            "explicit gasket + default demand flange (0.5 mm land < 0.75 mm gasket \
             half-width) must be rejected",
        );
        let msg = err.to_string();
        assert!(
            msg.contains("land_inner_offset_m") && msg.contains("gasket"),
            "error must name the demand land + the gasket-disjoint invariant; got: {msg}"
        );
    }

    #[test]
    fn accepts_explicit_gasket_with_demand_flange_when_land_widened() {
        // The kind-aware gate ALLOWS an explicit gasket + the default demand flange
        // when the seal land is widened past the 0.75 mm gasket half-width
        // (deliberate belt-and-suspenders).
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[gasket]
enabled = true

[flange]
land_inner_offset_m = 0.002
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!(cfg.gasket.enabled);
        assert!(cfg.flange.kind.is_none(), "default kind (demand)");
        assert!((cfg.flange.land_inner_offset_m.unwrap() - 0.002).abs() < 1e-12);
    }

    #[test]
    fn rejects_flange_inner_offset_overlapping_gasket() {
        // S2 cross-field gate (recon §F-4): with both gasket + a PLATE flange
        // enabled, `flange.inner_offset_m` MUST exceed half the gasket channel width
        // (1.5 mm / 2 = 0.75 mm at iter1). A 0.5 mm override falls below the
        // threshold → rejected at validate time. `kind = "plate"` (the default is
        // now demand, which checks `land_inner_offset_m` instead) + an explicit
        // `[gasket] enabled = true` (the gasket now defaults off, S4.5/3).
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.005
material = "ECOFLEX_00_30"

[gasket]
enabled = true

[flange]
kind = "plate"
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
wall_thickness_m = 0.005
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
    fn rejects_nonpositive_pour_gate_radius() {
        // The per-silicone sprue knob must be finite + > 0.
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"

[pour_gate]
gate_radius_m = 0.0
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg
            .validate()
            .expect_err("zero pour_gate.gate_radius_m must fail");
        assert!(
            err.to_string().contains("pour_gate.gate_radius_m"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn parses_pour_gate_radius_overrides() {
        let text = r#"
[scan]
cleaned_stl = "s.stl"
prep_toml = "s.prep.toml"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"

[pour_gate]
gate_radius_m = 0.008
vent_radius_m = 0.004
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!((cfg.pour_gate.gate_radius_m.unwrap() - 0.008).abs() < 1e-12);
        assert!((cfg.pour_gate.vent_radius_m.unwrap() - 0.004).abs() < 1e-12);
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

    #[test]
    fn rejects_bolt_pattern_without_flange() {
        // §B surviving cross-field gate (S5c): bolts clamp through the
        // flange, so [bolt_pattern] enabled with [flange] disabled has
        // no material to bolt. The offset-based wall/washer validators
        // were deleted in S5c (the seam-placement solver supersedes
        // them); this flange-presence check is the one that remains.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"

[flange]
enabled = false

[bolt_pattern]
enabled = true
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        let err = cfg
            .validate()
            .expect_err("bolt_pattern without flange must fail");
        assert!(
            err.to_string().contains("no material to bolt"),
            "expected flange-required error: {err}"
        );
    }

    #[test]
    fn accepts_bolt_pattern_with_flange_at_defaults() {
        // The complement: bolts enabled with the flange present (both
        // default-on) validate cleanly. Regression-anchors that the
        // surviving check doesn't reject the production-default config
        // now that the offset-based validators are gone.
        let text = r#"
[scan]
cleaned_stl = "iter1.cleaned.stl"
prep_toml = "iter1.cleaned.prep.toml"

[[layers]]
thickness_m = 0.006
material = "ECOFLEX_00_30"

[bolt_pattern]
enabled = true
"#;
        let cfg = CastConfig::from_toml_str(text).unwrap();
        cfg.validate().unwrap();
        assert!(cfg.bolt_pattern.enabled);
    }
}
