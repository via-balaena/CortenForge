//! User-dialed device-design state: the cavity inset, the ordered
//! layer stack, the silicone catalog the Layers panel offers.

use bevy::prelude::Resource;

/// Cavity panel state — the user-dialed `inset_m` by which the
/// cavity surface sits INSIDE the scan surface. Casting context: the
/// cavity is the void the appendage slides into; smaller than the
/// scan so the silicone "skin" between cavity and scan stretches
/// over the appendage, providing the snug fit. Stretch strain
/// ≈ `inset_m / local_scan_radius` — too little = sloppy fit, too
/// much = silicone tears.
///
/// Slice 4 v1: a single uniform inset along the entire scan. A
/// later slice may add the insertable-length clip
/// (`docs/ENGINEERING_SUITE_DESIGN.md` § 4) so the cavity covers
/// only the inserted portion, with a tangent-perpendicular end cap.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
pub struct CavityState {
    /// Distance (meters) by which the cavity surface sits inside the
    /// scan surface. The cavity surface is the uniform-offset
    /// isosurface of the cleaned scan's SDF at iso = -`inset_m`.
    pub inset_m: f64,
    /// Whether the cavity mesh entity is drawn this frame. User
    /// toggle in the Cavity panel.
    pub visible: bool,
}

impl CavityState {
    /// Default state. Inset defaults to [`CAVITY_DEFAULT_INSET_M`]
    /// (3 mm) — the minimum-acceptable buildable design: above
    /// Ecoflex's ~2 mm castability threshold, and ~10 % radial
    /// pre-strain on a typical scan cross-section. User dials UP for
    /// more pre-strain or DOWN to experiment.
    #[must_use]
    pub fn default_for_scan() -> Self {
        Self {
            inset_m: CAVITY_DEFAULT_INSET_M,
            visible: true,
        }
    }

    /// Slider range for the cavity inset, in meters. Returns
    /// `(0, CAVITY_INSET_SLIDER_MAX_M)` = 0 mm to 8 mm.
    ///
    /// **Why 8 mm**: an 8 mm interference on a typical body-part
    /// scan diameter (~70 mm) is ~11 % engineered compression —
    /// still compression-fit territory, well within Dragon Skin
    /// 20A's 580 % elongation budget. Going deeper than ~8 mm
    /// pushes the design out of the validated workshop envelope.
    ///
    /// The upper bound itself lives in [`CAVITY_INSET_SLIDER_MAX_M`]
    /// so consumer binaries (cf-device-design + cf-sim-research)
    /// format their millimeter readouts from a single source.
    ///
    /// For the FEM convergence-envelope research history behind the
    /// 8 mm value (Phase H4-2-C asymmetric one-sided bound,
    /// 2026-05-19), see the cf-sim-research-side render_cavity_section
    /// label and `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5.4.
    #[must_use]
    pub fn inset_slider_range_m() -> (f64, f64) {
        (0.0, CAVITY_INSET_SLIDER_MAX_M)
    }
}

/// Upper bound on [`CavityState::inset_slider_range_m`], in meters
/// (8 mm). Single source of truth for the cavity slider's cap —
/// consumer binaries (cf-device-design + cf-sim-research) format
/// their millimeter readouts from this const, and each carries a
/// `cavity_inset_slider_range_zero_to_eight_mm` sentinel test
/// asserting against it. See [`CavityState::inset_slider_range_m`]
/// for the workshop-envelope rationale + a pointer to the
/// sim-research history.
pub const CAVITY_INSET_SLIDER_MAX_M: f64 = 0.008;

/// Default cavity inset (meters). 3 mm = the minimum-acceptable
/// starting point for a buildable silicone device:
/// - Above Ecoflex 00-30's ~2 mm castability threshold (thinner
///   walls tear under stretch).
/// - Yields ~10 % radial pre-strain on a typical 25–35 mm-diameter
///   scan cross-section — meaningful snug fit, well within
///   elongation-at-break (~900 %).
///
/// The user dials UP for more pre-strain or DOWN to experiment
/// (down to 0 = cavity-coincident-with-scan, useful as a debug
/// reference but not a buildable design).
pub const CAVITY_DEFAULT_INSET_M: f64 = 0.003;

/// Silicone-material catalog the Layers panel offers. Each entry is
/// `(anchor_key, display_label, density_kg_m3)`.
///
/// The `anchor_key` strings mirror cf-cast's `cure::lookup` keys; the
/// densities mirror `sim-soft`'s `silicone_table.rs` anchor values
/// (Ecoflex 00-10 = 1040, the rest of the Ecoflex line + Dragon Skin
/// 10A/15 = 1070, Dragon Skin 20A/30A = 1080 kg/m³). Both are
/// mirrored by-name, not by-import: this crate carries the catalog
/// standalone (no cf-cast / sim-soft dep) so it stays composable.
/// cf-cast-cli re-validates the chosen anchor + resolves the runtime
/// density at `.design.toml` ingest.
pub const LAYER_MATERIALS: &[(&str, &str, f64)] = &[
    ("ECOFLEX_00_10", "Ecoflex 00-10 (super-soft)", 1040.0),
    ("ECOFLEX_00_20", "Ecoflex 00-20 (soft)", 1070.0),
    ("ECOFLEX_00_30", "Ecoflex 00-30 (medium-soft)", 1070.0),
    ("ECOFLEX_00_50", "Ecoflex 00-50 (firm-soft)", 1070.0),
    ("DRAGON_SKIN_10A", "Dragon Skin 10A (soft)", 1070.0),
    ("DRAGON_SKIN_15", "Dragon Skin 15 (medium)", 1070.0),
    ("DRAGON_SKIN_20A", "Dragon Skin 20A (firm)", 1080.0),
    ("DRAGON_SKIN_30A", "Dragon Skin 30A (firmest)", 1080.0),
];

/// Bulk density (kg/m³) for a layer's `material_anchor_key`, looked
/// up in [`LAYER_MATERIALS`]. Falls back to 1070 kg/m³ (the Ecoflex/
/// Dragon-Skin line median) for an unrecognized key — defensive only;
/// every key the Layers panel can set comes from the catalog.
#[must_use]
pub fn material_density(anchor_key: &str) -> f64 {
    LAYER_MATERIALS
        .iter()
        .find(|(key, _, _)| *key == anchor_key)
        .map_or(1070.0, |(_, _, density)| *density)
}

/// Maximum number of concentric silicone layers in the device wall.
/// 6 is a generous workshop cap; real designs typically use 1–3.
/// Setting a finite cap keeps panel scroll predictable + the
/// per-frame draw cost bounded.
pub const LAYER_COUNT_MAX: usize = 6;

/// Palette tinting the per-layer outer-surface mesh entities (and the
/// per-layer rows in the read-only sim-research panel). Repeats if
/// the layer count exceeds the palette length. Lifted to
/// `cf-device-types` per `docs/archive/SIM_DECOUPLE_PHASE_3_RECON.md` §2.5.a
/// so cf-device-design (3D shells + Sim panel layer rows) and
/// cf-sim-research (panel-row swatches) tint the same layer-index
/// with the same color.
pub const LAYER_SURFACE_PALETTE: &[(f32, f32, f32)] = &[
    (0.95, 0.80, 0.35), // amber
    (0.45, 0.70, 0.95), // sky blue
    (0.75, 0.55, 0.95), // lavender
    (0.55, 0.95, 0.65), // mint
    (0.95, 0.45, 0.70), // pink
];

/// One concentric silicone layer in the device wall. Layers are
/// ordered innermost-first: `layers[0]`'s inner surface = the cavity
/// surface, `layers[i]`'s outer surface = `layers[i+1]`'s inner
/// surface, `layers[N-1]`'s outer surface = the device's outer skin.
///
/// Every layer carries its own user-dialed `thickness_m` (slice 5
/// polish 3 pivot — there is no derived "last layer"). The device
/// wall total is the sum of layer thicknesses; the outer skin sits
/// at `sum(thickness) - cavity.inset_m` radially from the scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LayerSpec {
    /// Radial thickness (meters), user-dialed via a panel slider.
    /// For layer 0 this is the distance from the cavity surface to
    /// layer 0's outer surface; for layer `i > 0` it's the Δ from
    /// the prior layer's outer surface.
    pub thickness_m: f64,
    /// Smooth-On product anchor key (from [`LAYER_MATERIALS`]).
    /// `'static` lifetime because the entries are compile-time
    /// constants. This is the layer's *base* silicone — Slacker (if
    /// any) is dialed separately via `slacker_fraction`.
    pub material_anchor_key: &'static str,
    /// Slacker mass as a fraction of the base silicone's combined
    /// Part A + Part B mass — `0.0` is the base with no Slacker. The
    /// Layers panel only offers the discrete fractions on the base
    /// material's Smooth-On TB curve ([`crate::slacker::support`]); a
    /// base-material change that orphans the stored value snaps it
    /// back to `0.0` via [`crate::resolve_slacker_fraction`].
    pub slacker_fraction: f64,
    /// Whether this layer's outer-surface mesh is drawn this frame.
    /// Per-layer visibility (slice 5 polish 5) — each layer toggles
    /// independently so the user can isolate any single layer for
    /// inspection.
    pub visible: bool,
}

impl LayerSpec {
    /// Slider range for per-layer thickness (m). 1 mm minimum matches
    /// FDM-printable mold-wall minimum; 20 mm upper bound is generous
    /// for any single layer.
    #[must_use]
    pub fn thickness_slider_range_m() -> (f64, f64) {
        (0.001, 0.020)
    }
}

/// Bevy resource carrying the ordered layer stack. Default state is a
/// single Ecoflex 00-30 layer — the simplest buildable configuration.
/// The user adds layers + dials each thickness via the Layers panel;
/// the device wall total is the sum of layer thicknesses.
///
/// Invariant: `1 <= layers.len() <= LAYER_COUNT_MAX`. The panel's
/// add/remove controls preserve this (the "Remove layer" button is
/// hidden when only one layer remains; "+ Add layer" is hidden at
/// the cap).
#[derive(Resource, Debug, Clone, PartialEq)]
pub struct LayersState {
    /// Innermost-first ordered layer stack. Maintained between
    /// `1` and [`LAYER_COUNT_MAX`] entries by the panel's add/remove
    /// controls.
    pub layers: Vec<LayerSpec>,
}

impl LayersState {
    /// Default state: single Ecoflex 00-30 layer. The user adds more
    /// layers via the panel as needed.
    #[must_use]
    pub fn default_for_scan() -> Self {
        Self {
            layers: vec![LayerSpec {
                thickness_m: 0.005,
                material_anchor_key: "ECOFLEX_00_30",
                slacker_fraction: 0.0,
                visible: true,
            }],
        }
    }
}
