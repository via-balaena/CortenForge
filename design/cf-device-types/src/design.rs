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

    /// Slider range for the cavity inset, in meters. **Raised to
    /// 8 mm** as H4-2-C probe scaffolding (F3 recon B candidate
    /// H4-2-C — asymmetric one-sided bound, see
    /// `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5.4). H4-2-C
    /// cargo-test sweep showed cavity 5 mm + 8 mm reach partial
    /// seating where the H4-2-A 0.20 floor + the original 0.30 floor
    /// both panicked; user-driven visual gate at cavity 3 + 5 mm
    /// cleared 16/16 (verified 2026-05-19 LATE-NIGHT) matching the
    /// pre-H4 baseline. This cap raise unlocks 6 + 7 + 8 mm visual
    /// gates to confirm whether the GUI also reaches 16/16 there
    /// (cargo test hit 0/16 / 0/16 / 8/16 respectively due to the
    /// `cargo test` ↔ GUI divergence puzzle at bookmark §5.6).
    ///
    /// **5 mm pre-H4-2-C cap** set at F3 recon B C′.a ε-bisection
    /// ship 2026-05-18 LATE-EVENING (per
    /// `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md` §3 C′.a)
    /// and held through the E.b.4 case-E falsification 2026-05-19
    /// (per `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10).
    /// 5 mm was the highest cavity the C′.a-pinned ε = 0.075 mm
    /// converged 16/16 at WITHOUT H4 bounds plumbing; cavity >
    /// 5 mm previously fake-converged step 1 into an invalid Yeoh
    /// state and panicked at step 2 (or, post-commit `2739717e`
    /// end-of-solve check, panics honestly at step 1).
    ///
    /// **What H4-2-C enables**: the calibrated `0.8 · λ_break`
    /// tensile cap stays in force, but the family-uniform compressive
    /// floor (previously 0.30, then H4-2-A 0.20) is dropped at
    /// `MaterialField::sample_yeoh` time. Newton can now iterate
    /// through deep-compression equilibria (cavity 5 mm visual gate
    /// reached λ_min = 0.37 on one step without panicking). `det F >
    /// 0` inversion is the remaining compressive safety net.
    ///
    /// Cavity = 3 mm baseline preserved at the C′.a ε (verified
    /// user-driven visual gate 2026-05-18 LATE-EVENING, 16/16 + ZERO
    /// LM rescues — orthogonal to gated-A's class-1 rescue). Cavity
    /// = 5 mm also user-verified 16/16 with H4-2-C 2026-05-19
    /// LATE-NIGHT.
    ///
    /// **Physical strain context** at 8 mm: 8 mm interference on a
    /// typical body-part scan diameter (~70 mm) is ~11 % engineered
    /// compression — still compression-fit territory, well within
    /// DRAGON_SKIN_20A's 580 % elongation budget.
    ///
    /// **MAINTENANCE NOTE**: if you change the cap value here,
    /// mirror the change to **(1)** the egui label in
    /// cf-device-design's `render_cavity_section` (`(capped at 8 mm
    /// — ...)` string), AND **(2)** the sentinel test
    /// `cavity_inset_slider_range_zero_to_eight_mm` which pins the
    /// bound + carries the per-cap rationale. All three surfaces
    /// must agree on cap value AND on the binding-constraint
    /// attribution (solver envelope vs material validity vs
    /// scaffolding).
    #[must_use]
    pub fn inset_slider_range_m() -> (f64, f64) {
        (0.0, 0.008)
    }
}

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
