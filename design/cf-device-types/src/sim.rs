//! Sim-side projection of the design state, plus the per-run UI
//! enums.
//!
//! `(CavityState, LayersState)` carries the user-dialed design; the
//! insertion-sim consumes its decimated projection
//! ([`SimDesign`] / [`SimLayer`]) — radial thickness + anchor key +
//! Slacker fraction, with the rendering knobs (`visible`) stripped.
//! [`SlackerResolution`] records how a layer's effective material
//! was resolved (base / interpolated / floored). [`ScalarMode`] +
//! [`SimMode`] are the per-run UI enums consumed by the sim panel.

/// One concentric layer of the device wall, projected for the
/// insertion sim: a radial thickness + a base-silicone anchor key +
/// the per-layer Slacker mass fraction.
///
/// The decimated form of [`crate::LayerSpec`] (drops `visible` — a
/// viewport concern). `slacker_fraction` is plumbed straight through:
/// the insertion-sim's `effective_silicone_for_layer` shifts the
/// effective Shore + Yeoh params (see [`SlackerResolution`] for the
/// resolution table + the Shore-000 floor).
#[derive(Debug, Clone)]
pub struct SimLayer {
    /// Radial thickness (meters). Innermost-first ordering, same as
    /// `LayerSpec`.
    pub thickness_m: f64,
    /// Smooth-On base-silicone anchor key — one of the eight in
    /// [`crate::LAYER_MATERIALS`].
    pub anchor_key: String,
    /// Slacker mass as a fraction of the base silicone's Part A+B
    /// mass — `0.0` for the base alone, matches the
    /// [`crate::LayerSpec`]`.slacker_fraction` semantics. The
    /// TB-tabulated fractions are `{0.0, 0.25, 0.50, 0.75, 1.00}` per
    /// the Smooth-On Slacker Technical Bulletin (rev 011524DH);
    /// [`crate::slacker::support`] returns the curve. Off-curve
    /// values are snapped by [`crate::resolve_slacker_fraction`]
    /// before reaching the sim.
    pub slacker_fraction: f64,
}

/// Device-design parameters the insertion sim consumes: the cavity
/// inset + the ordered (innermost-first) layer stack. The decimated
/// projection of [`crate::CavityState`] + [`crate::LayersState`].
#[derive(Debug, Clone)]
pub struct SimDesign {
    /// Distance (meters) the cavity surface sits *inside* the scan
    /// surface — `CavityState::inset_m`. The cavity surface is at
    /// scan-SDF offset `-cavity_inset_m`.
    pub cavity_inset_m: f64,
    /// Innermost-first layer stack. `layers[0]`'s inner surface is
    /// the cavity surface; `layers[i]`'s outer surface is
    /// `layers[i + 1]`'s inner surface.
    pub layers: Vec<SimLayer>,
}

/// How `effective_silicone_for_layer` (in the insertion-sim) resolved
/// a layer — surfaces alongside the returned `SiliconeMaterial` so
/// the panel can flag fallbacks to the user.
///
/// Slacker pushes silicones across Shore scales (A → 00 → 000).
/// sim-soft's anchor table covers Shore 00 + Shore A; Shore 000 (the
/// gel scale, where most Slacker outcomes land) has no published
/// Yeoh anchors. The variants below capture every path the resolver
/// can take.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlackerResolution {
    /// `slacker_fraction == 0.0` (or `Support::NotRecommended` /
    /// `Support::NoData`) — material is the base anchor unchanged.
    Base,
    /// Slacker fraction snapped to a TB curve point whose effective
    /// hardness lands in Shore A or Shore 00, both of which sim-soft
    /// anchors. The material was built via
    /// `SiliconeMaterial::from_effective_shore`.
    Interpolated,
    /// Slacker fraction snapped to a TB point whose effective
    /// hardness is Shore 000 (OOO) — softer than sim-soft's softest
    /// published anchor (`ECOFLEX_00_10`). The resolver returns the
    /// `ECOFLEX_00_10` material as a conservative floor (slight
    /// over-stiffness vs ground truth). A proper Shore-000
    /// calibration is a future slice.
    FlooredAtSoftestAnchor,
}

/// Which scalar field drives the heat-map gradient — `Ψ` (J/m³) or
/// `‖P‖_F` (Pa). The Insertion Sim pipeline pre-computes vertex
/// colors for BOTH modes, so the user can toggle between them
/// without re-running the ramp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ScalarMode {
    /// Strain-energy density `Ψ`. Default — what "how strained is
    /// this region" intuitively reads as.
    #[default]
    EnergyDensity,
    /// First-Piola stress Frobenius norm `‖P‖_F`. The peak-hotspot
    /// scalar; cleaner for finding stress concentrations.
    StressFrobenius,
}

impl ScalarMode {
    /// Index into each step's `[Vec<f64>; 2]` in
    /// `InsertionSimOutputs::per_step_scalar_fields` +
    /// `InsertionSimOutputs::scalar_min_max`.
    #[must_use]
    pub fn buffer_index(self) -> usize {
        match self {
            Self::EnergyDensity => 0,
            Self::StressFrobenius => 1,
        }
    }

    /// Short user-facing label.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::EnergyDensity => "Ψ (J/m³)",
            Self::StressFrobenius => "‖P‖ (Pa)",
        }
    }
}

/// Which insertion-sim model the next Simulate click runs.
///
/// Per `docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` §2 Q2 + §4 SL.3 row.
/// Defaults to [`Sliding`](Self::Sliding) — the new sliding-intruder
/// FEM is the workshop-product model. [`GrowingIntruder`](Self::GrowingIntruder)
/// is the legacy uniform-offset ramp, preserved for "prototype
/// variations of the scanned size going through the cavity" per the
/// user's banked note (bookmark §2). SL.7 (pass B) reawakens the
/// growing-mode UI + render handling; SL.3 ships the dispatch with
/// both branches functionally wired but the per-mode UI affordances
/// (mode selector, intruder render rename) at SL.4 + SL.5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SimMode {
    /// New sliding-intruder model (default): constant-geometry rigid
    /// intruder translated along the centerline arc, per-position
    /// FEM-solved local cavity deformation. Requires a non-empty
    /// `Centerline` resource (`.prep.toml [centerline]`).
    #[default]
    Sliding,
    /// Legacy growing-intruder model: uniform SDF offset ramped from
    /// `cavity_inset_m / n_steps` to `cavity_inset_m` over the ramp,
    /// causing the whole cavity wall to deform uniformly at each
    /// step. Useful for "what if the scanned shape was bigger by 1%"
    /// prototyping (banked use case per bookmark §2 + §6).
    GrowingIntruder,
}
