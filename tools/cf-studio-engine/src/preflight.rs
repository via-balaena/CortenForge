//! "Will this cavity inset actually cast?", asked at step 3 instead of half an
//! hour into the export.
//!
//! ⚠ Writes NOTHING, and that is the design. Asking this through
//! [`crate::generate_molds_for_design`] would materialize `<stem>.design.toml`
//! beside the cleaned scan — and with the invented layer stack below, that
//! overwrites the operator's real design. `~/scans` is flat, so it sits right
//! next to the STL.
use std::path::{Path, PathBuf};

use cf_studio_core::RidgeOptions;
use cortenforge::cf_cap_planes::parse_cap_planes;
use cortenforge::cf_cast::plug_fit_verdict;
use cortenforge::cf_cast_cli::{
    CastConfig, DerivedSpec, LayerConfig, derive_spec_and_ribbon, load_scan_sdf,
    parse_centerline_from_prep_toml,
};

use crate::error::{EngineError, Result};
use crate::mold::canal_config_from_ridges;

/// What the pre-flight found at one cavity inset.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlugFit {
    /// Layer 0's plug casts as one piece — its floor lock fused to the body.
    Casts,
    /// It does not, carrying the cast's own words for why.
    WillNotCast {
        /// The cast's message, which already names the operator's levers.
        reason: String,
    },
}

/// The layer stack the probe invents, because step 3 runs before step 4 picks
/// the real one.
///
/// ⚠ The thickness is not inert. It feeds `sdf_bounds_padding_m`, the SDF box
/// is the scan AABB expanded by that, and the mesher anchors its lattice at
/// `bounds.min` stepping exactly one cell — so a different stack TRANSLATES the
/// grid the plug is sampled on.
///
/// ⚠⚠ Which makes the cell size an A/B's confound, not a detail.
/// `~/scans/base_mold`'s real stack sits 25 mm of padding from this probe, and
/// at the 0.5 mm print cell that is EXACTLY 50 cells — the two lattices
/// coincide, and comparing them agrees for reasons that have nothing to do with
/// the verdict transferring. This comment used to cite that run.
///
/// The claim as it now stands, measured 2026-09-08 at 1.5 mm, where the same
/// two stacks sit ⅓ of a cell apart and the meshes genuinely differ (face
/// counts throughout, and the plug breaks into a different NUMBER of pieces at
/// 7, 8 and 9 mm): **the verdict agrees at all 31 insets step 3's
/// integer-millimetre field can produce**, threshold 6→7 mm on both, at 2.5×
/// less time. Gated on the synthetic cone by
/// `the_invented_layer_stack_does_not_change_the_verdict`.
///
/// ⚠⚠ Must stay above `[cast].wall_thickness_m`. The canal gates its suction
/// bulge against both the cup wall and `layers.first()`, and the second would
/// be asked of THIS invented layer. See `the_probe_layer_outweighs_the_cup_wall`.
pub const PROBE_LAYER_THICKNESS_M: f64 = 0.006;
/// Cure anchor for the probe layer. Any catalog material works — the layer
/// exists to give the derivation a stack, and nothing here pours.
const PROBE_LAYER_MATERIAL: &str = "ECOFLEX_00_30";

/// Ask whether layer 0's plug casts as one piece at `cavity_inset_m`.
///
/// `mesh_cell_size_m` is not a speed knob — **pass the size you intend to cast
/// at**. Detachment turns on sub-cell grid alignment, so verdicts do not
/// transfer between cell sizes: measured 2026-09-08 on `~/scans/base_mold`,
/// 2.0 and 3.0 mm cells cast a 6.3 mm inset that both shipped sizes refuse.
///
/// ⚠ Enabled `ridges` override it anyway. The canal path pins layer 0's plug
/// at its own cell size (0.5 mm) whatever is passed, and composes displacement
/// onto the plug — which changes the answer AND the cost. Measured at 5 mm of
/// inset: smooth casts in 6.9 s, ridged refuses in 234 s, and the cast agrees.
/// Budget ~4 minutes with ridges on, not seconds.
///
/// # Errors
/// [`EngineError::ScanLoad`], [`EngineError::PrepInvalid`],
/// [`EngineError::NoCenterline`] or [`EngineError::MoldGen`] if the pre-flight
/// could not be *run*. A cast that runs and declines is
/// [`PlugFit::WillNotCast`], not an error — otherwise a missing prep file
/// reaches the operator as "your inset is too large".
pub fn plug_fit_preflight(
    cleaned_stl: &Path,
    prep_toml: &Path,
    cavity_inset_m: f64,
    ridges: &RidgeOptions,
    mesh_cell_size_m: f64,
) -> Result<PlugFit> {
    // ⚠ `for_design` is used for its CastDefaults — planar seam, split normal,
    // apex-axial pour gate, plug pins on — because those are what the wizard's
    // own cast uses, and they decide where the floor lock is anchored. A
    // hand-built config with different defaults would produce a different
    // ribbon and therefore answer a different question. The design PATH it
    // takes is never read: `design` is cleared on the next line in favour of
    // inline layers, so nothing has to exist on disk.
    let mut config = CastConfig::for_design(
        cleaned_stl.to_path_buf(),
        prep_toml.to_path_buf(),
        PathBuf::new(),
        mesh_cell_size_m,
        canal_config_from_ridges(ridges),
    );
    config.design = None;
    config.layers = vec![LayerConfig {
        thickness_m: PROBE_LAYER_THICKNESS_M,
        material: PROBE_LAYER_MATERIAL.to_string(),
        density_kg_m3: None,
        display_name: None,
        slacker_fraction: None,
    }];

    // The same two gates the run path applies before deriving (non-empty
    // layers; the apex pour bore and the cavity-floor slab both needing the
    // flat seam). They hold by construction for the config built above, and
    // are called anyway so a future change to `for_design`'s defaults surfaces
    // here as a clear error instead of somewhere inside the derivation.
    config
        .validate_layer_source()
        .and_then(|()| config.validate_after_layer_source())
        .map_err(|e| EngineError::MoldGen(format!("{e:#}")))?;

    let prep_text = std::fs::read_to_string(prep_toml).map_err(|e| EngineError::PrepInvalid {
        path: prep_toml.display().to_string(),
        reason: e.to_string(),
    })?;
    let centerline =
        parse_centerline_from_prep_toml(&prep_text).map_err(|e| EngineError::PrepInvalid {
            path: prep_toml.display().to_string(),
            reason: format!("{e:#}"),
        })?;
    if centerline.len() < 2 {
        return Err(EngineError::NoCenterline {
            path: prep_toml.display().to_string(),
        });
    }
    let cap_planes = parse_cap_planes(&prep_text).map_err(|e| EngineError::PrepInvalid {
        path: prep_toml.display().to_string(),
        reason: format!("{e:#}"),
    })?;

    // Asked of the config rather than recomputed: the run path needs the same
    // number, and an under-padded flood fill does not fail loudly — it answers
    // questions about a domain it does not cover.
    let loaded = load_scan_sdf(
        cleaned_stl,
        config.sdf_bounds_padding_m(),
        config.cast.mesh_cell_size_m,
    )
    .map_err(|e| EngineError::ScanLoad {
        path: cleaned_stl.display().to_string(),
        reason: format!("{e:#}"),
    })?;

    let DerivedSpec { spec, ribbon } = derive_spec_and_ribbon(
        &config,
        &loaded.sdf,
        loaded.aabb,
        &centerline,
        cavity_inset_m,
        &cap_planes,
    )
    .map_err(|e| EngineError::MoldGen(format!("{e:#}")))?;

    Ok(match plug_fit_verdict(&spec, &ribbon) {
        Ok(()) => PlugFit::Casts,
        Err(e) => PlugFit::WillNotCast {
            reason: e.to_string(),
        },
    })
}

#[cfg(test)]
mod tests {
    use cortenforge::cf_cast_cli::CastDefaults;

    use super::*;

    /// ★ The probe layer must outweigh the mold cup wall.
    ///
    /// This is a relationship between two constants that know nothing about
    /// each other — one here, one a cast default — and it cannot be reached
    /// behaviourally: while it HOLDS, the cup-wall gate trips first for every
    /// suction bulge, so the invented-layer gate is unreachable and no input
    /// can tell the two apart. That is exactly why it is asserted directly.
    /// Lower [`PROBE_LAYER_THICKNESS_M`] under the cup wall and a window opens
    /// in which the pre-flight fails on its own invented layer while the cast
    /// succeeds — reported to the operator as a suction bulb blowing out a
    /// shell they never configured.
    #[test]
    fn the_probe_layer_outweighs_the_cup_wall() {
        let cup_wall_m = CastDefaults::default().wall_thickness_m;
        assert!(
            cup_wall_m > 0.0,
            "a zero cup wall would make this pass for the wrong reason"
        );
        assert!(
            PROBE_LAYER_THICKNESS_M > cup_wall_m,
            "the probe layer ({PROBE_LAYER_THICKNESS_M} m) must exceed the cup \
             wall ({cup_wall_m} m), or the canal's suction-bulge gate can fail \
             the pre-flight on a layer the operator never chose"
        );
    }
}
