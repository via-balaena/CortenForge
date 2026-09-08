//! "Will this cavity inset actually cast?" — asked while the operator is still
//! holding the slider, instead of half an hour into the export.
//!
//! ★ The answer comes from the cast itself. [`plug_fit_verdict`] is the compose
//! half of the very function the export runs, so this is not a prediction *of*
//! the cast — it is the cast's own verdict, reached without writing anything.
//! There is no second implementation to drift.
//!
//! ⚠ It writes NOTHING. That is load-bearing, not tidiness: the obvious way to
//! ask this question is to run [`crate::generate_molds_for_design`] somewhere
//! harmless, and that function materializes `<stem>.design.toml` beside the
//! cleaned scan. A pre-flight carrying an invented layer stack would therefore
//! overwrite the operator's real design with a one-layer draft — on a flat scan
//! folder like `~/scans`, their actual saved work. Going through the derivation
//! directly removes the hazard rather than managing it.

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

/// The layer stack the probe invents.
///
/// ★ Step 3 sets the inset; the layer stack is not chosen until step 4, so a
/// pre-flight at step 3 has to supply one. Measured 2026-09-08 on
/// `~/scans/base_mold` at 0.5 mm cells: a one-layer stack and the real
/// three-layer stack give the SAME verdict on both sides of the threshold
/// (6.1 and 6.2 mm cast, 6.5 mm refuses), for 4.9× less time — the thin stack
/// shrinks the mold box, so there is far less to mesh.
///
/// ⚠ The verdict transfers; the MESH does not. The box grows with total stack
/// thickness, which moves the marching-cubes grid and shifts the plug ~0.6 mm
/// laterally. Never compare these bytes against a real cast's.
const PROBE_LAYER_THICKNESS_M: f64 = 0.006;
/// Cure anchor for the probe layer. Any catalog material works — the layer
/// exists to give the derivation a stack, and nothing here pours.
const PROBE_LAYER_MATERIAL: &str = "ECOFLEX_00_30";

/// Ask whether layer 0's plug casts as one piece at `cavity_inset_m`.
///
/// `mesh_cell_size_m` is not a speed knob — **pass the size you intend to cast
/// at**. Detachment turns on sub-cell grid alignment, so the verdict does not
/// transfer between cell sizes: measured 2026-09-08 on `~/scans/base_mold`,
/// 2.0 mm and 3.0 mm cells cast a 6.3 mm inset that BOTH shipped sizes (0.5 mm
/// Fine and 1.5 mm Fast) refuse. Asking at a convenient-but-unshipped size
/// buys a fast answer to a question nobody asked.
///
/// # Errors
/// [`EngineError::ScanLoad`], [`EngineError::PrepInvalid`],
/// [`EngineError::NoCenterline`] or [`EngineError::MoldGen`] if the pre-flight
/// could not be *run*.
///
/// ⚠ A cast that runs and declines is [`PlugFit::WillNotCast`], NOT an error.
/// Keeping the two apart is what stops a missing prep file from being reported
/// to the operator as "your inset is too large".
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

    // ⚠ Asked of the config rather than recomputed here. The run path needs
    // the same number, and a pre-flight that padded its flood fill differently
    // would not fail loudly — it would answer questions about a domain it does
    // not cover.
    let bounds_padding_m = config.sdf_bounds_padding_m();
    let loaded = load_scan_sdf(cleaned_stl, bounds_padding_m, mesh_cell_size_m).map_err(|e| {
        EngineError::ScanLoad {
            path: cleaned_stl.display().to_string(),
            reason: format!("{e:#}"),
        }
    })?;

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

    let DerivedSpec { spec, ribbon } = derive_spec_and_ribbon(
        &config,
        &loaded.sdf,
        loaded.aabb,
        &centerline,
        cavity_inset_m,
        &cap_planes,
    )
    .map_err(|e| EngineError::MoldGen(format!("{e:#}")))?;

    Ok(match plug_fit_verdict(&spec, &ribbon, 0) {
        Ok(()) => PlugFit::Casts,
        Err(e) => PlugFit::WillNotCast {
            reason: e.to_string(),
        },
    })
}
