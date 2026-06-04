//! The design-build boundary: turn the spine's owned [`DesignDraft`]
//! into a validated `.design.toml`, reusing the **headless**
//! `cf-device-types` types + writer (no Bevy — the payoff of making
//! that crate Bevy-optional).

use std::path::Path;

use cf_device_types::design_toml::{
    DesignToml, build_design_toml, save_design_toml, validate_design_toml,
};
use cf_device_types::{CavityState, LAYER_MATERIALS, LayerSpec, LayersState};
use cf_studio_core::DesignDraft;

use crate::error::{EngineError, Result};

/// Recover the catalog's `&'static str` key matching an owned key, or
/// `None` if the silicone isn't in the catalog.
fn resolve_anchor(key: &str) -> Option<&'static str> {
    LAYER_MATERIALS
        .iter()
        .find(|(catalog_key, _, _)| *catalog_key == key)
        .map(|(catalog_key, _, _)| *catalog_key)
}

/// Convert the spine's [`DesignDraft`] (owned design intent) into a
/// validated [`DesignToml`] anchored to `cleaned_stl`.
///
/// Each layer's owned `material_key` is resolved against the silicone
/// catalog to recover the `&'static str` the SDK types require; an
/// unknown key is rejected up front with its layer index.
///
/// # Errors
/// - [`EngineError::UnknownMaterial`] if a layer's silicone isn't in the catalog.
/// - [`EngineError::InvalidDesign`] if the assembled design fails
///   `cf-device-types` validation (no layers, non-finite thickness, negative inset).
pub fn design_toml_from_draft(cleaned_stl: &Path, draft: &DesignDraft) -> Result<DesignToml> {
    let cavity = CavityState {
        inset_m: draft.cavity_inset_m,
        visible: true,
    };
    let mut layers = Vec::with_capacity(draft.layers.len());
    for (index, layer) in draft.layers.iter().enumerate() {
        let anchor =
            resolve_anchor(&layer.material_key).ok_or_else(|| EngineError::UnknownMaterial {
                index,
                key: layer.material_key.clone(),
            })?;
        layers.push(LayerSpec {
            thickness_m: layer.thickness_m,
            material_anchor_key: anchor,
            slacker_fraction: layer.slacker_fraction,
            visible: true,
        });
    }
    let design = build_design_toml(cleaned_stl, &cavity, &LayersState { layers });
    // `{:#}` captures anyhow's full context chain (plain `to_string()`
    // would show only the outermost message and drop the cause).
    validate_design_toml(&design).map_err(|e| EngineError::InvalidDesign(format!("{e:#}")))?;
    Ok(design)
}

/// Build the design from `draft` and write it to `path` (atomic write
/// via `cf-device-types`). Returns the design that was written.
///
/// # Errors
/// As [`design_toml_from_draft`], plus [`EngineError::WriteDesign`] on I/O failure.
pub fn save_design_from_draft(
    cleaned_stl: &Path,
    draft: &DesignDraft,
    path: &Path,
) -> Result<DesignToml> {
    let design = design_toml_from_draft(cleaned_stl, draft)?;
    // `{:#}` keeps the full anyhow chain (e.g. the underlying io error
    // beneath save_design_toml's "rename …" context).
    save_design_toml(&design, path).map_err(|e| EngineError::WriteDesign(format!("{e:#}")))?;
    Ok(design)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

    use cf_device_types::design_toml::load_design_toml;
    use cf_studio_core::{DesignDraft, LayerDraft};

    use super::*;

    /// The base_mold 17.5 / 7.5 / 5 mm stack, soft→firm.
    fn draft() -> DesignDraft {
        DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![
                LayerDraft {
                    thickness_m: 0.0175,
                    material_key: "ECOFLEX_00_30".to_string(),
                    slacker_fraction: 0.25,
                },
                LayerDraft {
                    thickness_m: 0.0075,
                    material_key: "DRAGON_SKIN_10A".to_string(),
                    slacker_fraction: 0.0,
                },
                LayerDraft {
                    thickness_m: 0.005,
                    material_key: "DRAGON_SKIN_20A".to_string(),
                    slacker_fraction: 0.0,
                },
            ],
        }
    }

    #[test]
    fn draft_converts_to_matching_design_toml() {
        let design = design_toml_from_draft(Path::new("base_mold.cleaned.stl"), &draft()).unwrap();
        assert_eq!(design.cavity.inset_m, 0.005);
        assert_eq!(design.layers.len(), 3);
        assert_eq!(design.layers[0].material_anchor_key, "ECOFLEX_00_30");
        assert_eq!(design.layers[0].thickness_m, 0.0175);
        assert_eq!(design.layers[0].slacker_fraction, 0.25);
        assert_eq!(design.layers[1].material_anchor_key, "DRAGON_SKIN_10A");
        assert_eq!(design.layers[2].material_anchor_key, "DRAGON_SKIN_20A");
        assert_eq!(design.scan_ref.cleaned_stl, "base_mold.cleaned.stl");
    }

    #[test]
    fn unknown_material_is_rejected_with_its_layer_index() {
        let mut d = draft();
        d.layers[1].material_key = "NOT_A_SILICONE".to_string();
        let err = design_toml_from_draft(Path::new("s.stl"), &d).unwrap_err();
        assert!(
            matches!(&err, EngineError::UnknownMaterial { index: 1, key } if key == "NOT_A_SILICONE"),
            "got: {err:?}"
        );
    }

    #[test]
    fn empty_draft_is_rejected_as_invalid_design() {
        let d = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![],
        };
        let err = design_toml_from_draft(Path::new("s.stl"), &d).unwrap_err();
        assert!(matches!(err, EngineError::InvalidDesign(_)), "got: {err:?}");
    }

    #[test]
    fn negative_thickness_is_rejected_as_invalid_design() {
        let mut d = draft();
        d.layers[0].thickness_m = -0.001;
        let err = design_toml_from_draft(Path::new("s.stl"), &d).unwrap_err();
        assert!(matches!(err, EngineError::InvalidDesign(_)), "got: {err:?}");
    }

    #[test]
    fn negative_cavity_inset_is_rejected_as_invalid_design() {
        let mut d = draft();
        d.cavity_inset_m = -0.001;
        let err = design_toml_from_draft(Path::new("s.stl"), &d).unwrap_err();
        assert!(matches!(err, EngineError::InvalidDesign(_)), "got: {err:?}");
    }

    #[test]
    fn save_then_load_round_trips_through_disk() {
        let dir =
            std::env::temp_dir().join(format!("cf-studio-engine-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("base_mold.design.toml");

        save_design_from_draft(Path::new("base_mold.cleaned.stl"), &draft(), &path).unwrap();
        let loaded = load_design_toml(&path).unwrap();
        assert_eq!(loaded.layers.len(), 3);
        assert_eq!(loaded.cavity.inset_m, 0.005);
        assert_eq!(loaded.layers[0].material_anchor_key, "ECOFLEX_00_30");
        assert_eq!(loaded.layers[1].thickness_m, 0.0075);

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir(&dir);
    }
}
