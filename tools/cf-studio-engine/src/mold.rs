//! The mold-generation boundary (workflow step "Make molds"): drive the
//! cast pipeline from a **typed** [`CastConfig`] (no `cast.toml` to author
//! or re-parse) and collect the printable STLs + procedure + structured
//! pour plan into a [`MoldOutputs`].

use std::path::{Path, PathBuf};

use cf_cast_cli::{CastConfig, run_with_config};
use cf_studio_core::{DesignDraft, MoldOutputs};

use crate::error::{EngineError, Result};
use crate::pour::{LayerPour, build_pour_plan};

/// Generate the molds for a project: run the cast pipeline typed, then
/// gather the outputs.
///
/// `config`'s `[design]` source should point at the `design.toml` saved
/// in step 3, so the cavity inset + layer stack are lifted from it.
/// `draft` supplies the layer anchors + Slacker for the pour plan (it
/// matches that `design.toml` by construction, written from the same
/// draft). `base_dir` is what the config's relative paths resolve against
/// (the project's scan directory).
///
/// # Errors
/// - [`EngineError::MoldGen`] if the cast run fails or its outputs can't
///   be read back.
/// - [`EngineError::PourDataUnavailable`] if a layer has no cure data.
pub fn generate_molds(
    config: CastConfig,
    draft: &DesignDraft,
    base_dir: &Path,
    output_dir_override: Option<&Path>,
) -> Result<MoldOutputs> {
    let report = run_with_config(config, base_dir, output_dir_override)
        .map_err(|e| EngineError::MoldGen(format!("{e:#}")))?;

    // Per-layer poured masses come back in KILOGRAMS; the pour plan wants
    // grams (see LayerPour::mass_g). draft.layers is 1:1 with the run's
    // layers by construction (same design.toml).
    let masses_kg: Vec<f64> = report
        .v2
        .layers
        .iter()
        .map(|l| l.pour_volume.pour_mass_kg)
        .collect();
    let pour_plan = build_pour_plan(&pour_inputs(draft, &masses_kg))?;

    let stls = collect_stls(&report.out_dir)?;
    Ok(MoldOutputs {
        out_dir: report.out_dir,
        mold_stls: stls.mold,
        plug_stls: stls.plug,
        accessory_stls: stls.accessory,
        procedure_path: report.procedure_path,
        total_mass_g: report.total_mass_g,
        pour_plan,
    })
}

/// Zip the design (anchor keys + Slacker) with the run's per-layer masses
/// into [`LayerPour`]s, converting **kg → g**. Slacker is lifted only
/// when non-zero, matching the cast pipeline's own convention.
fn pour_inputs(draft: &DesignDraft, layer_masses_kg: &[f64]) -> Vec<LayerPour> {
    draft
        .layers
        .iter()
        .zip(layer_masses_kg)
        .map(|(layer, &mass_kg)| LayerPour {
            anchor_key: layer.material_key.clone(),
            mass_g: mass_kg * 1000.0,
            slacker_fraction: (layer.slacker_fraction > 0.0).then_some(layer.slacker_fraction),
        })
        .collect()
}

/// The cast run's `.stl` outputs, bucketed for the UI.
#[derive(Debug)]
struct CategorizedStls {
    mold: Vec<PathBuf>,
    plug: Vec<PathBuf>,
    accessory: Vec<PathBuf>,
}

/// Read + categorize the `.stl` files the run wrote under `out_dir/stls`,
/// by filename: `mold_layer_*` → mold halves, `plug_layer_*` → plugs,
/// everything else (platform, dowel, funnel) → accessories. Globbing the
/// directory is robust to which optional artifacts the run emitted.
fn collect_stls(out_dir: &Path) -> Result<CategorizedStls> {
    let stls_dir = out_dir.join("stls");
    let entries = std::fs::read_dir(&stls_dir).map_err(|e| {
        EngineError::MoldGen(format!("read output dir {}: {e}", stls_dir.display()))
    })?;
    let mut mold = Vec::new();
    let mut plug = Vec::new();
    let mut accessory = Vec::new();
    for entry in entries {
        let path = entry
            .map_err(|e| {
                EngineError::MoldGen(format!("read dir entry in {}: {e}", stls_dir.display()))
            })?
            .path();
        if path.extension().and_then(|e| e.to_str()) != Some("stl") {
            continue;
        }
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or_default();
        if name.starts_with("mold_layer_") {
            mold.push(path);
        } else if name.starts_with("plug_layer_") {
            plug.push(path);
        } else {
            accessory.push(path);
        }
    }
    mold.sort();
    plug.sort();
    accessory.sort();
    Ok(CategorizedStls {
        mold,
        plug,
        accessory,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

    use cf_studio_core::LayerDraft;

    use super::*;

    fn temp_dir(label: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "cf-studio-engine-mold-test-{}-{label}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn pour_inputs_converts_kg_to_grams_and_maps_slacker() {
        let draft = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![
                LayerDraft {
                    thickness_m: 0.0175,
                    material_key: "ECOFLEX_00_30".to_string(),
                    slacker_fraction: 0.25,
                },
                LayerDraft {
                    thickness_m: 0.005,
                    material_key: "DRAGON_SKIN_20A".to_string(),
                    slacker_fraction: 0.0,
                },
            ],
        };
        // Use kg values whose ×1000 is exact in f64 to keep the assert clean.
        let inputs = pour_inputs(&draft, &[0.5, 0.25]);
        assert_eq!(inputs.len(), 2);
        assert_eq!(inputs[0].mass_g, 500.0); // 0.5 kg → 500 g
        assert_eq!(inputs[0].anchor_key, "ECOFLEX_00_30");
        assert_eq!(inputs[0].slacker_fraction, Some(0.25));
        assert_eq!(inputs[1].mass_g, 250.0);
        assert_eq!(inputs[1].slacker_fraction, None); // 0.0 → None
    }

    #[test]
    fn collect_stls_buckets_by_filename() {
        let dir = temp_dir("collect");
        let stls = dir.join("stls");
        std::fs::create_dir_all(&stls).unwrap();
        for f in [
            "mold_layer_0_piece_0.stl",
            "mold_layer_0_piece_1.stl",
            "plug_layer_0.stl",
            "platform.stl",
            "dowel.stl",
            "procedure.md", // non-stl, ignored
        ] {
            std::fs::write(stls.join(f), b"x").unwrap();
        }

        let cat = collect_stls(&dir).unwrap();
        assert_eq!(cat.mold.len(), 2, "two mold halves");
        assert_eq!(cat.plug.len(), 1, "one plug");
        assert_eq!(cat.accessory.len(), 2, "platform + dowel; .md ignored");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn missing_output_dir_is_a_mold_gen_error() {
        let err = collect_stls(Path::new("/no/such/out")).unwrap_err();
        assert!(matches!(err, EngineError::MoldGen(_)), "got: {err:?}");
    }

    /// End-to-end integration on the real base_mold (slow ~13 min, needs
    /// the `~/scans/base_mold*` files). Run manually:
    /// `cargo test -p cf-studio-engine -- --ignored generate_molds`.
    #[test]
    #[ignore = "integration: ~13 min, needs ~/scans/base_mold files"]
    fn generate_molds_runs_base_mold_end_to_end() {
        let scans = PathBuf::from(std::env::var("HOME").unwrap()).join("scans");
        let cast_toml =
            std::fs::read_to_string(scans.join("cast.base_mold.canal.05.toml")).unwrap();
        let config = CastConfig::from_toml_str(&cast_toml).unwrap();
        let draft = DesignDraft {
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
        };

        let out = generate_molds(
            config,
            &draft,
            &scans,
            Some(Path::new("cast_base_mold_studio_verify")),
        )
        .unwrap();

        assert_eq!(out.pour_plan.steps.len(), 3);
        assert_eq!(out.mold_stls.len(), 6, "2 halves × 3 layers");
        assert_eq!(out.plug_stls.len(), 3, "1 plug × 3 layers");
        assert!(!out.accessory_stls.is_empty(), "platform + dowel");
        assert!(out.total_mass_g > 800.0 && out.total_mass_g < 900.0);
        assert_eq!(
            out.pour_plan.steps[0].material_display_name,
            "Ecoflex 00-30"
        );

        let _ = std::fs::remove_dir_all(scans.join("cast_base_mold_studio_verify"));
    }
}
