//! The mold-generation boundary (workflow step "Make molds"): drive the
//! cast pipeline from a **typed** [`CastConfig`] (no `cast.toml` to author
//! or re-parse) and collect the printable STLs + procedure + structured
//! pour plan into a [`MoldOutputs`].

use std::path::{Path, PathBuf};

use cf_cast_cli::{CastConfig, run_with_config};
use cf_studio_core::{DesignDraft, MoldOutputs};

use crate::design::save_design_from_draft;
use crate::error::{EngineError, Result};
use crate::pour::{LayerPour, build_pour_plan};

/// Generate the molds for the **guided-wizard** path, from the project's
/// own artifacts: the cleaned scan + its `.prep.toml` (step 2) and the
/// in-app [`DesignDraft`] (step 3). This is the single call a frontend
/// makes for "Make molds" — it is the composition that:
///
/// 1. writes the `design.toml` next to the cleaned scan (step 3 keeps the
///    draft only in memory, so it's materialized here, derived from the
///    scan's stem: `foo.cleaned.stl` → `foo.design.toml`);
/// 2. builds a typed [`CastConfig`] via [`CastConfig::for_design`] at the
///    chosen `mesh_cell_size_m` (the quality knob), with relative paths so
///    everything resolves under the scan's directory; and
/// 3. runs the cast via [`generate_molds`].
///
/// All arguments are `Send`, so a frontend can call this straight off a
/// background thread (the run is minutes-long). `cleaned_stl` must be an
/// absolute path to a `*.stl` in the scan directory; its parent is the
/// cast run's `base_dir`.
///
/// # Errors
/// - [`EngineError::MoldGen`] if a path lacks a filename, the design write
///   fails, or the cast run / output read-back fails.
/// - [`EngineError::WriteDesign`] / [`EngineError::InvalidDesign`] /
///   [`EngineError::UnknownMaterial`] if the draft can't be materialized.
/// - [`EngineError::PourDataUnavailable`] if a layer has no cure data.
pub fn generate_molds_for_design(
    cleaned_stl: &Path,
    prep_toml: &Path,
    draft: &DesignDraft,
    mesh_cell_size_m: f64,
    output_dir_override: Option<&Path>,
) -> Result<MoldOutputs> {
    let base_dir = cleaned_stl.parent().unwrap_or_else(|| Path::new("."));
    let cleaned_name = file_name(cleaned_stl)?;
    let prep_name = file_name(prep_toml)?;
    let design_name = design_filename(&cleaned_name);
    let design_path = base_dir.join(&design_name);

    // Materialize the in-app design. Anchor it to the cleaned scan's
    // filename (relative — it's informational; the config's scan block is
    // authoritative for the run).
    save_design_from_draft(Path::new(&cleaned_name), draft, &design_path)?;

    let config = CastConfig::for_design(
        PathBuf::from(&cleaned_name),
        PathBuf::from(&prep_name),
        PathBuf::from(&design_name),
        mesh_cell_size_m,
    );
    generate_molds(config, draft, base_dir, output_dir_override)
}

/// The filename component of `p` as an owned `String`, or a `MoldGen`
/// error naming the offending path.
fn file_name(p: &Path) -> Result<String> {
    p.file_name()
        .and_then(|n| n.to_str())
        .map(str::to_string)
        .ok_or_else(|| EngineError::MoldGen(format!("path has no file name: {}", p.display())))
}

/// Derive the `design.toml` filename from a cleaned-scan filename by
/// stripping a trailing `.cleaned.stl` (or bare `.stl`) and appending
/// `.design.toml`: `base_mold.cleaned.stl` → `base_mold.design.toml`.
fn design_filename(cleaned_name: &str) -> String {
    let stem = cleaned_name
        .strip_suffix(".cleaned.stl")
        .or_else(|| cleaned_name.strip_suffix(".stl"))
        .unwrap_or(cleaned_name);
    format!("{stem}.design.toml")
}

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
    fn design_filename_strips_cleaned_stl_then_appends_design_toml() {
        assert_eq!(
            design_filename("base_mold.cleaned.stl"),
            "base_mold.design.toml"
        );
        // Bare .stl (a scan saved without the .cleaned infix).
        assert_eq!(design_filename("foo.stl"), "foo.design.toml");
        // No recognized suffix → append as-is (defensive, shouldn't happen).
        assert_eq!(design_filename("weird"), "weird.design.toml");
    }

    #[test]
    fn file_name_extracts_component_or_errors() {
        assert_eq!(
            file_name(Path::new("/home/u/scans/base_mold.cleaned.stl")).unwrap(),
            "base_mold.cleaned.stl"
        );
        // Root has no filename component.
        assert!(matches!(
            file_name(Path::new("/")).unwrap_err(),
            EngineError::MoldGen(_)
        ));
    }

    #[test]
    fn generate_molds_for_design_writes_the_design_toml_before_running() {
        // The cast run itself is the slow part (covered by the #[ignore]
        // integration); here we only assert the design.toml is materialized
        // next to the scan, by pointing at a scan dir with no real geometry
        // so the run fails fast *after* the write.
        let dir = temp_dir("for-design-write");
        let cleaned = dir.join("base_mold.cleaned.stl");
        let prep = dir.join("base_mold.prep.toml");
        std::fs::write(&cleaned, b"not a real stl").unwrap();
        std::fs::write(&prep, b"[centerline]\npoints_m = [[0,0,0],[0,0,0.01]]\n").unwrap();

        let draft = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![LayerDraft {
                thickness_m: 0.0175,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            }],
        };

        // We don't care whether the cast run succeeds — only that the
        // design.toml was written first (the materialization is the new glue).
        let _ = generate_molds_for_design(&cleaned, &prep, &draft, 0.003, None);
        let design_path = dir.join("base_mold.design.toml");
        assert!(
            design_path.exists(),
            "design.toml materialized next to the scan"
        );
        let written = std::fs::read_to_string(&design_path).unwrap();
        assert!(written.contains("ECOFLEX_00_30"), "with the draft's layer");
        assert!(
            written.contains("base_mold.cleaned.stl"),
            "anchored to the cleaned scan filename"
        );

        let _ = std::fs::remove_dir_all(&dir);
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

    /// Drive the wizard mold-gen path (for_design recipe) on the real
    /// base_mold at `cell_size_m`, into a throwaway dir, and assert it casts
    /// (6 mold halves + 3 plugs + 3-step pour plan). Shared by the per-
    /// resolution integration tests. Uses the in-app stack the GUI defaults
    /// to (whole-mm 18 / 7 / 5) and writes base_mold.design.toml next to the
    /// scan (same content the wizard already wrote).
    fn cast_real_base_mold_via_wizard(cell_size_m: f64, out_name: &str) {
        let scans = PathBuf::from(std::env::var("HOME").unwrap()).join("scans");
        let cleaned = scans.join("base_mold.cleaned.stl");
        let prep = scans.join("base_mold.prep.toml");
        let draft = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![
                LayerDraft {
                    thickness_m: 0.018,
                    material_key: "ECOFLEX_00_30".to_string(),
                    slacker_fraction: 0.25,
                },
                LayerDraft {
                    thickness_m: 0.007,
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

        let out = generate_molds_for_design(
            &cleaned,
            &prep,
            &draft,
            cell_size_m,
            Some(Path::new(out_name)),
        )
        .unwrap();

        assert_eq!(out.mold_stls.len(), 6, "2 halves × 3 layers");
        assert_eq!(out.plug_stls.len(), 3, "1 plug × 3 layers");
        assert_eq!(out.pour_plan.steps.len(), 3);
        assert!(out.total_mass_g > 0.0);

        let _ = std::fs::remove_dir_all(scans.join(out_name));
    }

    /// Fine 0.5 mm — the GUI **default** (print quality; the physical
    /// fit-test print was 0.5 mm). Confirms the for_design recipe casts
    /// canal-*off* at 0.5 mm (only canal-*on* @0.5 mm was previously
    /// validated). Slow ~15 min. Run:
    /// `cargo test -p cf-studio-engine -- --ignored for_design_casts`.
    #[test]
    #[ignore = "integration: ~15 min, needs ~/scans/base_mold files"]
    fn generate_molds_for_design_casts_base_mold_at_fine() {
        cast_real_base_mold_via_wizard(0.0005, "cast_base_mold_studio_verify_0p5");
    }

    /// Fast 1.5 mm preview — much quicker (~minutes) than the 0.5 mm finish.
    #[test]
    #[ignore = "integration: ~minutes, needs ~/scans/base_mold files"]
    fn generate_molds_for_design_casts_base_mold_at_fast() {
        cast_real_base_mold_via_wizard(0.0015, "cast_base_mold_studio_verify_1p5");
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
