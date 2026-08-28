//! The mold-generation boundary (workflow step "Make molds"): drive the
//! cast pipeline from a **typed** [`CastConfig`] (no `cast.toml` to author
//! or re-parse) and collect the printable STLs + procedure + structured
//! pour plan into a [`MoldOutputs`].

use std::path::{Path, PathBuf};

use cf_studio_core::{DesignDraft, MoldOutputs, RidgeOptions};
use cortenforge::cf_cast_cli::{
    CanalConfig, CastConfig, CastMode, PartSelection, RingConfig, run_selected_with_config,
    run_with_config,
};

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
/// `ridges` is the optional surface texture chosen in "Shape your piece" (the
/// same field the live preview showed). It is composed onto the cleaned-scan
/// surface, so it rides every offset — the plug *and* every shell carry the
/// identical displacement and the wall thicknesses stay constant. A default
/// (disabled) [`RidgeOptions`] reproduces the historical no-texture cast
/// exactly. The cavity inset comes from `draft.cavity_inset_m` (set on the
/// same step).
///
/// `selection` chooses which mold pieces to generate. [`PartSelection::all`]
/// runs the full cast (the default wizard path, byte-identical to before);
/// a narrower selection meshes **only** the chosen pieces (e.g. one layer-0
/// plug) and skips the rest — the time-saving "regenerate just this part"
/// flow. The pour plan stays complete for every layer regardless.
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
// The wizard's single "make molds" entry point genuinely needs all of these
// inputs (scan paths, design, quality, ridges, part selection, cast mode);
// bundling them into a struct would just move the argument list, not shorten it.
#[allow(clippy::too_many_arguments)]
pub fn generate_molds_for_design(
    cleaned_stl: &Path,
    prep_toml: &Path,
    draft: &DesignDraft,
    mesh_cell_size_m: f64,
    ridges: &RidgeOptions,
    selection: &PartSelection,
    cast_mode: CastMode,
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
        canal_config_from_ridges(ridges),
    );
    // Detachable + everything selected → the validated full export (byte-
    // identical detachable cast). Otherwise — a subset, OR any bonded cast —
    // route through the selective export, which skips unselected pieces'
    // marching cubes and emits the procedure for `cast_mode`.
    if cast_mode == CastMode::Detachable && selection.is_all() {
        generate_molds(config, draft, base_dir, output_dir_override)
    } else {
        generate_selected_molds(
            config,
            draft,
            base_dir,
            output_dir_override,
            selection,
            cast_mode,
        )
    }
}

/// Run the cast for a subset of parts ([`run_selected_with_config`]) and
/// assemble a [`MoldOutputs`]. The pour plan is built from the per-layer
/// masses the selective run still computes for **every** layer, and the STL
/// buckets come from globbing what landed on disk (so only generated pieces
/// appear).
///
/// # Errors
/// [`EngineError::MoldGen`] if the cast run / output read-back fails;
/// [`EngineError::PourDataUnavailable`] if a layer has no cure data.
fn generate_selected_molds(
    config: CastConfig,
    draft: &DesignDraft,
    base_dir: &Path,
    output_dir_override: Option<&Path>,
    selection: &PartSelection,
    cast_mode: CastMode,
) -> Result<MoldOutputs> {
    let report =
        run_selected_with_config(config, base_dir, output_dir_override, selection, cast_mode)
            .map_err(|e| EngineError::MoldGen(format!("{e:#}")))?;
    let pour_plan = build_pour_plan(&pour_inputs(draft, &report.layer_pour_masses_kg)?)?;
    // Bucket from the run's AUTHORITATIVE written-paths list, NOT by globbing
    // the output dir: the dir is persistent and never cleared, so globbing
    // would merge stale pieces from a prior full run (possibly pre-edit
    // geometry) with the freshly regenerated piece. Only `report.written`
    // reflects what THIS selective run actually produced.
    let stls = categorize_stls(report.written);
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

/// Map the wizard's owned, sanitized [`RidgeOptions`] onto cf-cast-cli's
/// [`CanalConfig`]. A disabled `ridges` yields `CanalConfig::default()`
/// (the no-op the cast pipeline bit-preserves), so the canal-off path stays
/// byte-identical. When enabled, every field is set explicitly from the
/// options so the UI is the single source of truth (no reliance on the
/// `CanalSpec::iter1` fallbacks). `orientation_deg` maps to a frenulum
/// direction in the channel's cross-section: `θ → [sin θ, cos θ, 0]`, so
/// `0°` is the validated `[0, 1, 0]` default.
pub(crate) fn canal_config_from_ridges(ridges: &RidgeOptions) -> CanalConfig {
    if !ridges.enabled {
        return CanalConfig::default();
    }
    let theta = ridges.orientation_deg.to_radians();
    CanalConfig {
        enabled: true,
        rings: Some(
            ridges
                .rings
                .iter()
                .map(|r| RingConfig {
                    center_frac: r.position_frac,
                    depth_m: r.depth_m,
                    half_width_frac: r.half_width_frac,
                })
                .collect(),
        ),
        frenulum_dir: Some([theta.sin(), theta.cos(), 0.0]),
        texture_amplitude_m: Some(ridges.texture_depth_m),
        texture_pitch_m: Some(ridges.texture_spacing_m),
        dsection_depth_m: Some(ridges.side_pinch_depth_m),
        suction_bulge_m: Some(ridges.tip_relief_depth_m),
        // Plug-only mesh resolution stays at the CanalConfig default
        // (0.5 mm) so the ~1.5 mm ribs survive regardless of the cups'
        // mesh_cell_size_m.
        plug_mesh_cell_size_m: None,
    }
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
    let pour_plan = build_pour_plan(&pour_inputs(draft, &masses_kg)?)?;

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
///
/// # Errors
/// [`EngineError::MoldGen`] if the design's layer count doesn't match the
/// run's — `generate_molds` is public and takes an arbitrary `CastConfig`
/// whose design need not agree with `draft`, and a silent `zip` truncation
/// would drop pour steps while `total_mass_g` still reports the full mass.
fn pour_inputs(draft: &DesignDraft, layer_masses_kg: &[f64]) -> Result<Vec<LayerPour>> {
    if draft.layers.len() != layer_masses_kg.len() {
        return Err(EngineError::MoldGen(format!(
            "design has {} layer(s) but the cast produced {} — they must match",
            draft.layers.len(),
            layer_masses_kg.len()
        )));
    }
    Ok(draft
        .layers
        .iter()
        .zip(layer_masses_kg)
        .map(|(layer, &mass_kg)| LayerPour {
            anchor_key: layer.material_key.clone(),
            mass_g: mass_kg * 1000.0,
            slacker_fraction: (layer.slacker_fraction > 0.0).then_some(layer.slacker_fraction),
        })
        .collect())
}

/// The cast run's `.stl` outputs, bucketed for the UI.
#[derive(Debug)]
struct CategorizedStls {
    mold: Vec<PathBuf>,
    plug: Vec<PathBuf>,
    accessory: Vec<PathBuf>,
}

/// Categorize STL paths by filename: `mold_layer_*` → mold halves,
/// `plug_layer_*` → plugs, everything else (platform, dowel, funnel) →
/// accessories. Sorted within each bucket. Non-`.stl` paths are dropped.
fn categorize_stls(paths: impl IntoIterator<Item = PathBuf>) -> CategorizedStls {
    let mut mold = Vec::new();
    let mut plug = Vec::new();
    let mut accessory = Vec::new();
    for path in paths {
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
    CategorizedStls {
        mold,
        plug,
        accessory,
    }
}

/// Read + categorize the `.stl` files the run wrote under `out_dir/stls` by
/// globbing the directory (robust to which optional artifacts the run
/// emitted). Used by the **full-cast** path, where every file in the dir
/// belongs to this run. The selective path must NOT glob (the dir is
/// persistent + may hold stale pieces) — it categorizes the run's authoritative
/// written-paths list via [`categorize_stls`].
fn collect_stls(out_dir: &Path) -> Result<CategorizedStls> {
    let stls_dir = out_dir.join("stls");
    let entries = std::fs::read_dir(&stls_dir).map_err(|e| {
        EngineError::MoldGen(format!("read output dir {}: {e}", stls_dir.display()))
    })?;
    let mut paths = Vec::new();
    for entry in entries {
        let path = entry
            .map_err(|e| {
                EngineError::MoldGen(format!("read dir entry in {}: {e}", stls_dir.display()))
            })?
            .path();
        paths.push(path);
    }
    Ok(categorize_stls(paths))
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
    fn disabled_ridges_map_to_the_default_canal_off_config() {
        // The canal-off path must stay the bit-preserved no-op: a disabled
        // RidgeOptions yields exactly CanalConfig::default().
        let canal = canal_config_from_ridges(&RidgeOptions::default());
        assert!(!canal.enabled, "ridges off → canal disabled");
        assert!(canal.rings.is_none(), "no ring overrides → iter1 fallback");
        assert!(canal.frenulum_dir.is_none());
        assert!(canal.texture_amplitude_m.is_none());
    }

    #[test]
    fn enabled_ridges_map_every_field_explicitly() {
        let ridges = RidgeOptions {
            enabled: true,
            rings: vec![
                cf_studio_core::RidgeRing {
                    position_frac: 0.1,
                    depth_m: 0.003,
                    half_width_frac: 0.04,
                },
                cf_studio_core::RidgeRing {
                    position_frac: 0.5,
                    depth_m: 0.002,
                    half_width_frac: 0.05,
                },
            ],
            texture_depth_m: 0.0012,
            texture_spacing_m: 0.007,
            side_pinch_depth_m: 0.001,
            tip_relief_depth_m: 0.0025,
            orientation_deg: 0.0,
        };
        let canal = canal_config_from_ridges(&ridges);
        assert!(canal.enabled);
        let rings = canal.rings.expect("rings carried through");
        assert_eq!(rings.len(), 2);
        assert_eq!(rings[1].center_frac, 0.5);
        assert_eq!(rings[1].depth_m, 0.002);
        assert_eq!(canal.texture_amplitude_m, Some(0.0012));
        assert_eq!(canal.texture_pitch_m, Some(0.007));
        assert_eq!(canal.dsection_depth_m, Some(0.001));
        assert_eq!(canal.suction_bulge_m, Some(0.0025));
        // 0° orientation → the validated [0, 1, 0] default direction.
        let dir = canal.frenulum_dir.expect("orientation mapped");
        assert!(dir[0].abs() < 1e-12, "x ≈ 0");
        assert!((dir[1] - 1.0).abs() < 1e-12, "y ≈ 1");
        assert_eq!(dir[2], 0.0);
    }

    #[test]
    fn ridge_orientation_rotates_in_the_cross_section() {
        // 90° puts the asymmetry axis on +X; the channel-axis (z) component
        // stays zero so the projection keeps it perpendicular to the axis.
        let ridges = RidgeOptions {
            enabled: true,
            orientation_deg: 90.0,
            ..RidgeOptions::default()
        };
        let dir = canal_config_from_ridges(&ridges)
            .frenulum_dir
            .expect("orientation mapped");
        assert!((dir[0] - 1.0).abs() < 1e-12, "x ≈ 1 at 90°");
        assert!(dir[1].abs() < 1e-12, "y ≈ 0 at 90°");
        assert_eq!(dir[2], 0.0);
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
        let inputs = pour_inputs(&draft, &[0.5, 0.25]).unwrap();
        assert_eq!(inputs.len(), 2);
        assert_eq!(inputs[0].mass_g, 500.0); // 0.5 kg → 500 g
        assert_eq!(inputs[0].anchor_key, "ECOFLEX_00_30");
        assert_eq!(inputs[0].slacker_fraction, Some(0.25));
        assert_eq!(inputs[1].mass_g, 250.0);
        assert_eq!(inputs[1].slacker_fraction, None); // 0.0 → None
    }

    #[test]
    fn pour_inputs_rejects_layer_count_mismatch() {
        let draft = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![LayerDraft {
                thickness_m: 0.0175,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.0,
            }],
        };
        // 1 design layer vs 2 cast masses → error, not a silent truncation.
        let err = pour_inputs(&draft, &[0.5, 0.25]).unwrap_err();
        assert!(matches!(err, EngineError::MoldGen(_)), "got: {err:?}");
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
        let _ = generate_molds_for_design(
            &cleaned,
            &prep,
            &draft,
            0.003,
            &RidgeOptions::default(),
            &PartSelection::all(),
            CastMode::Detachable,
            None,
        );
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
    fn categorize_stls_buckets_only_the_given_paths() {
        // The selective-export fix: bucket the run's authoritative written-paths
        // list, NOT a glob of the persistent dir — so a stale prior-run STL that
        // happens to sit in the same folder is never reported. categorize_stls
        // only ever sees the list it's handed.
        let cat = categorize_stls(vec![
            PathBuf::from("out/stls/plug_layer_0.stl"),
            PathBuf::from("out/stls/procedure.md"), // non-stl, dropped
        ]);
        assert_eq!(cat.plug.len(), 1, "only the one regenerated plug");
        assert!(cat.mold.is_empty(), "no cup halves in the written list");
        assert!(cat.accessory.is_empty(), "no platform/dowel; .md dropped");
        // A stale full-run STL on disk is irrelevant — it's not in the list, so
        // it can't be mis-attributed to this run.
        assert!(
            !cat.plug
                .iter()
                .chain(&cat.mold)
                .chain(&cat.accessory)
                .any(|p| p.ends_with("mold_layer_0_piece_0.stl")),
        );
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
    fn cast_real_base_mold_via_wizard(
        cell_size_m: f64,
        ridges: &RidgeOptions,
        selection: &PartSelection,
        out_name: &str,
    ) {
        let (dir, cleaned, prep) = isolated_base_mold_fixture(out_name);
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
            ridges,
            selection,
            CastMode::Detachable,
            Some(Path::new(out_name)),
        )
        .unwrap();

        assert_eq!(out.mold_stls.len(), 6, "2 halves × 3 layers");
        assert_eq!(out.plug_stls.len(), 3, "1 plug × 3 layers");
        assert_eq!(out.pour_plan.steps.len(), 3);
        assert!(out.total_mass_g > 0.0);
        discard_fixture(&dir);
    }

    /// Remove a fixture directory once its test has PASSED.
    ///
    /// ⚠ Called after the assertions, never on the failure path, so a red gate
    /// leaves its output to be inspected — the behaviour the `~/scans` cleanup
    /// this replaced already had.
    ///
    /// ★ Not optional. A single gate produces up to ~240 MB of cast output, the
    /// directory name carries the PID so every run makes a fresh set, and the
    /// first draft of this change simply dropped the old cleanup: 1.43 GB
    /// accumulated across 12 directories before anyone looked (2026-08-27).
    fn discard_fixture(dir: &Path) {
        if let Err(err) = std::fs::remove_dir_all(dir) {
            eprintln!("WARN: could not remove fixture {}: {err}", dir.display());
        }
    }

    /// A private copy of the `base_mold` scan fixture, in this test's own temp
    /// directory. `None` when the fixture is absent (skip, as `edit.rs` does).
    ///
    /// ★★ WHY THIS EXISTS. These gates used to hand
    /// [`generate_molds_for_design`] the paths in `~/scans` directly, and it
    /// writes `<scan-stem>.design.toml` NEXT TO THE SCAN. Two consequences,
    /// both hit on 2026-08-27 the first time the set was run since ~June:
    ///
    /// 1. **They overwrote the operator's real `base_mold.design.toml`** — live
    ///    workshop data, replaced by this function's `DesignDraft`. `edit.rs`'s
    ///    sibling gate already documented the rule ("writes to a temp dir
    ///    (never `~/scans`)"); these did not follow it.
    /// 2. **They raced each other.** Five tests, one shared path, and cargo runs
    ///    them in parallel: three failed with `rename …design.toml.tmp →
    ///    …design.toml: No such file or directory`, one winning the rename and
    ///    the others losing their temp file. The per-test run commands in the
    ///    doc comments below are single-test filters, which is how that went
    ///    unnoticed — the set was never run as a set.
    ///
    /// ⚠ A gate that damages its own inputs does not get run, and a gate that
    /// does not get run rots. That is the actual reason these sat idle for
    /// three months, and it is fixed here rather than documented around.
    ///
    /// ⚠ The first version of this comment claimed the ~9 MB STL is copied
    /// rather than symlinked because "sharing it by symlink would put the
    /// `.design.toml` write back into a shared parent". **That is false** —
    /// `generate_molds_for_design` computes `base_dir.join(design_name)` from
    /// the parent of the path it is GIVEN, so a symlink inside the temp dir
    /// resolves to the temp dir either way. It is copied because a copy cannot
    /// be invalidated by the scan being edited mid-run, and 9 MB is cheap
    /// beside the ~240 MB of cast output a single gate produces.
    fn isolated_base_mold_fixture(label: &str) -> (PathBuf, PathBuf, PathBuf) {
        let scans = PathBuf::from(std::env::var("HOME").unwrap()).join("scans");
        let (src_stl, src_prep) = (
            scans.join("base_mold.cleaned.stl"),
            scans.join("base_mold.prep.toml"),
        );
        // ⚠⚠ PANICS, does not skip. An earlier draft of this helper returned
        // `None` here and each caller returned early — so on any machine
        // without these files all six gates reported GREEN having executed
        // nothing, and `eprintln!` from a passing test is captured, so it said
        // so silently. That is the "green by absence" failure this whole change
        // exists to remove, reintroduced while fixing it.
        //
        // These gates are `#[ignore]`d: they never run by accident, only when
        // someone asks for them by name. A missing fixture is therefore an
        // ERROR — the run that was requested cannot happen — not a pass.
        assert!(
            src_stl.exists() && src_prep.exists(),
            "MISSING FIXTURE: {} and {} are required by this #[ignore]d gate. \
             It runs only when explicitly asked for, so this is an error rather \
             than a skip — a silent pass here would report the mold path as \
             verified when nothing ran.",
            src_stl.display(),
            src_prep.display(),
        );
        let dir = std::env::temp_dir().join(format!(
            "cf-studio-mold-gate-{}-{label}",
            std::process::id()
        ));
        // ⚠ Defensive only, and deliberately kept: the PID is in the name, so
        // a PREVIOUS run's directory is never this one. What it actually guards
        // is a same-process collision (two labels colliding, or a re-entrant
        // call), where a leftover `.design.toml` would be exactly the state
        // these gates are supposed to produce themselves. Said plainly because
        // "clears stale state from the last run" would be a guard described by
        // a scenario it cannot see.
        if let Err(err) = std::fs::remove_dir_all(&dir) {
            assert_eq!(
                err.kind(),
                std::io::ErrorKind::NotFound,
                "could not clear {}: {err}",
                dir.display()
            );
        }
        std::fs::create_dir_all(&dir).unwrap();
        let (cleaned, prep) = (
            dir.join("base_mold.cleaned.stl"),
            dir.join("base_mold.prep.toml"),
        );
        std::fs::copy(&src_stl, &cleaned).unwrap();
        std::fs::copy(&src_prep, &prep).unwrap();
        (dir, cleaned, prep)
    }

    /// Fine 0.5 mm — the GUI **default** (print quality; the physical
    /// fit-test print was 0.5 mm). Confirms the for_design recipe casts
    /// canal-*off* at 0.5 mm (only canal-*on* @0.5 mm was previously
    /// validated). Slow ~15 min. Run:
    /// `cargo test -p cf-studio-engine -- --ignored for_design_casts`.
    #[test]
    #[ignore = "integration: ~15 min, needs ~/scans/base_mold files"]
    fn generate_molds_for_design_casts_base_mold_at_fine() {
        cast_real_base_mold_via_wizard(
            0.0005,
            &RidgeOptions::default(),
            &PartSelection::all(),
            "cast_base_mold_studio_verify_0p5",
        );
    }

    /// Fast 1.5 mm preview — much quicker than the 0.5 mm finish. Measured
    /// 2026-08-27, solo: **269 s**. (All four wizard gates run CONCURRENTLY in
    /// 1319 s wall — they are isolated now, so running the set is cheaper than
    /// running them one at a time.)
    #[test]
    #[ignore = "integration: ~4.5 min (measured 2026-08-27), needs ~/scans/base_mold files"]
    fn generate_molds_for_design_casts_base_mold_at_fast() {
        cast_real_base_mold_via_wizard(
            0.0015,
            &RidgeOptions::default(),
            &PartSelection::all(),
            "cast_base_mold_studio_verify_1p5",
        );
    }

    /// Interior-ridges ON through the wizard for_design recipe (planar seam
    /// with apex pour and demand flange). Canal-on and the for_design recipe
    /// were each validated separately but never together — this is the gate
    /// that confirms the ridge opt-in casts clean molds. Slow (the plug meshes
    /// at 0.5 mm regardless of the cup cell size). Run:
    /// `cargo test -p cf-studio-engine -- --ignored casts_base_mold_with_ridges`.
    #[test]
    #[ignore = "integration: slow, needs ~/scans/base_mold files"]
    fn generate_molds_for_design_casts_base_mold_with_ridges() {
        cast_real_base_mold_via_wizard(
            0.0015,
            &RidgeOptions {
                enabled: true,
                ..RidgeOptions::default()
            },
            &PartSelection::all(),
            "cast_base_mold_studio_verify_ridges",
        );
    }

    /// Selective export through the wizard: generate ONLY the layer-0 plug
    /// and confirm just that one STL lands (no cups, no other plugs), while
    /// the pour plan still covers all 3 layers. The whole point of the
    /// feature — re-print one piece without the full cast. Faster than a full
    /// run (only one plug is meshed). Run:
    /// `cargo test -p cf-studio-engine -- --ignored generate_only_layer0_plug`.
    #[test]
    #[ignore = "integration: needs ~/scans/base_mold files"]
    fn generate_only_layer0_plug_via_wizard() {
        let (dir, cleaned, prep) = isolated_base_mold_fixture("layer0-plug");
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
        let selection =
            PartSelection::from_ids([cortenforge::cf_cast_cli::PartId::Plug { layer_index: 0 }]);
        let out_name = "cast_base_mold_studio_verify_plug_only";

        let out = generate_molds_for_design(
            &cleaned,
            &prep,
            &draft,
            0.0015,
            &RidgeOptions::default(),
            &selection,
            CastMode::Detachable,
            Some(Path::new(out_name)),
        )
        .unwrap();

        assert!(out.mold_stls.is_empty(), "no cup halves generated");
        assert_eq!(out.plug_stls.len(), 1, "only the layer-0 plug");
        assert!(out.plug_stls[0].ends_with("plug_layer_0.stl"));
        assert!(out.accessory_stls.is_empty(), "no platform/dowel");
        // Pour plan still spans all 3 layers (instructions, not files).
        assert_eq!(out.pour_plan.steps.len(), 3);
        discard_fixture(&dir);
    }

    /// End-to-end integration on the real base_mold. Run manually:
    /// `cargo test -p cf-studio-engine -- --ignored generate_molds`.
    ///
    /// ⚠ **~61 MINUTES, not the "~13 min" this said until 2026-08-27.**
    /// Measured that day, solo, on an M4 laptop: 3650 s wall, of which
    /// `export_molds_v2` was 3281 s. Treat it as an upper bound — the machine
    /// had been building all session — but the ORDER is right and the old
    /// figure was not. That mattered: "~13 min" invites a casual
    /// `--ignored` sweep; an hour is a deliberate decision.
    ///
    /// ★ Third stale cost figure found in one day (the cf-cast iter-1 gate
    /// claimed ~10 s for what takes 0.07 s, and ~6 min for a 96 s cast run).
    /// The direction varies; the cause does not — measured once, written into
    /// a doc comment, never re-checked. If you change what this runs, re-measure
    /// and say when.
    #[test]
    #[ignore = "integration: ~61 min (measured 2026-08-27), needs ~/scans/base_mold files"]
    fn generate_molds_runs_base_mold_end_to_end() {
        // `generate_molds` takes the scan DIRECTORY (the cast spec names
        // `base_mold.cleaned.stl` relative to it), so it gets the temp dir the
        // fixture was copied into.
        // ⚠ EVERY precondition is checked BEFORE the fixture is created.
        // Ordered the other way, this skip returned with a 9.3 MB fixture
        // directory already on disk and no `discard_fixture` — a leak on the
        // exact path that fires for anyone whose machine lacks this spec.
        //
        // The cast spec is READ-ONLY input, so it is read in place rather than
        // copied — nothing writes back to it.
        let spec = PathBuf::from(std::env::var("HOME").unwrap())
            .join("scans/cast.base_mold.canal.05.toml");
        assert!(
            spec.exists(),
            "MISSING FIXTURE: {} is required by this #[ignore]d gate (see \
             `isolated_base_mold_fixture` for why this is an error, not a skip).",
            spec.display(),
        );
        let cast_toml = std::fs::read_to_string(&spec).unwrap();
        let (dir, cleaned, _prep) = isolated_base_mold_fixture("end-to-end");
        let config = CastConfig::from_toml_str(&cast_toml).unwrap();
        // ⚠ BOTH names derive from the fixture, via the SAME `design_filename`
        // production uses (`generate_molds_for_design` does
        // `base_dir.join(design_filename(&cleaned_name))`). An earlier draft
        // derived the scan name and left this one a literal — one line apart,
        // agreeing only because the fixture happens to be called `base_mold`.
        let cleaned_name = cleaned.file_name().unwrap().to_str().unwrap();
        let design_path = dir.join(design_filename(cleaned_name));
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

        // ★ Materialize the design from THIS test's own draft.
        //
        // `generate_molds` runs the cast from `config` — whose `[design] path`
        // names `base_mold.design.toml` — and uses `draft` only for the pour
        // plan. Its doc comment states the coupling: "draft.layers is 1:1 with
        // the run's layers by construction (same design.toml)". Reading the
        // operator's `~/scans/base_mold.design.toml` made that 1:1 a matter of
        // LUCK: edit the layer stack in the GUI and this test's mass and
        // material assertions drift with it, for reasons nothing here explains.
        // Writing it from the draft makes the invariant hold by construction,
        // which is what the comment already claims.
        save_design_from_draft(Path::new(cleaned_name), &draft, &design_path).unwrap();

        let out = generate_molds(
            config,
            &draft,
            &dir,
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
        discard_fixture(&dir);
    }
}
