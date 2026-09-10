//! The output folder is persistent, and holds more than one run's work.
//!
//! `stls/` is never cleared. A selective export writes one STL beside the
//! nine it skipped; a full export after a recipe change writes fewer files
//! than the folder already holds. In both cases files nobody regenerated
//! stay behind, and on disk they are indistinguishable from fresh ones —
//! same folder, same extension, same naming scheme.
//!
//! Two claims are gated here, end to end through the real cast:
//!
//! 1. A run reports **only what it wrote**. This used to be false on the
//!    full-cast path, which globbed the folder and handed every `.stl` in
//!    it back as its own output — and `export_print_package` copies that
//!    list to the slicer folder, so an orphan got printed.
//! 2. The folder **records which run wrote which file**, so the leftovers
//!    can be named rather than guessed at.
//!
//! ★ Synthetic, for the reason `common` gives: the real configuration
//! lives outside the repo in `~/scans`, where a gate rots because nothing
//! makes it run.

// Same convention as the sibling gates in this directory — see `common`.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod common;

use std::path::{Path, PathBuf};

use cf_studio_core::{DesignDraft, LayerDraft, MoldOutputs, PrepInput, RidgeOptions};
use cf_studio_engine::{
    CastMode, EditSession, PROBE_LAYER_THICKNESS_M, PartSelection, generate_molds_for_design,
};
use common::open_cone;

/// A filename no cast emits, so its presence in a run's outputs can only
/// come from the folder rather than the run.
const ORPHAN: &str = "plug_layer_9.stl";

/// Cast the synthetic cone into `dir`, returning the run's reported
/// outputs. Unlike `common::cast_synthetic` the directory **survives**, so
/// a second cast can be run against the folder the first one left.
fn cast_into(dir: &Path, parts: &PartSelection) -> MoldOutputs {
    let mut session = EditSession::from_mesh(
        PathBuf::from("synthetic.stl"),
        open_cone(0.006, 0.020, 0.030, 24, 13),
    );
    assert_eq!(
        session.detect_caps().loop_count,
        2,
        "an open cone has two boundary loops"
    );
    session.save(dir, "synthetic", "mm", 0).expect("prep saves");

    generate_molds_for_design(
        &PrepInput {
            cleaned_stl: dir.join("synthetic.cleaned.stl"),
            prep_toml: dir.join("synthetic.prep.toml"),
        },
        &DesignDraft {
            cavity_inset_m: 0.0015,
            layers: vec![LayerDraft {
                thickness_m: PROBE_LAYER_THICKNESS_M,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            }],
        },
        0.003,
        &RidgeOptions::default(),
        parts,
        // Detachable + everything selected is the one routing that takes the
        // full export — the path that used to glob. A subset, or Bonded,
        // goes through the selective export instead.
        CastMode::Detachable,
        Some(Path::new("out")),
    )
    .expect("the synthetic cone casts")
}

/// Every filename a run reported, across all three buckets.
fn reported(out: &MoldOutputs) -> Vec<String> {
    out.mold_stls
        .iter()
        .chain(&out.plug_stls)
        .chain(&out.accessory_stls)
        .map(|p| p.file_name().unwrap().to_string_lossy().to_string())
        .collect()
}

fn temp_dir(label: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "cf-studio-engine-provenance-{label}-{}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A full cast reports what it wrote, not what it found.
///
/// ⚠ The orphan is planted **between** two casts into the same folder,
/// which is what makes this end-to-end rather than a unit test of the
/// bucketing: the second run has to walk the real export and come back
/// without it. Planting it before any cast would not do — the folder
/// would not exist yet.
#[test]
fn a_full_cast_never_reports_a_file_an_earlier_run_left_behind() {
    let dir = temp_dir("full");
    let first = cast_into(&dir, &PartSelection::all());
    let stls_dir = first.out_dir.join(cortenforge::cf_cast_cli::STLS_SUBDIR);

    // The fixture is only meaningful if the run actually emitted something.
    assert!(
        !reported(&first).is_empty(),
        "the first cast emitted nothing to compare against"
    );

    std::fs::write(stls_dir.join(ORPHAN), b"solid orphan\nendsolid orphan\n").unwrap();
    let second = cast_into(&dir, &PartSelection::all());

    assert!(
        stls_dir.join(ORPHAN).is_file(),
        "the cast must not delete it either — the fix is to stop CLAIMING it, \
         not to throw away a file the user may still want"
    );
    // ⚠ Not `!contains(ORPHAN)` alone — an empty list satisfies that, so a
    // run that reported NOTHING would pass. Same design, same parts, so the
    // second cast must report exactly what the first did: the orphan
    // excluded AND nothing legitimate dropped.
    assert_eq!(
        reported(&second),
        reported(&first),
        "a re-cast of the same design must report its own output, and only that"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// The folder says which run wrote which file — the thing that was missing
/// when a June plug sat beside a September one and nothing told them apart.
#[test]
fn the_manifest_separates_this_runs_output_from_what_was_already_there() {
    let dir = temp_dir("manifest");
    let first = cast_into(&dir, &PartSelection::all());
    let stls_dir = first.out_dir.join(cortenforge::cf_cast_cli::STLS_SUBDIR);
    let manifest = stls_dir.join(cortenforge::cf_cast_cli::MANIFEST_FILENAME);

    assert!(manifest.is_file(), "a cast writes the manifest");
    let after_first = std::fs::read_to_string(&manifest).unwrap();
    assert!(
        after_first.contains("latest_run = 1"),
        "first cast is run 1:\n{after_first}"
    );

    // Regenerate ONE part. Everything else in the folder is now a leftover.
    let plug_only =
        PartSelection::from_ids([cortenforge::cf_cast_cli::PartId::Plug { layer_index: 0 }]);
    let second = cast_into(&dir, &plug_only);
    assert_eq!(
        reported(&second),
        vec!["plug_layer_0.stl"],
        "the selective run reports only the part it regenerated"
    );

    let after_second = std::fs::read_to_string(&manifest).unwrap();
    assert!(
        after_second.contains("latest_run = 2"),
        "second cast is run 2:\n{after_second}"
    );
    // The regenerated plug advanced; a cup half the run never touched kept
    // the run that wrote it. Both halves matter — a manifest that stamped
    // everything every run would report nothing stale, and one that stamped
    // nothing would report everything.
    assert!(
        after_second.contains("file = \u{22}plug_layer_0.stl\u{22}\nrun = 2"),
        "the regenerated plug is run 2:\n{after_second}"
    );
    assert!(
        after_second.contains("file = \u{22}mold_layer_0_piece_0.stl\u{22}\nrun = 1"),
        "the untouched cup half kept run 1:\n{after_second}"
    );

    let _ = std::fs::remove_dir_all(&dir);
}
