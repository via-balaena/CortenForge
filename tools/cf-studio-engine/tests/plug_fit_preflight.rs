//! The step-3 fit pre-flight must answer exactly what the cast would answer,
//! and must not touch the operator's files doing it.

// `panic`/`unwrap`/`expect` are the integration-test idiom here (same
// convention as the sibling cast gates): the crate denies them for library
// code, where errors are values, but a test failure has to be readable.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::{Path, PathBuf};

use cf_studio_core::{DesignDraft, LayerDraft, RidgeOptions};
use cf_studio_engine::{
    CastMode, EditSession, EngineError, PartId, PartSelection, PlugFit, generate_molds_for_design,
    plug_fit_preflight,
};

mod common;
use common::{CastOutcome, cast_synthetic_with_ridges, open_cone};

/// Cell size for both sides of the comparison.
///
/// ⚠ MATCHED on purpose, and the gate is void without it. Detachment turns on
/// sub-cell grid alignment, so pre-flight and cast agree only when asked at the
/// same resolution — measured 2026-09-08 on `~/scans/base_mold`, 2.0 and 3.0 mm
/// cells cast a 6.3 mm inset that both shipped sizes refuse. Matching the cell
/// size keeps this test about the COMPOSITION; whether one cell size predicts
/// another is a different question and not this test's business.
const CELL_SIZE_M: f64 = 0.003;

/// Insets to compare, in mm. On this cone at this cell size the plug detaches
/// from 6 mm (see `plug_lock_connectivity`), so 2 and 5 cast while 8 and 11 do
/// not — **both verdicts are exercised**, which is what stops a pre-flight that
/// always says one thing from passing.
const INSETS_MM: [f64; 4] = [2.0, 5.0, 8.0, 11.0];

/// The cone the cast gates here share: tapered, so the plug's base recedes from
/// the cap plane under an inward offset. A straight tube cannot show this.
fn cone() -> cortenforge::mesh::types::IndexedMesh {
    open_cone(0.006, 0.020, 0.030, 24, 13)
}

/// Write a cleaned scan + its `.prep.toml` into a fresh directory and return
/// it. The caller owns the cleanup.
///
/// ⚠ `caller` keeps concurrent fixtures apart: tests in one binary run on
/// parallel threads of ONE process, and this opens by DELETING the directory
/// it is about to build in.
fn fixture(caller: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("cf-preflight-{caller}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let mut session = EditSession::from_mesh(PathBuf::from("synthetic.stl"), cone());
    assert_eq!(
        session.detect_caps().loop_count,
        2,
        "an open cone has two boundary loops"
    );
    session
        .save(&dir, "synthetic", "mm", 0)
        .expect("prep saves");
    dir
}

/// `(cleaned scan, prep)` inside a fixture directory.
fn paths(dir: &Path) -> (PathBuf, PathBuf) {
    (
        dir.join("synthetic.cleaned.stl"),
        dir.join("synthetic.prep.toml"),
    )
}

/// ★ THE GATE. The pre-flight's verdict must equal the cast's, at every inset.
///
/// The oracle is the REAL CAST — not a literal I typed, and not a mirror of the
/// function under test. That matters: the pre-flight exists only to answer the
/// cast's question early, so the cast is the only thing that can say whether it
/// does. If `cf-cast`'s compose path ever changes and the pre-flight does not
/// follow, this fails.
///
/// ⚠ The whole sweep runs before anything is asserted. Failing on the first
/// disagreement would hide the SHAPE of a divergence across the range, and
/// could leave one of the two verdicts unreached.
#[test]
fn the_preflight_returns_the_verdict_the_cast_would() {
    let dir = fixture("agrees");
    let (cleaned, prep) = paths(&dir);
    let mut problems = Vec::new();

    // ⚠ BOTH texture settings. Ridges route the plug through the canal path,
    // which composes displacement onto it and overrides its mesh cell size —
    // and that CHANGES THE VERDICT: measured 2026-09-08 on `~/scans/base_mold`,
    // ridges ON refuse a 5 mm inset that ridges OFF cast. A sweep that only
    // tried the smooth piece would have agreed with the cast about half the
    // configurations the step-3 screen can produce.
    let textures = [
        ("smooth", RidgeOptions::default()),
        (
            "ridged",
            RidgeOptions {
                enabled: true,
                ..RidgeOptions::default()
            },
        ),
    ];
    for (label, ridges) in &textures {
        // ⚠ Per texture, not across both. Global flags would let the ridged
        // sweep be entirely one-sided while the smooth one carried the claim —
        // exactly the vacuity these are here to prevent.
        let mut saw_cast = false;
        let mut saw_refusal = false;
        for inset_mm in INSETS_MM {
            let inset_m = inset_mm / 1e3;
            let preflight = plug_fit_preflight(&cleaned, &prep, inset_m, ridges, CELL_SIZE_M)
                .expect("the pre-flight must RUN — a broken fixture is not a verdict");
            // The oracle: the same cast the wizard runs, same cell size, same
            // texture, same one-layer stack the probe invents.
            let cast = cast_synthetic_with_ridges(
                &format!("preflight-oracle-{label}-{inset_mm}"),
                cone(),
                inset_m,
                CELL_SIZE_M,
                &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
                ridges,
            );
            match (&preflight, &cast) {
                (PlugFit::Casts, CastOutcome::Cast(stls)) => {
                    // The selection asks for exactly one part, so exactly one STL
                    // comes back — asserted rather than assumed, because a widened
                    // selection would quietly compare against a cup piece.
                    assert_eq!(
                        stls.len(),
                        1,
                        "plug-only selection emits one STL, got {:?}",
                        stls.iter().map(|(n, _)| n).collect::<Vec<_>>()
                    );
                    saw_cast = true;
                }
                (PlugFit::WillNotCast { .. }, CastOutcome::Refused(_)) => saw_refusal = true,
                (PlugFit::Casts, CastOutcome::Refused(msg)) => problems.push(format!(
                    "{label} {inset_mm} mm: pre-flight cleared it but the cast REFUSED — \
                 the operator would be told to go ahead and then lose the run: {msg}"
                )),
                (PlugFit::WillNotCast { reason }, CastOutcome::Cast(_)) => problems.push(format!(
                    "{label} {inset_mm} mm: pre-flight refused an inset the cast HONOURS — \
                 a usable configuration blocked: {reason}"
                )),
            }
        }
        assert!(
            saw_cast,
            "no inset in {INSETS_MM:?} mm cast with the {label} piece — that \
             sweep is one-sided and proves nothing about agreement"
        );
        assert!(
            saw_refusal,
            "no inset in {INSETS_MM:?} mm was refused with the {label} piece — \
             that sweep is one-sided and proves nothing about agreement"
        );
    }
    let _ = std::fs::remove_dir_all(&dir);

    assert!(
        problems.is_empty(),
        "the pre-flight must answer what the cast answers:\n{}",
        problems.join("\n")
    );
}

/// ★★ The pre-flight must not write into the scan folder — and the reason this
/// assertion is not vacuous is asserted too.
///
/// The obvious way to build this pre-flight is to run a cast somewhere and read
/// the outcome. `generate_molds_for_design` materializes `<stem>.design.toml`
/// beside the cleaned scan, so a pre-flight carrying an INVENTED one-layer
/// stack would overwrite the operator's real design. On a flat scan folder
/// (`~/scans` holds `base_mold.design.toml` right next to the STL) that is
/// their saved work.
///
/// So this checks both halves: the pre-flight leaves the file alone, AND a real
/// cast does not — because an "it was not modified" assertion proves nothing
/// unless something in the test can actually modify it.
#[test]
fn the_preflight_leaves_the_scan_folder_alone() {
    let dir = fixture("no-writes");
    let (cleaned, prep) = paths(&dir);
    let design = dir.join("synthetic.design.toml");
    const SENTINEL: &str = "# the operator's real design — must survive a pre-flight\n";
    std::fs::write(&design, SENTINEL).unwrap();
    let before: Vec<_> = listing(&dir);

    plug_fit_preflight(
        &cleaned,
        &prep,
        0.005,
        &RidgeOptions::default(),
        CELL_SIZE_M,
    )
    .expect("the pre-flight must run");

    assert_eq!(
        std::fs::read_to_string(&design).unwrap(),
        SENTINEL,
        "the pre-flight rewrote the operator's design file"
    );
    assert_eq!(
        listing(&dir),
        before,
        "the pre-flight left something behind in the scan folder"
    );

    // The other half: prove the sentinel CAN be clobbered, so the assertion
    // above is a real gate and not a statement about an inert file.
    let draft = DesignDraft {
        cavity_inset_m: 0.005,
        layers: vec![LayerDraft {
            thickness_m: 0.006,
            material_key: "ECOFLEX_00_30".to_string(),
            slacker_fraction: 0.25,
        }],
    };
    let _ = generate_molds_for_design(
        &cleaned,
        &prep,
        &draft,
        CELL_SIZE_M,
        &RidgeOptions::default(),
        &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
        CastMode::Detachable,
        Some(Path::new("out")),
    );
    let after_cast = std::fs::read_to_string(&design).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    assert_ne!(
        after_cast, SENTINEL,
        "a real cast did NOT rewrite the design file — then the assertion above \
         guards nothing and this gate is vacuous"
    );
}

/// Sorted file names in `dir`, so "left something behind" is checkable.
fn listing(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().to_string())
        .collect();
    names.sort();
    names
}

/// Rewrite `prep`'s centerline down to `n` points, keeping the file valid TOML.
fn truncate_centerline(prep: &Path, n: usize) {
    let text = std::fs::read_to_string(prep).unwrap();
    let head = text
        .find("[centerline]")
        .expect("prep has a [centerline] section");
    let open = head
        + text[head..]
            .find("points_m = [")
            .expect("centerline has points");
    let start = open + "points_m = [".len();
    let mut depth = 1usize;
    let mut end = start;
    for (i, c) in text[start..].char_indices() {
        match c {
            '[' => depth += 1,
            ']' => {
                depth -= 1;
                if depth == 0 {
                    end = start + i;
                    break;
                }
            }
            _ => {}
        }
    }
    assert_eq!(
        depth,
        0,
        "unbalanced centerline array in {}",
        prep.display()
    );
    // Distinct points: `Ribbon::new` rejects a zero-length segment, so two
    // copies of the same point would fail for a different reason than the one
    // under test.
    let pts: String = (0..n)
        .map(|i| format!("[0.0, 0.0, {:.3}],\n", i as f64 * 0.01))
        .collect();
    std::fs::write(prep, format!("{}{}{}", &text[..start], pts, &text[end..])).unwrap();
}

/// ★ A prep the pre-flight cannot use is an ERROR, not a verdict.
///
/// The distinction is the whole reason [`PlugFit`] and [`EngineError`] are
/// separate types: if a missing centerline came back as "will not cast", the
/// operator would be told to reduce an inset that was never the problem.
///
/// ⚠ Both sides of the boundary, because one side does not pin it. Asserting
/// only that 1 point is refused leaves "refuse 2 as well" — which rejects a
/// centerline the cast accepts — passing just as happily.
#[test]
fn an_unusable_centerline_is_an_error_not_a_refusal() {
    let dir = fixture("centerline");
    let (cleaned, prep) = paths(&dir);

    truncate_centerline(&prep, 1);
    let one = plug_fit_preflight(
        &cleaned,
        &prep,
        0.005,
        &RidgeOptions::default(),
        CELL_SIZE_M,
    );
    assert!(
        matches!(one, Err(EngineError::NoCenterline { .. })),
        "a 1-point centerline must be reported as a missing centerline, got {one:?}"
    );

    truncate_centerline(&prep, 2);
    let two = plug_fit_preflight(
        &cleaned,
        &prep,
        0.005,
        &RidgeOptions::default(),
        CELL_SIZE_M,
    );
    let _ = std::fs::remove_dir_all(&dir);
    assert!(
        !matches!(two, Err(EngineError::NoCenterline { .. })),
        "2 points is the minimum `Ribbon::new` accepts — rejecting it turns a \
         castable prep away, got {two:?}"
    );
}

/// ★ The pre-flight must never refuse to RUN on a configuration the cast
/// honours. Its invented layer is the thing that could make it.
///
/// The derivation gates the canal's suction bulge — which the GUI's tip-relief
/// control maps straight onto — against BOTH the mold cup wall and the inner
/// layer's thickness. The cup wall is a shared cast default, so failing it is a
/// real failure the cast shares. The inner layer is INVENTED here, and failing
/// that one would report a suction bulb blowing out a 6 mm shell the operator
/// never configured.
///
/// ⚠ Which gate trips first is not something to reason about — it is a
/// relationship between two constants in different crates. So this asks the
/// cast, at a bulge far past anything the GUI can produce.
#[test]
fn a_ridge_setting_the_preflight_refuses_is_one_the_cast_refuses() {
    let dir = fixture("deep-relief");
    let (cleaned, prep) = paths(&dir);
    let ridges = RidgeOptions {
        enabled: true,
        // 20 mm — four times the GUI's 5 mm ceiling, and past the probe layer,
        // so if the invented layer can ever be the thing that fails, it fails
        // here.
        tip_relief_depth_m: 0.020,
        ..RidgeOptions::default()
    };
    let preflight = plug_fit_preflight(&cleaned, &prep, 0.005, &ridges, CELL_SIZE_M);
    // The oracle: the same cast, same ridges, same one-layer stack.
    let draft = DesignDraft {
        cavity_inset_m: 0.005,
        layers: vec![LayerDraft {
            thickness_m: 0.006,
            material_key: "ECOFLEX_00_30".to_string(),
            slacker_fraction: 0.25,
        }],
    };
    let cast = generate_molds_for_design(
        &cleaned,
        &prep,
        &draft,
        CELL_SIZE_M,
        &ridges,
        &PartSelection::from_ids([PartId::Plug { layer_index: 0 }]),
        CastMode::Detachable,
        Some(Path::new("out")),
    );
    let _ = std::fs::remove_dir_all(&dir);
    assert!(
        !(preflight.is_err() && cast.is_ok()),
        "the pre-flight refused to run on ridges the cast honours — its invented \
         layer is deciding the answer: {preflight:?}"
    );
}
