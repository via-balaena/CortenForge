//! Both cast modes through [`generate_molds_for_design`], end to end, on a
//! synthetic scan — the routing decision at the engine's front door.
//!
//! ★★ WHY SYNTHETIC. The mold path's only end-to-end gates copy `~/scans/
//! base_mold` and are `#[ignore]`d at 278 s (detachable) / 408 s (bonded) —
//! both measured 2026-09-06, same machine and cell size. A gate behind an
//! out-of-repo fixture that also costs seven minutes is one nobody runs: THAT
//! set sat unrun from ~June to 2026-08-27. A 100 mm tube at 3 mm cells runs the
//! SAME entry point for both modes in **7 s wall** (3.7 + 6.7 s serial — the
//! sum is not the elapsed time), in CI, with no asset.
//!
//! ★★★ WHAT THIS CATCHES THAT THE BIG GATES DO NOT. `cast_mode` reaches
//! exactly two places: the full-vs-selective routing predicate, and
//! `write_procedure_v2_for_mode`. Everything the existing bonded gate asserts
//! is blind to it: plug and cup counts come from `selection`, which the caller
//! computes SEPARATELY, and the pour steps from `draft.layers`. So dropping
//! `cast_mode` on the floor (or calling
//! `write_procedure_v2`, whose signature hard-codes `Detachable`) leaves every
//! one of those assertions green while every bonded operator is handed
//! detachable instructions: demold-and-nest steps for a process in which layer
//! N is never demolded. Nothing anywhere read a generated `procedure.md`'s
//! mode. These do.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

use cf_studio_core::{DesignDraft, LayerDraft, MoldOutputs, RidgeOptions};
use cf_studio_engine::{CastMode, EditSession, generate_molds_for_design};
use cortenforge::mesh::types::{IndexedMesh, Point3};

/// Cendrillon's own layer count, and load-bearing. At one layer the two modes
/// select the SAME parts (one plug either way), so the plug-count assertions
/// below would coincide — and detachable's heading collapses to `## Post-Cure`,
/// not the `## Post-Cure Assembly + Disassembly` this asserts. The heading
/// oracle alone would still discriminate; the rest of the gate would not.
const LAYERS: usize = 3;

/// An open round tube — `r` radius, `h` tall, open at both ends so cap
/// detection finds the two loops the centerline is fitted between.
fn open_cylinder(r: f64, h: f64, segs: usize, rings: usize) -> IndexedMesh {
    let mut vertices = Vec::new();
    for i in 0..rings {
        let z = h * i as f64 / (rings - 1) as f64;
        for s in 0..segs {
            let a = std::f64::consts::TAU * s as f64 / segs as f64;
            vertices.push(Point3::new(r * a.cos(), r * a.sin(), z));
        }
    }
    let mut faces = Vec::new();
    for i in 0..rings - 1 {
        let b = (i * segs) as u32;
        let t = ((i + 1) * segs) as u32;
        for s in 0..segs {
            let s2 = ((s + 1) % segs) as u32;
            let s = s as u32;
            faces.push([b + s, b + s2, t + s2]);
            faces.push([b + s, t + s2, t + s]);
        }
    }
    IndexedMesh { vertices, faces }
}

/// The wizard's three-layer draft, thinned so the cast stays cheap.
fn draft() -> DesignDraft {
    DesignDraft {
        cavity_inset_m: 0.002,
        layers: vec![
            LayerDraft {
                thickness_m: 0.006,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            },
            LayerDraft {
                thickness_m: 0.004,
                material_key: "DRAGON_SKIN_10A".to_string(),
                slacker_fraction: 0.0,
            },
            LayerDraft {
                thickness_m: 0.003,
                material_key: "DRAGON_SKIN_20A".to_string(),
                slacker_fraction: 0.0,
            },
        ],
    }
}

/// Prep the synthetic scan through [`EditSession`] — the same cleaned STL +
/// `.prep.toml` pair the wizard hands the cast — then cast it in `mode` with
/// the selection that mode implies, exactly as the app does.
///
/// Returns the outputs and the rendered `procedure.md`. Unlike the `~/scans`
/// gates, which keep their directory on failure so it can be inspected, this
/// one always removes it: everything the assertions read is already in the
/// returned values, and [`headings`] puts the deciding evidence in the failure
/// message rather than on disk.
fn cast(mode: CastMode, label: &str) -> (MoldOutputs, String) {
    let dir = std::env::temp_dir().join(format!("cf-studio-engine-{label}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    let mut session = EditSession::from_mesh(
        PathBuf::from("synthetic.stl"),
        open_cylinder(0.020, 0.100, 24, 13),
    );
    let scan = session.detect_caps();
    assert_eq!(scan.loop_count, 2, "an open tube has two boundary loops");
    session
        .save(&dir, "synthetic", "mm", 0)
        .expect("prep saves");

    let draft = draft();
    assert_eq!(draft.layers.len(), LAYERS);
    let out = generate_molds_for_design(
        &dir.join("synthetic.cleaned.stl"),
        &dir.join("synthetic.prep.toml"),
        &draft,
        0.003,
        &RidgeOptions::default(),
        &mode.part_selection(LAYERS),
        mode,
        Some(Path::new("out")),
    )
    .expect("the synthetic scan casts");

    let procedure = std::fs::read_to_string(&out.procedure_path).expect("procedure.md is written");
    let _ = std::fs::remove_dir_all(&dir);
    (out, procedure)
}

/// The sheet's `##` headings — the failure evidence, since the fixture is gone
/// by the time an assertion runs.
fn headings(procedure: &str) -> Vec<&str> {
    procedure.lines().filter(|l| l.starts_with("## ")).collect()
}

/// The row Cendrillon ships (`CENDRILLON_CAST_MODE`).
///
/// ⚠ The heading is the oracle, not the plug count: bonded drops the plugs
/// above layer 0 through its `PartSelection`, so `plug_stls.len() == 1` holds
/// even if `cast_mode` never reaches the procedure writer.
#[test]
fn a_bonded_cast_gets_bonded_instructions() {
    let (out, procedure) = cast(CastMode::Bonded, "bonded");

    assert!(
        procedure.contains("## Finishing"),
        "bonded never nests, so the sheet finishes rather than demolds: {:?}",
        headings(&procedure)
    );
    assert!(
        !procedure.contains("## Post-Cure"),
        "and must not carry the detachable post-cure assembly steps: {:?}",
        headings(&procedure)
    );

    assert_eq!(out.mold_stls.len(), 2 * LAYERS, "2 cup halves per layer");
    assert_eq!(
        out.plug_stls.len(),
        1,
        "bonded casts ONE plug — cured layer N is the plug for N+1"
    );
    assert_eq!(out.pour_plan.steps.len(), LAYERS, "a pour per layer");
}

/// The SDK default, and the other half of the oracle — without it
/// `## Finishing` could be in every sheet and the test above would still pass.
#[test]
fn a_detachable_cast_gets_detachable_instructions() {
    let (out, procedure) = cast(CastMode::Detachable, "detachable");

    assert!(
        procedure.contains("## Post-Cure Assembly + Disassembly"),
        "a multi-layer detachable cast nests and comes apart: {:?}",
        headings(&procedure)
    );
    assert!(
        !procedure.contains("## Finishing"),
        "and must not carry the bonded finishing steps: {:?}",
        headings(&procedure)
    );

    assert_eq!(out.mold_stls.len(), 2 * LAYERS, "2 cup halves per layer");
    assert_eq!(
        out.plug_stls.len(),
        LAYERS,
        "detachable casts a plug per layer"
    );
    assert_eq!(out.pour_plan.steps.len(), LAYERS, "a pour per layer");
}
