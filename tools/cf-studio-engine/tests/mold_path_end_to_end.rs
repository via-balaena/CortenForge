//! The engine's mold path, end to end, on a synthetic scan — the wizard's
//! front door (`generate_molds_for_design`) driven exactly as a frontend
//! drives it, with no out-of-repo asset.
//!
//! ★★ WHY SYNTHETIC. The mold path's `~/scans/base_mold` gates are `#[ignore]`d
//! at 278 s (detachable) / 408 s (bonded) — both measured 2026-09-06, same
//! machine and cell size — up to ~61 min for the full end-to-end. A gate behind
//! an out-of-repo fixture that also costs minutes is one nobody runs: that set
//! sat unrun from ~June to 2026-08-27. A small tube at 3 mm cells reaches the
//! SAME entry point in seconds, in CI.
//!
//! ★★★ THE SPLIT, and it is deliberate. The `~/scans` gates keep everything
//! they own about GEOMETRY — whether a real 9 MB scan with noisy normals and
//! near-degenerate triangles survives the mesher at print resolution, and the
//! physically-validated config. This file owns the DECISIONS: which export path
//! a cast takes, which instructions it is handed, which pieces it emits, and
//! whether an opt-in reaches the pipeline at all. Those are argument wires, and
//! an argument wire is exactly what a slow gate nobody runs cannot protect.

// `panic!` with context is the integration-test idiom here (same convention as
// `design/cf-cast/tests/*.rs`): the crate denies it for library code, where
// errors are values, but a test failure has to be readable.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::{Path, PathBuf};

use cf_studio_core::{DesignDraft, LayerDraft, MoldOutputs, RidgeOptions};
use cf_studio_engine::{CastMode, EditSession, PartId, PartSelection, generate_molds_for_design};
use cortenforge::mesh::types::{IndexedMesh, Point3};

/// Cendrillon's own layer count, and load-bearing for the cast-mode gates. At
/// one layer the two modes select the SAME parts (one plug either way), so the
/// plug-count assertions would coincide — and detachable's heading collapses to
/// `## Post-Cure`, not the `## Post-Cure Assembly + Disassembly` asserted below.
/// The heading oracle alone would still discriminate; the rest would not.
const LAYERS: usize = 3;

/// Tall enough that the three layers' cups are unambiguous pieces; every gate
/// that does not need the length overrides it, because the cast's cost scales
/// with the meshed volume.
const TUBE_H_M: f64 = 0.100;

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

/// The wizard's three-layer draft thinned so the cast stays cheap, truncated to
/// `layers` — the outermost layers drop first, so a 1-layer draft is the
/// ECOFLEX base every longer draft also starts from.
fn draft(layers: usize) -> DesignDraft {
    let mut draft = DesignDraft {
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
    };
    draft.layers.truncate(layers);
    draft
}

/// Prep a synthetic scan through [`EditSession`] — the same cleaned STL +
/// `.prep.toml` pair the wizard hands the cast — then cast it.
///
/// Returns the outputs and the rendered `procedure.md`. Unlike the `~/scans`
/// gates, which keep their directory on failure so it can be inspected, this
/// one always removes it: everything the assertions read is already in the
/// returned values, and [`headings`] puts the deciding evidence in the failure
/// message rather than on disk.
fn cast_with(
    label: &str,
    tube_height_m: f64,
    layers: usize,
    selection: &PartSelection,
    mode: CastMode,
    ridges: &RidgeOptions,
) -> CastRun {
    let dir = std::env::temp_dir().join(format!("cf-studio-engine-{label}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    let mut session = EditSession::from_mesh(
        PathBuf::from("synthetic.stl"),
        open_cylinder(0.020, tube_height_m, 24, 13),
    );
    let scan = session.detect_caps();
    assert_eq!(scan.loop_count, 2, "an open tube has two boundary loops");
    session
        .save(&dir, "synthetic", "mm", 0)
        .expect("prep saves");

    let out = generate_molds_for_design(
        &dir.join("synthetic.cleaned.stl"),
        &dir.join("synthetic.prep.toml"),
        &draft(layers),
        0.003,
        ridges,
        selection,
        mode,
        Some(Path::new("out")),
    )
    .expect("the synthetic scan casts");

    // ⚠ Every artifact an assertion needs is read HERE, while the fixture still
    // exists. The first draft of the ridges gate stat()ed a plug STL after this
    // function returned and failed on a path that had just been deleted — a
    // returned `PathBuf` outlives the file it names.
    let procedure = std::fs::read_to_string(&out.procedure_path).expect("procedure.md is written");
    let plug_triangles = out.plug_stls.iter().map(|p| stl_triangles(p)).collect();
    let _ = std::fs::remove_dir_all(&dir);
    CastRun {
        out,
        procedure,
        plug_triangles,
    }
}

/// One cast's results, detached from the fixture directory that produced them.
struct CastRun {
    out: MoldOutputs,
    procedure: String,
    /// Triangles in each emitted plug STL, in `out.plug_stls` order.
    plug_triangles: Vec<u64>,
}

/// A whole cast in `mode`, with the selection that mode implies — what the app
/// sends when every offered part is checked.
fn cast(mode: CastMode, label: &str) -> CastRun {
    cast_with(
        label,
        TUBE_H_M,
        LAYERS,
        &mode.part_selection(LAYERS),
        mode,
        &RidgeOptions::default(),
    )
}

/// The sheet's `##` headings — the failure evidence, since the fixture is gone
/// by the time an assertion runs.
fn headings(procedure: &str) -> Vec<&str> {
    procedure.lines().filter(|l| l.starts_with("## ")).collect()
}

/// Triangles in a binary STL: an 84-byte header then 50 bytes per facet. Read
/// as a COUNT rather than a file size because the count is the quantity the
/// claim is about — a canal adds geometry, it does not add bytes.
fn stl_triangles(path: &Path) -> u64 {
    let len = std::fs::metadata(path)
        .unwrap_or_else(|e| panic!("stat {}: {e}", path.display()))
        .len();
    assert!(
        len >= 84 && (len - 84).is_multiple_of(50),
        "{} is {len} bytes — not a binary STL",
        path.display()
    );
    (len - 84) / 50
}

// ── which instructions a cast is handed ─────────────────────────────────────
//
// `cast_mode` reaches exactly two places: the routing predicate below, and
// `write_procedure_v2_for_mode`. Everything the `~/scans` bonded gate asserts is
// blind to it — plug and cup counts come from `selection`, which the caller
// computes SEPARATELY, and the pour steps from `draft.layers`. So dropping
// `cast_mode` on the floor (or calling `write_procedure_v2`, whose signature
// hard-codes `Detachable`) leaves every one of those assertions green while a
// bonded operator is handed detachable instructions: demold-and-nest steps for a
// process in which layer N is never demolded. Nothing anywhere read a generated
// `procedure.md`'s mode before these.

/// The row Cendrillon ships (`CENDRILLON_CAST_MODE`).
///
/// ⚠ The heading is the oracle, not the plug count: bonded drops the plugs
/// above layer 0 through its `PartSelection`, so `plug_stls.len() == 1` holds
/// even if `cast_mode` never reaches the procedure writer.
#[test]
fn a_bonded_cast_gets_bonded_instructions() {
    let run = cast(CastMode::Bonded, "bonded");
    let (out, procedure) = (&run.out, &run.procedure);

    assert!(
        procedure.contains("## Finishing"),
        "bonded never nests, so the sheet finishes rather than demolds: {:?}",
        headings(procedure)
    );
    assert!(
        !procedure.contains("## Post-Cure"),
        "and must not carry the detachable post-cure assembly steps: {:?}",
        headings(procedure)
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
    let run = cast(CastMode::Detachable, "detachable");
    let (out, procedure) = (&run.out, &run.procedure);

    assert!(
        procedure.contains("## Post-Cure Assembly + Disassembly"),
        "a multi-layer detachable cast nests and comes apart: {:?}",
        headings(procedure)
    );
    assert!(
        !procedure.contains("## Finishing"),
        "and must not carry the bonded finishing steps: {:?}",
        headings(procedure)
    );

    assert_eq!(out.mold_stls.len(), 2 * LAYERS, "2 cup halves per layer");
    assert_eq!(
        out.plug_stls.len(),
        LAYERS,
        "detachable casts a plug per layer"
    );
    assert_eq!(out.pour_plan.steps.len(), LAYERS, "a pour per layer");
}

// ── which pieces a cast emits ───────────────────────────────────────────────

/// Selective export: ask for one plug, get one plug.
///
/// The feature's whole point — re-print a single piece without repeating the
/// cast — and until now it was owned only by an `#[ignore]`d `~/scans` gate.
///
/// ⚠ The pour plan is asserted at the FULL layer count on purpose. Masses are
/// computed for every layer whether or not its geometry was meshed, so a
/// partial run still hands the operator complete instructions; narrowing the
/// plan to the selection would be a silent regression this pins.
#[test]
fn a_selective_cast_emits_only_the_selected_piece() {
    let selection = PartSelection::from_ids([PartId::Plug { layer_index: 0 }]);
    assert!(!selection.is_all(), "the fixture is a genuine subset");

    let out = cast_with(
        "selective",
        TUBE_H_M,
        LAYERS,
        &selection,
        CastMode::Detachable,
        &RidgeOptions::default(),
    )
    .out;

    assert!(
        out.mold_stls.is_empty(),
        "no cup halves were selected: {:?}",
        out.mold_stls
    );
    assert!(
        out.accessory_stls.is_empty(),
        "no platform or dowels either: {:?}",
        out.accessory_stls
    );
    assert_eq!(out.plug_stls.len(), 1, "exactly the one plug asked for");
    assert!(
        out.plug_stls[0].ends_with("plug_layer_0.stl"),
        "and it is layer 0's: {}",
        out.plug_stls[0].display()
    );
    assert_eq!(
        out.pour_plan.steps.len(),
        LAYERS,
        "the plan still spans every layer — instructions, not files"
    );
}

// ── whether an opt-in reaches the pipeline ──────────────────────────────────

/// Interior ridges ON must actually reach the cast.
///
/// ★★ The `RidgeOptions -> CanalConfig` MAPPING has unit tests; what had none is
/// the wire — `canal_config_from_ridges(ridges)` being handed to
/// `CastConfig::for_design`. Break that one argument and the GUI's ridge toggle
/// silently does nothing: the operator prints smooth plugs, pours, and finds out
/// at demold that the grip rings are missing. That is a wasted physical cast,
/// and it is the same argument-wire class as `cast_mode` above.
///
/// ⚠ Cast twice, because a count alone proves nothing — the canal's effect is
/// only visible against the same tube with the toggle off. Measured 2026-09-07:
/// **1356 -> 53510 triangles (39x)**; the gate demands 10x, so it has ~4x of
/// margin against ordinary meshing drift.
///
/// ⚠ One layer, a 30 mm tube, and only layer 0's plug selected — the canal plug
/// meshes at 0.5 mm regardless of the cup cell size, so this is the geometry
/// that costs. Full-length three-layer at the same settings measures 31 s
/// against this configuration's 10 s, and gates the identical wire.
#[test]
fn ridges_enabled_reaches_the_cast() {
    let selection = PartSelection::from_ids([PartId::Plug { layer_index: 0 }]);
    let ridged = RidgeOptions {
        enabled: true,
        ..RidgeOptions::default()
    };
    assert!(
        !RidgeOptions::default().enabled && ridged.enabled,
        "the two configs differ only in the toggle this gate is about"
    );

    let smooth = cast_with(
        "ridges-off",
        0.030,
        1,
        &selection,
        CastMode::Detachable,
        &RidgeOptions::default(),
    );
    let ridgy = cast_with(
        "ridges-on",
        0.030,
        1,
        &selection,
        CastMode::Detachable,
        &ridged,
    );

    let (smooth_tris, ridgy_tris) = (smooth.plug_triangles[0], ridgy.plug_triangles[0]);
    assert!(
        ridgy_tris > 10 * smooth_tris,
        "ridges on must add ring geometry to the plug, but it went \
         {smooth_tris} -> {ridgy_tris} triangles"
    );
}
