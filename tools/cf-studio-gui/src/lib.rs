//! `cf-studio-gui` — the polished Slint GUI for CortenForge Studio.
//!
//! A thin client over [`cf_studio_engine`] + the [`cf_studio_core::Project`]
//! state machine — the same boundary the `cf-studio` CLI drives, so the
//! GUI and CLI behave identically.
//!
//! This lib holds the **headless, testable** part:
//! - [`step_rows`] maps a [`Project`] + the previewed step to the
//!   checklist rows the Slint markup renders;
//! - [`apply_scan`] / [`apply_prep`] / [`apply_design`] run a step's
//!   action against the engine and return a user-facing message (the
//!   GUI's analog of the CLI's `cmd_*`, but file-dialog picking lives in
//!   `main.rs`);
//! - [`nav_state`] computes the gated Back/Next availability.
//!
//! The Slint event-loop + `rfd` file-dialog glue lives in `main.rs` (it
//! needs a display to *run*, but compiles headlessly).

use std::fmt::Write as _;
use std::path::Path;

use cf_studio_core::{DesignDraft, MoldOutputs, PourPlan, PourStep, Project, Step};
use cf_studio_engine::{accept_prep, draft_from_design_toml, load_scan};

pub mod viewer;

/// A workflow step as the checklist shows it. `done` / `current` come
/// from the real project; `viewing` is whether this is the step shown in
/// the wizard body right now.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepRow {
    /// One-based step number.
    pub number: i32,
    /// Plain-language step title.
    pub title: String,
    /// Whether the project has completed this step.
    pub done: bool,
    /// Whether this is the project's current (furthest) step.
    pub current: bool,
    /// Whether this is the step currently shown in the wizard body.
    pub viewing: bool,
}

/// Build the six checklist rows for `project`, marking `viewed` as the
/// step shown in the body.
#[must_use]
pub fn step_rows(project: &Project, viewed: Step) -> Vec<StepRow> {
    Step::ALL
        .iter()
        .map(|&step| StepRow {
            number: i32::try_from(step.number()).unwrap_or(0),
            title: step.title().to_string(),
            done: project.is_complete(step),
            current: step == project.current_step(),
            viewing: step == viewed,
        })
        .collect()
}

/// Outcome of a step action: a user-facing message. `Ok` is a success
/// line (starts with "✓"); `Err` is the failure message to surface. The
/// frontend decides how to color/show it.
pub type StepOutcome = Result<String, String>;

/// Step 1 action — validate the scan loads + has geometry, then record it.
///
/// # Errors
/// The failure message if the scan is missing, unreadable, or empty.
pub fn apply_scan(project: &mut Project, scan_file: &Path) -> StepOutcome {
    let loaded = load_scan(scan_file).map_err(|e| e.to_string())?;
    let message = format!(
        "✓ Added scan: {} ({} vertices, {} faces)",
        scan_file.display(),
        loaded.vertex_count,
        loaded.face_count
    );
    project.set_scan(loaded.artifact());
    Ok(message)
}

/// Step 2 action — accept a cleaned scan + its `.prep.toml`.
///
/// # Errors
/// The failure message if the prep is invalid or the scan step isn't done.
pub fn apply_prep(project: &mut Project, cleaned_stl: &Path, prep_toml: &Path) -> StepOutcome {
    let prep = accept_prep(cleaned_stl, prep_toml).map_err(|e| e.to_string())?;
    project.set_prep(prep).map_err(|e| e.to_string())?;
    Ok("✓ Accepted cleaned scan + prep.".to_string())
}

/// Step 3 action — load a layer design from a `.design.toml`.
///
/// # Errors
/// The failure message if the design is invalid or the scan isn't cleaned.
pub fn apply_design(project: &mut Project, design_toml: &Path) -> StepOutcome {
    let draft = draft_from_design_toml(design_toml).map_err(|e| e.to_string())?;
    let message = format!(
        "✓ Design set: {} layer(s), {:.1} mm cavity inset.",
        draft.layers.len(),
        draft.cavity_inset_m * 1000.0
    );
    project.set_design(draft).map_err(|e| e.to_string())?;
    Ok(message)
}

/// Step 3 action — set a layer design built in-app (the layer-stack
/// editor), rather than loaded from a file.
///
/// # Errors
/// The failure message if the design is invalid or the scan isn't cleaned.
pub fn apply_design_draft(project: &mut Project, draft: DesignDraft) -> StepOutcome {
    let message = format!(
        "✓ Design set: {} layer(s), {:.1} mm cavity inset.",
        draft.layers.len(),
        draft.cavity_inset_m * 1000.0
    );
    project.set_design(draft).map_err(|e| e.to_string())?;
    Ok(message)
}

/// Marching-cubes cell size (meters) for the step-4 quality-picker index.
/// Index 0 = Fine 0.5 mm (the print-quality default — the physical fit-test
/// print was 0.5 mm); index 1 = Fast 1.5 mm preview. Any other index falls
/// back to the print-quality default. 3 mm is never offered (it drops the
/// flange web). **Must stay in lockstep with the picker's `model` order in
/// `app.slint`** — the test pins it.
#[must_use]
pub fn cell_size_m_for_quality(quality_idx: i32) -> f64 {
    match quality_idx {
        1 => 0.0015,
        _ => 0.0005,
    }
}

/// A human-readable summary of a completed mold run for the step-4 results
/// panel: piece counts, total silicone, the per-layer pour list, and where
/// the files landed.
#[must_use]
pub fn format_molds_summary(out: &MoldOutputs) -> String {
    let mut s = format!(
        "✓ {} mold piece(s) + {} plug(s)",
        out.mold_stls.len(),
        out.plug_stls.len(),
    );
    if !out.accessory_stls.is_empty() {
        let _ = write!(s, " + {} accessory part(s)", out.accessory_stls.len());
    }
    let _ = write!(
        s,
        "\nTotal silicone: {:.0} g across {} pour(s):",
        out.total_mass_g,
        out.pour_plan.steps.len(),
    );
    for step in &out.pour_plan.steps {
        let _ = write!(
            s,
            "\n  • Layer {}: {} — {:.0} g (pot life ~{} min)",
            step.layer_index + 1,
            step.material_display_name,
            step.mass_g,
            step.pot_life_minutes,
        );
    }
    let _ = write!(s, "\nSaved to: {}", out.out_dir.display());
    s
}

/// The step-5 (Print) status line, derived from project state: once the
/// files are exported, where they went; before that, how many printables
/// are waiting to be saved; nothing if the molds aren't made yet.
#[must_use]
pub fn print_step_summary(project: &Project) -> String {
    if let Some(export) = project.print() {
        return format!(
            "✓ Saved to {} — open that folder in your slicer to print each piece.",
            export.export_dir.display()
        );
    }
    if let Some(molds) = project.molds() {
        let pieces = molds.mold_stls.len() + molds.plug_stls.len() + molds.accessory_stls.len();
        return format!("Ready to save {pieces} printable file(s) + the step-by-step guide.");
    }
    String::new()
}

// ── step 6: the pour assistant ──────────────────────────────────────

/// One pour layer's full recipe line, e.g.
/// `"Ecoflex 00-30 — 500 g, mix 1:1, 25% Slacker · pot life ~25 min · cure ~4 h"`.
/// Slacker is shown only when present; the cure is omitted for the last
/// layer (nothing waits on it).
#[must_use]
fn pour_recipe_line(step: &PourStep, is_last: bool) -> String {
    let slacker = step
        .slacker_fraction
        .map(|f| format!(", {:.0}% Slacker", f * 100.0))
        .unwrap_or_default();
    let cure = if is_last {
        String::new()
    } else {
        format!(" · cure ~{:.0} h", step.cure_time_hours)
    };
    format!(
        "{} — {:.0} g, mix {}{slacker} · pot life ~{} min{cure}",
        step.material_display_name, step.mass_g, step.mix_ratio_a_to_b, step.pot_life_minutes,
    )
}

/// The full pour plan as a numbered overview (innermost layer first), for
/// the step-6 reference panel. Empty plan → a short placeholder.
#[must_use]
pub fn format_pour_plan(plan: &PourPlan) -> String {
    if plan.steps.is_empty() {
        return "(no pour layers)".to_string();
    }
    let last = plan.steps.len() - 1;
    let mut s = format!(
        "Pour plan — {} layer(s), innermost first:",
        plan.steps.len()
    );
    for (i, step) in plan.steps.iter().enumerate() {
        let _ = write!(s, "\n  {}. {}", i + 1, pour_recipe_line(step, i == last));
    }
    s
}

/// The active-layer instruction for the step-6 pour panel: which layer of
/// how many, its recipe, and what to do. `current` is 0-based.
#[must_use]
pub fn format_pour_active(plan: &PourPlan, current: usize) -> String {
    let Some(step) = plan.steps.get(current) else {
        return String::new();
    };
    let is_last = current + 1 == plan.steps.len();
    format!(
        "Layer {} of {} — {}\nMix it, start the timer, and pour before the working time runs out.",
        current + 1,
        plan.steps.len(),
        pour_recipe_line(step, is_last),
    )
}

/// A pot-life countdown's display text + urgency for the step-6 timer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PourCountdown {
    /// `"M:SS left"`, or a "time's up" line once expired.
    pub text: String,
    /// 0 = comfortable, 1 = warning (≤ 5 min left), 2 = expired (≤ 0).
    pub urgency: i32,
}

/// Format a pot-life countdown from the seconds remaining (negative or zero
/// = expired). Warns under five minutes.
#[must_use]
pub fn pour_countdown(remaining_secs: i64) -> PourCountdown {
    if remaining_secs <= 0 {
        return PourCountdown {
            text: "0:00 — working time's up. Pour now, or scrape and remix.".to_string(),
            urgency: 2,
        };
    }
    let text = format!(
        "⏱ {}:{:02} of working time left",
        remaining_secs / 60,
        remaining_secs % 60
    );
    PourCountdown {
        text,
        urgency: i32::from(remaining_secs <= 300),
    }
}

/// Whether Back/Next are available from the `viewed` screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavState {
    /// Back is available on any screen but the first.
    pub can_back: bool,
    /// Next is available once the viewed step is complete (and it isn't
    /// the last) — the wizard gate that stops you skipping ahead.
    pub can_next: bool,
}

/// Compute the gated navigation state for the `viewed` screen.
#[must_use]
pub fn nav_state(project: &Project, viewed: Step) -> NavState {
    NavState {
        can_back: viewed != Step::FIRST,
        can_next: project.is_complete(viewed) && viewed != Step::LAST,
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use super::*;

    const ONE_TRIANGLE_STL: &str = "\
solid t
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid t
";

    const PREP_WITH_CENTERLINE: &str = "\
[centerline]
points_m = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.01]]
";

    const DESIGN_TOML: &str = "\
[device_design]
tool_version = \"x\"
generated_at = \"2026-01-01T00:00:00Z\"
schema_version = 1
[scan_ref]
cleaned_stl = \"c.stl\"
[cavity]
inset_m = 0.005
visible = true
[[layers]]
thickness_m = 0.005
material_anchor_key = \"ECOFLEX_00_30\"
slacker_fraction = 0.0
visible = true
";

    fn dir(label: &str) -> PathBuf {
        let d =
            std::env::temp_dir().join(format!("cf-studio-gui-test-{}-{label}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn fresh_project_has_six_rows_all_undone_at_step_one() {
        let p = Project::new("t");
        let rows = step_rows(&p, Step::AddScan);
        assert_eq!(rows.len(), 6);
        assert!(
            rows.iter().all(|r| !r.done),
            "nothing done on a fresh project"
        );
        assert!(rows[0].current, "current step is AddScan");
        assert!(!rows[1].current);
        for (i, r) in rows.iter().enumerate() {
            assert_eq!(r.number, i32::try_from(i + 1).unwrap());
            assert!(!r.title.is_empty());
        }
    }

    #[test]
    fn viewing_marks_the_previewed_step_independent_of_current() {
        let p = Project::new("t"); // still at AddScan
        let rows = step_rows(&p, Step::MakeMolds);
        assert!(rows[3].viewing, "MakeMolds (index 3) is being viewed");
        assert!(!rows[0].viewing);
        assert!(rows[0].current, "but the project is still on AddScan");
        assert!(!rows[3].current);
    }

    #[test]
    fn apply_scan_records_and_returns_a_message() {
        let d = dir("scan");
        let stl = d.join("s.stl");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();

        let mut p = Project::new("t");
        let msg = apply_scan(&mut p, &stl).unwrap();
        assert!(msg.contains("Added scan"), "got: {msg}");
        assert!(p.is_complete(Step::AddScan));

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn apply_scan_missing_file_is_an_error_message() {
        let mut p = Project::new("t");
        assert!(apply_scan(&mut p, Path::new("/no/such/scan.stl")).is_err());
    }

    #[test]
    fn apply_prep_then_design_completes_steps_2_and_3() {
        let d = dir("flow");
        let stl = d.join("s.stl");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        let design = d.join("x.design.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();
        std::fs::write(&design, DESIGN_TOML).unwrap();

        let mut p = Project::new("t");
        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();
        assert!(p.is_complete(Step::CleanScan));
        let msg = apply_design(&mut p, &design).unwrap();
        assert!(msg.contains("Design set"), "got: {msg}");
        assert!(p.is_complete(Step::DesignLayers));

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn apply_design_draft_completes_step_3() {
        use cf_studio_core::LayerDraft;

        let d = dir("draftdesign");
        let stl = d.join("s.stl");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();

        let mut p = Project::new("t");
        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();

        // A design built in-app (the layer-stack editor's output).
        let draft = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![LayerDraft {
                thickness_m: 0.0175,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            }],
        };
        let msg = apply_design_draft(&mut p, draft).unwrap();
        assert!(msg.contains("Design set"), "got: {msg}");
        assert!(p.is_complete(Step::DesignLayers));

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn apply_prep_before_scan_is_rejected() {
        let d = dir("order");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();

        let mut p = Project::new("t");
        assert!(apply_prep(&mut p, &cleaned, &prep).is_err());

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn print_step_summary_reflects_project_state() {
        use cf_studio_core::{MoldOutputs, PourPlan, PrintExport};

        let d = dir("printsummary");
        let stl = d.join("s.stl");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        let design = d.join("x.design.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();
        std::fs::write(&design, DESIGN_TOML).unwrap();

        let mut p = Project::new("t");
        // Fresh / pre-molds: nothing to show.
        assert_eq!(print_step_summary(&p), "");

        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();
        apply_design(&mut p, &design).unwrap();

        // Molds made, not yet exported → "ready to save N".
        p.set_molds(MoldOutputs {
            out_dir: PathBuf::from("/tmp/out"),
            mold_stls: vec![PathBuf::from("a.stl"), PathBuf::from("b.stl")],
            plug_stls: vec![PathBuf::from("p.stl")],
            accessory_stls: vec![PathBuf::from("plat.stl")],
            procedure_path: PathBuf::from("proc.md"),
            total_mass_g: 100.0,
            pour_plan: PourPlan { steps: vec![] },
        })
        .unwrap();
        let s = print_step_summary(&p);
        assert!(s.contains("Ready to save 4 printable"), "got: {s}");

        // Exported → "saved to <dir>" takes precedence.
        p.set_print(PrintExport {
            export_dir: PathBuf::from("/tmp/print-out"),
        })
        .unwrap();
        let s = print_step_summary(&p);
        assert!(s.starts_with("✓ Saved to /tmp/print-out"), "got: {s}");

        let _ = std::fs::remove_dir_all(&d);
    }

    fn sample_plan() -> PourPlan {
        use cf_studio_core::PourStep;
        PourPlan {
            steps: vec![
                PourStep {
                    layer_index: 0,
                    material_display_name: "Ecoflex 00-30".to_string(),
                    mass_g: 500.0,
                    mix_ratio_a_to_b: "1:1".to_string(),
                    pot_life_minutes: 25,
                    cure_time_hours: 4.0,
                    slacker_fraction: Some(0.25),
                },
                PourStep {
                    layer_index: 1,
                    material_display_name: "Dragon Skin 20A".to_string(),
                    mass_g: 250.0,
                    mix_ratio_a_to_b: "1:1".to_string(),
                    pot_life_minutes: 25,
                    cure_time_hours: 5.0,
                    slacker_fraction: None,
                },
            ],
        }
    }

    #[test]
    fn pour_plan_text_numbers_layers_and_drops_last_cure() {
        let s = format_pour_plan(&sample_plan());
        assert!(s.contains("2 layer(s)"), "got: {s}");
        assert!(
            s.contains(
                "1. Ecoflex 00-30 — 500 g, mix 1:1, 25% Slacker · pot life ~25 min · cure ~4 h"
            ),
            "got: {s}"
        );
        // Last layer: no cure clause, no slacker.
        assert!(s.contains("2. Dragon Skin 20A — 250 g, mix 1:1 · pot life ~25 min"));
        assert!(
            !s.contains("cure ~5 h"),
            "last layer's cure is dropped: {s}"
        );
        assert_eq!(
            format_pour_plan(&PourPlan { steps: vec![] }),
            "(no pour layers)"
        );
    }

    #[test]
    fn pour_active_targets_the_current_layer() {
        let plan = sample_plan();
        let a0 = format_pour_active(&plan, 0);
        assert!(a0.starts_with("Layer 1 of 2 — Ecoflex 00-30"), "got: {a0}");
        let a1 = format_pour_active(&plan, 1);
        assert!(
            a1.starts_with("Layer 2 of 2 — Dragon Skin 20A"),
            "got: {a1}"
        );
        // Past the end → empty (all poured).
        assert_eq!(format_pour_active(&plan, 2), "");
    }

    #[test]
    fn countdown_warns_then_expires() {
        let ok = pour_countdown(20 * 60); // 20 min
        assert_eq!(ok.urgency, 0);
        assert!(ok.text.contains("20:00"), "got: {}", ok.text);

        let warn = pour_countdown(4 * 60 + 30); // 4:30 — under 5 min
        assert_eq!(warn.urgency, 1);
        assert!(warn.text.contains("4:30"), "got: {}", warn.text);

        assert_eq!(
            pour_countdown(300).urgency,
            1,
            "5 min is the warn threshold"
        );
        assert_eq!(pour_countdown(301).urgency, 0);

        let dead = pour_countdown(0);
        assert_eq!(dead.urgency, 2);
        assert!(dead.text.contains("time's up"), "got: {}", dead.text);
        assert_eq!(pour_countdown(-10).urgency, 2, "negative = expired");
    }

    #[test]
    fn quality_index_maps_to_cell_size() {
        // Index 0 (the picker default) must be the 0.5 mm print quality;
        // this is the mapping that was wrong once already.
        assert_eq!(
            cell_size_m_for_quality(0),
            0.0005,
            "default = print quality"
        );
        assert_eq!(cell_size_m_for_quality(1), 0.0015, "fast preview");
        // Out-of-range indices fall back to the safe print-quality default.
        assert_eq!(cell_size_m_for_quality(99), 0.0005);
        assert_eq!(cell_size_m_for_quality(-1), 0.0005);
    }

    #[test]
    fn molds_summary_lists_counts_mass_and_pours() {
        use cf_studio_core::{PourPlan, PourStep};

        let out = MoldOutputs {
            out_dir: PathBuf::from("/tmp/scans/out"),
            mold_stls: vec![PathBuf::from("a.stl"), PathBuf::from("b.stl")],
            plug_stls: vec![PathBuf::from("p.stl")],
            accessory_stls: vec![PathBuf::from("platform.stl")],
            procedure_path: PathBuf::from("procedure.md"),
            total_mass_g: 842.0,
            pour_plan: PourPlan {
                steps: vec![PourStep {
                    layer_index: 0,
                    material_display_name: "Ecoflex 00-30".to_string(),
                    mass_g: 500.0,
                    mix_ratio_a_to_b: "1:1".to_string(),
                    pot_life_minutes: 25,
                    cure_time_hours: 4.0,
                    slacker_fraction: Some(0.25),
                }],
            },
        };
        let s = format_molds_summary(&out);
        assert!(s.contains("2 mold piece(s) + 1 plug(s)"), "got: {s}");
        assert!(s.contains("1 accessory part(s)"), "got: {s}");
        assert!(s.contains("842 g across 1 pour(s)"), "got: {s}");
        // 1-based layer label, display name, grams, pot life.
        assert!(
            s.contains("Layer 1: Ecoflex 00-30 — 500 g (pot life ~25 min)"),
            "got: {s}"
        );
        assert!(s.contains("Saved to: /tmp/scans/out"), "got: {s}");
    }

    #[test]
    fn molds_summary_omits_accessories_when_none() {
        use cf_studio_core::PourPlan;

        let out = MoldOutputs {
            out_dir: PathBuf::from("/tmp/out"),
            mold_stls: vec![PathBuf::from("a.stl")],
            plug_stls: vec![],
            accessory_stls: vec![],
            procedure_path: PathBuf::from("p.md"),
            total_mass_g: 0.0,
            pour_plan: PourPlan { steps: vec![] },
        };
        let s = format_molds_summary(&out);
        assert!(!s.contains("accessory"), "no accessory clause: {s}");
        assert!(s.contains("0 pour(s)"), "got: {s}");
    }

    #[test]
    fn next_gate_opens_only_after_the_step_completes() {
        let d = dir("nav");
        let stl = d.join("s.stl");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();

        let mut p = Project::new("t");
        let before = nav_state(&p, Step::AddScan);
        assert!(!before.can_back, "no Back on the first screen");
        assert!(!before.can_next, "no Next until the scan is added");

        apply_scan(&mut p, &stl).unwrap();
        assert!(
            nav_state(&p, Step::AddScan).can_next,
            "Next opens once scan is done"
        );
        assert!(
            nav_state(&p, Step::CleanScan).can_back,
            "Back available off the first screen"
        );
        assert!(
            !nav_state(&p, Step::Pour).can_next,
            "no Next on the final screen"
        );

        let _ = std::fs::remove_dir_all(&d);
    }
}
