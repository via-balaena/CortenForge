//! `cf-studio` — the headless CLI frontend for CortenForge Studio.
//!
//! A thin client over [`cf_studio_engine`] (the step executors) and the
//! [`cf_studio_core::Project`] state machine (the spine). Each command
//! loads the project file, runs **one** workflow step, and saves — so the
//! workflow resumes across invocations, just like the GUI wizard will.
//! All the logic lives here in the lib so it is testable; `main.rs` is a
//! thin `clap` dispatcher.
//!
//! Commands cover the full workflow: project management (`new`, `status`)
//! plus the six steps — `scan`, `prep`, `design`, `molds`, `print`, `pour`.

use std::path::Path;

use anyhow::{Context, Result, bail};
use cf_studio_core::{PourPlan, PourRecord, PrintExport, Project, Step};
use cf_studio_engine::{
    CastConfig, accept_prep, draft_from_design_toml, generate_molds, load_scan,
};

/// `new <name>` — create a fresh project and save it. Refuses to clobber
/// an existing project file.
///
/// # Errors
/// If a project already exists at `project_path`, or the save fails.
pub fn cmd_new(project_path: &Path, name: &str) -> Result<()> {
    if project_path.exists() {
        bail!(
            "a project already exists at {} — choose a different --project path, or remove it first",
            project_path.display()
        );
    }
    let project = Project::new(name);
    save(&project, project_path)?;
    println!("Created project \"{name}\" → {}", project_path.display());
    print_status(&project);
    Ok(())
}

/// `status` — load the project and print where it is in the workflow.
///
/// # Errors
/// If the project file can't be loaded.
pub fn cmd_status(project_path: &Path) -> Result<()> {
    let project = load(project_path)?;
    print_status(&project);
    Ok(())
}

/// `scan <file>` — step 1: validate the scan loads + has geometry, then
/// record it on the project.
///
/// # Errors
/// If the project can't be loaded, the scan is missing/empty, or the save fails.
pub fn cmd_scan(project_path: &Path, scan_file: &Path) -> Result<()> {
    let mut project = load(project_path)?;
    let loaded = load_scan(scan_file).context("validate scan")?;
    project.set_scan(loaded.artifact());
    advance_after_step(&mut project);
    save(&project, project_path)?;
    println!(
        "✓ Added scan: {} ({} vertices, {} faces)",
        scan_file.display(),
        loaded.vertex_count,
        loaded.face_count
    );
    print_status(&project);
    Ok(())
}

/// `prep <cleaned.stl> <prep.toml>` — step 2: accept an existing cleaned
/// scan + `.prep.toml` (validated against what the cast requires).
///
/// # Errors
/// If the project can't be loaded, the prep is invalid, the scan step
/// isn't done, or the save fails.
pub fn cmd_prep(project_path: &Path, cleaned_stl: &Path, prep_toml: &Path) -> Result<()> {
    let mut project = load(project_path)?;
    let prep = accept_prep(cleaned_stl, prep_toml).context("accept cleaned scan + prep")?;
    project
        .set_prep(prep)
        .context("record prep on the project (is the scan step done?)")?;
    advance_after_step(&mut project);
    save(&project, project_path)?;
    println!("✓ Accepted cleaned scan + prep.");
    print_status(&project);
    Ok(())
}

/// `design <design.toml>` — step 3: load a layer design (cavity inset +
/// the soft→firm stack) from a `.design.toml`.
///
/// # Errors
/// If the project can't be loaded, the design is invalid, the scan isn't
/// cleaned yet, or the save fails.
pub fn cmd_design(project_path: &Path, design_toml: &Path) -> Result<()> {
    let mut project = load(project_path)?;
    let draft = draft_from_design_toml(design_toml).context("load layer design")?;
    let layer_count = draft.layers.len();
    let inset_mm = draft.cavity_inset_m * 1000.0;
    project
        .set_design(draft)
        .context("record design (is the scan cleaned yet?)")?;
    advance_after_step(&mut project);
    save(&project, project_path)?;
    println!("✓ Design set: {layer_count} layer(s), {inset_mm:.1} mm cavity inset.");
    print_status(&project);
    Ok(())
}

/// `molds <cast.toml>` — step 4: generate the printable molds + plugs +
/// the structured pour plan. This runs the cast pipeline and can take
/// several minutes.
///
/// # Errors
/// If the project can't be loaded, no design is set, the cast config is
/// unreadable, the run fails, or the save fails.
pub fn cmd_molds(project_path: &Path, cast_toml: &Path) -> Result<()> {
    let mut project = load(project_path)?;
    let draft = project
        .design()
        .context("no design yet — run `cf-studio design <design.toml>` first")?
        .clone();
    let text = std::fs::read_to_string(cast_toml)
        .with_context(|| format!("read cast config {}", cast_toml.display()))?;
    let config = CastConfig::from_toml_str(&text)
        .with_context(|| format!("parse cast config {}", cast_toml.display()))?;
    let base_dir = cast_toml.parent().unwrap_or_else(|| Path::new("."));

    println!("Generating molds — this can take several minutes. You can step away.");
    let molds = generate_molds(config, &draft, base_dir, None).context("generate molds")?;
    let summary = format!(
        "✓ Molds ready: {} mold halves + {} plug(s) + {} accessory part(s); total silicone {:.0} g.",
        molds.mold_stls.len(),
        molds.plug_stls.len(),
        molds.accessory_stls.len(),
        molds.total_mass_g,
    );
    project.set_molds(molds).context("record molds")?;
    advance_after_step(&mut project);
    save(&project, project_path)?;
    println!("{summary}");
    print_status(&project);
    Ok(())
}

/// `print <export-dir>` — step 5: record the folder you saved the
/// printable files to (the molds already live in the project's output
/// directory; this notes where you exported them for your slicer).
///
/// # Errors
/// If the project can't be loaded, the molds aren't made yet, or the save fails.
pub fn cmd_print(project_path: &Path, export_dir: &Path) -> Result<()> {
    let mut project = load(project_path)?;
    project
        .set_print(PrintExport {
            export_dir: export_dir.to_path_buf(),
        })
        .context("record print export (are the molds made yet?)")?;
    advance_after_step(&mut project);
    save(&project, project_path)?;
    println!("✓ Recorded print export folder: {}", export_dir.display());
    print_status(&project);
    Ok(())
}

/// `pour [--complete]` — step 6: show the guided pour plan (per layer:
/// material, mass, mix ratio, working time, cure). With `--complete`,
/// records the pour as done and finishes the project.
///
/// # Errors
/// If the project can't be loaded, the molds aren't made yet (no pour
/// plan), or — with `--complete` — the files weren't exported for print.
pub fn cmd_pour(project_path: &Path, complete: bool) -> Result<()> {
    let mut project = load(project_path)?;
    let plan = project
        .molds()
        .context("make the molds first — the pour plan comes from the cast run")?
        .pour_plan
        .clone();
    print_pour_plan(&plan);
    if complete {
        project
            .set_pour(PourRecord {
                layers_poured: plan.steps.len(),
            })
            .context("record the pour as done (have you exported the files for print?)")?;
        save(&project, project_path)?;
        println!("\n🎉 All layers recorded as poured — project complete.");
        print_status(&project);
    }
    Ok(())
}

// ── helpers ─────────────────────────────────────────────────────────

fn print_pour_plan(plan: &PourPlan) {
    if plan.steps.is_empty() {
        println!("(no pour layers)");
        return;
    }
    println!(
        "\nPour plan ({} layer(s), innermost first):",
        plan.steps.len()
    );
    for step in &plan.steps {
        let slacker = step
            .slacker_fraction
            .map(|f| format!(", {:.0}% Slacker", f * 100.0))
            .unwrap_or_default();
        println!(
            "  Layer {}: {} — {:.0} g, mix {}{slacker}",
            step.layer_index + 1,
            step.material_display_name,
            step.mass_g,
            step.mix_ratio_a_to_b,
        );
        println!(
            "      working time {} min · cure {:.0} h",
            step.pot_life_minutes, step.cure_time_hours
        );
    }
}

/// Completing a step advances the wizard to the next one, so `status`
/// (and the GUI, which this CLI mirrors) shows the *next* step as
/// current. A no-op at the final step. The step just set is complete, so
/// the only thing `advance` can report here is "already at the end" —
/// which is exactly the no-op we want, hence the deliberate ignore.
fn advance_after_step(project: &mut Project) {
    let _ = project.advance();
}

fn load(project_path: &Path) -> Result<Project> {
    Project::load(project_path).with_context(|| {
        format!(
            "load project from {} — run `cf-studio new <name>` first?",
            project_path.display()
        )
    })
}

fn save(project: &Project, project_path: &Path) -> Result<()> {
    project
        .save(project_path)
        .with_context(|| format!("save project to {}", project_path.display()))
}

fn print_status(project: &Project) {
    println!("\nProject: {}", project.name);
    for step in Step::ALL {
        let mark = if project.is_complete(step) {
            '✓'
        } else {
            '○'
        };
        let here = if step == project.current_step() {
            "   ← you are here"
        } else {
            ""
        };
        println!(
            "  {mark} Step {} of {}: {}{here}",
            step.number(),
            Step::TOTAL,
            step.title()
        );
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use cf_studio_core::{DesignDraft, LayerDraft, MoldOutputs, PourStep, PrepInput, ScanInput};

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

    fn dir(label: &str) -> PathBuf {
        let d =
            std::env::temp_dir().join(format!("cf-studio-cli-test-{}-{label}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn new_creates_a_project_at_add_scan() {
        let d = dir("new");
        let proj = d.join("p.json");
        cmd_new(&proj, "Wrist brace").unwrap();

        let loaded = Project::load(&proj).unwrap();
        assert_eq!(loaded.name, "Wrist brace");
        assert_eq!(loaded.current_step(), Step::AddScan);
        assert!(!loaded.is_complete(Step::AddScan));

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn new_refuses_to_clobber_existing() {
        let d = dir("clobber");
        let proj = d.join("p.json");
        cmd_new(&proj, "first").unwrap();
        let err = cmd_new(&proj, "second").unwrap_err();
        assert!(err.to_string().contains("already exists"), "got: {err}");
        // The original is untouched.
        assert_eq!(Project::load(&proj).unwrap().name, "first");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn scan_then_prep_advances_and_persists() {
        let d = dir("flow");
        let proj = d.join("p.json");
        let stl = d.join("scan.stl");
        let cleaned = d.join("scan.cleaned.stl");
        let prep = d.join("scan.prep.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();

        cmd_new(&proj, "flow").unwrap();
        cmd_scan(&proj, &stl).unwrap();
        // Reload between commands → exercises autosave/resume; and
        // completing scan advanced the wizard to step 2.
        let mid = Project::load(&proj).unwrap();
        assert!(mid.is_complete(Step::AddScan));
        assert_eq!(mid.current_step(), Step::CleanScan);

        cmd_prep(&proj, &cleaned, &prep).unwrap();
        let after = Project::load(&proj).unwrap();
        assert!(after.is_complete(Step::AddScan));
        assert!(after.is_complete(Step::CleanScan));
        // Completing prep advanced to step 3.
        assert_eq!(after.current_step(), Step::DesignLayers);

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn prep_before_scan_is_rejected() {
        let d = dir("order");
        let proj = d.join("p.json");
        let cleaned = d.join("scan.cleaned.stl");
        let prep = d.join("scan.prep.toml");
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();

        cmd_new(&proj, "order").unwrap();
        let err = cmd_prep(&proj, &cleaned, &prep).unwrap_err();
        // The spine's gate fires (scan step not done).
        assert!(format!("{err:#}").contains("Add your scan"), "got: {err:#}");

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn status_on_missing_project_errors_clearly() {
        let d = dir("missing");
        let err = cmd_status(&d.join("nope.json")).unwrap_err();
        assert!(err.to_string().contains("load project"), "got: {err}");
        let _ = std::fs::remove_dir_all(&d);
    }

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

    /// Build a project file advanced to "molds done" via the spine
    /// setters with stand-in artifacts (no real cast run needed) — so the
    /// print/pour command tests don't trigger the ~13-min mold-gen.
    fn write_project_at_molds(path: &Path) {
        let mut p = Project::new("t");
        p.set_scan(ScanInput {
            source_path: "s.stl".into(),
        });
        p.set_prep(PrepInput {
            cleaned_stl: "c.stl".into(),
            prep_toml: "p.toml".into(),
        })
        .unwrap();
        p.set_design(DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![LayerDraft {
                thickness_m: 0.005,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.0,
            }],
        })
        .unwrap();
        p.set_molds(MoldOutputs {
            out_dir: "out".into(),
            mold_stls: vec![
                "out/stls/mold_layer_0_piece_0.stl".into(),
                "out/stls/mold_layer_0_piece_1.stl".into(),
            ],
            plug_stls: vec!["out/stls/plug_layer_0.stl".into()],
            accessory_stls: vec!["out/stls/platform.stl".into()],
            procedure_path: "out/procedure.md".into(),
            total_mass_g: 842.9,
            pour_plan: PourPlan {
                steps: vec![PourStep {
                    layer_index: 0,
                    material_display_name: "Ecoflex 00-30".to_string(),
                    mass_g: 369.0,
                    mix_ratio_a_to_b: "1:1".to_string(),
                    pot_life_minutes: 45,
                    cure_time_hours: 4.0,
                    slacker_fraction: Some(0.25),
                }],
            },
        })
        .unwrap();
        p.save(path).unwrap();
    }

    #[test]
    fn design_sets_the_layer_stack_and_advances() {
        let d = dir("design");
        let proj = d.join("p.json");
        let design_toml = d.join("x.design.toml");
        std::fs::write(&design_toml, DESIGN_TOML).unwrap();

        // Project at "prep done", so the design step's prerequisite holds.
        let mut p = Project::new("t");
        p.set_scan(ScanInput {
            source_path: "s.stl".into(),
        });
        p.set_prep(PrepInput {
            cleaned_stl: "c.stl".into(),
            prep_toml: "p.toml".into(),
        })
        .unwrap();
        p.save(&proj).unwrap();

        cmd_design(&proj, &design_toml).unwrap();
        let after = Project::load(&proj).unwrap();
        assert!(after.is_complete(Step::DesignLayers));
        assert_eq!(after.design().unwrap().layers.len(), 1);
        assert_eq!(after.current_step(), Step::MakeMolds);

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn print_records_export_dir_and_advances() {
        let d = dir("print");
        let proj = d.join("p.json");
        write_project_at_molds(&proj);

        cmd_print(&proj, Path::new("/tmp/exports")).unwrap();
        let after = Project::load(&proj).unwrap();
        assert!(after.is_complete(Step::Print));
        assert_eq!(after.current_step(), Step::Pour);

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn pour_displays_plan_without_completing() {
        let d = dir("pour-show");
        let proj = d.join("p.json");
        write_project_at_molds(&proj);

        cmd_pour(&proj, false).unwrap();
        assert!(!Project::load(&proj).unwrap().is_finished());

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn pour_complete_requires_print_then_finishes() {
        let d = dir("pour-done");
        let proj = d.join("p.json");
        write_project_at_molds(&proj);

        // Before exporting for print, --complete is refused by the gate.
        let err = cmd_pour(&proj, true).unwrap_err();
        assert!(
            format!("{err:#}").contains("3D print the molds"),
            "got: {err:#}"
        );

        cmd_print(&proj, Path::new("/tmp/exports")).unwrap();
        cmd_pour(&proj, true).unwrap();
        assert!(Project::load(&proj).unwrap().is_finished());

        let _ = std::fs::remove_dir_all(&d);
    }
}
