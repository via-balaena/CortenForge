//! `cf-studio` — the headless CLI frontend for CortenForge Studio.
//!
//! A thin client over [`cf_studio_engine`] (the step executors) and the
//! [`cf_studio_core::Project`] state machine (the spine). Each command
//! loads the project file, runs **one** workflow step, and saves — so the
//! workflow resumes across invocations, just like the GUI wizard will.
//! All the logic lives here in the lib so it is testable; `main.rs` is a
//! thin `clap` dispatcher.
//!
//! This first slice covers project management + the two input steps that
//! need no extra input design: `new`, `status`, `scan`, `prep`. The
//! design / molds / pour commands follow.

use std::path::Path;

use anyhow::{Context, Result, bail};
use cf_studio_core::{Project, Step};
use cf_studio_engine::{accept_prep, load_scan};

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
    save(&project, project_path)?;
    println!("✓ Accepted cleaned scan + prep.");
    print_status(&project);
    Ok(())
}

// ── helpers ─────────────────────────────────────────────────────────

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
        // Reload between commands → exercises autosave/resume.
        assert!(Project::load(&proj).unwrap().is_complete(Step::AddScan));

        cmd_prep(&proj, &cleaned, &prep).unwrap();
        let after = Project::load(&proj).unwrap();
        assert!(after.is_complete(Step::AddScan));
        assert!(after.is_complete(Step::CleanScan));
        assert_eq!(after.current_step(), Step::CleanScan);

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
}
