//! `cf-studio-gui` binary — the Slint event-loop + `rfd` file-dialog
//! glue. The Project → display mapping and the per-step actions (and
//! their tests) live in the lib; this file picks files and wires state
//! into the UI.

// Slint's generated code uses unwrap/expect/panic internally; confine the
// crate's restriction-lint denies so they don't fire on generated code,
// while our own code below stays held to them.
mod ui {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
    slint::include_modules!();
}

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use cf_studio_core::{Project, Step};
use cf_studio_gui::{StepOutcome, apply_design, apply_prep, apply_scan, nav_state, step_rows};
use slint::{ComponentHandle, ModelRc, SharedString, VecModel};
use ui::{AppWindow, StepRow};

fn main() -> Result<(), slint::PlatformError> {
    // In-memory project for the session (autosave/resume is a later
    // increment). Shared via RefCell so each step callback can mutate it.
    let project = Rc::new(RefCell::new(Project::new("Untitled")));
    // The screen being shown, 0-based into Step::ALL.
    let viewed = Rc::new(Cell::new(0usize));

    let ui = AppWindow::new()?;
    refresh(&ui, &project.borrow(), viewed.get());
    let weak = ui.as_weak();

    // ── navigation (gated by the wizard state) ──────────────────
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        ui.on_back(move || {
            viewed.set(viewed.get().saturating_sub(1));
            if let Some(ui) = weak.upgrade() {
                ui.set_step_message(SharedString::default());
                refresh(&ui, &project.borrow(), viewed.get());
            }
        });
    }
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        ui.on_next(move || {
            let v = viewed.get();
            let step = Step::ALL.get(v).copied().unwrap_or(Step::FIRST);
            // Respect the gate even if the disabled button somehow fires.
            if nav_state(&project.borrow(), step).can_next {
                viewed.set(v + 1);
            }
            if let Some(ui) = weak.upgrade() {
                ui.set_step_message(SharedString::default());
                refresh(&ui, &project.borrow(), viewed.get());
            }
        });
    }
    ui.on_help(|| {
        // Placeholder — a help panel / per-step guidance arrives later.
    });

    // ── step actions (pick a file, run it through the engine) ───
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        ui.on_pick_scan(move || {
            let Some(path) = rfd::FileDialog::new()
                .set_title("Choose your 3D scan")
                .add_filter("3D scan", &["stl", "obj", "ply"])
                .pick_file()
            else {
                return;
            };
            let outcome = apply_scan(&mut project.borrow_mut(), &path);
            update(&weak, &project, viewed.get(), &outcome);
        });
    }
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        ui.on_pick_prep(move || {
            let Some(cleaned) = rfd::FileDialog::new()
                .set_title("Choose the cleaned scan (STL)")
                .add_filter("STL", &["stl"])
                .pick_file()
            else {
                return;
            };
            let Some(prep) = rfd::FileDialog::new()
                .set_title("Choose the .prep.toml")
                .add_filter("prep", &["toml"])
                .pick_file()
            else {
                return;
            };
            let outcome = apply_prep(&mut project.borrow_mut(), &cleaned, &prep);
            update(&weak, &project, viewed.get(), &outcome);
        });
    }
    {
        let (project, viewed, weak) = (project.clone(), viewed.clone(), weak.clone());
        ui.on_pick_design(move || {
            let Some(path) = rfd::FileDialog::new()
                .set_title("Choose the design file")
                .add_filter("design", &["toml"])
                .pick_file()
            else {
                return;
            };
            let outcome = apply_design(&mut project.borrow_mut(), &path);
            update(&weak, &project, viewed.get(), &outcome);
        });
    }

    ui.run()
}

/// After a step action: surface its message + re-render from the project.
fn update(
    weak: &slint::Weak<AppWindow>,
    project: &Rc<RefCell<Project>>,
    viewed_idx: usize,
    outcome: &StepOutcome,
) {
    if let Some(ui) = weak.upgrade() {
        set_message(&ui, outcome);
        refresh(&ui, &project.borrow(), viewed_idx);
    }
}

/// Push the project state + the previewed step into the UI properties.
fn refresh(ui: &AppWindow, project: &Project, viewed_idx: usize) {
    let viewed_step = Step::ALL.get(viewed_idx).copied().unwrap_or(Step::FIRST);
    let rows: Vec<StepRow> = step_rows(project, viewed_step)
        .into_iter()
        .map(|r| StepRow {
            number: r.number,
            title: r.title.into(),
            mark: if r.done { "✓" } else { "○" }.into(),
            // "you are here" marks the screen being shown (the wizard
            // cursor); ✓/○ shows real completion.
            here: if r.viewing {
                SharedString::from("   ← you are here")
            } else {
                SharedString::default()
            },
            viewing: r.viewing,
        })
        .collect();

    let nav = nav_state(project, viewed_step);
    ui.set_project_name(project.name.clone().into());
    ui.set_steps(ModelRc::new(VecModel::from(rows)));
    ui.set_viewed_number(i32::try_from(viewed_step.number()).unwrap_or(0));
    ui.set_total_steps(i32::try_from(Step::TOTAL).unwrap_or(6));
    ui.set_viewed_title(viewed_step.title().into());
    ui.set_can_back(nav.can_back);
    ui.set_can_next(nav.can_next);
}

/// Set the step-message text + its error styling from a step outcome.
fn set_message(ui: &AppWindow, outcome: &StepOutcome) {
    let (text, is_error) = match outcome {
        Ok(msg) => (msg.clone(), false),
        Err(msg) => (msg.clone(), true),
    };
    ui.set_step_message(text.into());
    ui.set_step_message_is_error(is_error);
}
