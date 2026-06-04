//! `cf-studio-gui` binary — the Slint event-loop glue. The Project →
//! display mapping (and its tests) live in the lib; the step executors
//! live in cf-studio-engine. This file only wires state into the UI and
//! UI callbacks back into navigation.

// Slint's generated code uses unwrap/expect/panic internally; confine the
// crate's restriction-lint denies so they don't fire on generated code,
// while our own code below stays held to them.
mod ui {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
    slint::include_modules!();
}

use std::cell::Cell;
use std::rc::Rc;

use cf_studio_core::{Project, Step};
use cf_studio_gui::step_rows;
use slint::{ComponentHandle, ModelRc, SharedString, VecModel};
use ui::{AppWindow, StepRow};

fn main() -> Result<(), slint::PlatformError> {
    // G1: a placeholder project. Creating / loading a real one (a start
    // screen, or a `--project` flag) arrives with the input steps.
    let project = Rc::new(Project::new("Untitled"));
    // UI-local preview cursor — the step shown in the body, 0-based into
    // Step::ALL. Distinct from the project's real current step until the
    // step controls land and the wizard actually advances.
    let viewed = Rc::new(Cell::new(0usize));

    let ui = AppWindow::new()?;
    refresh(&ui, &project, viewed.get());

    let weak = ui.as_weak();
    {
        let (viewed, project, weak) = (viewed.clone(), project.clone(), weak.clone());
        ui.on_back(move || {
            viewed.set(viewed.get().saturating_sub(1));
            if let Some(ui) = weak.upgrade() {
                refresh(&ui, &project, viewed.get());
            }
        });
    }
    {
        let (viewed, project, weak) = (viewed.clone(), project.clone(), weak.clone());
        ui.on_next(move || {
            let v = viewed.get();
            if v + 1 < Step::ALL.len() {
                viewed.set(v + 1);
            }
            if let Some(ui) = weak.upgrade() {
                refresh(&ui, &project, viewed.get());
            }
        });
    }
    ui.on_help(|| {
        // G1 placeholder — a help panel / per-step guidance arrives later.
    });

    ui.run()
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
            here: if r.current {
                SharedString::from("   ← you are here")
            } else {
                SharedString::default()
            },
            viewing: r.viewing,
        })
        .collect();

    ui.set_project_name(project.name.clone().into());
    ui.set_steps(ModelRc::new(VecModel::from(rows)));
    ui.set_viewed_number(i32::try_from(viewed_step.number()).unwrap_or(0));
    ui.set_total_steps(i32::try_from(Step::TOTAL).unwrap_or(6));
    ui.set_viewed_title(viewed_step.title().into());
    ui.set_can_back(viewed_idx > 0);
    ui.set_can_next(viewed_idx + 1 < Step::ALL.len());
}
