//! `cf-studio-gui` — the polished Slint GUI for CortenForge Studio.
//!
//! A thin client over [`cf_studio_engine`] + the [`cf_studio_core::Project`]
//! state machine — the same boundary the `cf-studio` CLI drives, so the
//! GUI and CLI behave identically.
//!
//! This lib holds the **headless, testable** part: the mapping from a
//! [`Project`] (plus the step being previewed) to the display rows the
//! Slint markup renders. The Slint event-loop glue lives in `main.rs`
//! (it needs a display to *run*, but compiles headlessly).
//!
//! G1 is the app shell + this state binding + navigation. The per-step
//! controls and the 3-D preview arrive in later phases.

use cf_studio_core::{Project, Step};

/// A workflow step as the checklist shows it. `done` / `current` come
/// from the real project; `viewing` is whether this is the step shown in
/// the wizard body right now (a UI-local preview cursor, distinct from
/// the project's actual current step until the step controls land).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepRow {
    /// One-based step number.
    pub number: i32,
    /// Plain-language step title.
    pub title: String,
    /// Whether the project has completed this step.
    pub done: bool,
    /// Whether this is the project's current step.
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

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

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
}
