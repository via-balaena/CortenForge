//! The app's live state: which screen is up, the project, and the wizard's
//! cursor over it.
//!
//! Every decision here is a plain function over plain data in the lib
//! ([`cf_studio_gui::nav_state`], [`WizardCursor`], [`PourSession`]); this
//! module only holds the state and routes button presses into it, so the egui
//! systems have nothing left to get wrong.

use std::path::Path;
use std::time::Instant;

use bevy::prelude::*;
use cf_studio_core::{PourRecord, Project};
use cf_studio_gui::{
    PourAdvance, PourSession, StepOutcome, WizardCursor, apply_scan, pot_life_duration,
};

/// The two top-level screens. The waiver is a *screen*, not an overlay: it is a
/// full-window gate, and modelling it as state keeps the wizard's systems from
/// running at all behind it.
#[derive(States, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub(crate) enum Screen {
    /// First-launch safety + age gate. Shown on **every** launch — acceptance
    /// is held in memory and never persisted.
    #[default]
    Waiver,
    /// The seven-step wizard.
    Wizard,
}

/// Everything the wizard reads and writes.
///
/// ⚠ `cursor` is **not** `project.current_step()`. Back pages the cursor over
/// completed work without touching the project; the project's own step moves
/// only when a step is *completed*.
#[derive(Resource)]
pub(crate) struct Studio {
    /// The session's project. In-memory only — autosave/resume is a later PR.
    pub(crate) project: Project,
    /// Which screen of the wizard is being looked at.
    pub(crate) cursor: WizardCursor,
    /// Which pour layer is active (session-only, like the original).
    pub(crate) pour: PourSession,
    /// Deadline of the running pot-life countdown; `None` = no timer.
    pub(crate) pour_deadline: Option<Instant>,
    /// The step message under the body, and whether it reads as an error.
    pub(crate) message: Option<StepOutcome>,
    /// A long job is running — gates the buttons that could clobber it.
    pub(crate) busy: bool,
}

impl Default for Studio {
    fn default() -> Self {
        Self {
            project: Project::new("Untitled"),
            cursor: WizardCursor::default(),
            pour: PourSession::default(),
            pour_deadline: None,
            message: None,
            busy: false,
        }
    }
}

impl Studio {
    /// Page back one screen, clearing the step message like the original did.
    pub(crate) fn back(&mut self) {
        self.cursor.back();
        self.message = None;
    }

    /// Page forward.
    ///
    /// ⚠ The gate lives in [`WizardCursor::next`], which refuses to move when
    /// the viewed step is incomplete — the disabled button is not trusted,
    /// because an immediate-mode frame can deliver a click against last frame's
    /// enablement. Do NOT re-check it here: a mutation test showed a duplicate
    /// guard makes the gate untestable from this side, since removing either
    /// copy leaves the other passing.
    pub(crate) fn next(&mut self) {
        self.cursor.next(&self.project);
        self.message = None;
    }

    /// Record a newly chosen scan.
    ///
    /// [`Project::set_scan`] clears every downstream artifact, so the session's
    /// own cursors into that work — the pour layer and any running pot-life
    /// timer — go with it, or they point into a plan that no longer exists.
    ///
    /// # Errors
    /// The engine's message if the scan is missing, unreadable, or empty. On a
    /// failure nothing is recorded and nothing is reset.
    pub(crate) fn record_scan(&mut self, scan_file: &Path) -> StepOutcome {
        let outcome = apply_scan(&mut self.project, scan_file);
        if outcome.is_ok() {
            self.pour = PourSession::default();
            self.pour_deadline = None;
        }
        outcome
    }

    /// Start (or restart) the current layer's pot-life countdown.
    ///
    /// Restart is intentional: after the working time expires you scrape,
    /// remix, and click again.
    pub(crate) fn start_pour_timer(&mut self) {
        let minutes = self.project.molds().and_then(|m| {
            m.pour_plan
                .steps
                .get(self.pour.current())
                .map(|s| s.pot_life_minutes)
        });
        let Some(minutes) = minutes else { return };
        self.pour_deadline = Some(Instant::now() + pot_life_duration(minutes));
    }

    /// Seconds left on the countdown, or `None` when no timer is running.
    pub(crate) fn pour_remaining_secs(&self) -> Option<i64> {
        let deadline = self.pour_deadline?;
        Some(
            deadline
                .checked_duration_since(Instant::now())
                .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX)),
        )
    }

    /// Mark the active layer poured and step on, recording completion on the
    /// last one. Stops any running countdown either way.
    pub(crate) fn mark_poured(&mut self) {
        let total = self.project.molds().map_or(0, |m| m.pour_plan.steps.len());
        let advance = self.pour.advance(total);
        if advance == PourAdvance::NoPlan {
            return;
        }
        self.pour_deadline = None;
        if let PourAdvance::Complete { layers_poured } = advance {
            self.message = Some(match self.project.set_pour(PourRecord { layers_poured }) {
                Ok(()) => Ok("🎉 All layers poured — your device is complete!".to_string()),
                Err(e) => Err(format!("Couldn't record completion: {e}")),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cf_studio_core::Step;

    /// A fresh session: nothing done, cursor on the first screen.
    fn fresh() -> Studio {
        Studio::default()
    }

    #[test]
    fn back_from_the_first_screen_is_a_no_op() {
        let mut s = fresh();
        s.back();
        assert_eq!(s.cursor.viewed(), Step::FIRST, "cannot page before step 1");
    }

    #[test]
    fn next_is_refused_while_the_viewed_step_is_incomplete() {
        // The gate, re-checked in `next()` rather than trusted to the disabled
        // button — an immediate-mode frame can deliver a stale click.
        let mut s = fresh();
        s.next();
        assert_eq!(
            s.cursor.viewed(),
            Step::FIRST,
            "an incomplete step must not let the cursor advance"
        );
    }

    #[test]
    fn paging_clears_the_step_message() {
        let mut s = fresh();
        s.message = Some(Err("stale".to_string()));
        s.back();
        assert!(s.message.is_none(), "Back clears the message");
        s.message = Some(Err("stale".to_string()));
        s.next();
        assert!(s.message.is_none(), "Next clears it too, even when gated");
    }

    #[test]
    fn marking_poured_without_a_plan_changes_nothing() {
        // A project with no molds has no pour plan. The button must not advance
        // a cursor into a plan that does not exist, or claim completion.
        let mut s = fresh();
        s.mark_poured();
        assert_eq!(s.pour.current(), 0, "no plan, no advance");
        assert!(s.message.is_none(), "and nothing reported as finished");
    }

    #[test]
    fn starting_the_timer_without_a_plan_leaves_it_stopped() {
        let mut s = fresh();
        s.start_pour_timer();
        assert!(s.pour_deadline.is_none(), "no pot life to count down");
        assert!(
            s.pour_remaining_secs().is_none(),
            "so no countdown is shown"
        );
    }

    /// A project walked to [`Step::MakeMolds`] with a two-layer pour plan.
    ///
    /// Every artifact on the way is plain data, so this needs no files and no
    /// cast run — the setters only gate on the previous step being complete.
    fn with_pour_plan() -> Studio {
        use cf_studio_core::{
            DesignDraft, LayerDraft, MoldOutputs, PlugDraft, PourPlan, PourStep, PrepInput,
            ScanInput,
        };

        let layer = || LayerDraft {
            thickness_m: 0.003,
            material_key: "ECOFLEX_00_30".to_string(),
            slacker_fraction: 0.0,
        };
        let step = |layer_index: usize, pot_life_minutes: u32| PourStep {
            layer_index,
            material_display_name: "Ecoflex 00-30".to_string(),
            mass_g: 40.0,
            mix_ratio_a_to_b: "1:1".to_string(),
            pot_life_minutes,
            cure_time_hours: 4.0,
            slacker_fraction: None,
        };

        let mut s = fresh();
        s.project.set_scan(ScanInput {
            source_path: "scan.stl".into(),
        });
        let steps: [Result<(), cf_studio_core::StudioError>; 4] = [
            s.project.set_prep(PrepInput {
                cleaned_stl: "scan.cleaned.stl".into(),
                prep_toml: "scan.prep.toml".into(),
            }),
            s.project.set_plug(PlugDraft::default()),
            // One layer per pour step, as a real cast run produces.
            s.project.set_design(DesignDraft {
                cavity_inset_m: 0.0,
                layers: vec![layer(), layer()],
            }),
            s.project.set_molds(MoldOutputs {
                out_dir: "out".into(),
                mold_stls: Vec::new(),
                plug_stls: Vec::new(),
                accessory_stls: Vec::new(),
                procedure_path: "procedure.md".into(),
                total_mass_g: 80.0,
                pour_plan: PourPlan {
                    steps: vec![step(0, POT_LIFE_MINUTES), step(1, 45)],
                },
            }),
        ];
        assert!(
            steps.iter().all(Result::is_ok),
            "the fixture must reach MakeMolds: {steps:?}"
        );
        s
    }

    /// The first layer's working time, in minutes — the value the countdown
    /// under test is expected to start from.
    const POT_LIFE_MINUTES: u32 = 30;

    /// Both halves matter: the timer must start at all, and it must count
    /// *down from* the pot life rather than to a moment already past.
    #[test]
    fn the_pour_timer_starts_at_the_layers_pot_life() {
        let mut s = with_pour_plan();
        s.start_pour_timer();

        assert!(s.pour_deadline.is_some(), "a plan means the timer starts");
        let remaining = s.pour_remaining_secs().unwrap_or(-1);
        let expected = i64::from(POT_LIFE_MINUTES) * 60;
        assert!(
            remaining > expected - 60 && remaining <= expected,
            "the deadline must be {expected}s ahead, not behind: {remaining}"
        );
    }

    /// Both halves matter: the cursor moves on, and the clock stops. The clock
    /// is what catches an inverted `PourAdvance::NoPlan` check, since `advance`
    /// has already moved the cursor by the time that check runs.
    #[test]
    fn marking_a_layer_poured_advances_and_stops_the_clock() {
        let mut s = with_pour_plan();
        s.start_pour_timer();
        assert!(
            s.pour_deadline.is_some(),
            "precondition: a timer is running"
        );

        s.mark_poured();

        assert_eq!(s.pour.current(), 1, "on to the second layer");
        assert!(s.pour_deadline.is_none(), "and the countdown stopped");
    }

    /// A minimal valid ASCII STL — one triangle, enough for `load_scan`.
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

    /// `set_scan` clears the molds, so a pour cursor into their plan is stale.
    /// Without this reset a second scan resumes mid-pour on the first's plan.
    #[test]
    fn recording_a_scan_drops_the_previous_scans_pour_session() {
        let scan = std::env::temp_dir().join(format!(
            "cf-studio-gui-record-scan-{}.stl",
            std::process::id()
        ));
        assert!(
            std::fs::write(&scan, ONE_TRIANGLE_STL).is_ok(),
            "the fixture must be writable"
        );

        let mut s = fresh();
        s.pour.advance(3);
        s.pour_deadline = Some(Instant::now() + std::time::Duration::from_secs(600));
        assert_eq!(s.pour.current(), 1, "the fixture must start mid-pour");

        let outcome = s.record_scan(&scan);
        let _ = std::fs::remove_file(&scan);

        assert!(outcome.is_ok(), "the fixture must load: {outcome:?}");
        assert_eq!(s.pour.current(), 0, "the pour cursor was the old scan's");
        assert!(s.pour_deadline.is_none(), "and so was its countdown");
    }

    #[test]
    fn an_expired_deadline_reads_as_zero_not_as_stopped() {
        // The distinction the pour screen rests on: `None` hides the countdown,
        // `Some(0)` shows "working time's up" in red.
        let mut s = fresh();
        s.pour_deadline = Some(Instant::now() - std::time::Duration::from_secs(60));
        assert_eq!(
            s.pour_remaining_secs(),
            Some(0),
            "an expired timer is still a running timer"
        );
    }
}
