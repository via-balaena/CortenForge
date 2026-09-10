//! The app's live state: which screen is up, the project, and the wizard's
//! cursor over it.
//!
//! Every decision here is a plain function over plain data in the lib
//! ([`cf_studio_gui::nav_state`], [`WizardCursor`], [`PourSession`]); this
//! module only holds the state and routes button presses into it, so the egui
//! systems have nothing left to get wrong.

use std::path::{Path, PathBuf};
use std::time::Instant;

use bevy::prelude::*;
use cf_studio_core::{PourRecord, Project, Step};
use cf_studio_engine::{RunProvenance, folder_provenance};
use cf_studio_gui::{
    PourAdvance, PourSession, StepOutcome, WizardCursor, apply_scan, pot_life_duration,
};

/// A step's outcome, and the step that produced it.
///
/// The pair is the point: an outcome with no owner gets shown on whatever
/// screen happens to be up, and gets cleared by whatever happens to page.
pub(crate) struct StepNote {
    /// The step whose action produced this.
    step: Step,
    /// `Ok` reads as a success line, `Err` as a refusal.
    outcome: StepOutcome,
}

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

/// A Save that has not yet finished asking the user where to write.
///
/// Held on [`Studio`] because it gates every other control through
/// `accepting_actions` — the one definition a running job and an open picker
/// already go through. A Save with no question outstanding never reaches here:
/// it writes and lands in the same frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum PendingSave {
    /// `{stem}`'s outputs already sit in `dir` and the modal is asking.
    Confirming {
        /// The folder the collision was found in.
        ///
        /// ⚠ Carried, not re-derived from the scan's own folder: a second
        /// collision can be found in a folder the user picked, and re-deriving
        /// would then offer to overwrite the wrong one.
        dir: PathBuf,
        /// Smoothing as it read when Save was clicked.
        ///
        /// Carried for the reason every [`crate::edit::EditIntent`] carries its
        /// fields — the answer applies to the question that was asked — and so
        /// the picker's poller need not borrow step 2's fields to find it.
        smoothing: usize,
    },
    /// The user asked for a different folder; the picker for it is open.
    ChoosingFolder {
        /// See [`Self::Confirming::smoothing`].
        smoothing: usize,
    },
}

impl PendingSave {
    /// The smoothing the Save was requested with.
    pub(crate) const fn smoothing(&self) -> usize {
        match *self {
            Self::Confirming { smoothing, .. } | Self::ChoosingFolder { smoothing } => smoothing,
        }
    }
}

/// Everything the wizard reads and writes.
///
/// ⚠ `cursor` is **not** `project.current_step()`. Back pages the cursor over
/// completed work without touching the project; the project's own step moves
/// only when a step is *completed*.
#[derive(Resource)]
pub(crate) struct Studio {
    /// The session's project. Saved beside the scan by [`crate::autosave`].
    pub(crate) project: Project,
    /// Which screen of the wizard is being looked at.
    pub(crate) cursor: WizardCursor,
    /// Which pour layer is active (session-only, like the original).
    pub(crate) pour: PourSession,
    /// Deadline of the running pot-life countdown; `None` = no timer.
    pub(crate) pour_deadline: Option<Instant>,
    /// The step message under the body, and the step it belongs to.
    ///
    /// ⚠ Carried TOGETHER on purpose. Held apart, a message can outlive the
    /// screen that produced it — which is how a step-5 refusal came to be
    /// wiped by a page turn, leaving the operator with a cast that had failed
    /// and an app that said nothing.
    ///
    /// ⚠ [`StepNote`]'s own fields are private, so this can be seen from a
    /// sibling module but not taken apart there: [`Studio::note_for`] is the
    /// only way to the outcome, and [`Studio::say`] the only way to set one.
    /// The rule is the compiler's, not this comment's.
    pub(crate) message: Option<StepNote>,
    /// A long job is running — gates the buttons that could clobber it.
    pub(crate) busy: bool,
    /// A Save waiting on the user; `None` unless one asked a question.
    pub(crate) pending_save: Option<PendingSave>,
    /// What the output folder holds, re-read when a cast finishes and when a
    /// project is resumed.
    ///
    /// ⚠ NEVER saved. Staleness is a property of the folder **now**, so a
    /// persisted answer would itself go stale — which is the exact failure
    /// this warns about. `None` = not known: no molds yet, or a folder no
    /// cast has stamped.
    pub(crate) stale: Option<RunProvenance>,
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
            pending_save: None,
            stale: None,
        }
    }
}

impl Studio {
    /// Re-read the output folder's provenance from disk.
    ///
    /// Called when a cast finishes and when a project is resumed — the two
    /// moments the answer can have changed. Reading rather than remembering
    /// is the point: a project reopened days later sees the folder as it is
    /// now, including parts deleted or added outside the app.
    pub(crate) fn refresh_folder_provenance(&mut self) {
        self.stale = self
            .project
            .molds()
            .and_then(|molds| folder_provenance(&molds.out_dir));
    }

    /// Record the outcome of an action taken on the step now being viewed.
    ///
    /// The only way to set a message: the step is stamped here rather than by
    /// the caller, so no writer can leave a note that does not know where it
    /// belongs.
    pub(crate) fn say(&mut self, outcome: StepOutcome) {
        self.message = Some(StepNote {
            step: self.cursor.viewed(),
            outcome,
        });
    }

    /// The note's outcome, whatever step owns it.
    ///
    /// ⚠ For assertions and censuses only. The UI must go through
    /// [`Self::note_for`] — reading the outcome without its step is exactly
    /// what let a step-5 refusal render under step 3.
    #[cfg(test)]
    pub(crate) fn outcome(&self) -> Option<&StepOutcome> {
        self.message.as_ref().map(|note| &note.outcome)
    }

    /// The message to show while `viewed` is on screen, if it is that step's.
    pub(crate) fn note_for(&self, viewed: Step) -> Option<&StepOutcome> {
        self.message
            .as_ref()
            .filter(|note| note.step == viewed)
            .map(|note| &note.outcome)
    }

    /// Page back one screen.
    ///
    /// ⚠ Does NOT clear the step message, and that is the fix: it used to,
    /// "like the original did", so a refusal the operator paged away from was
    /// destroyed rather than remembered. The note names its own step and
    /// [`Studio::note_for`] shows it only there, so leaving it is safe.
    pub(crate) fn back(&mut self) {
        self.cursor.back();
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
            // A new scan is a new project with no molds, so the previous
            // one's folder is not this project's answer to anything.
            self.stale = None;
        }
        outcome
    }

    /// Take a project read back from disk as this session's own.
    ///
    /// ⚠ Everything else here is session-only and goes with it. The pour cursor
    /// and its countdown point into a plan this project may not have, and an
    /// `Instant` means nothing across runs — a pot-life countdown cannot
    /// resume, it restarts, which is what the pour screen already expects.
    pub(crate) fn resume(&mut self, project: Project) {
        // ⚠ `current_step`, not the furthest completed step derived here: the
        // file records where the user was, and a second definition of that
        // would drift from the one [`Project::migrate`] repairs.
        self.cursor = WizardCursor::new(project.current_step());
        self.project = project;
        self.pour = PourSession::default();
        self.pour_deadline = None;
        self.message = None;
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
            let outcome = match self.project.set_pour(PourRecord { layers_poured }) {
                Ok(()) => Ok("🎉 All layers poured — your device is complete!".to_string()),
                Err(e) => Err(format!("Couldn't record completion: {e}")),
            };
            self.say(outcome);
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

    /// ★★★ A refusal must outlive a page turn.
    ///
    /// It did not. `back`/`next` wiped the message "like the original did", so
    /// an operator who paged away from a failed step-5 cast came back to an app
    /// that said nothing — the molds had not been made and the report was gone.
    /// Found by hand-driving, 2026-09-09; the cast really had refused.
    ///
    /// ⚠ Two-sided, and both halves are load-bearing. The note is tied to the
    /// step that produced it, so paging away must HIDE it — a step-2 refusal
    /// means nothing under step 1 — while leaving it INTACT for the return. Drop
    /// the first half and the old wipe passes; drop the second and so does a
    /// note that leaks onto every screen.
    #[test]
    fn a_refusal_survives_paging_away_but_is_shown_only_on_its_own_step() {
        let mut s = fresh();
        s.cursor = WizardCursor::new(Step::CleanScan);
        s.say(Err("the molds did not cast".to_string()));

        assert!(
            matches!(s.note_for(Step::CleanScan), Some(Err(text)) if text.contains("did not cast")),
            "the note shows on the step that produced it: {:?}",
            s.outcome()
        );
        assert!(
            s.note_for(Step::AddScan).is_none(),
            "and on no other — a step-2 refusal means nothing under step 1"
        );

        s.back();
        assert_eq!(s.cursor.viewed(), Step::AddScan, "the page turned");
        assert!(
            s.note_for(Step::AddScan).is_none(),
            "so it is not shown here"
        );
        assert!(
            matches!(s.note_for(Step::CleanScan), Some(Err(text)) if text.contains("did not cast")),
            "but it was NOT destroyed — it is waiting on its own step: {:?}",
            s.outcome()
        );

        // Gated `next` (step 1 is incomplete) must not destroy it either.
        s.next();
        assert!(
            s.note_for(Step::CleanScan).is_some(),
            "a refused page turn is still not a reason to forget a refusal"
        );
    }

    /// A success line is a note like any other: it belongs to its step and is
    /// not wiped by paging. ⚠ Without this, "keep only `Err`" would pass every
    /// assertion above while silently dropping every "✔ …" the moment the
    /// operator looked at another screen.
    #[test]
    fn a_success_line_is_kept_on_its_own_step_too() {
        let mut s = fresh();
        s.cursor = WizardCursor::new(Step::CleanScan);
        s.say(Ok("✔ Welded".to_string()));
        s.back();
        assert!(
            matches!(s.note_for(Step::CleanScan), Some(Ok(text)) if text.contains("Welded")),
            "the success line is still on step 2: {:?}",
            s.outcome()
        );
    }

    #[test]
    fn marking_poured_without_a_plan_changes_nothing() {
        // A project with no molds has no pour plan. The button must not advance
        // a cursor into a plan that does not exist, or claim completion.
        let mut s = fresh();
        s.mark_poured();
        assert_eq!(s.pour.current(), 0, "no plan, no advance");
        assert!(s.outcome().is_none(), "and nothing reported as finished");
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

    /// The sibling of the gate above, and the same reason: a resumed project
    /// carries its own pour plan, so a cursor and a countdown into the one
    /// being replaced point into work this session no longer has.
    ///
    /// ⚠ `Instant` means nothing across runs, so the countdown cannot resume —
    /// it restarts, which is what the pour screen already expects.
    #[test]
    fn resuming_a_project_drops_the_replaced_sessions_pour_cursor() {
        let mut s = with_pour_plan();
        s.pour.advance(2);
        s.start_pour_timer();
        assert_eq!(s.pour.current(), 1, "the fixture must start mid-pour");
        assert!(s.pour_deadline.is_some(), "with a countdown running");

        s.resume(Project::new("resumed"));

        assert_eq!(s.pour.current(), 0, "the cursor was the replaced plan's");
        assert!(s.pour_deadline.is_none(), "and so was its countdown");
        assert_eq!(
            s.cursor.viewed(),
            Step::FIRST,
            "and the screen is the one the resumed project records"
        );
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
