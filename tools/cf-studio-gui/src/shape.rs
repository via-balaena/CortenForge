//! Step 3's field state: how snugly the finished piece fits.
//!
//! Holds the numbers between frames and hands out the [`PlugDraft`] a Continue
//! commits.

use bevy::prelude::*;
use cf_studio_core::{PlugDraft, RidgeOptions};
use cf_studio_gui::{StepBoxState, apply_plug};

use crate::state::Studio;

/// The inset field's bounds, in millimetres, as the pre-port screen had them.
/// Fixed — nothing about the scan moves them, unlike
/// [`crate::edit::EditControls::trim_range`].
const CAVITY_MIN_MM: i32 = 0;
const CAVITY_MAX_MM: i32 = 30;
/// The inset the screen opens on, as the pre-port screen did.
const DEFAULT_CAVITY_MM: i32 = 5;
/// One millimetre per click of the stepper.
pub(crate) const CAVITY_STEP_MM: i32 = 1;

/// The step-3 screen's field state, which outlives any one frame.
#[derive(Resource)]
pub(crate) struct ShapeControls {
    /// How far in from the scan surface the cavity sits, in millimetres.
    pub(crate) cavity_mm: StepBoxState,
}

impl Default for ShapeControls {
    fn default() -> Self {
        Self {
            cavity_mm: StepBoxState::new(DEFAULT_CAVITY_MM),
        }
    }
}

/// The inset field's `(min, max)`.
pub(crate) const fn cavity_range() -> (i32, i32) {
    (CAVITY_MIN_MM, CAVITY_MAX_MM)
}

impl ShapeControls {
    /// The plug the fields describe, in the SDK's meters.
    ///
    /// ⚠ Clamped for [`crate::edit::EditControls::simplify_target`]'s reason:
    /// typing does not commit, and [`StepBoxState::value`] tracks an
    /// in-progress edit unclamped by design, so a number typed and then clicked
    /// straight through arrives here out of range.
    pub(crate) fn plug_draft(&self) -> PlugDraft {
        let (min, max) = cavity_range();
        let mm = self.cavity_mm.value().clamp(min, max);
        PlugDraft {
            cavity_inset_m: f64::from(mm) / 1000.0,
            // The ridge editor is a later PR; off is the smooth piece.
            ridges: RidgeOptions::default(),
        }
    }
}

/// Commit the shaped plug, and move on to the layer stack if it took.
pub(crate) fn commit_plug(draft: PlugDraft, studio: &mut Studio) {
    let outcome = apply_plug(&mut studio.project, draft);
    if outcome.is_ok() {
        // ⚠ Before the report, not after: `Studio::next` clears the message, so
        // reporting first would land on step 4 with nothing said.
        studio.next();
    }
    studio.message = Some(outcome);
}

#[cfg(test)]
pub(crate) mod tests {
    #![allow(clippy::expect_used)]

    use std::path::PathBuf;

    use cf_studio_core::{PrepInput, Project, ScanInput, Step};
    use cf_studio_gui::WizardCursor;

    use super::*;

    /// A project with a cleaned scan accepted — the state step 3 is reached in,
    /// since [`Project::set_plug`] refuses before it.
    pub(crate) fn ready_to_shape() -> Project {
        let mut project = Project::new("shape gate");
        project.set_scan(ScanInput {
            source_path: PathBuf::from("scan.stl"),
        });
        project
            .set_prep(PrepInput {
                cleaned_stl: PathBuf::from("scan.cleaned.stl"),
                prep_toml: PathBuf::from("scan.prep.toml"),
            })
            .expect("each artifact is set in workflow order");
        project
    }

    /// The wizard parked on step 3 with `project` behind it.
    fn shaping(project: Project) -> Studio {
        Studio {
            project,
            cursor: WizardCursor::new(Step::ShapePiece),
            ..Studio::default()
        }
    }

    /// ★ The one conversion between the field and the SDK. A field showing 5
    /// that commits 5 *metres* casts a piece the size of a room, and no gate
    /// comparing a draft against another draft can see it.
    #[test]
    fn the_draft_carries_the_field_in_meters_and_inside_its_bounds() {
        let mut controls = ShapeControls::default();
        assert_eq!(
            controls.plug_draft().cavity_inset_m,
            0.005,
            "the field's 5 mm is 0.005 m"
        );

        // ⚠ Typing does not commit, so these reach `plug_draft` unclamped —
        // the path the clamp exists for.
        //
        // ⚠ Both edges. `clamp` swapped for a bare `min` holds the top and lets
        // a negative through, and a negative inset offsets the plug *outward*
        // — a cavity the scan no longer fits.
        for (typed, expected) in [(CAVITY_MAX_MM + 1, 0.030), (CAVITY_MIN_MM - 1, 0.0)] {
            *controls.cavity_mm.text_mut() = typed.to_string();
            controls.cavity_mm.on_typed();
            assert_eq!(
                controls.plug_draft().cavity_inset_m,
                expected,
                "{typed} mm is clamped to the bound it passed"
            );
        }
    }

    /// ★ The order is the trap: [`Studio::next`] clears the message, so a
    /// commit that reported before advancing lands on step 4 with nothing said.
    #[test]
    fn a_committed_plug_advances_the_wizard_and_says_what_it_shaped() {
        let mut studio = shaping(ready_to_shape());

        commit_plug(ShapeControls::default().plug_draft(), &mut studio);

        assert_eq!(studio.cursor.viewed(), Step::DesignLayers, "it moves on");
        assert_eq!(
            studio.project.plug().map(|plug| plug.cavity_inset_m),
            Some(0.005),
            "carrying the inset it was handed"
        );
        assert!(
            matches!(&studio.message, Some(Ok(text)) if text.contains("5.0 mm")),
            "and the report survives the advance: {:?}",
            studio.message
        );
    }

    /// ⚠ The refused arm. `set_plug` rejects a plug before the scan is cleaned,
    /// and advancing anyway would strand the user on a step whose artifact does
    /// not exist.
    #[test]
    fn a_refused_plug_leaves_the_wizard_where_it_was() {
        let mut studio = shaping(Project::new("no cleaned scan"));

        commit_plug(ShapeControls::default().plug_draft(), &mut studio);

        assert_eq!(studio.cursor.viewed(), Step::ShapePiece, "it stays put");
        assert!(studio.project.plug().is_none(), "and records nothing");
        assert!(
            matches!(&studio.message, Some(Err(_))),
            "with the reason on screen: {:?}",
            studio.message
        );
    }
}
