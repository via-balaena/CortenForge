//! Step 5's field state: which pieces to cast, and at what quality.
//!
//! The picker is *derived*, not edited into existence: the committed design
//! fixes how many layers there are, and therefore exactly which parts can be
//! generated. Nothing here decides that — [`cf_studio_gui::enumerate_parts`]
//! does, and this keeps the screen's copy of it in step.

use bevy::prelude::*;
use cf_studio_core::Step;
use cf_studio_gui::{CENDRILLON_CAST_MODE, PartPicker};

use crate::state::Studio;

/// The step-5 screen's field state, which outlives any one frame.
#[derive(Resource, Default)]
pub(crate) struct MoldControls {
    /// Which pieces to generate. Everything checked is the full cast.
    pub(crate) picker: PartPicker,
    /// The quality picker's index — 0 is fine, 1 is the fast preview. Held as
    /// the index the panel renders, and turned into a cell size only at the
    /// click, by `cell_size_m_for_quality`.
    pub(crate) quality_idx: i32,
    /// The committed layer count the picker was last built for.
    ///
    /// ⚠ The stamp is the layer *count*, not the row count. Rebuilding when
    /// the rows disagree with the design would look equivalent and is not: two
    /// different designs can enumerate the same number of parts, and the
    /// picker would then keep a checkbox pattern belonging to the old one.
    /// `None` means no design has been committed yet.
    stamp: Option<usize>,
}

/// Keep the part picker in step with the committed design while step 5 is up.
///
/// ⚠ A reconcile, not an entry hook — there is no such thing here. `shape.rs`'s
/// preview does the same: run every frame the step is viewed, and do nothing
/// unless the thing it derives from has actually changed.
///
/// Rebuilding checks every part, which is the right default (a full cast) and
/// is also what a changed design demands: the parts it offered before may not
/// exist any more.
pub(crate) fn drive_part_picker(mut controls: ResMut<MoldControls>, studio: Res<Studio>) {
    if studio.cursor.viewed() != Step::MakeMolds {
        return;
    }
    let layers = studio.project.design().map(|design| design.layers.len());
    if controls.stamp == layers {
        return;
    }
    controls.stamp = layers;
    controls.picker = layers.map_or_else(PartPicker::default, |count| {
        PartPicker::rebuild(count, CENDRILLON_CAST_MODE)
    });
}

#[cfg(test)]
pub(crate) mod tests {
    #![allow(clippy::expect_used)]

    use bevy::ecs::system::RunSystemOnce;
    use cf_studio_core::{DesignDraft, LayerDraft, PlugDraft, PrepInput, ScanInput};
    use cf_studio_gui::WizardCursor;

    use super::*;

    fn layer() -> LayerDraft {
        LayerDraft {
            thickness_m: 0.003,
            material_key: "ECOFLEX_00_30".to_string(),
            slacker_fraction: 0.0,
        }
    }

    /// A project with `layers` committed, parked on step 5.
    ///
    /// ⚠⚠ The scan paths are ABSOLUTE, under a temp dir, and that is not
    /// cosmetic. `start_molds` spawns the real cast, and the first thing
    /// `generate_molds_for_design` does is write `<stem>.design.toml` beside
    /// the cleaned scan. With a relative path its parent is `"."`, so the panel
    /// click gate wrote that file into the REPO — twice, once per crate the
    /// suite was run from. The run still fails a moment later on the missing
    /// geometry, which is all the gate needs; the write happens first.
    pub(crate) fn viewing_step_5_with(layers: usize) -> Studio {
        // ⚠⚠ UNIQUE PER CALL. `temp_dir` names the directory by label + PID and
        // `remove_dir_all`s it on entry, so a constant label means ~20 tests
        // across this file and `panel.rs` share one path and delete each other's
        // fixture mid-run — and `clicking_make_molds_in_the_running_wizard_
        // starts_the_cast` writes a real `design.toml` into it while another
        // test may be tearing it down.
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = crate::save::tests::temp_dir(&format!("molds-fixture-{n}"));
        let mut studio = Studio {
            cursor: WizardCursor::new(Step::MakeMolds),
            ..Studio::default()
        };
        studio.project.set_scan(ScanInput {
            source_path: "scan.stl".into(),
        });
        let built = [
            studio.project.set_prep(PrepInput {
                cleaned_stl: dir.join("scan.cleaned.stl"),
                prep_toml: dir.join("scan.prep.toml"),
            }),
            studio.project.set_plug(PlugDraft::default()),
            studio.project.set_design(DesignDraft {
                cavity_inset_m: 0.0,
                layers: std::iter::repeat_with(layer).take(layers).collect(),
            }),
        ];
        assert!(built.iter().all(Result::is_ok), "fixture: {built:?}");
        studio
    }

    fn reconcile(studio: Studio, controls: MoldControls) -> MoldControls {
        let mut app = App::new();
        app.insert_resource(studio).insert_resource(controls);
        app.world_mut()
            .run_system_once(drive_part_picker)
            .expect("the reconcile must run");
        app.world_mut()
            .remove_resource::<MoldControls>()
            .expect("controls survive")
    }

    /// Controls whose picker was derived, by the real reconcile, for a design
    /// of `layers` layers. Shared with `panel.rs` so the screen's gates draw
    /// the picker the app would actually hand them, not a hand-built one.
    pub(crate) fn controls_for(layers: usize) -> MoldControls {
        reconcile(viewing_step_5_with(layers), MoldControls::default())
    }

    #[test]
    fn the_picker_is_built_from_the_committed_design() {
        let controls = reconcile(viewing_step_5_with(2), MoldControls::default());
        // Bonded: 2 layers × 2 cups + the layer-0 plug + platform + dowels.
        assert_eq!(controls.picker.len(), 2 * 2 + 1 + 2);
        assert!(
            controls.picker.rows().all(|(_, checked)| checked),
            "a fresh picker is a full cast"
        );
    }

    /// ★★ The reason the stamp is the layer COUNT and not the row count.
    #[test]
    fn a_changed_design_rebuilds_the_picker() {
        let two = reconcile(viewing_step_5_with(2), MoldControls::default());
        let three = reconcile(viewing_step_5_with(3), two);
        assert_eq!(
            three.picker.len(),
            3 * 2 + 1 + 2,
            "the picker follows the design it was rebuilt for"
        );
    }

    /// ⚠⚠ This runs EVERY FRAME step 5 is up. Rebuilding unconditionally would
    /// re-check every box between the click and the next frame, so a user could
    /// never uncheck anything — and the screen would look like it was ignoring
    /// the mouse.
    #[test]
    fn a_redraw_does_not_clobber_the_boxes_the_user_unchecked() {
        let mut controls = reconcile(viewing_step_5_with(2), MoldControls::default());
        controls.picker.set_all(false);
        controls.picker.set_checked(0, true);

        let after = reconcile(viewing_step_5_with(2), controls);

        let checked: Vec<bool> = after.picker.rows().map(|(_, c)| c).collect();
        assert_eq!(
            checked,
            // 2 layers, bonded: 4 cups + the layer-0 plug + platform + dowels.
            [true, false, false, false, false, false, false],
            "an unchanged design leaves the user's choices exactly as they were"
        );
    }

    /// ⚠ Gated because the reconcile is a plain `Update` system: without the
    /// step guard it would rebuild the picker while the user is on step 4
    /// editing the design it derives from.
    #[test]
    fn the_picker_is_left_alone_off_step_5() {
        let mut studio = viewing_step_5_with(2);
        studio.cursor = WizardCursor::new(Step::DesignLayers);

        let after = reconcile(studio, MoldControls::default());

        assert!(
            after.picker.is_empty(),
            "nothing is derived until the step is actually up"
        );
    }

    /// ★★ The plugin's wiring, which the tests above cannot reach: they call
    /// `drive_part_picker` directly, so they gate the FUNCTION and say nothing
    /// about whether anything runs it. Verified by deleting the schedule entry
    /// — every other test in the crate stayed green, and step 5 would have
    /// opened on an empty parts card forever.
    ///
    /// ⚠ `ignore`, and `Update` by hand, for the reasons `jobs.rs`'s wiring
    /// gates give.
    #[test]
    fn the_plugin_registers_the_controls_and_runs_the_reconcile() {
        use bevy::state::app::StatesPlugin;

        let mut app = App::new();
        app.set_error_handler(bevy::ecs::error::ignore);
        app.add_plugins((MinimalPlugins, StatesPlugin, crate::plugin::StudioPlugin));
        app.insert_resource(viewing_step_5_with(2));

        // `resource::<MoldControls>()` would panic if the plugin had not
        // registered it — that is this line's second job.
        assert!(
            app.world().resource::<MoldControls>().picker.is_empty(),
            "nothing is derived before the schedule runs"
        );
        app.world_mut().run_schedule(Update);

        assert_eq!(
            app.world().resource::<MoldControls>().picker.len(),
            2 * 2 + 1 + 2,
            "the plugin's own schedule derived the picker from the design"
        );
    }

    #[test]
    fn no_design_means_no_parts_to_offer() {
        let studio = Studio {
            cursor: WizardCursor::new(Step::MakeMolds),
            ..Studio::default()
        };
        let after = reconcile(studio, MoldControls::default());
        assert!(after.picker.is_empty(), "nothing to cast without a design");
    }
}
