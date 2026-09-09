//! Step 4's field state: the silicone layers built outward off the shaped
//! piece.

use bevy::prelude::*;
use cf_studio_core::LayerDraft;
use cf_studio_gui::{LayerStack, apply_design_draft};

use crate::state::Studio;

/// The step-4 screen's field state, which outlives any one frame.
#[derive(Resource, Default)]
pub(crate) struct DesignControls {
    /// The silicone layers, innermost first.
    pub(crate) layers: LayerStack,
    /// The committed layers these rows were last agreed with.
    followed: Option<Vec<LayerDraft>>,
}

/// Keep step 4's rows in step with the committed design.
///
/// ⚠ The rows are what "Use this design" commits, so rows that do not follow
/// the project do not merely look wrong — the next click writes them over the
/// design they were supposed to be showing.
///
/// See [`crate::shape::drive_shape_controls`] for why this is a reconcile and
/// why it is not gated on the viewed step.
pub(crate) fn drive_design_controls(mut controls: ResMut<DesignControls>, studio: Res<Studio>) {
    let committed = studio
        .project
        .design()
        .map(|design| design.layers.as_slice());
    if controls.followed.as_deref() == committed {
        return;
    }
    controls.followed = committed.map(<[LayerDraft]>::to_vec);
    match committed.map(LayerStack::from_drafts) {
        // Nothing committed — the stack a fresh session opens on.
        None => controls.layers = LayerStack::default(),
        Some(Some(rows)) => controls.layers = rows,
        // ▶ A committed design naming a silicone this build no longer carries.
        // The rows cannot show it, so they are left as they were rather than
        // replaced by a stack that is not this design. Unreachable until a
        // project is loaded from disk, and nothing says so on screen yet — the
        // resume action is where that message belongs.
        Some(None) => {}
    }
}

/// Commit the layer stack as the project's design.
///
/// ⚠ Does not advance, where [`crate::shape::commit_plug`] does — the pre-port
/// screen left you on step 4 with Next → newly enabled, beside two buttons you
/// might reasonably reach for next.
pub(crate) fn commit_design(
    layers: Vec<LayerDraft>,
    controls: &mut DesignControls,
    studio: &mut Studio,
) {
    let outcome = apply_design_draft(&mut studio.project, layers);
    if outcome.is_ok() {
        // The rows ARE the design that just landed. See [`crate::shape::commit_plug`].
        controls.followed = studio.project.design().map(|design| design.layers.clone());
    }
    studio.message = Some(outcome);
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use bevy::ecs::system::RunSystemOnce;
    use cf_studio_core::{DesignDraft, PlugDraft, PrepInput, Project, ScanInput, Step};
    use cf_studio_gui::WizardCursor;

    use super::*;

    /// A project shaped and ready for a stack, with `layers` committed as its
    /// design when there are any.
    fn designed(layers: &[(&str, f64)]) -> Project {
        let mut project = Project::new("design gate");
        project.set_scan(ScanInput {
            source_path: "scan.stl".into(),
        });
        project
            .set_prep(PrepInput {
                cleaned_stl: "scan.cleaned.stl".into(),
                prep_toml: "scan.prep.toml".into(),
            })
            .expect("in workflow order");
        project
            .set_plug(PlugDraft::default())
            .expect("in workflow order");
        if !layers.is_empty() {
            project
                .set_design(DesignDraft {
                    // ⚠ The plug's, or `Project::validate` refuses the pair.
                    cavity_inset_m: 0.0,
                    layers: layers
                        .iter()
                        .map(|&(material_key, thickness_m)| LayerDraft {
                            thickness_m,
                            material_key: material_key.to_string(),
                            slacker_fraction: 0.0,
                        })
                        .collect(),
                })
                .expect("in workflow order");
        }
        project
    }

    /// The wizard parked on step 4 with `project` behind it.
    fn on_step_4(project: Project) -> Studio {
        Studio {
            project,
            cursor: WizardCursor::new(Step::DesignLayers),
            ..Studio::default()
        }
    }

    /// Run the real reconcile once and hand back what it left.
    fn reconcile(studio: Studio, controls: DesignControls) -> DesignControls {
        let mut app = App::new();
        app.insert_resource(studio).insert_resource(controls);
        app.world_mut()
            .run_system_once(drive_design_controls)
            .expect("the reconcile must run");
        app.world_mut()
            .remove_resource::<DesignControls>()
            .expect("controls survive")
    }

    /// The rows' thicknesses, in the whole millimetres the steppers edit.
    fn shown(controls: &DesignControls) -> Vec<i32> {
        controls
            .layers
            .rows()
            .iter()
            .map(|row| row.thickness_mm.value())
            .collect()
    }

    /// ★★ What resume is for. The rows are what "Use this design" commits, so
    /// a committed design that never reaches them is one click from being
    /// replaced by the opening stack.
    #[test]
    fn the_rows_follow_a_design_the_session_did_not_build() {
        let project = designed(&[("DRAGON_SKIN_10A", 0.002), ("DRAGON_SKIN_20A", 0.004)]);

        let after = reconcile(on_step_4(project), DesignControls::default());

        assert_eq!(
            shown(&after),
            vec![2, 4],
            "the rows are the committed stack"
        );
    }

    /// ⚠⚠ This runs every frame. Rebuilding the rows unconditionally would
    /// wipe every edit made since the last commit.
    #[test]
    fn a_redraw_does_not_clobber_the_row_the_user_added() {
        let project = designed(&[("DRAGON_SKIN_10A", 0.002)]);
        let mut editing = reconcile(on_step_4(project.clone()), DesignControls::default());
        editing.layers.add();
        assert_eq!(
            editing.layers.rows().len(),
            2,
            "the fixture carries an edit"
        );

        let after = reconcile(on_step_4(project), editing);

        assert_eq!(after.layers.rows().len(), 2, "the added row stands");
    }

    /// ⚠ The other side of it: `set_plug` clears the design, so the rows must
    /// go back to the stack a fresh screen opens on rather than keep a stack
    /// the project no longer holds.
    #[test]
    fn a_design_the_project_dropped_puts_the_rows_back_to_the_opening_stack() {
        let mut project = designed(&[("DRAGON_SKIN_10A", 0.002)]);
        let followed = reconcile(on_step_4(project.clone()), DesignControls::default());
        assert_eq!(shown(&followed), vec![2], "the fixture must start followed");

        project
            .set_plug(PlugDraft::default())
            .expect("re-shaping clears the design");
        let after = reconcile(on_step_4(project), followed);

        assert_eq!(
            after.layers,
            LayerStack::default(),
            "back to what a fresh screen opens on"
        );
    }

    /// ▶ The branch `jobs.rs` cannot reach: the design picker refuses an
    /// unknown silicone before it can land on the project, but a project file
    /// written by another build can carry one. The rows cannot show it, so
    /// they must not be replaced by a stack that is not this design —
    /// "Use this design" would then write the opening stack over it.
    ///
    /// ⚠ Nothing tells the user yet. That message belongs to the resume
    /// action, which does not exist; this gate holds the rows still until it
    /// does.
    #[test]
    fn a_design_naming_a_silicone_this_build_dropped_leaves_the_rows_alone() {
        let project = designed(&[("NOT_A_SILICONE", 0.002)]);
        let mut editing = DesignControls::default();
        editing.layers.add();
        let before = editing.layers.clone();

        let after = reconcile(on_step_4(project), editing);

        assert_eq!(
            after.layers, before,
            "the rows are left exactly as they were"
        );
    }

    /// ★★ "Use this design" is the moment the rows and the project agree, and
    /// it says so — otherwise the reconcile rebuilds the rows on the next
    /// frame, resetting any half-typed number in one of them.
    ///
    /// ⚠ Committed from rows the reconcile has never followed — a screen that
    /// had already followed this design would carry the stamp before the
    /// commit, and the commit's own would prove nothing.
    #[test]
    fn committing_the_rows_stops_the_reconcile_rebuilding_them() {
        let mut controls = DesignControls::default();
        let mut studio = on_step_4(designed(&[]));

        commit_design(controls.layers.drafts(), &mut controls, &mut studio);
        controls.layers.add();
        let after = reconcile(studio, controls);

        assert_eq!(
            after.layers.rows().len(),
            LayerStack::default().rows().len() + 1,
            "the row added after the commit is still there"
        );
    }

    /// The plugin's wiring: every gate above calls the system directly.
    #[test]
    fn the_plugin_runs_the_reconcile() {
        use bevy::state::app::StatesPlugin;

        let mut app = App::new();
        app.set_error_handler(bevy::ecs::error::ignore);
        app.add_plugins((MinimalPlugins, StatesPlugin, crate::plugin::StudioPlugin));
        app.insert_resource(on_step_4(designed(&[("DRAGON_SKIN_10A", 0.002)])));

        assert_eq!(
            shown(app.world().resource::<DesignControls>()),
            vec![18, 8, 5],
            "the opening stack, before the schedule runs"
        );
        app.world_mut().run_schedule(Update);

        assert_eq!(
            shown(app.world().resource::<DesignControls>()),
            vec![2],
            "the plugin's own schedule put the committed design on the rows"
        );
    }
}
