//! The wizard chrome — checklist, header, footer nav — and the per-step bodies.
//!
//! The panel decides nothing. It renders from the lib's plain functions
//! ([`step_rows`], [`nav_state`], the `format_*` family) and turns clicks into
//! the fields of [`Acted`], each executed by a function of its own. Keeping the
//! egui closure free of state transitions is what makes the transitions
//! reviewable — and testable, since they are all methods on `Studio` or plain
//! functions over an `EditSession`.
//!
//! ⚠ The split is **not** an accident of growth — see [`Acted`].

use std::time::Instant;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use cf_studio_core::{LayerDraft, PlugDraft, Step};
use cf_studio_gui::{
    BoundedField, CENDRILLON_CAST_MODE, FitQuestion, FitView, LayerRow, RingRow, Silicone,
    cell_size_m_for_quality, fit_check_is_due, format_fit_failure, format_fit_progress,
    format_fit_verdict, format_molds_summary, format_pour_active, format_pour_plan,
    format_resume_question, format_scan_stats, nav_state, pour_countdown, print_step_summary,
    step_rows,
};

use crate::autosave::{self, Autosave};
use crate::design::{DesignControls, commit_design};
use crate::dialogs::{DialogKind, PendingDialog};
use crate::edit::{
    EditControls, EditIntent, FloorShape, SIMPLIFY_STEP_FACES, SMOOTHING_STEP, STEP_MM,
    apply_edit_intent, simplify_range, smoothing_range,
};
use crate::jobs::{
    MoldsJob, MoldsStart, PlugFitJob, SimplifyJob, start_molds, start_plug_fit, start_simplify,
};
use crate::molds::MoldControls;
use crate::preview::PlugView;
use crate::save;
use crate::scan::ScanEdit;
use crate::shape::{RidgeFields, ShapeControls, commit_plug};
use crate::state::{PendingSave, Studio};
use crate::widgets::{
    ACTIVE_TEXT, CONTROL_TEXT, DONE_TEXT, ERROR_TEXT, GOOD_FILL, GOOD_TEXT, HEADING_TEXT,
    HINT_TEXT, LAYER_FILL, RIDGE_FILL, RIDGE_NOTE_TEXT, RING_FILL, STATS_TEXT, WARN_TEXT, card,
    centered_wrapped, cleanup_section, field_grid, step_box, wrapped_colored, wrapped_label,
};

/// The checklist column's width.
const CHECKLIST_WIDTH: f32 = 260.0;
/// The step body column's width.
const BODY_WIDTH: f32 = 420.0;
/// What the wizard's panels cover at any window width. The rest of the window
/// is the 3D view, which is why `main.rs` sizes the window against this.
pub(crate) const PANEL_WIDTH: f32 = CHECKLIST_WIDTH + BODY_WIDTH;

/// The step status line, a touch larger than body text so it reads as a result
/// rather than as more instructions.
const MESSAGE_SIZE: f32 = 17.0;
/// Between step 2's cleanup sections.
const SECTION_GAP: f32 = 14.0;
/// The working-mesh stats line's point size.
const STATS_SIZE: f32 = 15.0;
/// The "Rebuild the trimmed floor" sub-heading.
const SUBHEADING_SIZE: f32 = 13.0;
/// Its hint, a step smaller than a section's.
const SUBHINT_SIZE: f32 = 12.0;
/// Between a stacked control row and the button that acts on it.
const ROW_GAP: f32 = 6.0;
/// The ridge card's note: what a run with ridges costs.
const RIDGE_NOTE_SIZE: f32 = 14.0;
/// The grip-ring switch's label, sized as the section heading it is.
const RING_HEADING_SIZE: f32 = 15.0;
/// What the screen says when the preview is not the user's own body.
const STAND_IN_NOTE: &str = "The preview is a stand-in shape — your cleaned scan \
     couldn't be read. The ridges are real; the body is not yours.";
/// The rebuilt-floor picker. Fixed, or the combo stretches to fill the column.
const SHAPE_PICKER_WIDTH: f32 = 110.0;
/// One unit per click of every stepper the wizard draws — the pre-port
/// `StepBox` had no step property at all.
const FIELD_STEP: i32 = 1;
/// The layer material picker's width, arrow and padding included.
///
/// ⚠ A **floor**, not a cap: egui grows a `ComboBox` to its selected text and
/// never truncates it, so this cannot cut a silicone name off. What it buys is
/// three stacked pickers that stay one width instead of jittering as materials
/// change — which needs it to clear the *widest* name the catalog carries: 163
/// px of text plus 26 px of chrome, measured with the shipped fonts. A longer
/// silicone than any of today's breaks that, and
/// `the_material_pickers_stay_one_width_whatever_they_show` says so.
const SILICONE_PICKER_WIDTH: f32 = 190.0;

/// The quality picker's width — wide enough for the longer of the two labels.
const QUALITY_PICKER_WIDTH: f32 = 230.0;

/// The quality choices, in the order `cell_size_m_for_quality` indexes them.
///
/// ⚠ That pairing used to be unchecked; `the_quality_labels_match_the_cell_sizes_they_pick`
/// reads the millimetres back out of these labels.
const QUALITY_LABELS: [&str; 2] = ["Fine — 0.5 mm (print quality)", "Fast — 1.5 mm (preview)"];
/// The overwrite modal's width. Wider than the body column — it is centred on
/// the whole window and has to hold a folder path.
const MODAL_WIDTH: f32 = 460.0;

/// What the frame reported.
///
/// The fields are separate because executing an [`EditIntent`] borrows
/// [`ScanEdit`] mutably, which rebuilds a 200 000-face mesh; nothing else here
/// may pay that. See the warning on [`ScanEdit`].
#[derive(Default)]
struct Acted {
    /// A navigation or dialog action.
    nav: Option<Intent>,
    /// A step-2 cleanup op.
    edit: Option<EditIntent>,
    /// The face target a Simplify was clicked with.
    ///
    /// ⚠ Not an [`EditIntent`] variant: starting a Simplify only reads the
    /// scan. See [`start_simplify`].
    simplify: Option<usize>,
    /// The smoothing a Save was clicked with.
    ///
    /// ⚠ Not an [`EditIntent`] variant either, and for the same reason: a Save
    /// reads the scan and writes the *project*. See [`crate::save`].
    save: Option<usize>,
    /// The plug a step-3 Continue was clicked with.
    ///
    /// ⚠ Carried, not re-read at execution time — the same reason `simplify`
    /// and `save` carry theirs: the executor must not see a field the user has
    /// changed since the click.
    plug: Option<PlugDraft>,
    /// The question a step-3 fit check is to answer — the one "Check fit" was
    /// clicked with, or the one the fields settled on with nobody clicking.
    ///
    /// ⚠ Carried like `plug`, and it matters more here: the answer is matched
    /// back against this exact snapshot to decide whether it still describes
    /// the screen, so a question re-read at execution time would always match
    /// and the staleness rule would never fire.
    check_fit: Option<FitQuestion>,
    /// The layer stack a step-4 "Use this design" was clicked with, carried
    /// for the same reason as `plug`.
    design: Option<Vec<LayerDraft>>,
    /// The quality and part selection a step-5 "Make molds" was clicked with,
    /// carried for the same reason as `plug` and `design`.
    molds: Option<MoldsStart>,
}

impl Acted {
    /// Fold in what a nested piece of the screen reported; it wins where both
    /// did, and each field merges on its own.
    fn merge(&mut self, inner: Self) {
        self.nav = inner.nav.or(self.nav);
        self.edit = inner.edit.or(self.edit);
        self.simplify = inner.simplify.or(self.simplify);
        self.save = inner.save.or(self.save);
        self.plug = inner.plug.or(self.plug.take());
        self.check_fit = inner.check_fit.or(self.check_fit.take());
        self.design = inner.design.or(self.design.take());
        self.molds = inner.molds.or(self.molds.take());
    }
}

/// What the user asked for this frame. At most one — a frame cannot hold two
/// clicks, and modelling it as one value stops a "both fired" case existing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Intent {
    Back,
    Next,
    /// Step 1: choose the scan file to work from.
    PickScan,
    /// Step 4: choose a `.design.toml` instead of the editor's own stack.
    PickDesign,
    /// Step 6: choose a folder and copy the printable files into it.
    ExportPrint,
    /// Step 6: reveal the folder the files were copied to.
    OpenExportFolder,
    /// Step 7: start (or restart) the pot-life countdown.
    StartPourTimer,
    /// Step 7: mark the active layer poured.
    MarkPoured,
}

/// Draw the whole wizard and execute whatever was clicked.
// Its parameters ARE its dependency list, and the plugin gate checks that
// list against what the app registers. A `SystemParam` bundle would satisfy
// the lint by hiding the very thing that gate reads.
#[allow(clippy::too_many_arguments)]
pub(crate) fn wizard_screen(
    mut contexts: EguiContexts,
    mut studio: ResMut<Studio>,
    mut dialog: ResMut<PendingDialog>,
    mut scan: ResMut<ScanEdit>,
    mut controls: ResMut<EditControls>,
    mut shape: ResMut<ShapeControls>,
    mut design: ResMut<DesignControls>,
    mut molds: ResMut<MoldControls>,
    mut job: ResMut<SimplifyJob>,
    mut molds_job: ResMut<MoldsJob>,
    mut fit_job: ResMut<PlugFitJob>,
    mut autosave: ResMut<Autosave>,
    preview: Res<PlugView>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let mut acted = Acted::default();
    // ★ Once, and threaded down as a `bool`. Every screen used to re-derive it
    // from the resources it happened to hold, so a gate added here reached only
    // the screens somebody remembered to hand the new resource to.
    let ready = accepting_actions(&studio, &dialog, &autosave);

    egui::SidePanel::left("checklist")
        .resizable(false)
        .exact_width(CHECKLIST_WIDTH)
        .show(ctx, |ui| draw_checklist(ui, &studio));

    egui::TopBottomPanel::bottom("nav").show(ctx, |ui| {
        acted.nav = draw_nav(ui, &studio, ready).or(acted.nav);
    });

    body_column(ctx, |ui| {
        // ⚠ `&scan` — an immutable borrow. Reaching for `&mut` here, to save
        // passing it twice, would mark the resource changed on every frame the
        // wizard drew and re-mesh the scan 60 times a second.
        acted.merge(draw_body(
            ui,
            &studio,
            ready,
            autosave.note().as_deref(),
            &scan,
            &mut Editors {
                controls: &mut controls,
                shape: &mut shape,
                design: &mut design,
                molds: &mut molds,
            },
            &fit_job,
            preview.showing_proxy(),
        ));
    });

    // ⚠ Drawn from `pending_save` alone, outside the step match: the state that
    // gates every control and the modal that explains why it is gated are the
    // same `Option`, so neither can outlive the other.
    if let Some(PendingSave::Confirming { dir, smoothing }) = studio.pending_save.clone()
        && let Some(choice) = draw_save_modal(ctx, &save::overwrite_question(&studio, &dir))
    {
        // ⚠ `&scan`, immutably — a Save only reads the session.
        apply_save_choice(choice, &dir, smoothing, &scan, &mut studio, &mut dialog);
    }

    // ⚠ Drawn from the autosave's own state, outside the step match, for the
    // Save modal's reason: the value that suspends every write and the question
    // that explains why are one and the same, so neither can outlive the other.
    if let Some(reached) = autosave.asking_about()
        && let Some(choice) = draw_resume_modal(ctx, &format_resume_question(reached))
    {
        apply_resume_choice(choice, &mut autosave, &mut studio, &mut scan);
    }

    if let Some(intent) = acted.nav {
        apply_intent(intent, &mut studio, &mut dialog);
    }
    if let Some(intent) = acted.edit {
        apply_edit_intent(intent, &mut scan, &mut studio, &mut controls);
    }
    // ⚠ `&scan`, immutably — see [`start_simplify`].
    if let Some(target_faces) = acted.simplify {
        start_simplify(target_faces, &scan, &mut studio, &mut job);
    }
    // ⚠ `&scan`, immutably, for the same reason.
    if let Some(smoothing) = acted.save {
        save::save_to_default(&scan, &mut studio, smoothing);
    }
    if let Some(draft) = acted.plug {
        commit_plug(draft, &mut shape, &mut studio);
    }
    if let Some(question) = acted.check_fit {
        start_plug_fit(question, &studio, &mut fit_job);
    }
    if let Some(layers) = acted.design {
        commit_design(layers, &mut design, &mut studio);
    }
    if let Some(start) = acted.molds {
        start_molds(&start, &mut studio, &mut molds_job);
    }
    Ok(())
}

/// The column every step body is laid out in.
///
/// ⚠ Extracted so the layout gates lay out in *this*, not in a copy of it. A
/// test that rebuilt the panel would agree with itself while the app drifted.
fn body_column(ctx: &egui::Context, add: impl FnOnce(&mut egui::Ui)) {
    egui::SidePanel::right("body")
        .resizable(false)
        .exact_width(BODY_WIDTH)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, add);
        });
}

/// The seven-step progress list. `✔`/`○` is real completion; the arrow marks
/// the screen being viewed — two different things that must not be conflated.
fn draw_checklist(ui: &mut egui::Ui, studio: &Studio) {
    ui.add_space(8.0);
    ui.heading(&studio.project.name);
    ui.separator();
    for row in step_rows(&studio.project, studio.cursor.viewed()) {
        let mark = if row.done { "✔" } else { "○" };
        let text = format!("{mark}  {}. {}", row.number, row.title);
        let rich = if row.viewing {
            egui::RichText::new(format!("{text}   ← you are here")).strong()
        } else {
            egui::RichText::new(text)
        };
        ui.add(egui::Label::new(rich).wrap());
        ui.add_space(4.0);
    }
}

/// Whether the wizard is accepting actions.
///
/// A long job owns the app until it finishes, an open OS dialog owns it until
/// it resolves, an unanswered Save owns it until it is answered, and so does an
/// unanswered resume question — and **paging counts**: the picker's result lands
/// on whichever step the cursor has reached by then, and a scan landing resets
/// the project to step 1. One definition so a new control cannot honour half of
/// it, evaluated once per frame in [`wizard_screen`].
fn accepting_actions(studio: &Studio, dialog: &PendingDialog, autosave: &Autosave) -> bool {
    !studio.busy
        && !dialog.is_open()
        && studio.pending_save.is_none()
        && autosave.asking_about().is_none()
}

/// Back / Help / Next, gated by [`nav_state`].
fn draw_nav(ui: &mut egui::Ui, studio: &Studio, ready: bool) -> Option<Intent> {
    let nav = nav_state(&studio.project, studio.cursor.viewed());
    let mut intent = None;
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        if ui
            .add_enabled(nav.can_back && ready, egui::Button::new("← Back"))
            .clicked()
        {
            intent = Some(Intent::Back);
        }
        // ⚠ Reworded: this promised guidance "with the ported steps", and every
        // step is now ported. A disabled control whose tooltip names a
        // milestone outlives the milestone.
        ui.add_enabled(false, egui::Button::new("Help"))
            .on_disabled_hover_text("Per-step guidance isn't written yet.");
        if ui
            .add_enabled(nav.can_next && ready, egui::Button::new("Next →"))
            .clicked()
        {
            intent = Some(Intent::Next);
        }
    });
    ui.add_space(6.0);
    intent
}

/// The field state of every step editor, threaded to whichever screen is on.
///
/// ⚠ A plain struct, not a `SystemParam` bundle. `wizard_screen`'s own
/// parameter list is what the plugin gate reads against the app's registered
/// resources; this sits one level below that, where the only thing growing is
/// the number of ported screens.
struct Editors<'a> {
    /// Step 2's cleanup fields.
    controls: &'a mut EditControls,
    /// Step 3's cavity inset and ridges.
    shape: &'a mut ShapeControls,
    /// Step 4's silicone stack.
    design: &'a mut DesignControls,
    /// Step 5's part picker and quality.
    molds: &'a mut MoldControls,
}

/// The step message, then the body for the viewed step.
///
/// ⚠ The message sits **above** the body, where the pre-port screen put it —
/// *"so it's always visible, not buried at the bottom of a long, scrolling
/// step."* Step 2 is that long step, and it is the one whose ops the message
/// reports on, so the note's own reason applies here first.
// Seven step bodies read from here, and each new one needs what it needs. A
// bundle would only move the list somewhere the compiler stops checking that
// every caller supplies it.
#[allow(clippy::too_many_arguments)]
fn draw_body(
    ui: &mut egui::Ui,
    studio: &Studio,
    ready: bool,
    saving_note: Option<&str>,
    scan: &ScanEdit,
    editors: &mut Editors<'_>,
    fit_job: &PlugFitJob,
    showing_a_stand_in: bool,
) -> Acted {
    let viewed = studio.cursor.viewed();
    ui.add_space(8.0);
    ui.heading(format!(
        "Step {} of {} — {}",
        viewed.number(),
        Step::TOTAL,
        viewed.title()
    ));
    ui.separator();

    if let Some(message) = studio.note_for(viewed) {
        ui.add_space(8.0);
        // Centred, larger than body text, and green when it went well — the
        // pre-port status line's own styling. #870 rendered it as a plain label,
        // which lost the success/failure colour split entirely.
        let (text, color) = match message {
            Ok(text) => (text, DONE_TEXT),
            Err(text) => (text, ERROR_TEXT),
        };
        centered_wrapped(ui, MESSAGE_SIZE, color, text.clone());
    }

    // ⚠ On every step, beside the step message and for its reason — at the top
    // of the body rather than the bottom of a long scrolling one. It cannot BE
    // the step message: that is wiped by the next step action, and this has to
    // outlive every click until saving works again.
    if let Some(note) = saving_note {
        ui.add_space(ROW_GAP);
        centered_wrapped(ui, RIDGE_NOTE_SIZE, WARN_TEXT, note.to_owned());
    }

    // ⚠ On every step that shows the piece. It lived in `draw_shape_piece`,
    // correct while step 3 was the only such screen — once 4 and 5 showed it
    // too, a failed scan meant committing to a 36-minute cast against a
    // stand-in presented as the user's own body.
    if showing_a_stand_in && crate::scene::shows_the_piece(viewed) {
        ui.add_space(ROW_GAP);
        centered_wrapped(ui, RIDGE_NOTE_SIZE, WARN_TEXT, STAND_IN_NOTE);
    }

    // ⚠ Exhaustive on purpose, with no catch-all arm. A `_ =>` (or a list of
    // "not ported yet" steps) would silently render one step's screen for a
    // step added later; this way the compiler names the new arm.
    let mut acted = Acted::default();
    match viewed {
        Step::AddScan => acted.nav = draw_add_scan(ui, studio, ready),
        Step::CleanScan => {
            acted.merge(draw_clean_scan(ui, ready, scan, editors.controls));
        }
        Step::ShapePiece => {
            // ⚠ Read before the mutable borrow of `shape`, and read from step
            // 5's live picker rather than pinned here: the pre-flight's own doc
            // says to ask at the size the cast will run at, because detachment
            // turns on sub-cell grid alignment and verdicts do not transfer
            // between cell sizes.
            let cell_size_m = cell_size_m_for_quality(editors.molds.quality_idx);
            acted.merge(draw_shape_piece(
                ui,
                studio,
                ready,
                editors.shape,
                fit_job,
                cell_size_m,
            ));
        }
        Step::DesignLayers => {
            acted.merge(draw_design_layers(ui, ready, editors.design));
        }
        Step::MakeMolds => {
            acted.merge(draw_make_molds(ui, studio, ready, editors.molds));
        }
        Step::Print => acted.nav = draw_print(ui, studio, ready),
        Step::Pour => acted.nav = draw_pour(ui, studio),
    }
    acted
}

/// Step 2 — clean the scan, live, in the same viewport.
///
/// Laid out as an ordered sequence (First → Then → Next) so the cleanup order
/// is obvious, with later sections revealed only as earlier ones complete.
fn draw_clean_scan(
    ui: &mut egui::Ui,
    ready: bool,
    scan: &ScanEdit,
    controls: &mut EditControls,
) -> Acted {
    ui.add_space(8.0);
    let Some(active) = scan.active() else {
        wrapped_label(ui, "Add a scan in step 1 first.");
        return Acted::default();
    };
    let session = active.session();
    let has_centerline = session.has_centerline();
    // Reconstruct is offered once a floor trim has been applied *and* the
    // centerline it was cut along still exists.
    let has_floor_trim = session.reconstruct_available() && has_centerline;
    let mut acted = Acted::default();

    centered_wrapped(
        ui,
        STATS_SIZE,
        STATS_TEXT,
        format_scan_stats(session.face_count(), session.vertex_count()),
    );
    ui.add_space(SECTION_GAP);

    cleanup_section(
        ui,
        "First — tidy the scan",
        "Merge duplicate points so the rest works. A very heavy scan can be \
         lightened, too.",
        false,
        |ui| acted.merge(draw_tidy_row(ui, controls, ready)),
    );
    ui.add_space(SECTION_GAP);

    cleanup_section(
        ui,
        "Then — stand it upright",
        "Finds the open end and stands the scan vertical. Needed before you can \
         save.",
        has_centerline,
        |ui| {
            let label = if has_centerline {
                "Find floor again"
            } else {
                "Find floor"
            };
            ui.vertical_centered(|ui| {
                if ui.add_enabled(ready, egui::Button::new(label)).clicked() {
                    acted.edit = Some(EditIntent::FindFloor);
                }
            });
        },
    );

    // Trimming is measured along the centerline, so there is nothing to offer
    // until one exists.
    if has_centerline {
        ui.add_space(SECTION_GAP);
        cleanup_section(
            ui,
            "Next — trim the open end",
            "Trim the ragged open edge. Removing about 10 mm from the floor is a \
             good start.",
            false,
            |ui| {
                acted.edit = draw_trim_row(ui, controls, ready).or(acted.edit);
                // Nested inside the trim section, as it was pre-port: it undoes
                // part of the cut made directly above it, and reading as a
                // sibling section would make it look like a third way to trim.
                if has_floor_trim {
                    ui.add_space(SECTION_GAP);
                    acted.edit = draw_reconstruct_row(ui, controls, ready).or(acted.edit);
                }
            },
        );
    }

    // Last, as it was pre-port, and shown even with no centerline: the hint is
    // where the user is told which step above unblocks it.
    ui.add_space(SECTION_GAP);
    cleanup_section(
        ui,
        "Finally — save your cleaned scan",
        if has_centerline {
            ""
        } else {
            "Do \u{201c}stand it upright\u{201d} above first."
        },
        false,
        |ui| acted.save = draw_save_row(ui, controls, ready && has_centerline),
    );

    // Secondary, and centred like the rest of the section controls.
    ui.add_space(SECTION_GAP);
    ui.vertical_centered(|ui| {
        if ui
            .add_enabled(ready, egui::Button::new("Start over"))
            .clicked()
        {
            acted.edit = Some(EditIntent::Reset);
        }
    });
    acted
}

/// The save row: how much to smooth, then the button that writes both files.
///
/// ⚠ One row, as it was pre-port — *not* stacked like [`draw_trim_row`].
/// Measured, because that row's overflow makes stacking look like the safe
/// default: this one reaches 309 px of the 404 px column. Stacking it would be
/// layout guessed rather than measured.
fn draw_save_row(ui: &mut egui::Ui, controls: &mut EditControls, ready: bool) -> Option<usize> {
    let mut clicked = None;
    ui.vertical_centered(|ui| {
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "Smoothing");
            step_box(
                ui,
                &mut controls.smoothing,
                smoothing_range(),
                SMOOTHING_STEP,
                ready,
            );
            if ui
                .add_enabled(ready, egui::Button::new("Save cleaned scan"))
                .clicked()
            {
                clicked = Some(controls.smoothing_iters());
            }
        });
    });
    clicked
}

/// The tidy row: Weld, then a face target and Simplify.
///
/// ⚠ Grouped: label, field and button are one unit, so the only place the row
/// may break is between Weld and them.
fn draw_tidy_row(ui: &mut egui::Ui, controls: &mut EditControls, ready: bool) -> Acted {
    let mut acted = Acted::default();
    ui.horizontal_wrapped(|ui| {
        if ui
            .add_enabled(ready, egui::Button::new("Weld points"))
            .clicked()
        {
            acted.edit = Some(EditIntent::Weld);
        }
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "Simplify to");
            step_box(
                ui,
                &mut controls.target_faces,
                simplify_range(),
                SIMPLIFY_STEP_FACES,
                ready,
            );
            if ui
                .add_enabled(ready, egui::Button::new("Simplify"))
                .clicked()
            {
                acted.simplify = Some(controls.simplify_target());
            }
        });
    });
    acted
}

/// The trim controls: a stepper for each end, then Apply trim.
///
/// ⚠⚠ Stacked, and it must stay stacked: as one row these measure 492 px in a
/// 404 px column, and egui culls the button rather than wrapping it.
fn draw_trim_row(
    ui: &mut egui::Ui,
    controls: &mut EditControls,
    ready: bool,
) -> Option<EditIntent> {
    let mut intent = None;
    let range = controls.trim_range();
    ui.vertical_centered(|ui| {
        field_grid(ui, "trim-fields", |ui| {
            ui.colored_label(CONTROL_TEXT, "from tip");
            step_box(ui, &mut controls.tip_mm, range, STEP_MM, ready);
            ui.colored_label(CONTROL_TEXT, "mm");
            ui.end_row();
            ui.colored_label(CONTROL_TEXT, "from floor");
            step_box(ui, &mut controls.floor_mm, range, STEP_MM, ready);
            ui.colored_label(CONTROL_TEXT, "mm");
            ui.end_row();
        });
        ui.add_space(ROW_GAP);
        if ui
            .add_enabled(ready, egui::Button::new("Apply trim"))
            .clicked()
        {
            intent = Some(EditIntent::ApplyTrim {
                tip_mm: controls.tip_mm.value(),
                floor_mm: controls.floor_mm.value(),
            });
        }
    });
    intent
}

/// How the overwrite question was answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SaveChoice {
    /// Write over what is already in the folder.
    Overwrite,
    /// Open a picker for a different folder.
    ChooseFolder,
    /// Do not save.
    Cancel,
}

/// The overwrite question, over the whole window.
///
/// ⚠ An [`egui::Modal`], not the `rfd::MessageDialog` the pre-port used: a
/// native dialog blocks, and blocking from a Bevy system deadlocks the app on
/// macOS every time — see [`crate::dialogs`].
///
/// Returns the answer instead of acting on it, so all three outcomes are
/// reachable from a test without putting an OS picker on screen.
fn draw_save_modal(ctx: &egui::Context, question: &str) -> Option<SaveChoice> {
    let mut choice = None;
    let modal = egui::Modal::new(egui::Id::new("overwrite-outputs")).show(ctx, |ui| {
        ui.set_max_width(MODAL_WIDTH);
        centered_wrapped(ui, SUBHEADING_SIZE, HEADING_TEXT, "Output already exists");
        ui.add_space(ROW_GAP);
        wrapped_label(ui, question);
        ui.add_space(SECTION_GAP);
        ui.horizontal(|ui| {
            if ui.button("Overwrite").clicked() {
                choice = Some(SaveChoice::Overwrite);
            }
            if ui.button("Choose a different folder\u{2026}").clicked() {
                choice = Some(SaveChoice::ChooseFolder);
            }
            if ui.button("Cancel").clicked() {
                choice = Some(SaveChoice::Cancel);
            }
        });
    });
    // Escape and a click on the backdrop are the Cancel button by other means.
    if choice.is_none() && modal.should_close() {
        choice = Some(SaveChoice::Cancel);
    }
    choice
}

/// How the resume question was answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResumeChoice {
    /// Take the saved session as this session's project.
    Resume,
    /// Keep this session's own project, and let it replace the saved one.
    StartOver,
}

/// The resume question, over the whole window.
///
/// ⚠ No Escape, and no dismissal by clicking the backdrop, unlike
/// [`draw_save_modal`]. There is no safe default to map them to: closing as
/// "start over" throws away the saved session on a stray key, and closing as
/// "resume" answers for the user. The two buttons are the only way out, and
/// nothing is saved until one of them is pressed.
fn draw_resume_modal(ctx: &egui::Context, question: &str) -> Option<ResumeChoice> {
    let mut choice = None;
    egui::Modal::new(egui::Id::new("resume-session")).show(ctx, |ui| {
        ui.set_max_width(MODAL_WIDTH);
        centered_wrapped(
            ui,
            SUBHEADING_SIZE,
            HEADING_TEXT,
            "Pick up where you left off?",
        );
        ui.add_space(ROW_GAP);
        wrapped_label(ui, question);
        ui.add_space(SECTION_GAP);
        ui.horizontal(|ui| {
            if ui.button("Pick up where I left off").clicked() {
                choice = Some(ResumeChoice::Resume);
            }
            if ui.button("Start over from this scan").clicked() {
                choice = Some(ResumeChoice::StartOver);
            }
        });
    });
    choice
}

/// Execute the answer.
///
/// ⚠ Extracted for [`apply_save_choice`]'s reason, which applies harder here:
/// one branch replaces the whole project and the other authorises overwriting a
/// file, and neither is reachable from a test while it lives in a closure.
fn apply_resume_choice(
    choice: ResumeChoice,
    autosave: &mut Autosave,
    studio: &mut Studio,
    scan: &mut ScanEdit,
) {
    match choice {
        ResumeChoice::Resume => autosave::resume(autosave, studio, scan),
        ResumeChoice::StartOver => autosave::start_over(autosave),
    }
}

/// Re-cap the chopped floor. Titled so it is clear this rebuilds the floor the
/// trim above just took off, rather than being another way to trim.
fn draw_reconstruct_row(
    ui: &mut egui::Ui,
    controls: &mut EditControls,
    ready: bool,
) -> Option<EditIntent> {
    let mut intent = None;
    centered_wrapped(
        ui,
        SUBHEADING_SIZE,
        HEADING_TEXT,
        "Rebuild the trimmed floor",
    );
    ui.add_space(4.0);
    centered_wrapped(
        ui,
        SUBHINT_SIZE,
        HINT_TEXT,
        "Close the open end back up with a clean floor, rebuilt from the scan's \
         shape just above the cut.",
    );
    ui.add_space(4.0);

    let range = controls.reference_range();
    // Stacked for the same reason as the trim controls above.
    ui.vertical_centered(|ui| {
        field_grid(ui, "reconstruct-fields", |ui| {
            ui.colored_label(CONTROL_TEXT, "Shape");
            // ⚠ `add_enabled_ui` — see [`silicone_picker`]. This picker was
            // live during a Simplify until step 4 gave the app a second combo
            // and the omission showed up beside it.
            ui.add_enabled_ui(ready, |ui| {
                egui::ComboBox::from_id_salt("rebuilt-floor-shape")
                    .width(SHAPE_PICKER_WIDTH)
                    .selected_text(controls.shape.label())
                    .show_ui(ui, |ui| {
                        for shape in FloorShape::ALL {
                            ui.selectable_value(&mut controls.shape, shape, shape.label());
                        }
                    });
            });
            ui.end_row();
            ui.colored_label(CONTROL_TEXT, "from");
            step_box(ui, &mut controls.reference_mm, range, STEP_MM, ready);
            ui.colored_label(CONTROL_TEXT, "mm above cut");
            ui.end_row();
        });
        ui.add_space(ROW_GAP);
        if ui
            .add_enabled(ready, egui::Button::new("Reconstruct floor"))
            .clicked()
        {
            intent = Some(EditIntent::ReconstructFloor {
                shape: controls.shape,
                reference_mm: controls.reference_mm.value(),
            });
        }
    });
    intent
}

/// Step 1 — choose the scan. The 3D view behind the panel shows it.
fn draw_add_scan(ui: &mut egui::Ui, studio: &Studio, ready: bool) -> Option<Intent> {
    let mut intent = None;
    let has_scan = studio.project.is_complete(Step::AddScan);
    ui.add_space(8.0);
    if !has_scan {
        wrapped_label(ui, "Choose a scan to see it here — then drag to spin it.");
        ui.add_space(12.0);
    }
    let label = if has_scan {
        "Choose a different scan…"
    } else {
        "Choose scan file…"
    };
    if ui.add_enabled(ready, egui::Button::new(label)).clicked() {
        intent = Some(Intent::PickScan);
    }
    ui.add_space(8.0);
    wrapped_label(ui, "Works with STL, OBJ, PLY, and 3MF scans.");
    ui.add_space(12.0);
    wrapped_label(
        ui,
        "💡 Leave the bottom open — scan it like it's on a lazy Susan and \
         don't bother closing or capping the floor in your scanning software. \
         CortenForge trims and rebuilds the floor for you in the next step.",
    );
    intent
}

/// Step 5 — make the printable mold pieces. This is the one that runs the cast.
///
/// ⚠ The parts card is derived from the committed design by
/// [`crate::molds::drive_part_picker`], not built here. The screen only draws
/// it and reports what was clicked.
fn draw_make_molds(
    ui: &mut egui::Ui,
    studio: &Studio,
    ready: bool,
    molds: &mut MoldControls,
) -> Acted {
    let mut acted = Acted::default();
    ui.add_space(8.0);
    wrapped_label(
        ui,
        // ⚠ Both numbers are MEASURED, on the reference scan, in the mode this
        // app casts in — `the_app_casts_base_mold_bonded{,_fine}`. They were
        // "fifteen minutes" and "a few" until 2026-09-06, both carried over
        // from DETACHABLE runs the app never performs. If you change the cast,
        // re-run those two gates and change these with them.
        "CortenForge builds the printable mold from your cleaned scan and \
         silicone design. This runs the full cast — print quality takes around \
         forty minutes and the faster preview around seven, and the window \
         stays responsive while it works.",
    );
    ui.add_space(SECTION_GAP);

    ui.horizontal(|ui| {
        ui.colored_label(CONTROL_TEXT, "Quality");
        // ⚠ Inside `add_enabled_ui`, like every other picker here: a ComboBox
        // left live during a run is one the user can change under the cast.
        ui.add_enabled_ui(ready, |ui| {
            egui::ComboBox::from_id_salt("mold-quality")
                .width(QUALITY_PICKER_WIDTH)
                .selected_text(quality_label(molds.quality_idx))
                .show_ui(ui, |ui| {
                    for (index, label) in QUALITY_LABELS.iter().enumerate() {
                        let index = i32::try_from(index).unwrap_or(0);
                        ui.selectable_value(&mut molds.quality_idx, index, *label);
                    }
                });
        });
    });

    // ⚠ A snapshot: the card's empty-state branch reads this while its checkbox
    // loop reads `picker.rows()` live. Nothing between them changes the picker's
    // LENGTH today; a row-adding handler in the button row would.
    let has_rows = !molds.picker.is_empty();
    ui.add_space(ROW_GAP);
    ui.horizontal(|ui| {
        // ⚠ One gate, one place: `start_molds` does not check again. Two copies
        // of a rule make it untestable from either side.
        let enough_parts = molds.picker.any_checked();
        let castable = ready && enough_parts;
        let button = ui.add_enabled(castable, egui::Button::new(make_molds_label(studio.busy)));
        let button = match cast_hint(CastButton {
            ready,
            has_rows,
            enough_parts,
        }) {
            Some(hint) => button.on_disabled_hover_text(hint),
            None => button,
        };
        if button.clicked() {
            acted.molds = Some(MoldsStart {
                cell_size_m: cell_size_m_for_quality(molds.quality_idx),
                selection: molds.picker.selection(CENDRILLON_CAST_MODE),
            });
        }
    });

    ui.add_space(SECTION_GAP);
    draw_parts_picker(ui, molds, ready, has_rows);

    if let Some(summary) = studio.project.molds().map(format_molds_summary) {
        ui.add_space(SECTION_GAP);
        card(ui, GOOD_FILL, |ui| {
            wrapped_colored(ui, GOOD_TEXT, summary);
        });
    }
    acted
}

/// "Parts to generate": All / None, then a checkbox per generatable piece.
///
/// Everything is checked by default — a full cast. Unchecking regenerates just
/// the piece(s) you need and skips the rest, which is the whole point of the
/// selective path.
fn draw_parts_picker(ui: &mut egui::Ui, molds: &mut MoldControls, ready: bool, has_rows: bool) {
    card(ui, LAYER_FILL, |ui| {
        // ⚠ `has_rows` comes IN: the screen's "is there a design" rule has one
        // home, or the button's hover and the card's controls can end up
        // disagreeing about whether one is committed.
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "Parts to generate");
            if ui
                .add_enabled(ready && has_rows, egui::Button::new("All"))
                .clicked()
            {
                molds.picker.set_all(true);
            }
            if ui
                .add_enabled(ready && has_rows, egui::Button::new("None"))
                .clicked()
            {
                molds.picker.set_all(false);
            }
        });
        if !has_rows {
            ui.add_space(ROW_GAP);
            wrapped_colored(ui, HINT_TEXT, "Choose a design in step 4 first.");
            return;
        }
        // ⚠ Collected first: `rows()` borrows the picker that `set_checked`
        // needs mutably, and toggling inside the iteration would not compile.
        let rows: Vec<(usize, String, bool)> = molds
            .picker
            .rows()
            .enumerate()
            .map(|(index, (label, checked))| (index, label.to_string(), checked))
            .collect();
        for (index, label, checked) in rows {
            let mut checked = checked;
            // ⚠ The label belongs to the checkbox, not beside it: that is what
            // gives the control an accessible name to click by.
            if ui
                .add_enabled(ready, egui::Checkbox::new(&mut checked, label))
                .changed()
            {
                molds.picker.set_checked(index, checked);
            }
        }
    });
}

/// What to say on a disabled **Make molds**, or nothing.
///
/// ⚠ Hoisted out of the widget: nothing here can read a tooltip, so every
/// wording rule was invisible to the suite. Explain only what the user can act
/// on — while the app is held, "pick at least one part" beside a full checklist
/// is untrue.
const fn cast_hint(state: CastButton) -> Option<&'static str> {
    if state.ready && state.has_rows && !state.enough_parts {
        Some("Pick at least one part to generate.")
    } else {
        None
    }
}

/// What the Make-molds button knows about itself.
///
/// ⚠ Named fields, not three `bool`s: positional ones type-check in any order,
/// and swapping two left all 170 tests green while the app showed the wrong
/// tooltip.
#[derive(Debug, Clone, Copy)]
struct CastButton {
    /// The app is accepting actions at all.
    ready: bool,
    /// A design is committed, so there are parts to choose between.
    has_rows: bool,
    /// At least one of them is checked.
    enough_parts: bool,
}

/// What to say on a disabled layer **✖**, or nothing. Same rule as
/// [`cast_hint`]: the floor is worth explaining, being held is not.
const fn drop_hint(state: DropButton) -> Option<&'static str> {
    if state.ready && !state.removable {
        Some("The cast needs at least one layer.")
    } else {
        None
    }
}

/// What a layer's ✖ knows about itself. Named for the reason [`CastButton`] is.
#[derive(Debug, Clone, Copy)]
struct DropButton {
    /// The app is accepting actions at all.
    ready: bool,
    /// Dropping this layer would leave at least one behind.
    removable: bool,
}

/// The quality picker's label for `index`, falling back the way
/// [`cell_size_m_for_quality`] does — to print quality.
fn quality_label(index: i32) -> &'static str {
    usize::try_from(index)
        .ok()
        .and_then(|index| QUALITY_LABELS.get(index))
        .unwrap_or(&QUALITY_LABELS[0])
}

/// The cast button's text: the pre-port screen relabelled it rather than
/// showing a separate spinner.
fn make_molds_label(busy: bool) -> &'static str {
    if busy {
        "Making molds…"
    } else {
        "Make molds"
    }
}

/// The fit button's text, relabelled while a check runs rather than growing a
/// spinner — the same thing [`make_molds_label`] does for the cast.
fn check_fit_label(checking: bool) -> &'static str {
    if checking { "Checking…" } else { "Check fit" }
}

/// Step 3 — how snugly the piece fits, what is cut into it, then commit.
///
/// ★ The fit check starts itself once the fields stop moving, which is what
/// keeps the cavity stepper's fixed 0-30 mm honest: see
/// [`cf_studio_gui::fit_check_is_due`]. The button stays because it asks
/// immediately, and because it is the only way back from a check that could not
/// run at all.
fn draw_shape_piece(
    ui: &mut egui::Ui,
    studio: &Studio,
    ready: bool,
    shape: &mut ShapeControls,
    fit_job: &PlugFitJob,
    cell_size_m: f64,
) -> Acted {
    let checking = fit_job.is_running();
    let mut acted = Acted::default();
    ui.add_space(8.0);
    wrapped_label(
        ui,
        "Set how snugly the piece fits: the cavity sits this far in from your \
         scan's surface, all the way round.",
    );
    ui.add_space(SECTION_GAP);
    // ⚠ The click is only remembered here. The question it asks is built at the
    // bottom, once the ridge fields below have been drawn — those are half of
    // what the verdict depends on, and reading them before they are drawn would
    // ask about the previous frame.
    let check_clicked = ui
        .vertical_centered(|ui| {
            ui.horizontal(|ui| {
                ui.colored_label(CONTROL_TEXT, "Cavity inset");
                bounded_step_box(ui, &mut shape.cavity_mm, ready);
                ui.colored_label(CONTROL_TEXT, "mm");
                ui.add_space(ROW_GAP);
                ui.add_enabled(
                    ready && !checking,
                    egui::Button::new(check_fit_label(checking)),
                )
                .clicked()
            })
            .inner
        })
        .inner;
    ui.add_space(SECTION_GAP);
    draw_ridges(ui, &mut shape.ridges, ready);

    let question = FitQuestion {
        plug: shape.plug_draft(),
        cell_size_m,
        scan: studio.project.prep().and_then(crate::preview::scan_stamp),
    };
    let view = fit_job.view(&question);
    draw_fit_view(ui, &view, &question.plug);
    // ⚠ Advanced every frame, clicked or not. The clock is what tells an inset
    // the operator stepped through from the one they stopped on, and it is read
    // here rather than in a system of its own so it watches the question this
    // screen actually built — a second construction of it could drift.
    let settled_for = shape.settle.settled_for(&question, Instant::now());
    let askable = ready && studio.project.prep().is_some();
    if check_clicked || fit_check_is_due(&view, settled_for, askable) {
        acted.check_fit = Some(question);
    }

    ui.add_space(SECTION_GAP);
    ui.vertical_centered(|ui| {
        // ⚠ Never gated on the fit check, and less so now that the check
        // starts itself: one that runs for minutes without being asked for
        // would, gated here, be a wall the operator never chose to build.
        if ui
            .add_enabled(ready, egui::Button::new("Continue"))
            .clicked()
        {
            acted.plug = Some(shape.plug_draft());
        }
    });
    acted
}

/// The fit check's line: a running clock, a verdict about the fields on screen,
/// or nothing at all.
///
/// Coloured like the step message above it — green settled, red refused — so
/// the two read as the same kind of statement.
fn draw_fit_view(ui: &mut egui::Ui, view: &FitView<'_>, plug: &PlugDraft) {
    let (text, color) = match *view {
        FitView::Idle => return,
        FitView::Checking {
            elapsed_secs,
            slow,
            inset_mm,
        } => (
            format_fit_progress(elapsed_secs, slow, inset_mm),
            CONTROL_TEXT,
        ),
        FitView::Answered(fit) => match format_fit_verdict(fit, plug) {
            Ok(text) => (text, DONE_TEXT),
            Err(text) => (text, ERROR_TEXT),
        },
        FitView::Failed(reason) => (format_fit_failure(reason), ERROR_TEXT),
    };
    ui.add_space(ROW_GAP);
    centered_wrapped(ui, MESSAGE_SIZE, color, text);
}

/// The ridge editor: a master switch, and — once it is on — the card of
/// per-feature toggles.
fn draw_ridges(ui: &mut egui::Ui, ridges: &mut RidgeFields, ready: bool) {
    ui.vertical_centered(|ui| {
        ui.add_enabled(
            ready,
            egui::Checkbox::new(&mut ridges.enabled, "Add surface ridges (advanced)"),
        );
    });
    if !ridges.enabled {
        return;
    }
    ui.add_space(ROW_GAP);
    card(ui, RIDGE_FILL, |ui| {
        centered_wrapped(
            ui,
            RIDGE_NOTE_SIZE,
            RIDGE_NOTE_TEXT,
            "Ridges mesh the piece at fine 0.5 mm detail, so a run with them takes \
             print-quality time.",
        );
        ui.add_space(SECTION_GAP);
        draw_rings(ui, ridges, ready);
        ui.add_space(SECTION_GAP);
        field_grid(ui, "ridge-fields", |ui| {
            ridge_row(
                ui,
                RowToggle::Own(&mut ridges.texture_enabled),
                "Surface texture depth",
                &mut ridges.texture_depth,
                "×0.1 mm",
                ready,
            );
            ridge_row(
                ui,
                RowToggle::GovernedAbove(ridges.texture_enabled),
                "Surface texture spacing",
                &mut ridges.texture_spacing,
                "×0.1 mm",
                ready,
            );
            ridge_row(
                ui,
                RowToggle::Own(&mut ridges.side_pinch_enabled),
                "Side pinch depth",
                &mut ridges.side_pinch,
                "×0.1 mm",
                ready,
            );
            ridge_row(
                ui,
                RowToggle::Own(&mut ridges.tip_relief_enabled),
                "Tip relief depth",
                &mut ridges.tip_relief,
                "×0.1 mm",
                ready,
            );
            ridge_row(
                ui,
                RowToggle::Own(&mut ridges.orientation_enabled),
                "Feature orientation",
                &mut ridges.orientation,
                "°",
                ready,
            );
        });
    });
}

/// The grip rings: their switch, a card per ring, and the button that adds one.
fn draw_rings(ui: &mut egui::Ui, ridges: &mut RidgeFields, ready: bool) {
    ui.vertical_centered(|ui| {
        ui.add_enabled(
            ready,
            egui::Checkbox::new(
                &mut ridges.rings_enabled,
                egui::RichText::new("Grip rings")
                    .size(RING_HEADING_SIZE)
                    .color(CONTROL_TEXT),
            ),
        );
    });
    let live = ready && ridges.rings_enabled;
    // ⚠ Noted here and applied after the loop: the row drawing the ✖ is
    // borrowed out of the vector that removing it shortens.
    let mut dropped = None;
    for (index, ring) in ridges.rings.iter_mut().enumerate() {
        ui.add_space(ROW_GAP);
        if draw_ring(ui, index, ring, live) {
            dropped = Some(index);
        }
    }
    if let Some(index) = dropped {
        ridges.rings.remove(index);
    }
    ui.add_space(ROW_GAP);
    ui.vertical_centered(|ui| {
        if ui
            .add_enabled(live, egui::Button::new("+ Add ring"))
            .clicked()
        {
            ridges.add_ring();
        }
    });
}

/// One ring: which one it is, the ✖ that drops it, and its three fields.
/// Reports whether the ✖ was clicked.
///
/// ⚠ A card of stacked rows, not the pre-port screen's single line. That line
/// came to about 620 px of controls in a 404 px column, and egui culls what
/// overflows rather than wrapping it.
fn draw_ring(ui: &mut egui::Ui, index: usize, ring: &mut RingRow, live: bool) -> bool {
    let mut dropped = false;
    card(ui, RING_FILL, |ui| {
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, format!("Ring {}", index + 1));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                dropped = ui.add_enabled(live, egui::Button::new("✖")).clicked();
            });
        });
        field_grid(ui, &format!("ring-{index}"), |ui| {
            ring_field(ui, &mut ring.position, "% along", live);
            ring_field(ui, &mut ring.depth, "×0.1 mm deep", live);
            ring_field(ui, &mut ring.width, "% wide", live);
        });
    });
    dropped
}

/// One of a ring's fields: the stepper, and what its number means.
fn ring_field(ui: &mut egui::Ui, field: &mut BoundedField, unit: &str, live: bool) {
    bounded_step_box(ui, field, live);
    ui.colored_label(CONTROL_TEXT, unit);
    ui.end_row();
}

/// What switches a ridge row's field on.
enum RowToggle<'a> {
    /// The feature's own checkbox, drawn in the row.
    Own(&'a mut bool),
    /// The row above's, with a blank cell where the checkbox would go: texture
    /// spacing has no switch of its own, as the pre-port screen had it.
    GovernedAbove(bool),
}

/// One row of the ridge grid: its toggle, its label, its stepper and the unit.
fn ridge_row(
    ui: &mut egui::Ui,
    toggle: RowToggle<'_>,
    label: &str,
    field: &mut BoundedField,
    unit: &str,
    ready: bool,
) {
    let on = match toggle {
        RowToggle::Own(flag) => {
            ui.add_enabled(ready, egui::Checkbox::without_text(flag));
            *flag
        }
        RowToggle::GovernedAbove(on) => {
            // The blank cell. Without it the columns after it shift left and
            // this row stops lining up with the rest.
            ui.label("");
            on
        }
    };
    ui.colored_label(CONTROL_TEXT, label);
    bounded_step_box(ui, field, ready && on);
    ui.colored_label(CONTROL_TEXT, unit);
    ui.end_row();
}

/// A stepper for one of the shape or layer fields.
///
/// ⚠ The bounds come off the field, so the screen cannot enforce a limit the
/// commit does not. Given the wrong ones the field walks past its own maximum
/// and [`BoundedField::value`] quietly clamps it back — the screen showing one
/// number and the commit carrying another.
fn bounded_step_box(ui: &mut egui::Ui, field: &mut BoundedField, enabled: bool) {
    step_box(ui, &mut field.state, field.range, FIELD_STEP, enabled);
}

/// Step 4 — the silicone stack, built outward off the shaped piece.
fn draw_design_layers(ui: &mut egui::Ui, ready: bool, design: &mut DesignControls) -> Acted {
    let mut acted = Acted::default();
    ui.add_space(8.0);
    wrapped_label(
        ui,
        "Choose the silicone layers, innermost first — soft inside, firmer \
         outside. These build outward off the piece you just shaped.",
    );
    ui.add_space(SECTION_GAP);

    // ⚠ Noted here and applied after the loop: the row drawing the ✖ is
    // borrowed out of the stack that removing it shortens.
    let mut dropped = None;
    // ⚠ Disabled, where the pre-port screen left it live and dropped the click.
    let removable = design.layers.can_drop();
    for (index, layer) in design.layers.rows_mut().iter_mut().enumerate() {
        ui.add_space(ROW_GAP);
        if draw_layer(ui, index, layer, ready, removable) {
            dropped = Some(index);
        }
    }
    if let Some(index) = dropped {
        design.layers.remove(index);
    }

    ui.add_space(SECTION_GAP);
    ui.vertical_centered(|ui| {
        ui.horizontal(|ui| {
            if ui
                .add_enabled(ready, egui::Button::new("+ Add layer"))
                .clicked()
            {
                design.layers.add();
            }
            if ui
                .add_enabled(ready, egui::Button::new("Use this design"))
                .clicked()
            {
                acted.design = Some(design.layers.drafts());
            }
            if ui
                .add_enabled(ready, egui::Button::new("…or load a file"))
                .clicked()
            {
                acted.nav = Some(Intent::PickDesign);
            }
        });
    });
    acted
}

/// One layer: which one it is, the ✖ that drops it, its silicone and its two
/// fields. Reports whether the ✖ was clicked.
///
/// ⚠ A card of stacked rows, not the pre-port screen's single line. That line
/// put a material picker, two steppers, two unit labels and a button side by
/// side — wider than the ring row that already forced this shape, and egui
/// culls what overflows the column rather than wrapping it.
fn draw_layer(
    ui: &mut egui::Ui,
    index: usize,
    layer: &mut LayerRow,
    ready: bool,
    removable: bool,
) -> bool {
    let mut dropped = false;
    card(ui, LAYER_FILL, |ui| {
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, format!("Layer {}", index + 1));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let drop_button = ui.add_enabled(ready && removable, egui::Button::new("✖"));
                let drop_button = match drop_hint(DropButton { ready, removable }) {
                    Some(hint) => drop_button.on_disabled_hover_text(hint),
                    None => drop_button,
                };
                dropped = drop_button.clicked();
            });
        });
        field_grid(ui, &format!("layer-{index}"), |ui| {
            ui.colored_label(CONTROL_TEXT, "Silicone");
            silicone_picker(ui, index, &mut layer.material, ready);
            ui.end_row();
            ui.colored_label(CONTROL_TEXT, "Thickness");
            bounded_step_box(ui, &mut layer.thickness_mm, ready);
            ui.colored_label(CONTROL_TEXT, "mm");
            ui.end_row();
            ui.colored_label(CONTROL_TEXT, "Slacker");
            bounded_step_box(ui, &mut layer.slacker_pct, ready);
            ui.colored_label(CONTROL_TEXT, "%");
            ui.end_row();
        });
    });
    dropped
}

/// The material picker for one layer.
///
/// ⚠ `add_enabled_ui`, because `ComboBox` takes no `enabled` of its own — and
/// a picker left live during a long job is `accepting_actions` honoured by
/// half, which is the thing that rule exists to prevent.
fn silicone_picker(ui: &mut egui::Ui, index: usize, material: &mut Silicone, ready: bool) {
    ui.add_enabled_ui(ready, |ui| {
        egui::ComboBox::from_id_salt(format!("layer-{index}-silicone"))
            .width(SILICONE_PICKER_WIDTH)
            .selected_text(material.name)
            .show_ui(ui, |ui| {
                for silicone in Silicone::catalog() {
                    ui.selectable_value(material, silicone, silicone.name);
                }
            });
    });
}

/// Step 6 — save the printable files, then hand off to the slicer.
fn draw_print(ui: &mut egui::Ui, studio: &Studio, ready: bool) -> Option<Intent> {
    let mut intent = None;
    ui.add_space(8.0);
    wrapped_label(
        ui,
        "Your mold pieces are ready to print. Save them to a folder, then open \
         that folder in your slicer (like OrcaSlicer) and print each piece. The \
         printed molds are what you'll pour silicone into.",
    );
    ui.add_space(12.0);

    let exported = studio.project.print().is_some();
    // `busy` alone, not `ready`: an open dialog is not yet a running save.
    let label = if studio.busy {
        "Saving…"
    } else if exported {
        "Save again…"
    } else {
        "Save files for printing…"
    };
    ui.horizontal(|ui| {
        if ui.add_enabled(ready, egui::Button::new(label)).clicked() {
            intent = Some(Intent::ExportPrint);
        }
        if exported
            && ui
                .add_enabled(ready, egui::Button::new("Open folder"))
                .clicked()
        {
            intent = Some(Intent::OpenExportFolder);
        }
    });

    let summary = print_step_summary(&studio.project);
    if !summary.is_empty() {
        ui.add_space(12.0);
        card(ui, GOOD_FILL, |ui| wrapped_colored(ui, GOOD_TEXT, summary));
    }
    intent
}

/// Step 7 — the guided pour assistant: the plan, the active layer, the timer.
fn draw_pour(ui: &mut egui::Ui, studio: &Studio) -> Option<Intent> {
    let mut intent = None;
    let Some(molds) = studio.project.molds() else {
        ui.add_space(8.0);
        wrapped_label(ui, "Make the molds first — the pour plan comes from them.");
        return None;
    };
    ui.add_space(8.0);
    card(ui, GOOD_FILL, |ui| {
        wrapped_label(ui, format_pour_plan(&molds.pour_plan));
    });

    if studio.project.pour().is_some() {
        ui.add_space(12.0);
        card(ui, GOOD_FILL, |ui| {
            wrapped_colored(
                ui,
                GOOD_TEXT,
                "🎉 All layers poured — your device is complete! Let each layer \
                 cure fully before unmolding.",
            );
        });
        return None;
    }

    ui.add_space(12.0);
    wrapped_colored(
        ui,
        ACTIVE_TEXT,
        format_pour_active(&molds.pour_plan, studio.pour.current()),
    );

    if let Some(remaining) = studio.pour_remaining_secs() {
        let countdown = pour_countdown(remaining);
        let color = match countdown.urgency {
            2 => ERROR_TEXT,
            1 => WARN_TEXT,
            _ => GOOD_TEXT,
        };
        ui.add_space(8.0);
        ui.add(
            egui::Label::new(
                egui::RichText::new(countdown.text)
                    .size(22.0)
                    .strong()
                    .color(color),
            )
            .wrap(),
        );
    }

    ui.add_space(12.0);
    ui.horizontal(|ui| {
        // Always enabled: re-clicking restarts the clock, e.g. after the pot
        // life expired and you remixed.
        let timer_label = if studio.pour_deadline.is_some() {
            "Restart timer"
        } else {
            "Start pour timer"
        };
        if ui.button(timer_label).clicked() {
            intent = Some(Intent::StartPourTimer);
        }
        if ui.button("Mark this layer poured →").clicked() {
            intent = Some(Intent::MarkPoured);
        }
    });
    intent
}

/// Execute the answer the overwrite modal came back with.
///
/// ⚠ Extracted for [`apply_intent`]'s reason, and it is the one this branch
/// learned the hard way: inline in [`wizard_screen`] this was the only intent
/// kind without an executor a test could call, and all three miswirings passed
/// the whole suite — Cancel overwriting the files, Overwrite quietly saving
/// nothing, and a folder answer that opens no picker and leaves the app inert.
fn apply_save_choice(
    choice: SaveChoice,
    dir: &std::path::Path,
    smoothing: usize,
    scan: &ScanEdit,
    studio: &mut Studio,
    dialog: &mut PendingDialog,
) {
    match choice {
        SaveChoice::Overwrite => save::write_into(scan, studio, dir, smoothing),
        SaveChoice::ChooseFolder => {
            studio.pending_save = Some(PendingSave::ChoosingFolder { smoothing });
            dialog.pick_folder(
                DialogKind::PrepDest,
                "Choose a folder to save the cleaned scan",
            );
        }
        SaveChoice::Cancel => save::settle(studio, Ok("Save cancelled.".to_string())),
    }
}

/// Execute an intent. Every state transition in the wizard passes through here.
fn apply_intent(intent: Intent, studio: &mut Studio, dialog: &mut PendingDialog) {
    match intent {
        Intent::Back => studio.back(),
        Intent::Next => studio.next(),
        Intent::PickScan => dialog.pick_scan_file(),
        Intent::PickDesign => dialog.pick_design_file(),
        Intent::StartPourTimer => studio.start_pour_timer(),
        Intent::MarkPoured => studio.mark_poured(),
        Intent::OpenExportFolder => match studio.project.print().map(|p| p.export_dir.clone()) {
            Some(dir) => crate::jobs::reveal_in_file_manager(&dir),
            None => {
                studio.say(Err(
                    "Nothing exported yet — save the files first.".to_string()
                ));
            }
        },
        Intent::ExportPrint => {
            if studio.project.molds().is_none() {
                studio.say(Err("Make the molds first (step 5).".to_string()));
                return;
            }
            dialog.pick_folder(
                DialogKind::PrintDest,
                "Choose a folder to save the printable files",
            );
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    #![allow(clippy::expect_used)]

    use std::path::PathBuf;

    use cf_studio_core::{
        DesignDraft, LayerDraft, MoldOutputs, PourPlan, PourStep, PrepInput, Project, RidgeOptions,
        RidgeRing, ScanInput,
    };
    use cf_studio_gui::WizardCursor;
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable;

    use super::*;
    use crate::edit::tests::open_tube;
    use crate::egui_harness::{self, begin, click_on, end, painted_texts, settle};
    use crate::scan::ActiveScan;

    /// ⚠ Every click on every screen reaches [`wizard_screen`] through this.
    /// A dropped field does not error — the control just stops working.
    #[test]
    fn merging_lands_the_inner_report_without_dropping_the_outer_one() {
        let mut outer = Acted {
            nav: Some(Intent::Back),
            ..Acted::default()
        };

        outer.merge(Acted {
            edit: Some(EditIntent::Weld),
            simplify: Some(200_000),
            ..Acted::default()
        });

        assert_eq!(outer.nav, Some(Intent::Back), "the outer report survives");
        assert_eq!(outer.edit, Some(EditIntent::Weld), "the inner one lands");
        assert_eq!(outer.simplify, Some(200_000), "and each field on its own");
    }

    /// ★ The one definition every control on every screen is gated on.
    ///
    /// ⚠ Enumerated, not branched: it is a four-way `&&`, so a constant answer,
    /// a dropped term or an `||` each leave the app either frozen with nothing
    /// running or clickable in the middle of a job — and none of those report
    /// themselves. Sixteen states is all of them.
    #[test]
    fn actions_are_accepted_only_when_nothing_else_holds_the_app() {
        for busy in [false, true] {
            for dialog_open in [false, true] {
                for saving in [false, true] {
                    for asking in [false, true] {
                        let studio = Studio {
                            busy,
                            pending_save: saving
                                .then_some(PendingSave::ChoosingFolder { smoothing: 0 }),
                            ..Studio::default()
                        };
                        let dialog = if dialog_open {
                            PendingDialog::opened(DialogKind::ScanFile)
                        } else {
                            PendingDialog::default()
                        };
                        let autosave = if asking {
                            Autosave::asking(Project::new("held"), Step::CleanScan)
                        } else {
                            Autosave::default()
                        };

                        assert_eq!(
                            accepting_actions(&studio, &dialog, &autosave),
                            !busy && !dialog_open && !saving && !asking,
                            "busy={busy} dialog_open={dialog_open} saving={saving} asking={asking}"
                        );
                    }
                }
            }
        }
    }

    /// Where both fired the inner one wins, as the pre-merge code did.
    #[test]
    fn an_inner_report_wins_over_the_outer_one() {
        let mut outer = Acted {
            nav: Some(Intent::Back),
            ..Acted::default()
        };

        outer.merge(Acted {
            nav: Some(Intent::Next),
            ..Acted::default()
        });

        assert_eq!(outer.nav, Some(Intent::Next));
    }

    /// The body column's usable width, in points. #878's overflow was measured
    /// against this: a 492 px trim row in a 404 px column.
    const COLUMN_WIDTH: f32 = 404.0;
    /// Tall enough that nothing is cut off the bottom; too short and the fit
    /// check below fails rather than measuring half a screen.
    const COLUMN_HEIGHT: f32 = 1400.0;
    /// A trim the open tube is long enough to take from either end.
    const FIXTURE_TRIM_MM: i32 = 5;

    /// Lay `body` out in [`body_column`] — the column the wizard builds — and
    /// name the controls on it, in layout order, failing if any of them sits
    /// outside it.
    ///
    /// ⚠ Accessibility rects, not painted shapes: egui culls what overflows,
    /// so the shapes cannot show it. This rect is the widget's real position
    /// whether or not it was drawn.
    ///
    /// ⚠ The returned census is the other half, and each is vacuous alone: an
    /// overflowing control fails here, a vanished one fails the caller's
    /// `assert_eq!`. Fit alone passes an empty screen.
    fn controls_in_column(mut body: impl FnMut(&mut egui::Ui)) -> Vec<String> {
        use egui_kittest::kittest::NodeT;

        let column = std::cell::Cell::new(egui::Rect::NOTHING);
        let mut measured = |ui: &mut egui::Ui| {
            column.set(ui.max_rect());
            body(ui);
        };
        let harness = column_harness(&mut measured);

        let harness_fonts = harness.ctx.clone();
        // ⚠ Every piece of text on the screen, not just the control names.
        // Prose sits on `Role::Label` nodes and carries its text in `value()`,
        // not `label()`; reading only `label()` returns the buttons and makes
        // prose look unreachable, which leaves this screen's own labels — the
        // stepper's caption, the hint under a heading — in no gate at all.
        for node in harness.root().children_recursive() {
            let widget = node.accesskit_node();
            if let Some(text) = widget.label().or_else(|| widget.value()) {
                assert_renders(&harness_fonts, &text);
            }
        }
        let column = column.get();
        assert_eq!(
            column.width(),
            COLUMN_WIDTH,
            "the column #878 was measured in"
        );
        harness
            .root()
            .children_recursive()
            .filter_map(|node| {
                let widget = node.accesskit_node();
                let name = control_name(widget.role(), widget.label())?;
                let rect = node.rect();
                assert!(
                    column.contains_rect(rect),
                    "{name} is laid out at {rect:?}, outside the {column:?} column"
                );
                Some(name)
            })
            .collect()
    }

    /// A control, and what these gates call it: its accessible name, or its
    /// role when it has none.
    ///
    /// ⚠ An empty name is no name. A stepper's field reports none at all and a
    /// bare checkbox reports `""`; skipping either lets the very control the
    /// census is for slip past it, and `""` names nothing.
    fn control_name(role: egui::accesskit::Role, label: Option<String>) -> Option<String> {
        use egui::accesskit::Role;

        matches!(
            role,
            Role::Button | Role::TextInput | Role::ComboBox | Role::CheckBox
        )
        .then(|| {
            label
                .filter(|label| !label.is_empty())
                .unwrap_or_else(|| format!("{role:?}"))
        })
    }

    /// Fail if any character of `text` has no glyph in the fonts that ship.
    ///
    /// ⚠ Asks the font stack instead of encoding the answer. A codepoint gate
    /// only rejects the one character somebody already knew about.
    ///
    /// ⚠ Control characters are skipped: `has_glyph` says `false` for `\n`,
    /// which layout breaks the line on rather than drawing, so checking it
    /// would fail every multi-line message on screen.
    pub(crate) fn assert_renders(ctx: &egui::Context, text: &str) {
        let font = egui::FontId::default();
        for c in text.chars().filter(|c| !c.is_control()) {
            assert!(
                ctx.fonts_mut(|f| f.has_glyph(&font, c)),
                "U+{:04X} {c:?} has no glyph — it draws as a box in {text:?}",
                u32::from(c),
            );
        }
    }

    /// ★ The messages, which no census reaches: the accessibility tree names
    /// controls, and every one of these is prose under them.
    ///
    /// ⚠ This is the gate `format_save_done` needed. It shipped U+2713 `✓`,
    /// which no bundled font carries, because the check that existed named one
    /// function and compared one codepoint. The lib cannot run this itself —
    /// it is deliberately toolkit-free, and the fonts belong to the panel.
    ///
    /// ⚠ Every producer in the lib, not step 2's alone: the pour and mold
    /// lines carry the rarest glyphs in the app (`⏱`, `🎉`, `·`, `±`, `°`).
    #[test]
    fn every_message_is_drawable_in_the_fonts_that_ship() {
        let mut harness = Harness::new_ui(|_| {});
        harness.ctx.set_fonts(crate::plugin::font_definitions());
        harness.run();

        let project = ready_to_pour();
        let molds = project
            .molds()
            .expect("the fixture is driven to the pour")
            .clone();
        // A path, not a folder: the question only interpolates one, so creating
        // it would be filesystem work with a cleanup that a failure would skip.
        let dir = std::env::temp_dir().join("cf-glyph-gate");
        let (_scan, studio) = crate::save::tests::ready_to_save(&dir);

        let mut messages = vec![
            cf_studio_gui::format_save_done("base_mold", 180_236),
            cf_studio_gui::format_simplify_done(200_000, 12.3),
            cf_studio_gui::format_simplify_started(50_000),
            shaped_piece_report(),
            cf_studio_gui::format_floor_found(1, 29, 7.7),
            cf_studio_gui::format_floor_no_centerline(3),
            format_scan_stats(200_000, 600_000),
            cf_studio_gui::format_elapsed(3671),
            cf_studio_gui::print_step_summary(&project),
            cf_studio_gui::format_molds_summary(&molds),
            format_pour_plan(&molds.pour_plan),
            format_pour_active(&molds.pour_plan, 0),
            crate::save::overwrite_question(&studio, &dir),
            STAND_IN_NOTE.to_string(),
        ];
        // Each urgency band words itself differently.
        messages.extend([600_i64, 120, -30].map(|secs| pour_countdown(secs).text));
        // Step 3's fit check: both verdicts and both progress lines. The
        // refusal carries the cast's own words, em dash and all.
        let refused_plug = PlugDraft {
            cavity_inset_m: 0.011,
            ..PlugDraft::default()
        };
        messages.extend(
            [
                cf_studio_engine::PlugFit::Casts,
                cf_studio_engine::PlugFit::WillNotCast {
                    reason: "plug layer 0 came out in 3 pieces — a 62128-face body with a \
                             1044-face fragment beside it"
                        .to_string(),
                },
            ]
            .map(|fit| format_fit_verdict(&fit, &refused_plug).unwrap_or_else(|text| text)),
        );
        messages.extend([false, true].map(|slow| format_fit_progress(7, slow, 11.0)));
        messages.push(format_fit_failure("scan.cleaned.stl: no such file"));
        // Step 4's inexact-design warning — the first message in the app to
        // carry U+26A0 `⚠`, which is exactly the shape of the U+2713 miss above.
        messages.push(inexact_design_note());
        messages.push(
            cf_studio_gui::format_ignored_inset(0.012, 0.005)
                .expect("a file that disagrees with the plug must say so"),
        );

        for message in messages {
            assert_renders(&harness.ctx, &message);
        }

        // Step 2's op reports, which the lib does not produce. `↺ Reset` is one
        // of the three arrows `plugin::font_definitions` exists for, and the
        // only one no control label already covers — `← Back` and `Next →` are
        // in the nav census.
        let mut screen = cleanup_screen();
        let mut reports = Vec::new();
        for intent in [EditIntent::Weld, EditIntent::FindFloor, EditIntent::Reset] {
            apply_edit_intent(
                intent,
                &mut screen.scan,
                &mut screen.studio,
                &mut screen.controls,
            );
            let reported = screen.studio.outcome().expect("every op reports");
            let (Ok(text) | Err(text)) = reported;
            assert_renders(&harness.ctx, text);
            reports.push(text.clone());
        }
        // ⚠ Otherwise this loop checks whatever the ops happen to say. Reword
        // one — or let a guard report in its place — and the arrow this is here
        // for stops being checked, with the gate still green.
        assert!(
            reports.iter().any(|text| text.contains('\u{21ba}')),
            "no op reported the arrow this covers: {reports:?}"
        );
    }

    /// What a committed plug reports — the only `apply_plug` message the screen
    /// ever shows, and one no other producer here covers.
    fn shaped_piece_report() -> String {
        let mut project = crate::shape::tests::ready_to_shape();
        // ⚠ `expect`, not either arm: a refusal is a different string, and the
        // gate would go on checking it with nothing to say the message it
        // exists for had stopped being produced.
        cf_studio_gui::apply_plug(&mut project, ShapeControls::default().plug_draft())
            .expect("the fixture is ready to shape")
    }

    /// Step 2 with every section revealed: a scan loaded, a centerline traced,
    /// and a floor trim applied — the only state that shows the reconstruct
    /// block, and the state #878 was found in.
    struct CleanupScreen {
        studio: Studio,
        scan: ScanEdit,
        controls: EditControls,
    }

    fn cleanup_screen() -> CleanupScreen {
        let mut scan = ScanEdit::default();
        scan.set(ActiveScan::synthetic(open_tube()));
        let mut studio = Studio::default();
        let mut controls = EditControls::default();
        apply_edit_intent(EditIntent::FindFloor, &mut scan, &mut studio, &mut controls);
        apply_edit_intent(
            EditIntent::ApplyTrim {
                tip_mm: FIXTURE_TRIM_MM,
                floor_mm: FIXTURE_TRIM_MM,
            },
            &mut scan,
            &mut studio,
            &mut controls,
        );
        // ⚠ A failed op would hide the later sections and leave the census
        // trivially short — passing while measuring half the screen.
        let session = scan.active().map(ActiveScan::session);
        assert!(
            session.is_some_and(|s| s.has_centerline() && s.reconstruct_available()),
            "the fixture must reveal every section; last message: {:?}",
            studio.outcome()
        );
        CleanupScreen {
            studio,
            scan,
            controls,
        }
    }

    /// A project driven to the pour step — the only state `draw_pour` shows its
    /// buttons in, since every earlier artifact gates the next.
    pub(crate) fn ready_to_pour() -> Project {
        let mut project = Project::new("layout gate");
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
            .set_plug(PlugDraft {
                cavity_inset_m: 0.005,
                ridges: RidgeOptions::default(),
            })
            .expect("each artifact is set in workflow order");
        project
            .set_design(DesignDraft {
                cavity_inset_m: 0.005,
                layers: vec![LayerDraft {
                    thickness_m: 0.0175,
                    material_key: "ECOFLEX_00_30".to_string(),
                    slacker_fraction: 0.25,
                }],
            })
            .expect("each artifact is set in workflow order");
        project
            .set_molds(MoldOutputs {
                out_dir: PathBuf::from("out"),
                mold_stls: vec![PathBuf::from("out/mold.stl")],
                plug_stls: vec![PathBuf::from("out/plug.stl")],
                accessory_stls: vec![],
                procedure_path: PathBuf::from("out/procedure.md"),
                total_mass_g: 842.0,
                pour_plan: PourPlan {
                    steps: vec![PourStep {
                        layer_index: 0,
                        material_display_name: "Ecoflex 00-30".to_string(),
                        mass_g: 500.0,
                        mix_ratio_a_to_b: "1:1".to_string(),
                        pot_life_minutes: 25,
                        cure_time_hours: 4.0,
                        slacker_fraction: Some(0.25),
                    }],
                },
            })
            .expect("each artifact is set in workflow order");
        project
    }

    /// ★ The gate #878 did not have, on the screen it broke.
    #[test]
    fn every_control_on_the_cleanup_screen_is_inside_the_body_column() {
        let mut screen = cleanup_screen();

        let controls = controls_in_column(|ui| {
            let _ = draw_clean_scan(ui, true, &screen.scan, &mut screen.controls);
        });

        assert_eq!(
            controls,
            [
                "Weld points",
                "−",
                "TextInput",
                "+",
                "Simplify",
                "Find floor again",
                "−",
                "TextInput",
                "+",
                "−",
                "TextInput",
                "+",
                "Apply trim",
                "ComboBox",
                "−",
                "TextInput",
                "+",
                "Reconstruct floor",
                "−",
                "TextInput",
                "+",
                "Save cleaned scan",
                "Start over",
            ]
        );
    }

    /// The pour screen's action row is the shape that broke: ungrouped, wide
    /// labels.
    #[test]
    fn every_control_on_the_pour_screen_is_inside_the_body_column() {
        let studio = Studio {
            project: ready_to_pour(),
            ..Studio::default()
        };

        let controls = controls_in_column(|ui| {
            let _ = draw_pour(ui, &studio);
        });

        assert_eq!(controls, ["Start pour timer", "Mark this layer poured →"]);
    }

    #[test]
    fn every_control_on_the_simpler_screens_is_inside_the_body_column() {
        let studio = Studio::default();

        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_add_scan(ui, &studio, true);
            }),
            ["Choose scan file…"]
        );
        let mut shape = ShapeControls::default();
        assert_eq!(
            controls_in_column(shape_body(&mut shape)),
            [
                "−",
                "TextInput",
                "+",
                CHECK_FIT,
                "Add surface ridges (advanced)",
                "Continue"
            ]
        );
        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_print(ui, &studio, true);
            }),
            ["Save files for printing…"]
        );
        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_nav(ui, &studio, true);
            }),
            ["← Back", "Help", "Next →"]
        );
        assert!(controls_in_column(|ui| draw_checklist(ui, &studio)).is_empty());
    }

    /// Step 5's controls, in the order they are laid out.
    ///
    /// ▶ The quality picker censuses as a bare `"ComboBox"` — egui gives it no
    /// accessible name, app-wide, as with the stepper's `TextInput`.
    #[test]
    fn every_control_on_the_make_molds_screen_is_inside_the_body_column() {
        let studio = Studio::default();
        let mut empty = MoldControls::default();
        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_make_molds(ui, &studio, true, &mut empty);
            }),
            ["ComboBox", "Make molds", "All", "None"],
            "with no design the picker offers nothing to check"
        );

        let studio = crate::molds::tests::viewing_step_5_with(1);
        let mut molds = crate::molds::tests::controls_for(1);
        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_make_molds(ui, &studio, true, &mut molds);
            }),
            [
                "ComboBox",
                "Make molds",
                "All",
                "None",
                "Layer 1 — cup (left)",
                "Layer 1 — cup (right)",
                "Layer 1 — plug",
                "Platform",
                "Dowels",
            ],
            "every offered part is its own named checkbox"
        );
    }

    /// ⚠⚠ Every disabled state, and what it says. Nothing in this crate can
    /// read a tooltip, so before these were hoisted out of the widgets both
    /// wording rules could be deleted with the whole suite green.
    #[test]
    fn a_disabled_control_explains_only_what_the_user_can_act_on() {
        // ⚠ Every state, written out. A loop computing `expected` from the same
        // boolean expression the function uses is a mirror: edit both in one
        // sitting and the gate stays green.
        use CastButton as C;
        const PICK: Option<&str> = Some("Pick at least one part to generate.");
        assert_eq!(
            cast_hint(C {
                ready: true,
                has_rows: true,
                enough_parts: false
            }),
            PICK
        );
        assert_eq!(
            cast_hint(C {
                ready: true,
                has_rows: true,
                enough_parts: true
            }),
            None
        );
        assert_eq!(
            cast_hint(C {
                ready: true,
                has_rows: false,
                enough_parts: false
            }),
            None
        );
        assert_eq!(
            cast_hint(C {
                ready: true,
                has_rows: false,
                enough_parts: true
            }),
            None
        );
        assert_eq!(
            cast_hint(C {
                ready: false,
                has_rows: true,
                enough_parts: false
            }),
            None
        );
        assert_eq!(
            cast_hint(C {
                ready: false,
                has_rows: true,
                enough_parts: true
            }),
            None
        );
        assert_eq!(
            cast_hint(C {
                ready: false,
                has_rows: false,
                enough_parts: false
            }),
            None
        );
        assert_eq!(
            cast_hint(C {
                ready: false,
                has_rows: false,
                enough_parts: true
            }),
            None
        );

        use DropButton as D;
        const FLOOR: Option<&str> = Some("The cast needs at least one layer.");
        assert_eq!(
            drop_hint(D {
                ready: true,
                removable: false
            }),
            FLOOR
        );
        assert_eq!(
            drop_hint(D {
                ready: true,
                removable: true
            }),
            None
        );
        assert_eq!(
            drop_hint(D {
                ready: false,
                removable: false
            }),
            None
        );
        assert_eq!(
            drop_hint(D {
                ready: false,
                removable: true
            }),
            None
        );
    }

    /// The empty parts card's controls are DISABLED, not merely drawn.
    ///
    /// ⚠ The name census cannot tell: egui emits a disabled widget under the same
    /// label, so deleting `has_rows` left the suite green.
    #[test]
    fn the_empty_parts_card_offers_nothing_to_click() {
        let studio = Studio {
            cursor: WizardCursor::new(Step::MakeMolds),
            ..Studio::default()
        };

        for name in ["All", "None", "Make molds"] {
            let mut empty = MoldControls::default();
            assert_eq!(
                controls_disabled(
                    |ui| {
                        let _ = draw_make_molds(ui, &studio, true, &mut empty);
                    },
                    name
                ),
                [true],
                "{name} must be disabled with no design committed"
            );
        }

        // ...and live again once there is something to act on.
        let ready = crate::molds::tests::viewing_step_5_with(1);
        for name in ["All", "None", "Make molds"] {
            let mut molds = crate::molds::tests::controls_for(1);
            assert_eq!(
                controls_disabled(
                    |ui| {
                        let _ = draw_make_molds(ui, &ready, true, &mut molds);
                    },
                    name
                ),
                [false],
                "{name} must be live once a design offers parts"
            );
        }
    }

    /// ★★★ "Does anything CLICK it." Four defects in #885/#886 were all this
    /// question going unasked — the control drew, and nothing reached it.
    #[test]
    fn clicking_make_molds_in_the_running_wizard_starts_the_cast() {
        let mut app = app_running_the_wizard();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default());
        app.insert_resource(crate::molds::tests::viewing_step_5_with(1));
        app.insert_resource(crate::molds::tests::controls_for(1));

        click_on(&mut app, "Make molds");

        let studio = app.world().resource::<Studio>();
        assert!(
            studio.busy,
            "the click must reach `start_molds` and hold the app: {:?}",
            studio.outcome()
        );
        // ⚠ Exact, not `contains`. Seeding `shown_secs` and the opening line
        // from different seconds passed all 172 while the app announced a cast
        // that started this instant as already 0:07 elapsed.
        const OPENING_LINE: &str =
            "Making molds… 0:00 elapsed (this can take a while — the window stays responsive)";
        assert_eq!(
            studio.outcome().cloned(),
            Some(Ok(OPENING_LINE.to_string())),
            "and say so on screen, at the second the run actually started"
        );
    }

    /// ⚠ The disabled half of the same claim. `click_on` will happily "click" a
    /// disabled control and report success, so this asserts the EFFECT, not the
    /// click.
    #[test]
    fn make_molds_is_refused_when_no_part_is_checked() {
        let mut app = app_running_the_wizard();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default());
        app.insert_resource(crate::molds::tests::viewing_step_5_with(1));
        let mut molds = crate::molds::tests::controls_for(1);
        molds.picker.set_all(false);
        app.insert_resource(molds);

        click_on(&mut app, "Make molds");

        assert!(
            !app.world().resource::<Studio>().busy,
            "an empty selection must not start a half-hour run that meshes nothing"
        );
    }

    /// The All / None buttons, through the real screen.
    #[test]
    fn none_then_all_reaches_the_picker() {
        let mut app = app_running_the_wizard();
        app.insert_resource(crate::molds::tests::viewing_step_5_with(1));
        app.insert_resource(crate::molds::tests::controls_for(1));

        click_on(&mut app, "None");
        assert!(
            !app.world().resource::<MoldControls>().picker.any_checked(),
            "None must clear every box"
        );

        click_on(&mut app, "All");
        assert!(
            app.world()
                .resource::<MoldControls>()
                .picker
                .rows()
                .all(|(_, checked)| checked),
            "and All must put them back"
        );
    }

    /// ★ A checkbox toggles ITS OWN row. An off-by-one here would silently cast
    /// the wrong piece, and every other gate on this screen would stay green.
    #[test]
    fn a_part_checkbox_toggles_the_row_it_names() {
        let mut app = app_running_the_wizard();
        app.insert_resource(crate::molds::tests::viewing_step_5_with(1));
        app.insert_resource(crate::molds::tests::controls_for(1));

        click_on(&mut app, "Layer 1 — plug");

        let checked: Vec<(String, bool)> = app
            .world()
            .resource::<MoldControls>()
            .picker
            .rows()
            .map(|(label, checked)| (label.to_string(), checked))
            .collect();
        assert_eq!(
            checked
                .iter()
                .filter(|(_, checked)| !checked)
                .map(|(label, _)| label.as_str())
                .collect::<Vec<_>>(),
            ["Layer 1 — plug"],
            "exactly the row that was clicked came off: {checked:?}"
        );
    }

    /// ⚠⚠ The pairing `cell_size_m_for_quality`'s own doc said nothing checked.
    /// The labels and the cell sizes are two lists indexed by the same number,
    /// and swapping the labels would quietly cast a 36-minute run when the user
    /// asked for the 7-minute preview.
    #[test]
    fn the_quality_labels_match_the_cell_sizes_they_pick() {
        for (index, label) in QUALITY_LABELS.iter().enumerate() {
            let index = i32::try_from(index).expect("two labels");
            let mm = label
                .split_whitespace()
                .find_map(|word| word.parse::<f64>().ok());
            assert!(
                mm.is_some(),
                "the label must name its resolution, or this gate reads nothing: {label}"
            );
            if let Some(mm) = mm {
                assert!(
                    (cell_size_m_for_quality(index) - mm / 1000.0).abs() < f64::EPSILON,
                    "label {label:?} says {mm} mm but index {index} casts at {} m",
                    cell_size_m_for_quality(index),
                );
            }
        }
    }

    /// ★★★ The refusal reaches the SCREEN, on its own step and no other.
    ///
    /// ⚠ Without this the fix is only half gated. `Studio::note_for` is
    /// covered — cargo-mutants catches its `==`/`!=` — but nothing checked that
    /// the panel READS it: reverting this call site to the unfiltered note, the
    /// exact behaviour the fix removes, left all 332 tests green.
    ///
    /// Two-sided, and both halves are needed. Assert only presence and the
    /// unfiltered read passes; assert only absence and so does a panel that
    /// draws no message at all.
    #[test]
    fn a_refusal_is_painted_on_its_own_step_and_on_no_other() {
        // ⚠ A needle nothing else on the screen paints. Asserted absent on
        // another step below, which is what keeps it a needle rather than a
        // phrase the chrome supplies.
        const REFUSAL: &str = "the floor lock did not fuse";

        let mut app = app_running_the_wizard();
        let mut studio = crate::molds::tests::viewing_step_5_with(1);
        studio.say(Err(REFUSAL.to_string()));
        app.insert_resource(studio);
        settle(&mut app);
        assert!(
            painted_texts(&app)
                .iter()
                .any(|text| text.contains(REFUSAL)),
            "the step that refused must show it: {:?}",
            painted_texts(&app)
        );

        app.world_mut().resource_mut::<Studio>().back();
        settle(&mut app);
        assert!(
            !painted_texts(&app)
                .iter()
                .any(|text| text.contains(REFUSAL)),
            "and no other step may: {:?}",
            painted_texts(&app)
        );

        app.world_mut().resource_mut::<Studio>().next();
        settle(&mut app);
        assert!(
            painted_texts(&app)
                .iter()
                .any(|text| text.contains(REFUSAL)),
            "coming back, it is still there — paging hides a refusal, never \
             destroys it: {:?}",
            painted_texts(&app)
        );
    }

    /// The summary is derived from the project, not stored — so it must appear
    /// on a screen that was never the one the cast landed on.
    #[test]
    fn a_finished_cast_shows_its_summary_on_the_screen() {
        let mut app = app_running_the_wizard();
        let mut studio = crate::molds::tests::viewing_step_5_with(1);
        studio
            .project
            .set_molds(crate::jobs::tests::some_molds("out-panel"))
            .expect("the fixture records a cast");
        app.insert_resource(studio);

        settle(&mut app);

        // ⚠ One painted shape, not five. A multi-line galley carries its
        // newlines, so asserting on the first line alone would pass over a
        // card that had lost everything below it — which is what an earlier
        // read of this screen appeared to show, until the grep hiding the
        // continuation lines turned out to be the fault.
        let painted = painted_texts(&app);
        let card = painted
            .iter()
            .find(|text| text.contains("mold piece(s)"))
            .map_or("", String::as_str);
        for expected in [
            "✔ 1 mold piece(s) + 1 plug(s)",
            "Total silicone: 80 g across 2 pour(s):",
            "Layer 2: Ecoflex 00-30 — 40 g (pot life ~45 min)",
            "Saved to: out-panel",
        ] {
            assert!(
                card.contains(expected),
                "the summary card must carry {expected:?}, not just its first line: {card:?}"
            );
        }
    }

    /// Step 5 rendered for real, with `molds` as the step-5 controls.
    fn wizard_on_step_5(studio: Studio, molds: MoldControls) -> App {
        let mut app = app_running_the_wizard();
        app.insert_resource(studio);
        app.insert_resource(molds);
        settle(&mut app);
        app
    }

    /// ⚠ The button relabels itself for the length of the run — the pre-port
    /// screen's only progress indicator. Removing the branch left every other
    /// gate on this screen green.
    #[test]
    fn a_running_cast_relabels_the_button() {
        let studio = Studio {
            busy: true,
            ..crate::molds::tests::viewing_step_5_with(1)
        };
        let app = wizard_on_step_5(studio, crate::molds::tests::controls_for(1));

        let painted = painted_texts(&app);
        assert!(
            painted.iter().any(|text| text == "Making molds…"),
            "a cast in flight says so on the button: {painted:?}"
        );
        assert!(
            !painted.iter().any(|text| text == "Make molds"),
            "and the idle label is gone while it runs: {painted:?}"
        );
    }

    /// ⚠ The picker must show the choice it is ON, not always the first one.
    /// The cell size is read from the index separately, so a frozen label would
    /// cast at 1.5 mm while the screen still said 0.5.
    #[test]
    fn the_quality_picker_shows_the_choice_it_is_on() {
        let mut molds = crate::molds::tests::controls_for(1);
        molds.quality_idx = 1;
        let app = wizard_on_step_5(crate::molds::tests::viewing_step_5_with(1), molds);

        let painted = painted_texts(&app);
        assert!(
            painted.iter().any(|text| text == QUALITY_LABELS[1]),
            "the fast preview is what is selected: {painted:?}"
        );
        assert!(
            !painted.iter().any(|text| text == QUALITY_LABELS[0]),
            "and the fine label is not also showing: {painted:?}"
        );
    }

    /// An index no label exists for falls back to print quality, the same way
    /// `cell_size_m_for_quality` does — so the two cannot disagree about what
    /// an out-of-range index means.
    #[test]
    fn an_impossible_quality_index_reads_as_print_quality() {
        assert_eq!(quality_label(99), QUALITY_LABELS[0]);
        assert_eq!(quality_label(-1), QUALITY_LABELS[0]);
        assert!(
            (cell_size_m_for_quality(99) - 0.0005).abs() < f64::EPSILON,
            "and the cast agrees with the label"
        );
    }

    /// ⚠ The state a user reaches by paging back to step 4 and dropping the
    /// design. Without the hint the card is an empty grey box with two buttons
    /// that do nothing.
    #[test]
    fn an_empty_parts_card_says_where_the_design_comes_from() {
        // ⚠ Step 5 with nothing committed — NOT `Studio::default()`, which is
        // parked on step 1 and would render a different screen entirely.
        let studio = Studio {
            cursor: WizardCursor::new(Step::MakeMolds),
            ..Studio::default()
        };
        let app = wizard_on_step_5(studio, MoldControls::default());

        let painted = painted_texts(&app);
        assert!(
            painted
                .iter()
                .any(|text| text.contains("Choose a design in step 4 first")),
            "the empty card explains itself: {painted:?}"
        );
    }

    /// ⚠ The other half of the summary claim. A card that always draws would
    /// pass `a_finished_cast_shows_its_summary_on_the_screen` just as happily,
    /// and would show a stale or empty result before any cast had run.
    #[test]
    fn no_summary_card_before_the_first_cast() {
        let app = wizard_on_step_5(
            crate::molds::tests::viewing_step_5_with(1),
            crate::molds::tests::controls_for(1),
        );

        let painted = painted_texts(&app);
        assert!(
            !painted.iter().any(|text| text.contains("mold piece(s)")),
            "nothing has been cast, so there is no summary to show: {painted:?}"
        );
    }

    /// The wizard driven from the first screen to the last.
    ///
    /// ⚠ The real `Next →`, not the cursor: `can_next` is
    /// `project.is_complete(viewed)`, so moving the cursor pages over the gate.
    /// Unwritable until step 5 could complete.
    #[test]
    fn the_wizard_pages_from_the_first_screen_to_the_last() {
        use cf_studio_core::PrintExport;

        let mut studio = crate::molds::tests::viewing_step_5_with(1);
        studio
            .project
            .set_molds(crate::jobs::tests::some_molds("out-walk"))
            .expect("the cast is recorded");
        studio
            .project
            .set_print(PrintExport {
                export_dir: "out-walk".into(),
            })
            .expect("the print is recorded");
        studio.cursor = WizardCursor::new(Step::FIRST);

        let mut app = app_running_the_wizard();
        app.insert_resource(studio);
        app.insert_resource(crate::molds::tests::controls_for(1));

        for step in Step::ALL {
            settle(&mut app);
            let heading = format!("Step {} of 7 — {}", step.number(), step.title());
            let painted = painted_texts(&app);
            assert!(
                painted.iter().any(|text| text == &heading),
                "the walk must reach {heading:?}: {painted:?}"
            );
            if step != Step::LAST {
                click_on(&mut app, "Next →");
            }
        }

        assert_eq!(
            app.world().resource::<Studio>().cursor.viewed(),
            Step::LAST,
            "six clicks of Next must land on the pour screen"
        );
    }

    /// ⚠ Step 2's earlier states. Each shows text that exists in no other one —
    /// the hint naming the step that unblocks Save, and the line telling you to
    /// add a scan at all — so censusing only the revealed screen leaves both of
    /// them, and the controls beside them, ungated.
    #[test]
    fn every_earlier_state_of_the_cleanup_screen_is_laid_out_too() {
        let mut empty = CleanupScreen {
            scan: ScanEdit::default(),
            ..cleanup_screen()
        };
        assert!(
            controls_in_column(|ui| {
                let _ = draw_clean_scan(ui, true, &empty.scan, &mut empty.controls);
            })
            .is_empty(),
            "with no scan there is nothing to offer but the line saying so"
        );

        let mut screen = cleanup_screen();
        screen.scan.set(ActiveScan::synthetic(open_tube()));

        let controls = controls_in_column(|ui| {
            let _ = draw_clean_scan(ui, true, &screen.scan, &mut screen.controls);
        });

        assert_eq!(
            controls,
            [
                "Weld points",
                "−",
                "TextInput",
                "+",
                "Simplify",
                "Find floor",
                "−",
                "TextInput",
                "+",
                "Save cleaned scan",
                "Start over",
            ]
        );
    }

    /// Whether step 2's Save is disabled, for a screen that has — or has not —
    /// been stood up, against an app that is — or is not — accepting actions.
    fn save_button_disabled(screen: &mut CleanupScreen, ready: bool) -> Vec<bool> {
        controls_disabled(
            |ui| {
                let _ = draw_clean_scan(ui, ready, &screen.scan, &mut screen.controls);
            },
            "Save cleaned scan",
        )
    }

    /// Whether each control called `name` is disabled, in layout order.
    ///
    /// ⚠ The order is the gate. Six identical `+` buttons say nothing about
    /// which feature owns which — only their positions do.
    ///
    /// ⚠ The accessibility tree's own flag, and the count is half the answer.
    /// A gated control is on screen either way — that is how the user is told
    /// what to do first — so a control that vanished must not read as one that
    /// was refused.
    fn controls_disabled(mut body: impl FnMut(&mut egui::Ui), name: &str) -> Vec<bool> {
        use egui_kittest::kittest::NodeT;

        column_harness(&mut body)
            .root()
            .children_recursive()
            .filter_map(|node| {
                let widget = node.accesskit_node();
                (control_name(widget.role(), widget.label())? == name).then(|| widget.is_disabled())
            })
            .collect()
    }

    /// `body`, laid out and run in the column the wizard builds.
    ///
    /// ⚠ The fonts that ship, so what these gates measure — and check for
    /// glyphs — is what the app actually draws.
    fn column_harness<'a>(body: &'a mut dyn FnMut(&mut egui::Ui)) -> Harness<'a> {
        let mut harness = Harness::builder()
            .with_size(egui::Vec2::new(BODY_WIDTH, COLUMN_HEIGHT))
            .build(|ctx| body_column(ctx, &mut *body));
        harness.ctx.set_fonts(crate::plugin::font_definitions());
        harness.run();
        harness
    }

    /// ★ Save is gated on the centerline *and* on the app being free — both
    /// terms, because a test that only stands the scan up passes just as well
    /// with the `ready` half deleted.
    ///
    /// ⚠ `EditSession::save` refuses without a centerline, so an enabled button
    /// there buys the user a click and a "Save failed" for it; and a Save that
    /// ran while something held the app would write while the question that
    /// gated it is still on screen.
    #[test]
    fn save_is_offered_only_once_the_scan_is_stood_up_and_the_app_is_free() {
        assert_eq!(
            save_button_disabled(&mut cleanup_screen(), true),
            [false],
            "stood up, nothing else running"
        );

        let mut flat = cleanup_screen();
        // Back to a freshly loaded scan: no centerline, so no cast frame.
        flat.scan.set(ActiveScan::synthetic(open_tube()));
        assert_eq!(
            save_button_disabled(&mut flat, true),
            [true],
            "not yet stood up"
        );

        assert_eq!(
            save_button_disabled(&mut cleanup_screen(), false),
            [true],
            "stood up, but something holds the app"
        );
    }

    /// Lay the save row out with `controls`, click `label`, and report what the
    /// row itself said — the payload the click carries, not the field behind it.
    fn save_row_after(label: &str, controls: &mut EditControls) -> Option<usize> {
        let reported = std::cell::Cell::new(None);
        {
            let borrowed = std::cell::RefCell::new(&mut *controls);
            let mut body = |ui: &mut egui::Ui| {
                if let Some(smoothing) = draw_save_row(ui, &mut borrowed.borrow_mut(), true) {
                    reported.set(Some(smoothing));
                }
            };
            let mut harness = column_harness(&mut body);
            harness.get_by_label(label).click();
            harness.run();
        }
        reported.get()
    }

    /// ★ The click has to carry the number on screen. Every other save gate
    /// passes with the button sending a constant, the field pinned by its own
    /// bounds, or `smoothing_iters` answering 0.
    ///
    /// ⚠ One `+` is one more pass — the decision `SMOOTHING_STEP` records, and
    /// the reason it is not the face target's step.
    #[test]
    fn the_save_button_carries_the_smoothing_the_stepper_shows() {
        let shown = EditControls::default().smoothing_iters();

        let mut controls = EditControls::default();
        assert_eq!(
            save_row_after("Save cleaned scan", &mut controls),
            Some(shown),
            "the click carries what the field shows"
        );

        let mut controls = EditControls::default();
        assert_eq!(
            save_row_after("+", &mut controls),
            None,
            "stepping the field is not a save"
        );
        assert_eq!(
            save_row_after("Save cleaned scan", &mut controls),
            Some(shown + 1),
            "and the click after one + carries one more pass"
        );
    }

    /// Stand `wizard_screen` up as the Bevy system it is, with the egui pass
    /// `bevy_egui`'s plugin would normally open around it.
    fn app_running_the_wizard() -> App {
        let mut app = egui_harness::app();
        app.init_resource::<Studio>()
            .init_resource::<PendingDialog>()
            .init_resource::<Autosave>()
            .init_resource::<ScanEdit>()
            .init_resource::<EditControls>()
            .init_resource::<ShapeControls>()
            .init_resource::<DesignControls>()
            .init_resource::<MoldControls>()
            .init_resource::<SimplifyJob>()
            .init_resource::<MoldsJob>()
            .init_resource::<PlugFitJob>()
            .init_resource::<PlugView>()
            .add_systems(Update, (begin, wizard_screen, end).chain());
        app
    }

    /// The wizard on step 3 with the preview driver running behind it, against
    /// the cleaned scan `prep` names.
    fn wizard_previewing(prep: cf_studio_core::PrepInput) -> App {
        let mut app = app_running_the_wizard();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default())
            .add_systems(Update, crate::preview::drive_plug_preview.before(begin));
        app.insert_resource(Studio {
            project: crate::preview::tests::cleaned(prep),
            cursor: WizardCursor::new(Step::ShapePiece),
            ..Studio::default()
        });
        crate::preview::tests::settle(&mut app);
        // One more frame, so the panel draws against the settled preview rather
        // than against the state of the one before it.
        app.update();
        app
    }

    /// The honesty gate, driven from the resource rather than an argument.
    #[test]
    fn a_stand_in_preview_says_so_and_a_real_one_does_not() {
        let mut absent = wizard_previewing(crate::preview::tests::a_missing_scan());
        let mut real = wizard_previewing(crate::preview::tests::a_cleaned_scan("panel-note"));

        // ⚠⚠ This crate's OWN table, written independently of `scene.rs`'s.
        // Sharing one made the two gates a single oracle: flipping
        // `shows_the_piece` and the shared row together — a one-file edit —
        // shipped step 5 casting a piece it never displays, with 171 green.
        // Two tables that must be edited in step is the cross-check.
        let expected: [(Step, bool); Step::TOTAL] = [
            (Step::AddScan, false),
            (Step::CleanScan, false),
            (Step::ShapePiece, true),
            (Step::DesignLayers, true),
            (Step::MakeMolds, true),
            (Step::Print, false),
            (Step::Pour, false),
        ];
        // ⚠ Counting to `Step::TOTAL` is not coverage: a duplicated row and a
        // missing one also make seven.
        assert_eq!(
            expected.map(|(step, _)| step),
            Step::ALL,
            "the table must answer for every step, in order"
        );
        for (step, piece_step) in expected {
            for app in [&mut absent, &mut real] {
                app.world_mut().resource_mut::<Studio>().cursor = WizardCursor::new(step);
                settle(app);
            }

            let said = painted_texts(&absent).join(" ");
            let unsaid = painted_texts(&real).join(" ");

            assert_eq!(
                said.contains("stand-in"),
                piece_step,
                "step {} shows the piece: {piece_step} — the note must match: {said:?}",
                step.number()
            );
            assert!(
                !unsaid.contains("stand-in"),
                "and step {} must not disown a scan it could: {unsaid:?}",
                step.number()
            );
        }
        let _ = std::fs::remove_dir_all(crate::preview::tests::fixture_dir("panel-note"));
    }

    /// ★★★ The thrash guard. The driver runs behind a screen that redraws sixty
    /// times a second, and everything it decides from — the draft the fields
    /// describe, the cleaned scan on disk — has to read the same each time. Any
    /// one of them flickering costs a flood-filled SDF per frame, for ever, and
    /// on a real scan that is 191 ms of it.
    #[test]
    fn redrawing_the_wizard_does_not_rebuild_the_preview() {
        let mut app = wizard_previewing(crate::preview::tests::a_missing_scan());
        let settled = app.world().resource::<PlugView>().generation();
        assert!(settled > 0, "a piece has been meshed to hold still");

        for _ in 0..10 {
            app.update();
        }

        assert_eq!(
            app.world().resource::<PlugView>().generation(),
            settled,
            "drawing the screen must not re-mesh the piece"
        );
    }

    /// The wizard parked on step 3, with a scan cleaned behind it so
    /// [`Project::set_plug`] will take.
    fn wizard_on_step_three() -> App {
        let mut app = app_running_the_wizard();
        app.insert_resource(Studio {
            project: crate::shape::tests::ready_to_shape(),
            cursor: WizardCursor::new(Step::ShapePiece),
            ..Studio::default()
        });
        app
    }

    /// ★ `wizard_screen` is the system every click reaches the app through, and
    /// replacing it with a no-op passed everything: drawing leaves nothing in
    /// the ECS to observe.
    #[test]
    fn the_wizard_runs_as_a_system_and_paints_its_three_panels() {
        let mut app = app_running_the_wizard();

        settle(&mut app);

        let painted = painted_texts(&app);
        let shows = |needle: &str| painted.iter().any(|text| text.contains(needle));
        assert!(shows("Step 1 of 7"), "the body column: {painted:?}");
        assert!(shows("Next"), "the footer nav: {painted:?}");
        assert!(shows("1. Add your scan"), "and the checklist: {painted:?}");
    }

    /// ★★ Save's own wiring, which nothing else reaches. `draw_save_modal`,
    /// `apply_save_choice` and `save_to_default` are each driven directly — but
    /// deleting either call site from `wizard_screen` left the whole suite
    /// green, because a call site is not a function anyone can call.
    ///
    /// ⚠ Clicked through the real system: the button is painted on one frame
    /// and the pointer lands on it the next, exactly as a person produces it.
    #[test]
    fn clicking_save_in_the_running_wizard_writes_the_files() {
        let dir = crate::save::tests::temp_dir("through-the-wizard");
        let (scan, studio) = crate::save::tests::ready_to_save(&dir);
        let mut app = app_running_the_wizard();
        app.insert_resource(scan);
        app.insert_resource(studio);
        // Step 1 is complete the moment a scan is recorded, so this lands on 2.
        app.world_mut().resource_mut::<Studio>().next();

        click_on(&mut app, "Save cleaned scan");

        let studio = app.world().resource::<Studio>();
        assert!(
            dir.join("base.cleaned.stl").is_file() && studio.project.prep().is_some(),
            "the click wrote the files and completed the step: {:?}",
            studio.outcome()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The wizard with a saved session offered back, and what it is offering.
    fn wizard_asking_to_resume() -> (App, Project) {
        let saved = ready_to_pour();
        let reached = saved
            .furthest_completed()
            .expect("the fixture is a walked project");
        let mut app = app_running_the_wizard();
        app.insert_resource(Autosave::asking(saved.clone(), reached));
        (app, saved)
    }

    /// ⚠ The modal is drawn from the autosave in `wizard_screen` alone. Delete
    /// that block and the session sits with writing suspended, the question
    /// that suspended it nowhere on screen, and no test the wiser.
    #[test]
    fn a_saved_session_puts_its_question_on_screen() {
        let (mut app, saved) = wizard_asking_to_resume();

        settle(&mut app);

        let painted = painted_texts(&app);
        let reached = saved.furthest_completed().expect("a walked project");
        // ⚠ The whole question, not the step title inside it: the checklist
        // paints all seven titles on every frame, so a `contains(title)` here
        // passes with this modal deleted.
        assert!(
            painted.contains(&format_resume_question(reached)),
            "the modal draws the question, about the step reached: {painted:?}"
        );
        assert!(
            painted
                .iter()
                .any(|text| text.contains("Pick up where I left off")),
            "and offers both answers: {painted:?}"
        );
    }

    /// ★★ The answers' own wiring. `draw_resume_modal` reports a choice and
    /// `apply_resume_choice` executes one, but a call site is not a function
    /// anyone can call — and this is the click that replaces the whole project.
    #[test]
    fn picking_up_in_the_running_wizard_takes_the_saved_session() {
        let (mut app, saved) = wizard_asking_to_resume();

        click_on(&mut app, "Pick up where I left off");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project,
            saved,
            "the click landed the saved session: {:?}",
            studio.outcome()
        );
        assert_eq!(
            app.world().resource::<Autosave>().asking_about(),
            None,
            "and answered the question"
        );
    }

    /// ⚠ The other answer, through the same modal. Wiring both buttons to the
    /// same branch is a one-character mistake, and the one that resumes is the
    /// one that would silently discard this session instead.
    #[test]
    fn starting_over_in_the_running_wizard_keeps_this_sessions_project() {
        let (mut app, saved) = wizard_asking_to_resume();
        let mine = app.world().resource::<Studio>().project.clone();
        assert_ne!(mine, saved, "the two must differ, or neither answer shows");

        click_on(&mut app, "Start over from this scan");

        assert_eq!(
            app.world().resource::<Studio>().project,
            mine,
            "this session's project stands"
        );
        assert_eq!(
            app.world().resource::<Autosave>().asking_about(),
            None,
            "and the question is answered either way"
        );
    }

    /// ★★★ `accepting_actions` is gated on its own, and every screen is gated
    /// on the `bool` it is handed — but nothing said `wizard_screen` hands them
    /// the real one. Replaced by `true`, every control on every screen goes
    /// live in the middle of a job, and both of those gates stay green.
    #[test]
    fn the_wizard_hands_its_screens_the_real_gate() {
        let mut app = wizard_on_step_three();
        app.world_mut().resource_mut::<Studio>().busy = true;

        click_on(&mut app, "Continue");

        assert_eq!(
            app.world().resource::<Studio>().project.plug(),
            None,
            "a job is running, so Continue must not commit"
        );
    }

    /// ⚠ The note has to be on the screen the user is looking at, whichever one
    /// that is — `Studio::message` is wiped by the next step action, and this
    /// says their work is not being saved.
    #[test]
    fn a_session_that_cannot_save_says_so_on_the_screen() {
        let dir = crate::save::tests::temp_dir("cannot-save");
        let scan = dir.join("base.stl");
        std::fs::write(
            cf_studio_gui::autosave_path(&scan),
            b"written by something else",
        )
        .expect("a file this build cannot read");
        let mut autosave = Autosave::default();
        autosave.follow(&scan);
        let mut app = app_running_the_wizard();
        app.insert_resource(autosave);

        settle(&mut app);

        let painted = painted_texts(&app);
        assert!(
            painted
                .iter()
                .any(|text| text.contains("isn't being saved")),
            "the screen says the work is not being saved: {painted:?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ The modal is drawn from `pending_save` in `wizard_screen` alone. Delete
    /// that block and every control stays gated on a question with nothing on
    /// screen to answer — the app inert, and no test the wiser.
    #[test]
    fn a_held_save_puts_its_question_on_screen() {
        let dir = crate::save::tests::temp_dir("held-save");
        let (scan, mut studio) = crate::save::tests::ready_to_save(&dir);
        studio.pending_save = Some(PendingSave::Confirming {
            dir: dir.clone(),
            smoothing: 0,
        });
        let mut app = app_running_the_wizard();
        app.insert_resource(scan);
        app.insert_resource(studio);

        settle(&mut app);

        let painted = painted_texts(&app);
        assert!(
            painted.iter().any(|text| text.contains("already exist")),
            "the question is on screen: {painted:?}"
        );
        assert!(
            painted.iter().any(|text| text == "Overwrite"),
            "and so are its answers: {painted:?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★ The glue between two halves each well tested on its own: the modal
    /// returns the right answer, and `save` does the right thing — but nothing
    /// said the answer reaches the action it names. Miswiring any of the three
    /// passed the entire suite.
    ///
    /// ⚠ `ChooseFolder` is checked by the state it leaves, not by the picker
    /// opening: `pick_folder` would put a real OS dialog on screen, so the
    /// dialog here is already open and its call is a no-op. That the call
    /// exists at all is the one part of this arm a hand test still owns.
    #[test]
    fn each_modal_answer_reaches_the_action_it_names() {
        let dir = crate::save::tests::temp_dir("answers");
        let question = std::fs::read_to_string(dir.join("base.cleaned.stl"));
        assert!(question.is_err(), "the folder starts empty");

        let (scan, mut studio) = crate::save::tests::ready_to_save(&dir);
        let mut dialog = PendingDialog::default();
        apply_save_choice(
            SaveChoice::Overwrite,
            &dir,
            0,
            &scan,
            &mut studio,
            &mut dialog,
        );
        assert!(
            dir.join("base.cleaned.stl").is_file() && studio.project.prep().is_some(),
            "Overwrite writes and completes the step: {:?}",
            studio.outcome()
        );

        let (scan, mut studio) = crate::save::tests::ready_to_save(&dir);
        std::fs::write(dir.join("base.cleaned.stl"), b"keep me").expect("a decoy");
        apply_save_choice(SaveChoice::Cancel, &dir, 0, &scan, &mut studio, &mut dialog);
        assert_eq!(
            std::fs::read(dir.join("base.cleaned.stl")).expect("still there"),
            b"keep me",
            "Cancel must not write — miswired, it overwrites what the user kept"
        );
        assert!(studio.pending_save.is_none(), "and it hands the app back");

        let (scan, mut studio) = crate::save::tests::ready_to_save(&dir);
        let mut open = PendingDialog::opened(DialogKind::PrepDest);
        apply_save_choice(
            SaveChoice::ChooseFolder,
            &dir,
            7,
            &scan,
            &mut studio,
            &mut open,
        );
        assert_eq!(
            studio.pending_save,
            Some(PendingSave::ChoosingFolder { smoothing: 7 }),
            "a folder answer waits for the folder, carrying the smoothing"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★★ Step 3's own wiring, which nothing else reaches. `draw_shape_piece`
    /// hands back a draft and `commit_plug` applies one, but a call site is not
    /// a function anyone can call — deleting either left the suite green.
    ///
    /// ⚠ Clicked through one `+` first, so the button has to carry the number
    /// on screen. Sending a constant, or reading a field pinned by its own
    /// default, passes every other gate step 3 has.
    #[test]
    fn clicking_continue_in_the_running_wizard_shapes_the_piece() {
        let mut app = wizard_on_step_three();

        click_on(&mut app, "+");
        click_on(&mut app, "Continue");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project.plug().map(|plug| plug.cavity_inset_m),
            Some(0.006),
            "the 5 mm field, stepped once, committed 6 mm: {:?}",
            studio.outcome()
        );
        assert_eq!(
            studio.cursor.viewed(),
            Step::DesignLayers,
            "and Continue moved on"
        );

        settle(&mut app);
        // ★ The end of the trap `commit_plug` guards. `Studio::next` clears the
        // message, so reporting before advancing lands the user on step 4 with
        // nothing on it — which only the screen itself can show.
        let painted = painted_texts(&app);
        assert!(
            painted.iter().any(|text| text.contains("Step 4 of 7")),
            "the screen moved on with the cursor: {painted:?}"
        );
        assert!(
            painted.iter().any(|text| text.contains("Shaped piece")),
            "and carried the report onto it: {painted:?}"
        );
    }

    /// ⚠ The other half of the same claim: step 4 must have *stopped* saying
    /// it. A screen that draws its new controls under the notice still reads
    /// as unported, and every gate below counts controls, not what they sit
    /// beneath.
    #[test]
    fn the_layer_screen_no_longer_says_it_is_being_rebuilt() {
        let mut app = app_running_the_wizard();
        app.insert_resource(Studio {
            cursor: WizardCursor::new(Step::DesignLayers),
            ..Studio::default()
        });

        settle(&mut app);

        let painted = painted_texts(&app);
        assert!(
            !painted.iter().any(|text| text.contains("being rebuilt")),
            "step 4 is ported: {painted:?}"
        );
    }

    /// The ridge editor's master switch, by the name the census gives it.
    const MASTER_SWITCH: &str = "Add surface ridges (advanced)";

    /// The fit check's button, idle.
    const CHECK_FIT: &str = "Check fit";

    /// Step 3 with the ridge editor asked for — the state everything below
    /// this measures.
    fn ridges_on() -> ShapeControls {
        let mut shape = ShapeControls::default();
        shape.ridges.enabled = true;
        shape
    }

    /// The cell size these gates ask at.
    const STEP_THREE_CELL_M: f64 = 0.0005;

    /// The step-3 body for `shape`, on an app holding nothing and with no fit
    /// check asked for.
    fn shape_body(shape: &mut ShapeControls) -> impl FnMut(&mut egui::Ui) + '_ {
        move |ui| {
            let _ = draw_shape_piece(
                ui,
                &Studio::default(),
                true,
                shape,
                &PlugFitJob::default(),
                STEP_THREE_CELL_M,
            );
        }
    }

    /// The three controls one stepper puts on screen.
    const STEPPER: [&str; 3] = ["−", "TextInput", "+"];

    /// One ring's card: the ✖ that drops it, then its three steppers.
    const RING_CARD: [&str; 10] = [
        "✖",
        "−",
        "TextInput",
        "+",
        "−",
        "TextInput",
        "+",
        "−",
        "TextInput",
        "+",
    ];

    /// ⚠ #878's overflow is what stacking three ring cards and a grid into a
    /// 404 px column risks, and the editor is hidden until it is asked for —
    /// so the census of the opening screen reaches none of these.
    ///
    /// ⚠ Assembled from named blocks rather than derived from the screen: this
    /// is the list a person checks against the pre-port editor, and a block
    /// that moved, vanished or arrived twice changes it.
    #[test]
    fn the_revealed_ridge_editor_is_laid_out_inside_the_body_column_too() {
        let mut revealed = ridges_on();

        let controls = controls_in_column(shape_body(&mut revealed));

        assert_eq!(
            controls,
            [
                &STEPPER[..],
                &[CHECK_FIT],
                &[MASTER_SWITCH],
                &["Grip rings"],
                &RING_CARD,
                &RING_CARD,
                &RING_CARD,
                &["+ Add ring"],
                &["CheckBox"],
                &STEPPER,
                // Texture spacing, governed by the switch above it.
                &STEPPER,
                &["CheckBox"],
                &STEPPER,
                &["CheckBox"],
                &STEPPER,
                &["CheckBox"],
                &STEPPER,
                &["Continue"],
            ]
            .concat(),
            "the cavity field, the rings the SDK ships, then four switched rows \
             and one governed by the switch above it"
        );
    }

    /// Where each stepper's field is laid out, in the order they are drawn.
    fn stepper_rects(mut body: impl FnMut(&mut egui::Ui)) -> Vec<egui::Rect> {
        use egui::accesskit::Role;
        use egui_kittest::kittest::NodeT;

        column_harness(&mut body)
            .root()
            .children_recursive()
            .filter(|node| node.accesskit_node().role() == Role::TextInput)
            .map(|node| node.rect())
            .collect()
    }

    /// The steppers in the ring cards: three rings of three fields.
    const RING_STEPPERS: usize = 9;

    /// ⚠ The scalar row with no switch of its own carries a blank cell where
    /// the others carry a checkbox. Drop it and every cell after it shifts
    /// left on that row alone — a field that still fits, still works, and
    /// lines up with nothing.
    ///
    /// ⚠ The ring cards are checked as one group, not each card alone: three
    /// cards that each line up internally but sit at three different insets is
    /// the same defect one step out.
    #[test]
    fn every_field_lines_up_with_the_others_in_its_grid() {
        let mut revealed = ridges_on();

        let rects = stepper_rects(shape_body(&mut revealed));

        // The first is the cavity's, which sits above the editor entirely.
        let (_, editor) = rects.split_first().expect("step 3 has fifteen fields");
        let (rings, scalars) = editor.split_at(RING_STEPPERS);
        assert_eq!(scalars.len(), 5, "one field per ridge scalar: {rects:?}");

        for (grid, fields) in [("the ring cards", rings), ("the ridge grid", scalars)] {
            assert!(
                fields.iter().all(|rect| rect.left() == fields[0].left()),
                "{grid}: the fields must share a column: {fields:?}"
            );
        }
    }

    // The switch each of step 3's steppers is gated by, beyond the app being
    // free. The cavity field has none.
    const UNGATED: &str = "";
    const RINGS: &str = "grip rings";
    const TEXTURE: &str = "surface texture";
    const SIDE_PINCH: &str = "side pinch";
    const TIP_RELIEF: &str = "tip relief";
    const ORIENTATION: &str = "orientation";

    /// Step 3's fields, in the order their steppers are laid out, each with
    /// the switch that has to be on for it.
    const FIELDS: [(fn(&mut ShapeControls) -> &mut BoundedField, &str); 15] = [
        (|c| &mut c.cavity_mm, UNGATED),
        (|c| &mut c.ridges.rings[0].position, RINGS),
        (|c| &mut c.ridges.rings[0].depth, RINGS),
        (|c| &mut c.ridges.rings[0].width, RINGS),
        (|c| &mut c.ridges.rings[1].position, RINGS),
        (|c| &mut c.ridges.rings[1].depth, RINGS),
        (|c| &mut c.ridges.rings[1].width, RINGS),
        (|c| &mut c.ridges.rings[2].position, RINGS),
        (|c| &mut c.ridges.rings[2].depth, RINGS),
        (|c| &mut c.ridges.rings[2].width, RINGS),
        (|c| &mut c.ridges.texture_depth, TEXTURE),
        (|c| &mut c.ridges.texture_spacing, TEXTURE),
        (|c| &mut c.ridges.side_pinch, SIDE_PINCH),
        (|c| &mut c.ridges.tip_relief, TIP_RELIEF),
        (|c| &mut c.ridges.orientation, ORIENTATION),
    ];

    /// Every switch inside the ridge editor, and the flag it drives.
    const SWITCHES: [(&str, fn(&mut RidgeFields) -> &mut bool); 5] = [
        (RINGS, |r| &mut r.rings_enabled),
        (TEXTURE, |r| &mut r.texture_enabled),
        (SIDE_PINCH, |r| &mut r.side_pinch_enabled),
        (TIP_RELIEF, |r| &mut r.tip_relief_enabled),
        (ORIENTATION, |r| &mut r.orientation_enabled),
    ];

    /// ★ Five switches over fifteen steppers, all of them drawn by two
    /// helpers: a row handed its neighbour's switch gates the wrong field, and
    /// the screen reads the same either way.
    ///
    /// ⚠ Texture spacing is the row with no switch of its own, so it is the
    /// one a "every row owns its checkbox" port silently leaves live.
    ///
    /// ⚠ The switch has to gate something. A name in [`FIELDS`] that matches
    /// nothing in [`SWITCHES`] expects every stepper live — which is what a
    /// screen ignoring the switch entirely shows.
    #[test]
    fn each_ridge_toggle_gates_its_own_steppers() {
        let mut everything = ridges_on();
        assert_eq!(
            controls_disabled(shape_body(&mut everything), "+"),
            vec![false; FIELDS.len()],
            "with every feature on, every stepper is live"
        );

        for (name, switch) in SWITCHES {
            let mut off = ridges_on();
            *switch(&mut off.ridges) = false;
            let expected: Vec<bool> = FIELDS.iter().map(|(_, gate)| *gate == name).collect();

            assert!(expected.contains(&true), "{name} gates no stepper at all");
            assert_eq!(
                controls_disabled(shape_body(&mut off), "+"),
                expected,
                "{name} switched off gates its own steppers and no others"
            );
        }
    }

    /// Lay step 3 out with `shape` and click the `nth` button called `name`.
    fn click_nth(shape: &mut ShapeControls, name: &str, nth: usize) {
        let mut body = shape_body(shape);
        let mut harness = column_harness(&mut body);
        harness
            .get_all_by_label(name)
            .nth(nth)
            .expect("every stepper offers both buttons")
            .click();
        harness.run();
    }

    /// ★★ The screen is drawn with a range and the plug is read with one, and
    /// only this says they are the same range. Handed another field's bounds a
    /// stepper walks past its own limit, `plug_draft` clamps it back, and the
    /// screen shows one number while the plug carries another.
    ///
    /// ⚠ Driven from outside the bound rather than clicked up to it — texture
    /// spacing's range is 290 steps wide.
    #[test]
    fn every_stepper_stops_at_the_bound_its_field_commits_at() {
        for (nth, (pick, _)) in FIELDS.into_iter().enumerate() {
            let mut probe = ridges_on();
            let (min, max) = pick(&mut probe).range;

            for (typed, button, bound) in [(max + 50, "−", max), (min - 50, "+", min)] {
                let mut shape = ridges_on();
                let field = pick(&mut shape);
                *field.state.text_mut() = typed.to_string();
                field.state.on_typed();

                click_nth(&mut shape, button, nth);

                assert_eq!(
                    pick(&mut shape).state.value(),
                    bound,
                    "stepper {nth} was drawn with bounds other than its own"
                );
            }
        }
    }

    /// Every stepper's value on step 3, in the order they are laid out.
    fn stepper_values(shape: &mut ShapeControls) -> Vec<i32> {
        FIELDS
            .into_iter()
            .map(|(pick, _)| pick(shape).value())
            .collect()
    }

    /// ★★ Fifteen steppers from two helpers. A row bound to its neighbour's
    /// field moves the wrong number, and every value on screen stays
    /// plausible.
    #[test]
    fn each_stepper_on_step_three_moves_its_own_field_and_no_other() {
        let opening = stepper_values(&mut ridges_on());

        for nth in 0..opening.len() {
            let mut stepped = ridges_on();
            click_nth(&mut stepped, "+", nth);

            let expected: Vec<i32> = opening
                .iter()
                .enumerate()
                .map(|(i, value)| value + i32::from(i == nth))
                .collect();
            assert_eq!(
                stepper_values(&mut stepped),
                expected,
                "stepper {nth} moved something other than its own field"
            );
        }
    }

    /// ★★ Each ✖ drops the ring it sits in, and takes that ring's own state
    /// with it — uncommitted text included. Per-row state held in a `Vec`
    /// keyed by index leaves the edit bound to whatever is now that index,
    /// which is why [`cf_studio_gui::StepBoxState`] lives inside [`RingRow`].
    ///
    /// ⚠ Every ✖, not just the first: three identical buttons say nothing
    /// about which row each drops, and a remove that always took row 1 passes
    /// the whole suite on the first one.
    #[test]
    fn each_cross_drops_the_ring_it_sits_in_and_the_edit_typed_into_it() {
        let typed = ["71", "72", "73"];

        for dropped in 0..typed.len() {
            let mut shape = ridges_on();
            for (ring, mark) in shape.ridges.rings.iter_mut().zip(typed) {
                *ring.position.state.text_mut() = mark.to_owned();
                ring.position.state.on_typed();
            }

            click_nth(&mut shape, "✖", dropped);

            let left: Vec<&str> = shape
                .ridges
                .rings
                .iter()
                .map(|ring| ring.position.state.text())
                .collect();
            let expected: Vec<&str> = typed
                .iter()
                .enumerate()
                .filter_map(|(row, mark)| (row != dropped).then_some(*mark))
                .collect();
            assert_eq!(
                left,
                expected,
                "the ✖ in ring {} dropped another row, or left that row's text behind",
                dropped + 1
            );
        }
    }

    /// ★ The rings own two controls that are not steppers, and the sweep above
    /// reaches neither. Handed `ready` in place of the switch, the ✖ and the
    /// button stay live with the rings off — editing a set the plug will not
    /// carry.
    #[test]
    fn the_ring_switch_gates_the_controls_that_are_not_steppers() {
        for (name, count) in [("✖", 3), ("+ Add ring", 1)] {
            let mut on = ridges_on();
            assert_eq!(
                controls_disabled(shape_body(&mut on), name),
                vec![false; count],
                "{name} is offered with the rings on"
            );

            let mut off = ridges_on();
            off.ridges.rings_enabled = false;
            assert_eq!(
                controls_disabled(shape_body(&mut off), name),
                vec![true; count],
                "{name} must be withheld with the rings off"
            );
        }
    }

    /// ★★ The ring editor's own wiring, end to end: the buttons change the
    /// rows, the rows reach `plug_draft`, and the draft is what Continue
    /// commits.
    ///
    /// ⚠ Through 4c-2a the screen committed `RidgeOptions::default().rings`
    /// whatever the editor showed, and every gate on this screen passed.
    ///
    /// ⚠ One added and one dropped, so the editor ends on three rows — a
    /// fourth pushes Continue past the bottom of the window, where the
    /// harness cannot reach it.
    #[test]
    fn rings_edited_in_the_running_wizard_reach_the_committed_plug() {
        let mut app = wizard_on_step_three();

        click_on(&mut app, "Add surface ridges");
        click_on(&mut app, "+ Add ring");
        // The first ✖ on screen, which is ring 1's.
        click_on(&mut app, "✖");
        click_on(&mut app, "Continue");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project.plug().map(|plug| plug.ridges.rings.clone()),
            Some(vec![
                RidgeRing {
                    position_frac: 0.40,
                    depth_m: 0.002,
                    half_width_frac: 0.04,
                },
                RidgeRing {
                    position_frac: 0.55,
                    depth_m: 0.002,
                    half_width_frac: 0.04,
                },
                RidgeRing {
                    position_frac: 0.50,
                    depth_m: 0.002,
                    half_width_frac: 0.04,
                },
            ]),
            "the two the ✖ left, then the one the button added: {:?}",
            studio.outcome()
        );
    }

    /// ★★ The ring switch's own wiring, end to end: the checkbox binds to
    /// `rings_enabled` rather than to the master switch above it, and the
    /// field reaches the committed plug.
    ///
    /// ⚠ Both halves of one assertion. Bound to `ridges.enabled` instead, the
    /// click collapses the editor and commits a smooth piece — and every other
    /// gate on this screen passes, because none of them clicks this box.
    #[test]
    fn switching_the_rings_off_in_the_running_wizard_commits_the_canal_without_them() {
        let mut app = wizard_on_step_three();

        click_on(&mut app, "Add surface ridges");
        click_on(&mut app, "Grip rings");
        click_on(&mut app, "Continue");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project.plug().map(|plug| plug.ridges.clone()),
            Some(RidgeOptions {
                enabled: true,
                rings: Vec::new(),
                ..RidgeOptions::default()
            }),
            "the rings alone are gone: {:?}",
            studio.outcome()
        );
    }

    /// Every ring card's header, in the order they are drawn.
    fn ring_headers(app: &App) -> Vec<String> {
        painted_texts(app)
            .into_iter()
            .filter(|text| text.starts_with("Ring "))
            .collect()
    }

    /// ⚠ The header is the only thing tying a card to the ring it edits, and
    /// no census reaches it: it is prose, not a control. Numbered from the
    /// loop index it reads "Ring 0"; stored on the row it stops renumbering
    /// when one is dropped.
    #[test]
    fn the_ring_cards_are_numbered_from_one_in_the_order_they_are_drawn() {
        let mut app = wizard_on_step_three();

        click_on(&mut app, "Add surface ridges");
        settle(&mut app);
        assert_eq!(ring_headers(&app), ["Ring 1", "Ring 2", "Ring 3"]);

        click_on(&mut app, "✖");
        settle(&mut app);
        assert_eq!(
            ring_headers(&app),
            ["Ring 1", "Ring 2"],
            "the cards renumber when one is dropped"
        );
    }

    /// ★★ The master switch's own wiring, end to end: the checkbox binds to
    /// the field, the field reaches `plug_draft`, and the draft is what
    /// Continue commits.
    ///
    /// ⚠ Nothing on screen would show this broken. The ridges appear in the
    /// preview, and the preview is a later step — a screen that drew the whole
    /// editor and committed a smooth piece anyway looks exactly right.
    #[test]
    fn switching_ridges_on_in_the_running_wizard_commits_them() {
        let mut app = wizard_on_step_three();

        click_on(&mut app, "Add surface ridges");
        click_on(&mut app, "Continue");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project.plug().map(|plug| plug.ridges.clone()),
            Some(RidgeOptions {
                enabled: true,
                ..RidgeOptions::default()
            }),
            "the switch reached the committed plug: {:?}",
            studio.outcome()
        );
    }

    /// Whether each control called `name` on step 3 is disabled, drawn from a
    /// fresh `shape` against an app that is — or is not — accepting actions.
    fn shape_controls_disabled(shape: fn() -> ShapeControls, ready: bool, name: &str) -> Vec<bool> {
        let mut shape = shape();
        controls_disabled(
            |ui| {
                let _ = draw_shape_piece(
                    ui,
                    &Studio::default(),
                    ready,
                    &mut shape,
                    &PlugFitJob::default(),
                    STEP_THREE_CELL_M,
                );
            },
            name,
        )
    }

    /// The step-3 question the opening screen asks, at step 5's own default
    /// quality.
    fn opening_fit_question() -> FitQuestion {
        FitQuestion {
            plug: ShapeControls::default().plug_draft(),
            cell_size_m: STEP_THREE_CELL_M,
            // `Studio::default()` names no prep, so nothing is stamped — the
            // state these layout gates draw against.
            scan: None,
        }
    }

    /// Every piece of text `body` puts on screen, prose included.
    ///
    /// ⚠ `value()` as well as `label()`, for the reason
    /// [`controls_in_column`] gives: a verdict is prose on a `Role::Label`, and
    /// reading only `label()` would make it look as though nothing was drawn.
    fn prose_in_column(mut body: impl FnMut(&mut egui::Ui)) -> Vec<String> {
        use egui_kittest::kittest::NodeT;

        column_harness(&mut body)
            .root()
            .children_recursive()
            .filter_map(|node| {
                let widget = node.accesskit_node();
                widget.label().or_else(|| widget.value())
            })
            .collect()
    }

    /// Step 3 drawn against `job`, with nothing holding the app.
    fn shape_body_with(job: &PlugFitJob) -> impl FnMut(&mut egui::Ui) + '_ {
        move |ui| {
            let mut shape = ShapeControls::default();
            let _ = draw_shape_piece(
                ui,
                &Studio::default(),
                true,
                &mut shape,
                job,
                STEP_THREE_CELL_M,
            );
        }
    }

    /// ★★ Step 3's fit wiring, which nothing else reaches. `draw_shape_piece`
    /// hands back a question and `start_plug_fit` runs one, but a call site is
    /// not a function anyone can call — deleting the dispatch leaves both ends
    /// tested and the button dead.
    ///
    /// ⚠ Clicked through one `+` first, for the reason
    /// [`clicking_continue_in_the_running_wizard_shapes_the_piece`] does it:
    /// the check has to carry the number on screen. A constant, or the field
    /// read at its own default, passes without it.
    ///
    /// ⚠ No poller runs on this app, so the spawned check stays in flight and
    /// the question it was started with can be read back.
    #[test]
    fn clicking_check_fit_in_the_running_wizard_asks_about_the_field_on_screen() {
        let mut app = wizard_on_step_three();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default());
        // ⚠ Off its default, for the same reason the `+` below is clicked: at
        // index 0 a check pinned to print quality asks the right question by
        // accident, and reading step 5's picker at all goes ungated.
        app.world_mut().resource_mut::<MoldControls>().quality_idx = FAST_QUALITY_IDX;

        click_on(&mut app, "+");
        click_on(&mut app, CHECK_FIT);

        let job = app.world().resource::<PlugFitJob>();
        assert_eq!(
            job.asking().map(|question| question.plug.cavity_inset_m),
            Some(0.006),
            "the 5 mm field stepped once is what the check asked about"
        );
        assert_eq!(
            job.asking().map(|question| question.cell_size_m),
            Some(cell_size_m_for_quality(FAST_QUALITY_IDX)),
            "and it asked at the quality step 5 would have cast at"
        );
    }

    /// Pretend the question on screen has stood still long enough to settle.
    ///
    /// ⚠ The gates below run their frames in microseconds. Without this each
    /// would have to wait out [`cf_studio_gui::FIT_SETTLE`] in real time.
    fn stand_still(app: &mut App) {
        app.world_mut()
            .resource_mut::<ShapeControls>()
            .settle
            .back_date(cf_studio_gui::FIT_SETTLE);
    }

    /// ★★★ The unasked check, driven end to end through the system every click
    /// reaches the app by. `fit_check_is_due` and `FitSettle` are both tested
    /// on their own, and a call site is not a function anyone can call: delete
    /// the `||` in `draw_shape_piece` and both ends stay green while step 3
    /// goes back to costing a full cast to find out.
    ///
    /// ⚠ Clicked through one `+` first, for the reason
    /// [`clicking_check_fit_in_the_running_wizard_asks_about_the_field_on_screen`]
    /// does it: the check has to carry the number on screen, and a constant
    /// passes without it.
    ///
    /// ⚠ Two-sided. The first assertion is what says the settle is load-bearing
    /// — without it, a check that fires on the very first frame passes.
    #[test]
    fn an_inset_left_alone_is_checked_without_anyone_clicking() {
        let mut app = wizard_on_step_three();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default());
        app.world_mut().resource_mut::<MoldControls>().quality_idx = FAST_QUALITY_IDX;

        click_on(&mut app, "+");
        assert!(
            app.world().resource::<PlugFitJob>().asking().is_none(),
            "a stepper still being clicked has not settled on anything"
        );

        stand_still(&mut app);
        app.update();

        assert_eq!(
            app.world()
                .resource::<PlugFitJob>()
                .asking()
                .map(|question| question.plug.cavity_inset_m),
            Some(0.006),
            "the 5 mm field stepped once is what settled, and what was checked"
        );
    }

    /// ⚠ The scan half of `askable`, on the fixture that pins WHICH artifact it
    /// names. The raw scan is accepted and step 2 is not saved, so `prep` is
    /// absent where `scan` is present — and `start_plug_fit` needs the cleaned
    /// one. Against a project with neither, `prep()` and `scan()` read alike
    /// and either passes.
    ///
    /// ⚠⚠ The absence is asserted against a POSITIVE CONTROL in the same app,
    /// because absence alone is not evidence: the first version of this gate
    /// read `PlugFitJob::asking()`, which only ever names a check that got as
    /// far as SPAWNING. A refusal for want of a scan never does, so dropping
    /// the guard left the gate green. Clicking the button proves the line is
    /// reachable here at all.
    #[test]
    fn nothing_checks_itself_until_the_scan_is_cleaned() {
        let mut app = app_running_the_wizard();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default());
        let mut scanned = Project::new("fit gate");
        scanned.set_scan(ScanInput {
            source_path: PathBuf::from("scan.stl"),
        });
        app.insert_resource(Studio {
            project: scanned,
            cursor: WizardCursor::new(Step::ShapePiece),
            ..Studio::default()
        });
        let refused = |app: &App| {
            painted_texts(app)
                .iter()
                .any(|text| text.contains("Clean and save"))
        };

        settle(&mut app);
        stand_still(&mut app);
        settle(&mut app);
        let unasked = refused(&app);

        click_on(&mut app, CHECK_FIT);
        settle(&mut app);

        assert!(!unasked, "a settled screen with no scan asked nothing");
        assert!(
            refused(&app),
            "and the line it would have posted is reachable here: {:?}",
            painted_texts(&app)
        );
    }

    /// ★★★ The loop the operator actually lives in: read a verdict, move the
    /// inset, read the next one. Both halves of it, and neither is reachable
    /// from a gate that only watches the FIRST check start.
    ///
    /// ⚠ The answer is landed by handing back the question the check itself
    /// asked, never by rebuilding one here. A reconstructed question is a
    /// second copy of what `draw_shape_piece` composes, and a gate holding one
    /// agrees with itself while the screen drifts.
    #[test]
    fn a_moved_inset_is_checked_again_over_the_answer_it_replaces() {
        let mut app = wizard_on_step_three();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default());
        app.world_mut().resource_mut::<MoldControls>().quality_idx = FAST_QUALITY_IDX;

        settle(&mut app);
        stand_still(&mut app);
        app.update();
        let first = app
            .world()
            .resource::<PlugFitJob>()
            .asking()
            .cloned()
            .expect("the opening screen settles into a check of its own");

        // The state the operator is left looking at once it lands: a verdict
        // about exactly what is on screen.
        *app.world_mut().resource_mut::<PlugFitJob>() =
            PlugFitJob::answered_with(first.clone(), Ok(cf_studio_engine::PlugFit::Casts));
        settle(&mut app);
        stand_still(&mut app);
        app.update();
        assert!(
            app.world().resource::<PlugFitJob>().asking().is_none(),
            "a question already answered is not asked again, however long it stands"
        );

        click_on(&mut app, "+");
        stand_still(&mut app);
        app.update();

        assert_eq!(
            first.plug.cavity_inset_m, 0.005,
            "the opening screen's own 5 mm is what settled first"
        );
        assert_eq!(
            app.world()
                .resource::<PlugFitJob>()
                .asking()
                .map(|question| question.plug.cavity_inset_m),
            Some(0.006),
            "and the stepped 6 mm is asked about over the verdict it stale-dropped"
        );
    }

    /// Step 5's Fast preview, the quality that is not the default.
    const FAST_QUALITY_IDX: i32 = 1;

    /// ★★★ The axis no field on step 3 can carry, driven end to end. Step 2 is
    /// reachable from here and a second Save rewrites the cleaned scan in
    /// place: every field on this screen reads the same afterwards, and the
    /// body does not. A panel that left `scan` at `None` would keep the old
    /// verdict on screen, and `fit_view`'s own gate would still pass.
    ///
    /// ⚠ Two-sided. The first assertion says the scan is named at all; the
    /// second says the name follows the file rather than the path, which never
    /// moved.
    #[test]
    fn the_question_names_the_scan_it_is_about_not_the_path_to_it() {
        let dir = crate::save::tests::temp_dir("step3-fit-scan");
        let cleaned_stl = dir.join("s.cleaned.stl");
        // Never parsed — `scan_stamp` reads the file's metadata, not its body.
        let written = [
            std::fs::write(&cleaned_stl, b"first"),
            std::fs::write(dir.join("s.prep.toml"), b""),
        ];
        assert!(written.iter().all(Result::is_ok), "the fixture must write");

        let mut app = app_running_the_wizard();
        app.add_plugins(bevy::prelude::TaskPoolPlugin::default());
        app.insert_resource(Studio {
            project: crate::preview::tests::cleaned(cf_studio_core::PrepInput {
                cleaned_stl: cleaned_stl.clone(),
                prep_toml: dir.join("s.prep.toml"),
            }),
            cursor: WizardCursor::new(Step::ShapePiece),
            ..Studio::default()
        });

        click_on(&mut app, CHECK_FIT);
        let before = app
            .world()
            .resource::<PlugFitJob>()
            .asking()
            .map(|q| q.scan);

        // A Save at a different smoothing: same path, a different body. The job
        // is cleared because nothing here polls one, and a check in flight
        // relabels the button it would be clicked with.
        *app.world_mut().resource_mut::<PlugFitJob>() = PlugFitJob::default();
        let rewritten = std::fs::write(&cleaned_stl, b"second, and longer");
        click_on(&mut app, CHECK_FIT);
        let after = app
            .world()
            .resource::<PlugFitJob>()
            .asking()
            .map(|q| q.scan);

        let _ = std::fs::remove_dir_all(&dir);
        assert!(rewritten.is_ok(), "the fixture must be rewritable");
        assert!(
            before.flatten().is_some(),
            "the question has to name the scan it is about: {before:?}"
        );
        assert_ne!(
            before, after,
            "a scan rewritten in place is a different question, at the same path"
        );
    }

    /// ★★★ The one thing that must not regress: Continue may never wait on the
    /// check. It was the rule when the check was opt-in, and it is a harder one
    /// now that a settled screen starts a four-minute run on its own.
    ///
    /// ⚠ Three assertions, and the middle one is what says the button changed
    /// state rather than merely gaining a second label.
    #[test]
    fn a_running_check_stands_its_own_button_down_and_leaves_continue_alone() {
        let running = PlugFitJob::running_for(opening_fit_question());
        let mut body = shape_body_with(&running);

        // ⚠ The census, not a search for the name: it says there is exactly ONE
        // fit button — the same one relabelled rather than a second appearing —
        // and `controls_in_column` asserts the wider label still fits the
        // column, which no other gate draws this state to find out.
        assert_eq!(
            controls_in_column(&mut body),
            [
                "−",
                "TextInput",
                "+",
                "Checking…",
                MASTER_SWITCH,
                "Continue"
            ],
            "the running screen is the opening one with the fit button relabelled"
        );
        assert_eq!(
            controls_disabled(&mut body, "Checking…"),
            vec![true],
            "which refuses a second check while the first is in flight"
        );
        assert_eq!(
            controls_disabled(&mut body, "Continue"),
            vec![false],
            "Continue is never gated on the fit check"
        );
    }

    /// ★★★ The staleness rule reaching the screen. `fit_view` gates what came
    /// back on the question, but a panel that drew `answered` directly would
    /// show it about an inset the user has already changed — and the pure gate
    /// in the lib would still pass.
    ///
    /// ⚠ Both outcomes, because the panel draws them through arms of their own:
    /// one arm left out is an outcome the operator never sees.
    ///
    /// ⚠ Two-sided per outcome. Without the first assertion, a screen that
    /// draws nothing at all passes the second.
    #[test]
    fn what_the_check_came_back_with_is_drawn_only_while_it_describes_the_screen() {
        const REASON: &str = "plug layer 0 came out in 3 pieces";
        let outcomes: [(&str, Result<cf_studio_engine::PlugFit, String>); 2] = [
            (
                "the cast's refusal",
                Ok(cf_studio_engine::PlugFit::WillNotCast {
                    reason: REASON.to_string(),
                }),
            ),
            ("a check that could not run", Err(REASON.to_string())),
        ];
        let says_why = |job: &PlugFitJob| {
            prose_in_column(shape_body_with(job))
                .iter()
                .any(|text| text.contains(REASON))
        };

        let asked = opening_fit_question();
        let moved_on = FitQuestion {
            plug: PlugDraft {
                cavity_inset_m: 0.006,
                ..asked.plug.clone()
            },
            ..asked.clone()
        };

        for (what, outcome) in outcomes {
            assert!(
                says_why(&PlugFitJob::answered_with(asked.clone(), outcome.clone())),
                "{what} about the screen is on it, in the words it came back with"
            );
            assert!(
                !says_why(&PlugFitJob::answered_with(moved_on.clone(), outcome)),
                "{what} about an inset the screen has left is not shown at all"
            );
        }
    }

    /// ⚠ `accepting_actions` is gated on its own, but nothing said step 3 hands
    /// it to anything. With `ready` replaced by `true`, Continue commits and
    /// advances behind an open picker — and the folder that picker returns then
    /// lands on a step the user has already left.
    ///
    /// ⚠ Every control. A screen that gates only its button still lets the
    /// values under it move while a job reads them — and the ridge editor is
    /// drawn by a function of its own, which took `ready` with nothing asking
    /// whether it used it.
    ///
    /// ⚠ One held case, not one per way of holding the app: what holds it is
    /// [`accepting_actions`]'s own gate above, and re-deriving that here would
    /// have this test agree with it three times over instead of asking what
    /// this screen does with the answer.
    #[test]
    fn step_threes_controls_are_offered_only_while_the_app_is_free() {
        // ⚠ "CheckBox" is the four ridge switches, which carry no name of their
        // own. Without them the row helper could drop `ready` from its checkbox
        // alone and every other control would still report correctly.
        let screens: [(&str, fn() -> ShapeControls, &[&str]); 2] = [
            (
                "closed",
                ShapeControls::default,
                &["Continue", CHECK_FIT, "+", MASTER_SWITCH],
            ),
            (
                "open",
                ridges_on,
                &[
                    "Continue",
                    CHECK_FIT,
                    "+",
                    MASTER_SWITCH,
                    "CheckBox",
                    "Grip rings",
                    "✖",
                    "+ Add ring",
                ],
            ),
        ];

        for (screen, shape, names) in screens {
            for name in names {
                let offered = shape_controls_disabled(shape, true, name);
                assert!(
                    !offered.is_empty(),
                    "{name} is on the {screen} editor at all"
                );
                assert!(
                    offered.iter().all(|disabled| !disabled),
                    "{name} is offered on the {screen} editor when nothing holds the app"
                );

                // ⚠ The length too. `all` over an empty result is true, so a
                // control that vanished while the app was held — rather than
                // being offered and refused — would pass.
                assert_eq!(
                    shape_controls_disabled(shape, false, name),
                    vec![true; offered.len()],
                    "{name} is withheld on the {screen} editor while something holds the app"
                );
            }
        }
    }

    /// The question the modal is asked to render. Its wording belongs to
    /// [`save::overwrite_question`], not to this gate.
    const A_QUESTION: &str = "base.cleaned.stl / .prep.toml already exist in /tmp.";

    /// Lay the overwrite modal out, optionally `pick` one of its buttons, and
    /// report the buttons it offered and the answer it gave.
    ///
    /// ⚠ The answer is accumulated, not read off the last frame: `run` may draw
    /// several, and a later one reporting `None` would erase the click.
    fn modal_answer(pick: Option<&str>) -> (Vec<String>, Option<SaveChoice>) {
        use egui::accesskit::Role;
        use egui_kittest::kittest::NodeT;

        let answer = std::cell::Cell::new(None);
        let mut harness = Harness::builder()
            .with_size(egui::Vec2::new(MODAL_WIDTH * 2.0, COLUMN_HEIGHT))
            .build(|ctx| {
                if let Some(choice) = draw_save_modal(ctx, A_QUESTION) {
                    answer.set(Some(choice));
                }
            });
        harness.ctx.set_fonts(crate::plugin::font_definitions());
        harness.run();

        // ⚠ The modal is the eighth surface, and `controls_in_column` cannot
        // reach it: it is centred on the window, not laid out in the body
        // column. So the two things that helper does for every other screen —
        // does it fit, and can it be drawn — are done here instead.
        let placed: Vec<(String, egui::Rect)> = harness
            .root()
            .children_recursive()
            .filter(|node| node.accesskit_node().role() == Role::Button)
            .map(|node| {
                (
                    node.accesskit_node().label().unwrap_or_default(),
                    node.rect(),
                )
            })
            .collect();
        for (label, _) in &placed {
            assert_renders(&harness.ctx, label);
        }
        assert_renders(&harness.ctx, A_QUESTION);
        let left = placed.iter().map(|(_, r)| r.min.x).fold(f32::MAX, f32::min);
        let right = placed.iter().map(|(_, r)| r.max.x).fold(f32::MIN, f32::max);
        assert!(
            right - left <= MODAL_WIDTH,
            "the answers span {:.1} px of a {MODAL_WIDTH} px modal, and egui \
             clips the overflow rather than wrapping it: {placed:?}",
            right - left,
        );

        let buttons = placed.into_iter().map(|(label, _)| label).collect();
        if let Some(label) = pick {
            harness.get_by_label(label).click();
            harness.run();
        }
        (buttons, answer.get())
    }

    /// ★ The modal, as it is built. Its three answers are the only way out of a
    /// held Save — `pending_save` gates every other control — so a button that
    /// is missing, or wired to the wrong answer, strands the app.
    ///
    /// ⚠ Driven through the modal rather than asserted on [`SaveChoice`], which
    /// is three unit variants and agrees with itself. And it must answer only
    /// when clicked: `should_close` also fires on Escape and on the backdrop,
    /// and a modal that reported Cancel unprompted would cancel every save the
    /// moment it was raised.
    #[test]
    fn the_overwrite_modal_offers_three_answers_and_each_one_lands() {
        let (buttons, unclicked) = modal_answer(None);

        assert_eq!(
            buttons,
            ["Overwrite", "Choose a different folder…", "Cancel"]
        );
        assert_eq!(unclicked, None, "and it answers nothing until asked");
        assert_eq!(
            modal_answer(Some("Overwrite")).1,
            Some(SaveChoice::Overwrite)
        );
        assert_eq!(
            modal_answer(Some("Choose a different folder…")).1,
            Some(SaveChoice::ChooseFolder)
        );
        assert_eq!(modal_answer(Some("Cancel")).1, Some(SaveChoice::Cancel));
    }

    /// Lay `row` out, click its first `label` button, and hand back the
    /// controls it acted on.
    fn after_clicking(
        label: &str,
        mut controls: EditControls,
        mut row: impl FnMut(&mut egui::Ui, &mut EditControls),
    ) -> EditControls {
        {
            let controls = std::cell::RefCell::new(&mut controls);
            let mut body = |ui: &mut egui::Ui| row(ui, &mut controls.borrow_mut());
            let mut harness = column_harness(&mut body);
            // A row may hold two steppers; the first is the one asserted on.
            harness
                .get_all_by_label(label)
                .next()
                .expect("the row has a stepper")
                .click();
            harness.run();
        }
        controls
    }

    /// ★ The ± buttons were decorative on the face target: at the millimetre
    /// step the three mm fields use, reaching the floor from the default took
    /// 199 000 clicks.
    ///
    /// ⚠ Driven through the widget, not asserted on the constants. Only this
    /// says `step_box` hands each field *its own* step — a swap would leave
    /// both constants correct and both fields wrong.
    ///
    /// ⚠ Round trips, not single clicks. `+` and `−` take the step separately,
    /// so clicking only `+` passes a stepper that adds 10 000 and subtracts 1.
    #[test]
    fn each_stepper_moves_its_field_by_that_fields_step() {
        let tidy = |ui: &mut egui::Ui, c: &mut EditControls| {
            let _ = draw_tidy_row(ui, c, true);
        };
        let trim = |ui: &mut egui::Ui, c: &mut EditControls| {
            let _ = draw_trim_row(ui, c, true);
        };
        let faces = EditControls::default().target_faces.value();
        let tip = EditControls::default().tip_mm.value();

        let c = after_clicking("+", EditControls::default(), tidy);
        assert_eq!(c.target_faces.value(), faces + SIMPLIFY_STEP_FACES, "up");
        let c = after_clicking("−", c, tidy);
        assert_eq!(c.target_faces.value(), faces, "and back down");

        let c = after_clicking("+", EditControls::default(), trim);
        assert_eq!(
            c.tip_mm.value(),
            tip + STEP_MM,
            "up, a millimetre at a time"
        );
        let c = after_clicking("−", c, trim);
        assert_eq!(c.tip_mm.value(), tip, "and back down");
    }

    // ── step 4: the silicone layer stack ────────────────────────────────────

    /// Step 4's body, laid out with `design` behind it.
    fn design_body(design: &mut DesignControls) -> impl FnMut(&mut egui::Ui) + '_ {
        move |ui| {
            let _ = draw_design_layers(ui, true, design);
        }
    }

    /// One layer's card: the ✖ that drops it, its material picker, then its
    /// two steppers.
    const LAYER_CARD: [&str; 8] = [
        "✖",
        "ComboBox",
        "−",
        "TextInput",
        "+",
        "−",
        "TextInput",
        "+",
    ];

    /// ⚠ Assembled from named blocks rather than derived from the screen, for
    /// the reason on the ridge editor's census: this is the list a person
    /// checks against the pre-port editor, and a block that moved, vanished or
    /// arrived twice changes it.
    ///
    /// ⚠ `controls_in_column` also asserts every control lands *inside* the
    /// column, which is the whole reason a layer is a stacked card and not the
    /// pre-port's one-line row.
    #[test]
    fn the_layer_screen_is_laid_out_inside_the_body_column() {
        let mut design = DesignControls::default();

        let controls = controls_in_column(design_body(&mut design));

        assert_eq!(
            controls,
            [&LAYER_CARD[..], &LAYER_CARD, &LAYER_CARD, &ACTIONS,].concat(),
            "the three layers the screen opens on, then the three buttons"
        );
    }

    /// ★ The cast needs a stack, and `LayerStack::remove` refuses to empty one
    /// — but a live ✖ that does nothing is a button the user has to guess
    /// about. Both halves, because either alone leaves the other free to go.
    #[test]
    fn only_the_last_layers_cross_is_disabled() {
        let mut design = DesignControls::default();
        assert_eq!(
            controls_disabled(design_body(&mut design), "✖"),
            [false, false, false],
            "three layers: every ✖ is live"
        );

        design.layers.remove(2);
        design.layers.remove(1);

        assert_eq!(
            controls_disabled(design_body(&mut design), "✖"),
            [true],
            "the one that is left cannot be dropped"
        );
    }

    /// ⚠ Every control, by name — a screen that gates its buttons and leaves
    /// the pickers or the steppers live is `accepting_actions` honoured by
    /// half, which is the thing that rule exists to prevent.
    ///
    /// ⚠ The count is asserted beside the flags. "Nothing on this screen is
    /// live" is also true of a screen that drew nothing at all, and an empty
    /// result is not evidence.
    #[test]
    fn every_control_on_the_layer_screen_is_gated_while_the_app_works() {
        use egui_kittest::kittest::NodeT;

        let mut design = DesignControls::default();
        let mut body = |ui: &mut egui::Ui| {
            let _ = draw_design_layers(ui, false, &mut design);
        };

        let disabled: Vec<bool> = column_harness(&mut body)
            .root()
            .children_recursive()
            .filter_map(|node| {
                let widget = node.accesskit_node();
                control_name(widget.role(), widget.label())?;
                Some(widget.is_disabled())
            })
            .collect();

        assert_eq!(
            disabled.len(),
            3 * LAYER_CARD.len() + 3,
            "the whole screen is still drawn: three cards and three buttons"
        );
        assert_eq!(
            disabled,
            vec![true; disabled.len()],
            "and not one of them is live"
        );
    }

    /// Every button on the screen, by name, with the first layer's material
    /// picker opened or left shut.
    ///
    /// ⚠ Three passes after the click, not one: the click lands on the frame
    /// after it is queued, egui opens the popup on the next, and lays its items
    /// out on the one after that — the "a widget is placed from the previous
    /// pass" rule `settle` exists for on the Bevy side.
    fn design_buttons(design: &mut DesignControls, open_the_picker: bool) -> Vec<String> {
        use egui_kittest::kittest::NodeT;

        let mut body = design_body(design);
        let mut harness = column_harness(&mut body);
        if open_the_picker {
            harness
                .root()
                .children_recursive()
                .find(|node| node.accesskit_node().role() == egui::accesskit::Role::ComboBox)
                .expect("every layer card draws one")
                .click();
            for _ in 0..3 {
                harness.run();
            }
        }
        harness
            .root()
            .children_recursive()
            .filter_map(|node| {
                let widget = node.accesskit_node();
                (widget.role() == egui::accesskit::Role::Button)
                    .then(|| widget.label())
                    .flatten()
            })
            .collect()
    }

    /// The buttons under the cards, in the order the pre-port screen had them.
    const ACTIONS: [&str; 3] = ["+ Add layer", "Use this design", "…or load a file"];

    /// The note the lib writes when the rows cannot hold a loaded design —
    /// `base_mold`'s 17.5 mm layer against the opening stack's 18.
    fn inexact_design_note() -> String {
        cf_studio_gui::format_inexact_design(
            &cf_studio_gui::LayerStack::default(),
            &[cf_studio_core::LayerDraft {
                thickness_m: 0.0175,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            }],
        )
        .expect("a stack the steppers cannot hold must warn")
    }

    /// ★ That note quotes this screen's commit button by name, and the two live
    /// in different files — the lib cannot see a label the panel draws.
    /// `ACTIONS` is pinned to the drawn buttons by
    /// `the_layer_screen_is_laid_out_inside_the_body_column`, so tying the note
    /// to it is the only thing stopping a renamed button from leaving the note
    /// pointing at a control that is no longer there.
    #[test]
    fn the_inexact_design_note_names_the_button_that_would_write_the_rows() {
        let note = inexact_design_note();
        assert!(
            note.contains(ACTIONS[1]),
            "the note must name the button it warns about: {note}"
        );
    }

    /// The column the body lays out in, and the three action buttons' rects
    /// inside it, in the order they are drawn.
    fn action_button_rects(design: &mut DesignControls) -> (egui::Rect, Vec<egui::Rect>) {
        use egui_kittest::kittest::NodeT;

        let column = std::cell::Cell::new(egui::Rect::NOTHING);
        let mut inner = design_body(design);
        let mut body = |ui: &mut egui::Ui| {
            column.set(ui.max_rect());
            inner(ui);
        };
        let rects = column_harness(&mut body)
            .root()
            .children_recursive()
            .filter_map(|node| {
                let label = node.accesskit_node().label()?;
                ACTIONS.contains(&label.as_str()).then(|| node.rect())
            })
            .collect();
        (column.get(), rects)
    }

    /// ★ One row, as the pre-port screen had them. Stacked they still fit the
    /// column and still pass the census — three unrelated steps where the
    /// screen means one choice.
    ///
    /// ⚠ The row is LEFT-ALIGNED, where the pre-port centred it, and this
    /// pins that: `vertical_centered` centres a lone widget but not a
    /// `horizontal` inside it — the row claims the full width and lays out
    /// from its left edge, and `Layout::with_main_align(Center)` does not
    /// change that (both measured: 0 px left, 128.6 right). Step 3's cavity
    /// row has sat this way since #884; centring rows is one app-wide change,
    /// not this screen's.
    #[test]
    fn the_three_actions_sit_on_one_row_against_the_left_margin() {
        let mut design = DesignControls::default();

        let (column, rects) = action_button_rects(&mut design);

        assert_eq!(rects.len(), ACTIONS.len(), "all three are drawn: {rects:?}");
        assert!(
            rects
                .windows(2)
                .all(|pair| (pair[0].top() - pair[1].top()).abs() < 0.01),
            "one row: {rects:?}"
        );
        assert!(
            (rects[0].left() - column.left()).abs() < 1.0,
            "flush left, as every other row in this column is: {rects:?}"
        );
    }

    /// Lay step 4 out with `design` and click the `nth` button called `name`.
    fn click_nth_layer_button(design: &mut DesignControls, name: &str, nth: usize) {
        let mut body = design_body(design);
        let mut harness = column_harness(&mut body);
        harness
            .get_all_by_label(name)
            .nth(nth)
            .expect("the screen draws that many")
            .click();
        harness.run();
    }

    /// Every layer's `(thickness, slacker)`, in the order they are drawn.
    fn layer_fields(design: &DesignControls) -> Vec<(i32, i32)> {
        design
            .layers
            .rows()
            .iter()
            .map(|row| (row.thickness_mm.value(), row.slacker_pct.value()))
            .collect()
    }

    /// ★ Each card edits its own row. The screen draws all three from one loop
    /// over `rows_mut`, and a card reaching a fixed index — or three sibling
    /// `Ui`s sharing an id — moves the wrong layer while looking right.
    ///
    /// ⚠ The whole stack is asserted, not the layer that was meant to move: a
    /// stepper that moved two rows passes any check that only reads one.
    #[test]
    fn a_cards_stepper_moves_its_own_layer_and_no_other() {
        let mut design = DesignControls::default();

        // Thickness then slacker, card by card — so the third `+` is layer 2's
        // thickness and the sixth is layer 3's slacker.
        click_nth_layer_button(&mut design, "+", 2);
        click_nth_layer_button(&mut design, "+", 5);

        assert_eq!(
            layer_fields(&design),
            [(18, 25), (9, 0), (5, 1)],
            "layer 2 thickened and layer 3 softened, each on its own row"
        );
    }

    /// The silicone each layer's picker shows, in the order they are drawn.
    ///
    /// ⚠ `value()`, not `label()`. A `ComboBox` reports its selected text as
    /// its value and carries no label at all, which is why the control census
    /// records it by role — and why the census cannot see this.
    fn silicones_shown(design: &mut DesignControls) -> Vec<String> {
        use egui_kittest::kittest::NodeT;

        let mut body = design_body(design);
        column_harness(&mut body)
            .root()
            .children_recursive()
            .filter_map(|node| {
                let widget = node.accesskit_node();
                (widget.role() == egui::accesskit::Role::ComboBox)
                    .then(|| widget.value())
                    .flatten()
            })
            .collect()
    }

    /// ★ Which silicone a layer is poured in is the one thing on the card that
    /// cannot be inferred from anything else on it, and every other gate reads
    /// the picker by its *role*: emptied of its selected text it counts, lays
    /// out and opens exactly the same. Found by mutation.
    #[test]
    fn each_picker_shows_its_own_layers_silicone() {
        let mut design = DesignControls::default();

        assert_eq!(
            silicones_shown(&mut design),
            [
                "Ecoflex 00-30 (medium-soft)",
                "Dragon Skin 10A (soft)",
                "Dragon Skin 20A (firm)",
            ],
            "innermost first, each card naming its own row's silicone"
        );
    }

    /// ★ What the picker *offers*, which no other gate reaches: every census
    /// here reads the selected name off the shut control, and a picker wired
    /// to one silicone shows the same name and passes all of them.
    ///
    /// ⚠ The buttons the popup adds, not the buttons on screen — the cards
    /// draw ✖ and two steppers each, and the difference is the menu.
    #[test]
    fn the_material_picker_offers_every_silicone_in_the_catalog() {
        let mut design = DesignControls::default();
        let shut = design_buttons(&mut design, false);

        let opened = design_buttons(&mut design, true);

        let menu: Vec<String> = opened
            .into_iter()
            .filter(|button| !shut.contains(button))
            .collect();
        assert_eq!(
            menu,
            Silicone::catalog()
                .into_iter()
                .map(|silicone| silicone.name.to_string())
                .collect::<Vec<_>>(),
            "the whole catalog, in the order it offers it"
        );
    }

    /// Every material picker's laid-out width, in the order they are drawn.
    fn picker_widths(design: &mut DesignControls) -> Vec<f32> {
        use egui_kittest::kittest::NodeT;

        let mut body = design_body(design);
        column_harness(&mut body)
            .root()
            .children_recursive()
            .filter(|node| node.accesskit_node().role() == egui::accesskit::Role::ComboBox)
            .map(|node| node.rect().width())
            .collect()
    }

    /// ★ What [`SILICONE_PICKER_WIDTH`] is actually for. egui grows a
    /// `ComboBox` to its selected text and **never truncates** it — measured,
    /// by laying the widest picker out at 20 px and watching it come back the
    /// same 189 px as at 180 — so the gate this replaced, that the longest
    /// name "fits", could not have failed. What the constant buys is three
    /// cards that stay one width; below the widest name they jitter apart as
    /// the user changes materials, which is what this catches.
    ///
    /// ⚠ The three silicones the screen opens on have three different name
    /// widths, which is what makes this gate able to fail.
    #[test]
    fn the_material_pickers_stay_one_width_whatever_they_show() {
        let mut design = DesignControls::default();

        let widths = picker_widths(&mut design);

        assert_eq!(widths.len(), 3, "one picker per layer: {widths:?}");
        assert!(
            widths
                .windows(2)
                .all(|pair| (pair[0] - pair[1]).abs() < 0.01),
            "three silicones, three name widths, one picker width: {widths:?}"
        );
    }

    /// Lay step 4 out with `design`, click `label`, and report the navigation
    /// intent the screen itself raised.
    fn design_nav_after(label: &str, design: &mut DesignControls) -> Option<Intent> {
        let reported = std::cell::Cell::new(None);
        {
            let borrowed = std::cell::RefCell::new(&mut *design);
            let mut body = |ui: &mut egui::Ui| {
                let acted = draw_design_layers(ui, true, &mut borrowed.borrow_mut());
                if let Some(intent) = acted.nav {
                    reported.set(Some(intent));
                }
            };
            let mut harness = column_harness(&mut body);
            harness.get_by_label(label).click();
            harness.run();
        }
        reported.get()
    }

    /// ★ The button that reaches the file picker. Every gate above counts it
    /// and reads its label; a "…or load a file" wired to nothing counts and
    /// reads exactly the same.
    #[test]
    fn loading_a_file_asks_for_the_design_picker() {
        let mut design = DesignControls::default();

        assert_eq!(
            design_nav_after("…or load a file", &mut design),
            Some(Intent::PickDesign)
        );
        assert_eq!(
            design_nav_after("+ Add layer", &mut design),
            None,
            "and the buttons beside it raise no intent at all"
        );
    }

    /// Every layer card's header, in the order they are drawn.
    fn layer_headers(app: &App) -> Vec<String> {
        painted_texts(app)
            .into_iter()
            .filter(|text| text.starts_with("Layer "))
            .collect()
    }

    /// A project with the piece shaped — the state step 4 is reached in, since
    /// [`cf_studio_core::Project::set_design`] refuses before it.
    ///
    /// ⚠ Not the default inset. The design carries a copy of it, and a zero
    /// here is exactly what a dropped one would look like.
    fn ready_to_design() -> cf_studio_core::Project {
        let mut project = crate::shape::tests::ready_to_shape();
        project
            .set_plug(PlugDraft {
                cavity_inset_m: 0.004,
                ridges: RidgeOptions::default(),
            })
            .expect("each artifact is set in workflow order");
        project
    }

    /// The running wizard parked on step 4, with the piece already shaped.
    fn wizard_on_step_four() -> App {
        let mut app = app_running_the_wizard();
        app.insert_resource(Studio {
            project: ready_to_design(),
            cursor: WizardCursor::new(Step::DesignLayers),
            ..Studio::default()
        });
        app
    }

    /// ⚠ The header is the only thing tying a card to the layer it edits, and
    /// no census reaches it: it is prose, not a control. Numbered from the loop
    /// index it reads "Layer 0"; stored on the row it stops renumbering when
    /// one is dropped.
    #[test]
    fn the_layer_cards_are_numbered_from_one_in_the_order_they_are_drawn() {
        let mut app = wizard_on_step_four();

        settle(&mut app);
        assert_eq!(layer_headers(&app), ["Layer 1", "Layer 2", "Layer 3"]);

        click_on(&mut app, "✖");
        settle(&mut app);

        assert_eq!(
            layer_headers(&app),
            ["Layer 1", "Layer 2"],
            "the outer layer renumbered into the gap the first one left"
        );
    }

    /// ★★ The whole path, click to project: `draw_design_layers` hands back a
    /// stack and `commit_design` applies one, but a call site is not a function
    /// anyone can call — deleting either leaves every gate above green.
    ///
    /// ⚠ Clicked through one `+` first, so the button has to carry the number
    /// on screen. Sending the opening stack, or reading a field pinned by its
    /// own default, passes every other gate this step has.
    #[test]
    fn clicking_use_this_design_in_the_running_wizard_commits_the_stack() {
        let mut app = wizard_on_step_four();

        click_on(&mut app, "+ Add layer");
        click_on(&mut app, "Use this design");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project.design().map(|design| design
                .layers
                .iter()
                .map(|layer| layer.material_key.as_str())
                .collect::<Vec<_>>()),
            Some(vec![
                "ECOFLEX_00_30",
                "DRAGON_SKIN_10A",
                "DRAGON_SKIN_20A",
                "DRAGON_SKIN_10A"
            ]),
            "the opening stack plus the layer added on the outside: {:?}",
            studio.outcome()
        );
    }

    /// ★ The design carries its own copy of the inset for the cast engine, and
    /// step 3 is the step that owns the number. Built from the screen's own
    /// fields it would be zero, which the engine accepts.
    #[test]
    fn the_committed_design_carries_the_shaped_pieces_cavity_inset() {
        let mut app = wizard_on_step_four();

        click_on(&mut app, "Use this design");

        let studio = app.world().resource::<Studio>();
        let inset = studio.project.design().map(|design| design.cavity_inset_m);
        assert_eq!(inset, Some(0.004), "the plug's inset, not a fresh zero");
    }

    /// ⚠ Where step 3's Continue moves on, this one stays — "+ Add layer" and
    /// "…or load a file" are reasonable things to reach for after seeing the
    /// design took, and paging away from them would be the surprise. Next →
    /// carries the user on, and only becomes available here.
    #[test]
    fn committing_the_design_stays_on_the_screen_and_opens_next() {
        let mut app = wizard_on_step_four();
        assert!(
            !nav_state(
                &app.world().resource::<Studio>().project,
                Step::DesignLayers
            )
            .can_next,
            "Next is closed until the design is set"
        );

        click_on(&mut app, "Use this design");
        settle(&mut app);

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.cursor.viewed(),
            Step::DesignLayers,
            "the screen stayed put"
        );
        assert!(
            nav_state(&studio.project, Step::DesignLayers).can_next,
            "and Next opened"
        );
    }

    /// ★ `state.rs` tests each `Studio` transition on its own, but nothing
    /// said an [`Intent`] reaches the one it names. Back paging forward, or
    /// MarkPoured doing nothing, passes every test in that module.
    ///
    /// ⚠ The dialog starts open, so `pick_scan_file`, `pick_design_file` and
    /// `pick_folder` no-op instead of putting an OS picker on screen. That
    /// leaves [`Intent::PickScan`] and [`Intent::PickDesign`], whose only
    /// effect is opening one, to a hand test — the *button* that raises
    /// `PickDesign` is gated by `loading_a_file_asks_for_the_design_picker`.
    #[test]
    fn each_intent_reaches_the_transition_it_names() {
        let mut dialog = PendingDialog::opened(DialogKind::ScanFile);
        let mut studio = Studio {
            project: ready_to_pour(),
            ..Studio::default()
        };
        let start = studio.cursor.viewed();

        apply_intent(Intent::Next, &mut studio, &mut dialog);
        assert_ne!(studio.cursor.viewed(), start, "Next moves on");
        apply_intent(Intent::Back, &mut studio, &mut dialog);
        assert_eq!(studio.cursor.viewed(), start, "and Back comes back");

        apply_intent(Intent::StartPourTimer, &mut studio, &mut dialog);
        assert!(
            studio.pour_deadline.is_some(),
            "StartPourTimer starts the pot-life clock"
        );
        apply_intent(Intent::MarkPoured, &mut studio, &mut dialog);
        assert!(
            studio.pour_deadline.is_none(),
            "and MarkPoured stops it — a different transition from the same screen"
        );

        // The two step-6 intents, each on the branch that reports rather than
        // reaching for the filesystem.
        let mut bare = Studio::default();
        apply_intent(Intent::ExportPrint, &mut bare, &mut dialog);
        assert!(
            matches!(bare.outcome(), Some(Err(text)) if text.contains("molds")),
            "ExportPrint without molds says so: {:?}",
            bare.outcome()
        );
        apply_intent(Intent::OpenExportFolder, &mut bare, &mut dialog);
        assert!(
            matches!(bare.outcome(), Some(Err(text)) if text.contains("Nothing exported")),
            "OpenExportFolder with nothing exported says so: {:?}",
            bare.outcome()
        );
    }
}
