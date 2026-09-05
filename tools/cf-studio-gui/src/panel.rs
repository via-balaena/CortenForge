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

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use cf_studio_core::{PlugDraft, Step};
use cf_studio_gui::{
    format_pour_active, format_pour_plan, format_scan_stats, nav_state, pour_countdown,
    print_step_summary, step_rows,
};

use crate::dialogs::{DialogKind, PendingDialog};
use crate::edit::{
    EditControls, EditIntent, FloorShape, SIMPLIFY_STEP_FACES, SMOOTHING_STEP, STEP_MM,
    apply_edit_intent, simplify_range, smoothing_range,
};
use crate::jobs::{SimplifyJob, start_simplify};
use crate::save;
use crate::scan::ScanEdit;
use crate::shape::{BoundedField, RidgeFields, SHAPE_STEP, ShapeControls, commit_plug};
use crate::state::{PendingSave, Studio};
use crate::widgets::{
    ACTIVE_TEXT, CONTROL_TEXT, DONE_TEXT, ERROR_TEXT, GOOD_FILL, GOOD_TEXT, HEADING_TEXT,
    HINT_TEXT, RIDGE_FILL, RIDGE_NOTE_TEXT, STATS_TEXT, WARN_TEXT, card, centered_wrapped,
    cleanup_section, field_grid, step_box, wrapped_colored, wrapped_label,
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
/// The ridge card's notes: what ridges cost, and what is not on screen yet.
const RIDGE_NOTE_SIZE: f32 = 14.0;
/// The ridge grid's cells: a toggle, a label, a stepper and a unit.
const RIDGE_COLUMNS: usize = 4;
/// The rebuilt-floor picker. Fixed, or the combo stretches to fill the column.
const SHAPE_PICKER_WIDTH: f32 = 110.0;
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
pub(crate) fn wizard_screen(
    mut contexts: EguiContexts,
    mut studio: ResMut<Studio>,
    mut dialog: ResMut<PendingDialog>,
    mut scan: ResMut<ScanEdit>,
    mut controls: ResMut<EditControls>,
    mut shape: ResMut<ShapeControls>,
    mut job: ResMut<SimplifyJob>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let mut acted = Acted::default();

    egui::SidePanel::left("checklist")
        .resizable(false)
        .exact_width(CHECKLIST_WIDTH)
        .show(ctx, |ui| draw_checklist(ui, &studio));

    egui::TopBottomPanel::bottom("nav").show(ctx, |ui| {
        acted.nav = draw_nav(ui, &studio, &dialog).or(acted.nav);
    });

    body_column(ctx, |ui| {
        // ⚠ `&scan` — an immutable borrow. Reaching for `&mut` here, to save
        // passing it twice, would mark the resource changed on every frame the
        // wizard drew and re-mesh the scan 60 times a second.
        acted.merge(draw_body(
            ui,
            &studio,
            &dialog,
            &scan,
            &mut controls,
            &mut shape,
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
        commit_plug(draft, &mut studio);
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
/// it resolves, and an unanswered Save owns it until it is answered — and
/// **paging counts**: the picker's result lands on whichever step the cursor
/// has reached by then, and a scan landing resets the project to step 1. One
/// definition so a new control cannot honour half of it.
fn accepting_actions(studio: &Studio, dialog: &PendingDialog) -> bool {
    !studio.busy && !dialog.is_open() && studio.pending_save.is_none()
}

/// Back / Help / Next, gated by [`nav_state`].
fn draw_nav(ui: &mut egui::Ui, studio: &Studio, dialog: &PendingDialog) -> Option<Intent> {
    let nav = nav_state(&studio.project, studio.cursor.viewed());
    let ready = accepting_actions(studio, dialog);
    let mut intent = None;
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        if ui
            .add_enabled(nav.can_back && ready, egui::Button::new("← Back"))
            .clicked()
        {
            intent = Some(Intent::Back);
        }
        ui.add_enabled(false, egui::Button::new("Help"))
            .on_disabled_hover_text("Per-step guidance arrives with the ported steps.");
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

/// The step message, then the body for the viewed step.
///
/// ⚠ The message sits **above** the body, where the pre-port screen put it —
/// *"so it's always visible, not buried at the bottom of a long, scrolling
/// step."* Step 2 is that long step, and it is the one whose ops the message
/// reports on, so the note's own reason applies here first.
fn draw_body(
    ui: &mut egui::Ui,
    studio: &Studio,
    dialog: &PendingDialog,
    scan: &ScanEdit,
    controls: &mut EditControls,
    shape: &mut ShapeControls,
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

    if let Some(message) = &studio.message {
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

    // ⚠ Exhaustive on purpose, with no catch-all arm. A `_ =>` (or a list of
    // "not ported yet" steps) would silently render one step's screen for a
    // step added later; this way the compiler names the new arm.
    let mut acted = Acted::default();
    match viewed {
        Step::AddScan => acted.nav = draw_add_scan(ui, studio, dialog),
        Step::CleanScan => acted.merge(draw_clean_scan(ui, studio, dialog, scan, controls)),
        Step::ShapePiece => acted.plug = draw_shape_piece(ui, studio, dialog, shape),
        Step::DesignLayers | Step::MakeMolds => draw_porting_notice(ui),
        Step::Print => acted.nav = draw_print(ui, studio, dialog),
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
    studio: &Studio,
    dialog: &PendingDialog,
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
    let ready = accepting_actions(studio, dialog);
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
        field_grid(ui, "trim-fields", 3, |ui| {
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
        field_grid(ui, "reconstruct-fields", 3, |ui| {
            ui.colored_label(CONTROL_TEXT, "Shape");
            egui::ComboBox::from_id_salt("rebuilt-floor-shape")
                .width(SHAPE_PICKER_WIDTH)
                .selected_text(controls.shape.label())
                .show_ui(ui, |ui| {
                    for shape in FloorShape::ALL {
                        ui.selectable_value(&mut controls.shape, shape, shape.label());
                    }
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
fn draw_add_scan(ui: &mut egui::Ui, studio: &Studio, dialog: &PendingDialog) -> Option<Intent> {
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
    if ui
        .add_enabled(accepting_actions(studio, dialog), egui::Button::new(label))
        .clicked()
    {
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

/// Steps 4 and 5 during the Slint→Bevy port. Says what is missing and that the
/// work is not lost, rather than showing an empty screen that reads as a bug.
fn draw_porting_notice(ui: &mut egui::Ui) {
    ui.add_space(8.0);
    wrapped_label(
        ui,
        "This step is being rebuilt on the new interface and isn't available in \
         this build yet. Its logic is unchanged — only the screen is being \
         redrawn.",
    );
}

/// Step 3 — how snugly the piece fits, what is cut into it, then commit.
fn draw_shape_piece(
    ui: &mut egui::Ui,
    studio: &Studio,
    dialog: &PendingDialog,
    shape: &mut ShapeControls,
) -> Option<PlugDraft> {
    let ready = accepting_actions(studio, dialog);
    let mut draft = None;
    ui.add_space(8.0);
    wrapped_label(
        ui,
        "Set how snugly the piece fits: the cavity sits this far in from your \
         scan's surface, all the way round.",
    );
    ui.add_space(SECTION_GAP);
    ui.vertical_centered(|ui| {
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "Cavity inset");
            bounded_step_box(ui, &mut shape.cavity_mm, ready);
            ui.colored_label(CONTROL_TEXT, "mm");
        });
    });
    ui.add_space(SECTION_GAP);
    draw_ridges(ui, &mut shape.ridges, ready);
    ui.add_space(SECTION_GAP);
    ui.vertical_centered(|ui| {
        if ui
            .add_enabled(ready, egui::Button::new("Continue"))
            .clicked()
        {
            draft = Some(shape.plug_draft());
        }
    });
    draft
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
        ui.add_space(ROW_GAP);
        // ⚠ The rings are applied whether or not this screen can show them, so
        // it says so rather than committing geometry nobody has seen.
        centered_wrapped(
            ui,
            RIDGE_NOTE_SIZE,
            RIDGE_NOTE_TEXT,
            "Grip rings are still being rebuilt on the new interface. Until they are, \
             the three tested rings are cut as they always were.",
        );
        ui.add_space(SECTION_GAP);
        field_grid(ui, "ridge-fields", RIDGE_COLUMNS, |ui| {
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

/// A stepper for one of step 3's fields.
///
/// ⚠ The bounds come off the field, so the screen cannot enforce a limit the
/// commit does not. Given the wrong ones the field walks past its own maximum
/// and [`ShapeControls::plug_draft`] quietly clamps it back — the screen
/// showing one number and the plug carrying another.
fn bounded_step_box(ui: &mut egui::Ui, field: &mut BoundedField, enabled: bool) {
    step_box(ui, &mut field.state, field.range, SHAPE_STEP, enabled);
}

/// Step 6 — save the printable files, then hand off to the slicer.
fn draw_print(ui: &mut egui::Ui, studio: &Studio, dialog: &PendingDialog) -> Option<Intent> {
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
    let ready = accepting_actions(studio, dialog);
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
        Intent::StartPourTimer => studio.start_pour_timer(),
        Intent::MarkPoured => studio.mark_poured(),
        Intent::OpenExportFolder => match studio.project.print().map(|p| p.export_dir.clone()) {
            Some(dir) => crate::jobs::reveal_in_file_manager(&dir),
            None => {
                studio.message = Some(Err(
                    "Nothing exported yet — save the files first.".to_string()
                ));
            }
        },
        Intent::ExportPrint => {
            if studio.project.molds().is_none() {
                studio.message = Some(Err("Make the molds first (step 5).".to_string()));
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
        ScanInput,
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
    /// ⚠ Enumerated, not branched: it is a three-way `&&`, so a constant
    /// answer, a dropped term or an `||` each leave the app either frozen with
    /// nothing running or clickable in the middle of a job — and none of those
    /// report themselves. Eight states is all of them.
    #[test]
    fn actions_are_accepted_only_when_nothing_else_holds_the_app() {
        for busy in [false, true] {
            for dialog_open in [false, true] {
                for saving in [false, true] {
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

                    assert_eq!(
                        accepting_actions(&studio, &dialog),
                        !busy && !dialog_open && !saving,
                        "busy={busy} dialog_open={dialog_open} saving={saving}"
                    );
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
        ];
        // Each urgency band words itself differently.
        messages.extend([600_i64, 120, -30].map(|secs| pour_countdown(secs).text));

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
            let reported = screen.studio.message.as_ref().expect("every op reports");
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
        dialog: PendingDialog,
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
            studio.message
        );
        CleanupScreen {
            studio,
            dialog: PendingDialog::default(),
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
            let _ = draw_clean_scan(
                ui,
                &screen.studio,
                &screen.dialog,
                &screen.scan,
                &mut screen.controls,
            );
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
        let dialog = PendingDialog::default();

        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_add_scan(ui, &studio, &dialog);
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
                "Add surface ridges (advanced)",
                "Continue"
            ]
        );
        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_print(ui, &studio, &dialog);
            }),
            ["Save files for printing…"]
        );
        assert_eq!(
            controls_in_column(|ui| {
                let _ = draw_nav(ui, &studio, &dialog);
            }),
            ["← Back", "Help", "Next →"]
        );
        assert!(controls_in_column(draw_porting_notice).is_empty());
        assert!(controls_in_column(|ui| draw_checklist(ui, &studio)).is_empty());
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
                let _ = draw_clean_scan(
                    ui,
                    &empty.studio,
                    &empty.dialog,
                    &empty.scan,
                    &mut empty.controls,
                );
            })
            .is_empty(),
            "with no scan there is nothing to offer but the line saying so"
        );

        let mut screen = cleanup_screen();
        screen.scan.set(ActiveScan::synthetic(open_tube()));

        let controls = controls_in_column(|ui| {
            let _ = draw_clean_scan(
                ui,
                &screen.studio,
                &screen.dialog,
                &screen.scan,
                &mut screen.controls,
            );
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
    /// been stood up.
    fn save_button_disabled(screen: &mut CleanupScreen) -> Vec<bool> {
        controls_disabled(
            |ui| {
                let _ = draw_clean_scan(
                    ui,
                    &screen.studio,
                    &screen.dialog,
                    &screen.scan,
                    &mut screen.controls,
                );
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
    /// ran during a job or an open picker would write while the question that
    /// gated it is still on screen.
    #[test]
    fn save_is_offered_only_once_the_scan_is_stood_up_and_the_app_is_free() {
        assert_eq!(
            save_button_disabled(&mut cleanup_screen()),
            [false],
            "stood up, nothing else running"
        );

        let mut flat = cleanup_screen();
        // Back to a freshly loaded scan: no centerline, so no cast frame.
        flat.scan.set(ActiveScan::synthetic(open_tube()));
        assert_eq!(save_button_disabled(&mut flat), [true], "not yet stood up");

        let mut busy = cleanup_screen();
        busy.studio.busy = true;
        assert_eq!(save_button_disabled(&mut busy), [true], "a job is running");

        let mut asking = cleanup_screen();
        asking.dialog = PendingDialog::opened(DialogKind::PrepDest);
        assert_eq!(
            save_button_disabled(&mut asking),
            [true],
            "a picker is open"
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
            .init_resource::<ScanEdit>()
            .init_resource::<EditControls>()
            .init_resource::<ShapeControls>()
            .init_resource::<SimplifyJob>()
            .add_systems(Update, (begin, wizard_screen, end).chain());
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
            studio.message
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
            studio.message
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
        let mut app = app_running_the_wizard();
        app.insert_resource(Studio {
            project: crate::shape::tests::ready_to_shape(),
            cursor: WizardCursor::new(Step::ShapePiece),
            ..Studio::default()
        });

        click_on(&mut app, "+");
        click_on(&mut app, "Continue");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project.plug().map(|plug| plug.cavity_inset_m),
            Some(0.006),
            "the 5 mm field, stepped once, committed 6 mm: {:?}",
            studio.message
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

    /// ⚠ The steps still on the notice. Replacing `draw_porting_notice` with a
    /// no-op leaves them blank and passes the control census, which counts what
    /// a screen with no controls has none of.
    #[test]
    fn the_steps_still_being_ported_say_so_rather_than_showing_nothing() {
        for step in [Step::DesignLayers, Step::MakeMolds] {
            let mut app = app_running_the_wizard();
            app.insert_resource(Studio {
                cursor: WizardCursor::new(step),
                ..Studio::default()
            });

            settle(&mut app);

            let painted = painted_texts(&app);
            assert!(
                painted.iter().any(|text| text.contains("being rebuilt")),
                "step {} says so instead of showing nothing: {painted:?}",
                step.number()
            );
        }
    }

    /// The ridge editor's master switch, by the name the census gives it.
    const MASTER_SWITCH: &str = "Add surface ridges (advanced)";

    /// Step 3 with the ridge editor asked for — the state everything below
    /// this measures.
    fn ridges_on() -> ShapeControls {
        let mut shape = ShapeControls::default();
        shape.ridges.enabled = true;
        shape
    }

    /// The step-3 body for `shape`, on an app holding nothing.
    fn shape_body(shape: &mut ShapeControls) -> impl FnMut(&mut egui::Ui) + '_ {
        move |ui| {
            let _ = draw_shape_piece(ui, &Studio::default(), &PendingDialog::default(), shape);
        }
    }

    /// ⚠ #878's overflow is what a four-column grid in a 404 px column risks,
    /// and the editor is hidden until it is asked for — so the census of the
    /// opening screen reaches none of these.
    #[test]
    fn the_revealed_ridge_editor_is_laid_out_inside_the_body_column_too() {
        let mut revealed = ridges_on();

        let controls = controls_in_column(shape_body(&mut revealed));

        assert_eq!(
            controls,
            [
                "−",
                "TextInput",
                "+",
                "Add surface ridges (advanced)",
                "CheckBox",
                "−",
                "TextInput",
                "+",
                "−",
                "TextInput",
                "+",
                "CheckBox",
                "−",
                "TextInput",
                "+",
                "CheckBox",
                "−",
                "TextInput",
                "+",
                "CheckBox",
                "−",
                "TextInput",
                "+",
                "Continue"
            ],
            "four switched rows and one governed by the switch above it"
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

    /// ⚠ The one row with no switch of its own carries a blank cell where the
    /// others carry a checkbox. Drop it and every cell after it shifts left on
    /// that row alone — a field that still fits, still works, and lines up
    /// with nothing.
    #[test]
    fn every_ridge_row_lines_its_field_up_with_the_others() {
        let mut revealed = ridges_on();

        let rects = stepper_rects(shape_body(&mut revealed));

        // The first is the cavity's, which sits above the grid entirely.
        let (_, ridges) = rects.split_first().expect("step 3 has six fields");
        assert_eq!(ridges.len(), 5, "one field per ridge scalar: {rects:?}");
        assert!(
            ridges.iter().all(|rect| rect.left() == ridges[0].left()),
            "the grid's fields must share a column: {ridges:?}"
        );
    }

    /// ★ Four switches over six steppers, every row drawn by one helper: a row
    /// handed its neighbour's switch gates the wrong field, and the screen
    /// reads the same either way.
    ///
    /// ⚠ Texture spacing is the row with no switch of its own, so it is the
    /// one a "every row owns its checkbox" port silently leaves live.
    #[test]
    fn each_ridge_toggle_gates_its_own_stepper() {
        let mut everything = ridges_on();
        assert_eq!(
            controls_disabled(shape_body(&mut everything), "+"),
            [false; 6],
            "with every feature on, every stepper is live"
        );

        let cases: [(&str, fn(&mut RidgeFields) -> &mut bool, [bool; 6]); 4] = [
            (
                "texture",
                |r| &mut r.texture_enabled,
                [false, true, true, false, false, false],
            ),
            (
                "side pinch",
                |r| &mut r.side_pinch_enabled,
                [false, false, false, true, false, false],
            ),
            (
                "tip relief",
                |r| &mut r.tip_relief_enabled,
                [false, false, false, false, true, false],
            ),
            (
                "orientation",
                |r| &mut r.orientation_enabled,
                [false, false, false, false, false, true],
            ),
        ];

        for (name, toggle, expected) in cases {
            let mut off = ridges_on();
            *toggle(&mut off.ridges) = false;

            assert_eq!(
                controls_disabled(shape_body(&mut off), "+"),
                expected,
                "{name} switched off gates its own stepper and no other"
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

    /// Step 3's fields, in the order their steppers are laid out.
    const FIELDS: [fn(&mut ShapeControls) -> &mut BoundedField; 6] = [
        |c| &mut c.cavity_mm,
        |c| &mut c.ridges.texture_depth,
        |c| &mut c.ridges.texture_spacing,
        |c| &mut c.ridges.side_pinch,
        |c| &mut c.ridges.tip_relief,
        |c| &mut c.ridges.orientation,
    ];

    /// ★★ The screen is drawn with a range and the plug is read with one, and
    /// only this says they are the same range. Handed another field's bounds a
    /// stepper walks past its own limit, `plug_draft` clamps it back, and the
    /// screen shows one number while the plug carries another.
    ///
    /// ⚠ Driven from outside the bound rather than clicked up to it — texture
    /// spacing's range is 290 steps wide.
    #[test]
    fn every_stepper_stops_at_the_bound_its_field_commits_at() {
        for (nth, pick) in FIELDS.into_iter().enumerate() {
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
        FIELDS.into_iter().map(|pick| pick(shape).value()).collect()
    }

    /// ★★ Six steppers from one helper. A row bound to its neighbour's field
    /// moves the wrong number, and every value on screen stays plausible.
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

    /// ★★ The master switch's own wiring, end to end: the checkbox binds to
    /// the field, the field reaches `plug_draft`, and the draft is what
    /// Continue commits.
    ///
    /// ⚠ Nothing on screen would show this broken. The ridges appear in the
    /// preview, and the preview is a later step — a screen that drew the whole
    /// editor and committed a smooth piece anyway looks exactly right.
    #[test]
    fn switching_ridges_on_in_the_running_wizard_commits_them() {
        let mut app = app_running_the_wizard();
        app.insert_resource(Studio {
            project: crate::shape::tests::ready_to_shape(),
            cursor: WizardCursor::new(Step::ShapePiece),
            ..Studio::default()
        });

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
            studio.message
        );
    }

    /// Whether each control called `name` on step 3 is disabled, drawn from a
    /// fresh `shape` against an app held — or not — by `studio` and `dialog`.
    fn shape_controls_disabled(
        shape: fn() -> ShapeControls,
        studio: &Studio,
        dialog: &PendingDialog,
        name: &str,
    ) -> Vec<bool> {
        let mut shape = shape();
        controls_disabled(
            |ui| {
                let _ = draw_shape_piece(ui, studio, dialog, &mut shape);
            },
            name,
        )
    }

    /// ⚠ `accepting_actions` is gated on its own, but nothing said step 3 hands
    /// it to anything. With `ready` replaced by `true`, Continue commits and
    /// advances behind an open picker — and the folder that picker returns then
    /// lands on a step the user has already left.
    ///
    /// ⚠ Every control, and each way the app is held. A screen that gates only
    /// its button still lets the values under it move while a job reads them —
    /// and the ridge editor is drawn by a function of its own, which took
    /// `ready` with nothing asking whether it used it.
    #[test]
    fn step_threes_controls_are_offered_only_while_the_app_is_free() {
        let held = [
            (
                "a job",
                Studio {
                    busy: true,
                    ..Studio::default()
                },
                PendingDialog::default(),
            ),
            (
                "a picker",
                Studio::default(),
                PendingDialog::opened(DialogKind::ScanFile),
            ),
            (
                "a save",
                Studio {
                    pending_save: Some(PendingSave::ChoosingFolder { smoothing: 0 }),
                    ..Studio::default()
                },
                PendingDialog::default(),
            ),
        ];
        // ⚠ "CheckBox" is the four ridge switches, which carry no name of their
        // own. Without them the row helper could drop `ready` from its checkbox
        // alone and every other control would still report correctly.
        let screens: [(&str, fn() -> ShapeControls, &[&str]); 2] = [
            (
                "closed",
                ShapeControls::default,
                &["Continue", "+", MASTER_SWITCH],
            ),
            (
                "open",
                ridges_on,
                &["Continue", "+", MASTER_SWITCH, "CheckBox"],
            ),
        ];

        for (screen, shape, names) in screens {
            for name in names {
                let offered = shape_controls_disabled(
                    shape,
                    &Studio::default(),
                    &PendingDialog::default(),
                    name,
                );
                assert!(
                    !offered.is_empty(),
                    "{name} is on the {screen} editor at all"
                );
                assert!(
                    offered.iter().all(|disabled| !disabled),
                    "{name} is offered on the {screen} editor when nothing holds the app"
                );

                for (what, studio, dialog) in &held {
                    // ⚠ The length too. `all` over an empty result is true, so
                    // a control that vanished while the app was held — rather
                    // than being offered and refused — would pass.
                    assert_eq!(
                        shape_controls_disabled(shape, studio, dialog, name),
                        vec![true; offered.len()],
                        "{name} is withheld on the {screen} editor while {what} holds the app"
                    );
                }
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

    /// ★ `state.rs` tests each `Studio` transition on its own, but nothing
    /// said an [`Intent`] reaches the one it names. Back paging forward, or
    /// MarkPoured doing nothing, passes every test in that module.
    ///
    /// ⚠ The dialog starts open, so `pick_scan_file` and `pick_folder`
    /// no-op instead of putting an OS picker on screen. That leaves
    /// [`Intent::PickScan`], whose only effect is opening one, to a hand test.
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
            matches!(&bare.message, Some(Err(text)) if text.contains("molds")),
            "ExportPrint without molds says so: {:?}",
            bare.message
        );
        apply_intent(Intent::OpenExportFolder, &mut bare, &mut dialog);
        assert!(
            matches!(&bare.message, Some(Err(text)) if text.contains("Nothing exported")),
            "OpenExportFolder with nothing exported says so: {:?}",
            bare.message
        );
    }
}
