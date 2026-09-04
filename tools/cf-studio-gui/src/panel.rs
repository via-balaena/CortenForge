//! The wizard chrome — checklist, header, footer nav — and the per-step bodies.
//!
//! The panel decides nothing. It renders from the lib's plain functions
//! ([`step_rows`], [`nav_state`], the `format_*` family) and turns clicks into
//! an [`Intent`], an [`EditIntent`] or a Simplify target — executed by
//! [`apply_intent`], [`apply_edit_intent`] and [`start_simplify`] respectively.
//! Keeping the egui closure free of state transitions is what makes the
//! transitions reviewable — and testable, since they are all methods on
//! `Studio` or plain functions over an `EditSession`.
//!
//! ⚠ The three kinds are **not** an accident of growth — see [`Acted`].

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use cf_studio_core::Step;
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
use crate::state::{PendingSave, Studio};
use crate::widgets::{
    ACTIVE_TEXT, CONTROL_TEXT, DONE_TEXT, ERROR_TEXT, GOOD_FILL, GOOD_TEXT, HEADING_TEXT,
    HINT_TEXT, STATS_TEXT, WARN_TEXT, card, centered_wrapped, cleanup_section, field_grid,
    step_box, wrapped_colored, wrapped_label,
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
/// The rebuilt-floor picker. Fixed, or the combo stretches to fill the column.
const SHAPE_PICKER_WIDTH: f32 = 110.0;
/// The overwrite modal's width. Wider than the body column — it is centred on
/// the whole window and has to hold a folder path.
const MODAL_WIDTH: f32 = 460.0;

/// What the frame reported.
///
/// The three are separate because executing an [`EditIntent`] borrows
/// [`ScanEdit`] mutably, which rebuilds a 200 000-face mesh; the other two must
/// not pay that. See the warning on [`ScanEdit`].
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
}

impl Acted {
    /// Fold in what a nested piece of the screen reported; it wins where both
    /// did, and each field merges on its own.
    fn merge(&mut self, inner: Self) {
        self.nav = inner.nav.or(self.nav);
        self.edit = inner.edit.or(self.edit);
        self.simplify = inner.simplify.or(self.simplify);
        self.save = inner.save.or(self.save);
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
        acted.merge(draw_body(ui, &studio, &dialog, &scan, &mut controls));
    });

    // ⚠ Drawn from `pending_save` alone, outside the step match: the state that
    // gates every control and the modal that explains why it is gated are the
    // same `Option`, so neither can outlive the other.
    if let Some(PendingSave::Confirming { dir, smoothing }) = studio.pending_save.clone()
        && let Some(choice) = draw_save_modal(ctx, &save::overwrite_question(&studio, &dir))
    {
        match choice {
            // ⚠ `&scan`, immutably — a Save only reads the session.
            SaveChoice::Overwrite => save::write_into(&scan, &mut studio, &dir, smoothing),
            SaveChoice::ChooseFolder => {
                studio.pending_save = Some(PendingSave::ChoosingFolder { smoothing });
                dialog.pick_folder(
                    DialogKind::PrepDest,
                    "Choose a folder to save the cleaned scan",
                );
            }
            SaveChoice::Cancel => save::settle(&mut studio, Ok("Save cancelled.".to_string())),
        }
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
        Step::ShapePiece | Step::DesignLayers | Step::MakeMolds => draw_porting_notice(ui),
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
/// ⚠ Stacked for [`draw_trim_row`]'s reason. As one row the label, stepper and
/// a 17-character button do not fit the 404 px column, and egui culls the
/// overflow rather than wrapping it.
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
        });
        ui.add_space(ROW_GAP);
        if ui
            .add_enabled(ready, egui::Button::new("Save cleaned scan"))
            .clicked()
        {
            clicked = Some(controls.smoothing_iters());
        }
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
    // Escape, and a click on the backdrop, are the same "get me out of this"
    // the Cancel button is. Checked only if nothing was clicked, so a real
    // answer can never be overwritten by one.
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
        field_grid(ui, "reconstruct-fields", |ui| {
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

/// Steps 3–5 during the Slint→Bevy port. Says what is missing and that the work
/// is not lost, rather than showing an empty screen that reads as a bug.
fn draw_porting_notice(ui: &mut egui::Ui) {
    ui.add_space(8.0);
    wrapped_label(
        ui,
        "This step is being rebuilt on the new interface and isn't available in \
         this build yet. Its logic is unchanged — only the screen is being \
         redrawn. Steps 1, 2, 6 and 7 work today.",
    );
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
mod tests {
    #![allow(clippy::expect_used)]

    use std::path::PathBuf;

    use cf_studio_core::{
        DesignDraft, LayerDraft, MoldOutputs, PlugDraft, PourPlan, PourStep, PrepInput, Project,
        RidgeOptions, ScanInput,
    };
    use egui_kittest::Harness;
    use egui_kittest::kittest::Queryable;

    use super::*;
    use crate::edit::tests::open_tube;
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
        use egui::accesskit::Role;
        use egui_kittest::kittest::NodeT;

        let column = std::cell::Cell::new(egui::Rect::NOTHING);
        let mut harness = Harness::builder()
            .with_size(egui::Vec2::new(BODY_WIDTH, COLUMN_HEIGHT))
            .build(|ctx| {
                body_column(ctx, |ui| {
                    column.set(ui.max_rect());
                    body(ui);
                });
            });
        harness.run();

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
                let name = match widget.role() {
                    role @ (Role::Button | Role::TextInput | Role::ComboBox) => widget
                        .label()
                        // ⚠ Not `?`. A stepper's field, the shape picker and a
                        // text-free button report no label; skipping those lets
                        // the very control this gate is for slip past both halves.
                        .unwrap_or_else(|| format!("{role:?}")),
                    _ => return None,
                };
                let rect = node.rect();
                assert!(
                    column.contains_rect(rect),
                    "{name} is laid out at {rect:?}, outside the {column:?} column"
                );
                Some(name)
            })
            .collect()
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
    fn ready_to_pour() -> Project {
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

    /// Lay `draw_clean_scan` out for a screen that has, or has not, been stood
    /// up, and report whether its Save button is disabled.
    ///
    /// ⚠ The accessibility tree's own flag. Presence is not the question — the
    /// section is on screen either way, which is how the user is told what to
    /// do first.
    fn save_button_disabled(stood_up: bool) -> Option<bool> {
        use egui_kittest::kittest::NodeT;

        let mut screen = cleanup_screen();
        if !stood_up {
            // Back to a freshly loaded scan: no centerline, so no cast frame.
            screen.scan.set(ActiveScan::synthetic(open_tube()));
        }
        let mut harness = Harness::builder()
            .with_size(egui::Vec2::new(BODY_WIDTH, COLUMN_HEIGHT))
            .build(|ctx| {
                body_column(ctx, |ui| {
                    let _ = draw_clean_scan(
                        ui,
                        &screen.studio,
                        &screen.dialog,
                        &screen.scan,
                        &mut screen.controls,
                    );
                });
            });
        harness.run();
        harness
            .root()
            .children_recursive()
            .find(|node| node.accesskit_node().label().as_deref() == Some("Save cleaned scan"))
            .map(|node| node.accesskit_node().is_disabled())
    }

    /// ★ Save is gated on the centerline, not merely on the app being idle:
    /// `EditSession::save` refuses without one, so an enabled button here buys
    /// the user a click and a "Save failed" for it.
    #[test]
    fn save_is_offered_only_once_the_scan_has_been_stood_up() {
        assert_eq!(save_button_disabled(true), Some(false), "stood up");
        assert_eq!(save_button_disabled(false), Some(true), "not yet stood up");
    }

    /// Lay the save row out with `controls`, click `label`, and report what the
    /// row itself said — the payload the click carries, not the field behind it.
    fn save_row_after(label: &str, controls: &mut EditControls) -> Option<usize> {
        let reported = std::cell::Cell::new(None);
        {
            let borrowed = std::cell::RefCell::new(&mut *controls);
            let mut harness = Harness::builder()
                .with_size(egui::Vec2::new(BODY_WIDTH, COLUMN_HEIGHT))
                .build(|ctx| {
                    body_column(ctx, |ui| {
                        if let Some(smoothing) = draw_save_row(ui, &mut borrowed.borrow_mut(), true)
                        {
                            reported.set(Some(smoothing));
                        }
                    });
                });
            harness.run();
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
        harness.run();
        let buttons = harness
            .root()
            .children_recursive()
            .filter_map(|node| {
                let widget = node.accesskit_node();
                (widget.role() == Role::Button).then(|| widget.label().unwrap_or_default())
            })
            .collect();
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
            let mut harness = Harness::builder()
                .with_size(egui::Vec2::new(BODY_WIDTH, COLUMN_HEIGHT))
                .build(|ctx| {
                    body_column(ctx, |ui| row(ui, &mut controls.borrow_mut()));
                });
            harness.run();
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
}
