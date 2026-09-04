//! The wizard chrome — checklist, header, footer nav — and the per-step bodies.
//!
//! The panel decides nothing. It renders from the lib's plain functions
//! ([`step_rows`], [`nav_state`], the `format_*` family) and turns clicks into
//! an [`Intent`] or an [`EditIntent`], executed by [`apply_intent`] and
//! [`apply_edit_intent`] respectively. Keeping the egui closure free of state
//! transitions is what makes the transitions reviewable — and testable, since
//! they are all methods on `Studio` or plain functions over an `EditSession`.
//!
//! ⚠ The two intent types are **not** an accident of growth. Executing an
//! [`EditIntent`] borrows [`ScanEdit`] mutably, which marks it changed and
//! rebuilds a 200 000-face mesh; Back and Next must not take that path. See
//! [`Acted`].

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use cf_studio_core::Step;
use cf_studio_gui::{
    format_pour_active, format_pour_plan, format_scan_stats, nav_state, pour_countdown,
    print_step_summary, step_rows,
};

use crate::dialogs::{DialogKind, PendingDialog};
use crate::edit::{EditControls, EditIntent, FloorShape, apply_edit_intent};
use crate::scan::ScanEdit;
use crate::state::Studio;
use crate::widgets::{
    ACTIVE_TEXT, CONTROL_TEXT, DONE_TEXT, ERROR_TEXT, GOOD_FILL, GOOD_TEXT, HEADING_TEXT,
    HINT_TEXT, STATS_TEXT, WARN_TEXT, card, centered_wrapped, cleanup_section, step_box,
    wrapped_colored, wrapped_label,
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

/// What the body reported this frame.
///
/// The two are separate types because executing an [`EditIntent`] borrows
/// [`ScanEdit`] mutably, and that marks it changed — which costs a full rebuild
/// of a 200 000-face mesh. Routing Back and Next through the same call would
/// charge every navigation click for it. See the warning on [`ScanEdit`].
#[derive(Default)]
struct Acted {
    /// A navigation or dialog action.
    nav: Option<Intent>,
    /// A step-2 cleanup op.
    edit: Option<EditIntent>,
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

    egui::SidePanel::right("body")
        .resizable(false)
        .exact_width(BODY_WIDTH)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // ⚠ `&scan` — an immutable borrow. Reaching for `&mut` here, to
                // save passing it twice, would mark the resource changed on
                // every frame the wizard drew and re-mesh the scan 60 times a
                // second.
                let drawn = draw_body(ui, &studio, &dialog, &scan, &mut controls);
                acted.nav = drawn.nav.or(acted.nav);
                acted.edit = drawn.edit.or(acted.edit);
            });
        });

    if let Some(intent) = acted.nav {
        apply_intent(intent, &mut studio, &mut dialog);
    }
    if let Some(intent) = acted.edit {
        apply_edit_intent(intent, &mut scan, &mut studio, &mut controls);
    }
    Ok(())
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
/// A long job owns the app until it finishes and an open OS dialog owns it
/// until it resolves — and **paging counts**: the picker's result lands on
/// whichever step the cursor has reached by then, and a scan landing resets the
/// project to step 1. One definition so a new control cannot honour half of it.
fn accepting_actions(studio: &Studio, dialog: &PendingDialog) -> bool {
    !studio.busy && !dialog.is_open()
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
        Step::CleanScan => acted.edit = draw_clean_scan(ui, studio, dialog, scan, controls),
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
) -> Option<EditIntent> {
    ui.add_space(8.0);
    let Some(active) = scan.active() else {
        wrapped_label(ui, "Add a scan in step 1 first.");
        return None;
    };
    let session = active.session();
    let has_centerline = session.has_centerline();
    // Reconstruct is offered once a floor trim has been applied *and* the
    // centerline it was cut along still exists.
    let has_floor_trim = session.reconstruct_available() && has_centerline;
    let ready = accepting_actions(studio, dialog);
    let mut intent = None;

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
        |ui| {
            ui.vertical_centered(|ui| {
                if ui
                    .add_enabled(ready, egui::Button::new("Weld points"))
                    .clicked()
                {
                    intent = Some(EditIntent::Weld);
                }
            });
        },
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
                    intent = Some(EditIntent::FindFloor);
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
                intent = draw_trim_row(ui, controls, ready).or(intent);
                // Nested inside the trim section, as it was pre-port: it undoes
                // part of the cut made directly above it, and reading as a
                // sibling section would make it look like a third way to trim.
                if has_floor_trim {
                    ui.add_space(SECTION_GAP);
                    intent = draw_reconstruct_row(ui, controls, ready).or(intent);
                }
            },
        );
    }

    // Secondary, and centred like the rest of the section controls.
    ui.add_space(SECTION_GAP);
    ui.vertical_centered(|ui| {
        if ui
            .add_enabled(ready, egui::Button::new("Start over"))
            .clicked()
        {
            intent = Some(EditIntent::Reset);
        }
    });

    ui.add_space(SECTION_GAP);
    centered_wrapped(
        ui,
        SUBHINT_SIZE,
        HINT_TEXT,
        "Simplify and Save arrive in the next two builds of this screen.",
    );
    intent
}

/// The trim row: a stepper for each end, then Apply trim.
///
/// ⚠ Wrapping, not a plain row. The two steppers, their four labels and the
/// button come to more than the body column is wide, so a fixed horizontal
/// layout would push the button off the panel at every window size.
fn draw_trim_row(
    ui: &mut egui::Ui,
    controls: &mut EditControls,
    ready: bool,
) -> Option<EditIntent> {
    let mut intent = None;
    let range = controls.trim_range();
    // ⚠ Grouped, not six loose widgets. The row is wider than the body column
    // so it always wraps, and egui breaks between items — ungrouped, the break
    // lands wherever it likes and strands a bare "mm" on a line of its own,
    // away from the number it belongs to. Each group is a unit the wrap can only
    // place or push down whole.
    ui.horizontal_wrapped(|ui| {
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "from tip");
            step_box(ui, &mut controls.tip_mm, range, ready);
        });
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "mm    from floor");
            step_box(ui, &mut controls.floor_mm, range, ready);
        });
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "mm");
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
    });
    intent
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
    // Grouped for the same reason as the trim row above it.
    ui.horizontal_wrapped(|ui| {
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "Shape");
            egui::ComboBox::from_id_salt("rebuilt-floor-shape")
                .selected_text(controls.shape.label())
                .show_ui(ui, |ui| {
                    for shape in FloorShape::ALL {
                        ui.selectable_value(&mut controls.shape, shape, shape.label());
                    }
                });
        });
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "from");
            step_box(ui, &mut controls.reference_mm, range, ready);
        });
        ui.horizontal(|ui| {
            ui.colored_label(CONTROL_TEXT, "mm above cut");
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
