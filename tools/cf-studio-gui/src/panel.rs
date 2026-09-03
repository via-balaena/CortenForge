//! The wizard chrome — checklist, header, footer nav — and the per-step bodies.
//!
//! The panel decides nothing. It renders from the lib's plain functions
//! ([`step_rows`], [`nav_state`], the `format_*` family) and turns clicks into
//! an [`Intent`], which [`apply_intent`] executes in one place. Keeping the
//! egui closure free of state transitions is what makes the transitions
//! reviewable — and testable, since they are all methods on `Studio`.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use cf_studio_core::Step;
use cf_studio_gui::{
    format_pour_active, format_pour_plan, nav_state, pour_countdown, print_step_summary, step_rows,
};

use crate::dialogs::{DialogKind, PendingDialog};
use crate::state::Studio;
use crate::widgets::{
    ACTIVE_TEXT, ERROR_TEXT, GOOD_FILL, GOOD_TEXT, WARN_TEXT, card, wrapped_colored, wrapped_label,
};

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
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let mut intent = None;

    egui::SidePanel::left("checklist")
        .resizable(false)
        .exact_width(260.0)
        .show(ctx, |ui| draw_checklist(ui, &studio));

    egui::TopBottomPanel::bottom("nav").show(ctx, |ui| {
        intent = draw_nav(ui, &studio).or(intent);
    });

    egui::SidePanel::right("body")
        .resizable(false)
        .exact_width(420.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                intent = draw_body(ui, &studio, &dialog).or(intent);
            });
        });

    if let Some(intent) = intent {
        apply_intent(intent, &mut studio, &mut dialog);
    }
    Ok(())
}

/// The seven-step progress list. `✓`/`○` is real completion; the arrow marks
/// the screen being viewed — two different things that must not be conflated.
fn draw_checklist(ui: &mut egui::Ui, studio: &Studio) {
    ui.add_space(8.0);
    ui.heading(&studio.project.name);
    ui.separator();
    for row in step_rows(&studio.project, studio.cursor.viewed()) {
        let mark = if row.done { "✓" } else { "○" };
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

/// Back / Help / Next, gated by [`nav_state`].
fn draw_nav(ui: &mut egui::Ui, studio: &Studio) -> Option<Intent> {
    let nav = nav_state(&studio.project, studio.cursor.viewed());
    let mut intent = None;
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        if ui
            .add_enabled(nav.can_back && !studio.busy, egui::Button::new("← Back"))
            .clicked()
        {
            intent = Some(Intent::Back);
        }
        ui.add_enabled(false, egui::Button::new("Help"))
            .on_disabled_hover_text("Per-step guidance arrives with the ported steps.");
        if ui
            .add_enabled(nav.can_next && !studio.busy, egui::Button::new("Next →"))
            .clicked()
        {
            intent = Some(Intent::Next);
        }
    });
    ui.add_space(6.0);
    intent
}

/// The body for the viewed step, plus the step message beneath it.
fn draw_body(ui: &mut egui::Ui, studio: &Studio, dialog: &PendingDialog) -> Option<Intent> {
    let viewed = studio.cursor.viewed();
    ui.add_space(8.0);
    ui.heading(format!(
        "Step {} of {} — {}",
        viewed.number(),
        Step::TOTAL,
        viewed.title()
    ));
    ui.separator();

    // ⚠ Exhaustive on purpose, with no catch-all arm. A `_ =>` (or a list of
    // "not ported yet" steps) would silently render one step's screen for a
    // step added later; this way the compiler names the new arm.
    let intent = match viewed {
        Step::AddScan => draw_add_scan(ui, studio, dialog),
        Step::CleanScan | Step::ShapePiece | Step::DesignLayers | Step::MakeMolds => {
            draw_porting_notice(ui);
            None
        }
        Step::Print => draw_print(ui, studio, dialog),
        Step::Pour => draw_pour(ui, studio),
    };

    if let Some(message) = &studio.message {
        ui.add_space(12.0);
        match message {
            Ok(text) => wrapped_label(ui, text.clone()),
            Err(text) => wrapped_colored(ui, ERROR_TEXT, text.clone()),
        }
    }
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
    let blocked = studio.busy || dialog.is_open();
    if ui.add_enabled(!blocked, egui::Button::new(label)).clicked() {
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

/// Steps 2–5 during the Slint→Bevy port. Says what is missing and that the work
/// is not lost, rather than showing an empty screen that reads as a bug.
fn draw_porting_notice(ui: &mut egui::Ui) {
    ui.add_space(8.0);
    wrapped_label(
        ui,
        "This step is being rebuilt on the new interface and isn't available in \
         this build yet. Its logic is unchanged — only the screen is being \
         redrawn. Steps 6 and 7 work today.",
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
    let blocked = studio.busy || dialog.is_open();
    let label = if studio.busy {
        "Saving…"
    } else if exported {
        "Save again…"
    } else {
        "Save files for printing…"
    };
    ui.horizontal(|ui| {
        if ui.add_enabled(!blocked, egui::Button::new(label)).clicked() {
            intent = Some(Intent::ExportPrint);
        }
        if exported
            && ui
                .add_enabled(!blocked, egui::Button::new("Open folder"))
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
