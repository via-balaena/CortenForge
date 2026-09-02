//! The first-launch safety + age gate.
//!
//! ⚠⚠ **The terms below are legal text and are reproduced verbatim.** They
//! mirror `DISCLAIMER.md` / `NOTICE` at the repository root. Do not reword,
//! shorten, or "improve" them while porting UI around them — if the repository
//! documents change, change these to match, not the other way round.
//!
//! Shown on **every** launch: acceptance lives in this screen's local state and
//! is never persisted, so the gate cannot be skipped by a previous session.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::state::Screen;
use crate::widgets::wrapped_label;

/// The terms, in display order. The last is emphasised — it is the age gate.
const TERMS: [&str; 5] = [
    "Cendrillon is software provided by Via Balaena that helps you design molds and casting plans for custom silicone objects from your own 3D scans. It is a design-assistance tool only.",
    "The software and everything it produces are provided \"as is,\" without warranties of any kind, express or implied — including merchantability, fitness for a particular purpose, accuracy, or safety. Via Balaena does not warrant that any design, mold, or finished object is safe or suitable for any use, including contact with or insertion into the body.",
    "You use this software — and make and use anything created with it — entirely at your own risk. You alone are responsible for choosing body-safe materials and for proper mixing, curing, cleaning, and hygiene. Improperly cured or non-body-safe silicone can cause serious injury.",
    "Cendrillon is not a medical device and makes no medical, therapeutic, or health claims.",
    "To the fullest extent permitted by law, Via Balaena and its contributors are not liable for any injury, loss, or damage arising from the software or the objects you create with it, and you agree to indemnify Via Balaena against any claim arising from your use of either. If any part of these terms is unenforceable, the rest still applies.",
];

/// The age gate, shown bold beneath the terms.
const AGE_GATE: &str = "This software and the objects it helps you create are intended for adults and only for lawful use. By continuing you confirm you are at least 18 years old, or the age of majority where you live if that is higher.";

/// The confirmation checkbox's label — the thing being agreed to.
const CONFIRM: &str = "I have read and accept these terms, and I assume all risk.";

/// Draw the gate. `agreed` is this screen's own state and resets with the app.
pub(crate) fn waiver_screen(
    mut contexts: EguiContexts,
    mut agreed: Local<bool>,
    mut next: ResMut<NextState<Screen>>,
    mut exit: MessageWriter<AppExit>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.vertical_centered(|ui| {
            ui.set_max_width(640.0);
            ui.add_space(12.0);
            ui.heading(egui::RichText::new("Before you begin").size(30.0).strong());
            ui.add_space(8.0);

            // The terms scroll; the checkbox and buttons below must stay
            // reachable on a short window, so they sit outside the scroll area.
            let button_room = 96.0;
            egui::ScrollArea::vertical()
                .max_height((ui.available_height() - button_room).max(120.0))
                .show(ui, |ui| {
                    ui.vertical(|ui| {
                        for term in TERMS {
                            wrapped_label(ui, term);
                            ui.add_space(10.0);
                        }
                        ui.add(egui::Label::new(egui::RichText::new(AGE_GATE).strong()).wrap());
                    });
                });

            ui.add_space(10.0);
            ui.checkbox(&mut agreed, CONFIRM);
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button("Quit").clicked() {
                    exit.write(AppExit::Success);
                }
                // Clamped: a narrow window can leave less room than the
                // button needs, and a negative space is not a layout.
                ui.add_space((ui.available_width() - 150.0).max(0.0));
                if ui
                    .add_enabled(*agreed, egui::Button::new("Agree & Continue"))
                    .clicked()
                {
                    next.set(Screen::Wizard);
                }
            });
        });
    });
    Ok(())
}
