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

/// The gate's own width, and the column every gate test lays out in.
pub(crate) const GATE_WIDTH: f32 = 640.0;

/// How the gate was answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum WaiverChoice {
    /// Leave without accepting.
    Quit,
    /// Accepted; the wizard may start.
    Continue,
}

/// Lay the gate out and report what was clicked.
///
/// ⚠ Extracted from [`waiver_screen`] for the reason the wizard's own
/// executors are: a Bevy system is the one shape no test can drive, and this is
/// a legal and age gate — that it refuses to open until the box is ticked is
/// the single most consequential behaviour in the app.
pub(crate) fn draw_waiver(ui: &mut egui::Ui, agreed: &mut bool) -> Option<WaiverChoice> {
    let mut choice = None;
    ui.vertical_centered(|ui| {
        ui.set_max_width(GATE_WIDTH);
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
        ui.checkbox(agreed, CONFIRM);
        ui.add_space(8.0);
        ui.horizontal(|ui| {
            if ui.button("Quit").clicked() {
                choice = Some(WaiverChoice::Quit);
            }
            // Clamped: a narrow window can leave less room than the
            // button needs, and a negative space is not a layout.
            ui.add_space((ui.available_width() - 150.0).max(0.0));
            if ui
                .add_enabled(*agreed, egui::Button::new("Agree & Continue"))
                .clicked()
            {
                choice = Some(WaiverChoice::Continue);
            }
        });
    });
    choice
}

/// Draw the gate. `agreed` is this screen's own state and resets with the app.
pub(crate) fn waiver_screen(
    mut contexts: EguiContexts,
    mut agreed: Local<bool>,
    mut next: ResMut<NextState<Screen>>,
    mut exit: MessageWriter<AppExit>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let mut choice = None;
    egui::CentralPanel::default().show(ctx, |ui| {
        choice = draw_waiver(ui, &mut agreed);
    });
    match choice {
        Some(WaiverChoice::Quit) => {
            exit.write(AppExit::Success);
        }
        Some(WaiverChoice::Continue) => next.set(Screen::Wizard),
        None => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use egui::accesskit::Role;
    use egui_kittest::Harness;
    use egui_kittest::kittest::{NodeT, Queryable};

    use super::*;
    use crate::egui_harness::{self, Painted, begin, click_on, end};
    use crate::panel::tests::assert_renders;

    /// Stand `waiver_screen` up as the Bevy system it is, with the egui pass
    /// `bevy_egui`'s plugin would normally open around it.
    fn app_running_the_real_system() -> App {
        use bevy::state::app::StatesPlugin;

        let mut app = egui_harness::app();
        app.add_plugins(StatesPlugin)
            .init_state::<Screen>()
            .add_message::<AppExit>()
            .add_systems(Update, (begin, waiver_screen, end).chain());
        app
    }

    /// ★★ The whole stack, as the app runs it: the Bevy system draws the gate,
    /// a click on the box arms it, and a click on the answer opens the wizard.
    ///
    /// ⚠ The one thing every other test here cannot reach. `draw_waiver` is
    /// tested directly and `waiver_screen` is six lines of routing around it —
    /// but replacing the whole system with a no-op passed everything, because
    /// drawing has no effect on the ECS to observe. This observes the drawing.
    #[test]
    fn the_gate_runs_as_a_system_and_opens_only_after_both_clicks() {
        let mut app = app_running_the_real_system();
        app.update();

        let painted = &app.world().resource::<Painted>().0;
        assert!(
            painted.iter().any(|(text, _)| text == "Before you begin"),
            "the system painted the gate: {painted:?}"
        );

        // Ticking the box is not accepting: the wizard must still be shut.
        click_on(&mut app, CONFIRM);
        assert_eq!(
            app.world().resource::<State<Screen>>().get(),
            &Screen::Waiver,
            "ticking the box alone does not open the wizard"
        );

        click_on(&mut app, "Agree & Continue");
        app.update();
        assert_eq!(
            app.world().resource::<State<Screen>>().get(),
            &Screen::Wizard,
            "and the answer does"
        );
    }

    /// `main.rs`'s resize floor and its opening resolution — the narrowest and
    /// the widest this gate is normally shown at.
    const NARROWEST: (f32, f32) = (900.0, 850.0);
    const OPENING: (f32, f32) = (1280.0, 900.0);

    /// Lay the gate out at `size` with the box ticked or not, optionally click
    /// `pick`, and report its controls and the answer it gave.
    fn gate(
        size: (f32, f32),
        ticked: bool,
        pick: Option<&str>,
    ) -> (Vec<(String, bool, egui::Rect)>, Option<WaiverChoice>) {
        let agreed = std::cell::Cell::new(ticked);
        let answer = std::cell::Cell::new(None);
        let mut harness = Harness::builder()
            .with_size(egui::Vec2::new(size.0, size.1))
            .build(|ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    let mut state = agreed.get();
                    if let Some(choice) = draw_waiver(ui, &mut state) {
                        answer.set(Some(choice));
                    }
                    agreed.set(state);
                });
            });
        harness.ctx.set_fonts(crate::plugin::font_definitions());
        harness.run();
        let controls = harness
            .root()
            .children_recursive()
            .filter(|node| matches!(node.accesskit_node().role(), Role::Button | Role::CheckBox))
            .map(|node| {
                (
                    node.accesskit_node().label().unwrap_or_default(),
                    node.accesskit_node().is_disabled(),
                    node.rect(),
                )
            })
            .collect();
        if let Some(label) = pick {
            harness.get_by_label(label).click();
            harness.run();
        }
        (controls, answer.get())
    }

    fn disabled(controls: &[(String, bool, egui::Rect)], label: &str) -> bool {
        controls
            .iter()
            .find(|(name, _, _)| name == label)
            .map(|(_, off, _)| *off)
            .expect("the gate must offer this control")
    }

    /// ★★ The single most consequential behaviour in the app: this is a
    /// liability waiver and an 18+ age gate, and until now nothing proved it
    /// gates. It is shown on every launch and is never persisted.
    #[test]
    fn the_gate_does_not_open_until_the_box_is_ticked() {
        let (unticked, answer) = gate(OPENING, false, None);
        assert!(
            disabled(&unticked, "Agree & Continue"),
            "unticked, there is no way through: {unticked:?}"
        );
        assert_eq!(answer, None, "and nothing was answered by drawing it");

        let (ticked, _) = gate(OPENING, true, None);
        assert!(
            !disabled(&ticked, "Agree & Continue"),
            "ticked, the way through opens"
        );

        assert_eq!(
            gate(OPENING, true, Some("Agree & Continue")).1,
            Some(WaiverChoice::Continue),
            "and taking it accepts"
        );
    }

    /// ⚠ Quit must work without accepting anything. A gate you can only leave
    /// by agreeing is not a gate.
    #[test]
    fn quit_needs_no_agreement() {
        assert!(!disabled(&gate(OPENING, false, None).0, "Quit"));
        assert_eq!(
            gate(OPENING, false, Some("Quit")).1,
            Some(WaiverChoice::Quit)
        );
    }

    /// ⚠ At both ends of the range `main.rs` allows. The row places its second
    /// button with a hand-computed spacer, and egui clips what overflows
    /// rather than wrapping it — the #878 failure, on the one screen every
    /// launch has to pass through.
    #[test]
    fn the_gates_controls_fit_at_every_width_the_window_allows() {
        for size in [NARROWEST, OPENING] {
            let (controls, _) = gate(size, true, None);
            let left = controls
                .iter()
                .map(|(_, _, r)| r.min.x)
                .fold(f32::MAX, f32::min);
            let right = controls
                .iter()
                .map(|(_, _, r)| r.max.x)
                .fold(f32::MIN, f32::max);
            assert!(
                right - left <= GATE_WIDTH,
                "at {size:?} the controls span {:.1} px of a {GATE_WIDTH} px gate: {controls:?}",
                right - left,
            );
            for (label, _, rect) in &controls {
                assert!(
                    rect.min.x >= 0.0 && rect.max.x <= size.0,
                    "{label} is laid out at {rect:?}, off a {size:?} window"
                );
            }

            // ⚠ The spacer's actual job. Total width fits either way, so a
            // collapsed spacer passes the check above while putting the answer
            // that leaves and the answer that accepts 12 px apart — a mis-click
            // on a liability gate. Measured 457 px at both widths.
            let edge = |name: &str, right: bool| {
                controls
                    .iter()
                    .find(|(label, _, _)| label == name)
                    .map(|(_, _, r)| if right { r.max.x } else { r.min.x })
                    .expect("the gate must offer this control")
            };
            let apart = edge("Agree & Continue", false) - edge("Quit", true);
            assert!(
                apart >= 100.0,
                "at {size:?} the two answers are only {apart:.1} px apart"
            );
        }
    }

    /// ⚠ Legal text the user is asked to accept. A tofu box in it is a term
    /// they cannot read — and `format_save_done` proved this app ships glyphs
    /// its fonts do not have.
    #[test]
    fn every_word_the_gate_shows_is_drawable() {
        let mut harness = Harness::new_ui(|_| {});
        harness.ctx.set_fonts(crate::plugin::font_definitions());
        harness.run();

        for text in TERMS {
            assert_renders(&harness.ctx, text);
        }
        for text in [
            AGE_GATE,
            CONFIRM,
            "Before you begin",
            "Quit",
            "Agree & Continue",
        ] {
            assert_renders(&harness.ctx, text);
        }
    }
}
