//! Step 3's field state: how snugly the finished piece fits, and the ridges cut
//! into it.

use bevy::prelude::*;
use cf_studio_core::{PlugDraft, RidgeOptions};
use cf_studio_gui::{RidgeToggles, RingRow, StepBoxState, apply_plug, ridge_options_from_rows};

use crate::state::Studio;

/// One unit per click of every stepper on this screen — the pre-port `StepBox`
/// had no step property at all.
pub(crate) const SHAPE_STEP: i32 = 1;

/// A stepper field and the bounds it is committed inside.
///
/// ⚠ The two travel together because the range is needed twice — once to draw
/// the field, once to read it — and a field drawn against one range and read
/// against another is wrong in neither half alone.
pub(crate) struct BoundedField {
    /// The text and value the stepper edits.
    pub(crate) state: StepBoxState,
    /// `(min, max)`, in the field's own unit.
    pub(crate) range: (i32, i32),
}

impl BoundedField {
    /// A field showing `value`, editable within `range`.
    fn new(value: i32, range: (i32, i32)) -> Self {
        Self {
            state: StepBoxState::new(value),
            range,
        }
    }

    /// The value, inside its bounds.
    ///
    /// ⚠ Clamped here because typing does not commit: a number typed and left
    /// uncommitted reaches this unclamped. See [`StepBoxState`].
    pub(crate) fn value(&self) -> i32 {
        let (min, max) = self.range;
        self.state.value().clamp(min, max)
    }
}

/// The step-3 screen's field state, which outlives any one frame.
#[derive(Resource)]
pub(crate) struct ShapeControls {
    /// How far in from the scan surface the cavity sits, in millimetres.
    ///
    /// ⚠ Its bounds are fixed — nothing about the scan moves them, unlike
    /// [`crate::edit::EditControls::trim_range`].
    pub(crate) cavity_mm: BoundedField,
    /// The ridges cut into the piece's gripping face.
    pub(crate) ridges: RidgeFields,
}

impl Default for ShapeControls {
    fn default() -> Self {
        Self {
            cavity_mm: BoundedField::new(5, (0, 30)),
            ridges: RidgeFields::default(),
        }
    }
}

impl ShapeControls {
    /// The plug the fields describe, in the SDK's units.
    pub(crate) fn plug_draft(&self) -> PlugDraft {
        PlugDraft {
            cavity_inset_m: f64::from(self.cavity_mm.value()) / 1000.0,
            ridges: ridge_options_from_rows(&validated_rings(), self.ridges.toggles()),
        }
    }
}

/// The ridge editor's fields: a master switch, then a toggle and a scalar for
/// each feature.
///
/// ⚠ The grip rings are the other half of this editor and have no controls
/// yet. Until they do [`validated_rings`] stands in for them, so a piece can be
/// committed carrying rings the screen never showed.
pub(crate) struct RidgeFields {
    /// The whole feature. Off is the smooth piece.
    pub(crate) enabled: bool,
    /// The fine surface ribs — both their depth and their spacing.
    pub(crate) texture_enabled: bool,
    /// Rib depth, tenths of a millimetre.
    pub(crate) texture_depth: BoundedField,
    /// Rib pitch, tenths of a millimetre. ⚠ Governed by `texture_enabled` —
    /// it has no switch of its own.
    pub(crate) texture_spacing: BoundedField,
    /// The one-sided flattening.
    pub(crate) side_pinch_enabled: bool,
    /// Pinch depth, tenths of a millimetre.
    pub(crate) side_pinch: BoundedField,
    /// The outward pocket at the deep end.
    pub(crate) tip_relief_enabled: bool,
    /// Pocket depth, tenths of a millimetre.
    pub(crate) tip_relief: BoundedField,
    /// Whether the one-sided features are rotated off their default direction.
    pub(crate) orientation_enabled: bool,
    /// Where they sit around the channel axis, in degrees.
    pub(crate) orientation: BoundedField,
}

/// The pre-port screen's opening state: ridges off, every feature inside them
/// on, at the values [`RidgeOptions::default`] carries.
impl Default for RidgeFields {
    fn default() -> Self {
        Self {
            enabled: false,
            texture_enabled: true,
            texture_depth: BoundedField::new(15, (0, 50)),
            texture_spacing: BoundedField::new(80, (10, 300)),
            side_pinch_enabled: true,
            side_pinch: BoundedField::new(15, (0, 50)),
            tip_relief_enabled: true,
            tip_relief: BoundedField::new(30, (0, 50)),
            orientation_enabled: true,
            orientation: BoundedField::new(0, (0, 360)),
        }
    }
}

impl RidgeFields {
    /// What the toggles and fields say, in the units
    /// [`ridge_options_from_rows`] reads.
    fn toggles(&self) -> RidgeToggles {
        RidgeToggles {
            enabled: self.enabled,
            // ⚠ On, and not switchable: the rings the screen cannot show yet
            // are the ones the pre-port screen applied. See the type's note.
            rings_enabled: true,
            texture_enabled: self.texture_enabled,
            texture_depth_tenths_mm: self.texture_depth.value(),
            texture_spacing_tenths_mm: self.texture_spacing.value(),
            side_pinch_enabled: self.side_pinch_enabled,
            side_pinch_tenths_mm: self.side_pinch.value(),
            tip_relief_enabled: self.tip_relief_enabled,
            tip_relief_tenths_mm: self.tip_relief.value(),
            orientation_enabled: self.orientation_enabled,
            orientation_deg: self.orientation.value(),
        }
    }
}

/// The three grip rings the pre-port screen opened with, standing in until the
/// ring editor exists.
fn validated_rings() -> Vec<RingRow> {
    RidgeOptions::default()
        .rings
        .iter()
        .map(RingRow::from_ridge)
        .collect()
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
    fn the_draft_carries_the_cavity_field_in_meters() {
        assert_eq!(
            ShapeControls::default().plug_draft().cavity_inset_m,
            0.005,
            "the field's 5 mm is 0.005 m"
        );
    }

    /// ★★ Eleven numbers ported by hand, checked against the SDK's own
    /// validated canal rather than against a copy of themselves: every default,
    /// every unit conversion and every ring in one comparison.
    ///
    /// ⚠ The second half is what says the master switch is a switch. Gate the
    /// whole struct on it — zeroing the scalars when it is off — and the first
    /// assertion still passes.
    #[test]
    fn the_screen_opens_on_the_canal_the_sdk_calls_default() {
        let mut controls = ShapeControls::default();

        assert_eq!(
            controls.plug_draft().ridges,
            RidgeOptions::default(),
            "off, but carrying the canal — the pre-port opening state"
        );

        controls.ridges.enabled = true;
        assert_eq!(
            controls.plug_draft().ridges,
            RidgeOptions {
                enabled: true,
                ..RidgeOptions::default()
            },
            "and the master switch is the only thing it changes"
        );
    }

    /// A screen with every scalar on a different number, so a field wired to
    /// its neighbour's place shows up as the wrong one moving.
    fn distinct_scalars() -> ShapeControls {
        let mut controls = ShapeControls::default();
        for (pick, value) in SCALARS {
            pick(&mut controls).state = StepBoxState::new(value);
        }
        controls
    }

    /// The five ridge scalars, each with a value no other field carries and
    /// none of them a default. ⚠ Orientation's default is 0°, so switching it
    /// off would change nothing and its case below would prove nothing.
    const SCALARS: [(fn(&mut ShapeControls) -> &mut BoundedField, i32); 5] = [
        (|c| &mut c.ridges.texture_depth, 11),
        (|c| &mut c.ridges.texture_spacing, 99),
        (|c| &mut c.ridges.side_pinch, 22),
        (|c| &mut c.ridges.tip_relief, 33),
        (|c| &mut c.ridges.orientation, 44),
    ];

    /// ★ Eleven values copied into one struct literal across three units. A
    /// field wired to its neighbour's place is invisible while every number is
    /// its default — [`distinct_scalars`] is what makes it visible.
    #[test]
    fn each_ridge_scalar_lands_in_its_own_place_in_the_options() {
        let ridges = distinct_scalars().plug_draft().ridges;

        assert_eq!(ridges.texture_depth_m, 0.0011, "texture depth");
        assert_eq!(ridges.texture_spacing_m, 0.0099, "texture spacing");
        assert_eq!(ridges.side_pinch_depth_m, 0.0022, "side pinch");
        assert_eq!(ridges.tip_relief_depth_m, 0.0033, "tip relief");
        assert_eq!(ridges.orientation_deg, 44.0, "orientation");
    }

    /// One feature's switch: what to call it, the flag it drives, and the
    /// number that has to go to zero when it is off.
    type ToggleCase = (
        &'static str,
        fn(&mut ShapeControls) -> &mut bool,
        fn(&RidgeOptions) -> f64,
    );

    /// ★ Four toggles beside five scalars: one wired to its neighbour zeroes
    /// the wrong feature, and every number left on screen still looks
    /// plausible.
    #[test]
    fn each_ridge_toggle_zeroes_its_own_feature_and_leaves_the_rest_standing() {
        let cases: [ToggleCase; 4] = [
            (
                "texture",
                |c| &mut c.ridges.texture_enabled,
                |o| o.texture_depth_m,
            ),
            (
                "side pinch",
                |c| &mut c.ridges.side_pinch_enabled,
                |o| o.side_pinch_depth_m,
            ),
            (
                "tip relief",
                |c| &mut c.ridges.tip_relief_enabled,
                |o| o.tip_relief_depth_m,
            ),
            (
                "orientation",
                |c| &mut c.ridges.orientation_enabled,
                |o| o.orientation_deg,
            ),
        ];
        let all_on = distinct_scalars().plug_draft().ridges;

        for (name, toggle, read) in cases {
            let mut controls = distinct_scalars();
            *toggle(&mut controls) = false;
            let off = controls.plug_draft().ridges;

            assert_eq!(read(&off), 0.0, "{name} off must zero its own feature");
            for (other, _, read_other) in cases {
                assert!(
                    other == name || read_other(&off) == read_other(&all_on),
                    "{name} off moved {other} as well"
                );
            }
            assert_eq!(
                off.texture_spacing_m, all_on.texture_spacing_m,
                "{name} off must leave the spacing, which no toggle gates"
            );
        }
    }

    /// ⚠ Typing does not commit, so a number typed and left uncommitted reaches
    /// [`ShapeControls::plug_draft`] unclamped — the path the bounds exist for.
    ///
    /// ⚠ Both edges of every field. A `min` where a `clamp` belongs holds the
    /// top and lets a negative through, and a negative inset offsets the plug
    /// *outward* — a cavity the scan no longer fits.
    ///
    /// ⚠ Asserted through the committed plug, not through
    /// [`BoundedField::value`]: a caller reading `state` directly would bypass
    /// the clamp with the field's own gate still green.
    #[test]
    fn every_field_commits_inside_the_bounds_the_pre_port_screen_had() {
        // The pre-port `StepBox` minimum/maximum, in each field's own unit.
        let cases: [(
            &str,
            fn(&mut ShapeControls) -> &mut BoundedField,
            (i32, i32),
        ); 6] = [
            ("cavity", |c| &mut c.cavity_mm, (0, 30)),
            ("texture depth", |c| &mut c.ridges.texture_depth, (0, 50)),
            (
                "texture spacing",
                |c| &mut c.ridges.texture_spacing,
                (10, 300),
            ),
            ("side pinch", |c| &mut c.ridges.side_pinch, (0, 50)),
            ("tip relief", |c| &mut c.ridges.tip_relief, (0, 50)),
            ("orientation", |c| &mut c.ridges.orientation, (0, 360)),
        ];

        for (name, pick, (min, max)) in cases {
            assert_eq!(
                pick(&mut ShapeControls::default()).range,
                (min, max),
                "{name} is edited inside the bounds the pre-port screen gave it"
            );

            for (typed, bound) in [(max + 1, max), (min - 1, min)] {
                let mut typed_over = ShapeControls::default();
                let field = pick(&mut typed_over);
                *field.state.text_mut() = typed.to_string();
                field.state.on_typed();

                let mut at_bound = ShapeControls::default();
                pick(&mut at_bound).state = StepBoxState::new(bound);

                assert_eq!(
                    typed_over.plug_draft(),
                    at_bound.plug_draft(),
                    "{name}: {typed} must commit as {bound}"
                );
            }
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
