//! Step 3's field state: how snugly the finished piece fits, and the ridges cut
//! into it.

use bevy::prelude::*;
use cf_studio_core::{PlugDraft, RidgeOptions};
use cf_studio_gui::{
    BoundedField, RidgeToggles, RingRow, apply_plug, m_to_mm, m_to_tenths_mm,
    ridge_options_from_rows, whole_degrees,
};

use crate::state::Studio;

/// The cavity inset's stepper range, in whole millimetres.
const CAVITY_RANGE: (i32, i32) = (0, 30);

/// What a fresh screen shows before anything is committed.
const CAVITY_DEFAULT_MM: i32 = 5;

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
    /// The committed plug these fields were last agreed with.
    ///
    /// ⚠ The whole plug, not a summary: [`drive_shape_controls`] re-derives the
    /// fields the moment this stops matching the project, so anything it left
    /// out would be a change the screen never followed.
    followed: Option<PlugDraft>,
}

impl Default for ShapeControls {
    fn default() -> Self {
        Self {
            cavity_mm: BoundedField::new(CAVITY_DEFAULT_MM, CAVITY_RANGE),
            ridges: RidgeFields::default(),
            followed: None,
        }
    }
}

impl ShapeControls {
    /// The plug the fields describe, in the SDK's units.
    pub(crate) fn plug_draft(&self) -> PlugDraft {
        PlugDraft {
            cavity_inset_m: f64::from(self.cavity_mm.value()) / 1000.0,
            ridges: ridge_options_from_rows(&self.ridges.rings, self.ridges.toggles()),
        }
    }

    /// The fields that would cut `plug` — the inverse of [`Self::plug_draft`].
    ///
    /// ⚠ Lossy in one direction on purpose: the steppers edit whole
    /// millimetres, tenths of a millimetre and whole degrees, so a plug written
    /// off that grid — by hand, or by the CLI, which has no steppers — is shown
    /// rounded and clamped. What the fields say is what the piece is cut at.
    pub(crate) fn from_plug(plug: &PlugDraft) -> Self {
        let (min, max) = CAVITY_RANGE;
        Self {
            cavity_mm: BoundedField::new(
                m_to_mm(plug.cavity_inset_m).clamp(min, max),
                CAVITY_RANGE,
            ),
            ridges: RidgeFields::from_options(&plug.ridges),
            followed: Some(plug.clone()),
        }
    }
}

/// Keep step 3's fields in step with the committed plug.
///
/// ⚠ A per-frame reconcile, not an entry hook — [`crate::molds::drive_part_picker`]
/// is the house pattern. Unlike that one it is *not* gated on the viewed step:
/// its trigger is the exact plug the fields were last agreed with, so there is
/// no frame on which running it could throw the user's editing away.
///
/// ★ Nothing in a session can desync these — the fields are the only writer of
/// the plug. Loading a project from disk can, which is what this is for.
pub(crate) fn drive_shape_controls(mut controls: ResMut<ShapeControls>, studio: Res<Studio>) {
    let plug = studio.project.plug();
    if controls.followed.as_ref() == plug {
        return;
    }
    match plug {
        Some(plug) => *controls = ShapeControls::from_plug(plug),
        // ⚠ Nothing committed means nothing to follow, and the fields are then
        // the user's own. Re-picking the scan or re-cleaning it clears the
        // plug, and resetting the screen at that point would throw away numbers
        // they typed rather than reproduce a project. The stamp still goes: it
        // names an artifact that no longer exists.
        None => controls.followed = None,
    }
}

/// The ridge editor's fields: a master switch, the grip rings, then a toggle
/// and a scalar for each of the remaining features.
#[derive(Debug, PartialEq, Eq)]
pub(crate) struct RidgeFields {
    /// The whole feature. Off is the smooth piece.
    pub(crate) enabled: bool,
    /// The grip rings on/off.
    pub(crate) rings_enabled: bool,
    /// The rings themselves, in the order they are drawn.
    pub(crate) rings: Vec<RingRow>,
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
            rings_enabled: true,
            rings: RidgeOptions::default()
                .rings
                .iter()
                .map(RingRow::from_ridge)
                .collect(),
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
    /// The fields that would cut `options` — the inverse of [`Self::toggles`]
    /// composed with [`ridge_options_from_rows`].
    ///
    /// ⚠ `gate_ridge_options` zeroes a disabled feature's scalar, so the number
    /// sitting behind an unticked toggle is not in the artifact at all. It
    /// comes back as this screen's own default rather than as 0, which would
    /// make re-ticking the feature do nothing visible.
    pub(crate) fn from_options(options: &RidgeOptions) -> Self {
        let defaults = Self::default();
        // Every scalar here is drawn by a stepper, which prints the number it
        // is handed — so an off-grid or out-of-range artifact must be rounded
        // and clamped on the way in, or the screen states a value the piece is
        // not cut at.
        let clamped = |value: i32, field: BoundedField| {
            let (min, max) = field.range;
            BoundedField::new(value.clamp(min, max), field.range)
        };
        // A zeroed depth is the gated form of "this feature is off".
        let depth = |depth_m: f64, default: BoundedField| {
            if depth_m == 0.0 {
                (false, default)
            } else {
                (true, clamped(m_to_tenths_mm(depth_m), default))
            }
        };
        let (texture_enabled, texture_depth) =
            depth(options.texture_depth_m, defaults.texture_depth);
        let (side_pinch_enabled, side_pinch) =
            depth(options.side_pinch_depth_m, defaults.side_pinch);
        let (tip_relief_enabled, tip_relief) =
            depth(options.tip_relief_depth_m, defaults.tip_relief);
        Self {
            enabled: options.enabled,
            // An empty ring list is the gated form of "rings off", and the rows
            // behind the unticked box are this screen's own — as in a fresh
            // session, which also opens with rings to turn on.
            rings_enabled: !options.rings.is_empty(),
            rings: if options.rings.is_empty() {
                defaults.rings
            } else {
                options.rings.iter().map(RingRow::from_ridge).collect()
            },
            texture_enabled,
            texture_depth,
            // ⚠ Carried whether or not the texture is on: the pitch passes
            // through `gate_ridge_options` ungated, being inert at zero depth.
            texture_spacing: clamped(
                m_to_tenths_mm(options.texture_spacing_m),
                defaults.texture_spacing,
            ),
            side_pinch_enabled,
            side_pinch,
            tip_relief_enabled,
            tip_relief,
            // ⚠ Always on, where the four above are read back: 0° *is* the
            // default direction, so "off" and "on at 0°" cut the same piece —
            // and "on at 0°" is the state a fresh screen opens in.
            orientation_enabled: true,
            orientation: clamped(whole_degrees(options.orientation_deg), defaults.orientation),
        }
    }

    /// What the toggles and fields say, in the units
    /// [`ridge_options_from_rows`] reads.
    fn toggles(&self) -> RidgeToggles {
        RidgeToggles {
            enabled: self.enabled,
            rings_enabled: self.rings_enabled,
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

    /// Add a ring mid-channel, a conservative 2 mm deep, as wide as the ones
    /// that ship.
    pub(crate) fn add_ring(&mut self) {
        self.rings.push(RingRow::new(50, 20, 4));
    }
}

/// Commit the shaped plug, and move on to the layer stack if it took.
pub(crate) fn commit_plug(draft: PlugDraft, controls: &mut ShapeControls, studio: &mut Studio) {
    let outcome = apply_plug(&mut studio.project, draft);
    if outcome.is_ok() {
        // ⚠ The fields ARE the plug that just landed, so say so. Left for
        // [`drive_shape_controls`] to work out, it would re-derive them from
        // the artifact and throw away every number parked behind an unticked
        // toggle — `gate_ridge_options` does not record those.
        controls.followed = studio.project.plug().cloned();
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

    use bevy::ecs::system::RunSystemOnce;
    use cf_studio_core::{PrepInput, Project, RidgeRing, ScanInput, Step};
    use cf_studio_gui::{StepBoxState, WizardCursor};

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

    /// ★★ Every ported default, unit conversion and ring in one comparison,
    /// against the SDK's own validated canal rather than against a copy of
    /// itself.
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

    /// A screen with every scalar on a different number, none of them a
    /// default, so a field wired to its neighbour's place shows up as the wrong
    /// one moving.
    ///
    /// ⚠ Orientation especially. Its default is 0°, so a case that left it
    /// there would prove nothing when its switch is turned off.
    fn distinct_scalars() -> ShapeControls {
        let mut controls = ShapeControls::default();
        let ridges = &mut controls.ridges;
        ridges.texture_depth.state = StepBoxState::new(11);
        ridges.texture_spacing.state = StepBoxState::new(99);
        ridges.side_pinch.state = StepBoxState::new(22);
        ridges.tip_relief.state = StepBoxState::new(33);
        ridges.orientation.state = StepBoxState::new(44);
        controls
    }

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

    /// ★ Three rows of three fields, all flattened into one unit-converted
    /// vector: a row or a field wired to its neighbour's place cuts a
    /// plausible ring in the wrong groove.
    ///
    /// ⚠ Distinct in every one of the nine, and none of them a default. Ring 2
    /// and ring 3 open on the same depth and every ring on the same width, so
    /// a screen that read row 1 three times would pass on the shipped values.
    #[test]
    fn each_ring_row_lands_in_its_own_place_in_the_carve() {
        let mut controls = ShapeControls::default();
        controls.ridges.rings = vec![
            RingRow::new(11, 21, 31),
            RingRow::new(12, 22, 32),
            RingRow::new(13, 23, 33),
        ];

        assert_eq!(
            controls.plug_draft().ridges.rings,
            [
                RidgeRing {
                    position_frac: 0.11,
                    depth_m: 0.0021,
                    half_width_frac: 0.31,
                },
                RidgeRing {
                    position_frac: 0.12,
                    depth_m: 0.0022,
                    half_width_frac: 0.32,
                },
                RidgeRing {
                    position_frac: 0.13,
                    depth_m: 0.0023,
                    half_width_frac: 0.33,
                },
            ],
            "percent and tenths of a millimetre, row by row"
        );
    }

    /// ⚠ The rings are the one feature whose switch drops rows rather than
    /// zeroing a number, so no [`ToggleCase`] below reaches it.
    ///
    /// ⚠ Two-sided: without the second assertion a screen carrying no rings at
    /// all passes the first.
    #[test]
    fn the_ring_switch_drops_the_rows_and_leaves_the_rest_standing() {
        let on = distinct_scalars().plug_draft().ridges;
        let mut controls = distinct_scalars();
        controls.ridges.rings_enabled = false;

        let off = controls.plug_draft().ridges;

        assert!(off.rings.is_empty(), "the switch drops the rows");
        assert!(!on.rings.is_empty(), "which were being cut with it on");
        assert_eq!(
            RidgeOptions {
                rings: on.rings.clone(),
                ..off
            },
            on,
            "and nothing but the rings moved"
        );
    }

    /// ⚠ The values, not just the count: an add that appended a copy of a ring
    /// that ships gets the length right.
    #[test]
    fn an_added_ring_lands_mid_channel_at_the_shipped_width() {
        let mut controls = ShapeControls::default();
        let before = controls.ridges.rings.len();

        controls.ridges.add_ring();

        assert_eq!(controls.ridges.rings.len(), before + 1, "one more row");
        let added = controls
            .ridges
            .rings
            .last()
            .expect("a ring was just pushed");
        assert_eq!(
            (
                added.position.value(),
                added.depth.value(),
                added.width.value()
            ),
            (50, 20, 4),
            "mid-channel, a conservative 2 mm deep"
        );
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
        ); 9] = [
            ("cavity", |c| &mut c.cavity_mm, (0, 30)),
            // Ring 1 stands for all three: every row is built by the one
            // constructor, and which row a field belongs to is
            // `each_ring_row_lands_in_its_own_place_in_the_carve`'s claim.
            (
                "ring position",
                |c| &mut c.ridges.rings[0].position,
                (0, 100),
            ),
            ("ring depth", |c| &mut c.ridges.rings[0].depth, (0, 100)),
            ("ring width", |c| &mut c.ridges.rings[0].width, (1, 50)),
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

        let mut controls = ShapeControls::default();
        commit_plug(controls.plug_draft(), &mut controls, &mut studio);

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

        let mut controls = ShapeControls::default();
        commit_plug(controls.plug_draft(), &mut controls, &mut studio);

        assert_eq!(studio.cursor.viewed(), Step::ShapePiece, "it stays put");
        assert!(studio.project.plug().is_none(), "and records nothing");
        assert!(
            matches!(&studio.message, Some(Err(_))),
            "with the reason on screen: {:?}",
            studio.message
        );
    }

    // ── the fields follow the committed plug ────────────────────────────

    /// A screen shaped into a piece nothing else in this file produces: every
    /// scalar on its own number, the ridges actually on, and a cavity off the
    /// default — so "followed the plug" and "never moved" cannot pass as each
    /// other.
    fn a_shaped_screen() -> ShapeControls {
        let mut controls = distinct_scalars();
        controls.cavity_mm = BoundedField::new(12, CAVITY_RANGE);
        controls.ridges.enabled = true;
        controls
    }

    /// `project`, with `controls`' piece committed to it.
    fn shaped_into(controls: &ShapeControls) -> Project {
        let mut project = ready_to_shape();
        project
            .set_plug(controls.plug_draft())
            .expect("a cleaned scan accepts a plug");
        project
    }

    /// Run the real reconcile once and hand back what it left.
    fn reconcile(studio: Studio, controls: ShapeControls) -> ShapeControls {
        let mut app = App::new();
        app.insert_resource(studio).insert_resource(controls);
        app.world_mut()
            .run_system_once(drive_shape_controls)
            .expect("the reconcile must run");
        app.world_mut()
            .remove_resource::<ShapeControls>()
            .expect("controls survive")
    }

    /// ★★★ The property the reconcile rests on: what the fields would cut,
    /// read back into fields, cuts the same piece. Without it a project drifts
    /// a little every time it is reopened.
    ///
    /// ⚠ The oracle is `plug_draft` — the function the app commits through —
    /// not a second copy of `from_plug`'s arithmetic, which would agree with
    /// its own mistakes.
    #[test]
    fn reading_a_plug_back_into_the_fields_cuts_the_same_piece() {
        let mut all_off = a_shaped_screen();
        all_off.ridges.rings_enabled = false;
        all_off.ridges.texture_enabled = false;
        all_off.ridges.side_pinch_enabled = false;
        all_off.ridges.tip_relief_enabled = false;
        all_off.ridges.orientation_enabled = false;

        let mut some_off = a_shaped_screen();
        some_off.ridges.texture_enabled = false;
        some_off.ridges.side_pinch_enabled = false;

        for (label, controls) in [
            ("the screen's own opening state", ShapeControls::default()),
            (
                "every feature on, each on its own number",
                a_shaped_screen(),
            ),
            ("every feature switched off", all_off),
            ("rings, tip relief and orientation only", some_off),
        ] {
            let plug = controls.plug_draft();
            assert_eq!(
                ShapeControls::from_plug(&plug).plug_draft(),
                plug,
                "{label} does not survive the round trip"
            );
        }
    }

    /// ★ Reopening a project shaped at the defaults must hand back the screen
    /// the app launches with — the same toggles and the same numbers, not
    /// merely the same piece.
    ///
    /// ⚠ This is the only gate on the one read-back that is *not* "zero means
    /// off": 0° is a direction, not an absence, so the orientation switch
    /// comes back on. Every round trip below passes either way, because both
    /// readings cut the identical piece.
    #[test]
    fn the_opening_state_reads_back_as_the_opening_state() {
        let opening = ShapeControls::default();

        let read_back = ShapeControls::from_plug(&opening.plug_draft());

        assert_eq!(read_back.cavity_mm, opening.cavity_mm, "the cavity field");
        assert_eq!(read_back.ridges, opening.ridges, "and every ridge field");
    }

    /// ⚠ The number behind an unticked toggle is not in the artifact —
    /// `gate_ridge_options` zeroes it. Reading it back as 0 rather than as the
    /// screen's own default would make re-ticking the feature do nothing.
    #[test]
    fn a_feature_that_was_off_comes_back_with_a_number_to_turn_on() {
        let mut off = a_shaped_screen();
        off.ridges.texture_enabled = false;

        let read_back = ShapeControls::from_plug(&off.plug_draft()).ridges;

        assert!(!read_back.texture_enabled, "still off");
        assert_eq!(
            read_back.texture_depth.value(),
            RidgeFields::default().texture_depth.value(),
            "and showing a depth that would cut something once ticked"
        );
    }

    /// ⚠ A stepper prints the number it is handed, so a project written off
    /// the field's grid — by hand, or by the CLI, which has no steppers — must
    /// be rounded and clamped on the way in. Asserted on `state`, not on
    /// `value()`: `BoundedField::value` clamps on read, so a field carrying 51
    /// reads as 30 while showing 51.
    #[test]
    fn a_plug_off_the_fields_grid_is_shown_rounded_and_clamped() {
        let plug = PlugDraft {
            // 51.2 mm, where the stepper stops at 30.
            cavity_inset_m: 0.0512,
            ridges: RidgeOptions {
                enabled: true,
                rings: vec![RidgeRing {
                    position_frac: 3.0,
                    depth_m: 0.001_52,
                    half_width_frac: 0.0,
                }],
                texture_depth_m: 0.000_16,
                // 50 mm of pitch, where the stepper stops at 30.
                texture_spacing_m: 0.05,
                orientation_deg: 400.0,
                ..RidgeOptions::default()
            },
        };

        let fields = ShapeControls::from_plug(&plug);

        assert_eq!(fields.cavity_mm.state.value(), 30, "the cavity is clamped");
        assert_eq!(
            fields.ridges.texture_spacing.state.value(),
            300,
            "and so is a scalar the ridge editor holds"
        );
        assert_eq!(
            fields.ridges.orientation.state.value(),
            360,
            "and one it holds in degrees rather than tenths of a mm"
        );
        let ring = fields.ridges.rings.first().expect("the ring is carried");
        assert_eq!(ring.position.state.value(), 100, "and so is the position");
        assert_eq!(ring.width.state.value(), 1, "a ring of no width is not one");
        assert_eq!(ring.depth.state.value(), 15, "15.2 tenths of a mm rounds");
        assert_eq!(
            fields.ridges.texture_depth.state.value(),
            2,
            "and 1.6 tenths rounds up"
        );
    }

    /// ★★ What resume is for. A project carrying a plug this session never
    /// shaped has to reach the fields — Continue commits what they say, so
    /// left on the defaults it would write 5 mm and no ridges over the user's
    /// real piece.
    #[test]
    fn the_fields_follow_a_plug_the_session_did_not_shape() {
        let shaped = a_shaped_screen();

        let after = reconcile(
            shaping(shaped_into(&shaped)),
            // ⚠ A fresh screen, exactly as launching the app hands one over.
            ShapeControls::default(),
        );

        assert_eq!(
            after.plug_draft(),
            shaped.plug_draft(),
            "the fields cut the committed piece"
        );
    }

    /// ⚠⚠ This runs every frame step 3 is up. Re-deriving unconditionally
    /// would throw away every number typed since the last commit, so a user
    /// could not change anything — and the screen would look like it was
    /// ignoring the mouse.
    #[test]
    fn a_redraw_does_not_clobber_what_the_user_is_typing() {
        let shaped = a_shaped_screen();
        let project = shaped_into(&shaped);
        let mut typing = ShapeControls::from_plug(&shaped.plug_draft());
        typing.cavity_mm.state = StepBoxState::new(9);

        let after = reconcile(shaping(project), typing);

        assert_eq!(after.cavity_mm.value(), 9, "the number being typed stands");
    }

    /// ★★ Continue is the moment the fields and the project agree, and it says
    /// so. Left for the reconcile to work out, the next frame would re-derive
    /// the fields from the artifact — which does not record the number behind
    /// an unticked toggle, so a screen left with the texture off at 1.1 mm
    /// would come back at the default 1.5.
    #[test]
    fn committing_a_plug_leaves_the_number_behind_an_unticked_toggle_alone() {
        let mut controls = a_shaped_screen();
        controls.ridges.texture_enabled = false;
        let mut studio = shaping(ready_to_shape());

        commit_plug(controls.plug_draft(), &mut controls, &mut studio);
        let after = reconcile(studio, controls);

        assert!(!after.ridges.texture_enabled, "still unticked");
        assert_eq!(
            after.ridges.texture_depth.value(),
            11,
            "and still showing the number the user left in the box"
        );
    }

    /// ⚠ The other side of the gate above, and it is a *leave alone*.
    /// `set_scan` clears the plug, but the fields are an editor, not a display
    /// of the project: wiping them because an upstream step was redone would
    /// discard numbers the user typed — which is what the screen did before
    /// this reconcile existed, and what it must go on doing.
    ///
    /// ⚠ The stamp still has to go, or the next frame re-runs this one.
    #[test]
    fn dropping_the_plug_leaves_the_fields_for_the_user() {
        let shaped = a_shaped_screen();
        let mut project = shaped_into(&shaped);
        let followed = reconcile(shaping(project.clone()), ShapeControls::default());
        assert_eq!(
            followed.cavity_mm.value(),
            12,
            "the fixture must start on the shaped piece"
        );

        project.set_scan(ScanInput {
            source_path: PathBuf::from("another.stl"),
        });
        let after = reconcile(shaping(project.clone()), followed);

        assert_eq!(
            after.plug_draft(),
            shaped.plug_draft(),
            "the numbers on screen are still the user's"
        );
        // Re-running it must be a no-op, or the stamp was not cleared.
        let mut typing = reconcile(shaping(project.clone()), after);
        typing.cavity_mm.state = StepBoxState::new(7);
        let settled = reconcile(shaping(project), typing);
        assert_eq!(settled.cavity_mm.value(), 7, "and still editable");
    }

    /// ⚠ The stamp goes with the plug it named. Left behind, a project
    /// carrying that very plug reads as "already followed" and the fields
    /// never move to it — Continue then commits whatever is on screen over the
    /// piece that was loaded. Reachable the moment resume exists: pick a scan,
    /// resume its project, pick the same scan again.
    #[test]
    fn a_dropped_plug_takes_its_stamp_with_it() {
        let shaped = a_shaped_screen();
        let mut project = shaped_into(&shaped);
        let followed = reconcile(shaping(project.clone()), ShapeControls::default());

        // The scan is re-picked, which clears the plug and leaves the fields.
        project.set_scan(ScanInput {
            source_path: PathBuf::from("another.stl"),
        });
        let mut edited = reconcile(shaping(project), followed);
        edited.cavity_mm.state = StepBoxState::new(7);

        // Then a project carrying that same piece arrives.
        let after = reconcile(shaping(shaped_into(&shaped)), edited);

        assert_eq!(
            after.cavity_mm.value(),
            12,
            "the fields follow the plug that landed, not the stamp of the one that left"
        );
    }

    /// The plugin's wiring: every gate above calls the system directly, so
    /// they say nothing about what runs it.
    #[test]
    fn the_plugin_runs_the_reconcile() {
        use bevy::state::app::StatesPlugin;

        let mut app = App::new();
        app.set_error_handler(bevy::ecs::error::ignore);
        app.add_plugins((MinimalPlugins, StatesPlugin, crate::plugin::StudioPlugin));
        app.insert_resource(shaping(shaped_into(&a_shaped_screen())));

        assert_eq!(
            app.world().resource::<ShapeControls>().cavity_mm.value(),
            CAVITY_DEFAULT_MM,
            "nothing is followed before the schedule runs"
        );
        app.world_mut().run_schedule(Update);

        assert_eq!(
            app.world().resource::<ShapeControls>().cavity_mm.value(),
            12,
            "the plugin's own schedule put the committed plug on the fields"
        );
    }
}
