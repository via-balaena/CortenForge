//! Step 2's cleanup ops, and the numeric fields the screen drives them from.
//!
//! These are separated from [`crate::panel::Intent`] on purpose, and the split
//! is load-bearing rather than tidy: executing one of these borrows
//! [`ScanEdit`] mutably, which marks it changed and costs a full rebuild of a
//! 200 000-face mesh. Back and Next must not pay that. See the warning on
//! [`ScanEdit`].

use bevy::prelude::Resource;
use cf_studio_engine::{EditSession, ReconstructShape};
use cf_studio_gui::{
    FloorReadiness, StepBoxState, StepOutcome, format_floor_found, format_floor_no_centerline,
    trim_bound_mm,
};

use crate::scan::{ScanEdit, ViewUpdate};
use crate::state::Studio;

/// Trims start at nothing taken off.
const TRIM_MIN_MM: i32 = 0;
/// The reference zone needs some depth of scan to average a profile over.
const REFERENCE_MIN_MM: i32 = 5;
/// Defaults carried over from the pre-port screen: no tip trim, 10 mm off the
/// floor (the hint's "good start"), and a 25 mm reference zone.
const DEFAULT_TIP_MM: i32 = 0;
const DEFAULT_FLOOR_MM: i32 = 10;
const DEFAULT_REFERENCE_MM: i32 = 25;

/// Reported when a trim leaves nothing behind. The mesh is still trimmed — only
/// the view is held — so the wording asks for a smaller number rather than
/// claiming the op was refused.
const OVER_TRIM_MESSAGE: &str = "That trim removes the whole mesh — reduce it.";

/// How the rebuilt floor is shaped, as the picker offers it.
///
/// ⚠ "Flat" is the engine's [`ReconstructShape::Constant`]. The label describes
/// the result to someone holding the part; the engine name describes the radial
/// profile it extrudes. Do not rename either to match the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum FloorShape {
    #[default]
    Flat,
    Taper,
    Extrapolate,
}

impl FloorShape {
    /// The three choices, in picker order.
    pub(crate) const ALL: [Self; 3] = [Self::Flat, Self::Taper, Self::Extrapolate];

    /// The picker's label for this shape.
    pub(crate) const fn label(self) -> &'static str {
        match self {
            Self::Flat => "Flat",
            Self::Taper => "Taper",
            Self::Extrapolate => "Extrapolate",
        }
    }

    /// The engine's name for it.
    const fn to_engine(self) -> ReconstructShape {
        match self {
            Self::Flat => ReconstructShape::Constant,
            Self::Taper => ReconstructShape::Taper,
            Self::Extrapolate => ReconstructShape::Extrapolate,
        }
    }
}

/// The step-2 screen's field state, which outlives any one frame.
#[derive(Resource)]
pub(crate) struct EditControls {
    /// Millimetres to take off the closed tip.
    pub(crate) tip_mm: StepBoxState,
    /// Millimetres to take off the open floor.
    pub(crate) floor_mm: StepBoxState,
    /// How far above the cut to read the profile the new floor is built from.
    pub(crate) reference_mm: StepBoxState,
    /// The rebuilt floor's shape.
    pub(crate) shape: FloorShape,
    /// The upper bound all three fields clamp against — the centerline's arc
    /// length, since that is what a trim is measured along.
    bound_mm: i32,
}

impl Default for EditControls {
    fn default() -> Self {
        Self {
            tip_mm: StepBoxState::new(DEFAULT_TIP_MM),
            floor_mm: StepBoxState::new(DEFAULT_FLOOR_MM),
            reference_mm: StepBoxState::new(DEFAULT_REFERENCE_MM),
            shape: FloorShape::default(),
            // No centerline yet, so the floor of the bound is all there is.
            bound_mm: trim_bound_mm(0.0),
        }
    }
}

impl EditControls {
    /// The trim fields' `(min, max)`.
    pub(crate) const fn trim_range(&self) -> (i32, i32) {
        (TRIM_MIN_MM, self.bound_mm)
    }

    /// The reference-zone field's `(min, max)`.
    pub(crate) const fn reference_range(&self) -> (i32, i32) {
        (REFERENCE_MIN_MM, self.bound_mm)
    }

    /// Re-bound the fields when the centerline's arc length has moved.
    ///
    /// ⚠ Only when it actually moved. [`StepBoxState::sync_external`] discards a
    /// pending edit by design — the model wins when the bounds move — so running
    /// it after every op would wipe a half-typed number every time the user
    /// clicked a different button.
    fn rebound(&mut self, bound_mm: i32) {
        if bound_mm == self.bound_mm {
            return;
        }
        self.bound_mm = bound_mm;
        self.tip_mm
            .sync_external(self.tip_mm.value(), TRIM_MIN_MM, bound_mm);
        self.floor_mm
            .sync_external(self.floor_mm.value(), TRIM_MIN_MM, bound_mm);
        self.reference_mm
            .sync_external(self.reference_mm.value(), REFERENCE_MIN_MM, bound_mm);
    }
}

/// A step-2 cleanup op. Each carries the field values it was clicked with, so
/// the executor cannot read a field the user has since changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EditIntent {
    /// Merge coincident vertices, so the rest of the ops see real adjacency.
    Weld,
    /// Detect the open end and stand the scan upright on it.
    FindFloor,
    /// Cut the given depths off each end, along the centerline.
    ApplyTrim { tip_mm: i32, floor_mm: i32 },
    /// Close the trimmed floor back up.
    ReconstructFloor {
        shape: FloorShape,
        reference_mm: i32,
    },
    /// Throw the edits away and start from the scan as loaded.
    Reset,
}

/// Run a cleanup op and report what happened. The one place step 2 mutates the
/// scan.
pub(crate) fn apply_edit_intent(
    intent: EditIntent,
    scan: &mut ScanEdit,
    studio: &mut Studio,
    controls: &mut EditControls,
) {
    let outcome = match intent {
        EditIntent::Weld => scan.edit(weld),
        EditIntent::FindFloor => scan.edit(find_floor),
        EditIntent::ApplyTrim { tip_mm, floor_mm } => {
            scan.edit(|s| apply_trim(s, tip_mm, floor_mm))
        }
        EditIntent::ReconstructFloor {
            shape,
            reference_mm,
        } => scan.edit(|s| reconstruct_floor(s, shape, reference_mm)),
        EditIntent::Reset => scan.edit(reset),
    };
    // `None` means no scan is loaded, which the step-2 screen cannot be showing.
    let Some(outcome) = outcome else { return };

    // The over-trim guard. The op ran and the session holds its result; what is
    // held back is the *view*, because an empty mesh has nothing to draw and
    // dropping the last good one would leave the user staring at an empty
    // viewport with no way to see what they did.
    //
    // ⚠ This returns before re-bounding, as the pre-port code did. The trim IS
    // applied, so the centerline is now a stub — re-bounding to it would clamp
    // the number the user just typed down to that stub's length, taking away
    // the very field they are being told to reduce.
    if scan.view() == ViewUpdate::Hold {
        studio.message = Some(Err(OVER_TRIM_MESSAGE.to_string()));
        return;
    }
    studio.message = Some(outcome);

    if let Some(active) = scan.active() {
        controls.rebound(trim_bound_mm(active.session().centerline_arc_length_mm()));
    }
}

/// Merge coincident vertices.
fn weld(session: &mut EditSession) -> StepOutcome {
    let (before, after) = session.weld();
    Ok(format!("✓ Welded vertices: {before} → {after}"))
}

/// Find the open end and stand the scan up on it.
fn find_floor(session: &mut EditSession) -> StepOutcome {
    let scan = session.detect_caps();
    // Read the scan before levelling: levelling has a side effect, and the two
    // blocked cases must not pay for it.
    if let Some(blocked) =
        FloorReadiness::read(scan.loop_count, scan.looks_unwelded).blocked_message()
    {
        return Err(blocked.to_string());
    }
    session.level_to_floor().map_or_else(
        || Err(format_floor_no_centerline(scan.loop_count)),
        |tilt_deg| {
            Ok(format_floor_found(
                scan.loop_count,
                scan.centerline_segments,
                tilt_deg,
            ))
        },
    )
}

/// Trim both ends, then stand the result back up.
fn apply_trim(session: &mut EditSession, tip_mm: i32, floor_mm: i32) -> StepOutcome {
    session.apply_trim(f64::from(tip_mm), f64::from(floor_mm));
    // Re-level onto the new cut floor — the predicted/reconstructed floor path,
    // now that there is a trim.
    session.level_to_floor();
    Ok(format!(
        "✓ Trimmed — tip {tip_mm} mm, floor {floor_mm} mm — and re-leveled."
    ))
}

/// Rebuild the floor the trim cut away.
fn reconstruct_floor(
    session: &mut EditSession,
    shape: FloorShape,
    reference_mm: i32,
) -> StepOutcome {
    if !session.apply_reconstruct(f64::from(reference_mm), shape.to_engine()) {
        return Err("Apply a floor trim first, then reconstruct.".to_string());
    }
    session.level_to_floor();
    Ok(format!(
        "✓ Reconstructed the floor ({reference_mm} mm reference zone)."
    ))
}

/// Throw the edits away.
fn reset(session: &mut EditSession) -> StepOutcome {
    session.reset();
    Ok("↺ Reset to the original scan".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A field with an uncommitted edit in it: "10" typed up to "107".
    fn with_a_pending_floor_edit() -> EditControls {
        let mut c = EditControls::default();
        c.floor_mm.text_mut().push('7');
        c.floor_mm.on_typed();
        assert!(c.floor_mm.is_dirty(), "the fixture must start dirty");
        c
    }

    /// The guard `rebound` exists for. Every op re-reads the arc length, so
    /// without the equality check a weld would wipe a number the user was
    /// halfway through typing into the trim field beside it.
    #[test]
    fn rebounding_to_the_same_bound_leaves_a_pending_edit_alone() {
        let mut c = with_a_pending_floor_edit();
        let unchanged = c.trim_range().1;

        c.rebound(unchanged);

        assert!(c.floor_mm.is_dirty(), "an unchanged bound discards nothing");
        assert_eq!(c.floor_mm.text(), "107", "and leaves the text as typed");
    }

    /// The case `sync_external` was written for: the bound really moved, so the
    /// model wins and the value is pulled inside the new range.
    #[test]
    fn a_shrunken_bound_discards_the_pending_edit_and_clamps() {
        let mut c = EditControls::default();
        c.rebound(500);
        c.floor_mm.text_mut().clear();
        c.floor_mm.text_mut().push_str("400");
        c.floor_mm.on_typed();

        c.rebound(120);

        assert!(!c.floor_mm.is_dirty(), "the model wins when bounds move");
        assert_eq!(c.floor_mm.value(), 120, "clamped into the new range");
        assert_eq!(c.trim_range(), (TRIM_MIN_MM, 120), "and the range moved");
    }

    /// ⚠ The one mapping that is not its own name. A swap here would silently
    /// rebuild every floor with the wrong profile — the geometry would still
    /// come out, just wrong.
    #[test]
    fn flat_is_the_engines_constant_profile() {
        assert_eq!(FloorShape::Flat.to_engine(), ReconstructShape::Constant);
        assert_eq!(FloorShape::Taper.to_engine(), ReconstructShape::Taper);
        assert_eq!(
            FloorShape::Extrapolate.to_engine(),
            ReconstructShape::Extrapolate
        );
    }

    /// The pre-port picker was an index into `["Flat", "Taper", "Extrapolate"]`,
    /// so this order is the one users learned.
    #[test]
    fn the_picker_lists_the_shapes_in_the_pre_port_order() {
        let labels: Vec<&str> = FloorShape::ALL.iter().map(|s| s.label()).collect();
        assert_eq!(labels, ["Flat", "Taper", "Extrapolate"]);
    }
}
