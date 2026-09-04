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

use crate::scan::{ActiveScan, ScanEdit, ViewUpdate};
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

/// The Simplify target's floor and ceiling, and the value the field starts at
/// — the pre-port stepper's `minimum` / `maximum` / `value`, unchanged.
const SIMPLIFY_MIN_FACES: i32 = 1_000;
const SIMPLIFY_MAX_FACES: i32 = 1_000_000;
const DEFAULT_TARGET_FACES: i32 = 200_000;

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
    /// The face count a Simplify aims at.
    ///
    /// ⚠ Bounded by face count, not by the centerline, so [`Self::rebound`]
    /// must leave it alone — see [`simplify_range`].
    pub(crate) target_faces: StepBoxState,
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
            target_faces: StepBoxState::new(DEFAULT_TARGET_FACES),
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

    /// The target a Simplify runs at, in the units
    /// [`cf_studio_engine::run_simplify`] takes.
    ///
    /// ⚠ The clamp is not redundant with the stepper's own. Typing does not
    /// commit — [`StepBoxState::value`] tracks an in-progress edit unclamped, by
    /// design — so a number typed and then clicked straight through arrives
    /// here out of range. Clamping here is also what makes the conversion
    /// total, so that moving [`SIMPLIFY_MIN_FACES`] later cannot become a panic.
    #[allow(clippy::cast_sign_loss)] // The clamp removes the sign the lint is about.
    pub(crate) fn simplify_target(&self) -> usize {
        self.target_faces
            .value()
            .clamp(SIMPLIFY_MIN_FACES, SIMPLIFY_MAX_FACES) as usize
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

/// The Simplify stepper's `(min, max)`.
///
/// A free function rather than a method, and the asymmetry with
/// [`EditControls::trim_range`] is the point: a face target is not the mesh's
/// to re-bound, so there is no state for it to read.
pub(crate) const fn simplify_range() -> (i32, i32) {
    (SIMPLIFY_MIN_FACES, SIMPLIFY_MAX_FACES)
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
    land_edit(outcome, scan, studio, controls);
}

/// Report a finished step-2 op and re-bound the trim fields behind it.
///
/// Shared with the async Simplify's poller ([`crate::jobs::poll_simplify_job`])
/// rather than duplicated, because it is the pre-port `apply_edit` — the one
/// place that decided what happens *after* any step-2 op — and the two callers
/// drifting apart is how the trim fields would end up bounded by a centerline
/// that a Simplify had already cleared.
pub(crate) fn land_edit(
    outcome: StepOutcome,
    scan: &ScanEdit,
    studio: &mut Studio,
    controls: &mut EditControls,
) {
    // The over-trim guard. The op ran and the session holds its result; what is
    // held back is the *view*, because an empty mesh has nothing to draw and
    // dropping the last good one would leave the user staring at an empty
    // viewport with no way to see what they did.
    //
    // ⚠ This returns before re-bounding, as the pre-port code did. The trim IS
    // applied, so the centerline is now a stub — re-bounding to it would clamp
    // the number the user just typed down to that stub's length, taking away
    // the very field they are being told to reduce.
    //
    // ⚠ The wording names a trim because a trim is what reaches it.
    // `cf_scan_prep_core::simplify_mesh` returns the input unchanged when the
    // target is at or above the current face count, and decimates toward a
    // positive one otherwise, so a Simplify cannot arrive here having emptied
    // the mesh. The pre-port `apply_edit` carried this message on both paths.
    if scan.view() == ViewUpdate::Hold {
        studio.message = Some(Err(OVER_TRIM_MESSAGE.to_string()));
        return;
    }
    studio.message = Some(outcome);

    // ⚠ Only while a centerline exists to measure against. Every op that clears
    // one — weld, Simplify, reset — then reports an arc length of 0, and
    // `trim_bound_mm` floors that to a placeholder 10 mm. Re-bounding to the
    // placeholder would clamp the user's numbers against it: a 40 mm floor trim
    // would silently become 10 the moment they welded, and Find floor again
    // would not give it back. There is nothing to show meanwhile — the trim
    // section is hidden without a centerline — so holding the old bound costs
    // nothing and the next real trace replaces it.
    if let Some(session) = scan.active().map(ActiveScan::session)
        && session.has_centerline()
    {
        controls.rebound(trim_bound_mm(session.centerline_arc_length_mm()));
    }
}

/// Merge coincident vertices.
fn weld(session: &mut EditSession) -> StepOutcome {
    let (before, after) = session.weld();
    Ok(format!("✔ Welded vertices: {before} → {after}"))
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
        "✔ Trimmed — tip {tip_mm} mm, floor {floor_mm} mm — and re-leveled."
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
        "✔ Reconstructed the floor ({reference_mm} mm reference zone)."
    ))
}

/// Throw the edits away.
fn reset(session: &mut EditSession) -> StepOutcome {
    session.reset();
    Ok("↺ Reset to the original scan".to_string())
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::scan::ActiveScan;

    /// A minimal valid ASCII STL — one triangle, enough for `EditSession::load`.
    const ONE_TRIANGLE_STL: &str = "\
solid t
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid t
";

    /// A [`ScanEdit`] with that fixture loaded, plus the path to clean up.
    ///
    /// ★ Cheap on purpose. The executor was reachable only through a loaded
    /// scan, and a scan is just a file — so the "needs synthetic artifacts"
    /// excuse for leaving `apply_edit_intent` untested was worth about ten
    /// lines.
    fn loaded_scan(tag: &str) -> (ScanEdit, PathBuf) {
        let path = std::env::temp_dir().join(format!(
            "cf-studio-gui-edit-{tag}-{}.stl",
            std::process::id()
        ));
        assert!(
            std::fs::write(&path, ONE_TRIANGLE_STL).is_ok(),
            "the fixture must be writable"
        );
        let loaded = ActiveScan::load(&path);
        assert!(loaded.is_ok(), "the fixture must load: {:?}", loaded.err());
        let mut scan = ScanEdit::default();
        if let Ok(active) = loaded {
            scan.set(active);
        }
        (scan, path)
    }

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
    /// The executor end to end: the op runs, its message lands, and the
    /// viewport is asked to redraw *without* re-framing the camera.
    ///
    /// ⚠ Also the guard against an inverted over-trim check. Inverting `==` to
    /// `!=` reports "that trim removes the whole mesh" for every successful
    /// edit, which only an assertion on the success message catches.
    #[test]
    fn an_op_runs_reports_itself_and_asks_for_a_redraw() {
        let (mut scan, path) = loaded_scan("weld");
        let mut studio = Studio::default();
        let mut controls = EditControls::default();

        apply_edit_intent(EditIntent::Weld, &mut scan, &mut studio, &mut controls);
        let _ = std::fs::remove_file(&path);

        assert!(
            matches!(&studio.message, Some(Ok(text)) if text.contains("Welded")),
            "the op must run and report success: {:?}",
            studio.message
        );
        assert_eq!(
            scan.view(),
            ViewUpdate::Remesh,
            "an edit re-meshes; only a new scan moves the camera"
        );
    }

    /// Reconstruct needs a floor trim to rebuild. The screen hides the control
    /// until there is one, but the executor must refuse on its own — the
    /// button is not the guard.
    #[test]
    fn reconstructing_without_a_trim_is_refused() {
        let (mut scan, path) = loaded_scan("reconstruct");
        let mut studio = Studio::default();
        let mut controls = EditControls::default();

        apply_edit_intent(
            EditIntent::ReconstructFloor {
                shape: FloorShape::Flat,
                reference_mm: DEFAULT_REFERENCE_MM,
            },
            &mut scan,
            &mut studio,
            &mut controls,
        );
        let _ = std::fs::remove_file(&path);

        assert!(
            matches!(&studio.message, Some(Err(text)) if text.contains("Apply a floor trim first")),
            "with no trim there is nothing to rebuild: {:?}",
            studio.message
        );
    }

    /// ⚠ The face target is not a trim field. [`EditControls::rebound`] re-bounds
    /// three fields against the centerline's arc length; adding this one to that
    /// list would clamp a six-figure face count down to a few hundred
    /// millimetres the first time Find floor ran.
    #[test]
    fn re_bounding_the_trim_fields_leaves_the_face_target_alone() {
        let mut c = EditControls::default();
        c.target_faces.text_mut().clear();
        c.target_faces.text_mut().push_str("5000");
        c.target_faces.on_typed();
        assert!(c.target_faces.is_dirty(), "the fixture must start dirty");

        c.rebound(120);

        assert!(
            c.target_faces.is_dirty(),
            "a trim bound moving is not this field's business"
        );
        assert_eq!(
            c.simplify_target(),
            5_000,
            "and the typed target survives it"
        );
    }

    /// Typing does not commit, and [`StepBoxState`] tracks an in-progress edit
    /// unclamped by design — so a number typed and then clicked straight
    /// through is in range only because this conversion puts it there.
    #[test]
    fn a_typed_target_reaches_the_engine_inside_the_steppers_range() {
        let mut c = EditControls::default();

        c.target_faces.text_mut().clear();
        c.target_faces.text_mut().push_str("99999999");
        c.target_faces.on_typed();
        assert_eq!(
            c.simplify_target(),
            1_000_000,
            "clamped to SIMPLIFY_MAX_FACES"
        );

        c.target_faces.text_mut().clear();
        c.target_faces.text_mut().push_str("-5");
        c.target_faces.on_typed();
        assert_eq!(c.simplify_target(), 1_000, "and up to SIMPLIFY_MIN_FACES");
    }

    /// The pre-port stepper's bounds and starting value, which are what a
    /// user's habits with this field are built on.
    #[test]
    fn the_simplify_stepper_keeps_its_pre_port_bounds_and_default() {
        assert_eq!(simplify_range(), (1_000, 1_000_000));
        assert_eq!(EditControls::default().simplify_target(), 200_000);
    }

    /// [`open_tube`]'s height, in session units (metres) — 100 mm along the
    /// spine, so the bound it produces is readable at a glance.
    const TUBE_HEIGHT_M: f64 = 0.1;

    /// A welded square tube along +Z, open at both ends and [`TUBE_HEIGHT_M`]
    /// tall.
    ///
    /// Two open boundary loops with a spine between them is what `detect_caps`
    /// and `level_to_floor` trace a centerline from. The cube fixture is
    /// closed, so it can never have one — which is exactly what makes it the
    /// negative case below.
    fn open_tube() -> mesh_types::IndexedMesh {
        use mesh_types::Point3;

        const RINGS: usize = 4;
        const SIDE_M: f64 = 0.02;

        let mut vertices = Vec::new();
        for r in 0..RINGS {
            #[allow(clippy::cast_precision_loss)] // Four rings.
            let z = TUBE_HEIGHT_M * r as f64 / (RINGS - 1) as f64;
            vertices.push(Point3::new(0.0, 0.0, z));
            vertices.push(Point3::new(SIDE_M, 0.0, z));
            vertices.push(Point3::new(SIDE_M, SIDE_M, z));
            vertices.push(Point3::new(0.0, SIDE_M, z));
        }
        let mut faces = Vec::new();
        for r in 0..RINGS - 1 {
            let (b, t) = ((r * 4) as u32, ((r + 1) * 4) as u32);
            for k in 0..4u32 {
                let k2 = (k + 1) % 4;
                faces.push([b + k, b + k2, t + k2]);
                faces.push([b + k, t + k2, t + k]);
            }
        }
        mesh_types::IndexedMesh { vertices, faces }
    }

    /// The other half of the guard below: when there IS a centerline, the trim
    /// fields must be bounded to it.
    ///
    /// ⚠ Without this, deleting the re-bound from [`land_edit`] outright passes
    /// the whole suite — the guard's negative case would be the only half
    /// anyone had ever proved.
    ///
    /// ⚠ A window, not an equality. The trace is a polyline through slice
    /// centroids, so it stops half a slice short at each end and reads a little
    /// under the tube's full 100 mm — measured at 97. Pinning that exact number
    /// would pin the sampler's resolution, which is not this test's business.
    /// What must not happen is the 10 mm placeholder, and that is nowhere near.
    #[test]
    fn landing_an_op_that_traced_a_centerline_bounds_the_fields_to_it() {
        let mut scan = ScanEdit::default();
        scan.set(ActiveScan::synthetic(open_tube()));
        let mut studio = Studio::default();
        let mut controls = EditControls::default();
        let traced = scan.edit(find_floor);
        assert!(
            matches!(traced, Some(Ok(_))),
            "the fixture must stand up: {traced:?}"
        );

        land_edit(
            Ok("stood up".to_string()),
            &scan,
            &mut studio,
            &mut controls,
        );

        let bound = controls.trim_range().1;
        assert!(
            (90..=100).contains(&bound),
            "the bound must come from the tube's 100 mm spine, not the placeholder: {bound}"
        );
    }

    /// ★ The bug this guard exists for, driven through the landing itself.
    ///
    /// Every op that clears the centerline — weld, Simplify, reset — then
    /// reports an arc length of 0, which [`trim_bound_mm`] floors to a
    /// placeholder 10 mm. Re-bounding to that placeholder clamps numbers the
    /// user typed against a bound that describes nothing, and Find floor again
    /// does not give them back: [`StepBoxState::sync_external`] moves the
    /// value, not just the range. The sequence is the one the screen invites —
    /// Find floor, tidy, then "Find floor again".
    #[test]
    fn landing_an_op_that_cleared_the_centerline_holds_the_bound_it_no_longer_has() {
        let mut scan = ScanEdit::default();
        // A closed cube has no boundary loops, so the session has no centerline
        // — exactly the state a weld or a Simplify leaves behind.
        scan.set(ActiveScan::synthetic(mesh_types::unit_cube()));
        let mut studio = Studio::default();
        let mut controls = EditControls::default();
        // Stand in for the bound a Find floor had already established.
        controls.rebound(trim_bound_mm(147.6));
        controls
            .floor_mm
            .step(30, TRIM_MIN_MM, controls.trim_range().1);
        assert_eq!(
            controls.floor_mm.value(),
            40,
            "the fixture must start at 40"
        );

        land_edit(Ok("tidied".to_string()), &scan, &mut studio, &mut controls);

        assert_eq!(
            controls.trim_range().1,
            148,
            "the bound the last real trace left must stand"
        );
        assert_eq!(
            controls.floor_mm.value(),
            40,
            "and the trim the user typed against it"
        );
        assert_eq!(
            controls.reference_mm.value(),
            DEFAULT_REFERENCE_MM,
            "and so must the reference zone"
        );
    }

    /// The reference zone is not a trim: a trim may take off nothing, but a
    /// zone with no depth has no profile to average, so its floor is its own.
    #[test]
    fn the_reference_zone_keeps_its_own_floor_and_shares_the_bound() {
        let mut c = EditControls::default();

        assert_eq!(c.reference_range().0, REFERENCE_MIN_MM);
        assert_ne!(
            c.reference_range().0,
            c.trim_range().0,
            "a trim may be zero; a reference zone may not"
        );

        c.rebound(300);
        assert_eq!(
            c.reference_range(),
            (REFERENCE_MIN_MM, 300),
            "both fields are bounded by the same centerline"
        );
    }
}
