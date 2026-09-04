//! The loaded scan: the engine's edit session, the render lift, and the view
//! the two of them feed.

use std::path::Path;

use bevy::prelude::{Resource, Vec3};
use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::scale::RenderScale;
use cf_studio_engine::EditSession;
use mesh_types::{Bounded, IndexedMesh, Point3};

/// Scans carry no unit metadata; the workshop scanner exports millimeters and
/// the cast pipeline works in meters. (A units selector can override this
/// later; mm is `base_mold`'s default and cf-scan-prep's.)
const SCAN_SCALE_TO_M: f64 = 0.001;

/// `+Z` is up — the cast frame's demolding direction, shared with cf-scan-prep.
pub(crate) const SCAN_UP_AXIS: UpAxis = UpAxis::PlusZ;

/// What the latest change to the scan asks the viewport to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ViewUpdate {
    /// A newly loaded scan: rebuild the body and frame the camera on it.
    Reframe,
    /// An edit to the scan already on screen: rebuild the body, and leave the
    /// camera where the user put it. Re-framing here would snap the view back
    /// to the front on every weld and every trim.
    Remesh,
    /// Nothing renderable came of it — keep the view exactly as it is.
    Hold,
}

/// A loaded scan: the editing session, the render lift, and the cached view.
///
/// The lift is fixed at load and never re-derived. Step 2's ops change the mesh,
/// and a per-edit lift would resize the model on screen mid-op.
pub(crate) struct ActiveScan {
    /// The engine session every step-2 op runs against.
    session: EditSession,
    /// The lift from the scan's own size to the framed 1 m.
    scale: RenderScale,
    /// What the viewport draws, as of the last change.
    ///
    /// Cached rather than recomputed on demand: once a trim exists,
    /// [`EditSession::display_mesh`] runs a centerline trim, a weld and a cap
    /// (or reconstruction) pass over the whole scan. Both the viewport and the
    /// over-trim guard need it, and the guard has to see it *before* the
    /// viewport does.
    display: IndexedMesh,
    /// The centerline overlay, in Bevy space and already lifted.
    ///
    /// Cached for a sharper reason than the mesh: [`EditSession::display_centerline`]
    /// bakes each point about the working mesh's centroid, and that centroid is
    /// an `aabb()` call — a full scan of every vertex. Drawing straight from it
    /// would sweep 600 000 vertices per frame to place a few dozen points.
    centerline: Vec<Vec3>,
}

impl ActiveScan {
    /// Load the scan at `path`.
    ///
    /// # Errors
    /// The engine's message if the file is missing, unreadable, or empty.
    #[allow(clippy::cast_possible_truncation)] // f64 diagonal → the f32 the lift takes.
    pub(crate) fn load(path: &Path) -> Result<Self, String> {
        let session = EditSession::load(path, SCAN_SCALE_TO_M).map_err(|e| e.to_string())?;
        let diagonal = session.working().aabb().diagonal() as f32;
        let scale = RenderScale::for_diagonal(diagonal);
        Ok(Self {
            display: session.display_mesh(),
            centerline: lifted_centerline(&session.display_centerline(), scale),
            session,
            scale,
        })
    }

    /// The mesh the viewport draws.
    pub(crate) const fn display(&self) -> &IndexedMesh {
        &self.display
    }

    /// The centerline overlay, in Bevy space at the rendered lift. Empty until
    /// Find floor traces one.
    pub(crate) fn centerline(&self) -> &[Vec3] {
        &self.centerline
    }

    /// The render lift, for the entity transform and the camera framing.
    pub(crate) const fn scale(&self) -> RenderScale {
        self.scale
    }

    /// The session, for the stats and the gating the step-2 screen reads off it.
    pub(crate) const fn session(&self) -> &EditSession {
        &self.session
    }

    /// Recompute the cached view after an edit. `false` means the edit left
    /// nothing renderable, in which case the cache is left untouched so the
    /// last good view survives.
    fn refresh_view(&mut self) -> bool {
        let display = self.session.display_mesh();
        if display.faces.is_empty() {
            return false;
        }
        self.centerline = lifted_centerline(&self.session.display_centerline(), self.scale);
        self.display = display;
        true
    }
}

/// Centerline points in the same frame and at the same lift as the rendered
/// body.
///
/// Takes the points rather than the session because it does no session work —
/// which also makes the one piece of arithmetic here testable against the
/// entity transform it has to agree with.
///
/// ⚠ The lift is applied here by hand. Gizmos draw in world space — they are
/// not children of the scan entity, so [`RenderScale::transform`] never reaches
/// them, and an unlifted line would hide inside a body drawn several times
/// larger than it.
fn lifted_centerline(points: &[Point3<f64>], scale: RenderScale) -> Vec<Vec3> {
    points
        .iter()
        .map(|p| Vec3::from_array(SCAN_UP_AXIS.to_bevy_point(p)) * scale.0)
        .collect()
}

/// The scan being worked on, once one has been chosen.
///
/// ⚠ The only ways in are [`Self::set`] and [`Self::edit`], and each leaves a
/// [`ViewUpdate`] behind saying what the viewport should do about it. There is
/// deliberately no bare `&mut` accessor, for two measured reasons:
///
/// - any `&mut` reach into a Bevy resource marks it changed even with nothing
///   written, and this resource's change flag drives a full rebuild of a
///   200 000-face mesh;
/// - a failed load builds no [`ActiveScan`], so it cannot reach the slot and
///   cannot disturb the scan already on screen.
///
/// ⇒ **a navigation click must never take a path that borrows this mutably.**
/// That is why the panel emits step-2 edits as a separate intent type from
/// Back / Next: routing both through one `apply_intent` would charge a Back
/// click a full re-mesh and a camera jump.
#[derive(Resource)]
pub(crate) struct ScanEdit {
    scan: Option<ActiveScan>,
    view: ViewUpdate,
}

impl Default for ScanEdit {
    /// Nothing loaded, nothing for the viewport to do.
    fn default() -> Self {
        Self {
            scan: None,
            view: ViewUpdate::Hold,
        }
    }
}

impl ScanEdit {
    /// Make `scan` the active one, replacing any previous, and frame the camera
    /// on it.
    pub(crate) fn set(&mut self, scan: ActiveScan) {
        self.scan = Some(scan);
        self.view = ViewUpdate::Reframe;
    }

    /// The active scan, or `None` before one is chosen.
    pub(crate) const fn active(&self) -> Option<&ActiveScan> {
        self.scan.as_ref()
    }

    /// What the viewport should do about the latest change.
    pub(crate) const fn view(&self) -> ViewUpdate {
        self.view
    }

    /// Run a step-2 op against the session, then refresh the cached view from
    /// the result. Returns the op's own value, or `None` with no scan loaded.
    ///
    /// Check [`Self::view`] afterwards: [`ViewUpdate::Hold`] means the op left
    /// an unrenderable mesh, which the caller reports instead of the op's own
    /// message.
    pub(crate) fn edit<T>(&mut self, op: impl FnOnce(&mut EditSession) -> T) -> Option<T> {
        let scan = self.scan.as_mut()?;
        let value = op(&mut scan.session);
        self.view = if scan.refresh_view() {
            ViewUpdate::Remesh
        } else {
            ViewUpdate::Hold
        };
        Some(value)
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use mesh_types::unit_cube;

    use super::*;

    /// An [`ActiveScan`] over `mesh`, built the way [`ActiveScan::load`] builds
    /// one but without going through a file.
    ///
    /// The lift is pinned to 1.0: these tests are about *what the viewport is
    /// asked to do*, and a scale would only add arithmetic to the assertions.
    fn scan_over(mesh: IndexedMesh) -> ActiveScan {
        let session = EditSession::from_mesh(PathBuf::from("synthetic.stl"), mesh);
        ActiveScan {
            display: session.display_mesh(),
            centerline: lifted_centerline(&session.display_centerline(), RenderScale(1.0)),
            session,
            scale: RenderScale(1.0),
        }
    }

    /// ★★★ The invariant the overlay exists under: a gizmo point must land
    /// exactly where the scan entity puts the same point. Gizmos draw in world
    /// space and are not children of that entity, so the lift is applied by
    /// hand here — and a hand-applied lift is one typo away from an overlay
    /// that floats inside, or far outside, the body it describes.
    ///
    /// ⚠ The expectation is built from the *entity's* own pieces — the mesh
    /// conversion's axis swap and [`RenderScale::transform`] — not by
    /// restating this function's arithmetic, which would agree with itself no
    /// matter how wrong both were.
    #[test]
    fn a_lifted_point_lands_where_the_scan_entity_puts_the_same_point() {
        // 6.68x is `base_mold`'s measured lift; the offsets are asymmetric so a
        // swapped axis cannot pass by coincidence.
        let scale = RenderScale(6.68);
        let points = [
            Point3::new(0.011, 0.022, 0.033),
            Point3::new(0.0, 0.0, 0.05),
        ];

        let lifted = lifted_centerline(&points, scale);

        assert_eq!(lifted.len(), points.len(), "every point survives the lift");
        for (got, p) in lifted.iter().zip(&points) {
            let on_entity = scale
                .transform()
                .transform_point(Vec3::from_array(SCAN_UP_AXIS.to_bevy_point(p)));
            assert!(
                (*got - on_entity).length() < 1e-6,
                "overlay {got:?} does not sit on the body at {on_entity:?}"
            );
        }
    }

    /// A unit lift must be a true no-op, or a metre-scale scan's overlay drifts.
    #[test]
    fn a_unit_lift_only_swaps_axes() {
        let lifted = lifted_centerline(&[Point3::new(1.0, 2.0, 3.0)], RenderScale(1.0));
        assert_eq!(
            lifted,
            vec![Vec3::new(1.0, 3.0, 2.0)],
            "+Z up swaps y and z"
        );
    }

    #[test]
    fn a_newly_set_scan_asks_for_the_camera() {
        let mut edit = ScanEdit::default();
        edit.set(scan_over(unit_cube()));
        assert_eq!(edit.view(), ViewUpdate::Reframe);
    }

    /// An edit re-meshes but must NOT re-frame: the pre-port viewer preserved
    /// the orbit angle, and re-framing here snaps the view back to the front on
    /// every weld and trim.
    #[test]
    fn an_ordinary_edit_remeshes_without_reaching_for_the_camera() {
        let mut edit = ScanEdit::default();
        edit.set(scan_over(unit_cube()));

        let welded = edit.edit(EditSession::weld);

        assert!(welded.is_some(), "the op runs against the loaded scan");
        assert_eq!(
            edit.view(),
            ViewUpdate::Remesh,
            "an edit must never re-frame the camera"
        );
    }

    /// The over-trim guard, and the reason it exists: an edit that leaves
    /// nothing renderable must keep the last good mesh on screen, or the user
    /// is left staring at an empty viewport with no way to see what they did.
    ///
    /// ⚠ Both halves are asserted. A guard that returned [`ViewUpdate::Hold`]
    /// but still overwrote the cache would blank the view anyway, and the
    /// `view()` check alone cannot see that.
    #[test]
    fn an_edit_that_empties_the_mesh_holds_the_last_good_view() {
        let mut edit = ScanEdit::default();
        edit.set(scan_over(unit_cube()));
        let good_faces = edit.active().map(|a| a.display().faces.len());
        assert_eq!(good_faces, Some(12), "the fixture must start renderable");

        // Stand in for an over-trim: the session now yields an empty display.
        let ran = edit.edit(|session| {
            *session =
                EditSession::from_mesh(PathBuf::from("synthetic.stl"), IndexedMesh::default());
        });

        assert!(ran.is_some(), "the op still ran");
        assert_eq!(
            edit.view(),
            ViewUpdate::Hold,
            "nothing renderable came of it"
        );
        assert_eq!(
            edit.active().map(|a| a.display().faces.len()),
            good_faces,
            "the last good mesh must survive the guard"
        );
    }

    #[test]
    fn editing_before_a_scan_is_loaded_reports_nothing() {
        let mut edit = ScanEdit::default();
        assert!(edit.edit(EditSession::weld).is_none());
        assert_eq!(
            edit.view(),
            ViewUpdate::Hold,
            "and asks the viewport for nothing"
        );
    }
}
