//! The loaded scan: the engine's edit session, the render lift, and the view
//! the two of them feed.

use std::path::Path;

use bevy::prelude::{Resource, Vec3};
use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::scale::RenderScale;
use cf_studio_engine::EditSession;
use mesh_types::{Bounded, IndexedMesh};

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
            centerline: lifted_centerline(&session, scale),
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
        self.centerline = lifted_centerline(&self.session, self.scale);
        self.display = display;
        true
    }
}

/// The engine's display centerline, in the same frame and at the same lift as
/// the rendered body.
///
/// ⚠ The lift is applied here by hand. Gizmos draw in world space — they are
/// not children of the scan entity, so [`RenderScale::transform`] never reaches
/// them, and an unlifted line would hide inside a body drawn several times
/// larger than it.
fn lifted_centerline(session: &EditSession, scale: RenderScale) -> Vec<Vec3> {
    session
        .display_centerline()
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
