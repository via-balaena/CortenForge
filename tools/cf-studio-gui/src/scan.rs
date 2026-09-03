//! The loaded scan: the engine's edit session, and the render lift for it.

use std::path::Path;

use bevy::prelude::Resource;
use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::scale::RenderScale;
use cf_studio_engine::EditSession;
use mesh_types::Bounded;

/// Scans carry no unit metadata; the workshop scanner exports millimeters and
/// the cast pipeline works in meters. (A units selector can override this
/// later; mm is `base_mold`'s default and cf-scan-prep's.)
const SCAN_SCALE_TO_M: f64 = 0.001;

/// `+Z` is up — the cast frame's demolding direction, shared with cf-scan-prep.
pub(crate) const SCAN_UP_AXIS: UpAxis = UpAxis::PlusZ;

/// A loaded scan: the editing session, and the render lift chosen for it.
///
/// The lift is fixed at load and never re-derived. Step 2's ops change the mesh,
/// and a per-edit lift would resize the model on screen mid-op.
pub(crate) struct ActiveScan {
    /// The engine session every step-2 op runs against.
    pub(crate) session: EditSession,
    /// The lift from the scan's own size to the framed 1 m.
    pub(crate) scale: RenderScale,
}

impl ActiveScan {
    /// Load the scan at `path`.
    ///
    /// # Errors
    /// The engine's message if the file is missing, unreadable, or empty.
    #[allow(clippy::cast_possible_truncation)] // f64 diagonal → the f32 the lift takes.
    pub(crate) fn load(path: &Path) -> Result<Self, String> {
        let session = EditSession::load(path, SCAN_SCALE_TO_M).map_err(|e| e.to_string())?;
        let diagonal = session.display_mesh().aabb().diagonal() as f32;
        Ok(Self {
            session,
            scale: RenderScale::for_diagonal(diagonal),
        })
    }
}

/// The scan being worked on, once one has been chosen.
#[derive(Resource, Default)]
pub(crate) struct ScanEdit(Option<ActiveScan>);

impl ScanEdit {
    /// Make `scan` the active one, replacing any previous.
    ///
    /// ⚠ There is deliberately no `&mut` accessor. Measured: any `&mut` reach
    /// into this resource marks it changed even with nothing written, which
    /// redraws the viewport — so a failed load, which builds no [`ActiveScan`],
    /// cannot reach the slot and cannot disturb the scan already on screen.
    pub(crate) fn set(&mut self, scan: ActiveScan) {
        self.0 = Some(scan);
    }

    /// The active scan, or `None` before one is chosen.
    pub(crate) const fn active(&self) -> Option<&ActiveScan> {
        self.0.as_ref()
    }
}
