//! Error type for cf-cast operations.

use std::path::PathBuf;

use mesh_io::IoError;

/// Errors that can occur during a cf-cast pipeline run.
#[derive(thiserror::Error, Debug)]
pub enum CastError {
    /// An input [`cf_design::Solid`] had unbounded extent (e.g., a bare
    /// half-space or a [`cf_design::Solid::user_fn`] without explicit
    /// bounds). The exporter cannot sample an unbounded SDF onto a
    /// finite scalar grid.
    ///
    /// The associated string identifies which input failed (e.g.,
    /// `"layer_body"`, `"bounding_region"`, `"plug"`).
    #[error("infinite bounds on input Solid: {0}")]
    InfiniteBounds(&'static str),

    /// Marching cubes produced an empty mesh for the named output.
    /// Usually means the sampled SDF never crosses zero inside the
    /// grid — i.e., the surface lies entirely outside the sampled
    /// region.
    #[error("marching cubes produced an empty mesh for {0}")]
    MeshingEmpty(&'static str),

    /// The mesh failed the printability gate with one or more
    /// **blocking** Critical-severity issues — the cf-cast subset of
    /// `mesh-printability::IssueSeverity::Critical` that genuinely
    /// blocks printing (build-volume, watertight/manifold, thin
    /// walls, small features, trapped volumes). Tolerated Criticals
    /// (overhangs, bridges, MC self-intersection noise) do not
    /// trigger this variant; they surface via
    /// [`MoldExportReport`]'s validation field for caller inspection.
    ///
    /// [`MoldExportReport`]: crate::MoldExportReport
    #[error("mesh failed printability gate for {target} ({issue_count} blocking issue(s), {path})")]
    PrintabilityCritical {
        /// Which output failed: `"mold"` or `"plug"`.
        target: &'static str,
        /// Number of blocking Critical-severity issues found.
        issue_count: usize,
        /// Path the exporter was about to write when the gate fired.
        path: PathBuf,
    },

    /// STL write failed at the filesystem layer. Path is preserved
    /// across the boundary since [`IoError::Io`] does not carry it.
    #[error("STL write failed for {path}: {source}")]
    MeshIo {
        /// Path that was being written.
        path: PathBuf,
        /// Underlying [`IoError`] from `mesh-io`.
        #[source]
        source: IoError,
    },
}
