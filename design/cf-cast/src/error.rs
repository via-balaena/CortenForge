//! Error type for cf-cast operations.

use std::fmt;
use std::path::PathBuf;

use mesh_io::IoError;

/// Identifies which geometry or output mesh a [`CastError`] refers to.
///
/// Carries the layer index for per-layer targets so multi-layer
/// failures pinpoint the offending entry in `CastSpec::layers`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CastTarget {
    /// Input geometry: the `body` field of a [`crate::CastLayer`].
    /// `layer_index` matches the entry's position in `CastSpec::layers`
    /// (innermost-first; `0` is the innermost shell).
    LayerBody {
        /// Index into `CastSpec::layers`.
        layer_index: usize,
    },
    /// Output mesh: a per-layer mold cup. Indexed parallel to
    /// `CastSpec::layers`.
    Mold {
        /// Index into `CastSpec::layers`.
        layer_index: usize,
    },
    /// Input geometry: the shared `CastSpec::bounding_region`.
    BoundingRegion,
    /// Input geometry / output mesh: the shared `CastSpec::plug` used
    /// by the innermost cast.
    Plug,
}

impl fmt::Display for CastTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LayerBody { layer_index } => write!(f, "layer {layer_index} body"),
            Self::Mold { layer_index } => write!(f, "mold layer {layer_index}"),
            Self::BoundingRegion => write!(f, "bounding region"),
            Self::Plug => write!(f, "plug"),
        }
    }
}

/// Errors that can occur during a cf-cast pipeline run.
#[derive(thiserror::Error, Debug)]
pub enum CastError {
    /// [`CastSpec::layers`] was empty. At least one [`crate::CastLayer`]
    /// must be supplied for [`crate::CastSpec::export_molds`] to have
    /// work to do.
    ///
    /// [`CastSpec::layers`]: crate::CastSpec::layers
    #[error("CastSpec.layers must not be empty")]
    EmptyLayers,

    /// An input [`cf_design::Solid`] had unbounded extent (e.g., a bare
    /// half-space or a [`cf_design::Solid::user_fn`] without explicit
    /// bounds). The exporter cannot sample an unbounded SDF onto a
    /// finite scalar grid.
    #[error("infinite bounds on input Solid for {0}")]
    InfiniteBounds(CastTarget),

    /// Marching cubes produced an empty mesh for the target identified
    /// by the inner [`CastTarget`]. Usually means the sampled SDF
    /// never crosses zero inside the grid — i.e., the surface lies
    /// entirely outside the sampled region.
    #[error("marching cubes produced an empty mesh for {0}")]
    MeshingEmpty(CastTarget),

    /// The mesh failed the printability gate with one or more
    /// **blocking** Critical-severity issues — the cf-cast subset of
    /// `mesh-printability::IssueSeverity::Critical` that genuinely
    /// blocks printing (build-volume, watertight/manifold, thin
    /// walls, small features, trapped volumes). Tolerated Criticals
    /// (overhangs, bridges, MC self-intersection noise) do not
    /// trigger this variant; they surface via
    /// [`MoldExportReport`]'s validation fields for caller inspection.
    ///
    /// [`MoldExportReport`]: crate::MoldExportReport
    #[error("mesh failed printability gate for {target} ({issue_count} blocking issue(s), {path})")]
    PrintabilityCritical {
        /// Which output failed.
        target: CastTarget,
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
