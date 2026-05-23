//! Error type for cf-cast operations.

use std::fmt;
use std::path::PathBuf;

use mesh_io::IoError;

use crate::ribbon::PieceSide;

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
    /// `CastSpec::layers`. Used by v1's single-piece
    /// [`crate::CastSpec::export_molds`] pipeline.
    Mold {
        /// Index into `CastSpec::layers`.
        layer_index: usize,
    },
    /// Output mesh: one of the two v2 curve-following mold pieces for
    /// a given layer. Used by v2's
    /// [`crate::CastSpec::export_molds_v2`] pipeline.
    MoldPiece {
        /// Index into `CastSpec::layers`.
        layer_index: usize,
        /// Which side of the ribbon split surface this piece occupies.
        piece_side: PieceSide,
    },
    /// Input geometry: the shared `CastSpec::bounding_region`.
    BoundingRegion,
    /// Input geometry / output mesh: a per-layer plug. v1 + v2-pre-2.1
    /// shared one plug across the cast; v2.1 sub-leaf 2 ships one
    /// plug per layer for detachable-shell assembly. `layer_index` is
    /// `None` for v1's single-plug pipeline and `Some(N)` for v2's
    /// per-layer plug derived from `CastSpec::plug` (`N = 0`) or
    /// `layers[N - 1].body` (`N > 0`).
    Plug {
        /// Layer index for v2's per-layer plug; `None` for v1's
        /// single shared plug.
        layer_index: Option<usize>,
    },
    /// Output mesh: the workshop platform STL. Generated only when
    /// the ribbon's plug-pin kind has `include_t_bar = true` (so
    /// the T-bar protrusion needs a pocketed platform for the
    /// assembled mold to sit flat during pour + cure). Single
    /// artifact per cast (shared across layers).
    Platform,
    /// Output mesh: the workshop pour funnel STL.
    ///
    /// Generated only when the ribbon has a
    /// [`crate::pour::PourGateKind::Default`] pour gate enabled.
    /// Self-aligning nipple + flange + cone for honey-thick
    /// silicone pouring. Single artifact per cast (shared across
    /// layers).
    Funnel,
}

impl fmt::Display for CastTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LayerBody { layer_index } => write!(f, "layer {layer_index} body"),
            Self::Mold { layer_index } => write!(f, "mold layer {layer_index}"),
            Self::MoldPiece {
                layer_index,
                piece_side,
            } => write!(f, "mold layer {layer_index} piece {piece_side:?}"),
            Self::BoundingRegion => write!(f, "bounding region"),
            Self::Plug { layer_index: None } => write!(f, "plug"),
            Self::Plug {
                layer_index: Some(n),
            } => write!(f, "plug layer {n}"),
            Self::Platform => write!(f, "platform"),
            Self::Funnel => write!(f, "funnel"),
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

    /// The v2 centerline polyline curves more sharply than v2's 2-piece
    /// mold can demold. Per
    /// `docs/CURVE_FOLLOWING_DESIGN.md` §"Piece count selection":
    /// max tangent rotation `> 120°` refuses with this error;
    /// the user must split the scan upstream (3+-piece molds are v3
    /// scope).
    ///
    /// Fires at the top of
    /// [`crate::CastSpec::export_molds_v2`] before any meshing —
    /// pre-write atomicity is preserved.
    #[error(
        "centerline max tangent rotation {max_rotation_rad:.4} rad ({max_rotation_deg:.1}°) \
         exceeds v2 2-piece threshold {threshold_rad:.4} rad ({threshold_deg:.1}°); \
         split the scan upstream or wait for v3 multi-piece molds"
    )]
    CenterlineTooCurved {
        /// Maximum tangent rotation observed in the centerline
        /// (radians; sourced from [`crate::Ribbon::max_tangent_rotation_rad`]).
        max_rotation_rad: f64,
        /// Same value in degrees for human-readable error display.
        max_rotation_deg: f64,
        /// The v2 refusal threshold (`2π/3` rad = 120°).
        threshold_rad: f64,
        /// Same in degrees.
        threshold_deg: f64,
    },

    /// Post-MC mesh-CSG operation (mating-features stage) failed.
    ///
    /// The S3 plumbing surface raises this only when the input mesh
    /// fails manifold3d's shared-index manifoldness requirement at
    /// `IndexedMesh → Manifold` conversion (recon §11 item 3); the
    /// live pipeline's `solid_to_mm_mesh` output is shared-index by
    /// construction and the empty-`Vec<MatingTransform>` case
    /// short-circuits before the conversion, so this variant
    /// typically fires only when S4-S6 emit concrete transforms
    /// against an unexpected mesh shape. `context` names the
    /// failing step, `source` carries the manifold3d error chain.
    #[error("mesh-CSG operation failed for {target} ({context}): {source}")]
    MeshCsg {
        /// Which output failed.
        target: CastTarget,
        /// Short description of the failing step
        /// (e.g., `"mesh→manifold conversion"`).
        context: String,
        /// Underlying manifold3d error.
        #[source]
        source: manifold3d::CsgError,
    },

    /// A layer's computed pour mass exceeds the per-pour
    /// [`crate::CastSpec::mass_budget_kg`] gate. Fires before any
    /// meshing or STL write — pre-write atomicity guarantees no
    /// side effects.
    ///
    /// Compared per-layer (single pour event), not aggregated
    /// across same-material layers. The aggregate-holdings check
    /// is not enforced; F3 procedure-spec generation surfaces it
    /// as a "verify total holdings against the per-material sums"
    /// reminder for the workshop user.
    #[error(
        "layer {layer_index} ({material_display_name}) pour mass {mass_kg:.4} kg \
         exceeds budget {budget_kg:.4} kg"
    )]
    MassBudgetExceeded {
        /// Index into [`crate::CastSpec::layers`].
        layer_index: usize,
        /// Carried-through layer material display name.
        material_display_name: String,
        /// Computed pour mass in kilograms for this layer's shell.
        mass_kg: f64,
        /// Per-pour budget the layer overran ([`crate::CastSpec::mass_budget_kg`]).
        budget_kg: f64,
    },
}
