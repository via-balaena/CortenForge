//! `EditResult` — γ-locked three-variant topology classification.
//!
//! 1-tet θ-only variation always yields `ParameterOnly`.
//! `TopologyPreserving` / `TopologyChanging` reactivate at Phase G when
//! the SDF→mesh pipeline enters (spec §8, `sdf_bridge/`).

/// Classification of a design edit's effect on mesh topology.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EditResult {
    /// θ perturbation only — mesh topology and connectivity unchanged.
    ParameterOnly,
    /// Mesh vertex positions move, topology unchanged (e.g. SDF
    /// level-set translation).
    TopologyPreserving,
    /// Topology changes (vertex / tet addition, removal, reconnection).
    TopologyChanging,
}
