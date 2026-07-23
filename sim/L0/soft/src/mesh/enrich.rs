//! Tet4 → Tet10 edge-enrichment — the higher-order mesh producer.
//!
//! [`enrich_tet4_to_tet10`] takes a linear (4-node) tet mesh and adds the
//! midside nodes a [`Tet10`](crate::element::Tet10) element needs: one node at
//! the midpoint of each unique edge, shared between every tet that touches
//! that edge (a *conforming* enrichment). The output is a plain
//! [`Tet10Topology`] the Tet10 mesh-topology channel consumes downstream
//! (ladder rung 3a); this rung is the producer only, with no solver wiring.
//!
//! # Two invariants that keep the enrichment correct
//!
//! - **Corners are preserved verbatim.** The 4 corner `VertexId`s and their
//!   positions are unchanged; the new midside nodes are *appended after* every
//!   corner (`positions[..n_corners]` is bit-identical to the input). Keeping
//!   the corner id-space fixed is what lets rung 3a stay bit-identical to Tet4
//!   while midsides are pinned.
//! - **Identity via the global `(min, max)` key; local slot via the canonical
//!   table.** A midside node's *identity* (dedup across the tets sharing its
//!   edge) is keyed on the sorted global corner pair, exactly like the crate's
//!   other edge dedups ([`boundary_faces_from_topology`](super::boundary_faces_from_topology),
//!   `stuffing::edge_key`). Its *local slot* `4 + i` comes from
//!   [`TET10_EDGE_NODES`] and nothing else. **These must never be conflated:**
//!   the same global edge occupies different local slots in different tets, so
//!   letting the sorted key pick the slot would give each element its own
//!   silently-permuted edge→node map (§5 step 2). The
//!   local-slot-vs-global-identity test pins this.

use std::collections::HashMap;

use crate::Vec3;
use crate::element::TET10_EDGE_NODES;

use super::VertexId;

/// Enriched Tet10 connectivity produced by [`enrich_tet4_to_tet10`].
///
/// The corner vertices keep their input `VertexId`s and positions; the
/// deduplicated edge-midpoint vertices are appended after them. Consumed by
/// the Tet10 mesh-topology channel (ladder rung 3a).
#[derive(Clone, Debug, PartialEq)]
pub struct Tet10Topology {
    /// Corner positions (indices `0..n_corners`, unchanged from the input)
    /// followed by the deduplicated edge-midpoint positions.
    pub positions: Vec<Vec3>,
    /// Ten-node connectivity per tet. Slots `0..4` are the input corners
    /// (verbatim); slots `4..10` are the midside `VertexId`s, slot `4 + i`
    /// assigned from edge `i` of [`TET10_EDGE_NODES`].
    pub tets: Vec<[VertexId; 10]>,
    /// Number of corner vertices (= the input `positions.len()`). Every
    /// midside `VertexId` is `>= n_corners`; rung 3a uses this split to keep
    /// corners free and midsides pinned.
    pub n_corners: usize,
}

/// Enrich a linear tet mesh (`positions` + 4-node `tets`) into conforming
/// [`Tet10Topology`].
///
/// One midside node is created per *unique* edge (deduplicated across tets by
/// the sorted global corner pair) and placed at that edge's midpoint. Corner
/// ids and positions are preserved; midside ids are appended, assigned in
/// first-encounter order for determinism (the `HashMap` serves identity
/// lookup only — output order comes from the `tets` walk, sister to
/// [`boundary_faces_from_topology`](super::boundary_faces_from_topology)).
///
/// # Panics
///
/// Panics with an out-of-bounds index if any `tets` entry references a
/// `VertexId >= positions.len()` — the connectivity and positions must share
/// one mesh's id space, as every index-by-`VertexId` helper in this crate
/// requires.
//
// `len() as VertexId` is the `VertexId = u32` API tax (sister to
// `referenced_vertices`): vertex counts stay far below `u32::MAX`.
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub fn enrich_tet4_to_tet10(positions: &[Vec3], tets: &[[VertexId; 4]]) -> Tet10Topology {
    let n_corners = positions.len();
    let mut out_positions = positions.to_vec();
    // Sorted global `(min, max)` corner pair → midside `VertexId`. Identity
    // only: never the source of the local slot (see module docs).
    let mut midside_of: HashMap<(VertexId, VertexId), VertexId> = HashMap::new();

    let tets = tets
        .iter()
        .map(|&corners| {
            let mut nodes = [0; 10];
            nodes[..4].copy_from_slice(&corners);
            for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
                // `a`/`b` are LOCAL corner indices; look up the two global
                // vertices, then dedup on their sorted pair.
                let (va, vb) = (corners[a], corners[b]);
                let key = if va <= vb { (va, vb) } else { (vb, va) };
                let mid = *midside_of.entry(key).or_insert_with(|| {
                    let id = out_positions.len() as VertexId;
                    let midpoint = (positions[va as usize] + positions[vb as usize]) * 0.5;
                    out_positions.push(midpoint);
                    id
                });
                nodes[4 + i] = mid;
            }
            nodes
        })
        .collect();

    Tet10Topology {
        positions: out_positions,
        tets,
        n_corners,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A single tet with deliberately asymmetric corner positions, so every
    /// edge midpoint is distinct and an edge→slot permutation cannot hide.
    fn single_asymmetric_tet() -> (Vec<Vec3>, Vec<[VertexId; 4]>) {
        let positions = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.3, 0.1, -0.2),
            Vec3::new(0.2, 1.7, 0.3),
            Vec3::new(-0.1, 0.4, 2.1),
        ];
        let tets = vec![[0, 1, 2, 3]];
        (positions, tets)
    }

    #[test]
    fn corners_and_positions_preserved() {
        // n_corners split, corner ids in slots 0..4, and the corner positions
        // untouched — the bit-identity precondition rung 3a relies on.
        let (positions, tets) = single_asymmetric_tet();
        let out = enrich_tet4_to_tet10(&positions, &tets);

        assert_eq!(out.n_corners, 4);
        assert_eq!(&out.positions[..4], &positions[..]);
        assert_eq!(&out.tets[0][..4], &[0, 1, 2, 3]);
    }

    #[test]
    fn single_tet_creates_six_distinct_midsides() {
        // Six edges → six midside nodes, all appended after the corners.
        let (positions, tets) = single_asymmetric_tet();
        let out = enrich_tet4_to_tet10(&positions, &tets);

        assert_eq!(out.positions.len(), 4 + 6);
        let midsides = &out.tets[0][4..10];
        for (k, &m) in midsides.iter().enumerate() {
            assert!(m >= 4, "midside {k} id {m} must be >= n_corners");
        }
        let unique: std::collections::HashSet<_> = midsides.iter().collect();
        assert_eq!(unique.len(), 6, "the six midsides must be distinct");
    }

    #[test]
    fn local_slots_match_canonical_table() {
        // ★ The ordering detector: slot 4+i must hold the midpoint of the
        // corner edge TET10_EDGE_NODES[i]. Asymmetric positions make every
        // midpoint distinct, so a permuted table fails here.
        let (positions, tets) = single_asymmetric_tet();
        let out = enrich_tet4_to_tet10(&positions, &tets);
        let corners = tets[0];

        for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
            let expected = (positions[corners[a] as usize] + positions[corners[b] as usize]) * 0.5;
            let got = out.positions[out.tets[0][4 + i] as usize];
            assert!(
                (got - expected).norm() < 1e-15,
                "slot {} (edge {a}-{b}): got {got:?}, expected {expected:?}",
                4 + i
            );
        }
    }

    #[test]
    fn shared_edge_dedups_across_differing_local_slots() {
        // ★ Conforming + identity-vs-slot: two tets share the GLOBAL edge
        // {1, 2}, but it sits at a DIFFERENT local slot in each — slot 5
        // (edge (1,2)) in tet A, slot 6 (edge (0,2)) in tet B. They must
        // still resolve to one shared midside node, each at the correct slot.
        let positions = vec![
            Vec3::new(0.0, 0.0, 0.0), // 0
            Vec3::new(1.0, 0.0, 0.0), // 1
            Vec3::new(0.0, 1.0, 0.0), // 2
            Vec3::new(0.0, 0.0, 1.0), // 3
            Vec3::new(1.0, 1.0, 0.5), // 4
            Vec3::new(0.5, 0.0, 1.5), // 5
        ];
        // Tet A: global edge {1,2} = local (1,2) → slot 5.
        // Tet B: local corners [1,4,2,5] → global edge {1,2} = local (0,2) → slot 6.
        let tets = vec![[0, 1, 2, 3], [1, 4, 2, 5]];
        let out = enrich_tet4_to_tet10(&positions, &tets);

        let shared_a = out.tets[0][5]; // tet A, edge (1,2)
        let shared_b = out.tets[1][6]; // tet B, edge (0,2) = global {1,2}
        assert_eq!(
            shared_a, shared_b,
            "shared global edge must dedup to one node"
        );

        let expected = (positions[1] + positions[2]) * 0.5;
        assert!((out.positions[shared_a as usize] - expected).norm() < 1e-15);

        // Two tets, 6 edges each, one shared → 11 unique midsides.
        assert_eq!(out.positions.len(), 6 + 11);

        // And every slot in BOTH tets still matches the canonical table.
        for (t, &corners) in tets.iter().enumerate() {
            for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
                let expected =
                    (positions[corners[a] as usize] + positions[corners[b] as usize]) * 0.5;
                let got = out.positions[out.tets[t][4 + i] as usize];
                assert!(
                    (got - expected).norm() < 1e-15,
                    "tet {t} slot {}: table mismatch",
                    4 + i
                );
            }
        }
    }

    #[test]
    fn deterministic_midside_numbering() {
        // First-encounter assignment order is stable across runs.
        let (positions, tets) = single_asymmetric_tet();
        let a = enrich_tet4_to_tet10(&positions, &tets);
        let b = enrich_tet4_to_tet10(&positions, &tets);
        assert_eq!(a, b);
    }
}
