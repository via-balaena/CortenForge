//! Foundation patch for row 18 (`contact-force-readout`) — verify
//! [`PenaltyRigidContact::per_pair_readout`] is structurally equivalent
//! to the existing public [`ContactModel::active_pairs`] +
//! [`ContactModel::gradient`] surfaces.
//!
//! Row 14 (`compressive-block`) ships with an inline manual
//! reconstruction of per-active-pair forces from known plane geometry
//! (`examples/sim-soft/compressive-block/src/main.rs`'s top-face walk
//! at the `F_R_FEM` aggregation point), duplicating
//! [`penalty.rs`]'s gradient formula. Row 18 lifts that reconstruction
//! into a public surface; this test pins the lift's contract:
//!
//! 1. **Length parity** — `per_pair_readout(...)` returns the same
//!    number of entries as `active_pairs(...)` at the same positions.
//! 2. **Order parity** — the i-th readout describes the same
//!    `(vertex_id, primitive_id)` pair as the i-th `active_pairs`
//!    entry (both walk vertices outer × primitives inner).
//! 3. **Geometry parity** — every readout sits inside the contact
//!    band (`sd < d̂`), with `position` matching the input
//!    `positions[vertex_id]` bit-for-bit and `normal` matching
//!    `Sdf::grad(position)` bit-for-bit.
//! 4. **Force-vs-gradient parity** — `force_on_soft` is
//!    bit-equivalent to `−ContactGradient.contributions[0].1`
//!    (Newton's-3rd-law negation of the gradient the solver scatters
//!    into `f_int`). Bit-equality, not relative tolerance, because
//!    `per_pair_readout` and `gradient` execute identical arithmetic
//!    (`κ · (d̂ − sd) · n`) on the same `Sdf::eval` / `Sdf::grad`
//!    outputs.
//!
//! Test scene: Compressive-block scene helper `compressive_block_on_plane` at
//! `n_per_edge = 2` (smallest refinement, 27 vertices, 9 expected
//! active pairs at the rest configuration). The rest configuration
//! already places every top-face vertex strictly inside the d̂-band
//! (penetration `δ = 5e-5 m > d̂_override = 1e-5 m`) — no solver run
//! needed for the test, just helper construction + readout call.

use sim_soft::{
    ActivePairsFor, ContactModel, ContactPair, MaterialField, Mesh, PenaltyRigidContact,
    RigidPlane, SoftScene, Vec3,
};

/// Cube edge length (1 cm). Mirrors the compressive-block fixture / row 14.
const EDGE_LEN: f64 = 0.01;
/// Coarsest-refinement cell size — 9 expected active pairs at rest.
const CELL_SIZE: f64 = EDGE_LEN / 2.0;
/// Plate displacement (50 μm). Mirrors the compressive-block fixture / row 14.
const DISPLACEMENT: f64 = 5.0e-5;
/// Fixture-local penalty stiffness (mirrors `PENALTY_KAPPA_DEFAULT`).
const KAPPA: f64 = 1.0e4;
/// Fixture-local contact band override (100× smaller than the default).
const D_HAT_OVERRIDE: f64 = 1.0e-5;

const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

#[test]
// Mirrors `concentric_lame_shells.rs:678` precedent — single-test-fn
// readout-contract gates legitimately exceed clippy's 100-line soft
// cap once per-readout structural + geometric + force-vs-gradient +
// closed-form + sign-convention loops are inlined; extracting helpers
// would add indirection without improving clarity.
#[allow(clippy::too_many_lines)]
fn per_pair_readout_matches_active_pairs_and_gradient() {
    let materials = MaterialField::uniform(MU, LAMBDA);
    let (mesh, _bc, _initial, _default_contact) =
        SoftScene::compressive_block_on_plane(EDGE_LEN, CELL_SIZE, DISPLACEMENT, &materials);
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
    let contact = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT_OVERRIDE);

    let positions: Vec<Vec3> = mesh.positions().to_vec();

    let pairs = contact.active_pairs(&mesh, &positions);
    let readouts = contact.per_pair_readout(&mesh, &positions);

    // Length parity. At n=2 the top face is a 3×3 vertex grid (9
    // vertices); every top vertex penetrates by `δ = 5e-5 m`, well
    // past `d̂_override = 1e-5 m`, so 9 active pairs are expected.
    assert_eq!(
        readouts.len(),
        pairs.len(),
        "per_pair_readout length {} should match active_pairs length {}",
        readouts.len(),
        pairs.len(),
    );
    assert_eq!(
        readouts.len(),
        9,
        "expected 9 active pairs at n=2 rest config (3×3 top-face grid), got {}",
        readouts.len(),
    );

    for (idx, (readout, pair)) in readouts.iter().zip(pairs.iter()).enumerate() {
        // Order parity — readout's `pair` field describes the same
        // `(vertex_id, primitive_id)` as the matching `active_pairs`
        // entry. Both walks are vertices-outer × primitives-inner so
        // the orders agree by construction; this gate catches a
        // future regression where one walk diverges from the other.
        let ContactPair::Vertex {
            vertex_id: rid_v,
            primitive_id: rid_p,
        } = readout.pair;
        let ContactPair::Vertex {
            vertex_id: pid_v,
            primitive_id: pid_p,
        } = *pair;
        assert_eq!(
            (rid_v, rid_p),
            (pid_v, pid_p),
            "readout[{idx}] pair {rid_v:?}/{rid_p} should match active_pairs[{idx}] pair {pid_v:?}/{pid_p}",
        );

        // Geometry parity — `position` is the input vertex position
        // bit-for-bit (no transformation); `sd` is in the active
        // band; `normal` is the rigid plane's outward normal which
        // for `RigidPlane::new(Vec3::new(0,0,-1), ...)` is exactly
        // `(0, 0, -1)` everywhere.
        assert_eq!(
            readout.position, positions[rid_v as usize],
            "readout[{idx}] position should match positions[{rid_v}]",
        );
        assert!(
            readout.sd < D_HAT_OVERRIDE,
            "readout[{idx}] sd = {} should be < d_hat_override {}",
            readout.sd,
            D_HAT_OVERRIDE,
        );
        assert_eq!(
            readout.normal,
            Vec3::new(0.0, 0.0, -1.0),
            "readout[{idx}] normal should be the plane's outward normal (0, 0, -1)",
        );

        // Force-vs-gradient parity — `gradient` returns
        // `−κ·(d̂−sd)·n` per the trait contract; the soft-side force
        // is its negation. `per_pair_readout` and `gradient` execute
        // identical arithmetic on the same Sdf outputs, so equality
        // holds at the bit level (not just relative tolerance).
        let gradient = contact.gradient(pair, &positions);
        assert_eq!(
            gradient.contributions.len(),
            1,
            "Phase 5 vertex-vs-rigid pair contributes exactly one gradient entry",
        );
        let (grad_vid, grad_force) = gradient.contributions[0];
        assert_eq!(
            grad_vid, rid_v,
            "gradient contribution vertex_id should match the readout pair",
        );
        let force_from_gradient = -grad_force;
        assert_eq!(
            readout.force_on_soft.x.to_bits(),
            force_from_gradient.x.to_bits(),
            "readout[{idx}] force_on_soft.x should bit-match −gradient.x",
        );
        assert_eq!(
            readout.force_on_soft.y.to_bits(),
            force_from_gradient.y.to_bits(),
            "readout[{idx}] force_on_soft.y should bit-match −gradient.y",
        );
        assert_eq!(
            readout.force_on_soft.z.to_bits(),
            force_from_gradient.z.to_bits(),
            "readout[{idx}] force_on_soft.z should bit-match −gradient.z",
        );

        // Closed-form check on the formula itself: independent
        // computation of `κ · (d̂ − sd) · n` from the readout's own
        // `sd` + `normal` should bit-match `force_on_soft`.
        let recomputed = KAPPA * (D_HAT_OVERRIDE - readout.sd) * readout.normal;
        assert_eq!(
            readout.force_on_soft.x.to_bits(),
            recomputed.x.to_bits(),
            "readout[{idx}] force_on_soft.x should bit-match κ·(d̂−sd)·n.x",
        );
        assert_eq!(
            readout.force_on_soft.y.to_bits(),
            recomputed.y.to_bits(),
            "readout[{idx}] force_on_soft.y should bit-match κ·(d̂−sd)·n.y",
        );
        assert_eq!(
            readout.force_on_soft.z.to_bits(),
            recomputed.z.to_bits(),
            "readout[{idx}] force_on_soft.z should bit-match κ·(d̂−sd)·n.z",
        );

        // Sign sanity — the compressive block's plate sits at `z = L - δ` with outward
        // normal `n = -ẑ` (the `sd > 0` half-space is below the plate,
        // where the soft cube naturally lives). The top-face vertex at
        // `z = L` penetrates upward with `sd = -δ < 0`. The penalty
        // restoring force `+κ·(d̂-sd)·n` along `-ẑ` pushes the vertex
        // back DOWN out of the rigid; Newton's-3rd-law reaction on the
        // plate is `+ẑ` (the compressive-block fixture's positive `F_R_FEM`).
        assert!(
            readout.force_on_soft.z < 0.0,
            "readout[{idx}] force_on_soft.z = {} should be negative \
             (compressive-block plane outward normal n = -ẑ; restoring force pushes soft vertex \
             DOWN out of the rigid; rigid reaction is +ẑ)",
            readout.force_on_soft.z,
        );
    }

    // Aggregate: `Σ readouts.force_on_soft.z` should equal the
    // negation of the row-14-style F_R_FEM reconstruction (which
    // sums `+κ·(d̂-sd)` per active vertex — the *reaction on the
    // rigid*, not the force on the soft). Bit-equivalence holds at
    // the per-pair level (verified above), so the sum holds at
    // floating-point-summation parity.
    let sum_readout_z: f64 = readouts.iter().map(|r| r.force_on_soft.z).sum();
    let mut sum_manual_reaction: f64 = 0.0;
    for r in &readouts {
        let ContactPair::Vertex { vertex_id, .. } = r.pair;
        let z_v = positions[vertex_id as usize].z;
        let sd_manual = EDGE_LEN - DISPLACEMENT - z_v;
        // Manual reconstruction mirrors row 14
        // (compressive-block/src/main.rs at the F_R_FEM aggregation
        // point) — the sd computation is geometry-specific to a
        // plane with `n = -ẑ` and `offset = δ - L`.
        if sd_manual < D_HAT_OVERRIDE {
            sum_manual_reaction += KAPPA * (D_HAT_OVERRIDE - sd_manual);
        }
    }
    // Rigid reaction (positive) = −soft-side force.z (negative).
    assert_eq!(
        sum_readout_z.to_bits(),
        (-sum_manual_reaction).to_bits(),
        "Σ readouts.force_on_soft.z = {} should bit-match −manual_reaction = {}",
        sum_readout_z,
        -sum_manual_reaction,
    );
}
