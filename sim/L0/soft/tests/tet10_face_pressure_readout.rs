//! Tet10 rung 8d — the **face-consistent contact-pressure readout**.
//!
//! On a Tet10 mesh the solver's contact is the surface-integrated
//! [`ContactPair::Face`] barrier (rung 8b: corners carry ~0 force, midsides
//! carry the load). Before 8d, [`IpcRigidContact::per_pair_readout`] was a
//! per-VERTEX diagnostic — it reported the per-vertex barrier force (not the
//! solver's face-integrated force) and computed tributary area from the
//! three-node corner boundary faces, so the loaded midside nodes got zero area
//! → `NaN` pressure → silently dropped from [`peak_contact_pressure`]. 8d makes
//! the readout FACE-consistent on a Tet10 mesh: per-node forces reconstructed
//! from the same face barrier the solver scatters, with a consistent-P2
//! **deformed** tributary.
//!
//! The diagnostic must faithfully reconstruct the solver's contact, so the gates
//! are decomposition/partition invariants and a direct solver-force match — not
//! an FD gate (a per-vertex proxy would pass a naive "midsides non-NaN" check
//! while misreporting the force distribution):
//!
//! - **(i)** midside `peak_contact_pressure` no longer `NaN`-drops on a Tet10
//!   face contact — finite, positive.
//! - **(ii)** faithful decomposition `pressure · area · n̂ == force_on_soft`
//!   per well-defined node (`Σ p·a·n̂ == Σ force_on_soft`).
//! - **(iii)** per-node forces match the SOLVER's face-barrier scatter
//!   (`Σ_nodes force_on_soft == −Σ_pairs gradient`), not the per-vertex barrier
//!   (corners ~0, midsides loaded — the non-vacuity arm).
//! - **(iv)** the Tet4 (linear-mesh) path is byte-identical to the per-vertex
//!   readout (`tet4_readout_is_byte_identical_per_vertex`).
//! - **(v)** an independent absolute pin: a flat face at a known uniform gap
//!   reads `peak_pressure == κ|b'(gap)|`, read off the barrier + geometry, with
//!   the shared `rest_area` scale cancelling (not through a re-solve).

#![allow(
    // `.expect(...)` on a known-in-band barrier value and `panic!` in a
    // let-else that destructures a readout the SUT only ever emits as a Vertex
    // pair — both document invariants of this gate (test precedent:
    // `bonded_layer_indentation.rs`).
    clippy::expect_used,
    clippy::panic,
    // `positions.len() as f64` (centroid) and `vid as VertexId` (usize count →
    // u32 id) — the Mesh-trait API tax; the test meshes hold ~dozens of
    // vertices, far below f64/u32 precision limits.
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation
)]

use sim_soft::{
    ActivePairsFor, ContactModel, ContactPair, HandBuiltTetMesh, IpcRigidContact, MaterialField,
    Mesh, RigidPlane, SphereSdf, Tet10Mesh, TranslatedSdf, Vec3, VertexId, peak_contact_pressure,
};
use std::collections::HashMap;

const KAPPA: f64 = 1.0e4;
const D_HAT: f64 = 1.0e-2;
const EDGE: f64 = 0.10;

fn field() -> MaterialField {
    MaterialField::uniform(1.0e5, 4.0e5)
}

/// The IPC log-barrier `κ·b'(sd)` — the module's documented barrier first
/// derivative, floored exactly as `IpcRigidContact::barrier` does. `None`
/// outside the support (`sd ≥ d̂`). Reproduced here so the pins are independent
/// of the SUT's private method (an oracle-matches-SUT companion).
fn kappa_b_prime(sd: f64) -> Option<f64> {
    if sd >= D_HAT {
        return None;
    }
    let d = sd.max(D_HAT * 1.0e-6);
    let r = d - D_HAT;
    let ln = (d / D_HAT).ln();
    Some(KAPPA * r.mul_add(-2.0 * ln, -(r * r) / d))
}

/// A Tet10 cube from a `uniform_block(2)` linear cube, plus its rest positions.
fn tet10_cube() -> Tet10Mesh {
    let tet4 = HandBuiltTetMesh::uniform_block(2, EDGE, &field());
    Tet10Mesh::from_tet4(&tet4)
}

/// Minimum z over the mesh positions (the flat cube base).
fn min_z(positions: &[Vec3]) -> f64 {
    positions.iter().map(|p| p.z).fold(f64::INFINITY, f64::min)
}

/// Independently assemble the solver's net contact residual per node by walking
/// the model's own `active_pairs` and summing each pair's `gradient` — the force
/// the Newton solver scatters. `force_on_soft == −residual`.
fn solver_residual(
    contact: &IpcRigidContact,
    mesh: &Tet10Mesh,
    positions: &[Vec3],
) -> HashMap<VertexId, Vec3> {
    let mut residual: HashMap<VertexId, Vec3> = HashMap::new();
    for pair in ActivePairsFor::active_pairs(contact, mesh, positions) {
        for (vid, force) in contact.gradient(&pair, positions).contributions {
            *residual.entry(vid).or_insert_with(Vec3::zeros) += force;
        }
    }
    residual
}

/// **Gates (i), (ii), (iii), (v) on a flat plane contact.** The cube's flat base
/// is placed a uniform gap over a rigid plane; every active-face node sits at the
/// same signed distance.
#[test]
fn flat_plane_face_readout_is_faithful_and_matches_solver() {
    let mesh = tet10_cube();
    let positions: Vec<Vec3> = mesh.positions().to_vec(); // rest == deformed here
    let z_base = min_z(&positions);
    let gap = 0.4 * D_HAT;
    // Plane just below the base, outward normal +z: base nodes sit at +gap.
    let plane = RigidPlane::new(Vec3::z(), z_base - gap);
    let contact = IpcRigidContact::with_params(vec![plane], KAPPA, D_HAT);

    let readouts = contact.per_pair_readout(&mesh, &positions);
    assert!(
        !readouts.is_empty(),
        "the base must produce active face contact"
    );

    // (i) peak_contact_pressure recovers a finite, positive value (midsides no
    // longer NaN-drop).
    let peak = peak_contact_pressure(&readouts);
    assert!(
        peak.is_finite() && peak > 0.0,
        "peak pressure must be finite & positive, got {peak}",
    );

    // (v) absolute pin: on a flat face at a UNIFORM gap the midside pressure is
    // exactly the barrier |κ·b'(gap)| — the shared rest_area/deformed-area scale
    // cancels (force ∝ rest_area, tributary ∝ deformed_area, equal here), so this
    // is read off the barrier + geometry, not a re-solve.
    let expected_peak = kappa_b_prime(gap).expect("gap is inside the band").abs();
    assert!(
        (peak - expected_peak).abs() < 1e-6 * expected_peak,
        "flat-face peak pressure must equal |κ·b'(gap)| = {expected_peak}, got {peak}",
    );

    // (iii) per-node forces MATCH the solver's face-barrier scatter, and the
    // distribution is corners≈0 / midsides-loaded (NOT the per-vertex barrier,
    // which would load corners too — the non-vacuity arm).
    let residual = solver_residual(&contact, &mesh, &positions);
    let mut sum_readout = Vec3::zeros();
    let mut sum_solver = Vec3::zeros();
    let mut max_corner_force = 0.0f64;
    let mut min_midside_force = f64::INFINITY;
    for r in &readouts {
        let ContactPair::Vertex { vertex_id, .. } = r.pair else {
            panic!("face-node readouts identify their node via a Vertex pair");
        };
        let solver_force = -residual
            .get(&vertex_id)
            .copied()
            .unwrap_or_else(Vec3::zeros);
        assert!(
            (r.force_on_soft - solver_force).norm() < 1e-9 * solver_force.norm().max(1e-9),
            "node {vertex_id}: readout force {:?} must match solver scatter {:?}",
            r.force_on_soft,
            solver_force,
        );
        sum_readout += r.force_on_soft;
        sum_solver += solver_force;
        if (vertex_id as usize) < mesh.n_corners() {
            max_corner_force = max_corner_force.max(r.force_on_soft.norm());
        } else {
            min_midside_force = min_midside_force.min(r.force_on_soft.norm());
        }
    }
    assert!(
        (sum_readout - sum_solver).norm() < 1e-9 * sum_solver.norm(),
        "Σ readout force {sum_readout:?} must equal −Σ solver residual {sum_solver:?}",
    );
    assert!(
        max_corner_force < 1e-9,
        "corners must carry ~0 face-barrier force (not the per-vertex barrier), got {max_corner_force:e}",
    );
    assert!(
        min_midside_force > 1e-3,
        "loaded midsides must carry real force, got min {min_midside_force:e}",
    );

    // (ii) faithful decomposition: pressure·area·n̂ reconstructs force_on_soft for
    // every well-defined (area > 0) node, and the sum reconstructs the total.
    let mut sum_recon = Vec3::zeros();
    let mut sum_force = Vec3::zeros();
    for r in &readouts {
        sum_force += r.force_on_soft;
        if r.tributary_area > 0.0 && r.pressure.is_finite() {
            let recon = r.pressure * r.tributary_area * r.normal;
            assert!(
                (recon - r.force_on_soft).norm() < 1e-12 * r.force_on_soft.norm().max(1e-12),
                "node decomposition p·a·n̂ {recon:?} must equal force {:?}",
                r.force_on_soft,
            );
            sum_recon += recon;
        }
    }
    assert!(
        (sum_recon - sum_force).norm() < 1e-9 * sum_force.norm(),
        "Σ p·a·n̂ {sum_recon:?} must equal Σ force_on_soft {sum_force:?}",
    );
}

/// **Curved-contact validation of the force-direction normal (rung 8d).** On a
/// small (high-curvature) sphere the per-node force is a Gauss-point sum whose
/// direction is NOT the node's SDF normal — so the readout reports the force
/// direction, which is what keeps the decomposition invariant exact. A readout
/// that reported the primitive normal instead would FAIL the per-node
/// decomposition here (the non-vacuity check asserts the two normals genuinely
/// differ). Curved corner tributaries can be ≤ 0 → `NaN` pressure, filtered.
#[test]
fn curved_sphere_face_readout_uses_force_direction_normal() {
    let mesh = tet10_cube();
    let positions: Vec<Vec3> = mesh.positions().to_vec();
    let z_base = min_z(&positions);
    let cx = positions.iter().map(|p| p.x).sum::<f64>() / positions.len() as f64;
    let cy = positions.iter().map(|p| p.y).sum::<f64>() / positions.len() as f64;
    // A small sphere (radius ≈ half an edge → genuinely curved faces) with its
    // apex a small gap below the base centre: central base faces contact, edges
    // fall outside the band.
    let radius = 0.5 * EDGE;
    let gap = 0.4 * D_HAT;
    let centre = Vec3::new(cx, cy, z_base - gap - radius);
    let sphere = TranslatedSdf {
        inner: SphereSdf { radius },
        offset: centre,
    };
    let contact = IpcRigidContact::with_params(vec![sphere], KAPPA, D_HAT);

    let readouts = contact.per_pair_readout(&mesh, &positions);
    assert!(
        !readouts.is_empty(),
        "the apex must produce active face contact"
    );

    let peak = peak_contact_pressure(&readouts);
    assert!(
        peak.is_finite() && peak > 0.0,
        "curved peak pressure must be finite & positive, got {peak}",
    );

    // Per-node decomposition holds EXACTLY (force-direction normal) — a
    // primitive-normal readout would miss this on a curved patch.
    for r in &readouts {
        if r.tributary_area > 0.0 && r.pressure.is_finite() {
            let recon = r.pressure * r.tributary_area * r.normal;
            assert!(
                (recon - r.force_on_soft).norm() < 1e-10 * r.force_on_soft.norm().max(1e-12),
                "curved node decomposition must be exact: p·a·n̂ {recon:?} vs force {:?}",
                r.force_on_soft,
            );
        }
    }

    // Non-vacuity: for a loaded node the reported (force-direction) normal
    // genuinely differs from the SDF normal at that node — so the choice is
    // observable and the decomposition test above is not trivially satisfiable
    // by the primitive normal.
    let mut max_normal_gap = 0.0f64;
    for r in &readouts {
        if r.force_on_soft.norm() > 1e-3 {
            let ContactPair::Vertex { vertex_id, .. } = r.pair else {
                unreachable!()
            };
            let node = positions[vertex_id as usize];
            let sdf_normal = (node - centre).normalize();
            max_normal_gap = max_normal_gap.max((r.normal - sdf_normal).norm());
        }
    }
    eprintln!("MEASURED max_normal_gap = {max_normal_gap:e}");
    assert!(
        max_normal_gap > 1e-4,
        "on a curved patch the force-direction normal must differ from the SDF \
         normal (else the decomposition gate is vacuous), max gap {max_normal_gap:e}",
    );

    // A curved patch drives at least one corner tributary ≤ 0 → NaN pressure,
    // filtered from the peak (corners carry ~0 barrier force).
    let any_corner_nan = readouts.iter().any(|r| {
        matches!(r.pair, ContactPair::Vertex { vertex_id, .. } if (vertex_id as usize) < mesh.n_corners())
            && !r.pressure.is_finite()
    });
    assert!(
        any_corner_nan,
        "expected a corner node with NaN pressure (≤0 tributary) on a curved patch",
    );
}

/// **(iv) Tet4 byte-identity.** On a linear mesh (`boundary_faces6` is `None`)
/// `per_pair_readout` takes the per-vertex path — every field bit-identical to an
/// independent reconstruction of the historical per-vertex formula (barrier force
/// `−κ·b'·n̂` + `boundary_vertex_areas` tributary + `|force|/area` pressure). A
/// regression in the extracted per-vertex path fails this to the ULP.
#[test]
fn tet4_readout_is_byte_identical_per_vertex() {
    let mesh = HandBuiltTetMesh::uniform_block(2, EDGE, &field());
    let positions: Vec<Vec3> = mesh.positions().to_vec();
    let z_base = min_z(&positions);
    let gap = 0.4 * D_HAT;
    let plane = RigidPlane::new(Vec3::z(), z_base - gap);
    let normal = Vec3::z(); // plane grad is constant
    let contact = IpcRigidContact::with_params(vec![plane], KAPPA, D_HAT);

    let readouts = contact.per_pair_readout(&mesh, &positions);
    assert!(
        !readouts.is_empty(),
        "the base must produce active vertex contact"
    );

    // Independent per-vertex reconstruction in the SAME walk order (vertex outer,
    // single primitive inner) with the SAME arithmetic.
    let areas = sim_soft::boundary_vertex_areas(&positions, mesh.boundary_faces());
    let mut expected = Vec::new();
    for (vid, &p) in positions.iter().enumerate() {
        let sd = p.z - (z_base - gap);
        if let Some(kbp) = kappa_b_prime(sd) {
            let force = -kbp * normal;
            let area = areas[vid];
            let pressure = if area > 0.0 {
                force.norm() / area
            } else {
                f64::NAN
            };
            expected.push((vid as VertexId, sd, force, area, pressure));
        }
    }

    assert_eq!(
        readouts.len(),
        expected.len(),
        "per-vertex readout count must match the independent walk",
    );
    for (r, (vid, sd, force, area, pressure)) in readouts.iter().zip(&expected) {
        let ContactPair::Vertex { vertex_id, .. } = r.pair else {
            panic!("Tet4 readout must be a Vertex pair");
        };
        assert_eq!(vertex_id, *vid, "vertex id / walk order");
        assert_eq!(r.sd.to_bits(), sd.to_bits(), "sd bits (vid {vid})");
        for k in 0..3 {
            assert_eq!(
                r.force_on_soft[k].to_bits(),
                force[k].to_bits(),
                "force_on_soft[{k}] bits (vid {vid})",
            );
            assert_eq!(
                r.normal[k].to_bits(),
                normal[k].to_bits(),
                "normal[{k}] bits"
            );
        }
        assert_eq!(
            r.tributary_area.to_bits(),
            area.to_bits(),
            "tributary_area bits (vid {vid})",
        );
        assert_eq!(
            r.pressure.to_bits(),
            pressure.to_bits(),
            "pressure bits (vid {vid})",
        );
    }
}
