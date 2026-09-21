//! Tet-incidence filter on the contact active-set walks — a vertex that
//! no tet references is not part of the body, so it cannot carry a
//! contact pair.
//!
//! # What was wrong
//!
//! `Mesh::positions()` is the mesh's storage array, not its body.
//! `SdfMeshedTetMesh` retains every BCC lattice point it stuffed,
//! including corners of lattice tets that fell entirely outside the
//! SDF, and those orphans sit at their rest lattice coordinates —
//! routinely inside a rigid primitive, where they are maximally
//! "active". Both vertex-walking contact models iterated `positions()`
//! directly, so the active set was dominated by points that are not
//! the body.
//!
//! Measured on this file's fixture — a 1 cm sphere at a 3 mm cell
//! (`SoftScene::sphere_on_plane`) against a plane cut 2 mm into the
//! rest sphere, which every gate here uses except the solve one (that
//! takes the scene's own tangent plane, so the step is the shipped
//! configuration). `fixture_is_orphan_dominated` is the producer and
//! prints the current numbers on every run.
//!
//! # What it did NOT do
//!
//! It did not corrupt a solve. An unreferenced vertex is
//! Dirichlet-clamped by the construction-time orphan auto-pin
//! (`effective_pinned` — it must be, or its zero-mass, zero-stiffness
//! row would make the tangent singular), the Armijo merit is the FREE
//! residual norm, and a vertex pair's gradient and Hessian are diagonal
//! in that one vertex. So an orphan pair's blocks only ever landed in a
//! pinned row that `full_to_free_idx` drops.
//! `unreferenced_vertices_do_not_move_under_a_solve` pins that
//! mechanism, which is what licenses the filter to be a pure
//! subtraction: wasted SDF evaluations and a misleading active-set
//! diagnostic, removed.
//!
//! # The trap this file exists to keep shut
//!
//! `referenced_vertex_mask` must count a quadratic mesh's midside
//! nodes. They live in `positions()` alongside the corners and carry
//! real free DOFs under a Tet10 element, but they are absent from
//! `Mesh::tet_vertices` — so the obvious corner-only incidence walk
//! calls every one of them an orphan and silently deletes the contact
//! on a Tet10 body. `penalty_keeps_tet10_midside_nodes` is the gate;
//! on this fixture a corner-only mask would have dropped 3 092 of the
//! 3 653 live nodes.

#![allow(
    clippy::expect_used,
    // `pair_nodes` / `pair_key` refuse to guess an incidence rule for a
    // future `ContactPair` variant; a loud panic IS the gate there.
    clippy::panic
)]

use std::collections::BTreeSet;

use sim_soft::{
    ActivePairsFor, ContactPair, CpuNewtonSolver, IpcRigidContact, MaterialField, Mesh,
    PenaltyRigidContact, PenaltyRigidContactSolver, RigidPlane, SceneInitial, SdfMeshedTetMesh,
    SingleTetMesh, SoftScene, Solver, SolverConfig, Tet4, Tet10Mesh, Vec3, VertexId,
    filter_pair_readouts_to_referenced, referenced_vertex_mask, referenced_vertices,
};

// ── Fixture ──────────────────────────────────────────────────────────

/// Sphere radius (1 cm) — mirrors `hertz_sphere_plane.rs` and
/// `contact_drop_rest.rs` so the orphan profile is the one those
/// fixtures already live with.
const RADIUS: f64 = 1.0e-2;

/// Cell size (3 mm) — the Hertzian fixture's coarsest level. Coarse on
/// purpose: it maximises the orphan share, which is the thing under
/// test, and keeps the one solve in this file cheap.
const CELL_SIZE: f64 = 3.0e-3;

/// Lamé pair `(μ, λ)` — Ecoflex 00-30 + carbon black at `ν = 0.4`,
/// mirroring the contact-active regression net.
const MU: f64 = 2.0e5;
const LAMBDA: f64 = 8.0e5;

/// Penalty / barrier stiffness for the fixture-local primitives.
const KAPPA: f64 = 1.0e4;

/// Contact band (1 mm) — `PENALTY_DHAT_DEFAULT`.
const D_HAT: f64 = 1.0e-3;

/// Plane depth into the rest sphere (2 mm). The plane is raised INTO
/// the body rather than tangent to the south pole so that both halves
/// of every gate are non-vacuous at once: real body vertices are in
/// band (the "keeps" half) while the lattice orphans below `z = −R`
/// are also in band (the "drops" half).
const PLANE_CUT: f64 = 2.0e-3;

/// `+ẑ`-normal ground plane whose surface sits `PLANE_CUT` above the
/// rest sphere's south pole. `signed_distance(p) = p.z − offset`.
fn cutting_plane() -> RigidPlane {
    RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), PLANE_CUT - RADIUS)
}

/// The shared Tet4 fixture: a BCC-stuffed sphere (orphans and all) plus
/// its scene boundary conditions, initial state and drive.
fn sphere_fixture() -> (
    SdfMeshedTetMesh,
    sim_soft::BoundaryConditions,
    SceneInitial,
    sim_ml_chassis::Tensor<f64>,
) {
    let field = MaterialField::uniform(MU, LAMBDA);
    let (mesh, bc, initial, _scene_contact, theta) =
        SoftScene::sphere_on_plane(RADIUS, CELL_SIZE, 1.0, field)
            .expect("sphere_on_plane should mesh at radius = 1 cm, cell = 3 mm");
    (mesh, bc, initial, theta)
}

/// Every `VertexId` a contact pair names — the vertex for a vertex
/// pair, all six nodes for a face pair.
///
/// `ContactPair` is `#[non_exhaustive]`, and a new variant is exactly
/// the case that needs an incidence rule chosen deliberately, so this
/// refuses to guess one.
fn pair_nodes(pair: &ContactPair) -> Vec<VertexId> {
    match *pair {
        ContactPair::Vertex { vertex_id, .. } => vec![vertex_id],
        ContactPair::Face { nodes, .. } => nodes.to_vec(),
        ref other => panic!(
            "a new ContactPair variant ({other:?}) needs its own tet-incidence rule before \
             these gates can cover it",
        ),
    }
}

/// Identity of a pair for set comparison — `ContactPair` carries a
/// `rest_area` payload and implements no `PartialEq`, so compare the
/// `(nodes, primitive)` identity it is keyed by.
fn pair_key(pair: &ContactPair) -> (Vec<VertexId>, u32) {
    let primitive = match *pair {
        ContactPair::Vertex { primitive_id, .. } | ContactPair::Face { primitive_id, .. } => {
            primitive_id
        }
        ref other => panic!("a new ContactPair variant ({other:?}) needs a key rule"),
    };
    (pair_nodes(pair), primitive)
}

/// Assert that no pair in `pairs` names a vertex outside `mask`, naming
/// the first offender. `label` identifies the walk under test.
fn assert_no_unreferenced_node(pairs: &[ContactPair], mask: &[bool], label: &str) {
    let offender = pairs
        .iter()
        .flat_map(pair_nodes)
        .find(|&v| !mask[v as usize]);
    assert!(
        offender.is_none(),
        "{label} emitted a pair on vertex {offender:?}, which no tet references — \
         positions() is the lattice, not the body",
    );
}

// ── Non-vacuity: the fixture must actually contain the defect ────────

/// The whole file is vacuous if the fixture has no orphans in the
/// contact band, so measure that first and print the profile the other
/// gates rest on.
#[test]
fn fixture_is_orphan_dominated() {
    let (mesh, _bc, _initial, _theta) = sphere_fixture();
    let mask = referenced_vertex_mask(&mesh);
    let n_vertices = mesh.n_vertices();
    let n_referenced = mask.iter().filter(|&&r| r).count();
    let n_orphan = n_vertices - n_referenced;

    assert!(
        n_orphan > 0,
        "fixture has no unreferenced vertices at radius = {RADIUS}, cell = {CELL_SIZE}; \
         every gate in this file would be vacuous",
    );

    // Orphan reach: the lattice extends well past the body it meshed,
    // which is why orphans land inside a primitive placed at the body's
    // surface.
    let positions = mesh.positions();
    let body_min_z = positions
        .iter()
        .enumerate()
        .filter(|(i, _)| mask[*i])
        .map(|(_, p)| p.z)
        .fold(f64::INFINITY, f64::min);
    let orphan_min_z = positions
        .iter()
        .enumerate()
        .filter(|(i, _)| !mask[*i])
        .map(|(_, p)| p.z)
        .fold(f64::INFINITY, f64::min);
    assert!(
        orphan_min_z < body_min_z,
        "orphans reach z = {orphan_min_z} but the body bottoms out at {body_min_z}; \
         the lattice no longer extends past the body and this fixture no longer \
         reproduces the defect",
    );

    // How many pairs the unfiltered walk would have emitted, computed
    // through a producer the filter does not touch: `per_pair_readout`
    // keeps the raw list by design.
    let contact = PenaltyRigidContact::with_params(vec![cutting_plane()], KAPPA, D_HAT);
    let rest: Vec<Vec3> = positions.to_vec();
    let raw = contact.per_pair_readout(&mesh, &rest);
    let n_raw_orphan = raw
        .iter()
        .flat_map(|r| pair_nodes(&r.pair))
        .filter(|&v| !mask[v as usize])
        .count();

    assert!(
        n_raw_orphan > 0,
        "the unfiltered walk emits {} pairs but none on an orphan, so the filter \
         removes nothing here",
        raw.len(),
    );

    // Loss-free for any sim-soft mesh size.
    #[allow(clippy::cast_precision_loss)]
    let orphan_share = 100.0 * n_raw_orphan as f64 / raw.len() as f64;
    eprintln!(
        "fixture profile: {n_vertices} vertices, {n_referenced} referenced, {n_orphan} orphan; \
         unfiltered active set {} pairs of which {n_raw_orphan} orphan ({orphan_share:.1} %)",
        raw.len(),
    );
}

// ── The filter itself ────────────────────────────────────────────────

/// Penalty's active-set walk names no unreferenced vertex, and agrees
/// exactly with the established readout-side filter.
///
/// The equality is the load-bearing half: it pins the new walk against
/// `filter_pair_readouts_to_referenced`, an independent implementation
/// of the same rule that predates it, rather than against a count that
/// a mesher change would rot.
#[test]
fn penalty_active_pairs_names_no_unreferenced_vertex() {
    let (mesh, _bc, _initial, _theta) = sphere_fixture();
    let mask = referenced_vertex_mask(&mesh);
    let contact = PenaltyRigidContact::with_params(vec![cutting_plane()], KAPPA, D_HAT);
    let rest: Vec<Vec3> = mesh.positions().to_vec();

    let pairs = contact.active_pairs(&mesh, &rest);
    assert_no_unreferenced_node(&pairs, &mask, "PenaltyRigidContact::active_pairs");

    // Non-vacuous on both sides: it kept body pairs and dropped orphan
    // ones.
    assert!(
        !pairs.is_empty(),
        "the filter removed every pair — a plane cut {PLANE_CUT} m into the rest sphere \
         must leave body vertices in band",
    );
    let raw = contact.per_pair_readout(&mesh, &rest);
    assert!(
        raw.len() > pairs.len(),
        "the unfiltered walk ({}) should exceed the filtered one ({}) on a fixture with \
         {} orphans",
        raw.len(),
        pairs.len(),
        mask.iter().filter(|&&r| !r).count(),
    );

    // Exact agreement with the readout-side filter, pair for pair and
    // in order (both walks are vertices-outer × primitives-inner).
    let referenced = referenced_vertices(&mesh);
    let filtered_readouts = filter_pair_readouts_to_referenced(raw, &referenced);
    let from_readouts: Vec<(Vec<VertexId>, u32)> = filtered_readouts
        .iter()
        .map(|r| pair_key(&r.pair))
        .collect();
    let from_pairs: Vec<(Vec<VertexId>, u32)> = pairs.iter().map(pair_key).collect();
    assert_eq!(
        from_pairs, from_readouts,
        "active_pairs must equal per_pair_readout filtered through \
         filter_pair_readouts_to_referenced — the two apply the same incidence rule",
    );
}

/// The IPC barrier's per-vertex path (taken on a linear mesh, which
/// surfaces no six-node boundary faces) names no unreferenced vertex.
///
/// The barrier floors `d` at `d̂·1e-6`, so a deeply buried orphan's
/// force does not grow with depth — it saturates at the floor's
/// `O(κ·d̂·1e6)`, the same 1 mm or 40 mm inside.
#[test]
fn ipc_vertex_pairs_name_no_unreferenced_vertex() {
    let (mesh, _bc, _initial, _theta) = sphere_fixture();
    assert!(
        mesh.boundary_faces6().is_none(),
        "this gate covers the per-VERTEX path; a mesh with six-node boundary faces \
         would take the face path instead",
    );
    let mask = referenced_vertex_mask(&mesh);
    let contact = IpcRigidContact::with_params(vec![cutting_plane()], KAPPA, D_HAT);
    let rest: Vec<Vec3> = mesh.positions().to_vec();

    let pairs = contact.active_pairs(&mesh, &rest);
    assert_no_unreferenced_node(&pairs, &mask, "IpcRigidContact vertex path");
    assert!(
        !pairs.is_empty(),
        "the filter removed every IPC pair — body vertices must remain in band",
    );

    let raw = contact.per_pair_readout(&mesh, &rest);
    assert!(
        raw.len() > pairs.len(),
        "the unfiltered IPC walk ({}) should exceed the filtered one ({})",
        raw.len(),
        pairs.len(),
    );
}

/// ⚠ The midside trap. A corner-only incidence walk would call every
/// Tet10 midside node an orphan and delete the contact on a quadratic
/// body. Penalty stays on the per-vertex path regardless of element
/// order, so it is where that mistake would land.
#[test]
fn penalty_keeps_tet10_midside_nodes() {
    let (tet4, _bc, _initial, _theta) = sphere_fixture();
    let tet10: Tet10Mesh = Tet10Mesh::from_tet4(&tet4);
    let mask = referenced_vertex_mask(&tet10 as &dyn Mesh<_>);

    // The corner-only walk, built here so the gate compares against
    // the mistake rather than describing it.
    let mut corners: BTreeSet<VertexId> = BTreeSet::new();
    let mut midsides: BTreeSet<VertexId> = BTreeSet::new();
    // Tet counts stay far below u32::MAX (the Mesh-trait `TetId` tax).
    #[allow(clippy::cast_possible_truncation)]
    let n_tets = tet10.n_tets() as u32;
    for tet in 0..n_tets {
        corners.extend(tet10.tet_vertices(tet));
        midsides.extend(
            tet10
                .tet_midside_nodes(tet)
                .expect("a Tet10Mesh surfaces midside nodes for every tet"),
        );
    }
    assert!(
        !midsides.is_empty(),
        "enrichment produced no midside nodes, so this gate cannot see the trap",
    );
    // The premise the midside count rests on: corners and midsides are
    // disjoint sets, so a corner-only mask does not accidentally cover
    // any midside. Without this, "a corner-only mask would have dropped
    // all of them" is an assumption rather than a measurement.
    assert!(
        midsides.is_disjoint(&corners),
        "{} midside node(s) are also corner nodes — a corner-only mask would then cover \\
         part of the midside set, and this gate's premise does not hold",
        midsides.intersection(&corners).count(),
    );
    for &m in &midsides {
        assert!(
            mask[m as usize],
            "midside node {m} is a live Tet10 DOF but the incidence mask calls it an orphan",
        );
    }
    eprintln!(
        "tet10 incidence: {} corners + {} midsides = {} live of {} stored",
        corners.len(),
        midsides.len(),
        mask.iter().filter(|&&r| r).count(),
        tet10.n_vertices(),
    );

    // And the walk itself keeps them: with the plane cut into the body,
    // midside nodes near the cut must appear in the active set.
    let contact = PenaltyRigidContact::with_params(vec![cutting_plane()], KAPPA, D_HAT);
    let rest: Vec<Vec3> = tet10.positions().to_vec();
    let pairs = contact.active_pairs(&tet10, &rest);
    assert_no_unreferenced_node(&pairs, &mask, "PenaltyRigidContact on a Tet10 mesh");
    let n_midside_pairs = pairs
        .iter()
        .flat_map(pair_nodes)
        .filter(|v| midsides.contains(v))
        .count();
    assert!(
        n_midside_pairs > 0,
        "no midside node reached the active set on a plane cut {PLANE_CUT} m into the body — \
         a corner-only incidence mask would pass this file's other gates while deleting \
         Tet10 contact",
    );
}

/// The IPC face path needs no filter and must not acquire one: a
/// six-node boundary face is derived from tet connectivity, so it
/// structurally cannot name a vertex no tet references.
#[test]
fn ipc_face_pairs_are_structurally_referenced() {
    let (tet4, _bc, _initial, _theta) = sphere_fixture();
    let tet10: Tet10Mesh = Tet10Mesh::from_tet4(&tet4);
    assert!(
        tet10.boundary_faces6().is_some(),
        "a Tet10Mesh must surface six-node boundary faces or this gate covers the wrong path",
    );
    let mask = referenced_vertex_mask(&tet10 as &dyn Mesh<_>);
    let contact = IpcRigidContact::with_params(vec![cutting_plane()], KAPPA, D_HAT);
    let rest: Vec<Vec3> = tet10.positions().to_vec();

    let pairs = contact.active_pairs(&tet10, &rest);
    assert!(
        !pairs.is_empty(),
        "no face pairs on a plane cut {PLANE_CUT} m into the body",
    );
    assert!(
        pairs.iter().all(|p| matches!(p, ContactPair::Face { .. })),
        "a quadratic mesh must take the face barrier, not the vertex path",
    );
    assert_no_unreferenced_node(&pairs, &mask, "IpcRigidContact face path");
}

// ── The trait-level invariant, for mesh types that do not exist yet ──

/// A vertex any boundary face names must be in the mask, on every
/// `Mesh` impl in the tree.
///
/// This is the midside trap stated against the TRAIT instead of
/// against one fixture. `referenced_vertex_mask` learns incidence from
/// `tet_vertices` + `tet_midside_nodes`; the boundary-face channels are
/// built from the same connectivity, so the two must agree — and if a
/// future element type (P3, a shell) carries nodes the incidence walk
/// does not know about, its faces will name them and this fails, where
/// `penalty_keeps_tet10_midside_nodes` (which knows the word "midside")
/// would not.
///
/// A corner-only mask reddens this on the Tet10 arm, which is the
/// independent confirmation that it binds.
#[test]
fn every_boundary_face_node_is_in_the_mask() {
    fn check<M: sim_soft::Material>(label: &str, mesh: &dyn Mesh<M>) {
        let mask = referenced_vertex_mask(mesh);
        let mut checked = 0_usize;
        for face in mesh.boundary_faces() {
            for &v in face {
                assert!(
                    mask[v as usize],
                    "{label}: boundary_faces names vertex {v}, which the incidence mask \
                     calls an orphan",
                );
                checked += 1;
            }
        }
        if let Some(faces6) = mesh.boundary_faces6() {
            for face in faces6 {
                for &v in face {
                    assert!(
                        mask[v as usize],
                        "{label}: boundary_faces6 names node {v}, which the incidence mask \
                         calls an orphan",
                    );
                    checked += 1;
                }
            }
        }
        assert!(
            checked > 0,
            "{label} surfaced no boundary-face nodes, so this gate checked nothing",
        );
    }

    let field = MaterialField::uniform(MU, LAMBDA);
    let (tet4, _bc, _initial, _theta) = sphere_fixture();
    check("SdfMeshedTetMesh", &tet4);
    check(
        "Tet10Mesh",
        &Tet10Mesh::<sim_soft::NeoHookean>::from_tet4(&tet4) as &dyn Mesh<_>,
    );
    let (block, _bc2, _init2, _c2) =
        SoftScene::compressive_block_on_plane(0.02, 0.0025, 5.0e-5, &field);
    check("HandBuiltTetMesh", &block);
    check("SingleTetMesh", &SingleTetMesh::new(&field));
}

// ── The mechanism that licenses the filter ───────────────────────────

/// An unreferenced vertex cannot move, so nothing it was ever paired
/// with could reach the free system — which is why removing those pairs
/// changes the active set and not the answer.
///
/// Bit-for-bit, not approximately: the orphan auto-pin Dirichlet-clamps
/// these DOFs, so their final coordinates are their input coordinates
/// with no arithmetic applied at all. Made to fail by removing the
/// `effective_pinned` orphan union.
#[test]
fn unreferenced_vertices_do_not_move_under_a_solve() {
    let (mesh, bc, initial, theta) = sphere_fixture();
    let mask = referenced_vertex_mask(&mesh);
    let orphans: Vec<VertexId> = (0..mask.len())
        .filter(|&v| !mask[v])
        // Vertex counts stay far below u32::MAX.
        .map(|v| {
            #[allow(clippy::cast_possible_truncation)]
            let v = v as VertexId;
            v
        })
        .collect();
    assert!(
        !orphans.is_empty(),
        "no orphans in the fixture — nothing for this gate to hold still",
    );

    // The scene's own contact (plane tangent below the south pole) plus
    // the scene drive, so this is the shipped configuration rather than
    // a fixture-local one.
    let contact = PenaltyRigidContact::with_params(
        vec![RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), -RADIUS - D_HAT)],
        KAPPA,
        D_HAT,
    );
    let SceneInitial { x_prev, v_prev } = initial;
    let x_in: Vec<f64> = x_prev.as_slice().to_vec();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0e-3;
    let dt = cfg.dt;
    let solver: PenaltyRigidContactSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);
    let step = solver.replay_step(&x_prev, &v_prev, &theta, dt);

    for &v in &orphans {
        for axis in 0..3 {
            let i = 3 * (v as usize) + axis;
            assert_eq!(
                step.x_final[i].to_bits(),
                x_in[i].to_bits(),
                "orphan vertex {v} axis {axis} moved from {} to {} — the orphan auto-pin \
                 no longer holds, so contact pairs on unreferenced vertices could reach \
                 the free system",
                x_in[i],
                step.x_final[i],
            );
        }
    }

    // Non-vacuous: the body did move, so the solve was not a no-op.
    let moved = (0..mask.len())
        .filter(|&v| mask[v])
        .any(|v| (0..3).any(|a| step.x_final[3 * v + a].to_bits() != x_in[3 * v + a].to_bits()));
    assert!(
        moved,
        "no referenced vertex moved either — the solve did nothing, so holding the \
         orphans still proves nothing",
    );
}
