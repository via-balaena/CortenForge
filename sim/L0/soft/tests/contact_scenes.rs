//! Unit-config asserts for the Phase 5 commit 6 contact-aware scene
//! helpers — `SoftScene::compressive_block_on_plane` (V-3a scaffold),
//! `SoftScene::sphere_on_plane` (V-3 scaffold), `SoftScene::dropping_sphere`
//! (V-5 scaffold). Per `phase_5_penalty_contact_scope.md` §8 commit 6
//! gate: "new scene helpers compile + their unit-config asserts pass."
//!
//! These are sanity-level configurational checks — vertex counts vs
//! `cell_size`, BC pinned/loaded vertex counts, contact-model primitive
//! count, plane-orientation conventions documented in each helper's
//! docstring, initial-state tensor shape. The full physical-correctness
//! gates (V-3a uniaxial closed-form, V-3 Hertz, V-5 drop-and-rest) live
//! at commits 8 / 9 / 10 — not preempted here.

#![allow(
    // Helpers' meshing return `Result<…, MeshingError>`; the canonical
    // V-3 / V-5 scaffolds either mesh by construction at the documented
    // cell sizes or surface a regression worth investigating. Mirrors
    // `concentric_lame_shells.rs` precedent.
    clippy::expect_used
)]

use approx::assert_relative_eq;
use sim_soft::{ContactModel, LoadAxis, MaterialField, Mesh, SceneInitial, SoftScene, Vec3};

// ── compressive_block_on_plane (V-3a scaffold) ────────────────────────────

/// 1 cm cube at 5 mm cell size — V-3a's coarsest refinement level
/// (`n_per_edge = 2`).
const V3A_EDGE_LEN: f64 = 0.01;
const V3A_CELL_SIZE: f64 = 0.005;
const V3A_DISPLACEMENT: f64 = 5.0e-4;

fn ecoflex_field() -> MaterialField {
    // Phase 4 IV-3 / IV-5 default Lamé pair (`(μ, λ) = (1e5, 4e5)` →
    // ν = 0.4 compressible Neo-Hookean). Material-independence of
    // unit-config asserts means any uniform pair works; use the
    // canonical one for parity with the rest of the regression net.
    MaterialField::uniform(1.0e5, 4.0e5)
}

#[test]
fn compressive_block_vertex_count_matches_cube_grid() {
    // n_per_edge = 2 → (2+1)³ = 27 vertices.
    let (mesh, _bc, _initial, _contact) = SoftScene::compressive_block_on_plane(
        V3A_EDGE_LEN,
        V3A_CELL_SIZE,
        V3A_DISPLACEMENT,
        &ecoflex_field(),
    );
    assert_eq!(mesh.n_vertices(), 27);
}

#[test]
fn compressive_block_tet_count_is_six_per_cell() {
    // n_per_edge = 2 → 2·2·2 = 8 cells × 6 tets = 48.
    let (mesh, _bc, _initial, _contact) = SoftScene::compressive_block_on_plane(
        V3A_EDGE_LEN,
        V3A_CELL_SIZE,
        V3A_DISPLACEMENT,
        &ecoflex_field(),
    );
    assert_eq!(mesh.n_tets(), 48);
}

#[test]
fn compressive_block_pins_entire_bottom_face() {
    // Bottom face at z = 0 in a (n+1)·(n+1) grid → 9 vertices full-pinned
    // (the constrained-modulus approximation — see helper docstring).
    let (_mesh, bc, _initial, _contact) = SoftScene::compressive_block_on_plane(
        V3A_EDGE_LEN,
        V3A_CELL_SIZE,
        V3A_DISPLACEMENT,
        &ecoflex_field(),
    );
    assert_eq!(bc.pinned_vertices.len(), 9);
    assert!(bc.loaded_vertices.is_empty());
}

#[test]
fn compressive_block_initial_state_matches_rest_positions() {
    let (mesh, _bc, initial, _contact) = SoftScene::compressive_block_on_plane(
        V3A_EDGE_LEN,
        V3A_CELL_SIZE,
        V3A_DISPLACEMENT,
        &ecoflex_field(),
    );
    let SceneInitial { x_prev, v_prev } = initial;
    let n_dof = 3 * mesh.n_vertices();
    assert_eq!(x_prev.shape(), &[n_dof]);
    assert_eq!(v_prev.shape(), &[n_dof]);
    let x_data = x_prev.as_slice();
    for (v, pos) in mesh.positions().iter().enumerate() {
        assert_relative_eq!(x_data[3 * v], pos.x, epsilon = 1e-15);
        assert_relative_eq!(x_data[3 * v + 1], pos.y, epsilon = 1e-15);
        assert_relative_eq!(x_data[3 * v + 2], pos.z, epsilon = 1e-15);
    }
    // Velocity zero at rest.
    assert!(v_prev.as_slice().iter().all(|&v| v == 0.0));
}

#[test]
fn compressive_block_plane_penetrates_top_face_by_displacement() {
    // Helper docstring convention: at rest config, every top-face
    // vertex (z = edge_len) has signed_distance = -displacement
    // (penetrated by exactly `displacement`). Dispatch through the
    // ContactModel's gradient to verify the plane's orientation +
    // offset round-trip via PenaltyRigidContact's RigidPrimitive
    // dispatch.
    let (mesh, _bc, _initial, contact) = SoftScene::compressive_block_on_plane(
        V3A_EDGE_LEN,
        V3A_CELL_SIZE,
        V3A_DISPLACEMENT,
        &ecoflex_field(),
    );
    let positions: Vec<Vec3> = mesh.positions().to_vec();
    let pairs = contact.active_pairs(&mesh, &positions);
    // Every top-face vertex (n+1)² = 9 should be active (penetrated by
    // displacement = 5e-4 m, well inside d̂ = 1 mm default).
    let top_face_count = mesh
        .positions()
        .iter()
        .filter(|p| (p.z - V3A_EDGE_LEN).abs() < 0.5 * V3A_CELL_SIZE)
        .count();
    assert_eq!(top_face_count, 9);
    assert_eq!(pairs.len(), top_face_count);
}

#[test]
#[should_panic(expected = "must divide edge_len")]
fn compressive_block_panics_on_non_dividing_cell_size() {
    let _scene = SoftScene::compressive_block_on_plane(0.01, 0.003, 5.0e-4, &ecoflex_field());
}

#[test]
#[should_panic(expected = "must be finite and positive")]
fn compressive_block_panics_on_zero_edge_len() {
    let _scene = SoftScene::compressive_block_on_plane(0.0, 0.005, 5.0e-4, &ecoflex_field());
}

// ── sphere_on_plane (V-3 scaffold) ────────────────────────────────────────

const V3_RADIUS: f64 = 0.01;
const V3_CELL_SIZE: f64 = 0.005;
const V3_FORCE: f64 = 0.1;

#[test]
fn sphere_on_plane_top_band_is_nonempty_and_loaded_axis_z() {
    let (_mesh, bc, _initial, _contact, theta) =
        SoftScene::sphere_on_plane(V3_RADIUS, V3_CELL_SIZE, V3_FORCE, ecoflex_field())
            .expect("sphere_on_plane should mesh at canonical V-3 cell size");
    // Four-pin equator set: deduplicated cardinal-direction equator
    // vertices remove all 6 rigid-body modes (see scene.rs
    // sphere_on_plane "Boundary conditions" docstring section). At
    // generic BCC resolutions all four cardinal targets land on
    // distinct vertices; we assert at least 3 distinct pins to allow
    // for degenerate-resolution collapse where two cardinal targets
    // share a vertex.
    assert!(
        bc.pinned_vertices.len() >= 3 && bc.pinned_vertices.len() <= 4,
        "equator pin count {} outside expected [3, 4] range",
        bc.pinned_vertices.len(),
    );
    assert!(!bc.loaded_vertices.is_empty());
    for &(_, axis) in &bc.loaded_vertices {
        assert_eq!(axis, LoadAxis::AxisZ);
    }
    // AxisZ convention: theta is length-1 broadcast magnitude
    // (backward_euler.rs:807-817), not a per-vertex tensor.
    assert_eq!(theta.shape(), &[1]);
}

#[test]
fn sphere_on_plane_theta_broadcast_magnitude_sums_to_negative_force() {
    // AxisZ broadcast: theta is a length-1 tensor carrying
    // `-force / N_loaded`; the solver applies this scalar to every
    // loaded vertex's z-DOF (backward_euler.rs:807-817), so the total
    // applied force is `theta[0] * N_loaded = -force`.
    let (_mesh, bc, _initial, _contact, theta) =
        SoftScene::sphere_on_plane(V3_RADIUS, V3_CELL_SIZE, V3_FORCE, ecoflex_field())
            .expect("sphere_on_plane should mesh at canonical V-3 cell size");
    let theta_slice = theta.as_slice();
    assert_eq!(theta_slice.len(), 1);
    let mag = theta_slice[0];
    let n_loaded = bc.loaded_vertices.len();
    assert!(n_loaded > 0);
    // `n_loaded` is a small mesh-vertex count; cast is loss-free.
    #[allow(clippy::cast_precision_loss)]
    let total = mag * n_loaded as f64;
    assert_relative_eq!(total, -V3_FORCE, epsilon = 1e-12);
    // Sanity: per-vertex magnitude matches the documented form.
    #[allow(clippy::cast_precision_loss)]
    let expected_mag = -V3_FORCE / n_loaded as f64;
    assert_relative_eq!(mag, expected_mag, epsilon = 1e-15);
}

#[test]
fn sphere_on_plane_loaded_band_absent_from_active_pairs_at_rest() {
    // Helper docstring convention: at rest config the loaded
    // top-of-sphere band sits FAR from the plane (the plane lives at
    // `z = -(radius + d̂)`). No loaded vertex should appear in the
    // active set at rest — guards against future edits that
    // accidentally invert the plane normal or place it above the
    // sphere. Orphan BCC lattice vertices in the bbox below the
    // plane and any south-pole-region referenced vertices that fall
    // within `d̂` due to FP roundoff at the band edge are tolerated;
    // the solver auto-pins orphans, and band-edge active pairs
    // contribute negligible force (`κ · 0 · n` to numerical noise).
    // The full plane-orientation convention is load-tested by V-3
    // (commit 9) via Hertzian closed-form comparison.
    use std::collections::BTreeSet;

    use sim_soft::ContactPair;

    let (mesh, bc, _initial, contact, _theta) =
        SoftScene::sphere_on_plane(V3_RADIUS, V3_CELL_SIZE, V3_FORCE, ecoflex_field())
            .expect("sphere_on_plane should mesh at canonical V-3 cell size");
    let positions: Vec<Vec3> = mesh.positions().to_vec();
    let pairs = contact.active_pairs(&mesh, &positions);

    let loaded_set: BTreeSet<u32> = bc.loaded_vertices.iter().map(|&(v, _)| v).collect();
    for pair in &pairs {
        let ContactPair::Vertex { vertex_id, .. } = *pair;
        assert!(
            !loaded_set.contains(&vertex_id),
            "loaded top-of-sphere vertex {vertex_id} (rest position {:?}) should not be in \
             active pairs at rest config — plane convention violation",
            positions[vertex_id as usize],
        );
    }
}

#[test]
#[should_panic(expected = "radius must be finite and positive")]
fn sphere_on_plane_panics_on_zero_radius() {
    let _scene = SoftScene::sphere_on_plane(0.0, V3_CELL_SIZE, V3_FORCE, ecoflex_field());
}

// ── dropping_sphere (V-5 scaffold) ────────────────────────────────────────

const V5_RADIUS: f64 = 0.01;
const V5_CELL_SIZE: f64 = 0.005;
const V5_RELEASE_HEIGHT: f64 = 0.05;

#[test]
fn dropping_sphere_has_no_pinned_or_loaded_vertices() {
    let (_mesh, bc, _initial, _contact) =
        SoftScene::dropping_sphere(V5_RADIUS, V5_CELL_SIZE, V5_RELEASE_HEIGHT, ecoflex_field())
            .expect("dropping_sphere should mesh at canonical V-5 cell size");
    assert!(bc.pinned_vertices.is_empty());
    assert!(bc.loaded_vertices.is_empty());
}

#[test]
fn dropping_sphere_initial_z_is_rest_z_plus_release_height() {
    let (mesh, _bc, initial, _contact) =
        SoftScene::dropping_sphere(V5_RADIUS, V5_CELL_SIZE, V5_RELEASE_HEIGHT, ecoflex_field())
            .expect("dropping_sphere should mesh at canonical V-5 cell size");
    let SceneInitial { x_prev, v_prev } = initial;
    let n_dof = 3 * mesh.n_vertices();
    assert_eq!(x_prev.shape(), &[n_dof]);
    assert_eq!(v_prev.shape(), &[n_dof]);
    let x_data = x_prev.as_slice();
    for (v, pos) in mesh.positions().iter().enumerate() {
        assert_relative_eq!(x_data[3 * v], pos.x, epsilon = 1e-15);
        assert_relative_eq!(x_data[3 * v + 1], pos.y, epsilon = 1e-15);
        assert_relative_eq!(
            x_data[3 * v + 2],
            pos.z + V5_RELEASE_HEIGHT,
            epsilon = 1e-15,
        );
    }
}

#[test]
fn dropping_sphere_starts_clear_of_contact_band() {
    // Sphere released at z = release_height, plane at z = 0; lowest
    // vertex sits at z = release_height - radius >> d̂ at the
    // canonical V-5 sizes. Active-pair walk on x_prev should yield
    // zero pairs.
    let (mesh, _bc, initial, contact) =
        SoftScene::dropping_sphere(V5_RADIUS, V5_CELL_SIZE, V5_RELEASE_HEIGHT, ecoflex_field())
            .expect("dropping_sphere should mesh at canonical V-5 cell size");
    let x_data = initial.x_prev.as_slice();
    let n_vertices = mesh.n_vertices();
    let positions: Vec<Vec3> = (0..n_vertices)
        .map(|v| Vec3::new(x_data[3 * v], x_data[3 * v + 1], x_data[3 * v + 2]))
        .collect();
    let pairs = contact.active_pairs(&mesh, &positions);
    assert_eq!(
        pairs.len(),
        0,
        "dropping_sphere initial config should have zero active pairs (sphere clear of \
         contact band), got {} pairs",
        pairs.len(),
    );
}

#[test]
#[should_panic(expected = "must be finite and strictly greater than radius + d̂")]
fn dropping_sphere_panics_on_release_height_in_contact_band() {
    let _scene = SoftScene::dropping_sphere(V5_RADIUS, V5_CELL_SIZE, V5_RADIUS, ecoflex_field());
}

// ── PenaltyRigidContactSolver type alias smoke test ───────────────────────

#[test]
fn penalty_rigid_contact_solver_alias_resolves_for_handbuilt_mesh() {
    // Smoke test: the type alias added to lib.rs at commit 6
    // monomorphizes correctly when supplied a HandBuiltTetMesh from
    // `compressive_block_on_plane` and the helper-built
    // PenaltyRigidContact. Constructs the solver but does NOT invoke
    // `step` / `replay_step` — purely a "the alias resolves and
    // CpuNewtonSolver::new accepts its inputs" gate. The full
    // contact-dispatch hot-path exercise lives at V-1 commit 7.
    use sim_soft::{
        CpuNewtonSolver, HandBuiltTetMesh, PenaltyRigidContactSolver, SolverConfig, Tet4,
    };

    let (mesh, bc, _initial, contact) =
        SoftScene::compressive_block_on_plane(0.01, 0.005, 0.0, &ecoflex_field());
    let _solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, SolverConfig::skeleton(), bc);
}

#[test]
fn compressive_block_yields_consistent_pinned_set_with_external_predicate() {
    // Helper's predicate-based bottom-face selection should match an
    // external manual sweep over the same band tolerance — guards
    // against future BC-construction edits silently dropping
    // off-by-one vertices on the z = 0 face.
    let (mesh, bc, _initial, _contact) = SoftScene::compressive_block_on_plane(
        V3A_EDGE_LEN,
        V3A_CELL_SIZE,
        V3A_DISPLACEMENT,
        &ecoflex_field(),
    );
    let band_tol = 0.5 * V3A_CELL_SIZE;
    let manual: std::collections::BTreeSet<u32> = mesh
        .positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.z.abs() < band_tol)
        .map(|(i, _)| {
            // Cast safe: V-3a meshes have ≤ 729 vertices at finest
            // cell size (n=8 → 9³).
            #[allow(clippy::cast_possible_truncation)]
            let v = i as u32;
            v
        })
        .collect();
    let observed: std::collections::BTreeSet<u32> = bc.pinned_vertices.iter().copied().collect();
    assert_eq!(manual, observed);
}
