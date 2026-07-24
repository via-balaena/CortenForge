//! Tet4 byte-identity goldens for the Tet10 rung-4 multi-Gauss-point refactor.
//!
//! The pre-existing `to_bits()` goldens (`contact_passthrough`,
//! `invariant_iv_1_uniform_passthrough`) are **Neo-Hookean only**, so the
//! Yeoh material path, the roller (per-axis Dirichlet) free-DOF map, and the
//! Lamé/first-Piola kernels are tolerance-only and would MISS the ULP drift a
//! reordered assembly loop introduces. Ladder rung 4
//! (`docs/SIM_SOFT_TET10_PLAN.md` §3.1) makes the six assembly kernels generic
//! over `(N, G)` with Tet4 as the `(4, 1)` monomorphization; Tet4 must stay
//! **bit-identical**. This file extends the golden bit-set to a **Yeoh** scene
//! and a **roller** scene so that drift is caught, not just relaxed under a
//! tolerance.
//!
//! Both scenes are small, dense (5-vertex `two_tet_shared_face`) FEM solves —
//! the tier where bit-equality is achievable across rustc/LLVM minor versions
//! and macOS-arm64/Linux-x86_64 SIMD (per the `invariant_iv_1` two-tier
//! contract: dense small-FEM uses explicit `nalgebra::Matrix3` scalar ops; only
//! the sparse-solver-at-scale path drifts at the last bits).
//!
//! **Reference capture provenance.** Bit-patterns below were captured on the
//! pre-rung-4 tip of branch `tet10-rung4-multigp-stiffness` (== `main`
//! `053417c8`) via the `#[ignore]` capture harness at the bottom of this file
//! (`cargo test -p sim-soft --test tet4_byte_identity_extended -- --ignored
//! --nocapture`), rustc/host as recorded in the memory checkpoint. The rung-4
//! refactor must reproduce them exactly.
//!
//! **Failure protocol.** If a test fails, do NOT re-capture. First rule out a
//! real rung-4 byte-identity regression (a reordered Tet4 accumulation — most
//! likely the weight literal `det.abs()/6.0` reconstructed as `(1/6)·detJ`, or
//! the edge-vector Jacobian routed through the Tet10 Σ-form) and a toolchain
//! delta. Re-capture is sanctioned ONLY for a documented toolchain change.

#![allow(
    // Bit-equality assertions on f64 are the entire point of these goldens.
    clippy::float_cmp,
    clippy::expect_used
)]

use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, CpuTet4YeohSolver, HandBuiltTetMesh,
    LoadAxis, MaterialField, Mesh, MeshAdjacency, NullContact, QualityMetrics, Solver,
    SolverConfig, Tet4, Vec3, VertexId, Yeoh,
};

// ── Shared scene constants ───────────────────────────────────────────────

/// Load magnitude for the Yeoh scene. Small — a soft silicone Yeoh (μ ≈ few
/// kPa) under this `+ẑ` traction on the 0.1-scale tet stays well inside the
/// validity band and converges in a handful of Newton iters.
const THETA_YEOH: f64 = 2.0;

/// Load magnitude for the roller scene (Neo-Hookean, Ecoflex-class Lamé).
const THETA_ROLLER: f64 = 10.0;
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

// ── A Mesh<Yeoh> wrapper over an arbitrary HandBuiltTetMesh ───────────────

/// Borrow topology / aux surfaces from an inner [`HandBuiltTetMesh`] and
/// override `materials()` with a uniform per-tet [`Yeoh`]. The Newton hot path
/// reads only topology + materials, so the delegated diagnostic surfaces are
/// never touched. (Mirrors the `YeohBox` wrapper in `uniaxial_roller_coupon`.)
struct YeohMesh {
    inner: HandBuiltTetMesh,
    materials: Vec<Yeoh>,
}

impl YeohMesh {
    fn over(inner: HandBuiltTetMesh, yeoh: &Yeoh) -> Self {
        let materials = vec![yeoh.clone(); inner.n_tets()];
        Self { inner, materials }
    }
}

impl Mesh<Yeoh> for YeohMesh {
    fn n_tets(&self) -> usize {
        self.inner.n_tets()
    }
    fn n_vertices(&self) -> usize {
        self.inner.n_vertices()
    }
    fn tet_vertices(&self, tet: u32) -> [VertexId; 4] {
        self.inner.tet_vertices(tet)
    }
    fn positions(&self) -> &[Vec3] {
        self.inner.positions()
    }
    fn materials(&self) -> &[Yeoh] {
        &self.materials
    }
    fn adjacency(&self) -> &MeshAdjacency {
        self.inner.adjacency()
    }
    fn quality(&self) -> &QualityMetrics {
        self.inner.quality()
    }
    fn interface_flags(&self) -> &[bool] {
        self.inner.interface_flags()
    }
    fn boundary_faces(&self) -> &[[VertexId; 3]] {
        self.inner.boundary_faces()
    }
    // `n_tets() as u32`: the Mesh-trait API tax — `n_tets` returns `usize`,
    // `tet_vertices` takes `u32`; meshes stay far below `u32::MAX`.
    #[allow(clippy::cast_possible_truncation)]
    fn equals_structurally(&self, other: &dyn Mesh<Yeoh>) -> bool {
        self.n_vertices() == other.n_vertices()
            && self.n_tets() == other.n_tets()
            && (0..self.n_tets() as u32).all(|t| self.tet_vertices(t) == other.tet_vertices(t))
    }
}

// ── Scene runners ─────────────────────────────────────────────────────────

/// Yeoh `two_tet_shared_face`: pin 0/1/2/4, load vertex 3 in `+ẑ`, one static
/// `replay_step`. 5 vertices × 3 = 15 DOFs. Exercises `Yeoh::first_piola` /
/// `Yeoh::tangent` through the (rung-4-generic) assembly.
fn run_yeoh_shared_face() -> Vec<f64> {
    let yeoh: Yeoh = ECOFLEX_00_30_MEASURED.to_yeoh();
    let inner =
        HandBuiltTetMesh::two_tet_shared_face(&MaterialField::uniform(yeoh.mu(), yeoh.lambda()));
    let mesh = YeohMesh::over(inner, &yeoh);
    let cfg = SolverConfig::skeleton();
    let n_dof = 3 * mesh.n_vertices();
    let positions = mesh.positions().to_vec();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 4],
        roller_vertices: Vec::new(),
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    };
    let solver: CpuTet4YeohSolver<YeohMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let theta = Tensor::from_slice(&[THETA_YEOH], &[1]);
    solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt).x_final
}

/// Neo-Hookean `two_tet_shared_face` with a **roller** on vertex 4: pin 0/1/2
/// fully, roller-pin v4's x-axis only (`[true, false, false]` → y/z free), load
/// vertex 3 in `+ẑ`, one static `replay_step`. 15 DOFs, of which v4's y/z + v3's
/// xyz are free. Exercises the per-axis free-DOF map through the assembly.
fn run_roller_shared_face() -> Vec<f64> {
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&MaterialField::uniform(MU, LAMBDA));
    let cfg = SolverConfig::skeleton();
    let n_dof = 3 * mesh.n_vertices();
    let positions = mesh.positions().to_vec();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: vec![(4, [true, false, false])],
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    };
    let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let theta = Tensor::from_slice(&[THETA_ROLLER], &[1]);
    solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt).x_final
}

// ── Frozen pre-rung-4 reference bit-patterns ─────────────────────────────

const YEOH_SHARED_FACE_X_FINAL: [u64; 15] = [
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x3f1a_ccf1_4d3c_e3e8,
    0x3f1a_ccf1_4d3c_e457,
    0x3fb9_c1f6_b61c_debd,
    0x3fb4_7ae1_47ae_147b,
    0x3fb4_7ae1_47ae_147b,
    0x3fb4_7ae1_47ae_147b,
];
const ROLLER_SHARED_FACE_X_FINAL: [u64; 15] = [
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x3f28_f21b_a700_5128,
    0x3f2e_b781_9806_b1d8,
    0x3fb9_ca89_2c81_8394,
    0x3fb4_7ae1_47ae_147b,
    0x3fb4_8626_0c38_e0c7,
    0x3fb4_7135_fadd_f8c2,
];

fn assert_bit_equal(actual: &[f64], expected: &[u64], scene: &str) {
    assert_eq!(actual.len(), expected.len(), "{scene}: length drift");
    for (i, (val, &exp)) in actual.iter().zip(expected).enumerate() {
        let got = val.to_bits();
        assert_eq!(
            got,
            exp,
            "{scene}: x_final[{i}] bit drift — got {got:#018x} ({val:e}), expected \
             {exp:#018x} ({:e}). Rung-4 Tet4 byte-identity broke: check the \
             `det.abs()/6.0` weight literal and the edge-vector Jacobian branch \
             (SIM_SOFT_TET10_PLAN §3.1). NEVER re-bake to green.",
            f64::from_bits(exp),
        );
    }
}

#[test]
fn yeoh_shared_face_bit_identical() {
    assert_bit_equal(
        &run_yeoh_shared_face(),
        &YEOH_SHARED_FACE_X_FINAL,
        "yeoh_shared_face",
    );
}

#[test]
fn roller_shared_face_bit_identical() {
    assert_bit_equal(
        &run_roller_shared_face(),
        &ROLLER_SHARED_FACE_X_FINAL,
        "roller_shared_face",
    );
}

// ── Capture harness (run once on the pre-rung-4 tip to freeze the bits) ───

#[test]
#[ignore = "capture-only: prints golden bit-patterns; not a gate"]
fn capture_reference_bits() {
    let yeoh = run_yeoh_shared_face();
    let roller = run_roller_shared_face();
    let fmt = |v: &[f64]| {
        v.iter()
            .map(|x| format!("    {:#018x},", x.to_bits()))
            .collect::<Vec<_>>()
            .join("\n")
    };
    eprintln!("YEOH_SHARED_FACE_X_FINAL:\n{}", fmt(&yeoh));
    eprintln!("ROLLER_SHARED_FACE_X_FINAL:\n{}", fmt(&roller));
}
