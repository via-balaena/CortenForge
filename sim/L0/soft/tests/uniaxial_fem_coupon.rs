//! M1-S2 — FEM uniaxial coupon ≡ analytical (solver-vs-model separation).
//!
//! A constant-strain **patch test**: impose the analytical homogeneous
//! uniaxial-tension deformation `F = diag(λ, λ_t, λ_t)` (with `λ_t` from
//! [`free_transverse_uniaxial`]) as Dirichlet boundary conditions on a box
//! of `Tet4` elements, start the free interior nodes perturbed off that
//! field, and solve a single static step. A correct FEM solver must (1)
//! drive every free interior node back onto the homogeneous affine field
//! and (2) recover the analytical stress in each element.
//!
//! This separates **solver error from model error**: it confirms the
//! assembly + Dirichlet condensation + Newton solve add *no* error to a
//! homogeneous uniaxial state, so the residual gap in the measured-accuracy
//! gate (`uniaxial_measured_accuracy.rs`) is attributable to the material
//! parameters, not the solver.
//!
//! Runs on **`NeoHookean`** — the material `HandBuiltTetMesh` implements
//! `Mesh` for. The solver's assembly / Dirichlet condensation / Newton path
//! is generic over `M: Material`, so this validates the machinery the
//! `Yeoh` material also flows through; the Yeoh constitutive itself is
//! graded analytically in `uniaxial_measured_accuracy.rs` + the
//! `material::uniaxial` unit tests + `yeoh_contract.rs`.
//!
//! Scope: this imposes the full affine field on the boundary (a constant-
//! strain patch test). Independently *re-discovering* `λ_t` from a
//! traction-driven solve with traction-free lateral faces needs per-axis
//! (roller) Dirichlet BCs, which the Phase-2 BC surface
//! (`pinned_vertices` = full 3-DOF) does not yet expose — a deferred API
//! follow-up (M2). See `docs/soft_fidelity/03_phases/m1_silicone_uniaxial/recon.md`.

#![allow(
    // Vertex/DOF index arithmetic and the small `N`-per-axis cell count
    // sit far inside f64/u32 exact range — the standard Mesh-trait API tax.
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    // Local engineering scalars (λ, λ_t, F components) read more clearly
    // named inline than hoisted.
    clippy::many_single_char_names,
    // `.expect()` surfaces a malformed-fixture invariant (degenerate tet,
    // no interior element) as a test panic — the canonical test idiom.
    clippy::expect_used
)]

use nalgebra::Matrix3;
use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, Material,
    MaterialField, Mesh, NeoHookean, NullContact, Solver, SolverConfig, Tet4, Vec3, VertexId,
    free_transverse_uniaxial,
};

const L: f64 = 0.1; // decimeter cube edge (crate-canonical scale)
const N: usize = 4; // cells per axis (nz must be even); (N+1)³ nodes, (N−1)³ interior
const STATIC_DT: f64 = 1.0e3; // collapses inertia M/dt² → static equilibrium (IV-3 precedent)

// Representative Ecoflex-class compressible Neo-Hookean (ν = 0.40, λ = 4μ).
const MU: f64 = 16_900.0;

fn material() -> NeoHookean {
    NeoHookean::from_lame(MU, 4.0 * MU)
}

fn uniform_field() -> MaterialField {
    MaterialField::uniform(MU, 4.0 * MU)
}

/// A rest position lies on the cube boundary if any coordinate is at 0 or L.
fn on_boundary(p: Vec3) -> bool {
    let e = 1.0e-9;
    p.x.abs() < e
        || (p.x - L).abs() < e
        || p.y.abs() < e
        || (p.y - L).abs() < e
        || p.z.abs() < e
        || (p.z - L).abs() < e
}

#[test]
fn fem_coupon_reproduces_analytical_homogeneous_uniaxial() {
    let mat = material();
    for &lam in &[1.1_f64, 1.25, 1.5] {
        let lat = free_transverse_uniaxial(&mat, lam).transverse_stretch;
        let affine = |p: Vec3| Vec3::new(p.x * lam, p.y * lat, p.z * lat);

        let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(N, N, N, L, L, L, &uniform_field());
        let rest: Vec<Vec3> = mesh.positions().to_vec();
        let n = mesh.n_vertices();
        // Snapshot connectivity before the solver consumes the mesh.
        let tets: Vec<[VertexId; 4]> = (0..mesh.n_tets())
            .map(|t| mesh.tet_vertices(t as u32))
            .collect();

        // Dirichlet on every boundary node (held at the affine-deformed
        // position via x_prev). Free interior nodes start near the affine
        // equilibrium but PERTURBED, so the solver must actively drive them
        // back to it (a non-vacuous patch test) while staying inside the
        // fail-closed validity gate (σ < 2 — a rest start would over-stretch
        // the boundary-adjacent elements past it).
        let perturb = |v: usize| {
            let f = |k: usize| (k % 7) as f64 - 3.0; // deterministic, varied, in [−3, 3]
            1.0e-4 * Vec3::new(f(v), f(v + 1), f(v + 2)) // ≤ 3e-4 m (~1% of a cell)
        };
        let mut x_prev = vec![0.0_f64; 3 * n];
        let mut pinned: Vec<VertexId> = Vec::new();
        for (v, &p) in rest.iter().enumerate() {
            let target = if on_boundary(p) {
                pinned.push(v as VertexId);
                affine(p)
            } else {
                affine(p) + perturb(v)
            };
            x_prev[3 * v] = target.x;
            x_prev[3 * v + 1] = target.y;
            x_prev[3 * v + 2] = target.z;
        }
        assert!(pinned.len() < n, "expected free interior nodes at N={N}");

        let bc = BoundaryConditions {
            pinned_vertices: pinned,
            loaded_vertices: vec![],
        };
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = STATIC_DT;
        let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
            CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

        let step = solver.replay_step(
            &Tensor::from_slice(&x_prev, &[3 * n]),
            &Tensor::zeros(&[3 * n]),
            &Tensor::zeros(&[0]),
            cfg.dt,
        );
        let xf = step.x_final;

        // (1) PATCH TEST: free interior nodes land on the homogeneous affine
        // field. This is the solver-reproduces-homogeneous-deformation claim.
        let mut max_dev = 0.0_f64;
        for (v, &p) in rest.iter().enumerate() {
            if on_boundary(p) {
                continue;
            }
            let want = affine(p);
            let got = Vec3::new(xf[3 * v], xf[3 * v + 1], xf[3 * v + 2]);
            max_dev = max_dev.max((got - want).norm());
        }
        assert!(
            max_dev < 1.0e-6,
            "λ={lam}: interior node deviates {max_dev:.2e} m from the homogeneous affine field"
        );

        // (2) STRESS PIPELINE: an interior-touching element's recovered F
        // matches diag(λ, λ_t, λ_t), its lateral traction is ~0 (the
        // analytical primitive's free-transverse condition), and its axial
        // Cauchy stress equals the analytical value.
        let interior = |v: VertexId| !on_boundary(rest[v as usize]);
        let tet = *tets
            .iter()
            .find(|vs| vs.iter().any(|&v| interior(v)))
            .expect("an element touching an interior node");
        let f = deformation_gradient(&rest, &xf, tet);
        let f_expected = Matrix3::from_diagonal(&Vec3::new(lam, lat, lat));
        assert!(
            (f - f_expected).abs().max() < 1.0e-6,
            "λ={lam}: recovered F off homogeneous by {:.2e}",
            (f - f_expected).abs().max()
        );
        let p = mat.first_piola(&f);
        let j = f.determinant();
        let axial_cauchy = p[(0, 0)] * f[(0, 0)] / j;
        let analytic = free_transverse_uniaxial(&mat, lam).cauchy_stress;
        assert!(
            (axial_cauchy - analytic).abs() / analytic < 1.0e-6,
            "λ={lam}: FEM axial Cauchy {axial_cauchy} vs analytical {analytic}"
        );
        let axial = p[(0, 0)].abs().max(1.0);
        assert!(
            p[(1, 1)].abs() / axial < 1.0e-6,
            "λ={lam}: lateral traction not free"
        );
    }
}

/// Deformation gradient `F = D_s · D_m⁻¹` of one `Tet4` from rest + deformed
/// nodal positions (`xf` is vertex-major xyz).
fn deformation_gradient(rest: &[Vec3], xf: &[f64], tet: [VertexId; 4]) -> Matrix3<f64> {
    let [a, b, c, d] = tet.map(|v| v as usize);
    let xf_of = |i: usize| Vec3::new(xf[3 * i], xf[3 * i + 1], xf[3 * i + 2]);
    let dm = Matrix3::from_columns(&[rest[b] - rest[a], rest[c] - rest[a], rest[d] - rest[a]]);
    let ds = Matrix3::from_columns(&[
        xf_of(b) - xf_of(a),
        xf_of(c) - xf_of(a),
        xf_of(d) - xf_of(a),
    ]);
    ds * dm.try_inverse().expect("non-degenerate rest tet")
}
