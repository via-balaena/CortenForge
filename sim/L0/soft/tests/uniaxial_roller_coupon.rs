//! M2-S2 — roller free-lateral uniaxial coupon (the genuine free-lateral
//! solve that supersedes M1's constant-strain patch test).
//!
//! M1's `uniaxial_fem_coupon.rs` could only run a *constant-strain patch
//! test*: it imposed the full analytical `F = diag(λ, λ_t, λ_t)` on the
//! boundary (every boundary DOF pinned to the affine field), because the
//! Phase-2 BC surface was full-3-DOF-pin only — there was no way to leave a
//! face free to contract. M2-S1 added **roller / per-axis Dirichlet BCs**,
//! so this coupon does the real thing: it drives the axial faces, leaves the
//! lateral faces **free**, and lets the solver **independently re-discover**
//! the transverse stretch `λ_t`. `λ_t` is *solved*, not prescribed.
//!
//! The cell is a `1/8`-symmetry block: the three minimum faces are roller
//! (symmetry) planes — `x=0`: `u_x=0`, `y=0`: `u_y=0`, `z=0`: `u_z=0` —
//! which kill the six rigid-body modes with minimal constraint; the `x=L`
//! face is driven to `u_x = (λ−1)L`; and the `y=L` / `z=L` faces carry NO
//! constraint, so they contract freely. This removes exactly the rigid-body
//! modes (the tangent is non-singular) without the over-constraint that
//! would reintroduce volumetric locking (M1 risk R5).
//!
//! Runs on the **measured** material — the Path-3
//! [`ECOFLEX_00_30_MEASURED`](sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED)
//! `Yeoh` — through the real `CpuTet4YeohSolver`, so the coupon's stress is
//! graded against both the analytical free-transverse primitive (solver
//! fidelity) AND Marechal's measured curve (model fidelity). Two claims:
//!
//! 1. **Solver fidelity** — the FEM independently recovers `λ_t` and the
//!    axial Cauchy stress matching [`free_transverse_uniaxial`] to mesh
//!    tolerance, over the device window. The lateral faces are free, so this
//!    is a true free-transverse solve, not a prescribed-`λ_t` patch test.
//! 2. **Model fidelity** — the measured `Yeoh` reproduces Marechal's
//!    measured true-stress–stretch curve to ≤ 10 % RMS over λ ≤ 2 (the M1
//!    gate, re-asserted here on the same vendored asset). Chained with (1):
//!    the coupon reproduces *measurement* to that residual.
//!
//! The solver-driven stretches stop at λ = 1.9 (max stretch deviation 0.9 <
//! the `Yeoh` validity ceiling of 1.0); the model-fidelity RMS is analytical
//! and covers the full λ ≤ 2 window.
//!
//! See `docs/soft_fidelity/03_phases/m2_roller_bc_and_modes/recon.md`.

#![allow(
    // Vertex/DOF index arithmetic and the per-axis cell counts sit far
    // inside f64/u32 exact range — the standard Mesh-trait API tax.
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    // Local engineering scalars (λ, λ_t, F components) read clearer inline.
    clippy::many_single_char_names,
    // The coupon test threads BC setup, warm-start, solve, and the multi-claim
    // assert block in one readable sequence; splitting it would obscure the flow.
    clippy::too_many_lines,
    // `.expect()`/`.unwrap()` surface a malformed fixture or missing asset as
    // a test panic — the canonical fixture idiom in this crate's tests.
    clippy::expect_used,
    clippy::unwrap_used
)]

use nalgebra::Matrix3;
use sim_ml_chassis::Tensor;
use sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4YeohSolver, HandBuiltTetMesh, Material,
    MaterialField, Mesh, MeshAdjacency, NullContact, QualityMetrics, Solver, SolverConfig, Tet4,
    Vec3, VertexId, Yeoh, free_transverse_uniaxial,
};

const L: f64 = 0.1; // decimeter cube edge (crate-canonical scale)
const N: usize = 2; // cells per axis (even per cantilever_bilayer_beam); 3³ nodes w/ interior
// Large dt makes the inertial term M/dt² negligible vs stiffness, so a single
// backward-Euler `replay_step` reaches static equilibrium (mirrors the M1
// patch test + IV-3 `bonded_bilayer_beam`).
const STATIC_DT: f64 = 1.0e3;

// Solver-driven device window: λ ≤ 1.9 keeps max stretch deviation (0.9) under
// the Yeoh validity ceiling (1.0). Model-fidelity RMS below covers full λ ≤ 2.
const DRIVEN_STRETCHES: [f64; 5] = [1.1, 1.25, 1.5, 1.75, 1.9];

/// A `Mesh<Yeoh>` over a hand-built tet box. Topology, adjacency, quality,
/// and boundary faces are borrowed from an inner `HandBuiltTetMesh` (the
/// validated box generator); only `materials()` is overridden to carry the
/// per-tet measured `Yeoh`. The solver reads only topology + materials, so
/// the borrowed aux surfaces are never touched on the hot path — they are
/// delegated purely to satisfy the trait.
struct YeohBox {
    inner: HandBuiltTetMesh,
    materials: Vec<Yeoh>,
}

impl YeohBox {
    fn uniform(n: usize, l: f64, yeoh: &Yeoh) -> Self {
        // The MaterialField only seeds the inner NH cache (unused); the Yeoh
        // cache below is what `materials()` returns. Lamé values kept
        // consistent for clarity.
        let inner = HandBuiltTetMesh::cantilever_bilayer_beam(
            n,
            n,
            n,
            l,
            l,
            l,
            &MaterialField::uniform(yeoh.mu(), yeoh.lambda()),
        );
        let materials = vec![yeoh.clone(); inner.n_tets()];
        Self { inner, materials }
    }
}

impl Mesh<Yeoh> for YeohBox {
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
    // Diagnostic surfaces — never read by the Newton hot path; delegated to
    // the inner mesh purely to satisfy the trait.
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
    fn equals_structurally(&self, other: &dyn Mesh<Yeoh>) -> bool {
        self.n_vertices() == other.n_vertices()
            && self.n_tets() == other.n_tets()
            && (0..self.n_tets() as u32).all(|t| self.tet_vertices(t) == other.tet_vertices(t))
    }
}

/// A rest coordinate sits on a min/max face within tolerance.
fn at(coord: f64, value: f64) -> bool {
    (coord - value).abs() < 1.0e-9
}

#[test]
fn roller_coupon_rediscovers_lateral_stretch_and_matches_measured() {
    let yeoh: Yeoh = ECOFLEX_00_30_MEASURED.to_yeoh();

    for &lam in &DRIVEN_STRETCHES {
        let analytic = free_transverse_uniaxial(&yeoh, lam);
        let lat = analytic.transverse_stretch;
        let affine = |p: Vec3| Vec3::new(p.x * lam, p.y * lat, p.z * lat);

        let mesh = YeohBox::uniform(N, L, &yeoh);
        let rest: Vec<Vec3> = mesh.positions().to_vec();
        let n = mesh.n_vertices();
        let tets: Vec<[VertexId; 4]> = (0..mesh.n_tets())
            .map(|t| mesh.tet_vertices(t as u32))
            .collect();

        // BCs: rollers on the three min faces (symmetry planes) + the driven
        // +x face; the +y / +z faces carry NO constraint (free lateral). A
        // node on a shared edge OR-combines the masks (e.g. the origin gets
        // [t,t,t], a full pin). Free interior + lateral DOFs are solved.
        let mut rollers: Vec<(VertexId, [bool; 3])> = Vec::new();
        for (v, &p) in rest.iter().enumerate() {
            let mask = [
                at(p.x, 0.0) || at(p.x, L), // axial faces: x=0 (symmetry) + x=L (driven)
                at(p.y, 0.0),               // y=0 symmetry plane
                at(p.z, 0.0),               // z=0 symmetry plane
            ];
            if mask.iter().any(|&b| b) {
                rollers.push((v as VertexId, mask));
            }
        }

        // Warm-start at the affine field, then PERTURB the free DOFs so the
        // solver must actively drive them back — a non-vacuous re-discovery.
        // The constrained DOFs equal their affine values exactly (x=0→0,
        // x=L→λL, y=0→0, z=0→0), so the roller holds each at the right place;
        // the free +y/+z face and interior DOFs are the unknowns that must
        // converge onto λ_t.
        let perturb = |v: usize| {
            let f = |k: usize| (k % 7) as f64 - 3.0; // deterministic, in [−3, 3]
            1.0e-4 * Vec3::new(f(v), f(v + 1), f(v + 2))
        };
        let constrained: Vec<[bool; 3]> = {
            let mut c = vec![[false; 3]; n];
            for &(v, m) in &rollers {
                c[v as usize] = m;
            }
            c
        };
        let mut x_prev = vec![0.0_f64; 3 * n];
        for (v, &p) in rest.iter().enumerate() {
            let base = affine(p);
            let pert = perturb(v);
            for ax in 0..3 {
                // perturb only free DOFs; constrained DOFs stay exact.
                let val = base[ax] + if constrained[v][ax] { 0.0 } else { pert[ax] };
                x_prev[3 * v + ax] = val;
            }
        }

        let bc = BoundaryConditions {
            pinned_vertices: vec![],
            roller_vertices: rollers,
            loaded_vertices: vec![],
        };
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = STATIC_DT;
        let solver: CpuTet4YeohSolver<YeohBox> =
            CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

        let step = solver.replay_step(
            &Tensor::from_slice(&x_prev, &[3 * n]),
            &Tensor::zeros(&[3 * n]),
            &Tensor::zeros(&[0]),
            cfg.dt,
        );
        assert!(
            step.final_residual_norm < cfg.tol,
            "λ={lam}: Newton did not converge — residual {:.2e} ≥ tol {:.2e}",
            step.final_residual_norm,
            cfg.tol
        );
        let xf = step.x_final;

        // (1) RE-DISCOVERY: every node — including the FREE +y/+z lateral
        // faces and the interior — lands on the homogeneous affine field with
        // the analytical λ_t. The lateral faces were unconstrained, so their
        // contraction is solved, not imposed.
        let mut max_dev = 0.0_f64;
        for (v, &p) in rest.iter().enumerate() {
            let want = affine(p);
            let got = Vec3::new(xf[3 * v], xf[3 * v + 1], xf[3 * v + 2]);
            max_dev = max_dev.max((got - want).norm());
        }
        assert!(
            max_dev < 1.0e-6,
            "λ={lam}: node deviates {max_dev:.2e} m from the re-discovered homogeneous field"
        );

        // Direct λ_t read-off from a FREE +y-face node (rest y = L): its
        // solved y / L is the re-discovered transverse stretch.
        let yface = rest
            .iter()
            .position(|p| at(p.y, L) && !at(p.x, 0.0) && !at(p.x, L))
            .expect("a +y-face node off the axial faces");
        let lat_recovered = xf[3 * yface + 1] / L;
        assert!(
            (lat_recovered - lat).abs() < 1.0e-6,
            "λ={lam}: re-discovered λ_t {lat_recovered:.6} vs analytic {lat:.6}"
        );

        // (2) STRESS: an element's recovered F ≡ diag(λ, λ_t, λ_t), its axial
        // Cauchy stress ≡ the analytical value, and its lateral traction ~0.
        let f = deformation_gradient(&rest, &xf, tets[0]);
        let f_expected = Matrix3::from_diagonal(&Vec3::new(lam, lat, lat));
        assert!(
            (f - f_expected).abs().max() < 1.0e-6,
            "λ={lam}: recovered F off homogeneous by {:.2e}",
            (f - f_expected).abs().max()
        );
        let p = yeoh.first_piola(&f);
        let j = f.determinant();
        let axial_cauchy = p[(0, 0)] * f[(0, 0)] / j;
        let analytic_sigma = analytic.cauchy_stress;
        assert!(
            (axial_cauchy - analytic_sigma).abs() / analytic_sigma < 1.0e-6,
            "λ={lam}: FEM axial Cauchy {axial_cauchy} vs analytical {analytic_sigma}"
        );
        let axial = p[(0, 0)].abs().max(1.0);
        assert!(
            p[(1, 1)].abs() / axial < 1.0e-6,
            "λ={lam}: lateral traction not free"
        );

        // Report the direct FEM-vs-measurement deviation at the nearest
        // measured data point, for transparency.
        let (m_lam, m_sig) = nearest_measured(lam);
        eprintln!(
            "λ={lam:.2}: λ_t re-discovered {lat_recovered:.4} (analytic {lat:.4}); \
             FEM σ {axial_cauchy:.0} Pa vs nearest measured (λ={m_lam:.3}) {m_sig:.0} Pa"
        );
    }
}

/// Model fidelity: the measured `Yeoh` reproduces Marechal's measured curve
/// to ≤ 10 % RMS over λ ≤ 2 — the M1 gate, re-asserted on the same asset, so
/// chained with the solver-fidelity test above the coupon reproduces
/// measurement to this residual.
#[test]
fn measured_yeoh_matches_marechal_curve() {
    let yeoh = ECOFLEX_00_30_MEASURED.to_yeoh();
    let curve = measured_curve();
    let (lo, hi) = (1.1, 2.0);
    let mut sse = 0.0;
    let mut count: usize = 0;
    for &(lam, sig) in &curve {
        if lam < lo || lam > hi {
            continue;
        }
        let model = free_transverse_uniaxial(&yeoh, lam).cauchy_stress;
        let re = (model - sig) / sig.abs().max(1.0);
        sse += re * re;
        count += 1;
    }
    assert!(count > 0, "no measured points in [{lo}, {hi}]");
    let rms = (sse / count as f64).sqrt();
    eprintln!(
        "measured Yeoh vs Marechal curve: RMS {:.1}% over λ≤2 (n={count})",
        rms * 100.0
    );
    assert!(
        rms <= 0.10,
        "measured-curve RMS {:.1}% exceeds 10% gate",
        rms * 100.0
    );
}

const ASSET: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/assets/marechal_2021/ecoflex_00_30_uniaxial.json"
);

fn measured_curve() -> Vec<(f64, f64)> {
    let text = std::fs::read_to_string(ASSET).expect("read vendored Marechal asset");
    let v: serde_json::Value = serde_json::from_str(&text).expect("parse asset JSON");
    v["measured"]
        .as_array()
        .expect("measured array")
        .iter()
        .map(|p| {
            (
                p["stretch"].as_f64().unwrap(),
                p["cauchy_stress_pa"].as_f64().unwrap(),
            )
        })
        .collect()
}

/// Nearest measured (stretch, cauchy) data point to `lam` — for reporting the
/// direct FEM-vs-measurement deviation.
fn nearest_measured(lam: f64) -> (f64, f64) {
    measured_curve()
        .into_iter()
        .min_by(|a, b| {
            (a.0 - lam)
                .abs()
                .partial_cmp(&(b.0 - lam).abs())
                .expect("finite stretches")
        })
        .expect("non-empty curve")
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
