//! P2 (6-node) triangular boundary-face primitive for the surface-integrated
//! contact barrier (Tet10 ladder rung 8b).
//!
//! Where [`IpcRigidContact`](super::IpcRigidContact)'s per-vertex path applies
//! the barrier at each mesh vertex as a point energy, this primitive integrates
//! the barrier as an **energy density over the contact face**, weighted by the
//! face's **rest (reference) area** `A_rest`:
//!
//! ```text
//!     E = ∫_Γ_rest b(sd(x)) dA_rest = A_rest · Σ_q ŵ_q · b(sd(x(ξ_q)))
//! ```
//!
//! over the P2 face parametrised by `(u, v)` (barycentric `L0 = 1-u-v`, `L1 = u`,
//! `L2 = v`), with the isoparametric *current* position `x(u,v) = Σ_i N_i(u,v) x_i`
//! sampling the signed distance, and the area-fraction Gauss weights `ŵ_q` summing
//! to 1. Node order is the canonical `[c0, c1, c2, m01, m12, m02]` of
//! [`ContactPair::Face`](super::ContactPair::Face): the three corners followed by
//! the midsides of edges `(c0,c1)`, `(c1,c2)`, `(c0,c2)`.
//!
//! # Why rest area (not deformed area)
//!
//! A non-penetration barrier depends only on the *normal* gap, so it must produce
//! **normal forces only**. Weighting by the *current* (deformed) area couples the
//! barrier to tangential stretch and injects a spurious in-plane surface-tension
//! force — an artifact, not fidelity. The
//! `deformed_area_weighting_injects_a_tangential_force` gate below measures this
//! tangential force at **4.7 %** of the normal force on a flat uniform-pressure
//! face (a configuration-dependent figure, not a universal constant); deformed
//! weighting also loads the corners. The **rest-area** weight `A_rest` is a
//! per-face constant, so the barrier gradient is purely `∝ n̂` and the corner
//! shape functions integrate to exactly zero under uniform pressure
//! (`∫ N_corner dA = 0`, held by `uniform_pressure_is_normal_only_corners_zero`)
//! — the clean consistent P2 load.
//! (Weighting a contact barrier by a *fixed reference* measure rather than the
//! deformed one is the standard way to keep it a pure function of the gap.)
//!
//! # Gradient and Hessian
//!
//! With `n̂ = ∂sd/∂x` (the rigid SDF normal — evaluated, not differentiated: this
//! primitive shares the per-vertex path's documented flat-primitive scope, so the
//! curved-SDF `∂n̂/∂x` Hessian term is deferred to the curved surface, rung 8c),
//! and κ folded into the barrier scalars:
//!
//! ```text
//!     ∂E/∂x_i = A_rest · Σ_q ŵ_q  b'(sd) N_i n̂
//!     H_{ij}  = A_rest · Σ_q ŵ_q  b''(sd) N_i N_j (n̂⊗n̂)
//! ```
//!
//! Both are normal-only; the Hessian block set is the barrier's rank-1
//! `n̂⊗n̂` structure scaled by the PSD shape-function outer product `N_i N_j`, so
//! it is PSD wherever `b'' ≥ 0` (the convex barrier support) — no SPD projection
//! needed, matching the per-vertex path.

use crate::Vec3;
use nalgebra::Matrix3;

/// Barrier scalars and SDF normal at one face Gauss point — the per-point data
/// the caller ([`IpcRigidContact`](super::IpcRigidContact)) supplies from its
/// rigid primitive and κ-scaled barrier, keeping all SDF/κ knowledge on the
/// caller side and all P2 quadrature here.
///
/// All three barrier scalars are already multiplied by the stiffness κ, and are
/// zero for a point outside the barrier support (`sd ≥ d̂`) — an inactive Gauss
/// point contributes nothing, continuously (`b(d̂) = b'(d̂) = b''(d̂) = 0`).
pub(crate) struct FaceBarrierEval {
    /// Outward rigid-primitive normal `n̂ = ∂sd/∂x` at the point.
    pub normal: Vec3,
    /// `κ · b(sd)` — barrier energy density.
    pub b: f64,
    /// `κ · b'(sd)` — first derivative w.r.t. signed distance.
    pub b_d: f64,
    /// `κ · b''(sd)` — second derivative w.r.t. signed distance.
    pub b_dd: f64,
}

/// 3-point degree-2 symmetric triangle rule (strictly interior points), with
/// **area-fraction** weights summing to 1 (the physical rest area `A_rest` is
/// applied separately). Matches the Tet10 element's degree-2 quadrature order
/// and integrates the degree-2 corner shape functions exactly (the
/// uniform-pressure corner-zero property). The points are interior, so the SDF
/// is sampled at genuinely non-nodal locations — the defining property of the
/// surface-integrated barrier over node collocation.
const FACE_GP: [(f64, f64, f64); 3] = [
    (1.0 / 6.0, 1.0 / 6.0, 1.0 / 3.0),
    (2.0 / 3.0, 1.0 / 6.0, 1.0 / 3.0),
    (1.0 / 6.0, 2.0 / 3.0, 1.0 / 3.0),
];

/// P2 triangle shape functions at parametric `(u, v)` — `L0 = 1-u-v` is corner
/// 0's barycentric (the complement convention matching the Tet10 element), `L1 =
/// u`, `L2 = v`. Order `[c0, c1, c2, m01, m12, m02]`.
fn face_shape(u: f64, v: f64) -> [f64; 6] {
    let l0 = 1.0 - u - v;
    let (l1, l2) = (u, v);
    [
        l0 * (2.0 * l0 - 1.0), // c0
        l1 * (2.0 * l1 - 1.0), // c1
        l2 * (2.0 * l2 - 1.0), // c2
        4.0 * l0 * l1,         // m01 — edge (c0, c1)
        4.0 * l1 * l2,         // m12 — edge (c1, c2)
        4.0 * l0 * l2,         // m02 — edge (c0, c2)
    ]
}

/// Isoparametric current position at parametric `(u, v)`: `x = Σ_i N_i x_i`.
fn face_point(nodes: &[Vec3; 6], n: &[f64; 6]) -> Vec3 {
    let mut x = Vec3::zeros();
    for i in 0..6 {
        x += n[i] * nodes[i];
    }
    x
}

/// Surface-integrated barrier energy `E = A_rest · Σ_q ŵ_q b(sd(x_q))`.
pub(crate) fn face_energy<F: Fn(Vec3) -> FaceBarrierEval>(
    nodes: &[Vec3; 6],
    rest_area: f64,
    eval: F,
) -> f64 {
    let mut e = 0.0;
    for &(u, v, w) in &FACE_GP {
        let n = face_shape(u, v);
        e += w * eval(face_point(nodes, &n)).b;
    }
    rest_area * e
}

/// Per-node barrier gradient contributions `force = +∂E/∂x_i` (the residual
/// contribution, matching the per-vertex path's sign) — six entries, purely
/// along the SDF normal.
pub(crate) fn face_gradient<F: Fn(Vec3) -> FaceBarrierEval>(
    nodes: &[Vec3; 6],
    rest_area: f64,
    eval: F,
) -> [Vec3; 6] {
    let mut grad = [Vec3::zeros(); 6];
    for &(u, v, w) in &FACE_GP {
        let n = face_shape(u, v);
        let be = eval(face_point(nodes, &n));
        let s = rest_area * w * be.b_d;
        for i in 0..6 {
            grad[i] += (s * n[i]) * be.normal;
        }
    }
    grad
}

/// Per-node-pair barrier Hessian blocks — all 6×6 = 36 blocks (the assembler
/// applies its own free-DOF lower-triangle filter; the block set is symmetric,
/// `H_{ji} = H_{ij}`, since each block is a scalar times the symmetric `n̂⊗n̂`).
pub(crate) fn face_hessian<F: Fn(Vec3) -> FaceBarrierEval>(
    nodes: &[Vec3; 6],
    rest_area: f64,
    eval: F,
) -> [[Matrix3<f64>; 6]; 6] {
    let mut h = [[Matrix3::zeros(); 6]; 6];
    for &(u, v, w) in &FACE_GP {
        let n = face_shape(u, v);
        let be = eval(face_point(nodes, &n));
        let nn = be.normal * be.normal.transpose();
        let s = rest_area * w * be.b_dd;
        // `h_row` is block-row i (paired with N_i); `h_ij` is block (i,j)
        // (paired with N_j) — the rank-1 `n̂⊗n̂` scaled by `N_i N_j`.
        for (h_row, &ni) in h.iter_mut().zip(&n) {
            for (h_ij, &nj) in h_row.iter_mut().zip(&n) {
                *h_ij += (s * ni * nj) * nn;
            }
        }
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Partition of unity and Kronecker-delta at the parametric node locations,
    /// exercising the `[c0,c1,c2,m01,m12,m02]` order the SUT relies on.
    #[test]
    fn shape_partition_and_kronecker() {
        let nodes_uv = [
            (0.0, 0.0), // c0 (L0=1)
            (1.0, 0.0), // c1
            (0.0, 1.0), // c2
            (0.5, 0.0), // m01
            (0.5, 0.5), // m12
            (0.0, 0.5), // m02
        ];
        for (k, &(u, v)) in nodes_uv.iter().enumerate() {
            let n = face_shape(u, v);
            let sum: f64 = n.iter().sum();
            assert!((sum - 1.0).abs() < 1e-14, "partition of unity at node {k}");
            for (l, &nl) in n.iter().enumerate() {
                let expect = if l == k { 1.0 } else { 0.0 };
                assert!(
                    (nl - expect).abs() < 1e-14,
                    "N_{l} at node {k} = {nl}, expected {expect}",
                );
            }
        }
    }

    /// Area-fraction weights sum to 1 (the physical rest area is applied
    /// separately).
    #[test]
    fn quadrature_weights_sum_to_one() {
        let sum: f64 = FACE_GP.iter().map(|&(_, _, w)| w).sum();
        assert!((sum - 1.0).abs() < 1e-15);
    }

    /// A flat face at uniform pressure: corner gradient contributions vanish
    /// exactly (`∫ N_corner dA = 0`), midsides carry the load, and every force is
    /// along the normal (no spurious tangential component — the rest-area
    /// property option A lacked).
    #[test]
    fn uniform_pressure_is_normal_only_corners_zero() {
        let z = 0.02;
        let c = [
            Vec3::new(0.0, 0.0, z),
            Vec3::new(1.0, 0.0, z),
            Vec3::new(0.0, 1.0, z),
        ];
        let nodes = [
            c[0],
            c[1],
            c[2],
            (c[0] + c[1]) * 0.5,
            (c[1] + c[2]) * 0.5,
            (c[0] + c[2]) * 0.5,
        ];
        let eval = |_x: Vec3| FaceBarrierEval {
            normal: Vec3::new(0.0, 0.0, 1.0),
            b: 3.0,
            b_d: -7.0,
            b_dd: 5.0,
        };
        let g = face_gradient(&nodes, 0.5, eval);
        for (i, gi) in g.iter().enumerate() {
            // every force is along the (z) normal — no tangential component
            let tang = gi.x.hypot(gi.y);
            assert!(
                tang < 1e-14,
                "node {i} tangential force {tang:e} must vanish"
            );
        }
        let corner_max = g[0].norm().max(g[1].norm()).max(g[2].norm());
        let midside_min = g[3].norm().min(g[4].norm()).min(g[5].norm());
        assert!(
            corner_max < 1e-14,
            "corner gradients must vanish by integration, got {corner_max:e}",
        );
        assert!(
            midside_min > 1e-3,
            "midsides must carry the load, got min {midside_min:e}",
        );
    }

    /// The decision record for rest- vs deformed-area weighting, held as a
    /// re-runnable measurement (not a prose claim). On a flat face at uniform
    /// normal pressure, the shipped **rest-area** gradient is purely normal
    /// (tangential exactly 0), while the **deformed-area** alternative
    /// `E = Σ_q ŵ_q b(sd(x_q)) · J_A(x)` injects a spurious *tangential*
    /// surface-tension force — measured here as a fraction of the normal force.
    /// A non-penetration barrier depends only on the normal gap, so that
    /// tangential force is an artifact; this test is why rung 8b weights by rest
    /// area. (The `~4.7 %` figure cited in the module docs is this measurement.)
    // The barrier/geometry math reads clearest with conventional single-letter
    // names (gap `z`, barrier `b`, position `x`, parametric `u`/`v`, corner set
    // `c`); renaming them would obscure the formulas.
    #[allow(clippy::many_single_char_names)]
    #[test]
    fn deformed_area_weighting_injects_a_tangential_force() {
        // Flat unit-right-triangle face at a uniform gap z over the plane z = 0.
        let z = 0.02_f64;
        let d_hat = 0.05_f64;
        let c = [
            Vec3::new(0.0, 0.0, z),
            Vec3::new(1.0, 0.0, z),
            Vec3::new(0.0, 1.0, z),
        ];
        let nodes = [
            c[0],
            c[1],
            c[2],
            (c[0] + c[1]) * 0.5,
            (c[1] + c[2]) * 0.5,
            (c[0] + c[2]) * 0.5,
        ];
        // Real IPC log-barrier scalars at signed distance sd (plane normal ẑ).
        let b = |sd: f64| -> f64 {
            let r = sd - d_hat;
            -(r * r) * (sd / d_hat).ln()
        };
        let b_d = |sd: f64| -> f64 {
            let r = sd - d_hat;
            -2.0 * r * (sd / d_hat).ln() - r * r / sd
        };
        // Test-local P2 shape gradients (∂N/∂u, ∂N/∂v) for the deformed area
        // element J_A = ‖∂x/∂u × ∂x/∂v‖.
        let shape_grad = |u: f64, v: f64| -> ([f64; 6], [f64; 6]) {
            let l0 = 1.0 - u - v;
            (
                [
                    -(4.0 * l0 - 1.0),
                    4.0 * u - 1.0,
                    0.0,
                    4.0 * (l0 - u),
                    4.0 * v,
                    -4.0 * v,
                ],
                [
                    -(4.0 * l0 - 1.0),
                    0.0,
                    4.0 * v - 1.0,
                    -4.0 * u,
                    4.0 * u,
                    4.0 * (l0 - v),
                ],
            )
        };
        let rest_area = 0.5; // unit right triangle
        // Shipped rest-area gradient: purely normal.
        let g_rest = face_gradient(&nodes, rest_area, |x: Vec3| FaceBarrierEval {
            normal: Vec3::new(0.0, 0.0, 1.0),
            b: b(x.z),
            b_d: b_d(x.z),
            b_dd: 0.0,
        });
        let rest_tang = g_rest.iter().map(|g| g.x.hypot(g.y)).fold(0.0, f64::max);
        assert!(
            rest_tang < 1e-14,
            "rest-area gradient must be normal-only, tangential = {rest_tang:e}",
        );

        // Deformed-area energy E = Σ_q ŵ_q b(sd(x_q)) J_A(x), gradient via FD.
        let energy_def = |nn: &[Vec3; 6]| -> f64 {
            let mut e = 0.0;
            for &(u, v, w) in &FACE_GP {
                let sh = face_shape(u, v);
                let (dnu, dnv) = shape_grad(u, v);
                let x = face_point(nn, &sh);
                let mut g_u = Vec3::zeros();
                let mut g_v = Vec3::zeros();
                for i in 0..6 {
                    g_u += dnu[i] * nn[i];
                    g_v += dnv[i] * nn[i];
                }
                let j_a = g_u.cross(&g_v).norm();
                e += w * b(x.z) * j_a;
            }
            e
        };
        let eps = 1e-7;
        let mut def_tang = 0.0_f64;
        let mut def_norm = 0.0_f64;
        for i in 0..6 {
            let mut g = Vec3::zeros();
            for d in 0..3 {
                let mut np = nodes;
                np[i][d] += eps;
                let mut nm = nodes;
                nm[i][d] -= eps;
                g[d] = (energy_def(&np) - energy_def(&nm)) / (2.0 * eps);
            }
            def_tang = def_tang.max(g.x.hypot(g.y));
            def_norm = def_norm.max(g.z.abs());
        }
        let frac = def_tang / def_norm;
        eprintln!("deformed-area spurious tangential fraction = {frac:.4}");
        // The spurious tangential force is ~4.7% of the normal force here —
        // real, non-negligible, and exactly what rest-area weighting removes.
        assert!(
            (0.03..0.06).contains(&frac),
            "deformed-area tangential fraction {frac:.4} outside the expected ~4.7% band",
        );
    }

    /// The analytic Face gradient matches a central-difference FD of the exact
    /// energy (a face with `sd` varying across it — varying `b` — against a flat
    /// plane; rest area a fixed weight).
    #[test]
    fn gradient_matches_fd_energy() {
        let nodes = varying_gap_face();
        let eval = plane_barrier();
        let rest_area = 0.5;
        let analytic = face_gradient(&nodes, rest_area, &eval);
        let eps = 1e-7;
        for i in 0..6 {
            for d in 0..3 {
                let mut np = nodes;
                np[i][d] += eps;
                let mut nm = nodes;
                nm[i][d] -= eps;
                let fd = (face_energy(&np, rest_area, &eval) - face_energy(&nm, rest_area, &eval))
                    / (2.0 * eps);
                assert!(
                    (analytic[i][d] - fd).abs() < 1e-6 * analytic[i].norm().max(1e-6),
                    "grad[{i}][{d}]: analytic {} vs FD {}",
                    analytic[i][d],
                    fd,
                );
            }
        }
    }

    /// The analytic Face Hessian matches a central-difference FD of the exact
    /// gradient — the tangent the differentiable adjoint consumes.
    #[test]
    fn hessian_matches_fd_gradient() {
        let nodes = varying_gap_face();
        let eval = plane_barrier();
        let rest_area = 0.5;
        let analytic = face_hessian(&nodes, rest_area, &eval);
        let eps = 1e-6;
        let mut max_diff = 0.0f64;
        let mut max_mag = 0.0f64;
        for j in 0..6 {
            for e in 0..3 {
                let mut np = nodes;
                np[j][e] += eps;
                let gp = face_gradient(&np, rest_area, &eval);
                let mut nm = nodes;
                nm[j][e] -= eps;
                let gm = face_gradient(&nm, rest_area, &eval);
                for i in 0..6 {
                    for d in 0..3 {
                        let fd = (gp[i][d] - gm[i][d]) / (2.0 * eps);
                        let a = analytic[i][j][(d, e)];
                        // Assert finiteness of BOTH sides first: `f64::max(x, NaN)
                        // == x` would otherwise let a NaN slip through the
                        // max-reduction as a false green.
                        assert!(
                            a.is_finite() && fd.is_finite(),
                            "H[{i}][{j}]({d},{e}) non-finite: analytic {a}, fd {fd}",
                        );
                        max_diff = max_diff.max((a - fd).abs());
                        max_mag = max_mag.max(a.abs());
                    }
                }
            }
        }
        assert!(
            max_diff < 1e-5 * max_mag,
            "max|H_analytic - H_FD| = {max_diff:e} vs max|H| = {max_mag:e}",
        );
    }

    /// The global Hessian block set is symmetric: `H_{ji} = H_{ij}`.
    #[test]
    fn hessian_is_symmetric() {
        let nodes = varying_gap_face();
        let h = face_hessian(&nodes, 0.5, plane_barrier());
        let mut max_asym = 0.0f64;
        for (i, row) in h.iter().enumerate() {
            for (j, h_ij) in row.iter().enumerate() {
                let asym = (h_ij - h[j][i]).abs().max();
                max_asym = max_asym.max(asym);
            }
        }
        assert!(max_asym < 1e-12, "asymmetry {max_asym:e}");
    }

    /// A face whose nodes sit at different heights over the ground plane, so `sd`
    /// (hence `b`) varies across it — all Gauss points strictly inside the
    /// barrier band `(0, d̂)`.
    fn varying_gap_face() -> [Vec3; 6] {
        [
            Vec3::new(0.0, 0.0, 0.030),
            Vec3::new(1.0, 0.1, 0.012),
            Vec3::new(0.15, 0.9, 0.040),
            Vec3::new(0.55, 0.02, 0.020),
            Vec3::new(0.62, 0.55, 0.008),
            Vec3::new(0.05, 0.5, 0.035),
        ]
    }

    /// Flat ground-plane barrier `z = 0`, outward normal `ẑ` (`sd = x_z`, normal
    /// constant — the flat-primitive scope). Uses the real IPC log-barrier so the
    /// scalars are a genuine `(b, b', b'')` triple.
    fn plane_barrier() -> impl Fn(Vec3) -> FaceBarrierEval {
        let d_hat = 0.05_f64;
        let kappa = 1.0e4_f64;
        let n = Vec3::new(0.0, 0.0, 1.0);
        move |x: Vec3| {
            let sd = x.z;
            if sd >= d_hat {
                return FaceBarrierEval {
                    normal: n,
                    b: 0.0,
                    b_d: 0.0,
                    b_dd: 0.0,
                };
            }
            let d = sd.max(d_hat * 1e-6);
            let r = d - d_hat;
            let ln = (d / d_hat).ln();
            FaceBarrierEval {
                normal: n,
                b: kappa * (-(r * r) * ln),
                b_d: kappa * (-2.0 * r * ln - r * r / d),
                b_dd: kappa * (r * r / (d * d) - 4.0 * r / d - 2.0 * ln),
            }
        }
    }
}
