//! Quadratic tetrahedron (10 nodes, 4 Gauss points) — linear strain.
//!
//! A Tet10 element carries the 4 corner nodes of [`Tet4`](super::Tet4) plus
//! one midside node per edge (6 total). Its shape functions are quadratic in
//! the barycentric coordinates `(ξ, η, ζ)` on the reference tetrahedron
//! `ξ_i ≥ 0, Σ ξ_i ≤ 1`, so the deformation gradient — and hence strain and
//! stress — varies *linearly* across the element. That is the Tet4 → Tet10
//! step that lets a single element represent bending. The element math is
//! written down in the spec book at
//! `docs/studies/soft_body_architecture/src/30-discretization/00-element-choice/01-tet10.md`;
//! this module is the crate realization of it.
//!
//! # Canonical node convention (the single source of truth)
//!
//! **★ Corner convention (must match [`Tet4`](super::Tet4), `tet4.rs:26`).**
//! Local corner node `0` is the `1 − ξ − η − ζ` *complement* barycentric;
//! nodes `1`/`2`/`3` are `ξ`/`η`/`ζ`. Everything below silently inverts if a
//! caller ever takes node `0` to be `ξ` instead of the complement.
//!
//! **★ Edge → midside-node table ([`TET10_EDGE_NODES`]).** Local midside node
//! `4 + i` sits at the midpoint of the corner edge `TET10_EDGE_NODES[i]`:
//!
//! | midside node | corner edge | shape function |
//! |---|---|---|
//! | 4 | (0, 1) | `4 ξ₀ ξ`  |
//! | 5 | (1, 2) | `4 ξ η`   |
//! | 6 | (0, 2) | `4 ξ₀ η`  |
//! | 7 | (0, 3) | `4 ξ₀ ζ`  |
//! | 8 | (1, 3) | `4 ξ ζ`   |
//! | 9 | (2, 3) | `4 η ζ`   |
//!
//! (`ξ₀ := 1 − ξ − η − ζ` is corner 0's barycentric.) This is the spec's
//! *implicit* 1-based order `N_5..N_10` re-expressed 0-based; the spec writes
//! it nowhere as a table, so an implementer who codes the Abaqus/textbook
//! order gets a plausible-but-wrong element. **Every future Tet10 piece — the
//! edge-enrichment helper, the mesh-topology channel, multi-Gauss-point
//! assembly, and the 6-node boundary faces — must cite this one table**, and
//! the `(min, max)` global-vertex dedup key must decide node *identity* only,
//! never the local slot `4..9` (that comes from this table).
//!
//! # Quadrature
//!
//! Because the strain is non-constant, the element integrals do not collapse
//! to a single centroid evaluation as they do for Tet4. [`Tet10`] uses the
//! 4-point symmetric Stroud rule on the reference tetrahedron
//! ([`gauss_points`](Element::gauss_points)): degree-of-precision 2 — exact
//! for the linear-elastic stiffness integrand of a *straight-edged* element,
//! where the affine map's constant Jacobian keeps that integrand quadratic (a
//! curved/isoparametric Tet10 would not be exact) — with equal reference
//! weights `V_ref / 4 = 1/24` at the barycentric points `(a, b, b, b)` and its
//! permutations, `a = (5 + 3√5)/20`, `b = (5 − √5)/20`. The four weights sum
//! to the reference-tet volume `1/6`, exactly as Tet4's single centroid weight
//! does; per-element `|detJ|` scaling is the assembler's job.

use nalgebra::{SMatrix, SVector};

use super::Element;
use crate::Vec3;

/// Canonical edge → midside-node table (see module docs).
///
/// Local midside node `4 + i` is the midpoint of the corner edge
/// `TET10_EDGE_NODES[i]`. This is the single source of truth every Tet10
/// consumer cites; the `(min, max)` dedup key used elsewhere fixes node
/// identity only, never this local slot.
pub const TET10_EDGE_NODES: [(usize, usize); 6] = [(0, 1), (1, 2), (0, 2), (0, 3), (1, 3), (2, 3)];

/// Quadratic tetrahedron element. Linear strain, 30 DOFs total.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tet10;

impl Element<10, 4> for Tet10 {
    fn shape_functions(&self, xi: Vec3) -> SVector<f64, 10> {
        // Barycentric coordinates: l0 is the complement (corner 0), and
        // (x, y, z) = (ξ, η, ζ) are corners 1/2/3.
        let (x, y, z) = (xi.x, xi.y, xi.z);
        let l0 = 1.0 - x - y - z;

        SVector::<f64, 10>::from_column_slice(&[
            // Corners: N = ξ_c (2 ξ_c − 1).
            l0 * (2.0 * l0 - 1.0), // node 0 (complement)
            x * (2.0 * x - 1.0),   // node 1 (ξ)
            y * (2.0 * y - 1.0),   // node 2 (η)
            z * (2.0 * z - 1.0),   // node 3 (ζ)
            // Midsides: N = 4 ξ_a ξ_b for edge (a, b) per TET10_EDGE_NODES.
            4.0 * l0 * x, // node 4 — edge (0, 1)
            4.0 * x * y,  // node 5 — edge (1, 2)
            4.0 * l0 * y, // node 6 — edge (0, 2)
            4.0 * l0 * z, // node 7 — edge (0, 3)
            4.0 * x * z,  // node 8 — edge (1, 3)
            4.0 * y * z,  // node 9 — edge (2, 3)
        ])
    }

    // Row `a` = `∂N_a / ∂ξ` (linear in `xi`, since the shape functions are
    // quadratic). The complement's derivative `∂l0/∂ξ_k = −1` threads through
    // every corner-0 and midside term.
    fn shape_gradients(&self, xi: Vec3) -> SMatrix<f64, 10, 3> {
        let (x, y, z) = (xi.x, xi.y, xi.z);
        let l0 = 1.0 - x - y - z;

        // Each row is `[∂N_a/∂ξ, ∂N_a/∂η, ∂N_a/∂ζ]`.
        let rows: [[f64; 3]; 10] = [
            // Corners.
            [1.0 - 4.0 * l0, 1.0 - 4.0 * l0, 1.0 - 4.0 * l0], // node 0
            [4.0 * x - 1.0, 0.0, 0.0],                        // node 1
            [0.0, 4.0 * y - 1.0, 0.0],                        // node 2
            [0.0, 0.0, 4.0 * z - 1.0],                        // node 3
            // Midsides (4 ξ_a ξ_b): product rule with ∂l0 = −1.
            [4.0 * (l0 - x), -4.0 * x, -4.0 * x], // node 4 — 4 l0 x
            [4.0 * y, 4.0 * x, 0.0],              // node 5 — 4 x y
            [-4.0 * y, 4.0 * (l0 - y), -4.0 * y], // node 6 — 4 l0 y
            [-4.0 * z, -4.0 * z, 4.0 * (l0 - z)], // node 7 — 4 l0 z
            [4.0 * z, 0.0, 4.0 * x],              // node 8 — 4 x z
            [0.0, 4.0 * z, 4.0 * y],              // node 9 — 4 y z
        ];
        SMatrix::<f64, 10, 3>::from_row_iterator(rows.into_iter().flatten())
    }

    fn gauss_points(&self) -> [(Vec3, f64); 4] {
        // 4-point symmetric Stroud rule, degree-of-precision 2. Barycentric
        // points (a, b, b, b) and permutations; a + 3b = 1 (closed-form
        // verified). Each reference weight is V_ref / 4 = (1/6)/4 = 1/24, so
        // the four sum to the reference-tet volume 1/6.
        let sqrt5 = 5.0_f64.sqrt();
        let a = (5.0 + 3.0 * sqrt5) / 20.0; // ≈ 0.585410
        let b = (5.0 - sqrt5) / 20.0; // ≈ 0.138197
        let w = 1.0 / 24.0;

        // Parametric point = (ξ, η, ζ) = barycentric coords 1/2/3; the `a`
        // rotates through complement, ξ, η, ζ in turn.
        [
            (Vec3::new(b, b, b), w), // a on the complement (corner 0)
            (Vec3::new(a, b, b), w), // a on ξ (corner 1)
            (Vec3::new(b, a, b), w), // a on η (corner 2)
            (Vec3::new(b, b, a), w), // a on ζ (corner 3)
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{DMatrix, SMatrix};

    /// Parametric coordinates of every local node, derived from the corner
    /// convention and [`TET10_EDGE_NODES`] so the table itself is exercised.
    fn node_parametric_coords() -> [Vec3; 10] {
        let corners = [
            Vec3::new(0.0, 0.0, 0.0), // node 0 (complement)
            Vec3::new(1.0, 0.0, 0.0), // node 1 (ξ)
            Vec3::new(0.0, 1.0, 0.0), // node 2 (η)
            Vec3::new(0.0, 0.0, 1.0), // node 3 (ζ)
        ];
        let mut coords = [Vec3::zeros(); 10];
        coords[..4].copy_from_slice(&corners);
        for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
            coords[4 + i] = (corners[a] + corners[b]) * 0.5;
        }
        coords
    }

    /// A representative interior parametric point (not on any symmetry axis).
    fn sample_point() -> Vec3 {
        Vec3::new(0.17, 0.29, 0.11)
    }

    /// Ten node coordinates of a distorted (irregular) straight-edged Tet10:
    /// generic, asymmetric corners with each midside re-placed at its straight
    /// edge midpoint per [`TET10_EDGE_NODES`]. Straight edges keep the
    /// isoparametric map affine (so `detJ` is constant), while the asymmetry
    /// removes any element symmetry a midside permutation could hide behind.
    fn distorted_element_coords() -> [Vec3; 10] {
        let corners = [
            Vec3::new(0.1, 0.0, -0.2),
            Vec3::new(1.3, 0.2, 0.1),
            Vec3::new(0.2, 1.1, 0.3),
            Vec3::new(-0.1, 0.4, 1.2),
        ];
        let mut coords = [Vec3::zeros(); 10];
        coords[..4].copy_from_slice(&corners);
        for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
            coords[4 + i] = (corners[a] + corners[b]) * 0.5;
        }
        coords
    }

    #[test]
    fn partition_of_unity() {
        // Σ N_i = 1 everywhere, and (since the sum is constant) Σ ∇N_i = 0.
        for xi in [sample_point(), Vec3::new(0.05, 0.6, 0.3), Vec3::zeros()] {
            let n = Tet10.shape_functions(xi);
            assert!(
                (n.sum() - 1.0).abs() < 1e-12,
                "Σ N_i = {} at {xi:?}",
                n.sum()
            );

            let grad = Tet10.shape_gradients(xi);
            let grad_sum: Vec3 = grad.row_iter().map(|r| Vec3::new(r[0], r[1], r[2])).sum();
            assert!(grad_sum.norm() < 1e-12, "Σ ∇N_i = {grad_sum:?} at {xi:?}");
        }
    }

    #[test]
    fn kronecker_delta_at_nodes() {
        // N_i(node_j) = δ_ij — the defining interpolation property.
        let coords = node_parametric_coords();
        for (j, &xj) in coords.iter().enumerate() {
            let n = Tet10.shape_functions(xj);
            for i in 0..10 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (n[i] - expected).abs() < 1e-12,
                    "N_{i}(node {j}) = {}, expected {expected}",
                    n[i]
                );
            }
        }
    }

    #[test]
    fn shape_gradients_match_finite_difference() {
        // Central FD of shape_functions must match analytic shape_gradients.
        let xi = sample_point();
        let analytic = Tet10.shape_gradients(xi);
        let h = 1e-6;
        for axis in 0..3 {
            let mut plus = xi;
            let mut minus = xi;
            plus[axis] += h;
            minus[axis] -= h;
            let fd = (Tet10.shape_functions(plus) - Tet10.shape_functions(minus)) / (2.0 * h);
            for node in 0..10 {
                assert!(
                    (fd[node] - analytic[(node, axis)]).abs() < 1e-7,
                    "∂N_{node}/∂ξ_{axis}: fd {} vs analytic {}",
                    fd[node],
                    analytic[(node, axis)]
                );
            }
        }
    }

    #[test]
    fn quadrature_integrates_degree_two_exactly() {
        // ∫_ref ξ^i η^j ζ^k dV = i! j! k! / (i + j + k + 3)! on the reference
        // tet. Check every monomial up to total degree 2.
        fn factorial(n: i32) -> f64 {
            (1..=n).map(f64::from).product()
        }
        fn analytic(i: i32, j: i32, k: i32) -> f64 {
            factorial(i) * factorial(j) * factorial(k) / factorial(i + j + k + 3)
        }

        let gps = Tet10.gauss_points();
        let monomials: [(i32, i32, i32); 10] = [
            (0, 0, 0),
            (1, 0, 0),
            (0, 1, 0),
            (0, 0, 1),
            (2, 0, 0),
            (0, 2, 0),
            (0, 0, 2),
            (1, 1, 0),
            (0, 1, 1),
            (1, 0, 1),
        ];
        for (i, j, k) in monomials {
            let quad: f64 = gps
                .iter()
                .map(|&(p, w)| w * p.x.powi(i) * p.y.powi(j) * p.z.powi(k))
                .sum();
            let exact = analytic(i, j, k);
            assert!(
                (quad - exact).abs() < 1e-14,
                "∫ ξ^{i} η^{j} ζ^{k}: quad {quad} vs exact {exact}"
            );
        }
    }

    #[test]
    fn stroud_weights_sum_to_reference_volume() {
        // The four reference weights sum to the reference-tet volume 1/6,
        // matching Tet4's single centroid weight.
        let total: f64 = Tet10.gauss_points().iter().map(|&(_, w)| w).sum();
        assert!((total - 1.0 / 6.0).abs() < 1e-15, "Σ w = {total}");
    }

    // --- Rank / eigenspectrum micro-gate --------------------------------
    //
    // A single-element small-strain (F = I) linear-elastic K^e must have
    // exactly 6 zero eigenvalues (3 translations + 3 rotations) and 24
    // positive ones — no spurious zero-energy (hourglass) modes. This is a
    // self-contained element assembly, independent of the solver.
    //
    // ⚠ Permutation-invariant (`K → P K Pᵀ` is a similarity transform), so it
    // is BLIND to a midside-ordering bug — that is caught by the asymmetric /
    // quadratic-field patch test on an irregular element (Tet10 ladder step 5,
    // docs/SIM_SOFT_TET10_PLAN.md), not here. Its job is spurious-mode
    // detection and edge-table sanity.

    /// Isotropic linear-elastic 6×6 constitutive matrix in Voigt order
    /// `[xx, yy, zz, xy, yz, zx]` (engineering shear).
    fn elasticity_matrix(e: f64, nu: f64) -> SMatrix<f64, 6, 6> {
        let lambda = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
        let mu = e / (2.0 * (1.0 + nu));
        let mut d = SMatrix::<f64, 6, 6>::zeros();
        for a in 0..3 {
            for b in 0..3 {
                d[(a, b)] = lambda;
            }
            d[(a, a)] += 2.0 * mu;
            d[(3 + a, 3 + a)] = mu;
        }
        d
    }

    /// Build the 30×30 small-strain stiffness of one Tet10 with the given nodal
    /// coordinates, integrating over the 4 Stroud points. Uses the per-Gauss-
    /// point Σ-form Jacobian, so it is the same math the solver's
    /// `curved_gauss_geometry` assembles — exact for a straight-edged element
    /// (affine map ⇒ constant Jacobian) and the 4-point approximation for a
    /// curved one (see [`curved_quadrature_4pt_adequacy`]).
    fn element_stiffness(coords: &[Vec3; 10]) -> DMatrix<f64> {
        element_stiffness_with_rule(coords, &Tet10.gauss_points())
    }

    /// [`element_stiffness`] over an arbitrary quadrature `rule` (`(ξ, weight)`
    /// pairs on the reference tet) — lets the curved-quadrature gate integrate
    /// the same `K^e` with a dense reference rule instead of the 4-point Stroud.
    // Test elements are constructed non-degenerate, so a failed inverse is a
    // test-authoring bug worth panicking on immediately.
    #[allow(clippy::expect_used)]
    fn element_stiffness_with_rule(coords: &[Vec3; 10], rule: &[(Vec3, f64)]) -> DMatrix<f64> {
        let d_mat = elasticity_matrix(1.0, 0.3);
        let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
        let mut k_mat = DMatrix::<f64>::zeros(30, 30);

        for &(xi, weight) in rule {
            let grad_xi = Tet10.shape_gradients(xi); // 10×3, ∂N/∂ξ
            let jac = node_x.transpose() * grad_xi; // 3×3, ∂X/∂ξ = Σ X_i ⊗ ∇_ξ N_i
            let jac_inv = jac.try_inverse().expect("non-degenerate element");
            let det = jac.determinant();
            let grad_x = grad_xi * jac_inv; // 10×3, ∂N/∂X

            // Strain-displacement B (6×30) in the same Voigt order as `d_mat`.
            let mut b_mat = SMatrix::<f64, 6, 30>::zeros();
            for node in 0..10 {
                let (nx, ny, nz) = (grad_x[(node, 0)], grad_x[(node, 1)], grad_x[(node, 2)]);
                let col = 3 * node;
                b_mat[(0, col)] = nx;
                b_mat[(1, col + 1)] = ny;
                b_mat[(2, col + 2)] = nz;
                b_mat[(3, col)] = ny;
                b_mat[(3, col + 1)] = nx; // xy
                b_mat[(4, col + 1)] = nz;
                b_mat[(4, col + 2)] = ny; // yz
                b_mat[(5, col)] = nz;
                b_mat[(5, col + 2)] = nx; // zx
            }

            let contribution = b_mat.transpose() * d_mat * b_mat * (weight * det.abs());
            k_mat += DMatrix::from_fn(30, 30, |i, j| contribution[(i, j)]);
        }
        k_mat
    }

    /// Count (near-zero, positive) eigenvalues of a symmetric matrix, scaled
    /// by its spectral radius. Asserts none are meaningfully negative.
    fn eigen_counts(k: &DMatrix<f64>) -> (usize, usize) {
        let eig = k.clone().symmetric_eigenvalues();
        let scale = eig.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
        let tol = 1e-9 * scale;
        let mut zero = 0;
        let mut positive = 0;
        for &lambda in eig.iter() {
            assert!(lambda > -tol, "meaningfully negative eigenvalue {lambda}");
            if lambda.abs() <= tol {
                zero += 1;
            } else {
                positive += 1;
            }
        }
        (zero, positive)
    }

    #[test]
    fn rank_gate_reference_element() {
        let coords = node_parametric_coords(); // reference tet, straight edges
        let (zero, positive) = eigen_counts(&element_stiffness(&coords));
        assert_eq!(zero, 6, "expected 6 rigid-body modes");
        assert_eq!(positive, 24, "expected 24 deformation modes");
    }

    #[test]
    fn rank_gate_distorted_element() {
        let coords = distorted_element_coords();
        let (zero, positive) = eigen_counts(&element_stiffness(&coords));
        assert_eq!(zero, 6, "expected 6 rigid-body modes (distorted)");
        assert_eq!(positive, 24, "expected 24 deformation modes (distorted)");
    }

    // --- Rung 5: element-correctness gates (patch tests) -----------------
    //
    // These are the element-primitive companions to the production-path gates
    // in `solver::backward_euler::tests` (the finite-rotation rigid-body gate
    // and the asymmetric quadratic-field ordering detector). They pin the
    // element's completeness on a distorted element; the production gates then
    // prove the whole enrich → element_node_ids → assembler chain honors the
    // same node ordering (docs/SIM_SOFT_TET10_PLAN.md §5 step 5).

    /// Isoparametric completeness on a distorted element: the quadratic shape
    /// functions reproduce the (affine, straight-edged) geometry map exactly,
    /// `Σᵢ Nᵢ(ξ) Xᵢ = X(ξ) = X₀ + J·ξ`, at a non-symmetric interior point. A
    /// midside sitting at a slot that disagreed with its shape function's
    /// reference edge would break this.
    #[test]
    fn isoparametric_completeness_on_distorted_element() {
        let coords = distorted_element_coords();
        let xi = sample_point(); // off every symmetry axis
        let n = Tet10.shape_functions(xi);
        let interp = (0..10).fold(Vec3::zeros(), |acc, i| acc + n[i] * coords[i]);
        // Affine map from the four corners (straight edges ⇒ midsides inert here).
        let affine = coords[0]
            + (coords[1] - coords[0]) * xi.x
            + (coords[2] - coords[0]) * xi.y
            + (coords[3] - coords[0]) * xi.z;
        assert!(
            (interp - affine).norm() < 1e-13,
            "Σ Nᵢ(ξ) Xᵢ = {interp:?} ≠ affine X(ξ) = {affine:?}",
        );
    }

    /// Quadratic-field completeness at the element level: a genuinely quadratic
    /// displacement `u(X) = (a Y², b Z, c X Y)` lies in the Tet10 approximation
    /// space, so the reconstructed deformation gradient `F = Σ xₐ ⊗ ∇ₓNₐ`
    /// matches the analytic `I + ∇u(X(ξ))` to machine precision at a
    /// non-symmetric interior point. The asymmetric shear term `c X Y` is what
    /// makes a midside permutation visible — a *linear* field cannot (see
    /// [`constant_strain_patch_reproduces_linear_field`]).
    // Test elements are constructed non-degenerate, so a failed inverse is a
    // test-authoring bug worth panicking on (as in `element_stiffness`).
    #[allow(clippy::expect_used)]
    #[test]
    fn deformation_gradient_reproduces_quadratic_field() {
        let coords = distorted_element_coords();
        let xi = sample_point();

        // Element geometry at ξ (affine ⇒ constant, evaluated generically).
        let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
        let grad_xi = Tet10.shape_gradients(xi);
        let jac = node_x.transpose() * grad_xi;
        let grad_x = grad_xi * jac.try_inverse().expect("non-degenerate element");

        // Physical point X(ξ) and the analytic quadratic field there.
        let nvals = Tet10.shape_functions(xi);
        let x_phys = (0..10).fold(Vec3::zeros(), |acc, i| acc + nvals[i] * coords[i]);

        let (a, b, c) = (1.5, 2.0, 1.0);
        let u = |p: Vec3| Vec3::new(a * p.y * p.y, b * p.z, c * p.x * p.y);
        // ∇u rows = (u_x, u_y, u_z), cols = ∂/∂(X, Y, Z).
        let grad_u = |p: Vec3| {
            nalgebra::Matrix3::new(0.0, 2.0 * a * p.y, 0.0, 0.0, 0.0, b, c * p.y, c * p.x, 0.0)
        };

        // Displaced nodes, then reconstruct F = Σ xₐ ⊗ ∇ₓNₐ (the assembler's form).
        let x_def = SMatrix::<f64, 10, 3>::from_fn(|i, j| (coords[i] + u(coords[i]))[j]);
        let f_recon = x_def.transpose() * grad_x;
        let f_analytic = nalgebra::Matrix3::identity() + grad_u(x_phys);
        let max_diff = (f_recon - f_analytic)
            .iter()
            .fold(0.0_f64, |m, &v| m.max(v.abs()));
        assert!(
            max_diff < 1e-12,
            "reconstructed F deviates from I + ∇u(X(ξ)): max |Δ| = {max_diff:e}",
        );
    }

    /// Constant-strain (linear-field) patch companion — machine-precision on a
    /// straight-edged element, but deliberately BLIND to midside ordering: with
    /// `x = A·X`, the reconstructed `F = A · J_bug · J_bug⁻¹ = A` cancels any
    /// consistent slot permutation. This is why the ordering detector needs the
    /// quadratic field above (and the production gate in
    /// `solver::backward_euler::tests`), not a constant strain.
    // Non-degenerate test element ⇒ a failed inverse is a test-authoring bug
    // worth panicking on (as in `element_stiffness`).
    #[allow(clippy::expect_used)]
    #[test]
    fn constant_strain_patch_reproduces_linear_field() {
        let coords = distorted_element_coords();
        let xi = sample_point();
        let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
        let grad_xi = Tet10.shape_gradients(xi);
        let grad_x = grad_xi
            * (node_x.transpose() * grad_xi)
                .try_inverse()
                .expect("non-degenerate element");

        // A generic asymmetric constant strain G; F = I + G everywhere.
        let g = nalgebra::Matrix3::new(0.03, -0.02, 0.01, 0.00, 0.04, -0.01, 0.02, 0.01, -0.03);
        let a_map = nalgebra::Matrix3::identity() + g;
        let x_def = SMatrix::<f64, 10, 3>::from_fn(|i, j| (a_map * coords[i])[j]);
        let f_recon = x_def.transpose() * grad_x;
        let max_diff = (f_recon - a_map)
            .iter()
            .fold(0.0_f64, |m, &v| m.max(v.abs()));
        assert!(
            max_diff < 1e-13,
            "linear field must reproduce F = I + G exactly; max |Δ| = {max_diff:e}",
        );
    }

    // --- Curved (isoparametric) element gates ----------------------------
    //
    // The rung that lets a Tet10 honor a CURVED soft surface: with midsides
    // moved off the straight edge midpoints, the isoparametric map is genuinely
    // quadratic, so J(ξ) = Σ Xₐ ⊗ ∇_ξNₐ(ξ) — and detJ(ξ) — vary per Gauss
    // point. These gates validate the per-GP geometry the solver's
    // `curved_gauss_geometry` builds, and re-establish which rung-5 element
    // invariants survive curvature. (`element_stiffness` above is already the
    // per-GP Σ-form, so it doubles as the curved assembler.)

    /// A distorted element with each midside pushed OFF the straight edge
    /// midpoint, perpendicular to its edge, by `curv` × edge length — a
    /// genuinely curved isoparametric Tet10. `curv = 0` is the straight element.
    fn curved_element_coords(curv: f64) -> [Vec3; 10] {
        let corners = [
            Vec3::new(0.1, 0.0, -0.2),
            Vec3::new(1.3, 0.2, 0.1),
            Vec3::new(0.2, 1.1, 0.3),
            Vec3::new(-0.1, 0.4, 1.2),
        ];
        let bias = Vec3::new(0.31, -0.57, 0.76).normalize();
        let mut coords = [Vec3::zeros(); 10];
        coords[..4].copy_from_slice(&corners);
        for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
            let mid = (corners[a] + corners[b]) * 0.5;
            let edge = corners[b] - corners[a];
            let e = edge / edge.norm();
            // Component of `bias` ⟂ to the edge, so the midside bows the mapped
            // edge rather than sliding along it.
            let perp = bias - e * bias.dot(&e);
            coords[4 + i] = mid + perp.normalize() * (curv * edge.norm());
        }
        coords
    }

    /// The per-GP Jacobian `J(ξ) = Σ Xₐ ⊗ ∇_ξNₐ` equals the finite difference of
    /// the geometry map `X(ξ) = Σ Nₐ(ξ) Xₐ` — an INDEPENDENT check of the Σ-form
    /// the curved assembler uses (FD never touches that form).
    #[test]
    fn curved_jacobian_matches_fd_of_map() {
        let coords = curved_element_coords(0.10);
        let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
        let map = |xi: Vec3| {
            let n = Tet10.shape_functions(xi);
            (0..10).fold(Vec3::zeros(), |acc, i| acc + n[i] * coords[i])
        };
        let h = 1e-6;
        let mut max_err = 0.0_f64;
        for (xi, _) in Tet10.gauss_points() {
            let j_analytic = node_x.transpose() * Tet10.shape_gradients(xi);
            for k in 0..3 {
                let mut ep = xi;
                let mut em = xi;
                ep[k] += h;
                em[k] -= h;
                let col = (map(ep) - map(em)) / (2.0 * h);
                for i in 0..3 {
                    max_err = max_err.max((j_analytic[(i, k)] - col[i]).abs());
                }
            }
        }
        assert!(max_err < 1e-7, "curved J vs FD(map): max |Δ| = {max_err:e}");
    }

    /// Isoparametric completeness under curvature: `Xᵀ · grad_x_n = I` to machine
    /// precision at every Gauss point. This is the identity that guarantees a
    /// curved element still reproduces ANY linear (constant-strain) field
    /// exactly — the basis for [`constant_strain_reproduces_on_curved_element`].
    // Non-degenerate curved element ⇒ a failed inverse is a test-authoring bug.
    #[allow(clippy::expect_used)]
    #[test]
    fn curved_element_completeness_is_machine_exact() {
        let coords = curved_element_coords(0.10);
        let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
        let mut max_err = 0.0_f64;
        for (xi, _) in Tet10.gauss_points() {
            let grad_xi = Tet10.shape_gradients(xi);
            let grad_x = grad_xi
                * (node_x.transpose() * grad_xi)
                    .try_inverse()
                    .expect("non-degenerate curved element");
            let m = node_x.transpose() * grad_x; // must equal I
            for i in 0..3 {
                for k in 0..3 {
                    let want = f64::from(u8::from(i == k));
                    max_err = max_err.max((m[(i, k)] - want).abs());
                }
            }
        }
        assert!(
            max_err < 1e-12,
            "Xᵀ·grad_x_n = I under curvature: max |Δ| = {max_err:e}",
        );
    }

    /// Rank/eigenspectrum survives curvature: a curved single-element `K^e`
    /// still has exactly 6 rigid-body zeros + 24 positive modes. Rigid modes
    /// give zero strain pointwise → zero energy at every Gauss point, so they
    /// stay in the null space regardless of the per-GP Jacobian.
    #[test]
    fn rank_gate_curved_element() {
        let (zero, positive) = eigen_counts(&element_stiffness(&curved_element_coords(0.10)));
        assert_eq!(zero, 6, "expected 6 rigid-body modes (curved)");
        assert_eq!(positive, 24, "expected 24 deformation modes (curved)");
    }

    /// Constant-strain (linear field) reproduction is STILL machine-exact on a
    /// CURVED element — the empirical correction to the plan's expectation that
    /// it would only be approximate under curvature. It follows from
    /// completeness above: with `x = A·X`,
    /// `F = Σ (A·Xₐ) ⊗ ∇ₓNₐ = A·(Σ Xₐ ⊗ ∇ₓNₐ) = A·I = A` at every point,
    /// independent of edge curvature.
    // Non-degenerate curved element ⇒ a failed inverse is a test-authoring bug.
    #[allow(clippy::expect_used)]
    #[test]
    fn constant_strain_reproduces_on_curved_element() {
        let coords = curved_element_coords(0.10);
        let xi = sample_point();
        let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
        let grad_xi = Tet10.shape_gradients(xi);
        let grad_x = grad_xi
            * (node_x.transpose() * grad_xi)
                .try_inverse()
                .expect("non-degenerate curved element");
        let g = nalgebra::Matrix3::new(0.03, -0.02, 0.01, 0.00, 0.04, -0.01, 0.02, 0.01, -0.03);
        let a_map = nalgebra::Matrix3::identity() + g;
        let x_def = SMatrix::<f64, 10, 3>::from_fn(|i, j| (a_map * coords[i])[j]);
        let f_recon = x_def.transpose() * grad_x;
        let max_diff = (f_recon - a_map)
            .iter()
            .fold(0.0_f64, |m, &v| m.max(v.abs()));
        assert!(
            max_diff < 1e-12,
            "curved element must still reproduce F = A exactly; max |Δ| = {max_diff:e}",
        );
    }

    /// Quadratic-field reproduction DEGRADES on a curved element (the honest
    /// redefinition of the straight-only [`deformation_gradient_reproduces_quadratic_field`]).
    /// A physical-space quadratic composed with the quadratic geometry map is
    /// degree-4 — outside the P2 space — so `F_recon ≠ I + ∇u`, and the error
    /// vanishes as curvature → 0 (recovering the straight exact case).
    /// Characterized as monotone convergence, not a fixed threshold.
    // Non-degenerate curved element ⇒ a failed inverse is a test-authoring bug.
    #[allow(clippy::expect_used)]
    #[test]
    fn quadratic_field_reproduction_degrades_with_curvature() {
        let (a, b, c) = (1.5, 2.0, 1.0);
        let u = |p: Vec3| Vec3::new(a * p.y * p.y, b * p.z, c * p.x * p.y);
        let grad_u = |p: Vec3| {
            nalgebra::Matrix3::new(0.0, 2.0 * a * p.y, 0.0, 0.0, 0.0, b, c * p.y, c * p.x, 0.0)
        };
        let xi = sample_point();
        let err_at = |curv: f64| {
            let coords = curved_element_coords(curv);
            let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
            let grad_xi = Tet10.shape_gradients(xi);
            let grad_x = grad_xi
                * (node_x.transpose() * grad_xi)
                    .try_inverse()
                    .expect("non-degenerate element");
            let nvals = Tet10.shape_functions(xi);
            let x_phys = (0..10).fold(Vec3::zeros(), |acc, i| acc + nvals[i] * coords[i]);
            let x_def = SMatrix::<f64, 10, 3>::from_fn(|i, j| (coords[i] + u(coords[i]))[j]);
            let f_recon = x_def.transpose() * grad_x;
            let f_analytic = nalgebra::Matrix3::identity() + grad_u(x_phys);
            (f_recon - f_analytic)
                .iter()
                .fold(0.0_f64, |m, &v| m.max(v.abs()))
        };
        let (e0, e_mid, e_hi) = (err_at(0.0), err_at(0.05), err_at(0.10));
        assert!(
            e0 < 1e-12,
            "straight element must reproduce the quadratic exactly: {e0:e}"
        );
        assert!(
            e_hi > e_mid && e_mid > e0,
            "reproduction error must grow monotonically with curvature: {e0:e} < {e_mid:e} < {e_hi:e}",
        );
        assert!(
            e_hi > 1e-3,
            "degradation must be physically meaningful at curv 0.10: {e_hi:e}",
        );
    }

    /// 1-D Gauss–Legendre nodes + weights on `[-1, 1]` (Newton on the Legendre
    /// recurrence) — the building block of the dense reference tet rule.
    // cast_precision_loss: the quadrature order `n` is ≤ 20 here, far inside
    // f64's exact-integer range — no precision is lost.
    #[allow(clippy::cast_precision_loss)]
    fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
            let mut dp = 0.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, x);
                for k in 2..=n {
                    let kf = k as f64;
                    let p2 = ((2.0 * kf - 1.0) * x * p1 - (kf - 1.0) * p0) / kf;
                    p0 = p1;
                    p1 = p2;
                }
                dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
                let dx = -p1 / dp;
                x += dx;
                if dx.abs() < 1e-15 {
                    break;
                }
            }
            out.push((x, 2.0 / ((1.0 - x * x) * dp * dp)));
        }
        out
    }

    /// A DENSE reference quadrature on the reference tet: an `n³` Gauss–Legendre
    /// tensor rule mapped through the Duffy transform. Converges to the exact
    /// integral of any smooth integrand — the ruler the 4-point Stroud rule is
    /// measured against for curved elements.
    fn dense_tet_rule(n: usize) -> Vec<(Vec3, f64)> {
        let gl = gauss_legendre(n);
        let mut rule = Vec::with_capacity(n * n * n);
        for &(tu, wu0) in &gl {
            let (u, wu) = (0.5 * (tu + 1.0), 0.5 * wu0);
            for &(tv, wv0) in &gl {
                let (v, wv) = (0.5 * (tv + 1.0), 0.5 * wv0);
                for &(tw, ww0) in &gl {
                    let (w, ww) = (0.5 * (tw + 1.0), 0.5 * ww0);
                    let xi = Vec3::new(u, v * (1.0 - u), w * (1.0 - u) * (1.0 - v));
                    let jac = (1.0 - u) * (1.0 - u) * (1.0 - v);
                    rule.push((xi, jac * wu * wv * ww));
                }
            }
        }
        rule
    }

    /// Quadrature adequacy (Tet10 ladder Q3). The 4-point Stroud rule is
    /// degree-2-exact only for a straight (affine) element; a curved element's
    /// `grad_x_n` and `detJ(ξ)` make the `K^e` integrand rational, so no fixed
    /// rule is exact. This gate MEASURES the 4-point-vs-dense-reference gap on
    /// `K^e` and commits the characterization:
    ///
    /// - EXACT on a straight element (4-pt reproduces the dense rule).
    /// - Grows ≈ linearly with curvature, `gap ≈ 0.22 · (sagitta/edge)`
    ///   (committed points below). This element bows ALL SIX edges by
    ///   `curv · edge`, so it is a CONSERVATIVE proxy for a boundary element,
    ///   whose ~3 boundary-face edges alone are curved.
    ///
    /// CONCLUSION: keep 4-pt (`G = 4`) — for a well-resolved boundary
    /// (sagitta/edge ≲ 2 %) the gap is ≲ 0.5 %, an order under the ~8 %
    /// demand-#1 / few-% ν = 0.49 modeling floors. Escalating the rule would
    /// change the const-generic `G` across the whole solver type; instead the
    /// future SDF-projection rung inherits the budget "keep boundary
    /// sagitta/edge modest, or escalate `G` for a strongly-curved surface".
    #[test]
    fn curved_quadrature_4pt_adequacy() {
        let dense = dense_tet_rule(16);
        // Self-check the reference rule against a KNOWN value: it must integrate
        // the reference-tet volume ∫ 1 dV = 1/6 (harness-validates the ruler).
        let wsum: f64 = dense.iter().map(|&(_, w)| w).sum();
        assert!(
            (wsum - 1.0 / 6.0).abs() < 1e-12,
            "reference rule Σw = {wsum}"
        );

        let gap = |curv: f64| {
            let c = curved_element_coords(curv);
            let kref = element_stiffness_with_rule(&c, &dense);
            (element_stiffness(&c) - &kref).norm() / kref.norm()
        };

        // (a) Straight element: 4-pt Stroud equals the dense rule exactly.
        assert!(gap(0.0) < 1e-12, "straight: 4-pt must match dense exactly");

        // (b) Committed growth (bands ±2 %; deterministic sqrt5 + GL-16 Newton).
        let (g1, g2, g5) = (gap(0.01), gap(0.02), gap(0.05));
        assert!(
            (2.20e-3..2.29e-3).contains(&g1),
            "gap(0.01) drifted: {g1:e}"
        );
        assert!(
            (4.39e-3..4.57e-3).contains(&g2),
            "gap(0.02) drifted: {g2:e}"
        );
        assert!(
            (1.10e-2..1.15e-2).contains(&g5),
            "gap(0.05) drifted: {g5:e}"
        );

        // (c) Monotone ~linear growth, and adequacy at a well-resolved boundary
        // curvature (≤ 2 % sagitta/edge → gap well under the modeling floor).
        assert!(g5 > g2 && g2 > g1, "gap must grow with curvature");
        assert!(
            g2 < 5.0e-3,
            "4-pt must stay adequate for mild curvature: {g2:e}"
        );
    }
}
