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

    /// Build the 30×30 small-strain stiffness of one Tet10 with the given
    /// nodal coordinates, integrating over the 4 Stroud points. Straight
    /// edges ⇒ affine map ⇒ constant Jacobian, so K^e is exact here.
    // Test elements are constructed non-degenerate, so a failed inverse is a
    // test-authoring bug worth panicking on immediately.
    #[allow(clippy::expect_used)]
    fn element_stiffness(coords: &[Vec3; 10]) -> DMatrix<f64> {
        let d_mat = elasticity_matrix(1.0, 0.3);
        let node_x = SMatrix::<f64, 10, 3>::from_fn(|i, j| coords[i][j]);
        let mut k_mat = DMatrix::<f64>::zeros(30, 30);

        for (xi, weight) in Tet10.gauss_points() {
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
}
