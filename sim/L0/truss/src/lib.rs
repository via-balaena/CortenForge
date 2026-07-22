//! # sim-truss
//!
//! A linear-elastic **pin-jointed truss** solver.
//!
//! Given a fixed geometry, a set of supports, and a fixed load, it assembles the
//! structural stiffness matrix `K(A)`, solves `K·u = f` for the nodal
//! displacements `u`, and reports the **compliance** `C = fᵀu` together with its
//! **exact analytic sensitivity** `∂C/∂Aₑ` to every strut's cross-section area.
//!
//! This crate is **Layer 0** — zero Bevy, zero ML-framework dependencies, pure
//! structural mechanics over `nalgebra`. It is the first *linear* finite-element
//! primitive in the workspace (`sim-core` is rigid-body dynamics; `sim-soft` is
//! nonlinear Neo-Hookean FEM), and it exists to be the **differentiable forward
//! model** behind lattice-as-design-var co-design: the per-strut areas are the
//! design variables, compliance (inverse stiffness) is the physical outcome, and
//! `∂C/∂Aₑ` is the gradient a `cf-codesign` optimizer consumes. It knows nothing
//! about optimization itself — it is the structural half only.
//!
//! ## The model
//!
//! Each strut is a two-force **bar element**: it carries axial force only, with
//! axial stiffness `Eₑ·Aₑ / Lₑ` (`E` Young's modulus, `A` cross-section area,
//! `L` length). Its contribution to the global stiffness is `Aₑ · k̂ₑ`, where the
//! *unit-area* element stiffness `k̂ₑ` is the rank-one outer product of the bar's
//! unit axis `d̂ₑ`,
//!
//! ```text
//! k̂ₑ (per node block) = (Eₑ / Lₑ) · d̂ₑ d̂ₑᵀ ,   assembled as [[B, −B], [−B, B]].
//! ```
//!
//! The construction is **dimension-generic** over `const D: usize` — the same
//! code solves 2-D ([`Truss2`]) and 3-D ([`Truss3`]) trusses; the mission's 3-D
//! octet lattice and a legible 2-D textbook frame are literally the same solver.
//!
//! ## Units
//!
//! The solver is **unit-agnostic**: it only requires that all inputs use one
//! consistent system. In SI, with `E` in pascals, areas in m², node coordinates
//! (hence lengths) in metres, and forces in newtons, displacements come out in
//! metres and compliance `C = fᵀu` in joules. Any other consistent system works
//! identically.
//!
//! ## Sign conventions and the sensitivity
//!
//! Displacements follow the applied load; compliance `C = fᵀu ≥ 0` measures how
//! much the structure gives under `f` (small `C` = stiff). Because `K` is
//! symmetric and the load is fixed, the compliance sensitivity is **self-adjoint**
//! and needs no adjoint solve:
//!
//! ```text
//! ∂C/∂Aₑ = −uₑᵀ k̂ₑ uₑ = −(Eₑ / Lₑ) · (d̂ₑ · (u_j − u_i))²  ≤  0,
//! ```
//!
//! i.e. adding material to any engaged strut never increases compliance. The
//! bound is exactly zero for a strut whose two ends move together along its axis
//! (it carries no strain). See [`Truss::compliance_sensitivity`].
//!
//! ## Failure is physical, not a panic
//!
//! Driving areas toward zero eventually turns the structure into a **mechanism**
//! — a free node that can displace with no strain energy — and `K` stops being
//! positive-definite. The Cholesky solve then fails and returns
//! [`TrussError::SingularStiffness`] rather than a meaningless displacement,
//! marking the feasibility boundary a co-design optimizer must respect. (The
//! factorization tests *numerical* positive-definiteness, so it is the mechanism
//! signal in all but the ill-conditioned boundary case; see that error.)
//!
//! ## Example
//!
//! A symmetric two-bar truss (a "trestle"): two base nodes pinned, an apex node
//! pushed straight down. The apex deflection and compliance both have a closed
//! form, `Δ = P·L / (2·E·A·sin²θ)` and `C = P·Δ`.
//!
//! ```
//! use nalgebra::vector;
//! use sim_truss::{Load, Strut, Support, Truss2};
//!
//! let (b, h, e, a, p) = (1.0_f64, 1.0_f64, 210e9, 1e-4, 1000.0);
//! let truss = Truss2::new(
//!     vec![vector![-b, 0.0], vector![b, 0.0], vector![0.0, h]],
//!     vec![
//!         Strut { i: 0, j: 2, youngs_modulus: e },
//!         Strut { i: 1, j: 2, youngs_modulus: e },
//!     ],
//!     vec![
//!         Support { node: 0, fixed: [true, true] },
//!         Support { node: 1, fixed: [true, true] },
//!     ],
//!     vec![Load { node: 2, force: vector![0.0, -p] }],
//! )
//! .unwrap();
//!
//! let (c, dc_da) = truss.compliance_sensitivity(&[a, a]).unwrap();
//!
//! let l = (b * b + h * h).sqrt();
//! let sin_theta = h / l;
//! let expected_c = p * p * l / (2.0 * e * a * sin_theta * sin_theta);
//! assert!((c - expected_c).abs() / expected_c < 1e-12);
//! // Adding material to a load-bearing strut reduces compliance.
//! assert!(dc_da.iter().all(|&g| g < 0.0));
//! ```

// A public numerical primitive: the library body must never silently `unwrap` /
// `expect` (a panic in a solver is a denial of service to its consumer). Tests
// opt back out — see the `#[allow]` on the `tests` module.
#![deny(clippy::unwrap_used, clippy::expect_used)]

mod error;

pub use error::TrussError;

use nalgebra::{DMatrix, DVector, SMatrix, SVector};

/// A single bar element: the two node indices it connects and its Young's
/// modulus.
///
/// The cross-section **area is not stored here** — it is the design variable,
/// supplied per-solve to [`Truss::compliance_sensitivity`] and friends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Strut {
    /// Index of the first endpoint node.
    pub i: usize,
    /// Index of the second endpoint node.
    pub j: usize,
    /// Young's modulus `E` of the strut material (force / area).
    pub youngs_modulus: f64,
}

/// A support (Dirichlet boundary condition): the named node has each axis for
/// which `fixed[axis]` is `true` held at zero displacement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Support<const D: usize> {
    /// Index of the supported node.
    pub node: usize,
    /// Per-axis constraint flags; a `true` axis is pinned (zero displacement).
    pub fixed: [bool; D],
}

/// An applied nodal load: a force vector added to the named node's degrees of
/// freedom.
///
/// A component acting on an axis that is pinned by a [`Support`] is absorbed by
/// the support reaction — it does no work (that displacement is zero) and
/// contributes nothing to the compliance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Load<const D: usize> {
    /// Index of the loaded node.
    pub node: usize,
    /// The applied force vector.
    pub force: SVector<f64, D>,
}

/// The solved state of a truss at a given set of areas.
#[derive(Debug, Clone, PartialEq)]
pub struct TrussState<const D: usize> {
    /// Nodal displacements, one vector per node (fixed axes are exactly zero).
    pub displacements: Vec<SVector<f64, D>>,
    /// The compliance `C = fᵀu` of this solved state.
    pub compliance: f64,
}

/// Precomputed geometry of one strut (fixed with the node coordinates).
#[derive(Debug, Clone, Copy)]
struct Element<const D: usize> {
    i: usize,
    j: usize,
    /// Rest length `L = ‖x_j − x_i‖`.
    length: f64,
    /// Unit axis `d̂ = (x_j − x_i) / L`.
    axis: SVector<f64, D>,
    youngs_modulus: f64,
}

/// A linear-elastic pin-jointed truss in `D` dimensions.
///
/// Build one with [`Truss::new`]; the geometry, supports, and load are fixed for
/// the lifetime of the value, and only the per-strut areas vary. Use
/// [`Truss2`] / [`Truss3`] for the common 2-D and 3-D cases.
#[derive(Debug, Clone)]
pub struct Truss<const D: usize> {
    nodes: Vec<SVector<f64, D>>,
    elements: Vec<Element<D>>,
    /// Global DOF indices (`node * D + axis`) that are *free* (not supported),
    /// in ascending order — the reduced system's rows/columns.
    free_dofs: Vec<usize>,
    /// The full load vector over all `n_nodes * D` degrees of freedom.
    load: DVector<f64>,
}

impl<const D: usize> Truss<D> {
    /// Build a truss from node coordinates, struts, supports, and loads.
    ///
    /// # Errors
    ///
    /// Returns a [`TrussError`] if any strut/support/load names an out-of-range
    /// node ([`NodeIndexOutOfBounds`](TrussError::NodeIndexOutOfBounds)), a strut
    /// is a self-loop ([`SelfLoopStrut`](TrussError::SelfLoopStrut)), has
    /// coinciding endpoints ([`DegenerateStrut`](TrussError::DegenerateStrut)),
    /// or has a non-finite, non-positive Young's modulus
    /// ([`InvalidModulus`](TrussError::InvalidModulus)).
    pub fn new(
        nodes: Vec<SVector<f64, D>>,
        struts: Vec<Strut>,
        supports: Vec<Support<D>>,
        loads: Vec<Load<D>>,
    ) -> Result<Self, TrussError> {
        let n_nodes = nodes.len();
        let bound = |index: usize| {
            (index < n_nodes)
                .then_some(())
                .ok_or(TrussError::NodeIndexOutOfBounds { index, n_nodes })
        };

        // Precompute per-strut geometry, validating connectivity as we go.
        let mut elements = Vec::with_capacity(struts.len());
        for (e, strut) in struts.into_iter().enumerate() {
            bound(strut.i)?;
            bound(strut.j)?;
            if strut.i == strut.j {
                return Err(TrussError::SelfLoopStrut {
                    strut: e,
                    node: strut.i,
                });
            }
            if strut.youngs_modulus <= 0.0 || !strut.youngs_modulus.is_finite() {
                return Err(TrussError::InvalidModulus {
                    strut: e,
                    youngs_modulus: strut.youngs_modulus,
                });
            }
            let delta = nodes[strut.j] - nodes[strut.i];
            let length = delta.norm();
            if length == 0.0 {
                return Err(TrussError::DegenerateStrut { strut: e });
            }
            elements.push(Element {
                i: strut.i,
                j: strut.j,
                length,
                axis: delta / length,
                youngs_modulus: strut.youngs_modulus,
            });
        }

        // Mark supported DOFs, then collect the free ones in ascending order.
        let mut fixed = vec![false; n_nodes * D];
        for support in supports {
            bound(support.node)?;
            for (axis, &is_fixed) in support.fixed.iter().enumerate() {
                if is_fixed {
                    fixed[support.node * D + axis] = true;
                }
            }
        }
        let free_dofs: Vec<usize> = (0..n_nodes * D).filter(|&d| !fixed[d]).collect();

        // Assemble the full load vector.
        let mut load = DVector::zeros(n_nodes * D);
        for l in loads {
            bound(l.node)?;
            for axis in 0..D {
                load[l.node * D + axis] += l.force[axis];
            }
        }

        Ok(Self {
            nodes,
            elements,
            free_dofs,
            load,
        })
    }

    /// The number of nodes.
    #[must_use]
    pub const fn n_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// The number of struts (the length of the area vector [`solve`](Self::solve)
    /// and friends expect).
    #[must_use]
    pub const fn n_struts(&self) -> usize {
        self.elements.len()
    }

    /// The rest length `Lₑ` of each strut, in strut order.
    #[must_use]
    pub fn lengths(&self) -> Vec<f64> {
        self.elements.iter().map(|e| e.length).collect()
    }

    /// The structural volume `Σ Aₑ·Lₑ` at the given areas — a mass proxy (at unit
    /// density) a consumer can penalize to trade stiffness against material.
    ///
    /// # Errors
    ///
    /// Returns [`AreaCountMismatch`](TrussError::AreaCountMismatch) if `areas`
    /// does not have one entry per strut, or
    /// [`InvalidArea`](TrussError::InvalidArea) if any area is not finite `> 0`.
    pub fn volume(&self, areas: &[f64]) -> Result<f64, TrussError> {
        self.check_areas(areas)?;
        Ok(self
            .elements
            .iter()
            .zip(areas)
            .map(|(e, &a)| a * e.length)
            .sum())
    }

    /// Solve `K(A)·u = f` and return the nodal displacements and compliance.
    ///
    /// # Errors
    ///
    /// Returns [`AreaCountMismatch`](TrussError::AreaCountMismatch) /
    /// [`InvalidArea`](TrussError::InvalidArea) for an invalid area
    /// vector, or [`SingularStiffness`](TrussError::SingularStiffness) if the
    /// design is a mechanism (`K` is not positive-definite over the free DOFs).
    pub fn solve(&self, areas: &[f64]) -> Result<TrussState<D>, TrussError> {
        self.check_areas(areas)?;

        let n_free = self.free_dofs.len();
        // A fully-supported truss has nothing to solve: u ≡ 0, C = 0.
        if n_free == 0 {
            return Ok(TrussState {
                displacements: vec![SVector::zeros(); self.nodes.len()],
                compliance: 0.0,
            });
        }

        let k_reduced = self.assemble_reduced_stiffness(areas);
        let f_reduced =
            DVector::from_iterator(n_free, self.free_dofs.iter().map(|&d| self.load[d]));

        // Cholesky succeeds iff K is (numerically) symmetric positive-definite;
        // its failure signals a mechanism — or, at the ill-conditioned boundary,
        // a PD-but-near-singular K (see `TrussError::SingularStiffness`).
        let chol = nalgebra::Cholesky::new(k_reduced).ok_or(TrussError::SingularStiffness)?;
        let u_reduced = chol.solve(&f_reduced);

        // Scatter the free-DOF displacements back into per-node vectors.
        let mut displacements = vec![SVector::<f64, D>::zeros(); self.nodes.len()];
        for (slot, &dof) in self.free_dofs.iter().enumerate() {
            displacements[dof / D][dof % D] = u_reduced[slot];
        }
        let compliance = f_reduced.dot(&u_reduced);

        Ok(TrussState {
            displacements,
            compliance,
        })
    }

    /// The compliance `C = fᵀu` at the given areas.
    ///
    /// # Errors
    ///
    /// As [`solve`](Self::solve).
    pub fn compliance(&self, areas: &[f64]) -> Result<f64, TrussError> {
        Ok(self.solve(areas)?.compliance)
    }

    /// The compliance and its exact sensitivity `∂C/∂Aₑ` to every strut area.
    ///
    /// The gradient is self-adjoint (no second solve): for each strut,
    /// `∂C/∂Aₑ = −(Eₑ / Lₑ) · (d̂ₑ · (u_j − u_i))² ≤ 0`. It is validated against
    /// central finite differences in the crate's tests.
    ///
    /// # Errors
    ///
    /// As [`solve`](Self::solve).
    pub fn compliance_sensitivity(&self, areas: &[f64]) -> Result<(f64, Vec<f64>), TrussError> {
        let state = self.solve(areas)?;
        let grad = self
            .elements
            .iter()
            .map(|e| {
                let elongation = e
                    .axis
                    .dot(&(state.displacements[e.j] - state.displacements[e.i]));
                -(e.youngs_modulus / e.length) * elongation * elongation
            })
            .collect();
        Ok((state.compliance, grad))
    }

    /// Validate an area vector: one strictly-positive entry per strut.
    fn check_areas(&self, areas: &[f64]) -> Result<(), TrussError> {
        if areas.len() != self.elements.len() {
            return Err(TrussError::AreaCountMismatch {
                expected: self.elements.len(),
                got: areas.len(),
            });
        }
        for (strut, &area) in areas.iter().enumerate() {
            // Areas must be finite and strictly positive. `≤ 0` catches zero and
            // negatives; `!is_finite()` catches `NaN` (which compares false to
            // everything, so a bare `≤ 0` would miss it) and `±∞` (which would
            // otherwise pass and poison the stiffness matrix).
            if area <= 0.0 || !area.is_finite() {
                return Err(TrussError::InvalidArea { strut, area });
            }
        }
        Ok(())
    }

    /// Assemble the stiffness matrix restricted to the free degrees of freedom.
    ///
    /// The full matrix is `Σₑ Aₑ · k̂ₑ`; we build only the `n_free × n_free`
    /// reduced block directly, indexing through `free_slot`.
    fn assemble_reduced_stiffness(&self, areas: &[f64]) -> DMatrix<f64> {
        let n_free = self.free_dofs.len();
        // Inverse of `free_dofs`: global DOF → reduced index (or `usize::MAX` if
        // supported, i.e. dropped from the reduced system).
        let mut free_slot = vec![usize::MAX; self.nodes.len() * D];
        for (slot, &dof) in self.free_dofs.iter().enumerate() {
            free_slot[dof] = slot;
        }

        let mut stiffness = DMatrix::zeros(n_free, n_free);
        for (elem, &area) in self.elements.iter().zip(areas) {
            // Unit-area node block B = (E/L)·d̂ d̂ᵀ, scaled by this strut's area.
            let block: SMatrix<f64, D, D> =
                (area * elem.youngs_modulus / elem.length) * (elem.axis * elem.axis.transpose());
            // The 2×2 (in node-blocks) element stiffness is [[B, −B], [−B, B]].
            // Scatter each node-block pair, skipping any supported DOF.
            for (node_r, sign_r) in [(elem.i, 1.0), (elem.j, -1.0)] {
                for (node_c, sign_c) in [(elem.i, 1.0), (elem.j, -1.0)] {
                    let coupling = sign_r * sign_c; // +1 on the diagonal blocks, −1 off.
                    for row_axis in 0..D {
                        let row = free_slot[node_r * D + row_axis];
                        if row == usize::MAX {
                            continue;
                        }
                        for col_axis in 0..D {
                            let col = free_slot[node_c * D + col_axis];
                            if col == usize::MAX {
                                continue;
                            }
                            stiffness[(row, col)] += coupling * block[(row_axis, col_axis)];
                        }
                    }
                }
            }
        }
        stiffness
    }
}

/// A 2-D truss (`D = 2`).
pub type Truss2 = Truss<2>;
/// A 3-D truss (`D = 3`).
pub type Truss3 = Truss<3>;

#[cfg(test)]
// Tests assert against hand-computed closed forms and exercise the error paths,
// so `.unwrap()`/`.unwrap_err()` on known-good/known-bad inputs, exact float
// comparisons of exactly-representable results, and short single-letter names
// mirroring the closed-form variables (b, h, E, A, P, L) are intentional here.
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::many_single_char_names)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::vector;

    /// The symmetric two-bar "trestle" of the crate example, with its closed
    /// form. Returns `(truss, area, expected_compliance, expected_apex_drop)`.
    fn trestle(b: f64, h: f64, e: f64, a: f64, p: f64) -> (Truss2, f64, f64, f64) {
        let truss = Truss2::new(
            vec![vector![-b, 0.0], vector![b, 0.0], vector![0.0, h]],
            vec![
                Strut {
                    i: 0,
                    j: 2,
                    youngs_modulus: e,
                },
                Strut {
                    i: 1,
                    j: 2,
                    youngs_modulus: e,
                },
            ],
            vec![
                Support {
                    node: 0,
                    fixed: [true, true],
                },
                Support {
                    node: 1,
                    fixed: [true, true],
                },
            ],
            vec![Load {
                node: 2,
                force: vector![0.0, -p],
            }],
        )
        .unwrap();
        let l = b.hypot(h);
        let sin_theta = h / l;
        let drop = p * l / (2.0 * e * a * sin_theta * sin_theta);
        (truss, a, p * drop, drop)
    }

    #[test]
    fn trestle_matches_closed_form() {
        let (truss, a, expected_c, expected_drop) = trestle(1.0, 1.0, 210e9, 1e-4, 1000.0);
        let state = truss.solve(&[a, a]).unwrap();
        // Apex (node 2) moves straight down by the closed-form amount; base
        // nodes stay pinned.
        assert_relative_eq!(state.displacements[2][0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(
            state.displacements[2][1],
            -expected_drop,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            state.displacements[0],
            SVector::<f64, 2>::zeros(),
            epsilon = 1e-18
        );
        assert_relative_eq!(state.compliance, expected_c, max_relative = 1e-12);
    }

    #[test]
    fn rescaled_trestle_matches_closed_form() {
        // A different aspect ratio, modulus, area, and load. Still a left-right
        // symmetric configuration (base at ±b), so the same closed form applies —
        // this checks the parameter scaling, not an asymmetric deflection (see
        // `asymmetric_truss_sensitivity_matches_fd` for that).
        let (truss, a, expected_c, _) = trestle(2.0, 0.7, 7e10, 3e-4, 450.0);
        assert_relative_eq!(
            truss.compliance(&[a, a]).unwrap(),
            expected_c,
            max_relative = 1e-12
        );
    }

    /// A genuinely asymmetric truss: three fixed anchors at irregular positions
    /// carry one free node loaded off-axis, so it deflects in *both* x and y and
    /// the three struts have distinct directions, lengths, and moduli. No closed
    /// form — validated by the physics invariants plus central-FD sensitivity.
    fn asymmetric_truss() -> Truss2 {
        Truss2::new(
            vec![
                vector![0.0, 0.0],
                vector![2.0, 0.3],
                vector![0.4, 1.6],
                vector![1.1, 0.7],
            ],
            vec![
                Strut {
                    i: 0,
                    j: 3,
                    youngs_modulus: 9.0e10,
                },
                Strut {
                    i: 1,
                    j: 3,
                    youngs_modulus: 7.0e10,
                },
                Strut {
                    i: 2,
                    j: 3,
                    youngs_modulus: 12.0e10,
                },
            ],
            vec![
                Support {
                    node: 0,
                    fixed: [true, true],
                },
                Support {
                    node: 1,
                    fixed: [true, true],
                },
                Support {
                    node: 2,
                    fixed: [true, true],
                },
            ],
            vec![Load {
                node: 3,
                force: vector![300.0, -500.0],
            }],
        )
        .unwrap()
    }

    #[test]
    fn asymmetric_truss_deflects_in_two_dimensions() {
        let truss = asymmetric_truss();
        let state = truss.solve(&[1.3e-4, 0.9e-4, 1.7e-4]).unwrap();
        // Unlike the symmetric trestle (u_x ≡ 0), the free node moves in both
        // axes — the direction transform is genuinely exercised.
        assert!(state.displacements[3][0].abs() > 1e-9);
        assert!(state.displacements[3][1].abs() > 1e-9);
        assert!(state.compliance > 0.0);
    }

    #[test]
    fn asymmetric_sensitivity_matches_central_fd() {
        let truss = asymmetric_truss();
        let areas = [1.3e-4, 0.9e-4, 1.7e-4];
        let (_, grad) = truss.compliance_sensitivity(&areas).unwrap();

        let eps = 1e-9;
        for k in 0..truss.n_struts() {
            let mut plus = areas;
            let mut minus = areas;
            plus[k] += eps;
            minus[k] -= eps;
            let fd = (truss.compliance(&plus).unwrap() - truss.compliance(&minus).unwrap())
                / (2.0 * eps);
            assert_relative_eq!(grad[k], fd, max_relative = 1e-6);
            assert!(grad[k] < 0.0, "strut {k} sensitivity should be negative");
        }
    }

    #[test]
    fn sensitivity_matches_central_fd() {
        // Distinct areas so the two struts have independent, unequal gradients.
        let (truss, _, _, _) = trestle(1.3, 0.9, 9e10, 1e-4, 800.0);
        let areas = [1.7e-4, 0.8e-4];
        let (_, grad) = truss.compliance_sensitivity(&areas).unwrap();

        let eps = 1e-9;
        for k in 0..truss.n_struts() {
            let mut plus = areas;
            let mut minus = areas;
            plus[k] += eps;
            minus[k] -= eps;
            let fd = (truss.compliance(&plus).unwrap() - truss.compliance(&minus).unwrap())
                / (2.0 * eps);
            assert_relative_eq!(grad[k], fd, max_relative = 1e-6);
            // More material never increases compliance.
            assert!(grad[k] < 0.0, "strut {k} sensitivity should be negative");
        }
    }

    #[test]
    fn compliance_decreases_as_area_grows() {
        // Doubling every area halves compliance (K is linear in A, so C = fᵀK⁻¹f
        // scales as 1/A under a uniform scale).
        let (truss, a, _, _) = trestle(1.0, 1.0, 210e9, 1e-4, 1000.0);
        let c1 = truss.compliance(&[a, a]).unwrap();
        let c2 = truss.compliance(&[2.0 * a, 2.0 * a]).unwrap();
        assert_relative_eq!(c2, c1 / 2.0, max_relative = 1e-12);
    }

    #[test]
    fn axial_bar_in_3d() {
        // A single bar along +z, one end pinned, the other free only in z
        // (its x,y are supported to avoid a transverse mechanism). Pure axial:
        // u_z = F·L / (E·A).
        let (l, e, a, f) = (2.0_f64, 210e9, 5e-4, 3000.0);
        let truss = Truss3::new(
            vec![vector![0.0, 0.0, 0.0], vector![0.0, 0.0, l]],
            vec![Strut {
                i: 0,
                j: 1,
                youngs_modulus: e,
            }],
            vec![
                Support {
                    node: 0,
                    fixed: [true, true, true],
                },
                Support {
                    node: 1,
                    fixed: [true, true, false],
                },
            ],
            vec![Load {
                node: 1,
                force: vector![0.0, 0.0, f],
            }],
        )
        .unwrap();
        let state = truss.solve(&[a]).unwrap();
        assert_relative_eq!(
            state.displacements[1][2],
            f * l / (e * a),
            max_relative = 1e-12
        );
        assert_relative_eq!(state.compliance, f * f * l / (e * a), max_relative = 1e-12);
    }

    #[test]
    fn mechanism_reports_singular_stiffness() {
        // A single 2-D bar with a fully-free far node: the node can swing
        // transverse to the bar with no strain energy → K is singular.
        let truss = Truss2::new(
            vec![vector![0.0, 0.0], vector![1.0, 0.0]],
            vec![Strut {
                i: 0,
                j: 1,
                youngs_modulus: 210e9,
            }],
            vec![Support {
                node: 0,
                fixed: [true, true],
            }],
            vec![Load {
                node: 1,
                force: vector![0.0, -100.0],
            }],
        )
        .unwrap();
        assert_eq!(
            truss.solve(&[1e-4]).unwrap_err(),
            TrussError::SingularStiffness
        );
    }

    #[test]
    fn fully_supported_truss_has_zero_compliance() {
        let truss = Truss2::new(
            vec![vector![0.0, 0.0], vector![1.0, 0.0]],
            vec![Strut {
                i: 0,
                j: 1,
                youngs_modulus: 210e9,
            }],
            vec![
                Support {
                    node: 0,
                    fixed: [true, true],
                },
                Support {
                    node: 1,
                    fixed: [true, true],
                },
            ],
            vec![Load {
                node: 1,
                force: vector![5.0, 0.0],
            }],
        )
        .unwrap();
        let state = truss.solve(&[1e-4]).unwrap();
        assert_eq!(state.compliance, 0.0);
        assert!(
            state
                .displacements
                .iter()
                .all(|u| *u == SVector::<f64, 2>::zeros())
        );
    }

    #[test]
    fn volume_is_area_length_sum() {
        let (truss, _, _, _) = trestle(1.0, 1.0, 210e9, 1e-4, 1000.0);
        let l = 2.0_f64.sqrt(); // each bar spans (±1,0)→(0,1)
        assert_relative_eq!(
            truss.volume(&[2e-4, 3e-4]).unwrap(),
            2e-4 * l + 3e-4 * l,
            max_relative = 1e-12
        );
    }

    #[test]
    fn construction_and_area_errors() {
        let ok_nodes = vec![vector![0.0, 0.0], vector![1.0, 0.0]];
        let ok_strut = Strut {
            i: 0,
            j: 1,
            youngs_modulus: 1.0,
        };

        // Out-of-range strut node.
        assert!(matches!(
            Truss2::new(
                ok_nodes.clone(),
                vec![Strut {
                    i: 0,
                    j: 9,
                    youngs_modulus: 1.0
                }],
                vec![],
                vec![],
            ),
            Err(TrussError::NodeIndexOutOfBounds {
                index: 9,
                n_nodes: 2
            })
        ));
        // Self-loop.
        assert!(matches!(
            Truss2::new(
                ok_nodes.clone(),
                vec![Strut {
                    i: 1,
                    j: 1,
                    youngs_modulus: 1.0
                }],
                vec![],
                vec![],
            ),
            Err(TrussError::SelfLoopStrut { strut: 0, node: 1 })
        ));
        // Coincident endpoints.
        assert!(matches!(
            Truss2::new(
                vec![vector![0.0, 0.0], vector![0.0, 0.0]],
                vec![ok_strut],
                vec![],
                vec![],
            ),
            Err(TrussError::DegenerateStrut { strut: 0 })
        ));

        // Non-finite / non-positive Young's modulus, rejected at construction
        // (mirrors the area guard rather than surfacing later as a mechanism).
        for bad_e in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(matches!(
                Truss2::new(
                    ok_nodes.clone(),
                    vec![Strut {
                        i: 0,
                        j: 1,
                        youngs_modulus: bad_e
                    }],
                    vec![],
                    vec![],
                ),
                Err(TrussError::InvalidModulus { strut: 0, .. })
            ));
        }

        let truss = Truss2::new(ok_nodes, vec![ok_strut], vec![], vec![]).unwrap();
        assert!(matches!(
            truss.compliance(&[1.0, 2.0]),
            Err(TrussError::AreaCountMismatch {
                expected: 1,
                got: 2
            })
        ));
        // Zero, negative, NaN, and +∞ areas are all rejected as InvalidArea.
        for bad_a in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(matches!(
                truss.compliance(&[bad_a]),
                Err(TrussError::InvalidArea { strut: 0, .. })
            ));
        }
    }
}
