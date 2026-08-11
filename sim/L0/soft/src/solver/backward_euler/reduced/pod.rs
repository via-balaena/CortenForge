//! Proper-orthogonal-decomposition basis over displacement snapshots.

use nalgebra::{DMatrix, SymmetricEigen};

use super::snapshot::SnapshotSet;

/// Inner product the basis is orthonormal in.
///
/// Which one is better is an empirical question, so R1.0 measures both rather than
/// picking one on principle — see `docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md` §5.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Inner {
    /// Plain `uᵀv`. The simplest thing that can work.
    Euclidean,
    /// `uᵀMv` with the solver's **lumped diagonal** mass.
    ///
    /// Energy-consistent, and it makes the reduced mass matrix exactly the identity
    /// (`ΦᵀMΦ = I`), so the reduced tangent's `M/Δt²` term becomes `I/Δt²` — exact and
    /// free, with no projection needed. Cheap here only because the mass is diagonal.
    Mass,
}

/// A truncated POD basis: `r` orthonormal modes over `n_free` free DOFs.
#[derive(Clone, Debug)]
pub struct PodBasis {
    n_free: usize,
    /// Column-major modes, `r` columns of length `n_free`.
    modes: Vec<Vec<f64>>,
    /// Singular values of the (weighted) snapshot matrix, descending — **all** of them,
    /// not just the retained `r`, so the truncation can be re-judged without refitting.
    singular_values: Vec<f64>,
    inner: Inner,
    /// `sqrt(m_i)` per free DOF for [`Inner::Mass`]; empty for [`Inner::Euclidean`].
    sqrt_mass: Vec<f64>,
}

/// Why a basis could not be built.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PodError {
    /// No snapshots were supplied.
    Empty,
    /// A mass weight was non-positive or non-finite at the given free-DOF index.
    BadMass(usize),
    /// The mass slice was not exactly `n_free` long — most likely the solver's
    /// full-DOF `mass_per_dof` supplied where `mass_per_free_dof` was wanted.
    MassLen {
        /// Length supplied.
        got: usize,
        /// Length required (`n_free`).
        want: usize,
    },
    /// Every singular value was below the truncation floor — the snapshots carry no
    /// resolvable content (typically an all-zero ensemble).
    NoContent,
}

impl std::fmt::Display for PodError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => write!(f, "no snapshots supplied"),
            Self::BadMass(i) => write!(f, "non-positive or non-finite mass at free DOF {i}"),
            Self::MassLen { got, want } => write!(
                f,
                "mass slice has {got} entries but the basis spans {want} free DOFs \
                 (did a full-DOF mass_per_dof reach mass_per_free_dof?)"
            ),
            Self::NoContent => write!(f, "snapshot ensemble carries no resolvable content"),
        }
    }
}

impl std::error::Error for PodError {}

/// Relative floor on retained singular values.
///
/// ⚠ **This is set by the Gram route, not by taste.** Building the basis from
/// `G = UᵀU` (an `m × m` eigenproblem, cheap because `m ≪ n`) squares the singular
/// values: `σ` appears in `G` as `σ²`. So a mode with `σ/σ_max = 1e-9` lands at `1e-18`
/// in `G` and is indistinguishable from f64 round-off. Modes below this floor are
/// discarded rather than trusted. A direct thin SVD of `U` would resolve to ~`1e-16`
/// relative and is the upgrade if a basis ever needs modes that faint — nothing in the
/// `r ≤ 200` regime does.
const SIGMA_FLOOR_REL: f64 = 1e-8;

/// Per-free-DOF `sqrt(mass)` weights for `inner`, or empty for the Euclidean product.
///
/// In the mass inner product, POD of `u` under `uᵀMv` is ordinary POD of `w = M^{1/2}u`
/// under `wᵀv`; a lumped (diagonal) mass makes that a row scaling, which is the only
/// reason the mass product is affordable here.
///
/// # Errors
/// [`PodError::MassLen`] if the slice is not exactly `n_free` long,
/// [`PodError::BadMass`] if any weight is non-positive or non-finite.
fn weights_for(
    inner: Inner,
    mass_per_free_dof: &[f64],
    n_free: usize,
) -> Result<Vec<f64>, PodError> {
    match inner {
        Inner::Euclidean => Ok(Vec::new()),
        Inner::Mass => {
            // Exact length, in BOTH directions. Truncating an over-long slice would
            // silently accept the solver's full-DOF `mass_per_dof` in place of
            // `mass_per_free_dof` — a one-word slip between two similarly named
            // accessors — and build a wrong basis without complaint. Silent wrongness
            // is the failure mode this crate refuses everywhere else.
            if mass_per_free_dof.len() != n_free {
                return Err(PodError::MassLen {
                    got: mass_per_free_dof.len(),
                    want: n_free,
                });
            }
            let mut s = Vec::with_capacity(n_free);
            for (i, &mi) in mass_per_free_dof.iter().enumerate() {
                if !(mi.is_finite() && mi > 0.0) {
                    return Err(PodError::BadMass(i));
                }
                s.push(mi.sqrt());
            }
            Ok(s)
        }
    }
}

impl PodBasis {
    /// Fit a basis, retaining the smallest `r` that captures `energy_fraction` of the
    /// snapshot energy, capped at `max_modes`.
    ///
    /// ⚠ **Do not select `r` by `energy_fraction` alone — it is measured to mislead.**
    /// Retained energy is computed on the *training* snapshots and does not predict
    /// held-out error: on the R1.0 indentation fixture the 99.99 %-energy criterion
    /// picks `r = 6`, where held-out projection error is 2.7 %, while reaching 1 %
    /// needs `r = 40`. Pass `energy_fraction = 1.0` and control the size with
    /// `max_modes`, choosing it against **held-out** error (as
    /// `tests/reduced_pod_basis.rs` does). `energy_fraction` remains available for
    /// exploring a spectrum, which is what it is good for.
    ///
    /// `mass_per_free_dof` is consulted only for [`Inner::Mass`] and must be the lumped
    /// mass at each **free** DOF, in `free_dof_indices` order.
    ///
    /// # Errors
    /// [`PodError::Empty`] for an empty set, [`PodError::BadMass`] for a non-positive or
    /// non-finite mass weight, [`PodError::NoContent`] when nothing clears the
    /// `SIGMA_FLOOR_REL` truncation floor.
    pub fn fit(
        snapshots: &SnapshotSet,
        inner: Inner,
        mass_per_free_dof: &[f64],
        energy_fraction: f64,
        max_modes: usize,
    ) -> Result<Self, PodError> {
        if snapshots.is_empty() {
            return Err(PodError::Empty);
        }
        let n = snapshots.n_free();
        let m = snapshots.len();

        let sqrt_mass = weights_for(inner, mass_per_free_dof, n)?;
        let weight = |i: usize| -> f64 {
            match inner {
                Inner::Euclidean => 1.0,
                Inner::Mass => sqrt_mass[i],
            }
        };

        // Gram matrix G = WᵀW over the weighted snapshots (m × m, symmetric PSD).
        let cols = snapshots.columns();
        let mut gram = DMatrix::<f64>::zeros(m, m);
        for (a, col_a) in cols.iter().enumerate() {
            for (b, col_b) in cols.iter().enumerate().skip(a) {
                let acc: f64 = col_a
                    .iter()
                    .zip(col_b)
                    .enumerate()
                    .map(|(i, (va, vb))| {
                        let w = weight(i);
                        (w * va) * (w * vb)
                    })
                    .sum();
                gram[(a, b)] = acc;
                gram[(b, a)] = acc;
            }
        }

        // G = V Λ Vᵀ, eigenvalues ascending out of nalgebra; σ_k = sqrt(λ_k).
        let eig = SymmetricEigen::new(gram);
        let mut order: Vec<usize> = (0..m).collect();
        order.sort_by(|&a, &b| {
            eig.eigenvalues[b]
                .partial_cmp(&eig.eigenvalues[a])
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let singular_values: Vec<f64> = order
            .iter()
            .map(|&k| eig.eigenvalues[k].max(0.0).sqrt())
            .collect();
        let sigma_max = singular_values.first().copied().unwrap_or(0.0);
        if sigma_max <= 0.0 {
            return Err(PodError::NoContent);
        }

        // Retain by energy (σ², the snapshot energy) subject to the floor and the cap.
        let total: f64 = singular_values.iter().map(|s| s * s).sum();
        let mut kept = 0usize;
        let mut acc = 0.0;
        for (k, s) in singular_values.iter().enumerate() {
            if k >= max_modes || s / sigma_max < SIGMA_FLOOR_REL {
                break;
            }
            acc += s * s;
            kept = k + 1;
            if acc / total >= energy_fraction {
                break;
            }
        }
        if kept == 0 {
            return Err(PodError::NoContent);
        }

        // Φ = U V Λ^{-1/2}, un-weighted back to displacement space so `reconstruct`
        // returns a displacement, not a mass-scaled one.
        let mut modes = Vec::with_capacity(kept);
        for j in 0..kept {
            let k = order[j];
            let inv_sigma = 1.0 / singular_values[j];
            let mut phi = vec![0.0; n];
            for (a, col) in cols.iter().enumerate() {
                let vak = eig.eigenvectors[(a, k)];
                if vak == 0.0 {
                    continue;
                }
                for i in 0..n {
                    phi[i] += vak * col[i];
                }
            }
            // `Φ = U V Σ⁻¹`, with NO weight factor — for either inner product.
            //
            // The weighting is already inside `Σ` and `V`, because the Gram matrix was
            // built from the weighted snapshots: `G = ŨᵀŨ = UᵀMU`. So
            // `ΦᵀMΦ = Σ⁻¹Vᵀ(UᵀMU)VΣ⁻¹ = Σ⁻¹ΛΣ⁻¹ = I` exactly when `Φ` is left
            // unweighted. Re-applying `M^{1/2}` here would produce modes orthonormal in
            // neither product — caught by the R1.0 pilot, where it made every mass-inner
            // projection return zero and the error metric read a suspiciously exact 1.0.
            for p in &mut phi {
                *p *= inv_sigma;
            }
            modes.push(phi);
        }

        Ok(Self {
            n_free: n,
            modes,
            singular_values,
            inner,
            sqrt_mass,
        })
    }

    /// Retained mode count `r`.
    #[must_use]
    pub const fn n_modes(&self) -> usize {
        self.modes.len()
    }

    /// Free-DOF count the basis spans.
    #[must_use]
    pub const fn n_free(&self) -> usize {
        self.n_free
    }

    /// The inner product this basis is orthonormal in.
    #[must_use]
    pub const fn inner(&self) -> Inner {
        self.inner
    }

    /// All singular values, descending — including those truncated away.
    #[must_use]
    pub fn singular_values(&self) -> &[f64] {
        &self.singular_values
    }

    /// Fraction of snapshot energy the retained modes capture.
    #[must_use]
    pub fn retained_energy_fraction(&self) -> f64 {
        let total: f64 = self.singular_values.iter().map(|s| s * s).sum();
        if total <= 0.0 {
            return 0.0;
        }
        let kept: f64 = self
            .singular_values
            .iter()
            .take(self.modes.len())
            .map(|s| s * s)
            .sum();
        kept / total
    }

    /// `q = Φᵀ M u` (or `Φᵀ u` in the Euclidean inner product) — the coordinates of a
    /// **displacement**.
    ///
    /// For a residual or force use [`Self::project_covector`]; see its docs for why the
    /// two are different operations.
    ///
    /// # Panics
    /// Panics if `u.len() != n_free`.
    #[must_use]
    pub fn project(&self, u: &[f64]) -> Vec<f64> {
        assert!(u.len() == self.n_free, "u must have n_free entries");
        self.modes
            .iter()
            .map(|phi| match self.inner {
                Inner::Euclidean => phi.iter().zip(u).map(|(a, b)| a * b).sum(),
                Inner::Mass => phi
                    .iter()
                    .zip(u)
                    .zip(&self.sqrt_mass)
                    .map(|((a, b), s)| a * b * s * s)
                    .sum(),
            })
            .collect()
    }

    /// `Φᵀ f` — the **Galerkin projection of a force / residual**, with NO mass weight.
    ///
    /// ⚠ **This is not [`Self::project`], and the difference is not cosmetic.**
    /// A displacement is a vector: its coordinates in a mass-orthonormal basis are
    /// `q = ΦᵀMu`, which is what `project` computes. A residual is a covector (a force):
    /// the Galerkin condition tests it against the modes themselves, `Φᵀr = 0`, with no
    /// metric applied.
    ///
    /// Using `project` on a residual silently solves a different problem. The reduced
    /// Newton system pairs `Φᵀr` with the Jacobian `ΦᵀAΦ`; testing with `ΦᵀM` instead
    /// gives Jacobian `ΦᵀMAΦ`, so the search direction stops being consistent with the
    /// residual it is meant to reduce, and the line search stalls on the first step.
    /// That is precisely how this was found — see the R1.1 pilot.
    ///
    /// # Panics
    /// Panics if `f.len() != n_free`.
    #[must_use]
    pub fn project_covector(&self, f: &[f64]) -> Vec<f64> {
        assert!(f.len() == self.n_free, "f must have n_free entries");
        self.modes
            .iter()
            .map(|phi| phi.iter().zip(f).map(|(a, b)| a * b).sum())
            .collect()
    }

    /// `u ≈ Φ q`.
    ///
    /// # Panics
    /// Panics if `q.len() != n_modes`.
    #[must_use]
    pub fn reconstruct(&self, q: &[f64]) -> Vec<f64> {
        assert!(q.len() == self.modes.len(), "q must have n_modes entries");
        let mut u = vec![0.0; self.n_free];
        for (phi, &qk) in self.modes.iter().zip(q) {
            for (ui, pi) in u.iter_mut().zip(phi) {
                *ui += qk * pi;
            }
        }
        u
    }

    /// Relative projection error `‖u − ΦΦᵀu‖ / ‖u‖`, in the basis's own inner product.
    ///
    /// **This is the basis's ceiling**: no reduced solve can beat it, because it is the
    /// error of the best possible `q`. Returns `0.0` for a zero `u`.
    ///
    /// # Panics
    /// Panics if `u.len() != n_free`.
    #[must_use]
    pub fn projection_error(&self, u: &[f64]) -> f64 {
        let approx = self.reconstruct(&self.project(u));
        let norm = |v: &[f64]| -> f64 {
            match self.inner {
                Inner::Euclidean => v.iter().map(|a| a * a).sum::<f64>().sqrt(),
                Inner::Mass => v
                    .iter()
                    .zip(&self.sqrt_mass)
                    .map(|(a, s)| (a * s) * (a * s))
                    .sum::<f64>()
                    .sqrt(),
            }
        };
        let den = norm(u);
        if den <= 0.0 {
            return 0.0;
        }
        let diff: Vec<f64> = u.iter().zip(&approx).map(|(a, b)| a - b).collect();
        norm(&diff) / den
    }
}
