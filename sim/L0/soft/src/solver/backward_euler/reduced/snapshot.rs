//! Displacement snapshots — the training data a reduced basis is built from.

/// A set of free-DOF displacement snapshots, all of the same width.
///
/// A *snapshot* is `u = x − x_rest` restricted to the solver's **free** DOFs. Free,
/// not full, because the basis spans exactly the space the reduced solve works in:
/// pinned and roller-constrained DOFs are held at their `x_prev` values by the solver
/// and are not unknowns (see `CpuNewtonSolver::new`'s `free_dof_indices`, which
/// excludes both).
///
/// **Not mean-centred, deliberately.** Conventional POD subtracts the ensemble mean;
/// this does not. The rest state has to be exactly representable — `q = 0 ⇒ u = 0` —
/// because it is the initial condition of every trajectory the reduced model will be
/// asked to run. Centring would put the rest state at `q = −Φᵀū`, an arbitrary interior
/// point, and any error there shows up as a spurious force at `t = 0`.
#[derive(Clone, Debug, Default)]
pub struct SnapshotSet {
    n_free: usize,
    columns: Vec<Vec<f64>>,
}

impl SnapshotSet {
    /// Empty set over `n_free` free DOFs.
    #[must_use]
    pub const fn new(n_free: usize) -> Self {
        Self {
            n_free,
            columns: Vec::new(),
        }
    }

    /// Append one snapshot.
    ///
    /// # Panics
    /// Panics if `u.len() != n_free`. A width mismatch means the caller mixed meshes or
    /// boundary conditions, which would silently corrupt the basis.
    pub fn push(&mut self, u: &[f64]) {
        assert!(
            u.len() == self.n_free,
            "snapshot width {} does not match the set's free-DOF count {}",
            u.len(),
            self.n_free,
        );
        self.columns.push(u.to_vec());
    }

    /// Free-DOF count (the snapshot width).
    #[must_use]
    pub const fn n_free(&self) -> usize {
        self.n_free
    }

    /// Number of snapshots held.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.columns.len()
    }

    /// `true` when no snapshot has been pushed.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.columns.is_empty()
    }

    /// The snapshots, in insertion order.
    #[must_use]
    pub fn columns(&self) -> &[Vec<f64>] {
        &self.columns
    }

    /// Gather the free-DOF displacement `x − x_rest` from full-DOF vectors.
    ///
    /// `free_dof_indices` is the solver's own free-DOF map
    /// (`CpuNewtonSolver::free_dof_indices_scratch`); passing anything else produces a
    /// basis that does not line up with the solve.
    ///
    /// # Panics
    /// Panics if `x` and `x_rest` differ in length, or an index is out of range.
    #[must_use]
    pub fn free_displacement(x: &[f64], x_rest: &[f64], free_dof_indices: &[usize]) -> Vec<f64> {
        assert!(
            x.len() == x_rest.len(),
            "x ({}) and x_rest ({}) must have the same length",
            x.len(),
            x_rest.len(),
        );
        free_dof_indices.iter().map(|&i| x[i] - x_rest[i]).collect()
    }
}
