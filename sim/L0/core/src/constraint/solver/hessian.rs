//! Newton's sparse Hessian, for models of MuJoCo's sparse size.
//!
//! [`SparseHessian`] holds `H = M + J'·D·J` plus the cone contacts'
//! contributions in CSC storage, factored as `L·D·L'`. MuJoCo factors and
//! updates its own sparse Hessian (`MakeHessian`, `FactorizeHessian`,
//! `HessianIncremental`, `engine_solver.c:1516-1807`); this one is made anew
//! at each solver iteration, the same matrix rounded otherwise.

use crate::constraint::impedance::MJ_MINVAL;
use crate::integrate::implicit::{
    isd_damping, isd_stiffness, isd_tendon_damping, isd_tendon_stiffness, tendon_active_stiffness,
    tendon_all_dofs_sleeping,
};
use crate::types::{ConstraintState, ConstraintType, Data, ENABLE_SLEEP, Model, StepError};

/// A cone contact's first row, its dim and its cone Hessian (row-major).
pub type Cone = (usize, usize, [f64; 36]);

/// Sparse Hessian `H = M + J'·D·J + Σ J_c'·H_c·J_c` in CSC lower-triangle
/// format with cached symbolic/numeric `L·D·L'` factorization.
///
/// The sparsity pattern comes from:
/// - M: tree sparsity via `dof_parent` (entry (i,j) iff j is ancestor of i)
/// - J'·J over every constraint row, whatever its state, an elliptic
///   contact's rows together, as MuJoCo's pattern (`mju_sqrMatTDSparseCount`
///   over all rows), so a row that becomes quadratic or a cone later in the
///   solve has its entries
/// - under the implicit spring-damper integrator, each stiff or damped
///   tendon's `J'·J`
///
/// Symbolic factorization (elimination tree + L structure) is computed once
/// per `assemble` call; numeric factorization at each `refactor`. Solve is
/// forward/diagonal/back substitution.
pub struct SparseHessian {
    nv: usize,
    /// CSC column pointers for lower triangle of H (length nv+1).
    col_ptr: Vec<usize>,
    /// CSC row indices (length nnz). Sorted within each column.
    row_idx: Vec<usize>,
    /// CSC values (length nnz).
    vals: Vec<f64>,
    /// Elimination tree: `parent[j]` = parent of column j in etree, or None for root.
    etree: Vec<Option<usize>>,
    /// CSC column pointers for L factor (length nv+1).
    l_col_ptr: Vec<usize>,
    /// CSC row indices for L factor.
    l_row_idx: Vec<usize>,
    /// Numeric values of L factor (unit lower triangular: `L[j,j]` = 1, not stored).
    l_vals: Vec<f64>,
    /// Diagonal D from LDL^T factorization (length nv).
    l_diag: Vec<f64>,
}

impl SparseHessian {
    /// Build the sparse Hessian from Model/Data. Computes:
    /// 1. Sparsity pattern of H = M + J^T·D·J (CSC lower triangle)
    /// 2. Numeric values
    /// 3. Symbolic factorization (elimination tree + L structure)
    /// 4. Numeric LDL^T factorization
    // Indexed loop mutates parallel arrays at the same index; iterator forms would obscure the per-row update.
    #[allow(clippy::needless_range_loop)]
    pub fn assemble(
        model: &Model,
        data: &Data,
        nv: usize,
        implicit_sd: bool,
        state: &[ConstraintState],
        cones: &[Cone],
    ) -> Result<Self, StepError> {
        let nefc = data.efc_type.len();

        // --- Step 1: Determine sparsity pattern ---
        // A dense boolean mask per column: O(nv²), from 3,600 entries at the
        // sparse size up.
        let mut has_entry = vec![vec![false; nv]; nv]; // has_entry[col][row], row >= col

        // M sparsity: tree structure from dof_parent
        for i in 0..nv {
            has_entry[i][i] = true; // diagonal always present
            let mut p = model.dof_parent[i];
            while let Some(j) = p {
                // M[i,j] is non-zero (j < i since j is ancestor)
                has_entry[j][i] = true; // lower triangle: row=i, col=j
                p = model.dof_parent[j];
            }
        }

        // J'·J sparsity, whatever each row's state (MuJoCo's pattern): the
        // outer product of a row's non-zero columns, an elliptic contact's
        // rows taken together, since its cone term `J_c'·H_c·J_c` pairs a
        // column of one row with a column of another (MuJoCo gives every row
        // of a contact one pattern, `engine_solver.c:1698`).
        let mut r = 0;
        while r < nefc {
            let rows = if data.efc_type[r] == ConstraintType::ContactElliptic {
                data.efc_dim[r]
            } else {
                1
            };
            let nz_cols: Vec<usize> = (0..nv)
                .filter(|&col| (r..r + rows).any(|k| data.efc_J[(k, col)] != 0.0))
                .collect();
            // Mark all pairs (lower triangle)
            for &ci in &nz_cols {
                for &cj in &nz_cols {
                    let (lo, hi) = if ci <= cj { (ci, cj) } else { (cj, ci) };
                    has_entry[lo][hi] = true;
                }
            }
            r += rows;
        }

        // DT-35: Tendon K/D sparsity (ImplicitSpringDamper only).
        // Conservative: includes entries for all tendons with k or b nonzero,
        // regardless of deadband state. Actual values use deadband-aware k_active.
        if implicit_sd {
            let mut nz: Vec<usize> = Vec::with_capacity(8);
            for t in 0..model.ntendon {
                let kt = model.tendon_stiffness[t];
                let bt = model.tendon_damping[t];
                if kt == 0.0 && bt == 0.0 {
                    continue;
                }
                let j = &data.ten_J[t];
                nz.clear();
                for dof in 0..nv {
                    if j[dof] != 0.0 {
                        nz.push(dof);
                    }
                }
                for &ci in &nz {
                    for &cj in &nz {
                        let (lo, hi) = if ci <= cj { (ci, cj) } else { (cj, ci) };
                        has_entry[lo][hi] = true;
                    }
                }
            }
        }

        // --- Step 2: Build CSC arrays ---
        let mut col_ptr = vec![0usize; nv + 1];
        let mut row_idx_vec = Vec::new();
        let mut vals_vec = Vec::new();

        for col in 0..nv {
            col_ptr[col] = row_idx_vec.len();
            for row in col..nv {
                if has_entry[col][row] {
                    row_idx_vec.push(row);
                    vals_vec.push(0.0); // will be filled in fill_numeric
                }
            }
        }
        col_ptr[nv] = row_idx_vec.len();

        let mut h = Self {
            nv,
            col_ptr,
            row_idx: row_idx_vec,
            vals: vals_vec,
            etree: vec![None; nv],
            l_col_ptr: vec![0; nv + 1],
            l_row_idx: Vec::new(),
            l_vals: Vec::new(),
            l_diag: vec![0.0; nv],
        };

        // --- Step 3: Fill numeric values ---
        h.fill_numeric(model, data, nv, nefc, implicit_sd, state, cones);

        // --- Step 4: Symbolic factorization ---
        h.symbolic_factor();

        // --- Step 5: Numeric factorization ---
        h.numeric_factor()?;

        Ok(h)
    }

    /// Refactor with updated numeric values (same sparsity pattern), for new
    /// constraint states and cones.
    pub fn refactor(
        &mut self,
        model: &Model,
        data: &Data,
        implicit_sd: bool,
        state: &[ConstraintState],
        cones: &[Cone],
    ) -> Result<(), StepError> {
        let nv = self.nv;
        let nefc = data.efc_type.len();
        self.fill_numeric(model, data, nv, nefc, implicit_sd, state, cones);
        self.numeric_factor()
    }

    /// Fill CSC values with `H = M + Σ_{Quadratic} D_i·J_i'·J_i` plus each
    /// cone's `J_c'·H_c·J_c`. DT-35: When `implicit_sd` is true, also adds
    /// joint diagonal K/D and tendon non-diagonal K/D to match
    /// `build_m_impl`.
    // Each term reads as its own block; splitting it would scatter one sum.
    // Rows are read across parallel arrays at one index.
    #[allow(
        clippy::too_many_arguments,
        clippy::too_many_lines,
        clippy::needless_range_loop
    )]
    fn fill_numeric(
        &mut self,
        model: &Model,
        data: &Data,
        nv: usize,
        nefc: usize,
        implicit_sd: bool,
        state: &[ConstraintState],
        cones: &[Cone],
    ) {
        // Zero all values
        self.vals.iter_mut().for_each(|v| *v = 0.0);

        // Add M using tree sparsity: only walk (i, ancestor) pairs via dof_parent.
        // This is O(nv · depth) instead of O(nv²) — much cheaper for tree-structured
        // robots where depth << nv.
        for i in 0..nv {
            // Diagonal
            self.add(i, i, data.qM[(i, i)]);
            // Off-diagonal: walk ancestors
            let mut p = model.dof_parent[i];
            while let Some(j) = p {
                // M[i,j] non-zero, j < i (ancestor). Store in lower triangle: col=j, row=i.
                self.add(j, i, data.qM[(i, j)]);
                p = model.dof_parent[j];
            }
        }

        // DT-35: Add joint diagonal K/D and tendon non-diagonal K/D for
        // ImplicitSpringDamper. This matches build_m_impl.
        if implicit_sd {
            let h = model.timestep;
            let h2 = h * h;
            let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

            // Joint diagonal K/D
            for i in 0..nv {
                let kd = h * isd_damping(model, i) + h2 * isd_stiffness(model, i);
                if kd != 0.0 {
                    self.add(i, i, kd);
                }
            }

            // Tendon non-diagonal K/D (rank-1 outer products)
            let mut nz: Vec<(usize, f64)> = Vec::with_capacity(8);
            for t in 0..model.ntendon {
                if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
                    continue;
                }
                let kt = isd_tendon_stiffness(model, t);
                let bt = isd_tendon_damping(model, t);
                if kt == 0.0 && bt == 0.0 {
                    continue;
                }
                let j = &data.ten_J[t];
                let k_active =
                    tendon_active_stiffness(kt, data.ten_length[t], model.tendon_lengthspring[t]);
                let scale = h2 * k_active + h * bt;
                if scale == 0.0 {
                    continue;
                }
                nz.clear();
                for dof in 0..nv {
                    if j[dof] != 0.0 {
                        nz.push((dof, j[dof]));
                    }
                }
                for (ai, &(col_a, j_a)) in nz.iter().enumerate() {
                    let s_j_a = scale * j_a;
                    for &(col_b, j_b) in &nz[ai..] {
                        self.add(col_a, col_b, s_j_a * j_b);
                    }
                }
            }
        }

        // Add J^T · D · J for Quadratic rows.
        // Cache non-zero column indices per constraint row to avoid O(nv) scans
        // in the inner loop — reduces J^T·D·J fill from O(nefc·nv²) to
        // O(nefc·nnz_per_row²) where nnz_per_row is typically << nv.
        for r in 0..nefc {
            if state[r] != ConstraintState::Quadratic {
                continue;
            }
            let d_r = data.efc_D[r];
            // Collect non-zero columns for this J row
            let mut nz_cols: Vec<(usize, f64)> = Vec::new();
            for col in 0..nv {
                let j_val = data.efc_J[(r, col)];
                if j_val != 0.0 {
                    nz_cols.push((col, j_val));
                }
            }
            // Outer product over non-zero pairs only
            for (ai, &(col_a, j_a)) in nz_cols.iter().enumerate() {
                let d_j_a = d_r * j_a;
                for &(col_b, j_b) in &nz_cols[ai..] {
                    // Lower triangle: col_b >= col_a
                    self.add(col_a, col_b, d_j_a * j_b);
                }
            }
        }

        // Each cone: J_c'·H_c·J_c over its rows' non-zero columns.
        for &(row, dim, ref hc) in cones {
            let mut cols: Vec<usize> = (0..nv)
                .filter(|&c| (0..dim).any(|a| data.efc_J[(row + a, c)] != 0.0))
                .collect();
            cols.sort_unstable();
            // w = H_c·J_c on those columns.
            let w: Vec<Vec<f64>> = (0..dim)
                .map(|a| {
                    cols.iter()
                        .map(|&c| {
                            (0..dim)
                                .map(|b| hc[a * dim + b] * data.efc_J[(row + b, c)])
                                .sum()
                        })
                        .collect()
                })
                .collect();
            for (pi, &col_a) in cols.iter().enumerate() {
                for (qi, &col_b) in cols.iter().enumerate().skip(pi) {
                    let val: f64 = (0..dim)
                        .map(|a| data.efc_J[(row + a, col_a)] * w[a][qi])
                        .sum();
                    self.add(col_a, col_b, val);
                }
            }
        }
    }

    /// Add `value` to the stored entry (col, row), row ≥ col.
    ///
    /// # Panics
    ///
    /// If the pattern lacks the entry: [`Self::assemble`] lays out every
    /// entry the fill writes, so a miss is a bug in the pattern.
    fn add(&mut self, col: usize, row: usize, value: f64) {
        let start = self.col_ptr[col];
        let end = self.col_ptr[col + 1];
        // Binary search in the sorted row indices for this column
        let found = self.row_idx[start..end].binary_search(&row);
        assert!(
            found.is_ok(),
            "the sparse Hessian's pattern lacks entry ({row}, {col})"
        );
        if let Ok(offset) = found {
            self.vals[start + offset] += value;
        }
    }

    /// Compute elimination tree from the CSC sparsity pattern.
    /// etree\[j\] = min { i > j : L\[i,j\] != 0 }, which equals the first
    /// off-diagonal non-zero row in column j of L.
    ///
    /// Also computes the symbolic structure of L (l_col_ptr, l_row_idx).
    // Indexed loop mutates parallel arrays at the same index; iterator forms would obscure the per-row update.
    #[allow(clippy::needless_range_loop)]
    fn symbolic_factor(&mut self) {
        let n = self.nv;

        // Compute elimination tree using the Liu algorithm:
        // For each column j, the row indices of H below the diagonal determine
        // which columns will have fill in L. The parent of j in the etree is
        // the first row index > j that appears in column j of H or through
        // fill propagation.
        let mut parent = vec![None; n];
        let mut ancestor = vec![0usize; n]; // path-compressed ancestor

        for j in 0..n {
            ancestor[j] = j;
            let start = self.col_ptr[j];
            let end = self.col_ptr[j + 1];
            for k in start..end {
                let i = self.row_idx[k];
                if i <= j {
                    continue;
                }
                // Walk up the etree from i using path compression
                let mut r = i;
                while ancestor[r] != r && ancestor[r] != j {
                    let next = ancestor[r];
                    ancestor[r] = j;
                    r = next;
                }
                if ancestor[r] == r {
                    // r is a root — make j its parent
                    parent[r] = Some(j);
                    ancestor[r] = j;
                }
            }
        }

        self.etree = parent;

        // Compute symbolic L structure: for each column j, L has non-zero entries
        // at all rows that appear in H[:,j] below diagonal, plus fill from the etree.
        // Use row counts approach: for each column j, collect all row indices in the
        // subtree of j's column in H, then propagate up the etree.
        let mut l_row_sets: Vec<Vec<usize>> = vec![Vec::new(); n];

        for j in 0..n {
            let start = self.col_ptr[j];
            let end = self.col_ptr[j + 1];
            for k in start..end {
                let i = self.row_idx[k];
                if i > j {
                    l_row_sets[j].push(i);
                }
            }
        }

        // Propagate fill: for each column j (in order), merge its row set into
        // parent's row set (excluding j itself, since L[j,j] = 1 implicitly).
        for j in 0..n {
            l_row_sets[j].sort_unstable();
            l_row_sets[j].dedup();
            if let Some(p) = self.etree[j] {
                // All rows in L[:,j] that are > p are also in L[:,p]
                let fill: Vec<usize> = l_row_sets[j].iter().copied().filter(|&r| r > p).collect();
                // Need to clone to avoid borrow conflict
                l_row_sets[p].extend(fill);
            }
        }

        // Sort and dedup all sets again after propagation
        for j in 0..n {
            l_row_sets[j].sort_unstable();
            l_row_sets[j].dedup();
        }

        // Build CSC for L
        let mut l_col_ptr = vec![0usize; n + 1];
        let mut l_row_idx = Vec::new();
        for j in 0..n {
            l_col_ptr[j] = l_row_idx.len();
            l_row_idx.extend_from_slice(&l_row_sets[j]);
        }
        l_col_ptr[n] = l_row_idx.len();

        let l_nnz = l_row_idx.len();
        self.l_col_ptr = l_col_ptr;
        self.l_row_idx = l_row_idx;
        self.l_vals = vec![0.0; l_nnz];
        self.l_diag = vec![0.0; n];
    }

    /// Numeric LDL^T factorization.
    ///
    /// Computes L (unit lower triangular) and D (diagonal) such that H = L·D·L^T.
    /// Uses a left-looking approach: for each column j, subtract contributions from
    /// columns to the left, then scale.
    fn numeric_factor(&mut self) -> Result<(), StepError> {
        let n = self.nv;

        // Work arrays
        let mut y = vec![0.0f64; n]; // dense accumulator for column j of L*D
        let mut pattern = vec![false; n]; // which rows have non-zero in current column

        for j in 0..n {
            // Initialize y with column j of H (lower triangle)
            let h_start = self.col_ptr[j];
            let h_end = self.col_ptr[j + 1];
            for k in h_start..h_end {
                let i = self.row_idx[k];
                y[i] = self.vals[k];
                pattern[i] = true;
            }

            // Subtract contributions from earlier columns that have non-zero in row j
            // For each column k < j where L[j,k] != 0:
            //   y[i] -= L[j,k] * D[k] * L[i,k] for all i in L[:,k] with i >= j
            for k in 0..j {
                let l_start = self.l_col_ptr[k];
                let l_end = self.l_col_ptr[k + 1];

                // Find L[j,k] in L[:,k]
                let mut ljk = 0.0;
                for p in l_start..l_end {
                    if self.l_row_idx[p] == j {
                        ljk = self.l_vals[p];
                        break;
                    }
                    if self.l_row_idx[p] > j {
                        break; // sorted, so we're past j
                    }
                }

                if ljk == 0.0 {
                    continue;
                }

                let dk = self.l_diag[k];
                let ljk_dk = ljk * dk;

                // Subtract from diagonal
                y[j] -= ljk_dk * ljk;

                // Subtract from below-diagonal entries
                for p in l_start..l_end {
                    let i = self.l_row_idx[p];
                    if i <= j {
                        continue;
                    }
                    y[i] -= ljk_dk * self.l_vals[p];
                }
            }

            // Extract D[j] = y[j]. Below mjMINVAL MuJoCo stops ("rank-deficient
            // sparse Hessian").
            let dj = y[j];
            if dj < MJ_MINVAL {
                // Clean up work arrays before returning error
                for k in h_start..h_end {
                    let i = self.row_idx[k];
                    y[i] = 0.0;
                    pattern[i] = false;
                }
                return Err(StepError::CholeskyFailed);
            }
            self.l_diag[j] = dj;
            let dj_inv = 1.0 / dj;

            // Extract L[:,j] = y[j+1:] / D[j]
            let l_start = self.l_col_ptr[j];
            let l_end = self.l_col_ptr[j + 1];
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                self.l_vals[p] = y[i] * dj_inv;
            }

            // Clean up work arrays
            y[j] = 0.0;
            pattern[j] = false;
            for k in h_start..h_end {
                let i = self.row_idx[k];
                y[i] = 0.0;
                pattern[i] = false;
            }
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                y[i] = 0.0;
                pattern[i] = false;
            }
        }

        Ok(())
    }

    /// Solve L·D·L^T · x = b in place.
    // Substitution reads and writes `x` at the factor's row indices.
    #[allow(clippy::needless_range_loop)]
    pub fn solve(&self, x: &mut [f64]) {
        let n = self.nv;

        // Forward substitution: L · y = b
        // L is unit lower triangular (L[j,j] = 1, not stored)
        for j in 0..n {
            let xj = x[j];
            if xj == 0.0 {
                continue;
            }
            let l_start = self.l_col_ptr[j];
            let l_end = self.l_col_ptr[j + 1];
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                x[i] -= self.l_vals[p] * xj;
            }
        }

        // Diagonal solve: D · z = y
        for j in 0..n {
            x[j] /= self.l_diag[j];
        }

        // Back substitution: L^T · w = z
        // L^T is unit upper triangular
        for j in (0..n).rev() {
            let l_start = self.l_col_ptr[j];
            let l_end = self.l_col_ptr[j + 1];
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                x[j] -= self.l_vals[p] * x[i];
            }
        }
    }
}
