//! Newton Hessian assembly, factorization, and incremental updates.
//!
//! Contains the dense Hessian assembly (`assemble_hessian`), the sparse
//! Hessian (`SparseHessian`) with CSC storage and LDL^T factorization,
//! incremental rank-1 Cholesky updates (`hessian_incremental`), and
//! cone Hessian augmentation (`hessian_cone`).
//!
//! Corresponds to the Hessian infrastructure in MuJoCo's `engine_solver.c`
//! (§15.6–15.7).

use nalgebra::{DMatrix, DVector};

use crate::integrate::implicit::{tendon_active_stiffness, tendon_all_dofs_sleeping};
use crate::linalg::{cholesky_in_place, cholesky_rank1_update};
use crate::types::{ConstraintState, ConstraintType, Data, ENABLE_SLEEP, Model, StepError};

/// DOF count above which Newton switches from dense O(nv³) Cholesky to
/// sparse LDL^T. At nv ≈ 60, sparse becomes competitive with dense.
pub const NV_SPARSE_THRESHOLD: usize = 60;

/// Assemble the Newton Hessian and factor via Cholesky.
///
/// Phase A: H = M_eff + Σ_{Quadratic rows} D_i · J_i^T · J_i
/// (No cone Hessian in Phase A — cone rows are treated as Quadratic per-row.)
///
/// DT-35: `m_eff` is `M_impl` when `ImplicitSpringDamper` is active,
/// `data.qM` otherwise. The Hessian starts from `m_eff` so that tendon K/D
/// contributions are included in the Newton system.
///
/// Returns the Cholesky factor L (lower triangular) such that H = L · L^T,
/// or an error if the Hessian is not positive definite.
pub fn assemble_hessian(
    data: &Data,
    nv: usize,
    m_eff: &DMatrix<f64>,
) -> Result<DMatrix<f64>, StepError> {
    let nefc = data.efc_type.len();

    // Start with effective mass matrix (M or M_impl)
    let mut h = DMatrix::<f64>::zeros(nv, nv);
    for r in 0..nv {
        for c in 0..nv {
            h[(r, c)] = m_eff[(r, c)];
        }
    }

    // Add Σ_{Quadratic rows} D_i · J_i^T · J_i
    for i in 0..nefc {
        if data.efc_state[i] != ConstraintState::Quadratic {
            continue;
        }
        let d_i = data.efc_D[i];
        // Rank-1 update: H += D_i · j_i · j_i^T
        for r in 0..nv {
            let j_r = data.efc_J[(i, r)];
            if j_r == 0.0 {
                continue;
            }
            let d_j_r = d_i * j_r;
            for c in r..nv {
                let j_c = data.efc_J[(i, c)];
                if j_c == 0.0 {
                    continue;
                }
                let val = d_j_r * j_c;
                h[(r, c)] += val;
                if r != c {
                    h[(c, r)] += val;
                }
            }
        }
    }

    // Cholesky factorize in-place
    cholesky_in_place(&mut h)?;
    Ok(h)
}

// ============================================================================
// Sparse Hessian path (Phase C): for large systems (nv > NV_SPARSE_THRESHOLD)
// ============================================================================

/// Sparse Hessian H = M + J^T·D·J in CSC lower-triangle format with
/// cached symbolic/numeric LDL^T factorization.
///
/// The sparsity pattern comes from:
/// - M: tree sparsity via `dof_parent` (entry (i,j) iff j is ancestor of i)
/// - J^T·D·J: couples DOFs that share constraint rows
///
/// Symbolic factorization (elimination tree + L structure) is computed once
/// per `assemble` call. Numeric factorization is recomputed each Newton
/// iteration via `factor()`. Solve is forward/diagonal/back substitution.
pub struct SparseHessian {
    nv: usize,
    /// CSC column pointers for lower triangle of H (length nv+1).
    col_ptr: Vec<usize>,
    /// CSC row indices (length nnz). Sorted within each column.
    row_idx: Vec<usize>,
    /// CSC values (length nnz).
    vals: Vec<f64>,
    /// Elimination tree: parent[j] = parent of column j in etree, or None for root.
    etree: Vec<Option<usize>>,
    /// CSC column pointers for L factor (length nv+1).
    l_col_ptr: Vec<usize>,
    /// CSC row indices for L factor.
    l_row_idx: Vec<usize>,
    /// Numeric values of L factor (unit lower triangular: L[j,j] = 1, not stored).
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
    #[allow(clippy::needless_range_loop)]
    pub fn assemble(
        model: &Model,
        data: &Data,
        nv: usize,
        implicit_sd: bool,
    ) -> Result<Self, StepError> {
        let nefc = data.efc_type.len();

        // --- Step 1: Determine sparsity pattern ---
        // Use a dense boolean mask per column (acceptable since this is O(nv²) and
        // we only enter the sparse path when nv > 60 where this is ~3600 entries).
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

        // J^T·D·J sparsity: for each Quadratic constraint row, the outer product
        // of its non-zero J entries determines fill. We conservatively include all
        // non-zero pairs.
        for r in 0..nefc {
            if data.efc_state[r] != ConstraintState::Quadratic {
                continue;
            }
            // Collect non-zero column indices in this J row
            let mut nz_cols: Vec<usize> = Vec::new();
            for col in 0..nv {
                if data.efc_J[(r, col)] != 0.0 {
                    nz_cols.push(col);
                }
            }
            // Mark all pairs (lower triangle)
            for &ci in &nz_cols {
                for &cj in &nz_cols {
                    let (lo, hi) = if ci <= cj { (ci, cj) } else { (cj, ci) };
                    has_entry[lo][hi] = true;
                }
            }
        }

        // DT-35: Tendon K/D sparsity (ImplicitSpringDamper only).
        // Conservative: includes entries for all tendons with k > 0 or b > 0,
        // regardless of deadband state. Actual values use deadband-aware k_active.
        if implicit_sd {
            let mut nz: Vec<usize> = Vec::with_capacity(8);
            for t in 0..model.ntendon {
                let kt = model.tendon_stiffness[t];
                let bt = model.tendon_damping[t];
                if kt <= 0.0 && bt <= 0.0 {
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
        h.fill_numeric(model, data, nv, nefc, implicit_sd);

        // --- Step 4: Symbolic factorization ---
        h.symbolic_factor();

        // --- Step 5: Numeric factorization ---
        h.numeric_factor()?;

        Ok(h)
    }

    /// Refactor with updated numeric values (same sparsity pattern).
    /// Used when constraint states change but sparsity doesn't.
    pub fn refactor(
        &mut self,
        model: &Model,
        data: &Data,
        implicit_sd: bool,
    ) -> Result<(), StepError> {
        let nv = self.nv;
        let nefc = data.efc_type.len();
        self.fill_numeric(model, data, nv, nefc, implicit_sd);
        self.numeric_factor()
    }

    /// Fill CSC values with H = M + Σ_{Quadratic} D_i · J_i^T · J_i.
    /// DT-35: When `implicit_sd` is true, also adds joint diagonal K/D and
    /// tendon non-diagonal K/D to match `build_m_impl_for_newton`.
    fn fill_numeric(
        &mut self,
        model: &Model,
        data: &Data,
        nv: usize,
        nefc: usize,
        implicit_sd: bool,
    ) {
        // Zero all values
        self.vals.iter_mut().for_each(|v| *v = 0.0);

        // Add M using tree sparsity: only walk (i, ancestor) pairs via dof_parent.
        // This is O(nv · depth) instead of O(nv²) — much cheaper for tree-structured
        // robots where depth << nv.
        for i in 0..nv {
            // Diagonal
            if let Some(idx) = self.find_entry(i, i) {
                self.vals[idx] += data.qM[(i, i)];
            }
            // Off-diagonal: walk ancestors
            let mut p = model.dof_parent[i];
            while let Some(j) = p {
                // M[i,j] non-zero, j < i (ancestor). Store in lower triangle: col=j, row=i.
                let m_val = data.qM[(i, j)];
                if let Some(idx) = self.find_entry(j, i) {
                    self.vals[idx] += m_val;
                }
                p = model.dof_parent[j];
            }
        }

        // DT-35: Add joint diagonal K/D and tendon non-diagonal K/D for
        // ImplicitSpringDamper. This matches build_m_impl_for_newton.
        if implicit_sd {
            let h = model.timestep;
            let h2 = h * h;
            let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

            // Joint diagonal K/D
            for i in 0..nv {
                let kd = h * model.implicit_damping[i] + h2 * model.implicit_stiffness[i];
                if kd > 0.0 {
                    if let Some(idx) = self.find_entry(i, i) {
                        self.vals[idx] += kd;
                    }
                }
            }

            // Tendon non-diagonal K/D (rank-1 outer products)
            let mut nz: Vec<(usize, f64)> = Vec::with_capacity(8);
            for t in 0..model.ntendon {
                if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
                    continue;
                }
                let kt = model.tendon_stiffness[t];
                let bt = model.tendon_damping[t];
                if kt <= 0.0 && bt <= 0.0 {
                    continue;
                }
                let j = &data.ten_J[t];
                let k_active =
                    tendon_active_stiffness(kt, data.ten_length[t], model.tendon_lengthspring[t]);
                let scale = h2 * k_active + h * bt;
                if scale <= 0.0 {
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
                        if let Some(idx) = self.find_entry(col_a, col_b) {
                            self.vals[idx] += s_j_a * j_b;
                        }
                    }
                }
            }
        }

        // Add J^T · D · J for Quadratic rows.
        // Cache non-zero column indices per constraint row to avoid O(nv) scans
        // in the inner loop — reduces J^T·D·J fill from O(nefc·nv²) to
        // O(nefc·nnz_per_row²) where nnz_per_row is typically << nv.
        for r in 0..nefc {
            if data.efc_state[r] != ConstraintState::Quadratic {
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
                    if let Some(idx) = self.find_entry(col_a, col_b) {
                        self.vals[idx] += d_j_a * j_b;
                    }
                }
            }
        }
    }

    /// Find the CSC index for entry (col, row) where row >= col.
    fn find_entry(&self, col: usize, row: usize) -> Option<usize> {
        let start = self.col_ptr[col];
        let end = self.col_ptr[col + 1];
        // Binary search in the sorted row indices for this column
        self.row_idx[start..end]
            .binary_search(&row)
            .ok()
            .map(|offset| start + offset)
    }

    /// Compute elimination tree from the CSC sparsity pattern.
    /// etree\[j\] = min { i > j : L\[i,j\] != 0 }, which equals the first
    /// off-diagonal non-zero row in column j of L.
    ///
    /// Also computes the symbolic structure of L (l_col_ptr, l_row_idx).
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

            // Extract D[j] = y[j]
            let dj = y[j];
            if dj <= 0.0 {
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
    pub fn solve(&self, x: &mut DVector<f64>) {
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

/// Incrementally update the Cholesky factor when constraint states change.
///
/// Instead of reassembling the full Hessian and re-factoring, uses rank-1
/// updates and downdates to modify the existing Cholesky factor L:
/// - Old state != Quadratic AND new state == Quadratic → rank-1 update (add D_i · J_i^T · J_i)
/// - Old state == Quadratic AND new state != Quadratic → rank-1 downdate (remove D_i · J_i^T · J_i)
///
/// Falls back to full `assemble_hessian` if a downdate fails (would make H non-PD).
#[allow(clippy::needless_range_loop)]
pub fn hessian_incremental(
    data: &Data,
    nv: usize,
    chol_l: &mut DMatrix<f64>,
    old_states: &[ConstraintState],
    m_eff: &DMatrix<f64>,
) -> Result<(), StepError> {
    let nefc = data.efc_type.len();

    for i in 0..nefc {
        let was_quad = old_states[i] == ConstraintState::Quadratic;
        let is_quad = data.efc_state[i] == ConstraintState::Quadratic;

        if was_quad == is_quad {
            continue; // No change for this row
        }

        let d_i = data.efc_D[i];
        let sqrt_d = d_i.sqrt();

        // Build the rank-1 vector: v = sqrt(D_i) · J_i
        let mut v: Vec<f64> = (0..nv).map(|col| sqrt_d * data.efc_J[(i, col)]).collect();

        if !was_quad && is_quad {
            // Became Quadratic → rank-1 update (add)
            cholesky_rank1_update(chol_l, &mut v)?;
        } else {
            // Was Quadratic, now isn't → rank-1 downdate (remove)
            // For downdate: negate the vector before calling the update
            // If this fails, fall back to full reassembly
            for x in &mut v {
                *x = -*x;
            }
            if cholesky_rank1_update(chol_l, &mut v).is_err() {
                // Downdate failed — full reassembly
                *chol_l = assemble_hessian(data, nv, m_eff)?;
                return Ok(());
            }
        }
    }

    Ok(())
}

/// Produce L_cone from L by adding cone Hessian contributions via rank-1 updates.
///
/// For each cone-state contact with a stored `efc_cone_hessian[ci]`:
/// 1. Factor the local dim×dim H_c via Cholesky → L_local
/// 2. For each column k of L_local, build v = L_local[:,k]^T * J_contact (nv vector)
/// 3. Apply rank-1 update to L_cone with v
///
/// Returns the modified Cholesky factor, or falls back to full `assemble_hessian` if
/// any update fails.
pub fn hessian_cone(
    data: &Data,
    nv: usize,
    chol_l: &DMatrix<f64>,
) -> Result<DMatrix<f64>, StepError> {
    let mut l_cone = chol_l.clone();

    for (ci, hc_opt) in data.efc_cone_hessian.iter().enumerate() {
        let Some(hc) = hc_opt else {
            continue;
        };

        let dim = hc.nrows();

        // Find the starting efc row for this contact.
        // Scan efc_id to find the first row with id == ci and type == ContactElliptic.
        let Some(efc_start) = data
            .efc_id
            .iter()
            .position(|&id| id == ci && data.efc_type[id] == ConstraintType::ContactElliptic)
        else {
            continue;
        };

        // Factor H_c via Cholesky: H_c = L_local · L_local^T
        let mut l_local = hc.clone();
        if cholesky_in_place(&mut l_local).is_err() {
            // H_c not PD — skip this cone (degenerate)
            continue;
        }

        // For each column k of L_local, compute v = sum_j L_local[j,k] * J[efc_start+j, :]
        // Then rank-1 update L_cone with v
        for k in 0..dim {
            let mut v = vec![0.0_f64; nv];
            for j in k..dim {
                // L_local is lower triangular, so L_local[j,k] = 0 for j < k
                let ljk = l_local[(j, k)];
                if ljk == 0.0 {
                    continue;
                }
                let efc_row = efc_start + j;
                for (col, v_col) in v.iter_mut().enumerate() {
                    *v_col += ljk * data.efc_J[(efc_row, col)];
                }
            }
            cholesky_rank1_update(&mut l_cone, &mut v)?;
        }
    }

    Ok(l_cone)
}
