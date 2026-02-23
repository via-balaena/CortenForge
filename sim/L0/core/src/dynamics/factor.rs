//! Sparse L^T D L factorization of the mass matrix.
//!
//! Exploits the tree sparsity structure (from `dof_parent` chains) for O(n)
//! factorization and solve. Also includes CSR metadata precomputation for
//! the sparse layout. Corresponds to `mj_factorI` in MuJoCo's
//! `engine_util_sparse.c`.

use crate::types::{Data, ENABLE_SLEEP, Model};

impl Model {
    /// Compute CSR metadata for sparse LDL factorization from `dof_parent` chains.
    ///
    /// Must be called after `dof_parent` is finalized (in `ModelBuilder::build()` or
    /// test helpers that construct Model manually). The sparsity pattern is immutable
    /// after this call.
    ///
    /// Each row stores off-diagonal entries (ancestors) followed by the diagonal
    /// (self-index) as the last element, matching MuJoCo's `mj_factorI` layout.
    /// `rownnz[i]` includes the diagonal, so `rownnz[i] - 1` is the off-diagonal count.
    pub fn compute_qld_csr_metadata(&mut self) {
        let nv = self.nv;
        self.qLD_rownnz = vec![0; nv];
        self.qLD_rowadr = vec![0; nv];

        // Pass 1: Count entries per row (ancestors + 1 for diagonal)
        for i in 0..nv {
            let mut count = 0;
            let mut p = self.dof_parent[i];
            while let Some(j) = p {
                count += 1;
                p = self.dof_parent[j];
            }
            self.qLD_rownnz[i] = count + 1; // +1 for diagonal
        }

        // Pass 2: Compute row addresses (prefix sum)
        let mut offset = 0;
        for i in 0..nv {
            self.qLD_rowadr[i] = offset;
            offset += self.qLD_rownnz[i];
        }
        self.qLD_nnz = offset;

        // Pass 3: Fill column indices (ancestors ascending, then self-index for diagonal)
        self.qLD_colind = vec![0; self.qLD_nnz];
        for i in 0..nv {
            let mut ancestors = Vec::new();
            let mut p = self.dof_parent[i];
            while let Some(j) = p {
                ancestors.push(j);
                p = self.dof_parent[j];
            }
            ancestors.reverse(); // ascending order (root ancestor first)
            let start = self.qLD_rowadr[i];
            for (k, &col) in ancestors.iter().enumerate() {
                self.qLD_colind[start + k] = col;
            }
            // Diagonal as last element (matching MuJoCo's layout)
            self.qLD_colind[start + ancestors.len()] = i;
        }
    }
}

/// Full sparse L^T D L factorization of the mass matrix.
///
/// Processes all DOFs unconditionally. The elimination traverses leaves-to-root
/// using `dof_parent` chains, producing the L and D factors stored in `qLD_data`
/// and the inverse diagonal in `qLD_diag_inv`.
///
/// ## Sparsity Layout
///
/// Row `i` has `rownnz[i]` entries: off-diagonal ancestors followed by the diagonal.
/// Both are stored ascending in `qLD_colind`. Row `i`'s off-diagonal entries at
/// positions `0..a` (where `a` is `j`'s position in row `i`) are exactly
/// `ancestors(j)`, so `a == rownnz[j] - 1`. This means `row_i[0..a]` and
/// `row_j[0..rownnz_j-1]` have identical column indices — a simple element-wise
/// scaled addition suffices.
#[allow(non_snake_case)]
pub fn mj_factor_sparse(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let (rowadr, rownnz, colind) = model.qld_csr();

    // Phase 1: Copy M's sparse entries into flat CSR (diagonal included as last element).
    for i in 0..nv {
        let start = rowadr[i];
        let nnz = rownnz[i];
        for k in 0..nnz {
            data.qLD_data[start + k] = data.qM[(i, colind[start + k])];
        }
    }

    // Phase 2: Eliminate from leaves to root.
    // Process DOF i, scale its off-diagonals by 1/D[i], then propagate
    // rank-1 update to all ancestor pairs using bulk row addition.
    for i in (0..nv).rev() {
        let start_i = rowadr[i];
        let nnz_i = rownnz[i];
        let diag_pos = start_i + nnz_i - 1;
        let di = data.qLD_data[diag_pos];

        // Precompute inverse diagonal (matching MuJoCo's diaginv output).
        let inv_di = 1.0 / di;
        data.qLD_diag_inv[i] = inv_di;

        // Root DOFs with no off-diagonals (rownnz == 1): only diagonal, no elimination.
        let nnz_offdiag = nnz_i - 1;
        if nnz_offdiag == 0 {
            continue;
        }

        // Scale off-diagonals: L[i,j] = working[i,j] / D[i]
        for k in 0..nnz_offdiag {
            data.qLD_data[start_i + k] *= inv_di;
        }

        // Propagate rank-1 update to ancestors (deep-to-shallow, matching MuJoCo).
        // For each ancestor j of i (traversed from deepest to shallowest):
        //   - Diagonal update: D[j] -= L[i,j]^2 * D[i]
        //   - Bulk row update: row_j[0..nnz_j-1] -= L[i,j] * row_i[0..a] * D[i]
        //     where a is j's off-diag position in row i, and a == rownnz[j]-1.
        for a in (0..nnz_offdiag).rev() {
            let j = colind[start_i + a];
            let lij = data.qLD_data[start_i + a];

            // Diagonal update on ancestor j
            let j_diag_pos = rowadr[j] + rownnz[j] - 1;
            data.qLD_data[j_diag_pos] -= lij * lij * di;

            // Bulk row update: row_j[0..a] -= lij * D[i] * row_i[0..a]
            // Ancestor superset: off-diag count of j == a (i.e., rownnz[j] - 1 == a).
            let j_nnz_offdiag = rownnz[j] - 1;
            debug_assert_eq!(
                a, j_nnz_offdiag,
                "ancestor row superset property violated: \
                DOF {i} entry at position {a} maps to ancestor DOF {j} with offdiag_nnz={j_nnz_offdiag}"
            );
            let scale = -lij * di;

            // split_at_mut(start_i) safe: j < i guarantees rowadr[j]+rownnz[j] <= rowadr[i].
            let start_j = rowadr[j];
            let (lo, hi) = data.qLD_data.split_at_mut(start_i);
            let dst = &mut lo[start_j..start_j + a];
            let src = &hi[..a];
            for k in 0..a {
                dst[k] += scale * src[k];
            }
        }
    }

    data.qLD_valid = true;
}

/// Sleep-aware sparse L^T D L factorization (C3b).
///
/// When sleeping is enabled and some DOFs are asleep, only awake DOFs are
/// factored. Sleeping DOFs' `qLD_data` and `qLD_diag_inv` entries are preserved
/// from their last awake step.
///
/// # Tree Independence
///
/// `dof_parent` chains never cross tree boundaries. Sleeping is per-tree.
/// Therefore, the elimination of awake DOF `i` only updates ancestors in the
/// same (awake) tree — sleeping trees' qLD entries are never touched.
///
/// When all DOFs are awake (or sleep is disabled), dispatches to the full
/// `mj_factor_sparse` with zero overhead.
#[allow(non_snake_case)]
pub fn mj_factor_sparse_selective(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nv_awake < model.nv;

    if !sleep_filter {
        mj_factor_sparse(model, data);
        return;
    }

    let (rowadr, rownnz, colind) = model.qld_csr();

    // Phase 1: Copy M into CSR for awake DOFs only.
    // Sleeping DOFs' qLD_data entries are preserved from last awake step.
    // rownnz[i] includes diagonal; we copy all entries (off-diag + diag at last position).
    for idx in 0..data.nv_awake {
        let i = data.dof_awake_ind[idx];
        let start = rowadr[i];
        let nnz = rownnz[i];
        for k in 0..nnz {
            data.qLD_data[start + k] = data.qM[(i, colind[start + k])];
        }
    }

    // Phase 2: Eliminate awake DOFs only (reverse order).
    // dof_awake_ind is sorted ascending; reverse iteration gives leaf-to-root order.
    // All ancestors of an awake DOF are in the same awake tree (per-tree invariant).
    // rownnz[i] includes diagonal; off-diag count = rownnz[i] - 1.
    for idx in (0..data.nv_awake).rev() {
        let i = data.dof_awake_ind[idx];
        let start_i = rowadr[i];
        let nnz_i = rownnz[i];
        let diag_pos = start_i + nnz_i - 1;
        let di = data.qLD_data[diag_pos];

        let inv_di = 1.0 / di;
        data.qLD_diag_inv[i] = inv_di;

        let nnz_offdiag = nnz_i - 1;
        if nnz_offdiag == 0 {
            continue;
        }

        for k in 0..nnz_offdiag {
            data.qLD_data[start_i + k] *= inv_di;
        }

        for a in (0..nnz_offdiag).rev() {
            let j = colind[start_i + a];
            let lij = data.qLD_data[start_i + a];

            // Diagonal update on ancestor j
            let j_diag_pos = rowadr[j] + rownnz[j] - 1;
            data.qLD_data[j_diag_pos] -= lij * lij * di;

            let start_j = rowadr[j];
            let nnz_j_offdiag = rownnz[j] - 1;
            debug_assert_eq!(a, nnz_j_offdiag);
            let scale = -lij * di;

            let (lo, hi) = data.qLD_data.split_at_mut(start_i);
            let dst = &mut lo[start_j..start_j + nnz_j_offdiag];
            let src = &hi[..a];
            for k in 0..a {
                dst[k] += scale * src[k];
            }
        }
    }

    data.qLD_valid = true;
}
