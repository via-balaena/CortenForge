//! Sparse L^T D L factorization of the mass matrix.
//!
//! Exploits the tree sparsity structure (from `dof_parent` chains) for O(n)
//! factorization and solve. Also includes CSR metadata precomputation for
//! the sparse layout. Corresponds to `mj_factorI` in MuJoCo's
//! `engine_util_sparse.c`.

use crate::linalg::dot_sparse;
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

/// Factor `mat`, a matrix on a tree pattern whose row `k` holds, in
/// ascending columns, `k`'s ancestors and then `k` itself (`Model::M_*`), as
/// `L'·D·L` in place: MuJoCo 3.5.0's `mj_factorI`
/// (`engine_core_smooth.c:1991-2015`), operation for operation. From the
/// last row up, each row updates its ancestors' rows by its own entries
/// before it is divided by its diagonal, whose inverse goes to `diag_inv`. No
/// pivot is checked: a negative one factors as it comes, and a zero one gives
/// infinities, as in MuJoCo. `index` lists the rows to factor (MuJoCo's awake
/// dofs); every row when `None`.
pub fn mj_factor_i(
    mat: &mut [f64],
    diag_inv: &mut [f64],
    rownnz: &[usize],
    rowadr: &[usize],
    colind: &[usize],
    index: Option<&[usize]>,
) {
    let n = index.map_or(rownnz.len(), <[usize]>::len);
    for j in (0..n).rev() {
        let k = index.map_or(j, |ix| ix[j]);
        let start = rowadr[k];
        let diag = rownnz[k] - 1;
        let inv_d = 1.0 / mat[start + diag];
        diag_inv[k] = inv_d;
        // A row's ancestors come before it: rows are stored in dof order.
        let (above, row_k) = mat.split_at_mut(start);
        for adr in (0..diag).rev() {
            let i = colind[start + adr];
            let scale = -row_k[adr] * inv_d;
            let row_i = &mut above[rowadr[i]..rowadr[i] + rownnz[i]];
            for (x, v) in row_i.iter_mut().zip(row_k.iter()) {
                *x += v * scale;
            }
        }
        for x in &mut row_k[..diag] {
            *x *= inv_d;
        }
    }
}

/// Solve `L'·D·L·x = b` in place with [`mj_factor_i`]'s factor: MuJoCo
/// 3.5.0's `mj_solveLD` (`engine_core_smooth.c:2131-2214`) for one vector:
/// `x ← L⁻ᵀ·x`, each row with a non-zero entry scattered to its ancestors,
/// then `x ← D⁻¹·x`, then `x ← L⁻¹·x`, each row's sum taken as
/// [`dot_sparse`] takes it. `index` as [`mj_factor_i`]'s.
pub fn mj_solve_ld(
    x: &mut [f64],
    mat: &[f64],
    diag_inv: &[f64],
    rownnz: &[usize],
    rowadr: &[usize],
    colind: &[usize],
    index: Option<&[usize]>,
) {
    let n = index.map_or(rownnz.len(), <[usize]>::len);
    let row = |k: usize| index.map_or(k, |ix| ix[k]);
    for k in (0..n).rev() {
        let i = row(k);
        let x_i = x[i];
        if rownnz[i] == 1 || x_i == 0.0 {
            continue;
        }
        for adr in rowadr[i]..rowadr[i] + rownnz[i] - 1 {
            x[colind[adr]] -= mat[adr] * x_i;
        }
    }
    for k in 0..n {
        let i = row(k);
        x[i] *= diag_inv[i];
    }
    for k in 0..n {
        let i = row(k);
        let (adr, d) = (rowadr[i], rownnz[i] - 1);
        if d > 0 {
            let sum = dot_sparse(&mat[adr..adr + d], |t| x[colind[adr + t]]);
            x[i] -= sum;
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod sparse_factorization_tests {
    use super::mj_factor_sparse;
    use crate::linalg::{mj_solve_sparse, mj_solve_sparse_batch};
    use crate::types::{Data, Model};
    use approx::assert_relative_eq;
    use nalgebra::{DMatrix, DVector};

    /// Helper: build a Model + Data with a given dof_parent tree and mass matrix.
    /// Returns (model, data) with qM set to the given matrix and sparse factorization run.
    fn setup_sparse(nv: usize, dof_parent: Vec<Option<usize>>, qm: &DMatrix<f64>) -> (Model, Data) {
        let mut model = Model::empty();
        model.nv = nv;
        model.nq = nv;
        model.qpos0 = DVector::zeros(nv);
        model.implicit_stiffness = DVector::zeros(nv);
        model.implicit_damping = DVector::zeros(nv);
        model.implicit_springref = DVector::zeros(nv);
        model.dof_parent = dof_parent;
        // Fill other required fields for make_data
        model.dof_body = vec![0; nv];
        model.dof_jnt = vec![0; nv];
        model.dof_armature = vec![0.0; nv];
        model.dof_damping = vec![0.0; nv];
        model.dof_frictionloss = vec![0.0; nv];
        model.compute_qld_csr_metadata();
        let mut data = model.make_data();
        data.qM.copy_from(qm);
        mj_factor_sparse(&model, &mut data);
        (model, data)
    }

    /// Verify sparse solve matches nalgebra dense Cholesky.
    fn assert_solve_matches(model: &Model, data: &Data, qm: &DMatrix<f64>, nv: usize) {
        let rhs = DVector::from_fn(nv, |i, _| (i as f64 + 1.0) * 0.7);

        // Reference: nalgebra dense Cholesky
        let chol = qm.clone().cholesky().expect("nalgebra cholesky failed");
        let x_ref = chol.solve(&rhs);

        // CSR sparse solve — validate against reference
        let (rowadr, rownnz, colind) = model.qld_csr();
        let mut x_csr = rhs;
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut x_csr,
        );

        for i in 0..nv {
            assert_relative_eq!(x_csr[i], x_ref[i], epsilon = 1e-12, max_relative = 1e-12);
        }
    }

    #[test]
    fn single_hinge() {
        // nv=1, no parent. M = [[5.0]]
        let nv = 1;
        let dof_parent = vec![None];
        let mut qm = DMatrix::zeros(1, 1);
        qm[(0, 0)] = 5.0;

        let (model, data) = setup_sparse(nv, dof_parent, &qm);

        // D[0] = 5.0, L has no off-diag entries
        assert_relative_eq!(data.qld_diag(&model, 0), 5.0);
        assert_eq!(model.qLD_rownnz[0], 1); // diagonal only
        assert!(data.qLD_valid);

        assert_solve_matches(&model, &data, &qm, nv);
    }

    #[test]
    fn two_link_chain() {
        // nv=2, DOF 1 child of DOF 0
        // M = [[4, 2], [2, 3]]
        let nv = 2;
        let dof_parent = vec![None, Some(0)];
        let mut qm = DMatrix::zeros(2, 2);
        qm[(0, 0)] = 4.0;
        qm[(0, 1)] = 2.0;
        qm[(1, 0)] = 2.0;
        qm[(1, 1)] = 3.0;

        let (model, data) = setup_sparse(nv, dof_parent, &qm);

        // Manual: D[1] = 3, L[1,0] = 2/3, D[0] = 4 - (2/3)^2 * 3 = 4 - 4/3 = 8/3
        assert_relative_eq!(data.qld_diag(&model, 1), 3.0);
        assert_relative_eq!(data.qld_diag(&model, 0), 8.0 / 3.0, epsilon = 1e-14);
        assert_eq!(model.qLD_rownnz[1], 2); // 1 off-diag + 1 diagonal
        assert_eq!(model.qLD_colind[model.qLD_rowadr[1]], 0);
        assert_relative_eq!(
            data.qLD_data[model.qLD_rowadr[1]],
            2.0 / 3.0,
            epsilon = 1e-14
        );

        assert_solve_matches(&model, &data, &qm, nv);
    }

    #[test]
    fn branching_tree() {
        // nv=3: DOF 0 = root, DOF 1 = left child of 0, DOF 2 = right child of 0
        // DOFs 1 and 2 don't couple with each other.
        // M = [[5, 2, 1], [2, 4, 0], [1, 0, 3]]
        let nv = 3;
        let dof_parent = vec![None, Some(0), Some(0)];
        let mut qm = DMatrix::zeros(3, 3);
        qm[(0, 0)] = 5.0;
        qm[(0, 1)] = 2.0;
        qm[(1, 0)] = 2.0;
        qm[(1, 1)] = 4.0;
        qm[(0, 2)] = 1.0;
        qm[(2, 0)] = 1.0;
        qm[(2, 2)] = 3.0;

        let (model, data) = setup_sparse(nv, dof_parent, &qm);

        // D[2] = 3, L[2,0] = 1/3
        // D[1] = 4, L[1,0] = 2/4 = 0.5
        // D[0] = 5 - (0.5)^2 * 4 - (1/3)^2 * 3 = 5 - 1 - 1/3 = 11/3
        assert_relative_eq!(data.qld_diag(&model, 2), 3.0);
        assert_relative_eq!(data.qld_diag(&model, 1), 4.0);
        assert_relative_eq!(data.qld_diag(&model, 0), 11.0 / 3.0, epsilon = 1e-14);

        assert_solve_matches(&model, &data, &qm, nv);
    }

    #[test]
    fn ball_joint_chain() {
        // Simulates a ball joint (3 DOFs) followed by a hinge (1 DOF).
        // DOF tree: 0→1→2→3 (linear chain, nv=4)
        // This tests multi-DOF within a joint (ball: DOFs 0,1,2) plus a child.
        let nv = 4;
        let dof_parent = vec![None, Some(0), Some(1), Some(2)];

        // Build a random SPD matrix with tree sparsity: M[i,j] = 0 unless
        // j is ancestor of i or vice versa. For a chain, all entries are non-zero.
        let mut qm = DMatrix::zeros(nv, nv);
        // Diagonal
        qm[(0, 0)] = 10.0;
        qm[(1, 1)] = 8.0;
        qm[(2, 2)] = 6.0;
        qm[(3, 3)] = 4.0;
        // Off-diagonal (all are on the ancestor path for a chain)
        qm[(1, 0)] = 3.0;
        qm[(0, 1)] = 3.0;
        qm[(2, 0)] = 1.0;
        qm[(0, 2)] = 1.0;
        qm[(2, 1)] = 2.0;
        qm[(1, 2)] = 2.0;
        qm[(3, 0)] = 0.5;
        qm[(0, 3)] = 0.5;
        qm[(3, 1)] = 0.3;
        qm[(1, 3)] = 0.3;
        qm[(3, 2)] = 1.5;
        qm[(2, 3)] = 1.5;

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);
        assert_solve_matches(&model, &data, &qm, nv);
    }

    #[test]
    fn free_body_plus_hinge() {
        // Free joint (6 DOFs: 0→1→2→3→4→5) + hinge child (DOF 6, parent = DOF 5)
        // nv = 7
        let nv = 7;
        let dof_parent = vec![None, Some(0), Some(1), Some(2), Some(3), Some(4), Some(5)];

        // Build SPD matrix: A^T A + n*I with tree-compatible sparsity
        // For a chain, all entries can be non-zero. Use random SPD.
        let mut state: u64 = 12345;
        let mut next = || -> f64 {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            ((state >> 33) as f64) / f64::from(u32::MAX) - 0.5
        };
        let a = DMatrix::from_fn(nv, nv, |_, _| next());
        let qm = a.transpose() * &a + DMatrix::identity(nv, nv) * (nv as f64);

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);
        assert_solve_matches(&model, &data, &qm, nv);
    }

    #[test]
    fn complex_branching_tree() {
        // A more complex tree:
        //       0
        //      / \
        //     1   4
        //    / \
        //   2   3
        // nv=5, dof_parent = [None, Some(0), Some(1), Some(1), Some(0)]
        let nv = 5;
        let dof_parent = vec![None, Some(0), Some(1), Some(1), Some(0)];

        // Build SPD matrix with correct sparsity:
        // M[2,0], M[2,1] non-zero (ancestors of 2: 1, 0)
        // M[3,0], M[3,1] non-zero (ancestors of 3: 1, 0)
        // M[4,0] non-zero (ancestor of 4: 0)
        // M[2,3] = 0 (neither is ancestor of the other — siblings)
        // M[2,4] = 0 (4 is not ancestor of 2)
        // M[3,4] = 0 (4 is not ancestor of 3)
        let mut qm = DMatrix::zeros(nv, nv);
        qm[(0, 0)] = 10.0;
        qm[(1, 1)] = 8.0;
        qm[(2, 2)] = 5.0;
        qm[(3, 3)] = 6.0;
        qm[(4, 4)] = 7.0;
        // Ancestor pairs
        qm[(1, 0)] = 2.0;
        qm[(0, 1)] = 2.0;
        qm[(2, 0)] = 1.0;
        qm[(0, 2)] = 1.0;
        qm[(2, 1)] = 1.5;
        qm[(1, 2)] = 1.5;
        qm[(3, 0)] = 0.5;
        qm[(0, 3)] = 0.5;
        qm[(3, 1)] = 1.0;
        qm[(1, 3)] = 1.0;
        qm[(4, 0)] = 0.8;
        qm[(0, 4)] = 0.8;
        // Non-ancestor pairs are zero (enforced by zeros init)

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);

        // Verify zero entries in L for non-ancestor pairs
        // DOF 2's ancestors: [0, 1] — should not have entry for 3 or 4
        // rownnz includes diagonal (last element is self-index), so off-diag = 0..rownnz-1
        let (rowadr, rownnz, colind) = model.qld_csr();
        let nnz_offdiag_2 = rownnz[2] - 1;
        for k in 0..nnz_offdiag_2 {
            let col = colind[rowadr[2] + k];
            assert!(
                col == 0 || col == 1,
                "DOF 2 should only have ancestors 0 and 1"
            );
        }
        // Last element should be self-index (diagonal)
        assert_eq!(colind[rowadr[2] + nnz_offdiag_2], 2);
        // DOF 4's ancestors: [0] — should not have entry for 1, 2, or 3
        assert_eq!(model.qLD_rownnz[4], 2); // 1 off-diag + 1 diagonal
        assert_eq!(model.qLD_colind[model.qLD_rowadr[4]], 0);

        assert_solve_matches(&model, &data, &qm, nv);
    }

    #[test]
    fn invalidation_flag() {
        let nv = 2;
        let dof_parent = vec![None, Some(0)];
        let mut qm = DMatrix::zeros(2, 2);
        qm[(0, 0)] = 4.0;
        qm[(0, 1)] = 1.0;
        qm[(1, 0)] = 1.0;
        qm[(1, 1)] = 3.0;

        let (_, mut data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);

        // Simulate invalidation (as mj_crba would do)
        data.qLD_valid = false;
        assert!(!data.qLD_valid);
    }

    #[test]
    fn batch_solve_matches_single() {
        // nv=3 branching tree, solve 3 different RHS via batch and single.
        let nv = 3;
        let dof_parent = vec![None, Some(0), Some(0)];
        let mut qm = DMatrix::zeros(3, 3);
        qm[(0, 0)] = 5.0;
        qm[(0, 1)] = 2.0;
        qm[(1, 0)] = 2.0;
        qm[(1, 1)] = 4.0;
        qm[(0, 2)] = 1.0;
        qm[(2, 0)] = 1.0;
        qm[(2, 2)] = 3.0;

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        let (rowadr, rownnz, colind) = model.qld_csr();
        let n_rhs = 3;

        // Build nv × n_rhs matrix of RHS vectors.
        let mut batch_rhs = DMatrix::zeros(nv, n_rhs);
        for v in 0..n_rhs {
            for i in 0..nv {
                batch_rhs[(i, v)] = (i as f64 + 1.0) * (v as f64 + 0.5);
            }
        }

        // Single-vector solve for each column.
        let mut single_results = Vec::new();
        for v in 0..n_rhs {
            let mut x = batch_rhs.column(v).clone_owned();
            mj_solve_sparse(
                rowadr,
                rownnz,
                colind,
                &data.qLD_data,
                &data.qLD_diag_inv,
                &mut x,
            );
            single_results.push(x);
        }

        // Batch solve.
        let mut batch_x = batch_rhs;
        mj_solve_sparse_batch(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut batch_x,
        );

        // Compare: each batch column must match the corresponding single solve.
        for v in 0..n_rhs {
            for i in 0..nv {
                assert_relative_eq!(
                    batch_x[(i, v)],
                    single_results[v][i],
                    epsilon = 1e-14,
                    max_relative = 1e-14
                );
            }
        }
    }
}

/// MuJoCo 3.5.0's own factors (unfused build), from
/// `scripts/gen_implicit_reference.py`'s `factor` case: a hinge–hinge–ball
/// chain with a hinge off its root, after one step under implicitfast and
/// under implicit, the inputs MuJoCo factored and what it made of them.
#[cfg(test)]
#[allow(clippy::unreadable_literal, clippy::expect_used)]
mod mujoco_factor_tests {
    use super::{mj_factor_i, mj_solve_ld};
    use crate::linalg::{lu_factor_sparse, lu_solve_sparse};
    use crate::types::StepError;

    // implicitfast: M's pattern
    const M_ROWNNZ: [usize; 6] = [1, 2, 3, 4, 5, 2];
    const M_ROWADR: [usize; 6] = [0, 1, 3, 6, 10, 15];
    const M_COLIND: [usize; 17] = [0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 0, 1, 2, 3, 4, 0, 5];
    const QH_IN: [f64; 17] = [
        0.3185262992163552,
        0.0006530479446890686,
        0.08218841172669453,
        0.016734050001405636,
        0.00349468013468013,
        0.004073333333333321,
        0.005750755837432276,
        -0.010253602693602676,
        -0.00012000000000000205,
        0.004606666666666655,
        0.0074397118874488495,
        6.45117845117835e-05,
        0.000959999999999997,
        0.00031999999999999954,
        0.0013533333333333316,
        8.673617379884035e-19,
        0.013383333333333323,
    ];
    const QH: [f64; 17] = [
        0.23300582721760135,
        0.0007078978910017373,
        0.05680628467720641,
        3.4946805310635383,
        0.7910524795470573,
        0.0033657743889327703,
        0.8809557919545797,
        -2.2663546656762543,
        -0.07658242068870592,
        0.004531001642036113,
        5.497324054765167,
        0.04766880628949526,
        0.7093596059113287,
        0.2364532019704433,
        0.0013533333333333316,
        6.480909623823693e-17,
        0.013383333333333323,
    ];
    const QH_DIAG_INV: [f64; 6] = [
        4.291738159261193,
        17.603686030205232,
        297.108446510309,
        220.70175184280583,
        738.9162561576364,
        74.71980074719806,
    ];
    const Y: [f64; 6] = [0.7, -1.3, 0.4, 2.1, -0.9, 1.6];
    const X: [f64; 6] = [
        -2.746575156457358,
        53.21763556472989,
        328.64887090375686,
        658.6391252241617,
        -1041.3302016767364,
        119.5516811955169,
    ];
    // implicit: D's pattern
    const D_ROWNNZ: [usize; 6] = [6, 5, 5, 5, 5, 2];
    const D_ROWADR: [usize; 6] = [0, 6, 11, 16, 21, 26];
    const D_DIAG: [usize; 6] = [0, 1, 2, 3, 4, 1];
    const D_COLIND: [usize; 28] = [
        0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 5,
    ];
    const QLU_IN: [f64; 28] = [
        0.3183006743717101,
        0.0014665918070532386,
        0.01677126476286876,
        0.005428790371425388,
        0.007599069927127666,
        4.253305988839993e-05,
        -0.00017838854249614363,
        0.08228542540557082,
        0.003404284598352916,
        -0.01033646203421335,
        4.6451713735106315e-06,
        0.016654366232482974,
        0.003597464223195896,
        0.004065833333333321,
        -0.00018392459360537197,
        0.0009814903147158834,
        0.005977300644448057,
        -0.010270496484305133,
        -6.460873972796534e-05,
        0.004605046666666655,
        0.00035283459510958455,
        0.007435138357274059,
        9.652735387901272e-05,
        0.0009595096852841106,
        0.0003011854048904144,
        0.0013624533333333314,
        -4.253305988839818e-05,
        0.013383333333333323,
    ];
    const QLU: [f64; 28] = [
        0.2328009214818817,
        0.0057328456367309304,
        3.4894730046927607,
        0.8281182823276995,
        5.577490062383307,
        0.003178061760029885,
        -0.0003068327352869337,
        0.056675743626740396,
        0.8025558395201723,
        -2.2834940938545305,
        0.0034094168657842505,
        0.011657017662023057,
        0.002616207404909691,
        0.003346891364883099,
        -0.08855524459571222,
        0.7203845377328285,
        0.0040518225618672195,
        -0.010295494177062488,
        -0.0003130930019247949,
        0.0045270486701206505,
        0.2589700406445126,
        0.007435138357274059,
        9.652735387901272e-05,
        0.0009595096852841106,
        0.0003011854048904144,
        0.0013624533333333314,
        -4.253305988839818e-05,
        0.013383333333333323,
    ];

    fn assert_bits(what: &str, got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "{what}: length");
        for (i, (a, b)) in got.iter().zip(want).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "{what}[{i}]: {a}, MuJoCo {b}");
        }
    }

    /// `mj_factorI` on M's pattern: the factor and its inverse diagonal, bit
    /// for bit; a row with four ancestors among them.
    #[test]
    fn mj_factor_i_is_mujocos() {
        let mut qh = QH_IN;
        let mut diag_inv = [0.0; 6];
        mj_factor_i(
            &mut qh,
            &mut diag_inv,
            &M_ROWNNZ,
            &M_ROWADR,
            &M_COLIND,
            None,
        );
        assert_bits("qH", &qh, &QH);
        assert_bits("qHDiagInv", &diag_inv, &QH_DIAG_INV);
    }

    /// `mj_solveLD` with that factor (MuJoCo's `mj_solveM` after `mj_factorM`
    /// of the same input), bit for bit.
    #[test]
    fn mj_solve_ld_is_mujocos() {
        let mut x = Y;
        mj_solve_ld(
            &mut x,
            &QH,
            &QH_DIAG_INV,
            &M_ROWNNZ,
            &M_ROWADR,
            &M_COLIND,
            None,
        );
        assert_bits("x", &x, &X);
    }

    /// `mju_factorLUSparse` on D's pattern, bit for bit; and its solve undoes
    /// the product with the input (MuJoCo's solve is not callable from its
    /// Python bindings).
    #[test]
    fn lu_factor_sparse_is_mujocos() {
        let mut lu = QLU_IN;
        lu_factor_sparse(&mut lu, &D_ROWNNZ, &D_ROWADR, &D_COLIND, None).expect("factor");
        assert_bits("qLU", &lu, &QLU);

        let mut y = [0.0; 6];
        for (i, yi) in y.iter_mut().enumerate() {
            for adr in D_ROWADR[i]..D_ROWADR[i] + D_ROWNNZ[i] {
                *yi += QLU_IN[adr] * X[D_COLIND[adr]];
            }
        }
        let mut x = [0.0; 6];
        lu_solve_sparse(
            &mut x, &lu, &y, &D_ROWNNZ, &D_ROWADR, &D_DIAG, &D_COLIND, None,
        );
        for (a, b) in x.iter().zip(X) {
            assert!((a - b).abs() <= 1e-9 * b.abs(), "{a}, expected {b}");
        }
    }

    /// A pivot below mjMINVAL in magnitude is refused, where MuJoCo stops
    /// with "diagonal element too small" (`engine_util_solve.c:788-790`); a
    /// negative one above it is not.
    #[test]
    fn lu_factor_sparse_refuses_a_pivot_below_mjminval() {
        let one = |v: f64| lu_factor_sparse(&mut [v], &[1], &[0], &[0], None);
        assert_eq!(one(9e-16), Err(StepError::LuSingular));
        assert_eq!(one(-9e-16), Err(StepError::LuSingular));
        assert_eq!(one(-2e-15), Ok(()));
    }

    /// With an index (the awake dofs), only those rows are factored: row 5,
    /// left out, keeps its input, and row 4, which row 5's elimination does
    /// not reach, comes out as factored whole.
    #[test]
    fn mj_factor_i_takes_only_the_indexed_rows() {
        let mut whole = QH_IN;
        let mut whole_inv = [0.0; 6];
        mj_factor_i(
            &mut whole,
            &mut whole_inv,
            &M_ROWNNZ,
            &M_ROWADR,
            &M_COLIND,
            None,
        );
        let mut part = QH_IN;
        let mut part_inv = [0.0; 6];
        let index = [0, 1, 2, 3, 4];
        mj_factor_i(
            &mut part,
            &mut part_inv,
            &M_ROWNNZ,
            &M_ROWADR,
            &M_COLIND,
            Some(&index),
        );
        let row5 = M_ROWADR[5]..M_ROWADR[5] + M_ROWNNZ[5];
        assert_bits("row 5 left out", &part[row5.clone()], &QH_IN[row5]);
        assert_eq!(part_inv[5].to_bits(), 0.0_f64.to_bits());
        assert_bits("row 4", &part[10..15], &whole[10..15]);
    }
}
