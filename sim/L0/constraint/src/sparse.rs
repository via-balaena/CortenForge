//! Sparse matrix operations for constraint solvers.
//!
//! This module provides sparse matrix representations optimized for constraint
//! Jacobians, which are typically very sparse (each constraint only affects
//! 2 bodies out of potentially thousands).
//!
//! # Sparsity Pattern
//!
//! For a system with N bodies and M constraints, the Jacobian has dimensions:
//! - Rows: total constraint DOF (sum of each constraint's DOF)
//! - Columns: 6 * N (3 linear + 3 angular per body)
//!
//! Each constraint row only has non-zeros in columns corresponding to the
//! parent and child bodies (12 non-zeros per constraint row max).
//!
//! # Performance Benefits
//!
//! - **Memory**: O(M * 12) instead of O(M * 6N) for dense
//! - **Matrix-vector multiply**: O(M * 12) instead of O(M * 6N)
//! - **Effective mass computation**: Can exploit sparsity pattern

use nalgebra::{DMatrix, DVector, Vector3};
use nalgebra_sparse::{CooMatrix, CscMatrix, CsrMatrix};

/// Sparse constraint Jacobian in CSR format for efficient row operations.
///
/// CSR (Compressed Sparse Row) is optimal for:
/// - Row-wise iteration (iterating over constraints)
/// - Matrix-vector products J * v
#[derive(Debug, Clone)]
pub struct SparseJacobian {
    /// The sparse matrix in CSR format.
    matrix: CsrMatrix<f64>,
    /// Number of constraint rows.
    num_rows: usize,
    /// Number of columns (6 * `num_bodies`).
    num_cols: usize,
}

impl SparseJacobian {
    /// Build a sparse Jacobian from triplets.
    ///
    /// # Arguments
    ///
    /// * `num_rows` - Total number of constraint rows
    /// * `num_cols` - Total number of columns (6 * `num_bodies`)
    /// * `triplets` - (row, col, value) entries
    #[must_use]
    pub fn from_triplets(
        num_rows: usize,
        num_cols: usize,
        triplets: &[(usize, usize, f64)],
    ) -> Self {
        let mut coo = CooMatrix::new(num_rows, num_cols);

        for &(row, col, val) in triplets {
            if val.abs() > 1e-15 {
                // Skip near-zero values
                coo.push(row, col, val);
            }
        }

        let matrix = CsrMatrix::from(&coo);

        Self {
            matrix,
            num_rows,
            num_cols,
        }
    }

    /// Get the number of rows.
    #[must_use]
    pub const fn nrows(&self) -> usize {
        self.num_rows
    }

    /// Get the number of columns.
    #[must_use]
    pub const fn ncols(&self) -> usize {
        self.num_cols
    }

    /// Get the number of non-zero entries.
    #[must_use]
    pub fn nnz(&self) -> usize {
        self.matrix.nnz()
    }

    /// Compute J * v (matrix-vector product).
    #[must_use]
    pub fn mul_vec(&self, v: &DVector<f64>) -> DVector<f64> {
        let mut result = DVector::zeros(self.num_rows);

        // CSR format allows efficient row-wise iteration
        for (row_idx, row) in self.matrix.row_iter().enumerate() {
            let mut sum = 0.0;
            for (&col_idx, &val) in row.col_indices().iter().zip(row.values().iter()) {
                sum += val * v[col_idx];
            }
            result[row_idx] = sum;
        }

        result
    }

    /// Compute J^T * v (transpose matrix-vector product).
    #[must_use]
    pub fn mul_transpose_vec(&self, v: &DVector<f64>) -> DVector<f64> {
        let mut result = DVector::zeros(self.num_cols);

        // For CSR, transpose multiply requires column-wise accumulation
        for (row_idx, row) in self.matrix.row_iter().enumerate() {
            let v_row = v[row_idx];
            for (&col_idx, &val) in row.col_indices().iter().zip(row.values().iter()) {
                result[col_idx] += val * v_row;
            }
        }

        result
    }

    /// Convert to dense matrix (for testing or small systems).
    #[must_use]
    pub fn to_dense(&self) -> DMatrix<f64> {
        let mut dense = DMatrix::zeros(self.num_rows, self.num_cols);

        for (row_idx, row) in self.matrix.row_iter().enumerate() {
            for (&col_idx, &val) in row.col_indices().iter().zip(row.values().iter()) {
                dense[(row_idx, col_idx)] = val;
            }
        }

        dense
    }

    /// Get the underlying CSR matrix.
    #[must_use]
    pub const fn csr(&self) -> &CsrMatrix<f64> {
        &self.matrix
    }
}

/// Sparse effective mass matrix in CSC format.
///
/// The effective mass matrix A = J * M^-1 * J^T has a specific sparsity pattern
/// determined by which constraints share bodies. CSC (Compressed Sparse Column)
/// is optimal for:
/// - Cholesky factorization
/// - Triangular solves
#[derive(Debug, Clone)]
pub struct SparseEffectiveMass {
    /// The sparse matrix in CSC format.
    matrix: CscMatrix<f64>,
    /// Matrix dimension (square).
    size: usize,
    /// Regularization added to diagonal.
    regularization: f64,
}

impl SparseEffectiveMass {
    /// Build the effective mass matrix A = J * M^-1 * J^T.
    ///
    /// This exploits the block-diagonal structure of M^-1 for efficiency.
    ///
    /// # Arguments
    ///
    /// * `jacobian` - The sparse constraint Jacobian
    /// * `inv_mass_blocks` - Inverse mass for each body (6x6 blocks, stored as vectors)
    /// * `regularization` - Small positive value for numerical stability
    #[must_use]
    pub fn from_jacobian(
        jacobian: &SparseJacobian,
        inv_mass_blocks: &[InvMassBlock],
        regularization: f64,
    ) -> Self {
        let size = jacobian.nrows();
        let num_bodies = inv_mass_blocks.len();

        // Build A = J * M^-1 * J^T using triplet accumulation
        let mut coo = CooMatrix::new(size, size);

        // For each pair of constraint rows, compute their interaction
        // This is O(constraints^2 * avg_bodies_per_constraint^2) but with
        // good constant factors due to small body counts
        for (row_i, row) in jacobian.csr().row_iter().enumerate() {
            for (row_j, row2) in jacobian.csr().row_iter().enumerate().skip(row_i) {
                let mut dot = 0.0;

                // Find common bodies and compute dot product through M^-1
                for (&col_i, &val_i) in row.col_indices().iter().zip(row.values().iter()) {
                    let body_idx = col_i / 6;
                    if body_idx >= num_bodies {
                        continue;
                    }

                    let local_col = col_i % 6;

                    // Check if row_j also references this body
                    for (&col_j, &val_j) in row2.col_indices().iter().zip(row2.values().iter()) {
                        let body_idx_j = col_j / 6;
                        if body_idx_j != body_idx {
                            continue;
                        }

                        let local_col_j = col_j % 6;

                        // Get M^-1 element
                        let inv_mass_elem = inv_mass_blocks[body_idx].get(local_col, local_col_j);
                        dot += val_i * inv_mass_elem * val_j;
                    }
                }

                if dot.abs() > 1e-15 {
                    coo.push(row_i, row_j, dot);
                    if row_i != row_j {
                        coo.push(row_j, row_i, dot); // Symmetric
                    }
                }
            }
        }

        // Add regularization to diagonal
        for i in 0..size {
            coo.push(i, i, regularization);
        }

        let matrix = CscMatrix::from(&coo);

        Self {
            matrix,
            size,
            regularization,
        }
    }

    /// Solve A * x = b using Cholesky decomposition.
    ///
    /// Falls back to dense LU if Cholesky fails (non-SPD matrix).
    #[must_use]
    pub fn solve(&self, rhs: &DVector<f64>) -> DVector<f64> {
        // For now, convert to dense and use nalgebra's solvers
        // A full sparse Cholesky implementation would use a library like
        // sprs or faer, but this provides the interface for future optimization
        let dense = self.to_dense();

        dense.clone().cholesky().map_or_else(
            || {
                // Fall back to LU
                dense
                    .lu()
                    .solve(rhs)
                    .unwrap_or_else(|| DVector::zeros(rhs.len()))
            },
            |chol| chol.solve(rhs),
        )
    }

    /// Convert to dense matrix.
    #[must_use]
    pub fn to_dense(&self) -> DMatrix<f64> {
        let mut dense = DMatrix::zeros(self.size, self.size);

        for (col_idx, col) in self.matrix.col_iter().enumerate() {
            for (&row_idx, &val) in col.row_indices().iter().zip(col.values().iter()) {
                dense[(row_idx, col_idx)] = val;
            }
        }

        dense
    }

    /// Get the matrix size.
    #[must_use]
    pub const fn size(&self) -> usize {
        self.size
    }

    /// Get the regularization value.
    #[must_use]
    pub const fn regularization(&self) -> f64 {
        self.regularization
    }
}

/// Inverse mass block for a single body.
///
/// Stores the 6x6 inverse mass/inertia matrix efficiently.
/// The matrix is block-diagonal with structure:
/// ```text
/// [1/m * I_3   0      ]
/// [   0     I^-1_world]
/// ```
#[derive(Debug, Clone, Copy)]
pub struct InvMassBlock {
    /// Inverse mass (scalar, same for all 3 linear components).
    pub inv_mass: f64,
    /// Inverse inertia tensor in world frame (3x3).
    pub inv_inertia: [[f64; 3]; 3],
}

impl InvMassBlock {
    /// Create a new inverse mass block.
    #[must_use]
    pub const fn new(inv_mass: f64, inv_inertia: [[f64; 3]; 3]) -> Self {
        Self {
            inv_mass,
            inv_inertia,
        }
    }

    /// Create from mass and inertia matrices (nalgebra).
    #[must_use]
    pub fn from_nalgebra(inv_mass: f64, inv_inertia: &nalgebra::Matrix3<f64>) -> Self {
        let mut inv_i = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                inv_i[i][j] = inv_inertia[(i, j)];
            }
        }
        Self {
            inv_mass,
            inv_inertia: inv_i,
        }
    }

    /// Get element (row, col) of the 6x6 matrix.
    #[must_use]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        match (row, col) {
            // Linear part (upper-left 3x3 diagonal)
            (r, c) if r < 3 && c < 3 => {
                if r == c {
                    self.inv_mass
                } else {
                    0.0
                }
            }
            // Angular part (lower-right 3x3)
            (r, c) if r >= 3 && c >= 3 => self.inv_inertia[r - 3][c - 3],
            // Off-diagonal blocks are zero
            _ => 0.0,
        }
    }

    /// Create for a static body (zero inverse mass/inertia).
    #[must_use]
    pub const fn static_body() -> Self {
        Self {
            inv_mass: 0.0,
            inv_inertia: [[0.0; 3]; 3],
        }
    }
}

/// Builder for sparse Jacobians using triplet accumulation.
///
/// This is the preferred way to construct sparse Jacobians, as it allows
/// adding entries in any order and handles duplicates correctly.
#[derive(Debug, Clone)]
pub struct JacobianBuilder {
    triplets: Vec<(usize, usize, f64)>,
    num_rows: usize,
    num_cols: usize,
}

impl JacobianBuilder {
    /// Create a new Jacobian builder.
    #[must_use]
    pub fn new(num_rows: usize, num_cols: usize) -> Self {
        // Pre-allocate assuming ~12 non-zeros per constraint row
        let capacity = num_rows * 12;
        Self {
            triplets: Vec::with_capacity(capacity),
            num_rows,
            num_cols,
        }
    }

    /// Add a single entry.
    pub fn add(&mut self, row: usize, col: usize, value: f64) {
        debug_assert!(row < self.num_rows);
        debug_assert!(col < self.num_cols);
        if value.abs() > 1e-15 {
            self.triplets.push((row, col, value));
        }
    }

    /// Add a 3x3 block.
    pub fn add_block3(&mut self, row: usize, col: usize, block: &nalgebra::Matrix3<f64>) {
        for i in 0..3 {
            for j in 0..3 {
                self.add(row + i, col + j, block[(i, j)]);
            }
        }
    }

    /// Add a 3-vector as a row segment.
    pub fn add_vec3_row(&mut self, row: usize, col: usize, vec: &Vector3<f64>) {
        for i in 0..3 {
            self.add(row, col + i, vec[i]);
        }
    }

    /// Build the sparse Jacobian.
    #[must_use]
    pub fn build(self) -> SparseJacobian {
        SparseJacobian::from_triplets(self.num_rows, self.num_cols, &self.triplets)
    }
}

/// Threshold for switching between dense and sparse operations.
///
/// For small systems, dense operations may be faster due to better cache
/// utilization and SIMD. This threshold is conservative; actual crossover
/// depends on hardware.
pub const SPARSE_THRESHOLD_BODIES: usize = 16;

/// Check if sparse operations should be used.
#[must_use]
pub const fn should_use_sparse(num_bodies: usize, num_constraints: usize) -> bool {
    // Use sparse when:
    // 1. Enough bodies that dense would be wasteful
    // 2. Constraint-to-body ratio is reasonable (not fully connected)
    num_bodies >= SPARSE_THRESHOLD_BODIES && num_constraints < num_bodies * num_bodies / 4
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_sparse_jacobian_creation() {
        let triplets = vec![(0, 0, 1.0), (0, 1, 2.0), (1, 0, 3.0), (1, 2, 4.0)];

        let jacobian = SparseJacobian::from_triplets(2, 3, &triplets);

        assert_eq!(jacobian.nrows(), 2);
        assert_eq!(jacobian.ncols(), 3);
        assert_eq!(jacobian.nnz(), 4);
    }

    #[test]
    fn test_sparse_jacobian_mul_vec() {
        let triplets = vec![(0, 0, 1.0), (0, 1, 2.0), (1, 0, 3.0), (1, 1, 4.0)];

        let jacobian = SparseJacobian::from_triplets(2, 2, &triplets);
        let v = DVector::from_vec(vec![1.0, 2.0]);

        let result = jacobian.mul_vec(&v);

        // [1 2] [1]   [5]
        // [3 4] [2] = [11]
        assert_relative_eq!(result[0], 5.0, epsilon = 1e-10);
        assert_relative_eq!(result[1], 11.0, epsilon = 1e-10);
    }

    #[test]
    fn test_sparse_jacobian_transpose_mul() {
        let triplets = vec![(0, 0, 1.0), (0, 1, 2.0), (1, 0, 3.0), (1, 1, 4.0)];

        let jacobian = SparseJacobian::from_triplets(2, 2, &triplets);
        let v = DVector::from_vec(vec![1.0, 2.0]);

        let result = jacobian.mul_transpose_vec(&v);

        // [1 3] [1]   [7]
        // [2 4] [2] = [10]
        assert_relative_eq!(result[0], 7.0, epsilon = 1e-10);
        assert_relative_eq!(result[1], 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_sparse_to_dense() {
        let triplets = vec![(0, 0, 1.0), (0, 2, 3.0), (1, 1, 2.0)];

        let jacobian = SparseJacobian::from_triplets(2, 3, &triplets);
        let dense = jacobian.to_dense();

        assert_relative_eq!(dense[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(dense[(0, 1)], 0.0, epsilon = 1e-10);
        assert_relative_eq!(dense[(0, 2)], 3.0, epsilon = 1e-10);
        assert_relative_eq!(dense[(1, 0)], 0.0, epsilon = 1e-10);
        assert_relative_eq!(dense[(1, 1)], 2.0, epsilon = 1e-10);
        assert_relative_eq!(dense[(1, 2)], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_inv_mass_block() {
        let inv_mass = 0.5;
        let inv_inertia = [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]];

        let block = InvMassBlock::new(inv_mass, inv_inertia);

        // Linear part
        assert_relative_eq!(block.get(0, 0), 0.5, epsilon = 1e-10);
        assert_relative_eq!(block.get(1, 1), 0.5, epsilon = 1e-10);
        assert_relative_eq!(block.get(2, 2), 0.5, epsilon = 1e-10);
        assert_relative_eq!(block.get(0, 1), 0.0, epsilon = 1e-10);

        // Angular part
        assert_relative_eq!(block.get(3, 3), 1.0, epsilon = 1e-10);
        assert_relative_eq!(block.get(4, 4), 2.0, epsilon = 1e-10);
        assert_relative_eq!(block.get(5, 5), 3.0, epsilon = 1e-10);

        // Off-diagonal blocks
        assert_relative_eq!(block.get(0, 3), 0.0, epsilon = 1e-10);
        assert_relative_eq!(block.get(3, 0), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_jacobian_builder() {
        let mut builder = JacobianBuilder::new(2, 6);

        builder.add(0, 0, 1.0);
        builder.add(0, 3, 2.0);
        builder.add(1, 1, 3.0);
        builder.add(1, 4, 4.0);

        let jacobian = builder.build();

        assert_eq!(jacobian.nrows(), 2);
        assert_eq!(jacobian.ncols(), 6);
        assert_eq!(jacobian.nnz(), 4);
    }

    #[test]
    fn test_should_use_sparse() {
        // Small systems: dense
        assert!(!should_use_sparse(8, 10));

        // Large sparse systems: sparse
        assert!(should_use_sparse(100, 200));

        // Large but dense systems: dense
        assert!(!should_use_sparse(20, 400));
    }
}
