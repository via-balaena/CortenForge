//! The primal solver's sparse structures at MuJoCo's sparse size
//! ([`Model::is_sparse`](crate::types::Model::is_sparse)): the rows'
//! Jacobian as MuJoCo keeps it ([`SparseJ`]), and Newton's Hessian as MuJoCo
//! makes, factors, updates and solves it ([`SparseHessian`]): `H = J'·D·J +
//! M` on its lower triangle (`MakeHessian`, `FactorizeHessian`,
//! `engine_solver.c:1516-1670`, through `mju_sqrMatTDSparse` and
//! `mju_addToMatSparse`), factored in reverse order as `H = L'·L`
//! (`mju_cholFactorSymbolic`, `mju_cholFactorNumeric`,
//! `engine_util_solve.c:195-371`), updated by rank-one rotations
//! (`mju_cholUpdateSparse`, `:416-478`) and solved (`mju_cholSolveSparse`,
//! `:374-413`), operation for operation.

use crate::constraint::impedance::MJ_MINVAL;
use crate::linalg::dot_sparse;
use crate::types::Data;

use super::primal::Subproblem;

/// A subproblem's Jacobian at the sparse size in MuJoCo's compressed form, in
/// its numbering (`PrimalAllocate`, `engine_solver.c:960-986`): each row's
/// dof chain and the values there, and the transpose, each dof's rows in row
/// order with their values (`mju_transposeSparse`). A row's chain dofs are in
/// its island, in the island's order, which keeps the chain's.
pub(super) struct SparseJ {
    pub(super) rownnz: Vec<usize>,
    pub(super) rowadr: Vec<usize>,
    pub(super) colind: Vec<usize>,
    pub(super) val: Vec<f64>,
    pub(super) t_rownnz: Vec<usize>,
    pub(super) t_rowadr: Vec<usize>,
    pub(super) t_colind: Vec<usize>,
    pub(super) t_val: Vec<f64>,
}

impl SparseJ {
    pub(super) fn of(data: &Data, sub: &Subproblem) -> Self {
        let (mut rownnz, mut rowadr) = (Vec::new(), Vec::new());
        let (mut colind, mut val) = (Vec::new(), Vec::new());
        for &r in &sub.rows {
            let adr = data.efc_J_rowadr[r];
            rowadr.push(colind.len());
            rownnz.push(data.efc_J_rownnz[r]);
            for &c in &data.efc_J_colind[adr..adr + data.efc_J_rownnz[r]] {
                colind.push(sub.local(data, c));
                val.push(data.efc_J[(r, c)]);
            }
        }
        Self::with_transpose(rownnz, rowadr, colind, val, sub.dofs.len())
    }

    /// The rows given, over `nv` dofs, and their transpose.
    fn with_transpose(
        rownnz: Vec<usize>,
        rowadr: Vec<usize>,
        colind: Vec<usize>,
        val: Vec<f64>,
        nv: usize,
    ) -> Self {
        let mut t_rownnz = vec![0; nv];
        for &c in &colind {
            t_rownnz[c] += 1;
        }
        let t_rowadr = row_addresses(&t_rownnz);
        let mut next = t_rowadr.clone();
        let (mut t_colind, mut t_val) = (vec![0; colind.len()], vec![0.0; colind.len()]);
        for (r, (&adr, &nnz)) in rowadr.iter().zip(&rownnz).enumerate() {
            for k in adr..adr + nnz {
                let c = colind[k];
                t_colind[next[c]] = r;
                t_val[next[c]] = val[k];
                next[c] += 1;
            }
        }
        Self {
            rownnz,
            rowadr,
            colind,
            val,
            t_rownnz,
            t_rowadr,
            t_colind,
            t_val,
        }
    }

    /// Row `r`'s chain and its values there.
    pub(super) fn row(&self, r: usize) -> (&[usize], &[f64]) {
        let span = self.rowadr[r]..self.rowadr[r] + self.rownnz[r];
        (&self.colind[span.clone()], &self.val[span])
    }
}

/// Each row's start, from the rows' lengths.
fn row_addresses(rownnz: &[usize]) -> Vec<usize> {
    let mut adr = Vec::with_capacity(rownnz.len());
    let mut total = 0;
    for &n in rownnz {
        adr.push(total);
        total += n;
    }
    adr
}

/// A symmetric matrix's lower triangle and diagonal by rows, each row's
/// columns ascending and its diagonal last.
pub(super) struct LowerRows {
    pub(super) rownnz: Vec<usize>,
    pub(super) rowadr: Vec<usize>,
    pub(super) colind: Vec<usize>,
    pub(super) val: Vec<f64>,
}

/// Newton's Hessian at the sparse size: `H` by rows, its reverse-order
/// Cholesky factor `L` by rows (CSR) and by columns (`LT`, CSC, with
/// `lt_map[k]` the place in `l` of the entry `lt_colind[k]` names), and
/// `Lcone`, `L` with the cone contacts' contributions (MuJoCo's
/// `mjPrimalContext`'s `H_*`, `L_*`, `LT_*`, `Lcone`).
pub(super) struct SparseHessian {
    nv: usize,
    h_rownnz: Vec<usize>,
    h_rowadr: Vec<usize>,
    h_colind: Vec<usize>,
    h: Vec<f64>,
    /// Which of `H`'s entries `J'·D·J` has, and where `M` has each, if it
    /// does.
    h_jdj: Vec<bool>,
    h_mass: Vec<usize>,
    l_rownnz: Vec<usize>,
    l_rowadr: Vec<usize>,
    l_colind: Vec<usize>,
    l: Vec<f64>,
    lt_rownnz: Vec<usize>,
    lt_rowadr: Vec<usize>,
    lt_colind: Vec<usize>,
    lt_map: Vec<usize>,
    lcone: Vec<f64>,
    /// The entries of `J'·J`'s lower triangle, MuJoCo's
    /// `mju_sqrMatTDSparseCount` (a dof no row has adds none).
    jdj_count: usize,
}

/// No entry of `M` at this place of `H`.
const NO_MASS: usize = usize::MAX;

impl SparseHessian {
    /// `MakeHessian`'s sparse branch: `H`'s pattern, the lower triangle of
    /// `J'·J` over every row whatever its state (`mju_sqrMatTDSparseCount`),
    /// each dof's diagonal, and `M`'s, merged (`mju_addToMatSparse`); its
    /// values at `d` ([`Self::fill`]); and the factor's pattern from `H`'s
    /// upper triangle (`mju_cholFactorSymbolic`).
    pub(super) fn make(jac: &SparseJ, d_quad: &[f64], mass: &LowerRows) -> Self {
        let nv = mass.rownnz.len();
        let mut marker = vec![usize::MAX; nv];
        let (mut h_rownnz, mut h_colind) = (Vec::with_capacity(nv), Vec::new());
        let (mut h_jdj, mut h_mass) = (Vec::new(), Vec::new());
        let mut cols = Vec::new();
        let mut jdj_count = 0;
        for r in 0..nv {
            // The columns c ≤ r that a row with dof r also has.
            cols.clear();
            for k in jac.t_rowadr[r]..jac.t_rowadr[r] + jac.t_rownnz[r] {
                let (chain, _) = jac.row(jac.t_colind[k]);
                for &c in chain.iter().take_while(|&&c| c <= r) {
                    if marker[c] != r {
                        marker[c] = r;
                        cols.push(c);
                    }
                }
            }
            jdj_count += cols.len();
            // Every dof's diagonal, as `mju_sqrMatTDSparse` writes it (0 for
            // a dof no row has).
            if marker[r] != r {
                cols.push(r);
            }
            cols.sort_unstable();
            // Merged with M's row (ascending, diagonal last).
            let m_row = mass.rowadr[r]..mass.rowadr[r] + mass.rownnz[r];
            let (mut next_jdj, mut next_mass) = (0, m_row.start);
            let start = h_colind.len();
            loop {
                let from_jdj = cols.get(next_jdj).copied();
                let from_mass = (next_mass < m_row.end).then(|| mass.colind[next_mass]);
                let col = match (from_jdj, from_mass) {
                    (Some(lhs), Some(rhs)) => lhs.min(rhs),
                    (Some(only), None) | (None, Some(only)) => only,
                    (None, None) => break,
                };
                h_colind.push(col);
                h_jdj.push(from_jdj == Some(col));
                h_mass.push(if from_mass == Some(col) {
                    next_mass
                } else {
                    NO_MASS
                });
                if from_jdj == Some(col) {
                    next_jdj += 1;
                }
                if from_mass == Some(col) {
                    next_mass += 1;
                }
            }
            h_rownnz.push(h_colind.len() - start);
        }
        let h_rowadr = row_addresses(&h_rownnz);
        let nh = h_colind.len();
        let mut hess = Self {
            nv,
            h_rownnz,
            h_rowadr,
            h_colind,
            h: vec![0.0; nh],
            h_jdj,
            h_mass,
            l_rownnz: Vec::new(),
            l_rowadr: Vec::new(),
            l_colind: Vec::new(),
            l: Vec::new(),
            lt_rownnz: Vec::new(),
            lt_rowadr: Vec::new(),
            lt_colind: Vec::new(),
            lt_map: Vec::new(),
            lcone: Vec::new(),
            jdj_count,
        };
        hess.symbolic();
        hess.fill(jac, d_quad, mass);
        hess
    }

    /// `H = J'·D·J + M` at `d`, as `mju_sqrMatTDSparse` and
    /// `mju_addToMatSparse` make it (`engine_util_sparse.c:755-948`,
    /// `:204-213`): by column `c`, its diagonal `Σ (J·J)·D` over its rows in
    /// row order from 0, and each entry below it `J[r,j]·(D[r]·J[r,c])` over
    /// the rows of `c` in row order, the first set and the rest added; then
    /// `M` added where it has an entry.
    pub(super) fn fill(&mut self, jac: &SparseJ, d_quad: &[f64], mass: &LowerRows) {
        let nv = self.nv;
        let mut buffer = vec![0.0; nv];
        let mut set = vec![false; nv];
        let mut touched = Vec::new();
        for c in 0..nv {
            let rows = jac.t_rowadr[c]..jac.t_rowadr[c] + jac.t_rownnz[c];
            let mut diag = 0.0;
            for k in rows.clone() {
                let v = jac.t_val[k];
                diag += (v * v) * d_quad[jac.t_colind[k]];
            }
            for k in rows {
                let r = jac.t_colind[k];
                let v = d_quad[r] * jac.t_val[k];
                let (chain, vals) = jac.row(r);
                for (&col, &val) in chain.iter().zip(vals) {
                    if col <= c {
                        continue;
                    }
                    if set[col] {
                        buffer[col] += val * v;
                    } else {
                        set[col] = true;
                        touched.push(col);
                        buffer[col] = val * v;
                    }
                }
            }
            for &row in &touched {
                set[row] = false;
                let at = self.place(row, c);
                self.h[at] = buffer[row];
            }
            touched.clear();
            let at = self.place(c, c);
            self.h[at] = diag;
        }
        for (k, (&has_jdj, &m)) in self.h_jdj.iter().zip(&self.h_mass).enumerate() {
            if m == NO_MASS {
                continue;
            }
            self.h[k] = if has_jdj {
                self.h[k] + mass.val[m]
            } else {
                mass.val[m]
            };
        }
    }

    /// MuJoCo's `solver_nnz` for this Hessian (`engine_solver.c:1953-1958`):
    /// `L`'s entries, twice under an elliptic cone (`Lcone` beside it), and
    /// `H`'s count as `MakeHessian` keeps it, `J'·J`'s lower triangle and all
    /// of `M`'s `mass_nnz` entries (its overcount, `:1532-1533`).
    pub(super) fn solver_nnz(&self, mass_nnz: usize, elliptic: bool) -> usize {
        (1 + usize::from(elliptic)) * self.l.len() + self.jdj_count + mass_nnz
    }

    /// The place in `h` of the entry at row `row`, column `col`.
    fn place(&self, row: usize, col: usize) -> usize {
        let adr = self.h_rowadr[row];
        let cols = &self.h_colind[adr..adr + self.h_rownnz[row]];
        adr + cols
            .binary_search(&col)
            // `fill` writes only the entries `make` put in the pattern.
            .unwrap_or_else(|_| unreachable!("in the pattern"))
    }

    /// `mju_cholFactorSymbolic` (`engine_util_solve.c:195-294`): `L`'s and
    /// `LT`'s patterns from `H`'s upper triangle, by the elimination tree,
    /// counted and then filled, rows in reverse order.
    // L's and LT's arrays keep MuJoCo's names (`L_rownnz`, `LT_rownnz`, …).
    #[allow(clippy::similar_names)]
    fn symbolic(&mut self) {
        let n = self.nv;
        // H's upper triangle: row r lists the rows i of H with an entry at
        // column r, in ascending order (`mju_transposeSparse`).
        let mut ht: Vec<Vec<usize>> = vec![Vec::new(); n];
        for i in 0..n {
            for &c in &self.h_colind[self.h_rowadr[i]..self.h_rowadr[i] + self.h_rownnz[i]] {
                ht[c].push(i);
            }
        }
        let mut parent = vec![None; n];
        let mut flag = vec![0; n];
        let mut l_rownnz = vec![0; n];
        let mut lt_rownnz = vec![0; n];
        for r in (0..n).rev() {
            parent[r] = None;
            flag[r] = r;
            l_rownnz[r] = 1;
            lt_rownnz[r] = 1;
            for &start in &ht[r] {
                let mut i = Some(start).filter(|&i| i > r);
                while let Some(row) = i.filter(|&row| flag[row] != r) {
                    if parent[row].is_none() {
                        parent[row] = Some(r);
                    }
                    l_rownnz[row] += 1;
                    lt_rownnz[r] += 1;
                    flag[row] = r;
                    i = parent[row];
                }
            }
        }
        let l_rowadr = row_addresses(&l_rownnz);
        let lt_rowadr = row_addresses(&lt_rownnz);
        let nnz = l_rownnz.iter().sum();
        let mut l_colind = vec![0; nnz];
        let mut lt_colind = vec![0; nnz];
        let mut lt_map = vec![0; nnz];
        let mut cursor: Vec<usize> = (0..n).map(|r| l_rowadr[r] + l_rownnz[r] - 1).collect();
        let mut lt_write = lt_rowadr.clone();
        parent.fill(None);
        for r in (0..n).rev() {
            parent[r] = None;
            flag[r] = r;
            let diag = l_rowadr[r] + l_rownnz[r] - 1;
            l_colind[diag] = r;
            lt_colind[lt_write[r]] = r;
            lt_map[lt_write[r]] = diag;
            lt_write[r] += 1;
            for &start in &ht[r] {
                let mut i = Some(start).filter(|&i| i > r);
                while let Some(row) = i.filter(|&row| flag[row] != r) {
                    if parent[row].is_none() {
                        parent[row] = Some(r);
                    }
                    cursor[row] -= 1;
                    l_colind[cursor[row]] = r;
                    lt_colind[lt_write[r]] = row;
                    lt_map[lt_write[r]] = cursor[row];
                    lt_write[r] += 1;
                    flag[row] = r;
                    i = parent[row];
                }
            }
        }
        self.l_rownnz = l_rownnz;
        self.l_rowadr = l_rowadr;
        self.l_colind = l_colind;
        self.l = vec![0.0; nnz];
        self.lt_rownnz = lt_rownnz;
        self.lt_rowadr = lt_rowadr;
        self.lt_colind = lt_colind;
        self.lt_map = lt_map;
        self.lcone = vec![0.0; nnz];
    }

    /// `L` from `H`, `mju_cholFactorNumeric` with mjMINVAL for the smallest
    /// diagonal (`engine_util_solve.c:299-371`); returns the rank.
    pub(super) fn factor(&mut self) -> usize {
        let n = self.nv;
        let mut rank = n;
        let mut dense = vec![0.0; n];
        for r in (0..n).rev() {
            for k in self.h_rowadr[r]..self.h_rowadr[r] + self.h_rownnz[r] {
                dense[self.h_colind[k]] = self.h[k];
            }
            let lt = self.lt_rowadr[r];
            for k in 1..self.lt_rownnz[r] {
                let c = self.lt_colind[lt + k];
                let at = self.lt_map[lt + k];
                let l_cr = self.l[at];
                let c_adr = self.l_rowadr[c];
                for i in c_adr..=at {
                    dense[self.l_colind[i]] -= l_cr * self.l[i];
                }
            }
            let mut diag = dense[r];
            if diag < MJ_MINVAL {
                diag = MJ_MINVAL;
                rank -= 1;
            }
            let l_rr = diag.sqrt();
            let l_rr_inv = 1.0 / l_rr;
            let (adr, nnz) = (self.l_rowadr[r], self.l_rownnz[r]);
            for i in adr..adr + nnz - 1 {
                self.l[i] = dense[self.l_colind[i]] * l_rr_inv;
            }
            self.l[adr + nnz - 1] = l_rr;
            for i in adr..adr + nnz {
                dense[self.l_colind[i]] = 0.0;
            }
        }
        rank
    }

    /// `Lcone = L`, `HessianCone`'s start.
    pub(super) fn copy_to_cone(&mut self) {
        self.lcone.copy_from_slice(&self.l);
    }

    /// `L'·L ± x·x'` in place, on `Lcone` with `cone`, else on `L`, `x` given
    /// at the ascending dofs `x_ind`: `mju_cholUpdateSparse`'s Givens
    /// rotations from `x`'s last dof down (`engine_util_solve.c:416-478`);
    /// returns the rank.
    // Single-letter names (`x`, `c`, `s`, `r`) are MuJoCo's own, the rotation's.
    #[allow(clippy::many_single_char_names)]
    pub(super) fn update(&mut self, cone: bool, x: &[f64], x_ind: &[usize], plus: bool) -> usize {
        let n = self.nv;
        let Some(&start) = x_ind.last() else {
            return n;
        };
        let mut dense = vec![0.0; start + 1];
        for (&i, &v) in x_ind.iter().zip(x) {
            dense[i] = v;
        }
        let mat = if cone { &mut self.lcone } else { &mut self.l };
        let mut rank = n;
        for row in (0..=start).rev() {
            if dense[row] == 0.0 {
                continue;
            }
            let nnz = self.l_rownnz[row] - 1;
            let adr = self.l_rowadr[row];
            let diag = mat[adr + nnz];
            let x_row = dense[row];
            let mut tmp = diag * diag + if plus { x_row * x_row } else { -x_row * x_row };
            if tmp < MJ_MINVAL {
                tmp = MJ_MINVAL;
                rank -= 1;
            }
            let r = tmp.sqrt();
            mat[adr + nnz] = r;
            let c = diag / r;
            let s = -x_row / r;
            let s_signed = if plus { -s } else { s };
            for i in 0..nnz {
                let j = self.l_colind[adr + i];
                let dense_j = dense[j];
                let mat_val = mat[adr + i];
                mat[adr + i] = c * mat_val + s_signed * dense_j;
                dense[j] = s * mat_val + c * dense_j;
            }
        }
        rank
    }

    /// `x ← H⁻¹·x` through `Lcone` with `cone`, else `L`:
    /// `mju_cholSolveSparse` (`engine_util_solve.c:374-413`), `L⁻ᵀ` from the
    /// last row, then `L⁻¹` with each row's `mju_dotSparse`.
    pub(super) fn solve(&self, cone: bool, x: &mut [f64]) {
        let mat = if cone { &self.lcone } else { &self.l };
        for i in (0..self.nv).rev() {
            if x[i] != 0.0 {
                let (adr, nnz) = (self.l_rowadr[i], self.l_rownnz[i]);
                x[i] /= mat[adr + nnz - 1];
                let tmp = x[i];
                for j in 0..nnz - 1 {
                    x[self.l_colind[adr + j]] -= mat[adr + j] * tmp;
                }
            }
        }
        for i in 0..self.nv {
            let (adr, nnz) = (self.l_rowadr[i], self.l_rownnz[i]);
            if nnz > 1 {
                let sum = dot_sparse(&mat[adr..adr + nnz - 1], |k| x[self.l_colind[adr + k]]);
                x[i] -= sum;
            }
            x[i] /= mat[adr + nnz - 1];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A Hessian made at one `D` and filled at another is, bit for bit, the
    /// one made at the second, and so is its factor: `fill` rewrites every
    /// entry, M's alone among them (row 1, column 0, which no row of `J`
    /// has). It is how `factorize_hessian` rebuilds after an update loses
    /// rank.
    #[test]
    fn a_hessian_filled_again_is_made_anew() {
        // Rows: dof 0; dofs 1 and 2.
        let jac = SparseJ::with_transpose(
            vec![1, 2],
            vec![0, 1],
            vec![0, 1, 2],
            vec![2.0, 1.5, -0.5],
            3,
        );
        let mass = LowerRows {
            rownnz: vec![1, 2, 1],
            rowadr: vec![0, 1, 3],
            colind: vec![0, 0, 1, 2],
            val: vec![1.0, 0.3, 2.0, 1.5],
        };
        let mut filled = SparseHessian::make(&jac, &[0.7, 1.9], &mass);
        filled.fill(&jac, &[0.2, 0.4], &mass);
        let mut made = SparseHessian::make(&jac, &[0.2, 0.4], &mass);
        let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
        assert!(made.h.iter().all(|x| x.is_finite()), "H is finite");
        assert_eq!(bits(&filled.h), bits(&made.h), "H");
        assert_eq!(filled.factor(), 3, "full rank");
        assert_eq!(made.factor(), 3, "full rank");
        assert!(made.l.iter().all(|x| x.is_finite()), "L is finite");
        assert_eq!(bits(&filled.l), bits(&made.l), "L");
    }
}
