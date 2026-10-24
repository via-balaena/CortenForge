//! The rows' inverse inertia with their regularization, `efc_AR = J·M⁻¹·J' +
//! diag(R)`, which MuJoCo's dual solvers read (PGS and noslip), made as
//! MuJoCo's `mj_projectConstraint` makes it
//! (`engine_core_constraint.c:2140-2371`): `B = J·L⁻¹·D^{-1/2}` from the mass
//! matrix's factor `M = L'·D·L`, then `AR = B·B'` with `R` added on its
//! diagonal, dense or at the sparse size ([`Model::is_sparse`]), operation
//! for operation.

use crate::linalg::mju_dot;
use crate::types::{Data, Model};

/// `mjMAXSUPER` (`engine_util_sparse.c:752`): the most columns of `AR` one
/// pass of the sparse product makes together.
const MAX_SUPER: usize = 8;

/// Makes `Data::efc_AR` while a dual solver runs ([`Model::is_dual`]).
///
/// As MuJoCo's position stage does; it is left empty otherwise. At the
/// sparse size `efc_AR` holds each row's values at `efc_AR_rowadr`, its
/// columns in `efc_AR_colind`, ascending.
pub fn mj_project_constraint(model: &Model, data: &mut Data) {
    data.efc_AR.clear();
    data.efc_AR_rownnz.clear();
    data.efc_AR_rowadr.clear();
    data.efc_AR_colind.clear();
    if data.efc_type.is_empty() || !model.is_dual() {
        return;
    }
    // D^{-1/2}, each dof's diagonal the last entry of its row of the factor.
    let sqrt_inv_d: Vec<f64> = (0..model.nv)
        .map(|i| 1.0 / data.qLD_data[model.M_rowadr[i] + model.M_rownnz[i] - 1].sqrt())
        .collect();
    if model.is_sparse() {
        project_sparse(model, data, &sqrt_inv_d);
    } else {
        project_dense(model, data, &sqrt_inv_d);
    }
}

/// `efc_AR` over the implicit spring-damper integrator's `M_impl` (this
/// crate's own; MuJoCo has no such integrator), from its Cholesky factor:
/// dense, `AR[i][k] = J_k · M_impl⁻¹ · J_i'` by `mju_dot`, `R` added on the
/// diagonal; the sparse columns cleared, as `M_impl` need not keep `M`'s
/// pattern.
pub fn project_implicit(data: &mut Data, factor: &nalgebra::DMatrix<f64>) {
    let nefc = data.efc_type.len();
    let nv = data.efc_J.ncols();
    data.efc_AR_rownnz.clear();
    data.efc_AR_rowadr.clear();
    data.efc_AR_colind.clear();
    let rows: Vec<Vec<f64>> = (0..nefc)
        .map(|r| (0..nv).map(|c| data.efc_J[(r, c)]).collect())
        .collect();
    let mut ar = vec![0.0; nefc * nefc];
    for i in 0..nefc {
        let mut w = nalgebra::DVector::from_column_slice(&rows[i]);
        crate::linalg::cholesky_solve_in_place(factor, &mut w);
        for k in 0..nefc {
            ar[i * nefc + k] = mju_dot(&rows[k], w.as_slice());
        }
        ar[i * (nefc + 1)] += data.efc_R[i];
    }
    data.efc_AR = ar;
}

/// The dense branch (`engine_core_constraint.c:2337-2368`): `B` by
/// `mj_solveM2` (`engine_core_smooth.c:2228-2271`), then `AR = B·B'` by
/// `mju_sqrMatTD` (`engine_util_blas.c:886-926`), each entry on or below the
/// diagonal summed from 0 over the dofs in order, a dof skipped where the
/// row's own entry is 0, the upper triangle mirrored, and `R` added on the
/// diagonal.
fn project_dense(model: &Model, data: &mut Data, sqrt_inv_d: &[f64]) {
    let (nefc, nv) = (data.efc_type.len(), model.nv);
    let mut b = vec![0.0; nefc * nv];
    for r in 0..nefc {
        for c in 0..nv {
            b[r * nv + c] = data.efc_J[(r, c)];
        }
    }
    solve_m2(model, data, &mut b, sqrt_inv_d, nefc);
    let mut ar = vec![0.0; nefc * nefc];
    for i in 0..nefc {
        for j in 0..nv {
            let scl = b[i * nv + j];
            if scl != 0.0 {
                for k in 0..=i {
                    ar[i * nefc + k] += b[k * nv + j] * scl;
                }
            }
        }
    }
    for i in 0..nefc {
        for j in i + 1..nefc {
            ar[i * nefc + j] = ar[j * nefc + i];
        }
        ar[i * (nefc + 1)] += data.efc_R[i];
    }
    data.efc_AR = ar;
}

/// `mj_solveM2` (`engine_core_smooth.c:2228-2271`) over `n` rows of `nv`:
/// `x ← L⁻ᵀ·x` from the last dof to the second, a dof skipped when it is
/// simple (`dof_simplenum`) or its entry 0, then `x ← D^{-1/2}·x`.
fn solve_m2(model: &Model, data: &Data, x: &mut [f64], sqrt_inv_d: &[f64], n: usize) {
    let nv = model.nv;
    for i in (1..nv).rev() {
        if model.dof_simplenum.get(i).is_some_and(|&k| k > 0) {
            continue;
        }
        let start = model.M_rowadr[i];
        let end = start + model.M_rownnz[i] - 1;
        for offset in (0..n * nv).step_by(nv) {
            let x_i = x[offset + i];
            if x_i != 0.0 {
                for adr in start..end {
                    x[offset + model.M_colind[adr]] -= data.qLD_data[adr] * x_i;
                }
            }
        }
    }
    for (i, &inv) in sqrt_inv_d.iter().enumerate() {
        for offset in (0..n * nv).step_by(nv) {
            x[offset + i] *= inv;
        }
    }
}

/// The sparse branch (`engine_core_constraint.c:2157-2335`): `B`'s pattern,
/// each row's dof chain completed by `M`'s rows; its values, `J`'s at the
/// chain and 0 at the completed dofs; `B ← B·L⁻ᵀ·D^{-1/2}` in place; `B`'s
/// supernodes and transpose; `AR`'s pattern counted
/// (`mju_sqrMatTDSparseCount`) and its values made (`mju_sqrMatTDSparse`,
/// with no diagonal), `R` added on the diagonal.
// One function, so the steps read as MuJoCo's do; `b`, `bt`, `ar` are its
// `B`, `BT`, `AR`, and their arrays keep MuJoCo's names (`B_rownnz`,
// `BT_rownnz`, …).
#[allow(clippy::too_many_lines, clippy::similar_names)]
fn project_sparse(model: &Model, data: &mut Data, sqrt_inv_d: &[f64]) {
    let nefc = data.efc_type.len();
    let nv = model.nv;

    // B's row lengths: the dofs M couples to each dof of the row's chain,
    // each counted once.
    let mut b_rownnz = vec![0; nefc];
    let mut b_rowadr = vec![0; nefc];
    let mut marker = vec![usize::MAX; nv];
    for r in 0..nefc {
        let mut nnz = 0;
        let start = data.efc_J_rowadr[r];
        for i in (start..start + data.efc_J_rownnz[r]).rev() {
            let j = data.efc_J_colind[i];
            if marker[j] == r {
                continue;
            }
            let adr = model.M_rowadr[j];
            for &c in &model.M_colind[adr..adr + model.M_rownnz[j]] {
                if marker[c] != r {
                    marker[c] = r;
                    nnz += 1;
                }
            }
        }
        b_rownnz[r] = nnz;
        if r + 1 < nefc {
            b_rowadr[r + 1] = b_rowadr[r] + nnz;
        }
    }
    let nb = b_rowadr[nefc - 1] + b_rownnz[nefc - 1];

    // B's columns and values, each row's chain completed in reverse: the
    // larger of J's next column and the parent of the dof last placed.
    let mut b = vec![0.0; nb];
    let mut b_colind = vec![0; nb];
    for r in 0..nefc {
        let end = b_rowadr[r] + b_rownnz[r];
        let adr_j = data.efc_J_rowadr[r];
        let mut remain_j = data.efc_J_rownnz[r];
        let mut nnz_b = 0;
        loop {
            let src = (remain_j > 0).then(|| data.efc_J_colind[adr_j + remain_j - 1]);
            let dst = if nnz_b > 0 {
                model.dof_parent[b_colind[end - nnz_b]]
            } else {
                None
            };
            let (col, value) = match (src, dst) {
                (None, None) => break,
                (Some(s), Some(d)) if s < d => (d, 0.0),
                (None, Some(d)) => (d, 0.0),
                (Some(s), _) => {
                    remain_j -= 1;
                    (s, data.efc_J[(r, s)])
                }
            };
            nnz_b += 1;
            b_colind[end - nnz_b] = col;
            b[end - nnz_b] = value;
        }
        // MuJoCo's own check (`mjERROR`, `engine_core_constraint.c:2251-2253`).
        debug_assert_eq!(nnz_b, b_rownnz[r], "B's row {r}: counted and filled differ");
    }

    // B ← B·L⁻ᵀ, each row's entries from the last, then times D^{-1/2}.
    for r in 0..nefc {
        let (adr_b, nnz_b) = (b_rowadr[r], b_rownnz[r]);
        for i in (adr_b..adr_b + nnz_b).rev() {
            let scl = b[i];
            if scl == 0.0 {
                continue;
            }
            let j = b_colind[i];
            let adr_c = model.M_rowadr[j];
            let nnz_c = model.M_rownnz[j] - 1;
            add_to_scl_sparse_inc(
                &mut b[adr_b..adr_b + nnz_b],
                &b_colind[adr_b..adr_b + nnz_b],
                &data.qLD_data[adr_c..adr_c + nnz_c],
                &model.M_colind[adr_c..adr_c + nnz_c],
                -scl,
            );
        }
        for i in adr_b..adr_b + nnz_b {
            b[i] *= sqrt_inv_d[b_colind[i]];
        }
    }

    let b_rowsuper = super_sparse(&b_rownnz, &b_rowadr, &b_colind);
    let (bt, bt_rownnz, bt_rowadr, bt_colind) =
        transpose_sparse(&b, nv, &b_rownnz, &b_rowadr, &b_colind);

    // AR's rows, counted (`mju_sqrMatTDSparseCount`, upper triangle included),
    // then made (`mju_sqrMatTDSparse`).
    let mut ar_rownnz = vec![0; nefc];
    let mut ar_rowadr = vec![0; nefc];
    let na = sqr_count(
        &mut ar_rownnz,
        &mut ar_rowadr,
        (&bt_rownnz, &bt_rowadr, &bt_colind),
        (&b_rownnz, &b_rowadr, &b_colind, &b_rowsuper),
    );
    let mut ar = vec![0.0; na];
    let mut ar_colind = vec![0; na];
    let diagind = sqr_fill(
        &mut ar,
        &mut ar_rownnz,
        &ar_rowadr,
        &mut ar_colind,
        (&bt, &bt_rownnz, &bt_rowadr, &bt_colind),
        (&b, &b_rownnz, &b_rowadr, &b_colind, &b_rowsuper),
    );
    for (i, &at) in diagind.iter().enumerate() {
        ar[at] += data.efc_R[i];
    }
    data.efc_AR = ar;
    data.efc_AR_rownnz = ar_rownnz;
    data.efc_AR_rowadr = ar_rowadr;
    data.efc_AR_colind = ar_colind;
}

/// `dst += scl·src` where their indices meet, both ascending
/// (`mju_addToSclSparseInc`, `engine_util_sparse.c:342-389`).
fn add_to_scl_sparse_inc(
    dst: &mut [f64],
    ind_dst: &[usize],
    src: &[f64],
    ind_src: &[usize],
    scl: f64,
) {
    let (mut s, mut d) = (0, 0);
    while s < src.len() && d < dst.len() {
        match ind_src[s].cmp(&ind_dst[d]) {
            std::cmp::Ordering::Equal => {
                dst[d] += scl * src[s];
                s += 1;
                d += 1;
            }
            std::cmp::Ordering::Less => s += 1,
            std::cmp::Ordering::Greater => d += 1,
        }
    }
}

/// Each row's supernode count: how many rows after it share its columns
/// (`mju_superSparse`, `engine_util_sparse.c:616-645`).
fn super_sparse(rownnz: &[usize], rowadr: &[usize], colind: &[usize]) -> Vec<usize> {
    let nr = rownnz.len();
    let mut rowsuper = vec![0; nr];
    for r in 0..nr.saturating_sub(1) {
        let same = rownnz[r] == rownnz[r + 1]
            && colind[rowadr[r]..rowadr[r] + rownnz[r]]
                == colind[rowadr[r + 1]..rowadr[r + 1] + rownnz[r + 1]];
        rowsuper[r] = usize::from(same);
    }
    for r in (0..nr.saturating_sub(1)).rev() {
        if rowsuper[r] > 0 {
            rowsuper[r] += rowsuper[r + 1];
        }
    }
    rowsuper
}

/// A CSR matrix's transpose over `nc` columns, each row of the result in
/// ascending order (`mju_transposeSparse`, `engine_util_sparse.c:540-612`,
/// with no supernodes).
fn transpose_sparse(
    val: &[f64],
    nc: usize,
    rownnz: &[usize],
    rowadr: &[usize],
    colind: &[usize],
) -> (Vec<f64>, Vec<usize>, Vec<usize>, Vec<usize>) {
    let mut t_rownnz = vec![0; nc];
    for (r, &nnz) in rownnz.iter().enumerate() {
        for &c in &colind[rowadr[r]..rowadr[r] + nnz] {
            t_rownnz[c] += 1;
        }
    }
    let mut t_rowadr = vec![0; nc];
    for i in 1..nc {
        t_rowadr[i] = t_rowadr[i - 1] + t_rownnz[i - 1];
    }
    let n = colind.len();
    let (mut t_val, mut t_colind) = (vec![0.0; n], vec![0; n]);
    let mut next = t_rowadr.clone();
    for (r, &nnz) in rownnz.iter().enumerate() {
        for i in rowadr[r]..rowadr[r] + nnz {
            let c = colind[i];
            t_colind[next[c]] = r;
            t_val[next[c]] = val[i];
            next[c] += 1;
        }
    }
    (t_val, t_rownnz, t_rowadr, t_colind)
}

/// `mju_sqrMatTDSparseCount` (`engine_util_sparse.c:649-740`) for `B·B'`,
/// upper triangle included: each row's count and start, returning the
/// total. A row whose supernode predecessor shares its columns takes that
/// row's count and adds its diagonal.
// `B`'s and `BT`'s arrays keep MuJoCo's names (`B_rownnz`, `BT_rownnz`, …).
#[allow(clippy::similar_names)]
fn sqr_count(
    res_rownnz: &mut [usize],
    res_rowadr: &mut [usize],
    (bt_rownnz, bt_rowadr, bt_colind): (&[usize], &[usize], &[usize]),
    (b_rownnz, b_rowadr, b_colind, b_rowsuper): (&[usize], &[usize], &[usize], &[usize]),
) -> usize {
    let nr = res_rownnz.len();
    let mut chain = vec![0; 2 * nr];
    let mut nchain = 0;
    // Where the last row's chain starts in `chain` (MuJoCo's `res_colind`).
    let mut res_colind = 0;
    for r in 0..nr {
        if r > 0 && b_rowsuper[r - 1] > 0 {
            res_rownnz[r] = res_rownnz[r - 1];
            for j in 0..nchain {
                res_rownnz[chain[res_colind + j]] += 1;
            }
            if b_rownnz[r] > 0 {
                chain[res_colind + nchain] = r;
                nchain += 1;
                res_rownnz[r] += 1;
            }
        } else {
            let (mut inew, mut iold) = (0, nr);
            nchain = 0;
            for i in 0..b_rownnz[r] {
                let c = b_colind[b_rowadr[r] + i];
                std::mem::swap(&mut inew, &mut iold);
                let mut nnew = 0;
                let mut adr = 0;
                for &col in &bt_colind[bt_rowadr[c]..bt_rowadr[c] + bt_rownnz[c]] {
                    while adr < nchain && chain[iold + adr] < col && chain[iold + adr] <= r {
                        chain[inew + nnew] = chain[iold + adr];
                        nnew += 1;
                        adr += 1;
                    }
                    if col > r {
                        break;
                    }
                    if adr < nchain && chain[iold + adr] == col {
                        adr += 1;
                    }
                    chain[inew + nnew] = col;
                    nnew += 1;
                }
                while adr < nchain && chain[iold + adr] <= r {
                    chain[inew + nnew] = chain[iold + adr];
                    nnew += 1;
                    adr += 1;
                }
                nchain = nnew;
            }
            res_rownnz[r] = nchain;
            res_colind = inew;
            // The upper triangle, the diagonal not counted twice.
            let end = if nchain > 0 && chain[res_colind + nchain - 1] == r {
                nchain - 1
            } else {
                nchain
            };
            for j in 0..end {
                res_rownnz[chain[res_colind + j]] += 1;
            }
        }
    }
    for r in 1..nr {
        res_rowadr[r] = res_rowadr[r - 1] + res_rownnz[r - 1];
    }
    res_rowadr[nr - 1] + res_rownnz[nr - 1]
}

/// `mju_sqrMatTDSparse` (`engine_util_sparse.c:755-948`) for `AR = B·B'`
/// with no diagonal weight: by column `c` of `AR`, with up to [`MAX_SUPER`]
/// supernode columns at once, its diagonal `mju_dot` of `B`'s row with
/// itself, and each entry below it `Σ BT[r, j]·B[c, r]` over `B`'s row `c`
/// in order, the first set and the rest added; then the upper triangle
/// mirrored. Returns each row's diagonal entry's place.
// MuJoCo's names; the two matrices' arrays travel as tuples.
#[allow(clippy::many_single_char_names, clippy::similar_names)]
fn sqr_fill(
    res: &mut [f64],
    res_rownnz: &mut [usize],
    res_rowadr: &[usize],
    res_colind: &mut [usize],
    (bt, bt_rownnz, bt_rowadr, bt_colind): (&[f64], &[usize], &[usize], &[usize]),
    (b, b_rownnz, b_rowadr, b_colind, b_rowsuper): (&[f64], &[usize], &[usize], &[usize], &[usize]),
) -> Vec<usize> {
    let nc = res_rownnz.len();
    let nr = bt_rownnz.len();
    let mut marker = vec![false; nc];
    let mut buffer = vec![0.0; nc * MAX_SUPER];
    let mut buffer_idx = Vec::with_capacity(nc);
    let mut rowstart = vec![0; nr];
    res_rownnz.fill(0);
    let mut c = 0;
    while c < nc {
        buffer_idx.clear();
        let (nnz, adr) = (b_rownnz[c], b_rowadr[c]);
        let ind = &b_colind[adr..adr + nnz];
        let ns = 1 + b_rowsuper[c].min(MAX_SUPER - 1);
        let val: Vec<&[f64]> = (0..ns)
            .map(|s| &b[b_rowadr[c + s]..b_rowadr[c + s] + nnz])
            .collect();
        let diag: Vec<f64> = val.iter().map(|v| mju_dot(v, v)).collect();
        for (i, &r) in ind.iter().enumerate() {
            let (adr_t, nnz_t) = (bt_rowadr[r], bt_rownnz[r]);
            let mut v = [0.0; MAX_SUPER];
            for s in 0..ns {
                v[s] = val[s][i];
            }
            for k in rowstart[r]..nnz_t {
                let j = bt_colind[adr_t + k];
                if j <= c {
                    rowstart[r] += 1;
                    continue;
                }
                let vk = bt[adr_t + k];
                if marker[j] {
                    for s in 0..ns {
                        buffer[s * nc + j] += vk * v[s];
                    }
                } else {
                    marker[j] = true;
                    buffer_idx.push(j);
                    for s in 0..ns {
                        buffer[s * nc + j] = vk * v[s];
                    }
                }
            }
        }
        for &j in &buffer_idx {
            marker[j] = false;
            let adr_j = res_rowadr[j] + res_rownnz[j];
            let nm = ns.min(j - c);
            res_rownnz[j] += nm;
            for s in 0..nm {
                res[adr_j + s] = buffer[s * nc + j];
                res_colind[adr_j + s] = c + s;
            }
        }
        for s in 0..ns {
            let at = res_rowadr[c + s] + res_rownnz[c + s];
            res_rownnz[c + s] += 1;
            res_colind[at] = c + s;
            res[at] = diag[s];
        }
        c += ns;
    }
    let diagind: Vec<usize> = (0..nc).map(|i| res_rowadr[i] + res_rownnz[i] - 1).collect();
    for i in 0..nc {
        let start = res_rowadr[i];
        for j in start..start + res_rownnz[i] - 1 {
            let row = res_colind[j];
            let at = res_rowadr[row] + res_rownnz[row];
            res_rownnz[row] += 1;
            res[at] = res[j];
            res_colind[at] = i;
        }
    }
    diagind
}
