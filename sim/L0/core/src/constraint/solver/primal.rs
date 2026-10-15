//! The primal solver Newton and CG share: MuJoCo 3.5.0's `mj_solPrimal`
//! (`engine_solver.c:1811-1968`), with the warmstart it starts from
//! (`warmstart`, `engine_forward.c:611-687`), its constraint update
//! (`mj_constraintUpdate_impl`, `engine_core_constraint.c:2394-2587`), its line
//! search (`PrimalPrepare`, `PrimalEval`, `updateBracket`, `PrimalSearch`,
//! `engine_solver.c:1075-1511`) and Newton's Hessian (`MakeHessian`,
//! `FactorizeHessian`, `HessianCone`, `HessianIncremental`, `:1516-1807`),
//! operation for operation on MuJoCo's dense path. With islands it solves
//! each island apart, over its own dofs and rows in island order and
//! scaled by its own inertia, as `mj_fwdConstraint` does
//! (`engine_forward.c:762-797`); otherwise every row at once (MuJoCo's
//! monolithic case, `island` −1).
//!
//! At MuJoCo's sparse size (`nv ≥ 60`, `mj_isSparse`,
//! `engine_core_util.c:32-39`) Newton factors its Hessian with
//! [`SparseHessian`], made anew at every iteration where MuJoCo updates its
//! sparse factor: the same matrix, rounded otherwise. There MuJoCo also takes
//! `J·v` and `J'·f` as sparse products, which sim-core takes dense,
//! so CG as well rounds otherwise.

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::MJ_MINVAL;
use crate::constraint::solver::hessian::{SparseHessian, SparseInputs};
use crate::dynamics::factor::mj_solve_ld;
use crate::linalg::{
    cholesky_solve_in_place, mju_chol_factor, mju_chol_solve, mju_chol_update, mju_dot, mju_norm,
};
use crate::types::flags::disabled;
use crate::types::{
    ConstraintState, ConstraintType, DISABLE_ISLAND, DISABLE_WARMSTART, Data, Integrator, Model,
    SolverStat, StepError,
};

/// The islands MuJoCo keeps iteration counts and statistics for
/// (`mjNISLAND`; `saveStats`, `engine_solver.c:38-64`, and `:1946-1951`): a
/// later island is solved but not counted.
const MJ_NISLAND: usize = 20;

/// The mass matrix the primal solver minimises over.
pub enum PrimalMass<'a> {
    /// `qM` on the mass matrix's pattern, solved with its `L'·D·L` factor:
    /// MuJoCo's.
    Tree,
    /// The implicit spring-damper integrator's `M + h·D + h²·K` (an extension
    /// of this crate's) and its Cholesky factor.
    Implicit {
        m: &'a DMatrix<f64>,
        factor: &'a DMatrix<f64>,
    },
}

/// One solve's mass matrix, on its dofs (`mjPrimalContext`'s inertia).
enum Mass<'a> {
    /// `M` on its tree pattern and its `L'·D·L` factor.
    Tree(TreeMass),
    /// The implicit spring-damper integrator's `M_impl` and its Cholesky
    /// factor, over every dof.
    Implicit {
        m: &'a DMatrix<f64>,
        factor: &'a DMatrix<f64>,
    },
}

/// `M` and its factor `qLD` on a solve's dofs, in its numbering: for an
/// island, the block MuJoCo copies into `iM` and `iLD`
/// (`mju_blockDiagSparse`, `engine_island.c:503-509`); for every dof, `qM`
/// and `qLD` as they are.
struct TreeMass {
    rownnz: Vec<usize>,
    rowadr: Vec<usize>,
    colind: Vec<usize>,
    m: Vec<f64>,
    ld: Vec<f64>,
    diag_inv: Vec<f64>,
}

impl TreeMass {
    fn new(model: &Model, data: &Data, sub: &Subproblem) -> Self {
        let mut t = Self {
            rownnz: Vec::with_capacity(sub.dofs.len()),
            rowadr: Vec::with_capacity(sub.dofs.len()),
            colind: Vec::new(),
            m: Vec::new(),
            ld: Vec::new(),
            diag_inv: Vec::with_capacity(sub.dofs.len()),
        };
        for &dof in &sub.dofs {
            let adr = model.M_rowadr[dof];
            t.rowadr.push(t.colind.len());
            t.rownnz.push(model.M_rownnz[dof]);
            for k in adr..adr + model.M_rownnz[dof] {
                // An ancestor is in the dof's tree, so in the solve.
                let col = model.M_colind[k];
                t.colind.push(sub.local[col]);
                t.m.push(data.qM[(dof, col)]);
                t.ld.push(data.qLD_data[k]);
            }
            t.diag_inv.push(data.qLD_diag_inv[dof]);
        }
        t
    }

    /// The sum of `M`'s diagonal, an island's inertia
    /// (`engine_solver.c:1864-1870`).
    fn trace(&self) -> f64 {
        let mut sum = 0.0;
        for (adr, nnz) in self.rowadr.iter().zip(&self.rownnz) {
            sum += self.m[adr + nnz - 1];
        }
        sum
    }
}

impl Mass<'_> {
    /// `res = M·vec`; on the tree as `mju_mulSymVecSparse` multiplies
    /// (`engine_util_sparse.c:240-265`): each row's diagonal term, then its
    /// entries from the diagonal down, each added to its row and its column.
    fn mul(&self, vec: &[f64], res: &mut [f64]) {
        match self {
            Self::Tree(t) => {
                for i in 0..res.len() {
                    let adr = t.rowadr[i];
                    let diag = t.rownnz[i] - 1;
                    res[i] = t.m[adr + diag] * vec[i];
                    for k in (0..diag).rev() {
                        let j = t.colind[adr + k];
                        let val = t.m[adr + k];
                        res[i] += val * vec[j];
                        res[j] += val * vec[i];
                    }
                }
            }
            Self::Implicit { m, .. } => {
                for (r, out) in res.iter_mut().enumerate() {
                    *out = mju_dot(m.column(r).as_slice(), vec);
                }
            }
        }
    }

    /// Add `M` to the lower triangle and the diagonal of the dense row-major
    /// `nv × nv` matrix `h`; on the tree, as `mju_addToSymSparse` adds
    /// (`engine_util_sparse.c:217-235`, `flg_upper` 0).
    fn add_to_lower(&self, h: &mut [f64], nv: usize) {
        match self {
            Self::Tree(t) => {
                for i in 0..nv {
                    for adr in t.rowadr[i]..t.rowadr[i] + t.rownnz[i] {
                        h[i * nv + t.colind[adr]] += t.m[adr];
                    }
                }
            }
            Self::Implicit { m, .. } => {
                for i in 0..nv {
                    for j in 0..=i {
                        h[i * nv + j] += m[(i, j)];
                    }
                }
            }
        }
    }

    /// `x ← M⁻¹·x`; on the tree, as `mj_solveLD` solves.
    fn solve(&self, x: &mut [f64]) {
        match self {
            Self::Tree(t) => {
                mj_solve_ld(x, &t.ld, &t.diag_inv, &t.rownnz, &t.rowadr, &t.colind, None);
            }
            Self::Implicit { factor, .. } => {
                let mut v = DVector::from_column_slice(x);
                cholesky_solve_in_place(factor, &mut v);
                x.copy_from_slice(v.as_slice());
            }
        }
    }
}

/// The dofs and rows one solve takes, as `PrimalPointers` points the solver
/// at them (`engine_solver.c:855-951`): every dof and row (MuJoCo's island
/// −1), one island's in island order, or the rows in no island.
struct Subproblem {
    dofs: Vec<usize>,
    rows: Vec<usize>,
    /// Each dof's place in `dofs`, `usize::MAX` for a dof not in it.
    local: Vec<usize>,
}

impl Subproblem {
    fn new(nv: usize, dofs: Vec<usize>, rows: Vec<usize>) -> Self {
        let mut local = vec![usize::MAX; nv];
        for (l, &dof) in dofs.iter().enumerate() {
            local[dof] = l;
        }
        Self { dofs, rows, local }
    }

    fn whole(model: &Model, data: &Data) -> Self {
        Self::new(
            model.nv,
            (0..model.nv).collect(),
            (0..data.efc_type.len()).collect(),
        )
    }

    fn island(model: &Model, data: &Data, island: usize) -> Self {
        let dofs =
            data.island_idofadr[island]..data.island_idofadr[island] + data.island_nv[island];
        let rows =
            data.island_iefcadr[island]..data.island_iefcadr[island] + data.island_nefc[island];
        Self::new(
            model.nv,
            data.map_idof2dof[dofs].to_vec(),
            data.map_iefc2efc[rows].to_vec(),
        )
    }

    /// The rows in no island, without dofs. Every row MuJoCo makes is in an
    /// island; sim-core's partition reads a row's nonzero Jacobian entries,
    /// so a row whose entries are all zero (a coef-0 fixed tendon's at the
    /// sparse size) is in none.
    fn outside(model: &Model, data: &Data) -> Self {
        let rows = (0..data.efc_type.len())
            .filter(|&r| data.efc_island.get(r).is_none_or(|&i| i < 0))
            .collect();
        Self::new(model.nv, Vec::new(), rows)
    }
}

/// The constraint rows' data the update and the line search read
/// (`mjPrimalContext`'s efc arrays).
struct Rows {
    ne: usize,
    nf: usize,
    d: Vec<f64>,
    r: Vec<f64>,
    floss: Vec<f64>,
    ty: Vec<ConstraintType>,
    dim: Vec<usize>,
    mu: Vec<[f64; 5]>,
    id: Vec<usize>,
}

/// A contact's cone Hessian, row-major `dim × dim` (`mjContact::H`).
type ConeHessian = [f64; 36];

/// Update each row's state, force and cost at `jar = J·qacc − aref`, as
/// MuJoCo's `mj_constraintUpdate_impl` does
/// (`engine_core_constraint.c:2394-2587`), and return the cost. With `cone_h`,
/// each contact in the cone state gets its cone Hessian (MuJoCo's
/// `flg_coneHessian`), at its first row.
// One function so the per-kind branches read as MuJoCo's do; single-letter
// names (`s`, `n`, `t`, `u`) are MuJoCo's own.
#[allow(clippy::many_single_char_names, clippy::too_many_lines)]
fn constraint_update(
    rows: &Rows,
    jar: &[f64],
    state: &mut [ConstraintState],
    force: &mut [f64],
    mut cone_h: Option<&mut [ConeHessian]>,
) -> f64 {
    let nefc = jar.len();
    let mut s = 0.0;
    if nefc == 0 {
        return 0.0;
    }
    for i in 0..nefc {
        force[i] = -rows.d[i] * jar[i];
    }
    let mut i = 0;
    while i < nefc {
        // Equality.
        if i < rows.ne {
            s += 0.5 * rows.d[i] * jar[i] * jar[i];
            state[i] = ConstraintState::Quadratic;
            i += 1;
            continue;
        }
        // Friction loss.
        if i < rows.ne + rows.nf {
            let (r, f) = (rows.r[i], rows.floss[i]);
            if jar[i] <= -r * f {
                s += -0.5 * r * f * f - f * jar[i];
                force[i] = f;
                state[i] = ConstraintState::LinearNeg;
            } else if jar[i] >= r * f {
                s += -0.5 * r * f * f + f * jar[i];
                force[i] = -f;
                state[i] = ConstraintState::LinearPos;
            } else {
                s += 0.5 * rows.d[i] * jar[i] * jar[i];
                state[i] = ConstraintState::Quadratic;
            }
            i += 1;
            continue;
        }
        // Limit or contact.
        if rows.ty[i] != ConstraintType::ContactElliptic {
            if jar[i] >= 0.0 {
                force[i] = 0.0;
                state[i] = ConstraintState::Satisfied;
            } else {
                s += 0.5 * rows.d[i] * jar[i] * jar[i];
                state[i] = ConstraintState::Quadratic;
            }
            i += 1;
            continue;
        }
        // Contact with an elliptic cone, mapped to the regular dual cone.
        // μ here is friction[0]; MuJoCo's is friction[0]·√(R[i+1]/R[i])
        // (`engine_core_constraint.c:1540`), the same at impratio 1, where it
        // sets R[i+1] = R[i]. The elliptic rows' R, and this μ, are book P38's.
        let dim = rows.dim[i];
        let mu = rows.mu[i][0];
        let friction = &rows.mu[i];
        let mut u = [0.0; 6];
        u[0] = jar[i] * mu;
        for j in 1..dim {
            u[j] = jar[i + j] * friction[j - 1];
        }
        let n = u[0];
        let t = mju_norm(&u[1..dim]);
        if n >= mu * t || (t <= 0.0 && n >= 0.0) {
            force[i..i + dim].fill(0.0);
            state[i] = ConstraintState::Satisfied;
        } else if mu * n + t <= 0.0 || (t <= 0.0 && n < 0.0) {
            for j in 0..dim {
                s += 0.5 * rows.d[i + j] * jar[i + j] * jar[i + j];
            }
            state[i] = ConstraintState::Quadratic;
        } else {
            let dm = rows.d[i] / (mu * mu * (1.0 + mu * mu));
            let nmt = n - mu * t;
            s += 0.5 * dm * nmt * nmt;
            force[i] = -dm * nmt * mu;
            for j in 1..dim {
                force[i + j] = -force[i] / t * u[j] * friction[j - 1];
            }
            state[i] = ConstraintState::Cone;
            if let Some(cones) = cone_h.as_deref_mut() {
                let h = &mut cones[i];
                // First row (1, −μ/T·U), the upper block μN/T³·U·U', then
                // (μ² − μN/T) on the diagonal; both sides times
                // diag(μ, friction) and Dm; the lower triangle mirrored.
                let mut scl = -mu / t;
                h[0] = 1.0;
                for j in 1..dim {
                    h[j] = scl * u[j];
                }
                scl = mu * n / (t * t * t);
                for k in 1..dim {
                    for j in k..dim {
                        h[k * dim + j] = scl * u[j] * u[k];
                    }
                }
                scl = mu * mu - mu * n / t;
                for j in 1..dim {
                    h[j * (dim + 1)] += scl;
                }
                for k in 0..dim {
                    scl = dm * if k == 0 { mu } else { friction[k - 1] };
                    for j in k..dim {
                        h[k * dim + j] *= scl * if j == 0 { mu } else { friction[j - 1] };
                    }
                }
                for k in 0..dim {
                    for j in k + 1..dim {
                        h[j * dim + k] = h[k * dim + j];
                    }
                }
            }
        }
        for j in 1..dim {
            state[i + j] = state[i];
        }
        i += dim;
    }
    s
}

/// `res = mat·vec` for a dense row-major `nr × nc` matrix (`mju_mulMatVec`,
/// `engine_util_blas.c:785-789`).
fn mul_mat_vec(mat: &[f64], vec: &[f64], res: &mut [f64], nc: usize) {
    for (r, out) in res.iter_mut().enumerate() {
        *out = mju_dot(&mat[r * nc..(r + 1) * nc], vec);
    }
}

/// `res = mat'·vec` for a dense row-major `nr × nc` matrix (`mju_mulMatTVec`,
/// `engine_util_blas.c:793-802`).
fn mul_mat_t_vec(mat: &[f64], vec: &[f64], res: &mut [f64], nc: usize) {
    res.fill(0.0);
    for (r, &v) in vec.iter().enumerate() {
        if v != 0.0 {
            for (out, m) in res.iter_mut().zip(&mat[r * nc..(r + 1) * nc]) {
                *out += m * v;
            }
        }
    }
}

/// A line-search point (`mjPrimalPnt`).
#[derive(Debug, Clone, Copy, Default)]
struct Pnt {
    /// The step size along `search`.
    alpha: f64,
    /// The cost at `alpha`.
    cost: f64,
    /// The cost's first and second derivatives in `alpha`.
    deriv: [f64; 2],
}

/// The solver's state (`mjPrimalContext`) over one subproblem: the rows, the
/// dof arrays and the work arrays, MuJoCo's dense `efc_J` among them
/// (row-major), each in the subproblem's numbering.
struct Primal<'a> {
    newton: bool,
    nv: usize,
    nefc: usize,
    sub: Subproblem,
    mass: Mass<'a>,
    /// Each dof's parent dof, for the sparse Hessian.
    parent: Vec<Option<usize>>,
    rows: Rows,
    j: Vec<f64>,
    aref: Vec<f64>,
    state: Vec<ConstraintState>,
    force: Vec<f64>,
    cone_h: Vec<ConeHessian>,
    qfrc_smooth: Vec<f64>,
    qacc_smooth: Vec<f64>,
    qfrc_constraint: Vec<f64>,
    qacc: Vec<f64>,
    jaref: Vec<f64>,
    jv: Vec<f64>,
    ma: Vec<f64>,
    mv: Vec<f64>,
    grad: Vec<f64>,
    mgrad: Vec<f64>,
    search: Vec<f64>,
    /// Each row's cost along `search` while quadratic, `quad[0] + α·quad[1] +
    /// α²·quad[2]`, 3 per row: `[jaref·D·jaref/2, jv·D·jaref, jv·D·jv/2]`; an
    /// elliptic contact's first row sums its rows', and its cone terms take
    /// the slots of its second and third rows.
    quad: Vec<f64>,
    /// Newton: each row's `D` while quadratic, else 0.
    hd: Vec<f64>,
    /// Newton, dense: the Hessian's factor, and with cones the factor of the
    /// Hessian with their contributions.
    l: Vec<f64>,
    lcone: Vec<f64>,
    /// Newton, sparse: the Hessian with its cones, factored.
    sparse: Option<SparseHessian>,
    cost: f64,
    /// The Gauss term along `search`: `quad_gauss[0] + α·quad_gauss[1] +
    /// α²·quad_gauss[2]`.
    quad_gauss: [f64; 3],
    scale: f64,
    nactive: usize,
    ncone: usize,
    nupdate: usize,
    ls_iter: usize,
    ls_slope: f64,
}

impl<'a> Primal<'a> {
    /// The solver over `sub`, its inputs copied from `data` (`qacc` its
    /// start) as MuJoCo gathers an island's (`engine_forward.c:772-778`,
    /// `engine_island.c:511-600`).
    fn new(
        model: &Model,
        data: &Data,
        sub: Subproblem,
        mass: Mass<'a>,
        qfrc_smooth: &[f64],
        newton: bool,
    ) -> Self {
        let (nv, nefc) = (sub.dofs.len(), sub.rows.len());
        let mut j = vec![0.0; nefc * nv];
        for (lr, &r) in sub.rows.iter().enumerate() {
            for (lc, &c) in sub.dofs.iter().enumerate() {
                j[lr * nv + lc] = data.efc_J[(r, c)];
            }
        }
        let parent = sub
            .dofs
            .iter()
            .map(|&dof| model.dof_parent[dof].map(|p| sub.local[p]))
            .collect();
        let gather_rows = |a: &[f64]| sub.rows.iter().map(|&r| a[r]).collect::<Vec<_>>();
        let gather_dofs = |a: &[f64]| sub.dofs.iter().map(|&d| a[d]).collect::<Vec<_>>();
        let count =
            |ty: ConstraintType| sub.rows.iter().filter(|&&r| data.efc_type[r] == ty).count();
        let rows = Rows {
            ne: count(ConstraintType::Equality),
            nf: count(ConstraintType::FrictionLoss),
            d: gather_rows(&data.efc_D),
            r: gather_rows(&data.efc_R),
            floss: gather_rows(&data.efc_floss),
            ty: sub.rows.iter().map(|&r| data.efc_type[r]).collect(),
            dim: sub.rows.iter().map(|&r| data.efc_dim[r]).collect(),
            mu: sub.rows.iter().map(|&r| data.efc_mu[r]).collect(),
            id: sub.rows.iter().map(|&r| data.efc_id[r]).collect(),
        };
        let dense_newton = newton && !model.is_sparse();
        Self {
            newton,
            nv,
            nefc,
            parent,
            rows,
            j,
            aref: gather_rows(data.efc_aref.as_slice()),
            state: vec![ConstraintState::Satisfied; nefc],
            force: vec![0.0; nefc],
            cone_h: vec![[0.0; 36]; nefc],
            qfrc_smooth: gather_dofs(qfrc_smooth),
            qacc_smooth: gather_dofs(data.qacc_smooth.as_slice()),
            qfrc_constraint: vec![0.0; nv],
            qacc: gather_dofs(data.qacc.as_slice()),
            jaref: vec![0.0; nefc],
            jv: vec![0.0; nefc],
            ma: vec![0.0; nv],
            mv: vec![0.0; nv],
            grad: vec![0.0; nv],
            mgrad: vec![0.0; nv],
            search: vec![0.0; nv],
            quad: vec![0.0; nefc * 3],
            hd: vec![0.0; if newton { nefc } else { 0 }],
            l: vec![0.0; if dense_newton { nv * nv } else { 0 }],
            lcone: vec![0.0; if dense_newton { nv * nv } else { 0 }],
            sparse: None,
            cost: 0.0,
            quad_gauss: [0.0; 3],
            scale: 0.0,
            nactive: 0,
            ncone: 0,
            nupdate: 0,
            ls_iter: 0,
            ls_slope: 1.0,
            sub,
            mass,
        }
    }

    /// Choose the starting acceleration as MuJoCo's `warmstart` does
    /// (`engine_forward.c:611-687`), for the primal solvers: the warmstart
    /// unless the unconstrained acceleration costs less (a tie keeps the
    /// warmstart), then `qacc_smooth` in every dof outside the islands;
    /// `qacc_smooth` alone with warmstart disabled. Over every dof and row.
    fn warmstart(&mut self, model: &Model, data: &Data, efc_b: &[f64]) {
        let nv = self.nv;
        if disabled(model, DISABLE_WARMSTART) {
            self.qacc.copy_from_slice(&self.qacc_smooth);
            return;
        }
        let warm = data.qacc_warmstart.as_slice();
        self.qacc.copy_from_slice(warm);
        let mut jar = vec![0.0; self.nefc];
        mul_mat_vec(&self.j, warm, &mut jar, nv);
        for (x, a) in jar.iter_mut().zip(&self.aref) {
            *x -= a;
        }
        let mut cost_warmstart =
            constraint_update(&self.rows, &jar, &mut self.state, &mut self.force, None);
        let mut ma = vec![0.0; nv];
        self.mass.mul(warm, &mut ma);
        for i in 0..nv {
            cost_warmstart += 0.5 * (ma[i] - self.qfrc_smooth[i]) * (warm[i] - self.qacc_smooth[i]);
        }
        let cost_smooth =
            constraint_update(&self.rows, efc_b, &mut self.state, &mut self.force, None);
        if cost_warmstart > cost_smooth {
            self.qacc.copy_from_slice(&self.qacc_smooth);
        }
        if data.nisland > 0 {
            let last = data.nisland - 1;
            let nidof = data.island_idofadr[last] + data.island_nv[last];
            for &dof in &data.map_idof2dof[nidof..nv] {
                self.qacc[dof] = self.qacc_smooth[dof];
            }
        }
    }

    /// Update the rows' states and forces, `qfrc_constraint` and the cost
    /// (`PrimalUpdateConstraint`, `engine_solver.c:1010-1043`).
    fn update_constraint(&mut self, cone_hessian: bool) {
        let cones = cone_hessian.then_some(&mut self.cone_h[..]);
        self.cost = constraint_update(
            &self.rows,
            &self.jaref,
            &mut self.state,
            &mut self.force,
            cones,
        );
        mul_mat_t_vec(&self.j, &self.force, &mut self.qfrc_constraint, self.nv);
        self.nactive = 0;
        self.ncone = 0;
        for s in &self.state {
            self.nactive += usize::from(*s != ConstraintState::Satisfied);
            self.ncone += usize::from(*s == ConstraintState::Cone);
        }
        let mut gauss = 0.0;
        for i in 0..self.nv {
            gauss +=
                0.5 * (self.ma[i] - self.qfrc_smooth[i]) * (self.qacc[i] - self.qacc_smooth[i]);
        }
        self.quad_gauss[0] = gauss;
        self.cost += gauss;
    }

    /// `grad = M·qacc − qfrc_smooth − qfrc_constraint` and its preconditioned
    /// form: `H⁻¹·grad` for Newton, `M⁻¹·grad` for CG
    /// (`PrimalUpdateGradient`, `engine_solver.c:1047-1071`).
    fn update_gradient(&mut self) {
        for i in 0..self.nv {
            self.grad[i] = self.ma[i] - self.qfrc_smooth[i] - self.qfrc_constraint[i];
        }
        self.mgrad.copy_from_slice(&self.grad);
        if !self.newton {
            self.mass.solve(&mut self.mgrad);
        } else if let Some(sparse) = &self.sparse {
            sparse.solve(&mut self.mgrad);
        } else {
            let l = if self.ncone > 0 { &self.lcone } else { &self.l };
            mju_chol_solve(l, &mut self.mgrad, self.nv);
        }
    }

    /// The Gauss term's and each row's quadratic in the step along `search`,
    /// and each elliptic contact's cone terms (`PrimalPrepare`,
    /// `engine_solver.c:1075-1147`). A contact's cone terms go in the slots
    /// of its second and third rows.
    // Single-letter names (`d`, `q`, `u`, `v`) are MuJoCo's own.
    #[allow(clippy::many_single_char_names)]
    fn prepare(&mut self) {
        self.quad_gauss[1] =
            mju_dot(&self.search, &self.ma) - mju_dot(&self.qfrc_smooth, &self.search);
        self.quad_gauss[2] = 0.5 * mju_dot(&self.search, &self.mv);
        let (jv, jaref, d) = (&self.jv, &self.jaref, &self.rows.d);
        let mut i = 0;
        while i < self.nefc {
            let q = 3 * i;
            let dj0 = d[i] * jaref[i];
            self.quad[q] = jaref[i] * dj0;
            self.quad[q + 1] = jv[i] * dj0;
            self.quad[q + 2] = jv[i] * d[i] * jv[i];
            let next = if self.rows.ty[i] == ConstraintType::ContactElliptic {
                let dim = self.rows.dim[i];
                // μ as in `constraint_update` (book P38).
                let mu = self.rows.mu[i][0];
                let friction = &self.rows.mu[i];
                for j in 1..dim {
                    let djj = d[i + j] * jaref[i + j];
                    self.quad[q] += jaref[i + j] * djj;
                    self.quad[q + 1] += jv[i + j] * djj;
                    self.quad[q + 2] += jv[i + j] * d[i + j] * jv[i + j];
                }
                let (mut u, mut v) = ([0.0; 6], [0.0; 6]);
                u[0] = jaref[i] * mu;
                v[0] = jv[i] * mu;
                for j in 1..dim {
                    u[j] = jaref[i + j] * friction[j - 1];
                    v[j] = jv[i + j] * friction[j - 1];
                }
                let (mut uu, mut uv, mut vv) = (0.0, 0.0, 0.0);
                for j in 1..dim {
                    uu += u[j] * u[j];
                    uv += u[j] * v[j];
                    vv += v[j] * v[j];
                }
                self.quad[q + 3] = u[0];
                self.quad[q + 4] = v[0];
                self.quad[q + 5] = uu;
                self.quad[q + 6] = uv;
                self.quad[q + 7] = vv;
                self.quad[q + 8] = d[i] / ((mu * mu) * (1.0 + (mu * mu)));
                i + dim
            } else {
                i + 1
            };
            self.quad[q] *= 0.5;
            self.quad[q + 2] *= 0.5;
            i = next;
        }
    }

    /// The cost along `search` at `alpha` and its first two derivatives
    /// (`PrimalEval`, `engine_solver.c:1160-1294`); counts the evaluation.
    // Single-letter names (`f`, `x`, `n`, `t`) are MuJoCo's own.
    #[allow(clippy::many_single_char_names)]
    fn eval(&mut self, alpha: f64) -> Pnt {
        let rows = &self.rows;
        let mut cost = 0.0;
        let mut deriv = [0.0, 0.0];
        let mut total = self.quad_gauss;
        let add = |total: &mut [f64; 3], q: &[f64]| {
            total[0] += q[0];
            total[1] += q[1];
            total[2] += q[2];
        };
        let mut i = 0;
        while i < self.nefc {
            let quad = &self.quad[3 * i..];
            if i < rows.ne {
                add(&mut total, quad);
                i += 1;
                continue;
            }
            if i < rows.ne + rows.nf {
                let (start, dir) = (self.jaref[i], self.jv[i]);
                let x = start + alpha * dir;
                let f = rows.floss[i];
                let rf = rows.r[i] * f;
                if -rf < x && x < rf {
                    add(&mut total, quad);
                } else if x <= -rf {
                    add(&mut total, &[f * (-0.5 * rf - start), -f * dir, 0.0]);
                } else {
                    add(&mut total, &[f * (-0.5 * rf + start), f * dir, 0.0]);
                }
                i += 1;
                continue;
            }
            if rows.ty[i] == ConstraintType::ContactElliptic {
                // μ as in `constraint_update` (book P38).
                let mu = rows.mu[i][0];
                let (u0, v0, uu, uv, vv, dm) =
                    (quad[3], quad[4], quad[5], quad[6], quad[7], quad[8]);
                let n = u0 + alpha * v0;
                let tsqr = uu + alpha * (2.0 * uv + alpha * vv);
                if tsqr <= 0.0 {
                    if n < 0.0 {
                        add(&mut total, quad);
                    }
                } else {
                    let t = tsqr.sqrt();
                    if n >= mu * t {
                        // Top zone: no cost.
                    } else if mu * n + t <= 0.0 {
                        add(&mut total, quad);
                    } else {
                        let n1 = v0;
                        let t1 = (uv + alpha * vv) / t;
                        let t2 = vv / t - (uv + alpha * vv) * t1 / (t * t);
                        cost += 0.5 * dm * (n - mu * t) * (n - mu * t);
                        deriv[0] += dm * (n - mu * t) * (n1 - mu * t1);
                        deriv[1] +=
                            dm * ((n1 - mu * t1) * (n1 - mu * t1) + (n - mu * t) * (-mu * t2));
                    }
                }
                i += rows.dim[i];
            } else {
                if self.jaref[i] + alpha * self.jv[i] < 0.0 {
                    add(&mut total, quad);
                }
                i += 1;
            }
        }
        cost += alpha * alpha * total[2] + alpha * total[1] + total[0];
        deriv[0] += 2.0 * alpha * total[2] + total[1];
        deriv[1] += 2.0 * total[2];
        if deriv[1] <= 0.0 {
            log::warn!("Linesearch objective is not convex");
            deriv[1] = MJ_MINVAL;
        }
        self.ls_iter += 1;
        Pnt { alpha, cost, deriv }
    }

    /// Replace `p` by the candidate on its side of the minimum that comes
    /// closest, and evaluate the Newton step from it into `next`
    /// (`updateBracket`, `engine_solver.c:1298-1324`). Returns whether `p`
    /// changed.
    fn update_bracket(&mut self, p: &mut Pnt, candidates: &[Pnt; 3], next: &mut Pnt) -> bool {
        let mut updated = false;
        for c in candidates {
            if (p.deriv[0] < 0.0 && c.deriv[0] < 0.0 && p.deriv[0] < c.deriv[0])
                || (p.deriv[0] > 0.0 && c.deriv[0] > 0.0 && p.deriv[0] > c.deriv[0])
            {
                *p = *c;
                updated = true;
            }
        }
        if updated {
            *next = self.eval(p.alpha - p.deriv[0] / p.deriv[1]);
        }
        updated
    }

    /// The step along `search` (`PrimalSearch`, `engine_solver.c:1328-1511`):
    /// a Newton step from 0, Newton steps on one side until the slope
    /// changes sign, then a bracketed search over three candidates; the
    /// first point whose slope is under the scaled tolerance, else the best
    /// bracket end, else 0. Sets `ls_slope`.
    fn line_search(&mut self, tolerance: f64, ls_iterations: usize) -> f64 {
        self.ls_iter = 0;
        self.ls_slope = 1.0;
        let snorm = mju_norm(&self.search);
        if snorm < MJ_MINVAL {
            return 0.0;
        }
        let gtol = tolerance * snorm / self.scale;
        let slopescl = self.scale / snorm;
        self.mass.mul(&self.search, &mut self.mv);
        mul_mat_vec(&self.j, &self.search, &mut self.jv, self.nv);
        self.prepare();

        let p0 = self.eval(0.0);
        let mut p1 = self.eval(p0.alpha - p0.deriv[0] / p0.deriv[1]);
        if p1.deriv[0].abs() < gtol {
            self.ls_slope = p1.deriv[0].abs() * slopescl;
            return p1.alpha;
        }

        // One-sided search.
        let dir = if p1.deriv[0] < 0.0 { 1.0 } else { -1.0 };
        let mut p2 = p0;
        while p1.deriv[0] * dir <= -gtol && self.ls_iter < ls_iterations {
            p2 = p1;
            p1 = self.eval(p1.alpha - p1.deriv[0] / p1.deriv[1]);
            if p1.deriv[0].abs() < gtol {
                self.ls_slope = p1.deriv[0].abs() * slopescl;
                return p1.alpha;
            }
        }
        if self.ls_iter >= ls_iterations {
            self.ls_slope = p1.deriv[0].abs() * slopescl;
            return p1.alpha;
        }

        // Bracketed search.
        let mut p2next = p1;
        let mut p1next = self.eval(p1.alpha - p1.deriv[0] / p1.deriv[1]);
        while self.ls_iter < ls_iterations {
            let pmid = self.eval(0.5 * (p1.alpha + p2.alpha));
            let candidates = [p1next, p2next, pmid];
            let mut best: Option<Pnt> = None;
            for c in candidates {
                if c.deriv[0].abs() < gtol && best.is_none_or(|b| c.cost < b.cost) {
                    best = Some(c);
                }
            }
            if let Some(b) = best {
                self.ls_slope = b.deriv[0].abs() * slopescl;
                return b.alpha;
            }
            let b1 = self.update_bracket(&mut p1, &candidates, &mut p1next);
            let b2 = self.update_bracket(&mut p2, &candidates, &mut p2next);
            if !b1 && !b2 {
                self.ls_slope = pmid.deriv[0].abs() * slopescl;
                return pmid.alpha;
            }
        }

        // The bracket end with the lower cost, if it improves on 0.
        if p1.cost <= p2.cost && p1.cost < p0.cost {
            self.ls_slope = p1.deriv[0].abs() * slopescl;
            p1.alpha
        } else if p2.cost <= p1.cost && p2.cost < p0.cost {
            self.ls_slope = p2.deriv[0].abs() * slopescl;
            p2.alpha
        } else {
            0.0
        }
    }

    /// Newton: each row's `D` while quadratic, else 0.
    fn hessian_d(&mut self) {
        for i in 0..self.nefc {
            self.hd[i] = if self.state[i] == ConstraintState::Quadratic {
                self.rows.d[i]
            } else {
                0.0
            };
        }
    }

    /// The cone contacts as (first row, dim, Hessian), for the sparse
    /// Hessian.
    fn cones(&self) -> Vec<(usize, usize, ConeHessian)> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.nefc {
            if self.state[i] == ConstraintState::Cone {
                let dim = self.rows.dim[i];
                out.push((i, dim, self.cone_h[i]));
                i += dim;
            } else {
                i += 1;
            }
        }
        out
    }

    /// Make the Hessian `M + J'·D·J` and factor it (`MakeHessian` and
    /// `FactorizeHessian`, `engine_solver.c:1516-1670`): dense, `D` over the
    /// quadratic rows into the lower triangle (`mju_sqrMatTD_impl`), `M`
    /// added, factored with mjMINVAL for the smallest diagonal, then the
    /// cones added (`HessianCone`); sparse, [`SparseHessian`] with the cones.
    fn factorize_hessian(&mut self, model: &Model, data: &Data) -> Result<(), StepError> {
        let nv = self.nv;
        self.hessian_d();
        if model.is_sparse() {
            let cones = self.cones();
            let inputs = SparseInputs {
                model,
                data,
                dofs: &self.sub.dofs,
                parent: &self.parent,
                j: &self.j,
                d: &self.rows.d,
                ty: &self.rows.ty,
                dim: &self.rows.dim,
                implicit_sd: model.integrator == Integrator::ImplicitSpringDamper,
            };
            match &mut self.sparse {
                Some(h) => h.refactor(&inputs, &self.state, &cones)?,
                None => self.sparse = Some(SparseHessian::assemble(&inputs, &self.state, &cones)?),
            }
        } else {
            self.l.fill(0.0);
            for r in 0..self.nefc {
                let dr = self.hd[r];
                if dr == 0.0 {
                    continue;
                }
                let row = &self.j[r * nv..(r + 1) * nv];
                for i in 0..nv {
                    let tmp = row[i];
                    if tmp != 0.0 {
                        let scl = tmp * dr;
                        for (h, j) in self.l[i * nv..=i * nv + i].iter_mut().zip(row) {
                            *h += j * scl;
                        }
                    }
                }
            }
            self.mass.add_to_lower(&mut self.l, nv);
            mju_chol_factor(&mut self.l, nv, MJ_MINVAL);
            if self.ncone > 0 {
                self.hessian_cone();
            }
        }
        self.nupdate = self.nefc;
        Ok(())
    }

    /// Dense: the factor with each cone's contribution `J'·H·J` added by
    /// rank-one updates of `H`'s factor columns (`HessianCone`,
    /// `engine_solver.c:1674-1742`).
    fn hessian_cone(&mut self) {
        let nv = self.nv;
        self.lcone.copy_from_slice(&self.l);
        let mut ltj = vec![0.0; 6 * nv];
        let mut i = 0;
        while i < self.nefc {
            if self.state[i] != ConstraintState::Cone {
                i += 1;
                continue;
            }
            let dim = self.rows.dim[i];
            let mut local = self.cone_h[i];
            mju_chol_factor(&mut local[..dim * dim], dim, MJ_MINVAL);
            ltj[..dim * nv].fill(0.0);
            for r in 0..dim {
                let row = &self.j[(i + r) * nv..(i + r + 1) * nv];
                for c in 0..=r {
                    let scl = local[r * dim + c];
                    for (out, j) in ltj[c * nv..(c + 1) * nv].iter_mut().zip(row) {
                        *out += j * scl;
                    }
                }
            }
            for r in 0..dim {
                mju_chol_update(&mut self.lcone, &mut ltj[r * nv..(r + 1) * nv], nv, true);
            }
            self.nupdate += dim;
            i += dim;
        }
    }

    /// Update the factor for the rows whose quadratic state changed: a
    /// rank-one update with `√D·J` for each row that entered it, a downdate
    /// for each that left; made anew when an update loses rank; then the
    /// cones (`HessianIncremental`, `engine_solver.c:1746-1807`). Sparse:
    /// made anew, with MuJoCo's update count.
    // Rows are read across parallel arrays at one index, as MuJoCo's loop does.
    #[allow(clippy::needless_range_loop)]
    fn hessian_incremental(
        &mut self,
        model: &Model,
        data: &Data,
        old_state: &[ConstraintState],
    ) -> Result<(), StepError> {
        let nv = self.nv;
        self.nupdate = 0;
        if model.is_sparse() {
            let quad = ConstraintState::Quadratic;
            let changed = old_state
                .iter()
                .zip(&self.state)
                .filter(|(o, n)| (**o == quad) != (**n == quad))
                .count();
            self.factorize_hessian(model, data)?;
            self.nupdate = changed + self.cones().iter().map(|c| c.1).sum::<usize>();
            return Ok(());
        }
        let mut vec = vec![0.0; nv];
        for i in 0..self.nefc {
            let was = old_state[i] == ConstraintState::Quadratic;
            let is = self.state[i] == ConstraintState::Quadratic;
            if was == is {
                continue;
            }
            let s = self.rows.d[i].sqrt();
            for (v, j) in vec.iter_mut().zip(&self.j[i * nv..(i + 1) * nv]) {
                *v = j * s;
            }
            let rank = mju_chol_update(&mut self.l, &mut vec, nv, is);
            self.nupdate += 1;
            if rank < nv {
                return self.factorize_hessian(model, data);
            }
        }
        if self.ncone > 0 {
            self.hessian_cone();
        }
        Ok(())
    }
}

impl Primal<'_> {
    /// `mj_solPrimal`'s loop from the start it is given (`engine_solver.c:1811-1945`),
    /// improvement and gradient scaled by `scale`; returns each iteration's
    /// statistics.
    // The driver reads as `mj_solPrimal` does, start to end.
    #[allow(clippy::too_many_lines)]
    fn solve(
        &mut self,
        model: &Model,
        data: &Data,
        scale: f64,
    ) -> Result<Vec<SolverStat>, StepError> {
        let (nv, nefc, newton) = (self.nv, self.nefc, self.newton);
        let cone_hessian = newton && model.cone == 1;
        let mut gradold = vec![0.0; if newton { 0 } else { nv }];
        let mut mgradold = vec![0.0; if newton { 0 } else { nv }];
        let mut mgraddif = vec![0.0; if newton { 0 } else { nv }];

        self.mass.mul(&self.qacc, &mut self.ma);
        mul_mat_vec(&self.j, &self.qacc, &mut self.jaref, nv);
        for (x, a) in self.jaref.iter_mut().zip(&self.aref) {
            *x -= a;
        }

        self.update_constraint(cone_hessian);
        if newton {
            self.factorize_hessian(model, data)?;
        }
        self.update_gradient();
        for (s, g) in self.search.iter_mut().zip(&self.mgrad) {
            *s = g * -1.0;
        }
        self.scale = scale;

        let tolerance = model.solver_tolerance;
        let mut stats = Vec::new();
        while stats.len() < model.solver_iterations {
            let alpha = self.line_search(tolerance * model.ls_tolerance, model.ls_iterations);
            if alpha == 0.0 {
                break;
            }
            for i in 0..nv {
                self.qacc[i] += self.search[i] * alpha;
                self.ma[i] += self.mv[i] * alpha;
            }
            for i in 0..nefc {
                self.jaref[i] += self.jv[i] * alpha;
            }
            if !newton {
                gradold.copy_from_slice(&self.grad);
                mgradold.copy_from_slice(&self.mgrad);
            }
            let old_state = self.state.clone();
            let old_cost = self.cost;

            self.update_constraint(cone_hessian);
            if newton {
                self.hessian_incremental(model, data, &old_state)?;
            }
            self.update_gradient();

            let nchange = old_state
                .iter()
                .zip(&self.state)
                .filter(|(o, n)| o != n)
                .count();
            let improvement = self.scale * (old_cost - self.cost);
            let gradient = self.scale * mju_norm(&self.grad);
            stats.push(SolverStat {
                improvement,
                gradient,
                lineslope: self.ls_slope,
                nactive: self.nactive,
                nchange,
                neval: self.ls_iter,
                nupdate: self.nupdate,
            });
            if improvement < tolerance || gradient < tolerance {
                break;
            }

            if newton {
                for (s, g) in self.search.iter_mut().zip(&self.mgrad) {
                    *s = g * -1.0;
                }
            } else {
                // Polak-Ribière.
                for i in 0..nv {
                    mgraddif[i] = self.mgrad[i] - mgradold[i];
                }
                let mut beta =
                    mju_dot(&self.grad, &mgraddif) / MJ_MINVAL.max(mju_dot(&gradold, &mgradold));
                if beta < 0.0 {
                    beta = 0.0;
                }
                for i in 0..nv {
                    self.search[i] = -self.mgrad[i] + beta * self.search[i];
                }
            }
        }
        Ok(stats)
    }

    /// The solve's iterate, forces, residuals and states into `data` at its
    /// dofs and rows, and each of its contacts in the cone its cone Hessian.
    fn write_back(&self, data: &mut Data) {
        for (l, &dof) in self.sub.dofs.iter().enumerate() {
            data.qacc[dof] = self.qacc[l];
            data.qfrc_constraint[dof] = self.qfrc_constraint[l];
        }
        for (l, &row) in self.sub.rows.iter().enumerate() {
            data.efc_force[row] = self.force[l];
            data.efc_jar[row] = self.jaref[l];
            data.efc_state[row] = self.state[l];
        }
        let mut i = 0;
        while i < self.nefc {
            let dim = self.rows.dim[i];
            if self.state[i] == ConstraintState::Cone {
                data.efc_cone_hessian[self.rows.id[i]] = Some(DMatrix::from_row_slice(
                    dim,
                    dim,
                    &self.cone_h[i][..dim * dim],
                ));
                data.ncone += 1;
            }
            i += if self.rows.ty[i] == ConstraintType::ContactElliptic {
                dim
            } else {
                1
            };
        }
    }
}

/// Solve the constraint rows with MuJoCo's primal solver, Newton when
/// `newton`, else CG, from MuJoCo's warmstart, which reads `efc_b`
/// (`mj_fwdConstraint`, `engine_forward.c:754-797`).
///
/// With islands (enabled, at least one, no noslip; and the tree's mass, not
/// the implicit spring-damper integrator's, whose tendon terms join trees the
/// islands do not), each island is solved apart from the warmstart's
/// iterate over its own dofs and rows, scaled by 1/Σ diag of its mass matrix
/// (`mj_solPrimal`, `engine_solver.c:1861-1872`); `solver_niter` sums the
/// first [`MJ_NISLAND`] islands' iterations and `solver_stat` holds their
/// statistics in island order, as MuJoCo records them. A dof outside every
/// island keeps the warmstart's `qacc_smooth`, a row outside every island
/// the warmstart's force and state. Otherwise every row at once, scaled by
/// 1/(`stat_meaninertia`·max(1, nv)) (`mj_solPrimal`'s island −1).
///
/// Leaves the iterate in `qacc`, and `efc_force`, `efc_state`, `efc_jar`,
/// `efc_cost`, `qfrc_constraint`, the cone Hessians, `solver_niter` and
/// `solver_stat`. Each row's state is its solve's: MuJoCo copies an island's
/// forces back but not its rows' states, which stay the warmstart's
/// (registry `D-ISLAND-EFC-STATE`). `mass` and `qfrc_smooth` are the
/// problem's (the implicit spring-damper integrator's are its own).
///
/// # Errors
///
/// `StepError::CholeskyFailed` when the sparse Hessian has a diagonal under
/// mjMINVAL, where MuJoCo stops with "rank-deficient sparse Hessian".
// The driver reads as `mj_fwdConstraint`'s solver half does, start to end.
#[allow(clippy::cast_precision_loss)]
pub fn mj_sol_primal(
    model: &Model,
    data: &mut Data,
    mass: &PrimalMass<'_>,
    qfrc_smooth: &DVector<f64>,
    newton: bool,
) -> Result<(), StepError> {
    let whole = Subproblem::whole(model, data);
    let whole_mass = match mass {
        PrimalMass::Tree => Mass::Tree(TreeMass::new(model, data, &whole)),
        PrimalMass::Implicit { m, factor } => Mass::Implicit { m, factor },
    };
    let qfrc_smooth = qfrc_smooth.as_slice();
    let mut ctx = Primal::new(model, data, whole, whole_mass, qfrc_smooth, newton);
    ctx.warmstart(model, data, data.efc_b.as_slice());
    for h in &mut data.efc_cone_hessian {
        *h = None;
    }
    data.ncone = 0;

    let islands = matches!(mass, PrimalMass::Tree)
        && model.noslip_iterations == 0
        && data.nisland > 0
        && !disabled(model, DISABLE_ISLAND);
    if !islands {
        let scale = 1.0 / (model.stat_meaninertia * model.nv.max(1) as f64);
        let stats = ctx.solve(model, data, scale)?;
        ctx.write_back(data);
        data.efc_cost = ctx.cost;
        data.solver_niter = stats.len();
        data.solver_stat = stats;
        return Ok(());
    }

    // Each island from the warmstart's iterate; every row first takes the
    // warmstart's force and state at `qacc_smooth`, and `efc_b` as its
    // residual, which a row in no island keeps.
    data.qacc.as_mut_slice().copy_from_slice(&ctx.qacc);
    data.efc_force.as_mut_slice().copy_from_slice(&ctx.force);
    data.efc_state.clone_from(&ctx.state);
    data.efc_jar.copy_from(&data.efc_b);
    data.efc_cost = 0.0;
    data.solver_niter = 0;
    data.solver_stat.clear();
    for island in 0..data.nisland {
        let sub = Subproblem::island(model, data, island);
        let tree = TreeMass::new(model, data, &sub);
        let scale = 1.0 / tree.trace();
        let mut ictx = Primal::new(model, data, sub, Mass::Tree(tree), qfrc_smooth, newton);
        let stats = ictx.solve(model, data, scale)?;
        ictx.write_back(data);
        data.efc_cost += ictx.cost;
        if island < MJ_NISLAND {
            data.solver_niter += stats.len();
            data.solver_stat.extend(stats);
        }
    }
    let outside = Subproblem::outside(model, data);
    if !outside.rows.is_empty() {
        let tree = TreeMass::new(model, data, &outside);
        let mut octx = Primal::new(model, data, outside, Mass::Tree(tree), qfrc_smooth, newton);
        for (jar, &row) in octx.jaref.iter_mut().zip(&octx.sub.rows) {
            *jar = data.efc_b[row];
        }
        octx.update_constraint(false);
        data.efc_cost += octx.cost;
    }
    Ok(())
}
