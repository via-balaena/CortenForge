//! Free-function helpers shared across the backward-Euler submodules.

use faer::sparse::Triplet;
use nalgebra::{Matrix3, SMatrix};

use crate::Vec3;
use crate::material::Material;
use crate::mesh::{Mesh, TetId, VertexId};

// ── free functions (assembly kernels) ──────────────────────────────────

/// Compose the `N` ordered element-local node ids from a tet's 4 corners and
/// (for a higher-order element, `N > 4`) its 6 midside nodes — consuming the
/// rung-3a additive midside channel (§3.2 of `docs/SIM_SOFT_TET10_PLAN.md`).
/// For a linear element (`N == 4`) the midsides are ignored and this returns
/// exactly [`Mesh::tet_vertices`]. Slots `4..N` follow the corners in the
/// canonical [`TET10_EDGE_NODES`](crate::element::TET10_EDGE_NODES) order, as
/// [`Mesh::tet_midside_nodes`] returns them.
///
/// The single source of the `[VertexId; N]` node list every generic assembly
/// kernel reads — the deformation-gradient / stiffness kernels, the HRZ mass
/// scatter, and the symbolic-incidence build all route through it so the node
/// ordering can never drift between them.
//
// expect_used: a higher-order (N > 4) element is only ever constructed against
// a midside-surfacing mesh (e.g. `Tet10Mesh`); a `None` here is a
// construction-time programmer error, declared in `new()`'s `# Panics`.
#[allow(clippy::expect_used)]
pub(super) fn element_node_ids<M, Msh, const N: usize>(mesh: &Msh, tet_id: TetId) -> [VertexId; N]
where
    M: Material,
    Msh: Mesh<M>,
{
    let mut nodes: [VertexId; N] = [0; N];
    nodes[..4].copy_from_slice(&mesh.tet_vertices(tet_id));
    if N > 4 {
        // `N > 4` is Tet10 (`N == 10`) in this ladder — the six midsides fill
        // slots `4..10`.
        let mids = mesh.tet_midside_nodes(tet_id).expect(
            "a higher-order (N > 4) element requires a mesh that surfaces midside \
             nodes, but Mesh::tet_midside_nodes returned None",
        );
        nodes[4..].copy_from_slice(&mids);
    }
    nodes
}

/// Extract one element's `N` node positions from the global DOF vector as an
/// `N × 3` matrix (row `a` = node `a`'s xyz). `x_elem[(a, k)] = x_full[3 *
/// nodes[a] + k]`.
///
/// For `N = 4` this reproduces the pre-rung-4 `[f64; 12]` vertex-major layout
/// value-for-value (`x_elem[(a, k)] == old x_elem[3 * a + k]`), so the
/// downstream [`deformation_gradient`] stays bit-identical.
pub(super) fn extract_element_dof_values<const N: usize>(
    x_full: &[f64],
    nodes: &[VertexId; N],
) -> SMatrix<f64, N, 3> {
    SMatrix::<f64, N, 3>::from_fn(|a, k| x_full[3 * nodes[a] as usize + k])
}

/// Per-element deformation gradient `F_ij = Σ_a x_{a,i} · ∂N_a/∂X_j`
/// (direct form) at one Gauss point.
///
/// `x_elem` is the `N × 3` element node matrix from
/// [`extract_element_dof_values`]; `grad_x_n` is that point's material-frame
/// shape gradient. At rest (`x_a = X_a`) returns the identity. The explicit
/// `a`-outer accumulation order is preserved from the pre-rung-4 `N = 4` loop
/// so Tet4 stays bit-identical (do not replace with a `nalgebra` matmul — that
/// reorders the reduction).
pub(super) fn deformation_gradient<const N: usize>(
    x_elem: &SMatrix<f64, N, 3>,
    grad_x_n: &SMatrix<f64, N, 3>,
) -> Matrix3<f64> {
    let mut f = Matrix3::zeros();
    for a in 0..N {
        for i in 0..3 {
            let x_ai = x_elem[(a, i)];
            for j in 0..3 {
                f[(i, j)] += x_ai * grad_x_n[(a, j)];
            }
        }
    }
    f
}

/// Full-DOF residual per scope §5 R-5: `r = (M/Δt²)·(x - x_prev -
/// Δt·v_prev) + f_int(x) - f_ext(θ)`, written into the caller-owned
/// `r` buffer.
///
/// `mass_per_dof` is the diagonal of the lumped mass matrix; for
/// vertices shared across multiple elements this exceeds `ρ V_e / 4`
/// of any single element. `mass / (dt * dt)` (rather than
/// `mass * (1.0 / (dt * dt))`) preserves the pre-Phase-2 1-tet
/// expression order at the last bit.
//
// 8 args mirrors the 8 input slices the residual formula reads from;
// bundling into a struct would add a name-the-fields ceremony for two
// callers with no readability gain.
#[allow(clippy::too_many_arguments)]
pub(super) fn residual_into(
    x_curr: &[f64],
    x_prev: &[f64],
    v_prev: &[f64],
    f_int: &[f64],
    f_ext: &[f64],
    mass_per_dof: &[f64],
    dt: f64,
    r: &mut [f64],
) {
    debug_assert!(x_curr.len() == r.len());
    debug_assert!(x_prev.len() == r.len());
    debug_assert!(v_prev.len() == r.len());
    debug_assert!(f_int.len() == r.len());
    debug_assert!(f_ext.len() == r.len());
    debug_assert!(mass_per_dof.len() == r.len());
    let dt2 = dt * dt;
    for i in 0..r.len() {
        let x_hat = dt.mul_add(v_prev[i], x_prev[i]);
        let mass_over_dt2 = mass_per_dof[i] / dt2;
        r[i] = mass_over_dt2.mul_add(x_curr[i] - x_hat, f_int[i]) - f_ext[i];
    }
}

/// Standard Armijo-stall panic message text, shared by `solve_impl`,
/// `Solver::try_step`, and `Solver::try_replay_step` so the three
/// panic surfaces emit byte-identical stderr regardless of which
/// entry point tripped the stall. Pulled into a free fn to dodge
/// the multi-line-string-literal indentation trap where continuation
/// lines silently embed leading whitespace into the message text —
/// drifted across the three call sites pre-polish (21-space vs
/// 17-space continuation), making the panic UX site-dependent.
pub(super) fn armijo_stall_panic_message(last_iter: usize, last_r_norm: f64) -> String {
    format!(
        "Armijo line-search stalled at Newton iter {last_iter} \
         (r_norm {last_r_norm:e}). Likely causes: non-SPD tangent \
         near solution (spec §3 R-2 violation), or near-singular \
         condensed system."
    )
}

/// Maximum diagonal entry magnitude (signed, not abs) across a
/// lower-tri triplet list. F3 uses this to scale the seed and ceiling
/// of the LM `+λI` retry loop relative to the assembled tangent's
/// dominant diagonal (per spec §2.1 / §2.2).
///
/// Returns `0.0` for a triplet list with no diagonal entries — the
/// caller (`factor_free_tangent`) `debug_assert!`s on `> 0`, so an
/// all-off-diagonal list trips fail-fast at the assertion. Production
/// assembly (`assemble_free_hessian_triplets`) always scatters a
/// positive mass-diagonal entry per free DOF, so the structural
/// invariant holds.
pub(super) fn triplets_max_diag(triplets: &[Triplet<usize, usize, f64>]) -> f64 {
    triplets
        .iter()
        .filter(|t| t.row == t.col)
        .map(|t| t.val)
        .fold(0.0_f64, f64::max)
}

/// Clone a triplet list and add `lambda` to each diagonal entry in
/// place (the F3 `+λI` regularization step per spec §2.1). Off-diagonal
/// entries are copied unchanged. Preserves triplet ordering (the
/// cached symbolic factor depends on it).
///
/// Mutates EXISTING diagonal entries — does NOT append fresh
/// `(k, k, λ)` triplets — because `SparseColMat::try_new_from_triplets`
/// rejects duplicates. The `assemble_free_hessian_triplets` mass
/// scatter (search for `Mass diagonal:`) guarantees a diagonal entry
/// per free DOF, so every `(k, k)` is mutated exactly once.
/// `O(n_triplets)` clone + `O(n_triplets)` walk = `O(n_triplets)`;
/// negligible vs the Llt factor cost.
pub(super) fn triplets_with_diagonal_offset(
    triplets: &[Triplet<usize, usize, f64>],
    lambda: f64,
) -> Vec<Triplet<usize, usize, f64>> {
    let mut out = triplets.to_vec();
    for t in &mut out {
        if t.row == t.col {
            t.val += lambda;
        }
    }
    out
}

/// Convert a flat `[f64]` DOF buffer (length `3·N`, vertex-major +
/// xyz-inner) to a `Vec<Vec3>` of length `N`.
///
/// Bridges the solver's flat representation to
/// [`crate::contact::ContactModel`]'s `&[Vec3]` argument shape (trait
/// surface unchanged from the pre-penalty era). Allocates per call
/// site — `assemble_global_int_force` and
/// `assemble_free_hessian_triplets` each build their own vec at the
/// contact-dispatch step. Negligible vs FEM assembly even at the
/// Hertzian fixture's finest-level scale.
pub(super) fn slice_to_vec3s(x_flat: &[f64]) -> Vec<Vec3> {
    debug_assert!(x_flat.len().is_multiple_of(3));
    let n = x_flat.len() / 3;
    let mut out = Vec::with_capacity(n);
    for v in 0..n {
        out.push(Vec3::new(
            x_flat[3 * v],
            x_flat[3 * v + 1],
            x_flat[3 * v + 2],
        ));
    }
    out
}
