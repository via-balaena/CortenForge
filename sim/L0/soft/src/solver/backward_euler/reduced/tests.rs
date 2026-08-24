//! Reduced-path unit tests that need `pub(super)` access.
//!
//! `project_tangent` is `pub(super)` within [`super`], so an integration test in
//! `tests/` cannot reach it and `backward_euler::tests` — a sibling, not a
//! descendant — cannot either. The reduced path's own internals are tested here;
//! everything reachable through the public API lives in `tests/reduced_*.rs`.

#![allow(
    // The vocabulary of a loud-failing test, hoisted once. Production code in
    // this module carries its own per-item allows.
    clippy::panic,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    // `x`/`y`/`r`/`n`/`p`/`v` mirror the matrix algebra the oracle reproduces
    // (`Y = AΦ`, rank `r`, size `n`) and carry the SAME justification the
    // production `project_tangent` records: longer names would break the
    // correspondence with the formula, which is the whole point of an oracle
    // written to be read against it.
    clippy::many_single_char_names
)]

use crate::solver::backward_euler::CpuNewtonSolver;
use crate::{
    BoundaryConditions, HandBuiltTetMesh, MaterialField, Mesh, NullContact, SolverConfig, Tet4,
    Vec3, pick_vertices_by_predicate,
};

/// A small pinned cantilever — `nz` must be even and `>= 2`.
fn rig(n_lat: usize, nz: usize) -> (crate::CpuTet4NHSolver<HandBuiltTetMesh>, Vec<f64>) {
    let field = MaterialField::uniform(1.0e5, 4.0e5);
    let mesh =
        HandBuiltTetMesh::cantilever_bilayer_beam(n_lat, n_lat, nz, 0.020, 0.020, 0.006, &field);
    let mut x_rest = vec![0.0; 3 * mesh.n_vertices()];
    for (c, p) in x_rest.chunks_exact_mut(3).zip(mesh.positions()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let bc = BoundaryConditions {
        pinned_vertices: pick_vertices_by_predicate(&mesh, |p: &Vec3| p.z.abs() < 1e-12),
        roller_vertices: Vec::new(),
        loaded_vertices: Vec::new(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0 / 60.0;
    cfg.density = 1030.0;
    (
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc),
        x_rest,
    )
}

/// ★ `project_tangent`'s layout change must be **BYTE-IDENTICAL**, not merely close.
///
/// Recon §2j knob 0 replaced a `Vec<Vec<f64>>` `Y` and `r(r+1)/2` column dot
/// products with a flat `n × r` buffer and `n` rank-1 updates. That is a pure
/// data-layout change: every `(i, j)` still sums its `p` terms low-to-high, so
/// the result is exact to the bit, and a tolerance-based test here would hide a
/// real reordering behind "close enough".
///
/// ⚠ The oracle is written from [`PodBasis::modes`] — the COLUMN-major copy —
/// while the SUT reads the flat row-major transpose. Different buffer, different
/// indexing, so a wrong transpose cannot satisfy both. An oracle that shared the
/// SUT's layout would pass with the transpose broken.
#[test]
fn project_tangent_layout_change_is_byte_identical() {
    use super::{Inner, PodBasis, ReducedNewtonSolver, SnapshotSet};

    let (solver, x_rest) = rig(4, 2);
    let free = solver.free_dof_indices().to_vec();
    let mass = solver.mass_per_free_dof();

    // Deterministic pseudo-random snapshots — the basis only has to be a real
    // `Φ` with orthonormal columns, not a physically meaningful one.
    let mut snaps = SnapshotSet::new(free.len());
    for k in 0..6_u64 {
        let u: Vec<f64> = (0..free.len())
            .map(|i| {
                let s = k
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(i as u64 + 1)
                    .wrapping_mul(2_862_933_555_777_941_757)
                    >> 33;
                1.0e-4 * ((s % 2000) as f64 / 1000.0 - 1.0)
            })
            .collect();
        snaps.push(&u);
    }
    let basis = PodBasis::fit(&snaps, Inner::Euclidean, &mass, 1.0, 5).expect("basis fits");
    let reduced = ReducedNewtonSolver::new(&solver, &basis, &x_rest);

    // Away from rest, so the tangent is not the rest-state special case.
    let mut x = x_rest.clone();
    for (i, &fd) in free.iter().enumerate() {
        x[fd] += 1.0e-4 * ((i % 7) as f64 - 3.0);
    }
    let dt = 1.0 / 60.0;

    let got = reduced.project_tangent(&x, Some(&x_rest), dt);

    // ── the pre-change form, verbatim ──
    let triplets = solver.assemble_free_hessian_triplets(&x, Some(&x_rest), dt);
    let modes = basis.modes();
    let (r, n) = (basis.n_modes(), basis.n_free());
    assert!(r >= 2, "need a non-trivial basis, got r = {r}");
    let mut y = vec![vec![0.0_f64; r]; n];
    for t in &triplets {
        let (row, col, v) = (t.row, t.col, t.val);
        for k in 0..r {
            y[row][k] += v * modes[k][col];
            if row != col {
                y[col][k] += v * modes[k][row];
            }
        }
    }

    let mut checked = 0_usize;
    for i in 0..r {
        for j in i..r {
            let want: f64 = (0..n).map(|p| modes[i][p] * y[p][j]).sum();
            assert!(want.is_finite(), "oracle produced {want} at ({i}, {j})");
            assert_eq!(
                got[(i, j)].to_bits(),
                want.to_bits(),
                "({i}, {j}): {} != {want} — the layout change altered the arithmetic, \
                 not just the memory order",
                got[(i, j)]
            );
            assert_eq!(got[(j, i)].to_bits(), want.to_bits(), "({j}, {i}) mirror");
            checked += 1;
        }
    }
    assert_eq!(
        checked,
        r * (r + 1) / 2,
        "every upper-triangle entry compared"
    );
    // The oracle must be capable of disagreeing: a basis of all-zero modes would
    // make every entry 0.0 and the comparison vacuous.
    assert!(
        (0..r).any(|i| got[(i, i)].abs() > 0.0),
        "vacuous: the whole projected tangent is zero"
    );
}
