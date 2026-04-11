# Future Work 10e — Deferred Item Tracker: Group 4 — Solver Optimizations

Part of the [Deferred Item Tracker](./future_work_10b.md) — see that file for full index and context.

---

## Group 4 — Solver Optimizations (9 items)

**Spec approach:** DT-38/41/42 each need individual specs (T3 — solver architecture).
DT-36/37/44 share a "Sparse Storage" spec (T2). DT-39/40 share a "Solver Robustness"
spec (T2). DT-43 implements directly (T1). Totals: 1 T1, 5 T2, 3 T3.

| §DT | Origin | Description | Priority | Tier |
|-----|--------|-------------|----------|------|
| DT-36 | §2 | Flat CSR format for `qLD_L` — replace `Vec<Vec<(usize, f64)>>` with CSR | Low | T2 |
| DT-37 | §2 | Sparse factorization for implicit integrator path — currently dense `cholesky_in_place` | Low | T2 |
| DT-38 | §3 | Implicit matrix-vector products for CG — avoid dense Delassus assembly for large contact counts | Medium | T3 |
| DT-39 | §15 | Body-weight diagonal approximation for `diagApprox` — add `body_invweight0`, `dof_invweight0` | Low | T2 |
| DT-40 | §15 | LDL^T factorization for Hessian — robustness on extreme stiffness settings | Low | T2 |
| ~~DT-41~~ | §15 | ~~Newton solver for implicit integrators — currently warns + falls back to PGS~~ **Done** — full Newton solver with warmstart, convergence, fallback; 8 tests | Low | T3 |
| DT-42 | §15/§16 | Per-island Newton solver dispatch — Hessian block-diagonal refactor needed | Low | T3 |
| DT-43 | §16 | Selective CRBA backward-pass optimization — defer to per-island block structure | Low | T1 |
| DT-44 | §16 | Sparse mass matrix for island extraction — currently dense `DMatrix` indexing | Low | T2 |
