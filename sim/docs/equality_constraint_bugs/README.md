# Equality Constraint Bugs

Found via stress-testing examples for the `equality-constraints/` domain.
Each bug has its own research document with root cause analysis and fix plan.

## Bugs

| # | Bug | Severity | Status |
|---|-----|----------|--------|
| 1 | [Weld-to-world drift in multi-constraint scenes](01_weld_multi_constraint_drift.md) | High | Investigating |
| 2 | [Body-to-body connect steady-state offset](02_connect_body_to_body_offset.md) | High | Investigating |
| 3 | [Double connect chain explosion](03_double_connect_explosion.md) | High | Investigating |

## Methodology

- Stress test binary: `examples/fundamentals/sim-cpu/equality-constraints/stress-test/`
- Single-constraint baselines all pass (0.37mm sag for weld, 0.31mm for connect)
- Bugs manifest only in multi-constraint or coupled scenarios
- Both PGS and Newton solvers produce identical results (solver converges in 1 iteration)
- Issue is in constraint formulation/assembly, NOT solver convergence

## Key files

- `sim/L0/core/src/constraint/equality.rs` — Jacobian extraction
- `sim/L0/core/src/constraint/equality_assembly.rs` — row assembly into efc_*
- `sim/L0/core/src/constraint/mod.rs` — unified constraint pipeline
- `sim/L0/core/src/constraint/solver/pgs.rs` — PGS solver
- `sim/L0/core/src/constraint/solver/newton.rs` — Newton solver
