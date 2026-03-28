# Solvers

Constraint solver examples. All use the same scene — two boxes stacked on
a ground plane — to isolate solver behavior from scene complexity.

## The 3 solvers

| Solver | Method | Iterations (this scene) |
|--------|--------|------------------------|
| **PGS** | Projected Gauss-Seidel (dual-space, iterative) | ~47 |
| **CG** | Conjugate Gradient (primal, M^-1 preconditioner) | ~31 |
| **Newton** | Full Newton (primal, H^-1, line search) | ~0 |

All three produce identical results — a rock-solid stack. The difference
is convergence speed: Newton uses quadratic convergence to solve the same
contact problem in 0-2 iterations that PGS needs ~47 for.

## Examples

| Example | Type | What it shows |
|---------|------|---------------|
| `comparison/` | Headless | Formatted table + 7 pass/fail checks |
| `comparison-visual/` | Bevy | 3 stacks side by side, HUD with live iteration counts |
| `pgs/` | Bevy | Single PGS stack with solver HUD |
| `cg/` | Bevy | Single CG stack with solver HUD |
| `newton/` | Bevy | Single Newton stack with solver HUD + fallback status |

## Run

```
cargo run -p example-solver-comparison --release          # headless table
cargo run -p example-solver-comparison-visual --release   # side-by-side
cargo run -p example-solver-pgs --release                 # PGS
cargo run -p example-solver-cg --release                  # CG
cargo run -p example-solver-newton --release              # Newton
```
