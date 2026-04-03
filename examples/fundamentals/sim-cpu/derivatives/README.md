# Derivatives — Finite-Difference Linearization

Seven examples demonstrating the derivatives module, which linearizes the
discrete simulation step `x_{t+1} = f(x_t, u_t)` into state-space matrices
A, B, C, D. These matrices are the foundation for LQR, MPC, iLQR/DDP,
system identification, and any control design workflow.

## API

- **`mjd_transition_fd`** — pure finite-difference linearization. Perturbs
  each state/control component, steps the simulation, measures differences.
  Works with any integrator, captures contact transitions naturally.
- **`mjd_transition_hybrid`** — hybrid analytical+FD. Uses analytical
  velocity derivatives (from `qDeriv`) for velocity columns of A, FD only
  for position columns. ~2x faster than pure FD.
- **`mjd_transition`** — auto-dispatch (hybrid when available, else FD).
- **`mjd_inverse_fd`** — FD Jacobians of inverse dynamics (DfDq, DfDv, DfDa).
- **`DerivativeConfig`** — controls epsilon, centered vs forward differences,
  hybrid mode, and sensor derivative computation.

## Examples

| Example | Concept | Checks |
|---------|---------|--------|
| [stress-test](stress-test/) | Headless: dimensions, eigenvalues, convergence, hybrid agreement, sensors, inverse, integrators, quaternions, contacts | 32 |
| linearize-pendulum | `mjd_transition_fd` — A, B matrices, eigenvalue analysis | 4 |
| control-design | LQR from linearization — A, B to closed-loop balance | 4 |
| hybrid-vs-fd | `mjd_transition_hybrid` vs FD — accuracy + timing | 4 |
| sensor-jacobians | C, D sensor derivative matrices | 5 |
| inverse-dynamics | `mjd_inverse_fd` — DfDq, DfDv, DfDa | 4 |
| convergence | eps + centered/forward tuning — O(eps) vs O(eps^2) | 4 |

## Run

```
cargo run -p example-derivatives-stress-test --release
```
