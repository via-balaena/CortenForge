# Derivatives Examples Spec

**Status:** Stress-test implemented (32/32). Visual examples 1–6 pending.
**Date:** 2026-04-03
**Module:** `sim_core::derivatives` (6,029 LOC)

---

## Principle

One concept per example. Each example demonstrates exactly one public API
entry point or one distinct capability of the derivatives module. No example
combines multiple concepts — this makes debugging, teaching, and review
straightforward.

---

## API Surface

The derivatives module linearizes the discrete simulation step
`x_{t+1} = f(x_t, u_t)` and inverse dynamics. The public API:

| Function | Output | Purpose |
|----------|--------|---------|
| `mjd_transition_fd` | A, B [, C, D] | Pure finite-difference linearization |
| `mjd_transition_hybrid` | A, B [, C, D] | Hybrid analytical+FD (velocity cols analytical) |
| `mjd_transition` | A, B [, C, D] | Auto-dispatch (hybrid when available, else FD) |
| `mjd_inverse_fd` | DfDq, DfDv, DfDa | FD Jacobians of inverse dynamics |
| `validate_analytical_vs_fd` | (err_A, err_B) | Compare hybrid vs FD on a model |
| `fd_convergence_check` | bool | Check FD convergence at two epsilon scales |
| `max_relative_error` | (err, location) | Matrix comparison utility |

Supporting types: `TransitionMatrices` (A, B, C, D), `DerivativeConfig`
(eps, centered, use_analytical, compute_sensor_derivatives),
`InverseDynamicsDerivatives` (DfDq, DfDv, DfDa).

---

## Examples

### 1. `linearize-pendulum` — Pure FD Transition Derivatives

**Concept:** `mjd_transition_fd()` — the simplest entry point.

**Scene:** Single hinge pendulum (1 DOF, no actuator). Set `qpos[0]` to
upright (π) so the equilibrium is unstable.

**What it does:**
1. Build model via `Model::n_link_pendulum(1, ...)`.
2. Set `qpos[0] = π`, call `forward()`.
3. Call `mjd_transition_fd()` with default config.
4. Print the 2×2 A matrix (dq, qvel blocks).
5. Compute eigenvalues of A. The unstable mode has |λ| > 1 (discrete-time
   instability). For small dt, the continuous-time eigenvalue ≈ √(g/L).
6. HUD displays: A matrix entries, eigenvalues, stability verdict.

**Visual:** Static pendulum at upright position. HUD shows the A matrix and
eigenvalue analysis. The pendulum doesn't move — this is a snapshot
linearization, not a simulation.

**Why this is one concept:** It only calls `mjd_transition_fd()` and
inspects the output. No hybrid, no sensors, no control, no inverse dynamics.

**Checks (clock-only):**
- A matrix dimensions are 2×2 (2*nv × 2*nv, nv=1)
- B matrix dimensions are 2×0 (no actuators)
- A has an eigenvalue with magnitude > 1 (unstable at upright)
- C, D are None (sensor derivatives not requested)

---

### 2. `control-design` — LQR from Linearization

**Concept:** Using A, B to design a controller (the "why" of linearization).

**Scene:** Cart-pole (slide + hinge, 2 DOF). One motor actuator on the
slide joint. Set to upright unstable equilibrium.

**What it does:**
1. Build cart-pole model (slide for cart, hinge for pole, motor on slide).
2. Set upright equilibrium: `qpos[1] = π`.
3. Call `mjd_transition_fd()` → A (4×4), B (4×1).
4. Solve discrete-time LQR: `K = dlqr(A, B, Q, R)` via iterating the
   discrete Riccati equation (pure Rust, no external dependency).
5. Run closed-loop: `ctrl[0] = -K * x_error` at each step.
6. The cart-pole balances. Without K, it falls.

**Visual:** Cart-pole balancing at upright. HUD shows K gains and current
state error norm. The pole stays vertical — the linearization-based
controller works.

**Why this is one concept:** The concept is "linearization enables control
design." The LQR solve is just linear algebra — the derivative module's
contribution is producing A, B.

**Checks (clock-only):**
- A matrix is 4×4, B is 4×1
- After 500 closed-loop steps, pole angle stays within ±0.1 rad of upright
- K gain vector has 4 elements (one per state dimension)
- Without control (K=0), pole angle exceeds π/4 within 100 steps

---

### 3. `hybrid-vs-fd` — Hybrid Analytical+FD Comparison

**Concept:** `mjd_transition_hybrid()` vs `mjd_transition_fd()` — same
output, lower cost.

**Scene:** 3-link pendulum (3 DOF, no actuators). Random initial angles
to avoid trivial linearization at zero.

**What it does:**
1. Build `Model::n_link_pendulum(3, ...)`.
2. Set random `qpos`, call `forward()`.
3. Call `mjd_transition_fd()` → A_fd, B_fd. Time it.
4. Call `mjd_transition_hybrid()` → A_hyb, B_hyb. Time it.
5. Compute `max_relative_error(A_fd, A_hyb)` — should be < 1e-4.
6. Print both timings and the max relative error.
7. HUD shows: A_fd vs A_hyb element-wise difference heatmap (6×6 grid),
   timing ratio, max error.

**Visual:** Static 3-link pendulum at the random pose. HUD shows the
comparison table: timing, max error, and a small difference heatmap.

**Why this is one concept:** Directly compares the two methods. Doesn't do
control, sensors, or inverse dynamics.

**Checks (clock-only):**
- A_fd and A_hyb have identical dimensions (6×6)
- `max_relative_error(A_fd, A_hyb)` < 1e-4
- `max_relative_error(B_fd, B_hyb)` < 1e-4 (both 6×0, trivially equal)
- Hybrid wall-clock time < FD wall-clock time (or within 20% — timing is noisy)

---

### 4. `sensor-jacobians` — Sensor Derivatives (C, D)

**Concept:** `compute_sensor_derivatives: true` → C, D matrices.

**Scene:** Single hinge pendulum with a position sensor (`jointpos`) and
a velocity sensor (`jointvel`), plus one motor actuator. This gives
`nsensordata = 2`, `nu = 1`, `nv = 1`.

**What it does:**
1. Build pendulum model with sensors and actuator.
2. Set `qpos[0] = 0.5`, `ctrl[0] = 0.1`, call `forward()`.
3. Call `mjd_transition_fd()` with `compute_sensor_derivatives: true`.
4. Inspect C (2×2) and D (2×1).
5. C tells you: "how do sensor readings change when state changes?"
   D tells you: "how do sensor readings change when control changes?"
6. Print C, D matrices with labeled rows (jointpos, jointvel).
7. HUD shows C, D matrices and their physical interpretation.

**Visual:** Pendulum at 0.5 rad with applied torque. HUD shows C, D
matrices with sensor names labeling the rows.

**Why this is one concept:** Only demonstrates the sensor derivative
path (C, D). No hybrid, no control design, no inverse dynamics.

**Checks (clock-only):**
- C is Some, D is Some (derivatives requested)
- C dimensions: 2 × 2 (nsensordata × (2*nv))
- D dimensions: 2 × 1 (nsensordata × nu)
- C is not all zeros (sensors respond to state)
- D is not all zeros (sensors respond to control indirectly)

---

### 5. `inverse-dynamics` — Inverse Dynamics Derivatives

**Concept:** `mjd_inverse_fd()` — Jacobians of the inverse dynamics map.

**Scene:** Double pendulum (2 DOF, no actuators). Non-zero qpos and qvel
so the derivatives are nontrivial.

**What it does:**
1. Build `Model::n_link_pendulum(2, ...)`.
2. Set `qpos = [0.3, -0.5]`, `qvel = [1.0, -0.5]`, call `forward()`.
3. Call `mjd_inverse_fd()` → DfDq (2×2), DfDv (2×2), DfDa (2×2).
4. DfDq: how inverse-dynamics forces change with position (gravity/Coriolis).
   DfDv: how they change with velocity (Coriolis/damping).
   DfDa: how they change with acceleration (the mass matrix M).
5. Verify DfDa ≈ M (inverse dynamics is `M*qacc + bias = qfrc`, so ∂f/∂a = M).
6. Print all three matrices with physical interpretation.
7. HUD shows DfDq, DfDv, DfDa, and the M comparison.

**Visual:** Double pendulum at the specified pose. HUD shows the three
Jacobian matrices and highlights that DfDa ≈ mass matrix.

**Why this is one concept:** Only calls `mjd_inverse_fd()`. No transition
derivatives, no sensors, no control.

**Checks (clock-only):**
- DfDq, DfDv, DfDa each 2×2 (nv × nv)
- DfDa matches mass matrix M within 1e-5 (∂(M*a + bias)/∂a = M)
- DfDq is not all zeros (gravity dependence on position)
- DfDv is not all zeros (Coriolis dependence on velocity)

---

### 6. `convergence` — FD Accuracy and Epsilon Tuning

**Concept:** How `eps` and `centered` affect FD accuracy.

**Scene:** Single hinge pendulum with one motor actuator (1 DOF, 1 control).
Non-zero state for nontrivial derivatives.

**What it does:**
1. Build simple pendulum with motor.
2. Compute A at 5 epsilon values: 1e-3, 1e-4, 1e-5, 1e-6, 1e-7.
3. For each eps, compute both centered and forward differences.
4. Use the smallest-eps centered result as "ground truth."
5. Print error table: eps | centered error | forward error.
6. Centered error scales as O(eps²): halving eps → 4× smaller error.
   Forward error scales as O(eps): halving eps → 2× smaller error.
7. HUD shows the convergence table.

**Visual:** Static pendulum. HUD shows the convergence table with errors
at each epsilon, demonstrating the quadratic vs linear convergence.

**Why this is one concept:** Only varies `DerivativeConfig.eps` and
`DerivativeConfig.centered`. No hybrid, no sensors, no inverse, no control.

**Checks (clock-only):**
- Centered error at eps=1e-5 is < centered error at eps=1e-4
  (monotone convergence)
- Forward error at eps=1e-5 is < forward error at eps=1e-4
- Centered error < forward error at every tested epsilon
  (centered is strictly more accurate)
- `fd_convergence_check()` returns true at default epsilon

---

### 7. `stress-test` — Headless Comprehensive Validation

**Concept:** None (cross-cutting). Validates all derivative paths without
a window.

**Checks (32):**

**Dimensions (4):**
1. A is (2*nv+na) × (2*nv+na) for pendulum with activation (use
   `<general dyntype="filter" .../>` — `<motor>` is stateless, na=0)
2. B is (2*nv+na) × nu for pendulum with motor
3. Free joint: A is 12×12 (nv=6 → 2*6), B matches nu
4. Ball joint: A is 6×6 (nv=3 → 2*3)

**Eigenvalue analysis (3):**
5. Upright pendulum A has eigenvalue with |λ| > 1 (unstable)
6. Downward pendulum A has all eigenvalues with |λ| ≤ 1 (stable)
7. Eigenvalues are real for symmetric 1-DOF system

**FD convergence (3):**
8. Centered: halving eps quarters the error (O(eps²))
9. Forward: halving eps halves the error (O(eps))
10. `fd_convergence_check()` returns true at default eps

**Hybrid vs FD agreement (3):**
11. `max_relative_error(A_fd, A_hybrid)` < 1e-4 for 3-link pendulum
12. `validate_analytical_vs_fd()` A+B errors < 1e-3 for actuated system
13. `validate_analytical_vs_fd()` returns errors < 1e-3 for unactuated system

**Sensor derivatives (3):**
14. C is Some when `compute_sensor_derivatives = true`
15. D is Some when `compute_sensor_derivatives = true`
16. C, D are None when `compute_sensor_derivatives = false` (default)

**Inverse dynamics (3):**
17. DfDq, DfDv, DfDa dimensions are nv × nv
18. DfDa ≈ mass matrix M (within 1e-5)
19. DfDv has damping contribution on diagonal when joint has damping

**Integrator coverage (4):**
20. Euler: derivatives compute without error
21. ImplicitSpringDamper: derivatives compute without error
22. ImplicitFast: derivatives compute without error
23. RK4: derivatives compute without error (falls back to pure FD)

**Quaternion handling (3):**
24. Ball joint A is 6×6, not 8×8 (tangent space, not coordinate space)
25. Free joint A is 12×12, not 14×14
26. Ball joint hybrid vs FD A agreement (< 2e-3, floor=1e-6)

**Actuator / activation (3):**
27. B column count equals nu for multi-actuator system
28. Activation dynamics: A diagonal block for act has expected filter entries
    (requires `<general dyntype="filter" .../>` — `<motor>` is stateless, na=0)
29. Zero-control linearization matches passive dynamics (B*0 = 0 contribution)

**Contact sensitivity (2):**
30. A matrix changes when contact stiffness changes (contact in A)
31. A differs between contact/no-contact configurations

**Config edge cases (1):**
32. `eps = 1e-8` (very small) still produces finite, non-NaN matrices

---

## Summary

| # | Example | Concept | Checks |
|---|---------|---------|--------|
| 1 | [linearize-pendulum](linearize-pendulum/) | `mjd_transition_fd` | 4 |
| 2 | [control-design](control-design/) | LQR from A, B | 4 |
| 3 | [hybrid-vs-fd](hybrid-vs-fd/) | `mjd_transition_hybrid` vs FD | 4 |
| 4 | [sensor-jacobians](sensor-jacobians/) | C, D sensor derivatives | 5 |
| 5 | [inverse-dynamics](inverse-dynamics/) | `mjd_inverse_fd` | 4 |
| 6 | [convergence](convergence/) | eps + centered tuning | 4 |
| 7 | [stress-test](stress-test/) | Cross-cutting validation | 32 |
| | **Total** | | **57** |

## Run

```
cargo run -p example-derivatives-linearize-pendulum --release
cargo run -p example-derivatives-control-design --release
cargo run -p example-derivatives-hybrid-vs-fd --release
cargo run -p example-derivatives-sensor-jacobians --release
cargo run -p example-derivatives-inverse-dynamics --release
cargo run -p example-derivatives-convergence --release
cargo run -p example-derivatives-stress-test --release
```
