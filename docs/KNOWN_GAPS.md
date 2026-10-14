# Known Conformance Gaps

A living record of physics-conformance gaps that are currently tracked as
`#[ignore]`d tests rather than silently passing. Each entry names the tests,
the **measured** current magnitude (with the date measured — magnitudes drift
as the solvers improve, so the number lives here, not in the test's `ignore`
reason string), the evidence for the root cause, and status.

> Scope note: this file tracks *correctness gaps*. It deliberately excludes the
> many `#[ignore]`d tests that are simply **slow** (multi-minute RL/thermostat
> runs, `--release`-only heavy tests, benchmarks) — those are opt-in by design,
> not unresolved bugs.

---

## Gap 1 — closed 2026-10-14: golden-flag `qacc` constraint-solve residual

**Tests:** `sim/L0/tests/integration/golden_flags.rs`, all 26 MuJoCo-3.4.0
golden-flag tests; 24 of them were `#[ignore]`d with a `qacc` residual of
~2e-3 at step 0–2 (measured 2026-07-05) against `TOLERANCE = 1e-8`.

**Closed by** Newton and CG becoming MuJoCo's primal solver (book P34): all 26
pass at `1e-8`, none is ignored. The residual came from Newton: removing a row
from its Hessian's factor added it again, and an unconverged solve fell back to
PGS. Fixing the removal alone made them pass (A18 §1, measured on the planning
prototype). Given MuJoCo's own inputs the solver now matches MuJoCo 3.5.0 bit
for bit (`integration/primal_solver_parity.rs`).

---

## Gap 2 — mm-scale free-body contact stacking at the default timestep

**Test:** `sim/L0/tests/integration/collision_primitives.rs::sphere_stack_dynamic`.

**Symptom (measured 2026-07-05):** two free-body spheres (radius 5 mm, mass
0.655 g) dropped to stack under gravity **collapse** to `gap ≈ 0` instead of
resting one-on-top-of-the-other. Observed final heights `z_lo = z_up = 4.988`
(tightened `solref="0.005 1"`) and `4.804` (default solref) — the upper sphere
sinks through the lower.

**NOT universal — it is a timestep/scale-stiffness limit, not a missing
feature.** The same scenario stacks correctly outside the mm-scale +
default-`dt` corner (measured 2026-07-05, from the diagnostic sweep that
previously lived beside this test and is summarized here):

| Configuration                         | Result            | gap (expected) |
| ------------------------------------- | ----------------- | -------------- |
| meter-scale, `dt = 0.002`             | **stacks** ✓      | 0.999 (1.0)    |
| mm-scale, `dt = 0.002`, tight solref  | collapses ✗       | 0.000 (10)     |
| mm-scale, `dt = 0.002`, default solref| collapses ✗       | 0.000 (10)     |
| mm-scale, `dt = 0.0005`               | **stacks** ✓      | 9.608 (10)     |
| mm-scale, `dt = 0.0001`               | **stacks** ✓      | 9.608 (10)     |

The mm-scale contact natural frequency is ~1000× higher than at m-scale, so the
default `0.002 s` step cannot resolve the contact dynamics and the pair
tunnels. This is the same mm-scale stiffness that `cf-design`'s model builder
works around by dropping to a `0.0005 s` (2 kHz) timestep.

**Status:** open for the mm-scale + default-`dt` regime. A real fix is either an
implicit/stiff contact integrator or automatic timestep selection at mm-scale —
engine-track work, not SDK tightening. (The earlier
`"free-body stacking not implemented yet"` framing was inaccurate: it *is*
implemented and works in every regime except this one.)
