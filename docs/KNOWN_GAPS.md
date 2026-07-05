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

## Gap 1 — Golden-flag `qacc` constraint-solve residual

**Tests:** `sim/L0/tests/integration/golden_flags.rs` — 24 of the 26
MuJoCo-3.4.0 golden-flag conformance tests (`golden_baseline` +
`golden_disable_*` / `golden_enable_*`).

**Symptom (measured 2026-07-05):** each failing test reports a `qacc` mismatch
at step 0–2, dof 0, with a **uniform absolute residual of ~2e-3** (full range
observed: `3.5e-5` … `2.2e-3`) against `TOLERANCE = 1e-8`. On values of
`qacc ≈ 20–130` this is a relative error of ~2e-5.

**Root cause — the constraint solve.** Two flags make the residual vanish
(both drop below `1e-8`, and are therefore **no longer `#[ignore]`d**):

- `golden_disable_constraint` — disabling the constraint solver entirely.
- `golden_disable_frictionloss` — disabling the frictionloss term.

So the residual lives in the constraint / friction solve path, not the base
articulated-body dynamics.

**History:** the `ignore` strings previously claimed `qacc diff ~1.69` (and
per-flag values up to `~25.9`). The gap has since shrunk ~1000× to the uniform
~2e-3 above; the old magnitudes were stale. Reason strings now point here
instead of hard-coding a number that re-rots.

**Status:** open. Reaching `1e-8` likely requires bit-level matching of
MuJoCo's constraint solver (PGS/Newton iteration order, warm-start, impedance
arithmetic) and may be aspirational at that tolerance. Not scheduled — belongs
to the professional-tier engine track, not SDK tightening.

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
