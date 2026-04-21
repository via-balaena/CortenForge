# Regression tests against MuJoCo flex baseline

Cross-solver drift is the class of bug that unit tests cannot catch — every trait impl returns its own spec'd values, every module passes its own suite, and the composition silently moves away from what a second, independent solver says the same scene looks like. The regression-vs-MuJoCo-flex harness is `sim-soft`'s insurance against that drift. The [parent Ch 04 claim 2](../04-testing.md) sets the scope: MuJoCo flex is a regression *baseline*, not a fidelity target — the two solvers solve different PDEs and will not agree numerically, and the test does not fail on numerical disagreement. It fails when the *direction* of disagreement flips against what [Part 1 Ch 03's thesis](../../10-physical/03-thesis.md) predicts.

## The oracle — Track 1B platform coverage

[Part 12 Ch 01](../../120-roadmap/01-track-1b.md) commits MuJoCo flex as coverage-and-regression infrastructure that ticks along with Phases A–I without gating any of them. Flex is MuJoCo's beta soft-body module: tet-mesh-based, running on MuJoCo's sparse Newton and constraint-projection machinery, with a simplified elastic constitutive model and a penalty-style contact formulation. It is not IPC; it is not differentiable in the sense [Part 6](../../60-differentiability/00-what-autograd-needs.md) requires; and it does not carry SDF-valued material fields. It *is* an independent, shipped, actively-maintained soft-body solver that can run `sim-soft`'s canonical scenes at matched parameters — which is exactly the shape a regression oracle needs.

The concrete regression-harness deliverable Track 1B ships is:

- One canonical-problem example running against flex on the same SDF-authored cavity-and-probe geometry from [Part 1 Ch 00](../../10-physical/00-canonical.md), with the closest available approximation of `sim-soft`'s material and contact parameters;
- A comparison test that reads `sim-soft`'s and flex's outputs on a shared small-scale scene and asserts the disagreement has the right shape;
- A documented "what flex cannot do" table whose rows grow as `sim-soft` adds capabilities flex has no counterpart for (IPC contact, exact IFT gradients, SDF-field materials, multi-layer viscoelasticity).

This sub-leaf specifies how the comparison test reads the two solvers' outputs and how it decides pass/fail.

## What gets compared — `Observable` outputs, not solver state

The regression test does not compare raw solver state. Per-vertex positions would disagree by construction — different time integrators with different contact formulations produce different trajectories even at matched boundary conditions, and an equality test at the position-vector level would fire on every run for reasons unrelated to correctness. The comparison is at the [`Observable` trait](../01-traits/00-core.md) surface from `sim-soft`'s [`readout/`](../00-module-layout/09-readout.md) and its flex-backed analogue; specifically, the γ-locked [`RewardBreakdown`](../../100-optimization/00-forward.md) and the `stress_field` / `pressure_field` outputs.

Concretely, the test reads four quantities at converged equilibrium on a matched scene:

| Quantity | Source on both sides | Tolerance basis |
|---|---|---|
| `RewardBreakdown.pressure_uniformity` | `sim-soft::readout::Observable::reward_breakdown`; flex-side analogue over the same surface triangles | Variance of a scalar field — a single real number per scene; 5–15% tolerance on elastic-only scenes per Track 1B's 1%-reward-ish budget |
| `RewardBreakdown.coverage` | Both sides read from the same surface-contact predicate applied to their respective deformed meshes | Fraction in $[0, 1]$; 2–5 percentage-point tolerance |
| `RewardBreakdown.peak_pressure` | The smoothed-max over contact-pressure values on both sides | 15–25% tolerance — the peak is dominated by the contact formulation's stiffness, and flex's penalty stiffness is a tuning knob with no direct `sim-soft` counterpart |
| von Mises stress field histogram | `Observable::stress_field` on both sides, binned over the mesh | Earth-mover distance on the binned histogram; thresholded against a calibrated reference run |

None of the thresholds above are load-bearing numbers in this sub-leaf — they are placeholder envelopes pending the Phase B–C calibration runs that [Track 1B](../../120-roadmap/01-track-1b.md)'s first regression will ship. What *is* load-bearing is the choice of Observable-layer comparison, because it means the regression harness exercises exactly the surface [`readout/`](../00-module-layout/09-readout.md) is committed to — if a future `sim-soft` change silently alters the `stress_field` definition, the regression catches it against a solver that never made that change.

## The direction-of-disagreement test

The [parent Ch 04 claim 2](../04-testing.md) cashes out here. Disagreement between `sim-soft` and flex is expected in four directions specifically named by [Part 1 Ch 02](../../10-physical/02-what-goes-wrong.md) and predicted by [the thesis](../../10-physical/03-thesis.md):

- **Volume loss under squeeze.** [Part 1 Ch 02 §00](../../10-physical/02-what-goes-wrong/00-volume-loss.md) names volume loss as flex's expected failure on high-strain deformation; [Part 2 Ch 04](../../20-materials/04-hyperelastic.md)'s near-incompressible neo-Hookean is `sim-soft`'s fix. Expected direction: $|\Delta V|_\text{flex} > |\Delta V|_\text{sim-soft}$.
- **Popping near separation.** [Part 1 Ch 02 §02](../../10-physical/02-what-goes-wrong/02-popping.md) + [Part 4 Ch 00](../../40-contact/00-why-ipc.md) predict penalty contact will exhibit discontinuous peak-pressure jumps as the probe retracts through the penalty band; IPC's smooth barrier gives a continuous release. Expected direction: the spectral energy of `peak_pressure`'s time derivative above 100 Hz is higher on flex than on `sim-soft`.
- **Wobble after release.** [Part 1 Ch 02 §01](../../10-physical/02-what-goes-wrong/01-wobble.md) predicts an under-damped oscillation on the flex side when the probe retracts, and a damped viscoelastic response on the `sim-soft` side once [Part 2 Ch 07 Prony viscoelasticity](../../20-materials/07-viscoelastic.md) lands in Phase H. Expected direction: the decay time of the surface-displacement mode on flex exceeds `sim-soft`'s by the Phase-H viscoelastic spectrum's dominant $\tau$.
- **Rim deformation at the contact edge.** [Part 1 Ch 02 §04](../../10-physical/02-what-goes-wrong/04-rim.md) predicts that Tet4-only meshes on both solvers will look polygonal at the contact edge, but that `sim-soft`'s Phase H Tet10 + adaptive refinement will produce a smoother rim. Expected direction (Phase H onward): the surface-curvature histogram near the contact edge is smoother on `sim-soft` than on flex.

The regression test fires on each of the four directions as an asymmetric inequality, not as a two-sided bounds check. Disagreement in the *expected* direction, within the calibrated envelope, passes. Disagreement in the *opposite* direction — flex shows less volume loss, more damping, less popping, or smoother rim than `sim-soft` — fails and blocks merge. Disagreement outside the envelope in either direction flags the test as *needs-recalibration*, not *failed*; calibration drift is itself a signal that the thesis-alignment has shifted and warrants investigation rather than a green CI tick.

This is what makes the regression harness load-bearing. A numerical match would be an accident; the *signed* disagreement is the thing the thesis claims, and the thesis is what the test enforces.

## Cadence and Phase landing

Per the [parent Ch 04 spine](../04-testing.md), the regression suite runs weekly in CI and pre-merge on any PR that touches `solver/`, `element/`, or `contact/` — the three modules whose changes are most likely to perturb cross-solver agreement. The phase-by-phase activation follows [Track 1B's touchpoints](../../120-roadmap/01-track-1b.md):

- **Phase B.** First regression fires: a ~1k-tet elastic-only compression benchmark, no contact, comparing the composed `RewardBreakdown.pressure_uniformity` and the stress-field histogram. The reference tolerance is calibrated at Phase B landing and frozen as the Phase B baseline.
- **Phase C.** Contact comes online; the regression extends to `coverage` and `peak_pressure`. The popping direction-of-disagreement test activates. [Part 4 Ch 00's](../../40-contact/00-why-ipc.md) IPC-vs-penalty argument becomes a test assertion at this phase.
- **Phase D.** [`ForwardMap::evaluate`](../../100-optimization/00-forward.md) composes end-to-end on CPU; the regression now reads through the γ-locked `ForwardMap` surface and asserts the composed reward's direction-of-disagreement on the canonical scene. No new failure directions; the existing ones run through the `ForwardMap` composition now.
- **Phase H.** Tet10, Mooney-Rivlin, and Prony viscoelasticity land. Many rows of the test matrix become "N/A in flex" per Track 1B — flex has no Tet10, no Mooney-Rivlin, no Prony. The harness prints these rows as coverage-missing rather than failure, which is itself a signal that `sim-soft`'s capability surface has grown past flex's. The wobble-decay-time and rim-curvature-histogram tests activate at this phase.
- **Phase I.** Visual-layer parity — the [`sim-bevy` shader pipeline](../../90-visual/05-sim-bevy.md) reads per-vertex attributes from either backend per [Track 1B](../../120-roadmap/01-track-1b.md); a visual feature that only works with `sim-soft` is flagged as a platform regression, not a feature. This is where [visual-regression §02](02-visual.md) and this sub-leaf overlap: `02-visual.md` owns the golden-image comparison, this sub-leaf owns the cross-solver-shader-input check at the backend-agnostic boundary.

The full regression suite runs in minutes on a single host — both solvers are production-tuned on their respective canonical scenes — so the weekly-CI cadence is a budget ceiling, not a floor. Pre-merge triggers on `solver/ | element/ | contact/` changes are unconditional.

## What regression does not test

- **Numerical equality.** The comparison is signed-direction, not magnitude-matching. A test that asserted `sim-soft` and flex match to 3 digits on a contact-rich scene would never pass — the two solvers are solving different PDEs.
- **Per-vertex position equality.** Different integrators produce different trajectories. The comparison reads `Observable`-layer outputs, not raw `NewtonStep` state.
- **GPU parity.** CPU-vs-GPU 5-digit agreement is [gradcheck §03](03-gradcheck.md)'s Phase E deliverable, not this harness's concern. The regression suite runs on the CPU `sim-soft` path — GPU drift is caught by the CPU-vs-GPU gradcheck, not by cross-solver regression.
- **Full-visual-output regression.** Shader-level output comparison against golden images is [visual regression §02](02-visual.md)'s job. This sub-leaf reads per-vertex attribute *values*; the shader's pixel-level composition of those values is the next layer.
- **Physical-print-loop regression.** [Part 10 Ch 05 `MeasurementReport` ingestion](../../100-optimization/05-sim-to-real.md) compares sim outputs against measured physical prints, and is [post-Phase-I per Part 12 Ch 07](../../120-roadmap/07-open-questions.md). No row of this harness binds against real-world measurement data; a printed-artifact-vs-sim comparison is a different test class with a different oracle and different tolerances, and it lands as its own harness post-Phase-I rather than being absorbed into this one.

## What this sub-leaf commits downstream

- **[Track 1B's regression deliverable](../../120-roadmap/01-track-1b.md)** binds to this sub-leaf's direction-of-disagreement predicates; the "1% reward-ish" tolerance in Track 1B's elastic-only benchmark is the Phase B row of this harness's table.
- **[`readout/`'s Observable outputs](../00-module-layout/09-readout.md)** are the comparison surface — any addition of a new `RewardBreakdown` term (see [Ch 00 readout claim 1](../00-module-layout/09-readout.md)) requires a corresponding column in this harness or a documented "N/A in flex" annotation.
- **[The thesis's direction-of-disagreement predictions](../../10-physical/03-thesis.md)** from [Part 1 Ch 02's](../../10-physical/02-what-goes-wrong.md) failure-mode inventory become test assertions at the Phase landings above. A failing test is prima facie evidence the thesis is wrong on this scene, or that the implementation has drifted from the thesis — either way a merge-blocker that needs investigation, not a "flake" to re-run.
