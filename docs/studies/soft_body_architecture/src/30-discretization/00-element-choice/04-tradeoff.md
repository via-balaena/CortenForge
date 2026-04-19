# Cost/accuracy tradeoff

The [element-choice parent](../00-element-choice.md) named four claims (Tet4 default, Tet10 upgrade, Hex8 rejected, mixed deferred) and pointed at this leaf for "the numbers behind the commitment". This leaf lays out the comparison framework — per-element cost, per-DOF accuracy, conditioning, contact-compatibility — and identifies which axes are decisive at which Phase. Specific benchmark numbers against the [MuJoCo flex regression baseline](../../../120-roadmap/01-track-1b.md) are a Phase D measurement deliverable; this leaf names the comparison axes Phase D will populate, and identifies the qualitative tradeoffs already known from the FEM literature that shape the commitment.

## The comparison axes

Five axes that matter for the canonical-problem soft-body workload:

| Axis | What it measures | Decisive at |
|---|---|---|
| Per-element FLOPs | Work per Newton residual + tangent eval | Hot-path cost — dominates when N elements is large |
| Per-DOF energy-error | Strain-energy error vs reference solution at fixed DOF count | Phase D vs Phase H fidelity gates |
| Sparse-system conditioning | Newton-iteration count to convergence; preconditioner cost | Solver wall-clock dominance regime |
| Contact-band fidelity | Rim-deformation accuracy under IPC-driven loading | The [Phase H qualitative target](../../../10-physical/02-what-goes-wrong/04-rim.md) |
| SDF-mesh compatibility | Robustness of upstream meshing from arbitrary SDF | Workflow-fit, not simulator-fit |

The axes are not independent. Per-element FLOPs and per-DOF accuracy together determine wall-clock-to-target-fidelity; per-element FLOPs alone is misleading. Contact-band fidelity at fixed wall-clock is the figure of merit Phase H targets, not per-element cost in isolation.

## Per-element FLOPs — qualitative ratios

From the [Tet4 cost section](00-tet4.md) and the [Tet10 cost section](01-tet10.md), the per-element work decomposes:

| Element | Constitutive calls/elem | Stiffness blocks | Per-elem FLOPs (relative) |
|---|---|---|---|
| Tet4 | 1 | 16 | 1.0 |
| Tet10 (4-pt Gauss) | 4 | 100 | 3–5× |
| Hex8 (8-pt Gauss, full) | 8 | 64 | 4–6× |
| Hex8 (1-pt Gauss, reduced) | 1 | 64 | Comparable to Tet4 on call-count; assembly arithmetic 4× higher (more blocks); plus hourglass-control overhead |

The Tet10 ratio range comes from the Tet10 sub-leaf's three breakdowns (call count, assembly arithmetic, sparse-Hessian nonzero count) composing differently per workload. Hex8's full-integration ratio is exposed for comparison even though Hex8 is not shipped — the reduced-integration column makes clear why production codes use it where they can absorb the hourglass-control complexity.

These are per-element ratios. Wall-clock comparison requires multiplying by element-count-to-target-fidelity, which the next axis governs.

## Per-DOF energy-error — qualitative ranking

For the canonical-problem workload (compliant cavity-probe conformity at moderate stretch, contact-band concentration, $\nu \approx 0.49$ silicone), the FEM literature places the per-DOF error ranking at fixed mesh resolution as:

$$ \text{Hex8 (full)} \;\lesssim\; \text{Tet10} \;\ll\; \text{Tet4} $$

Tet4's constant-strain assumption inflates per-DOF error in the contact band — the same behavior the [parent's Claim 1 rim-deformation argument](../00-element-choice.md) names. Tet10's linear-strain admits the missing modes; Hex8 (full integration) is comparable to Tet10 on aligned-grid problems and worse on curved-domain problems. The wall-clock-to-target-fidelity ranking inverts the per-element-cost ranking on the canonical problem because Tet4 needs several-fold more elements to match Tet10's contact-band accuracy, and the per-element ratio reverses once that element-count multiplier is applied. The crossover ratio is workload-dependent (specific values populate at Phase D against the [MuJoCo flex baseline](../../../00-context/02-sota/08-mujoco-flex.md)) but the qualitative reversal is what motivates Phase H's Tet10-in-band commitment.

The Phase D regression suite ([Part 11 Ch 04 — regression sub-leaf](../../../110-crate/04-testing/01-regression.md)) measures the per-DOF energy-error against [MuJoCo flex's Tet4 output](../../../00-context/02-sota/08-mujoco-flex.md) on a fixed test geometry; the Phase H regression suite extends this with Tet10 baselines.

## Conditioning and Newton-iteration count

Higher-order elements (Tet10, Hex8 full-integration) produce stiffer-conditioned linear systems per Newton step, which can inflate the iteration count of the [Part 8 Ch 02 sparse solver](../../../80-gpu/02-sparse-solvers.md). Whether this matters in wall-clock terms depends on the preconditioner: a strong preconditioner (algebraic multigrid, [AMG sub-leaf](../../../80-gpu/02-sparse-solvers/02-amg.md)) recovers most of the conditioning penalty; a weak preconditioner (Jacobi, simple block-diagonal) does not. `sim-soft`'s solver picks ([Part 8 Ch 02 — preconditioning sub-leaf](../../../80-gpu/02-sparse-solvers/03-preconditioning.md)) determine how exposed Tet10 is to its own conditioning cost; the per-element FLOPs above are an underestimate when this exposure is high.

## Contact-band fidelity — the rim test

The [Part 1 Ch 02 rim-deformation failure](../../../10-physical/02-what-goes-wrong/04-rim.md) is the canonical-problem failure mode that Tet4-only Phase D ships with and Phase H must remove. Quantitative criteria the regression suite measures: peak-pressure error against analytic Hertzian reference at coarse-mesh and fine-mesh resolutions, contact-area error, surface-displacement RMS error in the band. These figures populate at Phase D against the [MuJoCo flex baseline](../../../00-context/02-sota/08-mujoco-flex.md) and extend at Phase H once Tet10-in-band is shipped. The leaf does not pre-state values; the [milestone — first working sim-soft](../../../120-roadmap/02-first-working.md) gates the data-collection that makes them measurable.

## SDF-mesh compatibility — the workflow-fit axis

The [Hex8 sub-leaf's upstream-meshing argument](02-hex8.md) is the concrete instance of this axis: Hex8 fails not because of per-element or per-DOF properties but because robust SDF → hex meshing does not exist. Tet4 and Tet10 share the upstream pipeline ([fTetWild](../../../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md) for both — Tet10 nodes are added as a midpoint-insertion post-pass on the Tet4 mesh), so the workflow-fit axis ranks them identically. Tet10's per-element extra cost is the only price for the fidelity gain in the contact band; the upstream meshing carries no penalty.

## What this sub-leaf commits the book to

- **The cost/accuracy comparison is multi-axis, not single-number.** No single metric ("FLOPs per element", "DOFs per element") justifies the element commitments — wall-clock-to-target-fidelity-on-canonical-problem is the figure of merit.
- **Per-element FLOPs ratios are a 3–5× Tet10 / 1.0 Tet4 spread on the canonical-problem regime.** The 4–6× Hex8-full ratio and 1.5–2× Hex8-reduced ratio are documented for completeness, not for selection.
- **Per-DOF energy-error ranking inverts the cost ranking on the canonical problem.** Tet10's 3–5× per-element penalty is overcome by the several-fold-fewer-elements-needed to match contact-band fidelity; this is the Phase H Tet10-in-band justification. The exact crossover ratio is a Phase D measurement deliverable.
- **Specific benchmark numbers are Phase D regression-suite output.** This leaf is the comparison framework; the [Part 11 Ch 04 regression-test sub-leaf](../../../110-crate/04-testing/01-regression.md) populates it with measurements against MuJoCo flex.
