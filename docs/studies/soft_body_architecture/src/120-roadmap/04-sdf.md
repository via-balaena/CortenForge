# Milestone — SDF pipeline

At the end of Phase G, the designer edits an SDF in `cf-design` and `sim-soft` responds. Geometry changes trigger a re-mesh, the solver warm-starts from the previous mesh's state, and the reward updates within the inner-loop budget of [Part 10 Ch 06](../100-optimization/06-full-loop.md). This is the milestone that turns `sim-soft` from a simulator driven by a hardcoded scene into a live design tool.

Phase G closes three commitments: [Part 7 Ch 00](../70-sdf-pipeline/00-sdf-primitive.md)'s SDF-as-universal-design-primitive, [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md)'s live re-meshing, and the [`cf-design` coupling](../110-crate/02-coupling/04-cf-design.md) from Part 11. The `sdf_bridge/` module lands here; before Phase G it does not exist.

## Deliverables

- **SDF → tet pipeline.** An `SdfField` trait object ingested from `cf-design` produces a tet mesh suitable for the Phase D solver, via the strategy chosen in [Part 7 Ch 01](../70-sdf-pipeline/01-tet-strategies.md). Tet mesh quality passes the aspect-ratio and dihedral-angle bounds from [Part 3 Ch 01](../30-discretization/01-mesh-quality.md).
- **`EditResult` classifier.** The change-detection machinery from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md) labels each edit `ParameterOnly`, `MaterialChanging`, or `TopologyChanging`. The classifier's accuracy is tested against a suite of synthetic edits with known ground-truth labels; misclassification rate stays under 1%.
- **Warm-started state transfer.** A `ParameterOnly` edit skips re-meshing entirely; a `MaterialChanging` edit re-factors the sparse system without re-meshing; a `TopologyChanging` edit re-meshes and transfers the deformation-state field by per-vertex interpolation from the old mesh onto the new, per the [state-transfer sub-chapter](../70-sdf-pipeline/04-live-remesh/02-state-transfer.md).
- **`cf-design` coupling live.** An authoring-side geometry edit in `cf-design` propagates to a `sim-soft` re-evaluation without manual intervention. The boundary carries `SdfField` and `MaterialField` trait objects, not a file-format payload, per the [cf-design coupling](../110-crate/02-coupling/04-cf-design.md).
- **Per-`EditClass` wall-time budget instantiated.** The three-class cost table from [Part 10 Ch 00](../100-optimization/00-forward.md) is populated with measured numbers from the Phase G implementation. Numbers landing outside the budgeted bands trigger a redesign before Phase H opens.

## What Phase G does not yet include

- **Differentiability through the mesh topology change.** A `TopologyChanging` edit raises `GradientEstimate::Noisy { variance }` per [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md). The smooth-gradient contract from Phase D holds for `ParameterOnly` and `MaterialChanging` edits; the noisy-gradient regime is explicit and flagged. Closing the topology-differentiation gap is a [Ch 07](07-open-questions.md) item, not a Phase G goal.
- **Fidelity upgrades from Phase H.** Tet10, mixed u-p, Prony viscoelasticity, HGO anisotropy, and adaptive refinement all land *after* Phase G. Phase G's re-meshing pipeline is Tet4-only and carries the known [Tet4 rim-deformation error](../10-physical/02-what-goes-wrong/04-rim.md) forward. Phase H's upgrades slot in without changing the SDF bridge's contract.
- **Visual layer.** Design-edit feedback at Phase G is still a flat-shaded render through the diagnostic pipeline. The visual ceiling from [Part 9](../90-visual/00-sss.md) does not arrive until Phase I.

## Exit criteria

Phase G closes when:

1. The canonical design-mode loop from [Part 10 Ch 06](../100-optimization/06-full-loop.md) closes against a *real* `cf-design` authoring session, not a scripted parameter sweep. The designer changes a geometric parameter in `cf-design`, and `sim-soft` re-evaluates within the per-`EditClass` budget.
2. The three-class `EditResult` classifier passes its accuracy regression suite (misclassification under 1%).
3. The [`sim-ml-chassis` coupling](../110-crate/02-coupling/03-ml-chassis.md) remains closed — the γ-locked `RewardBreakdown` and its scalar-composed gradient are consumable by the optimizer regardless of which `EditClass` produced them. Noisy gradients are explicit in the API, not silently substituted for exact ones.

Phase G is the first phase where the platform's headline user experience — *design in `cf-design`, evaluate in `sim-soft`, repeat* — actually exists. Phases A–F build the substrate; Phase G is where the substrate becomes a workflow.
