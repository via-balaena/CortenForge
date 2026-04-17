# Related engineering problems

The [canonical formulation](00-formulation.md) — a compliant cavity closed over a rigid probe — is intentionally generic. The same geometric and loading abstraction maps onto a range of real engineered parts where soft elastomeric structure conforms to a rigid or semi-rigid mating surface. This leaf names the mapping explicitly so that a reader working on one of the specific domain problems below can read the rest of the book as speaking directly to their problem, not to an abstract academic toy.

## The abstraction map

Each domain problem specializes the canonical cavity–probe geometry, picks the load mode, and emphasizes a subset of the four [reward terms](../01-reward.md). None of the problems below needs a new subsystem the canonical formulation does not already carry.

| Domain problem | Cavity specialization | Probe specialization | Dominant reward terms |
|---|---|---|---|
| Mechanical seal | Thin-walled annular seal | Rigid shaft or rotating post | Uniformity, peak-pressure bound |
| Compliant gripper fingertip | Concave pad closed over held object | Rigid object being held | Coverage, uniformity |
| Catheter introducer sheath | Thin-walled long cylindrical sleeve | Rigid instrument (guidewire, delivery catheter) | Stiffness bound, peak-pressure bound |
| Soft-robotic suction cup | Shallow cup with compliant rim | Rigid surface being gripped | Coverage, uniformity |
| Pulsatile compression sleeve | Cylindrical wearable over a limb | Rigid limb proxy at canonical scale | Uniformity, stiffness bound |
| Wearable mask/seal (CPAP, diving mask) | Contoured rim following a face or limb profile | Rigid anatomical proxy | Coverage, uniformity |
| Ostomy-bag flange, prosthetic liner | Annular soft flange pressed to skin/stoma | Rigid skin proxy | Coverage, peak-pressure bound |

The book does not commit to any single one of these as the privileged application. Part 10's optimization loop treats the reward weights as a design variable per the [composition sub-chapter](../01-reward/04-composition.md); a domain user re-weights the four reward terms to match their application and keeps every other subsystem as-is.

## How the canonical problem covers the list

Three features of the canonical formulation carry each of the seven domain problems above without modification.

**The cavity is generic in geometry.** Cylindrical inner-surface topology covers seals, introducer sheaths, compression sleeves, prosthetic liners, and gripper fingertips. The [SDF-as-primitive](../../70-sdf-pipeline/00-sdf-primitive.md) commitment means that non-cylindrical contours (contoured mask rim, suction-cup cup, stoma flange) are swapped in by replacing the `SdfField` and re-meshing through the same pipeline; the solver and the reward evaluation are unchanged.

**The probe is generic in its rigid-contact partner role.** Whether the rigid mating surface is a rotating shaft, a held object, an anatomical proxy, or a flat skin surface, the contact law from the cavity's side is the same IPC-friction pair. The probe's kinematics — rotating, static, pulsed, held under force — enter through [Part 5 Ch 03's rigid-body coupling](../../50-time-integration/03-coupling.md), not through the cavity's constitutive law.

**The reward terms are reweighted, not replaced.** A mechanical seal weighs peak-pressure bound highly (the seal damages the shaft if local pressure exceeds a limit); a gripper weighs coverage highly (the object slips if coverage is low); a compression sleeve weighs stiffness bound highly (the sleeve does nothing if transmitted force is too low). All four terms are already in the composition; a domain user sets $w_i$ without extending the framework.

## What a domain user changes, what they keep

The canonical problem is the default; a domain user replaces the default values, not the framework.

**A domain user changes:**

- The `SdfField` describing the cavity geometry. Contoured mask rims, shallow suction-cup cups, annular flanges, fingertip pads — any shape expressible as an SDF, or boolean-composed from primitives per [`cf-design`'s composition rules](../../70-sdf-pipeline/00-sdf-primitive/01-operations.md).
- The `MaterialField`'s stiffness and anisotropy pattern. Graded-durometer seals ([Part 2 Ch 09](../../20-materials/09-spatial-fields.md)), fibre-reinforced liners ([Part 2 Ch 06](../../20-materials/06-anisotropic.md)).
- The reward weights $w_i$ and the peak-pressure / minimum-stiffness thresholds in the corresponding barrier terms.
- The probe's geometry and motion pattern, via a different `ContactBody` entry in `SoftScene`.

**A domain user keeps:**

- The constitutive law ([Part 2 Ch 04 hyperelastic](../../20-materials/04-hyperelastic.md)), the contact solver ([Part 4 IPC](../../40-contact/00-why-ipc.md)), the time integrator ([Part 5 Ch 00 backward Euler](../../50-time-integration/00-backward-euler.md)), and the differentiable path through the steady state ([Part 6](../../60-differentiability/02-implicit-function.md)).
- The reward-term machinery — pressure uniformity, coverage, peak-bound barrier, stiffness-bound barrier — and the composition rule ([Part 1 Ch 01](../01-reward.md)).
- The optimization loop ([Part 10](../../100-optimization/00-forward.md)) and the sim-to-real correction ([Part 10 Ch 05](../../100-optimization/05-sim-to-real.md)).

This separation is what the [thesis commitment](../03-thesis.md) — one integrated stack, not two parallel pipelines — pays off: a gripper engineer and a catheter engineer share a simulator, a reward framework, and an optimization loop, and differ only in an `SdfField`, a `MaterialField`, and four reward weights.

## Problems the canonical formulation does not cover

The canonical geometry is soft-shell-over-rigid-core. Problems outside that shape are not covered by the canonical formulation as a drop-in specialization; they may still use `sim-soft`'s subsystems, but they exercise different rewards and sometimes different subsystems:

- **Soft–soft contact between two independently-deformable bodies** (a soft gripper closing on a soft object, two elastomeric parts mating). Covered by `sim-soft`'s contact solver — both bodies are tet-meshed and [IPC](../../40-contact/00-why-ipc.md) handles the mutual pair-detection — but the reward structure is different because there is no "intended contact surface" defined by one rigid body's exterior. A reader in this regime reweights the reward's coverage and peak-pressure terms to a mutually-defined surface, which is outside the canonical reward's definition.
- **Fluid-coupled soft bodies** (a soft pump chamber with internal fluid, blood vessels, hydraulic-actuated soft robots). Outside the book's scope — `sim-soft` does not ship a fluid solver, and fluid coupling is not on the Phase A–I roadmap.
- **Large-deformation bending-dominated problems** (a long cantilevered soft tongue, a high-aspect-ratio tentacle). These are not defect problems of the canonical formulation, but they do not exercise the conformity reward — coverage and uniformity are not the right objectives for a reaching tentacle. A reader interested in tentacles reads the solver half of the book (Parts 2, 5, 6, 8) and writes a different reward.

The [why-canonical sibling](02-why-canonical.md) walks through the subsystem exercise coverage in more detail; this leaf scopes the application coverage.
