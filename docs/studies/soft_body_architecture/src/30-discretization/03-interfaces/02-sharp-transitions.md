# Sharp material transitions

The [interfaces parent](../03-interfaces.md) committed `sim-soft` to a specific position on graded-stiffness regions: **the SDF-weighted blending produces a continuous stiffness field with no topological discontinuity, so the solver does not special-case sharp transitions; the per-tet material assignment + adaptive h-refinement together handle the case smoothly.** This leaf writes down what that means in concrete data-flow terms — what the [`SdfField` material parameters](../../../70-sdf-pipeline/02-material-assignment.md) deliver across a transition, when the [interface-band flagging](../../../70-sdf-pipeline/02-material-assignment.md) triggers refinement, and what the failure mode is when the transition is so sharp that even the refined mesh can't represent it.

## What "sharp transition" means here

A sharp material transition is a region where the stiffness (or another material parameter — viscosity, anisotropy direction, thermal coupling) varies rapidly across a small spatial extent. Carbon-black-loaded skin over a softer silicone core is the canonical example: the stiffness ratio is 10–100× across a transition zone of perhaps tens of microns — small relative to the soft-body scale, but not zero. The SDF that defines the carbon-black region produces a continuous stiffness field via [Part 7 Ch 02's blending machinery](../../../70-sdf-pipeline/02-material-assignment.md): inside the high-modulus region $E = E_\text{stiff}$; outside the soft region $E = E_\text{soft}$; in the smooth-blend transition $E$ varies continuously between the two.

The transition is **not a topological discontinuity**: there is no surface across which the displacement field is allowed to jump, no double-layer of vertices, no contact-pair generation. From the [`element/`](../../../110-crate/00-module-layout/01-element.md) module's perspective, every tet in the transition zone is just an ordinary tet with a stiffness parameter sampled from the SDF.

## What the per-tet material assignment delivers

Per the [Part 7 Ch 02 sampling sub-leaf](../../../70-sdf-pipeline/02-material-assignment/00-sampling.md):

- **Tet4 path (centroid sampling).** A tet straddling the transition zone is sampled at its centroid, returning a single $E$ value. If the centroid falls inside the high-stiffness band, the whole tet uses $E_\text{stiff}$; if outside, the whole tet uses $E_\text{soft}$. The within-tet variation is *not* represented — the transition is a step function on the per-tet stiffness field, and the size of the step depends on whether neighboring tets fall on the same or opposite sides of the band.
- **Tet10 path (per-Gauss-point sampling).** A tet straddling the transition is sampled at each of its 4 Gauss points, returning up to 4 different $E$ values. The within-tet variation is represented as long as the 4 Gauss points span the transition; the integration weights smoothly blend the high-stiffness and soft-stiffness contributions to the per-element stiffness $K^e$.

The Tet10 path produces noticeably better behavior in the transition zone than the Tet4 path because it does not require the mesh to be fine enough that each tet is fully on one side of the transition. The Tet4 path requires the mesh to resolve the transition by element-size; the Tet10 path requires only that the transition not jump *between* Gauss points.

## When refinement triggers — the indicator

The [interface-band flagging from Part 7 Ch 02](../../../70-sdf-pipeline/02-material-assignment.md) identifies tets whose centroid sample is "near" the transition (within a configurable distance of the transition surface). For these flagged tets:

- The [Part 7 Ch 03 stress-gradient criterion](../../../70-sdf-pipeline/03-adaptive-refine.md) measures whether the per-tet stress is changing rapidly across the tet's neighbors. A flagged tet with a high stress-gradient across a transition-spanning neighbor pair is a refinement candidate.
- The [Tet4 → Tet10 promotion (manual band)](../../00-element-choice/03-mixed.md) is also a candidate response: if the user knows in advance that the transition zone is a workload concern, the band-mask can include the transition zone, and Tet10's per-Gauss-point sampling handles it without h-refinement.

The two responses are not exclusive — the flagged region can be h-refined and Tet10-promoted at the same time. The decision is at the [Part 7 Ch 03 policy layer](../../../70-sdf-pipeline/03-adaptive-refine.md), not at this layer.

## What goes wrong on too-sharp transitions

The smooth-blend treatment fails when the transition's spatial extent is **smaller than the post-refinement element size** — the SDF blending region is so narrow that even the refined mesh can't span it with multiple elements. In that case:

- Tet4 path: each tet either fully resolves the high-stiffness band or fully misses it; the per-tet stiffness field has step-function structure that may not match the analytical solution at the band boundary.
- Tet10 path: the 4 Gauss points fall on the same side of the transition for most tets; only tets very close to the transition see a within-tet $E$ variation, and even then the variation is at most a 2:2 split (2 Gauss points on each side).

The failure manifests as **stress smearing at the band boundary** — the per-element averaging of stiffness inside the transition tets produces a stress field that is artificially smooth where the analytical solution would have a sharper structure. The visual symptom is a "fuzzy" interface in the rendered output; the quantitative symptom is a peak-stress underestimate at the band boundary.

The fix is upstream: the [`SdfField` blend region width](../../../70-sdf-pipeline/02-material-assignment/01-composition.md) is increased to be larger than the post-refinement element size, accepting that the transition is now slightly more diffuse than the physical material would have. This is acceptable for the canonical-problem workload because the carbon-black skin's actual transition zone is itself diffuse (mixing during deposition, percolation effects); a slightly wider blend zone is closer to the physical reality than an artificially-sharp one would be.

If the workload requires a genuinely sharp transition (e.g., a sputter-deposited metallization layer), the smooth-blend treatment is unsuitable and either a [bonded-interface](00-bonded.md) treatment (where the SDF defines a sharp topological boundary that the mesh resolves by element edges, with the per-tet material assignment giving the discontinuity at the interface) or a [sliding-interface](01-sliding.md) treatment (with double-layer vertices and IPC contact) is the right answer. The choice is design-time, not runtime — the transition character is a property of the physical part and is reflected in the [`SdfField` composition](../../../70-sdf-pipeline/02-material-assignment/01-composition.md) the user authors.

## What this sub-leaf commits the book to

- **Sharp-transition handling is a refinement trigger, not a new solver path.** The transition is continuous-stiffness; the solver treats every tet identically; the [interface-band flagging](../../../70-sdf-pipeline/02-material-assignment.md) directs refinement and Tet10 promotion to the right tets without changing the [`element/`](../../../110-crate/00-module-layout/01-element.md) trait surface.
- **Tet10 + Gauss-point sampling resolves transitions better than Tet4 + centroid sampling.** Within-tet variation is represented in Tet10; the Tet4 path requires the mesh to be fine enough that each tet is on one side. This is one of the workload-specific reasons Phase H promotes the contact-band region to Tet10.
- **Genuinely-sharp (single-element-thin) transitions exceed the smooth-blend treatment.** The fix is upstream — widen the [`SdfField` blend region](../../../70-sdf-pipeline/02-material-assignment/01-composition.md) — or treat the transition as a [bonded](00-bonded.md) or [sliding](01-sliding.md) interface with explicit interface elements. The choice is design-time.
- **The failure mode is stress smearing at the transition, caught at the visualization and stress-readout layer.** The [`readout/`](../../../110-crate/00-module-layout/09-readout.md) module's per-tet stress output makes the smearing visible during regression-test review; the [Part 11 Ch 04 visual regression](../../../110-crate/04-testing/02-visual.md) gate flags it.
