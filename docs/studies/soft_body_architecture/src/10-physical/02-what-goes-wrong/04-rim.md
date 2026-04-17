# Poor rim / edge deformation

The cavity's open rim is where the canonical problem's hardest geometric deformation lives — under probe insertion the rim flares outward, the interior inner surface conforms to the probe, and the boundary between the two is a high-curvature band of large strain and concentrated contact pressure. A simulator that represents this rim as polygonal facets, that produces pressure spikes at the mesh-induced corners, or that cannot capture the rim's curvature at all is producing a defect that specifically corrupts the rim-adjacent reward samples. The visual signature is a silicone glove that looks polygonal where it wraps the finger. The scientific signature is an element order too low for the local stress gradient. Same defect, two framings.

## What fails, and where it shows up

Two off-the-shelf patterns reproduce this failure on the [canonical problem](../00-canonical/00-formulation.md).

**Coarse Tet4 meshes without adaptive refinement.** Linear tetrahedra (4-node tets, Tet4) have linear shape functions — the displacement field is linear per element, so each element is bounded by four flat faces and deforms affinely. High-curvature deformation at the rim requires either many elements (fine mesh) or higher-order shape functions. At canonical-scale ~30k tets with uniform density, the rim is resolved by ~50–200 elements along its circumference, which visibly polygonalizes the rim curvature under contact. The visible facets are pressure-spike concentrators: contact hitting a flat face hits hardest at its centroid, producing a mesh-aligned peak that has no physical counterpart.

**Uniform refinement without contact awareness.** Refining the whole mesh to get the rim right wastes compute on bulk material that is not deforming significantly. Uniform edge-length halving in 3D inflates tet count by ≈8× (assembly and per-iteration solve cost scale with it), which pushes the solver past the Phase E frame budget without solving the rim-specific problem.

## Which reward terms it corrupts

- **Peak pressure.** Mesh-aligned pressure spikes at the rim's polygonal faces can exceed the material's tensile-strength ceiling at configurations where the physical material does not. The [peak-pressure barrier](../01-reward/02-peak-bounds.md) fires on artefacts rather than on real-material failure risk.
- **Pressure uniformity near the rim.** The rim's pressure band, which under a smooth rim would be a continuous distribution, becomes a ring of per-element spikes and troughs; the [uniformity term](../01-reward/00-pressure-uniformity.md) reports high variance localized at the rim.
- **Coverage at the rim band.** A polygonal rim makes some facets contact the probe earlier and others later as the probe advances, producing an inconsistent coverage fraction at the rim-adjacent surface; the [coverage term](../01-reward/01-coverage.md) is biased at contact transitions.
- **Design-parameter sensitivity.** Small changes to the rim's reference geometry (rounding radius, wall-thickness profile) produce mesh-re-triangulation events, which are themselves a [differentiability failure](../../60-differentiability/05-diff-meshing.md) — but before that failure dominates, the reward is noisy within a single mesh because element-face positions shift under design perturbations.

## How sim-soft fixes it

Two commitments together.

- **[Adaptive h-refinement driven by contact proximity and stress gradient](../../30-discretization/02-adaptive.md).** The mesh is not uniform. The contact band at the rim is subdivided dynamically — elements split in regions where stress gradient exceeds a threshold or where the contact-gap field is within a small multiple of the element's characteristic length. The rim is resolved at fine spatial scale while the bulk stays coarse.
- **[Tet10 quadratic tetrahedra in the rim band (Phase H)](../../30-discretization/00-element-choice.md).** A 10-node tetrahedron has quadratic shape functions — each element can bend per-element rather than only along element boundaries, so the same vertex count can capture higher-curvature deformation than Tet4. [Part 3 Ch 00](../../30-discretization/00-element-choice.md) commits Tet10 to Phase H with mixed Tet4/Tet10 meshes deferred beyond that.

The two fixes are complementary, not redundant. Adaptive refinement addresses mesh-resolution poverty at the rim; Tet10 addresses shape-function poverty per element. Either alone leaves a residual — fine Tet4 has flat faces at high enough magnification, coarse Tet10 still discretizes the circumference into fewer elements than adaptive resolution would provide. Both together hit a threshold below which the rim's mesh artefacts are smaller than the reward terms' other sources of error.

## Why the obvious cheap fix doesn't work

**"Just refine the whole mesh to ~300k tets."** Uniform refinement at this scale pushes the solver past the interactive design-mode target and does not concentrate resolution where it is needed. The bulk of the cavity does not need the refinement; the rim does. Adaptive refinement is the targeted version of the uniform-refinement argument.

**"Skip Tet10; adaptive Tet4 is enough."** Adaptive h-refinement with Tet4 gets the rim to arbitrary resolution but at a vertex-count multiplier that grows quadratically in edge-length reduction. Tet10 gets a given curvature at lower vertex count, with a ≈2.5× nonzero-count penalty per element that is less severe than the refinement multiplier. Both together is the compound fix.

**"Render the mesh with subdivision surfaces so it looks smooth."** Post-hoc subdivision ([Part 9 Ch 03](../../90-visual/03-subdivision.md)) makes the rendered rim look smooth without changing the physics. The physical pressure-spike artefacts are still there in the simulation; the visual smoothing hides them in the render but not in the reward. This is fine for Phase I visual polish; it is not a fix for the physics problem.

**"Set a low peak-pressure threshold so the artefact barrier never fires."** Lowering the [peak-bound ceiling](../01-reward/02-peak-bounds.md) to silence the rim artefact also lowers it for genuine material-failure risk. The barrier loses the signal that it was supposed to catch. The mesh fix is the physics-correct answer.
