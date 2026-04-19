# Volume consistency

The [mesh-quality parent](../01-mesh-quality.md) named volume consistency as a hard-failure gate — inverted tets (negative signed volume) fail mesh ingest outright with a `MeshIngestError::InvertedElements(Vec<TetId>)`. This leaf writes down what signed volume means for a tet, why even a small-positive minimum threshold (rather than just "non-negative") matters, what failure mode an inverted tet would induce in the solver if accepted, and how the gate composes with the [aspect-ratio](00-aspect-ratio.md) and [dihedral-angle](01-dihedral.md) gates.

## Signed volume and tet orientation

A tet with vertices $X_1, X_2, X_3, X_4$ in the reference configuration has signed volume:

$$ V^e = \tfrac{1}{6}\,\det\!\begin{pmatrix} X_2 - X_1 \\ X_3 - X_1 \\ X_4 - X_1 \end{pmatrix} $$

where the determinant is of the $3 \times 3$ matrix whose rows are the edge vectors from vertex 1. If the four vertices are listed in a right-handed (positive-orientation) order, $V^e > 0$; if the orientation is left-handed (the vertex ordering puts vertex 4 on the wrong side of the face $(X_1, X_2, X_3)$), $V^e < 0$. The convention `sim-soft` adopts is that all tet vertex tuples in the [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) module store right-handed orientation; positive $V^e$ is the only acceptable sign.

A tet with $V^e = 0$ exactly is geometrically degenerate (four coplanar vertices); a tet with $V^e < 0$ is geometrically inverted (vertices in left-handed order). Both are rejected at ingest, but for different reasons — the degenerate-zero case is a numerical-evaluation pathology (every shape-function gradient becomes ill-defined), the inverted-negative case is a mesh-construction error.

## The small-positive threshold

`sim-soft`'s ingest gate rejects not only $V^e \leq 0$ but also $V^e < V_\text{min}$ for a small positive $V_\text{min}$ — a tet whose volume is positive but tiny is acceptable geometrically and unacceptable numerically. The threshold $V_\text{min}$ is mesh-scale-dependent: typically expressed as a fraction of the median tet volume, e.g., $V_\text{min} = 10^{-6} \times V_\text{median}$. Tets near the threshold:

- Have shape-function gradients $\nabla_X N_i$ inversely proportional to $V^e$, so as $V^e \to 0$ the gradients diverge and the [Tet4 stiffness assembly](../../00-element-choice/00-tet4.md) produces a per-element stiffness with diverging entries.
- Contribute disproportionately to the Newton tangent's largest eigenvalue, dominating the global condition number.
- Pass the [aspect-ratio gate](00-aspect-ratio.md) and [dihedral-angle gate](01-dihedral.md) when the tet is small-but-regular (e.g., a tiny regular tet next to large normal-sized tets — radius ratio is regular, dihedrals are regular, but the absolute volume is too small to integrate stably).

The volume gate is therefore not redundant with the shape-quality gates; it catches small-but-shapely tets that the shape gates pass.

## Why an inverted tet would corrupt the solver

If an inverted tet were silently accepted, the failure mode is not loud:

1. The shape-function gradient $\nabla_X N_i$ for the "wrong-side" vertex points the wrong direction.
2. The deformation gradient $F$ computed on this tet is not the physical $F$; it has the wrong handedness and $\det F$ comes out negative.
3. The constitutive law evaluates on this $F$. For neo-Hookean, $\ln J = \ln \det F$ is undefined for $J < 0$ — the impl either silently returns NaN, gracefully clamps to a fallback (silently wrong stress), or errors loudly. Different `Material` impls handle this differently, and the [`Material`](../../../20-materials/00-trait-hierarchy/00-trait-surface.md) trait does not require any specific handling of physically-impossible $F$ inputs.
4. NaN propagates through the Newton tangent and the Hessian assembly; the [Part 8 Ch 02 sparse solver](../../../80-gpu/02-sparse-solvers.md) silently fails to converge, the [adaptive-$\Delta t$ logic](../../../50-time-integration/02-adaptive-dt.md) shrinks the step indefinitely, and the simulation stalls. The downstream symptom is "the solver hung"; the upstream cause is the inverted tet that should have been rejected at ingest.

This is precisely the [Part 1 Ch 02 "what existing solvers get wrong"](../../../10-physical/02-what-goes-wrong.md) failure pattern that `sim-soft` is designed to make impossible by construction. Failing loudly at the ingest gate, with the specific tet IDs in the error, surfaces the bug at the layer that can fix it.

## Caller recovery options

`MeshIngestError::InvertedElements(Vec<TetId>)` returned to the caller has three handling paths:

- **Local repair** (when the inversion is bounded — a single edge-flip resolves the tet's orientation): `sim-soft`'s [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) attempts the flip in-place if the post-flip mesh would also pass the [aspect-ratio](00-aspect-ratio.md) and [dihedral-angle](01-dihedral.md) gates. Otherwise, the repair is rejected and the error propagates.
- **Re-mesh from the SDF** (when many tets are inverted, indicating an upstream meshing problem): the [Part 7 Ch 04 live-remesh path](../../../70-sdf-pipeline/04-live-remesh.md) re-runs [tet generation](../../../70-sdf-pipeline/01-tet-strategies.md) with adjusted parameters.
- **Fall back to coarser mesh** (when the SDF itself has a pathology in the failing region): the caller drops the offending region's resolution and accepts the lower-fidelity result.

The choice between the three is a Phase H caller-policy decision, not a per-tet decision; the gate's contract is "report the failing tets, do not silently accept them".

## What this sub-leaf commits the book to

- **Signed volume convention: $V^e > 0$ is the only accepted sign.** Right-handed vertex tuples are required at the [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) module level; the [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) reorders vertex tuples on ingest if needed to enforce the convention.
- **The volume gate is two-stage: $V^e > 0$ (orientation) and $V^e > V_\text{min}$ (numerical conditioning).** Both stages are required; orientation alone passes degenerate-tiny tets that destabilize the solver downstream.
- **`MeshIngestError::InvertedElements(Vec<TetId>)` is a hard ingest failure, never silently repaired.** Repair is a caller-driven retry loop, not an implicit ingest behavior.
- **Volume gate composes with shape gates, not redundant.** Aspect-ratio and dihedral-angle gates catch the bad-shape-but-finite-volume cases; the volume gate catches the small-or-inverted cases the shape gates pass.
