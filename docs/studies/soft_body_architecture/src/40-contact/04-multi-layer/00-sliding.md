# Layer-to-layer sliding

The [parent spine](../04-multi-layer.md) named two IPC-pair-based inter-layer conditions — frictional sliding and free-sliding — and noted that bonded inter-mesh reduces to [Part 3 Ch 03 mesh continuity](../../30-discretization/03-interfaces/00-bonded.md) rather than being an IPC-pair case. This sub-leaf writes down each of the two IPC cases, explains why bonded routes through the mesh layer, connects the two IPC cases to the existing [Part 3 Ch 03 `01-sliding.md` sliding-interface mesh machinery](../../30-discretization/03-interfaces/01-sliding.md) and the [Ch 02 IPC-friction machinery](../02-friction/02-ipc-friction.md), and names the per-pair metadata the scene uses to distinguish them.

## Why bonded is not Ch 04 scope

[Part 3 Ch 03 `00-bonded.md`](../../30-discretization/03-interfaces/00-bonded.md) established that a bonded interface between two material regions is **a single continuous mesh** — interface vertices are shared between both regions' tets, and displacement continuity is structural to the FEM basis. No inter-mesh contact pair exists between the regions; no IPC barrier term is needed. The canonical sleeve-on-mount configuration ([Part 1 Ch 00 formulation](../../10-physical/00-canonical/00-formulation.md)) is bonded in this sense: one mesh, per-tet material assignment.

If two regions enter the pipeline as separately meshed layers and the physics demands they be bonded, there are two paths:

- **Merge the interface vertices at ingest time.** The two meshes are fused into a single continuous mesh at the interface, and the configuration falls back to the Part 3 Ch 03 bonded case. This is a mesh-level operation in the [`sdf_bridge/`](../../110-crate/00-module-layout/08-sdf-bridge.md) ingestion path, not a solver-level one.
- **Hypothesize a zero-gap hard constraint at the interface.** IPC does not support this. The IPC barrier $b(d, \hat d) = -(d-\hat d)^2 \ln(d/\hat d)$ diverges as $d \to 0^+$ but never enforces $d = 0$ identically; a Lagrange-multiplier zero-gap constraint is a different formulation that `sim-soft` does not ship. "Bonded via IPC" is not a supported configuration.

Ch 04's inter-mesh scope is therefore the two IPC-pair cases: frictional sliding and free-sliding. Both share the same mesh-level and contact-level machinery, differing only in the value of the per-pair friction coefficient.

## Frictional sliding

A frictional sliding interface between two separately meshed layers A and B is the case [Part 3 Ch 03 `01-sliding.md`](../../30-discretization/03-interfaces/01-sliding.md) prepared the mesh layer for: each interface vertex appears twice (once on A's mesh, once on B's mesh), the interface-pair table in [`contact/`](../../110-crate/00-module-layout/03-contact.md) records the (A-vertex, B-vertex) correspondence, and the IPC layer detects inter-mesh contact pairs via the [Ch 03 BVH broadphase](../03-self-contact/00-bvh.md) operating across meshes.

At the IPC-pair level, each detected inter-mesh pair contributes the same two terms a standard IPC contact pair contributes:

- **Barrier term.** $\kappa\, b(d_k, \hat d)$ where $d_k$ is the gap between the A-side and B-side primitives. Same [barrier function](../01-ipc-internals/00-barrier.md), same [adaptive-$\kappa$ schedule](../01-ipc-internals/01-adaptive-kappa.md), same [CCD filter](../01-ipc-internals/02-ccd.md).
- **Smoothed-Coulomb friction term.** $\mu_c\, \lambda_k^n\, f_0(\|u_T^k(x)\|)$ per [Ch 02 IPC-friction](../02-friction/02-ipc-friction.md). Per-pair $\mu_c$ comes from the interface-pair metadata (below); $\lambda_k^n$ and $T_k^n$ are lagged values from the prior Newton iterate.

No new mechanism. The frictional sliding inter-layer case *is* the composition of barrier + friction the earlier chapters wrote down. The interface-pair table's role is to mark these expected-persistent inter-mesh pairs as such — inter-layer pairs at a sliding interface are active every timestep (the two surfaces are in persistent contact by design), so the [persistent-contact-pair caching](../05-real-time/02-caching.md) handles them as tracked pairs rather than rediscovering them as transient detections each frame.

## Free-sliding

Free-sliding is the $\mu_c = 0$ limit. The barrier term alone enforces non-penetration at the interface, and the smoothed-Coulomb term vanishes: $\mu_c$ multiplies the whole friction potential ($\mu_c\, \lambda_k^n\, f_0$), so $\mu_c = 0$ gives identically zero friction energy, gradient, and Hessian contribution. Layer A's surface exerts no tangential force on layer B's surface; the two slide past each other in the tangential direction while remaining separated in the normal direction.

The [Part 1 Ch 00 canonical formulation](../../10-physical/00-canonical/00-formulation.md)'s "lubricated sleeve" variant is the canonical free-sliding case — an idealized inter-layer interface where fluid-lubrication has dropped effective $\mu_c$ to approximately zero. At the solver level free-sliding is not a separate code path; it is the frictional sliding case with $\mu_c = 0$ in the interface-pair metadata. The smoothed-Coulomb term still evaluates (returning zero), and an implementation may detect the $\mu_c = 0$ case at assembly time to skip the friction term entirely — a performance detail, not a correctness choice.

## Per-pair metadata on `SoftScene`

The [Ch 02 Coulomb sub-leaf](../02-friction/00-coulomb.md) stated that $\mu_c$ is a property of the pair (pair-material × pair-material), not of either body alone, and forward-referenced `sim-soft`'s per-pair metadata on `SoftScene`. This sub-leaf closes that reference: the interface-pair table for a multi-layer configuration stores, per inter-mesh pair:

- A reference to the A-side primitive and the B-side primitive (via the double-layer vertex map from [Part 3 Ch 03 sliding](../../30-discretization/03-interfaces/01-sliding.md)).
- The per-pair $\mu_c$ (or $\mu_c = 0$ for free-sliding).
- An optional per-pair barrier tolerance $\hat d_k$ (relevant for thin-material configurations — see [sub-chapter 02](02-thin-material.md)).

The $\mu_c$-value authoring — which physical material pair gets which coefficient — is the [Part 7 Ch 02 material-assignment API](../../70-sdf-pipeline/02-material-assignment.md)'s responsibility. The interface-pair table is populated at scene-ingest time (when the double-layer vertices are generated at mesh construction) and persists for the simulation's duration; it does not change per timestep.

## What this sub-leaf commits the book to

- **Two IPC-pair-based inter-layer conditions, not three.** Bonded inter-mesh is routed through the mesh layer (merge interface vertices → [Part 3 Ch 03 bonded](../../30-discretization/03-interfaces/00-bonded.md)) and is not an IPC-pair case. `sim-soft` exposes frictional sliding and free-sliding as the two supported inter-layer pair types, distinguished solely by the per-pair $\mu_c$.
- **Free-sliding is a parameter setting, not a separate solver mode.** $\mu_c = 0$ in the [Ch 02 friction term](../02-friction/02-ipc-friction.md); no new code path, no "free-sliding contact type" enum variant. The smoothed-Coulomb term's $\mu_c$-factored form makes this free of ceremony.
- **Interface-pair metadata (per-pair $\mu_c$, optional per-pair $\hat d_k$) lives in [`contact/`](../../110-crate/00-module-layout/03-contact.md).** The interface-pair table is the single source of truth for inter-mesh pair parameters; the pair-generation path reads it at broadphase, and the energy-assembly path reads it at force computation. The [Part 7 material-assignment API](../../70-sdf-pipeline/02-material-assignment.md) is how authoring values reach this table.
- **No Ch 04-specific contact primitive.** The `ContactBody` trait surface and the [BSR-3 sparse Hessian assembly](../../80-gpu/01-sparse-matrix/01-bsr.md) do not distinguish inter-mesh pairs from intra-mesh self-contact pairs; both go through the same assembly path with the same per-pair energy, gradient, and Hessian.
