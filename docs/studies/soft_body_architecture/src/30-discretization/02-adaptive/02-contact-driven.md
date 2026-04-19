# Contact-driven refinement

The [adaptive parent](../02-adaptive.md) and [Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md) both rejected contact-driven refinement as `sim-soft`'s primary refinement trigger, in favor of stress-gradient. This leaf writes down what contact-driven refinement is, why it is the natural-seeming default a beginner would reach for, the two failure modes that disqualify it as the primary trigger, and the [debug-flag role](../../../110-crate/00-module-layout/03-contact.md) it plays in `sim-soft` despite the rejection.

## What contact-driven refinement is

Given an active set of IPC contact pairs ([Part 4 Ch 01 IPC internals](../../../40-contact/01-ipc-internals.md)), each pair links two mesh elements (a vertex and a face, an edge and an edge, etc.) at distance below the IPC barrier width. A contact-driven refinement scheme marks for refinement any tet that:

- Contains a vertex involved in an active contact pair, or
- Is incident to an edge or face involved in an active contact pair, or
- Has a centroid within a fixed multiple of the IPC barrier width of any contact-pair location.

The criterion is **proximity-based** — it depends on the geometric configuration of contacts, not on the stress or strain field. A tet is refined if it is *near* a contact, regardless of whether the deformation field inside that tet has structure that needs more resolution.

The implementation is straightforward: run the [Part 4 Ch 03 BVH proximity query](../../../40-contact/03-self-contact/00-bvh.md) (already required for IPC contact detection), enumerate the touched tets, mark them, refine. The marking is essentially free given the BVH already exists; the refinement operation reuses the [h-refinement red-green machinery from Ch 02 Sec 00](00-h-refinement.md).

## Why it is the seemingly-natural default

Three reasons a beginner might pick contact-driven as the primary trigger:

- **The rim-deformation failure is a contact-zone failure.** [Part 1 Ch 02's rim-deformation](../../../10-physical/02-what-goes-wrong/04-rim.md) symptom appears at the contact band, so refining there feels like targeting the right region.
- **Contact infrastructure is already there.** The IPC machinery's BVH and active-set queries are reusable — no extra signal needs to be computed; the refinement signal is "wherever IPC is currently working".
- **Production IPC implementations often use contact-proximity refinement.** It is the path of least resistance if the IPC layer is the locus of computation; many published codes ship it.

These reasons are not wrong, just incomplete. The two failure modes below are why `sim-soft` rejects it.

## Failure mode 1 — over-refines on glancing contact

A glancing contact (two surfaces approaching with low normal force, or a brief contact during a transition) produces an active contact pair without producing significant deformation or stress. Contact-driven refinement marks the surrounding tets for refinement; stress-driven refinement does not (the local stress gradient is small).

Why this matters for `sim-soft`'s workload: the canonical-problem episodes have many transient contacts during probe approach and retreat, plus a smaller number of sustained high-stress contacts during the conformity phase. Contact-driven refinement spends DOF budget on transient low-stress contacts that never benefit from the resolution; stress-driven refinement reserves the budget for the sustained high-stress contacts that do. On a fixed DOF budget, contact-driven produces worse fidelity at the actual targets.

## Failure mode 2 — under-refines internal stress concentrations

A stress concentration that is *not* at a contact pair — for example, a stress riser at a probe shoulder where the diameter steps up, or at a material interface where a stiffness gradient crosses a threshold — produces no IPC contact pairs. Contact-driven refinement does not mark these tets for refinement because the criterion is contact-proximity, and the stress concentration is internal. Stress-driven refinement does mark them, because the local stress gradient is large.

This failure is harder to diagnose than failure mode 1: the symptom is poor fidelity at a region with no obvious contact-mesh signature, and the user inspecting the IPC active set would see nothing at the failing region. The fix is to switch to stress-driven, not to invent a new contact-adjacent heuristic.

## Where contact-driven still has a role — debug-only

Despite the rejection as a primary trigger, contact-driven refinement is implementable cheaply (per the BVH-already-exists argument above) and is useful as a **debug flag** for diagnosing IPC behavior. Two use cases:

- **IPC barrier-width tuning.** When choosing a new IPC barrier width or a new $\hat d$ ([Part 4 Ch 01 — barrier sub-leaf](../../../40-contact/01-ipc-internals/00-barrier.md)), running contact-driven refinement temporarily highlights every region the IPC layer is currently treating as in-contact, surfacing barrier-width-too-large or barrier-width-too-small symptoms visually.
- **Mesh-quality regression at contact boundaries.** When a mesh has poor quality at exactly the contact-band boundary (e.g., from upstream-meshing parameters that under-resolve thin contact regions), the contact-driven flag exposes which tets are being asked to deform across the band, surfacing whether the issue is mesh quality or solver tolerance.

The flag is in [`contact/`](../../../110-crate/00-module-layout/03-contact.md), not in `mesh/` or `readout/` — it is a contact-layer diagnostic, not a refinement policy.

## What this sub-leaf commits the book to

- **Contact-driven refinement is rejected as `sim-soft`'s primary refinement trigger.** Stress-driven ([Part 7 Ch 03](../../../70-sdf-pipeline/03-adaptive-refine.md)) is the primary trigger for both [h-refinement](00-h-refinement.md) and (when dynamic p-refinement lands post-Phase-H) p-refinement.
- **The rejection rests on two failure modes — over-refine on glancing contact, under-refine on internal stress concentration.** Both are workload-relevant for the canonical problem and not theoretically resolvable within the contact-proximity criterion family.
- **Contact-driven refinement remains available as a `[contact/]` debug flag.** Used for IPC barrier-width tuning and mesh-quality diagnosis at contact boundaries; not a production refinement path.
