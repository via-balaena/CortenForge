# IPC papers

Incremental Potential Contact (IPC) is the contact formulation [Part 4](../../40-contact/00-why-ipc.md) commits to. This leaf indexes the 2020 source paper and the follow-up work that makes IPC tractable at real-time rates.

Pass 1 populates only anchors referenced inline by the book. Pass 3 will expand each entry to full bibliographic form (venue, DOI, extracted-result summary) and add adjacent references the book does not currently inline-cite.

## Li et al. 2020 {#li-2020}

*Incremental Potential Contact: Intersection- and Inversion-free, Large-Deformation Dynamics.* ACM Transactions on Graphics (SIGGRAPH 2020).

The source paper for IPC. Introduces the smooth barrier $b(d) = -(d - \hat d)^2 \ln(d / \hat d)$ for gap $d$ less than tolerance $\hat d$, the adaptive barrier-stiffness schedule, the filtered line search that preserves the intersection-free invariant across Newton iterations, and the smoothed Coulomb friction term. Cited inline from [Part 6 Ch 01 (custom VJPs)](../../60-differentiability/01-custom-vjps.md) for the numerical-stabilization result the hand-stabilized barrier VJP inherits: the naive chain rule through $\ln$ produces divergent pieces that cancel to a finite gradient, and the rearranged derivation avoids the cancellation. Also referenced in prose from [Part 4 Ch 00](../../40-contact/00-why-ipc.md), [Part 4 Ch 05](../../40-contact/05-real-time.md), and [Part 1 Ch 03 (thesis)](../../10-physical/03-thesis.md) as "the 2020 offline paper" the real-time commitment extends.

## IPC Toolkit {#ipc-toolkit}

*IPC Toolkit* — open-source C++ library (with Python bindings) implementing the Incremental Potential Contact formulation. Maintained by the IPC research community (Ferguson and collaborators) and released on GitHub as `ipc-sim/ipc-toolkit`. Ships the barrier function, adaptive-$\kappa$ schedule, CCD-filtered line search, and primitive-pair proximity detection as reusable components. Serves as the reference implementation of Li et al. 2020 and subsequent IPC follow-ups; tracks the literature across releases (the original 2020 conservative step-capping was replaced by tighter inclusion-based CCD in later releases). Cited inline from [Part 4 Ch 01 adaptive-kappa](../../40-contact/01-ipc-internals/01-adaptive-kappa.md) for the `initial_barrier_stiffness` helper whose computation `sim-soft` will port, and from [Part 4 Ch 01 CCD](../../40-contact/01-ipc-internals/02-ccd.md) for the CCD-algorithm-as-porting-target framing.

## Pass 3 anchors (not yet inline-cited)

Reserved slots for works the Pass 3 bibliography will likely populate: the 2022 adaptive-barrier-width follow-ups, GPU CCD parallelization papers, and self-contact BVH work. None is anchored in Pass 1 because no inline citation currently references them; adding them is Pass 3 judgment.
