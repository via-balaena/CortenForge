# Co-simulation papers

References for the partitioned / fixed-point co-simulation scheme used across the `sim-soft` ↔ `sim-core` interface in [Part 5 Ch 03](../../50-time-integration/03-coupling.md). The cluster spans three generations: foundational partitioned analysis from the 1980s, the modern multi-timestep heterogeneous-solver references from the 2000s, and the contemporary survey consolidating fixed-point and quasi-Newton coupling for encapsulated solvers.

One constraint-coupled rigid–deformable FEM reference (FEBio / Maas et al. 2012) is also anchored here, cited inline from the spine's "academic analogue" framing; FEBio's augmented-Lagrangian enforcement stands in for the closest-academic-analogue approaches, even though it is not the single-Hessian monolithic assembly the spine rejects.

## Felippa and Park 1980 {#felippa-park-1980}

*Staggered Transient Analysis Procedures for Coupled Mechanical Systems: Formulation.* Computer Methods in Applied Mechanics and Engineering 24(1), pp. 61–111, 1980. Authors: Carlos A. Felippa (Applied Mechanics Laboratory, Lockheed Missiles and Space Co., Palo Alto, CA), K. C. Park (University of Colorado). DOI [10.1016/0045-7825(80)90040-7](https://doi.org/10.1016/0045-7825(80)90040-7).

The foundational partitioned-analysis paper. Introduces the staggered time-stepping framework in which a coupled multi-physics system is decomposed into subsystems, each advanced by its own analyser on its own step, with interface variables exchanged between advances. Formulates the staggered scheme with temporal extrapolation of coupling terms rather than fixed-point iteration — the 1980 paper is about the partitioned architecture itself (each subsystem treated by an independent code) rather than iteration-to-convergence. Cited inline from [Part 5 Ch 03 spine](../../50-time-integration/03-coupling.md) as the original anchor for the partitioned paradigm, and from [§01 fixed-point](../../50-time-integration/03-coupling/01-fixed-point.md) as the textbook precedent for holding the coupling payload constant across the subordinate simulator's steps (the zeroth-order case of the staggered-extrapolation family this paper formulates).

## Gravouil and Combescure 2001 {#gravouil-combescure-2001}

*Multi-Time-Step Explicit-Implicit Method for Non-Linear Structural Dynamics.* International Journal for Numerical Methods in Engineering 50(1), pp. 199–225, January 2001. Authors: Anthony Gravouil, Alain Combescure (both LaMCoS, INSA Lyon). DOI [10.1002/1097-0207(20010110)50:1<199::AID-NME132>3.0.CO;2-A](https://doi.org/10.1002/1097-0207(20010110)50:1<199::AID-NME132>3.0.CO;2-A). Also archived as [HAL hal-00455852](https://hal.science/hal-00455852).

The canonical reference for **partitioned coupling of subdomains with differing sub-domain timesteps**. Extends Newmark-family integrators to heterogeneous explicit-implicit pairs across a domain decomposition, enforcing velocity continuity at the interface via a dual Schur / Lagrange-multiplier formulation. Proves stability for the coupled scheme with material and geometric nonlinearities. This is the closest structural-dynamics analogue of `sim-soft` ↔ `sim-core`'s differing-timestep handshake. Cited inline from [§01 fixed-point](../../50-time-integration/03-coupling/01-fixed-point.md) — the fixed-point scheme's stability for differing sub-domain steps traces to this paper's framework.

## Kübler and Schiehlen 2000 {#kubler-schiehlen-2000}

*Two Methods of Simulator Coupling.* Mathematical and Computer Modelling of Dynamical Systems 6(2), pp. 93–113, June 2000. Authors: Ralf Kübler, Werner Schiehlen (both Institute for Engineering and Computational Mechanics, University of Stuttgart). DOI [10.1076/1387-3954(200006)6:2;1-M;FT093](https://doi.org/10.1076/1387-3954(200006)6:2;1-M;FT093).

A canonical multi-body-dynamics anchor for **simulator-coupling stability analysis**. Identifies when naive fixed-point iteration between coupled simulators converges and when it fails, and proposes two methods to ensure stability — algebraic-loop elimination and system reformulation at the module interface — as decoupling strategies for modular simulation. Cited inline from [§01 fixed-point](../../50-time-integration/03-coupling/01-fixed-point.md) when the fixed-point iterate is introduced — the Gauss-Seidel-type iteration between `sim-soft` and `sim-core` is well-posed precisely when this paper's convergence conditions hold (absence of implicit algebraic loops through the coupling).

## Gomes, Thule, Broman, Larsen, Vangheluwe 2018 {#gomes-2018}

*Co-Simulation: A Survey.* ACM Computing Surveys 51(3), Article 49, June 2018. Authors: Cláudio Gomes (Aarhus University), Casper Thule (Aarhus University), David Broman (KTH Royal Institute of Technology), Peter Gorm Larsen (Aarhus University), Hans Vangheluwe (University of Antwerp / McGill University). DOI [10.1145/3179993](https://doi.org/10.1145/3179993). arXiv preprint [1702.00686](https://arxiv.org/abs/1702.00686) (2017).

The modern comprehensive survey of co-simulation approaches. Covers master-algorithm architectures, communication-step-size control, fixed-point and interface-quasi-Newton (IQN-ILS and relatives) iteration schemes, and the FMI (Functional Mock-up Interface) standard for encapsulated solver coupling. Provides the contemporary taxonomy used when classifying the `sim-soft` ↔ `sim-core` scheme as outer-loop Gauss-Seidel fixed-point rather than IQN-accelerated or extrapolation-based. Cited inline from [§01 fixed-point](../../50-time-integration/03-coupling/01-fixed-point.md) in the "iteration count in practice" and "alternatives rejected (quasi-Newton acceleration)" discussions.

## Maas et al. 2012 — FEBio {#febio-2012}

*FEBio: Finite Elements for Biomechanics.* Journal of Biomechanical Engineering 134(1), Article 011005, January 2012. Authors: Steve A. Maas, Benjamin J. Ellis, Gerard A. Ateshian, Jeffrey A. Weiss. DOI [10.1115/1.4005694](https://doi.org/10.1115/1.4005694). Open-access mirror at [febio.org](https://febio.org/site/uploads/maas_jbme_2012.pdf).

The canonical FEBio paper. FEBio is a biomechanics-focused finite-element code with rigid-body support via **augmented-Lagrangian constraint coupling** — rigid bodies are constraint-coupled to deformable surfaces through Lagrange-multiplier iterations rather than assembled into a single monolithic stiffness matrix; the FEBio theory manual's section on the Augmented Lagrangian Method (§7.1.6 in the current theory-manual edition) describes the per-constraint iterative enforcement to user-specified tolerance. Cited inline from [Part 5 Ch 03 spine](../../50-time-integration/03-coupling.md) as the closest academic analogue to the rejected monolithic scheme: FEBio demonstrates that a unified rigid-deformable formulation is achievable, but via iterative constraint enforcement rather than single-Hessian assembly. The spine's point — that `sim-soft` rejects the monolithic path on architectural grounds — stands regardless of whether the closest analogue is single-Hessian or constraint-coupled.

## Pass 3 anchors (not yet inline-cited)

Reserved slots for works a Pass 3 bibliography expansion may populate: Farhat, Lesoinne, and Le Tallec 1998 on load and motion transfer for fluid-structure interaction (canonical reference for aero-structural partitioned coupling; venue and bibliographic details to be verified when cited inline); and any more recent IQN-ILS follow-ups (Degroote et al. 2010 and related) the study references by anchor when Phase F's measured iterate distribution lands.
