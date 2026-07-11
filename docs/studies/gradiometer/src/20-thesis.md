# Why a gradiometer is the SDK's obvious application

[Chapter 1](10-feasibility.md) established that a room-temperature MEMS gradiometer is
physically viable for the large-anomaly niche, and that its cost lives in two places:
**matching** and **self-gravity**. This chapter makes the sharper claim:

> A high-end gradiometer is not a random application of CortenForge. It is the application
> where the SDK's entire thesis — differentiable mechanical co-design of geometry — is the
> *hard part of the instrument*. The device falls out of the primitives. That is what
> "obvious, in the engineering sense" means.

The argument is a one-to-one mapping. The three things that make a high-end gradiometer
brutally expensive everywhere else are exactly CortenForge's three core competencies.

## The three-problems / three-primitives map

### 1. Matched proof-mass pairs → matched-mass-from-CAD + differentiable geometry

A gradiometer's whole game is common-mode rejection, and [Chapter 1, Gate 3](10-feasibility.md#gate-3--what-actually-makes-it-hard-and-where-cortenforge-earns-its-keep)
showed that demands **sub-ppm channel matching** to run on any moving platform. Matching two
proof masses is not a calibration problem in the end — it is a *geometry* problem: two
structures whose mechanical response is identical by construction.

That is precisely what differentiable geometry over the `Sdf`/`Solid` kernel is for. The two
channels are one parametric geometry; the mismatch $\varepsilon$ between their responses is a
scalar objective; the co-design optimizer drives it toward zero. The competitor hand-trims for
weeks; here matching is a gradient.

### 2. Subtracting the instrument's own mass → self-gravity-from-CAD

[Chapter 1](10-feasibility.md) put the instrument's own self-gravity at **~1000 E** — a hundred
times the signal, and drifting as the structure thermally expands. It cannot be shielded, so it
must be computed and subtracted, which requires the instrument's full mass distribution.

CortenForge already integrates mass distributions from geometry:
`cf_design::mechanism::mass::mass_properties(solid, density, cell_size)` returns mass, centre of mass,
and the inertia tensor by integrating over an implicit field. The gradiometer's self-gravity
model is the *next derivative* on top of that — the field and its gradient at the proof masses,
from the same CAD geometry — and nulling its temperature sensitivity is a differentiable-geometry
optimization. No budget vendor computes self-gravity from CAD; CortenForge would do it natively.

### 3. Co-optimizing the whole structure → the differentiable co-design loop

Sensitivity, matching, self-gravity nulling, and the noise floor are not independent knobs; they
trade against each other across the same geometry. Getting a good instrument means optimizing all
of them at once — a co-design loop over a differentiable multiphysics model.

CortenForge already has that loop: `cf-codesign` exposes a `CoDesignProblem` trait and an
`optimize(problem, x0, cfg)` driver (the same machinery behind the body↔device co-design work).
The gradiometer is a new `CoDesignProblem` whose loss combines the four terms above. The loop is
not new; only the objective is.

## Why the gradiometer and not the gravimeter

A **gravimeter** measures $g$ at a point — a single proof mass on a flexure. Real, and a fit, but
a *single-element* design problem with modest surface for co-design.

A **gradiometer** measures the *difference* across a baseline — two or more matched elements whose
entire reason for existing is common-mode rejection. "Matched, symmetric, coupling-nulled
mechanical structure" is a geometry-plus-mechanics-plus-tolerance problem: *more* design surface,
and *more of it* is the kind differentiable co-design collapses. The gradiometer is where the
leverage is highest, so it is the target. (The gravimeter remains a simpler fallback if the
gradiometer's load-bearing gate fails.)

## The honest caveat this thesis rests on

This mapping is a *positioning* claim. It says the gradiometer's hard problems are shaped like
CortenForge's primitives — which is necessary, and true. It does **not** yet prove the leverage is
large enough to matter: that a co-designed match measurably beats a hand-matched one under real
FEM behavior and manufacturing tolerances. That proof is the program's first gate
([Chapter 4, Gate 1](40-program.md#gate-1--does-the-matching-leverage-actually-exist---the-load-bearing-decision)), and the whole thesis is conditional on it.

With the mapping established, [Chapter 3](30-primitives.md) inventories exactly what the SDK
already provides and what must be built.
