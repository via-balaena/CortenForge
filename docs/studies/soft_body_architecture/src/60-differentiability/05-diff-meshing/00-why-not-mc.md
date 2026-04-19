# Why marching cubes isn't differentiable

The [parent chapter](../05-diff-meshing.md) names marching cubes as the canonical SDF-to-surface algorithm whose non-differentiability is the concrete obstruction `sim-soft` inherits. This sub-leaf writes out *which* step of the algorithm breaks differentiability, why the break is fundamental (not a precision or smoothing artifact), and why the same pathology afflicts every other meshing algorithm in the scientific-computing ecosystem.

## The function being differentiated

Treat a marching-cubes ingest as a map

$$
x_{\text{mesh}} : \theta_{\text{sdf}} \longmapsto (V, T)
$$

from SDF parameters $\theta_{\text{sdf}}$ to a mesh consisting of a vertex list $V \subset \mathbb{R}^3$ and a connectivity table $T$ (a list of tuples naming tets or triangles by vertex index). For the parent chapter's gradient $\partial L / \partial \theta_{\text{sdf}}$ to flow, autograd needs the map's Jacobian — how the mesh responds to an infinitesimal SDF-parameter perturbation.

Marching cubes computes the map in two stages. For each grid cell $c$:

- **Stage A — topology classification.** Evaluate the SDF at the eight corners of $c$, extract the sign pattern $s_c \in \{-, +\}^8$, and look up a canonical topology template $\tau(s_c)$ — one of [Lorensen & Cline 1987](https://dl.acm.org/doi/10.1145/37401.37422)'s 15 rotational equivalence classes covering all $2^8 = 256$ sign configurations. The template names how many vertices the cell contributes and which edges they sit on.
- **Stage B — vertex placement.** For each edge that $\tau(s_c)$ declares crossed, place a vertex at the zero-crossing location, found by linear interpolation of the two bracketing SDF values: if the edge runs between corners $a, b$ with SDF values $\phi_a, \phi_b$ of opposite sign, the vertex sits at $(1-t) a + t b$ where $t = \phi_a / (\phi_a - \phi_b)$.

The two stages have categorically different differentiability shapes.

## Stage B is smooth

Stage B's output — vertex positions on edges — is a smooth function of its inputs. Holding the topology $\tau(s_c)$ fixed, differentiating $t$ with respect to either $\phi_a$ or $\phi_b$ gives a closed-form expression, and the vertex position inherits that smoothness. A first-order perturbation of the SDF corner values produces a first-order movement of every existing vertex along its edge. One line of autograd wraps it.

## Stage A has a Dirac-delta derivative

Stage A's output — the topology template $\tau(s_c)$ — is a *discrete* function. The 15 templates partition the input space $\mathbb{R}^8$ (the SDF values at the eight corners) into open regions according to the sign pattern $s_c$, separated by the eight hyperplanes $\phi_i = 0$. Inside each region $\tau$ is constant; on the hyperplanes it jumps.

When an SDF parameter $\theta_{\text{sdf}}$ is swept and one corner value $\phi_i$ crosses zero, $s_c$ flips in one coordinate and $\tau$ jumps discontinuously. Mesh vertices appear or disappear, edges are added or removed, and the tet subdivision pattern changes. As a function of $\theta_{\text{sdf}}$, the mesh-level output has a *step discontinuity* at every parameter value where any corner crosses zero.

The derivative of a step is a Dirac delta at the crossing and zero elsewhere. The chain-rule product $\partial x_{\text{mesh}} / \partial \theta_{\text{sdf}} = (\partial x_{\text{mesh}} / \partial \tau)(\partial \tau / \partial \theta_{\text{sdf}})$ acquires a $\delta(\phi_i(\theta_{\text{sdf}}))$ factor that no smooth approximation recovers. Passing a reverse-mode adjoint through this step does not produce a numerically-finite gradient — it produces either zero (almost everywhere, because $\tau$ is locally constant) or a Dirac delta (at the crossing, which standard autograd tape cannot represent).

This is not a precision or discretization artifact. The function is *literally discontinuous*, and no amount of grid refinement, smoothing of the SDF, or higher-precision arithmetic changes that. The topology-classification step is combinatorial; its derivative is a distribution, not a function, and the scalar-tape autograd surface in [Ch 00 §00](../00-what-autograd-needs/00-generic.md) has no representation for it.

## The pathology generalizes to every ecosystem mesher

The same structural problem afflicts every other SDF-to-tet-mesh pipeline the book considers in [Part 7 Ch 01](../../70-sdf-pipeline/01-tet-strategies.md):

- **[Delaunay tetrahedralization](../../70-sdf-pipeline/01-tet-strategies/01-delaunay.md)** builds its mesh by incremental point insertion with edge flips that restore the empty-circumsphere property. Each flip is a discrete combinatorial choice — one face configuration or another, never a convex combination — and which flips happen depends on near-coplanarity predicates that are piecewise-constant in vertex coordinates. A smooth change in SDF parameters that moves a sample point across a coplanar configuration triggers a discontinuous tetra-topology jump.
- **[fTetWild](../../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md)** simplifies a conservative initial tet mesh by edge collapses, vertex smoothing, and face flips inside an $\epsilon$-envelope of the input surface. Every mesh-improvement operation is a discrete choice (collapse or don't, flip or don't) gated on predicates that are piecewise-constant in the current mesh geometry. The output mesh is a function of the input SDF, but the function is piecewise-constant over regions separated by the predicate-flip manifolds.

Meshing is a *combinatorial algorithm with a numerical front-end*. The numerical part — the SDF evaluations, the interpolations, the geometric predicates inside Stage B — is smooth. The combinatorial part — the topology-classification lookup, the edge-flip decisions, the collapse predicates — is piecewise-constant with jumps on codimension-1 manifolds. Derivatives across those manifolds are Dirac deltas. No implementation of any of these algorithms that the book has surveyed produces a smooth mesh-valued function of its SDF input.

## What this sub-leaf commits the book to

- **The non-differentiability is a structural property of the algorithm class**, not a fixable implementation detail. Marching cubes, Delaunay, and fTetWild all have the same topology-discontinuity pathology; replacing one with another does not recover a smooth map.
- **Stage B (vertex placement) is where the existing ecosystem meshers *are* differentiable**, and any closed-topology subinterval of an SDF-parameter sweep has a smooth gradient through vertex positions. The book's [topology-fixed-per-episode compromise](../05-diff-meshing.md) uses exactly this window.
- **The derivative at a topology change is a Dirac delta**, a distribution rather than a function. Research has taken three structural approaches: (i) *soft-relax* the topology choice into a continuous parameter so no genuine discontinuity remains ([§01 Deep MC](01-diff-mc.md) for the MC family; [§02 Rakotosaona / Son](02-diff-delaunay.md) for the Delaunay family; [§04 Option A](04-frontier.md) extends this line to a future volumetric DMTet); (ii) *keep* the topology discrete but amortize or localize it ([§01 Neural MC](01-diff-mc.md)'s binary-sign lookup; [§02 DMTet](02-diff-delaunay.md)'s rare-transition background tet grid); (iii) *integrate the Dirac analytically* as a boundary-integral contribution — demonstrated in differentiable rendering but unpublished for volumetric FEM ([§04 Option B](04-frontier.md)). The [FD wrapper of §03](03-fd-wrappers.md) is the fourth, unprincipled-but-usable alternative `sim-soft` commits to as the current fallback.
