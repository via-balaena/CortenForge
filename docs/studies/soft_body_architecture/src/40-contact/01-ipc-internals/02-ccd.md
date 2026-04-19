# Continuous collision detection

Discrete snapshot collision tests — is pair $(i, j)$ currently intersecting? — are the standard in rigid-body engines and penalty-contact FEM: after each step, check whether any pair overlaps, and respond if so. IPC cannot use discrete snapshots because the barrier's non-penetration guarantee is a path property, not an endpoint property. A trajectory from $x_t$ to $x_{t+1}$ that starts and ends in a non-penetrating configuration can still pass through penetration in between, and a discrete snapshot at $x_{t+1}$ would miss the crossing. Continuous collision detection (CCD) — a query that asks *does any pair cross zero distance between positions $x$ and $x + \alpha \Delta x$?* — is the machinery that makes the non-penetration guarantee hold across the trajectory interior, not just at step endpoints. This sub-leaf describes the CCD role in the filtered line search and names the algorithm class IPC uses.

## The CCD query

Given two configurations $x_a$ (start of a step or a trial Newton step) and $x_b$ (end), and a linear interpolation $x(\alpha) = (1 - \alpha) x_a + \alpha x_b$ for $\alpha \in [0, 1]$, the CCD query returns the earliest $\alpha^\ast \in [0, 1]$ at which some primitive pair $(i, j)$ has zero distance:

$$ \alpha^\ast = \min_{(i,j)} \{ \alpha : d_{ij}(x(\alpha)) = 0 \} $$

If no pair crosses zero distance over $\alpha \in [0, 1]$, the query returns $\alpha^\ast = 1$ (unrestricted step). If some pair crosses, the query returns the $\alpha$ of the earliest crossing, and the line search clamps the step to $\alpha < \alpha^\ast$ by a safety factor. The per-pair geometry reduces to either a point-triangle CCD test (when the primitive types are vertex and triangle) or an edge-edge CCD test (when both primitives are edges); those are the only two cases for a surface mesh in 3D.

## Point-triangle and edge-edge cases

**Point-triangle CCD.** Given a point $p$ moving from $p_a$ to $p_b$ and a triangle with vertices $v_1, v_2, v_3$ moving from $v_{1a}, v_{2a}, v_{3a}$ to $v_{1b}, v_{2b}, v_{3b}$, find the earliest $\alpha$ at which $p$ lies in the plane of the triangle AND within the triangle's interior. The coplanarity condition is a cubic polynomial in $\alpha$ (the signed volume of the tetrahedron formed by $p$ and the three triangle vertices, which is a trilinear form becoming cubic under the $\alpha$ parameterization); the interior-check is a barycentric-coordinate constraint. Roots of the cubic give candidate $\alpha$ values; interior checks filter out crossings that happen outside the triangle's actual extent.

**Edge-edge CCD.** Given edges $(a_1, a_2)$ and $(b_1, b_2)$ each moving linearly, find the earliest $\alpha$ at which the four endpoints become coplanar AND the closest-point parameters on each edge land within the edges' extents. The coplanarity condition is again a cubic in $\alpha$; interior checks are analogous to the point-triangle case.

Both cases have closed-form cubic-root expressions, but root-finding over cubics is numerically delicate near double roots and very-near-zero roots — the region of interest for CCD is exactly where $d \to 0$, which is where cubic root-finding is hardest.

## Algorithm classes

Two algorithm families exist for robust CCD root-finding:

**Analytic CCD.** Solves the cubic directly via closed-form formulas (Cardano), then filters roots by interval constraints. Fast when numerically well-conditioned, fragile near degenerate cases where the cubic has near-double roots.

**Interval CCD.** Represents the positions and resulting polynomials as intervals (rather than floating-point scalars), then bisects the $\alpha$-interval adaptively to isolate roots with guaranteed error bounds. Slower per query but robust in the degenerate-root regime. Inclusion-based methods in this family emerged in the 2021 and later follow-up literature and were adopted by subsequent [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) releases.

Li et al. 2020 introduced IPC using a conservative step-size cap based on spatial bounds rather than a full high-precision CCD — trial steps are clamped to a fraction of the minimum current gap to guarantee no primitive travels more than a fraction of its distance to its nearest neighbor. This is an under-stepping strategy: correct but sometimes over-conservative, yielding shorter steps than CCD-tight analysis would require. Later work (C-IPC, Li et al. 2021, and subsequent [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) releases) adopted tighter CCD queries. `sim-soft` will port whichever CCD variant the IPC Toolkit's current release uses at the time of [Part 8 GPU porting](../../80-gpu/00-wgpu-layout.md); the choice does not affect the barrier-energy formulation above it, only the per-step CCD cost and tightness.

## Integration with the filtered line search

The filtered line search of Li et al. 2020 uses CCD to provide a stronger guarantee than standard Armijo backtracking. A standard Armijo line search finds an $\alpha$ such that $U(x + \alpha \Delta x) < U(x) + c_1 \alpha \nabla U \cdot \Delta x$ — a decrease condition at the *endpoint* of the step. The filtered variant additionally requires that the CCD query over $[0, \alpha]$ return $\alpha^\ast > \alpha$ (no intersection anywhere in the step), and takes the minimum of the Armijo-acceptable step length and $\alpha^\ast$ (with a safety factor). This guarantees that every Newton iterate is reachable from the previous iterate via a non-intersecting straight-line path — not just that the endpoint is non-intersecting.

The combination preserves the intersection-free invariant across every Newton iterate of every timestep, which is the path-level statement of the [Ch 00 Guarantee 1](../00-why-ipc/02-ipc-guarantees.md) non-penetration property. Standard Armijo backtracking paired with a discrete snapshot at the endpoint would not provide this guarantee; CCD in the line search is what makes the guarantee structural rather than probabilistic.

## What this sub-leaf commits the book to

- **CCD is part of the contract, not an optional optimization.** Every trial Newton step in `sim-soft`'s contact-active regime goes through the CCD filter. A variant of `sim-soft` that used discrete snapshot collision checks instead would lose the Ch 00 Guarantee 1 non-penetration invariant and cease to be IPC in the technical sense.
- **The CCD algorithm is a porting target, not a design choice.** `sim-soft` inherits whichever CCD algorithm the [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit)'s current release uses. Conservative step-capping and inclusion-based interval methods both guarantee non-intersection by construction; analytic cubic root-finding achieves it when the cubic is numerically well-conditioned but can fail in degenerate cases, which is why the IPC line evolved away from it toward interval methods. The three families differ in tightness, per-query cost, and robustness — not in the correctness goal. [Part 8 Ch 00 GPU layout](../../80-gpu/00-wgpu-layout.md) discusses porting the chosen CCD variant to wgpu compute kernels.
- **Point-triangle and edge-edge are the only two primitive cases.** `sim-soft` surface meshes are triangle meshes; all contact pairs reduce to one of these two queries. Additional primitive types (point-point, edge-point, etc.) either reduce to these — the point-triangle tests that range over each vertex's incident triangles cover vertex-vertex approaches as a special case — or are excluded by the proximity filter.
