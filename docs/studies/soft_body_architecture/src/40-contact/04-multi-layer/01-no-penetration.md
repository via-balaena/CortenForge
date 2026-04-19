# No-penetration between stacked sleeves

A three-or-more-layer stack introduces inter-mesh contact pairs at every interface between adjacent layers. [Ch 00 Guarantee 1](../00-why-ipc/02-ipc-guarantees.md) established that IPC's barrier prevents any single pair's gap from crossing zero; this sub-leaf extends that guarantee to the multi-layer case and names why the extension is automatic. The key observation: IPC's total potential sums over all pairs in a single global minimization, so the Newton loop and the CCD filter act simultaneously on every interface, regardless of stack depth.

## The unified potential across all layers

For a configuration with $L$ separately meshed layers and some set of inter-layer and intra-layer contact pairs, the total potential at each timestep reads:

$$ U_\text{total}(x) = \tfrac{1}{2}(x - \hat x)^\top M(x - \hat x) + \sum_{l=1}^{L} h^2\, \Psi_\text{elastic}^{(l)}(x^{(l)}) + h^2 \kappa \sum_{k \in \mathcal{C}_\text{total}} b(d_k(x), \hat d) + h^2 \sum_{k \in \mathcal{C}_\text{total}} \mu_{c,k}\, \lambda_k^n\, f_0(\|u_T^k(x)\|) $$

where $x = (x^{(1)}, \ldots, x^{(L)})$ concatenates all layers' vertex positions, $\Psi_\text{elastic}^{(l)}$ is layer $l$'s per-layer elastic potential with that layer's material parameters, and $\mathcal{C}_\text{total} = \bigcup_{(A,B)}\, \mathcal{C}_{AB}$ is the union of active contact pairs across all inter-layer pairings (plus any intra-layer self-contact pairs from [Ch 03](../03-self-contact.md)). The barrier and friction sums range over this total active set, with no layer-pair structure leaking into the sums.

This framing is the architectural consequence of Li 2020's single-minimization formulation — the paper writes the total contact potential as one sum over all pairs across all contacting bodies, and the multi-layer case is an instance of that formulation, not an extension of it. The Newton loop minimizes this single $U_\text{total}(x)$; the inner linear solve factorizes one sparse Hessian covering all layers' degrees of freedom; the CCD filter from [Ch 01 CCD](../01-ipc-internals/02-ccd.md) runs on the full concatenated step $\Delta x = (\Delta x^{(1)}, \ldots, \Delta x^{(L)})$.

## Why there is no layer-by-layer alternative

The hypothetical architectural alternative — solving each layer's equilibrium sequentially in a Gauss-Seidel fashion (solve layer 1 with layers 2 through $L$ fixed, then layer 2 with updated layer 1 and fixed 3 through $L$, iterate) — preserves per-inner-step non-penetration (each inner solve has its own CCD) but loses the *coupled-equilibrium* convergence guarantee that the unified formulation provides through Newton on the full $U_\text{total}$. Consider a three-layer stack with a middle layer pinched between two outer layers under compression:

- **Global Newton (what IPC does).** One step minimizes $U_\text{total}$ over all three layers' positions simultaneously. CCD filters the full three-layer step against every pair; if any layer's trajectory would pass through any other's anywhere along the step, the step is truncated. Non-penetration holds across all layers across the full step interior.
- **Gauss-Seidel alternative (what IPC does not do).** Solve layer 1 with layers 2 and 3 fixed (letting layer 1 settle under the pinching load from layers 2 and 3 at their current positions). Then solve layer 2 with the updated layer 1 and fixed layer 3. Then layer 3. Each inner solve's CCD ensures that inner solve's step is non-penetrating, but nothing ties the inner solves together into a coupled equilibrium — the alternation between layers is not guaranteed to converge to the unified-global minimum, especially when inter-layer coupling is stiff. A pinching configuration where the middle layer must simultaneously respond to both outer layers' motion can take many outer-iteration passes to converge, and nothing prevents the iteration from cycling between alternative stable arrangements where each per-layer solve is locally optimal but the combined configuration is not the coupled minimum.

IPC's unified formulation is the path-correct approach for coupled multi-layer contact. Sequential / iterative constraint solvers are standard in rigid-body engines (where the per-body state is simpler and path-level invariants are not required), but they do not extend to IPC's CCD-filtered non-penetration guarantee.

## The middle-layer pinch case

A canonical three-layer pinch — probe pressing on a stacked sleeve-liner-core assembly — is the archetypal multi-layer contact case the canonical-problem workload stresses. At equilibrium, the middle layer's deformation depends on three coupled quantities:

- The compressive load the probe transmits through the top sleeve.
- The reactive stiffness of the core below.
- Inter-layer friction at both the sleeve-liner and liner-core interfaces (if either material pair has nonzero $\mu_c$).

The global Newton captures this three-way coupling through the Hessian's off-diagonal blocks automatically: pairs in $\mathcal{C}_\text{sleeve-liner}$ contribute entries linking sleeve DOFs to liner DOFs via the barrier's and friction's mixed partials; pairs in $\mathcal{C}_\text{liner-core}$ link liner DOFs to core DOFs; the resulting sparse Hessian has a block structure approximating block-tridiagonal, with some off-tridiagonal fill where non-adjacent layers come close enough for the [Ch 03 BVH](../03-self-contact/00-bvh.md) to detect pairs between them. One factorization captures all three coupling directions; one linear solve propagates the load through the stack.

This is why `sim-soft` does not ship a separate "multi-body coupling" module: the sparse Hessian's cross-block structure is an emergent property of barrier and friction terms being summed into one potential, not of any explicit multi-body equation.

## Stack depth is a cost property, not a correctness property

The IPC guarantees from Ch 00 — non-penetration, $C^2$ smoothness, warm-start capability, differentiability — hold at any stack depth, by the same argument. What scales with depth is cost, not correctness:

- **Inter-layer pair count** grows linearly with stack depth (each adjacent interface contributes its own active-pair set; non-adjacent-layer pairs appear when deformed layers come close).
- **Hessian sparsity fill** grows with the sum of per-layer DOF counts plus the inter-layer pair fill — more off-diagonal blocks as more inter-layer couplings activate.
- **Newton iteration count per timestep** can grow if the inter-layer coupling stiffens the problem (deep stacks under high load require more Newton iterates to equilibrate).

The [Ch 05 real-time chapter](../05-real-time.md)'s cost-scaling analysis treats stack depth as a cost parameter the [GPU lever](../05-real-time/00-gpu.md) and [caching lever](../05-real-time/02-caching.md) help amortize. Per-pair work stays at barrier + friction regardless of depth; the growth is in pair count and linear-solve fill. No correctness claim degrades with stack depth.

## What this sub-leaf commits the book to

- **No "multi-layer solver" module.** `sim-soft` does not ship a `MultiLayerSolver` or `LayerCouplingScheme`; the one-global-Newton architecture from [Ch 01 energy-based formulation](../01-ipc-internals/03-energy.md) handles any number of layers without special-casing. Number of layers is a runtime property of the scene, not a compile-time configuration.
- **CCD runs on the concatenated step, not layer-by-layer.** `sim-soft`'s [CCD filter](../01-ipc-internals/02-ccd.md) sees $\Delta x$ for all layers simultaneously; the filter returns one $\alpha^\ast$ bounding the step for all pairs across all layers. This is what extends the Ch 00 Guarantee 1 non-penetration invariant from the two-body case to arbitrary stack depth.
- **The middle-layer pinch case is representable without new machinery.** Three-way (or $L$-way) coupling via Hessian off-diagonal blocks is automatic in the [sparse Hessian assembly](../../80-gpu/01-sparse-matrix/01-bsr.md). No multi-body-coupling term, no separate middle-layer equilibrium equation, no explicit Gauss-Seidel outer loop.
- **Stack depth scales cost roughly linearly at fixed per-pair work.** The guarantees are depth-invariant; the [Ch 05 engineering levers](../05-real-time.md) are what keep per-step cost within the real-time budget at realistic stack depths.
