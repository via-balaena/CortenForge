# Loop vs Catmull-Clark

Two subdivision schemes dominate the rendering literature, and the choice between them is determined by the input mesh's topology. [Loop 1987](../../appendices/00-references/04-rendering.md)'s scheme operates on triangles; [Catmull & Clark 1978](../../appendices/00-references/04-rendering.md)'s operates on quadrilaterals. `sim-soft`'s tet mesh produces a triangular surface triangulation natively — each boundary tet face is a triangle — so Loop is the default. Catmull-Clark is available as an opt-in for users whose downstream pipeline prefers quads, with a re-tessellation pass injected upstream of subdivision.

## Loop subdivision

[Loop 1987](../../appendices/00-references/04-rendering.md)'s master's thesis at Utah introduces the scheme: each input triangle becomes four output triangles per level (insert a new vertex at each edge, connect them to form the four smaller triangles), and every vertex — old and new — is repositioned via a weighted-average stencil over its ring neighbors. The stencil weights are specifically designed so that the limit surface (infinite subdivision) is $C^2$-continuous everywhere except at **extraordinary vertices** (vertices with valence $\ne 6$ for interior points, or $\ne 4$ for boundary points), where it drops to $C^1$. For the canonical soft-body meshes `sim-soft` produces from tet boundaries, extraordinary vertices are unavoidable — the Euler characteristic forces at least some of them on any closed surface — but $C^1$ continuity is still smooth enough for visual purposes, and the shading techniques from [§0 normal perturbation](../02-micro-wrinkles.md) do not depend on higher-order continuity at vertices.

Loop produces a denser triangle mesh at each subdivision level: 4× triangles per level. The canonical 12k-triangle surface becomes 48k at one level and 192k at two. The second level is used only when [displacement mapping](../02-micro-wrinkles/01-displacement.md) is active and needs finer geometry to resolve wrinkle frequency; for default Phase I shading, one level suffices.

## Catmull-Clark subdivision

[Catmull & Clark 1978](../../appendices/00-references/04-rendering.md) in Computer-Aided Design 10(6) introduces subdivision on quadrilateral meshes. Each input quad becomes four smaller quads per level, with weighted-average stencils producing a limit surface that is bicubic B-spline at regular vertices (valence 4 interior, valence 3 boundary) and $C^1$ at extraordinary vertices. Catmull-Clark is widely used in production character animation — Pixar's RenderMan in particular is built around it — because animation-authored quad meshes are common and bicubic B-spline continuity matches the smoothness expected on character surfaces.

`sim-soft` does not ship Catmull-Clark as the default because its inputs are triangles: tet-boundary triangulation produces triangular faces. Running Catmull-Clark would require an additional re-tessellation step that merges triangle pairs into quadrilaterals, and not every triangulation admits a clean quadrangulation — the topology has to be cooperative. For users who want Catmull-Clark output (typically for downstream pipelines expecting quads), `sim-soft` exposes an optional `SubdivisionMode::CatmullClark` setting that invokes a pre-subdivision quadrangulation pass. Quality of the resulting mesh depends on how well the triangulation maps to quad pairs; for triangulations dominated by right-angled pairs (near rectangular-cross-section features), the result is good; for triangulations with many mixed-valence interior vertices, the quadrangulation introduces artifacts that Loop would not.

## Analytic evaluation via Stam 1998

Finite-level subdivision produces a mesh that approximates the infinite-level limit surface. For the shader pipeline, the finite approximation is exactly what the rasterizer wants — a dense triangle mesh. But some physics-aware rendering applications, particularly differentiable-rendering gradient flows through subdivision, benefit from evaluating the *limit surface* at arbitrary parameter values without actually subdividing to some high level. [Stam 1998](../../appendices/00-references/04-rendering.md) provides the standard closed-form evaluation for Catmull-Clark limit surfaces: eigendecomposition of the subdivision matrix gives an eigenbasis in which the limit-surface position and partial derivatives at any parameter value are a weighted combination of control-point positions with analytically-computable weights.

`sim-soft` implements Stam 1998 only for the regression-test path (compare finite-level subdivision output against the analytic limit surface to verify stencil correctness) and for the [Part 12 Ch 07 research-frontier differentiable-meshing experiments](../../120-roadmap/07-open-questions.md). The Phase I runtime uses finite-level Loop exclusively.

Closed-form evaluation for Loop limit surfaces has been worked out in subsequent literature following the same eigenbasis approach. `sim-soft`'s Loop default does not invoke any such closed-form evaluator at runtime — the finite-level form is what the shader consumes — but the hooks exist at the regression-test layer for Loop too if needed.

## Continuity summary

| Scheme | Input | Level multiplier | Regular-vertex continuity | Extraordinary-vertex continuity |
|---|---|---|---|---|
| Loop | triangles | 4× per level | $C^2$ | $C^1$ |
| Catmull-Clark | quads | 4× per level | bicubic B-spline (= $C^2$ at regular) | $C^1$ |

Both schemes are $C^1$ at extraordinary vertices. The difference that matters to `sim-soft` is input topology, not output continuity — both produce visually smooth surfaces at the scales Phase I cares about.

## What this sub-leaf commits the book to

- **Loop 1987 is the default subdivision scheme.** Triangles in, triangles out; matches `sim-soft`'s tet-boundary output natively; $C^2$ at regular vertices, $C^1$ at extraordinary.
- **Catmull-Clark 1978 is an opt-in via a quadrangulation pre-pass.** Useful when downstream pipelines expect quads; quality depends on how well the triangulation admits clean quad pairing.
- **Stam 1998 analytic evaluation is a regression-test tool**, not part of the Phase I runtime. Hooks exist for differentiable-rendering research-frontier work from [Part 12 Ch 07](../../120-roadmap/07-open-questions.md).
- **$C^1$ at extraordinary vertices is sufficient for the Phase I shaders.** None of the [§Ch 00–02 shaders](../00-sss.md) depend on higher-order continuity at vertices; the apparent smoothness comes from shader-level reconstruction and does not require $C^2$ at the geometric level.
