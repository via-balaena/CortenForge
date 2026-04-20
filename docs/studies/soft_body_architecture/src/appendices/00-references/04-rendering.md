# Rendering papers

[Part 9 (visual layer)](../../90-visual/00-sss.md)'s commitments — multi-layer subsurface scattering, anisotropic GGX specular, contact-driven micro-wrinkles, subdivision surfaces, thermal visualization — trace back to the rendering literature, alongside the differentiable-rendering frontier that [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) cites as the closest-adjacent precedent for topology-aware FEM gradients. Six clusters: differentiable rendering, subsurface scattering, microfacet and layered BRDFs, surface-detail (bump/displacement), subdivision surfaces, and perceptual colormaps. Each anchor below is traced to the specific Part 6 or Part 9 sub-leaf and claim the paper grounds.

## Differentiable rendering

## Li, Aittala, Durand & Lehtinen 2018 {#li-2018}

*Differentiable Monte Carlo Ray Tracing through Edge Sampling.* ACM Transactions on Graphics 37(6), Article 222 (SIGGRAPH Asia 2018), pp. 222:1–222:11. DOI [10.1145/3272127.3275109](https://doi.org/10.1145/3272127.3275109). [Author project page](https://people.csail.mit.edu/tzumao/diffrt/). Authors: Tzu-Mao Li, Miika Aittala, Frédo Durand, Jaakko Lehtinen.

The foundational paper on differentiable rendering through visibility discontinuities. Formulates the pixel-colour derivative as an interior smooth integral plus an explicit boundary integral along silhouette edges, with the boundary integral absorbing the Dirac contribution from the discontinuous visibility function, and a Monte-Carlo estimator that samples along edges via spatial-hierarchy importance sampling. Cited inline from [Part 6 Ch 05 (research frontier — Option B)](../../60-differentiability/05-diff-meshing.md) as the closest published parallel for a topology-aware FEM gradient: if the Dirac contribution at a mesh-topology-change boundary can be integrated analytically in the same style, the FD wrapper at topology boundaries can be replaced by an unbiased estimator. The extension to FEM topology changes is unpublished for volumetric FEM to the book's knowledge; [Part 12 Ch 07's Option B](../../120-roadmap/07-open-questions.md) names it as research frontier.

## Bangaru, Li & Durand 2020 {#bangaru-2020}

*Unbiased Warped-Area Sampling for Differentiable Rendering.* ACM Transactions on Graphics 39(6), Article 245 (SIGGRAPH Asia 2020), pp. 245:1–245:18. DOI [10.1145/3414685.3417833](https://doi.org/10.1145/3414685.3417833). [Author project page](https://people.csail.mit.edu/sbangaru/projects/was-2020/). Authors: Sai Bangaru, Tzu-Mao Li, Frédo Durand.

Applies the divergence theorem to convert [Li et al. 2018](#li-2018)'s edge-boundary integral into an area integral via a warped-area reparameterization, giving an unbiased estimator without explicit silhouette detection. Cited inline from [Part 6 Ch 05 (research frontier — Option B)](../../60-differentiability/05-diff-meshing.md) as the canonical unbiased generalization of the visibility-boundary treatment — the book's Option B invokes unbiased estimation explicitly, and Bangaru 2020 is the rendering-side reference point for what a working unbiased boundary-integral estimator looks like. The FEM topology-change analog would need its own warped-area formulation; the rendering-side precedent shows the mathematical structure is tractable.

## Subsurface scattering

## Jensen, Marschner, Levoy & Hanrahan 2001 {#jensen-2001}

*A Practical Model for Subsurface Light Transport.* Proceedings of the 28th Annual Conference on Computer Graphics and Interactive Techniques (SIGGRAPH 2001), pp. 511–518. DOI [10.1145/383259.383319](https://doi.org/10.1145/383259.383319). Authors: Henrik Wann Jensen (Stanford University at press time), Stephen R. Marschner (Stanford), Marc Levoy (Stanford), Pat Hanrahan (Stanford).

Introduces the dipole-diffusion approximation to the BSSRDF: a real point source at depth $z_r = 1/\sigma_t'$ below the entry point plus a virtual image source above the surface, positioned and signed to satisfy the diffusion boundary condition, with the surface-exit radial distribution $R_d(r)$ expressed as a dipole sum of two contributions each of the form $z(1 + \sigma_{tr} d)\, e^{-\sigma_{tr} d} / d^3$. Cited inline from [Part 9 Ch 00 §01 — BSSRDF basics](../../90-visual/00-sss/01-bssrdf.md) as the closed-form solution the chapter reduces the full 8D BSSRDF to. The book's diffusion-approximation section takes the algebraic reduction from this paper rather than reproducing the specific coefficients.

## Jimenez, Zsolnai, Jarabo, Freude, Auzinger, Wu, von der Pahlen, Wimmer & Gutierrez 2015 {#jimenez-2015}

*Separable Subsurface Scattering.* Computer Graphics Forum 34(6) (Pacific Graphics 2015), pp. 188–197. DOI [10.1111/cgf.12529](https://doi.org/10.1111/cgf.12529). Authors: Jorge Jimenez (Activision R&D), Károly Zsolnai (TU Wien), Adrián Jarabo (Universidad de Zaragoza), Christian Freude (TU Wien), Thomas Auzinger (TU Wien), Xian-Chun Wu (Activision R&D), Javier von der Pahlen (Activision R&D), Michael Wimmer (TU Wien), Diego Gutierrez (Universidad de Zaragoza).

The production-grade screen-space diffusion-profile convolution technique that decomposes the 2D spatial profile into two separable 1D convolutions, making per-pixel SSS tractable at real-time rates on modern GPUs. Cited inline from [Part 9 Ch 00 §02 — Real-time approximations](../../90-visual/00-sss/02-realtime.md) as the Phase I SSS commitment (tier-b diffusion-profile convolution), and from the [Part 9 Ch 00 (SSS spine)](../../90-visual/00-sss.md) as the industry-standard method `sim-soft` ships. Venue: Pacific Graphics 2015 via CGF issue 6 (not EGSR 2015, which is CGF issue 4 that year).

## d'Eon & Luebke 2007 {#deon-luebke-2007}

*Advanced Techniques for Realistic Real-Time Skin Rendering.* GPU Gems 3, Chapter 14 (Addison-Wesley / NVIDIA, 2007). [NVIDIA-hosted full text](https://developer.nvidia.com/gpugems/gpugems3/part-iii-rendering/chapter-14-advanced-techniques-realistic-real-time-skin). Authors: Eugene d'Eon (NVIDIA), David Luebke (NVIDIA).

The production-grade texture-space (UV-domain) diffusion-profile convolution technique for real-time skin rendering. Introduces the sum-of-Gaussians approximation to $R_d(r)$ that subsequent screen-space variants inherit — six Gaussians per color channel give visual fidelity matched against Monte Carlo ground truth on measured skin parameters. Cited inline from [Part 9 Ch 00 §00 — Stacked translucent materials](../../90-visual/00-sss/00-stacked.md) and [§02 — Real-time approximations](../../90-visual/00-sss/02-realtime.md) as the texture-space predecessor that Jimenez 2009 moved into screen space and Jimenez 2015 further optimized.

## Jimenez, Sundstedt & Gutierrez 2009 {#jimenez-2009}

*Screen-Space Perceptual Rendering of Human Skin.* ACM Transactions on Applied Perception 6(4), Article 23, 2009. DOI [10.1145/1609967.1609970](https://doi.org/10.1145/1609967.1609970). Authors: Jorge Jimenez (Universidad de Zaragoza at press time), Veronica Sundstedt (Trinity College Dublin), Diego Gutierrez (Universidad de Zaragoza).

Moves the diffusion-profile convolution into screen space, handling occlusion correctly without texture-atlasing via depth-aware sampling. The screen-space formulation is what [Jimenez 2015](#jimenez-2015)'s separable-kernel result builds on. Cited inline from [Part 9 Ch 00 §00](../../90-visual/00-sss/00-stacked.md) and [§02](../../90-visual/00-sss/02-realtime.md) as the screen-space predecessor of the separable-profile form `sim-soft` ships; the book's per-pixel-compound architecture inherits the screen-space domain from this paper and extends its single-layer assumption to the multi-layer compound-variance case.

## Christensen & Burley 2015 {#christensen-burley-2015}

*Approximate Reflectance Profiles for Efficient Subsurface Scattering.* Pixar Technical Memo #15-04, July 2015. [Pixar-hosted PDF](https://graphics.pixar.com/library/ApproxBSSRDF/paper.pdf). Authors: Per H. Christensen, Brent Burley (both Pixar Animation Studios).

Fits the $R_d(r)$ radial diffusion profile empirically to Monte Carlo ground-truth as a sum of two exponentials per color channel, parameterized by a single mean-free-path shape factor and a normalization. Captures the steeper near-$r = 0$ peak from single scattering plus the broader diffusion tail without a material-specific fit per new silicone — correcting the dipole's too-flat near-zero and too-quick falloff. Cited inline from [Part 9 Ch 00 §01](../../90-visual/00-sss/01-bssrdf.md) as `sim-soft`'s default stored profile shape, and from [Part 9 Ch 00 §00 — Stacked translucent materials](../../90-visual/00-sss/00-stacked.md) where the sum-of-exponentials form is contrasted against the sum-of-Gaussians compounding used for multi-layer runs.

## Microfacet and layered BRDFs

## Walter, Marschner, Li & Torrance 2007 {#walter-2007}

*Microfacet Models for Refraction through Rough Surfaces.* Rendering Techniques 2007 (Proceedings of the 18th Eurographics Symposium on Rendering, EGSR 2007), pp. 195–206. DOI [10.2312/EGWR/EGSR07/195-206](https://doi.org/10.2312/EGWR/EGSR07/195-206). Authors: Bruce Walter (Cornell University), Stephen R. Marschner (Cornell), Hongsong Li (Cornell), Kenneth E. Torrance (Cornell).

Introduces the GGX normal-distribution function (NDF) for microfacet-based surfaces, with heavier power-law tails than Beckmann or Phong that fit measured rough-surface BRDFs better at high roughness. The paper also gives the importance-sampling mapping from uniform $(u_1, u_2) \in [0, 1]^2$ to microfacet normal $\mathbf{m}$ that offline reference paths need. Cited inline from [Part 9 Ch 01 §01 — Anisotropic GGX](../../90-visual/01-anisotropic-reflection/01-ggx.md) as the isotropic-GGX source and from the [Part 9 Ch 01 (anisotropic-reflection spine)](../../90-visual/01-anisotropic-reflection.md) as the baseline NDF `sim-soft` adopts. The heavy tail is what motivates GGX over Phong for silicone at moderately-high $\alpha$.

## Burley 2012 {#burley-2012}

*Physically Based Shading at Disney.* ACM SIGGRAPH 2012 Courses — "Practical Physically Based Shading in Film and Game Production" (course notes). [Self-shadow-hosted PDF](https://blog.selfshadow.com/publications/s2012-shading-course/burley/s2012_pbs_disney_brdf_notes_v3.pdf). Author: Brent Burley (Walt Disney Animation Studios).

The Disney principled-BRDF course notes. Extends [Walter 2007](#walter-2007)'s isotropic GGX to an anisotropic form by replacing the single roughness $\alpha$ with two roughnesses $\alpha_x, \alpha_y$ along per-vertex tangent axes $\mathbf{t}_x, \mathbf{t}_y$, and introduces the Disney principled-BRDF parameterization widely adopted as the modern production-renderer baseline. Cited inline from [Part 9 Ch 01 §01](../../90-visual/01-anisotropic-reflection/01-ggx.md) as the anisotropic-parameterization source and the re-targetability argument (`disney/brdf` reference implementation, measured-material fits published in Disney form). Smith masking-shadowing in its anisotropic form per Burley is required for energy conservation in `sim-soft`'s specular shader.

## Jensen, Legakis & Dorsey 1999 {#jensen-legakis-dorsey-1999}

*Rendering of Wet Materials.* Rendering Techniques '99 (Proceedings of the 10th Eurographics Workshop on Rendering, EGWR 1999), pp. 273–282. DOI [10.2312/EGWR/EGWR99/273-282](https://doi.org/10.2312/EGWR/EGWR99/273-282). Authors: Henrik Wann Jensen (MIT at press time), Justin Legakis (MIT), Julie Dorsey (MIT).

The canonical rendering-literature reference for wet-surface appearance: a layered BRDF with an interfacial film that produces darkening, specular-shift, and a secondary reflection. Cited inline from [Part 9 Ch 01 §00 — Wet-surface BRDF](../../90-visual/01-anisotropic-reflection/00-wet.md) as the structural precedent `sim-soft` inherits; the book drops the moisture-film layer (Phase I does not model interfacial films), keeping only the specular-tightening modulation that describes silicone under contact.

## Ashikhmin & Shirley 2000 {#ashikhmin-shirley-2000}

*An Anisotropic Phong BRDF Model.* Journal of Graphics Tools 5(2), pp. 25–32, 2000. DOI [10.1080/10867651.2000.10487522](https://doi.org/10.1080/10867651.2000.10487522). Authors: Michael Ashikhmin (University of Utah at press time), Peter Shirley (University of Utah).

An anisotropic, energy-conserving, reciprocal Phong-based BRDF with two roughnesses along the per-vertex tangent axes — cheaper than GGX to evaluate. Cited inline from [Part 9 Ch 01 §01](../../90-visual/01-anisotropic-reflection/01-ggx.md) as the rejected alternative: the Phong tail does not match measured microfacet data at the high-roughness regime where silicone lives, and the industry ecosystem has standardised on GGX-native pipelines. The citation is carried so readers can see the comparison `sim-soft` evaluated against.

## Surface-detail (bump / displacement)

## Blinn 1978 {#blinn-1978}

*Simulation of Wrinkled Surfaces.* Proceedings of the 5th Annual Conference on Computer Graphics and Interactive Techniques (SIGGRAPH 1978), pp. 286–292. DOI [10.1145/800248.807075](https://doi.org/10.1145/800248.807075). Author: James F. Blinn (NYIT at press time).

The founding bump-mapping paper. Perturbs the surface normal at shading time based on a scalar *bump field* without moving geometry — the mathematical structure every subsequent normal-map, parallax-map, and surface-detail technique in graphics descends from. Cited inline from [Part 9 Ch 02 §00 — Normal maps from contact pressure](../../90-visual/02-micro-wrinkles/00-contact-normals.md) as the mathematical construction `sim-soft` reuses directly; the contact-pressure-driven bump signal that the shader consumes is the novel part, with no established rendering-literature precedent (see the sub-leaf for the honest-negative framing).

## Cook 1984 {#cook-1984}

*Shade Trees.* Computer Graphics 18(3) (Proceedings of SIGGRAPH 1984), pp. 223–231. DOI [10.1145/964965.808602](https://doi.org/10.1145/964965.808602). Author: Robert L. Cook (Lucasfilm / Pixar at press time).

Introduces the shade-tree abstraction for composable surface shading and lists displacement mapping as one of the surface-shading operators a shade tree can express — the first formal treatment of displacement as a shading-pipeline operation. Cited inline from [Part 9 Ch 02 §01 — Displacement mapping](../../90-visual/02-micro-wrinkles/01-displacement.md) as the origin of displacement mapping in the rendering literature; the Reyes renderer architecture (Cook, Carpenter & Catmull) productionised the technique, and `sim-soft`'s displacement pass inherits the move-vertices-by-height-field formulation from this lineage.

## Bridson, Marino & Fedkiw 2003 {#bridson-2003}

*Simulation of Clothing with Folds and Wrinkles.* Proceedings of the 2003 ACM SIGGRAPH / Eurographics Symposium on Computer Animation (SCA 2003), pp. 28–36. DOI [10.1145/846276.846284](https://doi.org/10.1145/846276.846284). Authors: Robert Bridson (Stanford at press time), Sebastian Marino (Alias|Wavefront), Ronald Fedkiw (Stanford).

Derives cloth folds and wrinkles from compressive deformation in the solver rather than from authored textures. Cited inline from [Part 9 Ch 02 §00](../../90-visual/02-micro-wrinkles/00-contact-normals.md) as the closest structural parallel to `sim-soft`'s physics-driven-wrinkles architecture — with the important difference that Bridson et al. resolve wrinkles in the cloth mesh's dynamics, whereas `sim-soft` keeps wrinkle reconstruction on the shader side of the physics-render boundary. The engineering rationale — physics drives wrinkles, art does not — is shared; the implementation layer differs.

## Subdivision surfaces

## Catmull & Clark 1978 {#catmull-clark-1978}

*Recursively generated B-spline surfaces on arbitrary topological meshes.* Computer-Aided Design 10(6), pp. 350–355, 1978. DOI [10.1016/0010-4485(78)90110-0](https://doi.org/10.1016/0010-4485(78)90110-0). Authors: Edwin Catmull (University of Utah at press time), James Clark (University of Utah).

The original quad subdivision scheme. Produces a bicubic B-spline limit surface at regular vertices, with $C^2$ continuity there and $C^1$ at extraordinary vertices. Cited inline from [Part 9 Ch 03 (subdivision spine)](../../90-visual/03-subdivision.md) and [Part 9 Ch 03 §01 — Loop vs Catmull-Clark](../../90-visual/03-subdivision/01-loop-vs-cc.md) as the quad-subdivision reference; `sim-soft` exposes Catmull-Clark as opt-in via a quadrangulation pre-pass for downstream pipelines that expect quad meshes, but the default is Loop on the native-triangle tet-boundary output.

## Loop 1987 {#loop-1987}

*Smooth Subdivision Surfaces Based on Triangles.* M.S. thesis, Department of Mathematics, University of Utah, August 1987. Author: Charles Loop.

The original triangle subdivision scheme. Each subdivision level refines each triangle into four smaller triangles while smoothing vertex positions via a weighted stencil over ring neighbors; the limit surface is $C^2$ at regular vertices and $C^1$ at extraordinary vertices. Cited inline from [Part 9 Ch 03](../../90-visual/03-subdivision.md), [§00 — Coarse-to-fine](../../90-visual/03-subdivision/00-coarse-to-fine.md), and [§01](../../90-visual/03-subdivision/01-loop-vs-cc.md) as the default subdivision scheme for `sim-soft`'s triangle-native tet-boundary output. The Utah thesis is the citation the subdivision literature standardised on; no subsequent journal version supersedes it as the canonical reference.

## Stam 1998 {#stam-1998}

*Exact Evaluation of Catmull-Clark Subdivision Surfaces at Arbitrary Parameter Values.* Proceedings of the 25th Annual Conference on Computer Graphics and Interactive Techniques (SIGGRAPH 1998), pp. 395–404. DOI [10.1145/280814.280945](https://doi.org/10.1145/280814.280945). Author: Jos Stam (Alias|wavefront at press time).

Gives a closed-form eigenbasis-decomposition evaluator for the Catmull-Clark limit surface at arbitrary $(u, v)$ parameter values, reducing analytic-surface queries to a finite matrix computation rather than unbounded recursive subdivision. Cited inline from [Part 9 Ch 03 §01](../../90-visual/03-subdivision/01-loop-vs-cc.md) as the regression-test tool — `sim-soft` does not invoke the Stam evaluator at runtime (finite-level subdivision is what the shader consumes), but the analytic evaluator is present as a correctness oracle against which finite-level output is regression-tested, and as an entry point for the [Part 12 Ch 07 differentiable-meshing research-frontier hooks](../../120-roadmap/07-open-questions.md).

## Perceptual colormaps

## Smith & van der Walt 2015 {#viridis-2015}

*A Better Default Colormap for Matplotlib.* SciPy 2015 conference talk (Austin, TX, July 2015). [BIDS project page](https://bids.github.io/colormap/). [Talk video (YouTube)](https://www.youtube.com/watch?v=xAoljeRJ3lU). Authors: Nathaniel J. Smith, Stéfan van der Walt.

The design rationale for the viridis perceptually-uniform colormap family (viridis, plasma, inferno, magma). Two separately-verifiable design properties: perceptual uniformity (equal steps along the scalar axis produce equal visually-perceived steps in CAM02-UCS space) and monotonic luminance (each position is brighter than earlier positions, so grayscale conversion preserves ordering). No accompanying paper or technical report — the SciPy 2015 talk is the canonical citation. Cited inline from [Part 9 Ch 04 §00 — Temperature → color mapping](../../90-visual/04-thermal-viz/00-color-map.md) as the origin-of-viridis reference; viridis landed in matplotlib 1.5 (October 2015) as an option and became the matplotlib default in 2.0 (January 2017).
