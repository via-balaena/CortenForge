# Real-time approximations

The [§1 BSSRDF](01-bssrdf.md) formalism reduces to a 1D radial profile $R_d(r)$ under the diffusion approximation on a locally-planar surface. This sub-leaf covers the three real-time techniques that convolve that profile against the lit-surface radiance in a shader budget compatible with interactive rendering. `sim-soft` ships the middle tier as its [Phase I](../../110-crate/03-build-order.md) commitment and keeps the lowest tier available as a debug fallback; the upper tier (volumetric Monte Carlo) is not shipped. Each tier's cost model and visual failure mode is stated so that the choice of tier is a defensible shader-architecture decision, not a taste preference.

## Tier (a) — Normal-tinted baseline

The cheapest technique reads the mesh normal $n$ at the current fragment, samples a scatter-color texture at an offset along $n$ (typically 1–3 mm in world space, mapped to screen space), and blends it with the surface albedo. One texture tap per pixel, constant cost, works on any GPU.

```wgsl
let tangent_offset: f32 = 2.0;                              // mm along normal
let offset_uv: vec2<f32> = uv + n.xy * tangent_offset * uv_per_mm;
let scatter_color: vec3<f32> = textureSample(scatter_tex, samp, offset_uv).rgb;
let out: vec3<f32> = mix(albedo, scatter_color, scatter_strength);
```

This is wrong in three specific ways: (1) the scatter profile does not vary with incoming-light direction — the total lighting is still modulated by $N \cdot L$ as for any shader, but the scattering *signature* (color tint, spatial spread) is a fixed offset rather than an angle-dependent response; (2) it does not vary with feature size, so a thin rim scatters the same as a thick cavity wall; (3) it does not compose across material layers, so the [§0 stacked-material](00-stacked.md) composition is silently ignored. The visual consequence is the "plastic-looking silicone" failure mode from the [parent chapter's §1](../00-sss.md). Tier (a) is shipped only as a **debug fallback** — enabled for performance regression testing or for hardware that cannot spare the Tier (b) budget. Production `sim-soft` runs never engage it.

## Tier (b) — Separable screen-space diffusion-profile convolution

`sim-soft`'s Phase I commitment. The algorithm convolves the lit-surface radiance (diffuse term) in screen space by the 1D radial profile $R_d(r)$ from [§1](01-bssrdf.md). The profile is made separable — decomposed as the sum of a small number of 1D Gaussians along two perpendicular axes — and the 2D convolution reduces to two 1D passes, each costing $2N_\text{taps}$ samples per pixel for a kernel half-width of $N_\text{taps}$.

The separable form is [Jimenez et al. 2015](../../appendices/00-references/04-rendering.md#jimenez-2015)'s contribution. A direct screen-space implementation of the full 2D convolution costs $N_\text{taps}^2$ samples per pixel; the separable decomposition drops that to $2 N_\text{taps}$, which for typical $N_\text{taps} \in [8, 16]$ is a 4–8× speedup on the inner loop. The technique has two earlier forms worth naming honestly: [d'Eon & Luebke 2007](../../appendices/00-references/04-rendering.md#deon-luebke-2007) ran the convolution in *texture* space (UV domain), working per-object with a sum-of-Gaussians approximation to $R_d(r)$; [Jimenez et al. 2009](../../appendices/00-references/04-rendering.md#jimenez-2009) moved the convolution into *screen* space, using depth-aware sampling to handle occlusion correctly without texture-atlasing; [Jimenez et al. 2015](../../appendices/00-references/04-rendering.md#jimenez-2015) then demonstrated that a single separable kernel matches the visual fidelity of the earlier sum-of-Gaussians fits at lower per-pixel cost. `sim-soft` inherits the 2015 form.

```wgsl
// Horizontal pass: 1D convolution along screen-space X
fn sss_horizontal(uv: vec2<f32>) -> vec3<f32> {
    var accum: vec3<f32> = vec3<f32>(0.0);
    for (var i: i32 = -N_TAPS; i <= N_TAPS; i = i + 1) {
        let offset_uv = uv + vec2<f32>(f32(i) * step_size, 0.0);
        let weight    = kernel_weight(f32(i));              // sum-of-Gaussians sample
        accum = accum + weight * textureSample(lit_diffuse, samp, offset_uv).rgb;
    }
    return accum;
}
// Vertical pass: same with (0.0, f32(i) * step_size) — commutes with horizontal.
```

`step_size` is parameterized by the per-pixel profile width, which itself is parameterized by the per-pixel thickness (from the [mesh-snapshot thickness attribute](../05-sim-bevy.md)) and the per-pixel material's diffusion profile (from the `DiffusionProfile` field on the [`MaterialField`](../../70-sdf-pipeline/00-sdf-primitive.md)). Thicker bodies get wider kernels — light has more material to scatter through; thinner features (rims, sharp edges) narrow the kernel toward the surface-BRDF limit (no scattering), matching the observed behavior at the diffusion approximation's breakdown below the material's mean free path.

Cost on the canonical 1080p scene with $N_\text{taps} = 12$: ≈2 ms for the convolution pair (horizontal + vertical pass), plus ≈0.2 ms G-buffer-read setup. [§0 stacked](00-stacked.md)'s variance-addition rule lets multi-layer stacks fold into a single compound kernel at shader-compile time, so a 2- or 3-layer silicone-skin-over-core render costs the same ≈2 ms as a single-layer body. This is the architectural payoff cited in the [parent chapter's §2](../00-sss.md): the middle tier is cheap enough to fit inside the Phase I 8–12 ms frame budget while delivering subjectively-indistinguishable output from path-traced ground truth on silicone-class materials.

## Tier (c) — Volumetric Monte Carlo (not shipped)

The ground-truth technique evaluates the BSSRDF integral from [§1](01-bssrdf.md) directly by Monte Carlo path tracing: sample an entry point on the surface, launch a photon into the material, propagate it through the volume with per-step scattering events drawn from the Henyey-Greenstein distribution parameterized by $(\sigma_s, \sigma_a, g)$, and accumulate surface-exit contributions. Quality is limited only by sample count; at tens of thousands of samples per pixel, the output is indistinguishable from a physical silicone photograph.

Cost is the problem. A 1080p image at 10k samples per pixel runs at minutes-per-frame on a modern GPU, completely outside the real-time budget. `sim-soft` does not ship a volumetric path tracer. Users who want ground-truth reference renders for publication or calibration export the current [`MeshSnapshot`](../05-sim-bevy.md) and the per-material $(\sigma_s, \sigma_a, g)$ triple to a dedicated renderer (Mitsuba 3, pbrt-v4, or Cycles), render offline, and compare.

The reference-render workflow exists as a named workflow on the [Phase I deliverable](../../110-crate/03-build-order.md) but runs outside `sim-bevy`'s pipeline. Tier (b)'s quality is validated against Tier (c) output periodically (a regression suite renders the same scene in both and compares; the [visual-regression test](../../110-crate/04-testing/02-visual.md) covers this) but the runtime does not invoke Tier (c).

## Cost comparison table

| Tier | Per-pixel cost | Kernel shape | Quality vs. BSSRDF ground truth | `sim-soft` role |
|---|---|---|---|---|
| (a) Normal-tinted | 1 tap | None; fixed offset | Fails on angle, size, stacking | Debug fallback only |
| (b) Separable screen-space | $\approx 2 N_\text{taps}$ taps | 1D separable (two 1D passes) | Subjectively equivalent on flat/curved silicone at canonical scale | **Phase I commitment** |
| (c) Volumetric Monte Carlo | $10^3$–$10^5$ paths | Full volumetric transport | Ground truth at high sample count | Not shipped; users export to external offline path tracer |

The jump from (a) to (b) is the one that matters: tier (b) is two 1D passes per layer-stack, stays inside the frame budget, and produces output that designers and viewers accept as "this is silicone." The jump from (b) to (c) is cost-to-validation — tier (c) is the reference (b) is regressed against, not the runtime commitment.

## Degradation and quality tiers

`sim-bevy`'s `SimSoftConfig` exposes an `sss_quality: SssQuality` setting with three values: `Debug` (tier a), `Production` (tier b default $N_\text{taps} = 12$), and `HighQuality` (tier b with $N_\text{taps} = 24$, ≈2× cost for finer tail accuracy). The default is `Production`. `HighQuality` is for close-up preview frames where the designer wants to see the profile's behavior on thin features; `Debug` is for performance diagnostics and the regression suite. Tier (c) is not in the enum — it is a separate offline workflow, not a runtime quality tier.

Under [adaptive-$\Delta t$ shrink events](../../50-time-integration/02-adaptive-dt.md), the renderer does not drop tiers — physics slowdowns are decoupled from render rate per [Ch 05's double-buffered streaming](../05-sim-bevy.md), so the renderer continues at `Production` quality while the physics catches up.

## What this sub-leaf commits the book to

- **Three tiers, one shipped.** Tier (b) — separable screen-space diffusion-profile convolution per [Jimenez et al. 2015](../../appendices/00-references/04-rendering.md#jimenez-2015) — is the Phase I commitment; tier (a) is a debug fallback; tier (c) is an offline workflow, not a runtime tier.
- **The separable decomposition is the engineering payoff.** Full 2D convolution at $N_\text{taps}^2$ taps per pixel is too slow; two 1D passes at $2 N_\text{taps}$ each stays inside the frame budget without visible quality loss on silicone-class materials.
- **Kernel width is per-pixel, driven by thickness + material `DiffusionProfile`.** Thicker pixels get wider kernels; thin features narrow toward the surface-BRDF limit, matching the diffusion approximation's breakdown at sub-mean-free-path thickness.
- **Tier (c) ground truth is for validation, not runtime.** The [visual-regression suite](../../110-crate/04-testing/02-visual.md) periodically compares tier (b) against an external path-tracer render; a persistent divergence is a bug, not a quality setting.
