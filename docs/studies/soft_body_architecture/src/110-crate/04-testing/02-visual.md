# Visual regression

Visual regression catches the class of bug that lives on the rendering side of the physics↔render bridge: a shader that silently computes the wrong roughness value, a per-vertex attribute whose byte ordering flipped during a refactor, a subdivision pass that now produces 32-bit indices instead of 16-bit, a thermal-overlay color map that shifted two stops darker. None of those show up in unit tests (shaders have no spec to unit-test against), regression-vs-flex ([§01](01-regression.md)) does not see them (it compares `Observable` outputs, not rendered pixels), and gradcheck ([§03](03-gradcheck.md)) cares about derivatives, not colors. The remaining oracle is prior rendered output — a fixed set of golden images, produced once after each intentional rendering-pipeline change, compared pixel-by-pixel against every subsequent build.

This is the test class most tightly scoped to the [Phase I visual layer](../03-build-order.md#the-committed-order). It does not activate before Phase I and it does not run on any PR that does not touch [`readout/`](../00-module-layout/09-readout.md) or the [`sim-bevy` coupling](../02-coupling/02-bevy.md).

## What the harness runs against

The test renders a fixed scene set through the full [`SimSoftPlugin`](../../90-visual/05-sim-bevy.md) pipeline — [Part 9 Ch 05 §00 mesh-streaming](../../90-visual/05-sim-bevy/00-mesh-streaming.md) pushes a `MeshSnapshot` through the double-buffer, the render graph reads the active buffer, the five-pass shader pipeline from [Part 9 Ch 05 §01](../../90-visual/05-sim-bevy/01-shader-pipeline.md) composites the frame — and compares the resulting framebuffer against the stored golden image.

The scene set is deliberately small. Too many scenes dilute the signal — every frame renders against GPU floating-point quirks that vary across drivers and vendors, so tolerances have to loosen as the scene count grows, and the harness starts catching driver drift instead of `sim-soft` drift. The Phase I shipping target is ten scenes, enough to exercise each pass of the pipeline from [Part 9 Ch 05 §01](../../90-visual/05-sim-bevy/01-shader-pipeline.md) at least once:

| Scene | What it exercises |
|---|---|
| Canonical cavity + probe at zero squeeze | Baseline: geometry pass, default [SSS profile](../../90-visual/00-sss.md), no contact pressure, no thermal load |
| Canonical + probe at mid-squeeze | Adds `contact_pressure` field into the [micro-wrinkle pass](../../90-visual/02-micro-wrinkles.md); [thickness field](../../90-visual/00-sss/02-realtime.md) varies as the cavity deforms |
| Canonical + probe at full wrap | Exercises [peak-pressure](../../10-physical/01-reward/02-peak-bounds.md) rendering and the wrinkle shader near barrier saturation |
| Two-layer sleeve at mid-squeeze | [SSS multi-layer stack](../../90-visual/00-sss/00-stacked.md) — layer-ID field keys into the per-material profile lookup |
| Anisotropic-fiber sleeve at mid-squeeze | [GGX anisotropic specular](../../90-visual/01-anisotropic-reflection/01-ggx.md) reads `anisotropy_dir` from the snapshot |
| Wet-surface variant of the sleeve | [Contact-flow anisotropy](../../90-visual/01-anisotropic-reflection/00-wet.md) uses contact-pressure gradient to orient the tangent axes |
| Thermal-overlay on mid-squeeze | [Thermal overlay pass](../../90-visual/04-thermal-viz.md) active; `temperature` field non-trivial via [`sim-thermostat` coupling](../02-coupling/01-thermostat.md) |
| Subdivided-surface test scene | [Loop subdivision](../../90-visual/03-subdivision.md) one level applied; tests render-mesh-is-physics-mesh thesis at the topology boundary |
| Adaptive-timestep-shrink freeze | Renderer pauses on a slow physics step per [Part 9 Ch 05 §00](../../90-visual/05-sim-bevy/00-mesh-streaming.md); the render is the previously-complete snapshot, no interpolation |
| Post-release wobble at +50 ms | Phase H Prony viscoelasticity active; tests that the viscoelastic material path composites identically to the elastic path |

Each scene renders to a fixed resolution (1920×1080), a fixed camera pose, a fixed lighting rig, and a fixed [`SimSoftConfig`](../02-coupling/02-bevy.md). The scenes are authored in code, stored in the repository, and produced via a deterministic offline render pass keyed off the `MeshSnapshot` the physics delivers at a fixed episode-step — not via capture from an interactive run.

## The comparison metric

Framebuffer-level equality is not the test. GPU floating-point output varies across drivers and across vendors; a byte-exact image match would fire on every driver upgrade, for reasons unrelated to `sim-soft`. The harness uses a calibrated perceptual metric:

- **Per-pixel L1 in sRGB** with a cutoff: any single pixel that moves by more than `0x04` in any channel counts as changed. Most shader drift crosses this cutoff; most driver drift does not.
- **Changed-pixel fraction**: the fraction of pixels crossing the cutoff, taken over the whole framebuffer. The failure threshold is 1% — a number calibrated during Phase I against driver-swap runs on the baseline golden set.
- **Structural-similarity (SSIM) backstop**: if per-pixel L1 is close to threshold, SSIM below 0.98 is an independent trip-wire for geometric drift (mesh topology or subdivision producing a different surface) that per-pixel L1 can miss on smooth gradient shifts.

Both numbers are reported on every run. A pass requires both to be under threshold. A failing test writes a diff image (per-pixel L1 heat map + SSIM overlay) into the CI artifact bucket so the failure mode is visually inspectable.

The `02-visual` harness does not attempt to work around driver drift. If a driver upgrade pushes the baseline over threshold on scenes that did not change, the failure is investigated, the visual change is confirmed benign, and the goldens are re-captured on the new driver — a controlled event, not silent. Re-capture is an explicit commit with its own review, not an automated "regenerate goldens" button.

## Upstream binding — `MeshSnapshot` and `Observable`

The `MeshSnapshot` from [Part 9 Ch 05 §00](../../90-visual/05-sim-bevy/00-mesh-streaming.md) is the input surface this harness exercises. Every field in the snapshot maps to a shader input, and every field is produced by the [`Observable` trait](../01-traits/00-core.md) from [`readout/`](../00-module-layout/09-readout.md): `stress_field` → `stress_per_vertex`; `temperature_field` → `temperature`; `pressure_field` at vertices → `contact_pressure`; `thickness`, `layer_id`, `anisotropy_dir`, `vertices`, `normals` from the surface-extraction pass owned by `readout/` and `sim-bevy`. A change to any `Observable` impl that perturbs a per-vertex output is visible at the rendered pixel; the visual regression is therefore an integration test of the `readout/ → sim-bevy → shader-pipeline` path from the upstream end.

A direct consequence: a PR that silently alters `Observable::stress_field`'s scalar definition will be caught by this harness even if no shader file changed, because the per-vertex values fed to the diagnostic-render pass will shift. This is exactly what [parent Ch 04](../04-testing.md) meant by "rendering-pipeline drift" covering the Observable-to-pixel chain.

## Cadence and Phase landing

Per the [parent Ch 04 spine](../04-testing.md), the visual-regression suite runs on every PR that touches `readout/` or `sim-bevy`. There is no weekly run — the suite is gated on those paths exclusively, because the golden set is meaningless if the physics has changed without the shader having changed (the regression-vs-flex path in [§01](01-regression.md) catches that class of drift at an earlier layer).

Phase landing follows the [build-order visual milestone](../03-build-order.md#the-committed-order):

- **Phase A–H: no visual regression.** The shader pipeline is not authored yet, and none of the early phases have rendered output worth regressing. Per [parent build-order commentary](../03-build-order.md), Phase D explicitly ships no shader preview; a Phase D "visual regression" would regress against a deliberately-ugly Tet4-only render and catch nothing meaningful.
- **Phase I: activation.** The first golden set is captured at Phase I landing against the committed scene list; all ten scenes go green as part of the Phase I shipping criterion. The suite then binds every `readout/` or `sim-bevy` PR going forward.

## What visual regression does not test

- **Physics correctness.** A shader that renders a physically-wrong simulation the same way every time still passes this harness. Physics correctness lives in [unit](00-unit.md), [regression](01-regression.md), and [gradcheck](03-gradcheck.md); this harness catches drift, not wrongness.
- **Cross-vendor portability.** The goldens are captured on one reference driver. Running the suite on a different vendor will produce expected driver drift; running it cross-vendor is not part of the Phase I deliverable.
- **Interactive performance.** Frame-rate regression is a separate instrumentation concern — [Phase E's 30+ FPS target](../03-build-order.md#the-committed-order) and Phase I's visual target are measured by different harnesses, not this one.
- **Physical-print visual comparison.** Comparing rendered `sim-soft` output against photographs of printed prototypes — the closure step of [Part 10 Ch 05's sim-to-real loop](../../100-optimization/05-sim-to-real.md) taken to the visual axis — is a [post-Phase-I concern](../../120-roadmap/07-open-questions.md). The oracle there is a `MeasurementReport`-analogue for visual data (calibrated photographs with known illumination), which [Phase I does not ship](../03-build-order.md#the-committed-order). When that harness lands, it extends this one rather than replacing it.
- **Custom-user-shader pipelines.** [Part 9 Ch 05 claim "What this does NOT commit"](../../90-visual/05-sim-bevy.md) flags that user-authored Bevy `Material` overrides bypass the default `SimSoftPlugin` pipeline; a user shader has no golden. Coverage for user-bypass scenes is a user concern, not a `sim-soft` deliverable.

## What this sub-leaf commits downstream

- **[`SimSoftPlugin`'s shader pipeline](../02-coupling/02-bevy.md)** is what the goldens record; changes to the plugin's pass structure, render-graph node placement, or composition order require a deliberate golden re-capture with documented rationale.
- **[`MeshSnapshot` field additions](../../90-visual/05-sim-bevy/00-mesh-streaming.md)** require a corresponding scene in the golden set that exercises the new field; a field added without a scene in which it meaningfully varies is an untested shader input, which is the class of silent drift this sub-leaf exists to catch.
- **[Phase I shipping criterion](../03-build-order.md#the-committed-order)** includes passing all ten golden-scene comparisons against the at-landing reference captures. A Phase I PR that regresses any scene blocks merge; benign regressions re-capture the golden as an explicit commit, not a CI override.
