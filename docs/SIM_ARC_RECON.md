# cf-device-design Fit-Visualization + Scoring — Sim Arc Recon

**Status**: RECON v1, no implementation yet. This doc replaces the stale-by-2-days slice 7 memo + the fit-viz bookmark with a single source of truth grounded in empirical screenshots from the user's actual iter-1 design (2026-05-18). The user's framing: "the sim is the final part of this entire workflow we need to do."

**Reading order for the next session**: §1 empirical baseline first (it overturns several stale memo claims), §3 user vision second (the target), §4 gap table third (the work), §5 decisions fourth (resolve before §6), §6 sub-leaf ladder fifth (the execution plan).

---

## 1. Empirical baseline — what actually ships on dev today

Captured 2026-05-18 by booting cf-device-design with iter-1 inputs (`~/scans/sock_over_capsule.cleaned.stl` + `.design.toml` + `.prep.toml`) and clicking the existing **[▶ Simulate Insertion]** button.

### 1a. 2-layer design (Ecoflex 00-30 10mm + Dragon Skin 20A 3mm, 3mm cavity inset)

| Metric | Value |
|---|---|
| Convergence | **16 of 16 steps — seated to 3.00 mm (FULL depth)** |
| Peak contact force | 123.5 N at 3.0 mm |
| Layer 0 (Ecoflex, 48,385 tets) | mean ψ = 3.61 kJ/m³, max ‖P‖ = 130 kPa, **λ range 0.40…1.66** |
| Layer 1 (Dragon Skin, 24,500 tets) | mean ψ = 0.74 kJ/m³, max ‖P‖ = 222 kPa, λ range 0.71…1.27 |
| Wall clock | ~30–60 s release per the slice 7 memo + agent timing |
| F-d curve | smooth monotonic, no discontinuities, polynomial-looking Yeoh response |
| Heat map | visible but subtle white/blue patches on outer Dragon Skin shell |

**Inner layer absorbs ~5× the energy density** of the outer; the soft-inner/stiff-outer design is doing what it should — Ecoflex sponges the deformation, Dragon Skin constrains lateral spread.

### 1b. 1-layer ablation (Ecoflex 00-30 10mm only, 3mm cavity inset)

| Metric | Value |
|---|---|
| Convergence | **13 of 16 steps — seated to 2.44 of 3.00 mm (partial; ~81% envelope)** |
| Peak contact force | 115 N at 2.44 mm |
| Layer 0 (52,840 tets) | mean ψ = 2.86 kJ/m³, max ‖P‖ = 168 kPa, λ range 0.43…1.63 |
| Stall mode | Armijo line-search stall on non-SPD tangent (per slice 7 memo's documented mode) |

**Design-validation finding**: the stiffer outer Dragon Skin layer is what enables full-depth convergence on iter-1. Without it, the sim stalls at ~80%. Sleeve-style 2-layer designs are reaching the FULL inset; single-layer wins reach ~80%; deeper insets or single-stiff would behave differently.

### 1c. Stale-memo corrections discovered

| Memo claim | Empirical reality |
|---|---|
| "Slice 7 STRUCTURAL ARC COMPLETE through 7.3b.2; **7.4 + 7.5 banked, not started**" | **7.4 SHIPPED** (`insertion_sim_ui.rs`, 1,144 LOC: button, async task via `AsyncComputeTaskPool`, F-d plot, step/layer tables, heat-map projection). **7.5 partial**: slacker → effective-Shore SHIPPED (`SimDesign.layers[i].slacker_fraction` resolves to `SiliconeMaterial::from_effective_shore` per `insertion_sim.rs:440`); **deformed-mesh-viz NOT shipped**. |
| "Real iter-1 scan ramp seats 0.94 mm (5/16 steps, ~31%)" | True for **single-layer Ecoflex**; false for the **actual user design** (2-layer reaches 16/16 / 3.00 mm / 100%). |
| "Slice 7.1 drops `slacker_fraction` from `SimDesign`" | Reverted in slice 7.5; `SimLayer` carries `slacker_fraction`, threaded into per-tet Yeoh material via `from_effective_shore`. |
| "`mod insertion_sim` is `#[cfg(test)]`-gated" | Module is `pub` at the bin scope; consumed by `insertion_sim_ui`. The cfg-gating mentioned in memory was a single `use sim_soft::ContactPair;` import on a test-only path. |

**Action**: cold-read pass at end of this arc should rewrite the slice 7 memo + fit-viz bookmark memo to point at this doc as the new source of truth.

---

## 2. Architectural thesis

cf-device-design's FEM insertion sim has shipped the **engine + half of the UI surface**: the user gets a F-d curve, per-layer stress/stretch tables, and a static heat map projected onto rest geometry. What's missing is the **visual fit story** (seeing the actual squish over time) and the **quantitative fit story** (scoring whether the fit is snug, crushing, or floating). The user vision per the fit-viz bookmark frames these as load-bearing TOGETHER: the visual story tells you "what's happening," the quantitative story tells you "is it good." Both are needed to close the engineering loop the rest of the tool is built around.

The shipped engine produces enough output to drive both stories — `RampStep.x_final` per step + `compute_tet_readouts` on-demand for any step + `PenaltyRigidContact::per_pair_readout` for pressure — but **the data → render + data → score paths aren't wired up**. This arc is rendering-side + scoring-side glue, not new FEM physics.

**Why now**: the workshop iter-1 print + cast lands the physical device; before then, the sim is the only feedback channel for "is the design choice good." After the physical cast lands, the sim becomes the design-iteration loop (try N designs in sim, cast the winner). Either way, the sim's user-facing surface needs both stories for the design choice to be reviewable.

---

## 3. End-state definition — user vision (verbatim from `project_cf_device_design_fit_viz_bookmark`)

> the goal is to basically slide the scan geometry into the cavity, and find the best possible fit for it. like if the main geometry is a sensor going in, snug pressure against it is a reward, but overly tight/hard/overly rigid collision is negative points. no collision at all for an area is also negative points (maximum good pressure of all surface area at all time basically, throughout the insertion and retraction)

**Interpretation** (six end-state requirements):

| # | Requirement | Why |
|---|---|---|
| E1 | **Animate insertion + retraction** in the viewport, scrubbable by step. | "throughout the insertion and retraction" |
| E2 | **Render the scan geometry as the rigid intruder** descending into the cavity. | "slide the scan geometry into the cavity" |
| E3 | **Render the deformed cavity walls** matching the per-step solver state. | "see the squish" / what FEM is actually computing |
| E4 | **Per-cavity-vertex pressure → goodness band** (snug ≠ crushing ≠ floating) → per-vertex color on the cavity surface | "snug pressure = reward, crushing = penalty, floating = penalty" |
| E5 | **Aggregate fit score over the entire ramp** (coverage × pressure-in-band × time, summed insertion + retraction) | "maximum good pressure of all surface area at all time" |
| E6 | **Design comparison support** — given two `.design.toml` files, run both, compare scores | "find the best possible fit" — implies sweeps; deferred to a future "auto-search" rung |

E1–E5 are the core arc. E6 is the fit-viz bookmark's rung 6 ("optional auto-search") and stays banked.

---

## 4. Gap table

Each row is one feature delta between end-state and current state. Effort sized for cold-read implementation; the order is sequencing-driven, not effort-driven.

| # | Feature | End-state | Current state | Disposition | Sub-leaf | Effort |
|---|---|---|---|---|---|---|
| G1 | Per-step playback timeline | scrubbable per-step UI control; viewport reflects selected step | only final-step shown; per-step table collapsed; no slider | NEW UI control on `InsertionSimState` (`displayed_step: Option<usize>`); plumb into mesh-rebuild change-detection | S1 | ~1 hr |
| G2 | Deformed cavity surface render | cavity mesh re-extracted from `RampStep.x_final` of `displayed_step` | cavity rendered from `CachedScanSdf` (rest geometry); heat-map projects per-tet scalars ONTO rest geometry | NEW pipeline: per-step `IndexedMesh` extraction from FEM tet positions → existing `build_bevy_mesh_from_indexed_with_colors`. Use sim-soft's `viz::boundary_surface` or roll our own surface extractor over BCC tets. | S2 | ~2-3 hr |
| G3 | Deformed layer shells render | layer shells track `displayed_step`; heat-map colors follow | layer shells from `CachedScanSdf` rest extraction; heat-map projection only valid at final step | extend G2's deformed extractor to per-layer (split tet set by material id); per-step `compute_tet_readouts` for heat-map | S3 | ~1-2 hr (depends on G2) |
| G4 | Scan-as-intruder render | scan mesh rendered as separate entity; translated along centerline by `interference_m` per `displayed_step` | scan rendered at rest only (the existing scan mesh entity); no intruder motion | NEW entity + per-step transform; load scan mesh once, translate by `step.interference_m × intrude_axis`. Axis = -centerline tangent at the cavity opening per cap plane. | S4 | ~1 hr |
| G5 | Pressure-band scoring | per-cavity-vertex pressure score; snug/crushing/floating classifier | no scoring; only raw force totals | NEW `score_step(per_pair_readout, cavity_vertex_set) → PressureScoreField`; classify per vertex via (low_thresh, high_thresh) band; emit per-vertex color | S5 | ~2-3 hr |
| G6 | Retraction ramp | run ramp backward: full insertion (N steps) → full retraction (N more steps) | only forward ramp exists | EXTEND `run_insertion_ramp` to optionally chain a reverse ramp (intruder retreats N → 0); store as additional `steps`. Quasi-static so reverse is just relabeling interference. | S6 | ~1 hr |
| G7 | Over-cycle aggregate score | scalar summary: ∫(coverage × in-band-pressure) over (insert+retract) cycle | none | NEW `OverCycleScore` aggregated post-ramp; surfaced in panel | S7 | ~1 hr |
| G8 | Per-layer Slacker UI slider | per-layer `slacker_fraction` editable from Layers panel | stored in `LayerSpec`, threaded to sim, but **no UI control to set it per-layer** (must be set via design.toml on-disk + reload) | NEW dropdown/slider in `render_layers_section`; snap to TB curve points via existing `resolve_slacker_fraction` | S8 | ~30 min |
| G9 | Heat-map contrast | visible color variation across the layer surface | white/blue patches barely readable on the iter-1 screenshot | TUNE existing `project_layer_heat_map` to use divergent colormap (e.g., turbo or viridis) + per-layer normalization | S9 | ~30 min |
| G10 | Convergence warning when partial-ramp | clear "this is partial" callout above F-d plot | text exists but easy to miss; F-d plot shows only what converged (looks complete) | TUNE existing `render_convergence_summary` to add visual marker on F-d plot at the seated depth + amber dashed line for target depth | S10 | ~30 min |

**Total effort**: ~11-14 hr. Sequencing-friendly (S1+S2+S3 unlock G4+G5+G6+G7; S8+S9+S10 are independent polish).

---

## 5. Decisions — surface for user input

### Closed (default-pickable per [[feedback-autonomous-architecture]])

- **D1 (deformed surface extraction strategy)**: use sim-soft's `viz::boundary_surface` if it accepts an externally-supplied deformed-positions array, OR roll our own BCC-tet surface extractor (extract triangles where exactly one of the 4 vertices is outside the body, take the matching face). **Default**: try sim-soft's existing primitive first (less code; the row-23 example already does this); fall back to a custom extractor only if the API doesn't accept arbitrary positions.
- **D2 (per-step heat-map cost)**: re-running `compute_tet_readouts` on every step change costs ~O(n_tets) per scrub event (~100ms at 73k tets). **Default**: cache per-step readouts on first scrub of each step (`HashMap<usize, Vec<TetReadout>>` on `InsertionSimState`). Memory ceiling: ~184 B × 73k tets × 16 steps ≈ 215 MB worst case — acceptable for design-review workflow.
- **D3 (retraction physics fidelity)**: quasi-static reverse-ramp is just relabeling — true retraction would solve a hysteresis cycle (path-dependent material response). **Default**: treat retraction as reverse-quasi-static (no hysteresis); document explicitly. Yeoh's load/unload is path-independent in the quasi-static limit anyway. Real visco-elastic hysteresis is a sim-soft v2 sub-arc, not this one.
- **D4 (intruder motion axis)**: scan translates along cavity-mouth normal (- cap-plane outward normal) by `interference_m`. **Default**: use cap-plane outward normal from `cf_cap_planes::CapPlane`; iter-1 has a single cap so this is unambiguous. Multi-cap designs would need a primary-cap convention.

### Open (need user input before S6+ start)

- **D5 (scoring band thresholds)**: snug-pressure band [low_kPa, high_kPa] — what are the numbers? The fit-viz vision says "snug ≠ crushing ≠ floating" but doesn't quantify. Workshop guess: low = 5 kPa (gentle contact), high = 100 kPa (firm but not bruising). User decides; defaults shipped + slider for tuning.
- **D6 (score aggregation formula)**: many shapes possible (mean coverage × in-band ratio × time? Penalize floating + crushing equally? Asymmetric penalty?). Default proposal: per-step score `s_k = (1/N_cavity) × Σ_v in_band(p_v)` where `in_band(p) ∈ [0, 1]` is a smooth tent over [low, high]; over-cycle score `= mean(s_k)` across insert + retract. User-editable formula? Or fixed?
- **D7 (UI surface for displayed-step scrub)**: timeline slider in the panel? Bevy viewport overlay? Both? **Recommend**: panel slider (consistent with existing UI) + a small frame number badge in the viewport.
- **D8 (PLY/JSON export of ramp + scores)**: should the [Simulate Insertion] button optionally save a sequence of PLYs (one per step) + a JSON scoring record to disk, for cf-view scrub + external analysis? Mirrors row-23's output shape. Adds ~30 min to S2/S3 if shipped together. **Recommend**: yes, optional checkbox in panel ("Save PLY sequence on completion").

### Settled by empirical baseline (no action needed)

- 2-layer iter-1 design reaches full 3mm — sim envelope is workshop-adequate; no need to push solver robustness this arc.
- Slacker is wired through; no UI-slider-to-sim integration work needed beyond G8 (surface the existing wiring).
- Heat-map data IS computed on final step; the gap is visualization layering (G2/G3) and per-step access (G1).

---

## 6. Sub-leaf ladder

Sequenced for empirical validation between sub-leaves. Each sub-leaf is independently shippable + user-reviewable in cf-view OR the egui app.

### S1 — Per-step playback timeline (G1) [~1 hr]

- Add `displayed_step: Option<usize>` to `InsertionSimState`. `None` = final step (current behavior).
- Add `egui::Slider` in panel (visible only when `last_run.is_some() && steps.len() > 1`).
- Add to `LayerMeshKey` change-detection so mesh rebuild triggers on step change.
- Layer/cavity meshes STILL extract from `CachedScanSdf` (rest geometry); only heat-map per-vertex colors change per step.

**Visual gate**: scrub through steps 0..N-1; heat-map colors smoothly transition from low (rest) to high (full insertion).

### S2 — Deformed cavity surface render (G2) [~2-3 hr]

- New `extract_deformed_cavity_mesh(geometry, x_final, layer_index=cavity)` helper.
- Investigate `sim_soft::viz::boundary_surface` for arbitrary-position support; fall back to custom BCC-tet surface extractor if it doesn't fit.
- Wire into `update_cavity_mesh` system — replace rest-from-SDF extraction with deformed-from-tets when `last_run.is_some() && displayed_step.is_some()`.
- "Show deformed" checkbox in panel (defaults ON when result available).

**Visual gate**: at step N-1, cavity surface visibly bulges outward from the rest position; heat-map shows pressure hot spots on the deformed surface.

### S3 — Deformed layer shells render (G3) [~1-2 hr]

- Extend S2's extractor per-layer (split tet set by material id, extract each layer's outward face).
- Wire into `update_layer_meshes` system.

**Visual gate**: at step N-1, all layer shells deformed consistently with the cavity; outer Dragon Skin layer's deformation visibly subtler than inner Ecoflex's.

### S4 — Scan-as-intruder render (G4) [~1 hr]

- New `IntruderEntity` resource + render system.
- Load cleaned scan mesh once (already loaded for the cavity SDF — share the Arc).
- Per displayed_step: translate scan by `step.interference_m × cavity_inward_normal`.

**Visual gate**: at step 0, scan sits at the cavity mouth, no overlap. At step N-1, scan is fully inserted (visibly overlapping with the cavity surface IF deformed cavity is shown — that's the "squish" overlap representing the press-fit).

### S5 — Pressure-band scoring (G5) [~2-3 hr; requires user D5 input]

- New `score_pressure_field(per_pair_readout, cavity_vertex_set, low, high) → PressureScoreField`.
- Per-vertex pressure = ‖force_on_soft‖ / vertex_area_estimate. Vertex area = (1/3) × sum of incident triangle areas in the cavity surface mesh.
- Tent function: `in_band(p) = clamp((p - low)/(span/4), 0, 1) × clamp(1 - (p - high)/(span/4), 0, 1)`.
- Per-vertex color: divergent map (red below low = floating, green in band = snug, blue above high = crushing).
- Toggle in panel: "Pressure scoring" (replaces heat-map when ON).

**Visual gate**: cavity surface colored red (no contact) → green (snug) → blue (crushed) bands, varying spatially around the cavity.

### S6 — Retraction ramp (G6) [~1 hr; requires D3]

- Extend `run_insertion_ramp` with optional `n_retraction_steps` param.
- Steps N..2N-1 reverse-walk `interference_m` from `cavity_inset_m` back to 0.
- Warm-start from prior step's `x_final` (same as insertion).
- `RampStep.phase: Insert | Retract` field for downstream UI/scoring.

**Visual gate**: scrub from 0 to 2N-1; first half = inserting, second half = retracting; deformed cavity smoothly returns to rest by step 2N-1.

### S7 — Over-cycle aggregate score (G7) [~1 hr; requires D6]

- New `compute_over_cycle_score(steps, score_field_per_step) → OverCycleScore`.
- Aggregate per-step coverage × in-band-pressure → scalar.
- Surface in panel below F-d plot: "Fit score: 0.74 (insert avg) / 0.71 (retract avg) / 0.73 (over-cycle)".

### Polish sub-leaves (independent, ship anytime)

- S8 — per-layer Slacker UI slider (G8) [~30 min] — surfaces existing wiring; small UX win.
- S9 — heat-map contrast tuning (G9) [~30 min] — divergent colormap + per-layer normalization.
- S10 — partial-ramp visual warning (G10) [~30 min] — amber dashed target line on F-d plot.

### Suggested execution order

S1 → S2 → S3 → S4 (visual squish story, ~5-7 hr) → S5 (D5 needed) → S6 → S7 (D6 needed) → S8 + S9 + S10 (polish).

**Ship breakpoints**: after S4 the user has the full visual story (animation + deformed mesh + intruder); a stop here is a complete arc and the workshop user can compare designs visually. S5–S7 add the quantitative story; S8–S10 are usability polish.

---

## 7. Fragility callouts

1. **Per-step memory ceiling** (D2). At 73k tets × 16 steps × ~184 B = 215 MB worst case for cached per-step readouts. Acceptable but worth surfacing; a future deeper-cell-size design at 200k tets × 32 steps would be 1.2 GB. Mitigation: LRU cache the K most-recently-scrubbed steps.
2. **Deformed mesh extraction performance**. Per-step extraction at 73k tets via marching-cubes-equivalent on a deformed grid is O(n_tets); ~100-500 ms. UI scrub responsiveness needs caching or background extraction.
3. **Bevy mesh thrashing on scrub**. The existing `LayerMeshKey` change-detection rebuilds the WHOLE layer mesh on any state change. Scrubbing through steps would rebuild every layer at every scrub event. Consider: extract deformed meshes once at sim-completion + cache as `Vec<Mesh>` keyed by step.
4. **2-layer design hides the depth envelope cliff**. The iter-1 2-layer reaches 16/16; a different design (single-stiff, very-soft-only, deep inset) might stall. The partial-ramp warning (S10) is important for trust.
5. **Pressure band thresholds (D5) drive all visual scoring**. Bad defaults will make every design look "all floating" or "all crushing." Recommend shipping a default + slider for tuning before the workshop user trusts the scoring.
6. **Quasi-static retraction is not real**. Document explicitly so users don't expect hysteresis behavior. The path-independent Yeoh limit is mathematically correct but a real silicone has visco-elastic memory.
7. **Single-cap convention for intruder axis (D4)**. iter-1 is single-cap so unambiguous; future multi-cap designs need a "primary cap" convention.
8. **Slice 7 memo is now stale**. Doesn't reflect 7.4 shipped, 7.5-slacker shipped, real-scan 2-layer envelope reaches full inset. Rewrite at end of this arc.

---

## 8. Banked research links

- [[project-cf-device-design-slice-7-plan]] — original FEM insertion sim plan + as-built notes through 7.3b.2. **Now stale**; this doc supersedes for the user-facing arc.
- [[project-cf-device-design-fit-viz-bookmark]] — original fit-viz 6-rung ladder; user vision verbatim. This doc realizes rungs 1-5; rung 6 (auto-search) stays banked.
- `examples/sim-soft/scan-fit-3layer-sleeve-yeoh-ramp/` — canonical multi-layer Yeoh ramp; production-quality readout extraction pattern (row 22/23). The `viz::design_surface_deformed(amplify=1.0)` call pattern is the reference for S2/S3.
- `sim/L0/soft/src/viz/mod.rs` — `boundary_surface`, `slab_cut`, `design_surface_deformed` primitives. Investigate API surface in S2.
- `cf-viewer/src/sequence.rs` — `Sequence` + `Playback` resources, frame-by-frame PLY scrub. Reference for S1 UI affordance + D8 PLY export path.
- `tools/cf-device-design/src/insertion_sim_ui.rs` — slice 7.4 shipped panel; the canonical "Run X / wait / display result" pattern in cf-device-design.
- `tools/cf-device-design/src/insertion_sim.rs:1452+` — `RampStep`, `StepReadout`, `TetReadout`, `InsertionResult`, `compute_tet_readouts`. All data needed for S1-S7 is here.
- `sim/L0/soft/src/contact/penalty.rs:140+188` — `PenaltyRigidContact::per_pair_readout` + `filter_pair_readouts_to_referenced`. S5 source of pressure data.
- [[project-cf-device-design-sdf-layers]] — patterns banked for the existing layer-mesh rendering pipeline; relevant for S2/S3 mesh rebuild change-detection.
- [[feedback-bookmark-when-surface-levers-exhaust]] — three-session pattern (bookmark / recon / implementation). This doc is the recon output; the next session is implementation.
- [[feedback-autonomous-architecture]] — drives the default-pick posture on D1-D4.
- [[feedback-cf-cast-tests-use-release]] — `--release` posture; FEM solves are 10-100× slower in debug.

---

## 9. Cold-read entry for next session

1. **Read §1 empirical baseline first**. The stale-memo corrections at §1c are load-bearing — don't trust the slice 7 memo's "7.4 + 7.5 banked" claim or the "0.94mm stall" envelope; the 2-day-old memo is wrong on both. The actual baseline: 2-layer iter-1 design reaches 16/16 / 3mm cleanly, slice 7.4 has shipped, slacker → effective-Shore is wired.

2. **Read §3 user vision second** (verbatim). The vision is "see the squish + score the fit"; arc is rendering glue + scoring glue, not new FEM physics.

3. **Read §6 sub-leaf ladder third**. Suggested execution: S1 → S2 → S3 → S4 ships the visual story (~5-7 hr); user can stop here and have a complete arc. S5–S7 add scoring (~4-5 hr); S8–S10 polish (~1.5 hr). Total ~11-14 hr.

4. **Resolve open decisions D5–D8 with user before starting S5**. D5 (pressure thresholds) and D6 (score formula) are the load-bearing-for-scoring decisions; D7 + D8 are UX preferences.

5. **Implementation start**: S1 is a ~1 hr UI-only change in `insertion_sim_ui.rs` (add slider + plumb to mesh-rebuild key). Lowest-risk first sub-leaf to validate the recon's empirical baseline + the UI integration approach.

6. **Per [[feedback-implement-measure-revert-pattern]]**: between sub-leaves, run the [Simulate Insertion] button on iter-1 + visually verify the new feature works on real data. Per [[feedback-cold-read-review-post-ship]]: after each sub-leaf, cold-read the diff in-session.

7. **At end of arc**: rewrite [[project-cf-device-design-slice-7-plan]] + [[project-cf-device-design-fit-viz-bookmark]] memos to point at this doc as the new source of truth. Mark this doc SHIPPED. New patterns banked from the arc go into a fresh memo.

---

## Acceptance criteria

Visual story (S1–S4):
- [ ] S1: per-step scrub slider works; heat map updates per step.
- [ ] S2: cavity surface visibly deforms per displayed step.
- [ ] S3: layer shells deform consistently; per-layer per-step heat-map valid.
- [ ] S4: scan visible in viewport, translates along intruder axis per step.

Quantitative story (S5–S7):
- [ ] S5: pressure-band coloring shows red/green/blue regions on cavity surface.
- [ ] S6: retraction ramp runs; scrub through 0 → 2N-1 covers full insert + retract cycle.
- [ ] S7: over-cycle scalar score displayed in panel; insert/retract/combined breakdown.

Polish (S8–S10):
- [ ] S8: per-layer Slacker fraction slider in Layers panel.
- [ ] S9: heat-map contrast tuned; iter-1 visibly shows variation, not subtle patches.
- [ ] S10: partial-ramp warning is visually prominent on F-d plot.

Cross-arc:
- [ ] Slice 7 + fit-viz bookmark memos rewritten / annotated.
- [ ] User-verified visual gate on iter-1 after S4 (visual story complete) AND after S7 (quantitative story complete).
- [ ] Per-sub-leaf cold-read pass per [[feedback-cold-read-review-post-ship]].
