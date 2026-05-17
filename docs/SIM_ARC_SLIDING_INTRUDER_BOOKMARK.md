# cf-device-design Sim Arc — Sliding-Intruder Pivot Bookmark

**Status**: BOOKMARK 2026-05-18 LATER. The S11 visual-rewrite arc shipped S11.1 (FEM-accurate per-step intruder) + S11.2 (layer slab translucency); the user's visual gate on S11.2 surfaced that the underlying FEM model itself is **wrong-fit for the actual product intent**, not just the rendering. This doc captures the pivot and the recon entry points; next session = recon to design the sliding-intruder FEM rewrite.

**Reading order**: §1 (what shipped this session) → §2 (the pivot — verbatim user ask + diagnosis) → §3 (why this is bigger than S11.3) → §4 (what survives) → §5 (recon entry points + decisions to resolve) → §6 (three-session cadence).

---

## 1. What shipped this session

| # | Commit | Headline |
|---|---|---|
| S11.1 | `f2a64195` | FEM-accurate per-step intruder via MC of `geometry.intruder` sampled into the cached SDF lattice. Replaced S4's translation visual proxy. Intruder grows from cavity-sized at step 1 to bare scan at step 16; deformed cavity tracks it within BCC chunkiness. |
| S11.2 | `fcc57835` | Layer slab rendering — each per-layer entity now carries inner + outer face triangles, rendered as `AlphaMode::Blend` band at `LAYER_SLAB_ALPHA = 0.35`. Bookmark §2 F2/F3 resolved: user can see through layer 1 to layer 0 to cavity to intruder at a glance. Heat-map gradient still readable under alpha. |

Tests: 145 → 152 (+7). Clippy `-D warnings` clean. Grade A across all automated criteria.

S11.3 (cavity hide on `show_deformed` + intruder teal recolor) was on the spec ladder but **not shipped** — the pivot below makes its scope unclear.

---

## 2. The pivot (user-confirmed direction)

**User-verbatim (2026-05-18 LATER visual gate on S11.2)**:

> "the intruder changing sizes could be valuable as a step in the future to prototype variations of the scanned size going through the cavity. but what we are simulating is the intruder at a constant scale/size going along the cap scan line. tip enters the mouth, goes to the end. that should be the sim."

> "the intruder is bigger than the cavity at its beginning state, it should still be able to put pressure on the walls without having to grow in scale as it progresses down the length of the cavity"

> "in its current state it doesnt really offer anything meaningful. this simulation is purely meant to simulate the insertable being inserted into the cavity from mouth to end" (after presented with three options A=animation only / B=faux interpolation / C=true per-position FEM)

**Translation**: the FEM's growing-intruder ramp (interference scales 0 → 3 mm, intruder iso grows from cavity-sized to bare-scan over 16 steps) is a numerical-stability device the workshop user doesn't recognize. The product intent is a **constant-size sliding intruder** translated along the centerline from mouth → fully seated, with per-position cavity deformation driven by the sliding-contact zone.

**User-selected option**: **C** — real per-position FEM with sliding intruder. Acknowledged multi-week arc.

---

## 3. Why this is bigger than S11.3

The current cf-device-design FEM is structurally a growing-intruder solver:

- `insertion_sim::run_insertion_ramp` ramps `interference_m` from `cavity_inset_m / n_steps` to `cavity_inset_m` over `n_steps` Newton solves, each warm-started from the prior.
- `intruder_contact_at(&intruder, bounds, interference_m, cavity_offset_m)` builds the contact primitive as `intruder.offset(interference_m + cavity_offset_m)` — i.e., the contact SDF grows uniformly.
- The cavity-side BCC vertices deform UNIFORMLY across the cavity as the intruder iso grows (every cavity-wall point sees the same offset growth, not a sliding contact patch).
- The intermediate steps are NOT physically meaningful per slide position — they're the solver's stability path to the press-fit state at step 16.

A sliding-intruder solve needs:

- A rigid intruder of CONSTANT geometry (the cleaned scan).
- A per-step intruder TRANSFORM (translation along centerline arc).
- A contact primitive built from the TRANSFORMED intruder's SDF at each step (signed distance to a transformed mesh; the cached scan SDF doesn't move, so we need a different contact model — either re-evaluate the source SDF at transformed query points OR build a new SDF per slide step).
- A different convergence path: at each slide position, the cavity deforms in the LOCAL contact zone (where the intruder currently is), not uniformly.

Notable: the BCC analysis mesh + Yeoh material partition + outer-skin Dirichlet pinning stay the same. The change is the contact ramp + the per-step contact SDF.

Effort scale: not a few hours. Probably 1-2 weeks for the FEM rewrite, plus another week of visual-gate iteration.

---

## 4. What survives the pivot

Despite the user's "doesn't offer anything meaningful in current state" verdict on the growing-intruder visual, the work this session and the slice-7 ladder are NOT throwaway. What survives:

### Infrastructure (S11.1 + S11.2)

- **`InsertionSimOutputs.cavity_boundary_faces` + `per_layer_outer_faces`** (S2/S3) — the BCC boundary classification (cavity-side vs outer-skin vs cap-rim) survives any contact model. The sliding-intruder FEM will produce a different `x_final` per step, but the face indexing is geometry-only.
- **`deformed_layer_slab_mesh_at`** (S11.2) — the inner + outer face concatenation is geometry-only; any per-step `x_final` feeds it.
- **`LAYER_SLAB_ALPHA = 0.35` + `AlphaMode::Blend`** — translucency is independent of intruder model. User-verified valuable.
- **`IntruderMeshKey` snapshot-and-compare** (S11.1) — the change-detection pattern in `update_intruder_mesh` survives. Whether the intruder mesh is a per-step SDF iso surface OR a transformed constant mesh, the key shape stays.
- **`sample_sdf_into_cached_template`** (sdf_layers helper, S11.1) — a primitive for sampling any `Sdf` onto a `ScalarGrid` template, then MC. Reusable for any sliding-intruder visualization that needs the moving intruder's iso surface.
- **`extract_layer_surface` cap-clipped composition + cached SDF infrastructure** (pre-S11) — unchanged.

### Banked knowledge

- The **per-step playback slider** (S1) + **viewport step badge** + **per-step scalar field cache** infrastructure — works for any per-step output, sliding-intruder or growing-intruder.
- The **heat-map projection** (S5-S7 stub) — projects per-tet scalars onto MC vertices; works for any per-step deformation.
- `[[project-bevy-is-changed-footgun]]` — the snapshot-and-compare cure pattern is load-bearing for any `ResMut`-held resource.
- The **3-session pattern** (bookmark / recon / implement) — exactly what we're doing now.

### Sniped-fix-style retrofits possible (don't gate on full FEM rewrite)

- **Translucent slab + heat map** (S11.2) is independently shippable — already shipped.
- **Cavity hide on show_deformed** (S11.3 first half) — still applicable when L0 slab inner face owns the cavity surface. Could ship now if the slab continues to be the deformed render. Held until the new FEM clarifies whether per-step BCC cavity render survives.

### What does NOT survive (likely revert in the new arc)

- **`per_step_intruder_meshes: Vec<IndexedMesh>`** + **`intruder_mesh_at` accessor** — the per-step intruder iso surfaces from the growing-intruder ramp are not the right product. The struct field + accessor stay as scaffolding but get repopulated with constant-mesh-at-transformed-position results in the new arc.
- **`update_intruder_mesh`'s mesh-asset rebuild** — likely returns to a `Transform` write path (constant mesh + per-step transform), much like the original S4 `update_intruder_transform`, but transform target is "intruder position along centerline at displayed_step / n_steps fraction" instead of S4's `interference_m × -cap_plane.normal`. The S4 code in git history (pre-S11.1) is a useful reference.

---

## 5. Recon entry points for the next session

### 5a. Cold-read order

1. **This bookmark** end-to-end.
2. `docs/SIM_ARC_VISUAL_REWRITE_SPEC.md` + `docs/SIM_ARC_VISUAL_REWRITE_BOOKMARK.md` — historical context, the spec being superseded.
3. `tools/cf-device-design/src/insertion_sim.rs` — full file. The growing-intruder ramp is at `run_insertion_ramp` (~line 1808); `intruder_contact_at` (~line 924); `build_insertion_geometry` (~line 669); `outer_skin_bc` boundary conditions; `RampStep` / `InsertionRamp` types.
4. `tools/cf-device-design/src/insertion_sim_ui.rs` — `kick_off_simulation`, `run_sim_pipeline`. The per-step intruder MC at lines ~688-720 will get repurposed.
5. `tools/cf-device-design/src/main.rs` — `update_intruder_mesh` (post-S11.1), `IntruderMeshKey` snapshot. Look at git history `git show f2a64195^:tools/cf-device-design/src/main.rs` for the pre-S11.1 `update_intruder_transform` (the S4 translation pattern, a reference for the new arc).
6. `tools/cf-device-design/src/main.rs` Centerline resource — already loaded from `.prep.toml` per main.rs around line 427-447. `centerline_arc_length_m` helper around line ~2706. The centerline polyline is in physics-frame meters.
7. `[[project-cf-device-design-sim-arc-recon]]` + `[[project-cf-device-design-sdf-layers]]` memory entries for the broader sim-arc context.

### 5b. Decisions to resolve in recon

- **D-Slide1**: contact model for the sliding intruder. Options: (a) re-evaluate cached scan SDF at transformed query points (cheap, no new SDF); (b) build a moving GridSdf per slide step (expensive, accurate); (c) signed distance to a parry BVH of the transformed scan (mid-cost). The cleaned scan IS its own SDF source — the cavity is `scan.offset(-inset)` — so (a) is naturally aligned.
- **D-Slide2**: slide axis parameterization. Centerline arc (curving through body) vs cap normal straight line. User explicitly said centerline. For curved centerlines, does the intruder ROTATE to follow tangent, or maintain rest orientation? (Rest orientation simpler; rotation matches "tip follows centerline" mental model better.)
- **D-Slide3**: ramp endpoints. Slide fraction `t` in `[0, 1]`. At `t = 0`: intruder tip at the cap plane, body fully outside. At `t = 1`: intruder fully seated, dome apex of intruder at end of centerline. Need to compute initial translation = `-arc_length × tangent_at_cap` so the intruder rest position is "outside the body" at `t = 0`.
- **D-Slide4**: warm-start strategy. Per-step Newton solve still needs warm-starting from the prior step's `x_final`; the warm start may need to be CONVECTED with the slide direction (cavity vertices in the intruder's wake stay deformed; vertices ahead are at rest until the intruder passes). This is the FEM solver re-design's central question.
- **D-Slide5**: step count. Currently `DEFAULT_N_STEPS = 16` (interference / step ≈ 0.19 mm). For sliding, `n_steps = 16` over `arc_length = 83 mm` gives ≈ 5 mm per slide step — coarse. Probably want `n_steps` proportional to arc length, perhaps 20-40 steps for iter-1.
- **D-Slide6**: convergence behavior at intermediate steps. With a sliding intruder, the FEM might fail to converge at certain steps (intruder transitioning through narrow region). Need a Fork-B-style "partial seating is honest engineering data" stall-handling, same as the current growing-intruder ramp.
- **D-Slide7**: intermediate-step interpretation in the panel. Currently the F-d curve is force vs interference depth. For sliding, it becomes force vs slide arc-length. The convergence summary copy ("seated to X mm full depth") becomes "intruder slid X mm of Y mm arc length".
- **D-Slide8**: the deformed cavity render. The BCC cavity-side boundary will deform LOCALLY at each step (only where the intruder currently is). Visual story: the user sees a "deformation wave" propagating along the cavity as they scrub. This is the core product story the current arc was trying to tell but couldn't.

### 5c. Open architectural questions

- Should the sliding-intruder FEM be a **separate solver** in `insertion_sim` (e.g., `run_sliding_insertion_ramp`) or **replace** `run_insertion_ramp`? Replacing risks regressing the growing-intruder tests (currently a few in `insertion_sim.rs`'s `#[cfg(test)] mod tests`); separate solver keeps both available but doubles maintenance.
- Should the growing-intruder ramp be **kept as "Engineering Review mode"** (per user's "could be valuable as a step in the future to prototype variations of the scanned size going through the cavity") behind a mode toggle? Recon should resolve.
- Is there a path to **reuse the BCC mesh + per-tet materials** unchanged across the two solvers? Likely yes — both ramps operate on the same body geometry and same Yeoh partition; only the contact ramp + boundary conditions differ.

---

## 6. Three-session cadence

Per `[[feedback-bookmark-when-surface-levers-exhaust]]`:

- **Session A (this)** — bookmark this pivot + ship S11.2 translucency as a standalone win.
- **Session B (next)** — recon. Cold-read this bookmark + the cf-device-design FEM crate. Resolve D-Slide1..D-Slide8. Produce `docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` with concrete signatures, file locations, sub-leaf ladder, per-sub-leaf visual gates, FEM test plan.
- **Session C (implementation)** — execute the spec sub-leaf by sub-leaf with per-leaf visual gates, per `[[feedback-implement-measure-revert-pattern]]`.

The FEM rewrite itself may need its own sub-cadence (a-priori sketch: small solver-only fixture first to validate sliding-contact convergence, then wire to the panel UI, then visual gate on iter-1). Recon should propose.

---

## 7. Cross-references

- `docs/SIM_ARC_RECON.md` — original recon, now annotated as superseded for the visual side; the empirical baseline section + slice-7 references remain accurate for the underlying FEM.
- `docs/SIM_ARC_VISUAL_REWRITE_SPEC.md` — the S11 spec. SHIPPED on its own terms (S11.1 + S11.2 land its decisions correctly); the **product fit** is what got falsified, not the spec's craftsmanship.
- `docs/SIM_ARC_VISUAL_REWRITE_BOOKMARK.md` — the prior bookmark. §3.5 "Explicitly NOT in scope" called out the slide animation explicitly. That call needs revision: the slide IS the product, not the future feature.
- `[[project-cf-device-design-sim-arc-recon]]` — headline memo to update to point at this bookmark.
- `[[project-cf-device-design-sim-arc-visual-rewrite]]` (if there is one) — bookmark + spec memos similarly to be marked PARTIAL with this bookmark as the successor.
- `[[feedback-bookmark-when-surface-levers-exhaust]]` — three-session pattern.
- `[[feedback-autonomous-architecture]]` — drove the in-session pivot acknowledgement.
- Commit `f2a64195` — S11.1 FEM-accurate intruder (will likely be repurposed in the new arc).
- Commit `fcc57835` — S11.2 layer slab translucency (independently survives).
