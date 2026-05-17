# cf-device-design Sim Arc — Visual Rewrite Bookmark

**Status**: BOOKMARK 2026-05-18 LATE. S1-S4 visual story arc SHIPPED + USER-VERIFIED in-session, surfaced 3 architectural issues that need a rewrite rather than further patches. This doc captures the empirical findings + a sketched complete solution; the next session = recon to refine the spec; session after = implementation.

**Reading order**: §1 (what shipped) → §2 (what the visual gate revealed) → §3 (the complete-solution sketch) → §4 (open decisions for recon) → §5 (recon-session entry point).

---

## 1. What shipped this session (cf-device-design sim-arc S1-S4 + 2 polish)

Per `docs/SIM_ARC_RECON.md` §6 sub-leaf ladder, the visual story arc S1-S4 shipped as:

| # | Commit | Headline |
|---|---|---|
| S1 | `01dfa153` | Per-step playback timeline — slider + viewport `Step N/16` badge + per-step scalar field cache. |
| S2 | `294b1982` | Deformed cavity surface render — `bcc_boundary_faces` snapshot + `deformed_boundary_mesh_at` + `update_cavity_mesh` snapshot-and-compare. |
| S3 | `8dd2ac71` | Deformed per-layer shells render — `per_layer_outer_faces` build + outer-skin pinned detection + `deformed_layer_mesh_at`. |
| S4 | `9a55cc2a` | Scan-as-intruder render — IntruderEntity spawn + per-step translation along `-cap_plane.normal`. |
| doc | `969223d6` | SIM_ARC_RECON.md annotated with SHIPPED rows + §10 cold-read findings + §11 commit table. |

Plus two in-session bug-fix commits from the visual gate session:

| # | Commit | Headline |
|---|---|---|
| fix | `da6eb548` | Visibility toggle no longer invalidates sim run — `sim_relevant_snapshot` extracts only sim-affecting fields (excludes `visible: bool` on cavity + layer). |
| fix | `6cabc9e2` | Cavity render = cavity-side faces only — filtered `cavity_boundary_faces` excludes outer-skin + cap-rim mixed faces (was rendering the WHOLE BCC boundary). |

Tests: 139 → 145 across the arc. Clippy `-D warnings` clean throughout. `cargo xtask grade cf-device-design` → A across all automated criteria for every commit.

---

## 2. What the user-verified visual gate revealed (2026-05-18 LATE iter-1 session)

User booted cf-device-design on the iter-1 inputs (`~/scans/sock_over_capsule.{cleaned.stl,design.toml,prep.toml}`), ran [▶ Simulate Insertion], and scrubbed the playback slider through 16 steps with the clip plane engaged. Three concrete findings (in priority order):

### F1. S4 intruder is the FULL SCAN translated, doesn't match what the FEM is actually computing

What I shipped (recon §6 S4): translate the scan rest geometry by `interference_m × -cap_plane.normal` per displayed step. At step 8 (1.5 mm interference), the orange intruder shell is the full-size scan, shifted 1.5 mm along the cap-inward axis.

What the FEM is actually computing: the intruder is an SDF offset of the cleaned scan by `(interference_m - inset_m)` — a uniform RADIAL SDF offset, not a rigid translation:

- At step 1 (interference = 0.19 mm): intruder offset = −2.81 mm → intruder is the scan SHRUNK by 2.81 mm uniformly. Roughly cavity-shaped.
- At step 16 (interference = 3.00 mm): intruder offset = 0 → intruder is the scan at FULL SIZE. Full press-fit interference.

The FEM intruder GROWS from cavity-sized to full-scan over the ramp; it does not translate axially. The cavity surface deforms outward to accommodate the growing intruder; at every step the cavity surface ≈ intruder surface in contact zones.

**Visual symptom of the mismatch**: with the clip plane engaged at step 8, the user sees the full-size translated scan (orange disc) sitting SMALLER than the deformed cavity (coral ring), with a clearly visible "air gap" of ~1.5 mm radially between them. User interpretation: "there is air between the probe and layer 0 — looks like the cavity is being treated as a wall." Actually the gap is the geometric mismatch between my translated-full-scan visual proxy and the FEM-correct shrunken-scan model.

The S4 commit message documented this gap explicitly ("Visual proxy, not FEM-literal") and chose to implement the user's earlier "slide into the cavity" mental model. The visual gate now contradicts that choice: the proxy is confusing more than illuminating.

### F2. Layer shells render as thin surfaces, not slabs — user reads the silicone wall as missing

What I shipped (S3): each layer entity renders its OUTER face only — a thin shell at the layer's outer iso surface. Two-layer iter-1:

- Layer 0 outer face = L0/L1 interface, ~10 mm out from scan (deforming).
- Layer 1 outer face = outer skin, ~13 mm out from scan (Dirichlet-pinned, static).

What the workshop user reads: through the clip plane, two concentric chunky rings (amber L0 outer + blue L1 outer) with empty space between them and between L0 outer and the cavity surface. User question: "where is the silicone wall?" The empty spaces ARE the silicone volumes, but they're not rendered → user interprets them as missing material.

User quote: "the wall for layer 0 does not go to the outside wall of the cavity, which it should."

### F3. Opaque outer shells occlude inner layers + cavity + intruder

Layer 1's blue outer skin material (`StandardMaterial`, alpha = 1.0) completely hides everything inside it. The user sees only the outermost shell unless they manually toggle Layer 1's "Show layer" off. Before the `da6eb548` fix, toggling Layer 1 off invalidated the sim run — making the visual gate impossible to perform without resetting. Post-fix the workflow works, but the user still has to manually toggle layer visibilities to see anything inside.

### Cross-cutting visual quality issue: BCC mesh chunkiness

The BCC analysis mesh at `SIM_CELL_SIZE_M = 4 mm` produces a coarse triangulation. The deformed cavity / layer renders look CHUNKY (visible per-triangle facets, ~few cm scale) compared to the smooth SDF iso renders in rest mode (~1 mm MC).

The S2/S3 commit messages documented this trade-off ("Coarser than the SDF iso (4 mm BCC vs ~1 mm MC) but it's the 'see the squish' view"). Visual gate confirms it's acceptable but not pretty. Translucency + slab rendering would partially compensate by giving the user MORE visual information per pixel.

---

## 3. The complete-solution sketch (the rewrite this bookmark proposes)

The three findings above are all symptoms of a deeper architectural choice: **S1-S4 rendered surfaces under the FEM-translation visual proxy; the right architecture renders FEM-accurate intruder + silicone slab volumes.**

### Tier 1 — must-have (the complete solution)

1. **FEM-accurate intruder render.** Replace S4's translation with per-step marching cubes on the cached scan SDF at iso = `interference_k - inset_m` for each converged step:
   - At sim completion (inside `run_sim_pipeline`'s async task), batch 16 MC extractions → 16 `IndexedMesh`es cached on `InsertionSimOutputs.per_step_intruder_meshes: Vec<IndexedMesh>`.
   - IntruderEntity renders the mesh for `displayed_step` via a new system; no per-frame transform math.
   - At step 16 the intruder = full scan; at step 1 the intruder ≈ cavity geometry. Cavity surface coincides with intruder in contact zones at every step.
   - Cost: per-step MC at iter-1 ≈ ~100 ms × 16 = ~2 s extra in the async task (dwarfed by the ~30-60 s ramp). Memory: 16 × ~5 K vertices × 24 B ≈ 2 MB.

2. **Layer slab rendering (both inner + outer faces, translucent material).** Replace S3's outer-face-only render with closed-shell slabs:
   - Each layer entity carries BOTH its inner face (= the prior layer's outer face, or cavity surface for L0) AND its outer face (current S3 behavior).
   - Material alpha ≈ 0.35 with `AlphaMode::Blend` so outer layers don't fully occlude inner ones.
   - Through the clip plane, each layer reads as a translucent thick band of silicone — the volume implied by depth ordering of the two visible surfaces.
   - For L0: inner face = the cavity surface triangles (already extracted as `cavity_boundary_faces` in the post-`6cabc9e2` state). Outer face = current `per_layer_outer_faces[0]`.
   - For L1+: inner face = previous layer's outer face. Outer face = current `per_layer_outer_faces[i]`.

3. **Cavity entity becomes "rest-only."** With L0's slab owning the cavity surface as its inner face, the standalone cavity entity is redundant in show_deformed mode. Hide it when `show_deformed && last_run.is_some()`. In rest mode, the cavity entity keeps its current behavior (rendering the SDF iso at `-cavity.inset_m`).

4. **Distinct intruder color.** Current intruder color `INTRUDER_COLOR = (0.95, 0.55, 0.20)` (warm orange) is too close to the cavity coral `CAVITY_COLOR = (0.95, 0.55, 0.45)`. When FEM-accurate intruder coincides with the cavity surface at contact, the colors merge. Pick a cool contrasting color so the intruder stays distinguishable from the cavity in contact zones. Candidate: teal `(0.20, 0.75, 0.85)` or magenta `(0.90, 0.30, 0.75)` — recon should A/B these visually.

### Tier 2 — polish (defer or bundle)

5. **Cap-rim closures on layer slabs.** Triangulate the annular cap-end of each layer (between layer i's inner face cap-rim and outer face cap-rim) so slabs look closed from cap-direction angles. Without this, viewing from the cap side shows the silicone "tube" with an open annular end. For iter-1 sock-over-capsule (cap is at the bottom), this matters only at extreme view angles; defer unless visual gate flags it.

6. **Conceptual legend in the Insertion Sim panel.** A compact text block explaining the colors (cavity = coral, L0 = amber, L1 = sky-blue, intruder = teal/whichever) so the user has the mental key. Replaces the need for the user to infer color semantics from the rendering.

7. **Wireframe / outline overlay option for intruder.** As an alternative to translucent material, render the intruder as a wireframe so it stays visible without occluding interior cavity geometry. Trade-off: wireframes can be harder to read at certain angles. Pick based on visual gate after Tier 1.

### Explicitly NOT in scope

- **"Slide-from-entrance" intruder animation.** The user's verbatim earlier ask ("ideally the probe tip starts at the beginning of the entrance hole at the bottom") was a workshop-storytelling visual that conflicts with FEM physics at partial steps (full-scan probe sticking through a half-deformed cavity → reads as a render bug). It can be added as a separate "Animation mode" toggle in a future arc, distinct from the "Engineering Review mode" that S1-S4 targets. The Engineering Review mode is what the workshop user actually needs for design iteration.
- **Pressure heat map on cavity surface.** That's S5 (recon §6 sub-leaf S5), a different arc that needs D6 (score formula) resolved first.
- **Per-step cavity centroid recompute** for `project_layer_heat_map`. Heat-map projection uses REST centroids; sub-cm deformation vs cm-scale tet spacing means the lookup mostly stays correct. Future polish if pressure scoring (S5) needs sharper localization.

### Effort estimate

| Item | Hours |
|---|---|
| (1) FEM intruder | ~2 |
| (2) Layer slabs + translucency | ~2-3 |
| (3) Hide cavity in show_deformed | ~10 min |
| (4) Recolor intruder | ~1 min |
| **Tier 1 total** | **~5 hr** |
| (5) Cap closures | ~1.5 |
| (6) Legend | ~30 min |
| (7) Wireframe option | ~1 |
| **Tier 1 + 2** | **~8 hr** |

---

## 4. Open decisions for recon to resolve

The complete-solution sketch (§3) defaults to autonomous picks for routine items; the following surface for explicit user input in the recon session:

### D-Vis1 (visual): intruder color

Pick between teal `(0.20, 0.75, 0.85)` and magenta `(0.90, 0.30, 0.75)` (or other). Recon should boot the app with a quick prototype + visually A/B. Default: teal if user doesn't have a strong opinion.

### D-Vis2 (visual): layer alpha value

Translucency value for layer slabs. Too low (alpha < 0.2) and the slab disappears; too high (alpha > 0.5) and outer layers occlude inner. Recon should A/B 0.25, 0.35, 0.45. Default: 0.35.

### D-Vis3 (architectural): rewrite S4 in-place or new sub-leaf

Option A: rewrite S4's IntruderEntity + spawn_intruder_mesh + update_intruder_transform in-place (replace the translation path with the FEM-accurate per-step mesh path).
Option B: ship as a new sub-leaf S11 (or S4.1), keep S4's translation code commented out for fallback.

Default: in-place rewrite (A) per [[feedback-strip-the-knob-when-default-works]]. The translation visual proxy clearly didn't earn its keep on the visual gate; replacement is cleaner than a knob.

### D-Vis4 (architectural): cavity entity in show_deformed mode

Option A: hide the cavity entity when show_deformed is on (redundant with L0's inner face render in the slab).
Option B: keep cavity entity always visible but in a different material (e.g., wireframe) so the user sees the "exact cavity surface" outline overlaid on the slab.

Default: A (hide). Simpler; user can compare modes by toggling show_deformed.

### D-Vis5 (architectural): cap-rim closure tier-2

Worth implementing in the same arc, or defer to a follow-up?

Default: defer to follow-up. Tier-1 should ship first to gate on visual reception. Cap closures are localized geometry that's easy to add post-Tier-1.

### D-Vis6 (sub-leaf ladder): single commit or sub-leaves?

The rewrite touches ~4 things (intruder rewrite, layer slabs, cavity hide, color). Per [[feedback-implement-measure-revert-pattern]], small revertable chunks are preferable when uncertainty is high. Sub-leaf candidates:

- S11.1 — FEM intruder (replace S4 translation).
- S11.2 — Layer slabs (rewrite S3 outer-face render).
- S11.3 — Cavity entity hide + intruder color (small UX adjustments).

Default: ship as three sub-leaves with per-sub-leaf visual gates.

---

## 5. Cold-read findings still relevant (from `SIM_ARC_RECON.md §10`)

Six items captured post-S4 ship before the visual gate. Disposition after the rewrite:

| # | Finding | Disposition after rewrite |
|---|---|---|
| 1 | `update_intruder_transform` ignores `ScanMeshVisible` toggle | RESOLVED — the rewrite restructures the intruder entity; `ScanMeshVisible` semantics can be revisited at that time. Or accept the asymmetry. |
| 2 | `InsertionSimOutputs.tet_centroids` doc rot | Still relevant — update doc to acknowledge per-step deformed projection. Bundle into rewrite polish. |
| 3 | OOB accessor behavior inconsistent (clamp vs None) | Still relevant — document on impl block. Bundle into rewrite polish. |
| 4 | 1-layer design = static outer skin in S3 | PARTIALLY RESOLVED by slab rendering — the cavity surface is now part of L0's slab, so even a 1-layer design shows the deforming cavity. Outer skin still static. |
| 5 | D8 PLY save deferred from S2 | Still relevant — fold into rewrite OR keep deferred. |
| 6 | `per_step_scalar_fields` memory ceiling at deeper configs | Still relevant — document but no action needed unless a deeper sim configures it. |

---

## 6. Recon-session entry point (next session)

1. **Cold-read this bookmark end-to-end** — especially §2 (what the visual gate revealed) + §3 (complete solution sketch). The findings here are load-bearing for the recon.

2. **Re-validate empirical baseline.** Boot cf-device-design with iter-1 inputs: `cargo run -p cf-device-design --release -- ~/scans/sock_over_capsule.cleaned.stl --design ~/scans/sock_over_capsule.design.toml --prep-toml ~/scans/sock_over_capsule.prep.toml`. Click [▶ Simulate Insertion], wait for 16/16. Confirm:
   - S1 slider scrubs steps; viewport `Step N/16` badge tracks.
   - "Show deformed cavity (vs rest)" toggle auto-true post-completion.
   - Visibility toggles (cavity, layer 0, layer 1) do NOT invalidate the run (`da6eb548` regression test holds).
   - Cavity entity renders at the cavity location (post-`6cabc9e2`), not at the outer skin position.
   - Intruder visible (translated full scan, orange).
   - Visual matches the three findings F1-F3 above. If anything is different, that's a regression to investigate before continuing.

3. **Refine the complete-solution spec** by resolving the open decisions in §4. Concrete output: `docs/SIM_ARC_VISUAL_REWRITE_SPEC.md` with implementation details:
   - Concrete file locations + function signatures for the changes.
   - API shape for `per_step_intruder_meshes` + slab face data.
   - Sub-leaf ladder (S11.1/.2/.3 per default in D-Vis6).
   - Per-sub-leaf visual gate criteria.
   - Test plan.

4. **Memo + MEMORY.md update** to point at next session = implementation of the spec.

5. **STOP after the spec lands.** Implementation is the next session per the three-session pattern. Visual gates between sub-leaves stay short + scoped.

---

## 7. Why the three-session pattern fits here

Per [[feedback-bookmark-when-surface-levers-exhaust]]: "when tuning hits diminishing returns + the failure mode is sharply characterized, stop, write a cold-readable bookmark, switch sessions for the recon."

Diminishing returns: this session shipped S1-S4 + 2 in-session fixes. The remaining issues are architectural (FEM-translation mismatch, surfaces-vs-slabs, occlusion) — not iterable by patches.

Sharply characterized failure mode: §2 captures all three findings concretely with screenshots referenced. The complete-solution sketch (§3) addresses each finding with a specific architectural change.

Session context: this session has consumed substantial context on user back-and-forth + screenshot reads + multiple code reads. A fresh session for recon will have full window for the design work; another fresh session for implementation will have full window for the ~5 hr of focused coding.

---

## Cross-references

- `docs/SIM_ARC_RECON.md` — the recon doc this bookmark builds on. §10 cold-read findings + §11 commit table remain valid.
- [[project-cf-device-design-sim-arc-recon]] — the headline memo; update to point at this bookmark.
- [[feedback-bookmark-when-surface-levers-exhaust]] — three-session pattern justification.
- [[feedback-autonomous-architecture]] — drives the autonomous picks in §4 defaults.
- [[feedback-cold-read-review-post-ship]] — the per-sub-leaf visual gate cadence the implementation session will follow.
- [[feedback-implement-measure-revert-pattern]] — the sub-leaf ladder rationale in D-Vis6.
- [[project-cf-device-design-sdf-layers]] — last arc where the bookmark/recon/implement pattern worked cleanly; this arc follows that precedent.
