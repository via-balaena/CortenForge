# The fit test — plan to work against

**Status:** open. Written 2026-09-23 at `6ebb5a3e` (PR #963, merged as `b7f98831`). Jon agreed the
Phase 2 flow with D1–D5 as recommended.
> ⛔ **2026-09-24: the physics is replaced.** The simulation this plan measured (Tet10, implicit,
> IPC-style contact) gives way to an explicit solver on wgpu, planned in
> [`SOFT_CONTACT_ARCHITECTURE_RECON.md`](SOFT_CONTACT_ARCHITECTURE_RECON.md). Its §15 is the first build.
> - **Phase 2's flow stands** (D1–D5), and so do D6–D7 (under Speed). D1's comfort limits now come from
>   published measurements.
> - **Phase 1** (the PolyFEM oracle) **is abandoned**, and **Phase 4 is superseded.** What they measured is
>   kept below, in short.
>   - The full record is in the history of a local tag, `fit-test-oracle-and-flow-pre-squash`.
>   - That tag is deliberately not pushed, because its history carries scan-derived geometry.
> - **§2 describes the replaced solver.**
> - **G2 and G4 are updated for the new solver.** G1, G3 and G5 keep the old solver's baselines, and the
>   unknowns (§7) are updated. *(2026-09-28: G1, G3 and G6 now also carry the new solver's first run on `base_mold`,
>   soft-contact recon §16x; the element collapsing at its seated tip is recon §16y and U20.)* *(2026-09-29: D1's
>   element size read again with an eight-times wall is recon §16z.)*

**Scene:** the product scan `base_mold` — 5 mm inset, 17 mm Dragon Skin 10A at 25 % Slacker, curved
centerline. ⛔ It is a sensitive scan: probes load it from outside the repo,
and it must never be committed.

**How to use this document.** Section 1 is what the fit test must do. Sections 2–3 say where it stands and
why. Section 4 is the plan, with a *done when* for every step. Section 5 lists the gates each step must keep.
Section 6 lists decisions not to relitigate. Section 7 lists what nobody knows yet. Tick items as they
close, and update the numbers in place with their referent. **Every claim here names a file, a probe or a
document. Leave anything else out.**

Companions:
- [`INSERTION_SIM_TET10_RENOVATION_RECON.md`](INSERTION_SIM_TET10_RENOVATION_RECON.md), the **Tet10 recon**:
  the measurement history behind §2's numbers, on the replaced solver.
- [`SOFT_CONTACT_ARCHITECTURE_RECON.md`](SOFT_CONTACT_ARCHITECTURE_RECON.md), the **soft-contact recon**:
  the new solver's plan.

---

## 1. What the fit test is for

In Jon's words (2026-09-23):

> "the inset of the hole will have different insets depending on the parameters chosen by the user. the
> physics is for simulating how the scanned object […] will do against the silicone (00-30 vs dragonskin
> 10, etc) and different insets. if the inset is too big/hole is too tight where it cant comfortably (we
> will have to define this threshold) go in, then the inset should be reduced. this sim is for a user to
> fine tune their [device] to be perfect for their [scan] so they dont have to
> (or reduced amount of) physically iterate with physically casting and testing. This will also in the
> future be used to simulate other things, like sliding a foot into a boot, for the tightest fit, thats
> still able to be slid into."

> "when its seated, the scan shouldnt overlap the wall. the wall should stretch, like in real life with
> real silicone. its a rigid/rigid-ish softbody (future applications) going into a softbody. we want the
> visual to be realistic, we want the numbers to be realistic"

> "i think this […] should be in our actual app — the main one that we migrated back from slint."

**Requirements:**

- [ ] **R1 — The question.** Can this scan slide in *comfortably*, at the user's chosen inset and
  silicone? If not, the inset should come down.
- [ ] **R2 — Along the path.** Insertability is decided on the way in, not at the seat. The heel through the
  ankle of a boot is tighter than the seated foot.
- [ ] **R3 — The wall stretches.** Seated or moving, the scan never passes *through* the silicone; the wall
  moves out of its way. Visuals and numbers must both be realistic.
- [ ] **R4 — Materials and insets are the user's knobs.** Different silicones (Ecoflex 00-30, Dragon Skin
  10A, …) and different insets must give different, trustworthy answers.
- [ ] **R5 — "Comfortably" is a threshold Jon will define.** Section 7, U1. Its limits now come from
  published measurements (D1).
- [ ] **R6 — It lives in the main app.** `tools/cf-studio-gui` (Cendrillon: Bevy + egui, Slint retired).
  The wizard is `cf_studio_core::Step` (`tools/cf-studio-core/src/step.rs:12`): `AddScan`, `CleanScan`,
  `ShapePiece`, `DesignLayers`, `MakeMolds`, `Print`, `Pour`. The fit test belongs between
  `DesignLayers` and `MakeMolds`, so a bad fit is caught before molds are made, printed and poured.
- [ ] **R7 — Soft intruders later.** Today the scan is rigid. Later, "rigid-ish" soft bodies go into a soft
  body, which needs body-to-body contact.
- [ ] **R8 — Other scenes later.** A foot into a boot: the tightest fit that can still be slid in.

**Decided:** the default model is **sliding, carrying the inset** (Jon, 2026-09-23). The growing model
inflates the scan in place and never sees the path.

---

## 2. Where it stands (verified at `6ebb5a3e`)

This section describes the replaced Tet10 solver. "Recon" in its tables means the Tet10 recon.

### 2a. What the user sees today (`tools/cf-sim-research`, the research viewer)

| Symptom | Cause | Referent |
|---|---|---|
| The scan is always inside the cavity | A still copy of the scan is drawn at the SEATED pose by default | `ScanMeshVisible::default()` is `true`, pinned by a test at `main.rs:2207` |
| With the bridge ticked, nothing moves; the wall just swells | The bridge forces the GROWING model | `kick_off_simulation`, `insertion_sim_ui.rs:707` |
| The moving scan appears only sometimes | It is drawn only in sliding mode with Show-deformed on | `visible_pose_for_intruder`, `main.rs:1097` |
| No frame shows the scan outside | The first recorded step is t = 1/n. At 16 steps the tip is already a sixteenth of the path past the entrance | `slide_pose_at`; `DEFAULT_N_STEPS = 16`, `insertion_sim_ui.rs:67` |
| **The scan shows through the wall** | The panel draws the FULL-SIZE scan (`spawn_intruder_mesh`, `main.rs:989`), but the physics pushes the wall with the scan SHRUNK by the inset | Measured, next row |
| ↳ measured | Shipped default (penalty sliding, 16 steps): **12/16** steps converge, reaching t = 0.75. At that step **822 of 1 684** drawn wall vertices sit inside the drawn scan, **720 deeper than 1 mm, deepest 6.06 mm** | A temporary probe through `run_sim_pipeline`, reverted. Recorded in memory `project_insertion_sim_answers_can_it_slide_in` |
| The heat map is slow on the bridge | It colours every solved-mesh node, though only about 4 % are drawn: **4.08 s per call** | Recon, `PER-GAUSS-POINT READOUTS` |
| The wall looks faceted | The deformed view draws corner triangles; the Tet10 midside curvature is not drawn | Recon, same section |
| The drawn wall is smoother than the part that gets poured | The simulated cavity is a smooth offset; the poured plug has three ridge rings (1.8–2.0 mm deep), texture, side pinch and tip relief | `product_scene` doc, `insertion_sim.rs` |

### 2b. The physics

| Gap | Referent |
|---|---|
| **Neither sliding ramp carries the inset.** The shipped `run_sliding_insertion_ramp` passes `interference_m = 0`. F4's warm-up, which ramped the inset in, was reverted. The Tet10 bridge (`run_sliding_insertion_ramp_tet10_ipc`, never wired) offsets the scan by `cavity_offset_m` alone. Fully seated, both contacts ARE the cavity surface, whatever the inset | `insertion_sim.rs`; `docs/archive/F4_FALSIFICATION_POSTMORTEM.md`; recon, `THE SLIDING MODEL ON THE PRODUCT SCAN` |
| **Our IPC is v1.** No continuous collision detection (`ccd_toi` returns ∞), fixed κ, one-way rigid kinematic coupling — its doc calls this "adequate for the small-dt keystone scene" | `sim/L0/soft/src/contact/ipc.rs:31–42, 488` |
| ⇒ **Step-count limits.** Without CCD, each step's closing must stay under the barrier band (d̂ = 1.2 mm). The sliding bridge first accepts **110** steps (as written) and **117** (bare scan); the closing is not monotone in the step count. The growing bridge seats **4.531 of 5 mm** at 32 steps and 3.438 mm at the panel's 16 | `what_the_sliding_contact_reaches_on_the_product_scan`; recon |
| **No friction** (μ = 0). Tet10 face contact asserts on friction (the face-friction reconciliation is a deferred rung). No μ data for silicone × lubricant × skin has been found in the repo (memory `project_insertion_sim_renovation`) *(2026-09-27: some found, U2)* | `sim/L0/soft/src/solver/backward_euler/assembly.rs:358` |
| **The intruder is rigid and teleported.** Each step places it at the next pose — turned about its tip to follow the centerline — then solves the wall. That motion alone asks the wall for up to **8.3 mm** of room with no inset (at t = 0.6875, ~14 mm inside the entrance). *(2026-09-27: most of it is the motion's own, U3.)* | `slide_pose_at`; recon |
| **Both models stop on element 516.** The growing bridge at 32 steps; the sliding bridge as written at 67/128. A validity-domain violation (over-stretched or inverted) | `the_bridge_ramp_over_a_stiffness_sweep_on_the_product_scan`; `the_sliding_bridge_as_written_on_the_product_scan` |
| **Non-penetration is checked on corner nodes only.** The face barrier loads midsides, which no probe checks | `the_sliding_bridge_as_written_on_the_product_scan` post-check |
| **The penalty paths go through the wall** at their shipped κ = 1e3, on every scene measured | Recon |

### 2c. Where the code lives

- [ ] The simulation is inside `tools/cf-sim-research`, a **binary-only** crate: `[[bin]]` only in its
  `Cargo.toml`, and `pub(crate) mod insertion_sim` at `main.rs:71`. **The main app cannot depend on it.**
  `cf-studio-engine` does not depend on `sim-soft`.
- [ ] The standing direction is app / SDK separation: apps are built on a stable SDK (memory
  `project_app_sdk_separation`). The simulation belongs on the SDK side.

---

## 3. Are we using the right simulation?

**No, not at this speed target.** In a validated library, an implicit, IPC-style solve of `base_mold`
took about 7 minutes per step (Phase 1, Speed), roughly 200× over D4's budget. Explicit dynamics is what
seal mounting and compression-stocking studies use, and the GPU precedent is TLED. The replacement is an
explicit, TLED-style solver on wgpu (soft-contact recon §4–§6), and its first experiment is its §15.

---

## 4. The plan

Each phase has a *done when*. Phase 1 is abandoned and Phase 4 superseded (see the banner).

### Phase 0 — Land what is measured
- [x] Merge #963 (green, 25/25 at `6ebb5a3e`), with `--body-file`. Merged as `b7f98831`.

### Phase 1 — The reference oracle (outside the repo). ⛔ ABANDONED 2026-09-24
PolyFEM, the IPC authors' library, was built outside the repo (PolyFEM commit `591b08bd`). It passed its own 49
contact scenes, then ran the product scene. Jon cut it on 2026-09-24 (*"I was pretty underwhelmed"*), and
its build and caches are deleted.

What it established about the product scene, which still holds:
- **Our Yeoh has an exact equivalent there:** its `NeoHookean`, plus a `SaintVenant` term with μ = 0 and
  λ = 8·C₂. The lateral stretch agreed to 4.7·10⁻¹¹.
- **The bare scan does not start clear of the wall.** At t = 0, 5 wall corners sit inside it, up to
  1.33 mm deep. A pre-roll of 4.7 mm back along the centerline's tangent leaves a gap of 1.42 mm. Any run
  that starts from our t = 0 pose meets this. *(2026-09-27, soft-contact recon §16w: the new path starts where every
  boundary node of the wall is 5 mm clear of the scan.)*
- **Our contact's intruder is not the raw scan.** Its SDF is built from the scan decimated to 2 500 faces
  and smoothed, and the cavity is its −5 mm level.
- **Our outer-skin pin** (`outer_skin_bc`) takes every vertex within half a cell of the outer envelope.
  That includes 646 vertices inside the wall. *(2026-09-27, recon §16w: the holds read the outer skin from boundary
  faces only, so they take none.)*
- **The path's frame count matters.** Straight-line motion between frames strays from the rigid pose by
  0.80, 0.34 and 0.12 mm at 32, 64 and 128 frames. *(2026-09-27, recon §16w: the fitted path is sampled until it
  strays at most G2's floor at the times it reads.)*
- **The wall's surface touches itself at two points:** a non-manifold edge, with gaps of 10⁻¹⁹ m. *(2026-09-27,
  recon §16w: on step 7's wall, meshed on the fixed mesher at the 5 mm inset and h_K2, it does not; other insets are
  not checked.)*
  - Our solver has never noticed, because it does not check a surface against itself. A solver with
    soft-on-soft contact will (soft-contact recon §9 decision 9).
  - Removing 4 tets (2.55 mm³) repaired it. The obvious fix, dropping the tets on the pinch edge, does
    not.
- **It was far too slow** (Speed, below).

The exporter, `export_the_product_scene_for_the_ipc_oracle`, is `9f448e72` in the local tag's history.

### Phase 2 — The fit-test flow, written before more physics
The asks: pick the inset and silicone; watch the scan go in **from outside**; see a push-force-versus-depth
curve and a pressure map; get a verdict against the comfort threshold that, if the fit is too tight, says
which inset would pass.

**Agreed by Jon, 2026-09-23, with D1–D5 as recommended.** Code is cited at `b7f98831`.

- [x] **Where it sits.** A new wizard step between `DesignLayers` and `MakeMolds`. It touches:
  - `Step` and every exhaustive match on it (`tools/cf-studio-core/src/step.rs:12–112`), the screen
    `match` in `draw_body` (`tools/cf-studio-gui/src/panel.rs:443–472`), and `shows_the_piece` with its
    test table (`tools/cf-studio-gui/src/scene.rs:215–220, 738–746`);
  - a fit artifact on `Project` (`tools/cf-studio-core/src/project.rs:256–271`), in `is_complete`,
    `clear_after` and `clear_artifact`;
  - `set_molds`, which requires `DesignLayers` today (`project.rs:469`) and would require the fit step;
  - the schema, v4 → v5 (`PROJECT_SCHEMA_VERSION`, `project.rs:32`), with a `migrate()` step like the one
    that inserted `ShapePiece` (`project.rs:581`);
  - the CLI's `cmd_molds` (`tools/cf-studio/src/lib.rs:133–160`), per D2.
- [x] **What the user does:**
  1. Arrives from "Use this design". The inset (set on `ShapePiece`, `PlugDraft.cavity_inset_m`) and
     the layer stack (set on `DesignLayers`) are shown here, not edited.
  2. Presses **Run fit test**. It runs off the UI thread like the plug-cast check (`PlugFitJob`,
     `tools/cf-studio-gui/src/jobs.rs:376–519`), showing step *k* of *n* and the elapsed time, with
     **Cancel**. The studio cannot cancel a running job today: a search for "cancel" finds only file
     dialogs and saves.
  3. Watches the scan start **outside** the device and slide in along the centerline. The device is cut
     away so the cavity shows, the wall stretches, and the cavity wall is coloured by contact pressure. A
     scrubber replays the steps. The scan drawn is the one the physics moved (R3). G1 and G2 (§5) are
     checked on every run, and a failure is reported, not drawn.
  4. Reads the push-force-versus-depth curve. No crate depends directly on a plotting crate (`egui_plot`,
     `plotters`); `plotters` arrives only through `criterion`. The research viewer draws its curve with
     egui's painter
     (`render_force_displacement_plot`, `tools/cf-sim-research/src/insertion_sim_ui.rs:1615`).
  5. Gets one of three verdicts, kept apart:
     - **Fits.** It slid in and seated. Shows the peak push force and where it came, and the seated
       pressure.
     - **Too tight.** Shows which reading crossed its limit and at what depth, and the largest inset that
       passes (D3).
     - **Could not finish.** The simulation stopped at *d* mm, and says why. This is not a verdict on the
       fit. Today's stop at element 516 is this case.
  6. Either presses **Use *n* mm**, or continues to `MakeMolds`. **Use *n* mm** re-commits `ShapePiece`
     at the new inset, which clears the design. The same layers are re-applied, then the plug-cast check
     and the fit test run again.
- [x] **What the physics must output**, per step along the path:

  | Output | Unit | Definition | Today |
  |---|---|---|---|
  | Depth | mm | Arc length the scan has slid in from the cap mouth, along the centerline | `SlideRampStep.arc_length_s_m` |
  | **Push force** | N | The work to advance the scan one more millimetre along its path, per millimetre: −Σ fᵢ · (dxᵢ/ds), where fᵢ are the contact forces on the scan and dxᵢ/ds is how far scan point *i* moves per unit of path | **Not computed.** The only force readout is \|Σ f\|, the size of the total force (`contact_force_magnitude_n`, `insertion_sim.rs:2647`). On a curved path that includes the sideways push. The research viewer plots it (`insertion_sim_ui.rs:1623`) |
  | Contact pressure | kPa | Normal traction on each cavity-wall face: its map, its peak and its area-weighted mean | The κ probes read the area-weighted mean (Tet10 recon). The heat map colours Ψ or ‖P‖ (`ScalarMode`, `design/cf-device-types/src/sim.rs:88`), not contact pressure |
  | Silicone stretch | — | Peak principal stretch, and its margin to the calibrated cap | `StepReadout.max_principal_stretch` (`insertion_sim.rs:2006`) |
  | Status | — | Converged, or stopped and why | `SolverFailure` |

- [x] **"Too tight" is computed from U1 (§7).** Decided in D1 below.
- [x] **Runtime budget.** Suggesting a different inset takes at least one more verdict. For
  comparison, the plug-cast check with ridges on is budgeted at "~4 minutes"
  (`tools/cf-studio-engine/src/preflight.rs:75`). The budget is D4.
- [x] **Inputs it must refuse or snap.** The studio accepts any whole-number Slacker % (`LAYER_SLACKER_RANGE`,
  `tools/cf-studio-gui/src/lib.rs:1193`) and never calls `resolve_slacker_fraction`. The simulation
  errors on a fraction that is off Smooth-On's curve. For a silicone with no Slacker data, or one where
  Slacker is not recommended, it silently uses the unsoftened material (`effective_silicone_for_layer`,
  `insertion_sim.rs:407–417`). The fit test must not report on a material it did not simulate. Either snap the value in `DesignLayers` and
  say so, or refuse with the reason. *(2026-09-27, recon §16w: the lowering refuses a wall where two materials meet,
  so a design whose layers differ in material is refused until the interface rule's PR.)*
- [x] **Decisions.** Jon agreed all five recommendations on 2026-09-23.
  - **D1 — U1, "comfortably".** *Decided:* two readings, each with a limit (see *Calibration* below).
    - *Getting it in:* the peak push force over the whole path (R2). Its geometric share, the push of the
      verdict's μ = 0 run, is read as its largest mean over 10 mm of travel (Jon, 2026-09-26, accepting the
      recommendation; soft-contact recon §16s). *Open:* whether the peak is read as it is at a low friction is
      decided after step 7 measures it there (Jon, 2026-09-26). *(2026-09-28, recon §16x: measured on `base_mold`, at
      the pairing library's lowest μ_f, as the largest mean over 1 mm of travel: from the element size K2 needs it
      moves 8.4 % over the first doubling of the elements and 2.2 % over the second, within K5's bar from twice. The
      element size D1's readings need is open, four times or finer, on the seated patch.)* *(2026-09-28, recon §16y:
      resisting the elements that collapse at the seated tip, those under half their nodes' volume, moves that peak by
      at most 0.4 % at four times h_K2's elements; its effect on the seated patch is U20.)* *(2026-09-29, recon §16z:
      from four to eight times h_K2's elements the peak moves −1.12 and −0.96 % at μ_f 0.104 and 0.18. D1's size is not
      picked: twice h_K2's elements or finer by the size rule, which takes h_K2's scatter for twice's, and eight times
      or finer read at 5 % flat. From four to eight times the frictionless patch moves +5.06 % against its replicate's
      0.71 %, which the size rule cannot tell from 5 %.)*
    - *Seated:* ~~the contact pressure at the seat, read as its area-weighted 95th percentile~~ **the contact
      force on the most-loaded 1 cm² patch, over 1 cm²** (Jon, 2026-09-26, accepting the recommendation after
      K5; soft-contact recon §16s). The percentile it replaces did not converge on the tube: its most-squeezed
      5 % was one or two rings of nodes (a diagnostic). 1 cm² is the algometer tip most pressure-pain studies in
      a 2021 review used, so the reading and the limit it will be judged against can be taken over the same
      area; the patch's size follows the data that calibrates it. The percentile and the peak are shown beside
      it but do not decide, so the verdict never rests on one face of the mesh.
    - The stretch margin is a separate durability warning, not part of comfort.
    - *Calibration:* ~~on Jon's casts~~ **from published measurements** (Jon, 2026-09-24: no home-lab
      calibration). The push-force anchor is the clinical axial-rigidity convention (~5.4 N). Pressure
      limits come from published comfort and pain-threshold data. The relevant tissue's pressure-pain threshold,
      measured with a 1 cm² tip, was found on 2026-09-27 (U1); the limits are not yet set.
  - **D2 — Gate or advise.** *Decided:* advise.
    - Continue stays open, as it does for the plug-cast check (`panel.rs:1162–1164`). The verdict, or
      "not run", is saved with the project and repeated on `MakeMolds`.
    - The fit step counts as complete once passed through, run or not. That keeps `validate()`'s
      completed-steps-in-order rule without forcing a run. The CLI's `cmd_molds` must then record "not
      run" rather than fail.
    - *Could not finish* never blocks anything: it is the simulation failing, not the fit.
    - Revisit only once the verdict's limits are validated (U1). Even then, ask ("make molds anyway?")
      rather than block. (Updated 2026-09-24: the original condition was a check against Jon's casts,
      which went with cast calibration.)
  - **D3 — The suggested inset.** *Decided:* on request, as one button: **Find the tightest inset
    that fits**. "Tightest" is Jon's rule for the boot (§1).
    - It searches down after *Too tight* and up after *Fits*.
    - Not automatic, because a search needs several verdicts (D4).
    - The studio's inset is whole millimetres (`ShapeControls.cavity_mm`,
      `tools/cf-studio-gui/src/shape.rs`). Down from 5 mm, a bisection over 4, 3, 2, 1 and 0 needs at
      most 3 verdicts. Up, it steps one millimetre at a time until a verdict fails.
    - Each candidate must pass the plug-cast check first (smooth: 6.9 s at 5 mm,
      `tools/cf-studio-engine/src/preflight.rs:74`). An inset that will not cast is never suggested.
    - Every run's result is kept, so a cancelled search still shows what it found.
    - The bisection assumes a smaller inset never reads tighter. The first searches check that by also
      running the answer's neighbour.
    - Make it automatic once runs meet D4.
  - **D4 — The runtime budget.** *Decided:* at most 5 minutes per run on this machine (Apple M4
    Pro) and 15 minutes per search, measured on `base_mold` as G6.
    - Jon, 2026-09-24: *"i just mean i want a fast simulation. but i dont want to sacrifice quality."*
      So it is a target, measured per press (U13), and quality is never traded for it.
    - Each step is shown as it is solved, so the user watches the scan go in during the run, not after
      it.
    - Five minutes is about what the app already asks for the plug-cast check with ridges on ("~4
      minutes", `preflight.rs:75`).
    - **A run is one simulation.** A verdict is the runs across its corners (soft-contact recon §15h).
    - The new solver's K1 sizes a run at ≤ 2 minutes (soft-contact recon §15a).
    - A verdict is 3 runs if stiffness scaling holds (the pairing's low and high μ, plus μ = 0), about
      6 minutes. The Mullins state is not a verdict corner (U11).
    - Full verdicts across a D3 search of 3–4 insets would take 18–24 minutes, over this
      15-minute limit (arithmetic). So the search uses the instant per-slice estimate to pick its
      candidates, and runs full verdicts at 1–2 insets (soft-contact recon §15g step 7). *(2026-09-26,
      recon §16r: on `base_mold` as meshed at the element size K2 needs, a press takes 0.29–0.61 of D4 at
      K1's rate, so 3–4 verdicts take 4–12 minutes, within 15; with the canal nodes projected (U17) at a
      quality floor of 0.5 and the viscosity, 15–21; arithmetic.)* *(2026-09-27, recon §16v: step 7's wall
      projects nothing; at ν 0.49 with the viscosity, 3–4 verdicts take 6–11 minutes over the insets measured,
      and 9–16 at the budget's worst corner (ν 0.495, the viscosity × 1.47); with the per-slice estimate picking
      1–2 insets, 3–8 at the worst corner; arithmetic.)* *(2026-09-28, recon §16x: those took the budget's loading;
      at the four times it that D1's readings need, a press takes 0.38 of D4 on the CPU at h_K2 and 3.8 at four
      times its elements (1.3 and 17 at K1's per-step budget, which the CPU beats), and full verdicts at 1–2 insets
      take 0.13–0.25 and 1.3–2.6 of the search's 15 minutes.)* *(2026-09-29, recon §16z: at eight times h_K2's
      elements, 11.8 of D4 on the CPU, and full verdicts at 1–2 insets 3.9–7.9 of the 15 minutes; D1's size is not
      picked.)*
    - If the new solver takes far longer on the same scene, revisit the budget rather than cut the
      physics.
  - **D5 — Where the inset changes.** *Decided:* on the fit screen, as **Try at *n* mm**, with D3's
    answer filled in.
    - It goes through the existing funnels in order: `set_plug` at the new inset, then
      `apply_design_draft` with the same layers (it copies the plug's inset, so the two cannot
      disagree; `tools/cf-studio-gui/src/lib.rs:260–272`), then the plug-cast check, then the fit test.
    - `ShapePiece` stays the inset's owner; the fit screen is a shortcut to it.
    - The previous runs stay listed for comparison, e.g. "5 mm: too tight · 4 mm: fits".
- **Done when:** the flow is written here and Jon has agreed it, and each output names the physics quantity
  behind it. **Done 2026-09-23.** D1's limits now come from published measurements (2026-09-24).

### Phase 3 — Make the research viewer honest (cheap, independent)
- [ ] Hide the seated copy of the scan while a sliding run is displayed.
- [ ] Record the outside starting frame (t = 0) so playback starts with the scan clear of the device.
- [ ] Label the model actually run. Ticking the bridge silently switches sliding to growing.
- [ ] Heat map: colour only drawn vertices, or cache each vertex's nearest element per run (rest positions
  are fixed now).
- **Done when:** playback on `base_mold` starts outside and never shows a scan the physics did not move.

### Phase 4 — Close the physics gaps. ⛔ SUPERSEDED 2026-09-24 by `SOFT_CONTACT_ARCHITECTURE_RECON.md`
The implicit solver's gaps listed here are not pursued, because the explicit solver replaces it
(soft-contact recon §15). They were: the inset in the sliding contact, CCD, adaptive κ, friction, driving the intruder,
element 516, and midside non-penetration.

### Speed — reaching the 5-minute budget (D4). Superseded by the soft-contact recon
Measured in PolyFEM on `base_mold`, 2026-09-23/24:
- **About 7 minutes per step at first contact:** three Newton solves, 30 iterations, 438 s. Only step 3
  was measured. At that rate, the path's 133 steps would take about 16 hours (arithmetic).
  - Step 3 converged only once the barrier stiffness was fixed at 464. With the defaults it had not
    finished after 40 minutes.
  - CHOLMOD took 15 s per iteration, against Apple Accelerate's 37 s.
- **Every lever tried below made it worse:**
  - two large steps: 205 iterations, still unconverged after an hour;
  - half steps: 47 iterations, 690 s;
  - a coarser 6 mm wall: its second contact step hit the 500-iteration limit. It is also a different
    shape, with 4.2 % less volume.

  A warm start and reusing the symbolic factorization were not tried.
- **The mouth rim is where one node jams** (42× the next node's force). **Whether it is the cause was not
  isolated.** A rounded-lip variant took 59 iterations and about 930 s. But first contact falls on a
  different part of the scan in the two runs, so they are not a like-for-like pair.

This is why the architecture changed (soft-contact recon §3).
- The soft-contact recon's K1 asks for a 100k-tet insertion in ≤ 2 minutes (its §15a).
- The precision gate this section once set, *"a precision spike on contact before any GPU contact
  code"*, is now the soft-contact recon's K3.

**Decided (Jon, 2026-09-24):**
- **D6 — The GPU target is wgpu, on Metal and Vulkan.** That rules out CUDA-only engines as product
  code.
- **D7 — An outside physics engine stays outside (oracle only) if it is not Rust; a Rust one may come
  inside.** Jon: *"outside only if not rust"*. PolyFEM (C++) was cut (Phase 1).

### Phase 5 — Into the main app
- [ ] Extract the simulation from `cf-sim-research` into a library crate on the SDK side.
- [ ] Build the fit-test step in `cf-studio-gui` to the Phase 2 flow.
- **Done when:** the wizard runs the fit test on a user's scan end to end, and its numbers on `base_mold`
  match the new solver's standalone run (soft-contact recon §15g step 7).

### Later
- [ ] Soft (rigid-ish) intruders: body-to-body contact (R7). ⚠ The wall mesh touches itself at two points
  (Phase 1; *on step 7's wall at the 5 mm inset it does not, recon §16w*). PolyFEM rejected it, and a solver with
  self-contact will need it fixed, at the mesher. This is
  now soft-contact recon §9 decision 9: soft-on-soft contact is designed in from the start, and built
  second.
- [ ] Foot into boot (R8).
- [ ] **A contact-guided scan** (U3; soft-contact recon §16t): the scan pushed along the canal and settling where
  the wall's forces balance, in place of the fitted pose. It needs two-way rigid–soft coupling (soft-contact recon
  §14a's `sim-coupling`). Whether it keeps the modulus corners' shortcut, which soft-contact recon §5d states for
  a prescribed motion, is not known. Step 7 prints the sideways force and twist the wall puts on
  the scan, which show how far the walls would push it off the fitted pose; what reading brings this forward is not
  yet set. *(2026-09-28, soft-contact recon §16x rule 10: set, a scan free to move changing D1's reading by more than
  5 %; on `base_mold` it changed it by −1.0 % at the seat and −2.8 % held at the centre of the frictionless run's
  largest 10 mm window, so no recommendation to bring this forward; the scan was free in four of the five freedoms the
  path leaves it, the turn about the path held.)*
- [ ] Ridges and texture in the simulated cavity (§2a, last row).
- [ ] **A lip radius for every silicone, the product's next evolution.** Jon, 2026-09-24: *"we can
  add a lip radius for all silicone, including the 00-30. its an upgrade and the next evolution of
  the product anyways. the geometry was just easier for casting with the sharp edge at the lip."*
  The fit test should show sharp against rounded; that comparison runs in soft-contact recon §15g step
  7. The scope
  below is from a code survey on
  2026-09-24; the first three items were checked by hand.
  - **One source of the edge.** `cf_design::pinned_floor_shell` cuts each shell flat with
    `shell.intersect(Solid::plane(..))` (`design/cf-design/src/solid_layered.rs:129`). The mold's plug is
    built with it (`tools/cf-cast-cli/src/derive.rs:254`), and so is the simulated cavity. A "lipped
    cavity" solid next to `pinned_floor_shell` could therefore serve both. That is an inference; build
    it and test it.
  - **Already banked.** `docs/CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md` accepts a "knife-edge brim" for v1
    (:125) and banks "Knife-edge brim smoothing" (:657).
  - **The parameter's path.** `PlugDraft` (step 3, ShapePiece), then `CavityState` / `CavityBlock` in
    `.design.toml` (with a serde default, so old files load), then `SimDesign` and the cast's derive.
  - **Mold risks the survey named.**
    - The plug prints dome-down, cap face up, so a flared base is a downward-facing overhang ending in a
      feather edge.
    - The plug/cup parting line moves onto the mouth face.
    - The radius must span several marching-cubes cells to survive meshing.
    - The canal's first ring sits near the mouth.
    - The scan-mesh-direct plug path bypasses solids.
    - The studio preview and `cf-device-geometry`'s layer surfaces build their own geometry and would
      drift.
  - **Simulated side.** A comparison-only rounded-lip option (`smooth_subtract`, whose `k` is a blend
    width, not a radius) was built for the oracle and removed with it. It is in the local tag's history
    at `371f4a54`. The lip belongs in the shared `cf-design` definition.

---

## 5. Gates every phase must keep

| Gate | What must hold | Baseline today |
|---|---|---|
| **G1 — Drawn wall outside drawn scan** | No drawn wall vertex inside the drawn scan (beyond a stated tolerance) | Shipped default: 720 of 1 684 more than 1 mm inside, deepest 6.06 mm. *(2026-09-28, recon §16x: the new solver's first run on `base_mold`, against the scan's exact distance with G2's bar as the tolerance: at most 0.096 of it over the reads, 0.019 at the end over every surface node, in every valid run)* |
| **G2 — Bounded penetration** | No node deeper than **1 % of the inset** (0.05 mm on `base_mold`) at any step; *(Jon, 2026-09-27, U18)* the bar is never below **0.02 mm**. An engineering call (Jon, 2026-09-24: *"your call, just need that balance of real life/visual tranferable realism for viusals and legit engineering work"*): penalty contact always penetrates slightly, so this bounds it as a numerical tolerance (soft-contact recon §15c). **The gate stands; if the contact law cannot meet it, the law changes** (recon 15g step 2). Kinematic projection has none. For the old Tet10 solver the gate was no node through, corners and midsides. *(2026-09-25: the explicit solver's contact law is now kinematic. On the benchmark tube the deepest node is at most 0.2 µm inside the grid over a run, and 0.9 µm inside the true surface at its end (soft-contact recon §16o). On `base_mold`, G2 rests mostly on the baked scan grid's own error, not yet measured.)* *(2026-09-26, recon §16r: measured; no grid tried meets it, U18.)* *(2026-09-27, recon §16u: that was the flood fill's sign; signed by parity, the grid's own error meets it at the 5 mm inset. Jon set the bar at smaller insets: 1 % of the inset, never below 0.02 mm, U18.)* | Met on the benchmark tube (recon §16o); on `base_mold` the grid's own error meets it at the 5 mm inset (U18), and a run judges it at step 7. *(2026-09-28, recon §16x: step 7's first run, against the scan's exact distance, at most 0.096 of the bar over its reads in every valid run)* |
| **G3 — Full seat** | The full inset is reached along the sliding path | Growing: 4.531 of 5 mm. Sliding with the inset: met by construction on the prescribed fitted path, in every valid run of step 7's first (recon §16x, 2026-09-28) |
| **G4 — κ independent of the schedule** (implicit solver only) | The derived κ does not change with the step count | Holds (the ceiling rule) |
| **G5 — Heat map in the rest frame** | `the_heat_map_reads_the_deformed_view_at_rest_positions` passes | Passes; fails under four mutations |
| **G6 — Runtime** | Within the Phase 2 budget | See Phase 2. *(2026-09-28, recon §16x: on `base_mold` a press takes 0.38 of D4 on the CPU at the element size K2 needs, and 3.8 at four times its elements; D1's readings need four times or finer, open)* *(2026-09-29, recon §16z: 11.8 at eight times, the size used and not picked, 27.5 re-estimating every 50 steps; D1's size is twice h_K2's elements or finer by the size rule, taking h_K2's scatter for twice's, and eight times or finer read at 5 % flat)* |

G1 and G2 should become committed probes (the scan is repo-excluded, so they run locally and cannot gate
in CI); G5 already gates in CI.

---

## 6. Decisions already made — do not relitigate

- **Sliding, carrying the inset, is the default** (Jon, 2026-09-23).
- **The product scene is `base_mold`**, not `sock_over_capsule` (memory `feedback_confirm_the_fixture_is_the_subject`).
- **κ = the ceiling**, justified by the cushioning requirement and independent of the step schedule; never
  chosen by depth (Tet10 recon). That holds for the implicit solver. The explicit solver's contact
  stiffness is k = s·m/Δt² (soft-contact recon §15c). *(Superseded 2026-09-25: its contact is now
  kinematic, with no stiffness; soft-contact recon §16o.)*
- **F-bar is not the fix** for element inversion on Tet10 (hard-gated off; no multi-Gauss-point analog).
- **External libraries are oracles only**, never shipped (memory `feedback_julia_above_the_line_rust_below`).
  Refined for physics engines (Jon, 2026-09-24): outside only if **not Rust**; a Rust engine may come
  inside.
- **The GPU target is wgpu, on Metal and Vulkan** (Jon, 2026-09-24).
- **The poured device's opening has a sharp edge today.** The simulated scene keeps it until the
  lip-radius upgrade lands; the upgrade applies to every silicone (Jon, 2026-09-24).
- **The scan's path is the rigid pose fitted to sliding along the canal** (Jon, 2026-09-27; U3, soft-contact recon
  §16t). The contact-guided scan is under Later.
- **Say "room the wall must make", never "overlap".** Distances are to the undeformed wall; Jon read
  "overlap" as the scan passing through the silicone.
- **The fit-test flow and D1–D5** (Jon, 2026-09-23; §4, Phase 2). In short:
  - comfort is the peak push force over the path plus the seated pressure on the most-loaded 1 cm²
    (2026-09-26; first the 95th percentile), with limits from published measurements (2026-09-24; originally
    Jon's casts);
  - the test advises and never blocks;
  - a search for the tightest inset that fits runs on request;
  - the budget is 5 minutes per run and 15 per search;
  - **Try at *n* mm** sits on the fit screen.

---

## 7. What nobody knows yet

- **U1 — "Comfortably."** The quantities and the method are decided (D1). The limits are not: they come
  from published measurements (D1, 2026-09-24), which still have to be gathered (soft-contact recon §8).
  Pressure-pain thresholds for the relevant tissue may not be published at all *(2026-09-27: one study was found,
  below)*. Jon, 2026-09-24: *"this
  we will have to dive into science to see whats out there"*. So the research comes first (soft-contact
  recon §15g step 9), and any fallback is decided after it.
  *Research round, 2026-09-27* (each source's text is kept outside the repo):
  - **The relevant tissue's pressure-pain threshold was measured with a 1 cm² tip**, the area D1's patch reads
    over ([Davis 2011](https://doi.org/10.1111/j.1743-6109.2010.02041.x)). In 46 healthy men it was 2.33 and
    2.37 kg/cm² at the two sites on the relevant tissue (about 230 kPa, arithmetic), with a standard deviation of
    about two thirds of the mean; in 55 men with a chronic pain condition, 0.99 and 0.92 kg/cm². It is the onset of
    pain as the pressure was slowly raised, at rest: not discomfort, not pressure held for minutes, and not the
    tissue's state in use. It was read in the author's thesis, which reprints the paper; the thesis chapter's own
    table differs slightly (2.45 and 2.49 at the same two sites), and why is not stated.
  - **Discomfort comes first:** at 48–62 % of the force that first hurt, on the thigh, chest and hip with a
    1 cm² tip ([Kozinc 2021](https://doi.org/10.1016/j.apergo.2021.103379)).
  - **Holding it matters:** held at 90 % of each person's discomfort level, at one site and through pads,
    discomfort came after 97 ± 91 s, from 10 s to the 5 min cap (Kozinc 2021).
  - **A larger loaded area lowered the threshold in kPa** on the thigh, chest and hip (20 cm² read 8–13 times lower
    than 1 cm²; Kozinc 2021, arithmetic on its tables) and on the foot (2 cm²'s medians were 53–63 % of 1 cm²'s in
    healthy controls; [Chantelau 2016](https://doi.org/10.3402/dfa.v7.31922)). An algometer loads 1 cm² with the
    skin around it unloaded, while D1's patch sits inside a larger contact. Neither the effect's direction nor its
    size is measured on the relevant tissue; if it holds there, a threshold taken at 1 cm² is not conservative for
    the patch on its own.
  - **Not found:** a discomfort threshold of the relevant tissue, any pressure held on it for longer than a ramp,
    or any threshold in its state of use.
  - The limits are Jon's to set (D1).
- **U2 — μ.** ~~No friction data for silicone × lubricant × skin has been found in the repo.~~
  *2026-09-27 (soft-contact recon §5c, re-sourced):* silicone on skin with a lubricant has been measured on the
  forearm, and over time in one subject: a water-based gel and a silicone lubricant started low (0.18 and 0.30 at the
  onset of sliding) and were at or above the dry value (0.94) within 5–20 min. None was found on the relevant skin,
  and none wet. Measured on skin, the pairings span about 0.1 to 1.2; tacky analogs read above 2, so the top is not
  bounded. Most of the range is above μ_f 0.3, where the solver's friction is not yet trusted (soft-contact recon
  §15g's list for steps 6–9).
- **U3 — Why the rigid path asks 8.3 mm of room** with no inset, near the entrance. It is settled before the
  new solver runs `base_mold` (soft-contact recon §15g step 6).
  *Answered 2026-09-27 (soft-contact recon §16t):* most of it is the path's own. The path turns the whole scan
  about its tip by the centreline's turn there. On a circular arc in a plane that carries the rest of the scan
  along the canal; where the curvature changes it does not. On `base_mold`, a rigid pose fitted to sliding along
  the canal asks about a quarter of the most room, and sliding while bending to follow the canal less still. Which
  part of the path's geometry makes the difference is not isolated. **Decided (Jon, 2026-09-27):** step 6 builds
  the fitted pose as the scan's path. It stays prescribed, so no rigid–soft coupling is needed; the contact-guided
  scan moves to Later.
- **U4 — Drake.** Whether its deformable material models fit this silicone.
- **U5–U8** (what the oracle supports, why element 516 inverts, midsides, the step size once CCD exists)
  belonged to the replaced solver, and do not carry over. Element inversion on the new solver is the
  soft-contact recon's K4.
- **U9 — The comfort readouts on a rigid scan.** A rigid intruder puts all the squeeze into the silicone;
  how far that is from soft tissue is unmeasured. *(2026-09-27, soft-contact recon §16t: a rigid scan's path also
  asks room that a scan bending to follow the canal would not: the fitted pose against the slide.)*
- **U10 — The mouth rim. Answered (Jon, 2026-09-24):** the poured device has the same sharp edge.
  It is fine in Ecoflex 00-30, and a comfort issue in Dragon Skin 10A and firmer. *(2026-09-26, soft-contact
  recon §16s: the one report of seated comfort from use; whether the 1 cm² patch reading agrees with it waits on
  D1's limits and a run on the product.)*
- **U11 — How a range becomes a verdict. Decided:** an engineering call (Jon, 2026-09-24: *"completely
  your call"*).
  - *Fits* if the whole interval across the corners is under the limit, and *Too tight* if all of it is
    over.
  - Otherwise the verdict names the corner that crosses the limit.
  - D3's search judges each inset at the pairing's nominal corner, so its bisection stays binary. The
    interval is shown alongside. *(2026-09-27, recon §5c, §16w: no nominal is sourced, and the pairing library holds
    none; which value D3 judges at is open.)*
  - Verdicts use the virgin state, the stiffest and so the conservative one. The Mullins-conditioned
    state is reported once per design, not run for every verdict.
- **U12 — The product's outer boundary. Answered** (Jon, 2026-09-24): no shell today, so the outside is
  free. A shell, bonding to one, and several layered silicone shells may come later (soft-contact recon
  §9 decision 10). *(2026-09-27, recon §16w: a rigid shell bonded to the wall is built; a case it slides along and
  layered shells wait on a PR of their own, before any verdict that uses them.)*
- **U13 — D4's unit of time. Answered** (Jon, 2026-09-24): *"i just mean i want a fast simulation. but i
  dont want to sacrifice quality."*
  - The 5 minutes is a target, measured per press (one verdict, about 3 simulations).
  - It is driven down, and the quality gates are never loosened for it (soft-contact recon §9 decision
    12).
- **U14 — How the device is held in use. Answered** (Jon, 2026-09-24): *"it really depends, it could be in a
  shell, connected to something like a robotic arm, or just held in the hand."*
  - Holding is a design option: a shell, a mount, or a hand as a soft, distributed support
    (soft-contact recon §9 decision 11).
  - A shell or a mount at the closed end confines the material there.
  - *(2026-09-27, soft-contact recon §16w.)* The lowering holds the wall by a mount or a rigid shell bonded to it,
    and step 7's first run is mounted at the closed end (Jon). A case the wall slides along, the hand, and bonded
    layers' interface rule wait on a PR of their own, before any verdict that uses them.
- **U15 — The silicone's damping and its rate dependence** (soft-contact recon §16p, 2026-09-25).
  - The solver now gives the material a viscosity, set from Ecoflex 00-30's published loss near 190 Hz. Without
    it the frictional tube fluttered, and its push read 15–18 % below μ_f·Σf_n at λ_a 1.1 (50k, 100k) and 29 %
    at λ_a 1.3 (50k); with it, 1.4–3.9 % below (50k, 100k). **No loss data was found for Dragon Skin 10A**,
    the product's silicone. One published fit's error bars put Ecoflex's own value at 5.2–10.3 Pa·s.
  - The frictional seated pressure depends on it: between undamped and damped, the tube's seated 95th percentile
    moved −20 % on the 10k mesh and +4 % on the 50k. *(2026-09-26, soft-contact recon §16s: the percentile is no
    longer the deciding reading; how the 1 cm² patch depends on the damping is not measured.)*
  - With it, the tube still fails the Coulomb push at μ_f 0.6 (0.934 against 5 %). U2's friction range decides whether
    the product's friction reaches that *(2026-09-27: U2's range reaches it)*.
  - *2026-09-27, research round:* **no loss data for any Dragon Skin 10 was found**, nor for any Slacker-softened
    silicone. Its only dissipation figures found are a rebound resilience of 56 %
    ([Emminger 2025](https://doi.org/10.1016/j.jmrt.2025.08.181)) and a stress relaxation of about 6 % over
    30 min ([Porte 2024](https://doi.org/10.1089/soro.2023.0004), read from its Fig. 6b). Ecoflex 00-30's value
    stands: four fractional fits from one group (one a preprint) give 6.6–7.5 Pa·s at
    190 Hz, and a DMA about 8.7 at 181 Hz, the top of its sweep ([Zhang 2025](https://doi.org/10.1073/pnas.2520808122),
    taking G as E/3) (arithmetic from each). A constant viscosity matched
    at 190 Hz gives an eighth of Ecoflex 00-30's measured loss at 10 Hz and a thirty-eighth at 1 Hz (Croquette's
    rheometer, read from its Fig. 7; arithmetic). At the rates of an insertion lasting seconds, the model's
    damping is far below Ecoflex 00-30's measured loss; how that moves the push force is not measured.
  - The same published fits stiffen the material by 12–25 % and 43–68 % between 1 and 10 Hz (arithmetic from
    each fit). The solver keeps the static modulus and adds only a viscosity, so it does not carry that
    stiffening. How much it raises the push force at the product's own insertion speed is not known.
  - *2026-09-25 (soft-contact recon §16q):* integrated explicitly, as the solver does (the viscous force at the
    half step behind), its cost grows on small elements. Its damping grows as the inverse square of the element
    size, and on the friction benchmark's block it shrinks the stable step 19× at 83 µm elements and 80× at
    20 µm, against 12–28 % on the tube's meshes. The product's element size is set in 2d; an implicit viscous
    step or a Maxwell branch would change this cost.
  - *2026-09-26 (soft-contact recon §16r):* on `base_mold`'s wall at the element size K2 needs, at ν 0.49 and Ecoflex
    00-30's η/μ, the viscosity cuts the stable step to 0.67 of the elastic one, and a press takes 0.43 of D4 with it
    (0.29 without) if the GPU meets K1, and 0.10 on the CPU. Across Croquette's error bars' 5.2–10.3 Pa·s it takes
    0.39–0.53. *(2026-09-27, recon §16v: read on the old wall before the mesher's Parity Rule fix; on step 7's wall at
    the 5 mm inset a viscous press takes 0.42 of D4 at K1's rate, and 0.59 at ν 0.495 with 1.47 times the viscosity.)*
    *(2026-09-28, recon §16x: at the loading D1's readings need, four times the budget's, a press takes 0.38 of D4 on
    the CPU at h_K2 and 3.8 at four times its elements; the viscosity's range moves the steps 0.83–1.39 times.)*
    *(2026-09-29, recon §16z: 11.8 at eight times its elements, the size used and not picked.)*
- **U16 — D1's readings do not converge on the tube** (soft-contact recon §16p, K5, 2026-09-25). The seated 95th
  percentile moved by 8–12 % from the 50k to the 100k mesh in three of four cases (−2.2 % in the fourth), and the
  frictionless push peak by 14 % (λ_a 1.1). The push with friction converged. §15a sends this back to D1's
  readings, or the lip radius, before step 7.
  *Resolved on the tube 2026-09-26 (soft-contact recon §16s):* by a diagnostic, the percentile's most-squeezed 5 %
  was one or two rings, at the seam and, with friction, the entry edge, and the frictionless push rippled as the
  nose crossed each ring. Jon replaced the seated reading with the most-loaded 1 cm² patch, and the geometric
  share is read over 10 mm of travel (D1). On them K5 passes from 50k to 100k (at most 1.3 %); 100k reads within
  3.6 % of a mesh 4× finer along the tube. **Open:** from 10k to 50k the frictionless patch and the geometric share
  do not converge, and `base_mold` at h_K2 is about the 10k tube's element size, so the element size they need on
  the product is measured in step 7 *(2026-09-28, recon §16x: measured and open, four times h_K2's elements or
  finer; an element collapsing at the seated tip under the mount is not yet settled)* *(2026-09-28, recon §16y: the
  collapse's effect is U20)* *(2026-09-29, recon §16z: not picked, twice h_K2's elements or finer by the size rule, taking
  h_K2's scatter for twice's, and eight times or finer read at 5 % flat)*. The
  lip radius stays under Later.
- **U17 — The product wall's canal surface** (soft-contact recon §16r, 2026-09-26). At the element size K2 needs,
  the old path's wall puts the canal nodes off the true canal surface: its 5th and 95th percentiles at −0.50 and
  +0.32 of an element (into the canal, and into the wall), the worst at 1.25. The scan's decimation is not the
  cause (the undecimated scan gives the same percentiles); between the wall's grid, its pre-smooth and the mesher,
  what does has not been isolated. Projecting the nodes onto the surface brings 95–99 % within 1 % of an element and
  costs the stable step: with the viscosity, a press then takes 1.03 of D4 at K1's rate at a quality floor of 0.5,
  and 8.6 at 0.1 (0.23 and 1.95 on the CPU). Step 6 chooses: a floor, a finer grid under the mesher, or a mesher
  that places the surface nodes itself. *(2026-09-27, soft-contact recon §16t: with the fitted pose, the room the
  path asks with no inset is of the same order as these offsets, so this choice moves a larger share of what step 7
  reads.)* *(2026-09-27, soft-contact recon §16u: this instrument's reference (`explicit_budget.rs`, `Truth`) and
  the wall's grid are signed by a flood fill, which `Truth` documents as reliable only more than a cell (1 mm) from
  the surface; D3's search reaches 0 mm, where the canal is the scan's surface. The choice is measured at a small
  inset too, against a reference signed by parity.)*
  *Answered 2026-09-27 (soft-contact recon §16v):* against a parity-signed reference the old path's offsets read
  the same. Changing one thing at a time at 5, 1 and 0 mm: the grid's pre-smooth carries most of the spread at 5 and
  0 mm, and the flood fill's sign matters at 1 mm; at 5 and 0 mm the grid's own sampling carries most of the rest;
  and once the wall is meshed from the scan's exact distance, what remains at every inset is the mesher's linear cut
  points. With the cut points located
  on that distance, every canal node more than an element from a cap plane lies on the true surface to the printed
  digits at all three insets, and no node is projected, so there is no quality floor. The stable step stays within
  0.86–1.03 of the old wall's at the same inset; a viscous press takes 0.42–0.56 of D4 at K1's rate, and 0.59–0.79
  at the budget's worst corner. That is step 7's wall (an engineering call). The measurement also found and fixed a
  defect in the mesher's Parity Rule, which could leave lattice vertices inside the wall exposed as boundary nodes.
- **U18 — G2 on `base_mold` rests on the scan grid** (soft-contact recon §16r, 2026-09-26). The contact law holds a
  node on the baked grid's surface (measured on the tube, §16o; on the product it needs a run, step 7), so G2 there
  is the grid's own error against the scan. No grid measured meets it: the nearest, 0.25 mm without the pre-smooth,
  lets a node sit 2.3 times the bar deep, and 1.8 % of the scan's points past it. That worst reading halved with each
  halving of the grid's spacing; by that trend, not measured, the bar is met near 0.1 mm, about 100× the 0.5 mm
  grid's samples. The pre-smooth cuts that share
  (to 0.34 % at 0.25 mm) and deepens the worst point (to 3.2 times), which sits beside the cap's rim. Step 6's bake
  sets the grid, and its pre-smooth, against this.
  *Answered 2026-09-27 (soft-contact recon §16u):* those grids were signed by a flood fill, whose sign is wrong
  within a quarter cell of the surface, and the readings above measured that. Signed by the parity of a ray's
  crossings, with no pre-smooth, the grid's own error meets G2 at the 5 mm inset at 0.25, 0.125 and 0.0625 mm, its
  worst point at 0.86, 0.53 and 0.30 of the bar. Step 6's bake adds a fine grid, stored in bricks near the surface.
  *Decided (Jon, 2026-09-27):* G2's bar was 1 % of the inset, so it shrank with the inset to zero at 0 mm, which
  D3's search reaches and no grid meets. It is now 1 % of the inset, but never below 0.02 mm. The obstacle is the
  scan whatever the inset, so a 0.0625 mm grid meets the bar at every inset, a 0.125 mm grid from 3 mm, and a
  0.25 mm grid from 5 mm. *Decided (Jon, 2026-09-27):* the product's fine grid is 0.0625 mm.
- **U19 — The silicone's bulk modulus, under a mount** (soft-contact recon §16x, 2026-09-28). Mounted at its closed
  end, the product's readings rise 1.4–5.3 % with each doubling of K from ν 0.49 to 0.4975, the second doubling's
  change more than half the first's. The silicone's own K is not measured, and §5b's sources put rubbers several doublings
  further on. Which ν a verdict reads at, and whether ν becomes a corner as friction is, trades speed against quality:
  Jon's call. *(2026-09-28, recon §16y: U20 was read at ν 0.49 only; a ν above it re-opens U20.)*
- **U20 — The element collapsing at the seated tip** (soft-contact recon §16y, 2026-09-28). Under the mount, an
  element at the product's seated tip shrinks to 9–16 % of its volume while its nodes stay within 12 % of theirs, a
  motion the element's node-averaged volumetric term does not resist; on the frictionless runs the step is set at or
  near it. Resisting just those elements, the ones under half their nodes' volume (a volumetric stabilization on them
  alone, whose own change of stiffness, of unknown sign, is in the readings):
  - at four times h_K2's elements, the frictional patches move +1.45 and +0.88 %, and the peak push +0.28 and
    +0.35 %;
  - the frictionless patch moves −0.26 % at h_K2 and +6.44 % at twice its elements; at four times, +5.06 % and
    +5.07 % in the last two runs, which did not clear the rule's cut, set beforehand, so it is not judged.

  On the public case, a looser cut moved the reading several times more; on the product that is not measured.

  Which element the product runs went to Jon (rule 3), and the next PR, D1's element size, waited on it:
  - the element as it is;
  - the stabilized element everywhere, which first needs its own checks, left an element collapsed at 2μ, moved the
    readings +13 to +22 % at 8μ (at h_K2), and whose cost is not measured;
  - or first one more run to settle the frictionless comparison at four times, past the sources' range.

  Rule 1's re-runs apply to whichever element runs. At twice h_K2's elements the frictionless run failed at 500 as it
  is and with its collapsing elements stabilized; the stabilized element everywhere was not run there.

  *Decided (2026-09-28): the element as it is.* Jon first chose it on a summary that understated the frictionless
  change at four times h_K2's elements; told of the correction, he left the call to me, and I keep it (recon §16y).
  With the collapse resisted, the highest friction's patch, the top of U11's interval, moves at most about 2 %. The
  element as it is reads the top lower at every size and the bottom lower at twice h_K2's elements, so a *fits* within
  about 2 % of a limit could be a wrong verdict. Every such change includes the stabilization's own change of stiffness, of unknown sign.
  Carried forward: the next PR reads D1's element size with this element and reads the collapse's change again there,
  at every corner; runs that fail with the loop's 500-step re-estimate are made again every 50 steps, and the product
  loop's step control is an open item before the GPU.
  *(2026-09-29, recon §16z: read again at eight times h_K2's elements, the size used and not picked, every corner's
  collapse cleared and each masked change is within 5 %, at most +4.11 %, on the frictionless patch: U20 stands there.
  At twice h_K2's elements, which the size rule does not exclude, the masked change was +6.44 % (recon §16y), and at
  four times not judged. At eight times, resisted reads the top of the corners' interval 0.31 % higher, so a *fits*
  within 0.31 % of a limit could be false, and the bottom 4.11 % higher, so a *too tight* within 4.11 % of a limit
  could be missed. Each includes the stabilization's own change of stiffness, of unknown sign.)*
---

## 8. References

- **Probes** (`tools/cf-sim-research/src/insertion_sim.rs`, all `#[ignore]`d, run locally with the scan):
  - `what_the_sliding_contact_reaches_on_the_product_scan`
  - `the_sliding_bridge_as_written_on_the_product_scan`
  - `the_ui_pipeline_runs_the_bridge_end_to_end_on_the_product_scan`
  - `the_bridge_ramp_over_a_stiffness_sweep_on_the_product_scan`
- **Gate:** `the_heat_map_reads_the_deformed_view_at_rest_positions` (`insertion_sim_ui.rs`).
- **Docs:** [`INSERTION_SIM_TET10_RENOVATION_RECON.md`](INSERTION_SIM_TET10_RENOVATION_RECON.md) (§8 items 3–5, `THE SLIDING MODEL ON
  THE PRODUCT SCAN`); `docs/archive/F4_FALSIFICATION_POSTMORTEM.md`; `sim/L0/soft/src/contact/ipc.rs`
  module docs.
- **Memory:** `project_insertion_sim_architecture_gaps`, `project_insertion_sim_answers_can_it_slide_in`,
  `project_insertion_sim_renovation`, `project_app_sdk_separation`,
  `project_cendrillon_slint_to_bevy_migration`.
