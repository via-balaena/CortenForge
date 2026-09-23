# The fit test — plan to work against

**Status:** open. Written 2026-09-23 at `6ebb5a3e` (branch `heat-map-frame-and-sliding-contact`, PR #963).
**Scene:** the product scan `base_mold` — 5 mm inset, 17 mm Dragon Skin 10A at 25 % Slacker, curved
centerline 120.9 mm long. ⛔ It is a sensitive anatomical scan: probes load it from outside the repo,
and it must never be committed.

**How to use this document.** Section 1 is what the fit test must do. Sections 2–3 say where it stands and
why. Section 4 is the plan, with a *done when* for every step. Section 5 lists the gates each step must keep.
Section 6 lists decisions not to relitigate. Section 7 lists what nobody knows yet. Tick items as they
close, and update the numbers in place with their referent. **Every claim here names a file, a probe or a
document. Leave anything else out.**

Companion: [`INSERTION_SIM_TET10_RENOVATION_RECON.md`](INSERTION_SIM_TET10_RENOVATION_RECON.md) — the
measurement history behind every number below.

---

## 1. What the fit test is for

In Jon's words (2026-09-23):

> "the inset of the hole will have different insets depending on the parameters chosen by the user. the
> physics is for simulating how the scanned object […] will do against the silicone (00-30 vs dragonskin
> 10, etc) and different insets. if the inset is too big/hole is too tight where it cant comfortably (we
> will have to define this threshold) go in, then the inset should be reduced. this sim is for a user to
> fine tune their adaptive pleasure device to be perfect for their scanned anatomy so they dont have to
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
- [ ] **R5 — "Comfortably" is a threshold Jon will define.** Section 7, U1.
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

### 2a. What the user sees today (`tools/cf-sim-research`, the research viewer)

| Symptom | Cause | Referent |
|---|---|---|
| The scan is always inside the cavity | A still copy of the scan is drawn at the SEATED pose by default | `ScanMeshVisible::default()` is `true`, pinned by a test at `main.rs:2207` |
| With the bridge ticked, nothing moves; the wall just swells | The bridge forces the GROWING model | `kick_off_simulation`, `insertion_sim_ui.rs:707` |
| The moving scan appears only sometimes | It is drawn only in sliding mode with Show-deformed on | `visible_pose_for_intruder`, `main.rs:1097` |
| No frame shows the scan outside | The first recorded step is t = 1/n. At 16 steps the tip is already 7.56 mm (120.94 / 16) past the entrance | `slide_pose_at`; `DEFAULT_N_STEPS = 16`, `insertion_sim_ui.rs:67` |
| **The scan shows through the wall** | The panel draws the FULL-SIZE scan (`spawn_intruder_mesh`, `main.rs:989`), but the physics pushes the wall with the scan SHRUNK by the inset | Measured, next row |
| ↳ measured | Shipped default (penalty sliding, 16 steps): **12/16** steps converge, reaching 90.7 of 120.9 mm. At that step **822 of 1 684** drawn wall vertices sit inside the drawn scan, **720 deeper than 1 mm, deepest 6.06 mm** | A temporary probe through `run_sim_pipeline`, reverted. Recorded in memory `project_insertion_sim_answers_can_it_slide_in` |
| The heat map is slow on the bridge | It colours every solved-mesh node (125 575), though only 5 261 are drawn: **4.08 s per call** | Recon, `PER-GAUSS-POINT READOUTS` |
| The wall looks faceted | The deformed view draws corner triangles; the Tet10 midside curvature is not drawn | Recon, same section |
| The drawn wall is smoother than the part that gets poured | The simulated cavity is a smooth offset; the poured plug has three ridge rings (1.8–2.0 mm deep), texture, side pinch and tip relief | `product_scene` doc, `insertion_sim.rs` |

### 2b. The physics

| Gap | Referent |
|---|---|
| **Neither sliding ramp carries the inset.** The shipped `run_sliding_insertion_ramp` passes `interference_m = 0`. F4's warm-up, which ramped the inset in, was reverted. The Tet10 bridge (`run_sliding_insertion_ramp_tet10_ipc`, never wired) offsets the scan by `cavity_offset_m` alone. Fully seated, both contacts ARE the cavity surface, whatever the inset | `insertion_sim.rs`; `docs/archive/F4_FALSIFICATION_POSTMORTEM.md`; recon, `THE SLIDING MODEL ON THE PRODUCT SCAN` |
| **Our IPC is v1.** No continuous collision detection (`ccd_toi` returns ∞), fixed κ, one-way rigid kinematic coupling — its doc calls this "adequate for the small-dt keystone scene" | `sim/L0/soft/src/contact/ipc.rs:31–42, 488` |
| ⇒ **Step-count limits.** Without CCD, each step's closing must stay under the barrier band (d̂ = 1.2 mm). The sliding bridge first accepts **110** steps (as written) and **117** (bare scan); the closing is not monotone in the step count. The growing bridge seats **4.531 of 5 mm** at 32 steps and 3.438 mm at the panel's 16 | `what_the_sliding_contact_reaches_on_the_product_scan`; recon |
| **No friction** (μ = 0). Tet10 face contact asserts on friction (the face-friction reconciliation is a deferred rung). No μ data for silicone × lubricant × skin has been found in the repo (memory `project_insertion_sim_renovation`) | `sim/L0/soft/src/solver/backward_euler/assembly.rs:358` |
| **The intruder is rigid and teleported.** Each step places it at the next pose — turned about its tip to follow the centerline — then solves the wall. That motion alone asks the wall for up to **8.3 mm** of room with no inset (at t = 0.6875, ~14 mm inside the entrance); why has not been isolated | `slide_pose_at`; recon |
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

**The approach is the right family:** hyperelastic finite elements (Yeoh on Tet10) with an IPC contact
barrier — the method of Li et al., *Incremental Potential Contact* (SIGGRAPH 2020), built for
large-deformation contact that never passes through, with continuous collision detection in its line search
and friction. **What is wrong is how ours is built and driven:** no CCD, a fixed κ, no friction, a teleported
rigid intruder, no inset in the sliding contact, and a mesh that inverts at element 516.

**Drake** (the robotics toolkit) has not been checked: whether its deformable bodies and material models
fit a hyperelastic silicone like this one is open (Section 7, U4), so nothing here compares against it
yet.

**Answer it with a measurement, not an opinion** (Phase 1): run the same scene in a validated IPC
implementation — the IPC authors' open-source PolyFEM and IPC Toolkit, whose method includes the CCD and
friction ours lacks; what the libraries support for THIS scene is checked first (Phase 1, U5). This follows
the standing rule: an outside library enters only as an **oracle**, is never shipped,
and must be someone else's validated code (memory `feedback_julia_above_the_line_rust_below`).

- If the oracle slides `base_mold` in at the inset, the approach works. Our gap is the missing pieces, and
  the oracle becomes the reference answer.
- If it cannot, the problem is the scene (mesh, material, the rigid pose model), not our solver.

---

## 4. The plan

Each phase has a *done when*. Phases 1 and 2 can run side by side; Phase 4's order comes from Phase 1.

### Phase 0 — Land what is measured
- [ ] Merge #963 (green, 25/25 at `6ebb5a3e`), with `--body-file`.

### Phase 1 — The reference oracle (outside the repo)
- [ ] Confirm what the oracle supports before exporting anything: the material model (Yeoh? if not,
  which model and how it is matched to ours), rigid-versus-deformable contact, friction, and a prescribed
  rigid path. Record the answers here.
- [ ] Export the SAME scene: the mesh (or an equivalent re-mesh — state which), the material per layer, the
  pinned outer skin, the intruder surface, and the path (`slide_pose_at`). Check the export against ours
  (volume, surface area, pinned count) before believing any result.
- [ ] Runs:
  - (a) inset carried (bare scan), frictionless;
  - (b) the same with μ at two or three values — a bracket, stated as a bracket, until real μ data exists;
  - (c) for comparison, our as-written zero-interference contact.
- [ ] Record per run: how far it gets (depth / t), push force along the path, minimum gap (all surface
  nodes), peak contact pressure, peak silicone stretch, and runtime.
- **Done when:** this section states whether a validated IPC slides `base_mold` in at 5 mm, with numbers
  and their provenance, and the decision rule in §3 has picked a branch.
- ⛔ The scan never enters the repo, and no oracle code or runtime enters the workspace.

### Phase 2 — The fit-test flow, written before more physics
- [ ] Write the user flow for the `cf-studio-gui` step between `DesignLayers` and `MakeMolds`:
  - pick the inset and silicone;
  - watch the scan go in **from outside**;
  - see a push-force-versus-depth curve and a pressure map;
  - get a verdict against the comfort threshold, which, if the fit is too tight, says which inset would
    pass.
- [ ] State what the physics must output for that flow: the per-step quantities, their units, and what
  "too tight" is computed from.
- [ ] Set a **runtime budget** per run. Today: the growing bridge takes 364 s at 16 steps, and the
  sliding bridge as written about 1 070 s at 128 steps.
- **Done when:** the flow is written here and Jon has agreed it, and each output names the physics quantity
  behind it.

### Phase 3 — Make the research viewer honest (cheap, independent)
- [ ] Hide the seated copy of the scan while a sliding run is displayed.
- [ ] Record the outside starting frame (t = 0) so playback starts with the scan clear of the device.
- [ ] Label the model actually run. Ticking the bridge silently switches sliding to growing.
- [ ] Heat map: colour only drawn vertices, or cache each vertex's nearest element per run (rest positions
  are fixed now).
- **Done when:** playback on `base_mold` starts outside and never shows a scan the physics did not move.

### Phase 4 — Close the physics gaps (order set by Phase 1)
- [ ] **Carry the inset in the sliding contact** (the bare scan). Gate: seated, the contact asks the rest
  wall for exactly the inset in room.
- [ ] **CCD** in the line search. Expected (to be measured) to remove the step-count limit in §2b.
- [ ] **Adaptive κ.**
- [ ] **Friction** on Tet10 face contact (replaces the assert at `assembly.rs:358`), plus a μ source for
  silicone × lubricant × skin.
- [ ] **Drive the intruder** instead of teleporting it: decide between a prescribed path with CCD and a
  force- or velocity-driven intruder guided by contact.
- [ ] **Element 516.** F-bar is not available (hard-gated off for Tet10). The realistic candidate is
  mesh-side: finer cells where strain concentrates.
- [ ] **Midside non-penetration** in every post-check.
- **Done when:** the sliding model with the inset seats `base_mold` fully, and G1–G4 (§5) hold, at the
  runtime budget set in Phase 2.

### Phase 5 — Into the main app
- [ ] Extract the simulation from `cf-sim-research` into a library crate on the SDK side.
- [ ] Build the fit-test step in `cf-studio-gui` to the Phase 2 flow.
- **Done when:** the wizard runs the fit test on a user's scan end to end, and the verdict matches the
  research tool's on `base_mold`.

### Later
- [ ] Soft (rigid-ish) intruders: body-to-body contact (R7).
- [ ] Foot into boot (R8).
- [ ] Ridges and texture in the simulated cavity (§2a, last row).

---

## 5. Gates every phase must keep

| Gate | What must hold | Baseline today |
|---|---|---|
| **G1 — Drawn wall outside drawn scan** | No drawn wall vertex inside the drawn scan (beyond a stated tolerance) | Shipped default: 720 of 1 684 more than 1 mm inside, deepest 6.06 mm |
| **G2 — No penetration** | No boundary node, **corners and midsides**, through the contact at any converged step | Corners only: 0 through at the sliding bridge's last step; midsides unchecked |
| **G3 — Full seat** | The full inset is reached along the sliding path | Growing: 4.531 of 5 mm. Sliding with the inset: not run |
| **G4 — κ independent of the schedule** | The derived κ does not change with the step count | Holds (the ceiling rule) |
| **G5 — Heat map in the rest frame** | `the_heat_map_reads_the_deformed_view_at_rest_positions` passes | Passes; fails under four mutations |
| **G6 — Runtime** | Within the Phase 2 budget | See Phase 2 |

G1 and G2 should become committed probes (the scan is repo-excluded, so they run locally and cannot gate
in CI); G5 already gates in CI.

---

## 6. Decisions already made — do not relitigate

- **Sliding, carrying the inset, is the default** (Jon, 2026-09-23).
- **The product scene is `base_mold`**, not `sock_over_capsule` (memory `feedback_confirm_the_fixture_is_the_subject`).
- **κ = the ceiling**, justified by the cushioning requirement and independent of the step schedule; never
  chosen by depth (recon).
- **F-bar is not the fix** for element inversion on Tet10 (hard-gated off; no multi-Gauss-point analog).
- **External libraries are oracles only**, never shipped (memory `feedback_julia_above_the_line_rust_below`).
- **Say "room the wall must make", never "overlap".** Distances are to the undeformed wall; Jon read
  "overlap" as the scan passing through the silicone.

---

## 7. What nobody knows yet

- **U1 — "Comfortably."** Which quantity, and what threshold: push force, contact pressure, silicone
  stretch, or a combination? Jon to define.
- **U2 — μ.** No friction data for silicone × lubricant × skin has been found in the repo.
- **U3 — Why the rigid path asks 8.3 mm of room** with no inset, near the entrance.
- **U4 — Drake.** Whether its deformable material models fit this silicone.
- **U5 — What the oracle supports** (Phase 1, first item).
- **U6 — Why element 516.** It is the same element in both models; mesh-side refinement is the candidate,
  untested.
- **U7 — Midsides.** Whether any midside node goes through the contact.
- **U8 — Step size once CCD exists.** Expected to relax; unmeasured.
- **U9 — The comfort readouts on a rigid scan.** A rigid intruder puts all the squeeze into the silicone;
  how far that is from soft tissue is unmeasured.

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
  `project_insertion_sim_renovation`, `project_pleasure_robot_arc`, `project_app_sdk_separation`,
  `project_cendrillon_slint_to_bevy_migration`.
