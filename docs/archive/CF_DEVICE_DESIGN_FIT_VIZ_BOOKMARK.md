# cf-device-design Fit Visualization + Scoring — Bookmark

**Status**: BOOKMARK 2026-05-16 (post-SDF-layers arc). User-stated
vision; no spec yet, no recon yet. Captured at session-end while
context is hot.

## User-stated vision (verbatim)

> the goal is to basically slide the scan geometry into the cavity,
> and find the best possible fit for it. like if the main geometry
> is a sensor going in, snug pressure against it is a reward, but
> overly tight/hard/overly rigid collision is negative points. no
> collision at all for an area is also negative points (maximum
> good pressure of all surface area at all time basically,
> throughout the insertion and retraction)

And:

> just babystepping our way to my vision through whatever examples
> you see fit is ideal

So this isn't ONE arc — it's a multi-arc program ladder. Each rung
is independently shippable + workshop-useful. The user has granted
autonomous-architect authority across the program (per
[[feedback-autonomous-architecture]]).

## Where we just came from (2026-05-16)

cf-device-design SDF-based layer surfaces arc just shipped on `dev`
(seven sub-leaves `d5f423ec` → `320d6dcd` + docs `dc90c9be`). PR
#248 is open. The cf-device-design preview now renders the cavity +
per-layer outer surfaces as uniform-offset isosurfaces of the
cleaned scan's SDF; the Insertion Sim panel runs against the
corrected geometry; heat-map projection onto the per-layer MC
vertex sets works. See `memory/project_cf_device_design_sdf_layers.md`
for that arc's patterns.

What's in place today that this program builds on:

- **`sdf_layers::CachedScanSdf`** — scan SDF + filled grid, rebuilt
  at scan load. Source of truth for surface geometry.
- **`insertion_sim` module** — Route-A SDF bridge → tet mesh + Yeoh
  materials + `CpuNewtonSolver` + `PenaltyRigidContact`. Currently
  presses a **rigid template intruder** into the cavity.
- **`InsertionRamp`** — quasi-static interference ramp (16 steps by
  default), per-step + per-tet engineering readouts.
- **Heat-map projection** — per-tet Ψ / ‖P‖ on each per-layer MC
  vertex set, rendered via per-vertex COLOR attribute.

## Ladder — proposed baby-step order

### **PREREQUISITE for rungs 2-6**: cavity mouth opening (BOOKMARKED 2026-05-16)

After rung 1 visually surfaced the closed-cavity issue, we
discovered the cavity is fully enclosed (per
`docs/CF_DEVICE_DESIGN_CAVITY_MOUTH_BOOKMARK.md`). Rungs 2-6
cannot ship meaningfully against a closed cavity:

- Rung 2 animates the existing insertion sim, which is silently
  broken against the closed cavity.
- Rung 3 needs the scan to enter the cavity — structurally
  impossible without an opening.
- Rungs 4-5 depend on rung 3.

User-pinned approach: cf-device-design holds each iso surface at
the cap plane cf-scan-prep already records in `[caps]`. No
cf-scan-prep changes. See the cavity-mouth bookmark for full
recon prep.

**Estimated 1 small-to-medium arc (3-5 sub-leaves)**. Must ship
BEFORE rungs 2-6 are worth touching.

### Rung 1: Clipping plane (A) — **SHIPPED 2026-05-16**

**Status**: SHIPPED on `dev` 2026-05-16. Six sub-leaves
(`46421011` → docs commit). Spec at
`docs/CF_DEVICE_DESIGN_CLIP_PLANE_SPEC.md`; arc-end memo at
`memory/project_cf_device_design_clip_plane_arc.md`.

**Pinned UX**: centerline-anchored, lengthways-only (plane contains
the centerline tangent — controls are "position along centerline"
+ "roll around tangent"). Hollow shell (per-fragment `discard`); no
filled cut cap. Both decisions made in-session with the user before
spec; recorded in the spec's "User-pinned decisions" section.

**Pinned approach**: Bevy 0.18 `ExtendedMaterial<StandardMaterial,
ClipPlaneExt>` with an embedded WGSL fragment shader (binding
slot 100 in `@group(#{MATERIAL_BIND_GROUP})`). One uniform-push
system per frame projects the resolved physics-frame plane into
render frame + writes the packed `(vec4, u32)` uniform to every
ClipPlaneMaterial asset; snapshot-and-compare via
`Local<Option<UniformKey>>` plus an `AssetEvent::Added` watcher
for new layers spawned mid-arc.

**Visual gate**: user-verified on iter-1 `sock_over_capsule.cleaned
.stl` — slider sweeps cleanly along the centerline; roll rotates
windshield-wiper style; cavity + layers + scan clip in lockstep.

### Rung 2: Per-step playback animation (formerly "B")

Make the existing insertion sim's per-step deformation viewable.
The ramp already runs 16 quasi-static steps and produces per-step
readouts, but only the FINAL deformed positions are kept (`ramp.result.final_per_tet`).
This rung:
- Retains per-step deformed vertex positions (cheap: 16 × n_verts
  × Vec3<f64>, a few MB on iter-1).
- Adds a timeline scrubber + play/pause to the Insertion Sim panel.
- Per-frame: rebuild per-layer MC mesh from the per-step deformed
  positions, OR (preferred) push deformed positions to the existing
  meshes as a per-frame vertex attribute update.

**Why second**: composes with rung 1 (clipping plane stays useful
during playback — you see the interior deform), and the underlying
sim infra is already in place. The change is mostly retain-more-data
+ UI scrubber + a per-frame mesh-update system.

**Open shape questions** (worth a recon):
- Is the right primitive to animate "MC mesh per step" (re-extract
  per frame — cheap MC on cached SDF, ~ms) OR "deformed proxy mesh"
  (push deformed tet positions to the existing meshes, harder
  because per-layer MC vertices don't have a 1:1 mapping to tet
  vertices)? Probably re-extract — cleaner.
- Wait — the deformed positions ARE the tet vertices, not the MC
  surface vertices. Need a per-step projection from deformed tet
  positions onto the layer surface. That's a real wrinkle the
  recon needs to design around. Possibly: skip the per-layer
  rendering entirely during playback and just show the tet-mesh
  wireframe / cross-section.

Estimated 1 medium arc (5–10 commits).

### Rung 3: Scan-as-intruder

Swap the rigid template intruder for the cleaned scan SDF. The
scan becomes the deformable body sliding into the cavity (or the
cavity deforms around it — depends on the physics framing). This
is the biggest physics change in the program; the rigid-vs-deformable-
intruder split changes contact-pair construction inside `sim-soft`.

**Why third**: rungs 1 + 2 don't need this — they animate the
existing rigid-intruder sim, which is already meaningful. Rung 3
elevates the sim's PHYSICAL FAITHFULNESS so the score (rung 4) is
worth computing. Without rung 3 the score answers "how does the
generic template press into the cavity," not "how does THIS sensor
fit."

**Open shape questions** (real recon territory):
- Does the cleaned scan deform OR stay rigid as it presses into the
  silicone? Both are defensible interpretations of "sensor going
  in"; user's intent is probably RIGID scan + DEFORMABLE silicone
  (the silicone gives, the sensor doesn't).
- If rigid: contact pair becomes "rigid SDF as intruder, deformable
  tet mesh as body." `PenaltyRigidContact` already handles rigid
  primitives; needs an SDF-source rigid variant.
- If deformable: scan needs its own tet mesh + materials; coupled
  contact. Much bigger arc; might be a v2 of this rung.
- How is the insertion KINEMATICS specified? Currently the rigid
  template has a single interference depth. Scan needs a trajectory
  (translation + maybe rotation) into the cavity. UI implications.
- Re-validates the existing FEM solver on the new contact pair —
  may surface convergence issues sim-soft's contact-pair history
  didn't catch.

Estimated 1 large arc (10–15 commits), preceded by a real spec.

### Rung 4: Per-surface pressure scoring

Convert per-tet stress to per-cavity-surface-area pressure. Define
a goodness band:
- `pressure ≈ 0` → bad (no contact, sensor slop).
- `pressure ∈ [low, high]` → good (snug fit).
- `pressure > high` → bad (crushing / rigid impingement).

Compute coverage = fraction of cavity surface in the goodness band,
integrated over the ramp (and later, over insertion + retraction —
rung 5). Surface this scalar (or per-region heat map) in the
Insertion Sim panel.

**Why fourth**: needs the actual sensor geometry (rung 3) to be
meaningful. Currently the rigid template would score whatever the
template-cavity interaction is, which doesn't tell the user
anything about THEIR design.

**Open shape questions**:
- What pressure metric? Surface-normal component of `‖P‖` on cavity
  surface? Or contact force per unit area from `PenaltyRigidContact`'s
  contact log? Latter is more direct but contact-discretization-
  dependent.
- What goodness band? Workshop-iter-1 case probably has a Shore-
  hardness-derived band that the user could later dial. Default:
  guess + iterate.
- Score over TIME or over FINAL STATE? User said "throughout the
  insertion and retraction" so time-integrated. Means rung 5 is
  almost a prerequisite.
- Visualization: scalar overlay (red/green per-surface-area patch),
  OR rolled-up score readout, OR both.

Estimated 1 medium arc (5–10 commits), needs rung 3 to land first.

### Rung 5: Retraction phase + over-cycle score

Run the ramp backward after insertion. Track the score across the
full insert+retract cycle. Possibly add per-step plot of score vs
ramp-step (so user sees WHERE in the motion the fit goes bad).

**Why fifth**: rung 4 alone can score the final state; rung 5
extends to the full cycle the user actually cares about. Cheaper
than rung 4 (mostly: run the ramp backward, accumulate the score).

Estimated 1 small arc (3–6 commits).

### Optional rung 6+: auto-search optimization

Sweep design parameters (cavity inset, per-layer thickness,
material choices) to maximize the score. UI: "Optimize" button +
constraint envelope (e.g. "keep total wall ≤ 15 mm").

**Why last**: the score has to be correct + the sim has to be fast
enough to evaluate many candidates. Both are downstream of rungs
3 + 4. Possibly never built — manual iteration with the score
readout might be enough.

## Suggested session shape

Per [[feedback-bookmark-when-surface-levers-exhaust]] three-session
pattern, each non-trivial rung gets its own bookmark + recon +
implementation:

- Rung 1 (clip plane): probably small enough to skip the bookmark
  pre-step and just recon → spec → implement in 1–2 sessions.
- Rung 2 (playback): recon-then-implement, 2 sessions.
- Rung 3 (scan-as-intruder): bookmark + recon + spec + implement,
  3+ sessions. This one is real architecture.
- Rung 4 + 5: probably combine into 1 spec since they're tightly
  coupled; recon together.

Each rung lands as its own PR if PR #248 has shipped by then;
otherwise stack into the same PR through the SDF-layers ship.

## Start-of-next-session checklist (cold read)

When the user wants to start the next arc:

1. `git log --oneline -5` — confirm dev HEAD at `dc90c9be` or
   later. If a workshop iter-1 cast happened in between, there may
   be cf-cast-cli commits; that's expected.
2. **Read this bookmark IN FULL.** It IS the recon prompt.
3. Pick a rung. Default: **rung 1 (clip plane)** — cheapest warm-
   up, decouples from the bigger physics questions.
4. For rung 1: explore Bevy clip-plane patterns (material shader
   uniform vs custom pipeline), pick the cheapest approach that
   composes with the existing per-layer mesh entity model, write
   a small spec at `docs/CF_DEVICE_DESIGN_CLIP_PLANE_SPEC.md`.
5. For rung 3 (when ready): real recon territory. Re-read sim-soft
   contact-pair code + `PenaltyRigidContact`. Open separate
   bookmark at `docs/CF_DEVICE_DESIGN_SCAN_INTRUDER_BOOKMARK.md`
   before specifying.

## Open / cross-cutting questions for whichever rung lands next

- **Rendering performance** when both clip plane + animated playback
  are on. Per-frame MC re-extraction is cheap (< 1 ms / layer at
  5 mm cell), but 16 frames × 6 layers × MC + per-vertex projection
  could pile up. Worth measuring early.
- **Cross-section CAP face rendering** — when the clip plane cuts
  through a layer mesh, the user probably wants to see the layer's
  CROSS-SECTION as a filled colored region (like a CT scan slice),
  not a hollow shell. This is non-trivial — needs an inside-the-
  layer fill, possibly via a second-pass shader OR a CPU-side mesh
  trim. Flag for rung 1's recon.
- **Sensor geometry source** for rung 3. Is the "scan" the same
  cleaned scan that the cavity is built from (= "fit the device
  back onto the body part it was scanned from"), OR a SEPARATE
  scan of the actual sensor that'll live in the device (=
  "fit the device around this sensor")? The user's phrasing
  "main geometry is a sensor going in" suggests the latter — a
  different scan than the body-part one. Has UX implications:
  second STL file input, possibly a "sensor STL" alongside the
  "body part STL."

## Related memory

- [[project-cf-device-design-sdf-layers]] — the arc that just
  shipped + the patterns banked from it.
- [[project-cf-device-design-slice-7-plan]] — the original FEM
  insertion sim plan from 2026-05-14; the rigid-template intruder
  + per-tet readouts are its outputs.
- [[project-bevy-is-changed-footgun]] — load-bearing for any system
  that watches sim_state.last_run changes (which the playback
  rung 2 will need).

## Why this bookmark exists

Captures the user's vision verbatim while context is hot, so the
next session can cold-read without me having to re-elicit the
goal. Also pins the ladder order I proposed + the user agreed to,
so future-me doesn't accidentally start with the wrong rung.
