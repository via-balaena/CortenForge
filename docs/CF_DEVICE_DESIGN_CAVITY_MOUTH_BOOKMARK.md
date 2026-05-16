# cf-device-design Cavity Mouth Opening — Bookmark

**Status**: RECON COMPLETE 2026-05-16 — spec at
[`CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`](CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md).
Implementation pending (5-sub-leaf ladder; spec calls it as the
next session per the three-session pattern).

**One-line structural finding from recon**: post-MC half-space clip
alone CANNOT open the cavity — the cavity iso surface for `iso = -T`
sits entirely on the body-interior side of the cap plane, so a half-
space test discards nothing. Recon settled on a **two-SDF
construction** (closed-body SDF for sign, dome+wall-only SDF for
magnitude) that surgically removes the cap polygon's influence on
the inward offset. The user-pinned approach (cf-scan-prep untouched,
each iso held at the cap plane, knife-edge OK) is preserved.

The rest of this bookmark is preserved as the original problem
write-up + design framing. Spec linked above is the source of truth
for implementation.

---

**Status (original)**: BOOKMARK 2026-05-16 EOD (post-clip-plane arc ship). No
spec yet, no recon yet. Captured at session-end because the
implication chain runs deep: this blocks the fit-viz program
(rungs 2–6) and silently invalidates the existing insertion-sim
results.

## Problem (user-observed at clip-plane visual gate)

The cavity (and every layer) is a **closed dome** today — top
dome, cylindrical wall, AND flat-floor cap. For an INSERTABLE
device this is wrong: the appendage needs an opening at the base
to slide in. The cast device would come out as a sealed silicone
pod; you'd have to physically cut it open to make it useful.

This is a **known-deferred feature**. The cavity-state doc in
`tools/cf-device-design/src/main.rs` explicitly calls it out:

```
/// Slice 4 v1: a single uniform inset along the entire scan. A
/// later slice may add the insertable-length clip … so the cavity
/// covers only the inserted portion, with a tangent-perpendicular
/// end cap.
```

The slice-4 author shipped uniform-inset-only as v1 and parked the
mouth feature. It's been latent ever since — invisible until the
clip plane (rung 1) made the closed cavity visually obvious.

## Why this blocks the fit-viz program

Without an open cavity:

- **Rung 2 (per-step playback)** animates the existing insertion-sim,
  which is structurally meaningless against a closed cavity (see
  next section).
- **Rung 3 (scan-as-intruder)** — the scan literally cannot enter
  the cavity. Hard block.
- **Rungs 4–5 (pressure scoring + retraction)** depend on rung 3.

So **the cavity-mouth feature is a prerequisite for rungs 2-6**,
not an independent track. The fit-viz program bookmark should be
updated to reflect this ordering.

## Why the existing insertion sim is silently broken

The slice-7 insertion sim presses a **rigid template intruder**
into the cavity. The cavity is fully closed today. So the rigid
template is pressing into an enclosed dome with no entry path —
either:

- The sim's contact solver is wedging the intruder into the dome
  through some non-physical penetration mode, OR
- The intruder doesn't actually contact anything meaningful and
  the per-tet readouts reflect minor mesh artifacts rather than
  insertion stress.

Either way: **anything the insertion sim panel currently shows is
suspect**. Worth a sanity recheck once the cavity opens —
particularly whether the rigid-template trajectory needs to be
re-anchored relative to the now-open cavity mouth.

## User-pinned approach (2026-05-16, in-session)

After two rounds of framing:

1. **Keep cf-scan-prep untouched.** cf-scan-prep already
   auto-caps open boundaries to produce a closed manifold for
   clean SDF computation. Don't change that.
2. **cf-device-design holds each iso surface at the cap plane.**
   When growing/shrinking the cavity + per-layer iso surfaces, the
   floor stays *at* the original cap plane — it doesn't get offset
   along with the rest of the surface. Mechanically: clip each iso
   extraction against the cap half-space.
3. **Knife-edge brim is acceptable for v1.** Cavity surface and
   outer skin surface both hold to the cap plane, so wall thickness
   collapses to zero exactly at the cap polygon edge. Castable
   (workshop iter-1 should be fine). Smoothing the knife-edge is a
   **post-processing followup**, not a v1 requirement.

### Geometric consequence (illustrated)

Vertical cross-section through the device, perpendicular to cap
plane:

```
   ___                ___
  /   \              /   \         outermost (Layer N outer)
 / / \ \            / / \ \        cavity (inset)
| |   | |          | |   | |       wall thickness = layer_N_offset - (-cavity_inset)
| |   | |          | |   | |
|_|___|_|__________|_|___|_|       ← cap plane (held fixed)
       ↑knife edge: inner & outer meet here
```

(top: closed dome; sides: full-wall silicone; bottom: open at the
cap polygon, walls taper to zero thickness at the cap edge.)

## What cf-scan-prep already gives us

`PrepCapsBlock` is already in `.prep.toml` (per
`tools/cf-scan-prep/src/main.rs:3412`):

```toml
[caps]
applied = true

[[caps.loops]]
loop_index = 0
vertex_count = 247
plane_fit_r_squared = 0.998
plane_normal = [0.02, 0.01, -1.00]      # outward normal at SCAN time (pre-bake)
plane_centroid_m = [0.001, 0.001, -0.045]
included = true
```

Per `PrepCapLoop` (~line 3421):
- `plane_normal` + `plane_centroid_m` define the cap plane.
- `included = true` filters out cap loops the user excluded at
  prep time.
- **Pre-bake frame** — the cap metadata is at *scan time*, before
  the user's Reorient + Recenter transforms. The cleaned STL on
  disk is *post-bake*, and `[centerline].points_m` is also
  post-bake. So cf-device-design needs to bake the `[transform]`
  block onto the cap plane(s) before using them.

The same `[transform]` block cf-cast-cli already consumes:
```toml
[transform.rotation]
quaternion = [w, x, y, z]
roll_deg = …
pitch_deg = …
yaw_deg = …

[transform.translation]
m = [x, y, z]
```

## Implementation sketch (for the recon session)

Rough ladder, ~3-5 sub-leaves:

1. **`.prep.toml` schema lift** — extend cf-device-design's
   `PrepTomlSubset` parser to read `[caps]` (currently it reads
   only `[centerline]` — see `parse_centerline` near
   `tools/cf-device-design/src/main.rs:336-361`). Bake the
   `[transform]` onto each `included` cap plane at load time so
   downstream consumers see post-bake planes. Resource:
   `CapPlanes { planes: Vec<CapPlane> }`.

2. **`sdf_layers::extract_layer_surface` clipping** — extract iso
   surface as today, then trim against each cap plane (discard the
   triangles whose centroids are on the cap-extension side). The
   MC mesh stays open at the cap edge — that *is* the mouth.
   Alternatives worth recon'ing:
   - **Pre-MC zero-out**: bias the SDF grid to push iso past the
     cap plane out of scope. Cleaner geometry; might leave the iso
     boundary as a clean ring polygon.
   - **Post-MC half-space cull**: discard triangles past the cap.
     Simpler; leaves jagged boundary unless re-stitched.
   - Recon picks based on visual quality of the resulting mouth.

3. **Update the SDF-derived `compute_validations`** — the cavity-
   self-intersection + pour-volume computations currently assume
   closed iso surfaces. Open iso surfaces have a slightly different
   divergence-theorem behavior. Quick math: the divergence theorem
   on an open surface gives the volume *of the closed surface* you
   get by attaching the cap polygon. Since our cap polygon IS the
   trim plane, this might just work — but worth confirming.

4. **Insertion sim re-check** — once the cavity opens, re-run the
   insertion sim with the rigid template and visually confirm the
   intruder enters through the mouth instead of magically appearing
   inside. May need to re-anchor the intruder trajectory; check the
   slice-7 entry-point logic in `insertion_sim.rs`.

5. **Clip plane composition check** — the rung-1 clip plane should
   still cut the device cleanly with the cavity open. Should
   compose naturally (the clip plane is a SEPARATE half-space at
   render time; the cap plane is a half-space at MC time); worth a
   visual gate.

## Recon questions (deferred until next session)

1. **Multi-cap-loop behavior.** Iter-1 sock-over-capsule has one
   cap loop (the open base). Body-part scans may have multiple
   (e.g., a residual limb scan might have a proximal cut AND a
   distal cut). When multiple `included` cap loops are present, do
   we clip against ALL of them (device has multiple mouths)? Or
   pick one (and how)? Default guess: all included loops clip
   independently — multi-mouth is fine, weird but consistent.

2. **Pre-MC vs post-MC clipping.** See sketch step 2. Pre-MC
   probably gives cleaner geometry but needs SDF grid manipulation
   that may be costlier per slider tick.

3. **Validation behavior on open surfaces.** Quick math should
   confirm but worth a unit test before shipping.

4. **Cap-edge mesh quality.** The trim boundary may produce jagged
   triangle edges. A polish pass that re-projects boundary
   vertices onto a perfect cap polygon might be needed.

## Post-processing followups (deferred, captured for v2+)

- **Knife-edge brim smoothing.** Round the inner + outer cavity
  edges at the cap plane so the wall thickness tapers smoothly
  toward the cap polygon rather than meeting at a knife edge. Both
  surfaces still meet at the same plane (so the opening stays
  knife-edged geometrically); just the cross-section curvature is
  softer. User-confirmed as a separate post-process step, not a
  v1 blocker.

- **Rim / lip slider.** If a workshop case wants a chunky rim at
  the opening (grip, rigidity), add a per-layer "rim thickness"
  slider that extrudes the layer past the cap plane by a small
  amount. Different from the knife-edge smoothing — this actually
  changes the manufactured shape.

## Suggested session shape

Per [[feedback-bookmark-when-surface-levers-exhaust]] three-session
pattern:

- **This session (BOOKMARK)**: capture above ✓.
- **Next session (RECON)**: cold-read this bookmark, answer recon
  questions 1-4, write a spec at
  `docs/CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`. May need to
  spelunk `sdf_layers::extract_layer_surface` + the marching-cubes
  primitive to pick clip approach.
- **Session after that (IMPLEMENTATION)**: ship the spec's
  sub-leaf ladder.

Each rung lands as its own PR if PR #248 has shipped by then;
otherwise stacks into the same PR.

## Start-of-next-session checklist (cold read)

When the user wants to start the recon:

1. `git log --oneline -5` — confirm dev HEAD at the
   clip-plane-shipped commit `d8a6ef3c` or later.
2. **Read this bookmark IN FULL.** It IS the recon prompt.
3. Re-read `tools/cf-scan-prep/src/main.rs:3275-3431` for the
   `PrepCapsBlock` schema, including the `orient_cap_normal_outward`
   convention (the `outward` direction needs to be unambiguous for
   clipping).
4. Re-read `tools/cf-device-design/src/sdf_layers.rs::extract_layer_surface`
   for the MC extraction surface to clip against.
5. Pick the clip approach (recon Q2), write the spec.

## Related memory

- [[project-cf-device-design-fit-viz-bookmark]] — parent program.
  Rungs 2-6 are BLOCKED on this bookmark shipping; update the
  parent bookmark to flag this as a prerequisite (this bookmark's
  ship commit should do that).
- [[project-cf-device-design-clip-plane-arc]] — the rung 1 ship
  that surfaced this issue at the visual gate.
- [[project-cf-device-design-slice-7-plan]] — the insertion sim
  this cavity-mouth feature implicates.
- [[project-cf-scan-prep-cap-winding-concern]] — adjacent issue
  in cf-scan-prep's cap-orientation logic. Worth re-reading
  during the recon to confirm the cap normal in `PrepCapsBlock`
  is the orientation we expect.
