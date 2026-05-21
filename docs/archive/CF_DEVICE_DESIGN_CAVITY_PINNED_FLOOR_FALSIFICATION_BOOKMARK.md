# cf-device-design Cavity Pinned-Floor — Scope-C Implementation FALSIFIED at Visual Gate #1

**Status**: Implementation shipped per scope-C spec (`docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_SPEC.md`), 6 sub-leaves on `dev` (`ccf9c7af` → `fdfeee1d`). Visual gate #1 FALSIFIED 2026-05-16 LATE-EVENING — the cavity has no flat floor at the cap plane; the construction the spec banked on cannot produce one.

Three-session pattern, second iteration: bookmark → recon → implementation, this time targeting a *new* SDF construction that actually delivers the user's geometric model. Bookmark goes here; recon session writes a new spec; implementation session lands the redesigned construction.

**Parent docs** (cold-read in order):
1. This bookmark.
2. `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md` §1 — user's verbatim geometric model. LOAD-BEARING.
3. `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_SPEC.md` (scope-C) — what shipped, including §1 Q2 (composition mechanism), §1 Q5 (two-SDF disposition), §7 (open risks). NOTE: the spec is wrong — its construction can't deliver the geometry; this bookmark captures why.

**Blocks**: cf-device-design preview correctness, insertion sim force-readout correctness, cf-cast-cli mold geometry correctness, fit-viz rungs 2-6 (re-blocked).

## What shipped + commit ladder

6 implementation commits on `dev`:

| # | Commit       | Crate                       | What                                                                 |
| - | ------------ | --------------------------- | -------------------------------------------------------------------- |
| 1 | `ccf9c7af`   | `cf-cap-planes` (NEW)       | Lift CapPlane / parse_cap_planes / dome_wall_only_mesh / diagnostic + NEW DomeWallSignedSdf adapter |
| 2 | `27851305`   | `cf-device-design`          | Rewrite `extract_layer_surface` to pinned-floor per-extract composition |
| 3 | `af7348fa`   | `cf-device-design`          | Revert `signed_volume_m3` to single-arg + delete `primary_cap_origin` |
| 4 | `2077c25f`   | `cf-design`                 | NEW `pinned_floor_shell<S: Sdf + 'static>` composite primitive       |
| 5 | `c1994571`   | `cf-device-design` (sim)    | Plumb insertion_sim cavity + outer via pinned_floor_shell           |
| 6 | `fdfeee1d`   | `cf-cast-cli`               | Plumb plug + per-layer bodies via pinned_floor_shell (LAST + rollback target) |

Test counts on dev: cf-cap-planes 19 / grade A; cf-design 776 + 2 ignored; cf-device-design 134 + 9 ignored; cf-cast-cli 14 derive + 6 integration. All release builds + clippy clean.

## What we expected at gate #1 vs. what we got

**Expected** (per user's geometric model): cavity = closed shell, dome top (shrunk inward by inset_m), cylindrical side walls (shrunk inward by inset_m), **flat floor pinned at cap plane**.

**Got** (image #5, cavity viewed alone with scan mesh + layers hidden): closed dome top, cylindrical side walls, **wavy/rolled rim hanging below cap plane**. No flat floor. Marching-cubes artifacts where the dome wall meets the cap-plane half-space clip.

The cavity is OPEN at the bottom (truncated at the cap plane), not CLOSED with a flat floor.

## Why the spec's construction can't produce a flat floor

The spec's pinned-floor primitive composes:

```
composed_sd(p) = max(grid_value(p) − offset_m, max over caps of cap_sd(p))
```

Where `grid_value(p) = sign(sd_closed(p)) × |sd_open(p)|` — the two-SDF construction the cavity-mouth arc shipped (sub-leaf 3, commit `544a8905`).

The fatal asymmetry: at points on the cap plane in the body's lateral interior (e.g. at body center on cap plane):

- `sd_closed(p)` is on the closed body's boundary (the cap polygon IS part of the closed mesh), so `sd_closed(p) ≈ 0`. Rust's `f64::signum(0) = +1.0`, so `sign = +1`.
- `sd_open(p)` is distance to the **dome wall** (since the cap polygon has been stripped from the open mesh). For the iter-1 sock fixture, this is ≈ **body_radius ≈ 35 mm**, not 0.
- `grid_value(p) = +1 × 35 = +35 mm`.

So on the cap plane at body center, `grid_value = +35 mm`. After offset shift for cavity at offset = -3 mm:

- `composed_sd = max(+35 − (−3), 0) = max(+38, 0) = +38 mm`. **OUTSIDE cavity**, by 38 mm.

For ANY point on the cap plane in the body's lateral interior, `composed_sd > 0` (outside cavity). The cavity NEVER has its iso=0 surface on the cap plane. Therefore the cavity has NO floor at the cap plane.

What we see in image #5 is the iso=0 surface terminating where the dome wall meets the cap-plane half-space clip — a "knife edge" at the cap rim, NOT a flat floor across the cap plane interior. The wavy/rolled appearance is marching-cubes sampling artifacts at the SDF discontinuity at the cap plane (grid_value jumps from `−body_radius` just below the cap plane to `+body_radius` just above, across the cap polygon).

**This is a structural property of the sign × |magnitude| construction**, not a tuning issue. No epsilon nudge, no half-space tweak, no MC config setting can fix it — the cavity floor LITERALLY isn't in the SDF.

## What the user's geometric model actually requires

Re-quoting verbatim from the parent bookmark §1:

> The cavity spawns a shrunk-down version of the cavity [scan] in every way except for the floor coming up. Every other atom on everything that isn't the flat floor can grow and shrink. Attached to this cavity is the wall. The wall is walls off of the cavity that dynamically grow and shrink as the cavity grows and shrinks. The wall also grows and shrinks for every atom that is not the flat floor, that's fixed to the plane (cavity's flat floor is fixed to the plane too). Then, for every layer added, it's just a new wall/cup around the n-1 layer wall.

The load-bearing phrase: **"except for the floor coming up"** and **"Every other atom on everything that isn't the flat floor can grow and shrink"**.

This is fundamentally an **anisotropic offset**: shrink everywhere EXCEPT in the cap-normal direction. The floor doesn't move when the inset changes; only the dome + side walls move.

Cavity inset = T means cavity = (scan dome + side wall, shrunk inward by T) ∪ (cap-plane region below the shrunk walls). Layer outer at thickness T means outer = (scan dome + side wall, grown outward by T) ∩ (region below cap plane).

In both cases, the floor stays at the cap plane regardless of T. Standard isotropic offset can't do this — offsetting a closed body by T moves ALL its faces by T, including the floor.

The spec's scope-C primitive attempted to fix this by composing dome-wall-only-SDF + cap-plane half-space, but the dome-wall-only-SDF has the discontinuity at the cap plane analyzed above. The composition produces a cavity that's truncated (not closed) at the cap plane.

## Candidate alternatives for the recon session

The recon's job is to find an SDF/Solid composition that delivers the user's geometric model. Candidates to investigate, ordered roughly by "complexity to implement":

### Candidate A: Set-theoretic anisotropic offset via Solid composition

Express the user's model directly as a boolean composition over the closed body B and a "T-thick rind around the open mesh":

```
thick_open_rind = Solid::from_sdf(rind_sdf, bounds)
  where rind_sdf(p) = T − |open_sdf(p)|  (positive INSIDE rind, negative outside)

cavity = body.subtract(thick_open_rind)
       = {p : p ∈ body AND dist(p, dome+side walls) > T}
```

Same idea for layer outer:

```
layer_outer = (body.union(thick_open_rind)).intersect(below_cap_plane_solid)
            = {p : (p ∈ body OR dist(p, dome+side walls) ≤ T) AND p below cap plane}
```

Why this might work:
- The cap plane region is automatically pinned because the T-thick rind only thickens around dome+side walls (cap polygon isn't part of the open mesh).
- Anisotropic-offset behavior emerges from the set composition: shrink/grow happens around the dome+side walls only.
- cf-design has all the boolean ops needed (Solid::subtract, ::union, ::intersect, ::from_sdf).

Open questions:
- Does the cap polygon being at the Taubin-smoothed position (not exactly at cap plane) cause off-by-ε artifacts? Probably yes — the body's "floor" is at the Taubin'd polygon, the user wants it at the recorded cap plane.
- How does this compose with the cached-grid preview path? `extract_layer_surface` runs per-frame at extract time; we'd need an SDF-level analog (not Solid-level) for the preview to stay fast.

### Candidate B: Pre-process the cleaned mesh to project cap-fan vertices to the cap plane, then use Candidate A

Identify cap-fan vertices via the face-normal rule (same as `dome_wall_only_mesh`), project them onto the recorded cap plane. Now the body's cap polygon sits exactly at the cap plane. Then apply Candidate A.

Pros: cap_polygon = cap_plane, so the anisotropic-offset construction lands the floor exactly at the cap plane.

Cons: pre-processing the mesh changes the closed SDF too — would re-derive sd_closed from the projected mesh. Need to verify this doesn't break other SDF consumers (insertion sim's contact, cf-cast-cli's mesher, etc.).

### Candidate C: Pre-process the cleaned mesh + standard isotropic offset

If the cap polygon is projected to the cap plane (Candidate B's pre-processing) AND we accept the floor at cap_polygon − T (for cavity) or cap_polygon + T (for layer outer), then standard offset works. Floor moves T inward with offset.

This DOESN'T match the user's "floor stays at cap plane" requirement, but it's a useful fallback to discuss with the user — maybe they don't care about exact pinning as long as the floor is FLAT (no Taubin wobble).

### Candidate D: True anisotropic offset in mesh-offset (rejected by spec §1 Q6, but worth revisiting)

Add "skip cap-normal direction in offset" support to mesh-offset's `offset_mesh`. Spec rejected as "100+ LOC spanning two crates, separate arc, new public API." But it's the cleanest geometric expression of the user's model. May be worth revisiting if Candidates A/B/C all have issues.

### Candidate E: Post-MC mesh-level construction

Extract the per-iso surface with standard `Solid::from_sdf(closed).offset(±T)`, then in mesh space STITCH the boundary loops near the cap plane to a flat cap-plane polygon. Spec §1 Q2 rejected this as "fragile: boundary loops post-MC may have interpolation jitter; multi-cap requires per-cap loop detection + classification." But it's mechanically straightforward — boundary loop detection + ear-clipping triangulation is a well-trodden path (cf-scan-prep already does it).

## What to keep vs. throw away

The cf-cap-planes crate creation (sub-leaf 1) is mostly REUSABLE regardless of the new SDF construction. Specifically:

**Keep**:
- `CapPlane` runtime type — needed by any approach.
- `parse_cap_planes` — needed by any approach.
- `dome_wall_only_mesh` — useful for any approach that wants the open mesh.
- `report_cap_face_classification` — permanent regression sentinel.
- Constants (`CAP_FACE_*`) — same.

**Reconsider**:
- `DomeWallSignedSdf` adapter — the sign × |magnitude| construction is what failed. May not be needed; could be removed; or could be repurposed.
- `cf-design::pinned_floor_shell` primitive — the function as written produces the wrong geometry. May be deleted or radically rewritten.
- `extract_layer_surface`'s per-extract composition (sub-leaf 2) — produces an open cavity. Rewrite.
- `insertion_sim` + `cf-cast-cli` plumbing — same.

**Revert vs. forward-fix**: the recon should decide whether to:
- (a) `git revert fdfeee1d c1994571 2077c25f 27851305` — back out sub-leaves 2/4/5/6, keep sub-leaf 1 + sub-leaf 3 (cf-cap-planes crate + signed_volume_m3 revert; both unconditionally good). Then build the new construction on a clean base.
- (b) Forward-fix: modify the existing primitives to use the new construction. Keeps the test scaffolding.

Recommend (a): rollback wastes less effort fighting the wrong abstraction. The new construction may look quite different.

## Patterns confirmed durable

Even though the SDF construction failed, the broader arc DID confirm some patterns worth banking:

- **Visual gate on the smallest possible shell catches construction-level bugs**. Looking at the cavity alone (smallest, where pinned-floor would be most visible) made the falsification immediately obvious. If we'd only looked at the layer outer (largest, where the Taubin-rolled cap rim hides the issue), we'd have ratified a broken construction.
- **`f64::signum(0) = +1`** is a load-bearing footgun in any SDF construction that flips sign at the body boundary. Worth a project-wide memory.
- **Two-SDF sign × magnitude constructions have inherent discontinuities** at any point where the two SDFs disagree about "is this on the body surface" — e.g. when one mesh has the cap polygon and the other doesn't. Document as a counter-pattern.
- **`dome_wall_only_mesh` works correctly** (the diagnostic showed 31 cap-fan faces stripped, plateau confirmed). The face-normal classifier is the durable contribution. The misstep was assuming a stripped-cap SDF is a useful pinned-floor primitive.

## Recon questions for next session

The recon should produce a new spec doc (`docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_REDESIGN_SPEC.md` or similar). Answer:

1. **SDF construction**: which candidate (A / B / C / D / E) wins? What's the math? Sketch the construction in cf-design Solid ops or as a per-cell SDF composition.
2. **Cap polygon position handling**: project Taubin'd cap-fan vertices to cap plane in pre-processing? Or accept the polygon at its Taubin'd position?
3. **Preview path performance**: the cached-grid + per-iso MC pattern relies on extract_layer_surface running in ~1.5 ms per frame. Can the new construction stay in that performance envelope, or do we need a different per-extract acceleration?
4. **Sim/cast path**: does the new construction at Solid level still mesh cleanly via SdfMeshedTetMesh / mesh-offset? Spot-check on iter-1 cube fixture before committing to the spec.
5. **Rollback strategy**: revert sub-leaves 2/4/5/6 first (clean base) or forward-fix (keep test scaffolding)?
6. **What stays from cf-cap-planes**: keep CapPlane + parse_cap_planes + dome_wall_only_mesh + diagnostic. Remove or repurpose DomeWallSignedSdf? Add new adapters?
7. **The per_tet_layer + material_field bucketing source**: the sub-leaf 5 deviation kept scan_sdf (NOT dome_wall_signed). Re-examine in the new construction's context.
8. **Validation**: how do we structurally verify the new construction produces a flat floor BEFORE running the visual gate? E.g., a unit test that samples composed_sd at points on the cap plane interior and asserts ~0.

## Start-of-recon-session checklist

1. `git log --oneline -10` — HEAD at `fdfeee1d` (sub-leaf 6); tree clean.
2. `git status` — clean.
3. **OPEN AND READ IN FULL**: this bookmark.
4. **OPEN AND READ §1**: `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md` — user's verbatim geometric model.
5. **OPEN AND SKIM**: `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_SPEC.md` — what shipped (scope-C). Don't treat as active — it's the falsified spec.
6. **READ THE SHIPPED CODE** (cold-read):
   - `design/cf-cap-planes/src/lib.rs` — entire crate; understand DomeWallSignedSdf semantics + the diagnostic output.
   - `design/cf-design/src/solid_layered.rs` — pinned_floor_shell as shipped.
   - `tools/cf-device-design/src/sdf_layers.rs:822-883` — extract_layer_surface as shipped.
   - `tools/cf-device-design/src/insertion_sim.rs:660-870` — build_insertion_geometry as shipped.
   - `tools/cf-cast-cli/src/derive.rs:166-260` — derive_spec_and_ribbon as shipped.
7. **REPRO THE FALSIFICATION** before designing the fix:
   - `cargo run -p cf-device-design --release -- ~/scans/sock_over_capsule.cleaned.stl`
   - Uncheck "Show scan mesh", uncheck "Show layer" on Layer 0 and Layer 1.
   - Confirm cavity has no flat floor (matches image #5 evidence).
   - Look at cap_planes diag in startup log — confirms 31 cap faces stripped (`grep cap-planes`).
8. Answer the 8 recon questions above, produce a new spec doc.
9. Decide rollback strategy (recommend revert sub-leaves 2/4/5/6, keep sub-leaf 1 + sub-leaf 3) — call out before any code change.

## Three-session pattern (second iteration)

| Session | Output | Status |
| ------- | ------ | ------ |
| Bookmark (THIS session) | This doc + memory updates | DONE |
| Recon | New spec doc with chosen construction | NEXT |
| Implementation | New sub-leaf ladder, working code, visual gate pass | After recon |

Cold-read instruction for the recon session: load this bookmark, the parent bookmark §1, and the shipped code surfaces listed above. The user's geometric model is the load-bearing context (anisotropic offset is mandatory; not optional).
