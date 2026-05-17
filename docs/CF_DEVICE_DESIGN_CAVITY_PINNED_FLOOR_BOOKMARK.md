# cf-device-design Cavity Pinned-Floor — Bookmark

**Status**: BOOKMARK 2026-05-16 EVENING. Session paused; recon to
happen in a new (fresh-context) session, not a continuation of this
one. Three-session pattern.

**Parent**: [`CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`](CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md)
(SHIPPED but solving wrong problem — see below).

**Blocks**: penetration sim, mold generation. **Both locked until
this is right.**

## The actual geometric model the user needs

(Verbatim from session 2026-05-16-evening, lightly punctuated.)

> The main shell geometry [scan] is loading in as a reference.
> Basically isn't real. The cavity spawns a shrunk-down version of
> the cavity [scan] in every way except for the floor coming up.
> Every other atom on everything that isn't the flat floor can grow
> and shrink. Attached to this cavity is the wall. The wall is
> walls off of the cavity that dynamically grow and shrink as the
> cavity grows and shrinks. The wall also grows and shrinks for
> every atom that is not the flat floor, that's fixed to the plane
> (cavity's flat floor is fixed to the plane too). Then, for every
> layer added, it's just a new wall/cup around the n-1 layer wall.
> It dynamically grows and shrinks with the cavity/other wall. When
> you want to grow its walls, it grows and shrinks in every way,
> except for the bottom floor atoms, which are fixed to the floor,
> and don't grow downward, past the plane.

Distilled:
- **Scan = reference**, not cast.
- **Cavity** = scan shrunk in every direction EXCEPT the floor
  stays pinned to the cap plane.
- **Layer 0 wall** = wraps around cavity, also shrinks/grows
  everywhere EXCEPT floor stays pinned.
- **Layer N wall** = wraps around Layer N-1, same rule: floor pinned,
  everything else flexes.
- **All floors coplanar** with the scan's cap plane.
- **All shells are closed manifolds** — the cavity is a "hole" that
  gets subtracted (CSG) from the layer 0 material for the
  penetration simulation.

## What shipped that DOES NOT meet this need

Cavity-mouth arc, 6 commits on dev (`03bb695f` → `cde8938b`):

| # | Commit       | Sub-leaf                                                 |
| - | ------------ | -------------------------------------------------------- |
| 1 | `03bb695f`   | `[caps]` parser + `CapPlanes` Bevy resource + bake.      |
| 2 | `cb03b144`   | `dome_wall_only_mesh` helper.                            |
| 3 | `544a8905`   | Two-SDF `build_cached_scan_sdf`; `Arc::clone` fast path. |
| 4 | `5ad09176`   | Post-MC Sutherland-Hodgman half-space clip; threading.   |
| 5 | `a403a4f3`   | Cap-centroid-origin `signed_volume_m3`.                  |
| - | `cde8938b`   | Face-normal cap-face detection (Taubin-drift fix).       |

The shipped code was specced to deliver an **OPEN cavity** (no
floor; a tube you'd see UP into from below the cap plane). The
spec called this "an opening at the base for insertables".

The visual gate on iter-1 (`~/scans/sock_over_capsule.cleaned.stl`,
session 2026-05-16-evening) revealed two-SDF + post-MC clip
*technically* opens the cavity at the cap plane (the diagnostic
confirmed boundary-ring termination at signed-dist = 0). But the
user's actual need is different from what the spec optimized for.

## Why it's the wrong fit

The spec optimized for "open cavity for insertables". The user's
actual need (clarified mid-session): "closed cavity, used as the
hole shape for CSG subtraction in the penetration sim, with all
shell floors coplanar at the cap plane."

These are NOT the same geometric primitive:

| Aspect | Spec / shipped | Actual need |
| --- | --- | --- |
| Cavity topology | Open (no floor) | Closed manifold |
| Cavity floor location | Doesn't exist (mesh terminates at cap plane boundary ring) | Pinned to cap plane |
| Layer outer floor | Trimmed AT cap plane (boundary ring) | Closed with flat floor AT cap plane |
| Use case downstream | Visual fit-viz (can see inside cavity) | CSG subtraction for FEM sim + mold CAD |

Layering "close the boundary ring" on top of two-SDF was discussed
mid-session and rejected as the wrong architectural starting point
— it bolts geometry onto a primitive designed for a different
purpose. Better to redesign from the actual need.

## What's locked downstream

- **Slice-7 insertion sim** — was already bookmarked at
  [`CF_DEVICE_DESIGN_INSERTION_SIM_OPEN_CAVITY_BOOKMARK.md`](CF_DEVICE_DESIGN_INSERTION_SIM_OPEN_CAVITY_BOOKMARK.md)
  pending re-tune for open cavity. Now SUPERSEDED — sim needs the
  closed-pinned-floor cavity (CSG-subtractable), not open cavity.
- **cf-cast-cli mold generation** — uses its own SDF approach
  today; same closed-pinned-floor primitive would unify the
  geometry across consumers.
- **Fit-viz rungs 2-6** — were technically unblocked by the
  shipped open cavity, but if the cavity is being re-architected,
  may need to revisit whether fit-viz consumes the same primitive
  or its own variant.

## Recon questions for next session

Carry to a fresh-context session (NOT a continuation of this one
— this session has too many in-flight threads, cold-read needs to
start clean from the user's geometric model + the bookmark below).

1. **API shape**: what's the right primitive's signature? Sketch:
   `fn build_shell_pinned_at_cap(scan_sdf, cap_planes, offset_m) -> ClosedMesh`?
   Or split into `wall + floor` separately for caller composition?
2. **Implementation strategy**: anisotropic SDF offset (skip the cap-
   normal direction)? Or post-MC delete-floor + stitch-down + flat-cap
   triangulation? Or use the cleaned-scan's actual cap polygon as
   the floor triangulation directly?
3. **Where it lives**: cf-device-design's `sdf_layers` module
   (current), or extract to `cf-design` as a shared primitive used
   by cf-device-design preview + insertion_sim + cf-cast-cli?
4. **Relationship between shell floors and scan cap polygon**: do
   all shells share the SAME triangulated cap polygon (the one
   cf-scan-prep emits), or does each shell get its own
   triangulation matched to its terminating ring?
5. **Two-SDF code disposition**: keep as scaffolding for the new
   approach (the open-mesh SDF could still be useful for wall
   construction), or rip out entirely and start fresh?
6. **Anisotropic offset feasibility**: can `mesh-sdf` /
   `mesh-offset` support "offset only in directions perpendicular
   to a given plane"? If not, what's the minimum addition needed?
7. **Multi-cap handling**: the iter-1 fixture has 1 cap; body-part
   scans may have 2 (proximal + distal). Pinned-floor at BOTH
   planes simultaneously is a different problem (each shell would
   have two floors).
8. **What does the insertion sim CSG actually want?** This is the
   load-bearing downstream constraint — the cavity-as-hole
   subtraction operation should drive the cavity mesh's required
   properties (closed, manifold, consistent winding, possibly
   watertight per `mesh-repair`'s definition).

## Start-of-next-session checklist (cold read)

1. `git log --oneline -10` — dev HEAD should be `cde8938b` (face-
   normal fix). Tree should be clean.
2. Read this bookmark IN FULL — the user's geometric model in §1 is
   the load-bearing context.
3. Read [`CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`](CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md)
   §"Implementation summary" — what shipped, so you don't re-derive.
4. Do NOT read the cavity-mouth bookmark/recon docs as if they're
   the active spec — they describe the *previous* approach
   (open cavity) that's now superseded.
5. Recon should produce a NEW spec doc
   (`CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_SPEC.md`) before any
   implementation. Three-session pattern.

## Patterns to carry forward (banked from the shipped arc)

Even though the shipped arc doesn't deliver what's needed, several
patterns are still durable + reusable:

- **`report_cap_face_classification` diagnostic** (sweep table +
  vertex-distance distribution) is a permanent regression sentinel
  for any face-vs-plane classification. Keep.
- **Face-normal + centroid-distance rule** for cap-face detection
  (replacing 1 µm vertex-distance) is robust against post-
  smoothing drift. Keep — applies regardless of whether shells
  end up open or closed.
- **Sutherland-Hodgman triangle-vs-plane clip** in
  `clip_mesh_against_cap_plane` is a generic mesh utility. Keep —
  reusable wherever a triangle mesh needs half-space trim.
- **CapPlane / CapPlanes resource + `parse_cap_planes`** parser is
  data-layer-only and survives any approach change. Keep.
- **Taubin smoothing surprise**: cf-scan-prep applies 8 iterations
  to the cleaned mesh post `auto_cap_open_boundaries` projection.
  This is a recurring "the cleaned STL is NOT what the prep.toml
  metadata implies" gotcha worth banking as a project memory.
