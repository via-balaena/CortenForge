# CF-CAST Flat Cavity-Floor Recon — (B) / C-B2

> **Status:** RECON SCAFFOLD (pre-implementation). Cold-read at end (§7).
> **Date:** 2026-05-29
> **Trigger:** workshop slicer eyeball of the `base_mold` cast (after the apex
> pour + fitted seam landed). Two observations, one root cause:
> - the plug-anchor pin / floor-lock are **square to the cap normal, not the
>   ground or the seam** — i.e. the plug seats along a ~4.3°-tilted axis;
> - the cup cavity **floor slopes** and the diagonal seam cutting that slope
>   leaves an **interior edge** on each half's floor.
> **Composes on:** the organic-parts arc (§4.2 of `CF_CAST_ORGANIC_PARTS_RECON.md`,
> where (B) was first scoped) + the now-shipped apex pour + fitted seam. This is
> the last open organic-parts item.

---

## 0. S0 MEASUREMENT (2026-05-29) — corrects the tilt assumption

The `.prep.toml` cap normal is 4.3° off vertical, but that's the SCAN frame. The
**cast output is already leveled**: a plane fit to `plug_layer_0.stl`'s bottom
band reads **0.25° from the build axis, RMS 0.26 mm, peak 0.73 mm** — flat and
level. So the canal session's `auto_level_to_floor` effectively already did the
re-level (L1 below); the residual tilt is negligible.

**Reframed conclusions:**
- **Obs 2 (plug "not square to the ground") is essentially a non-issue:** the
  plug bottom + pin are 0.25° off vertical = square to the ground within a
  quarter-degree. What reads as "square to something else" is that the plug axis
  is vertical while the **fitted seam is diagonal in azimuth** — two correct but
  non-parallel frames, not a tilt defect.
- **Obs 3 (sloped floor + interior edge) is NOT a tilt problem** — it's the cup
  cavity floor's **MC lumpiness** (the untrimmed cup-side floor + the
  `pinned_floor_shell` cap-edge kink, peak ~0.7 mm class) plus the **diagonal
  seam cutting the floor** into an interior edge. So (B) is a *flatness +
  coplanarity cleanup*, not a re-level.

This collapses the arc to **L2 only** (the bounded-slab floor flatten); L1 is
done. The slab's job is to turn the lumpy/edged cup floor into one clean flat
plane coplanar with the already-flat plug bottom. (TODO S0b: isolate + measure
the CUP-side floor lumpiness directly — harder than the plug bottom — to confirm
it's worth a slab vs accepting the ~0.7 mm class.)

## 1. Root cause (confirmed in code + on the scan)

Both the plug bottom and the cup cavity floor come from
`cf_design::pinned_floor_shell`, which builds the offset shell and then, for each
recorded cap, does `shell = shell.intersect(Solid::plane(cap_normal, offset))`
(`solid_layered.rs:122`). So the floor of BOTH the plug body and the cup cavity
is the **scan's cap plane**.

On `base_mold` that cap normal (from `.prep.toml [caps]`) is
`[0.058, −0.047, −0.997]` — **~4.3° off vertical** (cf-scan-prep's auto-PCA
lean). Therefore:
- **Plug bottom + plug-lock + pin** are flattened/oriented to the **tilted** cap
  plane (`build_plug_cap_trim_transform` trims the plug bottom at the cap plane,
  along `cap_normal`). → "square to the cap, not the ground." (obs 2)
- **Cup cavity floor** = the body's cap-plane bottom = the same **tilted** plane,
  PLUS it is NOT trimmed flat on the cup side (cup trim disabled since
  2026-05-24) → it also carries the MC lumpiness at the cap-plane × body-lateral
  edge (the `pinned_floor_shell` kink, recon §Q4-1-a). → "sloping + lumpy." (obs 3)
- The **fitted diagonal seam** cuts that tilted floor → the floor meets the seam
  at an angle → **interior edge** on each half. (obs 3)

This is the §3 cross-cutting finding of the organic-parts recon made concrete:
the seam tilt and the floor tilt share the cap-tilt root.

---

## 2. Two levers (they address different parts)

**(L1) Re-level the scan upstream (cf-scan-prep / OQ3).** If the scan is
reoriented so the cap normal = the build/demold axis (vertical), then the cap
plane is level → the plug bottom + plug features are square to the ground (fixes
obs 2 fully) AND the cup floor is level rather than sloped (fixes most of obs 3;
the MC lumpiness + the want-a-guaranteed-flat-coplanar-face remain). Cheapest per
the organic-parts §3 finding. cf-scan-prep-side (the canal session added
`auto_level_to_floor`, but `base_mold.prep.toml` still records a 4.3° cap —
either it predates the auto-level or the level wasn't re-applied/saved; confirm).

**(L2) Flatten the cup cavity floor in cf-cast (C-B2 bounded slab).** Independent
of tilt, guarantee the cup cavity floor is a single flat plane coplanar with the
trimmed plug bottom, so they clamp flush (no silicone ingress, printable). This
is the (B) item proper. Needed for the MC-lumpiness + the coplanar-flat guarantee
even after re-leveling.

The two compose: **re-level kills the tilt; the slab kills the lumpiness + makes
the floor a clean flat coplanar face.** §6 OQ-B1 asks the workshop which to do
for iter-1 (likely both, re-level first).

---

## 3. C-B2 design — bounded-slab floor flatten

The dead **C-B1** was a `SeamTrim` (halfspace) at the cap plane: it BISECTED the
cup because the cup straddles the cap plane (walls above, base below) — a
halfspace deletes one side (172 mm half → 23 mm stub, S2 2026-05-28).

**C-B2 is ADDITIVE + bounded, so it can't bisect.** Choose a flat floor plane
`P_floor` (⟂ the build axis, at/just below the cap-plane centroid). Then:

> **Fill everything below `P_floor` with cup material**: union
> `bounding_region ∩ {below P_floor}` into the cup piece. The cavity bottom then
> rests ON `P_floor` (a flat plane), with solid cup beneath it — no cavity below
> the floor, nothing deleted above. The plug bottom (trimmed to the SAME
> `P_floor`) seats flush on it.

Implementation options for the "fill below `P_floor`" op:
- **C-B2a (SDF-side union):** in `compose_piece_solid`, `piece.union(slab_solid)`
  where `slab_solid = bounding_region.intersect(halfspace_below(P_floor))`. Union
  is paradigm-safe (additive; the slab top = a new flat surface, no
  coincident-face subtraction). Simplest; no new mesh-CSG primitive.
- **C-B2b (post-MC mesh-CSG `UnionCuboid`):** add a `MatingTransform::UnionCuboid`
  (sibling of `SubtractCylinder`/`UnionTruncatedPyramid`) and union a cuboid slab
  post-MC (bit-precise). More machinery; matches the pour/bolt post-MC pattern.

Lean **C-B2a** (SDF union) unless MC at the slab-top plane misbehaves — the
flange/cup-wall are already SDF-composed, so one more union is natural.

**Plug side:** `build_plug_cap_trim_transform` already flattens the plug bottom;
retarget it (and the cup floor) to the SAME `P_floor` so the two faces are
coplanar. If `P_floor` is ⟂ the build axis (not the tilted cap), the plug trim
normal must change from `cap_normal` to the build axis too.

**Plug-lock socket / pin** interact: the socket is carved into the cup floor and
the lock protrudes from the plug bottom along `cap_normal`. If `P_floor` ⟂ build
axis, re-anchor the lock/socket to the build axis (else they tilt vs the flat
floor). Mirror the apex-pour "which frame" care.

---

## 4. Spike (S0, cheap, no production code)
Measure on the regenerated `base_mold` cup STLs (Python, like the seam-fit
spike): the cup cavity-floor planarity — fit a plane to the floor verts, report
tilt-from-vertical-build-axis + RMS + peak lump. Confirm ~4.3° tilt + size the
lumpiness. Also measure the plug-bottom plane (should be ~flat at the tilted cap)
to size the coplanarity gap. Decides whether re-level alone suffices or the slab
is needed.

## 5. Phasing
| Phase | Scope | Deliverable |
|---|---|---|
| **S0** | Spike: measure cup-floor tilt + lumpiness + plug-bottom plane on `base_mold`. | numbers sizing L1 vs L2 |
| **S1** | (workshop) re-level decision (OQ3) + re-prep if chosen. | level cap or not |
| **S2** | C-B2a slab: `P_floor` + cup-floor union + retarget plug trim to `P_floor`; re-anchor plug-lock/pin to the floor frame. Flatness gate. | flat coplanar floor |
| **S3** | `base_mold` regen + workshop eyeball; tune `P_floor` offset + bias. | empirical convergence |

## 6. Open questions for workshop
- **OQ-B1 (the big one):** re-level the scan upstream (L1), add the cf-cast slab
  (L2), or both for iter-1? (Recommend: re-level first if cheap, then the slab for
  the flat coplanar guarantee.)
- **OQ-B2:** `P_floor` ⟂ the build axis (level, robust to cap tilt) or = the cap
  plane (simplest, but tilted if not re-leveled)? If re-leveled, they coincide.
- **OQ-B3:** keep the plug-lock/pin along `cap_normal`, or re-anchor to the build
  axis when `P_floor` ⟂ build axis?

## 7. Cold-read pass-1
- **C1 (one root, two symptoms).** Obs 2 (plug tilt) + obs 3 (floor slope) are
  both the cap tilt; don't fix them separately. Re-level addresses the tilt for
  both; the slab addresses the floor's flatness/coplanarity. State which lever
  owns which symptom so we don't double-implement.
- **C2 (additive, not subtractive — the C-B1 lesson).** The dead C-B1 halfspace
  bisected the cup. C-B2 MUST be additive/bounded (fill below `P_floor`). Lead
  with that so we don't repeat the bisection.
- **C3 (frame consistency).** If `P_floor` ⟂ build axis but the plug-lock/pin
  stay on `cap_normal`, they'll tilt against the flat floor — the same
  "square-to-something-else" the workshop just flagged, moved to a different
  feature. Re-anchor everything to one frame.
- **C4 (re-level may be the cheap 80%).** Per organic-parts §3, re-leveling alone
  removes the tilt from BOTH the seam and the floor. The slab may then only need
  to clean MC lumpiness. Spike (S0) the lumpiness magnitude before building the
  slab — it might be small enough to defer.
