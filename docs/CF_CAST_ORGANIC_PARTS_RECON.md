# CF-CAST Organic-Parts Recon — Flat Seam Plane + Flat Floor Mating

> **Status:** RECON SCAFFOLD (pre-implementation). Cold-read pass-1 at end (§9).
> **Date:** 2026-05-28
> **Trigger:** Workshop cf-view smoke on the `3quartachub` canal cast (the first
> genuinely curved/organic part). Two flatness assumptions that held on the
> near-straight `sock_over_capsule` break on a curved part:
> - **(A)** the cup-piece seam/flange bows off-planar (it follows the curved
>   centerline), so the printed mating face isn't flat and won't seal.
> - **(B)** the plug bottom and the mold-cavity floor are domed/lumpy (they
>   follow the organic cap), so they don't seat flush — silicone seeps between
>   the plug base and the floor, and FDM can't print that lumpy mating surface.
> **Direction (workshop, 2026-05-28):** recon-scaffold-first; the best-fit
> split-plane *method* (A) is deliberately an open question to settle here.
> **Composes on:** `main` = `4734ce08` + the (uncommitted) canal arc. Independent
> of the canal — these are general organic-part fixes.

---

## 1. Problem statement

cf-cast was built and tuned on `sock_over_capsule` — a **near-straight** scan
(max tangent rotation ~3.4°). Two subsystems silently assume near-straightness:

**(A) The seam is curve-following, the flange is built on a single tangent-plane
approximation.** The mold cup is split into two halves by a `Ribbon` whose cut
surface is **piecewise-linear per segment** (each segment's cut normal is its
local `binormal = tangent × split_normal`). The flange and the half-space cut
both key off `Ribbon::seam_plane_reference()` — a **single plane** taken at the
**arc midpoint**. On a near-straight part that single plane matches the true seam
to ≤ 1 µm. On a curved part the binormal swings along the length, so:
- the single arc-midpoint plane is **tilted/mis-placed** relative to the part,
- the flange's perpendicular-to-seam thickness band tapers toward the ends
  (wedge), and
- the workshop sees a mating face that bows / a flange that isn't a clean flat
  plate (images 6/7).

The workshop's desired behavior: **pick one best-fit flat plane and cut both
halves on it.** It won't perfectly bisect an organic bulge, but the halves print
**mating-face-down**, so a planar seam is *guaranteed* flat and seals.

**(B) The floor is pinned to the cap plane but never trimmed flat; the cup-side
trim is disabled.** Both the plug bottom and the cup cavity floor come from
`pinned_floor_shell`, which intersects the shell with the cap **plane**. The
plug-side cap trim (`build_plug_cap_trim_transform`) flattens the plug bottom at
that plane; the **cup-side trim is DISABLED** (since 2026-05-24, paradigm-boundary
issue). On a curved/tilted organic cap the result is a domed/lumpy floor on the
cup side and an imperfect flat on the plug side. The workshop needs **both the
plug base and the cup cavity floor to be the same flat plane** so they clamp
flush (no silicone ingress, printable).

---

## 2. Current architecture (as found — file:line)

### 2.1 Seam split + flange (A)
| Concept | What it is | Source |
|---|---|---|
| `SplitNormal` | unit world dir the mold opens along (default +X) | `ribbon.rs:71` |
| `RibbonSegment` | per-segment frame: `tangent`, `binormal = tangent×split_normal` | `ribbon.rs:202` |
| Ribbon SDF | per-query closest-segment, `dot(p−proj, segment.binormal)` — **piecewise-linear** cut surface | `ribbon.rs:796` |
| `seam_plane_reference()` | **single** `(arc_midpoint, binormal_at_midpoint)` — used by flange + gates | `ribbon.rs:698` |
| `halfspace_solid(side, bounds, 0)` | half-space at the single seam-plane reference | `ribbon.rs:835` |
| `compose_piece_solid` | `CupWallShellSdf ∩ halfspace ∪ flange` | `piece.rs:289` (halfspace at `:323`) |
| `FlangeSdf` | flat slab: `max(body_dist−width, inner_offset−body_dist, \|dist_from_seam\|−thickness)`; `dist_from_seam` uses **single binormal** | `flange.rs:239` (eval `:254`) |
| piece-count gate | `max_tangent_rotation_rad() < 60° → 2 pieces` | `ribbon.rs:595` |

**Load-bearing assumption:** `seam_plane_reference()` is a single plane and the
flange uses a single binormal. Both are arc-midpoint approximations whose error
is `O(arc_length × max_rotation/2)` — negligible near-straight, visible curved.

### 2.2 Floor + cap trim (B)
| Concept | What it is | Source |
|---|---|---|
| `pinned_floor_shell` | shell, then `for cap: shell = shell.intersect(plane(normal, centroid·normal))` | `cf-design/src/solid_layered.rs:93` (cap loop `:122`) |
| `CapPlane` | **single** `(centroid, normal)` best-fit to the cap loop; cap-loop is non-planar on organic parts | `cf-cap-planes/src/lib.rs:130`, parse `:246` |
| plug cap trim | `build_plug_cap_trim_transform` → post-MC `SeamTrim` at cap plane, `−cap_normal`, +1 µm bias | `plug.rs:440`, bias `PLUG_CAP_TRIM_BIAS_M` `:184` |
| **cup cap trim** | `build_cup_cap_trim_transform` exists but **DISABLED** | `plug.rs:485`; disable note `piece.rs:432` |

**Load-bearing assumption:** the cap is a single plane and the cup floor needs no
explicit flat trim. The disable note says the cup floor wants the §Q-4 S2
`MatingTransform::UnionCuboid` slab (handles MC under- AND overshoot), which was
**never implemented**.

---

## 3. Diagnosis to confirm in S0 (don't assume)

The Explore map surfaced a tension worth nailing before coding: the cup mating
face is *already* an SDF half-space intersect at a **single** plane
(`seam_plane_reference`), which should make it flat — yet the workshop sees
bowing. **S0 (diagnostic, no production code) must pin which of these is the real
A-failure** on `3quartachub`:
- **H1 — tilted plane:** the seam plane is flat but tilted (arc-midpoint binormal
  ≠ vertical), so the piece leans / the flange isn't horizontal when set down.
- **H2 — poor bisection:** the single mid-plane doesn't cleanly bisect a curved
  body; the body crosses the plane such that the half-space leaves a ragged or
  multiply-connected mating region.
- **H3 — flange inner edge:** the flange *band* is flat but its inner edge tracks
  the curved body silhouette, reading as "not a flat plate" even though the
  mating face is planar.
Method: dump the cup-piece mating-face vertices, fit a plane, measure RMS
deviation + tilt vs vertical; visualize. Cheap; reuses the `mating_face_is_*flat`
test harness (`piece.rs:665`).

For **B** the diagnosis is already clear (cup trim disabled + floor follows a
tilted/curved cap), but S0 should also measure the plug-bottom + cup-floor
planarity RMS on `3quartachub` to size the trim.

### S0 RESULTS (2026-05-28 — Python plane-fits on the `3quartachub` cast STLs)
**(A) cup mating face: NOT flat — H1 + H2 both real.** piece_0's seam face
deviates **RMS 3.8 mm / peak 8.4 mm** from its best-fit plane AND that plane is
tilted **~3.5° from vertical**. So the seam is both *tilted* (H1) and *bowed*
several mm (H2) — not merely a flange-inner-edge illusion (H3). The single
arc-midpoint seam plane genuinely fails on this curved part. → S1 (explicit
best-fit plane) is justified.

**(B) floor mating:**
- **Plug bottom: mostly flat (RMS 0.61 mm) — the §Q-4 plug trim works — but with
  up to 4.3 mm peak lumps** over a 6.3 mm Z-span (MC noise at the cap edge + the
  plug-lock pyramid). The multi-mm protrusions are what break flush clamping.
- **Cup cavity floor: un-trimmed (cup-side trim DISABLED) → follows the organic
  cap → domed** (visually confirmed, images 8/10; numeric isolation from the cup
  STL was unreliable but it is un-trimmed by construction). → S2 (re-enable the
  cup trim + clean the plug base) is justified.

**Cross-cutting finding (high-leverage):** the ~3.5° seam tilt ≈ the **4.4° cap
tilt** from cf-scan-prep's auto-PCA lean (same root). **Re-leveling the scan
upstream cuts BOTH the seam tilt (half of A) AND the floor tilt (part of B)**,
leaving only the seam *bow* + the cup-floor trim for cf-cast. OQ3 is therefore
the cheapest first move and should precede S1/S2.

---

## 4. Proposed direction (to firm up after S0)

### 4.1 (A) Explicit best-fit cut plane
Replace the arc-midpoint `seam_plane_reference()` single-plane approximation with
an **explicitly fitted cut plane**, used consistently by the half-space cut
(`piece.rs:323`) AND the flange (`flange.rs:294`). The plane is computed once from
the body/centerline and is genuinely flat by construction, so both mating faces
are coplanar regardless of curvature.

The **fit method is OPEN (§8 OQ1)** — candidates:
- **C-A1 PCA, split_normal-respecting:** plane contains the centerline's dominant
  axis and is ⟂ to `split_normal`. Honors existing intent; one flat cut.
- **C-A2 balance-optimal:** plane that most evenly bisects the body (≈ equal
  halves), orientation free. Flattest + balanced but may surprise.
- **C-A3 keep the ribbon for demolding, fit a plane only for the flange face:**
  hybrid — cup wall stays curve-following (good demold), only the flange/clamp
  face is a fitted plane. (May reintroduce the bow at the wall↔flange join.)

Cross-cutting: the `max_tangent_rotation < 60°` 2-piece gate (`ribbon.rs:595`)
must be re-examined — a flat cut on a strongly curved part may make one half
non-demoldable (undercut against the flat plane). The recon must state the
curvature ceiling for a planar split + what happens above it.

### 4.2 (B) Flat floor mating on both plug + cup

> **S2 EMPIRICAL RESULT (2026-05-28): C-B1 is DEAD — a halfspace `SeamTrim`
> BISECTS the cup.** Re-enabling `build_cup_cap_trim_transform` (even with the
> plug-side 1 µm bias) collapsed a 172 mm full cup half to a 23 mm floor stub: the
> cup STRADDLES the cap plane (cavity walls above, floor base below), so a
> halfspace cut deletes one or the other. The plug-side trim works only because
> the plug lives entirely on one side of the cap plane. **Flattening an INTERNAL
> cavity floor while keeping material on both sides REQUIRES a bounded op (C-B2).**
> Reverted; `build_cup_cap_trim_transform` kept dormant (with bias) for C-B2 reuse.
> → **B's real path is C-B2; budget it as genuine geometry design, not a re-enable.**

Trim **both** the plug bottom and the cup cavity floor to **one shared flat
plane** so they seat flush. Lowest-risk first cut:
- **C-B1 (MVP): re-enable the cup-side trim + keep the plug-side trim**, both as
  post-MC `SeamTrim` at the cap plane with the proven 1 µm bias (the plug-side
  pattern from §Q-4 S1). Gives flat coplanar faces when the cap plane is a good
  fit. Risk: on a strongly domed cap the flat trim either bites into the floor or
  leaves a step where the dome meets the plane.
- **C-B2: `UnionCuboid`/slab flatten** (the deferred §Q-4 S2): union a flat slab
  at the cap-plane offset onto the floor so MC under- AND overshoot both resolve
  to one flat face. More robust to MC quantization than a subtractive trim.
- **C-B3 (full): curved-cap support** — store the cap-loop cloud in `CapPlane`,
  give `pinned_floor_shell` a curved-cap SDF. Large cf-cap-planes + cf-design
  refactor; almost certainly out of scope for iter-1 (the workshop only needs a
  *flat* mating face, not a cap that follows the dome).

Note the 4.4° cap tilt already observed on `3quartachub` (cf-scan-prep auto-PCA
lean): a flat floor trim should be **perpendicular to the demold/build axis**,
not to the tilted cap normal, OR the scan should be re-leveled in cf-scan-prep
first. §8 OQ3.

---

## 4.3 (C) Pour-gate + vent placement (NEW — workshop cf-view 2026-05-28)

> **APEX-POUR PIVOT — IMPLEMENTED 2026-05-29 (supersedes the floor-pour decision
> below).** Reasoning together with the workshop revised the locked
> opposite-ends/floor-pour plan: a floor pour only fills the cavity to the top if
> fed by a riser tall enough to reach the dome apex (connected-vessels: the cavity
> fills only to the pour source's liquid level), and a downward-pointing floor port
> can't take a gravity funnel. **The cavity's highest point IS the dome apex**, so
> pouring there makes complete fill self-evident (full the instant silicone reaches
> the bore — nothing sits above it) and lets trapped air migrate to that same
> point. Final design (workshop-locked):
>
> - **POUR — single AXIAL bore at the dome apex, ON the seam.** No splay. Lying in
>   the seam plane → it splits the flange: separating the two cup halves bisects the
>   channel into open half-troughs, so the cured sprue lifts out (no pull through a
>   blind hole) and the two half-bores re-register into one round hole on assembly.
>   Avoids the crowded floor entirely (plug-lock/dowels/bolts live at the floor).
> - **VENT — NOT modeled; hand-drilled.** The workshop has 0.1–3.0 mm carbide bits.
>   Air's ~1000× lower viscosity escapes a sub-mm hole the honey-thick silicone
>   won't weep through, so tiny drilled vents at the apex + any high spots (tuned to
>   what a test pour strands) beat a modeled vent — placed empirically, negligible
>   residue. (Modeled holes are cut post-MC as bit-precise mesh-CSG cylinders, so
>   the 3 mm MC grid never limited vent size — but hand-drilling is simpler + field-
>   tunable, and the workshop has the tools.)
> - **FUNNEL — un-bend it.** The 30° nipple bend existed only to match the V splay;
>   the apex bore points straight up (+Z up), so the nipple is STRAIGHT (zero tilt),
>   bowl up. Least-risky version of the "funnel re-placement" — it stays at the dome,
>   just straight. (`build_funnel_solid` already emits a standalone STL with no world
>   placement; only the bend angle was coupled to the pour.)
> - **BOLTS — bracket the pour bore.** The big bore through the flange removes clamp
>   material at the apex; the §B pattern's pour-gate *collision-skip* (which dropped
>   the nearby bolt — the known "lost 1 bolt of clamping" weakness) is replaced, for
>   this layout, by a *bracket*: a bolt stepped to just outside the clearance on EACH
>   side of the bore. Closes that weakness.
>
> **Implementation (opt-in, existing V-at-dome casts byte-identical):**
> `PourGateLayout::{VAtDome (default), ApexAxial}` on `PourGateSpec`; `build_pour_gate_transforms`
> branches (apex bore projected onto the seam plane + axis seam-normal component
> removed so it lies in-plane); `build_funnel_solid` zeroes the tilt for `ApexAxial`;
> `BoltPatternSpec::bracket_pour_gate` (default false) → `append_pour_bracket_bolts`;
> cf-cast-cli `[pour_gate].apex_axial = true` (default false) sets the layout + the
> bolt bracket in `derive.rs`. Procedure prose branches throughout.
>
> ---
>
> **SUPERSEDED — original floor-pour decouple (workshop, 2026-05-29, kept for the
> trail):** pour orientation = dome/glans UP; placement = OPPOSITE ENDS → VENT at the
> dome tip, POUR at the floor/mouth end (bottom-fill). Dropped because the floor pour
> needs a riser to the apex for complete fill and a downward floor port can't take a
> gravity funnel — apex-pour is strictly simpler and self-verifying for fill. The
> ripples it flagged (which-piece, floor-feature collision, funnel re-placement,
> ergonomics) all dissolved or simplified under apex-pour.

On the `3quartachub` cup the pour hole + aeration (vent) hole land **in the
flange** near the dome end; the workshop wants them **off the flange, at/near the
mold tip** (the dome apex), so the pour funnel + vent sit on the closed end, not
the clamp face. The pour gate is built by `crate::pour::build_pour_gate_transforms`
(ribbon `PourGateKind::Default`, "V-shape at dome end") anchored off the ribbon's
pour end. On a curved/organic part — and now with the §M flange continuous around
the perimeter — the V + vent legs collide with the flange instead of clearing it
at the tip. **Diagnosis + fix deferred to its own phase (S4):** confirm where the
pour/vent anchor lands vs the flange perimeter, then either move the anchor to the
true dome apex (outboard of the flange inner edge) or carve a flange relief so the
pour/vent clears. Likely interacts with the planar-seam flange (A). Not yet
scoped; see OQ5.

## 5. Phasing

| Phase | Scope | Deliverable |
|---|---|---|
| **S0** | Diagnostic probe (`#[ignore]`): dump `3quartachub` cup mating-face + plug-bottom + cup-floor vertices, fit planes, report RMS + tilt. Confirm H1/H2/H3 for (A) + size the (B) trims. NO production code. | which failure is real + numbers |
| **S1** | (A) `Ribbon::best_fit_seam_plane()` + wire into `piece.rs` halfspace + `flange.rs`; pick fit method per S0 + OQ1. Flatness gate on a curved fixture. | flat seam/flange on curved parts |
| **S2** | (B) flat floor mating: C-B1 (re-enable cup trim + bias) or C-B2 (slab) per S0. Both plug-bottom + cup-floor coplanar-flat. Wall/seat gate. | flush flat floor mating |
| **S3** | Curved-fixture regression + `3quartachub` regen + workshop cf-view/print smoke; tune curvature ceiling + trim bias. | empirical convergence |

Each code phase: cold-read + full gates (`cargo xtask grade-all`), per prior arcs.

---

## 6. Risks / unknowns
1. **Planar split vs demoldability:** a flat cut on a curved part can create an
   undercut on one half → won't release. The whole point of the ribbon was
   curve-following demold. Need the curvature ceiling (§8 OQ2).
   > **EMPIRICAL (2026-05-29, `3quartachub` apex-pour regen):** this is now
   > confirmed real AND shown to be a no-win for a single flat plane on this part.
   > The glans leans ~−Y off the shaft. Per-Z-band cup-half measurement: the shaft
   > bisects evenly (±23 mm both halves) but near the dome one half collapses to a
   > 4–12 mm sliver while the other wraps the dome (lopsided seam — surfaced by the
   > apex pour sitting right there). Flipping `split_normal` X→Y to put the seam
   > plane IN the curve plane (even dome bisection in theory) instead made the body
   > run **tangent** to the seam plane along the bend → **non-manifold cup MC**
   > (manifold3d `NotManifold`, fails before any pour/bolt CSG). So: `split_normal=X`
   > = manifold + lopsided dome; `split_normal=Y` = even dome + non-buildable. A
   > single flat seam cannot win both; the dome needs a curve-following / hybrid seam
   > (C-A3) or a 3-piece split. NOT an apex-pour bug (seam geometry is identical to
   > the V-pour cast); the apex pour only made it visible. Tracked as (A) follow-up.
2. **Piece imbalance:** a best-fit plane through an asymmetric organic bulge gives
   unequal halves; one may be a thin shard. Gate on min-piece extent.
3. **Cap tilt / curvature for (B):** a flat trim on a 4.4°-tilted (and domed) cap
   either steps or bites. Re-leveling upstream may be the cleaner half of the fix.
4. **MC quantization at the flat trim edge** is the same family that disabled the
   cup trim in the first place — bias / slab approach must be paradigm-safe
   ([[project-cf-cast-sdf-meshcsg-paradigm-boundary]]).

## 7. Out of scope
- Curved-cap SDF (C-B3) unless S0 proves a flat plane is unacceptable.
- 3+-piece molds for high-curvature parts (v3).
- The canal arc (independent; already shipped uncommitted).

## 8. Open questions for workshop
- **OQ1 (deferred from this session):** best-fit cut-plane method — C-A1 PCA /
  C-A2 balance-optimal / C-A3 flange-only hybrid. Resolve after S0's diagnosis.
- **OQ2:** curvature ceiling for a single planar split before a half becomes
  non-demoldable — refuse, warn, or fall back to the ribbon above it?
- **OQ3:** for the flat floor, trim ⟂ to the build/demold axis or re-level the
  scan in cf-scan-prep first (kill the 4.4° tilt at the source)?
- **OQ5 (C):** pour-gate + vent should anchor at the dome apex off the flange —
  move the pour/vent anchor outboard of the flange inner edge, or carve a flange
  relief at the tip? Confirm where it currently lands first.
- **OQ4:** does the flat flange replace the curve-following cup-wall split
  entirely (C-A1/A2), or only the flange face (C-A3, wall stays curved)?

---

## 9. Cold-read pass-1
- **C1 (tension surfaced, not buried).** The Explore map both says the mating
  face is a single-plane half-space cut (→ should be flat) AND that it bows.
  Rather than pick a story, §3 makes confirming the real A-failure (H1/H2/H3)
  the explicit S0 deliverable. Writing S1 against the wrong hypothesis is the
  biggest waste risk here.
- **C2 (WRONG — corrected by S2).** Pass-1 claimed (B) was "largely re-enabling
  `build_cup_cap_trim_transform` with the plug-side bias." **S2 empirically
  disproved this**: the SeamTrim halfspace BISECTS the cup (172 mm half → 23 mm
  stub) because the cup straddles the cap plane. The disable note was right; (B)
  needs the bounded C-B2 slab and is NOT smaller than (A). Lesson: a disabled
  primitive is a gift only if you re-read WHY it was disabled — the note explicitly
  said "needs the §Q-4 S2 UnionCuboid slab, not a halfspace trim," which pass-1
  under-weighted.
- **C3 (demoldability is the real (A) constraint, not flatness).** The flange is
  easy to make flat (fit a plane). The hard part is that a flat cut on a curved
  body can trap an undercut. The recon must lead with the curvature ceiling
  (OQ2), or S1 ships flat flanges that don't demold.
- **C4 (don't overreach to curved caps).** C-B3 is tempting but the workshop only
  asked for a *flat* mating face. A flat plane that the plug + cup both share is
  sufficient + far cheaper. Held out of scope unless S0 disproves it.
- **C5 (the 4.4° tilt is half the B problem).** Part of "the floor isn't flat" is
  the cf-scan-prep auto-PCA lean, not cf-cast. Fixing it upstream (re-level)
  shrinks (B) to just the trim. OQ3 forces that decision instead of compensating
  for the tilt downstream.
