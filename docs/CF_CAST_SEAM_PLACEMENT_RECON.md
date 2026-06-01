# CF-CAST Seam-Placement Solver Recon

> **Status:** RECON SCAFFOLD (pre-implementation). Consolidated 2026-05-31 (this is
> the converged design; the three cold-read passes that produced it are compressed
> into the decision log, §11).
> **Date:** 2026-05-31
> **Trigger:** workshop cf-view of the 0.5 mm `cast_base_mold_canal_05` apex-pour
> mold flagged a bolt hole hugging the sprue on one side of the dome apex with no
> mirror on the other, and an open question on whether that bolt's **M5 washer
> footprint** (5 mm radius) clears the **Ø10 pour bore** (5 mm radius). Both are
> symptoms of one root cause: **bolt + dowel placement is geometry-blind** — fixed
> count, arc-length-uniform around the body silhouette, the pour handled by three
> competing ad-hoc paths, and a collision model that checks the hole cylinder, never
> the washer footprint.
> **Goal:** replace uniform placement with a single constraint-aware solver that
> places **just enough** fasteners **exactly where the seam needs them**, and (end
> state) generates the flange to fit the fasteners rather than carrying a uniform
> band. The original washer-vs-sprue failure becomes **impossible by construction**
> (see §3.4 — the pour seed places a bolt as close as *feasible* each side, and
> feasibility includes washer-clears-bore).
>
> **Composes on:** the arbitrary-seam-plane silhouette API
> (`CF_CAST_ARBITRARY_SEAM_SILHOUETTE_RECON.md`, S1–S4 shipped) — `Silhouette2d`
> exposes arc-length parameterization, outward normals, signed distance, and ordered
> polylines in an arbitrary seam plane. The solver builds a clean placement substrate
> *on top of* that geometry layer (§3.1).
>
> **End state (agreed 2026-05-31):** the solver becomes the **default** — opt-in gate
> for one arc, prove on `base_mold`, then promote to default and **delete the legacy
> uniform loops, the three pour-handling paths, the uniform flange plate, and their
> config knobs**. Foundation rebuild is explicitly licensed ("reconstruct the
> foundation as deep as needed").

---

## 1. Problem statement

Two independent placement loops scatter cut-cylinders around the seam:

- **Bolts** (`bolt_pattern::build_bolt_pattern_transforms`): `count = 8` fixed,
  positions at `(k + 0.5) / count`, offset `13 mm` outboard.
- **Dowels** (`dowel_hole::build_dowel_hole_transforms`): `count = 4` fixed,
  positions at `(k + 0.5) / count`, offset `10 mm` outboard.

Neither knows *where the seam actually needs a fastener*. Five failures, all observed:

1. **Uniform spacing is gap-blind.** A bolted flange seal wants fasteners clustered
   where the seam tends to gap (corners, either side of a discontinuity) and sparse
   where it's naturally stiff. Arc-uniform spends bolts evenly regardless of demand.
2. **The pour is three ad-hoc paths that don't compose.** Reactive
   `collides_with_pour_gate` skip, proactive `place_pour_flanking_bolts`, and the
   `flank_bolts` toggle switching between them. With `flank_bolts = false` (production
   `base_mold`) the skip path silently drops the ring bolt that fell inside
   `2.8 + pour_r + 1.0 ≈ 8.8 mm` of the bore on one side while keeping its mirror —
   **the exact apex asymmetry the workshop flagged.**
3. **The collision model is the wrong footprint.** It checks the `2.75 mm` *hole*
   against the bore, never the `5 mm` *washer*. A kept near-apex bolt can clear the
   hole by `1 mm` of PLA yet have its washer overhang the open sprue — the workshop's
   open question, which the code cannot even represent.
4. **Count is a magic number, not "just enough."** `8` and `4` are derived from
   nothing. A bigger part is under-clamped; a smaller part is over-drilled.
5. **Bolt↔dowel non-collision is a coincidence.** It relies on `8 vs 4` giving a
   `1/16`-perimeter stagger, propped up by a config validator. Change either count and
   the guarantee evaporates.

---

## 2. Current architecture (as found — file:line)

> Line numbers below are approximate (from a structural read); verify against HEAD in
> S1 — the house style wants exact anchors.

### 2.1 Placement loops

| Concept | What it is | Source |
|---|---|---|
| `BoltPatternSpec` | `clearance_diameter 5.5`, `count 8`, `offset 13`, `skip_pour_gate_collision`, `pour_gate_clearance 1`, `bracket_pour_gate` | `bolt_pattern.rs:~149` |
| `build_bolt_pattern_transforms` | arc-uniform loop `(k+0.5)/count` → `Vec<MatingTransform>` | `bolt_pattern.rs:~309` |
| `bolt_center_at(t)` | silhouette point + outward normal × offset, projected onto binormal seam plane | `bolt_pattern.rs:~462` |
| `find_pour_arc_fraction` | 512-pt scan for where the bore axis is nearest the flange | `bolt_pattern.rs:~531` |
| `place_pour_flanking_bolts` | steps ±outward from `t_pour` until lateral clearance, one bolt/side | `bolt_pattern.rs:~569` |
| `collides_with_pour_gate` | drop bolt if hole within `bolt_r + pour_r + clearance` of bore axis | `bolt_pattern.rs:~274` |
| `DowelHoleSpec` | `diameter 3`, `clearance 0.1`, `depth 5/half`, `count 4`, `offset 10` | `dowel_hole.rs:~110` |
| `build_dowel_hole_transforms` | arc-uniform loop `(k+0.5)/count` → `Vec<MatingTransform>` | `dowel_hole.rs:~195` |
| bolt↔dowel stagger | implicit from `8 vs 4`; locked by a test + config validator | `bolt_pattern.rs:~39`, `config.rs:~993` |

### 2.2 The geometry substrate (reusable)

| Capability | Signature | Source |
|---|---|---|
| ordered closed loop | `polylines()`, `longest_polyline_with_arc_length()` | `silhouette_2d.rs:~457/~522` |
| arc-length point | `point_at_arc_fraction(t) -> Point2` | `silhouette_2d.rs:~561` |
| outward normal | `outward_normal_at_arc_fraction(t) -> Point2` | `silhouette_2d.rs:~604` |
| **signed distance (2D)** | `signed_distance_to(u, v) -> f64` — the feasibility field | `silhouette_2d.rs:~411` |
| plane↔world | `to_world`, `dir_to_world`, `SeamPlaneBasis::from_anchor_normal` | `silhouette_2d.rs:~393/~123` |
| seam basis | `Ribbon::seam_plane_basis() -> Option<SeamPlaneBasis>` | `ribbon.rs:~586` |
| flange | `FlangeSdf` — band `[inner_offset 2, width 20]`, `thickness 4/half` | `flange.rs:~16` |

### 2.3 Wiring

- Config: `BoltPatternConfig` (`config.rs:~487`), `DowelHoleConfig` (`config.rs:~437`),
  `PourGateConfig{apex_axial, flank_bolts}` (`config.rs:~312`); cross-field gates
  (`config.rs:~886`).
- Derive: features attached to the ribbon **flange → dowel → bolt**
  (`derive.rs:~556/~562/~583`); `bracket_pour_gate` set when `apex_axial && flank_bolts`
  (`derive.rs:~581`).
- Emit: positions become `SubtractCylinder` transforms carved **post-marching-cubes**
  by `apply_mating_transforms` (`spec.rs:~1104`). The carving is generic; only the
  positions are bespoke — so the placement renovation reuses the carve stage unchanged.

---

## 3. The converged design — one solver on a clean substrate

### 3.1 Substrate: a clean analytic flange path (not the raw MC silhouette)
The raw marching-squares silhouette polyline is sampled at a *fixed*
`SILHOUETTE_GRID_STEP_M = 0.5 mm` grid (independent of `mesh_cell_size_m`), so it carries
a ≤ 0.5 mm staircase and non-uniform vertex spacing. Positions on it are fine (sub-fastener
tolerance), but its *derivatives* — the outward normal and the curvature the corner-seeder
needs — are dominated by that staircase. **Build a clean placement curve once:** resample
to uniform arc-length stations, and take normals/curvature from a windowed stencil (the
low-pass lives in the *derivative*, not in moved points — no shrink). Arc length, outward
normal, and curvature all come from this curve; the MC silhouette is used only to *build*
it, never to *place* on it. (S1 correction: placement is *already* mesh-cell-independent
because the silhouette grid is the fixed 0.5 mm const — see §7.2; the substrate's job is
staircase-stable derivatives + uniform stations, not mesh-cell decoupling.)

### 3.2 Placement is a free 2D point (arc-length + radial offset)
A fastener position is a free **2D** point in the flange region: solve **both** its
arc-length `s` along the substrate **and** its radial offset `d` outboard. Centre `C(s,d)
= P(s) + d·n̂(s)`, projected onto the seam plane. `d` is no longer the hand-set 13/10 mm.

`d` is bounded by one surviving physical floor: the **inboard washer-vs-cup-wall-step
clearance** (the washer must clear the cup-wall outer step — this was *half* the old 13 mm
rationale). That floor is now *computed*, not hand-set; the *outboard* "fit inside a 20 mm
band" half of the old rule disappears entirely (the flange follows placement, §4).

### 3.3 Feasibility — two regimes (path-dependent)
A fastener of footprint radius `ρ` (the **washer** radius for bolts; hole + wall for
dowels) is feasible at `C` per whichever flange regime is active:

- **Incremental regime (uniform band still present):** the footprint disk lies inside the
  **2D band** `inner ≤ signed_distance ≤ width`. Because the signed distance is a true
  (1-Lipschitz) field, the disk of radius ρ is inside the band **iff `inner+ρ ≤ sd(center)
  ≤ width−ρ`** — a single center evaluation (S1 cold-read C1: exact-conservative, never
  false-accepts, and pinch-aware since sd is the true nearest-boundary distance in all
  directions; strictly better than the perimeter-sampling first proposed). A 2D query on
  the clean `SeamProfile` loop — **no built `FlangeSdf` is needed.**
- **Demand-flange regime (§4):** there *is* no band to fit inside — the flange is generated
  to host the fastener. Feasibility collapses to plain **clearance** tests: washer clears
  the body / cup-wall step (§3.2 floor), boss clears the pour channel, boss doesn't overlap
  a neighbour's footprint or boss.

Either way the predicate **subsumes** the old washer-vs-cup-wall offset, the washer-vs-pour
clearance the workshop asked about, and the pour collision skip.

### 3.4 Seeds (geometry → required positions)
Seeds are *generated*, never special-cased:
- **Pour brackets** — project the bore axis onto the seam (reuse `find_pour_arc_fraction`),
  emit two seeds "nearest *feasible* station on each side, as close as feasible." Because
  feasibility includes washer-clears-bore, **this is what makes the original failure
  impossible.** Replaces all three legacy pour paths.
- **Corners** — from the *smoothed* substrate's turning angle, a seed near each
  high-curvature vertex (gapping concentrates at corners). Threshold tuned in S0.
- **Registration extremes** (dowels only) — PCA / arc-length extremes of the long axis →
  two seeds at maximum moment arm.

The pour is modelled **once, correctly**: its *exclusion* is the absent flange / a swept
**channel** (not a point disk); its *seed* is the pierce arc-length.

### 3.5 Algorithm: subdivision first, sampler as fallback
For the common mostly-feasible seam, the elegant deterministic form is:
1. **Partition** the loop at the seeds and at the boundaries of infeasible intervals.
2. **Subdivide** each resulting arc into `ceil(len / max_pitch)` evenly-spaced points →
   **count emerges; no magic `8`.**
3. Seeds are fixed; subdivided points are even by construction (no relaxation needed).

Only when the feasible set is genuinely **hole-riddled** (so even subdivision can't satisfy
`max_pitch` without landing in gaps) fall back to **seeded, masked farthest-point
(Poisson-disk) insertion + a few capped Lloyd iterations** on the feasible sub-intervals.
Determinism throughout: ties broken by lowest arc-length; **no RNG**.

> **S2 IMPLEMENTATION NOTE (2026-06-01).** The Poisson/Lloyd fallback proved **unreachable**
> under the implemented feasibility model and was **not built**. Because the solver scans the
> *full* radial range at every arc, an arc is infeasible only when **no** offset works there —
> so the interval partition (step 1) is exact, and every physical obstacle (pour bore, dowel
> boss, band pinch — all ≫ the 1 mm `arc_step`) splits into its own feasible interval; even
> subdivision then never lands in a hole. The only residual case is a **sub-`arc_step` hole**
> (not physical for a fastener), handled by a deterministic **local nudge** of the offending
> point to the nearest feasible station in its gap — a fraction of the Poisson code, same
> determinism. (Revisit only if the demand flange's clearance-only feasibility at S4.5 ever
> produces a genuinely fragmented mask.)

### 3.6 One solver, run twice
```
place_fasteners(substrate, feasibility, excluded, FastenerClass {
    footprint_radius,        // washer R (bolt) | hole+wall (dowel)
    fill: Option<max_pitch>, // Some(pitch) bolt | None/loose dowel
    seeds,                   // pour+corners (bolt) | axis-extremes (dowel)
}) -> Vec<Placement>
```
- **Dowels run first** → footprint = hole+wall, seeds = axis extremes, fill = None.
- **Bolts run second** → footprint = washer R, seeds = pour brackets + corners,
  fill = `Some(max_pitch)`, and `excluded =` the **pour channel + every dowel footprint
  *and its boss*** (under §4, a dowel's boss is part of its exclusion, not just the hole).

Dowels-first is the **contract** (registration is primary; clamp fills around it), the
deliberate inverse of today's accidental `8-vs-4` ordering. Each `Placement` maps to the
existing `SubtractCylinder` (bolts through the flange thickness; dowels straddling the
seam) — **emission unchanged.**

### 3.7 `max_pitch`
For a hand-tightened silicone-mold flange the governing rule is **even contact pressure**
(silicone head ≈1 kPa is negligible), so `max_pitch` is a workmanship constant (~25–35 mm),
not a pressure calc. Pin it in S0 (`base_mold` seam length vs. a bolt count the workshop
judges right); expose as `[bolt_pattern].max_pitch_m` (default ~0.030). Under the demand
flange (§4) `max_pitch` and the seal-land stiffness are the *same physics* (§4.4) — keep it
a tuned constant for iter-1, wire the coupling once the physical gate gives a seal-pressure
number, so the two aren't tuned in conflict later.

### 3.8 Per-layer placement (C6)
The mold is 3 layers, each its own 2-piece cup with a different (growing) silhouette.
**Solve on the outermost (largest) silhouette, then snap/validate each position onto every
inner layer's substrate; drop (with `warn`) any position that can't be made feasible on
all layers.** Shared positions across the stack = the workshop aligns one pattern. (The
alternative — per-layer-distinct patterns — is out of scope, §9.)

---

## 4. Demand-driven flange (the end-state flange)

The uniform `[inner_offset 2, width 20]` band conflates three roles, which is why it's both
wasteful *and* pinch-prone: **(1) seal land** — the continuous PLA-on-PLA contact ring
(any interruption leaks); **(2) fastener host** — material needed only *at* the fasteners;
**(3) handling/stiffness**. Decouple them and each gets exactly what it needs.

### 4.1 The skeleton model — seal ring + per-fastener "tadpoles"
Express the flange as material within a role-dependent radius of a **demand skeleton**:
- **Seal ring** — a thin closed contact band hugging the cavity. Its inboard start is
  **gasket-dependent**: with `[gasket] = None` (current) the PLA-on-PLA land can hug the
  cavity tight (`body_dist ≈ 0`) to minimise the unsealed lip; `inner_offset = 2 mm` only
  exists to clear a gasket *channel*, so it applies only if a gasket is re-enabled. Width
  `land_width ≈ 6 mm` (seal-mechanics tunable). Continuous by construction.
- **Tadpole per fastener** — an SDF **capsule** swept from a seal-ring anchor out to the
  fastener centre, radius tapering `web_w/2 → boss_r`, `boss_r = washer_radius + wall_margin`.
  The spoke connects the boss to the ring (one manifold, clamp load transfers); the head
  hosts the footprint. When `d` is small the boss merges into the ring and the spoke is
  ~zero-length — the capsule handles it; **don't add a guard that rejects degenerate spokes.**

```
flange_sdf(P) = min(                                 // union of demand primitives
    seal_ring_band(P),                               // continuous closed land
    min_i  capsule(P;  ring_anchor_i → fastener_i,   // spoke→boss "tadpole"
                       r: web_w/2 → boss_r_i) )
  ∩  thickness_slab(P)                               // ±flange_thickness, both cup halves
```
All primitives are analytic in the seam-plane basis × thickness — compatible with the
silhouette-basis flange build + SDF→MC. Result: a **scalloped flange** (full seal ring
inside, bosses at each fastener, bays removed) — the classic valve-cover profile, now
*generated* from the fastener set.

### 4.2 The dependency flip (the payoff)
```
  OLD:  build uniform flange  →  place fasteners where the band allows  (feasibility search)
  NEW:  place fasteners (clearances only)  →  generate flange to fit    (feasibility guaranteed)
```
The "enough flange?" question stops being a constraint and becomes a build step. This is
what makes the magic 20 mm die alongside the magic 13/10 mm and the magic 8 — and it's why
the §3.3 feasibility mask collapses to clearance tests under this regime.

### 4.3 Risks specific to the demand flange
- **Seal continuity is sacred.** Bays may remove material only **outboard** of the seal
  ring; the inner contact land stays full-width all the way around. Enforce + test
  (closed-loop continuity check).
- **Mid-bay clamp loss** — with bolts only at bosses, does the land lose contact between
  them? That's `max_pitch`'s job, now load-bearing. Conservative fallback: a slightly wider
  continuous land so bays are inherently sealed; tighten at the physical gate.
- **More primitives → more F4 risk.** A union of N tadpoles + ring is more boolean junctions
  than one plate. Run the F4 gate on the demand flange; set spoke/boss minimum widths to
  print-safe floors.
- **Optional outer tie-rim** — a thin (1–2 mm) continuous *outer* rim linking the boss heads
  (a "ladder" flange) buys handling rigidity cheaply. Config toggle, default off (pure
  scallop); revisit if the bare scallop feels flimsy.

### 4.4 `max_pitch` = seal-land stiffness
The seal bowing open between bolts is *exactly* what sets `max_pitch`, so seal-land geometry
and bolt count are the same physics: `land_width, thickness → land stiffness → max_pitch →
count + positions → bosses at those positions`. Wire once a physical seal-pressure number
exists (§3.7).

---

## 5. What collapses

| Today | Becomes |
|---|---|
| `collides_with_pour_gate` + `place_pour_flanking_bolts` + `flank_bolts` toggle | a pour seed + a swept-channel exclusion |
| magic `count = 8` / `count = 4` | emergent from `max_pitch` + seeds (§3.5) |
| hole-vs-bore cylinder check | washer-footprint feasibility (§3.3) |
| `13 mm` / `10 mm` hand-tuned offsets | solved radial `d`, floored by inboard clearance (§3.2) |
| `8-vs-4` stagger coincidence + validator | shared point set, per-class radii (§3.6) |
| uniform 20 mm flange plate | seal ring + per-fastener bosses (§4) |
| `bracket_pour_gate`, `skip_pour_gate_collision`, `flank_bolts`, fixed offsets | **retired** |

---

## 6. Two build paths (the architecture decision)

§3.1 (clean substrate) is unconditionally core. The remaining choice is *when* the demand
flange (§4) lands, because **§4 subsumes the §3.3 feasibility mask** (under a demand flange,
placement is clearance-only — the band mask is never built):

- **Incremental** — ship the solver against a still-uniform band first (2D placement +
  feasibility mask live), prove on `base_mold`, *then* do §4 and delete the mask. Lower risk
  per step; the mask is built then thrown away.
- **Direct** — go straight to §4: placement is clearance-only from day one, the flange is
  generated to fit, the mask is never built. Fewer total concepts, no throwaway code, but
  the biggest single step (rewrites flange gen + flips derive order before the solver has a
  physical-gate win behind it).

**Lean incremental unless S0 shows the uniform band is the active blocker** (e.g. the apex
pinch can't host a feasible bolt at *any* offset) — then **direct**, since §4 is the actual
fix for a pinch. S0 makes this call.

---

## 7. Phasing (single source of truth)

| Phase | Does | Done when (acceptance gate) |
|---|---|---|
| **S0** measure | Instrument `cast_base_mold_canal_05`: actual bolt centres, washer-vs-pour on *both* apex sides, `flange_room` along the whole seam, which uniform stations land infeasibly. Pin `max_pitch` + corner-curvature threshold. **Decide incremental vs direct (§6).** | Washer-vs-sprue answered numerically; `max_pitch` + threshold pinned; §6 path chosen. |
| **S1** substrate + feasibility | Build the §3.1 clean analytic flange path; the 2D feasibility predicate (§3.3); per-layer snap (§3.8). Verify the as-found `file:line` (§2). Pure addition, zero behaviour change. | **DONE (§7.2).** Derivatives staircase-stable (normals < 8°, no false corner spike under 0.5 mm noise); feasibility (Lipschitz center test) + non-convex + corner cases unit-tested. |
| **S2** solver | `place_fasteners` (§3.5/§3.6): subdivision-first, Poisson fallback, deterministic. | Unit tests green on ≥4 synthetic seams incl. a masked/holed one — count, even spacing, seed honouring, exclusion, determinism under perturbation. |
| **S3** bolts (gated) | Route bolts through the solver behind `[cast].smart_placement` (default off). 2D placement. Regen `base_mold`, A/B vs current. | **DONE (§7.3).** `base_mold`: washer clears the Ø10 bore on **both** apex sides (+0.5/+0.6 mm); 14 bolts (S0 target 12–14; legacy 7–8); left/right apex symmetric. |
| **S4** dowels (gated) | Route dowels through the solver (registration-extreme seeds). | **DONE (§7.4).** `base_mold` A/B: 2 dowels at the long-axis extremes, all 3 layers share 2 dowels + 15 bolts, apex bracketed both sides every layer, no dowel↔bolt overlap (min 8.9 ≥ 8.6 mm), moment arm 152–177 mm (full long-axis span). |
| **S5** promote + delete | Flip `smart_placement` default **on**; delete the legacy *placement* machinery — uniform bolt/dowel loops, the three pour paths, the `flank_bolts`/`bracket_pour_gate`/`skip_pour_gate_collision` knobs + the bolt↔dowel arc-stagger validator; fold the duplicate per-layer silhouette rebuilds (§MA-7 reuse); re-baseline iter-1 byte-identity tests deliberately. **Keep the uniform `Plate` flange as the default** — the flange redesign is S4.5, so S5 stays a clean "promote the proven solver, delete legacy placement" cut with its own scoped re-baseline. | `grade-all` green; byte-identity re-baseline reviewed; no legacy *placement* path remains. |
| **S4.5** demand flange | §4: add `FlangeKind::Demand` (seal-ring + per-fastener tadpoles); flip the derive order (flange generated *after* placement, clearance-only feasibility). Lands as a sibling of `Plate` (own re-baseline) then **becomes the default for `base_mold`** — this is the print target. *Incremental path only; on the direct path this folds into S3.* | `base_mold` regen: mass/print-time ↓ vs plate; F4 clean; seal-ring continuity test passes; cf-view scallop + apex boss look right. |
| **gate** physical (print 1) | **PLAN (2026-06-01, user): print 1 happens only after the software is "basically perfect" — i.e. AFTER S5 *and* S4.5 — not on an intermediate uniform-plate build.** Workshop print + cast the finished `base_mold` (smart placement + demand flange): apex clamp holds? shared bolts/dowels seat? scalloped land seals between bolts? | Empirical — the single gate validates placement **and** seal-geometry together. Tradeoff: a failure needs disambiguating (placement vs seal); accepted because the §7.4 A/B already de-risks placement and the user prefers one print of the best software. Failure → iter-N; success closes the arc. |

---

## 7.1 S0 RESULTS (2026-05-31)

**Method:** env-gated probe in `build_bolt_pattern_transforms` (dumps the silhouette,
flange spec, bolt + pour cylinders); 3 mm regen of the production `base_mold` config
(placement is silhouette-driven ≈ cell-independent); analysed the **outermost** layer
(seam perimeter **394 mm**). Geometry confirmed: flange `[inner 2, width 20]` × 4 mm;
bolt r 2.75, offset 13; pour bore Ø10 at the apex, axis ≈ `(0.09, −0.14, 0.99)`,
spanning z ≈ 70→159, **piercing the seam at z ≈ 87 mm, y ≈ −9 mm**.

1. **Washer-vs-sprue (the original question) — RESOLVED, no defect.** Every surviving
   bolt clears the bore hugely: nearest is the `+y` apex bolt at **42 mm** from the bore
   axis (washer overhang **0**). The worst-case "~1.2 mm overhang" spec bound (from the
   8.8 mm collision-skip threshold) **does not occur** — the skip drops the close bolt
   entirely, so the *next*-nearest survivor is far. The as-built cast is fine on
   clearance; my earlier worst-case chat estimate was pessimistic.
2. **The apex asymmetry — CONFIRMED + explained.** Outer & middle layers carry **7**
   bolts (one collision-skipped), inner carries **8**. On the outermost the `+y` side's
   topmost bolt reaches **z = 66.8**, the `−y` side only **z = 44.6** — a 22 mm apex
   asymmetry because the `−y` apex bolt (which at offset 13 mm fouled the bore) was
   dropped, not relocated. The apex pierce (z ≈ 87) has **no clamp within 42 mm** → the
   real problem is an **under-clamped, asymmetric apex**, not washer fouling.
3. **Count/pitch → `max_pitch`.** 7–8 bolts on 394 mm = **49–56 mm mean pitch** — sparse
   for even-contact PLA-on-PLA sealing. Targets: 25 mm→16, 30 mm→14, 35 mm→12 bolts. So
   **"just enough" is *more* than today, not fewer** — the seal wants ~30–35 mm pitch
   (~12–14 bolts), placed well. Provisional `max_pitch = 0.030`; workshop confirms at the
   physical gate (OQ1).
4. **Corner threshold.** The raw silhouette carries spurious **90° MC-staircase**
   vertices and the flat cap/floor edge (z ≈ −60, 31–51°); the organic body itself is
   **smooth (mean 1.25°/vtx)**. → corner detection **must** run on the smoothed substrate
   (validates §3.1); `base_mold` has essentially **no genuine organic corners** — the
   pour bracket + even-fill dominate, cap-edge seeding is optional.
5. **§6 decision → INCREMENTAL (corrected — see VERIFICATION).** The apex *pierce station
   itself* is un-bolt-able at any offset — its outward normal is **∥ the bore axis**, so
   offsetting outboard slides the bolt *along* the bore, never clear of it (the bore test
   fails at every `d`). But the design never bolts the pierce — the pour-bracket seed
   steps to each side. **A feasible washer-bolt exists within 5–8 mm arc of the pierce on
   both sides** (outermost: −side 5.4 mm @ d=14.5, +side 7.7 mm @ d=11.5, each clearing the
   bore by ~10 mm). So the apex **can** be clamped close on both sides → incremental holds;
   the **pour-bracket seed (§3.4) + variable-`d` (§3.2)** is the fix (the offset differs
   per side — 14.5 vs 11.5 mm — so variable-`d` genuinely earns its place). The production
   cast fails only because `flank_bolts=false` *drops* the colliding bolts instead of
   bracketing. **Demand flange (§4) not required for `base_mold`.**

### VERIFICATION (0.5 mm, 2026-05-31 — cold read of the S0 impl)
A cold read of the probe + analysis (S0-C1…C6) caught a real error and prompted a
rigorous re-run at the **production 0.5 mm cell**, with apex feasibility computed **in
Rust** from the true `signed_distance_to` + offset geometry. (The 3 mm pass used an
offline test that approximated the outward normal in the world x–z plane — invalid,
because the fitted seam normal is **[0.838, 0.546, −0.001]**, ~33° off-axis.)
- **Findings 1–3 confirmed cell-independent.** Outermost perimeter (394.1 mm), bolt count
  (7), and bolt positions are stable 3 mm→0.5 mm; nearest bolt **42.1 mm** from the bore,
  washer overhang **0** — the **original washer-vs-sprue question: clears comfortably**,
  on the real cell.
- **The original Finding D ("feasible at d ≈ 14–15 mm") was WRONG and is retracted.** The
  rigorous test shows the pierce is infeasible at *every* offset; the correct result is
  the arc-scan in #5 (feasible 5–8 mm to each side). Same §6 conclusion (incremental), but
  for the right reason — pour-bracket *stepping*, not variable-`d` *at the pierce*.
- **Lesson for the solver:** the apex clamp is a **pour-bracket** problem (step to the
  nearest feasible station), and the normal-∥-axis obstruction at the pierce is geometric
  and fundamental — corroborating the §MA flank-bolt difficulty. §3.4's pour-bracket seed
  must search arc-position (not just offset) and own the apex.

**Net:** proceed **incremental** — core = §3.1 clean substrate + §3.2 variable-`d` + the
§3.4 pour-bracket seed + the §3.5/§3.6 solver. Demand flange (§4) deferred.

---

## 7.2 S1 RESULTS (2026-06-01)

**Shipped** `design/cf-cast/src/seam_profile.rs` — `SeamProfile`, the clean
placement substrate (§3.1). Pure addition; nothing in the pipeline calls it yet
(wired in S3). 8 unit tests + the full cf-cast lib suite green, clippy-clean
(pedantic+nursery), fmt-clean. §2 `file:line` anchors verified against HEAD (minor
offsets, no drift).

API: `from_silhouette` / `from_polyline` (uniform arc-length resample of the body
loop) → `point_at` / `tangent_at` / `outward_normal_at` / `curvature_at`,
`signed_distance` (clean-loop feasibility field), `band_feasible` (§3.3),
`nearest_arc` (§3.8 per-layer snap), `to_world`.

**Correction folded in (refines S0-C3).** The silhouette is sampled at a *fixed*
`SILHOUETTE_GRID_STEP_M = 0.5 mm`, **independent of `mesh_cell_size_m`** — so
placement was *already* mesh-cell-independent (S0 corroboration: the outer-layer
seam perimeter is bit-identical 394.141 mm at 3 mm and 0.5 mm). So S0-C3's "shifts
with `mesh_cell_size_m`" framing was wrong; the real noise is the fixed 0.5 mm
marching-squares **staircase**. The substrate's job is therefore (a) uniform
arc-length stations and (b) **staircase-stable normals + curvature** — done
*shrink-free*: stations are the faithful resampled loop (no Laplacian
point-smoothing, which would shrink high-curvature regions), and the low-pass
lives in the **derivative stencil** (windowed central-difference tangent/normal at
2 mm; Menger curvature at 8 mm).

**Gate (reframed).** Not "same curve at 1.5 vs 0.5 mm mesh cell" (trivially true —
the silhouette doesn't depend on the mesh cell) but **derivatives stable under the
0.5 mm staircase, no false corner spike**. Tested: against full-0.5 mm-grid
quantization of a 30 mm circle, normal-angle drift < 8° and curvature never spikes
to a corner-like value (stays < 3/r) — so the corner-seeder (§3.4) won't
false-trigger on staircase noise.

**Design note for S2+.** Curvature is a second derivative, so it needs a *wider*
window than the normal (turning signal `~W/r` vs staircase noise `~grid/W` ⇒
`W ≳ √(grid·r)`). The corner-seeder must use the robust Menger curvature, not a
finite-difference of the tangent.

**S1 cold-read pass (2026-06-01), folded in:** C1 — `band_feasible` switched from
16-point perimeter sampling to the **Lipschitz center test** `inner+ρ ≤ sd(c) ≤
width−ρ` (exact-conservative, O(1), more pinch-aware; see §3.3). C2 — added
**non-convex** (peanut, concave waist) and **sharp-corner** (square) tests — the
former verifies the outward normal + sign across a concavity, the latter that
Menger curvature actually spikes at a real corner (the corner-seeder's job). C3 —
shrank the outward-normal probe (2 mm → 0.5 mm) so it can't cross a sub-2 mm
concavity. C4 — clamped the curvature window to ≤ perimeter/4 (small-loop
antipodal guard). C5 — cleaned up casts. **Noted for S2:** `signed_distance` /
`nearest_arc` are O(n) scans — if the solver calls them O(n) times, add a spatial
index (S1-C6).

**Bookmark.** A real-body integration check (build `SeamProfile` from the
`base_mold` outer silhouette, confirm perimeter ≈ 394 mm + feasibility behaviour)
is deferred to S3, where the solver is wired and `base_mold` is regenerated anyway.

---

## 7.3 S3 RESULTS (2026-06-01)

**Shipped** bolt routing through the solver behind `[cast].smart_placement`
(default **off**; legacy byte-identical when off — full cf-cast lib suite 334/0).
`plan_smart_bolt_placements` (cf-cast `bolt_pattern.rs`) solves on the outermost
`SeamProfile`, then snaps each placement onto every layer (`seam_solver::snap_placement`,
§3.8) and keeps only positions feasible on **all** layers; `build_bolt_pattern_transforms`
gained `smart_bolts: Option<&[Point2]>`. Pour → `Exclusion::Channel` + a `PourPierce`
seed; dowels → `Disk` exclusions (`SeamPlaneBasis::project` maps the world transforms
into the seam plane). Feasibility: footprint = washer R (5 mm), band = flange
`[inner_offset, width]`, `d_floor = wall + washer + 1 mm` (≈ 11 mm — the computed form
of the legacy 13 mm offset; admits S0's feasible 11.5 mm apex bolt). `max_pitch` =
`DEFAULT_MAX_PITCH_M` (30 mm).

- **Bookmark resolved.** Outer `SeamProfile` perimeter on `base_mold` = **393.9 mm**
  (the expected ≈ 394).
- **Gate met (coarse-cell probe; placement is silhouette-driven, cell-independent):**
  14 bolts (S0 target 12–14; legacy 7–8 with a *dropped* apex bolt), apex bracketed on
  **both** sides clearing the Ø10 bore by **+0.5 / +0.6 mm**, left/right symmetric.

**Bug the real-body A/B caught (fixed here, regression-tested).** `place_fasteners`'
`Interval::Open` arm assumed `lo < hi`. When the infeasible zone (the pour) sits away
from arc 0, the complementary feasible run **wraps arc 0** → `feasible_intervals` emits
`Open{lo > hi}` (e.g. lo=111 mm, hi=99 mm). The old arm numeric-sorted to `[99, 111]`
and filled the **12 mm infeasible gap** instead of the **382 mm feasible arc**,
collapsing the ring to just the 2 pour brackets. Fix: the Open arm now works in forward
offsets from `lo` (`span = (hi-lo).rem_euclid(perim)`; modular boundary checks). The S2
synthetic tests passed only because they placed the pierce at arc 0 (so the infeasible
zone straddled 0 and the feasible run didn't wrap) — added
`pour_pierce_away_from_arc_zero_still_fills_the_ring`. **Lesson: test the solver with the
obstacle OFF arc 0.**

**Deferred / noted.** (a) `plan_smart_bolt_placements` rebuilds the seam + dowel
silhouettes per layer that the compose pass also builds (silhouette extraction is the
dominant per-piece cost, §MA-7) — roughly doubles silhouette work on a smart run;
acceptable for opt-in S3, fold into S5's legacy-deletion + silhouette reuse. (b) The
cf-cast-cli bolt↔dowel arc-stagger validator still gates the (now-superseded) legacy bolt
`count` even under `smart_placement`; harmless at default counts, clean up in S5.
(c) `[bolt_pattern].max_pitch_m` exposure (§3.7) deferred.

---

## 7.4 S4 RESULTS (2026-06-01)

**Shipped** dowel routing through the solver behind the same `[cast].smart_placement`
gate (default **off**; legacy byte-identical when off — full cf-cast lib suite
**339/0**, +5 S4 tests). New shared module `seam_placement.rs` houses the
plumbing both planners use — `seam_silhouette`, `pour_exclusions`,
`smart_center_to_world` (moved out of `bolt_pattern.rs`), and the extracted
`cross_layer_snap` (the §3.8 per-layer snap + all-layer-feasible filter) — so the
dowel and bolt planners are exact analogues with no bolt↔dowel module dependency.

`plan_smart_dowel_placements` (cf-cast `dowel_hole.rs`): `fill = None` (seeds
only), seeds = the **two long-axis registration extremes** at maximum moment arm
(`long_axis_extreme_seeds` — PCA principal axis over the loop's uniform stations,
min/max projection; deterministic coordinate-axis fallback when near-isotropic,
Risk #2). Feasibility: footprint = dowel hole + wall (`smart_dowel_footprint`,
≈ 3.6 mm), `d_floor = wall + footprint + 1 mm` (the computed analogue of the
legacy hand-set 10 mm offset). The pour bore is excluded as a swept channel so a
dowel seeded at the apex steps clear of it. `build_dowel_hole_transforms` gained
`smart_dowels: Option<&[Point2]>`.

**Dowels-first contract wired (§3.6):** `spec::mesh_and_gate_v2_pieces` now solves
the dowels first, then `plan_smart_bolt_placements` takes the per-layer dowel
plan and excludes those footprints (disk radius = `smart_dowel_footprint`, so the
washer keeps wall to each hole) — **replacing** the S3 legacy
`build_dowel_hole_transforms`-derived dowel exclusions. Both per-layer center
slices thread through `compose_piece_shared(.., smart_dowels, smart_bolts)`.

- **C1 RESOLVED — real-body A/B on `base_mold` (3 mm probe, release; placement is
  silhouette-driven / cell-independent).** The 3-layer cross-layer snap, never run
  end-to-end before, is now confirmed on the real body:

  | layer | perimeter | dowels | moment arm | bolts | apex (+/−) | min bolt↔dowel |
  |-------|-----------|--------|------------|-------|------------|----------------|
  | 0     | 336.8 mm  | **2**  | 152.2 mm   | **15**| +9.0/−15.1 | 10.5 mm        |
  | 1     | 368.4 mm  | **2**  | 170.5 mm   | **15**| +6.3/−9.0  | 8.9 mm         |
  | 2     | 393.9 mm  | **2**  | 177.3 mm   | **15**| +6.7/−10.4 | 9.4 mm         |

  All 3 layers carry **one shared dowel count (2)** and **one shared bolt count
  (15)**; the apex is bracketed on **both** sides of the pierce on every layer; the
  dowel moment arm (152–177 mm) is essentially the body's full long-axis span
  (maximal registration leverage); and every bolt washer clears every dowel
  footprint (min 8.9 mm ≥ the 8.6 mm = footprint 3.6 + washer 5.0 requirement) —
  **no overlap in the shared set.** Outer perimeter 393.9 mm matches the §7.3
  bookmark exactly. Bolt count rose 14 → 15 vs S3 because the 2 long-axis dowels
  free the arc space the legacy 4-dowel exclusions occupied.

**Deferred / noted.** (a) Same per-layer silhouette-rebuild duplication as S3 — the
dowel planner now adds a second set; fold into S5's silhouette reuse. (b) The
moment-arm gate is qualitative (no hard target in the recon); the 152–177 mm
result is the geometric maximum, so it passes trivially — pin a number only if the
physical gate shows dowels too close to the apex. (c) `DowelHoleSpec.count` is now
superseded by the solver's 2 (the legacy `(k+0.5)/count` loop only runs when
`smart_placement` is off); the count knob + the bolt↔dowel arc-stagger validator
both retire in S5.

---

## 8. Risks / unknowns

1. **A pinch the demand flange can't rescue.** If even bosses can't host a feasible bolt
   within `max_pitch` at the apex, the seam may under-clamp there. Surface loudly (warn on a
   dropped pour-bracket seed), never silently drop. §4 *is* the mitigation; if it's not
   enough, the apex needs a geometry rethink (3-piece, or local flange thickening).
2. **PCA degeneracy** on a near-symmetric part → unstable dowel-extreme seeds. Deterministic
   tie-break (lowest arc-length).
3. **Poisson/Lloyd determinism** (fallback path only) → cap iterations, deterministic
   midpoint rule, no RNG.
4. **Pour projection on a tilted fitted seam** — reuse `find_pour_arc_fraction`'s
   finite-segment clamp (the cold-read I1 fix); don't reintroduce the infinite-axis bug.
5. **Corner over-seeding** — detect curvature on the *smoothed* substrate (§3.1), not the raw
   silhouette, or MC facets fake corners. Threshold tuned in S0.
6. **F4 exposure of the demand flange** — more boolean junctions; gated at S4.5.
7. **S5 re-baselines every output** — the deletion is the point, but it must be a deliberate,
   reviewed re-baseline, not incidental drift. S4.5 then re-baselines again when `Demand`
   becomes the default flange — kept a separate software step (own scoped re-baseline) so a
   placement bug and a seal-geometry bug never share one re-baseline, even though both ship
   before the single physical print (gate).

---

## 9. Out of scope (this arc)

- **Per-layer distinct patterns** — solve-on-outermost + snap (§3.8) keeps one shared pattern.
- **Counter-bored / recessed heads** — workshop declined in §B scope-lock.
- **Integral registration** (tongue-and-groove / keyed seam vs loose dowels) — reopens the §M
  pin→dowel decision; its own future arc.
- **Derived `max_pitch` from a real seal-pressure model** — iter-1 keeps it a tuned constant
  (§3.7/§4.4); wire after the physical gate.

---

## 10. Open questions for workshop

1. **`max_pitch`** — confirm ~25–35 mm from S0 (`base_mold` seam length + a bolt count that
   feels right in the hand)?
2. **Dowel target** — pure 2-pin locating, or 2 locators + loose shear-fill along the long
   axis to resist clamshell peel during pour? (Solver knob; default 2 + minimal.)
3. **Corner seeding** — force bolts at high-curvature corners, or is even-fill enough?
   (Physical gate: does the real cast gap at corners?)
4. **Washer-over-trough** — must the washer fully clear the *open* pour trough (costs bolts
   near the apex), or is a small overhang acceptable since the trough is open and the funnel
   occupies it? S0 data + the print decide; feasibility supports either as a threshold.
5. **`land_width` + outer tie-rim** (§4) — start ~6 mm land; want the ladder rim for handling
   or pure scallop for minimum mass?

---

## 11. Decision log (cold-read passes 1–3, folded)

The converged design above absorbed three review passes; this is the audit trail of what
changed and why, so the body needn't carry the corrections inline.

- **Pass-1 (initial scaffold).** Established the one-solver framing, seeds, dowels-first,
  per-arc warn-on-dropped-seed. Open worry: "is the `FlangeSdf` buildable at placement time?"
- **Pass-2 corrections.**
  - *Mask is 2D, not 3D.* Washer support is a lateral question — the 2D silhouette band, not
    the 3D `FlangeSdf`. **Moots the pass-1 worry: no `FlangeSdf` is needed.** → §3.3.
  - *Kill the fixed offset.* The 13/10 mm offsets were magic; promote `d` to a solved DOF. →
    §3.2 (and N1 below: the inboard clearance *floor* survives, now computed).
  - *Substrate was MC-jittery.* Place on a clean analytic curve, not the raw silhouette. → §3.1.
  - *Simplify the algorithm.* Subdivision-first; Poisson/Lloyd is the holed-mask fallback only.
    → §3.5.
  - *Pour is a swept channel, not a disk.* Exclusion via absent flange / channel; seed at the
    pierce. → §3.4.
  - *Per-layer was undefined.* Solve-on-outermost + snap. → §3.8.
- **Pass-2 foundation expansions (licensed).** 2D placement (§3.2), clean substrate (§3.1),
  and the **demand-driven flange** (§4) — which *inverts* the flange↔placement dependency and
  subsumes the feasibility mask. Integral registration deferred (§9).
- **Pass-3 nits, folded:**
  - **N1** — the offset magic only half-dies: the outboard-band requirement disappears, but
    the inboard washer-vs-cup-wall clearance survives as a *computed floor* on `d`. → §3.2.
  - **N2** — the seal-ring inboard start is gasket-dependent (`inner_offset` only clears a
    gasket channel; PLA-on-PLA land can hug the cavity). → §4.1.
  - **N3** — under §4 the bolt exclusion set is the pour *channel* + every dowel footprint
    *and its boss*. → §3.6.
  - **N4** — small-`d` degenerate spoke is valid; don't guard against it. → §4.1.
  - **N5** — the original washer-vs-sprue failure is impossible by construction (pour seed +
    washer-clears-bore feasibility). → header + §3.4.
  - *Structural:* the cold-read corrections had accreted as appendices contradicting the body;
    this consolidation rewrote the body to the converged design and compressed the passes into
    this log.
