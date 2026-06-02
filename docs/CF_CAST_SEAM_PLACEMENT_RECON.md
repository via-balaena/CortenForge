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
| outward normal | `SeamProfile::outward_normal_at(s) -> Point2` (the old `Silhouette2d::outward_normal_at_arc_fraction` was removed on this branch) | `seam_profile.rs:~204` |
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
| **S5** promote + delete | Flip `smart_placement` default **on**; delete the legacy *placement* machinery — uniform bolt/dowel loops, the three pour paths, the `flank_bolts`/`bracket_pour_gate`/`skip_pour_gate_collision` knobs + the bolt↔dowel arc-stagger validator; fold the duplicate per-layer silhouette rebuilds (§MA-7 reuse — the byte-identical SOLVE-dedup half, S5d-(A); the re-baselining COMPOSE-share half S5d-(B) defers to S4.5); re-baseline iter-1 byte-identity tests deliberately. **Keep the uniform `Plate` flange as the default** — the flange redesign is S4.5, so S5 stays a clean "promote the proven solver, delete legacy placement" cut with its own scoped re-baseline. **Full sub-step plan + decisions in §7.5.** | `grade-all` green; byte-identity re-baseline reviewed; no legacy *placement* path remains. |
| **S4.5** demand flange | §4: add `FlangeKind::Demand` (seal-ring + per-fastener tadpoles); flip the derive order (flange generated *after* placement, clearance-only feasibility). Lands as a sibling of `Plate` (own re-baseline) then **becomes the default for `base_mold`** — this is the print target. *Incremental path only; on the direct path this folds into S3.* **Also fold in S5d-(B)** here (unify the solve-pad and flange-pad so a single silhouette per layer serves both the solver and the flange build — the §MA-7 compose-share that can't be byte-identical standalone; it rides this re-baseline for free). | `base_mold` regen: mass/print-time ↓ vs plate; F4 clean; seal-ring continuity test passes; cf-view scallop + apex boss look right. |
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

## 7.5 S5 PLAN (2026-06-01, agreed + cold-read-reviewed; pre-implementation)

**Two decisions, high conviction:**
- **(D1) Solver-only; the `smart_placement` field is removed.** It is migration
  scaffolding (born S3 to keep legacy byte-identical for the A/B), not a product
  feature. Once legacy is deleted there is no second behaviour to gate, so a retained
  flag would be a no-op knob that lies. Placement instead happens by **feature
  presence** — iff a bolt/dowel pattern *and* a flange exist — exactly like every
  sibling feature (`FlangeKind::Plate`, `BoltPatternKind::Auto`). `compose_piece_solid`
  (single-shot; only reached by tests + the public re-export — **v1 `export_molds` does
  NOT use it**, it carves the cup directly) runs a **single-layer solve**
  (`plan_smart_*(&[body], &[bounds], …)` = the cross-layer snap with a 1-element stack).
- **(D2) Full depth — the dead `count`/`silhouette_outboard_offset_m` knobs and the
  offset-based validators die.** Under the solver `count` is emergent and `d` is solved,
  so the knobs do nothing; the offset-based wall validators are not merely inert but
  **superseded** — the solver's `d_floor = wall + washer + margin` + band feasibility
  *guarantee by construction* what those validators *checked* (the §4.2 dependency flip
  in miniature). Keep the non-offset "bolt requires flange" check. **Accepted trade:** a
  too-narrow flange no longer fast-fails at config-parse; it surfaces as the solver's
  `cross_layer_snap` "dropped N" warn (Risk #1).

**Four commits (S5a behaviour, S5b/S5c deletion, S5d-(A) solve-dedup perf — all but S5a
byte-identical).** The **re-baseline invariant (Risk #7):** the *only* commit that changes
geometry is **S5a**. **S5b/S5c/S5d-(A) must regen `base_mold` byte-identical to S5a**
(release, 3 mm — placement is cell-independent) — S5b/S5c remove unreachable/dead code, S5d-(A)
reuses an identical silhouette instead of rebuilding it. A placement change and a
deletion/refactor never share a commit. (S5d-(B), the only silhouette fold that *can't* be
byte-identical, is deferred to S4.5 — see the S5d bullet below.) Commit boundaries are compile-clean (verified): the
`count`/`offset` spec fields are shared by `procedure.rs` + the CLI validators, so they
survive S5b and die with their consumers in S5c.

- **S5a — flip the default ON** (behaviour promotion only). `ribbon.rs` `Default`
  `smart_placement: false→true`; CLI `[cast].smart_placement` default `false→true`
  (field retained this commit). **In-tree re-baseline surface is ~empty** (no in-tree
  geometry test enables bolt+flange through a ribbon — `spec.rs:4062` is procedure-prose,
  never runs the solver; no CLI integration test enables them; legacy unit tests call the
  builders directly; smart tests set the flag explicitly). The real baseline is the
  **out-of-tree production regen — capture it BEFORE S5b** (the production TOML still
  parses pre-deletion).
- **S5b — delete legacy placement paths + the gate + pour-handling** (≈ the "Scoped" end
  state; compiles green). `bolt_pattern.rs`: drop the legacy arm of
  `build_bolt_pattern_transforms` (→ takes `&[Point2]`, folds `emit_smart_bolts`); delete
  `collides_with_pour_gate` / `find_pour_arc_fraction` / `place_pour_flanking_bolts` /
  `bolt_center_at` / `too_close_to_existing`; **move** `point_to_cylinder_axis_distance` /
  `pour_axis_distance` / `max_pour_radius` into the test module (now smart-test-only);
  drop `BoltPatternSpec.{skip_pour_gate_collision, pour_gate_clearance_m,
  bracket_pour_gate}`. `dowel_hole.rs`: drop the legacy arm (→ `&[Point2]`, folds
  `emit_smart_dowels`). `piece.rs`: `compose_piece_solid` → single-layer solve;
  `compose_piece_shared` smart params `Option<&[Point2]>`→`&[Point2]`. `spec.rs`:
  `mesh_and_gate_v2_pieces` gate `if ribbon.smart_placement && flange`→`if flange`.
  `ribbon.rs`: delete the `smart_placement` field + `with_smart_placement`.
  `procedure.rs`: delete the `if spec.skip_pour_gate_collision` block **and correct the
  §1648 prose** ("the §B bolt pattern brackets this bore" is now *factually wrong* — the
  solver brackets via the channel-exclusion interval edges, not §B). CLI `config.rs`:
  delete `[cast].smart_placement`, `PourGateConfig.flank_bolts`,
  `BoltPatternConfig.skip_pour_gate_collision`, the **arc-stagger validator**. CLI
  `derive.rs`: delete the `flank_bolts→bracket_pour_gate` block + `with_smart_placement`
  call + skip plumbing. **Checkpoint:** update the out-of-tree production TOML (drop
  `flank_bolts` — `deny_unknown_fields` rejects it post-deletion); full `grade-all`;
  regen byte-identical to the S5a baseline.
- **S5c — delete the dead `count`/`offset` knobs + offset-based validators** (completes
  "Full"). Drop `count` + `silhouette_outboard_offset_m` (+ their `DEFAULT_*` consts) from
  `BoltPatternSpec`/`DowelHoleSpec`; rewrite the `procedure.rs` dowel (§1067/1071) + bolt
  (§1159/1161) sections to **count-agnostic, solver-aware prose** (keeping a fixed
  `count` would make the procedure *lie* about the emergent per-layer count); delete CLI
  `{Bolt,Dowel}*Config.{count, silhouette_outboard_offset_m}` + the offset-based wall
  validators (dowel inboard/outboard, bolt inboard/outboard, washer-cup-wall-step) +
  derive plumbing; re-point/delete the config-validation unit tests that fed those
  validators (e.g. the `offset = 0.009` fixtures). Full `grade-all`; regen
  byte-identical to S5a/S5b; a generated `procedure.md` reads correctly.
- **S5d — fold the duplicate per-layer silhouette rebuilds (§MA-7).** DECISION (2026-06-01,
  after reading the construction sites): §MA-7 is **two opportunities with opposite risk**, so
  S5d is **split**, not "optional/maybe-drop":
  - **S5d-(A) — dedup the SOLVE-phase rebuild → DO NOW (this branch), byte-identical.** On a
    smart run both `plan_smart_dowel_placements` and `plan_smart_bolt_placements` call
    `seam_silhouette(body, ribbon, bounds, pad)` with the **identical** `pad = flange_width +
    SILHOUETTE_GRID_STEP_M` — the bolt planner rebuilds, bit-for-bit, the silhouette the dowel
    planner already built (3 redundant minutes-each extractions on `base_mold`; the grid is
    0.5 mm-fixed so this cost is cell-independent). Fix = build each layer's loop
    (`SeamProfile` + pour exclusions) **once** in `mesh_and_gate_v2_pieces` (a `build_layer_loops`
    helper in `seam_placement.rs`) and pass `&[LayerLoop]` to both planners; the bolt planner
    clones each layer's pour-exclusions and appends the dowel disks. **Byte-identical by
    construction** (same inputs already produce the same `SeamProfile`; reuse just skips the
    recompute) — it touches ONLY the solver orchestration, NOT the flange/compose path. Solve
    extractions 6→3 on `base_mold` (total 9→6); ~halves the solve-phase silhouette wall-time,
    which is exactly what the S4.5 placement probes hammer (they exit before MC). Own commit;
    verify the 6 cup hashes vs the S5a/b/c baseline + `grade-all`.
  - **S5d-(B) — share the SOLVE silhouette with the COMPOSE/flange silhouette → DEFER to S4.5.**
    `build_flange_solid` samples its silhouette with `pad = flange_width` (**no
    `+ SILHOUETTE_GRID_STEP_M`**), so the compose silhouette is NOT bit-identical to the solve
    one — unifying the pads to share a single extraction changes either the placements or the
    flange window → a cup re-baseline. That re-baseline is **free inside S4.5** (which rewrites +
    re-baselines the flange/compose path for the demand flange anyway) and would be a
    speed-only re-baseline if done standalone (forbidden by the Risk #7 hygiene). So (B) is a
    fold-in for S4.5, NOT part of S5. Byte-identical ceiling for S5 is therefore 9→6, achieved
    by (A).

**Behaviour note (benign, document in S5b):** legacy emitted dowels even without a flange
(the validator only runs `if dowel_hole.enabled && flange.enabled`); the solver requires a
flange. But a flangeless dowel lands beyond the 5 mm cup wall → its `SubtractCylinder`
carves empty space → **mesh-identical** (N no-op transforms → 0). **Blast radius
confirmed contained:** no consumer of the deleted fields exists outside `cf-cast` +
`cf-cast-cli` (workspace grep).

---

## 7.6 S4.5 PLAN (2026-06-01, agreed + decisions pinned; pre-implementation)

The demand-driven flange (§4) lands as a sibling of `Plate`, then becomes the
`base_mold` default = the print target. **Key architectural finding:** placement is
*already* decoupled from the flange *solid* — the planners read only `flange_spec`'s
`[inner_offset, width]` numbers as the feasibility band, and the solve already runs
before `build_flange_solid`. So the §4.2 "dependency flip" is a **feasibility-regime
swap + a flange-SDF swap**, NOT a control-flow rewrite.

**Two decisions pinned (user, 2026-06-01):**
- **(E1) Radial offset under `Demand` = pin at the inboard floor.** Collapse the radial
  DOF: `Feasibility::band(inner = d_floor_sd, width = d_floor_sd + footprint, d_floor,
  d_max = d_floor)` so every fastener hugs the seal ring at the washer-vs-cup-wall floor
  (§3.2). Minimal moment/material, shortest tadpole spokes. The apex is still bracketed by
  stepping ALONG the arc (the pour `Channel` exclusion + `PourPierce` seed), never by
  pushing `d` outboard — the demand flange makes pushing-out unnecessary (material follows
  the fastener). Supersedes the plate-era variable-`d` (the S0 11.5/14.5 mm apex asymmetry
  was a *band* artifact).
- **(E2) Seal land hugs the cavity (gasket-None).** `land_inner_offset_m ≈ 0.5 mm`
  (a tiny start for 0.5 mm-grid MC-quantization safety, not the Plate 2 mm gasket-clearance
  offset, recon §4.1 N2), `land_width_m ≈ 6 mm`. Maximizes PLA-on-PLA seal contact.

**New types (`flange.rs`):** `DemandFlangeSpec { land_inner_offset_m, land_width_m,
flange_thickness_m, web_width_m, boss_wall_margin_m }` + `FlangeKind::Demand(..)`. Add
`FlangeKind::lateral_reach_m() -> Option<f64>` (Demand: `land_inner + land_width +
boss_allowance`; Plate: `flange_width_m`) so `layer_mc_bounds` + `build_layer_loops`'s
`pad` are correct for both kinds; existing `.spec()` presence-callers unaffected.

**`DemandFlangeSdf` (seal ring + tadpoles + slab):**
```
demand_flange_sdf(P) = max(
    min( seal_ring_band(P),                       // continuous closed land (sd ∈ [land_inner, land_inner+land_width])
         min_i  capsule(P; anchor_i → center_i,   // per-fastener tadpole, r: web/2 → boss_r_i
                        boss_r = footprint + boss_wall_margin) ),
    binormal_slab(P) )                            // |dist_to_seam| − thickness  (identical to FlangeSdf)
```
- Seal ring reads the **shared `SeamProfile` signed distance** (NOT a freshly-built
  `Silhouette2d`) → completes §MA-7 **9→3** for the Demand path (S5d-(B) *dissolves* here,
  no separate commit; **Plate's `build_flange_solid` is left byte-identical**, untouched).
- **HARD GATE — `[[project-cf-cast-flat-mating-face-constraint]]`:** union/intersect are
  plain `min`/`max` ONLY — no `smooth_union`/fillet/gusset. Thickness term is the identical
  binormal slab Plate uses, so the mating face stays the §F-4 bit-precise planar cut. Run
  the `mating_face_is_mathematically_flat_and_coplanar` gate on a Demand fixture.

**Threading:** the planner returns `PlacedFastener { center, anchor, boss_r }` per fastener
(anchor = `center` pulled inboard along `profile.outward_normal_at` to the seal-ring outer
edge; the hole-carve still uses only `center`). `mesh_and_gate_v2_pieces` keeps `layer_loops`
alive and passes the per-layer `SeamProfile` into `compose_piece_shared` so the seal ring
reuses it.

**Commit sequencing (Risk #7 — attributable, never conflated). ALL COMMITTED on the
branch (not pushed); commit 2 was split 2a/2b:**
1. **dowel.stl solver-count** (fold-in 2) — **DONE `9caf8db2`**. `mesh_and_gate_v2_pieces`
   returns the shared placed-dowel count; `export_molds` threads it to
   `mesh_and_gate_v2_dowel`, replacing `const PRINTABLE_DOWEL_COUNT=4` →
   `build_dowel_array_mesh(.., count, ..)`. **Only `dowel.stl` changes** (4→2 rods on
   base_mold); the 6 cup pieces stay byte-identical to the (then-current) S5 baseline.
2a. **Flange-kind seam** — **DONE `42166ba1`, byte-identical.** `FlangeKind::lateral_reach_m()`
   replaces the Plate-only `spec()` as the placement gate; `build_layer_loops` + both
   planners + their feasibility builders take `&FlangeKind` (`*_feasibility`→`Option`);
   `compute_smart_placements` helper extracted. Only Plate/None arms — Plate unchanged.
   (Needed because `Demand.spec()` is `None`, which would have silently disabled placement.)
2b. **`FlangeKind::Demand` (dormant sibling)** — **DONE `2e3487ce`.** `DemandFlangeSpec` +
   `DemandFlangeSdf` (`max(min(seal_ring, min tadpoles), slab)`, plain min/max → §F-4 flat
   face; seal ring from the SHARED `SeamProfile` = the §MA-7 9→3 / S5d-(B) completion for the
   demand path, Plate `build_flange_solid` untouched) + `sd_tapered_capsule_2d` (iq round
   cone; `a2≤0`→larger disk = the pin-at-floor degenerate spoke, N4) + (E1) pin-at-floor
   feasibility arm + `build_tadpoles` + `compute_smart_placements` pre-builds the per-layer
   demand flange `Solid` → `SmartPlacements` → `compose_piece_shared(demand_flange)` + bolt
   emission via `thickness_m()` + CLI `[flange] kind` (default still plate this commit →
   cups byte-identical) + procedure `(Demand,*)` prose + 8 tests + the `derive.rs:88`
   doc-link fix. Demand A/B F4-passes all 6 cups, ~13 % lighter (scalloped).
3. **Flip the default to Demand** — **DONE.** CLI `[flange].kind` default → `"demand"`
   (derive: absent/`"demand"`→Demand, `"plate"`→legacy band) — the deliberate cup
   re-baseline. **Verified (3 mm release): the default flange now differs from the retiring
   Plate S5 baseline (flip took); absent-`kind` == explicit `"demand"` (byte-identical);
   fresh demand baseline at `/tmp/s45_demand_baseline_hashes.txt`.** A cup diff here is
   *seal-geometry*, not placement (§7.4 A/B de-risked placement). Remaining manual gate =
   cf-view eyeball of the scallop + apex boss, then the physical print.

Each commit: cf-cast lib + cf-cast-cli green; clippy pedantic+nursery + fmt; `RUSTDOCFLAGS=
"-D warnings"` doc-link check (the `pub`→`pub(crate)` backtick lint); `grade-all
--skip-coverage` 295/295; real-body probes in **release**; cold-read pre-commit.

---

## 7.7 PER-LAYER POUR-GATE SIZING PLAN (2026-06-02, agreed + decisions pinned; pre-implementation)

**The gap (workshop-spotted).** A 3-layer cast pours 3 *different* silicones (00-30 ~3k cps,
Dragon Skin 20A ~20k+ cps, 00-30), but all three cup gates are the **same Ø10 mm** and the cast
emits **one** funnel. The pipeline *knows* the per-layer recipe (`CastLayer.material:
MoldingMaterial` — drives cure protocol + procedure names), but the pour gate is **cast-global
and material-blind**: `ribbon.pour_gate` is one `PourGateSpec`; three consumers all read its single
`gate_radius_m` — `build_pour_gate_transforms(ribbon)` (→ the per-piece bore, `piece.rs:533`),
`build_funnel_solid(ribbon)` (→ one funnel, `funnel.rs:572/232`), and `build_layer_loops`
(→ the fastener gate-exclusion, built once for all layers, `seam_placement.rs:128`). Not a data
disconnect — an **unbuilt connection**. The risk it leaves: ~20k-cps Dragon Skin pours through a
throat sized for ~3k-cps 00-30.

**Three decisions pinned (user, 2026-06-02):**
- **Size home = `MoldingMaterial`.** Add `pour_gate_radius_m: Option<f64>`. `None` → cast default
  (`ribbon.pour_gate` spec `gate_radius_m`, still set by the global CLI knob). The recipe carries
  its own throat → "the silicone determines the sprue"; two layers sharing a recipe share the gate.
  A **calibratable number, not a viscosity formula** — the viscosity *model* stays deferred
  ([[project-cf-cast-per-silicone-sprue-funnel]]); the print tunes the number.
- **Scope = mechanism (byte-identical) + a Dragon-Skin bump (deliberate) so Print 1 tests it.**
- **Funnels named by recipe** — `funnel_ecoflex_00_30.stl`, `funnel_dragon_skin_20a.stl` (sanitized
  `display_name`), matching the per-layer procedure sections.

**Threading (one new param, default-`None` = byte-identical):**
- `build_pour_gate_transforms(ribbon, gate_radius_override: Option<f64>)` — `None` → `spec.gate_radius_m`
  (today); `Some(r)` → `r` for the **pour leg only** (vent stays `spec.vent_radius_m`). Position
  unchanged (apex-on-seam); only the bore radius varies. Ripple sites: `piece.rs:533`,
  `seam_placement.rs:128`, `funnel.rs:572`, + bolt_pattern/pour tests (pass `None`).
- **compose** (`piece.rs:533`) → the layer's resolved radius (`material.pour_gate_radius_m`).
- **placement** (`seam_placement.rs:128`) → the **MAX** radius across all layers. Fasteners are
  shared across layers (cross-layer snap, §3.8), so they must clear the *biggest* gate to be valid
  on every layer. All-equal (today) → max = default → identical placement.
- **funnel** → emit **one per distinct radius**, named by recipe. One distinct radius (today) →
  one funnel, byte-identical; two → a second `funnel_*.stl`. Throat sizing per funnel
  (`gate_radius − NIPPLE_DIAMETRAL_CLEARANCE_M/2`) unchanged.
- **procedure.md** — each per-layer section names its funnel + gate Ø; the "📁 Files" banner notes
  the multiple-funnel case.
- **CLI** — `pour_gate_radius_m` on the per-recipe material config; `derive.rs` sets it on the
  `MoldingMaterial`. Validate `> 0`; bound it (a gate un-bracketable by fasteners on some layer →
  error naming the layer).

**Characteristic to evaluate at the print (not a blocker).** The apex bore already breaks the demand
flange's seal-ring at the apex (it's on the seam, §4.3); a wider Dragon-Skin gate breaks it wider.
Existing behavior, just more of it — Print 1 evaluates whether that local gap seals under hand-clamp.

**Commit sequencing (Risk #7 — attributable, never conflated):**
1. **Mechanism — byte-identical.** Field (default `None`) + all threading + per-recipe funnel
   emission. Every material defaults → all gates = cast default → one distinct radius → one funnel,
   max = default placement. **Gate: regen byte-identical** to current canal_05 (proves it's pure
   wiring). Tests: override-`None`-equals-default, per-recipe funnel set from distinct radii,
   placement-uses-max.
2. **Dragon-Skin resize — deliberate re-baseline.** Set base_mold's Dragon Skin layer to **8 mm
   radius = Ø16 → ~Ø15.5 throat** (vs the 5 mm/Ø10 default). **Sizing math (2026-06-02, real
   numbers):** the DS layer is the worst case on every axis — ~20k cps (6.7× the 00-30 layers),
   220 g / ~206 mL (the largest pour), 25-min pot life (the shortest). Gravity Poiseuille
   (`Q = π·r⁴·ΔP/(8·μ·L)`, L≈35 mm throat, ΔP≈470 Pa) gives throat-only fill times: **Ø10 ≈ 21 min
   (no margin vs the 25-min pot life — risks gelling mid-pour); Ø14 ≈ 5.4 min; Ø16 ≈ 3.2 min.**
   `r ∝ μ^¼` ⇒ matching the proven 00-30 baseline flow (~1.1 mL/s, fills in ~1-2 min) needs
   `5·6.67^0.25 = 8 mm → Ø16` — the equal-flow choice (user, 2026-06-02). **Adopted sizing
   criterion (user, 2026-06-02): every layer's gate clears `Q ≥ 1.0 mL/s` at nominal head**
   (≈ the proven-comfortable 00-30 rate). Per-material min radius `r = [Q*·8μL/(π·ΔP)]^¼`: 00-30
   (3 Pa·s) → 4.88 mm (Ø10 default clears it, 1.10 mL/s); 00-30+Slacker (thinner) → smaller (Ø10
   clears); Dragon Skin (20 Pa·s) → 7.85 mm → **Ø16 (1.08 mL/s, just over the floor)**. So the floor
   keeps the 00-30 layers at the default Ø10 and lifts only DS to Ø16 — the same split, principled.
   The mL/s floor is a design *heuristic with margin* (absolute rate depends on head + throat-length
   estimates, both uncalibrated) — not yet an in-tool auto-sizer; per-recipe radii stay hand-set via
   the mechanism, and a viscosity→radius auto-sizer (needs viscosity in `MoldingMaterial` + the head/L
   assumptions pinned) is a **post-print follow-up** once a bench pour validates the model.

   **Reference map — all 8 silicone-table anchors (Smooth-On TDS mixed viscosity, 2026-06-02).**
   *Pour viscosity is NOT in the repo* — the `sim/L0/soft` silicone table carries cured-elastomer
   mechanics (shear modulus, Shore, Yeoh, density), not liquid rheology — so these come from TDS and
   the gate radius is sized externally. **Viscosity ≠ Shore hardness**: Ecoflex 00-10 (the *softest*)
   is 14k cps, thicker than the firmer 00-50 (8k). Gate Ø to clear `Q ≥ 1.0 mL/s` (`r ∝ μ^¼`,
   ref 00-30 @ 4.88 mm):

   | Anchor | μ (cps) | r_min | gate Ø | Anchor | μ (cps) | r_min | gate Ø |
   |---|---:|---:|---|---|---:|---:|---|
   | Ecoflex 00-20 | 3,000 | 4.88 | Ø10 | Dragon Skin 20A | 20,000 | 7.85 | Ø16 |
   | Ecoflex 00-30 | 3,000 | 4.88 | Ø10 | Dragon Skin 30A | 20,000 | 7.85 | Ø16 |
   | Ecoflex 00-50 | 8,000 | 6.24 | Ø13 | Dragon Skin 15 | 21,000 | 7.94 | Ø16 |
   | Ecoflex 00-10 | 14,000 | 7.18 | Ø15 | Dragon Skin 10A | 23,000 | 8.13 | Ø17 |

   Only the thin Ecoflex 00-20/00-30 ride the default Ø10; the rest need a bump (DS family clusters
   Ø16-17). Slacker only thins → a blend stays ≤ its base anchor's Ø. For base_mold, the lone
   deviation is DS 20A → Ø16. (When the auto-sizer is built post-print, this table is its seed data —
   add `pour_viscosity_cps` to the anchors.) Counter-pressure weighed:
   the apex bore breaks the demand seal-ring land wider at Ø16 than Ø10 (recon §4.3), but the apex
   is the pour entry + highest point + bolt-bracketed, so it's the least-bad place to widen; flow
   safety on a 220 g thick pour wins. Regen → that layer's bore widens + a second
   `funnel_dragon_skin_20a.stl` appears; **verify the seam-placement solver still brackets the Ø16
   bore** (placement uses the max radius across layers; the un-bracketable bound-validator fires if
   not) → reviewed diff + grade-all, on its own commit.

Each commit: cf-cast lib + cf-cast-cli green; clippy pedantic+nursery + fmt; `RUSTDOCFLAGS="-D warnings"`
doc-link check; `grade-all --skip-coverage` 295/295; real-body regen-diff in **release**; cold-read
pre-commit. Lands on a fresh branch off the current cleanup batch (or after it merges).

> **⚠ SUPERSEDED in part by §7.8 (2026-06-02).** §7.7's per-layer *gate sizing* stands, but its
> **separate per-recipe `funnel.stl` emission is replaced** by the §7.8 *integral split funnel*
> (the funnel becomes part of the cup, so there is no separate funnel artifact for apex-axial and
> no funnel-naming). And the flow model is corrected: §7.7's mL/s figures were computed at the gate
> radius, but with the *current separate funnel* the binding throat is the nipple lumen (gate Ø −
> 3.5 mm) — §7.8 removes the nipple, restoring the bore as the throat (and the §7.7-original
> gate-radius sizing table).

---

## 7.8 INTEGRAL SPLIT FUNNEL DESIGN (2026-06-02, agreed direction; pre-implementation)

**Genesis.** Mapping §7.7's per-silicone gates surfaced that the *current* separate funnel's nipple
**inserts into** the bore, and its 1.5 mm wall shrinks the lumen to `gate Ø − 3.5 mm` (Ø6.5 at the
Ø10 default) — a **~5.6× `r⁴` throat penalty** that the thick layers can't overcome without huge
gates. User's call: stop fighting it — **make the bore itself the funnel**: an *integral* funnel that
is part of the cup print, **split down the seam** like the bore/dowels/socket, rising straight out of
the apex with a **lumen continuous into the bore** (no inserted nipple, no step-down, no constriction).
NOT grooved into the flange (the demand flange is scalloped to a seal-ring + bosses — no plate to
groove); its own protruding structure. This **supersedes the separate `funnel.stl`** (§7.7 fold).

**Architecture — SDF-integral union (NOT post-MC).** The funnel is built into the cup-body `Solid`
in `compose_piece_with_shared`, so the existing **seam halfspace intersect bisects it for free**
(same mechanism that already halves the shell + flange pre-MC), and MC meshes it natively.
- *Why not post-MC* (the established mating-feature pattern): a post-MC **union** adds a *full* cone
  to *each* half → doubles. A seam-clipped additive primitive (like `UnionFloorSlab`) would work but
  needs a new truncated-cone primitive + seam-clip payload. SDF avoids both, and the funnel is
  *already* an SDF `Solid` (`funnel.rs::build_funnel_solid` / `truncated_cone`).
- *Why this is NOT the abandoned-SDF mistake* ([[feedback-read-prior-arc-memory-before-architectural-decisions]]):
  that was a **1.5 mm pin eaten by 3 mm-cell MC quantization**. The funnel is **cm-scale bulk
  material with ~2.5 mm walls at 0.5 mm production cells** — orders of magnitude above the
  quantization floor. Fallback if MC faceting of the lumen proves rough at the chosen cell: a
  post-MC exact truncated-cone primitive (`Union/SubtractTruncatedCone`). Verified at the production
  cell in the regen (F4 + cf-view eyeball).

**Geometry (coaxial with the apex-axial bore; axis lies IN the seam plane, so the seam bisects it).**
Cone widening from the bore at the cup surface up to the pour mouth (outboard tip = +Z top in pour
orientation):
- **base outer Ø** = bore Ø + 2·wall; **wall** ≈ 2.5 mm (robust printable half-shell).
- **mouth inner Ø** = **2.2× bore Ø** (pinned) — Ø10 bore → ~Ø22 mouth, Ø16 bore → ~Ø35.
- **height** = **18 mm** (pinned) — *modest by design*: short + cone-stiff so the bolt-bracketed
  base clamp dominates. Catch volume ~1.5 mL (an *aim* funnel; the workshop ladles continuously —
  a true reservoir, if wanted, is an un-registered cup held above, no leak path).
- **lumen** = the full pour channel: funnel taper (mouth inner → bore Ø) then straight bore (bore Ø
  through flange/wall into the cavity). The **throat is the bore Ø** (narrowest); the funnel only
  ever widens above it.

**Lumen / bore (apex-axial moves fully into SDF).** Build the channel as one negative `Solid`
(taper + bore cylinder, overlapped to weld) subtracted from the cup body in SDF, **retiring the
post-MC `SubtractCylinder` pour leg for `ApexAxial`** (`build_apex_axial_transforms` → SDF). The
seam intersect then bisects the bore into the two half-troughs exactly as today. V-at-dome keeps its
post-MC bore + separate funnel (legacy).

**Funnel vs. sprue (terminology, since it drives the next point).** The **funnel** is the PLA
structure — part of the *mold* (integral, prints with each cup). The **sprue** is the *cured
silicone* that fills the funnel + bore channel — the *waste* trimmed off the cast part at demold.
The funnel shapes the sprue. So the funnel is **sacrificial sprue-channel territory**: not the cast
product, NOT a seal surface (the seal is the inboard demand-flange land). That's why it can be crude.

**Seam-split + clamp/leak — NO INTERLOCK (corrected 2026-06-02, supersedes the pinned interlock).**
The funnel is **two flat half-cones continuing the cup's flat seam**, clamped by the same
bore-bracketing bolts — exactly like the rest of the mold. The earlier "ship a self-mating interlock
in commit 1" pin was built on an overstated splay risk: the cold-read found (i) the interlock was
geometrically incoherent (a planar cut yields *congruent* faces, not complementary tongue/groove —
a real interlock would have to be side-DEPENDENT), and (ii) the splay force is negligible anyway —
the funnel is the mold's *highest* point, so head there is a few cm → outward force ≈ `ΔP·area ≈
525 Pa · ~2 cm² ≈ 0.1 N` (~10 gf) on a short cone-stiff PLA shell rigidly tied to the bolt-clamped
base. It stays shut for the same reason the cup halves do. Stiffeners that DO ship: cone taper +
18 mm height keep it stiff. Any faint seam witness line lands on the *waste sprue* → trimmed away.
**Print 1 evaluates the residual weep**; only if it actually weeps do we add an interlock — and then
it must be **side-dependent** (tongue on one piece, groove on the other; seal-safe because the
funnel seam is sacrificial, not the flat seal land) or a side-agnostic cross-pin (loose part).

**Printability.** Cups print seam-down → the funnel axis is parallel to the bed and each half is a
**half-cone shell lying flat-(seam-)face-down** — zero overhang on the flat face; the curved shell
arcs from the bed up to a spine (mild self-supporting overhang near the spine, as any half-pipe).
Protrudes into bed *footprint*, not Z-height. Verify the spine at the chosen wall/Ø in slicer.

**Sprue.** Funnel cone + bore cure as one solid; split with the halves at demold → lifts straight
out (no blind pull); trim flush at the cup surface. Same as today's bore sprue, plus the cone.

**Per-layer bore sizing — corrected (no nipple → bore IS the throat).** Removing the nipple restores
the clean `r ∝ μ^¼` at the bore (no −1.75 mm offset), i.e. the §7.7-**original** gate-radius table:
00-20/30 → Ø10; 00-50 → Ø13; 00-10 → Ø15; **DS 20A/30A → Ø16**; DS 15 → Ø16; DS 10A → Ø17.
So **DS 20A = Ø16** (the user's pick) is again the 1.0 mL/s / equal-flow number. *(Conservative:
modeled throat-length ≈ current; a shorter effective throat without the 20 mm nipple would only
improve flow → these gates are an upper bound. Print calibrates.)*

**Impl constraints (from the 2026-06-02 cold-read / stress-test).**
1. **`layer_mc_bounds` must enclose the funnel** — it protrudes ~`height`+cone outboard along the
   bore axis at the apex, *past* today's `max(wall, flange_reach)+pad` box → expand or MC clips the
   funnel. **Cost: ~10–15% more cup MC cells** (mostly empty space around the cone) → ~+7 min on a
   ~56 min canal_05 regen + more memory. Standing tax until narrow-band MC (deferred) lands.
2. **Lumen must breach the cavity** — the SDF lumen has to reach inboard exactly as today's 90 mm
   `SubtractCylinder` bore does, or the bore is a *blind pocket* (silicone can't enter the cavity).
3. **Funnel base must overlap-weld into the cup shell** — the cone base meets the *curved* dome
   surface; needs a `BOWL_NIPPLE_OVERLAP`-style inboard overlap or it floats → non-manifold.
4. **Bore→SDF facets the bore edge on the mating face** (~0.5 mm at production cell) vs today's
   post-MC exact cylinder. ACCEPTED (apex/sprue, not the seal land); the alternative (keep bore
   post-MC, only funnel SDF) has a messier junction.
5. **Degenerate apex axis inherited** — if the dome outward axis can't lie in the seam plane
   (`build_apex_axial_transforms` `norm<1e-9` fallback), the funnel can't be cleanly bisected
   (goes lopsided). Pathological + already gated for the bore; the funnel amplifies the visible
   consequence. No new mitigation; the 2-piece gate already rejects this split.
6. **Placement bracket vs funnel base** — **AS-BUILT: widened UNCONDITIONALLY.** The original
   scope was "verify-first, maybe a no-op (only widen if the washer overlaps the funnel wall),"
   but the apex pour-exclusion is now *always* the funnel base + comfort
   (`integral_funnel_exclusion_radius = bore_r + INTEGRAL_FUNNEL_WALL_M(2.5 mm) +
   INTEGRAL_FUNNEL_FASTENER_CLEARANCE_M(2 mm)`, `pour.rs`). The conservative always-widen is
   simpler and guarantees the apex bracket bolts clear the funnel cone on every body, with no
   per-body overlap check — chosen over the conditional approach.
7. **F4 may flag the funnel spine** (half-pipe top → near-horizontal) — check the cup F4 overhang
   policy on a fixture before the full regen.
8. **Broad surface area** — V-at-dome KEEPS the separate funnel, so export / procedure /
   progress-counter / v1 + tests all branch on `layout` (apex-axial: no separate `funnel.stl`).

**Code fold-in.**
- `pour.rs` — new `build_integral_funnel_channel(ribbon, bore_r) -> (outer_cone: Solid, lumen: Solid)`
  (reuse `funnel.rs::truncated_cone`); `build_pour_gate_transforms` returns empty for `ApexAxial`
  (bore now SDF).
- `piece.rs::compose_piece_with_shared` — after `shell ∩ halfspace ∪ flange`:
  `base = (base ∪ (outer_cone ∩ halfspace)) ∖ lumen` (lumen subtract carves both the funnel hole and
  the bore; side-safe). Threads the per-layer bore radius (§7.7 mechanism).
- `funnel.rs` — `build_funnel_solid` / `mesh_and_gate_v2_funnel` no longer emit for `ApexAxial`
  (separate `funnel.stl` gone for the default); `truncated_cone` reused by the integral builder.
- `spec.rs` — drop the funnel artifact (+ its progress tick) for `ApexAxial`; per-layer bore radius
  flows from `MoldingMaterial.pour_gate_radius_m` (§7.7) into the integral builder.
- `procedure.md` — replace "separate funnel, reused across layers" prose with the integral split
  funnel (pour into the apex funnel; trim the funnel+bore sprue).

**Commit sequencing (Risk #7).** This is a *deliberate geometry change* throughout (the apex pour
re-architects), so byte-identity is NOT the gate — reviewed regen-diff + cf-view eyeball + F4 are.
1. **Integral funnel mechanism** — SDF funnel+bore for `ApexAxial` (two flat half-cones, NO
   interlock), bounds + placement-bracket updates, retire the separate funnel for apex-axial.
   Default bore Ø10 (all layers) → one clean regen-diff vs the separate-funnel baseline (the bore
   region + the new integral funnel; no separate `funnel.stl`). cf-view: funnel splits cleanly,
   flat half-cones, sprue path obvious.
2. **Per-layer bore sizing live** (§7.7 mechanism on the integral throat) — DS 20A → Ø16; regen-diff
   shows only the DS layer's funnel+bore widen; verify the bracket still clears the Ø16 funnel base.

Each: cf-cast lib + cli green; clippy+fmt+rustdoc; grade-all 295/295; release regen-diff + cf-view;
cold-read pre-commit. Same branch as §7.7 (the per-silicone-pour-gate arc).

**Decisions pinned (user, 2026-06-02):** (1) funnel **mouth = 2.2× bore Ø, height = 18 mm**
(balanced catch/stiffness); (2) **NO interlock — two flat half-cones** continuing the cup seam,
clamped like the rest of the mold (splay force ≈ 0.1 N, negligible; the funnel is sacrificial sprue
territory). Print 1 evaluates weep; add a side-dependent interlock later only if it actually weeps.
(3) **SDF-integral** approach (post-MC exact cone is the faceting-fallback only).

---

## 7.9 SEAM-PLACEMENT VISUAL CLEANUP (2026-06-02, from the §7.8 cf-view eyeball)

Reviewing the integral-funnel regen surfaced three placement artifacts in the (pre-existing,
PR #262) seam solver, all about the demand-flange BOSSES (radius `footprint + boss_wall_margin`,
~7 mm for bolts) being bigger than the WASHER footprint (5 mm) the spacing was sized to. Fixed as
one cleanup commit (each verified by a throwaway fastener-position probe, then reverted):

1. **Coincident bolts on the inner loop.** `cross_layer_snap` projects the outer-loop master
   pattern onto each layer; a SMALLER inner loop compresses two distinct positions onto ~one point.
   It never re-deduped → layer-0 carried a 0.04 mm duplicate (two bolts in one hole). **Fix:**
   re-dedup each layer's snapped positions.
2. **Bolt+bolt "siamese" bosses.** Even after (1), bolts at ~10–13 mm survived the washer-sized
   dedup (`2×5 mm`), and their ~7 mm-radius bosses merged into a fused blob. **Fix:** decouple a
   `separation_radius` (the BOSS radius) from `footprint_radius` (washer); dedup + `cross_layer_snap`
   use `2×boss_r ≈ 14 mm`. Feasibility/clearance stays washer-based, so fasteners keep their radial
   band and only space farther along the loop (counts drop slightly: 15→12-13 bolts/layer — the
   intended trade; still ample clamping). **Boss-awareness is bolt↔bolt ONLY** (user, 2026-06-02):
   dowel↔bolt and dowel↔dowel stay footprint-based (a dowel+bolt boss touch is acceptable; sizing
   the bolt off the dowel BOSS was nudging the apex bracket bolt off the pour).
0. **Washer-vs-cup-wall clearance bumped 1 mm → 2 mm** (`WASHER_CUP_WALL_MARGIN_M`). At 1 mm the
   bolt boss (7 mm) overlapped the cup-wall band by ~1 mm and the washer cleared the rising cavity
   wall by only 1 mm — too tight to seat a driver. 2 mm makes the boss just clear the wall band and
   the washer clear by 2 mm (ample for the precision screwdriver used here). Verified by probe that
   the 3D `∖ body` clip never bites a boss/washer: worst boss-ring sits ~4 mm clear of the body,
   washer-ring ~6 mm, 0 bolts with the body inside the washer ring (the cavity is a full
   wall-thickness inboard; the boss-into-wall is a flat coplanar union, not a carve).
3. **Dowel sitting in the apex pour-bracket zone.** Dowels seed at the long-axis extremes (max
   anti-rotation leverage), but for a tall body one extreme IS the dome apex — the pour + clamp-bolt
   zone — so a registration dowel landed where a clamp bolt belongs (and crowded the bracket).
   **Fix:** for the dowel solve only, grow the pour keep-out into the bracket zone
   (`inflate_pour_channels` by ~`2×bolt_boss_r`) so that seed snaps to a clean point just below the
   bracket (still high → good leverage; the base extreme is clear + unaffected). The apex is now
   pour + bracket-bolts only. Trade: the relocated dowel sits ~22 mm off the apex (vs the tip),
   a modest leverage cost the bolt count covers.

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
   reviewed re-baseline, not incidental drift. **Precise invariant (per §7.5):** only **S5a**
   (the default flip) changes geometry; **S5b/S5c/S5d-(A) regen byte-identical to S5a** — S5b/S5c
   delete unreachable/dead code, S5d-(A) reuses an identical silhouette instead of rebuilding it,
   so a placement change and a deletion/refactor never share a commit. (S5d-(B), the silhouette
   fold that *can't* be byte-identical, is deliberately deferred to S4.5's re-baseline — never a
   speed-only re-baseline.) The
   identity holds for **smart-default configs** (all in-tree + production); `smart_placement =
   false` has no successor (it is the path being deleted). S4.5 then re-baselines again when
   `Demand` becomes the default flange — kept a separate software step (own scoped re-baseline)
   so a placement bug and a seal-geometry bug never share one re-baseline, even though both ship
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
