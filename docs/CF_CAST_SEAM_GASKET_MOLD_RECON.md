# cf-cast seam-gasket-mold recon (2026-05-25)

**Status:** scaffold. Triggered by workshop user cf-view check on
post-(4') iter-1 STLs (commit `9ea66296`) flagging silicone-leak
risk at the cup-half seam. Cross-half mating tolerance measurements
this session confirmed the STL geometry is bit-precise within MC cell
quantization (82% of seam-face vertices match to <1 µm; remaining
18% are within one MC cell = 3 mm on the SAME surface). Leak risk
is dominated by FDM print tolerance (~200 µm) which the (4')
ship's "below print resolution" framing did NOT address.

**Picked architectural fix:** per-layer silicone gasket strip,
cast in its own simple flat-tray mold, installed on the cup-half
seam face before pouring the layer's silicone. Silicone-silicone
seam interface = zero cure-inhibition risk + mechanically
compressible seal that absorbs FDM tolerance + small STL-level
seam-face mismatch.

**Why NOT alternatives** (decided 2026-05-25 with workshop user):

- **Workshop putty fillet only** → REJECTED. Workshop user's "no
  shortcuts" framing. Mitigation, not source-fix.
- **O-ring groove** → DEFERRED. Cleaner geometrically but requires
  user to source / cut O-rings per cup. Adds workshop sourcing
  step that the gasket-mold approach avoids (workshop user
  already has silicone + FDM stack).
- **TPU sealing strip co-extrusion** → REJECTED. Requires
  multi-material printer (Bambu A1 base model lacks; AMS add-on
  not on the §G-3 target floor) + TPU/Ecoflex inhibition risk
  needs per-filament-brand compatibility testing.
- **TPU cup halves entirely** → REJECTED. Whole-cup mold release
  becomes load-bearing for cast quality + plasticizer leaching
  risk on every interior surface, not just seam.
- **Increased `RIBBON_PIECE_OVERLAP_M`** → FALSIFIED as a fix.
  Overlap is design-time SDF margin; printed parts don't actually
  overlap in physical assembly (rigid FDM materials can't
  interpenetrate). Bigger overlap shifts the seam-face location
  but doesn't help FDM tolerance directly.

**Out of scope:**

- Finding C (plug-lock socket cavity boolean junction noise at
  cup-floor × pyramid intersection). Separate concern, still
  workshop-judgment-deferred per
  `docs/CF_CAST_POST_SALVAGE_TRIAGE.md`. The gasket addresses
  cup-half seam leakage only.
- Funnel × cup pour-gate sealing (different interface, different
  geometry).
- Plug-bottom × cup-floor mating (silicone pools beneath plug if
  leaky, doesn't escape the cast — not a containment failure).

## §G-0 Why this recon

Empirical data this session (commits `06286520`, `f5d80112`,
`394961e2`, `9ea66296` on dev; measurements at
`~/scans/cast_iter1_bare/` with all mating transforms disabled):

| Metric | Value | Implication |
|--------|-------|-------------|
| Seam-face cross-half vertex match (bit-precise, ≤ 1 µm) | 82% | recon-4 (P) §F-4 SDF→MC invariant holds for the bulk seam face |
| Remaining mismatch (within 3 mm = one MC cell width) | 18% | MC vertex placement differs along the SAME surface; not a geometric defect |
| Max cross-half vertex distance | 3 mm = 1 MC cell | Surfaces match to <1 µm everywhere; only vertex placement differs |
| FDM print tolerance (Bambu A1 + default + Jayo) | ~100-200 µm | Dominates over STL geometry for leak risk |
| Ecoflex 00-30 leak threshold | ~100 µm | Below FDM tolerance worst case |

The STL geometry is already as bit-precise as MC can produce.
Further geometric refinement at the cf-cast level won't reduce
the leak risk because the dominant gap source is downstream in
the print process.

A compressible silicone gasket between cup halves absorbs both
the STL-level cell-quantization mismatch AND the FDM print
tolerance, providing a positive seal independent of those error
sources.

## §G-1 Gasket geometry concept

**Gasket = thin closed-loop silicone strip** in the shape of the
cup-piece seam-face perimeter (outer rectangular boundary + inner
body-cavity opening curve, on the same plane).

- **Cross-section:** trapezoidal or rectangular, ~1-2 mm wide ×
  0.5-1 mm tall. Pinned at S1 via deflection/compression
  calculation against Ecoflex 00-30 shore hardness vs target
  clamping pressure.
- **Routing:** follows the cup-half seam-face perimeter
  (curved-ribbon-following). Open at the body-cavity opening
  (the silicone fills the cavity past the gasket — gasket is the
  CONTAINMENT perimeter, not a cavity floor).
- **Material:** Ecoflex 00-30 first pick (matches cast material;
  same workshop silicone pool). Dragon Skin 10A as fallback if
  Ecoflex compresses too freely. Decided at S2 calibration.

## §G-2 Per-layer scope

Each cast layer has a different outer body shape (offset by
layer wall thickness) → different seam-face perimeter → **one
gasket per layer**.

3-layer cast → 3 gasket molds + 3 gaskets per cast cycle.

Alternative considered + REJECTED: one gasket sized for the
outermost layer used across all 3 layers. Inner-layer gaskets
would be oversized → trail past the inner cup walls into the
silicone cavity → contaminate the cast. Per-layer is the only
clean variant.

## §G-3 Gasket mold form factor

**Single-piece flat tray with closed-loop channel.** Open-top so
workshop user pours silicone in from above, screeds level with a
flat tool, cures, peels gasket out.

- **Tray base:** flat slab, ~3-5 mm thick (rigidity for handling).
- **Channel:** carved into tray top surface, depth ≈ gasket
  thickness (~1 mm).
- **Channel walls:** vertical or slightly tapered for demold.
- **No cup-style 2-piece split** — gasket is small + thin, demolds
  cleanly from an open-top mold via peel-out. Workshop simplicity.

Alternative considered + DEFERRED: dovetail retention channel
(gasket has dovetail tab on bottom that snaps into matching
cup-half seam-face groove). Adds geometric registration but
requires modifying the cup-half seam face. Defer to S5 — if S1-S4
ships with simple peel-stick gasket and workshop iter-3 shows
gasket-shift problems, escalate to dovetail.

## §G-4 Gasket attachment to cup half

**Default:** silicone tackiness alone. Cured Ecoflex 00-30 is
tacky enough to adhere lightly to PLA cup half. Workshop user
positions gasket by hand, presses lightly into place.

**Fallback (if tackiness insufficient):** dab of Smooth-On
Silicone Glue (or uncured silicone from the layer pour) on the
cup-half seam face to anchor the gasket before clamping.

**Fallback (if tackiness + glue insufficient):** geometric
retention via dovetail channel on cup half + matching dovetail
on gasket (S5 §G-3 escalation).

## §G-5 Channel routing around mating features

The cup-pin truncated-pyramid protrusions interrupt the cup-half
seam face. The gasket must either:

- **(a)** route AROUND each pin (channel splits + goes around
  each pin's footprint, leaving a small gap at each pin
  location). Workshop user trims uncured gasket around pins
  before final installation. Allows pin physical mating to
  provide local seal at pin locations.
- **(b)** notch into each pin's footprint (channel has matching
  pin-shaped voids; gasket has matching pin-shaped holes after
  cure). Pins poke through gasket on assembly. Adds precision
  requirement for gasket-mold geometry.
- **(c)** continuous channel ignoring pin locations (gasket
  drapes over pins on installation; pins compress the gasket
  locally). Simplest mold geometry; relies on gasket
  compressibility absorbing the pin compression.

**Picked at S2 via workshop test:** start with (c) (simplest);
fallback to (a) or (b) if iter-3 print + pour shows pin-induced
gasket distortion that leaks.

## §G-6 cf-cast composer architecture

New module `design/cf-cast/src/gasket_mold.rs`:

- `compose_gasket_mold_solid(ribbon, layer_body, bounding_region,
  spec) -> (Solid, Vec<MatingTransform>)`.
- Geometry: tray cuboid `Solid::cuboid` minus a channel `Solid`
  defined as the seam-face perimeter cross-section extruded
  perpendicular to the seam plane.
- Seam-face perimeter extraction: the cup-piece `Solid` already
  encodes the perimeter via
  `compose_piece_solid(Negative).intersect(ribbon.halfspace_solid(
  Positive, 0.0))` — a thin slab at the seam plane that captures
  the seam-face cross-section. Then extrude the resulting 2D
  shape into the gasket channel solid.
- Mating transforms: per §G-5 pick, mesh-CSG cylinder/pyramid
  voids for pin notch-outs OR continuous channel.

Likely additions to `mesh_csg.rs`: `MatingTransform::ExtrudeSlab`
(or similar) if the SDF extrusion path can't express the channel
cleanly. Decision pending at S1 SDF experimentation.

## §G-7 cf-cast-cli integration

cf-cast-cli emits 3 additional STLs per cast:
`gasket_mold_layer_0.stl`, `gasket_mold_layer_1.stl`,
`gasket_mold_layer_2.stl`.

New `[gasket]` config block in cast.toml schema:
```toml
[gasket]
enabled = true              # default true; set false to skip gasket molds
material = "ECOFLEX_00_30"  # gasket material; informs channel cross-section pick
cross_section = "trapezoidal" # rectangular | trapezoidal | round
```

Per-field defaults driven by `GasketSpec::iter1()`. S2
calibration narrows numeric values.

## §G-8 Procedure.rs workshop protocol

New section `## Gasket Casting + Installation Protocol`:

1. Print gasket molds alongside cup halves (same iter STLs).
2. For each layer N:
   - Pour ~5 mL of layer N's silicone into gasket_mold_layer_N
     channel (uses ~5 mL from the layer's main pour).
   - Cure per material TDS (4 hours for Ecoflex 00-30).
   - Peel gasket out of mold.
   - Position on Negative cup half seam face; press to adhere.
   - Close Positive cup half; clamp.
   - Pour remaining layer N silicone through pour-gate.

3. Reuse cured gaskets across cast cycles if they survive demold
   (Ecoflex gaskets are flexible enough for repeated cycles;
   replace if torn).

## §G-9 Cap-floor × plug interface (out of scope)

The plug-floor-lock pyramid socket on the cup-floor (Finding C in
post-salvage triage) is OUT OF SCOPE for this recon. The gasket
addresses cup-half seam leakage only.

The cap-floor × plug interface is a CONTAINED interface (plug
inside cup cavity, surrounded by cup walls). Silicone that leaks
between plug-bottom + cup-floor pools INSIDE the cast cavity, not
outside — it doesn't escape the mold, just contaminates the cast
floor with a thin layer of silicone. Workshop user trims
post-cure if visible.

If workshop iter-3 reveals cap-floor × plug leakage that affects
cast quality, open a separate Finding-C recon arc (likely
involves an internal seal feature on the plug-bottom or
cup-floor — different geometry from the seam gasket).

## §G-10 Open questions

1. **Gasket reusability lifecycle.** Empirical at S6 workshop
   iter-3: how many cast cycles before silicone fatigue + tear?
   Initial estimate: ~5-10 cycles per Ecoflex gasket; replace
   on visible damage.
2. **Gasket peel-out tolerance.** Initial gasket may stick to
   the tray channel walls; need a slight taper (~5° draft) on
   channel walls for clean release. Pick taper at S2.
3. **Curing the gasket inside the gasket mold.** Same workshop
   environment as the main cast (room temperature, ~73°F). No
   special handling.
4. **Cap-plane edge of gasket.** Does the gasket terminate at the
   cap-plane edge cleanly, or wrap around to seal the cap-plane
   wall × seam-plane corner? Pick at S2.
5. **Cup-pin protrusion clearance.** §G-5 (a)/(b)/(c) pick is
   workshop-empirical, pinned at S3 with first iter-3 print.

## §G-11 Workshop iter-3 unblock criteria

1. Gasket molds print cleanly on Bambu A1 + default + Jayo
   (`§R1` connectivity + cf-view smoke for channel geometry).
2. Gasket casts cleanly + demolds cleanly from gasket mold
   (workshop pour validation).
3. Gasket installs on cup half + halves close fully under clamp
   pressure (workshop dry-fit).
4. Silicone pour through funnel × pour-gate: no visible leakage
   at the cup-half seam during pour OR cure. Workshop user
   timestamps gasket-pour-start to silicone-pour-start and
   inspects the seam for silicone weep after both operations.

All 4 gates must clear for the gasket arc to ship.

## §G-12 Bail-out priority

1. **SDF composition fails on the channel geometry** → switch to
   mesh-CSG channel via `MatingTransform::SubtractChannel`
   (new variant). +100 LOC.
2. **Mesh-CSG channel fails F4 gate** → procedure-only workshop
   instruction (user cuts gasket from sheet silicone with seam
   template). Workshop-mitigation fallback; documents the
   geometric impossibility per [[feedback-bookmark-when-surface-levers-exhaust]].
3. **Gasket Ecoflex 00-30 too soft (over-compresses)** → switch
   gasket material to Dragon Skin 10A. S2 calibration; +0 LOC.
4. **Gasket Dragon Skin 10A too stiff (doesn't seal)** → switch
   cross-section from rectangular to trapezoidal/round for more
   compression-per-pressure. S2 calibration; +20 LOC if
   cross-section affects channel mold geometry.
5. **Bambu A1 can't print channel feature (too thin / too narrow)** →
   increase channel dimensions to FDM-printable minimum (typically
   ≥ 1 mm wall + ≥ 0.6 mm extrusion width); gasket grows
   correspondingly. S3 calibration.
6. **Workshop iter-3 print: gasket installs but doesn't seal** →
   open recon-2 on §G-5 routing (try (a) or (b) variants if (c)
   was picked) + §G-3 dovetail retention escalation.
7. **All geometric variants fail to seal under workshop iter-3** →
   bookmark + retreat to workshop-putty mitigation; document the
   `(4')-pattern` extension: silicone-tight FDM molds may require
   workshop putty for the specific material+tolerance combination
   shipped. Honest engineering boundary; not a cf-cast failure.

## §G-13 Implementation arc

8 phases S1-S8 estimate; ~5-7 wall-clock sessions for default
path. Single working branch `dev`, one PR at arc-close (omnibus
per [[feedback-omnibus-pr-single-branch]]).

- **S1: Gasket primitive + spec** (~200 LOC). New
  `GasketSpec` + `compose_gasket_mold_solid` skeleton +
  characterisation tests. SDF composition strategy decided.
  Bit-precise gasket-mold-channel-vs-seam-perimeter fit
  invariant test. iter-1 cf-cast-cli not yet regen'd.
- **S2: Channel cross-section + material calibration** (~100
  LOC). Pick rectangular / trapezoidal / round; pick Ecoflex
  vs Dragon Skin first iter. Compression calculation test
  (gasket thickness × material shore hardness vs workshop
  clamping pressure).
- **S3: Pin-routing + cf-cast-cli integration** (~150 LOC).
  Implement §G-5 pick (default (c) continuous channel; mating
  transforms for (a)/(b) if needed). cf-cast-cli emits gasket
  mold STLs alongside cup pieces. iter-1 regen + §R1 inspector
  + cf-view smoke.
- **S4: Procedure.rs workshop protocol** (~150 LOC prose +
  tests). New §"Gasket Casting + Installation Protocol" section.
  Cap-plane chamfer (4')-pattern-style anchoring tests.
- **S5: Dovetail retention (OPTIONAL — only if §G-4 fallback
  triggers)** (~200 LOC). Modify cup-half seam face to include
  matching dovetail groove for gasket retention.
- **S6: Workshop physical iter-3 print + pour test** (workshop
  user pause). Bambu A1 + default + Jayo. Validates §G-11 #1-#4.
- **S7: Calibration refinement based on S6 caliper data** (~50
  LOC). Tighten / loosen channel dimensions, material choice,
  cross-section profile based on workshop empirical data.
- **S8: Cold-read close + PR to main** (~50 LOC polish + memory
  + bookmark cleanup). Includes the cap-plane recon arc commits
  + the gasket recon arc commits + any deferred dead-code
  cleanup (`MatingTransform::UnionCylinder` per recon-1 §G-5,
  funnel-disconnection fix, etc.).

## §G-14 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rules 1-6, the following memories MUST be read by the S1
implementation session before any architectural decision:

- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — recon-4 (P)
  framework. The gasket-mold channel via mesh-CSG subtract on a
  flat-tray host is the canonical CONTAINED mesh-CSG case (per
  §F-2 framework), but the channel surface coinciding with the
  tray cap face is a potential WELDED-TO-BULK paradigm-boundary
  hazard. S1 SDF experimentation must verify the channel
  primitive doesn't trigger §F-2 failure mode.
- [[project-cf-cast-seam-face-film-recon4-impl]] — recon-4 (P)
  implementation. The seam-perimeter extraction in §G-6 reuses
  the same `Ribbon::halfspace_solid` invariant that recon-4 (P)
  validated. Don't re-litigate; re-use.
- [[project-cf-cast-cap-plane-flatness-bookmark]] — (4')-pattern
  ship + the no-pins isolation probe that confirmed dome+cap
  edge non-flatness is pin-independent. The gasket geometry has
  similar curved-centerline-following character at the bulk
  seam-perimeter; expect similar MC quantization signature.
- [[project-cf-cast-fdm-friendly-geometry-arc]] — the full
  salvage history. Cup-pin mesh-CSG truncated-pyramid is the
  current production mating-feature mechanism; gasket §G-5
  routing must coordinate with cup-pin placement.
- [[project-cf-cast-mating-features-s4-seam-plane]] +
  [[project-cf-cast-mating-features-s5-registration-pins]] +
  [[project-cf-cast-mating-features-s6-t-bar]] — full
  mating-features arc history. Establishes the bit-precise fit
  invariant pattern that the gasket-mold-channel-vs-seam-
  perimeter test should mirror.
- [[feedback-load-bearing-test-fixtures]] — gasket-mold tests
  should pair each "with channel" assertion with a "bare tray
  baseline" assertion to catch fixture breakage.
- [[feedback-cf-cast-tests-use-release]] — `cargo test --release
  -p cf-cast` for all S1-S8 test runs.

## §G-15 Cross-refs

- `docs/CF_CAST_CAP_PLANE_FLATNESS_BOOKMARK.md` — the cap-plane
  (4') recon arc that this session shipped. Same head-architect
  session triggered both arcs.
- `docs/CF_CAST_POST_SALVAGE_TRIAGE.md` — Finding C
  (socket-mouth obstruction) explicitly out of scope here.
- `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` — recon-4 (P)
  source-of-truth for the seam-face SDF halfspace invariant.
- `docs/CF_CAST_FDM_FRIENDLY_GEOMETRY_RECON.md` — recon-1 §G-3
  Bambu A1 + default + Jayo target FDM floor. Same target for
  gasket mold printability gates.
- `~/scans/cast_iter1_bare/` — bare-regen STL set from
  2026-05-25 session showing the 82% bit-precise cross-half
  match baseline (this recon's "before" geometric state).

## Status (open)

- Scaffold shipped 2026-05-25 dev `382426d5` (this recon doc).
- S1 shipped 2026-05-25 dev `1729796b` (+ cold-read pass-1
  `6c8bc7f8`): `GasketSpec` + `compose_gasket_mold_solid` skeleton
  + 11 characterisation tests + module docstring.
- S2 shipped 2026-05-25 dev `c7e042c6` (+ cold-read pass-1 — this
  status update): trapezoidal cross-section (workshop user picked
  §G-7 default) + `GasketMaterial` enum (parameterized — workshop
  empirically picks at S6 iter-3 pour) + Hookean
  `predicted_compression_m`; 222 lib tests + clippy + fmt clean.
- S3 — S8 still pending per §G-13. Next: S3 cf-cast-cli
  integration (~150 LOC).
- Workshop iter-3 print remains BLOCKED on cup-half seam leak
  risk pending S3 — S6 ship.
- Branch: `dev`, no push, no PR until arc close (per
  [[feedback-omnibus-pr-single-branch]]).
