# cf-cast FDM-friendly geometry — recon

**Status:** decisions §G-1 – §G-6 locked from design chat 2026-05-24;
probes §G-7 / §G-8 / §G-9 specified but DEFERRED to a probe-spike
session; implementation protocol §G-13 sketched at scaffold level.
Recon is a multi-turn session — this commit is the scaffold; probes +
final implementation-session estimate fill in later.

**Predecessors:**
[`docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md`](./CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md)
(recon-4 (P), shipped `24bdc221`) →
[`docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md`](./CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md)
(recon-3 (α), shipped `d1a16074`) → PR #255 merged `aadcfed6` →
2026-05-24 user observation post-Bambu-A1-iter-2-print: current
cylindrical pins + plug T-bar twist-lock print fine on calibrated
Bambu Lab but will bind on consumer FDM (Bambu A1 with default
settings + generic filament like Jayo) due to elephant-foot + first-
layer squish + circumferential cylinder-on-cylinder contact.

**Scope:** redesign the cup-piece registration pins + plug retention
mechanism for consumer-FDM-friendly geometry. **Cup-piece body
geometry, funnel nipple, half-capsule mold shell, pour gate, vent —
all OUT OF SCOPE.** Only the mating features get touched.

**Why a recon (not an inline tweak):** the redesign breaks the
`CylinderParent` shared-primitive invariant (the unifying type that
lets cup-pin, plug-shaft, T-bar, T-slot all consume
`MatingTransform::Union/SubtractCylinder`). It also deletes the
T-bar + shaft + T-slot retention mechanism (replaced by a press-fit
truncated pyramid internal to the mold cavity — eliminating the
shaft-through-cup-wall penetration that's today's silicone-leak
risk). These are architectural-ceiling changes affecting S5, S6,
S7 surfaces — multi-session implementation scope. Per
[[feedback-bookmark-when-surface-levers-exhaust]] +
[[feedback-implement-measure-revert-pattern]], decision-doc before
implementation when the failure surface is fully understood; the
design chat preceding this recon characterised the failure surface
sharply enough (FDM gantry mechanics + Bambu A1 floor) to commit.

## What this recon ships

A single decision doc (this one) with:

- Six locked design decisions §G-1 – §G-6 from the 2026-05-24 design
  chat: plug retention mechanism, pin cross-section shape, target FDM
  tolerance floor, print-orientation policy, primitive architecture,
  clearance budget.
- Three deferred feasibility probes §G-7 / §G-8 / §G-9 with branch
  expectations + bail-out paths. Probes run in a follow-up session
  before the implementation session — characterise manifold3d behaviour
  on prismatic primitives + truncated pyramid + first-layer chamfer
  geometry on representative cf-cast curved-shell hosts.
- Test-surface design §G-10 (PrismaticPin bit-precise fit invariant
  analog to S5; §R1 connectivity invariant carrying through).
- Workshop iter-N unblock criteria §G-11 (Bambu A1 + default + Jayo
  print gates, cf-view smoke, §R1 inspector).
- Bail-out priority §G-12 (what does recon-2 look like).
- Implementation-session scope estimate §G-13 — phase ordering
  S0...S8, per-phase LOC estimate, parallelization opportunities.

No production code edits in the recon scaffold itself. The probe
session adds throwaway test files in `mesh/mesh/tests/`. The
implementation arc lands the code across multiple sessions S1...S8.

## Session-cadence

Recon scaffold (this commit): single session, ~2 hr active. Probe
spike session (follow-up): single session, ~3-4 hr — three synthetic
probes against manifold3d, results land in §G-7 / §G-8 / §G-9. Then
implementation arc S1...S8 across an estimated 6-8 sessions, with one
workshop-user-physical gate (S7: Bambu A1 print + caliper acceptance)
that pauses the arc mid-flight.

Per [[feedback-cold-read-two-passes-for-non-trivial-diffs]]: every
S-phase commit gets a cold-read pass-2 polish bundle. Per
[[feedback-omnibus-pr-single-branch]]: all phases collapse onto the
existing `dev` branch — one PR at arc-close (recon + S1...S8 squash).

## Concrete questions this recon must answer

Each question's resolution is a boxed Decision line at the end of the
section (per the recon-2 / recon-3 / recon-4 template). Questions
§G-1 through §G-6 are LOCKED from design chat 2026-05-24; questions
§G-7 through §G-9 are deferred to the probe-spike session; §G-10
through §G-13 are recon-scaffold-resolvable.

### §G-1 — Plug retention mechanism

**Pour orientation note (load-bearing for what "floor" means):**
per procedure.rs:418-419, the mold pours **+Z up**, with the V-pour-
gate + vent at the **dome end of the centerline (= top in pour
orientation)**. The opposite end of the centerline — the
**cap plane** (where the device's open / cuff end is, per
`Ribbon::pour_end_hint` + plug.rs:38-46) — is at the **bottom** in
pour orientation. The user's **"floor of the mold"** = cup-piece
interior at the cap-plane end; the user's **"bottom of the plug"**
= the plug body's cap-plane face. Silicone leaking "out the bottom"
= leaking past the plug-to-cup-floor contact at the cap-plane.

Today's mechanism (post-S6, post-recon-4 (P)): plug has an axial
shaft (cylinder) protruding through the **cap-plane face of the
plug body** along the cap-normal direction (= downward in pour
orientation), exiting through a matching socket in the cup-piece
cap-plane wall. A T-bar (perpendicular cylinder) at the shaft's
deep end. Cup pieces carve half-T-slots in the seam plane that
accept the T-bar via **captive insertion** (procedure.rs:646-648):
lower the plug + T-bar into one cup half's half-T-slot, close the
second half over it — T-bar is captive against axial pull-out
without rotation needed. The shaft penetrates the cup-wall at the
cap plane, creating a through-hole that must be sealed against
silicone leak.

**Consumer FDM problems with the current mechanism:**

1. Cylindrical shaft in cylindrical cup-side socket binds on
   consumer FDM:
   - First-layer elephant foot at the shaft base bulges outward,
     making the bottom layer Ø > nominal Ø — the socket near its
     entry can't accept the bulged base
   - Cylinder lateral surface prints as polygonal facets at the slice
     resolution; cylinder-in-cylinder contact across the entire
     circumference has no design slack to absorb facet-vs-facet
     misalignment
2. T-bar overhang from shaft: T-bar is a perpendicular cylinder
   joining the shaft — creates a 90° unsupported overhang both
   sides of the join. Consumer FDM may sag here without support.
3. Shaft-through-cup-wall penetration leak path: today's design
   relies on shaft-OD-vs-socket-ID tight fit for the silicone seal.
   On consumer FDM where tolerance budget is larger, the shaft-
   socket gap can become a leak path; silicone wicks down through
   the cup wall and out the bottom.

**Design-chat decision (2026-05-24):** the current mechanism is
mechanically OVERKILL for the actual retention task + introduces
an FDM-fragile leak path. The retention need is narrower:

> "we just need an fdm simple pin that locks the mold plug flush
> against the floor of the mold when they are locked together, so
> silicone liquid doesnt get between the bottom of the plug and the
> floor of the mold shell interior, and also so silicone liquid
> doesnt leak out of the bottom" — user 2026-05-24

The retention's job is **seat plug flush against mold floor**, not
"resist captive pullout." This is a downward seating force against
the cap-plane floor, providing the silicone seal at the plug-bottom-
to-cup-floor contact. A press-fit truncated-pyramid mechanism
internal to the cavity does this job without any cup-wall
penetration.

**Locked: truncated-pyramid plug-floor lock.** Plug's cap-plane
face (the face that sits flush against the cup-piece cap-plane
interior surface — "the floor" in the user's framing) carries a
truncated pyramid protruding away from the plug body (= downward
in pour orientation, along `cap_normal`). The cup-piece cap-plane
interior surface carries the matching socket (cavity recessed
INTO the cup material, NOT through the cup wall — no leak path).
Workshop motion: during mold assembly, plug is lowered into the
open cup half (seam plane up) so the pyramid seats into the cup-
piece floor socket; the second cup half closes over the plug,
sandwiching it. Press-stop tactile feedback = the seam halves
bottom out flush against each other when the pyramid is fully
seated.

**Why this works on consumer FDM:**
- All flat planes + sharp angles + straight lines. FDM gantry
  produces these accurately because each face is a single straight
  travel move, not an arc-interpolated polygon.
- The taper angle pre-allocates space for elephant-foot bulge: the
  bed-facing first-layer face is the WIDEST face (truncated-pyramid
  base, with the chamfered first-layer band right at the bed
  contact). Elephant foot grows the base laterally; the socket's
  matching base is also widest at its bed contact, so the elephant-
  foot bulge meets matching elephant-foot bulge with no binding.
- The pyramid lives entirely INSIDE the mold cavity — no shaft
  penetration through the cup wall, eliminating today's leak-path
  failure mode.
- No rotational friction. No twist motion required.
- The press-stop tactile feedback ("plug seats when seam closes
  flush") is acceptable to the user — confirmed 2026-05-24 ("i
  prefer press stop to twist lock for fdm 10/10 times"; the user's
  "twist lock" framing was an earlier-in-conversation mis-framing
  by Claude — the current actual mechanism is captive insertion,
  not twist, but the user's preference for press-stop > anything-
  rotational stands regardless).

> **Decision §G-1.** **Plug retention = truncated-pyramid press-fit
> against the mold's cap-plane floor. T-bar / shaft / T-slot / cup-
> wall penetration DELETED.** The retention point stays at the
> cap-plane end of the plug-cup interface (same location as today's
> mechanism) but moves INSIDE the cavity (no through-hole). The
> truncated pyramid lives at the plug body's cap-plane face,
> pointing AWAY from the body (= downward in pour orientation);
> the matching socket lives recessed into the cup-piece cap-plane
> interior surface. Workshop motion is mold-assembly press-fit
> (seam-close engagement), no twist, no separate parts.

### §G-2 — Pin cross-section shape (cup ↔ cup registration)

Today's mechanism (post-recon-4 (P)): registration pins are
cylinders (radius 1.5 mm, half-length 5 mm, binormal-axis) embedded
in each cup piece. Pin in Negative half mates to socket in Positive
half across the seam plane. Mesh-CSG primitive via
`MatingTransform::UnionCylinder` + `SubtractCylinder` consuming a
shared `CylinderParent`.

**Consumer FDM problems with cylindrical registration pins:**

Same as §G-1 #1: elephant foot at first-layer pin base + circular
cross-section means cylinder-on-cylinder contact across full
circumference has no design slack. Pins printed standing up (Z-axis,
which is the natural orientation for cup pieces — seam face on bed)
get the worst elephant foot.

**Design-chat decision (2026-05-24):** trapezoidal / dovetail-shaped
pins. Rectangular base, sides tapering inward (4-sided pyramid-like
profile) to a chamfered flat-faced tip. All flat planes, sharp
edges, no curves on the lateral surfaces.

> "i have printed snap together molds well with dovetails that are
> arrow/trapezoid shaped and the lines were straight and the angles
> were sharp" — user 2026-05-24, empirical Bambu A1 prior art

**Why this works on consumer FDM:**
- Flat lateral faces don't bind on facet-vs-facet variation: contact
  is plane-on-plane, with design slack budgeted as a uniform offset.
- Tapered insertion + chamfered tip = self-centering on engagement,
  same as §G-1.
- Sharp edges + straight lines are exactly what FDM gantry kinematics
  produce cleanly (no arc interpolation, no segment-count vs
  filament-flow tradeoff).
- The bed-facing first-layer face is the rectangular base — elephant
  foot grows the base outward, and the socket's matching base also
  widens equally; bulge-meets-bulge with design slack absorbing.

> **Decision §G-2.** **Cup-piece registration pins = trapezoidal /
> truncated-pyramid pins (rectangular base, tapered walls, chamfered
> flat-faced tip).** Cylindrical registration pin design DELETED.
> Pin geometry is geometrically isomorphic to §G-1's plug-floor lock
> (both are truncated pyramids at different scales) — the same
> primitive can serve both purposes.

### §G-3 — Target FDM tolerance floor

Today's clearance values (post-recon-4 (P)): 0.20 mm diametral pin
clearance, 0.30 mm diametral T-bar / shaft clearance, 0.50 mm
diametral funnel-nipple clearance (asymmetric). Symmetric `/2`
convention (pin nominal + socket inflated by clearance / 2).

**Design-chat decision (2026-05-24):** the consumer-FDM target floor
is **Bambu A1 + default print settings + Jayo (or similar generic
budget) filament**. That is NOT bottom-tier (stock Ender / stock
Anycubic) — those have 0.4-0.5 mm dimensional error under load.
Bambu A1 with default settings + Jayo prints at maybe ±0.2 mm under
the worst-case combination of elephant foot + cooling shrinkage on
small features.

> "say like a bambulabs a1 is like the bottom tier of what we are
> planning the users of this will have. its just that they might
> not have it tuned perfectly, may be using generic/chea[per
> filaments, etc." — user 2026-05-24

**Clearance budget revision:** 0.20-0.30 mm symmetric calibrated-
printer values bump to **~0.25-0.35 mm symmetric** for the consumer-
FDM floor. Conservative enough to absorb Bambu A1 default-settings
error, tight enough to maintain the workshop-ergonomic snug-fit
contract (current iter-2 fit feel is the reference; loose-fit is a
regression).

**Asymmetric vs symmetric:** an open question — does the socket
print undersized (downward-cut holes on FDM have over-extrusion +
cooling shrinkage at their walls) systematically more than the pin
prints oversized? If yes, asymmetric clearance (push more budget to
the socket side) is a tighter fit at the same nominal value. The
funnel-nipple already uses asymmetric `/2` convention (S7) as
precedent. **Deferred to §G-8 numeric pick** — needs caliper data
from the §G-9 probe-spike Bambu A1 print of representative
geometry.

> **Decision §G-3.** **Target consumer FDM floor = Bambu A1 + default
> settings + Jayo (or equivalent generic filament).** Clearance
> budget bumps to ~0.25-0.35 mm symmetric (or asymmetric — pinned by
> §G-8 numeric value picks). Calibrated-printer (existing Bambu Lab
> P1S / X1C / Prusa MK4) fit becomes "loose but functional," not the
> design target; iter-2 STLs preserved at `~/scans/cast_iter2_calibrated/`
> as side-comparison reference, NOT as a regression target.

### §G-4 — Print-orientation policy

Today: no documented orientation discipline. cf-cast generates STLs;
workshop user (or default slicer) orients arbitrarily.

**Design-chat decision (2026-05-24):** lock orientation in
procedure.rs + slicing-aware design. Specifically:

- **Cup pieces** (6 total: 3 layers × Negative + Positive): **seam
  face down on bed.** Cup-pins point UP in Z (out of the bed-facing
  seam face). Cup-pin first-layer chamfer placed at the seam-face
  base (bed-adjacent). The cap-plane wall of the cup piece is a
  vertical wall perpendicular to the seam face; the plug-lock
  socket on this wall prints as a horizontal SIDE recess (no bed
  contact, no first-layer issue).
- **Plug** (3 total: per-layer): orientation **deferred to S4
  implementation** with the following constraints:
  - Plug body has a hemispherical / domed cap at one end (dome end)
    and a flat cap-plane face at the other end (cuff end). The §G-1
    truncated pyramid protrudes from the cap-plane face in the
    `cap_normal` direction (= away from plug body).
  - If printed cap-plane-face-down: pyramid would protrude INTO the
    bed → geometrically impossible. **Cap-plane-face-down is NOT a
    valid print orientation.**
  - Preferred orientation: **dome end on bed, cap-plane face up,
    pyramid pointing up.** Pros: flat cap-plane face's pyramid is at
    the top of the print = no elephant-foot issue; cons: dome contact
    patch on bed requires brim or raft, may need supports for the
    first few layers of the dome's curvature.
  - Alternative: plug on its side (centerline horizontal). Requires
    supports for body curvature overhangs.
  - S4 picks the specific orientation based on iter-1 / iter-2 plug
    geometry. procedure.rs documents the chosen orientation.
- **Funnel + platform**: today's `platform.stl` has a blind pocket
  to accept the T-bar shaft protruding through the cup-wall floor.
  Under §G-1, no shaft protrudes through the cup wall → the
  platform's blind pocket is no longer load-bearing. Platform.stl
  becomes a flat support for the mold (or could be deleted; locked
  for §G-13 S4 to decide).

**Why locked-orientation is the right policy here:**

- Mold is a fixed-purpose tool — there's one right print orientation
  per piece for FDM. Workshop user follows the procedure.rs
  instruction.
- First-layer chamfer placement (the dominant FDM-friendliness
  knob) requires knowing which face will be the bed-facing face.
  An orientation-agnostic geometry would have to chamfer EVERY
  bed-eligible face, bloating geometry + bleeding clearance budget.
- Slicer "you printed it wrong" warnings get sharp: cup piece prints
  upside-down → pins print as Z-axis HOLES (no chamfer at the hole
  base) which is correctly diagnosed as an unprintable orientation
  by the slicer; workshop user sees the issue at slice preview, not
  post-print.

**Per-piece procedure.rs documentation** must include:
- Which face goes on bed (cup pieces: seam face; plug: per S4
  decision — preferred dome end on bed, cap-plane face up)
- First-layer chamfer dimension (locked in §G-5 / §G-8) — load-
  bearing for the cup-pin (bed-adjacent); plug-pyramid chamfer
  contingent on the S4 print-orientation pick
- "Default Bambu A1 settings + Jayo filament" reference recipe
- Standard support setting (cup pieces: "none required" given
  chamfer + seam-face-down stability; plug: per S4 — likely brim
  for dome contact, possibly minor supports)

> **Decision §G-4.** **Cup-piece print orientation LOCKED: seam
> face on bed. Plug print orientation DEFERRED to S4** with the
> constraint that cap-plane-face-down is invalid (pyramid would
> protrude into bed); preferred dome-end-on-bed. First-layer
> chamfers placed only where the bed contact actually happens
> (load-bearing for cup-pin; plug-pyramid chamfer depends on S4
> orientation pick). Orientation-agnostic geometry REJECTED.

### §G-5 — Architectural primitive design: `PrismaticPin`

Today's primitive (post-S6): `CylinderParent { center_m, axis_unit,
half_length_m }` + `CylinderParams { parent, radius_m }`.
`MatingTransform::UnionCylinder` / `SubtractCylinder` consume these.

**Replacement primitive (locked from §G-1 + §G-2):** a truncated-
pyramid primitive serving both cup-pin and plug-floor-lock purposes
(geometric isomorphism — same shape at different scales). Working
name: `PrismaticPin` (or `TruncatedPyramid` — pinned at impl).

**Proposed type signature:**

```rust
pub struct PrismaticPinParent {
    pub center_m: Vec3,        // Mid-point of the pin's axis,
                                // in world coordinates
    pub axis_unit: Vec3,       // Insertion axis (unit length)
    pub base_half_extents_m: Vec2, // Base rectangle half-extents
                                    // in the plane perpendicular
                                    // to axis_unit
    pub tip_half_extents_m: Vec2,  // Tip rectangle half-extents
                                    // (tip_half_extents < base_half_extents
                                    // for a truncated pyramid)
    pub half_length_m: f64,    // Half-length along axis_unit
    pub base_chamfer_m: f64,   // First-layer chamfer height
                                // applied at the base end only
                                // (axis_unit points away from bed)
}

pub struct PrismaticPinParams {
    pub parent: PrismaticPinParent,
    pub diametral_clearance_m: f64,    // Per-side, applied to
                                        // base + tip extents on
                                        // socket inflate
    pub axial_clearance_m: f64,        // Applied to half_length
                                        // on socket inflate
}

pub enum MatingTransform {
    SeamTrim { ... },              // existing
    UnionCylinder(CylinderParams),  // existing — KEPT for
                                    // funnel-nipple + pour-gate
                                    // (S7 mating features that
                                    // are OUT OF SCOPE per §G-1's
                                    // funnel exclusion)
    SubtractCylinder(CylinderParams), // existing — KEPT for pour-gate
    UnionPrismaticPin(PrismaticPinParams),    // NEW
    SubtractPrismaticPin(PrismaticPinParams), // NEW
}
```

**Why the cylinder primitives stay:** the cup pour-gate cylinder is
the one surviving `MatingTransform::SubtractCylinder` consumer post-
this-arc (per [[project-cf-cast-mating-features-s7-funnel-pour-gate]]
the pour-gate is mesh-CSG; per [[project-cf-cast-funnel-disconnection-fix]]
the funnel nipple itself is back in SDF — uses no mesh-CSG cylinder).
So `CylinderParent` + `MatingTransform::SubtractCylinder` stay for
the pour-gate carve. `MatingTransform::UnionCylinder` becomes UNUSED
under the default path (all pin / T-bar / shaft consumers deleted);
keep the variant as defensive primitive for one cold-read pass, then
delete in S8 if confirmed unused.

**What gets deleted from the spec:**

`PinSpec` (today: `radius_m`, `half_length_m`, `pin_offset`,
`diametral_clearance_m`, `axial_clearance_m`) → REPLACED by
`PrismaticPinSpec` with the rectangular-base fields + chamfer.

`PlugPinSpec`'s T-bar + shaft fields (today:
`t_bar_radius_m`, `t_bar_length_m`, `t_bar_diametral_clearance_m`,
`t_bar_axial_clearance_m`, `shaft_radius_m`, `shaft_length_m`,
`shaft_diametral_clearance_m`, `shaft_axial_clearance_m`) → DELETED.
Replaced by a single `PrismaticPinSpec`-shaped block for the plug-
floor lock (base extents, tip extents, length, chamfer, clearances).

**Shared-primitive invariant (analog to S5 / S6 architectural keep):**
the S6 three-piece invariant ("plug ↔ Negative cup ↔ Positive cup
all consume the same primitive for bit-precise fit") was built on
the T-bar + shaft + cup-T-slot triple where the seam plane bisected
the T-bar's cylinder. Under §G-1, the truncated pyramid is at the
plug's cap-plane face — the seam plane likely bisects this pyramid
symmetrically (each cup half carves half the socket, same shape as
today's half-T-slot pattern), giving a **3-piece shared-primitive
invariant** (plug + Negative cup half + Positive cup half). An
asymmetric variant (socket entirely in one cup half) would be
2-piece. Decision deferred to S2 / S4 implementation; both are
geometrically supportable. The cup-pin shared-primitive invariant
is 2-piece (Negative-half pin + Positive-half socket), same shape
as today's S5 pin/socket pair.

> **Decision §G-5.** **New primitive `PrismaticPin` replaces
> `CylinderParent` for all mating features.** `MatingTransform`
> gains `UnionPrismaticPin` + `SubtractPrismaticPin` variants.
> `CylinderParent` + cylinder variants stay for non-mating features
> (cup pour-gate carve).
> `CylinderParent` shared-primitive invariant SUPERSEDED — the new
> shared primitive is `PrismaticPin`, shared between cup-pin
> (2-piece: pin + socket) and plug-floor lock (2 OR 3 piece per
> S2 / S4 implementation choice). S6 three-piece invariant analog
> carries through if plug-floor lock socket is seam-plane-symmetric
> across both cup halves.

### §G-6 — Clearance budget + first-layer chamfer recipe

Locked-from-design-chat parameter envelope (numeric values pinned in
§G-8 after the probe-spike caliper data):

**Cup-pin (PrismaticPin):**
- Base half-extents: ~1.5 mm × 1.5 mm to ~2.5 mm × 2.5 mm
  (matches today's pin Ø 3 mm-ish footprint; pin volume budget
  constrained by cup-wall thickness 5 mm and seam-plane wall area).
- Tip half-extents: ~70-85% of base (i.e., taper ratio 0.7-0.85).
  Steeper taper = stronger self-centering but smaller tip = weaker
  workshop registration "feel."
- Half-length: ~2-3 mm (similar to today's 3 mm post-recon-3 (α)
  reduction).
- Diametral clearance: 0.25-0.35 mm symmetric (§G-3); possibly
  asymmetric.
- Axial clearance: 0.5-1.0 mm at socket bottom (relief for socket-
  bottom over-extrusion bulge).
- Base chamfer: 45° × 0.4-0.8 mm absorbing elephant foot.

**Plug-floor lock (PrismaticPin at larger scale):**
- Base half-extents: ~3-5 mm × 3-5 mm (larger — plug-floor lock
  carries the seating force).
- Tip half-extents: ~60-75% of base (steeper taper than cup-pin —
  the plug-floor lock benefits more from wedge action).
- Half-length: ~3-5 mm. Constrained by cup-wall thickness at the
  cap-plane region (5 mm wall per [[project-cf-cast-mold-wall-arc-shipped]])
  — socket recess can be at most ~4 mm deep to retain ~1 mm cup
  material on the outside.
- Diametral clearance: 0.30-0.40 mm symmetric (slightly looser than
  cup-pin to ease seating; the wedge geometry locks regardless).
- Axial clearance: 0.5-1.0 mm at socket bottom (same recipe).
- Base chamfer: **contingent on §G-4 plug print orientation.** If
  plug prints dome-down (preferred), the pyramid is at the top of
  the print = no elephant-foot issue, base chamfer is OPTIONAL
  (could still be useful for lead-in alignment, just not for FDM
  reasons). If plug prints in a different orientation that puts
  the pyramid base near the bed, a 45° × 0.6-1.0 mm chamfer matches
  the cup-pin recipe.

**Chamfer placement:** ONLY where the feature contacts the bed in
its locked print orientation. For cup-pin (printed seam-face-down),
the chamfer is at the seam-face base — bed-adjacent in print
orientation. The cup-pin SOCKET (in the matching cup half) also
gets the chamfer at its bed-adjacent entry. For plug-pyramid +
plug-pyramid socket: no first-layer issue under the preferred §G-4
orientation; lead-in chamfer remains a soft option.

**Numeric values:** pinned in §G-8 after the §G-9 probe-spike print
+ caliper measurement.

> **Decision §G-6.** **Parameter envelope locked at typed-range
> level; numeric values pinned in §G-8 post-probe.** Cup-pin base
> 1.5-2.5 mm half-extent; plug-lock base 3-5 mm half-extent; taper
> ratios 0.6-0.85; clearances 0.25-0.40 mm; first-layer chamfer
> 0.4-1.0 mm at the BED-CONTACT base (load-bearing for cup-pin
> only; plug-pyramid chamfer optional per §G-4 print-orientation
> pick).

### §G-7 — manifold3d feasibility probe: PrismaticPin CSG on curved-shell host

**Question:** does `manifold3d` produce a clean (manifold, 1 connected
component, 0 F4 Critical issues) result when unioning / subtracting
a truncated-pyramid primitive onto a curved-shell host (cup-piece
SDF→MC topology with body cavity)?

The §F-3 / §R3-3 probe history (recon-3 + recon-4) characterised
manifold3d's behaviour on contained vs protruding CYLINDERS into
shell hosts. The PrismaticPin primitive is also a closed convex
solid; the same characterisation likely transfers. But:

- The pyramid has FLAT angled faces (not curved like cylinders).
  manifold3d's robustness on plane-vs-plane intersection (pyramid
  face × MC vertex face) at small dihedral angles is the open
  question.
- The pyramid base is a rectangle (4 vertices) vs cylinder base
  (32 vertices at SEGMENTS=32). The MC mesh has many more vertices
  per unit area; the boolean has to resolve a rectangle's edges
  against the MC mesh's many-edge surface.
- The truncated-pyramid + bed-chamfer compound geometry produces a
  shape with non-trivial topology even before composition.

**Probe.** Build a synthetic curved-shell host that mimics the cup-
piece envelope topology (a hollow torus segment, or a hollow ribbon
sweep, with the same wall-thickness + bounding-margin as iter-1):

```rust
// design/cf-cast/tests/g7_probe_prismatic_pin_on_curved_shell.rs
let body = curved_body_sdf();         // ribbon sweep at curvature
                                       // representative of iter-1
let bounding = body.offset(0.005);    // 5 mm wall
let shell = bounding.subtract(body);   // hollow shell
let shell_mesh = shell.to_marching_cubes_mesh(0.003); // 3 mm cells

let pin = build_prismatic_pin_along_axis(/* base_half=1.5mm,
                                            tip_half=1.2mm,
                                            half_length=3mm,
                                            chamfer=0.5mm */);

let result = apply_mating_transforms(&shell_mesh,
    &[MatingTransform::UnionPrismaticPin(pin)]);

assert_eq!(connected_components(&result), 1);
assert_eq!(f4_critical_issues(&result).len(), 0);
```

**Branches:**

- BRANCH A: 1 component, 0 critical issues. PrismaticPin behaves
  isomorphically to CylinderParent under manifold3d on shell hosts.
  Implementation proceeds per §G-13 default path.
- BRANCH B: > 1 component (disconnection like recon-3's pin bug pre-
  (P)). The PrismaticPin primitive has manifold3d-pathological
  topology that the cylinder didn't. Bail-out per §G-12 #1:
  pre-MC SDF union for the pin geometry (architectural pin: pin
  lives in SDF, the chamfered tip is the only mesh-CSG-side piece).
- BRANCH C: 1 component BUT F4 Critical issues at the boolean
  junction (similar to the funnel-nipple post-MC issue that
  triggered the recon-4 funnel-revert per
  [[project-cf-cast-funnel-disconnection-fix]]). Bail-out per §G-12
  #2: paradigm-boundary correction — pin lives in SDF (welded-to-
  bulk), chamfer band stays mesh-CSG.

### §G-7 Empirical outcome (probe-spike 2026-05-24)

Probe shipped on `dev` at
`design/cf-cast/tests/g7_g9_prismatic_pin_probe.rs` (8 tests, all
PASS post-characterisation). Host: half-sphere shell
`sphere(R).shell(t) ∩ halfspace(z > 0)` with R = 30 mm, `Solid::shell`
half-thickness 5 mm (total wall 10 mm — `Solid::shell` builds a `2t`-
thick shell centred on the original surface per
`evaluate.rs:107`), MC'd at the cf-cast-cli default cell size
(3 mm) and at an over-resolved control cell size (1 mm) for
quantization characterisation. Pin: §G-7 doc spec (base 1.5 mm /
tip 1.2 mm / half-length 3 mm / chamfer 0.5 mm), binormal-axis
oriented (+Y), centred at the wall midpoint (32.5 mm, 0, 5 mm).

**Findings:**

| Probe | Baseline components | Post-CSG components | New F4 Critical types | Branch |
|---|---:|---:|---|---|
| §G-7 Union @ 3 mm | 1 | **3** | `[SelfIntersecting]` | B + C |
| §G-7 Union @ 1 mm | 1 | **3** | `[SelfIntersecting]` | B + C |
| §G-7 Subtract @ 3 mm | 1 | **2** | `[]` | B only |
| §G-7 Subtract @ 1 mm | 1 | **2** | `[]` | B only |
| §G-12 #2 bail-out (SDF-side pin, 3 mm) | 1 | **1** | `[]` | A |

**Interpretation.** The §G-7 default path (mesh-CSG `Manifold::union` /
`difference` of a `Manifold::hull_pts` truncated-pyramid pin against
the SDF→MC half-shell host) FAILS the BRANCH-A criteria at BOTH cell
sizes. The 1 mm over-resolved control case fails identically to the
3 mm production case — so the failure is **not** pure MC
quantization, it is the mesh-CSG-union of a sharp-cornered convex
polyhedron against an MC-tessellated curved-shell host. This is the
exact paradigm-boundary pattern recon-4 characterised (bulk-welded
SDF/MC geometry × fine mesh-CSG primitives = paradigm-boundary risk;
see [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]).

The §G-12 #2 bail-out (pin geometry pre-MC, composed into the host
SDF via `Solid::union`) PASSES the BRANCH-A criteria — 1 component,
no new F4 Critical types. This is the recon-4 (P) §F-3 SDF-union pin
pattern transposed onto PrismaticPin: the architectural framework
already had this surface load-bearing for `CylinderParent` on
production cup-piece pieces; the probe confirms it carries over.

The recon-4 §F-3 docstring's inference — "if SDF-union pin works on
this fixture, mesh-CSG-union pin on the same clean 1-component
half-shell host is at least as robust topologically" — was an
UNTESTED extrapolation from the recon-3 §R3-3 manifold3d-DIRECT
cube-shell-host probe, not from a synthetic MC-curved-shell host
plus mesh-CSG primitive. The §G-7 probe IS that missing test, and
the inference is empirically FALSE for `hull_pts` truncated-pyramid
primitives. Open question whether this also extends to mesh-CSG
`Manifold::cylinder` primitives on MC-curved-shell hosts — production
iter-1 cup-piece STLs (recon-4 (P) §F-5 #1) pass §R1 at 1 component
per piece, so production-fixture cylinder × cup-piece-SDF works, but
this synthetic probe says we can NOT extrapolate to all
MC-curved-shell × mesh-CSG combinations.

> **Decision §G-7 (revised). BRANCH B + BRANCH C BOTH fire under
> the default mesh-CSG path.** §G-12 #2 bail-out (pin lives entirely
> in SDF) PICKED as the implementation path. The chamfer band per
> §G-9 emits as SDF too (cuboid subtract or smooth taper) — see §G-9
> revised decision below. `MatingTransform::UnionPrismaticPin` /
> `SubtractPrismaticPin` variants are NOT introduced (since the pin
> isn't mesh-CSG); the §G-13 implementation arc scope drops S2 and
> S5 LOC accordingly.

### §G-8 — Numeric value picks: clearances + chamfer dimensions

**Question:** what specific numeric values does the design ship?

Locked typed-range from §G-6, but specific picks need empirical
caliper data from a Bambu A1 + default + Jayo print of representative
geometry. Today's iter-2 values (Bambu Lab calibrated-printer
clearances: 0.20 mm symmetric for pins, 0.30 mm for T-bar / shaft)
are the floor for the consumer-FDM values (consumer needs MORE
clearance, not less).

**Probe.** Print a calibration article on Bambu A1 + default + Jayo:
- 4× cup-pin pairs at clearance 0.20 / 0.25 / 0.30 / 0.35 mm
- 4× plug-lock pairs at clearance 0.25 / 0.30 / 0.35 / 0.40 mm
- 3× chamfer dimensions: 0.4 / 0.6 / 0.8 mm
- All at representative scale (cup-pin base 1.5 mm half-extent;
  plug-lock base 4 mm half-extent)
- 2 print orientations (correct: base on bed; incorrect: side on bed
  — for slicer-warning calibration)

**Acceptance criterion:** for each pin / lock pair, can the workshop
user (a) seat without forcing, (b) achieve flush bed-to-bed contact
once seated, (c) retract for demolding without tools? Pick the
TIGHTEST clearance that PASSES all three criteria — that becomes
the spec default.

**Asymmetric clearance question:** caliper the socket's printed Ø
vs the pin's printed Ø separately. If socket prints undersized > pin
prints oversized by ≥ 0.1 mm consistently, push that asymmetry into
the spec (socket clearance > pin clearance) per the S7 funnel-nipple
precedent.

**Probe deferred to the workshop-user-physical S7 phase** in the
implementation arc — this is a print-and-caliper task, not a
synthetic spike. The recon doc pins ranges; implementation S7
substitutes specific values.

> **Decision §G-8. DEFERRED to implementation S7 workshop-physical
> calibration.** Recon ships the typed range from §G-6. Default
> starting values for the implementation arc: cup-pin 0.30 mm
> symmetric / chamfer 0.6 mm; plug-lock 0.35 mm symmetric /
> chamfer 0.8 mm. S7 calibration article narrows to ±0.05 mm
> precision.

### §G-9 — First-layer chamfer geometry (CSG vs slicer)

**Scope note:** the first-layer chamfer is load-bearing primarily
for the **cup-pin** (bed-adjacent base when cup pieces print
seam-face-down per §G-4). The plug-pyramid + plug-pyramid socket
do NOT have first-layer chamfer concerns under the preferred §G-4
plug orientation (dome on bed, pyramid at top of print). Cup-pin
socket on the matching cup half: also bed-adjacent (opening on
the seam-face = bed), also needs the chamfer at its entry. So
§G-9's question affects the cup-pin pair primarily.

**Question:** is the first-layer chamfer geometry generated by the
cf-cast pipeline (CSG-level chamfer on the PrismaticPin primitive),
or left to the slicer's "chamfer first layer" post-processing?

**Option (i) CSG-level chamfer:** PrismaticPin primitive carries a
`base_chamfer_m` field; the primitive's SDF / mesh emission includes
the chamfer band. Chamfer dimensions are designed-in, not slicer-
dependent. Print orientation is the only assumption needed.

**Option (ii) slicer-level chamfer:** primitive geometry is sharp at
the bed-facing edge; slicer's "elephant foot compensation" + first-
layer expansion settings handle the chamfer. Specific Bambu A1
settings need documenting in procedure.rs but no CSG complication.

**Tradeoffs:**

| Aspect | CSG (i) | Slicer (ii) |
|---|---|---|
| Reproducibility across slicers | Identical | Slicer-dependent |
| Geometry complexity | +chamfer band per primitive | None |
| Bambu A1 default settings | Match the designed chamfer | "Set elephant foot comp to X" instruction |
| Consumer FDM friendly | Defaults work | User must remember slicer setting |
| Generic filament behaviour | Designed for the worst filament | Slicer doesn't know filament; may over- or under-compensate |
| LOC cost | ~80-120 LOC (PrismaticPin geometry emission) | ~0 LOC |

**My lean:** Option (i) — CSG-level chamfer. The recon's premise is
"99% of users, generic filament, default settings" — that means we
DON'T trust the workshop user to set elephant-foot compensation
correctly. Designing the chamfer into the geometry shifts the
correctness burden from "user must know slicer feature" to "user
prints the file with default settings, geometry handles itself."

The LOC cost (~80-120 LOC) is real but bounded; the chamfer
emission is one function that runs once per primitive.

**Probe.** Print a comparison article: same PrismaticPin geometry
with (a) CSG-level 0.6 mm chamfer + slicer default elephant-foot
comp; (b) sharp CSG edge + slicer "first layer +0.6 mm" expansion.
Compare seating fit + lateral-face quality.

### §G-9 Empirical outcome (probe-spike 2026-05-24)

Sweep ran the §G-7 default mesh-CSG path with `chamfer_mm ∈ {0.0,
0.4, 0.6, 0.8, 1.0 mm}`. ALL chamfer values fail IDENTICALLY:
post-union components = 3 (baseline 1, +2 disconnected shells),
new F4 Critical types = `[SelfIntersecting]`. The §G-7 failure is
**chamfer-INDEPENDENT** — it's the underlying mesh-CSG-on-MC-curved-
shell paradigm boundary (§G-7 BRANCH B + C), not the chamfer band
geometry. The PrismaticPin primitive's `Manifold::hull_pts` build
succeeds cleanly at every chamfer (volume ≈ 41–44 mm³ across the
sweep, monotone-decreasing as chamfer narrows the base footprint;
manifold-clean per `is_empty() == false`); the failure is in the
union/difference op against the MC-curved-shell host, not in the
primitive.

> **Decision §G-9 (revised). CSG-LEVEL CHAMFER (option (i))
> RETAINED, but emitted SDF-SIDE per §G-12 #2.** The PrismaticPin
> primitive's chamfer band emits as part of the SDF composition —
> e.g. `cuboid(base_half) ∪ cuboid(inset_half, base_half)
> smooth-min` for the chamfered base, or per-axis half-extent
> reductions along the pin's axis for a piecewise-linear taper plus
> chamfer band. The §G-9 design contract (chamfer designed-in, no
> slicer configuration required) is intact; only the emission
> mechanism (SDF rather than mesh-CSG) changes. §G-12 #3 (slicer-
> level chamfer fallback) is NOT picked — the §G-7 mesh-CSG failure
> is chamfer-INDEPENDENT, so swapping CSG-chamfer for slicer-
> chamfer alone wouldn't unblock §G-7.

### §G-10 — Test surface design

Three test families inherit the existing architecture, redesigned
for the new primitive:

**(1) Bit-precise fit invariant (analog to S5
`pin_and_socket_fit_invariant`):** for every PrismaticPin pair
(cup-pin Negative + cup-pin Positive socket, plug-lock plug + cup
socket), the pin's vertex set is bit-identical to the socket's
vertex set after applying the diametral + axial clearance inflate
on the socket side. Math-verified to f64 precision, per
[[feedback-math-verify-geometric-contracts]]. Replaces S5's
`pin_and_socket_fit_invariant` + S6's `t_bar_*_fit_invariant` +
S6's `plug_shaft_*_fit_invariant` (each tests a different
3-piece-shared-primitive sub-aspect).

**(2) §R1 connectivity invariant (analog to existing
`iter_connectivity_inspector.rs`):** every production STL passes
`find_connected_components` at 1 component per piece (post-
recon-4 (P) baseline). Carries through under §G-7 BRANCH A; loosens
to 2 components per piece under §G-7 BRANCH B with documented
sliver location.

**(3) FDM-printability heuristic:** new gate, analog to F4 Critical
issues but FDM-tuned:
- Maximum overhang angle (from any face's normal to the print Z-axis)
  ≤ 45° for support-free printing
- Minimum first-layer feature size ≥ 0.4 mm (nozzle Ø)
- No bridges longer than 5 mm at first-layer-adjacent layers
- No floating geometry (chamfered tips, etc., are anchored)

The FDM heuristic is informs-not-gates (per S4 `is_blocking_critical`
target-aware framework) — F4 Critical for the FDM target audience,
informational for the slicer-tuned target.

**Per-phase test surfaces:**

| Phase | Test family | LOC |
|---|---|---:|
| S1 (probe spike) | §G-7 / §G-8 / §G-9 probes in `mesh/mesh/tests/` | ~150 (throwaway) |
| S2 (PrismaticPin primitive) | `prismatic_pin_sdf_evaluates_correctly`, `prismatic_pin_mesh_emission_topology`, `prismatic_pin_bit_equal_across_calls` | ~120 |
| S3 (cup-pin migration) | `cup_prismatic_pin_and_socket_fit_invariant`, `cup_pin_mesh_is_bit_equal_across_pieces` | ~150 |
| S4 (plug-floor lock migration) | `plug_lock_and_socket_fit_invariant`, `plug_lock_seats_flush_against_floor_synthetic` | ~120 |
| S5 (first-layer chamfer) | `prismatic_pin_chamfer_band_geometry`, `chamfer_does_not_affect_socket_fit` | ~80 |
| S6 (procedure.rs) | `procedure_md_contains_print_orientation`, `procedure_md_lists_target_floor` | ~40 |
| S7 (workshop-physical) | (no new code-level tests; calipers + cf-view smoke) | ~0 |
| S8 (cold-read polish) | (per-find polish) | ~60 |
| **TOTAL test LOC** | | **~570** |

> **Decision §G-10.** **Three test families: bit-precise fit
> invariant, §R1 connectivity, FDM-printability heuristic.** ~570
> LOC test code across S1-S8.

### §G-11 — Workshop iter-N unblock criteria

Workshop iter-2 unblock (Bambu Lab calibrated print) — ALREADY
UNBLOCKED per the PR #255 merge (`aadcfed6`). Not in scope for this
recon.

**Workshop iter-3 unblock (Bambu A1 + default + Jayo print):**

1. **§R1 connectivity inspector PASSES** on all production STLs
   (today: 11 = 6 cup pieces + 3 plugs + funnel + platform; post-§G-1
   the platform may retire to a flat support or be eliminated per
   §G-4, so count may drop to 10) at the spec threshold (1 component
   per piece, post-(P) baseline; allows ≤ 2 with sub-mm sliver under
   §G-7 BRANCH B).
2. **PrismaticPin bit-precise fit invariant PASSES** in
   `design/cf-cast` cargo tests.
3. **cf-view smoke gate (workshop-user-physical):** workshop user
   opens iter-3 STLs in cf-view, visually confirms:
   - Cup-pin geometry is trapezoidal + chamfered at base
   - Plug-floor lock is truncated-pyramid + chamfered at base
   - No T-bar / stem / T-slot remnants visible
   - No first-layer chamfer at wrong end (sanity check on §G-4
     orientation discipline)
4. **Bambu A1 + default + Jayo print success (workshop-user-physical):**
   workshop user prints 1 cup piece + 1 plug + 1 mating-pair-only
   small sample on Bambu A1, confirms:
   - Cup-pin seats into socket without forcing
   - Plug-floor lock seats plug flush against cup-piece floor
   - No bind, no jam, no need for post-print sanding
   - Press-stop tactile feedback feels right (workshop ergonomic
     judgment)

Gates #1 + #2 are mechanical; gates #3 + #4 are workshop-user-
physical and pause the implementation arc until cleared.

> **Decision §G-11.** **Iter-3 unblock quad-gate: §R1 connectivity
> + PrismaticPin fit invariant + cf-view smoke + Bambu A1 print.**
> All four MUST clear before PR opens. Gates #3 + #4 are workshop-
> user-physical per [[feedback-diagnostic-signal-vs-workshop-failure]]
> — the implementation arc pauses for these gates, doesn't skip
> them.

### §G-12 — Bail-out priority

Per the recon-3 §R3-6 / recon-4 §F-6 pattern:

1. **§G-7 BRANCH B fail (PrismaticPin disconnects on shell host):**
   pre-MC SDF union for the pin primitive (architectural pin
   migration: PrismaticPin's bulk lives in SDF, chamfer band stays
   mesh-CSG). ~50-100 LOC additional vs §G-13 default. Workshop
   iter-3 print not blocked.
2. **§G-7 BRANCH C fail (F4 Critical issues at boolean junction):**
   paradigm-boundary correction (§F framework from
   [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]): pin lives
   entirely in SDF (like the recon-4 funnel-nipple revert).
   ~100-150 LOC additional. The PrismaticPin primitive emits SDF +
   transforms-vec just like Cylinder does today.
3. **§G-9 CSG-chamfer geometry breaks manifold3d** (the chamfer band
   has thin-wall topology manifold3d can't resolve): fall back to
   §G-9 option (ii) (slicer-level chamfer), document elephant-foot
   compensation setting in procedure.rs. Removes ~80-120 LOC chamfer
   geometry; adds ~10 LOC procedure.rs.
4. **Bambu A1 print FAILS even with §G-9 fallback** (clearances
   wrong, geometry wrong): bookmark + new recon. Workshop iter-3
   blocks on the new recon. Possible re-pivot to entirely different
   mating-feature design (e.g., undercut snap-fit, magnetic
   registration, etc.).
5. **§G-11 fit invariant fails** (PrismaticPin emission is not bit-
   precise across pin / socket pair): single-phase implementation
   iteration; likely an emission-determinism bug per the S5
   `bit_equal_across_pieces` pattern.

> **Decision §G-12.** **Five-priority bail-out tree.** Each branch
> has a specific LOC delta + decision criteria. None of (1)-(3)
> require a new recon; (4) does. (5) is a debug iteration, not a
> bail-out.

> **Decision §G-12 (post-probe-spike, 2026-05-24). #2 PICKED** as the
> implementation path per §G-7 + §G-9 empirical outcomes. The probe
> falsified the §G-13 default mesh-CSG path at the synthetic-fixture
> level; the §G-12 #2 SDF-side bail-out PASSES BRANCH-A criteria on
> the same fixture. The S2-S8 implementation arc transposes into the
> §G-12 #2 architecture rather than the mesh-CSG default; LOC delta
> is approximately NEUTRAL (mesh-CSG primitive emission cost ≈ SDF
> emission cost; no `MatingTransform::*PrismaticPin` variants are
> introduced; chamfer band lives SDF-side per revised §G-9). See
> revised §G-13 below.

### §G-13 — Implementation-session scope estimate

| Phase | Subject | LOC | Notes |
|---|---|---:|---|
| S0 | Recon scaffold (this commit + pass-1 + pass-2 polish) | ~1000 | this doc |
| S1 | Probe spike: §G-7 + §G-9 manifold3d feasibility | ~700 | SHIPPED 2026-05-24 `design/cf-cast/tests/g7_g9_prismatic_pin_probe.rs`, 8 characterisation tests; §G-7 BRANCH B+C falsification + §G-12 #2 BRANCH-A confirmation |
| S2 | `PrismaticPin` SDF primitive (revised per §G-12 #2) | ~300 | new file `design/cf-cast/src/prismatic_pin.rs`; SDF-side emission via `Solid::cuboid` + taper + chamfer-band composition, params, bit-precise fit-determinism gate per §G-10 #1 — **no** `MatingTransform::*PrismaticPin` variants (pin lives entirely SDF-side) |
| S3 | Cup-piece registration pin migration | ~400 | SHIPPED 2026-05-24 `registration.rs` rewrite — `PinSpec` wraps `PrismaticPinSpec::cup_pin_default()` + arc_fractions; `build_registration_transforms` → `build_registration_sdf_ops` returning `Vec<Solid>` for SDF-union/subtract composition in `compose_piece_solid`; symmetric-across-seam pose with axis=binormal + lateral=split_normal; mesh-CSG cylinder transforms removed for cup-pin (kept for S6 plug-anchor + S7 pour-gate); procedure.rs prose updated for truncated-pyramid vocabulary; cf-cast-cli only needed surface-level touch-up (PinSpec::iter1 constructor preserved, no TOML field overrides exposed); 8 new + 5 deleted/migrated tests in registration.rs + spec.rs + piece.rs; iter-1 regen 303 s, all 11 production STLs PASS §R1 at 1 component each (§G-11 #1 cleared) |
| S4 | Plug-floor lock migration (replaces T-bar + shaft + T-slot) | ~500 | `plug.rs` major rewrite; `PlugPinSpec` shrinks (T-bar + shaft fields deleted); ~200 LOC test churn; cup-piece cap-plane-end socket geometry; `platform.stl` blind pocket retired (no shaft penetration) |
| S5 | First-layer chamfer geometry | ~80 | chamfer-band emitted SDF-side per revised §G-9 — folds into S2's PrismaticPin SDF primitive; minimal incremental cost above S2 |
| S6 | `procedure.rs` print-orientation + Bambu A1 target docs | ~120 | per-piece orientation prose, default-settings + Jayo reference recipe, cf-view sanity-check section |
| S7 | Workshop print + caliper calibration | ~50 | clearance + chamfer numeric pins per §G-8; cf-cast-cli iter-1 regen; cf-view smoke gates §G-11 #3 |
| S8 | Cold-read pass-2 polish | ~80 | doc lies, anchors, multi-line-string drift |
| **TOTAL** | | **~3230** | scope inflated by S1 probe (~+550 LOC vs original ~150 estimate — the probe-spike's characterisation surface is richer than the original recon-4 §F-3 spike, since it must lock 5 distinct findings); S2 + S5 LOC drop slightly under §G-12 #2 path (no mesh-CSG MatingTransform variants; chamfer folds into SDF primitive); net delta ~+830 LOC over original estimate |

**Phase ordering:**

S0 ships first (this commit). S1 follows in the next session (probe
spike). S2 is the architectural foundation — must clear before S3
or S4. S3 + S4 are SEQUENTIAL (S3 first because cup-pin is simpler;
S4 builds on S3 patterns). S5 follows S2 / S3 / S4 (chamfer is a
PrismaticPin emission detail). S6 is procedure.rs prose — can
parallelize with S5. S7 is workshop-user-physical (gates §G-11
#3+#4); the arc pauses for the print. S8 is the cold-read close.

**Per-phase test churn estimate:** ~570 LOC test code total per
§G-10. Distributed S2 (120) / S3 (150) / S4 (120) / S5 (80) / S6
(40) / S7 (0) / S8 (60).

**Per-phase paradigm boundary check (per
[[project-cf-cast-sdf-meshcsg-paradigm-boundary]]):** Under the
revised §G-12 #2 path (post-probe-spike), S2's PrismaticPin lives in
SDF, not mesh-CSG. S3 + S4 cup-pin + plug-floor lock are SDF-union
compositions (recon-4 §F-3 pattern). S5 chamfer band is SDF-side
(folds into S2). The PrismaticPin is now in the SAME column as the
recon-4 funnel-nipple revert and the cup pour-gate-carve (SDF for
bulk-welded features). The cup pour-gate-carve mesh-CSG transform
(`CylinderParent` + `MatingTransform::SubtractCylinder`) is the ONLY
remaining mesh-CSG mating-feature surface after this arc lands —
fine-feature mesh-CSG is now ONLY used where it doesn't paradigm-
boundary-cross.

**Estimated total wall-clock:** 5-7 sessions for S1-S8 default path.
+1-3 sessions if bail-out branches trigger.

> **Decision §G-13.** **8-phase implementation arc S1-S8, ~2400 LOC,
> ~5-7 wall-clock sessions for the default path.** Single working
> branch `dev` per [[feedback-omnibus-pr-single-branch]]; one PR at
> arc close. Each phase gets cold-read pass-2 polish. S7 is the
> workshop-user-physical pause.

## Bail-out branches for recon-1 (recursive — what does recon-2 look like?)

Recon-1 (this doc) commits decisions §G-1 through §G-6 from the
2026-05-24 design chat. If the §G-7 / §G-8 / §G-9 probe-spike session
falsifies a locked decision, that triggers a recon-2:

- **§G-7 falsifies §G-2 (PrismaticPin manifold3d-pathological):**
  bail-out §G-12 #1 + #2 may suffice OR a recon-2 may be needed to
  reframe the primitive (e.g., chamfered cylinder + flat-tipped
  cylinder hybrid).
- **§G-9 falsifies §G-6 (chamfer band breaks topology):** bail-out
  §G-12 #3 (slicer chamfer); no recon-2 needed.
- **S7 workshop-physical print FAILS the §G-11 #4 gate** (geometry
  wrong, not parameter wrong): bookmark + recon-2 with empirical
  failure data. The §G-1 / §G-2 architectural choices may need
  rethinking.

The canonical pattern (per the registration-pin + seam-face arcs):
**bookmark → recon-1 → probe-spike → implementation S1-S8 →
workshop-physical S7-pause → close**. Recon-2 fires only if S7
empirics falsify recon-1's framing.

## Memory + cross-refs

- `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` — recon-4 (P) decision
  doc. Established the SDF/MC ↔ mesh-CSG paradigm boundary framework
  that §G-7 BRANCH B/C bail-outs rely on.
- `docs/CF_CAST_REGISTRATION_PIN_DISCONNECTION_RECON_PLAN_V2.md` —
  recon-3 (α) decision doc. Established the §R1 connectivity-
  invariant inspector pattern carried through in §G-11.
- `docs/CF_CAST_MATING_FEATURES_RECON.md` — S0...S8 master plan for
  the cylinder primitives. This recon §G-5 supersedes the
  `CylinderParent` shared-primitive invariant from that plan.
- `design/cf-cast/src/mesh_csg.rs` — `CylinderParent` + `CylinderParams`
  + `MatingTransform` live here. §G-13 S2 adds `PrismaticPin` types
  here; S5 adds chamfer emission. `MatingTransform::SubtractCylinder`
  STAYS for the cup pour-gate carve; `MatingTransform::UnionCylinder`
  becomes unused and is candidate-for-deletion in S8.
- `design/cf-cast/src/registration.rs` — S5 cup-pin code. §G-13 S3
  rewrites against `PrismaticPin`. `PinSpec` → `PrismaticPinSpec`.
- `design/cf-cast/src/plug.rs` — S6 T-bar + stem + T-slot + plug-
  shaft code. §G-13 S4 deletes most of this; `PlugPinSpec` shrinks.
- `design/cf-cast/src/procedure.rs` — workshop prose. §G-13 S6 adds
  per-piece print-orientation + Bambu A1 + Jayo target.
- `design/cf-cast-cli/` — TOML cross-field validators + workshop-
  fixture integration tests. §G-13 S3 updates the pin field
  validator; S4 simplifies the plug field validator (fewer fields).
- [[project-cf-cast-fdm-friendly-geometry-arc]] — upcoming-arc
  project memory entry written 2026-05-24 before this recon.
  Now reflects locked decisions §G-1 – §G-6.
- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — load-bearing
  for §G-7 / §G-12 bail-out reasoning.
- [[project-cf-cast-mating-features-s5-registration-pins]] — S5
  predecessor; §G-13 S3 supersedes the pin code but keeps the
  fit-invariant test pattern.
- [[project-cf-cast-mating-features-s6-t-bar]] — S6 predecessor;
  §G-13 S4 deletes the T-bar + stem + T-slot + plug-shaft surfaces.
  Three-piece shared-primitive invariant dissolves (replaced by
  2-piece plug-lock + 2-piece cup-pin).
- [[project-cf-cast-mating-features-s7-funnel-pour-gate]] — S7
  predecessor; OUT OF SCOPE for this recon (funnel-nipple stays).
- [[feedback-math-verify-geometric-contracts]] — §G-10 bit-precise
  fit invariant follows this rule.
- [[feedback-cold-read-two-passes-for-non-trivial-diffs]] — every
  S-phase commit gets cold-read pass-2.
- [[feedback-omnibus-pr-single-branch]] — all phases on `dev`; one
  PR at arc-close.
- [[feedback-bookmark-when-surface-levers-exhaust]] — design-chat
  preceded this recon doc (the bookmark-equivalent).
- [[feedback-implement-measure-revert-pattern]] — probe-spike
  before implementation; spike data drives recon-2 if it triggers.
- [[feedback-diagnostic-signal-vs-workshop-failure]] — §G-11 #3 +
  #4 are workshop-user-physical gates, load-bearing not advisory.
- [[feedback-autonomous-architecture]] — recon scaffolds autonomously;
  workshop-physical gates pause the arc per §G-11.

## Status log

- **2026-05-24 — Recon-1 scaffold drafted.** Locked decisions §G-1 –
  §G-6 from the 2026-05-24 design chat: plug retention =
  truncated-pyramid press-fit (T-bar mechanism DELETED); cup-pin
  shape = trapezoidal / truncated-pyramid (cylinder DELETED);
  target FDM floor = Bambu A1 + default + Jayo (clearance budget
  ~0.25-0.35 mm); print orientation locked per piece in procedure.rs;
  `PrismaticPin` primitive replaces `CylinderParent` for mating
  features (`CylinderParent` retained for cup pour-gate carve);
  parameter envelope typed-ranged. Deferred §G-7 / §G-9 manifold3d
  feasibility probes to the next session (probe-spike). §G-8 numeric
  values deferred to S7 workshop-physical calibration. §G-13
  implementation arc estimated at 8 phases / ~2400 LOC / 5-7 sessions
  default. Workshop iter-3 print BLOCKED until implementation arc
  completes.
- **2026-05-24 — Cold-read pass-2 polish.** Fixed plug print
  orientation geometric impossibility in §G-4: cap-plane-face-down
  would protrude the pyramid INTO the bed (impossible for FDM).
  Plug print orientation deferred to S4 with constraint that
  cap-plane-down is invalid; preferred dome-end-on-bed (cap-plane
  up, pyramid up, requires brim for dome contact). Cascade fixes
  to §G-6 + §G-9: first-layer chamfer is load-bearing for the
  CUP-PIN (bed-adjacent base under cup-piece print orientation),
  but the plug-pyramid does NOT have first-layer issue under the
  preferred plug orientation (pyramid at top of print) — its
  chamfer becomes OPTIONAL (lead-in alignment use, not FDM
  elephant-foot use). §G-11 STL count noted as possibly dropping
  from 11 to 10 if platform.stl retires under §G-4. §G-13 S0 LOC
  updated to ~1000 (after pass-1 + pass-2).
- **2026-05-24 — Cold-read pass-1 polish.** Fixed cap-plane vs
  dome-end terminology error throughout §G-1 / §G-4 / §G-5 / §G-13.
  The plug-floor lock is at the **cap-plane end** of the plug (the
  open / cuff end of the device, where today's T-bar + shaft
  mechanism exits — see `plug.rs:1-46` for cap-plane = "pinned floor"
  of the plug), NOT the dome end (which is the closed / toe end of
  the device, where the V-pour-gate sits per `procedure.rs:418-419`).
  Per `procedure.rs` the pour orientation is **+Z up with dome end
  at top, cap plane at bottom** — the user's "floor of the mold" =
  cap-plane end interior. Also clarified that today's T-bar is
  **captive insertion** (per `procedure.rs:646-648`), NOT a twist-
  lock — the user's "twist lock" framing was an earlier-conversation
  mis-framing by Claude; the user's preference for press-stop over
  rotational mechanisms stands regardless. Workshop motion is mold-
  assembly press-fit (lower plug into open cup half, close second
  half) — NOT "drop through pour-gate opening" (plug is much wider
  than pour gate, geometrically impossible). `platform.stl`'s blind
  pocket retires under §G-1 (no shaft penetration through cup wall).
  S6 three-piece shared-primitive invariant CARRIES THROUGH (not
  "dissolves") if the truncated-pyramid socket is seam-plane-
  symmetric (each cup half carves half the socket, same pattern as
  today's half-T-slot). 2-piece vs 3-piece decision deferred to S2 /
  S4 implementation.
- **2026-05-24 — S1 probe-spike SHIPPED.** Probe-spike session lands
  `design/cf-cast/tests/g7_g9_prismatic_pin_probe.rs` (8 characterisation
  tests, all PASS, 187 cf-cast unit + 8 probe tests / clippy
  `-D warnings` / fmt clean, ~58 s release-test wall-clock). §G-7
  BRANCH determination: **B + C BOTH fire** under the §G-13 default
  mesh-CSG path (mesh-CSG union of a `Manifold::hull_pts` truncated-
  pyramid PrismaticPin onto an SDF→MC half-sphere shell host adds 2
  components AND introduces a `SelfIntersecting` F4 Critical at the
  boolean junction; symptoms reproduce at the over-resolved 1 mm
  control cell size, so failure is NOT pure MC quantization). §G-9
  characterisation: chamfer-INDEPENDENT (all values in {0.0, 0.4,
  0.6, 0.8, 1.0 mm} fail identically). §G-12 #2 bail-out (pin lives
  entirely in SDF, recon-4 §F-3 pattern) PASSES BRANCH-A on the same
  fixture — 1 component, no new F4 Critical types. **Decision: §G-12
  #2 PICKED** as the implementation path. The recon-4 §F-3 docstring's
  inference that mesh-CSG-union pin on a clean 1-component half-shell
  host is "at least as robust topologically" as SDF-union pin was an
  untested extrapolation; the probe falsifies it for `hull_pts`
  truncated-pyramid primitives. §G-13 implementation scope revised:
  S2 PrismaticPin primitive is SDF-side (no `MatingTransform::*PrismaticPin`
  variants); S5 chamfer band folds into S2 (SDF-side emission); LOC
  delta approximately neutral vs original mesh-CSG-default estimate.
  Workshop iter-3 print STILL BLOCKED until S2-S8 implementation
  completes; the empirical evidence has narrowed the implementation
  architecture but not unblocked the print.
- **2026-05-24 — S3 cup-piece registration pin migration SHIPPED.**
  `design/cf-cast/src/registration.rs` rewrite: cup-pin primitive
  migrated from mesh-CSG cylinders (`MatingTransform::UnionCylinder` /
  `SubtractCylinder` consuming a shared `CylinderParent`) to SDF-side
  `PrismaticPin` composition (`Solid::union` for pin / `Solid::subtract`
  for socket, pre-MC in `piece::compose_piece_solid`) per the §G-12 #2
  architecture picked by S1 probe-spike. `PinSpec` reshaped to wrap
  `PrismaticPinSpec::cup_pin_default()` + arc_fractions; field-level
  fixture overrides on cf-cast-cli intentionally not exposed (only the
  `[registration_pins].enabled` toggle is wired through the TOML —
  per-pin geometry is locked to the §G-6 / §G-8 defaults pending
  workshop-physical calibration in S7). `build_registration_transforms`
  renamed to `build_registration_sdf_ops` returning `Vec<Solid>`;
  `compose_piece_solid` now unions/subtracts per `PieceSide`. Pose
  derivation reuses the existing `surface_distance_along_ray`
  annulus-midpoint geometry; `axis_unit = +binormal`,
  `lateral_unit = +split_normal` (orthogonal-to-binormal by ribbon
  construction, deterministic for square-base pins where the rotation
  about `axis_unit` is geometrically immaterial). The chamfer band's
  bed-adjacency under §G-4 print orientation is unresolved at the SDF
  layer — flagged in the module docstring for §G-6 procedure.rs
  reconciliation in S6 (the symmetric-across-seam pose places the
  chamfer at the deepest-in-cup-wall axial end, not at the seam face;
  reconciling this requires either flipping the pose convention or
  revisiting the §G-4 cup print orientation). 201 cf-cast unit + 8
  probe + 48 cf-cast-cli + 7 integration tests / clippy `-D warnings` /
  fmt clean. cf-cast-cli iter-1 regen 303 s; **all 11 production STLs
  (6 cup pieces + 3 plugs + funnel + platform) PASS §R1 inspector at
  1 component each under `INSPECT_STL_R1=1`** — §G-11 #1 (mechanical
  unblock gate for workshop iter-3) CLEARED. §G-11 #3 (cf-view smoke)
  + #4 (Bambu A1 print) remain workshop-user-physical pauses. S2
  probe characterisation extrapolates to production: the §G-12 #2
  SDF-union path holds at production scale + curvature, not just on
  the synthetic half-sphere fixture. `MatingTransform::UnionCylinder`
  is now confirmed UNUSED in the cup-pin path; deletion-of-the-variant
  deferred to S8 cold-read per §G-5. Successor session: S4 plug-floor
  lock migration per §G-13 S4 (~500 LOC; T-bar + shaft + T-slot
  mechanism DELETED, replaced by truncated-pyramid press-fit against
  mold cap-plane floor; platform.stl blind pocket retires per §G-1 —
  no shaft penetration).
