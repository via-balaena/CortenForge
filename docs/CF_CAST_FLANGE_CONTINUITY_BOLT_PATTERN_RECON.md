# cf-cast §F flange-continuity + §B M5 bolt-pattern recon (2026-05-26)

**Status:** scaffold. Triggered by workshop user's 2026-05-26
session-end scope-lock
([[project-cf-cast-flange-continuity-plus-bolt-pattern-scope-lock]]):

> *"I want engine-valve-cover-style M5 through-bolts around the
> flange perimeter for iter-1 clamping. M5 both sides with washer +
> nut, 5.5 mm clearance, ~6 bolts per half at ~60 mm spacing
> (~12 around the closed mold). No counter-bore, no heat-set
> inserts — overkill."*

The locked bolt-pattern feature requires a CONTINUOUS flange around
the body perimeter (a bolt hole at a continuity gap = useless; a
hole near a gap = stress-riser crack initiator under torque). That
reframes the prior [[project-cf-cast-flange-perimeter-continuity-bookmark]]
"iter NOT blocked" verdict — flange-continuity is no longer just a
sealing concern, it's a hard prerequisite for the bolt pattern.

This recon covers TWO arcs that ship pre-iter-1-print:

- **§F flange-continuity fix** — replace `FlangeSdf::eval`'s 3D-
  projected `body_dist` with a 2D point-to-silhouette distance so
  the flange ring closes around body concavities.
- **§B M5 bolt-pattern feature** — new mesh-CSG `SubtractCylinder`-
  per-bolt emission around the now-continuous flange perimeter.

Sequencing: **§F first PR** (correctness fix; independent merge
value); **§B second PR** (feature; builds on §F's continuous flange).

**Picked architectural directions:**

- §F: **Candidate A — 2D silhouette polyline + point-to-polyline
  distance.** Sample body SDF on a 2D grid at the seam plane,
  marching-squares to a closed polyline, replace `body.evaluate(P_proj)`
  with `silhouette.distance_to(P.x, P.z)` in FlangeSdf.
- §B: **SubtractCylinder per bolt, axially aligned with binormal,
  centered on seam plane.** Pattern mirrors `build_pour_gate_transforms`
  (`pour.rs`) — same paradigm-safe POST-MC mesh-CSG primitive that
  the cup-piece halfspace cut already gracefully handles per-side.

## Cross-arc decision recap

The flange-continuity bug is **NOT a §Q-1 regression**: pre-§Q-1
the same `FlangeSdf::eval` code at `flange.rs:212-219` shipped the
same 3D-projected body_dist. It only became workshop-visible at
2 mm cells (post-§Q-1 unblock of finer-cell regen). The synthetic
cylinder fixture used by the original seam-flange S1 has a fully-
convex cross-section in the seam plane, so the 3D-vs-2D distance
discrepancy never fired in tests. Production sock-over-capsule
scans have concave seam-plane cross-sections (heel pocket, foot
arch silhouette) where the discrepancy IS visible.

The §F arc closes this gap with a load-bearing C-shape test fixture
per [[feedback-load-bearing-test-fixtures]] — fixture geometry
that surfaces the bug the convex cylinder missed.

## Out of scope

- **Cup-wall outer MC stair-stepping** — that's §SMD-4 S3 territory
  ([[project-cf-cast-scan-mesh-direct-recon]]); workshop-user-gated,
  lower priority than §F + §B per the scope-lock memo. §F + §B
  compose cleanly with future §SMD-4 S3 work — they touch the
  flange SDF + mating-transforms Vec, not the cup-wall outer mesh.
- **Counter-bored or recessed bolt heads** — locked OUT by workshop
  user ("no counter-bore"). Workshop user can add later via a
  separate `MatingTransform::SubtractCylinder` recess if needed.
- **Heat-set inserts** — locked OUT ("overkill"). Through-bolts +
  nuts both sides is the workshop user's chosen mechanical solution.
- **Flange-thickness override per bolt site** — the locked spec
  uses the standard `FlangeSpec::flange_thickness_m = 4 mm` per
  half; bolt-head bearing area is the standard DIN 125 washer's
  ~10 mm OD against this 4 mm flange thickness. No per-site
  thickening primitive.
- **§Q-1 cup-wall body-tracking shell composition** — unchanged.
  §F modifies FlangeSdf; the shell SDF + halfspace + pour-gate +
  registration + plug-lock-socket emission paths in
  `compose_piece_solid` are untouched.

## §F-0 / §B-0 Why this recon (workshop empirical signal)

**§F empirical signal** — cf-view smoke on
`~/scans/cast_iter1_q1_phase1_2mm_full/` after §Q-1 fix shipped
(2026-05-26):

> *"flange isn't all the way around"* — flange ring traces the
> body's seam-plane perimeter cleanly through convex regions but
> drops out where the perimeter is concave (back of foot / heel
> region).

**§B empirical signal** — workshop user (same session-end):

> *"I want to clamp this with bolts around the perimeter like an
> engine valve cover, so I can apply even pressure on the gasket
> without dancing rubber bands or C-clamps."*

The two findings are coupled: bolt placement on a flange with gaps
is geometrically unsafe (perimeter gap = no material for the bolt
to bear against; near-gap = crack initiator). §F is the prerequisite
correctness fix; §B is the workshop-clamping feature that gates the
combined arc as iter-1-print blocker.

## §F-1 Root cause analysis (recap from bookmark)

`FlangeSdf::eval` at `design/cf-cast/src/flange.rs:212-219`:

```rust
fn eval(&self, p: Point3<f64>) -> f64 {
    let projected = Point3::new(p.x, self.seam_plane_y, p.z);
    let body_dist = self.body.evaluate(&projected);
    let outer = body_dist - self.flange_width_m;
    let inner = self.flange_inner_offset_m - body_dist;
    let vertical = (p.y - self.seam_plane_y).abs() - self.flange_thickness_m;
    outer.max(inner).max(vertical)
}
```

`body.evaluate(projected)` returns the **3D** signed distance from
the projected point to the nearest body surface. The flange uses
this as a proxy for **2D in-plane distance to the body's silhouette
curve at Y = seam_plane_y**. The proxy works in convex regions
(closest 3D body surface IS the body's lateral surface at the same
Z, so 3D distance ≈ 2D in-plane distance). It fails in concave
regions: the closest 3D body surface might be a body cap end or a
curved-back inflection point that's > `flange_width_m` from the
query point even when the query point is at the silhouette
perimeter on the seam plane → `outer = body_dist - flange_width_m > 0`
→ no flange material emitted.

The flange ring traces the perimeter where the proxy holds (convex
sections, ~70-80 % of a sock-shape perimeter empirically) and drops
out where the proxy fails (concave sections, ~20-30 %).

## §F-2 Algorithm pick

Three candidates from the bookmark. Recapped here with the workshop
user's "build from upstream/foundational layer" directive applied
(see [[project-cf-cast-geometry-crispness-q1-fix]] +
[[project-cf-cast-geometry-crispness-q5-fix]] for the pattern: walk
the dependency chain until reaching the layer where the bug
originates).

### Candidate A: 2D silhouette polyline + point-to-polyline distance (PICKED)

1. At `build_flange_solid` time, sample `layer_body.evaluate` on a
   2D grid in (X, Z) at Y = `seam_plane_y`; run **marching squares**
   to produce one (or more) closed polylines tracing the SDF zero-
   crossing. This IS the body's seam-plane silhouette.
2. Cache the polyline inside `FlangeSdf` (replaces the `body: Solid`
   field).
3. `FlangeSdf::eval` does 2D point-to-polyline distance:
   `body_dist = silhouette.signed_distance_to(P.x, P.z)`
   — outside-positive convention (matches the body SDF's sign).

**Pros:**
- Exact 2D in-plane distance. Algorithm is well-known + bounded
  cost (point-to-polyline is O(n_segments) per `FlangeSdf::eval`
  call; marching-squares precompute is O(grid_cells) one-shot per
  layer).
- No external dependency. Marching squares + segment distance are
  ~150 LOC total.
- Walks the dependency chain upstream to the actual originating
  layer (FlangeSdf's distance metric).
- Determinism contract straightforward: same body + same grid step
  → bit-identical polyline → bit-identical distance evaluations.

**Cons:**
- 2D grid step is a new tunable. Resolution trade-off mirrors the
  main MC cell-size trade-off, but the polyline is sampled ONCE per
  layer not per MC vertex so cost stays cheap. Pick 0.5 mm grid step
  (matches the §F-flange's bound on workshop-visible features per
  [[project-cf-cast-geometry-crispness-s0]]).
- Polyline doesn't compose with `cf_design::Sdf` — it's a 2D
  precomputed structure stored inside FlangeSdf. Fine — FlangeSdf
  is already opaque to outside consumers; the field swap is
  internal.

### Candidate B: per-Z body cross-section radius lookup (REJECTED)

For each Z in body extent, precompute a 1D radius value. Replace
`body.evaluate(projected)` with `body_dist = |P.x - sign(P.x) * radius(P.z)|`.

**Rejected because:** assumes the body is X-Z symmetric (or that
the flange only needs distance to one lateral half). Production
sock-over-capsule scans are **NOT** axisymmetric in the seam plane —
the heel pocket extends asymmetrically along Z. A 1D-per-Z lookup
collapses the seam-plane silhouette into a function `radius(Z)`,
losing concavity information by construction.

### Candidate C: medial-axis offsetting library (REJECTED for iter-1)

Use clipper / polylabel / a 2D offset polygon library to compute
the body's seam-plane offset polygon at `flange_width_m`.

**Rejected because:**
- New external dependency. Workspace currently has no 2D-offset
  library; adding one is a separate decision.
- Candidate A's marching-squares approach gives the same
  geometric answer for the flange's needs (point-to-silhouette
  distance) without the dependency.
- Workshop's "build from upstream/foundational layer" directive
  prefers in-house algorithm walks over new external deps when the
  in-house option is bounded LOC.

Bookmarked: if a future arc surfaces additional 2D-offset needs
(e.g., custom seam-plane manipulations beyond flange + bolt-pattern),
revisit Candidate C as a workspace-wide 2D-CSG utility.

### Why A wins

Walks the dependency chain to the originating layer (FlangeSdf's
distance metric). Bounded LOC, no external dependency, exact
mathematical answer for the flange's needs. The C-shape test
fixture per §F-3 closes the test-coverage gap that let the bug
ship past S1's synthetic cylinder.

## §F-3 Implementation surface

### New 2D silhouette polyline type

New module `design/cf-cast/src/silhouette_2d.rs` (~150 LOC):

```rust
pub(crate) struct Silhouette2d {
    /// Closed polyline(s) — one per topologically-disconnected
    /// silhouette component (typically 1 for workshop sock scans).
    /// Each polyline's vertices are (X, Z) at Y = seam_plane_y.
    polylines: Vec<Vec<Point2<f64>>>,
}

impl Silhouette2d {
    /// Build by sampling `body.evaluate` on a 2D grid in (X, Z) at
    /// the given seam plane Y, then running marching squares to
    /// extract the SDF zero-crossing polylines.
    ///
    /// `bounds_2d_min_xz` / `bounds_2d_max_xz` use 2D `Point2<f64>`
    /// (no `Aabb2d` type exists in the workspace; cf-design re-exports
    /// 3D `Aabb` from cf-geometry only). Derive from the layer body's
    /// 3D AABB by projecting (X, Z) extents.
    pub fn from_body_at_seam_plane(
        body: &Solid,
        seam_plane_y: f64,
        bounds_2d_min_xz: Point2<f64>,
        bounds_2d_max_xz: Point2<f64>,
        grid_step_m: f64,
    ) -> Self { ... }

    /// 2D point-to-silhouette **signed** distance.
    /// Outside-positive (matches the body SDF's sign convention).
    pub fn signed_distance_to(&self, x: f64, z: f64) -> f64 { ... }
}
```

Marching-squares cases: standard 16-case lookup; linear interpolation
along grid edges crossed by SDF zero. Polyline closure invariant
verified by a per-polyline assertion (each polyline forms a closed
loop; topological disconnection produces multiple closed loops).

**Sign convention for `signed_distance_to`:**
- Outside silhouette (in flange-eligible region): positive distance.
- Inside silhouette (under body): negative distance.

Computed via:
1. Unsigned point-to-polyline distance (closest segment perpendicular
   distance, or vertex distance for convex Voronoi regions).
2. Sign from even/odd ray-cast test against the polyline (point-in-
   polygon → inside; outside → outside). Standard 2D inclusion test;
   O(n) per call alongside the distance computation.

### FlangeSdf field swap

`FlangeSdf` (`flange.rs:202-209`) changes:

```rust
struct FlangeSdf {
    silhouette: Silhouette2d,  // was: body: Solid
    seam_plane_y: f64,
    flange_width_m: f64,
    flange_thickness_m: f64,
    flange_inner_offset_m: f64,
}

impl Sdf for FlangeSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let body_dist = self.silhouette.signed_distance_to(p.x, p.z);
        let outer = body_dist - self.flange_width_m;
        let inner = self.flange_inner_offset_m - body_dist;
        let vertical = (p.y - self.seam_plane_y).abs() - self.flange_thickness_m;
        outer.max(inner).max(vertical)
    }
    // grad stays as the +Y arbitrary pick — Solid::from_sdf bridges
    // via FieldNode::UserFn with finite differences.
}
```

The `grad` stub is unchanged (the `max()` composition's analytical
gradient is multivalued at facet boundaries, and `Solid::from_sdf`
uses finite differences regardless).

### `build_flange_solid` signature

```rust
pub(crate) fn build_flange_solid(
    layer_body: &Solid,
    ribbon: &Ribbon,
    spec: &FlangeSpec,
    bounds: Aabb,
) -> Solid
```

Internally:
1. Compute `(bounds_min_xz, bounds_max_xz)` from the 3D `bounds`
   (project to XZ at seam_plane_y).
2. Build `Silhouette2d::from_body_at_seam_plane(layer_body,
   seam_plane_y, bounds_min_xz, bounds_max_xz,
   SILHOUETTE_GRID_STEP_M)`.
3. Construct `FlangeSdf` with the polyline (replaces the prior
   `body: Solid` clone-into-the-SDF pattern; Solid's
   `from_sdf`-via-`FieldNode::UserFn` still holds the FlangeSdf).

`SILHOUETTE_GRID_STEP_M` constant: **0.5 mm**, named `pub(crate)
const`. Picked to give sub-mm polyline accuracy at acceptable
precompute cost (a 200 mm × 200 mm grid at 0.5 mm = 160k samples
× body.evaluate cost; one-shot per layer ≈ < 1 s wall-clock for
the production sock scan).

### Determinism contract

- Same body SDF + same `bounds_2d` + same `grid_step_m` → bit-
  identical polyline (marching squares is deterministic given the
  inputs).
- Same polyline → bit-identical `signed_distance_to` evaluations
  (the algorithm is closed-form per segment).
- `FlangeSdf` is already `#[derive(Debug, Clone)]`; the field swap
  makes Clone CHEAPER (polyline is `Vec<Point2<f64>>` — no Solid
  graph to deep-clone for each `FlangeSdf::clone()` call). Per
  current production behavior `FieldNode::UserFn` only clones the
  SDF struct once at `Solid::from_sdf` time, so Clone cost isn't
  on the hot path.

## §B-1 Bolt placement algorithm

Two candidate placement algorithms.

### Candidate (i): Equal arc-length around silhouette curve (PICKED)

1. After `Silhouette2d` is built (re-used from §F), compute the
   polyline's total arc length.
2. Derive bolt count from target spacing:
   `n_bolts = round(arc_length / target_spacing_m).max(min_bolts)`.
3. Place bolt centers at equal arc-length intervals along the
   polyline, OUTBOARD-offset by `(flange_inner_offset_m + flange_width_m)/2`
   from each polyline vertex (= midpoint of the flange material's
   radial width, the natural bearing zone).
4. Each bolt's axis = ±binormal (perpendicular to seam plane).

**Pros:**
- Even spacing along the actual perimeter (matches the workshop
  user's "~60 mm spacing" intent regardless of perimeter shape).
- Re-uses the §F silhouette polyline → no new geometric machinery.
- Deterministic: same polyline → same bolt positions.

### Candidate (ii): Equal angular spacing around silhouette centroid (REJECTED)

Compute the silhouette's centroid, then place bolts at equal angles
from centroid.

**Rejected because:**
- For non-circular perimeters (which sock scans are NOT — they're
  elongated foot-shapes), equal-angular spacing is empirically
  uneven (bolts cluster along the short axis, sparse along the
  long axis). Workshop's "~60 mm spacing" requirement isn't met.
- More complex to constrain bolt count from spacing target.

### Bolt-count derivation

Workshop user: "~6 bolts per half / ~12 around the closed mold at
~60 mm spacing." Refine empirically:

```rust
const BOLT_SPACING_TARGET_M: f64 = 0.060;  // 60 mm
const BOLT_COUNT_MIN: usize = 6;            // safety floor (3 per half)
const BOLT_COUNT_MAX: usize = 24;           // safety ceiling
let n_bolts = (arc_length_m / BOLT_SPACING_TARGET_M)
    .round()
    .clamp(BOLT_COUNT_MIN as f64, BOLT_COUNT_MAX as f64) as usize;
```

Production sock scan: cf-scan-prep `~/scans/sock_over_capsule.prep.toml`
estimates seam-plane perimeter ~300-400 mm → 5-7 bolts at 60 mm
spacing. Workshop user's "~12 around closed mold" implies 6/half,
matching the upper end of this range. The `BOLT_COUNT_MIN = 6`
floor ensures workshop expectations are met even on smaller-than-
expected perimeters.

### Bolt-pose origin direction

Polyline traversal direction is deterministic by marching-squares
output ordering. The **OUTBOARD normal** at each polyline sample is
the 2D outward direction (perpendicular to the local tangent,
pointing away from silhouette interior). Bolt center sits at:

```text
bolt_center_xz = polyline_sample + outboard_normal_2d * radial_offset_m
bolt_center_y = seam_plane_y
where radial_offset_m = (flange_inner_offset_m + flange_width_m) / 2
                       = (0.002 + 0.015) / 2 = 0.0085 m
```

The bolt's lateral position is in the flange's radial midpoint —
not against the inboard edge (which is the gasket-channel boundary)
and not against the outboard edge (which is the clamp grip's
outermost reach). This gives **3.75 mm of flange material on each
side of the 5.5 mm hole** (= **0.68× hole-diameter**). The standard
plastic-fastener rule-of-thumb is wall_thickness ≥ 1× hole-Ø;
**0.68× is below the conservative floor**. See §B-4 for the full
trade-off + flange_width default decision required.

## §B-2 Bolt geometry

Locked workshop user spec (verbatim):

| Parameter | Value | Source |
|---|---|---|
| Fastener | M5 through-bolt | Workshop user |
| Clearance hole | 5.5 mm Ø (M5 close-fit per ISO 273) | Workshop user |
| Bolt count | ~6/half, ~12 around closed mold | Workshop user |
| Spacing | ~60 mm along perimeter | Workshop user |
| Counter-bore | None | Workshop user |
| Heat-set inserts | None | Workshop user |
| Washer | DIN 125 standard, ~10 mm OD | Inferred from spec |
| Placement | Outboard of seam line | Workshop user |

### MatingTransform emission

Each bolt position emits one `MatingTransform::SubtractCylinder`:

```rust
MatingTransform::SubtractCylinder {
    params: CylinderParams {
        parent: CylinderParent {
            center_m: Point3::new(bolt_center_x, seam_plane_y, bolt_center_z),
            axis: ribbon_binormal_at_centerline_sample,  // ±binormal direction
            half_length_m: BOLT_HOLE_HALF_LENGTH_M,
        },
        radius_m: BOLT_HOLE_RADIUS_M,  // 5.5 / 2 = 2.75 mm
        segments: BOLT_HOLE_SEGMENTS,   // 32 (matches POUR_GATE_SEGMENTS)
    },
}
```

Constants:
- `BOLT_HOLE_RADIUS_M = 0.00275` (= 5.5 mm Ø / 2).
- `BOLT_HOLE_SEGMENTS = 32` (matches pour-gate's facet count;
  chord error at 2.75 mm radius = `r(1 - cos(π/32))` ≈ 13 µm, well
  below FDM bead width).
- `BOLT_HOLE_HALF_LENGTH_M = 0.020` (40 mm total length —
  comfortably spans 2 × 4 mm flange thickness + workshop margin
  for misalignment + bolt-head clearance; cup-piece halfspace
  cut on each piece keeps only the relevant side of the cylinder).

### Why SubtractCylinder, not SDF carve

The flange is a thin (4 mm per half) slab; bolt-hole geometry is
~6 mm Ø through-hole. At cf-cast's default 3 mm MC cells the bolt
features would be sub-2-cell → eaten by quantization (same failure
mode that killed the SDF-side cup-pin per
[[feedback-read-prior-arc-memory-before-architectural-decisions]]).
POST-MC mesh-CSG keeps bolt-hole geometry at primitive-native
resolution (32-segment cylinder).

Paradigm-boundary placement: **CONTAINED**. The bolt cylinder
extends 40 mm along binormal, comfortably past both flange faces
(each 4 mm thick); the cylinder's lateral surface intersects only
the flange's planar faces (perpendicular plane × cylinder = clean
circle intersection). No face-coincident shared geometry → no
WELDED-TO-BULK paradigm issue.

### Per-piece emission

Each cup-piece emits the FULL bolt-hole cylinder Vec from its
`compose_piece_solid` call. Pattern matches `build_pour_gate_transforms`
(pour.rs):

> *"the post-MC mesh-CSG subtract of the full cylinder is a no-op
> for the portion outside the half-shell. Each piece keeps only
> its own side's half-cylinder carve."*

The bolt cylinder is centered on the seam plane and axially aligned
with binormal, so each cup-piece's halfspace cut keeps only its own
half of the cylinder carve. Both pieces' flanges get matching
bolt-hole halves; the closed mold has full through-holes.

## §B-3 Collision handling

Three potential collision sources for bolt positions on the flange:

### Pour-gate axes

Pour-gate ([[pour.rs]]) places one or two cylindrical channels
splaying from the dome-end centerline endpoint at ±30° from the
local outward axis along ±binormal. Pour leg lives on Positive
side (`+binormal` half); vent leg on Negative side (`-binormal`).

**Collision risk:** the pour-gate cylinders punch through the
cup-piece outer surface near the dome end. A bolt position within
~10 mm of a pour-leg surface intersection would have the bolt
cylinder partially overlap the pour-leg cylinder → manifold3d
mesh-CSG handles boolean overlap deterministically (the result is
the union of the two carves), but workshop-visually the bolt hole
loses material integrity (the bolt would have nothing to bear
against on its washer side).

**Mitigation (PICKED):** at bolt placement time, compute each
bolt's seam-plane (X, Z) position; reject any bolt whose seam-plane
distance to either pour-leg seam-plane projection is below
`MIN_BOLT_TO_POUR_GATE_M`. Redistribute the rejected bolts'
arc-length budget to the remaining positions (equal arc-length
re-spacing on the modified polyline).

Constant: `MIN_BOLT_TO_POUR_GATE_M = 0.015` (15 mm = bolt-hole
half-width 2.75 mm + pour-leg radius 5 mm + 7.25 mm clearance
margin).

### Registration pins

Registration pins ([[registration.rs]]) sit in the **cup-wall
annulus** at body-perimeter + `wall_thickness_m / 2` along
±split_normal. Pin centers are inboard of the body's lateral
surface (in cup-wall material), NOT on the flange.

**Collision risk:** none. Pins live ~5-10 mm inboard of the body
silhouette; bolt holes live ~8.5 mm OUTBOARD of the silhouette
(flange radial midpoint). The two feature sets are spatially
disjoint.

### Plug-floor-lock socket

Plug-floor-lock socket ([[plug.rs]] `build_plug_lock_socket_transform`)
is a single truncated-pyramid carve centered on the centerline at
the cap-plane endpoint. Lives in the cup-piece's cavity interior,
NOT on the flange.

**Collision risk:** none. Socket geometry is inboard of the body's
lateral surface; bolt holes are outboard.

### Layer-vs-layer-vs-bolt coordination

Production cf-cast emits **N layer-piece pairs** (typically 3 per
[[project-cf-cast-scan-mesh-direct-s1-1]]). The flange and bolt
pattern are per-layer (each layer's body has its own silhouette
polyline → its own bolt positions). Different layers can have
DIFFERENT bolt counts + positions if their silhouettes differ.

**Open question (see open question 1 below):** should bolt
positions be SHARED across layers (workshop user threads same bolt
through all layers) or per-layer (each layer aligns clamping to its
own perimeter shape)?

**Default pick:** per-layer. Each layer's silhouette polyline drives
its own bolt placement; bolts are short through-bolts within each
layer's flange thickness. Workshop user uses N separate bolt-rings
for N layers. If iter-1 physical test surfaces alignment problems
(e.g., layers can't be stacked without through-bolt registration),
revisit in a follow-up arc.

## §B-4 Flange-width sizing at bolt holes

Workshop user spec: 5.5 mm clearance hole on a flange whose default
`flange_width_m = 0.015` (15 mm).

**Inboard side of hole** (between hole edge and flange's inboard
boundary at body silhouette + 2 mm gasket gap):
- Hole edge at silhouette + 8.5 mm − 2.75 mm = silhouette + 5.75 mm
- Inboard flange edge at silhouette + 2.0 mm (flange_inner_offset_m)
- Inboard wall: 5.75 − 2.0 = **3.75 mm** material between hole and
  gasket channel

**Outboard side of hole** (between hole edge and flange's outboard
boundary at silhouette + 15 mm):
- Hole edge at silhouette + 8.5 mm + 2.75 mm = silhouette + 11.25 mm
- Outboard flange edge at silhouette + 15.0 mm
- Outboard wall: 15.0 − 11.25 = **3.75 mm** material between hole
  and outer edge

Both walls = 3.75 mm = **0.68× bolt-hole-Ø**. The standard plastic-
fastener rule-of-thumb (typical FDM design guidance: Bambu Lab,
Prusa, generic 3D-printing engineering) is:

| Wall thickness vs hole-Ø | Verdict |
|---|---|
| ≥ 1.0× | Safe |
| 0.5×–1.0× | Marginal — workshop should expect occasional crack initiation at over-torque |
| < 0.5× | Unsafe — high probability of cracking |

**0.68× = MARGINAL.** Workshop user's "hand-tighten until firm"
clamping torque (not full M5 spec preload) reduces the risk vs. a
wrench-torqued M5 fastener, but a single over-tightened bolt could
crack the inboard wall (which is closer to the gasket channel —
crack might propagate into the gasket sealing region).

### Decision required (open question 0 below)

Two options the workshop user should pick:

**Option A: Accept marginal walls; ship current `flange_width_m = 15 mm`.**
- Pro: workshop user already validated the 15 mm visual appearance
  via cf-view smoke on `~/scans/cast_iter1_q1_phase1_2mm_full/`;
  no flange-visual change.
- Con: tighter mechanical safety margin; workshop discipline
  required (hand-tighten only; replace cup-piece if a wall cracks).
- Cross-field constraint becomes:
  ```text
  flange_width_m >= flange_inner_offset_m + 2 × bolt_hole_radius_m × 2.36
  # = 0.002 + 5.5 × 2.36 / 1000 m ≈ 0.015 m  (= current default, just barely)
  ```

**Option B: Bump `flange_width_m` to 18.5 mm for safe (1× rule) walls.**
- Pro: each wall becomes 5.5 mm = 1.0× bolt-Ø, safe by rule-of-thumb;
  workshop user can torque to gasket-compression spec without crack
  worry.
- Con: visual change to flange (3.5 mm wider per side; ~25 %
  thicker visual appearance); ~10 % more PLA per cup-piece.
- Cross-field constraint becomes:
  ```text
  flange_width_m >= flange_inner_offset_m + bolt_hole_radius_m × 6
  # = 0.002 + 5.5 × 3 / 1000 m = 0.0185 m
  ```

**Recommended pick:** Option B (bump to 18.5 mm). Mechanical safety
margin matters more than visual flange thickness for iter-1; the
visual change is small (3.5 mm) and the workshop user can drop back
to 15 mm if iter-1 physical test surfaces no cracking concerns.
**Ask the workshop user before §B-S1.**

Both options enforce the cross-field constraint at cf-cast-cli S2
TOML parse:

```text
flange_width_m
    >= flange_inner_offset_m
       + (2 × WALL_THICKNESS_FACTOR + 1) × bolt_hole_radius_m
```

Where `WALL_THICKNESS_FACTOR` is either 0.68 (Option A — match
current marginal default) or 1.0 (Option B — safe rule-of-thumb).
Workshop user picks at §B-S1 kickoff.

## §F+§B Sequencing

**§F first** (PR #1):

- Independent correctness fix; valid merge target without §B.
- Closes [[project-cf-cast-flange-perimeter-continuity-bookmark]].
- Smaller blast radius — only touches FlangeSdf composition.
- C-shape test fixture per §F-T-1 catches future regressions.

**§B second** (PR #2):

- Builds on §F's continuous flange + reuses §F's `Silhouette2d`
  polyline for bolt placement.
- New feature, opt-in via `BoltPatternKind::Pattern(BoltPatternSpec)`
  on the ribbon (matches existing `RegistrationKind` / `PourGateKind`
  / `PlugPinKind` / `GasketKind` / `FlangeKind` patterns).
- Default `BoltPatternKind::None` keeps existing tests green
  without bolt-hole emission.

**Why not one combined PR:**

- Workshop user PR-review bandwidth + cold-read effort scale with
  PR scope. Two focused PRs ≤ one combined PR for review cost.
- §F has independent merge value (closes a workshop-visible
  artifact). Bundling it with §B holds the correctness fix hostage
  to the feature scope.
- Matches the cf-cast project pattern of single-arc PRs ([[project-
  cf-cast-q4-s1-shipped]] + [[project-cf-cast-scan-mesh-direct-s1]]
  + [[project-cf-cast-scan-mesh-direct-s1-1]] all shipped as
  scoped PRs).

## §F+§B API surface

### §F surface

- `flange.rs`: `FlangeSdf` struct field swap (`body: Solid` →
  `silhouette: Silhouette2d`). Public API unchanged (`FlangeSdf`
  is private; `build_flange_solid` + `build_flange_solid_for_side`
  signatures unchanged).
- New private module `silhouette_2d.rs`:
  - `pub(crate) struct Silhouette2d`
  - `pub(crate) fn from_body_at_seam_plane(...)`
  - `pub(crate) fn signed_distance_to(...) -> f64`
- `lib.rs`: `mod silhouette_2d;` (no re-export — internal use only
  by `flange.rs`).

### §B surface

- New `bolt_pattern.rs` module:
  ```rust
  pub struct BoltPatternSpec {
      pub bolt_hole_diameter_m: f64,   // default 0.0055 (5.5 mm M5 clearance)
      pub spacing_target_m: f64,        // default 0.060 (60 mm)
      pub min_count: usize,             // default 6
      pub max_count: usize,             // default 24
      pub min_pour_gate_clearance_m: f64,  // default 0.015 (15 mm)
  }

  pub enum BoltPatternKind {
      None,
      Pattern(BoltPatternSpec),
  }

  pub(crate) fn build_bolt_pattern_transforms(
      ribbon: &Ribbon,
      silhouette: &Silhouette2d,
      spec: &BoltPatternSpec,
  ) -> Vec<MatingTransform>;
  ```
- `ribbon.rs`: add `bolt_pattern: BoltPatternKind` field (default
  `None`); add `Ribbon::with_bolt_pattern` builder method.
- `piece.rs::compose_piece_solid`: emit `build_bolt_pattern_transforms`
  alongside existing pour-gate + registration + plug-lock-socket
  transforms. Each piece's halfspace cut keeps its own half of
  each bolt cylinder per the pour-gate pattern.

### Shared silhouette artifact

§F builds `Silhouette2d` inside `build_flange_solid`. §B needs the
SAME silhouette polyline for arc-length bolt placement. Two
options:

**(a) Build silhouette TWICE** — once in `build_flange_solid`,
once in `build_bolt_pattern_transforms`. Simple but wasteful
(re-runs marching squares).

**(b) Pre-compute silhouette in `compose_piece_solid`, pass to
both** — `compose_piece_solid` builds `Silhouette2d` once per
layer, passes `&Silhouette2d` to `build_flange_solid` and
`build_bolt_pattern_transforms`.

**Default pick:** (b). One marching-squares pass per layer, two
consumers. Adds a parameter to `build_flange_solid` (breaking
internal API, but module is private — no external callers).

## §F+§B cf-cast-cli interaction

### §F: transparent

No new TOML knobs. `FlangeSdf` internals are not exposed to
cf-cast-cli (the existing `[flange]` block already controls the
spec, not the SDF kernel).

### §B: new TOML block

```toml
[bolt_pattern]
enabled = true                        # default false; opt-in feature
bolt_hole_diameter_mm = 5.5           # default 5.5 (M5 clearance)
spacing_target_mm = 60.0              # default 60 (workshop user)
min_count = 6                         # default 6
max_count = 24                        # default 24
min_pour_gate_clearance_mm = 15.0     # default 15
```

Default `enabled = false` so existing test fixtures + cf-cast-cli
configs without `[bolt_pattern]` stay green (matches existing
pattern with `[pour_gate]` / `[registration_pins]`).

Workshop user's `~/scans/cast.toml` adds `[bolt_pattern]` with
`enabled = true` + workshop-locked defaults for iter-1.

### Cross-field validation

cf-cast-cli S2-style cross-spec gate at TOML parse:

```text
[flange].enabled == true                       # bolt pattern needs flange
[flange].flange_width_mm
    >= 2 * [flange].flange_inner_offset_mm
       + 2 * ([bolt_pattern].bolt_hole_diameter_mm/2) * 2.72
```

Reject with structured error if violated. Same pattern as the
existing flange inner_offset > half_gasket_channel_width gate.

## §F+§B test coverage

### §F-T-1: C-shape body test fixture (load-bearing)

Per [[feedback-load-bearing-test-fixtures]]: synthetic body with a
KNOWN concavity in the seam-plane cross-section. Asserts flange
material exists ALL THE WAY AROUND the body silhouette.

Geometry: 60 mm × 40 mm bounding box, U-shape or C-shape carved
out — the silhouette has both convex AND concave perimeter
segments. Marching-squares at 0.5 mm grid step produces ≥ 1
closed polyline tracing the C-shape outline.

Test asserts: at every polyline vertex, `FlangeSdf::eval` at
`(vertex.x, seam_plane_y, vertex.z) + outboard_normal * 1mm` is
**negative** (inside flange material). Probe angular coverage
should hit 100 % — no gaps in the flange ring around the C-shape.

Pre-§F current code FAILS this test in concave regions. Post-§F
fix PASSES.

### §F-T-2: marching-squares polyline closure

For a known-radius circle SDF, marching squares at 0.5 mm grid
step produces a polyline whose vertex count matches
`expected_perimeter / 0.5` ± ε; first vertex and last vertex are
within `grid_step` of each other (closure invariant).

### §F-T-3: point-to-polyline signed-distance correctness

For the same circle silhouette: at radial probes inside (negative)
and outside (positive), `signed_distance_to` agrees with the
analytical `|p| - r` formula to within `2 × grid_step` (polyline
discretization error bound).

### §F-T-4: synthetic cylinder regression (preserves S1)

Existing seam-flange S1 tests
(`flange_sdf_inside_at_body_perimeter_plus_offset`,
`flange_excludes_gasket_channel_region`, etc.) MUST stay green
post-§F. Same test fixture (synthetic cylinder) — only the
underlying FlangeSdf computation changes.

### §B-T-1: bolt-count derivation

For a known-perimeter silhouette (e.g., 240 mm circumference circle),
`build_bolt_pattern_transforms` with `spacing_target_m = 0.060`
returns exactly 4 bolts at 60 mm arc-length apart. Boundary cases:
30 mm circumference → clamped to `min_count = 6`; 2000 mm
circumference → clamped to `max_count = 24`.

### §B-T-2: bolt placement determinism

Same silhouette polyline + same spec → bit-identical bolt center
coordinates across calls. Same pattern as the existing pin
fit-invariant + pour-gate determinism tests.

### §B-T-3: pour-gate collision skip

Test fixture with a `PourGateKind::V` enabled at a known dome-end
position; place a bolt-pattern silhouette whose evenly-spaced bolt
positions would put one bolt within `MIN_BOLT_TO_POUR_GATE_M` of
the pour-leg axis. Assert: that bolt is OMITTED from the returned
transform Vec; remaining bolts re-distributed evenly along the
modified arc-length.

### §B-T-4: paradigm-safe placement

For each emitted `SubtractCylinder`: bolt center lies within the
flange's lateral envelope (flange_inner_offset_m + bolt_radius ≤
distance from silhouette ≤ flange_width_m − bolt_radius); bolt
half_length ≥ flange_thickness_m + clearance margin (cylinder
extends past both flange faces).

### §F+§B production-regen gates

- 2 mm full config: workshop user cf-view smoke confirms flange
  ring continuous around silhouette (no gaps at concavities); bolt
  holes visible in expected positions on both pieces; no F4
  Critical issues introduced.
- 3 mm full config: same gates at coarser cells (regression check).
- §R1 connectivity: cup-pieces remain 1-component each after
  bolt-hole subtraction (each piece's mesh has bolt-hole-count holes
  but stays manifold).

## §F+§B open questions

0. **Flange-width default (§B-4 above): Option A (15 mm, marginal
   0.68× walls) or Option B (18.5 mm, safe 1× walls)?**
   - **Default pick (recommend):** Option B. Mechanical safety
     margin worth the 3.5 mm visual flange-width change.
   - **Decision required before §B-S1.** Workshop user picks at
     kickoff.

1. **Per-layer vs shared bolt positions.** Default pick: per-layer.
   If iter-1 surfaces layer-alignment problems, add a
   `BoltPatternSpec.share_across_layers: bool` field and default
   to true with the layer-0 silhouette driving placement.

2. **Bolt-hole `axis` direction.** §B-2 picks "ribbon binormal at
   centerline sample" — but each bolt is at a different
   (X, Z) position on the perimeter, and `binormal` is defined
   per-centerline-sample (not per-perimeter-vertex). For workshop's
   sock geometry the centerline is roughly straight along one axis
   and binormal is approximately constant; for highly-curved
   centerlines the bolt axis at the far-perimeter position might
   want to be perpendicular to the LOCAL seam plane, not the
   centerline-sampled binormal.
   - **Default pick:** use `binormal_at_nearest_centerline_sample`
     for each bolt (snap each bolt's XZ position back to the
     nearest centerline arc fraction, take that fraction's binormal).
     Workshop sock-shape has near-constant binormal anyway → near-
     identical result; production correctness preserved for future
     more-curved centerlines.

3. **Should §F's `Silhouette2d` field replace `body: Solid` in
   `FlangeSdf`, or keep both?** Keeping both is safer (allows
   fallback to 3D body distance for unforeseen call paths); swapping
   is cleaner (no dead fields). 
   - **Default pick:** swap. `FlangeSdf` has no external callers
     and its docstring is the source of truth for its semantics.

4. **Marching-squares grid step (`SILHOUETTE_GRID_STEP_M`).** Picked
   0.5 mm. Trade-off: finer grid = more accurate polyline (better
   `signed_distance_to`) + more precompute cost + more polyline
   vertices (more per-eval cost). At 0.5 mm a 200 × 200 mm grid =
   160 k samples × ~1 µs body.evaluate = ~0.16 s per layer one-shot.
   - **Default pick:** 0.5 mm. Acceptable wall-clock cost; sub-mm
     polyline accuracy. If profiling at S0 surfaces this as a bottle-
     neck (>30 % of layer wall-clock), revisit.

5. **§B bolt-pattern emission per-layer vs at v2 export top level.**
   Currently `compose_piece_solid` is called per-layer-per-side.
   Bolt transforms emitted there are per-layer. Could move emission
   to the top-level `export_molds_v2` instead (one bolt-pattern Vec
   shared across layers).
   - **Default pick:** per-layer at `compose_piece_solid`. Matches
     existing pour-gate + registration pattern; each layer's
     silhouette potentially differs so per-layer emission is
     correct anyway.

6. **F4 gate behavior with N bolt-hole cylinders added.** Each bolt
   adds 32-segment cylinder mesh = ~64 vertices + ~128 faces. For
   12 bolts × 3 layers = ~4500 additional faces per cup-piece STL.
   Pre-§B post-MC mesh-CSG handles ~150k-face cup-pieces; +3 % face
   count is negligible. Per [[project-cf-cast-self-intersect-bvh-s1]]
   F4 self-intersect is now O(n log n) — no wall-clock blowup
   expected.
   - **Empirical S2 gate:** measure F4 wall-clock pre/post §B
     emission; expect < 5 % regression.

7. **Coordination with §SMD-4 S3 (cup-wall outer scan-mesh-direct).**
   §SMD-4 S3 replaces the SDF→MC cup-wall outer with mesh-side
   offset of the scan mesh. The flange + bolt-pattern transforms
   apply POST-MC against the resulting cup-piece mesh — same mesh-
   CSG interface regardless of whether the underlying mesh came
   from SDF→MC or scan-mesh-direct. **No coordination work needed;
   §F + §B compose cleanly with future §SMD-4 S3.**

## §F+§B implementation arc

Estimated 3-7 sessions across two PRs.

### PR #1: §F flange-continuity fix

#### §F-S1: Silhouette2d module + C-shape test fixture (probe spike)

**Scope:** ~200 LOC. New `silhouette_2d.rs` with marching squares
+ point-to-polyline distance; C-shape test fixture; assertion
that current `FlangeSdf` fails the C-shape coverage gate (red);
post-fix FlangeSdf field swap passes (green). No production regen
yet.

**Falsification gate:** §F-T-1 C-shape test fails pre-fix
(red→green confirmation that the fix addresses the documented
bug); §F-T-2 + §F-T-3 polyline + distance correctness tests
green; existing synthetic-cylinder regression tests (§F-T-4)
stay green.

#### §F-S2: production regen + workshop cf-view smoke

**Scope:** ~50 LOC plumbing (no cf-cast-cli changes — §F is
transparent to TOML per the API surface section). Production regen
at 2 mm + 3 mm cells on `~/scans/cast.toml`; workshop user cf-view
smoke gates flange-ring continuity around the sock-over-capsule
silhouette.

**Falsification gate:** workshop cf-view confirms flange wraps
all the way around the body perimeter (no gaps at concavities).
No new F4 Critical issues. 3 mm baseline still produces flange
material. cf-cast lib + cf-cast-cli tests stay green (existing
synthetic-cylinder regression gates preserved per §F-T-4).

#### §F-S3: Cold-read pass + commit + memory entry

**Scope:** cold-read all §F commits; commit + memory entry +
update [[project-cf-cast-flange-perimeter-continuity-bookmark]]
status to FIXED.

### PR #2: §B M5 bolt-pattern feature

#### §B-S1: bolt_pattern module + placement algorithm (probe spike)

**Scope:** ~150 LOC. New `bolt_pattern.rs` with arc-length placement
+ bolt-count derivation + pour-gate collision skip; unit tests
(§B-T-1 + §B-T-2 + §B-T-3 + §B-T-4); no `compose_piece_solid`
integration yet — emission stubbed via test-only call.

**Falsification gate:** all §B-T unit tests pass; bolt-count
derivation matches workshop spec at production sock perimeter
(5-7 bolts/half at 60 mm spacing).

#### §B-S2: compose_piece_solid integration + cf-cast-cli wire

**Scope:** ~100 LOC. Wire `build_bolt_pattern_transforms` into
`compose_piece_solid`'s transform Vec; add `[bolt_pattern]` TOML
block + cross-field validation; production regen at 2 mm cells
on workshop user's iter-1 config (bolt-pattern enabled).

**Falsification gate:** cup-piece STLs contain bolt holes at
expected positions; cup-piece §R1 connectivity intact; F4 clean.

#### §B-S3: workshop cf-view smoke + iter-1-print gate

**Scope:** workshop user cf-view + Orca smoke on
`~/scans/cast_iter1_post_F_and_B/` confirms (a) flange continuous,
(b) bolt holes outboard of seam at expected ~60 mm spacing,
(c) no collision with pour-gate, (d) bolt-head bearing area
adequate.

**Falsification gate:** workshop user GREEN-lights iter-1 print.

#### §B-S4: cold-read pass + commit + memory entry

**Scope:** cold-read all §B commits; commit + memory entry +
mark scope-lock memo SHIPPED.

## §F+§B prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]],
the following memories MUST be read before each phase:

- [[project-cf-cast-flange-perimeter-continuity-bookmark]] — the
  source diagnostic for §F. The fix-path Candidate A picked here
  matches the bookmark's recommendation.
- [[project-cf-cast-flange-continuity-plus-bolt-pattern-scope-lock]]
  — the workshop user's locked spec. Rule: bolt-pattern parameters
  MUST NOT silently drift from the locked values (M5, 5.5 mm,
  ~6/half, ~60 mm, no counter-bore, no inserts).
- [[project-cf-cast-seam-flange-s1]] — original FlangeSdf design;
  the synthetic-cylinder fixture that missed the bug. §F-T-4
  preserves these tests as regression gates.
- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — paradigm-
  boundary framework. §B's SubtractCylinder is CONTAINED placement
  by §B-2 geometry analysis; verify in S1 review.
- [[project-cf-cast-geometry-crispness-q1-fix]] — §Q-1 cup-wall
  body-tracking shell. §F changes FlangeSdf composition only;
  shell SDF unchanged.
- [[feedback-read-prior-arc-memory-before-architectural-decisions]]
  — rule #6: don't silently revert prior decisions. Workshop user
  locked NO counter-bore + NO heat-set inserts; §B implementation
  MUST honor those constraints.
- [[feedback-load-bearing-test-fixtures]] — load-bearing for §F-T-1
  C-shape fixture. Synthetic cylinder is convex-only; ship a fixture
  that exercises the failure mode.
- [[feedback-keep-asks-bold-and-short]] — workshop user feedback
  pattern; recon docs themselves are not asks, but mid-arc decision
  points (open question 0 — flange-width pick; open question 1 —
  per-layer-vs-shared bolts) should hit the workshop user as short
  focused asks at §B-S1 kickoff.

## Successor

§F-S1 probe spike picks up next session. Workshop user picks
empirical-first (write the C-shape fixture + verify it surfaces
the bug pre-fix) — same pattern as §Q-4 S1 + §SMD-4 S1 probe
spikes, validated by the §Q-1 + §Q-5 + scan-mesh-direct shipped
arcs. Smallest hypothesis falsification is the cheapest first step.

Cold-read pass-1 on this recon doc itself is mandatory before any
§F-S1 commits — prior recons (§Q-4, §SMD-4) caught 4-7 substantive
errors at cold-read; non-negotiable per the project's recon
discipline.
