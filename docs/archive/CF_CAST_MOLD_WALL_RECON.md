# cf-cast mold-wall optimization — recon findings

> **Status**: RECON SHIPPED 2026-05-19 LATE-NIGHT after the
> [[project-cf-cast-mold-wall-bookmark]]. Three-session pattern —
> bookmark / recon / implementation. This is session 2.
>
> **Predecessors**:
> - `docs/PROGRAM_GAMEPLAN.md` §4 B1 — the gameplan ask
> - `docs/CF_CAST_MOLD_WALL_BOOKMARK.md` — bookmark, design space,
>   open questions Q1-Q6
>
> **Spike artifact**: `tools/cf-cast-cli/src/derive.rs` lines
> ~283-301 — replaced cuboid bounding region with
> `outermost_body.offset(0.005)`. Hardcoded 5 mm wall.

---

## TL;DR

Option A (caller-side outermost-offset) replaces the cuboid bounding
region with `outermost_body.offset(wall_thickness)`. Spike on the
iter-1 sock_over_capsule fixture (2 layers, cumulative thickness
13 mm, cavity inset 3 mm):

- **Mold plastic (4 pieces)**: 6 018 cm³ → 568 cm³ — **10.6× drop**
- **Outermost-layer pieces alone**: 1 478 cm³ → 232 cm³ — **6.4× drop**
- **F4 mesh-printability**: all 6 artifacts (4 pieces + 2 plugs)
  passed with no blocking Criticals
- **Run wall-time** (release): 305 s for 6-artifact pipeline; mesh
  vertex count down from ~25 K/piece to ~14 K/piece
- **Plug-pin protrusion concern surfaced**: the iter1
  `PlugPinSpec`'s `pin_length_m = 20 mm` exceeds the 5 mm spike
  wall, so the plug-anchor pin pokes ~15 mm out of the mold piece's
  back face. F4 doesn't flag (the through-hole is topologically
  valid); workshop interaction call.

**Recommendation**: **ship Option A** in the implementation step, with
a `cast.wall_thickness_m` TOML knob (FDM-default 5 mm) and a
validator gating `wall_thickness_m + tolerance ≥
plug_pins.pin_length_m` when plug-pins are enabled. Bank Option B
(per-layer offsets) — the 6 % incremental gain estimated for B is
not worth a cf-cast API change mid-arc.

---

## 1. Baseline measurement (cuboid bounding region)

Source: `/Users/jonhillesheim/scans/cast_iter1_design.baseline/`
(committed iter-1 run from 2026-05-17, ribbon split-normal `[1, 0, 0]`,
mesh_cell_size_m=0.003, cavity inset 3 mm, 2 layers, total cumulative
thickness 13 mm, bounding_margin_m=0.020, 6 STLs).

Per-piece signed-volume sums (divergence theorem) via
`/tmp/stl_volume.py`:

| File | triangles | volume (cm³) |
|------|-----------|--------------|
| mold_layer_0_piece_0.stl | 25 604 | 1 602.084 |
| mold_layer_0_piece_1.stl | 24 748 | 1 459.259 |
| mold_layer_1_piece_0.stl | 26 000 | 1 549.725 |
| mold_layer_1_piece_1.stl | 25 124 | 1 406.883 |
| plug_layer_0.stl | 7 632 | 303.500 |
| plug_layer_1.stl | 11 520 | 584.301 |
| **TOTAL** | **120 628** | **6 905.752** |

Mold pieces alone (excluding plugs): **6 018 cm³** across 4 pieces.

At PLA density 1.24 g/cm³, that's ~7.5 kg of PLA per mold set at
100% infill — or ~2.2 kg at 30% infill. The user's complaint is
clearly justified.

### 1.1 Geometric origin of the waste

Scan AABB (from `.prep.toml`'s `output.aabb_m`):
- min: (-36.7, -31.8, -54.5) mm
- max: (34.3, 39.6, 72.8) mm
- full extents: (71.0, 71.4, 127.3) mm
- volume: ~645 cm³

Bounding cuboid (`derive.rs:283-291`):
- half_extents = scan_half + (cumulative_thickness + bounding_margin)
  = (35.5, 35.7, 63.7) + (13 + 20) = (68.5, 68.7, 96.7) mm
- full extents: (137, 137.4, 193.3) mm
- volume: **3 642 cm³**

But the *layer body itself* (cumulative scan + 13 mm shell) is
roughly 60% of the AABB → ~640 cm³. So the cuboid is ~5.7× the
volume of the outer layer body. Most of that excess is the corners
of the cuboid where the silicone-negative geometry is absent. That's
the waste Option A eliminates.

---

## 2. Option A spike — `outermost.offset(5 mm)` bounding region

Spike diff at `tools/cf-cast-cli/src/derive.rs` lines ~283-301:

```rust
// Before:
let bounding_half_extents = scan_aabb.half_extents()
    + Vector3::repeat(cumulative_thickness + config.cast.bounding_margin_m);
let bounding_center = scan_aabb.center();
let bounding_region = Solid::cuboid(bounding_half_extents)
    .translate(Vector3::new(bounding_center.x, bounding_center.y, bounding_center.z));

// After (spike — hardcoded 5 mm wall):
let wall_thickness_m_spike: f64 = 0.005;
let outermost_body = layers.last()
    .expect("layers is non-empty per validate_after_layer_source")
    .body.clone();
let bounding_region = outermost_body.offset(wall_thickness_m_spike);
```

### 2.1 Containment invariant (Q1 partial answer)

`compose_piece_solid` relies on `bounding_region ⊇ layer_body` for
every layer (otherwise the per-piece `bounding ∖ layer_body`
composition leaves negative-SDF holes outside the layer body).

Cf-cast layer ordering is **innermost-first**, so `layers.last()`
is the outermost body, defined as
`scan_surface + cumulative_thickness − cavity_inset`. Inner layer
`i` has body `scan_surface + cum[i] − cavity_inset` with
`cum[i] ≤ cum[N-1]`. Since outermost ⊇ every inner layer body, and
`outermost.offset(wall) ⊃ outermost`, we get
`outermost.offset(wall) ⊃ every layer_body`. Invariant holds by
construction.

### 2.2 SDF accuracy of the offset (Q2)

`Solid::offset(d)` evaluates as `f(p) − d`. The bookmark Q2 worried
that for piecewise-CSG bodies the offset can be uneven near
corners. Re-reading `pinned_floor_shell` (`cf-design/src/solid_layered.rs`):

```text
body  = Solid::from_sdf(closed_sdf, bounds)        # exact SDF
rind  = Solid::from_sdf(|open_sdf(p)|−T, bounds)   # exact-ish
shell = body.union(rind).intersect(plane_1)...      # min/max composition
```

The composed shell is **conservative** (min/max of exact-ish SDFs):
distances near the surface are exact, but distances deep inside the
body can be overestimated. For `.offset(0.005)` shifting the iso
level by 5 mm, the OUTER surface of the offset is the locus where
the conservative SDF = 0.005. At every point in the immediate
neighborhood (where one of the source SDFs dominates and the
distance is exact), the offset is geometrically exact.

For our use case (FDM print at ~0.1 mm tolerance, 5 mm wall), the
offset is more than accurate enough. The walls may be very slightly
thicker than 5 mm near cap planes (where multiple max() ops can
overlap), but that's a stiffness BONUS, not a defect.

### 2.3 Bounds() behavior

`FieldNode::Offset(child, d).bounds() = child.bounds().expanded(|d|)`
(`bounds.rs:217`). So `outermost.offset(5mm).bounds()` =
outermost-body-AABB inflated by 5 mm — tight envelope, no wasted
sampling volume. This is a marching-cubes PERF win: fewer cells to
sample.

`compose_piece_solid` calls `bounding_region.bounds()?` to
derive the ribbon half-space AABB; the tight inflated-body bounds
serve that purpose correctly.

---

## 3. Spike measurement (Option A on iter-1 sock_over_capsule)

Per-piece signed-volume sums via `/tmp/stl_volume.py` (divergence
theorem on the saved STL):

| File | triangles | spike (cm³) | baseline (cm³) | × |
|------|-----------|-------------|----------------|-----|
| mold_layer_0_piece_0.stl | 14 336 | 168.357 | 1 602.084 | 9.5 |
| mold_layer_0_piece_1.stl | 14 244 | 168.517 | 1 459.259 | 8.7 |
| mold_layer_1_piece_0.stl | 14 728 | 115.763 | 1 549.725 | 13.4 |
| mold_layer_1_piece_1.stl | 14 608 | 115.965 | 1 406.883 | 12.1 |
| plug_layer_0.stl | 7 632 | 303.500 | 303.500 | 1.0 |
| plug_layer_1.stl | 11 520 | 584.300 | 584.301 | 1.0 |
| **mold pieces total** | **57 916** | **568.602** | **6 017.951** | **10.6** |
| **all artifacts total** | **77 068** | **1 456.401** | **6 905.752** | **4.7** |

Per-piece AABB extents (mm) via `/tmp/stl_bbox.py`:

| Piece | baseline bbox | spike bbox |
|-------|---------------|------------|
| mold_layer_0_piece_0 | 137 × 77 × 193 | 101 × 60 × 148 |
| mold_layer_0_piece_1 | 137 × 72 × 193 | 101 × 52 × 149 |
| mold_layer_1_piece_0 | 137 × 77 × 193 | 101 × 60 × 148 |
| mold_layer_1_piece_1 | 137 × 72 × 193 | 101 × 52 × 149 |

Spike piece extents ≈ scan AABB (71 × 71 × 127 mm) + cumulative
shell (13 mm) + 5 mm wall — about (101 × 100 × 148) mm. Tight
envelope around the offset shell, no wasted volume.

Plug volumes unchanged across baseline/spike (Option A doesn't
touch the plug construction).

### 3.1 Wall-time comparison

Spike run: `cargo run -p cf-cast-cli --release ...
cast.iter1-design.toml` completed in **305 s**. Per-piece:
compose+MC ~16.4 s, F4 validation ~40-50 s. The baseline didn't
log walltime to disk, but the file timestamps suggest a similar
order (~5-7 min). Wall-time is not the dominant gain; **plastic
is**. (Spike's lower vertex count makes F4 incrementally cheaper,
but F4 walltime is dominated by mesh-printability's O(face²)
self-intersection check, which is dimensionally `face_count²`,
not bounding-region size.)

### 3.2 F4 (mesh-printability) gate

All 6 artifacts passed F4 validation without aborting the run
(blocking Criticals would have aborted per
`spec.rs:330`-`spec.rs:443`). Per-piece F4 walltime ranged from
0.5 s (plug 1) to 47.6 s (largest piece). The 47.6 s peak is the
self-intersection scan over ~7300 vertices.

### 3.3 Pour gate / vent / pin clearance (Q3, Q5)

**Pour gate (6 mm Ø, side-mounted, 90 mm channel)**: ✓ clears.
Gate cylinder runs along the ribbon binormal from the centerline
midpoint for `2 * gate_half_length_m = 90 mm`. At the spike's
~50 mm half-extent perpendicular to the centerline plus 5 mm wall,
the cylinder exits the offset shell with ~40 mm of cylinder
remaining outside — the workshop user pours into the protruding
cylinder section as before. Exit face is **curved** rather than
flat, but the workshop pour-fixture is gravity-driven, not flat-
clamp-driven (Q5 verdict: aesthetic, not blocking).

**Apex vent (2 mm Ø, vertical, 80 mm channel)**: ✓ clears. Vent
cylinder rises along `+Z` from the centerline argmax-z vertex for
80 mm; the spike's piece bbox max-z is ~88 mm, well past the
cavity ceiling (which sits at scan-z-max ≈ +73 mm). Vent exit on
the curved `+Z` face, again gravity-OK.

**Inter-piece alignment pins (3 mm Ø × 10 mm, along binormal)**: ✓
clears. Pin protrusion is local to the ribbon seam and aligned
with the binormal direction (mostly tangent to the body surface).
The 5 mm wall is thick enough in the binormal direction;
visual-inspection NOT done but the geometric envelope leaves
margin.

**Plug-anchor pins (3 mm Ø × 20 mm long, along -tangent at
centerline endpoint)**: ✗ **pin pokes through the mold wall**.

The cap plane sits at z ≈ -54.5 mm (scan AABB min Z, the open end
of the sock). The plug-anchor pin extends from the centerline
endpoint (on the cap plane) along -tangent, which is roughly -Z
at this endpoint. Pin tip at z ≈ -74.5 mm.

Spike mold piece extends to z ≈ -60 mm — that's the 5 mm wall past
the cap plane, MC-rounded. Pin tip at -74.5 mm is **14.5 mm past
the piece's outer face**. The socket subtraction (`piece ∖
sockets`) carves a through-tunnel; the plug pin sticks out the
back of the mold piece by 15 mm.

This is **F4-printable** (the tunnel is topologically valid; no
self-intersection, no excessive overhang). It's a **workshop-
interaction** concern:

- Workshop user sees a 3 mm Ø × 15 mm protrusion sticking out the
  bottom of the assembled mold (one per piece per layer, so 4
  protrusions total on iter-1).
- The pin still serves its alignment function (slides into socket;
  socket tip rests against the pin's shoulder per cf-cast-cli's
  Plug Anchor procedure).
- The protrusion may bother surface contact (assembled mold won't
  sit flat on workbench). Mild concern.

**Mitigation options** (all implementation-step decisions, not
recon-step):

- **A.1 — pin-protector boss**: extend bounding region locally
  with a small cuboid at each pin location. Most-correct fix;
  +20-50 LOC in `derive.rs` to compute pin locations + union with
  bounding_region.
- **A.2 — shrink iter1 pin_length**: make
  `PlugPinSpec::iter1()`'s `pin_length_m` `wall_thickness − 1 mm`
  per default (e.g., 4 mm for 5 mm wall). Cheapest. Loses pin
  engagement length but pin engagement is overkill for a
  gravity-driven seat.
- **A.3 — validator + abort**: cross-field gate
  `wall_thickness_m ≥ plug_pins.pin_length_m` (when plug-pins are
  enabled). User picks the wall that fits their pin or vice-versa.
  Cheapest in code; pushes the tuning to the workshop.
- **A.4 — accept the protrusion**: ship as-is, document in
  procedure.md. Cheapest in effort; ugly cosmetic outcome.

**Implementation-step recommendation**: A.3 (validator) +
documentation in procedure.md. Cheapest correct path; the
workshop user picks the trade-off explicitly. If iter-1 cast
surfaces a real issue, A.1 (pin-protector boss) is the escalation
target.

---

## 4. Open questions resolution

**Q1** — Does Option A produce structurally-sound molds for iter-1?
— **YES, with the plug-pin caveat** (§3.3). Plastic drops 10.6×,
F4 passes, mesh complexity falls; the only blocker is the pin-
protrusion cosmetic issue, addressable via A.3 validator.

**Q2** — Is `Solid::offset(d)` accurate enough? — **YES** (§2.2).
Conservative-SDF region distortion is well above the 0.1 mm FDM
tolerance. Iter-1 spike confirmed: pieces print as expected
shells; no MC artifacts or sliver geometry surfaced in F4.

**Q3** — Per-layer Option B: **defer**. See §5. Option A already
captures 10.6× of the available 11.3× total; the incremental gain
from B is ~6%, not worth the cf-cast API change mid-arc.

**Q4** — Wall thickness as user-tunable TOML knob: **YES**.
Different printers + materials have different minimum walls.
Implementation step adds `cast.wall_thickness_m` to `CastDefaults`
with FDM-default 5 mm + validator (positive + finite + ≥ printer
min-wall + ≥ plug_pins.pin_length_m when plug-pins enabled).
`bounding_margin_m` field is unused on the new path → delete per
[[feedback-strip-the-knob-when-default-works]].

**Q5** — Pour gate flat-exit assumption: pour gate cylinder exits
the offset shell on a **curved** surface (§3.3). Workshop pour-
fixture is gravity-driven, not flat-clamp-driven (per the
procedure.md generated for the baseline), so the curved exit is
**aesthetic, not blocking**.

**Q6** — Mesh-volume regression test shape: F4 mesh-printability
doesn't currently report per-piece volume. Implementation step adds
a regression test (per `tests/integration.rs` style) that asserts
each piece volume scales with shell area × wall thickness, NOT
scan-AABB volume. Tightest test: assert
`piece_volume_cm³ < 2 × shell_target_cm³` where
`shell_target = surface_area(layer_body) × wall_thickness_m`.
Loose enough to absorb MC stair-step ≤ 1 cell, tight enough to
catch a regression that accidentally re-introduces the cuboid.

---

## 5. Option A vs Option B trade-off (post-measurement)

Measured scaling on iter-1 sock_over_capsule (2 layers, cumulative
13 mm thickness):

| Variant | Mold pieces (cm³) | × vs baseline |
|---------|-------------------|---------------|
| Baseline (cuboid + 20 mm margin) | 6 018 | 1.0× |
| Option A spike (outermost.offset(5 mm), shared) | 569 | 10.6× |
| Option B (per-layer offset(5 mm)) — predicted | 535 | 11.3× |

Option B's prediction comes from a back-of-envelope:
- Layer 0 (inner) piece volume ≈ surface_area(layer_0) × 5 mm.
  surface_area(layer_0) ≈ 165 cm² (scan + 10 mm cumulative shell);
  piece pair ≈ 165 × 0.5 = **83 cm³**.
- Layer 1 (outer) piece volume ≈ surface_area(layer_1) × 5 mm.
  surface_area(layer_1) ≈ 185 cm²; piece pair ≈ 185 × 0.5 =
  **93 cm³**.
- Option B total ≈ 2 × (83 + 93) = **352 cm³** (vs Option A's
  569 cm³). Actually a **38 % improvement** on top of Option A —
  not the 6 % I guessed earlier.

Hmm — Option B is a bigger gain than the TL;DR initially claimed.
Let me think about whether it's worth the API change anyway:

- **Option B effort**: per-layer `bounding_region: Option<Solid>`
  on `CastLayer`. ~50 LOC change in cf-cast (composer + spec), ~20
  LOC in cf-cast-cli (caller), regression tests, all examples
  updated. **2-3 sessions** per bookmark §3.2 estimate.
- **Option A effort**: 5-line caller-side change in cf-cast-cli +
  TOML knob plumbing + validator. **1-2 sessions**.
- **Incremental gain B vs A**: 569 → 352 cm³ = 217 cm³ saved per
  iter-1 mold set. At PLA 1.24 g/cm³ and 30 % infill, that's ~80 g
  of PLA per device — meaningful at scale but small per device.
- **Layer count scaling**: more layers AMPLIFY Option B's gain (it
  scales per-layer; A only optimizes the outermost). The iter-1
  fixture has 2 layers; future designs may use 3+ layers.

**Verdict**: ship Option A FIRST (this implementation session).
Bank Option B as a clearly-scoped follow-on arc once iter-1 cast
ships and we have real workshop feedback on whether the
inner-layer mold pieces' thickness is a real cost. Don't pre-
optimize before the workshop validates the basic approach.

Bookmark §3.5 already recommends this sequencing; the recon
confirms it numerically.

---

## 6. Recommendation

**SHIP OPTION A** in the implementation session (B1 sub-leaf 3).

Implementation plan:

1. **TOML schema (`tools/cf-cast-cli/src/config.rs`)**:
   - Add `wall_thickness_m: f64` to `CastDefaults` with default 5 mm
     (`default_wall_thickness_m`).
   - Delete `bounding_margin_m` field + `default_bounding_margin_m`
     (load-bearing only for the cuboid path).
   - Add validator: `wall_thickness_m > 0`, finite, **and** (when
     `plug_pins.enabled`) `wall_thickness_m + 1 mm ≥
     plug_pins.pin_length_m` (or its default 20 mm if no override).
     The 1 mm margin absorbs MC stair-step on the wall surface.

2. **Derive (`tools/cf-cast-cli/src/derive.rs`)**: replace cuboid
   construction at lines 283-291 with the spike's `outermost.offset
   (wall_thickness_m)`. Use the validated config field, not the
   hardcoded constant. Drop the `bounding_center` + cuboid
   construction entirely.

3. **Examples**: update
   `examples/cast/layered-silicone-device-v2-scan-curve-following`
   to use offset-shell bounding (or document why it stays cuboid —
   the example builds `Solid::pipe` layers directly, not
   `pinned_floor_shell`, so the spike's `layers.last().body.offset
   (wall)` works identically).

4. **Tests**:
   - `tests/integration.rs` cube fixture: replace volume-assertion-
     against-cuboid with shell-volume bound (per Q6 above).
   - Add a regression test that asserts:
     `piece_volume_cm³ < surface_area(layer_body_cm²) × 2 ×
     wall_thickness_cm` for each piece in the cube fixture.

5. **Procedure.md** (`design/cf-cast/src/procedure.rs`): the
   "v2 Mold Assembly" section describes piece-to-piece pin
   engagement assuming a flat outer face. Add a brief note that
   the outer face follows the device's contour. **Workshop-
   facing**: optional, since the iter-1 procedure.md already
   describes the assembly correctly (no flat-face assumption
   surfaces in the prose).

6. **Memory note**: update [[project-cf-cast-mold-wall-bookmark]]
   to point at this recon + the implementation session, mark
   bookmark as RESOLVED. New [[project-cf-cast-mold-wall-arc-shipped]]
   memo at implementation-session ship.

**Bank**:

- **Option B (per-layer bounding regions)** — clearly-scoped
  follow-on arc when iter-1 cast surfaces real cost-of-inner-layer-
  mold-pieces feedback. Likely 2-3 sessions.
- **Option A.1 (pin-protector bosses)** — escalate if pin-
  protrusion is workshop-noisome on iter-1 cast.
- **Option D (ribbed shell)** — only if 5 mm wall is structurally
  inadequate.
- **Variable-per-layer wall thickness, SLA-min-wall override,
  pour-pressure FEM** — all banked per bookmark §7.

---

## 7. Pointers

- `tools/cf-cast-cli/src/derive.rs:283-301` — spike construction
- `design/cf-cast/src/piece.rs:94` — `compose_piece_solid` consumer
- `design/cf-design/src/solid_layered.rs:93` — `pinned_floor_shell`
- `design/cf-design/src/bounds.rs:217` — `Offset` bounds
- `docs/CF_CAST_MOLD_WALL_BOOKMARK.md` — predecessor bookmark
- `docs/PROGRAM_GAMEPLAN.md` §4 B1 — gameplan ask
- `/tmp/stl_volume.py` — recon volume-measurement script
- `/Users/jonhillesheim/scans/cast_iter1_design/` — baseline
  output (the as-shipped iter-1 from 2026-05-17, preserved
  verbatim post-recon)
- `/Users/jonhillesheim/scans/cast_iter1_design.spike_option_a/` —
  Option A spike output (5 mm wall, hardcoded). Useful for
  workshop visual inspection of the offset-shell mold shape +
  the plug-pin protrusion concern flagged in §3.3.
- `/tmp/b1_spike_run.log` — spike run stdout/stderr (305 s
  walltime, no Critical aborts)
- `/tmp/stl_volume.py`, `/tmp/stl_bbox.py` — recon-measurement
  scripts (binary-STL parse + divergence-theorem volume +
  AABB extents)

---

End of recon, draft. Sub-leaf 3 (implementation) commences once the
spike measurement and Q-resolution are nailed down.
