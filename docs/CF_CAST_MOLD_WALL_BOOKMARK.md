# cf-cast mold-wall optimization — bookmark

> **Status**: BOOKMARK, set 2026-05-19 LATE-NIGHT after the program
> gameplan reframe. First concrete user-facing arc under the new
> cadence. Three-session pattern per
> [[feedback-bookmark-when-surface-levers-exhaust]] —
> bookmark / recon / implementation.
>
> **Predecessors**:
> - `docs/PROGRAM_GAMEPLAN.md` §4 B1 — the gameplan ask
> - User verbatim 2026-05-19: "the mold we use currently use a ton
>   of plastic. they are cubes around the negatives, really, it
>   should be a wall around the negatives to optimize print
>   time/plastic use for FDM printers"

---

## TL;DR

cf-cast-cli currently generates mold pieces as `cuboid ∖ layer_body`,
producing a big plastic block with the silicone-negative carved
out. Target: replace the cuboid with a thin-wall shell that
follows the negative geometry, dropping print time + plastic by
roughly the volume ratio `cuboid_volume / shell_volume`. Typical
estimate for the iter-1 sock geometry: 5-10× plastic reduction.

The optimization is **caller-side** in cf-cast-cli; cf-cast's
`compose_piece_solid` already accepts an arbitrary `bounding_region:
Solid`, not just a cuboid. The fix is to construct
`bounding_region` as an offset-expansion of the outer layer body
instead of the scan AABB.

The non-trivial part is **per-layer wall thinning**: cf-cast's
v2/v2.1 API uses a SHARED `bounding_region` across all layers, so
the outermost layer gets a thin shell but inner-layer pieces are
still thick (the shared envelope is at `outermost.offset(wall)`).
For maximum plastic savings, each layer needs its own
body-offset-based bounding region — which is a cf-cast API change.

---

## 1. Current state

### 1.1 Where the cuboid is constructed

`tools/cf-cast-cli/src/derive.rs:283-291`:

```rust
// Bounding cuboid: enveloping every layer body + margin.
let bounding_half_extents = scan_aabb.half_extents()
    + Vector3::repeat(cumulative_thickness + config.cast.bounding_margin_m);
let bounding_center = scan_aabb.center();
let bounding_region = Solid::cuboid(bounding_half_extents).translate(Vector3::new(
    bounding_center.x,
    bounding_center.y,
    bounding_center.z,
));
```

This is the line that produces "cubes around the negatives." The
cuboid spans the scan AABB + cumulative layer thickness + a
caller-tunable margin (`config.cast.bounding_margin_m`).

### 1.2 How the cuboid is consumed

`design/cf-cast/src/piece.rs:94` — `compose_piece_solid`:

```rust
piece_sdf = bounding_region ∖ layer_body ∩ ribbon_side
```

Then registration pins are union'd in, pour-gate + air-vent
channels are subtracted, plug-pins are subtracted. The
`bounding_region` shape determines the MOLD WALL shape; the
`layer_body` carves the silicone-negative cavity.

### 1.3 Volume calculation

For iter-1 `sock_over_capsule` scan:
- Scan AABB: roughly 60 × 60 × 130 mm³ ≈ 470 cm³
- Outer-layer-body envelope (silicone shell outer surface):
  roughly r ≈ 35 mm × length 120 mm ≈ 460 cm³ (similar order)
- Wall thickness desired: ~5 mm (FDM structural minimum)
- Thin-wall mold piece volume: ~80-120 cm³ (estimate)
- **Plastic reduction: ~4-6× per piece**

Wall time savings track plastic 1:1 for FDM (extrusion-limited).

---

## 2. Target — wall around negatives

The minimal-correct fix:

```rust
// Replace the cuboid with an offset-expansion of the outermost
// layer body. The outer mold wall now follows the silicone-
// negative envelope at distance `wall_thickness_m`.
let outermost_body = layers.last().unwrap().body.clone();
let bounding_region = outermost_body.offset(config.cast.wall_thickness_m);
```

This swap is ~5 lines of code. Resulting `piece = bounding_region ∖
layer_body ∩ ribbon_side` becomes a thin shell at the outermost
layer; inner-layer pieces remain thick because they're carving the
same outer envelope.

For maximum plastic reduction across ALL layers, the cf-cast API
needs per-layer bounding regions (see §3 Option B).

---

## 3. Design space

### 3.1 Option A — caller-side outermost-offset (smallest change)

**What**: Replace the cuboid construction at `derive.rs:283-291`
with `bounding_region = outermost_body.offset(wall_thickness_m)`.

**Pros**:
- ~5-line code change in cf-cast-cli
- Zero cf-cast API change
- Outermost-layer mold piece becomes a thin shell

**Cons**:
- Inner-layer pieces remain thick (shared bounding region still
  envelopes the outermost body)
- Plastic savings concentrated on one layer (the outermost),
  typically the smallest plastic budget anyway

**Estimate**: 1-2 sessions (impl + verification + regression tests).

### 3.2 Option B — per-layer bounding region (cf-cast API change)

**What**: Replace the single `bounding_region: Solid` field on
`CastSpec` with a per-layer `bounding_region: Option<Solid>` (or
similar) on `CastLayer`. When `None`, auto-derive as
`layer.body.offset(wall_thickness_m)`.

**Pros**:
- Every layer's mold piece becomes a thin shell
- Maximum plastic savings (~80% reduction estimated for iter-1)
- Generalizes well to future layer-count variations

**Cons**:
- cf-cast API change touches CastSpec + compose_piece_solid +
  every caller
- More test surface to revalidate
- Risk of breaking v1 (`export_molds`) which assumes a single
  shared bounding region

**Estimate**: 2-4 sessions. Big enough to need its own
bookmark/recon split.

### 3.3 Option C — fixed-wall-thickness shared bounding (compromise)

**What**: Keep the single shared `bounding_region` field but
compute it as `Union(layers[i].body.offset(wall_thickness) for i in
0..N)`. The bounding envelope follows the union of all layer
bodies' offset-expansions.

**Pros**:
- No cf-cast API change
- Inner layers AND outer layer each get thin shells (because the
  union follows each layer's contour)

**Cons**:
- Union of nested solids degenerates to the outermost — the inner
  layers' offsets fall INSIDE the outermost's offset, so the union
  equals the outermost's offset. Same as Option A.
- Doesn't actually solve the inner-layer thickness problem

**Verdict**: probably degenerate to Option A; needs a quick check
at recon to confirm.

### 3.4 Option D — ribbed shell (structural reinforcement)

**What**: Option A or B's thin shell + structural ribs (radial or
axial spokes inside the mold wall) for stiffness during pour.

**Pros**:
- Allows even thinner walls (e.g., 2-3 mm) while staying
  structurally sound
- More plastic savings on top of Option A/B

**Cons**:
- Adds geometric complexity to mold pieces (ribs need to
  demold cleanly)
- Requires structural FEM or rule-of-thumb design — could be its
  own arc

**Verdict**: defer. Option A or B alone should hit "good enough"
without needing rib reinforcement at typical iter-1 silicone-pour
pressures (which are gravity-driven, not pressure-driven).

### 3.5 Recommendation

**Recon should evaluate Option A first** as the cheap spike. If
it produces an acceptable mold for iter-1 (outermost piece is
thin; inner pieces are still cuboid-ish but the total plastic
saving is meaningful), ship it. If the inner-layer thickness is
still the dominant plastic cost, escalate to Option B.

---

## 4. Constraints to verify at recon

### 4.1 Minimum wall thickness for FDM structural integrity

The mold piece must survive:
- Silicone pour pressure (gravity-driven; ~10 kPa hydrostatic for
  100 mm column)
- Curing-shrinkage stresses (Ecoflex shrinks ~0.1%; DS20A ~0.2%)
- Demolding handling (manual pressure to peel piece away from
  cured silicone)

FDM PLA / PETG rule-of-thumb minimum wall: **3-5 mm** for
hand-handled molds in this size range. Recon should confirm with
existing `mesh_printability::PrinterConfig::fdm_default()`'s
`min_wall_thickness` enforcement.

### 4.2 Pour gate + air vent clearance

`PourGateSpec` (cf-cast/src/pour.rs) carves a side-mounted gate
channel through the mold wall. The wall thickness needs to
accommodate the gate's channel diameter (typically ~5-8 mm). If
wall_thickness < gate_diameter + some tolerance, the gate breaks
through the wall.

Apex vent similar — vertical channel rising along +Z from the
layer body's z_max to the mold wall's outer surface. Needs the
wall to be at least the vent's diameter (~3-5 mm).

### 4.3 Alignment pin clearance

`PinSpec` (cf-cast/src/registration.rs) places inter-piece
alignment pins along the ribbon split. The pins protrude from one
piece, recess into the other. The wall thickness AT THE RIBBON
must accommodate the pin's protrusion length + diameter.

### 4.4 Plug-anchor pin clearance

`PlugPinSpec` (cf-cast/src/plug.rs) anchors the cured silicone +
plug assembly to the mold piece. Wall thickness near the plug
pins needs to allow the pin's recessed socket.

### 4.5 Demolding clearance

Ribbon-split molds: the ribbon's curve-following path needs to
intersect the outer wall (not the cuboid corner). For a thin
shell, the ribbon naturally intersects the outer wall as a curve;
no special handling needed beyond verifying the ribbon's
`bounds` are inside `bounding_region.bounds()` (existing cf-cast
gate).

V1 cup demolding: the `+Z` clip cuboid above each layer body must
extend past the outer wall. Currently
`CLIP_XY_SLACK_M = 100 mm` (1.4× scan AABB); a thin-wall shell
might benefit from a tighter clip.

---

## 5. Open questions for the recon

**Q1** — Does Option A produce structurally-sound molds for
iter-1? (Spike it; print one; pour silicone; see if it leaks /
cracks / demolds cleanly.)

**Q2** — Is `Solid::offset(d)` accurate enough for a thin-wall
target? `offset` returns `{x : sdf(x) < d}`. Accuracy depends on
the underlying SDF's gradient near the layer body's surface. For
SDF-derived bodies (`Solid::from_sdf`) the offset is exact; for
piecewise CSG bodies it can be inflated unevenly near corners.
For the iter-1 layer bodies (offset of flood-filled scan SDF),
this should be clean.

**Q3** — Per-layer Option B: is the cf-cast API change acceptable
mid-arc, or should we ship Option A as v1 and recon Option B
separately?

**Q4** — Wall thickness as user-tunable knob: should
`config.cast.wall_thickness_m` be exposed via TOML, or hardcoded
at the FDM-default? Probably TOML-exposed since different
printers + materials have different minimum walls.

**Q5** — Does the outer wall need to be axis-aligned-flat where
the pour gate exits? Per `PourGateSpec`, the gate exits on the
`±Y` binormal face. A curved-outer-wall mold means the gate
exits a curved surface, which might complicate the pour-clamp
fixture the workshop uses. Recon should check the
`build_pour_gate_solid` to see if it assumes a flat exit face.

**Q6** — Mesh-volume regression test: cf-cast has F4 validation
via `mesh-printability` — does it report mold piece volume in
the output? If so, the regression test can assert the new piece
volumes are within target range (e.g., 1-3× the silicone shell
volume, not 10-20× as today).

---

## 6. Sequencing

Three-session pattern:

**Session N (this) — bookmark.** This doc. Names the failure
mode (plastic waste), the design space (4 options), the
constraints (wall thickness + gates + pins + demolding), the
open questions.

**Session N+1 — recon.** Spike Option A in a branch; measure the
mold piece volume against the cuboid baseline; verify gate +
vent + pin + ribbon clearances; F4 validation pass; visual gate
on iter-1 sock geometry. If Option A is sufficient, write
findings; if not, recon Option B.

**Session N+2 — implementation.** Ship the chosen option to dev
behind `--release`. Add `wall_thickness_m` to `config.cast`,
update TOML schema, add regression test, update procedure.md
generation if the changed mold needs different workshop handling.

---

## 7. What stays banked

- **Option D (ribbed shell)** — defer unless Option A/B walls
  prove structurally inadequate.
- **Variable per-layer wall thickness** — different walls per
  layer based on layer's pour pressure (lower layers need
  thicker walls because they carry more cumulative weight).
  Probably overkill at iter-1 scale.
- **SLA-specific optimization** — SLA printers (SLS, MJF) have
  different min-wall constraints than FDM (1-2 mm vs 3-5 mm).
  Once the FDM target ships, generalize.
- **Pour-pressure FEM** — actual stress analysis of the mold
  under pour-pressure loading. Research-tier scope; banked
  unless a real failure surfaces.

---

## 8. Pointers

- `tools/cf-cast-cli/src/derive.rs:283-291` — the cuboid
  construction site
- `design/cf-cast/src/piece.rs:94` — `compose_piece_solid`
  consuming `bounding_region`
- `design/cf-cast/src/spec.rs:116` — `CastSpec.bounding_region`
  field
- `design/cf-cast/src/pour.rs` — `PourGateSpec` + gate
  construction (recon constraint check)
- `design/cf-cast/src/registration.rs` — `PinSpec` (recon
  constraint check)
- `design/cf-cast/src/plug.rs` — `PlugPinSpec` (recon constraint
  check)
- `tools/cf-cast-cli/src/config.rs:98` — `bounding_margin_m` (the
  current knob; will likely be renamed/replaced by
  `wall_thickness_m`)
- `docs/PROGRAM_GAMEPLAN.md` §4 B1 — parent gameplan ask

---

End of bookmark. Recon next session.
