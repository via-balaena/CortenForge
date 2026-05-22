# cf-cast mating-features — S0 bookmark

**Arc:** [`docs/CF_CAST_MATING_FEATURES_PLAN.md`](./CF_CAST_MATING_FEATURES_PLAN.md)
**Session:** S0 (Phase 1, bookmark — no production code).
**Predecessor:** PR #254 / `358d682c` shipped the iter-1 cast pipeline; its
workshop print physically falsified the mold's mating surfaces.

## What S0 ships

1. A walked inventory of every face-to-face mating contact across the 14 STLs
   `cf-cast-cli` emits for iter-1 — derived by reasoning about physical
   assembly, not by listing from memory (per the plan's G7).
2. A clearance-budget table mapping each mating pair to its current
   `*_slack_m` field (or absence thereof) and a proposed `*_clearance_m`
   target (per G6).
3. The S8 physical acceptance gate, quoted verbatim from the plan.

S0 is documentation only. No production code edits; the spec-field names
proposed in §"Clearance-budget table" are **proposals to be finalized by
S2 (recon) and shipped by S5/S6** — they are NOT being added to
`PinSpec`/`PlugPinSpec` this session.

## How iter-1 produces 14 STLs

`cf-cast-cli` consumes `~/scans/cast.toml` (workshop iter-1 spec) and emits,
into `~/scans/cast_iter1_design/`, the artifacts a single workshop pour
session uses:

| STL | Count | Source |
|-----|------:|--------|
| `mold_layer_N_piece_0.stl` (Negative side) | 4 (N ∈ {0..3}) | `spec.rs::mesh_and_gate_v2_piece` line 880 |
| `mold_layer_N_piece_1.stl` (Positive side) | 4 (N ∈ {0..3}) | same |
| `plug_layer_N.stl` | 4 | `spec.rs::mesh_and_gate_v2_plugs` line 934 |
| `platform.stl` | 1 | `spec.rs::mesh_and_gate_v2_platform` line 994 |
| `funnel.stl` | 1 | `spec.rs::mesh_and_gate_v2_funnel` line 1056 |

`procedure.md` (not an STL) is the workshop instruction sheet for the same set.

**Casting cadence (per `procedure.md`):** each layer is cast independently
against its own plug, then the cured silicone tubes nest post-cure. The
mold pieces from different layers **never touch each other during pour or
cure** — so layer-N cup ↔ layer-(N+1) cup contact is NOT a mating concern,
and the walk drops it from the inventory.

## Per-pour-session physical assembly

A single layer-N pour session involves seven printed parts:

```
                 funnel.stl
                     │ (nipple slides into pour-leg hole)
                     ▼
  mold_layer_N_piece_0  ─── seam ───  mold_layer_N_piece_1
   (Negative, vent leg)  registration  (Positive, pour leg)
          ▲                  pins              ▲
          │  (seam closes around plug T-bar)   │
          └─────────  plug_layer_N  ───────────┘
                     (T-bar in seam)
                          │
                          │ T-bar protrudes below cup outer face
                          ▼
                     platform.stl
                  (assembled mold rests on top;
                   T-bar drops into blind pocket)
```

Six classes of FDM-printed mating contact arise; the rest of this document
walks each in turn.

## Mating-feature inventory

Each entry below documents one contact class. "Instances" counts how many
copies of this contact exist across the iter-1 STL set.

### M1 — Cup seam plane (Negative ↔ Positive)

| Field | Value |
|---|---|
| **Partner pieces** | `mold_layer_N_piece_0` ↔ `mold_layer_N_piece_1` |
| **Instances** | 4 (one per layer) |
| **Current SDF encoding** | `piece.rs:103` ribbon half-space intersection; `piece.rs:64` `RIBBON_PIECE_OVERLAP_M = 0.0005 m` bias |
| **Observed defect (iter-1)** | "Mating faces between the two cup halves are not flat enough to seal silicone liquid-tight." |
| **Dominant mechanism** | A (CSG-not-C¹ at feature junctions — body cavity, pins, T-bar, pour-gate all kink the SDF at the seam plane) |
| **Target CAD primitive** | Exact plane (the ribbon plane through the cap-plane centroid, normal = ribbon binormal at apex). Plus a 0.5 mm post-MC inward bias if the architectural fix preserves the "MC never sees coincident surfaces" guarantee, OR zero bias if mesh-CSG handles coincident faces cleanly (S2 decides). |
| **Shared-primitive partner(s)** | This same plane (parent: `(point_on_plane, plane_normal)`) is consumed once per layer's piece-pair — the same primitive trims both Negative and Positive into matching faces. |
| **Current clearance** | **NEGATIVE 1.0 mm** (each side biased +0.5 mm inward, total seam overlap 1 mm — pieces interfere by 1 mm at the seam by design). The 1 mm overlap is an MC-numerics device per `piece.rs:51-64`, not a tolerance. |
| **Target clearance** | **0 mm (flush)** — seam is a sealing surface, not a slide-fit. Both faces trim exactly to the plane. |
| **Notes** | The seam plane is the single most load-bearing surface in the assembly: silicone leaks here defeat the entire pour. S4 ships this feature first. |

### M2 — Inter-piece registration pins / sockets

| Field | Value |
|---|---|
| **Partner pieces** | `mold_layer_N_piece_0` (pins unioned) ↔ `mold_layer_N_piece_1` (sockets subtracted) |
| **Instances** | 16 pairs (4 pin-pairs per layer × 4 layers) |
| **Current SDF encoding** | `piece.rs:121-127` unions/subtracts `registration::build_registration_solid` (`registration.rs:183`). Cylinder geometry: `registration.rs:217` `Solid::cylinder(pin_radius_m, pin_half_length_m).rotate(rotation).translate(pin_center)`. Iter-1 spec: `registration.rs:109-115` `PinSpec::iter1` → `pin_radius_m = 0.0015 m` (3 mm Ø), `pin_half_length_m = 0.005 m` (10 mm total length), `arc_fractions = vec![0.25, 0.75]` with bilateral mirror in `registration.rs:205-222` → **4 pin pairs per layer-piece-pair**. |
| **Observed defect (iter-1)** | "Interlocking registration-pin protrusion side is longer than the matching socket is deep." |
| **Dominant mechanism** | B (independent MC grids per piece — pin's terminal cap-disk lands at different cell offsets on each side; see `mesher.rs:33-35` `ScalarGrid::from_bounds(solid.bounds().min, ..., GRID_PADDING_CELLS=2)` — grid origin = `bounds.min - 2 * cell_size`, and the Negative-piece bounds include the pin protrusion while the Positive-piece bounds don't, so the two grid origins differ along the binormal). Also C (FDM bead-line tolerance on a 3 mm Ø pin / hole). |
| **Target CAD primitive** | Exact cylinder. Shared parent `(pin_center, pin_axis=binormal, pin_half_length_m)` — Negative pin uses parent radius; Positive socket uses parent radius + diametral clearance / 2, and parent half-length + axial clearance. |
| **Shared-primitive partner(s)** | One parent triple per pin instance; Negative-piece pin mesh + Positive-piece socket mesh both derive from it via deterministic CSG builders. Cross-piece test (per plan §S5) asserts `pin_cylinder_mesh_is_bit_equal_across_pieces` for the parent geometry. |
| **Current clearance** | **0 mm** — pin radius and socket radius are IDENTICAL, pin length and socket depth are IDENTICAL (the SDF encoding has no per-feature slack — the mating only works if the two MC grids land cells identically, which Mechanism B violates). |
| **Target clearance** | **0.20 mm diametral, 0.50 mm axial** (sliding press-fit — pieces must slide together perpendicular to seam; per plan §S5 baseline). |
| **Proposed spec fields (S5)** | `PinSpec::diametral_clearance_m`, `PinSpec::axial_clearance_m`. |
| **Notes** | The bilateral mirror (`registration.rs:205-222`) means each `arc_fraction` yields **two** pin pairs (one on each lateral side of the centerline along ±split_normal). With iter-1's two arc fractions, total = 4 pin pairs per layer-piece-pair, NOT the 2 that `procedure.rs:489` writes into `procedure.md` (`pin_count = spec.arc_fractions.len()` ignores the mirror — stale-doc bug; orthogonal to S0, log as a sweep candidate). |

### M3 — Plug T-bar / dual half-T-slot

| Field | Value |
|---|---|
| **Partner pieces** | `plug_layer_N` T-bar ↔ `mold_layer_N_piece_0` half-T-slot ↔ `mold_layer_N_piece_1` half-T-slot (**three pieces share one parent primitive**) |
| **Instances** | 4 layer-triplets (one per layer) |
| **Current SDF encoding** | Plug T-bar: `plug.rs:294-304` calls `build_t_bar_cylinder` (`plug.rs:414`) → `Solid::cylinder(t_bar_radius_m=0.003, t_bar_half_length_m=0.012)` rotated to axis `pour_outward × split_normal_vec`, unioned into the plug solid. Cup half-T-slot: `piece.rs:153` subtracts `plug::build_plug_socket_solid` (`plug.rs:325`), which calls the **same** `build_t_bar_cylinder` with radius `t_bar_radius_m + t_slot_radial_slack_m = 0.003 + 0.0001 = 0.0031 m`. Bisection across the seam is via `ribbon`-side intersection in `piece.rs:107` (T-bar axis lies in the seam plane). |
| **Observed defect (iter-1)** | "Plug T-bar does not fit into the T-slot carved in the cup pieces." |
| **Dominant mechanisms** | A + B + C — A (T-bar union into plug + T-slot subtraction from cup each glue cylinder SDF onto the plug-body / cup-body SDF), B (plug + Negative cup + Positive cup are THREE independent MC grids), C (FDM bead bulge at 0.1 mm diametral encoded clearance is already tighter than typical FDM precision). |
| **Target CAD primitive** | Exact cylinder, parent `(center=pour_anchor + pour_outward * pin_length_m, axis=pour_outward × split_normal, half_length=0.012)`. S2-G1 decision: default approach (a) — run the cylinder CSG **before** the seam trim from S4, so the seam trim bisects the meshed cylinder cleanly into two halves (alternative: build two half-cylinder primitives, one per piece, both deriving from the same parent triple + a side enum). |
| **Shared-primitive partner(s)** | One parent triple drives three meshes: plug T-bar (radius = parent r), Negative cup T-slot half (radius = parent r + diametral_clearance/2), Positive cup T-slot half (same). The bisection comes from the shared ribbon-plane CAD primitive (M1). |
| **Current clearance** | **0.2 mm diametral** (encoded as `t_slot_radial_slack_m = 0.0001 m` per side at `plug.rs:207`+`plug.rs:232`+`plug.rs:347`). **0 mm axial** (no end-face clearance on T-bar half-length — cup pieces close around the T-bar's flat circular ends with zero gap). |
| **Target clearance** | **0.30 mm diametral, 1.00 mm axial** (positional registration — plug drops into seam-open cup half, cup closes around it; captive insertion, no slide-fit needed; per plan §S6 baseline). |
| **Proposed spec fields (S6)** | `PlugPinSpec::t_bar_diametral_clearance_m`, `PlugPinSpec::t_bar_axial_clearance_m`. Existing `t_slot_radial_slack_m` (`plug.rs:207`) is **deleted** — replaced by the explicit diametral field. |

### M4 — Plug pin shaft / dual half-socket (cup-wall pour-end socket)

| Field | Value |
|---|---|
| **Partner pieces** | `plug_layer_N` pin shaft ↔ `mold_layer_N_piece_0` half-socket ↔ `mold_layer_N_piece_1` half-socket (three pieces, same pattern as M3) |
| **Instances** | 4 layer-triplets |
| **Current SDF encoding** | Plug pin: `plug.rs:293` calls `anchor_cylinder(pour.0, pour.1, pin_radius_m=0.003, pin_length_m=0.020)` (`plug.rs:521`) — cylinder anchored at cap-plane centroid extending outward along cap-plane normal. Cup socket: `piece.rs:153` subtracts `plug::build_plug_socket_solid` (`plug.rs:325`), which calls the same `anchor_cylinder` with radius `pin_radius_m + socket_radial_slack_m = 0.003 + 0.0005 = 0.0035 m`. iter-1 `cast.toml` overrides `pin_length_m` to a workshop-tuned value (the `[plug_pins]` table's defaults from `PlugPinSpec::iter1` apply when no override is set; the iter-1-shipped `procedure.md` reports "4.0 mm long pin" — derived clamp documented in `platform.rs:201`). |
| **Observed defect (iter-1)** | Latent — not in the user's three-defect list because pin_length≈4 mm makes engagement minimal at the 5 mm cup wall, so the misfit is hidden. At nominal 20 mm pin length (single-layer thick-wall cast), all three mechanisms would surface. |
| **Dominant mechanisms** | Same as M3: A + B + C. |
| **Target CAD primitive** | Exact cylinder, parent `(center=cap_centroid, axis=cap_outward_normal, length=pin_length_m)`. Sockets straddle the seam (same compose-on-both-pieces pattern as M3 + the pour-gate channel). Bisection: same approach-(a) recommendation as M3 (CSG before seam trim). |
| **Shared-primitive partner(s)** | One parent triple per layer drives three meshes: plug pin shaft (r = parent), Negative cup half-socket (r = parent + diametral/2), Positive cup half-socket (same). |
| **Current clearance** | **1.0 mm diametral** (`socket_radial_slack_m = 0.0005 m` per side at `plug.rs:227`+`plug.rs:332`; the wide slack is workshop-tuned for slide-fit insertion through the cup-wall socket while the cup is assembled — distinct from the M3 captive-insertion T-slot). **0 mm axial** (no end-clearance on pin length vs socket depth — the pin's shoulder rests on the cup wall outer face). |
| **Target clearance** | **0.30 mm diametral, 1.00 mm axial** (positional registration; per plan §S6 baseline). Diametral tightens vs current 1 mm; axial gains 1 mm of pocket-bottom relief. |
| **Proposed spec fields (S6)** | `PlugPinSpec::shaft_diametral_clearance_m`, `PlugPinSpec::shaft_axial_clearance_m`. Existing `socket_radial_slack_m` (`plug.rs:145`) is **deleted** — replaced by the explicit diametral field. |
| **Notes** | The current 1 mm diametral (`socket_radial_slack_m = 0.5 mm` per side) is too loose vs the plan's 0.30 mm target — pin wobbles in socket. Tightening will improve plug centering but requires the architectural fix to land first (without it, Mechanism B alone can swamp the tighter budget). |

### M5 — Pour-gate hole / funnel nipple + flange seat

| Field | Value |
|---|---|
| **Partner pieces** | `mold_layer_N_piece_1` (Positive cup) pour-leg hole ↔ `funnel.stl` nipple OD; `mold_layer_N_piece_1` outer face ↔ `funnel.stl` flange underside (resting contact) |
| **Instances** | 4 (nipple-into-hole + flange-on-outer-face per layer; one funnel STL serves all four pour sessions) |
| **Current SDF encoding** | Pour-leg cylinder: `piece.rs:139` subtracts `pour::build_pour_gate_solid` (`pour.rs:255`); `pour.rs::leg_cylinder` (`pour.rs:332`) builds `Solid::cylinder(spec.gate_radius_m=0.005, spec.gate_half_length_m=0.045)` (10 mm Ø, 90 mm long), axis = `cos30° * outward + sin30° * binormal` at the V apex. Funnel nipple: `funnel.rs:128` `nipple_outer_diameter = pour_gate_outer_diameter - FUNNEL_NIPPLE_SLACK_M = 0.010 - 0.0002 = 0.0098 m` (9.8 mm Ø). Funnel flange: `funnel.rs:73` `FUNNEL_FLANGE_OUTER_RADIUS_M = 0.010 m` (20 mm Ø disk, 1 mm thick). |
| **Observed defect (iter-1)** | Latent — not in user's defect list; the funnel was workshop-tested separately. |
| **Dominant mechanism** | C (FDM bead bulge on a 10 mm hole / 9.8 mm cylinder is the main concern). A + B are minor because the funnel is a single STL with its own grid and the hole CSG kink is far from the body cavity. |
| **Target CAD primitive** | Exact cylinder (the pour-leg axis is at 30° from outward — not axis-aligned with cardinal). Parent `(apex, pour_leg_axis=cos30°·outward+sin30°·binormal, gate_radius_m, gate_half_length_m)`. Funnel nipple = parent r − diametral_clearance/2; cup pour-hole = parent r + diametral_clearance/2. |
| **Shared-primitive partner(s)** | The pour-leg parent cylinder is shared between the cup-side carve (already exists) and the funnel-side nipple OD (currently *coincidentally* sized — `FUNNEL_NIPPLE_SLACK_M` is the only link). Post-fix, both meshes derive from one parent. |
| **Current clearance** | **0.2 mm diametral** (`FUNNEL_NIPPLE_SLACK_M = 0.0002 m` at `funnel.rs:65`). **0 mm axial** (flange seats directly on cup outer face). |
| **Target clearance** | **0.50 mm diametral** (resting contact — nipple drops loosely into hole; workshop user values visible slack over precision). **0.50 mm axial** (flange standoff above cup outer face — currently the flange contacts the MC stair-step jitter of the curved cup outer; a controlled standoff lets the flange seat cleanly on a tangent plane). Per plan §S0 baseline. |
| **Proposed spec field** | `funnel.rs::NIPPLE_DIAMETRAL_CLEARANCE_M` (replaces `FUNNEL_NIPPLE_SLACK_M`); new `funnel.rs::FLANGE_AXIAL_STANDOFF_M`. |
| **Notes** | Plan revision proposal: bump nipple diametral from current 0.2 mm to 0.50 mm. The 0.2 mm value was sized for "FDM holes print 0.1-0.2 mm undersized" (per `funnel.rs:60`), assuming the funnel nipple would print at nominal. That assumption is symmetric to M2: pin prints oversized + hole prints undersized → net interference ~0.3 mm at typical bead bulge. The 0.50 mm target absorbs this with workshop-friendly slop. |

### M6 — Plug T-bar / platform pocket (resting contact)

| Field | Value |
|---|---|
| **Partner pieces** | `plug_layer_N` T-bar protrusion ↔ `platform.stl` blind pocket |
| **Instances** | 4 (the platform STL's single pocket serves all four pour sessions; one T-bar per plug enters it during that layer's pour) |
| **Current SDF encoding** | T-bar geometry source: `plug.rs::pour_end_t_bar_geometry` (`plug.rs:374`) — returns parent `(center, axis, radius=0.003, half_length=0.012)`. Platform pocket: `platform.rs:163` builds `Solid::cuboid(half_x = t_bar_radius_m + PLATFORM_HOLE_LATERAL_SLACK_M=0.005, half_y = t_bar_half_length_m + PLATFORM_HOLE_LATERAL_SLACK_M=0.014, half_z = depth/2)` with `PLATFORM_HOLE_LATERAL_SLACK_M = 0.002 m` (per `platform.rs:80`), rotated to align with T-bar axis, blind pocket floor at `slab_top - pocket_depth` with `PLATFORM_POCKET_FLOOR_CLEARANCE_M = 0.005 m` (per `platform.rs:89`). |
| **Observed defect (iter-1)** | Not surfaced by user (no platform-specific defect reported). Generous slack means iter-1 likely works; mechanisms exist but are absorbed. |
| **Dominant mechanisms** | A + B + C, all dampened by the 2 mm lateral / 5 mm axial slack. |
| **Target CAD primitive** | Exact rectangular pocket (cuboid), CAD-derived from the same `pour_end_t_bar_geometry` parent triple that drives M3. |
| **Shared-primitive partner(s)** | Parent T-bar `(center, axis, radius, half_length)` already shared via `pour_end_t_bar_geometry` (`plug.rs:374`) — good. Post-S6, the pocket's lateral half-extents become `parent_radius + diametral_clearance/2` and `parent_half_length + axial_clearance/2`. |
| **Current clearance** | **4 mm diametral** (2 mm per side; `PLATFORM_HOLE_LATERAL_SLACK_M`), **5 mm axial** floor clearance (`PLATFORM_POCKET_FLOOR_CLEARANCE_M`). |
| **Target clearance** | **0.50 mm diametral minimum** for the architectural-correctness floor (plan §S0 resting-contact baseline). Workshop-ergonomics 2 mm lateral kept on top — the 0.50 mm floor is the minimum the geometry must produce, not the maximum it should produce; workshop sloppy-fit is fine here. **5 mm axial floor kept** (workshop-margin per `platform.rs:84`). |
| **Proposed spec fields (S7 sweep)** | Likely no spec change — current `PLATFORM_HOLE_LATERAL_SLACK_M` already exceeds the architectural minimum. S7 confirms the platform survives the M3 shared-primitive refactor unchanged. |
| **Notes** | Inventoried for completeness even though no defect is observed; the platform-pocket geometry depends on the same `pour_end_t_bar_geometry` parent as M3, so S6's refactor must preserve that wiring. |

## Out-of-scope contacts (walked + dropped)

The walk crossed these contacts; each is intentionally **NOT** in the
mating-features inventory:

- **Cup outer face ↔ platform top surface.** Cup outer is the offset of the
  curve-following body — not a flat plane. Workshop assembles with the T-bar
  in the pocket constraining pose; the cup hovers ~0.5 mm above the slab
  (`PLATFORM_TOP_GAP_M`). Not a tight-tolerance mating feature; gravity-rest
  with pose pinned by M6.
- **Plug body surface ↔ cup body cavity surface.** Same SDF on both sides
  (cured silicone press-fits between them). This is the *organic cavity*
  that the architectural fix explicitly preserves as SDF/MC. Out-of-scope
  for the CAD-exact migration by design.
- **Plug hemispherical cap ↔ cup cavity dome.** Same SDF on both sides; same
  organic-surface treatment.
- **Layer-N cup ↔ Layer-(N+1) cup contact face.** Procedure.md confirms each
  layer pours independently; mold pieces from different layers never touch.
- **Cured silicone tube ↔ next-layer cured silicone tube.** Post-cure nesting
  of cured silicone shells — flexibility absorbs interference; not an FDM
  printed-part mating concern.

## Clearance-budget table

Values quoted as **diametral × axial**, both in millimeters. "Pair" = the two
or three printed parts sharing one parent primitive.

| ID | Pair | Fit class | Current clearance | Target clearance | Proposed spec fields |
|---|---|---|---|---|---|
| M1 | cup ↔ cup (seam plane) | sealing surface | −1.00 (overlap) | 0.00 (flush) | none (geometric only — bias removed in S4) |
| M2 | pin ↔ socket (registration) | sliding press-fit | 0.00 | **0.20 × 0.50** | `PinSpec::diametral_clearance_m`, `PinSpec::axial_clearance_m` |
| M3 | T-bar ↔ T-slot (plug+cup+cup) | positional registration | 0.20 × 0.00 | **0.30 × 1.00** | `PlugPinSpec::t_bar_diametral_clearance_m`, `PlugPinSpec::t_bar_axial_clearance_m`; deletes `t_slot_radial_slack_m` |
| M4 | shaft ↔ cup-wall socket (plug+cup+cup) | positional registration | 1.00 × 0.00 | **0.30 × 1.00** | `PlugPinSpec::shaft_diametral_clearance_m`, `PlugPinSpec::shaft_axial_clearance_m`; deletes `socket_radial_slack_m` |
| M5 | funnel nipple ↔ pour-gate hole | resting contact | 0.20 × 0.00 | **0.50 × 0.50** | `funnel.rs::NIPPLE_DIAMETRAL_CLEARANCE_M`, `funnel.rs::FLANGE_AXIAL_STANDOFF_M`; replaces `FUNNEL_NIPPLE_SLACK_M` |
| M6 | T-bar ↔ platform pocket | resting contact | 4.00 × 5.00 (axial floor) | ≥**0.50 × 5.00** (architectural floor; current generous slack kept for workshop ergonomics) | none required |

### Baseline rationale (validating the plan's G6 proposals)

The plan §S0 proposes three fit classes; S0 confirms (with one revision
candidate) and documents the rationale by class.

**Sliding press-fit (M2): 0.20 mm diametral / 0.50 mm axial.**
The plan's Mechanism C analysis (Mechanism C of `CF_CAST_MATING_FEATURES_PLAN.md`)
predicts ~0.30 mm net diametral interference at default FDM settings (0.4 mm
nozzle, ~0.15 mm bead bulge per face × 2 faces). A 0.20 mm clearance budget
leaves ~−0.10 mm net interference at worst-case bulge — i.e., a snug press
fit, not binding. Hobbyist-FDM precision-sliding fit guidance commonly cites
0.20-0.30 mm at this nozzle size, so 0.20 mm is **at the tight end** of the
sound range. **Keep the plan's proposal; flag a print-trial in S1** to
empirically confirm a 0.20 mm-budget pin / socket pair doesn't bind on the
target printer + filament. If it binds, S2 recon revises up to 0.30 mm.

**Positional registration (M3, M4): 0.30 mm diametral / 1.00 mm axial.**
0.30 mm diametral is the FDM-precision-fit midpoint. Axial 1.00 mm is
generous — captive-insertion fits (M3) close around the part, and slide-fit
fits (M4) want pocket-bottom relief so the pin's shoulder seats on a
controlled flat (the cup outer face), not on the socket floor (which carries
MC stair-step). **Keep the plan's proposal.**

**Resting contact (M5, M6 architectural minimum): 0.50 mm.**
Workshop ergonomics — user wants visible play when landing the funnel into
the pour-gate hole and the assembled mold onto the platform. M5's current
0.20 mm is too tight under the same Mechanism-C logic above (net ~−0.10 mm
interference at the funnel-nipple / hole pair). **Keep the plan's 0.50 mm
proposal AND mark M5 as a candidate to update spec-side in S5/S6/S7.** M6's
current 4 mm/5 mm is generously above the architectural minimum — kept.

### Important non-revision: the asymmetric reason M4 ≠ M3

M3 and M4 share a fit class (positional registration) and a target
clearance (0.30 × 1.00 mm), but the current encodings are very different
(0.20 mm vs 1.00 mm diametral). The current 1.00 mm for M4 was chosen
because the workshop user **slides** the plug's pin into the assembled cup's
through-socket along the cap-plane normal — a slide-fit requires more slack
than M3's captive-insertion. The plan's recon must decide whether 0.30 mm
is right for both, or whether M4 stays at a looser ~0.50 mm slide-fit
clearance. **S2 (recon) escalation candidate** — flag for §"Clearance spec
fields (G6)".

## Acceptance gate (verbatim from plan §S0)

> Re-printed iter-1 mold halves seal flush, all pins and the T-bar drop
> into their matching pockets, calipers-verified ≤0.2 mm length/depth
> mismatch on each mating feature *after* spec clearance is subtracted.

S8 executes this against an iter-2 print of all 14 STLs.

## Open questions kicked to S2 (recon)

These S0 walks surfaced questions the recon should answer (a non-exhaustive
hook list — recon walks the whole spec).

1. **M4 fit class.** Current 1.00 mm diametral is a slide-fit (looser than
   plan's 0.30 mm positional baseline) because the plug is slid axially
   into an already-assembled cup. Either justify keeping the slide-fit (and
   add a fourth fit class to the table) or tighten to 0.30 mm with a print
   trial.
2. **M1 bias-vs-zero.** Does the post-MC mesh-CSG library handle two
   coincident half-space-trimmed faces cleanly (zero seam overlap), or does
   it need the 0.5 mm bias preserved as a *signed* trim offset? S1 spike
   exercises this directly.
3. **M5 funnel-flange standoff.** The 0.50 mm flange axial standoff
   proposed here is new; the current funnel rests flange-on-cup directly.
   Recon decides whether the standoff lives as a physical lip on the
   flange or whether the cup-outer-face mesh gets a local-flat patch under
   the flange footprint.
4. **Stale `procedure.md` pin count.** `procedure.rs:489` writes
   `pin_count = arc_fractions.len()` but the bilateral mirror in
   `registration.rs:205-222` produces 2× that. Doc-only bug, but on the S7
   sweep checklist as a "companion update" candidate.
5. **Spec-field naming drift vs plan §G6.** This bookmark proposes
   `PlugPinSpec::shaft_diametral_clearance_m` + `_axial_clearance_m` (split)
   and `funnel.rs::NIPPLE_DIAMETRAL_CLEARANCE_M` + `FLANGE_AXIAL_STANDOFF_M`,
   while plan §G6 lists `PlugPinSpec::shaft_clearance_m` (singular) and
   `funnel.rs::NIPPLE_CLEARANCE_M`. The split mirrors how M3 already splits
   `t_bar_diametral_clearance_m` + `t_bar_axial_clearance_m`, so the
   asymmetry within G6 itself is the source of drift. Recon picks one
   form consistently across all six clearance fields.

## Cross-references

- [`docs/CF_CAST_MATING_FEATURES_PLAN.md`](./CF_CAST_MATING_FEATURES_PLAN.md) — arc plan (source of truth for cadence).
- Memory: `project_cf_cast_mating_features_plan` (plan + cold-read trail).
- Memory: `project_workshop_iter1_cast_pipeline_pr254` (parent arc; PR #254 / `358d682c` shipped the iter-1 pipeline).
- Source files cited above (`piece.rs`, `registration.rs`, `plug.rs`, `pour.rs`, `funnel.rs`, `platform.rs`, `mesher.rs`, `spec.rs`, `procedure.rs`) under `design/cf-cast/src/`.
- iter-1 STLs at `~/scans/cast_iter1_design/` and config at `~/scans/cast.toml`.

## S1 — library spike decision memo

**Session:** S1 (Phase 1, library de-risk — no production code).
**Spike location (deleted at session end):** `~/spikes/cf-cast-mesh-csg-spike/`,
**940 LOC total** across `Cargo.toml` (32) + `src/main.rs` (599, of which
~300 is measurement/reporting infra) + `src/csgrs_lib.rs` (181) +
`src/manifold_lib.rs` (128), NOT a member of the cortenforge workspace
per plan §S1. (Plan §S1 budgeted ~200 LOC; the overshoot is dominated by
measurement scaffolding — welding, manifold-edge counter, F4 issue-type
tally, determinism re-run, comparison-table formatter — that the recon
will not need.)
**Verdict:** ✅ **Use `manifold3d` 0.1.8** for the post-MC mesh-CSG stage.
csgrs (BSP-based, pure Rust) is empirically falsified for this use case —
it produces non-manifold output on every operation against both fixtures.

### Method

The spike executed three operations — (a) plane-trim, (b) cylinder-
subtract, (c) cylinder-union — against two fixtures: a hand-built
10 mm cube uniformly subdivided to a 10×10 grid per face (6 faces × 200
triangles = 1200 triangles, similar density to a 10 mm cube MC-meshed
at 1 mm cells), and the real iter-1 Negative-piece STL (`~/scans/cast_iter1_design/mold_layer_0_piece_0.stl`,
15204 triangles, 7602 welded vertices, manifold-clean per a roll-your-own
`edges_with_two_faces == total_edges` check). Both fixtures were
**vertex-welded at 1e-6 mm tolerance** before being fed to either library
— a non-obvious requirement surfaced by manifold3d's input rejection of
unwelded STL data (see §"Findings worth banking", item 3).

For each (library, fixture, op) triple the spike measured: pass/fail,
output triangle count, wall-clock latency, output manifoldness (edges with
1 or 3+ incident faces), F4 grade via `mesh_printability::validate_for_printing`
under `PrinterConfig::fdm_default()` with issue-type tally, and
determinism (re-run + welded-output bit-hash compare).

Edge cases probed on the toy cube (plane-only): plane exactly on the
+X face, plane 0.1 µm inside the +X face (near-tangent), plane through
the +X+Y+Z corner vertex.

### Comparison table

Coordinates are mm. Per-op timings are release-build wall-clock; the
"open" column is welded-output edges with exactly one incident face
(non-zero ⇒ topological boundary, i.e., non-manifold for a solid).
"f4-types" shows the dominant `PrintIssueType` categories on F-grade
runs to distinguish topology bugs from print-feasibility flags.

| Library | Fixture | Op | OK | f_out | latency | open | 3+ | F4 | f4-top-types | Det |
|---|---|---|---|---|---|---|---|---|---|---|
| **csgrs** (git HEAD) | toy | plane-trim | ✓ | 602 | 13 ms | 44 | 0 | F | DetectorSkipped:2, NotWatertight:1, SelfIntersecting:1 | bit |
| csgrs | toy | cyl-sub | ✓ | 2080 | 12 ms | 182 | 0 | F | SmallFeature:2, NotWatertight:1, SelfIntersecting:1 | bit |
| csgrs | toy | cyl-uni | ✓ | 2080 | 9 ms | 182 | 0 | F | SmallFeature:2, NotWatertight:1, SelfIntersecting:1 | bit |
| csgrs | **real** | plane-trim | ✓ | 23759 | **801 ms** | **7541** | 0 | F | ExcessiveOverhang:35, LongBridge:25, SmallFeature:9 | bit |
| csgrs | **real** | cyl-sub | ✓ | 45102 | **773 ms** | **10540** | 0 | F | ExcessiveOverhang:22, LongBridge:3, SmallFeature:2 | bit |
| csgrs | **real** | cyl-uni | ✓ | 44730 | **767 ms** | **10522** | 0 | F | ExcessiveOverhang:21, LongBridge:3, SmallFeature:2 | bit |
| **manifold3d** 0.1.8 | toy | plane-trim | ✓ | 566 | 2 ms | **0** | 0 | **A** | — (zero issues) | bit |
| manifold3d | toy | cyl-sub | ✓ | 1310 | 1 ms | **0** | 0 | F | ExcessiveOverhang:1 | bit |
| manifold3d | toy | cyl-uni | ✓ | 1310 | 1 ms | **0** | 0 | F | ExcessiveOverhang:1 | bit |
| manifold3d | **real** | plane-trim | ✓ | 8668 | **7 ms** | **0** | 0 | F | ThinWall:33, ExcessiveOverhang:8, LongBridge:3 | bit |
| manifold3d | **real** | cyl-sub | ✓ | 15204 | **6 ms** | **0** | 0 | F | ExcessiveOverhang:14, LongBridge:3 | bit |
| manifold3d | **real** | cyl-uni | ✓ | 15328 | **5 ms** | **0** | 0 | F | ExcessiveOverhang:15, LongBridge:3, ThinWall:1 | bit |

**Edge cases (toy cube, plane-only):**

| Case | csgrs | manifold3d |
|---|---|---|
| Plane exactly on +X face | empty (0 v / 0 f) | empty (0 v / 0 f) |
| Plane 0.1 µm inside +X face | 966 v / 322 f, **966 open edges** | 125 v / 246 f, **0 open edges** |
| Plane through +X+Y+Z corner vertex | empty | empty |

### Verdict, by criterion

- **Correctness on edge cases.** Both libraries collapse trivially-empty
  cases to empty output. Only manifold3d handles near-tangent input
  cleanly; csgrs produces an open-boundary mesh whose edge count tracks
  the sliver region's boundary loop.
- **Output mesh manifoldness.** **manifold3d 6/6 zero open edges. csgrs
  6/6 non-zero open edges**, including 7541–10540 open edges on real-
  fixture operations even after output welding. This is the dispositive
  criterion — the architectural fix requires meshes that round-trip
  through STL export → cf-viewer → workshop print as closed solids.
- **F4 pass-through.** Both libraries score F on the real fixture under
  FDM defaults, but for different reasons. manifold3d's F-grades are
  dominated by `ThinWall` (intrinsic to the iter-1 piece's 5 mm
  `wall_thickness_m` near the pour-leg) and `ExcessiveOverhang`
  (intrinsic to the curve-following body geometry) — exactly the
  "expected to flake" cases plan §G5 explicitly accepts. csgrs's
  F-grades on real also surface `NotWatertight` + `SelfIntersecting` on
  the toy fixture and produce ~3× the triangle count of manifold3d on
  the cyl ops — symptoms of broken-topology output, not print-
  feasibility. The toy plane-trim under manifold3d hits F4 grade A
  (zero issues) — proof that manifold3d's CSG produces print-ready
  topology when the source geometry permits.
- **Build cost.** manifold3d 0.1.8 adds **14 transitive packages**
  including the C++ `manifold-csg-sys` (CMake + `cc`); from-scratch
  `cargo check --release` of a stub binary depending only on
  manifold3d took **3 min 14 s** (one-time C++ compile; cached after).
  csgrs at git HEAD adds **268 transitive packages** under default
  features (truck-* CAD kernel, bevy mesh adapters, image I/O, font
  handling, parry3d/rapier3d, etc.). Even with `default-features = false`
  csgrs's transitive set substantially exceeds manifold3d's.
- **License.** manifold3d: `Apache-2.0 OR MIT` (same for
  manifold-csg-sys, manifold-csg). csgrs: `MIT`. Both compatible with
  cortenforge.
- **Determinism (G3).** Both bit-deterministic across runs when output
  is welded to 1e-6 mm. csgrs's RAW output exhibited vertex-order
  drift across runs (BSP clip order varies), masked by welding. Plan
  §S2 shared-primitive invariant test should hash on welded form.

### Findings worth banking

1. **csgrs is empirically unsuitable for cf-cast's post-MC stage.**
   The BSP-based kernel's output is *structurally* non-manifold on both
   fixtures, both operations, after welding. This is not a tuning issue
   ("set tolerance lower") — BSP boolean output is known to produce
   T-junctions and degenerate triangles that no post-process fixes. Do
   not revisit csgrs for this use case in S2.

2. **`csgrs` 0.20.1 on crates.io is uninstallable** as of 2026-05-22 —
   it depends on yanked `core2 0.4.0` (confirmed `error: failed to
   select a version for the requirement core2 = "^0.4"` from
   `cargo add csgrs@0.20.1`). The git HEAD works but pins
   in-development versions of `i_overlay`/`i_float`/`i_shape`. This is a
   maintenance-signal supplemental data point reinforcing the verdict.

3. **manifold3d requires welded input.** `Manifold::from_mesh_f64`
   rejects unwelded STL data with `CsgError::ManifoldStatus(NotManifold)`.
   Production `solid_to_mm_mesh` already emits shared-index output, so
   this is a non-issue for the live pipeline; but any spike or test
   that round-trips through STL (`mesh-io::load_mesh`) must weld
   before constructing a `Manifold`. The welding pass is ~25 LOC + a
   `HashMap<(i64,i64,i64), u32>` keyed on coordinates quantized at
   1 µm (1e-6 mm; effectively bit-exact for the spike's input range).
   Bank this for the S3 plumbing refactor — the geometric-equivalence
   test helper plan §G2 mandates should likewise weld before comparing.

4. **manifold3d 0.1.8 is pre-1.0.** This is the principal residual risk.
   Mitigations: (a) the underlying C++ `manifold` kernel (`elalish/manifold`
   on GitHub) is a mature project — manifold3d 0.1.8 wraps
   `manifold-csg-sys` v3.4.107 of the C++ library — so the unstable
   layer is the Rust binding, not the kernel; (b) the Rust binding API
   surface cf-cast needs is small (`from_mesh_f64` + `to_mesh_f64` +
   `union`/`difference` + `cube` + `cylinder` + `translate` + `rotate`),
   so binding-API churn is bounded; (c) the binding is `Apache-2.0 OR MIT`
   and the source is readable (single-file `lib.rs` re-export of
   `manifold-csg`, which is itself ~2000 LOC across a handful of modules
   per the local `~/.cargo/registry/...` snapshot). If the 0.1.x line
   stalls or breaks, the minimum-viable fork is small. **Plan §S2 should
   record a contingency note:** pin manifold3d in the workspace
   `Cargo.toml` to an exact patch version; do not chase float-version
   ranges until the crate hits 1.0.

5. **manifold3d API needs an axis-orientation helper for cf-cast.**
   manifold3d's cylinder primitive is Z-axis-aligned only
   (`Manifold::cylinder(height, r_low, r_high, segments, center)`);
   axis-rotated cylinders require a `rotate(x_deg, y_deg, z_deg)`
   call. The spike used `nalgebra::Rotation3::rotation_between` →
   euler-XYZ-degrees as the bridge. **S3 should expose, for S5/S6 to
   consume, a `build_cylinder_along_axis(...) -> Manifold` helper at the
   mesh-CSG plumbing layer** so callers don't reimplement the
   axis-to-euler conversion six times. Same helper pattern for the
   half-space slab used by plane-trim (S4).

6. **manifold3d's transform API is euler-XYZ-degrees, not matrices**
   (it also has a `transform(&[f64; 12])` 4×3 affine form that's more
   precise — confirmed by inspecting the `manifold-csg` `Manifold::transform`
   method signature). For the
   shared-primitive invariant (one cylinder mesh used on both Negative
   and Positive sides), prefer the 4×3 affine form to keep round-off
   identical across the pair. The spike used eulers and still hit
   bit-determinism on the welded output, but the affine route is the
   safer default for the production shared-primitive cookbook.

7. **Real iter-1 Negative-piece is manifold-clean after welding.**
   15204 triangles, 7602 welded vertices, 22806 edges all with exactly
   two incident faces. This confirms the architectural assumption that
   the *current* SDF/MC output is sound topology — the iter-1 mating-
   surface defects are not "MC produces bad topology" but the three
   mechanisms (A CSG-not-C¹, B independent MC grids, C FDM bead
   tolerance) the plan diagnoses. Cf-cast's pipeline is producing
   manifold input for the post-MC CSG stage to consume.

### Spike code disposition

The spike crate at `~/spikes/cf-cast-mesh-csg-spike/` is deleted at
session end per plan §S1 ("Spike code thrown away. The two finalists'
deltas … go in the memo as a comparison table the recon references."),
along with its `target/` directory. The full run output is preserved in
this memo's table; raw stdout is at `/tmp/spike_final_results.log`
(also ephemeral — `/tmp/` is not persisted across reboots, but the table
above captures every measurement the recon needs).

### Bail-out branches — disposition

The plan §S1 bail-outs do **not** fire:

- ✗ "Both libraries produce F4-failing meshes on the real fixture" —
  *literally* true at the F-grade level, but manifold3d's F-grade is
  pure FDM-feasibility (`ThinWall` / `ExcessiveOverhang` / `LongBridge`)
  — issue types that flag intrinsic geometric properties of the input
  mesh (thin walls, steep slopes), not topology faults the CSG stage
  introduces. Manifold3d *preserves* the input mesh's geometric character
  rather than degrading it (zero open edges, zero 3+ edges, output
  triangle count within ~10% of input). Plan §G5 explicitly accepts this.
  Do not escalate to the hand-rolled-fallback session.
- ✗ Hand-rolled out of scope / no acceptable library → "pivot to Route B"
  — does not fire; manifold3d is acceptable.

### Open questions kicked to S2 (recon)

- **S2-A — Floating-point precision.** manifold3d's `from_mesh_f64`
  takes f64 vertices but `manifold-csg-sys` internally uses what the
  C++ Manifold project chooses (Manifold has had a historical f32-
  default that was lifted to f64 — confirm the v3.4.107 sys-crate is
  built against f64, not f32 with a precision-losing round-trip).
  S2-recon line item: read `manifold-csg-sys` build.rs + verify in the
  cmake configure of the C++ kernel.
- **S2-B — Affine vs euler shared-primitive cookbook.** Recon picks
  one transform path for the shared-primitive invariant and documents
  the helper signatures S3 introduces. Recommendation: 4×3 affine.
- **S2-C — `mesh-printability` welding inside the boundary?** The S3
  plumbing pipeline should weld each piece's mesh before mesh-CSG; the
  shared-primitive mesh is built directly from `Manifold::cylinder` so
  is already welded. Open: does the *output* of `Manifold::to_mesh_f64`
  ever need re-welding before STL save / F4 evaluation? The spike data
  (output `dv0` across re-runs, zero open edges) says no, but S2
  should confirm by reading the `to_mesh_f64` impl.

### Status — S1

- **2026-05-22 — S1 spike executed.** ~940 LOC throwaway crate; twelve
  (lib×fixture×op) triples (3 ops × 2 fixtures × 2 libs) + six
  (lib×plane-edge-case) probes (3 cases × 2 libs). Verdict:
  manifold3d 0.1.8. Spike code deleted; only this memo persists.

## Status

- **2026-05-22 — S0 bookmark drafted.** Inventory + clearance table +
  acceptance gate shipped. No production code changes.
- **2026-05-22 — S1 spike + ADR.** manifold3d 0.1.8 chosen; csgrs
  empirically falsified (non-manifold output on every operation, both
  fixtures). Spike code thrown away. S2 (recon) is the next session.
