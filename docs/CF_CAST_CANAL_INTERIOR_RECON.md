# CF-CAST Canal Interior Recon

> **Status:** RECON SCAFFOLD (pre-implementation). Cold-read pass-1 included at end (§12).
> **Date:** 2026-05-28
> **Trigger:** Design conversation — "we are not sure that just the negative of a
> penis is the best shape for the inside of a pocket pussy." Workshop user wants
> to explore a *designed* interior canal rather than the literal scan negative.
> **Direction locked (this session):** asymmetric / frenulum-targeted canal;
> scan = length/girth **budget only** (not the cavity shape).
> **Sequencing:** this arc composes on top of the shipped §M unified-mating-plane
> + §B bolt-pattern + iter-1 print-ready state (`main` = `4734ce08`). It does **not**
> touch the mold-cup / ribbon / flange / dowel / bolt subsystems — it replaces the
> **layer-0 plug** only.

---

## 1. Problem statement

The current pipeline produces an interior cavity that is the **exact negative of
the scanned object** (the penis/capsule), optionally inset inward by a uniform
`cavity_inset_m`. Both the plug and every layer body are `pinned_floor_shell`
offsets of the one shared scan SDF, built inline in
`derive_spec_and_ribbon` (`tools/cf-cast-cli/src/derive.rs`):

```rust
// plug = scan surface shrunk inward by the press-fit reservation     (derive.rs:257)
let plug = pinned_floor_shell(closed_sdf_arc, open_sdf, bounds, caps, -cavity_inset_m);
// layer N body = scan offset OUTWARD by cumulative thickness          (derive.rs:275)
let body = pinned_floor_shell(closed_sdf_arc, open_sdf, bounds, caps,
                              cumulative_so_far - cavity_inset_m);
// bounding_region = outermost body grown outward by wall thickness    (derive.rs:304)
```

When `cavity_inset_m == 0` and there are no cap planes, `pinned_floor_shell`
degenerates to `Solid::from_sdf(scan).offset(...)` bit-for-bit — i.e. **the plug
is the scan literal**.

The cavity (`= negative of the plug`) is therefore the penis shape verbatim. That
is **not** what high-quality strokers use, for physical reasons:

1. **Interference fit is the whole point.** A 1:1 negative has zero interference;
   the soft silicone stretches away under insertion and feels loose/numb.
   Functional designs deliberately undersize the canal so the material grips.
2. **Sensation comes from variation along the length**, not from anatomical
   match — tight rings, open chambers, ribs, asymmetric texture.
3. **Nerve density is concentrated** at the frenulum (underside of glans), the
   corona (ridge), and the glans tip. A uniform negative spreads stimulation
   over mostly-low-sensitivity surface.

**Design target:** replace the layer-0 plug with a **parametric canal** whose
*negative* is the designed interior, sized to the scan's length + girth so it
still fits the intended user, with asymmetric (frenulum-biased) texture.

---

## 2. Current architecture (as found)

### 2.1 The plug → cavity flow

| Concept | What it is | Source |
|---|---|---|
| **scan SDF** | flood-fill-signed SDF of the cleaned scan STL | `cf-cast-cli/src/lib.rs:129-149` |
| **plug (layer 0)** | `pinned_floor_shell(scan, -cavity_inset_m)` — the positive that forms the innermost cavity | `derive.rs:257` |
| **plug (layer N>0)** | `layers[N-1].body` — previous layer's outer surface | `spec.rs:1128-1132` |
| **layer body N** | `pinned_floor_shell(scan, +cumulative_thickness)` — cumulative outer surface | `derive.rs:275` |
| **layer N silicone shell** | `layer_body[N] − plug_for_layer[N]` (CSG subtract) | `spec.rs` `compute_pour_volumes` |
| **interior cavity** | negative of the layer-0 plug | (emergent) |

Everything is an **offset of the scan**. The cavity is the penis negative because
the plug is the scan.

### 2.2 The SDF system (what we build the canal with)

- **Core trait** `cf-geometry/src/sdf.rs:82` — `fn eval(&self, p: Point3<f64>) -> f64`
  (negative inside, positive outside) + `fn grad(...)`. `Send + Sync`.
- **`Solid`** (`cf-design/src/solid.rs`) is the high-level wrapper / expression
  tree. Bridge a hand-written SDF in via **`Solid::from_sdf(sdf, bounds)`** — the
  mesher takes the gradient by finite differences, so a custom `grad` can return
  an arbitrary unit vector.
- **Combinators:** `union`/`subtract`/`intersect` (+ `smooth_*` variants),
  `translate`/`rotate`/`scale_uniform`/`mirror`, `shell`/`offset`/`round`,
  `twist`/`bend`/`elongate`, `repeat_bounded`. All methods on `Solid`.
- **Relevant primitives:** `Loft` (variable **circular** cross-section along Z via
  Catmull-Rom radius stations — `solid.rs:331`), `Capsule`, `Cylinder`, `Torus`
  (for rings). **No existing non-circular / D-shaped / angularly-textured
  cross-section** — that is the novel part we must hand-write.
- **Custom-SDF precedent:** `FlangeSdf` (`cf-cast/src/flange.rs:238-275`) and
  `CupWallShellSdf` (`piece.rs`) — both implement `Sdf`, compose distance terms
  via `min`/`max`, and are wrapped with `Solid::from_sdf`. This is exactly the
  pattern the canal follows.

### 2.3 SDF → mesh

Marching cubes at `mesh_cell_size_m` (default **3 mm**, `config.rs`). Dual
contouring / adaptive-octree DC also available (`cf-design/src/mesher.rs`) for
sharp features. **⚠️ 3 mm cells will obliterate ~1.5 mm texture** — see §7.

### 2.4 Config plumbing (the feature-flag template)

`cast.toml` → `CastConfig` (`config.rs`) → `CastSpec` (`derive.rs`). The
`scan_mesh_direct_plug_layer_0` flag is the exact template to copy: a
`#[serde(default)]` field on the config struct, lifted in `derive.rs` behind a
safety gate, consumed in `spec.rs`. Existing geometry sections: `[scan] [cast]
[[layers]] [plug_pins] [pour_gate] [gasket] [flange] [dowel_hole] [bolt_pattern]`.

---

## 3. What the canal must produce

The canal is a **positive solid** (the plug). Its *negative* is the cavity. The
working zones, expressed along the insertion axis `z` (0 = mouth, increasing in),
as fractions of the budgeted length `L`:

| Zone | z range | Role | Geometry |
|---|---|---|---|
| 1. Entry ring | 0 – 0.10 L | corona "catch" | tight annular constriction, `r ≈ 0.45·r_p` |
| 2. Clearance chamber | 0.10 – 0.18 L | let corona clear | opens to `r ≈ 0.95·r_p` |
| 3. Stimulation zone | 0.18 – 0.60 L | the main event | asymmetric D-section + 2–3 secondary rings + frenulum-biased texture |
| 4. Collapsing taper | 0.60 – 0.92 L | gentle full contact | `r` tapers below `r_p` |
| 5. Suction chamber | 0.92 – 1.0 L | pneumatic pull | near-closed bulb + tunable vent |

`r_p(z)` = the **girth budget** = local insert radius read from the scan (§5).

---

## 4. Proposed architecture

### 4.1 `CanalPlugSdf` — a new hand-written SDF

New module `design/cf-cast/src/canal.rs`. Struct implements `Sdf`; wrapped with
`Solid::from_sdf` and substituted for `spec.plug` at the layer-0 seam.

```rust
/// Parametric interior-canal plug. Its NEGATIVE is the designed cavity.
/// Axis-aligned to a straightened centerline frame (see §5.3); the result
/// is transformed back onto the scan centerline before meshing.
struct CanalPlugSdf {
    length_m: f64,                 // L — budget from scan centerline arc length
    radius_profile: RadiusLut,     // r_p(z) — girth budget from scan (§5)
    zones: ZoneProfile,            // entry ring, chambers, taper, suction (§3)
    frenulum_dir: UnitVector3<f64>,// +y in canal frame — asymmetry axis (§4.3)
    asymmetry: AsymmetryParams,    // D-section + directional texture (§4.3)
    texture: TextureParams,        // rib/nub amplitude, pitch, kind
}

impl Sdf for CanalPlugSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        // 1. map p into canal frame -> (z, theta, r_xy)
        // 2. base canal radius R(z) from zones * radius_profile
        // 3. angular compression: R(z, theta) via D-section blend (§4.3)
        // 4. additive texture gated by directional weight w(theta) (§4.3)
        // 5. signed distance = r_xy - R(z, theta) - texture(z, theta), capped by ends
    }
    fn grad(&self, _p) -> Vector3<f64> { self.frenulum_dir.into_inner() } // FD fallback
}
```

### 4.2 Where it slots in

**Seam: the `let plug = pinned_floor_shell(...)` line at `derive.rs:257`** (the
cleanest point — the canal *is* the layer-0 plug). When `[canal].enabled`, build
`Solid::from_sdf(CanalPlugSdf{..}, bounds)` instead of the `pinned_floor_shell`
offset. Everything downstream (`spec.rs:1128-1140` layer-0 plug selection,
`add_plug_pins`, mesh, F4 gate, STL write) is unchanged — the plug is just a
different `Solid`. Note the canal also needs the scan AABB / centerline already in
scope here, which `derive_spec_and_ribbon` has (`scan_aabb`, `centerline`).

- Layers N>0 are **untouched** — they still derive from `layers[N-1].body`
  (scan-offset outward). The canal affects the innermost cavity only.
- `cavity_inset_m` becomes **inert** when the canal is enabled (the canal defines
  its own inner surface). Gate: warn if both set.
- **Mutually exclusive with `scan_mesh_direct_plug_layer_0`** — both target the
  layer-0 plug. Hard error if both enabled.

### 4.3 The asymmetry (the differentiator)

Two stacked mechanisms in `eval`, both keyed on the azimuth `θ` measured from
`frenulum_dir`:

1. **D-shaped cross-section.** Blend the radius between a tight frenulum-side
   value and a looser dorsal-side value:
   `R(z, θ) = R(z) · lerp(dorsal_factor, frenulum_factor, ½(1 + cos θ))`
   with starting `frenulum_factor ≈ 0.50`, `dorsal_factor ≈ 0.80`.
2. **Directional texture bias.** Texture amplitude scaled by
   `w(θ) = max(0, cos θ)` — full amplitude on the frenulum side, fading to smooth
   dorsally. Saves compression budget + casting complexity on the low-sensitivity
   wall.

Asymmetry is free peak-pleasure **because a handheld stroker has controlled
orientation** — the user aligns `frenulum_dir` to the underside.

### 4.4 Texture

Procedural additive term, e.g. annular ribs `A·sin(2π z / pitch)` and/or helical
ribs `A·sin(k₁z + k₂θ)`, gated by `w(θ)`. Starting `A ≈ 1.5 mm`. **Resolution
matters** — see §7 (needs sub-mm cells or adaptive DC on the plug only).

### 4.5 Config surface (`[canal]`)

```toml
[canal]
enabled              = true
# girth budget
girth_scale          = 1.0      # multiply scan-derived r_p(z) before zone factors
# zones (fractions of budgeted length L)
entry_ring_frac      = 0.45     # ×r_p at the catch ring
clearance_open_frac  = 0.95     # ×r_p in the clearance chamber
frenulum_factor      = 0.50     # ×r_p, frenulum wall, stim zone
dorsal_factor        = 0.80     # ×r_p, dorsal wall, stim zone
secondary_ring_count = 3
suction_chamber_frac = 0.08     # terminal length fraction
vent_diameter_m      = 0.002
# texture
texture_amplitude_m  = 0.0015
texture_pitch_m      = 0.008
texture_kind         = "annular"   # annular | helical | nubs
# orientation
frenulum_dir         = [0.0, 1.0, 0.0]   # in scan frame
# meshing override (texture needs finer cells than the 3 mm default)
plug_mesh_cell_size_m = 0.0005
```

Plumb identically to `scan_mesh_direct_plug_layer_0`: `#[serde(default)]` struct
in `config.rs`, lift in `derive.rs`, consume at the plug seam.

---

## 5. Extracting the budget from the scan

The scan provides **length** and **girth** only — not shape.

### 5.1 Length budget
Already available: `Ribbon::arc_length()` (`ribbon.rs:591`) over the centerline
polyline parsed from `[centerline].points_m` in the prep TOML. `L = arc_length`.

### 5.2 Girth budget — `RadiusLut` (NEW)
There is **no per-z radius profile today** (the flange-continuity bookmark already
flagged "per-Z radius LUT" as a missing primitive). Build it by sampling the scan
SDF radially at stations along the centerline:

- For each station `z_i` along the centerline, with frame `(tangent, normal,
  binormal)`, ray-march / bisect the scan SDF outward in the cross-section plane
  at several azimuths; take the mean (or per-azimuth) zero-crossing radius.
- Store as `RadiusLut { z: Vec<f64>, r: Vec<f64> }`, interpolated linearly.

This LUT is reusable — it also unblocks the flange-continuity 2D-distance fix
(separate arc). Worth landing as its own small primitive in S1.

### 5.3 Straightened-frame mapping
The centerline may curve. Build the canal in a **straightened** `z`-axis frame,
then map query points `p` → canal frame by arc-length projection onto the
centerline (reuse the ribbon's per-segment tangent/binormal). This keeps the zone
math 1-D while honoring the scan's curvature. (Simplest S1 fallback: assume a
straight centerline and gate on `max_tangent_rotation` being small — most socks
are near-straight. Curved support is an S-phase upgrade.)

---

## 6. Validation gates (NEW constraints the canal introduces)

### 6.1 Canal ⊂ body (wall-thickness floor) — **CRITICAL**
The outer housing is still `scan.offset(+thickness)`. The canal is parametric and
*independent*. If the canal radius anywhere exceeds `body_radius − min_wall`, the
silicone wall blows out to zero. Gate: sample canal surface vs `layers[0].body`,
require wall ≥ configurable floor (mirror the existing F4 `piece_min_wall_mm`
pattern). This is the canal analogue of the existing wall gates.

### 6.2 Plug pull-out from cured silicone — **CRITICAL, NEW**
Today the plug is convex (the scan) → trivially withdrawn after cure. A canal with
rings / chambers / a suction bulb has **undercuts**: the rigid FDM plug must be
pulled out through a narrower neck. This *works* for soft silicone (Ecoflex 00-30
~900% elongation, Dragon Skin ~1000% — the silicone stretches over the bumps),
but only up to a ratio. A fully-enclosed or steeply-re-entrant suction bulb may be
**non-demoldable with a single rigid core**.

Gate: compute max undercut ratio (chamber/bulb radius ÷ preceding neck radius) and
warn/refuse above a configurable threshold tied to material elongation. The
terminal suction chamber is the highest-risk feature — it may need to be (a)
amplitude-limited, (b) a soluble/meltable core (out of scope iter-1), or (c)
formed as a separate cast. **This constraint is invisible to a straight-cylinder
test fixture** — needs a chambered fixture (cf. the project's "load-bearing test
fixtures" lesson; the flange C-shape precedent).

### 6.3 Demoldability of the mold cup (unchanged)
The ribbon split / `max_tangent_rotation ≤ 120°` gate is about rigid-cup pieces
and is **unaffected** — the canal lives inside the plug, not the cup.

---

## 7. Resolution / meshing concern

`mesh_cell_size_m` default is **3 mm**; texture amplitude is ~1.5 mm and rings are
sub-cm. Marching cubes at 3 mm would erase all of it. Options:

1. **Per-plug fine cells** — mesh the layer-0 plug at `plug_mesh_cell_size_m`
   (e.g. 0.5 mm) while the cups stay at 3 mm. The plug bounding box is small, so
   the cost is bounded. (Requires threading a per-plug cell size into
   `mesh_solid_to_mm` at `spec.rs:1136` — currently global `spec.mesh_cell_size_m`.)
2. **Adaptive / dual contouring** for the plug — `mesh_adaptive` preserves sharp
   ring edges with far fewer cells (`mesher.rs`). Better for crisp rings; verify
   it handles the thin texture.

S0 spike should measure plug mesh time + face count at 0.5 mm and confirm texture
survives.

---

## 8. Phasing

| Phase | Scope | Deliverable |
|---|---|---|
| **S0** | Empirical spike (`#[ignore]` test, à la `s0_scan_mesh_direct_probe.rs`): hand-build a `CanalPlugSdf` with a couple of zones + texture, mesh at 0.5 mm, eyeball in cf-view, measure time/faces. Validate the SDF pattern + resolution before committing config surface. | probe test + decision on mesher (MC fine vs adaptive DC) |
| **S1** | `RadiusLut` primitive (§5.2) + length/girth budget extraction, with unit tests on a synthetic capsule scan. No canal yet — just the budget. (Also unblocks flange-continuity fix.) | `canal::RadiusLut` + tests |
| **S2** | `CanalPlugSdf` core: zones + circular `R(z)` (entry ring, clearance, taper) — **symmetric first**, no texture, no suction bulb. Wire `[canal]` config + the layer-0 seam. Wall-thickness gate (§6.1). | end-to-end canal cast (smooth, symmetric) |
| **S3** | Asymmetry: D-section + directional texture (§4.3–4.4) + fine/adaptive meshing (§7). | frenulum-targeted textured canal |
| **S4** | Suction chamber + vent + pull-out undercut gate (§6.2), with a chambered test fixture. | full design + demoldability gate |
| **S5** | Physical print + cast iteration; tune knobs against real material. | empirical convergence |

S0 is the immediate next action. Each code phase: cold-read pass + full gates
(`cargo xtask grade-all`) like every prior cf-cast arc.

---

## 9. Risks / unknowns

1. **Pull-out demoldability (§6.2)** is the biggest design risk and is genuinely
   empirical (depends on material + undercut geometry). May cap how aggressive the
   chambers/suction can be in iter-1.
2. **Resolution cost (§7)** — fine plug meshing could be slow; adaptive DC
   behavior on thin texture is unverified.
3. **Curved centerline (§5.3)** — straightened-frame mapping adds complexity;
   S1 can assume near-straight and gate on it.
4. **Wall blowout (§6.1)** — aggressive girth scaling could violate min wall
   against a slim scan; the gate catches it but may force conservative defaults.

---

## 10. Out of scope

- Mold-cup / ribbon / flange / dowel / bolt subsystems (untouched).
- Layers N>0 (untouched).
- Soluble/meltable cores for deep undercuts (future, if §6.2 forces it).
- cf-device-design `[design]`-path integration — start with inline `[[layers]]`.

## 11. Open questions for workshop user

- **OQ1 — frenulum orientation marker.** The asymmetry needs the user to orient
  the toy. Add a physical keying feature (flat / notch on the housing) so
  `frenulum_dir` is unambiguous in use? Or document orientation only?
- **OQ2 — suction aggressiveness vs demoldability.** How much suction chamber is
  worth the pull-out risk for iter-1? Start conservative (shallow bulb) and ramp?
- **OQ3 — girth scaling default.** Should the canal default to `girth_scale = 1.0`
  (canal at scan girth, interference from zone factors only) or pre-shrink the
  whole canal (`< 1.0`) for baseline grip independent of zones?
- **OQ4 — keep `cavity_inset_m`?** It goes inert under the canal. Delete the knob
  for the canal path, or leave it for the legacy negative path? (cf. project
  preference: strip the knob when the default works.)

---

## 12. Cold-read pass-1

- **C1 (corrected inline).** Initial draft slotted the canal at `spec.rs:1128`
  (the layer-0 plug *selection*) and named a `build_plug_solid` helper that does
  not exist. The plug is actually built inline as
  `let plug = pinned_floor_shell(..., -cavity_inset_m)` at `derive.rs:257`; that
  is where the scan-as-plug decision is *made*, and `spec.rs` then consumes
  whatever `Solid` it's given. Substituting there keeps `spec.rs` untouched. §4.2
  reflects this.
- **C2 (load-bearing).** First draft treated the cavity change as "just swap the
  plug." Re-reading `derive.rs:112` shows the **outer body is also scan-derived**
  (offset outward). So changing only the plug yields **variable wall thickness**
  between the designed canal and the penis-shaped housing — which is *fine and
  desirable*, but mandates the canal-⊂-body wall gate (§6.1). Without it, an
  aggressive canal silently blows out the wall.
- **C3 (the real surprise).** The plug-pull-out-from-silicone constraint (§6.2)
  does not exist today because the scan plug is convex. The canal makes it a
  first-class, possibly *blocking* constraint, and a straight-cylinder fixture
  cannot see it. Flagged CRITICAL and given its own phase (S4) + fixture
  requirement. This is the most likely thing to derail an over-ambitious iter-1.
- **C4 (resolution).** Easy to forget that the 3 mm production cell size erases
  the texture the whole design is *about*. §7 makes per-plug fine/adaptive meshing
  a named requirement, not an afterthought, and S0 measures it first.
- **C5 (scope honesty).** `RadiusLut` (§5.2) is genuinely new infrastructure, not
  a trivial read — there is no per-z radius today. Phased as S1 on its own, and
  noted as dual-use (also unblocks the flange-continuity 2D-distance fix).
- **C6 (mutual exclusion).** Canal and `scan_mesh_direct_plug_layer_0` both
  rewrite the layer-0 plug; §4.2 makes them a hard error together rather than
  silently letting one win.

---

## 13. S0 implementation spec (verified APIs — turnkey for next session)

All signatures below were read from the live tree this session, so the spike
should compile first-try. **S0 is a throwaway `#[ignore]`-gated probe** in
`design/cf-cast/tests/s0_canal_probe.rs` — no production code, fully deletable,
mirrors the existing `s0_scan_mesh_direct_probe.rs` scaffold.

### 13.1 Confirmed APIs
- **Sdf trait** (`cf-geometry/src/sdf.rs:82`, re-exported as `cf_design::Sdf`):
  `fn eval(&self, p: Point3<f64>) -> f64` + `fn grad(&self, p: Point3<f64>) -> Vector3<f64>`.
  Negative inside. `grad` can return an arbitrary unit vector (mesher uses finite
  differences via `from_sdf`).
- **Bridge to Solid:** `Solid::from_sdf<S: Sdf + 'static>(sdf: S, bounds: Aabb) -> Solid`
  (`cf-design/src/solid.rs:1105`).
- **SDF → mm mesh:** `cf_cast::mesher::solid_to_mm_mesh(&solid, cell_size_m, target)
  -> Result<IndexedMesh, CastError>` (`mesher.rs:28`) — samples a `ScalarGrid`,
  runs marching cubes, scales m→mm. `target` = `CastTarget::Plug { layer_index: Some(0) }`.
  ⚠️ `mesher` is currently a **private** module; the spike is an integration test
  (separate crate) so it needs either (a) `pub use` of `solid_to_mm_mesh` — check
  `lib.rs` exports; or (b) replicate the ~20-line MC loop inline in the probe
  (`ScalarGrid::from_bounds` + `marching_cubes` from `mesh_offset`, both already
  cf-cast deps). **(b) is the cleaner throwaway** — no production surface change.
- **Mesh type:** `mesh_types::IndexedMesh { pub vertices: Vec<Point3<f64>>,
  pub faces: Vec<[u32;3]> }` (CCW winding). Same type as `cf_geometry::IndexedMesh`.
- **STL write:** `mesh_io::save_stl(&mesh, path, binary: bool) -> IoResult<()>`
  (`mesh/mesh-io/src/stl.rs:283`).
- **Adaptive mesher (for the resolution comparison):** `solid.mesh_adaptive(tolerance)
  -> AttributedMesh` (`solid.rs:1239`); `AttributedMesh { pub vertices, pub faces }`
  (`mesh-types/src/attributed.rs:74`) — convert to `IndexedMesh` by copying both
  fields (same shapes). `mesh()` is uniform MC, `mesh_adaptive()` is octree DC.
- **Self-intersection check (reuse from the existing probe):**
  `mesh_repair::intersect::{detect_self_intersections, IntersectionParams}`.

### 13.2 What S0 builds
A minimal hand-written `CanalProbeSdf` (probe-local, *not* the production
`CanalPlugSdf`) exercising the three things that must work:
1. **Axial zones** — a circular `R(z)` with one entry-ring constriction + one
   clearance chamber + a taper. Confirms the SDF pattern meshes to a clean plug.
2. **Asymmetry** — D-section term keyed on azimuth from a `frenulum_dir`. Confirms
   non-circular cross-section meshes without artifacts.
3. **Texture** — additive `A·sin(2π z/pitch)` gated by `w(θ)=max(0,cosθ)`, `A=1.5mm`.
   Confirms texture *survives* the mesher.

Build it straight (centerline = z-axis), e.g. `length_m=0.12`, base `r_p=0.018`.
No scan input needed → S0 has **zero external dependencies** (unlike the
scan-mesh probe which needs `~/scans/...`). This makes it trivially runnable.

### 13.3 What S0 measures / emits (3 questions to answer)
- **Q-resolution:** mesh the same `CanalProbeSdf` at `cell_size_m ∈ {0.003 (prod),
  0.001, 0.0005}` via uniform MC, and once via `mesh_adaptive`. Report wall-clock,
  face count, and self-intersection count per setting. **Decision:** does the
  1.5 mm texture survive at 0.5 mm? Is adaptive DC cheaper for equal crispness?
  → picks the production mesher for S3.
- **Q-meshes-clean:** assert non-empty + watertight-ish (self-int count) for the
  D-section + texture combo. **Decision:** is the hand-written SDF pattern sound
  before building `RadiusLut` + config in S1/S2?
- **Q-pull-out (the showstopper):** emit `canal_probe.stl` to `~/scans/canal_s0/`
  with a **deliberate undercut** (an extra mid-canal chamber wider than its neck,
  ratio ~1.3–1.5). Workshop user prints it, pours a cup of scrap Ecoflex/Dragon
  Skin around it, and tries to pull the rigid plug out. **Decision:** does
  soft-silicone elongation tolerate the undercut, and at what ratio does it tear?
  This empirically bounds how aggressive S4's chambers/suction can be — and is the
  one thing no amount of code can answer.

### 13.4 Run command
```
cargo test --release -p cf-cast --test s0_canal_probe -- --ignored --nocapture
```

### 13.5 Risk-averse sequencing reminder
Do **not** start S1 (`RadiusLut`) or any config/`derive.rs` wiring until S0's
Q-pull-out physical test has come back. If undercuts tear at low ratios, the whole
zone model (rings/chambers/suction) needs rethinking *before* infrastructure is
built — that is the entire point of front-loading S0.
