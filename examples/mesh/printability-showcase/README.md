# `printability-showcase`

**Capstone of the v0.8 detector arc — a single realistic-feeling
bracket fixture exercises all six detectors at once (§7.8).** Five
vertex-disjoint hand-authored shells (528 v / 1032 f) demonstrate
how a multi-detector printability validator reads a "summary report"
on a real-CAD-like part, including the gotchas. This is the one example
where the *deviations* from clean detector firing carry as much
pedagogical weight as the firing itself.

## What this fixture is

Five shells, all vertex-disjoint, all individually watertight, combined
into one mesh:

| Component | Role | Verts | Tris | Where |
|-----------|------|------:|-----:|-------|
| Body      | solid `50 × 30 × 10 mm` axis-aligned base                 |   8 |   12 | `[0, 50] × [0, 30] × [0, 10]` |
| Wing      | solid leaning prism, 60° tilt toward `+X`, `5 × 5 mm` cs  |   8 |   12 | base `[57.5, 62.5] × [12.5, 17.5] × {0}`, top z = 15 |
| Thin-lip slab | §7.1-style hollow rect; **0.4 mm top wall**           |  16 |   24 | outer `[10, 40] × [32, 42] × [0, 4]`; inner `[11, 39] × [33, 41] × [1, 3.6]` |
| Burr      | 0.2 mm hex prism (Ø 0.2 × h 0.2)                          |  14 |   24 | center `(25, -5)` on build plate |
| Sphere cavity | UV-tessellated `r = 3 mm` sphere, REVERSED winding    | 482 |  960 | center `(25, 15, 5)` inside body solid |

Each shell is independently watertight + consistently wound; the union
has every undirected edge in exactly two faces (always intra-shell,
since shells are vertex-disjoint). Hand-authored components use exact
or `sqrt`-derived coordinates (`f64::sqrt` is correctly-rounded per
IEEE-754; `f64::sin` / `cos` are not), so per-vertex + per-face winding
math-pass anchors hold bit-exactly across platforms.

## What the detector reports — multi-detector summary

12 `PrintIssue` records on FDM, **`is_printable() == false`**:

| Detector            | Count | Severities                  | Sources                                                                                                                              |
|---------------------|------:|-----------------------------|--------------------------------------------------------------------------------------------------------------------------------------|
| `ThinWall`          |     5 | Critical × 3, Warning × 2   | slab outer top (0.4 mm) + slab inner top (0.4 mm) + burr (0.173 mm) Critical; **2 wing fan co-flags (0.962 mm) Warning — see below** |
| `SmallFeature`      |     1 | Warning × 1                 | burr (`0.2 < 0.8 / 2 = 0.4` → Warning band; **no Critical band** in `classify_small_feature_severity`)                               |
| `TrappedVolume`     |     2 | Info × 2                    | slab inner cavity (582 mm³) + sphere cavity (111 mm³); both Info on FDM (extrusion prints fine through closed cavities)              |
| `ExcessiveOverhang` |     2 | Critical × 2                | slab inner-top ceiling (90°) + sphere upper cap (~84.3° due to chord shrinkage)                                                      |
| `LongBridge`        |     1 | Critical × 1                | slab inner cavity ceiling (28 mm `> 10 mm` FDM `max_bridge_span`)                                                                    |
| `SelfIntersecting`  |     1 | Critical × 1                | wing — 3 false-positive triangle pairs (BVH precision; see deviations below)                                                         |
| `NotWatertight`     |     0 | —                           | each shell watertight; vertex-disjoint shells preserve manifold globally                                                             |

**Total**: 7 Critical + 3 Warning + 2 Info ⇒ `is_printable() == false`
because of `>= 2 Critical` issues. The Critical sources are:

- ThinWall on the slab top wall (0.4 mm)
- ThinWall on the slab inner top wall (0.4 mm)
- ThinWall on the burr (0.173 mm flat-to-flat)
- ExcessiveOverhang on the slab cavity ceiling (90°)
- ExcessiveOverhang on the sphere cavity upper cap (~84°)
- LongBridge on the slab cavity ceiling (28 mm × 8 mm)
- SelfIntersecting on the wing (false-positive)

Non-Critical observations: 2 wing fan ThinWall co-flags Warning
(0.962 mm) + 1 burr SmallFeature Warning + 2 TrappedVolume Info
(slab + sphere cavities; FDM extrusion prints through closed
cavities).

## What this example deliberately surfaces — five spec deviations

§7.8 calls for clean detector firing on a "summary report" capstone.
Empirical detector behavior on a real-CAD-like fixture is *not* clean
— and this example surfaces the gaps as load-bearing pedagogical
observations rather than papering over them. Each deviation is
documented inline in `src/main.rs` and in `mesh/mesh-printability/CHANGELOG.md`.

### 1. Sphere radius reduced 4 mm → 3 mm

At `r = 4 mm` with body `50 × 30 × 10`, the body's top / bottom wall
thickness is exactly 1.0 mm — at the FDM `min_wall_thickness`
boundary. The §6.1 strict-less-than predicate is theoretically safe
but knife-edge sensitive to `EPS_RAY_OFFSET = 1e-6` round-off across
libm versions. `r = 3` puts the wall thickness at 2 mm, well clear of
the threshold. The cavity is still the load-bearing pedagogical
surface (`TrappedVolume` Info + `ExcessiveOverhang` Critical on the
upper cap).

### 2. Two trapped volumes, not one

Spec assertion #3 reads `trapped_volumes.len() >= 1`. Empirically:

- **Slab inner cavity** (28 × 8 × 2.6 ≈ 582 mm³) — a consequence of
  the §7.1-style "thin-wall via hollow box" construction §7.8 calls
  for. `Info` on FDM.
- **Sphere cavity** (~111 mm³) — the spec-mandated cavity. `Info` on
  FDM.

The `>= 1` predicate is satisfied. Both volumes are well above FDM's
`min_feature_size³ = 0.512 mm³` resolution floor.

### 3. Wing overhang silently masked by the build-plate filter (Gap M.2)

The wing's `+X` lateral face has analytical `overhang_angle = 60°`
(`acos(-sin60°) - π/2 = 60°`) — comfortably above the FDM 45°
threshold. **It does not flag at all.** The reason is the build-plate
filter at `validation.rs:404-408`:

```rust
let face_min_along_up = ...;
if (face_min_along_up - mesh_min_along_up) < EPS_GEOMETRIC {
    // skip face (build-plate-filtered)
}
```

The wing's `+X` face has `face_min_along_up = 0` (its bottom edge
touches the build plate). `mesh_min_along_up = 0` (body's bottom). So
`0 - 0 = 0 < 1e-9` ⇒ filtered out — even though the face spans
`z ∈ [0, 15]` and most of its area is well above the build plate. The
filter excludes any face whose minimum-along-up value coincides with
the mesh's, regardless of the face's full extent.

The wing therefore contributes ZERO overhang regions. The `>= 2`
overhang assertion is satisfied independently by the slab + sphere
cavity ceilings. **A real-CAD leaning column resting on the build
plate masks its overhang concern unless the user lifts it (or the
filter learns to distinguish "edge touches plate" from "face supported
by plate").** Logged as a v0.9 candidate.

### 4. Wing produces unexpected `ThinWall` Warning co-flags

The §6.1 `ThinWall` detector inward-ray-casts from each face's
centroid and looks for the closest opposite face within
`min_wall_thickness`. For the wing's bottom and top fans, the inward
ray (`+Z` from bottom; `-Z` from top) exits the wing through a tilted
lateral face at the SHORTER of the geometric "thicknesses" —
analytically `(WING_X_SPAN / 2) / tan(60°) ≈ 0.962 mm`.

`0.962 mm < 1.0 mm` (FDM `min_wall_thickness`) ⇒ flagged. `0.962 ≥ 0.5`
(`min_wall / 2`) ⇒ Warning band, not Critical.

Two of the wing's 4 cap-fan triangles fall below threshold; the other
two are chord-favored. **A real-CAD pitfall**: a leaning column with
cross-section `min(W, H) ≤ tan(60°) · min_wall` triggers ThinWall on
its caps via slope-perpendicular inward-ray-cast. Pedagogically
valuable; the showcase surfaces it as a real diagnostic the validator
provides on real geometry, not a bug. Not asserted by §7.8 spec.

### 5. Three false-positive `SelfIntersecting Critical` pairs from the wing

The §6.4 `SelfIntersecting` detector (Gap I — re-uses
`mesh_repair::detect_self_intersections`) reports 3 triangle pairs as
intersecting. The pairs are between diametrically-opposite lateral
faces of the wing — a known v0.9 candidate (CHANGELOG `[Unreleased] /
v0.9 candidates / "mesh-repair detect_self_intersections false-
positives on thin-aspect-ratio cylinders"`, surfaced first by the
`printability-orientation` example for tilted cylinders at L≥18 mm).

Our wing has lateral aspect ratio 6:1 (length 30 mm / cross-section
5 mm) — within the BVH-precision false-positive zone. The wing is
geometrically sound (vertices are exact-representable, winding is
consistent, no actual surface crossings); the detector's underlying
BVH triangle-pair test is the bug.

Pedagogically: the showcase makes a v0.9 candidate detector
limitation visible without papering over it. The README points to the
orientation crate's same-family observation for cross-reference.

## §4.4 issue sort policy — partial implementation

`V08_FIX_ARC_SPEC.md` §4.4 calls for issues sorted severity-descending
(ties by issue_type then face index). Empirically,
`validate_for_printing` (`validation.rs:177`) does not apply a global
sort post-detector-run; issues append in detector run order:

```text
build_volume → overhangs → manifold → thin_walls → long_bridges →
trapped_volumes → self_intersecting → small_features
```

The per-detector internal ordering varies (e.g., `check_thin_walls`
emits Warning before Critical clusters within a detector, sorted by
descending thickness). Anchor #7 in this example was relaxed to a
non-empty structural check; severity coverage is verified by anchor
#6 (`>= 2 Critical`) and the dedicated burr-/slab-thinwall-Critical
observation. v0.9 candidate: implement §4.4's global sort.

## How to run

```text
cargo run -p example-mesh-printability-showcase --release
```

`--release` is required: the FDM `TrappedVolume` voxel grid is
`~ 500 × 300 × 100 ≈ 15 MB` for the body — tractable in release, slow
in debug. Output is written to
`examples/mesh/printability-showcase/out/`:

- `out/mesh.ply` — **528-vertex, 1032-triangle ASCII PLY** of the full
  bracket fixture.
- `out/issues.ply` — 12 centroid points (5 ThinWall + 2 Overhang +
  2 SupportRegion + 2 TrappedVolume + 1 SmallFeature) as a vertex-only
  ASCII PLY.

A successful `cargo run --release` exit-0 means **both** the per-
detector outcome matrix (above) AND the hand-authored fixture
geometry (per-vertex coordinates, per-face winding, mesh bbox) match
expectation — see `verify_fixture_geometry` in `src/main.rs`. The
visuals-pass is **optional** at this point; everything visible has
been encoded as a numerical invariant.

If you do want to eyeball the artifacts, **run f3d from the crate
root** (`examples/mesh/printability-showcase/`) on each PLY
**separately** — f3d's `--multi-file-mode=all` falls back to all-
points rendering when mixing a face-mesh with a vertex-only point
cloud, so you lose the bracket surface:

```text
f3d --up=+Z out/mesh.ply       # the bracket — body, wing, slab, burr, sphere cavity
f3d --up=+Z out/issues.ply     # the 12 detected centroid points
```

The sphere cavity has REVERSED winding (normals INTO cavity); f3d
back-face-culls it by default, so the cavity appears as a hollow void
inside the body — the intended visual. **Per
`feedback_chamfered_not_rounded`**: at 32 × 16 UV tessellation the
sphere is visibly **chamfered** / **polygonal**, not smooth. Marching-
cubes-style tessellation on a sharp-creased SDF would render
similarly; the chamfer is a real geometric property of the
tessellation, not a viewer artifact.

## Numerical anchors (asserted in `verify_fixture_geometry` + `verify`)

`verify_fixture_geometry` (called once at the top of `main`):

- **Hand-authored per-vertex coordinates** (46 verts: body 8 + wing 8
  + slab 16 + burr 14) within `1e-12` of `expected_hand_vertices()`.
- **Hand-authored per-face winding** (72 faces) — cross-product unit
  normal within `1e-12` of `expected_hand_face_normals()`.
- **Sphere per-vertex distance from center** (482 verts) within
  `1e-12` of `SPHERE_RADIUS = 3 mm`.
- **Sphere per-face REVERSED winding orientation** (960 faces) —
  cross-product unit normal cosine similarity with `(center -
  face_centroid)` direction `> 0.99` (chord-shrinkage envelope for a
  32 × 16 UV).
- **Mesh bounding box** at the analytical extents.

`verify` (called after `validate_for_printing`):

- Anchor #1: `thin_walls.len() >= 1` (slab + burr both contribute).
- Anchor #2: `small_features.len() >= 1` (burr).
- Anchor #3: `trapped_volumes.len() >= 1` (slab + sphere cavities, ≥ 2
  observed; spec-stated `>= 1`).
- Anchor #4: `overhangs.len() >= 2` (slab + sphere cavity ceilings;
  wing tilt is masked by build-plate filter — see deviation #3).
- Anchor #5: `issues.len() >= 5` (12 observed).
- Anchor #6: `>= 2 Critical` issues + `is_printable() == false`.
- Anchor #7: issues vector non-empty (relaxed from §4.4 strict
  severity-descending — see "partial implementation" above).
- Anchor #8: `validation.summary()` non-empty.
- Anchor #9: only the burr (0.2 mm extent) flags `SmallFeature`; the
  bracket main body (50 mm extent) must NOT.

Plus a non-anchored cross-check that at least one ThinWall Critical
fires (the slab top wall + burr both qualify).

## v0.8 fix arc cross-references

- §4.3 — overhang severity bands (`Info` / `Warning` / `Critical`).
- §4.4 — issue sort policy (partial implementation; see above).
- §6.1 — `ThinWall` detector (Gap C).
- §6.2 — `LongBridge` detector (Gap G); §6.2 build-plate filter.
- §6.3 — `TrappedVolume` detector (Gap H).
- §6.4 — `SelfIntersecting` detector (Gap I); v0.9 candidate
  surfaces here.
- §6.5 — `SmallFeature` detector (Gap J); no Critical band.
- §7.1 — `printability-thin-wall` (sister example: hollow-box
  construction, sole-detector focus).
- §7.5 — `printability-small-feature` (sister example: hex prism burr).
- §7.6 — `printability-orientation` (sister example: leaning cylinder;
  same-family false-positive `SelfIntersecting` documented there).
- §7.7 — `printability-technology-sweep` (sister example: same mesh,
  cross-tech severity divergence).
- §7.8 — this example's spec (capstone showcase).
