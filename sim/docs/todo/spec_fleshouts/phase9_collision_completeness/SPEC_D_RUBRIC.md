# Spec D — Convex Collision Solver Completeness: Quality Rubric

Grades the Spec D spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

The umbrella spec (PHASE9_UMBRELLA.md), session plan (SESSION_PLAN.md), and
future work entry (§50) all describe "conservative-advancement CCD" and
"time-of-impact estimation" for tunneling prevention. Empirical verification
against MuJoCo's C source reveals this is **fundamentally incorrect**.

**MuJoCo's "CCD" is NOT time-of-impact continuous collision detection.** The
acronym stands for "Convex Collision Detection" — the name originates from
the `libccd` library that MuJoCo optionally uses as a fallback convex solver.
The `ccd_iterations`, `ccd_tolerance`, `nativeccd`, and `multiccd` parameters
all relate to the convex collision **penetration solver** (GJK/EPA equivalent),
NOT to tunneling prevention.

MuJoCo does NOT implement conservative advancement or time-of-impact
estimation anywhere in its codebase. Tunneling prevention relies on:
1. Soft contacts (spring-damper overlap resolution)
2. Small enough timesteps
3. The margin system (detecting near-contacts before overlap)

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| "Conservative-advancement CCD for convex geom pairs" | MuJoCo has NO conservative advancement. `mjc_penetration()` is a convex-convex penetration solver (GJK/EPA equivalent), not a time-of-impact algorithm. | **Drop** — redefine scope as convex collision solver completeness |
| "Time-of-impact estimation using upper-bound velocity and GJK separation distance" | No time-of-impact computation exists in MuJoCo. | **Drop** — not a MuJoCo feature |
| "GJK distance query for conservative advancement inner loop" | GJK distance query EXISTS in MuJoCo but is used for **margin-zone contact generation** (detecting contacts when shapes are close but not overlapping), NOT for conservative advancement. | **Keep, reframe** — GJK distance query is still needed, but for margin-zone contacts |
| "`ccd_iterations` controls max bisection steps" | `ccd_iterations` controls max iterations of the convex penetration solver (GJK/EPA). MuJoCo header: "maximum number of convex collision solver iterations". Default: **35** (empirically verified). | **Keep** — wire to GJK/EPA iteration limits |
| "`ccd_tolerance` — distance threshold for CCD contact" | `ccd_tolerance` is the convergence tolerance of the convex collision solver. MuJoCo header: "convex collision solver tolerance". | **Keep** — wire to GJK/EPA convergence tolerance |
| "`nativeccd` / `multiccd` enable flags" | `nativeccd`: choose MuJoCo native solver vs libccd fallback. `multiccd`: multi-point contact generation for flat convex surfaces. | **Keep** — `nativeccd` is a conformant no-op (no libccd); `multiccd` is algorithmic work |
| "Velocity threshold for CCD pair selection" | No velocity-based CCD filtering in MuJoCo. | **Drop** — does not exist |
| "Multi-point CCD for flat surfaces (MULTICCD flag)" | MULTICCD generates multiple contact points from a single convex-convex pair by running the penetration solver with multiple initial search directions. This IS real and IS algorithmic. | **In scope** — primary algorithmic deliverable |

**Final scope:**
1. **GJK distance query** — extend `gjk_epa.rs` with `gjk_distance()` for
   non-overlapping convex shapes. Returns separation distance + closest
   points. Needed for margin-zone contact generation (currently missing for
   all GJK/EPA pairs).
2. **Margin-zone contact generation** — modify `narrow.rs` GJK/EPA path to
   use `gjk_distance()` when shapes don't overlap but are within margin
   distance. This closes a conformance gap where GJK/EPA pairs miss
   near-contact events that analytical pairs detect.
3. **Parse `ccd_tolerance`** from `<option>` — add to `MjcfOption` and
   `Model` options.
4. **Wire `ccd_iterations`** — thread from `Model` to `gjk_query()` and
   `epa_query()`, replacing hardcoded `GJK_MAX_ITERATIONS=64` and
   `EPA_MAX_ITERATIONS=64` (MuJoCo default: 35).
5. **Wire `ccd_tolerance`** — thread from `Model` to `epa_query()` and
   `gjk_distance()`, replacing hardcoded `EPA_TOLERANCE=1e-6`.
6. **`DISABLE_NATIVECCD`** — CortenForge has one convex solver (GJK/EPA).
   Flag is a conformant no-op (no libccd to fall back to). Remove existing
   `tracing::warn!` guard, keep flag storage for model fidelity.
7. **`ENABLE_MULTICCD`** — Multi-point contact generation for convex-convex
   pairs. Run GJK/EPA with multiple initial search directions to find
   additional contact points on flat surfaces. Remove existing
   `tracing::warn!` guard. This is the primary algorithmic deliverable.

**Items NOT in scope:**
- Conservative advancement: not a MuJoCo feature
- Time-of-impact estimation: not a MuJoCo feature
- Velocity-based pair filtering: not a MuJoCo feature
- Implementing libccd as a fallback: CortenForge's GJK/EPA IS the solver

---

## Empirical Ground Truth

### MuJoCo behavioral verification

MuJoCo version: 3.5.0 (headers from pip package `mujoco==3.5.0`).
Verified by reading C headers directly. C source functions referenced from
prior Phase 9 work (Spec C rubric EGT-9 references `mjc_penetration`).

### Codebase context

| File | Lines | Role | Spec D impact |
|------|-------|------|---------------|
| `sim/L0/core/src/gjk_epa.rs` | 1,425 | GJK/EPA intersection + penetration | Add `gjk_distance()` function; make `gjk_query()`/`epa_query()` accept configurable iterations/tolerance |
| `sim/L0/core/src/collision/narrow.rs` | 339 | Narrowphase dispatch | Modify GJK/EPA fallback path (lines 186-198) to use `gjk_distance()` for margin-zone contacts; add MULTICCD multi-contact path |
| `sim/L0/core/src/collision/mod.rs` | 1,099 | Broadphase + collision loop | Thread `ccd_iterations`/`ccd_tolerance` to narrowphase calls; handle multi-contact returns from MULTICCD |
| `sim/L0/core/src/collision_shape.rs` | 1,771 | CollisionShape + support function | Read-only consumer — `gjk_distance()` uses existing `support()` |
| `sim/L0/mjcf/src/types.rs` | 4,241 | MjcfOption, MjcfFlag | Add `ccd_tolerance: f64` to `MjcfOption` |
| `sim/L0/mjcf/src/parser.rs` | 5,949 | MJCF parser | Parse `ccd_tolerance` from `<option>` |
| `sim/L0/mjcf/src/config.rs` | ~170 | MjcfConfig | Add `ccd_tolerance` field |
| `sim/L0/core/src/types/model.rs` | ~1,300 | Model struct | Add `ccd_iterations: usize` and `ccd_tolerance: f64` to Model options |
| `sim/L0/mjcf/src/builder/mod.rs` | ~940 | Builder | Transfer `ccd_iterations`/`ccd_tolerance` to Model; remove `tracing::warn!` guards for nativeccd/multiccd |
| `sim/L0/core/src/collision/hfield.rs` | 1,026 | Hfield collision | Consumer of `gjk_epa_contact` — will benefit from configurable iterations/tolerance but does not need code changes (parameters threaded via function signature change) |
| `sim/L0/core/src/collision/mesh_collide.rs` | ~150 | Mesh collision dispatch | Calls `gjk_epa_contact` for mesh-mesh hull pairs (line 60) — needs config params, margin-zone fallback, AND MULTICCD support (this is the empirically verified MULTICCD path from EGT-7) |

**Exhaustive match/call sites that need updating:**
- `gjk_epa.rs:67` — `GJK_MAX_ITERATIONS = 64` → must accept parameter
- `gjk_epa.rs:70` — `EPA_MAX_ITERATIONS = 64` → must accept parameter
- `gjk_epa.rs:76` — `EPA_TOLERANCE = 1e-6` → must accept parameter
- `gjk_epa.rs:595` — `gjk_query()` signature → add `max_iterations` param
- `gjk_epa.rs:830` — `epa_query()` signature → add `max_iterations`, `tolerance` params
- `gjk_epa.rs:1078` — `gjk_epa_contact()` signature → add config params
- `narrow.rs:186` — `gjk_epa_contact(...)` call → pass config params
- `narrow.rs:186-198` — GJK/EPA path → add `gjk_distance()` fallback for margin-zone
- `mesh_collide.rs:60` — `gjk_epa_contact(...)` call → pass config params + margin-zone fallback + MULTICCD multi-contact (this is the primary MULTICCD path — EGT-7 verified mesh-mesh)
- `hfield.rs:162-163` — `gjk_epa_contact(...)` call → pass config params
- `builder/mod.rs:933-938` — MULTICCD/NATIVECCD `tracing::warn!` → remove
- `config.rs:155` — `ccd_enabled()` → dead code (never called anywhere), may be removed or wired into collision pipeline if needed

### EGT-1: MuJoCo `ccd_iterations` and `ccd_tolerance` semantics

**MuJoCo source:** `mjmodel.h` → `mjOption` struct

```c
mjtNum ccd_tolerance;           // convex collision solver tolerance
int ccd_iterations;             // maximum number of convex collision solver iterations
```

**Key insight from header comments:** These parameters control the convex
collision **solver** (not a time-of-impact algorithm). They map directly to
GJK/EPA iteration limits and convergence tolerance.

**MuJoCo defaults (verified empirically via Python API, MuJoCo 3.5.0):**
- `ccd_iterations`: **35** (NOT 50 — CortenForge's default of 50 is WRONG)
- `ccd_tolerance`: `1e-6`

**CortenForge current state:**
- `ccd_iterations`: Parsed in `parser.rs:253-254`, stored in `MjcfOption`
  (types.rs:422, **default 50 — INCORRECT, MuJoCo default is 35**),
  transferred to `MjcfConfig` (config.rs:70, 116). NOT on `Model` — does
  not reach the collision pipeline.
- `ccd_tolerance`: NOT parsed, NOT stored anywhere.

**CortenForge GJK/EPA current constants:**
- `GJK_MAX_ITERATIONS = 64` (gjk_epa.rs:67) — MuJoCo default is 35
- `EPA_MAX_ITERATIONS = 64` (gjk_epa.rs:70) — MuJoCo default is 35
- `EPA_TOLERANCE = 1e-6` (gjk_epa.rs:76) — matches MuJoCo default

### EGT-2: `DISABLE_NATIVECCD` and `ENABLE_MULTICCD` flag semantics

**MuJoCo source:** `mjmodel.h` → disable/enable flag enums

```c
mjDSBL_NATIVECCD    = 1<<17,    // native convex collision detection
mjENBL_MULTICCD     = 1<<4,     // multi-point convex collision detection
```

**`nativeccd` (default: enabled):** Controls whether MuJoCo uses its native
convex collision implementation or falls back to the `libccd` library. Both
produce the same result (convex-convex penetration), just with different
implementations. In CortenForge, there is no `libccd` — our GJK/EPA IS the
native solver. When `DISABLE_NATIVECCD` is set, the conformant behavior is
to continue using our solver (there is nothing to fall back to).

**`multiccd` (default: disabled):** When enabled, MuJoCo's `mjc_penetration`
generates multiple contact points for a single convex-convex pair. This
improves stability for flat-face contacts (e.g., box resting on box). The
mechanism: run the penetration solver multiple times with different initial
search directions (perturbing the initial GJK direction) to find contact
points at different locations on the contact surface. Returns up to
`mjMAXCONPAIR` (50) contacts per geom pair.

**CortenForge current state:**
- `DISABLE_NATIVECCD = 1 << 17` (enums.rs:654) — constant defined
- `ENABLE_MULTICCD = 1 << 4` (enums.rs:671) — constant defined
- `nativeccd: bool` on `MjcfFlag` (types.rs:198, default true) — parsed
- `multiccd: bool` on `MjcfFlag` (types.rs:214, default false) — parsed
- `builder/mod.rs:908-909` — `nativeccd` → `DISABLE_NATIVECCD` flag wiring
- `builder/mod.rs:923` — `multiccd` → `ENABLE_MULTICCD` flag wiring
- `builder/mod.rs:933-938` — `tracing::warn!` no-op guards for both

### EGT-3: Margin-zone contact gap in GJK/EPA path

**File:** `sim/L0/core/src/collision/narrow.rs` lines 186-198

Current GJK/EPA fallback path:
```rust
if let Some(result) = gjk_epa_contact(&collision_shape1, &pose1, &collision_shape2, &pose2) {
    if result.penetration > -margin {
        return Some(make_contact_from_geoms(...));
    }
}
```

`gjk_epa_contact` returns `None` when shapes do NOT overlap. This means
margin-zone contacts (shapes separated by less than margin but not
overlapping) are MISSED for all GJK/EPA pairs.

**Affected geom pairs — two call sites:**

Via `narrow.rs:186` (GJK/EPA fallback path):
- cylinder-cylinder
- cylinder-box (no analytical path — always GJK/EPA)
- cylinder-ellipsoid
- ellipsoid-ellipsoid
- ellipsoid-box
- ellipsoid-capsule
- cylinder-capsule (degenerate-case fallback from analytical)

Via `mesh_collide.rs:60` (mesh-mesh hull path):
- mesh-mesh (when both meshes have convex hulls — the MuJoCo-conformant path)

Both call sites call `gjk_epa_contact()` and return `None` for
non-overlapping shapes, so both have the same margin-zone gap.

**Analytical pairs handle margin correctly** because they compute signed
separation distance: `let penetration = sum_radii - dist;` can be negative,
and `if penetration > -margin` catches near-contacts. The GJK/EPA paths
have no equivalent — they return `None` for any non-overlapping pair.

**MuJoCo behavior:** `mjc_penetration()` handles both overlapping (returns
penetration depth) AND non-overlapping-within-margin (returns separation
distance as negative penetration, allowing margin-zone contact generation).
The GJK distance query is the mechanism that provides this.

### EGT-4: GJK distance algorithm

The GJK algorithm can compute not only intersection (origin inside Minkowski
difference) but also the minimum distance between two non-overlapping convex
shapes. This is the standard "GJK distance" extension:

1. Run GJK normally — build a simplex in Minkowski space
2. If the origin is NOT enclosed (shapes don't overlap):
   - The closest point on the current simplex to the origin gives the
     minimum Minkowski-space distance
   - The witness points (closest points on shapes A and B) are recovered
     from the simplex barycentric coordinates
   - Distance = `|closest_a - closest_b|`
3. If shapes DO overlap: return `None` (use EPA instead)

The existing `gjk_query()` (gjk_epa.rs:595) already builds the simplex and
detects non-intersection. The extension is to compute the closest point on
the simplex to the origin when non-intersecting, then extract witness points.

The key addition is a `closest_point_on_simplex()` function that:
- For a 1-simplex (point): return that point
- For a 2-simplex (line): project origin onto line segment, clamp
- For a 3-simplex (triangle): project origin onto triangle plane, check
  Voronoi regions
- Return barycentric coordinates to recover witness points from
  `MinkowskiPoint.support_a` / `support_b`

### EGT-5: MULTICCD algorithm

**MuJoCo's multi-CCD mechanism** (from `engine_collision_convex.c`):

When `ENABLE_MULTICCD` is set and a convex-convex pair generates a valid
contact, MuJoCo runs additional penetration queries with perturbed initial
search directions to find additional contact points on flat contact surfaces.

The algorithm (inferred from EGT-7 empirical results + MuJoCo header
comments — exact perturbation scheme needs verification from C source
`engine_collision_convex.c` during spec writing):
1. Run `mjc_penetration()` normally → get first contact (direction,
   penetration, normal, point)
2. If MULTICCD enabled:
   - Generate additional initial directions by perturbing the original
     search direction (exact scheme TBD — EGT-7 shows 3 additional
     contacts at face corners for mesh-mesh, suggesting 3 perturbations)
   - Run `mjc_penetration()` with each perturbed direction
   - Each run may find a different contact point on the flat surface
   - Filter duplicates (contacts closer than tolerance)
3. Return all unique contacts (up to `mjMAXCONPAIR` = 50)

**Open question for spec:** The exact perturbation scheme (number of
additional directions, rotation angles, axis of rotation) must be
determined from MuJoCo's C source. EGT-7 empirically confirms the
output (4 contacts at face corners) but not the internal algorithm.

**In CortenForge:** The equivalent is to run `gjk_epa_contact()` multiple
times with different initial search directions in `gjk_query()`. The
current `gjk_query()` starts with `direction = center_b - center_a`
(gjk_epa.rs:604). For MULTICCD, use perturbed initial directions to
explore different contact regions.

**Affected code paths (two `gjk_epa_contact` call sites):**
- `narrow.rs:186` — currently calls `gjk_epa_contact` once, returns single
  `Option<Contact>`. With MULTICCD, must collect multiple contacts.
- `mesh_collide.rs:60` — mesh-mesh hull path calls `gjk_epa_contact` once.
  This is the PRIMARY MULTICCD path (EGT-7 verified mesh-mesh produces
  1→4 contacts). Must also collect multiple contacts with MULTICCD.
- `mod.rs:457-464` — broadphase loop currently pushes one contact. Must
  handle multi-contact returns (similar to hfield multi-contact in
  `mod.rs:450-456`).
- `mod.rs:520-533` — mechanism 2 loop, same issue.

### EGT-6: `ccd_iterations` flow gap

Currently `ccd_iterations` is parsed and stored on `MjcfConfig` but NOT
transferred to `Model`. The collision pipeline reads from `Model`, not
`MjcfConfig`. The flow must be:

```
MJCF <option ccd_iterations="25"/>
  → parser.rs:253 → MjcfOption.ccd_iterations
  → config.rs:116 → MjcfConfig.ccd_iterations
  → builder → Model.ccd_iterations  ← MISSING
  → collision/narrow.rs → gjk_epa::gjk_query(max_iterations=model.ccd_iterations)
```

Similarly for `ccd_tolerance`:
```
MJCF <option ccd_tolerance="1e-8"/>
  → parser.rs → MjcfOption.ccd_tolerance  ← MISSING
  → config.rs → MjcfConfig.ccd_tolerance  ← MISSING
  → builder → Model.ccd_tolerance  ← MISSING
  → collision/narrow.rs → gjk_epa::epa_query(tolerance=model.ccd_tolerance)
```

### EGT-7: MULTICCD empirical verification

**MuJoCo version:** 3.5.0 (Python API).

**Test case:** Two box-shaped meshes (vertices only, auto-hull) with flat
face contact. Upper mesh (half-size 0.3) resting on lower mesh (half-size
0.5) at z=0.75.

```
Without MULTICCD: ncon=1
  Contact 0: pos=[-0.30 -0.30  0.475], depth=-0.050

With MULTICCD: ncon=4
  Contact 0: pos=[-0.30 -0.30  0.475], depth=-0.050
  Contact 1: pos=[ 0.30 -0.30  0.475], depth=-0.050
  Contact 2: pos=[ 0.30  0.30  0.475], depth=-0.050
  Contact 3: pos=[-0.30  0.30  0.475], depth=-0.050
```

**Key findings:**
1. MULTICCD generates 4 contacts at the corners of the upper mesh's face
   (the contact patch boundary), vs 1 contact without MULTICCD.
2. All 4 contacts have the same depth (same penetration everywhere on flat
   face).
3. MULTICCD does NOT affect ellipsoid-ellipsoid (curved contact — still 1
   contact). This confirms MULTICCD only generates additional points for
   flat contact patches.
4. MULTICCD does NOT affect box-box or box-plane or cylinder-plane — these
   pairs use analytical collision functions that already generate multiple
   contacts, bypassing `mjc_penetration()`.
5. MULTICCD ONLY affects pairs that go through the general convex solver
   (`mjc_penetration`): mesh-mesh, mesh-convex, and potentially
   cylinder-cylinder / ellipsoid-* pairs where flat contacts exist.

**Implication for CortenForge:** MULTICCD applies to the GJK/EPA fallback
path in `narrow.rs:186`. Only pairs that reach `gjk_epa_contact()` are
candidates. Analytical pairs (sphere-sphere, capsule-capsule, etc.) are
unaffected. The hfield path already has its own multi-contact mechanism
(prism iteration from Spec C).

### EGT-8: Margin-zone contact empirical verification

**MuJoCo version:** 3.5.0 (Python API).

**Test case:** Two ellipsoids with `margin="0.1"` each (combined margin
0.2), separated by ~0.05 (not overlapping but within margin).

```
Ellipsoid-ellipsoid, separation ≈ 0.05, combined margin = 0.2:
  ncon=1, contact.dist = 0.050 (positive = separated within margin)

Ellipsoid-ellipsoid, separation ≈ 0.05, combined margin = 0.02:
  ncon=0 (separation > margin, no contact)
```

**Key findings:**
1. MuJoCo DOES generate margin-zone contacts for convex solver pairs
   (ellipsoid-ellipsoid). The penetration solver returns the separation
   distance for non-overlapping shapes, and the pipeline generates a
   contact when `distance < combined_margin`.
2. MuJoCo's `contact.dist` field is the signed distance: negative =
   penetrating, positive = separated within margin.
3. CortenForge's GJK/EPA path (`narrow.rs:186`) CANNOT produce these
   contacts because `gjk_epa_contact()` returns `None` for non-overlapping
   shapes. This is a confirmed conformance gap.
4. Analytical pairs (sphere-sphere, etc.) handle margins correctly because
   they compute signed distance directly.

**Conformance gap quantification:** Any pair that goes through the GJK/EPA
fallback path (cylinder-cylinder, cylinder-ellipsoid, ellipsoid-ellipsoid,
ellipsoid-box, convex mesh pairs) will miss margin-zone contacts that
MuJoCo generates. The GJK distance query (EGT-4) is the fix.

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. **Critical for
> this spec: the scope correction from "conservative-advancement CCD" to
> "convex collision solver completeness" must be prominently documented.**

| Grade | Bar |
|-------|-----|
| **A+** | Scope correction documented prominently: MuJoCo's "CCD" is "Convex Collision Detection" (the `libccd` name), NOT "Continuous Collision Detection". No conservative advancement, no time-of-impact, no velocity-based filtering exists in MuJoCo. Every MuJoCo field cited with source file: `mjOption.ccd_iterations` and `mjOption.ccd_tolerance` from `mjmodel.h`, `mjDSBL_NATIVECCD` and `mjENBL_MULTICCD` from `mjmodel.h`, `mjc_penetration()` from `engine_collision_convex.c`. Edge cases addressed: zero-iteration CCD (`ccd_iterations=0` disables convex solver?), `ccd_tolerance=0` behavior, MULTICCD with non-flat contacts (no additional points), MULTICCD with already-overlapping vs margin-zone contacts, `DISABLE_NATIVECCD` when no libccd available. Default values stated with source: `ccd_iterations=35` (empirically verified, MuJoCo 3.5.0 — CortenForge's current default of 50 is WRONG), `ccd_tolerance=1e-6` (verified). |
| **A** | Scope correction clear. MuJoCo behavior described correctly from headers. Minor gaps in MULTICCD edge-case coverage. |
| **B** | Correct at high level, but scope correction unclear — reader might still think this is time-of-impact CCD. |
| **C** | Partially correct. Conservative advancement described as MuJoCo behavior. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. The GJK distance query,
> margin-zone contact generation, MULTICCD multi-point generation, and
> parameter wiring are all implementable without reading MuJoCo source.

| Grade | Bar |
|-------|-----|
| **A+** | Three algorithms fully specified: (1) GJK distance query — closest-point-on-simplex for 1/2/3-simplex cases, barycentric coordinate recovery of witness points, distance computation from witness points. (2) Margin-zone contact generation — `gjk_distance()` call when `gjk_epa_contact` returns `None`, contact generation when `distance < margin`, contact normal/point/depth from witness points. (3) MULTICCD — initial direction perturbation scheme, number and angles of perturbed directions, duplicate contact filtering with tolerance, multi-contact API design explicitly addressed (current `collide_geoms` and `collide_with_mesh` both return `Option<Contact>` — MULTICCD needs multi-contact; spec must define the return type change or new function, how `mod.rs` broadphase/mechanism-2 loops consume multiple contacts, and how `mesh_collide.rs:60` hull path returns multiple contacts). Parameter wiring path from `MjcfOption` through `Model` to `gjk_query()`/`epa_query()` fully specified. Each algorithm matches MuJoCo's computational steps. |
| **A** | Algorithms complete. One or two minor details left implicit. |
| **B** | Algorithm structure clear but GJK distance or MULTICCD details hand-waved. |
| **C** | Skeleton only — "add distance query" without algorithm. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo.

| Grade | Bar |
|-------|-----|
| **A+** | GJK/EPA function signature convention documented: current signatures use hardcoded constants, new signatures accept parameters. Convention difference table: `ccd_iterations` maps to both `GJK_MAX_ITERATIONS` and `EPA_MAX_ITERATIONS` (MuJoCo uses single iteration count for both). Contact normal convention: GJK/EPA returns "B toward A" (gjk_epa.rs:108) but `make_contact_from_geoms` expects "g1 toward g2" — translation rule stated (already handled in narrow.rs). `Model` vs `MjcfConfig` distinction: `ccd_iterations` currently on `MjcfConfig` only, must also be on `Model` for collision pipeline access. Margin sign convention: positive margin = detection zone extends beyond surface; negative penetration = separation distance; contact generated when `penetration > -margin` (equivalent to `distance < margin`). Depth field mapping: MuJoCo `contact.dist` (positive = separated, negative = penetrating) maps to CortenForge `Contact.depth` (positive = penetrating, negative = separated) — **opposite sign convention**. Margin-zone contacts from `gjk_distance()` must negate the separation distance to produce `Contact.depth`. `mjMAXCONPAIR` (50) maps to existing `MAX_CONTACTS_PER_PAIR` constant in hfield.rs:17 — MULTICCD should share this constant rather than redefine it. |
| **A** | Major conventions documented. Minor field-name mappings left implicit. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has three-part structure: (1) concrete model, (2) exact expected value or tolerance, (3) what to check. AC for GJK distance: unit test calling `gjk_distance()` directly with two sphere `CollisionShape`s separated by 0.5 units → distance = 0.5 - r1 - r2 (exact analytical oracle; note: sphere-sphere is dispatched analytically in `narrow.rs:105` so this AC tests the function in isolation, not the integration path). AC for margin-zone contact: two ellipsoids separated by ~0.05 with combined margin 0.2 → contact generated with `dist ≈ 0.05` (conformance: MuJoCo 3.5.0 verified, see EGT-8). AC for `ccd_iterations` wiring: parse non-default value, verify it reaches solver (iteration count observable via returned struct or test with deliberately low count). AC for `ccd_tolerance` wiring: parse non-default value, verify convergence tolerance changes. AC for MULTICCD: two box-shaped convex meshes with flat face contact, `multiccd` enabled → 4 contacts at face corners (conformance: MuJoCo 3.5.0 verified, see EGT-7; note: box-box is analytical/SAT and is unaffected by MULTICCD — must use mesh-mesh hull pairs that go through `gjk_epa_contact`). AC for `DISABLE_NATIVECCD`: flag parsed, no crash, solver still works. Code-review ACs: `tracing::warn!` guards removed, `ccd_tolerance` on `Model`, parameter threading complete. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs directionally correct but vague ("should work"). |
| **C** | ACs are aspirational statements, not tests. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory: coincident shapes (distance=0), shapes just touching (distance≈0), shapes at exactly margin distance, zero margin, large separation (no contact), degenerate shapes (zero-radius sphere, flat box), MULTICCD on non-flat contact (single contact expected), `ccd_iterations=0`, `ccd_tolerance=0`. Negative cases: shapes beyond margin → no contact, MULTICCD disabled → single contact. Regression: existing GJK/EPA tests unchanged (default parameters match current hardcoded values). At least one MuJoCo conformance test per major feature. At least one test uses a non-trivial model (multi-body scene with mixed geom types). |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs
> are explicitly stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. S1 (GJK distance query) is prerequisite for S2 (margin-zone contacts). S3 (parameter parsing/wiring) is independent of S1/S2. S4 (MULTICCD) depends on parameter wiring (S3) but not on GJK distance (S1). No dependency on Spec A (convex hull) — CCD operates on existing `CollisionShape` objects. No dependency on Spec C (hfield pairs) — hfield collision already has its own GJK/EPA call. Cross-spec note: Spec C's `hfield.rs` calls `gjk_epa_contact` → its signature will change when Spec D adds parameters → Spec C call site needs update. This is a known blast-radius item, not a dependency. |
| **A** | Order clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (EGT codebase context table). Behavioral changes: (1) `gjk_epa_contact` signature change adds parameters — ALL callers must update (narrow.rs:186, mesh_collide.rs:60, hfield.rs:162-163, any test calling `gjk_epa_contact` directly). (2) Default `ccd_iterations=35` vs current hardcoded 64 — slightly fewer max iterations by default, may change behavior for edge-case convergence (only causes issues if CortenForge's GJK/EPA converges slower than MuJoCo's native solver for the same shapes). (3) Margin-zone contacts now generated for GJK/EPA pairs (narrow.rs AND mesh_collide.rs hull path) — new contacts appear where none existed before → conformance improvement (toward MuJoCo behavior). (4) MULTICCD generates additional contacts → may increase `data.ncon` and affect solver behavior. (5) MULTICCD return type change: `collide_geoms` (narrow.rs:61) and `collide_with_mesh` (mesh_collide.rs:22) currently return `Option<Contact>` — MULTICCD requires multi-contact returns. This affects every caller of these functions: `mod.rs` mechanism-1 loop (lines 457-464), mechanism-2 loop (lines 520-533), and any code that pattern-matches on `Option<Contact>`. Existing test impact: GJK/EPA tests in `gjk_epa.rs` (20 tests) must be updated for new function signatures. sim-core domain test count stated (current baseline from test run). `tracing::warn!` removal is safe (no behavioral effect). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Signature-change blast radius unaddressed. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: "convex collision solver" (not "CCD" ambiguously), "GJK distance query" (not "separation distance" sometimes and "closest-point query" other times), "margin-zone contact" (not "near-contact" or "proximity contact"). File paths match between Specification and Files Affected. AC numbers match Traceability Matrix. Edge cases in MuJoCo Reference appear in Test Plan. The scope correction language is consistent — no section still describes "conservative advancement" or "time-of-impact". |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections still use "CCD" to mean "continuous collision detection". |
| **C** | Contradictions between sections. Scope correction not reflected throughout. |

### P9. GJK Distance Query Correctness *(domain-specific)*

> The GJK distance query algorithm is mathematically correct and handles
> all simplex cases. This is the foundational prerequisite for margin-zone
> contact generation.

| Grade | Bar |
|-------|-----|
| **A+** | GJK distance algorithm fully specified with: (1) Closest-point-on-simplex for simplex sizes 1-point, 2-line, 3-triangle, with Voronoi region analysis for triangle case. Explicit note that the 4-simplex (tetrahedron) case is NOT needed: in distance mode the simplex is always reduced to the closest sub-feature before reaching size 4 (standard GJK distance property — the existing `Simplex` struct supports size 4 for intersection mode, but distance mode never uses it). (2) Barycentric coordinate computation to recover witness points from `MinkowskiPoint.support_a`/`support_b`. (3) Distance = `(witness_a - witness_b).norm()`. (4) Degenerate case handling: coincident shapes (distance=0, transition to EPA), nearly-degenerate simplices (collinear points in triangle case), numerical stability guards (epsilon comparisons). (5) Convergence criterion: the simplex's closest-point distance decreases monotonically; terminate when improvement < tolerance. (6) Proof that existing `gjk_query()` simplex construction can be reused — only the termination and output computation differ between intersection and distance modes. |
| **A** | Algorithm correct. Minor degenerate case gaps. |
| **B** | Algorithm structure clear but closest-point-on-simplex incomplete. |
| **C** | Distance query described without algorithm. |

**Boundary with P2:** P2 grades overall algorithm completeness (all three
algorithms). P9 grades the mathematical correctness and completeness of the
GJK distance query specifically — the deepest algorithmic component.

### P10. Parameter Threading Architecture *(domain-specific)*

> The spec correctly describes how `ccd_iterations` and `ccd_tolerance` flow
> from MJCF parsing through model building to the collision pipeline, and how
> the flag constants (`DISABLE_NATIVECCD`, `ENABLE_MULTICCD`) gate behavior.

| Grade | Bar |
|-------|-----|
| **A+** | Complete data flow documented: `<option ccd_iterations="N" ccd_tolerance="T"/>` → `MjcfOption` fields → `MjcfConfig` fields → builder → `Model` fields → `mj_collision()` → `collide_geoms()` → `gjk_epa_contact(max_iter, tolerance)` → `gjk_query(max_iter)` / `epa_query(max_iter, tolerance)`. Default values verified at each layer. Flag flow: `<flag nativeccd="..." multiccd="..."/>` → `MjcfFlag` → builder → `Model.disableflags` / `Model.enableflags` → collision pipeline checks `disabled(model, DISABLE_NATIVECCD)` / `enabled(model, ENABLE_MULTICCD)`. Every function signature change documented. The pattern matches the T1 (§57 SDF options) threading pattern (sdf_iterations/sdf_initpoints → sdf.rs) for consistency. |
| **A** | Flow documented. Minor gaps in intermediate layers. |
| **B** | Parse and final use documented, but intermediate wiring hand-waved. |
| **C** | Parameters parsed but threading not specified. |

**Boundary with P2:** P2 grades algorithm completeness. P10 grades the
plumbing/architecture of parameter flow — ensuring the right values reach
the right functions at runtime.

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific functions (`gjk_query`,
      `epa_query`, `gjk_epa_contact`, `gjk_distance`), specific files
      (`gjk_epa.rs:67/70/76/595/830/1078`, `narrow.rs:186-198`,
      `builder/mod.rs:933-938`), and specific values (`ccd_iterations=35`,
      `GJK_MAX_ITERATIONS=64`, `mjMAXCONPAIR=50`). Two independent
      reviewers could assign consistent grades.

- [x] **Non-overlap:** P1 (MuJoCo reference + scope correction) vs P9
      (GJK distance algorithm correctness) — P1 grades whether the spec
      correctly describes MuJoCo's behavior; P9 grades whether the GJK
      distance algorithm is mathematically sound. P2 (algorithm completeness)
      vs P9 (GJK distance depth) — P2 covers all three algorithms at
      completeness level; P9 dives into GJK distance specifically for
      mathematical rigor. P2 vs P10 (parameter threading) — P2 covers
      algorithmic steps; P10 covers plumbing architecture. Boundaries
      documented in each criterion.

- [x] **Completeness:** The 10 criteria cover: MuJoCo reference + scope
      correction (P1), algorithms (P2), conventions (P3), ACs (P4), tests
      (P5), dependencies (P6), blast radius (P7), consistency (P8), GJK
      distance depth (P9), parameter threading (P10). The scope correction
      is the most important dimension — without P1, the entire spec would
      implement non-existent MuJoCo behavior.

- [x] **Gradeability:** P1 → MuJoCo Reference, Scope Adjustment. P2 →
      Specification sections. P3 → Convention Notes. P4 → Acceptance
      Criteria. P5 → Test Plan. P6 → Prerequisites + Execution Order. P7 →
      Risk & Blast Radius. P8 → cross-cutting. P9 → GJK Distance section
      (within Specification). P10 → Parameter threading section (within
      Specification or dedicated section).

- [x] **Conformance primacy:** P1 is tailored with the critical scope
      correction — the most important conformance finding in this rubric.
      The A+ bar for P1 requires explicit documentation that MuJoCo's CCD
      is NOT continuous collision detection. P4 requires margin-zone contact
      conformance values. P5 requires conformance tests. The scope
      adjustment drops non-MuJoCo features (conservative advancement,
      time-of-impact).

- [x] **Empirical grounding:** EGT-1 through EGT-6 filled in from header
      reading and codebase audit. EGT-7 and EGT-8 filled in from empirical
      verification via MuJoCo 3.5.0 Python API (MULTICCD contact count,
      margin-zone contact generation). Every A+ bar that references MuJoCo
      behavior has a corresponding EGT entry. The scope correction (EGT-1,
      EGT-2) is derived from MuJoCo header comments, not from docs or
      intuition.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Scope Adjustment |
| P2 | Specification (S1, S2, S3, S4) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | GJK Distance section (within S1 of Specification) |
| P10 | Parameter threading section (within S3 of Specification) |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. GJK Distance Query Correctness | | |
| P10. Parameter Threading Architecture | | |

**Overall: — (Rev 5)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | Umbrella and session plan describe "conservative-advancement CCD" and "time-of-impact estimation" — MuJoCo has neither | Rubric Phase 1 (EGT-1, EGT-2: header comment "convex collision solver", not "continuous collision") | Dropped conservative advancement and TOI entirely. Redefined scope as convex collision solver completeness: GJK distance, margin-zone contacts, parameter wiring, MULTICCD | Rubric Rev 1 |
| R2 | Scope | §50 future work says "fast-moving sphere does not tunnel through thin wall" — this AC implies time-of-impact CCD which MuJoCo doesn't have | Rubric Phase 1 (EGT-1) | Dropped tunneling AC. Replaced with conformance-relevant ACs (margin-zone contacts, MULTICCD multi-point contacts) | Rubric Rev 1 |
| R3 | P9 | GJK distance query still needed but for different reason than umbrella stated | Rubric Phase 1 (EGT-3: margin-zone contact gap in narrow.rs:186-198) | Reframed: GJK distance enables margin-zone contact generation for GJK/EPA pairs (conformance gap vs analytical pairs) | Rubric Rev 1 |
| R4 | P10 | `ccd_iterations` parsed but never reaches `Model` — parameter flow has a gap | Rubric Phase 1 (EGT-6: config.rs has field, Model does not) | Documented flow gap. P10 requires complete parameter threading from parse to collision pipeline | Rubric Rev 1 |
| R5 | Scope | `DISABLE_NATIVECCD` described as needing "libccd fallback" — CortenForge has no libccd | Rubric Phase 1 (EGT-2) | Redefined as conformant no-op. Remove `tracing::warn!`, keep flag for model fidelity | Rubric Rev 1 |
| R6 | P1 | Need to verify `ccd_tolerance` default value from MuJoCo | Rubric self-audit | Verified via Python API: `ccd_tolerance=1e-6` (matches `EPA_TOLERANCE`). Also discovered `ccd_iterations=35` (not 50). | Rubric Rev 2 |
| R7 | P1 | CortenForge default `ccd_iterations=50` does not match MuJoCo default of 35 | Rubric Phase 1 (EGT-1 empirical verification) | Added to EGT-1. The spec must fix the default in `MjcfOption` and `MjcfConfig`. | Rubric Rev 2 |
| R8 | P2 | MULTICCD algorithm described without empirical verification — assumed "perturbed initial directions" without proof | Rubric Phase 1 (EGT-7 empirical test) | Verified: mesh-mesh 1→4 contacts with MULTICCD. Only affects pairs through `mjc_penetration()`, not analytical pairs. Curved contacts (ellipsoid) correctly get 0 additional contacts. | Rubric Rev 2 |
| R9 | P9 | Margin-zone contact gap described without empirical verification | Rubric Phase 1 (EGT-8 empirical test) | Verified: MuJoCo generates margin-zone contacts for ellipsoid-ellipsoid with `contact.dist > 0`. CortenForge's GJK/EPA path returns `None` for non-overlapping → confirmed conformance gap. | Rubric Rev 2 |
| R10 | P4 | P4 references "cylinder-cylinder separated by 0.01" but EGT-8 only tested ellipsoid-ellipsoid | Rubric self-audit round 2 | P4 AC should use ellipsoid-ellipsoid (empirically verified pair) or test cylinder-cylinder explicitly before citing it | Rubric Rev 2 |
| R11 | P4, P7 | P4 MULTICCD AC said "box-on-box" but EGT-7 proves box-box is analytical (SAT at narrow.rs:144) and unaffected by MULTICCD | Stress test | Fixed: AC now specifies mesh-mesh hull pairs (the empirically verified MULTICCD path via mesh_collide.rs:60) | Rubric Rev 3 |
| R12 | P7 | `mesh_collide.rs:60` missing from codebase context table and call-site list — mesh-mesh hull path calls `gjk_epa_contact()` and has same margin-zone gap as narrow.rs:186, AND is the primary MULTICCD path | Stress test (verified: mesh_collide.rs imports and calls `gjk_epa_contact` at line 60) | Added mesh_collide.rs to codebase table, call-site list, EGT-3 affected pairs, and EGT-5 affected code paths | Rubric Rev 3 |
| R13 | P2 | EGT-5 described MULTICCD perturbation angles ("±45°") as ground truth but this is inferred, not verified from C source | Stress test | Marked as inferred with open question for spec: exact perturbation scheme must be determined from `engine_collision_convex.c` during spec writing | Rubric Rev 3 |
| R14 | EGT-3 | Parenthetical "cylinder-box (after analytical path fails)" was wrong — cylinder-box has NO analytical path, always goes through GJK/EPA | Stress test (verified: narrow.rs has no Cylinder+Box match arm before GJK/EPA fallback) | Fixed parenthetical to "(no analytical path — always GJK/EPA)" | Rubric Rev 3 |
| R15 | P7 | Test count "17 tests" was wrong — gjk_epa.rs has 20 `#[test]` annotations | Stress test (grep count) | Fixed to 20 | Rubric Rev 3 |
| R16 | P9 | P9 A+ bar lists 1/2/3-simplex cases but existing `Simplex` struct supports 4-simplex (tetrahedron) — implementer might wonder about missing case | Stress test round 2 (read gjk_epa.rs:115, verified do_simplex at line 678 handles size 4) | Added explicit note: 4-simplex case not needed in distance mode because simplex is always reduced to closest sub-feature before reaching size 4 (standard GJK distance property) | Rubric Rev 4 |
| R17 | P2, P7 | MULTICCD requires multi-contact but `collide_geoms` (narrow.rs:61) and `collide_with_mesh` (mesh_collide.rs:22) return `Option<Contact>` — return type change not addressed by any criterion | Stress test round 2 (verified return types in codebase) | Added to P2 A+ bar (spec must define multi-contact API design) and P7 A+ bar (return type change affects all callers of both functions) | Rubric Rev 4 |
| R18 | P3 | MuJoCo `contact.dist` (positive=separated) has opposite sign to CortenForge `Contact.depth` (positive=penetrating, contact_types.rs:46) — field name + sign mapping not stated | Stress test round 3 (read Contact struct) | Added to P3 A+ bar: explicit depth field mapping with sign convention and `gjk_distance()` negation requirement | Rubric Rev 5 |
| R19 | P4 | GJK distance AC uses "two spheres" but sphere-sphere is dispatched analytically (narrow.rs:105), never reaching GJK/EPA — ambiguous whether this is a unit test or integration test | Stress test round 3 | Clarified: AC tests `gjk_distance()` directly (unit test), not through `collide_geoms` integration path | Rubric Rev 5 |
| R20 | P3 | `MAX_CONTACTS_PER_PAIR = 50` exists in hfield.rs:17 — MULTICCD needs the same constant but rubric didn't note it should be shared | Stress test round 3 (grep for constant) | Added to P3 A+ bar: MULTICCD should share existing hfield constant rather than redefine | Rubric Rev 5 |
| R21 | — | `ccd_enabled()` at config.rs:154 is dead code (never called anywhere in codebase) | Stress test round 3 (grep for callers: 0 results) | Updated call-site list to note dead code status | Rubric Rev 5 |
