# Convex Collision Solver Completeness — Spec D

**Status:** Draft
**Phase:** Roadmap Phase 9 — Collision Completeness
**Effort:** L
**MuJoCo ref:** `mjc_penetration()` in `engine_collision_convex.c`;
  `mjOption.ccd_iterations` / `mjOption.ccd_tolerance` in `mjmodel.h`;
  `mjDSBL_NATIVECCD` / `mjENBL_MULTICCD` in `mjmodel.h`
**MuJoCo version:** 3.5.0 (headers from pip package `mujoco==3.5.0`)
**Test baseline:** 2,100+ sim domain tests
**Prerequisites:**
- Phase 9 Specs A–C complete (Spec A: `ba6c261`, Spec B: `5a2b949`, Spec C: `47a4fba`)
- None of Spec A/B/C are strict dependencies — Spec D operates on existing
  `CollisionShape` objects. Tests use non-mesh convex shapes (sphere, capsule,
  box, ellipsoid, cylinder) which don't require Spec A's convex hull.

**Independence:** This spec is independent of Specs A, B, C, and E per the
umbrella dependency graph. No shared mutable files with other in-progress specs.
`gjk_epa.rs` is single-owner (Spec D). `narrow.rs` and `mesh_collide.rs` are
touched by Spec D only for signature updates and new code paths; no other
in-progress spec modifies these functions.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

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
   non-overlapping convex shapes (margin-zone contact generation)
2. **Margin-zone contact generation** — modify GJK/EPA paths in `narrow.rs`
   and `mesh_collide.rs` to generate contacts for non-overlapping shapes
   within margin distance
3. **Parse `ccd_tolerance`** from `<option>` — add to `MjcfOption` and `Model`
4. **Wire `ccd_iterations`** — thread from `Model` to `gjk_query()`/`epa_query()`,
   replacing hardcoded `GJK_MAX_ITERATIONS=64` and `EPA_MAX_ITERATIONS=64`
5. **Wire `ccd_tolerance`** — thread from `Model` to `epa_query()` and
   `gjk_distance()`, replacing hardcoded `EPA_TOLERANCE=1e-6`
6. **`DISABLE_NATIVECCD`** — conformant no-op, remove `tracing::warn!` guard
7. **`ENABLE_MULTICCD`** — multi-point contact generation for convex-convex pairs

---

## Problem Statement

**Conformance gap** — CortenForge's GJK/EPA collision pipeline has three
conformance gaps relative to MuJoCo:

1. **Margin-zone contacts missing for GJK/EPA pairs.** MuJoCo's
   `mjc_penetration()` handles both overlapping shapes (penetration depth) AND
   non-overlapping shapes within margin distance (separation distance as
   negative penetration). CortenForge's `gjk_epa_contact()` returns `None`
   for non-overlapping shapes, so margin-zone contacts are **silently dropped**
   for all pairs routed through the GJK/EPA fallback path: cylinder-cylinder,
   cylinder-box, cylinder-ellipsoid, ellipsoid-ellipsoid, ellipsoid-box,
   ellipsoid-capsule (degenerate fallback), and mesh-mesh hull pairs.
   Analytical pairs handle margins correctly because they compute signed
   separation distance.

2. **Solver parameters not configurable.** `ccd_iterations` is parsed from
   MJCF but never reaches the collision pipeline — it stops at `MjcfConfig`
   and is not on `Model`. `ccd_tolerance` is not parsed at all. The GJK/EPA
   solver uses hardcoded constants (`GJK_MAX_ITERATIONS=64`,
   `EPA_MAX_ITERATIONS=64`, `EPA_TOLERANCE=1e-6`) instead of MuJoCo's
   defaults (`ccd_iterations=35`, `ccd_tolerance=1e-6`). The iteration count
   mismatch (64 vs 35) means CortenForge's solver runs up to 29 extra
   iterations per call.

3. **MULTICCD not implemented.** When `ENABLE_MULTICCD` is set, MuJoCo
   generates multiple contact points for a single convex-convex pair to
   stabilize flat-face contacts (e.g., box-shaped mesh resting on another).
   CortenForge logs a `tracing::warn!` and produces only a single contact.

---

## MuJoCo Reference

### `mjOption.ccd_iterations` and `mjOption.ccd_tolerance`

**Source:** `mjmodel.h` → `mjOption` struct

```c
mjtNum ccd_tolerance;    // convex collision solver tolerance
int ccd_iterations;      // maximum number of convex collision solver iterations
```

These parameters control the convex collision **solver** (GJK/EPA equivalent).
They are NOT time-of-impact parameters. They map directly to GJK/EPA iteration
limits and convergence tolerance.

**MuJoCo defaults (verified empirically via Python API, MuJoCo 3.5.0):**
- `ccd_iterations`: **35** (NOT 50 — CortenForge's `MjcfOption` default of
  50 is WRONG and must be fixed)
- `ccd_tolerance`: **1e-6**

**CortenForge current state:**
- `ccd_iterations`: Parsed in `parser.rs:253-254`, stored in `MjcfOption`
  (`types.rs:422`, default **50 — INCORRECT**), transferred to `MjcfConfig`
  (`config.rs:70,116`). **NOT on `Model`** — does not reach collision pipeline.
- `ccd_tolerance`: NOT parsed, NOT stored anywhere.
- `GJK_MAX_ITERATIONS = 64` (`gjk_epa.rs:67`) — MuJoCo default is 35.
- `EPA_MAX_ITERATIONS = 64` (`gjk_epa.rs:70`) — MuJoCo default is 35.
- `EPA_TOLERANCE = 1e-6` (`gjk_epa.rs:76`) — matches MuJoCo default.

### `DISABLE_NATIVECCD` and `ENABLE_MULTICCD` flags

**Source:** `mjmodel.h` → disable/enable flag enums

```c
mjDSBL_NATIVECCD    = 1<<17,    // native convex collision detection
mjENBL_MULTICCD     = 1<<4,     // multi-point convex collision detection
```

**`nativeccd` (default: enabled):** Controls whether MuJoCo uses its native
convex collision implementation or falls back to the `libccd` library. Both
produce the same result. In CortenForge, there is no `libccd` — our GJK/EPA
IS the native solver. When `DISABLE_NATIVECCD` is set, the conformant behavior
is to continue using our solver (there is nothing to fall back to).

**`multiccd` (default: disabled):** When enabled, MuJoCo's `mjc_penetration()`
generates multiple contact points for a single convex-convex pair. This
improves stability for flat-face contacts (e.g., box resting on box). The
mechanism: run the penetration solver multiple times with different initial
search directions to find contact points at different locations on the contact
surface. Returns up to `mjMAXCONPAIR` (50) contacts per geom pair.

**CortenForge current state:**
- `DISABLE_NATIVECCD = 1 << 17` (`enums.rs:654`) — constant defined
- `ENABLE_MULTICCD = 1 << 4` (`enums.rs:671`) — constant defined
- `nativeccd: bool` on `MjcfFlag` (`types.rs:198`, default true) — parsed
- `multiccd: bool` on `MjcfFlag` (`types.rs:214`, default false) — parsed
- `builder/mod.rs:908-909` — `nativeccd` → `DISABLE_NATIVECCD` flag wiring
- `builder/mod.rs:923` — `multiccd` → `ENABLE_MULTICCD` flag wiring
- `builder/mod.rs:933-938` — `tracing::warn!` no-op guards for both

### GJK distance algorithm

GJK can compute not only intersection (origin inside Minkowski difference)
but also the minimum distance between two non-overlapping convex shapes. This
is the standard "GJK distance" extension (van den Bergen, 2003):

1. Run GJK normally — build a simplex in Minkowski space
2. If the origin is NOT enclosed (shapes don't overlap):
   - The closest point on the current simplex to the origin gives the
     minimum Minkowski-space distance
   - The witness points (closest points on shapes A and B) are recovered
     from the simplex barycentric coordinates
   - Distance = `|witness_a - witness_b|`
3. If shapes DO overlap: return `None` (use EPA instead)

The existing `gjk_query()` (`gjk_epa.rs:595`) already builds the simplex and
detects non-intersection. The extension computes the closest point on the
simplex to the origin when non-intersecting, then extracts witness points.

Key addition: `closest_point_on_simplex()` function that handles:
- 1-simplex (point): return that point
- 2-simplex (line): project origin onto line segment, clamp to endpoints
- 3-simplex (triangle): project origin onto triangle plane, check Voronoi
  regions, return closest point on nearest feature
- 4-simplex (tetrahedron): NOT needed in distance mode — the simplex is
  always reduced to the closest sub-feature before reaching size 4
  (standard GJK distance property; in intersection mode the existing
  `Simplex` struct supports size 4, but distance mode never uses it)

### MULTICCD algorithm

**MuJoCo's multi-CCD mechanism:** When `ENABLE_MULTICCD` is set and a
convex-convex pair generates a valid contact, MuJoCo runs additional
penetration queries with perturbed initial search directions to find additional
contact points on flat contact surfaces.

**Empirical verification (MuJoCo 3.5.0, Python API):**

Two box-shaped meshes (vertices only, auto-hull) with flat face contact.
Upper mesh (half-size 0.3) resting on lower mesh (half-size 0.5) at z=0.75:

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
1. MULTICCD generates 4 contacts at the corners of the upper mesh's contact
   face, vs 1 contact without MULTICCD.
2. All 4 contacts have the same depth (same penetration on flat face).
3. MULTICCD does NOT affect ellipsoid-ellipsoid (curved contact — still 1
   contact). Confirms MULTICCD only generates additional points for flat
   contact patches.
4. MULTICCD does NOT affect box-box, box-plane, or cylinder-plane — these
   use analytical collision functions that already generate multi-contact,
   bypassing `mjc_penetration()`.
5. MULTICCD ONLY affects pairs that go through the general convex solver:
   mesh-mesh hull pairs, and GJK/EPA fallback pairs (cylinder-cylinder,
   cylinder-box, cylinder-ellipsoid, ellipsoid-ellipsoid, etc.) where flat
   contacts exist.

**CortenForge MULTICCD implementation strategy:** Run `gjk_epa_contact()` with
the first contact's negated normal as the initial direction for additional
queries. The perturbed directions are constructed by rotating the primary
contact normal by ±90° around two orthogonal axes in the contact tangent plane,
yielding 3 additional search directions (matching the 4-contact output from
EGT-7). Each additional call may find a different contact point on the flat
surface. Duplicate contacts (within tolerance) are filtered.

> **Uncertainty acknowledgement (rubric R13):** The perturbation scheme above
> (tangent-plane rotations yielding 3 extra directions) is inferred from the
> empirical EGT-7 output (4 contacts at box corners) and the high-level
> description of `mjc_penetration()` behavior. The exact perturbation vectors
> used by MuJoCo's C source have NOT been verified at the line-of-code level.
> During implementation, the C source `engine_collision_convex.c` MUST be read
> to confirm the exact perturbation scheme. If it differs, update the
> implementation to match. The acceptance criterion (AC8: 4 contacts at corners)
> is empirically ground-truthed regardless of the mechanism.

### Margin-zone contact gap

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
- ellipsoid-capsule (degenerate-case fallback from analytical)

Via `mesh_collide.rs:60` (mesh-mesh hull path):
- mesh-mesh (when both meshes have convex hulls)

**Empirical verification (MuJoCo 3.5.0, Python API):**

Two ellipsoids with `margin="0.1"` each (combined margin 0.2), separated
by ~0.05 (not overlapping but within margin):
```
Ellipsoid-ellipsoid, separation ≈ 0.05, combined margin = 0.2:
  ncon=1, contact.dist = 0.050 (positive = separated within margin)

Ellipsoid-ellipsoid, separation ≈ 0.05, combined margin = 0.02:
  ncon=0 (separation > margin, no contact)
```

MuJoCo DOES generate margin-zone contacts for convex solver pairs. CortenForge's
GJK/EPA path returns `None` for non-overlapping shapes — confirmed conformance gap.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| GJK/EPA iteration limit | Configurable via `ccd_iterations`, default 35. Single parameter controls both GJK and EPA. | Hardcoded: `GJK_MAX_ITERATIONS=64`, `EPA_MAX_ITERATIONS=64`. `ccd_iterations` parsed (default 50, WRONG) but not threaded to solver. |
| GJK/EPA convergence tolerance | Configurable via `ccd_tolerance`, default 1e-6. | Hardcoded: `EPA_TOLERANCE=1e-6`. `ccd_tolerance` not parsed. |
| Margin-zone contacts (GJK/EPA pairs) | Generated: convex solver returns separation distance for non-overlapping shapes, pipeline creates contact when `dist < margin`. | **Missing**: `gjk_epa_contact()` returns `None` for non-overlapping shapes. All GJK/EPA pairs miss margin-zone contacts. |
| Multi-point contacts (MULTICCD) | When enabled: runs penetration solver with multiple initial directions, generates up to 50 contacts per pair. | **Not implemented**: `tracing::warn!`, single contact only. |
| `DISABLE_NATIVECCD` flag | Disables native solver, falls back to libccd. | `tracing::warn!` no-op. Flag stored on Model but not acted on. |
| `ccd_iterations` default | 35 (verified MuJoCo 3.5.0) | 50 in `MjcfOption` (**WRONG**) |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `ccd_iterations` | Single int controlling both GJK and EPA loop limits | Separate `GJK_MAX_ITERATIONS` and `EPA_MAX_ITERATIONS` constants | Use `model.ccd_iterations` for both `gjk_query()` and `epa_query()` max iteration parameters |
| `ccd_tolerance` | Single float controlling convergence tolerance | `EPA_TOLERANCE` constant (GJK uses `EPSILON=1e-8` for direction checks, which is separate) | Use `model.ccd_tolerance` for `epa_query()` tolerance and `gjk_distance()` convergence threshold. Keep `EPSILON=1e-8` for geometric direction checks (not configurable) |
| Contact depth sign | `contact.dist`: positive = separated, negative = penetrating | `Contact.depth`: positive = penetrating, negative = separated | GJK distance result must be **negated** to produce `Contact.depth`. For margin-zone contacts: `depth = -separation_distance` (negative value, indicating non-penetrating contact) |
| Contact normal direction | `mjc_penetration()`: normal from shape_b toward shape_a | `GjkContact.normal`: from shape_b toward shape_a (same) | Direct port — no translation needed. `make_contact_from_geoms` already handles the convention. |
| Max contacts per pair | `mjMAXCONPAIR = 50` | `MAX_CONTACTS_PER_PAIR = 50` in `hfield.rs:17` | Share existing constant for MULTICCD rather than redefining |
| Model options location | `mjOption` struct — all options in one struct | Options spread across `Model` fields directly (e.g., `model.sdf_iterations`) | Add `ccd_iterations: usize` and `ccd_tolerance: f64` as direct fields on `Model`, following the `sdf_iterations` / `sdf_initpoints` pattern |
| `DISABLE_NATIVECCD` semantics | Bit set → use libccd fallback | No libccd exists | Conformant no-op: when `DISABLE_NATIVECCD` is set, continue using our GJK/EPA (there is nothing to fall back to) |

---

## Architecture Decisions

### AD-1: Multi-contact return type for MULTICCD

**Problem:** `collide_geoms()` (`narrow.rs:61`) returns `Option<Contact>` —
a single contact or nothing. MULTICCD requires returning multiple contacts
from a single geom pair. Changing the return type to `Vec<Contact>` affects
every caller (broadphase loops in `mod.rs` mechanism-1 and mechanism-2).

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Change `collide_geoms()` to return `Vec<Contact>` | Clean API, consistent with `collide_hfield_multi()` | Touches every caller; allocation overhead when MULTICCD disabled |
| 2 | Add separate `collide_geoms_multi()` function | No existing API change | Code duplication; two code paths to maintain |
| 3 | Use `SmallVec<[Contact; 4]>` return type | Avoids heap allocation for common case (1 or 4 contacts) | External dependency or manual inline-vec impl |

**Chosen:** Option 1 — Change `collide_geoms()` to return `Vec<Contact>`.
The `collide_hfield_multi()` path already uses `Vec<Contact>` and the
broadphase loop in `mod.rs:450-456` already iterates over multi-contact
results. Making `collide_geoms` consistent with this pattern is cleaner
than maintaining parallel code paths. The single-contact case returns a
1-element Vec (no heap allocation in practice due to Vec's inline optimization
for small sizes on most allocators). The `mesh_collide.rs` `collide_with_mesh()`
function must also be updated to return `Vec<Contact>` for the mesh-mesh hull
path's MULTICCD support.

### AD-2: GJK distance as modification of gjk_query vs separate function

**Problem:** Adding GJK distance computation. The existing `gjk_query()`
builds a simplex iteratively. GJK distance uses the same simplex construction
but with different termination: instead of checking "does the simplex enclose
the origin?", it tracks "what is the closest point on the simplex to the
origin?" and terminates when this distance converges.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Add parameters to `gjk_query()` to optionally compute distance | Reuses existing code | Complicates already-complex function; intersection mode doesn't need distance tracking |
| 2 | Write standalone `gjk_distance()` with shared helpers | Clean separation of concerns | Some code duplication (support_minkowski calls, simplex construction) |

**Chosen:** Option 2 — Standalone `gjk_distance()` function. GJK intersection
and GJK distance have fundamentally different iteration strategies: intersection
mode tries to enclose the origin (growing the simplex to 4 points), while
distance mode tracks the closest simplex feature to the origin (keeping the
simplex at 1-3 points). Merging them would add mode-dependent branching
throughout the loop body. The shared code (`support_minkowski`,
`MinkowskiPoint`, `Simplex`) is already factored for reuse.

---

## Specification

### S1. GJK Distance Query

**File:** `core/src/gjk_epa.rs` (new function, ~120 lines)
**MuJoCo equivalent:** GJK distance computation within `mjc_penetration()` in
`engine_collision_convex.c` — MuJoCo's convex solver returns both penetration
(overlapping) and separation distance (non-overlapping).
**Design decision:** Standalone `gjk_distance()` per AD-2. Uses the same
`support_minkowski()` and `Simplex` infrastructure as `gjk_query()` but with
distance-tracking iteration.

**New types:**

```rust
/// Result of GJK distance query for non-overlapping convex shapes.
#[derive(Debug, Clone)]
pub struct GjkDistanceResult {
    /// Minimum separating distance (> 0 if non-overlapping).
    pub distance: f64,
    /// Closest point on shape A (in world frame).
    pub witness_a: Point3<f64>,
    /// Closest point on shape B (in world frame).
    pub witness_b: Point3<f64>,
    /// Number of iterations used.
    pub iterations: usize,
}
```

**New function:**

```rust
/// Compute minimum separating distance between two non-overlapping convex shapes.
///
/// Returns `None` if shapes are overlapping (use `gjk_epa_contact` instead).
/// Uses GJK distance algorithm (van den Bergen, 2003): iteratively refine the
/// closest point on the Minkowski difference to the origin.
///
/// `max_iterations` and `tolerance` come from `model.ccd_iterations` and
/// `model.ccd_tolerance` respectively.
#[must_use]
pub fn gjk_distance(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,
    tolerance: f64,
) -> Option<GjkDistanceResult> {
    let center_a = pose_a.position;
    let center_b = pose_b.position;
    let dir_vec = center_b - center_a;
    let dir_norm = dir_vec.norm();

    let mut direction = if dir_norm > EPSILON {
        dir_vec / dir_norm
    } else {
        Vector3::x()
    };

    let mut simplex = Simplex::new();

    // Get first support point
    let first = support_minkowski(shape_a, pose_a, shape_b, pose_b, &direction);
    simplex.push(first);

    // Track closest distance to detect convergence
    let mut closest_dist_sq = first.point.coords.norm_squared();
    direction = -first.point.coords;

    for iteration in 0..max_iterations {
        let dir_norm_sq = direction.norm_squared();
        if dir_norm_sq < EPSILON * EPSILON {
            // Origin is on the simplex — shapes are touching (distance = 0)
            // Transition to EPA for penetration
            return None;
        }

        let dir_normalized = direction / dir_norm_sq.sqrt();
        let new_point = support_minkowski(shape_a, pose_a, shape_b, pose_b, &dir_normalized);

        // Check if we passed the origin — if so, shapes may overlap
        let dot = new_point.point.coords.dot(&dir_normalized);
        if dot < EPSILON {
            // Cannot get closer to origin in this direction — shapes don't overlap
            // Current simplex contains the closest feature
            break;
        }

        simplex.push(new_point);

        // Find closest point on simplex to origin and reduce simplex
        // to the closest sub-feature
        let (closest_point, bary) = closest_point_on_simplex_to_origin(&simplex);
        let new_dist_sq = closest_point.norm_squared();

        // Convergence check: relative distance improvement < tolerance.
        // Use relative criterion (improvement / current distance²) to handle
        // both small and large separations correctly. Falls back to absolute
        // tolerance when distance is near-zero to avoid division instability.
        let improvement = closest_dist_sq - new_dist_sq;
        if improvement < tolerance * tolerance
            || (closest_dist_sq > EPSILON && improvement / closest_dist_sq < tolerance) {
            // Converged — compute witness points from barycentric coords
            let (wa, wb) = recover_witness_points(&simplex, &bary);
            return Some(GjkDistanceResult {
                distance: new_dist_sq.sqrt(),
                witness_a: wa,
                witness_b: wb,
                iterations: iteration,
            });
        }

        closest_dist_sq = new_dist_sq;

        // Reduce simplex to closest sub-feature
        reduce_simplex_to_closest(&mut simplex, &bary);

        // New search direction: from closest point toward origin
        direction = -closest_point;
    }

    // Max iterations or early break — compute final result
    let (closest_point, bary) = closest_point_on_simplex_to_origin(&simplex);
    let dist = closest_point.norm();
    if dist < EPSILON {
        return None; // Touching — use EPA
    }
    let (wa, wb) = recover_witness_points(&simplex, &bary);
    Some(GjkDistanceResult {
        distance: dist,
        witness_a: wa,
        witness_b: wb,
        iterations: max_iterations,
    })
}
```

**Helper: `closest_point_on_simplex_to_origin()`**

Returns the closest point on the simplex to the origin, plus barycentric
coordinates for witness point recovery. Handles 1-3 simplex sizes:

```rust
/// Compute the closest point on the simplex to the origin.
///
/// Returns (closest_point_vector, barycentric_coordinates).
/// Barycentric coordinates are used to recover witness points from
/// the MinkowskiPoint's support_a/support_b fields.
fn closest_point_on_simplex_to_origin(
    simplex: &Simplex,
) -> (Vector3<f64>, Vec<(usize, f64)>) {
    match simplex.len() {
        1 => {
            // Point: closest point is the point itself
            let p = simplex.points[0].point.coords;
            (p, vec![(0, 1.0)])
        }
        2 => {
            // Line segment: project origin onto segment [A, B]
            let a = simplex.points[0].point.coords;
            let b = simplex.points[1].point.coords;
            let ab = b - a;
            let ao = -a;
            let t = ao.dot(&ab) / ab.norm_squared();
            let t = t.clamp(0.0, 1.0);
            let closest = a + t * ab;
            (closest, vec![(0, 1.0 - t), (1, t)])
        }
        3 => {
            // Triangle: project origin onto triangle plane, check Voronoi regions
            closest_point_on_triangle_to_origin(simplex)
        }
        _ => {
            // Should not happen in distance mode (simplex reduced before size 4)
            let p = simplex.points[0].point.coords;
            (p, vec![(0, 1.0)])
        }
    }
}
```

**Helper: `closest_point_on_triangle_to_origin()`**

Full Voronoi region analysis for the triangle case. Projects origin onto the
triangle ABC and checks all 7 Voronoi regions (3 vertices, 3 edges, 1 face):

```rust
fn closest_point_on_triangle_to_origin(
    simplex: &Simplex,
) -> (Vector3<f64>, Vec<(usize, f64)>) {
    let a = simplex.points[0].point.coords;
    let b = simplex.points[1].point.coords;
    let c = simplex.points[2].point.coords;

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);

    // Vertex A region
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, vec![(0, 1.0)]);
    }

    let bo = -b;
    let d3 = ab.dot(&bo);
    let d4 = ac.dot(&bo);

    // Vertex B region
    if d3 >= 0.0 && d4 <= d3 {
        return (b, vec![(1, 1.0)]);
    }

    // Edge AB region
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let closest = a + v * ab;
        return (closest, vec![(0, 1.0 - v), (1, v)]);
    }

    let co = -c;
    let d5 = ab.dot(&co);
    let d6 = ac.dot(&co);

    // Vertex C region
    if d6 >= 0.0 && d5 <= d6 {
        return (c, vec![(2, 1.0)]);
    }

    // Edge AC region
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let closest = a + w * ac;
        return (closest, vec![(0, 1.0 - w), (2, w)]);
    }

    // Edge BC region
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let closest = b + w * (c - b);
        return (closest, vec![(1, 1.0 - w), (2, w)]);
    }

    // Interior of triangle
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let closest = a + v * ab + w * ac;
    (closest, vec![(0, 1.0 - v - w), (1, v), (2, w)])
}
```

**Helper: `recover_witness_points()`**

```rust
/// Recover world-space witness points from barycentric coordinates.
fn recover_witness_points(
    simplex: &Simplex,
    bary: &[(usize, f64)],
) -> (Point3<f64>, Point3<f64>) {
    let mut wa = Point3::origin();
    let mut wb = Point3::origin();
    for &(idx, weight) in bary {
        wa += (simplex.points[idx].support_a.coords * weight);
        wb += (simplex.points[idx].support_b.coords * weight);
    }
    (wa, wb)
}
```

**Helper: `reduce_simplex_to_closest()`**

```rust
/// Reduce simplex to only the vertices that contribute to the closest point.
fn reduce_simplex_to_closest(simplex: &mut Simplex, bary: &[(usize, f64)]) {
    let active: Vec<MinkowskiPoint> = bary
        .iter()
        .filter(|(_, w)| *w > EPSILON)
        .map(|&(idx, _)| simplex.points[idx])
        .collect();
    simplex.set(&active);
}
```

### S2. Parameter Threading (Parse + Model + Wire)

**File:** `mjcf/src/types.rs`, `mjcf/src/parser.rs`, `mjcf/src/builder/mod.rs`,
  `core/src/types/model.rs`, `core/src/types/model_init.rs`
**MuJoCo equivalent:** `mjOption` struct fields in `mjmodel.h`
**Design decision:** Follow the `sdf_iterations`/`sdf_initpoints` pattern
  established by T1 (§57). Fields go directly on `Model`, threaded through
  builder. This pattern is already proven in the codebase.

**S2a. Fix `ccd_iterations` default (types.rs)**

```rust
// Before (types.rs:518):
pub ccd_iterations: usize,  // default 50

// After:
pub ccd_iterations: usize,  // default 35 (MuJoCo 3.5.0 verified)
```

The `Default` impl for `MjcfOption` must set `ccd_iterations` to **35**.

**S2b. Parse `ccd_tolerance` (parser.rs)**

Add parsing of `ccd_tolerance` from `<option>` element, alongside existing
`ccd_iterations` parsing at `parser.rs:253-254`:

```rust
if let Some(tol) = parse_float_attr(e, "ccd_tolerance") {
    option.ccd_tolerance = tol;
}
```

**S2c. Add `ccd_tolerance` to `MjcfOption` (types.rs)**

```rust
/// Convex collision solver convergence tolerance (default: 1e-6).
/// MuJoCo ref: `mjOption.ccd_tolerance` in `mjmodel.h`.
pub ccd_tolerance: f64,
```

Default: `1e-6`.

**S2d. Add fields to `Model` (model.rs, model_init.rs)**

```rust
// model.rs — alongside sdf_iterations/sdf_initpoints:
/// Maximum iterations for GJK/EPA convex collision solver (default: 35).
/// MuJoCo ref: `mjOption.ccd_iterations` in `mjmodel.h`.
pub ccd_iterations: usize,
/// Convergence tolerance for GJK/EPA convex collision solver (default: 1e-6).
/// MuJoCo ref: `mjOption.ccd_tolerance` in `mjmodel.h`.
pub ccd_tolerance: f64,
```

```rust
// model_init.rs — alongside sdf_iterations/sdf_initpoints defaults:
ccd_iterations: 35,  // MuJoCo default for convex solver iterations
ccd_tolerance: 1e-6, // MuJoCo default for convex solver tolerance
```

**S2e. Thread through builder (builder/mod.rs)**

In the model-building code where `sdf_iterations`/`sdf_initpoints` are
transferred from `MjcfConfig` to `Model`, add:

```rust
model.ccd_iterations = config.ccd_iterations;
model.ccd_tolerance = config.ccd_tolerance;
```

This requires adding `ccd_tolerance` to `ExtendedSolverConfig` (`config.rs`)
alongside the existing `ccd_iterations` field.

**S2f. Wire to `gjk_query()`, `epa_query()`, and internal callers (gjk_epa.rs)**

Note: `epa_with_expanded_simplex()` (`gjk_epa.rs:976`) recursively calls
`epa_query()`. This internal call must also receive `max_iterations` and
`tolerance` parameters. Update `epa_with_expanded_simplex()` signature to
accept and forward both parameters to its recursive `epa_query()` call.

Add `max_iterations` parameter to `gjk_query()`:
```rust
pub fn gjk_query(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,  // NEW: from model.ccd_iterations
) -> GjkResult {
    // ... replace GJK_MAX_ITERATIONS with max_iterations ...
}
```

Add `max_iterations` and `tolerance` parameters to `epa_query()`:
```rust
pub fn epa_query(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    simplex: &Simplex,
    max_iterations: usize,  // NEW: from model.ccd_iterations
    tolerance: f64,          // NEW: from model.ccd_tolerance
) -> Option<EpaResult> {
    // ... replace EPA_MAX_ITERATIONS with max_iterations ...
    // ... replace EPA_TOLERANCE with tolerance ...
}
```

Update `gjk_epa_contact()` to accept and forward parameters:
```rust
pub fn gjk_epa_contact(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,  // NEW
    tolerance: f64,          // NEW
) -> Option<GjkContact> {
    let gjk_result = gjk_query(shape_a, pose_a, shape_b, pose_b, max_iterations);
    if !gjk_result.intersecting {
        return None;
    }
    let epa = epa_query(shape_a, pose_a, shape_b, pose_b, &gjk_result.simplex,
                        max_iterations, tolerance)?;
    // ... rest unchanged ...
}
```

Update `gjk_intersection()` to accept `max_iterations`:
```rust
pub fn gjk_intersection(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,  // NEW
) -> bool {
    gjk_query(shape_a, pose_a, shape_b, pose_b, max_iterations).intersecting
}
```

The old constants `GJK_MAX_ITERATIONS`, `EPA_MAX_ITERATIONS`, and
`EPA_TOLERANCE` remain as module-level constants for use by tests that don't
have a Model context. They become the fallback defaults:

```rust
/// Default GJK max iterations (used when no Model context available).
/// MuJoCo default: 35. Historical CortenForge value: 64.
pub const GJK_MAX_ITERATIONS: usize = 35;
pub const EPA_MAX_ITERATIONS: usize = 35;
pub const EPA_TOLERANCE: f64 = 1e-6;
```

### S3. Margin-Zone Contact Generation

**File:** `core/src/collision/narrow.rs` (lines 186-198), `core/src/collision/mesh_collide.rs` (lines 57-71)
**MuJoCo equivalent:** `mjc_penetration()` returning separation distance for
  non-overlapping shapes, enabling margin-zone contacts in the dispatch pipeline.
**Design decision:** Add `gjk_distance()` fallback when `gjk_epa_contact()`
  returns `None`. This matches MuJoCo's behavior where the convex solver
  reports both penetration and separation.

**narrow.rs modification (lines 186-198):**

```rust
// Before:
if let Some(result) = gjk_epa_contact(&collision_shape1, &pose1, &collision_shape2, &pose2) {
    if result.penetration > -margin {
        return Some(make_contact_from_geoms(...));
    }
}

// After:
if let Some(result) = gjk_epa_contact(
    &collision_shape1, &pose1, &collision_shape2, &pose2,
    model.ccd_iterations, model.ccd_tolerance,
) {
    if result.penetration > -margin {
        return vec![make_contact_from_geoms(
            model,
            Vector3::new(result.point.x, result.point.y, result.point.z),
            result.normal,
            result.penetration,
            geom1, geom2, margin,
        )];
    }
} else if margin > 0.0 {
    // Shapes don't overlap — check if within margin distance
    if let Some(dist_result) = gjk_distance(
        &collision_shape1, &pose1, &collision_shape2, &pose2,
        model.ccd_iterations, model.ccd_tolerance,
    ) {
        if dist_result.distance < margin {
            // Margin-zone contact: depth is negative (separated)
            let depth = -dist_result.distance;
            let normal = (dist_result.witness_b - dist_result.witness_a).normalize();
            let midpoint = nalgebra::center(&dist_result.witness_a, &dist_result.witness_b);
            return vec![make_contact_from_geoms(
                model,
                midpoint.coords,
                normal,
                depth,
                geom1, geom2, margin,
            )];
        }
    }
}
```

**mesh_collide.rs modification (mesh-mesh hull path, lines 57-71):**

Same pattern: when `gjk_epa_contact` returns `None` for hull-hull pair, call
`gjk_distance()` and generate margin-zone contact if within margin:

```rust
if let (Some(hull1), Some(hull2)) = (mesh1.convex_hull(), mesh2.convex_hull()) {
    let shape1 = CollisionShape::convex_mesh_from_hull(hull1);
    let shape2 = CollisionShape::convex_mesh_from_hull(hull2);

    if let Some(gjk) = gjk_epa_contact(
        &shape1, &pose1, &shape2, &pose2,
        model.ccd_iterations, model.ccd_tolerance,
    ) {
        return vec![make_contact_from_geoms(
            model, gjk.point.coords, gjk.normal, gjk.penetration,
            geom1, geom2, margin,
        )];
    } else if margin > 0.0 {
        // Margin-zone contact for hull pairs
        if let Some(dist) = gjk_distance(
            &shape1, &pose1, &shape2, &pose2,
            model.ccd_iterations, model.ccd_tolerance,
        ) {
            if dist.distance < margin {
                let depth = -dist.distance;
                let normal = (dist.witness_b - dist.witness_a).normalize();
                let midpoint = nalgebra::center(&dist.witness_a, &dist.witness_b);
                return vec![make_contact_from_geoms(
                    model, midpoint.coords, normal, depth,
                    geom1, geom2, margin,
                )];
            }
        }
        return vec![];
    }
    return vec![];
}
```

### S4. MULTICCD Multi-Point Contact Generation

**File:** `core/src/collision/narrow.rs`, `core/src/collision/mesh_collide.rs`
**MuJoCo equivalent:** MULTICCD flag in `mjc_penetration()` — runs penetration
  solver with perturbed initial directions.
**Design decision:** Implement as a post-hoc loop after the primary contact is
  found. Generate 3 additional search directions by rotating the primary
  contact normal by ±90° around the two tangent axes. This produces 4 total
  contacts matching the MuJoCo 3.5.0 empirical result (EGT-7).

**MULTICCD helper function (new, in narrow.rs or gjk_epa.rs):**

Note: `MAX_CONTACTS_PER_PAIR` in `hfield.rs:17` is currently `const` (private).
It must be changed to `pub(crate) const` so `narrow.rs` can import it for the
MULTICCD cap. This is a visibility-only change — no behavioral impact.

```rust
use crate::types::{ENABLE_MULTICCD, enabled};
use crate::collision::hfield::MAX_CONTACTS_PER_PAIR;

/// Generate multiple contacts for a convex-convex pair using MULTICCD.
///
/// Runs GJK/EPA with perturbed initial search directions to find additional
/// contact points on flat contact surfaces. Returns 1-4 contacts (primary
/// + up to 3 additional).
///
/// Only called when `ENABLE_MULTICCD` is set.
fn multiccd_contacts(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    primary: &GjkContact,
    max_iterations: usize,
    tolerance: f64,
) -> Vec<GjkContact> {
    let mut contacts = vec![primary.clone()];

    // Generate 3 perturbed search directions by rotating the primary
    // contact normal ±90° around two orthogonal tangent axes.
    let (t1, t2) = compute_tangent_frame(&primary.normal);
    let perturbed_dirs = [t1, -t1, t2];

    for dir in &perturbed_dirs {
        if contacts.len() >= MAX_CONTACTS_PER_PAIR {
            break;
        }

        // Run GJK/EPA with perturbed initial direction
        if let Some(additional) = gjk_epa_contact_with_direction(
            shape_a, pose_a, shape_b, pose_b,
            *dir, max_iterations, tolerance,
        ) {
            // Filter duplicates: skip if too close to existing contact
            let dominated = contacts.iter().any(|existing| {
                let dp = (additional.point - existing.point).norm();
                dp < tolerance
            });
            if !dominated {
                contacts.push(additional);
            }
        }
    }

    contacts
}
```

**New `gjk_query_with_direction()` (gjk_epa.rs):**

A variant of `gjk_query` that accepts an initial search direction instead of
computing one from `center_b - center_a`. Used by `gjk_epa_contact_with_direction()`
below.

```rust
/// GJK intersection query with a specified initial search direction.
///
/// Identical to `gjk_query()` except the initial Minkowski search direction
/// is `initial_direction` instead of `center_b - center_a`. This causes
/// the simplex construction to start from a different support vertex,
/// potentially finding a different contact region on shapes with flat faces.
pub fn gjk_query_with_direction(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    initial_direction: Vector3<f64>,
    max_iterations: usize,
) -> GjkResult {
    // Same as gjk_query() body, but replace the initial direction computation
    // (center_b - center_a) with initial_direction. All other logic identical.
}
```

**New `gjk_epa_contact_with_direction()` (gjk_epa.rs):**

A variant of `gjk_epa_contact` that accepts an initial search direction
instead of using `center_b - center_a`:

```rust
/// GJK/EPA contact with a specified initial search direction.
///
/// Used by MULTICCD to explore different contact regions on flat surfaces.
pub fn gjk_epa_contact_with_direction(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    initial_direction: Vector3<f64>,
    max_iterations: usize,
    tolerance: f64,
) -> Option<GjkContact> {
    let gjk_result = gjk_query_with_direction(
        shape_a, pose_a, shape_b, pose_b,
        initial_direction, max_iterations,
    );
    if !gjk_result.intersecting {
        return None;
    }
    let epa = epa_query(
        shape_a, pose_a, shape_b, pose_b,
        &gjk_result.simplex, max_iterations, tolerance,
    )?;
    let contact_point = support(shape_a, pose_a, &(-epa.normal));
    Some(GjkContact {
        point: contact_point,
        normal: epa.normal,
        penetration: epa.depth.max(0.0),
    })
}
```

**Integration in narrow.rs:**

After obtaining the primary contact via `gjk_epa_contact`, check
`ENABLE_MULTICCD` and generate additional contacts:

```rust
// In the GJK/EPA fallback section of collide_geoms():
if let Some(result) = gjk_epa_contact(
    &collision_shape1, &pose1, &collision_shape2, &pose2,
    model.ccd_iterations, model.ccd_tolerance,
) {
    if result.penetration > -margin {
        let primary = make_contact_from_geoms(
            model, Vector3::new(result.point.x, result.point.y, result.point.z),
            result.normal, result.penetration, geom1, geom2, margin,
        );

        if enabled(model, ENABLE_MULTICCD) {
            let multi = multiccd_contacts(
                &collision_shape1, &pose1, &collision_shape2, &pose2,
                &result, model.ccd_iterations, model.ccd_tolerance,
            );
            // Primary contact first, then additional contacts.
            // multiccd_contacts() returns [primary, additional...] so we
            // convert all contacts, preserving order (primary at index 0).
            return multi.into_iter()
                .map(|gjk| make_contact_from_geoms(
                    model,
                    Vector3::new(gjk.point.x, gjk.point.y, gjk.point.z),
                    gjk.normal, gjk.penetration, geom1, geom2, margin,
                ))
                .collect();
        }

        return vec![primary];
    }
}
```

Same pattern for `mesh_collide.rs` mesh-mesh hull path.

### S5. Return Type Migration + Flag Cleanup

**File:** `core/src/collision/narrow.rs`, `core/src/collision/mesh_collide.rs`,
  `core/src/collision/mod.rs`, `mjcf/src/builder/mod.rs`
**MuJoCo equivalent:** N/A (internal plumbing)
**Design decision:** Per AD-1, change `collide_geoms()` return type from
  `Option<Contact>` to `Vec<Contact>`. Update all callers. Remove
  `tracing::warn!` stubs for MULTICCD/NATIVECCD.

**S5a. `collide_geoms()` return type change:**

```rust
// Before:
pub fn collide_geoms(...) -> Option<Contact> {

// After:
pub fn collide_geoms(...) -> Vec<Contact> {
```

All early returns change from `return Some(contact)` to `return vec![contact]`
and from `return None` to `return vec![]`. The GJK/EPA section returns
`Vec<Contact>` populated by S3 (margin-zone) and S4 (MULTICCD).

**S5b. `collide_with_mesh()` return type change:**

```rust
// Before:
pub fn collide_with_mesh(...) -> Option<Contact> {

// After:
pub fn collide_with_mesh(...) -> Vec<Contact> {
```

The mesh-mesh hull path returns `Vec<Contact>` (S3/S4). Mesh-primitive paths
remain single-contact but wrapped in `vec![]`.

**S5c. Broadphase loop updates (mod.rs):**

Mechanism-1 loop (lines 457-464):
```rust
// Before:
} else if let Some(contact) = collide_geoms(...) {
    data.contacts.push(contact);
    data.ncon += 1;
}

// After:
} else {
    let contacts = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    for contact in contacts {
        data.contacts.push(contact);
        data.ncon += 1;
    }
}
```

Mechanism-2 loop (lines 520-533):
```rust
// Before:
} else if let Some(mut contact) = collide_geoms(...) {
    apply_pair_overrides(&mut contact, pair);
    // ...

// After:
} else {
    let contacts = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    for mut contact in contacts {
        apply_pair_overrides(&mut contact, pair);
        apply_global_override(model, &mut contact, pair.gap);
        if !enabled(model, ENABLE_OVERRIDE) {
            contact.mu = assign_friction(model, &contact.mu);
            contact.friction = contact.mu[0];
        }
        data.contacts.push(contact);
        data.ncon += 1;
    }
}
```

**S5d. Remove `tracing::warn!` stubs (builder/mod.rs:933-938):**

```rust
// Remove these lines:
if flag.multiccd {
    tracing::warn!("ENABLE_MULTICCD set but CCD (§50) not implemented — flag has no effect");
}
if !flag.nativeccd {
    tracing::warn!("DISABLE_NATIVECCD set but CCD (§50) not implemented — flag has no effect");
}
```

**S5e. `DISABLE_NATIVECCD` handling:**

CortenForge has no libccd. When `DISABLE_NATIVECCD` is set, we continue using
our GJK/EPA solver (conformant no-op). No runtime behavior change needed —
just remove the warning. The flag remains stored on `Model.disableflags` for
model fidelity.

**S5f. Dead code: `config.rs:154` `ccd_enabled()` method:**

This method is never called anywhere in the codebase. Leave it in place
(it may become useful when collision enables/disables CCD features per-model).
Its logic should be updated to reflect the correct semantics:

```rust
pub fn ccd_enabled(&self) -> bool {
    self.ccd_iterations > 0
}
```

Remove the `&& self.flags.nativeccd` check — `nativeccd` controls native-vs-libccd
choice, not whether convex collision is enabled at all.

### S6. Update `hfield.rs` Call Site

**File:** `core/src/collision/hfield.rs` (line 163)
**MuJoCo equivalent:** N/A (blast radius from S2f signature change)
**Design decision:** Update the `gjk_epa_contact` call in `hfield.rs` to pass
  the new parameters. No algorithmic change — just signature compatibility.

```rust
// Before:
gjk_epa_contact(&prism_shape, &hf_pose, &conv_shape, &conv_pose)

// After:
gjk_epa_contact(
    &prism_shape, &hf_pose, &conv_shape, &conv_pose,
    model.ccd_iterations, model.ccd_tolerance,
)
```

The `collide_hfield_multi` function already receives `model: &Model`, so
`model.ccd_iterations` and `model.ccd_tolerance` are available.

---

## Acceptance Criteria

### AC1: GJK distance — separated spheres *(runtime test, analytically derived)*
**Given:** Two `CollisionShape::Sphere` shapes, radii 1.0 each, poses
  separated by 4.0 units along X-axis.
**After:** `gjk_distance()` called with `max_iterations=35`, `tolerance=1e-6`.
**Assert:** `distance = 2.0 ± 1e-6` (center distance 4.0 minus sum of radii 2.0).
  `witness_a.x ≈ 1.0`, `witness_b.x ≈ 3.0`.
**Field:** `GjkDistanceResult.distance`, `.witness_a`, `.witness_b`

### AC2: GJK distance — overlapping shapes return None *(runtime test, analytically derived)*
**Given:** Two `CollisionShape::Sphere` shapes, radii 1.0 each, poses
  separated by 1.0 unit along X-axis (overlapping).
**After:** `gjk_distance()` called.
**Assert:** Returns `None` (shapes overlap — use EPA instead).
**Field:** Return value is `None`

### AC3: Margin-zone contact — ellipsoid pair *(runtime test, MuJoCo-verified)*
**Given:** Two ellipsoid geoms with `margin="0.1"` each (combined margin 0.2),
  separated by ~0.05 (not overlapping but within margin).
**After:** `collide_geoms()` called with `margin=0.2`.
**Assert:** Returns 1 contact with `depth ≈ -0.05` (negative = separated).
  MuJoCo 3.5.0 produces `ncon=1, contact.dist = 0.050` (positive in MuJoCo
  convention, negated for our depth convention).
**Field:** `Contact.depth`

### AC4: Margin-zone contact — beyond margin produces no contact *(runtime test, MuJoCo-verified)*
**Given:** Same two ellipsoid geoms with `margin="0.01"` each (combined 0.02),
  separated by ~0.05 (beyond margin).
**After:** `collide_geoms()` called with `margin=0.02`.
**Assert:** Returns empty `Vec` (no contact — separation exceeds margin).
**Field:** Return value is empty

### AC5: `ccd_iterations` wiring — non-default value reaches solver *(runtime test)*
**Given:** MJCF with `<option ccd_iterations="10"/>`.
**After:** Model built from MJCF.
**Assert:** `model.ccd_iterations == 10`. When `gjk_epa_contact()` is called
  with `max_iterations=10`, GJK/EPA respects the limit (verifiable by setting
  iterations to 1 and confirming degraded convergence vs default 35).
**Field:** `Model.ccd_iterations`

### AC6: `ccd_tolerance` parsing — non-default value reaches solver *(runtime test)*
**Given:** MJCF with `<option ccd_tolerance="1e-8"/>`.
**After:** Model built from MJCF.
**Assert:** `model.ccd_tolerance == 1e-8`.
**Field:** `Model.ccd_tolerance`

### AC7: `ccd_iterations` default fixed to 35 *(runtime test)*
**Given:** MJCF with no `ccd_iterations` specified.
**After:** Model built from MJCF.
**Assert:** `model.ccd_iterations == 35` (was incorrectly 50).
**Field:** `Model.ccd_iterations`

### AC8: MULTICCD — mesh-mesh hull pair produces 4 contacts *(runtime test, MuJoCo-verified)*
**Given:** Two box-shaped mesh geoms (convex hulls), upper mesh (half-size 0.3)
  resting on lower mesh (half-size 0.5) at z=0.75. `<flag multiccd="true"/>`.
**After:** `collide_geoms()` (or `collide_with_mesh()`) called.
**Assert:** Returns 4 contacts, one at each corner of the upper mesh's bottom
  face (±0.3, ±0.3). All contacts have equal depth. Matches MuJoCo 3.5.0
  empirical output (EGT-7).
**Field:** `Vec<Contact>` length and positions

### AC9: MULTICCD disabled — single contact *(runtime test)*
**Given:** Same mesh-mesh setup as AC8, but `multiccd` disabled (default).
**After:** `collide_geoms()` called.
**Assert:** Returns 1 contact (single contact, no MULTICCD).
**Field:** `Vec<Contact>` length

### AC10: `DISABLE_NATIVECCD` — no crash, solver works *(runtime test)*
**Given:** MJCF with `<flag nativeccd="false"/>`.
**After:** Model built and simulation stepped.
**Assert:** No crash, no `tracing::warn!` emitted, collision detection works
  normally (our GJK/EPA continues operating).
**Field:** Model builds and runs without error

### AC11: `tracing::warn!` guards removed *(code review)*
**Assert:** `builder/mod.rs` no longer contains `tracing::warn!` messages for
  MULTICCD or NATIVECCD. The lines at `builder/mod.rs:933-938` are deleted.

### AC12: Existing GJK/EPA tests pass with updated signatures *(runtime test)*
**Given:** All 20 existing `#[test]` functions in `gjk_epa.rs`.
**After:** Signatures updated to accept `max_iterations`/`tolerance` parameters.
**Assert:** All 20 tests pass using the updated default constants
  (`GJK_MAX_ITERATIONS=35`, `EPA_MAX_ITERATIONS=35`, `EPA_TOLERANCE=1e-6`).
**Field:** Test pass/fail

### AC13: Return type migration — no semantic change for single-contact paths *(code review)*
**Assert:** All analytical collision paths in `collide_geoms()` (sphere-sphere,
  capsule-capsule, etc.) return `vec![contact]` or `vec![]` with identical
  logic to the pre-migration `Some(contact)` or `None`. No new contacts
  introduced for non-GJK/EPA paths.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (GJK distance separated) | T1 | Direct |
| AC2 (GJK distance overlapping) | T2 | Direct |
| AC3 (Margin-zone ellipsoid) | T3 | Direct |
| AC4 (Beyond margin no contact) | T4 | Direct + edge case |
| AC5 (ccd_iterations wiring) | T5 | Direct |
| AC6 (ccd_tolerance parsing) | T6 | Direct |
| AC7 (ccd_iterations default) | T7 | Direct |
| AC8 (MULTICCD 4 contacts) | T8 | Direct |
| AC9 (MULTICCD disabled) | T9 | Direct |
| AC10 (DISABLE_NATIVECCD) | T10 | Direct |
| AC11 (warn guards removed) | — | Code review (manual) |
| AC12 (existing tests pass) | T11 | Regression |
| AC13 (return type migration) | — | Code review (manual) |

---

## Test Plan

### T1: GJK distance — separated spheres → AC1
Two `CollisionShape::Sphere{radius: 1.0}` at `x=0` and `x=4`. Call
`gjk_distance()` with `max_iterations=35, tolerance=1e-6`. Assert
`distance ≈ 2.0`, `witness_a.x ≈ 1.0`, `witness_b.x ≈ 3.0`.
Analytically derived (sphere support function is exact).

### T2: GJK distance — overlapping spheres return None → AC2
Two `CollisionShape::Sphere{radius: 1.0}` at `x=0` and `x=1` (overlapping).
Call `gjk_distance()`. Assert returns `None`.

### T3: Margin-zone contact — ellipsoid pair within margin → AC3
Build a minimal Model with two ellipsoid geoms, `margin=0.1` each, separated
by ~0.05. Call `collide_geoms()` with combined `margin=0.2`. Assert returns
1 contact with `depth ≈ -0.05` (negative = separated within margin).
MuJoCo 3.5.0 verified: `ncon=1, contact.dist=0.050`.

### T4: Beyond margin — no contact generated → AC4
Same ellipsoid setup but with `margin=0.01` each (combined 0.02). Separation
~0.05 exceeds margin. Assert `collide_geoms()` returns empty Vec.

### T5: `ccd_iterations` non-default reaches Model → AC5
Parse `<option ccd_iterations="10"/>`. Assert `model.ccd_iterations == 10`.
Also test: set `ccd_iterations=1` and verify GJK/EPA produces degraded results
(lower penetration accuracy or failure to converge) vs `ccd_iterations=35`.

### T6: `ccd_tolerance` non-default reaches Model → AC6
Parse `<option ccd_tolerance="1e-8"/>`. Assert `model.ccd_tolerance == 1e-8`.

### T7: `ccd_iterations` default is 35 → AC7
Parse MJCF with no `ccd_iterations` attribute. Assert `model.ccd_iterations == 35`.

### T8: MULTICCD — mesh-mesh hull pair → AC8
Build Model with two box-shaped mesh geoms (will auto-compute convex hulls via
Spec A), upper mesh resting on lower, `multiccd="true"`. Run collision.
Assert 4 contacts returned, positioned at the 4 corners of the upper mesh's
bottom face. All contacts have equal depth.
MuJoCo 3.5.0 verified: 4 contacts at (±0.30, ±0.30).

### T9: MULTICCD disabled — single contact → AC9
Same setup as T8 but `multiccd` disabled (default). Assert 1 contact returned.

### T10: DISABLE_NATIVECCD — no crash → AC10
Build Model with `<flag nativeccd="false"/>`. Run collision between two
overlapping convex shapes. Assert no crash, contact generated normally.

### T11: Existing GJK/EPA tests regression → AC12
Run all 20 existing tests in `gjk_epa.rs` with the updated function signatures.
Tests use the module-level default constants (now 35/35/1e-6 instead of
64/64/1e-6). Assert all pass. The lower iteration count (35 vs 64) should not
affect any existing test because all test shapes converge well within 35
iterations.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Coincident shapes (distance=0) | GJK distance at boundary — should return `None` (transition to EPA) | T2 | AC2 |
| Shapes just touching (distance ≈ 0) | Numerical stability at zero-distance boundary | T12 | AC1 |
| Zero margin | Margin-zone path should not be entered (`margin > 0.0` guard) | T13 | AC3, AC4 |
| Large separation (no contact) | GJK distance terminates quickly, returns distance > margin | T4 | AC4 |
| `ccd_iterations=0` | Edge case: should GJK/EPA run at all? MuJoCo behavior TBD — likely immediate termination with no result | T14 | AC5 |
| `ccd_tolerance=0` | Edge case: zero tolerance may cause solver to never converge (improvement never < 0). Should behave like very tight tolerance — run to max iterations | T18 | AC6 |
| MULTICCD on curved contact | Single contact expected (no flat surface to find additional points on) | T15 | AC8 |
| GJK distance with box shapes | Non-sphere shapes exercise more complex simplex cases | T16 | AC1 |
| Multi-body mixed geom scene | Integration test with mixed geom types (sphere, capsule, ellipsoid, mesh) verifying no regression in a realistic scene | T19 | AC12, AC13 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T12 (just touching) | Two spheres exactly touching (distance = 0 ± epsilon) | Boundary between GJK distance and EPA modes |
| T13 (zero margin) | `collide_geoms()` with `margin=0.0` for GJK/EPA pair | Verify margin-zone path is not entered when margin is zero |
| T14 (iterations=0) | `gjk_epa_contact()` with `max_iterations=0` | Verify graceful handling of edge case |
| T15 (MULTICCD curved) | Two ellipsoids with MULTICCD enabled | Verify no additional contacts for curved surfaces |
| T16 (GJK distance boxes) | Two separated boxes, verify distance computation | Exercises triangle simplex case in `closest_point_on_simplex_to_origin` |
| T17 (sensor geom_distance signature compat) | `sensor/geom_distance.rs` calls `gjk_epa_contact()` and `gjk_query()` with updated signatures | Verify sensor geom_distance still works after signature changes; existing crude `closest_points_from_simplex()` fallback (lines 96-112) unchanged — improving it with `gjk_distance()` is deferred to sensor-side work |
| T18 (ccd_tolerance=0) | `gjk_epa_contact()` with `tolerance=0.0` | Verify solver runs to max iterations and still produces a valid contact (convergence check `< 0` is never true, so it exhausts iterations then returns final result) |
| T19 (multi-body integration) | Multi-body scene with sphere, capsule, ellipsoid, box, and mesh geoms in contact | Verify no regression in realistic scene — all pairs produce contacts consistent with pre-migration behavior (single-contact paths unchanged, GJK/EPA pairs may gain margin-zone contacts) |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `gjk_epa_contact()` signature | Takes 4 params (shapes + poses) | Takes 6 params (+max_iterations, +tolerance) | Toward MuJoCo | All callers (narrow.rs, mesh_collide.rs, hfield.rs, tests) | Add `model.ccd_iterations, model.ccd_tolerance` at call sites |
| `gjk_query()` signature | Takes 4 params | Takes 5 params (+max_iterations) | Toward MuJoCo | `gjk_intersection()`, `gjk_epa_contact()` | Forward parameter |
| `epa_query()` signature | Takes 5 params | Takes 7 params (+max_iterations, +tolerance) | Toward MuJoCo | `gjk_epa_contact()` | Forward parameters |
| Default iteration count | `GJK_MAX_ITERATIONS=64` | 35 (MuJoCo default) | Toward MuJoCo | Edge cases that converge between 35-64 iterations | Conformance improvement — MuJoCo uses 35 |
| `MjcfOption.ccd_iterations` default | 50 | 35 | Toward MuJoCo | Models relying on default | Conformance fix |
| `collide_geoms()` return type | `Option<Contact>` | `Vec<Contact>` | N/A (internal) | mod.rs broadphase loops | Iterate over Vec instead of pattern-match Option |
| `collide_with_mesh()` return type | `Option<Contact>` | `Vec<Contact>` | N/A (internal) | narrow.rs (mesh dispatch) | Same |
| Margin-zone contacts for GJK/EPA pairs | Not generated | Generated when shapes within margin | Toward MuJoCo | Models with ellipsoid/cylinder margin contacts | New contacts appear where none existed — conformance improvement |
| MULTICCD contacts | Not generated (single contact) | Up to 4 contacts for flat surfaces when flag enabled | Toward MuJoCo | Models with `multiccd="true"` | Additional contacts for stability |
| `tracing::warn!` removal | Warning logged for MULTICCD/NATIVECCD flags | No warning | N/A | Log consumers | Fewer false warnings |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `core/src/gjk_epa.rs` | Add `gjk_distance()`, `gjk_epa_contact_with_direction()`, helpers; update `gjk_query()`, `epa_query()`, `gjk_epa_contact()`, `gjk_intersection()` signatures; update constants | +180, ~30 modified |
| `core/src/collision/narrow.rs` | Return type `Option<Contact>` → `Vec<Contact>`; add margin-zone fallback; add MULTICCD integration | ~60 modified, +30 |
| `core/src/collision/mesh_collide.rs` | Return type change; margin-zone for hull path; MULTICCD for hull path | ~40 modified, +25 |
| `core/src/collision/mod.rs` | Update broadphase loops for Vec return type | ~20 modified |
| `core/src/collision/hfield.rs` | Update `gjk_epa_contact` call site (add params); change `MAX_CONTACTS_PER_PAIR` to `pub(crate)` | ~4 modified |
| `core/src/sensor/geom_distance.rs` | Update `gjk_epa_contact` call (line 79) and `gjk_query` call (line 97) signatures. Existing crude GJK distance fallback (lines 96-112 using `closest_points_from_simplex()`) may benefit from the new `gjk_distance()` — evaluate during implementation | ~6 modified |
| `core/src/convex_hull.rs` | Update test at line 1115 calling `gjk_epa_contact()` — add params | ~2 modified |
| `core/src/types/model.rs` | Add `ccd_iterations`, `ccd_tolerance` fields | +6 |
| `core/src/types/model_init.rs` | Add defaults for new fields | +2 |
| `mjcf/src/types.rs` | Add `ccd_tolerance` to `MjcfOption`; fix `ccd_iterations` default | ~4 modified |
| `mjcf/src/parser.rs` | Parse `ccd_tolerance` from `<option>` | +3 |
| `mjcf/src/config.rs` | Add `ccd_tolerance` to `ExtendedSolverConfig`; update `ccd_enabled()` | +4, ~2 modified |
| `mjcf/src/builder/mod.rs` | Thread `ccd_iterations`/`ccd_tolerance` to Model; remove `tracing::warn!` stubs | +3, -6 |
| Test files | New tests for all ACs | +250 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_gjk_spheres_intersecting` | `gjk_epa.rs` | Signature update (add params) | Default constants unchanged in value (35 sufficient) |
| `test_gjk_spheres_separated` | `gjk_epa.rs` | Signature update | Same |
| `test_gjk_epa_contact_spheres` | `gjk_epa.rs` | Signature update | Same |
| (all 20 GJK/EPA tests) | `gjk_epa.rs` | Signature update, all pass | Default iterations reduced 64→35 but all test shapes converge well within 35 |
| `test_option_parsing_ccd_iterations` | `parser.rs` | Pass unchanged | Parses explicit value, not affected by default change |
| Hfield collision tests | `hfield.rs` | `gjk_epa_contact` call site update | Parameters threaded from test Model |
| Mesh collision tests | `mesh_collide.rs` tests | Return type change: `Option<Contact>` → `Vec<Contact>` | Test assertions update from `.is_some()` to `.len() >= 1` |
| Sensor geom_distance tests | `sensor/geom_distance.rs` | `gjk_epa_contact` (line 79) and `gjk_query` (line 97) signature updates | Parameters threaded from Model; existing crude `closest_points_from_simplex()` fallback (lines 96-112) still works but may be improved with `gjk_distance()` |
| `convex_hull.rs` test (line 1115) | `convex_hull.rs` | `gjk_epa_contact` call signature update | Add default params |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `narrow.rs:104-106` | Sphere-sphere analytical dispatch | Analytical path — handles margin correctly, not GJK/EPA |
| `narrow.rs:110-113` | Capsule-capsule analytical dispatch | Same — analytical, margin-correct |
| `narrow.rs:144-147` | Box-box SAT dispatch | Same — analytical multi-contact already |
| `collision/plane.rs` | Plane collision | Analytical — not affected by GJK/EPA changes |
| `collision/sdf_collide.rs` | SDF collision | Separate subsystem — not GJK/EPA |
| `collision/pair_convex.rs` | Sphere/capsule/box analytical pairs | Not GJK/EPA |
| `collision/pair_cylinder.rs` | Cylinder analytical pairs | Cylinder-capsule degenerate fallback reaches GJK/EPA, but the analytical path itself is unchanged |

---

## Execution Order

1. **S2 first** (parameter threading) — parse `ccd_tolerance`, fix defaults,
   add fields to Model. This is pure plumbing with no behavioral change.
   Run tests to verify parsing. This also updates `gjk_query()`, `epa_query()`,
   `gjk_epa_contact()`, and `gjk_intersection()` signatures to accept the new
   parameters, which is a prerequisite for all subsequent sections.

2. **S1 second** (GJK distance query) — standalone new function with no
   callers yet. Test in isolation with T1, T2, T12, T16.

3. **S6 third** (hfield.rs call site update) — update `gjk_epa_contact` call
   in `hfield.rs` to pass new params from S2. Also update `sensor/geom_distance.rs`
   and `convex_hull.rs` call sites. Simple signature compatibility — no
   behavioral change. These must happen alongside or before S5 because all
   callers need updated signatures before the return type migration.

4. **S5 fourth** (return type migration + flag cleanup) — change
   `collide_geoms()`/`collide_with_mesh()` return types from `Option<Contact>`
   to `Vec<Contact>`, update broadphase loops in `mod.rs`. Remove
   `tracing::warn!` stubs. Run full domain tests to verify no regression.
   **Note:** S5 depends on S2 (signatures already updated) and S6 (all call
   sites already compatible). S5 does NOT change `gjk_epa_contact` signature —
   that was done in S2.

5. **S3 fifth** (margin-zone contacts) — depends on S1 (`gjk_distance()` exists)
   and S5 (return type is `Vec<Contact>`). Run T3, T4, T13.

6. **S4 sixth** (MULTICCD) — depends on S5 (multi-contact return type) and S2
   (parameterized `gjk_epa_contact`). Adds `gjk_query_with_direction()` and
   `gjk_epa_contact_with_direction()`. Run T8, T9, T15.

After each section lands, run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`
to verify conformance before proceeding.

---

## Performance Characterization

**Cost:** `gjk_distance()` has the same O(n) per-iteration complexity as
`gjk_query()` where n is vertex count. Each iteration performs one
`support_minkowski()` call (2 support function evaluations) and one
`closest_point_on_simplex_to_origin()` call (O(1) for simplex sizes 1-3).
Convergence is typically 10-20 iterations for common shapes.

**MULTICCD cost:** Up to 3 additional `gjk_epa_contact()` calls per pair when
MULTICCD enabled. Each call is O(GJK iterations + EPA iterations). For the
default `ccd_iterations=35`, worst case is ~4 × 35 = 140 iterations per pair.
This only applies when `ENABLE_MULTICCD` is set (disabled by default).

**Margin-zone cost:** One additional `gjk_distance()` call per GJK/EPA pair
when `gjk_epa_contact` returns `None` and `margin > 0`. This replaces the
current behavior of returning `None` immediately, so cost is `O(GJK iterations)`
additional per near-miss pair. In practice, margin-zone contacts are a small
fraction of total pairs (most pairs are either overlapping or far apart).

**Acceptable?** Yes. The per-pair cost increase is bounded and only affects
GJK/EPA pairs (not analytical pairs which are the majority). MULTICCD is
opt-in (disabled by default).

---

## Out of Scope

- **Conservative advancement / time-of-impact CCD** — MuJoCo does not
  implement this feature. Dropped from scope during rubric verification.
  *Conformance impact: none — MuJoCo doesn't have it.*

- **Velocity-based CCD pair filtering** — Does not exist in MuJoCo.
  *Conformance impact: none.*

- **libccd fallback implementation** — CortenForge's GJK/EPA IS the convex
  solver. No need for a second implementation. `DISABLE_NATIVECCD` is a
  conformant no-op. *Conformance impact: none.*

- **Non-convex GJK distance** — GJK distance only works for convex shapes.
  HeightField, SDF, and TriangleMesh are non-convex and have their own
  collision paths. *Conformance impact: none.*

- **MULTICCD for analytical pairs** — Box-box, sphere-sphere, etc. already
  generate multi-contact through analytical paths. MULTICCD only affects
  pairs routed through GJK/EPA. *Conformance impact: none — matches MuJoCo.*

- **GJK/EPA warm-starting** — Using previous frame's simplex as starting
  point. Performance optimization, not conformance. Tracked as potential
  future work. *Conformance impact: none.*

- **`ccd_enabled()` integration into collision pipeline** — The
  `config.rs:154` method exists but is never called. Wiring it into the
  collision pipeline to conditionally skip convex collision is out of scope.
  *Conformance impact: none — MuJoCo does not conditionally skip convex
  collision based on `ccd_iterations > 0`.*
