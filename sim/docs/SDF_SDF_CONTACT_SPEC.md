# SDF-SDF Contact Algorithm Redesign

## Status: SPEC — not yet implemented

## Problem

The current `sdf_sdf_contact` algorithm cannot support stacking or stable
SDF-vs-SDF contact. Two spheres placed on top of each other collapse into
the same position instead of stacking. Discovered during sdf-physics
example 07 (pair).

### Root causes

1. **No contact margin.** Contact is only detected when both SDF distances
   are strictly negative (full penetration). Bodies must already be
   interpenetrating before any force is generated. With high-velocity
   impacts or gravity, this means the bodies overlap significantly before
   the solver can react.

2. **Volume sampling, not surface intersection.** The algorithm samples
   a 3D grid in the AABB overlap region and collects points where both
   SDFs are negative. These points are scattered throughout the overlap
   volume — deep inside both bodies. Even with surface projection (added
   in this session), the projected contact points land ON one surface
   where the OTHER surface has near-zero penetration (because the surfaces
   are nearly coincident at the contact region). This produces tiny
   penetration depths that generate insufficient contact force.

3. **No focused contact patch.** For two spheres touching at one point,
   the contact should be a single well-defined point with a clear normal
   (line between centers). Instead, the algorithm produces 20+ scattered
   contacts with varying normals, whose forces partially cancel.

### What works (for reference)

- **sdf_plane_contact** works perfectly with multi-contact because:
  - The plane is infinite and analytical — no sampling ambiguity
  - Contact points are on the SDF surface, projected via gradient
  - Penetration is the surface point's distance below the plane
  - All normals are identical (plane normal) — forces don't cancel

- **MuJoCo analytical contacts** (sphere-sphere, box-box, etc.) work because:
  - Contact point is geometrically computed (e.g., midpoint on line between centers)
  - Normal is the separation direction
  - Penetration is the overlap distance
  - Margin parameter detects contact before full penetration

## Proposed approach

### Option A: Surface-tracing (recommended)

Instead of sampling the overlap volume, trace the contact from the
**surface** of each SDF:

1. **Identify candidate surface points.** For SDF A, iterate grid points
   near its surface (|dist_a| < threshold). For each surface point,
   transform to world space and query SDF B's distance.

2. **Contact criterion.** A surface point on SDF A is a contact candidate
   if `dist_b_at_surface_a < margin` (the point is inside or near the
   surface of B). The margin allows pre-penetration detection.

3. **Contact point.** The surface point on SDF A (already projected via
   gradient, same as sdf_plane_contact).

4. **Penetration.** `margin - dist_b_at_surface_a` (positive when the
   surface point is within the margin of B's interior). When
   `dist_b < 0`, this gives actual penetration depth. When
   `0 < dist_b < margin`, this gives a "soft" pre-contact force.

5. **Normal.** The gradient of SDF A at the contact point (outward from
   A's surface). This is the separation direction.

6. **Symmetry.** Also trace from SDF B's surface into SDF A (swap roles).
   Deduplicate contacts from both passes.

**Why this is better:**
- Contact points are always on an actual surface (not floating in space)
- Penetration is measured from a surface into the other body (physically meaningful)
- Normal is the surface gradient (well-defined separation direction)
- Margin allows early detection before interpenetration
- Same pattern as sdf_plane_contact (proven to work)

### Option B: Line-of-centers (simpler, less general)

Compute the line between the two bodies' centers of mass. Find where
each SDF surface crosses this line. Contact point is the midpoint,
normal is the line direction, penetration is the overlap along the line.

**Pros:** Simple, exact for spheres, single well-defined contact.
**Cons:** Only works for convex shapes, gives only 1 contact, doesn't
generalize to concave geometry (sockets).

### Recommendation

**Option A** — it generalizes to concave geometry (needed for steps 13-14)
and produces multiple contacts naturally. Option B could be a fast-path
for convex-convex cases but isn't worth implementing separately.

## Scope

- `sdf_sdf_contact` in `sim/L0/core/src/sdf/operations.rs`
- `collide_with_sdf` dispatcher (already handles Vec)
- Add `margin` parameter (can use model-level `geom_margin` values)
- Tests in `operations.rs`
- Verification: example 07 (pair) — two spheres stack stably

## Contact margin

MuJoCo uses `geom_margin` to extend the contact detection range beyond
the geometric surface. For SDF contacts, the margin should be:
- At least 1 cell size (to catch contacts within grid resolution)
- Configurable per-geom (use existing `model.geom_margin` array)
- Default: `cell_size * 1.0` (one grid cell of lookahead)

## Dependencies

- Blocks: examples 07-pair, 08-stack, 09+ (all SDF-SDF interactions)
- Does NOT block: examples 04-rest, 05-drop, 06-slide (SDF-plane only)
