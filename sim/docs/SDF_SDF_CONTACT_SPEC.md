# SDF-SDF Contact Algorithm Redesign

## Status: PARTIALLY IMPLEMENTED — surface-tracing in place, resolution issue remains

## Problem

`sdf_sdf_contact` cannot support stacking at 1mm grid resolution.
Two spheres placed vertically collapse into each other instead of stacking.

### What was fixed

- **Surface-tracing algorithm** replaces the old volume-sampling approach.
  Traces from each SDF's surface into the other with a configurable margin.
  Stress tests pass: correct upward net force for stacking configuration.

- **Contact margin** implemented — detects contacts before full penetration.

### What still fails

**Equatorial contact dominance at coarse resolution.** At 1mm cell size
on 5mm-radius spheres, the surface-tracing finds contacts around the
equatorial intersection ring (where both sphere surfaces cross at the
same Z level). These contacts have horizontal normals that push the
spheres apart laterally but not vertically. The polar contacts (which
would have vertical normals for stacking) either don't exist as grid
points or are overwhelmed by the equatorial contacts.

At fine resolution (0.065mm cells in stress tests), enough polar grid
points exist and the algorithm works. At 1mm cells, the grid is too
coarse to resolve the contact geometry.

### Root cause (updated)

The surface-tracing algorithm iterates grid points of the SRC SDF that
are near its surface. For a sphere with 1mm cells, the surface is
sampled at ~100 grid points. Near the pole (top/bottom), the number of
grid points on the exact contact cap is small (maybe 1-3), while the
equatorial ring has many more points (the ring has larger circumference).
The equatorial contacts dominate the contact set even though they have
the wrong (horizontal) normals for vertical stacking.

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

### Option C: Normal-weighted contact selection (new)

Keep the surface-tracing algorithm but weight contacts by how well their
normal aligns with the separation direction (line between centers of mass).
This would prefer polar contacts over equatorial ones for stacking.

**Pros:** Small change to existing algorithm, preserves multi-contact.
**Cons:** Requires computing separation direction, may not generalize
to concave geometry.

### Recommendation

**Option A with Option C filtering.** Surface-trace to get candidates,
then weight by normal alignment with separation direction. Drop contacts
whose normals are nearly perpendicular to the separation axis. This
combines the generality of surface-tracing with the focus of line-of-
centers, and should work at any resolution.

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
