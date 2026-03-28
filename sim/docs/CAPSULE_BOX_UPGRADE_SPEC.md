# Capsule-Box Collision Upgrade: MuJoCo 4-Phase Algorithm

**Date:** 2026-03-27
**Branch:** `feature/integrator-examples`
**Status:** Spec ready, implementation pending
**Parent spec:** `MULTI_CONTACT_ANALYSIS.md` Phase 3b

## Context

Phase 3b of the multi-contact foundation used a simplified approach for
capsule-box: test both endpoints as sphere-box + 5-point heuristic midpoint
sampling. This works for the common case (capsule lying on face → 2 contacts)
but uses a heuristic where MuJoCo uses exact geometry. This upgrade replaces
the heuristic with MuJoCo's full 4-phase `mjraw_CapsuleBox` algorithm.

**Reference:** `mjraw_CapsuleBox` (`engine_collision_box.c:118–590`)

---

## MuJoCo's Algorithm

MuJoCo finds 1–2 positions along the capsule segment, then delegates to
sphere-box collision at each. The key insight: the algorithm never constructs
contacts directly — it finds capsule parameter values `t ∈ [0,1]`, then
places a sphere (with capsule radius) at `cap_a + (cap_b - cap_a) * t` and
runs sphere-box at that position.

### Phase 1 — Face test (MuJoCo lines 179–208)

Transform capsule endpoints to box-local frame. For each endpoint, count
how many coordinates exceed box half-extents:
- 0 clamped → endpoint is inside the box
- 1 clamped → endpoint projects onto a face (closest feature is a face)
- 2+ clamped → endpoint is near an edge or corner (handled by Phase 2)

If an endpoint is on a face (≤1 clamped), record the feature:
- `face_axis`: which box axis (0/1/2) was clamped
- `is_endpoint_a`: which capsule endpoint
- Distance: Euclidean distance from endpoint to its clamped point

Priority: face test runs first. If a face feature is found with distance
less than any edge feature, it wins.

### Phase 2 — Edge test (MuJoCo lines 212–305)

A box has 12 edges: 4 parallel to each of the 3 local axes.

For each axis `a ∈ {0,1,2}`:
  For each corner `(sb, sc) ∈ {-1,+1}²` (where `b=(a+1)%3`, `c=(a+2)%3`):
    Edge from `[-half[a], sb*half[b], sc*half[c]]`
         to   `[+half[a], sb*half[b], sc*half[c]]`

For each of the 12 edges, compute closest point pair between the capsule
segment `[local_a, local_b]` and the edge segment using the parametric
variant of `closest_points_segments`. This gives:
- `s`: capsule parameter (0=endA, 1=endB)
- `t`: edge parameter (0=edge start, 1=edge end)
- `clamp_s`: 0=clamped-to-start, 1=interior, 2=clamped-to-end
- `clamp_t`: same encoding for edge parameter

Track the globally minimum distance across all 12 edges. Record:
- `t1 = s` (capsule parameter for primary contact)
- `clamp_capsule = clamp_s`
- `clamp_edge = clamp_t`
- `best_edge_axis = a` (which axis the winning edge is parallel to)

After Phases 1–2: have the primary capsule parameter `t1` and a feature
classification (Face or Edge with clamping metadata).

### Phase 3 — Second contact search (MuJoCo lines 398–566)

Based on the feature type, compute a second capsule parameter `t2`:

**Sub-case 3A: Edge, capsule endpoint clamped (`clamp_capsule ≠ 1`)**

The primary contact is at a capsule endpoint. The second contact is at the
opposite endpoint:
- `clamp_capsule == 0` → `t1 = 0.0`, `t2 = 1.0`
- `clamp_capsule == 2` → `t1 = 1.0`, `t2 = 0.0`

**Sub-case 3B: Edge, capsule interior (`clamp_capsule == 1`)**

T/X crossing configuration — the capsule crosses over a box edge in the
middle. Test both capsule endpoints as sphere-box, pick the one with smaller
distance to the box:
- Compute `dist_a = sphere_box_distance(local_a)`
- Compute `dist_b = sphere_box_distance(local_b)`
- `t2 = if dist_a < dist_b { 0.0 } else { 1.0 }`

**Sub-case 3C: Face closest**

One capsule endpoint projects onto a box face. Walk toward the other endpoint,
clamping to the face boundary:
- `t_on_face` = parameter of the on-face endpoint (0.0 or 1.0)
- `t_other` = parameter of the other endpoint (1.0 or 0.0)
- For each non-face axis `i ≠ face_axis`, compute where the capsule segment
  exits the face region (where coordinate `i` first exceeds `±half[i]`):
  ```
  delta = other_local[i] - on_face_local[i]
  if other exceeds +half[i]: t_cross = (half[i] - on_face[i]) / delta
  if other exceeds -half[i]: t_cross = (-half[i] - on_face[i]) / delta
  ```
  Clamp `t2` to the minimum of these crossings (first face exit).
- If the other endpoint is also on the face, `t2` is simply `t_other`.

**Filtering:** Skip `t2` if:
- `|t2 - t1| < GEOM_EPSILON` (same point)
- Sphere-box at `t2` doesn't penetrate (depth ≤ -margin)

### Phase 4 — Sphere-box delegation (MuJoCo lines 569–589)

Convert `t1` (and `t2` if found) to world-space sphere centers:
```
center = cap_a + (cap_b - cap_a) * t
```

For each center, run the existing `sphere_box_test` closure (clamp to box,
compute distance, check penetration, handle inside-box degenerate).
Dedup contacts within 1mm. Return 0–2 contacts.

---

## Implementation Steps

### Step 1: Add `closest_points_segments_parametric` utility

**File:** `sim/L0/core/src/forward/position.rs` (after line 460)
**Re-export:** `sim/L0/core/src/forward/mod.rs` (line 59)

```rust
/// Like `closest_points_segments` but also returns clamping state.
/// Returns `(s, t, clamp_s, clamp_t)` where:
/// - `s`, `t`: clamped parameter values ∈ [0, 1]
/// - `clamp_s`, `clamp_t`: 0=clamped-to-start, 1=interior, 2=clamped-to-end
pub fn closest_points_segments_parametric(
    p1: Vector3<f64>, q1: Vector3<f64>,
    p2: Vector3<f64>, q2: Vector3<f64>,
) -> (f64, f64, u8, u8)
```

Built by refactoring the existing function's clamping logic (lines 442–457
of position.rs) to record decisions instead of discarding them.

### Step 2: Rewrite `collide_capsule_box`

**File:** `sim/L0/core/src/collision/pair_cylinder.rs`

**Keep unchanged:**
- Function signature (called from `narrow.rs:128–135`)
- Preamble: capsule/box identification, g1/g2 ordering, normal_sign
- `sphere_box_test` closure

**Replace:** Everything after the closure (current lines ~340–394) with
the 4-phase algorithm described above.

**Extract helper:** `fn find_second_contact_t(...)` for Phase 3 logic.

**Add types:**
```rust
enum CapsuleBoxFeature {
    Face { face_axis: usize, is_endpoint_a: bool },
    Edge { clamp_capsule: u8, clamp_edge: u8, edge_axis: usize },
}
```

### Step 3: Add tests

**File:** `sim/L0/tests/integration/collision_primitives.rs`

New tests:
- `capsule_box_edge_contact` — capsule near box edge (not face), 1 contact
- `capsule_box_crossing_edge` — capsule perpendicular to edge (T config),
  1–2 contacts
- `capsule_box_oblique_face` — capsule 45° to face, one end on, other off

**Must pass existing:**
- `capsule_box_face_contact` (separated, 0 contacts)
- `capsule_box_face_overlap` (1–2 contacts, depth=0.05)
- `capsule_box_face_2_contacts` (2 contacts)

### Step 4: Run tests, update spec

```
cargo test -p sim-core -p sim-conformance-tests
cargo clippy -p sim-core -- -D warnings
```

All 1972 existing tests must pass. Update Phase 3b in
`MULTI_CONTACT_ANALYSIS.md` to reflect the full algorithm.

---

## Critical files

| File | Change |
|------|--------|
| `sim/L0/core/src/collision/pair_cylinder.rs` | Main rewrite (~250 LOC) |
| `sim/L0/core/src/forward/position.rs` | New utility (~60 LOC) |
| `sim/L0/core/src/forward/mod.rs` | Re-export (1 line) |
| `sim/L0/tests/integration/collision_primitives.rs` | New tests (~80 LOC) |
| `sim/docs/MULTI_CONTACT_ANALYSIS.md` | Update Phase 3b description |

## 12 Box Edges Enumeration

```
For axis a in {0, 1, 2}:
  b = (a+1) % 3
  c = (a+2) % 3
  For (sb, sc) in {(-1,-1), (-1,+1), (+1,-1), (+1,+1)}:
    start[a] = -half[a],  end[a] = +half[a]
    start[b] = sb*half[b], end[b] = sb*half[b]
    start[c] = sc*half[c], end[c] = sc*half[c]
```

## Feature Type Encoding (MuJoCo mapping)

| MuJoCo `cltype` | Meaning | Our enum |
|-----------------|---------|----------|
| -3 | endA on face | `Face { is_endpoint_a: true }` |
| -1 | endB on face | `Face { is_endpoint_a: false }` |
| 0 | capsule-start × edge-start | `Edge { clamp_capsule: 0, clamp_edge: 0 }` |
| 1 | capsule-start × edge-interior | `Edge { clamp_capsule: 0, clamp_edge: 1 }` |
| 2 | capsule-start × edge-end | `Edge { clamp_capsule: 0, clamp_edge: 2 }` |
| 3 | capsule-interior × edge-start | `Edge { clamp_capsule: 1, clamp_edge: 0 }` |
| 4 | capsule-interior × edge-interior | `Edge { clamp_capsule: 1, clamp_edge: 1 }` |
| 5 | capsule-interior × edge-end | `Edge { clamp_capsule: 1, clamp_edge: 2 }` |
| 6 | capsule-end × edge-start | `Edge { clamp_capsule: 2, clamp_edge: 0 }` |
| 7 | capsule-end × edge-interior | `Edge { clamp_capsule: 2, clamp_edge: 1 }` |
| 8 | capsule-end × edge-end | `Edge { clamp_capsule: 2, clamp_edge: 2 }` |

## Degenerate Cases

- **Capsule inside box:** All coordinates clamp to interior, distances are 0.
  `sphere_box_test` handles via nearest-face fallback. Phase 1 finds both
  endpoints inside (0 clamped), Phase 3 generates contacts at both endpoints.

- **Parallel edges:** `closest_points_segments_parametric` returns `s=0`
  with recomputed `t` when determinant ≈ 0 (matching existing behavior at
  position.rs:433). Clamping state reported as `clamp_s=0`.

- **Face-edge boundary:** When distances from Phase 1 and Phase 2 are
  within `GEOM_EPSILON`, prefer face (Phase 1) over edge (Phase 2). This
  matches MuJoCo's ordering where face is checked first.
