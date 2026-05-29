# CF-CAST Arbitrary-Seam-Plane Silhouette Recon

> **Status:** RECON SCAFFOLD (pre-implementation). Cold-read pass-1 at end (§7).
> **Date:** 2026-05-29
> **Trigger:** the apex-anchored best-fit flat seam (`seam_fit::best_fit_planar_seam`,
> committed `5f3673d1`, item A §4.1 of `CF_CAST_ORGANIC_PARTS_RECON.md`) fits a
> balanced flat plane to a leaning organic part and **builds the cup shell on every
> layer**, but with the flange on, the outer layers go **non-manifold**. A flange-off
> regen proved the seam cut is fine — the **flange/bolt/dowel silhouette** is the
> blocker. This arc removes that blocker so the fitted seam can be the default.
> **Composes on:** the seam-fit foundation (`5f3673d1`) — `planar_seam_fit` is the
> opt-in that this arc makes safe + eventually default.

---

## 1. Problem statement

`best_fit_planar_seam` returns a flat seam plane with an arbitrary (diagonal)
normal — on `3quartachub`, `[0.838, 0.546, −0.001]`, ~33° off the `Y` axis. The
cup-wall half-space cut + the seam plane itself handle that fine (the cup shell
builds on all layers). But three downstream consumers build a **2D silhouette in
the world X-Z plane at a constant Y**, baking in the assumption that the seam
normal is `≈ Y`:

- **`flange::FlangeSdf`** — lateral term `silhouette.signed_distance_to(p.x, p.z)`
  + seam-offset term `(p − seam_midpoint)·binormal`. The silhouette is sampled at
  `seam_plane_y` (`flange.rs:288+`), and its own docstring admits this is "approximately
  the body's seam-plane cross-section" only "for nearly-Y-aligned binormals".
- **`bolt_pattern`** — `Silhouette2d::from_body_at_y(...)`, then maps the 2D point
  back to world as `Point3::new(off·n.x + p.x, seam_midpoint.y, off·n.z + p.z)`.
- **`dowel_hole`** — same `from_body_at_y` + same `(x, seam_y, z)` mapping.

At a 33° seam the X-Z cross-section is NOT the body's true seam-plane
cross-section, so the flange silhouette is malformed → the flange slab
self-intersects / leaves shards on the larger outer-layer bodies → MC emits a
non-manifold mesh → `manifold3d NotManifold` at the mesh→Manifold conversion.

**Goal:** generalize the silhouette + its three consumers from "X-Z plane at
constant Y" to "an arbitrary seam plane `(anchor, U, V, normal)`", so the flange/
bolt/dowel are correct at any seam orientation — while keeping every existing
(Y-normal / binormal-seam) cast **byte-identical**.

---

## 2. Architecture (as found — file:line)

| Concept | What it is | Source |
|---|---|---|
| `Silhouette2d` | unordered marching-squares segments in `(x, z)` | `silhouette_2d.rs:91` |
| `from_body_at_y(body, seam_y, x_min..z_max)` | samples `body.evaluate(P)` on an **X-Z grid at fixed Y** | `silhouette_2d.rs:110` |
| `signed_distance_to(x, z)` | 2D signed distance to the segments | `silhouette_2d.rs:252` |
| `point_at_arc_fraction(t) -> Point2{x,z}` | arc-length point on the longest polyline | `silhouette_2d.rs:402` |
| `outward_normal_at_arc_fraction(t) -> Point2{x,z}` | 2D outward normal (a direction) | `silhouette_2d.rs:445` |
| `FlangeSdf.eval` | `body_dist = signed_distance_to(p.x, p.z)`; `dist_from_seam = (p−mid)·binormal` | `flange.rs:253` |
| `build_flange_solid` | builds silhouette at `seam_midpoint.y` | `flange.rs:288` |
| bolt center map | `Point3(off·n.x+p.x, seam_y, off·n.z+p.z)` then project onto seam plane | `bolt_pattern.rs:342` |
| dowel center map | identical pattern | `dowel_hole.rs:259` |
| seam plane source | `Ribbon::seam_plane_reference() -> (point, Unit<normal>)` (honors `planar_seam`) | `ribbon.rs:739` |

**Load-bearing assumption (everywhere above):** the seam plane is `Y = const`, its
2D coordinates are world `(X, Z)`, and "outboard offset in the seam plane" is a
nudge in world `(X, Z)`. True only when the seam normal ≈ `Y`.

---

## 3. Proposed design

### 3.1 The seam-plane basis
Define the seam plane by an orthonormal frame:
- `anchor A` — `seam_plane_reference().0` (the fitted plane's point = cap centroid).
- `normal N` — `seam_plane_reference().1` (the fitted plane's unit normal).
- `U, V` — any orthonormal pair spanning the plane (`U ⟂ N`, `V = N × U`). Pick `U`
  deterministically (e.g. project world `+Z` onto the plane; fall back to `+X` if
  `N ∥ Z`) so the basis is stable + reproducible (determinism contract).

In-plane 2D coords of a world point `P`: `u = (P − A)·U`, `v = (P − A)·V`. Map
back: `P = A + u·U + v·V`. Off-plane distance: `(P − A)·N`.

### 3.2 Generalize `Silhouette2d`
Add `from_body_in_plane(body, anchor, U, V, u_min, u_max, v_min, v_max)` that
samples `body.evaluate(A + u·U + v·V)` on the `(u, v)` grid (same marching-squares
core; `Point2` becomes in-plane `(u, v)` instead of world `(x, z)`). **Store the
basis `(A, U, V)` on the `Silhouette2d`** so consumers can map results to world.
Add helpers: `to_world(Point2) -> Point3`, `dir_to_world(Point2) -> Vector3`.

Keep `from_body_at_y` as a thin wrapper (`A = (0, seam_y, 0)`, `U = +X`, `V = +Z`)
so the default path is **bit-identical** to today.

### 3.3 Consumers
- **`FlangeSdf`**: store `(A, U, V, N)`. `body_dist = silhouette.signed_distance_to((p−A)·U, (p−A)·V)`; `dist_from_seam = (p−A)·N`. (Today's `p.x/p.z` + `·binormal` is the `U=X,V=Z,N=Y` special case.)
- **`bolt_pattern` / `dowel_hole`**: build the silhouette in-plane; map center =
  `silhouette.to_world(p_silh) + off · silhouette.dir_to_world(n_out)`; the existing
  "project onto seam plane" step becomes a no-op (already in-plane) but is kept as a
  guard.

### 3.4 Wiring + byte-identical strategy
The cleanest switch: have `build_flange_solid` / bolt / dowel ask the ribbon for the
seam **basis**. Add `Ribbon::seam_plane_basis() -> (A, U, V, N)`:
- **planar-seam-fit set** → the fitted `(A, N)` + derived `(U, V)` → in-plane path.
- **otherwise** (binormal seam or curve-following) → return `(A=(0,seam_y,0), U=+X,
  V=+Z, N=binormal)` so the consumers reduce to **exactly** `from_body_at_y` + the
  current maps → byte-identical. Gate the in-plane path on "is the seam appreciably
  off-Y" (or simply on `planar_seam_fit`) to guarantee zero change for existing casts.

### 3.5 Flip the default (S4)
Once flange/bolt/dowel build manifold at the fitted seam (regen-verified on
`3quartachub` + a curved fixture), make `planar_seam` use the fit by default (when
caps exist) and retire / keep `planar_seam_fit` as an override per
[[feedback-strip-the-knob-when-default-works]].

---

## 4. Phasing

| Phase | Scope | Deliverable |
|---|---|---|
| **S1** | `Silhouette2d::from_body_in_plane` + stored basis + `to_world`/`dir_to_world`; `from_body_at_y` wrapper. Unit-test in-plane == X-Z when basis is `(X,Z)`. | generalized silhouette, default unchanged |
| **S2** | `FlangeSdf` uses the basis; `Ribbon::seam_plane_basis()`. Flatness/extent gate on a tilted-seam fixture. | manifold flange at a diagonal seam |
| **S3** | `bolt_pattern` + `dowel_hole` use the basis. Existing-cast byte-identity test (Y-normal path). | bolts + dowels at a diagonal seam |
| **S4** | Regen `3quartachub` with `planar_seam_fit` → confirm manifold + balanced + F4; curved-fixture regression; then flip default + grade-all. | fitted seam shippable by default |

Each phase: cold-read + full gates (`cargo xtask grade-all`), per prior arcs.

---

## 5. Risks / unknowns
1. **Byte-identity for existing casts.** The Y-normal reduction must be EXACT (same
   grid origin, same sample points) or every shipped cast's STL shifts. Mitigate:
   keep `from_body_at_y` as the literal default path; only the fitted seam takes the
   new code. A regression test diffs a Y-normal cast's flange mesh pre/post.
2. **MC bounds in the tilted plane.** `build_flange_solid` expands an AABB by
   `flange_width`; in a tilted plane the flange's world AABB is larger/rotated — the
   MC region must still cover it. Size the `(u, v)` grid from the body's in-plane
   extent, not the world AABB.
3. **Demoldability ≠ manifold.** A diagonal seam can still trap an undercut on a
   strongly curved part (recon OQ2). This arc fixes *buildability*; the curvature
   ceiling for *release* is separate (organic-parts §6 Risk #1).
4. **`U` basis stability.** A poorly-chosen `U` (e.g. near-parallel to `N`) gives a
   degenerate frame; pick `U` robustly + deterministically (determinism contract).
5. **Flange grad.** `FlangeSdf::grad` returns the binormal arbitrarily (unused by
   MC); keep returning `N` so nothing downstream breaks.

## 6. Open questions for workshop
- **OQ1:** flip `planar_seam` to fit-by-default once S1–S3 land (and keep
  `planar_seam_fit` as an escape hatch), or leave fit opt-in for iter-1?
- **OQ2:** `U` convention — project world `+Z` into the plane (keeps "up" roughly
  up, nice for print orientation) vs the cap→apex axis? (Affects only the 2D
  parameterization + arc-fraction start, not the geometry.)
- **OQ3:** is a diagonal seam acceptable to the workshop ergonomically (the mold
  opens diagonally, bolts on a diagonal flange), or should the fit be constrained to
  keep the flange roughly vertical for bench handling?

---

## 7. Cold-read pass-1
- **C1 (the fix is a coordinate generalization, not new geometry).** Every consumer
  already does the right *operations* (2D silhouette distance, outboard offset,
  seam-offset slab) — they just hardcode the basis to `(X, Z, Y)`. Framing S1 as
  "add a basis + map through it" keeps the change mechanical + testable, and the
  Y-normal reduction is the natural byte-identity guard.
- **C2 (byte-identity is the top risk, not correctness).** The new math is
  straightforward; the danger is perturbing the thousands of shipped Y-normal casts.
  S1 must keep `from_body_at_y` as the literal default and prove (test) the in-plane
  path with an `(X,Z)` basis is identical — otherwise every existing flange STL moves.
- **C3 (don't conflate manifold with demoldable).** S4's regen proves the flange
  *builds*; it does NOT prove the diagonal split *releases*. Keep the curvature
  ceiling (organic-parts OQ2) as a separate gate so we don't ship a buildable mold
  that won't open.
- **C4 (MC bounds are a sneaky failure).** A correct in-plane silhouette with a
  too-small `(u,v)` grid clips the flange outer edge → a different non-manifold.
  Size the grid from in-plane body extent + `flange_width`, and assert coverage.
