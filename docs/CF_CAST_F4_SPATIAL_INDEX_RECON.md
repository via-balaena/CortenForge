# cf-cast F4 spatial-index recon (2026-05-25)

**Status:** scaffold. Follow-up arc to
[[project-cf-cast-parallel-meshing-s2]] (commit `d3ff8063`),
empirically motivated by the production iter-1 regen at S2:

| Metric | Pre-S1 baseline | Post-S2 measured |
|--------|----------------:|-----------------:|
| Wall clock | 33.6 min | **12.9 min (2.6×)** |
| Gasket F4 share of total | ~85% | **78%** |
| Per-gasket F4 time (longest) | 539 s | 587 s (~9% slower under 3-way contention) |

Parallel meshing extracted ~all the easy speedup. The remaining
~80% of wall-clock is the F4 gate on the 3 gasket molds; phase-
level concurrency would save only ~30-60 s. The algorithmic
optimization is the next layer.

**Picked architectural fix:** add a parry3d BVH (already a
workspace transitive dep via `mesh-sdf::TriMeshDistance`) inside
`mesh-printability::flag_thin_wall_faces`, replacing its current
pure O(face²) Möller-Trumbore double-loop with O(face × log face)
BVH-accelerated ray-casts. Same closest-hit answer to within FP
precision; same flagged-faces set in the validation output.

## Bottleneck attribution (NOT a guess)

`mesh/mesh-printability/src/validation.rs:785` `flag_thin_wall_faces`:

```rust
for i in 0..num_triangles {            // O(faces) outer
    // ... compute ray origin (centroid - ε·normal), direction (-normal)
    for (j, other) in mesh.faces.iter().enumerate() {  // O(faces) inner
        if j == i { continue; }
        if let Some(t) = moller_trumbore(origin, direction, ...) {
            if t < min_dist { min_dist = t; }
        }
    }
}
```

Cost on a 400 k-face gasket mesh: `400_000 × 400_000 = 1.6 × 10¹¹`
ray-triangle intersections, single-threaded. At ~3.5 ns per Möller-
Trumbore (modern CPU, well-optimized): **~560 s**. Observed: 540 s
serial / 587 s under 3-way parallel contention. Model matches.

**Other F4 checks audited** (`validate_for_printing` runs 8 in
sequence):

| Check | Complexity | Observation |
|-------|------------|-------------|
| `check_build_volume` | O(verts) | <1 s, not a target |
| `check_overhangs` | O(faces) normal test | <1 s |
| `check_basic_manifold` | O(edges) edge analysis | <1 s |
| **`check_thin_walls`** (`flag_thin_wall_faces`) | **O(face²) ray-cast double-loop** | **~540 s — bottleneck** |
| `check_long_bridges` | O(boundary_faces²?) | small for closed meshes |
| `check_trapped_volumes` | O(voxel-volume flood-fill) | bounded by AABB volume |
| `check_self_intersecting` | O(face²) AABB-pre-filtered + rayon-parallel | sparse geometry → fast |
| `check_small_features` | O(faces) BFS for components | <1 s |

**One check, one fix.** `check_thin_walls` is the only O(face²)
operation without a spatial-index acceleration in the F4 stack;
`check_self_intersecting` already has AABB pre-filtering + parallel
outer loop. The recon targets `flag_thin_wall_faces` specifically.

## Why parry3d's BVH (not a roll-your-own)

[`mesh_sdf::TriMeshDistance`](../mesh/mesh-sdf/src/lib.rs) is
already in the workspace + uses `parry3d::query::PointQuery` on a
parry3d-built BVH (`parry3d::shape::TriMesh`). Battle-tested via
cf-scan-prep's `flood_filled_sdf` pipeline + cf-device-design's
distance queries. Adding it as a direct dep of `mesh-printability`
is the smallest-blast change.

parry3d's `TriMesh::cast_local_ray` returns the nearest hit's `t`
(parametric distance along the ray) — directly substitutes the
inner Möller-Trumbore loop's `min_dist` accumulator.

**Rejected alternatives:**

- **Roll a BVH from scratch** → REJECTED. ~300-500 LOC of well-
  tested workspace code already does this; recreating it is pure
  ego.
- **R-tree** (`rstar` crate) → DEFER. Better for high-aspect-ratio
  scenes but parry3d's BVH is good enough for the gasket mesh's
  AABB distribution (tray + channel, low aspect).
- **k-d tree** → REJECTED. Slower for triangle-intersection queries
  vs BVH on real triangle meshes.
- **CUDA / wgpu offload** → DEFER. Same per-arc reasoning as
  `docs/CF_CAST_PARALLEL_MESHING_RECON.md` §P-15 footer: the algo
  win comes first; GPU offload is the next layer if needed.

## §B-0 Algorithmic gain estimate

| | Current (per gasket) | Post-BVH (per gasket) |
|---|---:|---:|
| BVH build cost | — | `O(n log n)` ≈ 0.5-1 s for n = 400 k |
| Ray-cast cost per face | `O(faces) × 3.5 ns/test` = 1.4 ms | `O(log faces) × 30 ns/node` ≈ 1 µs |
| Total ray-cast phase | 400 k × 1.4 ms = 560 s | 400 k × 1 µs = **0.4 s** |
| Grand total per gasket thin-wall check | **540 s** | **~1-2 s** |

Algorithmic speedup on `flag_thin_wall_faces`: **~300-500×**
empirical (theoretical n/log n ≈ 21,000× hides constants — BVH
traversal has branching, AABB tests, cache pressure).

**Other F4 checks unchanged** at this scope. Per-gasket F4 gate
total drops from ~540 s to **~20-50 s** (~10-25× per-mesh wall-
clock), dominated by whichever of the remaining checks is now
slowest (likely `check_self_intersecting` at ~10-30 s, even with
AABB pre-filter, on a 400 k-face mesh).

## §B-1 Projected production iter-1 regen post-§B

Combining with the existing parallel-meshing arc:

| Phase | S2 measured (12.9 min) | Post-§B projected |
|-------|----------------------:|---------------------:|
| Cup pieces × 6 (par_iter) | 54 s (longest single) | **~5-15 s** (smaller meshes, but still bound by thin-wall + others) |
| Plugs × 3 (par_iter) | 35 s | **~5-10 s** |
| Platform | 10 s | ~3-5 s |
| Funnel | 4 s | ~2-3 s |
| **Gaskets × 3 (par_iter)** | **587 s** (longest single) | **~20-50 s** (longest single, dominated by non-thin-wall checks) |
| Write + procedure | 5 s | 5 s |
| **TOTAL projection** | **753 s (12.9 min)** | **~50-90 s (~1-1.5 min)** |

The projection has a wide range because the **next bottleneck** in
the F4 stack (once thin-walls is killed) is uncertain — could be
self-intersect on a 400 k mesh, could be trapped-volume voxelization,
could be long-bridges. S1 implementation will measure and the
recon's S2 phase optionally tackles whichever is now the long pole.

**Upper bound** (if thin-walls drops to 0 ms and all other checks
combined are ~5 s per mesh): per-gasket F4 ≈ 5 s → gasket phase
parallel longest = 5 s → total ≈ **30 s production iter-1 regen**.
That's **60× improvement over S2** (=  **155× over the pre-S1
baseline**). Likely-actual: closer to ~90 s = 8× over S2.

## §B-2 Correctness invariant

The BVH-accelerated path MUST return the same flagged-faces set
+ the same per-face `min_dist` value as the O(face²) reference,
to within floating-point precision.

**Why this is achievable:** Möller-Trumbore + BVH-accelerated ray-
cast both compute `t` for the same ray vs the same triangle. The
BVH only changes WHICH triangles are tested (it prunes early); for
any triangle the BVH descends into, the per-triangle test is
identical. The BVH guarantees no false negatives (every triangle
the ray COULD hit is tested).

**Allowed differences:**

- Tie-breaking on equal `t` values (same `t` for two different
  faces — possible at the gasket's channel-floor MC step). The
  reference takes the LAST face encountered in the inner loop;
  the BVH takes whichever node it descended into last. Tie-break
  doesn't affect the FLAGGED status (only which face's ID we'd
  cite if reporting); resolution at S1 by deterministically
  picking the lower face index on equal `t`.
- FP rounding in the BVH's AABB test (a face could be skipped if
  its AABB-overlap calculation is FP-degenerate at the ray's
  bounding box edge). Resolution: parry3d's `cast_local_ray`
  uses inclusive bounds + epsilon-expand the BVH AABBs; the
  observed false-skip rate should be 0 on production meshes.

**Test plan** (§B-3 below): a regression test running BOTH the
O(face²) reference and the BVH variant on the same fixture mesh,
asserting identical `min_dist` per face (within `1e-9 m`
tolerance) + identical flagged-face set.

## §B-3 Test plan

Three load-bearing tests at S1:

1. **`thin_wall_bvh_matches_reference_o_n_squared`** — for a
   representative gasket-mold mesh (or smaller fixture), run both
   the O(n²) reference and the BVH path. Assert:
   - identical flagged-face count
   - identical flagged-face set (lower-face-index tie-break
     applied to both)
   - per-face `min_dist` within `1e-9 m` (FP tolerance)
2. **`thin_wall_bvh_runtime_under_target`** — release-mode timing
   gate. On a ~50 k-face fixture (small enough for CI), assert
   BVH path < 100 ms; reference path is allowed to take seconds
   (proves the speedup ratio without coupling to specific
   hardware).
3. **`thin_wall_bvh_handles_degenerate_inputs`** — empty mesh,
   single-face mesh, mesh with NaN vertices, mesh with degenerate
   (zero-area) faces. Each must produce the SAME result as the
   reference path (typically: skip + return empty flagged set).

Paired-baseline pattern per
[[feedback-load-bearing-test-fixtures]] — BVH-with vs reference-
without is the natural with/without pair.

## §B-4 Scope — thin-walls ONLY at S1

`flag_thin_wall_faces` is the ONLY function this recon targets at
S1. Other F4 checks remain unchanged.

**Why this scope:** the recon's #1 finding is that thin-walls is
~95% of the cost on the gasket case. Touching other checks at S1
would:
- Multiply the correctness-test scope (every check needs its own
  with/without regression).
- Risk regressing other consumers of `mesh-printability` (cup
  pieces, plugs, platform, funnel, future flange) for marginal
  gain on the gasket case specifically.

**Scope expansion (S2 of this arc, if needed):**
After S1 ships + iter-1 regen measurement, the next bottleneck is
revealed. If it's `check_self_intersecting` at ~30 s per gasket,
we could improve its AABB-pre-filter to a BVH-pre-filter (parry3d
provides `TriMesh::project_local_point` for nearest-pair queries).
If it's `check_trapped_volumes` at ~20 s, that's a separate
voxel-grid optimization. Each becomes its own S-phase or its own
follow-up arc.

## §B-5 cf-cast-cli integration

**None needed.** `mesh-printability::validate_for_printing` is a
workspace primitive consumed by `cf-cast::run_printability_gate`;
no public-API change. The BVH path is internal to the function.

The `mesh-printability` crate gains a `parry3d` direct dep (it
already transitively depends on parry3d via mesh-sdf's TriMesh
distance backend). No new TOML schema; no new workspace dep.

## §B-6 Procedure.rs implications

**None.** The F4 gate's behavior is unchanged from the consumer's
perspective — same checks, same results, faster runtime.

## §B-7 Per-arc memory cap (no regression)

The BVH adds ~64 bytes per face for the spatial structure
(`parry3d::shape::TriMesh` internals: each leaf node holds an
AABB + triangle index). For a 400 k-face gasket: ~25 MB extra
per mesh. Production iter-1 regen peak memory under S2 was
~480 MB. Post-BVH peak: ~480 + 3 × 25 = **~555 MB**. Still
comfortable on a 16 GB workshop machine.

The BVH is reusable across the 8 F4 checks within a single
`validate_for_printing` call (build once, query many) — possible
optimization at S1 if cheap to plumb, otherwise deferred to S2.

## §B-8 Out of scope (this recon)

- Optimizing `check_self_intersecting`, `check_long_bridges`,
  `check_trapped_volumes`, or any non-thin-wall F4 check at S1.
  (S2 of this arc, if S1 measurement reveals one of them as the
  new long pole.)
- Cancellation / SIGINT during F4 gate. parry3d's BVH queries are
  not preemptible; matches the existing F4 gate's non-cancellable
  contract.
- GPU offload of BVH queries (parry3d has CPU-only queries).
  Separate arc if S1 + S2 don't hit a workshop target.
- Adaptive MC over the gasket Y range (the parallel-meshing recon
  §P-15 bookmark mentioned this as an MC-side optimization —
  separate from F4).
- The cf-view + procedure side of the cast pipeline.

## §B-9 Open questions

1. **Tie-break determinism.** Equal-`t` ties between the reference
   path and BVH path. Resolution at S1 by lower-face-index pick
   in BOTH paths. Empirical: are there real ties on production
   gasket meshes? If yes, the reference path's current behavior
   (last-in-loop) is non-deterministic across builds + needs the
   same fix. Probably worth a separate reference-cleanup commit
   before the BVH commit.
2. **BVH build cost amortization.** The BVH is built once per
   `validate_for_printing` call. If thin-walls is the only
   consumer, that's fine. If a future S2 expands BVH usage to
   other checks, building once + sharing the BVH across checks
   saves 0.5-1 s per mesh × 14 meshes = ~10 s in iter-1 regen.
3. **Parry3d version pinning.** Workspace currently uses parry3d
   3.x via mesh-sdf. Mesh-printability's new direct dep should
   match this version to avoid two parry3d builds. Check at S1
   implementation.
4. **Memory budget under 4-core full concurrency.** If a future
   phase-level concurrency arc runs cup-pieces + plugs + gaskets
   all simultaneously, the BVH memory cost stacks: 14 meshes ×
   25 MB BVH ≈ 350 MB on top of mesh data. Still fits in 16 GB
   workshop machine but worth tracking.

## §B-10 Workshop iter-N unblock criteria

This recon is **NOT a workshop-iter-3 blocker.** Iter-3 print is
unblocked by the geometric arcs (seam-gasket-mold S3 +
parallel-meshing S2 already shipped; seam-flange S1-S5 pending).
F4 optimization is post-iter-3 quality-of-life — workshop user
re-runs cf-cast-cli on the production cast.toml in <2 min instead
of 13 min for design iteration.

Soft success criteria (informational, not blocking):

1. Production iter-1 regen wall-clock < 2 min (was 12.9 min post-S2).
2. Per-gasket thin-wall check < 5 s (was 540 s).
3. F4 validation results bit-identical to the O(n²) reference on
   ALL 14 production STLs (cup pieces + plugs + platform + funnel
   + gaskets).
4. No new clippy/-D-warnings or fmt regressions in mesh-printability.

## §B-11 Bail-out priority

If the BVH path produces different flagged-faces than the
reference:

1. **Cargo feature flag** (`mesh-printability/bvh-thin-walls`,
   default enabled). Toggle off via `--no-default-features --
   features <other>` for a serial-reference comparison run.
   First-line debugging.
2. **Per-test backout** — flip the BVH off in test fixtures only
   while keeping it on for the production path. Lets workshop
   user keep the speedup while we debug the test divergence.
3. **Revert the S1 commit** (`git revert <hash>`); reference path
   is restored.
4. **Deeper investigation**: dump the diverging mesh + the
   reference vs BVH `min_dist` per face; reproduce in a parry3d
   issue / fix the BVH-AABB edge case.
5. **Scope reduction**: ship the BVH only for meshes above a
   face-count threshold (e.g. `face_count > 50_000`); small
   meshes use the reference path. Reduces consumer-side risk
   while keeping the gasket-mold win.

## §B-12 Implementation arc

3 phases, ~3-4 wall-clock sessions:

- **S1: BVH in `flag_thin_wall_faces`** (~150-250 LOC). Add
  `parry3d` direct dep to `mesh-printability/Cargo.toml`. Build
  parry3d `TriMesh` once at the function entry; replace the inner
  Möller-Trumbore loop with `TriMesh::cast_local_ray`. Apply
  lower-face-index tie-break consistently (in both reference path
  + BVH path, behind a `#[cfg(test)]` reference fn for the
  regression test). Tests: paired BVH-vs-reference equality on
  fixture + small-mesh runtime gate + degenerate-input gate.
- **S2: cf-cast iter-1 regen + new-bottleneck identification**.
  Run production regen on `~/scans/cast.toml` (parallel + BVH);
  measure per-check timings to identify the new bottleneck.
  Decide: ship as-is (if 1-2 min hit), or scope a follow-up
  S3 for whichever check is now long pole.
- **S2 MEASURED (2026-06-02).** Per-check `MESH_PRINTABILITY_TIMING`
  on the post-narrow-band 0.5 mm production long pole
  (`cast_base_mold_canal_05` layer-2 piece, 530 k faces, welded
  manifold): **`trapped_volumes` = 138.3 s of 139.5 s total
  (99.1%)**; every other check < 0.5 s (`thin_walls` 0.47 s —
  the S1 BVH held; `self_intersecting` 0.38 s; `small_features`
  0.23 s; rest < 0.12 s). The long pole is the §6.3 voxel
  inside-mark, decisively — not self-intersect.
- **S3 SHIPPED (2026-06-02): YZ spatial index on the §6.3
  inside-mark.** Root cause: `mark_inside_voxels` was
  `O(ny·nz·n_faces)` — every `+X` scanline row brute-forced
  Möller-Trumbore against all 530 k faces (~130 B ray-tri tests
  at the 500-voxel/axis cap). Fix: build the parry3d `TriMesh`
  once (reusing [`build_parry_trimesh`] from S1) and per row query
  `qbvh.intersect_aabb` for the triangles overlapping the ray,
  running the **same** f64 `moller_trumbore` only on those
  candidates. The BVH culls; it does **not** do the intersection
  (precision-sensitive inside test stays on f64 Möller-Trumbore per
  §8.4 row 3). **Byte-identity is provable** (any pierced triangle's
  YZ AABB ⊇ its projection ∋ the ray point ⇒ always returned;
  culled faces are exactly the `None` rejects) and **gated**: a
  unit test asserts indexed == reference grid states on 3
  watertight fixtures × 2 voxel sizes, and a throwaway real-mesh
  probe confirmed byte-identical voxel grids on the 530 k-face
  piece. Measured: REFERENCE 136.857 s → **BVH 0.117 s (~1170×)**;
  F4 total ~140 s → a few seconds per big piece. The reference
  loop is preserved as `mark_inside_voxels_reference` (test oracle
  + degenerate-input fallback). ~150 LOC + 1 test in
  `validation.rs`; no Cargo change (parry3d-f64 already a dep from
  S1).
- **S4: Cold-read pass-1 + omnibus PR** alongside the gasket arc
  + parallel-meshing arc + (if shipped) seam-flange arc.

Total scope: ~250-400 LOC across S1-S3.

## §B-13 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rules 1-6, the following memories MUST be read by the S1
implementation session BEFORE touching `mesh-printability`:

- [[project-cf-cast-parallel-meshing-s2]] — empirically-measured
  baseline (12.9 min, 78% rule, per-task contention) that this
  recon's gain projection compounds against.
- [[project-cf-cast-seam-gasket-mold-s3]] — the 400 k-face gasket
  is the reference workload. Don't regress its F4 correctness.
- [[feedback-load-bearing-test-fixtures]] — paired with/without
  test pattern (BVH-with vs reference-without is the load-bearing
  regression gate).
- [[feedback-cf-cast-tests-use-release]] — release-mode tests
  mandatory for the §B-3 #2 runtime gate (debug-mode BVH builds
  are 50-100× slower).
- [[feedback-xtask-grade-strictness]] — clippy / rustdoc
  strictness; any new parry3d-facing code must clear both.

## §B-14 Cross-refs

- `mesh/mesh-printability/src/validation.rs:785`
  `flag_thin_wall_faces` — the function S1 modifies.
- `mesh/mesh-sdf/src/sdf.rs` `TriMeshDistance` — existing
  consumer of parry3d's BVH; reference for the API patterns.
- `docs/CF_CAST_PARALLEL_MESHING_RECON.md` §P-15 — the bookmark
  that triggered this recon. Compounds with the parallel-meshing
  arc; this recon is the spatial-index half.
- `~/scans/cast_iter1_parallel_s2_smoke/` — S2 production regen
  output (12.9 min baseline). Re-time after S1 ships.
- parry3d docs:
  https://docs.rs/parry3d/latest/parry3d/shape/struct.TriMesh.html
- Möller, T., & Trumbore, B. (1997). "Fast, Minimum Storage Ray/
  Triangle Intersection." — the algorithm both paths share inside
  the per-triangle test.

## Status

- Scaffold shipped 2026-05-25.
- **S1 shipped** (`flag_thin_wall_faces` BVH) — see §B-12.
- **S2 measured + S3 shipped 2026-06-02** (this commit):
  `trapped_volumes` pinned as the post-narrow-band long pole
  (138.3 s / 99.1% of F4) and fixed via the YZ spatial-index cull
  on `mark_inside_voxels` — byte-identical, ~1170× on the
  inside-mark phase. Gated by a unit byte-identity test +
  real-mesh probe + clippy/fmt/rustdoc clean. See §B-12 S2/S3.
- Branch: `feat/cf-cast-narrow-band-mc` (narrow-band MC + this
  F4-S3 share the regen-speedup theme).
- Branch: `dev`, no push, no PR until arc close (per
  [[feedback-omnibus-pr-single-branch]]).
- Parallel arcs in flight on same branch:
  - Seam-gasket-mold arc through S3
    ([[project-cf-cast-seam-gasket-mold-s3]], commit `fc9f30e7`).
  - Parallel-meshing arc through S2
    ([[project-cf-cast-parallel-meshing-s2]], commit `d3ff8063`).
  - Seam-flange recon scaffold + cold-read shipped (commits
    `1da08ab7`, `d4ca615f`); S1 implementation pending workshop
    user greenlight.
