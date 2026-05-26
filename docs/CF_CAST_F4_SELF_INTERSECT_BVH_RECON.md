# cf-cast F4 self-intersect BVH recon (2026-05-25)

**Status:** scaffold. Follow-up to the F4 spatial-index arc
([[project-cf-cast-f4-spatial-index-s3]] commit `876f20d4`),
empirically motivated by the S3 production iter-1 regen breakdown:

| Phase | Post-S3 wall | F4 cost dominator |
|-------|-------------:|--------------------|
| Cup pieces (6 parallel, longest single) | ~9 s | MC dominates |
| Plugs (3 parallel, longest single)      | ~4 s | MC dominates |
| Platform                                | 1.1 s | trapped_volumes (capped) |
| Funnel                                  | 3.3 s | (small mesh, no big lever) |
| **Gasket × 3 (longest single)**         | **65 s** | **30.6 s F4 = ~5 s trapped + ~25 s self_intersect + <0.5 s other** |
| Total                                   | ~109 s = **2.18 min wall** | |

The gasket phase now consumes ~60% of total wall-clock, and
**`mesh_repair::detect_self_intersections` at ~25 s per gasket is
the dominant F4 check** within that phase. trapped_volumes was
capped at S3 (~5 s); other checks are <0.5 s thanks to the S1 BVH
thin-walls swap. self_intersect is the last major lever.

**Picked architectural fix:** replace the explicit O(face²) pair-
enumeration loop in `mesh_repair::detect_self_intersections` with a
parry3d-f64 `Qbvh::traverse_bvtt(qbvh, qbvh)` self-overlap
enumeration. The per-pair SAT-based triangle-triangle test
(`triangles_intersect`) stays bit-identical; only the **pair
generation** path changes from O(n²) AABB-overlap scan to O(n log n)
BVH-accelerated overlap-pair enumeration. Same algorithmic gain
pattern as the F4 thin-walls S1.

## Why this recon

Empirical regen data ([[project-cf-cast-f4-spatial-index-s3]]):

- Gasket `check_self_intersecting`: 25-26 s per gasket on
  ~400 k-face meshes. mesh-repair's existing AABB-pre-filter +
  rayon-parallel outer loop is **not enough** at this scale — the
  inner pairwise loop is `O(n²)` worst case (`for j in (i+1)..n`).
- Cup-piece + plug self_intersecting: <0.05 s each on ~10-17 k-face
  meshes. The O(n²) cost only becomes meaningful at gasket scale.
- Sub-1-min iter-1 regen requires this check on gasket to drop to
  ~3 s.

**Workshop iter-N developer experience target:** sub-1-min iter-1
regen would put cf-cast-cli in the range of "design-iteration
acceptable" — workshop user can re-run after every spec tweak
without a substantial pause.

**Projected effect on production regen:**

| Phase | Post-S3 wall | Post-§S projection |
|-------|-------------:|--------------------:|
| Cup pieces | ~9 s | ~9 s (unchanged) |
| Plugs | ~4 s | ~4 s (unchanged) |
| Platform | 1.1 s | 1.1 s |
| Funnel | 3.3 s | 3.3 s |
| **Gasket × 3 (longest)** | **65 s** | **~37 s** (MC 34.5 + F4 ~3 s) |
| Total wall | **109 s (2.18 min)** | **~54 s (~0.9 min)** |

Cumulative speedup over pre-parallel-meshing 33.6 min baseline:
**~37× post-§S** (was 15.4× at S3, 8.1× at F4-S1, 2.6× at
parallel-meshing-S2).

## §S-0 Bottleneck attribution (code-survey-confirmed)

`mesh/mesh-repair/src/intersect.rs:149` `detect_self_intersections`:

```rust
let intersecting_pairs: Vec<(u32, u32)> = (0..face_count)
    .into_par_iter()
    .flat_map(|i| {
        let mut local_pairs = Vec::new();
        for j in (i + 1)..face_count {                  // O(n) inner
            if !aabbs[i].overlaps(&aabbs[j]) { continue; }  // O(1) pair test
            if let Some(ref adj) = adjacency && adj[i].contains(&(j as u32)) {
                continue;
            }
            if triangles_intersect(&triangles[i], &triangles[j], params.epsilon) {
                // ... record pair ...
            }
        }
        local_pairs
    })
    .collect();
```

**Cost on 400 k-face gasket:**

- Outer `into_par_iter()` over 400 k indices, parallel across N
  cores.
- Inner loop: `(i+1)..400_000`. Total pair-considerations =
  `n × (n-1) / 2 ≈ 8 × 10¹⁰`.
- Each pair-consideration does:
  1. `aabbs[i].overlaps(&aabbs[j])` — 6 inequality tests, ~3 ns
     per call on modern CPU (well-predicted branches, cache-
     friendly Vec<Aabb> access).
  2. Skip-adjacent check (HashSet lookup) on overlap-pass — ~30 ns.
  3. SAT-based `triangles_intersect` on overlap-pass — ~50 ns.

The AABB-overlap check is fast individually, but **enumerating
all `n²/2` pairs is the dominant cost** at gasket scale.
`8×10¹⁰ pairs × 3 ns/pair = 240 s single-threaded`; with rayon's
parallelism on the workshop machine (~11 logical cores, including
hyperthreading), wall time settles around 22-25 s, which matches
the observed 25-26 s well. The earlier "11× linear" scaling
estimate was conservative — AABB-overlap is compute-bound (not
memory-bandwidth-bound), so SMT/hyperthreading contributes
effective parallelism beyond the physical-core count.

The triangle-triangle intersection test itself (`triangles_intersect`,
SAT with 13 axes) is fine — it only runs on the small fraction of
AABB-overlapping pairs (~thousands on a clean gasket mesh).

**The pair generation is the issue, not the per-pair test.**

## §S-1 Picked architectural fix

`parry3d_f64::partitioning::Qbvh::traverse_bvtt(qbvh1, qbvh2,
visitor)` — simultaneous traversal of two Qbvhs. For self-overlap:
**traverse the same Qbvh against itself**. The visitor's `visit`
method is called with leaf-pairs whose AABBs overlap. parry3d's
SIMD-accelerated 4-wide BVH gives `O(k + n log n)` where `k` is the
number of overlapping pairs.

**Pseudocode** (illustrative — `OverlapPairCollector` is the
load-bearing implementation work; see "Visitor implementation
cost" below):

```rust
fn detect_self_intersections(mesh: &IndexedMesh, params: &IntersectionParams)
    -> SelfIntersectionResult
{
    // 1. Precompute Triangle + Aabb (unchanged).
    let triangles: Vec<Triangle> = mesh.triangles().collect();
    let aabbs: Vec<parry3d_f64::bounding_volume::Aabb> = ...;

    // 2. Build adjacency (unchanged).
    let adjacency = if params.skip_adjacent { Some(build_face_adjacency(...)) } else { None };

    // 3. Build a Qbvh keyed on triangle index.
    let mut qbvh = parry3d_f64::partitioning::Qbvh::new();
    let leaves: Vec<(u32, parry3d_f64::bounding_volume::Aabb)> = aabbs.iter()
        .enumerate()
        .map(|(i, a)| (i as u32, *a))
        .collect();
    qbvh.clear_and_rebuild(leaves.into_iter(), 0.0);

    // 4. Enumerate overlapping pairs via a custom visitor.
    //    `OverlapPairCollector` MUST enforce canonical pair
    //    ordering (`i < j`) and skip self-pairs (`i == j`):
    //    parry's `traverse_bvtt(self, self)` emits both `(i, j)`
    //    AND `(j, i)` for every non-trivial overlap, PLUS `(i, i)`
    //    for every leaf (since each leaf's AABB trivially
    //    overlaps itself). Without the canonical-pair filter the
    //    BVH path would emit 2× pair count of the reference path.
    let mut candidate_pairs: Vec<(u32, u32)> = Vec::new();
    let mut visitor = OverlapPairCollector::new(&mut candidate_pairs);
    qbvh.traverse_bvtt(&qbvh, &mut visitor);
    // After traversal: `candidate_pairs` contains exactly the
    // `(i, j)` pairs with `i < j` whose AABBs (epsilon-expanded)
    // overlap — same set the reference O(n²) loop would discover.

    // 5. For each candidate pair, run SAT-based triangles_intersect
    //    (unchanged from reference). Adjacency-skip is applied as
    //    a post-filter here rather than inside the visitor — picked
    //    over visitor-internal filtering per §S-8 #1 below
    //    (candidate-list scale is small; post-filter is simpler).
    let intersecting_pairs: Vec<(u32, u32)> = candidate_pairs
        .par_iter()
        .filter(|&&(i, j)| {
            let iu = i as usize; let ju = j as usize;
            if let Some(ref adj) = adjacency && adj[iu].contains(&j) { return false; }
            triangles_intersect(&triangles[iu], &triangles[ju], params.epsilon)
        })
        .copied()
        .collect();

    // 6. Truncate to max_reported + populate SelfIntersectionResult (unchanged).
    ...
}
```

**Visitor implementation cost.** `OverlapPairCollector` must
implement
`parry3d_f64::partitioning::SimdSimultaneousVisitor<u32, u32,
SimdAabb>` — a SIMD-aware trait whose `visit` method is called
with 4-wide leaf batches (parry3d's Qbvh stores 4 children per
node for SIMD AABB tests). The implementation is ~30-60 LOC of
trait boilerplate handling:
- Returning `SimdSimultaneousVisitStatus::MaybeContinue(mask)` for
  internal nodes (parry walks deeper where the mask is set).
- For leaf-leaf interactions, extracting the 4 left-leaf indices
  + 4 right-leaf indices, looping over the 4×4 = 16 combinations,
  and pushing canonical `(i, j)` pairs into `candidate_pairs`
  when `i < j` AND the mask bit is set for that (left, right)
  combination.
- Returning `SimdSimultaneousVisitStatus::Continue` for non-leaf
  pairings.

Not trivial to write but a one-time cost — pattern is reusable
for any future Qbvh-pair-enumeration consumer.

**Key correctness invariant:** the canonical-pair candidate set
returned by the visitor is the EXACT set of AABB-overlapping
`(i, j)` pairs with `i < j` (plus the epsilon-expansion baked into
the precomputed AABBs). Same set the O(n²) reference loop would
discover. The downstream `triangles_intersect` filter is
unchanged, so the final reported intersecting-pair set is
**bit-identical** to the reference path.

**No change to the public API.** `SelfIntersectionResult` shape,
`IntersectionParams` field semantics, `truncated` flag behavior,
adjacency-skip semantics — all preserved.

## §S-2 Algorithmic gain estimate

| Aspect | Current O(n²) | BVH O(n log n) |
|--------|--------------:|---------------:|
| Pair-considerations on 400 k-face gasket | 8×10¹⁰ | ~7×10⁶ |
| Pair-enumeration cost (interleaved AABB-test + SAT filter; rayon ×11) | ~25 s total | (replaced by BVH below) |
| BVH build cost (one-time per mesh)     | — | ~0.5-1 s |
| BVH traversal — overlap pair enumeration | — | ~1-2 s |
| Candidate filter via SAT (rayon-parallel, ~thousands of pairs) | (part of "~25 s" above) | ~0.5 s |
| **Total `detect_self_intersections`**   | **~25 s** | **~3 s** |

Reading the table: in the current O(n²) path, the inner loop
**interleaves** AABB-overlap pre-filter + SAT triangle test
across all `n²/2` pair-considerations — they're NOT separate
phases. The ~25 s wall is the whole loop's cost, dominated by
the AABB pre-filter (since the SAT branch only fires on the small
overlap-pass fraction). The BVH path **separates** those into
two distinct steps: BVH traversal emits candidate pairs (no SAT
called), then a downstream `par_iter().filter()` runs the SAT
test on candidates only.

**Per-gasket speedup: ~8× wall.** Per-mesh speedup compounds with
F4-S1's thin-walls BVH + F4-S3's voxel cap to drop gasket F4 from
64 s (post-S1) → 30.6 s (post-S3) → **~9 s (post-§S)**.

**Production iter-1 regen projection: 2.18 min → ~0.9 min (~37× over
pre-parallel-meshing 33.6 min baseline).**

## §S-3 Rejected alternatives

- **Custom-roll BVH in mesh-repair** → REJECTED. parry3d-f64 is
  already a workspace dep (added at F4-S1 for thin-walls). Rolling
  a second BVH implementation is pure wasted scope.
- **rstar (R-tree)** → REJECTED. Better for high-aspect-ratio
  spatial distributions but gasket triangle AABBs are roughly
  isotropic; Qbvh wins.
- **k-d tree of triangle centroids + radius queries** → REJECTED.
  Centroid-radius queries over-report on long-thin-triangle
  geometries (gasket channel walls are exactly this shape).
- **Use parry3d (f32 sibling) instead of parry3d-f64** → DEFER.
  parry3d-f64 is already in workspace from F4-S1. The
  pair-enumeration in this recon doesn't depend on FP precision
  the way thin-walls did (we only need AABB-overlap pairs as
  candidates; the SAT test is f64 regardless), so f32 might be
  acceptable here. BUT — keeping the dep set consistent with the
  rest of mesh-printability's parry3d-f64 path avoids
  double-parry-builds. Stay on parry3d-f64.
- **GPU offload via wgpu** → REJECTED at this scope. Same
  reasoning as the parallel-meshing recon §P-15: CPU BVH gets us
  to <1-min regen; GPU is diminishing returns.
- **Skip self_intersect check entirely for trusted geometry**
  → REJECTED. Workshop-user value is in the gate firing on
  unexpected boolean-artifact intersections; can't disable
  defaults.

## §S-4 Correctness invariant + test plan

**Invariant:** the BVH path returns the SAME set of
`intersecting_pairs` (mod canonical ordering — `(i, j)` with `i < j`)
as the reference O(n²) path, on the same input mesh + same
`IntersectionParams`.

Three load-bearing tests at S1:

1. **`self_intersect_bvh_matches_reference_o_n_squared`** — on a
   fixture with known self-intersections (e.g., the existing
   `examples/mesh/printability-self-intersecting` test mesh +
   the `make_box_with_pin_intersection` cf-cast-style fixture),
   run both `detect_self_intersections` (BVH) and
   `detect_self_intersections_reference` (O(n²), preserved as
   `#[cfg(test)] fn` like S1's thin-walls reference). Assert:
   - Identical `intersection_count`.
   - Identical `intersecting_pairs` set, compared as
     `HashSet<(u32, u32)>` to abstract over both paths'
     ordering quirks — reference uses rayon (non-deterministic
     across runs); BVH path may be deterministic (single-
     threaded `traverse_bvtt`) but ordering depends on Qbvh
     internal SIMD-leaf traversal order. HashSet equality is
     the right cross-path invariant.
   - Same `truncated` flag.
   - Same `has_intersections` boolean.
2. **`self_intersect_bvh_runtime_under_target`** — generate a
   procedural fixture with many AABBs but few intersections (e.g.,
   a 5 k-face grid). Release-mode timing gate: BVH path > 5×
   faster than reference. Conservative — production gasket scale
   hits ~8×.
3. **`self_intersect_bvh_respects_skip_adjacent`** — verify the
   adjacency-skip semantic is preserved (a mesh of two tris
   sharing an edge should NOT report an intersection under
   `skip_adjacent = true`).

Plus:

4. **`self_intersect_bvh_respects_max_reported`** — truncation
   semantics preserved (set `max_reported = 5` on a fixture with
   10 known intersections; assert exactly 5 reported +
   `truncated == true`).
5. **`self_intersect_bvh_handles_degenerate_inputs`** — empty
   mesh / single-face / zero-area face / NaN-vertex. Both BVH +
   reference gracefully return empty result (no panics).

Paired BVH-vs-reference pattern per
[[feedback-load-bearing-test-fixtures]] — same load-bearing
guarantee as F4-S1's thin-walls test.

## §S-5 Scope — mesh-repair internals only

- Modify `detect_self_intersections` body (~80-100 LOC change:
  replace the `(0..face_count).into_par_iter().flat_map(|i| { for
  j in (i+1)..face_count { ... } })` with the BVH-based pair
  enumeration).
- Add `parry3d-f64 = { workspace = true }` to
  `mesh/mesh-repair/Cargo.toml`.
- Add `#[cfg(test)] fn detect_self_intersections_reference` (~70
  LOC of preserved O(n²) impl for the regression test).
- Add 5 new tests (~200 LOC).
- No public API change.
- No workspace-dep churn (parry3d-f64 already added at F4-S1).

**Workspace consumers preserved:**

- `mesh-printability::check_self_intersecting`
  (`validation.rs:2129`) — calls
  `mesh_repair::intersect::detect_self_intersections(mesh,
  &IntersectionParams::default())`. Unchanged signature, same
  result struct.
- `mesh-repair-benches/benches/repair_benchmarks.rs` — benchmark
  suite. Same signature. Will measure the speedup automatically.
- `examples/mesh/printability-self-intersecting/src/main.rs`
  — workspace example. Same signature.

## §S-6 Workshop iter-3 unblock criteria

S4 of the F4-index arc is **NOT a workshop-iter-3 blocker** —
iter-3 print is already unblocked by the geometric arcs (gasket-
mold S3 + parallel-meshing S2). This recon optimizes design-
iteration wall-clock (12.9 min → 2.18 min → ~0.9 min projected).

Soft success criteria (informational, not blocking):

1. Production iter-1 regen wall-clock < 1 min (was 2.18 min at
   F4-S3; 33.6 min pre-parallel-meshing).
2. Per-gasket self_intersecting < 5 s (was ~25 s at F4-S3).
3. `mesh_repair::detect_self_intersections` BVH path returns
   bit-equivalent results to the O(n²) reference on ALL workspace
   test fixtures (mesh-repair tests + the new regression test +
   the mesh-printability tests + the cf-cast lib tests).
4. No new clippy/-D-warnings or fmt regressions.

## §S-7 Bail-out priority

If the BVH path produces different intersecting-pair sets than
the reference:

1. **Cargo feature flag** `mesh-repair/bvh-self-intersect`
   (default on). Toggle off with `--no-default-features --
   features <other>` for serial-reference fallback. First-line
   debugging.
2. **Per-test backout** — flip BVH off in test fixtures only
   while keeping production path BVH. Lets workshop user keep
   the speedup while we debug.
3. **Per-mesh face-count threshold** — skip BVH for small meshes
   (e.g. `face_count < 10_000`); use reference for cup pieces +
   plugs, BVH for gaskets only. Scope-reduction fallback.
4. **Revert the S1 commit** — per-phase commit on dev; `git revert`
   restores reference behavior.

**Specific concern: parry3d-f64 Qbvh leaf-overlap pair-enumeration
correctness on coincident-AABB inputs.** If two triangle AABBs
are EXACTLY equal (degenerate fixture), parry's traversal might
emit `(i, j)` AND `(j, i)`, OR omit one direction. S1 test must
include this fixture case + canonicalize pair direction
(`(min(i,j), max(i,j))`) in both paths.

## §S-8 Open questions

1. **Adjacency-skip placement: PICKED post-filter (decided at
   cold-read pass-1).** Adjacency check runs on the candidate-
   pair list returned by the visitor, not inside the visitor.
   Rationale: candidate-list scale is ~thousands of pairs on
   real meshes — even O(k) post-filter is fast. Visitor stays
   pure spatial (just emits AABB-overlap pairs), which is
   simpler to implement + test in isolation. The cost of
   skipping pairs slightly later is negligible.
2. **Visitor parallelization.** parry3d's `traverse_bvtt` is
   single-threaded. Could the visitor itself be parallelized
   (multiple subtree traversals in parallel)? Probably not worth
   it — the traversal is fast enough that the per-pair SAT
   check (parallelizable via rayon as today) is the dominant
   cost, not the traversal.
3. **AABB epsilon expansion.** mesh-repair currently expands each
   AABB by `params.epsilon` to catch near-coincident intersections.
   parry3d's Qbvh accepts pre-expanded AABBs; same expansion
   carries through transparently. Verify at S1.

## §S-9 Implementation arc

3 phases, ~2-3 wall-clock sessions:

- **S1: BVH in `detect_self_intersections`** (~150-250 LOC). Add
  `parry3d-f64` direct dep to mesh-repair/Cargo.toml. Replace
  O(n²) pair-enumeration with Qbvh traversal. Add
  `#[cfg(test)] fn detect_self_intersections_reference` preserving
  the O(n²) impl. Add 5 paired regression tests. cf-cast lib
  test suite + mesh-repair lib test suite + mesh-printability
  test suite all pass with bit-equivalent results.
- **S2: cf-cast production iter-1 regen + verify wall-clock**.
  Run `cargo run --release ... -- cast.toml --output-dir
  cast_iter1_post_self_intersect_smoke`. Confirm sub-1-min wall.
  Measure per-mesh self_intersect timing via the env-var-gated
  `MESH_PRINTABILITY_TIMING=1` infra from F4-S3.
- **S3: cold-read pass-1 + omnibus PR** (joint with the gasket
  arc + parallel-meshing arc + seam-flange arc — or this arc
  alone if the others aren't ready).

Total: ~250-400 LOC across S1-S2.

## §S-10 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rules 1-6, the following memories MUST be read by the S1
implementation session BEFORE touching mesh-repair:

- [[project-cf-cast-f4-spatial-index-s1]] — predecessor BVH ship
  on mesh-printability. parry3d-f64 dep was added here. Reuse
  the pattern (workspace-dep + `build_parry_trimesh`-style
  helper).
- [[project-cf-cast-f4-spatial-index-s2]] — empirical bottleneck-
  attribution method (env-var-gated `[F4-time]` per-check timing).
  S2 of this arc reuses this infra to verify wall-clock
  improvement.
- [[project-cf-cast-f4-spatial-index-s3]] — voxel-axis cap that
  makes `check_trapped_volumes` NOT the bottleneck anymore. This
  recon attacks the next bottleneck in line.
- [[feedback-load-bearing-test-fixtures]] — paired BVH-vs-
  reference test pattern. Same load-bearing guarantee as
  thin-walls S1.
- [[feedback-cf-cast-tests-use-release]] — release-mode tests
  mandatory for the §S-4 #2 runtime gate.
- [[feedback-xtask-grade-strictness]] — clippy / rustdoc
  strictness; any new parry3d-f64-facing code must clear both.

## §S-11 Cross-refs

- `mesh/mesh-repair/src/intersect.rs:149` `detect_self_intersections`
  — the function S1 modifies.
- `mesh/mesh-repair/src/intersect.rs:298` `triangles_intersect`
  — the SAT-based per-pair test (UNCHANGED; only pair generation
  changes).
- `mesh/mesh-repair/src/intersect.rs:266` `build_face_adjacency`
  — UNCHANGED; same edge-sharing detection.
- `mesh/mesh-printability/src/validation.rs:2129`
  `check_self_intersecting` — main consumer; signature unchanged.
- `parry3d-f64` docs:
  https://docs.rs/parry3d-f64/latest/parry3d_f64/partitioning/struct.Qbvh.html
- `~/scans/cast_iter1_post_f4_s3_smoke/` — F4-S3 baseline regen
  (2.18 min). Re-time at S2 of this arc.
- `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` §P-15 — original
  bookmark pointing at this work. This recon is the
  "self-intersect spatial-index" half of the F4 optimization
  story.

## §S-12 Follow-up arc bookmarks (if S1+S2 ship)

If production iter-1 regen hits <1 min, the **next bottleneck is
gasket MC compose+mesh at 34.5 s**. Possible follow-up arcs:

- **Adaptive MC cell sizing** (parry-shaped) — sample at coarse
  cell elsewhere + fine cell near the channel. ~100-200 LOC in
  mesh-offset.
- **Parallel gasket MC** — currently gasket compose+MC is per-task
  via rayon (parallel-meshing S1); within a task the MC pass is
  single-threaded. wgpu offload of MC sampling is the canonical
  next layer.

Out of scope here. Bookmark for after S2 measurement confirms
sub-1-min.

## Status (open)

- Scaffold shipped 2026-05-25 (this commit).
- Awaiting workshop user approval to proceed to S1.
- No code change yet; this is design doc only.
- Branch: `dev`, no push, no PR until arc close (per
  [[feedback-omnibus-pr-single-branch]]).
- Parallel arcs in flight on same branch:
  - Seam-gasket-mold arc through S3 (`fc9f30e7`).
  - Parallel-meshing arc through S2 (`d3ff8063`).
  - Seam-flange scaffold + cold-read shipped (`d4ca615f`).
  - F4 spatial-index arc through S3 (`876f20d4`); S4 is this
    recon's implementation; S5 cold-read + omnibus PR (joint
    with other arcs).
