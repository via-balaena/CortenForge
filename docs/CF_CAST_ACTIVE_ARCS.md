# cf-cast active arcs (2026-05-25)

Coordination doc for the four in-flight cf-cast arcs on `dev`.
Each arc has its own design recon + memory file; this doc
captures **status + sequencing + cross-arc interactions** in
one place so we don't lose track.

## Branch state

`dev` is **20 commits ahead of origin** (no push, no PR until arc
close per [[feedback-omnibus-pr-single-branch]]).

| # | Commit | Arc | Phase |
|---|---|---|---|
| 1 | `1729796b` | seam-gasket-mold | S1 ship |
| 2 | `6c8bc7f8` | seam-gasket-mold | S1 cold-read |
| 3 | `c7e042c6` | seam-gasket-mold | S2 ship |
| 4 | `eed6d515` | seam-gasket-mold | S2 cold-read |
| 5 | `fc9f30e7` | seam-gasket-mold | S3 ship |
| 6 | `be0542aa` | parallel-meshing | recon scaffold |
| 7 | `22a2c8fd` | parallel-meshing | S1 ship |
| 8 | `d3ff8063` | parallel-meshing | S2 ship |
| 9 | `1da08ab7` | seam-flange | recon scaffold |
| 10 | `d4ca615f` | seam-flange | recon cold-read |
| 11 | `74dcbb8c` | F4 spatial-index | recon scaffold |
| 12 | `e62fdac1` | (this doc) | active-arcs coordination v1 |
| 13 | `d42ddf11` | F4 spatial-index | S1 ship (BVH thin-walls) |
| 14 | `b1bda454` | (this doc) | active-arcs S1 update |
| 15 | `99415ff8` | (this doc) | active-arcs S2 update |
| 16 | `876f20d4` | F4 spatial-index | S3 ship (voxel cap) |
| 17 | `820125c7` | (this doc) | active-arcs S3 update |
| 18 | `2af5247f` | self-intersect-BVH | recon scaffold |
| 19 | `?`         | (this doc) | active-arcs Arc 5 added (this commit) |
| 20 | (pending)  | self-intersect-BVH | recon cold-read |

(S2 of F4 spatial-index was empirical-measurement-only; no code
commit. Counted as the S2 in the memory log but not in the commit
table.)

## Active arcs

### Arc 1 — seam-gasket-mold (gasket-seal arc)

- **Recon:** `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md`
- **State:** S3 shipped. **S4-S8 pending.**
- **Memory:** [[project-cf-cast-seam-gasket-mold-s1]],
  [[project-cf-cast-seam-gasket-mold-s2]],
  [[project-cf-cast-seam-gasket-mold-s3]]
- **Next phases (§G-13):**
  - S4 (~150 LOC + tests) — procedure.rs workshop protocol prose;
    new `## Gasket Casting + Installation Protocol` section.
  - S5 (~200 LOC, OPTIONAL) — dovetail retention escalation;
    triggered only if iter-3 reveals gasket-shift.
  - S6 — workshop physical iter-3 print + pour test
    (workshop-user pause).
  - S7 (~50 LOC) — calibration refinement on S6 data.
  - S8 (~50 LOC) — cold-read close + omnibus PR (joint with the
    other open arcs).
- **Workshop status:** iter-3 print geometrically unblocked (cup
  pieces + plugs + gasket molds all emit cleanly). Awaiting **S4
  prose + S6 physical pour**.

### Arc 2 — parallel-meshing (perf arc, layer 1)

- **Recon:** `docs/CF_CAST_PARALLEL_MESHING_RECON.md`
- **State:** S2 shipped. **S3-S4 pending.**
- **Memory:** [[project-cf-cast-parallel-meshing-s1]],
  [[project-cf-cast-parallel-meshing-s2]]
- **Empirical baseline:** 33.6 min (pre-S1) → **12.9 min (S2,
  measured)**. 2.6× actual. Per-task contention ~9%.
- **Next phases (§P-9):**
  - S3 (~50 LOC, optional) — per-unit log buffer (canonical-order
    stderr flush) + optional `--threads N` CLI flag + possibly
    phase-level concurrency (rayon::scope across the 5 phases).
    **Marginal value post-F4-arc** (see §"Sequencing" below).
  - S4 — cold-read close + omnibus PR.
- **§P-15 follow-up:** the F4 spatial-index arc (now its own arc;
  see Arc 4).

### Arc 3 — seam-flange (clampability arc)

- **Recon:** `docs/CF_CAST_SEAM_FLANGE_RECON.md` (+ cold-read
  pass-1 applied).
- **State:** Recon shipped. **S1-S5 pending.**
- **Memory:** _(none yet — created at S1 ship)_
- **Trigger:** the seam-gasket-mold S2 `GasketSpec.
  workshop_clamp_pressure_pa = 20 kPa` invariant requires even
  flat-clamp pressure to achieve the predicted gasket compression
  (~232 µm > §G-0 FDM-tolerance target of 200 µm). The current
  contoured cup exterior can't be evenly clamped by C-clamps.
- **Decision history**: cross-arc decision-staleness call —
  `docs/archive/CF_CAST_MOLD_WALL_RECON.md` §3.3 Q5 verdict
  ("aesthetic, not blocking") flipped to load-bearing post-
  gasket-arc.
- **Next phases (§F-13):**
  - S1 (~200 LOC) — `FlangeSpec` + `FlangeKind` enum + Ribbon
    field + builder + `compose_piece_solid` SDF union with
    Option (a) per-perimeter-offset flange. Paired-baseline
    tests + flange-vs-gasket lateral non-overlap probe.
  - S2 (~80 LOC) — cf-cast-cli `[flange]` config + derive +
    cross-field validation (`inner_offset_m >
    half_gasket_channel_width`).
  - S3 (~100 LOC) — procedure.rs workshop clamp protocol prose.
  - S4 — cf-cast-cli iter-1 regen on production cast.toml + §R1
    inspector + workshop cf-view smoke.
  - S5 — cold-read + omnibus PR.
- **Workshop dependency:** unblocks the gasket arc's S6 physical
  pour (gasket can't seal without even clamp pressure).

### Arc 4 — F4 spatial-index (perf arc, layer 2)

- **Recon:** `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md`
- **State:** **S1 + S2 + S3 shipped**; S4 pending (cold-read + PR, possibly S4 self_intersect BVH first).
- **Memory:** [[project-cf-cast-f4-spatial-index-s1]],
  [[project-cf-cast-f4-spatial-index-s2]],
  [[project-cf-cast-f4-spatial-index-s3]]
- **S1 ship (`d42ddf11`):** parry3d-f64 BVH for `flag_thin_wall_faces`. cf-cast lib test suite 78.20 s → 17.15 s. parry3d-f64 picked over parry3d after f32 drift caused 1.0 mm test boundary flip.
- **S2 (`2026-05-25`, no code commit):** empirical regen identified `check_trapped_volumes` (NOT self_intersecting) as the asymmetric long pole — 50 s on Positive cup pieces, 0.001 s on Negative (Negative skipped via watertight precondition due to pin-socket boolean open edges). Recon §B-0 projection (1-1.5 min) was 2.5× too optimistic.
- **S3 ship (`876f20d4`):** `MAX_VOXELS_PER_AXIS = 500` cap in `check_trapped_volumes` (workshop 200 mm cup: voxel_size 0.1 → 0.4 mm; grid 180 M → 2.8 M voxels = 65× fewer). Permanent env-var-gated per-check timing infra (`MESH_PRINTABILITY_TIMING=1`). **Production iter-1 regen 2:10.58 = 2.18 min = 15.4× over pre-parallel-meshing 33.6 min baseline.** Memory peak 1.2 GB → 127 MB. cf-cast lib test suite 17.15 s → 5.44 s (3× additional, 14× cumulative).
- **New bottleneck post-S3:** gasket `check_self_intersecting` at ~25 s each. Promoted to Arc 5 (self-intersect-BVH) — see below — rather than expanded inline in this arc, since it (a) touches a different workspace crate (`mesh-repair`), and (b) has its own recon doc + §S-N section numbering distinct from this arc's §B-N.
- **Open S4-S5 picks (workshop-user pick made 2026-05-25):** workshop user picked the sub-1-min path; the self_intersect work is tracked as Arc 5 below. This arc's S4 = cold-read pass-1 (incl. recon §B-2 amendment for f32/f64 + §B-0 amendment for "OTHER checks not negligible") + roll into the omnibus PR alongside Arc 5 ship.

### Arc 5 — self-intersect BVH (perf arc, layer 3)

- **Recon:** `docs/CF_CAST_F4_SELF_INTERSECT_BVH_RECON.md`
  (commit `2af5247f`).
- **State:** Recon shipped. **S1-S3 pending.**
- **Memory:** _(none yet — created at S1 ship)_
- **Trigger:** Arc 4 (F4 spatial-index) S3's regen identified gasket `check_self_intersecting` at ~25 s per gasket as the new long pole post-thin-walls-BVH + post-voxel-cap. mesh-repair's existing AABB-pre-filter + rayon-parallel inner loop is O(n²) worst case on 400 k-face meshes; pair generation is the bottleneck, not the SAT triangle-test.
- **Bottleneck (code-survey-confirmed):**
  `mesh/mesh-repair/src/intersect.rs:149` `detect_self_intersections`'s inner `for j in (i+1)..n` enumerates `n × (n-1) / 2 = 8×10¹⁰` pairs on a 400 k-face gasket. AABB-overlap test is fast (~5 ns) but enumerating all pairs is the dominant cost. Triangle-triangle SAT only runs on the small overlap-pass fraction.
- **Picked architectural fix:** `parry3d_f64::partitioning::Qbvh::traverse_bvtt(qbvh, qbvh, visitor)` — simultaneous traversal of the same Qbvh against itself enumerates AABB-overlapping pairs in O(k + n log n). Downstream SAT triangle-test (`triangles_intersect`) unchanged. parry3d-f64 already a workspace dep (added at Arc 4 S1).
- **Projected gain:** per-gasket `detect_self_intersections` 25 s → ~3 s (~8× wall). Production iter-1 regen 2.18 min → **~0.9 min (~37× over pre-parallel-meshing 33.6 min baseline)**.
- **Scope:** mesh-repair internals only; no public API change. 3 workspace consumers (mesh-printability, mesh-repair-benches, printability-self-intersecting example) auto-inherit the speedup.
- **Next phases (§S-9):**
  - S1 (~150-250 LOC) — Qbvh-based pair enumeration in `detect_self_intersections` + `#[cfg(test)] fn detect_self_intersections_reference` preserving O(n²) impl + 5 paired regression tests (matches-reference / runtime gate / skip-adjacent / max-reported truncation / degenerate inputs).
  - S2 — cf-cast production iter-1 regen + measure new wall-clock (target <1 min).
  - S3 — cold-read pass-1 + omnibus PR (joint with all in-flight arcs).
- **Trigger:** parallel-meshing-S2 empirical measurement showed
  gasket F4 = 78% of total wall-clock (587 s × 3 layers parallel,
  longest single drives the floor). Algorithmic optimization is
  the only remaining lever for >2× more speedup.
- **Bottleneck (code-survey-confirmed):**
  `mesh/mesh-printability/src/validation.rs:785`
  `flag_thin_wall_faces` is pure O(face²) Möller-Trumbore double-
  loop with no spatial index. parry3d's `TriMesh::cast_local_ray`
  is the drop-in replacement; parry3d is already a workspace
  transitive dep via `mesh-sdf::TriMeshDistance`.
- **Projected gain:** ~300-500× algorithmic on the inner loop;
  ~10-25× per-mesh F4 wall-clock; **~8-12× on production iter-1
  regen compounding with parallel-meshing-S2 (12.9 min → ~1-1.5
  min).**
- **Next phases (§B-12):**
  - S1 — ✅ SHIPPED (`d42ddf11`). parry3d-f64 BVH; `flag_thin_wall_faces` swapped; 3 regression tests pass.
  - S2 — production iter-1 regen + new-bottleneck identification on `~/scans/cast.toml`.
  - S3 (conditional) — targeted opt on whatever check is now the
    long pole.
  - S4 — cold-read pass-1 (including recon §B-2 amendment for f32/f64) + omnibus PR.

## Sequencing recommendation

Updated post-F4-S3-ship + Arc-5-recon-scaffold:

1. ✅ **F4-index S1+S2+S3.** Production iter-1 regen 33.6 → 2.18 min.
2. ✅ **Arc 5 self-intersect-BVH recon scaffold** (`2af5247f`).
3. **Arc 5 cold-read pass-1 next** (workshop user just asked for it).
4. **Arc 5 S1-S2-S3 ship** — sub-1-min iter-1 regen projected.
5. **Seam-flange S1-S5.** Each phase regen <2 min thanks to
   compounded perf wins. Workshop S6 physical pour gate clear.
6. **F4-index S4 (cold-read + recon amendments).** Joins the
   omnibus PR — no new code, just doc fixes for recon §B-0
   ("OTHER checks not negligible") + §B-2 (f32/f64 distinction).
7. **Parallel-meshing S3 (or skip).** Log buffer + `--threads`
   flag are cosmetic. **Skip** unless omnibus PR cold-read
   surfaces a log-readability complaint.

**Compounding regen progression (multiple steps measured 2026-05-25):**

| Order | After step | Iter-1 regen | Notes |
|-------|------------|-------------:|-------|
| Pre-S1-of-parallel-meshing | baseline | 33.6 min | |
| Parallel-meshing S2 ship | measured | 12.9 min (2.6×) | gasket F4 dominates 78% |
| F4-index S1 ship (`d42ddf11`) | (projection only — see next row) | | |
| **F4-index S2 production regen** | **measured** | **4.14 min (8.1×)** | recon §B-0 was 2.5× optimistic |
| **F4-index S3 ship (`876f20d4`)** | **measured** | **2.18 min (15.4×)** | voxel-cap on trapped_volumes |
| Arc 5 self-intersect-BVH S1-S2 (pending) | projected | **~0.9 min (~37×)** | Qbvh self-overlap pair enum |
| Seam-flange S1-S5 (pending) | each phase regen | ~0.9-1.5 min | iter-3 unblock |
| Final state at PR time | all S-phases shipped | **~0.9-1.5 min (22-37× over baseline)** | |

## Cross-arc correctness boundaries

What each S1 ship can't break in the others:

- **F4-index S1**: must produce **bit-identical** flagged-faces
  set + per-face `min_dist` for ALL 14 production STLs (cup
  pieces, plugs, platform, funnel, gaskets), to within FP
  precision. The seam-gasket-mold arc's 225 cf-cast lib tests +
  the seam-flange S1's new tests + the parallel-meshing's tests
  ALL run through the F4 gate. Regression here breaks every other
  arc's correctness gate.
- **Seam-flange S1**: must not change the gasket-mold STLs (the
  flange is a cup-piece-only feature). Must not change the cup-
  piece SDF for `FlangeKind::None` (preserves backward compat
  with non-flange test fixtures). The 225 lib tests' multi-layer
  fixtures use `Ribbon::new` (`gasket: GasketKind::None`,
  `flange: FlangeKind::None`) → unchanged geometry; tests stay
  bit-identical.
- **Parallel-meshing S3** (if shipped): must not change cf-cast
  output STL bytes vs S2; only changes log output ordering +
  optional CLI flag surface.

## Omnibus PR plan (when all arcs close)

Per [[feedback-omnibus-pr-single-branch]]: single PR to `main`
covering all four arcs. Expected scope at PR time:

- ~1500-2500 LOC total (gasket arc was +538 LOC at S3; flange
  arc ~380 LOC across S1-S3; F4-index ~250-400 LOC; parallel-
  meshing ~170 LOC).
- ~25-35 commits total (currently 11, plus pending S-phases).
- Production iter-1 regen <2 min (workshop-acceptable for
  design iteration).
- Workshop iter-3 physical pour gate clear.
- All recons + cold-reads in `docs/` (no archive yet — that's
  workshop-success-driven cleanup).

PR title draft: `cf-cast: silicone-seal arc + clampable seam +
parallel meshing + F4 BVH (iter-3 unblock)`.

## Open decisions pending

- **Parallel-meshing S3 ship-vs-skip.** Recommendation: skip
  unless cold-read surfaces a log-readability complaint at PR
  time. Defer pick to workshop-user during omnibus-PR cold-read.
- **Gasket-mold `cross_section` TOML field** (from gasket recon
  §G-7). Currently not exposed; deferred to "S4 cold-read or
  memory follow-up" per
  [[project-cf-cast-seam-gasket-mold-s3]].
- **Seam-flange edge chamfering** (§F-10 #1). Empirical at S6
  iter-3 if workshop user reports skin-grippy edges. Defer.
- **F4-index S3 scope** (§B-12). Conditional on what's the new
  bottleneck post-S1 measurement. Decision deferred to S2 of that
  arc.

## Decision log

Major architectural decisions across all arcs (chronological,
most recent first):

- **2026-05-25 (post-F4-S3)**: spin self-intersect-BVH work out
  into its own **Arc 5** rather than expanding F4-index S4. Two
  reasons: (a) different workspace crate (`mesh-repair` vs
  `mesh-printability`), (b) self-contained recon doc with §S-N
  section numbering distinct from F4's §B-N. F4-index S4
  collapses to cold-read pass-1 + recon amendments (no code).
- **2026-05-25 (F4-index S2 measurement)**: ship F4-index **S3**
  rather than ship-as-is at 4.14 min. Workshop developer-experience
  delta between 4 min and <2 min is meaningful (coffee-fetch vs
  "interrupt + come back"); the asymmetric Positive-cup F4 (50s
  vs Negative 0s) is a concentrated bottleneck likely tractable
  via the same BVH-pre-filter pattern as S1.
- **2026-05-25**: **parry3d-f64 (NOT parry3d)** picked for F4
  spatial-index S1 (commit `d42ddf11`). Discovered empirically
  during S1 testing — parry3d (f32 sibling) drifts Möller-
  Trumbore toi by ~1e-7 mm which crosses the
  `min_wall_thickness` boundary at the 1.0 mm slab fixture.
  Workspace adds 2 parry3d crates (mesh-sdf stays on f32; mesh-
  printability uses f64).
- **2026-05-25**: **Hybrid contoured-cup + flat-flange** picked
  for seam-flange (recon §F-13). Cross-arc decision-staleness
  call vs the archived mold-wall recon's "Option A contour
  only" decision. Preserves ~10× PLA savings while adding
  clamp-grip surface where the gasket actually seals.
- **2026-05-25**: **BVH via parry3d** picked for F4 spatial-
  index (recon §B-2). parry3d is already a workspace
  transitive dep; battle-tested via mesh-sdf.
- **2026-05-25**: **`rayon::par_iter` within each
  mesh_and_gate_v2_* phase** picked for parallel-meshing
  (recons §P-3 + §P-9). canonical-first-error via
  `collect::<Result<Vec<_>, _>>`. Phase-level concurrency
  (rayon::scope) deferred — marginal value confirmed
  empirically.
- **2026-05-25**: **Trapezoidal cross-section + parameterized
  material** picked for gasket mold (S2 workshop-user input).
  Cross-section + material exposed in cf-cast-cli TOML at S3.
- **2026-05-25**: **Per-layer gasket mold via projection-to-
  seam-plane SDF adapter** picked for gasket recon (§G-1 +
  §G-6). Mirrors `RibbonHalfspaceSdf` pattern.

## Cross-refs

- Per-arc recons: `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md`,
  `docs/CF_CAST_PARALLEL_MESHING_RECON.md`,
  `docs/CF_CAST_SEAM_FLANGE_RECON.md`,
  `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md`.
- Memory entries for shipped phases: see "Memory" entries per
  arc above.
- Archived predecessor: `docs/archive/CF_CAST_MOLD_WALL_RECON.md`
  (Option A contour-only — Q5 verdict amended by seam-flange
  recon).
- Update cadence: edit this doc at every ship-or-decision event
  across any of the four arcs. Status table + decision log are
  the most-frequent-update sections.
