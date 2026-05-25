# cf-cast active arcs (2026-05-25)

Coordination doc for the four in-flight cf-cast arcs on `dev`.
Each arc has its own design recon + memory file; this doc
captures **status + sequencing + cross-arc interactions** in
one place so we don't lose track.

## Branch state

`dev` is **N commits ahead of origin** (no push, no PR until arc
close per [[feedback-omnibus-pr-single-branch]]).

| # | Commit | Arc | Phase |
|---|---|---|---|
| 1 | `1729796b` | seam-gasket-mold | S1 ship |
| 2 | `6c8bc7f8` | seam-gasket-mold | S1 cold-read |
| 3 | `c7e042c6` | seam-gasket-mold | S2 ship |
| 4 | `eed6d515` | seam-gasket-mold | S2 cold-read |
| 5 | `fc9f30e7` | seam-gasket-mold | **S3 ship (current frontier)** |
| 6 | `be0542aa` | parallel-meshing | recon scaffold |
| 7 | `22a2c8fd` | parallel-meshing | S1 ship |
| 8 | `d3ff8063` | parallel-meshing | **S2 ship** |
| 9 | `1da08ab7` | seam-flange | recon scaffold |
| 10 | `d4ca615f` | seam-flange | recon cold-read |
| 11 | `74dcbb8c` | F4 spatial-index | recon scaffold |

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
- **State:** Recon shipped. **S1-S4 pending.**
- **Memory:** _(none yet — created at S1 ship)_
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
  - S1 (~150-250 LOC) — `parry3d` direct dep on mesh-
    printability; BVH-accelerated `flag_thin_wall_faces`; paired
    BVH-vs-reference regression test.
  - S2 — production iter-1 regen + new-bottleneck identification.
  - S3 (conditional) — targeted opt on whatever check is now the
    long pole.
  - S4 — cold-read + omnibus PR.

## Sequencing recommendation

The three pending arcs (flange / F4-index / parallel-meshing-S3)
are loosely independent at the code level, but **iter-1 regen
measurements depend on the ship order**. Recommended order:

1. **F4 spatial-index S1 first.** Single-function targeted change;
   biggest single win on a primitive that every other cf-cast
   STL flows through. Once shipped, iter-1 regen drops to ~2 min
   → the seam-flange S4 iter-1 regen step (and any future
   workshop-iteration cycle) becomes a routine ~2-min action
   rather than a 13-min pause. **Improves the developer
   experience for every other arc.**
2. **Seam-flange S1-S5 next.** Builds on the fast regen + adds
   the workshop-clampability that unblocks the gasket arc's S6
   physical pour. Each S1-S5 phase regen is fast enough to
   validate inline.
3. **Parallel-meshing S3 last (or skip).** Log buffer + `--threads`
   flag are cosmetic. Phase-level concurrency would have saved
   30-60 s pre-F4-arc; post-F4-arc the phases all run in <30 s
   each so the savings collapse to ~5-10 s. **Skip S3 entirely
   unless the omnibus PR cold-read surfaces a log-readability
   complaint.** Go straight to S4 (PR).

**Compounding regen projection:**

| Order | After step | Iter-1 regen |
|-------|------------|-------------:|
| Current (S2 baseline) | — | 12.9 min |
| 1. F4-index S1 | S1 ship | ~1-1.5 min |
| 2. Seam-flange S1-S4 | each phase regen | ~1-1.5 min |
| Final state at PR time | all S-phases shipped | ~1-1.5 min |

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
