# cf-cast parallel meshing recon (2026-05-25)

**Status:** scaffold. Triggered by S3 seam-gasket-mold iter-1 regen
([[project-cf-cast-seam-gasket-mold-s3]] commit `fc9f30e7`): adding
3 gasket molds at 0.5 mm cells pushed the production iter-1 regen
from ~5 min to **33.6 min** (2017.7 s). F4 gate dominates at ~540 s
per gasket × 3 = **27 min in F4 alone** — meshing itself is only
33 s per gasket. The 14 STLs that `export_molds_v2` emits are
mostly independent compose+mesh+gate units (cup pieces ⊥ plugs ⊥
platform ⊥ funnel ⊥ gasket molds); current pipeline runs them
serially.

**Picked architectural fix:** parallelize the per-unit compose +
mesh + F4 gate phases via `rayon::par_iter`. Preserves the existing
pre-write atomicity contract (all units gate-pass before any write).
Expected wall-clock reduction from ~28 min compute → ~10 min on a
typical 4-core workshop machine, limited by the longest single unit
(currently a gasket-mold F4 gate at ~540 s).

**Why NOT alternatives:**

- **F4 gate spatial-index upgrade (BVH for wall + overhang checks)**
  → DEFER. Bigger leverage (potentially 25× per-mesh gate from
  O(face²) → O(face × log face)) but dives into the
  `mesh-printability` crate internals; touches a primitive consumed
  by every cf-cast caller, needs its own recon arc + correctness
  gates. Should follow this recon as a separate arc once parallel
  meshing is in.
- **WGPU offload of MC sampling** → REJECTED at this scope. MC is
  only 33 s per gasket = ~5% of total time. The big-effort/low-
  reward ratio is poor here. The F4 gate doesn't parallelize cleanly
  onto GPU either (irregular nearest-neighbor geometry queries).
- **Reduce gasket MC cell size** (e.g. 1 mm) → REJECTED. Channel
  depth 0.8 mm at 1 mm cells = sub-cell → MC fragments the channel
  floor. Loses quality, which the user explicitly excluded.
- **Adaptive MC over the gasket Y range** (sample only the top 1 mm
  of the 3 mm tray, where the channel lives) → DEFER. Saves ~3×
  cells at no quality cost, but requires cf-design primitive
  surgery + risks introducing MC stitching artifacts at the
  sample-boundary. Separate arc if parallel meshing alone doesn't
  hit the workshop target.
- **Cache `body.evaluate` queries** for `compose_gasket_mold_solid`
  (which projects every channel-SDF probe to the seam plane,
  causing redundant SDF lookups across layers' shared seam-plane
  cross-sections) → DEFER. Would help compose+MC time but compose+
  MC is only 33 s per gasket; not the long pole.

**Out of scope:**

- Write-phase parallelization. The `save_stl` calls are
  filesystem-bound and total ~5 s in aggregate (small vs the 27 min
  F4 phase); parallelizing them risks corrupting STL counts /
  ordering in the report struct for marginal speedup. Keep serial.
- procedure.md generation. Already fast (<1 s) + sequential by
  necessity (single output file).
- F4 gate internal parallelization (single mesh's gate is single-
  threaded — that's the BVH/spatial-index separate arc territory
  above).
- Cancellation / SIGINT handling. Current cf-cast-cli doesn't
  support it; rayon doesn't either out of the box. Workshop user
  ctrl-C's the process if they need to abort. Leave unchanged.

## §P-0 Why this recon

S3 production iter-1 regen stderr trace
(`cast_iter1_gasket_s3_smoke/`, 2026-05-25 dev `fc9f30e7`):

| Phase                                   | Time (s) | % of total |
|-----------------------------------------|---------:|-----------:|
| Cup pieces × 6 (mesh + F4)              |     ~120 |        6 % |
| Plugs × 3 (mesh + F4)                   |      ~70 |       3 %  |
| Platform (mesh + F4)                    |       10 |       1 %  |
| Funnel (mesh + F4)                      |        4 |       0 %  |
| **Gasket molds × 3 (compose + MC)**     |      100 |       5 %  |
| **Gasket molds × 3 (F4 gate)**          |   **1620** |  **80 %**  |
| Write phase (14 STLs)                   |       ~5 |       0 %  |
| Other (procedure.md, accounting)        |       ~5 |       0 %  |
| **TOTAL**                               |  **2018** |    100 %   |

The F4 gate on the three gasket molds is the dominant cost. Each
gasket mesh has ~400 k faces (at 0.5 mm cells over the 200 × 30 ×
30 mm production bounding region), and `mesh-printability`'s
`validate_for_printing` runs multiple O(face²) checks (wall
thickness, overhang detection, sliver detection). 540 s per gasket
on a single core is the per-unit floor under the current F4
algorithm.

All 14 units are **input-independent**: cup piece N's compose +
mesh + gate doesn't read anything that plug M's mesh produces, and
vice versa. The pipeline is currently serial only because the
implementation walks units sequentially in `export_molds_v2`.

**The opportunity:** if we run the 14 units in parallel on a 4-core
machine, the wall-clock floor drops to **~9 min** (limited by the
slowest single unit's compute time = ~540 s F4 + ~33 s MC = ~573 s
per gasket; 3 gaskets × 1-core-each leaves the remaining 11 units
to share the other cores, finishing before the gasket molds). On
an 8-core machine all 14 units could run concurrently with wall-
clock → 573 s = 9.5 min.

## §P-1 Scope (what runs in parallel)

The `mesh_and_gate_v2_*` family in `design/cf-cast/src/spec.rs`:

- `mesh_and_gate_v2_pieces` — 2L cup pieces (Negative, Positive per
  layer). Currently iterates `for (layer_index, layer) in
  spec.layers.iter().enumerate()` then twice per layer.
- `mesh_and_gate_v2_plugs` — L plugs.
- `mesh_and_gate_v2_platform` — single optional platform.
- `mesh_and_gate_v2_funnel` — single optional funnel.
- `mesh_and_gate_v2_gaskets` — L gasket molds (S3-added).

Each unit's compose+mesh+gate is independent. The pre-write
atomicity contract (no STL lands on disk until ALL pass) is
preserved by collecting `PendingX` buffers across all units before
the write phase.

## §P-2 What doesn't run in parallel

- **Write phase** (`write_v2_artifacts`). The pre-write atomicity
  buffer flushes sequentially — file I/O is fast enough that the
  added complexity of parallel writes isn't worth it (~5 s total
  in the production regen).
- **A single mesh's internal F4 gate**. The O(face²) wall +
  overhang checks inside `mesh-printability` are single-threaded
  per mesh. Parallelizing those is the separate arc (BVH spatial
  index).
- **`procedure.md` generation**. Single output file, sequential by
  necessity, fast.
- **CastSpec validation, pour-volume integration, mass-budget
  check, ribbon construction** — all single-threaded pre-flight
  steps that finish in <1 s combined. No need to parallelize.

## §P-3 Concurrency model

`rayon::par_iter` over each `mesh_and_gate_v2_*` unit collection,
collecting into `Vec<Result<PendingX, CastError>>`. Then partition
the results — if any err, return the first error (canonical order
preserving). If all ok, unwrap into `Vec<PendingX>` for the write
phase.

**Per-unit input shape** (each rayon task captures by reference):

- `&CastSpec`, `&Ribbon`, `&Path` (out_dir) — all `Sync` already.
- Per-unit metadata (layer_index, piece_side, etc.) — small Copy
  values.

**Per-unit output:**

- `PendingPiece` / `PendingPlug` / `PendingPlatform` /
  `PendingFunnel` / `PendingGasket`. All hold owned `IndexedMesh`
  + `PrintValidation` + `PathBuf`.

**Order-preservation:** `par_iter().enumerate()` preserves index
ordering through `collect`. The output `Vec<PendingPiece>` etc.
must be innermost-first to match the existing serial contract
(consumed by `write_v2_artifacts` which depends on layer
ordering). rayon's `par_iter::collect` into `Vec<_>` preserves
input order naturally.

## §P-4 Error aggregation contract

The current serial pipeline returns the **first error encountered**
(walk halts on the first `?`). Two questions for the parallel
variant:

**(Q1) Deterministic-error-order vs first-finished-error.**

- **Deterministic-order (PICK)**: wait for all units; among errors,
  return the one with the lowest unit-index (canonical iteration
  order). Matches serial-pipeline semantics. Cost: pays the longest
  unit's wall-clock even if a fast unit errors early.
- First-finished: short-circuit on first error; rayon may still let
  in-flight tasks complete (no preemption). Non-deterministic which
  error surfaces if multiple units fail. Saves wall-clock on
  multi-failure cases but workshop multi-failure cases are rare.

**Picked: deterministic-order** for both reproducibility and
clearer workshop debugging (workshop user gets the same error
message on every re-run of a broken spec).

**(Q2) Multi-error reporting.**

- Current `CastError` is single-error. Surfacing N concurrent
  errors as a `Vec<CastError>` would require an enum-variant
  addition. Defer: in practice the second error is usually a
  consequence of the first (e.g. layer 1 cup-piece overflows
  bounding cuboid → so does layer 2 because they share an envelope
  +1 mm). Returning the first canonical error is sufficient.

## §P-5 Memory pressure

Current pre-write buffer peak (S3 production regen): ~480 MB at
the end of meshing, just before write phase. Three gasket meshes
at ~70 MB each dominate.

Under parallel meshing with full 14-unit concurrency:

- Peak memory grows as units mesh in parallel without waiting for
  one to finish before another starts. Worst case = all 14
  PendingX buffers alive simultaneously = ~700 MB (3 gaskets × 70
  MB + 6 cup pieces × 50 MB + 3 plugs × 20 MB + small platform/
  funnel).
- On a 16 GB workshop machine: comfortable.
- On a 4 GB constrained machine (none currently used per memory):
  could become an issue if cast sizes scale up.

**Concurrency cap.** Defaulting to `rayon::current_num_threads()`
(= num CPU cores) is the right floor. Optional `--threads N` CLI
flag in cf-cast-cli for workshop control. Defer to S2 if there's
a memory regression on production.

## §P-6 Logging

Each unit currently emits an `eprintln!` progress line after its
compose+mesh+gate completes. Under parallel meshing those lines
would interleave unreadably across units.

**Options:**

- **(a) Per-unit log buffer (PICK)**: each task accumulates its
  log lines into a `String`, returned alongside `PendingX`. After
  all units complete, flush the buffers in canonical order. Clean
  reading; trades log-immediacy for log-coherence.
- (b) Prefix-tagged interleaved lines: `[L0]`, `[L1]`, `[plug 2]`,
  etc. Live progress visible but harder to read for a 14-unit
  run.
- (c) Single line per unit (no compose / F4 sub-timings) emitted
  pre-task ("[cf-cast] queued L0 piece 0…") and post-task. Less
  detail than current.

**Picked (a).** Workshop user reads the log mostly post-mortem;
in-flight monitoring is via `ps`/`top` not log streaming. The
buffered approach gives a clean, sortable post-run summary.

## §P-7 Determinism on output

The MC algorithm + F4 validation are pure functions of their
inputs. Parallel execution doesn't change the per-unit output —
each mesh's vertex/face data + AABB + validation result MUST be
**bit-identical** to the serial pipeline.

**Test plan:**

- Run an existing serial test (e.g. `export_molds_v2_with_pour_gate
  _writes_pieces_plus_plug`) twice: once with serial pipeline,
  once with parallel. Assert STL byte-equivalence (post-write) +
  validation field equivalence (vertex/face counts, AABB).
- New test: `parallel_export_matches_serial` covering the iter1
  multi-layer fixture (the existing 2-layer test fixture is the
  right size for byte-equivalence comparison).

**Allowed diff:** stderr log order. That's the only non-
deterministic axis under (a) per-unit log buffering.

## §P-8 Pre-write atomicity preservation

The current contract: no STL lands on disk until every unit's F4
gate passes. Errors mid-pipeline abandon all in-memory PendingX
buffers (zero filesystem state mutation).

Under parallel meshing:

1. Spawn all unit tasks concurrently.
2. Wait for all to complete (rayon `collect::<Vec<Result<...>>>`).
3. Partition results — if any err, return the canonical-first err;
   drop all PendingX buffers (no FS mutation, contract preserved).
4. If all ok, proceed to serial write phase as today.

Contract is preserved by construction. No new FS-state-corruption
risk vs serial.

## §P-9 Rollout strategy

Three-phase implementation arc (S1-S3), one commit per phase, one
PR at arc-close per [[feedback-omnibus-pr-single-branch]]:

- **S1 — Parallelize gasket pipeline only** (~150 LOC). Smallest
  blast radius (gasket pipeline is the newest code; rest of cf-cast
  unaffected). Biggest single win (gasket F4 gate is 80% of regen
  time). Test: bit-identical output vs serial; release-mode timing
  check showing ~3× speedup on the gasket phase.
- **S2 — Parallelize cup pieces + plugs** (~100 LOC). Same pattern
  applied to `mesh_and_gate_v2_pieces` + `mesh_and_gate_v2_plugs`.
  These already parallelize within their own loop ranges; this
  unifies them with the gasket pipeline under a single concurrency
  pool.
- **S3 — Optional per-unit log buffer + `--threads` CLI flag**
  (~50 LOC). Defer the CLI flag if rayon defaults work fine in
  production. Phase the log buffer if (b) prefix-tagging proves
  readable enough.
- **S4 — Cold-read close + production regen timing comparison +
  PR**. Confirms workshop iter-1 regen drops from 33 min to ~10 min
  on a 4-core machine.

Total estimate: 3-4 wall-clock sessions, ~300 LOC.

## §P-10 Bail-out priority

If parallel meshing regresses correctness (mismatched STL bytes
vs serial), the bail-out ladder per
[[feedback-read-prior-arc-memory-before-architectural-decisions]]:

1. **Revert the failing phase commit.** Each phase is a single
   commit on dev; `git revert <hash>` restores serial behavior
   without losing surrounding work. Tested at S2 / S4.
2. **Fall back to single-threaded rayon pool** (`rayon::ThreadPool`
   with `num_threads(1)`). Validates that the bug is in
   parallelism per se vs the unit-iteration restructuring. Easy A/B
   test.
3. **Inspect for shared-mutable-state** (any `RefCell`, `Cell`,
   `unsafe`, `static mut` introduced by the diff). Rayon catches
   `!Send` / `!Sync` at compile time, so this should be the empty
   set, but worth a manual audit if behavior diverges from serial.
4. **Inspect for compile-time NaN-/inf-handling differences** in
   the MC + F4 paths (e.g. atomic min/max with NaN handling). MC
   is purely numerical; should be deterministic. If a unit
   produces different output under parallel vs serial, the bug is
   upstream of the parallelism.

## §P-11 Open questions

1. **Default thread count.** rayon defaults to all logical cores;
   workshop machine might want a user override to leave CPU for
   cf-view or browser usage during regen. Defer to S3 + ship a
   `--threads N` flag only if production surfaces a need.
2. **`mesh-printability` thread-safety.** Quick audit: the
   `validate_for_printing` entry point takes `&IndexedMesh` and
   `&PrinterConfig`, both immutable. No global mutable state in
   the crate (per a 30-second skim). Safe under rayon; verify at
   S1 with a thread-stress test.
3. **PendingX size invariants.** Each PendingX should be `Send +
   Sync` for rayon. `IndexedMesh` is just `Vec<f64>` /
   `Vec<usize>`; `PathBuf` is `Send + Sync`; `PrintValidation` is
   per-`mesh-printability` API. Verify at S1.
4. **Bounded thread pool for tests.** cf-cast lib tests + cf-cast-
   cli tests in CI shouldn't oversubscribe the test runner. rayon
   shares a global thread pool by default — tests running in
   parallel via cargo's harness will compete for the same pool.
   Either set rayon to a low thread count in tests, or accept some
   thrash. Defer to S1 — measure first.

## §P-12 Workshop-iter-N unblock criteria

A single quad-gate clearing this recon, ordered by load-bearing-
ness:

1. **Production iter-1 regen wall-clock < 12 min** on the workshop
   machine (was 33.6 min at S3). Measured via `time cf-cast-cli
   cast.toml --output-dir test`.
2. **Bit-identical STL output vs serial pipeline**. CI-gated via
   the new `parallel_export_matches_serial` test.
3. **All existing tests still pass** (225 cf-cast lib + 48 cf-
   cast-cli unit + 7 integration). Regression-bound.
4. **No new clippy `-D warnings` violations** + fmt clean. Same
   workspace lint floor.

## §P-13 Prior-arc memory checklist

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
rules 1-6, the following memories MUST be read by the S1
implementation session before any concurrency-introducing change:

- [[project-cf-cast-seam-gasket-mold-s3]] — the gasket pipeline
  this recon's biggest leverage hits. Confirms the
  `mesh_and_gate_v2_gaskets` function signature + the
  pre-write atomicity contract that must survive parallelization.
- [[project-cf-cast-mating-features-s4-seam-plane]] — original
  source of the pre-write atomicity contract (S4 of the original
  mating-features arc introduced PendingMold buffering for
  multi-layer F4 gate failures).
- [[project-cf-cast-fdm-friendly-geometry-arc]] — confirms cup-
  piece + plug compose paths are pure functions of their inputs
  (the FDM-friendly arc's S4-S6 mesh-CSG salvage didn't introduce
  any hidden global state); foundational for thread-safety.
- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — the
  paradigm-boundary framework is per-mesh, not per-cast; parallel
  meshing doesn't cross any paradigm boundary.
- [[feedback-cf-cast-tests-use-release]] — release-mode tests
  mandatory for the parallel-timing comparison test
  (`parallel_export_matches_serial`).
- [[feedback-xtask-grade-strictness]] — clippy / rustdoc
  strictness; parallel-meshing diff must clear both.

## §P-14 Cross-refs

- `~/scans/cast_iter1_gasket_s3_smoke/` — S3 iter-1 regen output
  (2017.7 s baseline). Compare timing here at §P-9 S1 + S4.
- `design/cf-cast/src/spec.rs` `mesh_and_gate_v2_*` functions —
  the parallelization targets.
- `design/cf-cast/src/spec.rs` `write_v2_artifacts` — the serial
  write phase that survives parallelization unchanged.
- `mesh-printability` crate — F4 gate; thread-safety audit target
  at §P-11 #2.
- rayon docs — `par_iter`, `ThreadPool`, `collect`.
- `docs/CF_CAST_SEAM_GASKET_MOLD_RECON.md` — sibling recon (the
  arc whose iter-1 regen surfaced the perf issue).

## §P-15 Followup arc (out-of-scope here, but on the roadmap)

If parallel meshing alone doesn't hit a workshop target of <5 min
iter-1 regen, the next arc tackles the F4 gate's internal O(face²)
checks: replace pairwise face-distance computation in
`mesh-printability` with a BVH/spatial-hash for nearest-neighbor
queries. Per-mesh F4 gate from 540 s → ~10-30 s = potentially
**25× per-unit speedup**, compounding with parallel meshing.
Bigger arc (touches a primitive consumed by every cf-cast call
site + needs correctness gates against the existing F4
validations). Separate recon: `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md`
when ready.

## Status (open)

- Scaffold shipped 2026-05-25 (this commit).
- Awaiting workshop user approval to proceed to S1.
- No code change yet; this is design doc only.
- cf-cast iter-1 regen at **33.6 min**; target post-S2 ~10 min on
  4-core, ~9 min on 8-core.
- Branch: `dev`, no push, no PR until arc close (per
  [[feedback-omnibus-pr-single-branch]]).
