# CortenForge Visual-Review Viewer — Design Plan

**Status:** SEED — to iterate across sessions before execution.

**Purpose:** Design the unified visual-review viewer that supersedes per-example tool chains (MeshLab + filter dialogs for sim-soft; f3d for mesh examples). One command, one window, every static-field artifact in the workspace. Living document — delete after the viewer ships and the workspace has migrated to it, per `feedback_code_speaks`.

---

## Originating signal

**2026-05-04, sim-soft examples arc PR1 commit 1 — `93b4d5a4` (sphere-sdf-eval).** Visual pass surfaced a gap the iter-2 visualization-convention round didn't anticipate:

- **MeshLab** opened the 1331-point PLY but required a 2-filter chain per example to render `extras["signed_distance"]` (`Filters → Quality Measure and Computations → Per Vertex Quality Function` with `func q = signed_distance`, then `Filters → Color Creation and Processing → Colorize by Vertex Quality`). Per-example friction × 10 PR1 rows = a lot of clicking, every review pass.
- **f3d** ignores per-vertex custom scalars entirely — it would just render 1331 white dots in a grid. f3d is geometry-only.
- **The user separately flagged that f3d had already been underwhelming for the mesh-v1.0 examples** — even though those examples' content IS geometric (which f3d nominally handles), f3d's rendering wasn't living up to the visual review the user wanted.

The PLY-vs-Bevy split locked in the sim-soft iter-2 round conflated two cases. **Mesh-v1.0's content is geometric** (shape, winding, surface, lattice density) — f3d nominally fits but disappoints. **Sim-soft Tier 1's content is field-over-space** (a scalar over a sampled domain) — no native viewer in the workspace renders that well. The fix is workspace-level: a unified viewer that handles both, replacing both f3d and MeshLab.

Per `feedback_examples_drive_gap_fixes` — examples drive engine fixes; this gap surfaced exactly when an example exercised it. Per `feedback_fix_gaps_before_continuing` — sim-soft PR1 paused at commit 1; viewer ships first.

---

## Anchoring goals

1. **Single tool** — one binary serves mesh, sim-soft, cf-design, integration, casting-domain, and future-domain static-artifact examples.
2. **Zero-friction visual pass** — `<binary> <path>` opens a colormapped, oriented, navigable view. No filter chains, no menu spelunking, no per-example viewer setup.
3. **Field-aware** — auto-detects per-vertex / per-face scalars and colormaps them appropriately (divergent for signed, sequential for unsigned, categorical for integer IDs). User toggles between scalars if multiple are present.
4. **Geometry-faithful** — meshes render with correct winding and shading; point clouds render with adjustable point size; lines render with controllable widths. The viewer does NOT silently re-orient or re-wind input data.
5. **Scriptable / reproducible** — viewer accepts a config file or CLI flags so a saved view (camera angle, scalar choice, colormap range) can be replayed for screenshots / regression-style visual review.

---

## Scope boundaries

**In scope (v1):**

- Static-field examples — sim-soft Tier 1-3 + Tier 5 rows 1-3, 8-11, 15-16, 19 (10 examples).
- Static-geometry examples — mesh-v1.0 examples (eventual retrofit; see open question 7).
- Format support: PLY (binary + ASCII), with custom per-vertex / per-face scalars.
- Cross-platform Bevy app with orbit camera, scalar selector, colormap selector.

**Out of scope (v1):**

- Dynamic playback — Tier 4 + Tier 6 (drop-and-rest, Hertz, compressive block, silicone device E2E) handled by full per-example Bevy apps because they need custom interaction (scrubbing, force arrows, contact-pressure overlay).
- Time-series / scrubbing / animation. Static-only for v1.
- Vector fields (gradient arrows, displacement vectors) — could extend the viewer's surface but no immediate consumer.
- JSON force-stretch / displacement traces — matplotlib post-hoc; not the viewer's job.
- Interactive editing or parameter sweeping — viewer is read-only.
- Headless / CI rendering — desktop tool only; CI builds it via `cargo build` for integrity but doesn't run it.

---

## Open questions to resolve during iteration

1. **Crate location.** `tools/viewer/` (alongside `xtask/`)? `cf-viewer/` at workspace root? `forge/viewer/`? Affects discoverability and whether the crate is a workspace member or a standalone tool. `xtask/` precedent: top-level workspace member, dev-tooling category.

2. **Binary name.** `cf-view`? `forge-view`? `cortenforge-view`? Affects user-facing CLI ergonomics and `cargo install` discoverability. Short + memorable + unambiguous.

3. **Input formats.** PLY-only for v1 (matches what every existing example emits)? OBJ / STL too (broader interop)? Some formats don't carry per-vertex scalars natively (STL has no scalar fields); how do we handle those? Library API in addition to file-input, for in-process use cases?

4. **Scalar detection.** Auto-discover all per-vertex / per-face scalars in the PLY and let the user toggle via UI dropdown? CLI flag like `--scalar=signed_distance` for explicit selection? Heuristic ranking by name (e.g., prefer "signed_distance" > "quality" > arbitrary) when multiple are present? What if zero scalars are present (geometry-only mesh)?

5. **Colormap policy.** Divergent (cool→white→warm) for signed scalars, sequential (viridis) for unsigned, categorical for integer IDs — but **how does the viewer decide which a scalar is**? Heuristics on min/max sign? Naming convention? Explicit metadata in the PLY? Manual override CLI flag?

6. **Up-axis convention.** `+Z` to match the build-plate orientation already used by mesh-v1.0 (which sets `f3d --up=+Z`)? Hard-code at startup, accept a `--up=+Z` flag, or auto-rotate the camera intelligently? What about examples whose natural orientation differs (e.g., a layered sphere has no natural "up")?

7. **Mesh-v1.0 retrofit story.** When do the 9 mesh-v1.0 examples migrate from f3d to the new viewer? **Same PR as viewer ships** (forces a full migration test against real consumers)? **Followup PR** after the viewer's API stabilizes against fresh consumers? **"As touched"** during normal development churn? Affects PR shape and migration risk.

8. **CI handling.** Does the viewer build in CI (`cargo build` integrity for compile-correctness)? Does it run in CI (probably not — desktop tool, no headless render server in standard CI runners)? Gate the viewer crate behind a feature flag so the workspace's default-build doesn't pull in Bevy on every CI run?

---

## Adjacent / related decisions (not gating but worth tracking)

- **Sim-soft Bevy adapter** for deforming tet meshes (boundary-face re-extraction per frame from `x_current`) is the Tier-4-and-Tier-6 dynamic-playback path. **Not the unified viewer's responsibility** — those are per-example Bevy apps. The adapter could live in the same crate as the viewer if there's natural reuse (mesh rendering primitives, colormap utilities, camera setup), or in `sim/L1/sim-bevy/` if the dynamic-playback story diverges.
- **Casting-domain / cf-design future examples** — the viewer should anticipate them. cf-design likely emits the same PLY shape (geometry + per-vertex scalars). Casting domain may want time-series support eventually — but that's v2, not v1.

---

## Iteration log

_(append session-by-session; date-stamped; what changed and why)_

- **2026-05-04 (seed):** Initial structure + originating signal + anchoring goals + scope + 8 open questions. No design decisions yet — that's iter 1's work. Sim-soft examples arc paused at PR1 commit 1; viewer ships first per `feedback_fix_gaps_before_continuing`.

---

## Cross-references

- **Originating arc:** [`sim/L0/soft/EXAMPLE_INVENTORY.md`](../sim/L0/soft/EXAMPLE_INVENTORY.md) — sim-soft examples arc; visualization convention section flags this doc as the supersession point.
- **Existing viewer-pattern reference:** [`examples/mesh/README.md`](../examples/mesh/README.md) — current f3d convention; mesh retrofit target (open question 7). f3d's `--up=+Z` flag and per-example viewer-instruction blocks are the precedent the unified viewer replaces.
- **Bevy precedent in workspace:** `examples/fundamentals/sim-cpu/` — 60+ Bevy examples with the headless-asserts-pre-render + visual-mode-default pattern. The unified viewer is simpler than these (no simulation, just rendering) but inherits their build-time + dep-weight cost profile.
- **Cadence memos:**
  - [`feedback_examples_drive_gap_fixes`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md) — examples are *just as much* for fixing gaps as for showcasing.
  - [`feedback_fix_gaps_before_continuing`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_fix_gaps_before_continuing.md) — when examples reveal engine gaps, stop and fix the engine first.
  - [`feedback_planning_as_therapy`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_planning_as_therapy.md) — for ambitious personal projects, default to ceiling-level architecture; planning depth IS the product.
  - [`feedback_master_architect_delegation`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_master_architect_delegation.md) — during architectural execution with documented plan, Claude makes tech decisions; brings only project-scope / UX / blocker calls.
  - [`feedback_no_allergy_to_bold_choices`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_no_allergy_to_bold_choices.md) — don't pre-weight to minimum-diff / battle-tested options.
  - [`feedback_simplify_examples`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_simplify_examples.md) — viewer's simplicity reduces per-example complexity.
  - [`feedback_code_speaks`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_code_speaks.md) — delete this doc after the viewer ships and the workspace has migrated.

---

## Resume-here for next session

1. Read this file end-to-end.
2. Read the sim-soft inventory's iter-3 log entry + superseded-banner block in `sim/L0/soft/EXAMPLE_INVENTORY.md` for the originating-signal context.
3. Confirm anchoring goals + scope still right; redirect if not.
4. Begin walking the 8 open questions. Per `feedback_master_architect_delegation`, lock decisions where confident as head-architect calls; flag `?` for user only on project-scope / UX / aesthetic items (e.g., binary name).
5. Update Iteration log with what changed and what's still open.
6. After all 8 are locked, propose PR shape + commit segmentation for the viewer arc — branch off main as `feature/cf-viewer-arc`.
