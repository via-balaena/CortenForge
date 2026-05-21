# docs/archive — superseded bookmarks + state docs

This directory holds three-session-pattern bookmark/state docs whose arcs have shipped (or been falsified + reverted, or otherwise superseded). Files are kept rather than deleted so the audit trail — bookmark → recon → implementation → postmortem — stays git-navigable when someone reads a memory entry that references the historical reasoning.

If you need to read one of these, the project memory entry it corresponds to is the canonical pointer; the doc is the detailed cold-read scratch space.

## Index

### Sim-arc (F3 / candidate recon arcs)

| Archived file | Status when archived | Successor / superseder |
|---|---|---|
| `CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md` | RESOLVED case A (C′.a shipped) | `docs/F3_RECON_A_*` series |
| `CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` | RESOLVED case E (consts reverted to disabled) | F3 recon B sweep |
| `CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` | RESOLVED case D (H4 plumbing kept, scaffolding reverted) | F3 recon B candidate H4 sweep |
| `F3_FALSIFICATION_BOOKMARK.md` | F3 spec shipped through recon B | `docs/F3_LM_REGULARIZATION_SPEC.md` |
| `H2_MESH_REFINEMENT_BOOKMARK.md` | RECON COMPLETE; Q1 FALSIFIED; H2 ARC SHELVED | Banked open per `[[project-program-gameplan]]` |
| `CAVITY_INSET_STALL_BOOKMARK.md` | RESOLVED IN-SESSION 2026-05-18; F4 implementation arc landed | F3/F4 recon B implementations |

### cf-cast / cf-scan-prep

| Archived file | Status when archived | Successor / superseder |
|---|---|---|
| `CF_CAST_MOLD_WALL_BOOKMARK.md` | RESOLVED-BY-RECON 2026-05-19 | `[[project-cf-cast-mold-wall-recon]]` + `[[project-cf-cast-mold-wall-arc-shipped]]` (PR #249) |
| `CENTERLINE_RECON_BOOKMARK.md` | RESOLVED 2026-05-16 PR #248 | `[[project-cf-scan-prep-centerline-recon-bookmark]]` + `[[project-centerline-spec]]` |
| `PR_246_BOOKMARK.md` | Paused; superseded by PR #247 + #248 | `[[project-cf-scan-prep-v1-incomplete-pivot]]` |

### cf-device-design

| Archived file | Status when archived | Successor / superseder |
|---|---|---|
| `CF_DEVICE_DESIGN_CAVITY_MOUTH_BOOKMARK.md` | SHIPPED 2026-05-16; later marked WRONG-FIT | Superseded by cavity-pinned-floor arc |
| `CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_BOOKMARK.md` | RECON COMPLETE; scope-C shipped + falsified + reverted | `[[project-cf-device-design-cavity-pinned-floor-redesign-spec]]` (candidate A shipped) |
| `CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_FALSIFICATION_BOOKMARK.md` | Visual gate #1 FALSIFIED 2026-05-16 | Resolved by redesign-spec arc (candidate A) |
| `CF_DEVICE_DESIGN_INSERTION_SIM_OPEN_CAVITY_BOOKMARK.md` | Silently invalidated by cavity-mouth arc | (no live successor — closed) |
| `CF_DEVICE_DESIGN_LAYER_PREVIEW_BOOKMARK.md` | Phases 1-3 RESOLVED 2026-05-16; Phase 4 escalated | `[[project-cf-device-design-sdf-layers]]` (arc shipped) |
| `CF_DEVICE_DESIGN_SDF_LAYERS_BOOKMARK.md` | RESOLVED 2026-05-16 | `[[project-cf-device-design-sdf-layers]]` (arc shipped on dev `320d6dcd`) |
| `CF_DEVICE_DESIGN_FIT_VIZ_BOOKMARK.md` | SUPERSEDED by sim-arc recon 2026-05-18 | `[[project-cf-device-design-sim-arc-recon]]` (S1-S11.2 shipped; rungs 2-5 absorbed) |
| `INSERTION_SIM_STATE.md` | 2026-05-14 slice-7 bookmark; SUPERSEDED 2026-05-18 | `[[project-cf-device-design-sim-arc-recon]]` (replaces stale slice-7 + fit-viz memos) |

### Sim-decouple / SDF-trait-split / PR review (3)

| Archived file | Status when archived | Successor / superseder |
|---|---|---|
| `SIM_DECOUPLE_PHASE_3_RECON.md` | Phase 5 SHIPPED 2026-05-20; arc CLOSED via PR #249 | `[[project-sim-decouple-refactor-plan]]` |
| `CF_SDF_TRAIT_SPLIT_RECON.md` | RECON SHIPPED 2026-05-20; arc CLOSED via PR #249 | `[[project-cf-sdf-trait-split-recon]]` |
| `PR_249_REVIEW.md` | PR #249 merged 2026-05-20 | GitHub PR is canonical |

Note: `SIM_DECOUPLE_PHASE_3_RECON.md` has 12 live in-tree references (Cargo.toml descriptions + module-level docstrings in cf-device-geometry + cf-device-types) with section anchors like `§2.5.b-d`. Path references were rewritten from `docs/...` to `docs/archive/...` in the archival commit so the section anchors stay resolvable.

### Postmortems + closed recons + falsified specs (5)

| Archived file | Status when archived | Successor / superseder |
|---|---|---|
| `F4_FALSIFICATION_POSTMORTEM.md` | FALSIFIED 2026-05-18 | F4 interference-homotopy ruled out; F3 LM was the alt that shipped |
| `H2_RECON_FINDINGS.md` | H2 arc SHELVED per gameplan | `[[project-program-gameplan]]` (sim-arc paused) |
| `INSERTION_SIM_RECON.md` | 2026-05-14 slice-7 recon; superseded by sim-arc recon | `[[project-cf-device-design-sim-arc-recon]]` |
| `CF_SCAN_PREP_V1_RECON.md` | Gap list; superseded by PR #247 + #248 | `[[project-cf-scan-prep-v1-incomplete-pivot]]` |
| `CF_CAST_MOLD_WALL_RECON.md` | RESOLVED; arc shipped in PR #249 | `[[project-cf-cast-mold-wall-arc-shipped]]` |

### Recon-iter specs (3)

| Archived file | Status when archived | Successor / superseder |
|---|---|---|
| `CANDIDATE_H4_YEOH_BOUND_CALIBRATION_SPEC.md` | SHIPPED + FALSIFIED case D; scaffolding reverted | F3 recon B candidate H4 sweep |
| `F3_RECON_A_GATED_LM_SPEC.md` | SHIPPED outcome B via PR #249 | `docs/F3_LM_REGULARIZATION_SPEC.md` (canonical) |
| `CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_SPEC.md` | scope-C shipped + falsified + reverted | `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_REDESIGN_SPEC.md` (candidate A canonical) |

## What stays in `docs/` (not archived)

Three sim-arc bookmarks remain in `docs/` proper because the sim is research-paced + currently paused per `[[project-program-gameplan]]`, and the recon entry points are still live restart targets when sim work resumes:

- `CAVITY_5MM_CHATTERING_BOOKMARK.md` (F3 recon A outcome-B follow-up)
- `SIM_ARC_SLIDING_INTRUDER_BOOKMARK.md` (S11.3 pivot — recon entry point)
- `SIM_ARC_VISUAL_REWRITE_BOOKMARK.md` (sim viewer architectural rewrite — recon entry point)

Move these here when their arcs ship or get formally shelved.
