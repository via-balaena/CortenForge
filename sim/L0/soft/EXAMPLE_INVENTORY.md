# sim-soft Example Inventory + Plan

**Status:** SEED — to iterate across sessions before execution.

**Purpose:** surface what sim-soft can do, what examples exist (user-facing vs internal fixture vs missing), and what gaps stand between today's platform state and the workflow we actually need. Living document — delete after the cleanup arc completes per `feedback_code_speaks`.

---

## Scope

- Examples whose primary subject is sim-soft simulation.
- Includes inputs from cf-design (design surface) and `mesh-sdf` (scan import) since both are part of the user-facing workflow.
- Excludes pure-cf-design or pure-mesh examples (those would be separate inventories if ever needed).

---

## Anchoring engineering goal

The layered silicone device cavity-fit workflow per `project_layered_silicone_device` memory memo. Concretely:

- 3-layer poured silicone body (Ecoflex 00-30 / Dragon Skin 00-30 class).
- Cavity is **custom-fit to a literal scanned reference geometry**. Scan flows: `mesh-sdf::MeshSdf` → `cf-design::Solid` → `sim-soft::Sdf` → BCC + Isosurface Stuffing → tet mesh → FEM. **No parametric approximation of the scan** — user rejected that as "feels like calipers."
- Rigid indenter at sim time is the same scanned SDF, wrapped as a `RigidSdfPrimitive`.
- Readout: stretch / contact pressure / force-displacement at the cavity wall.
- Calibration: post-iter-1 cast, fit effective modulus to absorb ν=0.40 locking error (Fork B per session 2026-05-04 — predictive Phase H is parked).

The inventory should still catalog examples that are NOT directly tied to this device — the platform's general capabilities matter — but the anchoring goal sets the priority ordering.

---

## Status legend

- `have` — user-facing example exists in `examples/` (repo root) or `sim/L0/soft/examples/`.
- `internal-fixture` — capability is exercised in `src/readout/scene.rs` or under `tests/`, but no user-facing demo.
- `missing` — no demo of any kind.
- `blocked` — example would require a platform gap to be filled first.

---

## Inventory table

| # | Concept | Simplest example | Capability shown | Status | Gap to land |
|---|---|---|---|---|---|
| 1 | _(TBD next session)_ | | | | |

_(table to be filled iteratively; aim ~15-25 rows total when complete)_

---

## Identified platform gaps so far

Sketched during the 2026-05-04 conversation; refine and re-validate during iteration:

- `mesh-sdf → cf-design` bridge — `MeshSdf` should be wrappable as a `cf_design::Solid` primitive composable with parametric ones. (Memory's pre-2026-05-04 line "`mesh-sdf` adapter stays unbuilt absent a real consumer" is now obsolete — silicone device scan workflow IS the consumer.)
- `cf-design → sim-soft` bridge — `cf_design::Solid` should impl `sim_soft::Sdf` (or an equivalent one-line conversion). Today every user wires their own glue.
- `RigidSdfPrimitive` — extends Phase 5's `RigidPrimitive` trait beyond `RigidPlane`. Wraps any `cf_design::Solid` (which transitively wraps a scan SDF). Required for non-planar rigid indenters.
- Force / pressure readout at contact — needed for relative-comparison sim and empirical-calibration loop.
- Material reference — code constants module, builder, or doc table for `{Ecoflex 00-10/20/30/50, Dragon Skin 10A/20A/30A}` with Young's modulus / ν / source. Stops "look it up somewhere" friction.
- `sim/L0/soft/examples/` directory — does not exist today. Closest end-to-end demo is `SoftScene::layered_silicone_sphere` inside `src/readout/scene.rs` (a fixture in production source, not user-facing).
- Crate naming inconsistency — directory `sim/L0/soft/` vs crate name `sim-soft` (minor; flag if it adds friction during examples authoring).

Not in this list (deliberate Fork B parking):

- Phase H — F-bar reformulation / Tet10 quadratic elements / accurate ν=0.49 silicone behavior. Parked. Calibration absorbs the ν=0.40 locking error for now.
- Coulomb-Stribeck friction. Parked unless a specific example demands it.
- Differentiable-design adjoint methods. Parked.
- Multi-physics (electrical/thermal coupling for the conductive composite middle layer). Parked.

---

## Open scope questions to resolve during iteration

1. Should the inventory include capability-coverage examples that are **not directly on the silicone-device path** (e.g., a "drop a soft cube on a plane" Phase 5 demo, a multi-element NH stretch test, a SDF→tet meshing showcase)? Probably yes — but how many? Risk: bloat. Reward: museum-tour discoverability of the platform.
2. Ordering — capability-coverage order (museum tour) vs silicone-device-driven order (engineering path) vs interleaved? Affects which gaps get prioritized.
3. Where do user-facing examples actually live: `examples/` at repo root (current pattern for mesh examples) vs `sim/L0/soft/examples/` (per-crate)? Repo-root is the established mesh-v1.0 convention; per-crate is more standard Rust layout. Pick one; document.
4. What does each example's README minimum look like? `feedback_museum_plaque_readmes` says one concept per visual; what's the analog for sim-soft (numerical results + screenshot? force-displacement plot? deformation visualization?).
5. Do we author all examples in one PR or stage them across multiple PRs? Phase 5 was 12 commits in one PR; this is a different shape (parallel examples, less interlocking). May want internal commit segmentation per `feedback_pr_size_ci_economics`.

---

## Iteration log

_(append session-by-session; date-stamped; what changed and why)_

- **2026-05-04 (seed):** Initial structure + scope + identified gaps. No table rows yet — that's the first iteration's work.

---

## Cross-references

- `project_layered_silicone_device` (memory) — anchoring engineering goal.
- `project_gameplan` (memory) — six-phase soft-body plan; Phase 5 shipped 2026-04-29.
- `project_phase_5_commit_log` (memory) — Phase 5 deliverables (`RigidPlane`, `PenaltyRigidContact`, `RigidPrimitive` trait).
- `feedback_examples_drive_gap_fixes` (memory) — examples are *just as much* for fixing gaps as for showcasing.
- `feedback_simplify_examples` (memory) — realistic-looking parts before combining domains.
- `feedback_one_at_a_time` (memory) + `feedback_one_at_a_time_review` — review examples individually with two-pass review.
- `feedback_museum_plaque_readmes` (memory) — one concept per visual.
- `feedback_pr_size_ci_economics` (memory) — internal commit segmentation, not PR splitting.
- `feedback_code_speaks` (memory) — delete this file after the cleanup arc completes.

## Resume-here for next session

1. Read this file end-to-end.
2. Read `project_layered_silicone_device` memory memo.
3. Confirm scope + open questions §1-§5 are still right; redirect if not.
4. Begin authoring inventory rows. Start with capabilities you're sure of; mark uncertain ones with `?` for follow-up.
5. Update Iteration log with what you did + what's still open.
