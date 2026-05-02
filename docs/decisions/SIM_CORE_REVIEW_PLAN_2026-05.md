# `sim/L0/core` /ultrareview plan — 2026-05-01 → 2026-05-05

| | |
|---|---|
| **Status** | LOCKED plan; pending `/ultrareview` invocation by user |
| **Budget** | 3 free `/ultrareview` runs, expire 2026-05-05; partial-runs count |
| **Target** | All 3 reviews on `sim/L0/core` (the 64,304-LOC MuJoCo-aligned rigid-body backbone) |
| **Strategic frame** | Bug-finding on the foundational physics core. Strategic / scope-molding work happens in interactive sessions, not /ultrareview slots. |

---

## §1. Why three reviews on sim/L0/core

`sim/L0/core` is **the** load-bearing physics backbone:
- Per `sim/L0/core/src/lib.rs`: "MuJoCo-aligned Model/Data architecture for physics-based simulations, including collision detection (GJK/EPA, BVH, height fields, SDF), contact resolution (PGS solver), and forward/inverse dynamics."
- Per memory: "MuJoCo as foundation — proven anchor, CortenForge extends beyond." Core IS the anchor.
- 64,304 LOC across 12 subsystems — too big for one /ultrareview bundle (docs warn "Large repos may need PR mode"). Sub-targeting is required.
- Bugs here are silent correctness defects (simulation produces wrong results) — the worst kind. /ultrareview's verification layer is purpose-built for catching these.

mjcf, urdf, mesh, soft, ml-chassis, rl, opt are NOT review targets in this plan. Loaders (mjcf/urdf) fail loudly at parse time; downstream crates can be reviewed post-quota for $5–$20 each if a real issue surfaces.

---

## §2. Three sub-bundles

`sim/L0/core` decomposes naturally into three coherent subsystems. Each becomes one /ultrareview pass.

### §2.1 Sub-bundle 1 — Solver path (~14,500 LOC)

The constraint-solving + dynamics + integration pipeline. Highest bug-density area in any physics codebase.

**Paths**:
- `sim/L0/core/src/constraint/` (8,771 LOC) — PGS / primal / no-slip solvers
- `sim/L0/core/src/dynamics/` (1,892 LOC)
- `sim/L0/core/src/integrate/` (771 LOC)
- `sim/L0/core/src/contact.rs` (1,505 LOC)
- `sim/L0/core/src/island/` (1,422 LOC)
- `sim/L0/core/src/energy.rs` (1,021 LOC)

**Focus framing for the multi-agent panel** (priority order):
1. Numerical stability + convergence in PGS / primal / no-slip solvers under degenerate inputs (rank-deficient Jacobians, near-zero contact distances, high-friction limits).
2. MuJoCo-PGS faithfulness — does the Rust port match MuJoCo's PGS reference behavior (warm-starting semantics, friction cone parameterization, restitution model)?
3. Energy conservation / drift across long simulations — symplectic-or-not characterization of the integrator.
4. Contact resolution correctness — does `contact.rs` produce contact points consistent with the broad-phase / narrow-phase output, with correct normals and frictional impulse application?
5. Constraint-island partitioning correctness — does `island/` cluster constraints such that no force-coupling crosses islands?
6. Rust-specific: panic-on-error vs error-propagation hygiene; lifetime / aliasing issues in mutable solver state.

### §2.2 Sub-bundle 2 — Collision path (~22,300 LOC)

Geometry, distance queries, contact detection. Largest sub-bundle by LOC; highest risk of edge-case bugs.

**Paths**:
- `sim/L0/core/src/collision/` (9,358 LOC) — broad-phase + narrow-phase
- `sim/L0/core/src/gjk_epa.rs` (1,346 LOC) — GJK + EPA distance + penetration
- `sim/L0/core/src/raycast.rs` (805 LOC)
- `sim/L0/core/src/sdf/` (5,213 LOC) — SDF support
- `sim/L0/core/src/heightfield.rs` (441 LOC)
- `sim/L0/core/src/mesh.rs` (1,748 LOC) — mesh-collider integration
- `sim/L0/core/src/mid_phase.rs` (6 LOC)
- `sim/L0/core/src/convex_hull.rs` (12 LOC)

**Focus framing**:
1. GJK + EPA correctness in degenerate cases — concentric shapes, exactly-touching shapes, parallel-face contacts. Both algorithms are notoriously fragile around degeneracies.
2. BVH + broad-phase determinism + correctness — does the broad-phase emit all real contacts and never spurious contacts?
3. Ray-cast precision — t-parameter correctness at glancing angles, hit-vs-near-miss boundaries, ray-origin-on-surface edge cases.
4. SDF query correctness — does `sdf/` evaluate sign + distance correctly for all primitive types, and do mesh-derived SDFs match the underlying mesh geometry?
5. Mesh-collider integration (`mesh.rs`) — vertex / face data transfer between mesh-types and collision; manifold preservation; convex-hull derivation.
6. Cross-platform FP determinism — collision tests are notoriously platform-divergent at level-set / ray-glancing boundaries.

### §2.3 Sub-bundle 3 — Actuation + dataflow path (~17,500 LOC)

Forward dynamics pipelines, Jacobians, tendons, sensors, batch processing, type-system invariants.

**Paths**:
- `sim/L0/core/src/forward/` (6,468 LOC)
- `sim/L0/core/src/jacobian/` (1,548 LOC)
- `sim/L0/core/src/tendon/` (1,352 LOC)
- `sim/L0/core/src/sensor/` (2,781 LOC)
- `sim/L0/core/src/inverse.rs` (47 LOC)
- `sim/L0/core/src/joint_visitor.rs` (206 LOC)
- `sim/L0/core/src/batch.rs` (796 LOC)
- `sim/L0/core/src/validation.rs` (827 LOC)
- `sim/L0/core/src/types/` (5,944 LOC) — shared physics types
- `sim/L0/core/src/plugin.rs` (673 LOC)
- `sim/L0/core/src/lib.rs` (357 LOC)
- `sim/L0/core/src/reset.rs` (13 LOC)
- `sim/L0/core/src/linalg.rs` (492 LOC)

**Focus framing**:
1. Forward kinematics + dynamics math — does `forward/` propagate joint state through the kinematic tree correctly per MuJoCo's reference behavior?
2. Jacobian computation — analytical vs autodiff consistency; chain rule correctness for compound joints.
3. Tendon + sensor model fidelity — do tendons apply forces correctly? Do sensors observe the correct quantities?
4. Batch processing (`batch.rs`) safety — multi-Data-instance independence; aliasing safety.
5. Type-system invariants in `types/` — Model immutability semantics, Data mutation safety, plugin interface contracts.
6. `validation.rs` — does it actually catch invalid Model configurations before they reach simulation?

---

## §3. Branch strategy

**Three separate branches off main**, one per review:

| Review | Branch name |
|--------|-------------|
| 1 (solver) | `feature/ultrareview-core-solver` |
| 2 (collision) | `feature/ultrareview-core-collision` |
| 3 (actuation) | `feature/ultrareview-core-actuation` |

Each branch contains exactly one new commit: a `REVIEW_FOCUS.md` doc at the repo root with the §2.x focus framing for that sub-bundle. No source-code changes. The branch off main + focus doc gives /ultrareview a stable target with explicit guidance.

**Why three branches, not one branch reused**: clean audit trail per review. Each branch is the historical record of "this is what we asked /ultrareview to look at on date X." After findings land, the branch can be deleted or kept for archival; doesn't matter — origin's reflog preserves the ref.

---

## §4. Calendar (May 1 → May 5)

| Date | Activity | Free runs after EOD |
|------|----------|---------------------|
| **2026-05-01** (today) | Plan locked. Branch 1 created (`feature/ultrareview-core-solver`) with `REVIEW_FOCUS.md`. User invokes `/ultrareview`. | 2 |
| **2026-05-02** | Review 1 delivery (~5–10 min, async). Findings documented in `docs/decisions/REVIEW_1_FINDINGS.md`. Triage — if any critical bugs, **pause + fix + reassess**. Otherwise proceed. | 2 |
| **2026-05-03** | Branch 2 created (`feature/ultrareview-core-collision`) with `REVIEW_FOCUS.md`. User invokes `/ultrareview`. Findings documented in `docs/decisions/REVIEW_2_FINDINGS.md`. | 1 |
| **2026-05-04** | Branch 3 created (`feature/ultrareview-core-actuation`) with `REVIEW_FOCUS.md`. User invokes `/ultrareview`. Findings documented in `docs/decisions/REVIEW_3_FINDINGS.md`. | 0 |
| **2026-05-05** | Final findings consolidation across all 3 reviews → `docs/decisions/CORE_REVIEW_SUMMARY_2026-05.md`. Budget closes. | 0 |

**Slack budget**: 1 day. /ultrareview turnaround is 5–10 min per the docs, so async delays should be minimal. If Review 1 surfaces a critical bug requiring a multi-day fix, calendar slips and Review 3 may forfeit. Acceptable outcome — Reviews 1+2 deliver the highest-leverage value.

---

## §5. Per-review pre-invocation gate

Before each `/ultrareview` invocation, all of these must be green:

1. The branch is created off latest main (not stale).
2. `REVIEW_FOCUS.md` exists at repo root, committed, with the §2.x focus framing copied verbatim + any session-specific additions.
3. `cargo build --workspace` is clean on the branch (don't waste a /ultrareview run on a non-building branch).
4. The user has explicitly approved this specific invocation (not a blanket pre-approval — each is a separate paid-on-overage commitment).
5. For Review 2 and 3 specifically: prior review's findings have been documented; critical-bug triage decision is documented (proceed / pause-and-fix).

Per `feedback_post_squash_merge_diff_verify`-adjacent hygiene: confirm branch state matches expectations before invocation.

---

## §6. Findings flow

Per the strategic-refinement decision (per-review findings memo, git-tracked):

| Stage | Artifact |
|-------|----------|
| Per-review findings | `docs/decisions/REVIEW_1_FINDINGS.md`, `REVIEW_2_FINDINGS.md`, `REVIEW_3_FINDINGS.md` — one per review, capturing the multi-agent panel's verified findings, severity rated, and triage decision |
| Cross-review consolidation | `docs/decisions/CORE_REVIEW_SUMMARY_2026-05.md` — landing zone for all findings, with cross-cutting patterns + prioritized fix queue |
| Fix tasks | Either inline TODOs in code with `// TODO(review-N-finding-K): ...` markers, OR separate fix-PRs scoped per finding, OR a single sweep-PR if findings cluster — per finding triage |

Findings memos are committed to the relevant review branch (or directly to main if the reviews land green and we're already merged); the consolidation doc is committed to main as the durable record. Per `feedback_code_speaks`: once fix-PRs ship, the findings memos can be deleted at end-of-arc — the fixes are in code; the memos are scaffolding.

---

## §7. Critical-bug response protocol

If any review surfaces a finding the panel + user agree is critical (silent correctness violation, memory unsafety, data corruption, etc.):

1. **Pause** — do not invoke the next review.
2. **Fix** — triage finding into a fix PR; land the fix.
3. **Reassess** the plan — does the fix change what Reviews 2-3 should focus on? E.g., if Review 1 finds a PGS-warm-starting bug, Review 2's collision focus might gain a "verify warm-starting interaction with broad-phase output" lens.
4. **Re-invoke** the next review with the updated focus framing.

If the fix takes more than 24h, calendar slips — Review 3 may forfeit. Acceptable.

Severity bar: silently-wrong simulation results, undefined behavior in safe code, data corruption. NOT acceptable as critical: API ergonomics complaints, performance regressions, missing test coverage, style issues.

---

## §8. What's deliberately OUT of scope for this plan

- mjcf review (loader; lower physics-bug-density; can review post-quota if needed).
- urdf review (lower physics-bug-density vs core; reviewable post-quota if a real consumer concern surfaces).
- mesh ecosystem review (mature, but not the rigid-body backbone; reviewable post-quota).
- soft, ml-chassis, rl, opt, thermo work (recently shipped or recently graded A).
- Strategic / scope-molding decisions (interactive-session work; /ultrareview is not the right tool for philosophy or strategic-direction calls — its verification layer is built for code-shaped findings, not judgment-laden debates).

---

## §9. Handoff to invocation

When user is ready to invoke Review 1:

```
1. Claude creates `feature/ultrareview-core-solver` branch off main.
2. Claude writes `REVIEW_FOCUS.md` at repo root (copy of §2.1 framing).
3. Claude commits + pushes branch.
4. User invokes `/ultrareview` (no args — bundles the local branch).
5. /ultrareview runs ~5–10 min in cloud sandbox.
6. Findings delivered; Claude documents in REVIEW_1_FINDINGS.md.
7. Triage; proceed or pause per §7.
```

Same shape for Reviews 2 + 3 with their respective branches + focus docs.

**Open prerequisite**: confirm 3 free runs are intact via `/help` or Anthropic dashboard before Review 1 invocation. If any prior session burned a run, plan compresses to 2 (or 1, or 0 — adjust as needed).

---

## §10. Cross-references

- [`mesh/MESH_V1_EXAMPLES_SCOPE.md`](../../mesh/MESH_V1_EXAMPLES_SCOPE.md) — v1.0 mesh examples spec; the other in-flight planning artifact this session.
- `sim/L0/core/src/lib.rs` — module-level docs; canonical description of the MuJoCo-aligned architecture under review.
- Memory: `project_mujoco_as_foundation.md` — MuJoCo-as-anchor framing.
