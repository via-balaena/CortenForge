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

Organized by capability tier (museum-tour readability). The silicone-device synthesis examples sit at the bottom — they are gated on bridge work that earlier rows do *not* require, so reading top-to-bottom is also a rough dependency-chain. Priority for *which PR each row ships in* is set by §Q5 below, not by table order.

Status legend per top of file: `have` / `internal-fixture` / `missing` / `blocked`. The `Capability shown` column names the *one* concept the example is the canonical visual for. The `Gap to land` column names the platform work + wrap work needed.

### Tier 1 — SDF primitives + meshing (Phase 3 surface)

| # | Concept | Simplest example | Capability shown | Status | Gap to land |
|---|---|---|---|---|---|
| 1 | SDF primitive eval + grad | `sphere-sdf-eval` — sample `SphereSdf::eval` + `grad` over a grid; emit signed-distance PLY (vertex extras like mesh-sdf-distance-query) | `Sdf` trait contract; analytic distance + unit gradient on the zero set | `internal-fixture` (unit tests in `sdf_bridge/sdf.rs`) | wrap as `examples/sim-soft/sphere-sdf-eval/`; numerical anchors = surface-zero + interior-negative + grad-norm-unit asserts |
| 2 | Boolean SDF composition (difference) | `hollow-shell-sdf` — `SphereSdf{R_outer} \ SphereSdf{R_cavity}` via `DifferenceSdf`; emit 2D slice cross-section PLY at z=0 plane | sharp-CSG difference for hollow bodies (book Part 7 §00 §01) | `internal-fixture` (used inside `SoftScene::layered_silicone_sphere`) | wrap as `examples/sim-soft/hollow-shell-sdf/`; assert the at-surface eval = 0 on inner + outer radii |
| 3 | SDF → tet meshing (single material) | `sdf-to-tet-sphere` — solid `SphereSdf` → BCC + Isosurface Stuffing tet mesh via `SdfMeshedTetMesh`; emit boundary-face PLY | Phase 3 SIGGRAPH-2007 stuffing pipeline; tet-quality bounds | `internal-fixture` (`sdf_quality_bounds.rs`, `sdf_pipeline_determinism.rs`) | wrap as `examples/sim-soft/sdf-to-tet-sphere/`; assert tet-count band + min/max edge ratio + `referenced_vertices` orphan-rejection invariant |
| 4 | Hand-built tet (skeleton path) | `single-tet-stretch` — `SoftScene::one_tet_cube` + uniaxial load via `BoundaryConditions`; print final-state Tensor + assert displacement sign + magnitude | `SingleTetMesh` + `BoundaryConditions` walking-skeleton pattern (alternative to SDF path for hand-authored scenes) | `internal-fixture` (Phase 1 invariant tests) | wrap as `examples/sim-soft/single-tet-stretch/`; assert `x_final[3*v3 + 2] > 0` + analytic displacement target |

### Tier 2 — Constitutive + multi-element (Phase 1–2 surface)

| # | Concept | Simplest example | Capability shown | Status | Gap to land |
|---|---|---|---|---|---|
| 5 | Neo-Hookean uniaxial stretch + inversion handling | `neo-hookean-uniaxial` — single `Tet4` + analytical NH stress-strain comparison across stretch values **including a sub-trace through compressive inversion** to show `InversionHandling` / `ValidityDomain` clamp behavior | `NeoHookean` constitutive law (`Material::psi` / `pk1` against closed-form) + robustness boundary at negative-J — both branches of one canonical example per Q2 fold-decision | `internal-fixture` (`material_fd.rs`, `element_fd.rs`) | wrap as `examples/sim-soft/neo-hookean-uniaxial/`; assert per-stretch PK1 within tolerance of analytic; assert clamp behavior at the `ValidityDomain` boundary; emit JSON force-stretch trace |
| 6 | Multi-element FEM stretch | `multi-element-stretch` — N-tet `HandBuiltTetMesh` (e.g., 8-tet hex split) under uniform Dirichlet stretch; assert per-element stress agreement | Phase 2 multi-element assembly; per-tet local → global stitch via `referenced_vertices` | `internal-fixture` (`multi_element_grad_scaling.rs`, `multi_element_isolation.rs`) | wrap as `examples/sim-soft/multi-element-stretch/`; assert stress uniformity across all tets within FP tolerance |

### Tier 3 — Multi-material spatial fields (Phase 4 surface)

| # | Concept | Simplest example | Capability shown | Status | Gap to land |
|---|---|---|---|---|---|
| 8 | Layered scalar material field | `layered-scalar-field` — `LayeredScalarField` over a SDF stack (3 concentric shells); emit per-tet material-tag PLY (color = layer ID) | sharp-boundary spatial composition; `Field<f64>: Send + Sync` trait-object stack | `internal-fixture` (`sdf_material_tagging.rs`, `multi_material_validity.rs`) | wrap as `examples/sim-soft/layered-scalar-field/`; assert per-tet material assignment matches the SDF's predicate; visual = tagged surface PLY |
| 9 | Blended scalar material field | `blended-scalar-field` — `BlendedScalarField` smooth sigmoid transition between two layers; same-shape PLY as #8 but with per-tet (μ, λ) gradient instead of step | smooth-transition material grading (vs #8's sharp CSG step) | `internal-fixture` (`field_unit.rs`) | wrap as `examples/sim-soft/blended-scalar-field/`; assert monotonic μ-gradient across the transition zone |
| 10 | Bonded multi-material continuity | `bonded-bilayer-beam` — bonded soft+stiff bilayer cantilever beam under tip load; assert displacement asymmetry vs single-material baseline | shared-vertex multi-material coupling (no inter-layer slip) | `internal-fixture` (`bonded_bilayer_beam.rs`, `multi_material_continuity.rs`) | wrap as `examples/sim-soft/bonded-bilayer-beam/`; assert tip-displacement matches bilayer-beam analytic to 3 digits |
| 11 | Three-shell concentric hollow sphere (IV-5) | `concentric-lame-shells` — `SoftScene::layered_silicone_sphere` with internal pressure load; assert radial displacement profile against Lamé closed-form for 3-layer composite | the canonical IV-5 multi-material + SDF + hollow-body integration test, exposed as a user-facing demo | `internal-fixture` (`concentric_lame_shells.rs`) | wrap as `examples/sim-soft/concentric-lame-shells/`; reuse `LAYERED_SPHERE_R_*` constants; assert radial displacement vs Lamé per shell |

### Tier 4 — Penalty contact (Phase 5 surface)

| # | Concept | Simplest example | Capability shown | Status | Gap to land |
|---|---|---|---|---|---|
| 12 | Drop-and-rest on plane | `soft-drop-on-plane` — soft sphere falls under gravity, settles on `RigidPlane`; assert final COM height = R + tolerance | one-way penalty contact + rest-state convergence (V-5) | `internal-fixture` (`contact_drop_rest.rs`) | wrap as `examples/sim-soft/soft-drop-on-plane/`; emit final-frame deformed-mesh PLY; assert quiescence (max velocity < threshold) |
| 13 | Hertzian sphere ↔ plane | `hertz-sphere-plane` — soft sphere quasi-statically pressed against `RigidPlane`; assert contact-patch radius vs Hertz analytic | analytical contact-pressure validation (V-3) | `internal-fixture` (`hertz_sphere_plane.rs`) | wrap as `examples/sim-soft/hertz-sphere-plane/`; assert patch radius within Hertz tolerance; emit pressure-vs-radius JSON |
| 14 | Compressive block under plates | `compressive-block` — soft cube between two `RigidPlane`s, top plate descends quasi-statically; assert engineering-stress curve | `PenaltyRigidContact` under bilateral compression (V-3a) | `internal-fixture` (`penalty_compressive_block.rs`) | wrap as `examples/sim-soft/compressive-block/`; emit force-displacement JSON; assert NH-derived stiffness slope |

### Tier 5 — Bridges + extensions (silicone-device path; **mostly blocked**)

| # | Concept | Simplest example | Capability shown | Status | Gap to land |
|---|---|---|---|---|---|
| 15 | mesh-sdf → cf-design bridge | `mesh-scan-as-solid` — load STL/PLY scan via `mesh_io`, build `mesh_sdf::SignedDistanceField`, sample via `cf_design::Sdf` adapter; render as marching-cubes preview | scanned-geometry-as-design-primitive (the literal-SDF path per device memo) | `missing` | **BLOCKED** per Q1 architecture decision: (a) define `cf_design::Sdf` trait (`eval(Point3) -> f64`, `grad(Point3) -> Vec3`) — owned by cf-design so orphan rules permit external impls; (b) `impl cf_design::Sdf for mesh_sdf::SignedDistanceField` lives in cf-design (mesh-sdf dep optional behind a feature if dep weight matters). No `MeshSdf` newtype — use the real type name. **Type bridge:** cf-design uses `Point3<f64>`, sim-soft uses `Vec3 = Vector3<f64>` — adapter must include the conversion |
| 16 | cf-design → sim-soft bridge | `solid-to-sim-soft` — compose `Solid::sphere(R)` boolean-difference `Solid::sphere(r)`; mesh via `SdfMeshedTetMesh::from_sdf(&solid, ...)` taking `&dyn cf_design::Sdf`; minimal pressurization scene | one-line glue from cf-design design surface to sim-soft simulation surface | `missing` | **BLOCKED** per Q5 architecture decision: `impl cf_design::Sdf for cf_design::Solid` directly (the existing inherent `Solid::evaluate(Point3) -> f64` becomes the trait `eval`; `grad` either analytical-per-primitive or central-difference fallback). Sim-soft's `Sdf` trait is **migrated to a re-export** of `cf_design::Sdf` (breaking change — acceptable per `feedback_planning_as_therapy` foundational-fix preference). `SphereSdf`/`DifferenceSdf` stay in sim-soft as deprecated test-only primitives until cf-design's `Solid::sphere()` covers their use sites; lazy migration |
| 17 | `RigidSdfPrimitive` (non-planar rigid indenter) | `soft-meets-rigid-sphere` — soft cube pressed by a rigid sphere indenter wrapping any `Sdf` (sphere first; later, the device's scan-derived rigid form) | extends Phase 5's `RigidPrimitive` trait beyond `RigidPlane` to arbitrary SDFs | `missing` | **BLOCKED** on Phase 5.5 retirement decision: `RigidSdfPrimitive` impl wrapping `Box<dyn Sdf>` + signed-distance + outward-normal (gradient). Pair-companion to Hertz example #13 once landed |
| 18 | Force / pressure readout at contact | `contact-force-readout` — same scene as #14, but extract per-contact-pair pressure + integrated normal force into a JSON trace | quantitative readout for relative-comparison sim + post-cast empirical calibration loop (Fork B) | `missing` | **BLOCKED** on `PenaltyRigidContact::contact_pairs() -> impl Iterator<Item = ContactPair>` accessor + a per-pair force readout helper. Phase 5 sets up `ContactPair` + `ContactGradient` but no public force aggregator |
| 19 | Material reference (silicone constants) | `silicone-material-table` — shipped doc table + code constants module mapping `{Ecoflex 00-10/20/30/50, Dragon Skin 10A/20A/30A}` → `(μ, λ, ρ)` w/ source citation | engineering-grade material lookup, not "guess your Lamé params from a search bar" | `missing` | **BLOCKED** on `sim/L0/soft/src/material/silicone_table.rs` (or similar) — tabulated `pub const` Lamé pairs + ν=0.40 compressible-NH framing per Fork B |

### Tier 6 — Synthesis (the engineering goal)

| # | Concept | Simplest example | Capability shown | Status | Gap to land |
|---|---|---|---|---|---|
| 20 | Layered silicone device E2E (cavity-fit) | `layered-silicone-device` — load scan via #15 → cf-design boolean composition (parametric layer SDFs ∩ scan complement for cavity) → sim-soft tet mesh → 3-layer multi-material assignment per #8 → static fit pose with rigid scan-SDF indenter per #17 → stretch / pressure / force-displacement readout per #18 | the anchoring engineering goal — relative-comparison sim of cavity tightness on a poured-silicone layered device | `blocked` | **BLOCKED** on rows #15 + #16 + #17 + #18 + #19. Material budget: 2 lb/silicone hard cap. Sanitization: "scanned reference geometry" only, no anatomical references anywhere visible |

### Deferred to future arcs (Q3, Q4 decisions — out of this inventory)

- **Forward-map gradcheck demo** (former row 21) — `SkeletonForwardMap` / `Differentiable` / `NewtonStepVjp` autograd surface. **Deferred to a future "differentiable-sim" arc.** Reasoning: the autograd surface deserves its own arc with multiple coordinated examples (forward gradcheck, scene-param sweep, parameter-fitting demo), not one orphan row here. The thermo-RL-loop motivation is real but tangential to the layered silicone device anchoring goal of this arc.
- **Reward breakdown demo** (former row 22) — `RewardWeights` / `RewardBreakdown` readout. **Deferred indefinitely.** Reasoning: no real `VecEnv`-wrapping consumer exists for sim-soft today; authoring a demo over an extension point with no consumer is cargo-cult coverage. Re-evaluate if and when a sim-soft RL env materializes.

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

## Open scope questions — answers + remaining `?`

### Q1: Capability-coverage examples not on the silicone-device path?

**Recommendation: yes, bounded by "every public-API surface gets ≤ 1 canonical example."** Tiers 1-4 of the inventory (rows 1-14) cover the existing platform museum-tour. Tier 5 (rows 15-19) is the device-path bridge work. Tier 6 (row 20) is the device synthesis. Rows 21-22 are flagged `?` for explicit defer-or-include.

**Why this bound:** (a) the mesh-v1.0 arc shipped 9 capability-coverage examples (PR #224 + #225) — sim-soft has roughly the same exposed-API surface area, so ~10-14 capability-coverage rows is precedent-aligned; (b) per `feedback_examples_drive_gap_fixes`, examples *expose* gaps — Tier 5 only became visible because Tier 1-4 was authored first; (c) per `feedback_museum_plaque_readmes`, the museum-tour is itself the value, not a dilution of the device-driven priority.

**Concrete cap:** ~14 user-facing examples planned for ship, plus the ≤ 1 silicone-device synthesis row gated on bridge work, with rows 21/22 to be confirmed-or-dropped before any execution. Total: ~15-16 user-facing examples.

### Q2: Ordering — capability-coverage vs silicone-device-driven vs interleaved?

**Recommendation: museum-tour order in the inventory table; PR-staging order driven by the silicone-device dependency chain.** These are two separate questions and conflating them is what made §Q2 hard to answer in the seed.

**Why:** the table is a *reference* — readers want to find "the canonical example for SDF→tet" without scanning a device-priority order; museum-tour ordering serves them best. *PR sequencing* is a different artifact (§Q5 below) and IS device-priority-driven, because per `feedback_examples_drive_gap_fixes`, what we author first is what reveals the bridges to fix — the device path drives that sequencing even though the table presents capability tiers.

### Q3: Where do user-facing examples live?

**Recommendation: `examples/sim-soft/<name>/` at the repo root.**

**Why:** the mesh-v1.0 convention (`examples/mesh/<name>/`) is the established repo pattern with shipped READMEs, layout docs, and a per-example template (verified via `examples/mesh/README.md` lines 141-153). Adopting it verbatim — same directory shape, same `[package].name = "example-sim-soft-<name>"` naming, same README template — costs nothing and locks one more layer of consistency across the repo.

`sim/L0/soft/examples/` (per-crate, native Rust convention) loses the cross-domain discoverability that `examples/EXAMPLES.md` provides and would be the only crate doing it that way. Reject.

**Doc obligation when first example lands:** add a `## fundamentals/sim-soft/` section to `examples/EXAMPLES.md` mirroring the existing `## fundamentals/sim-cpu/` and `## fundamentals/mesh/` (the mesh examples actually live at `examples/mesh/`, not `examples/fundamentals/mesh/` — the EXAMPLES.md doc is one PR behind reality and that drift is a small followup, not a blocker). Add a top-level `examples/sim-soft/README.md` mirroring `examples/mesh/README.md` with the same per-example template.

### Q4: README minimum for sim-soft examples?

**Recommendation: adopt the mesh-v1.0 template verbatim, with sim-soft-specific anchor + visual conventions:**

- **Concept paragraph** — the one capability this demonstrates (one sentence, the platform contract).
- **What it does** — input scene (mesh-or-SDF) + transformation applied (load / pressure / contact / step) + output written (PLY + JSON + optional PNG).
- **Numerical anchors** — math-pass-first per `feedback_math_pass_first_handauthored`. For SDF / meshing examples: tet count, vertex band counts, boundary-band non-emptiness asserts (already the pattern in `SoftScene::layered_silicone_sphere` lines 238-249). For FEM / contact / multi-material examples: per-vertex displacement targets, energy bounds, force-displacement slope, Lamé / Hertz / NH closed-form comparisons. Clean `cargo run --release` exit-0 IS the correctness signal.
- **Visuals** — written 1:1 with what the user opens in f3d / MeshLab. For deformation: emit boundary-face PLY of the deformed configuration; for material tagging: per-tet color PLY; for force-displacement: JSON traces (Python plot is post-hoc, not in-example). **f3d winding callout** per `feedback_f3d_winding_callout` if any example emits inward-wound shells.
- **Run** — `cargo run -p example-sim-soft-<name> --release` per `feedback_release_mode_heavy_tests`.

**Math-pass-first specifically:** mesh examples encode coords + windings + bbox numerically. FEM analogs: encode displacement magnitudes, energy values, residual norms, contact-pair counts. Per the `concentric_lame_shells.rs` precedent — the radial-displacement vs Lamé closed-form is exactly this, and that test should *be* the example.

### Q5: One PR or multiple?

**Recommendation: 3 PRs gated by the device-dependency chain, with internal commit segmentation per `feedback_pr_size_ci_economics`.**

- **PR1 — Capability-coverage core (Tier 1-3 rows 1-6, 8-11).** **10 user-facing examples** wrapped from existing internal fixtures. No new platform machinery. Lands `examples/sim-soft/` directory + `examples/EXAMPLES.md` doc updates + per-example READMEs. Internal commit segmentation: ~one commit per example for the two-pass review cadence (`feedback_one_at_a_time_review`). PR size precedent: PR #224 shipped 8 mesh examples; this is comparable.
- **PR2 — Contact + readout extensions (Tier 4 rows 12-14 + row 17 `RigidSdfPrimitive` + row 18 `force/pressure readout`).** Contains the platform extensions Tier 4 + `force/pressure readout` are gated on. Internal-fixture wraps for #12-14, new code for #17-18. Smaller LOC than PR1, larger architectural surface. Bevy real-time rendering convention introduced here (see Visualization convention below).
- **PR3 — Bridge + synthesis (Tier 5 rows 15-16, 19 + Tier 6 row 20).** Architectural: introduces `cf_design::Sdf` trait per Q1+Q5 decisions, migrates `sim_soft::Sdf` to a re-export, lands `mesh_sdf::SignedDistanceField` adapter and `Solid` impl. Plus the silicone-material-table and the layered silicone device E2E example. Heaviest architecturally; the device example may itself be many commits.

**Why this shape over "one big PR":** CI is ~25 min per PR per `feedback_pr_size_ci_economics`; gating PR2/PR3 work on PR1's user-visible review cycle catches museum-plaque issues early instead of shipping 14 examples then re-reading them all under one diff. Per `feedback_one_at_a_time` + `feedback_one_at_a_time_review`, examples are reviewed individually before the next lands — that fits commit segmentation, not PR size.

**Pre-squash tag** per `feedback_pre_squash_tag` for **PR3** since it introduces the `cf_design::Sdf` trait and likely adds book entries for it; tag `feature/sim-soft-examples-arc-pr3-pre-squash` before its squash-merge. PR1 / PR2 likely don't touch any book chapters, so no tag needed.

---

## Visualization convention (PLY/JSON vs Bevy)

> **2026-05-04 SUPERSEDED — pending viewer arc.** PR1 commit 1's visual pass (sphere-sdf-eval via MeshLab) surfaced a gap the PLY-vs-Bevy split below didn't anticipate: **field-over-space content** (signed distance, material tags, stress) has no native viewer in the workspace — MeshLab requires a per-example two-filter chain to render the per-vertex scalar; f3d ignores the scalar entirely. The user separately flagged that f3d was already underwhelming for the mesh-v1.0 examples. The fix is a unified Bevy-CLI viewer (`cf-view <path>`-shaped) that auto-colormaps per-vertex / per-face scalars and replaces both f3d and MeshLab for static-artifact review. **Sim-soft examples arc PAUSED at PR1 commit 1** until the viewer arc lands. New planning doc: [`docs/VIEWER_DESIGN.md`](../../docs/VIEWER_DESIGN.md). The split below is preserved for audit-trail context but is not the operative convention going forward.
>
> **Operative convention going forward (locked at this layer; details in viewer arc):**
> - Static fields + meshes: emit PLY (existing pattern), visual review via the unified Bevy viewer one-liner `cf-view <path>`.
> - Dynamics (Tier 4 + Tier 6): per-example Bevy app, unchanged from below.
> - JSON-only force-stretch / displacement traces (rows 4, 5, 6): unchanged — matplotlib post-hoc, not in viewer scope.

**(Original iter-2 content below — superseded as noted; preserved for audit trail.)**

**Decision: split by tier — static-artifact (PLY/JSON, f3d/MeshLab viewing) for Tier 1-3 + Tier 5; Bevy real-time playback for Tier 4 + Tier 6.**

**Why split rather than pick one:**

- The mesh-v1.0 convention (PLY → f3d) is correct for **static math-pass-first artifacts**. Mesh examples have no dynamics — what you write is what you show. SDF eval, meshing, constitutive stress curves, multi-material assignment, bridges-and-readouts — these are all "did the math right?" examples. PLY + JSON anchors are sufficient and the math-pass-first asserts ARE the correctness signal.
- `feedback_visible_contacts` is a **hard rule**: all contact and collision must be visually rendered. Tier 4's drop-and-rest, Hertz, and compressive-block examples are dynamic contact phenomena. Static post-step PLY snapshots are a degraded experience for those — you lose the *settling* in drop-and-rest, the *contact patch growth* in Hertz, the *compressive descent* in the block. Bevy gives you the dynamics.
- Tier 6 (the silicone device E2E) is the synthesis showcase — it deserves the full visual treatment. Bevy renders the layered body deforming under the scan-derived rigid indenter, with material-tagged colors and contact-pressure readout overlay.
- Tier 5 bridges (rows 15-19) are **plumbing examples** — "this scan loads as a Solid", "this Solid meshes via sim-soft", "this readout returns a force." Static PLY/JSON for those mirrors the mesh-sdf/mesh-measure precedent. The silicone device is where the bridges combine into a Bevy-rendered whole.

**Per-row visualization assignment:**

| Rows | Visualization | Output artifacts | Convention reference |
|---|---|---|---|
| 1, 2, 3, 8, 9, 10, 11, 15, 16, 19 | **PLY + JSON** | boundary-face / per-tet-tagged PLY, force/pressure JSON | `examples/mesh/README.md` template |
| 4, 5, 6 | **JSON only** | force-stretch / displacement JSON traces (no spatial artifact) | math-pass-first per `feedback_math_pass_first_handauthored` |
| 12, 13, 14, 17, 18, 20 | **Bevy real-time** | live deforming-tet-mesh render + headless-mode asserts | `examples/fundamentals/sim-cpu/` precedent |

**Bevy authoring convention:**

- Mirror the `examples/fundamentals/sim-cpu/` shape: a Bevy app that runs headless-mode asserts pre-render (the math-pass-first numerical anchors) AND visual-mode playback in default mode.
- Per `feedback_one_at_a_time_review`'s two-pass cadence: numerical asserts are Claude's pass (cargo run exit-0); visual playback is the user's pass.
- Material-tagged colors for multi-material rows (the layered device's three shells get three distinct colors).
- Contact-pressure visualization for Tier 4: per-vertex pressure as a colormap on the soft-body surface, contact-force arrows where the rigid primitive bites (per `feedback_visible_contacts`).
- `feedback_f3d_winding_callout` is f3d-specific and does not apply to Bevy-rendered examples.

**Cost reality check:**

Bevy examples are ~3-4× heavier per file vs PLY-emit (~200-400 LOC each vs ~50-100). At 6 Bevy examples here, that's ~6 × 300 = ~1.8k LOC of Bevy-app code. CI cost is non-trivial but the mesh-cpu examples at 60+ already absorb this cost; sim-soft adding 6 is incremental. The visual payoff for contact dynamics + the device synthesis is worth the LOC.

**Open follow-up:** does sim-soft need its own `sim-bevy`-style adapter to render deforming tet meshes (boundary faces re-extracted per frame from `x_current`)? Likely yes — author that adapter as part of PR2's first contact example, then reuse across rows 12-14, 17-18, 20. Land it in `sim/L1/sim-bevy/` (or a new `sim-bevy-soft` sibling) rather than baking into each example.

---

### Q1-Q5 status — all five resolved this round (head-architect calls)

1. **Q1 — `mesh-sdf → cf-design` adapter naming**: define `cf_design::Sdf` trait owned by cf-design; `impl cf_design::Sdf for mesh_sdf::SignedDistanceField` lives in cf-design (orphan-safe). No `MeshSdf` newtype — use the real type name everywhere. **Locked, see row 15 gap column.**
2. **Q2 — Row 7 `InversionHandling`**: **fold into row 5**. The Neo-Hookean uniaxial example extends through compressive inversion to show clamp behavior — coherent pedagogy, no separate row. **Locked.**
3. **Q3 — Row 21 forward-map gradcheck**: **defer to a future differentiable-sim arc**. The autograd surface deserves a coordinated arc, not an orphan example. **Locked.**
4. **Q4 — Row 22 RewardBreakdown demo**: **defer indefinitely** — no real `VecEnv`-wrapping consumer for sim-soft today. Re-evaluate if one materializes. **Locked.**
5. **Q5 — `cf_design::Solid` impl-ing `sim_soft::Sdf`**: **`impl cf_design::Sdf for cf_design::Solid` directly** (Solid's existing inherent `evaluate(Point3) -> f64` becomes the trait `eval`; `grad` analytical-per-primitive or central-difference fallback). Sim-soft's `Sdf` trait migrated to a re-export of cf-design's. **Locked, see row 16 gap column.**

---

## Iteration log

_(append session-by-session; date-stamped; what changed and why)_

- **2026-05-04 (seed):** Initial structure + scope + identified gaps. No table rows yet — that's the first iteration's work.
- **2026-05-04 (iter 1 — table draft + scope answers):** Recon read-pass: sim-soft `lib.rs` exports, `examples/mesh/` layout + README template, `tests/` directory shape (35 internal-fixture files), `SoftScene::layered_silicone_sphere`, cf-design `Solid` location, mesh-sdf `SignedDistanceField`. Filled the inventory table with **22 rows** organized into 6 capability tiers (Tier 1 SDF primitives + meshing, Tier 2 constitutive + multi-element, Tier 3 multi-material, Tier 4 penalty contact, Tier 5 bridges + extensions, Tier 6 synthesis), plus 2 `?` rows (forward-map gradcheck, reward breakdown) flagged out-of-scope-pending-user. Answered all five open scope questions: (Q1) ~14 user-facing capability-coverage examples bounded by ≤ 1 per public-API surface, plus the device synthesis row + ~2 `?` rows; (Q2) museum-tour ordering in the table, device-dependency-chain ordering in PR sequencing — these are different artifacts; (Q3) `examples/sim-soft/<name>/` at repo root, mesh-v1.0 convention adopted verbatim; (Q4) mesh-v1.0 README template with sim-soft-specific anchor + visual conventions (math-pass-first applies, FEM analogs for displacement / energy / residual / force-displacement); (Q5) **3 PRs** — PR1 capability-coverage core (Tier 1-3, ~10-11 examples wrapped from internal fixtures, no new machinery), PR2 contact + readout (Tier 4 + rows 17 + 18, RigidSdfPrimitive + force/pressure readout extensions), PR3 bridge + synthesis (Tier 5 rows 15-16-19 + Tier 6 row 20, mesh-sdf → cf-design bridge + silicone-material-table + layered silicone device E2E). Inventory table is now **executable** as a 3-PR plan once the 5 remaining `?` items are resolved.

  **Still open (`?` for user):** (1) naming for `mesh-sdf → cf-design` adapter — `MeshSdf` newtype vs `Solid::from_external_sdf<S: Sdf>` generic vs raw `SignedDistanceField`; (2) keep / drop / fold row 7 `InversionHandling`; (3) include row 21 `forward-map gradcheck` in this arc or defer to differentiable-sim arc; (4) include row 22 `RewardBreakdown` demo or defer until a `VecEnv` consumer exists (my read: defer); (5) `cf_design::Solid` impl-ing `sim_soft::Sdf` directly vs explicit conversion fn — shapes how every later example is authored.

  **Refinements observed but not actioned:** (a) `examples/EXAMPLES.md` is one-PR-behind reality — the mesh-v1.0 examples actually live at `examples/mesh/` not `examples/fundamentals/mesh/`, and that section needs a doc update at first sim-soft-example land; small followup, not blocking. (b) `Phase 5.5` retirement per memory's resume-here is mechanically consistent with this inventory — `RigidSdfPrimitive` (row 17) + pressure readout (row 18) are the two named items folded into PR2 here, and the parked items (Coulomb-Stribeck friction, insertion sweep) do not appear at all.

  **Next iteration entry point:** answer the 5 remaining `?` items above with the user, then re-validate the table by walking each row and confirming the "simplest example" sentence is concrete enough to author from. Math-pass-first numerical anchors should be made explicit per row before any execution.

- **2026-05-04 (iter 2 — head-architect calls + Bevy convention):** User granted head-architect authority on the 5 `?` items per `feedback_master_architect_delegation`. Recon read-pass: `cf_design::Solid` already exposes `evaluate(Point3<f64>) -> f64` as inherent method, no `Sdf` trait exists in cf-design today, so adding one is a clean greenfield decision (not a migration of an existing trait). Decisions locked:

  **Q1 (adapter naming):** `cf_design::Sdf` trait owned by cf-design (`eval(Point3) -> f64` + `grad(Point3) -> Vec3`). `impl cf_design::Sdf for mesh_sdf::SignedDistanceField` lives in cf-design — orphan-rule-safe since the trait is local. No `MeshSdf` newtype, use real type name.

  **Q2 (row 7 InversionHandling):** folded into row 5. Neo-Hookean uniaxial example extends through the compressive-inversion branch to exercise `ValidityDomain` clamping. Row 7 deleted from table.

  **Q3 (row 21 forward-map gradcheck):** deferred to a future differentiable-sim arc. Autograd deserves coordinated multi-example treatment, not an orphan row.

  **Q4 (row 22 RewardBreakdown):** deferred indefinitely until a real sim-soft `VecEnv` consumer materializes.

  **Q5 (Solid → Sdf glue):** `impl cf_design::Sdf for cf_design::Solid` directly. The existing inherent `Solid::evaluate` becomes the trait `eval`; `grad` is analytical-per-primitive or central-difference fallback. **Sim-soft's `Sdf` trait migrated to a re-export of cf-design's.** Breaking change accepted per user's foundational-fix preference. `SphereSdf`/`DifferenceSdf` stay in sim-soft as deprecated test-only primitives until cf-design's `Solid::sphere()`/boolean ops cover their use sites.

  **Bevy rendering convention (new Q6):** split by tier — static PLY/JSON for Tier 1-3 + Tier 5 (10 rows), Bevy real-time for Tier 4 + Tier 6 (6 rows). `feedback_visible_contacts` is the hard requirement that lands Tier 4 in Bevy. Bevy adapter for deforming tet meshes (boundary-face re-extraction per frame) is a sub-task of PR2's first contact example, lands in `sim/L1/sim-bevy/` or a sibling crate, reused across rows 12-14, 17-18, 20. Bevy authoring mirrors `examples/fundamentals/sim-cpu/` shape: headless-mode asserts + visual-mode playback under one `cargo run`.

  **Cascading updates:** PR sequencing in Q5 updated to reflect the new row counts (PR1 = 10 examples, was 11; PR2 unchanged at 5; PR3 = 4, was 5). Row numbering preserved (rows 7, 21, 22 deleted but remaining rows kept their numbers — re-numbering would invalidate the iteration log's row references).

  **Inventory is now executable as a 3-PR plan with all architectural decisions locked.**

  **Next iteration entry point:** before any execution, walk each row's "simplest example" sentence and convert it into a concrete numerical-anchor list (the math-pass-first asserts) — that's the last planning round before PR1 authoring begins. Could be done in this same multi-session arc, or could be the first sub-step of PR1 itself depending on how concrete the user wants the plan vs how much they want to leave to in-PR judgment.

- **2026-05-04 (iter 3 — PR1 commit 1 shipped + arc PAUSED for viewer gap):** PR1 commit 1 shipped at `93b4d5a4` — sphere-sdf-eval (row 1) at `examples/sim-soft/sphere-sdf-eval/`: 7 verify_* anchor groups, 11³ = 1331-point grid, PLY emit with `extras["signed_distance"]`. All asserts green (exit-0); pre-commit clean (fmt + clippy with file-level allows mirroring mesh-sdf-distance-query); cargo doc clean under `-D warnings`. Per-example template now established for PR1 rows 2-11.

  **Visual pass surfaced a load-bearing gap.** MeshLab opened the PLY but required a 2-filter chain (`Per Vertex Quality Function` with `func q = signed_distance` → `Colorize by Vertex Quality`) to render the field. Per-example friction across 10 PR1 rows. User then flagged that f3d had also been underwhelming for the mesh-v1.0 examples — geometry-only viewer, no scalar colormap, no field semantics. The PLY-vs-Bevy split locked in iter 2 conflated two cases: mesh-v1.0's content is geometric (f3d works), sim-soft Tier 1's content is field-over-space (no native viewer in the workspace). The fix is engine-level, not example-level: a unified Bevy-CLI viewer (`cf-view <path>`-shaped) that auto-colormaps per-vertex / per-face scalars and replaces both f3d and MeshLab.

  **Sim-soft examples arc PAUSED at PR1 commit 1.** Per `feedback_fix_gaps_before_continuing` — when examples reveal engine gaps, fix the engine first. Next 9 PR1 rows would compound the friction; better to surface and fix now than 9 examples later.

  **Visualization convention superseded** at the section banner above; operative go-forward convention is locked at the banner level (`cf-view <path>` for static fields + meshes; per-example Bevy unchanged for dynamics; matplotlib unchanged for JSON-only force-stretch traces). Detail decisions deferred to the new arc.

  **New arc opened: unified-viewer architecture.** Seed planning doc at [`docs/VIEWER_DESIGN.md`](../../docs/VIEWER_DESIGN.md). Branch strategy: stays on `feature/sim-soft-examples-arc` for the seed-doc commit (prose only, low conflict); when viewer code starts, branch off main as `feature/cf-viewer-arc` so the viewer ships as its own PR. Sim-soft branch rebases after viewer merges; sphere-sdf-eval's README updates to point at `cf-view <path>` post-rebase. Mesh-v1.0 retrofit to the unified viewer is a separate followup, not gated on the viewer arc itself.

  **Next iteration entry point for the sim-soft arc:** wait for viewer arc to ship; then resume PR1 at row 2 (`hollow-shell-sdf`) using the new viewer. **Next iteration entry point for the viewer arc:** see `docs/VIEWER_DESIGN.md` resume-here block.

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
