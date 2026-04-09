# 2026-04-09 (part 13) — Q5: cf-design end-to-end differentiability — RESOLVED (NO, with caveats)

> Extracted from `MASTER_PLAN.md` §7 part 13 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Doc review M3 (part 12) escalated Q5 from "deferred
  to Phase 5" to active foreground recon, on the asymmetric-risk
  argument: discovering "no" after months of Phase 2-4 commitment
  is exactly the avoidable surprise sharpen-the-axe forbids. This
  is the recon that closes the question. Recon scope was named
  in part 12: cf-design autograd integration, SDF boolean
  differentiability, marching cubes vs smooth alternative, the
  cf-design Phase 5 spec, and existing end-to-end gradient tests.
- **Verdict**: **NO**. The cf-design → sim-core parameter
  pipeline is *not* end-to-end differentiable. It is differentiable
  *up to* the SDF field level (analytically) and *no further*.
  Three concrete breaks downstream. The shipping Phase 5 design-
  through-physics optimization uses **finite differences over the
  full pipeline**, and the documentation says so explicitly.
- **What works** — analytic ∂f/∂θ at the SDF level:
  - `cf-design/src/param.rs` defines `ParamStore` (Arc-shared
    `RwLock<Vec<f64>>`) and `ParamRef` for naming scalar design
    variables. ~216 LOC, zero ML deps. Plain mutable f64 store.
  - `cf-design/src/param_gradient.rs` (~273 LOC) implements
    hand-rolled forward-mode AD over the `FieldNode` enum.
    Smooth booleans (`SmoothUnion`, `SmoothSubtract`,
    `SmoothIntersect`, `SmoothUnionAll`) carry full analytic
    chain-rule derivatives in θ, including the softmax-weighted
    `∂f/∂k` for the variable blend radius. Hard booleans
    (`Union`, `Subtract`, `Intersect`) follow the active child
    (correct subgradient, undefined exactly at the seam).
    Transforms (translate/rotate/scale/mirror/twist/bend/
    repeat) propagate via inverse pull-back. Tested against
    centered FD oracle to ε=1e-6 across 8+ unit tests
    (`param_gradient.rs:294-end`).
  - `cf-design/src/gradient.rs` (separate file) implements
    `∇f(p)` — the *spatial* gradient w.r.t. (x,y,z), used by
    dual contouring for vertex placement. Distinct from
    `param_gradient`; both are analytic, both forward-mode.
- **First break** — mesh extraction is topologically discrete:
  - `cf-design/src/mesher.rs:1` — "Marching cubes mesher with
    interval arithmetic pruning." Standard MC over a regular
    voxel grid; vertex/edge/face count changes discretely as θ
    crosses critical values. No smooth surrogate.
  - `cf-design/src/dual_contouring.rs:1-7` — manifold dual
    contouring with QEF vertex placement on the same regular
    voxel grid. Sharper features than MC, but still discrete
    sign-change topology. `adaptive_dc.rs` (1286 LOC) is the
    octree-adaptive variant — same fundamental break.
  - There is no differentiable marching cubes (DiffMC), no
    neural implicit surface extractor, no soft-occupancy mesher
    in the codebase. The cf-geometry::IndexedMesh that comes
    out the other side has no notion of θ-derivatives.
- **Second break** — mass properties are unwired:
  - `cf-design/src/mechanism/mass.rs:1-22` — `mass_properties`
    integrates `density · interior_indicator` over a regular
    grid with linear-fraction boundary weighting. Returns plain
    `f64` mass + `Point3<f64>` COM + `[f64; 6]` inertia tensor.
    No `param_gradient` propagation. The chain rule *could* be
    applied here in principle (the boundary integral has a
    well-defined θ-derivative via the divergence theorem on the
    smooth field), but it currently isn't.
  - This is the **fixable** break. Doable in a few hundred LOC,
    independent of the mesh-extraction question, would unlock
    analytic ∂(mass, COM, inertia)/∂θ. The Phase 5 spec already
    flagged it: `optim.rs:9-19` calls out "future hybrid
    approaches (analytic mass/inertia gradients + FD for the
    sim boundary)."
- **Third break** — the sim-core forward step is opaque to AD:
  - sim-core's `mj_step` is a hand-coded forward integrator with
    contact, constraints, and implicit damping. No autograd
    handshake. ml-bridge's autograd never touches `Model`/`Data`
    fields — it operates on f64 scalars in policy networks
    (`sim/L0/ml-bridge/src/autograd.rs`).
  - This is the same problem MuJoCo has been chasing for ~5
    years (MuJoCo MJX is the JAX port that solves it by
    rewriting the entire forward step in XLA). No shortcut.
- **The two AD systems are completely disconnected**:
  - `cf-design/Cargo.toml` does NOT depend on `sim-ml-bridge`.
  - `sim-ml-bridge/Cargo.toml` does NOT depend on `cf-design`.
  - Verified by direct read and by `grep`-ing for `cf_design`,
    `ParamStore`, `FieldNode` across `sim/L0/ml-bridge/src/` —
    zero matches.
  - cf-design's `param_gradient` is **forward-mode** analytic AD
    over a hand-rolled `FieldNode` enum (~273 LOC, scalar f64).
  - sim-ml-bridge's `autograd::Tape` is **reverse-mode** tape AD
    over scalar f64 ops (`add/sub/mul/neg/tanh/relu/square/ln/
    exp` + `affine/sum/mean`), explicitly RL-policy-oriented
    ("CPU f64 only," "no GPU tensors," "no batched tape").
  - There is no shared `Variable`/`Tensor` type, no shared scalar
    abstraction, and the two engines use opposite gradient
    styles. Fusing them is not a one-day port.
- **The shipping "differentiable design optimization"**:
  - `cf-design/src/optim.rs` is `minimize_fd` — centered finite-
    difference gradient descent over a user-provided objective
    closure. Lines 9-19 of the docstring are the smoking gun:
    > "For design-through-physics optimization, the end-to-end
    > gradient is `∂J/∂θ where θ → SDF(θ) → mesh → Model →
    > simulate → x_T → J(x_T)`. The middle of this chain
    > (`mesh → Model`) is not analytically differentiable, so
    > we use finite differences over the full pipeline.
    > Session 25's analytic `∂f/∂θ` enables future hybrid
    > approaches."
  - The Phase 5 integration test
    (`mechanism/integration.rs:328-457`,
    `phase5_parameterized_grasp_optimization`) drives a
    parameterized sphere onto a ground plane, measures contact
    force, and uses `minimize_fd` to find the radius that
    maximizes it. **Three** FD iterations, learning rate `1e-9`,
    `fd_eps=0.5`. End-to-end FD, full pipeline rebuild every
    eval (mechanism rebuild → MJCF emit → `sim_mjcf::load_model`
    → 500 sim steps → contact force sum → return scalar). No
    analytic gradients exercised anywhere in the test.
  - **There is no test, anywhere in the workspace, that
    exercises an end-to-end analytical θ-gradient from cf-design
    through sim-core**. `param_gradient` is exercised only
    against the field value itself, not against any downstream
    physics quantity.
- **The "Phase 5 = differentiable design optimization" framing
  is overstated**: Both the project-memory entry ("cf-design
  Phases 1–5 complete (including differentiable design
  optimization)") and §3 of this master plan ("cf-design
  (Phases 1–5 complete) with implicit surface composition,
  mechanism library, and **differentiable design optimization**")
  describe Phase 5 as if the gradient pipeline closes. It does
  not. The honest description is: **finite-difference design
  optimization through simulation, with analytic ∂f/∂θ at the
  SDF field level as a future hybrid lever.** This is a memory/
  doc nit, not a chassis nit; flagging here for future reference.
- **What this means for the build order** (the reason Q5 was
  escalated):
  - **Phases 1-4 are unaffected.** The Langevin thermostat
    chassis lives entirely on the sim-core side. None of the
    seven chassis decisions touches cf-design or its parameter
    pipeline. Phase 1 spec drafting can proceed unchanged.
  - **D3 (co-design) drops several rungs.** D3 was priority 3
    on the Research Directions ladder, ahead of D2 (stochastic
    resonance) and D4 (sim-to-real on a printed device). With
    the Q5 NO, D3 requires *either* a multi-month differentiable-
    forward-step research thread, *or* a surrogate-model
    workaround that crosses a fidelity boundary. Neither is
    sharpen-the-axe-cheap. **Recommend reshuffling**: D1 → D2
    → D4 → D3 → D5. D3 stays on the roadmap as the long-horizon
    "after we can differentiate through `mj_step`" item, not as
    the headline experiment.
  - **D1, D2, D4 are unblocked.** None of them require end-to-
    end gradients through cf-design. D1 (Brownian motor) and
    D2 (stochastic resonance) are pure sim experiments with
    fixed geometry, no design loop. D4 (sim-to-real on printed
    device) needs the design → print path, but the *training*
    happens via RL (sim-ml-bridge already shipped) on a fixed
    parameterization — no cf-design gradients required for the
    EBM training itself, only for *post-hoc* design refinement
    which is exactly what `minimize_fd` already does.
  - **The fixable second break (mass properties) is the cheapest
    capability gain.** Wiring `param_gradient` through
    `mass_properties` would give analytic ∂(mass, COM, inertia)/
    ∂θ for free, which is the part of the Model that benefits
    most from gradients (mass and inertia drive the dynamics
    linearly via `M` and `M⁻¹`). This is *not* on the thermo
    line's critical path, but it's worth flagging upstream to
    the cf-design team as a small, well-scoped follow-on.
- **Plausible reactions revisited** (from the Q5 entry in §5):
  1. **Build the differentiable layer earlier** — large surface
     area: differentiable mesh extraction (DiffMC or smooth
     surrogate) AND a differentiable forward step. Multi-month.
     Not recommended pre-Phase-1.
  2. **Re-prioritize D3 down the ladder** — RECOMMENDED. Cheap,
     consistent with sharpen-the-axe ("foundations over
     scaffolding"), preserves option to revisit later.
  3. **Surrogate model** — viable for D3 specifically, crosses a
     fidelity boundary. Defer the decision until D2/D4 results
     show whether D3's headline framing is still load-bearing.
  4. **Sibling crate wraps cf-design with custom autograd** —
     same surface area as option 1, just packaged differently.
     Not a free win.
- **Recon scope coverage** (from part 12 M3):
  - [x] cf-design autograd integration point — does not exist
    (no sim-ml-bridge dep); custom forward AD lives in
    `param_gradient.rs`.
  - [x] cf-design Phase 5 spec — `optim.rs` docstring is the
    authoritative description; no separate spec file
    (`docs/SDF_NATIVE_PHYSICS_SPEC.md` is the only doc and is
    about a different topic).
  - [x] SDF boolean differentiability — smooth booleans fully
    analytic; hard booleans subgradient-correct away from the
    seam.
  - [x] Mesh extraction — marching cubes + dual contouring,
    both topologically discrete, no smooth alternative.
  - [x] End-to-end gradient tests — none. `param_gradient` is
    tested against FD oracle on field values; the Phase 5
    integration test uses `minimize_fd` (full-pipeline FD)
    with no analytic gradient exercised.
- **Time spent**: ~30 minutes of focused reading. The answer
  was visible in the first file (`optim.rs:9-19`) but warranted
  the corroborating recon across `param_gradient.rs`,
  `mesher.rs`, `dual_contouring.rs`, `mass.rs`,
  `integration.rs`, both Cargo.toml files, and
  `ml-bridge/src/autograd.rs`. Asymmetric-risk argument from
  part 12 was correct: a half-day's recon would have been
  worth it even on a "yes," and the actual answer is "no" with
  build-order consequences.
- **What did NOT happen this session**: no code, no chassis
  edits, no Phase 1 spec drafting, no priority-ladder edit in
  §2.4 (the recommendation above is a recommendation, not a
  commit — pending user confirmation). The recon log entry is
  the only artifact.
- **Open follow-ons**:
  - Update §3 "Q5 status" line from "ESCALATED" to "RESOLVED
    (NO)" with a one-sentence summary pointer to part 13.
  - Update §5 Q5 entry to reflect resolution and link to this
    part.
  - Decide whether to reshuffle the §2.4 priority ladder
    (D1 → D2 → D4 → D3 → D5 recommended).
  - Decide whether to soften the "differentiable design
    optimization" phrasing in §3 and in the cf-design memory
    entry (`project_thermo_computing.md`, `MEMORY.md`) to
    "finite-difference design optimization with analytic
    ∂f/∂θ at the SDF level."
  - Optional cross-team flag: wiring `param_gradient` through
    `mass_properties` is a small, well-scoped cf-design follow-
    on that would unlock analytic mass/inertia gradients.
    Independent of the thermo line's critical path; surface to
    cf-design when convenient.
