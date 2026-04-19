# Dependency graph — what unlocks what

[Part 11 Ch 03](../110-crate/03-build-order.md) names the module-dependency order: `material/` before `element/` before `solver/` before `contact/`, and so on. This chapter names the same ordering one level up, as the dependency graph *between capabilities* the user of `sim-soft` sees: first-working-solve before GPU-acceleration before SDF-authoring-live before visual-layer-ceiling. Part 11 Ch 03 is the module view; this chapter is the capability view. They agree by construction — this chapter is the coordinating spine that every other chapter in Part 12 keys off, and if a milestone chapter claims a capability is available before its prerequisites on this graph, the claim is the bug.

This is an inherent leaf: the graph is small enough that splitting it across sub-chapters would hide the thing it is trying to show.

## The phase map

The book commits to nine phases, A through I. They are a total order — no overlap, no parallel tracks on the critical path. [Track 1B](01-track-1b.md) runs in parallel but is a coverage-and-regression track, not a capability track, so it does not appear on this graph as a node that unlocks a milestone. Phases A–I are named here by their headline deliverable; [Part 11 Ch 03](../110-crate/03-build-order.md) is the authoritative description of what each phase contains.

| Phase | Headline deliverable | Unlocks |
|---|---|---|
| A | `material/` + `mesh/` with gradcheck | Phase B |
| B | Element + solver on one material (elastic-only, CPU) | Phase C |
| C | IPC contact integrated into solver | Phase D |
| D | Readout + `sim-ml-chassis` boundary closed | **First working sim-soft** ([Ch 02](02-first-working.md)); optimizer phase 1 ([Ch 06](06-optimization.md)) |
| E | GPU port of hot paths | **Interactive rates** ([Ch 03](03-gpu.md)) |
| F | `sim-core` + `sim-thermostat` coupling | Phase G |
| G | SDF bridge + `cf-design` live | **Live design edits** ([Ch 04](04-sdf.md)); optimizer phase 2 ([Ch 06](06-optimization.md)) |
| H | Tet10, mixed u-p, Prony, HGO, adaptive refinement | Feeds Phase I fidelity |
| I | Visual layer via `sim-bevy` | **Visual ceiling** ([Ch 05](05-visual.md)); optimizer phase 3 ([Ch 06](06-optimization.md)) |

## The dependency graph

```text
                    Phase A — material/ + mesh/
                               │
                               ▼
                    Phase B — element + solver (CPU)
                               │
                               ▼
                    Phase C — IPC contact
                               │
                               ▼
                    Phase D — readout + ml-chassis boundary
                               │
                               ▼
              ★ Ch 02 — first working sim-soft
                               │
                               ▼
                    Phase E — GPU port of hot paths
                               │
                               ▼
              ★ Ch 03 — interactive rates
                               │
                               ▼
                    Phase F — sim-core + sim-thermostat
                               │
                               ▼
                    Phase G — SDF bridge + cf-design
                               │
                               ▼
              ★ Ch 04 — live design edits
                               │
                               ▼
                    Phase H — fidelity upgrades
                               │
                               ▼
                    Phase I — sim-bevy visual layer
                               │
                               ▼
              ★ Ch 05 — visual ceiling
                               │
                               ▼
              ────── POST-PHASE-I (not on roadmap) ──────
                               │
                               ▼
          Physical-print loop — see Ch 07 open-questions
```

A second thread runs through Phases D, G, and I: [Ch 06's optimizer](06-optimization.md) matures in three stages rather than landing at a single phase close, so the graph shows it as a thread rather than a node. Phase 1 of the optimizer ships with Phase D, phase 2 with Phase G, phase 3 with Phase I.

## What the graph says

Three observations the graph makes that the phase list alone does not.

**The critical path is a single chain.** No phase can start before its predecessor finishes, and there is no branching inside Phases A–I. This is a deliberate choice, justified at length in [Part 11 Ch 03's "Why this order"](../110-crate/03-build-order.md#why-this-order): each phase produces an artifact the next phase consumes as an oracle for debugging. GPU-first would lose the Phase D CPU solver as a regression baseline; contact-first would lose the Phase B elastic solver as a contact-debugging oracle. The serial structure is not a scheduling artifact — it is how the book's correctness story works.

**Four capability milestones, one optimizer thread.** [Ch 02](02-first-working.md), [Ch 03](03-gpu.md), [Ch 04](04-sdf.md), and [Ch 05](05-visual.md) are the four *capability* milestones — each one unlocks a qualitatively different user experience of the platform. [Ch 06](06-optimization.md) is the *optimizer* milestone, and it matures in three stages across Phases D, G, and I as the underlying capabilities come online. Ch 06 is not a fifth capability; it is the thread that weaves through the other four.

**Track 1B is not on this graph.** The [MuJoCo flex coverage baseline](01-track-1b.md) runs in parallel with Phases A–I from day one, and it does not gate any phase's start or end. Track 1B's deliverable is regression-test infrastructure: every time Phase B, D, or H produces a new `sim-soft` behavior, the flex baseline regression-tests against it. [Ch 01](01-track-1b.md) names what that means concretely.

**The post-Phase-I physical-print loop is not on this graph.** The [full design-print-rate loop](../100-optimization/06-full-loop.md) from Part 10 Ch 06 needs a physical printer, sensors, and a measurement protocol that the Phase A–I roadmap does not commit to acquiring. It is named here and in [Ch 07](07-open-questions.md) as a post-Phase-I deliverable; it is the highest-priority off-roadmap item, but it is off-roadmap. Inventing a new phase letter for it would misrepresent what the book commits to.

## Cross-cutting threads

Several commitments thread across phases without being confined to any one of them. They are called out because they are easy to forget when thinking about the graph as a sequence of headline deliverables.

**Gradcheck is continuous, not a phase gate.** The [gradcheck suite](../110-crate/04-testing/03-gradcheck.md) runs on every PR from Phase A onward. Every module that adds to the autograd surface extends the gradcheck suite before its phase closes. This is why the graph does not show gradcheck as a node: it is an axis running through every node.

**The faer factor-on-tape pattern is installed Phase B, re-used Phases C–I.** [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) and [Part 6 Ch 02](../60-differentiability/02-implicit-function.md) both commit to this pattern; [Part 8 Ch 02](../80-gpu/02-sparse-solvers.md) extends it to GPU with preconditioner + warm-start as the tape artifact. The pattern is established by Phase B and paid off by every subsequent phase. Removing it would invalidate the cost model in [Part 10 Ch 00](../100-optimization/00-forward.md).

**The `GradientEstimate::Noisy { variance }` flag is exposed Phase D, populated Phases D–I.** [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md), [Part 7 Ch 03](../70-sdf-pipeline/03-adaptive-refine.md), [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md), and [Part 5 Ch 02](../50-time-integration/02-adaptive-dt.md) all contribute events that raise it. The flag itself is part of the [ForwardMap trait](../100-optimization/00-forward.md) contract from Phase D; the events are wired in as their respective modules land.

**The `EditResult` taxonomy is exposed Phase G, used Phases G–I.** `ParameterOnly`, `MaterialChanging`, and `TopologyChanging` are the three classes from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md). Phase G wires them up; Phases H and I extend the table of wall-time budgets per class, not the taxonomy itself.

## Reading this graph against Part 11 Ch 03

[Part 11 Ch 03](../110-crate/03-build-order.md) closes by naming this chapter as its dual: it is the *module* view of the same sequence this chapter presents as the *capability* view. A module chart reads bottom-up (what code compiles before what); a capability chart reads top-down (what user-visible deliverable unlocks before what). A disagreement between the two — for example, a capability appearing here before the modules it needs compile there — is always a bug, either in this chapter or in Part 11 Ch 03.

The rest of Part 12 expands each node in the graph. [Ch 01](01-track-1b.md) covers the parallel regression track; [Ch 02](02-first-working.md)–[Ch 05](05-visual.md) cover the four capability milestones in order; [Ch 06](06-optimization.md) covers the optimizer thread that runs through them; [Ch 07](07-open-questions.md) names what is off the graph.
