# Track 1B — MuJoCo flex platform coverage baseline

The book has so far treated `sim-soft` as the only soft-body solver the project ships. That is not quite accurate. A second soft-body capability — MuJoCo's `flex` module — is integrated into the platform as *coverage-and-regression infrastructure*, not as a production solver. This chapter names what that second track is, why the book keeps it at arm's length from the primary capability narrative, and what the deliverable looks like.

Track 1B runs in parallel with Phases A–I from day one. It does not gate any `sim-soft` phase's start or end, and it does not appear as a node on [Ch 00's dependency graph](00-dependencies.md).

## What MuJoCo flex is

`flex` is MuJoCo's beta soft-body module, introduced in MuJoCo 3.0 and still flagged as experimental. It is a tet-mesh-based deformable model built on MuJoCo's existing solver infrastructure — roughly 2,800 LOC of C, using MuJoCo's sparse Newton and constraint-projection machinery, with a simplified elastic constitutive model and a penalty-style contact formulation. It is not IPC, it is not fully differentiable in the sense [Part 6](../60-differentiability/00-what-autograd-needs.md) requires, and it does not carry SDF-valued material fields. It *is* shipped, tested, and maintained by the MuJoCo team, and it runs today.

## Why coverage-baseline and not wrapper

The scoping decision that opened the study committed to building `sim-soft` fresh rather than wrapping `flex`. The reasons live across Parts 1–9 but summarize to three: `flex` is a penalty-contact solver ([Part 4 Ch 00](../40-contact/00-why-ipc.md) rejects that), `flex` does not expose the autograd tape `sim-soft` needs (Part 6 depends on it), and `flex` does not carry SDF-valued material fields ([Part 7 Ch 00](../70-sdf-pipeline/00-sdf-primitive.md) requires them). Wrapping `flex` would force the book's commitments into an adapter layer and lose at least one commitment per feature.

But `flex` is useful as a *coverage baseline*: the platform's coverage matrix should include every soft-body solver the broader ecosystem ships, so that platform-level infrastructure — mesh I/O, material-spec parsing, contact visualization, the `sim-bevy` rendering pipeline — is exercised against more than one backend. A feature that works only against `sim-soft` and breaks against `flex` is more likely to be wrapping a `sim-soft` peculiarity than to be solving a generic platform concern. Track 1B exists to enforce the generic-platform discipline.

## What Track 1B ships

Track 1B's deliverable is a single `examples/` entry plus a regression-test harness. Concretely:

- **A canonical-problem example running against `flex`.** The same SDF-authored compliant-cavity-and-probe geometry from [Part 1 Ch 00](../10-physical/00-canonical.md), meshed and run through MuJoCo's `flex` solver with the closest approximation of `sim-soft`'s material and contact parameters. Rendered via `sim-bevy`, reward-readout via `readout/`.
- **A regression test comparing `flex` and `sim-soft` on a shared small-scale problem.** On a ~1k-tet elastic-only benchmark with no contact, `sim-soft`'s Phase B deliverable should agree with `flex` to within the residual tolerance both solvers target — roughly 1% on scalar reward, looser on vector fields because the element choices may not coincide. Disagreement outside the tolerance is a bug in one solver or the other that warrants investigation; agreement is evidence that `sim-soft`'s baseline is not wildly wrong.
- **A documented "what flex cannot do" table.** Each `sim-soft` capability that exceeds `flex`'s reach — IPC contact without parameter tuning, exact IFT gradients, SDF-field materials, multi-layer viscoelasticity — is named against its phase landing. The table is how a reader arriving via `flex` learns what the extra `sim-soft` machinery is buying.

## What Track 1B does not ship

- **A production workflow through `flex`.** The example exists for coverage, not to invite users to build designs against `flex` as though it were the production stack. A user who wants IPC, differentiability, or SDF authoring is pointed at `sim-soft` — and during Phases A–D, is told to wait.
- **A translator that maps `sim-soft` designs onto `flex` inputs.** The regression harness uses a manually-constructed `flex` model matched to a specific `sim-soft` configuration. An automatic translator would force compromises across the very commitments `sim-soft` was built fresh to honor.
- **Fidelity parity.** The regression harness holds `flex` and `sim-soft` to the same answer only on the narrow shared regime where both solvers sit inside their validity regions. Outside that — large strain, near-incompressible regime, high contact pressure — `sim-soft` is expected to diverge from `flex`, and the divergence direction is expected to be `sim-soft` → measured reality, `flex` → artifact from [Part 1 Ch 02](../10-physical/02-what-goes-wrong.md).

## Relationship to `sim-soft` phases

Track 1B ticks along with Phase A through Phase I without gating any of them. Three explicit touchpoints:

- **Phase B landing.** The first regression comparison fires when `sim-soft`'s Phase B elastic-only solver exists, against `flex` on the same elastic-only problem.
- **Phase H landing.** The regression table grows to cover [near-incompressibility](../20-materials/05-incompressibility.md) and [viscoelasticity](../20-materials/07-viscoelastic.md) as `sim-soft` adds them; `flex` often has no counterpart, and those rows are marked "N/A in flex" in the shared table — that is itself part of the coverage signal.
- **Phase I landing.** The visual layer is exercised against `flex` output as well as `sim-soft` output; the [`sim-bevy` shader pipeline](../90-visual/05-sim-bevy.md) should read per-vertex attributes from either backend without change. A `sim-bevy` feature that only works with `sim-soft` is flagged as a platform regression, not a feature.

Track 1B's purpose across all three touchpoints is the same: to force the platform's non-`sim-soft` layers — mesh I/O, material parsing, visualization, coverage infrastructure — to remain backend-agnostic. That discipline is cheap when installed early and expensive to retrofit. Track 1B is the installation.
