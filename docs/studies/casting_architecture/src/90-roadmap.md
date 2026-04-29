# Roadmap and open questions

What this part covers (when authored to depth):

## Phased build plan

Mirroring the soft-body architecture book's phase pattern. Each phase is a foundation for the next; each ships independently with its own walking-skeleton invariants.

- **Phase A — Walking skeleton.** Single concrete pipeline: `SphereSdf` → two-piece planar-parted mold STL. Walking-skeleton invariants (closed cavity, correct SDF inversion, mold STL passes basic mesh validity). No multi-shot, no manufacturability checks. Lands the crate.
- **Phase B — Mold geometry depth.** Pour ports, vents, cores. Demoldability check for planar parting. Wall-thickness analysis. Layered silicone device's outer-shell mold becomes castable in isolation.
- **Phase C — Multi-shot sequencing.** Sequencing logic for nested shells. Inter-shot adhesion strategies. Layered silicone device's full three-shot sequence becomes specifiable. Output: a casting plan a human can execute.
- **Phase D — Forward manufacturability.** `ForwardManufacturability` API wires manufacturability into the design loop's `RewardBreakdown`. Manufacturability gradients via FD wrapper. The optimizer can now route designs away from un-castable regions.
- **Phase E — Differentiability deepening.** Analytic gradients for the smooth manufacturability terms (wall thickness, draft, cost). FD wrapper retained for demoldability. Sim-to-real residual GP for cast-side bias correction.
- **Phase F — Free-form parting.** Algorithmic parting-surface generation for non-planar geometries. Research-frontier work — probably runs in parallel to phases C–E rather than blocking them.
- **Phase G — Output ecosystem.** STEP / 3MF / JSON / Markdown output formats. Bill of materials. Inspection plans. Whatever the outer-loop measurement workflow needs.

## Dependencies

Most phases stack linearly. Notable cross-domain dependencies:

- Phase D (forward manufacturability) requires the soft-body book's Phase G (residual GP infrastructure) to be present, since they share the residual machinery.
- Phase E (differentiability) requires the soft-body book's Phase E (chassis GPU port + multi-step adjoint) to be useful at scale — a CPU-only differentiable manufacturability path works for the bench cases but doesn't compose with the optimizer at production rates.
- Phase F (free-form parting) is genuinely independent — it's a self-contained research problem and can land anytime after Phase A.

## Open questions

The big ones, in priority order:

1. **Differentiable demoldability.** Pass/fail analysis is discrete; the optimizer wants gradients. FD wrapper is the pragmatic baseline; analytic relaxations are open research. Closely related to the soft-body book's [Part 6 Ch 05 differentiable meshing](../../soft_body_architecture/src/60-differentiability/05-diff-meshing.md) problem.
2. **Free-form parting surface generation.** Manual today (commercial tools, designer judgment). Algorithmic candidates exist (silhouette extraction, convex decomposition) but none are production-quality. Probably the casting domain's hardest research problem.
3. **Cross-domain reward composition.** When physics reward and manufacturability reward conflict (a soft, compliant geometry that's hard to demold vs. a stiffer geometry that demolds cleanly), the design loop has to navigate the Pareto front. Hand-tuned weights vs. preference-learned vs. explicit Pareto optimization. Probably preference-learned, but the experiments to confirm this haven't been run.
4. **Cast-side material database curation.** Same as the soft-body book's material data part. The two databases have to stay synchronized — Ecoflex 00-30's cast-side properties (working time, demold time, shrinkage) AND its sim-side properties (Lamé parameters, viscoelastic spectrum) come from the same datasheet and should be co-located. Implementation question, not a research one.
5. **Multi-shot cure-clock modeling.** Each shot has its own cure clock; the system has overlapping clocks during a sequence. Modeling this affects both the casting plan (when can shot N+1 start?) and the simulator (does shot N's cure-state matter for shot N+1's interface mechanics?). Probably overkill for Phase A–C; matters for Phase D+ when the full design loop closes.

## What this study is not

- Not a CAM toolchain. The output is mold geometry + a casting plan; the actual fab is somebody else's problem. No CNC g-code, no FDM slicing.
- Not a manufacturing-execution system. The casting plan tells a human (or robot) what to do; it doesn't drive a fab line.
- Not an injection-molding study. Different fab path with different design constraints; deserves its own study if CortenForge ever needs it.
- Not the only fab path. The soft-body book's full loop already references "the printer" generically — for additive paths, [active learning](../../soft_body_architecture/src/100-optimization/04-active-learning.md) covers what to print next without needing this book's machinery.

## Where this lands in the larger CortenForge story

Casting is the second domain CortenForge takes seriously after soft-body simulation. The thesis: simulation and manufacturing are not separate concerns — they're two consumers of the same SDF + `MaterialField` design surface, and the optimization loop that closes them produces designs neither domain could reach in isolation.

If this book's depth pass demonstrates that, the platform's "design → simulate → manufacture → measure → improve" loop is real, end-to-end, in code. Whether it produces parts the world cares about is a separate question — but the instrumentation will be there to find out.
