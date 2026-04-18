# Testing strategy

`sim-soft` ships with four orthogonal testing strategies. Each one catches a different class of regression; together they form the "A-grade or it doesn't ship" bar translated to soft-body simulation.

| Test kind | What it catches | When it runs |
|---|---|---|
| [Unit tests](04-testing/00-unit.md) | Per-module correctness on small inputs — constitutive law stress values, element stiffness matrices, IPC barrier evaluations, mesh quality metrics | Every `cargo test`, every PR |
| [Regression vs. MuJoCo flex](04-testing/01-regression.md) | Cross-solver drift on shared scenes — Tet4 + neo-Hookean against flex's PBD-on-tets at matched parameters, on scenes both can express | Weekly in CI, and before any merge that touches `solver/`, `element/`, or `contact/` |
| [Visual regression](04-testing/02-visual.md) | Rendering-pipeline drift — deformed mesh topology, per-vertex attribute values, shader output on a small fixed screenshot set | Every PR that touches `readout/` or `sim-bevy` coupling |
| [Differentiability — gradcheck](04-testing/03-gradcheck.md) | Analytic gradients matching finite differences to 5–6 digits on each module and on the full forward + adjoint composition | Every PR that touches `autograd/`, `material/`, `contact/`, or `solver/` |

Two claims Ch 04 rests on:

1. **Gradcheck is the strongest correctness test in the crate.** A solver that passes gradcheck is, by construction, consistent between forward and reverse — the same energy is differentiated analytically and by finite differences, and the two agree. This catches a wider class of bugs than unit tests catch alone; a unit test might check `barrier(d)` but miss a sign error in `d/dd barrier(d)` that gradcheck catches instantly. Gradcheck is therefore required on every PR touching anything on the gradient path, not optional.
2. **MuJoCo flex is the regression baseline, not the fidelity target.** The Track 1B platform coverage for `sim-mjcf` flex ([Part 12 Ch 01](../120-roadmap/01-track-1b.md)) exists so `sim-soft` has an already-working soft-body solver to compare against on the same scenes. The comparison is not "sim-soft should match flex numerically" — they solve different PDEs. The comparison is "where the two disagree, the disagreement should be in the direction the [Part 1 Ch 03 thesis](../10-physical/03-thesis.md) predicts" — less volume loss, less popping, more realistic rim deformation. The regression test fails when the disagreement has the wrong sign, not when the numbers differ.
