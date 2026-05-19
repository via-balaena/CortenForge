# H2 — mesh refinement at cavity surface (foundational fix)

> **STATUS — BOOKMARK.** Per [[feedback-bookmark-when-surface-levers-exhaust]]
> three-session pattern: this is the bookmark.  Recon next session;
> implementation the session after.
>
> 2026-05-19 LATE-NIGHT.  Foundational fix for the indefinite-tangent
> pivot-stagnation pattern surfaced by the N_STEPS resonance recon
> (`b61sdccu3` log at `/tmp/n_steps_recon.log`).  The symptomatic
> N_STEPS-bump-to-20 ships separately in the same session as the small
> win; this bookmark scopes the real foundational fix.
>
> **Predecessor memos**:
> - [[project-h4-arc-falsified]] — H4 arc shipped at cavity 3+5 mm full,
>   8 mm partial; 6+7 mm wall identified as separate sub-arc.
> - [[project-cf-device-design-sim-arc-recon]] — sim-arc parent.
> - `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10.4 — H2 named
>   as the "real product fix" (~500 LOC primitive); banked behind H4.
> - `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5.5 + §5.8 —
>   6+7 mm Newton convergence wall flagged as separate sub-arc.

---

## TL;DR

The 6+7 mm Newton convergence wall under H4-2-C is **path-dependent
local stagnation in indefinite-tangent terrain** at a specific
intruder pose (slide-step = 5.21 mm at N=16, cavity 6+7 mm).  The
recon `n_steps_recon_sweep_at_cavity_6_and_7_mm` showed N=12 and N=20
both converge full at the same cavities — so the wall is a specific
**resonance** between intruder pose and per-vertex stiffness
anisotropy at the cavity wall, not a slide-fraction-monotonic
threshold and not a true material limit.

The pathological signature is a **persistent indefinite eigenmode at
specific dofs** — pivot index 3618 was the non-PD pivot for 25
consecutive Newton iters (51-80); pivot index 2442 was the non-PD
pivot for 50 consecutive iters (92-149).  Newton's LU-fallback
descent navigates around the indefinite directions but can't reduce
the residual below ~0.54 — Levenberg-Marquardt rescues with λ ≈ 1e4
knock the residual down once each but Newton snaps back into the
same basin.

**Why mesh refinement fixes it**: the persistent indefinite mode lives
at one or two cavity-wall vertices whose stiffness contribution is
the dominant source of the indefinite tangent.  Finer mesh
distributes the contact + material stiffness over more vertices, so
no single vertex contribution can dominate the eigenstructure of
the Newton tangent.  The pathological pivot indices "smear out"
across the refined mesh — no single dof index can repeat 25-50× in
a row because the strain is shared across neighbors.

This is the same line of reasoning E.b §10.4 took to recommend H2:
mesh refinement was banked behind H4 as the real product fix.  H4
shipped (cavity 3+5 mm full, 8 mm partial); the cavity 6+7 mm gap
the H4 arc left open is the inflection point for H2.

---

## 1. The diagnosis the recon banked

### 1.1 N-knob exhaustion — every N has a different cavity hole

Three sweeps this session under H4-2-C, iter-1 sock_over_capsule,
dual-layer Ecoflex+Slacker inner / DS20A outer (the user's GUI
default):

| cavity | N=12 | N=16 (current default) | N=20 |
|---|---|---|---|
| 3 mm | 12/12 ✓ | 16/16 ✓ | 20/20 ✓ |
| **5 mm** | **0/12 ✗ step-0 r=0.304** | **16/16 ✓ full (λ_min=0.37)** | **0/20 ✗ step-0 r=0.398** |
| 6 mm | 12/12 ✓ | 0/16 ✗ step-0 r=0.536 | 20/20 ✓ |
| 7 mm | 12/12 ✓ | 0/16 ✗ step-0 r=0.294 | 20/20 ✓ |
| 8 mm | 8/12 (67 %) | 12/16 (75 %) | 20/20 ✓ full |

**No single N covers all 5 cavities.**  Cavity 5 mm is uniquely
served by N=16; cavity 6+7 mm are uniquely lost by N=16.  Each N
has its own pathological cavity hole — the wall position is
cavity-and-N-pose-specific, not a slide-fraction-monotonic
threshold.  Per [[feedback-scan-is-example-not-spec]] a per-cavity
N lookup would be scan-specific tactical (iter-2 will have
different holes); the foundational fix is mesh refinement.

### 1.2 Per-iter Newton trace at the failing cells

From `/tmp/n_steps_recon.log` cavity 6 mm step 0 trace (N=16, the
failing case):

| iter range | r_norm | dominant non-PD pivot index |
|---|---|---|
| 0-15 | 101 → 6.0 | rotating (3273, 2661, 2716, 22921, ...) |
| 15-30 | 6.0 → 1.0 | mixed (2442 starts appearing 4×) |
| 30-50 | 1.0 → 0.85 | **25077 stuck for 8× in a row** |
| 51-80 | 0.85 → 0.68 | **3618 stuck for 25× in a row** |
| 81 | 0.68 | LM rescue λ = 7.9e3 |
| 82-91 | 0.65 → 0.64 | back to mixed (3618, 2443, 2444) |
| 92-149 | 0.64 → 0.54 | **2442 stuck for 50× in a row** |
| 139 | 0.54 | LM rescue λ = 9.9e3 |
| 149 (terminal) | 0.536 | 2442 |

The N=12 and N=20 cells at the same cavity 6 mm have similar pool of
non-PD pivots in the early iters (LU fallback fires every iter) but
**never lock onto a single index for more than a few iters in a
row** — the residual descends smoothly through 22-35 iters and
converges below tol = 1e-1.

The N=16 step-0 intruder pose at cavity 6+7 mm puts ONE OR TWO
specific cavity-wall vertices into a contact + material strain
configuration where their stiffness contribution makes the global
tangent persistently indefinite in a direction nearly orthogonal
to the residual gradient.  Newton can't break out.

## 2. Open design questions

### Q1 — uniform vs adaptive refinement

Two flavors:

- **Uniform**: halve `cell_size_m` globally from 4 mm to 2 mm.
  Quick to implement; 8× tet count + 8× wall-clock.  iter-1 sock has
  72 880 tets at 4 mm; at 2 mm that's ~580 k tets.  Ramp time
  scales worse than linearly (LU factorization is O(n^1.5) for
  sparse PD systems; the indefinite-tangent LU fallback is worse).
  Cavity 6 mm ramp went from ~3 min at 4 mm 16 steps to probably
  ~30-60 min at 2 mm 16 steps.

- **Adaptive**: refine near the cavity surface only (within e.g.
  3 mm of the cavity isosurface).  Cell size 4 mm globally, 2 mm
  near cavity, possibly 1 mm at the cavity boundary itself.
  Better runtime characteristics but requires plumbing variable
  cell size through `SdfMeshedTetMesh` + the BCC mesher in
  `mesh-sdf`.

Recommendation: spike uniform 2 mm first as falsification
("does refinement actually fix the resonance?").  If yes, design
the adaptive primitive properly.  If no, the indefinite-tangent
diagnosis is wrong + we escalate to a deeper solver-side fix.

### Q2 — what does "adaptive" mean concretely?

Options:

- **Distance-banded** — cell size as a step function of SDF distance:
  `cell_size(d) = 4 mm if d > band, else 2 mm` for `band = 5 mm`.
- **Sigmoid-banded** — smooth interpolation between coarse + fine,
  no T-junctions.
- **Octree-style** — recursive subdivision of BCC cells near the
  cavity.  More invasive change to the mesher.

The current `mesh-sdf` BCC mesher is single-cell-size.  Need to
audit whether it supports variable cell size or if this requires a
new primitive.

### Q3 — is the cavity surface the only place that needs refinement?

The recon log shows the persistent indefinite pivot indices (3618,
2442) — but we don't know their physical mesh locations.
Pre-implementation step: instrument `factor_and_solve_free` to print
the world-space position of the non-PD pivot vertex.  If they're
all on the cavity wall near the intruder slide-pose tip, the
distance-banded refinement is sufficient.  If they're scattered
(e.g., some outer-skin vertices, some near cap planes), the
diagnosis is less clean + the refinement strategy needs to be more
nuanced.

### Q4 — what's the regression substrate?

The `n_steps_recon_sweep_at_cavity_6_and_7_mm` test gets deleted
when A ships.  The H2 arc needs a fresh regression substrate that:

- Runs the cavity 6+7 mm sliding ramp at N=16 (the previously
  pathological case) post-refinement
- Asserts the persistent-pivot-index pattern is gone (no single
  non-PD index repeats >5× in a row, or similar)
- Holds the cavity 3+5+8 mm cases as control (must still converge
  post-refinement)

This is a derivative of the h4_sweep test scaffolding; can be
adapted.

## 3. Scope estimate

Per E.b §10.4 the estimate was ~500 LOC mesh-refinement primitive.
Updated estimate after recon:

- **mesh-sdf BCC mesher with variable cell size**: ~300 LOC
  (a new `BccMesherAdaptive` primitive that takes a `cell_size_fn:
  Fn(Vec3) -> f64` instead of a const).  Big chunk of work; the
  cell connectivity logic is the hard part — T-junctions or
  conforming refinement?
- **distance-banded predicate plumbing through cf-device-design**:
  ~100 LOC (a new builder fn `build_insertion_geometry_adaptive`
  that takes a banded refinement spec, calls the new mesher,
  threads the variable-cell-size result through the rest of the
  pipeline).
- **regression substrate adaptation**: ~50 LOC.
- **GUI hookup** (if user-visible): ~50 LOC.

**Total: ~500 LOC**, 2-3 sessions.

If Q1 falsifies uniform 2 mm (mesh refinement doesn't fix the
resonance), this all gets cut + we escalate to deeper solver work.

## 4. Sequencing

Three-session pattern per [[feedback-bookmark-when-surface-levers-exhaust]]:

1. **This bookmark** (session N) — current session, captures the
   diagnosis + design space.
2. **Recon** (session N+1) — Q1 falsification: uniform 2 mm spike
   at cavity 6 mm, observe whether the indefinite-tangent pivot
   stagnation pattern persists.  Cheap test (no new primitive —
   just halve `cell_size_m` in the existing test).  Q3 instrumentation
   to map pivot indices to physical locations.
3. **Implementation** (session N+2 onward) — if recon confirms,
   build the adaptive primitive per Q2.

## 5. What stays banked behind H2

- **Phase H F-bar / mixed-u-p decorator** for the ν=0.40 root cause.
  This is the long-term right answer for compressive accuracy; H2
  is the right answer for the indefinite-tangent resonance.
  Independent arcs.  See [[project-h4-arc-falsified]] "What stayed
  dormant" §Option B for the F-bar arc framing.
- **Per-cavity N selection** — A's bump to N=20 doesn't generalize
  to all future scans.  Could be improved with an auto-fallback on
  step-0 stall (try N+4 if step 0 doesn't converge in 50 iters).
  Banked unless A's N=20 falsifies on a new scan.
- **Modified Cholesky / block preconditioner** in sim-soft —
  bigger sim-soft solver change to better handle indefinite tangents
  natively.  Bigger surface than H2; deferred unless H2 falsifies.

## 6. Pointers

- Recon log: `/tmp/n_steps_recon.log` (captured 2026-05-19
  LATE-NIGHT, retain until H2 ships).
- Predecessor recon: E.b §10.1 had the original pre-H4 N_STEPS
  sweep at cavity 6 mm.  This session's recon is the post-H4-2-C
  version (no fake-convergence artifact).
- Mesh build site: `build_insertion_geometry` at
  `tools/cf-device-design/src/insertion_sim.rs:758-815`.
- Cell size: `SIM_CELL_SIZE_M = 0.004` at
  `tools/cf-device-design/src/insertion_sim_ui.rs:56`.
- Pin band: `0.5 * cell_size_m` — refinement changes this too.
- Non-PD pivot logging: `factor_and_solve_free` in
  `sim/L0/soft/src/solver/backward_euler.rs` (eprintln site).

---

End of bookmark.
