# H2 — recon outcomes (Q1 + Q3)

> **STATUS — IN PROGRESS.** Recon session per
> [[feedback-bookmark-when-surface-levers-exhaust]] three-session
> pattern, building on `docs/H2_MESH_REFINEMENT_BOOKMARK.md`.
>
> 2026-05-19 LATE-NIGHT — Q3 instrumentation + 4 mm baseline
> complete; Q1 2 mm spike running.
>
> Predecessors:
> - `docs/H2_MESH_REFINEMENT_BOOKMARK.md` — recon design + Q1-Q4 framing
> - `/tmp/n_steps_recon.log` — pre-recon N_STEPS exhaustion log
> - `/tmp/h2_recon_4mm.log` — Q3 baseline (this session)
> - `/tmp/h2_recon_2mm.log` — Q1 spike (this session, running)

---

## 0. Diagnostic instrumentation

Added an `x_curr: &[f64]` thread-through from
`try_gated_factor_solve_armijo` down to `try_lu_fallback` (and the
IFT-adjoint path `try_factor_at_position` → `try_factor_free_tangent`)
in `sim/L0/soft/src/solver/backward_euler.rs`. After each existing
`"sim-soft: faer LU fallback fired ..."` line, the fallback decodes
`pivot_free_idx → free_dof_indices[pivot_free_idx] → vertex_id` and
emits a follow-up:

```
└─ H2-Q3: pivot vertex_id=NNN (comp=x|y|z, full_dof=MMM), world pos (X, Y, Z) mm
```

All diagnostic sites are marked `H2-Q3 DIAG` for clean revert. The
`step` panicking path (`lu_fallback`, `factor_free_tangent`,
`factor_and_solve_free`) is **not** instrumented — the Newton driver
goes through `try_step` exclusively in this recon.

The recon test `h2_recon_cavity_6mm_n16` (cf-device-design, gated
behind `#[ignore]`) mirrors `h4_sweep_sliding_ramp_on_iter1_scan`
single-celled at cavity=6 mm, N=16, dual-layer Ecoflex+Slacker inner
10 mm + DS20A outer 3 mm. `cell_size_m` is overridable via env var
`CF_DEVICE_DESIGN_H2_CELL_SIZE_MM` (default 4.0 for Q3 baseline; set
to 2.0 for Q1 spike).

---

## 1. Q3 — pivot vertex position map (4 mm baseline)

### 1.1 Where the persistent pivots live

`/tmp/h2_recon_4mm.log` — 72 862 tets / 52 286 vertices / 210 s
wall-clock; step 0 stalls at terminal r = 0.536 after Newton iter
cap 150.

**Pivot-vertex frequency** (150 LU fallback events at step 0):

| count | vertex_id | world pos (mm)              | cyl. radius r (mm) | z (mm) |
|------:|----------:|-----------------------------|-------------------:|-------:|
|    72 |     5114  | ( 8.00, −27.99, −36.01)     |  29.1              |  −36   |
|    32 |     6645  | (31.99,  16.02, −28.04)     |  35.8              |  −28   |
|    13 |    40687  | ( 2.00, −34.00, +38.00)     |  34.1              |  +38   |
|     4 |     6111  | (−3.99,  32.02, −32.44)     |  32.3              |  −32   |
|     4 |     5354  | (−32.02, 11.98, −35.98)     |  34.2              |  −36   |
|     3 |    38615  | (...)                       |  ~34               |  ~+38  |
|     2 |     7956 / 6709 / 5135 / 37610 | various   |  ~30               |  varies |

Top two pivots account for **104 / 150 = 69 %** of all non-PD events.

### 1.2 Consecutive-iter bursts (the real signal)

Beyond raw frequency, the **consecutive-iter runs** are diagnostic of
which pivot is locked into the indefinite eigenmode:

| iter range | pivot       | length | what's happening                            |
|-----------:|------------:|-------:|---------------------------------------------|
|   0 – 14   | rotating    |    —   | residual descends 101 → ~30, mixed pool     |
|  15 – 18   | **v5114**   |   4×   | first persistent appearance                 |
|  30 – 34   | **v40687**  |   5×   |                                             |
|  45 – 47   | **v40687**  |   3×   |                                             |
|  52 – 83   | **v6645**   | **32×**| r descends 1.0 → ~0.68 then plateaus        |
|  84 – 91   | **v5114**   |   8×   |                                             |
| **93 – 149**| **v5114**  |**57×** | r STALLS at 0.536 (terminal)                |

**Terminal stall is locked into v5114's local indefinite eigenmode** —
57 consecutive Newton iters with pivot v5114 dominating, residual
floor 0.536 (5.36× tolerance), LU fallback descends a step each iter
but never escapes v5114's basin.

### 1.3 Spatial geometry of the persistent pivots

Cleaned-scan iter-1 sock_over_capsule geometry:
- Cap plane (z ≈ −53 mm), centerline runs cap → apex (z ≈ +70 mm)
- Body OD r ≈ 30 mm typical
- Cavity inset 6 mm — cavity wall at r ≈ 24 mm (estimate)
- Inner layer (Ecoflex+Slacker): r ≈ 24 → 34 mm
- Outer layer (DS20A): r ≈ 34 → 37 mm

The dominant pivots sit at **r = 29 – 36 mm** — that's the **inner
layer (cavity-region) band**, NOT the outer skin. They cluster in
the **lower body half** (z = −28 to −36 mm), ~17 – 25 mm above the
cap plane. v40687 + sibling pivots at z = +38 mm form a second
much-smaller cluster at the top of the body (intruder-tip-side of
the contact patch).

Azimuthally the pivots are dispersed — v5114 at azimuth ~285°,
v6645 at ~27°, v5354 at ~160°, v6111 at ~97°, v40687 at ~273°.
**Not a single local feature** — multiple cavity-wall vertices in
the lower-body band have the same pathological stiffness
configuration. The body-axis sliding pose at step 0 (slide arc
5.21 mm, intruder advancing into cavity from the cap end) loads
the lower-body cavity wall in a rotationally near-symmetric way;
each azimuth produces its own pathological-stiffness vertex.

### 1.4 Q3 verdict — bookmark hypothesis CONFIRMED

The H2 bookmark §1 hypothesis was: "the persistent indefinite mode
lives at one or two cavity-wall vertices whose stiffness contribution
is the dominant source of the indefinite tangent." Q3 data
**confirms** this:

- Two vertices (v5114, v6645) own 69 % of non-PD pivots
- v5114 owns the terminal stall (57× consecutive run; residual frozen
  at 0.536)
- All dominant pivots are in the **cavity-wall band** (r = 29 – 36 mm)
  and clustered in the **lower body half** (z = −28 to −36 mm)
- Pivots are dispersed azimuthally → not a single local mesh defect,
  but a characteristic-band failure mode

**Refinement strategy implication for Q2**: distance-banded
refinement near the cavity surface should be sufficient. The
pathological pivots are characterized by cavity proximity (small
SDF-to-cavity distance), not by a specific geometric feature that
would require octree-style local refinement. A `cell_size(d) = 4 mm
if d > band else 2 mm` step function with `band ≈ 5 mm` from the
cavity SDF zero-crossing should cover the dominant-pivot population.

The lower-body z-clustering is a CONSEQUENCE of the slide-step 0
intruder pose (intruder entering from the cap end). At later
slide-steps the contact loading would shift azimuthally + axially;
distance-banded refinement keeps the geometric property invariant
(cavity-surface-adjacent vertices are refined regardless of which
ones are currently loaded).

---

## 2. Q1 — uniform 2 mm cell-size falsification spike

### 2.1 Setup

Same recon test, `CF_DEVICE_DESIGN_H2_CELL_SIZE_MM=2.0`. Expected
~8× tet count (~580k tets), wall-clock 30 – 60 min.

### 2.2 Outcome

_Pending — log streaming to `/tmp/h2_recon_2mm.log`. This section
to be filled in when Q1 completes._

### 2.3 Q1 verdict

_Pending._

Two outcomes:

- **Pass** (pivot stagnation pattern smears out + step 0 residual
  descends past 0.54): H2 confirmed. Implementation session can
  proceed with adaptive mesher per bookmark §3 + §1.4 above
  (distance-banded the right scope).
- **Fail** (same persistent-pivot pattern at the refined mesh):
  H2 falsified. Mesh refinement is not the remedy. Escalate to
  bookmark §5 deferred options: modified Cholesky, F-bar / mixed
  u-p, or block preconditioner — bigger sim-soft surface.

---

## 3. Sequencing forward

If Q1 passes:
- **Session N+2 (implementation)**: build adaptive BCC mesher
  primitive (~300 LOC mesh-sdf) + distance-banded predicate
  threading through cf-device-design (~100 LOC) + regression
  substrate (~50 LOC). Optional GUI hookup (~50 LOC).
- The diagnostic `H2-Q3 DIAG` instrumentation should be reverted
  before the implementation arc lands (or promoted to a permanent
  observability hook gated behind a debug flag).
- The `h2_recon_cavity_6mm_n16` test gets either deleted or
  refactored into a regression sentinel asserting "no pivot vertex
  appears >5× consecutively post-refinement at cavity 6+7 mm".

If Q1 fails:
- New bookmark covering modified Cholesky or F-bar arc; H2 retired.

---

End of recon findings (Q3 complete; Q1 pending).
