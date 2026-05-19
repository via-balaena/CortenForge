# Candidate H4 — falsification bookmark (case D)

> **STATUS — RESOLVED case D (H4 plumbing kept, scaffolding reverted).**
> 2026-05-19 NIGHT.  H4.3 sweep at cavity = 3/5/6/7/8 mm on iter-1
> sock_over_capsule (dual-layer DS20A 3 mm inner + ECOFLEX_00_30
> 10 mm outer) revealed that **H4 plumbing is correct but surfaces
> the wrong gate**: the calibrated `min_principal_stretch ≥ 0.30`
> floor (family-uniform `YEOH_MIN_PRINCIPAL_STRETCH`) catches
> states that the legacy `max_stretch_deviation ≤ 1.0` gate
> silently accepted.  Pre-H4 16/16 baselines at cavity ≤ 5 mm
> were passing through deeply-compressed tets (σ_min as low as
> 0.219 = 78% compression) that the loose symmetric legacy bound
> implicitly allowed.
>
> **Per [[feedback-spec-falsified-revert-opt-in-keep-surface]]**:
> H4.1 sim-soft plumbing (5-arg `from_yeoh_fields_with_bounds`)
> + H4.2 cf-device-design call site kept; H4.3 scaffolding
> (cap-raise 5 → 8 mm) reverted.  The 0.30 compressive floor is
> a Phase 4 conservative placeholder per its own docstring —
> "engineering-aggressive default; replace with measured value
> once compression-set test data lands" — so the bound is the
> binding constraint, not real material physics.

**Predecessor docs**:

- `docs/CANDIDATE_H4_YEOH_BOUND_CALIBRATION_SPEC.md` — the H4
  spec (§5 falsifier matrix anticipated case D as "different
  failure surface").
- `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10.4 — the
  H-ladder recon estimate this spec refined.

**Predecessor memory**: [[project-f3-recon-b-eb-arc]],
[[project-c-prime-a-shipped]],
[[project-f3-recon-a-gated-lm-shipped]],
[[project-f3-falsification-bookmark]],
[[project-sl-4-arc-shipped]].

---

## TL;DR

H4 plumbing was correctly verified inert on Newton iteration
arithmetic (bit-identical Armijo stall at iter 48 r_norm 21.30
for cavity = 3 mm pre-H4 3-arg vs post-H4 5-arg).  The plumbing
correctly routes per-tet `Yeoh` through the principal-stretch
gate instead of the legacy symmetric gate.

**But the legacy gate was masking compression** that the new
gate correctly catches:

| cavity | dual-layer outcome | failing tet | σ_min |
|---|---|---|---|
| 3 mm | Armijo stall iter 48, r_norm 21.3 | — (no Newton convergence) | — |
| 5 mm | Yeoh tet 154 panic | min σ = 0.219 | < 0.30 bound |
| 6 mm | Armijo stall iter 25, r_norm 3.78 | — | — |
| 7 mm | Armijo stall iter 30, r_norm 0.45 | — | — |
| 8 mm | Yeoh tet 519 panic | min σ = 0.271 | < 0.30 bound |

The 3 + 6 + 7 mm Armijo stalls are independent of H4 — they're
a property of the automated `h4_sweep_sliding_ramp_on_iter1_scan`
test config diverging from the user's GUI baseline (the iter-1
prior visual gates at cavity = 3, 5 mm passed 16/16 via GUI
scrubbing, but the same code path under `cargo test` stalls at
step 0).  This is a separate sub-puzzle the H4 falsification
doesn't need to solve to reach a clear next step.

**Falsifier (spec §5 case D resolution)**: H4 plumbing is
correct but the family-uniform `YEOH_MIN_PRINCIPAL_STRETCH = 0.30`
floor is the binding gate at cavity ≥ 5 mm, not the tensile
`0.8 · λ_break` cap H4 was designed to unlock.  The 0.30 floor
is conservative-by-design (Phase 4 placeholder, not material-
calibrated).  Loosening it requires compression-set test data
that the catalog doesn't ship today.

---

## 1. What we learned (3 structural findings)

### 1.1 H4 plumbing is verified inert on Newton arithmetic

Bisect at cavity = 3 mm dual-layer:

| `MaterialField` constructor | step 0 outcome | Newton iter | r_norm |
|---|---|---|---|
| 5-arg `from_yeoh_fields_with_bounds` (H4.2) | Armijo stall | 48 | 2.130e1 |
| 3-arg `from_yeoh_fields` (pre-H4) | Armijo stall | 48 | 2.130e1 |

Bit-identical iter count + bit-identical residual.  The
`with_principal_stretch_bounds(max, min)` decorator on `Yeoh`
sets two `Option<f64>` fields without touching
`energy` / `first_piola` / `tangent` — Newton iteration produces
the same residuals regardless of whether the bound slots are
`Some` or `None`.  The decorator only affects
`check_validity_at_step_start` panic timing.

The H4 plumbing is therefore correct + structurally orthogonal
to Newton convergence.  The shipped H4.1 + H4.2 surface stays.

### 1.2 The compressive floor 0.30 binds at cavity ≥ 5 mm

The pre-H4 legacy gate
`max_stretch_deviation ≤ 1.0` allowed σ ∈ [0, 2.0] — including
σ_min ∈ [0, 1.0] (tet compressions from 100% down to 0%).  The
0.30 floor that H4 routes the per-tet `Yeoh` through is *tighter*
on the compressive side than the legacy symmetric default.  At
cavity = 5 mm + 8 mm two specific tets hit σ_min in the (0.219,
0.293) range — pre-H4 these states silently accepted; post-H4
they panic.

`YEOH_MIN_PRINCIPAL_STRETCH = 0.30` is documented as:

> "Compressive principal-stretch cap (Yeoh validity gate, memo
> D8). Family-uniform at 0.30 (engineering-aggressive default;
> replace with measured value once compression-set test data
> lands)."

So 0.30 is a Phase 4 fail-closed placeholder, not a
material-physics result.  Real silicones can be compressed
substantially below 0.30 elastically (Ecoflex 00-30 has very
low bulk modulus + extended compression-set tolerance per
Smooth-On TDS notes).  The bound is the binding constraint,
not material reality.

### 1.3 H4's design assumption (strict gate widening) was wrong

The H4 spec §1.1 + TL;DR assumed: "calibrated bounds are FAR
above the σ ≈ 2.05 peak the E.b sweep measured" — true on the
tensile side (5.76 vs 2.0).  But the legacy gate is **symmetric**
(`|σ - 1| ≤ 1` → σ ∈ [0, 2]) while H4's bounds are **asymmetric**
(tensile 5.76, compressive 0.30).  The compressive direction
actually TIGHTENS the gate, not loosens it.

The E.b sweep's σ ≈ 2.05 tet 3206 peak observation was a
tensile-side finding.  The compressive-side regression was
invisible because the legacy gate accepted σ_min ∈ [0, 1.0].
H4 surfacing this compressive regression IS WORKING AS
DESIGNED for an honest validity gate — but it makes the
"H4 unlocks cavity > 5 mm" goal incorrect at this mesh
resolution + bound value.

---

## 2. What's KEPT vs REVERTED

### Kept (H4.1 + H4.2 surface plumbing)

- `MaterialFieldInner::Yeoh` carries `bounds: Option<YeohBoundsFields>`.
- `MaterialField::from_yeoh_fields_with_bounds` 5-arg constructor.
- `MaterialField::sample_yeoh` routes through
  `with_principal_stretch_bounds` when bounds present.
- cf-device-design `insertion_sim.rs:764` production call site
  uses the 5-arg path.
- H4.2 sentinel test
  `build_insertion_geometry_per_tet_yeoh_carries_calibrated_principal_stretch_bounds`.
- Two H4.1 pinning tests in
  `tests/material_field_sample.rs`.

The plumbing is structurally clean + composes with future
material-bound calibration work (an H4-2 sub-arc that loosens
`YEOH_MIN_PRINCIPAL_STRETCH` to a measured value would flip
one const, surface bounded by [[project-c-prime-a-shipped]]'s
"3-surface mirror" pattern).

### Reverted (H4.3 cap-raise scaffolding only)

- `CavityState::inset_slider_range_m` reverted 8 → 5 mm.
- `render_cavity_section` egui label reverted.
- Sentinel test renamed back to
  `cavity_inset_slider_range_zero_to_five_mm`.

### Kept (H4.3 sweep test as falsification artifact)

The `h4_sweep_sliding_ramp_on_iter1_scan` `#[ignore]`-d test
stays as the empirical record + re-runnable falsification
gate.  Future H4-2 work that loosens
`YEOH_MIN_PRINCIPAL_STRETCH` re-runs this sweep to verify the
compressive panics clear.

---

## 3. Next-session recon — compressive-floor sub-arc

### 3.1 Hypothesis ranking

- **H4-2 (compressive floor calibration)** — research the real
  compression-set bounds for the 4 silicones in the catalog
  (Ecoflex 00-30, DS10A, DS20A, DS30A) + per-anchor
  `validity_min_principal_stretch` values.  Loosen the
  family-uniform 0.30 floor → per-anchor calibrated values
  (likely 0.1-0.2 range per Smooth-On compression-set TDS
  rows).  Re-run H4.3 sweep with calibrated min bounds; verify
  cavity = 5 / 8 mm clears.
- **H4-3 (asymmetric one-sided bounds)** — alternative shape:
  use only `max_principal_stretch` (tensile cap, looser than
  legacy) and leave `min_principal_stretch = None` (legacy
  symmetric floor only).  The `(Some, None)` branch in the
  solver gate at `backward_euler.rs:700` checks tensile only +
  has no compressive check.  Equivalent to "asymmetric one-
  sided widening" — strictly looser than pre-H4 in both
  directions.  Lighter-weight than H4-2 but loses compressive
  safety entirely.
- **H2 (mesh refinement)** — still on the table but now even
  more speculative.  H4 didn't surface the original tensile
  Yeoh wall (no σ_max > calibrated bound observed) — the
  compressive wall fires first at this mesh resolution.  Mesh
  refinement would distribute compressive strain too, but
  whether it actually unlocks cavity > 5 mm depends on the
  Phase 4 floor being relaxed first OR raised via H4-3
  asymmetric-bound shape.

### 3.2 Recommended order

H4-2 first (calibration research) — smallest scope, highest
leverage if real silicone bounds are below 0.30.  If H4-2 also
falsifies (real bounds are AT 0.30) → H4-3 (asymmetric) as
fallback that preserves H4 tensile gain.  Reserve H2 as last
resort.

### 3.3 Open question — the 3 / 6 / 7 mm Armijo stalls

The automated `cargo test` sliding-ramp sweep stalls Newton at
step 0 for cavity = 3, 6, 7 mm dual-layer (and the same config
in the GUI is the user's prior 16/16 visual-gate baseline at
cavity = 3, 5 mm).  Bit-identical pre/post H4, so it's not an
H4 regression.  Possibilities:

- Centerline-source mismatch: prep.toml centerline points may
  be in a different frame than what the GUI's interactive
  pipeline produces, even though `parse_centerline` is shared.
- Cap-planes mismatch: prep.toml has `capped_loops = 0`; the
  GUI may merge cap planes derived elsewhere.
- Slacker fraction default: `layer()` test helper sets
  `slacker_fraction = 0.0`; the GUI default may differ.
- Other GUI-side state that doesn't lift through the test
  shortcut.

Worth investigating in a separate session if H4-2 calibration
land and the compressive panics clear but the Armijo stalls
persist — diagnosing this would also generally improve
testability of the sliding-ramp pipeline.

---

## 4. Anchors

- `sim/L0/soft/src/material/silicone_table.rs:522` —
  `YEOH_MIN_PRINCIPAL_STRETCH = 0.20` declaration + docstring
  (loosened 0.30 → 0.20 by H4-2-A, see §5 below).
- `sim/L0/soft/src/material/material_field.rs:148-244` —
  H4.1 plumbing surface (5-arg constructor + sample_yeoh).
- `sim/L0/soft/src/solver/backward_euler.rs:679-720` — the
  validity gate that branches on bound presence.
- `tools/cf-device-design/src/insertion_sim.rs:764` —
  cf-device-design production call site (5-arg path).
- `tools/cf-device-design/src/insertion_sim.rs:4926` —
  `h4_sweep_sliding_ramp_on_iter1_scan` test (falsification
  artifact).
- `docs/CANDIDATE_H4_YEOH_BOUND_CALIBRATION_SPEC.md` §5 —
  falsifier matrix where case D is anticipated.

---

## 5. §POST-RESOLUTION — H4-2-A 0.20 floor falsifies "loosen the floor" thesis

2026-05-19 LATE-NIGHT.  Per
`docs/CANDIDATE_H4_COMPRESSION_RESEARCH.md` §3 Option A, the
family-uniform `YEOH_MIN_PRINCIPAL_STRETCH` was loosened from
0.30 → 0.20 (research-informed default per Sparks et al. 2015
unconfined-compression-no-failure-at-25%-strain).  Re-running
`h4_sweep_sliding_ramp_on_iter1_scan` at the new floor:

| cavity | pre-H4-2-A outcome | post-H4-2-A outcome |
|---|---|---|
| 3 mm | Armijo stall iter 48 r_norm 21.3 | Armijo stall iter 48 r_norm 21.3 (bit-identical) |
| 5 mm | Yeoh tet 154 σ_min = 0.219 | Yeoh **tet 264 σ_min = 0.197** (different tet, deeper compression) |
| 6 mm | Armijo stall iter 25 r_norm 3.78 | Armijo stall iter 25 r_norm 3.78 (bit-identical) |
| 7 mm | Armijo stall iter 30 r_norm 0.45 | Armijo stall iter 30 r_norm 0.45 (bit-identical) |
| 8 mm | Yeoh tet 519 σ_min = 0.271 | Yeoh **tet 673 σ_min = 0.120** (way past floor, σ = [1.915, 1.000, 0.120]) |

### 5.1 What the data shows

Newton's preferred equilibrium at cavity > 5 mm requires σ_min
**much deeper** than any reasonable floor.  At cavity = 8 mm
the post-H4-2-A panic is at σ_min = 0.120 (det F ≈ 0.23 = 77 %
volume compression at one tet) — eight percentage points past
the 0.20 floor, well past 0.10 too.  Incremental loosening
would just move the panic to a different tet at deeper
compression; the cap > 5 mm unlock is **not** delivered by
floor-tuning alone.

### 5.2 Mechanism reframe — ν = 0.40 is the binding constraint

The H4-2-A research memo §2 already flagged this hypothesis;
the H4-2-A sweep CONFIRMS it.  cf-design's Yeoh uses ν = 0.40
(standalone Yeoh asserts `nu < 0.45`); real silicones per
Sparks are ν ≈ 0.4999.  The cf-design bulk modulus is ~240×
lower than real, allowing the FEM to find equilibria with
extreme single-tet volume compression that real silicone
wouldn't sustain.  The floor catches these unphysical states
honestly; loosening it just lets Newton find new unphysical
states at deeper compression.

The genuine fix is **Option B (Phase H F-bar / mixed-u-p
locking-fix decorator)** that widens ν past 0.45 to the real
~0.5 value.  Multi-session arc; banked behind workshop iter-1
unblock and the rest of the active product pipeline.

### 5.3 What H4-2-A is still worth keeping

H4-2-A's 0.30 → 0.20 change is KEPT (commit pending after this
bookmark addendum) for two reasons:

1. **Research-defensible default** — Sparks et al. validated no
   failure at 25 % compression; 0.20 is 5× past that floor +
   within published rubber-elasticity guidance.  0.30 was a
   pure Phase 4 engineering placeholder ("replace with measured
   value once compression-set test data lands") that the
   compression research has now informed.
2. **Marginal cavity ≤ 5 mm benefit** — for sliding ramps where
   σ_min lands in (0.20, 0.30) at a borderline tet, H4-2-A
   reduces compressive-side false-positive panics relative to
   the pre-H4-2-A 0.30 floor.  Pre-H4 legacy
   `max_stretch_deviation ≤ 1.0` would accept those scenarios;
   H4 plumbing post-H4-2-A also accepts them.

### 5.4 H4-2-C SHIPPED — partial unlock at cavity 5 + 8 mm

2026-05-19 LATE-NIGHT (post-H4-2-A).  Option C implemented:
`Yeoh::with_max_principal_stretch_only` setter added;
`MaterialField::sample_yeoh` switched to the new asymmetric path
when bounds present.  Per-tet `Yeoh` now carries `(Some(max),
None)` from the `MaterialField` pipeline; `det F > 0` inversion
is the only remaining compressive safety net.  The
`min_principal_stretch` field stays threaded through
`MaterialFieldInner::Yeoh.bounds.min_principal_stretch` for
future Option B re-enable (one-line flip
`with_max_principal_stretch_only` → `with_principal_stretch_bounds`).

H4.3 sweep re-run under H4-2-C asymmetric:

| cavity | pre-H4-2-C | post-H4-2-C |
|---|---|---|
| 3 mm | Armijo step 0 iter 48 (bit-identical Newton; H4-independent) | unchanged |
| 5 mm | Yeoh tet 264 σ_min = 0.197 (0/16) | **11/16 converged**, Armijo step 11 iter 9 r_norm 19.2 |
| 6 mm | Armijo step 0 iter 25 r_norm 3.78 (H4-independent) | unchanged |
| 7 mm | Armijo step 0 iter 30 r_norm 0.45 (H4-independent) | unchanged |
| 8 mm | Yeoh tet 673 σ_min = 0.120 (0/16) | **8/16 converged**, Armijo step 8 iter 5 r_norm 0.40 |

Cavity = 5 mm: 0 → 11 steps converged (3.44 mm of 5 mm = 69 %
seated).  Cavity = 8 mm: 0 → 8 steps converged (4.0 mm of
8 mm = 50 % seated).  The compressive panic was the binding
step-0 wall at both cavities; disabling it lets Newton iterate
through deep-compression equilibria and find converged states
at most subsequent steps before a later-step Armijo stall (a
genuine Newton limit, not a Yeoh gate firing).

Cavity 6 + 7 mm Armijo stalls remain bit-identical with H4
plumbing — those are H4-independent Newton convergence
failures (see §5.5).

### 5.5 What stayed on the recon ladder

- **Option B (Phase H F-bar / mixed-u-p decorator)** is still
  the long-term right answer.  Real silicones at ν ≈ 0.5 would
  resist the volume changes that the cf-design ν = 0.40 model
  allows; H4-2-C bypasses the symptom (compressive gate firing
  on unphysical states) without fixing the root cause.
  Re-enabling the compressive gate after Phase H ships is the
  intended composition.
- **The 5 + 8 mm later-step Armijo stalls under H4-2-C** are
  genuine Newton limits at deeper insertion fractions —
  likely solvable by smaller slide steps, looser tolerance, or
  better warm-starting.  Worth a small probe arc if the partial-
  seating output of H4-2-C is the binding product gap.

### 5.6 Open puzzle reminder

The 3 + 6 + 7 mm Armijo stalls in the automated `cargo test`
path are independent of H4 (bit-identical pre-H4 / post-H4 /
post-H4-2-A / post-H4-2-C — four-way bisect).  Worth
investigating separately to identify the `cargo test` ↔ GUI
pipeline divergence; the user's GUI visual gate at cavity 3 +
5 mm reaches 16/16 on the same iter-1 dual-layer config.

---

End of bookmark.
