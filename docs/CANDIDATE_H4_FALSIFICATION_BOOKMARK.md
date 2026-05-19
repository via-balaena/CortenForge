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

- `sim/L0/soft/src/material/silicone_table.rs:493` —
  `YEOH_MIN_PRINCIPAL_STRETCH = 0.30` declaration + docstring.
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

End of bookmark.
