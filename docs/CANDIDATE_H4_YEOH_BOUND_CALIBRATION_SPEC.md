# Candidate H4 — Yeoh bound calibration plumbing

> **STATUS — SHIPPED + FALSIFIED case D.** Spec was implemented as
> H4.1 + H4.2 (5-arg `from_yeoh_fields_with_bounds` plumbing); the
> cavity = 5 + 8 mm sweep triggered the spec's §5 case-D
> falsifier (compressive floor binds before tensile cap; the
> Phase 4 placeholder 0.30 was hiding deep-compression states).
> The asymmetric one-sided fix shipped as H4-2-C (drop compressive
> gate at sample time, keep tensile cap).  **Read
> `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` for the falsification
> + shipped behavior; this spec stays as audit trail for the
> original H4.1 / H4.2 design and the falsifier matrix that
> correctly anticipated case D.**
>
> 2026-05-19 EVENING.  F3 recon B candidate H4 per
> [[project-f3-recon-b-eb-arc]] §"What stayed dormant" + the
> E.b falsification bookmark §10.4 H-ladder. Refined scope: the
> per-anchor `0.8 · λ_break` validity bounds are **already in
> the silicone catalog** (`sim/L0/soft/src/material/silicone_table.rs:497-602`,
> 5.76–8.80 across anchors); the failure mode the E.b arc
> diagnosed (`max_stretch_deviation = 1.0` fires at σ = 2.05 on
> tet 3206 at cavity = 6 mm) is **a plumbing gap** —
> `MaterialField::sample_yeoh` drops the per-anchor bounds at
> the per-tet build site. Implementation ladder in §3-§4;
> sweep protocol at cavity = 6 mm in §6.
>
> **Parent docs**:
> - `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10.4 (recon
>   ladder ranking; H2/H4 the only remaining levers at cavity
>   > 5 mm).
> - `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` — the spec
>   shape this follows (C′.a-brevity).
>
> **Predecessor memory**: [[project-f3-recon-b-eb-arc]],
> [[project-c-prime-a-shipped]], [[project-f3-recon-a-gated-lm-shipped]],
> [[project-sl-4-arc-shipped]].

---

## TL;DR

The E.b falsification §10.4 ranked **H4 (Yeoh bound calibration)
~100-200 LOC pending datasheet research**. The research is
already done — every anchor in `silicone_table.rs` carries
`validity_max_principal_stretch = 0.8 · λ_break` (5.76 for
Dragon Skin 20A, 8.00 for Ecoflex 00-30, etc.), all
well-above σ = 2.05 hit at cavity = 6 mm.

The actual gap is at the per-tet sample site:

- `Yeoh::with_principal_stretch_bounds` exists +
  `SiliconeMaterial::to_yeoh` propagates the bounds to a Yeoh.
- The solver's `check_validity_at_step_start`
  (`backward_euler.rs:633`) routes to the principal-stretch
  gate when EITHER bound is `Some`, otherwise falls back to
  `max_stretch_deviation`.
- BUT `MaterialField::sample_yeoh`
  (`material_field.rs:261-272`) builds the per-tet Yeoh via
  `Yeoh::from_lame_and_c2(mu, lambda, c2)` — bounds are
  `None`. So every cf-device-design tet falls back to legacy
  `max_stretch_deviation = 1.0`.

H4 closes the gap: extend `MaterialFieldInner::Yeoh` with two
optional bound-field slots, add a 5-arg constructor, and
update the cf-device-design call site to pass per-layer bounds.

**Falsifier**: if cavity = 6 mm still fails at step 1 after
threading the bounds (different failure: real Newton stall, not
Yeoh) → H4 buys nothing past 5 mm; bookmark + escalate to H2
(mesh refinement). If 3 mm or 5 mm sanity gates regress under
the new gate, plumbing has a bug — debug, don't ship.

---

## 1. Problem statement

### 1.1 The catalog already has calibrated bounds

`sim/L0/soft/src/material/silicone_table.rs` ships every
anchor with `validity_max_principal_stretch = 0.8 · λ_break`:

| anchor | `validity_max_principal_stretch` | source (TDS) |
|---|---|---|
| ECOFLEX_00_10 | 7.20 | 800 % elongation at break |
| ECOFLEX_00_20 | 7.56 | 845 % |
| ECOFLEX_00_30 | 8.00 | 900 % |
| ECOFLEX_00_50 | 8.64 | 980 % |
| DRAGON_SKIN_10A | 8.80 | 1000 % |
| DRAGON_SKIN_15 | 6.97 | 771 % |
| DRAGON_SKIN_20A | 5.76 | 620 % |
| DRAGON_SKIN_30A | 3.71 | 364 % |

Compressive bound `validity_min_principal_stretch = 0.30`
family-uniform (`YEOH_MIN_PRINCIPAL_STRETCH`).

All of these are above the σ ≈ 2.05 peak the E.b sweep
measured at tet 3206 (cavity = 6 mm + 10+3 layer stack +
iter-1 sock + Ecoflex 00-30 outer / Dragon Skin 20A inner).

### 1.2 The plumbing drops them

`MaterialField::sample_yeoh` builds per-tet Yeoh via
`Yeoh::from_lame_and_c2(...)`. No `.with_principal_stretch_bounds(...)`
call. Comment at line 244-248 says verbatim "Validity bounds
default to `None` on the produced `Yeoh` — per-anchor bounds
are wired in by the row author via the
`SiliconeMaterial::to_yeoh` path, not the `MaterialField`
aggregator". That is the deliberate-but-now-load-bearing
design choice this spec changes.

The legacy default-`None` path was correct for callers that
genuinely want the symmetric `max_stretch_deviation = 1.0`
NH-style gate (early sim-soft examples). cf-device-design's
insertion sim is NOT such a caller — every layer's
`SiliconeMaterial` has known datasheet bounds. The plumbing
drops them.

### 1.3 What H4 does

Extend the Yeoh aggregator to optionally carry two more scalar
fields:

```rust
enum MaterialFieldInner {
    NeoHookean { mu, lambda },
    Yeoh {
        mu, c2, lambda,
        // NEW: optional asymmetric stretch caps. None → legacy
        // path (Yeoh built without bounds, falls back to
        // max_stretch_deviation gate). Some → per-tet Yeoh
        // carries max + min principal stretch caps.
        bounds: Option<YeohBoundsFields>,
    },
}

struct YeohBoundsFields {
    max_principal_stretch: Box<dyn Field<f64>>,
    min_principal_stretch: Box<dyn Field<f64>>,
}
```

New constructor `from_yeoh_fields_with_bounds(mu, c2, lambda,
max_p, min_p)`. Existing `from_yeoh_fields(mu, c2, lambda)`
keeps current behavior (bounds → `None`) for backward compat.
`sample_yeoh` reads the bounds when present and calls
`.with_principal_stretch_bounds(max, min)` on the produced
Yeoh.

---

## 2. Mechanism + design choices

### 2.1 Why optional, not always-on

Existing test + example callers (`tests/yeoh_contract.rs`,
`tests/material_field_sample.rs`, four sim-soft examples) call
`from_yeoh_fields(mu, c2, lambda)` and pin behavior against
the legacy `max_stretch_deviation` gate. Forcing them to
provide bounds would either break them or require synthesizing
sentinel values (`f64::INFINITY`) that would flip the gate
flavor and may regress unrelated tests. Keep the 3-arg path
bit-exact.

### 2.2 `(max_p, min_p)` is always paired

Per `Yeoh::with_principal_stretch_bounds(max, min)` — both
bounds are required in a single setter. The
`MaterialFieldInner::Yeoh::bounds` slot is therefore
`Option<YeohBoundsFields>` (paired), not two independent
`Option<Box<dyn Field<f64>>>` slots. Matches the upstream
contract: the validity gate at `backward_euler.rs:679-683`
flips to the principal-stretch flavor on `(Some, _)` or
`(_, Some)`; this aggregator doesn't ship the asymmetric
single-bound case because no production caller needs it.

### 2.3 No new field-of-bounds variant

`max_principal_stretch` + `min_principal_stretch` are scalars
in cf-device-design (one value per layer, sampled by
`LayeredScalarField` keyed on the cleaned-scan SDF). Same
type as `mu`, `c2`, `lambda` — `Box<dyn Field<f64>>`. The
`Field<f64>` trait already accepts any sampling backend, so
this requires no new field types.

### 2.4 Mirror-on-change invariant

Update sites the mirror docstring should call out:

- `MaterialFieldInner::Yeoh` — add `bounds` slot.
- `MaterialField::from_yeoh_fields` — 3-arg legacy.
- `MaterialField::from_yeoh_fields_with_bounds` — 5-arg new.
- `MaterialField::sample_yeoh` — branch on `bounds.as_ref()`.
- `sample_yeoh` docstring — bounds-aware behavior.
- `from_yeoh_fields` docstring — annotate "legacy bounds-
  None path; new callers prefer `from_yeoh_fields_with_bounds`".

---

## 3. sim-soft surface changes

### 3.1 `MaterialFieldInner::Yeoh` variant

Add `bounds: Option<YeohBoundsFields>` to the existing
variant. Add the `YeohBoundsFields` struct in the same module.

### 3.2 New constructor

```rust
#[must_use]
pub fn from_yeoh_fields_with_bounds(
    mu: Box<dyn Field<f64>>,
    c2: Box<dyn Field<f64>>,
    lambda: Box<dyn Field<f64>>,
    max_principal_stretch: Box<dyn Field<f64>>,
    min_principal_stretch: Box<dyn Field<f64>>,
) -> Self {
    Self {
        inner: MaterialFieldInner::Yeoh {
            mu, c2, lambda,
            bounds: Some(YeohBoundsFields {
                max_principal_stretch,
                min_principal_stretch,
            }),
        },
        interface_sdf: None,
    }
}
```

### 3.3 `sample_yeoh` update

```rust
pub fn sample_yeoh(&self, x_ref: Vec3) -> Yeoh {
    match &self.inner {
        MaterialFieldInner::Yeoh { mu, c2, lambda, bounds } => {
            let y = Yeoh::from_lame_and_c2(
                mu.sample(x_ref),
                lambda.sample(x_ref),
                c2.sample(x_ref),
            );
            match bounds {
                None => y,
                Some(b) => y.with_principal_stretch_bounds(
                    b.max_principal_stretch.sample(x_ref),
                    b.min_principal_stretch.sample(x_ref),
                ),
            }
        }
        MaterialFieldInner::NeoHookean { .. } => panic!(
            "MaterialField::sample_yeoh expects Yeoh variant; this field is NH — use sample"
        ),
    }
}
```

### 3.4 Unit tests

In `sim/L0/soft/tests/material_field_sample.rs`:

- `material_field_from_yeoh_fields_with_bounds_carries_bounds` —
  build with `ConstantField::new(5.76)` / `ConstantField::new(0.30)`,
  call `sample_yeoh(Vec3::zeros())`, assert
  `validity().max_principal_stretch == Some(5.76)` AND
  `validity().min_principal_stretch == Some(0.30)`.
- `material_field_from_yeoh_fields_keeps_legacy_none_bounds` —
  build via 3-arg constructor, assert `validity()` slots are
  `None` (legacy behavior pinned).

### 3.5 No solver-side changes

`backward_euler.rs:633` `check_validity_at_step_start` already
routes to per-principal-stretch gate when bounds present.
No change.

---

## 4. cf-device-design wiring

### 4.1 Two new layered fields at `insertion_sim.rs:764`

Add `max_principal_stretch_field` + `min_principal_stretch_field`
mirrors of the existing `(μ, C₂, λ)` triple, sourced from each
layer's resolved `SiliconeMaterial`:

```rust
let material_field = MaterialField::from_yeoh_fields_with_bounds(
    layered_param_field(&scan_sdf, &thresholds, materials.iter().map(|m| m.mu).collect()),
    layered_param_field(&scan_sdf, &thresholds, materials.iter().map(|m| m.c2).collect()),
    layered_param_field(&scan_sdf, &thresholds, materials.iter().map(|m| m.lambda).collect()),
    layered_param_field(&scan_sdf, &thresholds, materials.iter().map(|m| m.validity_max_principal_stretch).collect()),
    layered_param_field(&scan_sdf, &thresholds, materials.iter().map(|m| m.validity_min_principal_stretch).collect()),
);
```

### 4.2 Test call site at `insertion_sim.rs:4933`

Mirror the production change. If the test's intent is to pin
legacy NH-symmetric behavior, leave on `from_yeoh_fields` (3-arg).
Otherwise re-route to 5-arg with the same per-layer bounds.

### 4.3 Sentinel test

In cf-device-design's test surface:

`insertion_sim_per_tet_yeoh_carries_calibrated_principal_stretch_bounds`
— build the geometry, walk the produced mesh's
`materials()`, assert every tet's `validity()` returns
`max_principal_stretch.is_some()` AND
`min_principal_stretch.is_some()`. Pin against legacy
`None`-bounds drift.

---

## 5. Falsifier matrix

| case | observation at cavity = 6 mm | meaning | action |
|---|---|---|---|
| **A — H4 wins** | 16/16 converged, no Yeoh panic, peak σ < per-anchor bound | plumbing was the only gate; calibrated bounds match real material | H4.4 cap raise 5 → 6 mm (mirror C′.a precedent) |
| **B — different failure** | step 1 stalls at large r_norm OR Yeoh fires at tet σ > per-anchor bound (real material limit) | H4 closes the bound gap but cavity = 6 mm is past actual material validity OR a different non-bound failure surfaces | bookmark + escalate to H2 (mesh refinement) |
| **C — 3 / 5 mm regression** | 3 mm or 5 mm sanity gate breaks under new gate | plumbing bug | debug — don't ship |
| **D — silent fake-converge** | 6 mm "converges" but post-solve check fires at end-of-solve check (commit 2739717e) | new gate caught what the legacy gate missed differently | analyze the σ peak vs the calibrated bound — case A or B per the σ/bound ratio |

Cases B + C are the not-ship cases. Case A + D-with-σ < bound
ship.

---

## 6. Sweep protocol at cavity = 6 mm

Default settings: layers 10 + 3 mm (`DEFAULT_N_STEPS = 16`),
`sock_over_capsule.cleaned.stl`, iter-1 layer materials
(outer Ecoflex 00-30 + inner Dragon Skin 20A). The user-driven
visual gate is the same scrub-1-to-16 as C′.a + E.b
predecessors.

**Sanity gates first** (run before declaring case A):

| cavity | expected | result | binding constraint |
|---|---|---|---|
| 3 mm | 16/16 (no LM, no Yeoh) | TBD | none |
| 5 mm | 16/16 (no LM, no Yeoh) | TBD | none |

**Primary gate**:

| cavity | E.b §10 baseline (legacy bounds) | H4 expected | result |
|---|---|---|---|
| 6 mm | 1/16 fake-converged → step 2 Yeoh tet 3206 ms = 1.05 | 16/16 (σ peak < 5.76 = DS20A bound) | TBD |

If primary passes 16/16: probe gates at 7 mm + 8 mm to find the
new ceiling. Cap raise lands at the highest cavity where
sanity holds + primary clears.

If primary fails 6 mm at all (case B): step-by-step diagnostic —
`SolveDiagnostics::max_principal_stretch` row already reports
the per-step peak. Compare against per-layer bounds. If σ
exceeds the bound → real material limit at that mesh resolution
(H2 lever). If σ < bound but Newton stalls → different
failure class; bookmark + recon-iter.

---

## 7. Anchors

- `sim/L0/soft/src/material/material_field.rs` — primary site.
- `sim/L0/soft/src/material/yeoh.rs:72-76` —
  `with_principal_stretch_bounds`.
- `sim/L0/soft/src/material/silicone_table.rs:497-602` —
  per-anchor bounds.
- `sim/L0/soft/src/solver/backward_euler.rs:633-720` — the
  validity gate that branches on `(max_p, min_p)` presence.
- `tools/cf-device-design/src/insertion_sim.rs:764` —
  cf-device-design call site.
- `tools/cf-device-design/src/insertion_sim.rs:4933` — test
  call site.
- `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10.4 — H-ladder
  ranking + recon estimate (this spec refines).

---

## 8. Implementation ladder

H4.1: sim-soft plumbing (variant + constructor + sample_yeoh
+ 2 unit tests).
H4.2: cf-device-design wiring (call site + sentinel test).
H4.3: cavity = 6 mm sweep + sanity gates 3, 5 mm.
H4.4: case A → cap raise (mirror C′.a 3-surface pattern); case
B → falsification bookmark; case C → debug.
H4.5: cold-read pass-1 + pass-2.

Per [[feedback-implement-measure-revert-pattern]] — cheap
plumbing, just ship and measure. If case B surfaces, the empirical
falsification reshapes whether H2 is worth the ~500 LOC.

---

End of spec.
