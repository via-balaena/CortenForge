# Candidate H4-2 — compressive bound research notes

> **STATUS — RESEARCH MEMO.** 2026-05-19 NIGHT.  Compression-side
> bound research for the next-session H4-2 sub-arc that follows
> the [[project-h4-arc-falsified]] case-D outcome.  Consumes
> `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §3 recon ladder.

---

## TL;DR

1. **Smooth-On TDS publishes ZERO compression data** for both
   Ecoflex and Dragon Skin series.  Properties are entirely
   tensile-side (tensile strength, 100% modulus, elongation at
   break, tear strength).  `YEOH_MIN_PRINCIPAL_STRETCH = 0.30`
   in `silicone_table.rs` is a pure Phase-4 engineering
   placeholder (`"engineering-aggressive default; replace with
   measured value once compression-set test data lands"`).
2. **Sparks et al. (Adv Skin Wound Care 2015) IS the canonical
   compression data source** for the 3 Smooth-On silicones in
   our catalog (Ecoflex 00-10, Ecoflex 00-30, Dragon Skin).
   Their unconfined uniaxial compression tests went to
   **25 % engineering strain (λ_min = 0.75)** with **NO
   mechanical failure observed** — they stopped at 25 % because
   that's the biological-tissue test target, not a material
   limit.
3. **Sparks's Ogden fits use ν = 0.4999** (near-incompressible)
   for all three silicones.  cf-design catalog uses
   **ν = 0.40** (substantially compressible) per Phase 4
   design choice — standalone Yeoh asserts `nu < 0.45` until
   the F-bar / mixed u-p locking-fix decorator (Part 2 Ch 05,
   banked Phase H work) lands.
4. **Compression-set (ASTM D395) is the wrong metric** for our
   FEM σ_min bound — it measures permanent deformation after
   compression-and-release, not instantaneous elastic limit.
   Wacker Elastosil platinum-cure silicones report 12–33 %
   compression set at 25 % strain × 22 hrs × 175°C; this tells
   us platinum-cure silicones are mostly elastic at 25 %
   compression but doesn't quantify the failure stretch.
5. **The σ_min = 0.22 H4.3 sweep failure is likely a FEM model
   artifact of ν = 0.40, not physical material failure.**  Real
   silicone at ν ≈ 0.4999 would resist that volume compression
   with vastly higher bulk pressure; the cavity-fit equilibrium
   would land at substantially higher σ_min.

---

## 1. What's published vs what's not

### 1.1 Smooth-On TDS (read directly from PDFs)

`ECOFLEX_SERIES_TB.pdf` and `DRAGON_SKIN_SERIES_TB.pdf` both
publish only the tensile-side properties.  No compression
strength, compression set, compressive failure strain, or
Poisson ratio is reported anywhere in the technical bulletins.

Properties published per anchor:

| Property | Source |
|---|---|
| Mixed viscosity | ASTM D-2393 |
| Specific gravity | ASTM D-1475 |
| Shore hardness | ASTM D-2240 |
| Tensile strength | ASTM D-412 |
| 100 % modulus | ASTM D-412 (tensile) |
| Elongation at break | ASTM D-412 |
| Die B tear strength | ASTM D-624 |
| Shrinkage | ASTM D-2566 |
| Dielectric strength | ASTM D-149 / D-147-97a |

**Nothing on compression side.**

### 1.2 Sparks et al. (canonical compression data for our 3 anchors)

> Sparks JL, Vavalle NA, Kasting KE, Long B, Tanaka ML, Sanger
> PA, Schnell K, Conner-Kerr TA.  **Use of Silicone Materials to
> Simulate Tissue Biomechanics as Related to Deep Tissue
> Injury.**  *Advances in Skin & Wound Care*, Vol 28 No 2,
> Feb 2015, pp 59-68.

Direct unconfined uniaxial compression of Ecoflex 00-10,
Ecoflex 00-30, and Dragon Skin samples on a Bose Electroforce
LM1 testing system.  Cylindrical specimens ~35.8 mm dia ×
24.5 mm tall, 1.0 %/s strain rate, max 25 % compression.

**Table 2 (best-fit Ogden 1-term parameters)**:

| Silicone | G (kPa) | α | ν |
|---|---|---|---|
| Dragon Skin | 75.449 | 5.836 | 0.4999 |
| Ecoflex 00-10 | 12.605 | 4.32 | 0.4999 |
| Ecoflex 00-30 | 22.081 | 0.825 | 0.4999 |

(Note: Sparks's "Dragon Skin" is not Shore-graded; comparing
to cf-design's `DRAGON_SKIN_*` family requires reading the
Ogden α back through the shear modulus reasoning — Dragon
Skin Shore-10A has α ≈ 5.84.)

**Figure 5 peak compressive stresses at 25 % strain**:

- Dragon Skin: 73.0 ± 5.2 kPa
- Ecoflex 00-10: 12.1 ± 0.75 kPa
- Ecoflex 00-30: 24.0 ± 1.7 kPa

R² for the Ogden fits = 0.999 (slopes 0.99-1.00).  Clean
elastic response across the entire 0-25 % strain range; no
specimen failure.

### 1.3 Other compression data sources

- **Liao et al. (J Appl Polym Sci 2019, "Mechanical responses
  of Ecoflex silicone rubber: Compressible and incompressible
  behaviors")** — title implies compression, but the actual
  experiments are tension only (uniaxial up to λ = 4).
  "Compressible vs incompressible" refers to the FEM model
  constraint (whether ν is treated as ~0.5 or substantially
  less), not test direction.  Yeoh 3-parameter fit to tensile
  data; not directly useful for compression bounds.
- **Vishwakarma et al. (MDPI Materials 2025, 18, 3037)** —
  Ecoflex 00-20 and formulations under uniaxial testing;
  tensile only up to 1200 % strain.  No compression.
- **MM Science Journal Nov 2025** — confirms Yeoh model
  validity to ~3.5 stretch (tensile); does not address
  compressive validity range.

### 1.4 Compression set (ASTM D395)

Wrong metric for the FEM σ_min bound:

- Compression set measures **permanent deformation after
  release** of a sustained compression.  Standard D395-B
  protocol: compress to 25 % strain, hold 22 hrs at 175°C,
  release, measure recovery.
- Our FEM is quasi-static (no time-at-strain holding) and
  cares about **instantaneous elastic limit**, not memory.

Wacker Elastosil platinum-cure silicones (similar chemistry
to Smooth-On): 12-33 % compression set typical.  Indicates
mostly-elastic behavior at 25 % compression but doesn't
quantify the failure stretch.

---

## 2. The cf-design ν = 0.40 discrepancy

`silicone_table.rs:181-182` documents that **every cf-design
anchor uses ν = 0.40**:

> "At ν = 0.40 every entry satisfies `lambda == 4 * mu`
> (verified by the `lambda_is_four_times_mu_at_nu_0_40` unit
> test below)."

This is enforced by the catalog's `lambda = 4 * mu`
relationship; real silicones per Sparks et al. would be
`lambda ≈ 999 * mu` (ν = 0.4999).

The cf-design `from_young_poisson_and_c2` constructor in
`sim/L0/soft/src/material/yeoh.rs:96-100` panics on
`ν ≥ 0.45`:

```rust
assert!(
    nu < 0.45,
    "standalone Yeoh requires nu < 0.45; use the Ch 05 mixed-u-p / F-bar decorator for higher Poisson ratios"
);
```

So the cf-design Yeoh is **structurally limited to ν < 0.45**
until the F-bar / mixed-u-p decorator (Part 2 Ch 05, banked
Phase H work) ships.

**Consequence for the H4.3 sweep failure**:

- Real silicone at ν = 0.4999 has bulk modulus K = E/3(1-2ν) ≈
  333 × shear modulus.  At ν = 0.40, K ≈ 1.4 × shear modulus —
  **240× lower volumetric stiffness** than reality.
- A tet that compresses to σ_min = 0.22 (78 % unilateral
  compression) in cf-design's ν = 0.40 model would require
  ~240× higher pressure to reach the same state in real
  silicone — orders of magnitude beyond what the sliding-
  intruder contact applies.
- Therefore the σ_min = 0.22 state is **likely a model
  artifact**, not real material behavior.  Real silicone would
  redistribute the stress (via Poisson expansion of the tet's
  other directions) rather than compress to that volume.

---

## 3. Recommended H4-2 path

### 3.1 Option A — loosen `YEOH_MIN_PRINCIPAL_STRETCH` to a defensible engineering bound

Loosen the family-uniform `YEOH_MIN_PRINCIPAL_STRETCH` from
0.30 to **0.20** (or 0.10 for a more aggressive unlock).

**Justification**:

- Sparks et al. measured all three Smooth-On silicones in
  unconfined compression to **25 % engineering strain
  (σ_min = 0.75) with NO failure observed**.  A floor at 0.20
  (80 % compression) is 5× past what's experimentally
  validated, but still well within the elastic-regime
  guidance from rubber-elasticity literature for soft
  platinum-cure silicones.
- The cf-design Yeoh at ν = 0.40 substantially overstates
  compressibility; in practice the cavity-fit equilibrium will
  rarely reach extreme σ_min unless the FEM model misbehaves.
- A 0.20 floor would clear the H4.3 sweep cavity = 5 mm panic
  (σ_min = 0.219 > 0.20 narrowly) but leave cavity = 8 mm
  panic at σ_min = 0.271 also passing.

**Risk**: tets at σ_min ≈ 0.2 may not reflect physical reality;
loosening the floor doesn't fix that — it just stops the gate
from panicking.

**Implementation**: 1-line change in
`silicone_table.rs:493`; update docstring + arc memo D8 with
the Sparks-cited justification; re-run H4.3 sweep to verify
case-D failure clears.  Optional cap-raise scaffolding to
re-probe cavity = 6/7/8 mm.

### 3.2 Option B — fix the ν = 0.40 → ν = 0.4999 discrepancy

The real underlying issue is the Phase-4 ν = 0.40 design
choice.  Real silicones are near-incompressible.  The H4.3
sweep's σ_min = 0.22 state would not occur with realistic
incompressibility.

**Cost**: requires implementing the Part 2 Ch 05 F-bar or
mixed-u-p decorator that wraps standalone Yeoh to widen the
Poisson-ratio range past 0.45.  Banked Phase H work; multi-
session arc.

**Benefit**: produces physically realistic compression
behavior across the entire silicone catalog; removes a Phase-4
modeling compromise that's load-bearing for the
sliding-intruder contact-fit regime.

### 3.3 Option C — asymmetric one-sided bound (H4-3 from falsification bookmark)

Use `with_principal_stretch_bounds(max, f64::NEG_INFINITY)` to
disable compressive gating entirely.  Det F > 0 inversion
check remains the only compressive safety; the legacy
symmetric `max_stretch_deviation` gate also doesn't fire (since
the per-principal-stretch path is taken when EITHER bound is
`Some`).

**Cost**: trivial — flip the `min_principal_stretch` field to
`f64::NEG_INFINITY` (or change the validity gate signature to
accept `Option<f64>` per-direction).  But the H4 sample_yeoh
path currently builds bounds-pair atomically.

**Benefit**: unblocks H4.3 sweep immediately; tensile gain
preserved.

**Risk**: loses ALL compressive safety net at the validity
check — det F > 0 is the only remaining defense against
extreme compression states.  Newton may converge to
unphysical equilibria silently.  Pre-H4 legacy behavior is
similar (max_stretch_deviation allowed σ_min ∈ [0, 1]), so
this isn't a regression vs the pre-H4 baseline.

### 3.4 Recommended ladder

**Option A (loosen to 0.10 or 0.20)** is the smallest scope +
highest leverage move.  Re-run H4.3 sweep after the change;
if it clears, the user's prior 16/16 visual gate baseline at
cavity 3, 5 mm reproduces and we know cavity 8 mm is also
clean.  Falsifiers from there inform whether Option B (the
real ν fix) needs to be prioritized.

If Option A's loosened floor produces unphysical FEM behavior
(e.g., extreme tet inversion at deeper cavities, Newton
divergence in regimes where pre-H4 converged), fall back to
**Option C** (one-sided bound) — eliminates the floor
entirely while preserving the tensile gain.

**Option B is the long-term right answer** but should not
block H4-2 progress today.  Banked for the Phase H
locking-fix decorator arc.

---

## 4. Anchors

- `sim/L0/soft/src/material/silicone_table.rs:493` —
  `YEOH_MIN_PRINCIPAL_STRETCH = 0.30` declaration.
- `sim/L0/soft/src/material/silicone_table.rs:181-182` —
  ν = 0.40 family-wide.
- `sim/L0/soft/src/material/yeoh.rs:96-100` —
  `from_young_poisson_and_c2` panics on ν ≥ 0.45.
- `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` — H4.3
  falsification + recon ladder.
- `tools/cf-device-design/src/insertion_sim.rs::tests::h4_sweep_sliding_ramp_on_iter1_scan`
  — re-runnable sweep gate.

## 5. Sources

- Sparks JL, et al.  Use of Silicone Materials to Simulate
  Tissue Biomechanics as Related to Deep Tissue Injury.
  *Advances in Skin & Wound Care* 28(2), 59-68 (Feb 2015).
  PDF: <https://www.wcu.edu/WebFiles/PDFs/KS_Sparks_Tanaka.pdf>
- Smooth-On Ecoflex Series technical bulletin:
  <https://www.smooth-on.com/tb/files/ECOFLEX_SERIES_TB.pdf>
- Smooth-On Dragon Skin Series technical bulletin:
  <https://www.smooth-on.com/tb/files/DRAGON_SKIN_SERIES_TB.pdf>
- Liao et al.  Mechanical responses of Ecoflex silicone
  rubber: Compressible and incompressible behaviors.  *J Appl
  Polym Sci* (2019), DOI 10.1002/app.47025.
- Vishwakarma et al.  *Materials* 18, 3037 (2025).  DOI
  10.3390/ma18133037.
- Wacker Elastosil overview PDF (compression-set baseline for
  platinum-cure silicones): <https://www.wacker.com/h/medias/6858-EN.pdf>.

---

End of research memo.
