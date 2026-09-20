# Wiring `RewardBreakdown` onto the insertion ramp — spec

> **Status:** SPEC, pre-implementation. Written before any code so a wrong
> formula is cheap to fix.
>
> **What this specifies:** how the four conformity-reward terms are computed
> from an insertion-ramp step, with exact formulas, sign conventions, the
> intended-contact-surface derivation, and every constant's provenance.
>
> **What it does NOT:** it does not tune the free constants (§6.3), does not
> claim the resulting score is calibrated against any physical measurement, and
> does not touch the solver. Recon and citations verified at `6dc7ee03`.

## 1. Why this is item 2 of the renovation

`docs/INSERTION_SIM_TET10_RENOVATION_RECON.md` §8 orders the work by *when each
step becomes measurable*. This is the oracle step: example row 25
(`scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp-open-mouth`) is the geometry
closest to the product and is a **demo, not a validator** — its interference-fit
load case inverts the force sign and strain ordering its sibling rows gate on,
so it has no physics oracle. ⛔ **Until this lands, no solver change below it in
§8 is measurable — only different.**

## 2. The hook point

`aggregate_step_readout(per_tet: &[TetReadout], contact_readouts: &[ContactPairReadout])
-> StepReadout` (`tools/cf-sim-research/src/insertion_sim.rs:1910`) already
receives everything the reward needs and **discards it**:

```rust
pub struct ContactPairReadout {
    pub pair: ContactPair,
    pub position: Vec3,
    pub sd: f64,
    pub normal: Vec3,
    pub force_on_soft: Vec3,
    pub tributary_area: f64,   // <- dA
    pub pressure: f64,         // <- p(x)
}
```

Today it keeps `n_active_contact_pairs` and the force sum. ⇒ **No new plumbing
is required to obtain per-element pressure and area.** They are in hand at the
one function that throws them away.

`peak_contact_pressure` (`contact/mod.rs:283`) already reduces the same slice,
but to a **true max**, which §5.3 replaces with a smooth `L^q` max.

⚠ On a Tet10 mesh these readouts are face-consistent only as of rung 8d; before
it, midside nodes got `tributary_area = 0` ⇒ `NaN` pressure ⇒ silently dropped.
Any consumer must treat a non-finite `pressure` as a defect, never as zero.

## 3. Γ — the intended contact surface

Both `J_unif` and `J_cov` integrate over Γ, and coverage's denominator is
**|Γ|**, deliberately not the active region:

> *"computing the mean only where p > p_th would make the denominator depend on
> the numerator and coverage would trivially score 1 regardless of extent."*
> — `10-physical/01-reward/01-coverage.md`

⛔ **Γ does not exist in `sim-soft`.** There is no surface-subset tagging of any
kind (verified against controls: `boundary_faces` 98 hits, `interface_flags` 39,
`materials` 145; every tag-like term 0). `Mesh::boundary_faces()` is the **whole**
boundary — outer envelope, cavity wall and rim together.

Using the whole boundary is **not** an acceptable fallback: the outer envelope
can never contact the intruder, so coverage would be structurally capped far
below 1 and the number would mean nothing.

### 3.1 Derivation, not tagging

`insertion_sim` already knows the cavity analytically — it is the scan SDF at a
known offset (`insertion_sim.rs:556`, "Scan-SDF offset (m) of the cavity surface
— `-cavity_inset_m`"). So:

```text
face f ∈ Γ   ⟺   | scan_sdf(centroid(f)) − cavity_offset | < tol
```

★ This mirrors the rule already shipped for `Mesh::interface_flags` —
`|φ(x_c)| < L_e` with `L_e` the arithmetic mean of the tet's six edge lengths
(`mesh/mod.rs:230`). Adopt the same shape and the same scale for `tol` rather
than inventing a new one.

**Why derive rather than tag:** no new mesh channel, no `Mesh` trait change, and
Γ stays correct automatically as `cavity_inset_m` changes — which matters
because the inset is the design variable the optimizer will move.

⚠ Open: the rim band. A face on the rim fillet is neither clearly cavity nor
clearly envelope, and §4's Γ test will include or exclude it depending on `tol`.
This is the same band `04-rim.md` identifies as where all four reward terms are
most fragile. **Gate on the Γ face count being stable under `tol` perturbation**
(§7), and record the sensitivity rather than assuming it away.

## 4. Sign conventions — verified, all four

| term | quantity | sense | contribution |
|---|---|---|---|
| uniformity | `J_unif` (weighted CoV²) | lower better, optimum 0 | `R_unif = −J_unif` |
| coverage | `J_cov` ∈ [0,1] | higher better | `R_cov = +J_cov` |
| peak | `B_peak` (barrier) | lower better, 0 when slack | `R_peak = −B_peak` |
| stiffness | `B_stiff` (barrier) | lower better, 0 when slack | `R_stiff = −B_stiff` |

⇒ All four are higher-is-better after sign conversion, so
`RewardBreakdown::score_with` (already implemented, fixed-weight sum with
`NaN`-dropping) is the correct composition — mode 1 of
`01-reward/04-composition.md`, canonical baseline `w = 1/4` each.

## 5. The four terms

Let `p_i` and `a_i` be the pressure and tributary area of contact readout `i`
restricted to Γ, and `A = Σ_{f ∈ Γ} area(f)` the full intended-surface area.

### 5.1 Pressure uniformity — weighted squared CoV

```text
w(p)   = logistic(β_w (p − p_th))
p̄      = Σ w(p_i) p_i a_i / Σ w(p_i) a_i
J_unif = (1/p̄²) · Σ w(p_i)(p_i − p̄)² a_i / Σ w(p_i) a_i
```

⚠ **Squared, not a standard deviation** — `√x` is non-smooth at `x = 0`, which
is exactly the global optimum the optimizer converges onto. ⚠ **CoV, not
absolute spread** — keeps the term scale-invariant so the weight does not need
retuning when operating pressure changes.

### 5.2 Coverage — area-weighted mean of a smooth indicator

```text
c(x)  = logistic(β_c (p(x) − p_th))
J_cov = (1/A) · Σ_{i ∈ Γ} c(p_i) a_i
```

⚠ Denominator is `A` (full Γ), per §3. ⚠ Same `p_th` as §5.1 **by
construction** — the two terms must agree on what counts as contacted.
⚠ Logistic, not a binary indicator: a step makes the derivative w.r.t. any
geometric parameter a delta function at contact-boundary flips.

### 5.3 Peak-pressure barrier

```text
p̂_peak = ( (1/A) · Σ_{i ∈ Γ} p_i^q a_i )^(1/q)          q ∈ [8, 16]
m      = p_max − p̂_peak
B_peak = −(m − m̂)² · log(m / m̂)   for 0 < m < m̂
       = 0                          for m ≥ m̂
       = +∞                         for m ≤ 0
```

Inverted IPC barrier: `m` plays the role of gap `d`, `m̂` of barrier width `d̂`.
`C²` at `m = m̂`, diverges at the ceiling.

### 5.4 Effective-stiffness barrier

Prescribed-axial-engagement mode, which is what the ramp does:

```text
k_eff   = F_ax / δ
m_k     = k_eff − k_min
B_stiff = same inverted IPC form with tolerance m̂_k
```

`F_ax` and `δ` are already paired in
`InsertionResult::force_displacement_curve: Vec<(interference_m, force_N)>`.

## 6. Constants — three tiers, and the third is the problem

### 6.1 Sourced

- **`p_max` ← tensile strength**, per `appendices/03-notation.md`:
  *"Peak-pressure ceiling (per-material, read from tensile strength)."*
  ⛔ Take it from the **Smooth-On TDS**, not from the study appendix — see §8.1.
- **`m̂` = 10–25 % of `p_max`**, per `02-peak-bounds.md:33`: *"Setting `m̂` at a
  fraction of the ceiling (typical: 10–25 % of `p_max`) keeps the barrier's
  active region scale-consistent across materials without per-material
  retuning."* ⇒ adopt **15 %** (mid-range) and name it, so a later change is a
  one-line edit against a stated range rather than a hunt for a literal.
- **weights** = 1/4 each (canonical baseline). ⚠ These only mean what they say
  once the barrier is normalised per §8.5.

### 6.2 Derived

- **`q` ∈ [8, 16]** — "large enough to approximate the true maximum to a few
  percent". Pick and *measure* the approximation error against the true max
  (§7), rather than asserting it.
- **`β_w`, `β_c`** — the book fixes these by a stated condition: *"sharpness β_w
  chosen so `w` saturates within ±`p_th`/2 of the threshold."* Taking
  "saturates" as logistic within 1 % of its asymptote (`logistic(±4.6) ≈
  0.99/0.01`):

  ```text
  β_w · (p_th/2) ≈ 4.6   ⇒   β_w ≈ 9.2 / p_th
  ```

  ⚠ The 1 % reading of "saturates" is **mine, not the book's** — it is the free
  choice inside a derived constant, and it must be labelled as such in code.

### 6.3 ⛔ FREE — no published value

| constant | what the book says | status |
|---|---|---|
| `p_th` | "a small fraction of the material's tensile strength" | ⛔ **no number** |
| `β_c` | "typically somewhat larger (steeper) than `β_w`" | ⛔ **relative only** |
| `m̂_k` | stiffness barrier tolerance | ⛔ none given — **moot**, `stiffness_bound` is `NaN` |
| `k_min` | "material- and application-dependent minimum" | ⛔ no source — **moot**, see below |

⚠ **Correction:** an earlier draft of this spec listed `m̂` here. It is **not**
free — `02-peak-bounds.md:33` sets it explicitly: *"Setting `m̂` at a fraction
of the ceiling (typical: **10–25 % of `p_max`**) keeps the barrier's active
region scale-consistent across materials without per-material retuning."*
It belongs in §6.1 as sourced.

★★★ **These must not be buried as literals.** Each is a named, documented input
carrying either its derivation or an explicit *"chosen, not measured; here is
what would replace it."* A magic number inside the function that defines "good"
for the whole product is the worst place in the codebase for an unfalsifiable
constant.

⚠ **`k_min` has no material basis at all** — unlike `p_max`, nothing in the
material data implies it. It is an application requirement ("the sleeve must
transmit at least X N per mm of engagement"), and it does not exist yet. Until
it does, `stiffness_bound` should be `NaN` (which `score_with` already drops)
rather than computed against an invented floor.

## 7. Verification plan — how each term gets a non-vacuous gate

⛔ Every gate below must be made to FAIL before it is believed.

1. **Γ**: face count > 0; Γ ⊊ all boundary faces (strict); **stability under
   `tol` perturbation** (§3.1's rim-band risk). Negative control: a Γ test with
   `tol = ∞` must select every face and trip the strictness gate.
2. **Uniformity**: `J_unif = 0` for a synthetic uniform pressure field;
   monotone increase under injected spread.

   ⛔ **Scale invariance is ASYMPTOTIC, not exact — an earlier draft of this
   spec asserted exact invariance, and that gate would have REJECTED a correct
   implementation.** `w(p) = logistic(β_w(p − p_th))` depends on *absolute*
   pressure through the fixed threshold, so doubling every `p_i` moves elements
   deeper into saturation and changes the weights. Measured on a 4-element
   field at `p_th = 5 %` of Ecoflex 00-30's tensile strength:

   ```text
   pressures near p_th :  J(p)=0.19843191  J(2p)=0.20000000   rel-diff 7.9e-03
   pressures at  10x   :  J(p)=0.20000000  J(2p)=0.20000000   rel-diff 0
   pressures at 100x   :  J(p)=0.20000000  J(2p)=0.20000000   rel-diff 0
   ```

   ⇒ The gate must assert invariance **only in the saturated regime**
   (all `p_i ≳ 10 p_th`), and separately assert that the near-threshold regime
   *does* differ — otherwise the gate cannot tell a weighted CoV from an
   unweighted one, which is the thing it exists to distinguish.
3. **Coverage**: `→ 1` when every Γ element is loaded above `p_th`, `→ 0` when
   none is; strictly between otherwise. Guard the denominator: coverage must
   *fall* when Γ grows with the loaded set fixed.
4. **Peak**: `p̂_peak ≤ max(p_i)` always. ⛔ **Record `p̂_peak / max(p_i)` on
   the real field as an output**, per §8.4 — the approximation error is
   field-dependent and 13–25 % at `q = 16` for a narrow contact band, so it
   must be visible, not assumed. `B_peak = 0` when `m ≥ m̂`; finite and
   increasing as `m → 0⁺`; `+∞` only at `m ≤ 0`.
5. **Stiffness**: `k_eff` matches a hand computation off
   `force_displacement_curve` at a chosen step.
6. **Composition**: `score_with` on a known breakdown equals the hand-summed
   weighted value, including the `NaN`-drop path.

⚠ A uniform synthetic field makes gates 2 and 3 vacuous in the same way a
uniform material field made the Tet10 carry-gate vacuous (PR #949) — **vary the
field** and assert the values are distinguishable.

## 8. Findings from recon that change what gets written

### 8.1 ⛔ The study's tensile table disagrees with the TDS on one grade

Cross-checked `appendices/02-material-db.md` against the Smooth-On series
bulletins (retrieved and digest-checked in PR #947):

| grade | TDS | study appendix | Δ vs TDS |
|---|---|---|---|
| Ecoflex 00-10 | 120 psi = 827.4 kPa | ≈800 kPa | **−3.3 %** (low) |
| Ecoflex 00-20 | 160 psi = 1103.2 kPa | ≈1100 kPa | −0.3 % |
| Ecoflex 00-30 | 200 psi = 1379.0 kPa | ≈1380 kPa | +0.1 % |
| Ecoflex 00-50 | 315 psi = 2171.8 kPa | ≈2170 kPa | −0.1 % |
| Dragon Skin 10A | 475 psi = 3275.0 kPa | ≈3.3 MPa | +0.8 % |
| Dragon Skin 20A | 550 psi = 3792.1 kPa | ≈3.8 MPa | +0.2 % |
| **Dragon Skin 30A** | **500 psi = 3447.4 kPa** | **≈3.6 MPa** | ✗ **+4.4 %** (high) |

⛔ **Direction matters more than magnitude here, and only one row errs the
unsafe way.** `B_peak` diverges *at* `p_max`, so a ceiling set **high** tells the
optimizer it has headroom the material does not have. Dragon Skin 30A is the
only grade above +1 %, and it is high. Ecoflex 00-10's −3.3 % is the
second-largest deviation but errs **low**, which is merely conservative.

⇒ **Seed `p_max` from the TDS.** Do not read it from the study appendix.

### 8.2 ⚠ Tensile strength is NOT monotonic in durometer

Dragon Skin 30 (500 psi) is **weaker** than Dragon Skin 20 (550 psi), in the
published data. `silicone_table.rs` already asserts
`mu_is_non_decreasing_along_hardness_order()`, so the natural instinct when
adding a tensile field — extend the ordering invariant to it — **produces a test
that fails on correct data.** Document the non-monotonicity at the field.

### 8.3 ⚠ `RewardBreakdown`'s field docs describe different quantities

The struct's doc comments say *hydrostatic pressure across the domain*, *Cauchy
stress vs material bound*, and *material tangent eigenspectrum*. This spec
computes the **book's** quantities — contact pressure over Γ, a
tensile-strength ceiling, and transmitted force. The only existing constructor
(`observable/basic.rs:135`) implements neither: it is a Stage-1 skeleton whose
`peak_bound` is a sum of displacement DOFs and whose `stiffness_bound` is
`θ[0]/peak_bound`, with two terms hardcoded `NaN`.

⇒ **Decision: follow the book.** The canonical problem *is* this problem, and
the field docs read as placeholders written before the terms were specified.
The struct's doc comments must be corrected in the same change, or the next
reader inherits the same three-way disagreement.

## 8.4 ⛔ `L^q` accuracy depends on the peak's AREA FRACTION, not just `q`

The book says `q ∈ [8, 16]` approximates the true maximum *"to a few percent"*.
Measured against a synthetic field where the peak occupies a varying fraction
of Γ (200 elements, peak at 1.0, remainder at 0.02):

| peak area fraction | q=8 | q=16 | q=24 | q=32 |
|---|---|---|---|---|
| 50 % | 8.3 % | 4.2 % | 2.8 % | 2.1 % |
| 25 % | 15.9 % | 8.3 % | 5.6 % | 4.2 % |
| 10 % | 25.0 % | 13.4 % | 9.1 % | 6.9 % |
| 5 % | 31.2 % | 17.1 % | 11.7 % | 8.9 % |
| 1 % | 43.8 % | 25.0 % | 17.5 % | 13.4 % |

(values = **underestimate** of the true max)

⇒ *"a few percent"* holds only when the peak covers roughly half of Γ. A
realistic insertion contact is a **narrow band**, where `q = 16` reads 13–25 %
low.

⛔⛔ **And the error is under-protective, in the same direction as §8.1's
ceiling error.** `m = p_max − p̂_peak`; an underestimated `p̂_peak` inflates the
margin, so the barrier reports slack that does not exist. Two independent
under-protective errors compound.

★ **Therefore `q` is NOT a free pick from `[8, 16]`.** The implementation must
**measure** `p̂_peak / max(p_i)` on the real field and record it alongside the
score, so the approximation error is visible rather than assumed. §7 gate 4
asserts this.

## 8.5 ⛔ The four terms are NOT commensurate — equal weights do not work

`J_unif` and `J_cov` are dimensionless and O(1). `B_peak` carries units of
**pressure²** and its magnitude scales as `m̂²`:

```text
m̂ = 1e2 Pa   B(m = 0.5 m̂) = 1.73e3    B(m = 0.1 m̂) = 1.87e4
m̂ = 1e3 Pa   B(m = 0.5 m̂) = 1.73e5    B(m = 0.1 m̂) = 1.87e6
m̂ = 1e4 Pa   B(m = 0.5 m̂) = 1.73e7    B(m = 0.1 m̂) = 1.87e8
```

⇒ With the canonical equal weights `w = 1/4`, **`B_peak` swamps the other terms
entirely** — the composed score would be a barrier reading with rounding noise
from uniformity and coverage.

⚠ **The book does not address this.** `04-composition.md` discusses rescaling
the *weights* (`Σ w_i = 1`) but never the terms' relative magnitudes, and the
barrier leaves never state their units.

★ **Resolution adopted here: normalise the barrier to be dimensionless** by
dividing by `m̂²`, i.e. report `B_peak / m̂²`. This preserves the `C²` shape, the
zero at `m = m̂` and the divergence at `m = 0`, while putting the term on the
same O(1) footing as uniformity and coverage so the canonical equal weights
mean what they say. ⚠ **This is a deviation from the book and must be recorded
as one** — it changes the composed score's absolute value, not its argmin at
fixed weights.

## 8.6 ⚠ Numerical hazards found by direct test

- **`L^q` overflow.** `Σ p_i^q` with `q = 16` and `p ~ 1e8 Pa` reaches `1e128`.
  It does not overflow `f64` at realistic pressures, but the **normalised** form
  `max(p) · (Σ (p_i/max(p))^q a_i / A)^(1/q)` is free insurance and was verified
  bit-equal to the naive form across `1e3 … 1e8` Pa. Use it.
- **`logistic` overflow.** `β_w ≈ 9.2/p_th` with small `p_th` makes the exponent
  large; guard the `exp` (return 0 below ≈ −700) rather than relying on `f64`
  saturation.
- **Barrier growth is logarithmic, i.e. SOFT.** `B` reaches only 1.8e5 at
  `m = 1e-6 · m̂`. It is `+∞` only at `m ≤ 0` exactly. Do not assume the barrier
  alone keeps a search away from the ceiling.
- **Degenerate inputs return `NaN`, which is correct** — empty contact and
  all-zero pressure both make the weighted denominator zero. `score_with`
  already drops `NaN`. A single contact point returns `J_unif = 0` (perfect
  uniformity on one sample), which is *semantically* wrong on its own and is
  exactly why coverage is a separate term — the composition catches it, the
  term cannot.

## 9. What this spec does not settle

1. **Whether the resulting score is physically meaningful.** It is computable
   and internally consistent; nothing here calibrates it against a measured
   part. `01-reward.md` Claim 3 says the reward is defined on solver-exported
   fields precisely so it *can* later be evaluated on a real cast — that
   comparison is not in scope.
2. **`k_min`** — §6.3. Until an application requirement exists,
   `stiffness_bound` stays `NaN`.
3. **The rim band** — §3.1. Gated for stability, not resolved.
4. **Tet4 vs Tet10.** Nothing here depends on element order, but the pressure
   field it consumes is only face-consistent on Tet10 (rung 8d). On the current
   Tet4 path the numbers will be per-vertex point forces, which §2's ⚠ flags.
   **This spec is computable before the Tet10 adoption and more trustworthy
   after it.**
