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
- **weights** = 1/4 each (canonical baseline).

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
| `p_th` | "a small fraction of the material's tensile strength" | **no number** |
| `m̂` | barrier engagement margin | **none given** |
| `m̂_k` | stiffness barrier tolerance | **none given** |
| `k_min` | "material- and application-dependent minimum" | **no source** |

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
   monotone increase under injected spread. Scale invariance: doubling every
   `p_i` leaves `J_unif` unchanged (this is the CoV property, and it is the
   discriminator against a std-dev implementation).
3. **Coverage**: `→ 1` when every Γ element is loaded above `p_th`, `→ 0` when
   none is; strictly between otherwise. Guard the denominator: coverage must
   *fall* when Γ grows with the loaded set fixed.
4. **Peak**: `p̂_peak ≤ max(p_i)` always, and within a stated percentage of it
   on the real field — measured, not assumed. `B_peak = 0` when `m ≥ m̂`;
   finite and increasing as `m → 0⁺`.
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
