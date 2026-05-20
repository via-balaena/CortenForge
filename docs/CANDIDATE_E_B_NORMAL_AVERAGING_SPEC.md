# Candidate E.b — per-query normal averaging at the contact site

> **STATUS — SPEC.** 2026-05-18 NIGHT. F3 recon B candidate E.b
> per [[project-c2-sweep-falsification-bookmark]] §3 +
> [[project-c-prime-a-shipped]] §"What stayed dormant" — the next
> mechanism in the C → C′.a → E.b ladder to extend the converging
> cavity envelope past the C′.a-pinned 5 mm cap. Implementation
> ladder in §3-§4; sweep protocol at cavity = 6 mm in §6.
>
> **Parent docs**:
> - `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md` §3.E (E
>   candidates) + §8 (falsifier matrix) — the bookmark this spec
>   consumes.
> - `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` — C.0 spec, the
>   shape this spec follows (shorter, C′.a-brevity).
> - `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` — the grandparent
>   3-hypothesis bookmark (hyp 1 is what this spec attacks).
>
> **Predecessor memory**:
> [[project-c-prime-a-shipped]], [[project-c2-sweep-falsification-bookmark]],
> [[project-cavity-5mm-chattering-bookmark]],
> [[project-f3-recon-a-gated-lm-shipped]], [[project-f3-falsification-bookmark]],
> [[project-sl-4-arc-shipped]].

---

## TL;DR

C′.a found a 16/16 converging window at ε = 0.075 mm for
cavity ≤ 5 mm but stalls at cavity 6 mm (C.3 probe gate
r_norm = 0.536, 2 LM rescues). The C.2 falsification bookmark
left three hypotheses uninvestigated; **C′.a confirmed hyp 3
(band-widening backfire) on the upper side but hyp 1 (SDF normal
discontinuity) is still untested**.

E.b probes hyp 1 by smoothing the contact normal `n` at per-pair
query time: replace `n = prim.grad(p)` with
`n_avg(p) = normalize(Σ_i prim.grad(p + r·u_i))` over `k` offset
samples. `k = 1` (no averaging) is the bit-equal disabled state;
`k > 1` averages over `k - 1` axis-aligned offsets plus the
center sample. The composed Hessian's `n_avg ⊗ n_avg` is still
rank-1 PSD; the eigenstructure across active-set flips
smooths in proportion to `r` (offset radius). Compose with
C′.a's existing gap smoothing.

**Falsifier**: if no `(k, r)` in the swept space converges 16/16
at cavity = 6 mm, hyp 1 is FALSIFIED (or our k/r ranges were
wrong) and the next move is candidate F (step-0 warmup, hyp 2).

---

## 1. Problem statement

### 1.1 Hyp 1 (verbatim from the C.2 falsification bookmark §TL;DR)

> **SDF normal discontinuities** — C.0 spec §7 open question,
> never tested.  For mesh-derived SDFs
> (`mesh_sdf::SignedDistanceField`), the contact normal `n =
> ∂d/∂p` jumps at closest-triangle partition boundaries.
> Smoothing the energy ramp leaves the normal-direction
> chattering unaddressed.  ε = 0.25's chronic non-PD pivots
> stuck at one tet (index 1671) for ~20 iters smell like this:
> one vertex's contact normal is chattering across a triangle-
> partition boundary even though the gap function is smoothed.

### 1.2 What the actual contact normal looks like in our stack

The bookmark's hyp 1 wording presumes a triangle-direct SDF.
Our actual sliding-intruder stack is:

1. Cleaned scan `IndexedMesh` → mesh-sdf for sampled distance.
2. Samples deposited on a 3D grid (3 mm cells for iter-1) +
   **Gaussian-smoothed at σ = 1.0 cells** (see
   `GRID_SDF_SMOOTH_SIGMA_CELLS` docstring in `insertion_sim.rs`:1287).
3. `GridSdf` wraps the smoothed grid; `Sdf::grad` calls
   `gradient_clamped` which does **centered finite differences**
   (`(f(p+eps) - f(p-eps)) / (2·eps)`, `eps = 0.5·cell_size`,
   so `two_eps = cell_size`) on the trilinear-interpolated grid,
   then normalizes (`cf-geometry/src/sdf.rs:445`).
4. `TransformedSdf<GridSdf>` rotates the grad by the slide-pose
   rotation (`insertion_sim.rs:2188`).
5. `Solid::from_sdf(_, _).offset(_)` does not change the
   gradient (offset only shifts the iso-surface).
6. The grad is queried at the per-pair contact site in
   `PenaltyRigidContact::pair_contribution`'s consumers
   (`per_pair_readout` line 500, `gradient` line 571,
   `hessian` line 589 — see `sim/L0/soft/src/contact/penalty.rs`).

The **discontinuity in the contact normal field** in our stack
is therefore not at "triangle-partition boundaries" — it's at
**grid cell faces** where centered-FD on a trilinear-interpolated
grid produces piecewise-trilinear grad (C⁰ in `p` with derivative
jumps at the cell boundaries). After normalization, the unit
normal direction `n(p)` is continuous everywhere but has slope
discontinuities across the cell-face grid (an O(cell_size) grid
of slope kinks across the body bbox).

The pre-smoothing pass at σ = 1.0 cell reduces but does not
eliminate this — slope kinks at cell faces are a consequence
of the trilinear-interp + FD discretization, not of the source
SDF's regularity.

### 1.3 What E.b does

E.b averages the contact normal over a small neighborhood
around the query point, smoothing the cell-face slope kinks. At
per-pair query, replace `n = prim.grad(p)` with

```
n_avg(p) = normalize(Σ_{i=0..k} w_i · prim.grad(p + r·u_i))
```

with `u_0 = 0` (center sample), `u_1..u_6 = ±e_x, ±e_y, ±e_z`
(axis-aligned offsets), weights `w_i = 1` for now (equal-weight
mean of the k samples), and `r` an offset radius (free
parameter, default 1 mm = `d̂` so offset points stay within the
contact band).

`k = 1` (center sample only) is bit-equal to the pre-E.b
behavior. `k > 1` averages over the center + offsets, smoothing
the normal direction across grid cell faces within the radius
`r` neighborhood.

The **bookmark's "k nearest triangles" generalizes** to **"k
offset query points around the contact site"** for any Sdf
implementor — the mechanism doesn't require triangles to exist,
just an Sdf trait implementation. This is a strict
generalization of the bookmark's mechanism.

---

## 2. Mechanism + design choices

### 2.1 Offset-query averaging at the per-pair contact site

The averaging happens in sim-soft's `PenaltyRigidContact`,
inside the per-pair query path. **Only the normal `n` is
averaged** — the signed distance `sd` continues to come from
`prim.eval(p)` (single center query). This keeps the gap
function (and therefore C.1's gap-smoothing) bit-equivalent;
only the Hessian's `n ⊗ n` rank-1 direction and the gradient's
`n` direction inherit the averaging.

### 2.2 Discretization

The handoff suggested sweeping `k ∈ {2, 4, 8, 16}` but those
counts don't compose naturally with axis-aligned offset
directions. The spec pins the discretization to **axis-aligned
neighborhoods**:

| `k` | offset directions | rationale |
|---|---|---|
| 1 | none (center only) | identity; bit-equal disabled state |
| 7 | center + ±e_x, ±e_y, ±e_z | 6-face neighborhood; cheapest non-trivial averaging |
| 19 | k=7 + 12 edge-midpoint offsets `±(e_i ± e_j) / √2` for `i ≠ j` | 6-face + all 12 cube edges; symmetric across antipodal pairs |
| 27 | full 3×3×3 stencil (face + edge + corner offsets) | maximum smoothing per radius; reserved for §7 fallback |

**Default k = 7** is the iter-1 sweep choice. k = 19 / k = 27
are §7 fallback discretizations if k = 7 produces non-monotonic
or insufficient improvement at the swept `r` range. Note: the
handoff text loosely suggested `k ∈ {2, 4, 8, 16}` but those
counts don't compose with axis-aligned offsets; the spec pins
`{1, 7, 19, 27}` as the closed-set choices.

### 2.3 Offset radius `r`

`r` is the **physical scale of the averaging neighborhood**
(meters). Initial sweep values at cavity = 6 mm:

| `r` | physical scale | comment |
|---|---|---|
| 0.5 mm | ≈ ½ d̂; ≈ 1/6 grid cell | small smoothing — probes the boundary between disabled + meaningful |
| 1.0 mm | = d̂; ≈ 1/3 grid cell | matches the contact band; default |
| 2.0 mm | = 2·d̂; ≈ 2/3 grid cell | aggressive — risks averaging out the actual normal direction |

The U-shaped pattern C′.a found for ε is a real risk here too:
too-small `r` doesn't smooth enough; too-large `r` averages
across genuine surface curvature features. Default to bisection-
friendly sweep ordering (see §6).

### 2.4 Renormalization

After summing `k` normalized grads, the sum may have norm `< k`
(in regions of high normal-direction variation, the cancellations
shrink the sum). Renormalize: `n_avg = sum / ‖sum‖`. If
`‖sum‖ < 1e-10` (deeply pathological), fall back to the center
grad `prim.grad(p)`. This matches the existing degenerate-grad
fallback in `gradient_clamped` line 423-428.

### 2.5 Composition with C′.a's gap smoothing

E.b is **strictly orthogonal** to C′.a's gap-function smoothing:

- C′.a smooths the **gap function `R(sd)`** at the active-set
  boundary (`sd ∈ (d̂, d̂ + ε)`).
- E.b smooths the **normal direction `n(p)`** at the per-pair
  query.

The two compose multiplicatively in the assembled Hessian:
`H_pair = c.d2_energy_d_sd2 · (n_avg ⊗ n_avg)`. The
`c.d2_energy_d_sd2` factor inherits C′.a's smoothing (C⁰ at the
boundary); the `n_avg ⊗ n_avg` factor inherits E.b's smoothing
(C∞ inside the active band when k > 1, modulo the cell-face
slope kinks that average out).

The combined behavior at cavity = 6 mm is the empirical test
of the composition (§6).

### 2.6 Mirror-on-change invariant

The averaging logic lives in a single private helper
`PenaltyRigidContact::averaged_normal` (`§3.4`). The three
consumer sites (`per_pair_readout`, `ContactModel::gradient`,
`ContactModel::hessian`) all call this helper — never `prim.grad`
directly. Future changes to the averaging discretization update
the helper only.

This matches the C.1 spec's pattern (single `pair_is_active`
gate; five consumers call it). See `pair_is_active`'s
MAINTENANCE NOTE in `penalty.rs:399`.

---

## 3. sim-soft surface changes

### 3.1 New struct fields on `PenaltyRigidContact`

```rust
/// Number of offset samples for per-query normal averaging.
/// `1` (default) disables averaging — `n = prim.grad(p)` bit-equal
/// to pre-E.b behavior. `7` enables the 6-face axis-aligned
/// neighborhood. `13` / `27` are §7-reserved richer discretizations.
normal_avg_k: u8,
/// Offset radius (m) for per-query normal averaging. Only used
/// when `normal_avg_k > 1`. Default `0.0` (which the constructor
/// allows only when `normal_avg_k == 1`).
normal_avg_radius_m: f64,
```

Defaults set by the existing constructors (`new`, `with_params`,
`with_params_and_interior_cutoff`, `with_params_and_smoothing`,
`with_params_and_smoothing_and_interior_cutoff`) all initialize
both fields to `(1, 0.0)` — bit-equal pre-E.b.

### 3.2 New constructors

Two new constructors:

```rust
pub fn with_params_and_smoothing_and_normal_averaging<I>(
    primitives: I,
    kappa: f64,
    d_hat: f64,
    smoothing_eps_m: f64,
    normal_avg_k: u8,
    normal_avg_radius_m: f64,
) -> Self;

pub fn with_params_and_smoothing_and_normal_averaging_and_interior_cutoff<I>(
    primitives: I,
    kappa: f64,
    d_hat: f64,
    smoothing_eps_m: f64,
    normal_avg_k: u8,
    normal_avg_radius_m: f64,
    interior_cutoff: f64,
) -> Self;
```

Asserts on each:
- `normal_avg_k ∈ {1, 7, 19, 27}` (panic on other values — the
  discretization is closed-set; future discretizations re-spec).
- `normal_avg_radius_m >= 0.0 && is_finite`. Required `> 0.0`
  iff `normal_avg_k > 1` (the `k > 1` case with `r == 0.0` is a
  silent identity; assert disallows the misuse).
- Existing C.1 assertions on `smoothing_eps_m` + `interior_cutoff`
  preserved.

Naming: the 7-arg `with_params_and_smoothing_and_normal_averaging_and_interior_cutoff`
is long but follows the existing 5-arg naming convention. C.1's
spec went through the same length anxiety; "explicit
construction" beat "feature-flag struct" there + here too.

### 3.3 `pair_is_active` unchanged

Normal averaging affects `n` direction only. The active-set
predicate `sd < d̂ + smoothing_eps_m` is independent of `n` —
remains unchanged. Comment in `pair_is_active`'s MAINTENANCE
NOTE updated to mention E.b's orthogonal axis.

### 3.4 Private helper `averaged_normal`

```rust
/// Single-source-of-truth for per-pair contact normal computation.
/// At k == 1 returns prim.grad(p) bit-equal; at k > 1 averages
/// over the k - 1 axis-aligned offset points plus the center
/// sample, renormalized.
///
/// **MAINTENANCE NOTE** — any future change to the averaging
/// discretization (offset directions, weights, fallback policy)
/// updates this method only. The three consumer sites
/// (per_pair_readout, ContactModel::gradient, ContactModel::hessian)
/// call this helper rather than prim.grad directly; see §2.6 of
/// CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md for the mirror-on-change-
/// prevention rationale (parallel to pair_is_active's
/// MAINTENANCE NOTE for the active-band predicate).
///
/// Returns Vec3::z() as a documented degenerate fallback (matches
/// gradient_clamped's degenerate-grad behavior). Renormalization
/// failure (‖sum‖ < 1e-10) falls back to the center grad before
/// the Vec3::z() last-resort fallback.
fn averaged_normal(&self, prim: &dyn Sdf, p: Point3<f64>) -> Vec3;
```

Centralizes the smoothing logic so the three consumer sites
just call `self.averaged_normal(prim.as_ref(), p)`. Mirror-on-change
invariant per §2.6.

### 3.5 Three consumer sites updated

- `per_pair_readout` (penalty.rs:500): `let normal = self.averaged_normal(prim.as_ref(), p_pt);`
- `ContactModel::gradient` (penalty.rs:571): `let n = self.averaged_normal(prim.as_ref(), p);`
- `ContactModel::hessian` (penalty.rs:589): `let n = self.averaged_normal(prim.as_ref(), p);`

The helper signature takes `prim: &dyn Sdf`; call sites pass
`prim.as_ref()` (the `Box<dyn Sdf>` deref). Existing call shape
preserved otherwise.

### 3.6 Unit tests

New test file `sim/L0/soft/tests/penalty_normal_averaging.rs`
(mirroring `penalty_smoothing.rs`). Tests:

- **bit-equal at k=1**: averaged_normal at `k=1, r=0.0` returns
  `prim.grad(p)` exactly (use SphereSdf with known closed-form
  grad as the reference). Required for regression gate.
- **bit-equal at k=1, r>0**: same as above with `r=0.5e-3` —
  the helper ignores `r` when `k=1`.
- **axis-aligned k=7 on a plane**: SDF = RigidPlane, grad is
  constant `+z` everywhere; averaged at `k=7, r=0.5e-3` returns
  `+z` exactly (all 7 samples agree).
- **axis-aligned k=7 on a sphere**: SphereSdf r=1m, query at
  `(1, 0, 0)` with `r_avg=0.01m`, k=7: the 6 offsets are at
  `(1±0.01, 0, 0)`, `(1, ±0.01, 0)`, `(1, 0, ±0.01)`. The
  averaged normal still points outward radially (`+x`) by
  symmetry — but the exact value differs by 2nd-order curvature
  from the center grad. Assert `n_avg.normalize() ≈ +x` to a
  loose tolerance (1e-4 or curvature-aware).
- **degenerate renormalization fallback**: build a contrived
  Sdf that returns alternating +z / -z grads at the 6 offsets;
  sum = 0; helper falls back to center grad.
- **consumer parity**: per_pair_readout + gradient + hessian
  all flow through `averaged_normal`. Concretely (at
  `k=7, r=0.5e-3` on SphereSdf): for the same active pair,
  assert (a) `readout.normal` equals the `gradient`-computed
  `n` (the contribution direction is collinear with
  `readout.normal` up to the `c.d_energy_d_sd` scalar), and
  (b) the `hessian` 3×3 block equals
  `c.d2_energy_d_sd2 · (n_avg · n_avgᵀ)` with the same
  `n_avg = readout.normal`. No eigendecomposition needed —
  the rank-1 outer product is computed directly from the
  same averaged normal.

Total: ~6 unit tests, ~150 LOC for `penalty_normal_averaging.rs`.

---

## 4. cf-device-design wiring

### 4.1 New consts in `insertion_sim.rs`

```rust
/// Number of offset samples for per-query contact-normal averaging
/// (F3 recon B candidate E.b). `1` (default) disables averaging;
/// `7` enables the 6-face axis-aligned neighborhood. See
/// `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` for the design.
/// Initial value pending the cavity = 6 mm sweep; the E.b.4
/// case-A ship pins the value + mirrors the full `(k, r)` sweep
/// table in this docstring (parallel to
/// INSERTION_CONTACT_SMOOTHING_EPS_M's C′.a sweep-table mirror)
/// + updates the sentinel test below.
const INSERTION_CONTACT_NORMAL_AVG_K: u8 = 1;

/// Offset radius (m) for per-query contact-normal averaging.
/// Only used when INSERTION_CONTACT_NORMAL_AVG_K > 1. Default
/// `0.0` (matching the disabled state). E.b.4 case-A ship
/// re-pins to the swept-optimum value alongside K above.
const INSERTION_CONTACT_NORMAL_AVG_RADIUS_M: f64 = 0.0;
```

**3-surface mirror on E.b.4 case A** (parallel to C′.a's
3-surface mirror for the cavity-cap):
1. Both consts' docstrings carry the full `(k, r)` sweep table
   + the cavity ≤ 5 mm sanity-gate result.
2. The sentinel tests in §4.3 pin the new value.
3. The bookmark addendum (§9 ladder E.b.4) carries the
   case-A post-resolution data + the patterns banked.

Both initialize to the **disabled state** (`k=1, r=0`) —
bit-equal-when-dormant per
[[feedback-spec-falsified-revert-opt-in-keep-surface]]. The
E.b.2 commit lands the const + plumbing; the E.b.3 sweep finds
the converging `(k, r)`; the E.b.4 case-A commit re-pins the
const + raises the cavity-cap.

### 4.2 Both intruder builders re-routed

- `intruder_contact_at` (insertion_sim.rs:1008) → route through
  `with_params_and_smoothing_and_normal_averaging`. Add new
  4-tuple `(κ, d̂, ε, k, r)` constructor args.
- `intruder_contact_sliding_at` (insertion_sim.rs:2369) → route
  through `with_params_and_smoothing_and_normal_averaging_and_interior_cutoff`.

Bit-equal at the disabled state: `k=1, r=0` short-circuits in
`averaged_normal` to `prim.grad(p)` exactly, matching pre-E.b.

### 4.3 Sentinel tests

Mirror the C′.a `insertion_contact_smoothing_eps_m_sentinel`
pattern. New tests:

```rust
#[test]
fn insertion_contact_normal_avg_k_sentinel() {
    // Initial pin: 1 (disabled). Sweep result pins this to 7
    // (or whatever converges 16/16 at cavity = 6 mm).
    assert_eq!(INSERTION_CONTACT_NORMAL_AVG_K, 1u8); // edit on case A
}

#[test]
fn insertion_contact_normal_avg_radius_m_sentinel() {
    let expected = 0.0_f64; // initial; case A re-pins
    assert!((INSERTION_CONTACT_NORMAL_AVG_RADIUS_M - expected).abs() < 1e-12);
}
```

Plus an asserts-on-bounds sentinel:

```rust
#[test]
fn insertion_contact_normal_avg_consts_well_formed() {
    assert!(matches!(INSERTION_CONTACT_NORMAL_AVG_K, 1 | 7 | 19 | 27));
    assert!(INSERTION_CONTACT_NORMAL_AVG_RADIUS_M >= 0.0);
    assert!(INSERTION_CONTACT_NORMAL_AVG_RADIUS_M.is_finite());
    if INSERTION_CONTACT_NORMAL_AVG_K > 1 {
        assert!(INSERTION_CONTACT_NORMAL_AVG_RADIUS_M > 0.0);
    }
}
```

---

## 5. Falsifier matrix

Possible outcomes after the E.b.3 cavity = 6 mm sweep:

| Outcome | Meaning | Next |
|---|---|---|
| **Case A** — at least one `(k, r)` converges 16/16 at cavity = 6 mm + 3 + 5 mm sanity gates preserved (16/16 ZERO LM rescues) | E.b ships; pin `(k, r)`; raise the cavity-cap to 6 mm (or higher if 7 mm also converges). | E.b.4 case A: cap raise + sentinel update + bookmark §9 case-A section. |
| **Case B** — non-monotonic sweep response (analog to C.2's ε non-monotonicity) | Hyp 1 + the offset-averaging mechanism interact in a way the spec didn't predict — write a bookmark like C.2's. | E.b.4 case B: revert opt-in + write a falsification bookmark + escalate to F.a (κ ramp warmup, hyp 2). |
| **Case C** — every `(k, r)` swept produces r_norm worse than the C′.a baseline (cavity 6 mm r_norm = 0.536) | Hyp 1 is contributing in the wrong direction (averaging is shifting the normal toward something less productive than the cell-FD grad) | E.b.4 case C: revert + bookmark + escalate to F.a. |
| **Case D** — `k = 7` doesn't help but a richer discretization (k = 13 / 27) does | Discretization choice is the binding constraint, not the averaging mechanism. Re-spec with richer discretization as default. | Same as case A but with the richer k pinned. |
| **Case E** — 3 or 5 mm sanity gate at the chosen `(k, r)` drops below 16/16 converged | E.b composes badly with C′.a's gap smoothing — hard regression of the existing baseline. | E.b.4 case E: revert opt-in + bookmark + escalate to F.a. |
| **Case F** — sanity gates at 3 + 5 mm still converge 16/16 but engage LM rescue where C′.a did not, OR average solver iters per step increases meaningfully (>20%) | Mechanism is "active" outside the chattering envelope (smoothing the normal everywhere, not just at cell faces near active-set boundaries). Soft regression; ship only if cavity 6 mm gain outweighs it. | Case-A pin with the `(k, r)` re-framed as cavity-conditional (see §7.3); cap raise stays at 5 mm + add an opt-in path for cavity ∈ (5, 7]. |

---

## 6. Sweep protocol at cavity = 6 mm

The sweep walks `(k, r)` ordered by total averaging effort:

1. **`(k, r) = (1, 0)`** — baseline (bit-equal C′.a-only). Should
   reproduce the C.3 6 mm probe stall (r_norm 0.536, 2 LM
   rescues). If not, the wire-up is broken — bisect before
   continuing.
2. **`(k, r) = (7, 0.5e-3)`** — smallest non-trivial averaging.
   Per C′.a precedent, smallest non-trivial sample first to
   probe the "is the mechanism doing anything" question.
3. **`(k, r) = (7, 1.0e-3)`** — middle radius (= d̂).
4. **`(k, r) = (7, 2.0e-3)`** — large radius.
5. If 3 + 4 monotonically improve, sweep 7 + 2 more r samples
   at `{1.5e-3, 3.0e-3}` for the bisection extension.
6. If non-monotonic (case B above), bookmark + recon.

After finding the best `(k=7, r=*)`, sanity gates:

- **3 mm sanity gate** — must converge 16/16 with ZERO LM
  rescues (matches C′.a baseline). Regression here is case E.
- **5 mm sanity gate** — must converge 16/16 with ZERO LM
  rescues (matches C′.a 5 mm). Regression here is case E.
- **7 mm probe gate** — does the converging window extend
  past 6 mm? Stretch goal; not a falsification gate either way.

Estimated user time: ~5 min per sweep cycle × 4 baseline-then-3
sweep + 3 sanity = ~35 min user time. Cap rises to 6 mm
scaffolding (3-surface mirror) is part of E.b.3.

Per [[feedback-cf-cast-tests-use-release]] all sweeps run with
`--release`. Per [[feedback-long-running-commands-use-file-redirect]]
sweep logs redirect to file + tail.

---

## 7. Open questions

### 7.1 Discretization fallback

If `k = 7` produces non-trivial improvement but doesn't close
the gap to 16/16 at cavity = 6 mm, re-spec with `k = 13` or
`k = 27` as default. The implementation's `averaged_normal`
helper already accepts those; the spec change is just the
default const + the assert range. Possibly an E.b iter-2.

### 7.2 Non-uniform offset weights

Equal-weight averaging gives all `k` samples the same
contribution. Gaussian-weighted (weights ∝ `exp(-|u_i|² / 2σ²)`)
would emphasize the center. Deferred — equal-weight is the
simplest non-trivial choice + matches the bookmark's
"k nearest triangles" semantics (no weighting in the bookmark
wording). Re-spec only if case D surfaces.

### 7.3 Per-cavity `(k, r)` tuning

C′.a's ε = 0.075 mm doesn't generalize past 5 mm. If E.b's
`(k, r)` also has cavity-dependence, the right product
posture might be **cavity-conditional `(k, r)`**: at cavity ≤ 5
mm use `(1, 0)` (C′.a-only); at cavity ∈ (5, 7] mm use the E.b
pin. This adds a UI slider per
[[feedback-strip-the-knob-when-default-works]] — deferred
until empirical evidence. If case A passes the 3 + 5 mm sanity
gates with the E.b pin, no slider needed.

### 7.4 Smoothed-grad pre-computation cache

Per-pair query of `prim.grad(p)` at `k = 7` is 7× the work of
`k = 1`. For an iter-1 sock at cavity = 6 mm, the inner solve
does ~200 Newton iters, each evaluating ~10⁴ active pairs.
Wall-clock impact: ~7×–10× slowdown of the per-pair grad cost,
which is ~10-20% of total solve time. Estimated 1.5×–2×
overall slowdown — acceptable for the cavity = 6 mm regime
which is currently NON-CONVERGENT (no time at all).

If E.b ships + the slowdown is unacceptable on cavity ≤ 5 mm
where C′.a is already enough, **the cavity-conditional posture**
from §7.3 is the right product fix. Cache deferred.

### 7.5 Implication for the Hessian PSD argument

C.0 spec §5.1 ban on regularity-without-conditioning still
applies. E.b's `n_avg ⊗ n_avg` Hessian block is rank-1 PSD by
construction (any outer product of a non-zero vector with
itself is PSD). The smoothing improves the **alignment** of
`n_avg` across the active set's neighborhood — which is the
conditioning property the C.0 spec called out as missing.
Bank: E.b is the conditioning argument C.0 spec was missing.

---

## 8. Anchors

**Spec doc**: `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md`
(this file).

**Predecessor docs**:
- `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md` (§3.E, §8).
- `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` (C.0 spec — the
  shape this spec follows).
- `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` (3-hypothesis
  grandparent).
- `docs/F3_FALSIFICATION_BOOKMARK.md` (3-class failure
  model).

**Predecessor memory**:
- [[project-c-prime-a-shipped]] — direct predecessor, dormant
  E candidates noted in its §"What stayed dormant".
- [[project-c2-sweep-falsification-bookmark]] — bookmark with
  the 3 hypotheses + the E ladder.
- [[project-cavity-5mm-chattering-bookmark]] — class-2
  parent, partial-RESOLVED by C′.a; class-1 (LM rescue) is
  orthogonal.
- [[project-f3-recon-a-gated-lm-shipped]] — gated A is
  orthogonal class-1 LM rescue.

**Code sites (sim-soft, to modify)**:
- `sim/L0/soft/src/contact/penalty.rs:136-197` —
  `PenaltyRigidContact` struct + field additions.
- `sim/L0/soft/src/contact/penalty.rs:215-517` — `impl
  PenaltyRigidContact` with 5 existing constructors; add 2 new
  + private `averaged_normal` helper.
- `sim/L0/soft/src/contact/penalty.rs:500` — `per_pair_readout`
  grad call site.
- `sim/L0/soft/src/contact/penalty.rs:571` —
  `ContactModel::gradient` grad call site.
- `sim/L0/soft/src/contact/penalty.rs:589` —
  `ContactModel::hessian` grad call site.
- `sim/L0/soft/tests/penalty_normal_averaging.rs` — new test
  file mirroring `penalty_smoothing.rs`.

**Code sites (cf-device-design, to modify)**:
- `tools/cf-device-design/src/insertion_sim.rs:999` —
  `INSERTION_CONTACT_SMOOTHING_EPS_M` neighbor; add the two
  new normal-averaging consts immediately after.
- `tools/cf-device-design/src/insertion_sim.rs:1008` —
  `intruder_contact_at` builder; re-route through new
  constructor.
- `tools/cf-device-design/src/insertion_sim.rs:2369` —
  `intruder_contact_sliding_at` builder; re-route through new
  cutoff constructor.
- `tools/cf-device-design/src/insertion_sim.rs` `mod tests` —
  new sentinel tests per §4.3.

**Code sites kept untouched (don't modify)**:
- C′.a-pinned `INSERTION_CONTACT_SMOOTHING_EPS_M = 0.075e-3`
  (the gap-smoothing const is orthogonal).
- C′.a-pinned cavity-cap (5 mm) — E.b.4 case A raises this
  if (and only if) cavity 6 mm converges.
- Gated A's `try_solve_impl` + `try_gated_factor_solve_armijo`
  (orthogonal class-1 LM rescue).
- `catch_unwind` belt-and-suspenders in
  `run_sliding_insertion_ramp` (class-3 Yeoh safety).
- LmConfig::fork_b opt-in.
- C.1 sim-soft surface (`with_params_and_smoothing*`
  constructors stay — they remain bit-equal-when-dormant).
- mesh-sdf D arc primitives (`TriMeshDistance`, `FloodFillSign`,
  `CachedGridSdf`) — out-of-arc.
- mesh-offset / mesh-lattice / cf-cast / cf-scan-prep /
  cf-cast-cli — out-of-arc.

---

## 9. Implementation ladder

Mirrors the C.0 → C.1 → C.2 → C.3 cadence:

- **E.b.1** — sim-soft primitive + unit tests (~150 LOC).
  Bit-equal-when-k=1 contract via the new constructors. New
  test file `penalty_normal_averaging.rs` (~150 LOC). Cold-read
  pass-1 + pass-2 after.
- **E.b.2** — cf-device-design opt-in (~50 LOC). New consts
  pinned at disabled state (`k=1, r=0`); both intruder builders
  re-routed through new constructors; sentinel tests added.
  Cold-read pass-1 + pass-2 after.
- **E.b.3** — sweep at cavity = 6 mm (3-surface cavity-cap
  scaffolding raise + user-driven scrub cycles). Bookmark
  addendum (case A pin or case B bookmark).
- **E.b.4** — case A (3-surface cap-raise mirror per the C′.a
  §9.5 pattern — `CavityState::inset_slider_range_m` updates to
  `(0.0, 0.006)`, egui label updates with the E.b ship rationale,
  cavity-cap sentinel test renamed + asserts the new max + carries
  the case-A ship rationale + the new const sweep table — plus
  bookmark `docs/CANDIDATE_C_SWEEP_FALSIFICATION_BOOKMARK.md`
  addendum + patterns banked) OR case B (revert opt-in + bookmark
  + escalate to F.a). Cold-read pass-1 + pass-2 after the
  non-trivial ship commit per
  [[feedback-cold-read-two-passes-for-non-trivial-diffs]].

Per [[feedback-strip-the-knob-when-default-works]] no UI
surface for the new consts in any case — pure programmer-set
defaults.

Per [[feedback-workaround-removal-verification]] pre-existing
safety nets (catch_unwind, LM rescue, C′.a smoothing) stay
belt-and-suspenders through E.b.

---

End of spec.
