# Candidate C — smoothed-contact penalty barrier — design spec

> **STATUS — DRAFT.** Recon-iter-1 of `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md`
> §4 sub-leaf ladder. The bookmark surfaced class-2 active-set chattering
> as the binding pathology at cavity ≥ 5 mm and proposed smoothed contact
> as the next-arc mechanism; this spec is the recon that picks an actual
> ladder for implementation. Per
> [[feedback-bookmark-when-surface-levers-exhaust]] three-session
> pattern: bookmark already shipped, this spec is the recon session, the
> implementation (C.1 sim-soft primitive + C.2 cf-device-design opt-in)
> is a separate session because total scope (~300 LOC sim-soft + ε
> calibration sweep) exceeds the implement-measure-revert envelope.

**Predecessor docs**:
- `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` — the bookmark this spec acts
  on. §2 verbatim Gate B trace + 7-row class-2 mental-model table are
  the empirical floor candidate C must cross. §3 mechanism preview is
  PROVISIONAL — this spec replaces it with an implementation-grounded
  prescription informed by reading `penalty.rs` internals.
- `docs/F3_RECON_A_GATED_LM_SPEC.md` — the immediate predecessor recon.
  §F3.1 bit-equal-when-dormant contract is inherited. §2.5 "no new
  SolverFailure variant" stance is also inherited (candidate C is a
  contact-model upgrade; no solver-API surface change).
- `docs/F3_FALSIFICATION_BOOKMARK.md` — the parent falsification arc.
  §2 three-failure-class mental model identifies class 2 (active-set
  discontinuity) as the class candidate C addresses; classes 1
  (indefinite tangent) + 3 (Yeoh material validity) are orthogonal arcs.
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — the original arc all
  candidates target. §8 Run 2 baseline `r_norm = 1.78` after ~5 iters
  is the apples-to-apples pre-F3 baseline.

**Predecessor memory**: [[project-cavity-5mm-chattering-bookmark]] (direct
predecessor), [[project-f3-recon-a-gated-lm-shipped]] (sibling recon
this spec mirrors structurally), [[project-f3-falsification-bookmark]],
[[project-cavity-inset-stall-bookmark]],
[[project-f4-falsification-postmortem]], [[project-sl-4-arc-shipped]].

---

## TL;DR

Gated A (F3 recon A) restored the cavity = 3 mm baseline but left
cavity = 5 mm at a structural `r_norm ≈ 1.784` floor that LM rescue
cannot cross — the bookmark's diagnosis is **class-2 active-set
chattering** in `PenaltyRigidContact`: when a Newton step crosses an
active-pair on/off boundary (`sd = d̂`), the **per-pair** Hessian
contribution `H_pair = κ · n⊗n` (rank-1) appears or disappears
discontinuously. The full `H_contact(x) = Σ_pairs H_pair` is the sum
over active pairs (rank up to `3 · n_active`); its discontinuity at
active-set flips breaks the quadratic regularity Newton's contraction
estimate depends on.

**Candidate C's mechanism**: mollify the penalty energy so the
contact Hessian is C⁰ across the active-pair boundary. The bookmark
§3 sketched multiplicative `(d̂-sd)² · smooth_step(d̂-sd, ε)`;
this spec keeps that shape but tightens the prescription to a
**polynomial taper over `[d̂, d̂+ε]`** (extends the active band by
`ε`; full-penalty formula unchanged for `sd ≤ d̂`; smooth ramp-to-zero
for `sd ∈ (d̂, d̂+ε)`; zero for `sd ≥ d̂+ε`). The polynomial choice
(cubic-Hermite C¹ vs quintic-Hermite C² vs higher) is a C.1
implementation decision; the spec mandates **at minimum C¹** at the
inactive end (so the gradient is continuous across pair flips) and
**recommends C² as the empirical default** (so the Hessian is also
continuous, addressing the chattering directly).

**Bit-equal-when-dormant contract**: at `ε = 0`, all formulas reduce
bit-for-bit to today's `PenaltyRigidContact`. The new `ε > 0` opt-in
is the cf-device-design Fork-B-only change; sim-soft production
callers + existing fixture tests stay bit-equal.

**The empirical bet**: at cavity = 5 mm, smoothed contact with an `ε`
in the empirical sweep range (likely `ε ∈ [0.1, 2.0] · d̂` where
`d̂ = 1 mm`) eliminates the chattering floor; Newton converges below
`tol = 1e-1` in a reasonable iter budget. At cavity = 3 mm, the
smoothed-contact run remains 16/16 (deep-contact pairs at `sd ≪ d̂`
are bit-equal to the hard penalty; only pairs in the boundary band
`[d̂, d̂+ε]` see the polynomial taper). The converged fixed point
may differ from pre-F3 by `O(ε)` at near-boundary contributors (the
smoothing taper shifts the local force balance in the band), but the
3 mm baseline's deep-contact regime is bit-equal so 16/16 convergence
is preserved by construction. If the empirical sweep finds NO `ε`
that clears cavity = 5 mm, the falsifier escalates to candidate D
(mesh refinement) or sliding-ramp restart with sub-step time
integration.

Cavity = 8 mm is **OUT OF SCOPE** for this spec — class 3 (Yeoh
material validity at tet 1078 / 1324) is orthogonal. Candidate B
addresses class 3 separately.

---

## 1. The three failure classes (inherited mental model)

Direct lift from `docs/F3_FALSIFICATION_BOOKMARK.md` §2. Repeated here
so this spec is self-contained for the cold reader; phrasing tightened
to candidate C's framing.

1. **Indefinite tangent / non-descent direction** — addressed by F3
   gated A (`docs/F3_RECON_A_GATED_LM_SPEC.md`). Class-1 LM rescues
   are EMPIRICALLY confirmed at cavity = 5 mm Gate B (1 LM escalation
   at iter 57, λ = 6.67e-3, 1 retry, Armijo-accepted). **Candidate C
   does NOT address this class** — gated A is the right lever and
   stays as-is.

2. **Active-set discontinuity in `H_contact(x)`** — the class candidate
   C addresses. The contact Hessian `H_pair = κ · n⊗n` is rank-1 when
   the pair is active (`sd < d̂`) and rank-0 when inactive. As a
   Newton step crosses `sd = d̂` for any pair, `H_contact(x)` jumps in
   the FUNCTION VALUE (not just eigenstructure). No diagonal additive
   `+λI` can smooth a value discontinuity. The empirical signature is
   the cavity = 5 mm `r_norm ≈ 1.784` floor that gated A's extended
   57-iter Newton walk asymptotes to.

3. **Yeoh material validity at deep insets** — Phase 4 Decision Q
   fail-closed surface (`backward_euler.rs:678`). At cavity = 8 mm
   F3.4 Gate C reached `max_stretch_deviation = 1.209` at tet 1324
   (gated A Gate C re-confirmed at tet 1078). Geometry-inherent (cavity
   wall must stretch 2.2× somewhere at 8 mm inset on the iter-1 sock
   scan); orthogonal to contact-model regularity. **Candidate C does
   NOT address this class** — candidate B (material-validity
   safe-step) is the orthogonal arc.

---

## 2. Mechanism — smoothed penalty barrier

### 2.1 Where the discontinuity actually lives — Hessian, not gradient

`PenaltyRigidContact`'s hard penalty (`sim/L0/soft/src/contact/penalty.rs`):

| Formula | `sd < d̂` (active) | `sd = d̂` (boundary) | `sd > d̂` (inactive) |
|---|---|---|---|
| Energy `E` | `0.5 · κ · (d̂-sd)²` | `0` | `0` |
| Gradient `∇E` (per-vertex contribution) | `-κ · (d̂-sd) · n` | `0` | `0` |
| Hessian `H` (per-vertex 3×3 block) | `κ · n ⊗ n` | `0` (penalty.rs:326 gate `d >= self.d_hat` returns `ContactHessian::default()` at sd = d̂ exactly) | `0` |

Energy is **C⁰** at `sd = d̂` (value matches: 0 = 0). Gradient is
**C⁰** at `sd = d̂` (limit from the left `-κ·0·n = 0` matches the
gate-returned zero). Hessian is **discontinuous** at `sd = d̂` (limit
from the left is `κ · n ⊗ n`; gate at `sd = d̂` returns 0 — finite
jump).

Consequence for Newton: the assembled tangent
`A(x) = M/dt² + K(x) + H_contact(x)` is piecewise-continuous in `x`,
with jumps when any pair flips active. Newton's quadratic convergence
estimate `‖x_{k+1} - x*‖ ≤ C · ‖x_k - x*‖²` requires `A(x)` continuous
near `x*`; with `A` discontinuous, the contraction degrades to linear
(or worse, oscillatory). The bookmark §2.1 Gate B trace's `r_norm`
asymptote at 1.78 — decreasing by `O(1e-5)` per iter at iter 58+, then
Armijo stalling at iter 61 when the per-iter decrease falls below the
Armijo `c1 = 1e-4` tolerance — is the linear-rate-asymptotic-to-floor
signature.

Note: the bookmark §2.2 7-row table describes an "iter 1→2 r_norm
JUMP UP (10.94 → 56.65)" as the active-set-flip signature. That row
is by-geometry-analogy with F3.4 Gate B; gated A's iter-1 r_norm is
unobservable from stderr (Llt-PD iters don't log). The bookmark §2.2
caveat is acknowledged; the diagnostic conclusion (class-2 active-set
chattering) stands on the iter-57-onwards linear-floor signature
regardless of whether iter 1→2 was a single discontinuous jump or a
fast-but-continuous fall through a transient.

### 2.2 Smoothing prescription — taper over `[d̂, d̂+ε]`

The bookmark §3 proposed a multiplicative `(d̂-sd)² · smooth_step(d̂-sd, ε)`
where `smooth_step` is a cubic/sigmoid over `[-ε, +ε]`. This spec keeps
the multiplicative shape but tightens the prescription:

**Smoothed energy** (one parameter `ε > 0`):

```
E_smooth(sd, ε) = 
    0.5 · κ · (d̂ - sd)² · ramp(sd, ε)

    where ramp(sd, ε) =
        1                              for sd ≤ d̂              (full penalty)
        smooth_polynomial(sd, d̂, ε)    for d̂ < sd < d̂ + ε     (transition band)
        0                              for sd ≥ d̂ + ε          (inactive)
```

The transition lives in `[d̂, d̂+ε]` rather than `[d̂-ε, d̂+ε]` (the
bookmark's symmetric proposal) for **three reasons**:

1. **Penetration semantics are unchanged**: for `sd < d̂` the smoothed
   energy is bit-identical to the hard penalty. The active band's
   physical interpretation (penalty against penetration into `sd < d̂`)
   carries through. The smoothing is one-sided — it only changes
   inactive-side behavior near the boundary.

2. **Bit-equality at penetrated state**: deep contact (`sd ≪ d̂` —
   most of the per-iter active set in the chattering regime) is
   bit-equal to today's hard penalty. The smoothing only changes
   behavior in the boundary band, where chattering happens.

3. **Conservative tradeoff**: the bookmark's symmetric variant
   (`sd ∈ [d̂-ε, d̂+ε]` smoothing) softens the active-side too, which
   reduces the effective contact stiffness at near-boundary pairs.
   That's an extra knob the spec doesn't need — one-sided smoothing
   isolates the chattering fix from the contact-stiffness regime.

**Polynomial choice** (`smooth_polynomial`): the spec mandates **at
minimum C¹** at both endpoints (`sd = d̂` and `sd = d̂+ε`) so the
gradient is continuous across pair flips, and **recommends C² as the
empirical default** so the Hessian is also continuous (the actual
chattering fix).

Three candidate polynomials, ordered by smoothness:

| Polynomial | Smoothness | LOC per evaluation | Notes |
|---|---|---|---|
| **Cubic Hermite** | C¹ at endpoints, C⁰ second derivative | ~5 lines | Gradient continuous; Hessian discontinuous at endpoints. **May NOT clear chattering** — endpoint Hessian jumps remain. Cheapest to implement; useful as a falsification gate. |
| **Quintic Hermite** | C² at endpoints | ~8 lines | Gradient + Hessian continuous. **Expected to clear chattering**. Recommended default. |
| **Heptic / higher** | C³ at endpoints | ~12 lines | Third derivative continuous; Newton sees C² assembled tangent. YAGNI unless quintic also chatters. |

**Implementation guidance** (binding on C.1, non-binding on this
spec's correctness):
- Start with **quintic Hermite** (the expected default per the
  C²-required-for-Hessian-continuity argument).
- If quintic clears the falsifier matrix → ship.
- If quintic chatters (unlikely but possible — the smoothing is
  applied per-pair and the assembled tangent's continuity also
  depends on how pairs compose; see §7 open questions), escalate to
  heptic OR escalate to candidate D (mesh refinement / sub-step time
  integration).
- DO NOT start with cubic — it leaves the chattering pathology at
  the smoothing endpoints. Cubic is the **falsification gate**, not
  the production polynomial.

Concretely, **quintic Hermite over `[d̂, d̂+ε]` with boundary
conditions `(1, 0, 0)` at `sd = d̂` and `(0, 0, 0)` at `sd = d̂+ε`**
(value, first derivative, second derivative). The standard
Hermite-quintic basis gives an explicit closed form; the C.1
implementer derives the coefficients.

### 2.3 Active-pair gate extension

The current gate in `active_pairs` (penalty.rs:367) + `per_pair_readout`
(penalty.rs:232) is `sd < d̂`. With smoothing, pairs in
`sd ∈ [d̂, d̂+ε)` also contribute (their `ramp(sd, ε) > 0`). So the
gate extends to `sd < d̂ + ε`.

At `ε = 0`, the gate is `sd < d̂` bit-equally — the bit-equal-when-
dormant contract holds. The `interior_cutoff` filter (penalty.rs:108)
composes orthogonally — its `sd < -c` exclusion fires before the
upper-band check, independent of `ε`.

**Performance consideration**: extending the gate widens the active
band, which increases the per-iter pair count by approximately
`|grad(SDF)| · ε · surface_area / cell_volume` — a small constant
factor for typical `ε ∈ [0.1, 2.0] · d̂`. Not a hot-path concern at
the cf-device-design scene scale (~50k tets, ~10k surface vertices)
but worth flagging for sim-soft consumers with finer meshes.

### 2.4 Per-pair vs global ε surface

**Decision: global `ε` per `PenaltyRigidContact` instance.** All
primitives the instance carries share the same `ε`.

**Rationale**:
- The cf-device-design insertion sim (the only Fork-B consumer) has
  a single primitive (the intruder `Solid::from_sdf(transformed)`)
  per contact instance. Per-primitive `ε` is YAGNI.
- Per-pair `ε` (different `ε` per `(vertex, primitive)` pair) would
  require dynamic per-vertex state and break the
  `with_params_and_smoothing(...)` constructor's by-value semantics.
  YAGNI³.
- A future Fork-A consumer that wants per-primitive `ε` can add a
  `with_per_primitive_smoothing(...)` constructor variant when the
  need surfaces. This spec does NOT pre-allocate that API surface.
  YAGNI per the F3 spec §2.4 precedent (gated A removed Eager LM
  rather than gating it behind a config enum; same posture).

The `interior_cutoff: Option<f64>` field precedent (penalty.rs:108)
shows the established pattern: optional per-instance scalar with
`None` (or `0.0`) meaning "off". Candidate C adds a sibling field
`smoothing_eps_m: f64` with `0.0` meaning hard penalty.

### 2.5 API surface — extend `PenaltyRigidContact`, no new type

**Decision: extend `PenaltyRigidContact` with a `smoothing_eps_m: f64`
field + a new constructor `with_params_and_smoothing(...)`. No
`SmoothedPenaltyRigidContact` sibling type.**

**Rationale**:
- A sibling type duplicates the `ContactModel` + `ActivePairsFor`
  impls (~100 LOC of mechanical duplication). Composition via wrapper
  is awkward because `PenaltyRigidContact` is `!Clone`.
- Extending the existing type with a field defaulting to `0.0`
  preserves the bit-equal-when-dormant contract by construction:
  every existing constructor (`new`, `with_params`,
  `with_params_and_interior_cutoff`) initializes `smoothing_eps_m = 0.0`
  and the existing fixture tests pass bit-equal.
- The new constructor `with_params_and_smoothing(primitives, kappa,
  d_hat, smoothing_eps_m)` joins the existing builder family. A
  fourth constructor `with_params_and_smoothing_and_interior_cutoff(...)`
  may be needed if cf-device-design's sliding ramp wants both
  smoothing + interior cutoff (likely — `intruder_contact_sliding_at`
  currently uses the cutoff-variant constructor). C.1 picks; this
  spec sketches both options and the implementer chooses.

**Proposed field layout** (extends `PenaltyRigidContact` at
penalty.rs:74-109):

```rust
pub struct PenaltyRigidContact {
    primitives: Vec<Box<dyn Sdf>>,
    kappa: f64,
    d_hat: f64,
    interior_cutoff: Option<f64>,
    /// One-sided smoothing window (m) above `d_hat`. When `> 0`, pairs
    /// with `sd ∈ (d_hat, d_hat + smoothing_eps_m)` contribute a
    /// polynomial-tapered penalty that smoothly reaches 0 at
    /// `sd = d_hat + smoothing_eps_m`. Addresses class-2 active-set
    /// chattering in `H_contact(x)` (see
    /// `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md`).
    ///
    /// `0.0` (default) — hard penalty, bit-equal to pre-candidate-C
    /// behavior. Existing constructors (`new`, `with_params`,
    /// `with_params_and_interior_cutoff`) initialize this to `0.0`;
    /// the `with_params_and_smoothing(...)` constructor opts in.
    smoothing_eps_m: f64,
}
```

**MAINTENANCE NOTE for the C.1 implementer**: the
`smoothing_eps_m > 0` branch is the new code path. Five sites in
penalty.rs gate on `sd < self.d_hat` (or `d >= self.d_hat`) and
must be touched in lockstep:
1. `active_pairs` (penalty.rs:367) — gate `sd < self.d_hat`.
2. `per_pair_readout` (penalty.rs:232) — gate `sd < self.d_hat`.
3. `energy` (penalty.rs:291) — gate `d >= self.d_hat`.
4. `gradient` (penalty.rs:307) — gate `d >= self.d_hat`.
5. `hessian` (penalty.rs:326) — gate `d >= self.d_hat`.

All five must agree: when `ε = 0` the gate is `sd < d̂` / `d >= d̂`
bit-equally; when `ε > 0` the gate extends to `sd < d̂ + ε` /
`d >= d̂ + ε` AND the formula branches on whether `sd ≤ d̂` (full
penalty) or `d̂ < sd < d̂ + ε` (polynomial taper). **A mismatched
gate between any two sites would break the
`per_pair_readout`-mirrors-`active_pairs` contract** (penalty.rs:198
docstring pins this). The implementer SHOULD factor the gate +
formula into a single private method on `PenaltyRigidContact` (e.g.,
`fn pair_contribution(&self, sd: f64, n: Vec3) -> Option<PairContribution>`)
shared by all 5 sites. The factoring is non-load-bearing (today's
code inlines the gate at each site) but the spec recommends it to
prevent future mirror-on-change drift.

### 2.6 Where the code change lands

**`sim/L0/soft/src/contact/penalty.rs`** — THE landing site for C.1.
The five gates (§2.5) + the new constructor + the new `smoothing_eps_m`
field + the polynomial taper helper land here. ~150 LOC additions, no
deletions (additive change preserving the existing surface).

**`tools/cf-device-design/src/insertion_sim.rs`** — the Fork-B
opt-in lands here for C.2:
- `INSERTION_CONTACT_SMOOTHING_EPS_M` const alongside
  `INSERTION_CONTACT_KAPPA` (line 929) + `INSERTION_CONTACT_DHAT`
  (line 934).
- `intruder_contact_at` (line 943) routes through the smoothing
  variant constructor.
- `intruder_contact_sliding_at` (line 2289) routes through the
  smoothing-AND-cutoff variant constructor (if added).
- ~5-15 LOC depending on whether the constructor combinatorics
  collapse cleanly.

**No changes** to `sim/L0/soft/src/contact/mod.rs` — the `ContactModel`
+ `ActivePairsFor` traits are unchanged. The smoothed behavior lives
entirely inside `PenaltyRigidContact`'s implementation of those
traits.

**No changes** to `sim/L0/soft/src/solver/backward_euler.rs` — the
solver consumes `active_pairs` + `gradient` + `hessian` via the trait
surface; the smoothed contact just changes what those methods return
at near-boundary pairs.

### 2.7 No new `ContactModel` trait methods or defaults

The `ContactModel` trait (`mod.rs:130-143`) has **four REQUIRED
methods**: `energy`, `gradient`, `hessian`, `ccd_toi`. Candidate C
adds **no new required methods** and **no new defaults** to the trait.
The smoothing is a per-impl concern; other impls (`NullContact`,
future IPC) opt out by not exposing `with_params_and_smoothing(...)`
constructors.

The `ActivePairsFor<M>` trait (`mod.rs:154-158`) has one required
method (`active_pairs`). Unchanged.

**Rationale** (F3 §F3.1 + F3 §2.5 carry-over): "describe the failure
surface, not the rescue mechanism" — the trait surface describes
WHAT contact models do; the chattering rescue is a per-impl HOW.

---

## 3. Backward-compat contract (bit-equal-when-dormant — F3 §F3.1 carry-over)

**The contract**: every existing sim-soft test must pass *bit-equal*
under candidate C's new code path with `ε = 0`. Concretely:

- All existing constructors (`new`, `with_params`,
  `with_params_and_interior_cutoff`) initialize `smoothing_eps_m = 0.0`.
- At `smoothing_eps_m = 0.0`, the §2.5 five gates evaluate to
  identical branches as today (`sd < d̂` bit-equal; formulas inside
  the active branch unchanged).
- The polynomial taper helper is only invoked on the `sd > d̂` side,
  which today returns 0 in all 5 sites — the new code path is
  unreachable at `ε = 0`.

**The hardest regression evidence**:
- `sim/L0/soft/tests/contact_unit.rs:148, 296, 392` — exact-numeric
  hand-checked penalty value assertions.
- `sim/L0/soft/tests/hertz_sphere_plane.rs:515` — analytic Hertz
  benchmark with tight numeric tolerance.
- `sim/L0/soft/tests/penalty_compressive_block.rs:313` — convergence
  trajectory pinned across the fixture geometry.
- `sim/L0/soft/tests/penalty_interior_cutoff.rs:41` — interior-cutoff
  variant's behavior (composes orthogonally with smoothing).
- `examples/sim-soft/compressive-block/src/main.rs:734` — example's
  visual output bit-stability.

**The bit-equal verification gate**: `cargo test -p sim-soft --release`
green with **no test modifications**. If any of the regression-net
tests break, the C.1 implementation has a `ε = 0` bug — fix before
proceeding. Per [[feedback-cf-cast-tests-use-release]] always use
`--release` for sim-soft test runs.

**Hot-path performance**: at `ε = 0`, the polynomial-taper branch is
unreachable. The 5-gate checks add a single `f64` comparison per
gate-evaluation site — measurably zero. At `ε > 0`, the active band
widens (more pairs per iter) and the taper polynomial fires per
near-boundary pair (~10 flops per pair). Both are below the
mass-matrix factor cost dominance. No micro-bench required for C.1.

---

## 4. Fork-A vs Fork-B (carry-over from F3 §4)

- **Fork B (cf-device-design)**: opts in via
  `INSERTION_CONTACT_SMOOTHING_EPS_M` const + the
  `with_params_and_smoothing(...)` constructor at
  `intruder_contact_at` (line 943) + `intruder_contact_sliding_at`
  (line 2289). Per the F3 §4 Fork-B carry-over pattern, the surface
  plumbing (`try_replay_step` + `solver_failure_message` +
  `catch_unwind` belt-and-suspenders + gated-A `lm_regularization =
  Some(LmConfig::fork_b())`) is unchanged — gated A and smoothed
  contact compose orthogonally (gated A handles class-1 indefinite
  tangent, smoothed contact handles class-2 chattering).

- **Fork A (production sim-soft)**: NOT in this spec.
  `smoothing_eps_m` stays `0.0` for all sim-soft consumers other
  than cf-device-design. If a future Fork-A consumer needs smoothing
  (e.g., the row 21/22 silicone-sleeve insertion sim — see
  `project_sim_soft_row_22_patterns.md` referenced from
  penalty.rs:264 — hits the same chattering pathology), they opt in
  at their consumer's empirical surface; YAGNI on the abstraction
  today.

---

## 5. Falsifier matrix (when C doesn't work, what we learn)

All comparisons against the **gated-A baseline**:
- Cavity = 3 mm: 16/16 converged, ZERO LM seedings (the §F3 recon A
  outcome-B Gate A result).
- Cavity = 5 mm: 0/16, Armijo stall iter 61 at `r_norm = 1.784`
  (gated-A Gate B result; the structural floor candidate C must
  cross).
- Cavity = 7 mm: untested under gated A (the UI cap was lowered to
  4 mm post-gated-A). Candidate C raises the UI cap empirically per
  C.3.

Four outcomes ranked by what they tell us about the failure-class
mental model:

| Outcome | Cavity 3 mm | Cavity 5 mm | Cavity 7 mm | What it means | Next action |
|---|---|---|---|---|---|
| **A. SHIP-IT** | 16/16 converged (baseline preserved) | 16/16 converged at some empirical `ε ∈ [0.1, 2.0] · d̂` | 16/16 converged | Class-2 chattering was the binding pathology at cavity 5-7 mm; smoothed contact resolves it. | Ship; raise UI cap to 7 mm (or 8 mm if Yeoh validity passes at the chosen `ε`); update F3 falsification bookmark with "RESOLVED full by candidate C". |
| **B. PARTIAL SHIP-IT** | 16/16 converged | 16/16 converged | Stalls at `r_norm` floor < 1.78 (any improvement vs pre-F3) | Class 2 was the 5 mm pathology but a separate class binds at 7 mm (likely Yeoh validity — class 3 starts before cavity = 8 mm at some pose). | Ship; cap UI at 5 mm (or 6 mm if 7 mm stall is mild — implementer judgment); bookmark + recon for candidate B (material-validity safe-step). |
| **C. REGRESSION** | < 16/16 converged | (any) | (any) | Bit-equal-when-dormant contract was VIOLATED — implementation bug in §2.5's 5-site gate lockstep or §2.2's polynomial taper. | Revert C.1 + C.2; debug the contract violation (probably a gate that fires at `sd < d̂ + ε` even when `ε = 0`); re-implement. |
| **D. WORSE-THAN-GATED-A** | 16/16 converged | `r_norm` floor > 1.78 OR < 16/16 converged at empirically all `ε > 0` tested | (any) | The smoothing prescription is structurally wrong for class-2 — the polynomial taper may be widening the active band into pairs whose normals are ill-conditioned (the Hessian's eigenstructure worsens). Cubic-vs-quintic-vs-heptic escalation may help, or the chattering's actual mechanism is at a different geometric scale than the smoothing window. | Revert C.1 + C.2; bookmark D's empirical data (per-iter active-set size + `r_norm` trajectory under each `ε`); recon for candidate D (mesh refinement / sub-step time integration). |

**Outcome A vs B**: the binary question C empirically tests at cavity
= 7 mm. C's mechanism is sound iff smoothed contact clears chattering
at the deeper-than-validated cavity = 5-7 mm range. Cavity = 5 mm
alone is the necessary-condition gate (a candidate that fails 5 mm
fails C entirely); cavity = 7 mm is the sufficiency check.

**Outcome D**: the pessimistic case. If escalating cubic → quintic
→ heptic still chatters, the active-set discontinuity may not be the
true binding pathology and the bookmark's class-2 diagnosis was
wrong. Plausible but the gated-A Gate B data strongly supports the
chattering diagnosis (the `r_norm` linear floor signature at iter
58+ is textbook class-2).

**Recommendation**: outcome A is the realistic prior; outcome B is
plausible if 7 mm shows a separate-class binding pathology at the
chosen `ε` (untested under gated A — cavity = 7 mm sits between the
gated-A-validated 5 mm chattering regime and the gated-A-Gate-C
8 mm Yeoh-validity regime; whether 7 mm hits chattering, validity,
or neither is the unknown C.3's user-driven gate resolves); outcome
C is an implementation-bug check; outcome D is the surprise case
that bookmarks + escalates to candidate D.

---

## 6. Sub-leaf ladder

Sized per [[feedback-bookmark-when-surface-levers-exhaust]]
three-session pattern. C.0 this session (spec only); C.1 + C.2 + C.3
+ C.4 in a separate session (or sessions if the calibration sweep
fans out).

### C.0 — recon spec — THIS SESSION's first + only artifact

- This doc.
- No code changes.
- Cold-read pass-1 + pass-2 + polish commit (per
  [[feedback-cold-read-review-post-ship]] +
  [[feedback-cold-read-two-passes-for-non-trivial-diffs]] — spec
  exceeds 800 lines, well past the 300-LOC pass-2 trigger; the
  parallel-surfaces hazard at §2.5's 5-site enumeration makes
  pass-2 non-optional).
- Memory updates ([[project-cavity-5mm-chattering-bookmark]] gets
  "C.0 spec SHIPPED, next: C.1 sim-soft primitive"; MEMORY.md
  "Resume here" updated with new commit chain).

### C.1 — sim-soft `PenaltyRigidContact` smoothing extension — separate session (medium)

- Add `smoothing_eps_m: f64` field + new constructor(s) per §2.5
  (the §2.5 4th constructor combinatorics is a judgment call;
  implement only the variants cf-device-design needs — likely 1 or 2
  new constructors).
- Refactor the §2.5 5-gate sites to share a single private gate +
  formula helper (recommended per §2.5 mirror-on-change prevention).
- Implement the polynomial taper helper (per §2.2 — start with
  quintic Hermite over `[d̂, d̂+ε]` with `(1,0,0) → (0,0,0)` boundary
  conditions; derive closed-form coefficients).
- Unit tests (~10 new test cases):
  - **Bit-equal at `ε = 0`**: every existing penalty test still
    passes (the §3 contract verification gate).
  - **Continuity at `sd = d̂`**: assert `lim sd → d̂⁻` value /
    gradient / Hessian match the smoothed formula's evaluation at
    `sd = d̂` (the active-band-boundary).
  - **Continuity at `sd = d̂ + ε`**: assert value / gradient /
    (for quintic) Hessian go to 0 smoothly at the smoothing-window
    boundary.
  - **Monotonicity in the transition band**: value strictly
    decreasing in `sd ∈ (d̂, d̂+ε)`.
  - **Gradient continuity across pair flip in a Newton step**:
    fixture where a single pair crosses `sd = d̂` during a synthetic
    Newton-step + line-search; assert the residual's directional
    derivative matches the assembled gradient.
- Acceptance: `cargo test -p sim-soft --release` green; all existing
  fixtures bit-equal; new tests green.
- Sizing: ~150 LOC penalty.rs additions + ~150 LOC tests = ~300 LOC.
  Per [[feedback-implement-measure-revert-pattern]], if the
  implementation grows past ~400 LOC (e.g., the polynomial gets
  complicated, the constructor combinatorics fan out), STOP + bookmark
  per [[feedback-bookmark-when-surface-levers-exhaust]] — escalate to
  a separate implementation session.

### C.2 — cf-device-design opt-in + empirical ε calibration sweep — separate session (medium)

- Add `INSERTION_CONTACT_SMOOTHING_EPS_M` const alongside KAPPA +
  DHAT (insertion_sim.rs:929-934).
- Route `intruder_contact_at` (line 943) + `intruder_contact_sliding_at`
  (line 2289) through the new smoothing-variant constructor(s).
- Update the inline `INSERTION_CONTACT_KAPPA` docstring to point at
  this spec (the `1e3` κ choice rationale is unchanged; smoothing
  composes orthogonally with κ).
- **User-driven visual gate ε-calibration sweep** at cavity = 5 mm,
  layers 10+3 mm (the apples-to-apples gated-A Gate B regime):
  `ε ∈ {0.1, 0.25, 0.5, 1.0, 2.0} mm` (5 values × 16 ramp steps =
  80 sliding-ramp solves). Find the smallest `ε` that converges
  16/16. The chosen `ε` becomes
  `INSERTION_CONTACT_SMOOTHING_EPS_M` default. Output: chosen `ε`
  hand-off into C.3 + per-`ε` convergence row count documented in
  C.4's bookkeeping.
- Pinning the chosen `ε`: a sentinel test in cf-device-design that
  asserts the const value + carries a comment with the empirical
  evidence (the per-`ε` convergence row count).
- Acceptance: `cargo test -p cf-device-design --release` green;
  clippy clean; user-driven visual gate at cavity = 3 mm (sanity
  check baseline preservation) + cavity = 5 mm at the chosen `ε`
  (the candidate-validation gate at the gated-A-validated regime).
- Sizing: ~20 LOC code + ~5 LOC sentinel test + 80-iter user-driven
  calibration sweep (substantial user time, ~30-60 min of UI
  scrubbing).

### C.3 — UI cap restoration (outcome-dependent) — same session as C.2

The cap-restoration is gated by **per-cavity user-driven visual
gates** at the chosen `ε` from C.2. C.2 only validated 3 + 5 mm; C.3
extends to 6, 7, (optionally 8) mm to validate the §5 falsifier-
matrix outcome columns:

- **Probe gate 1**: cavity = 6 mm at chosen `ε`. Outcome A's
  intermediate confirmation; if 6 mm chatters, fall back to outcome
  B's 5 mm cap.
- **Probe gate 2**: cavity = 7 mm at chosen `ε`. Outcome A's
  validation per §5 7-mm column.
- **Probe gate 3 (optional)**: cavity = 8 mm at chosen `ε`. If Yeoh
  validity passes (no `max_stretch_deviation > 1.0` panic), outcome A
  raises the cap to 8 mm instead of 7 mm.

Per §5 falsifier matrix outcomes:
- **Outcome A** (3+5+6+7 mm all 16/16): raise UI cap from 4 mm →
  7 mm (or 8 mm if probe gate 3 passes). Update the 3 parallel
  surfaces per the gated-A precedent (main.rs:227-277
  `inset_slider_range_m` docstring + main.rs:2103 egui label
  MAINTENANCE NOTE + main.rs:2997 sentinel test). Mirror-on-change
  MAINTENANCE NOTES across all 3 surfaces (the
  [[feedback-cold-read-two-passes-for-non-trivial-diffs]] precedent
  banked at gated-A's a1c26105 polish).
- **Outcome B** (3+5 mm both 16/16 but 6 or 7 mm stalls): raise UI
  cap from 4 mm → 5 mm or 6 mm (implementer judgment based on the
  probe-gate-1 result). Same 3-surface MAINTENANCE NOTE pattern.
- **Outcomes C / D**: NO UI cap change. Per §5, revert C.1 + C.2.

### C.4 — Bookkeeping update — same session as C.3

- Append §9 "RESOLVED full by candidate C" footer to
  `docs/F3_FALSIFICATION_BOOKMARK.md` (or "RESOLVED partial" for
  outcome B; "FALSIFIED" for outcomes C/D and trigger candidate D
  recon).
- Append RESOLUTION footer to
  `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` pointing at C's outcome.
- Update [[project-cavity-5mm-chattering-bookmark]] memory with
  outcome status + final UI cap.
- Update [[project-f3-falsification-bookmark]] memory with
  RESOLVED-full status (if outcome A) or candidate-D pointer (if
  outcome D).
- Update MEMORY.md "Resume here" section.

---

## 7. What this spec does NOT specify (and what's an open question)

- **Polynomial coefficient values** — C.1 derives closed-form
  quintic-Hermite coefficients. Spec mandates the boundary
  conditions, not the algebra.
- **The exact `ε` value** — C.2 picks empirically. The spec's
  sweep range `[0.1, 2.0] · d̂` is the starting envelope; C.2 may
  extend if no value in the range converges.
- **Per-primitive `ε`** (§2.4). Global scalar is the surface choice.
- **`SmoothedPenaltyRigidContact` sibling type** (§2.5). Extends
  `PenaltyRigidContact` in place.
- **Symmetric smoothing band `[d̂-ε, d̂+ε]`** (§2.2). One-sided
  taper `[d̂, d̂+ε]` is the prescription; symmetric smoothing
  pollutes the active-side regime.
- **IPC barrier formulation** (`-(d-d̂)² · log(d/d̂)`). The IPC
  barrier is a different mechanism (divergent as `d → 0⁺`); the
  cf-device-design penalty regime has a finite max contact force,
  not a hard interpenetration prevention. IPC is a future-phase
  upgrade (BF-12, Phase H per `penalty.rs:1-13` module docs);
  candidate C is a class-2-fix targeted at the existing penalty
  surface.
- **Candidate B (material-validity safe-step)**. Orthogonal class-3
  arc. If outcome A or B ships with chosen `ε`, raising UI cap past
  7 mm needs B separately.
- **Candidate D (mesh refinement / sub-step time integration)**.
  Outcome-D escalation path. Higher cost; defer unless C falsifies
  the class-2 diagnosis.
- **Gated A's mechanism** (gated LM rescue at first-pass Armijo
  failure). Orthogonal to smoothed contact; KEPT as-is per
  [[project-f3-recon-a-gated-lm-shipped]]. Gated A handles class 1;
  smoothed contact handles class 2; the two compose.
- **Hot-path microbenchmarks for `ε > 0`** (§3). The penalty hot
  path is not the bottleneck in cf-device-design (the mass-matrix
  factor dominates); per the F3 §F3.1 precedent, perf measurement is
  not blocking for C.1.
- **Open question: does C¹ smoothing (cubic Hermite) suffice?** §2.2
  recommends quintic but doesn't pre-falsify cubic. The C.1
  implementer MAY run cubic first as a falsification gate — if cubic
  clears 5 mm, the chattering's actual binding regularity threshold
  was lower than C² and the spec's recommendation was conservative.
  Either result is informative; document the per-polynomial
  convergence data in C.4's bookkeeping.
- **Open question: do per-pair contributions compose additively
  C²?** §2.2's C² argument applies per-pair; the assembled
  `H_contact(x)` is a sum over active pairs. Sum of C² functions
  is C², so the assembled tangent inherits the per-pair regularity
  — **but only as long as the active-pair SET doesn't change
  discontinuously with `x`**. The §2.3 gate extension
  (`sd < d̂ + ε`) means a pair joins the active set when `sd` falls
  below `d̂ + ε` (where the polynomial taper is 0) and leaves when
  `sd` exceeds `d̂ + ε`. At the entry boundary the contribution is
  0 + 0 + 0, so the discontinuity in active-set membership is
  invisible at the function-value / gradient / Hessian level. This
  is the **structural argument** for why the smoothing actually works:
  it converts the active-set discontinuity into a continuous-by-
  composition mechanism. C.1 implementation MUST verify this
  composition argument with a fixture test (the §C.1 "gradient
  continuity across pair flip" test).
- **Open question: SDF normal `n = ∂d/∂p` continuity**. For
  mesh-derived SDFs (`mesh_sdf::SignedDistanceField`), `n` may have
  discontinuities at the closest-triangle-partition boundaries
  (independent of `sd = d̂` flips). Smoothing the gap function
  doesn't address this orthogonal source of chattering. If the
  empirical sweep finds no `ε` that converges 5 mm, the SDF-normal
  discontinuity may be the binding pathology — bookmark + escalate
  to candidate D (mesh refinement reduces normal-discontinuity
  density).

---

## 8. Anchors

**Predecessor commits** (on `sim-arc/sl-4-intruder-render`):
- `9a1433b8` — F3 recon A spec (A.0).
- `61b31f9b` — F3 recon A spec cold-read polish.
- `59997c41` — F3 recon A implementation (A.1+A.2+A.3).
- `c16fe375` — F3 recon A outcome-B follow-up (A.4: UI cap 8→4 mm +
  cavity-5mm bookmark + memory).
- `a1c26105` — F3 recon A outcome-B cold-read polish (5 findings).
- `edba9f48` — C.0 spec (initial draft).
- `f1dc082f` — C.0 spec cold-read polish (10 findings, pass-1 + pass-2).

**Predecessor docs**:
- `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` — the bookmark this acts on.
- `docs/F3_RECON_A_GATED_LM_SPEC.md` — sibling recon (structural
  template; bit-equal-when-dormant contract).
- `docs/F3_FALSIFICATION_BOOKMARK.md` — parent arc + three-class
  mental model.
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — original arc; §8 Run 2
  baseline 1.78 N is the empirical floor C must cross.
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — sibling falsification
  postmortem (F5 entry in §10 candidate scoring is what this spec
  builds on).

**Code sites — Section C.1 (sim-soft `PenaltyRigidContact` extension)**:
- `sim/L0/soft/src/contact/penalty.rs:74-109` — struct definition (add
  `smoothing_eps_m: f64` field).
- `sim/L0/soft/src/contact/penalty.rs:111-146` — `new` + `with_params`
  constructors (initialize `smoothing_eps_m = 0.0`).
- `sim/L0/soft/src/contact/penalty.rs:148-190` —
  `with_params_and_interior_cutoff` constructor (initialize
  `smoothing_eps_m = 0.0`).
- `sim/L0/soft/src/contact/penalty.rs:216-249` — `per_pair_readout`
  (gate site 2 of 5).
- `sim/L0/soft/src/contact/penalty.rs:283-340` — `ContactModel` impl
  (`energy` / `gradient` / `hessian` — gate sites 3-5).
- `sim/L0/soft/src/contact/penalty.rs:342-377` — `ActivePairsFor<M>`
  impl (`active_pairs` — gate site 1).
- NEW: `with_params_and_smoothing(primitives, kappa, d_hat,
  smoothing_eps_m)` constructor.
- NEW: `with_params_and_smoothing_and_interior_cutoff(primitives,
  kappa, d_hat, smoothing_eps_m, interior_cutoff)` constructor (if
  the sliding-ramp path needs both).
- NEW: private polynomial taper helper (e.g., `fn quintic_ramp(sd:
  f64, d_hat: f64, eps: f64) -> f64`).
- NEW: ~10 unit tests per the C.1 test list above.

**Code sites — Section C.1 (regression net — bit-equal verification)**:
- `sim/L0/soft/tests/contact_unit.rs:148, 296, 392` — exact-numeric
  penalty value assertions.
- `sim/L0/soft/tests/hertz_sphere_plane.rs:430-515` — analytic Hertz
  benchmark.
- `sim/L0/soft/tests/penalty_compressive_block.rs:235-313` —
  convergence-trajectory regression fixture.
- `sim/L0/soft/tests/penalty_interior_cutoff.rs:41-163` — interior
  cutoff variant (composes with smoothing).
- `sim/L0/soft/tests/contact_fd.rs:47` — finite-diff sanity of
  gradient.
- `sim/L0/soft/tests/contact_grad_hook.rs:232` — gradient-hook
  contract test.
- `sim/L0/soft/tests/contact_stability.rs:150` — stability fixture.
- `sim/L0/soft/tests/non_interpenetration.rs:132, 243` —
  non-interpenetration fixtures (the most penalty-sensitive of the
  set).
- `sim/L0/soft/tests/penalty_pair_readout.rs:68` —
  `per_pair_readout` mirrors `active_pairs` contract.

**Code sites — Section C.2 (cf-device-design opt-in)**:
- `tools/cf-device-design/src/insertion_sim.rs:921-934` —
  `INSERTION_CONTACT_KAPPA` + `INSERTION_CONTACT_DHAT` (the new const
  lands alongside).
- `tools/cf-device-design/src/insertion_sim.rs:943-956` —
  `intruder_contact_at` (route through smoothing-variant).
- `tools/cf-device-design/src/insertion_sim.rs:2289-2306` —
  `intruder_contact_sliding_at` (route through smoothing-AND-cutoff
  variant).

**Code sites — Section C.3 (UI cap restoration)**:
- `tools/cf-device-design/src/main.rs:227-277` —
  `CavityState::inset_slider_range_m` docstring + impl.
- `tools/cf-device-design/src/main.rs:2092-2113` — egui label
  + MAINTENANCE NOTE.
- `tools/cf-device-design/src/main.rs:2987-3016` — sentinel test +
  MAINTENANCE NOTE.

**Falsifier evidence — verbatim from gated-A Gate B stderr log**
(`/tmp/recon_a_gui.log` 2026-05-18 EVENING — quoted in
`docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` §2.1):

```
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 2 (free residual norm 5.665118902894688e1) (Llt non-PD pivot: NonPositivePivot { index: 13387 })
... [iters 3-56: progressive r_norm decrease ~38 → 1.78, LU+Armijo accepts every iter] ...
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 57 (free residual norm 1.7836752010550596e0) (Llt non-PD pivot: NonPositivePivot { index: 33374 })
sim-soft: LM seeded λ = 6.669388036959534e-3 at factor_and_solve_free at Newton iter 57 (free residual norm 1.7836752010550596e0)
sim-soft: LM converged in 1 retries to λ = 6.669388036959534e-3 at factor_and_solve_free at Newton iter 57
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 58 (free residual norm 1.7836728780581803e0)
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 60 (free residual norm 1.783667081522099e0)
sim-soft: faer LU fallback fired at factor_and_solve_free at Newton iter 61 (free residual norm 1.7836667564733832e0)
Error: sliding ramp failed at step 0 — no converged step. Armijo line-search stalled at Newton iter 61 (r_norm 1.784).
```

The iter-58→60→61 `r_norm` decrease of `O(1e-5)` per iter is the
linear-rate-asymptotic-to-floor signature class-2 chattering produces
(per §2.1 Hessian-discontinuity argument).

---

## 9. Implementation expectations + decision posture

- Per [[feedback-autonomous-architecture]] grant, drive C.1 + C.2 +
  C.3 + C.4 autonomously. Flag only big departures from this spec.
- Per [[feedback-implement-measure-revert-pattern]] the C.1
  implementation is a single session if it stays ≲ 400 LOC; the C.2
  calibration sweep is its own session (user-driven). Per
  [[feedback-bookmark-when-surface-levers-exhaust]] if C.1
  implementation reveals a surprise (e.g., the polynomial taper
  doesn't compose cleanly with the per-pair scatter, the
  constructor combinatorics blow up past 4 variants, or the
  bit-equal contract requires a subtle invariant the spec didn't
  anticipate), STOP and bookmark — three-session pattern continues
  inside the implementation phase.
- Per [[feedback-cold-read-review-post-ship]] cold-read pass after
  ship of each commit. Size-gate per
  [[feedback-cold-read-two-passes-for-non-trivial-diffs]]: pass-1
  sentence-level always; pass-2 structural if diff ≳ 300 LOC or has
  parallel surfaces. C.1's penalty.rs diff is exactly the §2.5
  5-parallel-surfaces pattern that triggered the F3.4 outcome-B
  polish's CR.P2.2 missing-maintenance-note finding — **pass-2 is
  not optional for C.1**.
- Per [[feedback-spec-falsified-revert-opt-in-keep-surface]] (banked
  from F3 falsification) — C's design preserves the bit-equal-
  when-dormant contract so a future falsification can do a 1-line
  revert (`INSERTION_CONTACT_SMOOTHING_EPS_M = 0.0`). The §5 outcome-
  C/D revert paths are the explicit revert surface.
- Per [[feedback-workaround-removal-verification]] (banked from F3.4
  hotfix) — C does NOT remove the F3.4 `catch_unwind`
  belt-and-suspenders in `run_sliding_insertion_ramp`. The Yeoh
  validity panic class (class 3) is still possible at outcome A's
  cavity = 7 mm if C lets Newton walk into a high-stretch region.
  `catch_unwind` stays.
- Per [[feedback-strip-the-knob-when-default-works]] — if C.2's
  calibration sweep finds a single `ε` value that converges
  3-7 mm + Yeoh-validity-passes, the `INSERTION_CONTACT_SMOOTHING_EPS_M`
  const stays a const (not a user-facing slider). DO NOT expose `ε`
  as a UI knob unless the empirical sweep finds a multi-modal
  outcome where different cavity insets prefer different `ε` (in
  which case bookmark for design-of-record discussion before adding
  the slider).
- Per [[feedback-cf-cast-tests-use-release]] always `--release` for
  cf-device-design / sim-soft test runs (debug-mode is 30× slower).
- Per [[feedback-long-running-commands-use-file-redirect]] if the
  C.2 calibration sweep's per-`ε` test runs cross 5 min, redirect
  stdout/stderr to a file + tail -f rather than relying on `tail
  -50` (which buffers until EOF).
- Per [[feedback-post-fmt-stage-all-changed]] if pre-commit fmt
  fails, stage ALL changed files (`cargo fmt -p sim-soft` may touch
  siblings of penalty.rs).
- Per [[feedback-plain-language-low-bandwidth]] keep C.1/C.2/C.3/C.4
  chat updates concise; math / jargon goes in commit messages, not
  user-facing chat.
