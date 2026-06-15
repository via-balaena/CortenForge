# Off-COM moment's long-rollout gradient residual — recon + S0 localization

*Keystone deepening. Written 2026-06-15. The open problem bookmarked by the analytic
geometric-stiffness leaf (#315, `geometric_stiffness_recon.md` §6): the
articulated-with-moment coupled gradient is machine-exact through n≈6 but the residual
GROWS with n (~1e-3 at n=10, ~5e-3 at n=12). It is moment-specific (the free-platen
articulated path is machine-exact at every n), smooth (IPC ≡ penalty, not an active-set
kink), and resistant to the blunt fixes (fresh-FK re-forward, lag-attribution, the true
position-row term — all measured WORSE). This leaf localizes the residual and (if
tractable) removes it.*

## 1. S0 localization spike (THROWAWAY, measure-first) — DONE

Two throwaway instruments (on branch `feat/keystone-moment-long-rollout-residual`,
uncommitted):
- `spike_perstep_mu_gradient(n)` — the gradient tape with a SEPARATE μ leaf per step
  (μ-only, λ held); one backward returns `[∂tip_z_N/∂μ_k]`, the tape's per-step μ
  contribution.
- `spike_z_muonly_schedule(&[μ])` — the forward oracle with a per-step μ schedule
  (μ-only, λ held). Perturbing μ at ONE step gives the ground-truth per-step
  contribution by central FD.

**Result (hinge scene, n=10, μ-only):**

```
step |   tape_k    |    fd_k     |  rel
  0  | -3.578e-9   | -3.574e-9   | 1.13e-3   ← dominant contribution AND error
  1  | -1.869e-10  | -1.868e-10  | 6.2e-4
  2  | -2.725e-10  | -2.724e-10  | 5.7e-4
  3  | -2.897e-10  | -2.896e-10  | 4.9e-4
  4  | -2.387e-10  | -2.386e-10  | 3.9e-4
  5  | -1.126e-10  | -1.126e-10  | 5.5e-5
  6  | -7.21e-11   | -7.21e-11   | 5.1e-5
  7  | -3.83e-11   | -3.83e-11   | 4.6e-5
  8  | -2.07e-11   | -2.07e-11   | 1.6e-8   ← machine-exact
  9  |  0          |  0          |  0       ← lagged output: step 9 never reaches tip_z_N
SUM  | -4.810e-9   | -4.805e-9   | total_rel = 9.5e-4
```

## 2. The finding (overturns the bookmark's "late re-engagement" guess)

The residual is **dominated by the FIRST step** (the violent initial make): step 0
carries ~74% of the gradient and **~87% of the total error** (rel 1.13e-3). Per-step
relative error **decays monotonically** with step (1e-3 → 1.6e-8), with sharp drops
(k=4→5, k=7→8) that likely track liftoff/re-touch events in the active-set sequence.

**Interpretation.** The decomposition is by SOURCE step, but the error is injected
during PROPAGATION: a per-step contribution traverses all downstream steps, and step 0
traverses the most (9), step 8 the fewest (1) — so step 0 accumulates the most error.
Step 0 itself is fully FRESH and consistent (build forwards `self.data`, so its FK,
`c`, `jlin`, `G_vel` are all at q_0) yet its contribution carries the most error →
**the error is injected at the downstream STALE steps the contribution flows through**,
not at its source. It compounds multiplicatively → grows with n. The free-platen
(force-only) path injects ZERO such error, so it is specific to the MOMENT.

## 3. Refined mechanism hypothesis (to test, §4)

The leading candidate is a **one-step COM-reference frame skew specific to the moment**:

- The contact moment is computed about the **stale** COM `c = xipos(q_{k-1})`
  (`self.data` lags one step; both tape-forward and oracle do this, so forward values
  match — it is self-consistent).
- But the wrench RESPONSE `G_vel = Δt·M⁻¹·J(q_k)ᵀ` is at the **fresh** COM `xipos(q_k)`
  (the config the real `step` maps the wrench through; `fresh_xfrc_column`, the #313
  fix).
- So `τ_k` is defined about q_{k-1}'s COM but mapped through a Jacobian referenced at
  q_k's COM — a one-step COM-reference mismatch. The FORCE is COM-independent (no
  skew — why the force path is exact and the free platen is exact); the MOMENT is not.

This is consistent with everything: moment-specific, smooth, per-step, compounding, and
NOT fixed by the analytic geometric stiffness (which is about `∂(Jᵀw)/∂q`, a different
term). It also explains why the blunt fresh-FK (everything fresh) was WORSE — it changed
the plane height / soft-solve config too, not just the moment's COM reference.

**The targeted fix to test:** reference the moment about the FRESH COM (xipos(q_k)) in
BOTH the tape value and the oracle — a surgical forward change (only the moment's COM
reference point), NOT full fresh-FK. If the residual drops, confirmed.

## 4. Decomposition plan (NEXT)

1. **COM-frame test (primary).** Compute `τ` about the fresh COM in both `spike_*`
   forward and the tape; re-run the per-step localization. Does step-0's error drop?
2. **jlin eval-point.** `∂w/∂s` uses stale `jlin = ∂c/∂q` (correct for the stale-c
   forward). Confirm fresh `jlin` is WORSE (sanity: it must match the forward's stale c).
3. **Channel split.** If the COM-frame test is inconclusive, split the wrench node's
   `∂w/∂x*` (explicit-rᵢ vs via-gᵢ) / `∂w/∂h` / `∂w/∂s` contributions to the residual by
   selectively zeroing each in a matched tape+oracle.

Adequate for co-design in the meantime (DIRECTION + ~99.9% magnitude).
