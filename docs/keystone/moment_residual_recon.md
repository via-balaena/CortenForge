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

## 3b. COM-frame-skew hypothesis — FALSIFIED (2026-06-15)

Tested via a `fresh_com` toggle on both spike instruments: compute the moment about the
FRESH COM `xipos(q_k)` (scratch forward) with fresh `jlin`, in BOTH the tape value and
the oracle. Result: **no improvement** — total_rel 9.5e-4 (stale) → 1.07e-3 (fresh),
per-step rel errors slightly UP across the board. So the moment's COM reference point
(stale vs fresh) is NOT the residual source. (Consistent with COM also being the eval
point an earlier round found load-bearing-when-left-alone.)

Remaining facts after this round: the residual is moment-specific, per-step, compounding,
and the per-step error pattern has a sharp ~7× drop between step 4 (3.9e-4) and step 5
(5.5e-5) and machine-exactness by step 8 — suggesting the error is injected at specific
steps (candidate: active-set transition / re-engagement events), not uniformly.

**Next decisive test (§4.1):** replace the analytic wrench-node Jacobian with a per-step
FINITE DIFFERENCE of the real wrench readout `w(x*, h, s)` in the tape. If the residual
VANISHES, the analytic `ContactWrenchTrajVjp` has a (sub-1e-5-gate) error in its moment
rows that compounds; if it PERSISTS, the wrench node is fine and the error is in the
soft-state propagation / carry interaction with the moment.

## 3c. Active-set + moment-magnitude correlation (2026-06-15) — sharpens the suspect

Per-step active count and moment magnitude over the n=10 rollout:

```
step 0: active=25  |τ|=8.08e1   ← violent initial make, moment 4 ORDERS larger than any other step
steps 1-3: active=0             (liftoff)
step 4: active=1   |τ|=3.1e-3   (brief re-touch)
steps 5-6: active=0
steps 7-9: active=5→7→9  |τ|=4.6e-2→4.7e-1→7.2e-1   (sustained re-engagement)
```

The residual is dominated by **step 0**, which has a moment ~4 orders larger than any
other step (the violent make: 25 active pairs, deep penetration). **Key gap found:** the
wrench-node gate `contact_wrench_node_matches_readout_fd` validates at the PRE-soft-solve
positions (`c.positions()`, at rest) — but the rollout uses the wrench node at the
POST-solve, deeply-penetrated config (large forces, large moment). The analytic moment
Jacobian `∂τ/∂x*` (esp. the via-`gᵢ` part = per-pair force-Jacobian × moment arm) is the
prime suspect for carrying an error there that the rest-config gate never sees. Decisive
test: FD-validate the wrench node at step 0's ACTUAL (post-solve) config.

## 3d. Wrench node is EXACT at the actual rollout config (2026-06-15) — eliminations

FD-validated the wrench node at the REST config (what the merged gate uses) AND the
POST-soft-solve / deeply-penetrated config (the ACTUAL step-0 rollout config), with TRUE
per-channel relative error (not the gate's scaled metric, which hides ∂w/∂h errors behind
a 1e5 scale):

```
                       ∂w/∂x* (τx, τy, fz)        ∂w/∂h (τx, τy, fz)
REST       (active 25): 5e-10, 1e-9, 1e-10         3e-11, 3e-11, 2e-11
POST-SOLVE (active 25): 2e-9,  2e-9, 3e-11         3e-11, 3e-11, 3e-11
```

Both `∂w/∂x*` and `∂w/∂h` are MACHINE-EXACT (incl. the moment rows) at the actual
deep-penetration config. So the wrench node is NOT the source.

**State of the diagnosis — the deep puzzle.** EVERY local factor is now machine-exact at
the actual rollout config: `∂w/∂x*` (§3d), `∂w/∂h` (§3d), the carry `J_state` (#315
analytic), `G_vel` (`rigid_xfrc_column`, validated), the soft solve (free-platen exact).
The free-platen (no-moment) composition is exact at every n. Yet the moment composition
drifts ~1e-3 at n=10. Locally-exact factors composing to a wrong gradient ⇒ a STRUCTURAL
subtlety that only the moment exercises.

**Two remaining specific suspects (NEXT):**
1. **`∂w/∂s` (the moment's `c(q)` state feedback via `jlin`)** — the only wrench channel
   not yet checked with TRUE relative error (the gate scales it by `sq=1e3`). Note the
   COM-frame test made `c`+`jlin` fresh with no gain, but didn't isolate `∂w/∂s`'s
   accuracy per se.
2. **The §8a position-row drop on the TORQUE columns of `G`.** `∂qpos'/∂w = 0` is
   FD-correct for the FORCE columns (validated n=2, free platen), but the free platen has
   no torque, so zeroing the TORQUE columns' position-row was never tested. Restoring the
   FULL term (all 6 cols, `G_POS_DT`) was WORSE — but the torque columns may need a
   treatment distinct from the force columns. Test: zero force-col position-rows (as now)
   but restore torque-col position-rows.

## 3. (superseded) Refined mechanism hypothesis

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
