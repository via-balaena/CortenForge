# sim-soft Real-Time Path — Phase-1 Measurement + Recon (Phase E predecessor)

**Status**: RECON 2026-08-10 (rev 2026-08-26), v2.20. Phase 1 (measure) COMPLETE — all four requested
measurements taken; §2 reports them. Phase 2 (this recon) proposes the MOR +
hyper-reduction path with a staged ladder whose first rung is a kill-or-confirm.
**No dependency was added.** Phase 1's instrumentation was temporary (implement →
measure → revert, per `feedback_implement_measure_revert_pattern`) and is reverted;
§9 lists exactly what was touched and how to reproduce. ⚠ **§2f (v2.1) is the one
exception and is deliberate**: its `SolverConfig::initial_guess` is production code
that SHIPPED, because unlike a timing probe the thing being measured is the feature.
Its harness is permanent too — `tests/predictor_spike.rs`, `#[ignore]`d — so that
result is re-runnable without re-applying anything. §2g (v2.2) is NOT a second
exception: it also wrote production code, measured it, and **reverted it**, which
is the pattern rather than a departure from it.

**MISSION.md**: AMENDED 2026-08-10 with the "On speed as a ceiling" clause — see §8b.
The ladder is authorised in principle; R3's three conditions are now mission-level.
⚠ That clause cites **this document** as the measured basis for its "throughput is a
ceiling" premise, so §2b is load-bearing outside this recon. **Its concurrency reading
was CORRECTED on 2026-08-22 — the ceiling is 38 concurrent full-order envs, not the
"~20" earlier revisions carried, and the reduced path raises it to ~395. Quote §2b, not
any figure taken from a revision before v2.0.** The timing figures still want an idle
box (risk 1).

**One-line verdict**: the goal is **not refuted**, but the measurements move where the
work is. The frame-budget gap is ~**13–47× in DOF** (≈ 1 500 free DOF fits a 60 Hz
converged-Newton frame; a high-quality environment wants 20k–70k), f32 is
**safe for the factor and solve and unsafe for the residual** — a distinction the
brief did not anticipate and which changes the wgpu plan — and **contact costs
0.02 % of a contact-active frame**, so contact is a *structural* problem for MOR, not
a compute one. Two cheap full-order levers surfaced that must be measured before any
reduced-order code is written. **A third, added 2026-08-23: a Newton PREDICTOR cuts
ITERATION COUNT — the factor no rung of the ladder touches — by `1.95×` on the
representative workload, roughly HALVING what R3 must deliver and bringing it from
clearly above R3's `10×` kill floor to bracketing it. ⚠ The load-carrying variant is
NOT shippable as a default: it is fatal on contact-plus-load. Every other ratio, and
the R3 derivation, live in §2f and are deliberately not restated here — three review
rounds each found a fresh metric-mixing error in this sentence. → §2f**

> ### ⚠ Carry these three caveats with every number in this document
>
> They are stated in full at §1a and §10, and repeated here because the verdict above
> is what gets quoted and these are what make it quotable-with-honesty.
>
> 1. **No timing here was taken on an idle box.** The quoted §2 sweep ran with ~1.7 of
>    12 cores held by background (a macOS storage scan and WindowServer) and repeats
>    within **under 3 %**, monotone in every column — trustworthy, but still an upper
>    bound. Earlier sweeps taken under heavier load produced non-monotonic numbers and
>    were **discarded, not quoted**. The two IPC-contact rows are the only figures not
>    re-measured after R0. → §1a
> 2. **"≈ 1 500 free DOF at 60 Hz" is an INTERPOLATION, not a measured point.** It is
>    derived from the 540 → 3 000 DOF pair at the measured `n^1.51` low-end exponent,
>    using the IPC fixture's 6-iteration count. **Quote the measured anchor instead
>    where one is needed: 540 free DOF at 12.3 Newton iterations runs at 0.47× of a
>    60 Hz frame** — a directly measured case that fits. → §10 risk 2
> 3. **Newton iteration count grows with refinement** — peak 21 → 65 across 567 →
>    36 663 free DOF at `dt = 1/60`, all converged. Frame cost is
>    `iterations × per-iteration cost`, so quoting per-iteration scaling alone
>    understates it. (An earlier draft recorded this as an unexplained *oracle
>    convergence failure*; that was a too-low iteration cap in the measuring harness,
>    diagnosed in §3d.) → §3b.4, §3d
>    ⚠ **Every iteration count in §2 is a `PreviousState`-start figure.** §2f cuts them
>    2.0–4.7× with a predictor, and shows §2a's `37.0 iters/step` to be a
>    transient-window mean (the 60-step mean is 18.9; the *peak* reproduces). Quote §2f
>    alongside any iteration count taken from §2.
>
> Two further limits that bite anyone reusing these numbers: the ECSW speedup in §4b
> rests on a **literature claim this recon did not measure** (R3's kill condition
> exists to hold it to account), and §5's hybrid-domain-decomposition arithmetic
> **closes on paper only** — nothing about the coupling condition or the moving contact
> patch has been designed or measured.

**Reading order**: §1 (what was measured, and the fixtures) → §2 (the four numbers)
→ §3 (what the numbers say about feasibility, including where they bite the plan) →
§4 (the MOR + hyper-reduction path) → §5 (contact — the hard part) → §6
(differentiability through the reduced model) → §7 (the staged ladder, R0..R6) → §8
(MISSION.md tension — a decision for the head engineer) → §9 (reproduction + what
was reverted) → §10 (open risks) → §11 (version history).

---

## 1. What was measured, and on what

### 1a. Hardware, and an honest contention statement

| | |
|---|---|
| Hardware | Apple M4 Pro — 12 CPU cores, 24 GB unified memory |
| Toolchain | rustc 1.96.0, `--release`, `sim-soft` with `faer` + `rayon` + `feral-metis` as shipped |
| Date | 2026-08-10 |

⚠ **This box was NOT idle and the timings are therefore NOT comparable to the
`uncontended` figures in `sim/L0/soft/Cargo.toml` or `bonded_layer_indentation.rs`.**
A background workload (another project's simulation, plus a macOS storage-management
scan) held 1–10 of the 12 cores for most of the session; polling for an idle window
over ~25 minutes never produced one. Two consequences, both handled rather than
hidden:

1. The first sweep produced **non-monotonic** numbers (the same 3 000-free-DOF case
   measured 4.9 ms and 28.0 ms per factorization in two runs; `block:20` measured a
   *higher* per-factorization cost than the 2.7×-larger `block:28`). Those runs are
   discarded, not quoted.
2. The quoted sweep (§2a) was re-designed for reproducibility: **one process per
   case** (the all-in-one-process run accumulated 1.4 GiB RSS and self-contaminated),
   a **deterministic workload** (`dt = 1e-3`, which pins every step to exactly 2
   Newton iterations, so each case does identical work), and **min-of-3 repeats**.
   That table is monotone in every column and is the one to trust. Absolute times are
   an **upper bound** — a quiet box would be faster, which is the conservative
   direction for a feasibility question.

Per the house rule: nothing here is labelled uncontended, because nothing here was.

### 1b. Why `bonded_layer_indentation` could not answer Q1, and what replaced it

The brief is right: `bonded_layer_indentation` sets `STATIC_DT = 1.0`, which collapses
the `M/Δt²` inertial term so each increment is a pure static root-find. It cannot
price a frame. Three **dynamic** fixtures were built for this recon (all in the
temporary harness of §9):

| fixture | geometry | why |
|---|---|---|
| **`cantilever`** | 20 × 2 × 2 cm beam, `x = 0` face pinned, released horizontal from rest under gravity. Neo-Hookean, μ = 3.5e5, λ = 1.4e6 (E ≈ 1 MPa, ν = 0.4), ρ = 1030. Tip droops 3.7–8.6 cm on a 20 cm span. | **The representative fixture.** Large-deflection, so Newton genuinely works: 6–37 iterations per step. |
| **`block_sag`** | 5 cm Neo-Hookean cube, bottom face pinned, released from rest under gravity. μ = 2e5, λ = 8e5 (matches `contact_drop_rest` / `hertz_sphere_plane`). | **The floor.** Stiff and small-strain: converges in 0.5–2 Newton iterations, so it prices the *linear-algebra* cost with the nonlinearity switched off. |
| **`dynamic_indentation`** | The `bonded_layer_indentation` geometry (bonded elastic layer, rigid sphere lowered through an **IPC log-barrier**) driven at `dt = 1/60` with velocity carried step to step instead of `STATIC_DT`. | **The contact fixture.** 71 displacement increments, contact engaged throughout. |

A note on the first fixture worth recording: an initial `block_sag`-only design was
**abandoned mid-measurement** because it converged in 2 Newton iterations — it was
measuring a nearly-linear problem and would have flattered the frame budget by an
order of magnitude. The `cantilever` was added specifically to make Newton work. Both
are reported, because the *spread between them* is itself a result (§3c).

---

## 2. The four numbers

### 2a. Q1 — the frame-budget gap

Deterministic workload (`dt = 1e-3`, exactly 2 Newton iterations per step), one
process per case, **min of 3 repeats**, 10 steps each. `asmF` / `asmK` / `numF` /
`tri` are **per-call** milliseconds for internal-force assembly, tangent assembly,
numeric Cholesky and triangular solve.

| case | free DOF | pattern nnz | nnz(L) | L (MiB) | symbolic (ms) | ms/step | asmF | asmK | numF | tri |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| cantilever 20×2 | 540 | 8 235 | 15 660 | 0.1 | 1.1 | 1.53 | 0.02 | 0.33 | 0.21 | 0.013 |
| cantilever 40×4 | 3 000 | 54 591 | 325 836 | 2.5 | 8.2 | 18.26 | 0.12 | 2.68 | 4.58 | 0.134 |
| cantilever 60×6 | 8 820 | 172 179 | 1 952 541 | 14.9 | 86.2 | 91.07 | 0.39 | 9.61 | 29.27 | 0.857 |
| cantilever 80×8 | 19 440 | 394 119 | 6 099 875 | 46.5 | 215.0 | 251.63 | 0.92 | 22.13 | 85.72 | 4.411 |
| cantilever 100×10 | 36 300 | 753 531 | 14 712 634 | 112.2 | 449.7 | 578.36 | 1.80 | 43.19 | 208.08 | 11.537 |
| block n=8 | 1 944 | 37 071 | 370 071 | 2.8 | 5.7 | 14.27 | 0.09 | 2.01 | 5.10 | 0.127 |
| block n=16 | 13 872 | 289 959 | 6 170 772 | 47.1 | 156.5 | 158.90 | 0.74 | 17.04 | 84.02 | 4.872 |
| block n=20 | 26 460 | 563 571 | 15 930 702 | 121.5 | 310.9 | 356.46 | 1.43 | 33.69 | 198.83 | 10.068 |
| block n=24 | 45 000 | 970 623 | 33 557 350 | 256.0 | 607.9 | 1 598.35 | 2.91 | 64.47 | 1 038.29 | 67.743 |
| block n=28 | 70 644 | 1 537 611 | 62 876 521 | 479.7 | 1 061.8 | 1 522.55 | 3.90 | 93.17 | 958.84 | 32.355 |

**Cost of one Newton iteration** (`asmK + numF + tri + 2·asmF`, the work a single
iteration actually does):

| free DOF | 540 | 3 000 | 8 820 | 19 440 | 36 300 |
|---|---:|---:|---:|---:|---:|
| ms / Newton iteration | **0.57** | **7.63** | **40.5** | **114.1** | **266.4** |

Scaling from 3 000 → 36 300 free DOF (12.1×) costs 34.9× — i.e. **≈ n^1.43**.

**Newton iterations actually needed** (measured on the same fixtures at `dt = 1/60`,
one step per frame):

| fixture | free DOF | iters/step mean | min | max |
|---|---:|---:|---:|---:|
| cantilever 20×2 | 540 | 12.3 | 6 | 21 |
| cantilever 40×4 | 3 000 | 24.2 | 6 | 37 |
| cantilever 60×6 | 8 820 | 30.2 | 10 | 41 |
| cantilever 80×8 | 19 440 | 37.0 | 11 | 56 |
| cantilever 100×10 | 36 300 | — | — | **hit the 60-iteration cap; solve aborted** |
| `dynamic_indentation` (IPC) | 5 202 | 6.0 | — | 7 |
| `dynamic_indentation` (IPC) | 18 750 | 6.5 | — | 8 |
| block_sag (all sizes) | 1 944 – 70 644 | 0.5 – 0.6 | 0 | 2 |

**The frame budget, stated plainly.** ⚠ The whole of this paragraph assumes Newton
starts from `x_prev`; **§2f halves the representative iteration count (6 → 3) with a
predictor**, which moves every DOF figure below in the favourable direction. Read the
two together. Take the IPC indentation's 6 Newton iterations
as the honest representative count (the cantilever's 12–37 is a beam collapsing under
self-weight; 6 is a normal contact-active soft-tissue step). At 60 Hz that allows
16.7 / 6 = **2.78 ms per Newton iteration**. Interpolating the measured `n^1.51` low-end scaling gives:

> **≈ 1 500 free DOF fits a converged-Newton 60 Hz frame.** The measured anchor is
> firmer than the interpolation: **540 free DOF at 12.3 Newton iterations runs at
> 0.47× of a 60 Hz frame** — a directly measured case that fits, with 2× headroom.

Directly measured points either side of the budget, at `dt = 1/60`, one step/frame:

| case | free DOF | ms/frame | × 16.7 ms | × 33 ms |
|---|---:|---:|---:|---:|
| cantilever 20×2 | 540 | 7.93 | **0.47× — FITS** | 0.24× |
| cantilever 40×4 | 3 000 | 202.6 | 12.1× | 6.1× |
| cantilever 60×6 | 8 820 | 1 278 | 76.5× | 38.7× |
| cantilever 80×8 | 19 440 | 4 452 | 267× | 135× |
| block n=28 | 70 644 | 578 | 34.6× | 17.5× |
| IPC indentation a/cell 1.5 | 5 202 | 486.9 | 29.2× | 14.8× |
| IPC indentation a/cell 2 | 18 750 | 771.0 | 46.2× | 23.4× |

⚠⚠ **The two IPC rows are pre-R0 TIMINGS and the `46.2×` derived from them is NO
LONGER USED (v2.4).** §2d finding 4 measures both fixtures on a second box and
finds `771.0 ms` irreconcilable: two of three fixtures put that box ~1.6× SLOWER
than this one, and IPC 18 750 alone puts it 2× faster. The `a/cell` labels here
also do not reproduce these DOF counts under the current `dims_for`, so the
fixture likely differed too. §2f now measures the gap post-R0 directly instead.

A "high-quality environment" in this codebase's own terms is the 20 k–70 k free-DOF
range (`Cargo.toml` cites 70 k free DOF; the conformed-disc and FSU meshes sit in
that band). **The gap is 13–47× in DOF.**

⚠ **CORRECTED 2026-08-22, and in time this document now quotes MEASURED points rather
than a derived exponent.** The sentence here read *"25–90× in DOF, which at `n^1.28` is
70–350× in time"*, and both halves were defective. **25–90×** is the pre-R0 pair
(20 k–70 k over the old 800-DOF reachable figure; v1.3 replaced 800 with 1 500 and
missed this line). **`n^1.28` appears nowhere else in this document and has no
producer** — the two exponents actually measured here are `n^1.51` (low-end) and
`n^1.43` (per-iteration), so the "70–350×" rested on a number with no source.

The table above already brackets the target band with direct measurements, and the
spread between them is the finding rather than noise:

| directly measured, in or near the 20 k–70 k band | Newton iters/step | × 16.7 ms |
|---|---:|---:|
| block n=28, 70 644 free DOF | 0.5–0.6 | **34.6×** |
| IPC indentation, 18 750 free DOF | 6.5 | ~~46.2×~~ → **53.2×** |
| cantilever 80×8, 19 440 free DOF | 37.0 | **267×** |

⚠ The IPC row is the one figure §2d finding 4 could not reconcile across boxes;
`53.2×` is its **measured post-R0** replacement (§2f), which is the number R3 is
now sized against. The other two rows are untouched and remain pre-R0/post-R0 as
originally noted.

⇒ **35–267× in time across the target band** — and the ordering **inverts**: the
19 440-DOF cantilever costs **7.7× more** than the 70 644-DOF block (4 452 vs 578 ms),
because it takes 37 Newton iterations against 0.5–0.6. Trajectory nonlinearity dominates
mesh size outright here — **§3b.3** states the same thing about iteration counts *at
comparable DOF*; across the band it is strong enough to reverse the ordering — and that
is precisely why a single DOF exponent was the wrong instrument. **Quote a row**, per
the measured-anchor rule in the header caveat.

**Substepping does not rescue it, and is measurably worse.** `cantilever 40×4` costs
470.7 ms as one `dt = 1/60` step (24.2 Newton iterations). The same frame as 16.7
substeps of `dt = 1e-3` costs 39.01 × 16.67 = **650 ms** (2 iterations each = 33.4
iterations total). Splitting the step buys better conditioning per iteration and pays
~1.4× more wall time for it.

### 2b. Q2 — memory per session

Two numbers, both in the §2a table. The Cholesky factor size is **exact**
(`SymbolicCholesky::len_val() × 8` bytes) and contention-immune:

| free DOF | 1 944 | 6 084 | 13 872 | 26 460 | 45 000 | 70 644 |
|---|---:|---:|---:|---:|---:|---:|
| nnz(L) | 370 k | 2.21 M | 6.17 M | 15.9 M | 33.6 M | **62.9 M** |
| L, f64 (MiB) | 2.8 | 16.8 | 47.1 | 121.5 | 256.0 | **479.7** |
| fill per row | 190 | 363 | 445 | 602 | 746 | **890** |
| process max RSS (MiB) | 22.6 | 69.7 | 135.7 | 299.9 | 564.7 | **919.1** |

- **Fill per row grows as ≈ n^0.35** — consistent with the nested-dissection figures
  already recorded in `ordering.rs` (334 → 458 → 648).
- At the top of the range the factor alone is **480 MiB** and a single session **peaks**
  at **~0.9 GiB**.
- **⚠⚠ CORRECTED 2026-08-22 — an earlier revision read "roughly 20 concurrent full-order
  sessions on a 24 GB box" off this row. That reading was wrong, and the error was in the
  model, not the measurement.** `process max RSS` is a **peak**, and at 70 644 free DOF
  **84 % of it is transient**: `OrderedLlt`'s value buffer is allocated inside
  `factor_free_tangent` and dropped when the step returns — `CpuNewtonSolver` retains no
  numeric factor, as its field list shows. Dividing 24 GB by a peak charges every session
  for memory that is released between steps.

  Re-measured on this table's own fixture and sizes (`uniform_block`, bottom face pinned),
  with an **exact** `getrusage` high-water mark rather than a sampled one:

  | | 70 644 free DOF |
  |---|---:|
  | held (survives the step) | **146 MiB** |
  | transient (factor + assembly) | **779 MiB** |
  | peak — reproduces this table's 919.1 | **926 MiB** (0.8 % apart) |

  The quantity that actually caps batching is the **marginal** cost of one more
  *concurrently stepping* env — the slope of exact peak against env count `K`:

  | free DOF | full-order | reduced (`r = 40`) |
  |---:|---:|---:|
  | 13 872 | 86 MiB/env → 283 envs | 17 MiB/env → 1 445 envs |
  | 26 460 | 175 MiB/env → 140 envs | 35 MiB/env → 698 envs |
  | **70 644** | **638 MiB/env → 38 envs** | **61 MiB/env → 395 envs** |

  So the full-order ceiling is **38, not ~20**, and the reduced path lifts it **10.4×** —
  a win available from **R1 as built, with no hyper-reduction**. It is not free: the
  reduced solve still assembles full-order triplets and forms `AΦ` every Newton iteration,
  which is what the 61 MiB is.

  ⚠ **Carry these with the numbers.** (i) **Forward-only** — `replay_step` is `&self`, so
  both arms share one solver and the comparison is purely the transient; the tape-based
  differentiable `step` is `&mut self` and was **not** measured, so this says nothing about
  a co-design batch. (ii) `NullContact`. (iii) The reduced arm's *fixed* cost is **higher**
  (440 vs 264 MiB) because the basis is fitted in-process; production would load `Φ`.
  (iv) Repeat spread at `K = 8` is ±339 MiB (thread scheduling), putting the reduced slope
  in a 33–63 MiB/env bracket — the 10× holds at the pessimistic end. (v) The basis was a
  smooth analytic full-rank stand-in — a **memory-shape probe**, not a validated POD basis;
  no accuracy claim follows from it.

  ★ **Two ways this nearly read wrong, both caught by controls rather than by review.**
  A 25 ms RSS sampler agreed with `getrusage` to 0.0 % on three pilot cells and **missed
  36.7 %** on the fourth, because the reduced path's largest transient is short-lived. And
  at `K ≤ 4` the reduced arm reads *flat* (506 → 504 MiB) — an artifact of the 506 MiB
  **construction** high-water mark masking a smaller stepping transient, `ru_maxrss` being
  monotone. Only pushing to `K = 16` resolved the true slope. Reporting either unchecked
  would have overstated the result.

  **Producer**: a ~200-line standalone binary outside the tree, public API only
  (`uniform_block` → §2b's exact free-DOF counts, `CpuTet4NHSolver`, `PodBasis::fit`,
  `ReducedNewtonSolver`, `libc::getrusage`), envs stepped in `std::thread::scope`.
  **No repo change is needed to rebuild it** — unlike §9's timing work, nothing was patched
  into `sim-soft` and nothing had to be reverted.
- The whole-session **peak** is ~1.9× the factor, and the factor dominates that peak. But
  of what a session *holds*, the factor is none of it.

### 2c. Q3 — the f32 question. **This is the finding that changes the plan.**

The brief framed this as one question. It measured as **two, with opposite answers.**

**(i) f32 for the numeric factor and every triangular solve: SAFE.** Measured by
routing `OrderedLlt` through an f32 `factorize_numeric_llt` over the *same*
(type-agnostic) symbolic factor, with the assembly and residual left in f64, and
comparing in-process against the f64 reference.

| fixture | free DOF | forward displacement rel-L2 drift | ∂x\*/∂μ rel-L2 drift | ∂x\*/∂x_prev rel-L2 drift |
|---|---:|---:|---:|---:|
| cantilever, dt 1e-3 | 540 | 1.35e-11 | 1.80e-6 | 1.36e-6 |
| cantilever, dt 1e-3 | 3 000 | 1.22e-11 | 8.92e-7 | 1.66e-6 |
| cantilever, dt 1e-3 | 8 820 | 5.41e-11 | 2.98e-6 | 6.37e-6 |
| cantilever, dt 1e-3 | 19 440 | 7.92e-11 | 5.20e-6 | 4.76e-6 |
| cantilever, dt 1e-3 | 36 300 | 9.21e-10 | 5.63e-6 | 5.22e-6 |
| **cantilever, dt 1/60 (hard)** | 540 | 1.59e-7 | 1.70e-4 | 1.76e-4 |
| **cantilever, dt 1/60 (hard)** | 3 000 | 4.31e-8 | 1.41e-4 | 2.35e-4 |
| **cantilever, dt 1/60 (hard)** | 8 820 | 2.24e-7 | **9.29e-4** | 2.92e-4 |
| block, dt 1/60 | 1 944 | 9.87e-10 | 1.05e-6 | 5.15e-6 |
| block, dt 1/60 | 26 460 | 4.45e-9 | 1.39e-5 | 2.94e-5 |

Newton took the **same number of iterations** in f32 as in f64 in every case, and
converged to the same tolerance. Forward drift is at worst 2e-7 relative.

⚠ **The gradient is the weak leg, and it is weaker than the crate's own tolerance.**
On the hard large-deflection fixture the material-sensitivity gradient drifts
**9.3e-4 relative** — about 3 significant digits. `sim-soft`'s gradcheck fixtures
assert to `approx`'s **5-digit** tolerance (the `dev-dependencies` comment says so
explicitly). **An f32-solve gradient would fail the existing gradcheck suite as
written.** It is accurate enough to *optimize* with (3 digits is far past what a
first-order method needs); it is not accurate enough to *validate* with against the
current assertions.

**(ii) f32 for the residual and the convergence test: NOT SAFE at present tolerances.**
This was measured directly by instrumenting the Newton loop to report
`‖r‖ / ‖f_int‖` at every iteration. At convergence on the cantilever:

```
CANCEL iter=5  r=2.287205e-2  |f_int|=9.412901e-1  r/|f_int|=2.430e-2
CANCEL iter=6  r=3.582871e-7  |f_int|=9.410815e-1  r/|f_int|=3.807e-7   <- converged
...
CANCEL iter=14 r=1.965036e-2  |f_int|=1.980189e0   r/|f_int|=9.923e-3
CANCEL iter=15 r=5.565373e-7  |f_int|=1.979991e0   r/|f_int|=2.811e-7   <- converged
```

The converged residual sits at **2.8e-7 – 3.8e-7 of ‖f_int‖**, against
`ε_f32 = 1.19e-7`. That is a headroom of **2.4× – 3.2×**. The residual
`r = M/Δt²·(…) + f_int − f_ext` is a difference of terms of order ‖f_int‖, so a
single-precision assembly could not represent the converged residual to better than
~1 significant digit. Newton would not stall *wrongly* — it would simply be unable to
tell it had converged, and the `tol = 1e-6` gate would become noise-limited.

**What this means for wgpu / Phase E.** The brief's pivot — "WGSL has no f64, so a
wgpu Phase E forces f32; if f32 gradients collapse, that changes the whole plan" —
resolves as: **f32 does not collapse, but the split is not where the plan assumed.**
Confirmed by grep: **zero** `f64` tokens appear in any of the 13 shaders under
`sim/L0/gpu/src/shaders/`; every one is f32. The workable architecture is therefore
*mixed precision, not uniform f32*:

- **On the GPU in f32**: the reduced tangent factor and solve, and the element/Gauss
  kernels that build it. Measured safe.
- **On the CPU in f64**: the residual norm and the convergence test — a reduction over
  the reduced (small) residual vector, which is cheap by construction in a reduced-order
  scheme and does not need to be on the GPU at all.
- **Compensated summation** is the alternative if the residual must live on the GPU
  (Kahan/Neumaier in f32 buys ~2 extra digits and is pure WGSL). Not measured here;
  flagged as R2's decision point, not assumed.

The no-C-dependency L0 rule therefore **survives** — wgpu is viable — but it survives
on a mixed-precision design, not on the uniform-f32 one the brief posited.

**What was NOT measured, and why.** A *fully* f32 `sim-soft` (assembly, constitutive
evaluation, geometry) was not measured, because it is not cheap: **1 396 `f64` tokens
across 60 production files (28 831 production LOC), including 186 nalgebra types
pinned to `f64`** (`SMatrix<f64, N, 3>`, `Matrix3`, `DMatrix<f64>`, `Vec3 =
Vector3<f64>` — the last is a public type alias, so genericising it is a breaking
change to the crate's public surface). Per the brief's own instruction, this is
reported as unmeasured rather than estimated. §7 R2 scopes it.

### 2d. Q4 — where the time actually goes

Same deterministic sweep, phase shares of total step time:

| free DOF | asm force | asm tangent | **numeric factor** | tri solve | contact |
|---|---:|---:|---:|---:|---:|
| 540 (cantilever) | 10.2 % | **50.9 %** | 31.6 % | 1.9 % | — |
| 3 000 | 7.6 % | 31.6 % | **57.5 %** | 1.6 % | — |
| 8 820 | 5.4 % | 22.6 % | **68.9 %** | 2.1 % | — |
| 19 440 | 4.7 % | 18.6 % | **72.3 %** | 3.7 % | — |
| 26 460 (block) | 1.8 % | 10.6 % | **62.1 %** | 3.1 % | — |
| 70 644 (block) | 1.3 % | 8.1 % | **70.8 %** | 2.8 % | — |
| 5 202 (IPC contact, ~~pre-R0~~) | 0.5 % | 17.7 % | **80.6 %** | 0.3 % | **0.02 %** |
| 5 202 (IPC contact, **post-R0**) | 2.3 % | 23.3 % | **67.3 %** | 2.5 % | **2.2 %** |
| 18 750 (IPC contact, ~~pre-R0~~) | 0.7 % | 30.1 % | **65.6 %** | 2.3 % | **0.02 %** |
| 18 750 (IPC contact, **post-R0**) | 1.6 % | **16.4 %** | **75.0 %** | 4.0 % | **1.0 %** |

Symbolic factorization is one-shot per solver construction (1.0 ms at 540 DOF →
1 029 ms at 70 644 DOF) and is **not** on the frame budget — it is amortised over
every step of a session. That is already the shipped design (`construct.rs` builds it
once; `replace_contact` deliberately reuses it).

Five findings:

1. **The numeric factorization dominates from ~1–2 k free DOF upward.** Assembly leads
   only at the very bottom of the range (50.9 % against 31.6 % at 540 free DOF); by
   3 000 the factorization has taken over (57.5 %) and by 19 440 it is 72.3 %. This is
   consistent with `Cargo.toml`'s "factorization is ~78 % of a solve (profiled)",
   measured on the Tet10 bonded-layer case at 27 k–70 k DOF.

   ⚠ **This crossover moved, and R0 is why.** Before R0 it sat at ~15 k–20 k free DOF,
   with assembly at 81.9 % of a 540-DOF frame — which is what made R0 worth doing, and
   which §4b's original argument was built on. R0 removed that cost and left the
   factorization exposed. Recorded because an optimisation that relocates the
   bottleneck invalidates the reasoning that justified it, and the pre-R0 numbers are
   in git (`b0f4aa21`) if anyone needs them.

2. **The assembly cost that used to dominate was a data-structure choice, not physics
   — and it is now fixed.** `assemble_free_hessian_triplets` rebuilt a
   `BTreeMap<(usize, usize), f64>` on **every Newton iteration** to produce a sparsity
   pattern already fixed at construction. R0 replaced it with a flat buffer indexed by
   that pattern: **1.51–1.89× on the whole step, byte-identical output**
   (`e77023c7`, `43b198a2`). The numbers above are post-R0.

3. **★ The contact rows were re-measured post-R0, and R0's credit is now measured
   DIRECTLY (v2.4).** `cargo test --release -p sim-soft --features phase-timing
   --test phase_shares -- --ignored --nocapture` for the shares;
   `tests/r0_ab.rs` for the credit. Both are permanent (`src/profile.rs`), not
   scratch patches.

   ⚠⚠ **v2.3 derived R0's credit by differencing phase SHARES across two
   sessions and got `1.186×`. That was wrong by 37 %.** Mean-of-rounds credit is
   `1.620×`; on the median frame it is `1.584×`, i.e. 34 % — quote the aggregate
   with the number, per §2f. The premise was "R0
   touched only tangent assembly, so every other phase's absolute cost is
   unchanged" — and this document's own data falsify it: across the two sessions
   `asm force` moves `2.4×` and `tri solve` `1.7×`, where the premise predicts
   they move by the credit. The two large phases agreed and the two small ones
   did not, which is what differing phase boundaries between two instruments
   look like. It is not recoverable after the fact.

   > **The credit is a ratio of WALL TIMES and both trees are in git.** Check out
   > each, run the same fixture on the same box, interleave the arms, divide.
   > No shares, no cross-session transfer, no instrument to trust.

   | fixture | pre-R0 (`ecf4cfef^`) | post-R0 | **R0 credit** | Newton iters/step |
   |---|---:|---:|---:|---:|
   | IPC 5 202 | 304.0 ms | 165.4 ms | **1.84×** | 6.12 → 6.12 |
   | IPC 18 750 | 1 502.8 ms | 927.7 ms | **1.62×** | 6.51 → 6.51 |
   | cantilever 80×8 | 7 239.2 ms | 4 133.2 ms | **1.75×** | 35.80 → 35.80 |

   Mean ms/step, 6 interleaved rounds per arm at 18 750, 3 at 5 202, 2 for the
   cantilever; one box,
   same toolchain (1.96.0), harness byte-identical on both trees. **Newton
   iteration counts are identical to the last digit on every fixture** — R0
   changed cost, not numerics, which is the byte-identity claim of finding 2
   holding up under a second, independent check.

   ★ **All three land inside the `1.51–1.89×` whole-step range R0's own PR
   reported (finding 2). v2.3's `1.186×` lands outside it.** That is the
   arithmetic error behind it: finding 2's range was already a WHOLE-STEP figure,
   and §2f re-read it as a figure "on the share" and then Amdahl-discounted it a
   second time, down to `1.17×`. The share-differencing then appeared to confirm
   the discounted number. Two independent mistakes agreeing is why it survived.

4. **⚠⚠ Cross-session ABSOLUTE timings in this document are not reliable, and the
   `46.2×` gap is one of them.** v2.3 excused a mismatch as "a box difference, not
   a regression". It is not: a box difference is a single factor, and three
   fixtures measured on both boxes do not admit one.

   | fixture | recon's box (§2a) | this box | recon ÷ this |
   |---|---:|---:|---:|
   | IPC 5 202, pre-R0 | 486.9 ms | 304.0 ms | 1.60× |
   | cantilever 80×8, pre-R0 (`b0f4aa21`) | 11 272 ms | 7 239.2 ms | 1.56× |
   | cantilever 80×8, post-R0 | 4 452 ms | 4 133.2 ms | 1.08× |
   | **IPC 18 750, pre-R0** | **771.0 ms** | **1 502.8 ms** | **0.51×** |

   Two fixtures put the recon's box ~1.6× slower; **IPC 18 750 alone puts it 2×
   faster.** No box factor does that. §2a's `771.0 ms` is the outlier, and it is
   the sole source of the `46.2×` gap that every R3 requirement in this document
   is divided down from. Its `a/cell` labels also do not reproduce its DOF counts
   under the current `dims_for`, so the fixture itself likely differed.

   ⇒ **Stop deriving the frame-budget gap from a pre-R0 figure divided by an
   inferred credit. Measure post-R0 directly, on a named box.** That is what §2f
   now does. The pre-R0 5 202 row remains impossible on its own terms (its shares
   imply R0 made the fixture 0.917× SLOWER), but that is a symptom of the same
   disease, not a defect peculiar to one row — which is how v2.3 read it, and why
   it discarded that row and kept the conclusion instead of the reverse.

   ⚠ **The `contact` column is the contact PATH, not contact itself.** Both timed
   blocks marshal every vertex into a fresh `Vec<Vec3>` and run the broad-phase
   scan before any pair exists, so the column never reads zero: `cantilever 80×8`
   measures **1.6 %** of its frame there under `NullContact`. It also means
   `slice_to_vec3s` + `active_pairs` run TWICE per Newton iteration (force and
   tangent). Neither is on R3's path, and neither is costed here beyond this note.

   ⛔ **That last sentence is OVERTURNED by §2k (v2.13).** It was written when a
   rung's verdict was thought to turn on a whole-frame speedup, where this cost
   is `1–2 %`. Under `C/R = B/I` only the IRREDUCIBLE time decides, and on the
   reduced path at 18 750 this cost is **`69 %` of it** — the largest single term
   in the quantity that says whether R3 clears. It is not just on R3's path; at
   that size it very nearly is R3's path.

5. **Triangular solves are never the bottleneck** (0.3–2.9 % everywhere). Worth
   stating because it is the phase a naive "put the solve on the GPU" plan targets
   first, and it would buy essentially nothing.

### 2e. Q4 addendum — contact costs 0.02 % of a contact-active frame

The `dynamic_indentation` fixture, 18 750 free DOF, IPC log-barrier active on all 71
increments, raw counters (nanoseconds, call count):

```
contact pairs = ( 9 640 605 ns, 1 370 calls)     ~0.007 ms/call
contact grad  = (   652 183 ns,   937 calls)     ~0.0007 ms/call
contact hess  = ( 2 897 202 ns,   433 calls)     ~0.007 ms/call
asm_tangent   = (16 449 475 369 ns, 433 calls)   ~38 ms/call
numeric factor= (35 925 160 619 ns, 433 calls)   ~83 ms/call
```

Total contact evaluation: **13.2 ms against 52.4 s of assembly + factorization** —
0.025 %. The timers do fire (non-zero counts and nanoseconds); this is a real
measurement, not an unwired probe.

**Contact is not a compute cost. It is a structure cost.** What contact does to the
frame is (a) hold Newton at ~6 iterations where the unconstrained stiff block needs
0.5, and (b) add the barrier Hessian's rank-1 `κ·n⊗n` blocks to the tangent, which is
what the factorization then has to chew. That reframing matters for §5: a
hyper-reduction scheme that "makes contact cheap" is solving a problem that does not
exist.

### 2f. The OTHER factor — Newton ITERATION COUNT is a large, cheap lever

**Measured 2026-08-23** on `spike/newton-predictor`: the five original cells at
`368a73a0` (the commit that added the knob), the sixth — contact plus a body load,
added after review — at `98573f48`. ⚠ **Not a `main` SHA**: `main` was at `2dfe1335`, and because this
repo squash-merges, `368a73a0` will not survive onto `main` either. Re-run from the
harness named below rather than from a SHA. §2a decomposes a frame as
`iterations × per-iteration cost` and every rung of the §7 ladder moves only the
second factor — R1.1 measured the reduced solve taking *identical* iteration
counts to the oracle (⚠ inside its own load box only — #817 measured a ~17 %
premium outside it), and substepping measured 1.4× worse (§3c). The first
factor had never been attacked. It turns out to be the cheaper of the two.

**What changed in the code**: `SolverConfig::initial_guess`, a three-way choice of
where Newton starts. Backward Euler's residual is
`r(x) = (m/Δt²)·(x − x̂) + f_int(x) − f_ext` with `x̂ = x_prev + Δt·v_prev`, so the
choice is *which term the first iterate already satisfies*:

| variant | `x⁰` | zeroes |
|---|---|---|
| `PreviousState` (default) | `x_prev` | nothing — the pre-existing behaviour |
| `Inertial` | `x̂` | the inertial term, bit-exactly |
| `InertialWithLoad` | `x̂ + Δt²·f_ext/m` | everything but `f_int(x⁰)` |

`InertialWithLoad` is the exact backward-Euler answer for a DOF with no internal
force (BE composes to `x̂ + Δt²a` with **no** `½` — it is not the continuous
constant-acceleration kinematic), i.e. the incremental-potential literature's
predictive position. This is an *addition*, never a substitute: the default is a
no-op match arm, the pre-predictor path is bit-equal, and the guess changes the
path Newton walks, never the root. ⚠ **"Never the root" is a claim about where
Newton lands WHEN IT LANDS, not a promise that it always does** — the
contact-plus-load cell below has an arm that never converges at all. Where every
arm does converge, they agree: `6e-14`–`8e-12` relative on the four original
subject fixtures, `6e-9` on the `block_sag` control (still same-root; that fixture
is simply where the arms take wildly different PATHS).

**Method.** Three arms stepped in **lockstep** from a common initial state (arm 0
step k, arm 1 step k, …), never as block-ordered runs, so machine drift lands on
all three equally. Step-by-step interleaving *inside* a trajectory is impossible:
the arms reach the same root each step but not the same bits, so after step 1 they
are three genuinely different trajectories. `dt = 1/60`, `tol` unchanged at
`1e-10` (a predictor must not be allowed to buy iterations by converging less),
Newton cap 120. Harness: `sim-soft/tests/predictor_spike.rs`, `#[ignore]`d — it
carries the matrix and the pre-registered decision rule in its module docs.
⚠ Same contended box as the rest of this document; the RATIOS are the
trustworthy part.

| fixture | free DOF | arm | ΣIters | vs base | p50 | p95 | max | ms p50 | ms max |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|
| `block_sag` | 1 944 | `PreviousState` | 11 | 1.00× | 0 | 1 | 2 | 2.2 | 17.5 |
| | | `Inertial` | 38 | **3.46× worse** | 1 | 2 | 2 | 8.8 | 16.2 |
| | | `InertialWithLoad` | 360 | **32.7× worse** | 6 | 6 | 6 | 43.4 | 44.5 |
| IPC indentation | 5 202 | `PreviousState` | 433 | 1.00× | 6 | 7 | 7 | 156.8 | 184.9 |
| | | `Inertial` | 205 | **0.473×** | 2 | 5 | 6 | 58.6 | 157.5 |
| | | `InertialWithLoad` | 205 | **0.473×** | 2 | 5 | 6 | 57.8 | 156.1 |
| IPC indentation | 18 750 | `PreviousState` | 461 | 1.00× | 6 | 7 | 8 | 849.3 | 1 112.1 |
| | | `Inertial` | 236 | **0.512×** | 3 | 6 | 6 | 431.5 | 855.6 |
| | | `InertialWithLoad` | 236 | **0.512×** | 3 | 6 | 6 | 432.0 | 849.1 |
| `cantilever 40×4` | 3 000 | `PreviousState` | 746 | 1.00× | 10 | 33 | 38 | 75.8 | 296.9 |
| | | `Inertial` | 238 | **0.319×** | 4 | 5 | 10 | 31.7 | 76.7 |
| | | `InertialWithLoad` | 248 | **0.332×** | 4 | 5 | **5** | 31.9 | **39.5** |
| `cantilever 80×8` | 19 440 | `PreviousState` | 1 134 | 1.00× | 13 | 44 | 57 | 1 461.5 | 6 494.1 |
| | | `Inertial` | 241 | **0.213×** | 4 | 5 | 12 | 458.2 | 1 371.2 |
| | | `InertialWithLoad` | 278 | **0.245×** | 5 | 5 | **5** | 565.8 | **590.3** |

Zero convergence failures on any subject fixture — no Armijo stall, no iteration
cap, no validity violation, no non-finite state.

##### ⛔ Adding the missing cell: contact AND a body load kills `InertialWithLoad`

IPC 5 202, `gravity_z = ±9.81`, both directions because only one is adversarial
(`−z` loads the layer away from the indenter above it; `+z` drives it in):

| body load | `PreviousState` | `Inertial` | `InertialWithLoad` |
|---|---:|---:|---|
| `−z` (away from collider) | 434 | 205 (0.472×) | **DIED, step 0** — `ArmijoStall @ iter 0, ‖r‖ = NaN` |
| `+z` (into collider) | 433 | 205 (0.473×) | **DIED, step 0** — `ArmijoStall @ iter 0, ‖r‖ = 3.67e6` |

**Both directions, on the first step.** The scale says why: `Δt²g` at 60 Hz is
**2.7 mm**, against an IPC band `d_hat` of **2.5e-5 m** — `109×` the length scale the
barrier operates on, and 43 % of the layer's 6.4 mm thickness in a single iterate.
⚠ The `−z` failure is a `NaN`, and it is **not** the barrier: a uniform 2.7 mm
predicted drop on a layer whose base is PINNED inverts elements near the bottom, so
`det F ≤ 0`. The load term fails on the kinematic side as well as the contact side.

⇒ **`Inertial` is the robust arm.** It never once failed to converge, on any
fixture. ⚠ It does not win everywhere: on `block_sag` it is 3.5× worse than
`PreviousState` (38 vs 11) — see §2g, which measures all four arms together. `InertialWithLoad` wins the tail on ONE non-contact
fixture and is catastrophic on two of the four fixture classes (`block_sag` 32.7×
worse; contact-plus-load fatal). **Nothing here supports shipping the load term as a
default**, and a selector stops being an optimisation and becomes the precondition
for using it at all. ⛔ The `‖r‖` form of that selector was then built and killed
(§2g); the load term remains unshippable and its tail win unclaimed.

**On the IPC pair being identical — and what that hid.** `Inertial` and
`InertialWithLoad` agree to the digit on both IPC rows. That was predicted before the
run, not observed after it: the indentation fixture is displacement-controlled with
`gravity_z = 0` and an empty θ, so `f_ext ≡ 0` and the two variants are the same
point by construction. The harness's own self-check, and it passed.

⚠ **It also meant `Δt²·f_ext/m` — the ONLY term distinguishing the two arms — was
never exercised against a contact barrier.** The matrix crossed fixture × size ×
guess but never *load × contact*, and that omission sat directly under the arm the
variance result below recommends. Closed by measurement, not by caveat:

#### The gain, and what it does to the ladder

⚠ **Quote these with both their metric and their aggregate attached** — an earlier
revision of this paragraph paired a wall-clock ratio for one fixture with an
iteration ratio for the other, which flatters the pair by ~47 %.

| fixture | ΣIters | wall-clock, total | wall-clock, median frame |
|---|---:|---:|---:|
| IPC 18 750 (**representative**) | **1.95×** | 1.90× | 1.97× |
| `cantilever 80×8` (hard end) | **4.71×** | 4.63× | 3.19× |

Iterations and total wall-clock track each other closely, as they must. The
cantilever's *median* is the outlier at 3.19× because the baseline's total is
dominated by transient spikes the median never sees — which is the same fact §2f's
variance section is about, seen from the other side.

#### ★ Verdict against the PRE-REGISTERED rule

The harness's module docs fixed a decision rule before the first run: **WIN** =
≥20 % fewer total iterations on the cantilever AND ≥10 % on IPC with zero failures;
**KILL** = IPC loses convergence, or iteration count rises. Scored, rather than
re-argued from the prose above:

| arm | cantilever | IPC | failures | **verdict** |
|---|---:|---:|---|---|
| `Inertial` | −78.7 % | −48.8 % | none, any cell | **WIN** |
| `InertialWithLoad` | −75.5 % | −48.8 % | **IPC + load: dies at step 0** | **KILLED** |

The rule discriminates the two arms cleanly and it fired on its own terms — the
KILL condition is *"IPC loses convergence"*, and that is precisely what the sixth
cell produced. Recording it this way matters because the rule predates the data:
the alternative is choosing an arm from a prose argument written afterwards.

#### What it does to R3's requirement — derived here, from this document

The consequence for R3 is the reason this was measured before it. ⚠ An earlier
revision of this paragraph cited "§7 records 12–16×"; **§7 records no such figure**
— the only ×-value in it is R3's `10×` kill floor. The requirement is derived, and
the derivation belongs in the open where it can be audited, so here it is in full.

⚠⚠ **REPLACED in v2.4. The `9.89×` this section reported is WITHDRAWN.** It was
`pre-R0 gap ÷ R0 credit ÷ R1 ÷ predictor`, and two of those four factors were
unsound: the gap came from §2a's `771.0 ms`, which §2d finding 4 shows is the one
figure of three that no box factor reconciles, and the credit came from
cross-session share differencing, which §2d finding 3 shows was wrong by 37 %.
The old derivation is in git (`4545a2f6`); reproducing it here would only give a
wrong number a second airing.

**Dividing a pre-R0 gap by an inferred credit was the mistake.** A post-R0 gap can
simply be MEASURED — the code is what ships, the box is nameable, and no credit
enters the arithmetic at all:

| IPC 18 750, measured post-R0 on this box | value |
|---|---:|
| median frame (`tests/r0_ab.rs`, 6 interleaved rounds) | **887.9 ms** |
| gap to a 16.7 ms frame | **53.2×** |
| ÷ R1 `2×` (inside R1.1's load box) ÷ predictor `1.97×` | **R3 needs 13.5×** |
| ÷ R1 net `1.71×` (outside it, #817) ÷ predictor `1.97×` | **R3 needs 15.8×** |

> ### R3 needs 13.5–15.8× against a 10× kill floor — ABOVE it on every accounting.
>
> **R0's credit does not appear in this arithmetic at all, and that is the point.**
> `887.9 ms` is measured on the shipped code, so R0's benefit is already inside it;
> dividing by a credit again would double-count it. R0 is bigger than v2.3 thought
> (`1.62×`, not `1.186×`) — the requirement still rises, because the two errors
> pushed opposite ways and the gap error was the larger.
>
> ⚠ Compare like with like: v2.3's `46.2×` was a PRE-R0 gap, so its implied post-R0
> gap was `46.2 ÷ 1.186 =` **`39.0×`**. The measured post-R0 gap is **`53.2×`** —
> **1.37× worse than assumed**, and that factor is what moves R3 from `9.89×` to
> `13.5×`. **R3 can pass its own `10×` gate and still leave the frame budget
> missed** — the exact situation v2.3 claimed the predictor had removed.

⚠ **This is a per-box statement and it is not portable.** A frame budget is
absolute, so the gap belongs to the machine it was measured on; only the ratios
(R0, R1, the predictor) transfer. §2d finding 4 is the evidence that they are the
only things that do. ✅ **The reference box is now FIXED and enforced in code —
§2h.** `Mac16,8` (M4 Pro, 8P+4E), with a piloted contention gate that refuses a
gate-bearing run on a loaded box. Every `×` above is a quantity on that machine.

⚠ **What is NOT claimed.** That `53.2×` is the gap on the deployment target: no
such target is fixed anywhere in this document, and this box is one sample. What
IS claimed is that the previous `46.2×` was not one either, and had a measurement
behind it that three fixtures contradict.

✅ **Done in v2.3** — this was on R3's critical path as the single input deciding
which row was real, and §2d findings 4–5 measured it.

⚠ **The factors multiply only if the predictor is ported to the reduced solver.**
✅ **DONE — the port landed in #817 (`d23a29f1`) and the composition HOLDS**
(1.000 at R1.1's operating point). ⚠ That PR also found the reduced solve paying an
**iteration-count premium outside R1.1's load box** — ~17 % at ×2 load, not
truncation and not a residual floor but a rate effect. ⚠ **That is a premium on the
COUNT, not a change to R1's per-iteration `~2×`**; conflating the two is the
mixed-aggregate error this document keeps making (see §2f's "quote these with
both their metric and their aggregate attached"). What it does mean is that R1.1's
iteration-count PARITY (reduced never exceeds oracle) is a property of its load box,
and the ladder's composed figure inherits that. The paragraph below is the pre-port
reasoning, kept for the record.

R1's ~2× is a *per-iteration* saving and the predictor's is an *iteration-count*
saving, so they compose — but `reduced::newton` starts from `q_prev` and has no
predictor, so today a reduced solve would keep the full iteration count. That port
is small and is the top follow-up. Until it is measured, the composed figure is a
projection, not a result.

#### ★ The variance finding — `InertialWithLoad` makes frame cost nearly CONSTANT

§3c records that reduction preserves iteration counts exactly and therefore does
nothing for the *spike*, while VR is a p99 problem: a scene averaging 8 ms that
spikes to 40 is unusable. The predictor is the first thing measured here that
attacks the spike directly. Ratio of `ms max` to `ms p50` on `cantilever 80×8`:

| arm | ms p50 | ms max | max / p50 |
|---|---:|---:|---:|
| `PreviousState` | 1 461.5 | 6 494.1 | **4.44×** |
| `Inertial` | 458.2 | 1 371.2 | 2.99× |
| `InertialWithLoad` | 565.8 | 590.3 | **1.04×** |

Read against `Inertial` — the arm it is actually competing with —
`InertialWithLoad` costs **15 % more total iterations** (278 vs 241) and **23 %
more median frame time** (565.8 vs 458.2 ms) and buys a **2.32× tail cut**
(1 371.2 → 590.3 ms). Against the `PreviousState` baseline the tail cut is
**11.0×** (6 494.1 → 590.3 ms). Either way it turns a workload spiking 4.4× above
its own median into one flat to within 4.3 %.

⚠ Those are three different comparisons and it is easy to quote a ratio from one
against a baseline from another; the `max / p50` column above is the one figure
that needs no baseline at all. **On totals `Inertial` wins and on the tail
`InertialWithLoad` wins — mean and tail rank the two options oppositely**, cf.
`feedback_metric_choice_can_invert_findings`.

⛔ **This is a result about the TERM, not a recommendation for the ARM.** Measured on
one non-contact fixture; the same term is fatal on contact-plus-load (above). What
survives is: *a load-aware predictor can flatten the frame-time tail, and something
must gate when it fires.*

#### ⚠ `block_sag` got 32.7× WORSE, and that is the boundary of the technique

The negative control was pre-registered as "must not move — if it moves, the
harness is wrong". It moved, hard. The harness is not wrong; **the rule was
mis-specified.** `block_sag` was chosen because it has no *upside* (it converges at
iterate 0 already), and that was silently written down as though it also had no
*downside*. Its downside is wide open.

The result is real, not an artifact: all three arms converge to the same trajectory
(`6e-9` relative), the damage is monotone in how far the guess is thrown, and the
arithmetic is elementary — `Δt²g` at 60 Hz is **2.7 mm** of free fall per frame,
while a stiff pinned 5 cm block actually sags micrometres. The predictor is a bet
that a DOF's motion over one frame is **inertia-dominated**. It pays where the body
genuinely moves ballistically between frames and loses badly where stiffness pins
it, and the loss is largest for exactly the variant that is best elsewhere.

⇒ **`InertialWithLoad` cannot be an unconditional default.** The obvious selector
looked cheap: evaluate `‖r‖` at the candidate guesses and start from the smallest,
for a couple of extra internal-force assemblies per step.

⛔ **BUILT AND KILLED — see §2g.** The cost behaved as predicted in SHAPE (measured −0.15 % to
+6.70 % of total wall-clock; three candidates, not two), and it does rescue every cell the load term dies on. But it
selects the WORST arm on the cantilevers, because `‖r(x⁰)‖` is not a proxy for
distance to the root. Read §2g before proposing a variant of this; the paragraph
above is kept as the motivation it was, not as a live recommendation.

#### Two smaller things worth recording

- **The `block_sag` control's premise came out 3× stronger than pre-registered, and
  the same window effect explains it.** It was chosen against §2a's `0.5–0.6
  iters/step`; the baseline arm measures **11 iterations over 60 steps = 0.18**, with
  `p50 = 0` — most steps converge at iterate 0 without a single Newton iteration.
  That does not weaken the control, it sharpens it: the fixture has even less to give
  than the figure it was picked on, so its 32.7× degradation is measured against a
  floor that is nearly zero. Same cause as the cantilever row below — §2a's iteration
  table does not state its step count, and both mismatches resolve if it covered a
  short, transient-weighted window.
- **This document's `37.0 iters/step` for `cantilever 80×8` is a transient-window
  figure.** The baseline arm above reproduces the *peak* almost exactly (57 here vs
  56 in §2a/§3b.4) but its mean over 60 steps is **18.9**, because iteration count
  decays sharply once the beam stops falling from rest. §2a's iteration table does
  not state its step count and the two are consistent if it covered ~10 steps. Any
  argument resting on "37" is resting on the first fraction of a second.
- **The predictor helps the median far more than the tail on IPC** (p50 6→3, max
  8→6), and the opposite on the cantilever. Contact appears to set an iteration
  floor the predictor cannot get under — consistent with §2e's finding that contact
  is a *structural* cost that holds Newton at ~6 iterations.

---

### 2g. The `‖r‖` selector — built, measured, KILLED

§2f named a per-step selector as the precondition for using the load term:
evaluate `‖r‖` at each candidate guess, start from the smallest. Built as
`InitialGuess::Adaptive`, measured, **removed**. ⚠ The code was squashed away and
exists in no branch — **§9b carries the re-apply recipe**, and these numbers
cannot be re-derived without it.

| fixture | `PreviousState` | `Inertial` | `InertialWithLoad` | **`Adaptive`** |
|---|---:|---:|---:|---:|
| `block_sag` 1 944 | **11** | 38 | 360 | **11** ✓ |
| `cantilever 40×4` | 746 | **238** | 248 | **746** ✗ |
| `cantilever 80×8` | 1 134 | **241** | 278 | **1 134** ✗ |
| IPC 5 202 | 433 | **205** | 205 | 205 ✓ |
| IPC 18 750 | 461 | **236** | 236 | 236 ✓ |
| IPC + body load, −z / +z | 434 / 433 | **205 / 205** | **DIED / DIED at step 0** | 205 / 205 ✓ |

**It does the safety job**: rescues the only cell the load term dies on, in both
load directions, and on `block_sag` avoids both the 32.7× blowup and `Inertial`'s
3.5× penalty. Overhead **−0.15 % to +6.70 %** of wall-clock, against the arm it
selected (three extra assemblies per step; two is the floor, since Newton needs
one of them anyway).

⛔ **And on `cantilever 80×8` it selects the worst arm — 4.71× on iterations,
4.60× on wall-clock** (40×4: 3.13× / 3.05×). It discards the predictor's largest
win, on the fixture the arc was motivated by. **KILLED** on either metric.

#### ★ Why — `‖r(x⁰)‖` is not a proxy for distance to the ROOT

| fixture | ranking picks | correct? |
|---|---|---|
| IPC (± load) | `Inertial` | ✅ |
| `block_sag` | `PreviousState` | ✅ (11 vs 38) |
| **`cantilever` 40×4, 80×8** | `PreviousState` | ⛔ **wrong** |

The split is **the cantilever against everything else**, not contact against
gravity — `block_sag` is gravity-driven bending and the ranking is right there.
The rule fails on the one large-deflection, high-iteration fixture, i.e. exactly
where iteration count is worth optimising, and works wherever it matters least.

On the cantilever, `x_prev` has the SMALLER residual — the inertial term `−M·v/Δt`
is modest for a slowly-moving beam while moving to `x̂` changes `f_int` by more
than it saves — yet `x̂` converges 3–5× faster because it is nearer the root.

⚠ The failure is **silent**: every arm converges, nothing errors, only the
iteration count shows it. A selector judged on "did it converge" would ship.

★ **No evidence the per-step adaptivity was ever exercised.** `Adaptive`'s totals
equal a fixed arm's exactly on every row. ⚠ On the three IPC rows without a body
load that proves little — `f_ext ≡ 0` there makes `Inertial` and
`InertialWithLoad` the same point, so equal totals cannot distinguish "never
switched" from "switched between indistinguishable arms". On `block_sag` and the
two cantilevers the arms are genuinely distinct and the totals still match
exactly, which is where the claim actually rests.

#### ⇒ Where this leaves the load term

**Use `Inertial`.** Never failed on any fixture, best-or-tied on five of six, and
zero extra assemblies. Its worst case is `block_sag`, 3.5× off optimal at 409 ms
absolute.

**A kinematic veto by ELEMENT SIZE is dead too — on arithmetic, without a run.**
`Δt²g` = 2.725 mm is fixture-independent, so per element:

| fixture | element | `Δt²g` / element | the veto must |
|---|---:|---:|---|
| `block_sag` 1 944 | 6.25 mm | **0.44** | VETO (360 iters, 32.7× worse) |
| `cantilever 80×8` | 2.50 mm | **1.09** | ALLOW (best tail) |
| IPC + body load | 1.06 mm | **2.56** | VETO (dies at step 0) |

The required action is **VETO / ALLOW / VETO** as the ratio rises — non-monotonic,
so **no single element-size threshold separates them.**

⇒ What survives is narrower: a **contact-band** veto, where `Δt²g` is **109×**
`d_hat` = 2.5e-5 m and the margin is not close. That handles the contact cell;
`block_sag` needs a different mechanism entirely, and nothing here suggests one.
Until then the load term is unshippable and its tail win — **2.99× against
`Inertial`**, the arm it competes with, not the 4.44× against the baseline —
unclaimed.

---


### 2h. The REFERENCE BOX — what a `×` in this document is measured against

A frame budget is ABSOLUTE, so a gap to it belongs to the machine that produced
it. This document went four revisions without honouring that and withdrew its
headline twice as a result. The reference box is now fixed, and enforced in code
rather than in prose (`sim/L0/soft/tests/refbox/mod.rs`).

| | |
|---|---|
| **Model** | `Mac16,8` — Apple M4 Pro |
| **Cores** | 8 performance + 4 efficiency (12 logical) |
| **RAM** | 24 GiB |
| **OS / toolchain** | macOS 26.6.2 · rustc 1.96.0 (pinned by `rust-toolchain.toml`) |

**Hard-failed:** model and the P/E core split — the hardware the cost scales
with. **Stamped only:** RAM, OS, rustc; they drift for reasons unrelated to
speed, and the toolchain is pinned elsewhere. Every measurement prints the block,
so a figure cannot travel without its box again.

**All four inputs to R3's requirement are gated**, not just the two that
prompted this. R3 needs `gap ÷ R1 ÷ predictor`, and those terms come from four
harnesses — `phase_shares.rs` and `r0_ab.rs` (the gap), `reduced_predictor.rs`
(R1's `1.71×` net), `predictor_spike.rs` (the predictor's `1.97×`). ⚠ The last
two were missed on the first pass, and the predictor's `1.97×` is a **wall-clock
median-frame ratio** (§2f's third column), not an iteration count — so it is
box-sensitive exactly like the gap. Gating half the inputs and calling the box
fixed would have been an overclaim. Eleven call sites; the CI-run tests in those
files are deliberately left ungated.

#### The contention gate, and why the obvious statistic was the wrong one

Background load is the bigger threat, not different hardware. §2's own header
records the same 3 000-DOF fixture at **4.9 ms and 28.0 ms** in two runs — `5.7×`
from load alone, against the `~1.6×` seen across boxes.

The measurement fixtures cannot police themselves: the IPC ramp deepens its
indenter every step, so `max/p50` is `1.31` from physics, and the
`cantilever 80×8` transient runs 8× min-to-max. So a **separate probe** runs
first (~0.6 s): `cantilever 40×4`, 3 000 free DOF, `dt = 1e-3`, which holds the
Newton count constant at 3 — identical work every step, so any spread has one
source. Measured at **~61 % `numeric factor`** (61.1 / 60.8 % on two runs), so it loads the phase the real
measurements are bottlenecked on. The probe **checks its constant-work premise
and refuses to report if the count varies**; it caught a `[2, 3, 3, …]` first
step on its first run and declared itself invalid until warm-up was raised.

Thresholds were **piloted, not chosen** — 10 runs idle, then under deliberate
load. ⚠ The table below is the **second** pilot: §2i's parallel validity sweep
moved the probe's quiet median `24.47 → 22.75 ms`, the floor caught it, and every
constant was re-derived. The first pilot's numbers are kept underneath because
the two agree on the finding.

| load | `burst` (max/p50) | probe median |
|---|---|---:|
| idle (10 runs) | 1.008 – **1.054** | 22.60 – **23.36** ms |
| 4 cores busy (10 runs) | 1.030 – **1.078** | **24.17** – 24.93 ms |

First pilot, before the sweep went parallel:

| load | `burst` (max/p50) | probe median |
|---|---|---:|
| idle (10 runs) | 1.008 – **1.060** | 23.80 – **24.70** ms |
| 2 cores busy | 1.015 – 1.223 | **25.83** – 26.18 ms |
| 4 cores busy | 1.022 – **1.063** | 26.15 – 26.61 ms |
| 12 cores busy | 2.374 – 5.873 | 82.0 – 143.9 ms |

⚠ **The finding: `burst` does NOT detect sustained partial load.** At 4 busy
cores the second pilot reads `1.030–1.078×` against an idle `1.008–1.054×` — the
two ranges are `2.3 %` apart, no separation — because steady load slows *every*
step and therefore lands in the median. An earlier draft of the probe documented
`burst` as "the contention statistic" and claimed a median would hide what it
catches; that was backwards, and the pilot is what said so. **Replicated on a
different tree**, which is more than the original finding had. Both checks ship,
for different failure modes:

- **`probe median ≤ 23.7 ms`** — the sensitive check. Sits in the measured gap
  between idle max `23.36` and the lightest contaminated case `24.17`. Only
  `1.5 %` headroom, deliberately; a genuine baseline shift must be re-piloted,
  not accommodated by widening this.
- **`burst ≤ 1.30×`** — the gross-stall backstop, for a few long preemptions on
  a box whose median looks fine. Unchanged.
- **`probe median ≥ 19.5 ms`** — not a contention check. Catches the probe
  getting *cheaper*, which means the workload or solver moved and every constant
  here is stale. **It has now done exactly that once**, which is what produced
  this second pilot.

Negative-controlled end to end, both pilots: 3 cores busy refused a gate-bearing
run in `0.69 s` under the first, 4 cores busy in **`0.62 s`** under the second
(probe median `23.92 ms`), each naming the cause. Under the first, `burst` read
`1.02×` in the refusing run — precisely why it is not the gate on its own.

⚠⚠ **The probe measures the SUT, so this recurs.** It is a **full-order** solve, so
every optimisation on the full-order path invalidates all four constants. ⚠ Note what
that does NOT include `project_tangent`, §2i's next item: it is purely reduced-path and
the probe never runs the reduced path, so these constants should hold across it. ⚠ It
DOES include one change §2i openly invites — the probe is built with `NullContact`, so it
pays the same broad-phase and marshalling §2i measures at `2.9 %` and calls "free to
remove". Removing it would move the probe. So would any change to full-order assembly,
ordering or factorization. The design that would not need
re-baselining
is a probe independent of the code under test — a fixed synthetic workload
measuring the box alone. Deliberately not taken: the present probe is *measured*
to load the phase the gate-bearing runs are bottlenecked on, and a synthetic one
would have to re-earn that. Named so the next re-pilot can weigh the swap rather
than rediscover the cost.

⚠ **The same probe spreads `~7 %` across test BINARIES of one tree.** Readings of
`21.79` / `21.94 ms` came from `--features phase-timing` builds while
`refbox_pilot` and `r0_ab` sat at `22.6–23.4 ms`. Four observations is not an
explanation, but it is a floor under how tight any of these constants can ever be,
and `PROBE_P50_FLOOR_MS` is set from the lowest of them rather than from the
pilot's own minimum.

⚠ **The pilot's 10 runs undersample the tail, and the first real gated run said
so**: it passed at probe median `24.59 ms` (3.7 % under the ceiling) with burst
`1.07×` — already above the pilot's idle maximum of `1.060×`. `burst`'s `1.30×`
absorbs that; the median's `3.3 %` has less room. Expect this gate to fail
occasionally on a busy day. That is the intended bias — it fails CLOSED, and a
refused measurement costs a re-run while a contended one costs a retracted
finding.

#### ⚠ The gate is WEAKER for cross-tree A/B, and that is measured

`tests/r0_ab.rs` runs the same fixture on two trees, and the probe is
`cantilever 40×4` — a fixture **the optimisations themselves speed up**. On the
pre-R0 tree the probe reads **57.39 ms** against **24.47 ms** at R0 and
**22.75 ms** once the validity sweep went parallel, so applying the absolute
median ceiling would reject the pre-R0 arm on every run and the A/B could never
execute. This was caught by review before it shipped, not by the
gate.

`burst` IS scale-free and does transfer — idle pre-R0 reads `1.013–1.063×`
against post-R0's `1.008–1.060×` — so the cross-tree gate checks identity and
burst, and skips the median. **The cost: that arm has weaker contention
protection**, since burst misses sustained partial load. What covers it is
INTERLEAVING, measured rather than assumed:

| under a 3-core load | absolute times | **R0 credit** |
|---|---:|---:|
| IPC 18 750 | +3.4 % | **+0.4 %** |
| IPC 5 202 | +4.0 % | **+1.4 %** |

The load lands on both arms and leaves the ratio, which is why interleaving is
mandatory in `r0_ab.rs` rather than advisory.

#### R3's two numbers, both on this box

The document has been conflating these. They are different quantities:

| | value | what it decides |
|---|---:|---|
| **Feasibility gate** | **`I ≤ 16.7 ms`** | whether 60 Hz is reachable at all |
| **Frame-budget requirement** | **13.5 – 15.8×** | what R3 must actually deliver |
| ~~R3's own kill floor~~ | ~~`≥ 10×`~~ | ⇒ **§2j** — restated; now a complexity heuristic, not a gate |

⚠ **The kill floor was RESTATED in v2.8, before the measurement that would have
moved it** — it was a ratio over a baseline every rung on this ladder is chartered
to improve, so it got harder each time an earlier rung succeeded. §2j derives the
replacement: R3 clears iff its *irreducible* time fits the frame budget.

Both are quantities **on the reference box** and neither is portable. R3 passing
its floor and missing the budget is not a contradiction — it is the situation
§2f now predicts.

⚠ **Transfer to another box is UNVALIDATED, and deliberately not offered.**
Validating a transfer rule needs at least two boxes; one is available. The only
cross-box evidence here has it failing in opposite directions across fixture
families (`1.60× / 1.56× / 1.08×` … and `0.51×`), so a plausible-looking scaling
factor would be worse than none. Anything measured elsewhere is a different
number wearing the same name.

### 2i. R3's Amdahl ceiling — measured, and then unstraddled

§4b picks ECSW on a literature claim of **2–3 orders of magnitude**. That is a
speedup on *element integration*; R3's whole-step gain is capped by whatever
fraction of a REDUCED frame that is. Nobody had measured it — §2d is the
full-order path and the reduced solver carried no timers — so R3 was about to be
built without knowing its own ceiling (`tests/reduced_phase_shares.rs`).

**R1.1's operating point**: `16×16×6` cantilever bilayer, 5 202 free DOF, `r = 40`,
`NullContact`, 4.00 Newton iterations/step.

Measured, the bracket **straddled** the requirement — and one line of the table
decided it. That line has since been fixed. Both measurements are kept, because
the route from the first to the second is the reusable part.

#### As first measured (#822) — `72.07 ms/step`, the sweep serial

| phase | ms/step | share | ECSW removes it? |
|---|---:|---:|---|
| **`red proj K`** (`ΦᵀKΦ`) | **37.591** | **52.2 %** | yes |
| `asm tangent` | 23.880 | 33.1 % | yes |
| **`validity check`** | **6.684** | **9.3 %** | **only if R3's own design works** |
| `asm force` | 2.380 | 3.3 % | yes |
| `contact` (nested) | 1.874 | 2.6 % | no |
| `red proj r` (`Φᵀr`) | 0.977 | 1.4 % | yes |
| `red expand` | 0.453 | 0.6 % | yes |
| `residual form` | 0.038 | 0.1 % | only if R3's own design works |
| `red dense solve` (`r × r`) | 0.021 | 0.0 % | no |

Instrumented/wall **99.9 %**, so the denominator is essentially complete.

| bound | reducible | **ceiling `1/(1 − f)`** |
|---|---:|---:|
| certain only | 88.0 % | **8.3×** |
| plus what R3's own design removes | 97.4 % | **≳37×** ⚠ |

`8.3× … ≳37×` against a `13.5–15.8×` requirement and a `10×` floor: it straddled
both, so **the bound decided everything**, and what moved the bound was almost
entirely the `validity check` row.

#### What that row actually was: a serial `for` loop

`check_validity_at_step_start` walks every tet — a `det F` certificate plus a 3×3
SVD — and runs twice per step, at step start and again on convergence. No
cross-element coupling. `Mesh`, `Material`, `Element` and `ContactModel` were
already `Send + Sync` supertraits, so it parallelises with **no new trait bound
and no signature change**.

Under rayon on the reference box (12 threads, 9 216 tets):

| state | sequential | parallel | speedup |
|---|---:|---:|---:|
| rest | 1.506 ms | 0.311 ms | 4.83× |
| sheared `u_x = 0.15 z` | 2.050 ms | 0.382 ms | **5.37×** |

✅ **RE-MEASURED on the current tree (v2.15 audit), and it reproduces almost
exactly**: `seq 2.0515 ms`, `par 0.3790 ms`, **`5.41×`** — `0.07 %`, `0.8 %` and
`0.7 %` off the three figures above, across sessions and across every change
since. ★ **Why this row is durable when §2j/§2k's drifted:** it comes from an
in-binary seq-vs-par A/B taking a `p50` over repeats inside one process, so the
two arms share the machine state and the ratio cancels almost everything. The
figures that moved were single-run PHASE SHARES, where each number is one
sample of an absolute time. ⇒ **When a claim has to survive, build it as a
same-process A/B over repeats, not as a share.**

⚠ The `9.3 % → 1.9 %` frame shares in this section are PRE-`project_tangent` and
cannot be re-checked on the current tree at all: the denominator changed when
§2j nearly halved the frame.

⚠ The two states differ because at rest `F = I` exactly and the Jacobi SVD converges
in fewer sweeps, so a rest-only measurement understates the sweep. ⚠ These are lib unit
tests and therefore **not behind §2h's contention gate** — corroborating, not
gate-bearing. The gate-bearing figures are the profile shares below.

★ **The two instruments cross-check, and the way they disagree is the check.** Per sweep
(2 per step), the profile gives `6.684/2 = 3.342 ms` sequential and `1.229/2 = 0.615 ms`
parallel — both about `1.6×` ABOVE the microbench (`1.63×` and `1.61×`), because a solve
visits states more deformed than a `γ = 0.15` shear. The absolutes do not transfer; the
**ratio does**, `5.43×` in-solve against `5.37×` on the bench. An offset that is equal on
both arms is what a state-dependence looks like — a discrepancy on only one arm would
mean the timers.

Verdict-identical, and that is a property of the reduction rather than of the
traversal: violators are collected with their ids and reduced by `min_by_key`, so
first-violator-wins survives an unspecified visit order. The cost is paid only on
the failing path — no early exit, one `format!` per violator.

#### ⛔ …but below ~500 tets it is a REGRESSION, and that needed a threshold

The `5.37×` above is one mesh size. Swept, entering rayon costs a fixed ~`60 µs`
whatever the work, so on a small mesh the parallel sweep is **slower than the serial
one it replaced**:

| tets | seq | par | speedup |
|---:|---:|---:|---:|
| 12 | 0.0068 ms | 0.0578 ms | **0.12×** |
| 192 | 0.0909 | 0.1666 | **0.55×** |
| 432 | 0.1630 | 0.1599 | 1.02× |
| 1 200 | 0.2806 | 0.1608 | 1.75× |
| 9 216 | 2.0239 | 0.3696 | 5.48× |

Break-even is ~`430` tets deformed, ~`680` at rest (cheaper per element, so it needs
more of them to cover the same overhead). The skeleton solver is **one** tet, and the
unit fixtures are in the hundreds — so as first written this change made the gate up to
`8×` slower on exactly the meshes most of the suite uses.

`sweep_validity` now dispatches on `PARALLEL_SWEEP_MIN_TETS = 1024`, set above both
break-evens so the parallel path is taken only where it clearly pays rather than at the
crossover. Guarded by a `const` assertion beside it — a BUILD failure, not a test, so a
filtered run cannot skip it — if the constant is lowered back into the measured
regression band.

⚠ **Neither published figure moves**: §2i's fixture is 9 216 tets and §2h's probe is
3 840, both far above the threshold — confirmed by re-running both after the fix.

★ This was caught by sweeping a knob the first measurement held fixed. The speedup was
real and reproducible at the size it was measured at; what was wrong was the unstated
generalisation to all sizes.

#### The other two knobs, swept for the same reason

**Threads.** The mechanism above is not a fixed rayon-entry cost — it is a *wakeup*
cost, and it scales with the pool. Speedup on the sheared state:

| tets | 1 thr | 2 thr | 4 thr | 8 thr | 12 thr |
|---:|---:|---:|---:|---:|---:|
| 12 | 0.56× | 0.61× | 0.24× | 0.16× | **0.10×** |
| 192 | 0.99× | 1.66× | 1.06× | 1.12× | **0.39×** |
| 432 | 0.95× | 1.79× | 1.73× | 1.68× | **0.81×** |
| 768 | 0.95× | 1.80× | 2.14× | 2.43× | 1.41× |
| 1 536 | 0.96× | 1.84× | 2.60× | **3.27×** | 2.31× |
| 9 216 | 0.98× | 1.89× | 3.31× | **5.18×** | 5.11× |

Two things fall out, and only the first was anticipated:

1. **The size threshold's worst case is the DEFAULT pool.** At 12 threads break-even is
   between 432 and 768; at 2–8 threads it is at or below 192. `1024` clears the worst
   column, so it is safe for every pool measured.
2. ⚠ **In a single-thread pool the parallel path cannot win at any size** — `0.94–0.99×`
   the whole ladder, a small standing loss no size threshold removes. The dispatch now
   also requires `rayon::current_num_threads() > 1`.

⚠⚠ **A lead, not acted on: 12 threads is worse than 8 at every size measured.** The box
is 8 P + 4 E, and waking the four E-cores costs more than they return — `1.41×` against
`2.43×` at 768 tets, `2.31×` against `3.27×` at 1 536. At the two published fixtures the
gap is small (`5.11×` vs `5.18×` at 9 216), so nothing here is affected, but a P-core-only
pool looks worth `40–70 %` on mid-sized meshes. Not taken: the global pool is shared with
faer's factorization, so it is a change to the whole solver's threading, not to this
sweep, and it wants its own measurement.

**Element type.** Tet10's per-element certificate brackets a cubic over the whole
element and is far more expensive than Tet4's, so break-even should move DOWN. Stated as
an expectation and then measured, because "a more expensive element can only help" is
the same kind of inference that put the regression here to begin with:

| tets | Tet4 | Tet10 |
|---:|---:|---:|
| 192 | 0.55× | 1.12× |
| 432 | 1.02× | 1.79× |
| 768 | 1.31× | 2.38× |
| 1 200 | 1.75× | 3.08× |

Break-even ~200 tets, as predicted. The Tet4-derived threshold is therefore
**conservative** on Tet10 rather than wrong — it leaves some speedup unclaimed between
200 and 1 024 tets, which is the right direction for a constant piloted on one element
type.

#### As measured now — `65.97` / `66.04 ms/step`, and the bracket clears

**Two runs**, because the first draft of this section quoted one — for a quantity this
same section warns is run-to-run variable. Both are gate-passing runs on the reference
box, taken either side of the size-threshold fix below.

| phase | run 1 | run 2 | share | ECSW removes it? |
|---|---:|---:|---:|---|
| **`red proj K`** (`ΦᵀKΦ`) | **37.482** | **37.422** | **56.7–56.8 %** | yes |
| `asm tangent` | 23.377 | 23.530 | 35.4–35.6 % | yes |
| `asm force` | 2.365 | 2.364 | 3.6 % | yes |
| `contact` (nested) | 1.888 | 1.967 | 2.9–3.0 % | no |
| **`validity check`** | **1.229** | **1.197** | **1.8–1.9 %** | only if R3's own design works |
| `red proj r` (`Φᵀr`) | 0.961 | 0.962 | 1.5 % | yes |
| `red expand` | 0.456 | 0.462 | 0.7 % | yes |
| `residual form` | 0.038 | 0.037 | 0.1 % | only if R3's own design works |
| `red dense solve` (`r × r`) | 0.019 | 0.022 | 0.0 % | no |

| bound | reducible | **ceiling `1/(1 − f)`** |
|---|---:|---:|
| certain only | 95.0–95.1 % | **20.2–20.5×** |
| plus what R3's own design removes | 96.9–97.0 % | **≳32×** ⚠ |

> ### The bracket is 20.2–20.5× … ≳32×. R3 needs 13.5–15.8×, with a 10× floor.
>
> **It clears on either bound.** The straddle is gone.
>
> ⚠ The upper bound is still **ill-conditioned** and must not be quoted precisely: at
> `f ≈ 0.97` a `0.1 pp` shift moves it by ~2×. The two runs above differ by `0.1 pp`
> and their upper bounds read `34×` and `32×` — the lower is quoted. Two identical runs
> of the earlier configuration likewise gave `37.1×` and `38.9×`. What is solid is that
> it is far above `15.8×`, not its value.

⚠⚠ **`20.2–20.5×` IS TOO TIGHT — four further runs (v2.9) widen the lower bound to
`19.0–20.8×`.** Two runs were enough to stop quoting one, and not enough to bound
the spread. The cause is visible and is the one §2j predicted: the lower bound is
`T / I`, and `I` is only `~3.2 ms` of a `~66 ms` frame, so it is set by the
smallest rows in the table. `validity check` alone ranges `1.197–1.459 ms` across
the six runs (`±10 %`) — the parallel sweep's own scheduling variance — and drags
the ceiling with it.

| runs | `validity check` | ceiling `1/(1−f)` |
|---|---|---|
| the two above (v2.7) | `1.197`, `1.229` | `20.5×`, `20.8×` |
| four more (v2.9) | `1.333`–`1.459` | `19.0×`, `19.4×`, `19.5×`, `19.9×` |

★ The large rows do NOT show this: `red proj K` spans `36.80–37.48` and
`asm tangent` `23.23–23.53` across all six, both inside `1.9 %`. So this is not
session drift in the harness — it is a small, noisy quantity being the
denominator. **No conclusion moves**: `19.0×` still clears `13.5–15.8×`, and under
§2j's restated gate `I ≤ 16.7 ms` the margin is `4.9–5.3×`. Quote **`≥19×`**.
⚠ **That margin is this CONTACT-FREE fixture's.** On IPC 18 750 with contact it
is `1.36–1.47×` (§2k) — same gate, same box, `~4×` less room.

**Coherence check.** ⚠ This is a CROSS-SESSION comparison of absolute times, which §2d
finding 3 says not to trust — licensed here only by its own result: the untouched rows
agree to `0.3–2.1 %`, which is what says the two runs are comparable. The frame fell
`72.07 → 65.97 ms`, a `6.10 ms` saving; the
sweep row fell `6.684 → 1.229 ms`, `5.46 ms` of it. **89 % of the whole-frame
improvement is the line that was changed**, and the two large untouched rows barely
moved: `red proj K` `37.591 → 37.482` (`−0.3 %`) and `asm tangent`
`23.880 → 23.377` (`−2.1 %`), both inside this harness's run-to-run spread. Had
they moved materially, the reading would be about the timers, not the change.

#### ⛔ RETRACTED — the `ReducedValidityDomain` is not a performance prerequisite

The previous revision of this section argued that because the sweep at `9.3 %`
decided the bracket, R3's two halves were inseparable and the validity half had to
go first: *"without it R3's ceiling is `8.3×` and R3 cannot clear its own `10×`
gate."* **Withdrawn.** The sweep's share was a property of running it on one core,
not of the sweep. `ReducedValidityDomain` reverts to what §4c always said it was —
the defence against "smooth and wrong", a correctness feature, sequenced on its own
merits.

⚠ What survives the retraction is the *asymptotic* form of the argument, and it is
weaker than it looks. Under ECSW the reduced work drops to `O(n_selected)` while
the sweep stays `O(n_tets)/P`, so the sweep's share must grow with mesh refinement
and a constant factor only buys headroom. It is not currently possible to say how
much: this bracket is one mesh, and the measurement that would settle it is the
same contact-fixture measurement item 3 below already calls for.

#### `ΦᵀKΦ` is 57 % of the frame — hypothesis, not result

`red proj K` costs more than the assembly it projects, and removing the sweep
raised its share from `52.2 %` to `56.8 %`. `project_tangent` builds an `n × r`
dense `Y` as a **`Vec<Vec<f64>>`** and contracts it in a naive `O(n·r²)` triple
loop, single-threaded — structurally what R0's per-iteration `BTreeMap` was, and
what the validity sweep just turned out to be.

⚠ **Stated as the next measurement, not a finding** — this document's attributions
are wrong about a third of the time (§2d finding 3). It is worth testing first
because it pays twice: it improves R1's `~2×` directly, and it moves R3's ceiling
and R3's requirement in the same direction, so the net sign is not obvious from
the armchair.

Also visible: **`contact` costs 2.9 % of a frame with `NullContact`** — pure
marshalling and broad-phase on a scene with no contact.

#### The full-order path got almost nothing, and that is the point

The same change is worth `0.4–0.6 %` of a full-order frame — `1.008 ms` of
`155.60` at IPC 5 202, `3.134 ms` of `890.22` at IPC 18 750 — because there the
sparse factorization is `70–77 %` and squeezes everything else. So:

- **§2a's gap is unchanged.** A `0.4 %` line cannot move a `53.2×` gap, and it
  should not be restated from a single run: `r0_ab`'s `IPC_18750` p50 has read
  `824`, `843` and `888 ms` on this tree across runs, a `±4 %` spread that swamps
  the effect.
- **§2d needs no revision.** Re-measured on this tree its post-R0 rows land within
  `1–3 pp` of what it publishes, which is its own run-to-run spread.
- The `7.6 % → 9.6 %` full-vs-reduced comparison recorded at #822 is now
  `1.3 % → 1.9 %` on the same fixture — the same `~1.3×` inflation, `5.5×` cheaper
  on both sides.

⚠ The corollary is that **this was a reduced-path optimisation**, and the reduced
path is the one that has never been run with contact.

#### ⚠⚠ Ceiling and requirement are still on DIFFERENT FIXTURES

`13.5–15.8×` is IPC 18 750 **with contact**; this is R1.1's **contact-free**
5 202-DOF cantilever. The ladder defines R1 as *"linear subspace, no contact"*, so
**the reduced path has never been run on the fixture its requirement is stated
for**, and its phase mix there is unknown — contact would add irreducible cost and
push the ceiling DOWN. Comparing the two directly is the error of §2d finding 4
wearing different clothes. The solid claim is the narrow one: *at R1.1's own
operating point, R3's ceiling brackets `20.2–20.5×` to `≳32×`.*

#### ⚠ The reference box had to be re-baselined, and this will recur

§2h's probe is a **full-order** solve of `cantilever 40×4`, so it measures the SUT.
This change took its quiet median `24.47 → 22.75 ms` and left legitimate readings
`3.8 %` from the `21.0 ms` floor — which is precisely what that floor is for. All four
constants were re-piloted (§2h), and every future full-order optimisation will force
the same. ⚠ **`project_tangent`, the next item below, is NOT one** — it is reduced-path
and the probe never runs the reduced path. ⚠ But the probe is built with `NullContact`,
so the `2.9 %` marshalling cost noted above is *in* it: acting on "free to remove" would
force a re-pilot.

#### What this does to the ladder

The blocking item is discharged. **R3's viability no longer depends on the
validity-domain gate**, so the two halves of R3 are separable again and the
hyper-reduction half can be sized on its own.

Two of the three measurements still stand, in order:

1. **`project_tangent` as an implementation question** — `56.8 %` of a reduced
   frame, the R0-shaped target, and now the largest single line by a wide margin.
   ⚠ **It is a WALL-CLOCK item, not an R3 item** — §2j shows that speeding up any
   `Reducible::Yes` phase moves R3's ceiling and requirement by the same factor and
   leaves its margin exactly unchanged. Worth doing first anyway; just not because
   R3 is waiting on it.
2. **The reduced path on a CONTACT fixture** — the requirement lives there, R1 has
   never been validated there, and it is also what would settle how far the
   asymptotic form of the retracted argument reaches. ★ **This is the one R3 is
   actually waiting on**: the margin is `B / I`, and `I` has never been measured
   where the requirement lives (§2j).

`ReducedValidityDomain` stays in R3's scope as §4c's correctness feature, no
longer as a gate on whether R3 is worth building.

### 2j. `project_tangent` — PRE-REGISTRATION, written before the measurement

§2i names `ΦᵀKΦ` the next item and flags that *"it moves R3's ceiling and R3's
requirement in the same direction, so the net sign is not obvious from the
armchair."* It is obvious from the algebra, and the algebra needed writing down
before the run rather than after.

#### The net sign is not ambiguous — it is exactly zero

Write a reduced frame as `T = I + Red`, irreducible plus reducible, against a
frame budget `B`. ECSW's ceiling is the limit as the sampled subset goes to zero
cost, and the requirement is the gap to budget:

```text
C = T / I          R = T / B          C / R = B / I
```

**R3 clears iff `C ≥ R`, i.e. iff `I ≤ B`.** The margin depends on the budget and
the *irreducible* time and on nothing else. Speeding up any `Reducible::Yes` phase
shrinks `T`, which lowers `C` and `R` by the same factor and leaves `C / R` fixed.
Against §2i's **run 2** (run 1 gives `5.26×`, equally flat — the invariance is
algebraic, so the spread between runs is in `I`, not in the property):

| speedup `S` on `red proj K` | frame | ceiling `C` | requirement `R` | margin `C/R` |
|---:|---:|---:|---:|---:|
| 1× | 66.00 ms | 20.48× | 3.95× | **5.18×** |
| 4× | 37.93 ms | 11.77× | 2.27× | **5.18×** |
| 10.2× | 32.23 ms | 10.00× | 1.93× | **5.18×** |
| ∞ | 28.57 ms | 8.87× | 1.71× | **5.18×** |

⚠⚠ **`R = 3.95×` is this fixture's own requirement and is NOT §2f's
`13.5–15.8×`.** That one is IPC 18 750 **with contact and with the predictor
applied**; this is R1.1's contact-free 5 202-DOF cantilever run under
`InitialGuess::PreviousState`, i.e. no predictor. Same definition — the factor R3
must supply from the reduced baseline to reach a 16.7 ms frame — on a different
fixture. Reading one against the other is §2d finding 4 again; the invariance
itself is what transfers, the numbers are not.

> ### Pre-registered rule 1 — no run on `project_tangent` may conclude anything about whether R3 clears.
>
> It cannot. The quantity that decides that is `I`, and `project_tangent` is not
> in it.

★ The invariance is exact in ECSW's ideal limit. Real ECSW leaves `ε · Red`
behind, so feasibility is `I + ε · Red ≤ B` and shrinking `Red` makes R3 *easier*.
The invariance is therefore the conservative reading, not an optimistic one.

#### ⚠ What is NOT invariant — a gate written as a ratio

`C` itself falls, and §7's kill floor is stated as a ratio: *"R3's measured
speedup over §2a's (post-R0) baseline is under ~10×"*. That baseline is one every
rung on this ladder is chartered to improve.

| speedup on `ΦᵀKΦ` | frame (run 2) | ceiling, run 1 → run 2 | against a fixed `10×` |
|---:|---:|---:|---|
| 1× | 66.00 ms | 20.77× → 20.48× | clears |
| 4× | 37.93 ms | 11.91× → 11.77× | clears |
| 8× | 33.25 ms | 10.44× → 10.32× | **all but touching it** |
| **10.2–11.4×** | ~32 ms | **10.00×** | **exactly at it** |
| ∞ | 28.57 ms | 8.87× → 8.87× | fails |

⚠ **The crossover is quoted as a RANGE because it moves between runs** —
`11.38×` on run 1, `10.24×` on run 2. It is set by `I`, the smallest and
therefore noisiest column in §2i's table (`3.174` vs `3.223 ms`), so a single
figure here would be the same defect §2i's own headline was corrected for.

That range is well inside reach. It does not need the current loop to be
pathological: **8 cores and 2-wide `f64` SIMD are ~16× of headroom over a
single-threaded scalar triple loop**, before any layout change. **A gate that
gets harder every time an earlier rung succeeds is measuring the wrong thing.**

> ### ⇒ RESTATED — and restated BEFORE the measurement, which is the last moment it is legitimate
>
> §7 states its kill conditions *"so they cannot be renegotiated after the fact"*.
> Changing one after this measurement would be exactly that. Changing it now, with
> the reason on the record and the outcome still unknown, is not.
>
> | | was | is |
> |---|---|---|
> | R3's feasibility gate | speedup `≥ 10×` over the post-R0 baseline | **`I ≤ 16.7 ms`**, measured on the fixture the REQUIREMENT is stated for |
> | the `10×` | a kill floor | a **complexity heuristic** — re-derived against the baseline of the day, never carried |
>
> The two agree today (`I = 3.17–3.22 ms` over the two runs, and `20.5–20.8× ≥ 10×`).
> They diverge the moment any reducible phase is optimised, which is precisely what
> the next item is.

⚠ **Corollary for how results get quoted.** A re-quoted ceiling must be reported
beside its own re-measured requirement. Reading a post-fix `11.8×` against the
standing `13.5–15.8×` would say "R3 is now blocked" about a change that did not
move R3 at all — the requirement fell with it. Any ceiling quoted alone after this
point is a defect.

#### The prize, bounded before it is measured

| | frame | vs the 16.7 ms budget |
|---|---:|---:|
| today | 66.0 ms | 3.95× over |
| `ΦᵀKΦ` at `8×` | 33.1–33.3 ms | 1.98–1.99× over |
| `ΦᵀKΦ` at `∞` | 28.5–28.6 ms | **1.71× over** |

**`project_tangent` alone cannot reach 60 Hz on its own fixture**, and its
whole-frame ceiling is `2.31×` — a `10×` on a `56.8 %` line is `1.99×` of a
frame. At the floor, `asm tangent` is `82 %` of what is left and becomes the next
item by construction. Stated here so a good result cannot be oversold later.

#### ★ The corollary reorders §2i's list

Since the margin is `B / I`, only the irreducible mass can move R3's verdict — and
`I` is `3.17–3.22 ms` of a `66 ms` frame (rows are run 2):

| irreducible line | ms | share of `I` |
|---|---:|---:|
| `contact` marshalling — **on `NullContact`** | 1.967 | **61.0 %** |
| `validity check` | 1.197 | 37.1 % |
| `residual form` + `red dense solve` | 0.059 | 1.8 % |

⚠ **These rows do not sum to `I`, and that is not an error in the table.** The
harness counts the *unaccounted* remainder — instrumented time short of wall,
`~0.1 %` — as irreducible in both bounds, deliberately, so a missing cost centre
pushes the ceiling DOWN rather than up. `I` is therefore `T · (1 − f)` and runs a
few hundredths of a millisecond above the visible rows. Recomputing `I` by adding
these up understates it by `1–2 %` (measured `1.1 / 1.2 / 2.1 %` over the three
post-fix runs) and overstates the ceiling by the same; quote the harness's own
figure.

Removing the `NullContact` marshalling would take the margin `5.2× → 13.3×`, a
`2.57×` improvement, and it is the only line in the table that moves the verdict
at all. `red proj K`, at `56.8 %` of the frame, moves it by zero.

⇒ **`project_tangent` is a WALL-CLOCK item, not an R3 item.** It is still the
right thing to do first — largest line by a wide margin, R0-shaped, and a real
credit to R1 today — but §2i's ordering implies R3 is waiting on it and R3 is not.
**The measurement R3 waits on is item 2, the contact fixture**, because that is
the only place `I` is unknown.

⚠ And the `1.967 ms` above is marshalling on a scene with **no contact**. On a
real contact fixture `I` is larger and genuinely irreducible. Which is the point:
`I` has never been measured where the requirement lives.

✅ **Measured in v2.13 — see §2k.** `I` is `11.3–12.3 ms` on IPC 18 750 under the
settled predictor, a margin of `1.36–1.47×` against this fixture's `5.60×`. The
prediction that contact would dominate `I` held (`69 %` of it); the prediction
that the iteration count would carry over did not.

#### The knob matrix, and what each knob is for

⚠ **Knob 0 gates the rest — no fix is chosen until it reads.** §2d finding 3 puts
this document's attributions wrong about a third of the time, and `project_tangent`
is two loops with different asymptotics and different fixes.

| # | knob | values | why it is in the matrix |
|---|---|---|---|
| **0** | **which half** | `Y = AΦ` (`O(nnz·r)`) vs `ΦᵀY` (`O(n·r²)`), split by a temporary sub-timer — **and the achieved flop rate of each** | they need different fixes; "`ΦᵀKΦ` is slow" does not say which. ★ The flop rate separates the two candidate diagnoses: near scalar peak ⇒ this is honest serial arithmetic and the fix is threads + SIMD; far below ⇒ it is the `Vec<Vec<f64>>` layout, and the fix is a flat buffer first |
| 1 | `r` | 20 / **40** / 80 | phase A is `O(r)`, phase B is `O(r²)`, so the A:B ratio *moves with `r`* — a fix tuned at R1.1's `r = 40` can be the wrong fix at 80 |
| 2 | `n` — **free DOF, not tets** | R1.1's `5 202`, plus one smaller and one larger beam | #823's lesson verbatim: a rayon path measured at one size read `0.12×` at another. ⚠ **This is the expensive knob** — see the costing below |
| 3 | threads | 1 / 4 / 8 / 12 | §2i found 12 worse than 8 at every size on this 8P+4E box; the cost is a wakeup cost |
| 4 | element type | Tet4 / Tet10 | a denser pattern shifts `nnz` and therefore the A:B ratio. **Recorded, not gated** |

**⚠ Cost, stated before the matrix is run.** The knobs are not equally priced, and
the difference is a POD basis:

| knob | what it forces | cost |
|---|---|---|
| 0, 3 | nothing — same fixture, same basis | one harness run each |
| 1 (`r`) | re-truncate the basis; snapshots are reusable | cheap; `r` is a truncation of an existing SVD |
| **2 (`n`)** | **a whole new snapshot set** — 12 trajectories × 5 full-order steps, then an SVD, per size | **the dominant cost of the matrix** |
| 4 (Tet10) | new mesh *and* new snapshot set | as knob 2, and it is the one marked "recorded, not gated" |

⇒ **Run knob 0 first on the existing fixture.** It is one run, it costs nothing
extra, and it decides which fix is even a candidate — so the `n` sweep is not paid
twice.

✅ **Smoked: `9.2 s` wall for the whole harness**, snapshot generation and SVD
included (`5.7 s` of it inside the test). ⚠ **The paragraph above overstated the
cost and is corrected here rather than quietly**: the ordering of the knobs by
price is right, but in absolute terms *every* cell of this matrix is seconds, so
"the dominant cost of the matrix" means seconds rather than minutes and nothing in
it needs rationing. Run the whole thing.

**Pre-registered rules, in force for every run below:**

1. **No conclusion about R3's clearance** (rule 1 above).
2. **The credit is an interleaved wall-time A/B**, box-gated per §2h. Never share
   differencing — §2d finding 3, wrong by `37 %`.
3. **The headline is the whole-frame factor**, quoted against its `2.31×` cap, not
   the phase-local speedup. A `10×` on a `56.8 %` line is `1.99×` of a frame.
4. **A rayon path ships only with a piloted size threshold and a `const`
   assertion**, per #823's regression.
5. **Any re-quoted ceiling carries its own re-measured requirement.**
6. **State which half the fix addresses, and measure the other.** A layout change
   that helps one loop and pessimises the other is a wash that reads as a win if
   only the frame total is watched.

#### ✅ Knob 0, MEASURED — the split is `1.85 : 1` and BOTH halves are large

Three runs on the reference box, `r = 40`, `n = 5 202`, `4.00` Newton iterations
per step. The positive control — the two children bracket every statement of the
parent bar `basis.modes()`, so they must nearly exhaust it — read **`99.5 %` on
all three**, so the timers are where their names say and the ratio below means
something.

| | run a | run b | run c | share of frame |
|---|---:|---:|---:|---:|
| `Y = AΦ` — gather, `O(nnz·r)` | 24.191 | 24.157 | 23.865 | **36.4–36.7 %** |
| `Φᵀ Y` — contract, `O(n·r²)` | 13.010 | 13.162 | 12.882 | **19.7–19.9 %** |
| **A : B** | 1.86 | 1.84 | 1.85 | — |

**Neither half can be ignored, and that was the point of the knob.** Taking one to
zero cost and leaving the other alone caps the whole-frame win hard:

| | frame | whole-frame |
|---|---:|---:|
| gather → 0, contract untouched | 41.82 ms | **1.58×** |
| contract → 0, gather untouched | 52.87 ms | **1.25×** |
| both → 0 | 28.80 ms | 2.29× |
| both at a realistic `8×` | 33.44 ms | 1.97× |

⇒ **Pre-registered rule 6 binds here**: a fix that addresses one loop must report
what it did to the other, because "`ΦᵀKΦ` got faster" can mean `1.25×` or `2.29×`
of a frame.

#### ★ The contract is LATENCY-bound, which names the fix

The contract's flop count needs no new accessor: `calls · r(r+1)/2 · n · 2` =
**`34.13` Mflop/step**, and at `13.02 ms` that is **`2.62` GFLOP/s** — *one FMA
per `3.36` cycles* at `4.4 GHz`.

That is not a bandwidth wall and it is nowhere near a throughput wall. It is the
**serial dependency chain**: `(0..n).map(|p| modes[i][p] * y[p][j]).sum()` is one
accumulator threaded through `n` FMAs of ~4-cycle latency, so the machine is
idling ~3 cycles in 4 no matter how much SIMD or how many cores it has. §2j's
knob-0 rule said *"near scalar peak ⇒ threads + SIMD; far below ⇒ layout"* — the
answer is the second, and more specifically **restructuring before parallelism**.

⇒ **One change addresses both halves**, which is the outcome rule 6 was written to
check for:

- store the basis **transposed** (`Φᵀ` as a flat `n × r`) and `Y` as a flat
  `n × r` instead of `Vec<Vec<f64>>`;
- the **gather**'s inner `k` loop then walks `y[row·r ..]` and `phi_t[col·r ..]`
  contiguously — two AXPYs instead of `r` pointer-chases per triplet;
- the **contract** becomes `n` rank-1 updates into an `r × r` accumulator (`40 ×
  40 × 8 B = 12.8 KiB`, L1-resident) — a `syrk`. Each of the `n` steps writes
  `r(r+1)/2` *independent* accumulators, so the dependency chain that costs `3.36`
  cycles/FMA today is gone by construction.

⚠ The gather's own flop rate is NOT computed here: it needs the pattern `nnz` and
there is no public accessor for it. Adding one purely for a diagnostic was judged
scope creep. The gather's share is measured; only its efficiency is unquantified.

#### ✅ BUILT and MEASURED — `1.84×` of a reduced frame, byte-identically

The change is the one knob 0 named: `Φᵀ` cached as a flat `n × r` row-major buffer
alongside the existing column-major copy, `Y` flat `n × r` to match, the gather's
inner loop two contiguous AXPYs, and the contract `n` rank-1 updates into an
L1-resident `r × r` accumulator. No parallelism, no new dependency. Costs `1.59 MiB` for the second copy of `Φ` — **per basis, not per env**, since the solver borrows it, so §2b's memory ceiling is untouched.

| | before | after | factor |
|---|---:|---:|---:|
| **reduced frame** | 65.89 ms | **35.72 ms** | **1.84×** |

⚠ Those are POINT values, and this harness's own spread is `~2.1 %` (noted below
against the control). A later run of the same post-change configuration read
`34.86 ms`, which would put the ratio at `1.89×`. **Read the effect as `1.84×`
with a couple of percent either way**, not to three digits — it is far larger
than the spread, which is why the conclusion is safe and the digits are not.
| `red proj K` | 37.09 | 6.62 | 5.61× |
| ↳ `Y = AΦ` | 24.07 | 3.95 | 6.09× |
| ↳ `Φᵀ Y` | 13.02 | 2.67 | 4.88× |
| `asm tangent` (untouched — the control) | 23.24 | 23.73 | 0.98× |

Three runs each side, split control `100.0 %` on all three after. ⚠ The two arms
are block-ordered, not interleaved, which §2d finding 3 warns about — licensed
here by the control row: `asm tangent` is untouched by the change and moved
`2.1 %`, inside this harness's own spread, against a `1.84×` effect.

**Against the cap §2j fixed in advance: `1.84×` of an available `2.31×`.** Rule 3
satisfied — quoted whole-frame, not as the `5.61×` on the phase.

**Correctness is byte-identity, not a tolerance.** Both loops still sum their `p`
terms low-to-high, so the result is exact to the bit; the gate compares every
upper-triangle entry with `to_bits()` against the pre-change form, written from
the *column-major* `modes()` so a wrong transpose cannot satisfy both. Two
negative controls, both fire: reversing the contract's accumulation order changes
the 13th digit and is caught (**a tolerance test would have passed it**), and
dropping mode 0 from the flat transpose alone is caught.

⚠ That gate runs at `r = 5`, `n = 150`. The production-scale evidence is separate
and was free: the `r = 10 … 104` sweep at `n = 5 202` prints, per rank, a
displacement projection and retained energy plus four columns for each of three
objectives — **70 numeric fields over 20 lines, every one identical across the
change** to all printed digits. That is consistent with byte-identity at production scale rather
than proof of it; the `to_bits()` gate is what proves it, and it proves it small.
⚠ It took two review rounds to make this sentence true. The first version was
checked by a diff that stripped everything after `trajectory`, comparing **2 of
the 70**; the second reported **75**, having counted the `1.3` in each `R1.3`
header five times. The conclusion held throughout. Neither the coverage nor the
count did.

#### ★★ The invariant held, and the instrument printed the misreading anyway

§2j predicted that this could not move R3's margin. Measured:

| (three runs each side) | before | after |
|---|---:|---:|
| R3's ceiling | 19.4–19.9× | **11.2–12.2×** |
| this fixture's requirement `T/B` | 3.92–3.97× | **2.09–2.15×** |
| **margin `B/I`** | **4.91–5.02×** | **5.28–5.66×** |

⚠ **A fourth run of the post-change arm, taken in a later session on the same
tree, read `2.09×` — outside the three-run `2.12–2.15×` this table first
carried** (ceiling `11.7×` and margin `5.60×` both landed inside). The
requirement row is widened to `2.09–2.15×`; the conclusion is untouched, since
what §2j claims is that the MARGIN did not move and it did not. ⇒ Same lesson
§2k's audit reached independently: **three runs give a range that a fourth run
escapes.** Read every range in §2i–§2l as min–max observed, not as a bound.

The ceiling fell `~1.7×`, the requirement fell `~1.85×`, and **the margin did not
fall.** ⚠ It read slightly HIGHER afterwards, and that is not a benefit of the
change: `I` is mostly `contact` plus the validity sweep, and the sweep read
`1.33–1.43 ms` in the before runs against `1.11 ms` after — inside the `±10 %`
session spread §2i records for that row. Read the two bands as "unmoved", not as
`+8 %`. ⚠ **And `tests/reduced_phase_shares.rs` printed `"⚠ BRACKET STRADDLES the
requirement — the bound decides, so close it"`** — because it compared the new
`11.2×` against a hardcoded `needs 13.5–15.8×` that is both stale and a different
fixture. Exactly the defect §2j's corollary names. **The harness has been fixed**
to report the gate that decides — irreducible ms/step against the 16.7 ms budget,
and the margin — and to print this fixture's own `T/B` beside a warning not to
read the ceiling against §2f's number.

#### ✅ Knob 1 came free, and it is where the result is largest

`tests/reduced_gradient.rs::adjoint_gap_across_basis_sizes` already sweeps
`r = 10 … 104`. It **failed on the first run after the change**, asserting
`growth > 1.5` with the message *"if `ΦᵀAΦ` stopped dominating, §14's break-even
argument needs re-measuring"*. It had. Least squares over the sweep:

| | `O(r²)` /mode² | `O(r)` /mode | constant |
|---|---:|---:|---:|
| before | 0.0407 | 1.87 | 117.4 ms |
| after | **0.0025** | **0.69** | **112.3 ms** |
| | **16× down** | **2.7× down** | **unmoved — the control** |

⇒ **the break-even against the oracle moves `r ≈ 61` → `r ≈ 219`**, and at the R1
plan's rank ceiling of 104 the reduced path goes from `0.51×` the oracle to
`1.83×`. ⚠⚠ **`r ≈ 219` is an EXTRAPOLATION to twice the swept range and must not
be quoted as a measurement** — the fit is good over `10 … 104` (max residual
`1.2 ms`) but nothing constrains it past that, and an `O(r³)` term the sweep
cannot see would pull it in hard. What is MEASURED is the narrow claim: **every
rank in the swept range is now faster than the oracle**, where two of five were
slower before. `r ≈ 61` is by contrast an interpolation, and safe. Plan §14 finding 2's cost premise is overturned; **its conclusion
survives on finding 1 alone** — the accuracy columns are bit-identical and `Σx*`
gradient error is still `0.704` at `r = 104`, so rank still does not fix the
adjoint. The guard's bound was **re-measured, not widened**: it now asserts
`1.25–2.0` two-sided, piloted at `1.47/1.49/1.49`, so a regression of the layout
trips it from the other side.

★ This is the knob-1 prediction paying off in the direction the pre-registration
did not guess: it warned *"a fix tuned at `r = 40` can be the wrong fix at 80"*.
It is a **better** fix at 80, because the term it removes is the one that grows.

#### What is left

- **`asm tangent` is now `66.9 %` of a reduced frame** — the next item, exactly as
  the prize table predicted it would be.
- ⚠ **Knobs 2, 3 and 4 are NOT swept.** `n` (one fixture only), threads (the fix
  uses none — this `1.84×` is single-threaded, so the thread knob is unspent
  headroom rather than a risk) and Tet10. Recorded as open, not as done.

#### ✅ MUTATION TESTING — three read-rounds stopped converging, so the instrument changed

`cargo mutants`, scoped to the two files this work touched, tests run with
`--lib`. Read rounds 1–3 found 7 / 3 / 3 defects with severity falling from *"an
instrument published without being run"* to *"a tally said 75 where the data said
70"*, which is the point at which re-reading mostly generates new text to re-read.

| run | mutants | caught | survived | unviable |
|---|---:|---:|---:|---:|
| `project_tangent` | 71 | **61** | **0** | 10 |
| `profile.rs`, before | 36 | 0 | 29 | 7 |
| `profile.rs`, after, feature OFF | 36 | 22 | 7 | 7 |
| `profile.rs`, after, feature ON | 36 | **26** | **3** | 7 |

**`project_tangent` scores 100 % of what is killable.** The 10 unviable are all
whole-function return swaps (`DMatrix::new()`, `from_iter([0.0])`) that do not
typecheck.

★ **A perfect score is when to ask what did the killing.** `reduced/tests.rs` is
the only lib test that reaches `project_tangent`, so the result was attributed
rather than assumed: apply `row != col → row == col` and the suite fails; mark the
byte-identity test `#[ignore]` and **the same mutant passes 328 green tests**. One
test is doing all of it. That is the answer the pre-registration wanted — the gate
is load-bearing, not decorative — and also a warning, because `#[ignore]`ing it
returns `project_tangent` to zero lib-level coverage without turning anything red.

#### ⚠ `profile.rs` measured 0 of 29 — the instrument had no unit tests at all

Every number in §2d, §2i and §2j comes out of this module, and nothing that runs
by default exercised it. Among the survivors:

- **`delete ! in total_nanos`** — turns `filter(|p| !p.is_nested())` into its
  inverse, so the denominator becomes the nested slots alone and **every share in
  this document silently changes**;
- both constant replacements of **`Phase::is_nested`**, the predicate §2j made
  load-bearing;
- **`/` → `*` in `Phases::share`**, and `==` → `!=` on its divide-by-zero guard.

Eight unit tests now cover the slot arithmetic: nested slots excluded from
`total_nanos` (asserted two-sided, so an inverted filter lands somewhere
detectable), disjoint shares summing to `1.0`, the zero-total guard, per-slot
read-back, `is_nested` being exactly the three children, and the ECSW class the
harness's nesting correction keys on. **0 → 22 of 29.**

#### ★★ A cfg-gated module needs ONE RUN PER CONFIGURATION

The `profile.rs` survivors were first written up as *"equivalent mutants, and the
enabled arm can only be reached through box-gated harnesses, so `-j 1` with the
feature on"*. **That was wrong on both halves, and re-running it is how it was
found.** `-j 1` was never the issue, and the enabled arm did not need a harness.

`imp` exists twice, once per `#[cfg(feature = "phase-timing")]` arm. A mutation of
the arm that a given build does **not** compile changes nothing, cannot fail any
test, and is scored **MISSED** — indistinguishable in the report from a real
coverage hole. So the two runs see opposite halves:

| survivor | feature OFF | feature ON |
|---|---|---|
| `Timer::start`, `Drop`, `reset`, `snapshot` (timed arm) | 4 inert — not compiled | **caught** (`start` becomes *unviable*: the timed `Timer` has no `Default`) |
| `Timer::start`, `snapshot` (no-op arm) | 1 equivalent — the body *is* `default()` | 1 inert — not compiled |
| the `const` guard's `while i < N_SLOTS` | 2 | 2 |

⇒ **every live mutant is caught in the configuration where it is live**, and what
is left is the two that mutate the guard's own loop bound into vacuity. A
compile-time guard is not a test and no test can kill those; accepted, because the
protection that matters is `E0081`, a language rule that cannot be mutated away.

⚠ **A single mutation run over cfg-gated code silently overstates the gap.** Of
the original `0 of 29`, five were never live in the build being measured. Report
which configuration a mutation score belongs to, or it is not a score.

#### What closed the timed arm — and why a unit test could not

`Timer` records on drop into process-global `AtomicU64` statics, not
thread-locals, which is why every harness reading them needs `--test-threads=1`.
A `#[cfg(test)] mod` would have shared those counters with 337 lib tests, several
of which build `Timer`s once the feature is on, so any exact-count assertion would
race. **`tests/profile_contract.rs` is a separate integration binary**: its own
process, its own statics, one test function. It asserts the recording contract
with the feature on (drop books exactly one entry, into its own slot, non-zero;
`reset` clears every slot) and the **zero-cost contract with it off** — that the
no-op arm records nothing, which `profile`'s module header claims and nothing
asserted. Negative-controlled by hand against all three: gutting `Drop`, `reset`
or `snapshot` each fails it.

★ The `#[ignore]`d harnesses do check related properties — `phase_shares.rs`
asserts no slot reads zero after a real solve, §2j's split control asserts the two
children exhaust their parent — but they are box-gated and CI never runs them.
This one runs by default, in whichever configuration is being built.

### 2k. The contact fixture — `I` measured where the requirement lives (v2.13)

§2j closed by naming the one measurement R3 was actually waiting on: the margin
is `B / I`, and `I` had only ever been measured on a contact-free cantilever
where **61 % of it was `contact` marshalling on a `NullContact` scene**. The
largest term in the deciding quantity was a placeholder for the thing that was
absent. Measured now, on IPC 18 750 — the fixture §2f states the requirement for.

**The producer first.** The reduced path had never been run with contact at all,
and timing a solver that produces the wrong answer is not a measurement. The
question was not idle: `IpcRigidContact` clamps its barrier
(`sd.max(d̂ · 1e-6)`), so penetration is finite-energy rather than infinite, and
the reduced line search backtracks on `‖Φᵀr‖`, which cannot see a barrier spike
orthogonal to the subspace. **That mechanism is real and has no consequence
here.** At `r = 10` the solve converges on `‖Φᵀr‖ < 1e-10` while `‖r_free‖` is
`1.49e-4` — six orders hidden by the projection — and nothing penetrates, at any
rank down to `r = 2`, where the displacement field is `48 %` wrong.

★ The equilibrium gap converges in `r` far faster than the field around it: the
two are within `1.7×` at `r = 2`, but the gap is `~2 700×` better determined at
`r = 10` and `~54 000×` at `r = 20`. The barrier pins the contact state much
harder than the subspace can disturb it.

**Then the timing.** Three arms over one 71-step ramp, timing the last 8 steps —
the deepest, where the patch is largest.

⚠ A fourth review round corrected the reason given for that window. It said
early frames have "no active pairs"; they do. `z_start` is `1.2·d̂` above the
surface and the increment is `0.3·d̂`, so step 0 already sits at `0.90·d̂` and the
barrier is live for the entire ramp. The window is right for a different reason:
the PATCH grows (`+0.90 d̂` of clearance at step 0 against `-18 d̂` of
interference at the first timed step), so contact cost rises monotonically and
the trailing window is the conservative choice for `I`. Timing an early window
would understate the deciding quantity — the same error as timing a
contact-free fixture, just smaller. The harness now gates on pose DEPTH, which
discriminates, as well as on activity, which here cannot. Runs on §2h's
reference box:

| arm | ms/step | iters/step | `I` ms | `B/I` |
|---|---:|---:|---:|---:|
| full-order (`PreviousState`) | 812–844 | 6.00 | 667–695 | `0.02–0.03×` |
| reduced, `PreviousState` | 304–315 | 9.00 | 17.1–18.2 | **`0.92–0.98×`** ⛔ |
| reduced, `Inertial` | 181–185 | 5.25 | 11.3–12.3 | **`1.36–1.47×`** ✓ |

⚠⚠ **Every DERIVED figure in this section was recomputed over all nine runs by
a later audit, and several had been quoted from run 1 alone** — where, being
first, they sat at extremes of the eventual range. `asm tangent` read `68.7 %`
against a measured `66.7–68.7 %`; contact's share of `I` read `69 %` against
`65–69 %`; the validity sweep `29 %` against `29–33 %`; `I` without the sweep
`~8.7 ms` against `7.7–8.7`. None of them changed a conclusion, and all four
were the extreme rather than the middle. ⇒ **A derived figure computed once from
the first run and never re-derived as the sample grows is a stale figure that
still looks measured.**

⚠ **These are min–max OBSERVED over nine runs, not bounds.** A ninth run put
full-order `ms/step` at `844` and reduced-`Inertial` at `181` — both outside the
eight-run range this table first carried. What did NOT move across any of the
nine is what the rung turns on: `I`, `B/I`, and the iteration counts, which are
exact integers per step. Read the wall-clock column as context and the last two
as the result.

Nine runs at 18 750, five at 5 202. The same window at 5 202 reads `~3.3×` and
`~4.5×` — see finding 3, the margin is size-dependent. ⚠ The 18 750 ranges are
quoted tight because they decide the rung; the 5 202 ones are rounded because
they are context, and quoting context tight put this document on a treadmill
where every verification run invalidated a figure.

**R3 clears — on `1.4×`, where the contact-free fixture reports `5.60×`.** That
figure was not wrong; it was measured somewhere the requirement does not live,
and it overstated the margin by about `4×`.

Five things it changes:

1. ★★ **The predictor is LOAD-BEARING for R3, not an optimisation.** The same
   rung passes under `Inertial` and fails under `PreviousState`, in all eight
   runs, by `2–8 %`. `I` is dominated by per-iteration cost and the two counts
   are `5.25` against `9.00`. §2f recommended `Inertial` on iteration count
   alone; R3's viability now rests on that recommendation.
2. ★★ **The reduced path's iteration PENALTY grows with size, and the full-order
   path's does not.** Same trailing window at both sizes:

   | free DOF | full | prev | inertial | prev/full | inert/full |
   |---:|---:|---:|---:|---:|---:|
   | 5 202 | 6.00 | 6.88 | 4.12 | `1.15×` | `0.69×` |
   | 18 750 | 6.00 | 9.00 | 5.25 | **`1.50×`** | **`0.88×`** |

   Full-order is flat; reduction costs progressively more iterations as the
   problem grows. This is the extrapolation the producer run flagged and could
   not close, and it is what moved the answer — **not** the contact cost, which
   came in as projected.

   ⚠ **The first evidence for this was a mismatched comparison, and a review
   pass caught it before the branch shipped.** It read 5 202's WHOLE-RAMP average
   (`6.44`) against 18 750's LAST-8-STEPS average (`9.00`) and attributed the
   difference to size — but the last steps are the deepest indentation, where the
   count is highest regardless. Two variables had moved at once. Re-measured in
   one window, the effect survives and is LARGER on the reduced side than it
   looked (`1.15 → 1.50`, not `1.06 → 1.50`). The counts are exact integers per
   step, so the ratios reproduce; only the margins need repeats.
3. ★★ **The margin is not a constant of the ladder — it falls with size.** `I`
   is `3.6–3.8 ms` at 5 202 and `11.3–12.3 ms` at 18 750 against a fixed
   `16.7 ms` budget, so `B/I` goes `~4.5× → 1.36–1.47×`.

   ⚠ **Do not fit one exponent to that**, which a first pass did. `I`'s two real
   terms move in opposite directions and the blend is an artefact of where their
   crossover currently sits:

   | term | 5 202 | 18 750 | scaling | share of `I` |
   |---|---:|---:|---|---|
   *(one run at each size; the ranges above are over three and six)*
   | `contact` | 1.998 | 8.468 | **`n^1.13`** | 53 % → **69 %** |
   | `validity check` | 1.687 | 3.561 | `n^0.58` | 45 % → 29 % |

   Contact is taking over, so the blended `n^0.89` will keep rising toward
   contact's exponent. Summing the terms separately puts `I = 16.7 ms` at
   **`~26 k` free DOF** (the blended fit says `~28 k`) — about `1.4×` the
   requirement's own fixture. Two points per term; a marker for where to measure
   next, not a number. It does mean the `1.4×` above must not be read as "R3
   clears for soft bodies": it clears **at this size**.
4. ★★★ **Which overturns §2d's dismissal of the contact PATH, and locates the
   only available lever on R3's margin.**

   §2d finding 4 already described the mechanism: both timed blocks "marshal
   every vertex into a fresh `Vec<Vec3>` and run the broad-phase scan before any
   pair exists", twice per Newton iteration. It then set it aside —
   *"Neither is on R3's path, and neither is costed here beyond this note."*

   **That dismissal is now wrong, and `C/R = B/I` is why.** Contact is `69 %` of
   `I` at 18 750, `I` is the only quantity that moves a rung's verdict, and the
   two `O(n)` passes are the bulk of contact. So the contact PATH is not merely
   on R3's path — at this size it very nearly IS R3's path. The corollary runs
   the other way for the big lines: `asm tangent`, at `66.7–68.7 %` of the
   reduced FRAME, moves the margin by exactly zero — the same trap §2j's corollary caught
   for `red proj K`.

   Neither pass scales with how many pairs are ACTIVE: `slice_to_vec3s`
   allocates and copies every position per call, and `active_pairs` on a linear
   mesh resolves to `active_vertex_pairs`, which evaluates every primitive at
   every vertex with no broad phase.

   ⚠ **Two scopings, both from a third review round.** (a) The per-vertex walk is
   the LINEAR-mesh path; a Tet10 mesh dispatches to `active_face_pairs`, which
   iterates boundary faces. (b) The `n^1.13` is **not** a per-call scaling —
   contact's `4.24×` growth factors as `1.37×` more calls (Newton `4.12 → 5.25`
   iterations) × `3.08×` more work per call, and per call against `3.97×` more
   vertices that is `n_vert^0.88`, sublinear. The `O(n)` structure is real and in
   the source; the exponent was over-attributed to it. ⚠ No size of win is
   claimed for removing either pass.
5. **`I` is `65–69 %` contact and `29–33 %` the validity sweep at 18 750** (`53 %` /
   `45 %` at 5 202 — see finding 3). The sweep is
   `Reducible::PlannedByR3`, so §4c's `ReducedValidityDomain` is worth about
   `+0.5–0.8×` of margin here (`I` would fall to `7.7–8.7 ms`). That does NOT reverse
   v2.7's retraction — the rung clears either way — but at `1.4×` it is no longer
   irrelevant, which it was at `5.6×`.

**The projection this arc was carrying was sound.** It had been estimating
`1.43 ms` of contact per Newton iteration by dividing §2d's `1.0 %` share by
§2a's `927.7 ms`. Measured directly, on this box, in the same run as the thing it
is compared against: `1.26–1.75 ms`. The wide spread is expected — `contact` is
`~1 %` of a full-order frame, so its per-iteration figure is a small difference
of large numbers.

**Controls.** Coverage reads `100.0 %` on every arm. A cross-path control asserts
`contact` costs the same per CALL whichever way the solve is driven, since it is
the same code on the same mesh — piloted `0.764–1.173×` over sixteen
arm-comparisons across both sizes, banded `0.6–1.7×`. A window check asserts the barrier is active during the timed steps.
The producer check carries a negative control (the undeformed mesh at the final
pose reads `min_sd = −δ`, so a positive gap is the solve's doing) and a piloted
`gap_dev` limit shown failing at `r = 2` and `r = 4` rather than left green for a
reason it never demonstrates.

⚠ **What this does not establish.** The basis is IN-SAMPLE — trained on the
trajectory it is scored against — which isolates "does the algebra work against a
barrier" from "does POD generalise across contact configurations". The second
question, a laterally *moving* contact patch, is the classic advection-like POD
failure, is an R1 question, and remains open. Nothing here runs with friction or
a body load (§2f killed `InertialWithLoad` on contact-plus-load; all of this runs
`gravity_z = 0`). And the full-order `53.2×` gap is untouched.

Harness: `tests/reduced_contact.rs` — an always-on non-penetration gate (2.3 s,
runs in `tests-debug`), an `#[ignore]`d producer ladder over `r ∈ {2,4,10,20,40}`
× both predictors, and this `#[ignore]`d timing instrument (needs
`--features phase-timing` and `--test-threads=1`). The share table and the
`I ≤ B` verdict are shared with §2j's harness through `tests/reduced_report/`, so
the two fixtures cannot drift apart on the arithmetic that decides the rung.

### 2l. The basis does NOT generalise across contact positions (v2.14)

§2k timed the reduced path with contact. It could not say whether the path is
*right* off its training data, because every reduced measurement in this document
— §2i, §2j, §2k — uses an **in-sample basis**, fitted to the trajectory it is then
scored against. That isolated "does the algebra survive a barrier" from "does the
basis generalise", and it left the property R3 depends on untested.

Measured: one basis trained on five lateral indenter offsets spanning `±1a`
(355 snapshots, 5 202 free DOF), scored at four positions against **each
position's own full-order oracle**. Rel-L2 free-DOF displacement:

| position | `r=20` | `r=40` | `r=80` | `r=142` |
|---|---:|---:|---:|---:|
| IN-SAMPLE `+0.00a` | 2.4e-2 | 4.7e-3 | 1.1e-4 | **2.6e-6** |
| INTERP `+0.25a` | 3.2e-2 | 1.8e-2 | 9.5e-3 | **1.9e-3** |
| EXTRAP `+1.50a` | 3.1e-1 | 1.5e-1 | 3.2e-1 | **2.8e-1** |
| EXTRAP `+2.00a` | DIVERGED | 7.6e-1 | 1.0e0 | **1.1e0** |

⚠ **`+1.50a` at `r=20` was published as `3.2e-1` and is `3.1e-1`.** Corrected
2026-08-25 when §2m re-ran the matrix at three significant figures: the true
value is `3.149e-1`, the harness's own discriminator prints `{:.2e}` = `3.15e-1`,
and the table was then rounded a SECOND time from that to `3.2e-1`. A figure
derived from an already-rounded figure — the failure this document keeps naming.
The other 15 cells reproduce exactly.

1. ★★★ **The subspace is wrong for a translated patch, and RANK DOES NOT FIX IT.**
   In-sample buys four orders across the ladder. Both extrapolations are **flat**
   in rank and the far one gets worse. This is the classic advection-like POD
   failure: a localised feature that translates is nearly orthogonal to itself one
   patch-width away, so representing the family costs one mode per position rather
   than a few modes overall.
   ⇒ **It is not the free edge.** `+1.50a` keeps `1.5a` of clearance and fails
   alike; `+2.00a` keeps `1a`. The pair is why both are scored (§2d's rule: test
   the difference before naming a cause).
2. ★★ **Even INTERPOLATION costs three orders.** Between training points `0.5a`
   apart, `+0.25a` plateaus at `1.9e-3` where in-sample reaches `2.6e-6`, and it
   improves only ~1.2 orders over a 7× rank increase against in-sample's four.
   **The reduction advantage dies long before the accuracy does** — buying
   generalisation with rank means giving back the thing R1 exists for.
3. ★★★ **Out-of-domain failure is SILENT in 7 of the 8 arms.** Seven converge,
   complete all 71 steps, and do not penetrate — `min_sd` stays positive and
   inside the band — while being `14.8 %`–`109 %` wrong. **Convergence plus
   non-penetration is not a validity check**, and it is exactly the "smooth and
   wrong" mode §4b warns about, now observed rather than anticipated.
   ⚠ **The eighth is the honest bound**: `+2.00a` at `r=20` stalls at step 65 and
   IS announced, so the mode is silent, not universally silent — and it is the
   highest cell in the matrix (`119.6 %`, over the steps it completed).
   ⚠ `28 %` and `109 %` are the **`r=142` column**, not the range; corrected in
   v2.16 and again here where the column had been read as a bracket.
4. ★★ **`gap_dev` detects it, which is not what it was built for** — with a
   caveat that a later audit added and that anyone building §4c's gate on it
   needs. Across all eight failing arms the converged contact gap moves
   **`0.161–0.583 d̂`** against `3.1e-11` in-sample at the same rank: four to five
   orders over the regression threshold it was piloted at, and nine orders over
   the in-sample value.

   ⚠ **But it is an ERROR indicator, not an in-domain indicator.** At `r = 20` the
   IN-SAMPLE arm itself reads `1.6e-4`, well over that threshold — because the
   answer is genuinely poor there (`relL2 = 2.4e-2`). `gap_dev` tracks *"is this
   configuration being solved well"*, which conflates "outside the training hull"
   with "basis too small". For a validity GATE that is arguably the better
   quantity — both are reasons to refuse an answer — but the claim must be stated
   that way, and the gate needs its own threshold study rather than inheriting
   the `2e-6` regression figure. ⚠ It is also not monotone in `relL2` within a
   regime (interpolation gets *worse* in `gap_dev` from `r=40` to `r=80` while
   `relL2` improves), so it separates regimes by orders of magnitude and ranks
   nothing finer.

**What this does to the plan.** §4c already names "contact configuration class"
as part of the `ReducedValidityDomain` parameter box, so this VALIDATES that
design rather than contradicting it — but it moves the gate from prudent to
mandatory and puts a number on the cost of not having one. ⛔ **`ReducedValidityDomain`
is revived as a CORRECTNESS prerequisite.** v2.7's retraction of it as a
PERFORMANCE prerequisite stands untouched and is a separate claim.

⚠ **And it qualifies "R1 COMPLETE".** R1's gates (§11, R1.0–R1.3) were all posed
and passed on a fixed contact configuration; none of them asked this. R1 is
complete *as scoped*, and the scope excluded the question that decides whether
R3 is worth building. §7's R1 row carries the qualification.

**Where the structural answer probably lives:** §5's hybrid domain decomposition —
a full-order patch around the contact with a reduced far field — is the standard
response to a moving localised feature, and it is sketched there and unmeasured.
Goal-oriented enrichment (plan §14) addresses a different axis (objective family)
and does not obviously help here.

Harness: `tests/reduced_contact.rs::reduced_basis_generalises`, `#[ignore]`d,
~5 min. Re-run it against any future basis scheme; the in-sample row is its
two-sided control and the rank trend is the discriminator.

### 2m. §4c rung 1 — the online-signal candidates, and what survives a per-step test (v2.15, rewritten v2.16)

⚠⚠ **This section was rewritten on 2026-08-25 after a cold re-derivation.** The
first version reported a trajectory MAXIMUM for a question about a per-step check.
That one choice inverted three of its four verdicts. The superseded claims are
tabulated at the end; the harness now prints both quantiles so the mistake is not
re-makeable.

§2l revived `ReducedValidityDomain` as a CORRECTNESS prerequisite and named
`gap_dev` as the detector. **`gap_dev` cannot gate** — it is
`|min_sd − ORACLE min_sd|`, so a runtime check would need the full-order answer it
exists to avoid. Four oracle-free candidates were scored on §2l's own matrix
(ground truth reproduces it at 15 of 16 cells and corrects the sixteenth):

| | signal | normalised by |
|---|---|---|
| S1 | `‖r_free‖ / tol` | a solver tolerance (i.e. not by load) |
| S2 | excursion outside the per-mode training box | each mode's OWN half-width |
| S3 | distance to the NEAREST TRAINING SNAPSHOT | one global scale (cloud radius) |
| S4 | fraction of contact-active vertices never active in training | the arm's own active set |

#### The question a gate is actually asked

**Not** "does this separate the training span from outside it." That is a
partition over POSITION, and on this fixture position and error are monotone
together, so it flatters anything that tracks the indenter. A gate refuses an
answer for being too WRONG. Is there ONE threshold with `accept ⟺ relL2 ≤ τ`?

| | `1e-1` | `5e-2` | `3e-2` | `2e-2` | `1e-2` | `5e-3` | `3e-3` | `2e-3` | `1e-3` |
|---|---|---|---|---|---|---|---|---|---|
| **S1** worst step | 6.80× | 6.80× | 2.37× | 1.37× | 2.71× | 2.69× | FAIL | FAIL | 98.8× |
| **S1** median step | FAIL | FAIL | 1.35× | FAIL | 1.13× | 5.04× | FAIL | FAIL | 85.2× |
| **S2** either | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| **S3** worst step | 2.30× | 2.30× | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | 151× |
| **S3** median step | **1.94×** | **1.94×** | 1.00× | FAIL | FAIL | FAIL | FAIL | FAIL | 56.4× |
| **S4** worst step | ∞ | ∞ | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| **S4** median step | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |

★ **The two rows per signal are a pair.** A gate firing when ANY step crosses `θ`
reaches its verdict on the **worst step** — that row is the decision. The
**median** row says how much of the trajectory stands behind it: separation on
both is a plateau; separation only at the max rests on a minority of steps.

#### The four verdicts

- ⛔ **S2 is dead** — FAIL at every `τ` on both rows. §5's open item 3 was still
  wrong that `q`-space carries no position information (S2 separates positions
  strongly at fixed rank), but the per-mode box converts none of it into a rule.
- ⛔ **S4's `∞` margin is an ARTIFACT of the maximum.** At `+1.50a` its median is
  **exactly `0.000` at all four ranks** while the answer is `14.8–31.8 %` wrong.
  The denominator, now printed rather than inferred, is **10–18 active vertices at
  `+1.50a`, 4–12 at `+2.00a`**, against 21 in-sample ⇒ the signal is one or two
  vertices on a few steps. ★ And it is confounded: the active set SHRINKS out of
  domain (`21 → 4`), so the fraction rises partly because the contact patch is
  disappearing, not only because it is novel.
- ⚠ **S1 decides correctly at 7 of 9 tolerances on the worst step, and its
  separation does not survive per-step**: at the median, `+0.25a r=20` reads
  `7.923e7` against `+1.50a r=142`'s `5.838e7` — **inverted, 0.737×**. It is also
  **not free at R3**: `‖r_free‖` exists only because R1.1 sweeps every element,
  which is what §4b's ECSW removes.
- ✅ **S3 is the only candidate surviving both rows** — `2.30×` worst step,
  `1.94×` median. Interpolation reads `5.062e-1 / 5.063e-1 / 5.063e-1 / 5.069e-1`,
  a spread of **`0.138 %`** over a `7.1×` rank change.

★★ **Two-sided control, in the same column:** flatness would prove nothing if the
statistic were trivially rank-invariant. The IN-SAMPLE row of the same quantity
falls **`5 168×`** (`3.59e-3 → 6.94e-7`) over the same range — as it must, since
in-sample the reduced solution converges onto trajectories that ARE training
points.

#### Why S3 and not S2 — and it is NOT max-versus-norm

Both read the same coordinates. S2 normalises **per mode**; S3 by **one global
scale**. A per-mode divisor is set by the thinnest retained axis — always the
newest tail mode — so S2's IN-SAMPLE row climbs `104×` with rank on the one row
in-domain by definition. ⇒ **use ONE GLOBAL normaliser, not a per-mode one.**

⛔ v2.15 drew this as *"take a NORM over modes, never a MAX"*, and that is
**provably wrong**: `‖d‖₂ ≥ ‖d‖∞` pointwise, so a norm over the SAME
per-mode-normalised deviations is at least as rank-sensitive as the max (measured
at `2.06× → 3.48×` over `r = 20 → 142`). The operation was never the fault. §7 had
already recorded the wrong rule as rung 2's design.

#### The spectrum, which settles `r=160 → 142`

`142` modes of `355` snapshots at `σ_last/σ_0 = 1.012e-8` — the truncation is
`SIGMA_FLOOR_REL`, nothing structural. But **retained energy is `1.000000000`**:
the top rung already extracts everything this ensemble holds energetically while
spanning 142 of 355 directions. ⇒ "flat in rank out of domain" is not a truncation
statement — more modes cannot help, only different snapshots.

#### ⇒ What rung 1 delivered

**Not a gate, and not a gate design.** A narrowed candidate set, and a measured
reason each of the others fails. S3 is left standing at `~2×`, on a fitted
threshold, on one fixture. §4c's gate is **not ready to build**, and v2.15's
"rung 2 is implementation, not research" is withdrawn.

⇒ ⛔ **AND S3 DID NOT SURVIVE THE NEXT TWO RUNGS — read them before building on
anything above.** §2n (v2.17) supplies the held-out position this section says it
lacks: the `~2×` does NOT survive it, and S3's rank-independence — named above as
its signature — is precisely what disqualifies it as an accuracy gate. §2o (v2.18)
then shows it is blind to ENSEMBLE SIZE as well. The two-signal conclusion
withdrawn here returns in §2n on that evidence.

#### ⚠ What this does NOT establish

- **No held-out position exists.** Every margin is computed on the same 16 cells
  that chose the threshold. The hull edge is `1.0a`, the nearest scored
  out-of-domain point `1.5a`, and **nothing is sampled between** — so every window
  is an upper bound on an unmeasured quantity.
- **The DOMAIN/ERROR taxonomy is not falsifiable here.** Distance-from-training
  and error are monotone together across all four positions; no cell could have
  told the two kinds apart. The labels are definitions.
- **Extrapolation is confounded with the free edge** (`1.5a` and `1a` clearance on
  an `8a` plate), and `+0.25a` is **the only offset not on a lattice site**
  (`0.5` cells, against training at `0, ±1, ±2` and extrapolation at `3, 4`) — the
  one arm the product question rests on.
- **The in-sample column measures RECONSTRUCTION, not prediction**, and is the
  anchor every ratio is taken against.
- **One basis construction**; `snapshot_distance` assumes `Inner::Mass` and
  nothing checks it; **every arm ran `Inertial`** and the exposure is not S1's
  alone (S3 reads `q`, S4 reads `x_rest + Φq` — both the solved state); **no
  timing**; **no threshold set**.

#### ⛔ Superseded by this rewrite

| v2.15 published | v2.16 |
|---|---|
| "the same value to FOUR significant figures" | distinct at 4 and 3 s.f.; `0.138 %` spread, same at TWO |
| "take a NORM over modes, never a MAX" | the per-mode NORMALISER was the fault |
| "S4 wins that cut outright" | a maximum-only artifact; median `0.000` at `+1.50a` |
| "S3's margin is `141×`" | `2.30×` worst step, `1.94×` median |
| "the gate needs TWO signals" | not shown for GATING — that argument is about DIAGNOSIS. ⚠ The CONCLUSION returns in §2n on held-out evidence; only the v2.15 argument for it stays withdrawn |
| "`‖r_free‖` … and it is free" | free at R1.1, NOT under ECSW at R3 |
| "`28 %`–`109 %` wrong" | that is the `r=142` column; the range is `14.8 %`–`119.6 %` across all eight cells, `14.8 %`–`109 %` across the seven that complete |
| "`≥ 1.17` extrapolating" | the minimum is `1.165` |
| "byte-identical output" | the 37 measurement ROWS are identical; the files differ |
| pre-registration item 5 "HELD" | VOID — its antecedent (item 4) was falsified |

Harness: `tests/reduced_contact.rs::an_online_signal_separates_out_of_domain`,
`#[ignore]`d, `140.4–146.1 s` across six runs. ⚠ The six are NOT byte-identical as
files — the instrument gained tables and one column was relabelled — but every arm
line and every shared table cell agrees exactly across all six, which is the only
evidence any single number here is a determined one.
It now keeps every step rather than a running maximum, prints both quantiles,
`active_novelty`'s denominator, the singular-value spectrum, and computes the gate
table itself — so the analysis that overturned v2.15 cannot drift from the data
again.

### 2n. §4c rung 1b — the held-out position, and why rank-independence disqualifies the survivor (v2.17)

§2m ended with one candidate standing (`snapshot_distance`) and named its
**rank-independence** as the signature that made it a domain signal. It also
recorded that every margin it quoted was FITTED — derived from the same 16 cells
it was scored on, with no held-out position anywhere. This rung supplies one, and
the two facts collide.

**Arm A — leave-one-out.** Refit on `[-1, -0.5, +0.5, +1]a`, score at `+0.00a`.
**Arm B — lattice control.** `+1.75a` (off-lattice, `3.5` cells) against the
`+1.50a` / `+2.00a` bracket, both on it. 8 oracles, `140.8 s`. ⚠ Two runs exist;
the first (`139.7 s`) recorded the signals and printed none of them, so every
signal figure below is the second's. The 16 arm rows are identical across both.

#### ★★★ Rank-independence is why it cannot gate

Four ranks, ONE basis family, ONE hull — what a deployed gate actually sees, since
a gate ships with a single ensemble and a single threshold:

| | span across the four ranks |
|---|---|
| `max_rel_err` | **31.9×** (`3.733e-1 → 1.172e-2`) |
| `snapshot_distance` worst step | **1.005×** |
| `snapshot_distance` median step | 1.023× |
| `residual_excess` worst step | 6.409× |
| `active_novelty` | `0.000` at every rank |

A signal that does not move with rank **cannot report how well the basis resolved
the answer**, because resolution is the thing rank changes. Within-ensemble
threshold margins for `snapshot_distance` are `1.002 / 1.002 / 1.002 / 1.000` over
`τ = 1e-1 … 2e-2` — noise. `residual_excess` gets `2.883×` on the same cells,
because it is an error signal and tracks rank by construction.

⇒ **Rank-independence and accuracy-sensitivity are mutually exclusive**, and that
is a consequence of what each quantity measures rather than a design choice.

#### ⇒ The two-signal conclusion returns, on opposite evidence

§2m WITHDREW "the gate needs TWO signals" because the argument offered for it was
about DIAGNOSIS and had been sold as gating. It returns here on a **held-out**
position rather than a confounded position partition: `snapshot_distance` reads
position and is blind to resolution; `residual_excess` reads resolution, is
per-step blind to position (§2m), and is unavailable under ECSW. **Neither alone
gates**, and that is now measured rather than argued.

#### ⛔ §2m's fitted margin does not survive

The load-bearing form is WITHIN arm A, so no normaliser question touches it: four
readings spanning `0.9704–0.9756` carry errors of `1.2 %` to `37 %`. **No threshold
separates them** — below `0.9704` refuses all four, above `0.9756` accepts all four,
and there is nothing in between to place one at.

⚠ §2m's interval was `[0.5069, 1.165)` and arm A lands inside it, in the band
nothing was ever sampled in — but that is a **DIRECTION, not a number**: §2m's
interval is in the 355-snapshot ensemble's units, arm A's readings in the
284-snapshot ensemble's, and `snapshot_distance` divides by each ensemble's own
cloud radius. See the caveats.

#### The matched-gap pair

Held-out `+0.00a` and `+1.50a` both sit `0.5a` from their nearest training
snapshot and both ON the lattice — one interpolating, one extrapolating:

| rank | HELD-OUT `+0.00a` | `+1.50a` | ratio |
|---|---:|---:|---:|
| 20 | 3.733e-1 | 3.149e-1 | **0.84×** |
| 40 | 1.406e-1 | 1.478e-1 | **1.05×** |
| 80 | 2.902e-2 | 3.177e-1 | 10.95× |
| 116 / 142 | 1.172e-2 | 2.822e-1 | 24.08× |

★★★ At `r=20` **extrapolation is the better of the two**, and at `r=40` they are
indistinguishable. Being inside the hull does not make the answer better at a given
rank — it makes the answer **improvable by rank**: interpolation falls `31.9×`
across the sweep, extrapolation `1.12×`. ⇒ This sharpens §2l's "rank does not fix
it" to: **rank does not fix it OUTSIDE the hull, and does fix it inside, at a rate
the gap degrades.** ⚠ And at that matched gap `snapshot_distance`'s median reads
`4.117e-1` against `4.271e-1` — `1.04×` for a `24×` error difference. ⚠⚠ Those two
medians come from different ensembles and so from different normalisers: read
`1.04×` as a direction. The within-arm form needs no cross-arm comparison and says
the same — inside arm A the median moves `1.023×` across a `31.9×` error range.

#### The lattice control

`+1.75a` falls **INSIDE** the `+1.50a`/`+2.00a` bracket at all four ranks, and
error tracks gap smoothly across `0.5a → 0.75a → 1.0a` with no lattice jump. The
mesh confound raised against `+0.25a` is largely defused. ⚠ Evidence, not proof,
for the interpolation end: with `cell = 0.5a` every on-lattice point inside the
training span IS a training point, so no on-lattice interpolation control can
exist on this mesh.

#### ⚠ What this does NOT establish

- **One held-out position** — better than none, still a single point.
- ⚠⚠ **The two arms use different ensembles** (284 snapshots truncating at `r=116`
  vs 355 at `r=142`), and `snapshot_distance` divides by its own ensemble's cloud
  radius — so **every cross-arm number in this section is a DIRECTION, the median
  comparison included.** (v2.17 said the median comparison was sound. It is the
  same comparison.) The sound readings are the WITHIN-arm spans: `1.005×` worst
  step and `1.023×` median against `31.9×` in error. The normaliser ratio is
  unmeasured — one re-run printing `cloud_radius` per arm would settle it, and
  nothing above waits on it. ✅ §2o carries one cross-ensemble reading (`~1.1×`)
  and labels it a direction rather than a number — the form this section should
  have used.
- The lattice control tests the **extrapolation end only**.
- No timing, no threshold, one fixture, one basis construction.

Harness: `tests/reduced_contact.rs::the_signal_margin_on_a_held_out_position`,
`#[ignore]`d. ⚠ Its first version recorded the signals and PRINTED NONE of them,
leaving two of its own pre-registered items unanswerable from its output — a
prediction whose data is not emitted is not a prediction, and the harness now
emits it.

### 2o. §4c rung 1c — "density" is TWO factors, and the weaker one is what the signals measure (v2.18)

§10 risk 0 carried §4c's domain-coverage half as untouched and §5's open item 3 was
re-scoped to it: **how dense must the training ensemble be?** A parameter box in the
**Stated** clause is meaningless without it.

Every reduced result in this arc conflates two factors, because refining a training
grid moves both at once: the SPACING between neighbouring trajectories, and the
NUMBER of them. A 2-factor design over offsets whose oracles already exist, all
scored at the same held-out `+0.00a`, costing no new trajectory:

| ensemble | `r=20` | `r=40` | at its own ceiling |
|---|---:|---:|---:|
| **A** 4 traj, gap `0.50a` | 3.733e-1 | 1.406e-1 | 1.172e-2 (`r=116`) |
| **B** 2 traj, gap `0.50a` | 7.949e-1 | 3.790e-1 | 1.394e-1 (`r=64`) |
| **C** 2 traj, gap `1.00a` | 8.632e-1 | 4.448e-1 | 3.058e-1 (`r=65`) |

#### ★★★ At matched rank, SIZE dominates GAP

- **A vs B** — doubling the ENSEMBLE at fixed gap buys **`2.13×` / `2.70×`**.
- **B vs C** — doubling the GAP at fixed size costs **`1.09×` / `1.17×`**.

"Denser is better" resolves into two effects of unequal weight, and **the one every
online candidate in §2m measures — the gap — is the weaker.**

★★ **Size compounds, because it sets the rank CEILING.** A's 284 snapshots retain
116 modes; B's 142 retain 64 (C's 142 retain 65 — the ceiling comes from the
spectrum, not the count alone). B cannot pass `r=64` however much rank is asked for, so at
their respective ceilings A beats B by **`11.9×`** against the `2.1–2.7×` basis
quality alone accounts for. A small ensemble is penalised twice — a worse basis at
every rank, and a lower rank to stop at.

#### ⇒ Second strike against §2m's surviving candidate

A and B are at **the same gap** and differ `2.13–2.70×` in error. The domain
signals read:

- `active_novelty`: **`0.000` for both at every rank.** Identical.
- `snapshot_distance` median: A `0.411–0.420`, B `0.447–0.500` — ~`1.1×`, ⚠ and the
  normaliser changes across ensembles, so even that is a direction not a number.

**Neither can express ensemble size.** §2n showed `snapshot_distance` is blind to
RESOLUTION (rank-independent by construction); this shows it is blind to ENSEMBLE
SIZE too. It reads the sampling PITCH — measured here as the smaller of the two
things that set the error.

⇒ **A `ReducedValidityDomain` must state CARDINALITY and not only pitch, and the
online candidate cannot check the clause that matters more.**

#### ✅ And C is the positive control for `active_novelty`

C fires — `0.250 / 0.182 / 0.167` — where A and B read exactly zero, because
`[-1, +1]` leaves a hole at the centre that `±0.5a` sweeps through. So that signal
does measure something real, **COVERAGE**, and fires exactly when coverage fails.
This sweep measures coverage to be the factor that is NOT dominating.

#### ⚠ What this does NOT establish

- One held-out point, one gap pair, one size pair — `2.13–2.70×` and `1.09–1.17×`
  are two points each, not a scaling law.
- **Trajectory count and SNAPSHOT count are confounded**: B and C hold 142 snapshots
  because they hold 2 ramps of 71 steps. A longer ramp at fewer offsets would
  separate them; nothing here does.
- Only the matched-rank rows isolate a factor. The own-ceiling column mixes basis
  quality with the ceiling and is reported separately for that reason.

Harness: `tests/reduced_contact.rs::how_dense_the_training_ensemble_must_be`,
`#[ignore]`d, `82.7 s`, zero new full-order trajectories.

## 3. What the measurements say about feasibility

### 3a. The goal is not refuted

Nothing measured says real time is unreachable. The gap is large — **13–47× in DOF,
and 35–267× in measured frame time** (§2a; stated in time because that is the unit the
literature figure that follows is in) — but it is the *size* of gap that reduced-order modelling
is built for: ECSW/cubature literature routinely reports 2–3 orders of magnitude on
exactly this shape of problem (hyperelastic FEM, fixed mesh, repeated solves). The brief's central architectural
claim — that a reduced system is small and dense, so **converged Newton with an exact
direct solve is preserved** and the error moves into the quadrature approximation
where it can be measured against the full-order path — survives contact with the
numbers, and §2d.5 strengthens it: since triangular solves are already negligible,
shrinking the system to a dense `r × r` solve costs nothing that was load-bearing.

### 3b. Where the measurements bite the plan

1. **The f32 split (§2c) is different from the one assumed.** The plan must be
   mixed-precision from the start, and the *gradient* — not the forward solve — is the
   precision-critical path. Retrofitting that later means re-deriving the reduced
   adjoint twice.
2. **Two full-order levers must be measured before reduction is justified.** §2d.2's
   `BTreeMap` rebuild dominates precisely the DOF regime a real-time path targets. If
   removing it is worth (say) 2–3× at ≤ 5 k DOF, the reduction target moves, the rung-1
   kill/confirm threshold moves, and the honest reduction factor to claim moves. Rung
   R0 exists to prevent the project from crediting MOR with a win that a `Vec` would
   have bought.
3. **The `block_sag`/`cantilever` spread (0.5 vs 37 iterations at comparable DOF) is a
   warning about the validity domain.** Newton iteration count on this problem class is
   dominated by *how nonlinear the trajectory is*, not by DOF. Any real-time claim
   stated as "N DOF at 60 Hz" is under-specified; it has to be "N DOF at 60 Hz **on
   trajectories inside the stated validity domain**", which is exactly the accepted
   cost the brief already names. The measurement supports making that domain statement
   load-bearing rather than decorative.
4. **Newton iteration count grows with mesh refinement at fixed `dt`, and that
   compounds the frame-budget scaling.** Peak iterations per step across the cantilever
   sweep, all converged: **21 → 37 → 41 → 56 → 65** for 567 → 36 663 free DOF. Cost per
   step is `iterations × per-iteration cost`, so the effective DOF exponent is worse
   than the `n^1.43` of §2a's per-iteration row — a reduced model has to beat both
   factors, not just the linear algebra.

   **Mechanism** (measured, not assumed): Tet4 volumetric locking *decreases* with
   refinement, so the finer beam is genuinely softer and deflects further per step —
   `max |displacement|` after 6 steps rises 3.6e-2 → 5.7e-2 m and plateaus as the mesh
   converges. The finer mesh is not solving the same problem more slowly; it is solving
   a harder (less artificially stiff) problem. Iteration growth appears to plateau with
   it (52 → 56 → 62 → 65).

   ⚠ **This supersedes a claim earlier drafts of this document made, which was wrong.**
   v1.0–v1.5 recorded "the f64 oracle FAILS to converge at 36 300 free DOF, and this is
   unexplained", flagged it as a defect on the validation path, and ranked it above R1.
   It was not a defect. The harness that produced it set `max_newton_iter = 60`, an
   arbitrary choice; the case needs **65**. With a generous cap every size converges to
   `tol = 1e-6` at every step: [11, 32, 48, 62, 65, 64, 55, 40, 37, 22]. See §3d.

### 3c. What does *not* need to change

**The CPU f64 converged-Newton + sparse-direct path stays exactly as it is.** Nothing
measured argues against it, and §2b/§2c make the case *for* it: it is the only path
whose gradients are good to more than 3 digits, and it is therefore the only thing
that can grade the others. This recon proposes no change to it beyond R0's assembly
lever (which is a pure speedup, byte-identity-checkable).

---

### 3d. The "oracle does not converge" finding, diagnosed and withdrawn

Measured 2026-08-11 on `main` at `ecf4cfef` (post-R0). Five discriminators, run against
the same cantilever fixture:

| # | experiment | result |
|---|---|---|
| **A** | reproduce, `dt = 1/60`, cap 60 | `NewtonIterCap` at step 3, `r = 8.69e-3`, iterations `[11, 32, 48]` |
| **B** | same, cap **300** | **converges every step**: `[11, 32, 48, 62, 65, 64, 55, 40, 37, 22]`, tip 8.83e-2 m |
| **C** | `dt` sweep at 36 663 DOF | 1/60 → needs 65 · 1/120 → 11 · 1/240 → 4 · 1/500 → 2 · 1/1000 → 2 |
| **D** | mesh sweep, cap 300 | every size converges; peak iterations 21 / 37 / 41 / 56 / 65 |
| **E** | LM regularization on | **no change whatsoever** — identical failure, identical residual |

**What each rules out.** The failure variant is `NewtonIterCap`, not `ArmijoStall`
(the line search never ran out of descent), not `DoublyFailedFactor` (the tangent
factored as `Llt` every time — never non-PD, never needing the LU fallback), and not
`ValidityViolation` (no element inverted, no stretch bound exceeded). E confirms the
tangent stayed SPD throughout: LM only engages on an `Llt` failure, and it changed
nothing because there was nothing to rescue. C shows the iteration count is governed by
step size in the ordinary way. **Nothing here is a solver pathology.** It is a hard load
step — a soft beam released horizontal, asked to travel most of the way to a distant
equilibrium in one `1/60 s` step — solved correctly, in 65 iterations.

**Why the error was made and how it survived.** The measuring harness set
`max_newton_iter = 60` with no reasoning behind the number; the case needs 65. It then
failed *loudly and correctly*, and the loud failure was read as a finding about the
solver rather than about the harness. The lesson worth keeping: **a cap you chose
yourself is not evidence.** When a run hits a limit you set, the first hypothesis is the
limit, not the system under test — and the cheapest possible discriminator (raise it)
was not run for a day.

**One genuine sharp edge surfaced.** `SolverConfig::skeleton()` ships
`max_newton_iter: 10`. Every dynamic fixture in the crate overrides it (50, 80, 150),
because 10 is far below what any large-deflection step needs. That default is a
reasonable floor for the walking-skeleton scene it is named for, but it is a trap for a
new consumer, who will meet it as an opaque `NewtonIterCap` panic. Not fixed here —
flagged, because changing a shipped default is its own change with its own blast radius.

## 4. The MOR + hyper-reduction path

### 4a. Basis construction

**Snapshot POD from full-order runs.** Run the f64 oracle over a trajectory ensemble
that spans the intended validity domain, collect displacement snapshots
`U = [u₁ … u_m]` (`n × m`), take the thin SVD, keep the leading `r` left singular
vectors as `Φ` (`n × r`). Reduced coordinates `q ∈ ℝ^r`, `u ≈ Φq`.

- **No new dependency.** `nalgebra` is already a `sim-soft` dependency and ships
  `SVD`. At the sizes involved (`n` up to ~70 k, `m` in the hundreds) the economical
  route is the Gram-matrix eigendecomposition (`UᵀU`, `m × m`) rather than a dense
  `n × m` SVD — also pure `nalgebra`.
- **Snapshot ensemble is the real design problem, not the SVD.** The basis can only
  represent what the ensemble showed it. This is the same discipline the crate already
  applies to `MaterialField` validity, and it is where the validity domain is actually
  decided.
- **Modal derivatives** (second-order Taylor terms of the equilibrium manifold) are the
  standard cheap enrichment when the ensemble is thin. Deferred to R3; noted so the
  basis API does not foreclose it.

### 4b. Hyper-reduction

The reduced internal force `Φᵀf_int(Φq)` and tangent `ΦᵀK(Φq)Φ` still require a sweep
over **all** elements unless the quadrature is reduced. Two levers, and §2d sets their
relative size:

- **The basis alone attacks the larger half — but only above ~20 k free DOF.**
  Collapsing an `n`-DOF sparse factorization to a dense `r × r` solve targets the
  57–72 % of the frame that factorization costs. ⚠ The reduced tangent must still be
  *formed*: `ΦᵀAΦ` costs `2·nnz·r + 2·n·r²`, which is **linear in `n`** where the
  factorization it replaces is superlinear — so it wins asymptotically and is a wash at
  small `n`. At `r = 50` against §2a's measured `numF`, the arithmetic puts the
  crossover near 10–20 k free DOF: comparable-or-worse at 3 000, ~1.1× at 19 440, ~2.5×
  at 70 644. **That does not change R1's gate** — it is accuracy, not wall time — but it
  does mean a speedup at R1 should be expected only at the larger sizes. Worked in
  `docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md` §4 (arithmetic, not measurement).
- **Hyper-reduction is needed for the rest, and cannot be skipped.** Assembly is
  9–32 % of the frame, and a basis without hyper-reduction leaves all of it. That is a
  hard floor on what R1 alone can reach, and it is why R3 exists.

**Recommendation: ECSW (Energy-Conserving Sampling and Weighting).** Reasons, in the
order they matter here:

1. It selects a **subset of elements with positive weights**, so the reduced force is
   still assembled by the *existing* element kernels — the Yeoh/Neo-Hookean energy is
   preserved exactly, which is the brief's stated non-negotiable, and the constitutive
   code is untouched.
2. Its training problem is a **non-negative least squares** with a sparsity target,
   which is implementable in-crate (Lawson–Hanson NNLS is ~150 LOC over `nalgebra`) —
   **no new dependency**, which the L0 budget requires (§7's constraint block).
3. The weights are constants, so **the reduced model stays differentiable with respect
   to material and design parameters through exactly the same chain the full model
   uses** (§6).

Optimised cubature (Farbman/An et al.) is the alternative; it optimises points rather
than elements and its training is greedier. ECSW is chosen for (1) and (3), not for
accuracy superiority — that is not claimed and would need measuring.

### 4c. The validity domain: stated, measured, gated

> ⛔ **v2.14 — this is now a CORRECTNESS PREREQUISITE, not a prudent addition.**
> §2l measures the reduced solver returning a **converged, non-penetrating,
> `14.8–109 %`-wrong** answer once the contact patch moves outside the training
> hull, in 7 of its 8 extrapolation arms (the eighth stalls and is announced). The parameter box below already names "contact configuration class",
> so the design anticipated it — what changed is that the cost of shipping
> without the gate is now measured rather than argued. ⚠ v2.7's retraction of
> this section as a PERFORMANCE prerequisite is a separate claim and stands.
> ⛔ **`gap_dev` IS NOT A CANDIDATE FOR THE ONLINE SIGNAL, and an earlier version
> of this paragraph said it was.** It detects the failure well
> (`0.161–0.583 d̂` out-of-domain against `3.1e-11` in-sample at the same rank),
> but it is `|min_sd − ORACLE min_sd|` — it differences against the full-order
> answer, so it is a DIAGNOSTIC and can never gate anything. §2m (v2.15)
> measured four candidates that are genuinely oracle-free and found one that is
> flat in rank. ⚠ The separate caveat below stands and generalises: `gap_dev` is
> an ERROR indicator, not an in-domain one — at `r = 20` the in-sample arm reads
> `1.6e-4` because the answer is genuinely poor there — and §2m confirms the
> same conflation in the oracle-free error signal, which is exactly why the gate
> needs a DOMAIN signal beside it.


This mirrors `sim-soft`'s existing `ValidityDomain` on `Material` (which already gates
`inversion` and `max_stretch_deviation`, and which `check_validity_at_step_start`
already enforces at both step boundaries, fail-closed). The reduced model gets the
same treatment, one level up:

**Stated** — a `ReducedValidityDomain` declaring, at minimum: the snapshot ensemble's
parameter box (material range, load range, contact configuration class), a bound on
reduced-coordinate magnitude `‖q‖` relative to the training envelope, and the training
ensemble's identity (hash) so a basis can never be silently used off a different
ensemble.

**Measured** — against the full-order oracle, three quantities, all of which the
existing harness can already produce:
- **projection error** `‖u − ΦΦᵀu‖ / ‖u‖` — the basis's own ceiling, independent of
  hyper-reduction;
- **hyper-reduction error** `‖Φᵀf_ECSW − Φᵀf_full‖ / ‖Φᵀf_full‖` — the quadrature
  approximation in isolation, which is the term the brief correctly identifies as where
  the error enters;
- **end-to-end trajectory error** against the oracle, in the physical observable that
  matters (reaction force, contact pressure), not in DOF-space L2.

**Gated** — by the same fail-closed mechanism the material domain uses: leaving the
domain returns `SolverFailure::ValidityViolation` from the `try_` path and panics from
the direct path. ⚠ The one genuinely new thing: a reduced model can leave its domain
*silently and plausibly* — it will happily produce a smooth, wrong answer. The gate
therefore has to be **online**, on `‖q‖` and on a cheap residual proxy, not merely a
statement in a docstring. That is a design requirement, and it is the single most
important thing to get right in R3. ⚠ **"online" survived rung 1; the two specific
statistics named in this sentence did not** — see the block immediately below, which
is the scorecard on this paragraph and not a restatement of it.

#### ⚠ Rung 1 done — the online signal is MEASURED, and NO CANDIDATE SURVIVED IT (§2m v2.16, §2n v2.17, §2o v2.18)

⚠⚠ **This block was written against §2m alone and is read with §2n and §2o or not
at all.** §2m left one candidate standing; §2n and §2o each disqualified it, for
reasons that are properties of the quantity rather than of the fixture.

Four oracle-free candidates were scored on §2l's matrix. What the design above got
right, wrong, and left unsaid:

- ✅ **"the gate has to be ONLINE" was the load-bearing call.** `gap_dev`, the one
  detector §2l measured, differences against the ORACLE's `min_sd` and is therefore a
  diagnostic that could never have shipped as a gate.
- ⚠ **"on `‖q‖`" was right about the coordinates and wrong about the statistic.** A
  per-mode envelope box is dominated by the thinnest retained axis and has NO
  rank-independent threshold — its in-sample row climbs `104×` with rank. The distance
  to the nearest training snapshot, normalised by ONE GLOBAL scale, holds to `0.138 %`
  across a `7.1×` rank change. ⇒ **use one global normaliser, not a per-mode one.**
  ⛔ v2.15 drew this as "a NORM over modes, never a MAX" and that was provably wrong:
  `‖d‖₂ ≥ ‖d‖∞`, so a norm over the same per-mode-normalised deviations is at least as
  rank-sensitive as the max. The operation was never the fault.
  ⛔⛔ **And that `0.138 %` flatness is the DISQUALIFIER, not the recommendation
  (§2n).** On a held-out position the same statistic spans `1.005×` across four
  ranks while the error spans `31.9×`. **Rank-independence and accuracy-sensitivity
  are mutually exclusive** — a signal that does not move with rank cannot report
  how well the basis resolved the answer, because resolution is what rank changes.
  ⇒ `‖q‖`-space gives a DOMAIN reading and can never give an ACCURACY one. That is
  a constraint on the design, not a defect in the statistic.
- ⚠ **"and on a cheap residual proxy" was right, and it is free ONLY AT R1.1.**
  `‖r_free‖` is available because R1.1 sweeps every element — which is precisely what
  §4b's ECSW removes. At R3, the rung this gate ships on, the recommended backstop
  costs the assembly the rung exists to eliminate. v2.15 said "free" without this.
- ⛔ **v2.15 concluded the gate needs TWO signals; §2m WITHDREW it as a gating claim;
  §2n REINSTATES it on evidence §2m did not have. The conclusion stands — the
  argument for it does not.** §2m's gate table shows the error signal deciding
  correctly at 7 of 9 tolerances on the worst step while both geometric candidates
  fail at `1e-2`, which proves a DOMAIN-ONLY gate fails; v2.15's "opposite remedies"
  argument was about DIAGNOSIS and was sold as gating. §2n then measures the other
  half on a HELD-OUT position rather than a confounded position partition:
  `snapshot_distance` reads position and is blind to resolution, `residual_excess`
  reads resolution, is per-step blind to position, and is **unavailable under
  ECSW**. ⇒ **Neither alone gates, and the one that reads accuracy is the one R3
  cannot afford.** That is the sharp form of the problem rung 2 inherits.
- ⚠ The **"contact configuration class"** named in **Stated** has a candidate
  realisation in `active_novelty`, and §2m measured it to be **an artifact of a
  trajectory maximum**: its median at `+1.50a` is exactly zero at every rank, over an
  active set of 10–18 vertices. Not usable as stated. ✅ §2o rehabilitates it as a
  COVERAGE signal — it fires (`0.167–0.250`) on the `±1.0a`-only ensemble, whose
  training leaves a HOLE at the centre, and reads exactly `0.000` on the two whose
  `±0.5a` trajectories sweep through it. ⚠ A hole, not an absence: at its worst
  step C still recognises `75 %` of the scored active set. Coverage is measured
  there to be the factor that does NOT dominate.
- ⛔⛔ **The parameter box must state CARDINALITY, and no online candidate can
  check it (§2o).** "Density" is two factors. At matched rank, doubling the
  ENSEMBLE at fixed gap buys `2.13–2.70×`; doubling the GAP at fixed size costs
  `1.09–1.17×`. Size compounds through the rank ceiling to `11.9×`. **Every online
  candidate here reads the gap** — the weaker factor — and two ensembles at the
  SAME gap differing `2.1–2.7×` in error read identically on `active_novelty` and
  ~`1.1×` apart on `snapshot_distance` (⚠ a DIRECTION — the two ensembles
  normalise by their own cloud radii). A **Stated** clause that names a parameter
  box without a snapshot count is therefore under-specified, and the **Gated**
  clause cannot enforce the part that matters most.

⇒ **Rung 2 is NOT ready, and rung 1's survivor did not survive.** §2m left one
candidate standing (`snapshot_distance`, `~2×` on a fitted threshold on one
fixture); §2n disqualified it on resolution and §2o on cardinality. What rung 1
actually delivered is a narrowed candidate set, a measured reason each fails, and
**two hard constraints a gate design has to route around.** ⚠ They do not have
the same standing, and the more certain one is not the more interesting one:

1. **AFFORDABILITY, and it is definitional.** `‖r_free‖` requires the full element
   sweep; ECSW exists to remove exactly that sweep. No measurement is needed and
   none can overturn it. The error signal that reads accuracy is the one R3
   cannot have.
2. **A near-impossibility, and it is the load-bearing one.** A domain signal
   cannot read accuracy: accuracy at fixed ensemble is a function of rank, so a
   signal constant in rank cannot track it. The ARGUMENT is definitional; the
   PREMISE — that `snapshot_distance` is constant in rank — is measured at
   `1.005×`, on one statistic at one held-out position. Treat it as very strong
   evidence, not as a theorem.

v2.15's "rung 2 is implementation, not research" is withdrawn.

✅ **The prerequisite for rung 2 — an API change, not a measurement — is
DISCHARGED (v2.20).** `ReducedStep` used to expose only
`projected_residual_norm` / `full_residual_norm`, both scalars, so anything that
reads the residual VECTOR (a per-mode or per-region error indicator, which is how
v2.19 named the next candidate class — the sub-section below kills the per-mode
half) could not be built against the surface at all.
`ReducedStep::full_residual` now hands back the converged `r_free` itself —
`n_free` entries in `free_dof_indices()` order — and `full_residual_norm()`
becomes a method derived from it, so the norm and the vector cannot disagree.
A spatially resolved indicator groups it through the free-DOF index. Gated by
`reduced_newton_trajectory::a_converged_step_hands_back_the_residual_it_converged_on`,
which re-projects the handed-out vector and requires a bit-exact match against the
separately-recorded projected norm — the assertion that a stale Newton iterate
cannot pass.

⚠ **This unblocks BUILDING the candidate class; it does not make it affordable.**
Constraint 1 above is untouched: `r_free` still costs the full element sweep ECSW
exists to remove, so a spatially resolved indicator built on it is a RESEARCH
instrument on a path that is already paying the sweep, not a shippable R3 gate.
Constraint 2 is untouched too. **Rung 2 remains research, and it still has to
route around both.**

### ⛔ And the vector immediately KILLS half of what it unblocked: PER-MODE

The candidate class was written up as "per-mode or per-region". The first is
**dead on arrival, by construction**, and the vector is what shows it. The obvious
per-mode reading is `Φᵀr_free` componentwise — but that is exactly the quantity
the Galerkin solve drove below `tol`, so at convergence it holds no information
about which mode is at fault. Measured on the gate's fixture (`n_free = 150`,
`r = 8`, one step): the LARGEST of its 8 components is `2.6e-13`, against a
per-DOF max of `1.8e-2` in `r_free` — **11 orders down**, so every component is.
A per-mode indicator would have to test `r_free` against directions the basis does
**not** retain, which needs more than `Φ` and is therefore a different piece of
work than this API change.

**What survives is spatial.** On the same step, the top decile of DOFs carries
**`90.5 %`** of `‖r_free‖₁` — the out-of-span residual is strongly localized, so
grouping by region has something to read. ⚠ One fixture, one step, one rank: a
DIRECTION, not a law. Both numbers are printed by
`a_converged_step_hands_back_the_residual_it_converged_on`, so a second fixture
checks them without re-deriving anything. ⚠ That gate also prints a `ratio=`
field, and it is **not** the per-mode figure: it is `‖Φᵀr‖ / ‖r_free‖`, a
norm-over-norm that happens to land at a similar magnitude. The per-mode reading
is the `max |(Φᵀr)_i|` field beside it.

⇒ Rung 2's candidate space narrows again, before a single candidate is built:
**spatially resolved, or nothing.**

---

## 5. Contact — the hard part

**This is the section to be most sceptical of, and §2e sharpens why.** Contact costs
0.02 % of a frame to *evaluate*. What it does is put localized, high-rank,
topology-changing deformation into the displacement field — exactly the content a
global smooth subspace represents worst. A POD basis trained on trajectories with the
indenter at one location cannot represent the deformation when it arrives somewhere
else, and no amount of enrichment fixes that in general, because the space of contact
configurations is not low-dimensional.

**Assessment of hybrid domain decomposition** (full DOF in the contact region, reduced
basis in the bulk):

- **The mechanism is sound and the measurements support it.** The contact region is
  small: the IPC indentation engaged, at most, the vertices under a sphere of contact
  radius `a = 2.24 mm` on an 18 750-DOF mesh. A full-DOF patch of a few hundred DOF
  coupled to a reduced bulk of `r ≈ 30–100` gives a system of ~500–800 DOF — comfortably
  inside the **≈ 1 500 free DOF** §2a measures as reachable at 60 Hz, with roughly 2×
  headroom. The arithmetic closes, which is a genuine (if provisional) encouragement.
  ⚠ Until 2026-08-22 this sentence quoted §2a *by name* as saying "≈ 800 free DOF … is
  *exactly* the reachable size" — a pre-R0 figure §2a itself stopped carrying at v1.3.
  The correction turns "exactly at the limit" into "inside it with margin"; it does not
  rescue anything, because the hard part is the coupling condition below, not the count.
- **The coupling is where it is hard, and it is open research.** The interface between
  a full-DOF patch and a reduced bulk needs a compatibility condition. Options span
  static condensation of the bulk onto the interface (clean, but the condensed
  operator must be recomputed when the bulk basis is nonlinear), Lagrange-multiplier
  or mortar coupling (well-understood statically, less so with a moving contact patch
  and IPC's barrier), and Craig–Bampton-style substructuring with interface modes
  (mature in linear structural dynamics, not in the large-deformation hyperelastic
  contact setting).
- **The patch must move, and that is the genuinely unsolved part.** A fixed patch only
  works if the contact location is known in advance. Sliding or migrating contact
  requires re-partitioning at runtime, which changes the system's sparsity pattern —
  and `sim-soft`'s entire fast path is built on a symbolic factorization computed
  *once* at construction (`construct.rs`, and the explicit contract in
  `replace_contact` that contact may not widen the pattern). Re-partitioning per frame
  either re-runs the symbolic phase (measured at 1.0 ms at 540 DOF — affordable at
  reduced sizes, ~1 s at 70 k — or forces a conservative union pattern covering every
  reachable patch. **Neither has been designed, let alone measured.**

**What is honestly open research, stated as such:**
1. Moving-patch re-partitioning under a once-built symbolic factorization.
2. IPC barriers on a reduced bulk. IPC's guarantees (non-penetration, feasible line
   search) are stated over full nodal positions. Whether they survive projection onto a
   subspace — and what "feasible" means for `q` — is not settled in the literature we
   can rely on.
3. ⛔ **CLOSED by §2m (v2.16) as originally posed, and REPLACED by a harder
   question (§2n v2.17, §2o v2.18).** It read: "§4c's `‖q‖` gate does not detect
   *the indenter moved somewhere the ensemble never saw*." That was argued, never
   measured. Measured, `q`-space carries the position information plainly — what
   failed was the STATISTIC. A per-mode box is dominated by the thinnest retained
   axis, so it has no rank-independent threshold; the distance to the nearest
   training snapshot, normalised by ONE GLOBAL scale (the training cloud's
   radius), holds to `0.138 %` across a `7.1×` rank change and separates every
   position. ⇒ **use one global normaliser, not a per-mode one** (⛔ v2.15 wrote
   "a NORM over modes, never a MAX", which is provably wrong — `‖d‖₂ ≥ ‖d‖∞`).

   ⛔⛔ **The two follow-ups v2.16 named are now measured, and they reopen the item
   on a different axis.** (a) The HELD-OUT position exists (§2n): the same
   statistic spans `1.005×` across four ranks while the error spans `31.9×`, so
   its rank-independence — the property that made it a domain signal — is exactly
   what makes it blind to resolution. **Rank-independence and accuracy-sensitivity
   are mutually exclusive.** (b) DENSITY is TWO factors (§2o): ENSEMBLE SIZE beats
   GAP `2.13–2.70×` against `1.09–1.17×` at matched rank and compounds to `11.9×`
   through the rank ceiling, and every online candidate reads the GAP — the weaker
   one. ⇒ **What is open is no longer "does `q`-space carry position information"
   (it does) but "what reads ACCURACY at runtime without the element sweep ECSW
   removes".** One held-out point and one 2-factor cell are still one of each.

**Consequence for sequencing**: R1 is deliberately **no contact** (§7). The cheap
kill-or-confirm must not be entangled with the part that is open research, or a
failure will be uninterpretable.

---

## 6. Differentiability through the reduced model

Threaded in from R1, not retrofitted — the brief is right that this is the
retrofit-hostile piece.

**The mechanism is the same IFT the crate already uses.** The full model solves
`A·λ = g_free` at the converged `x_final` and contracts against `∂r/∂θ`
(`factor_at_position` + `solve_free_and_scatter`). The reduced model solves the same
thing on the reduced tangent `A_r = ΦᵀAΦ` (`r × r`, dense), with:

```
∂r_r/∂θ = Φᵀ (∂r/∂θ)          (weighted by the ECSW element subset)
```

Two extra derivative paths appear, and both must be decided at R1, not later:

1. **Through the basis `Φ`.** If `Φ` is treated as a **constant** (recomputed offline
   when the design changes), the reduced adjoint is a strict simplification of the full
   one and costs essentially nothing new. If `Φ` must be differentiated with respect to
   design parameters, the derivative of an SVD is required, and that is a materially
   harder object (degenerate singular values make it ill-defined). **Recommendation:
   `Φ` constant, with the basis's dependence on design parameters handled by declaring
   it part of the validity domain** — leave the domain, rebuild the basis. This is the
   defensible choice and it should be recorded as a decision, because reversing it later
   is expensive.
2. **Through the ECSW weights.** Same argument, same answer: constants, fixed at
   training, part of the domain.

**What it costs.** With both held constant: one `r × r` dense factorization per
gradient (negligible), plus the `Φᵀ(·)` projections, plus — the real cost — the
adjoint RHS still has to be assembled over the **ECSW element subset**, which is the
same subset the forward pass uses. So the differentiable path costs roughly what the
forward path costs, exactly as it does today. **The genuine cost is not runtime, it is
that the gradient's accuracy is now bounded by the hyper-reduction error, not by f64.**
That has to be measured against the oracle in the same way the forward error is (§4c),
and — connecting to §2c — it means gradient validation must stay on the f64 oracle
path, which is another reason not to touch it.

### ⚠ Amended by R1.2 (2026-08-11): the binding error term is not hyper-reduction

The paragraph above named the wrong bound, and R1.2 measured it before any
hyper-reduction existed. With `Φ` constant and every element still swept, the reduced
gradient's relative L2 error against the oracle's is **0.24–0.82 depending on the
objective**, at a step where the reduced *displacement* is accurate to 0.4–1.1 %. The
cause is neither hyper-reduction nor f64: it is that `Φ`, fitted to displacement
snapshots, does not span the **adjoint** field. Taking the reduced adjoint at the
oracle's own state changes the numbers in the fourth digit, so the reduced trajectory is
not implicated either. Full result and the three measurements that pin the diagnosis:
`docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md` §13.

Three consequences for this section:

1. **Decision 1 (`Φ` constant) stands, and is not what was falsified.** For a load
   parameter the dropped `dΦ/dθ` term is exactly zero — `Φ` depends on the training box,
   not on the θ being differentiated. R1.2's G1 gate confirms the constant-`Φ` IFT
   algebra to 2.85e-8, well inside the crate's 1e-5 gradcheck.
2. **The validity domain gains an axis this section did not anticipate.** It must
   constrain *what is differentiated*, not only where the model is evaluated: the same
   reduced model gives a 0.972-cosine gradient on an objective resembling its training
   loads and a 0.692-cosine one on an objective loading directions it never saw. A model
   qualified for its parameter box is not thereby qualified for an arbitrary objective on
   it.
3. **The lead is goal-oriented bases, and R1.3 has now MEASURED it.** Enriching `Φ` with
   adjoint snapshots attacks the measured cause directly, and it works: at `r = 40` an
   enriched basis reaches 0.0767 gradient error at 0.9972 cosine (0.0848 / 0.9965 on half
   the enrichment budget), beating plain POD's `r = 104` (0.101 / 0.9952) while running
   ~1.45x the oracle instead of 0.54x — more accurate and 2.7x faster.

   **Conditional**: the declared objective family must itself be low-dimensional. Smooth
   face weightings are; point probes are not (a Green's function at one node is nearly
   independent of the next), and enriching with an incompressible family spends modes for
   nothing while displacing forward content — measured as a clean subtraction at 3.7x of
   the forward accuracy.
   Plan §14 carries the numbers, the controls, and the offline spectrum check that tells
   you which case you are in *before* building a basis. R3 was already redirected once —
   away from modal derivatives, by R1.0's finding that rotation is not the bottleneck —
   and this makes the basis recipe goal-oriented from the start rather than plain POD.

   ⚠ Raising `r` is **not** the alternative. R1.3's sweep measured break-even between
   `r = 40` and `r = 80`: at the plan's rank ceiling the reduced model runs at half the
   oracle's speed, so the ranks that fix the gradient are the ranks where reduction has
   already stopped paying. That is an R1 statement — hyper-reduction moves the
   break-even, which is precisely R3's job.

---

## 7. The staged ladder

**Constraints that hold at every rung**: L0 tier rules (dependency budget — `sim-soft`
sits at 110 against its recorded 200 override; banned-crate list applies in full), **no
C toolchain**, wasm32 must build, `cargo xtask grade sim-soft` stays A. Note that
`wgpu` is *banned* in L0 and reaches `sim-soft` only through the existing
`tier_up_features = { gpu-probe = "L0-io" }` mechanism — any GPU rung must use that
same door, under its own feature, and must not enter the default build.

| rung | scope | gate | why here |
|---|---|---|---|
| **R0** ✅ **DONE** (`e77023c7`, `43b198a2`) | **Full-order assembly lever.** Replace the per-iteration `BTreeMap` rebuild in `assemble_free_hessian_triplets` with a pattern-indexed value buffer built once at construction. No algorithm change. | Byte-identity of the assembled triplets against the current path (the `feedback_float_refactor_byte_identity` recipe), plus a measured ms/iteration delta on the §2a fixtures. | §2d.2. Establishes the **honest baseline** the reduction is measured against. Cheap, self-contained, and a win regardless of whether anything downstream ships. |
| **R1** ✅ **DONE AS SCOPED** (`#744`, `#745`, R1.2) — ⚠ **v2.14: every R1 gate was posed on a FIXED contact configuration.** §2l measures the basis failing to generalise across contact POSITIONS (`14.8–109 %` over the seven extrapolation arms that complete, flat in rank), which none of R1.0–R1.3 asked. Complete, with the scope that excluded the question deciding R3. | **Linear subspace, no contact, no coupling.** POD basis from full-order snapshots on the `cantilever` fixture at 3 000 free DOF; reduced Newton with a dense `r × r` direct solve; `Φ` and quadrature both handled naively (full element sweep — **no hyper-reduction yet**). Differentiable path wired at the same time (§6, `Φ` constant). | Projection error vs the oracle < 1 % in tip displacement over the training trajectory; reduced gradient matches the oracle's to the crate's existing gradcheck tolerance. ⚠ **That second clause was wrong and was amended before R1.2 was built** — it asks two different functions to agree to 5 digits when their states already differ in the third. Split into a gradcheck-tolerance kill gate on the reduced model's *own* derivative and a measured comparison against the oracle; see the plan's §5/§7 and §13. **Wall time is explicitly NOT gated at R1** — without hyper-reduction it will not be faster, and pretending otherwise would corrupt the signal. | **The cheap kill-or-confirm.** It answers the one question that decides everything downstream: *does a low-dimensional subspace represent this material's deformation at all?* Fixture already exists; no new physics. |
| **R2** | **Precision decision.** Measure a full-f32 forward path on the reduced system (`r × r` is small enough to port by hand without touching the 1 396-`f64` production surface), and decide residual-in-f64-on-CPU vs compensated-summation-in-f32. | Reduced-model f32 forward drift and gradient drift vs the f64 reduced model, on R1's fixture; explicit go/no-go on whether the residual can live in f32. | §2c. Must precede any GPU work; deciding it after a shader exists means writing the shader twice. |
| **R3** **§2i brackets its Amdahl ceiling at `20.2–20.5×`–`≳32×`** (two runs) — clear of both its own `10×` floor and the budget's `13.5–15.8×` on either bound. ✅ v2.13: the reduced path HAS now run with contact — §2k measures `I = 11.3–12.3 ms` on IPC 18 750, margin `1.36–1.47×` under `Inertial` and a FAIL of `0.92–0.98×` under `PreviousState`. ⚠ Size-dependent — `~4.5×` at 5 202, and extrapolating the two terms of `I` separately, gone by ~`26 k` free DOF. Gate is **`I ≤ 16.7 ms`** on §2h's reference box (§2j — restated in v2.8 from `≥10×`, which was a ratio over a moving baseline). | **Hyper-reduction (ECSW) + the validity domain.** NNLS training over R1's snapshots; `ReducedValidityDomain` — ⛔ v2.18: rung 1 NARROWED the candidates to one and then **disqualified that one twice**; there is no gate design. §2m: `snapshot_distance` (nearest training snapshot, ONE GLOBAL normaliser, NOT per mode) is the only candidate surviving a per-step test, at `~2×` on a FITTED threshold. §2n supplies the held-out position §2m lacked and it fails there — `1.005×` of signal across `31.9×` of error, because **rank-independence and accuracy-sensitivity are mutually exclusive**. §2o: it is blind to ENSEMBLE SIZE too, and size is the dominant factor — doubling it at fixed gap buys `2.13–2.70×` where doubling the gap at fixed size costs `1.09–1.17×` ⇒ the domain must state CARDINALITY and no candidate can check it. `‖r_free‖ / tol` reads accuracy but is NOT available here: it is the full element sweep ECSW removes. ✅ v2.20: rung 2's prerequisite is DISCHARGED — `ReducedStep::full_residual` now exposes the converged `r_free` VECTOR (`full_residual_norm()` derives from it), so a spatially resolved indicator can be BUILT. ⛔ And the vector kills PER-MODE on arrival: `Φᵀr_free` is what the solve drove below `tol`, largest component measured at `2.6e-13` against a `1.8e-2` per-DOF max in `r_free`, `11` orders down — a per-mode indicator needs directions `Φ` does not retain. What survives is spatial: the top decile of DOFs carries `90.5 %` of `‖r_free‖₁`. ⚠ Buildable, not affordable: `r_free` is still the element sweep ECSW removes, so those candidates are research instruments, not R3 gates. Plus the three error measures of §4c. | Measured speedup vs §2a's baseline (post-R0), with the three §4c errors reported alongside. Domain gate demonstrated to fire on an out-of-domain trajectory. | This is where the frame-budget win actually arrives. Also where the "smooth and wrong" failure mode is defended against. |
| **R4** | **Hybrid domain decomposition, FIXED contact patch.** Full DOF under a stationary indenter, reduced bulk, on the `dynamic_indentation` geometry. | End-to-end reaction force vs the oracle, in the same band `bonded_layer_indentation` already asserts. | §5. Fixed patch first, because it isolates the coupling condition from the re-partitioning problem. |
| **R5** | **Moving patch.** Re-partitioning under a once-built symbolic factorization, or a conservative union pattern. | Sliding-contact trajectory vs the oracle. | §5's open-research item. **Explicitly gated on R4 succeeding**; if R4 fails, this is not attempted. |
| **R6** | **Rigid↔soft coupling.** | — | **Last, deliberately.** Per the brief, and it is the right call: the keystone coupling is itself the platform's hardest open problem (`MISSION.md` §2), and stacking it on an unsolved real-time reduced path would make any failure uninterpretable. |

**Kill conditions, stated in advance** (so they cannot be renegotiated after the
fact): if R1 shows the projection error is not small at any `r` worth having, the MOR
path is wrong for this material class and the recon is falsified — the correct
response is to revert to R0's win and re-recon, not to widen the basis until the
number looks acceptable. **R3's own gate was RESTATED in v2.8 — see §2j.** It read
*"if R3's measured speedup over §2a's (post-R0) baseline is under ~10×, reduction is
not paying for its complexity"*, which is a ratio against a baseline every rung here
is chartered to improve: succeeding at R1's own line items made R3 fail a gate it
otherwise passed, while simultaneously making R3 less necessary. It now reads: **R3
clears iff the frame's IRREDUCIBLE time fits the budget, `I ≤ 16.7 ms`, on the
fixture in question** — which is what `C ≥ R` reduces to. The `~10×` survives as a
complexity heuristic, re-derived against the baseline of the day and never carried.

---

## 8. MISSION.md tension — a decision for the head engineer

**The tension is real and I am not going to paper over it.**

`MISSION.md` §"How we sequence: quality and ceiling, not speed" says, verbatim:

> **We do not order this roadmap by how fast we can close the loop.** Demo velocity is
> the wrong objective. We order it by *quality* and by *ceiling* — building the
> deepest, ceiling-determining foundations first and to validated, high-fidelity
> standards.

and sequences the keystone soft↔rigid coupling second, "never deferred for a faster
demonstration."

A real-time path is, on its face, exactly the thing that sentence was written to
resist. Three readings are available and they lead to different work:

**(a) Real time is demo velocity, and the sentence forbids it.** The honest
consequence: this recon is premature, R0 is still worth doing (it is a pure quality
win), and R1+ waits behind the keystone and system-ID. Nothing is lost but time.

**(b) Real time is a ceiling property, not a velocity property.** The argument: the
capstone is an **RL-controlled** exoskeleton. RL needs sample throughput; §2b's measured
ceiling — **38 concurrent full-order envs on a 24 GB box at 70 644 free DOF** (corrected
2026-08-22 from "~20"; the reduced path raises it to ~395) — is a hard ceiling on that, and
`PERF_BASELINE.md` already shows the GPU path only pays at batch ≥ 1024. On this
reading, an interactive/batchable soft path is *ceiling-determining* for step 5 of the
mission's own thesis, and belongs in the sequence on the mission's own terms — not as a
demo.

**(c) Split the difference.** R0 and R1 now (they are cheap, and R1 answers a
question that is *itself* a fidelity question — "is this material's deformation
low-dimensional?" is a statement about the physics, not about frame rates), and gate
R3+ behind the keystone.

**My recommendation is (c)**, and I want to be clear about why rather than hedging:
R1's kill-or-confirm produces a fact about the material model that is worth knowing
regardless of whether real time is ever pursued, and it costs one fixture that already
exists. R3 onward is where genuine roadmap displacement starts, and that is the right
place to put the gate.

### 8a. Where the collision actually sits

Precision matters here, because the three rungs are affected very differently:

- **R0 does not collide at all.** It is a pure speedup with no accuracy trade, gated on
  byte-identity. It needs no authority from anyone.
- **R1 does not collide on fidelity.** "Is this material's deformation
  low-dimensional?" is a fact about the physics, not about frame rates. Only the
  scheduling objection applies, and it costs one fixture that already exists.
- **R3 is the collision point.** It is the first rung at which the repo would hold a
  solver whose answer is knowingly approximate *and consumable*, which is the moment
  `MISSION.md` §1's "the twin's fidelity caps everything downstream" and §4's "built
  only on a substrate we already trust" actually bite.
- **R4/R5 collide on attention**, not on fidelity — §2's "never deferred for a faster
  demonstration" names exactly that failure mode, and hybrid-DD contact would compete
  with the keystone for it.

One reading that looks like a collision and is not: "demo velocity is the wrong
objective" does not by itself forbid this work. `MISSION.md` already claims GPU
batching as shipped capability ("analytic derivatives, GPU, batching (`sim/L0/*`)")
and lists "GPU-batched, differentiable environments" in its breadth section. Real time
was an **unlisted** item, not a forbidden one.

### 8b. Resolution — MISSION.md amended 2026-08-10

Reading (b)/(c) taken; `MISSION.md` now carries an **"On speed as a ceiling"** clause
after the five-step sequence. It authorises a speed target *only* where it is
ceiling-determining for the capstone **and the ceiling has been measured rather than
argued**, and binds it to three conditions: a stated, runtime-gated validity domain;
the full-order reference solver as the unchanged instrument of record; and explicit
domain inheritance by any consumer of approximate results. It also restates that step
2 (the keystone) is not deferred, so the clause cannot be read as unlocking that.
Speed pursued for a demonstration still fails the test.

Consequence for this recon: **§7's ladder is authorised in principle, and the three
conditions are now mission-level requirements rather than recon-level
recommendations.** §4c's validity-domain design and §6's "`Φ` and the ECSW weights are
constants, and the basis's parameter dependence is part of the domain" decision are
what discharge condition 1; §3c's "the CPU f64 path stays exactly as it is" discharges
condition 2; condition 3 is a new obligation on any future consumer and is the reason
R3's gate must fire online rather than in a docstring.

---

## 9. Reproduction, and what was reverted

**No production code was written for Phase 1. No dependency was added.** Four
temporary instrumentation edits and three temporary test files existed during the
session and are reverted. (⚠ §2f, added in v2.1, is a later measurement that did NOT
follow this pattern — see the header.)

### 9a. Phase-1's own verification and reverted edits

Verified after the revert: `git status` shows only this doc as
untracked, and `cargo xtask grade sim-soft` returns **A on all seven automated
criteria** (Documentation, Clippy, Safety, Dependencies, Layer Integrity, WASM Compat;
Coverage is `—` under the crate's recorded `integration-only` profile, and API Design
is the standing manual-review marker).

Temporary edits (all reverted):

| file | edit |
|---|---|
| `src/scratch_profile.rs` | **new file** — 8 atomic ns/count timing slots |
| `src/lib.rs` | `pub mod scratch_profile;` |
| `src/solver/backward_euler/assembly.rs` | timers around internal-force assembly, tangent assembly, and the three contact sub-phases |
| `src/solver/backward_euler/factor.rs` | timers splitting numeric factor from triangular solve; `FactorInner::Llt32` + a `CF_SCRATCH_F32=1` env gate routing the numeric factor and all solves through f32 |
| `src/solver/backward_euler/ordering.rs` | `OrderedLlt32` — f32 numeric Cholesky over the same symbolic factor |
| `src/solver/backward_euler/construct.rs` | timer around `build_symbolic_factors` |
| `src/solver/backward_euler/newton.rs` | `CF_SCRATCH_CANCEL=1` per-iteration `‖r‖`/`‖f_int‖` probe |
| `src/solver/backward_euler/mod.rs` | `scratch_factor_nnz()` / `scratch_n_free()` accessors |
| `tests/zz_scratch_realtime.rs` | the `cantilever` / `block_sag` harness, one case per process via `CF_CASE` |
| `tests/zz_scratch_f32.rs` | in-process f64-vs-f32 forward + gradient comparison |
| `tests/zz_scratch_contact.rs` | the `dynamic_indentation` IPC harness |

The raw logs are in the session scratchpad (`det.log`, `perproc.log`, `f32b.log`,
`contact2.log`, `cancel.log`). To reproduce, re-apply the instrumentation from the
table above — it is mechanical — and drive it with the per-case env vars. If this
measurement is to be repeated regularly, the right move is **not** to re-apply
scratch patches but to land the timing slots properly behind a feature flag; that is a
small, separate PR and is not proposed here.

### 9b. Reproducing §2g's selector measurement (v2.2)

§2g's code was written, measured and reverted, and the revert was squashed, so
**nothing in git reproduces it**. To re-run, re-apply — it is mechanical:

| where | edit |
|---|---|
| `config.rs` | add `Adaptive` to `InitialGuess`. ⚠ `#[non_exhaustive]` makes this additive DOWNSTREAM only — in-crate there are TWO exhaustive matches to extend, `newton.rs`'s placement and `reduced/newton.rs`'s `let mut q = match …`; the compiler finds both. ⚠ It does NOT find the four sites below |
| `newton.rs` | split the existing per-variant placement into `place_guess(variant, …)`, and REWRITE `apply_initial_guess` (it already exists) as a dispatcher. Its signature must gain `x_prev`, which `assemble_global_int_force` needs. For `Adaptive`: per candidate **`x_curr.copy_from_slice(x_prev)` FIRST**, then `place_guess`, assemble `f_int`, `residual_into`, `free_residual_norm`; keep the smallest FINITE; then reset and re-place the winner. Hoist `f_int`/`r_full` above the call so candidates reuse them |
| `newton.rs` | widen `free_residual_norm` to `pub(super)` if the unit tests are wanted |
| `reduced/newton.rs` | add `Adaptive` to `assert_supported_initial_guess`'s refusal AND to the `let mut q = match …` arm (an `unreachable!` beside `InertialWithLoad`) — the reduced loop's own norm is the PROJECTED `‖Φᵀr‖`, a different quantity |
| `tests/predictor_spike.rs` | extend `ARMS` to four and widen each `solvers` array literal |
| `src/.../tests.rs` | ⚠ two 3-element array literals compile CLEANLY and silently skip a fourth variant — the candidate list in `initial_guess_variants_produce_the_documented_first_iterate` (zipped against a 3-element `expected`) and `guess_changes_path_not_root` |
| `helpers.rs` | ⚠ `initial_guess_stall_hint`'s `matches!(…, PreviousState)` early-return also compiles clean, then emits "the first iterate is extrapolated off x_prev" on every stall — false whenever the selector chose `PreviousState` |
| `tests/predictor_spike.rs` | ⚠ `run_arms` has two hardcoded `3`s (`vec![x0.to_vec(); 3]`, `vec![vec![0.0; n_dof]; 3]`) that the compiler does NOT catch: `out` is sized from `ARMS` while `x`/`v` stay at 3, so step 0 panics on `x[3]` |

⚠⚠ **Keep the non-finite filter: the ordering does NOT make it optional.** An
earlier reading held that `PreviousState` going first guarantees a finite running
best, because its residual is finite whenever the step-start validity check
passed. That is false under a moving contact barrier: the check tests element `F`
(inversion, max stretch), not barrier distance, and `replace_contact` advances the
collider between steps — so `x_prev`, converged against the OLD pose, can penetrate
the NEW one and return `‖r‖ = NaN`. The running best is then NaN, every later
`cand < best` is false, and the selector silently degenerates to always-
`PreviousState`: the killed behaviour, with no error and no NaN in the output.

⚠⚠ **The `copy_from_slice` reset is not optional and its absence is SILENT.**
Placement is in-place and reads its own base (`x_curr[i] = dt.mul_add(v_prev[i],
x_curr[i])`), so looping without resetting gives the third candidate
`x_prev + 2Δt·v + Δt²f/m` instead of `x̂ + Δt²f/m` — no error, no panic, just a
wrong guess and a wrong ranking.

⚠ Evaluate `PreviousState` first regardless — it is the one point already validated
this step, so a cheap tie should not move off it.

⚠ A discriminating unit fixture needs the scene's candidate ordering to be
non-default. On the 1-tet skeleton at θ = 1 the residuals are `PreviousState` 0.914
< `Inertial` 6.44 < `InertialWithLoad` 20.0, so a selector hard-wired to
`PreviousState` is indistinguishable from a working one; at θ = 1000 the order flips
to `Inertial` 992.6 < `PreviousState` 999.9 and the choice becomes observable.

⚠ **Precision on "no committed tree":** the implementation WAS committed, as
`15c8f27a`, and then orphaned by the squash. It is on no branch, was never pushed,
and will be garbage-collected — so it is recoverable on the machine that made it
until then, and by nobody else ever. The recipe above is the durable record.


---

## 10. Open risks

0. ⛔ **THE BASIS DOES NOT GENERALISE ACROSS CONTACT POSITIONS, and rank does not
   fix it (§2l, v2.14).** `14.8–109 %` error one to two patch-radii outside the
   training hull, flat in rank, and **silent in 7 of 8 arms** — they converge and
   do not penetrate; the eighth stalls and is announced. Listed first because it is the only measured result that can
   invalidate R3 rather than resize it: hyper-reducing a basis that is wrong off
   its training data makes a wrong answer cheaper. The response is §4c's gate
   (now mandatory) plus, structurally, §5's hybrid domain decomposition — which
   is sketched and unmeasured. ⚠ Interpolation *inside* the hull is usable
   (`1.9e-3` on a `0.5a` grid), so the risk is a domain-coverage and gating
   problem, not a dead end.

   ⚠ **The GATING half is NARROWED, not discharged (§2m, v2.16 — v2.15 claimed
   discharged and was wrong).** Of four oracle-free candidates, one survives a
   per-step test: distance to the nearest training snapshot reads `5.062e-1 …
   5.069e-1` at interpolation (`0.138 %` spread over a `7.1×` rank change),
   against `≤ 3.6e-3` in-sample and `≥ 1.165` extrapolating — a `2.30×` worst-step
   and `1.94×` median-step window. The other three fail measurably: the per-mode
   box at every tolerance, the vertex-novelty signal as a trajectory-maximum
   artifact (median exactly zero at `+1.50a`), and the residual proxy per-step
   (inverted, `0.737×`) and under ECSW (`r_free` is what hyper-reduction removes).
   ⇒ §4c's gate is NOT ready to build. ⛔ **And §2n (v2.17) supplied the held-out
   position: the surviving candidate's `~2×` does NOT survive it.** Within one
   ensemble the error spans `31.9×` while that signal spans `1.005×` — its
   rank-independence, which §2m named as its signature, is exactly what makes it
   blind to resolution. ⇒ a DOMAIN signal and an ERROR signal are provably
   different quantities and §4c needs both; neither alone gates. ⚠ The
   DOMAIN-COVERAGE half is answered in part by §2n/§2o (v2.18): "density" is TWO
   factors, ENSEMBLE SIZE dominates GAP `2.1–2.7×` against `1.1×` at matched rank
   and compounds through the rank ceiling to `11.9×`, and **both online candidates
   are blind to size** — so the domain must state CARDINALITY and neither candidate
   can check it. ⚠ One held-out point is still one point.
1. **Every timing number was taken on a contended box (§1a).** Absolute times are
   upper bounds. The *shapes* (scaling exponents, phase shares, crossover point) are
   robust across repeats; the absolutes are not benchmark-grade. Re-take §2a on an idle
   machine before quoting any figure externally.
2. **The `≈ 1 500 free DOF at 60 Hz` headline is an interpolation**, not a measured
   point — it is derived from the 540 → 3 000 DOF pair at the measured `n^1.51` low-end
   exponent, using the IPC fixture's 6-iteration count. The bracketing measurements
   (540 DOF at **0.47×** budget — it *fits*, with 2× headroom; 3 000 DOF at 12.1×) *are*
   direct. Treat 1 500 as an order-of-magnitude statement.
   ⚠ **This risk carried the pre-R0 figures — 800 / `n^1.38` / 1.37× / 28× — from v1.3
   to 2026-08-22.** v1.3 re-measured them and updated §2a, the verdict and the header
   caveat, but missed §10 and §5. The doc therefore stated, in two places at once, that
   the 540-DOF anchor **missed** the budget by 1.37× and that it **fitted** at 0.47× —
   and this is the paragraph telling readers which figure is safe to quote.
3. ~~**The `cantilever` at 36 300 free DOF does not converge.**~~ **RESOLVED
   2026-08-11 — this risk was mine, not the solver's.** The failing harness capped
   Newton at 60 iterations; the case needs 65. Every size converges with a generous
   cap. Full diagnosis in §3d; the residual real finding (iteration count grows with
   refinement) moved to §3b.4. Left visible rather than deleted because it was ranked
   above R1 on the strength of the error.
4. **f32 gradient drift (9.3e-4) exceeds the existing gradcheck tolerance (5 digits).**
   Any f32 path needs its own tolerance policy, decided deliberately rather than by
   loosening an existing assertion.
5. **§4b's ECSW recommendation rests on a literature claim about achievable speedup
   that this recon did not measure.** R3's gate exists precisely to hold it to account.
   ⚠ Its form CHANGED in v2.8: the response is no longer "< ~10× over R0" — that was a
   ratio over a baseline the rest of the ladder keeps moving — but **`I > 16.7 ms`**,
   the frame's irreducible time failing to fit the budget (§2j). ★ §2i's measured
   ceiling of `20.2–20.5×` is already the strongest evidence either way, and it is
   an *upper bound on ECSW's reach here*, not a confirmation of the literature claim.
   ✅ **v2.13**: §2k answers the gate on the right fixture — `I = 11.3–12.3 ms`,
   margin `1.36–1.47×`. It CLEARS, with `~4×` less room than the contact-free
   fixture implied, and only under the `Inertial` predictor.
6. **§5's hybrid-DD arithmetic closes on paper** (a few hundred patch DOF + `r ≈
   30–100` lands inside the reachable ≈ 1 500 DOF). That is encouraging and it is not
   evidence. Nothing about the coupling condition or the moving patch has been
   measured or designed.
7. **The `BTreeMap` finding (§2d.2) is a profile attribution, not a proven fix.** R0's
   gate is a measured delta for exactly that reason; the recon does not claim the
   speedup in advance.

---

## 11. Cross-references

- `MISSION.md` §"How we sequence" — the tension of §8.
- `sim/L0/soft/Cargo.toml` — the recorded rayon / nested-dissection benchmarks
  (uncontended), the 200-dep override, and `tier_up_features = { gpu-probe = "L0-io" }`.
- `sim/L0/soft/tests/bonded_layer_indentation.rs` §Cost — the "measure parallel wins;
  do not derive them" rule this recon's §1a obeys, and the `STATIC_DT` note that
  motivated the new dynamic fixtures.
- `sim/L0/gpu-benches/PERF_BASELINE.md` — CPU-vs-GPU crossover (rayon CPU leads to
  n_env ≈ 1024; GPU never overtakes with contact) and its f32-GPU-vs-f64-CPU caveat.
- `sim/L0/gpu/src/shaders/*.wgsl` — 13 shaders, zero `f64` tokens (verified).
- `sim/L0/soft/src/solver/backward_euler/assembly.rs:393` — the per-iteration
  `BTreeMap` rebuild of §2d.2.
- `sim/L0/soft/src/solver/backward_euler/ordering.rs` — nested-dissection fill figures
  that §2b's fill-per-row growth reproduces.
- `sim/L0/soft/src/material/` — the `ValidityDomain` §4c mirrors.

## 12. Version history

- **v2.20 (2026-08-26)** — **§4c WRAP-UP: rung 2's API prerequisite, shipped.**
  Production code, deliberately — v2.19 closed rung 1 with zero production code
  and recorded that rung 2 could not START because `ReducedStep` handed back
  residual NORMS only. `ReducedStep::full_residual` now carries the converged
  `r_free` vector (`n_free` entries, `free_dof_indices()` order) and
  `full_residual_norm()` is a method derived from it rather than a field stored
  beside it, so the two cannot disagree. The vector is MOVED out of the
  convergence check that already assembled it: no extra allocation, no extra
  arithmetic. New debug-runnable gate
  `a_converged_step_hands_back_the_residual_it_converged_on` in
  `reduced_newton_trajectory.rs`, on a coarse `4x4x2` rig (`n_free = 150`,
  `r = 8`, 0.04 s); its three assertions were negative-controlled by three
  mutants — a stale iterate perturbed by one part in `1e9`, a zeroed vector, and
  a truncated one — each killed by a different assertion. ⛔ **A review pass then
  found the vector KILLS half of what it unblocked, and that three passing mutants
  had not noticed:** the write-up said the class was "per-mode or per-region", but
  `Φᵀr_free` is precisely what the solve drives below `tol`, so per-mode carries no
  signal at convergence — its largest component measured at `2.6e-13` against a
  `1.8e-2` per-DOF max in `r_free`, `11` orders down. What survives is spatial: the
  top decile of DOFs carries `90.5 %` of `‖r_free‖₁`. Both numbers are now printed
  by the gate. The same pass deleted a fourth assertion, `projected_residual_norm <
  TOL`, as VACUOUS — the solve constructs a step only from inside
  `if proj_norm < tol`, so it restated convergence rather than testing it, and it
  was the one assertion no mutant had killed. ⚠ **This changes what rung 2 CAN be
  built from; it changes nothing about the two constraints.**
  `r_free` still costs the element sweep, so the per-mode / per-region class is a
  research instrument, not an R3 gate, and rung 2 stays research rather than
  implementation.

- **v2.19 (2026-08-25)** — **CONSISTENCY PASS over §2l–§2o and everything
  downstream of them. No new measurement; every figure re-derived from the run
  logs.** ⛔ **§4c's "rung 1 done" block had never been updated for §2n or §2o** —
  it still recommended `snapshot_distance` on the strength of the `0.138 %`
  rank-flatness that §2n showed to be the DISQUALIFIER, still carried the
  two-signal conclusion as withdrawn when §2n reinstates it, and still ended "one
  candidate standing". That is the block rung 2 would have been built from.
  Rewritten, with the impossibility and the affordability limit separated and the
  `ReducedStep`-exposes-only-a-NORM prerequisite named. ⛔ **§5 open item 3 was
  claimed answered by v2.18's own changelog and was never edited** — it still
  listed the held-out position and the density question as open. Rewritten.
  ⛔ **§7's R3 row** still read "rung 2 must first get a HELD-OUT position; there
  is none". ⛔ **`28–109 %` survived as a live range in §4c, §7's R1 row and §10
  risk 0**, which §2m's own superseded table names as a column misread as a
  bracket; the range over the eight extrapolation cells is `14.8–119.6 %`.
  ⛔ **"the failing arms converge and complete all 71 steps" is true of SEVEN of
  the eight** — `+2.00a` at `r=20` stalls at step 65, which §2l's own table marks
  DIVERGED two paragraphs above the sentence. Silent, not universally silent, and
  the honest bound is stronger for saying so. ⛔ **§2n asserted that its median
  comparison was sound in the same bullet that ruled cross-arm comparison
  unsound** — `4.117e-1` vs `4.271e-1` is a cross-arm comparison, as are the
  `[0.5069, 1.165)` containment and pre-registration item 3's linearity reading.
  All three relabelled as DIRECTIONS; the section's load-bearing claims are
  within-arm and untouched (`1.005×` / `1.023×` of signal against `31.9×` of
  error). ⚠ §2n's wall time quoted run 1 (`139.7 s`) while every signal figure
  came from run 2 (`140.8 s`, 16 arm rows identical). ⚠ "142 snapshots retain 64
  modes" is B's ceiling; C's 142 retain 65. ⚠ The test file's module doc still
  announced "the other TWO studies" and "the FIRST of three"; there are six.

  ★★ **A SECOND round found 8 defects in this pass's own fixes**, which is the
  expected place for them. The load-bearing ones: the new module doc claimed all
  six studies share one fixture (only the last FOUR share `GEN_A_OVER_CELL` and
  `BASIS_RANKS`; the producer check runs the same mesh over its own rank ladder,
  and the timing run has no rank ladder at all — a fixed `r=40` over two mesh
  sizes, one of which is `a/cell = 3`); "the only cell above `109 %`"
  was wrong at the rounding boundary — `+2.00a` at `r=142` reads `109.3 %`, so the
  diverged arm is the HIGHEST cell (`119.6 %`), not the only one; the §4c rewrite
  quoted the cross-ensemble `~1.1×` as a NUMBER in the same pass that relabelled
  it a direction everywhere else; "two impossibility results" overstated — only
  the domain-signal one is a proof, the ECSW one is affordability; and "§2o leans
  on no cross-ensemble number" was false — §2o carries one and LABELS it, which is
  the form §2n should have used.

  ★★ **ROUND 3 found 4 more, in round 2's fixes** — and one had the CERTAINTY
  backwards: "only the domain-signal result is a proof" is inverted. The ECSW
  constraint is definitional (`‖r_free‖` needs the sweep ECSW removes; no
  measurement can overturn it); the rank-independence result is a definitional
  ARGUMENT on a MEASURED premise (`1.005×`, one statistic, one held-out position).
  Also: "C's training never sweeps the scored patch" overstated the coverage
  control — C's novelty maxes at `0.250`, so it still recognises `75 %` of the
  scored active set; it leaves a HOLE, which is what §2o said.

  ★★ **ROUND 4 found 3, and the worst PREDATES this branch.** The producer
  check's module doc listed `r ∈ {10, 20, 40, 60}` while `R_LADDER` sweeps
  `{2, 4, 10, 20, 40, 60}` — dropping the two rungs its own const doc calls the
  two-sided half. ⛔ And three ratios in its "Measured" prose (`1.7×`, `~2 700×`,
  `~54 000×`) do NOT reproduce from the table three lines above them, which gives
  `2.37×`, `3.79e3×`, `7.62e4×`; every published figure is `0.71×` of the table's.
  Withdrawn pending the 2026-08-24 log rather than overwritten, since a
  correction computed from a 3-s.f. table is the failure this document keeps
  naming. The finding's DIRECTION is unaffected.

  ⇒ **the fix is the least-reviewed text** — four rounds on this branch have now
  each found defects in the previous round's fixes (`9728a9a9`, `e3a55587`,
  `1222a698`, `12a13d36`). ⚠ Round 4 also shows the corollary: once the new text
  converges, the rounds start finding OLD text instead.

- **v2.18 (2026-08-25)** — **§2o, new: "density" is TWO factors and the online
  signals measure the WEAKER one.** A 2-factor design at a fixed held-out point,
  zero new trajectories: at matched rank, doubling the ENSEMBLE at fixed gap buys
  `2.13–2.70×` while doubling the GAP at fixed size costs only `1.09–1.17×`. ★★
  Size COMPOUNDS by setting the rank ceiling (284 snapshots → 116 modes, 142 → 64),
  so at their own ceilings the larger ensemble wins `11.9×`. ⇒ **second strike on
  §2m's survivor:** two ensembles at the SAME gap differ `2.1–2.7×` in error while
  `active_novelty` reads `0.000` for both and `snapshot_distance` reads ~`1.1×`
  apart — **neither can express ensemble size**, so a validity domain must state
  CARDINALITY and the online candidate cannot check it. ✅ Positive control:
  `active_novelty` DOES fire (`0.167–0.250`) on the ensemble that leaves a genuine
  hole, so it measures COVERAGE correctly — coverage is just not what dominates.
  §5 item 3 and §10 risk 0's domain-coverage half answered in part.
- **v2.17 (2026-08-25)** — **§2n, new: the first HELD-OUT position, and it
  disqualifies §2m's survivor.** Leave-one-out (refit on `[-1,-0.5,+0.5,+1]a`,
  score at `+0.00a`) plus an off-lattice extrapolation control. ★★★ **Within one
  ensemble across four ranks the error spans `31.9×` while `snapshot_distance`
  spans `1.005×`** — its single-ensemble threshold margins are `1.000–1.002×`,
  noise, against `2.883×` for the residual proxy. ⇒ **rank-independence and
  accuracy-sensitivity are mutually exclusive**, so the property §2m celebrated is
  the one that disqualifies it. ⇒ **The two-signal conclusion v2.16 withdrew
  RETURNS**, now on a held-out position rather than a confounded position
  partition. ⛔ §2m's `[0.5069, 1.165)` interval does not survive: the held-out
  arms read `0.9704–0.9756`, INSIDE it, carrying a `32×` error spread at one
  value. ★★★ **Matched gap and matched lattice status, interpolating vs
  extrapolating: at `r=20` extrapolation is BETTER (`0.84×`) and at `r=40` they are
  indistinguishable (`1.05×`), diverging to `24×` only at top rank** — being inside
  the hull does not make an answer better at a given rank, it makes it IMPROVABLE
  by rank (interp falls `31.9×`, extrap `1.12×`), which sharpens §2l. ✅ Lattice
  control: `+1.75a` falls inside the `+1.50a`/`+2.00a` bracket at all four ranks,
  so the mesh confound against `+0.25a` is largely defused. ✅ `active_novelty`
  reads `0.000` at the held-out point at every rank while the error runs
  `1.2–37 %` — the pre-registered fail-open, confirmed.
- **v2.16 (2026-08-25)** — **§2m REWRITTEN after a cold re-derivation; v2.15's
  headline was an artifact.** v2.15 reported a trajectory MAXIMUM for a question
  about a per-step check, and that single choice inverted three of its four
  verdicts. Rebuilt the harness to keep every step, print both quantiles,
  `active_novelty`'s denominator, the singular-value spectrum, and compute the
  gate table itself. ⛔ **The gate table is the correction:** asking "is there ONE
  threshold with `accept ⟺ relL2 ≤ τ`" — the question a gate is actually asked,
  which v2.15 never asked — the error proxy decides at 7 of 9 tolerances on the
  worst step while BOTH geometric candidates FAIL at `1e-2`. ⇒ **"the gate needs
  TWO signals" is WITHDRAWN as a gating claim**; it is an argument about
  DIAGNOSIS. ⛔ **`active_novelty`'s `∞` margin was a maximum-only artifact** —
  median exactly `0.000` at `+1.50a` at all four ranks, over an active set of
  10–18 vertices that SHRINKS out of domain (`21 → 4`), so the fraction rises
  partly because contact is disappearing. ⛔ **`‖r_free‖/tol` is not free at R3**
  (it is the element sweep ECSW removes) and its separation INVERTS per-step
  (`0.737×`). ✅ **One candidate survives both passes:** `snapshot_distance`,
  `2.30×` worst step and `1.94×` median. ⛔ **"a NORM over modes, never a MAX" was
  provably wrong** (`‖d‖₂ ≥ ‖d‖∞`); the per-mode NORMALISER was the fault, and §7
  had already recorded the wrong rule as rung 2's design. ⛔ Further corrections:
  "four significant figures" is TWO (`0.138 %` spread); `28–109 %` is the `r=142`
  column and the range is `14.8–119.6 %`; `≥1.17` is `1.165`; "byte-identical
  output" holds for the measurement ROWS, not the files; pre-registration item 5
  was marked HELD although its antecedent was falsified. ★ New caveats: **no
  held-out position exists** so every margin is fitted; `+0.25a` is the ONLY
  offset off the mesh lattice; the DOMAIN/ERROR taxonomy is not falsifiable on
  this fixture. ⇒ §4c's gate is NOT ready to build; §10 risk 0 and §7's R3 row
  restated from "discharged" to "narrowed".
- **v2.15 (2026-08-25)** — §2m added: four oracle-free candidates scored on §2l's
  matrix, `gap_dev` disqualified as oracle-dependent, §5's open item 3 refuted
  (`q`-space does carry position information). ⚠ **Its headline findings and its
  conclusion are superseded by v2.16** — see the table at the end of §2m. What
  survives: the candidate set, the `gap_dev` retraction, and the §5 refutation.
- **v2.14 (2026-08-24)** — **§2l, new: the basis does NOT generalise across
  contact POSITIONS.** One basis trained on five lateral indenter offsets
  (`±1a`), scored at four positions against each one's own oracle: in-sample
  `2.6e-6`, interpolation `1.9e-3`, extrapolation `0.28` and `1.09` — ★★★ **flat
  in rank**, the classic advection-like POD failure, and NOT the free edge (the
  nearer extrapolation keeps `1.5a` of clearance and fails alike). ★★★ Failure is
  **SILENT**: the bad arms converge, complete, and do not penetrate. ★★ `gap_dev`
  detects it anyway (`0.161–0.583 d̂` vs `3.1e-11`), which is not what it was
  built for, and (⛔ WRONG, corrected in v2.15: `gap_dev` differences against the
  ORACLE, so it can never gate) was called a candidate online signal for §4c — ⚠ as an ERROR indicator
  rather than an in-domain one, needing its own threshold, since at `r = 20` the
  in-sample arm also clears the regression figure. ⇒ **`ReducedValidityDomain`
  revived as a CORRECTNESS prerequisite** (v2.7's retraction of it as a
  PERFORMANCE prerequisite stands); §7's R1 row qualified to "DONE AS SCOPED",
  since every R1 gate was posed on a fixed contact configuration; §10 carries it
  as risk 0, the only measured result that can invalidate R3 rather than resize
  it.
- **v2.13 (2026-08-24)** — **§2k, new: `I` measured on IPC 18 750, the fixture the
  requirement is stated for.** §2j had named this as the one measurement R3 was
  waiting on, because `I` had only ever been read on a contact-free cantilever
  where `61 %` of it was `contact` marshalling on a `NullContact` scene. **R3
  CLEARS at `1.36–1.47×`, against the `5.60×` that fixture reports** — the old
  number was measured somewhere the requirement does not live and overstated the
  margin `~4×`. ★★ Two things it changed that were not on the list: the PREDICTOR
  is now load-bearing (`PreviousState` FAILS at `0.92–0.98×` in all eight runs,
  `Inertial` passes), and the reduced path's iteration penalty GROWS with size
  while the full-order path's does not (reduced/full `1.15× → 1.50×`) — which is
  what moved the answer, not the contact cost, which came in as projected
  (`1.26–1.75 ms/iteration` against `1.43` extrapolated). The pre-registered
  failure mode — a projected Armijo tunnelling through a finite-energy barrier —
  is real as a mechanism (`‖r_free‖ = 1.49e-4` behind `‖Φᵀr‖ < 1e-10` at `r = 10`)
  and produced no penetration at any rank down to `r = 2`. §2i, §2j, §7 and §10
  updated where they said the reduced path had never run with contact.
  ⚠ **A review pass before this branch shipped found the size claim resting on a
  MISMATCHED comparison** — 5 202's whole-ramp average against 18 750's
  last-8-steps average — and the harness now sweeps both sizes in one window. The
  effect survived and grew (`1.15 → 1.50`, not `1.06 → 1.50`). That re-measurement
  also produced §2k's third finding: **the margin falls with size** (`~4.5×`
  at 5 202, `1.36–1.47×` at 18 750). ★★★ A SECOND round then found that fitting one
  exponent to `I` hid two terms moving in opposite directions — `contact` at
  `n^1.13` and the validity sweep at `n^0.58` — which corrects the headroom
  marker to `~26 k` free DOF and, more usefully, **located the only
  available lever on R3's margin**: contact is `65–69 %` of `I`, and its slot is two
  `O(n)` passes. ★★ A THIRD round found that §2d had already described those
  passes and dismissed them — *"Neither is on R3's path"* — a sentence
  `C/R = B/I` now OVERTURNS; §2d carries the retraction. That round also narrowed
  the mechanism twice (the per-vertex walk is the linear-mesh path; the `n^1.13`
  is `1.37×` more calls × `n_vert^0.88` per call, not a linear per-call cost) and
  closed a hole in the non-penetration gate itself, which reduced with `f64::min`
  and so SWALLOWED `NaN` — a diverged configuration would have passed. Round 1
  caught a `#[test]` attribute swallowed into a doc comment, which had silently
  removed the timing instrument from the suite; round 2, a published control
  range that did not match its own pilot data.
- **v2.12 (2026-08-24)** — **§2j: v2.11's account of the `profile.rs` survivors was
  wrong, and re-running it is how that was found.** It said they were equivalent
  mutants reachable only through box-gated harnesses, needing `-j 1 --features
  phase-timing`. `-j 1` was never the issue and no harness was needed. ★★ The real
  finding is general: **`imp` exists twice, once per `cfg` arm, and a mutation of the
  arm a build does not compile is scored MISSED — indistinguishable from a real
  coverage hole.** The two runs see opposite halves; five of the original `0 of 29`
  were never live in the build being measured. ⇒ **a mutation score over cfg-gated
  code must name its configuration, or it is not a score.** With the feature on:
  **26 caught, 3 survived** — two of which mutate the `const` guard's own loop bound
  into vacuity, unkillable because a compile-time guard is not a test (`E0081` is the
  protection that matters and cannot be mutated away), and one inert. Every live
  mutant is now caught in the configuration where it is live. What closed the timed
  arm is `tests/profile_contract.rs`, a **separate integration binary** — its own
  process, therefore its own statics, which is what a `#[cfg(test)] mod` could not
  have: the counters are process-global and 337 lib tests would race them. It asserts
  the recording contract with the feature on and the **zero-cost contract with it
  off**, the latter being a module-header claim that nothing had ever checked.
  Negative-controlled against gutted `Drop`, `reset` and `snapshot`.

- **v2.11 (2026-08-24)** — **§2j: mutation testing, after three read-rounds stopped
  converging.** `project_tangent` scores **61 of 61 killable mutants** (10 unviable
  return swaps). ★ A perfect score is when to ask what did the killing, so it was
  attributed rather than assumed: `row != col → row == col` fails the suite, and with
  the byte-identity test `#[ignore]`d **the same mutant passes 328 green tests**. One
  test does all of it — load-bearing, and a warning that ignoring it silently returns
  the function to zero coverage. ⚠ **`profile.rs` measured 0 of 29**: the module that
  produces every figure in §2d / §2i / §2j had no unit tests, and its survivors included
  `delete ! in total_nanos` — which inverts the disjoint/nested filter and rewrites every
  share in this document — plus both constant replacements of `Phase::is_nested` and
  `/` → `*` in `Phases::share`. Eight unit tests take it to **22 of 29**. The remaining
  seven are not worth chasing and the reason is recorded: two mutate the `const` guard's
  own loop bound (a compile-time guard is not a test; `E0081` is the real protection and
  cannot be mutated away), and five are in the feature-gated `imp` module whose disabled
  arm makes them equivalent mutants. ⚠⚠ **The enabled arm's coverage cannot be measured
  by mutation on this box**: it lives in `#[ignore]`d, reference-box-gated harnesses, and
  running those 4-way parallel would trip the contention gate, fail the test, and score
  the mutant **caught for a reason unrelated to the mutation** — a gate that cannot
  *pass* for the reason it claims. Needs `-j 1` with `--features phase-timing`.

- **v2.10 (2026-08-24)** — **§2j BUILT: `project_tangent` is `1.84×` of a reduced
  frame, byte-identically — and the margin did not fall, exactly as pre-registered.**
  A flat `n × r` `Y` with a cached transposed `Φ` turns the gather into contiguous
  AXPYs and the contract into `n` rank-1 updates against an L1-resident `r × r`
  accumulator. Frame `65.89 → 35.72 ms`; `red proj K` `5.61×` (`Y=AΦ` `6.09×`,
  `ΦᵀY` `4.88×`); `asm tangent`, untouched, moved `2.1 %` and is the control that
  licenses the block-ordered comparison. Against §2j's pre-fixed cap: `1.84×` of an
  available `2.31×`. **No parallelism** — the thread knob is unspent. ★ Correctness
  is BYTE-IDENTITY: same terms, same order, gated with `to_bits()` against the
  pre-change form written from the column-major `modes()` so a wrong transpose
  cannot satisfy both, and negative-controlled twice (reversing the contract's
  accumulation order changes the 13th digit and is caught — **a tolerance test
  would have passed it**). ★★ **The invariant was confirmed by experiment**: over
  three runs a side the ceiling fell `19.4–19.9× → 11.2–12.2×` and this fixture's
  requirement `3.92–3.97× → 2.12–2.15×`, while the margin `B/I` did NOT fall
  (`4.91–5.02× → 5.28–5.66×`; the small rise is the validity sweep's own `±10 %`,
  not a benefit of the change). ⚠ `tests/reduced_phase_shares.rs` printed
  *"BRACKET STRADDLES the requirement"* anyway, comparing the new ceiling against a
  hardcoded stale `13.5–15.8×` from a different fixture — the exact defect §2j's
  corollary names, now fixed: the harness reports irreducible ms/step against the
  budget and the margin. ✅ **Knob 1 came free and is where the result is largest**:
  `adjoint_gap_across_basis_sizes` failed on the first run, as its own message asked
  it to. Re-measured, the `O(r²)` coefficient fell **16×** and `O(r)` **2.7×** with
  the r-independent constant unmoved (`117 → 112 ms`, the control), so **R1's
  break-even against the oracle moves `r ≈ 61` → `r ≈ 219`** and at rank 104 the
  reduced path goes `0.51× → 1.83×` the oracle. ⚠⚠ `r ≈ 219` is an EXTRAPOLATION
  to twice the swept range, not a measurement; the measured claim is that **every
  rank in the sweep is now faster than the oracle**, where two of five were slower. Plan §14 finding 2's cost premise is
  overturned and **its conclusion survives on finding 1 alone** — accuracy is
  bit-identical, `Σx*` error is still `0.704` at `r = 104`. The guard was
  re-measured, not widened (`1.25–2.0` two-sided, piloted `1.47/1.49/1.49`).
  ⇒ **`asm tangent` is now `66.9 %` of a reduced frame**, the next item, as the
  prize table said it would be. ⚠ Knobs 2 / 3 / 4 remain unswept. ★ Two review
  rounds. Round 1 found seven defects (an instrument change published without ever
  being run; point values for run-variable quantities in the section forbidding
  them; `r ≈ 219` quoted as measurement rather than extrapolation). Round 2 found
  three more, all inside round 1's own fixes — including that its new slot-uniqueness
  guard **claimed to check every `Phase` but only iterated `Phase::ALL`**, blind to
  exactly the "someone adds a variant" case it was written for. Closed properly:
  `Phase` now carries explicit discriminants and `index()` is `self as usize`, so a
  duplicate slot is a COMPILE ERROR (`E0081`) for every variant, in or out of `ALL`.
  The `const` block keeps the range check. Both negative-controlled.

- **v2.9 (2026-08-24)** — **§2j knob 0 MEASURED: `ΦᵀKΦ` splits `1.85 : 1`, both
  halves are large, and the contract is LATENCY-bound — so one layout change fixes
  both.** `Y = AΦ` is `36.4–36.7 %` of a reduced frame and `Φᵀ Y` is `19.7–19.9 %`,
  stable over three runs, with the split's positive control at `99.5 %` every time.
  ⇒ fixing only the gather caps the whole-frame win at `1.58×` and only the
  contract at `1.25×`, against `2.29×` for both — which is exactly what
  pre-registered rule 6 exists to make visible. ★ The contract's flop rate is
  computable from public quantities alone: `34.13` Mflop/step at `13.02 ms` is
  `2.62` GFLOP/s, **one FMA per `3.36` cycles**, i.e. the serial dependency chain
  of a `sum()` over `n` — not bandwidth, not throughput. Knob 0's pre-registered
  reading ("near scalar peak ⇒ threads + SIMD; far below ⇒ layout") therefore
  says restructure first: a flat `n × r` `Y` and a transposed `Φ` turn the gather's
  inner loop into two contiguous AXPYs and the contract into `n` rank-1 updates
  against a `12.8 KiB` L1-resident `r × r` accumulator, which removes the
  dependency chain by construction. ⚠⚠ **Also widens §2i's published ceiling:
  `20.2–20.5×` was two runs and is TOO TIGHT — four more give `19.0–20.8×`.** The
  cause is §2j's own prediction: the bound is `T/I` and `I` is `~3.2 ms`, so the
  `±10 %` scheduling variance of the `1.2–1.5 ms` validity sweep sets it, while the
  large rows hold inside `1.9 %`. No conclusion moves; quote `≥19×`. ⚠ §2j's
  costing is corrected too — the harness was smoked at **`9.2 s` wall**, so the
  knob ordering by price stands but "the dominant cost of the matrix" means
  seconds, and nothing in it needs rationing.

- **v2.8 (2026-08-24)** — **§2j: R3's kill floor RESTATED, before the measurement
  that would have moved it — and `project_tangent` reclassified as a wall-clock item,
  not an R3 item.** Written as a pre-registration for §2i's next item, and the
  algebra settled the question §2i had left open. Write a reduced frame `T = I + Red`
  against budget `B`: the ceiling is `C = T/I`, the requirement `R = T/B`, so
  **`C/R = B/I`** and R3 clears iff **`I ≤ B`**. ⇒ speeding up any `Reducible::Yes`
  phase lowers `C` and `R` by the same factor and **leaves R3's margin exactly
  unchanged** (`5.18×` at every `S`, verified against v2.7's run 2). §2i had called
  the net sign "not obvious from the armchair"; it is zero. ⚠ What is NOT invariant
  is a gate written as a RATIO: `≥10×` over §2a's post-R0 baseline gets harder every
  time an earlier rung succeeds, and a `10.24×` win on `ΦᵀKΦ` — well inside reach for
  a scalar `Vec<Vec<f64>>` triple loop — would have driven the ceiling to exactly
  `10.00×` and failed it. **Restated as `I ≤ 16.7 ms`**; the `~10×` survives as a
  complexity heuristic, re-derived per measurement rather than carried. Restated
  BEFORE the run, which §7's own "cannot be renegotiated after the fact" makes the
  last legitimate moment. ★ The margin being `B/I` also reorders §2i's list: `I` is
  `3.223 ms`, of which **`61 %` is `contact` marshalling on a `NullContact` scene**
  and `37 %` the validity sweep, so those are the only lines that can move R3's
  verdict while `red proj K` at `56.8 %` of the frame moves it by zero. **The
  measurement R3 waits on is the CONTACT fixture**, the one place `I` is unknown.
  ⚠ The prize is bounded in advance: `project_tangent` at `S = ∞` leaves a `28.57 ms`
  frame, still `1.71×` over budget, with `asm tangent` then `82 %` of it — so it
  **cannot reach 60 Hz on its own fixture**, whole-frame cap `2.31×`. Six
  pre-registered rules and a five-knob matrix (knob 0, which of the two loops is
  slow, gates the rest) are recorded in §2j. ⚠ No code changed and no measurement was
  taken. ★ Also fixes this document's **status line, which had read `v2.3` since
  v2.4** while the history below ran to v2.7.

- **v2.7 (2026-08-23)** — **§2i: the bracket UNSTRADDLED — R3's ceiling is now
  `20.2–20.5×` … `≳32×` over two runs, clear of both its `10×` floor and the
  budget's `13.5–15.8×`.**
  v2.6 said the whole bracket turned on one line, the validity element sweep at
  `9.3 %`, and concluded the `ReducedValidityDomain` was a PERFORMANCE prerequisite
  without which R3 could not clear its own gate. ⛔ **That conclusion is RETRACTED.**
  The sweep's share was a property of running it on one core: it is a per-tet loop
  with no cross-element coupling, run twice per step, and `Mesh` / `Material` /
  `Element` / `ContactModel` were already `Send + Sync` supertraits, so it
  parallelises with no new bound and no signature change. Measured `5.37×` on the
  sweep (9 216 tets, sheared state, 12 threads), taking it from `9.3 %` to `1.9 %`
  of a reduced frame and the frame from `72.07` to `~66 ms/step`. ⛔ ⚠ First version was
  a REGRESSION below ~500 tets — entering rayon costs a fixed ~`60 µs`, so at 12 tets it
  read `0.12×` and at 192 tets `0.55×`, and the skeleton solver is one tet. Fixed with a
  piloted `PARALLEL_SWEEP_MIN_TETS = 1024` (break-even ~430 deformed / ~680 at rest),
  guarded by a `const` assertion; neither published figure moves, both fixtures being
  far above it. The other two held-fixed knobs were then swept too: **threads** (the
  cost is a WAKEUP cost, worst on the default 12-thread pool, and in a 1-thread pool the
  parallel path never wins at any size — dispatch now also requires
  `current_num_threads() > 1`) and **element type** (Tet10 break-even ~200 tets, so the
  Tet4-derived threshold is conservative there, as predicted). ⚠⚠ Lead recorded, not
  acted on: **12 threads is worse than 8 at every size** — the 4 E-cores cost more than
  they return, worth `40–70 %` on mid-sized meshes, but the pool is shared with faer.
  Verdict-identical
  — violators reduce by `min_by_key` on `tet_id`, so first-violator-wins survives an
  unspecified visit order; negative-controlled by swapping in `max_by_key`.
  `ReducedValidityDomain` reverts to §4c's correctness feature and R3's two halves
  are separable again. ★ Coherence check: `89 %` of the whole-frame improvement is
  the row that changed, and the two large untouched rows moved `< 1.5 %`. ⚠ The
  full-order path gains almost nothing (`0.4–0.6 %` of a frame, where factorization
  is `70–77 %`), so **§2a's gap and §2d's table are unchanged** — this was a
  reduced-path optimisation, and the reduced path is the one that has never run with
  contact. ⚠ `ΦᵀKΦ` rises to `56.8 %` of a reduced frame, now the largest line by a
  wide margin. ⚠ **§2h had to be re-baselined**: the probe is a full solve, so this
  change moved its quiet median `24.47 → 22.75 ms` and left legitimate readings
  `3.8 %` from the `21.0` floor — which is what that floor is for. Re-piloted (idle
  `22.60–23.36`, 4 busy cores `24.17–24.93`), ceiling `25.5 → 23.7`, floor
  `21.0 → 19.5`, burst unchanged; the finding that **burst cannot detect sustained
  partial load replicated on a different tree**. Named as a standing cost of a probe
  that measures the SUT — though ⚠ NOT by `project_tangent`, which is reduced-path while
  the probe is full-order. It WOULD be moved by removing the `NullContact` marshalling,
  since the probe is built with `NullContact`. ▶ Next, in order: (1) `project_tangent`,
  (2) the reduced path WITH contact.

- **v2.6 (2026-08-23)** — **§2i: R3's Amdahl ceiling MEASURED, and it brackets
  `8.3×` … `≳37×`** (the upper bound is ill-conditioned near `f → 1` — two identical
  runs gave 37.1 and 38.9 — so only its order matters).** §4b picked ECSW on a literature claim of 2–3 orders of magnitude,
  but that is a speedup on element integration and R3's whole-step gain is capped
  by what fraction of a REDUCED frame that is — never measured, because the reduced
  solver carried no timers. Six slots added plus `Phase::ecsw_reducible`, now a
  three-way `Reducible` so "removable only if R3's own design works" is reported as
  a separate bound instead of being folded in. At R1.1's operating point (5 202 free
  DOF, `r = 40`, 99.9 % instrumented): **`ΦᵀKΦ` is 52.2 % of the frame**, more than
  the assembly it projects. ⇒ **The bracket straddles R3's own `10×` floor AND the
  budget's `13.5–15.8×`, so the bound decides everything.** ★ What moves it is one
  line: the **validity element sweep at 9.3 %**. R3's scope already includes
  replacing it, but as a §4c correctness feature — it is actually a PERFORMANCE
  prerequisite, and without it R3 cannot clear its own gate. ⚠⚠ Ceiling and
  requirement are on DIFFERENT FIXTURES — `13.5–15.8×` is IPC 18 750 with contact,
  this is R1.1's contact-free cantilever, and the reduced path has never been run
  with contact at all. ⚠ Two errors caught before publication: `Contact` nests
  inside two reducible parents so the broad-phase was counted as removable
  (`10.0×`→`8.1×`), and the first version left 9.7 % uninstrumented and treated it
  as irreducible, which alone spanned `8.1×`–`37×`. ▶ Next: (1) can the
  `ReducedValidityDomain` gate actually replace the sweep, (2) `project_tangent` as
  an implementation question — a naive `O(n·r²)` contraction over `Vec<Vec<f64>>`,
  an R0-shaped target stated as hypothesis, (3) the reduced path WITH contact.

- **v2.5 (2026-08-23)** — **§2h: the reference box is FIXED, and enforced in code.**
  `Mac16,8` (Apple M4 Pro, 8P+4E) — hard-failed on model and core split, stamped on
  RAM/OS/rustc, via `tests/refbox/mod.rs`, which both gate-bearing harnesses now call.
  A ~0.6 s probe (`cantilever 40×4`, `dt = 1e-3`, Newton count pinned at 3) runs first
  and refuses the measurement on a loaded box. ★ **Thresholds piloted, not chosen**:
  10 idle runs then deliberate load. ⚠ **The pilot overturned the design** — `burst`
  (max/p50), which the probe originally documented as "the contention statistic",
  does NOT separate sustained partial load (`1.063×` at 4 busy cores against a
  `1.060×` idle max), because steady load lands in the median. The absolute median is
  the sensitive check; `burst` is kept as the gross-stall backstop. Negative-controlled:
  3 busy cores are refused in 0.69 s. ⚠ The probe also **caught its own premise
  failing** on its first run (`[2, 3, 3, …]` iterations) and declared itself invalid.
  ⚠ **Transfer between boxes remains UNVALIDATED and is not offered** — it needs two
  boxes, and the one cross-box dataset shows it failing in opposite directions.
  R3's two numbers are now stated separately: kill floor `≥10×`, frame budget
  `13.5–15.8×`, both on this box. ⚠ Review caught that the gate would have BROKEN
  the cross-tree A/B — the probe fixture is one R0 speeds up (57.39 ms pre-R0 vs
  24.47 post), so the median ceiling would reject the pre-R0 arm every run; that
  path now checks identity and the scale-free `burst` only, and leans on
  interleaving for the rest. ⚠ Review also caught that only 2 of the 4 harnesses
  feeding R3's arithmetic had been gated — `predictor_spike.rs` and
  `reduced_predictor.rs` were missed, and the predictor's `1.97×` is wall-clock,
  not an iteration count. All four are gated now.

- **v2.4 (2026-08-23)** — **⛔ v2.3's `9.89×` is WITHDRAWN, and the method that
  produced it is retired.** Two independent defects, both found by review rather
  than by the instrument. (1) **R0's credit is `1.62×` at IPC 18 750, not
  `1.186×`** — measured by a direct wall-time A/B across `ecf4cfef^`→post-R0 on one
  box, interleaved, with Newton iteration counts identical to the last digit on all
  three fixtures (`tests/r0_ab.rs`). Cross-session share differencing was wrong by
  37 %; its premise "R0 touched only tangent assembly" is falsified by this
  document's own rows (`asm force` moves 2.4×, `tri solve` 1.7×). All three
  measured credits (1.62× / 1.75× / 1.84×) land inside the `1.51–1.89×` whole-step
  range R0's own PR reported; `1.186×` does not — §2f had Amdahl-discounted a figure
  that was already whole-step, and the share differencing appeared to confirm it.
  (2) **The `46.2×` gap is unsound.** §2a's `771.0 ms` is the one figure of three
  that no box factor reconciles (two fixtures put the recon's box 1.6× SLOWER, that
  one puts it 2× faster), so "box difference, not a regression" was an excuse, not
  an explanation. ⇒ **The gap is now MEASURED post-R0 on a named box: 887.9 ms,
  `53.2×`, and R3 needs `13.5–15.8×` against a `10×` floor — ABOVE it.**
  ⚠ Also corrected: the phase-timing instrument booked contact-Hessian work to
  `AssembleTangent`, which the contact-free positive control could not catch; the
  post-R0 contact rows above are the re-measured ones.
  ⚠ **The pre-registered cantilever control for the new A/B FAILED** — it targeted
  the recon's published `2.53×` and measured `1.75×`. Recorded rather than
  discarded: the target is itself a cross-session ratio over an unknown window on a
  strongly transient trajectory (1.4 s to 11.4 s within one 12-step run), i.e. an
  instance of exactly what finding 4 documents. It does not rescue `1.186×`, and it
  does mean the A/B's *external* validation rests on the `1.51–1.89×` range alone.

- **v2.3 (2026-08-23)** — ⛔ **SUPERSEDED BY v2.4 — the headline result below is
  withdrawn; kept for the record.** §2d's two `pre-R0` contact rows re-measured.
  Timers are now a permanent feature (`src/profile.rs`,
  `phase-timing`, zero-cost when off) rather than a scratch patch, and
  `tests/phase_shares.rs` re-runs the table. R0 touched only tangent assembly, so
  the share shift gives its credit directly: `asmK` 30.1 % → 17.1 % at 18 750 ⇒
  **R0 = 1.186×**, and **R3 needs 9.89× against its 10× kill floor** — viable by one
  percent, where the two Amdahl bounds had straddled it (8.2× / 10.1×).
  ★ Validated before reading: the instrument reproduces the already-post-R0
  `cantilever 80×8` row at 1.02 / 1.04 / 1.00 / 0.96×.
  ⚠ **The pre-R0 5 202 row is UNSOUND**, not merely stale — its shares imply R0 made
  that fixture 0.917× SLOWER, which a strictly cheaper byte-identical data structure
  cannot do. ⚠ Absolute ms/step is not comparable across sessions (917.6 vs 771.0 at
  18 750 is box difference); only shares are.

- **v2.2 (2026-08-23)** — **§2g: the `‖r‖` selector was built, measured on all six
  cells, and KILLED.** It rescues the cell `InertialWithLoad` dies on and avoids
  `block_sag`'s 32.7× blowup, at −0.15–6.70 % overhead, but selects the WORST arm
  on the cantilevers (1 134 vs `Inertial`'s 241 at 19 440 DOF). Cause: `‖r(x⁰)‖`
  is not a proxy for distance to the ROOT, and the split is the cantilever against
  everything else — `block_sag` is gravity-driven bending and the ranking is right
  there. `InitialGuess::Adaptive` was removed rather than shipped; §9b carries the
  re-apply recipe, since the code was squashed away. **An element-size veto is
  ruled out by the same section on arithmetic**; a contact-band veto is not.
  ⚠ Also marks that §2f's R3 table assumes R1 = 2×, which #817 showed holds only
  inside R1.1's load box.
- **v2.1 (2026-08-23)** — **§2f added: the iteration-count factor is a lever, and a
  large one.** `SolverConfig::initial_guess` (a Newton predictor; default bit-equal)
  measured across six fixture/size/load cells, three arms stepped in lockstep. 1.95×
  fewer iterations on the representative IPC workload and 4.71× on `cantilever 80×8`
  (wall-clock totals 1.90× / 4.63×; see §2f for the median column and for why the
  three aggregates must not be mixed), zero
  convergence failures, trajectories identical to 6e-14…8e-12 on the subjects. **R3's required gain
  is roughly HALVED — from 11.5–19.8× to 5.8–10.1× depending on how much credit R0
  gets, a derivation §2f now shows in full rather than citing.** (⚠ v2.3 claimed to close that fork at
  `1.186×`/`9.89×`; ⛔ v2.4 WITHDREW both — R0 is `1.62×` and R3 needs `13.5–15.8×`.) Before the predictor
  the requirement sat clearly above R3's `10×` kill floor on every accounting; it now
  brackets it (comfortably under on the optimistic row, 10.1× vs 10× on the measured
  one). ⚠ An earlier draft of this entry claimed the floor now cleanly exceeds the
  need and cited a §7 figure that does not exist. ⛔ A sixth cell added after review —
  contact PLUS a body load — **kills `InertialWithLoad` at step 0 in both load
  directions**, so `Inertial` is the robust arm and the load term needs a gate before
  it can be used at all. `InertialWithLoad` additionally
  collapses the frame-time tail (`ms max / ms p50` 4.44× → **1.04×**), the first thing
  measured here that attacks VARIANCE rather than the mean. ⚠ Two limits recorded with
  it: the factors compose only once the predictor is ported to `reduced::newton`
  (unmeasured), and the `block_sag` control got **32.7× worse**, which bounds the
  technique — the predictor is a bet that motion over a frame is inertia-dominated,
  and a stiff pinned block is where that bet is wrong by ~1000×. The header verdict,
  caveat 3 and §2a's frame-budget paragraph were re-pointed at §2f.
- **v2.0 (2026-08-22)** — **§2b's concurrency reading corrected, and it was a modelling
  error rather than a bad measurement.** `process max RSS` is a peak; 84 % of the 919 MiB
  at 70 644 free DOF is a transient numeric factor that `CpuNewtonSolver` does not retain,
  so dividing 24 GB by it undercounted. Re-measured marginal cost per concurrently
  stepping env: **638 MiB (full) vs 61 MiB (reduced)** ⇒ **38 vs ~395 envs**, i.e. the
  ceiling is nearly 2× better than recorded *and* R1 as built already lifts it 10.4×
  without hyper-reduction. §8b's reading (b) re-pointed at the corrected figure. Header
  version stamp also corrected — it read v1.7 while this section already recorded v1.9.
  **Separately, three sites that v1.3 missed were brought up to date**: §10 risk 2, §10
  risk 6 and §5's hybrid-DD paragraph still carried the pre-R0 frame-budget figures
  (800 free DOF / `n^1.38` / 540 DOF at 1.37× / 3 000 DOF at 28×), so the doc asserted
  both that the 540-DOF anchor missed the 60 Hz budget by 1.37× and that it fitted at
  0.47×. §5 was additionally misquoting §2a by name.
  **A third round found the same drift inside §2a itself** — the section the other two
  fixes were checked against. Its DOF-gap sentence still read 25–90× (the pre-R0 pair),
  as did §3a, and it derived a "70–350× in time" figure from **`n^1.28`, an exponent
  with no producer anywhere in this document**. Both corrected to 13–47×, and the time
  claim replaced with the three directly measured rows that bracket the target band
  (34.6× / 46.2× / 267×) — a spread governed by trajectory nonlinearity, not DOF.
- **v1.9 (2026-08-12)** — R1.3 landed; §6's goal-oriented-basis lead upgraded from a
  hypothesis to a measurement (enriched `r = 40` beats plain `r = 104`, 2.7x faster,
  conditional on the objective family being low-dimensional) and the "just raise `r`"
  alternative closed off with a measured break-even between `r = 40` and `r = 80`.
- **v1.8 (2026-08-11)** — **R1 complete, and it falsified another of this document's own
  predictions.** §6 named hyper-reduction as the bound on gradient accuracy; R1.2
  measured a 0.24–0.82 relative gradient error with no hyper-reduction present, traced to
  the basis not spanning the adjoint field. §6 amended with the correction, the validity
  domain's new "what you differentiate" axis, and the goal-oriented-basis lead it hands
  R3. §7's R1 row marked DONE and its gradient gate corrected. The `Φ`-constant decision
  itself is unaffected — it was confirmed to 2.85e-8.
- **v1.7 (2026-08-11)** — §4b given the size qualifier it was missing. v1.6 said a basis
  alone attacks the larger half; true asymptotically, but forming `ΦᵀAΦ` is linear in `n`
  where the factorization is superlinear, so it is a wash at ~3 000 free DOF and only
  wins above ~10–20 k. Surfaced while scoping R1 — see
  `docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md`, which also adds the fixture argument and
  mandatory held-out trajectories that §7's R1 row did not carry.
- **v1.6 (2026-08-11)** — **open risk 3 withdrawn: the oracle was never broken.** Five
  discriminators (§3d) show `NewtonIterCap` and nothing else — no Armijo stall, no
  non-PD tangent, no element inversion, and LM changes nothing because there is nothing
  to rescue. The measuring harness capped Newton at 60; the case needs 65. With a
  generous cap every size converges. The residual real finding — iteration count grows
  with refinement (21 → 65 across 567 → 36 663 free DOF), which compounds the
  frame-budget scaling — moved into §3b.4 with its mechanism (Tet4 locking decreases
  under refinement, so the finer beam is genuinely softer and deflects further). The
  withdrawn risk is struck through rather than deleted, because it had been ranked
  above R1. No measurement changed; one conclusion did.
- **v1.5 (2026-08-10)** — **collapsed to a single baseline.** v1.3/v1.4 carried pre-R0
  and post-R0 numbers side by side with banners; that structure was a footgun and I
  tripped over it within one commit (v1.4 exists only because v1.3's header quoted the
  superseded set). Every §2 table now states the post-R0 measurement, §2f is folded
  into §2a/§2d, §2d.1 states the current crossover with one paragraph on why it moved,
  and §4b argues from the current shape instead of narrating its own revision. The
  pre-R0 measurements live in git (`b0f4aa21`) and in PR #741/#742, which is where an
  audit trail belongs — not in a live planning document. No measurement changed.
- **v1.4 (2026-08-10)** — header verdict and caveats 1–2 updated to point at §2f. v1.3
  added the post-R0 baseline but left the most-quoted block still stating the pre-R0
  numbers with no pointer — recreating, in the same document, the caveat-far-from-claim
  defect v1.2 existed to fix.
- **v1.3 (2026-08-10)** — **R0 landed and falsified one of this document's own
  conclusions.** §2f adds the post-R0 baseline, measured on the quietest box of the
  session: the assembly/factorization crossover fell from ~15–20 k free DOF to ~1–2 k,
  so §2d.1 is now obsolete for the shipped code (kept unedited, with a banner, because
  it is what motivated R0). §4b's "hyper-reduction IS the reduced scheme" argument
  rested on that crossover and is restated — a basis alone now attacks the larger half,
  and ECSW becomes the second lever rather than the whole scheme. Reachable size roughly
  doubled (540 free DOF at 12.3 Newton iterations now fits a 60 Hz frame at 0.47× budget,
  measured, where §2a had it at 1.37× over); the gap narrows from 25–90× to 13–47× in
  DOF. R3's kill condition now refers to §2f.
- **v1.2 (2026-08-10)** — caveat block promoted into the header, beside the verdict
  that quotes the numbers. The three limits were already at §1a and §10, but the
  most-read block in the document carried none of them, which is the wrong way round:
  a caveat 700 lines below the claim it qualifies is a caveat that will be skipped.
  No measurement or conclusion changed.
- **v1.1 (2026-08-10)** — §8 resolved. `MISSION.md` amended with the "On interactive
  speed" clause; §8a added to pin the collision to R3 specifically (R0/R1 do not
  collide on fidelity; R4/R5 collide on attention). No measurement changed.
- **v1 (2026-08-10)** — first issue. Phase 1 measured (four questions, three new
  dynamic fixtures); Phase 2 recon written. Two findings not anticipated by the brief:
  the f32 question splits into factor/solve (safe) vs residual (not safe at present
  tolerance), and contact costs 0.02 % of a contact-active frame. One full-order lever
  (§2d.2) surfaced and promoted to rung R0 ahead of any reduced-order work.
