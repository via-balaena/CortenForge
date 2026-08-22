# sim-soft Real-Time Path — Phase-1 Measurement + Recon (Phase E predecessor)

**Status**: RECON 2026-08-10 (rev 2026-08-22), v2.0. Phase 1 (measure) COMPLETE — all four requested
measurements taken; §2 reports them. Phase 2 (this recon) proposes the MOR +
hyper-reduction path with a staged ladder whose first rung is a kill-or-confirm.
**No production code was written and no dependency was added.** The measurement
instrumentation was temporary (implement → measure → revert, per
`feedback_implement_measure_revert_pattern`) and is reverted; §9 lists exactly what
was touched and how to reproduce.

**MISSION.md**: AMENDED 2026-08-10 with the "On speed as a ceiling" clause — see §8b.
The ladder is authorised in principle; R3's three conditions are now mission-level.
⚠ That clause cites **this document** as the measured basis for its "throughput is a
ceiling" premise, so §2b is load-bearing outside this recon — re-take it on an idle
box (risk 1) before anyone leans on it further.

**One-line verdict**: the goal is **not refuted**, but the measurements move where the
work is. The frame-budget gap is ~**13–47× in DOF** (≈ 1 500 free DOF fits a 60 Hz
converged-Newton frame; a high-quality environment wants 20k–70k), f32 is
**safe for the factor and solve and unsafe for the residual** — a distinction the
brief did not anticipate and which changes the wgpu plan — and **contact costs
0.02 % of a contact-active frame**, so contact is a *structural* problem for MOR, not
a compute one. Two cheap full-order levers surfaced that must be measured before any
reduced-order code is written.

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

**The frame budget, stated plainly.** Take the IPC indentation's 6 Newton iterations
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

⚠ The two IPC rows are the only pre-R0 timings still quoted anywhere in this
document; the contact arm was not re-run after R0. Read them as upper bounds.

A "high-quality environment" in this codebase's own terms is the 20 k–70 k free-DOF
range (`Cargo.toml` cites 70 k free DOF; the conformed-disc and FSU meshes sit in
that band). **The gap is 25–90× in DOF, which at n^1.28 is 70–350× in time.**

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
| 5 202 (IPC contact, pre-R0) | 0.5 % | 17.7 % | **80.6 %** | 0.3 % | **0.02 %** |
| 18 750 (IPC contact, pre-R0) | 0.7 % | 30.1 % | **65.6 %** | 2.3 % | **0.02 %** |

Symbolic factorization is one-shot per solver construction (1.0 ms at 540 DOF →
1 029 ms at 70 644 DOF) and is **not** on the frame budget — it is amortised over
every step of a session. That is already the shipped design (`construct.rs` builds it
once; `replace_contact` deliberately reuses it).

Three findings:

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

3. **Triangular solves are never the bottleneck** (0.3–2.9 % everywhere). Worth
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

---

## 3. What the measurements say about feasibility

### 3a. The goal is not refuted

Nothing measured says real time is unreachable. The gap is large (25–90× in DOF) but
it is the *size* of gap that reduced-order modelling is built for: ECSW/cubature
literature routinely reports 2–3 orders of magnitude on exactly this shape of problem
(hyperelastic FEM, fixed mesh, repeated solves). The brief's central architectural
claim — that a reduced system is small and dense, so **converged Newton with an exact
direct solve is preserved** and the error moves into the quadrature approximation
where it can be measured against the full-order path — survives contact with the
numbers, and §2d.3 strengthens it: since triangular solves are already negligible,
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
important thing to get right in R3.

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
  coupled to a reduced bulk of `r ≈ 30–100` gives a system of ~500–800 DOF — which
  §2a's "≈ 800 free DOF fits a 60 Hz converged frame" says is *exactly* the reachable
  size. The arithmetic closes, which is a genuine (if provisional) encouragement.
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
3. Validity-domain statement for contact. §4c's `‖q‖` gate does not detect "the
   indenter moved somewhere the ensemble never saw."

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
| **R1** ✅ **DONE** (`#744`, `#745`, R1.2) | **Linear subspace, no contact, no coupling.** POD basis from full-order snapshots on the `cantilever` fixture at 3 000 free DOF; reduced Newton with a dense `r × r` direct solve; `Φ` and quadrature both handled naively (full element sweep — **no hyper-reduction yet**). Differentiable path wired at the same time (§6, `Φ` constant). | Projection error vs the oracle < 1 % in tip displacement over the training trajectory; reduced gradient matches the oracle's to the crate's existing gradcheck tolerance. ⚠ **That second clause was wrong and was amended before R1.2 was built** — it asks two different functions to agree to 5 digits when their states already differ in the third. Split into a gradcheck-tolerance kill gate on the reduced model's *own* derivative and a measured comparison against the oracle; see the plan's §5/§7 and §13. **Wall time is explicitly NOT gated at R1** — without hyper-reduction it will not be faster, and pretending otherwise would corrupt the signal. | **The cheap kill-or-confirm.** It answers the one question that decides everything downstream: *does a low-dimensional subspace represent this material's deformation at all?* Fixture already exists; no new physics. |
| **R2** | **Precision decision.** Measure a full-f32 forward path on the reduced system (`r × r` is small enough to port by hand without touching the 1 396-`f64` production surface), and decide residual-in-f64-on-CPU vs compensated-summation-in-f32. | Reduced-model f32 forward drift and gradient drift vs the f64 reduced model, on R1's fixture; explicit go/no-go on whether the residual can live in f32. | §2c. Must precede any GPU work; deciding it after a shader exists means writing the shader twice. |
| **R3** | **Hyper-reduction (ECSW) + the validity domain.** NNLS training over R1's snapshots; `ReducedValidityDomain` with the online `‖q‖` + residual-proxy gate; the three error measures of §4c. | Measured speedup vs §2a's baseline (post-R0), with the three §4c errors reported alongside. Domain gate demonstrated to fire on an out-of-domain trajectory. | This is where the frame-budget win actually arrives. Also where the "smooth and wrong" failure mode is defended against. |
| **R4** | **Hybrid domain decomposition, FIXED contact patch.** Full DOF under a stationary indenter, reduced bulk, on the `dynamic_indentation` geometry. | End-to-end reaction force vs the oracle, in the same band `bonded_layer_indentation` already asserts. | §5. Fixed patch first, because it isolates the coupling condition from the re-partitioning problem. |
| **R5** | **Moving patch.** Re-partitioning under a once-built symbolic factorization, or a conservative union pattern. | Sliding-contact trajectory vs the oracle. | §5's open-research item. **Explicitly gated on R4 succeeding**; if R4 fails, this is not attempted. |
| **R6** | **Rigid↔soft coupling.** | — | **Last, deliberately.** Per the brief, and it is the right call: the keystone coupling is itself the platform's hardest open problem (`MISSION.md` §2), and stacking it on an unsolved real-time reduced path would make any failure uninterpretable. |

**Kill conditions, stated in advance** (so they cannot be renegotiated after the
fact): if R1 shows the projection error is not small at any `r` worth having, the MOR
path is wrong for this material class and the recon is falsified — the correct
response is to revert to R0's win and re-recon, not to widen the basis until the
number looks acceptable. If R3's measured speedup over §2a's (post-R0) baseline is under ~10×,
reduction is not paying for its complexity and the honest move is to bank it.

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

**No production code was written. No dependency was added.** Four temporary
instrumentation edits and three temporary test files existed during the session and
are reverted. Verified after the revert: `git status` shows only this doc as
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

---

## 10. Open risks

1. **Every timing number was taken on a contended box (§1a).** Absolute times are
   upper bounds. The *shapes* (scaling exponents, phase shares, crossover point) are
   robust across repeats; the absolutes are not benchmark-grade. Re-take §2a on an idle
   machine before quoting any figure externally.
2. **The `≈ 800 free DOF at 60 Hz` headline is an interpolation**, not a measured
   point — it is derived from the 540 → 3 000 DOF pair at the measured `n^1.38` low-end
   exponent, using the IPC fixture's 6-iteration count. The bracketing measurements
   (540 DOF at 1.37× budget; 3 000 DOF at 28×) *are* direct. Treat 800 as an order-of-
   magnitude statement.
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
   that this recon did not measure.** R3's gate exists precisely to hold it to account,
   and R3's kill condition (< ~10× over R0) is the response if it does not hold.
6. **§5's hybrid-DD arithmetic closes on paper** (a few hundred patch DOF + `r ≈
   30–100` lands inside the reachable ~800 DOF). That is encouraging and it is not
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

- **v2.0 (2026-08-22)** — **§2b's concurrency reading corrected, and it was a modelling
  error rather than a bad measurement.** `process max RSS` is a peak; 84 % of the 919 MiB
  at 70 644 free DOF is a transient numeric factor that `CpuNewtonSolver` does not retain,
  so dividing 24 GB by it undercounted. Re-measured marginal cost per concurrently
  stepping env: **638 MiB (full) vs 61 MiB (reduced)** ⇒ **38 vs ~395 envs**, i.e. the
  ceiling is nearly 2× better than recorded *and* R1 as built already lifts it 10.4×
  without hyper-reduction. §8b's reading (b) re-pointed at the corrected figure. Header
  version stamp also corrected — it read v1.7 while this section already recorded v1.9.
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
