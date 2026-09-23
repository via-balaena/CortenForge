# `insertion_sim` → Tet10 + IPC face barrier — renovation recon

> **What this document is.** A measurement of the gap between
> `tools/cf-sim-research/src/insertion_sim.rs` and the `sim-soft` capability
> that landed after it was written. Every claim below is either a `file:line`,
> a command that can be re-run, or an explicitly-labelled unknown.
>
> **What it is not.** A plan with a schedule, or a statement that the work is
> understood well enough to cost. Section §7 lists what this recon did **not**
> measure, and that list is longer than the measured part.
>
> Measured at `6d1c5188`, 2026-09-19.

## 0. The one-sentence finding

**The correct contact-pressure field exists only on Tet10; the constitutive
model the sleeve requires exists only on Tet4; nothing connects them.**

```
impl<M: BuildableFromField> Mesh<M> for SdfMeshedTetMesh<M>   generic over material
impl Mesh for Tet10Mesh                                       default M = NeoHookean
```

`sim/L0/soft/src/sdf_bridge/sdf_meshed_tet_mesh.rs:543` ·
`sim/L0/soft/src/mesh/tet10_mesh.rs:653` ·
trait default at `sim/L0/soft/src/mesh/mod.rs:76`
(`pub trait Mesh<M: Material = NeoHookean>`).

## 1. Why this matters for the insertion problem specifically

The conformity reward (`sim/L0/soft/src/readout/reward_breakdown.rs`) scores
four terms, two of which are contact-pressure fields:

| term | needs |
|---|---|
| pressure uniformity | pressure distributed over the contact surface |
| coverage | contact indicator per surface element |
| peak pressure bound | `peak_contact_pressure` (`contact/mod.rs:283`) |
| effective stiffness floor | transmitted normal force |

**On Tet4, IPC emits `ContactPair::Vertex` — per-vertex point forces with
corner-face tributary areas.** On Tet10 it emits `ContactPair::Face`, the
surface-integrated barrier `E = A_rest · Σ_q ŵ_q b(sd(x_q))` sampled at face
Gauss points (`contact/mod.rs:48` enum; `contact/ipc.rs:182` emission).

The selection is automatic and silent:

```rust
// contact/ipc.rs:182
mesh.boundary_faces6().map_or_else( /* vertex path */, /* face path */ )
```

`boundary_faces6()` defaults to `None` (`mesh/mod.rs:189`) and is implemented
only by `Tet10Mesh` (`mesh/tet10_mesh.rs:696`). `SdfMeshedTetMesh` does not
implement it. **A Tet4 mesh therefore takes the vertex path with no
diagnostic.**

⚠ Rung 8d's own module doc records what the pre-8d readout did wrong, which is
the failure mode a vertex-path pressure readout still has on Tet10-shaped
loads: *"the loaded midside nodes got zero area → NaN pressure → silently
dropped from `peak_contact_pressure`"*, with *"corners carry ~0 force, midsides
carry the load"* (`sim/L0/soft/tests/tet10_face_pressure_readout.rs:1-22`).

## 2. What `insertion_sim` uses today

Measured by grep against `tools/cf-sim-research/src/insertion_sim.rs` (5,806
lines), controls in parentheses:

| capability | count |
|---|---|
| `PenaltyRigidContact` | 21 |
| `Yeoh` | 87 |
| `Tet4` | 5 |
| *(control)* `contact` | 166 |
| `IpcContact` / `ipc` | **0** |
| `friction` | **0** |
| `Tet10` | **0** |
| `autograd` | **0** |
| `RewardBreakdown` / `uniformity` / `conformity` | **0** |
| `peak_contact_pressure` | **0** |

Construction site: `insertion_sim.rs:1227`
`CpuNewtonSolver::new(Tet4, mesh, contact, config, bc)` with
`PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging`
(`insertion_sim.rs:1076`).

## 3. What is free today

| piece | evidence |
|---|---|
| `Tet10Mesh::from_tet4(&dyn Mesh)` — takes **any** mesh | `mesh/tet10_mesh.rs:101` |
| `.with_sdf_projected_boundary(sdf, quality_floor)` — midsides snapped to the **true scan SDF** | `mesh/tet10_mesh.rs:514` |
| `enrich_tet4_to_tet10` exported | `sim/L0/soft/src/lib.rs:69` |
| IPC auto-emits `Face` on a Tet10 mesh; Tet4 output byte-identical | `contact/ipc.rs:166,182` |
| `peak_contact_pressure` face-consistent (rung 8d) | `contact/mod.rs:283` |
| `cf-sim-research` does **not** break on the `ContactPair::Face` variant — it only *constructs* `Vertex{..}`, legal under enum-level `#[non_exhaustive]`; spike-confirmed | `docs/SIM_SOFT_TET10_PLAN.md` rung-8a correction |

The `with_sdf_projected_boundary` path is the one that matters most for a
scan-derived cavity: it integrates over the real curved geometry rather than
inscribed facet chords — *"exact geometry IS the exact physics"*
(`sim/L0/soft/tests/tet10_sdf_projection.rs:1-8`).

## 4. The blocker, and the fork behind it

### 4.1 Blocker

`Tet10Mesh` implements `Mesh` at the trait's default material only. Compile
probe (written, run, deleted — reproduce by re-creating it):

```rust
// sim/L0/soft/tests/<tmp>.rs
type Control = CpuNewtonSolver<Tet10, Tet10Mesh, IpcRigidContact, NeoHookean, 10, 4>; // compiles
type Target  = CpuNewtonSolver<Tet10, Tet10Mesh, IpcRigidContact, Yeoh,       10, 4>; // FAILS
// error[E0277]: the trait bound `Tet10Mesh: Mesh<sim_soft::Yeoh>` is not satisfied
```

Yeoh is not optional for this scene. Example row 22 records the wall:
*"Neo-Hookean validity domain trips at `max_disp ≈ 7 mm`"*; row 23 pivoted to
Yeoh to reach the 8 mm target (`examples/sim-soft/README.md`).

### 4.2 ⛔ CORRECTED — there is no fork; the choice was already measured

> **This section previously presented an A-vs-B fork (Tet10+Yeoh vs
> Tet4+mixed-u-p). That framing was wrong** and is kept corrected rather than
> deleted, because the way it was wrong is instructive: it was built from the
> book's general argument without checking whether the code had already
> decided. See §9.

**`MixedUP` is not missing by accident. It was deliberately not built**, because
a pre-registered measurement found the higher-order element sufficient
(`docs/SIM_SOFT_TET10_PLAN.md`, rung-6 verdict):

> **"★ rung 6 returned ACCEPT: pure-displacement Tet10 is adequate at ν = 0.49
> and Taylor-Hood P2-P1 is NOT built"** — ν = 0.49 reads `0.0314` `Continuum` /
> `0.0148` `Facet` against a **pre-registered ≤ 0.10 bar**, mesh-stable to h/4,
> scoped to mean displacement, resting on `tet10_lame_decision.rs` +
> `tet10_bending_locking.rs`.

⇒ **Tet10 is this codebase's near-incompressibility path *and* its rim path.**
One change, both failure modes. The book's preference for mixed u-p is a
general argument; rung 6 is a measurement on this solver, and it is the one
that governs.

#### What does exist: F-bar, unused

`solver/backward_euler/fbar.rs` (1,219 lines, enabled by `config.fbar`) is the
nodal-averaged F-bar volumetric-locking cure. **Nothing in the sleeve path
enables it** — `insertion_sim` and both scan-fit sleeve example rows grep to 0.

Its own module doc ranks it, measured against the analytic Lamé thick-shell
oracle on **cavity-wall displacement** — this product's quantity:

| method | ν = 0.40 | ν = 0.49 |
|---|---|---|
| plain Tet4 | converges to analytic | **−23 %** (locked, under-predicts) |
| Tet4 + F-bar | ~+5 % over-soft | **+21 %** over-soft |
| mixed u-p / Tet10 | — | *"the quantitatively accurate near-incompressible paths"* |

> *"Use F-bar for ν = 0.49 **stability** and qualitative / relative work; **do
> not read 'cures locking' as 'accurate at ν = 0.49.'** Roadmap to the accurate
> element: `docs/SIM_SOFT_TET10_PLAN.md`."*

#### ★★ The prize is larger than rim accuracy: the ν the sleeve is modelled at

`NeoHookean::from_young_poisson` carries a hard cap:

```rust
assert!(nu < 0.45, "standalone NeoHookean requires nu < 0.45; use the Ch 05 \
        locking-fix decorator for higher Poisson ratios");
```

So `silicone_table.rs` runs silicone at **ν = 0.40**, and says so:

> *"the 0.40 framing **deliberately introduces volumetric locking error** that
> calibration absorbs into the effective μ at post-cast time. **Tet10 + F-bar at
> Phase H recovers the near-incompressible regime without the ν shift.**"*

Real silicone is **ν ≈ 0.499**
(`20-materials/05-incompressibility.md`), which calls dropping ν to keep a Tet4
solver converging *"what cheap soft-body pipelines do"* and names the failure
mode directly:

> *"a compressed **sleeve** appears to **deflate rather than bulge sideways**"*

⇒ For an insertion device, bulge-versus-deflate is not a fidelity detail — it
is the wall's entire mechanical behaviour. **The element change is what lets
the sleeve be modelled as silicone rather than as a compressible
approximation**, and that is a larger correctness gain than the rim geometry
this section originally argued from.

⚠ Still open, and not closed by the above: the book's prescribed rim fix is
**two** commitments — adaptive h-refinement **and** Tet10 — and states *"Either
alone leaves a residual."* **Adaptive h-refinement is not implemented**
(`mesh/mod.rs:146` names it as Phase H future work; the other grep hits are a
uniform block subdivider and interval-arithmetic validity certification, neither
of which is mesh refinement). Tet10 buys the element half of a two-part fix.

### 4.3 The strongest single argument for Path A

Example row 24 uses `CELL_SIZE = 0.004` m against layer thicknesses of
6 / 4 / 4 mm (cumulative offsets `0.006 / 0.010 / 0.014` at
`examples/sim-soft/scan-fit-3layer-sleeve-yeoh-axial-zoned-ramp/src/main.rs:313-334`).

**The mesh cell size equals the thinnest layer — one element through
thickness.** A Tet4 is constant-strain with flat faces; across a single
element it cannot represent through-thickness bending at all. Tet10 gives
quadratic variation through the *same* element, buying bending fidelity
without the ≈8× tet-count inflation uniform refinement would cost
(`04-rim.md`).

## 5. Friction — a hard constraint I mis-stated earlier

**Tet10 face contact is frictionless, enforced by a panic**
(`sim/L0/soft/src/solver/backward_euler/assembly.rs:358`):

```rust
assert!(
    !matches!(pair, crate::contact::ContactPair::Face { .. }),
    "Tet10 face contact with friction (μ = {mu}) is not reconciled — rung 8b ships \
     face contact frictionless (set friction_mu = 0); face-friction is a deferred rung",
);
```

The smoothed-Coulomb model reads a per-vertex `λ = |force|` from each gradient
contribution, which a face pair's six distributed forces do not satisfy.

✅ **Measured correction (2026-09-22): this costs the bridge NOTHING today.**
`friction` appears **zero times** in all of `tools/cf-sim-research/src/`, and
`SolverConfig::skeleton()` sets `friction_mu: 0.0`
(`solver/backward_euler/config.rs:369`) — so `insertion_sim` is **already
frictionless**. The assert forecloses a future gain; it does not cause a
regression, and this section should not be read as though it did.

⇒ **"Adopting IPC gets you friction" is true on Tet4 and false on the Tet10
face path.** For an insertion device friction is not a detail — it is a large
part of the transmitted axial force the stiffness-floor reward term measures —
so face-friction reconciliation is on this product's critical path even though
it is a deferred rung upstream.

## 6. Newton-convergence expectation

`40-contact/04-multi-layer/02-thin-material.md` gives the governing rule and
the cost:

> **`d̂ < ℓ / 2`** — the barrier's active band extends `d̂` inward from each
> surface; below `ℓ ≲ 2d̂` both barriers are active at interior vertices, the
> `b''` contributions add, and the Hessian condition number spikes.
>
> *"Thin-material scenes typically need **2–5× more Newton iterations per
> timestep**."*

**Applied to row 24's stack: `ℓ_min = 4 mm` ⇒ `d̂ < 2 mm`.** That is
comfortably outside the pathological regime, so the 2–5× multiplier is **not**
expected to apply at these thicknesses. It *will* apply if a production sleeve
goes to sub-millimetre layers.

⚠ The book flags `d̂ < ℓ/2` as *"a practitioner-level engineering heuristic
rather than a published theorem"* — Li et al. 2020 treats `d̂` as a global
scalar chosen from the scene length scale and does not analyse the thin-layer
regime; C-IPC covers codim-1 shells and codim-2 rods, not this codim-0 thin
volumetric case.

## 7. ⛔ What this recon did NOT measure

Listed because the confidence of §4 rests on these being open, not closed.

1. **No Tet10 solve has been run.** Every type-level result above is a compile
   probe. Nothing here measures convergence, residuals, or wall-clock.
2. **Tet10 × Yeoh is unprecedented in both artifacts.** Yeoh appears in 3 of
   310 study files and **none pair it with element order**. The book's
   constitutive ladder is NH → Mooney-Rivlin → **Ogden**, with Ogden
   *"dominating Ecoflex curve fits above 100% strain"*; `sim-soft` implements
   NH and Yeoh and **no Ogden**. Row 23 pivoted to Yeoh because
   *"Mooney-Rivlin was math-falsified on Smooth-On TDS data"* — a code-side
   pragmatic choice the book never blessed. Row 23's 8 mm stretches
   `[2.06, 1.22, 0.073]` sit **above** 100% strain, i.e. in the regime the book
   assigns to Ogden. Whether Yeoh is adequate there is **not determined here**.
3. **Newton cost at Tet10 is unmeasured for this scene.** Row 23 needed 77
   Newton iterations at step 16 (cap 150) with Tet4 + Yeoh + penalty. Tet10 has
   ~2.5× the nonzeros and substitutes a log barrier for a penalty spring. The
   §6 rule says the *thin-material* multiplier should not fire; it says nothing
   about the element-order cost.
4. **Midside projection folds elements, and Gauss-point positivity cannot see
   it.** `mesh/tet10_mesh.rs:478-500` records the measurement:

   ```text
   input     : 624 tets | corner-folded 0 | worst corner ratio  1.000000
   projected : 624 tets | corner-folded 8 | worst corner ratio -0.051117
   ```

   *"All eight folded elements kept **every Gauss point positive** (best of the
   folded read +0.309), which is why the old gate passed... **The four-point
   rule cannot see a fold confined to a corner region.**"* The cure is a
   `quality_floor` held as a *fraction of original quality*, not a positivity
   test — *"a bare positivity bar is the `det J → 0⁺` boundary"*, and under one
   *"twelve of its eighteen folded elements sat at a ratio of precisely its
   `quality_floor`."*

   ⇒ 1.3 % of elements folded on a **624-tet canonical sphere**. A scan-derived
   cavity has far more curvature variation, and no bound for that case is
   established here. ⛔ **Do not gate this with `det J > 0`.**
   (Also at `sim/L0/soft/tests/tet10_sdf_projection.rs:15,48`.)
5. **Shear locking is unexamined.** `03-higher-order.md` notes Tet10 under
   bending can shear-lock, requiring B-bar or Bathe-Dvorkin, and puts it out of
   scope. A sleeve wall in flexure is a bending problem.
6. **Material-field sampling at Tet10 is unverified.**
   `BuildableFromField::cache_from_field` samples at **tet centroids** and its
   own doc says *"the Tet4 default — Phase H Tet10 will sample at four Gauss
   points instead and bypass this helper"*
   (`material/material_field.rs:440-448`). Row 24's axial zoning and any future
   filler field depend on this.
7. **`cf-sim-research` is `[[bin]]`-only**; `insertion_sim` is `pub(crate)`
   with a module-level `#![allow(dead_code)]`. It cannot be consumed by
   cf-studio and has no library API surface for `cargo xtask grade`. That is a
   packaging question this recon does not answer.

## 8. Work items — ordered by *when each becomes measurable*

> ⚠ **Reordered after the blast radius of item 4 was measured.** The original
> ordering put per-Gauss-point material sampling second, on the reasoning that
> breaking changes are cheapest while consumers are few. That reasoning does
> not survive the count: `Mesh::materials()` has **4 implementors but 119 call
> sites**, and wiring the reward adds **zero** new `materials()` consumers
> (it reads contact and stress fields). So deferring the breaking change costs
> nothing — while doing it first means spending 119 edits on a change whose
> benefit cannot yet be measured.
>
> **The ordering principle is therefore: build the oracle, then change things
> against it.**

1. ✅ **`impl Mesh<M> for Tet10Mesh`** — generalise off the `NeoHookean`
   default. Additive (a defaulted type parameter), zero downstream breakage.
   Gates everything below: `Tet10Mesh<Yeoh>` is what pairs the rung-8b face
   barrier with the only constitutive model that reaches the 8 mm target.

2. **Wire `RewardBreakdown`** — §1's four terms, computed on the insertion
   result. Additive. This is the step that gives example row 25
   (`...-open-mouth`) — the geometry closest to the product — its first physics
   oracle; it is currently a **demo, not a validator**, because the
   interference-fit load case inverts the force sign and strain ordering that
   its sibling rows gate on. ⛔ **Until this lands, no solver change below is
   measurable — only different.**

3. **`SdfMeshedTetMesh` → `Tet10Mesh` bridge inside `insertion_sim`**, plus
   fresh `d̂` / `κ`: IPC barrier parameters do not carry over from penalty's
   smoothing and normal-averaging tuning. This is the renovation proper, and
   item 2 is what says whether it improved anything.

   ✅ **`κ` is now derived rather than swept**, and the arithmetic is shared
   code, not a note: `sim/L0/soft/src/contact/barrier.rs` holds the one
   implementation of `b`/`b'`/`b''` (the solver calls it too) plus
   `face_barrier_standoff` / `face_barrier_kappa`. Because the face barrier
   integrates `κ·b` over the rest area with weights summing to 1, **`κ·|b'(d)|`
   is a traction in pascals** — so `κ` is fixed by the traction the scene
   carries and the standoff the march needs, between a floor
   (`σ/|b'(ρ · ramp step)|`, ρ below) and a ceiling (`σ/|b'(d̂/2)|`).
   ⚠ The floor is **derived** from a measured stall mechanism; the ceiling is a
   **stated requirement** — `d̂/2` is a round number, and the decade selection
   survives anywhere in roughly `[d̂/2.8, d̂/1.4]`.
   `kappa_is_derived_and_not_swept` in
   `sim/L0/soft/tests/tet10_yeoh_ipc_convergence.rs` evaluates both on every
   build.

   ⚠ **Three things that do not carry to the bridge unexamined.** The floor is
   set by the *marching increment*, not the physics, so a different ramp
   schedule moves it. It also divides by `|b'|` at `ρ · step`, where `ρ` is the
   contact patch's **non-uniformity** (`d_eff / min_sd`, measured 1.04–1.26
   here). And the interval is `|b'(ρ·step)| / |b'(d̂/2)|` = **9.5×** wide
   regardless of `σ`, so it selects a decade only while the design traction is
   known to within roughly `[0.47×, 4.5×]`; `insertion_sim`'s traction is its
   own measurement, not this fixture's **30.4 kPa**.

   ✅ **`ρ` was the one of those three that could be checked before the bridge,
   and it was — on an enveloping cell, where an earlier revision of this line
   said a closing cavity *"has no reason to share that number."* It shares it.**
   A 10 mm spherical bore in a 12 mm wall, driven by a growing rigid sphere,
   reports `ρ ∈ [1.176, 1.221]` all the way to its convergence wall — under the
   `1.30` the floor is derived with, and *tighter* than the flat plate's own
   1.04–1.26 spread. ⚠ That is one idealised cavity: a sphere has a uniform
   gap by symmetry, so this measures the discretisation's contribution to `ρ`
   and not a scan-derived cavity's shape irregularity, which remains the
   bridge's risk.

   ⛔ **What did NOT survive is `σ`'s definition.** `F_z / A_flat` needs a net
   force projected onto an axis, and on a closed patch every normal is radial:
   measured, the enveloping cell's `‖ΣF‖ / Σ‖f‖` is **1.11e-3 falling to 1.45e-4**
   against the plate's 0.9992, so the net force understates its contact by three
   to four orders of magnitude. There is no cavity analogue of `A_flat` because there is
   no force to divide. The replacement is to stop going through force at all and
   read the traction the barrier integrates — the area-weighted mean of
   `κ·|b'(sd)|` — which tracks `F_z / A_flat` on the plate at a ratio constant
   to **0.9360–0.9365 over thirteen rungs**.

   ✅ **GRADED MATERIALS (item 3b) — measured, and both readings above
   survive.** The same shell now also carries `insertion_sim`'s row-23 stack
   (Ecoflex 00-20 / Dragon Skin 10A / Dragon Skin 20A, innermost first) through
   the same `LayeredScalarField` keyed on the cavity SDF. Grading costs **32 %
   of the depth** — the wall moves from 4.720 mm of radial interference to
   3.220 mm — and moves the stall mode with it: the uniform cell ends at
   `ArmijoStall(iter 5)`, which is *not* the marching-feasibility mode the `κ`
   floor is derived against, while the graded cell ends at **iter 0**, which
   is. ⇒ on a graded wall the derived floor describes the failure it was built
   to describe. What does **not** change: `ρ` stays inside `1.30`
   (**[1.1632, 1.2115]**, a band about 4 % wider than uniform) and the
   enveloping patch still cancels its own net force (**1.068e-3 → 1.281e-4**).
   Both readings are geometric, not artefacts of material uniformity, so the
   bridge can lean on them over a layered wall.

   ⚠⚠ **A volume-weighted modulus is the wrong estimator for a graded wall.**
   The stack volume-averages to **3.48×** the uniform anchor; the measured
   stiffening is **1.222× at rest rising to 1.719×** at the wall. Load enters
   at the bore, where the stack is *softer* than the baseline (18 kPa against
   23 kPa), and the stiff shells carrying 56 % of the volume sit against the
   pinned skin — the layers load in **series**, not in parallel.
   ⭐⭐ **That mechanism is measured, not asserted.** A volume average is
   **order-blind** — the stack and its inverse weight identically — so
   inverting the stack is a real falsifier for a claim about position.
   Measured: **1.3320 → 2.3104**, the excess over unity going 0.3320 → 1.3104.
   ⇒ the same three anchors in the same volume fractions give **74 % different
   stiffening by ARRANGEMENT ALONE**, which is a sharper indictment of
   volume-averaging than the 2× discrepancy on its own.
   ⭐ The finding does not depend on which modulus is averaged: volume-weighting
   gives 3.476× for `μ`, 3.476× for `λ` and 3.453× for `C₂`, because the
   anchors are a **self-similar family** (`λ = 4μ` exactly, `C₂ ≈ 0.089 μ`,
   ν = 0.400 throughout).

   ⚠⚠ **RETRACTED, and the retraction is the useful part.** An earlier revision
   of this paragraph said *"`μ` is not even the dominant term — flattening `μ`
   moves the ratio 0.049 where flattening `λ` moves it 0.097, and `λ` leads
   because the shell is sealed."* Both halves are wrong. Because `λ = 4μ` for
   every anchor, holding one Lamé parameter fixed while the other grades
   produces a body whose **ν varies by layer** (0.379 → 0.476, or 0.418 →
   0.224) — not a silicone, and not "the same material with one influence
   removed". There is no `μ`-versus-`λ` split to make on this stack.
   ✅ The split that IS physical holds ν fixed — `(μ, λ)` together against
   `C₂` — and the linear modulus carries **18.5×** what `C₂` does
   (excess above unity: 0.3320 baseline, 0.0566 with the linear part
   flattened, 0.3171 with `C₂` flattened). ⇒ **the stiffening is
   linear-elastic**, not a Yeoh-nonlinearity effect, at these stretches.

   ⚠ **The open-mouth caveat stands on its own evidence, not on that
   reasoning.** Item 3a independently measured that the sealed cell forces
   22.7 % volumetric compression at depth and is stiffer than the open-mouth
   sleeve it proxies. ⇒ **read the 1.22–1.72× band as an upper bound** because
   the CELL is stiffer, not because of anything about which parameter carries
   the grading.

   ✅✅ **STEP 0 — THE BASELINE, MEASURED (2026-09-22 at `a0cfa901`).** All
   three `#[ignore]`d ramps reach **16/16 to their full 3.00 mm inset** in 67 s
   release. ⚠ The archive's 31 % / 2.62 mm figures are
   **pre-N3** — the Gaussian pre-smooth shipped as
   `GRID_SDF_SMOOTH_SIGMA_CELLS` = 1.0 in slice 7.3d and took the scan to 100 %.
   They should not be quoted as current.

   ⚠ **"Reaches 16/16" is not yet "at ceiling", and the first revision of this
   section stated the conclusion from the weaker reading.** All three ramps run
   at a **3 mm** inset; `cf_device_types::CAVITY_INSET_SLIDER_MAX_M` is
   **8 mm**. Reaching a requested 3 mm says nothing about 8 mm. Measured
   across the rest of the range (synthetic icosphere, release):

   | inset | 3 mm | 4 mm | 5 mm | 6 mm | 7 mm | 8 mm |
   |---|---|---|---|---|---|---|
   | tets | 45 654 | 45 156 | 42 981 | 38 736 | 37 884 | 35 670 |
   | steps | 16/16 | 16/16 | 16/16 | 16/16 | 16/16 | 16/16 |

   ✅ **Now** the depth envelope is at ceiling — across the whole product
   slider range, not at one point in it. Pinned by
   `the_ramp_converges_across_the_whole_cavity_slider_range`.
   ⭐ It also falsifies a claim in shipped code:
   `INSERTION_CONTACT_SMOOTHING_EPS_M`'s docstring recorded *"converges 16/16 at
   cavity ≤ 5 mm but stalls at cavity 6 mm"* and named the UI slider as *"the
   cap that enforces this bound"*. Both are pre-pre-smooth history — 6, 7 and
   8 mm all converge, and that cap is 8 mm and never enforced 5 mm. Corrected
   at the const.

   ⭐⭐⭐ **What replaces it is a SPLIT result.** Re-running the same three
   ramps at `tol` = 1e-6 instead of the shipped `INSERTION_SOLVE_TOL` = 1e-1:

   | fixture | at 1e-1 | at 1e-6 |
   |---|---|---|
   | analytical sphere shell (46 584 tets) | 16/16 @ 3.00 mm, but **13 of 16 steps take ONE Newton iteration** and the residual *rises* 3.28e-2 → 6.50e-2 | 16/16 @ 3.00 mm, 4–7 iters, ~1e-7 |
   | synthetic icosphere | 16/16 @ 3.00 mm | 16/16 @ 3.00 mm, iters spiking **73 / 35 / 73** |
   | **real iter-1 scan (68 087 tets)** | **16/16 @ 3.00 mm** | ⛔ **stalls at step 4 — 0.75 mm, 25 % of the inset** — Armijo stall at Newton iter 3, `r_norm` **4.13e-3** |

   ⚠⚠ **That scan row is ONE of two scenes, and not the product's.** It is
   `run_insertion_ramp_on_iter1_scan` — a SINGLE Ecoflex 00-30 layer with NO cap
   planes, which routes `pinned_floor_shell` through the closed-cavity
   short-circuit. The GUI default is the dual-layer stack with the `prep.toml`
   cap plane (72 935 tets) and does not reach 16/16 even at `tol` = 1e-1. Both
   are measured below, under "`σ` AND `ρ` ARE NOW MEASURED" — which also finds
   that this row's full depth is reached **through** the wall, not against it.

   ⇒ **On the single-layer scan the full-depth result is bought with the
   tolerance: requiring a converged solution costs 4× the usable depth.** On
   both idealised fixtures it costs *iterations*, not depth, and does not move
   the answer — the icosphere's final step reads F 0.67 → 0.64 N, identical
   `λ` ∈ [0.447, 1.239], max ‖P‖ 1.66e5 either way.

   ⚠⚠ **This disqualifies the analytical sphere shell as *the* baseline.** It
   was built in May to isolate SDF smoothness and it does that job — but it is
   **too well-conditioned to see this failure**, so inheriting it as the
   reference hides the only reading that matters. A fixture is fit for one
   question; re-qualify it before reusing it for the next.

   ✅ **Pinned in CI, on a fixture found by search rather than taste.** The
   claim lives on the repo-excluded scan, so it cannot gate directly. A
   **9 258-tet** synthetic scene (20 mm radius, 8 mm wall, 3 mm inset, 4 mm
   cell) carries it instead:
   `the_insertion_solves_convergence_is_bounded_by_its_tolerance` measures what
   the solver reaches when asked (1e-6), what it returns at the shipped
   tolerance on the *same* scene and depth, and the gap between them —
   **2.3e5×**, measured 5.339e-2 against 2.286e-7. Deeper, the shipped
   tolerance accepts a residual above 1e-3 outright.
   `the_tolerance_knob_changes_only_the_tolerance` holds the delegation by
   Debug-equality, so a field a future `SolverConfig` adds cannot slip it, and
   `the_shipped_entry_point_solves_at_the_shipped_tolerance` holds the other
   half of that seam — *which* tolerance the shipped path passes. ⚠ That one
   exists because its absence was a surviving mutation: retuning the shipped
   default to 1e-6 passed the whole 107-test suite.
   ⛔⛔ **WHAT THAT GATE IS NOT: a measure of the bridge.** An earlier revision
   of this section said it was *"meant to be rewritten when the bridge lands;
   that rewrite's diff is the payoff."* Wrong. The gap is a property of
   `INSERTION_SOLVE_TOL` measured against achievable precision, and **the bridge
   does not change that constant** — a better-conditioned solver would if
   anything reach further below 1e-6 and make the ratio **larger**. The gate is
   expected to sit still.
   ✅ **The quantity the bridge must actually improve is the deepest
   interference solvable at a TIGHT tolerance** — which is exactly the
   platform-dependent one below, so it is **reported by a diagnostic on a named
   platform, not gated**
   (`the_deep_steps_stall_boundary_is_platform_dependent`).

   ⛔⛔ **AND THE THING THAT GATE MAY NOT ASSERT: WHERE THE STALL FALLS.** The
   first revision of it asserted that the same scene, 0.3 mm deeper, *could
   not* reach 1e-3 at all. That held on macOS/ARM — Armijo stall at Newton
   iter 108, `r_norm` **2.78e-3**, the same mode and decade as the scan's
   4.13e-3, which is exactly why the fixture looked worth having — and
   **failed on Linux/x86 CI**, where the identical commit drove the residual
   past 1.44e-3 by iter 39 and kept going. The `faer` LU fallback fires on both
   (a non-SPD tangent at a few recurring pivots); which side of the Armijo edge
   that lands on is decided by arithmetic a test cannot pin.
   ⇒ **assert the SIZE of the gap, which is a ratio, not the LOCATION of the
   edge, which is a threshold.** Recorded as the `#[ignore]`d diagnostic
   `the_deep_steps_stall_boundary_is_platform_dependent`.
   ⚠⚠ **This conditions the table above**: the real scan's stall at `tol` =
   1e-6 is likewise a **single-platform measurement** (macOS/ARM) and should be
   read as one. It is far deeper into failure — step 4 of 16, not a marginal
   edge — but it has not been reproduced on a second platform.

   ✅✅ **`σ` AND `ρ` ARE NOW MEASURED FOR THIS SCENE — and the thing that
   decides whether they are usable is not the contact model but WHERE they are
   read: at the shipped `κ` the patch is through the wall, and on the wrong
   area basis `σ` is 22 % low.** Three `#[ignore]`d probes in
   `tools/cf-sim-research/src/insertion_sim.rs` read the area-weighted mean
   traction `Σ‖f‖ / Σa` and the gap distribution off every converged step of a
   16-step ramp:
   `the_bridges_design_traction_and_patch_nonuniformity_on_the_synthetic_sphere`,
   `…_on_the_real_scan` (two scenes), and
   `the_design_traction_is_measured_against_the_stiffness_that_produced_it`.

   ⚠⚠ **EVERY NUMBER BELOW IS A SINGLE-PLATFORM MEASUREMENT (macOS/ARM,
   2026-09-22)**, and this document already records why that matters: the
   identical commit stalled at Armijo iter 108 here and ran past it on
   Linux/x86. Two readings below are **stall boundaries** and should be read as
   this machine's — `κ` = 1e6 converging *no* step, and the 4/16-vs-16/16 step
   counts. The tractions and gap ratios are converged-state quantities and are
   the more portable half; none of it has been reproduced on a second
   platform.

   ⭐ **Every number this section MEASURED is regenerated by three commands.**
   They are the referent: a figure here that the probes no longer print is a
   figure that has drifted, and one quoted without re-running them is a claim
   about a run nobody did — which is how a `1.003×` that exists in no source
   file got into an earlier revision of this section.

   ⚠ **Three figures below are NOT among them**, and are cited rather than
   measured here: the enveloping cell's `ρ ∈ [1.176, 1.221]` and its
   `Σ‖f‖/‖Σf‖` of 900–6900×, both from *other* probes in
   `tet10_yeoh_ipc_convergence`, and the `1.30` floor constant, which is
   `PATCH_NONUNIFORMITY` in that file and is checkable by reading it. Running
   the three commands will not reproduce those, and an earlier revision of this
   very block said it would.

   ```text
   cargo test -p cf-sim-research --release --bin cf-sim-research \
       the_bridges_design_traction_and_patch_nonuniformity -- --ignored --nocapture
   cargo test -p cf-sim-research --release --bin cf-sim-research \
       the_design_traction_is_measured_against_the_stiffness -- --ignored --nocapture
   cargo test -p sim-soft --release --test tet10_yeoh_ipc_convergence \
       is_the_contact_traction_a_property_of_the_scene_or_of_kappa -- --ignored --nocapture
   ```

   ⚠ The third is the **fixture's own** probe, not this module's. Its answer is
   stored in no source file — it exists only in a run — which is why the
   comparison against it carries the command rather than a remembered value.

   ⚠⚠ **`σ` MOVES WITH THE CONTACT STIFFNESS — on this scene AND on the
   fixture — and an earlier revision of this section got the comparison
   backwards.** It claimed *"σ spans 3.77× where the fixture's spans 1.003×"*.
   Both halves were wrong:

   - **The `1.003×` was never measured.** It appears in no source file. Running
     the fixture's own probe
     (`is_the_contact_traction_a_property_of_the_scene_or_of_kappa`, 123 s)
     prints **1.532×** over `κ` spanning 100×, at a common plane height, all
     three arms non-penetrating.
   - **The `3.77×` was not comparable to it.** It spans three decades against
     the fixture's two, and it includes arms whose patch is driven *through*
     the intruder — states this same section calls *"not a traction on
     anything"*.

   ✅ **Measured per decade over the SAME arm set on both scenes** — the two
   stiffest that solve, `κ` = 1e4→1e5, which are the only arms fully seated on
   both — on the **rest**-area basis the derivation consumes:

   | | fixture | sphere | 1-layer scan |
   |---|---|---|---|
   | arms compared | 1e6→1e8 | **1e4→1e5** | **1e4→1e5** |
   | **`σ` per decade** | **1.238×** | **1.047×** | **1.048×** |

   ⇒ **`σ`'s coupling to `κ` here is TIGHTER than the fixture's, not looser**,
   and the two scenes agree with each other to **0.12 %**.
   ⚠ **Widening the sphere's set to 1e3→1e5 gives 1.198× per decade instead**,
   because its `κ` = 1e3 arm is only marginally seated (`min_sd` +0.054 mm, 8
   pairs already at zero tributary area, `ρ` 4.92) — so a per-scene "seated"
   predicate picks a different arm set on each scene and its numbers do not
   compare between them. The probe labels which of its two figures is
   cross-scene for exactly this reason.
   The fixture's own docstring says the coupling is expected — *"It does move a
   little, and it must"* — and that what matters is its size against the spread
   of `κ` driving it. By that test this scene is the better-behaved one.

   ⛔ **What IS a real problem is where the shipped `κ` reads it.** At
   `κ` = 1e3 the single-layer scan's patch is **through the wall**
   (`min_sd` −0.373 mm), so σ there is not the traction of a seated state at
   all. That is a *penetration* finding, not a coupling finding, and it is the
   one that disqualifies the shipped reading.

   | penalty `κ` | steps | `σ`(rest) kPa | `min_sd` mm | `ρ` | seated |
   |---|---|---|---|---|---|
   | 1e2 | 16/16 | 27.70 | **−1.755** | 0.788 | ⛔ through |
   | 1e3 (shipped) | 16/16 | 81.51 | 0.0544 | 4.923 | barely |
   | 1e4 | 16/16 | 111.78 | 0.8517 | 1.054 | ✅ |
   | 1e5 | 16/16 | **117.01** | 0.9841 | 1.005 | ✅ |
   | 1e6 | **0/16** | — | — | — | stalls at step 0 |

   ⚠ **The stiff limit is not reachable on this path.** `κ` = 1e6 stalls at
   **step 0** on both scenes, so 1e5 is the stiffest that solves and `σ` is
   still moving **4.7 % (sphere) / 4.8 % (scan)** from 1e4 ⇒ **a lower bound
   approaching ~117 kPa**, not a
   converged rigid-contact traction.

   ⛔⛔ **`σ` must be read on the REST-area basis.**
   `ContactPairReadout::tributary_area` is the
   **deformed** tributary and sim-soft says at the source that *"the
   surface-integrated barrier is weighted by the face's rest area, so the
   barrier weight and this pressure tributary are deliberately different
   measures"*; `face_barrier_kappa` inverts the rest-normalised relation, and
   the fixture's own reading accumulates `flat_area += *rest_area`. Measured,
   the two bases differ by **22–23 %** at full depth (117.01 against 94.98 kPa
   on the sphere) — a bias straight into a shipped constant. Both are now
   reported, and the derivation consumes the rest one.

   ✅✅ **The same sweep on the REAL SCAN agrees, and it also says what the
   penetration above actually is.** Single-layer scene, 68 087 tets, full 3 mm:

   | penalty `κ` | steps | `σ`(**rest**) kPa | `min_sd` (mm) | 5 % tail (mm) | `ρ(min)` | `ρ(tail)` |
   |---|---|---|---|---|---|---|
   | 1e2 | 16/16 | 28.74 | −2.059 | −1.733 | — | — |
   | 1e3 (shipped) | 16/16 | 83.23 | **−0.373** | **−0.042** | — | — |
   | **1e4** | 16/16 | 114.35 | **+0.760** | +0.834 | **1.177** | **1.073** |
   | 1e5 | 16/16 | **119.84** | +0.972 | +0.982 | 1.017 | 1.007 |
   | 1e6 | **0/16** | — | — | — | — | — |

   ⭐⭐ **`κ` = 1e4 reaches the full inset WITHOUT penetrating.** The −0.373 mm
   at the shipped `κ` is not the geometry refusing to seat — it is the penalty
   being **an order of magnitude too soft to hold the wall out**, and one decade
   of stiffening removes it entirely while reaching the same 16/16.

   ⭐ **The two scenes agree to within their own noise**, which is the evidence
   that this is a property of the path rather than of either fixture: `σ` spans
   **1.048× per decade** (scan) against **1.047×** (sphere) over the SAME arms
   (`κ` = 1e4→1e5, the only ones seated on both), agreeing to 0.12 % and both
   well under the fixture's 1.238×; `κ` = 1e6 stalls on both. ⇒ **for the
   bridge, `σ` ≈ 117 kPa as a lower bound and `ρ` ∈ [1.00, 1.18]** — ⛔⛔ **SUPERSEDED: that is the 3 mm Ecoflex design both scenes above share, not the product. On the PRODUCT scan (`base_mold`) it is σ = 58.9 kPa and ρ ∈ [1.00, 1.11]; see `σ RE-MEASURED ON THE PRODUCT SCAN`** — read on the
   REST basis at `κ` = 1e4–1e5 on a non-penetrating full-depth seat — not the
   6.85 kPa / 1.22 the shipped stiffness reports off a 25 % seat.

   ⭐⭐⭐ **THE DERIVATION AT THOSE NUMBERS** — ⛔⛔ **"those numbers" are
   SUPERSEDED**: the two 3 mm Ecoflex scenes above, at a σ about 2× the one
   that ships. Kept for its *shape*, not its values. Evaluated at the stiffest
   arm that solved on each scene, `ramp step` = 0.1875 mm:

   | scene | `σ` | `ρ(tail)` | `d̂` = 1.0 mm | `d̂` = 1.2 mm | `d̂` = 2.0 mm |
   |---|---|---|---|---|---|
   | sphere @ 1e5 | 117.01 kPa | 1.004 | [1.88e7, 9.81e7] | [1.27e7, 8.17e7] | [4.50e6, 4.90e7] ✅ |
   | **1-layer scan @ 1e5** | 119.84 kPa | 1.007 | [1.94e7, 1.00e8] | **[1.31e7, 8.37e7]** | [4.62e6, 5.02e7] ✅ |

   ⇒ **`κ` for the bridge is ≈ 1.3e7–1.9e7 at `d̂` = 1.0–1.2 mm**, or `1e7` fits
   at `d̂` = 2 mm (✅ marks the bands whose interval contains `1e7`).

   ⛔ **The fixture's `1e7` sits 24 % BELOW the scan's floor at the fixture's
   own band** — 1e7 against 1.31e7 at `d̂` = 1.2 mm — and below is the side that
   fails: it is the value that does not hold one ramp increment open, which is
   exactly the mechanism the floor exists to exclude. ⚠ Two effects pull
   against each other here and the larger wins: this scene needs a 0.191 mm
   standoff against the fixture's 0.13 mm, and a wider standoff needs *less*
   `κ` — but `σ` differing **3.94×** (119.84 against 30.4 kPa) dominates it.

   ⭐ **The same sweep disposes of a `ρ` this document would otherwise have
   inherited.** At the shipped `κ` the sphere reports `ρ` = **4.92** at full
   depth, which reads like shape irregularity on a geometry that has none by
   symmetry. It is not: the same pose at `κ = 1e5` reports **1.005**. The 4.92
   was a soft contact letting one region sink.

   ⛔ **`min_sd` is ONE VERTEX**, and the floor divides `|b'|` at `ρ · step`, so
   one bad tet on a scan-derived patch moves a shipped constant. `ρ` is now
   reported against the **area-weighted 5 % tail** beside the minimum, and the
   two are not close where it matters — sphere at full depth **4.923 vs 2.349**,
   which at `d̂` = 1 mm is the difference between a floor of 3.65e9 (interval
   **EMPTY**) and 4.20e7 (0.13 decades wide).

   ⛔⛔ **The `16/16 @ 3.00 mm` scan row above is a PENETRATING state.**
   Re-measured on the single-layer, cap-plane-free scene it was read off — the
   probe reproduces its **68 087 tets** exactly — `min_sd` crosses zero at step
   10 and reaches **−0.373 mm** at full depth, and the **5 % area tail** is
   at **−0.042 mm**, so this is a *region* through the wall and not an outlier
   vertex. **94** of 3 181 pairs have lost their tributary area entirely. `ρ`
   is negative there and the probe prints `undefined` for every bound rather
   than the arithmetic. ⇒ read that row as *the ramp completed*, not as *the
   wall seated*. ✅ **And it is the STIFFNESS, not the geometry** — the sweep
   above seats this same scene cleanly at `κ` = 1e4.

   ⚠⚠ **That row is also about a scene the product does not run.** The GUI
   default — the dual-layer Ecoflex 00-30 + 50 % Slacker 10 mm / DS20A 3 mm
   stack with the `prep.toml` cap plane, **72 935 tets** — reaches **step 4 of
   16: 0.75 mm, 25 % of the inset**, at the *shipped* `INSERTION_SOLVE_TOL` =
   1e-1. Armijo at Newton iter 136, `r_norm` **1.2131694152574887e-1**,
   hovering just above the tolerance it is judged by, and reproduced
   bit-identically across two runs.

   ✅ **On that scene the derivation closes, and it is the most trustworthy `ρ`
   of the three** — the patch is barely non-uniform, so the two definitions
   nearly agree. `σ`(rest) = **6.85 kPa**, `ρ(min)` = **1.223**, `ρ(tail)` =
   **1.086**, ramp step 0.1875 mm:

   | `d̂` (mm) | floor | ceiling | decades | holds 1e7 |
   |---|---|---|---|---|
   | **0.5** | 9.14e6 | 1.14e7 | 0.09 | **yes** |
   | 1.0 | 1.39e6 | 5.68e6 | 0.61 | no |
   | 1.2 | 9.26e5 | 4.74e6 | 0.71 | no |
   | 2.0 | 3.18e5 | 2.84e6 | 0.95 | no |

   ⇒ at `d̂` = 0.5 mm the interval contains `1e7`; at the fixture's 1.2 mm it
   does not, and `1e7` sits **above** the ceiling.

   ✅ **This closes the `ρ` risk this section named.** The enveloping-cell check
   above could only measure *the discretisation's* contribution, because a sphere
   has a uniform gap by symmetry, and left *a scan-derived cavity's shape
   irregularity* open as the bridge's own risk. Measured: the scan's `ρ` is
   **1.073–1.223**, which **straddles** the idealised cell's `[1.176, 1.221]`
   rather than sitting inside it: the *area-tail* measure reads **1.073–1.086**,
   **below** the cell's band, and the *minimum* measure **1.177–1.223**, inside
   it bar a hair at the top. Both sit under the `1.30` the floor is derived
   with, and below is the favourable direction — a smaller ρ is a smaller
   required standoff and a lower floor. **Shape irregularity costs essentially
   nothing here** — what does cost is the contact stiffness the number is read at, which
   was not on the list.
   ⚠⚠ **But do not carry these two numbers to the bridge.** They are read at the
   shipped stiffness — the reading the sweep above disqualifies — off a seat that
   reached 25 % of its inset. The ones to carry are the stiff-arm numbers: `σ`
   ≈ 117 kPa, `ρ` ∈ [1.00, 1.18] — ⛔ both SUPERSEDED by the product scan (58.9 kPa, [1.00, 1.11]). What this table shows is the *shape* of the
   derivation on a real scan — a narrow interval that closes only at a small `d̂`.

   ✅ **The enveloping-patch cancellation, quantified inside this pipeline.**
   `Σ‖f‖/‖Σf‖` measures **1588–3095×** on the synthetic sphere, **214–455×** on
   the single-layer scan and **9.2→7.9×** on the GUI default, against the
   idealised cell's 900–6900×. ⚠ Only the last is monotone in depth; the other
   two are quoted as ranges because they are not.

   ⛔ **Its cause is UNIDENTIFIED, and the ordering invites a guess that the
   data does not support.** `gui-dflt` and `1layer` are the **same scan** and
   differ **52×** at the **same depth** (7.92 against 410.71 at 0.75 mm) while
   differing in *two* things at once — layer stack and cap-plane topology — so
   nothing here isolates either. An earlier revision named "how enclosing the
   patch is"; no enclosure metric was ever measured. ⇒ **the spread is
   measured; its cause is not.** What stands is the operational point: the F-d
   curve is a correct *net seating resistance* and cannot be divided by an
   area.

   ⚠ **What the marching schedule can and cannot fix.** The floor is the only
   bound that moves with the increment, so a finer march widens the interval —
   on the GUI default at `d̂` = 1 mm, 16 steps gives 0.61 decades and 512 gives
   2.09. It never brings `1e7` inside at that band, because the *ceiling* is
   fixed by `σ` and `d̂`. ⇒ **the band is the lever on the ceiling; the schedule
   is the lever on the floor.**


   ---

   ### ⛔ THE BRIDGE, FIRST PASS — SUPERSEDED. Read the product-scan sections.

   > ⛔⛔ **EVERYTHING IN THIS SECTION IS SYNTHETIC SCENES AT A κ THAT NO LONGER
   > SHIPS.** Its tables are `tol-fixture` and `sphere-40mm`, and its κ was
   > derived from **σ = 117 kPa** (the 3 mm Ecoflex scenes of #959) by a **geometric-centre**
   > selector. Both were later replaced: σ is **58.9 kPa** on the product scan,
   > and κ is now the **ceiling**. Depth numbers here are not comparable to
   > anything current.
   >
   > ▶ **The current result is `THE HEAD-TO-HEAD, ON THE PRODUCT SCAN AT THE
   > FIXED κ`**, below. The reading order that matches how this was learned:
   > σ re-measured → floor falsified → κ fixed → head-to-head.
   >
   > ★ Kept rather than deleted because the synthetic scenes are what MISLED,
   > and that is the transferable part: they were too well-conditioned to show
   > either the failure or the benefit.

   ### ▶ THE BRIDGE ITSELF — first pass, synthetic scenes, 2026-09-22

   `run_insertion_ramp_tet10_ipc` runs the same scene, the same boundary
   conditions and the same rigid intruder through a swapped solve triple —
   `Tet4` → `Tet10`, `SdfMeshedTetMesh` → `Tet10Mesh`, `PenaltyRigidContact`
   → `IpcRigidContact` — with `κ` derived from the `σ` above rather than swept.
   The shipped `run_insertion_ramp` is untouched.

   ⭐⭐⭐ **THE RESULT, and it is what the bridge was justified on.** Both arms
   run the same 16-step ramp to a 3 mm inset at a ladder of residual
   tolerances. Depth alone is not the payoff quantity — #959 showed a ramp can
   reach full depth *by penetrating* — so `min_sd` and the area-weighted 5 %
   tail are reported beside it. **A depth reached through the wall is not a
   seat.**

   | scene | tol | arm | steps | depth | `min_sd` | 5 % tail | σ |
   |---|---|---|---|---|---|---|---|
   | tol-fixture | **1e-1** (ships) | tet4+penalty | 16/16 | 3.000 mm | **−0.306 mm** | **−0.213 mm** | 76.6 kPa |
   | tol-fixture | 1e-1 | **tet10+ipc** | 16/16 | 3.000 mm | **+0.184 mm** | **+0.235 mm** | 124.4 kPa |
   | tol-fixture | **1e-2** | tet4+penalty | **3/16** | 0.563 mm | +0.635 mm | +0.656 mm | 26.4 kPa |
   | tol-fixture | 1e-2 | **tet10+ipc** | **16/16** | 3.000 mm | +0.184 mm | +0.236 mm | 124.4 kPa |
   | tol-fixture | 1e-3 | tet4+penalty | 3/16 | 0.563 mm | +0.635 mm | +0.655 mm | 26.4 kPa |
   | tol-fixture | 1e-3 | tet10+ipc | **0/16** | — | — | — | — |
   | sphere-40mm | 1e-1 | tet4+penalty | 16/16 | 3.000 mm | +0.054 mm | +0.114 mm | 68.3 kPa |
   | sphere-40mm | 1e-1 | **tet10+ipc** | 16/16 | 3.000 mm | **+0.360 mm** | **+0.402 mm** | 88.6 kPa |
   | sphere-40mm | 1e-2 | tet4+penalty | 16/16 | 3.000 mm | +0.058 mm | +0.114 mm | 68.3 kPa |
   | sphere-40mm | 1e-2 | **tet10+ipc** | 16/16 | 3.000 mm | **+0.360 mm** | **+0.403 mm** | 88.6 kPa |
   | sphere-40mm | 1e-3 | tet4+penalty | 16/16 | 3.000 mm | +0.058 mm | +0.114 mm | 68.3 kPa |
   | sphere-40mm | 1e-3 | tet10+ipc | **5/16** | 0.938 mm | +0.619 mm | +0.655 mm | 32.0 kPa |
   | sphere-40mm | 1e-4 | tet4+penalty | 16/16 | 3.000 mm | +0.058 mm | +0.114 mm | 68.3 kPa |
   | sphere-40mm | 1e-4 | tet10+ipc | **0/16** | — | — | — | — |

   Two things fall out. ⚠ **They do not have the same reach, and saying so is
   the point** — one holds on both scenes, the other on one:

   1. ⭐⭐⭐ **The bridge holds a standoff the penalty path does not — on BOTH
      scenes.** At the shipped tolerance the baseline's `min_sd` is −0.306 mm
      on the tolerance fixture, with its 5 % area tail at −0.213 mm: a *region*
      through the wall, not an outlier. On the sphere it stays out, but by
      0.054 mm. The bridge holds **+0.184 mm** and **+0.360 mm** — **3.4× and
      6.6×** the baseline's clearance. ⇒ same depth, different contact state,
      and the difference is largest exactly where the baseline is worst.
   2. ⭐⭐ **On the tolerance fixture, one more decade of residual collapses the
      baseline's usable depth 5.3× and does not move the bridge's.** The
      penalty arm reaches 0.563 mm of the 3 mm inset at `tol` = 1e-2; the
      bridge still reaches 3.000 mm, and gives the *same answer* — `min_sd`
      +0.184 mm, σ 124.4 kPa at both 1e-1 and 1e-2, agreeing to four
      significant figures. ⇒ on that scene the payoff quantity — deepest
      interference solvable at a tight tolerance with `min_sd > 0` and the 5 %
      tail > 0 — goes **0.563 mm → 3.000 mm**.

      ⛔⛔ **AND THE SPHERE DOES NOT SHOW IT.** At `tol` = 1e-2 the penalty arm
      reaches 16/16 there too, with its answer unchanged to three significant
      figures. So claim 2 is a statement about the tolerance fixture, not about
      the two solve paths. ⇒ **do not state it more broadly than the scene it
      was measured on** — this is the same trap #958 recorded, where the
      analytical shell was *too well-conditioned to see the failure* and
      inheriting it as "the" baseline hid the only result that mattered. The
      scene that discriminates is the ill-conditioned one, and which scenes
      those are is not predictable from their geometry.

   ⛔⛔ **AND THE HONEST HALF, which decides that the default must NOT flip
   yet: the bridge COSTS tolerance headroom, and it is a cliff not a slope.**
   At `tol` = 1e-3 it reaches **0/16** on the tolerance fixture and **5/16** on
   the sphere, where the penalty path reaches 3/16 and **16/16**; at 1e-4 the
   penalty path still reaches 16/16 on the sphere at `r` = 9.45e-5. The bridge
   stalls inside its own approach at `r ≈ 1e-4`, "non-SPD tangent near
   solution". It either seats fully or not at all.

   ⇒ on the SYNTHETIC scenes the bridge buys CONTACT STATE and costs TOLERANCE
   HEADROOM. ⛔⛔ **That verdict does not survive the product geometry — see
   the next block, and read it before quoting the sentence above.**

   ⛔⛔ **TWO CANDIDATE CAUSES ARE MEASURED FALSE.** The cliff is not `κ` — see
   the sweep below — and it is not where the approach starts: moving the
   approach margin from `d̂/2` to a full `d̂`, so the first solve is genuinely
   unloaded (the barrier is inactive at `sd ≥ d̂`) rather than mid-band, leaves
   every converging row **identical to four significant figures** (`min_sd`
   +0.1841 / +0.1844, σ 124.39 / 124.37 kPa) and does not move the cliff at
   all. ⭐ That inertness is worth having on its own: it says the converged
   answer does not depend on where the march began, in the regime where the
   march works. The margin is kept at `d̂` because it is the derived choice —
   feasible by definition rather than by a chosen fraction — not because it
   bought anything.

   ▶ **The untried lead is the geometry, not the solver**: boundary midsides
   sit under the curved cavity surface by a measured sagitta, and
   `Tet10Mesh::with_curved_midsides` exists precisely to fix that. It is not
   applied here.

   ---

   ### ⚠ SOCK_OVER_CAPSULE — kept for the method, SUPERSEDED as "the product"

   ⛔⛔ **Read the σ block below before quoting any number here.** This whole
   section calls `sock_over_capsule` "the product geometry". It is not — the
   product scene is `base_mold`, and everything below was additionally
   measured at `κ` derived from a σ that was **2× too large**. The comparison
   METHOD stands; the scene attribution and the stiffness do not.

   Measured 2026-09-22, macOS/ARM, 1775 s, on the real iter-1 scan in **both**
   topologies. ⛔ The scan is repo-excluded, so nothing here can ever gate; this
   reports on a named platform.

   | scene | tol | arm | steps | depth | `min_sd` | 5 % tail | σ |
   |---|---|---|---|---|---|---|---|
   | 1layer (68 087 tets) | 1e-1 | tet4+penalty | 16/16 | 3.000 mm | **−0.373 mm** | **−0.042 mm** | 70.4 kPa |
   | 1layer | 1e-1 | **tet10+ipc** | 16/16 | 3.000 mm | **+0.094 mm** | **+0.347 mm** | 90.0 kPa |
   | 1layer | 1e-2 | tet4+penalty | 6/16 | 1.125 mm | +0.345 mm | +0.554 mm | 32.5 kPa |
   | 1layer | 1e-2 | **tet10+ipc** | **14/16** | **2.625 mm** | +0.130 mm | +0.390 mm | 76.3 kPa |
   | **gui-dflt** (72 935 tets) | **1e-1** | tet4+penalty | **4/16** | **0.750 mm** | +0.763 mm | +0.859 mm | 6.8 kPa |
   | **gui-dflt** | **1e-1** | **tet10+ipc** | **16/16** | **3.000 mm** | +0.358 mm | +0.535 mm | 36.3 kPa |
   | gui-dflt | 1e-2 | tet4+penalty | **0/16** | — | — | — | — |
   | gui-dflt | 1e-2 | **tet10+ipc** | 3/16 | 0.563 mm | +0.674 mm | +0.849 mm | 3.9 kPa |

   ⚠ **[SOCK-ERA HEADLINE — SUPERSEDED.** The scene called "the one the GUI
   actually runs" here is `sock_over_capsule`'s dual-layer stack, not the
   product's `base_mold`, and the κ behind it was ~2× too large. The current
   headline is in `THE HEAD-TO-HEAD, ON THE PRODUCT SCAN AT THE FIXED κ`.**]**
   As measured then: at the tolerance that already ships, the bridge took the
   seatable depth from a QUARTER of the slider range to ALL of it — 0.750 mm →
   3.000 mm, 4× — non-penetrating,
   with +0.535 mm of clearance across the 5 % area tail. σ rises 6.8 → 36.3 kPa
   because the wall is genuinely engaged rather than barely touching.

   **The bridge wins all four comparisons**, and each is a different kind of win:
   - `1layer` @ 1e-1 — **same depth, penetrating → seated.** A negative 5 % area
     tail means a *region* through the wall, not one bad vertex; +0.347 mm
     replaces −0.042 mm on 11 037 pairs against 3 181.
   - `1layer` @ 1e-2 — **1.125 → 2.625 mm (2.33×)**, both clean.
   - `gui-dflt` @ 1e-1 — **0.750 → 3.000 mm (4×)**, the product result.
   - `gui-dflt` @ 1e-2 — **nothing → 0.563 mm.** The baseline fails to converge
     step 0 at all ("Newton failed to converge within 150 iterations").

   ⛔⛔ **THE SYNTHETIC SCENES MISLED, AND THAT IS THE TRANSFERABLE PART.** The
   sphere said there was *no* depth win; the tolerance fixture said the win
   needed a tolerance tighter than ships. THIS scan said there was a
   4× win at the shipped tolerance — ⚠ and it is `sock_over_capsule`, so the
   lesson below transfers while the number does not. ⇒ this is #958's lesson arriving from
   the other side — there it was the analytical shell being too well-conditioned
   to see a failure; here it is two synthetic scenes being too well-conditioned
   to see a *benefit*. **Re-qualify a fixture for each question; a verdict read
   off the easy scenes is not a verdict.**

   ⚠ **What is NOT measured here**: the real scan was run at 1e-1 and 1e-2 only,
   so the bridge's tolerance cliff (0/16 at 1e-3 on the synthetic scenes) is
   **unprobed on the product geometry**. ⭐ What can be said is that on
   `gui-dflt` the BASELINE's cliff comes first — it is already at zero by 1e-2,
   a decade before the bridge is.

   ### ✅✅✅ THE HEAD-TO-HEAD, ON THE PRODUCT SCAN AT THE FIXED κ

   `base_mold`, 5 mm inset, 17 mm DRAGON_SKIN_10A @ 25 % Slacker. Both arms,
   both schedules, both tolerances — schedule controlled, because at 16 steps
   the bridge has not yet cleared its own contact stall.

   | steps | tol | arm | depth | `min_sd` | 5 % tail | seated? |
   |---|---|---|---|---|---|---|
   | 16 | 1e-1 | penalty | 4.062 mm | −0.186 | +0.200 | ⛔ |
   | 16 | 1e-1 | **bridge** | 3.438 mm | **+0.306** | **+0.502** | ✅ |
   | 16 | 1e-2 | penalty | 4.375 mm | −0.262 | +0.137 | ⛔ |
   | 16 | 1e-2 | **bridge** | 1.562 mm | **+0.454** | **+0.661** | ✅ |
   | **32** | **1e-1** | penalty | **5.000 mm** | **−0.444** | **−0.005** | ⛔⛔ |
   | **32** | **1e-1** | **bridge** | **4.531 mm** | **+0.216** | **+0.414** | ✅ |
   | 32 | 1e-2 | penalty | 1.875 mm | +0.395 | +0.607 | ✅ |
   | 32 | 1e-2 | **bridge** | **3.438 mm** | **+0.306** | **+0.502** | ✅ |

   ⇒ ⭐⭐⭐ **THE PENALTY PATH NEVER PRODUCES A DEEPER VALID SEAT THAN THE
   BRIDGE, IN ANY CELL.** Where it goes deeper it is through the wall; where it
   stays out of the wall it is shallower. The one cell where both are valid —
   32 steps at 1e-2 — the bridge is **1.83× deeper** (3.438 vs 1.875 mm).

   ⛔⛔ **And at 32 steps / shipped tol the penalty path's 5 mm is a REGION
   through the wall**, not an outlier: the 5 % area tail is **−0.005 mm**. A
   finer schedule let it reach 100 % by penetrating harder (`min_sd` −0.186 →
   −0.444 mm). ⇒ *depth reached* and *wall seated* are now visibly different
   quantities on the product geometry, and only the second is a fit.

   ⭐⭐ **The bridge's answer is PATH-INDEPENDENT; the baseline's is not.** The
   bridge reaches 3.4375 mm twice by different routes — 11/16 at tol 1e-1 and
   22/32 at tol 1e-2 — and reports `min_sd` 0.3062 vs 0.3063 mm and σ 39.33 vs
   39.33 kPa. Four significant figures, from a different schedule AND a
   different tolerance. That is what a converged contact solution looks like.

   ⚠ **Best operating point: the bridge at 32 steps, shipped tol — 4.531 mm
   (90.6 %) seated.** Its limit there is the element inversion, not contact.

   ⚠ **The bridge IS more tolerance-sensitive, and that is unfixed** — at 16
   steps it drops 11/16 → 5/16 going from 1e-1 to 1e-2 (a *convergence* stall
   at Newton iter 6, not a feasibility one) while the baseline improves. A
   finer schedule masks it (at 32 steps the bridge wins at both tolerances),
   but it was not addressed by the κ fix and should not be assumed gone.

   ### ✅ THE κ DERIVATION, FIXED — ship the ceiling, refine the schedule

   The floor was selecting κ **from the increment**, so a finer march lowered
   κ and the barrier held proportionally less: the feasibility threshold chased
   the step downward and could never be caught. Holding κ at the **ceiling**
   (the stated cushioning requirement, which does not depend on the increment)
   and refining the schedule underneath it:

   | steps | step | held | depth | was (centre κ) |
   |---|---|---|---|---|
   | 16 | 0.3125 mm | 0.3062 mm | **3.438 mm (68.8 %)** | 1.875 mm (37.5 %) |
   | 32 | 0.1562 mm | 0.2163 mm | **4.531 mm (90.6 %)** | 2.344 mm (46.9 %) |
   | 64 | 0.0781 mm | 0.2146 mm | **4.531 mm (90.6 %)** | 3.047 mm (60.9 %) |

   ⇒ ⭐⭐⭐ **The contact-feasibility wall is gone.** 32 and 64 steps stop at
   the SAME depth and on a DIFFERENT failure — `validity violation at tet 516:
   det F = -0.030`, an element turning inside out. That is a material and mesh
   limit, not a barrier one. **1.8–1.9× more depth at both schedules.**

   ⇒ `bridge_face_barrier_kappa` now returns the ceiling and no longer varies
   with the schedule. The floor is still computed and printed
   (`the_bridges_barrier_band_reports_a_floor_and_ships_a_ceiling`) but does
   not select. The schedule requirement becomes **`step < held standoff`**, and
   the held standoff must be MEASURED — it depends on wall compliance, which no
   closed form here knows.

   ⚠ **What still limits the ramp**: element inversion at 4.531 mm of a 5 mm
   inset, schedule-independent. ⛔ **F-bar is NOT a candidate**: it is hard-gated off for Tet10 by an assert
   in `newton.rs` — *"F-bar's single-Gauss-point volumetric constraint has no
   multi-Gauss-point Tet10 analog"*. Using it would mean deriving that analog
   first. The realistic candidate is mesh-side: a finer cell where the strain
   concentrates.

   ---

   ### ⛔⛔⛔ HOW THE FLOOR WAS FALSIFIED (kept — the method is the point)

   The floor is derived as *"κ such that the barrier holds `ρ · step` open at
   traction σ"*, and PR #954 pinned it as a derivation rather than a sweep.
   On `base_mold` it does not describe what happens.

   **The bridge stalls at 6/16 (1.875 mm of a 5 mm inset)** with an
   infeasible-start signature — Armijo at Newton iteration **0**, `r_norm`
   3.6e4. The last converged step held `min_sd` 0.2936 mm against a 0.3125 mm
   increment: the increment outran the standoff, which is precisely what the
   floor exists to prevent.

   Two candidate fixes were tested, and **both failed in ways that identify
   the real defect**:

   **1. March finer** — the derivation's own prescription, since the floor is a
   marching-scheme number.

   | steps | step | κ | held | held/step | depth |
   |---|---|---|---|---|---|
   | 16 | 0.3125 mm | 2.4137e7 | 0.2936 mm | **0.94** | 37.5 % |
   | 32 | 0.1562 mm | 1.5605e7 | 0.1347 mm | **0.86** | 46.9 % |
   | 64 | 0.0781 mm | 1.1015e7 | 0.0635 mm | **0.81** | 60.9 % |

   Depth improves, but the feasibility ratio **degrades**. Refining lowers the
   floor, which lowers κ, which holds proportionally less — the threshold
   shrinks with the step, so the march can never catch it.

   **2. Raise ρ** — on the theory that the gap-ratio substitution under-predicts
   the barrier-inverted ρ (the sweep's own output warns the two "coincide only
   where the traction-gap map is near-linear").

   | ρ | required | κ | held | depth |
   |---|---|---|---|---|
   | 1.12 | 0.350 mm | 2.4137e7 | **0.2936 mm** | 1.875 mm |
   | 1.30 | 0.406 mm | 2.7220e7 | **0.2947 mm** | 1.875 mm |
   | 1.50 | 0.469 mm | 3.1036e7 | **0.2944 mm** | 2.188 mm |
   | 1.75 | 0.547 mm | 3.6623e7 | **0.2766 mm** | 2.812 mm |

   ⇒ ⭐⭐⭐ **THE HELD STANDOFF IS FLAT AT ~0.294 mm ACROSS A 1.5× RANGE OF κ.**
   It does not respond to the barrier. **The floor's premise — that κ sets the
   standoff — is false in this regime.** The standoff is set by geometry and
   elastic equilibrium; κ only changes the traction required to reach it.

   ⭐ That is consistent with a number measured earlier and not connected until
   now: **σ moves only 1.0183× per decade of κ** on this scene. A gap that
   barely moves while the load adjusts is exactly a geometry-determined
   standoff. The two measurements agree, and the derivation contradicts both.

   ⚠ **Depth DOES improve with κ** (1.875 → 2.812 mm from 2.4e7 → 3.7e7), so a
   stiffer barrier helps — just not through the mechanism the floor claims.
   ⛔ **Do not "fix" this by sweeping κ for depth.** What is owed is a
   re-derivation against a correct model of what sets the standoff. The
   interval `[floor, ceiling]` currently selects a stiffness on a premise the
   scene does not honour, and a number that happens to work would be a sweep
   wearing a derivation's clothes.

   ⚠ **What still stands**: the ceiling (stay out of the cushion) is a stated
   requirement and is untouched by this; and on the sim-soft fixture the floor
   DID predict its stall (1e6 stalls above 17.567 kPa, measured 14.255). So
   this is a transfer failure to a compliant open-mouth wall, not proof the
   derivation was always wrong.

   ### ⛔⛔ σ RE-MEASURED ON THE PRODUCT SCAN — and it is HALF

   Every scan number above is `sock_over_capsule`. The product scene is
   **`base_mold`** as the CF Studio project configures it: a **5 mm** inset
   through **17 mm of DRAGON_SKIN_10A at 25 % Slacker**, against sock's 3 mm
   through Ecoflex 00-30. Re-measured 2026-09-22 by the same method as #959,
   at a common depth of 3.4375 mm:

   | κ | steps | σ (rest) | `min_sd` | 5 % tail | ρ | ρ tail | |
   |---|---|---|---|---|---|---|---|
   | 1e2 | 16/16 | 23.40 kPa | −1.920 mm | −1.415 mm | — | — | ⛔ through |
   | **1e3 (ships)** | 13/16 | 50.05 kPa | **−0.041 mm** | +0.342 mm | — | 1.641 | ⛔ through |
   | **1e4** | **16/16** | 57.83 kPa | +0.851 mm | +0.914 mm | 1.113 | 1.036 | seated |
   | **1e5** | 11/16 | **58.89 kPa** | +0.985 mm | +0.991 mm | 1.010 | 1.004 | seated |
   | 1e6 | 0/16 | — | — | — | — | — | stalled |

   ⇒ **σ ≈ 58.9 kPa, not 117 kPa**, and **ρ ∈ [1.00, 1.11]**, not [1.00, 1.18].
   `κ` scales linearly with σ, so every derived stiffness taken before this was
   about **2× too large**. At `d̂` = 1.2 mm on the product schedule the bracket
   is now `[1.4162e7, 4.1138e7]` with a derived **2.4137e7**, against the
   3.5377e7 that had been shipping.

   ⭐ **It is close to converged, not a loose bound.** σ moves **1.0183× per
   decade** across the seated arms — the flattest coupling of any scene tried
   (sphere 1.047×, sock scan 1.048×, sim-soft fixture 1.238×).

   ⛔ **The shipped κ = 1e3 is through the wall here too** (`min_sd`
   −0.041 mm), which is now three scenes out of three.

   ⛔⛔ **And the area basis matters MORE on this scene**: σ(rest) 58.89 kPa
   against σ(deformed) **41.52 kPa** — a **41.8 %** gap, nearly double sock's
   22–23 %. Reading σ on the deformed basis would be a much worse error here.

   ⚠ **The stiff end arrives a decade earlier than on sock**: `κ = 1e5` already
   degrades to 11/16 and `1e6` converges nothing, where the sock scenes solved
   at 1e5 and stalled at 1e6. And `κ = 1e4` is the only arm that reaches FULL
   depth — the shipped 1e3 stalls at 13/16.

   ### ⭐⭐ WHAT A STEP COSTS — and it inverts too  ⚠ SOCK-ERA NUMBERS

   > ⚠ **Measured on `sock_over_capsule` at the pre-fix κ.** The "real scan" row
   > below is sock's `gui-dflt` (72 935 tets); the product scan `base_mold` is
   > 65 293. `what_a_bridge_step_costs` now points at `base_mold`, and these
   > timings have NOT been re-taken there. The MECHANISM — per-step cost tracks
   > how hard the solve is, not element order — is what to carry; the seconds
   > are not current.


   `RampStep::wall_time_s`, measured around `replay_step` alone. Idle machine,
   macOS/ARM, shipped `tol`. ⛔ A diagnostic; wall clock is contended and this
   never gates.

   | scene | tets | DOF (Tet4 → Tet10) | penalty s/step | **bridge s/step** |
   |---|---|---|---|---|
   | tol-fixture | 9 258 | 31 143 → 69 954 (×2.25) | 0.03 | **0.78** |
   | sphere-40mm | 45 654 | 124 086 → 309 024 (×2.49) | 0.21 | **5.92** |
   | **gui-dflt (real scan)** | **72 935** | **189 456 → 482 124 (×2.54)** | **48.87** | **7.82** |

   (medians; geometry build is 0.02 / 0.08 / 0.69 s and never dominates)

   ⭐⭐⭐ **On the product mesh the bridge is ~6× FASTER PER STEP than the
   penalty path** — 7.82 s against 48.87 s — while carrying 2.54× the DOF. On
   both synthetic scenes it is **25–28× SLOWER**. ⇒ **per-step cost is not a
   property of element order; it is a property of how hard the solve is.** The
   penalty path's per-step time explodes on the real scan because it is
   thrashing — the Armijo stall at Newton iteration 136 and the repeated
   `Llt non-PD` fallbacks are the same phenomenon seen from the clock.

   ⇒ practically: on the GUI's own scene the bridge delivers **16/16 steps in
   146 s of solve** where the penalty path spends **124 s to deliver 4**.
   Roughly the same wall clock, four times the depth.

   ⚠ **The bridge's total excludes its approach**, which is solved and not
   recorded. At `d̂` = 1.2 mm and a 0.1875 mm increment that is **at least
   `ceil(d̂/step)` = 7 extra solves**, so ≥ ~55 s more on the real scan. Its
   honest end-to-end figure is ~200 s, not 146 s.

   ⚠ **VERDICT AS OF THAT PASS — since superseded twice** (σ re-measured, κ
   re-derived); the standing verdict is in `THE HEAD-TO-HEAD, ON THE PRODUCT
   SCAN AT THE FIXED κ`. It read: the solver-side case for the bridge is made on
   the geometry that matters. ⛔ The remaining blocker to making it the default is
   **not** the solver — it is that `compute_tet_readouts` is corner-linear, so
   the UI's per-tet heat map would report Tet4-quality stress off a Tet10 solve
   (the corner-readout note, now `✅ PER-GAUSS-POINT READOUTS` below). That is
   a readout fix, and it is the next piece of work, not a reason to doubt the
   result. ✅ **That fix has landed — and it found a second blocker this verdict
   did not know about**, in the same section.

   ⛔⛔ **`κ` AT TWO TOLERANCES — at `1e-6` no `κ` converges; at the shipped
   `1e-1`, `κ` decides the seat.** `the_bridge_ramp_over_a_stiffness_sweep` on
   `tolerance_fixture` (3 mm inset, 0.1875 mm schedule): a log grid over
   1e6–1e9, five per decade, plus the bracket floor (7.26e6) and the shipped
   value (4.11e7, the ceiling). Measured 2026-09-23, macOS arm64, release.

   Asked for `1e-6`, **every arm reports 0/16**, and every stall is inside the
   feasible-start approach, which marches up toward 0 in 0.1875 mm steps — so
   `-0.1875 mm` is its last solve:

   ```text
   κ              approach stalls at
   1.0e6–1.6e6    -0.1875 mm
   2.5e6–2.5e7    -1.3125 mm    (FLOOR among them)
   4.0e7–6.3e7    -0.9375 mm    (SHIPPED among them)
   1.0e8          -0.7500 mm
   1.6e8–1.0e9    -1.3125 mm
   ```

   Asked for the shipped `1e-1`:

   ```text
   κ              steps     depth           min_sd, 5 % tail
   1.0e6–2.5e6     0/16     —               fail the first recorded step
   4.0e6–1.6e7   2–13/16    0.38–2.44 mm    both > 0, then stall
   2.5e7–2.5e8    16/16     3.000 mm        +0.136–0.535, +0.178–0.607 mm
   4.0e8–1.0e9   14–15/16   2.63–2.81 mm    stall on ELEMENT INVERSION (det F < 0)
   ```

   ⇒ At `1e-6` no `κ` in three decades removes the conditioning floor. At
   `1e-1` a full, non-penetrating seat needs `κ` inside one decade on this
   grid, **2.5e7–2.5e8**, and the shipped value is in it — 0.21 decades above
   the lowest grid point that seats, 0.79 below the highest. **Why the
   approach's reach is non-monotone in `κ`, and why the stiff end fails by
   inversion, has not been isolated.**

   ⚠ **Until 2026-09-23 this block headlined "`κ` IS NOT THE BINDING
   CONSTRAINT" with no tolerance**, and said *"all five stall the same way, at
   Newton iteration 4–5, at residuals between 1e-4 and 7e-3"*. That sentence was
   written during #960 and does not reproduce at its merge — including at the
   1e6 and 1e9 arms, whose κ never changed. What moved it was not isolated. The
   headline holds at `1e-6` only.

   ⛔⛔ **RETRACTED — the derivation below is the one that was replaced.** It
   selected the geometric centre of `[floor, ceiling]` and made κ depend on the
   increment. Measured on the product scan, that is exactly what made refining
   the march self-defeating; κ is now the **ceiling** and does not move with the
   schedule (see `THE κ DERIVATION, FIXED`). Kept for the record — the numbers
   below are at #959's σ = 117 kPa and ship nowhere.

   ⚠ Its stated rationale — the floor belongs to the MARCHING SCHEME, so a
   stored constant goes wrong when `n_steps` changes — was *sound reasoning from
   a false premise*: the floor only belongs to the marching scheme if κ sets the
   standoff, and on a stiff wall it does not. At `d̂` = 1.2 mm over a 16-step
   3 mm ramp it gave:

   ```text
   floor    σ / |b'(ρ · step)| = 1.5315e7   hold one increment open
   ceiling  σ / |b'(d̂ / 2)|    = 8.1717e7   stay out of the cushion
   derived  geometric centre   = 3.5377e7   0.727 decades wide
   ```

   ⚠ No round decade sits inside — which is why THAT revision shipped a
   log-midpoint. `the_bridges_barrier_band_reports_a_floor_and_ships_a_ceiling`
   re-derives the bands on every build, and shows
   the boundary where the derivation must refuse: at a 4-step schedule
   `ρ·step` = 0.885 mm and three of the four bands have no bracket at all.

   ▶▶ **THREE THINGS THE SWAP BROKE THAT A COMPILE COULD NOT SEE.** All three
   compiled, none panicked, and each would have shipped a wrong answer quietly:

   1. ⛔⛔ **IPC is an INTERIOR-POINT method and the ramp had no feasible
      start.** At interference 0 the cavity surface and the intruder coincide,
      so the first increment hands Newton a state already through the wall:
      residual at iteration 0 was **4.08e6 N**. The penalty path tolerates that
      start; a barrier cannot. The ramp now marches in from a clearance in
      increments of the same size, solving but not recording the approach —
      which is exactly what the `κ` floor is *for*, since "hold `ρ·step` open"
      is the condition that makes every later start feasible. Residual at the
      first increment fell to **1.21e-3**.
   2. ⛔ **The clearance must be measured over the REFERENCED vertices, not
      over `positions()`.** `positions()` is the lattice, not the body: the
      worst rest "penetration" over all vertices is **11.5427 mm** — an orphan
      BCC lattice point near the cavity centre — against **0.2522 mm** over the
      vertices the solver sees. Taking the former inflated the approach from 5
      increments to 65, and the ramp never reached its recorded steps.
   3. ⚠ **Enrichment puts boundary midsides UNDER the curved cavity surface.**
      `from_tet4` places every midside at the straight-edge midpoint, so a
      boundary midside sits under the true surface by the sagitta. Measured on
      the tolerance fixture: tightest corner −0.1354 mm, tightest midside
      −0.2522 mm, an excess of **0.117 mm** against a predicted `h²/8R` of
      **0.118 mm** at `h` = 4 mm, `R` = 17 mm. `Tet10Mesh::with_curved_midsides`
      is the cure and is **not** applied here — the approach clearance absorbs
      it instead, so the reported `min_sd` carries that bias and is
      conservative by roughly one sagitta.

   ⛔⛔ **THE READOUT'S PAIR KIND IS NOT THE SOLVER'S, and a gate written the
   obvious way fails on a correct bridge.** `IpcRigidContact::active_pairs`
   emits `ContactPair::Face` — that is what the solver scatters. But
   `per_pair_readout` on the same mesh emits per-NODE `ContactPair::Vertex`
   naming the six face nodes, because the face-integrated barrier loads the
   MIDSIDES and puts ~0 on the corners. Measured: **1442 of 1442 readouts were
   `Vertex`** on a mesh whose solver contact was entirely face-integrated. ⇒
   the selector probe must read `active_pairs`; and
   `filter_pair_readouts_to_referenced`'s `unreachable!()` on `Face` is
   therefore never reached from this path, so the shipped filter is kept.

   ⚠ **But it survives only because `referenced_vertex_mask` walks
   `tet_midside_nodes`.** Every loaded node on the face path IS a midside, so a
   corners-only "referenced" set would delete the entire contact patch and
   return a clean, empty, non-panicking readout — conformity 0 on a good
   design. `the_bridges_midside_readouts_survive_the_orphan_filter` runs that
   counterfactual rather than trusting the property.

   ### ✅ PER-GAUSS-POINT READOUTS — the bridge's heat map, and a second silent defect

   ✅ **The per-tet readouts are read at the Gauss points.** They were
   corner-linear: `F` from four corner displacements, which on a quadratic
   element is the linear part of a field that is no longer linear. Now
   `ReadoutMesh` snapshots the mesh the ramp SOLVED — corners and midsides —
   and reads `F` at each of Tet10's four Gauss points through the isoparametric
   Jacobians, and `TetReadout` reduces those points to the element-mean energy,
   the peak stress and the stretch range. Every ramp returns its
   `ReadoutMesh`; the UI re-derives per-step readouts through it rather than
   through its own Tet4 snapshot. Gated by
   `the_readout_resolves_the_tet10_strain_at_every_gauss_point` (`F` against
   the analytic gradient of a quadratic field) and
   `the_tet10_readout_mesh_places_every_elements_midsides` (midside order, on
   every element of the enriched tolerance fixture); each fails under an
   injected bug. Tet4 readouts are bit-identical to the corner construction —
   all 16 step aggregates and every field of all 9 258 tets on
   `tolerance_fixture`.

   ⭐ **What the corner readout was getting wrong** —
   `what_the_corner_readout_missed_on_the_bridge` reads both off the SAME
   converged state, the last step each ramp reaches, so only the readout
   differs. macOS arm64, release, 2026-09-23:

   | scene | reached | peak ‖P‖ | max stretch | min stretch | mean Ψ | hotspot element | elements off by > 10 % in ‖P‖ | median per-element Δ‖P‖ |
   |---|---|---|---|---|---|---|---|---|
   | `tolerance_fixture` | 16/16, 3.000 mm | 564 → 313 kPa | 1.366 → 1.474 | 0.260 → 0.319 | 16.5 → 15.2 kJ/m³ | #1118 → #3617 | 84.7 % | 33 % |
   | `base_mold` (product) | 29/32, 4.531 mm | 912 → 245 kPa | 1.841 → 1.947 | 0.160 → 0.237 | 5.34 → 5.09 kJ/m³ | #3652 → #60516 | 66.4 % | 17 % |

   (each cell is corner → per-GP.) The product ramp reproduced its recorded
   operating point — 29/32, 4.531 mm, stopping on the same inversion at tet 516
   (`det F = -0.030`) — as it must: the readout runs after the solve.

   On both scenes the corner readout reads peak stress and mean energy HIGH
   and both stretch extremes LOW. Why it errs in those directions has not been
   isolated — the table is what to carry.

   ⛔ **A second silent bridge defect, same root cause.** The UI found the
   outer skin by comparing `x_final` against its own Tet4 snapshot. On the
   bridge `x_final` carries the midsides, the lengths differ, and the function
   returned an EMPTY set rather than an error — **0 outer-skin vertices**,
   against 15 481 from the solved mesh on `tolerance_fixture` and 53 789 on
   `base_mold`. Nothing read as outer skin, so the outer layer's deformed shell
   and the cavity-face filter both degraded without a word. It now takes the
   solved mesh's rest positions and refuses a mismatch
   (`detect_outer_skin_vertices_refuses_a_mismatched_mesh`).
   ⇒ **the readouts were not the only blocker to defaulting the bridge**, as
   the verdict above had it — they were the one that had been found.

   ⚠ **The search for more of this class**, and its boundary: every UI site
   that pairs data taken from the Tet4 snapshot with a ramp's state. Faces
   (cavity boundary, per-layer outer, slab) index `x_final` by corner id, which
   enrichment preserves — sound, though they draw the corner triangles and not
   the midside curvature. The heat map's centroid lookup and layer map index
   the readouts by Tet4 tet id — sound only because enrichment keeps element
   order, which `the_tet10_readout_mesh_places_every_elements_midsides` now
   pins. The readouts and the outer skin were the two that were not sound.
   `main.rs` reads no ramp state at all (`x_final`, `final_x`,
   `final_per_tet`, `tet_vertices`, `readout`: zero code hits). Not searched:
   anything outside `tools/cf-sim-research`.

4. **Per-Gauss-point material sampling** (§7.6). The expensive one: a
   return-shape change to `Mesh::materials()` reaching 119 call sites.
   Deferred to here so its benefit arrives as a number rather than a belief.

   ⚠ **Scope it before writing it — per-GP is the right cure for a *smooth*
   field and the wrong one for a *sharp* interface.** An element carrying four
   different materials has a discontinuous constitutive response inside one
   continuous shape-function basis. For a continuous gradient (filler fraction
   φ, a thermal field, a blended zone) that is correct and is the whole point.
   For a layer boundary it is not: it smears the interface at 4-point
   resolution where the honest fix is mesh conformity, so the element does not
   straddle at all. The repo already distinguishes the two —
   [`Mesh::interface_flags`] implements the `|φ(x_c)| < L_e` straddle test and
   is populated at construction — so this item should ship **with** a decision
   about flagged tets, not merely "sample four times everywhere."

   ⛔⛔ **MEASURED during item 3b, and it conditions the sentence above: at
   `CELL` = 4 mm the straddle flag is SATURATED and cannot isolate a layer
   boundary.** `L_e` is the tet's six-edge mean, **measured 3.495 mm** here, so
   the straddle band `2·L_e` = **6.990 mm is 1.75× the 4 mm layer** it
   delimits. (The gate measures `L_e` itself; an earlier revision quoted
   "≈3.4 mm" from a sizing probe that had since been deleted, and it was 2.7 %
   off as well as unsourced.) On the
   graded shell (8 736 tets) it flags **3 504 (40.1 %)** at the r = 14 mm
   boundary and **5 484 (62.8 %)** at r = 18 mm; in its sharpest form,
   **boundary 0 flags more tets (3 504) than the entire layer it bounds
   contains (2 304)**. ⇒ "a decision about flagged tets" would at this
   resolution be a decision about most of the body, which is not the seam
   treatment the sentence intends. Item 4 needs either a finer cell or a
   criterion that is not `L_e`-wide — and this is a *lower* bound on the
   problem, since these boundaries are exactly concentric spheres and a real
   scan's irregular offsets can only flag more.
   Pinned by `the_interface_flag_cannot_isolate_a_layer_boundary_at_this_cell_size`.

5. **Face-friction reconciliation**, or an explicit frictionless declaration
   recorded as a known limitation (§5).

## 9. Corrections made during this recon

Recorded because they are evidence about the method, which carries forward.

- **A compile probe returned green for the wrong reason.** `Tet10 × IPC ×
  Yeoh` was first probed against `SdfMeshedTetMesh<Yeoh>`, which compiles — but
  that mesh returns `None` from `boundary_faces6()`, so it silently selects the
  **vertex** path. The green proved the opposite of what it appeared to. The
  real target (`Tet10Mesh`) does not compile. ⇒ When probing a capability
  selected by a runtime `Option`, the probe must pin the *selector*, not only
  the types.
- **"Tet10 cures volumetric locking" was wrong** (§4.2) — it mitigates,
  does not cure, and is adopted for rim accuracy instead.
- **"Adopting IPC gets you friction" was wrong on the face path** (§5).
- **§4.2's A-vs-B fork did not exist.** It was assembled from the book's
  general preference for mixed u-p without checking the ladder's rung-6
  verdict, which had already measured pure-displacement Tet10 as adequate at
  ν = 0.49 and recorded Taylor-Hood P2-P1 as deliberately not built. ⇒ **When a
  document and a codebase disagree about a design choice, the codebase may have
  measured something the document only argued.** Check the ladder's verdicts
  before reasoning from the study's chapters.
