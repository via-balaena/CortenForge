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

   ✅✅ **`σ` AND `ρ` ARE NOW MEASURED FOR THIS SCENE (2026-09-22 at
   `89169377`) — and the first thing they say is that the penalty path cannot
   supply them.** Three `#[ignore]`d probes in
   `tools/cf-sim-research/src/insertion_sim.rs` read the area-weighted mean
   traction `Σ‖f‖ / Σa` and the gap distribution off every converged step of a
   16-step ramp:
   `the_bridges_design_traction_and_patch_nonuniformity_on_the_synthetic_sphere`,
   `…_on_the_real_scan` (two scenes), and
   `the_design_traction_is_measured_against_the_stiffness_that_produced_it`.

   ⛔⛔ **`σ` is NOT independent of the contact stiffness here, and the entire
   derivation rests on it being so.** `tet10_yeoh_ipc_convergence` states the
   condition in as many words — *"`κ = σ/|b'(d)|` is only a derivation if `σ` …
   is a property of the material and the compression, not of the barrier
   stiffness being solved for"* — and measures **1.003×** for it. Swept through
   `insertion_sim`'s penalty path on the synthetic sphere, at the full 3 mm
   inset:

   | penalty `κ` | steps | `σ` (kPa) | `min_sd` (mm) | `ρ` | `Σ‖f‖/‖Σf‖` |
   |---|---|---|---|---|---|
   | 1e2 | 16/16 | 25.22 | **−1.755** | 0.788 | 946 |
   | 1e3 (shipped) | 16/16 | 68.34 | 0.0544 | 4.923 | 2123 |
   | 1e4 | 16/16 | 91.13 | 0.8517 | 1.054 | 1106 |
   | 1e5 | 16/16 | 94.98 | 0.9841 | 1.005 | 958 |
   | 1e6 | **0/16** | — | — | — | — |

   ⇒ **`σ` spans 3.77×** where the fixture's spans 1.003×. The independence is a
   property of a **stiff** contact — the gap adjusts and the load does not — and
   at the shipped `κ = 1e3` the area-weighted mean gap runs from a sixth to three
   quarters of the way into a 1 mm band as the ramp deepens, so
   the equilibrium moves with the stiffness. **A `σ` read at the shipped
   stiffness is not the design traction.**

   ⚠ **And the stiff limit is not reachable on this path.** `κ = 1e6` stalls at
   **step 0** (Armijo, Newton iter 61, `r_norm` 1.96e1) — 1e5 is the stiffest
   that solves, and `σ` is still moving 4 % between 1e4 and 1e5. What the sweep
   supplies is a **lower bound approaching ~95 kPa**, not a converged rigid-body
   traction. ⛔ At the other end `κ = 1e2` has the *whole patch* through the
   intruder (`min_sd` −1.755 mm); its 25.22 kPa is not a traction on anything.

   ✅✅ **The same sweep on the REAL SCAN agrees, and it also says what the
   penetration above actually is.** Single-layer scene, 68 087 tets, full 3 mm:

   | penalty `κ` | steps | `σ` (kPa) | `min_sd` (mm) | 5 % tail (mm) | `ρ(min)` | `ρ(tail)` |
   |---|---|---|---|---|---|---|
   | 1e2 | 16/16 | 26.23 | −2.059 | −1.733 | — | — |
   | 1e3 (shipped) | 16/16 | 70.37 | **−0.373** | **−0.042** | — | — |
   | **1e4** | 16/16 | 94.03 | **+0.760** | +0.834 | **1.177** | **1.073** |
   | 1e5 | 16/16 | 98.08 | +0.972 | +0.982 | 1.017 | 1.007 |
   | 1e6 | **0/16** | — | — | — | — | — |

   ⭐⭐ **`κ` = 1e4 reaches the full inset WITHOUT penetrating.** The −0.373 mm
   at the shipped `κ` is not the geometry refusing to seat — it is the penalty
   being **an order of magnitude too soft to hold the wall out**, and one decade
   of stiffening removes it entirely while reaching the same 16/16.

   ⭐ **The two scenes agree to within their own noise**, which is the evidence
   that this is a property of the path rather than of either fixture: `σ` spans
   **3.74×** (scan) against **3.77×** (sphere); the last two arms sit **1.043×**
   apart (scan) against **1.042×** (sphere); `κ` = 1e6 stalls on both. ⇒ **for
   the bridge, `σ` ≈ 95 kPa as a lower bound and `ρ` ∈ [1.01, 1.18]**, read at
   `κ` = 1e4–1e5 on a non-penetrating full-depth seat — not the 6.78 kPa / 1.22
   the shipped stiffness reports off a 25 % seat.

   ⭐⭐⭐ **THE DERIVATION AT THOSE NUMBERS** — evaluated at the stiffest arm
   that solved on each scene, `ramp step` = 0.1875 mm:

   | scene | `σ` | `ρ(tail)` | `d̂` = 1.0 mm | `d̂` = 1.2 mm | `d̂` = 2.0 mm |
   |---|---|---|---|---|---|
   | sphere @ 1e5 | 94.98 kPa | 1.004 | [1.53e7, 7.96e7] | [1.03e7, 6.63e7] | [3.65e6, 3.98e7] ✅ |
   | **1-layer scan @ 1e5** | 98.08 kPa | 1.007 | [1.58e7, 8.22e7] | **[1.07e7, 6.85e7]** | [3.78e6, 4.11e7] ✅ |

   ⇒ **`κ` for the bridge is ≈ 1.1e7–1.6e7 at `d̂` = 1.0–1.2 mm**, or `1e7` fits
   at `d̂` = 2 mm (✅ marks the bands whose interval contains `1e7`).

   ⭐ **The fixture's `1e7` misses the scan's floor at the fixture's own band by
   8 %** — 1e7 against 1.07e7 at `d̂` = 1.2 mm. That is far closer than `σ`
   differing **3.2×** (98.08 against 30.4 kPa) would suggest, because the two
   errors partly cancel: this scene's required standoff is 0.191 mm against the
   fixture's 0.13 mm, and a wider standoff needs *less* `κ`. ⛔ **Do not read
   that near-miss as the constant transferring.** It transfers by coincidence at
   one band, and the sign of the miss matters: `1e7` is **below** the floor, so
   it is the value that does not hold one ramp increment open, which is the
   failure the floor exists to exclude.

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
   nearly agree. `σ` = **6.78 kPa**, `ρ(min)` = **1.223**, `ρ(tail)` =
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
   **1.086–1.223**, inside the same band the idealised cell reported
   (`[1.176, 1.221]`) and under the `1.30` the floor is derived with. **Shape
   irregularity costs essentially nothing here** — what does cost is the contact
   stiffness the number is read at, which was not on the list.
   ⚠⚠ **But do not carry these two numbers to the bridge.** They are read at the
   shipped stiffness — the reading the sweep above disqualifies — off a seat that
   reached 25 % of its inset. The ones to carry are the stiff-arm numbers: `σ`
   ≈ 95 kPa, `ρ` ∈ [1.01, 1.18]. What this table shows is the *shape* of the
   derivation on a real scan — a narrow interval that closes only at a small `d̂`.

   ✅ **The enveloping-patch cancellation, quantified inside this pipeline.**
   `Σ‖f‖/‖Σf‖` measures **1588–3095×** on the synthetic sphere, **214–455×** on
   the single-layer scan and **9.2→7.9×** on the GUI default, against the
   idealised cell's 900–6900×. ⚠ Only the last is monotone in depth; the other
   two are quoted as ranges because they are not. The effect's SIZE is a property of how enclosing
   the patch is, and the product scan is the least enclosing of the three. The
   F-d curve remains a correct *net seating resistance*; what it cannot be is
   divided by an area.

   ⚠ **What the marching schedule can and cannot fix.** The floor is the only
   bound that moves with the increment, so a finer march widens the interval —
   on the GUI default at `d̂` = 1 mm, 16 steps gives 0.61 decades and 512 gives
   2.09. It never brings `1e7` inside at that band, because the *ceiling* is
   fixed by `σ` and `d̂`. ⇒ **the band is the lever on the ceiling; the schedule
   is the lever on the floor.**

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
