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
   here) — a closing cavity has no reason to share that number. And the
   interval is `|b'(ρ·step)| / |b'(d̂/2)|` = **9.5×** wide regardless of `σ`, so
   it selects a decade only while the design traction is known to within
   roughly `[0.47×, 4.5×]`; `insertion_sim`'s traction is its own measurement,
   not this fixture's **30.4 kPa**.

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
