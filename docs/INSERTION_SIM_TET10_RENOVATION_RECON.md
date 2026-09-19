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

### 4.2 The fork — the two cures fix different failure modes

`docs/studies/soft_body_architecture/src/20-materials/05-incompressibility/03-higher-order.md`
states that Tet10 is **not** the locking cure:

> *"A pure-displacement Tet10 element at ν = 0.499 still locks, just less
> severely than Tet4."*
> *"Tet10 lands at Phase H **not because of its locking behavior** but because
> of its accuracy improvement on the rim-deformation failure mode."*
> *"Tet4 + mixed u-p is cheaper than Tet10 for comparable accuracy."*

| path | fixes | cost | implemented? |
|---|---|---|---|
| **A. Tet10 + Yeoh** | rim polygonalization, through-thickness bending | ~2.5× stiffness nonzeros; Cholesky super-linear in nonzeros | element ✅, `Mesh<Yeoh>` ❌ |
| **B. Tet4 + mixed u-p** | volumetric locking at ν → 0.5 | book says cheaper | ❌ **`MixedUP` does not exist in `sim-soft`** (grep: 0 hits) |

**Path A is recommended, and the reason is geometric, not general.** The
rim-deformation chapter
(`10-physical/02-what-goes-wrong/04-rim.md`) describes this product's exact
geometry — *"the cavity's open rim... under probe insertion the rim flares
outward, the interior inner surface conforms to the probe"* — and names a
polygonal rim as corrupting **all four** reward terms: peak pressure fires on
mesh artefacts, uniformity reports a rim ring of spikes and troughs, coverage
biases at contact transitions, and design sensitivity goes noisy *within a
single mesh*. Path B addresses none of that.

⚠ The book's prescribed rim fix is **two** commitments — adaptive
h-refinement **and** Tet10 — and states *"either alone leaves a residual."*
**Adaptive h-refinement is also not implemented** (`mesh/mod.rs:146` names it
as Phase H future work; the other grep hits are a uniform block subdivider and
interval-arithmetic validity certification, neither of which is mesh
refinement). Path A therefore buys the element half of a two-part fix.

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

## 8. Work items, in dependency order

1. **`impl Mesh<M> for Tet10Mesh`** — generalise off the NeoHookean default.
   Gates everything else. Nothing downstream can proceed without it.
2. **Per-Gauss-point material sampling at Tet10** — per §7.6.
3. **SdfMeshedTetMesh → Tet10Mesh bridge inside `insertion_sim`**, plus fresh
   `d̂` / `κ`: IPC barrier parameters do not carry over from penalty's
   smoothing and normal-averaging tuning.
4. **Face-friction reconciliation**, or an explicit frictionless declaration
   recorded as a known limitation (§5).
5. **Wire `RewardBreakdown`** — §1's four terms become computable on a pressure
   field that means something. Until then no solver change is *measurable*:
   example row 25 (`...-open-mouth`), the geometry closest to the product, is a
   **demo with no physics oracle** precisely because the interference-fit load
   case inverts the force sign and strain ordering its sibling rows gate on.

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
