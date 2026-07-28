# FSU Tet4 → Tet10 coupling-migration plan

Make the whole Tet10 element ladder (`docs/SIM_SOFT_TET10_PLAN.md`, rungs 1–8 + curved
element + SDF-projection mesher, all merged #680–#700) **load-bearing in the flagship
FSU segment**: a curved-isoparametric Tet10 intervertebral disc, bonded to the *real*
curved vertebral endplate.

This is the payoff of the element ladder and the endgame of *exact-geometry-IS-exact-
physics* for the FSU: today the disc physics runs on flat-faced constant-strain Tet4,
which **bending-locks** (over-stiff in flexion); this arc puts a genuinely-curved
higher-order element on the genuinely-curved bone.

> **★ Justification (settled with the head engineer, §0.3):** the win is **disc-physics
> correctness + exact geometry**, NOT a segment-ROM improvement. The Tet10 disc is ~33%
> softer (more correct) in its own bending stiffness, but the disc is only ~0.4% of the
> segment's restoring moment at ROM, so the *segment* moment–rotation barely moves. We
> proceed on correctness/principle grounds (and because a Tet10 disc is load-bearing for
> future co-design gradients through the disc), gating on the **standalone k_disc FOM**
> plus a **direct exact-geometry residual gate** (§4.3 — new in v2; v1 had no gate on the
> arc's own stated payoff).

> **Checkpoint:** written against `main` @ `59e61daa`, hardened by two adversarial review
> rounds: a 5-front diamond-review (v1, §8.1) and a 4-front stress-test that reshaped the
> ladder (v2, §8.2). **Rungs 0 (#704), 1 (#705) and 2 are built — see each rung's BUILT note
> in §3, which records what the build changed about the plan. Next action: rung 3.**

> **★★ v2 CHANGED THE LADDER, not just the wording.** The stress-test found that v1's
> rung 1 gate could not fail, v1's ROM sanity assert could not trip *and* would fail on
> day one, v1's rung-0 golden could never run in CI, v1's §2.2 recipe would panic, and
> v1's scope claim ("no downstream break, the element param is additive") was false. The
> rung order and numbering are **different from v1** — see §3.

---

## 0. Recon results — the two gating questions, plus the reframe the review forced

Recon: a code read (Q1) + a measurement spike (Q0, built/run/reverted — twice, the second
time correcting a bond confound the diamond-review caught). All numbers below are the record.

### 0.1 Q0 — is the element-order payoff real? **YES, ~33% (corrected from a first-pass 43%).**

A spike meshed the real disc once and ran an *identical* node-based bonded flexion probe
(pin both endplate bands, rotate the superior band by θ = ±0.5°, read the reaction moment)
through arms differing only in the element. **Three arms, one shared mesh (7849 nodes /
2257 referenced corners), `DiscParams::default`, all fully converged to the solver tol (1e-10):**

| arm | element + bond | mean k_disc (N·m/rad) | ratio vs Tet4 |
|---|---|---|---|
| Tet4 (shipped) | full face tie | **−0.2800** | 1.000 |
| Tet10 corners-only | **under-tied (artifact)** | −0.1596 | 0.570 |
| Tet10 full face tie | face midsides pinned | **−0.1861** | **0.665** |

- **Tet4 reproduced the shipped ≈ −0.28** (bit-identical to `build_bonded_disc(..None)`;
  the disconnected-rim `largest_component` mesh) — validate-harness-first passed, so the
  shared bond logic is faithful.
- **The honest element-order payoff is ratio 0.665 → ~33.5% softening** (Tet10 full face
  tie). The first-pass 0.570/43% was **inflated by a bond artifact**: the spike's band
  filter (`referenced_vertices`, corners-only) left the bonded-face *midsides free*, so the
  Tet10 disc was corner-spot-welded, not face-tied — a looser bond that softens k_disc on
  top of the element effect. The migration builds the *full face tie* (§2.2), so **0.665 is
  the number; do not quote 0.570 as the element effect.**
- Converged genuinely (full-face flex residual 2.8e-13 @ 10 Newton iters; ext 4.3e-11;
  flex/ext symmetry 1.013 = a real restoring equilibrium). The indefinite-tangent LU
  fallback fires every iteration (benign, [[simsoft-nonpd-lu-fallback]]) but Newton reaches
  tol; pinning the face midsides *improved* conditioning vs corners-only.

**★ 0.665 is NOT re-runnable today** — the spike was reverted, so by this repo's own rule
(*committed re-runnable measurement or it's unverified*) it is **prior context, not a
verified anchor**. Rung 1 exists specifically to re-earn it as committed code (§3).

**Attribution + honesty bounds (three, all load-bearing — do not drop in any summary):**
1. **Element order, not geometry/mesh.** Arm C's corners are byte-identical to Tet4's; only
   the quadratic enrichment differs. So the ~33% is pure element order at fixed geometry.
2. **"Bending-locking" is the mechanism, but the cantilever number is an UPPER bound.** The
   committed `tet10_bending_locking.rs` (Tet4 recovers ~46% of analytic tip deflection @
   ν = 0.30, Tet10 ~95%) is a *slender free cantilever*; the FSU disc is *stubby, doubly-
   bonded, ν = 0.4*, a regime where constant-strain Tet4 locks **less**. The 33% is the
   directly-measured disc value; the cantilever explains its direction, it does not bound it.
3. **"Correct" is not established, only the delta.** −0.1861 is not proven to be the
   converged truth (a refined Tet4 would also soften). The settled claim: **Tet10 relaxes
   this bending mode ~1/3 vs Tet4, and Tet10 is the higher-order/more-accurate element.**
   ⇒ **v2 consequence:** an *exact* ratio anchor cannot be gated on an admittedly-unconverged
   mesh. Rung 1 gates a pre-registered **bracket** + commits the measured value; the
   h-refinement that would earn an accuracy claim is rung 5 (§3), not a deferral.

**Spike operational facts (carry into the rungs):**
- **★ Band on the SURFACE-AABB z-range, not the tet-mesh z-range.** `build_bonded_disc`
  bands off the scaled input-surface AABB (`sim/L1/fsu-model/src/lib.rs:301-308`); banding
  off tet-mesh positions gives wrong, ~2.4× softer thresholds.
- Mesh: Tet4 7849 nodes (2257 referenced corners; the rest padded-lattice orphans, retained
  by `largest_component`) → enrich preserves all corners + adds edge midsides → Tet10 19449
  (7849 corners incl. orphans + 11600 midsides; still 2257 *referenced* corners). **Full-face
  bonded bands: inferior 228→1005, superior 367→1598.**
- Cost: full 3-arm spike ~3–4 min release; Tet10 solves dominate (~10× Tet4, indefinite LU).

### 0.2 Q1 — is `BondedSandwich` generic over the element? **CLEAN for `E`; but `Msh` changes too (v2 correction).**

The bond math in `sim/L1/coupling/src/bonded.rs` is **entirely node-based** (`Bond` = vertex-
id sets + body-frame rest offsets; `resolve` writes posed Dirichlet targets, calls
`replay_step`, reads `nodal_reaction_forces`, reduces to a wrench). The **only** three Tet4
sites: the `use Tet4` import, `type DiscSolver<Msh> = CpuNewtonSolver<Tet4, Msh, NullContact>`
(`bonded.rs:165`), and the `CpuNewtonSolver::new(Tet4, ..)` call (`bonded.rs:300`). Verified
exhaustive by a full read of all 686 lines: no hardcoded `4`, no `tet_vertices` use, no
per-corner loop. `resolve`, `face_wrench`, `face_reaction_cotangent` and `body_pose_cotwist`
all index by the bonded `Vec<VertexId>` and `3 * n_vertices`. The solver is already generic
(`impl<E,Msh,C,M,const N,const G>`), and the Tet10 bonded **forward** path (`replay_step`,
`nodal_reaction_forces`) has **no `N==4` assert that fires** (only the `fbar && N!=4` guard,
and the disc has `fbar:false`).

**★★ v2 CORRECTION — the scope claim in v1 §1 ("No downstream public-enum break; the element
type param is additive with a default") was FALSE.** A Tet10 disc needs a mesh that supplies
midsides, and `Mesh::tet_midside_nodes` defaults to `None` (`sim/L0/soft/src/mesh/mod.rs:105`)
with an override **only** on `Tet10Mesh` (`sim/L0/soft/src/mesh/tet10_mesh.rs:328`) — never on
`SdfMeshedTetMesh`. So `BondedSandwich<SdfMeshedTetMesh, Tet10, 10, 4>` type-checks but is
*broken*; the real type is `BondedSandwich<Tet10Mesh, Tet10, 10, 4>`. **`Msh` itself changes**,
which propagates `BondedDisc` → `CoupledFsu` → `pub fsu: CoupledFsu`, a **public field** in a
downstream crate (`tools/cf-spine-studio/src/scene.rs:222`). §2.1 resolves this at rung 0
rather than deferring it — it determines whether the studio-cost mitigation (§5.1) is even
expressible.

Two further diamond-review fixes, both still standing:
- **Parameter order must be `Msh`-first.** `BondedSandwich<E, Msh, ..>` (E first) would
  *source-break* 5 explicit `BondedSandwich<SdfMeshedTetMesh>` sites (prod field
  `sim/L1/fsu-model/src/lib.rs:133` + 4 in `sim/L1/coupling/tests/rung6c_disc_geometry.rs:204,
  292, 308, 495`). `<Msh, E, N, G>` keeps `<SdfMeshedTetMesh>` binding `Msh` → byte-identical.
  Verified by compiling a minimal reproduction of the declaration plus all four use shapes.
- **The differentiable `probe_with_pose_gradient` is UNGATED for Tet10.** Its VJP passes the
  frictionless guard (`SolverConfig::skeleton()` sets `friction_mu: 0.0` and `bonded.rs` never
  overrides it, so the `mu == 0.0 || N == 4` assert passes for N=10), so a Tet10 pose gradient
  would run silently. All four callers are tests (`coupling/src/tests.rs:1859, 1888`,
  `tests/bonded_pose_gradient.rs:161`, `tests/rung6c_disc_geometry.rs:550`); `cf-codesign`
  consumes `StaggeredCoupling`, not the bonded disc — so this is a **genuine deferral, not a
  regression in disguise**. §2.3 restricts it at the type level.

### 0.3 The reframe — the disc is ~0.4% of segment ROM (diamond-review, head-engineer call)

The 33% k_disc softening is large **in the disc's own FOM**, but the assembled `CoupledFsu`
bakes the disc as a hinge spring that is **~0.4% of the flexion restoring moment at ROM**
(the segment is ligament-k-dominated: at ~6° flexion, `M_disc ≈ 0.28·0.105 ≈ 0.029 N·m`
against the `|M| = 7.5 N·m` band). A 33% softer disc shifts segment ROM by **~0.008°** —
invisible.

**★ v2 correction to the supporting claim.** v1 said the ROM verdict is "a `println!` in an
`#[ignore]`d, non-CI test — **not an enforced gate today**". That is true of
`rung7_fsu_validation.rs` (`#[ignore]` at `:282`; the two `report_rom` calls at `:465-471`),
but v1 **missed a hard assert elsewhere**:

- `sim/L1/coupling/tests/fsu_coupled_contact.rs:40-47,72` asserts `CoupledFsu::build`'s
  `k_disc` to `RUNG7_K_DISC = -0.2819 ± K_DISC_TOL = 0.02`, plus flexion ROM to
  `6.13 ± 0.15°` (`:118`) and the extension ROM literature band (`:246-251`).
- A Tet10 disc gives `k ≈ -0.186`; `|−0.186 − (−0.2819)| = 0.096` — **that assert fails by
  ~4.8×.** *(Magnitude is indicative, not exact: −0.186 is the spike's ±0.5° mean on the
  **raw** mesh, whereas `K_DISC_PROBE` is 0.86° flexion on what will by rung 4 be a curved,
  conformed disc. The direction and the order of magnitude are solid; the re-anchor value
  comes from rung 4's own measurement.)*

The reframe itself **survives**: `fsu_coupled_contact.rs:52` is *also* `#[ignore]`d and
env-gated on the BodyParts3D triad, so it is not CI-enforced either. What changes is the
**ladder**: a deliberate, committed **re-anchor** of `RUNG7_K_DISC` is a required deliverable
(rung 4, §3) — with the measurement in the constant's doc comment, per
[[feedback_anti_rot_invariants_vs_exact_anchors]]. **Never widen `K_DISC_TOL`**, and never
touch `ROM_TOL_DEG` / `LIT_EXTENSION_DEG`: those are the arc's real ROM tripwire, and the plan
*predicts* a 0.008° shift — if they move, the prediction was wrong and that is a finding.

**Consequence (settled): the arc's payoff is DISC-PHYSICS CORRECTNESS + exact geometry, not
a segment-ROM swing.** Gate on the standalone k_disc FOM **and** the exact-geometry residual
(§4.3), with ROM as a *sanity* check ("nothing broke"), not the payoff metric.

---

## 1. What the migration touches — recon sizing (v2: wider than v1 claimed)

- **`sim/L0/soft`** — element/enrich/`Tet10Mesh`/`with_sdf_projected_boundary`/curved element
  all merged. Rung 3 **adds** `Tet10Mesh::with_projected_midsides(&[(VertexId, Vec3)],
  quality_floor)` — see §2.6; the existing `with_sdf_projected_boundary` is **not** reusable.
- **`sim/L1/coupling`** (`bonded.rs`) — parameterize `BondedSandwich<Msh>` → `<Msh, E, N, G>`
  (Msh-first), `Tet4` default. Node-based methods unchanged. `probe_with_pose_gradient` moves
  into a Tet4-only impl block (§2.3).
- **`sim/L1/fsu-model`** (`lib.rs`, `coupled.rs`) — `BondedDisc` and `CoupledFsu` gain
  `<Msh, E, N, G>` with Tet4/`SdfMeshedTetMesh` defaults (§2.1). New full-face band logic
  (§2.2). `CoupledFsu::build` flips to the curved Tet10 conformed disc at rung 4.
- **`tools/cf-spine-studio`** — *not in v1's scope list, but it is in scope.* `pub fsu:
  CoupledFsu` (`scene.rs:222`) is a public field; and its `capture_ramp` capture is documented
  at ~85 s (`scene.rs:314`, `coupled.rs:246`) — a Tet10 disc makes that an estimated ~14 min
  (§5.1 shows the arithmetic and its 14–45 min band).

**Downstream re-grade list** ([[feedback_grade_downstream_crates_on_public_type_change]] —
`BondedSandwich`/`BondedDisc`/`CoupledFsu` are public types): `sim-coupling`, `cf-fsu-model`,
`sim-bevy-soft`, `cf-spine-studio`, `cf-codesign`, `cortenforge` (facade), `sim/sim`, and the
five `examples/integration/*-viewer` crates. Run `cargo xtask grade <crate>` on each before
push; do not assume the default keeps them clean.

---

## 2. Architecture decisions

### 2.1 Parameterize, `Msh`-first — and resolve `CoupledFsu` NOW, not "at rung 2"

```rust
// sim/L1/coupling/src/bonded.rs
pub struct BondedSandwich<Msh = HandBuiltTetMesh, E = Tet4, const N: usize = 4, const G: usize = 1>
// sim/L1/fsu-model — same shape, so the studio's `pub fsu: CoupledFsu` stays source-compatible
pub struct BondedDisc<Msh = SdfMeshedTetMesh, E = Tet4, const N: usize = 4, const G: usize = 1>
pub struct CoupledFsu<Msh = SdfMeshedTetMesh, E = Tet4, const N: usize = 4, const G: usize = 1>
```

`Msh`-first keeps every explicit `<SdfMeshedTetMesh>` annotation binding `Msh` → the Tet4
monomorphization is unchanged (§0.2). Defaults keep `scene.rs:222` compiling untouched.

Three mechanical corrections the stress-test found in v1's §2.1:

1. **The `DiscSolver` alias must be spelled out in full.** `CpuTet4NHSolver` /
   `CpuTet10NHSolver` (`sim/L0/soft/src/lib.rs:108-109, 126-127`) are two *distinct* aliases
   pinned to concrete `<Tet4,…,4,1>` / `<Tet10,…,10,4>`; a generic alias cannot select between
   them. Use `type DiscSolver<Msh, E, const N: usize, const G: usize> =
   CpuNewtonSolver<E, Msh, NullContact, NeoHookean, N, G>`. Reassuringly that *is* today's type
   — `CpuNewtonSolver`'s defaults are `M = NeoHookean, N = 4, G = 1`, so spelling them out is a
   no-op for Tet4.
2. **Add `E: Element<N, G> + Default` to the impl bound.** `bonded.rs:300` passes an element
   *value*, but `Element` (`sim/L0/soft/src/element/mod.rs:21`) does not require `Default`.
   Satisfiable: `Default` is derived on both `Tet4` and `Tet10`.
3. **`MAX_NEWTON_ITER` must become an *associated* const.** It is a module-level free `const`
   today (`bonded.rs:54`); a free `const` item cannot reference an impl's generic `N` (E0401).
   Use `const MAX_NEWTON_ITER: usize = if N == 4 { 50 } else { 400 };` on the generic impl.
   v1's parenthetical *"(a cap, not a byte-identity risk either way)"* was wrong: the cap is
   hard-failing, not truncating (`newton.rs:537` returns `SolverFailure::NewtonIterCap`,
   and `replay_step` panics on it). Honest phrasing: **Tet4 keeps 50 → identical; the only
   path raising it could change is one that currently hard-fails.**

### 2.2 The full-face bonded band — NEW logic, and v1's recipe would have panicked

For a physically-correct Tet10 bond the whole bonded region follows the endplate, so the
bonded set = band corners **plus** every midside whose two parent corners are both in the
corner band. v1 wrote this as a per-tet loop that **emits duplicate `VertexId`s** — and
`BondedSandwich::from_tet_mesh` hard-asserts uniqueness (`bonded.rs:263-271`, *"the lower
bonded face has a duplicate vertex id"*), because a repeat would double-count that node's
reaction in the endplate wrench. Corrected recipe:

```rust
use sim_soft::element::TET10_EDGE_NODES;   // NOT re-exported at the crate root:
                                           // sim/L0/soft/src/lib.rs:58 exports only {Element, Tet4}
// corner_band: computed PRE-enrich on the Tet4 mesh, banded on the SURFACE-AABB z-range
// (lib.rs:301-308), orphan-filtered. Still valid post-enrich: enrichment preserves corner
// VertexIds (sim/L0/soft/src/mesh/enrich.rs:12-16, test `corners_and_positions_preserved`).
let mut band: BTreeSet<VertexId> = corner_band.iter().copied().collect();  // BTreeSet, NOT HashSet
for t in 0..tet10.n_tets() as TetId {
    let c = tet10.tet_vertices(t);
    let Some(m) = tet10.tet_midside_nodes(t) else { continue };  // no .unwrap() — grade Safety cell
    for (i, &(a, b)) in TET10_EDGE_NODES.iter().enumerate() {
        if corner_band.contains(&c[a]) && corner_band.contains(&c[b]) {
            band.insert(m[i]);
        }
    }
}
let verts: Vec<VertexId> = band.into_iter().collect();
```

**`BTreeSet` is load-bearing twice over:** it dedups (the panic above), and it fixes iteration
order — `face_wrench` sums `Σ rᵢ × fᵢ` in `bond.verts` order (`bonded.rs:637-648`), so a
`HashSet` would make the k_disc FOM non-bit-reproducible run to run. The existing Tet4 band is
ascending (`pick_vertices_by_predicate` walks `positions()` in id order); keep that.

**★★ THE SILENT TRAP — `referenced_vertices` is corner-only by construction.** It walks
`mesh.tet_vertices` only (`sim/L0/soft/src/mesh/mod.rs:472-480`), so if the migration keeps
today's band code (`fsu-model/src/lib.rs:298-308`) and merely swaps the mesh type, `pick_vertices_by_predicate`
**will** pick midsides and the `referenced.contains(v)` filter **will** silently delete every
one of them — **reproducing the exact under-tie that inflated 43%, this time by omission.**
Note the solver's *internal* referenced set does add midsides (`backward_euler/construct.rs:356-376`, `if N > 4`)
but it is private, so the two disagree by design. **Do not reuse `referenced_vertices` to filter
a Tet10 band.** (Orphan midsides cannot exist — midsides are created only inside the tet walk,
`enrich.rs:83-103` — so no midside analogue of the orphan filter is needed.)

**★ Name the modeling choice.** The Tet4 band rule is purely **geometric** (a z-slab predicate
over all positions); this midside rule is **topological** (closure over band corners). They are
not the same set: a midside between an in-band corner and an out-of-band corner can sit
*inside* the slab yet stay free under the topological rule, so the geometric generalization
would pin strictly more midsides and give a *different* ratio. We choose the topological
closure because it guarantees "element fully inside the band ⇒ fully pinned" and avoids
half-pinned edges. **1005/1598 and 0.665 are specific to this rule** — this is the §2.2
instance of [[feedback_spike_verifies_math_gate_verifies_modeling_choice]].

**★ Correct the rationale.** v1 said "a midside between two bonded corners lies on the bonded
face". It does not: `DiscParams::band_frac = 0.18` (`fsu-model/src/lib.rs:103`, "band thickness as a fraction of the disc's SI extent", one band each side) makes the bonded set a
**volumetric slab** (~36% of the disc rigidly tied), so most pinned midsides are *interior*.
The honest statement: *a Tet10 slab tie must pin the midsides interior to the slab as well as
those on its face — otherwise the slab is corner-spot-welded.* This matters at rung 3, where
only the **surface-restricted subset** gets projected.

**★ No readout change is needed** (v1 implied one). `nodal_reaction_forces` returns the full
`3·n_vertices` vector with all 10 nodes at N=10, `face_wrench` iterates `bond.verts`, and
`from_tet_mesh` derives the Dirichlet set *from* the bonded set (`bonded.rs:295-297`). So
**pinned ≡ bonded ≡ read**, and the corners-only artifact structurally cannot recur on the read
side — *provided* midsides enter via the `lower_verts`/`upper_verts` passed to `from_tet_mesh`,
never via a separate `BoundaryConditions` path. State that as the invariant.

**★ Cite the edge table; never re-type it.** `TET10_EDGE_NODES` is
`[(0,1),(1,2),(0,2),(0,3),(1,3),(2,3)]` at `sim/L0/soft/src/element/tet10.rs:63-67`, matching
the shape functions node-for-node, pinned by `tet10_mesh.rs::tet_midside_nodes_match_canonical_table`
and `enrich.rs::local_slots_match_canonical_table`. Never use the `(min,max)` global dedup key
for the local slot (`enrich.rs:18-26` warns about exactly this). **A permuted table would be
caught by no k_disc gate** — it pins the wrong midsides, a partially self-cancelling error that
lands inside any plausible ratio band. The §4.2 cross-check exists for this.

> **⚠ Measured at rung 1 — correcting this paragraph's original claim that a permuted table
> "pins roughly the same *number* of midsides".** Rotating the slot index by one on the synthetic
> disc moved the band from 413 to 647 ids, so *that* permutation would also have tripped an exact
> size assert. Set equality is still the right instrument (a size-preserving permutation is
> possible, and a size assert would then see nothing), but do not repeat the claim that a permuted
> table is invisible to the band counts.

### 2.3 Differentiable bond — restrict at the type level, not with a runtime panic

Keep the forward path fully generic. Move `probe_with_pose_gradient` into
`impl<Msh: Mesh> BondedSandwich<Msh, Tet4, 4, 1> { … }`. Tet10 then simply **does not have the
method** — no panic branch to test, no unreachable code costing `xtask grade` Coverage, and no
way for a future consumer to reach it at runtime. (v1 proposed a fail-loud runtime `N==4`
guard; the parameterization hands us a compile-time one for free.) FD-gate the N=10 channel
only when a co-design consumer needs a Tet10 disc pose gradient. Out of ROM-validation scope.

### 2.4 Newton budget + LU fallback are expected

The Tet10 bonded solve is indefinite-tangent-heavy (LU fallback most iterations —
[[simsoft-nonpd-lu-fallback]], benign, converging at the shipped resolution). Scale
`MAX_NEWTON_ITER` for N>4 as an associated const (§2.1); document on it. The plan does not
chase PD-ness.

### 2.5 Ordering invariant: conform → enrich → project midsides → **bond**

The bond is the terminal step. `BondedSandwich::from_tet_mesh` snapshots
`rest = mesh.positions().to_vec()` (`bonded.rs:277`) and `Bond::new` freezes body-frame offsets
from it — **anything that moves a node after that point silently ties the bond to the
un-projected geometry.** That is the reachable hazard, and v1 did not state it.

v1's stated hazard ("enrich before conform leaves midsides behind") is **already type-enforced
and unreachable**: `with_projected_nodes` exists only on `SdfMeshedTetMesh`
(`sdf_meshed_tet_mesh.rs:356`); `Tet10Mesh` has no corner-conform method, so "enrich first,
then conform corners" does not type-check. Keep it as a note, not a risk.

Supporting facts (verified): enrichment uses **current** positions (`enrich.rs:95`,
`midpoint = (positions[va] + positions[vb]) * 0.5`), so conform-then-enrich is correct; and
corner ids/positions are preserved bit-identically, so a pre-enrich `corner_band` stays valid.
⚠ `Tet10Mesh::from_tet4`'s "source must be linear" check is a `debug_assert!`
(`tet10_mesh.rs:105-110`) — **absent in release, and the FOM probes run release.**

### 2.6 Rung 3 needs a NEW sim-soft helper — `with_sdf_projected_boundary` is not reusable

v1 said rung 2b would "reuse `boundary_faces6` selection (#700) + SI-alignment + quality-floor
back-off (#701)" and called the mitigations "all built + proven". **The Tet10 quality floor is
not built.** `Tet10Mesh::with_sdf_projected_boundary` (`tet10_mesh.rs:219-311`) takes *one*
`Sdf`, projects **every** boundary midside of the whole disc (including the annular rim the
#701 settled call requires to stay straight), has no distance cap and no direction gate, and
its validity predicate is a bare `d.is_finite() && *d > 0.0` (`:265`) — exactly what #701's own
doc rejects: *"a bare `detJ > 0` bisects each backed-off node onto the `detJ → 0⁺` degeneracy
boundary — manufacturing slivers exactly where the move was largest"*
(`sdf_meshed_tet_mesh.rs:325-329`).

⇒ Rung 3 adds **`Tet10Mesh::with_projected_midsides(&[(VertexId, Vec3)], quality_floor)`** —
the exact Tet10 analogue of `SdfMeshedTetMesh::with_projected_nodes` (caller-supplied targets +
fractional quality floor, anatomy-free), reusing `boundary_faces6` only for *candidate
selection*. The anatomy discriminator stays in `cf-fsu-geometry` (nearest-of-two oracles,
`SI_CONFORM_CAP_BONDED`, `SI_CONFORM_MIN_ALIGN`).

**The inversion oracle is `Tet10::rest_jacobian_dets` over all 4 Gauss points of every incident
element — NOT `mesh.quality()`.** `Tet10Mesh` never recomputes `QualityMetrics` after a midside
move and those metrics are 4-corner quantities anyway (`tet10_mesh.rs:114`, `mesh/mod.rs:52-67`),
so a `quality()`-based check is structurally blind to midside-induced degeneracy.

---

## 3. The ladder — build order (v2 RESHAPED; numbering differs from v1)

Each rung: recon-confirm → commit → gating cold-reads → fixes-on-top → ASK-push → CI → merge.

**★★ Two ordering changes from v1, both load-bearing:**

- **Tet10 lands on the RAW mesh first (rung 1), conform comes after (rung 2).** v1 gated
  rung 2a on `ratio ≈ 0.665` but *re-measured on the conformed mesh*, where nobody has measured
  anything — an anchor against an unmeasured baseline. On the raw mesh the expected value is
  **predicted in advance** (0.665, §0.1), so the first Tet10 gate becomes a
  validate-the-harness-against-a-known-value ([[feedback_validate_new_harness_against_known_value]])
  instead of a guess. The two axes are independent: conform touches `fsu-model/src/lib.rs:338`, element
  touches `bonded.rs:165`.
- **`CoupledFsu` is flipped exactly ONCE, at the end (rung 4).** v1 flipped it at rung 1 and
  again at rung 2a, which would have required two separate `RUNG7_K_DISC` re-anchors — and
  v1's rung-1 gate could not have detected either flip (below). Rungs 1–3 move one variable at
  a time on the **standalone** disc, where the FOM is ~3 min (§5.1) and the gates actually
  observe the change.

---

**Rung 0 — parameterize + a license-free byte-identity golden.** `sim-coupling` + `fsu-model`
(the `BondedDisc`/`CoupledFsu` defaults, §2.1) + `probe_with_pose_gradient` into a Tet4-only
impl block (§2.3). Forward-inert.
**Gate:** existing tolerance tests unchanged **+ a new `to_bits` golden** — see §4.1 for why
v1's version could not be built. Re-grade the 12 downstream crates (§1).

> **✅ BUILT (2026-07-28).** One refinement found during the build, recorded so a later reader
> does not over-read this rung: **`CoupledFsu` gained the type parameters but kept a CONCRETE
> `impl`.** `CoupledFsu::build` assembles through `build_bonded_disc`, which returns the Tet4
> arm, so a `CoupledFsu` over another element would have no methods and nothing constructs
> one — genericizing those methods with no consumer to gate them would be speculative
> ([[feedback_consumer_gated_completeness]]). The struct parameters still discharge rung 0's
> actual job (proving the public type stays source-compatible, incl. `cf-spine-studio`'s
> `pub fsu: CoupledFsu`). `BondedDisc`'s `impl` **is** generic, which is what rung 1 needs.
> Method genericization moves to rung 4, with its real Tet10 consumer.

**Rung 1 — straight Tet10 + full-face bond, STANDALONE, on the RAW mesh.** `fsu-model` builds a
`BondedDisc<Tet10Mesh, Tet10, 10, 4>` (enrich §2.5, full-face band §2.2). **`CoupledFsu`
untouched.**
**Gate (§4.2):** re-earn 0.665 as committed code — pre-registered bracket, band-count
cross-check, soundness. This is the rung that converts §0.1's reverted spike into a re-runnable
measurement.

> **✅ BUILT (2026-07-28).** `build_bonded_disc_tet10` + `full_face_band`, both arms fed by one
> new private `prepare_disc` (mesh → bands → conform → posed two-box scene) so the element is
> the *only* difference between them by construction, not by inspection. **The spike is
> reproduced exactly:** bands `228 → 1005` / `367 → 1598`, nodes `7849 → 19449`, and
> `k_disc` Tet4 −0.2811 flex / −0.2788 ext vs Tet10 full-face −0.1873 / −0.1849 ⇒ **ratio
> 0.666 flex / 0.663 ext** (pre-registered bracket 0.60..=0.73, then tightened to ±5 % of the
> measured values). Per-direction means reproduce the spike to four decimals (Tet4 −0.27995 vs
> −0.2800; Tet10 −0.1861 vs −0.1861). FOM cost **122 s**, close to §5.1's ~3 min estimate.
>
> **The table's "Tet4 path untouched" was PROVEN, not argued.** Both arms share one new private
> `prepare_disc`, and the refactor is a *pure move*: a sorted code-line multiset diff of the old
> `build_bonded_disc` body against `prepare_disc` + the new Tet4 tail removes **zero** old code
> lines (the only additions are the struct signature and field forwarding), so no floating-point
> operation changed position — the recipe from `feedback-pure-move-refactor-recipe`.
>
> **★ One §4.2 correction the build found:** step 3's "the same set computed a second way —
> every midside slot of every `boundary_faces6` face whose 3 corners are all in the corner
> band — asserted **identical**" cannot hold. §2.2 itself says the band is a volumetric slab
> whose pinned midsides are *mostly interior*, and an interior midside appears on no boundary
> face — so equality is false by construction and that assert would have failed on day one
> (the same shape of defect as v1's day-one ROM assert, §4.6). The built cross-check instead
> identifies each midside's parent corners **geometrically** (a straight Tet10 midside sits at
> exactly `(p[a] + p[b]) · 0.5`, so midpoint coincidence names its parents with no reference to
> `TET10_EDGE_NODES`) and asserts set equality against the band actually bonded; the
> boundary-face route is kept as the **subset** check it can be. **Teeth measured, not
> asserted:** a permuted slot table gives 413 vs 647 ids and a `referenced_vertices` filter
> gives 413 vs 111 — both fail, on the license-free synthetic disc, in CI.

**Rung 2 — Strategy-B endplate conform, measured on BOTH elements, standalone.** Reuses #701's
`with_projected_nodes` + `bonded_conform_target` machinery on the Tet4 corner band, then
enriches (§2.5).
**Gate (§4.3):** the **exact-geometry residual gate** (new — this is the arc's stated payoff and
v1 gated it nowhere) + the per-element conform delta + **the large-angle sweep** (§4.5). Still
no `CoupledFsu`.

> **✅ BUILT (2026-07-28).** Gates only — **no production code changed.** The plumbing rung 1
> shipped (`build_bonded_disc_tet10(.., Some(endplates))`, and `prepare_disc` conforming corners
> *before* the enrichment) already had the right shape, so this rung is measurements, not
> machinery: the §4.3 residual gate, the §4.5 sweep, the per-element delta table, rung 3's
> committed "before" arm, and a license-free residual arm for CI.
>
> **The byte-identity in the table above is structural, and that was verified mechanically rather
> than by eye:** diffing hunk line numbers against the start of `mod tests`, the only changes
> outside the test module are `//!` module-doc comments in `lib.rs` and one `//` comment in
> `coupled.rs::build` — no code line outside the tests module changed. Rung 1's FOM re-runs
> unchanged (0.666 / 0.663) to confirm it behaviourally as well.
>
> - **§4.3 residual (the arc's payoff, gated for the first time).** Of **583** bonded-face
>   boundary nodes the SI-alignment guard authorises **233**; **231** are delivered and **2 are
>   backed off entirely** by the quality floor. Over the *authorised* set the distance to the
>   nearer vertebra falls **max 5.724 → 3.575 mm, RMS 1.332 → 0.656 mm** — the conform halves the
>   seated population's distance to the bone. It does not zero it: the back-off refuses to invert
>   a tet, so the worst authorised node still ends up 3.575 mm out.
> - **★ The gate is on the AUTHORISED set, not the moved set** — "nodes that moved" is selected on
>   the outcome. This is what the plan meant by watching for vacuity, and the 2 dropped nodes are
>   invisible to #701's committed `max_seat`. **★★ What the choice buys was MEASURED on two
>   mutants, and it is narrower than the tidy argument:** with 214 of 233 authorised nodes dropped
>   from the move list, a strict-decrease gate on the *moved* set still improves
>   (RMS 2.000 → 1.103) and **passes**, while the authorised set's max stays 5.724 → 5.724 and
>   **fails** — that is the case the definition exists for. But against a back-off that degrades
>   every node alike (`DISC_CONFORM_QUALITY_FLOOR` 0.05 → 0.50) the two sets move together (moved
>   RMS 0.659 → 0.908 vs authorised 0.656 → 0.901) and the choice buys **nothing**; there it is the
>   committed population split and the ±5 % pins that catch it. Both mutants trip the split assert.
> - **★ The rim is reported, never averaged in.** 350 of 583 candidates are guard-declined (the
>   overhanging annulus, max 12.577 mm — a closest-point artifact of reaching sideways for the
>   body wall), left straight by #701's settled call. They are reported separately; the
>   all-candidate figures (RMS 3.494 → 3.416) are gated only on *non-increase*, since a declined
>   node's residual is identical in both arms by construction.
> - **§4.5 large-angle sweep — PASSED, and it retires a deferral.** Both conformed arms complete
>   the full **±6.0°** chain in 0.1° steps (180 solves/arm), every step converging, conserving and
>   strictly restoring; peak |M| 0.0300 (Tet4) / 0.0205 (Tet10) N·m. So **`coupled.rs`'s "a
>   whole-face-conformed mesh spawns sliver tets that fail the sweep" is a STRATEGY-A fact and
>   does not apply to the Strategy-B node conform** — that reason for rung 4's deferral is gone,
>   leaving only the `RUNG7_K_DISC` re-anchor. And the Tet10 angle envelope, previously driven
>   only to ±0.5°, is measured and is not narrower than the linear arm's.
> - **Per-element conform delta.** `k_disc` (N·m/rad), flex / ext: Tet4 raw −0.2811 / −0.2788 →
>   conformed −0.2760 / −0.2738; Tet10 raw −0.1873 / −0.1849 → conformed −0.1844 / −0.1820.
>   **The two axes are nearly orthogonal** — conform ratio 0.982 / 0.984 (same to within 0.2 % on
>   both elements), element ratio 0.666/0.663 raw vs 0.668/0.665 conformed.
> - **★ CLAIM CORRECTED: the conform costs ~1.8 % of `k_disc`, not ~4 %.** #701's FOM printed its
>   arms at two decimals (−0.28 → −0.27) and the ~4 % figure was read off that rounding; at four
>   decimals it is −0.2811 → −0.2760. Corrected at both twins (`lib.rs` FOM comment and
>   `coupled.rs::build`) — the #701/rung-0/rung-1 half-fixed-twin pattern, caught by grepping for
>   the number rather than re-reading the file.
> - **★ A stepping fact the build measured, and the reason the delta FOM is stepped:** the RAW
>   Tet4 disc survives a single +0.5° → −0.5° jump (rung 1 probes it exactly that way) but the
>   **conformed** one does not — the same 1° jump drives tet 7495 to a principal stretch of 2.845
>   and trips the solver's fail-closed validity bound. Every arm of the delta table is therefore
>   walked in ≤ 0.5° increments, and the raw column still reproduces rung 1's two-probe numbers to
>   four decimals, so the stepping does not move the measurement.
> - **Cost, measured:** residual FOM **0.8 s** (no solve — it reads the rest configuration the
>   bond snapshots), delta FOM **96 s**, large-angle sweep **897 s idle / 1138 s under load**
>   (Tet10-dominated, ~5 s per warm-started 0.1° solve). The sweep is a same-shape check on §5.1's *estimate* for a Tet10
>   `capture_ramp` (150-175 solves, 14-45 min): 180 solves in ~15 min lands at the bottom of that
>   band. Not a substitute for rung 4 measuring `capture_ramp` itself, which also runs the
>   per-frame equilibrium bisection.
> - **§4.7 license-free coverage, with teeth NOTHING ELSE IN THE CRATE HAS — measured.** The
>   synthetic conform test now also gates the residual against its box "endplates" (conformed
>   max/RMS < 10 % of raw), so the residual machinery is CI-enforced even though every anatomy gate
>   here is `#[ignore]`d. A mutant that seats only every other band node **survives every #701
>   assert in that test** — `seated > 0` holds, both frame-bridge z-extremes still land on their
>   box faces because the nodes that do seat set them, and the sweep still solves — and is caught
>   by the new residual assert alone, at max 2.0000 → 2.0000 with RMS only halved
>   (1.4577 → 1.0349). (A first mutant, disabling the conform outright, was caught by #701's
>   pre-existing assert instead and so proved nothing about the new one — the separating mutant is
>   the one recorded.) ⚠ The synthetic geometry has *no* declined nodes, so the guard-decline path
>   itself is exercised only on real anatomy.
> - **★ COVERAGE: this rung cost the crate a letter grade, and the fix is where the gates LIVE.**
>   `cargo xtask grade` Coverage runs `cargo llvm-cov --lib`, which instruments the whole library
>   target *including* `#[cfg(test)] mod tests` — so an `#[ignore]`d, licence-gated FOM charges its
>   entire body to the grade while executing in no coverage run, ever. Measured with the identical
>   command on both trees (baseline in an isolated worktree, so the working tree was never at
>   risk): `lib.rs` regions **1562 → 2254**, of which **596 of the 692 added are uncovered (86 %)**;
>   crate lines **71.9 % → 57.9 %**, a B → C drop. `coupled.rs` came back **byte-identical**
>   (697 regions / 137 missed on both sides), which independently re-confirms that no production
>   code changed. **Do not read this as pre-existing** — `main` was already under the 75 % A bar,
>   but the 14-point fall is this rung's.
>   **Fix applied here:** `sim-coupling` keeps all 35 of its gates in `tests/`, including its own
>   licence-gated ones, and `cf-fsu-model` had no `tests/` directory at all. The two rung-2 gates
>   that touch **no private item** — `conform_delta_by_element` and
>   `conformed_disc_large_angle_envelope` — moved there, which costs **no API surface** and keeps
>   this rung's zero-production-diff property. The other two genuinely need `prepare_disc` /
>   `BondedDisc.sandwich` and so must stay in the library target.
>   **Measured:** lines **57.9 % → 63.9 %** (regions 54.7 % → 60.1 %), so the move recovers **6 of
>   the 13.9 points**, and `cargo xtask grade cf-fsu-model` goes **F → B**: Coverage 63.9 % **B**,
>   Clippy 0 **A**, Documentation 0 **A**, Safety **A**, Dependencies **A**. **That is the same
>   letter `main` carries** (71.9 % is also B), so the rung ships at grade parity — the C was only
>   ever the pre-move state. Both moved gates re-run identical (`k_disc` table unchanged to four
>   decimals; the sweep still reaches ±6.0° at peak |M| 0.0300 / 0.0205, 936 s), so the move is
>   behaviour-preserving.
>   **⚠ Parity is not the bar, and the gap is real:** the branch still sits **−8.0 pts** under
>   `main` inside that band, and B is not A. That remaining 8 is precisely the two library-bound
>   gates.
>   **New CI surface, verified not assumed:** this gives `cf-fsu-model` its first integration-test
>   binaries. `quality-gate.yml:471` puts the crate in `tests-release` shard 1 and that step runs
>   `cargo nextest`, which schedules "across ALL test binaries" — so the two new files are compiled
>   and run by CI (their `#[ignore]`d bodies skipped, exactly like the lib gates). No
>   `tests-debug` enum registration is needed; the crate is in no debug shard.
>   **⚠ Named, not silently absorbed:** the residual and rung-3-baseline gates still sit in
>   `src/lib.rs`, as do rung 1's `tet10_full_face_bond_element_order_fom` and #701's FOMs. Moving
>   *those* needs `prepare_disc` and a bonded-band accessor made public — a crate-wide change with
>   nothing to do with the Tet10 ladder, so it belongs in its own PR, not in this rung.
> - **Also committed as rung 3's "before" arm:** the straight-Tet10 bonded-face boundary midsides
>   (1562 of them) sit max 12.464 / RMS 3.366 mm off the bone. ⚠ Like the all-candidate corner
>   figures, that aggregate is dominated by the rim that stays straight by design — rung 3 must
>   compare the *authorised* region like for like, not quote a drop in this number as its payoff.

**Rung 3 — curved Tet10: project bonded-face boundary midsides onto the real endplate.** The
exact-geometry endgame. New `sim-soft` helper (§2.6) + `fsu-model` wiring; interior-band
midsides stay at edge midpoints (projecting them was the #699 false-degradation bug); the
overhanging annular rim stays straight (#701 settled call — Sharpey's fibers attach to the ring
apophysis, not the endplate face).
**Gate (§4.3 + §4.4):** residual strictly decreases again; coverage + quality floor; k_disc
additive shift committed.

**Rung 4 — flip `CoupledFsu`, re-anchor once, ROM verdict.** `coupled.rs:187` `None` →
the curved-Tet10 conformed disc. **Deliverables:** (a) the **single** deliberate re-anchor of
`RUNG7_K_DISC` (`fsu_coupled_contact.rs:41`) with the measured value in its doc comment;
(b) the **full-ROM ramp gate** (§4.5); (c) the flexion-only ROM assert (§4.6); (d) the
studio-element decision, *measured* (§5.1); (e) the doc-drift sweep (§7).

**Rung 5 — h-refinement: does `k_disc` converge?** §0.1 bound 3 admits −0.186 is not proven
converged, which is precisely why rung 1 gates a bracket rather than a point. One refinement
arm (`cell` 0.003 → 0.002) on the curved Tet10 disc, plus a refined **Tet4** arm to check it
moves *toward* the Tet10 value rather than away.
**Gate:** committed convergence table. **This is what promotes the claim from "we measured a
delta" to "Tet10 is the more accurate element"** — it is the rung that earns the arc's
justification, not a §6 deferral. (v1's rung 3 was a phantom: its gate column duplicated rung
2b's with no independent falsifier. It is replaced by this.)

---

## 4. Gates — written so each one can fail

| rung | crate(s) | payoff gate | sanity | byte-identity |
|---|---|---|---|---|
| 0 | coupling, fsu-model | — (inert) | existing tolerance tests unchanged | **§4.1 license-free `to_bits` golden, bits frozen pre-change** |
| 1 | fsu-model | **§4.2** ratio bracket + band-count cross-check | restoring + conserving + converged, both arms | Tet4 path untouched |
| 2 | fsu-model | **§4.3** exact-geometry residual ↓ | **§4.5** large-angle sweep; per-element conform delta | Tet4+raw arm byte-identical |
| 3 | sim-soft + fsu-model | **§4.3** residual ↓ again; **§4.4** coverage + quality floor | k_disc additive shift committed | straight-Tet10 arm untouched |
| 4 | fsu-model, coupling | single `RUNG7_K_DISC` re-anchor, measured | **§4.5** full ramp; **§4.6** flexion ROM assert | — |
| 5 | fsu-model | committed convergence table | — | — |

**★ CI reality, stated plainly** (v1's table implied protection that does not exist): `sim-coupling`
and `cf-fsu-model` run only in `tests-release` shard 1 (`.github/workflows/quality-gate.yml:471`);
they are in **no** `tests-debug` shard, so no `tests-debug` enum registration is needed. But CI
never runs `--ignored`, and **every real-anatomy gate is `#[ignore]`d + env-gated on the ephemeral
BodyParts3D triad.** So: **§4.1's golden and the §4.7 synthetic arms are the ONLY CI-enforced
gates in this arc; everything else must be run by hand before each push.** Do not let the table
read as CI protection ([[feedback_hedges_compress_out_in_summaries]]).

### 4.1 Rung 0's golden — license-free, full-surface, frozen pre-change

v1 specified "a new committed `to_bits` golden on the Tet4 bonded **k_disc**". That quantity
exists **only** on the licensed-anatomy path (`build_bonded_disc` on a BodyParts3D STL, every
consumer `#[ignore]`d) → the golden would never execute in CI, so risk 3's mitigation would not
mitigate. Replacement:

- **Fixture:** `sim/L1/coupling/src/tests.rs:1755 fn bonded_fsu()` — the 2-cell
  `HandBuiltTetMesh` block, license-free, `--lib` (which is also what `xtask grade` Coverage
  measures). Not weaker than the real disc for this purpose: the parameterization is purely
  type-level, and both monomorphizations instantiate the *same* generic bodies in `resolve` /
  `face_wrench` / `Bond`.
- **Fingerprint:** after a compression `probe()` and a flexion `probe()`, XOR-of-`to_bits` plus
  an `L2.to_bits()` over the **full** `last_reaction()` and `last_targets()` slices — the repo
  idiom at `sim/L0/soft/tests/tet10_material_sensitivity.rs:293-296, 326-341`. Covers the whole
  `resolve` output surface, not one scalar.
- **★ Bits frozen on `main` @ `59e61daa`, BEFORE the parameterization commit**, carried into the
  same PR, capture command recorded in the PR body. v1 was silent on capture order; bits captured
  *after* the refactor make the gate tautologically green
  ([[feedback_meta_audit_introduced_claims]]). Precedent states this explicitly:
  `tet10_material_sensitivity.rs:26` *"(fingerprints frozen from the pre-widen code)"*.
- **Do NOT** implement it as an in-binary A/B between `BondedSandwich<SdfMeshedTetMesh>` and
  `BondedSandwich<SdfMeshedTetMesh, Tet4, 4, 1>` — after the change those are the *same type* and
  the comparison is vacuous. The only meaningful comparison crosses the refactor boundary in time.
- **No literal `to_bits` on any ≳10k-DOF solve.** Determinism was *measured*, not assumed —
  but by the v2 review, **not** by a committed test (which is exactly what this golden fixes;
  until it lands, the precondition is unverified by this repo's own standard): three
  consecutive runs of `bonded_sandwich_fsu` are byte-identical, every `HashSet` on the bonded path
  is `len`/`contains`-only, `largest_component` carries an explicit deterministic tie-break, and
  faer is built without rayon (`sim/L0/soft/Cargo.toml:74`) so parallelism is `Par::Seq`. But at
  ~23.5k DOF faer switches to supernodal kernels with runtime `pulp::Simd` CPU-feature dispatch,
  and a partial-pivot LU can amplify a reduction-order difference into a pivot flip. For any
  real-disc number use a **1e-10 relative tolerance**, never `to_bits`.

### 4.2 Rung 1's payoff gate — a pre-registered bracket, plus a cross-check the ratio can't fake

One committed test, both arms on the **raw** mesh, corners byte-identical, differing only in
element + band:

1. **Soundness (hard):** both arms restoring, conserving (`‖ΣF‖+‖ΣM‖ < 1e-8`), converged to
   solver tol — asserted, not printed.
2. **Direction + magnitude (hard, pre-registered):** `|k_tet10| < |k_tet4|` **and**
   `ratio ∈ 0.60..=0.73`, asserted **per direction** (flexion and extension separately — §0.1
   quoted a *mean*, and the arms differed 1.3%). The bracket is pre-registered around the spike's
   0.665 — **a prediction from reverted code (§0.1), not a verified anchor** — because this rung
   reproduces the spike's exact configuration; a value outside it means the harness does not
   reproduce the spike and the rung stops.
3. **★ Full-face-tie cross-check (hard, non-vacuity):** exact band sizes asserted `==`. The
   spike's 228→1005 / 367→1598 are **expected values from reverted code, not anchors** — measure
   them in this rung and commit the measured numbers; a mismatch against the spike is itself the
   finding. **Plus the same set computed a second way**, asserted identical. This is the gate that
   *reliably* catches a `TET10_EDGE_NODES` mis-index or a silent `referenced_vertices` filter
   (§2.2) — the size assert catches some of those too, but only when they happen to change the
   size (see §2.2's rung-1 measurement). A ratio band catches neither reliably: a **partial**
   under-tie moves the ratio *toward* 0.570 and lands comfortably inside any band drawn around
   0.665. (A *fully* corner-only tie at 0.570 does fall outside both the bracket and the shipped
   ±5 % pin — it is the partial case and the permuted table that the stiffness cannot see.)
   **⚠ Corrected during the build:** the "second way" **cannot** be the `boundary_faces6` route
   this plan originally named (every midside slot of every face whose 3 corners are in the band).
   The band is a volumetric slab and its pinned midsides are mostly *interior* (§2.2), so they
   appear on no boundary face and set equality is false by construction. The independent
   recomputation is **geometric** instead — midpoint coincidence identifies each midside's parent
   corners without consulting the slot table, which is what makes it independent of the code
   under test — and the boundary-face route survives as a *subset* assert. See rung 1's BUILT
   note in §3.
4. **Pin (fixup commit, after the first green run):** tighten step 2 to a ±5% no-regression
   assert around the measured value, measurement in the doc comment — the #701 shape
   (`fsu-model/src/lib.rs:1222`, `:1229`).

### 4.3 The exact-geometry residual gate (rungs 2 and 3) — NEW; the arc's payoff had no gate

Every gate in v1 measured `k_disc`, node-move magnitude, or ROM. **None measured the arc's
stated justification.** #701's FOM commits `max_seat` — how far nodes *moved* — not how close
they *ended up* to the bone, and the SI-alignment guard can silently decline nodes
(`fsu-model/src/lib.rs:186-195`).

**Gate:** committed **max and RMS `|oracle.eval(node)|`** over the bonded-face boundary nodes,
asserted (a) below a committed threshold and (b) **strictly smaller than the previous rung's
arm** — raw → conformed at rung 2, straight → curved at rung 3. Two-sided committed bands, per
the #701 shape. This is the direct measurement of *exact-geometry-IS-exact-physics*; `k_disc`
is the physics consequence, not the geometric claim.

### 4.4 Rung 3's validity gate — must be stricter than the helper guarantees

v1's "0 inverted / 0 sliver" was **tautological** (the projection back-off makes the straight
position always feasible, so non-inversion is a construction guarantee) and its sliver half was
**false** (no Tet10 quality floor exists — §2.6). Replacement:

- **Coverage, not max-move:** the fraction of bonded-face boundary midsides that reached their
  *full* projection (`|moved − target| < 1e-9`), committed, with a two-sided band on both
  `max_move` and `mean_move`. Both `with_projected_nodes` and `with_sdf_projected_boundary`
  back off **silently** — a run where 95% of midsides stayed straight would pass a
  non-vacuity-plus-max-move gate.
- **Element validity:** assert `detJ ≥ 0.05·detJ_rest` at every Gauss point of every incident
  element — *stricter* than the helper's `detJ > 0`, therefore falsifiable. If it fails, the fix
  is the quality floor in the new helper (§2.6), not a widened gate.
- Note the analogous guard is **vacuous at rung 1**: a straight Tet10 built from a valid Tet4 is
  affine with the identical corner Jacobian.

### 4.5 The large-angle gate — rungs 2 and 4 (v1 had no rung exercising production angles)

Every FOM in v1 is a **sub-degree** probe, explicitly scoped: *"inside the spike-validated
conformed SPD range (±0.5°)"* (`fsu-model/src/lib.rs:1136`). But `K_DISC_PROBE` is **0.86°**
(`coupled.rs:48`) — already outside it — and `capture_ramp` walks the FEM disc to the full
±ROM (~6° flexion / ~4.5° extension) in 0.1° sub-steps (`coupled.rs:52, 454`). The repo's own
deferral note names large-angle failure as the reason the conform was deferred: *"a whole-face-
conformed mesh spawns sliver tets that fail the sweep"* (`coupled.rs:139-141`).

- **Rung 2 (standalone, cheap):** a large-angle symmetric sweep to ±6° on the conformed disc,
  asserting every step solves and stays restoring. Catches the documented sliver-at-angle failure
  *before* rung 4 pays for a full ramp.
- **Rung 4 (integrated):** `CoupledFsu::capture_ramp(&moment_ramp())` completes on the curved
  Tet10 disc, with committed max node displacement and a per-frame `detJ > 0` check on the
  *deformed* configuration. Note `fsu_coupled_contact.rs:169-173` does **not** cover this — its
  equilibrium assert runs the analytic bushing path and never re-solves the FEM.

### 4.6 ROM: flexion-only, and only after it has been measured once

v1 said "promote `report_rom`'s band membership to a real `assert`". Two problems:

1. **It cannot trip for rungs 1–3.** `rung7_fsu_validation.rs` has **zero** `CoupledFsu`
   references — it builds its own disc with `None` (`:300-308`), its own ligaments, its own
   facet grid. Only rung 4 changes anything it observes.
2. **A blanket promotion fails on day one.** `report_rom` is called twice; the **extension** call
   is documented *"the physiological extension limiter is facet contact, excluded from the
   headline"* (`:466-471`), so disc+ligaments never reach 7.5 N·m, `rom_at` returns `None`, and
   `report_rom` takes the "too LAX" branch — **which has no band at all.** There is nothing to
   promote. (`rom_at` returns `Option<f64>`; a naive band assert does not even type-check without
   deciding what `None` means.)

**Settled:** at rung 4, promote **flexion only** —
`assert!(rom_at(..).is_some_and(|d| LIT_FLEXION_DEG.0 <= d && d <= LIT_FLEXION_DEG.1))` — with an
assert message saying this is a band-membership sanity check on an uncalibrated `K_LIG = 20.0`,
not a payoff metric. The **extension** call stays a `println!`; the enforced extension band lives
in `fsu_coupled_contact.rs:246`. Also note the tension this creates with rung 7's own design
(*"match-or-REPORT: a measured gap is a valid result, not a failure"*) and say so, rather than
silently converting a reporting harness into an exact anchor.

### 4.7 License-free Tet10 coverage — the only thing CI can actually run

`cargo test --release -p cf-fsu-model --lib` is **1.64 s** (13 passed / 4 ignored; the 4 are the
anatomy gates). All 13 passing tests run on `synthetic_disc()` — a 24×20×6 mm box at `cell = 3 mm`
(`fsu-model/src/lib.rs:594-599`) — and `synthetic_fsu()` passes `None` explicitly (`coupled.rs:814-815`), so
**with no action the Tet10 path would execute in no CI job at all.**

Add, per rung, a `synthetic_disc()`-based Tet10 bonded probe asserting: restoring + conserving +
converged, corner count preserved, midside count added, and the §4.2 band-count cross-check
(on synthetic numbers). At 1.64 s today, even a 10× Tet10 arm costs ~16 s in a 45-min job — free.

---

## 5. Risks the build must manage

### 5.1 Cost — the risk is real but v1 pointed it at the wrong target

v1 attributed the cost risk to the k_disc FOM gate. **The gate is cheap; the studio is not.**

**Arithmetic** (anchors: `coupled.rs:246` *"`capture_ramp` (~85 s vs the ~5 s build)"*;
`CAPTURE_SUBSTEP = 0.1°`; travel 0 → −4.5° → +6.0° = 15.0° ⇒ 150 sub-steps + ≤25 per-frame
roundings ⇒ 150–175 solves ⇒ Tet4 per-solve ≈ 85/160 ≈ **0.53 s**):

| path | Tet4 | Tet10 (~10×/solve) |
|---|---|---|
| **rung-1/2/3 k_disc FOM** (2 arms × ~6 solves) | ~25 s | **~3 min** — not a blocker |
| **`capture_ramp`** (150–175 solves) | ~85 s | **~14 min** (band 14–45 min) |

The 14–45 min band accounts for referenced DOF going 6 771 → ~41 600 and the iteration cap
rising 50 → 400. **The inherited "~60 min" figure is from `tet10_indentation_demand1.rs:492`** —
a face-contact indentation test, single-threaded, no warm start, different mesh. Not transferable
in either direction; do not quote it for this path.

That ~14 min lands on **`tools/cf-spine-studio`** (`scene.rs:333` drives `capture_ramp`), whose
capture is documented at ~85 s in three places (`coupled.rs:246`, `scene.rs:267`, `scene.rs:314`).

**★ OPEN DECISION for rung 4 — what does the studio's disc default to?** Recommendation:
**Tet10, consistent with [[feedback_exact_geometry_is_exact_physics]]** (cost is explicitly not
the decider for contact fidelity), with `CAPTURE_SUBSTEP` made N-aware the way `MAX_NEWTON_ITER`
is (0.1° → ~0.3° cuts to ~5 min) **and the coarser step re-validated against §4.5's sweep**.
Decide it on a **measured** capture time at rung 4, not on this estimate. If the measurement
forces Tet4 for the studio, that must be a named, documented default — and note the coupling it
creates: `k_disc` is read off `render_disc` (`coupled.rs:192`), so "Tet4 render disc" silently
means "Tet4 `k_disc`" unless the measurement is split from the render disc.

### 5.2 Curved-midside inversion (rung 3) — the #699/#700 failure mode

Mitigate with boundary-faces-only projection + a quality floor + the rest-Jacobian guard.
**⚠ Correction to v1's "all built + proven": the Tet10 quality floor does NOT exist** (§2.6). The
Tet4 one does (`DISC_CONFORM_QUALITY_FLOOR = 0.05`, `fsu-model/src/lib.rs:72`); rung 3 must port it.

### 5.3 The parameterization silently changes Tet4 (rung 0) — mitigated by §4.1's golden

…which does not exist today, and which v1 specified in a form that could never have run.

### 5.4 ROM does NOT break — but a hard `k_disc` assert does

The disc is ~0.4% of ROM, so a 33% softer disc cannot move the band; `ROM_TOL_DEG` and
`LIT_EXTENSION_DEG` are tripwires, not payoff gates, and if they move the *prediction* was wrong
— re-anchor honestly, never tune Tet10 to the old number
([[feedback_anti_rot_invariants_vs_exact_anchors]]). **What does break is `RUNG7_K_DISC ± 0.02`
(§0.3), by roughly 4.8× the tolerance** — an indicative magnitude, not an exact prediction
(§0.3 states why: raw-mesh ±0.5° mean vs a 0.86° probe on a conformed, curved disc). That is a
planned, single, documented re-anchor at rung 4, not a surprise.

### 5.5 Rollback — what happens if a gate comes out wrong (v1 had no clause)

- **Rung 1 ratio outside `0.60..=0.73`:** stop. The harness does not reproduce the spike. Debug
  the band first (§4.2 step 3 will usually already have fired), then the enrichment. **Do not
  widen the bracket to admit the observed value** — that is the falsified-spec case; revert
  opt-in and keep the surface ([[feedback_spec_falsified_revert_opt_in_keep_surface]]).
- **Rung 2/3 residual does not decrease:** the conform/projection is not seating. Treat as a
  geometry bug, not a gate-tuning problem.
- **Rung 5 shows Tet4-refined converging away from Tet10:** that falsifies §0.1 bound 3's
  optimistic reading and the arc's accuracy claim must be *narrowed in writing* to the measured
  delta. The element-order justification survives; the accuracy claim does not.

---

## 6. Deferred / open questions (named, not silent)

- **Differentiable Tet10 bond** (`probe_with_pose_gradient` at N=10) — type-restricted to Tet4
  now (§2.3); FD-gate when a co-design consumer needs it. **Verified not a regression:** no
  production caller exists (`cf-codesign` consumes `StaggeredCoupling`).
- **Studio disc element default** — §5.1, decided on measurement at rung 4.
- **Viz: the studio renders the Tet4 field even with a Tet10 disc.** v1 noted only that
  `soft_boundary_faces` is corner-only so edges render straight. It is worse than that: the
  skinning weights at `scene.rs:178-204` are inverse-distance over the *corner-only* candidate
  set, so the ~11 600 midside DOFs — the entire quadratic enrichment that makes rung 3
  meaningful — contribute **nothing** to the rendered surface. Nothing breaks (corner ids are
  preserved), but it silently renders the linear field. Cheap fix available today: feed
  `boundary_faces6()` (`mesh/mod.rs:189`, already implemented for Tet10) into
  `weighted_tet_nodes`' candidate set — corner triangles for topology, all 6 nodes for sampling.
  **This is a direct dent in `rendered === contacts` and should be its own follow-on PR.**
- **`BondedDisc` generic vs. parallel constructor** — resolved: generic (§2.1). The rung-1 gate
  needs both element arms live simultaneously, which settles it.
- **Curved-element quadrature budget** — 4-pt kept; the #699 rung measured K-error grows
  ~0.22·(sagitta/edge) *(inherited from the curved-element arc, not re-measured here)*; a
  bonded-face boundary element bows ~3 edges, adequate for a well-resolved disc surface.

---

## 7. Doc-drift checklist (a rung deliverable, not an afterthought)

[[feedback_no_rationalizing_doc_drift]]. Rung 4 invalidates:

- `sim/L1/fsu-model/src/coupled.rs:135-186` — the ~10-line block stating *"The BONDED FEM disc
  here stays on the RAW (un-conformed) geometry (`None`)"*, including the claim *"flipping it
  would shift the `k_disc` baked into the rung-7-validated ROM equilibrium"* — **already
  inaccurate today**, since rung 7 does not use `CoupledFsu` (§4.6). ⚠ **Rung 2 discharged two
  parts of this early**, because leaving them would have been drift rather than deferral: the
  "~4 %" shift is measured at ~1.8 %, and the *"a whole-face-conformed mesh spawns sliver tets
  that fail the sweep"* rationale is a Strategy-A fact that the ±6.0° Strategy-B sweep retires.
  What is left for rung 4 is the `None` → conformed-Tet10 flip itself and the `RUNG7_K_DISC`
  re-anchor.
- The "~85 s" / "~5 s" capture figures: `coupled.rs:246`, `coupled.rs:875`,
  `tools/cf-spine-studio/src/scene.rs:267, 314` (and the `:7`, `:14`, `:214`, `:234` prose).
- Module docs: `sim/L1/fsu-model/src/lib.rs:8, 30, 35`; `tools/cf-spine-studio/src/main.rs:8, 21, 30`.
- ~~`sim/L1/fsu-model/src/lib.rs:1136`'s "±0.5° spike-validated conformed SPD range" scope
  note~~ — **done at rung 2**, along with the module-level "the quadratic arm's large-angle
  envelope is unmeasured" note, both superseded by §4.5's measured ±6.0°.

---

## 8. Provenance

### 8.1 v1 — the 5-front diamond-review

1 isolated-worktree Q0 re-measurement + 4 read-only audits. Material catches: the **Q0 bond
confound** (two independent agents — corners-only pinning inflated 43% → corrected ~33%/0.665,
§0.1, §2.2); the **ROM reframe** (disc is ~0.4% of ROM, §0.3); **Q1 param-order source-break +
VJP guard scoping** (§0.2, §2.1, §2.3); **enrich-after-conform ordering + rung-2a baseline
reconciliation + no-Tet4-golden** (§2.5, §3, §4); an **honesty pass**.

### 8.2 v2 — the 4-front stress-test (this revision)

Four read-only adversarial agents (read-only by design —
[[feedback_parallel_mutating_coldreads_need_isolation]]), on gate falsifiability, rung-0
byte-identity feasibility, the §2.2 band recipe vs the real APIs, and cost/omissions. The
head engineer independently verified the load-bearing claims before folding them in
(`tet_midside_nodes` impls, the studio's public field, the uniqueness assert, the ignore-gating
of `fsu_coupled_contact.rs`). Ladder-changing catches:

- **v1's rung-1 gate could not fail** — `disc_endplate_conform_moment_rotation_fom` builds its
  own raw and conformed discs and never mentions `CoupledFsu`, so it passes bit-identically
  before and after the flip it was meant to gate. → §3 (rungs decoupled from `CoupledFsu`).
- **A hard `k_disc` assert v1 missed**, which *v1's* rung 2a would have broken by roughly 4.8×
  the tolerance (indicative magnitude — see §0.3's caveat). → §0.3, rung 4.
- **v1's ROM sanity assert could not trip and would fail on day one.** → §4.6.
- **v1's rung-0 golden could never have run in CI.** → §4.1.
- **v1's §2.2 recipe would have panicked** (duplicate midside ids), and reusing today's band
  code would silently re-create the corners-only under-tie. → §2.2.
- **v1's scope claim was false** — `Msh` changes too, breaking a downstream public field. → §0.2, §1, §2.1.
- **The arc's own stated payoff had no gate.** → §4.3.
- **The cost risk pointed at the wrong target** (the FOM gate is ~3 min; the ~14 min lands on
  the studio's `capture_ramp`). **Both are ESTIMATES** from the committed ~85 s / ~5 s anchors,
  not measurements of a Tet10 run — see §5.1's arithmetic and its 14–45 min band, and do not
  let the hedge drop in a summary ([[feedback_hedges_compress_out_in_summaries]]). → §5.1.
- Plus: rung ordering (Tet10-on-raw first, validate against a known value), v1's rung 3 was a
  phantom (replaced by h-refinement), the `MAX_NEWTON_ITER`/alias/`Default`-bound mechanics,
  the missing rollback clause, doc-drift list, and downstream re-grade list.

Prior arcs: [[project-tet10-fbar-element-upgrade]] (element ladder),
[[project-fsu-disc-endplate-conform]] (#701 conform machinery),
[[project-keystone-soft-rigid-coupling]] (BondedSandwich keystone). Q0 spike (reverted):
built `build_bonded_disc`'s disc pipeline, ran the node-based bonded probe through Tet4 /
straight-Tet10-corners-only / straight-Tet10-full-face-tie arms; validated Tet4 vs the
shipped −0.28. Q1 read: `bonded.rs`, `fsu-model/src/{lib,coupled}.rs`, the generic
`CpuNewtonSolver<E,Msh,C,M,N,G>`.
