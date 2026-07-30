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
> ladder (v2, §8.2). **Rungs 0 (#704), 1 (#705), 2 (#706), 3 (#707) and 4 are built — see each rung's
> BUILT note in §3, which records what the build changed about the plan. Rungs 4b and 5 are
> independent — 4b seats the coupled disc, 5 bounds its error. **Next action: rung 5**, specced
> against the shipped code on 2026-07-30 and then **reshaped by a 4-front adversarial stress-test**
> (§3's rung-5 entry, §4.8's gate, §8.3's record). It goes first because `RUNG7_K_DISC = −0.1882`
> is *shipped and load-bearing* while §0.1 bound 3 still admits it is not proven converged — rung 5
> hardens the layer underneath 4b. ⚠ **The stress-test changed the rung's QUESTION:** "is Tet10
> more accurate here" is a **theorem** (§3), so rung 5 measures *how much error is left* and
> delivers a **bracket**, not an h-refinement convergence claim. **Rung 5.0 steps 0 and 1 are ✅
> DONE, and step 1 BLOCKED the h-ladder on the real disc** (§3 confound 1): a **1.67 % cell change
> moves `k4` by 16.7 %**, because the superior bonded band loses **47 % of its pinned nodes** — the
> clamp *depth* is stable (0.065 %) but the pinned *population* is not, and the §4.8 gate as
> written would have missed it. **▶ NEXT: a head-engineer call on the fork** — ship the measured
> **mesh-realization uncertainty band** on `RUNG7_K_DISC` (~9 % on `k10`: ~¼ of the whole element
> effect, ~180× rung 3's curving effect), and defer h-convergence to a rung that first makes the
> band population converge.**

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
  fallback fires every iteration (benign, [[project-simsoft-nonpd-lu-fallback]]) but Newton reaches
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
  **raw** mesh, whereas `K_DISC_PROBE` is a 0.86° flexion probe. The direction and the order of
  magnitude are solid; the re-anchor value comes from rung 4's own measurement.)* **✅ MEASURED at
  rung 4: −0.1882 (straight Tet10, the shipped arm), i.e. 4.69 × the tolerance — the indicative
  figure was right.** ⚠ **STALE PREMISE CORRECTED (rung-5 spec, 2026-07-30):** this parenthetical
  used to say the probe ran on "what will by rung 4 be a **curved, conformed** disc". It does not
  — rung 4 SPLIT, and `CoupledFsu::build` passes `None` (`fsu-model/src/coupled.rs:299`), so the
  shipped arm is **raw and straight**; conforming it is rung 4b. The half-fix was visible in place:
  the ✅ MEASURED sentence was appended while the parenthetical's premise was left standing
  ([[feedback_meta_audit_introduced_claims]]). This matters for rung 5, whose amendment 1 rests on
  `RUNG7_K_DISC` tracing to the **raw** disc.

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
  all merged. Rung 3 **added** `Tet10Mesh::with_projected_midsides(&[(VertexId, Vec3)],
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
[[project-simsoft-nonpd-lu-fallback]], benign, converging at the shipped resolution). Scale
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

> **✅ BUILT as specified.** One thing this section got right and one it left open. Right: the
> helper is anatomy-free with caller-supplied targets, and the oracle is
> `Tet10::rest_jacobian_dets` over all four Gauss points, not `mesh.quality()`. Left open: it
> said "the exact Tet10 analogue of `with_projected_nodes`" and the build took *the quality
> floor value* along with the shape — which cost a debugging cycle. A Tet10 midside needs its
> own floor (0.4 vs the corner 0.05); see rung 3's BUILT note in §3 for the measured table.
>
> ⇒ Rung 3 adds **`Tet10Mesh::with_projected_midsides(&[(VertexId, Vec3)], quality_floor)`** —
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
>   **The two axes are nearly orthogonal** — conform ratio **0.982 / 0.982 (Tet4) and 0.985 /
>   0.985 (Tet10)** (same to within 0.2 % on both elements), element ratio 0.666/0.663 raw vs
>   0.668/0.665 conformed. ⚠ This line read "0.982 / 0.984" until the rung-5 spec's stress-test
>   checked it against the committed asserts (`tests/conform_delta_by_element.rs:166-170`): **0.984
>   appears nowhere in the crate** — it was a stale pre-rung-3 extension figure (0.1820/0.1849).
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

> **✅ BUILT (2026-07-28).** `sim-soft` gains `Tet10Mesh::with_projected_midsides(&[(VertexId,
> Vec3)], quality_floor)` exactly as §2.6 specified (caller-supplied targets, anatomy-free,
> quality floor over **all four Gauss points**, corner ids rejected loudly); `fsu-model` gains
> `bonded_face_boundary_midsides` + `endplate_midside_conform_moves` and wires them into
> `build_bonded_disc_tet10`, so `Some(endplates)` now means **curved**, not merely conformed.
> The `None` arm is untouched — rung 1's element-order FOM re-runs at exactly 0.666 / 0.663 with
> `k_disc` −0.2811 / −0.2788 and −0.1873 / −0.1849, unchanged to four decimals.
>
> - **★★★ THE FINDING, and it changed a production constant: the corner quality floor is the
>   wrong floor for midsides, and reusing it produces a mesh that is valid by the projector's
>   own rule and no longer drivable.** At floor 0.05 the curved disc solves in 0.05° steps but
>   stalls Newton with a `NaN` residual on a single **0.15°** jump from rest — against 0.5° for
>   the straight-conformed disc rung 2 measured, a five-fold loss of step envelope, on the path
>   `capture_ramp` drives at 0.1° with a per-frame equilibrium bisection on top. The
>   `conform_delta_by_element` FOM found it (it walks ±0.5° steps and died), *after* the
>   residual gate had passed — the geometric gate cannot see conditioning. Fix: a separate
>   `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR = 0.4`, chosen on a measured seven-row table (in the
>   constant's doc): RMS residual 0.658 (floor 0.05) → 0.694 (floor 0.40), the 0.5° jump
>   stalling at 0.05/0.20 and solving from 0.25 up. **The whole cost of a well-conditioned mesh
>   is 0.036 mm of RMS residual**, and the restoring moment at 0.5° is identical to six digits
>   across every row — the floor moves the conditioning, not the physics. 0.40 is margin above
>   the measured 0.20→0.25 cliff, deliberately not the edge of it.
>   *Mechanism:* a Tet4 corner move perturbs an affine element's single Jacobian; a Tet10
>   midside move bends the Jacobian at four interior Gauss points, so the same *fraction* is not
>   the same distortion.
> - **§4.3 residual, like for like.** Of **1562** bonded-face boundary midsides the SI-alignment
>   guard authorises **580** (37 %, close to the 40 % it authorises among the corners — 233 of 583);
>   the other 982 are the rim, left straight by design. Over the *authorised* set the distance
>   to the nearer vertebra falls **max 3.860 → 3.842 mm, RMS 0.796 → 0.694**. ⚠ *Rung 4
>   re-anchored these when it raised `DISC_CONFORM_QUALITY_FLOOR` 0.05 → 0.25 — the midsides
>   are projected from conformed CORNERS, so the whole table moved (now 3.973 → 3.971 mm,
>   RMS 0.881 → 0.767, coverage 67.4 %). The numbers in this note are what RUNG 3 measured;
>   the committed pins are rung 4's.* ★ The endpoint is
>   the result worth quoting: rung 2 left the authorised **corners** at 0.656 mm RMS, so the
>   midsides now sit essentially as close to the bone as the corners they span (0.694 vs 0.656,
>   6 % apart) — **the bonded face is uniformly seated instead of seated at its corners and
>   chording between them.** The
>   relative move is smaller than rung 2's (which halved 1.332 → 0.656) for a structural reason:
>   the corners were conformed before enrichment, so a straight midside starts at 0.796, not
>   1.332. The max barely moves because the worst authorised midside is one the quality floor
>   refuses to seat — the same fact rung 2 recorded one level down.
> - **★ The like-for-like discipline was honoured, and it is checkable:** the ALL-candidate
>   straight figures come out at exactly rung 2's committed baseline (1562 midsides, max 12.464 /
>   RMS 3.366), because the gate measures both arms from one prepared mesh in one run. Rung 2's
>   standalone `straight_tet10_midsides_chord_across_the_endplate_fom` is therefore **replaced**
>   by this two-arm gate rather than left to drift alongside it.
> - **§4.4, written so neither half can be tautological.** *Coverage:* **67.9 %** of the
>   authorised midsides reach their FULL projection (max move 1.388 mm, mean 0.127), committed
>   two-sided — both helpers back off silently, so a run where most midsides stayed straight
>   passes a non-vacuity gate. (Which of the three members catches which failure is *not* the
>   tidy story it looks like — see the mutant record below.) *Element validity:* `detJ ≥ 0.4·detJ_rest` at every Gauss point of
>   **every** element, not merely the ones the projector's own incidence map covers — that
>   difference is the falsifiable content, and it is stated on the helper. ⚠ **The validity
>   *value* is NOT committed, on purpose:** bisection converges onto the constraint boundary, so
>   the worst ratio equals the floor exactly whenever any node backs off at all. Committing
>   0.4000 with a ±5 % band would read as a measurement and be a tautology — the very shape §4.4
>   exists to replace. The inequality is asserted; the coverage triple is the statistic that
>   moves — though *which member* moves depends on the failure, see the mutant record below.
> - **★ A selection variant was measured and REFUTED, not argued.** The natural hypothesis for
>   the 32 % non-delivery is that it concentrates in midsides whose parent corners were
>   themselves guard-declined. Gating on both parents being authorised gives 484 of 580 at
>   **79.5 %** delivered — barely better — while dropping 96 midsides that were *improving*
>   (RMS 1.477 → 1.198). The simpler rule ships. ⚠ That pair was measured at the corner floor
>   0.05, before the constant split (72.4 % ungated vs 79.5 % parent-gated) — do not read 79.5 %
>   against the 67.9 % committed above.
> - **★★ THE COLD-READ MUTANTS (three, all measured on the real disc).** (1) *Incidence map
>   narrowed* from `t[4..10]` to `t[4..7]`: worst `detJ/detJ_rest` → **−9.7870** — genuinely
>   inverted elements — while the §4.3 residual gate still IMPROVES (0.796 → 0.711) and passes.
>   A geometry gate cannot see a folded element; the whole-mesh validity sweep can. It also
>   exposed an ordering defect (the coverage pin fired first, so an inverted mesh was reported as
>   a coverage failure), fixed by asserting validity first. (2) *Silent 0.2 mm cap*: the
>   delivered fraction moves the WRONG way (67.9 % → 68.6 %) and `max_move`/`mean_move` plus the
>   §4.3 pin catch it — which **refutes this rung's own advertised justification** for the
>   fraction. (3) *Projection disabled outright* (the whole rung silently reverted): **six of the
>   seven** licence-gated anatomy gates stay GREEN — rung 2's corner residual gate, rung 1's
>   element-order FOM, #701's conform FOM, the replayable sweep, both lofted-disc gates — and
>   only this rung's gate fails, plus the license-free arm, so CI catches it too. That is the
>   separating mutant: it survives every pre-existing assert.
> - **k_disc additive shift, committed:** the conformed Tet10 arm goes −0.1844 / −0.1820
>   (straight midsides, rung 2) → **−0.1845 / −0.1821** (curved), a **0.05 %** shift — an order
>   of magnitude below the conform's own ~1.5-1.8 % and two below the element order's ~33 %.
>   That is the arc's framing holding, not a null result: the geometric claim is measured
>   directly by the residual gate, and `k_disc` is its consequence.
> - **§4.7 license-free arm — a planar fixture could not gate this rung at all.** A straight
>   midside between two corners seated on a *plane* already lies on that plane, so against the
>   box "endplates" #701 and rung 2 use, rung 3 is a no-op. The new `sphere_mesh` fixture (two
>   30 mm spheres tangent to the disc faces) is the smallest license-free geometry that produces
>   a chord gap: 604 midsides, residual max 0.0896 → 0.0001 / RMS 0.0291 → 0.0000 mm, 100 %
>   delivered, and the curved disc bonds and sweeps restoring — the only place CI drives a
>   genuinely curved element through the bonded solve.
> - **⚠ The `assert_full_face_band` trap was honoured:** both gates feed it the **pre-projection**
>   mesh. Its independence comes from midpoint coincidence, which the projection destroys by
>   design; the topology it checks is unchanged by the projection.

**Rung 4 — flip `CoupledFsu`, re-anchor once, ROM verdict.** `coupled.rs:187` `None` →
the curved-Tet10 conformed disc. **Deliverables:** (a) the **single** deliberate re-anchor of
`RUNG7_K_DISC` (`fsu_coupled_contact.rs:41`) with the measured value in its doc comment;
(b) the **full-ROM ramp gate** (§4.5); (c) the flexion-only ROM assert (§4.6); (d) the
studio-element decision, *measured* (§5.1); (e) the doc-drift sweep (§7).

> **✅ BUILT (2026-07-28) — and SPLIT. Rung 4 ships the ELEMENT; the conform becomes rung 4b.**
> `CoupledFsu::build` assembles on `build_bonded_disc_tet10(.., None)`. The method
> genericization rung 0 deferred landed with it: the `impl` is now generic over `<Msh, E, N, G>`,
> the struct's **defaults flipped to `Tet10Mesh/Tet10/10/4`**, and the linear arm stays
> constructible as `CoupledFsuTet4`. `cf-spine-studio`'s `pub fsu: CoupledFsu` compiles
> untouched, as rung 0 predicted.
>
> **★★★ THE SPLIT, and why this rung's own wording caused the trouble.** As specified, "flip to
> the curved Tet10 conformed disc" moves **two** variables — `CoupledFsu` previously passed
> `None`, so the flip changed the element *and* switched the conform on. Rungs 1–3 moved one
> variable each, deliberately; rung 4 quietly broke that discipline, and it cost three
> bisections to find out why the studio path was failing. Measured, on a **lofted** (painted)
> disc — the `cf-spine-studio` geometry — driven ±6° in 0.1° steps:
>
> | arm | flexion | extension |
> |---|---|---|
> | Tet4 raw / Tet10 raw | +6.00° | −6.00° |
> | Tet4 **conformed** | inverts at **+3.70°** | −6.00° |
> | Tet10 **curved** | inverts at **+2.40°** | inverts at **−2.80°** |
>
> ⚠ Both senses, and it matters: Tet4's conform fails in **flexion only** (it takes the full
> extension travel), and Tet10's worst direction is **flexion at +2.40°**, not the −2.80° this
> arc quoted while each arm was driven in a single sense. The earlier measurement also compared
> a +6° control against a −6° subject — two variables at once, in the gate built to enforce
> one-variable attribution.
>
> Both raw arms are fine ⇒ **the conform is the failing variable, not the element**, on both.
> So rung 4 ships the element and **rung 4b** ships the conform, with a measured entry
> criterion: *a conformed lofted disc completes ±6° on both elements*.
>
> - **(a) The flip + a source-compatibility limit rung 0's claim does NOT cover.** A type
>   parameter's **default does not disambiguate inherent-method resolution**: with `build` on
>   both concrete impls, every existing `CoupledFsu::build(..)` call site fails with `E0034
>   multiple applicable items in scope`. Measured, not predicted — it forced the baseline
>   constructor to be named `CoupledFsuTet4::build_baseline`. Defaults keep the *type*
>   source-compatible; they do not extend that to an overloaded associated function.
> - **★ An unplanned production change the flip forced: every drive must be WALKED.**
>   `K_DISC_PROBE` is 0.86°, past any from-rest jump a conformed disc is measured to survive.
>   `CoupledFsu` now tracks its disc's angle and routes the probe, the return to rest, and every
>   capture leg through one `drive_disc` stepper. Verified element-neutral: the linear arm still
>   reads **−0.2819 to four decimals**.
> - **(b) The single re-anchor: `RUNG7_K_DISC` −0.2819 → −0.1882** (4.69 × `K_DISC_TOL`;
>   `K_DISC_TOL` unchanged). The ratio **0.668** lands on rung 1's standalone straight-Tet10
>   ratio (0.666 / 0.663) — the cross-check that the flip carried the element effect into the
>   assembly without adding an assembly-level artifact.
> - **★★ §0.3's prediction HELD, measured rather than derived.** Both arms in one run
>   (`coupled_element_shift`): a **33 % softer** disc moves segment flexion ROM
>   **6.1321° → 6.1403° = +0.0082°** against a predicted ~0.008°. Extension moves +0.0008° —
>   10× less, because it is facet-capped, which is the mechanism the reframe claimed.
>   **`ROM_TOL_DEG` and `LIT_EXTENSION_DEG` did not move.**
> - **(c) §4.5 full ramp PASSES** (`coupled_tet10_ramp`): 25 frames, −4.474° … +6.140°, max node
>   displacement **2.5908 mm** (pinned ±5 %), and `detJ > 0` at every Gauss point of every
>   element **on each frame's deformed configuration** — an invariant nothing in this arc had
>   checked (floors and ±6° sweeps all gate the *rest* mesh). **The quadratic disc deforms with
>   *less* element distortion than the linear one** (0.8751 vs 0.8484).
> - **⚠ A trap that did not apply, recorded so it is not re-inherited:** `capture_ramp`'s
>   per-frame equilibrium bisection never touches the FEM — it roots on the analytic bushing
>   plus the facet SDF. The FEM only sees the monotone ≤0.1° chain.
> - **(d) §4.6 flexion-only promotion done — and its stated premise is FALSE.** §4.6 claims
>   "only rung 4 changes anything [`rung7_fsu_validation.rs`] observes". That test builds its own
>   disc (`build_bonded_disc(.., None)`) and has **zero** `CoupledFsu` references, so rung 4
>   changes *nothing* it observes. The promotion ships as an anti-rot anchor with that stated at
>   the call site. The asserts that **do** observe the flip are all in `fsu_coupled_contact.rs`.
> - **(e) §5.1 settled on MEASUREMENT: the Studio keeps the quadratic disc.** Build 6.9 s →
>   **67.8 s**; capture 32.2 s → **583.1 s (9.7 min)**. Rationale in `scene.rs`'s `BUILD_HINT`:
>   `k_disc` is read off the *render* disc, so a linear Studio would silently give the picture
>   different disc physics from the library's. `CAPTURE_SUBSTEP` N-awareness is named and **not
>   pulled** — a conditioning change needs its own measured envelope.
> - **★ §5.1's cost model was wrong in both factors, and they cancelled.** The committed "~85 s"
>   linear capture anchor was never measured: it is **32.2 s** (2.6× high). The per-solve element
>   ratio is **18.1×**, not ~10×. The 14–45 min estimate was close only by offset. Quote the
>   measurements, not the arithmetic.
> - **§7 doc-drift discharged**, including four **UI strings a user reads while waiting**; they
>   now derive from `scene::BUILD_HINT` / `CAPTURE_HINT`.
> - **§4.7 license-free arm:** a synthetic quadratic `CoupledFsu` (sharing one assembly with the
>   linear fixture) captures a ramp in `--lib`, asserting deformed-configuration validity and
>   that a **re-capture** runs and is deterministic (⚠ not that it walks back — the ramp's angles
>   come from the analytic bushing, so they match by construction; the walk-back is exercised by
>   the anatomy ramp at ±6°). Narrowed from ±0.5 to ±0.1 N·m
>   after measuring that the wider ramp cost 99 s of CI for coverage the narrow one gives
>   identically (15.6 s).
>
> **★★★ A DEFECT THIS RUNG FOUND IN RUNG-2 MACHINERY — `DISC_CONFORM_QUALITY_FLOOR` 0.05 → 0.25.**
> Before the split was settled, the first failure was at the *small-angle* probe, and it was a
> different bug: **0.05 was measured on ONE disc and does not transfer.** On a lofted disc it
> produces a mesh valid by the projector's own rule and undrivable — the 0.86° probe died on a
> fail-closed validity violation, on both elements. The quadratic element is the stricter
> constraint (the same direction rung 3 found for midsides) and its cliff is **bracketed in
> (0.10, 0.15]** — bracketed, not located, since drivability is a converge/refuse boundary the
> sweep only samples. 0.25 therefore sits 2.5× above the last floor known to stall and 1.67×
> above the first known to drive; the conservative reading is the second, still above rung 3's
> 1.6×. It costs 0.094 mm of RMS residual, and `k_disc` holds still across every drivable
> floor — **0.07 % (Tet4) / 0.14 % (Tet10)**, measured and asserted, so the floor moves the
> conditioning and not the physics. (That read "<0.1 %" until the audit gave it a producer:
> it was wrong, and the range quoted beside it — 0.12 % / 0.31 % — had refuted it in place
> all along, because nothing recomputed either.)
> This **generalizes rung 3's rule** from "do not inherit a tuning constant across element
> orders" to "**or across input geometries**". The full two-geometry table lives in the
> constant's doc, regenerated by `conform_quality_floor_selection_fom`.
>
> **★ The gate corrected the table on its first run.** The row for 0.10 read "Tet10 drives"
> — measured through the *production* step walker it stalls. The throwaway sweep that first
> produced the table walked to 0.86° in nine ~0.0956° steps; production walks 0.1° steps to
> 0.9°. A slightly further, slightly coarser walk answers differently at the margin. This is
> the concrete payoff of `max_drivable_angle` (one definition of "drives", at the production
> step) and of committing the sweep: a table nothing regenerates cannot notice it is stale. Consequently **rungs 2 and 3's committed tables were re-anchored** (corner
> residual max/RMS 3.575/0.656 → 3.833/0.750; midside 0.796→0.694 becomes 0.881→0.767; coverage
> 67.9 % → 67.4 %; #701 max seat 4.457 → 4.156 mm; population split 231+2 → 230+3, the split
> assert doing exactly the job rung 2 built it for). Rung 3's seven-row midside-floor table was
> measured *at* the old corner floor and is marked as the floor-to-floor comparison it is; the
> shipped 0.40 is re-verified drivable on both geometries, and the midside cliff was **not**
> re-derived (it could only move permissively).
>
> **New gate so this input is never untested again:** `b6_lofted_disc_bonds_seats_and_sweeps`
> now DRIVES the Strategy-B conformed lofted disc past `K_DISC_PROBE` on both elements. It
> extends an existing licence-gated test rather than adding one, so it costs no coverage.

> **⚠ COVERAGE COST, accepted deliberately (user call, 2026-07-28).** `cf-fsu-model` grades
> **C (58.2 %)** against `main`'s **B (71.9 %)** — Coverage alone; Clippy / Documentation /
> Safety / Dependencies are all A, and `sim-soft`, `sim-coupling` and `cf-spine-studio` are all
> A. The cause is the structural trap rung 2 documented: `cargo llvm-cov --lib` instruments
> `#[cfg(test)] mod tests`, so an `#[ignore]`d licence-gated gate charges its whole body to the
> grade while executing in **no** coverage run, ever.
>
> Unlike rung 2's, these gates **cannot** be moved to `tests/`: they need `lofted_disc`,
> `prepare_disc_at`, `ConformFloors` and `bond_prepared_*`, all private, and exporting them
> would mean adding public API whose only consumer is a test. The trade was made with the cost
> known: those gates are what caught the stale `0.10` drivability row and the false "<0.1 %"
> conditioning claim, and deleting them to protect a letter would optimise the metric against
> the thing the metric exists to measure. The crate-wide `tests/` migration
> ([[project-ci-per-crate-coverage-blind-spot]]) recovers ~8 of the 13.7 points and remains its
> own PR. CI runs `--skip-coverage`, so this gates no merge.

**Rung 4b — seat the coupled disc on the real endplate.** `CoupledFsu::build`'s `None` →
`Some(..)`, i.e. the exact-geometry payoff for the assembled segment. **Blocked on a measured
entry criterion, not a preference:** the Strategy-B conform must survive ±6° in 0.1° steps on a
**lofted** disc on both elements *in both senses* (today: Tet4 +3.70° flexion but a full
−6.00° extension; Tet10 +2.40° / −2.80°; both raw arms reach ±6° both ways). Note this is a *distinct* mechanism from the rest-mesh conditioning rung 4 fixed by
raising `DISC_CONFORM_QUALITY_FLOOR` — it is deformation-time inversion and survives that fix.
Likely lines of attack, none yet measured: do not move a node already seated (a lofted disc's
caps *are* the endplates, so most of its conform is a no-op that only adds risk); revisit the
band selection on a disc whose caps coincide with the bone; or a deformation-aware back-off.
**Gate:** the lofted arm of `b6_lofted_disc_bonds_seats_and_sweeps` extended to ±6°, plus the
`RUNG7_K_DISC` re-anchor that seating will force. ⚠ Do NOT carry a predicted value here: an
earlier draft quoted −0.1855 for the conformed arm, which no gate in the tree produces. 4b
measures it.

**Rung 5 — how much error is left in the shipped `k_disc`?** §0.1 bound 3 admits −0.186 is not
proven converged, which is why rung 1 gates a bracket rather than a point. Zero production diff,
like rung 2 — this rung is measurement only.

> **★★★ SPECCED 2026-07-30, then RESHAPED by a 4-front adversarial stress-test (§8.3).** The first
> draft asked "is Tet10 more accurate?" and answered it with an h-refinement ladder plus a
> Richardson extrapolation. The stress-test falsified enough of that draft to change the rung's
> *question*, not just its gates. Every correction below was earned against the shipped code; the
> superseded draft's errors are recorded in §8.3 so they are not re-introduced.

**★★★ THE REFRAME — "Tet10 is more accurate here" is a THEOREM, not a measurement.** Verified in
the code, not assumed:
- `nodal_reaction_forces` returns exactly `−f_int` — the consistent variational flux, not a
  recovered-stress surface integral (`sim/L0/soft/src/solver/backward_euler/assembly.rs:287-294`).
- `flexion_moment` sums `target × react` over the upper face with the pose imposed as an exact
  rigid rotation (`fsu-model/src/lib.rs:1104-1129`), and the box origin cancels.

So the measured moment **is** `dU_h/dθ` to Newton tolerance, and `k_disc = U_h''(0)`. Two
consequences follow with no experiment:
1. Conforming displacement FEM under affine Dirichlet driving over-estimates stiffness:
   **`|k_h| ≥ |k*|`, always.** The old draft's "prediction 1" asserted the *sign* of this — it was
   never falsifiable content.
2. On the **raw** disc the Tet10 is straight-sided on the *same partition*, so `P1 ⊂ P2`; a rigid
   rotation is affine, so the Dirichlet data is represented exactly in both spaces (**this is what
   the full-face midside tie buys**). Therefore **`|k10| ≤ |k4|` on every shared mesh, and Tet10
   is strictly closer to `k*`.** Rung 1's `k10.abs() < k4.abs()` assert cannot fail for a correct
   build.

⚠ Two stated premises, both cheap to keep honest rather than assume: the arms differ in
**quadrature** as well as element order (`Element<4,1>` vs `Element<10,4>`, and neo-Hookean is not
polynomial), so the nesting is approximate and *a slice of the 33.5 % is integration, not element
order, with nothing currently separating them*; and the bound needs the solver on an energy
**minimizer**, not a saddle. See §6 for the one-line check that would settle the second.

**⇒ THE RUNG'S ACTUAL DELIVERABLE — a bracket, not an extrapolation:**

```
|k*|  ≤  |k10(fine)|  ≤  |k10(coarse)|  ≤  |k4|
```

and the gap **`|k10(coarse)| − |k10(fine)|` is a committed LOWER BOUND on the error in the shipped
`RUNG7_K_DISC = −0.1882`.** That statement is monotone-free, noise-tolerant, needs no fixed
refinement ratio, and survives every confound below — which is exactly why it replaces the
extrapolated `k*`. **Richardson is DROPPED**, not deferred: see §8.3 finding 1 for why the
superseded ladder could not compute it anyway, and §6 for the honest route to a two-sided bracket.

**★ AMENDMENT (retained, and better-founded than first argued) — the RAW disc, `endplates = None`.**
The original spec said "the curved Tet10 disc". Three reasons, strongest first:
- **Straight-sided is what makes `P1 ⊂ P2` hold**, and the nesting is the whole theorem above.
- The 4-point Stroud rule is **degree-of-precision 2 — exact for the straight-sided linear-elastic
  integrand and explicitly not exact for a curved isoparametric Tet10** (`element/tet10.rs:41-53`),
  so a curved arm carries an extra, h-dependent variational crime.
- The conform's quality-floor back-off is **h-dependent** — `with_projected_nodes` backs off against
  the scale-free ratio `detJ ≥ floor·detJ_rest` while the target is a fixed physical point on the
  bone, so a fixed move degrades a smaller tet more and each level achieves *different geometry*.
- The number under test is rung 1's raw `0.666 / 0.663`, which is what `RUNG7_K_DISC` traces to
  (`coupled.rs:299` passes `None`; the coupled probe's own ratio is 0.668 at 0.86°).

⚠ **The trade this makes, named:** the conclusion transfers from the raw disc to the shipped one on
the assumption that conforming changes the *level* but not the *rate* — and the conform acts
precisely at the singular clamp ring where the rate is set. Rungs 2/3's 0.982/0.985 and 0.05 % are
**level** measurements at one `h`; they do not license a rate transfer. State this in the result.

### ⚠⚠ The confounds — one named in the first draft, and it was NOT the big one

**★★ (1) THE CLAMP PLANE IS LATTICE-QUANTIZED — the mechanism all four stress-test fronts found,
and ✅ MEASURED AT RUNG 5.0 STEP 0 TO BE NEGLIGIBLE AT THESE CELLS.** `BccLattice::new` anchors
node planes to **global multiples of `cell`**, not to the disc bbox
(`sim/L0/soft/src/sdf_bridge/lattice.rs:257-262,296-300`), so the geometry↔lattice phase changes
discontinuously with `cell`; the bonded band is a fixed physical slab — `band = band_frac·(hi_z −
lo_z)`, `band_frac = 0.18` off the *surface* AABB (`lib.rs:587,624-625`). The concern was that the
realized clamp depth is therefore quantized with a `cell`-dependent quantum.

> **✅ MEASURED (`rung5_step0_realized_band_across_the_ladder_fom`, committed BEFORE it ran):**
>
> | `cell` (m) | referenced corners | verts | band (`cell/2` units) | inferior band | superior band | **free height** |
> |---|---|---|---|---|---|---|
> | 0.0030 | 2 257 | 7 849 | 2.318 | 228 | 367 | **12.392 mm** |
> | 0.0020 | 8 307 | 28 032 | 3.477 | 667 | 1 895 | **12.404 mm** |
> | 0.0015 | 18 485 | 56 403 | 4.637 | 1 328 | 4 717 | **12.381 mm** |
>
> **Disc SI extent = 19.3190 mm** (native), so **`band` = 3.4774 mm.**
>
> **★★★ THE ILLUSTRATION THAT PRECEDED THIS WAS WRONG BY ~70×, AND WRONG IN ITS PREMISE.** The
> pre-measurement text assumed a ~10 mm disc, giving a ~1.8 mm band *smaller than the coarse
> cell* and a **±13 %** non-monotone swing in free height. Measured: the disc is **19.32 mm**, the
> band is **3.48 mm — LARGER than the coarse cell**, and the free height is constant to
> **0.19 %** (12.392 / 12.404 / 12.381 mm). ⇒ **The clamp-DEPTH confound is real in mechanism and
> negligible in magnitude.** ⚠⚠ **But do NOT read that as "the ladder stands" — I did, and step 1
> falsified it below.** Free height is one of *two* things the lattice phase controls, and it is
> the stable one; the pinned *population* is the other. *Why the illustration failed: it assumed
> near-surface node `z` is confined to
> multiples of `cell/2`. It is not — `warp_lattice`, boundary secant-interpolation and the BCC
> odd sublattice all break that layering — and the band spans 2.3–4.6 layer-spacings at every
> level, so it never sits inside a single layer gap.*
>
> **✅ Free harness validation, unprompted:** the `cell = 0.003` row reproduces rung 1's committed
> **7 849 verts and bands 228 / 367** exactly ⇒ this measurement is on the same artifact rung 1
> measured ([[feedback_validate_new_harness_against_known_value]]). It also gives **2 257
> referenced corners** a producer for the first time — §0.1 had carried it as prose.
>
> **✅ No aliasing:** corners go 2 257 → 8 307 → 18 485 (**3.68× / 8.19×** against cell ratios
> 3.375× / 8×), strictly increasing and tracking volume scaling ⇒ all three levels are live and
> distinct, which is the precondition the ladder needed.

- **⚠ NEW, and nobody predicted it — the band asymmetry GROWS sharply under refinement.** sup/inf
  goes **1.61 → 2.84 → 3.55** (228/367, 667/1 895, 1 328/4 717): the two endplate faces resolve at
  very different rates. It is **common-mode in `k10/k4` at fixed `h`** so it does not threaten the
  deliverable, but it is a real geometric fact about this disc and it means the two faces' pinned
  populations are *not* interchangeable. Carry it into any per-face reasoning.
- **A domain gate could not have seen any of this**: retained volume, corner count and retained
  fraction are all stable while the *boundary condition* moves. The lesson stands even though the
  magnitude came out small — a gate on the property you changed misses the one you broke
  ([[feedback_gate_on_the_property_misses_conditioning]]). **The clamp-plane gate is retained**
  (§4.8 assert 2) precisely because its cheapness is what let it be settled in 0.4 s.
- **⇒ `cell` remains a *reported* refinement parameter, not the asserted one.** Commit the realized
  clamp planes and free height per level; assert the free height's constancy against the measured
  0.19 % rather than assuming it.

> **★★★ STEP 1 FALSIFIED THE STEP-0 VERDICT. THE LADDER IS BLOCKED ON THE REAL DISC.**
> Step 0 measured the clamp *depth* and found it stable, and I concluded from that "the confound
> is negligible, the ladder stands". **Step 1 falsified the conclusion, not the measurement** —
> the depth was never the moving part.
>
> **MEASURED (`rung5_step1_mesh_realization_noise_floor_fom`, 90 s, 1.77 GB peak RSS):**
>
> | `cell` | corners | tets | inf band | **sup band** | free height | Tet4 flex/ext | Tet10 flex/ext | ratio |
> |---|---|---|---|---|---|---|---|---|
> | 0.00300 | 2 257 | 7 759 | 228 | **367** | 12.3922 mm | −0.2811 / −0.2788 | −0.1873 / −0.1849 | 0.6664 / 0.6630 |
> | 0.00305 | 1 893 | 6 818 | 231 | **193** | 12.3841 mm | −0.2341 / −0.2335 | −0.1699 / −0.1687 | 0.7259 / 0.7225 |
>
> **A 1.67 % cell change moves `k4` by 16.7 %, `k10` by 9.3 %, and the RATIO by 8.9 %.**
>
> **★★★ THE MECHANISM IS THE PINNED POPULATION, NOT THE CLAMP PLANE.** Free height moved
> **0.065 %** — the boundary's *depth* is stable, exactly as step 0 said. But the **superior band
> lost 47 % of its nodes (367 → 193)**: the disc's superior surface sits near-tangent to a lattice
> layer, so a 1.67 % cell change sweeps a whole layer of nodes out of the Dirichlet set. Half the
> constraints on the top face vanish ⇒ a softer disc. **The §4.8 gate as designed would have MISSED
> this**, because it asserts the clamp *plane* and this is the pinned *population*.
> ⇒ **Gate the POPULATION, not just the plane** — and normalize it, since the raw count must grow
> under refinement.
>
> **⇒ The ladder cannot separate element convergence from a non-converging Dirichlet population.**
> Across the ladder the superior band grows *faster* than volume (367 → 1 895 → 4 717) while the
> inferior grows *slower* (228 → 667 → 1 328); sup/inf runs **1.61 → 2.84 → 3.55**. The §3
> "unpredicted but common-mode in the ratio" note is **retracted**: the ratio moved 8.9 %, so it is
> not immune. What survives of that claim is only the narrow version — *at a fixed `cell`* both arms
> share one mesh, so the per-level ratio attributes to element order. Across levels it does not.
>
> **⇒ WHAT RUNG 5 CAN STILL DELIVER, HONESTLY:** a **mesh-realization uncertainty band on the
> shipped `RUNG7_K_DISC`** — ~9 % on `k10` for a 1.7 % cell perturbation. That is a real and useful
> number: it **dwarfs rung 3's 0.05 % curving effect and is ~¼ of the whole element effect**, and it
> says the shipped `−0.1882` is a point estimate carrying far more mesh uncertainty than the arc has
> been quoting ([[project-uq-ensemble-stochastic-direction]] — report distributions, not
> false-precision points). The h-convergence claim is **BLOCKED pending a band-selection rule whose
> pinned population converges**, which is a production change and its own rung.
>
> **★★ PROCESS: the ordering worked, and cheaply.** Step 0 (0.4 s) said proceed; step 1 (90 s) said
> stop. Neither is wasted and the sequence cost 90 seconds — against a ladder that would have
> produced a plausible, publishable, *wrong* convergence table. **A gate on the property you
> changed misses the one you broke** ([[feedback_gate_on_the_property_misses_conditioning]]) fired
> here on my own gate, one turn after I wrote it.

**(2) `largest_component` moves the domain — direction UNKNOWN, and pre-registering one is
harmful.** A finer cell may resolve the disc's sub-cell-thin tapering rim better and retain more —
but selection is by **tet COUNT, not volume** (`sdf_meshed_tet_mesh.rs:282-291`), so a count-winner
need not be a volume-winner in a sliver-rich refined mesh; and connectivity is **percolation-like**,
so a coarse cell bridging a taper that a finer lattice severs makes retention **non-monotone**.
⇒ Pre-register as **"direction unknown; measure and report"**. Stating a direction would license
reading an unexpected result as a harness bug.
- **★ Volume is the WRONG metric for it.** Bending stiffness weights material by `r²` about the ML
  axis while volume does not; the dropped rim islands sit at **maximum radius**, where volume
  sensitivity is lowest and stiffness sensitivity is highest. Report **`∫r² dV` about the ML axis**
  and the retained SI extent, not volume alone.

**(3) `stuffing::warp_lattice` displaces near-boundary lattice vertices before stuffing**
(`sdf_meshed_tet_mesh.rs:150-152`), and boundary nodes are placed by **linear secant interpolation
of φ, never projected** (`stuffing.rs:290-296,366-373`) — a third h-dependent effect on the achieved
geometry. Named, not gated; it is subsumed by measuring the noise floor.

**★ WHAT THE RATIO BUYS, PRECISELY.** Both arms at one `cell` come from one prepared mesh, and
`full_face_band` widens the corner band **topologically** (`lib.rs:917-938`), so *every* confound
above is common-mode in `k10/k4` at fixed `h`. The per-level ratio is clean; the absolutes are not.
That is why the deliverable is a per-level bracket plus per-level ratios, and why no
cross-level absolute trend is asserted.

**★★ CONFOUNDS RULED OUT with code evidence — do not re-litigate** (stress-test front 3):
- **Rigid box poses** (`lib.rs:672-678`) drift with `h`, but the Dirichlet data is `x ↦ Rx` and the
  moment is taken about the origin, so the box origin cancels exactly; contact is `NullContact`.
- **`MaterialField::uniform`** is a `ConstantField` sampled per element and position-ignoring —
  bit-identical for every tet regardless of size (`material/material_field.rs:153-158,455-470`).
- **Orphan DOFs** are auto-pinned into `effective_pinned` and condensed out before assembly
  (`construct.rs:356-360,394-399,572-581`); no zero rows reach the tangent. (Reporting *referenced*
  corners is still right, for a better reason than the first draft gave.)
- **Warm-start inertia** is negligible: `M/Δt²` at `static_dt = 1e3` is `1e-6·M`.

**★ Solver facts, corrected.** The Newton tolerance is an absolute unscaled ℓ2 over free DOFs
(`newton.rs:220-227`, `tol = 1e-10`, no `/√N`) — but the first draft's inference from that was
**wrong in the direction that matters**: per-node force ~ `h²` and DOF count ~ `h⁻³`, so at fixed
*relative* accuracy `‖r‖₂ ~ h^{1/2}`, i.e. a fixed absolute tolerance is *relatively looser* on a
finer mesh (≈ √2 over the whole ladder — practically irrelevant either way). **Drop the
term-counting argument; it is not load-bearing.** What makes under-convergence impossible is that
the iteration cap **hard-fails**: `MAX_NEWTON_ITER = if N == 4 {50} else {400}` (`bonded.rs:251`)
→ `replay_step` → `solve_impl` → `NewtonIterCap` → `panic!` (`newton.rs:267-285`), traced
end-to-end. Three sibling surfaces (`ArmijoStall`, `DoublyFailedFactor`, `ValidityViolation`) panic
on the same path ⇒ **four loud failure modes, none silent.** ⚠ Also: the solve is warm-started
(`bonded.rs:367`), so iteration counts are path-dependent and comparable across levels only if the
drive path is identical.

### ▶ RUNG 5.0 — the de-risking spike, aimed at MEMORY not wall-clock

**★ Cost is not the risk; the first draft aimed the spike wrong.** Quoting the *measured* anchors
rather than re-deriving from `O(N²)` (§5.1's own instruction, which the first draft disregarded):
rung 4 measured referenced DOF 6 771 → ~41 600 (**6.14×**) at **18.1×** cost, an empirical exponent
of **1.60**, so 3.375× cells is **~7×**, not the ~11× first written.

⚠ **The per-solve figures below are DERIVED from committed aggregates, not measured** — each is a
total ÷ a count, and one of those counts is a *range*: `capture_ramp` 32.2 s over "150–175 solves"
⇒ **~0.20 s** per warm Tet4 solve (**±8 % from the range alone**); ×18.1 ⇒ **~3.6 s** per warm
Tet10 solve; Tet4 build 6.9 s less its 18 warm solves ⇒ **~3.3 s** meshing at `cell = 0.003`. On
those, the full ladder is **~4–10 min in release — smaller than the ±6° sweep this crate already
runs**, so the ladder is not cost-gated. But this is the third cost model in this document and the
first two were wrong; treat the range as an order-of-magnitude and let rung 5.0 replace it.

**The real ceiling is memory.** Tet10 at `cell = 0.002` is ~140k DOF; the solver holds **two**
symbolic factorizations for its lifetime — `SymbolicLlt` on the lower triangle *and* `SymbolicLu`
on the full reflected pattern (`construct.rs:160-191`) — and rebuilds the numeric LU every
iteration, boxed because it is "substantially larger than `Llt`" (`factor.rs:56-60`). Fill ~`N^{4/3}`
puts it plausibly at **3–8 GB before the simultaneously-live Tet4 arm.** ⚠ **OOM here is not
graceful**: `SymbolicLu::try_new(..).expect("symbolic LU factorization of free-block pattern failed")`
(`construct.rs:188-189`) panics with a message that reads as a pattern bug.

**The spike, in order:**
0. **✅ DONE — the disc's SI extent and the REALIZED band, at all three candidate cells.**
   `rung5_step0_realized_band_across_the_ladder_fom` (committed before it ran; **0.4 s**, meshing
   only, no solve). **VERDICT AS OF STEP 0: the ladder stands** — free height constant to 0.19 %,
   all three levels live and distinct, so no `band_frac` re-registration and no cell re-choice.
   ⚠⚠ **STEP 1 THEN FALSIFIED THAT VERDICT — though not this measurement.** The clamp *depth* is
   stable exactly as measured here; it was never the moving part. The pinned *population* is
   (superior band −47 % across a 1.67 % cell change), and the h-ladder is blocked on the real disc.
   **Read this row as "the depth is fine", never as "the ladder is fine."**
   Full table and the ~70× correction it forced are in §3 confound 1. ★ It also reproduced rung
   1's committed 7 849 verts / 228 / 367 for free, which is what says it is measuring the same
   artifact rung 1 did. **This step cost 0.4 s and settled the confound that reshaped the rung —
   the cheapest thing in the ladder was also the highest-value.**
1. **The mesh-realization NOISE FLOOR — nothing in the table is interpretable without it.**
   Run the coarse level twice, at `cell = 0.003` and `cell = 0.00305`: a 1.7 % resolution change
   against a 33 % signal. Anything beyond `1.7 % × dk/dh` is pure realization noise — and because
   the lattice is origin-anchored, a 1.7 % cell change can flip a node layer in or out of the band,
   so this doubles as the direct probe of confound 1. **Pre-register the floor before the ladder
   is fixed.** Context: rung 1 records the bonded moment reproducing to `< 1e-3` relative, and
   §4.1 warns that past ~23.5k DOF faer switches to supernodal kernels where a pivot flip can move
   the answer — **every rung-5 level except Tet4-at-0.003 is past that threshold.**
2. **Tet10 at `cell = 0.002`: one build, one ±0.5° solve.** Record peak RSS, wall-clock, iterations.
   **State a budget with a threshold action** — if peak RSS exceeds it, drop the Tet10 fine arm to
   `cell = 0.0025` rather than reshaping mid-FOM.
3. **Tet4 at `cell = 0.0015`: one build, one solve.** The *most refined mesh in the study*, against
   a **50**-iteration cap. Nearly free, and the first draft omitted it. (Expected to be comfortable:
   ~54k DOF, SPD tangent ⇒ `Llt` succeeds and the LU fallback never fires.)
4. **Does a refined arm survive the single ±0.5° jump?** Rung 2 measured that the *raw* Tet4 disc
   does and the *conformed* one does not — at `cell = 0.003` only. If a refined arm needs stepping,
   every solve becomes 5–10 and the cost model moves; rung 2 also records that stepping does not
   move the measurement (its raw column reproduces rung 1 to four decimals).

⚠ Nothing above says to hold every level live at once. **Drop each level's arms before building
the next**, or the ladder's peak RSS is the sum rather than the max.

### ★ FILE PLACEMENT — decided, with the cost stated (user call, 2026-07-30)

The first draft put the FOM in `tests/` claiming "zero private items, verified against the current
signatures". **That was verified only for the payoff number.** The validity metrics it also
mandated are unreachable there: `BondedDisc.sandwich` is private with no accessor (`lib.rs:318`),
`deformed_nodes_native()` returns *total* nodes including orphans (the metric this rung forbids),
no tet connectivity is exposed, and `PreparedDisc`/`duplicate` are `#[cfg(test)]`-private
(`lib.rs:483,528`) — so the "one shared mesh object" argument does not even apply from `tests/`.

**⇒ The licensed FOM lives in `src/lib.rs`'s test module**, where the domain metrics, the band
sizes and `PreparedDisc::duplicate` are all reachable. **Coverage cost accepted deliberately**, on
the same terms as rung 4's: `cf-fsu-model` already grades **C (58.2 %)** by a recorded user call,
an `#[ignore]`d licence-gated FOM charges `llvm-cov --lib` while executing in no coverage run, and
CI runs `--skip-coverage` so it gates no merge. Measure the new letter against the 58.2 % baseline
and record it — do not delete gates to protect a letter.

**⇒ Newton residual + iteration count are DROPPED as a gate.** They are unreachable from *either*
location: `BondedSandwich::resolve` discards `NewtonStep` (`bonded.rs:375,388`) and `BondStep`
carries only wrenches (`bonded.rs:88-99`), so reporting them needs new public API in **sim-coupling**
— a keystone diff to serve one FOM, which is surface nobody asked for
([[feedback_consumer_gated_completeness]]). The four hard-failing surfaces make under-convergence
loud without it, and `min_jacobian_ratio()` + `flexion_moment`'s conservation residual are both
free.

**⇒ A license-free synthetic arm carries the CONVERGENCE ASSERTION** — the first draft had this
backwards. `synthetic_disc()` is a 24×20×6 mm box: **no tapering rim, so no `largest_component`
drift and a stable domain**, it retains the same clamp-ring singularity class, it is cheap enough
to refine much further, and it is the only part of this rung **CI can run**. Let the box assert
convergence *behaviour*; let the real disc commit per-level numbers. The magnitude does not
transfer — the behaviour does, and behaviour is what this rung asks about.

**Gate:** §4.8.

---

## 4. Gates — written so each one can fail

| rung | crate(s) | payoff gate | sanity | byte-identity |
|---|---|---|---|---|
| 0 | coupling, fsu-model | — (inert) | existing tolerance tests unchanged | **§4.1 license-free `to_bits` golden, bits frozen pre-change** |
| 1 | fsu-model | **§4.2** ratio bracket + band-count cross-check | restoring + conserving + converged, both arms | Tet4 path untouched |
| 2 | fsu-model | **§4.3** exact-geometry residual ↓ | **§4.5** large-angle sweep; per-element conform delta | Tet4+raw arm byte-identical |
| 3 ✅ | sim-soft + fsu-model | **§4.3** residual ↓ again (authorised RMS 0.881 → 0.767 mm, re-anchored at rung 4) | **§4.4** coverage 67.4 % + per-Gauss-point floor; k_disc shift 0.05 % committed | straight-Tet10 arm untouched (rung-1 FOM re-runs at 0.666 / 0.663) |
| 4 ✅ | fsu-model, coupling | single `RUNG7_K_DISC` re-anchor, measured (−0.2819 → −0.1882) | **§4.5** full ramp completes, `detJ > 0` on the DEFORMED config; **§4.6** flexion ROM assert; segment shift +0.0082° vs predicted ~0.008° | — |
| 4b | fsu-model | **§4.3** residual, on the COUPLED disc | lofted disc completes ±6° conformed, both elements | — |
| 5 | fsu-model | **§4.8** the bracket `\|k*\| ≤ \|k10(fine)\| ≤ \|k10(coarse)\| ≤ \|k4\|`, ±5 % two-sided pins; headline = a lower bound on the shipped `RUNG7_K_DISC` error | liveness (strictly-monotone DOFs per arm) → rung-1 known-value reproduction → **clamp-plane constancy** → `min_jacobian_ratio` → domain metrics | zero production diff; rung-2 `llvm-cov` oracle on **`coupled.rs` + `coupling/src/bonded.rs`** (NOT `src/lib.rs` — this rung adds tests to it) |

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

> **✅ BUILT — and one half of it turned out to be tautological in a way this section did not
> anticipate.** The *element-validity value* is pinned to the floor by construction: the
> back-off bisects onto the constraint boundary, so the worst `detJ/detJ_rest` equals
> `quality_floor` exactly whenever any node backs off. Committing it two-sided would have been
> the same defect this section replaced, one level down. What ships is the **inequality over
> every element** (falsifiable against the projector's own incidence bookkeeping — see
> `worst_gauss_det_ratio`) plus the **coverage triple**. ⚠ And the tidy version of *that* claim
> — "the delivered fraction is the member with the teeth" — was itself refuted by a cold-read
> mutant: a silent 0.2 mm cap on every move takes the fraction the WRONG way (67.9 % → 68.6 %),
> because a smaller request is easier to satisfy. The fraction covers the back-off-engages-more
> direction; `max_move`/`mean_move` and the §4.3 pin cover the other. See rung 3's BUILT note.

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

### 4.8 Rung 5's gate — a bracket with two-sided pins, ordered so the cheap checks fire first

**★★ THE FAILURE THIS GATE EXISTS TO CATCH FIRST (stress-test front 1): an arm that silently does
not refine passes every payoff assert, and a naive domain gate reads it as the STRONGEST evidence
of validity.** If the fine Tet10 arm is built from unmodified `params` — one shadowed variable —
then `δ10 = 0` *exactly*: the headline becomes "Tet10 does not move at all", the per-level domain
metrics come back bit-identical (and "flat" is what a domain gate wants), and the residual and
iteration counts match perfectly. **⇒ Assert 0 is liveness, and it separates two things the first
draft conflated under one heading:**

| | must be | asserted on |
|---|---|---|
| **did the DISCRETIZATION change?** | strictly **monotone** | referenced corners, per **arm**, per level |
| **did the DOMAIN change?** | **flat** to a stated band | retained `∫r² dV`, SI extent, retained fraction |

One gate cannot want both. Assert both, separately, on both the licensed and the synthetic arms.

**Order — validity before payoff** (rung 3's ordering fix: a payoff number read off an invalid mesh
is worse than no number; its incidence-walk mutant improved the residual while inverting tets):

0. **Liveness**, per arm per level, per the table above. Cheapest, and it gates everything.
1. **Harness validation against a KNOWN value — free, and the first draft omitted it.** The coarse
   level *is* the shipped configuration (`DiscParams::default().cell = 0.003`), so the ladder's
   `h₁` must reproduce rung 1's committed raw-disc numbers: **Tet4 −0.2811 / −0.2788, Tet10
   −0.1873 / −0.1849, ratio 0.666 / 0.663**, to four decimals. This is the repo's own
   validate-a-new-harness-against-a-known-value rule ([[feedback_validate_new_harness_against_known_value]]),
   available at zero cost on the exact quantity the rung measures, and it is the cheapest possible
   check that the multi-resolution harness is not silently a different probe (wrong `theta`, wrong
   pivot, `Some` instead of `None`, an arm wired to the wrong builder).
2. **★★ THE CLAMP-PLANE GATE — ✅ ALREADY MEASURED AT RUNG 5.0 STEP 0; here it becomes a
   REGRESSION gate.** Per level, commit the realized clamp planes (max z over the inferior band,
   min z over the superior), the **free height**, and the **node layers through it**. **Assert the
   free height is constant across levels to within 0.5 %** — a two-sided pin around the *measured*
   0.19 % spread (12.392 / 12.404 / 12.381 mm), not a guessed tolerance. Commit the per-level band
   sizes alongside; note the **sup/inf asymmetry grows 1.61 → 2.84 → 3.55**, so a symmetric
   expectation on the two faces would be wrong.
   ⚠ **What this gate is FOR has changed, and saying so matters:** step 0 settled that the
   lattice-phase confound is negligible at these cells, so this is no longer the gate that decides
   whether the ladder is valid — it is the gate that catches a *future* change (a new `band_frac`,
   a different disc, a mesher change) silently re-opening it. **If it ever fires, `band_frac` is
   re-registered per level to hold the realized planes fixed** — the arc's rule that a tuning
   constant is never inherited across element orders *or input geometries*, now extended to
   *resolutions* — and that adjustment is pre-registered, not discovered mid-run.
3. **Mesh validity.** `BondedDisc::min_jacobian_ratio()` per arm per level — public, free (it reads
   the last solve), and precisely the deformed-configuration invariant rung 4 added because
   "nothing in this arc checked the deformed configuration before rung 4". A finer BCC lattice
   against the disc's sub-cell-thin tapering rim is exactly where slivers appear. Plus
   `flexion_moment`'s conservation residual `‖ΣF‖+‖ΣM‖ < 1e-8` — recorded for comparability, **not
   counted as an independent assert**: for a gravity-free quasi-static Dirichlet-only scene it
   follows from Newton convergence and cannot see a wrong band, wrong material or wrong element.
4. **Domain metrics**, per the table above, with the direction **unregistered** (§3 confound 2).
   Exceeding the band means **abstain and say so** — not widen.
5. **The payoff: the bracket**, with two-sided pins.

**★ The bracket, and its pins.** Commit `|k*| ≤ |k10(fine)| ≤ |k10(coarse)| ≤ |k4(coarse)|` and the
headline number **`|k10(coarse)| − |k10(fine)|` = a lower bound on the error in the shipped
`RUNG7_K_DISC`**. Then pin it the way every other payoff gate in this document is pinned — **±5 %
two-sided around each measured value**, added in a fixup commit once measured (§4.2 step 4, §4.3's
"#701 shape"). The first draft's "commit a convergence table" asserted nothing: committing a table
is a record, and a refactor moving `k4(0.002)` from −0.26 to −0.20 would have tripped nothing.

**★ The bracket's ordering asserts are NOT independent of the theorem** (§3): `|k10| ≤ |k4|` is
entailed by `P1 ⊂ P2`, so it is a *wiring* check — it fails only if an arm is mis-built. Say that
where the assert lives, so no summary counts it as evidence for the element.

**★★ HOW MANY PREDICTIONS THERE HONESTLY ARE — the first draft claimed four and had one and a
half.** Pred 4 ("Tet4's finest lands on Tet10's side") is strictly *entailed* by pred 1 ("`|k4|`
decreases monotonically"); pred 3 ("ratio rises") is an arithmetic consequence of preds 1 + 2. Under
a pure-noise null, "4/4 pre-registered predictions confirmed" had roughly a **1-in-8** chance of
occurring by accident — the hedges-compress-out failure in structural form, since a summary would
legitimately write "4/4". What survives, stated as the whole set:
- **The bracket ordering** — wiring, per above.
- **`|k10(coarse)| − |k10(fine)|` is small relative to `|k10| − |k4|`** — the one substantive
  statement, and it needs a **pre-registered factor and a noise floor**, not the word "much".
  Register both from rung 5.0's measured floor, before the ladder runs. Note a pure boundary-geometry
  artifact softening both arms proportionally yields exactly `δ10 = 0.665·δ4`, which a motivated
  reader could call "much less" — so the factor must sit well below that.
- **Delta shrinkage on the Tet4 sequence** — `|k4(h₂)−k4(h₃)|` normalized for the refinement ratio
  is smaller than `|k4(h₁)−k4(h₂)|`. **Genuinely independent of monotonicity** (a sequence can
  decrease monotonically with *growing* steps, i.e. diverge) and it is the actual signature of
  asymptotic convergence. Report; assert only if it clears the noise floor.
- **No cross-level absolute trend is asserted at all** — confounds 1–3 sit on the absolutes, and
  the theorem already fixes the sign. Absolutes are committed as numbers, not as a trend.

**★ CI arm — the license-free `synthetic_disc()` box carries the convergence ASSERTION** (§3): a
stable domain, the same clamp-ring singularity class, cheap enough to refine further, and the only
part of this rung CI runs. It asserts liveness, the clamp-plane constancy, `min_jacobian_ratio`, and
the bracket ordering. ⚠ Its **magnitude** is reported, never asserted against real-disc numbers — a
24×20×6 mm box at 3 mm cells is a different bending problem, exactly as §4.2 already records.

**Where the code lives:** licensed FOM in `src/lib.rs`'s test module (coverage cost accepted, §3);
synthetic arm alongside it. **Neither `assert_full_face_band` nor the `to_bits` golden is re-run**:
the band rule is topological and cell-size independent, rung 1 mutation-verified it, and §4.1
forbids literal `to_bits` on any ≳10k-DOF solve (past ~23.5k DOF faer's supernodal kernels make a
pivot flip realistic) — **use a 1e-10 relative tolerance for every real-disc number here.**

**Byte-identity:** rung 5 touches no production code, so rung 2's free `llvm-cov` oracle applies —
but scope it correctly: it holds for **`coupled.rs` and `sim/L1/coupling/src/bonded.rs`**, *not*
for `src/lib.rs`, whose counts must change because this rung adds tests to it.

**Rung 5 must not touch `RUNG7_K_DISC`'s value.** It measures the standalone disc; any re-anchor
belongs to 4b. It **must** update that constant's provenance comment
(`sim/L1/coupling/tests/fsu_coupled_contact.rs:60-88`) with the measured error bound — see §7.

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

> **⚠ MEASURED AT RUNG 4 — both factors above are wrong, and they cancelled.** The `~85 s`
> Tet4 anchor this arithmetic rests on was never measured: `capture_ramp` on the linear disc
> is **32.2 s**, 2.6× less. The per-solve element ratio is **18.1×**, not `~10×`. The
> quadratic capture is **583.1 s (9.7 min)** — below this band's floor, and close only because
> the two errors offset. The build phase, which this section did not cost at all, goes
> **6.9 s → 67.8 s** (the `k_disc` probe is now 18 warm-started Tet10 solves). Quote the
> rung-4 measurements, not this arithmetic.
>
> *(Superseded once during rung 4 itself: the first measurement — 33.0 s / 23.2× / 764.3 s /
> 85.7 s — was taken on the CONFORMED quadratic disc, before the rung split to the straight
> arm. The figures above are the shipped configuration. Both sets are real; only one describes
> what `CoupledFsu::build` does.)*

The 14–45 min band accounts for referenced DOF going 6 771 → ~41 600 and the iteration cap
rising 50 → 400. **The inherited "~60 min" figure is from `tet10_indentation_demand1.rs:492`** —
a face-contact indentation test, single-threaded, no warm start, different mesh. Not transferable
in either direction; do not quote it for this path.

That ~14 min lands on **`tools/cf-spine-studio`** (`scene.rs:333` drives `capture_ramp`), whose
capture is documented at ~85 s in three places (`coupled.rs:246`, `scene.rs:267`, `scene.rs:314`).

**★ DECISION for rung 4 — what does the studio's disc default to?** *(SETTLED at rung 4 on
the measurement below: **the quadratic disc**. See the rung-4 BUILT note in §3 and
`cf-spine-studio`'s `scene::BUILD_HINT`.)* Recommendation:
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

> **✅ BUILT — and "port it" was the trap.** Porting the *mechanism* was right; porting the
> *value* produced a valid-but-undrivable mesh (§3's rung-3 note). The risk this section names
> did not materialise as inversion — no element inverted in any shipped configuration, at either
> floor — but as a collapsed **step envelope**, which no geometric gate observes. (Inversion is
> reachable: the §4.4 mutant that breaks the projector's incidence map drives elements to
> `detJ/detJ_rest = −9.787`. The guard works; it is the *floor value* that was wrong.) `DISC_MIDSIDE_CONFORM_QUALITY_FLOOR = 0.4`.

### 5.3 The parameterization silently changes Tet4 (rung 0) — mitigated by §4.1's golden

…which does not exist today, and which v1 specified in a form that could never have run.

### 5.4 ROM does NOT break — but a hard `k_disc` assert does

The disc is ~0.4% of ROM, so a 33% softer disc cannot move the band; `ROM_TOL_DEG` and
`LIT_EXTENSION_DEG` are tripwires, not payoff gates, and if they move the *prediction* was wrong
— re-anchor honestly, never tune Tet10 to the old number
([[feedback_anti_rot_invariants_vs_exact_anchors]]). **What does break is `RUNG7_K_DISC ± 0.02`
(§0.3), by roughly 4.8× the tolerance** — an indicative magnitude, not an exact prediction
(§0.3 states why: raw-mesh ±0.5° mean vs a 0.86° probe — ⚠ **not** "on a conformed, curved disc",
the stale premise §0.3 now corrects; the shipped arm is raw and straight). That is a
planned, single, documented re-anchor at rung 4, not a surprise. **✅ Both halves confirmed by
measurement at rung 4: `k_disc` moved 4.69 × its tolerance (re-anchored −0.2819 → −0.1882),
and segment flexion ROM moved +0.0082° against a predicted ~0.008° — so `ROM_TOL_DEG` and
`LIT_EXTENSION_DEG` never had to move.**

### 5.5 Rollback — what happens if a gate comes out wrong (v1 had no clause)

- **Rung 1 ratio outside `0.60..=0.73`:** stop. The harness does not reproduce the spike. Debug
  the band first (§4.2 step 3 will usually already have fired), then the enrichment. **Do not
  widen the bracket to admit the observed value** — that is the falsified-spec case; revert
  opt-in and keep the surface ([[feedback_spec_falsified_revert_opt_in_keep_surface]]).
- **Rung 2/3 residual does not decrease:** the conform/projection is not seating. Treat as a
  geometry bug, not a gate-tuning problem.
- **Rung 5's liveness assert fires (§4.8 assert 0):** an arm did not actually refine. This is a
  harness bug, not a result — **do not read the payoff numbers at all**, because the failure mode
  it catches (`δ10 = 0` exactly) otherwise produces the arc's *ideal* headline.
- **Rung 5's clamp-plane gate fires (free height outside the measured 0.5 % pin):** ⚠ **this is
  now a REGRESSION, not a discovery** — step 0 measured the spread at 0.19 % on the shipped
  `band_frac` and disc, so a firing means something changed underneath (a new `band_frac`, a
  different input mesh, a mesher change) and re-opened the lattice-phase confound. Re-register
  `band_frac` per level to hold the realized planes fixed, then re-run — **do not** proceed and
  caveat it in prose. The per-level ratio stays usable meanwhile (common-mode); the absolutes do
  not.
- **Rung 5's coarse level does not reproduce rung 1's committed numbers (§4.8 assert 1):** stop.
  The multi-resolution harness is a different probe from the one that produced `RUNG7_K_DISC`, and
  nothing downstream of it means anything.
- **The bracket ordering fails (`|k10| > |k4|` at some level):** this contradicts a *theorem*
  (§3's `P1 ⊂ P2` nesting), so it is a wiring bug — an arm mis-built, the wrong `endplates`, or a
  band that dropped midsides. Debug the build; do not report it as an element finding.
- **`|k10(coarse)| − |k10(fine)|` does not clear the measured noise floor:** the honest result is
  **"the shipped Tet10 error is below what this harness can resolve"** — which is a *useful*
  answer and must be reported as one, not padded into a convergence narrative. It does not falsify
  the migration: §3's theorem still gives `|k10| ≤ |k4|`. Narrow §0.1 bound 3 and rung 1's doc
  comment to say the error is *bounded but unresolved*, both — the twins.
- **The Tet4 sequence is non-monotone:** **expected, not a failure.** The meshes are re-phased and
  non-nested (§3 confounds 1–3), so the monotonicity that nested spaces would guarantee does not
  apply. Report it; it is evidence about the confounds, not about the element.
- **Tet10 at `cell = 0.002` exceeds the RSS budget (rung 5.0 step 2):** drop that arm to
  `cell = 0.0025` — **do not** drop the Tet10 refinement entirely, since the fine Tet10 point *is*
  the bracket's tightest bound and the rung's whole deliverable.

---

## 6. Deferred / open questions (named, not silent)

- **Differentiable Tet10 bond** (`probe_with_pose_gradient` at N=10) — type-restricted to Tet4
  now (§2.3); FD-gate when a co-design consumer needs it. **Verified not a regression:** no
  production caller exists (`cf-codesign` consumes `StaggeredCoupling`).
- ~~**Studio disc element default**~~ — **DECIDED at rung 4 on the measured cost: the quadratic
  disc**, so the Studio simulates what the SDK ships. `k_disc` is read off the *render* disc, so
  a linear Studio would silently give the picture different disc physics from the library's.
  The `CAPTURE_SUBSTEP` N-awareness lever is named but deliberately unpulled — a conditioning
  change needs its own measured envelope (rung 3), not a plausible argument.
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
- **★ Is the converged bonded tangent actually PD? (rung 5's one loose premise.)** §3's energy
  argument — the whole reason "Tet10 is more accurate here" is a theorem rather than a
  measurement — needs the solver on a **minimiser**, not a saddle. Today the `Llt` → `Lu`
  fallback fires on most iterations (`factor.rs:344,369,383`), but it is triggered by
  `LltError::Numeric`, which faer raises on *any* non-positive pivot — indefinite,
  semi-definite, or merely rank-deficient alike — so its firing is **not** evidence of a saddle.
  The check is one line: report whether `Llt` succeeds at the **converged** state (or the LM
  bump needed to make it). If it is PD there, the accuracy claim is proven for free. Deferred
  because it needs a `sim-coupling` accessor and rung 5 is zero-production-diff (§3); the
  theorem is stated *with* this premise named rather than assumed.
- **★ A genuinely two-sided bracket on `k*` needs an INDEPENDENT solver.** Rung 5 delivers
  `|k*| ≤ |k10(fine)| ≤ …` — an upper bound and a measured error *floor*, not a two-sided
  bracket, because every arm shares this repo's element, mesher, and bond. The only honest route
  to the other side is a second FEM on the same STL (CalculiX / FEniCS via `uv`) — the one
  cross-check that does not route through the artifact under test
  ([[feedback_cross_check_must_not_route_through_the_artifact_under_test]]). Expensive, and named
  here as the deferral rather than letting an extrapolation stand in for it.

---

## 7. Doc-drift checklist (a rung deliverable, not an afterthought)

[[feedback_no_rationalizing_doc_drift]]. **✅ ALL DISCHARGED at rung 4** — struck through
below, with one addition the checklist had missed. Rung 4 invalidated:

- ~~`sim/L1/fsu-model/src/coupled.rs:135-186`~~ — **done**: the ~10-line block stating *"The BONDED FEM disc
  here stays on the RAW (un-conformed) geometry (`None`)"*, including the claim *"flipping it
  would shift the `k_disc` baked into the rung-7-validated ROM equilibrium"* — **already
  inaccurate today**, since rung 7 does not use `CoupledFsu` (§4.6). ⚠ **Rung 2 discharged two
  parts of this early**, because leaving them would have been drift rather than deferral: the
  "~4 %" shift is measured at ~1.8 %, and the *"a whole-face-conformed mesh spawns sliver tets
  that fail the sweep"* rationale is a Strategy-A fact that the ±6.0° Strategy-B sweep retires.
  What is left for rung 4 is the `None` → conformed-Tet10 flip itself and the `RUNG7_K_DISC`
  re-anchor.
- ~~The "~85 s" / "~5 s" capture figures~~ — **done**, and the checklist **undercounted the
  blast radius by 3×**: the two figures were written out at ~24 sites across `cf-fsu-model` and
  `cf-spine-studio`, **four of them UI strings a user reads while waiting** (a key hint and two
  spinner captions) — user-visible drift, not just prose. They now derive from two measured
  constants, `cf-spine-studio`'s `scene::BUILD_HINT` / `CAPTURE_HINT`, so the next cost change
  moves one place. ⚠ Both figures were also *wrong before the flip*: the linear capture is 32.2 s,
  not 85 s (see §5.1).
- ~~Module docs: `sim/L1/fsu-model/src/lib.rs:8, 30, 35`; `tools/cf-spine-studio/src/main.rs:8, 21, 30`~~ — **done**.
- **★ Addition this checklist missed** — `coupled.rs`'s module doc claimed the bushing is
  linearised because *"the disc FEM only converges sub-degree"*. That was already false when it
  was written (the limit is on the **step size**, not the angle reached — `capture_ramp` walks
  the full ±ROM), and rung 4 makes it conspicuous. Corrected: the linearisation is what keeps the
  equilibrium bisection cheap, and the bisection never touches the FEM at all.
- ~~`sim/L1/fsu-model/src/lib.rs:1136`'s "±0.5° spike-validated conformed SPD range" scope
  note~~ — **done at rung 2**, along with the module-level "the quadratic arm's large-angle
  envelope is unmeasured" note, both superseded by §4.5's measured ±6.0°.

### 7.1 Rung 5's obligations — OPEN (added 2026-07-30 with the rung-5 spec)

⚠ **The list above was "ALL DISCHARGED at rung 4"; rung 5 re-opens it.** Two of these were missed
by the first draft, which wrote a narrowing instruction for the *failure* path only — so a
**successful** rung 5 would have left both twins still asserting the claim is un-earned:
- **`sim/L1/fsu-model/src/lib.rs:2857-2859`** — rung 1's FOM doc comment ("−0.1861 is not proven to
  be the converged truth … Earning the accuracy claim is rung 5's h-refinement"). Rung 5 must
  rewrite this **on either outcome**: the accuracy claim is a *theorem* (§3), and what rung 5 adds
  is the measured **error bound**, not the claim.
- **§0.1 bound 3** (this document) — same asymmetry, same fix. Bound 3's "a refined Tet4 would also
  soften" is right; its implication that the accuracy claim awaits h-refinement is not.
- **`sim/L1/coupling/tests/fsu_coupled_contact.rs:60-88`** — `RUNG7_K_DISC`'s provenance block.
  §4.8 forbids touching the constant's **value**; the comment must still gain the measured error
  bound, or the checkpoint's "rung 5 hardens the layer underneath 4b" is a claim with no trace at
  the artifact it hardens.
- **The checkpoint block (§ top) and §4's table row 5** — update on landing.
- **Cross-doc, previously unlinked:** `docs/SIM_SOFT_ROADMAP.md:80` and `:140` still list
  "❌ Convergence study (mesh-refinement sanity check) — Do finer cells give similar answers?
  Credibility gate" as open. Rung 5 is that study **for the disc only**; say so in both places
  rather than letting the roadmap read as though it were still untouched, and do not let it read
  as though the whole engine were covered.
- **Anything rung 5.0 measures that contradicts §5.1** — its cost model has now been wrong twice
  (once at rung 4, once in the rung-5 first draft, which re-derived from theory in a section whose
  own instruction is to quote the measurements). Fix §5.1 in place rather than adding a third
  estimate beside it.

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

### 8.3 Rung 5's spec — the 4-front stress-test (2026-07-30)

Four independent read-only adversarial fronts against the first rung-5 draft: **gate falsifiability
· every code claim vs the tree · the numerics argument · cost/omissions/internal consistency.**
Read-only by design — a prior round of this arc established that parallel *mutating* reviewers
contaminate each other ([[feedback_parallel_mutating_coldreads_need_isolation]]). It reshaped the
rung's **question**, not just its gates. Recorded so the same errors are not re-introduced:

- **★★★ 4 of 4 fronts independently found the confound the draft did NOT name** — the
  lattice-quantized clamp plane (§3 confound 1). The draft named `largest_component` and missed a
  larger, non-monotone effect sitting on the same predictions. Convergence of four independent
  fronts is why it is now the gate with teeth (§4.8 assert 2).
- **★★★ The rung's premise was wrong.** "Is Tet10 more accurate here" is provable a priori from
  the way `k_disc` is computed (§3's reframe) — the draft built a ladder to earn a claim already
  earned, and staked it on an h-insensitivity heuristic that the clamp-ring singularity makes
  unreliable (both elements can share a fractional order there). ⇒ bracket, not extrapolation.
- **Richardson could not be computed on the draft's own ladder** — `{0.003, 0.002, 0.0015}` has
  successive ratios **1.5 and 1.333**, and the quoted three-point formula requires a *fixed* `r`.
  The draft's own "h ratios 1 : 0.667 : 0.5" is what hid it (ratios to `h₁`, not successive).
  A meaningless `p` would have landed inside its own `[0, 4]` acceptance window.
- **The draft's file placement made its own asserts uncomputable.** "Verified against the current
  signatures" had been verified **only for the payoff number**; the validity and telemetry metrics
  it also mandated are unreachable from `tests/`, and its "one shared mesh via
  `PreparedDisc::duplicate`" argument does not apply there at all (`#[cfg(test)]`-private). ⇒ §3's
  placement decision, and the telemetry gate dropped rather than taking a keystone diff.
- **"Four pre-registered predictions" was one and a half** — pred 4 ⊂ pred 1, pred 3 ⊂ preds 1+2.
  Under a pure-noise null, "4/4 confirmed" had ≈ 1-in-8 odds of occurring by accident, and a
  summary would legitimately have written "4/4". → §4.8's honest count.
- **A number with no producer, in a doc about numbers with no producers:** the draft cited "rung 3
  lifted the grade 63.9 → 65.3 %". `git log --all -S"65.3"` matches **only that commit**; 63.9 % is
  *rung 2's* post-move figure, produced by moving gates **out** of the lib target, and the current
  baseline is **58.2 %**. It also re-narrated rung 4's *deliberate, recorded, user-approved*
  coverage trade as an oversight. → [[feedback_a_number_without_a_producer_is_not_a_measurement]]
- **Two hedges compressed out:** "the LU fallback fires **every** iteration" (the shipped const doc
  says "**most**"; "every" traces to the reverted spike) and "conform ratio 0.982 / **0.984**"
  (committed asserts say 0.982/0.982 and 0.985/0.985; 0.984 appears nowhere in the crate — a stale
  pre-rung-3 figure the draft inherited from §0.1 and made load-bearing).
  → [[feedback_hedges_compress_out_in_summaries]]
- **The cost model was re-derived from theory in the section that says not to.** §5.1's own lesson
  is "quote the rung-4 measurements, not this arithmetic"; the draft wrote `3.375² ≈ 11×` anyway
  against a *measured* exponent of 1.60 (≈ 7×). Corrected, the ladder is **~4–10 min in release**
  and was never cost-gated — **memory** is the ceiling, and the draft's spike measured the wrong
  arm. → §3's rung 5.0.
- **Two fronts DISAGREED on the Newton tolerance, and resolving it deleted the argument.** One said
  refinement tightens the criterion, one said it loosens it. Per-node force ~ `h²` against DOF
  count ~ `h⁻³` gives `‖r‖₂ ~ h^{1/2}` at fixed relative accuracy — *relatively looser*, ≈ √2 over
  the whole ladder, i.e. negligible either way. The draft's stated reason was backwards and its
  conclusion was right for a different reason (the cap hard-fails). **Where reviewers disagree,
  check whether the claim is load-bearing at all before picking a winner.**
- **Four confounds RULED OUT with code evidence** (box poses cancel, `MaterialField::uniform` is
  position-independent, orphan DOFs are condensed pre-assembly, warm-start inertia is `1e-6·M`) —
  gates that no longer need writing. → §3.
- Plus: the liveness hole (§4.8 assert 0), the free rung-1 known-value reproduction the draft
  omitted, `min_jacobian_ratio()` being public and free, `∫r² dV` over volume for a rim confound,
  `largest_component`'s direction being **unknown** (tet-count selection, percolation) rather than
  the direction the draft asserted, the noise floor needing to be measured **first**, and the
  synthetic/licensed arms being the wrong way round.

**★★ THE REVIEW PASS ON THE RESHAPE ITSELF (same day) — the rewrite introduced two of its own.**
The reshape adopted the fronts' `file:line` citations wholesale into an authoritative doc *before*
checking them — the exact failure the arc keeps catching, committed inside the commit that
documents it ([[feedback_meta_audit_introduced_claims]]). Re-verified afterwards, and all of these
**hold**: world-origin lattice anchoring (`lattice.rs:257-262,296-300`), `largest_component`
selecting by **tet count** (`max_by(count_a.cmp(count_b))`, `sdf_meshed_tet_mesh.rs:282-291`), the
Stroud rule's *"degree-of-precision 2 … a curved/isoparametric Tet10 would not be exact"*
(`tet10.rs:41-53`, verbatim), both symbolic factorizations held for the solver's lifetime
(`construct.rs` returns `(SymbolicLlt, SymbolicLu)`), orphan referencing (`construct.rs:354-362`),
and rung 1's *"reproduces to < 1e-3 relative across captures"* (`lib.rs:2962-2963`). The
1.60 exponent is genuinely derivable: 41 600/6 771 = 6.145, log(18.1)/log(6.145) = 1.595. **Two did
NOT hold as written:**
- **The disc's SI extent is committed NOWHERE**, so the clamp-quantization *magnitude* (~1.8 mm
  band, 1.5/1.0/1.5 depths, ±13 %) was an inherited assumption stated as fact. Mechanism verified,
  magnitude demoted to an illustration, and measuring it became **rung 5.0 step 0**.
  **✅ THE ILLUSTRATION WAS THEN FALSIFIED BY ~70× ON FIRST RUN** — the disc is **19.32 mm**, so
  the band (3.48 mm) is *larger* than the coarse cell rather than smaller, and the free height is
  constant to **0.19 %**, not ±13 % (§3 confound 1). **★ Note which way the error ran: four
  independent adversarial fronts converged on a confound, and their agreement was about the
  MECHANISM — which held — while the magnitude every one of them carried was inherited from a
  single unmeasured assumption and was wrong by two orders of magnitude.** Consensus among
  reviewers is not evidence about a number; only a producer is.
- **"Measured per-solve times" were DERIVED** — aggregates ÷ counts, one count being the range
  "150–175 solves" (±8 % before compounding). Relabelled.
**⇒ The lesson generalizes past this rung: adopting a reviewer's citation is authoring it.** Four
independent fronts agreeing raises the odds a claim is true; it does not make it *verified*, and a
plan doc is where unverified claims become load-bearing.

Prior arcs: [[project-tet10-fbar-element-upgrade]] (element ladder),
[[project-fsu-disc-endplate-conform]] (#701 conform machinery),
[[project-keystone-soft-rigid-coupling]] (BondedSandwich keystone). Q0 spike (reverted):
built `build_bonded_disc`'s disc pipeline, ran the node-based bonded probe through Tet4 /
straight-Tet10-corners-only / straight-Tet10-full-face-tie arms; validated Tet4 vs the
shipped −0.28. Q1 read: `bonded.rs`, `fsu-model/src/{lib,coupled}.rs`, the generic
`CpuNewtonSolver<E,Msh,C,M,N,G>`.
