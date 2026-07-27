# FSU Tet4 → Tet10 coupling-migration plan

Make the whole Tet10 element ladder (`docs/SIM_SOFT_TET10_PLAN.md`, rungs 1–8 + curved
element + SDF-projection mesher, all merged #680–#700) **load-bearing in the flagship
FSU segment**: a curved-isoparametric Tet10 intervertebral disc, bonded to the *real*
curved vertebral endplate.

This is the payoff of the element ladder and the endgame of *exact-geometry-IS-exact-
physics* for the FSU: today the disc physics runs on flat-faced constant-strain Tet4,
which **bending-locks** (over-stiff in flexion); this arc puts a genuinely-curved
higher-order element on the genuinely-curved bone.

> **★ Justification (settled with the head engineer after the diamond-review, see §0.3):**
> the win is **disc-physics correctness + exact geometry**, NOT a segment-ROM improvement.
> The Tet10 disc is ~33% softer (more correct) in its own bending stiffness, but the disc
> is only ~0.4% of the segment's restoring moment at ROM, so the *segment* moment–rotation
> barely moves. We proceed on the correctness/principle grounds (and because a Tet10 disc
> is load-bearing for future co-design gradients through the disc), gating on the
> **standalone k_disc FOM**, with ROM as a promoted-to-assert sanity check.

> **Checkpoint:** `main` @ `3287d750` (after #701), clean, no live branch. This is a
> **plan**, hardened by a 5-front diamond-review (§7). Next action: commit this plan as its
> own PR, then rung 0.

---

## 0. Recon results — the two gating questions, plus the reframe the review forced

Recon: a code read (Q1) + a measurement spike (Q0, built/run/reverted — twice, the second
time correcting a bond confound the diamond-review caught). All numbers below are the record.

### 0.1 Q0 — is the element-order payoff real? **YES, ~33% (corrected from a first-pass 43%).**

A spike meshed the real disc once and ran an *identical* node-based bonded flexion probe
(pin both endplate bands, rotate the superior band by θ = ±0.5°, read the reaction moment)
through arms differing only in the element. **Three arms, one shared 7849-corner mesh,
`DiscParams::default`, all fully converged to the solver tol (1e-10):**

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
  the number; do not quote 0.570 as the element effect.** (This confound was caught by two
  independent diamond-review agents — see §7.)
- Converged genuinely (full-face flex residual 2.8e-13 @ 10 Newton iters; ext 4.3e-11;
  flex/ext symmetry 1.013 = a real restoring equilibrium). The indefinite-tangent LU
  fallback fires every iteration (benign, [[simsoft-nonpd-lu-fallback]]) but Newton reaches
  tol; pinning the face midsides *improved* conditioning vs corners-only.

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
   this bending mode ~1/3 vs Tet4, and Tet10 is the higher-order/more-accurate element.** A
   mesh-convergence study (optional, §6) is what would let us *claim accuracy*, not just delta.

**Spike operational facts (carry into the rungs):**
- **★ Band on the SURFACE-AABB z-range, not the tet-mesh z-range.** `build_bonded_disc`
  bands off the scaled input-surface AABB (`lib.rs:256/293`); banding off tet-mesh positions
  gives wrong, ~2.4× softer thresholds. (Spike footgun; the migration must band on the surface.)
- Mesh: Tet4 7849 nodes → Tet10 19449 (2257 corners + ~17k midsides). **Full-face bonded
  bands: inferior 228→1005, superior 367→1598** (corners + the face midsides between two
  bonded corners). The first-pass "228/367 identical on both meshes" was the confound
  (corners-only) — the correct Tet10 bond has strictly more nodes.
- Cost: full 3-arm spike ~3–4 min release; Tet10 solves dominate (~10× Tet4, indefinite LU).

### 0.2 Q1 — is `BondedSandwich` generic over the element? **CLEAN — Tet4 is a type-alias parameter, not baked math.** (code read, diamond-verified)

The bond math in `sim/L1/coupling/src/bonded.rs` is **entirely node-based** (`Bond` = vertex-
id sets + body-frame rest offsets; `resolve` writes posed Dirichlet targets, calls
`replay_step`, reads `nodal_reaction_forces`, reduces to a wrench). The **only** three Tet4
sites: the `use Tet4` import, `type DiscSolver<Msh> = CpuNewtonSolver<Tet4, Msh, NullContact>`
(`bonded.rs:165`), and the `CpuNewtonSolver::new(Tet4, ..)` call. The solver is already
generic (`impl<E,Msh,C,M,const N,const G>`), and the Tet10 bonded **forward** path
(`replay_step`, `nodal_reaction_forces`) has **no `N==4` assert that fires** (only the
`fbar && N!=4` guard, and the disc has `fbar:false`). ⇒ **mechanical parameterization, NOT
a keystone rewrite.** Two diamond-review fixes folded in:
- **Parameter order must be `Msh`-first.** `BondedSandwich<E, Msh, ..>` (E first) would
  *source-break* 5 explicit `BondedSandwich<SdfMeshedTetMesh>` sites (prod field
  `fsu-model/lib.rs:133` + 4 in `rung6c_disc_geometry.rs`). `<Msh, E, N, G>` keeps
  `<SdfMeshedTetMesh>` binding `Msh` → byte-identical (§2.1).
- **The differentiable `probe_with_pose_gradient` is UNGATED for Tet10.** Its VJP
  (`equilibrium_dirichlet_reaction_vjp`) passes the frictionless guard (`mu==0`) for any N,
  so a Tet10 pose gradient would run silently. The underlying channel *is* FD-gated
  (`tet10_dirichlet_reaction_sensitivity.rs`) but the *composed* bonded pose-gradient is
  Tet4-only-gated → **fail-loud `N==4` guard on that method** until a co-design consumer
  needs it (§2.3). Forward path (what ROM validation needs) is fully generic.

### 0.3 The reframe — the disc is ~0.4% of segment ROM (diamond-review, head-engineer call)

The 33% k_disc softening is large **in the disc's own FOM**, but the assembled `CoupledFsu`
bakes the disc as a hinge spring that is **~0.4% of the flexion restoring moment at ROM**
(the segment is ligament-k-dominated: at ~6° flexion, `M_disc ≈ 0.28·0.105 ≈ 0.029 N·m`
against the `|M| = 7.5 N·m` band). A 33% softer disc shifts segment ROM by **~0.008°** —
invisible. And `rung7_fsu_validation.rs`'s ROM-band verdict is a `println!` in an
`#[ignore]`d, non-CI test — **not an enforced gate today.**

**Consequence (settled): the arc's payoff is DISC-PHYSICS CORRECTNESS + exact geometry, not
a segment-ROM swing.** We proceed (Tet10 disc is the correct element + is load-bearing for
future co-design gradients through the disc) and **gate on the standalone k_disc FOM** (a
real hard assert), while promoting the ROM-band check to a real assert and running it as a
*sanity* check ("nothing broke"), not the payoff metric.

---

## 1. What the migration touches — recon sizing

- **`sim/L0/soft`** — element/enrich/Tet10Mesh/`with_sdf_projected_boundary`/curved element
  all merged. A Tet10 **bonded-band midside projection** helper may be added (rung 2b — the
  anatomy-gated quality-floor analogue of #701 `with_projected_nodes` / #700
  `with_sdf_projected_boundary`).
- **`sim/L1/coupling`** (`bonded.rs`) — parameterize `BondedSandwich<Msh>` → `<Msh, E, N, G>`
  (Msh-first), `Tet4` default. Node-based methods unchanged. Fail-loud guard on the Tet10
  pose-gradient method (§2.3).
- **`sim/L1/fsu-model`** (`lib.rs`, `coupled.rs`) — `BondedDisc`/`build_bonded_disc` gain the
  element choice; `CoupledFsu::build` flips `render_disc` to Strategy-B (rung 1) then to
  Tet10 (rung 2). New band logic to pin bonded-face midsides (§2.2).

No downstream public-enum break; the element type param is additive with a default.

---

## 2. Architecture decisions

### 2.1 Parameterize, `Msh`-first (byte-identical Tet4 default)

`BondedSandwich<Msh = HandBuiltTetMesh>` → `BondedSandwich<Msh = HandBuiltTetMesh, E = Tet4, const N: usize = 4, const G: usize = 1>`.
`DiscSolver` alias re-expressed via `CpuTet4NHSolver`/`CpuTet10NHSolver` (or the full
`CpuNewtonSolver<E, Msh, NullContact, NeoHookean, N, G>`) to avoid threading the material
param by hand. `Msh`-first keeps every explicit `<SdfMeshedTetMesh>` annotation binding
`Msh` (Q1) → the Tet4 monomorphization is unchanged. `MAX_NEWTON_ITER` becomes N-aware
(`if N==4 {50} else {~400}`) — Tet4 keeps 50 (a cap, not a byte-identity risk either way,
but gated for cleanliness). `BondedDisc` (fsu-model) gains the element param (lean: generic
`BondedDisc<E, Msh>` over a parallel constructor — matches the parameterize-over-fork norm;
decide at rung 2). *Rung-0 gate proves Tet4 byte-identity (§4).*

### 2.2 The full-face bonded band — NEW logic to pin bonded-face midsides

**This is not "no new band logic" (the first-pass plan was wrong here).** For a physically-
correct Tet10 bond the whole endplate-facing surface follows the endplate, so the bonded set
= band corners **plus every midside whose two parent corners are both in the corner band**
(a midside between two bonded corners lies on the bonded face). Recipe:

```rust
// corner_band: HashSet<VertexId> banded on the SURFACE-AABB z-range, orphan-filtered.
// For each tet: c = tet_vertices(t); m = tet_midside_nodes(t).unwrap();
// for i in 0..6 { let (a,b) = TET10_EDGE_NODES[i];        // = [(0,1),(1,2),(0,2),(0,3),(1,3),(2,3)]
//     if corner_band.contains(c[a]) && corner_band.contains(c[b]) { pin m[i]; } }
```

Spike-validated: bands grow inf 228→1005, sup 367→1598, and the reaction/moment readout
must then sum over the **full** (corner+midside) superior set (midsides now carry reaction).
For a *straight* Tet10 (rung 2a) these midsides sit at edge midpoints. For a *curved* disc
(rung 2b) the bonded-face **boundary** midsides get projected onto the endplate oracle
(interior-band midsides stay at edge midpoints — projecting interior midsides was the exact
#699 false-degradation bug); reuse `boundary_faces6` selection (#700) + SI-alignment +
quality-floor back-off (#701). The overhanging annular rim stays straight (the #701 settled
call — Sharpey's fibers attach to the ring apophysis, not the endplate face).

### 2.3 Differentiable bond — forward-generic, fail-loud on the Tet10 adjoint

Keep the forward path fully generic. Add a fail-loud `N==4` guard on
`probe_with_pose_gradient` (its composed VJP is Tet4-only-gated; §0.2). FD-gate it for N=10
only when a co-design consumer needs a Tet10 disc pose gradient. Out of ROM-validation scope.

### 2.4 Newton budget + LU fallback are expected

The Tet10 bonded solve is indefinite-tangent-heavy (LU fallback most iterations —
[[simsoft-nonpd-lu-fallback]], benign, converging at the shipped resolution; the bending-
locking gate corroborates convergence-at-high-iteration for near-incompressible Tet10).
Scale `MAX_NEWTON_ITER` for N>4; document on the const. The plan does not chase PD-ness.

### 2.5 Enrich-AFTER-conform ordering invariant

For rung 2a/2b: conform the **corner** band first (`with_projected_nodes` on the Tet4
`SdfMeshedTetMesh`, `lib.rs:338`), **then** enrich to Tet10 (midsides derive from the moved
corners), **then** (rung 2b) project boundary midsides. Enriching before conforming would
leave placed midsides behind the corner moves → detached/warped bonded-face elements. State
as a rung-2 implementation invariant + a post-enrich rest-Jacobian guard (no degenerate/
inverted element at rest).

---

## 3. The ladder — build order (one variable at a time; gate on the k_disc FOM)

Each rung: recon-confirm → commit → gating cold-reads → fixes-on-top → ASK-push → CI → merge.

- **Rung 0 — parameterize `BondedSandwich` (Msh-first) + fail-loud Tet10 pose-gradient guard
  (forward-inert).** `sim-coupling` only. **Gate:** existing tolerance tests unchanged **+ a
  new committed `to_bits` golden on the Tet4 bonded k_disc** (there is no byte-identity golden
  on this path today — add one so the parameterization genuinely can't hide a Tet4 regression).

- **Rung 1 — `CoupledFsu` → Strategy-B endplate conform on Tet4.** `fsu-model`. Flip
  `CoupledFsu::build`'s `render_disc` from `..None` to `Some(EndplateConform{..})`
  (`coupled.rs:187`; machinery exists, #701). Only k_disc changes — pivot/axis/ligament sites
  are provably conform-independent (diamond-verified). **Gate:** the committed
  `disc_endplate_conform_moment_rotation_fom` (k_disc ~4% ext / small flex shift) **+ promote
  `report_rom`'s band membership to a real `assert`** and run it as a sanity check.

- **Rung 2a — Tet4 → *straight* Tet10 disc + full-face bond.** `fsu-model` builds a Tet10
  `BondedDisc` (enrich-after-conform §2.5; full-face band §2.2). **Gate (the payoff gate): a
  committed k_disc FOM measuring Tet4 vs straight-Tet10 on the SAME conformed mesh, asserting
  ratio ≈ 0.665** (reconciled baseline — both arms on rung-1's conformed disc, not the raw
  spike baseline; re-measure the ratio on the conformed mesh, expect ≈ 0.665±band). Both arms
  restoring + conserving + converged. ROM sanity assert re-run (expected in-band; ~0.01° shift).

- **Rung 2b — curved Tet10: project bonded-face boundary midsides onto the real endplate.**
  The exact-geometry endgame. `sim-soft` Tet10 boundary-midside projection helper + `fsu-model`
  wiring. **Gate:** boundary midsides seated (non-vacuity, committed max-move); 0 inverted /
  0 sliver (rest-Jacobian guard); k_disc additive shift measured + committed; solve sound;
  ROM sanity assert. (Rung-2b viz note: `soft_boundary_faces` is corner-only triangles → the
  *rendered* Tet10 surface is straight-edged; FOM unaffected, a separate viz follow-on.)

- **Rung 3 — final k_disc FOM + ROM verdict (curved Tet10 on the real bone).** May fold into
  2b's gate rather than a separate PR. Deliverable: the disc's moment–rotation on a curved
  Tet10 disc bonded to the real endplate (committed), + the ROM sanity verdict re-established.

**2a/2b split confirmed** (user): element order (2a) and curved surface (2b) are separate
attributable FOM shifts.

---

## 4. Gates per rung

| rung | crate(s) | payoff gate (k_disc FOM) | sanity | byte-identity |
|---|---|---|---|---|
| 0 | coupling | — (inert) | existing tolerance tests unchanged | **new Tet4 `to_bits` golden** |
| 1 | fsu-model | committed conform k_disc FOM (~4% ext) | ROM band **assert** (promoted) | Tet4 (element unchanged) |
| 2a | fsu-model | **Tet4-vs-Tet10 ratio ≈ 0.665 on conformed mesh** | ROM band assert; restoring+converged | Tet4 path untouched |
| 2b | sim-soft + fsu-model | k_disc additive shift committed | ROM assert; 0 inverted/sliver | affine/Tet4 byte-identical |
| 3 | (coupling test) | curved-Tet10 disc moment–rotation committed | ROM verdict | — |

All k_disc FOM gates are committed + re-runnable (env-gated on the BodyParts3D triad).

---

## 5. Risks the build must manage

1. **Newton convergence / cost is the real #1 risk** (not ROM). Tet10 bonded solve is
   indefinite + ~10× per-solve; `capture_ramp` drives many solves (the 6d Tet10 indentation
   was ~60 min). Mitigate: warm-started sub-degree steps (existing), N-aware `max_newton_iter`,
   `#[ignore]` the heavy FOM probes as one-shot, budget CI.
2. **Curved-midside inversion (rung 2b)** — the #699/#700 failure mode. Mitigate: boundary-
   faces-only projection + quality-floor back-off + rest-Jacobian guard (all built + proven).
3. **The parameterization silently changes Tet4 (rung 0)** — mitigate with the new `to_bits`
   golden (there is none today).
4. **ROM does NOT break** (corrected from the first-pass "most likely risk"): the disc is
   ~0.4% of ROM, so a 33% softer disc cannot move the band. The ROM assert is a sanity
   tripwire ("nothing else broke"), not a payoff gate. If it *did* move (it won't from the
   disc), the response is to re-anchor honestly, never tune Tet10 to the old number
   ([[feedback_anti_rot_invariants_vs_exact_anchors]]).

---

## 6. Deferred / open questions (named, not silent)

- **Differentiable Tet10 bond** (`probe_with_pose_gradient` at N=10) — fail-loud guard now;
  FD-gate when a co-design consumer needs the Tet10 disc pose gradient (§2.3).
- **Mesh convergence of k_disc** — is −0.186 the converged truth, or does refined Tet4 reach
  it? Not a blocker (§0.1 bound 3); a Tet10 h-refinement check would let us *claim* accuracy.
- **`BondedDisc` generic vs. parallel constructor** (§2.1) — decide at rung 2.
- **Curved-element quadrature budget** — 4-pt kept; the #699 rung measured K-error grows
  ~0.22·(sagitta/edge) *(inherited from the curved-element arc, not re-measured here)*; a
  bonded-face boundary element bows ~3 edges, adequate for a well-resolved disc surface.

---

## 7. Provenance — the diamond-review (5 fronts)

This plan was hardened by a 5-front adversarial diamond-review (1 isolated-worktree Q0
re-measurement + 4 read-only audits). Material catches folded in above:
- **Q0 bond confound (2 independent agents):** the first spike pinned corners only → free
  bonded-face midsides → the 43% was inflated. Corrected full-face measurement = **~33%
  (0.665)**. → §0.1, §2.2.
- **ROM reframe (sequencing agent):** disc is ~0.4% of ROM; the ROM "gate" is a `println!`
  in an `#[ignore]`d test. → §0.3, §3, §4, §5.
- **Q1 param-order source-break + VJP guard scoping (keystone agent).** → §0.2, §2.1, §2.3.
- **Enrich-after-conform ordering + rung-2a baseline reconciliation + no-Tet4-golden
  (sequencing agent).** → §2.5, §3, §4.
- **Honesty pass:** softened fact-vs-expectation claims (byte-identical/no-break/benign →
  scoped or gate-verified); tagged inherited prior-arc numbers; corrected the "228/367
  identical bands" inference. → throughout.

Prior arcs: [[project-tet10-fbar-element-upgrade]] (element ladder),
[[project-fsu-disc-endplate-conform]] (#701 conform machinery),
[[project-keystone-soft-rigid-coupling]] (BondedSandwich keystone). Q0 spike (reverted):
built `build_bonded_disc`'s disc pipeline, ran the node-based bonded probe through Tet4 /
straight-Tet10-corners-only / straight-Tet10-full-face-tie arms; validated Tet4 vs the
shipped −0.28. Q1 read: `bonded.rs`, `fsu-model/src/{lib,coupled}.rs`, the generic
`CpuNewtonSolver<E,Msh,C,M,N,G>`.
