# Leg region — one joint → a dialable thigh–knee–shank twin (scan-free)

**Status:** RECON 2026-06-07. **A1 core de-risked** (general tree-FK reproduces the knee oracle to
machine zero — §"A1 spike"). Builds on the merged knee twin (`g1_knee_kinematics`, PR #277) and the
parametric-builder-first stack (PRs #278/#279).
**Goal:** generalize the one-joint pipeline into a **simulatable thigh–knee–shank region twin** that
is built **without a scan** and can be **dialed** (anthropometric parameters). Geometry (bone meshes
+ skin) is a *later* phase; scans are a *later* `ParamSource`. Representative muscle set, not all 92.

---

## Decision context (confirmed with the user 2026-06-07)

- **v1 = bodies/regions/parts made WITHOUT a scan**; scans get added later as a refinement layer.
- **Both artifacts, sequenced: simulatable twin FIRST, then the geometric body.**
- **Depth-first**: perfect the thigh–knee–shank region (the capstone's joint) before broadening.
- **Representative muscle set** (the existing 4 for A1), not the full 92.

## Cold-read of gait2392 (the whole model, not just the knee)

Verified against the vendored `gait2392.osim`:
- **Every joint is a `CustomJoint`** (12 of them): `ground_pelvis`, `hip_r`, `knee_r`, `ankle_r`,
  `subtalar_r`, `mtp_r` (+ left + `back`). Each has TransformAxes whose functions are **Linear** (=a
  DOF), **Constant**, or **SimmSpline** (coupled) — the exact shapes the knee parser already handles.
  Hip = 3 rotation DOFs + constant translations; knee = 1 rotation + coupled spline translations;
  ankle = 1 rotation + constants.
- **WRAP-FREE: 0 actual wrap geoms** (the `WrapObjectSet`s are empty containers; gait2392 uses
  conditional via-points instead). ⇒ the straight-segment oracle (`path length = Σ segments between
  active points`) generalizes to the **whole leg** with no wrap math. Big de-risk.
- **FK offsets**: `hip_r.location_in_parent` = the `hip_in_pelvis` we already use; `knee_r`'s is
  zero (consistent with today's oracle); all `orientation_in_parent` = 0.
- **92 muscles** total. A1 keeps the existing 4 (`rect_fem_r`, `vas_int_r`, `bifemlh_r`,
  `semimem_r`) — already a representative quads+hamstrings thigh–knee–shank set.

## The general IR (target shape)

A minimal MJCF/OpenSim-like kinematic tree, replacing the bespoke `Subgraph`:
- `Body { name, parent, location_in_parent, joint }` — a kinematic tree.
- `Joint = Vec<TransformAxis>`; `TransformAxis { rotation: bool, axis, function: TransformFn }`.
- `TransformFn { coordinate, kind: Linear{coeff} | Constant{v} | Spline(..) }` (gait2392 functions
  are single-coordinate).
- `Coordinate { name, default, range }` — the DOFs.
- Muscles as today (`Vec<PathPoint>`, already general).
- **FK convention** (validated): `world(child) = world(parent) · T(location_in_parent) ·
  [T(Σ translation axes) · R(Π rotation axes)]`, i.e. a point maps `x → R·x + t`. This is exactly
  `Isometry3::from_parts(translation, rotation)` composed down the tree.

## A1 spike — RESULT

`tools/cf-osim/tests/spike_general_fk.rs` (throwaway `#[ignore]`) builds a general `Body`-tree FK
(transform-axis composition + parent-chain walk) representing the knee chain (pelvis →
femur[hip welded at neutral] → tibia[knee coupled]) and compares its moment arms to
`oracle::Kinematics` (`Variant::TRUTH`) for all four muscles, 0→100° flexion.

**Result: max |Δ moment arm| = 0.00e0 mm** (literal machine zero) — the general transform-axis
composition is the *same math* as the bespoke `femur()/tibia()`, so the IR generalization is sound.
The hardest part (the FK convention on a tree) is retired before any clean build.

## Execution ladder

- **A1 — general IR + emit, no-regression on the knee.** *(DONE — PR-1: general IR + FK in
  `cf-msk-lib`; PR-2: the structural cutover.)* The general IR lives in `cf-msk-lib`; `cf-osim`'s
  `parse_leg_chain` reads the chain into a `Model`; the oracle reads the knee from that `Model` with
  its validated math unchanged; the general emitter is the new `cf-mjcf-emit` crate (the deferred
  split). **Exit (as delivered, through the general path):** (1) at A1 the general IR FK reproduced
  the bespoke oracle's moment arms to **machine zero** (`general_ir_fk`); A2 then folded the oracle
  *into* that FK (the bespoke knee math + the self-check were retired — the FK is now the oracle,
  anchored directly by the real-OpenSim cross-check). (2) `build_canonical` reproduces the oracle
  within the **5 mm S1 gate** for all four muscles (`bifemlh_r`, with no dropped conditional, matches
  to ~machine precision), and is byte-stable against the general emitter's own committed snapshot
  (`knee_ref.xml`). Note: the emitted MJCF is **not** byte-identical to the retired bespoke emitter
  (the general emitter uses principled names/structure); functional no-regression is the oracle gate,
  not byte-identity to the old emitter.
- **A2 — extend to thigh–knee–shank.** *(DONE — PR-1: oracle generalized to a multi-coordinate
  pose; PR-2: the hip unweld.)* `parse_leg_chain` reads the hip `CustomJoint` into the femur joint
  (3 rotation DOFs + zero translations) + adds the hip coordinates; the general emitter gives the
  femur its 3 hip hinges automatically; the knee stays coupled. No new muscles needed — 3 of the 4
  existing muscles (rect_fem_r/bifemlh_r/semimem_r) already span the hip (vas_int_r is femur-only).
  The IR now retains each `MovingPathPoint`'s driving coordinate (so the patella is driven correctly
  once >1 free DOF exists). **R-rot RETIRED:** the real-OpenSim cross-check
  (`moment_arms_opensim.json`, generated via opensim 4.6) is evaluated at a **multi-DOF base pose**
  (3 simultaneous non-zero hip rotations) — the oracle reproduces OpenSim's hip moment arms to
  ~0.001 mm (knee 0.078 mm), pinning the rotation-composition order. The emitted twin reproduces the
  oracle about every DOF at that pose (MuJoCo multi-hinge == our FK). Plausibility: vas_int_r (femur-
  only) has ~0 hip moment arm; the hip-spanning muscles ~50–63 mm.
- **A3 — real anthropometric `BodyParams`.** Generalize from per-segment *scale* to *lengths/girths*
  (+ a sex/percentile default, joint default poses). `CanonicalSource` becomes a dial-able generator.
  **The oracle is not lost — it relocates** to the morph (see *A3 plan* below): the per-axis morph
  *is* OpenSim's ScaleTool, so real OpenSim 4.6 grades it (Tier 1). The *parameter-choice* layer is
  what has no moment-arm oracle ⇒ shape-correlation + plausibility + anthro-table cross-check (§7).
- **A4 — `RandomizerSource`.** Sample the parameter space → a population of leg twins (training data).

## A3 plan (locked 2026-06-07)

**Structural facts from the cold-read that shape A3:**
- **Where lengths live:** `knee_r.location_in_parent = (0,0,0)` ⇒ the **femur length is the knee
  joint's coupled translation at neutral** (in the femur frame). `ankle_r.location_in_parent =
  (0,−0.425,0)` ⇒ the **tibia length is the ankle offset** — *which the IR does not parse* (the
  chain stops at the tibia). So a real shank length is literally unrepresentable until the ankle
  exists. **Decision: A3 adds the ankle** (`ankle_r`→a `talus` length-grounding body) — the A1 parser
  already handles its 1-rotation+constants form, our 4 muscles don't cross it (oracle-preserving
  no-regression checkpoint), and it is the only way to *define* a tibia length to dial.
- **Latent scale-convention inconsistency in `realize`:** `location_in_parent` scales with the body's
  *own* segment, but joint translations scale with the *parent*. Uniform-scale (exact dilation) and
  shape-correlation are both blind to this — it only bites once params mean real anisotropic
  lengths. The **differential oracle (Tier 1) + length round-trip (Tier 2) adjudicate and pin it.**
- **Full per-axis (anisotropic) morph:** OpenSim ScaleTool is per-axis (x,y,z) per body. `realize`
  grows from a scalar-per-segment to a per-segment **`Vector3` scale**, applied component-wise in the
  body frame (limb long axis = body-frame *y*): **length → axial (y); girth → transverse (x,z).**
  This makes the morph a one-to-one match for OpenSim's ScaleTool, which is what makes Tier 1 tight.
  Girth scale is grounded relative to the anthro table (`table_girth(p) / table_girth(canonical)`),
  not a bone-mesh envelope (we don't vendor bone meshes).

**Validation = a three-tier pyramid (the keystone):**
1. **Differential oracle** *(machinery; spike-gated)* — drive real OpenSim 4.6 ScaleTool with the same
   per-axis factors → `scaled_moment_arms_opensim.json`; grade the realized+emitted twin vs it over an
   anisotropic grid. Upgrades "uniform = exact dilation (analytic)" to "anisotropic = matches
   OpenSim's own scaling (empirical)." Spike first; degrade to the analytic anchor if non-reproducible.
2. **Internal consistency** *(derivation)* — uniform → ×s exact (keep); **length & girth round-trip**
   (dial L → realize → measure == L); determinism/idempotence.
3. **Plausibility / shape-corr / anthro cross-check** *(the parameter-choice layer — no moment-arm
   oracle)* — shape-corr ≥0.95 vs canonical across a percentile sweep (extends Spike B); segment-ratio
   & MA-magnitude bounds; generator output == published table.

Honesty (vision's two-claims rule): **Tier 1+2 prove the machinery is correct and matches OpenSim's
scaling; Tier 3 proves plausibility, not personhood.** A dialed body is "a clone of someone with these
proportions," never a validated individual.

**Slicing (each its own PR; n+1 cold-read cleanup; pre-PR local ultra-review):**
- **A3-PR1** — ankle (`talus`) + real **lengths** (end-to-end: `from_lengths` + `ScanSource` drives
  per-segment tibia scale) + per-axis **girth** *machinery* in `BodyParams`/`realize`
  (`with_girth_scales`; the real girth→scale *derivation* needs an anthropometric reference → arrives
  with the generator in PR3). Pin the scale convention with the length + girth round-trip.
  No-regression: 4 muscles unchanged by the ankle.
- **A3-PR2** — differential oracle. *(DONE.)* Spike confirmed real OpenSim 4.6 `Model.scale` with
  manual per-axis `Vec3` factors reproduces our morph (femur/tibia long axis = body-frame *y* in
  both; uniform AND anisotropic). Productionized: `gen_scaled_moment_arms.py` →
  `scaled_moment_arms_opensim.json` (a grid of length/girth configs over the knee ROM) + a `cf-osim`
  cross-check that `realize`s the same factors and grades the **oracle-on-realized model** (keeps the
  conditional points — so it validates the *morph machinery*, not the emitted MJCF; the emit's
  deep-flexion residual is the separate S1 dropped-conditional approximation, shown to stay within
  the 5 mm gate *under* scaling by `emit_tracks_oracle_under_scaling`) vs OpenSim's ScaleTool.
  **Result: 0.31–0.37 mm RMSE — within the same sub-mm band as the unscaled cross-check (~0.3 mm),
  machine-checked by a tight 0.8 mm gate: for this scope, the morph reproduces OpenSim's ScaleTool.**
  Scope: knee moment arms (4 muscles, 0…−100°, neutral hip, gait2392); hip-under-scaling not graded;
  pelvis is the fixed root (knee moment arms are translation-invariant to it). Tier 1 holds for
  anisotropic length + girth on this scope.
- **A3-PR3** — sex/percentile generator. *(DONE.)* `cf_msk_lib::AnthroSource::new(sex,
  stature_percentile)` (+ `.with_girth_percentile(p)`; percentiles in the open interval `(0,1)`)
  scales the template **proportionally** from published stature/girth distributions (ANSUR II stature;
  Winter segment∝stature; representative ANSUR girths), with the template as the **reference
  percentile** (50th-male) so it reproduces the template exactly there and dials an honest *relative*
  family (validates machinery, not personhood; the definitional trochanter-vs-hip-joint mismatch is
  sidestepped). `probit` (Acklam) maps percentile→z; no dep, crate stays pure. **Joint default poses
  are out of scope** (a `BodyParams` is size, not pose; the body is built at neutral). **Tier-3
  validation:** internal consistency (reference⇒identity, monotonicity, boundary-rejection —
  cf-msk-lib unit tests) + plausibility (segment lengths physiological/ordered across sampled
  percentiles, real gait2392) + shape-correlation (now consolidated into the PR4 scorecard/gate —
  see A3-PR4): the **default coupled** family (girth tracks stature) clears the §7 ≥0.95 bar (worst
  **0.979** among sampled percentiles 0.01–0.99, both sexes); an **extreme decoupled** build
  (tall+lean / short+stocky) is the documented boundary — the most girth-sensitive hamstring dips to
  **~0.87** (reported, loose 0.80 sanity floor), consistent with PR1's "girth is ~20–30%, not
  second-order."
- **A3-PR4** — three-tier scorecard harness + A4 randomizer prep. *(DONE.)* Consolidated the
  scattered A3 validation into ONE grader, `cf_osim::scorecard` (it *mirrors the pattern* of the g1
  scorecard — an independent grader over a generated body, no `Fitter`/scan — rather than extending
  it; lives in `cf-osim`, the only crate with both the oracle and the real-OpenSim references).
  **The keystone (tier-applicability):** a per-body card cannot re-run OpenSim, so the honest grade
  is **T2 (exact axial-length scaling) + T3 (per-body oracle FK: plausibility + shape-corr) + T1 as
  *coverage*** — `DiffOracleEnvelope` checks whether the body's per-axis factors fall inside the
  differential-oracle grid's tested per-axis ranges (an axis-box claim; combined points graded only
  at the `gen_*`/`realistic_mix` configs), with the one-time T1 grid result **cited**, not recomputed.
  `Regime::{Coupled,Decoupled,Unknown}`: shape-corr is **gated ≥0.95 for the coupled regime** and
  **reported** for the decoupled tail (≥0.80 floor) — a tall-lean body's curve shape legitimately
  differs, so hard-gating it would be dishonest. Regime comes from generator provenance
  (`AnthroSource::is_coupled`), never inferred for generated bodies; `Regime::infer` is a conservative
  (biased-to-strict) fallback for unlabelled params. `Scorecard::grade` /
  `grade_population`(`BodyEntry`) → `BodyScorecard` / `PopulationScorecard` (the API A4's
  `RandomizerSource` feeds). `examples/a3_scorecard` (printed report) + `tests/a3_gate` (the formal
  gate, superseding the retired `cf-osim/tests/anthro_validation.rs`). The differential-oracle grid
  was extended to bracket the generator's factor range (single-axis <1 configs + the generator's own
  coupled extremes at the sampled 1st/99th percentiles, both sexes → the whole coupled family is
  in-envelope; morph still tracks ScaleTool to worst **0.369 mm** under the 0.8 mm gate).

## Risks

- **R-FK — tree-FK convention.** *Retired by the A1 spike (0.0 mm).*
- **R-rot — multi-DOF rotation order.** *Retired by A2:* the real-OpenSim cross-check at a multi-DOF
  base pose (3 simultaneous non-zero hip rotations) reproduces OpenSim's hip moment arms to ~0.001 mm,
  pinning the composition order (R1·R2·R3 intrinsic, SpatialTransform order).
- **R-wrap — wrap surfaces.** *Retired:* gait2392 is wrap-free across the whole model.
- **R-emit — general emitter fidelity.** The general `cf-mjcf-emit` must round-trip an arbitrary
  chain through the import-only engine. De-risk = the A1 no-regression checkpoint (reproduce the knee
  bytes) before extending.
- **R-val — no oracle for dialed bodies (A3).** *Reframed (see A3 plan):* the oracle **relocates**,
  it isn't lost — the per-axis morph *is* OpenSim's ScaleTool, so real OpenSim 4.6 grades the morph
  machinery (Tier 1, spike-gated). Only the *parameter-choice* layer (which factors a percentile body
  "should" have) has no moment-arm oracle → internal-consistency + shape-correlation + anthro-table.

## First concrete action

Implement A1's general IR + parser, gated by the no-regression checkpoint (general path reproduces
the knee). The spike's `Model`/`Body`/`Axis`/`Func` types are the prototype to productionize.
