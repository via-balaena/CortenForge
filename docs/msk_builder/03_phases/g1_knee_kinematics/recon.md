# G1 — Knee kinematics (scan → validated articulated knee)

**Status:** RECON drafted 2026-06-07; **S0 bridge spike COMPLETE** (§6 "S0 RESULTS") — bare-hinge
rejected by data; **S1 coupled-knee model COMPLETE** (§6 "S1 RESULTS") — reproduces the
OpenSim-geometry oracle within 5 mm for all 4 muscles; **the oracle itself is now VALIDATED against
real OpenSim 4.6 to 0.3 mm RMSE** (`opensim_cross_check.rs`), so the figures have an independent
anchor. Reviewed 2026-06-07 (adversarial 3-reviewer pass: no math/physics bugs; convention verified
vs OpenSim docs; linear→cubic splines, guards, claim re-scoping applied). Decisions locked (§5).
**Reframed 2026-06-07 to parametric-builder-first** — the builder produces a *canonical* knee from
the validated library with **no scan required**; a scan, when present, is one `ParamSource` among
several (`../../02_foundations/library_parameter_architecture.md`). This changes the entry point,
**not** any S0/S1 result below.
**Phase goal:** produce a knee skeleton+tendons in MJCF — *canonical* (library defaults, no scan) or
*scan-fitted* (landmarks → parameters → morph) — that articulates inside the skin envelope, with
joint center + moment-arm curves matching the OpenSim oracle within tolerance. Purely kinematic —
no muscle force, no hardware.
**Predecessor:** none (first phase). **Successor:** G2 (Hill-muscle-driven motion).
**Mission linkage:** deliverable #4 (musculoskeletal model builder), thesis steps 1–2.

---

## 1. Empirical baseline — what ships today

Nothing anatomy-specific ships. The substrate (`../../02_foundations/existing_substrate.md`)
gives us, verified by read-only sweep:

- A rigid-body engine where **a knee hinge + spatial tendons with sphere/cylinder wrapping** are
  first-class, and **moment arms fall out of `data.ten_J`** projected onto the joint DOF
  (`sim/L0/core/src/tendon/spatial.rs`).
- An MJCF **importer** but **no exporter and no programmatic `Model` builder**
  (`sim/L0/mjcf/`) — every generated model must be emitted as XML and round-tripped.
- A mature scan→watertight-twin pipeline with centerline / PCA / cross-section / radial-profile
  (`tools/cf-scan-prep-core/src/lib.rs`) and `flood_filled_sdf` inside/outside
  (`mesh/mesh-sdf`).
- A **Hill muscle model** (`sim/L0/core/src/forward/hill.rs`) that is MuJoCo-piecewise, not
  OpenSim Thelen/Millard — irrelevant to G1 (kinematic only), flagged for G2 (R5).

There is **no** anatomical landmark detection, **no** mesh segmentation into anatomical regions,
**no** template/registration machinery, and **no** biomech-format (`.osim`) support anywhere.

## 2. Architectural thesis

Four moves make G1 tractable and *validated in software*:

1. **Stand on validated anatomy, don't re-derive it.** A bridged OpenSim lower-limb model
   carries literature-validated joint centers and muscle paths. We morph it rather than
   synthesize muscle routing — keeping the research risk off the thing we can't easily check.
2. **The template is also the oracle.** Because we compute moment arms in-engine from `ten_J`, we
   can grade our converted/scaled knee against the *same* OpenSim model's published curves. The
   first gate needs no hardware.
3. **The missing authoring path is an MJCF emitter, not a new engine.** Since the engine is
   import-only, the "programmatic builder" is a deterministic XML emitter (`cf-mjcf-emit`) that
   the existing importer accepts and round-trips. This is the smallest thing that unblocks
   everything else.
4. **Parameters morph an anatomical IR; the scan is just one source of parameters.** The library
   (`cf-msk-lib`) holds a generalized biomech IR + a pure `realize(template, params) -> BodySpec`
   morph; a `ParamSource` trait feeds it. `CanonicalSource` (library defaults) gives a full body
   with **no scan** — so the scanner stops being a blocking dependency. `ScanSource` (cf-anthro
   landmarks → params, tier a) and `RandomizerSource` (free training data) are siblings, not the
   pipeline. The raw scan mesh is never deformed (tier c is rejected). Full sketch:
   `../../02_foundations/library_parameter_architecture.md`.

## 3. End-state definition (G1)

The builder produces a `knee.xml` MJCF — from `CanonicalSource` (library defaults, **no scan**) or,
given a cleaned leg scan (from `cf-scan-prep`), from `ScanSource` (`knee_<subject>.xml`). Both flow
through the same `realize` → `cf-mjcf-emit` path; only the `BodyParams` differ. The emitted model:
- places a 1-DOF knee hinge at the anatomical joint center/axis (canonical, or derived from scan
  landmarks);
- carries the four representative crossing muscles (rectus femoris, one vastus, biceps femoris
  long head, semimembranosus) as spatial tendons with their wrap geoms;
- articulates 0→100° flexion with **all bone & tendon geometry inside the scan skin envelope**;
- reproduces the OpenSim oracle's **joint center (<5 mm)** and **moment-arm curve shapes
  (corr ≥0.95, length-normalized)** after scaling.

Out of scope for G1 (deferred): patellofemoral mechanism (patella = fixed site), moving instant
center / deep-flexion fidelity (>100°), muscle force / activation (G2), soft tissue, hardware.

## 4. Gap table

| Capability | Current | G1 end-state | Disposition | Where |
|---|---|---|---|---|
| `.osim` → biomech IR | none | knee subgraph (2 bodies, 1 joint, 4 muscles, wraps) parsed | **build** | `tools/cf-osim/` |
| Anatomical IR + library | `cf-osim::Subgraph` (knee-only, in importer) | source-agnostic IR + `BodyTemplate` w/ per-segment scaling rules | **build (extract)** | `tools/cf-msk-lib/` |
| Parameter interface | scan fused into `place()` | `BodyParams` (named, `Measurable`-gated) + `ParamSource` trait; `Canonical`/`Randomizer` sources | **build** | `tools/cf-msk-lib/` |
| MJCF emitter | import-only | deterministic emitter the importer round-trips | **build** | `sim/L0/mjcf-emit/` |
| Moment-arm extraction | exists (`ten_J`) | projected onto knee DOF, sampled over ROM | **reuse** | `sim/L0/core/tendon/spatial.rs` |
| Knee landmark detection | centerline/girth only | joint-line, epicondyle width, segment lengths | **build** | `tools/cf-anthro/` |
| Morph + scan registration | none | pure `realize()` (per-segment scale, rules in the library) + `ScanSource` (landmarks → `BodyParams`) | **build** | `cf-msk-lib` (morph) + `cf-msk-fit` (`ScanSource`) |
| Skin-envelope containment | `flood_filled_sdf` exists | per-angle worst-case protrusion over ROM | **build (thin)** | `tools/cf-msk-fit/` + `mesh-sdf` |
| Oracle comparison harness | none | G1 scorecard (axis, moment-arm RMSE, protrusion) | **build** | `cf-msk-validate` |
| Watertight scan ingest | shipped | reuse as-is | **reuse** | `cf-scan-prep-core` |

## 5. Decisions

### Closed (locked with the user 2026-06-07)
- **D1 — First target = knee, as a 1-DOF hinge at G1.** Moving-center fidelity deferred.
- **D2 — Fitting = template registration + scan-driven scaling** (OpenSim Scale-tool analogue).
  Muscle paths inherited, not synthesized.
- **D3 — Interop = bridge OpenSim `.osim` → MJCF, spiked first.** Converted model = template + oracle.
- **D4 — G1 gate = kinematics validated vs the OpenSim oracle**, no hardware. G2 = muscle-driven.
- **D5 — Primary input model = gait2392** (most-published lower-limb gait model).
  **VENDORED 2026-06-07** (Apache-2.0, from `opensim-org/opensim-core`) at
  `sim/L0/tests/assets/opensim_gait2392/gait2392.osim` — see its `PROVENANCE.md`. The generic
  Thelen template in `opensim-models` was rejected for vendoring (no LICENSE file); the
  Apache-2.0 opensim-core copy is structurally identical for S0 (moment arms depend on path
  geometry, not the muscle force model). Rajagopal = secondary cross-check only.
- **D6 — G1 ROM bounded to 0–100°**, where a fixed hinge is honest; 100–120° flagged (R3).
  *(Confirmed: the vendored model's `knee_angle_r` spans −120°…+10°.)*

### Architecture (folded in 2026-06-07 — `02_foundations/library_parameter_architecture.md`)
- **D7 — Parameters morph the IR, then emit MJCF.** The raw scan mesh is never deformed (tier c
  rejected). Emit from the morphed IR via `cf-mjcf-emit`.
- **D8 — One `BodyParams`, many `ParamSource`s.** Decouples scan from builder; `CanonicalSource`
  makes the program builder-first; `RandomizerSource` yields free training data.
- **D9 — Scaling rules live on the segment, not in the solver.** Adding a bone never edits
  `realize()`.
- **D10 — `Measurable` is a type, not a comment.** A `ScanSource` may only set scanner-measurable
  params (lengths/girths/centers); the rest fall back to template defaults / anthropometric priors.
- **D11 — Joint coupling stays symbolic through the morph.** Scaling changes geometry, not the
  rolling-glide relation — consistent with "1-DOF *coordinate* knee at G1, coupling carried."

### Open (resolve during S0, before committing the program)
- **O1 — Does the gait2392 knee's coupled tibial translation matter at <100°?** *Confirmed
  concrete:* the vendored model's `knee_r` is a `CustomJoint` whose `knee_angle_r` drives
  `translation1`/`translation2` via `SimmSpline` (coupled tibiofemoral translation). Measure in
  S0 the moment-arm error from dropping those splines for a pure hinge; if it exceeds tolerance
  below 100°, narrow the validated ROM or add a `Joint`-coupled length term (R3).
- **O2 — Can fixed `Site`s reproduce the OpenSim patella mechanism within tolerance?** *Sharpened
  by S0 parsing (2026-06-07):* the vendored gait2392 has **zero wrap objects** — the knee/patella
  mechanism is modeled entirely with **`MovingPathPoint`** (8) and **`ConditionalPathPoint`** (26).
  The quadriceps (`rect_fem_r`, `vas_int_r`) effective insertion is a `MovingPathPoint` on
  `tibia_r` whose location is a `SimmSpline` of `knee_angle_r` (emulating the patella); the
  hamstrings (`semimem_r`, `bifemlh_r`) use plain `PathPoint`s. So the open question is concretely:
  *does freezing the moving quad insertion at its θ=0 location stay within tolerance?* Expected
  answer: fine for hamstrings, **not** for quads → `cf-mjcf-emit` will need `MovingPathPoint`
  support (or an explicit patella body). Decided by the S0 per-muscle moment-arm RMSE.
- **O3 — Scan bank.** Which real leg scans (and synthetic-from-oracle scans) form the G1 test
  set? Need ≥3; synthetic scans can be generated by skinning the oracle for an early, clean signal.

## 6. Sub-leaf ladder (horizontal slices — each independently demonstrable)

Through-line artifact refined slice by slice: **a knee MJCF that loads in the importer and
articulates.**

### S0 — OpenSim→MJCF bridge spike *(FIRST action; #1 risk; throwaway-grade)*
Convert *only* the gait2392 knee subgraph (femur + tibia hinge; 4 muscles + wraps; patella =
fixed site) to a minimal MJCF; load through `sim/L0/mjcf`; FK across 0→100°; extract moment arms
from `data.ten_J`; compare to gait2392's published curves + axis.
- **Artifact:** table + overlay plot — oracle vs converted flexion-axis (mm) and moment-arm
  curves (mm vs flexion) for the four muscles.
- **Exit:** joint-center **<5 mm**; moment-arm **RMSE <5 mm** across 0–100° for all four muscles.
- **Front-loads R4:** the very first sub-step is hand-authoring a minimal MJCF (one hinge + one
  spatial tendon + cylinder wrap), loading it, and reading back `ten_J` — proving the
  emit→import→moment-arm loop closes and pinning the schema subset.

**S0 progress (2026-06-07):**
- **R4 micro-spike GREEN.** `tools/cf-osim` crate created (the on-plan converter home; starts
  spike-grade). `cf-osim::moment_arm_sweep` extracts `-ten_J` at a joint DOF;
  `tests/r4_micro_spike.rs` (3 tests pass) proves the loop closes on an ideal-pulley knee — `-ten_J`
  matches a length-only finite difference (extraction + sign correct) **and** equals the wrap
  cylinder radius (analytic anchor).
- **Oracle is self-contained — no OpenSim binary needed (key finding).** Because the vendored
  gait2392 has **no wrap objects** (O2), OpenSim's path length is exactly the sum of straight
  segments between active path points. So the oracle `L(θ)` can be computed directly from the
  `.osim` point coordinates under the true coupled-knee kinematics (evaluating `MovingPathPoint`
  splines + `ConditionalPathPoint` ranges) — and our straight-multi-site spatial tendon computes
  the *same* sum. The our-engine-vs-oracle gap therefore isolates exactly the G1 approximations
  (frozen hinge = R3; frozen moving quad point = O2; dropped conditional points), which is precisely
  what S0 must measure. No literature digitizing, no OpenSim install.
- **S0 COMPLETE — scorecard below.** Full converter shipped in `tools/cf-osim` (`xml` tree,
  `osim` parser w/ `SimmSpline`, `oracle` straight-segment path length under true coupled
  kinematics, `emit` frozen-hinge MJCF) + `tests/s0_knee_bridge.rs` (`#[ignore]`).

**S0 RESULTS (sweep 0–100° flexion, moment-arm RMSE vs the re-derived oracle, mm; cubic splines):**

| muscle | moving pts | oracle MA@0° | **total Δ (engine vs oracle)** | R3 (coupling) | O2 (patella) | Cond | engine≈analytic |
|---|---|---|---|---|---|---|---|
| rect_fem_r | 1 | +49.9 | 10.7 | 12.4 | 21.5 | 3.7 | **0.001** |
| vas_int_r | 1 | +49.9 | 9.7 | 12.6 | 22.0 | 2.9 | **0.001** |
| bifemlh_r | 0 | −26.5 | 14.7 | **14.7** | 0 | 0 | **0.001** |
| semimem_r | 0 | −30.5 | 8.5 | **14.7** | 0 | 7.8 | **0.001** |

> **What "oracle"/"truth" means here (read first).** The oracle is OUR re-derivation of OpenSim's
> `GeometryPath` geometry from the `.osim` coordinates — with no wrap objects it is a straight-segment
> sum, evaluated with **natural cubic** `SimmSpline`s. **It is now VALIDATED against real OpenSim 4.6**
> (`opensim` PyPI wheel, run on this exact model): our oracle reproduces OpenSim's knee moment arms to
> **0.29–0.32 mm RMSE** (max 0.67 mm at deep flexion) for all four muscles over 0–100°
> (`tools/cf-osim/tests/opensim_cross_check.rs` vs the vendored `knee_moment_arms_opensim.json`). So
> "vs oracle / vs truth" figures now have an **independent anchor** — the oracle is faithful OpenSim
> geometry to sub-mm, not merely self-consistent. (The separate engine↔analytic "0.001 mm" check is a
> distinct, weaker plumbing check — same straight-segment formula both sides.)

**Verdict:**
1. **R1 + R4 de-risked (plumbing + representation), oracle now independently validated.** The
   engine's `-ten_J` matches the analytic all-frozen model to **0.001 mm** for every muscle (same
   formula both sides ⇒ this confirms the parse→emit→import→extract plumbing). And the oracle it's
   graded against is itself now validated against **real OpenSim 4.6 to 0.3 mm RMSE** (see the banner
   above) — so the S0 numbers below are differences from a faithful OpenSim reproduction, not a
   self-referential model. The MJCF representation + extraction are correct; the kinematic
   approximation is the gap.
2. **The bare 1-DOF hinge G1 simplification is REJECTED by data.** A frozen hinge misses the 5 mm
   budget for **all four** muscles (8.5–14.7 mm). The dominant error is **R3 — the coupled tibial
   translation (~12–15 mm even for plain-point hamstrings)**, *inverting* the prior assumption that
   the patella (O2) would dominate. *(The R3/O2/Cond columns are oracle-internal sensitivities —
   non-additive and partly cancelling: quads freeze both → 10.5 mm vs 22 mm for O2 alone — so they
   rank the approximations; R3-dominance is cleanest on the hamstrings, where R3 ≈ total.)*
3. **Decision driven (supersedes the optimistic side of D1/O1/O2):** G1 must **model the knee
   coupling**, not approximate it away. `cf-mjcf-emit`'s first requirements are now concrete:
   (a) coordinate-coupled tibial translation (MuJoCo equality-joint coupling of slide DOFs to the
   flexion angle); (b) the patella `MovingPathPoint` (moving site or explicit patella body);
   (c) `ConditionalPathPoint` membership. The "knee as a *bare* hinge" reading of D1 is retired;
   "1-DOF *coordinate*" stands (one independent DOF, with the rest coupled to it).

**S1 RESULTS (2026-06-07) — coupled-knee model reproduces the re-derived oracle within 5 mm:**

The S1 model (`cf_osim::emit::emit_coupled_knee`) represents the gait2392 knee instead of
approximating it: coupled tibial translation → two coupled slide DOFs (`knee_tx`, `knee_ty`);
each patella `MovingPathPoint` → a small coupled "patella" body (3 slide DOFs, site at origin);
conditional points dropped (a static tendon can't toggle membership; a one-off probe showed
dropping beats always-including for semimem — but it does drop a point active in [0,−32°], so it's
a real structural approximation, not free). Driving the coupled DOFs along the gait2392 splines and
reading moment arms via the TOTAL derivative (`cf_osim::coupled_moment_arm`, finite-diff of
`ten_length` along the coupled manifold — `-ten_J[knee]` alone would be only the partial):

| muscle | S0 bare hinge | **S1 coupled** | residual source |
|---|---|---|---|
| rect_fem_r | 10.7 | **3.71** ✓ | dropped deep-flexion conditional |
| vas_int_r | 9.7 | **2.89** ✓ | dropped conditional |
| bifemlh_r | 14.7 | **0.00** ✓ | — (pure translation; ~tautological, see below) |
| semimem_r | 8.5 | **2.49** ✓ | dropped in-range conditional |

**All four < 5 mm.** *What this proves and its limit:* S1 drives the coupled DOFs with the **same
splines the oracle uses**, so this is a **driven-consistency / representation** test — it shows the
emitted coupled MJCF faithfully *reproduces* the oracle kinematics (bifemlh = 0.00 is near-
tautological by construction; the residuals come ONLY from the dropped conditional points). It is
**not** an independent check against real OpenSim. What it genuinely establishes: the engine's
spatial-tendon + multi-DOF machinery *can* represent the gait2392 coupled knee, and the bare-hinge
error S0 found is recovered by the coupled representation. Incremental attribution was clean:
coupled translation alone took bifemlh 14.7→0.00; adding patella bodies took the quads ~20→~3;
dropping conditionals took semimem 7.8→2.5. *Note:* coupling alone made the quads **worse** (20 mm)
by unmasking the patella error that had partially cancelled R3 in S0 — coupling + patella must be
done together. Test: `tools/cf-osim/tests/s1_coupled_knee.rs`.

**S1 remaining (next):** (1) ✅ **DONE — real-OpenSim cross-check** (oracle vs OpenSim 4.6,
**0.3 mm RMSE**; `opensim_cross_check.rs` + vendored `knee_moment_arms_opensim.json`). The oracle is
now an independently-validated reproduction of OpenSim geometry, so the S0/S1 figures inherit a real
anchor. (2) freeze a `knee_ref.xml` MJCF template for downstream grading (the validated moment-arm
*curves* are already the vendored OpenSim JSON); (3) promote `cf-osim` from spike-grade panics to a
`Result`-based clean converter; (4) G2-prep: emit MuJoCo equality-joint coupling (poly-fit the
splines) so the model self-actuates, and measure the poly-fit residual; (5) upgrade conditional
points from "dropped" to a proper membership toggle / wrap geom.

### S1 — Frozen oracle template + the library/parameter spine
Promote the spike to a clean deterministic converter; commit the reference (unscaled) knee MJCF
+ a serialized **oracle bundle** (joint center, axis, per-muscle moment-arm curves, path points
in canonical frames). Stand up the builder spine: extract the source-agnostic IR into `cf-msk-lib`,
and prove the **builder-first** path end-to-end (`CanonicalSource` → `realize` → `cf-mjcf-emit` → a
canonical knee, no scan).
- **Builds:** `cf-osim` (reader → IR + oracle), `cf-msk-lib` (IR extracted from `cf-osim::Subgraph`,
  `BodyTemplate`, `BodyParams`, `CanonicalSource`, `realize()` — uniform scale first),
  `cf-mjcf-emit` (emitter; `cf-osim::emit` migrates here).
- **Artifact:** committed `knee_ref.xml` + `knee_oracle.json`; a passing test that re-derives
  moment arms from the *imported* emitted model and matches the bundle; **plus** a builder-first
  test: `CanonicalSource → realize → emit` reproduces `knee_ref.xml` from library defaults alone.
- **Exit:** byte-stable emit; importer accepts it; in-engine moment arms reproduce the oracle
  within S0 tolerance; the canonical (no-scan) body emits and passes the same oracle check.
  *(This bundle is the ground truth the rest of the program is graded on.)*

### S2 — Landmark detection on the scan *(heuristics-first, no ML)*
From a cleaned leg scan, detect: knee joint-line height (cross-sectional-area minimum along the
centerline between thigh & calf girth maxima), epicondyle mediolateral width (OBB extent at that
level), segment lengths (centerline arc-length), girths.
- **Reuses:** `compute_centerline_polyline`, PCA/OBB, cross-section + radial-profile, SDF.
- **Builds:** `cf-anthro`; emits `landmarks.toml` + an overlay render.
- **Exit (vs hand-annotation on ≥3 scans):** knee-line **±10 mm**; epicondyle width **±8 mm**;
  segment length **±5%**.

### S3 — `ScanSource` + per-segment morph
Make the scan one `ParamSource`: map the S2 detected landmarks → `BodyParams` (`ScanSource`), and
upgrade `realize()` from uniform to per-segment anisotropic scale so the morphed template's model
landmarks coincide with the scan's. Same `realize` → `cf-mjcf-emit` path as the canonical body —
only the params differ.
- **Builds:** `cf-msk-fit::ScanSource` (`cf_anthro::Landmarks` → `BodyParams`); the per-segment
  `ScaleRule` in `cf-msk-lib` (the recon's `ScaleSpec`, now living on the segment). Closed-form
  per-axis scale from paired distances first; upgrade to Procrustes/LM only if residual demands.
- **Checkpoint (no behavior change):** `ScanSource` routed through the new seam must reproduce the
  pre-refactor `cf-msk-fit::place` result — a refactor-safety gate before per-segment scaling moves
  anything.
- **Exit:** landmark residual **<5 mm RMS**; scaled bone lengths within **±3%** of scan segments.

### S4 — Articulation inside the skin envelope
Drive the scaled knee hinge through 0→100°; sample bone-geom + tendon-path points; query the
scan SDF; report worst-case protrusion per angle. Visual gate via `cf-viewer`/`sim-bevy`.
- **Builds:** containment evaluator in `cf-msk-fit`.
- **Exit:** **≤2 mm** worst-case protrusion through ROM (configurable band via `mesh-offset`);
  tendon paths non-self-intersecting and wrap surfaces engaged.

### S5 — G1 GATE: full validation harness
Assemble end-to-end (scan → landmarks → scaled knee → articulation) and run the three-check
comparison (§7) against the S1 oracle bundle.
- **Builds:** `cf-msk-validate` (or a module in `cf-msk-fit`) → the G1 scorecard.
- **Exit:** all §7 tolerances met on the scan bank. G2 explicitly deferred.

### Suggested execution order
S0 → (gate decision on O1/O2) → S1 → S2 ∥ S3-prep → S3 → S4 → S5. **S1 already yields a usable
canonical (no-scan) knee** via `CanonicalSource` — the scan path (S2/S3) is additive personalization,
not a prerequisite for a body to exist. S2 can proceed in parallel with S1 once S0 passes, since it
only needs scans, not the oracle. `RandomizerSource` (training data) is a cheap add once `realize()`
exists (post-S1); tier-(b) `ShapeModelSource` is post-G1.

## 7. G1 validation strategy (what is compared, metric, tolerance)

All against the **S1 oracle bundle**. Three independent checks:

1. **Joint center / flexion axis** — distance between scaled-model and oracle flexion-axis
   midpoint, expressed *relative to detected scan landmarks*. Tol: **<5 mm** center, **<5°** axis.
2. **Moment-arm curves** — rectus femoris, vastus, biceps femoris LH, semimembranosus, every 5°
   over 0–100°, from `data.ten_J` projected on the knee DOF.
   (a) *Unscaled reference* (S1 acceptance): **RMSE <5 mm**.
   (b) *Scaled subject*: **shape correlation ≥0.95** + length-normalized shape RMSE in-band
   (absolute magnitude legitimately scales with the subject).
3. **Skin-envelope containment** — through 0–100° via `flood_filled_sdf`: worst-case signed
   distance of any bone vertex / tendon point outside skin **≤2 mm**, zero gross protrusions.

G1 passes only when all three hold on the test-scan bank. The scorecard (axis distance,
per-muscle moment-arm RMSE, max protrusion) + overlay plots land in this doc at gate time.

## 8. New crates / modules

| Crate / module | Path | Role | Slice |
|---|---|---|---|
| `cf-osim` | `tools/cf-osim/` | `.osim` XML → biomech IR (library seed) + oracle (`Kinematics`, the grader) | S0/S1 |
| `cf-msk-lib` | `tools/cf-msk-lib/` | the IR (extracted from `cf-osim::Subgraph`), `BodyTemplate`, `BodyParams`, `realize()` morph, `ParamSource` trait + `Canonical`/`Randomizer` sources | S1 |
| `cf-mjcf-emit` | `sim/L0/mjcf-emit/` | IR → MJCF XML the importer accepts (`cf-osim::emit` migrates here; lives next to `sim/L0/mjcf` to share schema types + round-trip test) | S0/S1 |
| `cf-anthro` | `tools/cf-anthro/` | scan landmark detection → `landmarks.toml` | S2 |
| `cf-msk-fit` | `tools/cf-msk-fit/` | `ScanSource` (`Landmarks` → `BodyParams`) + per-segment registration + envelope containment | S3/S4 |
| `cf-msk-validate` | `tools/cf-msk-validate/` (or module in `cf-msk-fit`) | oracle comparison harness; G1 scorecard | S5 |

## 9. Risks + de-risking spikes

- **R1 — OpenSim→MJCF converter fidelity (#1).** Frame conventions (OpenSim Y-up, `<Frame>`
  graph), `MovingPathPoint` geometry has no direct MJCF analogue, wrap parameterization differs.
  *De-risk = S0.* If sphere/cylinder `Geom` + fixed `Site` wraps can't hit <5 mm moment-arm RMSE,
  escalate to conditional/moving path-points **before** committing the program (O2).
- **R2 — Landmark robustness.** Scan noise, pose variation, occlusion behind the knee, weak
  joint-line signal. *De-risk (parallel to S1):* run the S2 heuristic on a varied scan bank,
  measure scatter before the solver depends on it. G1 scoped to standing/extended-pose scans.
- **R3 — Knee-as-hinge vs real kinematics.** Fixed hinge mis-places contact at deep flexion.
  *De-risk:* in S0, plot oracle moment arms vs a fixed-axis fit; bound the validated ROM to where
  fixed-hinge error < tolerance (0–100°; flag 100–120°). Keeps D1 honest and bounded (O1).
- **R4 — No programmatic builder (must emit MJCF XML).** *De-risk (front-loaded in S0):*
  hand-author a minimal MJCF, load it, read back `ten_J` — proves the loop closes and pins the
  schema subset `cf-mjcf-emit` must produce.
- **R5 — Hill model: validation status (G2 foreshadow).** *Audited 2026-06-07 against
  `examples/COVERAGE_SPEC.md` + run.* **What G1/S0 rests on is strongly proven:** spatial-tendon
  length + Jacobian (→ moment arms via `ten_J`) pass the tendon stress-test **16/16** (incl. check
  16 "moment arm varies with config", which is literally `J[dof]`; it reproduces the cylinder-radius
  result of our R4 spike — corroborating but *not independent*, both lean on the same engine wrap
  solver) and **72** `spatial_tendons` integration tests (MuJoCo 3.4.0 conformance + finite-diff
  Jacobian). *(Counts per `examples/COVERAGE_SPEC.md` + a 2026-06-07 run of those suites.)* So the S0
  R3-dominance finding is trustworthy, not an engine artifact. **The Hill *force* model is numerically validated but not visually:** the muscle
  stress-test passes **51/51** incl. Hill FL/FV/FP curve pinning, Hill isometric force, pennation,
  and `Muscle == HillMuscle` activation — but every *visual* muscle example drives the MuJoCo
  `<muscle>`, not `HillMuscle` (the Hill visual example is an open Track-1B item). **Two real G2
  actions:** (a) build the missing Hill *visual* example (the repo's philosophy: visual examples
  catch what unit tests miss); (b) reconcile engine-Hill (Gaussian FL / hyperbolic FV / exp FP)
  with OpenSim Thelen/Millard. *Minor flag:* the stress-test pennation ratio is 0.9326 vs cos(20°)
  = 0.9397 (0.7%, passes tolerance) — worth a look if G2 needs pennation fidelity. G1 stays purely
  kinematic, so none of this blocks it.

## 10. Scope discipline (explicitly NOT in G1)

Patellofemoral mechanism · moving instant center / >100° flexion · muscle force/activation
(G2) · soft tissue & contact · hardware / physical sim-to-real (later gate) · any joint other
than the knee · whole-body generalization.

## 11. Verification

- **S0 spike** — the §6-S0 pass/fail table *is* the test (moment-arm RMSE + joint-center distance
  vs published gait2392 curves). Run as a throwaway `#[ignore]` test or example binary; the
  *number* is the deliverable. Do not start S1 until S0 passes.
- **Per slice S1–S5** — each exits on its own measurable criterion (byte-stable emit + oracle
  reproduction; landmark vs hand-annotation bands; solver residual; envelope containment; G1
  scorecard). Numeric gate via `cargo test`; visual gate via `cf-viewer`/`sim-bevy`.
- **G1 overall** — the §7 three-check scorecard on ≥3 leg scans, all tolerances met.

## 12. First concrete action

The S0 bridge spike (§6-S0). Input: the vendored `sim/L0/tests/assets/opensim_gait2392/gait2392.osim`
(Apache-2.0). Convert the knee subgraph + four
muscles to minimal MJCF; load; FK 0→100°; compare moment arms + axis to the published curves.
PASS = axis <5 mm and moment-arm RMSE <5 mm for all four muscles across 0–100°. On FAIL, diagnose
frame/units vs unmodeled `MovingPathPoint` before advancing — it dictates whether `cf-mjcf-emit`
needs moving path-point support.

## Critical files (reuse anchors)
- `sim/L0/core/src/tendon/spatial.rs` — moment arms via `ten_J` (the oracle-comparison quantity)
- `sim/L0/mjcf/src/builder/tendon.rs` — spatial-tendon + wrap MJCF schema subset to emit
- `sim/L0/mjcf/src/` — the importer contract `cf-mjcf-emit` must satisfy (no exporter exists)
- `sim/L0/core/src/forward/hill.rs`, `fiber.rs` — Hill muscle (G2)
- `tools/cf-scan-prep-core/src/lib.rs` — centerline / cross-section / radial-profile substrate
- `mesh/mesh-measure/src/` — circumference / OBB measurements
- `mesh/mesh-sdf/src/` — `flood_filled_sdf` for envelope containment
