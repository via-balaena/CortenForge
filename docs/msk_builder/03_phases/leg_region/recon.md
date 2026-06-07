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

- **A1 — general IR + emit, no-regression on the knee.** *(spike done)* Implement the general IR
  (extract into `cf-msk-lib`), the general parser (`cf-osim` reads all chain joints/bodies into it),
  the general oracle FK, and the general emitter (`cf-mjcf-emit`). **Exit:** `build_canonical`
  reproduces today's knee MJCF byte/oracle-identical *through the general path* — the same
  no-behavior-change checkpoint used for ScanSource. This is also where the deferred `cf-mjcf-emit`
  crate split finally lands.
- **A2 — extend to thigh–knee–shank.** Unweld the hip (3-DOF, held at a default pose) so the femur
  is placed by the hip joint; keep knee coupled. Add hip-spanning muscles to the representative set
  if needed for plausibility (e.g. a glute, iliopsoas). Oracle-validate every joint's moment arms.
- **A3 — real anthropometric `BodyParams`.** Generalize from per-segment *scale* to *lengths/girths*
  (+ a sex/percentile default, joint default poses). `CanonicalSource` becomes a dial-able generator.
  No oracle for a dialed body ⇒ validate by shape-correlation + plausibility (the §7 idea).
- **A4 — `RandomizerSource`.** Sample the parameter space → a population of leg twins (training data).

## Risks

- **R-FK — tree-FK convention.** *Retired by the A1 spike (0.0 mm).*
- **R-rot — multi-DOF rotation order.** The hip's 3 rotations compose in a specific order away from
  neutral. A1 holds the hip at neutral (order-independent); A2 must pin the order vs OpenSim when the
  hip articulates. Bounded, not yet exercised.
- **R-wrap — wrap surfaces.** *Retired:* gait2392 is wrap-free across the whole model.
- **R-emit — general emitter fidelity.** The general `cf-mjcf-emit` must round-trip an arbitrary
  chain through the import-only engine. De-risk = the A1 no-regression checkpoint (reproduce the knee
  bytes) before extending.
- **R-val — no oracle for dialed bodies (A3).** A custom-proportion body has no real subject. Fall
  back to internal-consistency (uniform scale = exact dilation, already proven) + shape-correlation.

## First concrete action

Implement A1's general IR + parser, gated by the no-regression checkpoint (general path reproduces
the knee). The spike's `Model`/`Body`/`Axis`/`Func` types are the prototype to productionize.
