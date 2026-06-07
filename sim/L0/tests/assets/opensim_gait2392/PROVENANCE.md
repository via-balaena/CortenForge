# Vendored asset — OpenSim gait2392 musculoskeletal model

Vendored 2026-06-07 for the musculoskeletal-builder arc (Mission deliverable #4).
Consumed by the **S0 OpenSim→MJCF bridge spike** — see
`docs/msk_builder/03_phases/g1_knee_kinematics/recon.md`.

## What this is

`gait2392.osim` — the classic 23-DOF, 92-muscle bipedal gait model
(Delp et al. 1990; 3D version by D.G. Thelen & A. Seth), here the example-subject
(`subject01`) variant. `OpenSimDocument Version="30000"`, `Millard2012EquilibriumMuscle`
actuators. It contains the knee joint and all four muscles the S0 spike targets
(rectus femoris, vasti, biceps femoris long head, semimembranosus).

For S0, **only the muscle *path geometry* (path points + wrap objects) and the joint
definition matter** — moment arms are a kinematic property of those, independent of the
muscle force model. So the Millard-vs-Thelen distinction is irrelevant here (it is a G2
concern; see recon risk R5).

## Source & license (the reason for THIS copy)

- **Source:** `opensim-org/opensim-core`, path
  `Bindings/Python/tutorials/resources/Tutorial 4/gait2392.osim`, branch `main`,
  fetched 2026-06-07.
- **License:** **Apache-2.0** (the opensim-core repo license). `LICENSE.txt` and `NOTICE`
  in this directory are copied verbatim from opensim-core.
- **Why this copy and not the canonical generic model:** the canonical *generic* template
  `Models/Gait2392_Simbody/gait2392_thelen2003muscle.osim` (Model `3DGaitModel2392`,
  Thelen muscles, Version 40000) lives in `opensim-org/opensim-models`, which ships **no
  LICENSE file** — legally ambiguous to vendor. The opensim-core copy is Apache-2.0 and
  structurally the same gait2392, so it is the clean-licensing choice. If a later phase
  wants the exact generic Thelen template and accepts its licensing, it is a drop-in swap
  (same converter); flag it for the user first.

## Note for the spike — the knee is NOT a pure hinge

`knee_r` is a `<CustomJoint>` whose single coordinate `knee_angle_r`
(range **−2.0943951 … 0.17453293 rad = −120° … +10°** flexion) drives, via `<SimmSpline>`
functions, **coupled tibiofemoral translations** (`translation1`, `translation2`) in
addition to the flexion rotation. This is the moving-instant-center coupling that the
locked decision D1 (knee-as-1-DOF-hinge at G1) deliberately approximates away.

→ S0 must (a) extract `knee_angle_r`'s flexion axis, and (b) **measure the moment-arm error
introduced by dropping the coupled translations** — this is exactly recon **O1/R3**, and it
sets the honest validated-ROM bound (target 0–100°, flag 100–120°).

## Also vendored — the real-OpenSim moment-arm reference

`knee_moment_arms_opensim.json` (+ generator `gen_moment_arms.py`) — the four target
muscles' knee moment arms over 0…−100° flexion, computed by **real OpenSim 4.6** (the
`opensim` PyPI wheel) on this exact model. This is the **independent anchor** for the
cf-osim oracle: `tools/cf-osim/tests/opensim_cross_check.rs` grades our re-derivation
against it and requires sub-2 mm RMSE (actual: ~0.3 mm). Committed so the cross-check
runs in CI with no OpenSim install; regenerate with `gen_moment_arms.py` (header has the
`uv` commands). Both files are derived from the Apache-2.0 model via Apache-2.0 OpenSim.

## Not vendored (yet)

The `.osim` references bone display meshes (`Geometry/*.vtp`) by name. They are **not**
needed for S0 (moment arms don't require bone meshes) and are **not** vendored here. The
S4 skin-envelope check uses the *scan* SDF, with simplified bone capsules — so geometry
meshes stay un-vendored unless a later phase needs them, at which point fetch from the same
Apache-2.0 source.
