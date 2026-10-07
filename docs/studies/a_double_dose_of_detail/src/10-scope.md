# Scope

An outside user reviewed the published 0.9.0 crates; the rows below are theirs (triaged), plus items found while planning (P-rows).

| row | kind | what |
|---|---|---|
| core-C1 | docs | callback call counts undocumented (and they differ from MuJoCo — A2 §1) |
| core-C2 | docs | springs+dampers off ⇒ the passive callback never runs |
| core-C3 | bug→parity | `step1` runs the control callback with actuation disabled (MuJoCo does too) |
| core-C4 | API | one slot per callback |
| core-L1 | bug | `energy_initial` never set by `forward` |
| core-L2 | bug | `reset_to_keyframe` is partial |
| core-L3 | API | a Data/Model mismatch panics |
| core-L4 | API | bad state auto-resets (parity, stays); the bad-ctrl check differs from MuJoCo |
| core-L5 | API | Model edits don't update derived caches |
| core-L6 | bug | implicitspringdamper: `forward()` changes qvel |
| core-L7 | bug | plugin data lost on clone; `make_data` panics; RK4 skips plugin advance |
| core-B1 | docs | `step_all` determinism fails with a stateful callback |
| core-B2 | docs | `BatchSim::reset` doc wrong |
| core-B3 | API | `model()` = env 0; no `forward_all`; no shape check |
| core-D1…D4 | docs/API | "immutable Model", "no allocation", 3 small errors; `SimulationConfig` never read |
| mjcf-H1 | bug | NaN/inf geom mass/density/fullinertia hangs `load_model` |
| mjcf-H2 | bug | deep nesting overflows the stack |
| mjcf-H3 | bug | ball then hinge on one body panics in `load_model` |
| mjcf-H4 | bug | ctrlrange lo > hi loads, first step panics |
| mjcf-S1 | bug | typos and unknown elements load silently; `intvelocity` missing |
| mjcf-S2 | bug | unparseable values fall back to defaults |
| mjcf-S3 | bug | default geom `size` not inherited |
| mjcf-S4 | bug | self-closing `<body/>` dropped |
| mjcf-S5 | bug | a 2nd `<compiler>`/`<option>` replaces the first |
| mjcf-S6 | bug | joint/ctrl ranges unchecked |
| mjcf-S7 | bug | impossible sizes, masses, inertias load |
| mjcf-S8 | bug | geometry attributes misread |
| mjcf-S9 | bug | NaN body pos loads; zero axis → Z |
| mjcf-S10 | bug | undefined class loads; "main" unknown; top-level defaults overwrite |
| mjcf-S11 | bug | an explicit value equal to the default is overwritten by the class |
| mjcf-S12 | bug | duplicate names: last wins |
| mjcf-S13 | bug | joints under `<frame>` invisible to validation |
| mjcf-S14 | bug | `<include` inside a comment refused |
| mjcf-S15 | API | an unnamed mesh gets `""` |
| mjcf-E1, E2 | API | errors flattened to `Unsupported`; no location |
| mjcf-D1, D2 | docs→code | vertex-only mesh; condim; empty `<contact/>`; `.mjb` size |
| P-L1 | bug | joint `limited` with no range gets ±π·deg2rad; `ctrllimited` with no range gets (−1, 1) |
| P-L19 | API | `new_per_env` panics undocumented |
| P-L24 | bug | a timestep ≤ 0 reaches `forward`/`step2`/thermostat |
| P-L25 | docs | SDF facts (single-threaded step, `sdf_maxcontact`, octree cell) |
| P-L26 | hang | a classless nested `<default>` loops forever |
| P-L27 | bug | model construction depends on hash order (71 of 1,478 corpus docs vary across processes) |
| P-L28 | compat | 200 of 253 MuJoCo submodule models fail to load |
| P-L31 | panic | `ENABLE_SLEEP` on a factory-built model panics; a ctrl-writing callback zeroes FD `B` |
| P-L32 | physics | any body with ≥ 2 joints diverges from MuJoCo (`qfrc_bias`); cause isolated and fix measured (21-multi-joint-bias, A12); fixed in Rigid-physics (P24) |
| P-L33 | bug | hybrid sensor derivatives read stale caches (wrong on `main`, CI green) |
| P-L34 | bug | actuator/sensor delay is parsed but never applied |
| P-L35 | hang/bug | `quat="0 0 0 0"` hangs; `MjcfModel::all_bodies()` skips levels |

Also fixed because the rules need it (A3 §4, A5 §4): defaults ignore `<frame>` composition; `discardvisual`/`fusestatic` read unresolved classes; composite elements take the user's defaults; the composite `curve` keyword map is wrong (`"s"` builds zero-length cables); sim-urdf emits `<inertial>` without `pos` (28 of 37 in-tree URDFs) and a `world` link as a duplicate body.
