# Settled in this spec

Each line: the call, why, and where the appendix records what would differ if it is wrong. A row a later answer replaced says so and gives the replacement; `C1…C11` are the conflict resolutions in the header of 20-rigid-physics.

| # | question | call | why |
|---|---|---|---|
| 2 | `MjcfError` shape | A3 §1's enum: `#[non_exhaustive]`, every variant struct-shaped and `#[non_exhaustive]`, `String` names, `Location` opaque, `Io{path,source}`, `Unsupported{feature}` reserved for stated limitations; plus `#[cfg(feature="mjb")] MjbTooLarge{size, limit}` | nothing outside sim-mjcf constructs it (A3 §1); A4 §10's names map onto it (30-error-type) |
| E2 | locations | message only: line + element path at parse, element + body at build (A3 §2) | no field on `Mjcf*` structs |
| 4 | S11 | 14 actuator + 2 sensor fields become `Option<T>` (`Prefix<N>`/`TfAuto` where partial); sensor defaults stay on the allowlist | A3 §3 |
| 5 | joints/freejoint/inertial directly in `<frame>` | refused, `Unsupported` (limitation); if ever implemented, MuJoCo ignores the frame's transform for `<inertial>` | A5 §3.7 |
| 6 | `energy_initial` | captured once per reset, crate-private flag cleared by `reset` | A1 §5 |
| 7 | Data/Model shape | `StepError::DataShapeMismatch{field, expected, actual}` over **24** lengths on `step`, `step1`, `step2`, `forward`, `forward_skip`; `inverse` documented, unchanged; `integrate` returns `Result` (C3 replaces this row's "documented, unchanged" for it) | 5 lengths let a geom-count mismatch through (A1 §1) |
| 8 | derived caches | additive `Model::recompute_derived()` (MuJoCo's `mj_setConst` analogue), trees computed in sim-core, factories call it; geom bounding radii and fixed tendon lengths move into it; cf-design's tree copy switches to it | one derivation for every producer; fixes P-L31 (A1 §7) |
| 9 | plugins | RK4 advances plugins; `try_make_data` + `MakeDataError`; `make_data` runs plugin `reset` (MuJoCo: `mj_initPlugin` then `mj_resetData`); no `Plugin::copy_data` | A1 §6 |
| 10 | per-env BatchSim | `try_new_per_env -> Result<_, PerEnvError<_>>` + panicking twin with `# Panics`; `PerEnvStack` reduced to `install_on`; `model_of`, `forward_all`; `EnvBatch` and the unused `prototype` argument deleted | P-L19 precedent ("Add them all now"), A1 §8 shape C |
| 11 | `SimulationConfig`, `SolverConfig`, `Gravity` | deleted, with sim-mjcf `config.rs`, `ExtendedSolverConfig`, and sim-mjcf's sim-types dependency; `SimError` deleted if it has no consumer at implementation time (measure) | no functional consumer (A1 §9, A3 §5) |
| 12 | chaining callbacks | documented clone pattern (freezes the pub `Callback.0`) | A2 §6 |
| 13 | vertex-only mesh; `.mjb` | convex hull, as MuJoCo; `.mjb` decode size limit → `MjbTooLarge` | A5 §3.8 |
| 14 | L27 hull order | in scope; flex edges in MuJoCo 3.5.0's order; hull in our own deterministic order (MuJoCo's comes from qhull — listed as a limitation); `clippy::iter_over_hash_type` denied in cf-geometry and sim-mjcf; mesh-io STEP keys sorted too | A6 §1 |
| 15 | L30 | the corpus covers all 184 example files with MJCF; the Rigid PRs verify them (40-verification); whether CI loads them later stays open (ledger) | A6 §3 |
| 16 | depth | MuJoCo's limit (self-closing refused at depth 500, open/close at 499); the 6 recursive pub entry points run on a 64 MiB thread; a 4096-deep guard for code-built trees; `all_bodies` iterative | 20.7 MB stack needed at opt-0 (A4 §1.4) |
| A2-Q1 | FD refusal shape | `StepError::UnsupportedIntegrator{integrator}` | derivative fns already return `StepError` |
| A2-Q2 | FD with history | panic → `Err` | P-L19 precedent |
| A3-O1 | actuator default cross-talk | refuse an element whose class actuator default came from a different shortcut kind (limitation; 0 corpus docs) | A3 §7 |
| A3-O2 | repeated top-level defaults | MuJoCo's snapshot-at-creation | parity |
| A3-O3 | `DefaultResolver` | crate-private | re-export later is additive |
| A3-O4 | negative muscle parameters | refused (MuJoCo silently ignores them), except `force`, refused only when the class sets `force ≥ 0` | rule: "silently other than asked" |
| A3-O5 | cylinder `bias[1..3]` | refused when non-zero (MuJoCo reads 3 values into one double) | rule: stricter |
| A4-Q1 | flex `<vertex>`/`<element>` children | replaced by C7: MuJoCo's meaning — `<flex body=…>` names existing bodies, in-tree flex docs are rewritten to declare their vertex bodies, our child form is refused | Jon, third round |
| A4-Q2 | `muscle@actearly` | allowlist | implemented, 1 use |
| A4-Q3 | `<deformable><flexcomp>` | allowlist (path differs from MuJoCo's body-level `flexcomp`) | 3 docs |
| A4-Q4 | overlay ↔ parser sync | read-tracking assertion in `cfg(test)` | otherwise an accepted attribute can silently go unread again |
| A4-Q5 | `MjcfBody` drop recursion | measure drop alone first; iterative `Drop` if > 512 KB | A4 §11 |
| A4-Q6 | hex floats | refused, limitation (0 uses) | MuJoCo accepts them via `strtod` |
| A4-Q7 | `freejoint damping` (3 examples) | removed from the docs (it was never read; behaviour unchanged) | dead attribute |
| A4-Q9 | weld `site1`/`site2` | refused, limitation (0 uses) | orientation part not worked through |
| A5-Q1 | rotation fit | moot: replaced by C6, MuJoCo's `eig3` in plain arithmetic (C1) | A10 §2.4 |
| A5-Q2 | automatic `lo == hi` | refused with lo > hi | MuJoCo treats it as silently unlimited |
| A5-Q3 | stored range of an unlimited joint | MuJoCo's stored values | behaviour-free; check `traj_fp` unchanged |
| A5-Q4 | `lengthrange` | honoured | MuJoCo keeps an explicit one |
| A5-Q5 | `actuatorfrcrange`/`actuatorfrclimited` | refused, limitation (0 in-tree uses) | needs new Model fields |
| A5-Q6 | muscle `actlimited` default | parity for MuJoCo `<muscle>`; forced (0,1) kept for our Hill/Millard extensions, documented | A5 §4.2 |
| A5-Q7 | URDF inertia triangle | refused, as MuJoCo's own URDF importer; fix the two example inertias | parity |
| A5-Q9 | caller-built and `.mjb` input | builder guards at every hang/panic point + docs that the typed API does not re-check every field | an exhaustive walk misses new fields |
| A5-Q10 | repeated `<pair>` | both kept (parity) | flips `builder/contact.rs:261` |
| A6-Q2 | flex element length ≠ dim+1 | refused | MuJoCo `user_mesh.cc:4108-4116` |
| A6-Q4 | Model's hash fields | left | no byte-comparing consumer |
| flex `density` | read nowhere today (71 docs set it, 44 to ≠ 1000) | removed from the docs, then refused by the schema | MuJoCo's `<flex>` has no `density`; dead |
| A1 open | `energy` flag `pub` | `pub(crate)` | A1 §5 |
