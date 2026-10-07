# Verification

A6 §4 is the protocol; the rules it enforces are below. `P…`/`L…` are the commits of 20-rigid-physics and 22-rigid-loading.

## Before implementation

- **Baseline on `main`**: corpus harness ×10 (the determinism mask, 71 docs), repo and submodule `.xml` (streaming-fingerprint harness only, one process per file under the watchdog of *Per commit*; a `{:#?}` fingerprint of `fourier_n1` is 2.39 GB of text), MuJoCo 3.5.0 oracle, `cargo xtask licensed-gates --run` (meshes fetched and verified per `design/cf-fsu-geometry/BODYPARTS3D.md`; the set of red gates at `main` is the baseline), the 24 ignored golden-flag tests' residuals, `--features mjb`, `--no-default-features`, the full validator fleet.
- **A home for the tooling** (T17): `sim/L0/tests/scripts/` (A20 §2.1's location for the generator). Each tool exists only in `$SCRATCH` and lands with the first commit that runs it. P01: the oracle build script `build_mujoco_oracle.sh` (*The unfused oracle*), `gen_census_golden.py`, the corpus extractor, the corpus harness and its streaming `harness2` (A6 §9), and `watchdog.py` (*Per commit*); the census prototype lands as `layer_e_census.rs` in `mujoco_conformance`. L01, the first MJCF commit: `buckets.py` (*Per commit*, the five buckets). P42, the first commit whose goldens it computes (`mju_makeFrame`, A21 §1.3): A21's no-FMA C harness.
- **A pilot of the per-commit loop on P01–P03** (T17): it builds the oracle (*The unfused oracle*), then runs every *Per commit* step on the first three commits, with its wall time and peak RSS written down, and a written procedure for a commit whose actual changes differ from its pre-registered list.
- **Runtime-generated MJCF captured at the baseline** (T17; A6 §4(c)): the 157 `format!` templates and the runtime producers (cf-design, cf-mjcf-emit, sim-urdf, therm-env, cf-codesign, cf-fsu-model, cf-osim), captured at `main` in a detached scratch worktree with an uncommitted dump patch, then again at each PR's head; A6 §4(d) joins ours at `main`, ours at the head, and MuJoCo. The push guard `grep -c CF-SCRATCH-MJCF-DUMP` must print 0 on the PR diff, after being made to print ≥ 1 on the worktree.

## Per commit

- **MJCF commits**: the five buckets (unchanged · ok→ok changed must be on the commit's pre-registered expected-change list · ok→err must map to the commit's rule · err→ok must be MuJoCo-ok · err→err message change), and NEW NONDETERMINISTIC empty from L02 (flex edges in MuJoCo's order) on. Compare MuJoCo by status, not message (its message varies for 2 docs).
- **Every rule**: its must-fail test made to fail at the parent, written down before the code.
- **A must-fail whose parent side hangs or grows** (T13). `timeout` bounds time, not memory, so such a test is not run at the parent under `timeout` alone. Either its failure at the parent is **established by reading** — the commit entry says so and cites the loop or the allocation — or it runs **one process per input file under an RSS watchdog that kills the process group at 2.5 GB, polling every ≤ 100 ms** (A6 §2 and §4 conventions: `watchdog.py 2500 <secs> 20`, a 20 ms poll, made to fail once before use). The cases the stress test named: L16's two ledger-L26 shapes (`defaults.rs:779-785` pushes without bound), L03's `hang_is_an_error_typed_model` and `mass_overflow_is_an_error`, L36's `stl_nonfinite_vertex_is_error`, L05's zero quaternion, L27's claimed `.mjb` length, L48's `nsample` (≈ 0.27 GB per actuator if the parent builds `Data`, by arithmetic from A7 §1.1). The same watchdog covers anything that loads submodule models, the MuJoCo oracle included: A6 §4(d) batches 100 corpus docs per process, while MuJoCo alone peaks at 1,555 MB on `fourier_n1` (A6 §2).
- **Oracle**: every new refusal is MuJoCo-refused or on the divergences list; no MuJoCo-ok doc newly fails unless listed.

## The census residuals

The decision is to fix every census cluster (01-decisions, second round). Each cluster below is closed only in part by its commit; the table gives each residual doc an owner commit (T16): the later of the commit whose code isolates it and the commit after which it is the doc's first difference. Counts are what each section measured on its own base — A21's census base holds A15 and the ledger-L32/L44a/L44b/L44c prototypes, not A18's solver port (P34), A19's fixes (P27, P28) or the sleep series (P20–P23) — and were **not measured as a series**. "Isolate" means the owner commit re-measures the doc and isolates its first differing quantity.

| cluster | residual | docs | first difference (where measured) | owner |
|---|---|---|---|---|
| NEW-FRAME (28) | 10 after P42's 18 | `0f322379`, `b26a81b9`, `c8902cf8` | one of 5 equally deep box–box corners; MuJoCo's C with FMA reproduces the wheel, without FMA reproduces our port (A21 §9) | P46 |
| | | `53df39ca`, `a1d4ffc7`, `d4ef2aa3`, `eb97180b` | solver termination: agree ≤ 7.9e-11 with `iterations=1000 tolerance=1e-15` on both sides (A21 §9) | isolate at P42 |
| | | `f935e97a` | t0 `qfrc_constraint`, contacts equal; a tight solver leaves 1.2e-9; not isolated further (A21 §9) | isolate at P42 |
| | | `1efe8000`, `eaace9f8` | MuJoCo's broadphase culls touching or barely overlapping pairs; the comparison that drops them is not isolated (A21 §9; Q69: parity) | P43a |
| census L41 (14) | 10 after P43's 3 and P46's 1 | `aeb76329`, `faa6b270` | t0 `qfrc_constraint`, contacts equal; a tight solver leaves 6.6e-8 / 4.3e-6 (A21 §9) | isolate at P43 |
| | | `05b1f2d2`, `31833131`, `140b7da3` | MuJoCo's broadphase culls touching or barely overlapping pairs (as above) | P43a |
| | | `a6e04a3c`, `d72ea476`, `a2f14cb1` | the sleep cluster's signature, init-asleep forward; not isolated (A21 §9) | isolate at P43 |
| | | `46a5290e`, `5dfbbb75` | ≤ 4e-5 at steps 25 / 47; a tight solver brings `5dfbbb75` to 6e-10, `46a5290e` to 3.3e-8 (A21 §9) | isolate at P43 |
| census L41list (30) | 1 after P43's 29 | `af3775ea` | accelerometer `sensordata` (A21 §9); P27 removes its sensor difference alone (A19 §1.6); P27 with P43 not measured | isolate at P43 |
| NEW-DEGEN (18) | 2 after P44's 14 and P45's 2 | `45d3e0ec` | parallel capsule–cylinder: convex-solver precision through our EPA (A21 §9, Q3); not measured through P48's port | isolate at P50 |
| | | `d985f32c` | equality rows between bodies without dofs (A21 §9); not among P36's 5 measured docs (A18 §7) | isolate at P45 |
| NEW-PAIRCOUNT (2) | 1 after P45 | `f8879911` | solver termination; agrees with `iterations=1000 tolerance=1e-15` on both sides (A21 §9) | isolate at P45 |
| Newton (9) | 3 after P34's 6 | `a0ff89f4` | box–plane corners at the late-onset step; agrees with the box–plane rule (A18 §2.5) | P43 |
| | | `c31a6ac0` | the plane–capsule axis hint (A18 §2.5); not measured with P42 | isolate at P42 |
| | | `87d99b20` | step 29; no formulation difference found; in MuJoCo itself a 1-ulp change of one `qpos` entry moves the plane contacts by 1.0e-12 (A18 §2.5, Q6) | isolate at P34 |
| CG (2) | 1 after P34 | `60c23753` | line-search evaluation count; emulating FMA gives MuJoCo's 51, and the trajectory still first differs at step 32 (A18 §3.3, §3.4) | isolate at P34 |
| ACCEL (8) | 1 after P27's 7 | `809fc2ad` | MuJoCo reads 0 on a world-welded body (A19 §1.4); Q43: keep ours under the wrong-value rule, with its test | P27 (`divergence=` row) |
| late onset (6) | 1 after P43's 5 | `28695e44` | box–box contact normals differ by 3.0e-3 (A18 §5.4; U17) | isolate at P46 |

The clusters are R3's list of partly closed clusters (*Stress test, round 1*, T16), plus census L41list (A21 §9) and CG (A16 §2, A18 §3.3). Other clusters' residuals were not swept for this table.

## The unfused oracle

MuJoCo 3.5.0's arm64 wheel fuses multiply-adds (A18 §3.4: 8,901 fused instructions in its arm64 slice). C1 says our arithmetic is plain, so the goldens come from an unfused build.

**The oracle is MuJoCo 3.5.0 and its Python bindings, built from the 3.5.0 source tag with `-ffp-contract=off` by a script P01 commits** (`sim/L0/tests/scripts/build_mujoco_oracle.sh`) — an offline tool, not a crate; `gen_census_golden.py` runs on those bindings. Measured on arm64 (50-stress-test, *The oracle measurement*): built with contraction on, it equals the wheel on every case compared; built with it off, it equals the plain Rust port of `mjuu_eig3` on 20,211 of 20,211 tensors, and differs from the wheel on 1,153 of 4,400 poses, 1,664 of 3,000 frames and 19,960 of 20,211 tensors (tensors: max 4.13e-12).

MuJoCo's x86_64 builds contain no fused instructions (0 counted in the Linux manylinux `.so`, the macOS x86_64 wheel, and the x86_64 slice of the arm64 wheel), but none was run: whether one equals the unfused build is not measured, so the goldens are not taken from them.

Not built yet: the whole library with contraction off (the XML parser and the C++ model compiler included); the T17 pilot builds it. Not seen: the AVX paths (`mjUSEAVX`) that a whole step takes on x86_64.

## Before push

Grade every touched crate and its downstream (`RAYON_NUM_THREADS=1`), clippy exactly as the hook runs it, doc-theft, licensed gates at the head, the review rounds.
