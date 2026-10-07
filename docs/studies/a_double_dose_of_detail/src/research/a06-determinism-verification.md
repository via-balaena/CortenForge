> Research for *A Double Dose of Detail*, written during planning by a read-only researcher at `3520544e`. Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid spec — determinism (ledger-L27), the fourier_n1 RAM hazard (ledger-L29), example coverage (ledger-L30), the verification plan, and the merged flip table

Researcher section, drafted 2026-10-06 against `main` @ `3520544e` (clean before and after; see the last section).
Scratch for everything below: `SCRATCH/rigid_spec/determinism_verification/` (written `DV/` from here on).
I did not edit the repo. Every probe ran in a `git archive HEAD` copy (`DV/repo`, `DV/repo_attrlog`) or in a probe crate with its own `target/`.

---

## 0. What this section decides, and what it leaves open

| item | decided here (with referent) | open (for the assembler / Jon) |
|---|---|---|
| ledger-L27 sites | **5 hash-order sites reach the Model**: `flex.rs:431`, `flex.rs:483`, `convex_hull.rs:284-290`, `:529`, `:616`. Two more are outside the Model path (`sleep.rs:381`, `mesh-io step.rs:103`) — §1.1 | none for the five |
| replacement order | flex edges: **MuJoCo 3.5.0's own order**. My scratch fix reproduces MuJoCo's `flex_edge` on 66 of the 67 flexes MuJoCo loads, and its interior `flex_edgeflap` on 103 of 103 edges (§1.4). Hinges: edge order. Hull: our own deterministic order (MuJoCo's comes from qhull, `user_mesh.cc:1861-1900`, which we cannot use: it is C) | the dim-3 tet re-orientation that makes the 67th flex differ (`user_mesh.cc:4272-4293`) — §1.10 |
| proof | harness ×10 over all 1,584 docs, before the fix and after it: 71 nondeterministic → 0. The 1,513 stable docs are unchanged. The 71 changed docs are listed. Submodule models ×5: 18 nondeterministic → 0 (§1.4) | — |
| gate | `#![deny(clippy::iter_over_hash_type)]` in cf-geometry and sim-mjcf. I made it fail on the original code (4 sites) and pass on the fix. **It cannot see non-`for` iteration**: it misses `convex_hull.rs:288` (§1.9) | whether to adopt it (recommended) |
| ledger-L29 | **The harness caused it, not the loader.** The loader peaks at 310 MB (watchdog). The model's `{:#?}` text is 2.39 GB / 59.5 M lines, and the harness kept three copies of it. A streaming fingerprint runs `fourier_n1` at 309 MB peak (§2) | — |
| ledger-L30 | **The corpus covers every in-tree file under `examples/**`, `sim/L1/bevy/examples` and `tools/cf-codesign/examples` that contains `<mujoco`.** That is 184 files: 278 static docs, all of which load. It also lists 27 `format!` templates there, which only runtime capture can see; 9 of those are in code no CI job runs (§3) | CI coverage going forward (deferred per decision 15) |
| flip table | 1,584 docs × ours × MuJoCo 3.5.0 × rule × CI reach (§5; full TSV `DV/fliptable.tsv`) | 6 refusal kinds with **no triage row** (§5.3) |
| new facts found on the way | `<flex density=…>` is **silently ignored** (measured: the model fingerprint is identical at density 1000 / 1 / absent). That contradicts "flex density = our extension" in the design. MuJoCo-native `<flex vertex= element=>` is ignored too. Boundary flaps differ from MuJoCo (§1.10, §5.2) | owners: parser / validation |

---

## 1. ledger-L27 — model construction must not depend on hash order

### 1.1 Now — every hash-order site on the MJCF build path

The build path is the six workspace crates that `cargo tree -p cortenforge-sim-mjcf -e normal --all-features` lists: sim-mjcf, sim-core, sim-types, cf-geometry, mesh-io and mesh-types.

**Method (two static sweeps plus one runtime sweep; each was checked against a known positive):**
1. `cargo clippy -p <the six> --lib --all-features -- -W clippy::iter_over_hash_type` (log `DV/iter_over_hash2.log`).
   My first run used `-A warnings`, and that silenced the lint: 0 hits. The rerun without it flags `flex.rs:431`, which is the known positive. **The lint covers `for` loops only.**
2. `DV/scripts/hash_iter_scan.py`, a heuristic. It collects identifiers bound to a `Hash{Map,Set}` in each file and flags `.iter() / .keys() / .values() / .into_iter() / .drain()` on them. It also flags `.collect::<Hash…>()` followed by an iterator call. This sweep found `convex_hull.rs:288`, which the lint missed.
3. Hash-typed struct fields: `git grep` for `HashMap|HashSet` in the five non-mjcf crates. Their iteration was checked workspace-wide.

| # | site | what iterates | reaches | evidence | fix |
|---|---|---|---|---|---|
| 1 | `sim/L0/mjcf/src/builder/flex.rs:431` `for &(a, b) in edge_set.keys()` | `HashMap<(usize,usize),bool>` | `flexedge_vert/length0/flexid/rigid/flap`, `flex_bending` slots, `flexedge_J_*` (all in edge order) | lint; harness (§1.4) | MuJoCo first-appearance order |
| 2 | `flex.rs:483` `for ((ve0, ve1), elems) in &edge_elements` | `HashMap<(usize,usize),Vec<usize>>` | `flexhinge_vert/angle0/flexid` order; force summation order in `forward/passive.rs:759` | lint; harness: trajectory varies for 47 docs | hinges in edge order |
| 3 | `design/cf-geometry/src/convex_hull.rs:284-290` `.collect::<HashSet<_>>().into_iter().collect()` | extremal candidates | `most_distant_pair` (`:309-325`) keeps the first of equally distant pairs (strict `>`), so ties pick a different initial simplex | scanner only (the lint misses it) | `sort_unstable` + `dedup` |
| 4 | `convex_hull.rs:529` `for &fi in &visible` | `HashSet<usize>` from `find_horizon` | horizon edge order → which edge `order_horizon_edges` (`:542`) starts from → cone-face order → `ConvexHull.faces/normals` order | lint | `visible` becomes a `Vec` in BFS order |
| 5 | `convex_hull.rs:616` `for &fi in &visible` | same set | orphan-point order → conflict lists → `farthest_idx` ties (strict `>`, `:684`) → which point is added → **the hull's vertex count** | lint; on `fourier_n1/n1.xml` the hull vertex total was 22520 / 22521 ×5 / 22522 / 22523 over 8 processes on main, and 22521 ×8 with the fix | same |
| 6 | `sim/L0/core/src/island/sleep.rs:381` `for trees in init_groups.values()` | `HashMap<usize,Vec<usize>>` | `Data` at reset, not the Model. By reading: `sleep_trees` (`:175-212`) writes only the trees of its own group, the groups are disjoint, and `trees` is pushed in ascending order (`:375-379`), so the order of groups cannot change any value | lint | none needed. Optionally use `BTreeMap` so a workspace-wide gate stays clean |
| 7 | `mesh/mesh-io/src/step.rs:103` `for shell_holder in table.shell.values()` | truck's `HashMap` | STEP mesh vertex/face order, **only if a consumer enables `mesh-io/step`**. No in-tree crate does (`git grep '"step"' -- '*Cargo.toml'` hits only mesh-io's docs.rs features line `:22`). sim-mjcf's `load_mesh_file` dispatches by extension (`builder/mesh.rs:129-140`), so `<mesh file="x.step">` would load under the feature | lint | open (Q3) |
| 8 | `Model`'s 11 `*_name_to_id: HashMap` + `contact_pair_set`, `contact_excludes: HashSet` (`sim/L0/core/src/types/model.rs:980-1012`) | lookups | their order is visible only through `Debug` and `serde`. The one in-tree iteration is order-free: `sim/L0/tests/integration/keyframes.rs:1076` `.values().any(..)` | grep | none (Q4) |

Membership-only hash sets, which no sweep flagged: `builder/compiler.rs:18,50,65`, `include.rs:27`, `parser/extension.rs:96`, `defaults.rs:50,754`, `validation.rs:160-489`, `convex_hull.rs:469`. Also the pub `MjcfFlex::weights_by_vertex() -> HashMap` (`types.rs:3821`), which has no in-tree caller.

**Producers of runtime MJCF**, swept the same way (`DV/iter_over_hash_producers.log`; crates sim-urdf, cf-mjcf-emit, cf-design, cf-osim, cf-codesign, cf-fsu-model, sim-therm-env):
- The lint flags `cf-design` `adaptive_dc.rs:359` (a max, order-free) and `mechanism/validate.rs:364` (warning order only).
- It flags `cf-design` `simplify.rs:265` (insert into a set) and `:458` (sorted at `:465`).
- It flags `cf-design` `simplify.rs:296`, which pushes `HashSet` edges into a `BinaryHeap`. **Whether equal-cost ties then pop in insertion order is not examined.**
- It flags `sim-soft` at three sites, but sim-soft is not on the MJCF path.
- The scanner finds only sorted-after or membership uses there, plus `cf-fsu-model lib.rs:1169` (`band.into_iter().collect()`, not MJCF).
- These are outside L27's scope as decided. They are listed so nobody re-derives them.

### 1.2 Target

- **Flex edges:** MuJoCo 3.5.0's order.
  - `mjCFlex::Compile` numbers edges by first appearance: elements in order, then each element's local edges in the `eledge[dim-1]` order (`src/user/user_mesh.cc:4297-4321`). The edge pair is stored as `(min, max)` (`:4307-4309`).
  - The local edge table is cable `(0,1)`; triangle `(1,2) (2,0) (0,1)`; tet `(0,1) (1,2) (2,0) (2,3) (0,3) (1,3)` (`user_mesh.cc:3447-3452`).
  - `flex_edge` is copied in that order (`user_model.cc:3501-3503`).
- **Flaps / hinges:** MuJoCo's `CreateFlapStencil` (`user_mesh.cc:3667-3719`) walks the same order. A flap's opposite vertices are [the one in the first triangle containing the edge, the one in the second].
  - We have no hinge array in MuJoCo terms. Our `flexhinge_*` is ours, and it goes in **edge order**, i.e. the interior edges in `flex_edge` order, so hinge *k* sits beside its edge's flap.
- **Convex hull:** a deterministic order of our own.
  - MuJoCo's hull is qhull's (`user_mesh.cc:1861` `MakeGraph`, `:1866` `"qhull Qt"`), so its face/vertex order is qhull's.
  - Matching it means porting qhull. Using qhull itself is a C dependency, which the no-C/C++ rule forbids.
  - ⇒ **a stated limitation, order only.** It belongs in the `MUJOCO_CONFORMANCE.md:321` divergences table: "hull vertex/face order is Quickhull-BFS, not qhull's".

### 1.3 Change

The scratch implementation is `DV/l27_probe.patch` (207 lines). `git apply --check` against HEAD passes.

**cf-geometry, `design/cf-geometry/src/convex_hull.rs`:** no public signature changes. `pub fn convex_hull(points: &[Point3<f64>], max_vertices: Option<usize>) -> Option<ConvexHull>` is unchanged; its output order becomes deterministic.
```rust
// find_initial_simplex, replacing :284-290
let mut candidates: Vec<usize> = min_idx.iter().chain(max_idx.iter()).copied().collect();
candidates.sort_unstable();
candidates.dedup();

// private; was -> (HashSet<usize>, Vec<(usize, usize, usize)>)
fn find_horizon(faces: &[Face], eye: &Point3<f64>, start_face: usize, epsilon: f64)
    -> (Vec<usize>, Vec<(usize, usize, usize)>)
// BFS with `is_visible: Vec<bool>` + `visible: Vec<usize>` (push order = BFS order).
// The horizon test becomes `ni == usize::MAX || !is_visible[ni]`.
// The original `!visible.contains(&usize::MAX)` was true, so this keeps behaviour.
```
`use std::collections::{HashSet, VecDeque}` becomes `HashSet`. Optionally, `init_conflict_graph`'s `simplex_set` (`:469`) becomes `simplex.contains(&pi)`; that is membership only and changes no output.

**sim-mjcf, `sim/L0/mjcf/src/builder/flex.rs`:**
```rust
/// MuJoCo `eledge` (user_mesh.cc:3447-3452 at 3.5.0); edges numbered by first appearance (:4297-4321).
fn local_edges(dim: usize) -> Option<&'static [(usize, usize)]> {
    match dim { 1 => Some(&[(0, 1)]),
                2 => Some(&[(1, 2), (2, 0), (0, 1)]),
                3 => Some(&[(0, 1), (1, 2), (2, 0), (2, 3), (0, 3), (1, 3)]),
                _ => None }
}
fn extract_flex_edges(&mut self, flex: &MjcfFlex, vert_start: usize, flex_id: usize)
    -> Result<(usize, HashMap<(usize, usize), usize>), ModelConversionError>   // was infallible
```
- One loop pushes each edge on first appearance. `edge_key_to_global` stays a `HashMap`, used **only for lookup**.
- `extract_flex_bending_hinges` builds `edge_elements: Vec<Vec<usize>>`, indexed by the local edge index, and iterates it in edge order. Each hinge's `(ve0, ve1)` is read back from `flexedge_vert`.
- The refusal: an element whose length ≠ `dim + 1`, or `dim ∉ 1..=3`, returns `ModelConversionError`. It cites `user_mesh.cc:4108-4116` ("dim must be 1, 2 or 3", "elem size must be multiple of (dim+1)").
  - The parser only produces `dim+1`-length chunks (`parser/deformable.rs:124-133`), so this fires only for `dim ∉ 1..=3`, or for a caller-built `MjcfFlex` (its `elements` field is pub, `types.rs:4043`).
  - The corpus uses `dim` 1, 2 and 3 only (`grep`: 32 / 56 / 1), so it flips no static doc.
  - The scratch probe returned an empty slice instead. That was acceptable for the corpus, not for the PR. Q2 records the alternative.

**Gate:** add `clippy::iter_over_hash_type` to the existing `#![deny(...)]` lines: `design/cf-geometry/src/lib.rs:23` and `sim/L0/mjcf/src/lib.rs:149` (§1.9).

### 1.4 Proof (measured)

All harness runs used the planner's corpus harness, rebuilt in `DV/harness`. One process per pass; the corpus is 1,584 static docs.

| run | model-fp varies | traj-fp varies | status/err varies |
|---|---|---|---|
| planner, main ×10 (`rigid_plan_tests/emb_{1..10}`) | 71 | 47 | 0 |
| mine, main copy ×10 (`DV/runs/before_*`) | **71** (same set; the 1,513 stable docs equal the planner's fingerprints) | 47 | 0 |
| hull fix only ×5 (`harness_hullonly`) | 60 (57 flex + 3 flexcomp) | 47 | 0 |
| flex fix only ×5 (`harness_flexonly`) | 10 (all vertex-only mesh) | 0 | 0 |
| **both fixes ×10** (`DV/runs/after_*`) | **0** | **0** | 0 |

- **Before → after buckets** (`DV/scripts/buckets.py`): 1,513 unchanged. 71 changed: their base value was nondeterministic; the new value equals one of the old values for 29 of them and is new for 42. No other bucket is non-empty.
- **Submodule models**, one process per file under the RSS watchdog:
  - On main ×5, 18 of the 51 loadable files vary in model fingerprint, and 3 also in trajectory (kinova `jaco_arm`, `gen3`, gen3 `scene`). With the fix ×5: 0.
  - With the streaming harness (§2) over all 253 files ×3, `fourier_n1` included: 0 vary, 0 killed, peak 313.5 MB.
- **Repo `.xml` (15):** 0 vary before or after; unchanged.
- **In one process** (`DV/l29probe*/src/bin/det.rs`, 50 repetitions each; main → fix):
  - `convex_hull` of an octahedron (6 points): 6 distinct results → 1. Cube corners: 3 → 1. A 3×3×3 grid: 3 → 1.
  - Loading the single-triangle flex doc: 6 → 1. Loading the single-tet doc: 48 → 1.
  - Loading 9 of the 11 vertex-only mesh docs: 2 → 1. The other 2 gave 1 → 1 (my in-process key omits hull vertex positions).
- **Parity with MuJoCo 3.5.0**, after the fix (`DV/scripts/mj_flexdump4.py` vs `our_flex_after.jsonl`):
  - Our extension `<flex><vertex/><element/>` syntax was converted to MuJoCo's attribute form, with one 3-slide-joint body per vertex. 55 of the 60 flex docs then load in MuJoCo.
  - **`flex_edge` order is identical for 13 of 13 dim-1 flexes and 53 of 53 dim-2 flexes.** The single dim-3 flex differs, because MuJoCo re-oriented its tet: MuJoCo `elem` `[0,2,1,3]`, ours `[0,1,2,3]`.
  - With `elastic2d="bend"` set, so that MuJoCo fills `flex_edgeflap` (`user_model.cc:3504-3510`), **interior flaps match on 103 of 103 edges**.
  - Boundary flaps differ on all 241 boundary edges: MuJoCo stores `[v, -1]`, ours `[-1, -1]`. That is not a determinism item (§1.10).
- **Hook check of the fix:** `cargo clippy -p {cortenforge-geometry, cortenforge-sim-mjcf} --all-targets --all-features -- -D warnings` exits 0 on the fixed copy, and `cargo fmt --check` exits 0 (read unpiped).

### 1.5 The expected-change list (each changes once, then stays fixed)

- **Embedded:** 71 docs = 72 source records, in `DV/l27_expected_change.tsv` (`source<TAB>ci<TAB>bucket`).
  - `sim/L0/tests/integration/flex_unified.rs` ×44, `flex_flex_collision.rs` ×12, `deformable_friction_dt25.rs` ×2, `collision_plane.rs` ×2 (1493, 1538), `runtime_flags.rs:2159`.
  - `sim/L0/mjcf/src/builder/build.rs:1447,1489`, `parser/tests.rs:1115`, `mjb.rs:609`.
  - 7 Bevy apps `examples/fundamentals/sim-cpu/mesh-collision/{mesh-box,mesh-capsule,mesh-cylinder,mesh-ellipsoid,mesh-on-mesh,mesh-on-plane,mesh-sphere}/src/main.rs`. No CI job runs these.
- **Submodules (CI never checks them out):**
  - dm_control: `mjcf/test_assets/{included,model_with_include}.xml` and `third_party/kinova/{jaco_arm,jaco_hand}.xml`.
  - menagerie: `agility_cassie/{cassie,scene}`, `arx_l5/{arx_l5,scene}`, `google_barkour_vb/{barkour_vb,barkour_vb_mjx,scene,scene_hfield_mjx,scene_mjx}`, `iit_softfoot/softfoot`, `kinova_gen3/{gen3,scene}`, `trs_so_arm100/{scene,so_arm100}`.
  - Plus `fourier_n1/{n1,scene}`: its hull vertex count varied on main; it was never fingerprinted on main, because the old harness could not run it.
- **What the expected-change list cannot see:** a doc whose variation is rare enough to be missed in 10 processes. 21 of the 71 showed only 2 distinct values in 10 runs. §1.9's static gate is the other half of the argument.

### 1.6 Tests to add (each fails on main, measured or computed)

1. **`design/cf-geometry/tests/convex_hull_tests.rs` — `hull_is_identical_across_repeated_calls`.**
   - Octahedron `(±1,0,0),(0,±1,0),(0,0,±1)`; cube corners; 3×3×3 grid. Call `convex_hull(&pts, None)` 50× and assert `faces` and `vertices` equal the first call.
   - On main: 6 / 3 / 3 distinct results in 50 calls (measured, `det.rs`). Fixed: 1 / 1 / 1.
2. **sim-mjcf — `flex_edges_follow_mujoco_order`.** Builder unit test, or `sim/L0/tests/integration/flex_unified.rs`. Expected values come from MuJoCo 3.5.0, measured with `DV/lie/pin_test.py`:
   - square, `elements "0 1 2  1 3 2"` (dim 2): `flexedge_vert` (minus `vert_start`) = `[[1,2],[0,2],[0,1],[2,3],[1,3]]`; `flexedge_flap[0] = [0,3]`; one hinge `[1,2,0,3]` (the hinge by reading, the rest measured).
   - cable `"0 1 1 2 2 3"`: `[[0,1],[1,2],[2,3]]`.
   - negatively oriented tet, vertices `0 0 0 / 0 1 0 / 1 0 0 / 0 0 1`: `[[0,1],[1,2],[0,2],[2,3],[0,3],[1,3]]`.
   - Our fixed loader produces exactly these (measured, `DV/flexdump_after`).
   - On main a single load matches the square with probability 1/120 (5! orders), so the test loads 20× and asserts every load. That makes a chance pass on main impossible in practice.
3. **sim-mjcf — `flex_build_is_repeatable`.** Load the single-triangle doc (`corpus/docs/44ed9033d6da7a3a.xml`, source `flex_unified.rs:18`) 20× in-process and assert identical `flexedge_vert` and `flexhinge_vert`. On main: 6 distinct in 50 (measured).
4. **The gate itself (§1.9).** It fails on the original code; this was measured.

### 1.7 Tests and examples that flip — none measured

I ran these on the fixed copy (`DV/test_after_*.log`), with the original code as baseline where noted:

| suite | CI job | result after the fix |
|---|---|---|
| `cargo test -p cortenforge-geometry` | tests-debug | 8 binaries, all ok (207 unit tests + 7 integration/doc binaries) |
| `cargo test -p cortenforge-sim-mjcf` | tests-debug | 385 lib + forward-conformance, ok |
| `cargo test -p cortenforge-sim-core` | tests-debug | 720 lib, ok |
| `cargo test -p sim-conformance-tests --release` | tests-release | 1,338 integration + 83 conformance, ok (27 ignored) |
| `cf-design-tests --test rung4_concave_contact --test rung4b_facet_contact` | **licensed gate, never in CI** | both ignored without the BodyParts3D meshes, **so not measured**. They call `convex_hull` directly (`rung4_concave_contact.rs:102`, `rung4b_facet_contact.rs:100`) ⇒ `licensed-gates --run --only cf-design-tests` is mandatory for this commit |
| validators `example-{composite,mesh-collision,raycasting,urdf}-stress-test` | validate-examples | all pass. **stdout byte-identical** between the original and fixed code (11/11, 21/21, 20/20, 35/35 checks) |

### 1.8 Downstream: cf-geometry's direct consumers of hull face/vertex order

cf-geometry is reached by 296 of the 314 workspace crates. Recomputed from `rigid_plan_tests/metadata.json` (all dependency kinds): 295 others + itself. sim-mjcf is reached by 223.

30 crates declare `cf-geometry` directly. Those that **read hull order**:
- **sim-core:**
  - `gjk_epa.rs:169-174`: support by hill-climbing over `hull.adjacency` from a warm start. Ties in the support value pick by vertex order.
  - `collision/hfield.rs:160`: a **runtime** hull of prism points on every hfield-vs-convex call. Hash order made this step-time nondeterministic too, by reading; none of the 3 hfield corpus docs varied in 100 steps.
  - `mesh.rs:206` stores the hull, serialized. `collision/mesh_collide.rs:164`, `collision/flex_narrow.rs:169` and `raycast.rs` read it.
- **sim-mjcf:** `builder/mesh.rs:57` computes it at build time. `builder/mesh.rs:786-788` computes inertia over `hull.faces`; face order is summation order, hence the model-fingerprint change.
- **sim-bevy:** `sim/L1/bevy/src/mesh.rs:82` renders `hull.vertices`. Render only.
- **cf-design-tests:** rung4/rung4b (licensed, above).
- **The `example-raycasting-stress-test` validator:** `main.rs:363`. Its output was identical (measured).
- **cf-geometry's own** `query/{closest_point,ray_cast,epa,gjk}.rs` and `support_map.rs`.

### 1.9 The regression gate, and its blind spot

- **What it is:** `clippy::iter_over_hash_type` added to the `#![deny]` at `cf-geometry/src/lib.rs:23` and `sim-mjcf/src/lib.rs:149`.
- **Made to fail** (`DV/gate_*.log`, the hook's exact clippy command):
  - Original `convex_hull.rs`: exit 101 at `:529` and `:616`.
  - Original `flex.rs` with fixed cf-geometry: exit 101 at `:431` and `:483`.
  - Both fixed: exit 0 for both crates.
- **Blind spot (measured):** the lint ignores iterator adaptors. It never flagged `convex_hull.rs:288`. Only the heuristic scanner and the runtime harness see that class.
  - So the PR's per-commit harness diff (§4b, "NEW NONDETERMINISTIC must be empty") remains the second gate.
- **Not proposed for sim-core:** `sleep.rs:381` is order-free by reading. Gating sim-core would need an `#[expect(…, reason)]` or a `BTreeMap` there (Q1).

### 1.10 Handed to other areas (found here, not determinism)

| finding | referent | owner |
|---|---|---|
| dim-3 tets: MuJoCo swaps v1/v2 when `dot(cross(v01,v02),v03) > 0` (`user_mesh.cc:4272-4293`); ours keeps the element and stores a signed `flexelem_volume0` (`flex.rs:116-126`). This is why 1 of the 67 flexes' edge order still differs | §1.4; `DV/lie/tet_pos.xml`: MuJoCo `elem [0,2,1,3]` | validation / flex. If adopted, it changes `flexelem_data`, volume signs and edge order for positively oriented tets ⇒ its own expected-change list |
| boundary flaps: MuJoCo `[v_opp, -1]`, ours `[-1, -1]` (241 of 241 boundary edges); MuJoCo fills flaps only when `elastic2d ∈ {1,3}` (`user_model.cc:3504-3510`), ours always | §1.4 | flex / bending |
| `<flex density=…>` is **ignored**: `parse_flex_attrs` (`parser/deformable.rs:220-260`) never reads it, despite its doc comment (`:219`). `MjcfFlex.density` stays 1000 (`types.rs:4086`). Measured: model fingerprint identical at density 1000 / 1 / absent, while changing `radius` changes it. 71 docs set it, **44 to a value ≠ 1000** (`1.0` ×24, `100` ×11, `500` ×4, `300` ×3, `0`, `0.1`) | `DV/lie/flex_density*.xml` | parser (mjcf-S1). Implementing it **changes the vertex masses of those 44 docs**; refusing it flips 71 docs (§5) |
| MuJoCo-native `<flex vertex="…" element="…">` attributes ignored (2 docs, `builder/mod.rs:1345,1379`) | attr probe (§5.1) | parser |
| STL vertices not deduplicated: `fourier_n1` ours 3,807,720 vertices for 1,269,240 faces (= 3×); MuJoCo 3.5.0 `nmeshvert` 629,958 for the same faces | `DV/scripts/mj_meshcount.py` | mesh (no row) |
| parser drops a trailing partial element and unparsable ints silently (`deformable.rs:120-133`); MuJoCo refuses (`user_mesh.cc:4115`) | reading | parser (mjcf-S2) |

### 1.11 Open questions (only ones the fixed decisions do not settle)

- **Q1. Adopt the lint gate?**
  - Options: (a) deny in cf-geometry and sim-mjcf; (b) also sim-core, with `sleep.rs:381` → `BTreeMap`; (c) none.
  - Recommendation: **(a)**.
  - If that is wrong: a future `for` over a hash in sim-core build code would reach the Model unflagged. The per-commit harness would still catch it if the corpus exercises it.
- **Q2. Element length ≠ dim+1, or dim ∉ 1..=3, in a caller-built `MjcfFlex`.**
  - Options: (a) refuse, citing `user_mesh.cc:4108-4116`; (b) fall back to all pairs i<j in lexicographic order.
  - Recommendation: **(a)**. It flips 0 static docs; runtime-generated flexes are not measured.
  - If that is wrong: a caller relying on 5-vertex elements gets an error instead of edges. No in-tree constructor builds such elements (`deformable.rs:552-598` emits 3 and 4).
- **Q3. `mesh-io` STEP iteration (site 7).**
  - Options: (a) sort `table.shell` keys in this PR; (b) leave it, because MuJoCo has no STEP meshes and a parity refusal of non-MuJoCo mesh formats may remove the path.
  - Recommendation: **(b)**, and raise it with the parser owner. It is unreachable in-tree.
  - If that is wrong: a downstream user with `mesh-io/step` gets nondeterministic STEP meshes until fixed.
- **Q4. `Model`'s hash fields.**
  - `Debug` / `serde` order varies. `.mjb` serializes `MjcfModel` (`mjb.rs:168-176`), not `Model`, so `.mjb` bytes are unaffected; not measured beyond reading.
  - Recommendation: **leave them**. The harness canonicalises map blocks.
  - If that is wrong: anyone byte-comparing a serde-serialized `Model` sees spurious diffs. In-tree there is no such consumer.

---

## 2. ledger-L29 — `fourier_n1`: the harness, not the loader

All runs below were under `DV/scripts/watchdog.py`: it SIGKILLs the process group above 2,500 MB, polling `ps -o pgid=,rss= -A` every 20 ms.
I made the watchdog lie once first: a process growing 50 MB / 0.1 s was killed at 315 MB against a 300 MB limit.

| run | peak RSS | time | result |
|---|---|---|---|
| `l29probe load n1.xml` (`load_model_from_file` only, main) | **310 MB** | 0.7 s | `nmesh=29`, 3,807,720 mesh vertices, 1,269,240 triangles, 22,521 hull vertices |
| `l29probe load scene.xml` | 309 MB | 0.7 s | same meshes |
| `l29probe dbgsize n1.xml` (`{:#?}` into a byte-counting `fmt::Write` sink, no allocation) | 310 MB | 12.4 s | **2,393,211,933 bytes, 59,485,699 lines** |
| planner's harness (`format!("{model:#?}")` → `Vec<&str>` → `canon` → `Vec<String>`) | 6.6 / 7.5 GB (planner) | 18 s | — |
| `harness2` (streaming fingerprint, below) | **309 MB** | 20.8 s | ok |

- **Arithmetic for the harness peak** (not measured directly):
  - the String: 2.39 GB;
  - `Vec<&str>`: 59.5 M × 16 B = 0.95 GB;
  - canon's owned copies: 2.39 GB of content + 59.5 M × 24 B headers = 1.43 GB.
  - Total ≈ 7.2 GB, against 7.5 GB observed.
- The same shape on `kinova_gen3/gen3.xml`: the loader peaked at 19 MB, the Debug text is 173 MB, and the planner's harness recorded 580 MB.
- ⇒ **Not a loader bug.** The loader's own footprint, 310 MB, is below MuJoCo 3.5.0's 1,555 MB on the same file (watchdog).
- **The fix is in the tool: `DV/harness/src/bin/harness2.rs`, a streaming fingerprint.**
  - It hashes lines as they are formatted, buffering only map/set blocks, which the canonicaliser sorts. Memory is bounded by the largest map block.
  - Checked against v1 on the 1,478 loadable docs: **the same partition** (1,423 distinct fingerprints each, a bijection; no other field differs).
  - Made to lie once: built against main, it reproduces the mask (68 docs vary in 5 runs).
  - Fingerprint values differ from v1's: v1 hashed `Vec<String>` with a length prefix. A baseline and a branch run must therefore use the same harness version.

---

## 3. ledger-L30 — does the corpus cover the never-loaded example MJCF?

Method: `DV/scripts/l30.py`.
- **Input set:** files containing `<mujoco` from `git grep -l '<mujoco' -- 'examples/**'` (162 files) plus `sim/L1/bevy/examples`, `sim/L1/sim-bevy-soft/examples`, `tools/cf-codesign/examples`, `**/examples/*.rs` (22 files).
- **Classification:** each file's crate comes from `metadata.json`. Each `<mujoco` occurrence was counted against the manifest records for that file.

| category | files | `<mujoco` occurrences | static docs in corpus (all load today) | `format!` templates (runtime only) | files missing from manifest |
|---|---|---|---|---|---|
| Bevy app `main()` | 137 | 137 | 132 | 5 | **0** |
| cargo example targets | 23 | 24 | 20 | 4 | **0** |
| validator crates (CI runs them) | 24 | 145 | 126 | 18 (+1 fragment) | **0** |

- ⇒ **Yes for static MJCF:** every in-tree example file carrying `<mujoco` is in `manifest.jsonl`, so the PR's harness verifies its static docs.
- My Bevy count is 137, against the ledger's 133. The two classifications were not reconciled: mine is "crate depends on a package named `*bevy*`".
- **Not covered: 9 templates in code no CI job runs.**
  - Bevy: `integrators/comparison-visual/src/main.rs:60`, `solvers/comparison-visual/src/main.rs:59`, `integration/coupled-impact-viewer/src/main.rs:70`, `integration/deescalation-contrast-viewer/src/main.rs:136`, `integration/two-way-striker-viewer/src/main.rs:81`.
  - Cargo examples: `sim/L1/sim-bevy-soft/examples/bonded_sandwich.rs:94`, `tools/cf-codesign/examples/emps_sim_to_real.rs:179`, `tools/cf-codesign/examples/real_double_pendulum_sim_to_real.rs:226,275`.
  - These need either running the binary under §4c's dump, which for Bevy means a window, or rendering the template by hand with its constants.
- **URDF-built examples** (`urdf-loading/*` call `sim_urdf::load_urdf_model` / `urdf_to_mjcf`) reach sim-mjcf only at runtime. §4c captures them for the validators among them.
- Whether CI should load these going forward stays deferred (decision 15).

---

## 4. The verification plan for the one-PR series (runnable protocol)

**Conventions:**
- `REPO=$REPO`, `SCR=<session scratchpad>`, `T=$SCR/rigid_plan_tests` (corpus, manifest, inputs). Tools live in `DV/scripts/` and `DV/harness/`.
- Everything below that builds a scratch tool uses its own `CARGO_TARGET_DIR` under `SCR`, `-j 4`, and a `timeout` at spawn.
- Anything that loads submodule models runs one process per file under `watchdog.py 2500 <secs> 20`.
- **HEAD and a clean tree before and after every measurement.**

### (a) Baseline on `main`, before the branch's first commit

| step | command | time (referent) |
|---|---|---|
| a0 | `git -C $REPO rev-parse HEAD; git -C $REPO status --short` | — |
| a1 | `git -C $REPO archive <main> \| tar -x -C $SCR/base_src`; point `DV/harness/Cargo.toml`'s two path deps at it; `CARGO_TARGET_DIR=$SCR/base_t cargo build --release -j 4 --bin harness2` | ~1 min cold (35.7 s for the warm-dep build measured here) |
| a2 | `cd $T; for i in {1..10}; do timeout 120 $BIN < in_embedded.tsv > base_emb_$i.jsonl; done; python3 -I DV/scripts/varying.py . base_emb_{1..10}.jsonl` → **the mask**: 71 at `3520544e` | 0.59–0.83 s per pass (planner `emb_1.time`, `run_main_1.err`) |
| a3 | `timeout 120 $BIN < in_repo_xml.tsv > base_repo.jsonl` | < 1 s |
| a4 | `for i in 1 2 3; do python3 -I DV/scripts/run_files.py $BIN base_sub_$i.jsonl sub_xml.txt; done` — all 253 submodule files, `fourier_n1` included (harness2 only) | 78 s per pass (233 s for 3, measured) |
| a5 | MuJoCo 3.5.0 oracle on the corpus: the batch loop in §4d over `$T/corpus/docs` → `mj350_full.jsonl` | ~2 s for 1,584 docs (measured, 16 batches of 100). ⚠ **MuJoCo's own message varies between runs for 2 docs** (`67ed261056e8c19d`, `ce4efcfe7cb49f86`: which missing `.stl` it names); status does not ⇒ compare status, and messages only after normalising |
| a6 | **licensed gates:** fetch and verify the BodyParts3D meshes per `design/cf-fsu-geometry/BODYPARTS3D.md`, export `CF_L4_STL CF_L5_STL CF_DISC_STL`, then `cargo xtask licensed-gates --run`; tally the runner's own `PASS`/`FAIL` lines, never the panic text | not measured here. The set of red gates at `main` is the baseline |
| a7 | `cargo test -p sim-conformance-tests --release --test integration golden_ -- --ignored 2>&1 \| tee base_golden.log`; for each of the 24 ignored tests record the first-mismatch line (`step`, `dof`, `diff`). The test stops at the first mismatch, so for whole residuals use a scratch probe computing max\|Δqacc\| per flag file | not measured |
| a8 | `cargo test -p cortenforge-sim-mjcf --features mjb` and `cargo test -p cortenforge-sim-mjcf --no-default-features` | not measured |
| a9 | full validator fleet: `cargo run -p xtask --release -- run-validators` | not measured |

### (b) Per commit — the five buckets

```
timeout 120 $BIN_HEAD < $T/in_embedded.tsv > head_emb.jsonl     # after L27: ×1 suffices; before L27: ×10
python3 -I DV/scripts/buckets.py base_emb_1.jsonl,...,base_emb_10.jsonl head_emb.jsonl    # same for _repo and _sub
```

| bucket | rule for passing |
|---|---|
| unchanged | — |
| ok→ok changed | every doc is on **this commit's pre-registered expected-change list** (L27: §1.5) |
| ok→err | every doc maps to **this commit's rule**, i.e. its error variant/message, and appears in §5's table under that rule. Anything else ⇒ stop |
| err→ok | the doc must be MuJoCo-3.5.0-ok (oracle), and the commit's rule must explain it |
| err→err message change | expected wholesale at mjcf-E1; afterwards only where this commit's rule now fires first |
| **NEW NONDETERMINISTIC** | must be **empty** from the L27 commits on |

Pre-register before each commit: the synthetic must-flip doc (it fails at the parent, passes at the commit), the expected-change list, and the expected ok→err set (from §5).

### (c) Runtime capture of the 157 templates and runtime-generated MJCF — dump code that cannot land

There are 157 `format!` templates: 104 in CI-run tests, 26 in lib code of CI-tested crates, 18 in validators, 9 never run. There are also 8 fragments. Runtime producers: cf-design, cf-mjcf-emit, sim-urdf, sim-therm-env, cf-codesign, cf-fsu-model, cf-osim. 28 CI-tested crates reach sim-mjcf (`DV/ci_tested_reaching_mjcf.txt`).

1. **A detached worktree, so there is no branch to push:** `git -C $REPO worktree add --detach $SCR/capture-wt <commit>`.
2. **Apply the dump as an uncommitted patch kept outside the repo:** `git -C $SCR/capture-wt apply $SCR/capture_dump.patch`.
   - The patch adds a block at the top of `parse_mjcf_str` (`sim/L0/mjcf/src/parser/mod.rs:50`). Both `load_model` (`builder/mod.rs:373`) and `load_model_from_file` (`:408`) go through it.
   - The block is guarded by `std::env::var_os("CF_SCRATCH_MJCF_DUMP")` and carries the marker comment `// CF-SCRATCH-MJCF-DUMP (never commit)`.
   - It writes `<dir>/<siphash>.xml` and appends `{id, test: thread name (libtest names threads after tests), exe, caller: first backtrace frame outside sim/L0/mjcf}` to `<dir>/index.<pid>.jsonl`.
   - A second, identical block in `load_model_from_file` records the file path, because asset paths must be replayed from the original file.
3. **Run the capture with its own target dir:** `CF_SCRATCH_MJCF_DUMP=$SCR/cap CARGO_TARGET_DIR=$SCR/cap_t`, then:
   - `cargo test -p <crate>` for each crate in `ci_tested_reaching_mjcf.txt` (per crate, never the workspace; release where CI uses release);
   - `cargo run -p xtask --release -- run-validators`.
4. **Coverage assertion:** every `format-template` record in `manifest.jsonl` (file:line) must have ≥ 1 captured doc whose `caller` is in that file. The 9 never-run templates (§3) are listed as uncovered, not silently dropped.
5. **Tear down:** `git -C $REPO worktree remove --force $SCR/capture-wt`, then `git -C $REPO worktree prune`.
6. **Guard, made to fail once:**
   - Before the final push: `git -C $REPO diff origin/main...HEAD | grep -c 'CF-SCRATCH-MJCF-DUMP'` must print 0.
   - Prove the grep works first: `git -C $SCR/capture-wt diff | grep -c 'CF-SCRATCH-MJCF-DUMP'` must print ≥ 1.
   - Belt and braces: grep for `CF_SCRATCH_MJCF_DUMP` as well.

### (d) MuJoCo 3.5.0 oracle over everything captured

```
cd <docs dir>; files=( *.xml ); i=1
while (( i <= ${#files} )); do timeout 120 $SCR/rigid_oracle_350/venv/bin/python -I DV/scripts/mj_full.py out.jsonl ${files[i,i+99]}; (( i += 100 )); done
```

- File-mode captures use `mujoco.MjModel.from_xml_path(<original path>)`.
- **Assertion:** join (ours@main, ours@head, MuJoCo).
  - Every ok→err at head must be (MuJoCo-err) **or** on the approved deviation list (decision 1's two lists, each entry naming its doc).
  - No MuJoCo-ok doc may newly fail unless it is on that list.
  - Every err→ok must be MuJoCo-ok.
  - Status only; messages are normalised (see a5).

### (e) What CI never runs, and this PR must run locally (exact commands)

| what | command | why |
|---|---|---|
| mjb feature tests | `cargo test -p cortenforge-sim-mjcf --features mjb` | `sim/L0/mjcf/Cargo.toml` says to run it; `grep -rn mjb .github/workflows/` finds nothing, and `--all-features` is deliberately absent from the test matrix (`quality-gate.yml:410`) |
| no-default-features | `cargo test -p cortenforge-sim-mjcf --no-default-features` | CI only does `cargo check -p cortenforge --no-default-features…` (`quality-gate.yml:1076-1079`) |
| 24 ignored golden-flag tests | `cargo test -p sim-conformance-tests --release --test integration golden_ -- --ignored` (before/after residuals as in a7) | `golden_flags.rs`: 24 of 26 are `#[ignore]` |
| licensed gates | `cargo xtask licensed-gates --run` on main first, then on the PR head; at minimum `--only cf-design-tests` for L27 (rung4/rung4b call `convex_hull`) | never in CI by licensing |
| submodule models | §4a step a4 at the parent and at each mjcf commit | CI never checks out submodules |
| full validator fleet | `cargo run -p xtask --release -- run-validators` | CI scopes it to the affected crates on PRs (`quality-gate.yml:1327-1341`); running it locally finds flips before CI does |
| grade | `RAYON_NUM_THREADS=1 cargo xtask grade <crate>` for every touched crate (sim-core, sim-types, sim-mjcf, cf-geometry, sim-thermostat if ledger-L24 touches it) and downstream consumers | grade runs only in CI shards; a clean push proves nothing |
| clippy as the hook runs it | `cargo clippy -p <crate> --all-targets --all-features -- -D warnings` | the pre-commit hook runs this per staged crate (`xtask/hooks/pre-commit:273`) |
| doc-theft | `cargo run -q -p xtask --release -- doc-theft --base origin/main` | CI gate; find it before pushing |
| the dump guard | §4c step 6 | — |

---

## 5. The consolidated flip table

**Sources:**
- ours today: `rigid_plan_tests/emb_1.jsonl` (status is deterministic across the 10 runs);
- MuJoCo 3.5.0: `DV/mj350_full.jsonl`, rerun keeping the full message, element and line. Its status agrees with `rigid_oracle_350/corpus350.jsonl` on 1,584 of 1,584;
- attributes our parser **never requested**: `DV/attr_ignored.jsonl`, from a scratch build that logs every `get_attribute_opt` call (`parser/attrs.rs:31`, the parser's only attribute accessor besides `include.rs:141`). Made to lie once: a doc with `typo="1"` and `dampnig="1"` reports exactly `geom@typo`, `joint@dampnig`;
- the deviation scanner (NaN/inf text, auto backwards `*range`, timestep ≤ 0, ball-then-slide, the 12 dropped sensor types). Made to flag all five on a crafted doc, and nothing on a clean one. **It found no trigger in any static doc that MuJoCo loads**;
- CI reach: the planner's `analyze2.py` verdict, corrected to map files under a crate's test targets as test code.

**Full table:** `DV/fliptable.tsv` (source file:line, doc, CI reach, ours, ours message, MuJoCo 3.5.0, rules, bucket). 1,686 source records, 1,584 docs.

**By unique doc:** 1,301 unchanged · **164 ok→err** · 13 ok→ok (L27 only) · 94 err→err (message) · 12 err→ok candidates.
- This assumes the allowlist keeps every extension the parser actually reads, and errors on everything it ignores (decisions 19/20).
- ok→err by CI reach: 145 CI tests, 7 validator mains, 9 Bevy apps, 4 `.md` (source records).

### 5.1 ok → err (the fixed decisions flip these)

CI key: T = CI-run test, V = validator (CI), B = Bevy app (never run), md = docs, X = cargo example.

| rule (owner) | docs | CI | sources |
|---|---|---|---|
| **mjcf-S1: `flex@density` ignored by both** (see §1.10: not an extension) | 71 | T71 | `flex_unified.rs` ×44, `flex_flex_collision.rs` ×12, `deformable_friction_dt25.rs:22,50,76`, `parser/tests.rs:2034`, … (TSV) |
| attribute length, "too much data" (mjcf-S2 family) | 20 | T20 | `equality_constraints.rs:1367` (polycoef); `phase8_spec_b_qcqp.rs:52,89,202,246,364,404,444,477,528,565`; `unified_solvers.rs:1268,…` (5-value friction) |
| **NO ROW: composite `curve` "invalid shape"** | 16 | T10 V3 B3 | `examples/fundamentals/sim-cpu/composites/{cable-catenary:58, cable-loaded:43, hanging-cable, stress-test}`, `composite.rs`, … |
| mjcf-S7 moving-body mass | 10 | T10 | `builder/compiler.rs:611`, `builder/mass.rs:330`, `fluid_forces.rs:119,185`, `spatial_tendons.rs:85,112,261`, `fluid_derivatives.rs:1393`, `sensor_phase6.rs:1547`, `tools/cf-osim/tests/r4_micro_spike.rs:30` |
| mjcf-S7 inertia triangle | 4 | V1 B3 | `free-joint/{spinning-toss:40, stress-test:29, tumble}`, `urdf-loading/inertia` |
| required attribute missing (mjcf-S1 family) | 4 | T3 V1 | `builder/compiler.rs:557`, `sensor_phase6.rs:17` (objtype), `callbacks.rs:199` (user `dim`), `urdf-loading/stress-test/src/main.rs:120` |
| mjcf-H4 / mjcf-S6 ctrlrange (damper/adhesion with no range) | 4 | T4 | `builder/compiler.rs:516`, `parser/tests.rs:1551`, `phase7_spec_a.rs:465,496` |
| **NO ROW: actuator `lengthrange` did not converge** | 4 | T4 | `parser/tests.rs:1454,1492`, `actuator_phase5.rs:194`, `phase7_spec_a.rs:271` |
| mjcf-S1 `freejoint@damping` ignored | 3 | V1 B2 | `equality-constraints/{connect-body-to-body:46, connect-to-world:46, …}` |
| mjcf-S7 hfield size not positive | 3 | T2 B1 | `builder/mod.rs:1250`, `raycast_heightfield.rs:14`, `raycasting/heightfield/src/main.rs:65` |
| **NO ROW: dangling reference** (bodypair / flex body) | 3 | T3 | `builder/mod.rs:1345,1379`, `composite.rs:361` |
| core-L7 / plugins (MuJoCo: plugin not found) | 3 | T3 | `parser/tests.rs:2227,2299,2327` |
| mjcf-S2 keyword (`{integrator}` str::replace template) | 3 | md3 | `sim/docs/todo/future_work_10d.md:1323,1448,1513` |
| decision 19 limitation: MuJoCo-valid, never read: `flex@vertex`, `flex@element` (2), `user@dim` (1), `edge@solref` (1) | 4 | T4 | `builder/mod.rs:1345,1379`; `parser/tests.rs:2274`; `flex_unified.rs:2138` |
| connect form (mjcf-S15 / ledger-L28) | 2 | T2 | `equality_constraints.rs:504,572` |
| **NO ROW: sleep-policy compile check** | 2 | T2 | `fluid_derivatives.rs:2768`, `sleeping.rs:2615` |
| mjcf-S2 keyword `flag="true"` / `integrator="euler"` | 3 | T2 md1 | `runtime_flags.rs:2551`, `implicit_integration.rs:371`, `MUJOCO_GAP_ANALYSIS.md:1209` |
| mjcf-S6 range: limited hinge `0 0` / ball `range[0]≠0` / autolimits=false + range | 3 | V1 T2 | **validator** `joint-limits/stress-test/src/main.rs:175`; `ball_joint_limits.rs:1215`; `builder/joint.rs:665` |
| mjcf-S10 undefined class · mjcf-S8 multiple orientations | 2 | T2 | `default_classes.rs:278`; `builder/body.rs:702` |
| mjcf-S1 `flex@selfcollide`, `flexcomp@density`, `flag@passive` ignored | 3 | T3 | `flex_flex_collision.rs:590`, `flex_unified.rs:185`, `runtime_flags.rs:1385` |
| **NO ROW: qhull fails on a flat mesh** (MuJoCo refuses; ours hulls it) | 1 | T1 | `mesh_inertia_modes.rs:341` |
| **NO ROW: limit sensor on an unlimited joint** | 1 | T1 | `mjcf_sensors.rs:221` |

**Allowlisted, so not flips:** our parser reads them and MuJoCo rejects them.
- `option integrator="implicitspringdamper"` (32), frame sensors `site=` / `body=` / `geom=` (36), `<distance>` equality (10), `hillmuscle` (5), `exactmeshinertia` (3), `<flexcomp>` (3), `flex@mass` (2), `adhesion@gear` (2), `<sensor>` in defaults (2), `<plugin>`, `option@nconmax`, default `motor@kp`, `touch@objtype` (1 each).
- **The design's "flex density 70 + freejoint damping 3" as extensions is contradicted:** the parser never reads either (attr probe; for density, also the fingerprint test).

### 5.2 err → ok candidates (ours refuses, MuJoCo 3.5.0 loads)

| source | ours | resolves to |
|---|---|---|
| `parser/tests.rs:1080,1137,1154,1984` | vertex-only mesh "requires face data" | mjcf-D1, decision 13 (hull in code ⇒ err→ok) |
| `parser/tests.rs:1256`, `MUJOCO_GAP_ANALYSIS.md:827` | connect "unknown body1" — the body is a self-closing `<body name="floating_body"/>` (doc read) | mjcf-S4 ⇒ err→ok |
| `frame.rs:716` | `<inertial>` in `<frame>` | decision 5 (defer ⇒ stays err, a listed deviation) |
| `fluid_forces.rs:1209` | `fluidcoef` with 2 of 5 values | attribute-length rule (MuJoCo loads it) — owner S2 |
| `tendon_springlength.rs:199` | `springlength="-0.5"` | owner: validation (MuJoCo loads it) |
| `keyframes.rs:805,836`, `tendon_springlength.rs:225` | NaN / inf | decision 3 ⇒ **stay refused** (deviation) |

### 5.3 What this table does not settle

- **Six refusal kinds have no triage row.** These are the **NO ROW** rows of §5.1: composite curve 16, lengthrange 4, dangling reference 3, sleep compile 2, qhull flat mesh 1, limit sensor 1. Each needs an owner and a parity verdict.
  - Two of them are MuJoCo failing to compute something we can compute: lengthrange non-convergence and qhull on a flat mesh. Rule 1 does not word that case.
- The table classifies docs by **MuJoCo's first error only**. A doc can carry a second rule our PR also adds; the per-commit buckets (§4b) find those.
- 157 templates and all runtime MJCF are outside it until §4c runs.
- The core-side runtime flip (ac29, `runtime_flags.rs:718`, core-L4) is not a load flip, so its doc shows as unchanged here.

---

## 6. Commit list (ledger-L27 lands right after mjcf-E1) and per-commit verification

| # | commit | content | verification |
|---|---|---|---|
| M1 | mjcf-E1 + E2 (another area) | error type | (b) with the ×10 mask: expect err→err message change only |
| **M2** | `fix(cf-geometry): convex hull no longer depends on hash order` | `convex_hull.rs` (§1.3); `#![deny(clippy::iter_over_hash_type)]` at `lib.rs:23`; test `hull_is_identical_across_repeated_calls` | `cargo test -p cortenforge-geometry`; `cargo test -p cortenforge-sim-core`; `cargo test -p sim-conformance-tests --release`; the 4 validators (expect identical stdout, as measured); **`licensed-gates --run --only cf-design-tests`**; hook clippy for cf-geometry; harness ×10: the mask shrinks to the 60 flex docs (measured with the hull-only build), mesh docs ok→ok changed once (11 embedded + submodule mesh models) |
| **M3** | `fix(sim-mjcf): flex edges in MuJoCo's order, hinges in edge order` | `flex.rs` (§1.3) incl. the dim / element-length refusal (Q2); `#![deny(clippy::iter_over_hash_type)]` at `lib.rs:149`; tests `flex_edges_follow_mujoco_order`, `flex_build_is_repeatable` | `cargo test -p cortenforge-sim-mjcf`; `cargo test -p sim-conformance-tests --release`; hook clippy; harness ×10 + submodules ×3: **0 nondeterministic** (measured with both fixes); vs M2: only the 60 flex docs change |
| M4… | the remaining mjcf commits | — | from here (b) needs **one** run per side; "NEW NONDETERMINISTIC" must stay empty |

- M2 and M3 are independent; either order compiles. I put the lower layer first.
- Neither changes a public signature. Behaviour changes: hull output order and flex edge/hinge order, plus M3's refusal of invalid flex dims.

---

## 7. Dependencies on other areas

- **mjcf_errors_defaults (mjcf-E1):** M3's refusal needs the error type; at minimum `ModelConversionError` with a message.
- **mjcf_validation:** dim-3 tet re-orientation (§1.10); the 6 unowned refusal kinds (§5.3); attribute length; S6/S7.
- **mjcf_parser:** flex `density`, `vertex=`/`element=`, and `freejoint damping` (the allowlist inputs are corrected in §5.1); the STEP / non-MuJoCo mesh format question (Q3); the silent partial-element drop.
- **core areas:** none for L27. sim-core's hull readers change only through cf-geometry's output order.

## 8. What my methods cannot see

- **Hash iteration** through adaptors that the scanner's per-file identifier heuristic misses: fields declared in another file, hash maps returned from functions, type aliases. The lint sees only `for` loops.
- **Nondeterminism not caused by hashing:** threads, addresses, `parallel`-feature reductions. The harness ran default features only.
- **Rare variation:** 10 processes per doc. The in-process probe showed variation for docs that never varied across processes, and vice versa.
- **Flip-table accuracy depends on three things:** MuJoCo's first error; the attribute probe's notion of "requested", since an attribute requested but then unused counts as read; and the corpus extraction, which I did not re-audit.
- **Not run by me:** licensed gates, the ignored golden tests, `--features mjb`, `--no-default-features`, grade, the full validator fleet, and any runtime capture.

## 9. Artifacts (session scratch, `DV/` = `SCRATCH/rigid_spec/determinism_verification/`)

- `l27_probe.patch` — the fix as probed. **Differs from §1.3 in one place:** it returns `&[]` for other element lengths.
- `fliptable.tsv`, `l27_expected_change.tsv`, `mj350_full.jsonl`, `attr_ignored.jsonl`, `mj350_schema.json`.
- `runs/` — every harness output named above.
- `scripts/` — `watchdog.py`, `run_files.py`, `varying.py`, `buckets.py`, `fliptable.py`, `flip_md.py`, `hazards.py`, `hash_iter_scan.py`, `mj_full.py`, `mj_flexdump{2,4}.py`, `mj_schema.py`, `l30.py`, `patch_*.py`.
- `harness/` — v1 `main.rs`, `bin/harness2.rs` streaming, `bin/flexdump.rs`.
- `l29probe/`, `attrprobe/` — sources.
- `harness_{before,after,flexonly,hullonly}`, `harness2_{before,after}` — binaries.
- Target dirs and the two repo copies were deleted at the end.

## Repo state

- Before: `3520544e747f6ba53e863b20b3c292cdeb9bb9ad`, `git status --short` empty.
- After (last step, after deleting my target dirs and both repo copies): HEAD `3520544e747f6ba53e863b20b3c292cdeb9bb9ad`, `git status --short` empty, `git worktree list` shows only the main checkout. Nothing of mine is left running (`pgrep` empty).
