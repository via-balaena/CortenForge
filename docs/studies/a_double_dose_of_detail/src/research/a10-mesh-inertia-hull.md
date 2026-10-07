> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid spec — mesh, inertia and hull parity (STL dedupe, principal axes, hull, flat meshes)

Area owner: researcher "mesh_inertia_hull". Items Jon put in this PR (RIGID_SPEC §2, "known divergences: fix in Rigid"): STL vertex deduplication, principal-axis order of a full inertia, hull face order; and (§2, "where MuJoCo itself fails, load … provided a test shows our result is right") the flat mesh with faces. Repo read at branch `fix/rigid-sim-core-mjcf`, whose code equals `main` @ `3520544e` (`git diff --stat 3520544e HEAD -- ':!sim/docs'` prints nothing). `MIH/` = `SCRATCH/rigid_spec/mesh_inertia_hull/`. MuJoCo citations are 3.5.0 (`SCRATCH/mj350src/mujoco`, 881544c) under `src/`.

Two things this section reports that are **not** in Jon's list, because the measurements for his items ran into them and they decide whether his items can be closed: (a) our mesh mass properties differ from MuJoCo's default by a median 1.7 % on real STL meshes (§3); (b) our convex hull is not convex on real meshes (§4.2). Both are in my area (mesh inertia, hull) and both have measured fixes.

---

## 0. Method, and what it cannot see

- **Oracle.** `mujoco==3.5.0` (`SCRATCH/rigid_oracle_350/venv`, macOS arm64 wheel), one-mesh docs and the corpus. Source read for every rule cited.
- **Data.**
  - 3,495 submodule mesh files (2,573 STL, 922 OBJ; `MIH/out/meshfiles_rel.txt`), each wrapped as `<mesh file=…/>` on a free body with default density (`MIH/cases/files/*.xml`). MuJoCo loads all 2,573 STL and 887 OBJ. All runs over these went under the RSS watchdog (`determinism_verification/scripts/watchdog.py`, 2.5 GB, poll ≤ 50 ms); peak 1,066 MB (MuJoCo), 174 MB (ours).
  - The 1,584 corpus docs; 20,211 inertia tensors (`MIH/scripts/gen_tensors.py`: 6 diagonal permutations, 4 repeated-eigenvalue diagonals, 200 rotated axisymmetric, 20,000 random rotated physical tensors, mass 1e-3…1e3, box dims 0.05…1).
- **Prototype.** A `git archive HEAD` copy (`MIH/repo`) with every change gated by an env var (`MIH_DEDUPE`, `MIH_LEFTHAND`, `MIH_EIG3`, `MIH_FROMTO`, `MIH_MJMESH`, `MIH_LEGACY`, `MIH_HULL_EXACT`, `MIH_FLATHULL`), so "main" and each variant run from one build. Diff: `MIH/prototype.patch` (722 lines). The copy's cf-geometry also carries A6's determinism fix (`determinism_verification/convex_hull_fixed.rs`) ungated, so its "main" hull = main + M2. A second untouched copy (`MIH/repo_main`) re-verified every headline "main" result (§4.2, §5, §6).
- **Probes.** `MIH/probe` (modes `eig3`, `eig3fma`, `bodies`, `meshes`, `hull`, `hullpts`, `traj`), DV's streaming harness rebuilt against the copy (`MIH/harness`, adds the final state vector).
- **Tests run in the scratch copy** (own `CARGO_TARGET_DIR`, never the repo): `sim-conformance-tests` (`integration` 1,338 + 27 ignored, `mujoco_conformance` 83), `cortenforge-sim-mjcf` (lib 385, `forward_conformance`), `cortenforge-geometry` (207 lib + 7 binaries); 14 validator examples. Under main env all pass.
- **Checkers made to fail once:** the bit-equality check flagged the unfused eig3 port (251 of 20,211 bit-exact); the hull convexity check flagged main (§4.2); the vertex-order check gives ~1e-8 on the true order and 1.9e-2…7.1e-2 with one swapped pair (4 files); the face check is index-exact (a swap changes indices).
- **Cannot see:**
  - MuJoCo built anywhere but macOS arm64 (no Rosetta here: `arch -x86_64` → "Bad CPU type").
  - The 157 `format!` templates except the mesh_inertia_modes ones I rendered (`MIH/cases/mim`).
  - Licensed gates; grade; clippy on the prototype (not run; the prototype is not lint-clean code).
  - Bevy apps; trajectories of submodule models; OBJ hull ids (§4.1).
  - Whether a NaN STL hang (§1) is in the hull or the inertia stage (not isolated).

---

## 1. STL vertex deduplication

**Now.**
- `mesh_io::load_stl` returns a triangle soup: 3 new vertices per facet (`mesh/mesh-io/src/stl.rs:181-187` binary, `:243-249` ASCII). `load_mesh_file` (`sim/L0/mjcf/src/builder/mesh.rs:129-178`) scales and passes it on unchanged.
- Measured: `fourier_n1/n1.xml` 3,807,720 vertices for 1,269,240 faces. A 12-facet cube STL → 36 vertices (MuJoCo 8).
- Also on this path, all measured on `MIH/cases/stl/*`:
  - A NaN coordinate in an STL **hangs** `load_model_from_file`, main and prototype alike: no output, killed by `timeout 30`.
  - `scale="-1 1 1"` with `inertia="exact"` → "mesh volume is negative". MuJoCo flips the winding and loads, mass 1000.
  - A binary STL whose 80-byte header starts with `solid` and has no NUL byte is read as ASCII → "stream did not contain valid UTF-8" (`stl.rs:110`, `:126-134`). **53 of the 2,573 submodule STL files fail this way**, in 5 models: unitree_g1 27, shadow_dexee 21, pal_tiago_dual 3, tetheria_aero_hand_open 1, unitree_go1 1 (ledger-L28 / P-L28). MuJoCo loads all 53.
  - ASCII STL loads here, parsed straight to f64 (`stl.rs:232-234`, not rounded to f32).

**MuJoCo 3.5.0.**
- `LoadSTL` (`user_mesh.cc:1209-1279`) is binary-only. It requires 1 ≤ nfaces ≤ 200000 (`:1229-1234`) and `nfaces*50 == size-84` (`:1237-1241`); ASCII fails these as "perhaps this is an ASCII file?". It refuses |coord| > 2^30 (`:1258-1261`).
- For a left-handed scale (`scale[0]*scale[1]*scale[2] > 0` false, `:1210`) it writes each facet's vertices 2 and 3 into swapped slots (`:1264-1268`; the vertex array itself stays in file order).
- It then calls `ProcessVertices(vert, true)` (`:1276`, body `:543-613`):
  - A non-finite coordinate is refused ("vertex coordinate %d is not finite", `:574-578`).
  - Vertices are keyed by float `==` (`VertexKey`, `:522-538`), so `-0.0` merges with `0.0`.
  - Ids are assigned by **first appearance** (`:572-586`); the kept vertex is the first appearance's value (`:603-612`); faces are remapped (`:596-601`).
- OBJ and MSH are **not** deduplicated: `LoadFromResource(resource_)` takes the default `remove_repeated = false` (`user_objects.h:1220`, `user_mesh.cc:646-668,750`). OBJ applies the same left-handed swap (`:1085-1104`).

**Measured parity of the prototype** (`dedupe_exact`, `MIH/prototype.patch` in `builder/mesh.rs`; dedupe on f64 values of the f32 input, before scale, key = bits with `0.0` for both zeros):
- **2,520 of 2,520** STL files that both load: `nmeshvert` equal, `nmeshface` equal, vertex **order** equal, `mesh_face` **index-identical**.
  - Order was checked by reconstructing MuJoCo's original-frame vertices `mesh_pos + R(mesh_quat)·mesh_vert` (MuJoCo stores vertices re-centred and rotated, `user_mesh.cc:1676-1686`). Max relative difference: 6.5e-6 of the diagonal (`mesh_vert` is f32).
- `fourier_n1/n1.xml` under the watchdog: 629,958 vertices (MuJoCo 3.5.0: 629,958, A6 §1.10) vs 3,807,720. Load 0.79 s vs 1.74 s (one run each). Peak RSS 316 MB vs 304 MB: the dedupe does **not** lower the peak; what sets the peak was not examined.
- Cube 36 → 8. The tetra with a `-0.0` copy of the origin: main 12, prototype 4 (MuJoCo 4).
- With `MIH_LEFTHAND` the negative-scale cube loads with mass 1000.

**What dedupe changes in our results** (main vs dedupe, 2,520 STL):
- Mass/inertia bits change in 2,486 files (34 bit-identical), max relative 2.3e-11. The default mode today is `Convex`, whose inertia is summed over the hull built from the (now deduplicated) points.
- Hull vertex/face counts change in 85.
- Under the legacy/exact/shell formulas (§3), mass is computed from faces whose coordinates and order do not change. I expect bit-identity there; **not measured**.
- Collision:
  - `collide_mesh_plane` scans mesh vertices in vertex order with strict `>` ties (`sim/L0/core/src/collision/mesh_collide.rs:419-435`), so contact choice among tied vertices can change.
  - BVH and triangle paths see the same triangles.
  - **Not measured** on a model.

**Target.**
- STL: MuJoCo's dedupe, first-appearance order. Non-finite coordinates refused, for STL and OBJ (parity, and decision 3). Left-handed winding swap for STL and OBJ (parity).
- The binary/ASCII decision by size: `84 + 50·n == len` ⇒ binary, whatever the header says. That fixes 53 files and matches MuJoCo's own test (`:1237`).
- **Open (Q1.1):** ASCII STL, more than 200,000 faces, |coord| > 2^30.

**Change.**
- `sim/L0/mjcf/src/builder/mesh.rs` (private module, no public signature change):
  ```rust
  /// MuJoCo 3.5.0 `mjCMesh::ProcessVertices(vert, remove_repeated = true)` (user_mesh.cc:543-613):
  /// float equality (so -0.0 == 0.0), first-appearance numbering, first value kept, faces remapped.
  /// The map is lookup-only (no iteration), so `clippy::iter_over_hash_type` stays clean.
  fn dedupe_vertices(vertices: &[Point3<f64>], faces: &[[u32; 3]]) -> (Vec<Point3<f64>>, Vec<[u32; 3]>);
  ```
  Called in `load_mesh_file` for `.stl` only, before scale.
- In `load_mesh_file`, for every file format:
  - refuse a non-finite vertex: `MjcfError::NonFinite` / `InvalidValue`, naming the mesh and vertex index, message containing "not finite";
  - swap face slots 1↔2 when `!(sx*sy*sz > 0.0)` for STL and OBJ.
- `mesh/mesh-io/src/stl.rs`: `is_binary_stl_header` becomes a size check. It needs the file length (`File::metadata`), so `load_stl` reads it before choosing; the signature is unchanged.
- Dedupe stays in sim-mjcf, not mesh-io. 25 non-sim files call `mesh_io::load_mesh`/`load_stl` (`git grep`, e.g. `design/cf-cast/tests`, `tools/cf-scan-prep-core`), and MuJoCo's dedupe is an MJCF rule.

**Tests to add** (each measured on main as stated):
1. `stl_vertices_deduplicated_in_first_appearance_order`. `mesh_io::save_mesh` a 12-facet cube (binary soup), load via MJCF:
   - `mesh_data[0].vertices().len() == 8`;
   - the vertex list equals the first-appearance list;
   - faces remapped.
   Main: 36.
2. `stl_negative_zero_merges`: the tetra soup with one `-0.0` origin. 4 vertices; the kept value has the first occurrence's sign. Main: 12.
3. `stl_left_handed_scale_flips_winding`: cube, `scale="-1 1 1" inertia="exact"` → loads, mass 1000 (MuJoCo 1000). Main: Err "mesh volume is negative".
4. `stl_nonfinite_vertex_is_error`: NaN coordinate → Err containing "not finite". Main: **hangs**. Make it fail once under `timeout`, never un-timed.
5. `binary_stl_with_solid_header_loads` (mesh-io unit test + MJCF test): header `b"solid cube"` padded with spaces, 12 facets → loads, 8 vertices. Main: Err "stream did not contain valid UTF-8".

**Tests/examples that flip:**
- None measured. `builder/mesh.rs:927-951` asserts `>= 4` vertices; the scale tests `:956-1041` compare two loads of the same file, so both deduplicate.
- `builder/mod.rs:997` and `:1036` assert only `nmesh`/success.
- No corpus doc loads an STL (5 reference one; the file is absent, so all fail as now).
- Submodule: 53 files change from error to load. Model-level status of the 5 models: **not measured**.

**Downstream.** sim-mjcf only (private builder). The mesh-io detection change reaches every `load_stl` caller; it can only turn a failing binary file into a loading one.

---

## 2. Principal axes of an inertia, and the body's inertial frame

### 2.1 Now
- `extract_inertial_properties` (`builder/mass.rs:86-135`) and `compute_inertia_from_geoms` (`:146-249`) use nalgebra `symmetric_eigen` (`:103`, `:233`), `abs()` of the eigenvalues (`:105`, `:235`) and an unbounded `Rotation3::from_matrix` (`:118`, `:246`).
- Eigenvalue order is whatever nalgebra returns. Measured on 20,000 random tensors: decreasing 4,458, increasing 0, mixed 15,542. MuJoCo: decreasing 20,000.
- **Bug (measured):** `compute_inertia_from_geoms` rotates each geom's inertia by `geom.quat` only (`:217`; also `geom_effective_com`, `geom.rs:320`). A geom oriented by `euler`/`axisangle`/`xyaxes`/`zaxis` is treated as unrotated (`process_geom` resolves them for `geom_quat`, `geom.rs:42-49`).
  - Two-geom body with a box at `euler="0 0 45"`: ours (0.0505, 0.4960, 0.5260); MuJoCo (0.5260, 0.4451, 0.1014). A different tensor, not just an order.
  - One box at `euler="10 20 30"`: ours `iquat` = identity; MuJoCo = the geom's quat (0.9437, 0.1277, 0.1449, 0.2685).
  - (`MIH/cases/euler2.xml`, `single_rot.xml`.)
- A single geom is eigendecomposed too. MuJoCo copies it (below).
- `compute_fromto_pose` (`geom.rs:817-854`) aligns z with `to − from`; MuJoCo uses `from − to`. Every fromto geom's `geom_quat` is ours rotated 180°: same shape, different frame.
- No `inertiagrouprange` (0 hits in `sim/L0/mjcf/src`) and no geom mass threshold in the selection.

### 2.2 MuJoCo 3.5.0
- `mjuu_eig3` (`user_util.cc:661-754`):
  - Jacobi iteration, ≤ 500 sweeps, with the rotation accumulated as a quaternion;
  - stop when the largest off-diagonal < 1e-12 or the rotation's cosine > 1 − 1e-12 (`kEigEPS`, `:660`);
  - eigenvalues sorted **decreasing** by a 0,1,0 bubble that swaps only if `a + 1e-12 < b`, each swap rotating the quaternion 90° (`:731-749`).
- Callers:
  - `mjuu_fullInertia` (`:785-811`; refuses `eigval[2] < mjEPS`, "inertia must have positive eigenvalues") for `<inertial fullinertia>` (`user_objects.cc:2413-2417`);
  - several geoms (`:2219-2225`);
  - fusestatic accumulation (`:2311-2317`);
  - the mesh frame (`user_mesh.cc:1648`).
- `InertiaFromGeom` (`user_objects.cc:2157-2227`):
  - selects geoms with `mass_ > mjEPS` and group in `inertiagrouprange` (`:2166-2170`);
  - **one geom: copies** its pos, quat and diagonal inertia (`:2175-2181`);
  - several: `mjuu_globalinertia` + `mjuu_offcenter` (`user_util.cc:509-538`), then eig3.
- Mesh geoms:
  - geom inertia from the mesh's equivalent box (`user_objects.cc:3201-3204`, `user_mesh.cc:1666-1670`);
  - geom frame = user frame ∘ mesh frame (`user_objects.cc:3749-3754`).
- fromto: `vec = from − to` (`:3693-3697`), `mjuu_z2quat` (`user_util.cc:395-408`).

### 2.3 Measured
1. **A bit-for-bit port exists.** `MIH/probe/src/eig3.rs` has two variants.
   - The plain transcription matches the wheel bitwise on only 251 of 20,211 tensors, within 4.13e-12 abs. Order and quaternion (up to sign, 1e-9) agree on all 20,211.
   - The same code with the multiply-adds clang fuses under its default `-ffp-contract=on` (each `a*b + c` inside one expression → `fma`, left product first; spelled out in `fma` module comments) matches **20,211 of 20,211 bitwise, sign of zero included**.
   - MuJoCo's own output is therefore build-dependent at the 1e-12 level. Its x86-64 flags are `-mavx` without `-mfma` (`cmake/CheckAvxSupport.cmake`, `MujocoOptions.cmake:69-78`), so no fusion can occur there **by reading — not measured**.
2. **MuJoCo's eig3 is inaccurate, and parity inherits it.**
   - Reconstructing the tensor from (inertia, iquat): MuJoCo's error vs the input is up to **1.40e-6 relative** on the random set, at every magnitude band from 1e-6 to 1e9 (`MIH/scripts/recon_main.py`). Main's is ≤ 4.3e-15.
   - By reading, this is the cosine stop (`c > 1 − 1e-12` ⇒ residual rotation up to √(2e-12) ≈ 1.4e-6 rad).
   - On **mesh** unit-density inertias, which are tiny for small meshes, the absolute 1e-12 thresholds are coarse. MuJoCo's principal mesh inertia differs from the exact eigenvalues by > 1e-6 relative in 739 of 2,520 STL files, > 1e-3 in 301, max 0.82 (dm_control dog bones; order-sensitive comparison). The prototype's eig3 reproduces these MuJoCo values: ≤ 1e-6 on 2,520 / 2,520 (§3).
   - The order there is not even decreasing: BONECa_10, MuJoCo (1.1957e-7, 1.1973e-7, 7.35e-9). `MIH/scripts/port_debug.py`.
3. **Corpus.** 1,213 docs both load, 1,818 bodies (`MIH/scripts/cmp_corpus_bodies.py`):

   | | bit-equal | ≤ 1e-12 | same tensor, other order/frame | other frame, same values | different tensor |
   |---|---|---|---|---|---|
   | main | 1,361 | 344 | 73 (63 docs) | 1 | 14 |
   | prototype: eig3 (fused) + one-geom copy + orientation alternatives + MuJoCo fromto + selection `mass > 1e-14`, group 0…5 | **1,494** | **280** | **5** | 0 | 14 |

   - eig3 + one-geom copy without the fromto change: 153 other-order (worse than main). The copy exposes our fromto convention, which then has to change too.
   - The 5 residuals:
     - `4eed75d1…`, `eef83d84…`: a fromto geom inside a rotated `<frame>`. Frame composition gives a different, equivalent quat; frames area.
     - `68a35ea3…`: fusestatic merges geoms; MuJoCo accumulates the child's inertia.
     - `7b6eff02…`, `9b0854df…`: embedded vertices are f32 in MuJoCo, §3.
   - The 14 different-tensor bodies (11 docs) have causes outside this item, §6.
4. **Trajectories** (harness, 100 steps, excited qvel; main vs the full parity prototype of §2 + §3; 53 docs masked as nondeterministic by two main runs):
   - model_fp changes in 271 docs (probe: 179 change body data, 200 change `geom_quat`);
   - traj_fp changes in 66, median max|Δq| 4.7e-16, 90 % 4.5e-14;
   - outliers:
     - `collision_primitives.rs:1279` 0.189 (horizontal capsule via `euler`: the inertia bug fix);
     - `urdf-loading/inertia/src/main.rs:64` 1.6e-9;
     - `fluid_forces.rs:119,185` NaN (geoms below mjEPS mass are now excluded ⇒ massless moving body; MuJoCo refuses these, A5 §3.5/M19).
   - List: `MIH/out/expected_change_traj.tsv`, `expected_change_parity.tsv`.
5. **Tests** (scratch copy, parity env):
   - integration 1,335/1,338. Flips:
     - `mesh_inertia_modes.rs:218` `t9_convex_mode_non_convex_mesh` asserts components in nalgebra order (6666.667, 3539.683, 9039.683); eig3 gives (9039.68, 6666.67, 3539.68), MuJoCo's;
     - `:241` `t10` (§3);
     - `fluid_derivatives.rs:1391` `t23_massless_body_skipped` (the mjEPS selection; on A5's S7 list as `:1393`).
   - `mujoco_conformance` 83/83, sim-mjcf 385/385.
6. **Validators**:
   - Run, all pass under parity: sensor-adv, tendon, muscle, joint-limits, derivatives, equality, composite, energy, passive, keyframes, ml-spaces, urdf (35/35), mesh-collision, raycasting. The last three ran with the exact-predicate hull (§4.2) as well.
   - Stdout changes in 3:
     - sensor-adv: `-0.0000` → `0.0000`;
     - equality: angvel at t = 2 s 0.067 → 0.016, energy −3.42 → −3.43 %;
     - urdf: full-inertia line prints (0.309, 0.195, 0.096) instead of (0.195, 0.096, 0.309).

### 2.4 Target and change
- **Target:** MuJoCo's eig3, its inertial-frame construction, its fromto axis.
- New private module `sim/L0/mjcf/src/builder/eig3.rs`:
  ```rust
  /// MuJoCo 3.5.0 `mjuu_eig3` (user_util.cc:661-754), multiply-adds fused as the reference build fuses them.
  /// Returns eigenvalues in decreasing order and the frame quaternion (w, x, y, z).
  pub(crate) fn eig3(mat: &[f64; 9]) -> ([f64; 3], [f64; 4]);
  /// `mjuu_fullInertia` (user_util.cc:785-811): refuses a non-finite tensor and eigval[2] < 1e-14.
  pub(crate) fn full_inertia(full: &[f64; 6]) -> Result<(Vector3<f64>, UnitQuaternion<f64>), ModelConversionError>;
  pub(crate) fn global_inertia(local: &[f64; 3], quat: &[f64; 4]) -> [f64; 6]; // :509-527
  pub(crate) fn offcenter(mass: f64, v: &[f64; 3]) -> [f64; 6];               // :530-538
  ```
- `builder/mass.rs`:
  - `extract_inertial_properties` → `full_inertia`;
  - `compute_inertia_from_geoms(…, compiler: &MjcfCompiler) -> Result<(f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>), ModelConversionError>`:
    - MuJoCo's selection, copy and combination;
    - geom orientation through `resolve_orientation`;
    - a mesh geom's principal frame from `eig3` of the **unit-density** mesh tensor, composed with the geom quat;
    - its inertia from the equivalent box.
- `builder/geom.rs`: `compute_fromto_pose` → `from − to` + `z2quat`. This changes `geom_quat`, so it also changes fromto geoms' `geom_xmat` and anything reading it: frame sensors with `objtype="geom"` on such geoms, contact frames, and the order of a capsule's two endpoint contacts. The shape is unchanged. Beyond the corpus, tests and validators above, **not measured**.
- These replace A5 §3.1's `principal_axes` (nalgebra + `from_matrix_unchecked`). eig3 has no rotation fit, so **A5-Q1 is moot**. The non-finite guard goes on eig3's output (500-sweep bound, so a NaN input returns NaN, then `Err`).
- `inertiagrouprange` is today an unknown attribute, so it is refused after M15. Implementing it is one comparison in the selection; recommend implementing it here (Q2.3).

**Tests to add** (each fails on main, measured):
1. `eig3_matches_mujoco_bitwise`: 64 of the 20,211 tensors with MuJoCo's (inertia, iquat) as hex literals (`MIH/out/eig/mj.json`). The diagonal permutations, `2 2 2 0.5 0 0` → (2.5, 2.0, 1.5) with iquat (0.6533, 0.6533, 0.2706, 0.2706), axisymmetric, random. Main: 1 of 6 diagonal permutations right, A5 case wrong.
2. `single_geom_body_copies_geom_frame`: `single_rot.xml` → iquat == geom quat, inertia == box diagonal in geom axes. Main: iquat identity.
3. `geom_euler_rotates_body_inertia`: `euler2.xml` → (0.52595, 0.44509, 0.10139). Main: (0.0505, 0.4960, 0.5260).
4. `fromto_axis_is_from_minus_to`: capsule `fromto="0 0 0 0 0 -1"` → `geom_quat` identity (MuJoCo). Main: (0, 1, 0, 0).
5. Rewrite `t9` to MuJoCo's decreasing values (9039.683, 6666.667, 3539.683).

---

## 3. Mesh mass properties (found while measuring §1–§2)

**Now.**
- The default mesh inertia mode is `Convex`: `types.rs:291-295`, comment "MuJoCo default: enum value 0"; `builder/mesh.rs:61`; `sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B.md:33` chose it from the enum ordinal.
- `Legacy` is "absolute tetrahedron volumes" with the apex at the **origin** (`mesh.rs:692-784`, `det.abs()` at `:711`).
- Exact mode has no orientation-consistency check.
- Embedded vertices are kept as f64.

**MuJoCo 3.5.0.**
- Default `mesh->inertia = mjMESH_INERTIA_LEGACY` (`user_init.c:242`; 3.4.0 `:252` too). Measured on an L-prism with no attribute: mass 3.0 = legacy (convex 3.5).
- `Process()` (`user_mesh.cc:1537-1711`):
  - face centroid (`:1510-1534`);
  - CoM and volume with the pyramid apex at that centroid (`ComputeVolume :1373-1409`, `abs` per face for legacy);
  - inertia about the CoM with the apex at the CoM (`ComputeInertia :1713-1776`), whose total becomes the volume (`:1634-1639`);
  - errors: "surface area is too small" (`:1608`), "volume is negative" (`:1625`), "volume is too small … Try setting inertia to shell" (`:1627`), "eigenvalue … must be positive", "violate A + B >= C" with rtol 1e-6, atol 1e-9 (`:1650-1662`).
- Exact mode refuses inconsistently oriented faces (`:1555-1563`).
- User (embedded) vertices are `float` (`ProcessVertices(spec_vert_)`, `:321`).

**Measured.**
- **Python port** of MuJoCo's pipeline (`MIH/scripts/mj_mesh_mass_port.py`) reproduces MuJoCo's body mass on 2,519 / 2,520 STL to ≤ 1e-12 relative (max 4.1e-11), and the CoM to 6.8e-12.
- **Ours vs MuJoCo, default mode**:
  - STL: main median 1.74e-2, max ×102; 1,360 of 2,520 off by > 1e-3.
  - OBJ: main median 0.30.
  - Our `legacy` formula alone (`MIH_LEGACY`, without the port) is worse: STL median relative difference 42. Our legacy puts every pyramid's apex at the origin (`mesh.rs:705-713`); MuJoCo's puts it at the face centroid (volume, CoM) and the CoM (inertia). The port with MuJoCo's apexes matches.
- **Rust prototype** (`mj_mesh_props`, `MIH_MJMESH` + `MIH_LEGACY`, with §1 and §2):
  - STL mass ≤ 4.5e-11 relative on 2,520 / 2,520; principal inertia (order-sensitive) ≤ 1e-6 on 2,520; iquat ≤ 1e-6 on 2,519.
  - OBJ: median 2.1e-8 (f32 input, below); 15 > 1e-3, 13 of them among the 17 OBJ whose face count differs from MuJoCo's (MuJoCo reads the first OBJ shape only and splits quads, `:1081-1110`; cause per file **not isolated**).
- **Explicit modes, main** (400 random STL, `MIH/cases/modes`):
  - convex ≤ 1.7e-10 on 390 / 390 and shell ≤ 7.6e-15 on 390 / 390. The 10 missing are the §1 header bug.
  - exact: 340 within 1e-9; MuJoCo refuses 32 for inconsistent orientation (ours loads them); 6 differ by > 1e-3 (open meshes, where the apex matters).
- **f32 vertices**: `9b0854df…` (tetra, `0.28868`). Ours vs MuJoCo 2.6e-8 relative; with the XML values pre-rounded to f32, ours = MuJoCo to 2 ulps (`MIH/cases/tetra_f32.xml`). 2 of the 26 corpus docs with `<mesh vertex=>` hold a value not exact in f32.
- **Corpus**: the mesh part of the prototype changes no corpus doc beyond §2's list; the corpus meshes are closed and convex.
- **CI flips**: `mesh_inertia_modes.rs:241` `t10_legacy_mode` asserts 5000 for the L-shape. MuJoCo 3.5.0 gives **5322.5475**, I (7454.6087, 5978.4797, 2341.7171); the prototype gives the same. The test pins our origin-apex formula.
- `t1_default_mode_is_convex` (`:52`) passes either way (a cube) but names the wrong default.

**Target.** MuJoCo's pipeline and default. Recommend embedded vertices rounded to f32 as MuJoCo does (Q3.1).

**Change.** `builder/mesh.rs`:
```rust
/// MuJoCo 3.5.0 mesh mass properties (user_mesh.cc:1510-1534, 1537-1711, 1373-1435, 1713-1776):
/// (volume or area, CoM, unit-density inertia about the CoM). Convex mode on the hull's faces.
pub(crate) fn mesh_mass_properties(mesh: &TriangleMeshData, mode: MeshInertia) -> Result<MeshProps, ModelConversionError>;
```
- It replaces `compute_mesh_inertia{,_shell,_legacy,_on_hull,_by_mode}` (`mesh.rs:463-884`). Its AABB fallbacks (`:756-768`, `:654-667`) become MuJoCo's errors.
- `impl Default for MeshInertia` → `Legacy`.
- `mesh.rs:61` default → `Legacy`.
- `process_mesh` adds the exact-mode orientation check (sorted half-edges, `:1539-1563`).
- The four functions are `pub` inside private `mod builder`; `git grep` finds no caller outside sim-mjcf.

**Tests:**
- `t10` → MuJoCo's values. Main: 5000 (pinned wrong).
- `t1` renamed `t1_default_mode_is_legacy`, plus an L-shape with no attribute → mass 5322.55. Main: 7000 (convex).
- `exact_mode_refuses_inconsistent_orientation`: one face flipped on the cube, `inertia="exact"` → Err "inconsistent orientation". Main: loads (**not run**; the 32 STL cases are measured).
- A submodule-free regression: the L-prism of `MIH/scripts/legacy_default.py` (mass 3.0, I (1.8333, 1.5, 0.8333)).

---

## 4. Hull

### 4.1 Our hull vs MuJoCo's, as sets
- MuJoCo's hull is `mesh_graph`: `[numvert, numface, vert_edgeadr, vert_globalid, edge_localid, face_globalid]` (`user_mesh.cc:1897-1899`). It is built by qhull with `"qhull Qt"` (`:1866`; facet merging C-0 by default, then triangulated) on the pre-transform vertices (`:1567` before `:1603`).
- Compared by coordinates: our referenced hull vertices vs MuJoCo's `vert_globalid`, and faces as triples up to rotation. Uses the dedupe dumps, whose order matches MuJoCo's (§1). `MIH/scripts/cmp_hull.py`:

| STL, 2,520 meshes | same vertex set and face set | same vertex set, other triangulation | vertex sets differ |
|---|---|---|---|
| main hull (+ dedupe) | 1,674 | 514 | 332 (308 only by points within 1e-9 × diag of the other hull; max distance **5.4e-2 × diag**) |
| exact-predicate hull (§4.2) | **1,702** | 519 | 299 (**all** within 3.6e-8 × diag; 297 within 1e-9). Ours keeps 1,356 near-coplanar points qhull merged; qhull keeps 7 we dropped |

- "Other triangulation" = coplanar facets split differently (e.g. `dm_control/.../cube.stl`: 12 faces each, other diagonals). That is qhull's `Qt` choice, so it is a set difference, not an order difference.
- **OBJ is not trusted:** MuJoCo's own OBJ hull fails to contain its own `mesh_vert` (by > 1e-6 × diag) in 210 of 834 files, by up to at least 11 % (`anybotics_anymal_b/assets/anymal_hip_l_4.obj`; only the first 5 offenders were printed). Why its graph ids and `mesh_vert` disagree for OBJ is **not isolated**.
- `maxhullvert` (150 STL × {8, 32}): equal vertex sets 0 / 148 and 2 / 148; mean Jaccard 0.28 and 0.47. qhull's `TA` adds points in its facet-queue order; ours adds the globally farthest. In-tree use: 3 corpus docs, menagerie `robotstudio_so101/so101.xml`.

### 4.2 Our hull is not convex on real meshes (found; correctness, not order)
- **Measured on main** (deduplicated input; also with A6's deterministic fix, and re-run on the untouched copy): a mesh point lies outside our hull by
  - > 1e-9 × diag in 82 of 2,518 STL;
  - > 1e-6 in 30;
  - > 1e-3 in 12;
  - worst **5.35 %** (`toddlerbot_2xm/assets/neck_rod_2_visual.stl`).
  - MuJoCo's qhull: max 3.2e-5 (f32 storage). OBJ ours: 8 / 3 / 0.
- **Cause, instrumented** (`MIH_HULLDBG`):
  - The visible set is a BFS over neighbours with `dist > epsilon` (`convex_hull.rs:519-520`, ε = 1e-10 × diag, `:261`).
  - Near-coplanar faces (|dist| ≲ 1e-10) break it into pieces. A face visible from the eye by up to 3.1e-3 then survives (first when the neck_rod hull had 231 vertices), and the hull stops being convex.
  - Reassigning orphans to every face instead of the cone (`:672`) does not fix it (measured: still 8.0e-2).
- **Minimal reproduction, licence-free** (`MIH/cases/synth8.txt`, round numbers):
  ```
  -15.5 -1.5 13.5 / -13.9 1.6 17.8 / -11.2 1.6 19.0 / -10.1 1.6 15.4 / -10.5 1.6 14.5 /
  12.4 1.5999999 -17.2 / 10.8 1.5999999 -18.5 / 12.0 1.5999999 -16.3
  ```
  Our hull: 8 vertices, 12 faces, a point 6.6 % of the diagonal outside. MuJoCo: 8, 12, convex to 1.1e-8. With `1.6` everywhere, or a perturbation of 1e-5, ours is correct. Found by delta-debugging neck_rod to 8 points (`MIH/scripts/ddmin_hull.py`), then rounded.
- **Fix measured: exact orientation predicates.** `robust::orient3d` (Shewchuk), for conflict assignment and horizon visibility (`MIH_HULL_EXACT`):
  - worst outside distance ~1e-17 × diag on synth8 and the 5 worst meshes;
  - the neck_rod hull grows from 236 to 329 vertices: main was **missing 93**;
  - the 3,495-file pass took 25 s vs 22 s (one run each);
  - corpus: model_fp changes in 5 docs (all `flex_unified.rs`), traj ≤ 5e-16;
  - cf-geometry 207 + integration 1,338 + conformance 83 + sim-mjcf 385 all pass; mesh-collision and raycasting validators print identical output.
  - `robust` 1.2.0 is already in `Cargo.lock` via `spade`. It has no dependencies and is pure Rust, so no C/C++.
- **Dangling vertices.** `ConvexHull.vertices` keeps every point ever added (`:606`), including ones whose faces were all deleted: 82 such vertices in 71 of 3,424 meshes. Never vertex 0 in this data, so hill-climbing from 0 was not hit. They inflate `vertex_count()`, and the struct's own invariant ("All face indices are valid…", `:24-30`) does not cover them.
- **Change:**
  - In `design/cf-geometry/src/convex_hull.rs`, the signature `pub fn convex_hull(points: &[Point3<f64>], max_vertices: Option<usize>) -> Option<ConvexHull>` is unchanged. Visibility and conflict tests become `orient3d(face) < 0` (strictly outside); the float distance only ranks candidates.
  - The output keeps only face-referenced vertices, renumbered in first-reference order.
  - `Cargo.toml` gets `robust = "1.2"`, as a workspace dependency.
  - The ε stays only for the degenerate initial simplex (`:251-262`, `:327-379`).
- **Tests:**
  - `hull_contains_every_input_point` on synth8: main fails, 6.6e-2.
  - A property test over seeded random near-coplanar sets: perturbations 1e-7…1e-12 on 2–4 planes. Asserts every input point is within 1e-12 × diag inside every face plane, and every vertex is referenced. Its failure rate on main was **not measured**: run it on main first and keep only seeds that fail there.
  - `hull_has_no_unreferenced_vertices`.
- **Flips:** none measured in CI (above). The licensed `cf-design-tests` rung4/rung4b call `convex_hull` directly (A6 §1.7) and were not run.

### 4.3 Is the ORDER observable?
- **Yes, in both simulators.** Measured on a 0.2 m cube mesh resting on a plane, spinning at 3 rad/s about z, 500 steps (`MIH/cases/order/box_p{0,1,2}.xml`, the same geometry with permuted input vertex order):
  - MuJoCo 3.5.0 picks a different 3 of the 4 tied bottom corners and ends at x,y = ±6.4e-5 (mirror images).
  - Ours, likewise, ±8.9e-5 (p0 = p1 ≠ p2).
- MuJoCo's plane–mesh contacts are the hill-climbed support vertex plus its hull-graph neighbours in qhull's edge order (`engine_collision_convex.c:1108-1135`).
- Ours is MuJoCo's *no-graph* path, a scan of mesh vertices (`mesh_collide.rs:380-470`, "Path B").
- Ours also finds no contact at exactly zero distance (ncon 0 at t = 0, MuJoCo 3), from `depth > -margin` (`:428`) vs MuJoCo's `dist > margin` reject (`:1056`). That is a **separate divergence**, not order.
- Matching MuJoCo's trajectory there needs qhull's triangulation and vertex order **and** MuJoCo's graph-path contact algorithm. Order alone is not enough.

### 4.4 What matching qhull's exact order would take (assessment; no C/C++ allowed)
- Order and sets both come out of qhull's whole pipeline:
  - initial simplex (`qh_maxsimplex`);
  - facet visit order (`qh_nextfurthest`, `facet_next`);
  - partitioning (`qh_partitionall`);
  - pre-merging under C-0 / `_zero-centrum`;
  - `qh_triangulate`;
  - `FORALLvertices` / `FORALLfacets` list order (newest-first insertion, `toporient` flip at `user_mesh.cc:1974-1990`);
  - `TA` for maxhullvert.
- The measured differences above are already **set** differences: 519 triangulations, 299 merges, every maxhullvert case. They are near-coplanar points (all within 3.6e-8 × diag of the other hull) and coplanar-facet triangulations, which qhull settles by merging and `Qt`, not by tie-breaking. A port would have to reproduce qhull's merge decisions, which depend on its roundoff model (`Error-roundoff`, `_one-merge`, `_near-inside` printed in its diagnostics).
- Size: libqhull_r is ~30 k lines of C. The 3-d convex-hull path that MuJoCo exercises (geom, merge, poly, poly2, qset, mem, user, triangulate) is most of it; not counted precisely.
- Risk: bit-level agreement depends on qhull's floating-point evaluation order. As eig3 showed (§2.3), that differs by compiler flag, so even a faithful port matches one build of MuJoCo, not all.
- **Recommendation:** do not port. Fix correctness (§4.2), keep our deterministic order (A6), and list as a stated limitation:
  - "hull vertex/face order, coplanar-facet triangulation, near-coplanar (≤ 3.6e-8 × diag measured) vertex inclusion, and maxhullvert vertex choice differ from qhull";
  - plus, as a separate row, "plane–mesh contacts use MuJoCo's no-graph path" (§4.3).

---

## 5. Flat mesh with faces (Jon: load where MuJoCo fails, if a test shows our result is right)

**MuJoCo 3.5.0, measured** (`MIH/cases/flat/flat_*.xml`, unit square, 2 triangles, free body):

| inertia | collision on | collision off (`contype=0 conaffinity=0`) |
|---|---|---|
| shell | "qhull error" | **loads: mass 1000, I (166.667, 83.333, 83.333), ipos (0.5, 0.5, 0), iquat (0.5, 0.5, −0.5, 0.5)** |
| legacy / exact | "qhull error" | "mesh volume is too small … Try setting inertia to shell" |
| convex | "qhull error" | "qhull error" (convex inertia needs the hull) |

- A hull is needed iff a mesh geom has `contype`, `conaffinity`, or a `<pair>`, or the mesh is convex-inertia (`user_model.cc:4905-4922`). qhull refuses coplanar input (`user_mesh.cc:2040`).

**Ours today, measured** (main, untouched copy):
- Loads every row (`convex_hull = None`).
- Shell: mass 1000, I (83.333, 83.333, 166.667): MuJoCo's values in nalgebra's order. legacy/exact/convex: **mass 0, inertia 0 on a free body**.
- Collision, primitive dropped 1 mm onto the flat tile vs MuJoCo on a plane / 2 cm slab (`MIH/cases/flat/on_*`), 1,000 steps:

| geom | MuJoCo plane z | ours on flat mesh |
|---|---|---|
| sphere | 0.049632818 | 0.049632818 (equal) |
| capsule (upright) | 0.099632818 | 0.099632818 (equal) |
| box | 0.049892245 | **−19.59, falls through, ncon 0** |
| cylinder | 0.049857847 | **−19.59, falls through** |
| ellipsoid | 0.049632818 | **−19.59, falls through** |

- Cause, by reading: box/cylinder/ellipsoid vs mesh run GJK/EPA on the hull and return no contacts without one (`mesh_collide.rs:193-233`, `:280-323`, `warn!` only). Sphere/capsule use the triangle BVH.
- A zero-thickness planar hull (2-D hull polygon, faces both sides; `MIH_FLATHULL`) produces contacts, but box/cylinder/ellipsoid still tumble and fall through, at both centre and off-centre drops (measured). So it is not a fix.

**Verdict under Jon's condition: no test can show our result right for collision today.** That test is "box/cylinder/ellipsoid rest on a flat mesh tile at MuJoCo's plane height", and it fails on main and with the planar-hull attempt. Per §2 it comes back to Jon (Q5.1). Recommendation:
- **parity: refuse** a mesh whose hull is needed and degenerate. Message names the mesh and says the points are coplanar or collinear (MuJoCo: "qhull error").
- Refuse legacy/exact flat meshes with MuJoCo's "mesh volume is too small" (from §3's pipeline).
- **Load** the case MuJoCo loads (shell, no collision). Its inertia test passes with §2/§3: prototype (166.667, 83.333, 83.333), equal to MuJoCo's printed digits.

**Change.** The refusal is a post-geom pass in the builder: the needed-hull condition depends on the geoms, which are built after meshes (`process_mesh`, `mesh.rs:40-84`). Same loop as MuJoCo's `user_model.cc:4905-4922`, over `geom_type == Mesh` with `contype|conaffinity != 0`, in a pair, or `mesh_inertia_modes[id] == Convex`.

**Tests:**
- `flat_mesh_with_collision_is_refused`. Main: loads.
- `flat_shell_mesh_without_collision_loads`: 1000, (166.667, 83.333, 83.333), ipos (0.5, 0.5, 0), MuJoCo's iquat. Main: wrong order.
- `flat_mesh_legacy_volume_too_small`. Main: loads with mass 0.

**Flips:**
- `mesh_inertia_modes.rs:339` `t16_degenerate_mesh_shell` (collision on): becomes an error. Rewrite with `contype="0" conaffinity="0"` and MuJoCo's values.
- `exactmeshinertia.rs:354` `ac7_zero_volume_degenerate_mesh`: becomes an error. Rewrite as a refusal test; its `exactmeshinertia` attribute is a schema error at 3.5.0, A5.
- Both CI-run; the corpus doc for t16 is `8b802de5…`.

---

## 6. Found outside my items (hand-offs, all measured unless said)

1. **Box vs cylinder/ellipsoid/capsule falls through a box** (sim-core collision; untouched copy). On a 20 cm-thick box slab, dropped 1 mm, ours: cylinder z −19.43, ellipsoid −19.46, capsule −0.072 at 1,000 steps. A contact appears at step 20, then penetration grows. MuJoCo: 0.04755, 0.04963, 0.09963 (`MIH/cases/flat/on_slab10_*.xml`). Not isolated. **Not mine; serious.**
2. Ellipsoid shell inertia: ours (182716.6, 143035.5, 80350.2) vs MuJoCo (180751.0, 140683.1, 81026.3) for `size="1 2 3"`. `mesh_inertia_modes.rs:195` passes on a 2 % tolerance. Primitive inertia area.
3. A fromto geom under a rotated `<frame>` composes to another equivalent quat (2 corpus docs, §2.3). Frames area.
4. fusestatic moves geoms; MuJoCo accumulates body inertia with eig3 (`user_objects.cc:2239-2318`; `a6fb60bd…`, `68a35ea3…`). Compiler area.
5. discardvisual: MuJoCo's body mass still includes the discarded visual geoms (11.43 and 8.38), ours drops them first (4.19) (`5f4d4961…`, `b6ad1797…`). Compiler area.
6. Bodies whose only geom carries no MuJoCo mass (a plane): ours mass 1.0, inertia 0.001; MuJoCo mass 0, inertia 0, iquat = body frame (`02be4f10…`, `02eb5ff4…`, `11beabb1…`, `38a06052…`, `5456113f…`, `a9f02cd7…`). A5 §3.5.
7. Composite `curve="s"` cable bodies (`263f733b…`, 4 bodies). A4 curve map.
8. OBJ: 17 files with a face count ≠ MuJoCo's (first shape only, quads); MuJoCo's OBJ graph ids disagree with its `mesh_vert` (§4.1). OBJ coordinates are f32 in MuJoCo (tinyobj).
9. MuJoCo refuses fewer than 4 vertices for every mesh, not only vertex-only ones (`CheckInitialMesh`, `user_mesh.cc:1806-1808`). Ours with faces + 3 vertices: **not measured**. A5 D1.
10. Plane–mesh: graph path vs our scan, and contact at zero distance (§4.3). Collision area.

---

## 7. Commit list (this area; each compiles and carries its flips)

| # | commit | rows/items | after | flips (CI-run) |
|---|---|---|---|---|
| MH1 | `fix(cf-geometry): convex hull is convex: exact orientation predicates, no unreferenced vertices` | hull §4.2 | M2 (same file; M2's gate and expected list are measured on the float version) | none measured; licensed rung4/rung4b to run |
| MH2 | `fix(sim-mjcf): principal axes as MuJoCo (mjuu_eig3)` | principal-axis order §2 | M4: eig3 **is** M4's `principal_axes`, so merge MH2 into M4 or put it directly after with M4's non-finite guard on eig3's output | `mesh_inertia_modes.rs:218` t9 |
| MH3 | `fix(sim-mjcf): body inertial frame as MuJoCo: one geom copied, orientation alternatives, fromto axis from−to` | §2 bug + fromto | MH2; M19 (S7 moving-body mass), else `fluid_derivatives.rs:1391` t23 and `fluid_forces.rs:119,185` go NaN instead of refusing | t23 (if before M19) |
| MH4 | `fix(sim-mjcf): mesh mass properties as MuJoCo (legacy default, centroid apex, exact-mode orientation check)` | §3 | MH2 (mesh frame via eig3), M1 (error variants) | `:241` t10, `:52` t1 (rename) |
| MH5 | `fix(sim-mjcf,mesh-io): STL vertices deduplicated as MuJoCo; non-finite vertices refused; left-handed scale; binary STL by size` | STL dedupe §1 | MH1 (hull built once on deduplicated points), M1 | none measured |
| MH6 | `fix(sim-mjcf): a mesh that needs a hull and has none is an error` | flat mesh §5 | MH1, MH4, M23 (vertex-only hull uses the same check) | `:339` t16 (rewrite), `exactmeshinertia.rs:354` ac7 (rewrite) |

Expected-change lists:
- Corpus, MH2+MH3+MH4+MH5: 271 docs model_fp, 66 traj (`MIH/out/expected_change_traj.tsv`).
- MH1: 5 docs (`expected_change_exacthull.tsv`). The unreferenced-vertex removal was **not** run over the corpus.
- Submodule meshes: §1, §3, §4 counts.

Divergences-table rows:
- hull order/coplanar/maxhullvert (limitation);
- eig3 build dependence ("bit-exact to the arm64 wheel; within 4.1e-12 of an unfused build"; not a deviation);
- flat-mesh refusal (parity, none);
- ASCII STL / > 200,000 faces (per Q1.1).

## 8. Dependencies on other areas
- M1 error variants: non-finite vertex, invalid mesh (degenerate hull, volume too small, inconsistent orientation), STL format.
- M2 before MH1.
- M4 shares MH2.
- M15 (schema): `inertiagrouprange`, `refpos`/`refquat` (MuJoCo mesh attributes, `user_mesh.cc:1442-1468`; ours not checked).
- M19 before MH3.
- M23 after MH6.
- Frames area (§6.3), compiler area (§6.4–5), collision area (§6.1, §6.10).
- A6's verification protocol re-run per commit (the 53-doc mask here came from two runs, not ten).

## 9. Open questions (only ones the fixed decisions leave open)
- **Q1.1 STL formats MuJoCo cannot read.** (a) ASCII STL: lenient load (MuJoCo fails on valid input), with ASCII values rounded to f32 and a test that ASCII and binary encodings of one mesh give a bit-identical Model; or refuse (parity). (b) More than 200,000 faces: lenient load with a mass test; or refuse. (c) |coord| > 2^30: refuse (parity).
  - *Recommend* (a) lenient, (b) lenient, (c) refuse.
  - If wrong: (a)/(b) users get models MuJoCo would not load, listed as lenient deviations; (c) refuses meshes with coordinates above 2^30; in-tree effect **not measured**.
- **Q2.1 Fused vs plain eig3.** Fused is bit-exact to the wheel the repo tests against (20,211 / 20,211). Plain matches an unfused build by reading.
  - *Recommend* fused. The golden data's platform is not recorded in-tree (`gen_conformance_reference.py` header), so which build it came from is unknown.
  - If wrong: bit comparisons against an x86-64 MuJoCo differ at ≤ 4.1e-12 instead of 0.
- **Q2.2 Inherit eig3's error.** Its cosine stop leaves up to 1.4e-6 relative error in the reconstructed tensor, and up to 82 % in small-mesh principal moments (§2.3).
  - Options: (a) parity (eig3 everywhere); (b) an accurate eigensolver with MuJoCo's ordering, deviating where MuJoCo is inaccurate.
  - *Recommend (a)*: Jon asked to fix the divergence, and (b) needs its own divergence row plus a rule for choosing among equivalent frames.
  - If wrong: 301 of 2,520 submodule STL bodies keep inertia > 1e-3 away from exact, as in MuJoCo.
- **Q2.3 `inertiagrouprange`.** Implement it in MH3 (one comparison; MuJoCo's default 0…5 already applied in the prototype), or refuse it as a limitation.
  - *Recommend* implement.
  - If wrong: a model setting it loads with MuJoCo's semantics instead of failing.
- **Q3.1 Embedded vertices as f32.**
  - *Recommend* parity: round in the parser's mesh path; 2 corpus docs change at ~1e-8.
  - If wrong: masses differ from MuJoCo by ~1e-8 for decimal vertex data.
- **Q5.1 Flat mesh** (§5). *Recommend* refuse where MuJoCo needs a hull; load shell + no collision. The alternative is a real flat-mesh collision path (the planar-hull attempt failed) before any lenient load.
  - If wrong: t16 and ac7 stay loaded with boxes falling through.

## 10. Artifacts (`MIH/`) and repo state
- `prototype.patch` (the scratch copies `repo/`, `repo_main/`, all `target*/` dirs and the per-file binary dumps `out/dump_{dedupe,exact}/` were deleted at the end; re-create with `git archive HEAD` + `patch -p1` and the probe's `meshes` mode); `probe/`, `probe_main/`, `harness/` (sources).
- `scripts/` (gen_tensors, cmp_eig, cmp_port[_tol], recon_main, mj_files, cmp_files, cmp_files_mass, cmp_hull, hull_convexity, mj_mesh_mass_port, port_debug, ddmin_hull, synth_hulls, change_list, traj_cmp, mj_corpus_bodies, f32round_doc, legacy_default).
- `cases/` (euler2, single_rot, flat/, order/, stl/, mim/, synth8, modes/, mhv/).
- `out/` (eig/, mjfiles/, files_*, corpus_*, h_*, expected_change_*.tsv, val/, tests/*.log).
- Repo state: HEAD `ea11c8d6` at start; `139f7b25` appeared during the work (a docs-only commit to `RIGID_SPEC.md` by the caller; `git diff --stat 3520544e 139f7b25 -- ':!sim/docs'` empty). `git status --short` empty before and after my work.
