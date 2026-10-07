> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# r3 — convex, mesh and height-field collision vs MuJoCo 3.5.0

Researcher section, 2026-10-06. Repo read at branch `fix/rigid-sim-core-mjcf` @ `fbc0ad54`, whose source equals `main` @ `3520544e` (`git diff --stat 3520544e HEAD -- ':!sim/docs'` prints nothing). `R3/` = `SCRATCH/rigid_spec/r3_collision_convex/`. MuJoCo citations are 3.5.0 (`SCRATCH/mj350src/mujoco`, tag 3.5.0, 881544c) under `src/`. Oracle: `mujoco==3.5.0` (`SCRATCH/rigid_oracle_350/venv`, macOS arm64 wheel).

Three scratch copies (`git archive HEAD`, own `target/`):
- `R3/repo_base` — HEAD (= main).
- `R3/repo_fix` — HEAD + A15's fix (`isolate_fallthrough/out/fix.diff`) + A10's hull fix (the convex-hull part of `mesh_inertia_hull/prototype.patch`, exact predicates switched on unconditionally). Diff: `R3/out/fix_vs_base.diff`. **This is the starting point the brief names.**
- `R3/repo_ccd` ("port" below) — `repo_fix` + this section's prototype. Diff: `R3/out/port_vs_fix.diff` (2,769 lines; the port is `sim/L0/core/src/collision/mjccd.rs`). Env switches inside it (`R3_PORT`, `R3_HF`, `R3_HFNORM`, `R3_HFBOUND`, `R3_HFRAW`, `R3_MP`, `R3_MESHPRIM`) turn each change off for A/B; test hooks `R3_GRAPH`, `R3_MESHCENTER`, `R3_SETCONTACT` are measurement-only.

Probes: `R3/probe_src/probe.rs` (per-step trace, state replay, a single `mjc_ccd`/`mjc_Convex`/hfield call on given poses) and `R3/probe_src/census.rs` (the A16 census harness), built once per copy (`R3/probe_{base,fix,ccd}`). MuJoCo side: `R3/scripts/mj.py` (same JSON) and **`R3/scripts/mjccd_oracle.py`, which calls the wheel's exported `mjc_ccd`, `mjc_initCCDObj` and `mjc_Convex` through ctypes** on MuJoCo's own `mjModel`/`mjData`, so internal outputs (witness points, iteration counts, EPA status) can be compared, not just contacts.

## 0. Summary

| item | first differing quantity | cause (ours / MuJoCo) | fix | measured after |
|---|---|---|---|---|
| 1a. capsule–cylinder, census `3a13c90a`, `6369216c` | the contact itself at t0: dist, pos, normal (pos off by 0.45 m in `3a13c90a`; in `6369216c` dist off by 9.8e-8, normal by 3.5e-6, pos by 4.7e-5) | ours: analytic `collide_cylinder_capsule` (`collision/pair_cylinder.rs:149`) + cf-geometry GJK/EPA fallback (`narrow.rs:156-236`); MuJoCo: `mjc_Convex` (`engine/engine_collision_driver.c:41-52`) → native CCD `mjc_ccd` (`engine/engine_collision_gjk.c:2208-2357`) | **port `mjc_ccd` + `mjc_Convex`** | `6369216c` agrees in every census quantity; `3a13c90a` state agrees (traj ≤ 8.3e-13), only the tangent frame differs (A16 NEW-FRAME) |
| 1b. mesh–plane, census `1aab796f` | contact pos (vertex vs midpoint) and which vertices | ours `collide_mesh_plane` (`mesh_collide.rs:398`); MuJoCo `mjc_PlaneConvex` (`engine_collision_convex.c:1042-1138`) | port `mjc_PlaneConvex` | pos fixed; vertex choice at the exact tie still differs: **needs qhull's hull graph**. With MuJoCo's graph injected the doc agrees at t0 and step 1, then the Newton cluster (A16) |
| 2. MULTICCD | contact count and positions (4 slab corners vs MuJoCo's 5 rim points) | ours `multiccd_contacts` (`narrow.rs:284-327`); MuJoCo `mjc_Convex` perturbation search (`engine_collision_convex.c:930-998`) + native multi-contact (`engine_collision_gjk.c:2063-2187`) | port both | perturbation path bitwise; box/mesh multi-contact needs mesh polygons — **not ported** (§4.2) |
| 3. height fields | (a) prism heights: data not normalised, rows not flipped; (b) **no contact at all** over the field's −x/−y half | (a) `builder/mesh.rs:193-291` vs `user_objects.cc:4559-4573`, `xml/xml_native_reader.cc:3442-3449`; (b) hfield AABB/rbound from the corner-origin `HeightFieldData` (`builder/build.rs:580-589`, `design/cf-geometry/src/heightfield.rs:377-382`) vs MuJoCo's centred box (`user_objects.cc:3423-3426`, `:3576-3582`) — the broadphase culls the pair; (c) prism diagonal (`hfield.rs:235-241` vs `engine_collision_convex.c:1165-1182, 1314-1349`) | normalise + flip rows; MuJoCo's bounds; port `mjc_ConvexHField` | 10/10 bumpy fixtures rest (A15 fix: 7 of 10 fall through); contacts bitwise at identical poses after MuJoCo's frame renormalisation |
| 4. tumbling 3-axis ellipsoid | — (no collision difference left) | the A15 EPA's normal differs from MuJoCo's by up to 2.8e-2 rad at edges and corners (A15 §4.1) | the port | 40 resting pose-sweep cases: max \|Δz\| **1.0e-13** (A15: 1.05e-2) |
| 5. cylinder rocking | — at identical states; trajectories separate from **step 3** | first difference is `qvel` after the Euler update (6.9e-18), with `qacc` bit-identical: MuJoCo fuses `qvel += h·qacc` (`engine_util_blas.c:577`, via `engine_forward.c:912`), ours does not (`integrate/mod.rs:152,164,179`) — outside collision | (collision: the port) | contacts bitwise at 1,000/1,000 replayed MuJoCo states of the worst rocking case |

**Key finding behind the bitwise results.** A line-by-line Rust port of `mjc_ccd` with plain arithmetic did *not* reproduce the wheel: on `6369216c` it found a symmetric EPA face 120° away (witness `x2` = (3.19e-8, 1.84e-8, 0.2) vs MuJoCo's (3.2e-20, −3.69e-8, 0.2)). Compiling `engine_collision_gjk.c` myself (scratch-only, `R3/cbuild/`) with `-ffp-contract=on` reproduced the wheel bit for bit; with `-ffp-contract=off` it reproduced my plain port. The arm64 wheel fuses `a*b + c` inside each C expression (the same finding as A10 §2.3 for `mjuu_eig3`). With every such expression written as `f64::mul_add`, the port is bit-identical to the wheel on every input tried (§2.3).

---

## 1. The fixture is the subject

- **Census docs**: the three corpus docs are run by the A16 harness binary (`R3/probe_src/census.rs`, unchanged apart from dropping its second variant) and compared with A16's MuJoCo output (`parity_census/out/mj.jsonl`) by A16's `compare.py`. No model field differs (`model_diffs: []` for all three, `R3/out/cmp3_*.jsonl`).
- **Geom poses**: on `6369216c` at qpos0, `geom_xpos` and `geom_xmat` of all 7 geoms are **bit-identical** to MuJoCo's (`probe kin` vs `mj.py kin`, `R3/out/kin_6369_*.jsonl`). At general states they are not: on the sphere-like ellipsoid sweep case the body geom's `geom_xmat` differs by 2.2e-16 at replayed state 51 and 5.6e-16 at state 200 (same `qpos`, `R3/out/kin_*.jsonl`). Ours computes it as `(body_quat * geom_quat).to_rotation_matrix()` (`forward/position.rs:187-189`); MuJoCo copies `xmat` for a same-frame geom or uses `mju_quat2Mat` (`engine/engine_core_util.c:868-882`). Which of these produces the ulp **was not isolated**. Because of it, every bitwise comparison of the collision code below feeds **MuJoCo's own poses** into the port (probe mode `ccdbatch`), so collision is compared alone.
- **Instruments made to lie** (each failed when it should):
  - `ccdbatch_cmp.py`: one witness coordinate moved by 1 ulp → 1 of 1,000 calls flagged; the old A15 GJK path through the same replay → all 946 states with a contact differ, up to 1.3e-4 in pos (`R3/out/lie/`).
  - `convexbatch_cmp.py`: pair margin moved by 1 ulp → 293 of 300 calls flagged; geom order swapped (index order instead of MuJoCo's type order) → 297 of 300 flagged (`R3/out/lie/convex_lie*.jsonl`).
  - `statecmp.sh` (contacts at replayed MuJoCo states): its first version counted sign-flipped normals as differences (our contacts keep index order, MuJoCo orders by type, `engine_collision_driver.c:1475-1480`); it now orients ours to MuJoCo's order before comparing.

## 2. The port

### 2.1 What is ported (`R3/repo_ccd/sim/L0/core/src/collision/mjccd.rs`)

| MuJoCo 3.5.0 | port |
|---|---|
| `mjc_ccd` (`engine_collision_gjk.c:2208-2357`), incl. the sphere→point / capsule→segment shrink and `inflate` (`:2189-2205`) | `mjc_ccd` |
| `gjk` (`:171-297`), `gjkIntersect` (`:386-442`), `subdistance`/`S3D`/`S2D`/`S1D` (`:532-812`) | same names |
| `polytope2/3/4` (`:896-1157`), `attachFace`, `horizon` (`:1183-1271`), `epa` (`:1293-1438`), `epaWitness` (`:1273-1291`) | same names |
| `multicontact`, `polygonClip`, `polygonQuad`, box normals/faces (`:1442-2187`) | **box only**; the mesh branch needs mesh polygons (§4.2) |
| support functions `mjc_*Support` (`engine_collision_convex.c:159-455`), `mjc_initCCDObj` (`:709-769`), `mjc_prism_center` (`:119-127`) | `CcdObj::support`, `CcdObj::center` |
| `mjc_CCDIteration` (native branch, `:788-821`), `maxContacts` (`:889-914`), `mjc_Convex` incl. the MULTICCD perturbation search (`:917-1000`) | `ccd_iteration`, `max_contacts`, `mjc_convex` |
| `mjc_ConvexHField` (`:1185-1363`) with `mjc_penetration` (`:49-87`) | `mjc_convex_hfield` |
| `mjc_PlaneConvex`, mesh part (`:1042-1138`), with `mjccd_support`'s mesh branch (`:597-680`) | `mj_plane_mesh_contacts` (in `mesh_collide.rs`) |

Dispatch (`narrow.rs`, `mesh_collide.rs`, `hfield.rs`): every pair `mjCOLLISIONFUNC` sends to `mjc_Convex` (`engine_collision_driver.c:41-52`: sphere–ellipsoid/mesh, capsule–ellipsoid/cylinder/mesh, ellipsoid–ellipsoid/cylinder/box/mesh, cylinder–cylinder/box/mesh, box–mesh, mesh–mesh) goes through the port, **called in MuJoCo's type order**; the normal is negated when that order is the reverse of our geom-index order, so `Contact.geom1/geom2` keep today's convention (A15 Q6 stays open). Hfield pairs go through `mjc_convex_hfield`, plane–mesh through `mj_plane_mesh_contacts`.

### 2.2 Arithmetic: the reference build fuses multiply-adds

- Scratch builds of `engine_collision_gjk.c` (`R3/cbuild/libgjk_{on,off}.dylib`, plus a 20-line `extra.c` for the two non-exported support functions; diagnosis only, nothing of it goes anywhere near the repo), called with the same `mjCCDObj`s as the wheel on `6369216c`: `-ffp-contract=on` → every status field equal to the wheel's; `-ffp-contract=off` → `x2 = (3.1932485186907955e-08, 1.8436228918649534e-08, 0.2)`, which is what the plain Rust port printed (`R3/out/ccd1_*.json`).
- The port therefore writes each C expression of the form `a*b + c` (left product first, chains left to right) as `f64::mul_add`, also inside the support functions, `mju_normalize3`, `mju_mulMatVec3`, `mju_makeFrame`, `rotmat`, the S2D minors and the prism vertex `dx*c - size0`. Compound assignments `x += a*b` are fused too (`inflate`, the support margin).
- Bit-identity with the wheel is therefore a property of **this build** of MuJoCo. Its x86-64 build enables AVX without FMA (`cmake/CheckAvxSupport.cmake`, per A10 §2.3) — by reading, unfused there; not measured (no x86 MuJoCo here).

### 2.3 Measured agreement with the wheel, identical poses

| comparison | calls | result |
|---|---|---|
| `mjc_ccd`, census `3a13c90a` and `6369216c` at t0 | 2 | all status fields equal (ret, dist, nx, x1, x2, gjk/epa iterations, epa_status, nsimplex, simplex) |
| `mjc_ccd` on MuJoCo's 1,000 states of: rocking cylinder, sphere-like ellipsoid tumble, 3-axis ellipsoid tumble (pose sweep) | 3,000 | 3,000 bit-identical |
| `mjc_ccd` with a 0.01 margin (cylinder, 3-axis ellipsoid on a box) | 2,000 | 2,000 bit-identical |
| `mjc_Convex` (incl. MULTICCD perturbation) on the A15 `opt/` fixtures | 3,000 (6,964 contacts) | 3,000 bit-identical |
| `mjc_Convex` on 300 random poses × {plain, margin 0.02, MULTICCD} × 8 pairs (sphere–ellipsoid, capsule–ellipsoid, capsule–cylinder, ellipsoid–ellipsoid, ellipsoid–cylinder, ellipsoid–box, cylinder–cylinder, cylinder–box; `R3/scripts/gen_pairs.py`) | 7,200 (7,463 contacts) | 7,200 bit-identical |
| hfield pair through the full step (MuJoCo's contacts from `mj_forward`) on 6 hfield trajectories | 6,000 (12,787 contacts) | dist and pos bit-identical; normal 1 ulp off in 3,470 calls; **6,000 bit-identical once the normal is renormalised as `mj_setContact` does** (`engine_collision_driver.c:1418` → `mju_makeFrame`) |

The last row means the stored contact normal must pass through `mju_normalize3` once more, as MuJoCo's frame completion does. That belongs with A16's NEW-FRAME work (porting `mju_makeFrame`, `engine_util_spatial.c:508-534`); it is listed as a dependency (§12), not done here.

---

## 3. Item 1 — census clusters NEW-CONVEX (2 docs) and NEW-MESHPLANE (1 doc)

### 3.1 Capsule–cylinder (`3a13c90a` = `integration/collision_primitives.rs:1091`; `6369216c` = `integration/collision_performance.rs:292`)

t0 contact, ours (base = fix for `3a13c90a`) vs MuJoCo (normal oriented to our geom order):

| doc | | dist | pos | normal |
|---|---|---|---|---|
| `3a13c90a` (capsule through a cylinder, deep) | base, fix | −0.45 | (0.525, 0, 0) | (1, 0, 0) |
| | **port** | −0.4499898833045849(**0**) | (0.074996945802, −0.00042875385, 5.94006e-7) | (0.999954606627, −0.009528077486, 2.06e-5) |
| | MuJoCo | −0.4499898833045849 | same | same |
| `6369216c` (cap on cap, coaxial) | fix | −0.03999990153 | (4.7e-5, −6.9e-6, 0.18000005) | (−9.8e-7, 2.6e-6, 1) |
| | **port** | −0.03999999999998301 | (0, −1.8425e-8, 0.18) | (0, −9.22372e-7, 1) |
| | MuJoCo | −0.03999999999998301 | same | same |

- MuJoCo's own answer on `3a13c90a` is 1.0e-5 short of the geometric depth 0.45 and its normal 9.5e-3 rad off the x axis. That is EPA's stopping tolerance (`ccd_tolerance` 1e-6) on a deep, curved contact; reproducing it needs MuJoCo's EPA, not a better one.
- `6369216c` is a symmetric tie: three equivalent EPA faces 120° apart (`polytope2`'s 120° rotation, `engine_collision_gjk.c:859-875`). Only the fused arithmetic picks MuJoCo's face (§2.2).
- Census with the port (`R3/out/cmp3_ccd.jsonl`): `6369216c` no differing quantity at t0, step 1 or step 100, trajectory ≤ 6.3e-15. `3a13c90a` trajectory ≤ 8.3e-13, remaining difference `con_frame` (2.0e-7: tangent `t1`; A16 NEW-FRAME).

### 3.2 Mesh–plane (`1aab796f` = `integration/collision_plane.rs:1493`; A16 marks it hash-order nondeterministic)

A unit-cube mesh 0.1 deep in a plane, axis-aligned, so the four bottom vertices tie exactly.

| | contact z | vertices chosen (x, y) |
|---|---|---|
| base / fix | −0.1 (the vertex) | (−.5,−.5), (.5,−.5), (.5,.5) |
| port | −0.05 (vertex − ½·dist·n, `engine_collision_convex.c:1063`; extra contacts in `addplanemesh` `:1008-1039`) | (−.5,−.5), (.5,.5), (.5,−.5) |
| MuJoCo | −0.05 | (−.5,.5), (.5,.5), (.5,−.5) |

- What MuJoCo does differently, by reading, all ported: first contact = support vertex along −n by hill-climbing the hull graph from local vertex 0 (`mjccd_support`, `:624-670`); extra contacts only among that vertex's **graph neighbours**, in graph edge order (`:1109-1135`); the 0.3·rbound separation test is against the **first** contact only (`addplanemesh`, `:1008-1039`); at most 3 (`maxplanemesh`, `:1004`). Ours scans all vertices, sorts by depth and tests separation against every accepted contact (`mesh_collide.rs:398-483`).
- The remaining difference is the tie: which of four equal-depth vertices the hill climb reaches, and the neighbour order. Both come from qhull's graph (`mesh_graph`; MuJoCo's local vertex 0 is (−.5, .5, −.5), `R3/out/mp_graph.txt` from `R3/scripts/mjgraph.py`). **With MuJoCo's graph injected (`R3_GRAPH`)**: the census doc agrees at t0 and step 1; its trajectory first exceeds 1e-9 at step 16 (4.2e-9). At step 15 the contacts agree to 1.1e-16 and `qfrc_constraint` differs by 1.1e-5 relative (`R3/out/cmp_1aab_k.jsonl`) — A16's NEW-NEWTON cluster, not collision.
- Untied variants (`R3/xml/mp/tilt*.xml`, body tilted 3°/2°/1°): with MuJoCo's graph the three contacts are equal to MuJoCo's to the printed 9 digits at step 1; the step-1 `qfrc_constraint` differs 1e-4 relative and shrinks to 2e-8 with `iterations=1000 tolerance=1e-15` in both engines (`R3/out/mp_tilt*_tight_*`) — again the Newton cluster.
- Without MuJoCo's graph the tilted case picks a different second vertex ((−.5,−.5) instead of (−.5,.5)): our hull's adjacency lacks the diagonal qhull chose. So plane–mesh parity is **Jon's hull-order decision** (RIGID_SPEC §2: keep our order, list the limitation): set-equal hulls are not enough when the graph decides.

## 4. Item 2 — MULTICCD

### 4.1 Perturbation search (every non-box/mesh pair): ported, bitwise

MuJoCo, with `<flag multiccd="enable"/>` and no margin, gives box–box 8 contacts in one pass and box/mesh–box/mesh 4 (`maxContacts`, `:889-914`); every other pair gets 1 and, unless a geom is a sphere or ellipsoid, 4 more CCD calls with both geoms rotated ±1e-3 rad about the contact tangents, keeping contacts more than 1e-3·min(rbound) apart (`:930-998`).

A15 fixtures (`R3/xml/opt/`), 1,000 steps, max \|Δ(qpos,qvel)\| vs MuJoCo:

| fixture | main | A15 fix | port |
|---|---|---|---|
| cylinder on box, MULTICCD | 5.7e-3 (4 contacts at the slab corners; MuJoCo 5) | 5.7e-3 | **1.7e-12** (5 contacts; first > 1e-12 at step 735) |
| 3-axis ellipsoid on box, MULTICCD | 14.5 (falls through) | 2.3 | **1.1e-13** (1 contact) |
| 3-axis ellipsoid, box margin 0.01 (A15 Q5) | — | 0.30 | **7.5e-14** |
| cylinder, box margin 0.01 | — | 1.13 | 1.39 (bitwise at identical poses, §2.3; trajectory not) |

The cylinder-with-margin trajectory separates from step 8 although its `mjc_ccd` and `mjc_Convex` calls are bit-identical on MuJoCo's states. Its contact point jumps across the cap (flat cap parallel to the face, inside the margin). That it is the §7 integrator ulp amplified by this is **not measured**.

### 4.2 Box/mesh multi-contact: needs mesh polygons — not ported

- MuJoCo's `multicontact` (`engine_collision_gjk.c:2063-2187`) clips the two touching faces. For a mesh it reads `mesh_polyadr/polynormal/polyvert/polymap` (`:1695-1817`, `:1989-2011`), built at compile by `mjCMesh::MakePolygons` (`user_mesh.cc:2949-3005`, merging coplanar hull triangles) and `MakePolygonNormals` (`:2722-2731`). We have no polygon data.
- Measured gap (box on a mesh slab with MULTICCD, `R3/out/mesh/*_mccd*`): MuJoCo 4 contacts in 993 of 1,000 steps; port 1 contact (mesh side gives no normals, so `multicontact` returns); A15 fix 4 contacts at face corners, trajectory up to 8.8 m off.
- The port has the box branches (box–box never reaches `mjc_Convex`: MuJoCo's table sends it to `mjc_BoxBox`). So the box branch is exercised only once mesh polygons exist.
- `sim/L0/core/src/collision/mod.rs:1367` `test_multiccd_enabled_multiple_contacts` (mesh–mesh, MULTICCD, expects > 1 contact) **fails on the port**: 1 contact. The commit that routes box/mesh pairs through the port must carry the polygon port (Q4).

## 5. Item 3 — height fields

Fixtures: A15's 8×8 flat/bumpy fields and constant field (`R3/xml/hf/`), plus non-square fields (`hfns_*`). 1,000 steps.

### 5.1 Four defects, isolated one at a time

| # | defect | ours | MuJoCo 3.5.0 | measured effect |
|---|---|---|---|---|
| a | elevation not normalised | `builder/mesh.rs:253` (`e * size[2]`) | `data -= min; if (max − min > mjEPS) data /= (max − min)`, in float (`user_objects.cc:4559-4573`) | constant 0.5 field: sphere rests at 0.0996 (main), 0.049633 = MuJoCo (port) |
| b | row order | file order | inline data reversed, "XML string is top-to-bottom" (`xml/xml_native_reader.cc:3442-3449`); PNG rows reversed (`user_objects.cc:4444-4448`) | before: normal's y component mirrored (A15 §4.4); after: contacts bitwise |
| c | **bounds** | AABB/rbound from `HeightFieldData::aabb()` (`builder/build.rs:580-589`), which is corner-origin, x ∈ [0, 2·size0] (`cf-geometry/src/heightfield.rs:377-382`) | rbound √(s0²+s1²+max(s2²,s3²)), box [−s0,−s1,−s3, s0,s1,s2] (`user_objects.cc:3423-3426`, `:3576-3582`) | **the fall-through**: the broadphase culls the pair once the body's box leaves the +x/+y quarter; table below |
| d | prism triangulation | cell split along (c+1,r)–(c,r+1) (`collision/hfield.rs:235-241`) | sliding 3-vertex window, split along (c−1,r)–(c,r+1) (`engine_collision_convex.c:1165-1182`, loop `:1314-1349`) | covered by the port; not isolated separately |
| e | non-square cells resampled | `builder/mesh.rs:255-288` (> 1 % dx/dy difference → bilinear resample to a square grid) | keeps the grid; `dx`, `dy` from size and grid (`:1306-1307`) | 5×9 field: sphere falls through on main (−19.44); 0.060318 vs MuJoCo 0.060317 without resampling |
| f | PNG scaled `/65535`, rows not flipped | `builder/mesh.rs:304-335` | min/max-normalised like (a), rows flipped | not measured (no PNG fixture; 4 corpus docs use `file=`) |

**Fall-through isolated** (bumpy field, 6 fixtures, final z; MuJoCo 0.045–0.073; `R3/out/hf/`, env toggles on one binary):

| configuration | falls through |
|---|---|
| A15 fix hfield code, data as today, our bounds | 5 of 6 |
| A15 fix hfield code, data as today, **MuJoCo's bounds** | **0 of 6** (final z within 1.3e-2 of MuJoCo) |
| port + normalised/flipped data, our bounds | 4 of 6 |
| port + normalised/flipped data + MuJoCo's bounds | 0 of 6 |

### 5.2 Result with all four

- Bumpy 10/10 and flat 10/10 rest. Bumpy max \|Δz\| ≤ 4.9e-3, flat ≤ 1.8e-2 (box tilted). Before (A15 fix): 7 of the 10 bumpy fixtures (sphere ×2, capsule ×2, ellipsoid ×2, tilted cylinder) fall to z ≈ −8…−14; on main, the two bumpy fixtures run there fall too (sphere up −14.40, tilted capsule −10.21) and the constant field rests at 0.0996.
- At MuJoCo's own 1,000 states, contact **counts** agree in every state of every replayed case (8 cases, `R3/out/hf/*.sc_ccd.ours.jsonl`; before the bounds fix 466–768 states per case differed in count).
- With MuJoCo's poses, the hfield pair is bitwise after the `mj_setContact` normalisation (§2.3).

## 6. Item 4 — the tumbling 3-axis ellipsoid

- A15 §4.3: 1.05e-2 m over the run, 0.65 rad final orientation, worst cases `e45_45`/`e90`; A15 attributed nothing ("not isolated").
- The pose sweep re-run with the port (`R3/out/sweep_ccd_pose.tsv`, 480 cases, A15's `sweep.py`/`classify.py`/`agree.py`): all 480 have MuJoCo's outcome. For the 40 resting 3-axis-ellipsoid cases: max \|Δz\| over the run **1.0e-13**, final orientation ≤ 7.3e-8 rad. Size sweep (288 cases, `sweep_ccd_size.tsv`): 3-axis ellipsoid ≤ 2.6e-12 (A15 fix: 4.4e-6).
- The first differing quantity *before* the port was therefore the convex contact. With the port, `mjc_ccd` on MuJoCo's 1,000 states of the worst case (`ellipsoid3_…_e45_45_0_xy0_0_boxfirst`) is bit-identical in all 1,000.
- The **sphere-shaped** ellipsoid (radii equal) still ends 2.7e-2 rad off in orientation in its worst case (rest \|Δz\| ≤ 2.2e-7). On MuJoCo's states its contacts differ from MuJoCo's by up to 5.2e-6 in pos when *our* kinematics compute the poses, and are bit-identical when MuJoCo's poses are fed in. So the residual enters through the `geom_xmat` ulp (§1), amplified by a contact whose normal is ill-conditioned on a sphere. Not isolated further.

## 7. Item 5 — MuJoCo's cylinder rocks on its cap

- What "the same thing for the same reason" can mean measurably: at identical states, our contact is MuJoCo's. On MuJoCo's 1,000 states of the worst rocking case (`cylinder_…_d0.05_e0_0_0_xy0_0_bodyfirst`), the port's `mjc_ccd` output is bit-identical in 1,000 of 1,000.
- In both engines' own runs the single contact wanders across the cap: it changes cap quadrant 457 times in MuJoCo's run and 412 in the port's (`cylinder_…_d0.001_e0_0_0_xy0_0_boxfirst`), 308 and 371 (`…_d0.05_…_bodyfirst`) (`R3/out/sweep_ccd_pose/`). The counts differ because the trajectories separate.
- Why the witness wanders: **not isolated**. One hypothesis was tested and is **false**: the contact is not on the rim (2–4 of ~990 contact points lie within 1e-6 of radius 0.05, both engines).
- Why the trajectories separate: state bits first differ at **step 3** by 6.9e-18, with `qacc` and `qfrc_constraint` bit-identical at that step. The differing element is `qvel[2]`: MuJoCo's value equals `fma(qacc, h, qvel)`, ours `qvel + h·qacc` (`R3/out/sweep_ccd_pose/cylinder_…`, checked with exact rationals). Sphere cases show the same step-3 difference. It is in `integrate/mod.rs:152,164,179` vs `mju_addToScl` (`engine_util_blas.c:577`; scalar path on arm64, AVX path on x86 adds without fusing, by reading). **Outside this area** (hand-off, §12).
- Resting cylinder after the port: pose sweep max \|Δz\| 1.3e-4 (A15 fix 7.8e-4); size sweep 3.2e-4 (A15 fix 4.4e-4).

## 8. Found alongside (measured unless said)

1. **Mesh pairs need MuJoCo's mesh frame.** MuJoCo stores mesh vertices re-centred at the mesh centre of mass, rotated to principal axes, as `float` (`user_mesh.cc:1676-1686`, A10 §1), and moves the geom frame there (`user_objects.cc:3749-3754`). The CCD's GJK starts from the geom centre (`mjc_center`, `engine_collision_convex.c:95-116`), which is therefore the mesh's centre of mass in MuJoCo and our user frame origin. On an f32-exact slab whose principal frame is the identity (`R3/xml/mesh/slab2_*`), moving only the start point (`R3_MESHCENTER="0 0 -0.5"`) takes the trajectory difference from 0.59 to 1.3e-4 (upright cylinder), 2.0 to 2.2e-4 (tilted cylinder), 0.55 to 1.5e-12 (tilted 3-axis ellipsoid). On A15's slab (z = −0.2, not f32-exact) MuJoCo's vertex z is −0.20000000298 (`R3/out/mesh/slab_graph.txt`), so that case also needs f32 vertices. A10's area (Q2).
2. **Exhaustive mesh support iterates all mesh vertices in file order** when the mesh has fewer than 10 vertices (`mjMESH_HILLCLIMB_MIN`, `engine_collision_convex.c:728-735` with `mjc_meshSupport` `:336-379`); the port follows that.
3. **Exact touch gives no contact in MuJoCo's convex path** (`dist < 0` required, `engine_collision_convex.c:804`). Measured with MJCF copies of the fixtures of `collision/mod.rs:1535` and `:1731` (`R3/xml/flip/`): MuJoCo 0 contacts, port 0, A15 fix 1 (depth −0.0).
4. **Sphere and capsule against a mesh**: MuJoCo collides them with the hull (`mjc_Convex`); ours uses the triangle BVH (`mesh_collide.rs:184-193`, `:271-280`). The port routes them to the hull. A15's convex-slab capsule case: 1.29 → 3.1e-14; the f32-exact slab: 1.68 → 1.9e-14. Non-convex meshes now collide as their hull, as in MuJoCo.
5. **`<flag nativeccd="disable">` is a silent no-op** (`builder/mod.rs:946-947`, comment `:972`). MuJoCo switches to libccd MPR plus `mjc_fixNormal` (`engine_collision_convex.c:822-853`, `:1473`). 0 corpus docs set it (grep). Q5.
6. Box–box and capsule–box stay outside this area: their pose-sweep rest differences (6.3e-4, 8.1e-8) are unchanged by the port (they never reach it).

## 9. Bit-identity elsewhere and census

**Corpus, streaming harness** (`isolate_fallthrough/harness_fix/src/bin/harness2.rs` rebuilt against `repo_fix` and `repo_ccd`, ×3 each, under the 2.5 GB RSS watchdog, every child exit code 0, none killed; `R3/out/corpus/cmp_emb.txt`):
- 1,584 docs: **1,517 unchanged**, 59 nondeterministic within a variant (59 of 59 are in the 71-doc mask `parity_census/nondet71.txt`), **8 changed**:
  - trajectory, 4: `1aab796f` (Mesh–Plane), `3a13c90a` and `45d3e0ec` (Capsule–Cylinder), `87d99b20` (`sim/L1/bevy/examples/collision_shapes.rs:31`: Cylinder–Cylinder, Ellipsoid–Sphere among others). Each has a pair type the port takes over;
  - model only, 4: `26d223bf`, `3204f47a`, `bcec7642` (the three corpus docs with an inline hfield and no contact in 100 steps: §5 builder and bounds changes) and `9a51402a` (`flex_unified.rs:1056`, no hfield, in A6's nondeterminism mask; model_fp stable within each binary, different between them — attributed to ledger-L27 by the mask, not by a cause found here).
- Unchanged although touching a pair type the A15 harness lists as affected: 12 (9 Box–Capsule, which the port does not touch; `6369216c` Capsule–Cylinder; 1 flex–Sphere, 1 flex–Box).
- 15 repo `.xml`: 15 unchanged.

**Census** (A16 pipeline: `compare.py` + `classify.py` on all 1,214 both-load docs, `R3/out/cls_{fix,ccd}.jsonl`): class changes fix → port are exactly 2 — `3a13c90a` state → api, `6369216c` api → agree. **No doc leaves "agree"** (796 → 797 agree; model 192, api 94 unchanged; state 132 → 131).

## 10. Tests and examples

**Suites, fix copy vs port copy** (`R3/out/tests_{fix,ccd}/`, `RAYON_NUM_THREADS=1`, own target dir):

| suite | fix | port |
|---|---|---|
| cortenforge-sim-core `--lib` | 720 pass | **716 pass, 4 fail** |
| cortenforge-sim-mjcf (lib, forward_conformance, doc) | 385 + 1 + 1 (3 ignored) | identical |
| sim-conformance-tests `integration` | 1,338 / 27 ignored | **1,337, 1 fail** |
| sim-conformance-tests `mujoco_conformance` | 83 | 83 |
| cortenforge-geometry | 207 lib + binaries | identical |

**Flips** (all CI-run):

| test | asserts | port | MuJoCo | action |
|---|---|---|---|---|
| `integration/collision_primitives.rs:1090` `cylinder_capsule_perpendicular` (asserts at `:1124`) | depth 0.45 ± `DEPTH_TOL` (geometric) | 0.44998988330458495 | 0.4499898833045849 (= port bitwise) | rewrite to MuJoCo's value |
| `core/src/collision/mod.rs:1367` `test_multiccd_enabled_multiple_contacts` | > 1 contact, mesh–mesh MULTICCD | 1 | 4 per the test's own note ("EGT-7"); this fixture **not run in MuJoCo** | passes again with the polygon port (§4.2, Q4) |
| `mod.rs:1535` `test_mesh_cylinder_not_capsule_regression` | a contact at exact touch | 0 | 0 (§8.3) | rewrite: penetrate by a margin, keep the capsule-vs-cylinder intent |
| `mod.rs:1731` `test_mesh_ellipsoid_not_sphere_regression` | a contact at exact touch | 0 | 0 | same |
| `mod.rs:1602` `test_mesh_cylinder_no_hull_graceful` | no contact when the mesh has no hull | a contact (port uses MuJoCo's no-graph exhaustive support, `engine_collision_convex.c:336-379`) | MuJoCo always has a hull for a colliding mesh (qhull is mandatory, A10 §5) | rewrite to MuJoCo's no-graph semantics (contact from the vertex set), or keep "no hull → none" as a deviation |

**Validators**: all 22 `example_kind = "validator"` packages under `examples/fundamentals/sim-cpu`, `cargo run --release`, fix copy vs port copy (`R3/out/val_{fix,ccd}/`). Fix: 22 exit 0. Port: 21 exit 0 with stdout identical to the fix; **`example-mesh-collision-stress-test` exits 1, 18/21 PASS** (CI's validate-examples would go red):

| check | fix | port | MuJoCo (same scene as MJCF, `R3/xml/val/`) |
|---|---|---|---|
| 6. mesh–box rest height, MULTICCD (`stress-test/src/main.rs:244-300`) | z 0.0499, ncon 4 | **FAIL** z 0.0346, ncon 1 | z 0.0499, ncon 4 |
| 16./17. mesh–mesh wedge on platform, MULTICCD (`:493-560`) | vz 0.0002, z 0.0999 | **FAIL** vz −0.0275, z 0.0872, ncon 1 | vz 0, z 0.0999, ncon 4 |
| 12. cylinder on mesh, MULTICCD contact count | 4 | 5 | not run |
| 7, 9 force ratios, 8 rest height | pass | pass, other values | — |

Checks 6, 16, 17 are the box/mesh multi-contact gap of §4.2: MuJoCo rests on 4 polygon-clipped contacts, the port has 1. They pass again only with C2 (§12).

Method note: the runner's first exit codes were the RSS watchdog's own (always 0); the child's code is in the watchdog's JSON line, which is what the counts above use.

**Not run**: sim-gpu, sim-thermostat, sim-urdf, cf-design(-tests), L1 crates, the Bevy apps (`examples/fundamentals/sim-cpu/mesh-collision/*`, `raycasting/heightfield`), licensed gates, grade, clippy/fmt on the prototype (it carries env switches and is not lint-clean code), the 24 ignored golden-flag tests (incl. `golden_flags.rs:325` `golden_disable_nativeccd`, see Q5).

## 11. Per item: Now / Target / Change / Tests / Downstream

### R3-A. Native convex collision (items 1a, 2, 4, 5)

- **Now:** `narrow.rs:58-270` dispatch: analytic capsule–cylinder with a GJK fallback (`:156-168`, `pair_cylinder.rs:149`), cf-geometry GJK/EPA for the rest (`:170-236`, `gjk_epa.rs:573`), MULTICCD = support-face corners of A (`:284-327`), margin zone = `gjk_distance` (`:234-267`); mesh hull pairs via `gjk_epa_shape_pair` (`mesh_collide.rs:26`); sphere/capsule–mesh via the triangle BVH.
- **Target:** `mjc_Convex` → `mjc_ccd` (`engine_collision_convex.c:917-1000`, `engine_collision_gjk.c:2208-2357`), computed in MuJoCo's type order, arithmetic contracted as the reference build.
- **Change (all `pub(crate)` inside `sim_core::collision`; no public signature changes):**
  ```rust
  // sim/L0/core/src/collision/mjccd.rs
  pub(crate) struct CcdObj<'a> { /* geom type, pos, mat (row-major [f64; 9]), size, margin,
                                    mesh: Option<MeshRef<'a>>, prism: [[f64; 3]; 6], vertindex, meshindex */ }
  pub(crate) struct MeshRef<'a> { verts: &'a [Point3<f64>],                         // all vertices, file order
                                  hull: Option<(&'a [Point3<f64>], &'a [Vec<u32>])> } // hull vertices + graph
  pub(crate) struct CcdConfig { max_iterations: usize, tolerance: f64, max_contacts: usize, dist_cutoff: f64 }
  pub(crate) struct CcdStatus { dist: f64, x1: [[f64; 3]; MAXCONPAIR], x2: [[f64; 3]; MAXCONPAIR], nx: usize, /* … */ }
  pub(crate) fn mjc_ccd(config: &CcdConfig, status: &mut CcdStatus, o1: &mut CcdObj, o2: &mut CcdObj) -> f64;
  pub(crate) struct RawContact { dist: f64, pos: [f64; 3], normal: [f64; 3] }
  pub(crate) fn mjc_convex(ccd_iterations: usize, ccd_tolerance: f64, multiccd: bool,
                           o1: &mut CcdObj, o2: &mut CcdObj, rbound1: f64, rbound2: f64, margin: f64) -> Vec<RawContact>;
  // narrow.rs
  pub(crate) const fn mj_type_rank(t: GeomType) -> u8;          // mjtGeom order
  pub(crate) fn mj_convex_pair(a: GeomType, b: GeomType) -> bool; // mjCOLLISIONFUNC == mjc_Convex
  ```
  The prototype uses `Vec`s inside the polytope (one allocation set per call). MuJoCo uses thread-local static arrays of 6·170 faces (`:2210-2215`). Allocation cost was not measured; a per-`Data` arena is the obvious home if it matters.
  Delete: `collide_cylinder_capsule` and its fallback, `multiccd_contacts`, `support_face_points`, `gjk_epa_shape_pair`, the margin `gjk_distance` branch. A15's F1 (EPA witness) still lands: `flex_narrow.rs:177` and cf-geometry's own callers keep using cf-geometry EPA. A15's F2 (capsule–box) is unaffected.
- **Tests to add** (each fails on the fix copy unless said; MuJoCo values from the wheel):
  1. `mjc_ccd_matches_mujoco_bitwise` (sim-core unit): a table of (geom types, sizes, poses, margin) → (dist, x1, x2, gjk/epa iterations) generated by `R3/scripts/mjccd_oracle.py`. Covers the 8 pair types × {plain, margin, MULTICCD}, the two census docs and tie cases. New function, so no "main" run. It was made to fail: a 1-ulp change flags it (§1); the unfused build flags `6369216c`.
  2. `capsule_cylinder_deep_matches_mujoco` (rewrites `collision_primitives.rs:1090`): `3a13c90a`, dist −0.4499898833045849, pos (0.074996945802, −0.00042875385, 5.94006e-7), to 1e-12. Main: −0.45 / (0.525, 0, 0).
  3. `capsule_cylinder_coaxial_tie_matches_mujoco`: `6369216c` t0, dist −0.03999999999998301 and normal (0, ±9.22372e-7, ∓1) to 1e-15. Fix: normal (−9.8e-7, 2.6e-6, 1).
  4. `multiccd_cylinder_on_box_as_mujoco`: `opt/cylinder_multiccd.xml` at a recorded state, 5 contacts at MuJoCo's positions. Main and fix: 4 at the slab corners.
  5. `multiccd_skips_ellipsoids`: `opt/ellipsoid3_multiccd.xml`, 1 contact. Main: falls through (14.5 off MuJoCo after 1,000 steps); A15 fix: 4 contacts at the slab corners.
  6. `ellipsoid3_tumble_rests_as_mujoco`: sweep case `ellipsoid3_hx0.5_hz0.1_s0.05_d0.05_e45_45_0_xy0_0_boxfirst`, 1,000 steps, \|Δz\| ≤ 1e-9 against MuJoCo's trajectory. Port 1.0e-13; A15 fix up to 1.05e-2.
  7. `convex_contact_at_exact_touch_is_absent`: §8.3 fixtures, 0 contacts. Fix: 1.
- **Flips:** §10.
- **Downstream:** none outside sim-core calls the replaced functions (`git grep` for `collide_cylinder_capsule`, `multiccd_contacts`, `gjk_epa_contact`, `support_face_points`, `GjkContact`, `geom_to_shape`: no hits outside `sim/L0/core/src`). Behaviour changes reach every crate that steps a model with these pairs: `87d99b20` (Bevy example) changes; the 22 validators in §10.

### R3-B. Plane–mesh (item 1b)

- **Now:** `mesh_collide.rs:398-483` (`collide_mesh_plane`), called from `:235`, `:325`.
- **Target:** `mjc_PlaneConvex` (`engine_collision_convex.c:1042-1138`).
- **Change:** `fn mj_plane_mesh_contacts(model, geom1, geom2, pos1, mat1, pos2, mat2, margin) -> Vec<Contact>` (private) replaces it in the dispatcher. `collide_mesh_plane` stays a `pub` function only if kept for the doc references at `examples/…/mesh-on-plane/src/main.rs:5` and `integration/mesh_contact_force_diagnostic.rs:170` (both comments; no caller) — recommend delete and fix the comments.
- **Tests:** `mesh_plane_contacts_at_midpoint`: `1aab796f` at t0, 3 contacts at z = −0.05 (as a set over the tied vertices). Main: z = −0.1.
- **Limitation:** tie order from qhull's graph (Jon's hull-order decision). Divergences row: "plane–mesh and mesh support choose among tied vertices by our hull graph, not qhull's".

### R3-C. Height fields (item 3)

- **Now:** §5.1 a–f.
- **Target:** MuJoCo's data (a, b, e, f), bounds (c), `mjc_ConvexHField` (d).
- **Change:**
  - sim-mjcf `builder/mesh.rs` `convert_mjcf_hfield`: parse as `f32`, flip rows, normalise as MuJoCo, no resampling. `load_hfield_png` the same (rows flipped, min/max-normalised).
  - Bounds: hfield `geom_rbound`/`geom_aabb` from `hfield_size`. Home: K6's `Model::recompute_derived` if geom radii move there (RIGID_SPEC §3 #8), else `builder/build.rs:580-589`.
  - sim-core `hfield.rs`: `collide_hfield_multi` body → `mjc_convex_hfield`; delete `build_prism` and `compute_local_aabb`.
  - Non-square grids need `HeightFieldData` to hold dx ≠ dy (Q6). Its consumers: `collision/hfield.rs`, `flex_narrow.rs:205`, `sdf_collide.rs:226`, `raycast.rs:298`, `builder/build.rs:582`.
- **Tests to add:**
  1. `hfield_rows_bottom_to_top` (sim-mjcf): 2×2 field `elevation="0 0 1 1"` → data row 0 = (1, 1). Main: (0, 0).
  2. `hfield_elevation_normalised`: constant 0.5 field, sphere rests at 0.0496328 (MuJoCo 0.049633). Main 0.09964.
  3. `hfield_bounds_are_centred`: `geom_rbound` = √(s0²+s1²+max(s2²,s3²)) and the centred box. Main: corner-origin.
  4. `bodies_rest_on_bumpy_hfield_as_mujoco`: the 10 bumpy fixtures, no fall-through, final z within 5e-3 of MuJoCo. Main: e.g. sphere −14.40, tilted capsule −10.21.
  5. `hfield_contacts_match_mujoco_bitwise`: recorded poses → contacts (golden from the wheel). Fix copy: differs (surface).
  6. `nonsquare_hfield_as_mujoco` (if Q6 = keep the grid): `hfns_sphere`, final z 0.060317 ± 1e-5. Main −19.44.
- **Flips:** none in the suites run (sim-mjcf's `builder/mod.rs:1247` `test_hfield_inline_elevation_still_works` passes). The 3 hfield corpus docs change model only (§9). `example-raycasting-stress-test`: exit 0, stdout identical to the fix copy (§10).
- **Downstream:** raycasting reads `hfield_data` (`raycast.rs:298`), so the flip and normalisation change raycast hits on MJCF height fields. `integration/raycast_heightfield.rs` passes on the port (in the 1,337). `examples/…/raycasting/heightfield` replaces `hfield_data` after load (`main.rs:203-204`), so only its bounds change; that Bevy app was not run.

### R3-D. Mesh frame for collision (found, §8.1) — dependency, not done here

Recommend that A10's mesh processing adopt MuJoCo's mesh frame for collision too (Q2).

## 12. Commit list (proposed) and PR placement

| # | PR | commit | contents | carries | after |
|---|---|---|---|---|---|
| C1 | Rigid-physics | `feat(sim-core): MuJoCo's native convex collision (mjc_ccd) ported` | `collision/mjccd.rs` (GJK, EPA, polytopes, witness, box multi-contact), unused by dispatch | test 1 (bitwise table) | — |
| C2 | Rigid-physics | `feat(sim-core): mesh polygons and MuJoCo's mesh multi-contact` | `MakePolygons`/`MakePolygonNormals` port on the hull (`sim-core/src/mesh.rs` hull post-processing), mesh branches of `multicontact` | a box-on-mesh MULTICCD test vs MuJoCo; **not prototyped** | C1 |
| C3 | Rigid-physics | `fix(sim-core): convex pairs collide through mjc_Convex as MuJoCo` | dispatch in `narrow.rs`, `mesh_collide.rs`; deletions (§11 R3-A) | tests 2–7; flips `collision_primitives.rs:1090`, `mod.rs:1535/1602/1731` | C1, **C2** (without it `mod.rs:1367` and the mesh-collision validator's checks 6, 16, 17 are red, measured) |
| C4 | Rigid-physics | `fix(sim-core): plane–mesh contacts as MuJoCo (midpoint, hull-graph neighbours)` | `mesh_collide.rs` | R3-B test | C3 |
| C5 | Rigid-physics | `fix(sim-core): height fields collide as MuJoCo (prism order, native CCD) and are bounded as MuJoCo` | `hfield.rs` port; bounds (in K6's `recompute_derived`, or C7 below) | R3-C tests 3, 5 | C1, K6 |
| C6 | Rigid-loading | `fix(sim-mjcf): height-field data as MuJoCo (rows bottom-to-top, normalised, grid kept)` | `builder/mesh.rs`; (+ cf-geometry `HeightFieldData` spacing if Q6 = keep) | R3-C tests 1, 2, 4, 6 | C5 |
| C7 | Rigid-loading | (only if K6 does not take geom bounds) `fix(sim-mjcf): height-field bounds as MuJoCo` | `builder/build.rs:580-589` | R3-C test 3 | — |

- Per-commit greenness: **not measured**. The prototype is one copy with all changes; its suite results are §10.
- C3 changes census goldens' agreement (§9); the census gate's baseline must be taken after C3–C5 or the ratchet must expect these 2 moves.
- Note the order: the fall-through fix (bounds) is in C5/C7 — the bumpy-field bodies fall through until it lands, whatever the order of the rest.

**Dependencies on other areas:**
- A15 F1/F2 (still needed: flex–mesh-hull EPA, capsule–box). A10 MH1 (exact hull; the port's hill-climb uses hull + adjacency).
- A10 mesh processing (MH4/MH5 + Q2 here): MuJoCo's mesh frame and f32 vertices are what make mesh pairs bitwise.
- A16 NEW-FRAME (`mju_makeFrame`): renormalises the normal (needed for bitwise hfield/convex normals, §2.3) and fixes `3a13c90a`'s tangent.
- K6 (`recompute_derived`) for the hfield bounds.
- Core integrator and kinematics (hand-off): the fused `qvel` update (§7) and the `geom_xmat` ulp (§1) are the first differing quantities of the long trajectories that remain (rocking cylinder, sphere-like ellipsoid, margin cylinder).
- A15 Q6 (contact geom order): the port computes in MuJoCo's order (required: index order changes the bits in 297 of 300 random poses, §1) and negates the normal for our order.

## 13. Open questions (the fixed decisions do not settle these)

**Q1. Fused arithmetic in the port.**
- Options: (a) `mul_add` exactly where clang contracts (bit-identical to the arm64 wheel that produces the census goldens); (b) plain arithmetic.
- *Recommend (a).*
- If wrong: with (a) on an x86-64 build without the FMA target feature, each `mul_add` is a libm call. The cost is **not measured** (no x86 here); results stay identical. With (b) the `6369216c` tie resolves to another face, so that census doc differs from its golden by 1.4e-6 in the normal and 2.8e-8 in pos (measured with the unfused port), above the gate's 1e-9.

**Q2. MuJoCo's mesh frame for collision** (§8.1; A10's area).
- Options: (a) adopt it fully: vertices in the mesh's CoM/principal frame as f32, `geom_pos`/`geom_quat` absorb `mesh_pos`/`mesh_quat`, as `user_objects.cc:3749-3754`. Breaking for anyone reading mesh `geom_pos`/`geom_quat` or `TriangleMeshData` vertices. (b) Keep our frame and give the CCD object the mesh CoM as its centre.
- *Recommend (a)*, in A10's mesh-mass-properties commit, since inertia and collision then share one frame.
- If wrong: (b) is what `R3_MESHCENTER` measured — 1.3e-4 / 2.2e-4 / 1.5e-12 on the exact slab — but it cannot be bitwise (different pose arithmetic, f64 vs f32 vertices).

**Q3. qhull's graph.** Jon's decision keeps our hull order (RIGID_SPEC §2). Measured consequence here: plane–mesh and support ties (`1aab796f` stays "state"; with MuJoCo's graph it agrees until the Newton cluster). *Recommend*: list as a limitation, no port. Nothing to add beyond A10 §4.4.

**Q4. Mesh polygons and mesh multi-contact** (§4.2).
- Options: (a) port `MakePolygons` (`user_mesh.cc:2949-3005` + the `MeshPolygon` helper `:2736-2948`) and the mesh branches of `multicontact` in Rigid-physics; (b) new ledger row, keeping A15's corner contacts for box/mesh MULTICCD pairs meanwhile.
- *Recommend (a)*: it is the last MULTICCD gap, and C3 without it turns `mod.rs:1367` red.
- If wrong under (b): MULTICCD box/mesh pairs keep corner contacts at wrong positions (box on mesh slab: up to 8.8 m off over 1,000 steps).
- Polygon vertex order depends on the hull triangulation, so bitwise parity with qhull is not expected even with (a); whether contact *sets* match was not measured.

**Q5. `nativeccd="disable"`** (§8.5).
- Options: (a) refuse as a stated limitation; (b) port libccd's MPR (`mjc_penetration`'s fallback, `engine_collision_convex.c:49-53`; libccd is BSD C, so a Rust port is allowed under the no-C++ rule) plus `mjc_fixNormal` (`:1473-1625`); (c) keep the silent no-op (the parity rule forbids it).
- *Recommend (a)*: 0 corpus docs.
- If wrong: flips `collision/mod.rs:1269` `test_disable_nativeccd_no_crash` (CI) and the ignored `golden_flags.rs:325`.

**Q6. Non-square height fields** (§5.1 e).
- Options: (a) `cf_geometry::HeightFieldData` gains separate x/y spacing — a public change in a published crate, 5 consumers in sim (§11 R3-C); (b) refuse dx ≠ dy beyond 1 % as a stated limitation.
- *Recommend (a)*.
- If wrong: (b) refuses models MuJoCo loads (0 in the corpus); keeping today's resampling is a silent deviation (main: the 5×9 fixture's sphere falls through).

## 14. What this method cannot see

- **MuJoCo built anywhere but macOS arm64.** The bitwise results hold against this wheel; x86-64 MuJoCo is unfused by reading (Q1).
- **Validators** other than the 22 sim-cpu ones (mesh/, sim-ml, sim-soft, cast, integration examples) were not run.
- **Not run:** sim-gpu, sim-thermostat, sim-urdf, cf-design, cf-osim, L1 crates and Bevy apps, licensed gates, grade, clippy/fmt of a lint-clean version, the ignored golden-flag tests, x86 performance of `mul_add`, allocation cost of the polytope `Vec`s.
- **Mesh pairs** were measured on 2 slab meshes and 1 "gem", not on real meshes; the mesh polygon port is not prototyped.
- **Corpus blind spots** (as A15/A16): 370 docs one engine refuses (no hfield doc collides in the census set), runtime-generated MJCF, contacts after step 100, flex docs.
- **Two causes left unisolated**: why the cylinder's EPA witness wanders across the cap (both engines; §7), and the source of the `geom_xmat` ulp (§1).

**Repo state.** Before: `fix/rigid-sim-core-mjcf` @ `fbc0ad54`, `git status --short` empty. After: HEAD `fbc0ad54`, `git status --short` empty; no git write command run. All builds ran in the scratch copies with their own target dirs, which are deleted; trace files over 200 kB under `R3/out/` are gzipped in place (a cited `.jsonl` may be `.jsonl.gz`). The scratch C builds of §2.2 are deleted (sources `R3/cbuild/extra.c`, `R3/cbuild/stub/` kept).
