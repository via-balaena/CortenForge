> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–3) disagree, the spec chapters win.

# r3 — primitive collision: isolate and fix the census clusters (Rigid-physics)

Researcher section, 2026-10-06. Repo read-only: at start `git status --short` was empty and HEAD `fbc0ad54` (its source equals `3520544e`: the commits after it touch only `sim/docs/todo/spec_fleshouts/`). End state: §13. Everything below comes from scratch under `SCRATCH/rigid_spec/r3_collision_primitives/` (written `R3/`). MuJoCo citations are from `SCRATCH/mj350src/mujoco` (tag 3.5.0); the oracle is `SCRATCH/rigid_oracle_350/venv` (mujoco 3.5.0, macOS arm64 wheel).

**Starting point.** As instructed, every measurement starts from the A15 fix applied (`isolate_fallthrough/out/fix.diff`, EPA witness + capsule–box sign). Two bases:
- **census base** = the census researcher's `core_fix_b` (A15 + the L32/L44a/L44b/L44c prototypes), run with the census env `ISO_IMP=1 ISO_FIX=2 ISO_FLEX=2`. My re-run of it is **byte-identical** to the census's `out/fixB_all.jsonl` (`cmp`), so "base" below is the census's 850-agree state.
- **repo base** = `3520544e` + A15 only (`isolate_fallthrough/repo_fix`), for the bit-identity runs, the in-tree suites and the final code.

## 0. Summary

| # | item | cause (ours / MuJoCo) | fix (commit) | census docs that now agree |
|---|---|---|---|---|
| 1 | contact tangent frame (NEW-FRAME, 28) | ours picks the axis of smallest \|n\| (`types/contact_types.rs:326-376`); MuJoCo `mju_makeFrame` (`engine_util_spatial.c:508-534`) plus the plane–capsule axis hint (`engine_collision_primitive.c:80-85`) | **F** | 18/28 (13 by the default rule, 5 by the hint) |
| 2 | degenerate geometry (NEW-DEGEN, 18) | ours: +Z fallbacks, `−r` for a sphere center in a box, no contact for a sphere center on a cylinder axis; MuJoCo: `mjraw_SphereSphere`'s `z1×z2`, `mjraw_SphereBox`'s nearest face, `mjc_SphereCylinder`'s side/cap choice | **P**, **C** | 16/18 |
| 3 | contact position off by \|dist\| (NEW-POS, 8; + sphere–box) | ours puts the point at `surface + n·pen/2` (outside both shapes); MuJoCo at the midpoint | **P** | 8/8 |
| 4 | capsule–box selection and deep overlap | ours: own 4-phase algorithm; MuJoCo `mjraw_CapsuleBox` (`engine_collision_box.c:119-591`) | **C** (port) | 1/2 NEW-PAIRCOUNT; the other doc (f8879911) agrees with `iterations=1000 tolerance=1e-15` on both sides (§9) |
| 5a | parallel capsule–capsule count | ours: 2.6° threshold and own endpoint rules; MuJoCo `|det| < mjMINVAL` branch (`engine_collision_primitive.c:389-481`) | **C** | 1/2 NEW-PAIRCOUNT |
| 5b | ellipsoid–box: no contact inside the margin | `cf_geometry::gjk_distance` returns the wrong distance (0.709 for a 0.001 gap) | **G** | none in the corpus; fixture and sensors below |
| 5c | contact geom order | ours: per-function (often index) order; MuJoCo: lower `mjtGeom` first (`engine_collision_driver.c:1475-1480`) | **O** | 0 (the census compares contacts order-free) |
| 6 | contact inclusion (L41 14, L41list 30) | ours: list only `dist < margin`, give **every** listed contact rows; MuJoCo: list `dist ≤ margin` per function, no rows when `dist ≥ margin − gap` or `nv == 0` | **I** | L41list 29/30, L41 4/14 |
| — | found alongside: box–box | ours: own SAT + clipping (112 of 241 random overlapping poses have a different contact count, `R3/scripts/static_cmp.py`); MuJoCo `mjc_BoxBox` + the driver's filter | **B** | +2 docs (1 L41, 1 RK4MOCAP) |
| — | found alongside: capsule–cylinder dispatch | ours: analytic (position offset by \|dist\|, other end chosen when parallel); MuJoCo `mjc_Convex` | **Y** (open question Q3) | 0 (moves NEW-DEGEN 45d3e0ec from 0.05 m to EPA precision) |

**Census (1,214 docs both engines load, census base):** agree 850 → **931** with all eight commits; **0 regressions at every commit** (§2). In my six clusters (100 docs): **76 agree**. Each of the 24 remaining was followed to its first differing quantity (§9); none points at a ported function: solver termination (11 agree in state when both engines run `iterations=1000 tolerance=1e-15`), MuJoCo's broadphase culling touching pairs, sleep, FMA contraction in MuJoCo's build (which equally deep box–box corner is kept), convex-solver precision, equality rows on static bodies, accelerometer sensors.

**Ports are exact against MuJoCo's C.** Seven ported functions plus `mju_makeFrame` give **bit-identical** output to MuJoCo 3.5.0's own C source compiled without FMA contraction, on 400 random poses per pair type (200 poses × 2 margins, both XML geom orders, 15 % degenerate placements) and 3,000 frames (§1.3). Compiled **with** contraction (`-ffp-contract=on`), the same C differs from the port in contact count on 1/400 box–box and 2/400 capsule–box poses, and it reproduces the wheel where checked (box–box at census doc 0f322379's step 1; `capsule_box_7_0047`) (§1.3, Q2).

---

## 1. Method

### 1.1 Gated scratch copy, cumulative variants

`R3/core` (census base) and `R3/repo` (repo base) carry the changes behind a runtime switch `r3(letter)` read from env `R3` (letters O F I P C G Y B). Cumulative variants `""`, `O`, `OF`, `OFI`, `OFIP`, `OFIPC`, `OFIPCG`, `OFIPCGY`, `OFIPCGYB` give per-commit attribution with one binary. With `R3` empty, the binary's census output is byte-identical to the census base (`cmp out/v_none.jsonl out/base.jsonl`). The final, ungated series is `R3/final` (scratch git, base commit `908209e` = repo base): one commit per letter plus fmt/clippy and docs; `R3/out/fix.diff` and `R3/out/patches/` hold it. **The final series is bit-identical to the gated all-on variant** on every deterministic corpus doc (`fpcmp.py OFIPCGYB final`: 1,529 unchanged, 0 changed, 70 nondeterministic).

### 1.2 Census and fingerprints

- Census: the census researcher's harness and scripts (`parity_census/scripts/compare.py`, `classify.py`), run by `R3/scripts/runv.sh`; per-label tallies by `R3/scripts/summ.py`; per-doc table `R3/out/census_docs.tsv` (label, doc, source `file:line`, class before/after, the commit at which it first agrees).
- Bit-identity: the streaming fingerprint harness (`determinism_verification` `harness2.rs`, A15's `pairs` field) extended with `con_fp` (hash of every contact's geoms, dim, depth, includemargin, pos, normal, frame bits and the efc count, every step), `nexcl`, `nswap` (contacts whose geom order differs from MuJoCo's) and `nframe` (contacts whose frame differs between the old and new rules). 1,584 corpus docs + the 15 repo `.xml`, 100 steps, ×3 runs per variant (`R3/scripts/run_fp.sh`, RSS watchdog 2.5 GB). `R3/scripts/attrib.py` checks that every changed doc satisfies its commit's predicate.

### 1.3 Function-level oracle: MuJoCo's C, with and without FMA

- `R3/scripts/mj_narrow.py` calls the wheel's exported `mjc_BoxBox` etc. through `ctypes` on the wheel's own geom poses (`sizeof(mjContact)` = 576, offsets 0/8/32 checked with a C `offsetof` probe, `R3/csz/sz.c`).
- `R3/cprim/` compiles MuJoCo's own `engine_collision_primitive.c`, `engine_collision_box.c` (with the driver's box–box filter, `engine_collision_driver.c:1545-1611`, pasted verbatim), and `engine_util_*.c` from the 3.5.0 source, twice: `-ffp-contract=off` and `=on`. `R3/gjkprobe/src/bin/bbprobe.rs` runs the Rust ports on the same inputs; outputs are compared as f64 bit patterns.

| pair (200 poses × margin 0 and 0.05) | port == C, no FMA (bitwise) | C with FMA == C without | contact count, port vs C with FMA |
|---|---|---|---|
| sphere–sphere, sphere–capsule ×2 orders, sphere–cylinder ×2, sphere–box ×2, capsule–capsule, capsule–box ×2, box–box | **400/400 each** | 39–216 of 400 differ in at least one bit | equal except box–box 1/400, capsule–box 2/400 |
| `mju_makeFrame` (3,000 normals, with and without hint, hint ∥ normal) | **3000/3000** | 1336/3000 equal | — |

The comparator is not vacuous: the same comparison flags FMA-on vs FMA-off in 10–54 % of poses per pair. For the two capsule–box count mismatches, `R3/scripts/tri.sh` shows: from-source C without FMA = port (1 contact), with FMA = wheel (2 contacts) for `capsule_box_7_0047`; for `capsule_box_7_0055` the port on the wheel's poses gives the wheel's count, so the census mismatch came from our FK's input roundoff, not the port.

### 1.4 Random-pose agreement with the wheel (`R3/scripts/pair_table.py`, t0 contacts, tolerance 1e-9)

| pair | overlapping poses | base agree / count / differ | fix agree / count / differ | fix worst |
|---|---|---|---|---|
| box–box | 114 | 47 / 55 / 12 | 113 / 0 / 0, + 1 pose with no contact in MuJoCo and the fix (Q7) | 5.1e-16 |
| box–capsule, capsule–box | 107, 116 | 51/45/11, 53/41/22 | 107/0/0, 114/2/0 | 1.2e-13 |
| box–sphere, sphere–box | 123, 137 | 0/0/123, 0/0/137 | all agree | 1.8e-15 |
| capsule–capsule | 108 | 89 / 10 / 9 | 108 / 0 / 0 | 2.8e-14 |
| capsule–sphere, sphere–capsule | 125, 129 | 109/0/16, 110/0/19 | all agree | 1.6e-15 |
| cylinder–sphere, sphere–cylinder | 139, 146 | 0/8/131, 0/12/134 | all agree | 1.4e-15 |
| sphere–sphere | 191 | 176 / 0 / 15 | 191 / 0 / 0 | 2.2e-16 |
| capsule–cylinder, cylinder–capsule (Y, GJK/EPA) | 98, 109 | 0/6/92, 0/12/97 | 3/3/89, 11/8/86 | dist ≤ 8.3e-7 when shallow; see Q3 |

("count" = different number of contacts; "base" here is the census base without my commits.)

---

## 2. Census per commit (cumulative, census base; `R3/out/cls_v_*.jsonl`)

| variant | agree | NEW-FRAME (28) | NEW-DEGEN (18) | NEW-POS (8) | NEW-PAIRCOUNT (2) | L41 (14) | L41list (30) | regressions |
|---|---|---|---|---|---|---|---|---|
| base | 850 | 0 | 0 | 0 | 0 | 0 | 0 | — |
| +O | 850 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| +F | 868 | 18 | 0 | 0 | 0 | 0 | 0 | 0 |
| +I | 903 | 18 | 0 | 0 | 0 | 3 | 29 | 0 |
| +P | 926 | 18 | 14 | 8 | 0 | 3 | 29 | 0 |
| +C | 929 | 18 | 16 | 8 | 1 | 3 | 29 | 0 |
| +G | 929 | (same) | | | | | | 0 |
| +Y | 929 | (same) | | | | | | 0 |
| +B | 931 | 18 | 16 | 8 | 1 | 4 | 29 | 0 |

Outside my clusters, 5 more docs agree: NEW-LATE ×3 at I (`noslip.rs:600`, `:695`, `:806`), NEW-RK4MOCAP ×2 (`equality_constraints.rs:1232` at B, `mocap-bodies/tilt-drop main.rs:39` at P).

## 3. Bit-identity elsewhere (repo base, ×3 runs; `R3/out/fp_*.jsonl`)

| commit | unchanged | changed (trajectory / contacts only) | every change explained by | touching but unchanged |
|---|---|---|---|---|
| O | 1503 | 26 (20 / 6) | a contact whose legacy order differs from MuJoCo's (`nswap > 0`) | 0 |
| F | 1434 | 95 (84 / 11) | a contact whose old and new frames differ (`nframe > 0`) or a plane–capsule contact | 0 |
| I | 1447 | 81 (19 / 62) | an excluded contact (`nexcl > 0`), a changed contact count, or `nv = 0` (3 docs with no joints) | 0 |
| P | 1475 | 53 (46 / 7) | a sphere–{sphere, capsule, cylinder, box} contact | 0 |
| C | 1511 | 18 (16 / 2) | a capsule–capsule or capsule–box contact | 0 |
| G | 1528 | **0** | — | 0 |
| Y | 1527 | 2 (2 / 0) | a capsule–cylinder contact | 1 |
| B | 1513 | 16 (16 / 0) | a box–box contact | 0 |
| all | **1366** | 163 (106 / 57) | | |

Nondeterministic docs (ledger-L27 hash order, 66–71 per comparison) are excluded; two `model_fp`-only changes are L27 docs (`f6d9bfb4`, `d15b3550`), which no commit here can cause (model building is untouched). Every unaffected deterministic doc is bit-identical. G changes no corpus doc: no corpus doc has a GJK margin-zone contact or a non-sphere geom-distance sensor.

---

## 4. Item 1 — contact tangent frame (commit F)

**Now.** `compute_tangent_frame` (`types/contact_types.rs:326-376`) picks the axis of smallest |n| and takes t1 = normalize(n × axis), t2 = n × t1; its doc says this "matches MuJoCo's `mju_makeFrame`". `make_contact_from_geoms` (`collision/narrow.rs:447`) and `Contact::{with_solver_params, with_condim}` (`:201`, `:277`) use it; plane–capsule (`collision/plane.rs:152-186`) passes no hint. For n = +z both rules give the same bits; for n = −z they flip t1 and t2; for n = ±x and general normals they differ.

**Target.** `mju_makeFrame` (`engine_util_spatial.c:508-534`), called for every contact in `mj_setContact` (`engine_collision_driver.c:1418`): t1 = hint if `|hint|² ≥ 0.25`, else (0,1,0) when |n_y| < 0.5, else (0,0,1); Gram–Schmidt, normalize, t2 = n × t1. The only hint MuJoCo sets is the capsule axis in `mjc_PlaneCapsule` (`engine_collision_primitive.c:80-85`); every other function zeroes `frame+3`.

**Change.** `compute_tangent_frame(normal)` becomes `compute_tangent_frame_hint(normal, &zeros)`; new `pub fn compute_tangent_frame_hint(normal: &Vector3<f64>, hint: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>)` (crate-visible: `types::contact_types` is `pub(crate)`); products written in MuJoCo's operand order; plane–capsule sets `contact.frame = compute_tangent_frame_hint(&plane_normal, &axis)`. The stored normal is not renormalized (MuJoCo's `mju_makeFrame` renormalizes `frame[0..3]` in place; ours stays as the collision function wrote it — a ≤1-ulp difference, not changed to keep other bits).

**Measured.** 18/28 NEW-FRAME docs agree (census). The 10 left: box–box ×3 (fixed only in state under a tight solver, §9), solver termination ×5, broadphase ×2 (§9).

**Tests to add** (`R3/regress/tests/vs_mujoco.rs`, each against the wheel's golden): `f_sphere_sphere_x` (two spheres along x: t1 = (0,1,0); at the parent t1 = (0,0,1)), `f_plane_capsule` (horizontal capsule on a plane: t1 = (1,0,0) = axis; parent (0,1,0)). Both fail at the parent and pass at F (§10).

**Flips.** None in the suites (§10). 95 corpus docs change bits (§3).

**Downstream.** `Contact.frame` is read only inside sim-core (`constraint/jacobian.rs:246`, friction rows; `forward/acceleration.rs:451`, `:477`, `cfrc_ext`). `git grep '\.frame\['` outside `sim/L0/core/src`: no hits. `Contact::new/with_condim/with_solver_params` callers outside sim-core: `sim/L0/tests/integration/sensor_phase6.rs` only (frames not asserted).

## 5. Item 2 — degenerate geometry and Item 3 — positions (commit P)

**Now.**
- sphere–sphere `pair_convex.rs:14-58`: coincident centers → normal +Z (`:37-41`); listed only when `penetration > -margin` (`:34`).
- sphere–capsule `:193-283`: +Z fallback (`:244`).
- sphere–box `:284-374`: center inside the box → depth `r − 0` (dist −r: −0.05 in the census doc where MuJoCo gives −0.15); position `closest_world + n·pen/2` (`:348`), i.e. outside both shapes by pen/2 on the sphere side.
- cylinder–sphere `pair_cylinder.rs:18-139`: center on the axis (`:63-71`) → closest point on the curved surface, so the depth is `r_s − r_c`: **no contact** in every measured case (sphere 0.05 inside cylinder 0.06, `p_sphere_on_cylinder_axis`; census d497c31e, df6c7736, fb6bf805); position `closest_on_cyl + n·pen/2` (`:114`).

**Target (all `engine_collision_primitive.c` unless noted).** `mjraw_SphereSphere` `:245-276` (listed iff `|c|² ≤ (margin + r1 + r2)²` `:252`; coincident centers → n = z1 × z2, (1,0,0) when parallel `:263-269`; pos = p1 + n·(r1 + dist/2)); `mjraw_SphereCapsule` `:288-303`; `mjc_SphereCylinder` `:315-385` (inside: side vs cap by smaller depth; cap via plane–sphere with flipped normal; corner via a zero-radius sphere); `mjraw_SphereBox` (`engine_collision_box.c:39-92`; inside `:61-77`: nearest face, dist = −closest − r).

**Change.** New `collision/mj_primitive.rs` (pub items in a `pub(crate)` module): `pub struct RawContact { dist, pos: [f64; 3], normal: [f64; 3] }`, `raw_sphere_sphere`, `raw_sphere_capsule`, `raw_sphere_cylinder`, `raw_sphere_box`, statement-by-statement ports with MuJoCo's helpers (`mju_normalize3`, `mju_clip`, `mju_clampVec`, `mji_*`). `collide_sphere_sphere`, `collide_sphere_capsule`, `collide_sphere_box` (`pair_convex.rs`) and `collide_cylinder_sphere` (`pair_cylinder.rs`) become wrappers that put the sphere first and build `Contact`s through a new `pub fn contacts_from_raw(model, raw, geom1, geom2, margin) -> Vec<Contact>`. Signature change (crate-private): `collide_sphere_sphere` gains `mat1`, `mat2` (needed for `z1 × z2`); `narrow.rs:103` passes them.

**Measured.** NEW-POS 8/8 and NEW-DEGEN 14/18 at P, +2 at C; §1.3–1.4 for random poses. This includes sphere–box (`pair_convex.rs:348`, same `+pen/2` offset), which A15 left bit-identical only because its brief required it.

**Tests to add** (fail at the parent OFI, pass at P; values are the wheel's): `p_concentric_spheres` (parent pos (0,0,0.05), MuJoCo (0.05,0,0)); `p_sphere_in_box` (parent dist −0.1, MuJoCo −0.15); `p_sphere_on_cylinder_axis` (parent ncon 0, MuJoCo 1 at dist −0.11); `p_sphere_box_shallow` (parent pos z 0.51, MuJoCo 0.49); `p_cylinder_sphere_side` (parent pos x 0.3125, MuJoCo 0.2875).

**Flips.** sim-core lib, at P (they call the replaced functions directly): `pair_convex.rs:442` `sphere_sphere_coincident_picks_z_normal` (pins +Z; MuJoCo (1,0,0)), `:651` `sphere_box_reversed_order_geom_dispatch` (pins the box→sphere normal; MuJoCo sphere first), `pair_cylinder.rs:925` `cyl_sphere_side_penetration`, `:1030` `cyl_sphere_cap_penetration` (same, normal sign). All are CI-run unit tests; each should be rewritten to the MuJoCo value.

## 6. Item 4 — capsule–box and Item 5a — parallel capsule–capsule (commit C)

**Now.** capsule–box `pair_cylinder.rs:262-577` (A15 fixed `:305` and `:349`; selection is ours: face/edge phases, `dedup_dist_sq` `:535-546`); capsule–capsule `pair_convex.rs:69-187` (parallel when `|a1·a2| > 0.999` `:97`, i.e. within 2.6°, then its own endpoint rules `:99-149`; degenerate +Z `:168`).

**Target.** `mjraw_CapsuleBox` (`engine_collision_box.c:119-591`; its `j == 2` block `:296-393` only computes locals that nothing reads and is left out) calling `mjraw_SphereBox` for up to two points; `mjraw_CapsuleCapsule` (`engine_collision_primitive.c:389-481`: general case when `|det| ≥ mjMINVAL` `:406`, else endpoint tests that may return the same point twice).

**Change.** `raw_capsule_capsule`, `raw_capsule_box` in `mj_primitive.rs`; `collide_capsule_capsule` (adds `mat`s already present) and `collide_capsule_box` become wrappers (capsule first). Deleted as unused: `CapsuleBoxFeature`, `forward::closest_points_segments_parametric` (`forward/position.rs:481-598` + its re-export `forward/mod.rs:69`), `PARALLEL_THRESHOLD`. **A15's F2 (capsule–box sign and position, `pair_cylinder.rs:305`, `:349`) is superseded**: C deletes those lines. A15's 7 regression tests pass with all my commits (`R3/regress/tests/a15.rs`, measured at the head; capsule rest height 0.0996328181575398 to 1e-9).

**Measured.** NEW-DEGEN +2 (crossing capsules), NEW-PAIRCOUNT 2fac9a08 (`collision_primitives.rs:473`). f8879911 (`collision_primitives.rs:1363`, T-cross; A15 measured it 0.124 m off) now matches MuJoCo's single contact at t0; over 100 census steps it stays within 1.6e-9 in qpos and 1.9e-8 in qvel, the first difference being a solver-termination jump at step 18 (§9).

**Tests to add** (fail at OFIP, pass at C): `c_collinear_capsules` (end-to-end capsules: parent 1 contact, MuJoCo 2 identical), `c_crossing_capsules` (axes intersect: parent n = +Z, MuJoCo (0,0,−1) from z1 × z2), `c_capsule_box_edge` (parent 2 contacts, MuJoCo 1).

**Flips.** Integration (CI): `collision_primitives.rs:504` `capsule_capsule_endpoint_endpoint` asserts 1 contact; MuJoCo returns 2 (measured with the wheel on the test's XML).

## 7. Item 5b — ellipsoid–box margin (commit G) and 5c — geom order (commit O)

### 7.1 G

**Now.** `cf_geometry::gjk_distance` (`design/cf-geometry/src/query/gjk.rs:123-207`) breaks when the new support point does not pass the origin and the simplex has ≥ 3 points (`:166`), stops on a relative-improvement test (`:178-183`), and `closest_point_on_simplex_to_origin` treats a 4-point simplex as its first point (`:424`); which of these produces the wrong distance was not isolated — the rewrite below replaces all three. Measured (`R3/gjkprobe`): box (0.5, 0.5, 0.1) vs ellipsoid (0.05, 0.04, 0.03) at gaps 0.001 / 0.0005 / 0.005 / 0.009 / 0.02 returns **0.7087 / 0.7087 / 0.7088 / 0.7088 / 0.0342** (the distance to a box corner). So the margin-zone path (`collision/narrow.rs:234-267`) sees no contact.

**Target.** MuJoCo's `gjk` termination (`engine_collision_gjk.c:171-277`): stop when `x·(x − s) < ½·tol²` (`:185`, `:202-208`) or x stops changing. MuJoCo reaches the margin zone through EPA on margin-inflated shapes (`engine_collision_convex.c:688-690`, `:807`), not a separate distance query.

**Change.** `gjk_distance` keeps its signature; its loop becomes: support along −v, MuJoCo's Frank–Wolfe test, push, closest point of the 1–4-point simplex (new private `closest_point_on_simplex_full`: `None` when the origin is inside the tetrahedron), reduce, stop when v is unchanged; witnesses from the last simplex's barycentrics. Semantics change of a published function (cortenforge-geometry): it now returns the distance it documents.

**Measured.** Distances above → 0.001, 0.0005, 0.005, 0.009, 0.02 within 2e-11. A15's margin fixture (ellipsoid 0.05/0.04/0.03 over a box with margin 0.01, census excitation, 1,000 steps): final z 0.031083 → **0.040120** (MuJoCo 0.040119), max |Δz| 1.1e-2 → **2.1e-5**; cylinder fixture 7.5e-3 → 1.3e-4; capsule unchanged (analytic, 4.5e-9). Geom distance/normal/fromto sensors (`R3/xml/g_distance_sensors.xml`, box–ellipsoid, box–cylinder, ellipsoid–cylinder): max |Δ| vs MuJoCo **1.98e-2 → 8.7e-16**. The census sees none of this (no corpus doc exercises it, §3).

**Tests to add.** `gjk_distance_box_ellipsoid_near_touching` (cf-geometry, `R3/regress/tests/gjk.rs`: gap within 1e-9 for four gaps; base returns 0.7087); `g_ellipsoid_box_margin` (contact at t0 with MuJoCo's dist 0.0010000628 within 1e-4; base ncon 0). Both fail at the parent, pass at G.

**Flips.** None (cf-geometry 394/394, sim-core lib, integration unchanged at G).

**Downstream.** `cf_geometry::gjk_distance` callers: `cf-geometry/src/query/closest_point.rs:250` (point-to-hull), `sim_core::gjk_epa::gjk_distance` (`gjk_epa.rs:645`) → `collision/narrow.rs:238` (margin zone), `collision/mesh_collide.rs:83` (mesh margin zone, not measured), `sensor/geom_distance.rs:105` (distance sensors, measured above).

### 7.2 O

**Now.** No ordering rule: sphere–capsule, sphere–box, cylinder–sphere put the lower **index** first (`pair_convex.rs:250-262`, `:353-369`; `pair_cylinder.rs:116-121`); plane functions put the plane first; GJK and box–box keep the call order. `mj_collision` sorts by `(geom1, geom2)` (`collision/mod.rs:665`).

**Target.** `mj_collideGeoms` swaps so that `geom_type[g1] ≤ geom_type[g2]` (`engine_collision_driver.c:1475-1480`), keeping the tested order for equal types; the normal points g1 → g2; `contactcompare` sorts by the unswapped pair (`:226-256`).

**Change.** `pub(crate) const fn GeomType::mjtgeom(self) -> u8` (MuJoCo's enum values; ours declares variants in another order, `types/enums.rs:100-120`); `fn order_like_mujoco(model, contacts, g1, g2)` in `collision/mod.rs`, applied to every rigid contact from `collide_geoms` and `collide_hfield_multi` for broadphase and `<pair>` contacts: swap geoms, negate normal, rebuild frame; sort key `(min(g1,g2), max(g1,g2))`; `Contact.geom1` doc states the rule. The ported functions (P, C, B) produce MuJoCo's order natively.

**Measured.** No census doc changes class (the census matches contacts by unordered pair and flips normals: `con_key` sorts the pair, `cmp_contacts` negates the normal, `parity_census/scripts/compare.py:218-256`); 26 corpus docs change bits (§3).

**Test to add.** `o_sphere_box_order` (sphere geom 1 above box geom 0: `(geom1, geom2) == (1, 0)`, normal (−1,0,0)); fails at base, passes at O.

**Flips.** Integration (CI), all assert normals in index order: `collision_primitives.rs:340` `sphere_capsule_endpoint_contact`, `:558` `sphere_box_face_contact`, `:923` `cylinder_sphere_side_contact`, `:968` `cylinder_sphere_cap_contact`. The MuJoCo goldens for their XML (`R3/xml/flips/`, wheel) give exactly the new geom order and normal.

**Downstream** (`git grep '\.geom1\b\|\.geom2\b'` outside the collision module, Contact readers only): sim-core `constraint/jacobian.rs`, `island/mod.rs`, `forward/actuation.rs` (adhesion: symmetric in the pair, `J(b2) − J(b1)` along the normal), `constraint/contact_assembly.rs`, `types/data.rs:1036-1062` (`contacts_*` helpers, order-agnostic); sim-gpu `pipeline/tests.rs:1737` (bounds only), `contact_conformance_tests.rs:191` and `test_fixtures/conformance.rs:588` (copy ids into injected contacts); `sim/L0/tests/mujoco_conformance/layer_b.rs:170` (compares pairs with MuJoCo's reference: 83/83 still pass); printed only: `examples/fundamentals/sim-cpu/contact-filtering/{bitmask,exclude-pairs,ghost-layers}`, `examples/sdf-physics/cpu/{07-pair,08-stack}`, `integration/{collision_test_utils.rs:647, contact_debug_diagnostic.rs:56}`; order-free: `contact-filtering/stress-test main.rs:507`, `sleep-wake/stress-test main.rs:700`, `integration/adhesion.rs:690`, `design/cf-design/src/mechanism/model_builder.rs:2735` (|n_z| only).

## 8. Item 6 — contact inclusion (commit I)

**Now.**
- Listing: strict `dist < margin` in plane–sphere (`plane.rs:56`), plane–capsule (`:170`), sphere–sphere (`pair_convex.rs:34`, on `r1+r2−|c|`); box–plane emits **all** bottom-face corners when the deepest is within the margin (`plane.rs:79-151`, rule at `:130`) — the census's "box–plane corners at dist +0.35".
- Rows: every listed contact gets rows (`constraint/assembly.rs:233-243` counts them, `contact_assembly.rs:37` assembles them), even though `Contact.includemargin`'s doc (`types/contact_types.rs:56-60`) says a contact at `dist ≥ includemargin` is excluded. With `nv == 0` a static `<pair>` still gets rows (census doc 2fcd4f39: 10 rows, MuJoCo 0).
- Islands: every contact is an island edge (`island/mod.rs:61`).

**Target (MuJoCo's rule, found in the source).**
1. Each collision function lists `dist ≤ margin` by its own test — inclusive for the primitives: `cdist > margin + r → 0` (`engine_collision_primitive.c:39`), `|c|² > (margin+r1+r2)² → 0` (`:252`), plane–box per corner `dist + ldist > margin || ldist > 0 → skip`, at most 4 (`:220`); the convex path lists `dist < margin` (inflated-shape EPA, `engine_collision_convex.c:806-807`).
2. `mj_setContact` sets `exclude = (dist ≥ includemargin)`, `includemargin = margin − gap` (`engine_collision_driver.c:1414-1415`, called with `margin-gap` at `:1646`); `d->ncon` counts the contact.
3. `mj_instantiateContact` makes no rows for an excluded contact (`engine_core_constraint.c:1016-1019`) and none at all when `nv == 0` (`:996`); in sparse mode (`nv ≥ 60` with `mjJAC_AUTO`, `engine_core_util.c:32-39`) a contact whose bodies have no dofs gets `exclude = 3` (`:1029-1032`).
4. Islands come from rows (`engine_island.c:188-240`); wake-up (`engine_sleep.c:279-320`) and adhesion (`engine_core_smooth.c:1645-1710`, counts `exclude` 0 and 1) see every listed contact.

**Change.** `pub fn Contact::is_excluded(&self) -> bool { -self.depth >= self.includemargin }` (additive public API); `assemble_unified_constraints` and `assemble_contact_rows` skip excluded contacts and all contacts when `nv == 0` (excluded contacts keep their index, so `efc_id → contacts[ci]` consumers are unaffected); `island/mod.rs` edge extraction skips excluded contacts; plane–sphere and plane–capsule list on `!(cdist > margin + r)` written as an early return (NaN still lists, as in C); box–plane ported (`mjc_PlaneBox`, `engine_collision_primitive.c:196-239`); docs of `Data::contacts`, `Data::ncon` and the three `contacts_*` helpers say excluded contacts are included. Not implemented: rule 3's sparse-mode `exclude = 3` (ours has no sparse/dense switch; it matters only for a contact between two dof-less bodies in a model with `nv ≥ 60`).

**Measured.** L41list 29/30, L41 3/14 at I (+1 at B: `solvers/pgs main.rs:72`), NEW-LATE +3; L41's remaining 10 are traced in §9.

**Tests to add** (fail at OF, pass at I): `i_sphere_touching_plane` (sphere resting at dist 0, margin 0: MuJoCo ncon 1, nefc 0; parent ncon 0); `i_box_plane_gap` (plane margin 0.1 gap 0.08, box tilted 10°: MuJoCo 4 contacts, 2 excluded, nefc 8; parent nefc 16); `i_static_pair` (`<pair>` of two static spheres, nv 0: MuJoCo ncon 1, nefc 0; parent nefc 4).

**Flips.** Integration (CI): `cg_solver.rs:307` `test_cg_single_contact_direct` — its sphere starts at dist 0 (`cg_solver.rs:23-26`), the first listed contact is excluded, CG has no rows and does 0 iterations. MuJoCo makes no rows for it either; the test should wait for `nefc > 0` (or start the sphere 1 mm lower).

**Downstream.** `data.ncon`/`data.contacts` now include excluded contacts: `contacts_involving_geom`/`between_geoms`/`between_bodies` (used by the three contact-filtering examples: validator `contact-filtering/stress-test` passes), `examples/fundamentals/sim-cpu/mesh-collision/stress-test main.rs:598` (`ncon == 0` for separated meshes: mesh paths unchanged, passes), `keyframes/stress-test main.rs:302` (after reset), `solvers/stress-test main.rs:111`, `integration/{sim-informed-design,full-pipeline}` (filter `depth > 1e-8`, unaffected), `cf-design-tests/tests/l4_physics.rs:172` (count in a message).

## 9. What the remaining 24 cluster docs need (first differing quantity, measured)

| docs (label, source) | first difference after all commits | cause, and how it was isolated |
|---|---|---|
| 0f322379, b26a81b9, c8902cf8 (NEW-FRAME; `collision_primitives.rs:761`, `:648`, `collision_edge_cases.rs:636`) | step 1, one of 5 box–box contacts at another (equally deep) corner | MuJoCo's FMA contraction: on the wheel's step-1 poses the port equals MuJoCo's C without FMA bit for bit, and the C **with** FMA equals the wheel (`tri.sh`; the 3×3 product `mju_mulMatTMat3` gives `rot[0]` = 1.0000000000000002 = `fma(m6,m6,fma(m0,m0,m3·m3))`, flipping a 1e-20 residue's sign and the corner choice). With `iterations=1000 tolerance=1e-15` on both sides the trajectories agree to 3e-15 |
| 53df39ca, a1d4ffc7, d4ef2aa3, eb97180b (NEW-FRAME), f8879911 (NEW-PAIRCOUNT) | step 14–18, contacts and rows equal, `qfrc_constraint` 1.6e-4 | solver termination: with the tight solver on both sides they agree to ≤ 7.9e-11 over 100 steps (`R3/out/tight_cmp_all.jsonl`); the NEW-NEWTON cluster's open question |
| f935e97a (NEW-FRAME), aeb76329, faa6b270 (L41; solvers/newton, solvers/cg) | t0 `qfrc_constraint`, contacts equal | solver; tight solver leaves 1.2e-9 / 6.6e-8 / 4.3e-6. Not isolated further |
| 05b1f2d2, 31833131, 140b7da3 (L41), 1efe8000, eaace9f8 (NEW-FRAME, API only after I) | MuJoCo lists no contact for spheres touching or overlapping by ≤ 1.5e-10 | culled before MuJoCo's narrowphase: in 31833131 at step 6 (MuJoCo's own state, overlap 5.6e-17) MuJoCo lists nothing with geom margin 0 or 1e-9 and lists the contact with 1e-6 (`R3/scripts/bp_probe.py`), while its sphere–sphere test is inclusive. Its broadphase casts bounds to float (`engine_collision_driver.c:1036-1038`; float32 spacing at 0.5 is 6e-8); which comparison drops the pair was not isolated. Ours (f64 SAP, inclusive) lists them. Open question Q1 |
| a6e04a3c, d72ea476, a2f14cb1 (L41; sleep-wake examples) | t0 `qfrc_bias` / step 28 | the SLEEP cluster's signature (init-asleep forward); not isolated here |
| 46a5290e, 5dfbbb75 (L41) | steps 25 / 47, ≤ 4e-5 | 5dfbbb75 agrees in state with a tight solver (6e-10); 46a5290e drops to 3.3e-8. Not isolated |
| 45d3e0ec (NEW-DEGEN; parallel capsule–cylinder) | dist 2.6e-8, normal 4.9e-5 (with Y) | convex solver precision: MuJoCo uses `mjc_Convex`; our EPA is not MuJoCo's (Q3) |
| d985f32c (NEW-DEGEN) | `nefc`: equality rows between bodies without dofs | the NEW-EQSTATIC cluster; its contact now agrees |
| af3775ea (L41list) | step 1/100 `sensordata` | accelerometer (NEW-ACCEL cluster); its contacts agree |

## 10. Tests

### 10.1 Tests to add (scratch `R3/regress`, wheel goldens in `R3/out/golden.jsonl`; fail at the parent commit, pass at their commit — measured on the cumulative variants)

| commit | tests |
|---|---|
| O | `o_sphere_box_order` |
| F | `f_sphere_sphere_x`, `f_plane_capsule` |
| I | `i_sphere_touching_plane`, `i_box_plane_gap`, `i_static_pair` |
| P | `p_concentric_spheres`, `p_sphere_in_box`, `p_sphere_on_cylinder_axis`, `p_sphere_box_shallow`, `p_cylinder_sphere_side` |
| C | `c_collinear_capsules`, `c_crossing_capsules`, `c_capsule_box_edge` |
| B | `b_box_box_random` (one random pose: parent 6 contacts, MuJoCo 4 — all 4 to 1e-12) |
| G | `gjk_distance_box_ellipsoid_near_touching`, `g_ellipsoid_box_margin` |
| Y | `y_capsule_cylinder_parallel` (tolerance 5e-3: our EPA's normal is 2e-3 off here) |

Tolerance 1e-12 for every ported pair. In the repo they belong in `sim/L0/tests/integration/collision_primitives.rs` (contact-level), `cg_solver`-style rows checks in a new `contact_inclusion.rs`, and `design/cf-geometry/src/query/gjk.rs` tests (G). A function-level bitwise test against MuJoCo's C needs a C build in CI — not proposed; the golden fixtures above are the executable referent.

### 10.2 Tests and examples that flip

Suites run on the final series (`R3/final`, own target): cortenforge-geometry 394/394 · sim-core lib **715/720** · sim-mjcf 387/387 · sim-conformance-tests integration **1331/1338** · mujoco_conformance 83/83 · sim-urdf 49/49 · sim-gpu lib 84/84 (fix only; not compared with base). Base (repo + A15): all pass.

| commit | test (all CI-run) | why |
|---|---|---|
| O | `integration/collision_primitives.rs:340`, `:558`, `:923`, `:968` | normal asserted in index order; MuJoCo puts the sphere first |
| I | `integration/cg_solver.rs:307` | the first listed contact is excluded (dist 0) |
| P | `core/src/collision/pair_convex.rs:442`, `:651`; `pair_cylinder.rs:925`, `:1030` | +Z degenerate normal; index-order normals |
| C | `integration/collision_primitives.rs:504` | MuJoCo returns 2 contacts for end-to-end capsules |
| Y | `integration/collision_primitives.rs:1124` (depth 0.45 exact; MuJoCo 0.4499899, ours 0.4499933); `pair_cylinder.rs:1086` (rewritten to `collide_geoms`: EPA normal off by 8.4e-4 at tolerance 1e-6) | capsule–cylinder now through GJK/EPA |

Validators: the 26 sim-cpu and sim-ml validators (`R3/out/val_pkgs.txt`) exit 0 at base and at the head. Printed numbers change in 5: `contact-tuning` (low-µ slide 7.110 → 7.112 m), `derivatives` (7.53e-1 → 7.69e-1), `equality-constraints` (pivot errors), `mocap-bodies` (weld separation 0.050 → 0.075, threshold 0.15), `solvers` (**PGS avg iterations 47.4 → 49.1 against a threshold of 50**). Corpus docs that change, by CI reach (`R3/out/changed_docs_sources.tsv`, reach from `determinism_verification/fliptable.tsv`): 140 in CI tests (all pass, except the flips above), 10 in CI validators (all pass), 1 repo golden model (`sim/L0/tests/assets/golden/conformance/models/weld_model.xml`; mujoco_conformance 83/83), and never run: 10 Bevy apps (`contact-tuning/{friction-slide,pair-override,condim-compare}`, `solvers/{newton,pgs,cg}`, `sleep-wake/wake-on-contact`, `equality-constraints/weld-body-to-body`, `joint-limits/hinge-limits`, `mocap-bodies/tilt-drop`), `sim/L1/bevy/examples/collision_shapes.rs:31`, and `contact-filtering/ghost-layers` (listed as "bin, crate TR").

---

## 11. Commit list — all in **Rigid-physics**

Order as measured; each compiles with zero warnings (`cargo check -p cortenforge-sim-core -p cortenforge-geometry --all-targets`, per commit). Clippy (`--all-targets --all-features -- -D warnings`), rustfmt and `cargo doc` were run on the head only; the fmt/lint fixes sit in one extra commit in scratch and belong folded into their commits.

| # | commit | contents | adds tests | flips |
|---|---|---|---|---|
| R1 | `fix(sim-core): contacts carry MuJoCo's geom order (lower mjtGeom first)` | O | 1 | 4 integration |
| R2 | `fix(sim-core): contact tangent frame as MuJoCo's mju_makeFrame, with the plane–capsule axis hint` | F | 2 | — |
| R3 | `fix(sim-core): contacts listed at dist ≤ margin get constraint rows only below margin − gap` | I (+ box–plane port, `Contact::is_excluded`, islands, docs) | 3 | 1 integration |
| R4 | `fix(sim-core): sphere–sphere, –capsule, –cylinder, –box ported from MuJoCo` | P (+ `mj_primitive.rs`) | 5 | 4 sim-core unit |
| R5 | `fix(sim-core): capsule–capsule and capsule–box ported from MuJoCo` | C (supersedes A15's F2) | 3 | 1 integration |
| R6 | `fix(sim-core): box–box ported from MuJoCo with the driver's box–box filter` | B | 1 | — |
| R7 | `fix(cf-geometry): gjk_distance stops on MuJoCo's Frank–Wolfe gap` | G | 2 | — |
| R8 | `fix(sim-core): capsule–cylinder through the convex solver, as MuJoCo` | Y (Q3) | 1 | 1 integration, 1 unit |

Size: 16 files, +1,869 / −1,570 (`R3/out/fix.diff`; `mj_primitive.rs` is 1,451 lines). Breaking: contact geom order and normal sign (R1), frame values (R2), `ncon` semantics (R3) — no signature change outside crate-private items; additive `Contact::is_excluded`.

**Dependencies.** A15 F1 (EPA witness) before R8 (Y uses EPA); A15 F2 dropped or before R5. The census measurements also had L32, L44a/b/c applied (census base); my commits share files with those prototypes (`constraint/assembly.rs`: L44a/ISO_FLEX edit equality/flex rows, R3 the contact row count; `forward/mod.rs`) but not functions. With the convex-collision section (`rigid_spec/r3_collision_convex`, in progress at the time): if it ports MuJoCo's native CCD, R7 may be superseded for the margin zone and sensors, and R8 should route to that solver; with the sleep section: R3 changes island edges (excluded contacts no longer join islands) — the sleep census docs were not re-checked against a sleep fix. The census gate: these commits raise its agree count by 81 with no regression; it must be regenerated after them.

## 12. Open questions (not settled by the fixed decisions)

1. **MuJoCo's broadphase culls touching and barely overlapping pairs** (5 census docs; the stage that drops them was not isolated, §9). Options: (a) port `mj_broadphase` + `mj_SAP` (covariance frame, body AAMMs, float32 values, stable sort; `engine_collision_driver.c:1021-1300`); (b) keep ours and list a **lenient** deviation — MuJoCo misses a penetrating contact (1.5e-10 overlap in 05b1f2d2 at step 2; 5.6e-17 in 31833131, which a 1e-6 margin brings back); the correctness test: two spheres overlapping by 1e-10 get a contact. **Recommend (b).** If wrong: those 5 docs keep differing — 1efe8000 and eaace9f8 in the contact list only (state within 2.4e-8), 31833131 and 05b1f2d2 by a contact impulse one step apart (max qvel difference 3.2e-2 and 1.2e-2), 140b7da3 by rows for a 2.8e-17 overlap at t0 — and users comparing `ncon` with MuJoCo see contacts MuJoCo culls (1.5e-10 and 5.6e-17 overlaps measured).
2. **FMA contraction in MuJoCo's build.** The wheel's `mju_mulMatTMat3` result equals the FMA-contracted formula, and MuJoCo's C compiled with contraction reproduces the wheel where checked (§1.3, §9); exactly tied branch decisions (aligned boxes, a capsule along a box face) then go another way. Options: (a) accept and list (the census ratchet keeps 0f322379, b26a81b9, c8902cf8 differing at default solver tolerance); (b) emulate clang's contraction with `f64::mul_add` in the ports. **Recommend (a)**: the result depends on how MuJoCo was compiled — its own C without contraction computes exactly what ours computes (measured, §1.3); whether MuJoCo's x86-64 wheels contract was not checked — and `mul_add` without hardware FMA is a libm call. If wrong: 3 census docs and ~0.5 % of random box–box/capsule–box poses differ in which equally deep contact is chosen.
3. **Capsule–cylinder routing (R8).** Options: (a) R8 now (MuJoCo's dispatch; shallow contacts within 8.3e-7 in dist, parallel ones 4.9e-5–2e-3 in normal; MuJoCo's convex solver reports no contact for 11 of 207 overlapping random poses, all deep (dist ≤ −0.21), where ours reports one); (b) keep the analytic function, fixing only its position offset (`pair_cylinder.rs:224`, `+pen/2` → `−pen/2`) — exact depths, but parallel cases pick the other end; (c) land R8 together with a port of MuJoCo's native CCD. **Recommend (c)**, (a) if the CCD port is not in Rigid. If wrong: (b) keeps 45d3e0ec 0.8 m off in contact position.
4. **`Data::contacts_*` helpers** now count excluded contacts. Options: count all (as `ncon`, MuJoCo's meaning; docs changed in scratch) or filter `!is_excluded()` to keep the old word "active". **Recommend count all**: one meaning for every contact list. If wrong: code using them to detect "touching" sees dist-0 contacts (validators pass either way).
5. **Stored normal renormalization.** MuJoCo renormalizes the normal in `mju_makeFrame`; ours does not (≤ 1 ulp). How many docs that would change, and whether any census doc would move, was not measured. **Recommend leave** and list it; if wrong, contact normals differ from MuJoCo's by ≤ 1 ulp.
6. **Contact order** (sort key `(min, max)` geom pair) still differs from MuJoCo's body-pair-signature order for multi-geom bodies and merged `<pair>`s (`engine_collision_driver.c:298-326`). The contact order sets the efc row order; its effect on results was not measured. **Recommend** a ledger row, not Rigid; if wrong, solvers that sweep rows in order see the rows of such models in another order.
7. **MuJoCo's box–box filter can remove every contact of a deep overlap.** In `box_box_7_0105` (two equally oriented boxes overlapping by 0.347) the wheel's `mjc_BoxBox` returns 4 contacts and the driver's filter (`engine_collision_driver.c:1549-1611`: drop a contact outside one 1.01-inflated box and not inside the other) removes all 4, so MuJoCo — and the port — report none; ours before R6 reported 4 (measured with the wheel's exported `mjc_BoxBox` and the from-source C with and without the filter). 1 of 114 overlapping random box–box poses. Options: (a) parity, list as a known MuJoCo behaviour; (b) a lenient deviation keeping the raw contacts when the filter would empty the list, with a test that the pose gets a 0.347-deep contact. **Recommend (a)** for Rigid, with a ledger row: (b) invents a rule MuJoCo does not have, and its right contact set is not defined by any source. If wrong: such poses keep passing through each other, as in MuJoCo.

## 13. What this method cannot see

- **Flex**: no flex doc loads in both engines; flex contact generation still filters at `depth > −includemargin` (`flex_collide.rs:118,149`, `flex_self.rs:117,201`) where MuJoCo lists and excludes — not changed, not measured.
- **mesh, hfield, sdf, GJK paths**: their listing comparisons (`<` vs `≤`) were not examined; their contacts get the new order, frame and exclusion only.
- **Param mixing order**: contacts swapped by `order_like_mujoco` keep `contact_param` computed in the legacy order; with unequal `solmix` that differs from MuJoCo by rounding (not measured).
- **Census limits** (as A16 §5): 1,214 docs, 100 steps, one excitation; runtime MJCF (157 templates, cf-design, sim-urdf) not captured; drift below 1e-9; one cause can hide a second.
- **Not run**: sim-gpu against base (only the head: 84/84), cf-design(-tests), sim-thermostat, sim-rl/ml-chassis, L1 crates, Bevy examples, licensed gates, `cargo xtask grade`, MULTICCD, noslip beyond the corpus, elliptic cones beyond the corpus, sleep with my changes outside the corpus, x86-64.
- **Per-commit greenness** of the ungated series: compile per commit yes; suites only at the head (flip attribution per commit comes from the gated variants); clippy/fmt only at the head.
- **Performance**: not measured (the box–box and capsule–box ports replace SAT/clipping code; no timing taken).

**Repo state.** Before: `git status --short` empty, HEAD `fbc0ad54`. After: `git status --short` empty, but HEAD is `b37ecb15` ("docs: the Rigid spec becomes a tracked book", 23:50), a commit I did not make — I ran no git write command in the repo. `git diff --name-only fbc0ad54 b37ecb15` lists only `docs/studies/a_double_dose_of_detail/**` and `sim/docs/**`, so the source is still `3520544e`'s. All builds used scratch copies and scratch target dirs, deleted at the end (`R3/*/target`, `R3/target_*`); census `.jsonl` outputs over 5 MB are gzipped in place.
