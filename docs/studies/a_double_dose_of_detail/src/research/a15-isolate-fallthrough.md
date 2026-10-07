> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# ledger-L42: a capsule, cylinder or ellipsoid falls through a box

Researcher section, 2026-10-06. The repo source used is `3520544e`: the copies were made at `139f7b25`, whose source equals it (see the repo-state note at the end). Every number below comes from a script in `SCRATCH/rigid_spec/isolate_fallthrough/` (written `IF/`): scripts in `IF/scripts/`, outputs in `IF/out/`, fixtures in `IF/xml/`. The MuJoCo oracle is `SCRATCH/rigid_oracle_350/venv` (mujoco 3.5.0). MuJoCo citations are from `SCRATCH/mj350src/mujoco` (tag 3.5.0).

Probes:
- `IF/probe` and `IF/probe_fix`: a per-step contact trace (`traj`), a static batch narrowphase (`collide_batch`) and a model dump (`model`), built against `IF/repo_main` and `IF/repo_fix`.
- `IF/scripts/mjtraj.py`: the same three commands in MuJoCo, with the same JSON.

## 0. Summary

The cause is two separate defects. Both produce a contact at the right step with the right distance, and that contact cannot hold the body:

| pair | first difference (step 8 of the hand-off fixtures, the first contact in both engines) | cause |
|---|---|---|
| box–cylinder, box–ellipsoid (and every other GJK/EPA pair) | contact **position** is the box's far bottom corner `(±0.5, ±0.5, −0.2)`. MuJoCo's is at the touching point, `(0.017, 0.017, −4.9e-5)` | `design/cf-geometry/src/query/epa.rs:74-75`: the witness point is A's support along **−normal**, but the normal points from A to B. That gives A's point farthest *from* B |
| box–capsule | contact **normal reversed** relative to geom order. The position is also off by the depth | `sim/L0/core/src/collision/pair_cylinder.rs:305`: `normal_sign` is inverted in both geom orders. The position bug is at `:349` |

- A 3-file fix (`IF/out/fix.diff`, +65/−23) gives every box-slab and convex-mesh-slab case swept MuJoCo's outcome (§4.2–4.4), within the tolerances stated there. A bumpy height field still lets bodies through, with and without the fix (§4.4).
- It leaves sphere–box, box–box and every plane contact bit-identical (§4.5).
- No in-tree test flips. None of them ever checked a contact normal's sign or a GJK contact position (§5).
- "20 cm" is not a threshold. On main, at every swept value of slab thickness (1 cm to 1 m), slab width (0.2 m to 4 m) and primitive size (1 cm to 20 cm), most capsule, cylinder and ellipsoid cases sink below the box top: 24–30 of 32 per thickness, 39–65 of 48–72 per width, 35–65 of 48–72 per size (§3.3).

---

## 1. The fixture is the subject

`IF/scripts/cmp_model.py` compares `ftprobe model` with `mjtraj.py model` on the hand-off's three files (`mesh_inertia_hull/cases/flat/on_slab10_{capsule,cylinder,ellipsoid}.xml`, copied to `IF/xml/`). The comparison is in `IF/out/model_cmp.txt`.
- **Identical (difference 0):**
  - geom type, size, pos, margin (0), gap (0);
  - solref `[0.02, 1]`, solimp `[0.9, 0.95, 0.001, 0.5, 2]`, condim 3, friction `(1, 0.005, 0.0001)`;
  - body mass, timestep 0.002, gravity;
  - Euler, pyramidal cone, Newton, 100 iterations, tolerance 1e-8, ccd 35 / 1e-6, no flags.
- **Body inertia** differs by ≤ 4.3e-19.
- **Trajectory before contact:** for steps 1–7, before any contact, `qpos` is **bit-identical** in both engines (max |Δ| = 0).

The divergence does not come from the fixture.

## 2. The first difference

The per-step traces are `IF/out/{main,mj}_slab10_{capsule,cylinder,ellipsoid}.jsonl`, rendered side by side with `IF/scripts/side.py`. Both engines create exactly one contact at **step 8**. The same box (`g` 0 in ours) and body are involved; MuJoCo orders geoms by type (`engine_collision_driver.c:1475-1480`), so its `geom1` is the primitive.

| | dist (ours / MuJoCo) | normal, ours (g1 = box) / MuJoCo (g1 = primitive) | pos, ours / MuJoCo | Fz on the body after the step |
|---|---|---|---|---|
| capsule | −9.872e-5 / −9.872e-5 (Δ 1.4e-17) | (0, 0, **−1**) / (0, 0, −1): ours points from geom2 to geom1 | (0, 0, **+4.94e-5**) / (0, 0, −4.94e-5) | **0** / 28.91 |
| cylinder | −9.872e-5 / −9.872e-5 (Δ 1e-20) | (0, 0, +1) / (0, 0, −1): both point geom1→geom2 | **(0.5, 0.5, −0.2)** / (0.017, 0.017, −4.9e-5) | **0.099** / 15.08 |
| ellipsoid | −9.8719e-5 / −9.8717e-5 (Δ 2.6e-9) | (−5.9e-10, −5.9e-10, 1) / (3e-7, −3e-7, −1) | **(0.5, 0.5, −0.2)** / (8.9e-6, 8.9e-6, −4.9e-5) | **0.045** / 11.56 |

- **The contact is generated, at the right step and distance. It is not lost later.**
  - What is wrong is where it is (cylinder, ellipsoid) or which way it pushes (capsule).
  - From step 8 on, ours gives the body ~1 % of MuJoCo's support force, and it sinks.
- **Capsule:** it ends embedded at z = −0.0718, with 2 contacts of depth exactly 0.05 = r (`IF/out/main_slab10_capsule.jsonl`, step 1000).
- **Cylinder and ellipsoid:** they leave contact and fall to z ≈ −19.4.
- The hand-off's "a contact appears at step 20" was not reproduced: step 8 in both engines, for all three files.

## 3. Cause

### 3.1 Cylinder and ellipsoid: the EPA witness point

**MuJoCo.**
- Dispatch: box–cylinder and box–ellipsoid go to `mjc_Convex` (`engine_collision_driver.c:47-48`, the `mjCOLLISIONFUNC` table).
- `mjc_CCDIteration` (`engine_collision_convex.c:787-817`) calls `mjc_ccd`. Its EPA recovers the witness points with `epaWitness` (`engine_collision_gjk.c:1273-1287`), which takes the affine coordinates of the origin's projection on the closest polytope face and applies them to the face vertices' support points on each geom.
- The contact is then `pos = ½(x1 + x2)`, normal `x1 − x2` normalised (`engine_collision_convex.c:807-812`).

**Ours.**
- Dispatch: box–cylinder and box–ellipsoid fall to the GJK/EPA slow path (`sim/L0/core/src/collision/narrow.rs:171-236`).
- `gjk_epa_contact` (`sim/L0/core/src/gjk_epa.rs:573-591`) returns `point: pen.point_a` (`:587`), and `narrow.rs:226` uses it as the contact position.
- `cf_geometry::epa_penetration` sets `point_a = a.support(&(-epa_result.normal))` (`design/cf-geometry/src/query/epa.rs:74`) and `point_b = b.support(&normal)` (`:75`).
- That is A's deepest point only if the normal points from B to A, which is what the doc at `epa.rs:25` (and `gjk_epa.rs:64`) says.
- **The normal points from A to B.** Measured:
  - with A = the box below and B = the body above, the normal is +z (§2 table, ours);
  - it is used unchanged as geom1→geom2 (`narrow.rs:227`);
  - in the fix, `point_a − point_b = depth · normal` holds to ≤ 3.5e-12 over 1,795 overlaps (test 7, §6).
- So `point_a` is the box's support along −z: one of its bottom corners. `(±0.5, ±0.5, −0.2)` is exactly that; which corner depends on the 1e-10-level x/y noise in the normal.
- Even A's support along **+normal** would be wrong. For a flat face it is one of the face's corners, `(±0.5, ±0.5, 0)`, still 0.7 m from the cylinder. So the fix is the polytope witness, not a sign flip.

**Same defect, other consumers** of `gjk_epa_contact().point`. Measured where listed; mesh and hfield are in §4.4:
- every GJK pair in `narrow.rs`: cylinder–cylinder, cylinder–ellipsoid, ellipsoid–ellipsoid, sphere–ellipsoid, capsule–ellipsoid, box–cylinder, box–ellipsoid, and the capsule–cylinder fallback (`narrow.rs:156-168`);
- mesh hull vs box/cylinder/ellipsoid/mesh (`mesh_collide.rs:37`, `:74`);
- hfield prisms (`hfield.rs:167`, `:187`);
- flex vertex vs mesh hull (`flex_narrow.rs:177`, `:189`; **not measured**).

### 3.2 Capsule: the normal sign

**MuJoCo.**
- `mjc_CapsuleBox` (`engine_collision_box.c:595-600`) calls `mjraw_CapsuleBox` (`:119`), which calls `mjraw_SphereBox` per contact (`:579`, `:587`).
- Its frame is `mat2 · (clamped − center)/|·|` (`:54`, `:83`): from the capsule (g1) toward the box (g2).
- Its position is the midpoint of the box surface point and the sphere's deepest point (`:79-82`).

**Ours.**
- `collide_capsule_box` (`pair_cylinder.rs:262`) takes its normal from `sphere_box_test`. That normal is `(sphere_center − box_pt)/dist` (`:318`, `:326-327`), i.e. from the box toward the capsule.
- `normal_sign = if capsule_geom < box_geom { 1.0 } else { -1.0 }` (`:305`) keeps that box→capsule direction when the capsule is g1, and flips it to capsule→box when the box is g1. **Both are the reverse of g1→g2.**
- Measured in both orders: `IF/xml/capsule_first.xml` (capsule is g1) gives +z, the box-first file gives −z. MuJoCo gives capsule→box = −z both times.
- The position `box_pt + normal·(pen/2)` (`:349`) lies on the capsule side of the box surface, outside both shapes. It should be `box_pt − normal·(pen/2)`.
- `collide_sphere_box` (`pair_convex.rs:348`) has the same position expression. Its normal sign is correct (`pair_convex.rs:356-363`); see §7 Q3.

### 3.3 Is 20 cm a threshold? No.

`IF/scripts/sweep.py … size` runs every primitive centred and upright, dropped 1 mm, for 1,000 steps, in both engines. Results are in `IF/out/sweep_main_size.tsv`.
- Slab half-thickness: 0.005, 0.01, 0.05, 0.1, 0.2, 0.5 m.
- Slab half-width: 0.1, 0.5, 2 m.
- Primitive size: 0.01, 0.05, 0.2 m (only where size ≤ half-width).
- "Centre below the box top at some step", on main: capsule **31/48**, cylinder **44/48**, ellipsoid **44/48**, 3-axis ellipsoid (0.05, 0.04, 0.03 × size) **43/48**, sphere 0/48, box 0/48.
- No thickness, width or size held all of them.

---

## 4. The fix and what it measured

The diff is `IF/out/fix.diff`, applied in `IF/repo_fix`.

```rust
// design/cf-geometry/src/query/epa.rs — EpaResult gains witness_a/witness_b,
// built from the closest face (MuJoCo epaWitness, engine_collision_gjk.c:1273)
fn from_face(vertices: &[MinkowskiPoint], face: &EpaFace) -> Self   // private
//   affine coords (l0,l1,l2) of face.normal*face.distance in the face's Minkowski triangle,
//   witness_a = Σ l_i·support_a_i, witness_b = Σ l_i·support_b_i
// epa_penetration: point_a = witness_a, point_b = witness_b (was a.support(-n), b.support(n))
// Penetration.normal doc: "from A toward B" (was "from B toward A")

// sim/L0/core/src/gjk_epa.rs:587
point: nalgebra::center(&pen.point_a, &pen.point_b),   // was pen.point_a  (MuJoCo pos = ½(x1+x2))

// sim/L0/core/src/collision/pair_cylinder.rs
let normal_sign = if box_geom < capsule_geom { 1.0 } else { -1.0 };   // :305, was inverted
let contact_pos = box_pt - normal * (penetration * 0.5);              // :349, was `+`
```

- No public signature changes.
- Semantics change for two public fields:
  - `cf_geometry::Penetration::{point_a, point_b}`: now true witnesses; before, A's and B's far-side support points;
  - `sim_core::gjk_epa::GjkContact::point`: now the witness midpoint; before, `point_a`.

### 4.1 Static narrowphase: the contact itself

`IF/scripts/static_cmp.py` sets random orientations at a sampled penetration of −0.2 % to +4 % of the size. Positions are sampled over the face, at an edge, or at a corner. Both engines run `mj_forward`, and the script compares the deepest contact. Outputs: `IF/out/static_s005.txt`, `IF/out/static_regions.txt`. Shown here: max over 200–400 poses per row, all regions and sizes (s = 0.01, 0.05, 0.2 on a 1 × 1 × 0.2 slab, s = 0.05 on a 4 × 4 × 0.01 slab).

| pair | | max \|Δdist\| | max normal angle | max \|Δpos\| |
|---|---|---|---|---|
| box–capsule | main | 8.3e-17 | **π (every contact)** | 7.9e-3 (= depth) |
| | fix | 8.3e-17 | 1.5e-8 | **1.1e-15** |
| box–cylinder | main | 8.5e-7 | 5.6e-2 | **5.7 m** (every contact) |
| | fix | 8.5e-7 | 5.6e-2 | **7.7e-4** |
| box–ellipsoid (3-axis) | main | 9.8e-7 | 2.8e-2 | **5.7 m** (every contact) |
| | fix | 9.8e-7 | 2.8e-2 | **3.3e-4** |

- The distance and normal columns are identical in main and fix: the fix changes only position and capsule sign.
- The normal angle is ≤ 2.4e-6 on face poses for cylinder and ellipsoid. The larger values are edge and corner poses.
- For a cylinder cap on a face, the touching set is a disc and any point in it is a valid witness. Ours and MuJoCo pick points within 7.7e-4 m of each other.

Other GJK pairs, a fixed geom against a free one at random poses (`IF/scripts/pair_static.py`, `IF/out/pair_static.txt`, 300 poses each). Shallow contacts (< 1 cm), max |Δpos|, main → fix:

| pair | main → fix |
|---|---|
| ellipsoid–ellipsoid | 0.23 m → 2.2e-5 |
| cylinder–cylinder | 0.26 → 3.0e-5 |
| cylinder–ellipsoid | 0.26 → 9.1e-6 |
| sphere–ellipsoid | 0.20 → 2.6e-6 |
| capsule–ellipsoid | 0.35 → 8.0e-6 |

### 4.2 Dynamics: resting, size sweep

The size sweep is `IF/out/sweep_fix_size.tsv`. Its results are the max over the 48 cases per primitive and over all 1,000 steps:

| | capsule | cylinder | ellipsoid | 3-axis ellipsoid | sphere / box (untouched) |
|---|---|---|---|---|---|
| centre below box top (main → fix) | 31 → **0** | 44 → **0** | 44 → **0** | 43 → **0** | 0 → 0 |
| max \|z_ours − z_MuJoCo\| over the run (fix) | **4.2e-13** | **4.4e-4** | **2.2e-6** | **4.4e-6** | 4.2e-13 / 1.0e-13 |

- The cylinder is the loose one in every table. MuJoCo's own single-contact cylinder rocks on its cap: its contact point jumps between `±(0.035, 0.035)` over steps 9–30 (`IF/out/mj_slab10_cylinder.jsonl`).
- Ours rocks too, at different points (`IF/out/fixepa_slab10_cylinder.jsonl`).
- Both end within 7.2e-5 m on the hand-off fixture (MuJoCo 0.0475508, fix 0.0474791).

### 4.3 Dynamics: tilted, edge and corner, both geom orders

The pose sweep is `IF/scripts/sweep.py … pose`: 480 cases, outputs in `IF/out/sweep_{main,fix}_pose.tsv`.
- 6 primitives.
- Euler angles (0,0,0), (30,0,0), (90,0,0), (20,35,10), (45,45,0).
- (x, y) = centre, edge (0.5, 0), corner (0.5, 0.5), near-corner (0.45, 0.3).
- Box listed first, or on a body after the free body.
- Drop height 1 mm or 5 cm.

`IF/scripts/classify.py` sorts each run as "through" (centre below the box top while over the footprint), "off" (left the footprint: edge and corner placements topple off), or "rest":

| | main: ours / MuJoCo | fix: ours / MuJoCo |
|---|---|---|
| capsule | through/off 48, through/rest 32 | off/off 48, rest/rest 32 |
| cylinder | through 20, off/rest 8, rest/off 1, matching 51 | off/off 44, rest/rest 36 |
| ellipsoid | through/rest 20, matching 60 | off/off 40, rest/rest 40 |
| 3-axis ellipsoid | through/rest 20, off/rest 6, matching 54 | off/off 40, rest/rest 40 |
| sphere, box | all matching | all matching (bit-identical to main) |

**With the fix, all 480 cases have MuJoCo's outcome.** For the cases where both rest (`IF/scripts/agree.py`):

| fix, rest cases | max \|Δz\| over run | max \|Δz\| final | \|ΔFz̄\|/Fz̄ (last 100 steps) | final orientation Δ |
|---|---|---|---|---|
| capsule (32) | 8.1e-8 | 9.5e-14 | 7.2e-10 | 8.9e-6 rad |
| ellipsoid (40) | 2.3e-7 | 1.0e-7 | 2.4e-7 | 2.7e-2 rad (a sphere-shaped ellipsoid's spin) |
| cylinder (36) | 7.8e-4 | 7.8e-4 | 1.1e-3 | 0.35 rad |
| 3-axis ellipsoid (40) | **1.05e-2** | 5.0e-3 | 9.9e-2 | 0.65 rad |
| box–box, untouched (40) | 6.3e-4 | 1.6e-5 | 6.9e-6 | 9.4e-4 rad |

- The 3-axis ellipsoid's worst cases are tumbling drops (`e45_45`, `e90`). These are the GJK contacts whose normals differ from MuJoCo's by up to 2.8e-2 rad (§4.1). That difference exists on main too.
- **What makes the tumbles differ has not been isolated.**
- For the toppling ("off") cases, the first 50 steps agree to ≤ 2.7e-4 (capsule), 3.8e-4 (cylinder), 9.7e-8 (ellipsoid) and 5.7e-6 (3-axis).

### 4.4 The same defect on convex meshes and height fields

**Convex mesh slab** (an 8-vertex 1 × 1 × 0.2 box mesh with faces, `IF/xml/mesh/`, `IF/out/mesh_pairs.txt`), 1,000 steps, final z:

| | main | fix | MuJoCo | max \|Δz\| fix vs MuJoCo |
|---|---|---|---|---|
| box on mesh, upright / tilted | **−19.57 / −19.50** | 0.04529 / 0.04523 | 0.04527 / 0.04526 | 1.9e-4 / 1.8e-4 |
| cylinder on mesh, upright / tilted | **−19.31 / −19.34** | 0.04751 / 0.04769 | 0.04748 / 0.04762 | 3.4e-5 / 1.0e-4 |
| ellipsoid on mesh, upright / tilted | **−19.57 / −19.48** | 0.029633 / 0.04101 | 0.029633 / 0.04105 | 2.7e-14 / 2.6e-4 |
| free convex mesh ("gem") on a box, upright / tilted | **−19.56 / −19.56** | 0.04963 / 0.04009 | 0.04963 / 0.04008 | 7.5e-10 / 7.9e-5 |
| sphere, capsule on mesh; gem on plane | equal to MuJoCo; main = fix bit-identical (BVH / plane paths) | | | |

- This bears on `mesh_inertia_hull.md` §5. That section says box, cylinder and ellipsoid fall through even a *planar* hull. It was measured before this fix; **re-measure it on top of this fix** (not done here).

**Height field**, 8 × 8, inline elevation (`IF/xml/hf/`, `IF/out/hfield_pairs.txt`):
- **Flat** (all zeros): the fix moves every primitive toward MuJoCo. Max |Δz| main → fix:
  - box 4.3e-3 → 1.3e-3
  - sphere 2.3e-3 → 7.1e-5
  - ellipsoid 1.1e-2 → 1.5e-5
  - capsule 2.8e-2 → 3.2e-3
  - cylinder 5.0e-3 → 3.6e-4
- **Flat, falls through on main:** the tilted box and upright ellipsoid fall through on main and rest with the fix.
- **Bumpy** (values 0, 0.5, 1): sphere, capsule, ellipsoid and the tilted cylinder **still fall through with the fix** (they fall on main too). At the first contact the normal's y component is mirrored relative to MuJoCo's. The first contact also comes later and lower: ours at step 78, z 0.0798; MuJoCo at step 75, z 0.0888 (`IF/out/hf_hf_bumpy_sphere_up.*`). The surfaces the two engines see differ. **Not isolated** (§7 Q4).
- **Two defects outside this fix:**
  - **ours does not normalise inline elevation.** MuJoCo maps it to [0, 1] (`user_objects.cc:4559-4573`). Measured: with constant 0.5 data, a sphere rests at 0.0996 in ours and 0.0496 in MuJoCo (`IF/out/hfconst_sphere.*`). The fixtures above use data whose normalisation is the identity;
  - the bumpy-hfield fall-through above.

### 4.5 Unaffected pairs stay bit-identical

**Sweeps.** Every sphere and box trace is byte-identical between main and fix: 48 + 48 in the size sweep, 80 + 80 in the pose sweep (`cmp` of the `.ours.jsonl` files).

**Corpus** (`SCRATCH/rigid_plan_tests/corpus`, 1,584 docs, 100 steps with the excitation pattern):
- Harness: the **streaming** fingerprint `harness2.rs` from `determinism_verification/harness`. I added one output field, `pairs`: the geom-type pairs of every contact seen. Copies are in `IF/harness_{main,fix}`.
- Runs: main ×3 and fix ×3. Comparison in `IF/out/corpus_emb_cmp.txt` (`IF/scripts/corpus_cmp.py`).
- **1,505 unchanged, 67 nondeterministic, 12 changed.**

  | changed (12) | why |
  |---|---|
  | 3 `model_fp` only (`1aab796f…`, `1fdcd7bb…`, `6cb090b7…`) | all three take 2 distinct values over the 10 main runs in `determinism_verification/runs/before_*` (ledger-L27 hash order), not the fix |
  | 9 trajectory: 7 with box–capsule contacts, `6369216c…` (capsule–cylinder fallback), `87d99b20…` (cylinder–cylinder, ellipsoid–sphere) | every one has a contact pair the fix touches (attributed by pair type; per-commit runs not done) |

- The 67 nondeterministic docs (the known L27 class) have only flex, flex-self or no contacts in 100 steps. None touches a changed pair.
- **Every deterministic doc whose contacts are only of unaffected types is identical:**

  | pair type | docs |
  |---|---|
  | plane–sphere | 106 |
  | sphere–sphere | 33 |
  | box–plane | 25 |
  | box–box | 15 |
  | capsule–capsule | 9 |
  | capsule–plane | 9 |
  | cylinder–plane | 8 |
  | box–sphere | 7 |
  | ellipsoid–plane | 6 |
  | capsule–sphere | 4 |
  | cylinder–sphere | 4 |
  | flex–box | 1 |
  | flex–sphere | 1 |

- 6 docs touch a changed pair type and still did not change:
  - 2 box–capsule docs (`2c4fddb6…`, `e2bd912f…`). The contact exists, and the fix moves it to MuJoCo's `pos`/normal (checked: pos z 2.07 → 2.03 = MuJoCo). Why the trajectory bits are unchanged was not measured;
  - 2 capsule–cylinder docs (analytic path, no fallback);
  - 1 flex–sphere and 1 flex–box doc.
- The 15 repo `.xml` files are identical.
- The harness can fail: it shows the 9 trajectory changes above.

**Golden flags** (`--ignored golden_`): 24 fail on both, with identical first-mismatch lines (`IF/out/golden_{main,fix}.log`).

### 4.6 Changed corpus docs against MuJoCo

`IF/scripts/doc_cmp.py` runs 1,000 steps without excitation; output in `IF/out/changed_docs_vs_mj.tsv`. Max |Δqpos| vs MuJoCo, main → fix:

| doc | source | main → fix |
|---|---|---|
| `53df39ca…` | `collision_primitives.rs:1171` | 1.08 → **6.0e-16** |
| `636fc5d5…` | `:1279` | 1.53 → **2.5e-15** |
| `f56833a8…` | `:1406` | 2.16 → **3.6e-15** |
| `f935e97a…` | `:1323` | 0.51 → **1.9e-8** |
| `b86fda11…` | weld example, Bevy app | 1.9e-3 → **1.1e-15** |
| `f8879911…` | `:1363`, capsule across a box edge | 1.64 → **0.124** |
| `87d99b20…` | `sim/L1/bevy/examples/collision_shapes.rs:31` | 11.17 → 1.84 |
| `6369216c…`, `7f76ba4f…` | | 0 in both (they differ only under the harness excitation) |

- **`f8879911…` remainder.** It comes from contact **selection**, not sign:
  - ours makes 2 contacts, at `(0, 0.5, 0.475)` and the edge `(0, 0.497, 0.497)`;
  - MuJoCo makes 1, at the other end, `(0, −0.5, 0.475)`;
  - see §7 Q2.
- **`87d99b20…` remainder.** The scene starts four bodies 1.5 m *below* the ground plane. Fix and main both first diverge from MuJoCo at step 29, with only plane contacts active. Not isolated.

### 4.7 Hook checks on the fix copy

- `cargo clippy -p {cortenforge-geometry, cortenforge-sim-core} --all-targets --all-features -- -D warnings` exits 0. The first run failed on `suspicious_operation_groupings` and `doc_markdown`; both are fixed in the diff.
- `rustfmt --check` on the three files exits 0.
- After the lint edits, 27 static outputs and 480 pose traces were re-run and are byte-identical to the measured ones.

---

## 5. In-tree tests and examples

**Suites, main vs fix,** each in its scratch copy (`IF/out/tests_{main,fix}/`):

| suite | main | fix |
|---|---|---|
| cortenforge-geometry (lib, tests, doc) | 394 pass / 1 ignored | identical |
| cortenforge-sim-core `--lib` | 720 | identical |
| cortenforge-sim-mjcf (lib, forward_conformance, doc) | 387 / 3 ignored | identical |
| sim-conformance-tests `integration` | 1338 / 27 ignored | identical |
| sim-conformance-tests `mujoco_conformance` | 83 | identical |
| cortenforge-sim-urdf | 49 / 1 | identical |
| cf-design-tests | 9 / 5 | identical |
| validator `example-mesh-collision-stress-test` | 21/21 PASS | 21/21 PASS |
| validator `example-urdf-stress-test` | exit 0 | exit 0, output identical |

- **Integration on main:** the first run of `collision_performance::scaling_collision_bodies` failed ("2× bodies → 28× time"). It ran concurrently with the fix suite. Alone it passes 3/3, so it is a timing test under load, not a flip.
- **Mesh-collision validator:** only its F section changes. The ellipsoid rests at z 0.0478 → **0.0496** (expect ≈ 0.05), and force/weight goes 0.9921 → **1.0000**.

**No test I ran flips, and none pinned the wrong behaviour. They never exercised it:**
- `collision_primitives.rs:1138-1446`, the capsule–box tests: they check `ncon` and depth only, never the normal or position.
- `pair_cylinder.rs:1139` `capsule_box_face_penetration` asserts `normal.x.abs()`, so it is blind to the sign.
- The `epa.rs` tests (`:317-478`) never read `point_a`/`point_b`.
- `mesh_cylinder_ellipsoid.rs`:
  - T10 `mesh_cylinder_settling` (`:109`) runs with MULTICCD. That path uses face points of A (`narrow.rs:284-325`), not the EPA witness. Its contact positions on main were **not measured**;
  - T11 `mesh_ellipsoid_settling` (`:183`) passes on main and on the fix. Why it does not catch the far-corner contact was **not isolated**: my smaller ellipsoid on a mesh slab falls through on main (§4.4).

**Static examples whose behaviour changes** (§4.5, §4.6; CI reach from `determinism_verification/fliptable.tsv`):
- CI tests:
  - `collision_primitives.rs:1171, 1279, 1323, 1363, 1406`;
  - `collision_performance.rs:292`;
  - `sim/L0/mjcf/src/lib.rs:226` (unit test `test_two_link_arm_model_data`).

  All of them pass after the fix.
- Never run:
  - `sim/L1/bevy/examples/collision_shapes.rs:31` (cargo example);
  - `examples/fundamentals/sim-cpu/equality-constraints/weld-body-to-body/src/main.rs:42` (Bevy app);
  - the 7 Bevy `mesh-collision/*` apps. Their docs are ledger-L27 nondeterministic and had no contact in 100 steps, so the harness cannot see them.

**GPU:** `sim/L0/gpu` has no primitive-pair narrowphase. Non-SDF pairs are CPU-only (`sim/L0/gpu/src/pipeline/collision.rs:630`, `_ => None, // Non-SDF pairs: CPU-only`). Nothing to change there; sim-gpu was not run.

---

## 6. Tests to add (each fails on main and passes on the fix; measured)

The scratch versions are `IF/out/fallthrough_tests.rs` (crates `IF/regress_{main,fix}`, `--release`). Main: **0/7 pass**. Fix: **7/7 pass**. Values are from `IF/out/regress_fix_values.txt`.

| # | test | input | main | fix | MuJoCo 3.5.0 golden |
|---|---|---|---|---|---|
| 1 | `capsule_rests_on_box_as_mujoco` | capsule r = h = 0.05 on a 1 × 1 × 0.2 box slab, 1,000 steps | z −0.0718 | z 0.0996328181574896 | 0.09963281815753983; tolerance 1e-9 |
| 2 | `ellipsoid_rests_on_box_as_mujoco` | sphere-shaped ellipsoid 0.05 on the same slab | z −19.46 | 0.0496328180 | 0.04963279732; tolerance 1e-6 |
| 3 | `cylinder_rests_on_box_as_mujoco` | cylinder 0.05 / 0.05 on the same slab | z −19.43 | 0.0474791 | 0.0475508; tolerance 1e-3 (rocking contact), Fz̄ within 5 % of weight |
| 4 | `box_cylinder_ellipsoid_rest_on_convex_mesh_slab_as_mujoco` | the §4.4 mesh slab | box z −19.57 (the loop stops at the first failure) | within 3.3e-5 | MuJoCo's three finals; tolerance 1e-3 |
| 5 | `capsule_box_normal_points_from_geom1_to_geom2_and_pos_is_the_midpoint` | static, both geom orders | normal reversed | normal·(x_g2 − x_g1) > 0; depth 5e-4 and pos (0, 0, −2.5e-4) to 1e-12 | MuJoCo's dist and pos |
| 6 | `epa_contact_point_is_between_the_two_surfaces_in_both_orders` | `gjk_epa_contact`, slab vs ellipsoid, both argument orders | point at a slab corner | within 1e-4 of (0.1, −0.05, −5e-5) | — |
| 7 | `epa_witness_difference_is_depth_times_normal` | 1,795 random overlaps of box, ellipsoid, cylinder, capsule | 0.58 | 3.5e-12 | asserted ≤ 1e-10 |

- Test 7 guards the witness definition directly.
- In the repo, 1–3 and 5 belong in `sim/L0/tests/integration/collision_primitives.rs` and 4 in `mesh_cylinder_ellipsoid.rs`.
- 6 belongs in `gjk_epa.rs` tests, and 7 in `epa.rs` tests (via `PosedShape`, or cf-geometry's own `Translated` helper).

---

## 7. Per item: Now / Target / Change / Downstream / Open questions

### ledger-L42a: GJK/EPA contact position

- **Now:** `epa.rs:74-75` uses each shape's far-side support (§3.1). `gjk_epa.rs:587` takes `point_a`. The docs at `epa.rs:25` and `gjk_epa.rs:64` give the normal as B→A; the measured normal is A→B. `mesh_collide.rs:19-22` carries the same wrong convention in a comment.
- **Target:** MuJoCo `epaWitness` (`engine_collision_gjk.c:1273-1287`), with contact `pos = ½(x1 + x2)` (`engine_collision_convex.c:807-812`).
- **Change:**
  - private `EpaResult { depth, normal, witness_a, witness_b }`;
  - private `EpaResult::from_face(&[MinkowskiPoint], &EpaFace) -> Self`;
  - `epa_penetration` returns the witnesses;
  - `gjk_epa_contact` returns their midpoint;
  - rewrite the three comments that give the normal as B→A: `epa.rs:25`, `gjk_epa.rs:62-64`, `mesh_collide.rs:19-22`.
- **Downstream:**
  - `epa_penetration` and `GjkContact` have no in-tree caller outside cf-geometry and sim-core (`git grep 'GjkContact|gjk_epa_contact|epa_penetration'`);
  - inside sim-core, the callers are `narrow.rs:188`, `mesh_collide.rs:37`, `hfield.rs:167` and `flex_narrow.rs:177`;
  - `cortenforge-geometry` is published, so external users of `Penetration::point_a` see the semantics change (0.10.0 allows that).

### ledger-L42b: capsule–box normal sign and position

- **Now:** §3.2, `pair_cylinder.rs:305` and `:349`.
- **Target:** `mjraw_SphereBox`'s frame and pos (`engine_collision_box.c:54`, `:79-83`), reached through `mjraw_CapsuleBox` (`:119`, `:579`, `:587`).
- **Change:** the two lines, with comments citing MuJoCo.
- **Downstream:** `collide_capsule_box` is `pub`, but `git grep` finds no caller outside `sim/L0/core/src/collision/`.

### Open questions (the fixed decisions do not settle these)

1. **MULTICCD keeps the far-corner contacts.**
   - `multiccd_contacts` (`narrow.rs:284-325`) places every contact at `support_face_points(shape_a, …, normal)` (`:299`): the face corners of A.
   - Measured with `<flag multiccd="enable"/>` (`IF/out/opt_multiccd_margin.txt`):
     - a cylinder on the slab gets 4 contacts at the slab's corners `(±0.5, ±0.5, 0)`. MuJoCo makes 5 on the cylinder's rim;
     - z still agrees to 1.8e-5 (why corner contacts hold it was not isolated);
     - a 3-axis ellipsoid gets the same 4 corners and rests 2.8e-2 off MuJoCo. MuJoCo skips multiccd for spheres and ellipsoids (`engine_collision_convex.c:935-937`) and uses a perturbation search for the other pairs (`:935-998`).
   - Options: (a) port MuJoCo's multiccd in this PR; (b) a new ledger row.
   - I recommend (b): it is a separate algorithm, and with this fix the non-MULTICCD default holds.
   - If that is wrong: MULTICCD users keep corner contacts on large flat geoms. That is visible as torque artefacts when the body is off-centre (not measured).
2. **Capsule–box contact selection and deep overlap still differ from `mjraw_CapsuleBox`.** Shallow contacts match to 1.1e-15 (§4.1). But:
   - `ncon` differs in 7–11 % of edge and corner poses. Over all capsule static runs there are 65 `ncon` mismatches. In 55, MuJoCo emits a duplicate contact at the same `pos` and ours deduplicates it (`pair_cylinder.rs` `dedup_dist_sq`). In the other 10, ours emits a second, distinct contact. The shared contact matches in all 65;
   - overlaps ≥ 1 cm differ by up to 5.85e-2 in dist and π/2 in normal (`IF/out/pair_static.txt`). Our sphere-centre-inside branch sets `penetration = r` (`pair_cylinder.rs:320`, `:329-346`); MuJoCo's is `closest + r` (`engine_collision_box.c:61-77`);
   - the T-cross doc (`collision_primitives.rs:1363`) ends 0.124 m off.

   Options: (a) port `mjraw_CapsuleBox` (`engine_collision_box.c:119-592`) in its own commit; (b) a new ledger row. I recommend (a) under the parity rule. If that is wrong, edge, corner and deep capsule–box contacts stay non-MuJoCo, as measured above.
3. **Sphere–box has the same position offset and deep-overlap branch.**
   - Position: `pair_convex.rs:348` is the `+` form, so |Δpos| = depth (≤ 9.4e-3 for depth < 1 cm).
   - Deep overlap: dist differs by up to 3.5e-2.
   - The brief requires sphere–box to stay bit-identical in *this* fix, so it is untouched.
   - In the sweeps, resting sphere trajectories match MuJoCo to 4.2e-13. Rolling or sliding spheres were not measured.
   - Options: fix it in a separate commit, with flips limited to sliding or rolling spheres (not measured), or list it as a divergence. I recommend fixing it. If that is wrong, sphere–box friction torque keeps a lever arm that is off by the depth.
4. **Height fields** (§4.4): no normalisation of inline elevation, a mirrored normal on a non-symmetric field, and fall-through on a bumpy field with or without this fix. Not isolated; a new ledger row.
5. **Margin with an ellipsoid.**
   - With `margin = 0.01` on the box, MuJoCo makes a contact from step 1 (dist +1e-3) and rests the 3-axis ellipsoid at 0.0396.
   - Ours makes no contact until penetration (step 8) and rests at 0.0300 (fix; on main it falls through).
   - The margin-zone path is `narrow.rs:234-267` (`gjk_distance`). The cylinder with a margin agrees to 9.7e-4 and the capsule to 1.9e-13.
   - Not isolated; a new ledger row.
6. **Contact geom order.**
   - MuJoCo swaps g1 and g2 so that `type1 ≤ type2` (`engine_collision_driver.c:1475-1480`). Ours keeps index order.
   - Physics is identical: §4 compares normals after orienting them. But `Contact.geom1/geom2` and the normal's sign differ from MuJoCo whenever the lower-index geom has the higher type.
   - Options: match MuJoCo, which is breaking for code that reads contacts, or list it as a divergence. **No recommendation**: I did not survey who reads `Contact.geom1`.

## 8. Commit list and dependencies

| # | commit | contents | tests carried | flips |
|---|---|---|---|---|
| F1 | `fix(cf-geometry,sim-core): EPA witness points from the closest face; GJK contact at their midpoint (MuJoCo epaWitness)` | `epa.rs`, `gjk_epa.rs`, the three comment rewrites | 2, 3, 4, 6, 7 | none of the suites in §5. Corpus trajectory changes: `6369216c…` (CI, passes) and `87d99b20…` (never run). Validator mesh-collision F section numbers improve |
| F2 | `fix(sim-core): capsule–box contact normal from geom1 to geom2, position at the midpoint (MuJoCo mjraw_SphereBox)` | `pair_cylinder.rs:305`, `:349` | 1, 5 | none of the suites. Corpus: 7 box–capsule docs (5 CI tests and 1 unit test, all passing; 1 Bevy app) |

- F1 and F2 touch disjoint code. Their order is free.
- Each was only measured as part of the combined fix, so per-commit green is **not measured**.
- No dependency on another area's code.
- Re-measure the flat-mesh verdict in `mesh_inertia_hull.md` §5 (MH6) on top of F1. §4.4 shows that the F1 defect makes box, cylinder and ellipsoid fall through a convex mesh slab on main. Whether it is also what made them fall through their planar hull was not measured.
- Q2 and Q3 would be F3 and F4 if accepted. Q1, Q4 and Q5 need new ledger rows.

## What this method cannot see

- **MuJoCo's EPA is not ported.** Contact positions on flat-face contacts and normals at edges and corners agree within the tolerances above, not bitwise. So the cylinder and the tumbling 3-axis ellipsoid are matched to 7.8e-4 m and 1.05e-2 m respectively, not to round-off.
- **Not run:**
  - sim-gpu, sim-thermostat, sim-rl, ml-chassis, opt, L1 crates, cf-osim, cf-msk-fit, cf-codesign;
  - the validator fleet other than mesh-collision and urdf;
  - the licensed gates; `cargo xtask grade`;
  - runtime-generated MJCF (`format!` templates), other than through the two validators.
- **Not measured:**
  - flex vertex vs mesh hull (`flex_narrow.rs:177`);
  - sleep;
  - integrators other than Euler;
  - elliptic cones;
  - margins other than the one case in Q5;
  - noslip.
- **Corpus blind spot:** the 67 nondeterministic corpus docs cannot be compared by the harness (none had an affected contact in 100 steps). Contacts that begin after step 100 are invisible to it.
- **Sweep limits:** a single geometry family per sweep (one slab, primitives ≤ 0.2 m), timestep 0.002 only.

**Repo state.**
- Before: branch `fix/rigid-sim-core-mjcf` @ `139f7b25`, `git status --short` empty.
- After: `git status --short` is still empty, but HEAD is `e48267f8`, a commit I did not make. I ran no git write command.
- That commit adds only `sim/docs/todo/spec_fleshouts/rigid_0_10/A7…A14*.md` (`git diff --name-only 139f7b25 HEAD`). `git diff --name-only 3520544e HEAD -- ':!sim/docs'` is empty, so the source I built against equals `3520544e`.
- All probes ran in `git archive` copies (`IF/repo_main`, `IF/repo_fix`) with their own target dirs. Those target dirs were deleted at the end. Every `.jsonl` over 200 kB under `IF/out/` was gzipped in place. A `.jsonl` path cited above may therefore be `.jsonl.gz` on disk. To re-run, rebuild the probes from `IF/probe*/` and `IF/harness_*/`.
