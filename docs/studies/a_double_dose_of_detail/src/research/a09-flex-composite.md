> Research for *A Double Dose of Detail*, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid spec — flex and composite parity (dim-3 tet orientation, flex flaps / `elastic2d`, 2-body and 1-body cables, the composite `curve` keywords)

Researcher section, drafted 2026-10-06 against HEAD `ea11c8d6` (branch `fix/rigid-sim-core-mjcf`; code identical to `main` @ `3520544e`). Read-only on the repo; `git status --short` empty before and after (last section).

**Scratch:** `SCRATCH/rigid_spec/flex_composite/` (written `FC/` below).
- `FC/base` = `git archive HEAD` + A6's `l27_probe.patch` (flex edges in MuJoCo order, "M3"). Every "on main" measurement below is on this copy, i.e. main **with M3 applied**, as the brief says to assume.
- `FC/fix` = `FC/base` + my prototype, **env-switched** so one build gives every variant: `FC_REORIENT` (item 1), `FC_VOLABS` (unsigned rest volume), `FC_FLAPS` (item 2a), `FC_E2D` (item 2b), `FC_CURVE` (MuJoCo curve keywords, item 4), `FC_F32VERT` (f32 cable vertices, item 4). The env switches are scratch-only. The PR code is specified under "Change".
- Probes: `FC/probe_{base,fix}` (own `target/`; bin `fcdump` dumps flex arrays, bodies/joints/geoms/sites/excludes and trajectories as JSON; bin `harness` = A6's corpus harness, unchanged).
- MuJoCo 3.5.0 side: `FC/scripts/mj_flex.py`, `mj_model.py`, `mj_bend_rest.py`, `mj_bend_force.py` (run with `-I`).

---

## 0. Decisions and open items

| item | decided here (referent) | open (Q = §6) |
|---|---|---|
| 1 dim-3 tet orientation | Port MuJoCo's swap (`user_mesh.cc:4272-4293`) into the builder. **Measured: elems and `flex_edge` then equal MuJoCo's for 6/6 dim-3 cases (1–135 tets, mixed and shuffled orientation).** No sim-core code reads tet orientation (§1.4): 1 corpus doc changes its model, 0 change trajectory, and 1 CI test flips (only through `flexelem_volume0`'s sign). | Q1: `flexelem_volume0` unsigned (recommended) or deleted |
| 2a flap representation | Flaps for dim 2 only. A boundary edge is `[opp, -1]`; dim 1 and dim 3 get no flaps or hinges. **Measured: 46 docs model-only, 0 trajectory, 0 test flips.** | — |
| 2b `elastic2d` | Implement it (MuJoCo default `none`). Flaps, bending coefficients and our Bridson hinges exist only for `bend`/`both`; `stretch`/`both` are refused (limitation). The 39 in-tree flexes that have bending today get `elastic2d="bend"`: **measured 0 trajectory changes, 0 test flips**. Without that rewrite, 21 docs change trajectory and 12 CI tests flip. | Q2 bending-coefficient order (MuJoCo's defect); Q3 pins + bending; Q4 non-manifold edges; Q7 `bending_model` without `elastic2d` |
| 3 2-body cables | MuJoCo's refusal is in its own `AddCableBody` (`user_composite.cc:351-353` vs `:356-357`). The fixed check comes from **MuJoCo's own generator via MjSpec**: the exclude renamed `B_1`→`B_last` and the result written out as explicit XML at 17 digits. **Measured: our cable = MuJoCo's explicit model to ≤ 4.25e-15 qpos / 1.09e-13 qvel over 500 steps, 7 of 7 docs that ours loads.** No code change: ours already loads them. A golden test pins it. | — |
| 3′ 1-body cables (new) | MuJoCo reads uninitialised `tnext` (UB). 3.5.0 gives a NaN body quaternion (4/4 measured), 3.4.0 a finite one. Ours gives a correct frame (300/300 directions) → lenient, listed, tested. | Q6 the second site |
| 4 `curve` keywords | Our map is wrong, as A3/A4 said (MuJoCo `xml_native_reader.cc:851-856`, `:2566`, `:2570`); the fix is M8 (parser area). After the fix and the doc rewrites, **every composite-generated quantity of the 17 docs both load equals MuJoCo's to ≤ 6.9e-16**, provided cable vertices pass through f32 as MuJoCo stores them (`user_composite.h:81`). Without that, the gap is up to 8.6e-8 (quaternions). 500-step trajectories agree to ≤ 4.25e-15 except where a `<connect>` equality is present (handed off). | Q5 f32 vertices (parity recommended); Q8 trailing whitespace |

---

## 1. Item 1 — dim-3 tetrahedron re-orientation

### 1.1 Now
- `sim/L0/mjcf/src/builder/flex.rs:107-128` stores each element in input order.
  - Its rest volume is **signed**: `e1.dot(&e2.cross(&e3)) / 6.0` (`:122`). The comment there ("to match the constraint assembly formula") names a consumer that does not exist (§1.4).
- Edges come from the element order (`extract_flex_edges`, `:407`; with M3 in MuJoCo's first-appearance order).
- So for a tet with `dot(cross(v01,v02),v03) > 0` our `flexelem_data` and `flexedge_vert` differ from MuJoCo's.
- Measured on the base copy (`FC/mj_tet.jsonl` vs `FC/ours_tet_base.jsonl`, `scripts/cmp_flex.py`):

| case (FC/cases) | tets | ours = MuJoCo? |
|---|---|---|
| `tet_pos.xml` (0 0 0 / 1 0 0 / 0 1 0 / 0 0 1) | 1 | elems DIFF (ours [0,1,2,3], MuJoCo [0,2,1,3]); edge set same, order differs |
| `tet_neg.xml` | 1 | same |
| `tetbox3.xml` (our `generate_box_mesh` 5-tet split, 3³ grid) | 40 | elems DIFF 32/40 |
| `tetbox3_jit.xml` (jittered) | 40 | DIFF 32/40 |
| `tetbox3_perm.xml` (each tet's vertices shuffled) | 40 | DIFF 18/40 |
| `tetbox4_permjit.xml` | 135 | DIFF 64/135 |

### 1.2 Target (MuJoCo 3.5.0)
- `mjCFlex::Compile` computes vertex world positions at qpos0 (`user_mesh.cc:4236-4268`, `vertxpos`).
- For `dim == 3` it swaps elem[1] and elem[2] when `dot(cross(v01, v02), v03) > 0` (`:4272-4293`; "faces (0,1,2) (0,2,3) (0,3,1) (1,3,2)" outward).
- Then it creates edges from the re-oriented elements (`:4297-4321`).
- Parity, no deviation.

### 1.3 Change (sim-mjcf, private)
```rust
// sim/L0/mjcf/src/builder/flex.rs
/// MuJoCo 3.5.0 `mjCFlex::Compile` (user_mesh.cc:4272-4293): reorder each tetrahedron so its
/// faces (0,1,2) (0,2,3) (0,3,1) (1,3,2) are right-handed outward, by swapping vertices 1 and 2
/// when dot(cross(v01, v02), v03) > 0. `world_pos`: vertex world positions at qpos0.
fn orient_tetrahedra(world_pos: &[Vector3<f64>], elements: &mut [Vec<usize>])
```
- Call it in `process_flex_bodies` (`flex.rs:31`) on an owned copy of `elements` when `dim == 3`. It must run **before** `compute_vertex_masses`, `extract_flex_edges`, the flap/hinge pass and the element loop.
- Positions:
  - Use the same world positions the builder uses for rest lengths: `flex.vertices` after node resolution (`flex.rs:47-75`).
  - Once A4-Q1 adds MuJoCo's `<flex body= vertex=>` form, these must be MuJoCo's `vertxpos`: body `xpos0` plus the body rotation times the offset (`user_mesh.cc:4236-4268`). That form is not implemented today, so this path is unmeasured (§8).
- Arithmetic order matches MuJoCo's: `mjuu_crossvec` then `mjuu_dot3`; nalgebra's `cross` has the same component formula. A difference can only flip near-degenerate tets (dot ≈ 0); not measured.
- `flexelem_volume0` (`flex.rs:116-126`; pub `Model` field `model.rs:505`): after the swap our signed formula gives ≤ 0 for **every** tet (measured: 0 positive of 40/40/40/135, `FC/ours_tet_fix2.jsonl`). The sign no longer carries information. **Q1: store `|V|`** (prototype `FC_VOLABS`).
- Also update the field doc (`model.rs:504`) and the comment at `flex.rs:120-121`.

### 1.4 Every consumer of element vertex order and of the volume sign
Method: `git grep` of `flexelem_data|flexelem_dataadr|flexelem_datanum|flexelem_volume0|flexhinge_|flexedge_vert|flex_dim` over the whole workspace (non-`.md`). Outside `sim/L0/{core,mjcf,tests}` it returns **0 files**. The same grep returns hits inside, so the empty result is a real absence.

| consumer | file:line | depends on vertex order? | evidence |
|---|---|---|---|
| `flexelem_volume0` | `model.rs:505`; written `flex.rs:126` | **no reader in sim-core**. Its one reader is the test `flex_unified.rs:409` (`> 0`) | git grep |
| 3D elasticity / volume constraints | — | **do not exist in sim-core**: `flex_young`/`flex_poisson` are read nowhere outside `model.rs`/`model_init.rs`; the flex block of `equality_assembly.rs:76-135` is edge-length rows only | git grep; reading |
| element faces in collision | `collision/flex_collide.rs:75-78` (`elem_vertices`), `:135-140` (tet faces), `:30-70` (internal), `flex_self.rs:151-167` (`collide_tetrahedra`), `:66-84` | face **set** unchanged; order and winding change. The sphere–triangle normal is two-sided (`flex_narrow.rs:453`), so the contact set is the same and its push order can differ | reading + trajectories below |
| element AABBs / BVH / adjacency | `flex_collide.rs:250`, `flex_self.rs:342`, `flex.rs` `compute_element_adjacency` (sorted) | no | reading |
| vertex mass lumping | `flex.rs:690` uses `.abs()` | value no; last bit yes (the triple product is evaluated in another operand order) | `body_mass` bit-unequal on 2 of 7 cases |
| edge order (follows element order) | passive edge springs `forward/passive.rs:500-547`; edge constraint rows `equality_assembly.rs:77`; `dynamics/flex.rs:31` | summation / row order only | trajectories below |
| dim-3 hinges | built by `flex.rs:459-519` when 2 tets share an edge; read only by Bridson, which skips `dim != 2` (`passive.rs:558`) | built, never read | item 2a removes them |

**Dynamics, measured** (`FC/traj_tet_{base,fix}.jsonl`, 1000 steps, `scripts/cmp_traj.py`):
- `tet_pos`, `tet_neg` and the corpus tet (`ac3_tet` = `flex_unified.rs:62`): **bit-identical** before and after.
- The tet boxes differ by max |Δqpos| 1.3e-6 … 3.1e-6 over 1000 steps (displacement scale 2.08). The same mesh with only its tet vertex order shuffled (`tetbox3` vs `tetbox3_perm`) already differs by **3.0e-6 on main**.
- The re-orientation therefore moves trajectories by the same amount as relabelling the input, which is physics-neutral. What inside the step makes the relabel noise was not isolated.
- "Become MuJoCo's" is out of reach for dim-3 dynamics: MuJoCo's tet elasticity (`flex_stiffness`, `user_mesh.cc:4347-4361`, `ComputeStiffness` `:3634-3660`) has no counterpart in sim-core (§7 H6). After the change the **arrays** match MuJoCo's (6/6 cases, `FC/ours_tet_fix2.jsonl`).

### 1.5 Tests to add (each fails on main + M3; values measured)
1. `flex_tet_reoriented_as_mujoco` (`sim/L0/tests/integration/flex_unified.rs`). Doc `tet_pos`.
   - Assert `flexelem_data == [0,2,1,3]` and `flexedge_vert == [[0,2],[1,2],[0,1],[1,3],[0,3],[2,3]]`, MuJoCo 3.5.0's values (`FC/mj_tet.jsonl`).
   - Main+M3: `[0,1,2,3]` and `[[0,1],[1,2],[0,2],[2,3],[0,3],[1,3]]`.
2. `flex_tets_all_outward_after_build`. Doc `tetbox3_perm` (40 tets, generator `FC/scripts/gen_tetbox.py`).
   - Assert every element has `dot(cross(v01,v02),v03) <= 0`, and the edges are the first-appearance order of the stored elements.
   - Main+M3: 18 of 40 positive.
3. (Q1 = unsigned) `flexelem_volume0_is_unsigned`. Doc `tet_neg` → `+1/6`. Main: `-1/6` (measured).

### 1.6 Flips
- Corpus (harness, `FC/h_V0` vs `FC/h_V1`): 1477 unchanged, **1 model-only**: `6a1c03dabed649d2` = `flex_unified.rs:62`. Its trajectory fingerprint is unchanged.
- CI tests (full `integration` binary, 1338 tests, `FC/it_fix_V1.log`): **1 flip**, `flex_unified::ac3_solid_compression` (`:409` asserts `rest_volume > 0`). With `FC_VOLABS`, 0 flips (`it_fix_V1abs.log`).
- sim-mjcf lib (385 tests): 0 flips.

### 1.7 Downstream
- `Model::flexelem_volume0` (pub): read by nothing outside the builder and `flex_unified.rs:409` (git grep).
- No signature changes.

---

## 2. Item 2 — flex flaps, and the `elastic2d` gate they hang on

### 2.1 Now
- `extract_flex_edges` pushes `[-1, -1]` for every edge (`flex.rs:442`).
- `extract_flex_bending_hinges` (`:459-545`) fills `[opp_a, opp_b]` for every edge shared by exactly 2 elements, **in every dim**: dim-3 edges shared by 2 tets get flaps and hinges, e.g. 24 hinges on `tetbox3`, measured.
- Boundary edges keep `[-1, -1]`.
- Cotangent coefficients are computed for every dim-2 flex with `young > 0 && thickness > 0` (`:525`).
- `elastic2d` is not parsed anywhere (git grep: 0 hits in `*.rs`). The Phase-10 spec recorded both choices as deliberate (`sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B.md:51`, `:360`). `elastic2d` is roadmap row DT-86 (`sim/docs/todo/POST_V1_ROADMAP.md:92`).

### 2.2 MuJoCo 3.5.0
- Flaps exist only for dim 2 (`user_mesh.cc:4326-4329` `CreateFlapStencil`). Elements are walked in order with local edges `(1,2) (2,0) (0,1)` (`:3664`).
- The first triangle holding an edge sets `[v0, v1, opp, -1]` (`:3697-3699`); the next sets slot 3 (`:3705`). A third would **overwrite** it.
- `flex_edgeflap` is stored only when `dim == 2 && elastic2d ∈ {1 bend, 3 both}`, else `[-1,-1]` (`user_model.cc:3504-3510`). `elastic2d` defaults to 0 = none (`mjs_defaultFlex` memset, `user_init.c:211-233`; meaning `mjspec.h:457`; keywords `xml_native_reader.cc:924-929`, read at `:1541` / `:2785`).
- Bending coefficients exist only for `dim == 2 && elastic2d ∈ {1,3} && young > 0` (`user_mesh.cc:4363-4374`). `thickness < 0` there is an error (`:4365-4367`). Boundary edges stay 0 (`:3744`).
- A vertex body without 3 joints plus bending is an error: "pins are not supported for bending", with MuJoCo's own `TODO(quaglino)` (`:4083-4086`).
- The engine skips `flap[1] == -1` (`engine_passive.c:213-217`).
- So by default MuJoCo applies **no bending** to a dim-2 flex; we always do.

### 2.3 What reads flaps and bending in sim-core
- `forward/passive.rs:555-660`: cotangent, `dim == 2` only (`:558`); it skips `flap[1] == -1` (`:573-575`), so `flap[0]` of a boundary edge is never read.
- Bridson, our extension: `apply_bridson_bending` (`:747`) reads `flexhinge_*`, dim 2 only (`:558`).
- Nothing else (git grep `flexedge_flap|flex_bending\b|flexhinge_`: `passive.rs` and the tests only).

### 2.4 Target and change
**2a — representation (parity, no behaviour change).**
- `extract_flex_bending_hinges` becomes `extract_flex_flaps` and follows `CreateFlapStencil`:
  - dim 2 only;
  - elements in order, local edges `(1,2),(2,0),(0,1)` (the `local_edges(3)` table M3 adds);
  - first triangle → `[opp, -1]`, second → slot 1.
- Hinges (ours) are built from the stored interior flaps, in edge order; none for dim 1 or 3.
- Coefficients are computed exactly as today, in **our** order (Q2).
- `Model` doc comments change: `flexedge_flap` (`model.rs:466-473`, "[opp,-1] on a boundary edge; [-1,-1] when dim ≠ 2 or bending is off") and `flex_bending` (`:474-479`).

**2b — the `elastic2d` gate (parity, plus the deviations of §6).**
```rust
// sim/L0/mjcf/src/types.rs
/// MuJoCo `elastic2d` (`<flex><elasticity>`), xml_native_reader.cc:924-929; default None
/// (mjs_defaultFlex, user_init.c:211). Only `Bend` is implemented.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[non_exhaustive]
pub enum FlexElastic2d { #[default] None, Bend, Stretch, Both }

pub struct MjcfFlex { /* … */ pub elastic2d: FlexElastic2d, /* … */ }
```
- **Parser** (`parse_flex_elasticity_attrs`, `parser/deformable.rs:348`, flex and flexcomp): exact keywords `none|bend|stretch|both`, through A4's keyword reader (M6/M8).
- **Builder** — bending is on iff `dim == 2 && elastic2d ∈ {Bend, Both}`. When it is on:
  - store the flaps;
  - compute coefficients if `young > 0`;
  - build the hinges.

  When it is off: all flaps `[-1,-1]`, coefficients 0, no hinges.
- **New refusals** (`MjcfError` per A3 §1):
  - `elastic2d="stretch"|"both"` → `Unsupported` (stated limitation: 2D membrane elasticity, MuJoCo's `Stencil2D` `user_mesh.cc:4352-4355`, is not implemented);
  - `Bend` + `young > 0` + `thickness < 0` → `InvalidValue`, MuJoCo's message "thickness must be positive for bending stiffness" (`:4366`);
  - `bending_model` set without `Bend` → `InvalidValue` (Q7);
  - non-manifold edge with `Bend` → `InvalidModel` (Q4);
  - pins with `Bend` → **load** (Q3).
- **Doc rewrites, same commit:** add `elastic2d="bend"` to every in-tree dim-2 flex whose `<elasticity>` has `thickness > 0`, i.e. exactly the flexes that have bending today (`FC/scripts/add_e2d_rs.py`):
  - `sim/L0/tests/integration/flex_unified.rs` ×36, `deformable_friction_dt25.rs` ×2, `runtime_flags.rs` ×1 (31 unique corpus docs);
  - inline literals and `format!` templates alike, e.g. `flex_unified.rs:2625,2945,3192,3248,3299,3401`.
  - Flexes without thickness are **not** rewritten: with `bend` added, MuJoCo refuses 12 of them for thickness (measured), and today they have no bending anyway.

### 2.5 Measured
- **Flap arrays vs MuJoCo 3.5.0, after 2a+2b+1 and the rewrites** (80 corpus flex docs converted for MuJoCo as A6 did, `FC/scripts/cmp_flex.py`):
  - In every doc both load (68), every flex's elems, edges and flaps equal MuJoCo's. That covers dim 1, dim 2 with and without `elastic2d`, and the dim-3 tet; boundary flaps are `[opp,-1]` exactly as MuJoCo.
  - Before: 241/241 boundary flaps differed (A6 §1.4).
- **Bending coefficients vs MuJoCo** (`FC/scripts/cmp_bend.py`):
  - 52 interior edges whose first triangle holds the edge as (min,max): equal to ≤ 1.73e-16 relative.
  - 6 interior (max,min) edges: equal to 0, because each has a symmetric diamond (c₀ = c₁).
  - So no corpus edge exposes Q2's defect. Boundary edges: 0 = 0.
- **The MuJoCo defect** (`FC/scripts/mj_bend_rest.py`, `mj_bend_force.py`):
  - MuJoCo computes `flex_bending` in the first triangle's edge orientation (`user_mesh.cc:3697-3699`, `:4371`) but applies it in sorted `flex_edge` order (`engine_passive.c:213`).
  - Case: a **flat** irregular diamond at its rest shape, gravity off, `elastic2d="bend"`, elements `0 2 1  2 3 1`:
    - MuJoCo `max|qfrc_spring| = 0.1145`; ours `5.6e-17`.
    - Same mesh as `0 1 2  1 3 2`: MuJoCo 6.2e-17, ours 5.6e-17.
    - Deflected (qpos 11 = 0.1, qpos 2 = −0.05): ours = MuJoCo to 6.1e-17 in the (min,max) case; differs by 0.114 in the (max,min) case.
  - A correct bending energy is stationary at its flat rest shape, so a nonzero force there is wrong. Ours computes and applies in the same order.
- **Pins + bending** (`FC/cases/bend_{pin,nopin}.xml`):
  - Deflected free-vertex forces with vertex 0 pinned equal the unpinned mesh's forces on those vertices exactly (max |Δ| = 0.0; max |f| 0.024).
  - MuJoCo refuses: "pins are not supported for bending".
  - Exposure: 6 corpus docs (7 sources: `flex_unified.rs:134,152,1091,1142,3079,3108,3350`).
- **Non-manifold edge** (`FC/cases/nonmanifold.xml`, 3 triangles on one edge): MuJoCo stores flap `[2,4]`, i.e. the first and **last** triangle, silently dropping the middle one. Exposure: 0 non-manifold edges in 80 corpus flex docs (the checker was shown to fire on the synthetic case).

### 2.6 Tests to add
Fails on main unless marked "pin".

1. `flex_boundary_flaps_as_mujoco`: square `elements="0 1 2  1 3 2"` + `elastic2d="bend"` → flaps `[[0,3],[1,-1],[2,-1],[1,-1],[2,-1]]` (MuJoCo, measured). Main: `[[0,3],[-1,-1]×4]`.
2. `flex_no_bending_without_elastic2d`: same mesh without `elastic2d` → all flaps `[-1,-1]`, `flex_bending` all 0, `nflexhinge == 0`. Main: interior flap `[0,3]`, nonzero B, 1 hinge.
3. `flex_dim3_has_no_flaps_or_hinges`: `tetbox3` → 0 hinges, all `[-1,-1]`. Main: 24 hinges.
4. `elastic2d_stretch_and_both_refused` (limitation). Main: both load.
5. `elastic2d_bend_requires_thickness`: `young="1e4"`, no thickness → error with MuJoCo's message. Main: loads.
6. `bending_model_requires_elastic2d` (Q7). Main: loads.
7. `flex_nonmanifold_edge_with_bending_refused` (Q4). Main: loads, edge treated as boundary.
8. *pin* `flat_mesh_has_no_bending_force_at_rest`: irregular (max,min) diamond, `max|qfrc_spring| < 1e-12` after `forward` (ours 5.6e-17; MuJoCo 0.1145). Passes on main; it is the proof for the Q2 deviation.
9. *pin* `pinned_vertex_bending_equals_unpinned_on_free_vertices` (Q3). Passes on main (Δ = 0.0).

### 2.7 Flips (measured)
- **2a** (`h_V0`→`h_V2`): 46 docs model-only, 0 trajectory, 0 status. CI: 0 of 1338 integration and 0 of 385 sim-mjcf lib tests flip.
- **2b with the rewrites** (`h_V2`→`h_V3_e2d`): 2 docs model-only (`builder/build.rs:1447,1489`: self-closing flexcomps with no elasticity, so their flaps empty out); 0 trajectory. CI: 0 flips.
- **2b without the rewrites** (`h_V3`): 21 docs change trajectory and 25 model only. 12 CI tests flip: `flex_unified::{ac6_bending_stiffness, ac11_flexcomp_expansion, ac19_bending_damping_only, ac20_bending_stability_clamp, spec_a_t4_simulation_regression_bit_identical, spec_a_t10_multi_flex_model, specb_t1, specb_t3, specb_t4, specb_t7_bridson_regression, specb_t10, specb_t12}` (`FC/it_fix_V3.log`).
- All of items 1+2 on top of the rewrites (`FC/it_fix_S123.log`, `lib_fix_L123.log`): 1338/1338 and 385/385 pass.
- Expected-change list: `FC/expected_change.tsv` (rows F1, F2, F3).

### 2.8 Downstream
- `MjcfFlex` gains a field. It is not `#[non_exhaustive]`; in-tree it is built only through `Default` (git grep `MjcfFlex {`), so no call site breaks.
- `.mjb` serialises `MjcfModel` with bincode (`mjb.rs:175-176`, `MJB_VERSION = 3` at `:52`), so the version bumps, shared with the other areas' `MjcfModel` changes.
- `Model` field types are unchanged; only `flexedge_flap` contents change.

---

## 3. Item 3 — 2-body cables MuJoCo refuses

### 3.1 MuJoCo's failure is its own
- `AddCableBody` (`user_composite.cc:317-445`) sets `lastidx = count[0]-2` (`:324`). For 2 bodies, ix 0 is both `first` and `secondlast`.
- The `first` branch wins and sets `next_body = B_{ix+1}` = `B_1` (`:351-353`). Body ix 1 is named `B_last` (`:356-357`).
- The exclude is added with `B_1` (`:428-433`), and compile fails: "body 'B_1' not found in bodypair 0" (`user_objects.cc:5861`).
- 3.4.0 has the same code (`:353`, `:361-363`) and the same refusal (measured).
- Exposure: 8 corpus docs, all with prefixes as written (`B_1`, `RB_1`, `AB_1`):
  - `composite.rs:199,226,301,333,361,423`, `parser/tests.rs:2143,2168`.
  - Ours refuses one of them (`composite.rs:423`) for `initial="xyz"`; that refusal is §6's stricter row.

### 3.2 The reference
- `FC/scripts/mj_model.py`, route `explicit`:
  - `MjSpec.from_string` runs MuJoCo's own composite expander. The one bad exclude name is renamed `<p>B_1 → <p>B_last`.
  - `spec.to_xml()` then writes the bodies, joints, geoms, sites and the exclude out explicitly. The XML writer's precision is raised to 17 through the exported `_mjPRIVATE__set_xml_precision`; at the default 6 digits the reference was off by 3.6e-6.
  - `from_xml_string` loads that explicit model.
- Checks on the reference:
  - The explicit model equals `spec.compile()` to ≤ 2.1e-17 (7 docs bit-equal, 1 at 2.1e-17).
  - 3.4.0 loads it and agrees with 3.5.0 to ≤ 2.8e-16 at step 500.

### 3.3 Measured
`FC/comp_mj_traj.jsonl`, `FC/comp_ours_traj.jsonl`, `scripts/cmp_cable.py`, `cmp_traj_mj.py`, `cmp_explicit.py`.
- Names, parents, joint types and bodies, geom types and bodies, sites and excludes all equal (7/7).
- Numeric fields (with item 4's f32 vertices):
  - `body_pos` exactly equal;
  - quaternions ≤ 2.8e-17;
  - masses ≤ 5.6e-17;
  - full inertia tensors ≤ 6.9e-18;
  - geom size and pos ≤ 3.5e-17.
- Trajectories, 500 steps, dt 0.002:
  - ours vs MuJoCo's explicit model: max |Δqpos| ≤ 4.25e-15, |Δqvel| ≤ 1.09e-13 (qvel up to 10.7);
  - ours(composite) vs ours(explicit XML): ≤ 1.8e-15.
- All of this already holds **on main**: the uservert doc `1cb45321…` against the golden at step 500 gives 2.5e-15 qpos with the base probe.

### 3.4 Change
- **None to the cable code.** Ours already loads these, and its exclude naming is right (`builder/composite.rs:349-357`).
- Correct the false comment at `builder/composite.rs:390-393`: 3.5.0 has a single `if (last || first)`, `user_composite.cc:436`, not "two separate `if` checks".
- Divergences row (§6).

### 3.5 Test to add (*pin*: passes on main; it is the "test shows our result is right")
- `two_body_cable_matches_mujoco_generator` (`sim/L0/tests/integration/composite.rs`).
- Inputs:
  - doc `composite.rs:361` (uservert `0 0 0 1 0 0 2 0 1`, so the segments bend);
  - fixture = MuJoCo's explicit model (`FC/golden/two_body_cable_explicit.xml`, `<custom>` stripped);
  - golden = MuJoCo 3.5.0 qpos/qvel at steps 1, 10, 100, 500 (`FC/golden/two_body_cable_golden_350.json`). qpos@500 = `[0.44481391341378135, -2.3365384483202473e-17, 1.1699910876516626e-16, -0.8956230135684972, 0.8834962039277522, -6.988600740279299e-17, 7.471241714046432e-17, -0.4684383178661332]`.
- Assert:
  - structure equal by name to the fixture, incl. `contact_excludes = {(B_first, B_last)}`;
  - |qpos − golden| ≤ 1e-12 and |qvel − golden| ≤ 1e-10 at each step (measured 2.5e-15 and 2.1e-14).

### 3.6 Found on the way — 1-body cables (`count="2"` or 2 vertices)
- **MuJoCo's defect.**
  - `double … tnext[3]` is uninitialised (`user_composite.cc:330`), only set `if (!last)` (`:340`), and read by `mjuu_updateFrame` on the first edge (`user_util.cc:620-634`, `cross(tangent, tnxt)`).
  - With one body, first == last, so the frame comes from uninitialised memory.
  - Measured: 3.5.0 `body_quat = NaN` for 4/4 directions, and "Nan, Inf or huge value in QACC" from step 0. 3.4.0 gives a finite `[0.707,0,0,0.707]` for an edge along +y.
  - MuJoCo also gives its only site the name `S_first` and puts it at `pos = length`, the far end (`:351-355` + `:436-441`).
- **Ours.** `tnext = 0` (`builder/composite.rs:327`).
  - Measured over 300 directions (6 axes + 294 random, `FC/onebody64.jsonl`): body x-axis = edge direction to ≤ 6.1e-16, |q| − 1 ≤ 2.2e-16, rotation orthonormal to ≤ 1.6e-15, 200-step trajectories finite.
  - Ours has two sites: `S_first` at 0 and `S_last` at length (pinned by `composite.rs:252` t7).
- Recommendation: lenient, listed. Test *pin* `one_body_cable_frame_is_defined`: the 6 axis directions plus 2 oblique ones (f32-exact coordinates) → x-axis = edge to 1e-12, |q| = 1, 500 steps finite. Q6 is the site.

---

## 4. Item 4 — the composite `curve` keywords

### 4.1 Confirmed
- **Ours** (`parser/composite.rs:287-308`): `s`→Sin, `c`→Cos, `l`→Line, `0|zero`→Zero, tokens past 3 dropped.
- **MuJoCo 3.5.0:** `shape_map` (`xml_native_reader.cc:851-856`) is `s`→LINE, `cos(s)`→COS, `sin(s)`→SIN, `0`→ZERO. More than 3 tokens is "maximum of 3 components" (`:2566`); an unknown token is "invalid shape" (`:2570`). Defaults: ZERO ×3 and `size (1,0,0)` (`user_composite.cc:66-67`). Same map at 3.4.0 (A3).
- Measured on 3.5.0:
  - `l 0 0` and `S 0 0` → invalid shape;
  - `s 0 0 0` → max 3;
  - ` s 0 0`, `s 0`, `s` and `s  0 0` load;
  - **`s 0 0 ` (one trailing space) is refused, "maximum of 3 components"**: the read loop runs past EOF and re-reads the last token as a fourth component (`:2558-2575`). Q8.

### 4.2 Change
- The map itself is **M8** (parser area). My prototype (`FC_CURVE`) reproduces MuJoCo's results on all 10 keyword cases above except the trailing space (we accept; Q8).
- Doc rewrites (must land with M8; `FC/scripts/rewrite_curve.py`; rule `l→s`, and `l 0 s → s 0 sin(s)` to keep geometry):
  - `composite.rs` ×14, `implicit_integration.rs` ×1, `parser/tests.rs` ×2;
  - examples `composites/stress-test` ×5 literals **plus the template arguments at `stress-test/src/main.rs:69,86,114,245` (`"l 0 0"`) and `:282` (`"s c 0"` → `"sin(s) cos(s) 0"`)**;
  - `hanging-cable` ×3, `cable-catenary` ×2, `cable-loaded` ×1;
  - READMEs `examples/fundamentals/sim-cpu/composites/README.md:25-30` and `cable-catenary/README.md:24`.
- The corpus extractor cannot see the `:282` template argument. Without that rewrite, check 7 of the stress-test **validator** loads `"s c 0"`, which MuJoCo's map refuses, and validate-examples goes red.
- **C1 (mine): cable vertices pass through f32.**
  - MuJoCo stores them as `std::vector<float> uservert` (`user_composite.h:81`). They are read as float (`xml_native_reader.cc:2552`) and generated vertices are pushed into it (`user_composite.cc:285-286`).
  - Change: in `builder/composite.rs:175` (`generate_vertices`), apply `f64::from(x as f32)` to both the user and the generated vertices. No type change. Q5.

### 4.3 Measured — our generated cable vs MuJoCo's, after the fix
Data: `FC/comp_*`, `scripts/cmp_cable.py`, by name, composite-generated elements only.
- **Corpus:** 24 docs contain `<composite>`.
  - With the rewrites MuJoCo loads 11 directly, plus 8 through §3's reference, and refuses 5 by attribute rules. Ours refuses the same 5, with different message text: "Cable must be one-dimensional" ×2, "Positive spacing…", "geom type…", "vertex or count".
  - Ours also refuses `composite.rs:423` (`initial="xyz"`, §6).
  - That leaves **17 docs both load and simulate**, plus `composite.rs:253` (count = 2), which is §3.6.
- **Structure:** body/joint/geom/site names, parents, joint types and bodies, geom types, excludes: equal in 17/17.
- **Numbers with f32 vertices:**
  - `body_pos` max 0;
  - `body_quat` ≤ 6.9e-16;
  - `body_mass` ≤ 5.6e-17;
  - full inertia tensor ≤ 6.9e-18;
  - `body_ipos`, `geom_size`, `geom_pos`, `site_pos` ≤ 6.9e-17;
  - joint damping, stiffness and armature exactly equal.
- **Numbers without f32 vertices:** `body_quat` up to 8.6e-8, `body_pos` up to 4.3e-8, masses up to 1.25e-8 (re-measured against the 17-digit reference).
- **Trajectories, 500 steps:**
  - 14 docs ≤ 4.25e-15 qpos.
  - 3 docs with an `<equality><connect>` (`cable-loaded:43`, `cable-catenary:58`, `implicit_integration.rs:1202`) differ by 8.9e-9 … 4.8e-5. Removing only the `<equality>` block brings all 3 to ≤ 2.3e-15, and disabling contacts does not (H4).
  - With f64 vertices, 6 docs without a `<connect>` drift by 5.4e-9 … 1.5e-8.
- **Golden must-fail for M8 + C1** (`FC/golden/sine_cable*.{xml,json}`):
  - Doc: `count="6 1 1" curve="s 0 sin(s)" size="1 0.2 1"`.
  - MuJoCo 3.5.0 `body_pos[B_1] = (0.23199064833328079, 0, 0)` and `body_quat[B_first] = (0.6822946080732275, -0.6822946080732275, -0.1856719359359429, -0.1856719359359429)`.
  - Ours with M8+C1: pos Δ 0, quat Δ 2.2e-16, mass Δ 1.4e-17. Ours with M8 only: pos Δ 1.8e-8. Main: refuses `sin(s)`.
- Fields that still differ **in representation only**, and are owned elsewhere (§7):
  - `geom_quat`: the geom z-axis is equal up to sign on every capsule (4.4e-16) — fromto sign, H1;
  - `body_inertia`/`body_iquat`: axis order — H2;
  - `jnt_range` of an unlimited ball or free joint — H3.

### 4.4 Tests to add
- **M8's** (fails on main): `cable_curve_keywords_as_mujoco`:
  - the sine golden above;
  - `l 0 0` / `S 0 0` / `zero 0 0` → "invalid shape";
  - `s 0 0 0` → "maximum of 3 components";
  - `cos(s) sin(s) 0` loads.
- **C1** (fails on main + M8): `cable_vertices_round_through_f32`: `count="4 1 1" curve="s 0 0" size="1"` → `body_pos[B_1].x == f64::from((1.0_f64/3.0) as f32)` (0.3333333432674408); main gives 0.3333333333333333. Plus the sine golden to 1e-15.

### 4.5 Flips (measured, `FC/h_curve*.jsonl`, `it_fix_I4/I5`, `t_L4b`, stress-test)
- M8 with the rewrites: 1 doc model+trajectory, `composite.rs:25`. Its `s 0 0` changes from sine (zero-length segments, A3) to MuJoCo's line; its asserts are structural (`:37-75`) and it passes.
- Without the rewrites: 9 integration tests (`composite::{t2,t3,t5,t6,t7,t9,t10,t12}`, `implicit_integration::test_implicitfast_connect_ball_chain_stability`) and 2 lib tests (`parser::tests::test_parse_composite_in_{body,worldbody}`) flip.
- C1: 10 docs model+trajectory (`FC/expected_change.tsv`, rows C1); 0 test flips.
- **Validator `example-composite-stress-test`:** stdout **byte-identical** between main (old keywords) and fix (rewritten keywords, MuJoCo map), with and without f32. rc 0, 12 PASS lines.
- The Bevy demos `hanging-cable`, `cable-catenary` and `cable-loaded` (no `example_kind`) were not run.

---

## 5. Commit list (each carries its tests, rewrites and expected-change rows) and dependencies

| # | commit | contents | depends |
|---|---|---|---|
| F1 | `fix(sim-mjcf): dim-3 flex elements oriented as MuJoCo` | `orient_tetrahedra`; `flexelem_volume0` per Q1 + docs; tests §1.5 | M3 (edge order) |
| F2 | `fix(sim-mjcf): flex flaps as MuJoCo — dim 2 only, boundary [opp, -1]` | `extract_flex_flaps`; hinges from flaps; Model doc comments; tests §2.6 #1 (with `elastic2d` written but ignored — passes after F2), #3 | M3 |
| F3 | `feat(sim-mjcf): flex elastic2d — bending only when asked, as MuJoCo` | `FlexElastic2d`; parse; gate; refusals; 39 doc rewrites; tests §2.6 #2, #4–#9; DT-86 rows (`POST_V1_ROADMAP.md:92`, `future_work_10i.md:26`) | F2; M1 (variants); M6/M8 (keyword reader); **before M15** (else the schema pre-pass first refuses `elastic2d` as unimplemented); interacts with M14 (A4-Q1 rewrites the same 81 docs to MuJoCo's flex form — whichever lands second rebases the other's rewrites) |
| (M8) | parser area | curve map + §4.2 rewrites + test §4.4 M8 | M6 |
| C1 | `fix(sim-mjcf): cable vertices pass through f32 as MuJoCo` | §4.2 C1; test §4.4 C1; comment fix `composite.rs:390-393`; tests §3.5 and §3.6 (pins) | M8 |
| (M26) | docs | §6 rows into `MUJOCO_CONFORMANCE.md:321` | all |

Other dependencies:
- A3's resolve pass (M11, composite elements take MuJoCo's built-in defaults, c01/c02) leaves the cable comparison unaffected: 0 of 24 composite docs use defaults, childclass or frames (A3 §4).
- A3's `checksize` (M13) must follow M8 (A3 §3).
- Bit-equal cable model arrays also need H1–H3 from other areas; the dynamics do not.

---

## 6. Divergence rows proposed for `MUJOCO_CONFORMANCE.md:321`, and the open questions

| kind | row | test |
|---|---|---|
| lenient | 2-body cable: MuJoCo names the exclude's 2nd body `B_1` (`user_composite.cc:351-353`), which does not exist → refuses; we load | §3.5 |
| lenient | 1-body cable: MuJoCo builds its frame from uninitialised memory (`:330`, `:340`, `user_util.cc:620-634`; NaN at 3.5.0); we build a defined frame and two sites | §3.6 |
| correctness | cotangent bending coefficients: MuJoCo computes them in the first triangle's edge order and applies them in sorted order (`user_mesh.cc:3697-3699`, `:4371`; `engine_passive.c:213`), giving a nonzero force on a flat rest mesh; we compute and apply in one order | §2.6 #8 |
| lenient (Q3) | pins + bending: MuJoCo refuses ("pins are not supported for bending", `:4083-4086`, its TODO); we load | §2.6 #9 |
| stricter (Q4) | non-manifold edge + bending: MuJoCo silently pairs the first and last triangle (`:3705`); we refuse | §2.6 #7 |
| limitation | `elastic2d="stretch"`/`"both"` (2D membrane, `:4352-4355`) refused | §2.6 #4 |
| stricter | composite `initial` other than `ball\|none\|free`: MuJoCo treats it as ball (`user_composite.cc:418-421`); we refuse (exists today, `builder/composite.rs:128`) | `composite.rs:423` |
| lenient (Q8) | `curve` with trailing whitespace: MuJoCo's read loop refuses it (`xml_native_reader.cc:2558-2575`); we accept | new: `"s 0 0 "` builds as `"s 0 0"` |
| extension | `bending_model` (Bridson) — allowlisted (A4); now requires `elastic2d` bend/both | §2.6 #6 |

Open questions (only ones the fixed decisions do not settle):

- **Q1. `flexelem_volume0`** after re-orientation. Options: (a) unsigned |V|; (b) delete the pub field (no sim-core reader; MuJoCo has none); (c) signed (now always ≤ 0).
  - **Recommend (a)**: smallest diff; 0 test flips (measured); the field means "rest volume" independent of input order.
  - If (b) is wanted: one pub field goes, and `flex_unified.rs:409-410` loses one assertion.
- **Q2. Bending-coefficient order.** Options: inherit MuJoCo's order (bit-equal `flex_bending`, nonzero rest force) or keep ours (correct). The rule's "silently does something other than asked → refuse" does not fit: refusing would refuse valid meshes we compute correctly.
  - **Recommend keep ours, listed as a correctness deviation**, with test #8. The 0.1145 force is measured on a synthetic case only; 0 corpus edges are affected.
  - If wrong: our `flex_bending` differs from MuJoCo's on (max,min) edges of asymmetric diamonds.
- **Q3. Pins + bending.** Options: refuse (parity; flips 6 corpus docs / 7 sources in `flex_unified.rs` to errors) or load (lenient).
  - **Recommend load**: our forces are shown right (Δ = 0.0 vs the unpinned mesh); MuJoCo refuses for lack of support, not for an error in the file.
  - If wrong: those 7 test docs need unpinned rewrites.
- **Q4. Non-manifold edge + bending.** Options: MuJoCo's first+last pairing (parity) or refuse.
  - **Recommend refuse** (MuJoCo silently ignores the middle triangle).
  - 0 corpus docs either way.
- **Q5. f32 cable vertices.** Options: parity or keep f64.
  - **Recommend parity**: model arrays agree with MuJoCo to ≤ 6.9e-16 instead of ≤ 8.6e-8, and trajectories to ≤ 4.25e-15 instead of ≤ 1.5e-8.
  - Cost: 10 docs change (C1 rows), and input coordinates lose ~6e-8 relative precision.
- **Q6. 1-body cable sites.** Options: ours (S_first at 0 + S_last at length, pinned by t7) or MuJoCo's single `S_first` at the far end.
  - **Recommend ours**: MuJoCo's 1-body output is undefined anyway (§3.6).
- **Q7. `bending_model` without `elastic2d`.** Options: refuse; `bridson` implies bend; or silently no bending.
  - **Recommend refuse** (nothing silent; a 1-attribute fix for the user).
  - Exposure after F3's rewrites: 0 in-tree (every Bridson doc has thickness > 0, so it gets `bend`). Measured: all tests pass.
- **Q8. Trailing whitespace in `curve`.**
  - **Recommend accept (lenient)**. This interacts with A4's "keywords exact" rule, so the parser owner should confirm.
  - 0 corpus uses.

---

## 7. Handed to other areas (found here, measured unless marked)

| id | finding | referent | owner |
|---|---|---|---|
| H1 | geom `fromto` orientation: MuJoCo aligns z with **from − to** (`user_objects.cc:3953`, `:3971`); ours with to − from (`builder/geom.rs:817-848`). Every fromto geom's `geom_quat` differs; capsule z-axes are equal up to sign (4.4e-16). Shapes are symmetric under the flip; xmat-derived outputs differ (not measured) | `cmp_cable.py` | mjcf-S8 (geometry attributes) |
| H2 | principal-axis order: all cable bodies have `body_inertia`/`body_iquat` in a different axis order; the full tensor agrees to ≤ 6.9e-18 | `cmp_cable.py` | A5 §4.4 / Jon's divergence list |
| H3 | stored range of an unlimited joint: ours ball `(−0.0548, 0.0548)` — the default `(−π, π)` passed through deg→rad (0.0548 = π²/180); free `(−π, π)`; MuJoCo `[0,0]` | `cmp_cable.py` | A5-Q3 |
| H4 | `<equality><connect>` with `implicitfast` + Newton diverges from MuJoCo by up to 4.8e-5 qpos in 500 steps on 3 cable docs; ≤ 2.3e-15 with the equality removed; cause not isolated | `FC/comp_noeq` | core / constraints |
| H5 | ours adds an edge-length constraint row for **every** flex edge (`equality_assembly.rs:76-135`); MuJoCo only via flex equality. By reading, not measured | reading | flex (later) |
| H6 | no dim-3 (`flex_stiffness`) or 2D membrane elasticity in sim-core (git grep `flex_young`), so dim-3 dynamics cannot match MuJoCo | git grep | flex (SVK, later) |
| H7 | `SPEC_B.md:246` says `elastic2d == 0` is "membrane only"; MuJoCo: 0 = none (`mjspec.h:457`; membrane only for ≥ 2, `user_mesh.cc:4352`) | reading | todo doc (not shipped) |
| H8 | per-vertex flex damping (`passive.rs:474-490`) — not compared with MuJoCo | not examined | flex |

---

## 8. What my methods cannot see
- **The MuJoCo-native flex form** (`<flex body= vertex= element=>`, A4-Q1) is not implemented.
  - All flex comparisons used our extension form, converted for MuJoCo as one world body with 3 slides per vertex.
  - Orientation with vertex offsets in **rotated** parent bodies is untested.
- **Runtime MJCF:** covered only through the CI tests and the one validator I ran.
  - No code outside `sim/L0` or `examples/…/composites` emits `<flex`, `<flexcomp` or `<composite` (git grep).
  - The 3 Bevy composite demos were not run.
- **Corpus exposure of Q2:** 0 edges. The defect is shown on synthetic meshes only.
- **MuJoCo's 1-body cable is UB:** "NaN at 3.5.0, finite at 3.4.0" describes these two wheels on this machine, nothing more.
- **Trajectory comparisons:** these used the harness's or my own initial states (`qvel` excitation, or rest). Contacts were active only where the docs have them.
- **Not run on the prototype:**
  - clippy, grade and `--features mjb`;
  - `cargo test -p sim-conformance-tests --release` (the integration tests ran in the test profile);
  - the licensed gates.
- **Prototype shape:** env-switched scratch code, not the PR's form.
- **Determinism of my new code:**
  - Flaps iterate elements and `local_edges` in order, with no hash iteration (prototype `FC/fix/sim/L0/mjcf/src/builder/flex.rs`, `extract_flex_flaps_mujoco`).
  - Harness ×2 on base gave 0 variation.
  - The fix runs were single passes per variant, not ×10.

## 9. Artifacts (`FC/`)
- **Inputs:**
  - `cases/` (tets, tet boxes, bending, pins, non-manifold, curve, 1-body);
  - `comp/` (24 rewritten composite docs + `*.explicit.xml` + `index.tsv`);
  - `corpus_e2d/`, `corpus_curve/` (rewritten corpora);
  - `golden/` (2-body fixture and 3.5.0 golden, sine-cable golden).
- **Results:**
  - `h_*.jsonl` (harness runs);
  - `it_fix_*.log`, `t_*.log`, `lib_fix_*.log` (test runs);
  - `ex_*.log` (validator);
  - `expected_change.tsv` (per-commit list).
- **Scripts:** `scripts/` (`patch_fix_flex.py` and `patch_fix_curve.py` are the prototype edits, each anchor asserted unique).
- `target/` directories are deleted (see below).

## Repo state
- Before: HEAD `ea11c8d6e3d996176b8dec6d965e05b203ae1ea7`, `git status --short` empty.
- After: HEAD `139f7b25d318b04e2f738e20590f898d82264bcb`, `git status --short` empty.
  - The new commit (`docs(sim): Rigid spec: P-L32 cause isolated and fix measured`) is a docs-only change to `RIGID_SPEC.md` (11+/3−), made by someone else during this run. I ran no git write command.
  - All measurements used the code at `ea11c8d6`, which is identical to `main`.
- Scratch `target/` dirs deleted (base, fix, probe_base, probe_fix; 4.4 GB). `FC/` is 221 MB of sources and results. Nothing of mine is still running.
