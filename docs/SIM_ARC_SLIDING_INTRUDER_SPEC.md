# cf-device-design Sim Arc — Sliding-Intruder FEM Spec

**Status**: SPEC 2026-05-18 LATER. Recon output following the pivot bookmarked at `docs/SIM_ARC_SLIDING_INTRUDER_BOOKMARK.md` (dev `1ac4fd12`). Drives the next-session implementation arc. Three-session pattern enters its final phase: bookmark `1ac4fd12` → recon (this spec) → implementation (next session, multi-week).

**Reading order**: §1 (problem recap + product intent) → §2 (D-Slide decisions + architectural Qs) → §3 (architectural shape + primitives + APIs) → §4 (sub-leaf ladder with per-leaf gates) → §5 (pre-implementation gates) → §6 (open risks) → §7 (cross-references).

---

## 1. Problem recap

The S11.2 visual gate on dev `fcc57835` revealed that the current FEM is **structurally wrong-fit for the workshop product story**, not just visually under-rendered. `insertion_sim::run_insertion_ramp` ramps a UNIFORM SDF offset of the cleaned scan from `cavity_inset_m / n_steps` to `cavity_inset_m` over `n_steps` Newton solves — the entire cavity wall sees the same offset growth every step. Intermediate steps are a numerical stability path to the press-fit, not physically meaningful per-position deformation.

User's actual ask (verbatim, 2026-05-18 LATER):

> "what we are simulating is the intruder at a constant scale/size going along the cap scan line. tip enters the mouth, goes to the end. that should be the sim."

> "the intruder is bigger than the cavity at its beginning state, it should still be able to put pressure on the walls without having to grow in scale as it progresses down the length of the cavity"

> "in its current state it doesnt really offer anything meaningful. this simulation is purely meant to simulate the insertable being inserted into the cavity from mouth to end" (after option C selection)

Translation: the simulation must show a **constant-geometry rigid intruder** sliding along the centerline from cap mouth (= centerline's floor end, the open rim) toward the closed end (= centerline's tip / dome apex), with per-position FEM-solved cavity deformation **localized to the contact zone where the intruder currently sits**. The user expects to see a deformation wave propagate along the cavity as they scrub the playback slider.

The cleaned scan IS the rigid intruder. The cavity (= `scan.offset(-cavity_inset_m)`) is smaller than the scan by `cavity_inset_m`, so when the intruder lands in the cavity it interferes by `cavity_inset_m` (3 mm default) locally in the contact band. Behind the intruder (already-passed cavity), walls relax back toward rest; ahead of the intruder (not-yet-engaged cavity), walls sit at rest.

---

## 2. Decisions

### 2a. D-Slide1..D-Slide8 resolved

| # | Decision | Rationale |
|---|----------|-----------|
| **D-Slide1** | **Contact model = TransformedSdf adapter** wrapping the existing `geometry.intruder: GridSdf`. Adapter takes a rigid transform `T` (translation only iter-1, `Isometry3<f64>` long-term) and delegates `eval(p) = inner.eval(T⁻¹ p)`, `grad(p) = R · inner.grad(T⁻¹ p)`. Plugs into `PenaltyRigidContact::with_params(vec![transformed], κ, d̂)` per slide step. | Bookmark option (a). Zero new SDF computation per step (vs option (b) "build moving GridSdf"); zero parry BVH crate dep (vs option (c)). Static grid covers the body bbox + outer_offset margin; inverse-transformed query points for sliding land WITHIN that coverage in all body-relevant configurations (intruder rest pose at `t=1`, intruder displaced into cap_outward half-space at `t=0` → BCC vertices land DEEPER inside the static grid, well within coverage). Inactive pairs (`sd > d̂`) at clamp boundaries are silently skipped by `PenaltyRigidContact::active_pairs`. Sdf trait is exactly `eval + grad` — one matrix multiply per call, no allocation. |
| **D-Slide2** | **Slide path = centerline polyline arc** (translation only iter-1; rotation banked as iter-2 follow-up). At slide fraction `t ∈ [0, 1]` the intruder's TIP (cleaned-scan-frame `centerline[0]`) translates to `point_along_polyline_at_arc_distance(centerline, L · (1 − t))` (i.e., walking backward from the tip toward the floor by arc-length `L·(1-t)`). Translation vector = `tip_pos(t) − centerline[0]`. The rest of the intruder mesh translates rigidly. | User explicitly said centerline ("along the cap scan line"). Centerline ordering convention (cf-scan-prep `trim_mesh_along_centerline` docs): index 0 = TIP (closed end / dome), index N-1 = FLOOR (open end / cap rim). At `t=1` (fully seated): tip at `centerline[0]` = rest pose. At `t=0` (start of slide): tip at `centerline.last()` = cap mouth, intruder body extends +cap_outward into the air outside the body. Translation-only iter-1 because iter-1 sock-over-capsule is mostly straight (rotation is a no-op to first order) and rotation introduces a coordinate-frame choice (intruder "up" vector) that needs more product input. The cf-scan-prep `point_along_polyline_at_arc_distance` helper lives at `tools/cf-scan-prep/src/main.rs:1016` — implement a parallel copy in cf-device-design (30 lines), don't introduce a tools→tools dep. |
| **D-Slide3** | **Ramp endpoints = inclusive seated**: `t_k = (k + 1) / n_steps` for `k ∈ 0..n_steps`. First step (`k=0`) is `1/n_steps` slid in; last step (`k=n_steps-1`) at `t = 1.0` is fully seated. Rest pose is implicit warm-start, not a recorded step. | Mirrors the current `run_insertion_ramp` parameterization (`insertion_sim.rs:1898`) so per-step record count semantics carry over unchanged. The first step has the intruder PARTLY in the cavity already — Newton convergence is easier than starting at `t = 0` exactly (which would have zero contact pairs and a degenerate-but-trivial solve). |
| **D-Slide4** | **Warm-start = same chained `x_final` pattern** as the current ramp. No "convection" of the warm start with the slide direction. Each Newton solve at step `k+1` starts from step `k`'s converged positions; vertices behind the intruder relax back toward rest under elastic forces alone (penalty inactive there because `sd > d̂`); vertices ahead stay at rest (already at rest); vertices in the new contact zone deform under the rebuilt contact's penalty. | The active-set discontinuity at `d = d̂` per `penalty.rs:50` may cause local Armijo stalls at step boundaries, but the existing `INSERTION_SOLVE_TOL = 1e-1` + `catch_unwind` flow at `insertion_sim.rs:1905` already absorbs that as graceful Fork-B "we got to step k of N" behavior. Convection-aware warm starts (offset behind-vertices in the slide direction, leave ahead-vertices alone) would help convergence but add ~50 LOC of mesh-side bookkeeping that's premature before we measure the convergence regression. Recon flag the convergence rate as a §6 open risk; iter-1 visual gate is the falsification trigger. |
| **D-Slide5** | **n_steps default = `max(16, ceil(L_m / 5e-3))`** via a new `DEFAULT_SLIDE_STEP_SIZE_M: f64 = 5e-3` const in cf-device-design. iter-1 sock-over-capsule L ≈ 83 mm → 17 steps; longer body parts (200 mm) → 40 steps. | Programmer-set, not user-tunable — same posture as `DEFAULT_N_STEPS = 16` for the growing-intruder ramp. 5 mm per step gives a deformation-wave that visibly propagates (vs 1 mm steps where the wave reads as continuous), still small enough for Newton convergence stability. Per-step solve cost is roughly equal to the growing-intruder solve (~1 s/step at iter-1), so a 17-step ramp finishes in ~17 s, a 40-step ramp in ~40 s. Both inside the user-acceptable wait envelope (current ramp is 10-20 s). |
| **D-Slide6** | **Stall handling = same Fork-B `catch_unwind` + `failed_at_step` + `failure_reason`** as the current ramp. Partial-seating ramp is honest engineering data. UI surfaces "slid X mm of Y mm" instead of the current "seated to X mm full depth". | Bookmark §5b D-Slide6 confirms. No solver-side change beyond the contact rebuild. |
| **D-Slide7** | **UI text update for sliding semantics**: convergence summary becomes `"slid X mm of Y mm arc length (step k of N)"`; F-d plot x-axis becomes `arc_length_m * 1000` (mm), label "Slide arc-length (mm)"; per-step table column "Interference (mm)" → "Slid (mm)"; per-step row interpretation = arc length, not press-fit depth; viewport step badge `"Step k/N — slid X mm of Y mm"`. | Same surfaces, different physical interpretation. The F-d curve character changes: instead of monotonically-rising press-fit force, sliding produces a "force vs insertion progress" curve that may rise then fall (contact zone passing a wide section) or be roughly flat (uniform cross-section). This IS the engineering data the workshop user wants. |
| **D-Slide8** | **Deformed cavity render = unchanged plumbing**. `cavity_boundary_faces` + `per_layer_outer_faces` are geometry-only (BCC face classification, FEM-model-independent); `deformed_layer_slab_mesh_at(layer_idx, step)` pairs them with per-step `x_final` to produce the slab render at any displayed step. The sliding ramp produces per-step `x_final` with the same shape as the growing-intruder ramp; the localized-deformation visual reads correctly **by construction** the moment the new solver outputs land. | This is the load-bearing implication of the S11.2 plumbing surviving the pivot (bookmark §4). No new render code; the user sees the deformation wave the moment `run_sliding_insertion_ramp` is plugged in behind `run_sim_pipeline`. |

### 2b. Architectural questions resolved

| # | Question | Decision | Rationale |
|---|----------|----------|-----------|
| **Q1** | Separate solver vs replace `run_insertion_ramp`? | **Separate `run_sliding_insertion_ramp` alongside; keep `run_insertion_ramp` intact**. Both reuse `build_insertion_geometry`, `outer_skin_bc`, `compute_tet_readouts`, `aggregate_step_readout`, `intruder_contact_at` becomes a sliding sibling `intruder_contact_sliding_at`. | User-blessed: the growing-intruder ramp is preserved as "future mode to prototype variations of the scanned size going through the cavity" (bookmark §6 banked). Separate function = zero risk of regressing the 13 existing growing-intruder tests in `insertion_sim.rs::tests`. Doubled maintenance is small (the two ramps share 90% of their body). Test stories stay clean: growing tests cover `run_insertion_ramp`, new sliding tests cover `run_sliding_insertion_ramp`. |
| **Q2** | Keep growing-intruder as Engineering Review mode behind a toggle? | **Yes — `SimMode { Sliding, GrowingIntruder }` enum threaded through `InsertionSimState` + a Sim panel toggle**. Defaults to `Sliding`. The growing-intruder option label = `"Growing intruder (variation prototyping)"`; a small `i` info-tooltip explains the use case in the user's own words. | Bookmark §5c question + §6 banked. Implementation can ship in two passes: pass A (this arc) gets sliding working with the toggle present but only `Sliding` mode wired; pass B adds the `GrowingIntruder` branch in `run_sim_pipeline`. Pass A is the unblocker; B is small. |
| **Q3** | Reuse BCC mesh + per-tet materials across both solvers? | **Yes, unchanged**. `build_insertion_geometry` returns the same `InsertionGeometry` for both; the intruder + bounds field is the static rigid scan. The sliding pose lives in the ramp's per-step state, not in the geometry. The per-tet `Yeoh` material partition is unchanged. | The geometry struct is contact-model-agnostic. The only divergence between ramps is the per-step contact primitive build (uniform offset vs transformed primitive) + the per-step pose tracking. Zero struct changes needed. |

---

## 3. Architectural shape

### 3a. Data-flow diagram

```
┌────────────────────────────────────────────────────────────────────────┐
│  insertion_sim.rs (FEM crate)                                          │
│  ───────────────────────────────────────────────────────────────────── │
│                                                                        │
│  build_insertion_geometry(scan, design, cap_planes, ...) → InsertionGeometry
│  (unchanged — shared by both ramps)                                    │
│                                                                        │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ NEW: TransformedSdf<S: Sdf> { inner: S, transform: Isometry3 }  │   │
│  │   impl Sdf:                                                     │   │
│  │     eval(p) = inner.eval(transform⁻¹ * p)                       │   │
│  │     grad(p) = transform.rotation * inner.grad(transform⁻¹ * p)  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                        │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ NEW: intruder_contact_sliding_at(                               │   │
│  │   intruder: &GridSdf,                                           │   │
│  │   bounds: Aabb,                                                 │   │
│  │   slide_pose: Isometry3<f64>,                                   │   │
│  │   cavity_offset_m: f64,                                         │   │
│  │ ) → PenaltyRigidContact                                         │   │
│  │   { wraps TransformedSdf(intruder.clone(), slide_pose),         │   │
│  │     then .offset(cavity_offset_m) — i.e. Solid::from_sdf(...).  │   │
│  │     offset(cavity_offset_m) where the SDF is the transformed    │   │
│  │     scan (NOT the offset-grown scan). }                         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                        │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ NEW: SlideRampStep { slide_fraction_t: f64, arc_length_s_m,     │   │
│  │   iter_count, final_residual_norm, x_final, readout }           │   │
│  │ NEW: SlideRamp { steps, failed_at_step, failure_reason,         │   │
│  │   final_x, n_pinned, result, intruder_poses: Vec<Isometry3> }   │   │
│  │                                                                 │   │
│  │ NEW: run_sliding_insertion_ramp(                                │   │
│  │   geometry: InsertionGeometry,                                  │   │
│  │   centerline_polyline_m: &[Point3<f64>],                        │   │
│  │   n_steps: usize,                                               │   │
│  │ ) → Result<SlideRamp>                                           │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│  insertion_sim_ui.rs (Bevy crate)                                      │
│  ───────────────────────────────────────────────────────────────────── │
│                                                                        │
│  enum SimMode { Sliding, GrowingIntruder } — added to InsertionSimState│
│                                                                        │
│  InsertionSimOutputs {                                                 │
│    ramp_kind: RampKind { Sliding(SlideRamp), Growing(InsertionRamp) }, │
│    per_layer, tet_centroids, per_tet_layer,                            │
│    per_step_scalar_fields, scalar_min_max,                             │
│    cavity_boundary_faces, per_layer_outer_faces,                       │
│    // REPLACES: per_step_intruder_meshes (delete)                      │
│    intruder_poses: Vec<Isometry3<f64>>,  // identity for Growing       │
│  }                                                                     │
│                                                                        │
│  run_sim_pipeline(...)                                                 │
│    match sim_mode {                                                    │
│      Sliding => run_sliding_insertion_ramp(geometry,                   │
│                    &centerline_polyline_m, n_steps),                   │
│      GrowingIntruder => run_insertion_ramp(geometry, n_steps),         │
│    }                                                                   │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│  main.rs (rendering)                                                   │
│  ───────────────────────────────────────────────────────────────────── │
│                                                                        │
│  spawn_intruder_mesh — back to S4 pattern: mount the cleaned scan as   │
│    a constant mesh asset at startup (decimated to render-tier, e.g.    │
│    ~30k faces via meshopt, only for the IntruderEntity). Hidden by    │
│    default.                                                            │
│                                                                        │
│  update_intruder_mesh → update_intruder_transform (renamed back to S4) │
│    Reads InsertionSimState.last_run.intruder_poses[displayed_step],    │
│    converts physics→bevy frame via up.to_bevy_normal + render_scale,   │
│    writes Transform.translation. Visibility = show_deformed && some.   │
│    IntruderMeshKey shrinks to (displayed_step, last_run_generation,    │
│    show_deformed) — same change-detection pattern; no mesh asset       │
│    rebuild.                                                            │
└────────────────────────────────────────────────────────────────────────┘
```

### 3b. TransformedSdf design

```rust
/// Adapter: present a static SDF as if it had been rigidly transformed in
/// world space. `eval(p)` returns the signed distance from `p` to the
/// transformed surface; `grad(p)` returns the gradient in world frame.
///
/// For a rigid transform T: p ↦ R p + t (rotation + translation):
///   T⁻¹ q = R^T (q − t)
///   sd_transformed(q) = sd_inner(T⁻¹ q)         (signed dist is rigid-invariant)
///   ∇sd_transformed(q) = R · ∇sd_inner(T⁻¹ q)   (gradient rotates with transform)
///
/// For translation-only (R = I), this collapses to a query-point shift +
/// pass-through gradient — no matrix multiplies in the hot path.
///
/// **Bounds caveat**: `inner` may be backed by a finite-extent `GridSdf`
/// with `distance_clamped` semantics outside its grid. After the inverse
/// transform, queries that fall outside the grid are silently clamped to
/// the nearest grid sample. For sliding-intruder use this is benign:
/// active contact pairs (`sd < d̂ = 1 mm`) only fire when the intruder
/// surface is within 1 mm of the body wall, which puts the inverse-
/// transformed query point WELL INSIDE the static intruder grid's body-
/// covering coverage. Pairs outside the grid coverage report large `sd`
/// values that fall outside `d̂` and are skipped by `active_pairs`.
pub struct TransformedSdf<S: Sdf> {
    inner: S,
    transform: Isometry3<f64>,
    // Cache T⁻¹ to avoid recomputing per query.
    inverse: Isometry3<f64>,
}

impl<S: Sdf> TransformedSdf<S> {
    pub fn new(inner: S, transform: Isometry3<f64>) -> Self { ... }
}

impl<S: Sdf> Sdf for TransformedSdf<S> {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.inner.eval(self.inverse * p)
    }
    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.transform.rotation * self.inner.grad(self.inverse * p)
    }
}
```

Lives in `insertion_sim.rs` (or factored into `cf-design::sdf` if it earns reuse beyond this arc; defer until a second consumer emerges).

### 3c. Slide pose computation

```rust
/// Compute the rigid transform for the intruder at slide fraction `t`.
/// t = 0 → tip at cap mouth (centerline.last()), body extends +cap_outward.
/// t = 1 → rest pose (identity transform).
///
/// Translation-only iter-1; full Isometry3 in scaffolding to absorb the
/// iter-2 rotation followup without API churn.
fn slide_pose_at(centerline: &[Point3<f64>], t: f64) -> Isometry3<f64> {
    let l = polyline_arc_length_m(centerline);
    let tip_rest = centerline[0];
    let walk_from_tip = l * (1.0 - t).clamp(0.0, 1.0);
    let (tip_world, _tangent) =
        point_along_polyline_at_arc_distance(centerline, walk_from_tip)
            .unwrap_or((tip_rest, Vector3::z()));  // degenerate-polyline safety
    let translation = tip_world - tip_rest;
    Isometry3::translation(translation.x, translation.y, translation.z)
}
```

Inputs come from the existing `centerline_polyline_m` already loaded from `.prep.toml` (resource `Centerline`, populated at startup; passed into the sliding ramp through the `kick_off_simulation` snapshot + the async task input — same plumbing as `cap_planes_clone`).

### 3d. SlideRamp loop sketch

```rust
pub fn run_sliding_insertion_ramp(
    geometry: InsertionGeometry,
    centerline_polyline_m: &[Point3<f64>],
    n_steps: usize,
) -> Result<SlideRamp> {
    if n_steps == 0 { return Err(...); }
    if centerline_polyline_m.len() < 2 {
        return Err(anyhow!("sliding ramp requires centerline polyline (≥ 2 points)"));
    }
    let InsertionGeometry { mesh, intruder, cavity_offset_m, outer_offset_m,
                            bounds, cell_size_m, n_tets, per_tet_layer: _ } = geometry;

    // Reuse outer_skin_bc unchanged — outer skin does not move with slide.
    let bc = outer_skin_bc(&mesh, &intruder, bounds, outer_offset_m, cell_size_m)?;
    // Snapshot per-tet immutables before mesh clones (same as run_insertion_ramp).
    let rest_positions: Vec<Vec3> = mesh.positions().to_vec();
    let tets: Vec<[VertexId; 4]> = (0..n_tets as TetId).map(|t| mesh.tet_vertices(t)).collect();
    let materials: Vec<Yeoh> = mesh.materials().to_vec();
    let referenced: Vec<VertexId> = referenced_vertices(&mesh);
    let l_m = polyline_arc_length_m(centerline_polyline_m);

    let config = insertion_solver_config();
    let mut x_prev_flat: Vec<f64> = rest_positions.iter()
        .flat_map(|p| [p.x, p.y, p.z]).collect();
    let n_dof = 3 * mesh.n_vertices();
    let v_prev = Tensor::zeros(&[n_dof]);
    let empty_theta: [f64; 0] = [];
    let theta = Tensor::from_slice(&empty_theta, &[0]);

    let mut steps: Vec<SlideRampStep> = Vec::with_capacity(n_steps);
    let mut intruder_poses: Vec<Isometry3<f64>> = Vec::with_capacity(n_steps);
    let mut failed_at_step = None;
    let mut failure_reason = None;
    for k in 0..n_steps {
        let t = (k + 1) as f64 / n_steps as f64;
        let pose = slide_pose_at(centerline_polyline_m, t);
        let contact = intruder_contact_sliding_at(&intruder, bounds, pose, cavity_offset_m);
        let solver = CpuNewtonSolver::new(Tet4, mesh.clone(), contact, config, bc.clone());
        let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            solver.replay_step(&x_prev, &v_prev, &theta, config.dt)
        }));
        match outcome {
            Ok(step) => {
                let positions_k: Vec<Vec3> = positions_from_flat(&step.x_final);
                let readout_contact = intruder_contact_sliding_at(
                    &intruder, bounds, pose, cavity_offset_m);
                let raw_readouts = readout_contact.per_pair_readout(&mesh, &positions_k);
                let contact_readouts =
                    filter_pair_readouts_to_referenced(raw_readouts, &referenced);
                let per_tet =
                    compute_tet_readouts(&rest_positions, &positions_k, &tets, &materials);
                let step_readout = aggregate_step_readout(&per_tet, &contact_readouts);
                steps.push(SlideRampStep {
                    slide_fraction_t: t,
                    arc_length_s_m: t * l_m,
                    iter_count: step.iter_count,
                    final_residual_norm: step.final_residual_norm,
                    x_final: step.x_final.clone(),
                    readout: step_readout,
                });
                intruder_poses.push(pose);
                x_prev_flat = step.x_final;
            }
            Err(payload) => {
                failed_at_step = Some(k);
                failure_reason = Some(panic_message(&*payload));
                break;
            }
        }
    }
    // Final-step per-tet detail + F-arc-length curve (same pattern as the growing ramp).
    let result = steps.last().map(|last| { ... compute_tet_readouts at last x_final ...
        SlideResult {
            final_per_tet,
            force_arc_length_curve: steps.iter()
                .map(|s| (s.arc_length_s_m, s.readout.contact_force_magnitude_n))
                .collect(),
        }
    });
    Ok(SlideRamp { steps, failed_at_step, failure_reason, final_x: x_prev_flat,
                   n_pinned: bc.pinned_vertices.len(), result, intruder_poses })
}
```

### 3e. UI struct shape

```rust
pub enum RampKind {
    Sliding(SlideRamp),
    Growing(InsertionRamp),
}

pub struct InsertionSimOutputs {
    pub ramp_kind: RampKind,
    pub per_layer: Vec<LayerAggregate>,
    pub tet_centroids: Vec<Vector3<f64>>,
    pub per_tet_layer: Vec<usize>,
    pub per_step_scalar_fields: Vec<[Vec<f64>; 2]>,
    pub scalar_min_max: [(f64, f64); 2],
    pub cavity_boundary_faces: Vec<[u32; 3]>,
    pub per_layer_outer_faces: Vec<Vec<[u32; 3]>>,
    // Replaces per_step_intruder_meshes:
    pub intruder_poses: Vec<Isometry3<f64>>,
    // For sliding, populated by SlideRamp.intruder_poses.
    // For growing, populated with `vec![Isometry3::identity(); steps.len()]`
    // (growing-intruder rendering will need a separate scale-state field if
    // we want to still visualize the iso-grow inside that mode — defer to
    // the pass-B "growing mode reawakening" sub-leaf).
    pub n_steps_converged: usize,  // borrowed from whichever ramp kind
}

impl InsertionSimOutputs {
    pub fn step_count(&self) -> usize { ... }
    pub fn intruder_pose_at(&self, step: usize) -> Option<Isometry3<f64>> { ... }
    pub fn force_arc_length_curve(&self) -> &[(f64, f64)] { ... }  // unified accessor
    // deformed_layer_slab_mesh_at, deformed_layer_mesh_at, deformed_boundary_mesh_at
    // — UNCHANGED (operate on per-step x_final by index, common to both ramps).
}
```

### 3f. Render system rename

`update_intruder_mesh` (S11.1) → `update_intruder_transform` (sliding); `IntruderMeshKey` (3-field) stays the same shape (still tracks `displayed_step`, `last_run_generation`, `show_deformed`); the body changes from mesh-asset swap to `Transform.translation` write driven by `intruder_pose_at(displayed_step).translation`. Mount the constant cleaned-scan mesh ONCE at startup in `spawn_intruder_mesh` (back to S4 pattern). When `SimMode::Sliding` and a run is loaded, write the per-step translation; otherwise hide.

For `SimMode::GrowingIntruder` (pass B), the path can render either (i) bare scan at the seated step + identity transform (one frame at the end of the ramp, since the growing mode's per-step intruder doesn't have a transform meaning) or (ii) restore the S11.1 mesh-asset swap inside this branch. Pass B decides.

---

## 4. Sub-leaf ladder

Each sub-leaf is a single commit with a per-leaf gate. The ladder is sized so each leaf is ≤ ~300 LOC + tests; the implementation arc is ~6-8 leaves over 2-3 sessions.

| # | Leaf | Scope | Per-leaf gate | Tests |
|---|------|-------|---------------|-------|
| **SL.0** | **Pre-implementation gates (this section + §5)** | Verify baseline: `cargo test -p cf-device-design --release` passes the existing 152; user-verified iter-1 still loads in cf-device-design tool. No code change — just the cold-read + bench. | Baseline green; tool launches; sock_over_capsule renders. | n/a (sanity). |
| **SL.1** | **`TransformedSdf` adapter + `slide_pose_at` + `point_along_polyline_at_arc_distance` (local copy) + `polyline_arc_length_m` (local copy)** in `insertion_sim.rs` (with `#[allow(dead_code)]` until consumed). | Pure helpers; no FEM wiring. | `cargo test -p cf-device-design --release` green; new tests for `TransformedSdf` against a `RigidPlane` (eval/grad at translated + rotated poses); `slide_pose_at(t=0)` lands tip at floor, `t=1` lands tip at rest, `t=0.5` lands tip mid-arc. | Add ~6 unit tests; allow lints for the dead path until SL.3 consumes it. |
| **SL.2** | **Solver-only fixture for `run_sliding_insertion_ramp`** — synthetic icosphere body + radial centerline (no Bevy, no `.prep.toml`). Implement `intruder_contact_sliding_at`, `SlideRampStep`, `SlideRamp`, `run_sliding_insertion_ramp`. **No UI wiring yet.** | Validate FEM convergence on a controlled body before exposing to the panel UI. | Synthetic-icosphere fixture passes: with cavity_inset = 1 mm, 8-step ramp, all 8 steps converge; final contact force magnitude > 0 N at the seated step; intermediate steps show monotonically-changing force; rest-of-cavity vertices (far from contact zone) have `‖x_final − rest‖ < 0.5 mm` (local deformation, not uniform). | New test module `sliding_insertion_ramp_tests`; 3-4 fixture tests. |
| **SL.3** | **Wire the new ramp through `run_sim_pipeline` + `InsertionSimOutputs` struct refactor**. Add `SimMode { Sliding, GrowingIntruder }` to `InsertionSimState` (default `Sliding`). Refactor `InsertionSimOutputs` to `RampKind` + `intruder_poses` replacing `per_step_intruder_meshes`. Update `run_sim_pipeline` to dispatch on `sim_mode`. Pipe `centerline_polyline_m` through `kick_off_simulation` (snapshot the `Centerline` resource into the async task input). At this leaf, the `GrowingIntruder` branch can panic-or-return-empty stub (pass B fills it). | The FEM output now flows to the existing render plumbing. | iter-1 sock_over_capsule loads, "Simulate" produces a SlideRamp, the deformed-cavity render shows a deformation wave propagating from cap mouth to closed end as the user scrubs `displayed_step`. The intruder entity is still hidden / wrong-rendered (SL.4 fixes); cavity render alone validates the FEM. | Existing 13 growing-intruder ramp tests still pass (separate function); new fixture tests stay green; iter-1 visual gate is the primary signal. |
| **SL.4** | **Intruder render: rename `update_intruder_mesh` → `update_intruder_transform`, mount the constant cleaned-scan mesh at startup, write per-step `Transform.translation`** from `last_run.intruder_pose_at(displayed_step).translation`. Drop the S11.1 mesh-asset swap. For performance on high-face scans, consider a one-time `meshopt::simplify_decoder` to ~30k faces for the IntruderEntity ONLY (the ScanMeshEntity keeps full resolution). | The visual intent is complete. | iter-1 visual gate: user-verified that the rendered intruder slides along the centerline arc from cap mouth to closed end as `displayed_step` scrubs; the rendered intruder coincides with the deformed cavity in the contact zone at every step. | Update the existing `IntruderMeshKey` tests for the new body; remove the per-step intruder MC contract test from `insertion_sim_ui.rs::tests`. |
| **SL.5** | **UI text update for sliding semantics** (D-Slide7): convergence summary, F-d plot axis label, per-step table header, step badge copy. Add the `SimMode` enum selector to the Sim panel (defaults to Sliding; GrowingIntruder option present but greyed-out, "Coming in pass B" tooltip). | UI reads accurately for the sliding model. | iter-1 visual gate: copy reads correctly; F-d plot's x-axis is "Slide arc-length (mm)"; per-step table shows "Slid (mm)"; step badge says "Step k/N — slid X mm of Y mm". | Add 2-3 panel-render tests that pin the copy strings (similar to existing `render_convergence_summary` test pattern). |
| **SL.6** | **Visual gate iter-1 + post-ship cold read** per `[[feedback-cold-read-review-post-ship]]`. Bundle real bugs + doc lies + test gaps + bikeshed into one polish commit if found. | User-verified end-to-end visual + cold-read clean. | iter-1 user sign-off; cold-read finds ≤ 3 polish items (typical). | Polish commit only; no new logic. |
| **SL.7** (optional pass B) | **Reawaken GrowingIntruder mode behind the UI selector**. Add the `RampKind::Growing` branch to `run_sim_pipeline` that calls the existing `run_insertion_ramp`; populate `intruder_poses` as `vec![Isometry3::identity(); steps.len()]`; in `update_intruder_transform`, when in growing mode, swap the IntruderEntity to render at rest-pose-translated-by-cap-normal (replicating the original S4 visual for the growing case — acceptable since growing-mode is now explicitly "prototyping a variation", not the primary workshop view). Ungrey the panel selector. | The "future use case" the user banked is accessible. | Switching mode produces a different ramp; growing mode still converges (the existing 13 tests guarantee); both modes co-exist in one tool. | n/a (functionality already tested). |

Implementation-arc cadence: SL.0 + SL.1 + SL.2 in session N (recon-then-fixture day); SL.3 + SL.4 in session N+1 (FEM-to-render plumbing); SL.5 + SL.6 in session N+2 (polish + visual gate); SL.7 optional in session N+3.

---

## 5. Pre-implementation gates

Before SL.1 lands, the implementation session must validate:

1. **Baseline test pass**: `cargo test -p cf-device-design --release` returns 152 passes, clippy `-D warnings` clean. If not, fix the baseline before adding new code.
2. **Iter-1 tool launches**: `cargo run --release -p cf-device-design -- --cleaned-stl <iter-1 path>` opens the viewport, scan visible, no panics. Confirms the rendering plumbing is in the state the bookmark left it.
3. **Verify centerline available**: the iter-1 `.prep.toml` has a `[centerline]` block with ≥ 2 points; `parse_centerline` returns a non-empty `Vec<Point3<f64>>`. If centerline is absent, sliding mode cannot ship — surface a panel error and fall back to growing mode (this is part of SL.3 but the data dependency must be checked at SL.0).
4. **Solver fixture validates contract before UI integration** (SL.2 gate): the synthetic-icosphere fixture is the falsifier. If contact convergence fails on a clean controlled body, the problem is in the contact primitive (SL.1) or the warm-start strategy (D-Slide4), not the body geometry; debug there before touching the UI plumbing.

The fixture exists in code as `sliding_insertion_ramp_tests::synthetic_icosphere_ramp_converges_all_steps` and similar; running it locally is a single `cargo test -p cf-device-design --release sliding_insertion_ramp_tests` invocation.

---

## 6. Open risks

1. **Centerline absence on iter-2+ scans**. cf-scan-prep populates `[centerline]` based on first-cap-loop heuristic (`tools/cf-scan-prep/src/main.rs:5367`). A multi-cap scan without an explicit "primary cap" may have an empty centerline or a degenerate one. **Mitigation**: SL.3 surfaces a "no centerline; sliding-mode requires `[centerline]`" panel error and falls back to growing mode. SL.7 promotes growing mode out of greyed-out state to handle this gracefully.
2. **High curvature → rigid-translation intersection artifacts**. A scan with a tight bend (e.g., elbow joint, wrist) will have the rigid intruder body wildly intersecting the cavity wall at intermediate slide positions. The FEM will resolve this by deforming the cavity outward (physically correct: the rigid object squishes the silicone), but the contact pressures + stretches may exceed the Yeoh material's calibrated stretch range, surfacing as `max_principal_stretch > validity.max_principal_stretch` warnings in the per-layer aggregates. **Mitigation**: iter-1 is mostly straight (capsule fixture) so this won't bite; if iter-2 surfaces it visibly, the iter-2 follow-up is D-Slide2 rotation (= match intruder local-tangent to centerline-tangent), banked as a sub-leaf.
3. **Convergence rate without convection-aware warm starts** (D-Slide4 caveat). The current `INSERTION_SOLVE_TOL = 1e-1` + 150 Newton iter cap may stall at slide steps where the contact set changes rapidly (intruder entering / leaving narrow sections). The Fork-B catch_unwind handles this gracefully but at the cost of partial-seating ramps. **Mitigation**: if iter-1 shows frequent stalls before full seating, a follow-up sub-leaf adds convection-aware warm starts (offset behind-vertices by the slide delta; leave ahead-vertices at rest). 50-100 LOC of mesh-side bookkeeping; banked.
4. **High-face cleaned scan performance for the IntruderEntity render**. iter-1 cleaned scan is ~167k faces (cf-cap-planes module docs). Bevy can render this 60fps for one static entity, but the spawn cost is ~100 ms at startup. **Mitigation**: SL.4 considers `meshopt::simplify_decoder` to ~30k faces for the intruder render asset only — same pattern as cf-device-design's existing `decimate_for_sdf` precedent. If startup time stays under 1 s, this can be skipped.
5. **`TransformedSdf` gradient at grid clamp boundaries**. When the inverse-transformed query point lands outside the `GridSdf`'s grid coverage, `grad_clamped` returns the gradient at the nearest grid sample. For active contact pairs (`sd < d̂`), this is benign (the query lands well inside the grid). For inactive pairs, the gradient is never evaluated. **Mitigation**: SL.2 fixture test verifies that grad evaluations on active pairs at extreme slide positions (`t = 0.1`, `t = 0.9`) return finite, unit-norm gradients (within 5% of `1.0`). Failures here would surface as Newton non-convergence; trust the test gate.
6. **Force-arc-length curve interpretation for the F-d plot consumer**. The existing F-d plot consumer (`render_force_displacement_plot`) reads `(interference_m, force_n)` pairs. The sliding ramp's curve is `(arc_length_s_m, force_n)`. SL.5 updates the axis label and consumer signature. **Mitigation**: SL.3 adds a `force_arc_length_curve()` unified accessor on `InsertionSimOutputs` returning `&[(f64, f64)]` in the ramp-natural units; the plot consumer is signature-agnostic. The label string drives interpretation.

---

## 7. Cross-references

- **Bookmark**: `docs/SIM_ARC_SLIDING_INTRUDER_BOOKMARK.md` (this spec's parent).
- **Bookmark memory**: `[[project-cf-device-design-sliding-intruder-bookmark]]`.
- **Superseded recon**: `docs/SIM_ARC_RECON.md` (S1-S4 + S11.1 + S11.2 shipped on its terms; product fit falsified mid-arc, not spec craftsmanship).
- **Superseded visual-rewrite spec**: `docs/SIM_ARC_VISUAL_REWRITE_SPEC.md` (the S11 spec — its §3.5 "explicitly NOT in scope" called out slide animation; that call is now superseded by D-Slide2).
- **Pivot commit**: dev `1ac4fd12` ships the bookmark; this spec ships on top of it (anticipated commit name: `docs(sim-arc): SIM_ARC_SLIDING_INTRUDER_SPEC.md — recon output`).
- **Three-session pattern**: `[[feedback-bookmark-when-surface-levers-exhaust]]`.
- **Autonomous-architect authority**: `[[feedback-autonomous-architecture]]` — drove the in-session decision-resolution posture for this spec.
- **Cold-read post-ship**: `[[feedback-cold-read-review-post-ship]]` — SL.6 of the ladder.
- **Code locations** (anchored as of dev `1ac4fd12`):
  - `tools/cf-device-design/src/insertion_sim.rs:1838` — `run_insertion_ramp` (the growing-intruder ramp, preserved).
  - `tools/cf-device-design/src/insertion_sim.rs:929` — `intruder_contact_at` (uniform-offset contact, kept for growing mode).
  - `tools/cf-device-design/src/insertion_sim.rs:669` — `build_insertion_geometry` (shared, unchanged).
  - `tools/cf-device-design/src/insertion_sim.rs:956` — `outer_skin_bc` (shared, unchanged).
  - `tools/cf-device-design/src/insertion_sim_ui.rs:130` — `InsertionSimOutputs` struct (refactor target at SL.3).
  - `tools/cf-device-design/src/insertion_sim_ui.rs:667` — `run_sim_pipeline` (dispatch target at SL.3).
  - `tools/cf-device-design/src/insertion_sim_ui.rs:551` — `kick_off_simulation` (centerline-polyline plumbing target at SL.3).
  - `tools/cf-device-design/src/main.rs:1747` — `spawn_intruder_mesh` (re-mount target at SL.4).
  - `tools/cf-device-design/src/main.rs:1811` — `update_intruder_mesh` (rename target at SL.4).
  - `tools/cf-device-design/src/main.rs:1782` — `IntruderMeshKey` struct (shape preserved).
  - `tools/cf-device-design/src/main.rs:363` — `parse_centerline` (existing prep-toml parser).
  - `tools/cf-scan-prep/src/main.rs:1016` — `point_along_polyline_at_arc_distance` (local copy target at SL.1; do not introduce a tools→tools dep).
  - `design/cf-cap-planes/src/lib.rs:131` — `CapPlane` (mostly informational — sliding doesn't consume cap normals after SL.3).
  - `design/cf-design/src/sdf.rs:28` — `Sdf` trait (TransformedSdf implements it).
  - `sim/L0/soft/src/contact/penalty.rs:74` — `PenaltyRigidContact` (sliding contact primitive composes it).

---

## Patterns to bank when the implementation arc concludes

Probable banks (subject to what the implementation surfaces):

- **Three-session pattern executed in full on a mid-arc product pivot** — bookmark → recon → implementation. The earlier `INSERTION_SIM_STATE.md` example was a within-session bookmark; this is the cross-session version where the product intent itself shifted.
- **"FEM solver model is wrong-fit, not just the rendering"** — an escalation criterion for when a visual gate uncovers something architectural. Distinct from the "visual rewrite bookmark" pattern (which is a render-only pivot). The triggering signal here was user-as-product-arbiter contradicting the spec's "explicitly NOT in scope" annotation.
- **TransformedSdf trait adapter** — reusable beyond this arc as a primitive for any moving-rigid-body SDF contact scenario.
- **Solver-only fixture before UI integration** — SL.2 separates FEM correctness from rendering correctness; lets each phase fail closed independently. Mirrors row 23's `scan-fit-3layer-sleeve-yeoh-ramp` example fixture pattern.
- **"Spec's explicitly-NOT-in-scope is a hypothesis, not a commitment"** — the visual-rewrite spec listed slide animation as out-of-scope; the user falsified that hypothesis mid-arc. Future specs should annotate out-of-scope items with the user-validation level (e.g., "user-confirmed out of scope" vs "spec-author-presumed out of scope").
