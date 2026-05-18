# cf-device-design Sim Arc — Sliding-Intruder Contact-Primitive Recon

**Status**: IMPLEMENTATION SHIPPED 2026-05-19 on branch `sim-arc/sliding-intruder-contact-recon`. CR.0+ → CR.4 landed across commits `d34d1181` (probe + iter-1 step-0 histogram) → `c0c7d3a0` (CR.1 sim-soft interior_cutoff) → `61d8b2fd` (CR.2 cf-device-design wire-up) → `cc8340a2` (CR.3 contact-force sentinel + icosphere docstring flip + new probe unit test). CR.4 iter-1 visual gate completed — see §10 below. CR.5 cold-read next.

**Status of recon spec**: RECON 2026-05-19 (v5 — five-iteration convergence after three cold-read passes that successively eliminated wrong architectural framings). Three-session pattern's second instance for this arc: bookmark (`a1f13a57` — SL.3 visual-gate failure recorded at [[project-cf-device-design-sliding-intruder-bookmark]]) → recon (this spec) → implementation (SHIPPED).

**The fix**: add an `interior_cutoff: Option<f64>` field to `PenaltyRigidContact` and filter the active-set walk to exclude pairs where `sd < -interior_cutoff`. cf-device-design's sliding ramp passes `2 × design.cavity_inset_m` (= 6 mm default) as the cutoff. ~20 LOC of sim-soft addition + signature change to `run_sliding_insertion_ramp` to thread `cavity_inset_m`. Existing `TransformedSdf<GridSdf>` contact primitive unchanged. No SDF gymnastics, no vertex-identity classification, no chain-rule math.

**Reading order**: §1 (falsification recap + why earlier framings failed) → §2 (interior_cutoff design) → §3 (decisions D-Contact1..D-Contact4) → §4 (architectural shape + APIs) → §5 (sub-leaf ladder CR.0..CR.5) → §6 (pre-implementation gates) → §7 (open risks) → §8 (cross-references) → §9 (version history — audit trail).

---

## 1. Falsification recap

### 1a. The SL.3 stall

`docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` §3b shipped the `TransformedSdf<S: Sdf>` adapter with the load-bearing claim (lines `:152-166`) that the static intruder `GridSdf` covers the body bbox + outer margin, so inverse-transformed query points for any well-conditioned slide pose land within the grid and report physically-meaningful signed distance. SL.3 falsified this on iter-1 sock_over_capsule.

**Mechanism**: the static intruder `GridSdf` is built via `build_grid_sdf` over the CLOSED cleaned scan and flood-filled to return NEGATIVE inside the closed body. The closed cleaned scan IS the body-part shape; the device's silicone shell wraps around it with cavity = `scan.offset(-cavity_inset_m)`. At any non-rest slide pose, the TransformedSdf evaluates `inner.eval(inverse_pose * world_p)` — and the inverse-transform shifts world points along the centerline BACKWARD by the slide distance. Any soft-body BCC vertex whose inverse-transformed position lands inside the closed scan's body-part interior gets deep-negative `sd` (because the GridSdf reports interior of the closed scan as flood-filled negative). After `.offset(cavity_offset_m = -3 mm)` composition (`Solid::offset(d)` = `f(p) - d` per `design/cf-design/src/solid.rs:880`), the composed result stays well under `d̂ = 1 mm` → `PenaltyRigidContact::active_pairs` (`sim/L0/soft/src/contact/penalty.rs:273`) fires → force `κ·(d̂ - sd)·n` with `(d̂ - sd)` in tens of millimeters → tangent system non-SPD or grossly inconsistent → Armijo line-search stalls. Observed at SL.3 iter-1 visual gate: `r_norm=4.14, α=4.77e-7, Newton iter 41` at step 0.

The mechanism is NOT specific to a vertex CLASS (body bulk, orphans, cavity wall) — it fires for ANY vertex whose inverse-transformed position lands in the static scan's closed interior. Even cavity-wall vertices, which are "the right vertices to participate in contact," fire pathologically at non-seated slide poses when their inverse-transformed positions land deep in the static body part. The bookmark's first diagnosis ("body bulk + orphan margin vertices") is an instance of the broader pattern, not the full mechanism.

The growing ramp at `intruder_contact_at` (`:929`) sidesteps this because `intruder.offset(interference_m + cavity_offset_m)` SHRINKS the intruder for early steps and only lands at the scan surface at step 16. Body BCC interior vertices never see deep-negative `sd` in the growing case.

### 1b. Why the bookmark's options (a) and (b) all converged on (c) at the active-set level

The bookmark presented four options for the fix:
- (a) parry TriMesh of the scan surface;
- (b) vertex-mask filter;
- (c) clamp-SDF-to-positive past a band;
- (d) expand the GridSdf — dismissed as not addressing the deep-interior issue.

v1-v3 pursued (a) in various forms; v4 pivoted to (b). All four iterations were withdrawn (see §9 for the audit trail). The pattern that emerged:

- **(a) parry TriMesh alone** doesn't fix anything — closed-body parry-pseudo-normal-signed distance has the SAME deep-interior pathology as flood-fill-signed GridSdf (verified during v1 + v2 design).
- **(a) + band-clamp SDF wrapper** fights the `Sdf` trait contract — smoothstep blend is non-monotonic; FLOOR clamp has zero-gradient pathology (verified during v2 + v3).
- **(b) vertex-mask by REST-position cavity proximity** filters orphans + body bulk correctly but DOES NOT filter the inverse-transform pathology for cavity-wall vertices in the mask (verified during v4 cold-read).

What ALL three approaches converged on, when extrapolated: the only durable fix is to **exclude pairs where the queried `sd` is itself pathologically deep-negative**, regardless of which vertex it belongs to. This is option (c) — applied NOT as an SDF wrapper (which fights the `Sdf` trait) but as a filter at the active-set walk inside `PenaltyRigidContact`. The Sdf trait stays intact; the contact model decides which pairs participate. The bookmark dismissed (c) as "hacky stopgap" based on the SDF-wrapper formulation; at the active-set level it's a principled active-set restriction with one clear physical interpretation: "a vertex more than one design margin past the engineered interference is in a non-physical configuration the FEM can't productively resolve; exclude it and let the rest of the system progress."

---

## 2. interior_cutoff design

**Core idea**: `PenaltyRigidContact::active_pairs` currently fires for every `(vertex, primitive)` pair where `prim.eval(p) < d̂`. Add a second condition: `prim.eval(p) >= -interior_cutoff_m` (where `interior_cutoff_m` is a positive number, default infinity = current behavior). The active set becomes the BAND `sd ∈ [-interior_cutoff_m, d̂]`. Pairs whose sd is deep-negative are silently skipped — no force contribution, no gradient call, no Hessian call.

**Physical interpretation**: a soft-body vertex with `sd = -50 mm` is 50 mm "inside" the rigid primitive. That's a non-physical configuration for a sliding rigid intruder against a thin silicone shell — the silicone can't wrap that far around the intruder. The most useful thing the FEM can do is treat the vertex as "not currently in contact" and let elastic restoring forces move it. Once its motion brings sd back into the band, contact fires normally.

**Default cutoff**: `2 × design.cavity_inset_m` (= 6 mm by default). One design margin past the engineered interference. Sized so vertices physically in contact (sd ∈ [-3 mm, 0]) are inside the band; vertices in the pathological inverse-transform-into-body-part configuration (sd typically -20 mm to -50 mm at iter-1) are excluded.

**Why this works where (a), (b), (c)-as-SDF-wrapper, (d) all failed**:

- **No `Sdf` trait gymnastics**: the underlying SDF stays a real signed distance function with monotonic eval and outward grad. The filter is at the contact model, not at the SDF.
- **No chain rule math**: the filter is a scalar comparison. No smoothstep, no analytic derivative, no chain rule.
- **No vertex-identity classification**: every vertex is eligible to fire; only the per-pair sd value determines participation. Pose-dependent automatically (rebuilt every active_pairs call inside the Newton solve).
- **No new SDF construction**: existing `TransformedSdf<GridSdf>` contact primitive untouched.
- **No vertex-mask construction sweep**: zero pre-ramp cost.

**Cost**: one extra scalar comparison per `active_pairs::eval` call (essentially free). Per-step cost unchanged from SL.3 baseline.

**Why default cutoff at the call site, not at the sim-soft default**: sim-soft can't know what the right cutoff is for a given physical scenario — it depends on the rigid body's geometry + the soft body's expected deformation envelope. Default `None` (= current behavior, no filter) preserves byte-identical behavior for every existing PenaltyRigidContact consumer (Hertz sphere, compressive block, plane-contact tests, the growing ramp). cf-device-design's sliding ramp opts in by setting `interior_cutoff_m = 2 × cavity_inset_m`.

---

## 3. Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| **D-Contact1** | **Add `interior_cutoff: Option<f64>` field to `PenaltyRigidContact` + a new constructor `with_params_and_interior_cutoff(primitives, kappa, d_hat, interior_cutoff)`** with `debug_assert!(interior_cutoff > d_hat)` to guard against misconfiguration (cited as a P2 in v5 cold-read; see §7 risk #6). Existing `new` and `with_params` constructors default `interior_cutoff` to `None`; behavior unchanged for ALL current consumers (verified ~10+ tests in `sim/L0/soft/tests/` touching `PenaltyRigidContact`). | Smallest viable sim-soft API surface — one Option<f64> field + one constructor variant. The Option keeps every existing test fixture byte-identical. |
| **D-Contact2** | **Filter `active_pairs` (penalty.rs:273-288) AND `per_pair_readout` (penalty.rs:140-167) to require `sd >= -interior_cutoff` when set.** Both walks compute `sd = prim.eval(p)`; add `if let Some(cutoff) = self.interior_cutoff { if sd < -cutoff { continue; } }` immediately after the eval call, BEFORE the existing `sd < d_hat` band-gate. | The filter is at the same code site as the band-gate — same per-pair scalar comparison cost. `gradient`/`hessian`/`energy` are pair-indexed (called by the solver via the `ContactModel` trait on pairs that `active_pairs` produced), so they automatically respect the filter. |
| **D-Contact3** | **cf-device-design passes `interior_cutoff_m = 2 × design.cavity_inset_m` (= 6 mm default) at sliding-ramp setup.** `run_sliding_insertion_ramp` gains a `cavity_inset_m: f64` parameter (cannot derive from `geometry` — `cavity_offset_m = -cavity_inset_m` is stored, but the sliding ramp consumes the positive value for the cutoff sizing). `intruder_contact_sliding_at` gains the same parameter and forwards it to `PenaltyRigidContact::with_params_and_interior_cutoff`. | Physical anchor (one design margin past engineered interference), not a tuning knob. Scales naturally with `cavity_inset_m`. Banked: if a future workshop measurement shows a need, expose as a `RunSlidingInsertionRampConfig` field. |
| **D-Contact4** | **No change to**: `InsertionGeometry` struct, `outer_skin_bc`, `intruder_contact_at` (growing ramp), `TransformedSdf`, slide_pose_at, polyline helpers, SlideRamp* shapes, `run_sim_pipeline` (sliding branch passes `design.cavity_inset_m` — already in scope at the dispatch site, verified at `insertion_sim_ui.rs:928`-ish region). | Minimum blast radius. SL.3 ship state survives the fix; only the per-step contact-build line changes. |

---

## 4. Architectural shape

### 4a. Data-flow diagram (delta from SL.3 ship state)

```
┌────────────────────────────────────────────────────────────────────────┐
│  sim/L0/soft/src/contact/penalty.rs                                    │
│  ───────────────────────────────────────────────────────────────────── │
│                                                                        │
│  CHANGE: PenaltyRigidContact                                           │
│    fields: + interior_cutoff: Option<f64>                              │
│    ADD: pub fn with_params_and_interior_cutoff(                       │
│           primitives, kappa, d_hat, interior_cutoff) -> Self           │
│                                                                        │
│  CHANGE: active_pairs (line :273-288)                                  │
│    Add filter `if let Some(c) = self.interior_cutoff {                 │
│      if sd < -c { continue; } }` AFTER `let sd = prim.eval(p_pt)`      │
│    AND BEFORE the existing `if sd < self.d_hat { … }` band-gate.       │
│                                                                        │
│  CHANGE: per_pair_readout (line :140-167)                              │
│    Same filter shape at the same position relative to the band-gate.   │
│                                                                        │
│  Existing constructors (new, with_params) unchanged — interior_cutoff  │
│  defaults to None. Test fixtures byte-identical.                       │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│  tools/cf-device-design/src/insertion_sim.rs                          │
│  ───────────────────────────────────────────────────────────────────── │
│                                                                        │
│  KEEP: TransformedSdf, slide_pose_at, polyline helpers, all SlideRamp* │
│         types (UNCHANGED)                                              │
│                                                                        │
│  CHANGE: intruder_contact_sliding_at signature                         │
│    Old: (intruder, bounds, pose, cavity_offset_m)                      │
│    New: (intruder, bounds, pose, cavity_offset_m, cavity_inset_m)      │
│    Body: change the PenaltyRigidContact constructor to                 │
│      PenaltyRigidContact::with_params_and_interior_cutoff(             │
│        vec![intruder_solid],                                           │
│        INSERTION_CONTACT_KAPPA,                                        │
│        INSERTION_CONTACT_DHAT,                                         │
│        2.0 * cavity_inset_m,                                           │
│      )                                                                 │
│                                                                        │
│  CHANGE: run_sliding_insertion_ramp signature                          │
│    Adds: cavity_inset_m: f64                                           │
│    Body: forward cavity_inset_m into intruder_contact_sliding_at       │
│          (2 sites — main contact build + readout rebuild).             │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│  insertion_sim_ui.rs — forward cavity_inset_m                          │
│  ───────────────────────────────────────────────────────────────────── │
│                                                                        │
│  run_sim_pipeline sliding branch passes design.cavity_inset_m into     │
│  run_sliding_insertion_ramp. _cached_sdf parameter stays as-is.        │
└────────────────────────────────────────────────────────────────────────┘
```

### 4b. `PenaltyRigidContact::with_params_and_interior_cutoff` (sim-soft addition)

```rust
// In sim/L0/soft/src/contact/penalty.rs

pub struct PenaltyRigidContact {
    primitives: Vec<Box<dyn Sdf>>,
    kappa: f64,
    d_hat: f64,
    /// Optional active-set lower bound. When `Some(c)`, the active-set
    /// walk excludes pairs whose signed distance is below `-c` (the
    /// "pathologically deep interior" band). `None` (default) preserves
    /// pre-recon behavior — every pair in the contact band fires.
    ///
    /// Use case: rigid-vs-soft sims where the rigid SDF is defined
    /// globally over a closed-body domain (e.g., a flood-filled
    /// `GridSdf` of a cleaned body-part scan), and a moving pose can
    /// project soft-body vertices into deep-interior regions where
    /// the SDF reports `sd << 0`. Without the filter, those pairs
    /// generate force `κ·(d̂ - sd)·n` with `(d̂ - sd)` in tens of
    /// millimeters, breaking Newton convergence (Armijo stall). With
    /// the filter, those pairs are silently skipped; elastic forces
    /// can move the vertex back into the contact band, at which point
    /// normal penalty contact resumes.
    ///
    /// Motivating consumer: cortenforge cf-device-design sliding-
    /// intruder FEM ramp (see
    /// `docs/SIM_ARC_SLIDING_INTRUDER_CONTACT_RECON.md`).
    interior_cutoff: Option<f64>,
}

impl PenaltyRigidContact {
    /// Construct with default `(κ, d̂)` from `PENALTY_KAPPA_DEFAULT` /
    /// `PENALTY_DHAT_DEFAULT`. No interior cutoff.
    #[must_use]
    pub fn new<I>(primitives: I) -> Self
    where I: IntoIterator, I::Item: Sdf + 'static,
    { Self::with_params(primitives, PENALTY_KAPPA_DEFAULT, PENALTY_DHAT_DEFAULT) }

    /// Construct with non-default `(κ, d̂)`. No interior cutoff.
    #[must_use]
    pub fn with_params<I>(primitives: I, kappa: f64, d_hat: f64) -> Self
    where I: IntoIterator, I::Item: Sdf + 'static,
    {
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self { primitives, kappa, d_hat, interior_cutoff: None }
    }

    /// Construct with non-default `(κ, d̂)` plus an interior cutoff.
    /// See [`Self::interior_cutoff`] field docs for the use case.
    ///
    /// `interior_cutoff` must be positive.
    #[must_use]
    pub fn with_params_and_interior_cutoff<I>(
        primitives: I,
        kappa: f64,
        d_hat: f64,
        interior_cutoff: f64,
    ) -> Self
    where I: IntoIterator, I::Item: Sdf + 'static,
    {
        assert!(
            interior_cutoff > 0.0 && interior_cutoff.is_finite(),
            "interior_cutoff must be positive and finite, got {interior_cutoff}",
        );
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self { primitives, kappa, d_hat, interior_cutoff: Some(interior_cutoff) }
    }
}
```

### 4c. `active_pairs` + `per_pair_readout` filter

```rust
// active_pairs body, around penalty.rs:273-288 (delta only):
for (vid, &p) in positions.iter().enumerate() {
    let p_pt = Point3::from(p);
    for (pid, prim) in self.primitives.iter().enumerate() {
        let sd = prim.eval(p_pt);
        // NEW: interior cutoff — skip pathologically deep-interior pairs.
        if let Some(c) = self.interior_cutoff {
            if sd < -c { continue; }
        }
        // EXISTING: contact band-gate.
        if sd < self.d_hat {
            pairs.push(ContactPair::Vertex {
                vertex_id: vid as VertexId,
                primitive_id: pid as u32,
            });
        }
    }
}

// per_pair_readout body, similar shape at penalty.rs:140-167.
```

### 4d. `intruder_contact_sliding_at` rewrite (cf-device-design)

```rust
fn intruder_contact_sliding_at(
    intruder: &GridSdf,
    bounds: Aabb,
    slide_pose: Isometry3<f64>,
    cavity_offset_m: f64,
    cavity_inset_m: f64,                // NEW
) -> PenaltyRigidContact {
    let transformed = TransformedSdf::new(intruder.clone(), slide_pose);
    let intruder_solid = Solid::from_sdf(transformed, bounds).offset(cavity_offset_m);
    PenaltyRigidContact::with_params_and_interior_cutoff(
        vec![intruder_solid],
        INSERTION_CONTACT_KAPPA,
        INSERTION_CONTACT_DHAT,
        2.0 * cavity_inset_m,           // D-Contact3
    )
}
```

The only difference from the current SL.3 body: `with_params` → `with_params_and_interior_cutoff` + the new `cavity_inset_m` parameter.

### 4e. `run_sliding_insertion_ramp` signature change

```rust
pub fn run_sliding_insertion_ramp(
    geometry: InsertionGeometry,
    centerline_polyline_m: &[Point3<f64>],
    n_steps: usize,
    cavity_inset_m: f64,                // NEW
) -> Result<SlideRamp> {
    // ... validation unchanged ...
    // ... per-step loop unchanged in shape; both contact build lines
    //     gain the cavity_inset_m parameter:
    //     intruder_contact_sliding_at(&intruder, bounds, pose,
    //                                 cavity_offset_m, cavity_inset_m)
    // ...
}
```

`run_sim_pipeline` sliding branch passes `design.cavity_inset_m` (already in scope).

---

## 5. Sub-leaf ladder

| # | Leaf | Scope | Per-leaf gate | Tests |
|---|------|-------|---------------|-------|
| **CR.0** | **Pre-implementation gates** (§6) — baseline test pass on sim-soft + cf-device-design; iter-1 launches in growing mode; recon doc cold-read clean. | n/a | Baseline tests green; tool launches; spec read end-to-end. | n/a (sanity). |
| **CR.0+** | **Instrumented probe step — falsify the 6 mm cutoff before committing**. Add a one-shot diagnostic in `intruder_contact_sliding_at` (gated by a `cfg!(debug_assertions)` or a temporary `eprintln!`) that, at step 0 of iter-1 sliding ramp, buckets the per-pair raw `transformed.eval` values into bins (e.g., `[+∞, +d̂)`, `[+d̂, 0)`, `[0, -3 mm)`, `[-3, -6 mm)`, `[-6, -10 mm)`, `[-10, -20 mm)`, `[-20, -∞)`) and prints the bucket counts. Run iter-1 sliding ramp ONCE pre-fix (with the existing `with_params`, no cutoff filter). Read the histogram: if substantial mass lives in `[-3 mm, -6 mm)` (= composed sd `[0, -3 mm)` per `Solid::offset(-3 mm)` arithmetic — i.e., the part of the band that's INSIDE the v5 cutoff and would still fire), the cutoff is well-placed. If substantial mass also lives in `[-6 mm, -9 mm)` (composed sd `[-3 mm, -6 mm)` — physical contact at the deep edge of the band), confirm those are physically-meaningful active pairs not pathological. If iter-1 step-0 histogram shows tens-of-thousands of pairs at `transformed.eval < -10 mm` (composed sd `< -7 mm`), the 6 mm cutoff correctly catches them. After the probe step, REMOVE the diagnostic (or guard it permanently). | Histogram printed; verdict logged in CR.0+ commit message: "6 mm cutoff catches N pathological pairs; M physical pairs preserved." | n/a (diagnostic). |
| **CR.1** | **sim-soft addition**: `PenaltyRigidContact::with_params_and_interior_cutoff` constructor + `interior_cutoff: Option<f64>` field + filter at `active_pairs` (penalty.rs:273-288) + filter at `per_pair_readout` (penalty.rs:140-167). Per §4b + §4c. New tests in a dedicated file `sim/L0/soft/tests/penalty_interior_cutoff.rs` (mirrors the `penalty_pair_readout.rs` + `penalty_compressive_block.rs` integration-test precedent). | `cargo test -p sim-soft --release` green; existing PenaltyRigidContact tests pass byte-identical (cutoff defaults to None). 3 new unit tests pin filter behavior. | sim-soft API surface gains one constructor + one optional field; no behavior change for existing consumers. | 3 new sim-soft tests: (i) `penalty_with_interior_cutoff_excludes_deep_interior_pair` — synthetic 1-primitive fixture with one vertex at `sd = -10 mm` and cutoff = `5 mm`; assert that pair is excluded from `active_pairs`; (ii) `penalty_with_interior_cutoff_keeps_within_band_pair` — same fixture with vertex at `sd = -3 mm` and cutoff = `5 mm`; assert pair IS included; (iii) `penalty_per_pair_readout_respects_interior_cutoff` — same shape on `per_pair_readout`. |
| **CR.2** | **cf-device-design wire-up**: signature changes per §4d + §4e to `intruder_contact_sliding_at` + `run_sliding_insertion_ramp`. `run_sim_pipeline` sliding branch passes `design.cavity_inset_m`. Existing in-tree test call sites that need signature updates: (a) `insertion_sim.rs:4789` `run_sliding_insertion_ramp(geometry, &single_point, 8)` → append `, 3e-3`; (b) `insertion_sim.rs:4819` `intruder_contact_sliding_at(&geometry.intruder, geometry.bounds, pose, -0.003)` → append `, 3e-3` (cavity_inset_m for synthetic fixture); (c) `insertion_sim.rs:4878` (main `run_sliding_insertion_ramp` call in `sliding_insertion_ramp_converges_on_synthetic_icosphere`) → append `, 3e-3`; (d) `insertion_sim.rs:4981` (`sliding_insertion_ramp_localizes_deformation_at_intermediate_step`) → append `, 3e-3`. Production call site: `insertion_sim_ui.rs:928`-ish `run_sliding_insertion_ramp(geometry, &centerline_polyline_m, n_steps)` → append `, design.cavity_inset_m`. | `cargo test -p cf-device-design --release` runs to completion. Existing 161 tests pass. | Build green; baseline tests pass. | No new cf-device-design tests at CR.2; CR.3 amends the icosphere test. |
| **CR.3** | **Contact-force regression sentinel** in `sliding_insertion_ramp_converges_on_synthetic_icosphere` (insertion_sim.rs:4872-4954). Amend the first-converged-step assertion to bound `step.readout.contact_force_magnitude_n` to a physically plausible upper limit for the synthetic icosphere fixture. The fixture is a ~40 mm-radius icosphere cup with `cavity_inset_m = 3 mm`, `κ = INSERTION_CONTACT_KAPPA = 1e3` (insertion_sim.rs:915); physically meaningful contact force on a partial seating with O(100) active pairs at ~1 mm interference is `κ × interference × n_pairs ≈ 1e3 × 1e-3 × 100 ≈ 100 N`. Bound at **50 N** for the first-step assertion (slide fraction `t = 1/16 ≈ 0.0625` — small interference, modest active pair count). Under SL.3 v1 the same test would report contact force in the kN range (~1e3 × 30e-3 × 10000 ≈ 3e5 N for the body-bulk firing) — well above 50 N — so the sentinel directly catches regression. **Additionally**: this fixture's docstring at `insertion_sim.rs:4837-4858` frames the step-4 stall as "Yeoh validity wall + narrowing cross-section"; the per-PR-review postmortem already flagged this framing as wishful (the real root cause was the SAME SL.3 mechanism). Post-v5 fix, the expectation is that **all 16 steps converge cleanly**; UPDATE THE DOCSTRING to reflect the new expectation, and CHANGE the test's Fork-B-tolerant assertion to assert `ramp.failed_at_step.is_none()`. Retain the Yeoh-validity-wall branch as a safety net (asserts the failure message matches "validity violation"/"stretch" if a step DOES stall) but flip the expected case. Add a complementary unit test for the cf-device-design `intruder_contact_sliding_at` directly: build a synthetic GridSdf + slide pose + probe position where `transformed.eval ≈ -40 mm`, assert `active_pairs.is_empty()` (because `interior_cutoff = 6 mm` excludes it). | The sentinel detects regression to the SL.3 mode; the docstring update + all-16-steps gate are the affirmative correctness assertion that v5 fixed the icosphere fixture's real failure mode. | 1 amended icosphere test (contact-force-magnitude bound at step 0 + flip Fork-B tolerance to all-16-converge + docstring update) + 1 new cf-device-design unit test (`intruder_contact_sliding_at_excludes_deep_interior_probe`). |
| **CR.4** | **iter-1 visual gate + wall-clock measurement**: launch cf-device-design tool, load iter-1 sock_over_capsule, Simulate (sliding mode) → SlideRamp converges, deformed-cavity render shows a deformation wave propagating from cap mouth to closed end as the user scrubs `displayed_step`. Record wall-clock in commit message. | User-verified visual gate passes; wall-clock recorded. | n/a (visual gate). |
| **CR.5** | **Cold-read post-ship** per `[[feedback-cold-read-review-post-ship]]`. Bundle real bugs + doc lies + test gaps + bikeshed found in the CR.1-CR.4 diff. (Sliding-arc P1 cleanup from the bookmark's PR-review tracked separately.) | Cold-read finds ≤ 3 polish items (typical); polish commit lands. | Polish commit only. |

Implementation-arc cadence: CR.0 + CR.1 + CR.2 + CR.3 in session N (the "land + revalidate" session — small enough to fit in one); CR.4 + CR.5 in session N+1 (visual gate + cold read). No CR.6 — v5 is small enough that no optional perf lever is anticipated.

After CR.5 ships, the original SL.4 (intruder render rewrite from hide-stub to per-step transform writes) is unblocked.

---

## 6. Pre-implementation gates

Before CR.1 lands:

1. **Baseline test pass**: `cargo test -p sim-soft --release` AND `cargo test -p cf-device-design --release` both green, clippy `-D warnings` clean.
2. **Iter-1 growing-mode still loads**: `cargo run --release -p cf-device-design -- --cleaned-stl <iter-1 path>` opens the viewport; growing-mode Simulate produces a 16-step ramp.
3. **Centerline available**: iter-1 `.prep.toml` has a `[centerline]` block with ≥ 2 points.
4. **Recon spec cold-read**: this v5 document is read end-to-end. The §2 interior_cutoff design is the load-bearing departure from v1-v4.

---

## 7. Open risks

1. **Cutoff at `2 × cavity_inset_m = 6 mm` might be too aggressive** for a scenario where the soft body genuinely deforms past 6 mm before equilibrium (e.g., extreme bend in the centerline + tight cavity). A vertex deforming past 6 mm of penetration would silently get "no contact" → cavity wall continues to deform under elastic forces without contact opposition → under-deformation visible as a "vertex pinned/dead" symptom at CR.4 visual gate. Mitigation: CR.4 visual gate catches gross under-deformation; if iter-1 surfaces it, expose `interior_cutoff_factor` as a `RunSlidingInsertionRampConfig` field (default 2.0). Iter-1 sock-over-capsule is mostly straight + the cavity_inset_m is the engineered design margin, so this risk is low at iter-1.

2. **Per-pair sd-discontinuity at the cutoff boundary** — a vertex moving from `sd = -5.9 mm` (active, force `κ · 6.9 mm = 6.9 N` at `κ = 1e3`) to `sd = -6.1 mm` (inactive, force 0 N) inside one Newton iteration creates an Armijo-hostile discontinuity. **This is NOT symmetric with the existing band-gate discontinuity**: at the `sd = d̂` boundary, force vanishes continuously (`κ · 0 · n = 0` on both sides). At the interior cutoff, force jumps from ~7 N per pair to 0 — discontinuous. If many pairs straddle the cutoff in a single Newton iter (correlated motion through the tiny boundary band), the cumulative residual jump could be tens to hundreds of N, potentially triggering an Armijo backtrack-loop of its own. **Mitigation strategy** (sequenced): (a) CR.4 iter-1 visual gate is the falsifier — if it stalls with cutoff-boundary chatter symptoms, (b) CR.5 polish commit adds a smooth-shoulder ramp `force = κ · (d̂ - sd) · smoothstep((sd + c) / shoulder, 0, 1) · n` over the last ~0.5 mm of the cutoff (continuous force vanishing). The reason to NOT add the shoulder upfront: it doubles the implementation scope and obscures the falsification signal. The reason it would likely work: shoulder smooths the discontinuity without changing the active-set decision boundary, and the in-band gradient stays `κ · n` (no chain rule). **If CR.0+ probe step shows >1000 pairs straddling the cutoff band**, escalate the shoulder into CR.1.

3. **No way for an external observer to know if `interior_cutoff` actually fired during a ramp**. If CR.4 visual gate looks correct but the cutoff IS silently filtering more pairs than it should, the under-deformation could be subtle. Mitigation: instrument `SlideRampStep.readout` with a `n_excluded_by_interior_cutoff: usize` counter for diagnostic visibility. Optional; CR.5 polish commit can fold it in if useful.

4. **Sim-soft's interior_cutoff is a new public API surface** with a specific use case described in the doc-comment. Other consumers may misinterpret it. Mitigation: the doc-comment cites this recon as the motivating example; future consumers can read the rationale. The `Option<f64>` default-None preserves byte-identical behavior for anyone who doesn't opt in.

5. **`Solid::from_sdf(transformed, bounds).offset(cavity_offset_m)` composition's eval semantics** — composed sd = `transformed.eval(p) - cavity_offset_m`. With `cavity_offset_m = -3 mm`, composed sd = `transformed.eval(p) + 3 mm`. The interior_cutoff applies to the COMPOSED eval, not the raw `transformed.eval`. So `interior_cutoff = 6 mm` excludes pairs where `composed_eval < -6 mm` → `transformed.eval < -9 mm` (raw signed distance from rigid scan surface < -9 mm in inverse-transformed frame). For iter-1 sock body (max body extent ~80 mm), the deep-interior firing was at `transformed.eval ≈ -30 mm`; that's well past the -9 mm cutoff. Verified the cutoff threshold catches the bug at the DEEP end. The CR.0+ probe step (§5) validates that the NEAR-CUTOFF distribution is also acceptable — if a substantial pair count lives in `transformed.eval ∈ [-9 mm, -3 mm]` (composed ∈ [-6, 0]) AND those pairs are pathological (not physically meaningful contact), the cutoff factor needs to rise from `2 ×` to `3 ×` or `4 ×` `cavity_inset_m`.

6. **Misconfiguration of `interior_cutoff < d_hat`** would give an empty active band `[max(-c, -d_hat), d_hat] = [-c, d_hat]` that excludes legitimate near-contact pairs. cf-device-design's `2 × cavity_inset_m = 6 mm` is comfortably `>> d̂ = 1 mm` so no current consumer hits this. Mitigation: add `debug_assert!(interior_cutoff > d_hat, "interior_cutoff ({c}) must exceed d_hat ({d_hat}) to leave room for near-contact pairs")` in `with_params_and_interior_cutoff`. Cheap; cited as a P2 in the v5 cold-read.

---

## 8. Cross-references

- **Parent bookmark**: `docs/SIM_ARC_SLIDING_INTRUDER_BOOKMARK.md` + `[[project-cf-device-design-sliding-intruder-bookmark]]`.
- **Original sliding-intruder spec**: `docs/SIM_ARC_SLIDING_INTRUDER_SPEC.md` — §3b grid-coverage caveat at `:152-166` is the falsified claim; §3a-§3e remain valid for everything OUTSIDE the contact-model active-set filter. This recon supersedes the contact-primitive active-set behavior only.
- **Three-session pattern**: `[[feedback-bookmark-when-surface-levers-exhaust]]` — second instance for this arc.
- **Implement-measure-revert**: `[[feedback-implement-measure-revert-pattern]]` — CR.3's amended icosphere test is the implement-measure step.
- **Cold-read post-ship**: `[[feedback-cold-read-review-post-ship]]` — CR.5 of the ladder.
- **User not the spec cold-reader**: `[[feedback-user-not-spec-cold-reader]]` — all v1-v4 cold-reads + the v5 cold-read use sibling agents.
- **Code anchors (SL.3 ship state on `main` `a1f13a57`)**:
  - `sim/L0/soft/src/contact/penalty.rs:74-114` — `PenaltyRigidContact` struct + constructors (CR.1 adds field + `with_params_and_interior_cutoff`).
  - `sim/L0/soft/src/contact/penalty.rs:140-167` — `per_pair_readout` (CR.1 adds filter).
  - `sim/L0/soft/src/contact/penalty.rs:273-288` — `active_pairs` walk (CR.1 adds filter).
  - `sim/L0/soft/src/contact/penalty.rs:5-10` — module-level docs noting active-set discontinuity at `d = d̂` (the precedent for risk #2).
  - `tools/cf-device-design/src/insertion_sim.rs:2020-2046` — `TransformedSdf` (UNCHANGED).
  - `tools/cf-device-design/src/insertion_sim.rs:2176-2189` — `intruder_contact_sliding_at` (CR.2 signature + constructor swap per §4d).
  - `tools/cf-device-design/src/insertion_sim.rs:2303-2435` — `run_sliding_insertion_ramp` (CR.2 signature + forward).
  - `tools/cf-device-design/src/insertion_sim.rs:929-942` — `intruder_contact_at` (growing-ramp reference; UNCHANGED).
  - `tools/cf-device-design/src/insertion_sim.rs:956-983` — `outer_skin_bc` (UNCHANGED).
  - `tools/cf-device-design/src/insertion_sim.rs:4872-4954` — `sliding_insertion_ramp_converges_on_synthetic_icosphere` (CR.3 amend with contact-force sentinel).
  - `tools/cf-device-design/src/insertion_sim_ui.rs:928` — `run_sliding_insertion_ramp` dispatch (CR.2 add `design.cavity_inset_m`).
  - `design/cf-design/src/solid.rs:878-900` — `Solid::offset` semantics (verified at v3 + reconfirmed in §7 risk #5).

---

## 9. Version history (audit trail)

| Version | Date | Approach | Outcome |
|---------|------|----------|---------|
| **v1** | 2026-05-19 | (a) parry TriMesh + hand-rolled `RigidScanContact` adapter. | Withdrawn after sibling cold-read: P0 sign-of-offset bug; P0 fictional gate-test name; P1 missed simplification (`CachedScanSdf.sdf_closed` already plumbed). |
| **v2** | 2026-05-19 | (a) reuse `CachedScanSdf.sdf_closed` via existing `TransformedSdf` + new `BandClampedSdf` adapter with smoothstep transition. | Withdrawn after second cold-read: P0 `BandClampedSdf::grad` dropped the analytic `dw/dv · (INACTIVE - v) · inner.grad` chain-rule term; P0 CR.3 tightening to "all 16 steps converge" would fail for non-SL.3 reason (icosphere fixture's documented narrowing cross-section). |
| **v3** | 2026-05-19 | (a) v2 patched: added analytic `dw/dv` term to grad; dropped `INACTIVE` to 10 mm; rewrote CR.3 to use active-pair-count sentinel. | Withdrawn after manual math check: the smoothstep blend produces a NON-MONOTONIC eval inside the band, violating the `Sdf` trait contract. **The band-clamp approach as an SDF wrapper fundamentally fights the `Sdf` trait.** |
| **v4** | 2026-05-19 | (b) vertex-mask via `PenaltyRigidContact::with_params_and_mask` + cf-device-design computes mask from rest-position cavity proximity. | Withdrawn after third cold-read: P0 the mask filters body bulk + orphans but NOT cavity-wall vertices whose inverse-transformed positions land inside the closed scan's body-part interior at non-seated slide poses — the SL.3 stall mechanism persists for the kept-in-mask cavity vertices. The mask is a vertex-IDENTITY filter; the underlying issue is a per-pair sd-VALUE pathology. |
| **v5** | 2026-05-19 | (c) interior_cutoff filter at the active-set walk inside `PenaltyRigidContact` (NOT inside an SDF wrapper). | **This document.** Pending sibling cold-read. |

**Insights banked across the v1-v4 withdrawals** (carry into the cold-read-review-post-ship pattern):

- **"Modifying the SDF semantics to selectively turn off contact past a depth" fights the `Sdf` trait.** Smoothstep blend → non-monotonic; FLOOR clamp → zero-Hessian. The right fix excludes problematic pairs at the active-set walk, not at the SDF. (Banked from v2 + v3 withdrawals.)
- **Vertex-identity filtering doesn't address pose-dependent SDF pathologies.** The vertex-mask approach (v4) correctly excludes orphans + body bulk by IDENTITY, but the SL.3 stall fires for cavity-wall vertices too when their inverse-transformed positions land in the static body interior. The right filter discriminates on the per-pair sd VALUE, not on vertex identity. (Banked from v4 withdrawal.)
- **Two cold-reads + one manual math check is the right cadence when each rewrite introduces a non-trivial new construct.** Each cold-read found real issues; v3's non-monotonicity needed a manual math check because the smoothstep blend was novel construct that cold-read agents reasoned about correctly but didn't fully simulate. (Banked from v2-v3 transitions.)
- **"The bookmark's option list is a direction, not a complete fix."** The bookmark presented (a)/(b)/(c)/(d) as alternatives; the v1-v4 iterations showed that (a) by itself doesn't suffice, (b) by itself doesn't suffice, (c) AS AN SDF WRAPPER doesn't suffice. The convergence point was (c) AT THE ACTIVE-SET WALK — a placement the bookmark didn't enumerate. Recon spec craft is about finding the right LEVEL at which to apply a fix, not just picking from a menu.
- **When iteration count > 3, ask: is the problem genuinely hard, or is my framing wrong?** Five iterations on one spec is a lot. In this case, each iteration genuinely surfaced a real architectural insight, and v5 looks dramatically simpler than v1-v4 — a sign the design is converging on the minimal viable shape. The simplicity of v5 (one Option<f64> field + one filter line) compared to v4 (sim-soft API change + cf-device-design mask construction + signature pollution) is the right end-state for a problem of this kind.

---

## 10. As-built record (CR.0+ → CR.4)

### 10a. Commit ladder

| # | Commit | Crate(s) | Headline |
| - | ------ | -------- | -------- |
| CR.0+ | `d34d1181` | cf-device-design | One-shot step-0 raw `transformed.eval` histogram probe on `run_sliding_insertion_ramp`. Falsified spec §5's "tens of thousands" prediction (actual: 2,232 pathological pairs at iter-1) but validated the 6 mm cutoff value (well-placed: 2.5 k pathological filtered, 2.2 k physical preserved, ~540 straddling — below §7 risk #2 shoulder threshold). |
| CR.1 | `c0c7d3a0` | sim-soft | `PenaltyRigidContact::with_params_and_interior_cutoff` constructor + `interior_cutoff: Option<f64>` field + filter at `active_pairs` (`penalty.rs:300`) and `per_pair_readout` (`penalty.rs:157`). 3 new sim-soft unit tests in `sim/L0/soft/tests/penalty_interior_cutoff.rs`. Probe removed. |
| CR.2 | `61d8b2fd` | cf-device-design | `intruder_contact_sliding_at` + `run_sliding_insertion_ramp` thread `cavity_inset_m`. 5 call sites updated. Production dispatch passes `design.cavity_inset_m`. |
| CR.3 | `cc8340a2` | cf-device-design | Step-0 contact-force SL.3-regression sentinel (50 N bound) + new `intruder_contact_sliding_at_excludes_deep_interior_probe` fast unit test. Spec deviation: all-16-converge gate on icosphere fixture softened back to Fork-B tolerance — empirical run showed the icosphere genuinely stalls at the Yeoh validity wall around step 3 (max_stretch_deviation = 1.33), confirming v5 fixed the SL.3 mode but the icosphere's narrowing geometry has a separate genuine stall. Docstring rewritten with the two-mechanism story. |

### 10b. CR.4 iter-1 visual gate — numerical-affirmative pass with locality verification deferred to SL.4

Tool launched with `cargo run --release -p cf-device-design -- ~/scans/sock_over_capsule.cleaned.stl`. Sliding-mode Simulate ran to completion in **~minutes** (wall-clock; precise instrumentation deferred). Headline UI panel readout:

| Signal | Pre-v5 (bookmark `f2de9a8d`) | Post-v5 (CR.4 run) |
| ------ | ---------------------------- | ------------------ |
| Step 0 outcome | Armijo stall, `r_norm=4.14, α=4.77e-7, Newton iter 41` | Converged |
| Steps reached | 0 of 16 | **16 of 16** (full 83.35 mm depth) |
| Force curve | n/a | Monotone climb 1.4 N → 3.0 N peak (~step 14), tails to ~1.2 N at step 16 (full seating) |
| Yeoh λ range | n/a | Layer 0 (48,385 tets): `0.88..1.07`; Layer 1 (24,495 tets): `0.98..1.01` — both physical, no validity-wall violations |
| Cavity / min-wall validations | n/a | All green |

The 3.0 N peak force at κ = 1e3 implies ~3 mm interference at peak seating — exactly `cavity_inset_m`. Physics is consistent end-to-end.

**What's verified by CR.4**: the v5 `interior_cutoff` fix eliminates the SL.3 Armijo stall on iter-1; the FEM converges all 16 ramp steps to the full 83.35 mm seating depth; per-step + per-layer engineering scalars are physical; no rendering anomalies in the cavity surface across the step-scrub range.

**What's NOT verified by CR.4 (deferred to SL.4)**: the recon arc shipped on top of SL.3's `update_intruder_mesh` hide-stub (per [[project-cf-device-design-sliding-intruder-bookmark]]). Without the intruder mesh rendered per-step, the spec's "deformation wave propagates from cap mouth to closed end as user scrubs `displayed_step`" check is unverifiable from the cavity-only view alone. Cavity-wall deformation across steps IS subtle on the sock geometry (peak displacement ~mm-scale on an 80 mm body), and without the intruder as a positional reference the eye has nothing to anchor to. SL.4 (rewrite `update_intruder_mesh` from hide-stub to per-step `Transform` writes per the bookmark) is the natural next inflection point and will unblock the full visual locality verification.

**What stays the regression sentinel for the SL.3 mode in CI**: the CR.3 step-0 contact-force ≤ 50 N bound in `sliding_insertion_ramp_converges_on_synthetic_icosphere`. The icosphere fixture observed 0.28 N at step 0 post-v5 (vs kN range pre-v5) — three orders of magnitude below the bound, so the sentinel sharply discriminates regression to the deep-interior firing.

### 10c. Three patterns banked (additions to the spec's §9 insights)

- **Empirical falsification of a spec-level confidence claim is high-information data**, not a problem to paper over. CR.0+'s histogram falsified spec §5's "tens of thousands" pathological-pair prediction (actual: 2.2 k, one order of magnitude smaller) — but the underlying cutoff value remained well-placed, so the empirical data narrowed our understanding without invalidating the fix. CR.3's icosphere all-converge gate falsification told us the synthetic fixture has a separate Yeoh-validity-wall stall mode, requiring us to soften the gate AND keep the SL.3 regression sentinel (the contact-force bound). Both fit the previously-banked [[feedback-implement-measure-revert-pattern]].
- **Visual gates with a stubbed render are only partially conclusive.** SL.3 shipped with `update_intruder_mesh` as a hide-stub, scoped on the assumption that the intruder render would land at SL.4 BEFORE the iter-1 visual gate. The recon arc inserted CR.0+ → CR.4 between SL.3 and SL.4; CR.4's visual locality check is consequently blocked until SL.4 ships. The lesson: when a render dependency is stubbed, downstream visual gates need an explicit dependency arrow OR they need to fall back to numerical-affirmative pass + explicit deferral (this arc's choice).
- **The recon arc's three-session bookmark/recon/implementation cadence held a third time** (after the cavity-pinned-floor arc and the SDF-layers arc). The recon spec converged across 5 iterations + 5 sibling cold-reads to the simplest viable shape (~20 LOC sim-soft addition); the implementation arc ran straight through CR.0+/CR.1/CR.2/CR.3 with one minor spec-deviation handled inline (the CR.3 all-converge gate softening). User involvement was at exactly the right granularity: CR.0+ path choice (β chosen), CR.3 spec deviation surfaced before commit, CR.4 visual-gate disposition. Confirms the cadence as the durable pattern.

---
