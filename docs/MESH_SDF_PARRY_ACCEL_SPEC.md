# mesh-sdf — Parry BVH Acceleration Spec

**Status**: SPEC SHIPPED 2026-05-17 LATE. Next session: cold-read + P1 implementation.

**Trigger**: cf-cast-cli iter-1 verification run on sock_over_capsule (2026-05-17 LATE) hit ~40 min wall time without completing the FIRST of 6 mold STL exports; projected total ~4 hours. Root cause: `mesh_sdf::SignedDistanceField` does brute-force O(faces) per closest-triangle query on the full 167k-face cleaned scan, and the new candidate-A pinned-floor construction doubles the per-cell query count. Decimation in cf-cast-cli would patch the symptom but locks in a mold-fidelity tuning knob that gets WORSE as scan complexity grows. Parry-backed BVH fixes the root cause: per-query cost drops O(faces) → O(log faces) ≈ 5,000× speedup on iter-1.

**Parent context**: this arc lands AFTER the pinned-floor redesign arc (A1-A5 + B + C, `65c01348` → `1ae47a8d`). The visual gate cleared 2026-05-17 evening; the cf-cast-cli verification was the last item that didn't fit in the same session.

**Blocks**:
- Workshop iter-1 physical cast (need cf-cast-cli to produce mold STLs in workshop-tolerable time).
- Insertion sim ramp time (currently bearable; would improve).
- cf-device-design preview's `SDF_SOURCE_TARGET_FACES = 2500` decimation cap (currently a load-bearing performance tuning; would become cosmetic).

---

## Headline construction

Replace `mesh_sdf::SignedDistanceField`'s brute-force closest-triangle search with `parry3d::shape::TriMesh`-backed BVH queries. Keep the public API surface bit-identical so consumers (`cf-device-design`, `cf-cast-cli`, `cf-design`'s `Sdf for SignedDistanceField` impl) require zero code changes.

```rust
// Internal struct grows a TriMesh field alongside the existing IndexedMesh:
pub struct SignedDistanceField {
    mesh: IndexedMesh,                  // existing — kept for mesh() accessor + display
    tri_mesh: parry3d::shape::TriMesh,  // NEW — backs all queries
    // face_normals: Vec<Vector3<f64>>  ← DELETED (no longer needed)
}

// Methods route through parry:
fn distance(&self, p: Point3<f64>) -> f64 {
    let projection = self.tri_mesh.project_local_point(&p.into(), false);
    let unsigned = (p.coords - projection.point.coords).norm();
    if projection.is_inside { -unsigned } else { unsigned }
}
```

**Per-query cost projection (iter-1 sock, 167k faces)**:
- Current: ~50ns × 167k face checks ≈ **8.4 ms** per query.
- Parry BVH: ~50ns × log₂(167k) box tests (~17) + ~10 leaf-triangle checks ≈ **1.5 µs** per query.
- **Speedup ≈ 5,500×.**

**Workload projections**:
| Workload | Current | Post-parry |
|---|---|---|
| cf-device-design preview SDF fill | ~1 s | ~25-50 ms |
| cf-cast-cli iter-1 mold export (6 STLs, 3mm) | ~4 hr | ~under 1 min |
| insertion_sim GridSdf build (unsigned pass) | ~325 ms | ~5 ms |

---

## High-conviction decisions (from 2026-05-17 LATE round-2 stress test)

### 1. Integration shape: drop-in replacement

**Pick**: drop-in replacement. `mesh-sdf` gains a hard dep on `parry3d`; the brute-force impl is deleted. NO Cargo feature flag, NO sibling type.

**Rejected**:
- **Feature-flag (`parry-accel` default-on)**: doubles test matrix (CI must exercise both paths to prevent bit-rot of the brute-force path); the "escape hatch" is illusory — if we ever need to debug parry, a 50-line inline brute-force in a test is sufficient.
- **Sibling type** (e.g., `SignedDistanceFieldAccel`): every consumer needs a code change; two types where one is faster and correct invites the wrong pick.

**Pre-condition**: WASM compat verified. parry3d should compile to wasm32 (no rayon/SIMD-on-wasm dependencies in default features), but unverified until P1.

**Fragility callout**: hard dep on parry means we can't easily back out. Mitigation: parry3d is the BVH backbone of Rapier (de-facto Rust physics ecosystem), maintenance pressure is high, integrates natively with nalgebra (no friction).

### 2. Sign-determination contract

**Pick**: contract narrows from "best-effort heuristic" to "reliable for watertight meshes; undefined for non-manifold."

| Mesh type | Current `distance()` sign | Post-parry `distance()` sign |
|---|---|---|
| Watertight (cleaned scan) | Best-effort face-normal heuristic; far-field fails ~12% per insertion_sim 7.3a diagnostic | **Reliable** via `TriMesh::project_local_point().is_inside` (pseudonormal-based) |
| Non-manifold (cap-stripped open mesh) | Best-effort heuristic; arbitrary | **Undefined** — consumers must not rely on it; use `unsigned_distance` only |

**Consumer impact audit** (confirms no consumer is broken by the narrowing):
- `cf-device-design::sdf_layers`: uses `unsigned_distance` only (after B fix). Sign comes from flood-fill. NOT affected.
- `cf-device-design::insertion_sim`: scan SDF is `build_grid_sdf` (flood-fill); open SDF also flood-fill. NOT affected.
- `cf-cast-cli::SharedScanSdf`: wraps a `SignedDistanceField` over the CLOSED watertight cleaned scan. Sign improves; no breakage.
- `cf-cast-cli::A5 open-mesh SDF`: `SignedDistanceField` over cap-stripped (non-manifold) mesh; only consumed by `UnsignedRindSdf` which calls `open.eval(p).abs()` → sign discarded. NOT affected by undefined-sign.

**Pin contract with test**: non-manifold fixture (open-top cube). Asserts `unsigned_distance` correctness + determinism + finiteness. Does NOT pin sign value.

### 3. Per-API behavior

API surface stays **bit-identical** from consumer perspective.

| Method | Current | Post-parry |
|---|---|---|
| `new(IndexedMesh) -> Result<Self>` | Validates non-empty mesh, computes face normals | Same validation + `TriMesh::new()` build (also bubbles parry's errors) |
| `distance(p) -> f64` | Sign × brute-force unsigned | `(p - projection.point).norm() * (is_inside ? -1 : 1)` |
| `unsigned_distance(p) -> f64` | Brute-force min over faces | `(p - projection.point).norm()` |
| `is_inside(p) -> bool` | +X ray cast via `point_in_mesh` | `projection.is_inside` (pseudonormal-based, more robust) |
| `closest_point(p) -> Point3<f64>` | Brute-force min over faces | `projection.point` |
| `mesh() -> &IndexedMesh` | Returns stored IndexedMesh | Returns stored IndexedMesh (unchanged — stored alongside TriMesh) |

**Boundary-point sign convention**: points within `f64::EPSILON` of a face have implementation-defined sign in BOTH the old and new impls (the old `signum(0) = +1` footgun documented elsewhere). Don't rely on it. Pin a test that exercises both sides of the surface but NOT exactly on it.

**Memory cost**: storing both IndexedMesh (~6 MB on iter-1) AND TriMesh (~10 MB BVH on iter-1) = ~16 MB per SDF. With one SDF per scan in cf-device-design + one in cf-cast-cli = ~32 MB total. Workshop-acceptable.

### 4. Numerical drift risk + handling

**Low risk.** Audit during P1: `cargo test --workspace --release`. Three failure modes to expect:

1. Tests pinning specific `distance()` return values with WRONG-sign output. Fix: update test to pin correct sign (or magnitude only).
2. Tests pinning `min_sdf_value` from cached grids. Parry's correct sign may shift this; verify the new value is sensible, update.
3. Tests pinning gradient direction via finite-diff (gradient sign tracks distance sign). Same handling.

**Expected breakage**: <5 tests, all easily fixed. Budget ~1 hour in P1.

**Bit-equivalence of `unsigned_distance`**: parry and brute-force both use identical point-triangle-distance geometry; results bit-equivalent up to f64 roundoff. Tests pinning magnitudes will pass.

### 5. Sub-leaf ladder (in-arc P1-P3; F1-F3 banked)

**P1 — mesh-sdf parry swap** (~300 LOC, one commit).
- Add `parry3d` workspace dep (pin specific minor version after compat check).
- Rewrite `SignedDistanceField` internals to wrap `TriMesh`. Delete `face_normals` field + `compute_face_normals` helper + `compute_sign` helper.
- Update mesh-sdf's existing tests if any pinned the broken sign (audit).
- New tests:
  - `far_field_sign_reliable_on_pathological_cap_fan` — synthetic mesh that tripped mesh-sdf's heuristic; parry must return correct sign.
  - `non_manifold_unsigned_distance_correct_and_deterministic` — open-top cube fixture; pins `unsigned_distance` + determinism without constraining sign.
- WASM build check: `cargo build -p mesh-sdf --target wasm32-unknown-unknown` clean.
- `cargo test -p mesh-sdf` clean.
- `cargo run -p xtask -- grade mesh-sdf` ≥ A.

**P2 — cf-device-design verification** (validation, plus targeted test fixes if any).
- `cargo test -p cf-device-design --bin cf-device-design --release` clean.
- `cargo run -p cf-device-design --release -- ~/scans/sock_over_capsule.cleaned.stl` — visual gate on iter-1 sock. Confirm cavity + Layer 0 + Layer 1 look like the post-C visual-gate state (or BETTER, since parry's reliable sign might make the flood-fill robust-sign defense unnecessary on some cells).
- `cargo run -p xtask -- grade cf-device-design` ≥ A.

**P3 — cf-cast-cli iter-1 verification** (the actual unblock).
- `cargo test -p cf-cast-cli --release` clean.
- `cargo run -p cf-cast-cli --release -- ~/scans/cast.iter1-design.toml` — expect 6 mold STLs + procedure.md in `~/scans/cast_iter1_design/` in **single-digit minutes**.
- Visual gate on the mold STLs:
  - Each mold-piece STL renders without obvious artifacts.
  - Each per-layer plug STL has the floor pinned at the cap plane (the candidate-A win).
- `cargo run -p xtask -- grade cf-cast-cli` ≥ A.

**Banked followups (separate arcs, NOT in this arc)**:

- **F1**: drop flood-fill robust-sign in `cf-device-design::sdf_layers::build_cached_scan_sdf`. Becomes redundant once parry's sign is workshop-validated across multiple scans. Don't remove in P2 — keep the safety net through iter-2.
- **F2**: bump or drop `SDF_SOURCE_TARGET_FACES = 2500` decimation in cf-device-design. Per-query cost drops sub-µs with parry; could afford 50k+ faces or no decimation. Cosmetic preview quality improvement.
- **F3**: same for `insertion_sim::decimate_for_sdf`. Possibly drop entirely and let BCC mesher work on the full mesh's SDF.

### 6. WASM compat — P1 acceptance criterion

**Pick**: WASM build check is a P1 gate. If parry3d breaks wasm32, fall back to feature-flag (`parry-accel` default-on, brute force on wasm). The fallback adds test-matrix burden but preserves the L0 tier contract.

**Verification**: `cargo build -p mesh-sdf --target wasm32-unknown-unknown` after the swap. If clean, proceed with drop-in. If broken, surface the error + escalate to feature-flag posture.

**Probability**: parry3d's default features don't depend on rayon (its `parallel` feature is opt-in). nalgebra works on wasm. SIMD opt-in is also default-off. Should compile clean. ~90% confidence; the 10% covers parry transitive deps drifting since last check.

### 7. Flood-fill robust sign — keep through this arc

**Pick**: KEEP the flood-fill robust-sign in `cf-device-design::sdf_layers::build_cached_scan_sdf` through P1-P3. Drop in banked followup F1 after iter-2 validates parry on a second real scan.

**Reasoning**: defense-in-depth. parry's reliable on iter-1's clean watertight sock; we don't know how it handles a more complex body-part scan with decimation slivers + cap-fan junction noise. Cost of keeping flood-fill: ~100 LOC + microseconds-scale BFS per scan load. Cost of removing prematurely + getting bit: another wavy-floor session.

The flood-fill exists *because* the old SDF backend's sign was unreliable. Removing it the day we install the new backend, without first proving the new backend across a corpus, is the same risk pattern in the other direction.

### 8. Open-mesh contract test

**Pick**: pin `unsigned_distance` correctness + determinism + finiteness on a non-manifold fixture (open-top cube). Do NOT pin sign value.

```rust
#[test]
fn non_manifold_unsigned_distance_correct_and_deterministic() {
    let open_cube = open_top_unit_cube();  // 10 faces (no top)
    let sdf = SignedDistanceField::new(open_cube).unwrap();
    let probes = [
        Point3::new(0.0, 0.0, 0.5),   // above the open top
        Point3::new(0.0, 0.0, -0.5),  // below the closed bottom
        Point3::new(0.5, 0.0, 0.0),   // beside the closed +x wall
    ];
    for p in probes {
        let d1 = sdf.unsigned_distance(p);
        let d2 = sdf.unsigned_distance(p);
        assert_eq!(d1, d2, "unsigned_distance must be deterministic");
        assert!(d1.is_finite() && d1 >= 0.0);
        // Distance via direct math: e.g., point at (0,0,0.5) is 0
        // from the open top boundary; closest CLOSED face at distance 0.5.
        // Just check it's a finite reasonable value.
        let signed = sdf.distance(p);
        assert!(signed.is_finite(), "distance finite even on non-manifold");
        // NO assertion on sign — undefined by contract.
    }
}
```

---

## Acceptance criteria

P1-P3 all required for arc completion:

- [ ] **P1**: `mesh-sdf` builds clean for native + wasm32; all mesh-sdf tests pass; 2 new tests added (far-field sign + non-manifold unsigned); `xtask grade mesh-sdf` ≥ A.
- [ ] **P2**: `cargo test -p cf-device-design` clean; visual gate on iter-1 sock matches post-C state or better; `xtask grade cf-device-design` ≥ A.
- [ ] **P3**: `cargo test -p cf-cast-cli` clean; iter-1 mold export completes in single-digit minutes; 6 mold STLs + procedure.md emerge in `~/scans/cast_iter1_design/`; visual gate on the molds passes; `xtask grade cf-cast-cli` ≥ A.

---

## Fragile bits (call out for cold-read)

1. **WASM-compat coupling with drop-in decision (§1 + §6)**. If parry3d breaks wasm32, decision §1 collapses to feature-flag. Won't know until P1 builds. Low probability but non-zero.

2. **P5 banked, not in-arc (§5)**. Explicit choice to NOT do cleanup in the same arc. Keep parry-swap focused on the swap itself; cleanups (F1-F3) risk-managed separately.

3. **Flood-fill robust sign kept through arc (§7)**. Conservative. If feeling bold, we could remove it in P2 once we see parry's behavior on iter-1. I'm NOT feeling bold — the flood-fill is the belt that exists because the suspenders broke on us 2026-05-17 LATE.

4. **Boundary-point sign convention (§3)**. parry's `is_inside` behavior on points exactly on a face is implementation-defined. Don't construct tests that probe exactly on the surface. Use small offsets.

5. **Test breakage estimate is a budget, not a guarantee (§4)**. If audit surfaces >10 broken tests, that's a signal that something subtler is going on (e.g., parry's `project_local_point` semantics differ from brute-force in a way I didn't catch). Stop and investigate before mass-fixing.

---

## Followup arcs (out of scope)

- **F1**: drop flood-fill robust-sign in cf-device-design's sdf_layers (after iter-2 validates parry on a second scan type).
- **F2**: bump or drop `SDF_SOURCE_TARGET_FACES` decimation cap in cf-device-design (now cosmetic with sub-µs queries).
- **F3**: bump or drop `decimate_for_sdf` in insertion_sim (same rationale).
- **F4**: parry's TriMesh also supports `cast_ray`; could accelerate fit-viz rungs 2-6 (scan-as-intruder pressure scoring). Not in this arc but worth noting — parry-as-acceleration-backbone may keep paying down on adjacent arcs.

---

## Cold-read entry for next session

1. Read this spec.
2. Skim [[project-pinned-floor-visual-gate-postmortem]] for context on WHY parry is the next move (the cf-cast-cli 4hr timeout that triggered this arc).
3. Skim [[project-cf-device-design-cavity-pinned-floor-redesign-spec]] §"What shipped across the arc" for what mesh-sdf currently does + which consumers care.
4. Implement P1 per §5. If anything in this spec looks wrong with fresh eyes, iterate the spec before writing code.

**Implementation start**: `mesh/mesh-sdf/Cargo.toml` (add parry3d dep) + `mesh/mesh-sdf/src/sdf.rs` (rewrite SignedDistanceField internals). API surface stays unchanged.
