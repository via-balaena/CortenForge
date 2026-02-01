# MUJOCO_GAP_ANALYSIS.md Full Correction Plan

## Problem
`docs/MUJOCO_GAP_ANALYSIS.md` (1,986 lines) is actively misleading. The executive summary claims "~100%" completion and lists deleted/stubbed features as "Fully Implemented." The document is self-contradictory: implementation notes say "removed in Phase 3 consolidation" while tables say "Implemented."

18+ false claims identified. The document needs section-by-section correction.

## Approach
- Fix in-place: correct status labels, update executive summary, fix tables
- Do NOT delete implementation notes for removed features (they serve as historical reference showing what was built and why it was removed)
- Change status labels to accurately reflect current state: `Removed`, `Standalone (not in pipeline)`, `Stub`, `Placeholder`
- Update executive summary completion estimate
- Add a note at the top cross-referencing `sim/FUTURE_WORK.md` for the current roadmap

## Execution Steps

Each step targets one section. Verify before writing.

### Step 1: Document Header + Executive Summary (lines 1-44)
- [x] Add cross-reference notice: "For the current roadmap with verified code references, see `sim/FUTURE_WORK.md`."
- [x] Change "~100%" to an honest estimate (~65-70% of core pipeline features)
- [x] Fix "Fully Implemented" list — replaced with 4 categories: Fully Implemented, Placeholder/Stub, Standalone Crates, Removed
- [x] Fix "Recently Completed" table — removed "island-parallel solving" from multi-threading claim
- [x] Updated "feature-complete" line with honest qualification
- [x] Updated Legend table with new status labels (Standalone, Placeholder, Removed)

### Step 2: Section 1 — Integration Methods (lines 58-117)
- [x] Fix table: RK4 → **Placeholder**, ExplicitEuler/VelocityVerlet/ImplicitVelocity/ImplicitFast → **Standalone**
- [x] Added blockquote explaining the two integration systems (pipeline enum vs `integrators.rs` trait)
- [x] Cross-referenced FUTURE_WORK #8 and C1

### Step 3: Section 2 — Constraint Solvers (lines 119-278)
- [x] Fix table: PGS → **Implemented (no SOR)**, Newton → **Removed**, CG → **Standalone**, Islands → **Removed**, Warm starting → kept
- [x] Added blockquote explaining pipeline PGS vs deleted sim-constraint PGS, and CGSolver status
- [x] Updated section headers: Newton → "⚠️ REMOVED", PGS → "⚠️ REMOVED from sim-constraint; reimplemented in pipeline", Islands → "⚠️ REMOVED"
- [x] Kept implementation notes as historical reference with clear removal notices

### Step 4: Section 7 — Actuators (lines 526-625)
- [x] Fixed table: Motor/PositionServo/VelocityServo/PD/Damper → kept **Implemented**; IntegratedVelocity/Pneumatic/Muscle/Adhesion/Custom → **Standalone**
- [x] Added blockquote explaining pipeline actuators vs standalone sim-constraint/sim-muscle actuators
- [x] Cross-referenced FUTURE_WORK #4 and #5

### Step 5: Section 8 — Sensors (lines 626-670)
- [x] Fixed table: Force/Torque → **Stub**, Touch → **Stub**, Rangefinder → **Stub**, Magnetometer → **Missing**, added TendonPos/Vel/ActuatorPos/Vel/SubtreeAngMom as **Missing**
- [x] Added blockquote explaining two sensor systems (sim-sensor standalone vs pipeline stubs)
- [x] Cross-referenced FUTURE_WORK #6 for all 10 stubbed/missing sensors

### Step 6: Section 9 — Tendons (lines 672-768)
- [x] Fixed table: all 4 items → **Standalone** (sim-tendon crate, not in pipeline)
- [x] Added blockquote: sim-tendon 3,919 lines, zero pipeline coupling, Model/Data scaffolds never populated
- [x] Cross-referenced FUTURE_WORK #4

### Step 7: Section 11 — Deformables (lines 848-992)
- [x] Fixed table: all 4 items → **Standalone**, added Deformable-rigid collision as **Missing**
- [x] Added blockquote: sim-deformable 7,733 lines, XPBD not called from `Data::step()`, `ConstraintType::Collision` unimplemented
- [x] Updated both section headers (Skinned Meshes, Deformables) to note "standalone only"
- [x] Cross-referenced FUTURE_WORK #9

### Step 8: Section 12 — Performance Optimizations (lines 994-1163)
- [x] Fixed table: Sparse matrix ops → **Removed**, Constraint islands → **Removed**, Multi-threading → **Partial**
- [x] Updated multi-threading notes header to "⚠️ PARTIALLY REMOVED" with blockquote explaining deletion
- [x] Cross-referenced FUTURE_WORK #10 for batched simulation (future rayon usage)
- [x] SIMD and Sleeping bodies sections kept as-is (accurate)

### Step 9: Priority Roadmap (lines 1367-1956)
- [x] Fix "ALL MAJOR FEATURES COMPLETED" header (line 1369) → "⚠️ Status: Partially Complete"
- [x] Fix Phase 2 (lines 1417-1422): Newton solver and constraint islands annotated as → ⚠️ Removed
- [x] Fix Phase 3 (lines 1424-1429): Muscles, Tendons, Deformables annotated as → ⚠️ Standalone
- [x] Fix Phase 4 (lines 1431-1500): Sparse matrix ops → ⚠️ Removed, ImplicitFast → ⚠️ Standalone
- [x] Fix Phase 9 SDF status: was "❌ TODO" but SDF is implemented → ✅ COMPLETED
- [x] Fix Phase 9 CG solver → ⚠️ Standalone, Multi-threading → ⚠️ Partial
- [x] Added standalone annotations to implementation notes (CG, Tendon, Flex Edge)
- [x] Cross-reference `sim/FUTURE_WORK.md` as the authoritative roadmap (added note after Phase 9)

### Step 10: File Reference + Final Review
- [x] Update File Reference table: fixed sim-core (removed world.rs/stepper.rs/broad_phase.rs, added collision_shape.rs/sdf.rs/mesh.rs/raycast.rs), sim-types (added body.rs/config.rs), sim-urdf (loader.rs→converter.rs), sim-mjcf (added model_builder.rs/defaults.rs/config.rs/mjb.rs), sim-tendon (added cable.rs), sim-deformable (added skinning.rs/material.rs/mesh.rs). Added ⚠️ standalone labels.
- [x] Read full corrected document end-to-end (2,040 lines)
- [x] Verify no remaining false "Implemented" claims: automated grep found 3 remaining false claims in §10 (Tendon coupling, Flex edge) and Phase 9 table — all fixed. Also found §4 SDF row stale ("Missing" → "Implemented ✅") — fixed.

## Files Modified
- `docs/MUJOCO_GAP_ANALYSIS.md` — in-place corrections (only production file changed)
- `sim/docs/GAP_ANALYSIS_CORRECTION_PLAN.md` — this plan file (delete when done)

## Verification
- After all steps: grep for "Implemented" and verify each claim is accurate
- Cross-check against `sim/FUTURE_WORK.md` items 1-11 and cleanup tasks C1-C2
