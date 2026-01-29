# MuJoCo Parity Implementation Specification

> **Status**: ACTIVE — Roadmap for achieving MuJoCo feature parity.
>
> **Todorov Standard**: Single source of truth. No duplication. O(n) where possible.
> Compute once, use everywhere. Profile before optimizing.
>
> **Last Updated**: 2026-01-28

---

## Executive Summary

This document specifies the implementation roadmap for addressing identified weak spots
in the `sim` codebase compared to MuJoCo. Tasks are ordered by **foundational dependency**:
earlier phases unlock later phases.

### What's Already Working

Before diving into gaps, note what's **already implemented**:

| Feature | Status | Location |
|---------|--------|----------|
| Joint stiffness storage | ✅ `Model.jnt_stiffness` | `mujoco_pipeline.rs:5672` |
| Joint stiffness in passive forces | ✅ `mj_passive()` | `mujoco_pipeline.rs:10654` |
| Joint damping | ✅ Full | `mujoco_pipeline.rs:10657` |
| Joint armature | ✅ Storage + CRBA | `mujoco_pipeline.rs:10266` |
| Equality constraint parsing | ✅ MJCF parser | `parser.rs:1103` |
| ConnectConstraint type | ✅ Full API | `equality.rs:1411` |
| Implicit integrators | ✅ With damping | `integrators.rs:296-453` |
| Warm-starting (PGS/CG/Newton) | ✅ Full | `pgs.rs:463`, `cg.rs:424` |
| `dof_frictionloss` field | ⚠️ Exists but unused | `mujoco_pipeline.rs:5693` |

### Priority Classification

| Priority | Criteria | Example |
|----------|----------|---------|
| **P0** | Blocks other work; architectural foundation | Spring reference position |
| **P1** | Significant capability gap; MuJoCo conformance | Kinematic loop integration |
| **P2** | Quality/robustness improvement | `unwrap()` elimination |
| **P3** | Nice to have; polish | Additional test coverage |

---

## Phase 0: Foundation (P0)

> **Principle**: Fix gaps in existing infrastructure before adding new features.

---

### 0.1 Spring Reference Position (`springref`)

**Status**: PARSED BUT NOT APPLIED

**Problem**: MuJoCo uses `spring_ref` as the spring equilibrium position, but our
implementation uses `qpos0` (the initial joint position) instead.

**Current State**:

```rust
// sim/L0/mjcf/src/types.rs:984
pub struct MjcfJoint {
    pub spring_ref: f64,  // ✅ Parsed from MJCF
    // ...
}

// sim/L0/core/src/mujoco_pipeline.rs:10652
// ❌ Uses qpos0 instead of spring_ref
let q0 = model.qpos0.get(qpos_adr).copied().unwrap_or(0.0);
data.qfrc_passive[dof_adr] -= stiffness * (q - q0);  // Should be (q - spring_ref)
```

**MuJoCo Semantics**:
- `qpos0`: Initial joint position at model load (for `mj_resetData()`)
- `springref`: Spring equilibrium position (for passive spring force)
- These are distinct: a joint can start at 0° but have a spring pulling toward 30°

**Implementation**:

1. **Add `jnt_springref` to Model**:

   ```rust
   // mujoco_pipeline.rs - Model struct
   pub struct Model {
       // Existing:
       pub jnt_stiffness: Vec<f64>,
       pub jnt_damping: Vec<f64>,

       // NEW:
       /// Per-joint spring reference position (rad or m).
       /// Spring force: F = -k * (q - springref)
       pub jnt_springref: Vec<f64>,
   }
   ```

2. **Populate from MJCF**:

   ```rust
   // model_builder.rs
   self.jnt_springref.push(joint.spring_ref);
   ```

3. **Use in mj_passive()**:

   ```rust
   // mujoco_pipeline.rs:10652
   let springref = model.jnt_springref[jnt_id];
   let q = data.qpos[qpos_adr];
   data.qfrc_passive[dof_adr] -= stiffness * (q - springref);
   ```

**Tests Required**:

- [ ] `spring_ref != 0` shifts equilibrium correctly
- [ ] `spring_ref == qpos0` produces same behavior as current code
- [ ] Spring oscillates around `spring_ref`, not `qpos0`

**Files Changed**:

- `sim/L0/core/src/mujoco_pipeline.rs` — Add field, use in `mj_passive()`
- `sim/L0/mjcf/src/model_builder.rs` — Populate field

**Effort**: 2-3 hours

---

### 0.2 Joint Friction Loss

**Status**: PARSED BUT NOT APPLIED

**Problem**: `frictionloss` is parsed from MJCF but not stored or applied.

**MuJoCo Semantics**: Friction loss is Coulomb friction at the joint:
- Force = `frictionloss * sign(velocity)` opposing motion
- Provides velocity-independent resistance

**Implementation**:

1. **Add `jnt_frictionloss` to Model**:

   ```rust
   pub jnt_frictionloss: Vec<f64>,
   ```

2. **Apply in mj_passive()**:

   ```rust
   // After damping
   let frictionloss = model.jnt_frictionloss[jnt_id];
   if frictionloss > 0.0 {
       let vel = data.qvel[dof_adr];
       // Smooth approximation of sign() to avoid discontinuity
       let friction_force = -frictionloss * vel.signum() * (1.0 - (-vel.abs() * 100.0).exp());
       data.qfrc_passive[dof_adr] += friction_force;
   }
   ```

**Files Changed**:

- `sim/L0/core/src/mujoco_pipeline.rs` — Add field, apply in `mj_passive()`
- `sim/L0/mjcf/src/model_builder.rs` — Populate field

**Effort**: 2 hours

---

### 0.3 Constraint Solver Stiffness Clarification

**Status**: CORRECTLY HARDCODED (NOT A BUG)

**Important Clarification**: The hardcoded `10000.0` in `solver.rs` is for **positional
constraint stabilization** (Baumgarte stabilization), NOT joint spring stiffness.

```rust
// solver.rs:357 — This is CORRECT for constraint stabilization
let stiffness = 10000.0;  // Penalty stiffness for constraint violation
let damping = 1000.0;     // Damping for constraint velocity
```

**These are different concepts**:

| Concept | Purpose | Location | Source |
|---------|---------|----------|--------|
| **Joint spring stiffness** | Passive spring force | `mj_passive()` | `jnt_stiffness` from MJCF |
| **Constraint stabilization** | Prevent joint drift | `solver.rs` | Internal penalty |

**The constraint solver penalty is intentionally high** — it prevents position drift
at joint anchors. This is not meant to be user-configurable in the same way as
joint spring stiffness.

**However**, the constraint solver could benefit from:
- Using `config.baumgarte_factor` more consistently
- Making penalty stiffness configurable in `ConstraintSolverConfig`

**Low Priority**: This is working as designed. Consider cleanup in Phase 3.

---

## Phase 1: Core Capabilities (P1)

> **Principle**: Build major features that expand what the simulator can do.

---

### 1.1 Equality Constraint Integration

**Status**: PARSED BUT NOT APPLIED

**Problem**: Equality constraints (`<connect>`, `<weld>`, etc.) are fully parsed
from MJCF and stored in `MjcfModel.equality`, but they are **never converted**
to runtime Model fields.

**Current State**:

```rust
// MJCF Parsing: ✅ COMPLETE
// sim/L0/mjcf/src/parser.rs:1103
fn parse_equality<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfEquality> {
    // Parses <connect>, <weld>, <joint>, <distance>
}

// Model Storage: ✅ FIELDS EXIST
// sim/L0/core/src/mujoco_pipeline.rs:5836
pub neq: usize,
pub eq_type: Vec<EqualityType>,
pub eq_obj1id: Vec<usize>,
// ...

// Conversion: ❌ MISSING
// sim/L0/mjcf/src/model_builder.rs:1127
// Equality constraints (empty for now - will be populated from MJCF equality definitions)
eq_type: vec![],  // Always empty!

// Constraint Types: ✅ EXIST
// sim/L0/constraint/src/equality.rs
pub struct ConnectConstraint { ... }  // Full implementation with tests
```

**Implementation**:

1. **Convert MJCF equality constraints in model_builder.rs**:

   ```rust
   // model_builder.rs - new function
   fn convert_equality_constraints(
       mjcf: &MjcfModel,
       body_name_to_id: &HashMap<String, usize>,
       joint_name_to_id: &HashMap<String, usize>,
   ) -> EqualityConstraints {
       let mut eq = EqualityConstraints::default();

       // Convert connect constraints
       for connect in &mjcf.equality.connects {
           let body1_id = body_name_to_id.get(&connect.body1)
               .expect("connect body1 not found");
           let body2_id = connect.body2.as_ref()
               .map(|name| body_name_to_id.get(name).expect("connect body2 not found"));

           eq.eq_type.push(EqualityType::Connect);
           eq.eq_obj1id.push(*body1_id);
           eq.eq_obj2id.push(body2_id.copied().unwrap_or(0));
           eq.eq_data.push(connect_to_data(connect));
           eq.eq_active.push(connect.active);
           // ...
       }

       // Convert weld, joint, distance constraints similarly...
       eq
   }
   ```

2. **Apply equality constraints in step()**:

   ```rust
   // mujoco_pipeline.rs - step() or mj_fwdConstraint()
   fn apply_equality_constraints(model: &Model, data: &mut Data) {
       for i in 0..model.neq {
           if !model.eq_active[i] {
               continue;
           }

           match model.eq_type[i] {
               EqualityType::Connect => {
                   apply_connect_constraint(model, data, i);
               }
               EqualityType::Weld => {
                   apply_weld_constraint(model, data, i);
               }
               // ...
           }
       }
   }
   ```

3. **Use existing ConnectConstraint for force computation**:

   The `ConnectConstraint::compute_error()` and related methods in `equality.rs`
   already implement the math — just need to wire them up.

**Tests Required**:

- [ ] MJCF with `<connect>` produces non-empty `eq_type`
- [ ] Connect constraint maintains anchor coincidence
- [ ] Weld constraint maintains relative pose
- [ ] Loop closure (4-bar linkage) works

**Files Changed**:

- `sim/L0/mjcf/src/model_builder.rs` — Convert MJCF equality to Model
- `sim/L0/core/src/mujoco_pipeline.rs` — Apply constraints in step()

**Effort**: 16-24 hours

---

### 1.2 Implicit Integration with Stiffness

**Status**: DAMPING ONLY

**Problem**: The implicit integrator handles damping but not stiffness:

```rust
// integrators.rs:322
// v_{t+h} = (v_t + h * a) / (1 + h * d)
// ✅ Handles damping (d) implicitly
// ❌ Does not handle stiffness (k) implicitly
```

**MuJoCo's implicit integrator** solves:
```
M * (v_new - v_old) = h * (f - d*v_new - k*(x_new - x_eq))
```

This requires treating spring forces implicitly for stability with high stiffness.

**Implementation**:

```rust
/// Integrate with implicit spring-damper treatment.
///
/// Solves: (M + h*D + h²*K) * v_new = M*v + h*f - h*K*(x - x_eq)
pub fn integrate_implicit_spring_damper(
    state: &mut RigidBodyState,
    external_accel: Vector3<f64>,
    equilibrium: Point3<f64>,
    stiffness: f64,
    damping: f64,
    mass: f64,
    dt: f64,
) {
    // Effective mass includes spring and damper contributions
    let effective_mass = mass + dt * damping + dt * dt * stiffness;

    // Spring force toward equilibrium
    let displacement = state.pose.position - equilibrium;
    let spring_force = -stiffness * displacement;

    // Implicit solve
    let impulse = mass * state.twist.linear + dt * (external_accel * mass + spring_force);
    state.twist.linear = impulse / effective_mass;

    // Position update
    state.pose.position += state.twist.linear * dt;
}
```

**Note**: This is for single-body implicit treatment. Full joint-space implicit
integration is more complex and may not be needed for most use cases.

**Effort**: 8-12 hours

---

### 1.3 Warm-Starting for Constraint Solver

**Status**: ✅ IMPLEMENTED

**Current State**:

```rust
// pgs.rs:91
pub warm_starting: bool,  // ✅ Config

// pgs.rs:463-476 — Warm-start initialization
let warm_started = self.config.warm_starting
    && self.cached_lambda.len() == total_constraint_rows;

let mut lambda = if warm_started {
    DVector::from_iterator(
        total_constraint_rows,
        self.cached_lambda.iter().map(|&v| v * self.config.warm_start_factor),
    )
} else {
    DVector::zeros(total_constraint_rows)
};
```

**Implemented in**:
- `pgs.rs` — PGS solver with cached lambda
- `cg.rs` — Conjugate gradient solver
- `newton.rs` — Newton solver
- `contact/solver.rs` — Contact solver with cached forces
- `mujoco_pipeline.rs:4711` — Contact correspondence warm-start

**No further work needed** — warm-starting is fully functional.

---

## Phase 2: Robustness (P2)

> **Principle**: Eliminate defensive `unwrap()` calls that can panic in edge cases.

---

### 2.1 SDF Collision Robustness

**Location**: `sim/L0/core/src/sdf.rs`
**Count**: 52 `unwrap()` calls

**Strategy**: Convert to safe defaults for degenerate cases.

**Effort**: 12-16 hours

---

### 2.2 Heightfield Robustness

**Location**: `sim/L0/core/src/heightfield.rs`
**Count**: 21 `unwrap()` calls

**Strategy**: Use `get()` with fallbacks for out-of-bounds queries.

**Effort**: 8-10 hours

---

### 2.3 Contact Robustness

**Location**: `sim/L0/contact/src/contact.rs`
**Count**: 2 `unwrap()` calls (not 14 as originally estimated)

**Strategy**: Minimal changes needed — review for safety.

**Effort**: 1-2 hours

---

## Phase 3: Test Coverage (P3)

> **Principle**: Tests are documentation. Missing tests indicate missing guarantees.

---

### 3.1 Spring Reference Tests

```rust
#[test]
fn springref_shifts_equilibrium() {
    let xml = r#"
        <mujoco>
          <worldbody>
            <body name="pendulum">
              <joint name="j1" type="hinge" stiffness="100" springref="0.5"/>
              <geom type="capsule" size="0.1 0.5"/>
            </body>
          </worldbody>
        </mujoco>
    "#;

    let model = load_model(xml).unwrap();
    let mut data = model.make_data();

    // Start at q=0, spring pulls toward springref=0.5
    assert!(data.qpos[0].abs() < 1e-6);

    // Let it settle
    for _ in 0..1000 {
        data.step(&model);
    }

    // Should settle near springref, not qpos0
    assert!((data.qpos[0] - 0.5).abs() < 0.01);
}
```

---

### 3.2 Equality Constraint Tests

```rust
#[test]
fn connect_constraint_closes_loop() {
    let xml = r#"
        <mujoco>
          <worldbody>
            <body name="link1">
              <joint type="hinge" axis="0 0 1"/>
              <geom type="capsule" fromto="0 0 0  1 0 0" size="0.1"/>
              <body name="link2" pos="1 0 0">
                <joint type="hinge" axis="0 0 1"/>
                <geom type="capsule" fromto="0 0 0  1 0 0" size="0.1"/>
              </body>
            </body>
          </worldbody>
          <equality>
            <connect body1="link2" anchor="1 0 0"/>
          </equality>
        </mujoco>
    "#;

    let model = load_model(xml).unwrap();
    assert_eq!(model.neq, 1);

    let mut data = model.make_data();
    data.step(&model);

    // Endpoint should stay at world origin
    // (testing that connect constraint is enforced)
}
```

---

## Implementation Checklist

### Phase 0: Foundation

- [ ] **0.1** Add `jnt_springref` to Model
- [ ] **0.1** Populate `jnt_springref` from MJCF
- [ ] **0.1** Use `jnt_springref` in `mj_passive()` instead of `qpos0`
- [ ] **0.2** Populate `dof_frictionloss` from MJCF `joint.frictionloss`
- [ ] **0.2** Apply friction loss in `mj_passive()`

### Phase 1: Core Capabilities

- [ ] **1.1** Convert MJCF equality constraints to Model fields
- [ ] **1.1** Apply equality constraints in `step()` / constraint solver
- [ ] **1.1** Test 4-bar linkage and loop closure
- [ ] **1.2** Add implicit spring-damper integrator
- [x] **1.3** ~~Implement warm-starting in PGS solver~~ (Already implemented)

### Phase 2: Robustness

- [ ] **2.1** Eliminate unwraps in `sdf.rs` (52 occurrences)
- [ ] **2.2** Eliminate unwraps in `heightfield.rs` (21 occurrences)
- [ ] **2.3** Review unwraps in `contact.rs` (2 occurrences)

### Phase 3: Test Coverage

- [ ] **3.1** Spring reference equilibrium tests
- [ ] **3.2** Equality constraint loop closure tests
- [ ] **3.3** Implicit integrator accuracy tests

---

## File Reference Map

| Component | Primary File | Key Lines |
|-----------|-------------|-----------|
| Joint stiffness (storage) | `mujoco_pipeline.rs` | 5672 |
| Joint stiffness (applied) | `mujoco_pipeline.rs` | 10654 |
| Joint damping | `mujoco_pipeline.rs` | 10657 |
| Joint armature | `mujoco_pipeline.rs` | 10266 |
| Equality parsing | `parser.rs` | 1103 |
| Equality types | `types.rs` | 1495 |
| ConnectConstraint | `equality.rs` | 1411 |
| Model eq_* fields | `mujoco_pipeline.rs` | 5836-5858 |
| Implicit integrator | `integrators.rs` | 296-453 |
| Constraint solver | `solver.rs` | 340-713 |
| SDF collision | `sdf.rs` | * |
| Heightfield | `heightfield.rs` | * |

---

## Success Criteria

### Functional

- [ ] `springref` shifts spring equilibrium (not using `qpos0`)
- [ ] `frictionloss` produces velocity-independent resistance
- [ ] Equality constraints (`<connect>`) close kinematic loops
- [ ] Warm-starting reduces PGS iterations by ≥30%

### Robustness

- [ ] No `unwrap()` in hot paths (`sdf.rs`, `heightfield.rs`)
- [ ] Degenerate inputs produce sensible defaults (no panic)

### Quality

- [ ] All new code has tests
- [ ] Single source of truth for joint properties

---

## Appendix A: What's NOT a Bug

### Constraint Solver Penalty Stiffness

The hardcoded `10000.0` in `solver.rs` is **intentional**:

```rust
// solver.rs:357
let stiffness = 10000.0;  // Penalty for constraint violation
```

This is **positional constraint stabilization**, not joint spring stiffness.
It prevents drift at joint anchors. Making this configurable is low priority
and would only matter for very specific use cases (soft constraints).

### Joint Stiffness in Passive Forces

Joint stiffness **IS** already applied:

```rust
// mujoco_pipeline.rs:10654
data.qfrc_passive[dof_adr] -= stiffness * (q - q0);
```

The only issue is using `qpos0` instead of `springref` for the equilibrium.

---

## Appendix B: MuJoCo Feature Comparison (Updated)

| Feature | MuJoCo | sim | Gap |
|---------|--------|-----|-----|
| Joint stiffness | ✅ Full | ✅ Applied | — |
| Spring reference | ✅ Full | ⚠️ Uses qpos0 | Phase 0.1 |
| Joint damping | ✅ Full | ✅ Full | — |
| Joint armature | ✅ Full | ✅ Full | — |
| Friction loss | ✅ Full | ⚠️ Field exists, not populated/applied | Phase 0.2 |
| Kinematic loops | ✅ Full | ⚠️ Parsed, not applied | Phase 1.1 |
| Equality (connect) | ✅ Full | ⚠️ Parsed, not applied | Phase 1.1 |
| Equality (weld) | ✅ Full | ⚠️ Parsed, not applied | Phase 1.1 |
| Warm-starting | ✅ Full | ✅ Full (PGS/CG/Newton) | — |
| Implicit integration | ✅ Full | ⚠️ Damping only | Phase 1.2 |
| Soft contacts | ✅ Full | ✅ Full | — |
| Friction | ✅ Full | ✅ Full | — |
| Mesh collision | ✅ Full | ✅ Full | — |
| Sensors | ✅ Full | ✅ Full | — |
| Muscles | ✅ Full | ✅ Full | — |
| Tendons | ✅ Full | ✅ Full | — |

---

*End of Specification*
