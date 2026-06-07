# Architecture sketch — the anatomical library & the parameter interface

*2026-06-07. Design sketch for the parametric-builder-first reframe (`01_vision/vision.md`).
Not yet a recon; this fixes the seams and crate boundaries so the G1 work lands in the right shape.*

## The one idea

**Parameters morph an anatomical IR; MJCF is emitted from the morphed IR.** The raw scan mesh is
never the thing we deform (that is tier-(c), rejected in the vision). A scan — when present —
only *produces a parameter vector*, exactly like a domain randomizer or the canonical defaults do.
Where the parameters come from is **pluggable**; the body builder doesn't know or care.

```
  ┌─────────────────────────┐
  │   Anatomical Library    │   BodyTemplate (canonical knee → leg → body),
  │   (IR + scaling rules)   │   seeded from the validated OpenSim corpus
  └────────────┬────────────┘
               │ template
               ▼
  ParamSource ──► BodyParams ──► realize(template, params) ──► BodySpec ──► emit ──► MJCF ──► engine
   (pluggable)    (flat, named,     (morph operator,           (morphed IR    │                  │
                   identifiable)     pure fn)                   instance)      │            oracle harness
                                                                               │            grades it (cf-osim)
   three sources today:                                                  cf-mjcf-emit
     • CanonicalSource   → template defaults (no scan)  ← builder-first
     • ScanSource        → cf-anthro Landmarks → params (tier a)
     • RandomizerSource  → sample within ranges (free training data)
   later:
     • ShapeModelSource  → a few SSM PCs (tier b)
```

The key move: today `cf-msk-fit::place_knee(&Subgraph, &Landmarks) -> Placement` fuses **the
landmark→placement step** (params + morph) with the output, and emits an ad-hoc **render overlay**
(`Bone`/`PlacedMuscle`), not MJCF. We split those into three named seams — `ParamSource`,
`realize()`, `cf-mjcf-emit` — so the scan stops being special and the output becomes a real MJCF
body. (Mesh→`Landmarks` extraction already lives separately, in `cf-anthro`.)

## The four core abstractions

### 1. The anatomical IR — generalized `cf-osim::Subgraph`

`cf-osim::osim::Subgraph` is already the v0 IR (one template: the knee). Promote it to a general,
source-agnostic biomech IR and move it into a new `cf-msk-lib`:

```rust
// cf-msk-lib::ir
pub struct Segment {            // a bone
    pub name: String,
    pub parent: Option<String>, // kinematic-tree parent
    pub frame: Isometry3<f64>,  // pose in parent frame (the joint transform)
    pub geometry: GeomHandle,   // mesh asset / primitive, in segment frame
    pub inertial: Inertial,     // mass, com, inertia (scales with the segment)
    pub scaling: ScaleRule,     // ← which params drive this segment, and how
}

pub struct Joint {              // generalizes KneeJoint + MovingSplines
    pub name: String,
    pub center: Point3<f64>,
    pub dof: JointDof,          // Hinge { axis } | Coupled { splines } | Ball | …
    pub rom: (f64, f64),        // honest ROM bound (G1 hinge validity, recon R3)
}

pub struct MusclePath {         // generalizes Muscle + PathPoint
    pub name: String,
    pub points: Vec<PathPoint>, // origin / via (Fixed|Conditional|Moving) / insertion
    pub wraps: Vec<WrapSurface>,// sphere / cylinder → engine Geom wraps
    pub hill: HillParams,       // G2; carried but unused at G1
}
```

The IR keeps **joint coupling symbolic** (the `MovingSplines` survive morphing untouched —
scaling changes geometry, not the rolling-gliding *relationship*). That matches the locked
decision: G1 emits the hinge, the coupling rides along for later gates.

### 2. `BodyParams` — flat, named, identifiable

```rust
// cf-msk-lib::params
pub struct Param {
    pub value: f64,
    pub default: f64,           // from the canonical template
    pub range: (f64, f64),      // valid bounds (also the randomizer's support)
    pub measurable: Measurable, // ← the discriminator from the vision doc
}
pub enum Measurable { ByScanner, AnthroPrior, FixedAnatomy }

pub struct BodyParams(BTreeMap<ParamId, Param>);  // femur_len, tibia_len,
                                                  // epicondyle_width, knee_center, …
```

`Measurable` encodes the honesty discipline **in the type system**: a `ScanSource` may only set
`ByScanner` params; everything else falls back to the template default or an anthropometric prior.
A bad scanner therefore *cannot* silently inject garbage geometry — it can only touch the handful
of quantities it can actually measure (lengths, girths, joint centers).

### 3. `realize()` — the morph operator (a pure function)

```rust
// cf-msk-lib::morph
pub fn realize(template: &BodyTemplate, params: &BodyParams) -> BodySpec
```

This generalizes today's single similarity transform into **per-segment anisotropic scaling**
(the recon's deferred `ScaleSpec`). Crucially, *how* each segment responds lives in its
`ScaleRule` in the **library**, not in the solver — so adding the tibia, then the foot, then the
arm never edits the morph code. Deterministic and side-effect-free, so it's trivially testable and
re-runnable (matters for the randomizer and for resume-able fitting).

### 4. `ParamSource` — the parameter *interface* (the pluggable seam)

```rust
// cf-msk-lib::source
pub trait ParamSource {
    fn params(&self, template: &BodyTemplate) -> BodyParams;
}

// cf-msk-lib              — CanonicalSource (defaults), RandomizerSource (domain randomization)
// cf-msk-fit              — ScanSource { landmarks: cf_anthro::Landmarks }   (tier a)
// later                   — ShapeModelSource { pcs: Vec<f64> }              (tier b)
```

`CanonicalSource` is what makes the program **builder-first**: a full, simulatable body with zero
scan input. `RandomizerSource` is the **free-training-data** generator (sample each `ByScanner`
param across its range → a population of bodies for the detector / RL / system-ID). `ScanSource`
is today's `cf-msk-fit`, demoted from "the pipeline" to "one source among three."

## Crate layout & dependency graph

```
            cf-msk-lib  ── IR · BodyTemplate · BodyParams · realize() · ParamSource
            ▲    ▲    ▲      + CanonicalSource + RandomizerSource
   ┌────────┘    │    └──────────┐
 cf-osim     cf-mjcf-emit     cf-msk-fit ── ScanSource (Landmarks → BodyParams) + fit/registration
 (.osim →    (IR → MJCF text;     ▲
  IR; +       cf-osim::emit       │
  oracle)     migrates here)   cf-anthro ── scan → Landmarks
```

- **`cf-msk-lib`** *(new)* — owns the IR, templates, params, morph, and the source trait. The
  heart of "the library / parameter interface."
- **`cf-osim`** *(exists, refocus)* — `.osim` → IR (the **library seed**) and the **oracle**
  (`oracle::Kinematics`, the grader). `emit.rs` moves out to `cf-mjcf-emit`.
- **`cf-mjcf-emit`** *(new — already on the build list, recon R4)* — IR → MJCF text, round-tripped
  through the import-only `sim/L0/mjcf`. The engine has no programmatic builder, so this is the
  only way out.
- **`cf-msk-fit`** *(exists, narrows)* — becomes the `ScanSource` + registration solver. Loses the
  emit/overlay responsibility to `cf-mjcf-emit`.
- **`cf-anthro`** *(exists, unchanged)* — scan → `Landmarks`. Stays dependency-light (it does not
  depend on `cf-msk-lib`; the `Landmarks → BodyParams` adapter lives in `cf-msk-fit`).

## What changes vs. today

| Concern | Today | After |
|---|---|---|
| IR | `cf-osim::Subgraph` (knee-only, in the `cf-osim` bridge) | `cf-msk-lib::ir` (general, source-agnostic) |
| Build a body with no scan | not possible | `CanonicalSource` → `realize` → emit |
| Scan's role | the pipeline entry point (`place_knee(…)`) | one `ParamSource` among three |
| Morph | single similarity transform, inside `place` | `realize()`, per-segment, rules in the library |
| Output | `Placement` render overlay (bones as segments) | real MJCF body via `cf-mjcf-emit` |
| Emit | `cf-osim::emit` | `cf-mjcf-emit` (migrated) |
| Training data | none | `RandomizerSource` |
| Bad-scanner safety | implicit | `Measurable` gate in the type |

## Five decisions this sketch locks

1. **Parameters morph the IR, then emit MJCF.** Never deform the scan mesh (no tier-(c)).
2. **One `BodyParams`, many `ParamSource`s.** Decouples scan from builder; randomizer falls out free.
3. **Scaling rules live on the segment, not in the solver.** Adding a bone never edits `realize()`.
4. **`Measurable` is a type, not a comment.** The scanner can only touch what it can measure.
5. **Coupling stays symbolic through the morph.** Scaling changes geometry, not the joint relation —
   consistent with "hinge at G1, coupling deferred."

## Build order (smallest first, each independently checkable)

1. **Extract `cf-msk-lib::ir`** from `cf-osim::Subgraph`; `cf-osim` re-exports during migration.
2. **`cf-mjcf-emit`** = move `cf-osim::emit`, round-trip a canonical knee through `sim/L0/mjcf`,
   grade against the oracle (proves emit fidelity *before* any morphing). This is recon S-emit / R4.
3. **`BodyParams` + `CanonicalSource` + `realize()` (uniform scale first).** Emit a canonical knee
   with no scan → first builder-first artifact.
4. **`ScanSource`** = wrap `cf-anthro::Landmarks` → `BodyParams`; reproduce today's `cf-msk-fit`
   result through the new seam (no behavior change — a refactor checkpoint).
5. **Per-segment `ScaleRule`** (the deferred `ScaleSpec`), then **`RandomizerSource`** for training
   data. Tier-(b) `ShapeModelSource` is post-G1.

Detail and the S0→S5 ladder it plugs into: `../03_phases/g1_knee_kinematics/recon.md`.
