# Example Building Blocks Spec

**Status:** Approved — ready for implementation
**Date:** 2026-03-25
**Motivation:** The slide-joint example revealed three patterns that will recur
across every Track 1 example. Promoting them to reusable building blocks
prevents copy-paste and ensures consistent quality.

---

## 1. Spring Coil Mesh — `sim_bevy::mesh::spring_coil`

### Problem

Every example involving springs, tendons, or muscles needs a helical coil
visual that stretches/compresses dynamically. The slide-joint example has a
100-line mesh generator inlined. This will be needed in: slide-joint,
actuators, muscles, equality-constraints, contact-tuning, and any tendon demo.

### API

```rust
// sim-bevy/src/mesh.rs (or new file sim-bevy/src/mesh/coil.rs)

/// Parameters for a helical coil spring mesh.
pub struct SpringCoilParams {
    /// Number of coil turns.
    pub turns: u32,
    /// Radius of the helix (distance from axis to wire center).
    pub radius: f32,
    /// Radius of the wire tube cross-section.
    pub tube_radius: f32,
    /// Segments per full turn (controls smoothness). Default: 24.
    pub segments_per_turn: u32,
    /// Tube cross-section segments. Default: 8.
    pub tube_segments: u32,
    /// Minimum length clamp to avoid degenerate geometry. Default: 0.05.
    /// Adjust for your simulation scale (e.g. 0.001 for mm-scale sims).
    pub min_length: f32,
}

impl Default for SpringCoilParams {
    fn default() -> Self {
        Self {
            turns: 10,
            radius: 0.06,
            tube_radius: 0.008,
            segments_per_turn: 24,
            tube_segments: 8,
            min_length: 0.05,
        }
    }
}

/// Generate a helical coil mesh from `x=0` to `x=length`.
///
/// The coil axis is along +X. Negative or very small lengths are clamped
/// to `params.min_length` — never panics, always returns a valid mesh.
///
/// Intended for dynamic use: regenerate each frame with a new length to
/// animate spring compression/extension.
///
/// `usage` controls GPU/CPU asset residency (pass `RenderAssetUsages::default()`
/// for typical use, or `RENDER_WORLD` for GPU-only).
pub fn spring_coil(
    params: &SpringCoilParams,
    length: f32,
    usage: RenderAssetUsages,
) -> Mesh;
```

### Design decisions

- **No low-poly variant.** The caller tunes `segments_per_turn` and
  `tube_segments` in the params struct. One function, full control.
- **`RenderAssetUsages` is an explicit parameter.** No hidden defaults —
  the caller decides GPU/CPU residency. Avoids a breaking change later
  when someone needs non-default usage.
- **`min_length` is configurable.** Hardcoding 0.05 breaks mm-scale or
  10m-scale simulations. The params struct default (0.05) works for typical
  meter-scale sims; the caller overrides for their scale.
- Axis is always +X (Bevy space). The caller positions/rotates the entity.
- Returns a `Mesh` with positions, normals, and triangle indices. No UVs.
- The `params` struct is separate from `length` because params are constant
  while length changes every frame.

### Robustness guarantees

- Negative length → clamped to `min_length`, no panic.
- Zero turns → returns a degenerate but valid empty mesh.
- Zero radius → returns a line-degenerate mesh (no tube), still valid.

---

## 2. Material Override — `sim_bevy::materials`

### Problem

`spawn_model_geoms` creates flat-color materials from MJCF `rgba` values.
For polished examples, we need metallic PBR materials (roughness, metallic,
specular). The slide-joint example uses a 30-line query-based override system
that matches geom names to materials. Every example will need this.

### API

```rust
// sim-bevy/src/materials.rs (new file)

/// Metallic material preset for common surface types.
///
/// Each preset defines metallic and roughness values. Call `material()` for
/// the preset's default color, or `with_color()` to override the base color
/// while keeping the metallic/roughness properties.
pub enum MetalPreset {
    /// Polished steel: high metallic (0.9), low roughness (0.25).
    /// Default color: light grey (0.52, 0.53, 0.56).
    PolishedSteel,
    /// Brushed metal: high metallic (0.85), medium roughness (0.35).
    /// Default color: neutral grey (0.48, 0.48, 0.50).
    BrushedMetal,
    /// Cast iron: medium metallic (0.6), high roughness (0.55).
    /// Default color: dark grey (0.35, 0.33, 0.32).
    CastIron,
    /// Spring/tendon wire: steel with slight warmth.
    /// High metallic (0.85), low-medium roughness (0.35).
    /// Default color: warm grey (0.55, 0.58, 0.60).
    SpringWire,
    /// Anodized aluminum with a tint color.
    /// High metallic (0.7), low roughness (0.3).
    /// Color provided by caller.
    Anodized(Color),
}

impl MetalPreset {
    /// Create a `StandardMaterial` with this preset's default color.
    pub fn material(&self) -> StandardMaterial;

    /// Create a `StandardMaterial` with a custom base color.
    ///
    /// Keeps the preset's metallic/roughness values, overrides the color.
    /// Use this for e.g. a polished-steel block in red.
    pub fn with_color(&self, color: Color) -> StandardMaterial;
}

/// Override the material of a geom identified by name.
///
/// Finds the entity with `ModelGeomIndex` matching the named geom and
/// replaces its `MeshMaterial3d<StandardMaterial>`.
///
/// Returns `true` if the geom was found and updated.
pub fn override_geom_material_by_name(
    model: &Model,
    geom_name: &str,
    material: Handle<StandardMaterial>,
    query: &mut Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) -> bool;

/// Override the material of a geom identified by index.
///
/// Directly matches `ModelGeomIndex(index)`.
///
/// Returns `true` if the geom was found and updated.
pub fn override_geom_material_by_index(
    index: usize,
    material: Handle<StandardMaterial>,
    query: &mut Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) -> bool;

/// Override materials for multiple geoms at once (by name).
///
/// Takes a slice of `(geom_name, material_handle)` pairs.
/// Returns the number of geoms successfully updated.
pub fn override_geom_materials_by_name(
    model: &Model,
    overrides: &[(&str, Handle<StandardMaterial>)],
    query: &mut Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) -> usize;
```

### Design decisions

- **`SpringWire` preset included.** Springs are first-class visuals in this
  codebase. A dedicated preset ensures consistent spring appearance across
  every example — one change propagates everywhere.
- **`with_color()` method.** Presets define metallic/roughness, not color.
  A polished-steel block in red uses `PolishedSteel.with_color(red)`.
  Without this, the caller would skip presets entirely and build materials
  from scratch, defeating the purpose.
- **Both name and index overrides.** Name for MJCF-loaded models (ergonomic),
  index for programmatic models, URDF, procedural generation. Two explicit
  methods, no ambiguity.
- **Plain functions, not Bevy systems.** Maximum flexibility — callable from
  setup, one-shot systems, callbacks, anywhere. The caller passes the query.
- One-shot operation (not persistent per-frame). No overhead after setup.

### Alternative: MJCF-level material attributes

Instead of post-spawn overrides, we could extend the MJCF parser to support
metallic/roughness attributes directly (non-standard MJCF). This would be
cleaner but diverges from MuJoCo compatibility. **Decision: post-spawn
override for now. Revisit if every example needs it.**

---

## 3. Validation Framework — `sim_core::validation`

### Problem

Every example needs the same core checks: oscillation period, energy
behavior, constraint satisfaction. The slide-joint example has 60 lines of
validation code. Ball-joint, integrators, solvers, actuators, etc. will all
need variations of the same pattern.

### API

```rust
// sim-core/src/validation.rs (new module)
// Zero Bevy dependency — pure data structs + println formatter.

/// A single validation check result.
///
/// Each tracker pre-formats its own `detail` string because check types
/// have fundamentally different shapes:
/// - Period: "measured=1.4739s  expected=1.4735s  error=0.02%"
/// - Energy: "max increase=0.00e0J"
/// - Limits: "max violation=0.000000m"
///
/// Raw values are accessible through each tracker's own methods
/// (e.g. `measured_period()`, `max_increase()`, `max_violation()`).
pub struct Check {
    /// Short label for this check (e.g. "Period", "Energy", "Limits").
    pub name: &'static str,
    /// Whether the check passed.
    pub pass: bool,
    /// Pre-formatted detail string (units, values, thresholds).
    pub detail: String,
}

/// Tracks zero crossings of a scalar signal for period measurement.
/// Uses linear interpolation between samples for sub-frame precision.
pub struct PeriodTracker {
    last_value: f64,
    last_time: f64,
    crossings: Vec<f64>,
}

impl PeriodTracker {
    pub fn new() -> Self;

    /// Feed a new sample. Call once per frame with the current value and time.
    ///
    /// Internally uses linear interpolation to estimate the exact crossing
    /// time between consecutive samples.
    ///
    /// Gracefully handles edge cases:
    /// - Time going backwards (sim reset): sample is skipped.
    /// - First call: stores initial state, no crossing possible.
    pub fn sample(&mut self, value: f64, time: f64);

    /// Compute the average period from recorded zero crossings.
    /// Returns `None` if fewer than 2 crossings recorded.
    pub fn measured_period(&self) -> Option<f64>;

    /// Number of zero crossings recorded so far.
    pub fn crossing_count(&self) -> usize;

    /// Check measured period against expected, with a percentage threshold.
    ///
    /// Returns a failing check with descriptive detail if no crossings
    /// were recorded (never panics).
    pub fn check(&self, expected: f64, threshold_pct: f64) -> Check;
}

/// Tracks energy to verify monotonic decrease (for damped systems).
pub struct EnergyMonotonicityTracker {
    prev: Option<f64>,
    max_increase: f64,
}

impl EnergyMonotonicityTracker {
    pub fn new() -> Self;

    /// Feed a new energy sample. Call once per frame.
    pub fn sample(&mut self, energy: f64);

    /// The largest single-frame energy increase observed.
    pub fn max_increase(&self) -> f64;

    /// Check that energy never increased by more than threshold (in Joules).
    ///
    /// Returns a passing check if no samples were recorded (vacuously true).
    pub fn check(&self, threshold: f64) -> Check;
}

/// Tracks joint/tendon limit violations.
pub struct LimitTracker {
    max_violation: f64,
}

impl LimitTracker {
    pub fn new() -> Self;

    /// Feed a new position sample with the allowed range.
    pub fn sample(&mut self, value: f64, lo: f64, hi: f64);

    /// The largest violation observed (in meters or radians).
    pub fn max_violation(&self) -> f64;

    /// Check that violations never exceeded threshold.
    ///
    /// Returns a passing check if no samples were recorded (vacuously true).
    pub fn check(&self, threshold: f64) -> Check;
}

/// Print a formatted validation report and return whether all checks passed.
///
/// Output format (one line per check, aligned):
/// ```text
/// === title ===
///   Period:  measured=1.4739s  expected=1.4735s  error=0.02%  PASS
///   Energy:  max increase=0.00e0J  PASS
///   Limits:  max violation=0.000000m  PASS
/// ===...===
/// ```
///
/// Returns `true` if every check passed. Enables CI: run headless, check
/// the return value.
pub fn print_report(title: &str, checks: &[Check]) -> bool;

/// Check whether all checks passed without printing.
///
/// For pure headless/test use where stdout is not desired.
pub fn all_passed(checks: &[Check]) -> bool;
```

### Design decisions

- **Lives in `sim_core::validation`.** The trackers have zero Bevy dependency
  (pure structs, `println!` only). Put it where it belongs from day one.
- **`Check` uses `detail: String`, not structured fields.** Different check
  types have fundamentally different shapes (period has measured/expected/error,
  energy has just max_increase, limits has just max_violation). A rigid struct
  with `threshold_pct` lies for two of three trackers. Each tracker formats
  its own detail; `print_report` aligns and appends PASS/FAIL. Raw values
  stay accessible through each tracker's own accessor methods.
- **Linear interpolation always on** in `PeriodTracker`. Frame-rate sampling
  alone gave 4.5% error; interpolation gave 0.02%. Two orders of magnitude
  for three lines of code. No opt-out.
- **`print_report` returns `bool`.** Both prints and returns all-passed
  status. Enables CI integration without stdout parsing.
- **`all_passed` for headless.** Separate function that doesn't print.
- **Composable.** Each tracker is independent. Examples pick what they need.
- **Not a Bevy system.** Plain structs called from the example's own system.

### Robustness guarantees

- `PeriodTracker::sample` with time going backwards → skipped, no panic.
- `PeriodTracker::check` with zero crossings → failing Check with
  `detail: "insufficient zero crossings"`.
- `EnergyMonotonicityTracker::check` with zero samples → passing Check
  (vacuously true — nothing violated).
- `LimitTracker::check` with zero samples → passing Check (vacuously true).
- No tracker method ever panics regardless of call order or input values.

### Future trackers (not in v1)

These can be added when specific examples need them:
- `EnergyConservationTracker` — for undamped systems (drift < threshold)
- `QuaternionNormTracker` — for ball/free joints (norm stays near 1.0)
- `ConvergenceTracker` — for solvers (iteration count, residual)
- `TrajectoryTracker` — for inverse dynamics (tracking error vs reference)

---

## Implementation order

1. **Validation framework** (sim-core) — no dependencies, used by everything
2. **Spring coil mesh** (sim-bevy) — needed by slide-joint refactor
3. **Material override** (sim-bevy) — needed by slide-joint refactor
4. **Refactor slide-joint** to use all three building blocks
5. Verify slide-joint still passes all validations
