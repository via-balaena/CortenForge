# Foundations — the existing substrate (what we recycle)

*2026-06-07. Inventory from a read-only sweep of the repo; file paths verified.*

The builder is green-field, but most of the heavy machinery already exists. This is the
recycle list — what we build *on*, so the new work is only the connective tissue.

## Rigid-body engine — `sim/L0/core`

MuJoCo-aligned, immutable `Model` + mutable `Data` split (`types/model.rs`, `types/data.rs`).
- **Kinematic trees & joints** — hinge / ball / slide / free, with limits, stiffness, damping,
  armature. A knee hinge is a first-class primitive.
- **Spatial tendons with wrapping** — `tendon/spatial.rs`, `tendon/wrap_math.rs`. Wrap types are
  `Site` / `Geom` / `Joint` / `Pulley` with **sphere & cylinder** wrap geoms — these map directly
  onto OpenSim's `PathPoint` (→ `Site`) and `WrapCylinder` / `WrapSphere` (→ `Geom`). This is why
  the OpenSim path representation is reachable in our engine.
- **Moment arms — for free.** The tendon Jacobian `data.ten_J` (populated by the spatial-tendon
  forward pass) projected onto a joint DOF *is* the muscle moment arm — exactly the quantity G1
  grades against the oracle. No new physics needed to extract it.
- **Hill muscle model** — `forward/hill.rs`, `forward/fiber.rs`: force-length / force-velocity /
  pennation. Present, but MuJoCo-piecewise (not OpenSim Thelen/Millard) — a **G2** concern, not
  G1 (G1 is purely kinematic). See recon risk R5.

## Model authoring — `sim/L0/mjcf` (import-only)

Full MJCF *importer*: bodies, joints, fixed + spatial tendons, sites, wrap geoms, mesh geoms,
muscles, actuators (`builder/tendon.rs`, `builder/joint.rs`, `builder/body.rs`, …).

**The load-bearing constraint:** there is **no MJCF *exporter*** and **no programmatic `Model`
builder** — models are constructed *only* by parsing MJCF XML. So every model we generate must be
emitted as MJCF text and round-tripped through this importer. This is why a new `cf-mjcf-emit`
crate is on the build list (recon §"new crates", risk R4). URDF (`sim/L0/urdf`) is joints-only —
no tendon/muscle concept — so it is **not** a viable path for musculoskeletal models.

## Scan → watertight twin — `tools/cf-scan-prep-core/src/lib.rs`

Production-grade, already used by Studio. Emits a cleaned watertight STL (meters) + `.prep.toml`
provenance. Relevant entry points:
- `compute_centerline_polyline()` — 1D centroid-chain spine of a tubular body part (legs/arms).
- `compute_pca_orientation()` / `auto_pca_in_place()` — pose-normalize to a principal axis.
- cross-section sampling + radial-profile sampling — girth/area along the centerline.
- `clip_mesh_against_plane()` / `trim_mesh_along_centerline()` — segment a limb at a level.

These are the **landmark-detection substrate** (S2): the knee reads as a cross-sectional-area
minimum along the centerline between the thigh and calf girth maxima.

## Mesh suite

- `mesh/mesh-measure` — `cross_section()`, `circumference_at_height()`, `oriented_bounding_box()`
  (PCA OBB — epicondyle mediolateral width), `closest_point_on_mesh()`.
- `mesh/mesh-sdf` — `flood_filled_sdf()`, robust inside/outside. This is the **skin-envelope
  containment** test for G1 (does any bone/tendon point poke through the skin across the ROM?).
- `mesh/mesh-offset`, `mesh/mesh-shell` — envelope tolerance bands; marching-cubes isosurfacing.
- `mesh/mesh-repair` — weld / fill / validate (ensure manifold input before fitting).

## Soft-tissue FEM — `sim/L0/soft` (for later, not G1)

Hyperelastic (Neo-Hookean + Yeoh), SDF→tet meshing, penalty rigid contact, stress/pressure
readout. This is the **skeleton↔tissue** side of the twin — downstream of G1's rigid+tendon
skeleton, and the surface the keystone soft↔rigid coupling will act on. Not on the G1 path.

## What is genuinely missing (the build list)

| Gap | Why it's not already here |
|---|---|
| OpenSim `.osim` reader → biomech IR | No biomech format support anywhere in the repo |
| MJCF **emitter** | `sim/L0/mjcf` is import-only; no programmatic builder |
| Anatomical **landmark detection** | Scan pipeline finds centerlines/girths, not joint centers |
| Anthropometric **scaling/registration** solver | No template-fit / morph machinery |
| Oracle-comparison + envelope **validation harness** | No biomech ground-truth comparison exists |

Detail and the slice-by-slice plan: `../03_phases/g1_knee_kinematics/recon.md`.
