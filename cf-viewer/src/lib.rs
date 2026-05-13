//! `cf-viewer` library — PLY loading + the [`ViewerInput`] data type +
//! scene-setup helpers shared with `tools/cf-scan-prep`.
//!
//! Wraps [`mesh_io::load_ply_attributed`] in a Bevy-friendly resource that
//! carries the loaded geometry plus a deterministically-ordered list of
//! per-vertex scalar names. Submodules:
//!
//! - [`mesh`] — `AttributedMesh → Bevy Mesh` conversion (face case) and
//!   the point-cloud sizing constant.
//! - [`colormap`] — Q5 distribution detection + per-vertex RGBA mapping
//!   (commit 4).
//! - [`ui`] — scalar + colormap dropdowns via `bevy_egui` (commit 5).
//! - [`cli`] — `clap`-derived `--scalar` / `--colormap` / `--up` flags
//!   that seed [`ui::Selection`] + [`UpAxis`] before the dropdowns see
//!   them (commit 6).
//!
//! Top-level scene helpers shared across viewer binaries:
//!
//! - [`RenderScale`] / [`compute_render_scale`] / [`scale_aabb`] — lift
//!   sub-meter scenes to Bevy's pipeline-default human-scale regime.
//! - [`setup_camera_and_lighting`] — spawn orbit camera + directional
//!   light + global ambient framed on a scaled AABB.
//! - [`spawn_face_mesh`] — flat-shaded `IndexedMesh` → Bevy entity spawn
//!   for STL-style face meshes; accepts pre-computed per-vertex RGBA
//!   colors but does NOT run scalar-field → colormap detection (that
//!   lives in `cf-view`'s `spawn_geometry` system). `cf-view`'s PLY-
//!   stored-normals dispatch keeps its own inline
//!   [`mesh::build_face_mesh`] path.
//!
//! The orbit camera + up-axis convention previously housed in this crate
//! moved to [`cf_bevy_common`] at sim-soft PR2 C2b so sim-bevy-soft and
//! sim-bevy can share the same controller. [`UpAxis`] is re-exported here
//! for back-compat with cf-viewer's existing call sites.

pub mod cli;
pub mod colormap;
pub mod mesh;
pub mod sequence;
pub mod ui;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use bevy::prelude::*;
pub use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::camera::OrbitCamera;
use cf_bevy_common::mesh::triangle_mesh_flat_shaded;
use mesh_io::{load_ply_attributed, load_stl};
use mesh_types::{Aabb, AttributedMesh, IndexedMesh, Point3};

pub use sequence::Sequence;

/// Multi-STL assembly inputs — N loaded meshes ready to spawn
/// into a single scene at their world-coordinate positions.
///
/// Inserted as a Bevy resource by `main.rs` when [`InputMode::Assembly`]
/// is selected via `--assembly`. The spawn system enumerates this
/// Vec at startup and creates one entity per piece, each tagged with
/// an `AssemblyPiece` component (lives in `main.rs`) for per-piece
/// visibility toggling.
///
/// Distinct from [`ViewerInput`] (which is the single-mesh
/// PLY/Sequence carrier) — assembly mode does NOT participate in
/// scrub/playback or scalar-dropdown machinery.
#[derive(Resource, Debug)]
pub struct AssemblyInputs {
    /// Per-piece loaded mesh + filename, lex-sorted to match the
    /// directory's natural order.
    pub pieces: Vec<AssemblyPieceInput>,
}

/// One piece of an [`AssemblyInputs`]: its loaded mesh + the source
/// filename for UI display.
#[derive(Debug)]
pub struct AssemblyPieceInput {
    /// Loaded mesh in world-coordinate positions (cf-cast STLs are
    /// already in mm-frame world coordinates per the v2 export
    /// pipeline, so no re-positioning is needed).
    pub mesh: AttributedMesh,
    /// Filename (basename, no directory). Used for the per-piece
    /// visibility-toggle label in the side panel.
    pub name: String,
}

/// Loaded artifact ready for visualization.
///
/// `scalar_names` is the alphabetically-ordered list of per-vertex
/// `extras` keys (Q4 lock — see `docs/VIEWER_DESIGN.md`). Auto-selection
/// picks the first; the [`ui::scalar_and_colormap_panel`] dropdown
/// (commit 5) and the `--scalar=<name>` CLI flag (commit 6) override.
///
/// [`ui::scalar_and_colormap_panel`]: crate::ui::scalar_and_colormap_panel
#[derive(Resource)]
pub struct ViewerInput {
    /// Geometry plus per-vertex attributes loaded from the PLY.
    pub mesh: AttributedMesh,

    /// Sorted names of per-vertex scalar extras (`mesh.extras` keys).
    pub scalar_names: Vec<String>,
}

/// Load a PLY or STL file into a [`ViewerInput`].
///
/// File-extension dispatch:
///
/// - `.stl` (case-insensitive) → [`mesh_io::load_stl`] → wrap in
///   [`AttributedMesh::new`]. STL carries no per-vertex extras, so
///   `scalar_names` is empty and the
///   [`ui::scalar_and_colormap_panel`] dropdown gates itself out.
///   Useful for inspecting mold-piece + plug STLs from
///   `cf-cast::export_molds_v2` output without round-tripping through
///   a PLY converter.
/// - Anything else (default) → [`mesh_io::load_ply_attributed`]; the
///   `extras` map keys project into `scalar_names` alphabetically
///   (`BTreeMap` iteration order, deterministic across loads).
///
/// # Errors
///
/// Returns an error if the file cannot be read or the file is
/// malformed for its detected format.
pub fn load_input(path: &Path) -> Result<ViewerInput> {
    if is_stl_path(path) {
        let geometry =
            load_stl(path).with_context(|| format!("loading STL from {}", path.display()))?;
        let mesh = AttributedMesh::new(geometry);
        return Ok(ViewerInput {
            mesh,
            scalar_names: Vec::new(),
        });
    }
    let mesh = load_ply_attributed(path)
        .with_context(|| format!("loading PLY from {}", path.display()))?;
    let scalar_names: Vec<String> = mesh.extras.keys().cloned().collect();
    Ok(ViewerInput { mesh, scalar_names })
}

/// Load every STL in `stls` into an [`AssemblyInputs`] ready for
/// multi-piece spawning. Each STL is loaded independently via
/// [`mesh_io::load_stl`] + wrapped in [`AttributedMesh::new`]; the
/// per-piece basename is captured for the UI label.
///
/// # Errors
///
/// Returns the first per-piece load error encountered (with the
/// failing path attached via [`anyhow::Context`]). Caller is
/// responsible for ensuring `stls` is non-empty if the downstream
/// spawn system requires it.
pub fn load_assembly_inputs(stls: &[PathBuf]) -> Result<AssemblyInputs> {
    let mut pieces = Vec::with_capacity(stls.len());
    for path in stls {
        let geometry =
            load_stl(path).with_context(|| format!("loading STL from {}", path.display()))?;
        let mesh = AttributedMesh::new(geometry);
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .map_or_else(|| path.display().to_string(), String::from);
        pieces.push(AssemblyPieceInput { mesh, name });
    }
    Ok(AssemblyInputs { pieces })
}

/// Does `path`'s extension match `.stl` (case-insensitive)?
/// Used by [`load_input`] for format dispatch + by
/// [`discover_stl_sequence`] for STL-frame detection.
fn is_stl_path(path: &Path) -> bool {
    path.extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("stl"))
}

/// Lex-sorted list of `*_step_<digits>.ply` files under `dir`.
///
/// The sim-soft `viz::design_surface_deformed` PLY series writes
/// zero-padded step indices (e.g. `sleeve_design_surface_deformed_step_01.ply`),
/// so lex order matches step order. Other files in the directory
/// (final-state PLYs, JSON readouts, slab cuts) are silently skipped.
///
/// This is the D1 slice's input-mode detector for `cf-view <dir>` mode
/// (per `docs/SIM_SOFT_ROADMAP.md` Track D, leaf D1).
///
/// # Errors
///
/// - `dir` cannot be read.
/// - no entries match the `*_step_<digits>.ply` pattern.
pub fn discover_ply_sequence(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut frames: Vec<PathBuf> = Vec::new();
    let read_dir =
        std::fs::read_dir(dir).with_context(|| format!("reading directory {}", dir.display()))?;
    for entry in read_dir {
        let entry = entry.with_context(|| format!("reading dir entry under {}", dir.display()))?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        if is_sequence_frame_name(name) {
            frames.push(path);
        }
    }
    if frames.is_empty() {
        bail!(
            "no PLY sequence frames found in {} (looking for *_step_<digits>.ply)",
            dir.display(),
        );
    }
    frames.sort();
    Ok(frames)
}

/// Does the filename match the `*_step_<digits>.ply` sequence-frame
/// convention? `_step_` must appear in the stem, followed by at least
/// one ASCII digit before the `.ply` extension.
fn is_sequence_frame_name(name: &str) -> bool {
    let Some(stem) = name.strip_suffix(".ply") else {
        return false;
    };
    let Some(idx) = stem.rfind("_step_") else {
        return false;
    };
    let digits = &stem[idx + "_step_".len()..];
    !digits.is_empty() && digits.chars().all(|c| c.is_ascii_digit())
}

/// Lex-sorted list of `*.stl` files under `dir`.
///
/// Workshop-iter-1 visual-review use case (see
/// `docs/CURVE_FOLLOWING_DESIGN.md` Step 11 + the v2 cf-cast example
/// crate's `out/` directory): step through N piece STLs + plug + any
/// other STLs in the directory one at a time via the same scrub UI
/// the PLY-frame mode uses. STLs have no temporal meaning here —
/// they're separate pieces — but reusing the
/// [`crate::sequence::Sequence`] resource + scrub UI gives the
/// user piece-by-piece inspection without a new viewer mode.
///
/// Unlike [`discover_ply_sequence`], no filename convention is
/// enforced; any `*.stl` file qualifies. Lex order matches the v2
/// example's natural sort (`mold_layer_0_piece_0.stl` <
/// `mold_layer_0_piece_1.stl` < `mold_layer_1_piece_0.stl` < … <
/// `plug.stl`).
///
/// # Errors
///
/// - `dir` cannot be read.
/// - no `*.stl` entries match.
pub fn discover_stl_sequence(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut frames: Vec<PathBuf> = Vec::new();
    let read_dir =
        std::fs::read_dir(dir).with_context(|| format!("reading directory {}", dir.display()))?;
    for entry in read_dir {
        let entry = entry.with_context(|| format!("reading dir entry under {}", dir.display()))?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        if is_stl_path(&path) {
            frames.push(path);
        }
    }
    if frames.is_empty() {
        bail!(
            "no STL frames found in {} (looking for *.stl)",
            dir.display(),
        );
    }
    frames.sort();
    Ok(frames)
}

/// Detected CLI-input form: single file, multi-frame sequence, or
/// multi-STL assembly. Single/Sequence variants pre-date Option C
/// (STL assembly mode); Assembly is the cf-cast multi-piece-mold
/// visual-review entry point per
/// `docs/CURVE_FOLLOWING_DESIGN.md` Step 11 + the v2 cast example's
/// `out/` directory.
#[derive(Debug, Clone)]
pub enum InputMode {
    /// Single static PLY or STL — the existing pre-D1 viewer
    /// behavior, extended for STL by the load_input extension-
    /// dispatch path.
    Single(PathBuf),
    /// Lex-sorted sequence of `*_step_<n>.ply` frames OR `*.stl`
    /// files, with the initial `current` index set to 0. Scrub UI
    /// flips between frames one at a time.
    Sequence(Sequence),
    /// Multi-STL assembly: spawn ALL STLs in a directory at their
    /// world-coordinate positions simultaneously. Per-piece
    /// visibility toggle in the side panel; per-piece distinct
    /// colors. No scrub UI (assembly has no temporal meaning).
    Assembly {
        /// Lex-sorted list of STL paths. Same discovery as
        /// `Sequence` for STLs ([`discover_stl_sequence`]), but
        /// loaded all at once rather than one-at-a-time.
        stls: Vec<PathBuf>,
    },
}

impl InputMode {
    /// Path of the frame to load first. For [`InputMode::Single`] this
    /// is the CLI-supplied path; for [`InputMode::Sequence`] it is
    /// frame 0 of the sorted series.
    ///
    /// # Errors
    ///
    /// Returns an error only in the (logically unreachable) case where
    /// a [`Sequence`] was constructed with an empty `frames` vec.
    pub fn initial_frame(&self) -> Result<PathBuf> {
        match self {
            Self::Single(p) => Ok(p.clone()),
            Self::Sequence(s) => s
                .current_path()
                .map(Path::to_path_buf)
                .ok_or_else(|| anyhow::anyhow!("sequence has no current frame")),
            Self::Assembly { stls } => stls
                .first()
                .cloned()
                .ok_or_else(|| anyhow::anyhow!("assembly has no STLs")),
        }
    }
}

/// Detect whether the CLI path resolves to a single file or a directory
/// of sequence frames.
///
/// Pure detection — no stdout side effects (see `main.rs` for the
/// human-facing detection log).
///
/// **Directory dispatch**: try [`discover_ply_sequence`] first
/// (`*_step_<digits>.ply` convention); if no PLY frames match, fall
/// back to [`discover_stl_sequence`] (`*.stl` lex-sorted). Both
/// produce a [`Sequence`] consumed by the same scrub UI. Mixed
/// directories prefer PLY (the sim-soft animation use case) — STLs
/// are silently skipped when a PLY sequence is present.
///
/// **Single-file dispatch**: any non-directory path falls through to
/// [`load_input`]'s extension-based dispatch (`.stl` → STL, else
/// PLY). No probing at this layer.
///
/// # Errors
///
/// Returns an error when the input directory contains neither PLY
/// sequence frames nor any STL files, naming both patterns in the
/// error so the user knows what was searched.
pub fn detect_input_mode(path: &Path) -> Result<InputMode> {
    if path.is_dir() {
        match discover_ply_sequence(path) {
            Ok(frames) => return Ok(InputMode::Sequence(Sequence::new(frames))),
            Err(_ply_err) => {
                // Fall through to STL discovery; if that also fails,
                // bubble a combined error that names both patterns.
            }
        }
        match discover_stl_sequence(path) {
            Ok(frames) => Ok(InputMode::Sequence(Sequence::new(frames))),
            Err(_stl_err) => bail!(
                "no PLY sequence frames (*_step_<digits>.ply) or STL files (*.stl) found in {}",
                path.display(),
            ),
        }
    } else {
        Ok(InputMode::Single(path.to_path_buf()))
    }
}

// ---------------------------------------------------------------------------
// Scene-setup helpers — shared with `tools/cf-scan-prep` per the Stage 2.5
// design spec (`docs/SCAN_PREP_DESIGN.md` §Architectural decisions §Tool home).
// ---------------------------------------------------------------------------

/// Render-side scale factor applied uniformly to all spawned geometry so
/// sub-meter scenes lift to Bevy's pipeline-default human-scale (~1 m)
/// regime. Bevy 0.18's defaults — near plane `0.1 m`,
/// [`OrbitCamera::framing_for_aabb`]'s internal `.max(1.0)` clamp on
/// diagonal, AmbientLight brightness — were tuned for human-scale scenes;
/// at sim-soft's cm-scale (e.g. row 13's 52.6 mm bbox diagonal — the
/// BCC mesher allocates a cube of side `2 (R + margin)` around the
/// 1 cm sphere, with margin scaling per cell-size), the framing helper
/// clamps diagonal up to `1.0` and places the camera 1.5 m away —
/// geometry then renders as a single dot. Lifting the rendered scene to
/// ~1 m diagonal puts everything safely within the defaults' working
/// range. mesh-v1.0 examples already at meter scale get
/// `render_scale = 1.0` (no change).
///
/// Banked at sim-soft EXAMPLE_INVENTORY iter-12 as the cf-view application
/// of inventory iter-11 pattern (b) (RENDER_SCALE-as-rendering-pipeline-
/// default-workaround); same root cause as sim-bevy-soft's row-12 +
/// row-13 RENDER_SCALE policy.
#[derive(Resource, Clone, Copy, Debug)]
pub struct RenderScale(pub f32);

/// Compute the render scale from the raw bbox diagonal: lift sub-meter
/// scenes to a 1 m target diagonal; meter+ scenes render at native scale.
/// Degenerate (zero / non-finite) diagonals fall back to `1.0` so the
/// downstream framing helper's own clamp handles them.
#[must_use]
pub fn compute_render_scale(raw_diagonal: f32) -> f32 {
    const TARGET_DIAGONAL: f32 = 1.0;
    if !raw_diagonal.is_finite() || raw_diagonal <= 0.0 || raw_diagonal >= TARGET_DIAGONAL {
        1.0
    } else {
        TARGET_DIAGONAL / raw_diagonal
    }
}

/// Apply a uniform scale factor to an [`Aabb`]'s corners. Used to compute
/// the camera-framing AABB at render scale (the rendered geometry's
/// bbox), distinct from the loaded mesh's physics-scale AABB.
#[must_use]
pub fn scale_aabb(raw: &Aabb, scale: f32) -> Aabb {
    let s = scale as f64;
    Aabb::from_corners(
        Point3::new(raw.min.x * s, raw.min.y * s, raw.min.z * s),
        Point3::new(raw.max.x * s, raw.max.y * s, raw.max.z * s),
    )
}

/// Spawn the orbit camera + directional key light + global ambient light,
/// framed on the supplied AABB (which the caller has already scaled via
/// [`scale_aabb`] to match the rendered geometry's bbox).
///
/// `up` controls the input-frame → Bevy-Y-up swap that
/// [`OrbitCamera::framing_for_aabb`] applies to the camera target so the
/// directional light's anchor stays aligned under any `--up=<...>`.
///
/// Light placement uses a clamped diagonal (`max(1.0)`) so single-point
/// degenerate AABBs + very small bboxes don't park the light below the
/// visible-on-screen floor.
// f64 → f32 is intentional for Bevy.
#[allow(clippy::cast_possible_truncation)]
pub fn setup_camera_and_lighting(commands: &mut Commands, scaled_aabb: &Aabb, up: UpAxis) {
    let center_physics = scaled_aabb.center();
    // Same input → Bevy frame swap as `build_face_mesh` so the light
    // anchor stays aligned with the rendered geometry under any `--up=<...>`.
    let center_bevy = Vec3::from_array(up.to_bevy_point(&center_physics));
    let diagonal = (scaled_aabb.diagonal() as f32).max(1.0);

    // Orbit camera framed corner-on at 1.5 × diagonal. `framing_for_aabb`
    // mirrors the up-axis swap so the camera target tracks the rendered
    // geometry's center.
    let orbit = OrbitCamera::framing_for_aabb(scaled_aabb, up);
    let mut transform = Transform::default();
    orbit.apply_to_transform(&mut transform);
    commands.spawn((Camera3d::default(), orbit, transform));

    // Strong directional key light + bright global ambient so geometry
    // stays readable in the geometry-only path. Bevy 0.18: `AmbientLight`
    // is now per-camera; world-wide ambient is `GlobalAmbientLight`
    // (sim-bevy `scene.rs:101` precedent).
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_translation(center_bevy + Vec3::new(diagonal, diagonal * 2.0, diagonal))
            .looking_at(center_bevy, Vec3::Y),
    ));
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 1_200.0,
        ..default()
    });
}

/// Spawn a flat-shaded face-mesh entity from an [`IndexedMesh`] (STL-style
/// input, no stored normals, no PLY per-vertex scalars). The Bevy mesh
/// build delegates to [`cf_bevy_common::mesh::triangle_mesh_flat_shaded`]
/// for one face-normal per triangle (WYSIWYP rendering).
///
/// Returns the spawned [`Entity`] so the caller can stamp marker
/// components for the despawn-on-change pattern (`cf-view` uses
/// [`ui::GeometryEntity`]; `cf-scan-prep` will use its own scan-mesh
/// marker). The helper deliberately does NOT attach a marker —
/// responsibility for entity lifecycle stays with the caller.
///
/// `vertex_colors`, when supplied, toggles the spawned material to
/// `unlit = true` so the colormap-painted hue isn't overwritten by PBR
/// shading. Length must match `indexed_mesh.vertices.len()` (debug-
/// asserted inside [`triangle_mesh_flat_shaded`]).
///
/// `cf-view`'s PLY-stored-normals dispatch keeps its own inline path
/// via [`mesh::build_face_mesh`] — that wrapper handles the
/// `Option<mesh.normals>` branch and lives in `main.rs`'s
/// `spawn_geometry` system.
pub fn spawn_face_mesh(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    indexed_mesh: &IndexedMesh,
    vertex_colors: Option<&[[f32; 4]]>,
    up: UpAxis,
    transform: Transform,
) -> Entity {
    let unlit = vertex_colors.is_some();
    let material = StandardMaterial {
        base_color: Color::srgb(0.70, 0.72, 0.78),
        metallic: 0.10,
        perceptual_roughness: 0.6,
        double_sided: true,
        cull_mode: None,
        unlit,
        ..default()
    };
    let bevy_mesh = triangle_mesh_flat_shaded(indexed_mesh, vertex_colors, up);
    commands
        .spawn((
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(materials.add(material)),
            transform,
        ))
        .id()
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::fs::File;

    use mesh_io::save_ply_attributed;
    use mesh_types::{IndexedMesh, Point3};

    #[test]
    fn sequence_frame_name_accepts_zero_padded_step_suffix() {
        assert!(is_sequence_frame_name(
            "sleeve_design_surface_deformed_step_01.ply"
        ));
        assert!(is_sequence_frame_name(
            "sleeve_design_surface_deformed_step_16.ply"
        ));
        // Single-digit + multi-digit both legal.
        assert!(is_sequence_frame_name("frame_step_0.ply"));
        assert!(is_sequence_frame_name("frame_step_1234.ply"));
    }

    #[test]
    fn sequence_frame_name_rejects_non_matching() {
        // No `_step_` marker.
        assert!(!is_sequence_frame_name("sleeve_design_surface_final.ply"));
        // Not a PLY.
        assert!(!is_sequence_frame_name("sleeve_step_01.json"));
        // Marker present but no digits after.
        assert!(!is_sequence_frame_name("frame_step_.ply"));
        // Non-digit suffix (alpha).
        assert!(!is_sequence_frame_name("frame_step_final.ply"));
        // Mixed alpha+digit suffix.
        assert!(!is_sequence_frame_name("frame_step_01a.ply"));
        // Slab-cut convention (no step suffix).
        assert!(!is_sequence_frame_name("sleeve_slab_cut_x0_final.ply"));
    }

    #[test]
    fn discover_ply_sequence_filters_and_sorts() -> Result<()> {
        let dir = tempfile::tempdir()?;
        // Mix of matching + non-matching files; deliberately create out
        // of lex order so the sort step is meaningful.
        for name in [
            "sleeve_design_surface_deformed_step_03.ply",
            "sleeve_design_surface_deformed_step_01.ply",
            "sleeve_design_surface_deformed_step_02.ply",
            "sleeve_design_surface_final.ply", // skipped: no step
            "sleeve_slab_cut_x0_final.ply",    // skipped: no step
            "ramp_curve.json",                 // skipped: not .ply
        ] {
            File::create(dir.path().join(name))?;
        }
        let frames = discover_ply_sequence(dir.path())?;
        let names: Vec<String> = frames
            .iter()
            .filter_map(|p| p.file_name().and_then(|n| n.to_str()).map(String::from))
            .collect();
        assert_eq!(
            names,
            vec![
                "sleeve_design_surface_deformed_step_01.ply".to_string(),
                "sleeve_design_surface_deformed_step_02.ply".to_string(),
                "sleeve_design_surface_deformed_step_03.ply".to_string(),
            ],
        );
        Ok(())
    }

    #[test]
    fn discover_ply_sequence_errors_on_no_matches() -> Result<()> {
        let dir = tempfile::tempdir()?;
        // Files present, but none match the step pattern.
        for name in ["body_final.ply", "readout.json", "slab_cut_z0.ply"] {
            File::create(dir.path().join(name))?;
        }
        let err = discover_ply_sequence(dir.path()).err();
        assert!(err.is_some(), "expected error for no matching frames");
        let msg = err.map(|e| e.to_string()).unwrap_or_default();
        assert!(
            msg.contains("no PLY sequence frames"),
            "error should mention missing frames: {msg}",
        );
        Ok(())
    }

    #[test]
    fn discover_ply_sequence_errors_on_missing_dir() {
        let result = discover_ply_sequence(Path::new("/nonexistent/path/for/test"));
        assert!(result.is_err(), "expected error for unreadable dir");
    }

    #[test]
    fn detect_input_mode_returns_single_for_file_input() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let file_path = dir.path().join("one_off.ply");
        File::create(&file_path)?;
        let mode = detect_input_mode(&file_path)?;
        assert!(
            matches!(&mode, InputMode::Single(p) if p == &file_path),
            "expected Single({}); got {mode:?}",
            file_path.display(),
        );
        assert_eq!(mode.initial_frame()?, file_path);
        Ok(())
    }

    #[test]
    fn detect_input_mode_returns_sequence_for_dir_input() -> Result<()> {
        let dir = tempfile::tempdir()?;
        for name in [
            "body_step_02.ply",
            "body_step_00.ply",
            "body_step_01.ply",
            "body_final.ply",
        ] {
            File::create(dir.path().join(name))?;
        }
        let mode = detect_input_mode(dir.path())?;
        let seq = match &mode {
            InputMode::Sequence(s) => s,
            InputMode::Single(_) => {
                return Err(anyhow::anyhow!(
                    "expected Sequence variant for dir input; got Single",
                ));
            }
            InputMode::Assembly { .. } => {
                return Err(anyhow::anyhow!(
                    "expected Sequence variant for dir input; got Assembly",
                ));
            }
        };
        assert_eq!(seq.len(), 3, "should see 3 step frames + 1 skipped");
        assert_eq!(seq.current, 0);
        let initial = mode.initial_frame()?;
        assert_eq!(
            initial.file_name().and_then(|n| n.to_str()),
            Some("body_step_00.ply"),
        );
        Ok(())
    }

    // ----- STL discovery + dispatch -----------------------------------

    #[test]
    fn is_stl_path_matches_extension_case_insensitive() {
        assert!(is_stl_path(Path::new("foo.stl")));
        assert!(is_stl_path(Path::new("/abs/path/MOLD.STL")));
        assert!(is_stl_path(Path::new("mixed.Stl")));
        assert!(!is_stl_path(Path::new("foo.ply")));
        assert!(!is_stl_path(Path::new("foo.json")));
        assert!(!is_stl_path(Path::new("no_extension")));
    }

    #[test]
    fn discover_stl_sequence_lex_sorts_all_stl_files() -> Result<()> {
        // No `_step_<digits>` convention — any *.stl qualifies.
        // Out-of-order creation so the sort is meaningful.
        let dir = tempfile::tempdir()?;
        for name in [
            "plug.stl",
            "mold_layer_0_piece_1.stl",
            "mold_layer_0_piece_0.stl",
            "mold_layer_1_piece_0.stl",
            "readout.json", // skipped: not .stl
        ] {
            File::create(dir.path().join(name))?;
        }
        let frames = discover_stl_sequence(dir.path())?;
        let names: Vec<String> = frames
            .iter()
            .filter_map(|p| p.file_name().and_then(|n| n.to_str()).map(String::from))
            .collect();
        assert_eq!(
            names,
            vec![
                "mold_layer_0_piece_0.stl".to_string(),
                "mold_layer_0_piece_1.stl".to_string(),
                "mold_layer_1_piece_0.stl".to_string(),
                "plug.stl".to_string(),
            ],
        );
        Ok(())
    }

    #[test]
    fn discover_stl_sequence_errors_on_no_matches() -> Result<()> {
        let dir = tempfile::tempdir()?;
        for name in ["body.ply", "readout.json"] {
            File::create(dir.path().join(name))?;
        }
        let err = discover_stl_sequence(dir.path()).err();
        let msg = err.map(|e| e.to_string()).unwrap_or_default();
        assert!(
            msg.contains("no STL frames"),
            "error should mention missing STL frames: {msg}",
        );
        Ok(())
    }

    #[test]
    fn detect_input_mode_falls_back_to_stl_when_no_ply_sequence() -> Result<()> {
        // STL-only directory: detect_input_mode tries PLY first, sees
        // no `*_step_<digits>.ply` matches, falls through to
        // `discover_stl_sequence`. Returns Sequence variant either way.
        let dir = tempfile::tempdir()?;
        for name in ["mold_layer_0_piece_0.stl", "mold_layer_0_piece_1.stl"] {
            File::create(dir.path().join(name))?;
        }
        let mode = detect_input_mode(dir.path())?;
        let seq = match &mode {
            InputMode::Sequence(s) => s,
            InputMode::Single(_) => {
                return Err(anyhow::anyhow!(
                    "expected Sequence for STL dir input; got Single",
                ));
            }
            InputMode::Assembly { .. } => {
                return Err(anyhow::anyhow!(
                    "expected Sequence for STL dir input; got Assembly",
                ));
            }
        };
        assert_eq!(seq.len(), 2);
        let initial = mode.initial_frame()?;
        assert_eq!(
            initial.file_name().and_then(|n| n.to_str()),
            Some("mold_layer_0_piece_0.stl"),
        );
        Ok(())
    }

    #[test]
    fn detect_input_mode_prefers_ply_when_both_present() -> Result<()> {
        // Mixed-content directory with BOTH PLY frames AND STL files:
        // PLY-sequence detection wins; STLs are silently skipped.
        // (The sim-soft animation use case dominates the directory-input
        // expectation; STL fallback only kicks in when there's no PLY
        // animation to play.)
        let dir = tempfile::tempdir()?;
        for name in ["body_step_00.ply", "body_step_01.ply", "extra_geometry.stl"] {
            File::create(dir.path().join(name))?;
        }
        let mode = detect_input_mode(dir.path())?;
        let seq = match &mode {
            InputMode::Sequence(s) => s,
            InputMode::Single(_) => {
                return Err(anyhow::anyhow!(
                    "expected Sequence for mixed dir input; got Single",
                ));
            }
            InputMode::Assembly { .. } => {
                return Err(anyhow::anyhow!(
                    "expected Sequence for mixed dir input; got Assembly",
                ));
            }
        };
        // PLY frames win — STL is excluded.
        assert_eq!(seq.len(), 2);
        for frame in &seq.frames {
            assert!(
                frame.extension().and_then(|e| e.to_str()) == Some("ply"),
                "PLY-mode should not include STL frames: {frame:?}",
            );
        }
        Ok(())
    }

    #[test]
    fn detect_input_mode_errors_naming_both_patterns_on_empty_dir() -> Result<()> {
        // Empty directory: neither PLY nor STL discovery succeeds.
        // Error message must name BOTH patterns so the user knows what
        // was searched.
        let dir = tempfile::tempdir()?;
        for name in ["readout.json", "notes.txt"] {
            File::create(dir.path().join(name))?;
        }
        let err = detect_input_mode(dir.path()).err();
        let msg = err.map(|e| e.to_string()).unwrap_or_default();
        assert!(
            msg.contains("PLY sequence frames") && msg.contains("STL files"),
            "error should name both patterns: {msg}",
        );
        Ok(())
    }

    #[test]
    fn input_mode_assembly_initial_frame_is_first_stl() -> Result<()> {
        let stls = vec![
            PathBuf::from("/tmp/mold_layer_0_piece_0.stl"),
            PathBuf::from("/tmp/mold_layer_0_piece_1.stl"),
            PathBuf::from("/tmp/plug.stl"),
        ];
        let mode = InputMode::Assembly { stls: stls.clone() };
        let initial = mode.initial_frame()?;
        assert_eq!(initial, stls[0]);
        Ok(())
    }

    #[test]
    fn input_mode_assembly_initial_frame_errors_on_empty() {
        let mode = InputMode::Assembly { stls: Vec::new() };
        let result = mode.initial_frame();
        assert!(result.is_err());
        assert!(
            result
                .err()
                .map(|e| e.to_string())
                .unwrap_or_default()
                .contains("assembly has no STLs")
        );
    }

    #[test]
    fn load_assembly_inputs_loads_every_stl_and_captures_basenames() -> Result<()> {
        // Round-trip via save_stl so we have real STL files to load.
        use mesh_io::save_stl;
        use mesh_types::IndexedMesh as MTIndexedMesh;
        let dir = tempfile::tempdir()?;
        let mut paths = Vec::new();
        for name in ["piece_a.stl", "piece_b.stl", "plug.stl"] {
            let path = dir.path().join(name);
            let mut mesh = MTIndexedMesh::new();
            mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
            mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
            mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
            mesh.faces.push([0, 1, 2]);
            save_stl(&mesh, &path, true)?;
            paths.push(path);
        }
        let assembly = load_assembly_inputs(&paths)?;
        assert_eq!(assembly.pieces.len(), 3);
        // Lex order matches input order here (we passed paths in order).
        assert_eq!(assembly.pieces[0].name, "piece_a.stl");
        assert_eq!(assembly.pieces[1].name, "piece_b.stl");
        assert_eq!(assembly.pieces[2].name, "plug.stl");
        // Each piece carries a valid 3-vertex 1-face mesh.
        for piece in &assembly.pieces {
            assert_eq!(piece.mesh.geometry.vertices.len(), 3);
            assert_eq!(piece.mesh.geometry.faces.len(), 1);
        }
        Ok(())
    }

    #[test]
    fn load_input_dispatches_to_stl_loader_on_stl_extension() -> Result<()> {
        // Write a minimal valid binary STL (header + 0 faces) and verify
        // load_input picks the STL path. `mesh_io::save_stl` round-trip
        // produces a real STL we can re-load. Using the same pattern as
        // discover_stl_sequence's fixture but with valid bytes.
        use mesh_io::save_stl;
        use mesh_types::IndexedMesh as MTIndexedMesh;
        let dir = tempfile::tempdir()?;
        let stl_path = dir.path().join("trivial.stl");
        // 1-triangle STL — minimal non-empty geometry so load_stl's
        // checks pass.
        let mut mesh = MTIndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        save_stl(&mesh, &stl_path, true)?;
        let input = load_input(&stl_path)?;
        assert_eq!(input.mesh.geometry.vertices.len(), 3);
        assert_eq!(input.mesh.geometry.faces.len(), 1);
        assert!(
            input.scalar_names.is_empty(),
            "STL has no per-vertex scalars",
        );
        Ok(())
    }

    // ----- compute_render_scale ---------------------------------------

    /// Sub-meter scenes lift to the 1 m target: `0.05 m → 20×`.
    #[test]
    fn compute_render_scale_lifts_sub_meter_to_one() {
        assert!((compute_render_scale(0.05) - 20.0).abs() < 1e-6);
        assert!((compute_render_scale(0.5) - 2.0).abs() < 1e-6);
    }

    /// Meter+ scenes pass through unchanged. The `1 m` boundary itself is
    /// included in the no-lift regime (the impl gates on `>= TARGET`).
    #[test]
    fn compute_render_scale_passthrough_at_meter_plus() {
        assert_eq!(compute_render_scale(1.0), 1.0);
        assert_eq!(compute_render_scale(2.5), 1.0);
        assert_eq!(compute_render_scale(100.0), 1.0);
    }

    /// Degenerate inputs (zero, negative, NaN, Inf) fall back to `1.0` so
    /// the downstream framing helper's own `max(1.0)` clamp can handle the
    /// scene without dividing by zero here.
    #[test]
    fn compute_render_scale_falls_back_to_one_on_degenerate() {
        assert_eq!(compute_render_scale(0.0), 1.0);
        assert_eq!(compute_render_scale(-1.0), 1.0);
        assert_eq!(compute_render_scale(f32::NAN), 1.0);
        assert_eq!(compute_render_scale(f32::INFINITY), 1.0);
    }

    // ----- scale_aabb -------------------------------------------------

    /// `scale = 1.0` is the identity (load-bearing for meter+ scenes
    /// where `compute_render_scale` returns `1.0`).
    #[test]
    fn scale_aabb_identity_at_unit_scale() {
        let raw = Aabb::from_corners(Point3::new(-0.1, -0.2, -0.3), Point3::new(0.4, 0.5, 0.6));
        let scaled = scale_aabb(&raw, 1.0);
        assert_eq!(scaled.min, raw.min);
        assert_eq!(scaled.max, raw.max);
    }

    /// Uniform scaling is corner-wise linear: `min × s` and `max × s`
    /// reach the rendered-frame bbox.
    #[test]
    fn scale_aabb_scales_corners_linearly() {
        let raw = Aabb::from_corners(
            Point3::new(-0.05, -0.05, -0.05),
            Point3::new(0.05, 0.05, 0.05),
        );
        let scaled = scale_aabb(&raw, 10.0);
        assert!((scaled.min.x - -0.5).abs() < 1e-9);
        assert!((scaled.max.z - 0.5).abs() < 1e-9);
        // Diagonal scales by the same factor.
        assert!((scaled.diagonal() - raw.diagonal() * 10.0).abs() < 1e-9);
    }

    // ----- existing round-trip ---------------------------------------

    /// Round-trip a synthetic point-cloud PLY (faces empty, two scalars)
    /// through `save_ply_attributed` → [`load_input`] and verify shape.
    ///
    /// Synthetic rather than fixture-based: the natural sphere-sdf-eval
    /// fixture under `examples/sim-soft/sphere-sdf-eval/out/` is gitignored
    /// and regenerated by `cargo run -p sphere-sdf-eval`. The user-facing
    /// fixture smoke check happens via the stdout vertex/scalar print in
    /// `main.rs` during the visual-review cadence step.
    #[test]
    fn round_trip_point_cloud_with_scalars() -> Result<()> {
        let geometry = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            Vec::new(),
        );
        let mut mesh = AttributedMesh::new(geometry);
        // Two extras inserted in reverse-alphabetical order; the loader
        // should still surface them sorted via the `BTreeMap` projection.
        mesh.insert_extra("zeta", vec![-1.0, -2.0, -3.0, -4.0])?;
        mesh.insert_extra("phi", vec![1.0, 2.0, 3.0, 4.0])?;

        let dir = tempfile::tempdir()?;
        let out_path = dir.path().join("test.ply");
        save_ply_attributed(&mesh, &out_path, true)?;

        let input = load_input(&out_path)?;
        assert_eq!(input.mesh.vertex_count(), 4);
        assert_eq!(input.mesh.face_count(), 0);
        assert_eq!(
            input.scalar_names,
            vec!["phi".to_string(), "zeta".to_string()],
        );
        Ok(())
    }
}
