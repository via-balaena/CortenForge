//! `cf-scan-prep` — GUI tool for preprocessing raw 3D scans into
//! watertight cast-frame STLs for the cf-cast mold pipeline.
//!
//! Stage 2.5 of the casting roadmap (`docs/CASTING_ROADMAP.md`); closes
//! the gap between the v1 capsule fallback example and a real
//! scan-driven cast example by producing a cleaned STL + provenance
//! TOML that the cast example (`examples/cast/layered-silicone-device-v1-scan/`,
//! commit #14) consumes. Full design at `docs/SCAN_PREP_DESIGN.md`.
//!
//! ## Commit #3 scope (this file's initial shape)
//!
//! - Positional CLI arg: path to the input scan STL.
//! - `--stl-units mm|m|inch` flag (default `mm`) — STL files don't carry
//!   unit metadata; cf-cast and the rest of the workspace work in meters,
//!   so loaded vertex coordinates are scaled into meters at load.
//! - STL load via [`mesh_io::load_stl`].
//! - Bevy window rendering the raw scan via cf-viewer's
//!   [`cf_viewer::setup_camera_and_lighting`] +
//!   [`cf_viewer::spawn_face_mesh`] helpers under the cast-frame
//!   [`UpAxis::PlusZ`] convention (`+Z` is the demolding direction).
//! - The orbit camera frames the raw mesh's AABB at load.
//! - Load failures surface as a red error-text overlay in the Bevy
//!   window; the app stays open so the user can read the message before
//!   pressing Esc to exit.
//!
//! Subsequent commits add the egui side panels (Scan Info / Simplify /
//! Reorient / Recenter / Clip / Cap / Mouth / Cleaned AABB), the
//! cleaned-STL + TOML writer, and the load-time mesh-repair diagnostics.

use std::path::PathBuf;

use anyhow::Result;
use bevy::prelude::*;
use cf_bevy_common::prelude::*;
use cf_viewer::{
    RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting, spawn_face_mesh,
};
use clap::{Parser, ValueEnum};
use mesh_io::load_stl;
use mesh_types::{Bounded, IndexedMesh};

/// Cast-frame up-axis convention: `+Z` is the demolding direction
/// (`docs/SCAN_PREP_DESIGN.md` §Architectural decisions §Tool home). All
/// future panels (Reorient / Recenter / Clip / Mouth ext.) work in this
/// frame; the user adjusts the scan-frame → cast-frame transform via the
/// Reorient sliders (commit #6).
const SCAN_UP_AXIS: UpAxis = UpAxis::PlusZ;

#[derive(Parser, Debug)]
#[command(
    name = "cf-scan-prep",
    about = "Preprocess a raw 3D scan STL into a watertight cast-frame STL for cf-cast",
    version
)]
struct Cli {
    /// Path to the input scan STL.
    path: PathBuf,

    /// Unit convention assumed for the input STL's vertex coordinates.
    /// STL files carry no unit metadata; industry default is millimeters.
    /// cf-cast operates in meters, so loaded vertices are multiplied by
    /// the conversion factor at load. Surfaces in the Scan Info panel
    /// (commit #4) so the assumption stays visible — loading a meter-
    /// scale STL with default `mm` produces a 1000× too-small mesh.
    #[arg(long, value_enum, default_value_t = StlUnits::Mm)]
    stl_units: StlUnits,
}

/// Unit convention for the input STL's vertex coordinates. The selected
/// variant's [`Self::to_meters_factor`] is multiplied into every vertex
/// at load to bring coordinates into the workspace's meter convention.
#[derive(ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
enum StlUnits {
    /// Millimeters — industry default for STL exports.
    Mm,
    /// Meters — identity; no unit scaling applied.
    M,
    /// Inches — NIST-exact conversion (`1 in ≡ 0.0254 m`).
    Inch,
}

impl StlUnits {
    /// Multiplier applied to each vertex coordinate to convert the
    /// loaded STL into meters.
    fn to_meters_factor(self) -> f64 {
        match self {
            Self::Mm => 0.001,
            Self::M => 1.0,
            Self::Inch => 0.0254,
        }
    }
}

/// Bevy resource carrying the loaded scan mesh in meters (post unit
/// conversion). Owned by the render-mode `App`; absent under the
/// error-overlay path.
#[derive(Resource)]
struct ScanMesh(IndexedMesh);

/// Bevy resource carrying the load-failure message under the
/// error-overlay path. Mutually exclusive with [`ScanMesh`].
#[derive(Resource)]
struct LoadError(String);

fn main() -> Result<()> {
    let cli = Cli::parse();

    match try_load_scan(&cli) {
        Ok(scan_mesh) => run_render_app(scan_mesh),
        Err(err) => {
            let msg = format!("Failed to load {}: {err:#}", cli.path.display());
            eprintln!("{msg}");
            run_error_overlay_app(msg);
        }
    }
    Ok(())
}

/// Load the STL at `cli.path` and convert its vertex coordinates into
/// meters per `cli.stl_units`. Returns the loaded mesh on success.
///
/// # Errors
///
/// Bubbles up [`mesh_io::load_stl`]'s I/O + parse errors verbatim. The
/// caller wraps the message into the error-overlay path.
fn try_load_scan(cli: &Cli) -> Result<IndexedMesh> {
    let mut mesh = load_stl(&cli.path)?;
    scale_vertices_in_place(&mut mesh, cli.stl_units.to_meters_factor());
    Ok(mesh)
}

/// Multiply each vertex of `mesh` by `factor` in place. No-op when
/// `factor` is exactly `1.0` (skips the f64 vertex walk for the
/// `--stl-units m` identity case).
fn scale_vertices_in_place(mesh: &mut IndexedMesh, factor: f64) {
    if factor == 1.0 {
        return;
    }
    for v in &mut mesh.vertices {
        v.x *= factor;
        v.y *= factor;
        v.z *= factor;
    }
}

/// Run the render-mode Bevy app: spawn camera + lighting + the loaded
/// scan mesh, framed on its raw AABB. Returns when the window closes.
fn run_render_app(scan_mesh: IndexedMesh) {
    #[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy.
    let raw_diagonal = scan_mesh.aabb().diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);
    println!(
        "loaded {} vertices, {} faces; bbox diagonal = {raw_diagonal:.4} m; \
         render_scale = {render_scale:.2}×",
        scan_mesh.vertices.len(),
        scan_mesh.faces.len(),
    );

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-scan-prep".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SCAN_UP_AXIS)
        .insert_resource(RenderScale(render_scale))
        .insert_resource(ScanMesh(scan_mesh))
        .add_systems(Startup, setup_render_scene)
        .add_systems(Update, exit_on_esc)
        .run();
}

/// Run the error-overlay Bevy app: blank 3D scene + a red error text
/// node + a hint to press Esc. The app stays open until the user exits
/// so the load failure is visible (and recoverable: fix the path /
/// `--stl-units` and re-run).
fn run_error_overlay_app(err_msg: String) {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-scan-prep — load failed".into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(LoadError(err_msg))
        .add_systems(Startup, setup_error_overlay)
        .add_systems(Update, exit_on_esc)
        .run();
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn setup_render_scene(
    mut commands: Commands,
    scan: Res<ScanMesh>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let scale = render_scale.0;
    let raw_aabb = scan.0.aabb();
    let scaled_aabb = scale_aabb(&raw_aabb, scale);
    setup_camera_and_lighting(&mut commands, &scaled_aabb, *up);
    let entity_transform = Transform::from_scale(Vec3::splat(scale));
    let _entity = spawn_face_mesh(
        &mut commands,
        meshes.as_mut(),
        materials.as_mut(),
        &scan.0,
        None,
        *up,
        entity_transform,
    );
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn setup_error_overlay(mut commands: Commands, err: Res<LoadError>) {
    // Camera3d is required so the UI render pass has a target; the scene
    // is intentionally empty (the overlay sits over a dark backdrop).
    commands.spawn(Camera3d::default());
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(24.0),
            left: Val::Px(24.0),
            max_width: Val::Vw(80.0),
            ..default()
        },
        Text::new(err.0.clone()),
        TextColor(Color::srgb(0.95, 0.30, 0.30)),
        TextFont {
            font_size: 18.0,
            ..default()
        },
    ));
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(24.0),
            left: Val::Px(24.0),
            ..default()
        },
        Text::new("press Esc to exit"),
        TextColor(Color::srgb(0.70, 0.70, 0.70)),
        TextFont {
            font_size: 14.0,
            ..default()
        },
    ));
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use mesh_types::Point3;

    // ----- StlUnits conversion factors -------------------------------

    #[test]
    fn stl_units_mm_returns_one_thousandth() {
        assert!((StlUnits::Mm.to_meters_factor() - 0.001).abs() < f64::EPSILON);
    }

    #[test]
    fn stl_units_m_is_identity() {
        assert_eq!(StlUnits::M.to_meters_factor(), 1.0);
    }

    /// `1 inch ≡ 0.0254 m` exactly (NIST conversion). Pinned bit-exact so
    /// future changes that drift the factor surface here, not in
    /// downstream cast geometry.
    #[test]
    fn stl_units_inch_uses_nist_exact_factor() {
        assert_eq!(StlUnits::Inch.to_meters_factor(), 0.0254);
    }

    // ----- scale_vertices_in_place -----------------------------------

    fn one_triangle_at(scale: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(scale, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, scale, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, scale));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    /// `factor = 1.0` is a no-op fast path — load with `--stl-units m`
    /// must leave vertices bit-identical (avoids subtle FP drift on
    /// re-saved meter-scale STLs).
    #[test]
    fn scale_vertices_identity_is_bit_exact_no_op() {
        let mut mesh = one_triangle_at(1.234_567_891_234_567);
        let original = mesh.vertices.clone();
        scale_vertices_in_place(&mut mesh, 1.0);
        assert_eq!(mesh.vertices, original);
    }

    /// `factor = 0.001` (the `mm → m` case) walks every vertex and
    /// scales each component. Asserts on the three components of the
    /// scaled triangle independently to catch axis-confusion bugs.
    #[test]
    fn scale_vertices_mm_to_meters_scales_each_component() {
        let mut mesh = one_triangle_at(80.0); // 80 mm
        scale_vertices_in_place(&mut mesh, 0.001);
        assert!((mesh.vertices[0].x - 0.080).abs() < 1e-12);
        assert!((mesh.vertices[1].y - 0.080).abs() < 1e-12);
        assert!((mesh.vertices[2].z - 0.080).abs() < 1e-12);
    }

    /// `factor = 0.0254` (the `inch → m` case) on a unit-inch triangle
    /// lands on the NIST-exact meter coordinate; no FP drift beyond f64
    /// representation of the constant itself.
    #[test]
    fn scale_vertices_inch_to_meters_uses_nist_factor() {
        let mut mesh = one_triangle_at(1.0); // 1 inch
        scale_vertices_in_place(&mut mesh, StlUnits::Inch.to_meters_factor());
        assert_eq!(mesh.vertices[0].x, 0.0254);
        assert_eq!(mesh.vertices[1].y, 0.0254);
        assert_eq!(mesh.vertices[2].z, 0.0254);
    }

    // ----- CLI parsing ------------------------------------------------

    /// Default unit is `Mm` per the workspace convention surfaced in
    /// the Scan Info panel (commit #4). Pinned because changing the
    /// default silently 1000×-shifts every cleaned STL.
    #[test]
    fn cli_default_stl_units_is_mm() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from(["cf-scan-prep", "scan.stl"])?;
        assert_eq!(cli.stl_units, StlUnits::Mm);
        assert_eq!(cli.path, PathBuf::from("scan.stl"));
        Ok(())
    }

    /// `--stl-units m` parses to the identity-factor variant.
    #[test]
    fn cli_stl_units_m_parses_to_meters_variant() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from(["cf-scan-prep", "scan.stl", "--stl-units", "m"])?;
        assert_eq!(cli.stl_units, StlUnits::M);
        Ok(())
    }

    /// `--stl-units inch` parses to the NIST-factor variant.
    #[test]
    fn cli_stl_units_inch_parses_to_inch_variant() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from(["cf-scan-prep", "scan.stl", "--stl-units", "inch"])?;
        assert_eq!(cli.stl_units, StlUnits::Inch);
        Ok(())
    }
}
