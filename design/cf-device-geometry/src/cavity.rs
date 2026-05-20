//! Shared cavity-render surface — the rest-frame cavity entity that
//! both binaries spawn at startup.
//!
//! Lifted from `tools/cf-device-design/src/main.rs` per
//! `docs/SIM_DECOUPLE_PHASE_3_RECON.md` §2.5.c (§1.7 "Duplicate").
//! The cavity is rest-frame (no `InsertionSimState` dependence —
//! parent plan §4 Phase 4 step 4 keeps the rest-frame iso path in
//! cf-device-design post-strip), so the spawner lives here in the
//! shared crate and the deformed-cavity update system (which DOES
//! read sim state) lives in cf-sim-research's Phase 3 code.

use bevy::pbr::ExtendedMaterial;
use bevy::prelude::*;
use cf_bevy_common::axis::UpAxis;
use cf_device_types::CavityState;
use cf_viewer::RenderScale;

use crate::bevy_mesh::build_bevy_mesh_from_indexed;
use crate::clip_plane::{ClipPlaneExt, ClipPlaneMaterial};
use crate::sdf_layers::{self, CachedScanSdf, CapPlanes};

/// Marker component for the cavity mesh entity. One entity at
/// runtime.
#[derive(Component)]
pub struct CavityEntity;

/// Cavity surface color (`StandardMaterial::base_color`). The
/// cavity is the inner void surface; coral distinguishes it from
/// the layer palette + scan + axis arrows. Material is double-
/// sided so the user sees the inside when layers are hidden.
pub const CAVITY_COLOR: (f32, f32, f32) = (0.95, 0.55, 0.45);

/// Spawn the cavity mesh entity at startup. Layer entities spawn
/// lazily on the first state-change tick.
///
/// Extracts the cavity surface from the cached scan SDF at
/// `iso = -cavity.inset_m` (inward offset by the inset distance).
/// The SDF iso is exactly perpendicular to the source surface by
/// construction (no apex-nipple artifacts).
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
pub fn spawn_cavity_mesh(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ClipPlaneMaterial>>,
    cached_sdf: Res<CachedScanSdf>,
    cap_planes: Res<CapPlanes>,
    cavity: Res<CavityState>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
) {
    let cavity_indexed =
        sdf_layers::extract_layer_surface(&cached_sdf, &cap_planes.planes, -cavity.inset_m);
    let cavity_mesh = meshes.add(build_bevy_mesh_from_indexed(
        &cavity_indexed,
        *up,
        render_scale.0,
    ));
    let cavity_material = materials.add(ExtendedMaterial {
        base: StandardMaterial {
            base_color: Color::srgb(CAVITY_COLOR.0, CAVITY_COLOR.1, CAVITY_COLOR.2),
            double_sided: true,
            cull_mode: None,
            ..default()
        },
        extension: ClipPlaneExt::default(),
    });
    commands.spawn((
        Mesh3d(cavity_mesh),
        MeshMaterial3d(cavity_material),
        CavityEntity,
    ));
}
