//! The FSU scene entities. The two vertebrae are spawned once at startup and
//! persist across modes ([`spawn_bones`] — so the 3D world is never empty and
//! previews the paint target); the deforming disc is Simulate-scoped, spawned
//! from the solved surface on entering Simulate ([`spawn_disc`]) and torn down
//! on leaving ([`leave_simulate`]), which also returns L4 to rest.

use bevy::prelude::*;
use cf_viewer::UpAxis;
use mesh_types::{AttributedMesh, IndexedMesh};
use nalgebra::Vector3;
use sim_bevy::mesh::triangle_mesh_from_attributed;
use sim_bevy_soft::mesh::build_soft_mesh;

// ── Per-tissue palette (a serious anatomical read, not toy colors). ──
const BONE_COLOR: Color = Color::srgb(0.90, 0.88, 0.82); // ivory cortical bone
const DISC_COLOR: Color = Color::srgb(0.20, 0.62, 0.68); // teal cartilage (opaque — see disc material)

/// Which tissue an entity is, so the visibility panel can toggle it.
#[derive(Component, Clone, Copy)]
pub(crate) enum TissuePart {
    L4,
    L5,
    Disc,
}

/// Marks the superior (L4) vertebra — the one flexion rotates about the pivot.
#[derive(Component)]
pub(crate) struct FlexedL4;

/// Marks the deforming disc soft-mesh entity (its positions are rewritten each frame).
#[derive(Component)]
pub(crate) struct DiscMesh;

/// The three input meshes (native mm), kept resident: L4/L5 spawn the always-on
/// bones + frame the camera, and all three are the inputs to the on-demand solve
/// (see [`crate::solve`]). In the next rung the disc field is replaced by the
/// painted-and-lofted disc.
#[derive(Resource)]
pub(crate) struct SourceMeshes {
    pub(crate) l4: IndexedMesh,
    pub(crate) l5: IndexedMesh,
    pub(crate) disc: IndexedMesh,
}

/// Build a SMOOTH-shaded Bevy mesh from an indexed surface: area-weighted
/// per-vertex normals (`AttributedMesh::compute_normals`) rendered by the
/// shared attributed converter (f64→f32 + Z-up→Y-up handled internally).
fn smooth_mesh(indexed: &IndexedMesh) -> Mesh {
    let mut attributed = AttributedMesh::from(indexed.clone());
    attributed.compute_normals();
    triangle_mesh_from_attributed(&attributed)
}

/// Spawn one bone (vertebra) surface with its material + a visibility-toggle
/// marker, returning the entity so the caller can tag the flexing one.
fn spawn_bone(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    indexed: &IndexedMesh,
    part: TissuePart,
) -> Entity {
    let material = StandardMaterial {
        base_color: BONE_COLOR,
        metallic: 0.05,
        perceptual_roughness: 0.7,
        double_sided: true,
        cull_mode: None,
        ..default()
    };
    commands
        .spawn((
            Mesh3d(meshes.add(smooth_mesh(indexed))),
            MeshMaterial3d(materials.add(material)),
            Transform::default(),
            Visibility::Visible,
            part,
        ))
        .id()
}

/// Spawn the two vertebrae (L5 fixed, L4 tagged as the flexing body) at startup.
/// They persist across all modes.
pub(crate) fn spawn_bones(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    sources: Res<SourceMeshes>,
) {
    spawn_bone(
        &mut commands,
        &mut meshes,
        &mut materials,
        &sources.l5,
        TissuePart::L5,
    );
    let l4 = spawn_bone(
        &mut commands,
        &mut meshes,
        &mut materials,
        &sources.l4,
        TissuePart::L4,
    );
    commands.entity(l4).insert(FlexedL4);
}

/// Spawn the deforming disc from the solved render surface — the clean STL lens
/// (not the ragged tet boundary); `flexion_update` rewrites its vertices by the
/// FEM field each frame. Called directly by the solve poll (so the disc + its
/// replay resources are inserted together, with no cross-schedule timing gap);
/// torn down by [`leave_simulate`].
pub(crate) fn spawn_disc(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    disc_surface: &IndexedMesh,
) {
    let disc_verts: Vec<Vector3<f64>> = disc_surface.vertices.iter().map(|p| p.coords).collect();
    let disc_mesh = build_soft_mesh(&disc_verts, &disc_surface.faces, UpAxis::PlusZ);
    // Opaque teal, double-sided (a thin closed shell reads fine as a solid lens; the
    // gap it fills doesn't need see-through, and blending an unsorted shell smears it).
    let disc_material = StandardMaterial {
        base_color: DISC_COLOR,
        metallic: 0.05,
        perceptual_roughness: 0.7,
        double_sided: true,
        cull_mode: None,
        alpha_mode: AlphaMode::Opaque,
        ..default()
    };
    commands.spawn((
        Mesh3d(meshes.add(disc_mesh)),
        MeshMaterial3d(materials.add(disc_material)),
        Transform::default(),
        Visibility::Visible,
        TissuePart::Disc,
        DiscMesh,
    ));
}

/// On leaving Simulate: despawn the disc and return L4 to rest, so Design shows
/// the bare bones at rest. The bones + camera persist (so the 3D world is never
/// empty — quitting against an empty world deadlocks wgpu teardown on macOS).
pub(crate) fn leave_simulate(
    mut commands: Commands,
    disc: Query<Entity, With<DiscMesh>>,
    mut flexed: Query<&mut Transform, With<FlexedL4>>,
) {
    for entity in &disc {
        commands.entity(entity).despawn();
    }
    for mut transform in &mut flexed {
        *transform = Transform::default();
    }
}
