//! The FSU scene entities: the two vertebrae (per-tissue bone material) and the
//! clean deforming disc surface, spawned from the assembled [`crate::scene`] and
//! then driven per frame by [`crate::replay`].

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

/// The startup meshes — consumed to build the GPU assets. The disc is the clean
/// STL surface (deformed per frame by the FEM field, not the ragged tet
/// boundary). Kept resident so the scene can be re-spawned on a state re-entry.
#[derive(Resource)]
pub(crate) struct SceneMeshes {
    pub(crate) l4: IndexedMesh,
    pub(crate) l5: IndexedMesh,
    pub(crate) disc_surface: IndexedMesh,
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

/// Spawn the FSU scene entities (both vertebrae + the deforming disc surface)
/// from [`SceneMeshes`]. The camera + lighting are spawned separately (see
/// `setup_camera` in `main`), so this owns only the tissue geometry.
pub(crate) fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scene_meshes: Res<SceneMeshes>,
) {
    // L5 (inferior) — fixed. L4 (superior) — tagged as the flexing body.
    spawn_bone(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scene_meshes.l5,
        TissuePart::L5,
    );
    let l4 = spawn_bone(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scene_meshes.l4,
        TissuePart::L4,
    );
    commands.entity(l4).insert(FlexedL4);

    // Disc — the CLEAN STL surface (a smooth watertight lens), not the ragged tet
    // boundary. `build_soft_mesh` gives it the Z-up→Y-up swap + smooth normals; built at
    // rest, `flexion_update` displaces its vertices by the FEM field each frame.
    let disc_verts: Vec<Vector3<f64>> = scene_meshes
        .disc_surface
        .vertices
        .iter()
        .map(|p| p.coords)
        .collect();
    let disc_mesh = build_soft_mesh(&disc_verts, &scene_meshes.disc_surface.faces, UpAxis::PlusZ);
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
