//! The scene entities across modes. Design/Building/Preview show the two vertebrae as
//! **paintable** bodies ([`spawn_paint_bodies`]) — Preview adds the static conformed disc
//! ([`spawn_preview_disc`]); Simulate shows them as smooth flexing bones plus the deforming
//! disc ([`spawn_bones`] + [`spawn_disc`]). The two vertebra representations are per-mode so
//! the paint mesh (flat-shaded, per-vertex colourable) and the anatomical bone mesh (smooth)
//! each get their own render.
//!
//! ★ Never-empty-render invariant (the macOS wgpu quit-deadlock guard): a drawn mesh exists
//! in every mode where quit is reachable — the paint bodies in Design/Building/Preview/
//! Solving, the bones+disc in Simulate — and the swaps happen within one `StateTransition`
//! command flush, so no frame is empty.

use bevy::prelude::*;
use cf_mesh_paint::prelude::{MeshPaintConfig, PaintBody, PaintTargets, paint_render_mesh};
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

/// Marks the static **Preview** disc — the conformed render surface shown after the ~5 s
/// build, while the user decides whether to run the ~85 s capture. Kept distinct from the
/// deforming [`DiscMesh`] so it is despawned when leaving the built state (entering Design
/// or Simulate) without touching the sim disc.
#[derive(Component)]
pub(crate) struct PreviewDisc;

/// The two vertebra meshes (native mm), kept resident: they seed the Design
/// paint bodies + the Simulate bones + frame the camera, and are two of the
/// three inputs to the on-demand solve (the disc is painted, not loaded).
#[derive(Resource)]
pub(crate) struct SourceMeshes {
    pub(crate) l4: IndexedMesh,
    pub(crate) l5: IndexedMesh,
}

/// Build a SMOOTH-shaded Bevy mesh from an indexed surface: area-weighted
/// per-vertex normals (`AttributedMesh::compute_normals`) rendered by the
/// shared attributed converter (f64→f32 + Z-up→Y-up handled internally).
fn smooth_mesh(indexed: &IndexedMesh) -> Mesh {
    let mut attributed = AttributedMesh::from(indexed.clone());
    attributed.compute_normals();
    triangle_mesh_from_attributed(&attributed)
}

/// Spawn the two vertebrae as **paintable bodies** — the Design representation.
/// Each carries a flat-shaded, per-vertex-colourable render mesh (the picking
/// contract [`cf_mesh_paint`] needs) and a [`PaintBody`]; the ordered set goes
/// into [`PaintTargets`]. Run at startup (initial Design) and on leaving
/// Simulate. Despawned by [`despawn_paint_bodies`] on entering Simulate.
pub(crate) fn spawn_paint_bodies(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    sources: Res<SourceMeshes>,
) {
    // Seed each render mesh with the base colour the brush restores on erase.
    let base = MeshPaintConfig::default().base_color;
    let mut entities = Vec::new();
    for (mesh, name) in [(&sources.l4, "L4"), (&sources.l5, "L5")] {
        let handle = meshes.add(paint_render_mesh(mesh, UpAxis::PlusZ, base));
        let material = materials.add(StandardMaterial {
            base_color: Color::WHITE, // per-vertex face colours show through
            perceptual_roughness: 0.85,
            double_sided: true,
            cull_mode: None,
            ..default()
        });
        let entity = commands
            .spawn((
                Mesh3d(handle.clone()),
                MeshMaterial3d(material),
                Transform::default(),
                PaintBody::new(name, mesh.clone(), handle),
            ))
            .id();
        entities.push(entity);
    }
    commands.insert_resource(PaintTargets::new(entities));
}

/// Despawn the paint bodies on entering Simulate (the smooth bones take over).
pub(crate) fn despawn_paint_bodies(mut commands: Commands, bodies: Query<Entity, With<PaintBody>>) {
    for entity in &bodies {
        commands.entity(entity).despawn();
    }
}

/// Show only the **active** paint body (hide the other), so the vertebra you are
/// painting isn't occluded by its neighbour — `Tab` (which cycles the active
/// body) swaps which one is visible. Runs while painting (Design).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn update_paint_visibility(
    targets: Res<PaintTargets>,
    mut bodies: Query<(Entity, &mut Visibility), With<PaintBody>>,
) {
    let active = targets.active_entity();
    for (entity, mut visibility) in &mut bodies {
        *visibility = if Some(entity) == active {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Show BOTH paint bodies on entering Preview. Design hides the inactive vertebra while
/// painting (see [`update_paint_visibility`]); Preview wants both visible so the conformed
/// disc reads as seated between the two vertebrae.
pub(crate) fn show_paint_bodies(mut bodies: Query<&mut Visibility, With<PaintBody>>) {
    for mut visibility in &mut bodies {
        *visibility = Visibility::Visible;
    }
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

/// Spawn the two smooth vertebrae (L5 fixed, L4 tagged as the flexing body) on
/// entering Simulate — the anatomical representation. Torn down (with the disc)
/// by [`despawn_sim_scene`] on leaving.
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

/// Build the teal disc render mesh from a conformed surface — the same soft-mesh
/// construction the deforming disc uses, so the static Preview disc and the animated
/// Simulate disc are visually identical (only the sim rewrites its vertices per frame).
fn disc_mesh(disc_surface: &IndexedMesh) -> Mesh {
    let disc_verts: Vec<Vector3<f64>> = disc_surface.vertices.iter().map(|p| p.coords).collect();
    build_soft_mesh(&disc_verts, &disc_surface.faces, UpAxis::PlusZ)
}

/// The disc material: opaque teal, double-sided (a thin closed shell reads fine as a
/// solid lens; the gap it fills doesn't need see-through, and blending an unsorted shell
/// smears it). Shared by the Preview and Simulate discs.
fn disc_material() -> StandardMaterial {
    StandardMaterial {
        base_color: DISC_COLOR,
        metallic: 0.05,
        perceptual_roughness: 0.7,
        double_sided: true,
        cull_mode: None,
        alpha_mode: AlphaMode::Opaque,
        ..default()
    }
}

/// Spawn the deforming disc from the solved render surface — the clean STL lens
/// (not the ragged tet boundary); `flexion_update` rewrites its vertices by the
/// FEM field each frame. Called directly by the solve poll (so the disc + its
/// replay resources are inserted together, with no cross-schedule timing gap).
pub(crate) fn spawn_disc(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    disc_surface: &IndexedMesh,
) {
    commands.spawn((
        Mesh3d(meshes.add(disc_mesh(disc_surface))),
        MeshMaterial3d(materials.add(disc_material())),
        Transform::default(),
        Visibility::Visible,
        TissuePart::Disc,
        DiscMesh,
    ));
}

/// Spawn the static **Preview** disc on entering Preview — the conformed render surface
/// from the held build, shown (over the resident paint bodies) so the user sees the real
/// lofted + conformed disc before committing to the ~85 s capture. Despawned by
/// [`despawn_preview_disc`] on leaving the built state.
pub(crate) fn spawn_preview_disc(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    held: NonSend<crate::solve::HeldBuildSlot>,
) {
    let Some(build) = held.0.as_ref() else {
        return; // no held build (shouldn't happen in Preview) — nothing to show
    };
    commands.spawn((
        Mesh3d(meshes.add(disc_mesh(&build.disc_surface))),
        MeshMaterial3d(materials.add(disc_material())),
        Transform::default(),
        Visibility::Visible,
        PreviewDisc,
    ));
}

/// Despawn the Preview disc on leaving the built state (entering Design to repaint, or
/// Simulate where the deforming disc takes over). Harmless when none exists.
pub(crate) fn despawn_preview_disc(
    mut commands: Commands,
    discs: Query<Entity, With<PreviewDisc>>,
) {
    for entity in &discs {
        commands.entity(entity).despawn();
    }
}

/// On leaving Simulate: despawn the whole sim scene (both bones + the disc — all
/// carry [`TissuePart`]). The paint bodies are re-spawned in the same
/// `StateTransition` flush (see `run_app`), so Design is never an empty world.
pub(crate) fn despawn_sim_scene(mut commands: Commands, tissue: Query<Entity, With<TissuePart>>) {
    for entity in &tissue {
        commands.entity(entity).despawn();
    }
}
