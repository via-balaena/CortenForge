//! Paintable bodies and the render-mesh contract the brush paints on.
//!
//! A consumer registers each paintable body as a [`PaintBody`] component and
//! tracks the ordered set (with the brush-active one) in the [`PaintTargets`]
//! resource. The render mesh a body carries **must** be built with
//! [`paint_render_mesh`] — it seeds the per-face color attribute the brush
//! recolors, and it preserves the picking contract the hover ray-cast relies
//! on: `cf_bevy_common::mesh::triangle_mesh_flat_shaded` emits three vertices
//! per face **in face order**, so a mesh ray hit's `triangle_index` is exactly
//! the source [`IndexedMesh`] face id.

use std::collections::HashSet;

use bevy::prelude::*;
use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::mesh::triangle_mesh_flat_shaded;
use cf_geometry::IndexedMesh;
use mesh_select::FaceField;

/// A paintable body: its source mesh, the render-mesh handle whose per-face
/// colors the brush drives, the precomputed [`FaceField`] the brush queries
/// against, and the set of painted face ids.
///
/// Construct with [`PaintBody::new`], spawn it alongside a `Mesh3d` built by
/// [`paint_render_mesh`] and the material of your choice, then register the
/// entity in [`PaintTargets`]. Read the selection back out with
/// [`PaintBody::painted`] + [`PaintBody::source`].
#[derive(Component)]
pub struct PaintBody {
    pub(crate) name: String,
    pub(crate) source: IndexedMesh,
    pub(crate) mesh: Handle<Mesh>,
    pub(crate) field: FaceField,
    pub(crate) painted: HashSet<usize>,
}

impl PaintBody {
    /// Build a paintable body from its `source` mesh and the handle of its
    /// render mesh (which must come from [`paint_render_mesh`]). The
    /// [`FaceField`] the brush queries is computed here, once.
    #[must_use]
    pub fn new(name: impl Into<String>, source: IndexedMesh, mesh: Handle<Mesh>) -> Self {
        let field = FaceField::new(&source);
        Self {
            name: name.into(),
            source,
            mesh,
            field,
            painted: HashSet::new(),
        }
    }

    /// The body's display name (used by consumer HUDs to label it).
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The source mesh the painted face ids index into — the geometry a
    /// consumer feeds downstream (e.g. loft the painted patch).
    #[must_use]
    pub fn source(&self) -> &IndexedMesh {
        &self.source
    }

    /// The set of currently painted face ids (indices into `source().faces`).
    #[must_use]
    pub fn painted(&self) -> &HashSet<usize> {
        &self.painted
    }

    /// How many faces are currently painted.
    #[must_use]
    pub fn painted_count(&self) -> usize {
        self.painted.len()
    }

    /// Whether nothing is painted yet.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.painted.is_empty()
    }

    /// Handle of the render mesh whose per-face colors the brush drives.
    #[must_use]
    pub fn mesh(&self) -> &Handle<Mesh> {
        &self.mesh
    }
}

/// The registered paintable bodies, in order, and which one the brush acts on
/// (cycled with `Tab`). A consumer builds this after spawning its bodies with
/// [`PaintTargets::new`]; the plugin inserts an empty default so its systems
/// run safely before registration.
#[derive(Resource, Default)]
pub struct PaintTargets {
    entities: Vec<Entity>,
    active: usize,
}

impl PaintTargets {
    /// Register `entities` in order, with the first one active.
    #[must_use]
    pub fn new(entities: Vec<Entity>) -> Self {
        Self {
            entities,
            active: 0,
        }
    }

    /// The brush-active entity, or `None` if nothing is registered.
    #[must_use]
    pub fn active_entity(&self) -> Option<Entity> {
        self.entities.get(self.active).copied()
    }

    /// The registered entities, in registration order.
    #[must_use]
    pub fn entities(&self) -> &[Entity] {
        &self.entities
    }

    /// The index of the active body.
    #[must_use]
    pub fn active_index(&self) -> usize {
        self.active
    }

    /// Advance the active body to the next registered one (wrapping). No-op
    /// when nothing is registered.
    pub fn cycle(&mut self) {
        if !self.entities.is_empty() {
            self.active = (self.active + 1) % self.entities.len();
        }
    }

    /// Make `entity` active if it is registered.
    pub fn set_active(&mut self, entity: Entity) {
        if let Some(index) = self.entities.iter().position(|&e| e == entity) {
            self.active = index;
        }
    }
}

/// Build a body's render mesh: a flat-shaded [`IndexedMesh`] → Bevy `Mesh` with
/// a per-face color attribute seeded to `base_color`, oriented by `up`.
///
/// This is the one supported way to build a [`PaintBody`]'s render mesh: it
/// owns the picking contract (three vertices per face, in face order) and the
/// base-color convention the brush restores on erase.
#[must_use]
pub fn paint_render_mesh(source: &IndexedMesh, up: UpAxis, base_color: [f32; 4]) -> Mesh {
    let seed = vec![base_color; source.vertices.len()];
    triangle_mesh_flat_shaded(source, Some(&seed), up)
}

/// Set the three emitted vertices of face `f` (`3f, 3f+1, 3f+2`) to `color`.
pub(crate) fn recolor(colors: &mut [[f32; 4]], f: usize, color: [f32; 4]) {
    let base = f * 3;
    for k in 0..3 {
        if let Some(slot) = colors.get_mut(base + k) {
            *slot = color;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri() -> IndexedMesh {
        // Two triangles sharing an edge — enough for a FaceField + face ids 0,1.
        IndexedMesh::from_raw(
            &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0],
            &[0, 1, 2, 1, 3, 2],
        )
    }

    #[test]
    fn new_body_starts_empty() {
        let body = PaintBody::new("L4", tri(), Handle::default());
        assert_eq!(body.name(), "L4");
        assert!(body.is_empty());
        assert_eq!(body.painted_count(), 0);
        assert_eq!(body.source().faces.len(), 2);
    }

    #[test]
    fn targets_cycle_wraps_and_reports_active() {
        let mut world = World::new();
        let (a, b) = (world.spawn_empty().id(), world.spawn_empty().id());
        let mut t = PaintTargets::new(vec![a, b]);
        assert_eq!(t.active_index(), 0);
        assert_eq!(t.active_entity(), Some(a));
        t.cycle();
        assert_eq!(t.active_entity(), Some(b));
        t.cycle();
        assert_eq!(t.active_entity(), Some(a));
    }

    #[test]
    fn empty_targets_have_no_active_and_cycle_is_noop() {
        let mut t = PaintTargets::default();
        assert_eq!(t.active_entity(), None);
        t.cycle();
        assert_eq!(t.active_entity(), None);
    }

    #[test]
    fn set_active_selects_registered_entity_only() {
        let mut world = World::new();
        let (a, b, unregistered) = (
            world.spawn_empty().id(),
            world.spawn_empty().id(),
            world.spawn_empty().id(),
        );
        let mut t = PaintTargets::new(vec![a, b]);
        t.set_active(b);
        assert_eq!(t.active_index(), 1);
        t.set_active(unregistered); // not registered — no change
        assert_eq!(t.active_index(), 1);
    }

    #[test]
    fn paint_render_mesh_seeds_base_color_per_emitted_vertex() {
        let mesh = paint_render_mesh(&tri(), UpAxis::PlusZ, [0.8, 0.78, 0.72, 1.0]);
        // Flat shading emits three vertices per face: 2 faces → 6 colors, all
        // seeded to the base color.
        let seeded = matches!(
            mesh.attribute(Mesh::ATTRIBUTE_COLOR),
            Some(bevy::mesh::VertexAttributeValues::Float32x4(c))
                if c.len() == 6 && c.iter().all(|v| *v == [0.8, 0.78, 0.72, 1.0])
        );
        assert!(
            seeded,
            "render mesh must carry a base-seeded Float32x4 COLOR"
        );
    }

    #[test]
    fn recolor_sets_exactly_one_face_triple() {
        let mut colors = vec![[0.0; 4]; 6];
        recolor(&mut colors, 1, [1.0, 0.0, 0.0, 1.0]);
        assert_eq!(&colors[0..3], &[[0.0; 4]; 3]); // face 0 untouched
        assert!(colors[3..6].iter().all(|c| *c == [1.0, 0.0, 0.0, 1.0]));
    }

    #[test]
    fn recolor_out_of_range_face_is_ignored() {
        let mut colors = vec![[0.0; 4]; 6];
        recolor(&mut colors, 99, [1.0, 0.0, 0.0, 1.0]); // no panic, no change
        assert!(colors.iter().all(|c| *c == [0.0; 4]));
    }
}
