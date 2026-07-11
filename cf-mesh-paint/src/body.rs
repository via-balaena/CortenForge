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

use bevy::mesh::VertexAttributeValues;
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

    /// Restore a saved selection, repainting the render mesh from scratch: replace the
    /// painted set with `faces`, drive **every** face to `base`, then the selected faces to
    /// `highlight`. The inverse of reading [`Self::painted`] out — a consumer that re-spawns
    /// a body (e.g. after leaving the paint view) calls it so the selection survives both as
    /// face ids and as the visible highlight. Repainting the whole buffer (rather than only
    /// the selected faces) makes it idempotent: the visible state is fully defined by
    /// `faces`, independent of the mesh's prior colors. `base`/`highlight` are the brush's
    /// two colors (e.g. [`MeshPaintConfig`](crate::MeshPaintConfig)'s
    /// `base_color`/`highlight_color`). Face ids at or beyond the source face count are
    /// dropped (from both the painted set and the mesh), so [`Self::painted`] stays a valid
    /// set of indices into `source().faces` even when restoring against a changed mesh.
    pub fn restore(
        &mut self,
        faces: impl IntoIterator<Item = usize>,
        meshes: &mut Assets<Mesh>,
        base: [f32; 4],
        highlight: [f32; 4],
    ) {
        // Drop any id past the source faces so the painted set a consumer reads back (and
        // indexes into `source().faces`) can never carry a stale/invalid index.
        let n_faces = self.source.faces.len();
        self.painted = faces.into_iter().filter(|&f| f < n_faces).collect();
        let Some(mesh) = meshes.get_mut(&self.mesh) else {
            return;
        };
        let Some(VertexAttributeValues::Float32x4(colors)) =
            mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR)
        else {
            return;
        };
        for c in colors.iter_mut() {
            *c = base;
        }
        for &f in &self.painted {
            recolor(colors, f, highlight);
        }
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

    const BASE: [f32; 4] = [0.80, 0.78, 0.72, 1.0];
    const HL: [f32; 4] = [0.90, 0.30, 0.20, 1.0];

    /// Read the render mesh's per-emitted-vertex COLOR buffer back out (empty if the mesh or
    /// its Float32x4 COLOR attribute is absent — asserted non-empty by each caller).
    fn face_colors(meshes: &Assets<Mesh>, body: &PaintBody) -> Vec<[f32; 4]> {
        match meshes
            .get(body.mesh())
            .map(|m| m.attribute(Mesh::ATTRIBUTE_COLOR))
        {
            Some(Some(VertexAttributeValues::Float32x4(c))) => c.clone(),
            _ => Vec::new(),
        }
    }

    #[test]
    fn restore_seeds_the_painted_set_and_highlights_those_faces() {
        let source = tri(); // 2 faces → 6 emitted-vertex colors
        let mut meshes = Assets::<Mesh>::default();
        let handle = meshes.add(paint_render_mesh(&source, UpAxis::PlusZ, BASE));
        let mut body = PaintBody::new("L4", source, handle);

        body.restore([1usize], &mut meshes, BASE, HL);

        assert_eq!(body.painted_count(), 1);
        assert!(body.painted().contains(&1));
        let colors = face_colors(&meshes, &body);
        assert_eq!(colors.len(), 6, "2 faces → 6 emitted-vertex colors");
        assert!(colors[0..3].iter().all(|c| *c == BASE), "face 0 stays base");
        assert!(colors[3..6].iter().all(|c| *c == HL), "face 1 highlighted");
    }

    #[test]
    fn restore_repaints_from_scratch_replacing_a_prior_selection() {
        // The idempotence guarantee: restoring a narrower set fully replaces the prior one,
        // driving the dropped face's mesh color back to base (not leaving a stale highlight).
        let source = tri();
        let mut meshes = Assets::<Mesh>::default();
        let handle = meshes.add(paint_render_mesh(&source, UpAxis::PlusZ, BASE));
        let mut body = PaintBody::new("L4", source, handle);

        body.restore([0usize, 1], &mut meshes, BASE, HL); // both faces
        body.restore([1usize], &mut meshes, BASE, HL); // now only face 1

        assert_eq!(body.painted_count(), 1);
        assert!(body.painted().contains(&1) && !body.painted().contains(&0));
        let colors = face_colors(&meshes, &body);
        assert_eq!(colors.len(), 6, "2 faces → 6 emitted-vertex colors");
        assert!(
            colors[0..3].iter().all(|c| *c == BASE),
            "face 0 must revert to base, not keep a stale highlight"
        );
        assert!(
            colors[3..6].iter().all(|c| *c == HL),
            "face 1 stays highlighted"
        );
    }

    #[test]
    fn restore_empty_selection_clears_every_face_to_base() {
        // The clearing corner of the idempotence claim: an empty selection drives the whole
        // buffer to base and leaves nothing painted.
        let source = tri();
        let mut meshes = Assets::<Mesh>::default();
        let handle = meshes.add(paint_render_mesh(&source, UpAxis::PlusZ, BASE));
        let mut body = PaintBody::new("L4", source, handle);

        body.restore([0usize, 1], &mut meshes, BASE, HL); // paint both first
        body.restore(std::iter::empty(), &mut meshes, BASE, HL); // then clear

        assert!(body.is_empty());
        let colors = face_colors(&meshes, &body);
        assert_eq!(colors.len(), 6);
        assert!(
            colors.iter().all(|c| *c == BASE),
            "all faces revert to base"
        );
    }

    #[test]
    fn restore_drops_out_of_range_face_ids() {
        // A stale/invalid id (from restoring against a changed mesh) must not enter the
        // painted set — a consumer indexes it into source().faces downstream.
        let source = tri(); // faces 0 and 1 only
        let mut meshes = Assets::<Mesh>::default();
        let handle = meshes.add(paint_render_mesh(&source, UpAxis::PlusZ, BASE));
        let mut body = PaintBody::new("L4", source, handle);

        body.restore([0usize, 99], &mut meshes, BASE, HL);

        assert_eq!(body.painted_count(), 1, "the out-of-range id 99 is dropped");
        assert!(body.painted().contains(&0) && !body.painted().contains(&99));
        let colors = face_colors(&meshes, &body);
        assert_eq!(colors.len(), 6);
        assert!(colors[0..3].iter().all(|c| *c == HL), "face 0 highlighted");
        assert!(colors[3..6].iter().all(|c| *c == BASE), "face 1 stays base");
    }
}
