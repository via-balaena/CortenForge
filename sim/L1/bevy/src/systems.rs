//! ECS systems for physics visualization.
//!
//! These systems synchronize Model/Data physics state with Bevy entities.
//! For the primary visualization systems, see [`crate::model_data`].

use bevy::prelude::*;

use crate::components::{CollisionShapeVisual, VisGroup};
use crate::resources::{CachedContacts, ViewerConfig};

/// Updates collision shape visibility based on configuration.
///
/// This system only runs when [`ViewerConfig`] has changed, avoiding
/// unnecessary iterations over all collision shape entities every frame.
/// Shapes are filtered by both the global `show_collision_shapes` toggle
/// and per-group visibility via [`VisGroup`].
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters are passed by value
pub fn update_shape_visibility(
    config: Res<ViewerConfig>,
    mut shapes: Query<(&mut Visibility, Option<&VisGroup>), With<CollisionShapeVisual>>,
) {
    // Early exit if config hasn't changed
    if !config.is_changed() {
        return;
    }

    for (mut vis, group) in &mut shapes {
        let group_visible = match group {
            Some(g) => {
                let idx = usize::try_from(g.0.max(0)).unwrap_or(0).min(5);
                config.show_groups[idx]
            }
            None => true, // No group component → always visible when shapes enabled
        };
        *vis = if config.show_collision_shapes && group_visible {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }
}

/// Updates the contact cache.
///
/// This system clears and updates the contact cache for use by gizmo rendering systems.
/// For Model/Data based contact caching, the contacts are computed during `step()`.
#[allow(clippy::needless_pass_by_value)] // Bevy system parameters are passed by value
pub fn update_cached_contacts(mut cached_contacts: ResMut<CachedContacts>) {
    // For Model/Data API, contacts are handled differently.
    // This system now just ensures the cache is available as a resource.
    // Users can populate it manually if needed.
    let _ = &mut cached_contacts;
}

/// System set for physics visualization.
///
/// These sets control the ordering of visualization systems:
/// 1. `TransformSync` - Updates transforms from physics state
/// 2. `ContactCache` - Caches contacts for gizmo rendering
///
/// Debug gizmos run in [`DebugGizmosSet`](crate::gizmos::DebugGizmosSet) after `ContactCache`.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SimViewerSet {
    /// Transform synchronization.
    TransformSync,
    /// Contact cache update (after physics, before gizmos).
    ContactCache,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn viewer_config_defaults() {
        let config = ViewerConfig::default();
        assert!(config.show_collision_shapes);
        assert!(config.show_contacts);
    }

    #[test]
    fn cached_contacts_operations() {
        let mut cache = CachedContacts::default();
        assert!(cache.is_empty());
        cache.clear();
        assert!(cache.is_empty());
    }
}
