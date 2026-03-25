//! Material presets and geom material overrides.
//!
//! [`MetalPreset`] provides PBR metallic/roughness presets for common surface
//! types. The `override_geom_material_*` functions replace flat MJCF-derived
//! materials with metallic PBR materials after [`spawn_model_geoms`](super::model_data::spawn_model_geoms).
//!
//! # Example
//!
//! ```rust,ignore
//! // In a Bevy system after spawn_model_geoms:
//! let steel = materials.add(MetalPreset::PolishedSteel.material());
//! let red_metal = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.8, 0.2, 0.15)));
//!
//! override_geom_material_by_name(&model, "rail", steel.clone(), &mut query);
//! override_geom_material_by_name(&model, "block", red_metal, &mut query);
//! ```

use bevy::prelude::*;
use sim_core::Model;

use crate::model_data::ModelGeomIndex;

/// Metallic material preset for common surface types.
///
/// Each preset defines metallic and roughness values. Call [`material()`](MetalPreset::material)
/// for the preset's default color, or [`with_color()`](MetalPreset::with_color)
/// to override the base color while keeping the metallic/roughness properties.
#[derive(Debug, Clone)]
pub enum MetalPreset {
    /// Polished steel: high metallic (0.9), low roughness (0.25).
    /// Default color: light grey (0.52, 0.53, 0.56).
    PolishedSteel,
    /// Brushed metal: high metallic (0.85), medium roughness (0.35).
    /// Default color: neutral grey (0.48, 0.48, 0.50).
    BrushedMetal,
    /// Cast iron: medium metallic (0.6), high roughness (0.55).
    /// Default color: dark grey (0.35, 0.33, 0.32).
    CastIron,
    /// Spring/tendon wire: steel with slight warmth.
    /// High metallic (0.85), low-medium roughness (0.35).
    /// Default color: warm grey (0.55, 0.58, 0.60).
    SpringWire,
    /// Anodized aluminum with a tint color.
    /// High metallic (0.7), low roughness (0.3).
    /// Color provided by caller.
    Anodized(Color),
}

impl MetalPreset {
    /// Create a [`StandardMaterial`] with this preset's default color.
    #[must_use]
    pub fn material(&self) -> StandardMaterial {
        let (color, metallic, roughness) = self.properties();
        StandardMaterial {
            base_color: color,
            metallic,
            perceptual_roughness: roughness,
            ..default()
        }
    }

    /// Create a [`StandardMaterial`] with a custom base color.
    ///
    /// Keeps the preset's metallic/roughness values, overrides the color.
    /// Use this for e.g. a polished-steel block in red.
    #[must_use]
    pub fn with_color(&self, color: Color) -> StandardMaterial {
        let (_, metallic, roughness) = self.properties();
        StandardMaterial {
            base_color: color,
            metallic,
            perceptual_roughness: roughness,
            ..default()
        }
    }

    /// Returns `(default_color, metallic, roughness)` for this preset.
    fn properties(&self) -> (Color, f32, f32) {
        match self {
            Self::PolishedSteel => (Color::srgb(0.52, 0.53, 0.56), 0.9, 0.25),
            Self::BrushedMetal => (Color::srgb(0.48, 0.48, 0.50), 0.85, 0.35),
            Self::CastIron => (Color::srgb(0.35, 0.33, 0.32), 0.6, 0.55),
            Self::SpringWire => (Color::srgb(0.55, 0.58, 0.60), 0.85, 0.35),
            Self::Anodized(color) => (*color, 0.7, 0.3),
        }
    }
}

/// Override the material of a geom identified by name.
///
/// Finds the entity with [`ModelGeomIndex`] matching the named geom and
/// replaces its [`MeshMaterial3d<StandardMaterial>`].
///
/// Returns `true` if the geom was found and updated.
pub fn override_geom_material_by_name(
    model: &Model,
    geom_name: &str,
    material: Handle<StandardMaterial>,
    query: &mut Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) -> bool {
    let Some(&index) = model.geom_name_to_id.get(geom_name) else {
        return false;
    };
    override_geom_material_by_index(index, material, query)
}

/// Override the material of a geom identified by index.
///
/// Directly matches [`ModelGeomIndex`]`(index)`.
///
/// Returns `true` if the geom was found and updated.
pub fn override_geom_material_by_index(
    index: usize,
    material: Handle<StandardMaterial>,
    query: &mut Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) -> bool {
    for (geom_idx, mut mat_handle) in query.iter_mut() {
        if geom_idx.0 == index {
            mat_handle.0 = material;
            return true;
        }
    }
    false
}

/// Override materials for multiple geoms at once (by name).
///
/// Takes a slice of `(geom_name, material_handle)` pairs.
/// Returns the number of geoms successfully updated.
pub fn override_geom_materials_by_name(
    model: &Model,
    overrides: &[(&str, Handle<StandardMaterial>)],
    query: &mut Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) -> usize {
    // Build index→material map from name→material pairs
    let index_map: Vec<(usize, Handle<StandardMaterial>)> = overrides
        .iter()
        .filter_map(|(name, mat)| {
            model
                .geom_name_to_id
                .get(*name)
                .map(|&idx| (idx, mat.clone()))
        })
        .collect();

    let mut count = 0;
    for (geom_idx, mut mat_handle) in query.iter_mut() {
        for (target_idx, target_mat) in &index_map {
            if geom_idx.0 == *target_idx {
                mat_handle.0 = target_mat.clone();
                count += 1;
                break;
            }
        }
    }
    count
}

/// Override materials for multiple geoms at once (by name), with a per-entity callback.
///
/// Works like [`override_geom_materials_by_name`] but accepts an extended query
/// with [`Entity`] access and calls `callback(entity, geom_name, commands)` for
/// each matched geom after applying its material.
///
/// Use this when you need to attach additional components (e.g.
/// [`TrailGizmo`](crate::gizmos::TrailGizmo)) to specific geoms by name.
///
/// Returns the number of geoms successfully updated.
pub fn override_geom_materials_by_name_with(
    model: &Model,
    overrides: &[(&str, Handle<StandardMaterial>)],
    query: &mut Query<(
        Entity,
        &ModelGeomIndex,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
    commands: &mut Commands,
    callback: impl Fn(Entity, &str, &mut Commands),
) -> usize {
    // Build index→(material, name) map from name→material pairs
    let index_map: Vec<(usize, Handle<StandardMaterial>, &str)> = overrides
        .iter()
        .filter_map(|(name, mat)| {
            model
                .geom_name_to_id
                .get(*name)
                .map(|&idx| (idx, mat.clone(), *name))
        })
        .collect();

    let mut count = 0;
    for (entity, geom_idx, mut mat_handle) in query.iter_mut() {
        for (target_idx, target_mat, name) in &index_map {
            if geom_idx.0 == *target_idx {
                mat_handle.0 = target_mat.clone();
                callback(entity, name, commands);
                count += 1;
                break;
            }
        }
    }
    count
}
