//! Auto-derive a v2.1 [`cf_cast::CastSpec`] + [`cf_cast::Ribbon`]
//! from a parsed [`crate::config::CastConfig`] + a scan-SDF +
//! centerline polyline.
//!
//! Pure (no I/O); the heavy lifting (STL load, marching cubes, file
//! writes) happens upstream / downstream of this module.

use std::sync::Arc;

use anyhow::{Context, Result, bail};
use cf_cap_planes::{CapPlane, DomeWallSignedSdf, dome_wall_only_mesh};
use cf_cast::{
    CastLayer, CastSpec, MoldingMaterial, PinSpec, PlugPinKind, PlugPinSpec, PourGateKind,
    PourGateSpec, RegistrationKind, Ribbon, SplitNormal,
};
use cf_design::{Sdf, Solid, pinned_floor_shell};
use cf_geometry::{Aabb, IndexedMesh};
use mesh_printability::PrinterConfig;
use mesh_sdf::SignedDistanceField;
use nalgebra::{Point3, Vector3};

use crate::config::{CastConfig, LayerConfig};
use crate::scan::SharedScanSdf;

/// Derived spec + ribbon — the bundle the orchestrator needs to call
/// [`cf_cast::CastSpec::export_molds_v2`] +
/// [`cf_cast::CastSpec::write_procedure_v2`].
#[derive(Debug)]
pub struct DerivedSpec {
    /// Fully-built v2.1 [`CastSpec`].
    pub spec: CastSpec,
    /// Curve-following [`Ribbon`] with registration / pour-gate /
    /// plug-pin overrides applied per the config.
    pub ribbon: Ribbon,
}

/// The cf-cast cure anchors this bridge recognizes, as
/// `(anchor_key, display_name, density_kg_m3)` triples.
///
/// Densities mirror sim-soft's `silicone_table.rs` (Ecoflex 00-10 =
/// 1040, the rest of the Ecoflex line + Dragon Skin 10A/15 = 1070,
/// Dragon Skin 20A/30A = 1080 kg/m³). Reproduced here rather than
/// imported because cf-cast (and cf-cast-cli) are deliberately
/// decoupled from the FEM crate's dep tree. Structurally identical
/// to cf-device-design's `LAYER_MATERIALS` — same by-name-mirror
/// posture. Single source for all three anchor lookups below
/// ([`density_for_anchor`], [`display_name_for_anchor`], and
/// `build_material`'s `anchor_key` resolution).
const ANCHORS: &[(&str, &str, f64)] = &[
    ("ECOFLEX_00_10", "Ecoflex 00-10", 1040.0),
    ("ECOFLEX_00_20", "Ecoflex 00-20", 1070.0),
    ("ECOFLEX_00_30", "Ecoflex 00-30", 1070.0),
    ("ECOFLEX_00_50", "Ecoflex 00-50", 1070.0),
    ("DRAGON_SKIN_10A", "Dragon Skin 10A", 1070.0),
    ("DRAGON_SKIN_15", "Dragon Skin 15", 1070.0),
    ("DRAGON_SKIN_20A", "Dragon Skin 20A", 1080.0),
    ("DRAGON_SKIN_30A", "Dragon Skin 30A", 1080.0),
];

/// Resolve a cf-cast cure anchor key to a Smooth-On TDS density
/// (kg/m³). Returns `None` for unrecognized anchors; the caller can
/// layer a per-layer `density_kg_m3` override on top. See
/// `ANCHORS` for the table + the FEM-decoupling rationale.
#[must_use]
pub fn density_for_anchor(anchor: &str) -> Option<f64> {
    ANCHORS
        .iter()
        .find(|(key, _, _)| *key == anchor)
        .map(|(_, _, density)| *density)
}

/// Resolve a cf-cast cure anchor key to its human-readable display
/// name. Returns `None` for unrecognized anchors. See `ANCHORS`.
#[must_use]
pub fn display_name_for_anchor(anchor: &str) -> Option<&'static str> {
    ANCHORS
        .iter()
        .find(|(key, _, _)| *key == anchor)
        .map(|(_, name, _)| *name)
}

/// Build a [`MoldingMaterial`] from one [`LayerConfig`].
///
/// - Density: `layer.density_kg_m3` if `Some`; otherwise
///   [`density_for_anchor(&layer.material)`].
/// - Display name: `layer.display_name` if `Some`; otherwise
///   [`display_name_for_anchor`]; otherwise the anchor key itself.
/// - `anchor_key`: `Some(&'static str)` if the anchor is recognized
///   (so cf-cast's `cure::lookup` can resolve it in procedure
///   markdown); `None` for custom-density anchors.
fn build_material(layer: &LayerConfig) -> Result<MoldingMaterial> {
    let density_kg_m3 = layer
        .density_kg_m3
        .or_else(|| density_for_anchor(&layer.material))
        .with_context(|| {
            format!(
                "material '{}' has no known density and no density_kg_m3 override",
                layer.material
            )
        })?;

    let display_name = layer
        .display_name
        .clone()
        .or_else(|| display_name_for_anchor(&layer.material).map(str::to_string))
        .unwrap_or_else(|| layer.material.clone());

    // Resolve to a recognized cf-cast cure anchor (`&'static str`)
    // only if the anchor matches one of `ANCHORS`' known keys.
    // Custom keys surface in the procedure markdown as the "consult
    // Smooth-On TDS" placeholder branch.
    let anchor_key: Option<&'static str> = ANCHORS
        .iter()
        .find(|(key, _, _)| *key == layer.material.as_str())
        .map(|(key, _, _)| *key);

    Ok(MoldingMaterial {
        display_name,
        density_kg_m3,
        anchor_key,
    })
}

/// Auto-derive [`CastSpec`] + [`Ribbon`] from the parsed config + scan
/// SDF + centerline polyline.
///
/// Geometry plan (Option A.1 — plug + layers inset inward by
/// `cavity_inset_m`):
///
/// - `plug` = `Solid::from_sdf(scan_sdf, scan_aabb_padded).offset(-cavity_inset_m)`
///   — the plug is the scan surface shrunk inward by the press-fit
///   reservation. Layer 0's silicone shell sits immediately outside
///   this surface, so the cured part has a cavity that's `cavity_inset_m`
///   smaller than the scan, leaving a press-fit interference for the
///   real device to snap into. When `cavity_inset_m = 0.0` (the inline-
///   layers path with no design.toml) the plug degenerates to the scan
///   literal — matching pre-slice-9.6 v2 behavior bit-exactly.
/// - `layers[N].body` = `Solid::from_sdf(scan_sdf, scan_aabb_padded)
///   .offset(sum(thickness[0..=N]) - cavity_inset_m)` — each layer's
///   outer surface is shifted inward by the same `cavity_inset_m`, so
///   the whole stack moves together and inter-layer thickness is
///   preserved.
/// - `bounding_region` = `Solid::cuboid(half_extents).translate(scan_center)`
///   where `half_extents = scan.half_extents + cumulative_thickness +
///   bounding_margin` (per-axis). The cuboid envelope is conservative —
///   it ignores the inward inset shift, which can only shrink the
///   bounded region — so the SDF eval domain stays valid.
///
/// All five solids share the same underlying `Arc<SignedDistanceField>`
/// via [`SharedScanSdf::clone`], so the heavy mesh + face-normal arrays
/// live in one allocation regardless of layer count.
///
/// The AABB passed to [`Solid::from_sdf`] is the scan AABB padded by
/// the cumulative outermost thickness — that ensures the outermost
/// offset surface still falls inside the SDF's evaluation domain (the
/// mesher uses bounds for octree pruning).
///
/// `cavity_inset_m` mirrors `design.cavity.inset_m` from
/// `cf-device-design`'s `.design.toml`. The design-sourced path
/// ([`crate::run`]) lifts it from [`crate::design_ref::DesignRef`]; the
/// inline-layers path passes `0.0` so old cast.toml configs without a
/// `[design]` block produce the same molds they always did.
///
/// # Errors
///
/// - empty layers (caught earlier by [`CastConfig::validate`], but
///   defensive here too)
/// - centerline polyline non-finite or too short for [`Ribbon::new`]
/// - non-normalizable split-normal (caught by [`SplitNormal::new`])
pub fn derive_spec_and_ribbon(
    config: &CastConfig,
    scan_sdf: &SharedScanSdf,
    scan_mesh: &IndexedMesh,
    scan_aabb: Aabb,
    centerline: &[Point3<f64>],
    cap_planes: &[CapPlane],
    cavity_inset_m: f64,
) -> Result<DerivedSpec> {
    if config.layers.is_empty() {
        bail!("derive_spec_and_ribbon called with empty layers");
    }

    let cumulative_thickness: f64 = config.layers.iter().map(|l| l.thickness_m).sum();

    // Pad the SDF evaluation bounds by the cumulative outward offset
    // PLUS the bounding margin — the outer-most layer's surface lives
    // at `scan_surface + cumulative_thickness`, and the bounding region
    // adds another `bounding_margin_m` of cup-wall material. The
    // mesher walks the SDF over `bounding_region.bounds()`, so the
    // SDF must produce finite distances over that whole domain.
    // The inward `cavity_inset_m` shift can only shrink the outermost
    // surface, so the existing padding stays an upper bound.
    let sdf_bounds_pad = cumulative_thickness + config.cast.bounding_margin_m;
    let sdf_bounds = scan_aabb.expanded(sdf_bounds_pad);

    // Pinned-floor scope-C sub-leaf 6: build a DomeWallSignedSdf
    // adapter combining the closed-body scan SDF (sign source) with a
    // second SDF over the cap-polygon-stripped open mesh (magnitude
    // source via `.abs()`). pinned_floor_shell then offsets this
    // composed field and intersects with each cap's body-interior
    // half-space, producing CLOSED shells with flat floors pinned at
    // every cap plane.
    //
    // No-caps fast path: cap_planes empty → closed === open (same
    // Arc), pinned_floor_shell with empty caps collapses to
    // `Solid::from_sdf(scan_sdf).offset(offset_m)` — byte-identical
    // to the pre-pinned-floor uniform-offset construction. The raw-
    // scan no-`.prep.toml`-caps path is the regression sentinel for
    // the entire workshop iter-1 pipeline.
    //
    // Open-mesh SDF construction uses `mesh_sdf::SignedDistanceField`
    // (same as the closed scan, NOT insertion_sim's flood-fill
    // `build_grid_sdf`): the §7 risk #2 sign-heuristic concern bites
    // only when the SIGN of the open SDF is consumed, but
    // DomeWallSignedSdf only takes the OPEN SDF's `.abs()` — so the
    // unsigned-magnitude path is reliable regardless of mesh-sdf's
    // sign heuristic on the non-manifold open mesh.
    let dome_wall_signed = if cap_planes.is_empty() {
        let arc: Arc<dyn Sdf> = scan_sdf.inner_arc();
        DomeWallSignedSdf {
            closed: arc.clone(),
            open: arc,
        }
    } else {
        let open_mesh = dome_wall_only_mesh(scan_mesh, cap_planes);
        let open_sdf = SignedDistanceField::new(open_mesh)
            .context("build SignedDistanceField from cap-polygon-stripped scan mesh")?;
        let closed_arc: Arc<dyn Sdf> = scan_sdf.inner_arc();
        let open_arc: Arc<dyn Sdf> = Arc::new(open_sdf);
        DomeWallSignedSdf {
            closed: closed_arc,
            open: open_arc,
        }
    };
    let cap_tuples: Vec<_> = cap_planes.iter().map(CapPlane::as_tuple).collect();

    // Plug = scan shrunk inward by the press-fit reservation
    // (`design.cavity.inset_m` — 0 for inline-layers configs), composed
    // as a pinned-floor shell so the plug's floor sits at the cap
    // plane (NOT extended past it via uniform offset). `offset_m =
    // -cavity_inset_m`: the inward-shrink path.
    let plug = pinned_floor_shell(
        dome_wall_signed.clone(),
        sdf_bounds,
        &cap_tuples,
        -cavity_inset_m,
    );

    // Layer bodies — cumulative outward offset, shifted inward by
    // `cavity_inset_m` so the whole stack sits on top of the inset
    // plug surface (preserves inter-layer thickness, matches
    // cf-device-design's `outer.subtract(cavity)` body-construction
    // logic). Each body is a pinned-floor shell so every layer's outer
    // surface terminates with a flat floor at the cap plane (matches
    // cf-device-design's preview + sim, ensuring the cast mold geometry
    // produces a part whose cap-plane face is flat).
    let mut layers = Vec::with_capacity(config.layers.len());
    let mut cumulative_so_far = 0.0;
    for layer_cfg in &config.layers {
        cumulative_so_far += layer_cfg.thickness_m;
        let body = pinned_floor_shell(
            dome_wall_signed.clone(),
            sdf_bounds,
            &cap_tuples,
            cumulative_so_far - cavity_inset_m,
        );
        let material = build_material(layer_cfg).with_context(|| {
            format!(
                "build material for layer with thickness {} m",
                layer_cfg.thickness_m
            )
        })?;
        layers.push(CastLayer { body, material });
    }

    // Bounding cuboid: enveloping every layer body + margin.
    let bounding_half_extents = scan_aabb.half_extents()
        + Vector3::repeat(cumulative_thickness + config.cast.bounding_margin_m);
    let bounding_center = scan_aabb.center();
    let bounding_region = Solid::cuboid(bounding_half_extents).translate(Vector3::new(
        bounding_center.x,
        bounding_center.y,
        bounding_center.z,
    ));

    let printer_config =
        PrinterConfig::fdm_default().with_min_wall_thickness(config.cast.piece_min_wall_mm);

    let spec = CastSpec {
        layers,
        plug,
        bounding_region,
        mesh_cell_size_m: config.cast.mesh_cell_size_m,
        printer_config,
        mass_budget_kg: config.cast.mass_budget_kg,
    };

    let split_normal_vec = Vector3::new(
        config.cast.split_normal[0],
        config.cast.split_normal[1],
        config.cast.split_normal[2],
    );
    let split = SplitNormal::new(split_normal_vec).with_context(|| {
        format!(
            "split_normal {:?} could not be normalized — must be non-zero magnitude",
            config.cast.split_normal
        )
    })?;
    let mut ribbon = Ribbon::new(centerline.to_vec(), split).with_context(|| {
        format!(
            "build Ribbon from centerline with {} points",
            centerline.len()
        )
    })?;

    if config.registration_pins.enabled {
        ribbon = ribbon.with_registration(RegistrationKind::Pins(PinSpec::iter1()));
    }
    if config.pour_gate.enabled {
        ribbon = ribbon.with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
    }
    if config.plug_pins.enabled {
        let mut plug_pin_spec = PlugPinSpec::iter1();
        if let Some(len) = config.plug_pins.pin_length_m {
            plug_pin_spec.pin_length_m = len;
        }
        if let Some(b) = config.plug_pins.include_dome_pin {
            plug_pin_spec.include_dome_pin = b;
        }
        ribbon = ribbon.with_plug_pins(PlugPinKind::Axial(plug_pin_spec));
    }

    Ok(DerivedSpec { spec, ribbon })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::config::{CastConfig, CastDefaults, LayerConfig, PlugPinConfig, PourGateConfig};
    use cf_geometry::IndexedMesh;
    use mesh_sdf::SignedDistanceField;

    fn unit_cube_aabb() -> Aabb {
        Aabb::new(
            Point3::new(-0.05, -0.05, -0.05),
            Point3::new(0.05, 0.05, 0.05),
        )
    }

    fn unit_cube_mesh() -> IndexedMesh {
        let mut m = IndexedMesh::new();
        let coords = [
            (-0.05, -0.05, -0.05),
            (0.05, -0.05, -0.05),
            (0.05, 0.05, -0.05),
            (-0.05, 0.05, -0.05),
            (-0.05, -0.05, 0.05),
            (0.05, -0.05, 0.05),
            (0.05, 0.05, 0.05),
            (-0.05, 0.05, 0.05),
        ];
        for (x, y, z) in coords {
            m.vertices.push(Point3::new(x, y, z));
        }
        for f in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ] {
            m.faces.push(f);
        }
        m
    }

    fn unit_cube_sdf() -> SharedScanSdf {
        SharedScanSdf::new(SignedDistanceField::new(unit_cube_mesh()).unwrap())
    }

    fn straight_x_centerline() -> Vec<Point3<f64>> {
        vec![
            Point3::new(-0.04, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.04, 0.0, 0.0),
        ]
    }

    fn three_layer_config() -> CastConfig {
        CastConfig {
            scan: crate::config::ScanConfig {
                cleaned_stl: std::path::PathBuf::from("scan.stl"),
                prep_toml: std::path::PathBuf::from("scan.prep.toml"),
            },
            cast: CastDefaults::default(),
            design: None,
            layers: vec![
                LayerConfig {
                    thickness_m: 0.006,
                    material: "ECOFLEX_00_30".to_string(),
                    density_kg_m3: None,
                    display_name: None,
                    slacker_fraction: None,
                },
                LayerConfig {
                    thickness_m: 0.004,
                    material: "DRAGON_SKIN_10A".to_string(),
                    density_kg_m3: None,
                    display_name: None,
                    slacker_fraction: None,
                },
                LayerConfig {
                    thickness_m: 0.004,
                    material: "ECOFLEX_00_30".to_string(),
                    density_kg_m3: None,
                    display_name: None,
                    slacker_fraction: None,
                },
            ],
            plug_pins: PlugPinConfig::default(),
            pour_gate: PourGateConfig::default(),
            registration_pins: crate::config::RegistrationConfig::default(),
        }
    }

    #[test]
    fn density_for_known_anchors() {
        assert_eq!(density_for_anchor("ECOFLEX_00_10"), Some(1040.0));
        assert_eq!(density_for_anchor("ECOFLEX_00_30"), Some(1070.0));
        assert_eq!(density_for_anchor("DRAGON_SKIN_30A"), Some(1080.0));
        assert_eq!(density_for_anchor("NOT_A_THING"), None);
    }

    #[test]
    fn display_name_for_known_anchors() {
        assert_eq!(
            display_name_for_anchor("ECOFLEX_00_30"),
            Some("Ecoflex 00-30")
        );
        assert_eq!(
            display_name_for_anchor("DRAGON_SKIN_10A"),
            Some("Dragon Skin 10A")
        );
        assert_eq!(display_name_for_anchor("CUSTOM"), None);
    }

    #[test]
    fn anchors_table_covers_eight_keys_with_in_band_densities() {
        // The single ANCHORS table now backs all three anchor
        // lookups (density / display name / anchor_key); pin its
        // shape so an accidental edit — dropped row, typo'd density —
        // trips here.
        assert_eq!(ANCHORS.len(), 8);
        for (key, name, density) in ANCHORS {
            assert!(
                (1040.0..=1080.0).contains(density),
                "{key}: density {density} kg/m³ outside the silicone band",
            );
            assert!(!name.is_empty(), "{key}: display name must be non-empty");
        }
    }

    #[test]
    fn build_material_uses_anchor_defaults() {
        let cfg = LayerConfig {
            thickness_m: 0.006,
            material: "ECOFLEX_00_30".to_string(),
            density_kg_m3: None,
            display_name: None,
            slacker_fraction: None,
        };
        let m = build_material(&cfg).unwrap();
        assert_eq!(m.display_name, "Ecoflex 00-30");
        assert!((m.density_kg_m3 - 1070.0).abs() < 1e-12);
        assert_eq!(m.anchor_key, Some("ECOFLEX_00_30"));
    }

    #[test]
    fn build_material_overrides_take_precedence() {
        let cfg = LayerConfig {
            thickness_m: 0.006,
            material: "ECOFLEX_00_30".to_string(),
            density_kg_m3: Some(1234.0),
            display_name: Some("Custom Ecoflex".to_string()),
            slacker_fraction: None,
        };
        let m = build_material(&cfg).unwrap();
        assert_eq!(m.display_name, "Custom Ecoflex");
        assert!((m.density_kg_m3 - 1234.0).abs() < 1e-12);
        assert_eq!(m.anchor_key, Some("ECOFLEX_00_30"));
    }

    #[test]
    fn build_material_custom_anchor_with_density() {
        let cfg = LayerConfig {
            thickness_m: 0.006,
            material: "CUSTOM_GRADE".to_string(),
            density_kg_m3: Some(1080.0),
            display_name: Some("Custom".to_string()),
            slacker_fraction: None,
        };
        let m = build_material(&cfg).unwrap();
        assert_eq!(m.display_name, "Custom");
        assert!((m.density_kg_m3 - 1080.0).abs() < 1e-12);
        assert_eq!(m.anchor_key, None);
    }

    #[test]
    fn derive_three_layer_spec_carries_through_layers_and_materials() {
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            0.0,
        )
        .unwrap();
        assert_eq!(derived.spec.layers.len(), 3);
        assert_eq!(
            derived.spec.layers[0].material.display_name,
            "Ecoflex 00-30"
        );
        assert_eq!(
            derived.spec.layers[1].material.display_name,
            "Dragon Skin 10A"
        );
        assert_eq!(
            derived.spec.layers[2].material.display_name,
            "Ecoflex 00-30"
        );
        assert!((derived.spec.mesh_cell_size_m - 0.003).abs() < 1e-12);
    }

    #[test]
    fn derive_ribbon_carries_registration_when_enabled() {
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            0.0,
        )
        .unwrap();
        assert!(
            matches!(derived.ribbon.registration, RegistrationKind::Pins(_)),
            "registration_pins.enabled=true must produce Pins(_)"
        );
        assert!(
            matches!(derived.ribbon.pour_gate, PourGateKind::Default(_)),
            "pour_gate.enabled=true must produce Default(_)"
        );
        assert!(
            matches!(derived.ribbon.plug_pins, PlugPinKind::Axial(_)),
            "plug_pins.enabled=true must produce Axial(_)"
        );
    }

    #[test]
    fn derive_ribbon_disables_features_when_config_disables_them() {
        let mut cfg = three_layer_config();
        cfg.registration_pins.enabled = false;
        cfg.pour_gate.enabled = false;
        cfg.plug_pins.enabled = false;
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            0.0,
        )
        .unwrap();
        assert!(matches!(
            derived.ribbon.registration,
            RegistrationKind::None
        ));
        assert!(matches!(derived.ribbon.pour_gate, PourGateKind::None));
        assert!(matches!(derived.ribbon.plug_pins, PlugPinKind::None));
    }

    #[test]
    fn derive_plug_pin_length_override_passes_through() {
        let mut cfg = three_layer_config();
        cfg.plug_pins.pin_length_m = Some(0.028);
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            0.0,
        )
        .unwrap();
        let PlugPinKind::Axial(spec) = &derived.ribbon.plug_pins else {
            unreachable!("plug_pins.enabled=true should yield Axial(_)")
        };
        assert!((spec.pin_length_m - 0.028).abs() < 1e-12);
    }

    #[test]
    fn derive_empty_layers_errors() {
        let mut cfg = three_layer_config();
        cfg.layers.clear();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let err = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            0.0,
        )
        .expect_err("empty layers must fail");
        assert!(err.to_string().contains("empty"), "unexpected error: {err}");
    }

    /// Slice 9.6 — when `cavity_inset_m > 0`, the plug iso-surface
    /// must sit INSIDE the scan surface (the press-fit reservation),
    /// and every layer's outer iso-surface must be shifted inward by
    /// the same amount so inter-layer thickness is preserved.
    ///
    /// The unit cube has half-extent 0.05 m. With `inset = 0.005` and
    /// layer-0 thickness 0.006:
    ///
    /// - plug zero-iso at `+x = 0.045` (scan minus inset).
    /// - layer-0 outer zero-iso at `+x = 0.051` (scan + thickness_0 - inset).
    ///
    /// Equivalently: layer-0 shell thickness = 0.051 − 0.045 = 0.006,
    /// matching `thickness_m` exactly (the inset cancels in the
    /// difference).
    #[test]
    fn derive_inset_shrinks_plug_and_shifts_layers_inward() {
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let inset = 0.005;
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            inset,
        )
        .unwrap();

        // Plug zero-iso is at ±0.045 (scan minus inset).
        let plug = &derived.spec.plug;
        assert!(
            plug.evaluate(&Point3::new(0.044, 0.0, 0.0)) < 0.0,
            "plug interior must be inside the inset surface",
        );
        assert!(
            plug.evaluate(&Point3::new(0.046, 0.0, 0.0)) > 0.0,
            "plug exterior must be outside the inset surface — scan surface (0.05) is OUTSIDE the shrunk plug",
        );
        assert!(
            (plug.evaluate(&Point3::new(0.045, 0.0, 0.0))).abs() < 1e-3,
            "plug zero-iso at 0.045 expected (scan 0.05 minus inset 0.005)",
        );

        // Layer-0 outer zero-iso is at ±0.051 (scan + thickness_0 - inset).
        let layer0 = &derived.spec.layers[0].body;
        assert!(
            layer0.evaluate(&Point3::new(0.050, 0.0, 0.0)) < 0.0,
            "layer-0 interior must include the scan surface",
        );
        assert!(
            layer0.evaluate(&Point3::new(0.052, 0.0, 0.0)) > 0.0,
            "layer-0 exterior at +0.052 must be past the outer iso",
        );

        // Sanity check: inset 0.0 reproduces pre-slice-9.6 behavior
        // (plug zero-iso back at the scan surface).
        let derived_zero = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            0.0,
        )
        .unwrap();
        assert!(
            (derived_zero
                .spec
                .plug
                .evaluate(&Point3::new(0.050, 0.0, 0.0)))
            .abs()
                < 1e-3,
            "inset = 0.0 must put plug zero-iso at the scan surface (0.050)",
        );
    }

    // ----- Pinned-floor scope-C sub-leaf 6 ----------------------------

    #[test]
    fn derive_no_caps_byte_identical_to_pre_pinned_floor_path() {
        // Regression sentinel for the raw-scan path (no `.prep.toml`
        // caps). With empty cap_planes, pinned_floor_shell collapses
        // to `Solid::from_sdf(scan_sdf).offset(offset_m)`. Validates
        // by sampling the plug + each layer body at probe points
        // matching the pre-pinned-floor `derive_inset_shrinks_plug_*`
        // test expectations (the inset = 0.0 path is the canonical
        // "no behavior change" check).
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[],
            0.0,
        )
        .unwrap();

        // Plug zero-iso at scan surface (x = 0.05) with inset = 0 +
        // no caps — same as legacy.
        let plug = &derived.spec.plug;
        assert!(plug.evaluate(&Point3::new(0.050, 0.0, 0.0)).abs() < 1e-3);
        assert!(plug.evaluate(&Point3::new(0.045, 0.0, 0.0)) < 0.0);
        assert!(plug.evaluate(&Point3::new(0.055, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn derive_with_caps_pins_plug_floor_at_cap_plane() {
        // Cap at +z = 0.05 (top face of unit cube), normal +Z, no
        // inset. Plug = pinned_floor_shell(scan, &[cap], 0.0) — plug
        // should have a flat floor at the cap plane: above the cap
        // plane the plug is clipped (outside); below the cap plane
        // the plug = the scan body.
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let cap = CapPlane {
            centroid: Point3::new(0.0, 0.0, 0.05),
            normal: Vector3::new(0.0, 0.0, 1.0),
            vertex_count: 4,
            loop_index: 0,
        };
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            &unit_cube_mesh(),
            unit_cube_aabb(),
            &centerline,
            &[cap],
            0.0,
        )
        .unwrap();

        let plug = &derived.spec.plug;
        // Below the cap plane at the scan center: deep inside body →
        // negative.
        assert!(
            plug.evaluate(&Point3::new(0.0, 0.0, 0.0)) < 0.0,
            "plug interior below cap should be inside",
        );
        // Just above the cap plane: clipped → outside.
        assert!(
            plug.evaluate(&Point3::new(0.0, 0.0, 0.06)) > 0.0,
            "above cap plane (z=0.06) should be clipped (outside)",
        );
    }
}
