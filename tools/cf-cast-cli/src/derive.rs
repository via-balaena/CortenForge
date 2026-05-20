//! Auto-derive a v2.1 [`cf_cast::CastSpec`] + [`cf_cast::Ribbon`]
//! from a parsed [`crate::config::CastConfig`] + a scan-SDF +
//! centerline polyline.
//!
//! Pure (no I/O); the heavy lifting (STL load, marching cubes, file
//! writes) happens upstream / downstream of this module.

use std::sync::Arc;

use anyhow::{Context, Result, bail};
use cf_cap_planes::{CapPlane, dome_wall_only_mesh};
use cf_cast::{
    CastLayer, CastSpec, MoldingMaterial, PinSpec, PlugPinKind, PlugPinSpec, PourGateKind,
    PourGateSpec, RegistrationKind, Ribbon, SplitNormal,
};
use cf_design::pinned_floor_shell;
use cf_geometry::Aabb;
use mesh_printability::PrinterConfig;
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
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
/// `cavity_inset_m`, threaded through `cf_design::pinned_floor_shell`
/// for the candidate-A pinned-floor construction):
///
/// - `plug` = `pinned_floor_shell(scan_sdf, open_scan_sdf, scan_aabb_padded,
///   cap_planes, -cavity_inset_m)` — the plug is the scan surface
///   shrunk inward by the press-fit reservation, with its floor
///   pinned at every cap plane when caps are present (and a plain
///   isotropic offset when not). When `cavity_inset_m = 0.0` (the
///   inline-layers path with no design.toml) and `cap_planes` is empty,
///   the plug degenerates to the scan literal — matching pre-slice-9.6
///   v2 behavior bit-exactly.
/// - `layers[N].body` = `pinned_floor_shell(scan_sdf, open_scan_sdf,
///   scan_aabb_padded, cap_planes, sum(thickness[0..=N]) - cavity_inset_m)`
///   — each layer's outer surface is shifted inward by the same
///   `cavity_inset_m`, so the whole stack moves together and inter-layer
///   thickness is preserved; the cap-plane fold pins every layer's
///   floor at the cap plane.
/// - `bounding_region` = `outermost_layer_body.offset(wall_thickness_m)`
///   — the outermost cumulative layer body grown outward by the
///   cup-wall thickness. The offset shell follows the device contour
///   (vs the v1 cuboid envelope), so the mold piece is a contoured
///   rind around the device rather than a brick with a contour-shaped
///   hole; on iter-1 sock_over_capsule this drops mold-piece plastic
///   ~10×. Containment is upheld because cf-cast layer ordering is
///   innermost-first (so `layers.last().body` ⊇ every inner layer
///   body) and `.offset(d)` for `d > 0` produces a superset region
///   — `bounding ⊇ every layer_body` holds by construction. See
///   `docs/CF_CAST_MOLD_WALL_RECON.md` §2.1.
///
/// All five solids share the same underlying flood-fill-signed scan
/// SDF via [`SharedScanSdf::clone`], so the heavy parry BVH + flood-fill
/// grid live in one allocation regardless of layer count. The
/// cap-stripped open SDF (built once when `cap_planes` is non-empty) is
/// similarly `Arc`-shared so the candidate-A unsigned-rind adapter
/// inside `pinned_floor_shell` queries the same allocation across every
/// call.
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
/// `cap_planes` is parsed out of the scan's `.prep.toml` via
/// `cf_cap_planes::parse_cap_planes`; empty for scans without a
/// `[caps]` block or where every loop is excluded (the no-caps fast
/// path inside `pinned_floor_shell` then short-circuits to a plain
/// isotropic offset — byte-identical to the pre-pinned-floor mold
/// geometry the v2 example crate ships).
///
/// # Errors
///
/// - empty layers (caught earlier by [`CastConfig::validate`], but
///   defensive here too)
/// - centerline polyline non-finite or too short for [`Ribbon::new`]
/// - non-normalizable split-normal (caught by [`SplitNormal::new`])
/// - cap-stripped open mesh empty or unsuitable for
///   [`TriMeshDistance::new`] (only on the with-caps path)
pub fn derive_spec_and_ribbon(
    config: &CastConfig,
    scan_sdf: &SharedScanSdf,
    scan_aabb: Aabb,
    centerline: &[Point3<f64>],
    cavity_inset_m: f64,
    cap_planes: &[CapPlane],
) -> Result<DerivedSpec> {
    if config.layers.is_empty() {
        bail!("derive_spec_and_ribbon called with empty layers");
    }

    let cumulative_thickness: f64 = config.layers.iter().map(|l| l.thickness_m).sum();

    // Pad the SDF evaluation bounds by the cumulative outward offset
    // PLUS the cup-wall thickness — the outer-most layer's surface
    // lives at `scan_surface + cumulative_thickness`, and the bounding
    // region grows that outward by another `wall_thickness_m`. The
    // mesher walks the SDF over `bounding_region.bounds()`, so the
    // SDF must produce finite distances over that whole domain.
    // The inward `cavity_inset_m` shift can only shrink the outermost
    // surface, so the existing padding stays an upper bound.
    let sdf_bounds_pad = cumulative_thickness + config.cast.wall_thickness_m;
    let sdf_bounds = scan_aabb.expanded(sdf_bounds_pad);

    // Candidate-A two-SDF construction (per the redesign spec §2 A5
    // at `docs/CF_DEVICE_DESIGN_CAVITY_PINNED_FLOOR_REDESIGN_SPEC.md`).
    // The closed-body SDF is the existing scan SDF (flood-fill-signed
    // via [`crate::scan::SharedScanSdf`] — the load-bearing sign-defense
    // that pre-D.5 cf-cast-cli was missing, root-caused at
    // [[project-cf-cast-plug-layer-0-watertight-discovery]]). The
    // open-body SDF is built once over the cap-stripped scan mesh and
    // shared by Arc across the plug + every layer body call. With no
    // cap planes the primitive short-circuits to the previous
    // `Solid::from_sdf(scan).offset(...)` formulation, so the
    // inline-layers path stays byte-identical to pre-pinned-floor.
    //
    // open_sdf stays on pseudo-normal sign (the cheap parry path) —
    // `pinned_floor_shell`'s private `UnsignedRindSdf` adapter consumes
    // only `.eval(p).abs()`, so the sign oracle's output is mathematically
    // irrelevant on the open side. Switching it to flood-fill would add a
    // grid build with no behavior change. See spec §D.5 #4
    // (`docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`).
    let closed_sdf_arc: Arc<dyn cf_design::Sdf> = Arc::new(scan_sdf.clone());
    let open_sdf_arc: Arc<dyn cf_design::Sdf> = if cap_planes.is_empty() {
        Arc::clone(&closed_sdf_arc)
    } else {
        let open_mesh = dome_wall_only_mesh(scan_sdf.mesh(), cap_planes);
        let open_distance = TriMeshDistance::new(open_mesh)
            .context("build TriMeshDistance from the cap-stripped scan mesh")?;
        let open_sign = PseudoNormalSign::from_distance(&open_distance);
        Arc::new(Signed {
            distance: open_distance,
            sign: open_sign,
        })
    };
    let cap_tuples: Vec<(Point3<f64>, Vector3<f64>)> =
        cap_planes.iter().map(CapPlane::as_tuple).collect();

    // Plug = scan shrunk inward by the press-fit reservation
    // (`design.cavity.inset_m` — 0 for inline-layers configs). Under
    // candidate A this is the cavity surface with its floor pinned at
    // every cap plane; under the no-caps fast path it degenerates to
    // `Solid::from_sdf(scan).offset(-cavity_inset_m)` bit-for-bit.
    let plug = pinned_floor_shell(
        closed_sdf_arc.clone(),
        open_sdf_arc.clone(),
        sdf_bounds,
        &cap_tuples,
        -cavity_inset_m,
    );

    // Layer bodies — cumulative outward offset, shifted inward by
    // `cavity_inset_m` so the whole stack sits on top of the inset
    // plug surface (preserves inter-layer thickness, matches
    // cf-device-design's `outer.subtract(cavity)` body-construction
    // logic). Same Arc-clone pattern: every layer threads the same
    // shared closed + open SDFs into `pinned_floor_shell`.
    let mut layers = Vec::with_capacity(config.layers.len());
    let mut cumulative_so_far = 0.0;
    for layer_cfg in &config.layers {
        cumulative_so_far += layer_cfg.thickness_m;
        let body = pinned_floor_shell(
            closed_sdf_arc.clone(),
            open_sdf_arc.clone(),
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

    // Bounding region: outermost layer body grown outward by the
    // cup-wall thickness. Contour-following rind (Option A from
    // `docs/CF_CAST_MOLD_WALL_RECON.md`), replaces the pre-arc cuboid
    // envelope. The outermost body's offset distance is
    // `cumulative_thickness - cavity_inset_m` (same as the last loop
    // iteration's body construction); building the bounding region
    // directly from `pinned_floor_shell` avoids a `.last().unwrap()`
    // on `layers` and keeps the offset relationship explicit.
    let bounding_region = pinned_floor_shell(
        closed_sdf_arc.clone(),
        open_sdf_arc.clone(),
        sdf_bounds,
        &cap_tuples,
        cumulative_thickness - cavity_inset_m,
    )
    .offset(config.cast.wall_thickness_m);

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
    use cf_design::Solid;
    use cf_geometry::IndexedMesh;
    use mesh_sdf::{WALL_THRESHOLD_FACTOR_DEFAULT, flood_filled_sdf};

    fn unit_cube_aabb() -> Aabb {
        Aabb::new(
            Point3::new(-0.05, -0.05, -0.05),
            Point3::new(0.05, 0.05, 0.05),
        )
    }

    fn unit_cube_sdf() -> SharedScanSdf {
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
        // Flood-fill bounds = unit_cube_aabb() expanded by the same
        // padding `derive_spec_and_ribbon` computes from its config
        // (cumulative thickness + bounding margin = 14 mm + 20 mm =
        // 34 mm); round up to 50 mm for headroom and to keep the helper
        // independent of the three-layer fixture's exact thicknesses.
        //
        // `cell_size` is intentionally NOT a divisor of the cube's
        // 0.05 m half-extent: any aligned cell_size (0.01, 0.005,
        // 0.0025, …) puts a lattice node EXACTLY on the cube surface,
        // tags it Wall, and lets the BFS label expansion pick an
        // arbitrary sign for that node — which then propagates to
        // SharedScanSdf probes that round to it. 0.0017 m (prime-ish)
        // keeps every lattice node off the cube surface so the sign at
        // probe-distance > wall_threshold is unambiguously
        // flood-reachable (Outside) or unreachable (Inside).
        let bounds = unit_cube_aabb().expanded(0.05);
        let (signed, _report) =
            flood_filled_sdf(m, bounds, 0.0017, WALL_THRESHOLD_FACTOR_DEFAULT).unwrap();
        SharedScanSdf::new(signed)
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
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
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
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
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
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
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
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
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
        let err = derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[])
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
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, inset, &[]).unwrap();

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
        let derived_zero =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
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

    // ── Sub-leaf A5: candidate-A pinned-floor plug + bodies ─────────

    #[test]
    fn derive_no_prep_toml_byte_identical_to_pre_pinned_floor() {
        // No-caps fast path inside `pinned_floor_shell` returns
        // `Solid::from_sdf(closed).offset(offset)` — byte-identical to
        // the pre-pinned-floor `Solid::from_sdf(scan_sdf).offset(...)`
        // formulation cf-cast-cli used before this sub-leaf. The
        // primitive-level byte equality is pinned by
        // `pinned_floor_shell_empty_caps_byte_identical_to_offset` in
        // cf-design; at the consumer level we just need to prove
        // multiple probes on plug + each layer body match a hand-built
        // reference `Solid::from_sdf(scan_sdf).offset(...)` to f64
        // tolerance.
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let inset = 0.005;
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, inset, &[]).unwrap();

        let sdf_bounds_pad: f64 =
            cfg.layers.iter().map(|l| l.thickness_m).sum::<f64>() + cfg.cast.wall_thickness_m;
        let sdf_bounds = unit_cube_aabb().expanded(sdf_bounds_pad);
        let plug_ref = Solid::from_sdf(sdf.clone(), sdf_bounds).offset(-inset);

        let probes = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.03, 0.02, 0.01),
            Point3::new(-0.04, -0.02, 0.0),
            Point3::new(0.045, 0.0, 0.0),
            Point3::new(0.06, 0.0, 0.0),
        ];
        for p in probes {
            let a = derived.spec.plug.evaluate(&p);
            let b = plug_ref.evaluate(&p);
            assert!(
                (a - b).abs() < 1e-12,
                "plug no-caps fast path drifted at {p:?}: {a} vs reference {b}",
            );
        }

        let mut cumulative = 0.0_f64;
        for (i, layer_cfg) in cfg.layers.iter().enumerate() {
            cumulative += layer_cfg.thickness_m;
            let body_ref = Solid::from_sdf(sdf.clone(), sdf_bounds).offset(cumulative - inset);
            for p in probes {
                let a = derived.spec.layers[i].body.evaluate(&p);
                let b = body_ref.evaluate(&p);
                assert!(
                    (a - b).abs() < 1e-12,
                    "layer {i} no-caps fast path drifted at {p:?}: {a} vs reference {b}",
                );
            }
        }
    }

    #[test]
    fn derive_with_caps_plug_iso_zero_on_cap_plane_body_center() {
        // Synthetic unit-cube scan + a single cap on the +z face. The
        // plug = `pinned_floor_shell(scan, open_scan, bounds, [cap],
        // -inset)` and at the body center on the cap plane the cavity
        // SDF must sit on the boundary (≈ 0) — the consumer-level
        // surface of the structural-validation gate pinned at cf-design
        // level by
        // `pinned_floor_shell_cavity_iso_zero_on_cap_plane_at_body_center`.
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let cap = CapPlane {
            centroid: Point3::new(0.0, 0.0, 0.05),
            normal: Vector3::new(0.0, 0.0, 1.0),
            vertex_count: 4,
            loop_index: 0,
        };
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.005, &[cap])
                .unwrap();
        // Body center on cap plane: cavity SDF ≈ 0.
        let sd_on_cap = derived.spec.plug.evaluate(&Point3::new(0.0, 0.0, 0.05));
        assert!(
            sd_on_cap.abs() < 0.01,
            "plug SDF at body center on cap plane must be ≈ 0 (got {sd_on_cap})",
        );
        // Above cap plane: outside the plug (clipped).
        let sd_above = derived.spec.plug.evaluate(&Point3::new(0.0, 0.0, 0.06));
        assert!(
            sd_above > 0.0,
            "above cap plane must be outside plug, got {sd_above}",
        );
        // Deep inside body, well below the cap plane: strictly inside.
        let sd_deep = derived.spec.plug.evaluate(&Point3::new(0.0, 0.0, -0.02));
        assert!(
            sd_deep < 0.0,
            "deep interior must be inside plug, got {sd_deep}",
        );
    }
}
