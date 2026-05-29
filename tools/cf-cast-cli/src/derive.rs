//! Auto-derive a v2.1 [`cf_cast::CastSpec`] + [`cf_cast::Ribbon`]
//! from a parsed [`crate::config::CastConfig`] + a scan-SDF +
//! centerline polyline.
//!
//! Pure (no I/O); the heavy lifting (STL load, marching cubes, file
//! writes) happens upstream / downstream of this module.

use std::sync::Arc;

use anyhow::{Context, Result, bail};
use cf_cap_planes::{CapPlane, dome_wall_only_mesh};
use cf_cast::bolt_pattern::{BoltPatternKind, BoltPatternSpec};
use cf_cast::dowel_hole::{DowelHoleKind, DowelHoleSpec};
use cf_cast::{
    CanalSpec, CastLayer, CastSpec, FlangeKind, FlangeSpec, GasketKind, GasketMaterial, GasketSpec,
    MoldingMaterial, PlugPinKind, PlugPinSpec, PourGateKind, PourGateLayout, PourGateSpec, Ribbon,
    SplitNormal, best_fit_planar_seam, build_canal_plug,
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
/// The AABB passed to [`cf_design::Solid::from_sdf`] is the scan AABB padded by
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

    // Opt-in scan-mesh-direct routing for plug_layer_0 (S1 of
    // `docs/CF_CAST_SCAN_MESH_DIRECT_RECON.md`). Gated on both the
    // user-set TOML flag AND `cavity_inset_m == 0` — non-zero inset
    // would emit an oversized plug because the scan mesh has no
    // inward offset baked in. S2 of the recon extends this to the
    // offset cases with an explicit per-layer offset parameter.
    //
    // When the user opted in but `cavity_inset_m > 0`, surface a loud
    // warning so the silent fall-through to SDF/MC doesn't look like
    // a successful opt-in in the production-regen log. `eprintln`
    // rather than `bail!` because the SDF/MC path is still
    // workshop-correct; we just want to flag that the requested
    // feature didn't engage.
    let scan_mesh_for_plug_layer_0 = if config.cast.scan_mesh_direct_plug_layer_0 {
        if cavity_inset_m == 0.0 {
            Some(std::sync::Arc::new(scan_sdf.mesh().clone()))
        } else {
            eprintln!(
                "[cf-cast] WARNING: cast.scan_mesh_direct_plug_layer_0 = true was requested \
                 but cavity_inset_m = {cavity_inset_m_mm:.3} mm > 0; opt-in IGNORED for this \
                 run and the legacy SDF/MC plug pipeline is used. (Recon §SMD-4 S2 extends \
                 scan-mesh-direct to non-zero inset cases with an explicit per-layer offset \
                 parameter.)",
                cavity_inset_m_mm = cavity_inset_m * 1e3,
            );
            None
        }
    } else {
        None
    };

    // Canal Interior arc (Candidate A): compose parametric grip +
    // stimulation features onto the scan-derived layer-0 plug. Baseline
    // girth is unchanged — the layer/inset machinery (`cavity_inset_m`)
    // still owns tightness; the canal only ADDS features (grip rings,
    // frenulum D-section pinch, frenulum-gated texture, terminal suction
    // bulb). All features pinch the channel INWARD (more wall, safe)
    // except the suction bulb, which bulges OUTWARD into the inner
    // silicone shell — that one is wall-gated below.
    let (plug, plug_layer_0_mesh_cell_size_m) = if config.canal.enabled {
        // Mutually exclusive with scan-mesh-direct: both rewrite the
        // layer-0 plug. (scan-mesh-direct copies the scan mesh as-is,
        // which has no SDF surface to compose canal features onto.)
        if config.cast.scan_mesh_direct_plug_layer_0 {
            bail!(
                "[canal].enabled and [cast].scan_mesh_direct_plug_layer_0 are mutually \
                 exclusive — both rewrite the layer-0 plug. Disable one (the canal needs \
                 the SDF/MC plug path to compose its features)."
            );
        }
        let canal_spec = resolve_canal_spec(&config.canal);

        // Wall gate (§6.1, the only outward feature): the suction bulb
        // bulges the plug outward into the innermost silicone shell. If
        // the bulge plus the min-wall floor exceeds the inner layer's
        // thickness, the shell blows out to zero there. Inward features
        // (rings/D-section/texture) only ADD wall, so they need no gate.
        if canal_spec.suction_bulge_m > 0.0 {
            let min_wall_m = config.cast.piece_min_wall_mm / 1000.0;
            match config.layers.first().map(|l| l.thickness_m) {
                Some(inner_thickness_m) => {
                    if canal_spec.suction_bulge_m + min_wall_m > inner_thickness_m {
                        bail!(
                            "[canal] suction_bulge_m = {:.1} mm + min wall {:.1} mm exceeds the \
                             inner layer thickness {:.1} mm — the suction bulb would blow out the \
                             innermost silicone shell. Reduce suction_bulge_m or thicken layer 0.",
                            canal_spec.suction_bulge_m * 1e3,
                            min_wall_m * 1e3,
                            inner_thickness_m * 1e3,
                        );
                    }
                }
                None => {
                    eprintln!(
                        "[cf-cast] WARNING: [canal] suction bulb enabled but the inner layer \
                         thickness is unknown (design-sourced layers); the suction-bulb wall gate \
                         is SKIPPED. Verify the suction-bulb region clears the inner shell in \
                         cf-view before casting."
                    );
                }
            }
        }

        // Anchor the canal's frac=0 (mouth) at the cap-plane end. The
        // cap is the scan's cut base = the cavity's insertion opening;
        // without this, a centerline that runs deep→mouth (as
        // cf-scan-prep produced on the iter-1 clone) would invert the
        // canal and pile the suction bulb onto the flat floor cap.
        let mouth_anchor = cap_tuples.first().map(|(centroid, _normal)| *centroid);
        let canal_plug = build_canal_plug(&plug, centerline, mouth_anchor, &canal_spec);
        let cell = canal_spec.plug_mesh_cell_size_m;
        (canal_plug, Some(cell))
    } else {
        (plug, None)
    };

    let spec = CastSpec {
        layers,
        plug,
        bounding_region,
        wall_thickness_m: config.cast.wall_thickness_m,
        mesh_cell_size_m: config.cast.mesh_cell_size_m,
        printer_config,
        mass_budget_kg: config.cast.mass_budget_kg,
        scan_mesh_for_plug_layer_0,
        plug_layer_0_mesh_cell_size_m,
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

    // Plumb the cap-plane (centroid, outward_normal) through as
    // the ribbon's pour-end anchor when the scan has caps recorded
    // in `.prep.toml [caps]`. The plug-pin builders anchor the
    // pin cylinder at this centroid and extend it along the
    // outward normal — landing on the plug body's pinned floor
    // (which sits at the cap plane via `pinned_floor_shell`).
    //
    // **Why the centroid + normal, not a centerline endpoint**:
    // cf-scan-prep's `compute_centerline_polyline` trims the
    // centerline `trim_floor_mm` (typically 40 mm) ABOVE the cap
    // plane to keep the polyline inside the body interior. So
    // `centerline.last()` lives 40 mm inside the plug body, where
    // a pin anchored there is entirely buried (no visible
    // protrusion + no socket carve in the cup wall). The cap
    // plane is where the plug body's pinned floor sits, so a pin
    // anchored at the cap centroid extending along the outward
    // normal straddles the plug surface and cup wall correctly.
    //
    // Multiple caps (rare in practice) → use the first one; the
    // assumption that "the cap geometry is at the pour end" is
    // what justifies the hint in the first place.
    if let Some((centroid, normal)) = cap_tuples.first() {
        ribbon = ribbon.with_pour_end_hint(*centroid, *normal);
    }

    // (CF_CAST_ORGANIC_PARTS_RECON.md): opt-in flat seam for organic/curved
    // parts. The cup-wall halfspace + flange both read the ribbon's seam, so a
    // single call retargets both.
    //
    // Item A §4.1: by default (`planar_seam_fit`, default true) FIT the flat
    // plane to the body — anchored through the cap-centroid → apex axis and
    // rotated to the most-even split — so it follows the part's lean and
    // bisects the dome evenly instead of skimming it into a sliver. The
    // flange/bolt/dowel silhouette is built in the fitted plane (S2/S3), so it
    // builds manifold at any orientation. Needs caps for the apex anchor; with
    // no caps, or with `planar_seam_fit = false` (the escape hatch), it falls
    // back to the binormal-flatten `with_planar_seam`.
    if config.cast.planar_seam {
        let fitted_plane = if config.cast.planar_seam_fit {
            cap_tuples.first().and_then(|(centroid, normal)| {
                best_fit_planar_seam(scan_sdf.mesh(), *centroid, *normal)
            })
        } else {
            None
        };
        ribbon = match fitted_plane {
            Some((point, seam_normal)) => {
                let n = seam_normal.into_inner();
                eprintln!(
                    "[cf-cast-cli] planar seam fitted to body (apex-anchored): \
                     normal [{:.3}, {:.3}, {:.3}]",
                    n.x, n.y, n.z,
                );
                // The fit optimizes the SPLIT BALANCE, not demoldability — a flat
                // seam on a strongly-curved part can split 50/50 yet trap an
                // undercut against the plane on one half. There is no undercut
                // gate yet (recon CF_CAST_FLAT_FLOOR_RECON / organic-parts OQ2);
                // surface the risk on high-curvature parts so it isn't silent.
                let curl_deg = ribbon.max_tangent_rotation_rad().to_degrees();
                if curl_deg > 30.0 {
                    eprintln!(
                        "[cf-cast] WARNING: fitted flat seam on a strongly-curved part \
                         (max tangent rotation {curl_deg:.0}°) — a flat seam can trap a \
                         demold undercut on one half; verify the two halves release on \
                         the physical print.",
                    );
                }
                ribbon.with_planar_seam_at(point, n)
            }
            None => ribbon.with_planar_seam(),
        };
    }

    if config.pour_gate.enabled {
        // Organic-parts opt-in: single axial pour bore at the dome
        // apex on the seam (splits the flange; straight funnel;
        // hand-drilled vents) vs the iter-1 V-shape. Organic-parts
        // arc §4.3, 2026-05-29.
        let mut pour_spec = PourGateSpec::iter1();
        if config.pour_gate.apex_axial {
            pour_spec.layout = PourGateLayout::ApexAxial;
        }
        ribbon = ribbon.with_pour_gate(PourGateKind::Default(pour_spec));
    }
    if config.plug_pins.enabled {
        // Post-S4 of the FDM-friendly geometry arc the `pin_length_m`
        // + `include_dome_pin` per-field TOML overrides are retired
        // (see `config::PlugPinConfig` docstring). The bridge passes
        // the workshop-iter-3 `PlugPinSpec::iter1()` default — S7
        // workshop-physical calibration narrows the §G-6 / §G-8
        // numeric values rather than threading them through TOML.
        ribbon = ribbon.with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
    }
    if config.gasket.enabled {
        // S3 of the seam-gasket-mold arc per recon §G-7. Material
        // override is workshop-empirical (S6 iter-3 pour picks
        // Ecoflex vs Dragon Skin); cross-section + draft pinned at
        // S2 iter1 defaults until S6 calibration demands a tweak.
        let material = resolve_gasket_material(config.gasket.material.as_deref())?;
        let gasket_spec = GasketSpec {
            material,
            ..GasketSpec::iter1()
        };
        ribbon = ribbon.with_gasket(GasketKind::Mold(gasket_spec));
    }
    if config.flange.enabled {
        // S2 of the seam-flange arc per recon §F-6. Per-field
        // overrides fall back to FlangeSpec::iter1() at None; the
        // cross-field gasket-disjoint invariant (recon §F-4) is
        // already enforced in config::validate_after_layer_source,
        // so the spec built here is known-valid.
        let flange_spec = resolve_flange_spec(&config.flange);
        ribbon = ribbon.with_flange(FlangeKind::Plate(flange_spec));
    }
    if config.dowel_hole.enabled {
        // §M-S2 of [[project-cf-cast-unified-mating-plane-recon]].
        // Per-field overrides fall back to DowelHoleSpec::iter1().
        let dowel_spec = resolve_dowel_hole_spec(&config.dowel_hole);
        ribbon = ribbon.with_dowel_hole(DowelHoleKind::Auto(dowel_spec));
    }
    if config.bolt_pattern.enabled {
        // §B of [[project-cf-cast-flange-continuity-bolt-pattern-recon]].
        // Per-field overrides fall back to BoltPatternSpec::iter1();
        // cross-field invariants (flange-required, wall-thickness,
        // dowel-stagger) are gated in
        // config::validate_after_layer_source.
        let mut bolt_spec = resolve_bolt_pattern_spec(&config.bolt_pattern);
        // With the apex-axial pour the bore splits the flange at the
        // dome apex; bracket it (a bolt just outside the clearance on
        // each side) instead of dropping nearby bolts. Organic-parts
        // arc §4.3, 2026-05-29.
        if config.pour_gate.enabled && config.pour_gate.apex_axial {
            bolt_spec.bracket_pour_gate = true;
        }
        ribbon = ribbon.with_bolt_pattern(BoltPatternKind::Auto(bolt_spec));
    }

    Ok(DerivedSpec { spec, ribbon })
}

/// Resolve the [`crate::config::GasketConfig::material`] string into a
/// [`cf_cast::GasketMaterial`] enum value. `None` falls back to the
/// iter1 default (Ecoflex 00-30 per recon §G-1).
///
/// Recognized keys follow the same UPPER_SNAKE_CASE convention as
/// [`crate::config::LayerConfig::material`]: `"ECOFLEX_00_30"` +
/// `"DRAGON_SKIN_10A"`.
///
/// # Errors
///
/// Returns an error with the full set of recognized keys in the
/// message when the key is unknown (typo-friendly diagnostic).
fn resolve_gasket_material(key: Option<&str>) -> Result<GasketMaterial> {
    match key {
        None | Some("ECOFLEX_00_30") => Ok(GasketMaterial::Ecoflex0030),
        Some("DRAGON_SKIN_10A") => Ok(GasketMaterial::DragonSkin10A),
        Some(other) => bail!(
            "cast.toml: unknown gasket material {other:?}. Recognized: \
             \"ECOFLEX_00_30\", \"DRAGON_SKIN_10A\"."
        ),
    }
}

/// Resolve a [`crate::config::FlangeConfig`] into a
/// [`cf_cast::FlangeSpec`], falling back to
/// [`FlangeSpec::iter1`] for each `None` per-field override.
///
/// Pure (no validation — finiteness + positivity + cross-field
/// gasket-disjoint gates already ran in
/// [`crate::config::CastConfig::validate_after_layer_source`]).
/// Returns by-value because `FlangeSpec` is plain-old-f64-struct
/// (Copy-able by construction).
fn resolve_flange_spec(config: &crate::config::FlangeConfig) -> FlangeSpec {
    let iter1 = FlangeSpec::iter1();
    FlangeSpec {
        flange_width_m: config.width_m.unwrap_or(iter1.flange_width_m),
        flange_thickness_m: config.thickness_m.unwrap_or(iter1.flange_thickness_m),
        flange_inner_offset_m: config.inner_offset_m.unwrap_or(iter1.flange_inner_offset_m),
    }
}

/// Resolve a [`crate::config::CanalConfig`] into a [`CanalSpec`],
/// falling back to [`CanalSpec::iter1`] for each `None` per-field
/// override. Canal Interior arc (Candidate A) — additive grip +
/// stimulation features on the layer-0 plug; baseline girth stays
/// owned by `cavity_inset_m`.
fn resolve_canal_spec(config: &crate::config::CanalConfig) -> CanalSpec {
    let mut spec = CanalSpec::iter1();
    if let Some(dir) = config.frenulum_dir {
        spec.frenulum_dir = Vector3::new(dir[0], dir[1], dir[2]);
    }
    if let Some(amp) = config.texture_amplitude_m {
        spec.texture_amp_m = amp;
    }
    if let Some(pitch) = config.texture_pitch_m {
        spec.texture_pitch_m = pitch;
    }
    if let Some(depth) = config.dsection_depth_m {
        spec.dsection_depth_m = depth;
    }
    if let Some(bulge) = config.suction_bulge_m {
        spec.suction_bulge_m = bulge;
    }
    if let Some(cell) = config.plug_mesh_cell_size_m {
        spec.plug_mesh_cell_size_m = cell;
    }
    spec
}

/// Resolve a [`crate::config::DowelHoleConfig`] into a
/// [`DowelHoleSpec`], falling back to [`DowelHoleSpec::iter1`] for
/// each `None` per-field override. §M-S2 of
/// [[project-cf-cast-unified-mating-plane-recon]].
fn resolve_dowel_hole_spec(config: &crate::config::DowelHoleConfig) -> DowelHoleSpec {
    let iter1 = DowelHoleSpec::iter1();
    DowelHoleSpec {
        diameter_m: config.diameter_m.unwrap_or(iter1.diameter_m),
        clearance_m: config.clearance_m.unwrap_or(iter1.clearance_m),
        depth_m: config.depth_m.unwrap_or(iter1.depth_m),
        count: config.count.unwrap_or(iter1.count),
        silhouette_outboard_offset_m: config
            .silhouette_outboard_offset_m
            .unwrap_or(iter1.silhouette_outboard_offset_m),
    }
}

/// Resolve a [`crate::config::BoltPatternConfig`] into a
/// [`BoltPatternSpec`], falling back to [`BoltPatternSpec::iter1`]
/// for each `None` per-field override. §B of
/// [[project-cf-cast-flange-continuity-bolt-pattern-recon]].
fn resolve_bolt_pattern_spec(config: &crate::config::BoltPatternConfig) -> BoltPatternSpec {
    let iter1 = BoltPatternSpec::iter1();
    BoltPatternSpec {
        clearance_diameter_m: config
            .clearance_diameter_m
            .unwrap_or(iter1.clearance_diameter_m),
        count: config.count.unwrap_or(iter1.count),
        silhouette_outboard_offset_m: config
            .silhouette_outboard_offset_m
            .unwrap_or(iter1.silhouette_outboard_offset_m),
        skip_pour_gate_collision: config
            .skip_pour_gate_collision
            .unwrap_or(iter1.skip_pour_gate_collision),
        pour_gate_clearance_m: config
            .pour_gate_clearance_m
            .unwrap_or(iter1.pour_gate_clearance_m),
        // Set by the caller from `[pour_gate].apex_axial`, not a
        // `[bolt_pattern]` field — the bracket only makes sense when
        // the apex-axial pour bore splits the flange.
        bracket_pour_gate: iter1.bracket_pour_gate,
    }
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
            gasket: crate::config::GasketConfig::default(),
            flange: crate::config::FlangeConfig::default(),
            dowel_hole: crate::config::DowelHoleConfig::default(),
            bolt_pattern: crate::config::BoltPatternConfig::default(),
            canal: crate::config::CanalConfig::default(),
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
    fn derive_ribbon_carries_features_when_enabled() {
        // §M-S4 (2026-05-27): the legacy prismatic-pin registration
        // path is retired; this test no longer probes
        // `ribbon.registration` — see the dowel-hole-on-by-default
        // matcher below for the post-§M-S4 registration equivalent.
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
        assert!(
            matches!(derived.ribbon.pour_gate, PourGateKind::Default(_)),
            "pour_gate.enabled=true must produce Default(_)"
        );
        assert!(
            matches!(derived.ribbon.plug_pins, PlugPinKind::Axial(_)),
            "plug_pins.enabled=true must produce Axial(_)"
        );
        assert!(
            matches!(derived.ribbon.dowel_hole, DowelHoleKind::Auto(_)),
            "dowel_hole.enabled=true must produce Auto(_)"
        );
        assert!(
            matches!(derived.ribbon.bolt_pattern, BoltPatternKind::Auto(_)),
            "bolt_pattern.enabled=true must produce Auto(_)"
        );
    }

    #[test]
    fn derive_ribbon_disables_features_when_config_disables_them() {
        let mut cfg = three_layer_config();
        cfg.pour_gate.enabled = false;
        cfg.plug_pins.enabled = false;
        cfg.dowel_hole.enabled = false;
        cfg.bolt_pattern.enabled = false;
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
        assert!(matches!(derived.ribbon.pour_gate, PourGateKind::None));
        assert!(matches!(derived.ribbon.plug_pins, PlugPinKind::None));
        assert!(matches!(derived.ribbon.dowel_hole, DowelHoleKind::None));
        assert!(matches!(derived.ribbon.bolt_pattern, BoltPatternKind::None));
    }

    #[test]
    fn derive_plug_pin_axial_uses_iter1_defaults() {
        // Post-S4 the per-field TOML overrides on `[plug_pins]`
        // (pre-S4: `pin_length_m`, `include_dome_pin`) are retired
        // — the bridge passes `PlugPinSpec::iter1()` directly. The
        // pre-S4 `derive_plug_pin_length_override_passes_through`
        // test retired with the override path; this test pins the
        // post-S4 contract.
        let cfg = three_layer_config();
        let sdf = unit_cube_sdf();
        let centerline = straight_x_centerline();
        let derived =
            derive_spec_and_ribbon(&cfg, &sdf, unit_cube_aabb(), &centerline, 0.0, &[]).unwrap();
        let PlugPinKind::Axial(spec) = &derived.ribbon.plug_pins else {
            unreachable!("plug_pins.enabled=true should yield Axial(_)")
        };
        assert_eq!(*spec, PlugPinSpec::iter1());
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

    /// S1 of CF_CAST_SCAN_MESH_DIRECT_RECON.md: when the cast.toml
    /// opt-in is set and `cavity_inset_m == 0`, the derived spec
    /// must carry the scan mesh through to `export_molds_v2`.
    #[test]
    fn scan_mesh_direct_opt_in_populates_spec_field_at_zero_inset() {
        let mut cfg = three_layer_config();
        cfg.cast.scan_mesh_direct_plug_layer_0 = true;
        let sdf = unit_cube_sdf();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            unit_cube_aabb(),
            &straight_x_centerline(),
            0.0,
            &[],
        )
        .unwrap();
        let mesh = derived
            .spec
            .scan_mesh_for_plug_layer_0
            .as_ref()
            .expect("opt-in at cavity_inset_m == 0 must populate the field");
        // Mesh-identity gate: the populated mesh must come from
        // `scan_sdf.mesh()` (same vertex + face counts as the
        // unit_cube_sdf fixture). Catches accidental routing to a
        // different mesh source.
        assert_eq!(mesh.vertices.len(), sdf.mesh().vertices.len());
        assert_eq!(mesh.faces.len(), sdf.mesh().faces.len());
    }

    /// Sibling of `_at_zero_inset` above: the cavity_inset_m > 0 gate
    /// silently drops the opt-in to None (with a stderr warning the
    /// integration test can't easily assert on). Pre-S2, scan-mesh-
    /// direct doesn't support non-zero inset.
    #[test]
    fn scan_mesh_direct_opt_in_ignored_when_cavity_inset_nonzero() {
        let mut cfg = three_layer_config();
        cfg.cast.scan_mesh_direct_plug_layer_0 = true;
        let sdf = unit_cube_sdf();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            unit_cube_aabb(),
            &straight_x_centerline(),
            0.001,
            &[],
        )
        .unwrap();
        assert!(
            derived.spec.scan_mesh_for_plug_layer_0.is_none(),
            "scan-mesh-direct opt-in must NOT engage when cavity_inset_m > 0 \
             (the scan mesh has no inward offset baked in; S2 of the recon \
             extends this path to non-zero inset)",
        );
    }

    /// Default cast.toml (no `scan_mesh_direct_plug_layer_0`) preserves
    /// pre-S1 bit-for-bit output. Pairs with the opt-in tests above.
    #[test]
    fn scan_mesh_direct_default_leaves_spec_field_unset() {
        let cfg = three_layer_config();
        assert!(
            !cfg.cast.scan_mesh_direct_plug_layer_0,
            "TOML default must be false so existing cast.toml files \
             continue through the SDF/MC plug pipeline",
        );
        let sdf = unit_cube_sdf();
        let derived = derive_spec_and_ribbon(
            &cfg,
            &sdf,
            unit_cube_aabb(),
            &straight_x_centerline(),
            0.0,
            &[],
        )
        .unwrap();
        assert!(derived.spec.scan_mesh_for_plug_layer_0.is_none());
    }
}
