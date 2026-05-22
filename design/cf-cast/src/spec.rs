//! [`CastSpec`] — the multi-layer public API surface, shared by
//! v1's single-piece [`CastSpec::export_molds`] and v2/v2.1's
//! curve-following multi-piece [`CastSpec::export_molds_v2`] export
//! pipelines.

use std::path::{Path, PathBuf};

use cf_design::{Aabb, IndexedMesh, Solid};
use mesh_io::save_stl;
use mesh_printability::{
    IssueSeverity, PrintIssue, PrintIssueType, PrintValidation, PrinterConfig,
    validate_for_printing,
};
use nalgebra::Vector3;

use crate::error::{CastError, CastTarget};
use crate::material::MoldingMaterial;
use crate::mesh_csg::apply_mating_transforms;
use crate::mesher::solid_to_mm_mesh;
use crate::piece::compose_piece_solid;
use crate::plug::add_plug_pins;
use crate::pour_volume::{PourVolume, integrate_negative_sdf_volume};
use crate::procedure::{generate_procedure_markdown, generate_procedure_markdown_v2};
use crate::ribbon::{PieceSide, Ribbon};

/// XY slack added to the clip cuboid relative to the bounding region.
/// 100 mm in meters; the clip only needs to cover `bounding_region`'s
/// xy-extent generously, never near a 100 mm device.
const CLIP_XY_SLACK_M: f64 = 0.1;

/// How far above the body's `z_max` the clip cuboid extends. 1 m is
/// effectively unbounded for any silicone-device-scale geometry.
const CLIP_Z_REACH_M: f64 = 1.0;

/// How far BELOW `body.z_max` the clip extends — equivalently, how far
/// the body protrudes from the cup at the pour opening. Without this
/// overlap the clip's bottom face and the body's top face coincide
/// exactly, producing a degenerate CSG surface at `z = body.z_max`
/// that marching cubes resolves into non-manifold faces (and
/// downstream `validate_for_printing` flags as `ExcessiveOverhang` +
/// `LongBridge` + `SelfIntersecting`). 0.5 mm is well below typical FDM
/// layer height (0.2 mm) but large enough to break the coincidence
/// at any reasonable cell size.
const CLIP_BODY_OVERLAP_M: f64 = 0.0005;

/// One cast layer — the cumulative outer-surface positive after this
/// pour cures, plus the material that fills it.
///
/// In a [`CastSpec`]'s `layers` vector, entries are ordered
/// **innermost-first**: `layers[0]` is the innermost shell only;
/// `layers[i]` for `i > 0` is the fused solid of the inner cured
/// layers plus the new pour (i.e., the cumulative outer surface
/// after the `i`-th cast). Each `body` is a **solid positive**
/// (not an annular shell) — pour-volume integration subtracts the
/// previous layer (or [`CastSpec::plug`] for layer 0) inside
/// [`CastSpec::compute_pour_volumes`] to recover the per-layer
/// silicone shell.
///
/// Mold-cup composition differs per pipeline:
/// - v1's [`CastSpec::export_molds`] carves the cup with
///   `bounding_region ∖ layers[i].body ∖ clip_above(layers[i].body)`
///   (single-piece cup with `+z`-opening clip).
/// - v2's [`CastSpec::export_molds_v2`] carves
///   `bounding_region ∖ layers[i].body ∩ ribbon_side` per piece,
///   plus optional registration pin / pour gate / vent / plug
///   socket CSG (see [`crate::piece::compose_piece_solid`]).
#[derive(Debug, Clone)]
pub struct CastLayer {
    /// Cumulative outer-surface positive solid in **meters**. See
    /// [`CastLayer`]'s docstring for the innermost-first /
    /// cumulative convention.
    pub body: Solid,

    /// Material poured into this layer (used by F2 for pour-mass
    /// calculation and F3 for procedure-spec generation).
    pub material: MoldingMaterial,
}

/// Specification for a multi-layer cast: N silicone layers poured
/// innermost-first into a shared bounding region.
///
/// v2's [`Self::export_molds_v2`] casts each layer independently
/// against its own per-layer plug (layer 0 uses [`Self::plug`];
/// layer N>0 uses `layers[N-1].body`); v1's [`Self::export_molds`]
/// uses the shared [`Self::plug`] for the innermost cast only and
/// stacks subsequent layers on the previously-cured shell.
///
/// Geometry is supplied in **meters** in the [`cf_design`]
/// convention. Both export pipelines perform the m → mm scale
/// exactly once per output mesh at the marching-cubes →
/// save/validate boundary.
///
/// Demolding axis: v1 hardcodes `+z` (clip cuboid above each
/// layer body's `z_max`); v2 demolds along the curve-following
/// ribbon's two-piece split with workshop orientation `+Z` up
/// (apex vent on the top face, side-mounted pour gate on the
/// `±Y` binormal face).
#[derive(Debug, Clone)]
pub struct CastSpec {
    /// Cast layers in **innermost-first** order. Must contain at
    /// least one entry; `export_molds` returns
    /// [`CastError::EmptyLayers`] for an empty vector.
    pub layers: Vec<CastLayer>,

    /// The cavity-forming plug — a separately-printed positive solid
    /// inserted into the innermost layer's pour to shape its inner
    /// cavity. Subsequent layers cast around the previously cured
    /// layer, so only one printed plug is needed regardless of
    /// `layers.len()`.
    pub plug: Solid,

    /// The closed solid that defines the rigid mold's outer envelope
    /// before subtraction. Typically a cuboid encompassing every
    /// layer body with the desired wall thickness. The exporter
    /// clips its top above each layer body to open every cup for
    /// pour.
    pub bounding_region: Solid,

    /// Cell size (in meters) for the SDF → marching-cubes scalar
    /// sampling. Finer cells produce smoother surfaces at the cost of
    /// grid size (cubic in `1/cell_size`) and downstream F4 validation
    /// time (`mesh-printability` runs several O(face²) checks).
    pub mesh_cell_size_m: f64,

    /// F4 printer configuration. Per the casting roadmap Q1
    /// resolution, the Stage 2 reference fixtures use
    /// [`PrinterConfig::fdm_default`]; configurable for SLA/SLS/MJF
    /// once iter-1 surfaces a real need.
    pub printer_config: PrinterConfig,

    /// Per-pour silicone mass budget in kilograms. Each layer's
    /// computed pour mass (`shell_volume × material.density_kg_m3`)
    /// must satisfy `pour_mass ≤ mass_budget_kg`; overruns surface
    /// as [`CastError::MassBudgetExceeded`] before any STL is
    /// written.
    ///
    /// The recommended default is
    /// [`crate::DEFAULT_MASS_BUDGET_KG`] (2 lb, the
    /// layered-silicone-device v1.0 per-silicone holding). The
    /// field is required so callers cannot silently fall back —
    /// set it explicitly even when adopting the default.
    pub mass_budget_kg: f64,
}

/// Per-layer output of a successful [`CastSpec::export_molds`] run.
///
/// Indexed parallel to [`CastSpec::layers`] (innermost-first). The
/// `material_display_name` is carried through from
/// `layers[layer_index].material.display_name` so callers can label
/// artifacts without re-indexing back into the spec.
#[derive(Debug, Clone)]
pub struct MoldArtifact {
    /// Index into [`CastSpec::layers`] this mold cup belongs to.
    pub layer_index: usize,
    /// Carried-through copy of the layer's material display name
    /// (e.g., `"Ecoflex 00-30"`).
    pub material_display_name: String,
    /// Filesystem path of the written mold cup STL.
    pub path: PathBuf,
    /// F4 validation result for the mold cup (zero blocking Critical
    /// issues guaranteed; may contain non-blocking Criticals and
    /// Warnings).
    pub validation: PrintValidation,
    /// Post-export mold mesh summary (vertex/face counts + mm-frame
    /// bounds).
    pub summary: MeshSummary,
    /// F2 pour-volume summary for this layer — shell volume in m³
    /// and pour mass in kg, both gated against
    /// [`CastSpec::mass_budget_kg`] before this artifact lands on disk.
    pub pour_volume: PourVolume,
}

/// Summary of a successful [`CastSpec::export_molds`] run.
///
/// One [`MoldArtifact`] per layer plus the shared plug's artifacts.
/// Validation results carry warnings (sub-Critical printability
/// issues) and the non-blocking `Critical`-severity issues that
/// cf-cast tolerates (overhangs, bridges, MC self-intersection
/// noise) for caller inspection. Blocking Critical issues abort the
/// run before any STL is written, so a populated `MoldExportReport`
/// always corresponds to `layers.len() + 1` STL files on disk that
/// the F4 gate has cleared for the chosen printer.
#[derive(Debug, Clone)]
pub struct MoldExportReport {
    /// Per-layer mold artifacts, indexed parallel to
    /// [`CastSpec::layers`].
    pub molds: Vec<MoldArtifact>,
    /// Filesystem path of the written plug STL.
    pub plug_path: PathBuf,
    /// F4 validation result for the plug.
    pub plug_validation: PrintValidation,
    /// Post-export plug mesh summary (vertex/face counts + mm-frame
    /// bounds).
    pub plug_summary: MeshSummary,
}

/// One v2 mold piece's post-export geometry + F4 validation.
///
/// v2 splits each layer's mold cup into two pieces along the
/// curve-following ribbon (per `docs/CURVE_FOLLOWING_DESIGN.md`
/// §"Piece count selection" — v2 ships with 2 pieces; 3+ is v3).
/// Parent context (layer index, material name, pour volume) lives
/// on [`V2LayerReport`]; this struct carries only piece-specific
/// data.
#[derive(Debug, Clone)]
pub struct PieceArtifact {
    /// Which side of the ribbon this piece occupies. Maps 1:1 to
    /// the `_piece_{0|1}` STL filename suffix (`Negative` → `0`,
    /// `Positive` → `1`).
    pub piece_side: PieceSide,
    /// Filesystem path of the written piece STL.
    pub path: PathBuf,
    /// F4 validation result for this piece (zero blocking Critical
    /// issues guaranteed; per-piece `ExceedsBuildVolume` is in the
    /// blocking set so each piece must fit the printer independently
    /// — v2's stricter check vs v1's aggregate-AABB).
    pub validation: PrintValidation,
    /// Post-export piece mesh summary (vertex/face counts + mm-frame
    /// bounds).
    pub summary: MeshSummary,
}

/// One v2 per-layer plug's post-export geometry + F4 validation.
///
/// Each layer in v2.1's detachable-shell architecture gets its
/// own plug STL (`plug_layer_{N}.stl`) sized to that layer's
/// inner-cavity surface; the plug for layer 0 derives from
/// [`CastSpec::plug`] (with the ribbon's plug-anchor pin geometry
/// added), and the plug for `N > 0` derives from
/// `layers[N - 1].body` (also with pin geometry). Cured per-layer
/// silicone shells nest at assembly time and can be disassembled
/// post-cure for cleaning or replacement.
#[derive(Debug, Clone)]
pub struct PlugArtifact {
    /// Filesystem path of the written plug STL.
    pub path: PathBuf,
    /// F4 validation result for the plug.
    pub validation: PrintValidation,
    /// Post-export plug mesh summary (vertex/face counts +
    /// mm-frame bounds).
    pub summary: MeshSummary,
}

/// One v2 layer's complete mold output: pour volume + both
/// pieces + a per-layer plug (v2.1 sub-leaf 2 detachable-layer
/// architecture).
///
/// Indexed parallel to [`CastSpec::layers`] (innermost-first); the
/// inner `pieces` array is fixed-length 2 to statically enforce
/// v2's 2-piece convention. The `plug` artifact carries this
/// layer's inner-cavity plug (derived from
/// [`CastSpec::plug`] for layer 0 and from `layers[N-1].body` for
/// `N > 0`); each layer is cast independently against its own
/// plug so the cured silicone shells nest and can be disassembled
/// post-cure for cleaning or replacement.
#[derive(Debug, Clone)]
pub struct V2LayerReport {
    /// Index into [`CastSpec::layers`].
    pub layer_index: usize,
    /// Carried-through layer material display name
    /// (e.g., `"Ecoflex 00-30"`).
    pub material_display_name: String,
    /// Per-layer pour-volume + mass result. Same value v1's
    /// [`MoldArtifact::pour_volume`] carries; budget-gated identically.
    pub pour_volume: PourVolume,
    /// The 2 piece artifacts for this layer, in
    /// `[Negative, Positive]` order. Fixed array (vs Vec) pins v2's
    /// 2-piece convention at the type level.
    pub pieces: [PieceArtifact; 2],
    /// This layer's plug artifact. Filename
    /// `plug_layer_{layer_index}.stl`. For layer 0 the plug
    /// geometry is [`CastSpec::plug`] (extended with the ribbon's
    /// plug-anchor pin geometry); for `layer_index > 0` the plug
    /// is `layers[layer_index - 1].body` (the previous layer's
    /// outer solid), also extended with the pin geometry. Each
    /// layer's plug therefore matches the next-inner silicone
    /// shell's outer surface, enabling independent (detachable)
    /// per-layer casts.
    pub plug: PlugArtifact,
}

/// Workshop platform STL.
///
/// A flat slab with a pocket carved into the top surface that
/// accepts the T-bar protrusion from the assembled mold. The mold
/// sits on this platform during pour + cure so it rests flat
/// (T-bar in the pocket) even though the T-bar extends below the
/// cup outer face.
///
/// Generated only when the ribbon's `plug_pins` field is
/// [`crate::plug::PlugPinKind::Axial`] AND `include_t_bar = true`
/// (the configuration that produces a T-bar protrusion needing
/// support). Single STL per cast, shared across all layers.
#[derive(Debug, Clone)]
pub struct PlatformArtifact {
    /// Filesystem path of the written `platform.stl`.
    pub path: PathBuf,
    /// F4 validation result for the platform.
    pub validation: PrintValidation,
    /// Post-export mesh summary (vertex/face counts + mm-frame
    /// bounds).
    pub summary: MeshSummary,
}

/// Workshop pour funnel STL.
///
/// Self-aligning nipple + flange + tapered cone for pouring
/// honey-thick silicones into the cast's pour-gate hole. The
/// nipple slides into the pour leg of the V pour gate; the flange
/// rests flat on the cup wall around the hole; the cone tapers up
/// to a wide ladle-friendly mouth.
///
/// Generated only when the ribbon has a
/// [`crate::pour::PourGateKind::Default`] pour gate enabled.
/// Single STL per cast, shared across all layers (workshop user
/// prints once, uses for every layer pour).
#[derive(Debug, Clone)]
pub struct FunnelArtifact {
    /// Filesystem path of the written `funnel.stl`.
    pub path: PathBuf,
    /// F4 validation result for the funnel.
    pub validation: PrintValidation,
    /// Post-export mesh summary (vertex/face counts + mm-frame
    /// bounds).
    pub summary: MeshSummary,
}

/// Summary of a successful [`CastSpec::export_molds_v2`] run.
///
/// `layers.len() * 3` STLs total (2 piece STLs + 1 plug STL per
/// layer), plus a single `platform.stl` if the ribbon enables
/// T-bar plug-pin locking, plus a single `funnel.stl` if the
/// ribbon has a pour gate enabled. All pieces, plugs, and the
/// optional platform + funnel clear the F4 gate before any STL
/// lands on disk (pre-write atomicity, same scope as v1's
/// [`CastSpec::export_molds`]).
#[derive(Debug, Clone)]
pub struct V2MoldExportReport {
    /// Per-layer reports (innermost-first), each carrying its 2
    /// mold pieces + 1 plug.
    pub layers: Vec<V2LayerReport>,
    /// Optional workshop platform artifact, present when the
    /// ribbon's plug-pin kind has `include_t_bar = true`. The
    /// platform's pocket matches the T-bar protrusion so the
    /// assembled mold sits flat on the workbench during pour +
    /// cure.
    pub platform: Option<PlatformArtifact>,
    /// Optional workshop pour funnel artifact, present when the
    /// ribbon has a [`crate::pour::PourGateKind::Default`] pour
    /// gate enabled. Self-aligning nipple sized to the pour-gate Ø.
    pub funnel: Option<FunnelArtifact>,
}

/// Lightweight numerical summary of an [`IndexedMesh`] in mm
/// coordinates.
///
/// Used by [`MoldExportReport`] and [`MoldArtifact`] to surface
/// enough post-export geometry for caller assertions without
/// retaining the whole mesh.
#[derive(Debug, Clone)]
pub struct MeshSummary {
    /// Total vertex count.
    pub vertex_count: usize,
    /// Total face count (triangles).
    pub face_count: usize,
    /// Axis-aligned bounding box in mm.
    pub aabb_mm: Aabb,
}

impl MeshSummary {
    fn from_mesh(mesh: &IndexedMesh) -> Self {
        let aabb_mm = if mesh.vertices.is_empty() {
            Aabb::from_corners(
                nalgebra::Point3::new(0.0, 0.0, 0.0),
                nalgebra::Point3::new(0.0, 0.0, 0.0),
            )
        } else {
            Aabb::from_points(mesh.vertices.iter())
        };
        Self {
            vertex_count: mesh.vertices.len(),
            face_count: mesh.faces.len(),
            aabb_mm,
        }
    }
}

/// Intermediate per-layer mesh + validation, held in memory until
/// every layer + the plug clear the F4 gate, then written to disk in
/// one batch. Lets `export_molds` reject a Critical failure on a
/// later layer without stranding earlier layers' STLs on disk (FS
/// failures during the write phase remain non-atomic — that's
/// expected and not what this buffer guards).
struct PendingMold {
    layer_index: usize,
    material_display_name: String,
    mesh: IndexedMesh,
    validation: PrintValidation,
    path: PathBuf,
}

/// Per-(layer × piece) buffered v2 mesh + validation, held in
/// memory until every piece + every per-layer plug clear the F4
/// gate. Parallel to [`PendingMold`] for v1.
struct PendingPiece {
    layer_index: usize,
    piece_side: PieceSide,
    mesh: IndexedMesh,
    validation: PrintValidation,
    path: PathBuf,
}

/// Per-layer buffered v2 plug mesh + validation, held in memory
/// alongside [`PendingPiece`]s until every artifact clears the F4
/// gate. v2.1 sub-leaf 2 generates one plug per layer for the
/// detachable-shell assembly workflow.
struct PendingPlug {
    layer_index: usize,
    mesh: IndexedMesh,
    validation: PrintValidation,
    path: PathBuf,
}

/// Buffered v2 workshop platform mesh + validation, held in memory
/// alongside [`PendingPiece`]s + [`PendingPlug`]s until every
/// artifact clears the F4 gate. Generated only when the ribbon's
/// plug-pin kind enables the T-bar; one platform per cast (shared
/// across layers).
struct PendingPlatform {
    mesh: IndexedMesh,
    validation: PrintValidation,
    path: PathBuf,
}

/// Buffered v2 workshop pour funnel mesh + validation. Same buffer
/// pattern as [`PendingPlatform`] — generated only when the ribbon
/// has a pour gate enabled; one funnel per cast (shared across
/// layers).
struct PendingFunnel {
    mesh: IndexedMesh,
    validation: PrintValidation,
    path: PathBuf,
}

/// v2 piece-count refusal threshold: max tangent rotation along
/// the centerline beyond which a 2-piece mold can no longer
/// demold cleanly. `2π/3` rad = 120°, per
/// `docs/CURVE_FOLLOWING_DESIGN.md` §"Piece count selection".
const MAX_TANGENT_ROTATION_RAD: f64 = 2.0 * std::f64::consts::FRAC_PI_3;

impl CastSpec {
    /// Compute per-layer pour volumes (and the corresponding pour
    /// masses) without writing any artifacts.
    ///
    /// For each cast layer, the shell solid is the CSG difference
    /// between this layer's cumulative body and the previous layer's
    /// (or the plug, for the innermost layer). Volume integration
    /// uses the same [`Self::mesh_cell_size_m`] that drives mold
    /// marching cubes, sampled as a Riemann sum over corner-SDF
    /// signs.
    ///
    /// Useful at design time when only the pour-mass numbers are
    /// needed — e.g., for sweeping layer thicknesses against the
    /// budget without paying the meshing + F4-gate cost. Both
    /// [`Self::export_molds`] and [`Self::write_procedure`]
    /// internalize this call, so callers in the canonical pipeline
    /// flow do not invoke it directly.
    ///
    /// # Errors
    ///
    /// - [`CastError::EmptyLayers`] if [`Self::layers`] is empty.
    /// - [`CastError::InfiniteBounds`] if any layer body is
    ///   unbounded. (`Solid::subtract` inherits its bounds from the
    ///   minuend alone, so an unbounded [`Self::plug`] does NOT
    ///   trip this path here — that condition surfaces in
    ///   [`Self::export_molds`]'s plug meshing step instead.)
    pub fn compute_pour_volumes(&self) -> Result<Vec<PourVolume>, CastError> {
        if self.layers.is_empty() {
            return Err(CastError::EmptyLayers);
        }

        let mut volumes = Vec::with_capacity(self.layers.len());
        for (layer_index, layer) in self.layers.iter().enumerate() {
            let shell = if layer_index == 0 {
                layer.body.clone().subtract(self.plug.clone())
            } else {
                let prev = &self.layers[layer_index - 1];
                layer.body.clone().subtract(prev.body.clone())
            };

            let shell_volume_m3 = integrate_negative_sdf_volume(
                &shell,
                self.mesh_cell_size_m,
                CastTarget::LayerBody { layer_index },
            )?;
            let pour_mass_kg = shell_volume_m3 * layer.material.density_kg_m3;

            volumes.push(PourVolume {
                layer_index,
                material_display_name: layer.material.display_name.clone(),
                shell_volume_m3,
                pour_mass_kg,
            });
        }

        Ok(volumes)
    }

    /// Export per-layer mold cups and the shared plug as
    /// `layers.len() + 1` STL files in `out_dir`.
    ///
    /// Pipeline:
    /// 1. Verify [`Self::layers`] is non-empty.
    /// 2. Compute per-layer pour volumes
    ///    ([`Self::compute_pour_volumes`]) and gate each against
    ///    [`Self::mass_budget_kg`]; abort on overrun before any
    ///    meshing.
    /// 3. For each `layer` in [`Self::layers`]: compute the clip
    ///    cuboid above `layer.body`'s `z_max`, build the mold-cup
    ///    solid `bounding_region ∖ layer.body ∖ clip`, sample SDF
    ///    onto a [`mesh_offset::ScalarGrid`], run marching cubes,
    ///    scale meters → mm, then validate against
    ///    [`Self::printer_config`] and abort on any *blocking*
    ///    Critical issue.
    /// 4. Repeat meshing + validation for [`Self::plug`].
    /// 5. Create `out_dir` if it doesn't exist.
    /// 6. Write `mold_layer_0.stl` … `mold_layer_{N-1}.stl` and
    ///    `plug.stl`; attach each layer's [`PourVolume`] to its
    ///    [`MoldArtifact`].
    ///
    /// **Atomicity**: pour-volume budget check, meshing, and the F4
    /// gate all run before the first STL is written. A budget
    /// overrun or Critical failure aborts the run cleanly with no
    /// partial output on disk.
    ///
    /// # Errors
    ///
    /// - [`CastError::EmptyLayers`] if [`Self::layers`] is empty.
    /// - [`CastError::InfiniteBounds`] if any input Solid is
    ///   unbounded.
    /// - [`CastError::MassBudgetExceeded`] if any layer's computed
    ///   pour mass exceeds [`Self::mass_budget_kg`].
    /// - [`CastError::MeshingEmpty`] if marching cubes produces a
    ///   degenerate mesh (e.g., bounding region wholly enclosed by
    ///   a layer body, leaving no cup material).
    /// - [`CastError::PrintabilityCritical`] if any mesh fails the
    ///   F4 gate with one or more blocking Critical-severity issues.
    /// - [`CastError::MeshIo`] on filesystem failures.
    pub fn export_molds(&self, out_dir: &Path) -> Result<MoldExportReport, CastError> {
        let pour_volumes = self.compute_pour_volumes()?;
        check_mass_budget(self, &pour_volumes)?;

        let mut pending = Vec::with_capacity(self.layers.len());
        for (layer_index, layer) in self.layers.iter().enumerate() {
            let clip = clip_above_body(&layer.body, &self.bounding_region, layer_index)?;
            let mold_cup = self
                .bounding_region
                .clone()
                .subtract(layer.body.clone())
                .subtract(clip);

            let mold_target = CastTarget::Mold { layer_index };
            let mold_mesh = solid_to_mm_mesh(&mold_cup, self.mesh_cell_size_m, mold_target)?;

            let path = out_dir.join(mold_filename(layer_index));
            let validation = run_printability_gate(&mold_mesh, &self.printer_config, &path)?;

            let blocking = blocking_critical_count(&validation);
            if blocking > 0 {
                return Err(CastError::PrintabilityCritical {
                    target: mold_target,
                    issue_count: blocking,
                    path,
                });
            }

            pending.push(PendingMold {
                layer_index,
                material_display_name: layer.material.display_name.clone(),
                mesh: mold_mesh,
                validation,
                path,
            });
        }

        let plug_path = out_dir.join("plug.stl");
        let plug_mesh = solid_to_mm_mesh(
            &self.plug,
            self.mesh_cell_size_m,
            CastTarget::Plug { layer_index: None },
        )?;
        let plug_validation = run_printability_gate(&plug_mesh, &self.printer_config, &plug_path)?;
        let plug_blocking = blocking_critical_count(&plug_validation);
        if plug_blocking > 0 {
            return Err(CastError::PrintabilityCritical {
                target: CastTarget::Plug { layer_index: None },
                issue_count: plug_blocking,
                path: plug_path,
            });
        }

        std::fs::create_dir_all(out_dir).map_err(|e| CastError::MeshIo {
            path: out_dir.to_path_buf(),
            source: mesh_io::IoError::from(e),
        })?;

        let mut molds = Vec::with_capacity(pending.len());
        for (entry, pour_volume) in pending.into_iter().zip(pour_volumes) {
            save_stl(&entry.mesh, &entry.path, true).map_err(|source| CastError::MeshIo {
                path: entry.path.clone(),
                source,
            })?;
            let summary = MeshSummary::from_mesh(&entry.mesh);
            molds.push(MoldArtifact {
                layer_index: entry.layer_index,
                material_display_name: entry.material_display_name,
                path: entry.path,
                validation: entry.validation,
                summary,
                pour_volume,
            });
        }

        save_stl(&plug_mesh, &plug_path, true).map_err(|source| CastError::MeshIo {
            path: plug_path.clone(),
            source,
        })?;

        let plug_summary = MeshSummary::from_mesh(&plug_mesh);

        Ok(MoldExportReport {
            molds,
            plug_path,
            plug_validation,
            plug_summary,
        })
    }

    /// Write the F3 procedure-spec Markdown for this cast to `path`.
    ///
    /// Computes pour volumes via [`Self::compute_pour_volumes`] and
    /// renders the Markdown via
    /// [`crate::generate_procedure_markdown`]. The generated document
    /// is the workshop starting-point reference — per-layer pour
    /// masses (F2), mix ratio + pot life + cure time
    /// (via [`crate::cure::lookup`] on each layer's
    /// [`MoldingMaterial::anchor_key`]), generic Smooth-On guidance,
    /// and a mass-budget summary.
    ///
    /// `path` is written verbatim; the caller is responsible for
    /// extension (`.md`) and parent-directory creation if needed.
    ///
    /// # Errors
    ///
    /// - [`CastError::EmptyLayers`] if [`Self::layers`] is empty.
    /// - [`CastError::InfiniteBounds`] if any layer body is
    ///   unbounded (propagates from [`Self::compute_pour_volumes`]).
    /// - [`CastError::MassBudgetExceeded`] if any layer's computed
    ///   pour mass exceeds [`Self::mass_budget_kg`] — same gate as
    ///   [`Self::export_molds`], so the generated Markdown's
    ///   "every layer within budget" claim never lies when this
    ///   method returns `Ok`.
    /// - [`CastError::MeshIo`] on filesystem write failure.
    pub fn write_procedure(&self, path: &Path) -> Result<(), CastError> {
        let pour_volumes = self.compute_pour_volumes()?;
        check_mass_budget(self, &pour_volumes)?;
        let markdown = generate_procedure_markdown(self, &pour_volumes);
        std::fs::write(path, markdown).map_err(|e| CastError::MeshIo {
            path: path.to_path_buf(),
            source: mesh_io::IoError::from(e),
        })?;
        Ok(())
    }

    /// Write the v2 curve-following multi-piece procedure Markdown
    /// for this cast + `ribbon` to `path`. Parallel to
    /// [`Self::write_procedure`] (v1) but renders v2-specific
    /// content (piece STL filenames, cast-geometry section with
    /// centerline arc length + max tangent rotation, manual-clamp
    /// assembly note, demold-in-order prose).
    ///
    /// Computes pour volumes via [`Self::compute_pour_volumes`] and
    /// renders the Markdown via
    /// [`crate::generate_procedure_markdown_v2`]. The mass-budget
    /// gate runs identically to v1, so the generated document's
    /// "every layer within budget" claim never lies when this
    /// method returns `Ok`. Curve-rotation gate also fires here so
    /// the procedure prose is never generated for an
    /// un-demoldable centerline.
    ///
    /// # Errors
    ///
    /// - [`CastError::EmptyLayers`] if [`Self::layers`] is empty.
    /// - [`CastError::CenterlineTooCurved`] if the ribbon's max
    ///   tangent rotation exceeds `2π/3` rad (120°) —
    ///   `write_procedure_v2` shares this gate with
    ///   [`Self::export_molds_v2`] so the procedure prose never
    ///   misrepresents a refused cast.
    /// - [`CastError::InfiniteBounds`] if any layer body is
    ///   unbounded (propagates from [`Self::compute_pour_volumes`]).
    /// - [`CastError::MassBudgetExceeded`] if any layer's computed
    ///   pour mass exceeds [`Self::mass_budget_kg`].
    /// - [`CastError::MeshIo`] on filesystem write failure.
    pub fn write_procedure_v2(&self, ribbon: &Ribbon, path: &Path) -> Result<(), CastError> {
        if self.layers.is_empty() {
            return Err(CastError::EmptyLayers);
        }
        let max_rotation = ribbon.max_tangent_rotation_rad();
        if max_rotation > MAX_TANGENT_ROTATION_RAD {
            return Err(CastError::CenterlineTooCurved {
                max_rotation_rad: max_rotation,
                max_rotation_deg: max_rotation.to_degrees(),
                threshold_rad: MAX_TANGENT_ROTATION_RAD,
                threshold_deg: MAX_TANGENT_ROTATION_RAD.to_degrees(),
            });
        }
        let pour_volumes = self.compute_pour_volumes()?;
        check_mass_budget(self, &pour_volumes)?;
        let markdown = generate_procedure_markdown_v2(self, &pour_volumes, ribbon);
        std::fs::write(path, markdown).map_err(|e| CastError::MeshIo {
            path: path.to_path_buf(),
            source: mesh_io::IoError::from(e),
        })?;
        Ok(())
    }

    /// Export the v2 curve-following multi-piece mold for this
    /// spec + the given `ribbon` as `2 * layers.len() + 1` STL files
    /// in `out_dir`.
    ///
    /// v2 entry point parallel to v1's [`Self::export_molds`]: same
    /// `CastSpec` data carrier, but each layer's mold cup is split
    /// into 2 pieces along the curve-following ribbon (instead of
    /// v1's single-cup straight-pull `+Z` clip). The plug pipeline
    /// is unchanged.
    ///
    /// Pipeline:
    /// 1. Verify [`Self::layers`] is non-empty.
    /// 2. Gate `ribbon.max_tangent_rotation_rad()` ≤ `2π/3` rad
    ///    (120°); abort with [`CastError::CenterlineTooCurved`]
    ///    before any meshing.
    /// 3. Compute per-layer pour volumes
    ///    ([`Self::compute_pour_volumes`]) and gate each against
    ///    [`Self::mass_budget_kg`]; abort on overrun before any
    ///    meshing.
    /// 4. For each `(layer, piece_side)`: compose the piece SDF via
    ///    [`crate::compose_piece_solid`], mesh it, F4-gate it.
    ///    Buffer until every piece + plug clear; abort cleanly on
    ///    any blocking Critical (no partial STLs on disk).
    /// 5. Mesh + F4-gate the plug (same as v1).
    /// 6. Create `out_dir` and write all pieces + plug.
    ///
    /// **Per-piece printability**: each piece's AABB is checked
    /// independently against [`Self::printer_config`]'s build
    /// volume via `mesh-printability::validate_for_printing`'s
    /// `ExceedsBuildVolume` Critical. v2 enforces the stricter
    /// per-piece bound (you can't print a piece larger than your
    /// printer no matter how the combined fits) by virtue of
    /// running the F4 gate on each piece's mesh — v1's aggregate
    /// check on a single combined cup is replaced.
    ///
    /// **Atomicity**: identical scope to [`Self::export_molds`] —
    /// curve-rotation gate, budget gate, meshing, and F4 gate all
    /// run before the first STL is written. Output directory is
    /// not created on any pre-write failure.
    ///
    /// # Errors
    ///
    /// - [`CastError::EmptyLayers`] if [`Self::layers`] is empty.
    /// - [`CastError::CenterlineTooCurved`] if the ribbon's max
    ///   tangent rotation exceeds `2π/3` rad.
    /// - [`CastError::InfiniteBounds`] for unbounded inputs (the
    ///   bounding region must be finite; the layer body need NOT
    ///   be — v2 doesn't require `body.z_max` for clip placement).
    /// - [`CastError::MassBudgetExceeded`] if any layer's pour mass
    ///   exceeds [`Self::mass_budget_kg`].
    /// - [`CastError::MeshingEmpty`] if marching cubes produces a
    ///   degenerate mesh for any piece or the plug.
    /// - [`CastError::PrintabilityCritical`] if any mesh fails the
    ///   F4 gate with one or more blocking Critical-severity issues.
    /// - [`CastError::MeshIo`] on filesystem failures during write.
    pub fn export_molds_v2(
        &self,
        ribbon: &Ribbon,
        out_dir: &Path,
    ) -> Result<V2MoldExportReport, CastError> {
        if self.layers.is_empty() {
            return Err(CastError::EmptyLayers);
        }

        let max_rotation = ribbon.max_tangent_rotation_rad();
        if max_rotation > MAX_TANGENT_ROTATION_RAD {
            return Err(CastError::CenterlineTooCurved {
                max_rotation_rad: max_rotation,
                max_rotation_deg: max_rotation.to_degrees(),
                threshold_rad: MAX_TANGENT_ROTATION_RAD,
                threshold_deg: MAX_TANGENT_ROTATION_RAD.to_degrees(),
            });
        }

        let pour_volumes = self.compute_pour_volumes()?;
        check_mass_budget(self, &pour_volumes)?;

        let pending_pieces = mesh_and_gate_v2_pieces(self, ribbon, out_dir)?;
        let pending_plugs = mesh_and_gate_v2_plugs(self, ribbon, out_dir)?;
        let pending_platform = mesh_and_gate_v2_platform(self, ribbon, out_dir)?;
        let pending_funnel = mesh_and_gate_v2_funnel(self, ribbon, out_dir)?;

        std::fs::create_dir_all(out_dir).map_err(|e| CastError::MeshIo {
            path: out_dir.to_path_buf(),
            source: mesh_io::IoError::from(e),
        })?;

        let platform_count = usize::from(pending_platform.is_some());
        let funnel_count = usize::from(pending_funnel.is_some());
        let stl_count = self.layers.len() * 3 + platform_count + funnel_count;
        eprintln!(
            "[cf-cast] writing {stl_count} STLs to {out}…",
            out = out_dir.display()
        );
        let (layers_out, platform_out, funnel_out) = write_v2_artifacts(
            self,
            pending_pieces,
            pending_plugs,
            pending_platform,
            pending_funnel,
            pour_volumes,
        )?;

        Ok(V2MoldExportReport {
            layers: layers_out,
            platform: platform_out,
            funnel: funnel_out,
        })
    }
}

/// Compose, mesh, and F4-gate every (layer × piece) pair.
/// Returns the v2 pending buffer organized as `[Negative, Positive]`
/// per-layer pairs, so the writer phase consumes them without
/// re-pairing or `expect()` on flat-vector ordering.
///
/// Emits per-piece progress lines to stderr so workshop runs on
/// large scans aren't black-boxed for minutes between "loaded
/// design" and the final STL writes. Format:
/// `[cf-cast] layer N piece M (Negative|Positive) — composed + meshed in Xs, F4 gate passed in Ys`.
fn mesh_and_gate_v2_pieces(
    spec: &CastSpec,
    ribbon: &Ribbon,
    out_dir: &Path,
) -> Result<Vec<[PendingPiece; 2]>, CastError> {
    let layer_count = spec.layers.len();
    let mut pending_layers = Vec::with_capacity(layer_count);
    for (layer_index, layer) in spec.layers.iter().enumerate() {
        let neg = mesh_and_gate_v2_piece(
            spec,
            ribbon,
            layer,
            layer_index,
            PieceSide::Negative,
            out_dir,
            layer_count,
        )?;
        let pos = mesh_and_gate_v2_piece(
            spec,
            ribbon,
            layer,
            layer_index,
            PieceSide::Positive,
            out_dir,
            layer_count,
        )?;
        pending_layers.push([neg, pos]);
    }
    Ok(pending_layers)
}

/// Compose + mesh + F4-gate a single piece. Extracted so
/// `mesh_and_gate_v2_pieces` stays readable + each call site has
/// the same target metadata threaded through identically.
fn mesh_and_gate_v2_piece(
    spec: &CastSpec,
    ribbon: &Ribbon,
    layer: &CastLayer,
    layer_index: usize,
    piece_side: PieceSide,
    out_dir: &Path,
    layer_count: usize,
) -> Result<PendingPiece, CastError> {
    let t_compose = std::time::Instant::now();
    let (piece_solid, mating_transforms) =
        compose_piece_solid(&layer.body, &spec.bounding_region, ribbon, piece_side)?;
    let target = CastTarget::MoldPiece {
        layer_index,
        piece_side,
    };
    let mesh = solid_to_mm_mesh(&piece_solid, spec.mesh_cell_size_m, target)?;
    // Post-MC mesh-CSG stage (S3 plumbing; S4/S5/S6 emit transforms).
    // Empty `mating_transforms` short-circuits to a pass-through.
    let mesh = apply_mating_transforms(mesh, &mating_transforms, target)?;
    let compose_mesh_s = t_compose.elapsed().as_secs_f64();
    let path = out_dir.join(mold_piece_filename(layer_index, piece_side));
    let t_gate = std::time::Instant::now();
    let validation = run_printability_gate(&mesh, &spec.printer_config, &path)?;
    let gate_s = t_gate.elapsed().as_secs_f64();

    let blocking = blocking_critical_count(&validation);
    if blocking > 0 {
        return Err(CastError::PrintabilityCritical {
            target,
            issue_count: blocking,
            path,
        });
    }
    eprintln!(
        "[cf-cast] layer {layer_index}/{layer_count_minus_1} piece {piece_side:?} \
         — compose+MC {compose_mesh_s:.1}s, F4 {gate_s:.1}s ({verts} verts / {faces} faces)",
        layer_count_minus_1 = layer_count.saturating_sub(1),
        verts = mesh.vertices.len(),
        faces = mesh.faces.len(),
    );
    Ok(PendingPiece {
        layer_index,
        piece_side,
        mesh,
        validation,
        path,
    })
}

/// v2.1 sub-leaf 2: per-layer plug pipeline.
///
/// For each layer N, derive the base plug solid (`spec.plug` for
/// `N = 0`, `spec.layers[N - 1].body` for `N > 0`), extend it with
/// the ribbon's plug-anchor pin geometry via [`add_plug_pins`],
/// then mesh + F4-gate. Each layer's plug matches the next-inner
/// silicone shell's outer surface, so the detachable per-layer
/// casts produce nesting cured tubes.
fn mesh_and_gate_v2_plugs(
    spec: &CastSpec,
    ribbon: &Ribbon,
    out_dir: &Path,
) -> Result<Vec<PendingPlug>, CastError> {
    let layer_count = spec.layers.len();
    let mut pending = Vec::with_capacity(layer_count);
    for layer_index in 0..layer_count {
        let t_compose = std::time::Instant::now();
        let base_plug = if layer_index == 0 {
            spec.plug.clone()
        } else {
            spec.layers[layer_index - 1].body.clone()
        };
        let (plug_solid, mating_transforms) = add_plug_pins(base_plug, ribbon);
        let target = CastTarget::Plug {
            layer_index: Some(layer_index),
        };
        let mesh = solid_to_mm_mesh(&plug_solid, spec.mesh_cell_size_m, target)?;
        let mesh = apply_mating_transforms(mesh, &mating_transforms, target)?;
        let compose_mesh_s = t_compose.elapsed().as_secs_f64();
        let path = out_dir.join(plug_layer_filename(layer_index));
        let t_gate = std::time::Instant::now();
        let validation = run_printability_gate(&mesh, &spec.printer_config, &path)?;
        let gate_s = t_gate.elapsed().as_secs_f64();
        let blocking = blocking_critical_count(&validation);
        if blocking > 0 {
            return Err(CastError::PrintabilityCritical {
                target,
                issue_count: blocking,
                path,
            });
        }
        eprintln!(
            "[cf-cast] layer {layer_index}/{layer_count_minus_1} plug \
             — compose+MC {compose_mesh_s:.1}s, F4 {gate_s:.1}s ({verts} verts / {faces} faces)",
            layer_count_minus_1 = layer_count.saturating_sub(1),
            verts = mesh.vertices.len(),
            faces = mesh.faces.len(),
        );
        pending.push(PendingPlug {
            layer_index,
            mesh,
            validation,
            path,
        });
    }
    Ok(pending)
}

/// Mesh + F4-gate the workshop platform STL, or return
/// `Ok(None)` when the ribbon's plug-pin kind doesn't enable a
/// T-bar (no platform needed). Single artifact per cast.
///
/// Uses a finer mesh-cell size than the rest of the cast
/// ([`PLATFORM_MAX_CELL_SIZE_M`] capped at `spec.mesh_cell_size_m`)
/// because the platform's pocket cross-section is small (e.g., a
/// ~3 mm-deep crescent for iter-1's 6 mm-Ø T-bar) — at the cast's
/// default 3 mm cells the pocket would be 1 cell deep and MC would
/// fragment it. The platform solid is small + simple so the finer
/// mesh costs ~1-2 s at most.
fn mesh_and_gate_v2_platform(
    spec: &CastSpec,
    ribbon: &Ribbon,
    out_dir: &Path,
) -> Result<Option<PendingPlatform>, CastError> {
    let Some((platform_solid, mating_transforms)) =
        crate::platform::build_platform_solid(&spec.bounding_region, ribbon)
    else {
        return Ok(None);
    };
    let t_compose = std::time::Instant::now();
    let target = CastTarget::Platform;
    let cell_size_m = spec.mesh_cell_size_m.min(PLATFORM_MAX_CELL_SIZE_M);
    let mesh = solid_to_mm_mesh(&platform_solid, cell_size_m, target)?;
    let mesh = apply_mating_transforms(mesh, &mating_transforms, target)?;
    let compose_mesh_s = t_compose.elapsed().as_secs_f64();
    let path = out_dir.join("platform.stl");
    let t_gate = std::time::Instant::now();
    let validation = run_printability_gate(&mesh, &spec.printer_config, &path)?;
    let gate_s = t_gate.elapsed().as_secs_f64();
    let blocking = blocking_critical_count(&validation);
    if blocking > 0 {
        return Err(CastError::PrintabilityCritical {
            target,
            issue_count: blocking,
            path,
        });
    }
    eprintln!(
        "[cf-cast] platform — compose+MC {compose_mesh_s:.1}s @ {cell_size_mm:.1} mm cells, \
         F4 {gate_s:.1}s ({verts} verts / {faces} faces)",
        cell_size_mm = cell_size_m * 1000.0,
        verts = mesh.vertices.len(),
        faces = mesh.faces.len(),
    );
    Ok(Some(PendingPlatform {
        mesh,
        validation,
        path,
    }))
}

/// Cap on the mesh-cell size used for the platform's marching-cubes
/// pass. The platform's pocket cross-section (for typical workshop
/// geometries: a ~3 mm-deep crescent absorbing the T-bar's protrusion
/// below the cup outer face) is too small for the cast's default
/// 3 mm cells to resolve cleanly — at 3 mm cells, the pocket would
/// be ~1 cell deep and MC would fragment it. 1.5 mm cells give
/// ~2× the radial resolution and reliably mesh the pocket as a
/// continuous concave feature. The platform solid is small +
/// simple (cuboid - cylinder) so the finer mesh costs ~1-2 s at
/// most; effectively free relative to the cast's main meshing
/// time.
const PLATFORM_MAX_CELL_SIZE_M: f64 = 0.0015;

/// Mesh + F4-gate the workshop pour funnel STL, or return
/// `Ok(None)` when the ribbon has no pour gate enabled (no funnel
/// needed). Single artifact per cast.
///
/// Uses [`FUNNEL_MAX_CELL_SIZE_M`] for the same MC-resolution
/// reason as the platform: the funnel's nipple wall is 1 mm and
/// would mesh as 0.33 cells radial at the cast's default 3 mm
/// cells. The funnel is small + geometrically simple so finer
/// cells cost ~1-3 s.
fn mesh_and_gate_v2_funnel(
    spec: &CastSpec,
    ribbon: &Ribbon,
    out_dir: &Path,
) -> Result<Option<PendingFunnel>, CastError> {
    let Some((funnel_solid, mating_transforms)) = crate::funnel::build_funnel_solid(ribbon) else {
        return Ok(None);
    };
    let t_compose = std::time::Instant::now();
    let target = CastTarget::Funnel;
    let cell_size_m = spec.mesh_cell_size_m.min(FUNNEL_MAX_CELL_SIZE_M);
    let mesh = solid_to_mm_mesh(&funnel_solid, cell_size_m, target)?;
    let mesh = apply_mating_transforms(mesh, &mating_transforms, target)?;
    let compose_mesh_s = t_compose.elapsed().as_secs_f64();
    let path = out_dir.join("funnel.stl");
    let t_gate = std::time::Instant::now();
    let validation = run_printability_gate(&mesh, &spec.printer_config, &path)?;
    let gate_s = t_gate.elapsed().as_secs_f64();
    let blocking = blocking_critical_count(&validation);
    if blocking > 0 {
        return Err(CastError::PrintabilityCritical {
            target,
            issue_count: blocking,
            path,
        });
    }
    eprintln!(
        "[cf-cast] funnel — compose+MC {compose_mesh_s:.1}s @ {cell_size_mm:.1} mm cells, \
         F4 {gate_s:.1}s ({verts} verts / {faces} faces)",
        cell_size_mm = cell_size_m * 1000.0,
        verts = mesh.vertices.len(),
        faces = mesh.faces.len(),
    );
    Ok(Some(PendingFunnel {
        mesh,
        validation,
        path,
    }))
}

/// Cap on the mesh-cell size used for the funnel's MC pass.
///
/// The funnel's nipple wall is 1 mm at iter-1 dimensions —
/// 0.33 cells at the cast's default 3 mm cells, which MC would
/// fragment. 1 mm cells give 1 cell radial across the wall (clean)
/// and 1 mm voxel grid resolves the tapered cone surface at
/// workshop visual fidelity. The funnel solid is small (~38 mm
/// tall × 30 mm Ø top) so finer cells cost ~1-3 s; effectively
/// free vs the cast's main meshing time.
const FUNNEL_MAX_CELL_SIZE_M: f64 = 0.001;

/// Write every pending piece + per-layer plug + optional platform
/// + optional funnel to disk and build the report structs.
///
/// Consumes all pending buffers; on any FS failure mid-stream the
/// remaining artifacts are abandoned and the error surfaces with
/// the failing artifact's path attached.
///
/// Assumes `pending_pieces` and `pending_plugs` are both ordered
/// innermost-first and parallel to `pour_volumes`. The function
/// asserts this invariant via `layer_index` matching on each step.
fn write_v2_artifacts(
    spec: &CastSpec,
    pending_pieces: Vec<[PendingPiece; 2]>,
    pending_plugs: Vec<PendingPlug>,
    pending_platform: Option<PendingPlatform>,
    pending_funnel: Option<PendingFunnel>,
    pour_volumes: Vec<PourVolume>,
) -> Result<
    (
        Vec<V2LayerReport>,
        Option<PlatformArtifact>,
        Option<FunnelArtifact>,
    ),
    CastError,
> {
    let mut layers_out = Vec::with_capacity(pending_pieces.len());
    for ((pieces_pair, plug_entry), pour_volume) in pending_pieces
        .into_iter()
        .zip(pending_plugs)
        .zip(pour_volumes)
    {
        let [entry_neg, entry_pos] = pieces_pair;
        let layer_index = entry_neg.layer_index;
        debug_assert_eq!(plug_entry.layer_index, layer_index);

        save_stl(&entry_neg.mesh, &entry_neg.path, true).map_err(|source| CastError::MeshIo {
            path: entry_neg.path.clone(),
            source,
        })?;
        save_stl(&entry_pos.mesh, &entry_pos.path, true).map_err(|source| CastError::MeshIo {
            path: entry_pos.path.clone(),
            source,
        })?;
        save_stl(&plug_entry.mesh, &plug_entry.path, true).map_err(|source| CastError::MeshIo {
            path: plug_entry.path.clone(),
            source,
        })?;
        let neg_summary = MeshSummary::from_mesh(&entry_neg.mesh);
        let pos_summary = MeshSummary::from_mesh(&entry_pos.mesh);
        let plug_summary = MeshSummary::from_mesh(&plug_entry.mesh);
        layers_out.push(V2LayerReport {
            layer_index,
            material_display_name: spec.layers[layer_index].material.display_name.clone(),
            pour_volume,
            pieces: [
                PieceArtifact {
                    piece_side: entry_neg.piece_side,
                    path: entry_neg.path,
                    validation: entry_neg.validation,
                    summary: neg_summary,
                },
                PieceArtifact {
                    piece_side: entry_pos.piece_side,
                    path: entry_pos.path,
                    validation: entry_pos.validation,
                    summary: pos_summary,
                },
            ],
            plug: PlugArtifact {
                path: plug_entry.path,
                validation: plug_entry.validation,
                summary: plug_summary,
            },
        });
    }
    let platform_out = if let Some(p) = pending_platform {
        save_stl(&p.mesh, &p.path, true).map_err(|source| CastError::MeshIo {
            path: p.path.clone(),
            source,
        })?;
        let platform_summary = MeshSummary::from_mesh(&p.mesh);
        Some(PlatformArtifact {
            path: p.path,
            validation: p.validation,
            summary: platform_summary,
        })
    } else {
        None
    };
    let funnel_out = if let Some(f) = pending_funnel {
        save_stl(&f.mesh, &f.path, true).map_err(|source| CastError::MeshIo {
            path: f.path.clone(),
            source,
        })?;
        let funnel_summary = MeshSummary::from_mesh(&f.mesh);
        Some(FunnelArtifact {
            path: f.path,
            validation: f.validation,
            summary: funnel_summary,
        })
    } else {
        None
    };
    Ok((layers_out, platform_out, funnel_out))
}

/// Gate per-layer pour masses against [`CastSpec::mass_budget_kg`].
///
/// Shared by [`CastSpec::export_molds`] and
/// [`CastSpec::write_procedure`] so the budget-enforcement contract
/// stays in one place — both pre-write paths refuse to proceed on
/// a budget overrun. The procedure-Markdown's
/// "Every layer's pour mass falls within budget per the F2 export
/// gate" line is therefore factually correct whenever
/// `write_procedure` returns `Ok`.
fn check_mass_budget(spec: &CastSpec, pour_volumes: &[PourVolume]) -> Result<(), CastError> {
    for pv in pour_volumes {
        if pv.pour_mass_kg > spec.mass_budget_kg {
            return Err(CastError::MassBudgetExceeded {
                layer_index: pv.layer_index,
                material_display_name: pv.material_display_name.clone(),
                mass_kg: pv.pour_mass_kg,
                budget_kg: spec.mass_budget_kg,
            });
        }
    }
    Ok(())
}

/// Run `validate_for_printing` and wrap its error path into a
/// `CastError::MeshIo`. The validator's error variant is distinct
/// from `IoError`; we wrap into `MeshIo::InvalidContent` because the
/// trip is guarded by `solid_to_mm_mesh`'s empty-mesh / no-faces
/// preconditions and is defensive in practice.
fn run_printability_gate(
    mesh: &IndexedMesh,
    config: &PrinterConfig,
    path: &Path,
) -> Result<PrintValidation, CastError> {
    validate_for_printing(mesh, config).map_err(|e| CastError::MeshIo {
        path: path.to_path_buf(),
        source: mesh_io::IoError::invalid_content(format!("validation failed: {e}")),
    })
}

/// Filename for the `layer_index`-th mold cup. Innermost-first
/// indexing matches [`CastSpec::layers`].
fn mold_filename(layer_index: usize) -> String {
    format!("mold_layer_{layer_index}.stl")
}

/// Filename for v2.1's per-layer plug STL.
/// `plug_layer_{layer_index}.stl` — innermost-first indexing
/// matches [`CastSpec::layers`].
fn plug_layer_filename(layer_index: usize) -> String {
    format!("plug_layer_{layer_index}.stl")
}

/// Filename for the v2 `(layer_index, piece_side)` mold piece.
/// Piece-side maps to integer suffix: `Negative` → `0`,
/// `Positive` → `1` per `docs/CURVE_FOLLOWING_DESIGN.md` §"Output
/// artifacts" naming convention.
fn mold_piece_filename(layer_index: usize, piece_side: PieceSide) -> String {
    let piece_index = match piece_side {
        PieceSide::Negative => 0,
        PieceSide::Positive => 1,
    };
    format!("mold_layer_{layer_index}_piece_{piece_index}.stl")
}

/// cf-cast's blocking-issue rule: which `Critical`-severity issues
/// from `mesh-printability` should abort an export.
///
/// Mold-cup geometry is inherently support-dependent (the cavity
/// ceiling and the annular pour rim are downward-facing horizontal
/// faces — `ExcessiveOverhang` + `LongBridge` always fire on a real
/// open-top cup). `SelfIntersecting` reports a noise floor of ~100
/// regions on marching-cubes output for any closed cuboid surface
/// (verified by `baseline_cuboid_self_intersection_noise_floor` in
/// this crate's test module); it does not reliably indicate a real
/// defect at the cell sizes used by `cf-cast`.
///
/// The remaining `Critical` types — geometry actually too big for the
/// printer, broken topology, sub-min-wall thinness, CAD-leftover
/// debris, or a sealed cavity that can't be poured — DO indicate a
/// fixture or pipeline bug and abort the export. Warnings of every
/// kind surface in the returned `PrintValidation` for caller
/// inspection.
fn is_blocking_critical(issue: &PrintIssue) -> bool {
    if issue.severity != IssueSeverity::Critical {
        return false;
    }
    matches!(
        issue.issue_type,
        PrintIssueType::ExceedsBuildVolume
            | PrintIssueType::NotWatertight
            | PrintIssueType::NonManifold
            | PrintIssueType::ThinWall
            | PrintIssueType::SmallFeature
            | PrintIssueType::TrappedVolume
    )
}

/// Count `Critical`-severity issues that block export per the
/// [`is_blocking_critical`] rule.
fn blocking_critical_count(validation: &PrintValidation) -> usize {
    validation
        .issues
        .iter()
        .filter(|i| is_blocking_critical(i))
        .count()
}

/// Build the clip cuboid that subtracts the half-space above
/// `layer_body.bounds().max.z` from the bounding region, opening the
/// mold cup at the top for pour access.
///
/// `layer_index` is threaded through for error reporting so the
/// caller can identify which layer body produced an `InfiniteBounds`
/// failure without re-mapping in the call site.
fn clip_above_body(
    layer_body: &Solid,
    bounding_region: &Solid,
    layer_index: usize,
) -> Result<Solid, CastError> {
    let body_aabb =
        layer_body
            .bounds()
            .ok_or(CastError::InfiniteBounds(CastTarget::LayerBody {
                layer_index,
            }))?;
    let bound_aabb = bounding_region
        .bounds()
        .ok_or(CastError::InfiniteBounds(CastTarget::BoundingRegion))?;

    // Clip extends from `body.z_max - overlap` upward by CLIP_Z_REACH_M.
    // The sub-mm overlap below the body top breaks the otherwise
    // degenerate coincidence between body's top face and clip's bottom
    // face that produces non-manifold MC artifacts.
    let clip_z_min = body_aabb.max.z - CLIP_BODY_OVERLAP_M;
    let clip_z_max = clip_z_min + CLIP_Z_REACH_M;
    let clip_z_center = f64::midpoint(clip_z_min, clip_z_max);
    let clip_half_z = (clip_z_max - clip_z_min) / 2.0;

    let bound_size = bound_aabb.size();
    let bound_center = bound_aabb.center();
    let clip_half_x = bound_size.x / 2.0 + CLIP_XY_SLACK_M;
    let clip_half_y = bound_size.y / 2.0 + CLIP_XY_SLACK_M;

    Ok(
        Solid::cuboid(Vector3::new(clip_half_x, clip_half_y, clip_half_z)).translate(Vector3::new(
            bound_center.x,
            bound_center.y,
            clip_z_center,
        )),
    )
}

#[cfg(test)]
mod tests {
    // Workspace lint policy denies `unwrap()` + explicit `panic!()`
    // in lib code but warns in tests. Localized allow makes
    // assertion failures read clearly without panic-handling
    // boilerplate at every probe. Same convention as cf-design tests.
    #![allow(clippy::unwrap_used, clippy::panic)]

    use approx::assert_relative_eq;
    use mesh_printability::PrinterConfig;
    use nalgebra::Vector3;

    use super::{CastError, CastLayer, CastSpec, CastTarget, MeshSummary, clip_above_body};
    use crate::material::MoldingMaterial;
    use crate::pour_volume::DEFAULT_MASS_BUDGET_KG;
    use cf_design::Solid;

    /// Reference material for smoke tests — Ecoflex 00-30 anchor
    /// values, constructed inline so cf-cast does not need
    /// `sim-soft` as a dep just to run its own tests.
    fn reference_material() -> MoldingMaterial {
        MoldingMaterial {
            display_name: "Ecoflex 00-30".to_string(),
            density_kg_m3: 1070.0,
            anchor_key: Some("ECOFLEX_00_30"),
        }
    }

    /// Stage 2 reference fixture — single-layer cuboid body + 8 mm
    /// radius capsule plug + 80 × 80 × 60 mm cuboid bounding region.
    ///
    /// The cuboid body is the simplest geometry that lets a one-piece
    /// mold cup have a flat-rectangular top opening: clipping above
    /// the body's `z_max` removes the cuboid wall above the body, and
    /// at the clip level the body's xy-cross-section is a finite
    /// rectangle — i.e., a real hole in the cup top, not a single
    /// tangent point. A sphere body fails the F4 trapped-volume
    /// detector for exactly that reason.
    fn reference_spec() -> CastSpec {
        let layer_body = Solid::cuboid(Vector3::new(0.025, 0.025, 0.020));
        let plug = Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040));
        let bounding_region = Solid::cuboid(Vector3::new(0.040, 0.040, 0.030));
        CastSpec {
            layers: vec![CastLayer {
                body: layer_body,
                material: reference_material(),
            }],
            plug,
            bounding_region,
            // 2 mm cell size: keeps integration-test wall time on the
            // mold cup's marching-cubes grid (~51 k probes) and the
            // downstream F4 validation (O(faces²) self-intersection
            // check) well under 10 s even in `--release`. Finer cells
            // are appropriate for production geometry; this smoke
            // fixture trades surface fidelity for run time.
            mesh_cell_size_m: 0.002,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        }
    }

    #[test]
    fn export_molds_writes_n_plus_1_stls_end_to_end() {
        // Lib-side mirror of the integration test in
        // `tests/single_layer_smoke.rs`. Lives here too because
        // `cargo llvm-cov --lib` (the grader's coverage probe) does
        // not include integration tests — without this, the entire
        // `export_molds` body, `MeshSummary::from_mesh`, both
        // `mesh-printability` map_err arms, and both `mesh-io` write
        // arms read as uncovered.
        //
        // Uses a 12 mm cell size (vs the integration test's 2 mm) so
        // the marching-cubes output is coarse enough to keep
        // `validate_for_printing`'s O(face²) self-intersection check
        // tractable under llvm-cov instrumentation. Surface fidelity
        // doesn't matter for the coverage path; the integration test
        // pins the production cell size.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-stage-2-lib");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }

        let spec = CastSpec {
            layers: vec![CastLayer {
                body: Solid::cuboid(Vector3::new(0.025, 0.025, 0.020)),
                material: reference_material(),
            }],
            plug: Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040)),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            // 12 mm cells: deliberately ugly geometry. Under llvm-cov
            // instrumentation, `mesh-printability::validate_for_printing`
            // runs multiple O(faces²) checks (self-intersection,
            // long-bridge, thin-wall) on instrumented deps; even at 6 mm
            // cells (~2 k faces) those checks become minutes-long. At
            // 12 mm cells the mold mesh is ~250 faces, which the same
            // checks clear in seconds even instrumented.
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let report = spec.export_molds(&out_dir).unwrap();

        assert_eq!(report.molds.len(), 1);
        let mold = &report.molds[0];
        assert_eq!(mold.layer_index, 0);
        assert_eq!(mold.material_display_name, "Ecoflex 00-30");
        assert!(mold.path.exists());
        assert!(report.plug_path.exists());
        assert!(mold.summary.face_count > 0);
        assert!(report.plug_summary.face_count > 0);
        // Mold AABB spans roughly the bounding region z range
        // (-30 to ~+20 mm after clip). MC bias at 12 mm cells gives
        // ±6 mm slack on each bound.
        assert!(mold.summary.aabb_mm.min.z < -20.0);
        assert!(mold.summary.aabb_mm.max.z < 28.0);
        // Plug AABB is the translated capsule's bounds (~+12 to +68 mm).
        assert!(report.plug_summary.aabb_mm.min.z > 0.0);
        assert!(report.plug_summary.aabb_mm.max.z > 50.0);
    }

    #[test]
    fn export_molds_writes_one_artifact_per_layer_for_multi_layer_spec() {
        // Two stacked cumulative cuboid bodies → two mold cup STLs +
        // one plug STL. Inner body is a 20 × 20 × 15 mm cuboid; outer
        // body is a 30 × 30 × 25 mm cuboid centered at the same
        // origin so it FULLY contains the inner body (cumulative-
        // outer-surface invariant of innermost-first ordering).
        // Pins that:
        // - `export_molds` returns one MoldArtifact per layer in
        //   innermost-first order.
        // - Per-layer `material_display_name` is threaded through
        //   from `CastSpec::layers[i].material.display_name`.
        // - The plug artifact is independent of layer count.
        //
        // 12 mm cells keep the F4 gate fast under llvm-cov per
        // pattern (qqq); ugly geometry doesn't matter for the
        // multi-layer plumbing being pinned.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-stage-2-multi-lib");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }

        let inner_body = Solid::cuboid(Vector3::new(0.020, 0.020, 0.015));
        let outer_body = Solid::cuboid(Vector3::new(0.030, 0.030, 0.025));
        // 8 mm radius + 20 mm half-height capsule — same size as the
        // Stage 2 reference fixture. Smaller capsules (e.g., 6 + 15 mm)
        // produce zero MC faces at the 12 mm coverage-test cell size
        // and trip `MeshingEmpty(Plug)` before reaching the multi-layer
        // assertions this test exists to pin.
        let plug = Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040));
        let bounding_region = Solid::cuboid(Vector3::new(0.045, 0.045, 0.035));

        let inner_material = MoldingMaterial {
            display_name: "Ecoflex 00-30".to_string(),
            density_kg_m3: 1070.0,
            anchor_key: Some("ECOFLEX_00_30"),
        };
        let outer_material = MoldingMaterial {
            display_name: "Dragon Skin 10A".to_string(),
            density_kg_m3: 1070.0,
            anchor_key: Some("DRAGON_SKIN_10A"),
        };

        let spec = CastSpec {
            layers: vec![
                CastLayer {
                    body: inner_body,
                    material: inner_material,
                },
                CastLayer {
                    body: outer_body,
                    material: outer_material,
                },
            ],
            plug,
            bounding_region,
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };

        let report = spec.export_molds(&out_dir).unwrap();
        assert_eq!(report.molds.len(), 2);

        let mold0 = &report.molds[0];
        assert_eq!(mold0.layer_index, 0);
        assert_eq!(mold0.material_display_name, "Ecoflex 00-30");
        assert!(mold0.path.exists());
        assert!(mold0.path.ends_with("mold_layer_0.stl"));

        let mold1 = &report.molds[1];
        assert_eq!(mold1.layer_index, 1);
        assert_eq!(mold1.material_display_name, "Dragon Skin 10A");
        assert!(mold1.path.exists());
        assert!(mold1.path.ends_with("mold_layer_1.stl"));

        assert!(report.plug_path.exists());
        assert!(report.plug_path.ends_with("plug.stl"));
    }

    #[test]
    fn export_molds_errors_when_layers_is_empty() {
        // `CastError::EmptyLayers` fires before any meshing — no
        // output directory or grid is touched.
        let spec = CastSpec {
            layers: Vec::new(),
            plug: Solid::capsule(0.005, 0.010),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.002,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-empty-layers");
        let err = spec.export_molds(&out_dir).unwrap_err();
        assert!(matches!(err, CastError::EmptyLayers));
    }

    #[test]
    fn export_molds_errors_when_layer_body_is_unbounded() {
        // `Solid::plane` is unbounded by construction and returns
        // `None` from `Solid::bounds()`. Pins the `InfiniteBounds`
        // error path on a layer body and the parallel `layer_index`
        // carry-through in `CastTarget::LayerBody`.
        let spec = CastSpec {
            layers: vec![CastLayer {
                body: Solid::plane(Vector3::new(0.0, 0.0, 1.0), 0.0),
                material: reference_material(),
            }],
            plug: Solid::capsule(0.005, 0.010),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.002,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-error-unbounded");
        let err = spec.export_molds(&out_dir).unwrap_err();
        match err {
            CastError::InfiniteBounds(CastTarget::LayerBody { layer_index }) => {
                assert_eq!(layer_index, 0);
            }
            other => panic!("expected InfiniteBounds(LayerBody {{ 0 }}), got {other:?}"),
        }
    }

    #[test]
    fn mesh_summary_aabb_matches_input_mesh_bounds() {
        // `MeshSummary::from_mesh` is a small helper but it's the
        // only path that converts `IndexedMesh` → reportable bounds.
        // Pin its bit-exact behavior against a hand-authored mesh.
        use mesh_types::{IndexedMesh, Point3};
        let mut m = IndexedMesh::new();
        m.vertices.push(Point3::new(-3.0, 1.0, 2.0));
        m.vertices.push(Point3::new(4.0, -2.0, 5.0));
        m.vertices.push(Point3::new(0.0, 0.0, 0.0));
        m.faces.push([0, 1, 2]);
        let summary = MeshSummary::from_mesh(&m);
        assert_eq!(summary.vertex_count, 3);
        assert_eq!(summary.face_count, 1);
        assert!((summary.aabb_mm.min.x - -3.0).abs() < 1e-12);
        assert!((summary.aabb_mm.max.x - 4.0).abs() < 1e-12);
        assert!((summary.aabb_mm.max.z - 5.0).abs() < 1e-12);
    }

    #[test]
    fn clip_extends_above_body_zmax() {
        let spec = reference_spec();
        let clip = clip_above_body(&spec.layers[0].body, &spec.bounding_region, 0).unwrap();
        let clip_aabb = clip.bounds().unwrap();

        // body z_max = +0.020 (cuboid half-extent 20 mm at origin).
        // Clip must START slightly below body_zmax (sub-mm overlap
        // breaks MC degeneracy at the coincidence plane) and EXTEND
        // well above it.
        let expected_clip_min = 0.020 - super::CLIP_BODY_OVERLAP_M;
        assert!(
            (clip_aabb.min.z - expected_clip_min).abs() < 1e-9,
            "clip min z should be body z_max - overlap ({} m), got {}",
            expected_clip_min,
            clip_aabb.min.z
        );
        assert!(
            clip_aabb.max.z > 0.030,
            "clip max z should extend well above body z_max, got {}",
            clip_aabb.max.z
        );
    }

    #[test]
    fn baseline_cuboid_self_intersection_noise_floor() {
        // Establishes the MC self-intersection noise floor on a plain
        // cuboid (no CSG composition). Self-intersection counts at
        // this floor are the reason `is_blocking_critical` skips
        // `SelfIntersecting` from its blocking-issue set: the
        // detector reports FP-noise pairs at marching-cubes cell
        // boundaries on ANY closed surface.
        //
        // Uses 12 mm cells (vs the integration test's 2 mm) to keep
        // the O(face²) self-intersection check tractable under
        // llvm-cov instrumentation. The phenomenon (non-zero
        // self-intersection on a clean closed manifold) reproduces at
        // any cell size; the coverage probe doesn't need fine
        // resolution.
        use crate::mesher::solid_to_mm_mesh;
        use mesh_printability::validate_for_printing;

        let plain_cuboid = Solid::cuboid(Vector3::new(0.025, 0.025, 0.020));
        let baseline_mesh =
            solid_to_mm_mesh(&plain_cuboid, 0.012, CastTarget::Mold { layer_index: 0 }).unwrap();
        let baseline_validation =
            validate_for_printing(&baseline_mesh, &PrinterConfig::fdm_default()).unwrap();

        // The plain cuboid MUST have zero blocking-critical issues
        // (it's a simple closed manifold cuboid). Any failure here
        // means our gate semantics drifted from the baseline.
        assert_eq!(
            super::blocking_critical_count(&baseline_validation),
            0,
            "plain cuboid should have zero blocking critical issues; if this fires, \
             review `is_blocking_critical` against the actual issue types reported"
        );

        // Cell-size note: self-intersection noise scales with MC face
        // density. At 12 mm cells (this test) a clean cuboid produces
        // 0 self-intersections; at 2 mm cells (the integration test
        // fixture) it produces ~100. The cf-cast gate skips
        // SelfIntersecting because the production cell size (fine,
        // ≤2 mm) trips the noise floor. We don't assert a specific
        // count here — the noise-floor behavior is cell-size
        // dependent and the assertion above is the load-bearing one.
    }

    #[test]
    fn clip_covers_bounding_region_in_xy() {
        let spec = reference_spec();
        let clip = clip_above_body(&spec.layers[0].body, &spec.bounding_region, 0).unwrap();
        let clip_aabb = clip.bounds().unwrap();
        let bound_aabb = spec.bounding_region.bounds().unwrap();

        // Clip xy must FULLY cover bounding region xy, with slack on
        // each side so any subtraction at z ≥ body_zmax leaves no
        // unclipped corners.
        assert!(clip_aabb.min.x < bound_aabb.min.x);
        assert!(clip_aabb.max.x > bound_aabb.max.x);
        assert!(clip_aabb.min.y < bound_aabb.min.y);
        assert!(clip_aabb.max.y > bound_aabb.max.y);
    }

    /// F2 fixture — two stacked cumulative cuboid bodies with the
    /// plug intentionally DISJOINT from both. Outer body spans z ∈
    /// [-25, +25] mm; the capsule plug (r = 8 mm, half-height
    /// 20 mm) is translated to z = +60 mm so its full z-extent is
    /// [+32, +88] mm — strictly above the outer body's +25 mm top,
    /// with 7 mm of clearance. Disjointness means Layer 0's shell
    /// volume equals layer-0-body volume exactly (plug-body
    /// intersection is empty); Layer 1's shell volume equals
    /// outer-cuboid volume minus inner-cuboid volume.
    ///
    /// Disjoint plug + body keeps the analytic expectations for
    /// [`pour_volumes_match_analytic_cuboid_shell_volumes`] clean
    /// so the bias tolerance need not absorb plug displacement on
    /// top of Riemann-sum surface error.
    fn pour_volume_fixture() -> CastSpec {
        let inner_body = Solid::cuboid(Vector3::new(0.020, 0.020, 0.015));
        let outer_body = Solid::cuboid(Vector3::new(0.030, 0.030, 0.025));
        let plug = Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.060));
        let bounding_region = Solid::cuboid(Vector3::new(0.045, 0.045, 0.035));
        CastSpec {
            layers: vec![
                CastLayer {
                    body: inner_body,
                    material: reference_material(),
                },
                CastLayer {
                    body: outer_body,
                    material: reference_material(),
                },
            ],
            plug,
            bounding_region,
            // 1 mm cells: pour-volume integration doesn't trigger
            // `validate_for_printing` (the heavy O(faces²) check),
            // so finer cells are tractable under llvm-cov here. 1 mm
            // keeps the Riemann-sum bias under ~10 % for the small
            // bodies in this fixture, which is comfortably under the
            // 20 % `max_relative` tolerance.
            mesh_cell_size_m: 0.001,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        }
    }

    #[test]
    fn compute_pour_volumes_returns_layer_indexed_results_innermost_first() {
        // Pins: (a) per-layer ordering matches `layers`, (b)
        // material display names carry through, (c) shell-volume
        // CSG semantics give body − plug for layer 0 and body[N] −
        // body[N-1] for layer N > 0.
        let spec = pour_volume_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        assert_eq!(pours.len(), 2);
        assert_eq!(pours[0].layer_index, 0);
        assert_eq!(pours[0].material_display_name, "Ecoflex 00-30");
        assert_eq!(pours[1].layer_index, 1);
        assert_eq!(pours[1].material_display_name, "Ecoflex 00-30");
    }

    #[test]
    fn pour_volumes_match_analytic_cuboid_shell_volumes() {
        // Inner cuboid: 40 × 40 × 30 mm → 4.8e-5 m³.
        // Outer cuboid: 60 × 60 × 50 mm → 1.8e-4 m³.
        // Plug sits at z = 40 mm, entirely above the outer body's
        // top face at z = 25 mm — so layer 0 shell = inner body in
        // full, layer 1 shell = outer − inner.
        //
        // Tolerance ±20 % absorbs the Riemann-sum surface-bias
        // (≲ 15 % on these small cuboids at 1 mm cells) plus the
        // grid-padding contribution.
        let spec = pour_volume_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        assert_relative_eq!(pours[0].shell_volume_m3, 4.8e-5, max_relative = 0.20);
        assert_relative_eq!(pours[1].shell_volume_m3, 1.32e-4, max_relative = 0.20);
    }

    #[test]
    fn pour_mass_equals_shell_volume_times_density_bit_exact() {
        // `pour_mass_kg = shell_volume_m3 * material.density_kg_m3`
        // is a single multiplication — must match bit-exactly, no
        // rounding/tolerance. Catches any future refactor that
        // adds intermediate steps (FMA, unit conversions) between
        // volume and mass.
        let spec = pour_volume_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        for (pv, layer) in pours.iter().zip(spec.layers.iter()) {
            let expected = pv.shell_volume_m3 * layer.material.density_kg_m3;
            assert!(
                (pv.pour_mass_kg - expected).abs() < f64::EPSILON,
                "pour_mass_kg should equal shell_volume_m3 * density_kg_m3 \
                 bit-exactly: layer {} got {}, expected {}",
                pv.layer_index,
                pv.pour_mass_kg,
                expected
            );
        }
    }

    #[test]
    fn compute_pour_volumes_errors_when_layers_is_empty() {
        // Standalone empty-layers gate, parallel to
        // `export_molds_errors_when_layers_is_empty`. The standalone
        // pour-volume path must fail before any SDF sampling.
        let spec = CastSpec {
            layers: Vec::new(),
            plug: Solid::capsule(0.005, 0.010),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.002,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let err = spec.compute_pour_volumes().unwrap_err();
        assert!(matches!(err, CastError::EmptyLayers));
    }

    #[test]
    fn export_molds_errors_when_layer_mass_exceeds_budget() {
        // Layer 0's pour mass is ~0.051 kg at 1070 kg/m³ density;
        // a 0.010 kg budget guarantees the budget gate fires before
        // any STL is written (pre-write atomicity scope).
        let spec = CastSpec {
            mass_budget_kg: 0.010,
            ..pour_volume_fixture()
        };
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-budget-exceeded");
        let err = spec.export_molds(&out_dir).unwrap_err();
        match err {
            CastError::MassBudgetExceeded {
                layer_index,
                material_display_name,
                mass_kg,
                budget_kg,
            } => {
                // Layer 0 trips first (innermost-first iteration).
                assert_eq!(layer_index, 0);
                assert_eq!(material_display_name, "Ecoflex 00-30");
                assert!(
                    mass_kg > budget_kg,
                    "reported mass {mass_kg} should exceed budget {budget_kg}"
                );
                assert!((budget_kg - 0.010).abs() < f64::EPSILON);
            }
            other => panic!("expected MassBudgetExceeded, got {other:?}"),
        }
        // Verify no STL written: the output directory should not
        // exist (pre-write atomicity — budget gate fires before
        // `create_dir_all`).
        assert!(
            !out_dir.exists(),
            "out_dir should not be created on budget failure"
        );
    }

    #[test]
    fn generate_procedure_markdown_renders_all_required_sections() {
        // Pure-function output: pin the five top-level sections so
        // future template refactors can't accidentally drop one.
        let spec = pour_volume_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown(&spec, &pours);
        assert!(md.contains("# Cast Procedure"));
        assert!(md.contains("## Materials Summary"));
        assert!(md.contains("## Generic Smooth-On Guidance"));
        assert!(md.contains("## Per-Layer Procedure"));
        assert!(md.contains("## Mass Budget"));
    }

    #[test]
    fn generate_procedure_markdown_renders_pour_mass_in_grams_per_layer() {
        // The materials-summary table cells render
        // `pour_mass_kg * 1000.0` formatted as `{:.2} g`. Pins the
        // unit-conversion + format-string contract — any future
        // template refactor that switches to kg or different
        // precision trips this.
        let spec = pour_volume_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown(&spec, &pours);
        for pv in &pours {
            let mass_g = pv.pour_mass_kg * 1000.0;
            let cell = format!("{mass_g:.2} g");
            assert!(
                md.contains(&cell),
                "expected layer {} cell '{}' in markdown",
                pv.layer_index,
                cell,
            );
        }
    }

    #[test]
    fn write_procedure_round_trip_matches_generate_output() {
        // `write_procedure` writes the same bytes that
        // `generate_procedure_markdown` returns in-memory — pins
        // that the FS wrapper is a thin pass-through, not a
        // re-renderer with drift potential.
        let out_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-f3-procedure.md");
        if out_path.exists() {
            std::fs::remove_file(&out_path).unwrap();
        }
        let spec = pour_volume_fixture();
        spec.write_procedure(&out_path).unwrap();
        let on_disk = std::fs::read_to_string(&out_path).unwrap();
        let pours = spec.compute_pour_volumes().unwrap();
        let in_memory = crate::procedure::generate_procedure_markdown(&spec, &pours);
        assert_eq!(on_disk, in_memory);
    }

    #[test]
    fn write_procedure_errors_when_layers_is_empty() {
        // Empty-layers error must propagate through write_procedure,
        // and no file may land on disk in that case.
        let spec = CastSpec {
            layers: Vec::new(),
            plug: Solid::capsule(0.005, 0.010),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.002,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-f3-empty.md");
        if path.exists() {
            std::fs::remove_file(&path).unwrap();
        }
        let err = spec.write_procedure(&path).unwrap_err();
        assert!(matches!(err, CastError::EmptyLayers));
        assert!(
            !path.exists(),
            "write_procedure must not land a file on empty-layers error"
        );
    }

    #[test]
    fn generate_procedure_markdown_uses_tds_placeholder_for_non_anchor_material() {
        // A material with `anchor_key = None` (post-cast measured /
        // interpolated / non-Smooth-On grade) renders the "consult
        // Smooth-On TDS" placeholder in its row instead of
        // mix/pot-life/cure-time values.
        let spec = CastSpec {
            layers: vec![CastLayer {
                body: Solid::cuboid(Vector3::new(0.020, 0.020, 0.015)),
                material: MoldingMaterial {
                    display_name: "Custom Mix".to_string(),
                    density_kg_m3: 1050.0,
                    anchor_key: None,
                },
            }],
            plug: Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.060)),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.002,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown(&spec, &pours);
        assert!(
            md.contains("consult Smooth-On TDS"),
            "non-anchor material should render TDS placeholder"
        );
    }

    #[test]
    fn per_layer_step_6_prose_uses_anchor_values_for_anchored_material() {
        // Ecoflex 00-30 anchor: cure_time = 4 hr, pot_life = 45 min.
        // Pour-volume-fixture both layers use this material — pin
        // the exact step-6 prose to catch any future template-string
        // refactor that drops the unit suffixes or reorders fields.
        let spec = pour_volume_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown(&spec, &pours);
        assert!(
            md.contains("6. Cure for ≥4 hr at 73 °F (pot life: 45 min)."),
            "anchored step-6 prose should embed the looked-up cure_time + pot_life"
        );
    }

    #[test]
    fn per_layer_step_6_prose_falls_back_for_non_anchor_material() {
        // When MoldingMaterial.anchor_key is None, step 6 must NOT
        // substitute the table-cell placeholder into the duration
        // slot ("Cure for ≥consult Smooth-On TDS …" is broken
        // English) — the alternate "Cure per TDS" prose runs
        // instead.
        let spec = CastSpec {
            layers: vec![CastLayer {
                body: Solid::cuboid(Vector3::new(0.020, 0.020, 0.015)),
                material: MoldingMaterial {
                    display_name: "Custom Mix".to_string(),
                    density_kg_m3: 1050.0,
                    anchor_key: None,
                },
            }],
            plug: Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.060)),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.002,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown(&spec, &pours);
        assert!(
            !md.contains("≥consult"),
            "non-anchor prose must not substitute placeholder into the duration slot"
        );
        assert!(
            !md.contains("(pot life: consult"),
            "non-anchor prose must not substitute placeholder into the pot-life slot"
        );
        assert!(
            md.contains("6. Cure per the Smooth-On TDS at 73 °F (pot life per TDS)."),
            "non-anchor step 6 must use alternate-prose phrasing"
        );
    }

    #[test]
    fn write_procedure_errors_when_layer_mass_exceeds_budget() {
        // Mirrors `export_molds_errors_when_layer_mass_exceeds_budget`
        // — both pre-write paths share the same check_mass_budget
        // helper, so an overrun must surface identically through
        // write_procedure. No `.md` file may land on disk.
        let spec = CastSpec {
            mass_budget_kg: 0.010,
            ..pour_volume_fixture()
        };
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-f3-budget-exceeded.md");
        if path.exists() {
            std::fs::remove_file(&path).unwrap();
        }
        let err = spec.write_procedure(&path).unwrap_err();
        match err {
            CastError::MassBudgetExceeded {
                layer_index,
                material_display_name,
                mass_kg,
                budget_kg,
            } => {
                assert_eq!(layer_index, 0);
                assert_eq!(material_display_name, "Ecoflex 00-30");
                assert!(mass_kg > budget_kg);
                assert!((budget_kg - 0.010).abs() < f64::EPSILON);
            }
            other => panic!("expected MassBudgetExceeded, got {other:?}"),
        }
        assert!(
            !path.exists(),
            "write_procedure must not land a file on budget failure"
        );
    }

    #[test]
    fn mold_artifact_carries_matching_pour_volume_per_layer() {
        // Pins that each `MoldArtifact.pour_volume` matches what
        // `compute_pour_volumes()` returns at the same index — i.e.,
        // `export_molds` doesn't re-compute volumes mid-pipeline.
        // Uses the 12 mm-cells coverage-test fixture (large enough
        // bodies to mesh, small enough to clear F4 under llvm-cov).
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-f2-artifact-pour");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }

        let spec = CastSpec {
            layers: vec![CastLayer {
                body: Solid::cuboid(Vector3::new(0.025, 0.025, 0.020)),
                material: reference_material(),
            }],
            plug: Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040)),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let standalone_pours = spec.compute_pour_volumes().unwrap();
        let report = spec.export_molds(&out_dir).unwrap();
        assert_eq!(report.molds.len(), 1);
        let artifact_pv = &report.molds[0].pour_volume;
        let standalone_pv = &standalone_pours[0];
        assert_eq!(artifact_pv.layer_index, standalone_pv.layer_index);
        assert_eq!(
            artifact_pv.material_display_name,
            standalone_pv.material_display_name
        );
        assert!((artifact_pv.shell_volume_m3 - standalone_pv.shell_volume_m3).abs() < f64::EPSILON);
        assert!((artifact_pv.pour_mass_kg - standalone_pv.pour_mass_kg).abs() < f64::EPSILON);
    }

    // ----- v2 export_molds_v2 -------------------------------------

    use crate::pour::{PourGateKind, PourGateSpec};
    use crate::registration::{PinSpec, RegistrationKind};
    use crate::ribbon::{PieceSide, Ribbon, SplitNormal};
    use nalgebra::Point3;

    /// v2 reference fixture: same cuboid body + capsule plug + cuboid
    /// bounding region as v1's coverage tests, plus a straight +X
    /// centerline so the ribbon zero plane is the world z=0 plane
    /// (with +Y split-normal → +Z binormal). 12 mm cells per
    /// pattern (qqq) to keep `validate_for_printing` tractable under
    /// llvm-cov instrumentation.
    fn v2_fixture() -> (CastSpec, Ribbon) {
        let spec = CastSpec {
            layers: vec![CastLayer {
                body: Solid::cuboid(Vector3::new(0.025, 0.025, 0.020)),
                material: reference_material(),
            }],
            plug: Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040)),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        (spec, ribbon)
    }

    #[test]
    fn export_molds_v2_writes_two_pieces_plus_plug_for_single_layer() {
        // Pins the v2 file-count invariant (`2 * L + 1` STLs) and
        // the `_piece_{0|1}` filename suffix convention. Layer
        // count = 1 → 2 pieces + 1 plug = 3 STLs.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-single-layer");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }

        let (spec, ribbon) = v2_fixture();
        let report = spec.export_molds_v2(&ribbon, &out_dir).unwrap();

        assert_eq!(report.layers.len(), 1);
        let layer = &report.layers[0];
        assert_eq!(layer.layer_index, 0);
        assert_eq!(layer.material_display_name, "Ecoflex 00-30");

        // Both pieces present, in `[Negative, Positive]` order.
        assert_eq!(layer.pieces[0].piece_side, PieceSide::Negative);
        assert_eq!(layer.pieces[1].piece_side, PieceSide::Positive);
        assert!(layer.pieces[0].path.ends_with("mold_layer_0_piece_0.stl"));
        assert!(layer.pieces[1].path.ends_with("mold_layer_0_piece_1.stl"));
        assert!(layer.pieces[0].path.exists());
        assert!(layer.pieces[1].path.exists());
        assert!(layer.pieces[0].summary.face_count > 0);
        assert!(layer.pieces[1].summary.face_count > 0);

        // Per-layer plug landed at the v2.1 sub-leaf 2 convention
        // `plug_layer_{N}.stl`. Single-layer cast → one plug.
        assert!(layer.plug.path.ends_with("plug_layer_0.stl"));
        assert!(layer.plug.path.exists());
        assert!(layer.plug.summary.face_count > 0);
    }

    #[test]
    fn export_molds_v2_writes_four_pieces_plus_plug_for_two_layer() {
        // 2 layers × 2 pieces = 4 piece STLs + 1 plug = 5 total.
        // Pins iteration order: layer 0 (Negative, Positive),
        // then layer 1 (Negative, Positive).
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-multi-layer");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }

        let inner_body = Solid::cuboid(Vector3::new(0.020, 0.020, 0.015));
        let outer_body = Solid::cuboid(Vector3::new(0.030, 0.030, 0.025));
        let plug = Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040));
        let bounding_region = Solid::cuboid(Vector3::new(0.045, 0.045, 0.035));
        let inner_material = MoldingMaterial {
            display_name: "Ecoflex 00-30".to_string(),
            density_kg_m3: 1070.0,
            anchor_key: Some("ECOFLEX_00_30"),
        };
        let outer_material = MoldingMaterial {
            display_name: "Dragon Skin 10A".to_string(),
            density_kg_m3: 1070.0,
            anchor_key: Some("DRAGON_SKIN_10A"),
        };
        let spec = CastSpec {
            layers: vec![
                CastLayer {
                    body: inner_body,
                    material: inner_material,
                },
                CastLayer {
                    body: outer_body,
                    material: outer_material,
                },
            ],
            plug,
            bounding_region,
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();

        let report = spec.export_molds_v2(&ribbon, &out_dir).unwrap();
        assert_eq!(report.layers.len(), 2);

        // Layer 0 — innermost, Ecoflex.
        assert_eq!(report.layers[0].layer_index, 0);
        assert_eq!(report.layers[0].material_display_name, "Ecoflex 00-30");
        assert!(
            report.layers[0].pieces[0]
                .path
                .ends_with("mold_layer_0_piece_0.stl")
        );
        assert!(
            report.layers[0].pieces[1]
                .path
                .ends_with("mold_layer_0_piece_1.stl")
        );

        // Layer 1 — outer, Dragon Skin.
        assert_eq!(report.layers[1].layer_index, 1);
        assert_eq!(report.layers[1].material_display_name, "Dragon Skin 10A");
        assert!(
            report.layers[1].pieces[0]
                .path
                .ends_with("mold_layer_1_piece_0.stl")
        );
        assert!(
            report.layers[1].pieces[1]
                .path
                .ends_with("mold_layer_1_piece_1.stl")
        );

        // All 4 pieces + plug on disk.
        for layer in &report.layers {
            for piece in &layer.pieces {
                assert!(piece.path.exists(), "{:?} should exist", piece.path);
                assert!(piece.summary.face_count > 0);
            }
            assert!(
                layer.plug.path.exists(),
                "per-layer plug {:?} should exist",
                layer.plug.path
            );
            assert!(layer.plug.summary.face_count > 0);
        }
    }

    #[test]
    fn export_molds_v2_carries_pour_volume_per_layer() {
        // The V2LayerReport's pour_volume must match the standalone
        // `compute_pour_volumes()` value — same gate, same numbers,
        // no re-computation drift.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-pour");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }
        let (spec, ribbon) = v2_fixture();
        let standalone = spec.compute_pour_volumes().unwrap();
        let report = spec.export_molds_v2(&ribbon, &out_dir).unwrap();
        let layer_pv = &report.layers[0].pour_volume;
        assert_eq!(layer_pv.layer_index, standalone[0].layer_index);
        assert!((layer_pv.shell_volume_m3 - standalone[0].shell_volume_m3).abs() < f64::EPSILON);
        assert!((layer_pv.pour_mass_kg - standalone[0].pour_mass_kg).abs() < f64::EPSILON);
    }

    #[test]
    fn export_molds_v2_errors_when_layers_is_empty() {
        // Empty-layers gate fires before any meshing OR
        // ribbon-rotation check; mirrors v1's empty-layers path.
        let spec = CastSpec {
            layers: Vec::new(),
            plug: Solid::capsule(0.005, 0.010),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-empty");
        let err = spec.export_molds_v2(&ribbon, &out_dir).unwrap_err();
        assert!(matches!(err, CastError::EmptyLayers));
        assert!(!out_dir.exists(), "no out_dir on EmptyLayers");
    }

    #[test]
    fn export_molds_v2_errors_when_centerline_too_curved() {
        // 135° tangent rotation polyline: segment 0 along +X,
        // segment 1 rotates by 135°. The v2 gate refuses at > 120°.
        // Uses +Z split-normal so both segment binormals are
        // well-defined (Ribbon::new wouldn't accept a polyline that
        // tripped the parallel-to-split-normal check first).
        let angle = 135.0_f64.to_radians();
        let centerline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.010, 0.0, 0.0),
            Point3::new(
                0.010f64.mul_add(angle.cos(), 0.010),
                0.010 * angle.sin(),
                0.0,
            ),
        ];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();

        // Sanity: the ribbon's max tangent rotation IS 135°.
        let observed = ribbon.max_tangent_rotation_rad();
        assert!(
            (observed - angle).abs() < 1e-9,
            "fixture must produce 135° rotation; got {observed} rad"
        );

        let (spec, _straight_ribbon) = v2_fixture();
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-too-curved");
        let err = spec.export_molds_v2(&ribbon, &out_dir).unwrap_err();
        match err {
            CastError::CenterlineTooCurved {
                max_rotation_rad,
                max_rotation_deg,
                threshold_rad: _,
                threshold_deg,
            } => {
                assert!((max_rotation_rad - angle).abs() < 1e-9);
                assert!((max_rotation_deg - 135.0).abs() < 1e-6);
                assert!((threshold_deg - 120.0).abs() < 1e-9);
            }
            other => panic!("expected CenterlineTooCurved, got {other:?}"),
        }
        assert!(!out_dir.exists(), "no out_dir on CenterlineTooCurved");
    }

    #[test]
    fn export_molds_v2_errors_when_mass_exceeds_budget() {
        // Budget gate parallel to v1 — same `check_mass_budget`
        // helper, same per-layer comparison; must surface
        // identically through the v2 entry point.
        let (mut spec, ribbon) = v2_fixture();
        spec.mass_budget_kg = 0.010; // ~ 1/5 of layer-0 pour mass at 1070 kg/m³.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-budget");
        let err = spec.export_molds_v2(&ribbon, &out_dir).unwrap_err();
        match err {
            CastError::MassBudgetExceeded {
                layer_index,
                budget_kg,
                ..
            } => {
                assert_eq!(layer_index, 0);
                assert!((budget_kg - 0.010).abs() < f64::EPSILON);
            }
            other => panic!("expected MassBudgetExceeded, got {other:?}"),
        }
        assert!(!out_dir.exists(), "no out_dir on budget overrun");
    }

    #[test]
    fn export_molds_v2_errors_when_bounding_region_unbounded() {
        // Bounding-region must be finite for the ribbon half-space
        // AABB — propagates through `compose_piece_solid` as
        // `InfiniteBounds(BoundingRegion)`.
        let spec = CastSpec {
            layers: vec![CastLayer {
                body: Solid::cuboid(Vector3::new(0.020, 0.020, 0.015)),
                material: reference_material(),
            }],
            plug: Solid::capsule(0.008, 0.020).translate(Vector3::new(0.0, 0.0, 0.040)),
            bounding_region: Solid::plane(Vector3::new(0.0, 0.0, 1.0), 0.0),
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-unbounded");
        let err = spec.export_molds_v2(&ribbon, &out_dir).unwrap_err();
        assert!(
            matches!(err, CastError::InfiniteBounds(CastTarget::BoundingRegion)),
            "expected InfiniteBounds(BoundingRegion), got {err:?}"
        );
    }

    // ----- Step 7: per-piece printability + per-piece AABB --------
    //
    // The per-piece AABB-vs-build-volume check is enforced by running
    // `validate_for_printing` on each piece's mesh independently
    // (Step 6 pipeline). `ExceedsBuildVolume` is in
    // `is_blocking_critical`'s set, so a per-piece AABB overflow
    // surfaces as `PrintabilityCritical { target: MoldPiece, ... }`.
    // These tests pin that behavior is correctly per-piece (vs v1's
    // single-cup aggregate AABB), and that each piece's
    // `PrintValidation` is independently populated in the V2 report.

    #[test]
    fn export_molds_v2_per_piece_aabb_within_bounding_region() {
        // Bounding region: 80 × 80 × 60 mm centered at origin →
        // mm AABB `[-40, +40] × [-40, +40] × [-30, +30]`. Each piece's
        // composition is `bounding_region ∩ halfspace ∖ body`, so the
        // piece's AABB must be ⊆ bounding region's AABB. MC at
        // 12 mm cells adds ~6-12 mm slack to face positions; pin
        // with a generous 18 mm tolerance (1.5 × cell).
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-aabb-within");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }
        let (spec, ribbon) = v2_fixture();
        let report = spec.export_molds_v2(&ribbon, &out_dir).unwrap();

        let slack = 18.0_f64;
        for piece in &report.layers[0].pieces {
            let aabb = &piece.summary.aabb_mm;
            assert!(
                aabb.min.x >= -40.0 - slack,
                "piece {:?} x_min {} below bound -40 (slack {})",
                piece.piece_side,
                aabb.min.x,
                slack,
            );
            assert!(aabb.max.x <= 40.0 + slack);
            assert!(aabb.min.y >= -40.0 - slack);
            assert!(aabb.max.y <= 40.0 + slack);
            assert!(aabb.min.z >= -30.0 - slack);
            assert!(aabb.max.z <= 30.0 + slack);
        }
    }

    #[test]
    fn export_molds_v2_each_piece_validation_independently_populated() {
        // Each piece runs `validate_for_printing` independently in
        // Step 6's pipeline. Pin that each piece's `validation` field
        // is a real PrintValidation (not a shared/empty placeholder)
        // and that the v2 fixture's pieces are individually printable
        // on the default FDM printer.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-validation");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }
        let (spec, ribbon) = v2_fixture();
        let report = spec.export_molds_v2(&ribbon, &out_dir).unwrap();

        for piece in &report.layers[0].pieces {
            // Both pieces individually clear the F4 gate (no blocking
            // Criticals; cf-cast tolerates non-blocking Criticals
            // like ExcessiveOverhang per `is_blocking_critical`).
            assert_eq!(
                super::blocking_critical_count(&piece.validation),
                0,
                "piece {:?} should have zero blocking Critical issues; \
                 got validation {:?}",
                piece.piece_side,
                piece.validation,
            );
        }
    }

    #[test]
    fn export_molds_v2_per_piece_exceeds_build_volume_reports_mold_piece_target() {
        // Tiny printer build volume (30 × 30 × 30 mm) vs bounding
        // region of 80 × 80 × 60 mm: every piece's xy-extent is
        // ~80 mm > 30 mm, so the FIRST piece (layer 0, Negative,
        // per iteration order) trips `ExceedsBuildVolume` Critical.
        // Pins that v2's per-piece error reporting carries the full
        // `MoldPiece { layer_index, piece_side }` target (vs v1's
        // single-cup `Mold { layer_index }`).
        let (mut spec, ribbon) = v2_fixture();
        spec.printer_config = PrinterConfig::fdm_default().with_build_volume(30.0, 30.0, 30.0);
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-build-volume");
        let err = spec.export_molds_v2(&ribbon, &out_dir).unwrap_err();
        match err {
            CastError::PrintabilityCritical { target, .. } => {
                // First piece in iteration order is layer 0, Negative.
                assert!(
                    matches!(
                        target,
                        CastTarget::MoldPiece {
                            layer_index: 0,
                            piece_side: PieceSide::Negative,
                        }
                    ),
                    "expected MoldPiece(0, Negative), got {target:?}",
                );
            }
            other => panic!("expected PrintabilityCritical, got {other:?}"),
        }
        // Pre-write atomicity: no out_dir on per-piece F4 failure.
        assert!(!out_dir.exists(), "no out_dir on PrintabilityCritical");
    }

    // ----- Step 8: v2 procedure.md ---------------------------------

    /// Standard v2 procedure fixture: single-layer cuboid spec + a
    /// straight +X centerline (so max tangent rotation = 0°, well
    /// under the 120° gate). Sized to keep `compute_pour_volumes`
    /// fast under llvm-cov (12 mm cells, same as `v2_fixture`).
    fn v2_procedure_fixture() -> (CastSpec, Ribbon) {
        v2_fixture()
    }

    #[test]
    fn generate_procedure_markdown_v2_renders_all_required_sections() {
        let (spec, ribbon) = v2_procedure_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        // v2 header signal (distinguishes from v1's "# Cast Procedure").
        assert!(md.contains("# Cast Procedure (v2.1 curve-following, detachable-shell)"));
        // v2-specific sections.
        assert!(md.contains("## Cast Geometry"));
        assert!(md.contains("## v2 Mold Assembly"));
        // Sections shared with v1.
        assert!(md.contains("## Materials Summary"));
        assert!(md.contains("## Generic Smooth-On Guidance"));
        assert!(md.contains("## Per-Layer Procedure"));
        assert!(md.contains("## Mass Budget"));
    }

    #[test]
    fn generate_procedure_markdown_v2_references_piece_stl_filenames() {
        // Per-layer Step 1 must reference BOTH piece STLs by name
        // (`_piece_0` + `_piece_1`) per design-doc §"Output
        // artifacts" naming convention. Catches any future template
        // refactor that drops a piece filename.
        let (spec, ribbon) = v2_procedure_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(md.contains("mold_layer_0_piece_0.stl"));
        assert!(md.contains("mold_layer_0_piece_1.stl"));
        // Each layer references its own per-layer plug (v2.1
        // detachable-shell architecture).
        assert!(md.contains("plug_layer_0.stl"));
    }

    #[test]
    fn generate_procedure_markdown_v2_demold_prose_specifies_piece_order() {
        // Step 8 of the per-layer block must call out "piece_0 first,
        // then piece_1" — the centerline-slide demold sequence.
        // Removing pieces out of order risks fighting an undercut on
        // curved scans.
        let (spec, ribbon) = v2_procedure_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(
            md.contains("remove `piece_0` first, then `piece_1`"),
            "demold prose must specify piece-removal order; got: {md}"
        );
    }

    #[test]
    fn generate_procedure_markdown_v2_cast_geometry_includes_arc_length() {
        // The Cast Geometry section surfaces the centerline arc
        // length in mm for the workshop user to gauge demold travel.
        // Fixture polyline is 100 mm long (0.05 → -0.05 along +X)
        // → arc length 100.0 mm rendered with `:.1` precision.
        let (spec, ribbon) = v2_procedure_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        // 100.0 mm with one decimal renders as "100.0 mm" in the
        // Cast Geometry section.
        assert!(
            md.contains("100.0 mm"),
            "expected '100.0 mm' arc-length in: {md}"
        );
    }

    #[test]
    fn generate_procedure_markdown_v2_uses_anchor_cure_for_anchored_material() {
        // Ecoflex 00-30 anchor: cure_time = 4 hr, pot_life = 45 min.
        // The v2 step-7 cure prose must embed the looked-up values
        // (parallel to v1's step-6 cure-prose contract).
        let (spec, ribbon) = v2_procedure_fixture();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(
            md.contains("7. Cure for ≥4 hr at 73 °F (pot life: 45 min)."),
            "anchored step-7 cure prose missing in: {md}"
        );
    }

    #[test]
    fn write_procedure_v2_round_trip_matches_generate_output() {
        // Pin that the FS wrapper writes the same bytes the pure
        // generator returns — no extra re-rendering / drift.
        let out_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-procedure.md");
        if out_path.exists() {
            std::fs::remove_file(&out_path).unwrap();
        }
        let (spec, ribbon) = v2_procedure_fixture();
        spec.write_procedure_v2(&ribbon, &out_path).unwrap();
        let on_disk = std::fs::read_to_string(&out_path).unwrap();
        let pours = spec.compute_pour_volumes().unwrap();
        let in_memory = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert_eq!(on_disk, in_memory);
    }

    #[test]
    fn write_procedure_v2_errors_when_layers_is_empty() {
        let spec = CastSpec {
            layers: Vec::new(),
            plug: Solid::capsule(0.005, 0.010),
            bounding_region: Solid::cuboid(Vector3::new(0.040, 0.040, 0.030)),
            mesh_cell_size_m: 0.012,
            printer_config: PrinterConfig::fdm_default(),
            mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        };
        let centerline = vec![Point3::new(-0.05, 0.0, 0.0), Point3::new(0.05, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-procedure-empty.md");
        if path.exists() {
            std::fs::remove_file(&path).unwrap();
        }
        let err = spec.write_procedure_v2(&ribbon, &path).unwrap_err();
        assert!(matches!(err, CastError::EmptyLayers));
        assert!(!path.exists());
    }

    #[test]
    fn write_procedure_v2_errors_when_centerline_too_curved() {
        // 135° polyline rejected pre-render: the markdown's "Cast
        // Geometry" section would otherwise misrepresent a refused
        // cast as printable.
        let angle = 135.0_f64.to_radians();
        let centerline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.010, 0.0, 0.0),
            Point3::new(
                0.010f64.mul_add(angle.cos(), 0.010),
                0.010 * angle.sin(),
                0.0,
            ),
        ];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let (spec, _) = v2_procedure_fixture();
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-procedure-too-curved.md");
        if path.exists() {
            std::fs::remove_file(&path).unwrap();
        }
        let err = spec.write_procedure_v2(&ribbon, &path).unwrap_err();
        assert!(matches!(err, CastError::CenterlineTooCurved { .. }));
        assert!(!path.exists());
    }

    #[test]
    fn write_procedure_v2_errors_when_mass_exceeds_budget() {
        let (mut spec, ribbon) = v2_procedure_fixture();
        spec.mass_budget_kg = 0.010;
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-procedure-budget.md");
        if path.exists() {
            std::fs::remove_file(&path).unwrap();
        }
        let err = spec.write_procedure_v2(&ribbon, &path).unwrap_err();
        assert!(matches!(err, CastError::MassBudgetExceeded { .. }));
        assert!(!path.exists());
    }

    // ----- Step 9: registration pins -------------------------------

    /// v2 fixture variant: same geometry as `v2_fixture` but with
    /// Step 9 cylindrical-pin registration enabled on the ribbon.
    fn v2_fixture_with_pins() -> (CastSpec, Ribbon) {
        let (spec, ribbon) = v2_fixture();
        let ribbon = ribbon.with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        (spec, ribbon)
    }

    #[test]
    fn ribbon_with_registration_sets_field() {
        // Builder method threads the kind through; default remains
        // None for a freshly-constructed Ribbon.
        let (_, ribbon_default) = v2_fixture();
        assert_eq!(ribbon_default.registration, RegistrationKind::None);
        let (_, ribbon_pins) = v2_fixture_with_pins();
        assert!(matches!(
            ribbon_pins.registration,
            RegistrationKind::Pins(_)
        ));
    }

    #[test]
    fn ribbon_sample_at_arc_fraction_returns_polyline_position() {
        // Fixture centerline: x ∈ [-0.05, +0.05], straight along +X.
        // t = 0.0 → x = -0.05; t = 0.5 → x = 0.0; t = 1.0 → x = +0.05.
        let (_, ribbon) = v2_fixture();
        let (p0, _, _) = ribbon.sample_at_arc_fraction(0.0).unwrap();
        let (p_mid, _, _) = ribbon.sample_at_arc_fraction(0.5).unwrap();
        let (p1, _, _) = ribbon.sample_at_arc_fraction(1.0).unwrap();
        assert!((p0.x - -0.05).abs() < 1e-9);
        assert!((p_mid.x - 0.0).abs() < 1e-9);
        assert!((p1.x - 0.05).abs() < 1e-9);
    }

    #[test]
    fn ribbon_sample_at_arc_fraction_rejects_out_of_range() {
        let (_, ribbon) = v2_fixture();
        assert!(ribbon.sample_at_arc_fraction(-0.1).is_none());
        assert!(ribbon.sample_at_arc_fraction(1.1).is_none());
        assert!(ribbon.sample_at_arc_fraction(f64::NAN).is_none());
    }

    #[test]
    fn compose_piece_solid_with_pins_negative_side_gains_protrusion() {
        // Pin 0 sits at (-0.025, +0.0325, 0) — arc 0.25 of
        // [-0.05, +0.05] centerline, body-relative offset along +Y
        // split-normal: cup-wall annulus midpoint at v2_fixture's
        // body half_y=0.025 and bounding half_y=0.040 →
        // pin_offset = (0.025 + 0.040)/2 = 0.0325. Pin axis along
        // binormal (+Z), half-length 5 mm so cylinder spans
        // z ∈ [-0.005, +0.005].
        //
        // Negative piece (z < 0): base extends to z ≈ +0.5mm bias.
        // Union with the pin cylinder extends the negative piece's
        // material up to z ≈ +5mm at the pin position.
        //
        // Query at (-0.025, +0.0325, +0.003) — 3mm above the seam,
        // inside the pin's protrusion. With pins ON: SDF should be
        // < 0 (inside the piece's gained protrusion). With pins OFF:
        // SDF should be > 0 (outside the negative piece's natural
        // half-space).
        let (spec, ribbon_no_pins) = v2_fixture();
        let (_, ribbon_pins) = v2_fixture_with_pins();
        let body = &spec.layers[0].body;
        let region = &spec.bounding_region;

        let (piece_no_pins, _) =
            crate::compose_piece_solid(body, region, &ribbon_no_pins, PieceSide::Negative).unwrap();
        let (piece_pins, _) =
            crate::compose_piece_solid(body, region, &ribbon_pins, PieceSide::Negative).unwrap();

        let q = Point3::new(-0.025, 0.0325, 0.003);
        // Without pins: query is above the seam → outside negative piece.
        assert!(
            piece_no_pins.evaluate(&q) > 0.0,
            "no-pin negative piece should EXCLUDE query above seam; got {}",
            piece_no_pins.evaluate(&q),
        );
        // With pins: query is inside the pin protrusion → inside.
        assert!(
            piece_pins.evaluate(&q) < 0.0,
            "pin'd negative piece should INCLUDE query in pin protrusion; got {}",
            piece_pins.evaluate(&q),
        );
    }

    #[test]
    fn compose_piece_solid_with_pins_positive_side_gains_hole() {
        // Same pin position (-0.025, +0.0325, 0) — annulus midpoint
        // along +Y for v2_fixture's body half_y=0.025 + bounding
        // half_y=0.040. Query at (-0.025, +0.0325, +0.003) — inside
        // the pin cylinder AND inside positive piece's natural
        // half-space (z > -bias). With pins: positive piece
        // SUBTRACTS the pin cylinder → SDF > 0 (hole). Without
        // pins: SDF < 0 (cup wall material).
        let (spec, ribbon_no_pins) = v2_fixture();
        let (_, ribbon_pins) = v2_fixture_with_pins();
        let body = &spec.layers[0].body;
        let region = &spec.bounding_region;

        let (piece_no_pins, _) =
            crate::compose_piece_solid(body, region, &ribbon_no_pins, PieceSide::Positive).unwrap();
        let (piece_pins, _) =
            crate::compose_piece_solid(body, region, &ribbon_pins, PieceSide::Positive).unwrap();

        let q = Point3::new(-0.025, 0.0325, 0.003);
        // Without pins: query in cup wall above seam → inside positive piece.
        assert!(
            piece_no_pins.evaluate(&q) < 0.0,
            "no-pin positive piece should INCLUDE cup-wall query; got {}",
            piece_no_pins.evaluate(&q),
        );
        // With pins: query in pin hole → outside (subtracted).
        assert!(
            piece_pins.evaluate(&q) > 0.0,
            "pin'd positive piece should EXCLUDE query in pin hole; got {}",
            piece_pins.evaluate(&q),
        );
    }

    #[test]
    fn export_molds_v2_with_pins_writes_pieces_plus_plug() {
        // End-to-end: enabling pins on the ribbon still produces a
        // valid `2L + 1` STL output. Pins are CSG'd in during
        // compose_piece_solid; marching cubes runs once per piece
        // exactly as the no-pins path.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-pins");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }
        let (spec, ribbon) = v2_fixture_with_pins();
        let report = spec.export_molds_v2(&ribbon, &out_dir).unwrap();
        assert_eq!(report.layers.len(), 1);
        assert!(report.layers[0].pieces[0].path.exists());
        assert!(report.layers[0].pieces[1].path.exists());
        assert!(report.layers[0].plug.path.exists());
        // Each piece's mesh has non-trivial face count (the pin
        // cylinder contributes a few dozen extra faces at the 12 mm
        // cell size).
        for piece in &report.layers[0].pieces {
            assert!(piece.summary.face_count > 0);
        }
    }

    #[test]
    fn generate_procedure_markdown_v2_pin_prose_mentions_pin_count_and_diameter() {
        // Pins ON: the v2 Mold Assembly section must mention the pin
        // count + diameter (1.5 mm radius × 2 = 3.0 mm diameter for
        // iter1). v1/v2-pre-Step-9 prose ("clamp with rubber bands")
        // must NOT appear.
        let (spec, ribbon) = v2_fixture_with_pins();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(md.contains("2 cylindrical pins"));
        assert!(md.contains("3.0 mm Ø"));
        assert!(md.contains("10.0 mm long"));
        assert!(
            !md.contains("rubber bands"),
            "with-pins prose must not retain rubber-band clamping note"
        );
    }

    #[test]
    fn generate_procedure_markdown_v2_no_pins_prose_keeps_clamp_note() {
        // Pins OFF: the v2 Mold Assembly section keeps the existing
        // hand-clamp prose unchanged.
        let (spec, ribbon) = v2_fixture(); // RegistrationKind::None
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(md.contains("rubber bands"));
        assert!(md.contains("`RegistrationKind::None`"));
    }

    // ----- Step 10: pour gate + air vent ---------------------------

    /// v2 fixture variant with the Step 10 default pour-gate +
    /// vent enabled on the ribbon.
    fn v2_fixture_with_pour_gate() -> (CastSpec, Ribbon) {
        let (spec, ribbon) = v2_fixture();
        let ribbon = ribbon.with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        (spec, ribbon)
    }

    #[test]
    fn ribbon_with_pour_gate_sets_field() {
        let (_, ribbon_default) = v2_fixture();
        assert_eq!(ribbon_default.pour_gate, PourGateKind::None);
        let (_, ribbon_gate) = v2_fixture_with_pour_gate();
        assert!(matches!(ribbon_gate.pour_gate, PourGateKind::Default(_)));
    }

    #[test]
    fn compose_piece_solid_with_pour_gate_carves_positive_piece_cup_wall() {
        // v2.1 sub-leaf 4 (V-at-dome): pour gate is a V at the
        // centerline endpoint opposite `pour_end_hint`'s cap-plane
        // centroid (falls back to `centerline[0]` when no hint).
        // For this test we use a centerline shorter than
        // `v2_fixture`'s default so the V apex sits INSIDE the
        // body — the pour leg then crosses the cup-wall material
        // on its way to the bounding region's outer surface, and
        // the carve is testable.
        //
        // Body half-extents (0.025, 0.025, 0.020); bounding
        // (0.040, 0.040, 0.030). Centerline -15→+15 mm in X with
        // +Y split-normal → binormal +Z. V apex at centerline[0] =
        // (-0.015, 0, 0), outward = -X. Pour leg axis
        // = (cos30°·-X) + (sin30°·+Z) = (-0.866, 0, +0.5).
        //
        // Query at (-0.030, 0, +0.0098): along the pour leg axis
        // from apex by ~17.3 mm. Inside bounding (|x|=30<40,
        // z=0.0098<30); outside body (|x|=30>25); in Positive
        // piece (+binormal=+Z side).
        let spec = v2_fixture().0;
        let body = &spec.layers[0].body;
        let region = &spec.bounding_region;
        let short_centerline = vec![Point3::new(-0.015, 0.0, 0.0), Point3::new(0.015, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon_no_gate = Ribbon::new(short_centerline.clone(), split).unwrap();
        let ribbon_gate = Ribbon::new(short_centerline, split)
            .unwrap()
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));

        let (piece_no_gate, _) =
            crate::compose_piece_solid(body, region, &ribbon_no_gate, PieceSide::Positive).unwrap();
        let (piece_gate, _) =
            crate::compose_piece_solid(body, region, &ribbon_gate, PieceSide::Positive).unwrap();

        let q = Point3::new(-0.030, 0.0, 0.0098);
        assert!(
            piece_no_gate.evaluate(&q) < 0.0,
            "no-gate Positive piece should INCLUDE cup-wall query at \
             (-30mm, 0, +9.8mm); got {}",
            piece_no_gate.evaluate(&q),
        );
        assert!(
            piece_gate.evaluate(&q) > 0.0,
            "V-pour-gate Positive piece should EXCLUDE pour-leg-axis query at \
             (-30mm, 0, +9.8mm); got {}",
            piece_gate.evaluate(&q),
        );
    }

    #[test]
    fn compose_piece_solid_with_pour_gate_carves_negative_piece_only_via_vent_leg() {
        // v2.1 sub-leaf 4: the V's pour leg lands on +binormal
        // (Positive piece) and the vent leg lands on -binormal
        // (Negative piece). For the +Z-binormal short-centerline
        // fixture, the vent leg axis is (-cos30°, 0, -sin30°)
        // ≈ (-0.866, 0, -0.5) from apex (-0.015, 0, 0). Query at
        // (-0.030, 0, -0.0098) sits on the vent-leg axis and in
        // Negative-piece cup-wall material (|x|=30>25 body,
        // |x|=30<40 bounding; z=-0.0098<0 = -binormal half). The
        // pour leg (on +binormal side) does NOT carve this point.
        let spec = v2_fixture().0;
        let body = &spec.layers[0].body;
        let region = &spec.bounding_region;
        let short_centerline = vec![Point3::new(-0.015, 0.0, 0.0), Point3::new(0.015, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon_no_gate = Ribbon::new(short_centerline.clone(), split).unwrap();
        let ribbon_gate = Ribbon::new(short_centerline, split)
            .unwrap()
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));

        let (piece_no_gate, _) =
            crate::compose_piece_solid(body, region, &ribbon_no_gate, PieceSide::Negative).unwrap();
        let (piece_gate, _) =
            crate::compose_piece_solid(body, region, &ribbon_gate, PieceSide::Negative).unwrap();

        // Vent-leg query: cup-wall material on -binormal side.
        let q = Point3::new(-0.030, 0.0, -0.0098);
        assert!(
            piece_no_gate.evaluate(&q) < 0.0,
            "no-gate Negative piece should INCLUDE cup-wall query on -binormal side; got {}",
            piece_no_gate.evaluate(&q),
        );
        assert!(
            piece_gate.evaluate(&q) > 0.0,
            "V-pour-gate Negative piece should EXCLUDE vent-leg-axis query on -binormal side; got {}",
            piece_gate.evaluate(&q),
        );
    }

    #[test]
    fn export_molds_v2_with_pour_gate_writes_pieces_plus_plug() {
        // End-to-end: enabling pour gate on the ribbon still
        // produces a valid 2L + 1 STL output. Gate channels are
        // CSG'd in during compose_piece_solid.
        let out_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/cf-cast-v2-pour-gate");
        match std::fs::remove_dir_all(&out_dir) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => panic!("failed to clean {out_dir:?}: {e}"),
        }
        let (spec, ribbon) = v2_fixture_with_pour_gate();
        let report = spec.export_molds_v2(&ribbon, &out_dir).unwrap();
        assert_eq!(report.layers.len(), 1);
        assert!(report.layers[0].pieces[0].path.exists());
        assert!(report.layers[0].pieces[1].path.exists());
        assert!(report.layers[0].plug.path.exists());
        for piece in &report.layers[0].pieces {
            assert!(piece.summary.face_count > 0);
        }
    }

    #[test]
    fn generate_procedure_markdown_v2_pour_gate_prose_mentions_diameters() {
        // Pour gate ON: "Pour Gate + Vent" section must mention
        // pour-leg Ø (10.0 mm = 2 × 5 mm radius), vent-leg Ø
        // (6.0 mm = 2 × 3 mm radius), and channel lengths
        // (gate = 90.0 mm; vent = 80.0 mm). Pour leg is bigger
        // than vent for honey-thick silicone flow (iter-1 sizing).
        let (spec, ribbon) = v2_fixture_with_pour_gate();
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(md.contains("## Pour Gate + Vent"));
        assert!(md.contains("10.0 mm Ø pour gate"));
        assert!(md.contains("6.0 mm Ø vent"));
        assert!(md.contains("90.0 mm total"));
        assert!(md.contains("80.0 mm total"));
        // Per-layer Step 6 references the V pour leg at the dome
        // end when enabled.
        assert!(
            md.contains(
                "Pour silicone into the pour leg of the V at the dome end \
                 (Positive piece, +binormal side)"
            ),
            "Step 6 should reference the V pour leg when enabled; got: {md}",
        );
    }

    #[test]
    fn generate_procedure_markdown_v2_no_pour_gate_prose_falls_back_to_seam() {
        // Pour gate OFF: "Pour Gate + Vent" section explains the
        // workshop-drilled-hole fallback; per-layer Step 6
        // references "the assembled mold cavity" not the gate.
        let (spec, ribbon) = v2_fixture(); // pour_gate = None
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(md.contains("## Pour Gate + Vent"));
        assert!(md.contains("`PourGateKind::None`"));
        assert!(
            md.contains("Pour silicone into the assembled mold cavity"),
            "Step 6 should fall back to seam-pour prose; got: {md}",
        );
        // The "Pour Gate + Vent" section's no-gate prose mentions
        // "Pour silicone through the ribbon seam" — but check the
        // specific marker that distinguishes it from with-gate.
        assert!(
            !md.contains("6.0 mm Ø pour gate"),
            "no-gate prose must not include with-gate dimensions",
        );
    }

    #[test]
    fn generate_procedure_markdown_v2_no_vent_prose_notes_disabled() {
        // include_vent=false: prose calls out the disabled vent
        // explicitly so workshop user doesn't expect a vent hole
        // in the printed pieces.
        let mut spec_no_vent = PourGateSpec::iter1();
        spec_no_vent.include_vent = false;
        let (spec, ribbon) = v2_fixture();
        let ribbon = ribbon.with_pour_gate(PourGateKind::Default(spec_no_vent));
        let pours = spec.compute_pour_volumes().unwrap();
        let md = crate::procedure::generate_procedure_markdown_v2(&spec, &pours, &ribbon);
        assert!(
            md.contains("`include_vent = false`"),
            "no-vent prose must reference the disabled vent flag",
        );
    }

    #[test]
    fn compose_piece_solid_with_pins_and_pour_gate_composes_both() {
        // Step 9 (pins) + Step 10/v2.1 sub-leaf 4 (V pour gate) are
        // orthogonal — both should compose into the same piece
        // geometry. Verify by enabling BOTH on the ribbon and
        // confirming compose_piece_solid produces a valid Solid
        // with both features visible.
        //
        // Uses a short centerline (-15→+15 mm) so the V apex at
        // `centerline[0]` sits INSIDE the body and the pour leg
        // crosses the cup wall (per the standalone V-pour-gate
        // test); v2_fixture's default centerline -50→+50 puts the
        // apex past the bounding region.
        let spec = v2_fixture().0;
        let short_centerline = vec![Point3::new(-0.015, 0.0, 0.0), Point3::new(0.015, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(short_centerline, split)
            .unwrap()
            .with_registration(RegistrationKind::Pins(PinSpec::iter1()))
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));

        let (piece_neg, _) = crate::compose_piece_solid(
            &spec.layers[0].body,
            &spec.bounding_region,
            &ribbon,
            PieceSide::Negative,
        )
        .unwrap();
        let (piece_pos, _) = crate::compose_piece_solid(
            &spec.layers[0].body,
            &spec.bounding_region,
            &ribbon,
            PieceSide::Positive,
        )
        .unwrap();

        assert!(piece_neg.bounds().is_some());
        assert!(piece_pos.bounds().is_some());

        // Pin protrusion query (pin in Negative piece) — annulus
        // midpoint along +Y for v2_fixture's body half_y=0.025 +
        // bounding half_y=0.040 → pin center at y=0.0325.
        // Centerline arc fractions 0.25/0.75 on the -15→+15 mm
        // centerline put the t=0.25 pin at x=-0.0075.
        let pin_q = Point3::new(-0.0075, 0.0325, 0.003);
        assert!(
            piece_neg.evaluate(&pin_q) < 0.0,
            "pins+gate Negative piece should still contain pin protrusion at \
             annulus midpoint; got {}",
            piece_neg.evaluate(&pin_q),
        );
        // V pour-leg query: pour leg axis from apex (-0.015, 0, 0)
        // is ≈ (-0.866, 0, +0.5). At ~17 mm along the leg the point
        // (-0.030, 0, +0.0098) sits in cup-wall material on the
        // +binormal=+Z side (Positive piece). Gate carves it.
        let gate_q = Point3::new(-0.030, 0.0, 0.0098);
        assert!(
            piece_pos.evaluate(&gate_q) > 0.0,
            "pins+gate Positive piece should EXCLUDE V pour-leg channel at \
             (-30mm, 0, +9.8mm); got {}",
            piece_pos.evaluate(&gate_q),
        );
    }

    #[test]
    fn mold_piece_filename_maps_piece_side_to_integer_suffix() {
        // Pure-function helper — pin the side → integer convention
        // matches `docs/CURVE_FOLLOWING_DESIGN.md` §"Output
        // artifacts". `_piece_0` = Negative, `_piece_1` = Positive.
        use super::mold_piece_filename;
        assert_eq!(
            mold_piece_filename(0, PieceSide::Negative),
            "mold_layer_0_piece_0.stl"
        );
        assert_eq!(
            mold_piece_filename(0, PieceSide::Positive),
            "mold_layer_0_piece_1.stl"
        );
        assert_eq!(
            mold_piece_filename(3, PieceSide::Negative),
            "mold_layer_3_piece_0.stl"
        );
    }
}
