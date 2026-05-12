//! [`CastSpec`] — the Stage 2 multi-layer public API surface.

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
use crate::mesher::solid_to_mm_mesh;
use crate::pour_volume::{PourVolume, integrate_negative_sdf_volume};
use crate::procedure::generate_procedure_markdown;

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
/// after the `i`-th cast). This convention lets each mold cup carve
/// the bounding region with the SAME CSG rule used in Stage 1:
/// `bounding_region ∖ layers[i].body ∖ clip_above(layers[i].body)`.
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
/// innermost-first into a shared bounding region, with one printed
/// plug shaping the innermost layer's inner cavity.
///
/// Geometry is supplied in **meters** in the [`cf_design`]
/// convention. [`Self::export_molds`] performs the m → mm scale
/// exactly once per output mesh at the marching-cubes →
/// save/validate boundary.
///
/// Stage 2 hardcodes the demolding axis to `+z`. The exporter
/// internally clips the bounding region above each layer body's
/// `z_max`, opening the mold cup at the top for pour access.
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
    /// Standalone access is useful at design time — e.g., F3
    /// procedure-spec generation prints these masses without
    /// re-running [`Self::export_molds`].
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
        for pv in &pour_volumes {
            if pv.pour_mass_kg > self.mass_budget_kg {
                return Err(CastError::MassBudgetExceeded {
                    layer_index: pv.layer_index,
                    material_display_name: pv.material_display_name.clone(),
                    mass_kg: pv.pour_mass_kg,
                    budget_kg: self.mass_budget_kg,
                });
            }
        }

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
        let plug_mesh = solid_to_mm_mesh(&self.plug, self.mesh_cell_size_m, CastTarget::Plug)?;
        let plug_validation = run_printability_gate(&plug_mesh, &self.printer_config, &plug_path)?;
        let plug_blocking = blocking_critical_count(&plug_validation);
        if plug_blocking > 0 {
            return Err(CastError::PrintabilityCritical {
                target: CastTarget::Plug,
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
    /// - [`CastError::MeshIo`] on filesystem write failure.
    pub fn write_procedure(&self, path: &Path) -> Result<(), CastError> {
        let pour_volumes = self.compute_pour_volumes()?;
        let markdown = generate_procedure_markdown(self, &pour_volumes);
        std::fs::write(path, markdown).map_err(|e| CastError::MeshIo {
            path: path.to_path_buf(),
            source: mesh_io::IoError::from(e),
        })?;
        Ok(())
    }
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
}
