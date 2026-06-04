//! The [`Project`] — the workflow's serializable state plus the state
//! machine that keeps it valid.
//!
//! # Invariants
//!
//! A `Project` is always in a legal state. The setters and navigation
//! methods are the only way to mutate it, and they preserve:
//!
//! 1. **Prefix completeness** — the completed steps are a contiguous
//!    prefix (`AddScan`, then `CleanScan`, …). You cannot have molds
//!    without a design.
//! 2. **Reachable current step** — every step *before* `current_step`
//!    is complete (you can only be on a screen you advanced to).
//! 3. **Downstream invalidation** — re-doing an earlier step clears
//!    every artifact after it, because the later work is now stale.
//!
//! [`Project::load`] re-checks 1 and 2 on data read from disk, so a
//! hand-edited or corrupted project file is rejected rather than
//! driven in an illegal state.

use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::error::{Result, StudioError};
use crate::step::Step;

/// The project-file schema version this build reads and writes.
/// Bumped only when the on-disk shape changes; a project declaring a
/// higher version is rejected by [`Project::load`].
pub const PROJECT_SCHEMA_VERSION: u32 = 1;

// ── per-step artifacts ──────────────────────────────────────────────
//
// Each step, when completed, records an artifact. "Step is complete"
// ≡ "its artifact is `Some`". Artifacts are owned + serializable; in
// particular the design draft mirrors the `.design.toml` schema with
// an *owned* material key (the SDK's in-memory type uses `&'static
// str`, which cannot round-trip through a persisted file).

/// Artifact of [`Step::AddScan`] — the chosen raw scan.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScanInput {
    /// Path to the raw scan file the user picked (`.stl` / `.obj` / …).
    pub source_path: PathBuf,
}

/// Artifact of [`Step::CleanScan`] — the castable, sealed scan plus the
/// `.prep.toml` (cap planes + centerline) it produced.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrepInput {
    /// The cleaned, watertight STL.
    pub cleaned_stl: PathBuf,
    /// The `.prep.toml` carrying cap planes + the centerline polyline.
    pub prep_toml: PathBuf,
}

/// One silicone layer in the [`DesignDraft`] stack, innermost first.
/// Mirrors a `.design.toml` `[[layers]]` block with an owned key.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LayerDraft {
    /// Layer thickness in meters.
    pub thickness_m: f64,
    /// Silicone catalog key, e.g. `"ECOFLEX_00_30"`. Owned so the
    /// project file round-trips (the SDK uses a `&'static str`).
    pub material_key: String,
    /// Smooth-On Slacker™ softening fraction (`0.0` = none).
    pub slacker_fraction: f64,
}

/// Artifact of [`Step::DesignLayers`] — cavity inset + the layer stack.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DesignDraft {
    /// How far the cavity is inset from the scan surface, in meters
    /// (the "snugness" of the fit / press-fit reservation).
    pub cavity_inset_m: f64,
    /// The ordered (innermost-first) layer stack.
    pub layers: Vec<LayerDraft>,
}

/// One layer's pour instructions, derived from the cast run. This is
/// the structured data the Step-6 pour assistant renders (mass, mix
/// ratio, pot-life timer, cure wait) — not free-form markdown.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PourStep {
    /// Zero-based layer index, innermost first.
    pub layer_index: usize,
    /// Display name of the silicone, e.g. `"Ecoflex 00-30"`.
    pub material_display_name: String,
    /// Total silicone mass for this layer, in grams.
    pub mass_g: f64,
    /// Part A : Part B mix ratio by weight, e.g. `"1:1"`.
    pub mix_ratio_a_to_b: String,
    /// Working time before the silicone starts to set, in minutes.
    pub pot_life_minutes: u32,
    /// Cure time before the next layer, in hours.
    pub cure_time_hours: f64,
    /// Slacker fraction for this layer, if any.
    pub slacker_fraction: Option<f64>,
}

/// The full ordered pour plan for the piece — one [`PourStep`] per
/// layer, innermost first.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct PourPlan {
    /// Pour steps, in pour order (innermost layer first).
    pub steps: Vec<PourStep>,
}

/// Artifact of [`Step::MakeMolds`] — the generated, printable outputs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MoldOutputs {
    /// Directory the cast run wrote into.
    pub out_dir: PathBuf,
    /// Mold-piece STL paths (the two halves per layer).
    pub mold_stls: Vec<PathBuf>,
    /// Plug STL paths (one per layer).
    pub plug_stls: Vec<PathBuf>,
    /// Other printable STLs the cast emits (e.g. the workshop platform
    /// and registration dowels) — not mold halves or plugs, but still
    /// part of what the user prints.
    pub accessory_stls: Vec<PathBuf>,
    /// The generated human-readable procedure document.
    pub procedure_path: PathBuf,
    /// Total silicone mass across all layers, in grams.
    pub total_mass_g: f64,
    /// Structured pour plan for the Step-6 assistant.
    pub pour_plan: PourPlan,
}

/// Artifact of [`Step::Print`] — where the user saved the files to
/// hand to their slicer / printer.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrintExport {
    /// Folder the printable files were exported to.
    pub export_dir: PathBuf,
}

/// Artifact of [`Step::Pour`] — a record that the pour was completed.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PourRecord {
    /// Number of layers the user marked as poured.
    pub layers_poured: usize,
}

// ── the project ─────────────────────────────────────────────────────

/// A guided-workflow project: the user's progress through the six
/// steps, plus each completed step's artifact. Serializable for
/// autosave/resume; mutated only through the invariant-preserving
/// methods below.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Project {
    /// On-disk schema version. See [`PROJECT_SCHEMA_VERSION`].
    schema_version: u32,
    /// Human-given project name.
    pub name: String,
    /// The step the user is currently on (the visible screen).
    current_step: Step,
    scan: Option<ScanInput>,
    prep: Option<PrepInput>,
    design: Option<DesignDraft>,
    molds: Option<MoldOutputs>,
    print: Option<PrintExport>,
    pour: Option<PourRecord>,
}

impl Project {
    /// Start a fresh project on [`Step::AddScan`] with nothing done.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            schema_version: PROJECT_SCHEMA_VERSION,
            name: name.into(),
            current_step: Step::FIRST,
            scan: None,
            prep: None,
            design: None,
            molds: None,
            print: None,
            pour: None,
        }
    }

    // ── read access ─────────────────────────────────────────────────

    /// The schema version recorded in this project.
    #[must_use]
    pub fn schema_version(&self) -> u32 {
        self.schema_version
    }

    /// The step the user is currently on.
    #[must_use]
    pub fn current_step(&self) -> Step {
        self.current_step
    }

    /// Whether a given step has been completed (its artifact is present).
    #[must_use]
    pub fn is_complete(&self, step: Step) -> bool {
        match step {
            Step::AddScan => self.scan.is_some(),
            Step::CleanScan => self.prep.is_some(),
            Step::DesignLayers => self.design.is_some(),
            Step::MakeMolds => self.molds.is_some(),
            Step::Print => self.print.is_some(),
            Step::Pour => self.pour.is_some(),
        }
    }

    /// Whether the whole workflow is finished (the pour is recorded).
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.is_complete(Step::Pour)
    }

    /// The furthest step completed so far, or `None` if nothing is done.
    #[must_use]
    pub fn furthest_completed(&self) -> Option<Step> {
        Step::ALL.into_iter().rev().find(|s| self.is_complete(*s))
    }

    /// The scan artifact, if [`Step::AddScan`] is complete.
    #[must_use]
    pub fn scan(&self) -> Option<&ScanInput> {
        self.scan.as_ref()
    }

    /// The prep artifact, if [`Step::CleanScan`] is complete.
    #[must_use]
    pub fn prep(&self) -> Option<&PrepInput> {
        self.prep.as_ref()
    }

    /// The design draft, if [`Step::DesignLayers`] is complete.
    #[must_use]
    pub fn design(&self) -> Option<&DesignDraft> {
        self.design.as_ref()
    }

    /// The mold outputs, if [`Step::MakeMolds`] is complete.
    #[must_use]
    pub fn molds(&self) -> Option<&MoldOutputs> {
        self.molds.as_ref()
    }

    /// The print export, if [`Step::Print`] is complete.
    #[must_use]
    pub fn print(&self) -> Option<&PrintExport> {
        self.print.as_ref()
    }

    /// The pour record, if [`Step::Pour`] is complete.
    #[must_use]
    pub fn pour(&self) -> Option<&PourRecord> {
        self.pour.as_ref()
    }

    // ── navigation ───────────────────────────────────────────────────

    /// Advance to the next step. Fails if the current step is not yet
    /// complete, or if already on the final step.
    ///
    /// # Errors
    ///
    /// - [`StudioError::StepNotReady`] if the current step's action
    ///   has not been completed.
    /// - [`StudioError::AlreadyAtEnd`] if on the final step.
    pub fn advance(&mut self) -> Result<Step> {
        if !self.is_complete(self.current_step) {
            return Err(StudioError::StepNotReady {
                from: self.current_step,
                reason: format!("\"{}\" must be completed first", self.current_step.title()),
            });
        }
        let next = self.current_step.next().ok_or(StudioError::AlreadyAtEnd)?;
        self.current_step = next;
        Ok(next)
    }

    /// Go back to the previous step. Completed work is preserved (so a
    /// plain Back→Next leaves the project unchanged); editing a step
    /// is what invalidates downstream work, via the setters.
    ///
    /// # Errors
    ///
    /// [`StudioError::AlreadyAtStart`] if on the first step.
    pub fn go_back(&mut self) -> Result<Step> {
        let prev = self
            .current_step
            .prev()
            .ok_or(StudioError::AlreadyAtStart)?;
        self.current_step = prev;
        Ok(prev)
    }

    // ── step completion (setters) ─────────────────────────────────────
    //
    // Each setter records its step's artifact, clears every downstream
    // artifact (later work is now stale), and parks `current_step` on
    // that step. Setters past the first require the prior step done.

    /// Record the chosen scan ([`Step::AddScan`]). Re-choosing a scan
    /// resets the whole project, since everything downstream depended
    /// on it.
    pub fn set_scan(&mut self, scan: ScanInput) {
        self.scan = Some(scan);
        self.clear_after(Step::AddScan);
        self.current_step = Step::AddScan;
    }

    /// Record the cleaned scan ([`Step::CleanScan`]).
    ///
    /// # Errors
    /// [`StudioError::StepNotReady`] if no scan has been added.
    pub fn set_prep(&mut self, prep: PrepInput) -> Result<()> {
        self.require_complete(Step::AddScan)?;
        self.prep = Some(prep);
        self.clear_after(Step::CleanScan);
        self.current_step = Step::CleanScan;
        Ok(())
    }

    /// Record the layer design ([`Step::DesignLayers`]).
    ///
    /// # Errors
    /// [`StudioError::StepNotReady`] if the scan has not been cleaned.
    pub fn set_design(&mut self, design: DesignDraft) -> Result<()> {
        self.require_complete(Step::CleanScan)?;
        self.design = Some(design);
        self.clear_after(Step::DesignLayers);
        self.current_step = Step::DesignLayers;
        Ok(())
    }

    /// Record the generated molds ([`Step::MakeMolds`]).
    ///
    /// # Errors
    /// [`StudioError::StepNotReady`] if there is no design yet.
    pub fn set_molds(&mut self, molds: MoldOutputs) -> Result<()> {
        self.require_complete(Step::DesignLayers)?;
        self.molds = Some(molds);
        self.clear_after(Step::MakeMolds);
        self.current_step = Step::MakeMolds;
        Ok(())
    }

    /// Record the print export ([`Step::Print`]).
    ///
    /// # Errors
    /// [`StudioError::StepNotReady`] if the molds have not been made.
    pub fn set_print(&mut self, print: PrintExport) -> Result<()> {
        self.require_complete(Step::MakeMolds)?;
        self.print = Some(print);
        self.clear_after(Step::Print);
        self.current_step = Step::Print;
        Ok(())
    }

    /// Record the completed pour ([`Step::Pour`]).
    ///
    /// # Errors
    /// [`StudioError::StepNotReady`] if the files were not exported.
    pub fn set_pour(&mut self, pour: PourRecord) -> Result<()> {
        self.require_complete(Step::Print)?;
        self.pour = Some(pour);
        self.current_step = Step::Pour;
        Ok(())
    }

    // ── persistence ───────────────────────────────────────────────────

    /// Save the project to `path` as pretty JSON, atomically (write to
    /// `<path>.tmp` then rename) so an interrupted save never corrupts
    /// a previously-good file. This is the autosave primitive.
    ///
    /// # Errors
    /// [`StudioError::Serialize`] (including a project whose JSON would not
    /// parse back — see below) or [`StudioError::Io`].
    pub fn save(&self, path: &Path) -> Result<()> {
        let body = serde_json::to_string_pretty(self)
            .map_err(|e| StudioError::Serialize(e.to_string()))?;
        // Verify the JSON round-trips before we replace the previous good
        // file. serde encodes a non-finite f64 (NaN / ±Inf) as the literal
        // `null`, which then fails to deserialize back into the non-Option
        // float artifacts (e.g. a degenerate cast mass / cure time). Catching
        // it here keeps the durability guarantee honest: an unparseable
        // autosave can never overwrite a working project file. (This checks
        // deserialization only, not `load`'s `validate` — `save` is the
        // no-corruption primitive, not a workflow-invariant gate.)
        serde_json::from_str::<Self>(&body).map_err(|e| {
            StudioError::Serialize(format!(
                "refusing to write a project whose JSON would not parse back: {e}"
            ))
        })?;
        let mut tmp = path.to_path_buf().into_os_string();
        tmp.push(".tmp");
        let tmp = PathBuf::from(tmp);
        fs::write(&tmp, body.as_bytes()).map_err(|e| StudioError::Io {
            path: tmp.display().to_string(),
            source: e,
        })?;
        if let Err(e) = fs::rename(&tmp, path) {
            let _ = fs::remove_file(&tmp);
            return Err(StudioError::Io {
                path: path.display().to_string(),
                source: e,
            });
        }
        Ok(())
    }

    /// Load and validate a project from `path`. Rejects a newer schema
    /// version and any project that violates the workflow invariants.
    ///
    /// # Errors
    /// [`StudioError::Io`], [`StudioError::Deserialize`],
    /// [`StudioError::UnsupportedSchema`], or
    /// [`StudioError::InvalidProject`].
    pub fn load(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path).map_err(|e| StudioError::Io {
            path: path.display().to_string(),
            source: e,
        })?;
        let project: Self =
            serde_json::from_str(&text).map_err(|e| StudioError::Deserialize(e.to_string()))?;
        project.validate()?;
        Ok(project)
    }

    /// Check the workflow invariants. Run automatically by
    /// [`Project::load`]; exposed so callers can re-check after any
    /// out-of-band construction.
    ///
    /// # Errors
    /// [`StudioError::UnsupportedSchema`] or [`StudioError::InvalidProject`].
    pub fn validate(&self) -> Result<()> {
        if self.schema_version > PROJECT_SCHEMA_VERSION {
            return Err(StudioError::UnsupportedSchema {
                found: self.schema_version,
                supported: PROJECT_SCHEMA_VERSION,
            });
        }
        // Invariant 1: completed steps form a contiguous prefix.
        let mut hit_incomplete = false;
        for step in Step::ALL {
            if self.is_complete(step) {
                if hit_incomplete {
                    return Err(StudioError::InvalidProject(format!(
                        "step {step:?} is complete but an earlier step is not \
                         (completed steps must be a contiguous prefix)"
                    )));
                }
            } else {
                hit_incomplete = true;
            }
        }
        // Invariant 2: every step before the current one is complete.
        for step in Step::ALL {
            if step < self.current_step && !self.is_complete(step) {
                return Err(StudioError::InvalidProject(format!(
                    "current step {:?} is unreachable: earlier step {step:?} is incomplete",
                    self.current_step
                )));
            }
        }
        Ok(())
    }

    // ── internals ──────────────────────────────────────────────────────

    fn require_complete(&self, step: Step) -> Result<()> {
        if self.is_complete(step) {
            Ok(())
        } else {
            Err(StudioError::StepNotReady {
                from: step,
                reason: format!("\"{}\" must be completed first", step.title()),
            })
        }
    }

    fn clear_artifact(&mut self, step: Step) {
        match step {
            Step::AddScan => self.scan = None,
            Step::CleanScan => self.prep = None,
            Step::DesignLayers => self.design = None,
            Step::MakeMolds => self.molds = None,
            Step::Print => self.print = None,
            Step::Pour => self.pour = None,
        }
    }

    /// Clear every artifact strictly after `step` in the workflow order.
    fn clear_after(&mut self, step: Step) {
        for s in Step::ALL {
            if s > step {
                self.clear_artifact(s);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

    use super::*;

    fn scan() -> ScanInput {
        ScanInput {
            source_path: PathBuf::from("scan.stl"),
        }
    }
    fn prep() -> PrepInput {
        PrepInput {
            cleaned_stl: PathBuf::from("scan.cleaned.stl"),
            prep_toml: PathBuf::from("scan.prep.toml"),
        }
    }
    fn design() -> DesignDraft {
        DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![LayerDraft {
                thickness_m: 0.0175,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            }],
        }
    }
    fn molds() -> MoldOutputs {
        MoldOutputs {
            out_dir: PathBuf::from("out"),
            mold_stls: vec![PathBuf::from("out/mold_layer_0_piece_0.stl")],
            plug_stls: vec![PathBuf::from("out/plug_layer_0.stl")],
            accessory_stls: vec![PathBuf::from("out/platform.stl")],
            procedure_path: PathBuf::from("out/procedure.md"),
            total_mass_g: 842.9,
            pour_plan: PourPlan::default(),
        }
    }
    fn print() -> PrintExport {
        PrintExport {
            export_dir: PathBuf::from("/Desktop/MyPiece"),
        }
    }
    fn pour() -> PourRecord {
        PourRecord { layers_poured: 3 }
    }

    /// Drive a project through every step in order.
    fn fully_completed() -> Project {
        let mut p = Project::new("test");
        p.set_scan(scan());
        p.set_prep(prep()).unwrap();
        p.set_design(design()).unwrap();
        p.set_molds(molds()).unwrap();
        p.set_print(print()).unwrap();
        p.set_pour(pour()).unwrap();
        p
    }

    #[test]
    fn fresh_project_starts_at_add_scan_with_nothing_done() {
        let p = Project::new("fresh");
        assert_eq!(p.current_step(), Step::AddScan);
        assert_eq!(p.furthest_completed(), None);
        assert!(!p.is_finished());
        for step in Step::ALL {
            assert!(!p.is_complete(step));
        }
    }

    #[test]
    fn happy_path_advances_screen_by_screen() {
        let mut p = Project::new("happy");

        p.set_scan(scan());
        assert_eq!(p.advance().unwrap(), Step::CleanScan);

        p.set_prep(prep()).unwrap();
        assert_eq!(p.advance().unwrap(), Step::DesignLayers);

        p.set_design(design()).unwrap();
        assert_eq!(p.advance().unwrap(), Step::MakeMolds);

        p.set_molds(molds()).unwrap();
        assert_eq!(p.advance().unwrap(), Step::Print);

        p.set_print(print()).unwrap();
        assert_eq!(p.advance().unwrap(), Step::Pour);

        p.set_pour(pour()).unwrap();
        assert!(p.is_finished());
        assert_eq!(p.furthest_completed(), Some(Step::Pour));
    }

    #[test]
    fn cannot_advance_past_an_incomplete_step() {
        let mut p = Project::new("gate");
        let err = p.advance().unwrap_err();
        assert!(matches!(
            err,
            StudioError::StepNotReady {
                from: Step::AddScan,
                ..
            }
        ));
    }

    #[test]
    fn cannot_advance_past_the_final_step() {
        let mut p = fully_completed();
        assert_eq!(p.current_step(), Step::Pour);
        assert!(matches!(
            p.advance().unwrap_err(),
            StudioError::AlreadyAtEnd
        ));
    }

    #[test]
    fn setters_require_their_prerequisite() {
        let mut p = Project::new("prereq");
        assert!(matches!(
            p.set_prep(prep()).unwrap_err(),
            StudioError::StepNotReady {
                from: Step::AddScan,
                ..
            }
        ));
        assert!(matches!(
            p.set_design(design()).unwrap_err(),
            StudioError::StepNotReady {
                from: Step::CleanScan,
                ..
            }
        ));
        assert!(matches!(
            p.set_molds(molds()).unwrap_err(),
            StudioError::StepNotReady {
                from: Step::DesignLayers,
                ..
            }
        ));
    }

    #[test]
    fn editing_an_earlier_step_invalidates_downstream() {
        let mut p = fully_completed();
        // Go back and re-pick the layer design.
        p.go_back().unwrap(); // Pour -> Print
        p.go_back().unwrap(); // Print -> MakeMolds
        p.go_back().unwrap(); // MakeMolds -> DesignLayers
        assert_eq!(p.current_step(), Step::DesignLayers);

        p.set_design(design()).unwrap();
        // Everything after the design is now stale and cleared.
        assert!(p.is_complete(Step::DesignLayers));
        assert!(!p.is_complete(Step::MakeMolds));
        assert!(!p.is_complete(Step::Print));
        assert!(!p.is_complete(Step::Pour));
        assert_eq!(p.current_step(), Step::DesignLayers);
        // ...but the scan + prep upstream are untouched.
        assert!(p.is_complete(Step::AddScan));
        assert!(p.is_complete(Step::CleanScan));
    }

    #[test]
    fn re_adding_the_scan_resets_everything_downstream() {
        let mut p = fully_completed();
        p.set_scan(scan());
        assert_eq!(p.current_step(), Step::AddScan);
        assert!(p.is_complete(Step::AddScan));
        for step in [
            Step::CleanScan,
            Step::DesignLayers,
            Step::MakeMolds,
            Step::Print,
            Step::Pour,
        ] {
            assert!(!p.is_complete(step), "{step:?} should have been cleared");
        }
    }

    #[test]
    fn back_then_next_preserves_work() {
        let mut p = fully_completed();
        p.go_back().unwrap(); // -> Print
        assert_eq!(p.advance().unwrap(), Step::Pour); // Print is still complete
        assert!(p.is_finished());
    }

    #[test]
    fn cannot_go_back_from_the_first_step() {
        let mut p = Project::new("start");
        assert!(matches!(
            p.go_back().unwrap_err(),
            StudioError::AlreadyAtStart
        ));
    }

    #[test]
    fn serde_round_trip_preserves_everything() {
        let p = fully_completed();
        let json = serde_json::to_string_pretty(&p).unwrap();
        let back: Project = serde_json::from_str(&json).unwrap();
        assert_eq!(p, back);
        back.validate().unwrap();
    }

    #[test]
    fn save_and_load_round_trip_on_disk() {
        let dir = std::env::temp_dir().join(format!("cf-studio-core-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("project.json");

        let p = fully_completed();
        p.save(&path).unwrap();
        let loaded = Project::load(&path).unwrap();
        assert_eq!(p, loaded);

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir(&dir);
    }

    #[test]
    fn save_refuses_a_non_loadable_project_and_keeps_the_good_file() {
        let dir =
            std::env::temp_dir().join(format!("cf-studio-core-nanguard-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("project.json");

        // A good project is on disk.
        let good = fully_completed();
        good.save(&path).unwrap();

        // A non-finite cast mass serializes to JSON `null`, which won't load
        // back. save() must refuse it (before touching the file) rather than
        // overwrite the working project.
        let mut bad = good.clone();
        bad.molds.as_mut().unwrap().total_mass_g = f64::NAN;
        assert!(
            bad.save(&path).is_err(),
            "save must refuse a project that would not round-trip"
        );
        // The previously-good file is untouched and still loads.
        assert_eq!(Project::load(&path).unwrap(), good);

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir(&dir);
    }

    #[test]
    fn load_rejects_a_future_schema_version() {
        let mut value = serde_json::to_value(fully_completed()).unwrap();
        value["schema_version"] = serde_json::json!(PROJECT_SCHEMA_VERSION + 1);
        let text = serde_json::to_string(&value).unwrap();
        let err = serde_json::from_str::<Project>(&text)
            .map_err(|e| StudioError::Deserialize(e.to_string()))
            .and_then(|p| p.validate().map(|()| p))
            .unwrap_err();
        assert!(matches!(err, StudioError::UnsupportedSchema { .. }));
    }

    #[test]
    fn validate_rejects_a_completion_gap() {
        // molds present but design missing — not a contiguous prefix.
        let mut value = serde_json::to_value(fully_completed()).unwrap();
        value["design"] = serde_json::Value::Null;
        value["current_step"] = serde_json::json!("AddScan"); // keep reachability legal-ish
        let p: Project = serde_json::from_value(value).unwrap();
        let err = p.validate().unwrap_err();
        assert!(matches!(err, StudioError::InvalidProject(_)));
    }

    #[test]
    fn validate_rejects_an_unreachable_current_step() {
        // Only the scan is done, but the user is parked on DesignLayers
        // (which would require CleanScan complete).
        let mut p = Project::new("unreachable");
        p.set_scan(scan());
        let mut value = serde_json::to_value(&p).unwrap();
        value["current_step"] = serde_json::json!("DesignLayers");
        let tampered: Project = serde_json::from_value(value).unwrap();
        let err = tampered.validate().unwrap_err();
        assert!(matches!(err, StudioError::InvalidProject(_)));
    }

    #[test]
    fn a_legal_partial_project_validates() {
        let mut p = Project::new("partial");
        p.set_scan(scan());
        p.set_prep(prep()).unwrap();
        // On CleanScan, design not yet done — perfectly legal.
        p.validate().unwrap();
        assert_eq!(p.current_step(), Step::CleanScan);
        assert_eq!(p.furthest_completed(), Some(Step::CleanScan));
    }
}
