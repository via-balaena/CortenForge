//! `cf-studio-gui` — the polished Bevy + egui GUI for CortenForge Studio.
//!
//! A thin client over [`cf_studio_engine`] + the [`cf_studio_core::Project`]
//! state machine — the same boundary the `cf-studio` CLI drives, so the
//! GUI and CLI behave identically.
//!
//! This lib holds the **headless, testable** part:
//! - [`step_rows`] maps a [`Project`] + the previewed step to the
//!   checklist rows the wizard panel renders;
//! - [`apply_scan`] / [`apply_prep`] / [`apply_design`] run a step's
//!   action against the engine and return a user-facing message (the
//!   GUI's analog of the CLI's `cmd_*`, but file-dialog picking lives in
//!   the binary);
//! - [`nav_state`] computes the gated Back/Next availability;
//! - [`inspect_autosave`] decides what the project file beside a scan has
//!   to say when that scan is picked again.
//!
//! The Bevy app, the egui panels and the file-dialog glue live in the
//! binary's modules (they need a display to *run*, but compile headlessly).

use std::fmt::Write as _;
use std::io::ErrorKind;
use std::path::{Path, PathBuf};
use std::time::Duration;

use cf_studio_core::{
    DesignDraft, LayerDraft, MoldOutputs, PlugDraft, PourPlan, PourStep, Project, RidgeOptions,
    RidgeRing, Step, StudioError,
};
use cf_studio_engine::{
    CastMode, PartId, PartSelection, PieceSide, PlugFit, accept_prep, draft_from_design_toml,
    load_scan, silicone_catalog,
};
pub use cf_studio_engine::{ManifestEntry, RunProvenance, UNKNOWN_RUN};

/// A workflow step as the checklist shows it. `done` / `current` come
/// from the real project; `viewing` is whether this is the step shown in
/// the wizard body right now.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepRow {
    /// One-based step number.
    pub number: i32,
    /// Plain-language step title.
    pub title: String,
    /// Whether the project has completed this step.
    pub done: bool,
    /// Whether this is the project's current (furthest) step.
    pub current: bool,
    /// Whether this is the step currently shown in the wizard body.
    pub viewing: bool,
}

/// Build the [`Step::TOTAL`] checklist rows for `project`, marking `viewed`
/// as the step shown in the body.
#[must_use]
pub fn step_rows(project: &Project, viewed: Step) -> Vec<StepRow> {
    Step::ALL
        .iter()
        .map(|&step| StepRow {
            number: i32::try_from(step.number()).unwrap_or(0),
            title: step.title().to_string(),
            done: project.is_complete(step),
            current: step == project.current_step(),
            viewing: step == viewed,
        })
        .collect()
}

/// Outcome of a step action: a user-facing message. `Ok` is a success
/// line (starts with "✔"); `Err` is the failure message to surface. The
/// frontend decides how to color/show it.
pub type StepOutcome = Result<String, String>;

/// Step 1 action — validate the scan loads + has geometry, then record it.
///
/// # Errors
/// The failure message if the scan is missing, unreadable, or empty.
pub fn apply_scan(project: &mut Project, scan_file: &Path) -> StepOutcome {
    let loaded = load_scan(scan_file).map_err(|e| e.to_string())?;
    let message = format!(
        "✔ Added scan: {} ({} vertices, {} faces)",
        scan_file.display(),
        loaded.vertex_count,
        loaded.face_count
    );
    project.set_scan(loaded.artifact());
    Ok(message)
}

/// Used when a scan's filename yields no usable stem — in practice, one that is
/// not valid UTF-8.
pub const FALLBACK_STEM: &str = "scan";

/// What a session's project file is called, after the scan's own stem.
const PROJECT_SUFFIX: &str = ".cfproject.json";

/// The folder a scan's derived files land in, and the stem they are named
/// after — the scan's own folder and filename, so everything cut from a scan
/// sits beside it.
///
/// ★ One definition, because step 2's cleaned pair and [`autosave_path`] are
/// both named from it. Named apart, a project file could point at a cleaned
/// scan whose name says it came from somewhere else.
#[must_use]
pub fn scan_outputs(source: &Path) -> (PathBuf, String) {
    let dir = source
        .parent()
        .map_or_else(|| PathBuf::from("."), Path::to_path_buf);
    let stem = source
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or(FALLBACK_STEM)
        .to_owned();
    (dir, stem)
}

/// Where a session working from `source` saves its project.
#[must_use]
pub fn autosave_path(source: &Path) -> PathBuf {
    let (dir, stem) = scan_outputs(source);
    dir.join(format!("{stem}{PROJECT_SUFFIX}"))
}

/// How far a project has got, when that is further than the scan pick itself —
/// the one test for "there is something here worth keeping".
///
/// ★★★ One definition, used both ways round. A session below this bar is not
/// worth a file, and a file below it is not worth a question. Two definitions
/// would let the writer drop a project file beside every scan the user ever
/// clicked, each one the reader then ignores.
#[must_use]
pub fn worth_keeping(project: &Project) -> Option<Step> {
    project
        .furthest_completed()
        .filter(|&reached| reached > Step::AddScan)
}

/// What the project file beside a scan has to say when that scan is picked.
#[derive(Debug, Clone, PartialEq)]
pub enum ResumeOffer {
    /// Nothing worth asking about: no file, or one recording only the pick
    /// itself. Saving may start straight away.
    Fresh,
    /// A session to offer back.
    Resumable {
        /// The saved project, carried **whole** — "resume" has to produce this,
        /// and re-reading the file after the question was answered would let it
        /// change under the answer.
        project: Box<Project>,
        /// How far it got. Resolved here, where the test for "worth asking
        /// about" is actually made, so no consumer has to re-derive it and find
        /// the `None` this variant already rules out.
        reached: Step,
    },
    /// A file this build cannot read: hand-edited, or written by a newer
    /// Cendrillon. The string is why, for the screen.
    Unreadable(String),
}

/// Read the project file at `path`, and decide what to offer.
///
/// ★★★ Call this **before** the session's first write to `path`. A write first
/// lands this session's empty project on the very file being read, and there is
/// nothing left to offer.
#[must_use]
pub fn inspect_autosave(path: &Path) -> ResumeOffer {
    match Project::load(path) {
        // ⚠ A scan picked and abandoned records nothing worth a question, and
        // asking about it would put a modal in front of an ordinary re-pick.
        Ok(project) => match worth_keeping(&project) {
            Some(reached) => ResumeOffer::Resumable {
                project: Box::new(project),
                reached,
            },
            None => ResumeOffer::Fresh,
        },
        // ⚠ Asked of the error rather than of `path.exists()`: two reads can
        // disagree, and the one that says "unreadable" about a file that is not
        // there would stop the session saving at all.
        Err(StudioError::Io { source, .. }) if source.kind() == ErrorKind::NotFound => {
            ResumeOffer::Fresh
        }
        Err(e) => ResumeOffer::Unreadable(e.to_string()),
    }
}

/// The resume question, naming how far the saved session got.
#[must_use]
pub fn format_resume_question(reached: Step) -> String {
    format!(
        "A saved session for this scan is already on disk. It got as far as \
         step {} — {}.\n\nPick up to carry on from that work. Start over to \
         leave it behind — this session takes its place.",
        reached.number(),
        reached.title(),
    )
}

/// Step 2 action — accept a cleaned scan + its `.prep.toml`.
///
/// # Errors
/// The failure message if the prep is invalid or the scan step isn't done.
pub fn apply_prep(project: &mut Project, cleaned_stl: &Path, prep_toml: &Path) -> StepOutcome {
    let prep = accept_prep(cleaned_stl, prep_toml).map_err(|e| e.to_string())?;
    project.set_prep(prep).map_err(|e| e.to_string())?;
    Ok("✔ Accepted cleaned scan + prep.".to_string())
}

/// Step 4 action — load a layer design from a `.design.toml`.
///
/// # Errors
/// The failure message if the design is invalid or the scan isn't cleaned.
pub fn apply_design(project: &mut Project, design_toml: &Path) -> StepOutcome {
    let draft = draft_from_design_toml(design_toml).map_err(|e| e.to_string())?;
    let from_file = draft.cavity_inset_m;
    let message = apply_design_draft(project, draft.layers)?;
    let used = project.design().map_or(from_file, |set| set.cavity_inset_m);
    Ok(message + &format_ignored_inset(from_file, used).unwrap_or_default())
}

/// What to add when a loaded design named a cavity inset this app did not use.
///
/// ⚠ Compared **as the message prints them**, not as `f64`s. A `.design.toml`
/// carries the inset to 1 µm and other tools write it from float sliders, so an
/// exact `!=` fires on differences no one can see — and the note would then
/// read "Its 5.0 mm cavity inset was not used" directly after "5.0 mm cavity
/// inset". A difference the user cannot see is not one to explain.
#[must_use]
pub fn format_ignored_inset(from_file_m: f64, used_m: f64) -> Option<String> {
    (shown_inset_mm(from_file_m) != shown_inset_mm(used_m)).then(|| {
        format!(
            " \u{26a0} Its {} mm cavity inset was not used \u{2014} the piece you shaped \
             on step 3 sets that.",
            shown_inset_mm(from_file_m),
        )
    })
}

/// A cavity inset in millimetres, as every message about one prints it.
///
/// ⚠ One definition because [`format_ignored_inset`] *compares* insets at this
/// precision and both messages *print* at it. Split, a precision changed on one
/// side brings back a note that fires while showing the same number twice.
fn shown_inset_mm(inset_m: f64) -> String {
    format!("{:.1}", inset_m * 1000.0)
}

/// Step 4 action — set the layer stack, however it was arrived at: built in the
/// editor, or read out of a `.design.toml`.
///
/// ★ The cavity inset is **not** a parameter. It belongs to the piece step 3
/// shaped — that piece is what the preview draws and what "Check fit" gave its
/// verdict on, and `start_molds` casts the design. A caller allowed to supply
/// its own would be allowed to leave `Project::plug` and `Project::design`
/// disagreeing, and the mould would then not be the cavity the user was shown.
/// One funnel, so no caller can.
///
/// # Errors
/// The failure message if the design is invalid or the scan isn't cleaned.
pub fn apply_design_draft(project: &mut Project, layers: Vec<LayerDraft>) -> StepOutcome {
    let draft = DesignDraft {
        cavity_inset_m: project.plug().map_or(0.0, |plug| plug.cavity_inset_m),
        layers,
    };
    let message = format!(
        "✔ Design set: {} layer(s), {} mm cavity inset.",
        draft.layers.len(),
        shown_inset_mm(draft.cavity_inset_m),
    );
    project.set_design(draft).map_err(|e| e.to_string())?;
    Ok(message)
}

/// Commit the shaped plug ([`cf_studio_core::Step::ShapePiece`]) — the cavity
/// inset (snugness) + the surface ridges, tuned against the live preview. A
/// default [`cf_studio_core::PlugDraft`] (zero inset, ridges off) is the
/// smooth, snug-fit baseline. The ridges ride every offset, so the same field
/// shapes the plug and every shell at a constant wall.
///
/// # Errors
/// Surfaces [`cf_studio_core::StudioError`] as a string if the scan has not
/// been cleaned.
pub fn apply_plug(project: &mut Project, plug: cf_studio_core::PlugDraft) -> StepOutcome {
    let message = format!(
        "✔ Shaped piece: {:.1} mm inset{}.",
        plug.cavity_inset_m * 1000.0,
        if plug.ridges.enabled {
            ", ridges on"
        } else {
            ", no ridges"
        }
    );
    project.set_plug(plug).map_err(|e| e.to_string())?;
    Ok(message)
}

/// The "Shape your piece" ridge controls, read off the UI in SI units, plus
/// the per-feature toggles. Pure input to [`gate_ridge_options`] (the UI/model
/// reads live in `main.rs`; this keeps the gating logic testable).
#[derive(Debug, Clone, PartialEq)]
pub struct RidgeControls {
    /// Master toggle (the whole ridge feature).
    pub enabled: bool,
    /// Grip rings on/off + the ring set.
    pub rings_enabled: bool,
    /// The ring set (already in SI units).
    pub rings: Vec<RidgeRing>,
    /// Surface texture on/off + its depth / spacing (meters).
    pub texture_enabled: bool,
    pub texture_depth_m: f64,
    pub texture_spacing_m: f64,
    /// Side pinch on/off + depth (meters).
    pub side_pinch_enabled: bool,
    pub side_pinch_depth_m: f64,
    /// Tip relief on/off + depth (meters).
    pub tip_relief_enabled: bool,
    pub tip_relief_depth_m: f64,
    /// Feature orientation on/off + angle (degrees).
    pub orientation_enabled: bool,
    pub orientation_deg: f64,
}

/// Apply the per-feature toggles to build the owned [`RidgeOptions`]: a feature
/// that is OFF contributes nothing (rings emptied / depth `0.0` / orientation
/// `0°`), so the user can mix and match (e.g. grip rings without the fine
/// texture). The SAME gated value feeds the live preview and the cast, so what
/// you toggle is what gets cut.
#[must_use]
pub fn gate_ridge_options(c: RidgeControls) -> RidgeOptions {
    RidgeOptions {
        enabled: c.enabled,
        rings: if c.rings_enabled { c.rings } else { Vec::new() },
        texture_depth_m: if c.texture_enabled {
            c.texture_depth_m
        } else {
            0.0
        },
        texture_spacing_m: c.texture_spacing_m,
        side_pinch_depth_m: if c.side_pinch_enabled {
            c.side_pinch_depth_m
        } else {
            0.0
        },
        tip_relief_depth_m: if c.tip_relief_enabled {
            c.tip_relief_depth_m
        } else {
            0.0
        },
        orientation_deg: if c.orientation_enabled {
            c.orientation_deg
        } else {
            0.0
        },
    }
}

/// Marching-cubes cell size (meters) for the step-5 quality-picker index.
/// Index 0 = Fine 0.5 mm (the print-quality default — the physical fit-test
/// print was 0.5 mm); index 1 = Fast 1.5 mm preview. Any other index falls
/// back to the print-quality default. 3 mm is never offered (it drops the
/// flange web). **Must stay in lockstep with the quality picker's option order
/// in the step-5 panel.** Nothing checks that pairing; only the index→size
/// mapping is pinned, by `quality_index_maps_to_cell_size`.
#[must_use]
pub fn cell_size_m_for_quality(quality_idx: i32) -> f64 {
    match quality_idx {
        1 => 0.0015,
        _ => 0.0005,
    }
}

/// The cast mode Cendrillon casts in.
///
/// Bonded drops every plug above layer 0 and stops `part_selection_from_checks`
/// collapsing to `PartSelection::all`. ⚠ The pre-port binary pinned this and the
/// port dropped it; the gates below read this constant, not a `CastMode`
/// literal.
pub const CENDRILLON_CAST_MODE: CastMode = CastMode::Bonded;

/// Enumerate the generatable parts for a design with `layer_count` layers,
/// in display order, as `(id, label)`. Per layer: two cup halves + a plug,
/// then the shared workshop platform + dowels (the apex pour funnel is
/// integral, and the gasket is off, so neither is offered).
///
/// In [`CastMode::Bonded`] only the **layer-0** plug is offered — the
/// per-layer plugs above 0 are redundant (the cured layer N is the plug for
/// layer N+1), so they are not listed (or generated). The step-5 part picker
/// renders the labels; the ids build the [`PartSelection`].
#[must_use]
pub fn enumerate_parts(layer_count: usize, mode: CastMode) -> Vec<(PartId, String)> {
    let mut parts = Vec::with_capacity(layer_count * 3 + 2);
    for i in 0..layer_count {
        let n = i + 1;
        parts.push((
            PartId::Cup {
                layer_index: i,
                side: PieceSide::Negative,
            },
            format!("Layer {n} — cup (left)"),
        ));
        parts.push((
            PartId::Cup {
                layer_index: i,
                side: PieceSide::Positive,
            },
            format!("Layer {n} — cup (right)"),
        ));
        // Bonded casts with one plug (layer 0); detachable prints one per layer.
        if mode == CastMode::Detachable || i == 0 {
            parts.push((PartId::Plug { layer_index: i }, format!("Layer {n} — plug")));
        }
    }
    parts.push((PartId::Platform, "Platform".to_string()));
    parts.push((PartId::Dowel, "Dowels".to_string()));
    parts
}

/// Build a [`PartSelection`] from the enumerated `parts` and a parallel
/// `checked` mask.
///
/// In [`CastMode::Detachable`], "everything checked" returns
/// [`PartSelection::all`] — the validated full-cast path. In
/// [`CastMode::Bonded`] it always selects exactly the checked parts (never
/// `all`), so the cast routes through the selective + bonded-procedure path
/// (and `parts` already omits the redundant plugs).
#[must_use]
pub fn part_selection_from_checks(
    parts: &[(PartId, String)],
    checked: &[bool],
    mode: CastMode,
) -> PartSelection {
    let all_checked = checked.len() == parts.len() && checked.iter().all(|&c| c);
    if all_checked && mode == CastMode::Detachable {
        PartSelection::all()
    } else {
        PartSelection::from_ids(
            parts
                .iter()
                .enumerate()
                .filter(|(i, _)| checked.get(*i).copied().unwrap_or(false))
                .map(|(_, (id, _))| *id),
        )
    }
}

/// A human-readable summary of a completed mold run for the step-5 results
/// panel: piece counts, total silicone, the per-layer pour list, and where
/// the files landed.
#[must_use]
pub fn format_molds_summary(out: &MoldOutputs) -> String {
    let mut s = format!(
        "✔ {} mold piece(s) + {} plug(s)",
        out.mold_stls.len(),
        out.plug_stls.len(),
    );
    if !out.accessory_stls.is_empty() {
        let _ = write!(s, " + {} accessory part(s)", out.accessory_stls.len());
    }
    let _ = write!(
        s,
        "\nTotal silicone: {:.0} g across {} pour(s):",
        out.total_mass_g,
        out.pour_plan.steps.len(),
    );
    for step in &out.pour_plan.steps {
        let _ = write!(
            s,
            "\n  • Layer {}: {} — {:.0} g (pot life ~{} min)",
            step.layer_index + 1,
            step.material_display_name,
            step.mass_g,
            step.pot_life_minutes,
        );
    }
    let _ = write!(s, "\nSaved to: {}", out.out_dir.display());
    s
}

/// One line per not-regenerated part, naming the run that wrote it.
///
/// A part at [`UNKNOWN_RUN`] was already in the folder when the first
/// manifest was written: it is of unknown vintage, not merely older, and the
/// line distinguishes the two because deleting one is a different decision.
fn stale_bullets(stale: &[ManifestEntry]) -> String {
    let mut s = String::new();
    for entry in stale {
        if entry.run == UNKNOWN_RUN {
            let _ = write!(s, "\n  • {} — never recorded", entry.file);
        } else {
            let _ = write!(s, "\n  • {} — run {}", entry.file, entry.run);
        }
    }
    s
}

/// The note about what the cast's output folder holds **besides** the latest
/// cast's output, or `None` when there is nothing worth saying.
///
/// ⚠⚠ `out_dir` is named in the message, and that is the whole reason it is a
/// parameter. This is shown on **two** steps: on step 5 it sits under a card
/// that already says where the cast landed, but on step 6 the screen is about
/// a print folder the user picked from a dialog, while this roster is still
/// read from the CAST's folder. A sentence saying "this folder" is true on one
/// step and a claim about the wrong directory on the other. Naming the
/// directory makes the referent impossible to mistake on either.
///
/// ⚠ Call only where a cast has been recorded — the panel gates this on the
/// same `project.molds()` the summary above it uses. A `None` `provenance`
/// then means **unknown**, not clean, and says so rather than staying quiet.
/// It does not say WHY: `folder_provenance` returns `None` down four
/// different paths, and three of them have their own gates.
#[must_use]
pub fn format_stale_parts(provenance: Option<&RunProvenance>, out_dir: &Path) -> Option<String> {
    let dir = out_dir.display();
    let Some(p) = provenance else {
        return Some(format!(
            "⚠ {dir} carries no record of which parts are current. \
             The next cast will record it."
        ));
    };
    if p.stale.is_empty() {
        return None;
    }
    let mut s = format!(
        "⚠ {n} part(s) in {dir} were NOT regenerated by the latest cast \
         (run {run}) — check these before printing:",
        n = p.stale.len(),
        run = p.run,
    );
    s.push_str(&stale_bullets(&p.stale));
    Some(s)
}

/// What the print destination holds once a save is done, or `None` when it
/// holds exactly what that save wrote.
///
/// The destination is never cleared, so it ends up holding the union of what
/// was already there and what was just copied: `dest_stl_count` exceeding
/// `stl_count` is the exact test for "there is a printable in this folder
/// that this save did not write".
///
/// ⚠ A `None` `dest_stl_count` means the folder could not be read back. Say
/// nothing — it is not a zero, and guessing here would put a made-up count in
/// front of someone about to spend hours of filament.
#[must_use]
pub fn format_print_destination_note(
    stl_count: usize,
    dest_stl_count: Option<usize>,
) -> Option<String> {
    let total = dest_stl_count?;
    let extra = total.checked_sub(stl_count).filter(|n| *n > 0)?;
    Some(format!(
        "⚠ That folder now holds {total} printable file(s) — {extra} left by an \
         earlier save, which this one did not replace."
    ))
}

/// The step-6 (Print) status line, derived from project state: once the
/// files are exported, where they went; before that, how many printables
/// are waiting to be saved; nothing if the molds aren't made yet.
#[must_use]
pub fn print_step_summary(project: &Project) -> String {
    if let Some(export) = project.print() {
        return format!(
            "✔ Saved to {} — open that folder in your slicer to print each piece.",
            export.export_dir.display()
        );
    }
    if let Some(molds) = project.molds() {
        let pieces = molds.mold_stls.len() + molds.plug_stls.len() + molds.accessory_stls.len();
        return format!(
            "Ready to save the {pieces} part(s) this cast made + the step-by-step guide."
        );
    }
    String::new()
}

// ── step 7: the pour assistant ──────────────────────────────────────

/// One pour layer's full recipe line. Without Slacker, e.g.
/// `"Dragon Skin 20A — 250 g, mix 1:1 · pot life ~25 min · cure ~5 h"`. With
/// Slacker, the cavity-fill mass is broken into the corrected mix —
/// `"Ecoflex 00-30 — 200 g A + 200 g B + 100 g Slacker = 500 g mix (25% of base)
/// · pot life ~25 min · cure ~4 h"` — so the base is scaled DOWN to share the
/// cavity with the Slacker (NOT a full base pour with Slacker added on top). The
/// cure is omitted for the last layer (nothing waits on it).
#[must_use]
fn pour_recipe_line(step: &PourStep, is_last: bool) -> String {
    let cure = if is_last {
        String::new()
    } else {
        format!(" · cure ~{:.0} h", step.cure_time_hours)
    };
    // `mass_g` is the cavity-fill TOTAL mix mass (base density ≈ base+Slacker mix
    // density — Slacker publishes no density). For a Slacker layer, split it so
    // base + Slacker = mass_g: base = mass_g/(1+sf) = A + B (1:1, the ratio for
    // every Slacker-compatible Smooth-On platinum silicone), Slacker = sf·base by
    // weight. Without Slacker, show the mass + A:B ratio as before.
    let recipe = match step.slacker_fraction {
        Some(sf) => {
            let base = step.mass_g / (1.0 + sf);
            let part = base / 2.0;
            let slacker = sf * base;
            format!(
                "{part:.0} g A + {part:.0} g B + {slacker:.0} g Slacker = {:.0} g mix ({:.0}% of base)",
                step.mass_g,
                sf * 100.0,
            )
        }
        None => format!("{:.0} g, mix {}", step.mass_g, step.mix_ratio_a_to_b),
    };
    format!(
        "{} — {recipe} · pot life ~{} min{cure}",
        step.material_display_name, step.pot_life_minutes,
    )
}

/// The full pour plan as a numbered overview (innermost layer first), for
/// the step-7 reference panel. Empty plan → a short placeholder.
#[must_use]
pub fn format_pour_plan(plan: &PourPlan) -> String {
    if plan.steps.is_empty() {
        return "(no pour layers)".to_string();
    }
    let last = plan.steps.len() - 1;
    let mut s = format!(
        "Pour plan — {} layer(s), innermost first:",
        plan.steps.len()
    );
    for (i, step) in plan.steps.iter().enumerate() {
        let _ = write!(s, "\n  {}. {}", i + 1, pour_recipe_line(step, i == last));
    }
    s
}

/// The active-layer instruction for the step-7 pour panel: which layer of
/// how many, its recipe, and what to do. `current` is 0-based.
#[must_use]
pub fn format_pour_active(plan: &PourPlan, current: usize) -> String {
    let Some(step) = plan.steps.get(current) else {
        return String::new();
    };
    let is_last = current + 1 == plan.steps.len();
    format!(
        "Layer {} of {} — {}\nMix it, start the timer, and pour before the working time runs out.",
        current + 1,
        plan.steps.len(),
        pour_recipe_line(step, is_last),
    )
}

/// A pot-life countdown's display text + urgency for the step-7 timer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PourCountdown {
    /// `"M:SS left"`, or a "time's up" line once expired.
    pub text: String,
    /// 0 = comfortable, 1 = warning (≤ 5 min left), 2 = expired (≤ 0).
    pub urgency: i32,
}

/// A pot life as a [`Duration`] — the working time the step-7 countdown runs.
///
/// ⚠ Extracted from the timer's call site so the minutes→seconds conversion is
/// reachable from a test. Mutation testing showed the `* 60` was killed by
/// nothing: a wrong factor here does not *look* wrong, it silently gives a
/// 25-SECOND working time for a 25-minute silicone, and the user finds out when
/// the pour sets in the cup.
#[must_use]
pub fn pot_life_duration(minutes: u32) -> Duration {
    Duration::from_secs(u64::from(minutes) * 60)
}

/// Format a pot-life countdown from the seconds remaining (negative or zero
/// = expired). Warns under five minutes.
#[must_use]
pub fn pour_countdown(remaining_secs: i64) -> PourCountdown {
    if remaining_secs <= 0 {
        return PourCountdown {
            text: "0:00 — working time's up. Pour now, or scrape and remix.".to_string(),
            urgency: 2,
        };
    }
    let text = format!(
        "⏱ {}:{:02} of working time left",
        remaining_secs / 60,
        remaining_secs % 60
    );
    PourCountdown {
        text,
        urgency: i32::from(remaining_secs <= 300),
    }
}

/// Whether Back/Next are available from the `viewed` screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavState {
    /// Back is available on any screen but the first.
    pub can_back: bool,
    /// Next is available once the viewed step is complete (and it isn't
    /// the last) — the wizard gate that stops you skipping ahead.
    pub can_next: bool,
}

/// Compute the gated navigation state for the `viewed` screen.
#[must_use]
pub fn nav_state(project: &Project, viewed: Step) -> NavState {
    NavState {
        can_back: viewed != Step::FIRST,
        can_next: project.is_complete(viewed) && viewed != Step::LAST,
    }
}

/// The wizard's view cursor — which screen the user is *looking at*.
///
/// ⚠ This is **not** `Project::current_step()`. The two are independent and both
/// live: Back moves the cursor without touching the project, so you can page back
/// over completed work and return. The project's own step only moves when a step
/// is *completed*. Conflating them is the bug this type exists to make impossible.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WizardCursor {
    viewed: Step,
}

impl Default for WizardCursor {
    fn default() -> Self {
        Self {
            viewed: Step::FIRST,
        }
    }
}

impl WizardCursor {
    /// A cursor parked on `viewed`.
    #[must_use]
    pub const fn new(viewed: Step) -> Self {
        Self { viewed }
    }

    /// The screen currently shown.
    #[must_use]
    pub const fn viewed(self) -> Step {
        self.viewed
    }

    /// Page back one screen. A no-op on the first — the Slint original used
    /// `saturating_sub`, and `Step::prev()` returning `None` is the same
    /// contract without the index arithmetic.
    pub const fn back(&mut self) {
        if let Some(prev) = self.viewed.prev() {
            self.viewed = prev;
        }
    }

    /// Page forward, **gated on [`nav_state`]**.
    ///
    /// ⚠ The gate is re-checked here rather than trusted to the disabled button.
    /// The Slint original carried the same belt-and-braces check with the comment
    /// "Respect the gate even if the disabled button somehow fires" — an immediate-
    /// mode UI makes that failure *more* likely, not less, because a stale frame
    /// can deliver a click against last frame's enablement.
    ///
    /// Returns `true` if the cursor actually moved.
    pub fn next(&mut self, project: &Project) -> bool {
        if !nav_state(project, self.viewed).can_next {
            return false;
        }
        match self.viewed.next() {
            Some(n) => {
                self.viewed = n;
                true
            }
            None => false,
        }
    }
}

/// What [`PourSession::advance`] wants the caller to do next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PourAdvance {
    /// No pour plan (or an empty one) — the button should do nothing at all.
    NoPlan,
    /// Moved on to another layer; `poured` layers are now done.
    Layer { poured: usize },
    /// The last layer was poured — the caller must record completion on the
    /// `Project` via `set_pour(PourRecord { layers_poured })`.
    Complete { layers_poured: usize },
}

/// Which layer the pour assistant is working on, within one session.
///
/// ⚠ **Session-only, deliberately.** `Project` records only the *final*
/// completion, so this cursor does not survive a restart. See the autosave work:
/// resuming mid-pour returns you to layer 1. Extending `Project` to carry it is a
/// schema change, not a UI change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PourSession {
    current: usize,
}

impl PourSession {
    /// The 0-based layer being poured.
    #[must_use]
    pub const fn current(self) -> usize {
        self.current
    }

    /// Mark the current layer poured and step on.
    ///
    /// `total` is the pour plan's step count; **zero is a no-op**, matching the
    /// Slint original's early return — a project with no molds has no plan, and
    /// the button must not advance a cursor into a plan that does not exist.
    pub const fn advance(&mut self, total: usize) -> PourAdvance {
        if total == 0 {
            return PourAdvance::NoPlan;
        }
        self.current += 1;
        if self.current >= total {
            PourAdvance::Complete {
                layers_poured: total,
            }
        } else {
            PourAdvance::Layer {
                poured: self.current,
            }
        }
    }
}

/// Elapsed time for a long job's status line, as `M:SS`.
///
/// Lifted from an inline `format!` so the long-job status text is testable; the
/// jobs it labels run 7–36 minutes, so the minutes field is the part that
/// matters and the one an off-by-one would hide.
#[must_use]
pub fn format_elapsed(secs: u64) -> String {
    format!("{}:{:02}", secs / 60, secs % 60)
}

/// The status line while the cast runs, refreshed once a second.
///
/// ⚠ Single-spaced before the parenthesis; the pre-port built this with a
/// trailing `\` continuation and emitted two spaces.
#[must_use]
pub fn format_molds_progress(secs: u64) -> String {
    format!(
        "Making molds… {} elapsed (this can take a while — the window stays responsive)",
        format_elapsed(secs),
    )
}

/// A numeric field's edit state — the toolkit-agnostic half of the stepper.
///
/// ## Why this is not just an `i32`
///
/// The Slint `StepBox` component encoded four rules that a naive spinner gets
/// wrong, and all four are load-bearing because **committing re-meshes the
/// preview** (hundreds of milliseconds). Getting them wrong is not a cosmetic
/// bug; it is a stutter on every keystroke.
///
/// 1. **Typing does not clamp.** Clamping mid-type snaps the field to the bound
///    the moment you overshoot, which is jarring while you are still typing the
///    second digit of `25`. Clamping happens at commit.
/// 2. **Typing does not commit.** Only Enter, the ± buttons, or losing focus do.
/// 3. **Blur commits only if dirty.** Clicking into a field and out again must
///    not trigger a re-mesh. This is why `dirty` exists rather than comparing
///    values — a user who types `30` over `30` has changed nothing, and the
///    value comparison in [`Self::commit`] catches that too.
/// 4. **Empty text keeps the old value.** Clearing the box mid-edit must not be
///    read as zero.
///
/// ⚠ Decimal text is rejected rather than truncated: parsing is `parse::<i32>()`
/// and anything it rejects — `"3.7"` — leaves the old value standing. The Slint
/// original accepted that via `to-float()` into an `int`, so the two differ for
/// such input. With Slint gone this is simply the behaviour, not an open
/// question.
///
/// ⚠ **Store one of these per row, inside the row struct** — never in a `Vec`
/// keyed by row index. Removing ring #1 must take ring #1's uncommitted text with
/// it; index-keyed state would leave it bound to what is now ring #1 (formerly
/// #2). Slint's per-row widgets made this impossible; an immediate-mode UI does
/// not, so the invariant has to live in the data layout.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepBoxState {
    text: String,
    /// The live parsed value. May be out of range while typing (rule 1).
    value: i32,
    /// The last value actually committed. ⚠ Tracked separately because
    /// `on_typed` updates `value` eagerly, so a commit that compared `value`
    /// before and after clamping could never detect a change — it would compare
    /// the typed value against itself and report "nothing happened" for every
    /// in-range edit. The first version of this type had exactly that bug.
    committed: i32,
    dirty: bool,
}

impl StepBoxState {
    /// A field showing `value`, clean.
    #[must_use]
    pub fn new(value: i32) -> Self {
        Self {
            text: value.to_string(),
            value,
            committed: value,
            dirty: false,
        }
    }

    /// The last committed value.
    #[must_use]
    pub const fn value(&self) -> i32 {
        self.value
    }

    /// Whether there is an uncommitted edit pending.
    #[must_use]
    pub const fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// The text buffer the text widget binds to.
    pub const fn text_mut(&mut self) -> &mut String {
        &mut self.text
    }

    /// The text buffer, for rendering.
    #[must_use]
    pub fn text(&self) -> &str {
        &self.text
    }

    /// Call after the text widget reports a change. Parses without clamping
    /// (rule 1) and marks the edit pending (rule 2).
    ///
    /// Unparseable or empty text leaves `value` alone (rule 4) but still marks
    /// dirty, so that blurring an empty box re-renders the old value rather than
    /// leaving the box blank.
    pub fn on_typed(&mut self) {
        if let Ok(v) = self.text.trim().parse::<i32>() {
            self.value = v;
        }
        self.dirty = true;
    }

    /// Commit an edit: clamp, clear the pending flag, re-render the text.
    ///
    /// Returns `Some(value)` **only when a commit actually changed something** —
    /// a clean blur returns `None` (rule 3), and so does re-typing the same
    /// number. The caller uses that to decide whether to re-mesh.
    /// ⚠ There is deliberately **no** `if !dirty { return None }` early exit.
    /// It would be dead logic: `value != committed` implies `dirty` on every
    /// path (`new`, `step`, `commit` and `sync_external` all set the two equal),
    /// so the comparison below already covers the clean case. A mutation test
    /// removing that guard passed the whole suite, which is what surfaced it.
    /// Rule 4 also needs this method to actually RUN on a dirty-but-unchanged
    /// field, so it can refill a box the user emptied.
    pub fn commit(&mut self, min: i32, max: i32) -> Option<i32> {
        self.value = self.value.clamp(min, max);
        self.text = self.value.to_string();
        self.dirty = false;
        let changed = self.value != self.committed;
        self.committed = self.value;
        changed.then_some(self.value)
    }

    /// The ± buttons. Steps by `delta`, clamps, and commits in one action.
    pub fn step(&mut self, delta: i32, min: i32, max: i32) -> Option<i32> {
        self.value = self.value.saturating_add(delta).clamp(min, max);
        self.text = self.value.to_string();
        self.dirty = false;
        let changed = self.value != self.committed;
        self.committed = self.value;
        changed.then_some(self.value)
    }

    /// The model changed underneath — e.g. a trim bound shrank because the mesh
    /// did. Discards any pending edit; the model wins.
    ///
    /// ⚠ **Clamps**, and takes the bounds for that reason. The scenario this
    /// method exists for is the one where the bounds themselves moved, so the
    /// incoming value can be outside them. Without the clamp the field would show
    /// an out-of-range number that [`Self::commit`] could never correct, because
    /// a clean field returns early — the value would only be fixed if the user
    /// happened to edit that field again.
    pub fn sync_external(&mut self, value: i32, min: i32, max: i32) {
        self.value = value.clamp(min, max);
        self.committed = self.value;
        self.text = self.value.to_string();
        self.dirty = false;
    }
}

/// A stepper field and the bounds it is committed inside.
///
/// ⚠ The two travel together because the range is needed twice — once to draw
/// the field, once to read it — and a field drawn against one range and read
/// against another is wrong in neither half alone.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoundedField {
    /// The text and value the stepper edits.
    pub state: StepBoxState,
    /// `(min, max)`, in the field's own unit.
    pub range: (i32, i32),
}

impl BoundedField {
    /// A field showing `value`, editable within `range`.
    #[must_use]
    pub fn new(value: i32, range: (i32, i32)) -> Self {
        Self {
            state: StepBoxState::new(value),
            range,
        }
    }

    /// A field showing `value` pulled inside `range`.
    ///
    /// ⚠ For reading a committed artifact back into the screen that edits it.
    /// The stepper prints the number it is handed, so an artifact from outside
    /// the range has to be clamped on the way in — [`Self::value`] clamps on
    /// read, the text does not.
    #[must_use]
    pub fn clamped(value: i32, range: (i32, i32)) -> Self {
        let (min, max) = range;
        Self::new(value.clamp(min, max), range)
    }

    /// The value, inside its bounds.
    ///
    /// ⚠ Clamped here because typing does not commit: a number typed and left
    /// uncommitted reaches this unclamped. See [`StepBoxState`].
    #[must_use]
    pub fn value(&self) -> i32 {
        let (min, max) = self.range;
        self.state.value().clamp(min, max)
    }
}

/// Axial position: 0 = opening … 100 = deep end.
const RING_POSITION_RANGE: (i32, i32) = (0, 100);
/// Inward pinch depth, tenths of a millimetre.
const RING_DEPTH_RANGE: (i32, i32) = (0, 100);
/// Half-width of the ring's axial support, percent. ⚠ One, not zero: a ring of
/// no width is not a ring.
const RING_WIDTH_RANGE: (i32, i32) = (1, 50);

/// One grip ring in the "Shape your piece" editor, in the integer units the UI
/// edits (percent, tenths of a millimetre) rather than the SDK's meters.
///
/// The three fields live **inside the row** on purpose — see the warning on
/// [`StepBoxState`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RingRow {
    /// Where the ring sits along the channel.
    pub position: BoundedField,
    /// How far it pinches inward.
    pub depth: BoundedField,
    /// How much of the channel it spans.
    pub width: BoundedField,
}

impl RingRow {
    /// A row at the UI's own units, inside the bounds its steppers offer —
    /// which it is held to: the stepper renders the number it is given, so a
    /// row outside the range would print a value the piece is not cut at.
    #[must_use]
    pub fn new(position_pct: i32, depth_tenths_mm: i32, width_pct: i32) -> Self {
        Self {
            position: BoundedField::clamped(position_pct, RING_POSITION_RANGE),
            depth: BoundedField::clamped(depth_tenths_mm, RING_DEPTH_RANGE),
            width: BoundedField::clamped(width_pct, RING_WIDTH_RANGE),
        }
    }

    /// Build a row from an owned SDK ring (meters/fractions → integer UI units).
    #[must_use]
    pub fn from_ridge(ring: &RidgeRing) -> Self {
        Self::new(
            scale_to_i32(ring.position_frac, 100.0),
            m_to_tenths_mm(ring.depth_m),
            scale_to_i32(ring.half_width_frac, 100.0),
        )
    }

    /// The inverse: integer UI units → an owned SDK ring.
    ///
    /// ⚠ This round-trip was untested before the toolkit split: only the
    /// `RidgeRing → RingRow` direction had a function, and the reverse was
    /// inlined in a Slint closure reading `&AppWindow`.
    #[must_use]
    pub fn to_ridge(&self) -> RidgeRing {
        RidgeRing {
            position_frac: f64::from(self.position.value()) / 100.0,
            depth_m: tenths_mm_to_m(self.depth.value()),
            half_width_frac: f64::from(self.width.value()) / 100.0,
        }
    }
}

/// Tenths of a millimetre → meters. The UI's depth unit throughout.
#[must_use]
pub fn tenths_mm_to_m(tenths: i32) -> f64 {
    f64::from(tenths) / 10_000.0
}

/// Meters → tenths of a millimetre. The inverse of [`tenths_mm_to_m`], for
/// reading a committed artifact back into the fields that produced it.
#[must_use]
pub fn m_to_tenths_mm(m: f64) -> i32 {
    scale_to_i32(m, 10_000.0)
}

/// Meters → whole millimetres.
#[must_use]
pub fn m_to_mm(m: f64) -> i32 {
    scale_to_i32(m, 1000.0)
}

/// An angle → the whole degrees the orientation stepper edits.
#[must_use]
pub fn whole_degrees(deg: f64) -> i32 {
    scale_to_i32(deg, 1.0)
}

/// A fraction/length → the UI's integer unit, rounded.
fn scale_to_i32(value: f64, scale: f64) -> i32 {
    // Float -> int `as` has saturated (and mapped NaN to 0) since Rust 1.45; the
    // workspace MSRV is 1.92, so this is guaranteed rather than incidental. That
    // is the behaviour we want: a garbage spec renders a clamped control instead
    // of panicking the app.
    (value * scale).round() as i32
}

/// The five per-feature toggles plus their scalar controls, read off whatever UI
/// is driving them.
///
/// Exists so [`ridge_options_from_rows`] can be a pure function. Its predecessor
/// took `&AppWindow` and was therefore untestable by construction — the reason
/// none of this arithmetic had a test before.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RidgeToggles {
    /// Master switch for the whole interior-ridge feature set.
    pub enabled: bool,
    /// Grip rings on/off.
    pub rings_enabled: bool,
    /// Surface texture on/off, with its depth + spacing in tenths of a mm.
    pub texture_enabled: bool,
    /// Texture depth, tenths of a millimetre.
    pub texture_depth_tenths_mm: i32,
    /// Texture spacing, tenths of a millimetre.
    pub texture_spacing_tenths_mm: i32,
    /// Lateral pinch on/off, with depth in tenths of a mm.
    pub side_pinch_enabled: bool,
    /// Side-pinch depth, tenths of a millimetre.
    pub side_pinch_tenths_mm: i32,
    /// Tip relief on/off, with depth in tenths of a mm.
    pub tip_relief_enabled: bool,
    /// Tip-relief depth, tenths of a millimetre.
    pub tip_relief_tenths_mm: i32,
    /// Orientation override on/off, with the angle in whole degrees.
    pub orientation_enabled: bool,
    /// Orientation, degrees.
    pub orientation_deg: i32,
}

/// Ring rows + toggles → an owned [`RidgeOptions`], with each disabled feature
/// zeroed by the already-tested [`gate_ridge_options`].
#[must_use]
pub fn ridge_options_from_rows(rows: &[RingRow], toggles: RidgeToggles) -> RidgeOptions {
    gate_ridge_options(RidgeControls {
        enabled: toggles.enabled,
        rings_enabled: toggles.rings_enabled,
        rings: rows.iter().map(RingRow::to_ridge).collect(),
        texture_enabled: toggles.texture_enabled,
        texture_depth_m: tenths_mm_to_m(toggles.texture_depth_tenths_mm),
        texture_spacing_m: tenths_mm_to_m(toggles.texture_spacing_tenths_mm),
        side_pinch_enabled: toggles.side_pinch_enabled,
        side_pinch_depth_m: tenths_mm_to_m(toggles.side_pinch_tenths_mm),
        tip_relief_enabled: toggles.tip_relief_enabled,
        tip_relief_depth_m: tenths_mm_to_m(toggles.tip_relief_tenths_mm),
        orientation_enabled: toggles.orientation_enabled,
        orientation_deg: f64::from(toggles.orientation_deg),
    })
}

/// Layer thickness, whole millimetres. ⚠ One, not zero — the cast meshes a
/// shell per layer, and a layer of no thickness is not a layer.
const LAYER_THICKNESS_RANGE: (i32, i32) = (1, 100);
/// Slacker™ softening, as a percentage of the layer's mix.
const LAYER_SLACKER_RANGE: (i32, i32) = (0, 100);

/// The stack the step-4 screen opens on, carried over from the pre-port
/// screen: a soft, slacker-softened inner layer under two progressively firmer
/// ones. `(catalog key, thickness mm, slacker %)`.
///
/// ⚠ The pre-port called it "≈ the `base_mold` recipe"; no `base_mold` design
/// is in the repo, so that is as far as it can be checked. A starting point,
/// not a prescription — every field is editable, and a `.design.toml` replaces
/// the lot.
const OPENING_STACK: [(&str, i32, i32); 3] = [
    ("ECOFLEX_00_30", 18, 25),
    ("DRAGON_SKIN_10A", 8, 0),
    ("DRAGON_SKIN_20A", 5, 0),
];

/// What "+ Add layer" adds, in `OPENING_STACK`'s units: a middling silicone,
/// thin, unsoftened.
const ADDED_LAYER: (&str, i32, i32) = ("DRAGON_SKIN_10A", 5, 0);

/// One entry of the silicone catalog: the key a [`LayerDraft`] carries, and
/// the name the material picker shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Silicone {
    /// The SDK's catalog key, e.g. `"ECOFLEX_00_30"`.
    pub key: &'static str,
    /// The display name, e.g. `"Ecoflex 00-30 (medium-soft)"`.
    pub name: &'static str,
}

impl Silicone {
    /// The catalog, in the order the picker offers it.
    #[must_use]
    pub fn catalog() -> Vec<Self> {
        silicone_catalog()
            .into_iter()
            .map(|(key, name)| Self { key, name })
            .collect()
    }

    /// The entry for `key`, or `None` if that silicone is not in the catalog.
    #[must_use]
    pub fn from_key(key: &str) -> Option<Self> {
        Self::catalog().into_iter().find(|s| s.key == key)
    }
}

/// One silicone layer in the step-4 editor, in the integer units the UI edits
/// (whole millimetres, percent) rather than the SDK's meters and fractions.
///
/// The two stepper fields live **inside the row** on purpose — see the
/// warning on [`StepBoxState`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LayerRow {
    /// Which silicone this layer is poured in.
    ///
    /// ⚠ The catalog entry itself, not an index into the catalog and not a
    /// bare key: the row needs no lookup to draw its picker or to build its
    /// [`LayerDraft`], so there is no "silicone not found" branch to get
    /// wrong. The pre-port screen held an index and carried a fallback
    /// material for exactly that branch.
    pub material: Silicone,
    /// How thick this layer is poured, whole millimetres. A `.design.toml`
    /// carries sub-millimetre thicknesses; this editor does not offer them.
    pub thickness_mm: BoundedField,
    /// How much Slacker™ softens the mix, percent.
    pub slacker_pct: BoundedField,
}

impl LayerRow {
    /// A row at the UI's own units, inside the bounds its steppers offer.
    #[must_use]
    pub fn new(material: Silicone, thickness_mm: i32, slacker_pct: i32) -> Self {
        Self {
            material,
            thickness_mm: BoundedField::clamped(thickness_mm, LAYER_THICKNESS_RANGE),
            slacker_pct: BoundedField::clamped(slacker_pct, LAYER_SLACKER_RANGE),
        }
    }

    /// The layer this row describes, in the SDK's units.
    #[must_use]
    pub fn draft(&self) -> LayerDraft {
        LayerDraft {
            thickness_m: f64::from(self.thickness_mm.value()) / 1000.0,
            material_key: self.material.key.to_string(),
            slacker_fraction: f64::from(self.slacker_pct.value()) / 100.0,
        }
    }

    /// The row for `draft`, or `None` if its silicone is not one the catalog
    /// carries — the editor has no picker entry to show it with, and a row
    /// that substituted another silicone would be committed as if the user
    /// had chosen it.
    ///
    /// ⚠ Lossy the other way, and it matters: this editor edits **whole**
    /// millimetres and whole percent, while a `.design.toml` carries neither
    /// bound. `base_mold`'s own stack is 17.5 / 7.5 / 5 mm, so a row for it
    /// reads 18 / 8 / 5 — see [`format_inexact_design`], which is what tells
    /// the user before the button beside the rows commits them.
    ///
    #[must_use]
    pub fn from_draft(draft: &LayerDraft) -> Option<Self> {
        Some(Self::new(
            Silicone::from_key(&draft.material_key)?,
            m_to_mm(draft.thickness_m),
            scale_to_i32(draft.slacker_fraction, 100.0),
        ))
    }
}

/// The step-4 silicone stack: the layers built outward off the shaped plug,
/// innermost first.
///
/// Owns the rows so the "never drop the last layer" rule is one tested method
/// rather than a condition spelled out at the button — which is where the
/// pre-port screen kept it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LayerStack {
    rows: Vec<LayerRow>,
}

/// `OPENING_STACK`, skipping any silicone the catalog no longer carries —
/// pinned whole by `the_screen_opens_on_the_pre_ports_stack`, which fails
/// loudly if one goes missing.
impl Default for LayerStack {
    fn default() -> Self {
        Self {
            rows: OPENING_STACK
                .iter()
                .filter_map(|&(key, thickness_mm, slacker_pct)| {
                    Some(LayerRow::new(
                        Silicone::from_key(key)?,
                        thickness_mm,
                        slacker_pct,
                    ))
                })
                .collect(),
        }
    }
}

impl LayerStack {
    /// The rows, innermost first, for rendering.
    #[must_use]
    pub fn rows(&self) -> &[LayerRow] {
        &self.rows
    }

    /// The rows, for the screen that edits them in place.
    #[must_use]
    pub fn rows_mut(&mut self) -> &mut [LayerRow] {
        &mut self.rows
    }

    /// The stack `layers` describes, or `None` if it is empty or names a
    /// silicone the catalog no longer carries.
    ///
    /// `None` means the caller leaves its rows as they are: a screen showing
    /// a stack this editor substituted would be committed as the user's own
    /// choice by the button beside it.
    #[must_use]
    pub fn from_drafts(layers: &[LayerDraft]) -> Option<Self> {
        if layers.is_empty() {
            return None;
        }
        Some(Self {
            rows: layers
                .iter()
                .map(LayerRow::from_draft)
                .collect::<Option<Vec<_>>>()?,
        })
    }

    /// Whether a layer can be dropped: the cast needs a stack, so the last one
    /// stays. Read by both the ✖ that offers the drop and [`Self::remove`] that
    /// performs it, which is how they cannot disagree.
    #[must_use]
    pub fn can_drop(&self) -> bool {
        self.rows.len() > 1
    }

    /// Add a layer on the outside, where the "+ Add layer" button puts it.
    pub fn add(&mut self) {
        let (key, thickness_mm, slacker_pct) = ADDED_LAYER;
        if let Some(material) = Silicone::from_key(key) {
            self.rows
                .push(LayerRow::new(material, thickness_mm, slacker_pct));
        }
    }

    /// Drop one layer. A no-op on the last — see [`Self::can_drop`].
    pub fn remove(&mut self, index: usize) {
        if self.can_drop() && index < self.rows.len() {
            self.rows.remove(index);
        }
    }

    /// The stack in the SDK's units, innermost first.
    #[must_use]
    pub fn drafts(&self) -> Vec<LayerDraft> {
        self.rows.iter().map(LayerRow::draft).collect()
    }
}

/// What to add to the "design set" message when the rows cannot hold the design
/// that just landed, exactly.
///
/// `None` when the rows **are** the design. It matters because the button beside
/// them commits the rows, not the file.
///
/// ⚠ Names no cause, because there are three and each describes the others
/// wrongly: the steppers round (17.5 mm → 18), clamp up (0.2 mm → 1) and clamp
/// down (150 mm → 100), and the slacker field does the same. The rows are on
/// screen carrying the real numbers, so the message points at them instead.
#[must_use]
pub fn format_inexact_design(shown: &LayerStack, loaded: &[LayerDraft]) -> Option<String> {
    (shown.drafts() != loaded).then(|| {
        " \u{26a0} Not exactly this file \u{2014} the rows below are what \"Use this design\" \
         would write."
            .to_string()
    })
}

/// The step-5 part picker: which cast pieces to generate.
///
/// Owns the label/checked rows **and** the parallel `PartId`s, which the Slint
/// version kept in two structures (a `VecModel<PartRow>` beside a
/// `RefCell<Vec<PartId>>`) that had to be rebuilt in lockstep. One type removes
/// the chance of them disagreeing.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PartPicker {
    rows: Vec<(PartId, String, bool)>,
}

impl PartPicker {
    /// Rebuild for a layer count, everything checked (the default: make it all).
    #[must_use]
    pub fn rebuild(layer_count: usize, mode: CastMode) -> Self {
        Self {
            rows: enumerate_parts(layer_count, mode)
                .into_iter()
                .map(|(id, label)| (id, label, true))
                .collect(),
        }
    }

    /// `(label, checked)` for rendering.
    pub fn rows(&self) -> impl Iterator<Item = (&str, bool)> {
        self.rows.iter().map(|(_, l, c)| (l.as_str(), *c))
    }

    /// Number of rows.
    #[must_use]
    pub fn len(&self) -> usize {
        self.rows.len()
    }

    /// Whether the picker has no rows at all (no design yet).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    /// Toggle one row. Out-of-range indices are ignored.
    pub fn set_checked(&mut self, index: usize, checked: bool) {
        if let Some(row) = self.rows.get_mut(index) {
            row.2 = checked;
        }
    }

    /// The All / None buttons.
    pub fn set_all(&mut self, checked: bool) {
        for row in &mut self.rows {
            row.2 = checked;
        }
    }

    /// `true` when at least one part is checked — make-molds needs ≥1 piece.
    #[must_use]
    pub fn any_checked(&self) -> bool {
        self.rows.iter().any(|(_, _, c)| *c)
    }

    /// The checked rows as a [`PartSelection`]; all-checked collapses to
    /// [`PartSelection::all`] via the already-tested `part_selection_from_checks`.
    #[must_use]
    pub fn selection(&self, mode: CastMode) -> PartSelection {
        let parts: Vec<(PartId, String)> = self
            .rows
            .iter()
            .map(|(id, l, _)| (*id, l.clone()))
            .collect();
        let checked: Vec<bool> = self.rows.iter().map(|(_, _, c)| *c).collect();
        part_selection_from_checks(&parts, &checked, mode)
    }
}

/// The step-2 working-mesh stats line, shown above the cleanup controls.
#[must_use]
pub fn format_scan_stats(faces: usize, vertices: usize) -> String {
    format!("{faces} faces · {vertices} vertices")
}

/// The smallest trim bound the fields will offer, in millimetres.
///
/// A degenerate centerline would otherwise collapse both fields to a single
/// value, leaving nothing to type into and no way to recover.
const TRIM_BOUND_MIN_MM: i32 = 10;

/// The largest, in millimetres — a sanity rail against a nonsense arc
/// length, not a physical limit.
const TRIM_BOUND_MAX_MM: i32 = 100_000;

/// The trim fields' upper bound, in whole millimetres.
///
/// Trimming is measured along the centerline, so its arc length is the most
/// that can be taken off. This is also what [`StepBoxState::commit`] clamps
/// against, which is why the bound is floored rather than passed through.
///
/// A non-finite arc length saturates rather than wrapping, then clamps — so
/// a centerline that failed to trace still yields a usable field.
#[must_use]
#[allow(clippy::cast_possible_truncation)] // Saturating; the clamp bounds it either way.
pub fn trim_bound_mm(centerline_arc_length_mm: f64) -> i32 {
    (centerline_arc_length_mm.round() as i32).clamp(TRIM_BOUND_MIN_MM, TRIM_BOUND_MAX_MM)
}

/// Whether a cap scan can be stood upright, and what to say when it cannot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FloorReadiness {
    /// Raw, unwelded vertex soup: the boundary loops reported are seams between
    /// duplicated vertices, not the real open end.
    Unwelded,
    /// Welded, but closed — there is no open boundary to stand the scan on.
    NoOpenEdges,
    /// Worth trying to level.
    Ready,
}

impl FloorReadiness {
    /// Read a `detect_caps` summary.
    ///
    /// ⚠ Order matters. Unwelded soup reports *many* loops, so it sails past
    /// a `loop_count` test — the weld prompt has to win, or the user is told
    /// the scan is ready on the strength of a count that means nothing.
    #[must_use]
    pub const fn read(loop_count: usize, looks_unwelded: bool) -> Self {
        if looks_unwelded {
            Self::Unwelded
        } else if loop_count == 0 {
            Self::NoOpenEdges
        } else {
            Self::Ready
        }
    }

    /// Why the scan cannot be stood up, or `None` when it can.
    #[must_use]
    pub const fn blocked_message(self) -> Option<&'static str> {
        match self {
            Self::Unwelded => Some("Looks like a raw scan — click Weld first, then Find floor."),
            Self::NoOpenEdges => Some("No open edges found to stand it on."),
            Self::Ready => None,
        }
    }
}

/// "Find floor" succeeded: what it found, and how far it tipped the scan.
#[must_use]
pub fn format_floor_found(loop_count: usize, centerline_segments: usize, tilt_deg: f64) -> String {
    format!(
        "✔ Found floor — {loop_count} open loop(s), \
         {centerline_segments}-segment centerline. \
         Stood upright (corrected {tilt_deg:.0}° tilt)."
    )
}

/// Open loops were found, but no centerline could be traced through them —
/// so there is no axis to level against.
#[must_use]
pub fn format_floor_no_centerline(loop_count: usize) -> String {
    format!("Found {loop_count} loop(s) but couldn't trace a centerline to level by.")
}

/// A Simplify has started: what it is aiming at, and how long to expect.
///
/// The estimate is part of the message on purpose — this is the only op in the
/// wizard that takes long enough for a still window to read as a hang.
#[must_use]
pub fn format_simplify_started(target_faces: usize) -> String {
    format!("Simplifying to {target_faces} faces… (this can take ~10–40 s)")
}

/// A Simplify has landed: the target it was asked for, and what it cost.
///
/// ⚠ Reports the target that was asked for, not the count that resulted — as
/// the pre-port message did. The stats line above the step-2 controls carries
/// the count the mesh actually ended up with.
#[must_use]
pub fn format_simplify_done(target_faces: usize, secs: f64) -> String {
    format!("✔ Simplified to {target_faces} faces ({secs:.1}s).")
}

/// A Save has landed: what was written, and that the step is now complete.
///
/// ⚠ Names both files. They are written as a pair and [`apply_prep`] accepts
/// them as a pair, so reporting only the STL would leave the user hunting for
/// the file the cast actually reads.
#[must_use]
pub fn format_save_done(stem: &str, face_count: usize) -> String {
    format!(
        "✔ Saved {stem}.cleaned.stl ({face_count} faces) + {stem}.prep.toml — \
         step complete, click Next →."
    )
}

/// Everything a plug-fit verdict depends on, snapshotted when the check was
/// asked for.
///
/// ★ The staleness rule, as one value. A verdict is only ever about a
/// particular question, so the screen compares the whole question rather than
/// watching fields go by: the cavity inset, the ridge master switch, every ring
/// row and all five ridge scalars live inside [`PlugDraft`], and a ridge field
/// added later is covered without anyone remembering to hook it up.
#[derive(Debug, Clone, PartialEq)]
pub struct FitQuestion {
    /// The plug as step 3's fields read when the check was started.
    pub plug: PlugDraft,
    /// The cell size the cast would run at — part of the question, not a speed
    /// knob. Detachment turns on sub-cell grid alignment, so a verdict does not
    /// transfer between cell sizes; see `cf_studio_engine::plug_fit_preflight`.
    pub cell_size_m: f64,
    /// Which cleaned scan it is about: length and modified time, or `None` when
    /// that could not be read.
    ///
    /// ⚠ Not the path, and this is the axis the fields cannot cover. Step 2 is
    /// reachable from step 3 and a second Save writes the cleaned scan back to
    /// the same place at a different smoothing — the inset never moves, the
    /// body does, and without this a verdict about the body that was replaced
    /// stays on screen. Stamped by `crate::preview::scan_stamp`, which tells
    /// the 3D preview the same thing.
    pub scan: Option<(u64, std::time::SystemTime)>,
}

/// What step 3 has to say about the fit, this frame.
#[derive(Debug, Clone, PartialEq)]
pub enum FitView<'a> {
    /// Nothing has been asked about the settings on screen.
    Idle,
    /// A check is in flight, and has been for this many whole seconds.
    Checking {
        /// Whole seconds since it started.
        elapsed_secs: u64,
        /// Whether this one runs in minutes rather than seconds — see
        /// [`fit_check_is_slow`].
        slow: bool,
        /// The inset being checked, in millimetres — ⚠ NOT necessarily the one
        /// on screen. Nothing stops the fields moving while a check runs, and
        /// the answer is then dropped on arrival; naming the question in flight
        /// is what stops a five-minute wait ending in nothing anyone can
        /// explain.
        inset_mm: f64,
    },
    /// The cast's verdict on the settings on screen.
    Answered(&'a PlugFit),
    /// The check could not be RUN — an unreadable scan, a bad prep file. ⚠ Not
    /// a verdict: the inset may be perfectly fine and nobody found out.
    Failed(&'a str),
}

/// Decide what step 3 shows about the fit.
///
/// ⚠ A running check outranks a standing answer. The answer it is about to
/// replace describes the same question, so leaving it up would read as settled
/// while the screen is busy deciding whether it still is.
///
/// ⚠ An answer to a question nobody is asking any more is dropped, not
/// relabelled: what step 3 shows describes the fields as they read *now*. A run
/// whose fields move mid-flight lands into that same drop, which is why nothing
/// needs to cancel it — a stale run costs time, never a wrong answer.
#[must_use]
pub fn fit_view<'a>(
    running: Option<(&FitQuestion, u64)>,
    answered: Option<&'a (FitQuestion, Result<PlugFit, String>)>,
    current: &FitQuestion,
) -> FitView<'a> {
    if let Some((asked, elapsed_secs)) = running {
        return FitView::Checking {
            elapsed_secs,
            slow: fit_check_is_slow(asked),
            inset_mm: asked.plug.cavity_inset_m * 1000.0,
        };
    }
    match answered {
        Some((asked, Ok(fit))) if asked == current => FitView::Answered(fit),
        Some((asked, Err(reason))) if asked == current => FitView::Failed(reason),
        _ => FitView::Idle,
    }
}

/// The fit verdict as step 3's status line: `Ok` reads as settled, `Err` as a
/// refusal.
///
/// ⚠⚠ THE TWO DIRECTIONS ARE NOT EQUALLY STRONG, and the wording follows that.
/// A refusal is sound: the cast composes the same plug and declines the same
/// way. A pass is NOT — `plug_fit_verdict` runs the compose half only, and the
/// export F4-gates the mesh afterwards and can still refuse
/// (`CastError::PrintabilityCritical`). `cf-studio-engine`'s own gate is
/// one-directional for exactly this reason. So the settled line claims what was
/// actually checked — the plug came out in one piece — and never that the cast
/// will succeed.
///
/// ⚠ The refusal carries the cast's own words verbatim. They already name the
/// operator's levers, and a paraphrase here would be a second explanation of
/// the same failure, free to drift from the one the cast will give.
pub fn format_fit_verdict(fit: &PlugFit, plug: &PlugDraft) -> StepOutcome {
    let inset_mm = plug.cavity_inset_m * 1000.0;
    match fit {
        PlugFit::Casts => Ok(format!(
            "✔ A {inset_mm:.1} mm inset leaves the plug in one piece."
        )),
        PlugFit::WillNotCast { reason } => Err(format!(
            "✖ A {inset_mm:.1} mm inset will not cast: {reason}"
        )),
    }
}

/// Whether a fit check runs in minutes rather than seconds.
///
/// Two things drive the cost, and ridges are only one of them. Enabled ridges
/// pin layer 0's plug at the canal's own 0.5 mm whatever cell size is passed,
/// so they are slow at every quality; smooth, the cost tracks the cell size —
/// which is why the comparison is against [`cell_size_m_for_quality`]'s own
/// Fast size rather than a number written down here.
///
/// Measured 2026-09-08 on `~/scans/base_mold`, one smooth plug: **6.7 s at
/// 1.5 mm, 80.3 s at 0.5 mm, 289.4 s ridged**. Only the Fast-quality smooth
/// case is quick, and it is not the quality step 5 opens on.
#[must_use]
pub fn fit_check_is_slow(question: &FitQuestion) -> bool {
    question.plug.ridges.enabled || question.cell_size_m < cell_size_m_for_quality(FAST_QUALITY)
}

/// The quality index whose cell size is the one a fit check can be quick at.
const FAST_QUALITY: i32 = 1;

/// How long step 3's settings must stand still before the fit check runs
/// without anyone clicking for it.
///
/// ⚠ The trade it sets is lopsided in cost. Too short, and a value merely
/// stepped through buys a check that is dropped the moment it lands — 80 s at
/// the quality step 5 opens on, 291 s with ridges (see [`fit_check_is_slow`]).
/// Too long, and the operator is on step 4 before the answer arrives, which is
/// the whole failure this exists to stop.
pub const FIT_SETTLE: Duration = Duration::from_millis(1500);

/// Whether step 3 should run the fit check nobody clicked for.
///
/// ★ This is what makes the inset range honest. Step 3's cavity stepper offers
/// a fixed 0-30 mm whatever scan is loaded, and what a scan can take is neither
/// a constant nor predictable from the scan: `~/scans/base_mold`, measured
/// 2026-09-09, refuses above 11 mm at Fast and 12 at Fine, and past ~20 mm has
/// no plug left to mesh at all. So the range stays wide — a clamp would refuse
/// insets that do cast — and the cast's own verdict arrives at step 3 instead
/// of being bought with a full cast at step 5.
///
/// ⚠ Only from [`FitView::Idle`], which is the single state meaning nothing is
/// in flight and nothing on screen already answers these settings. `Failed` is
/// deliberately not included: it is a check that could not RUN, so re-firing on
/// it would spin.
///
/// ⚠ `askable` is the app accepting actions AND the CLEANED scan being present
/// — `prep`, exactly what `start_plug_fit` needs, not the raw scan that arrives
/// a step earlier.
///
/// ⚠⚠ Nothing in the app as it stands can trip either half: `nav_state` will
/// not carry the operator to step 3 before step 2 completes, and a loaded
/// project has its `current_step` clamped by `Project::migrate` and its gaps
/// rejected by `Project::validate`. They are here because the check now starts
/// with NO ONE in the loop. A button that answers "Clean and save the scan
/// first" is informative; a screen that posts it by itself — onto a `Failed`
/// that never re-fires — is a red line the operator did not ask for and cannot
/// clear.
#[must_use]
pub fn fit_check_is_due(view: &FitView<'_>, settled_for: Duration, askable: bool) -> bool {
    askable && settled_for >= FIT_SETTLE && matches!(view, FitView::Idle)
}

/// A check that could not run, as step 3's status line.
///
/// ⚠ Reported here rather than through `Studio::message`, and that is the
/// point. Every other job in `jobs.rs` takes `Studio::busy`, which disables the
/// nav and pins the operator on the step that started it — so the shared
/// message always lands where it makes sense. This one deliberately does not,
/// so a failure sent there would surface on whatever step they had walked on
/// to, and would replace a running cast's progress line while it did.
#[must_use]
pub fn format_fit_failure(reason: &str) -> String {
    format!("✖ Couldn't check the fit: {reason}")
}

/// The check's progress line.
///
/// ⚠ Every field here is read off the question the run was *started* with,
/// never off the live screen. Editing the inset, the ridge switch or step 5's
/// quality mid-run changes neither what is running nor what it will cost — and
/// the inset is named precisely so a run the screen has moved away from is
/// visibly about something else, rather than looking like it is checking what
/// the operator is now looking at.
#[must_use]
pub fn format_fit_progress(elapsed_secs: u64, slow: bool, inset_mm: f64) -> String {
    let cost = if slow {
        " — this takes a few minutes."
    } else {
        ""
    };
    format!("Checking {inset_mm:.1} mm… {elapsed_secs}s{cost}")
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use super::*;

    /// A folder of this test's own, named and PID-scoped so concurrent runs
    /// cannot read each other's files.
    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("cf-autosave-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a temp dir");
        dir
    }

    /// A project taken as far as `furthest`, with every artifact on the way.
    ///
    /// ⚠ Plain data all the way: the setters gate on the previous step, not on
    /// anything being on disk, so this needs no cast run and no scan file.
    fn walked_to(furthest: Step) -> Project {
        use cf_studio_core::{PrepInput, ScanInput};

        let mut project = Project::new("resume gate");
        project.set_scan(ScanInput {
            source_path: "/scans/base.stl".into(),
        });
        if furthest >= Step::CleanScan {
            project
                .set_prep(PrepInput {
                    cleaned_stl: "/scans/base.cleaned.stl".into(),
                    prep_toml: "/scans/base.prep.toml".into(),
                })
                .expect("in workflow order");
        }
        if furthest >= Step::ShapePiece {
            project
                .set_plug(PlugDraft::default())
                .expect("in workflow order");
        }
        assert_eq!(
            project.furthest_completed(),
            Some(furthest),
            "the fixture must stop exactly where it says"
        );
        project
    }

    /// ★ The naming rule, and the coupling it exists for: the project and the
    /// cleaned scan it points at are named off the same `(dir, stem)`, so they
    /// cannot end up in different folders or under different names.
    #[test]
    fn the_project_file_is_named_off_the_same_stem_as_the_cleaned_scan() {
        let scan = Path::new("/scans/left-forearm.stl");
        let (dir, stem) = scan_outputs(scan);

        assert_eq!(dir, PathBuf::from("/scans"), "beside the scan");
        assert_eq!(stem, "left-forearm");
        assert_eq!(
            autosave_path(scan),
            dir.join(format!("{stem}.cfproject.json")),
            "and named off that same stem"
        );
    }

    /// The awkward paths, because this is a total function and every caller
    /// treats what it returns as somewhere to write.
    #[test]
    fn every_scan_path_names_exactly_one_file_to_write() {
        assert_eq!(
            autosave_path(Path::new("base.stl")),
            PathBuf::from("base.cfproject.json"),
            "no folder named means the working directory, beside the scan"
        );
        assert_eq!(
            autosave_path(Path::new("/scans/.stl")),
            PathBuf::from("/scans/.stl.cfproject.json"),
            "a dotfile's whole name is its stem"
        );
        // ⚠ A path with no filename at all still has to name a file, or the
        // fallback stem is dead code and this writes to a directory.
        assert_eq!(
            autosave_path(Path::new("/")),
            PathBuf::from(format!("./{FALLBACK_STEM}.cfproject.json")),
            "nothing to name it after falls back rather than writing nowhere"
        );
    }

    /// ⚠ No file is the ordinary case — the first time any scan is picked — and
    /// it has to read as "go ahead and save", not as a reason to stop.
    #[test]
    fn a_scan_with_no_project_beside_it_is_fresh() {
        let dir = temp_dir("no-file");

        assert_eq!(
            inspect_autosave(&dir.join("nothing-here.cfproject.json")),
            ResumeOffer::Fresh
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★★ A pick and nothing else is not work worth a question. Without this
    /// the modal lands in front of an ordinary re-pick of the same scan, and
    /// the only answer that gets the user anywhere is the one that overwrites.
    #[test]
    fn a_project_recording_only_the_scan_is_fresh() {
        let dir = temp_dir("scan-only");
        let path = dir.join("base.cfproject.json");
        walked_to(Step::AddScan)
            .save(&path)
            .expect("a fixture file");

        assert_eq!(inspect_autosave(&path), ResumeOffer::Fresh);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★★★ The offer itself, carrying the project whole — "resume" has to
    /// produce it — and the step it reached, which is the whole question.
    #[test]
    fn a_project_past_the_scan_is_offered_back_whole() {
        let dir = temp_dir("resumable");
        let path = dir.join("base.cfproject.json");
        let saved = walked_to(Step::ShapePiece);
        saved.save(&path).expect("a fixture file");

        assert_eq!(
            inspect_autosave(&path),
            ResumeOffer::Resumable {
                project: Box::new(saved),
                reached: Step::ShapePiece,
            },
            "the project comes back as it went in, and says how far it got"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The boundary between the two, one step apart. Asserted as a pair
    /// because a `>=` for the `>` moves it by exactly one step, and either
    /// side alone reads as correct.
    #[test]
    fn the_offer_starts_at_the_first_step_past_the_pick() {
        let dir = temp_dir("boundary");
        let scan_only = dir.join("a.cfproject.json");
        let cleaned = dir.join("b.cfproject.json");
        walked_to(Step::AddScan).save(&scan_only).expect("a file");
        walked_to(Step::CleanScan).save(&cleaned).expect("a file");

        assert_eq!(inspect_autosave(&scan_only), ResumeOffer::Fresh);
        assert!(
            matches!(
                inspect_autosave(&cleaned),
                ResumeOffer::Resumable {
                    reached: Step::CleanScan,
                    ..
                }
            ),
            "one step further is worth asking about"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★★★ Every way a file can be unreadable, because the consequence is the
    /// same one and it is the worst one: a build that treats an unreadable file
    /// as "nothing there" overwrites work it merely failed to understand.
    ///
    /// ⚠ Enumerated over real failures — hand-edited, written by a newer
    /// Cendrillon, and a file this build's own invariants reject (#899's).
    #[test]
    fn a_file_this_build_cannot_read_is_never_treated_as_absent() {
        use cf_studio_core::PROJECT_SCHEMA_VERSION;

        let dir = temp_dir("unreadable");
        let hand_edited = dir.join("a.cfproject.json");
        std::fs::write(&hand_edited, b"{ not json at all").expect("a file");

        // ⚠ Written by editing what `save` produced, not by hand: a hand-written
        // body could be refused for a typo instead of for its version, and this
        // would pass while measuring nothing.
        let from_the_future = dir.join("b.cfproject.json");
        let written = dir.join("current.cfproject.json");
        walked_to(Step::ShapePiece).save(&written).expect("a file");
        let body = std::fs::read_to_string(&written).expect("what save wrote");
        let bumped = body.replace(
            &format!("\"schema_version\": {PROJECT_SCHEMA_VERSION}"),
            &format!("\"schema_version\": {}", PROJECT_SCHEMA_VERSION + 1),
        );
        assert_ne!(bumped, body, "the version must actually have moved");
        std::fs::write(&from_the_future, bumped).expect("a file");

        // ⚠ Not every unreadable file is unreadable for a reason inside it. A
        // folder in the file's place — a sync tool's doing, or a hand-made
        // mistake — fails the read with an I/O error that is NOT "not found",
        // and reading that as "nothing there" writes over whatever it holds.
        let in_the_way = dir.join("d.cfproject.json");
        std::fs::create_dir(&in_the_way).expect("something else in the file's place");

        // #899's invariant: the design's inset is a copy of the plug's.
        let disagreeing = dir.join("c.cfproject.json");
        let mut project = walked_to(Step::ShapePiece);
        project
            .set_design(DesignDraft {
                cavity_inset_m: 0.010,
                layers: vec![LayerDraft {
                    thickness_m: 0.003,
                    material_key: "ECOFLEX_00_30".to_string(),
                    slacker_fraction: 0.0,
                }],
            })
            .expect("in workflow order");
        // `save` is the no-corruption primitive, not an invariant gate, so a
        // project the loader will refuse can be written — which is the point.
        project.save(&disagreeing).expect("a file");

        for (what, path) in [
            ("hand-edited", &hand_edited),
            ("from a newer build", &from_the_future),
            ("failing this build's invariants", &disagreeing),
            ("not a file at all", &in_the_way),
        ] {
            assert!(
                matches!(inspect_autosave(path), ResumeOffer::Unreadable(_)),
                "a file {what} must never be overwritten: {:?}",
                inspect_autosave(path)
            );
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The question names the step, not the file — it is the only thing the
    /// user can use to decide.
    #[test]
    fn the_resume_question_names_how_far_the_saved_session_got() {
        let question = format_resume_question(Step::DesignLayers);

        assert!(
            question.contains(Step::DesignLayers.title()),
            "the step the user would come back to: {question}"
        );
        assert!(
            question.contains("this session takes its place"),
            "and what the other answer costs: {question}"
        );
    }

    const ONE_TRIANGLE_STL: &str = "\
solid t
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid t
";

    const PREP_WITH_CENTERLINE: &str = "\
[centerline]
points_m = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.01]]
";

    const DESIGN_TOML: &str = "\
[device_design]
tool_version = \"x\"
generated_at = \"2026-01-01T00:00:00Z\"
schema_version = 1
[scan_ref]
cleaned_stl = \"c.stl\"
[cavity]
inset_m = 0.005
visible = true
[[layers]]
thickness_m = 0.005
material_anchor_key = \"ECOFLEX_00_30\"
slacker_fraction = 0.0
visible = true
";

    fn dir(label: &str) -> PathBuf {
        let d =
            std::env::temp_dir().join(format!("cf-studio-gui-test-{}-{label}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn apply_plug_skips_with_default_and_gates_on_clean_scan() {
        use cf_studio_core::PlugDraft;
        let mut p = Project::new("t");
        // Before the scan is cleaned, the plug step can't be committed.
        assert!(apply_plug(&mut p, PlugDraft::default()).is_err());

        let d = dir("plug");
        let stl = d.join("s.stl");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        let cleaned = d.join("s.cleaned.stl");
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        let prep = d.join("s.prep.toml");
        std::fs::write(&prep, "[centerline]\npoints_m = [[0,0,0],[0,0,0.01]]\n").unwrap();
        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();

        // The default plug (no ridges) is the skip — records + advances.
        let msg = apply_plug(&mut p, PlugDraft::default()).unwrap();
        assert!(msg.contains("no ridges"), "got: {msg}");
        assert!(p.is_complete(Step::ShapePiece));

        // Ridges on reports them.
        let msg = apply_plug(
            &mut p,
            PlugDraft {
                cavity_inset_m: 0.004,
                ridges: cf_studio_core::RidgeOptions {
                    enabled: true,
                    ..cf_studio_core::RidgeOptions::default()
                },
            },
        )
        .unwrap();
        assert!(msg.contains("ridges on"), "got: {msg}");

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn fresh_project_has_seven_rows_all_undone_at_step_one() {
        let p = Project::new("t");
        let rows = step_rows(&p, Step::AddScan);
        assert_eq!(rows.len(), 7);
        assert!(
            rows.iter().all(|r| !r.done),
            "nothing done on a fresh project"
        );
        assert!(rows[0].current, "current step is AddScan");
        assert!(!rows[1].current);
        for (i, r) in rows.iter().enumerate() {
            assert_eq!(r.number, i32::try_from(i + 1).unwrap());
            assert!(!r.title.is_empty());
        }
    }

    #[test]
    fn viewing_marks_the_previewed_step_independent_of_current() {
        let p = Project::new("t"); // still at AddScan
        let rows = step_rows(&p, Step::MakeMolds);
        assert!(rows[4].viewing, "MakeMolds (index 4) is being viewed");
        assert!(!rows[0].viewing);
        assert!(rows[0].current, "but the project is still on AddScan");
        assert!(!rows[4].current);
    }

    #[test]
    fn apply_scan_records_and_returns_a_message() {
        let d = dir("scan");
        let stl = d.join("s.stl");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();

        let mut p = Project::new("t");
        let msg = apply_scan(&mut p, &stl).unwrap();
        assert!(msg.contains("Added scan"), "got: {msg}");
        assert!(p.is_complete(Step::AddScan));

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn apply_scan_missing_file_is_an_error_message() {
        let mut p = Project::new("t");
        assert!(apply_scan(&mut p, Path::new("/no/such/scan.stl")).is_err());
    }

    /// The piece DESIGN_TOML is written for — a 5 mm cavity, so a fixture
    /// pairing the two carries no inset clash.
    fn shaped_at_5mm() -> cf_studio_core::PlugDraft {
        cf_studio_core::PlugDraft {
            cavity_inset_m: 0.005,
            ..cf_studio_core::PlugDraft::default()
        }
    }

    /// A design file walked onto `plug`, and what came back.
    fn design_file_onto(
        label: &str,
        toml: &str,
        plug: cf_studio_core::PlugDraft,
    ) -> (Project, String) {
        let d = dir(label);
        let (stl, cleaned, prep, design) = (
            d.join("s.stl"),
            d.join("c.stl"),
            d.join("p.prep.toml"),
            d.join("x.design.toml"),
        );
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();
        std::fs::write(&design, toml).unwrap();

        let mut p = Project::new("t");
        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();
        apply_plug(&mut p, plug).unwrap();
        let message = apply_design(&mut p, &design).unwrap();
        let _ = std::fs::remove_dir_all(&d);
        (p, message)
    }

    /// ★★ `Project::plug` and `Project::design` each carry a cavity inset, and
    /// two different parts of the app read them: the preview and "Check fit"
    /// read the plug, `start_molds` casts the design. A file that set one and
    /// not the other made the mould a different cavity from the one the user
    /// was shown — silently, and with the fit check's ✔ still on screen.
    #[test]
    fn a_design_files_cavity_inset_does_not_override_the_shaped_piece() {
        // DESIGN_TOML says 5 mm; make the file disagree with the plug.
        let toml = DESIGN_TOML.replace("inset_m = 0.005", "inset_m = 0.012");
        let (project, message) = design_file_onto("inset-clash", &toml, shaped_at_5mm());

        assert_eq!(
            project.design().map(|d| d.cavity_inset_m),
            project.plug().map(|p| p.cavity_inset_m),
            "the two insets must not be allowed to differ",
        );
        assert!(
            message.contains("5.0 mm cavity inset") && message.contains("12.0 mm"),
            "and the message names both what was used and what was dropped: {message}",
        );
    }

    /// ★ A `.design.toml` carries the inset to 1 µm, and the tools that write
    /// one do it from float sliders — `round_to_micrometers` exists in
    /// `cf-device-types` because of the IEEE-754 noise that produces. Compared
    /// as `f64`s, the note fires on a difference nobody can see and then reads
    /// "Its 5.0 mm cavity inset was not used" right after "5.0 mm cavity
    /// inset". It is compared as the message prints it instead.
    #[test]
    fn an_inset_difference_too_small_to_show_is_not_reported() {
        assert_eq!(
            format_ignored_inset(0.005_01, 0.005),
            None,
            "5.01 mm and 5.0 mm both print as 5.0",
        );
        let seen = format_ignored_inset(0.005_5, 0.005).unwrap_or_default();
        assert!(
            seen.contains("5.5 mm"),
            "but a difference that shows is reported, and names the file's own \
             number: {seen}",
        );
    }

    /// A file this app wrote carries the inset it wrote from, so the common
    /// case says nothing extra — or the warning stops meaning anything.
    #[test]
    fn a_design_file_that_agrees_with_the_piece_is_reported_plainly() {
        let (_, message) = design_file_onto("inset-agrees", DESIGN_TOML, shaped_at_5mm());

        assert_eq!(
            message, "✔ Design set: 1 layer(s), 5.0 mm cavity inset.",
            "nothing was dropped, so nothing is said about it",
        );
    }

    #[test]
    fn apply_prep_then_design_completes_steps_2_and_3() {
        let d = dir("flow");
        let stl = d.join("s.stl");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        let design = d.join("x.design.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();
        std::fs::write(&design, DESIGN_TOML).unwrap();

        let mut p = Project::new("t");
        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();
        assert!(p.is_complete(Step::CleanScan));
        // ⚠ 5 mm, matching DESIGN_TOML's own `inset_m`. A plug that
        // disagreed would put the ignored-inset note on every message
        // these fixtures produce, about a clash they are not testing.
        apply_plug(&mut p, shaped_at_5mm()).unwrap();
        let msg = apply_design(&mut p, &design).unwrap();
        // Exact, not `contains`: a fixture whose plug drifts off DESIGN_TOML's
        // own inset appends the ignored-inset note here, and a substring match
        // stays green while the fixture quietly stops being the case it names.
        assert_eq!(msg, "✔ Design set: 1 layer(s), 5.0 mm cavity inset.");
        assert!(p.is_complete(Step::DesignLayers));

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn apply_design_draft_completes_step_3() {
        use cf_studio_core::LayerDraft;

        let d = dir("draftdesign");
        let stl = d.join("s.stl");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();

        let mut p = Project::new("t");
        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();
        apply_plug(
            &mut p,
            cf_studio_core::PlugDraft {
                cavity_inset_m: 0.005,
                ..cf_studio_core::PlugDraft::default()
            },
        )
        .unwrap();

        // A stack built in-app (the layer-stack editor's output). It carries no
        // inset — the funnel takes that off the plug above, which is the whole
        // reason `Project::plug` and `Project::design` cannot come apart.
        let msg = apply_design_draft(
            &mut p,
            vec![LayerDraft {
                thickness_m: 0.0175,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            }],
        )
        .unwrap();
        assert!(msg.contains("5.0 mm cavity inset"), "got: {msg}");
        assert!(p.is_complete(Step::DesignLayers));
        assert_eq!(
            p.design().map(|d| d.cavity_inset_m),
            p.plug().map(|plug| plug.cavity_inset_m),
            "the stack took the shaped piece's inset, not one of its own",
        );

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn apply_prep_before_scan_is_rejected() {
        let d = dir("order");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();

        let mut p = Project::new("t");
        assert!(apply_prep(&mut p, &cleaned, &prep).is_err());

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn print_step_summary_reflects_project_state() {
        use cf_studio_core::{MoldOutputs, PourPlan, PrintExport};

        let d = dir("printsummary");
        let stl = d.join("s.stl");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        let design = d.join("x.design.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();
        std::fs::write(&design, DESIGN_TOML).unwrap();

        let mut p = Project::new("t");
        // Fresh / pre-molds: nothing to show.
        assert_eq!(print_step_summary(&p), "");

        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();
        // ⚠ 5 mm, matching DESIGN_TOML's own `inset_m`. A plug that
        // disagreed would put the ignored-inset note on every message
        // these fixtures produce, about a clash they are not testing.
        apply_plug(&mut p, shaped_at_5mm()).unwrap();
        apply_design(&mut p, &design).unwrap();

        // Molds made, not yet exported → "ready to save N".
        p.set_molds(MoldOutputs {
            out_dir: PathBuf::from("/tmp/out"),
            mold_stls: vec![PathBuf::from("a.stl"), PathBuf::from("b.stl")],
            plug_stls: vec![PathBuf::from("p.stl")],
            accessory_stls: vec![PathBuf::from("plat.stl")],
            procedure_path: PathBuf::from("proc.md"),
            total_mass_g: 100.0,
            pour_plan: PourPlan { steps: vec![] },
        })
        .unwrap();
        let s = print_step_summary(&p);
        assert!(
            s.contains("Ready to save the 4 part(s) this cast made"),
            "got: {s}"
        );

        // Exported → "saved to <dir>" takes precedence.
        p.set_print(PrintExport {
            export_dir: PathBuf::from("/tmp/print-out"),
        })
        .unwrap();
        let s = print_step_summary(&p);
        assert!(s.starts_with("✔ Saved to /tmp/print-out"), "got: {s}");

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn pot_life_converts_minutes_to_seconds() {
        // The factor, pinned. `+ 60` would give 85 s and `/ 60` would give 0 —
        // both plausible-looking numbers on a countdown, both ruinous.
        assert_eq!(
            pot_life_duration(25),
            Duration::from_secs(1500),
            "25 minutes of working time is 1500 seconds"
        );
        assert_eq!(pot_life_duration(1), Duration::from_secs(60));
        assert_eq!(
            pot_life_duration(0),
            Duration::ZERO,
            "a zero pot life is already expired, not unlimited"
        );
    }

    fn sample_plan() -> PourPlan {
        use cf_studio_core::PourStep;
        PourPlan {
            steps: vec![
                PourStep {
                    layer_index: 0,
                    material_display_name: "Ecoflex 00-30".to_string(),
                    mass_g: 500.0,
                    mix_ratio_a_to_b: "1:1".to_string(),
                    pot_life_minutes: 25,
                    cure_time_hours: 4.0,
                    slacker_fraction: Some(0.25),
                },
                PourStep {
                    layer_index: 1,
                    material_display_name: "Dragon Skin 20A".to_string(),
                    mass_g: 250.0,
                    mix_ratio_a_to_b: "1:1".to_string(),
                    pot_life_minutes: 25,
                    cure_time_hours: 5.0,
                    slacker_fraction: None,
                },
            ],
        }
    }

    #[test]
    fn pour_plan_text_numbers_layers_and_drops_last_cure() {
        let s = format_pour_plan(&sample_plan());
        assert!(s.contains("2 layer(s)"), "got: {s}");
        // 500 g cavity-fill mix, 25% Slacker → base 400 g (200 A + 200 B) +
        // 100 g Slacker = 500 g (no overfill), NOT 500 g base + 125 g Slacker.
        assert!(
            s.contains(
                "1. Ecoflex 00-30 — 200 g A + 200 g B + 100 g Slacker = 500 g mix (25% of base) · pot life ~25 min · cure ~4 h"
            ),
            "got: {s}"
        );
        // Last layer: no cure clause, no slacker.
        assert!(s.contains("2. Dragon Skin 20A — 250 g, mix 1:1 · pot life ~25 min"));
        assert!(
            !s.contains("cure ~5 h"),
            "last layer's cure is dropped: {s}"
        );
        assert_eq!(
            format_pour_plan(&PourPlan { steps: vec![] }),
            "(no pour layers)"
        );
    }

    #[test]
    fn pour_active_targets_the_current_layer() {
        let plan = sample_plan();
        let a0 = format_pour_active(&plan, 0);
        assert!(a0.starts_with("Layer 1 of 2 — Ecoflex 00-30"), "got: {a0}");
        let a1 = format_pour_active(&plan, 1);
        assert!(
            a1.starts_with("Layer 2 of 2 — Dragon Skin 20A"),
            "got: {a1}"
        );
        // Past the end → empty (all poured).
        assert_eq!(format_pour_active(&plan, 2), "");

        // ⚠ `is_last` decides whether " · cure ~N h" is appended, and the
        // asserts above are all `starts_with`, so they never reach the suffix.
        // Two mutants survived on that: inverting `==` to `!=`, and `current + 1`
        // to `current * 1`. Either one tells the user to wait a cure period
        // after the FINAL layer, or drops the wait BETWEEN layers — a pour
        // instruction, not a cosmetic string.
        assert!(
            a0.contains("cure ~4 h"),
            "a non-last layer must state its cure wait: {a0}"
        );
        assert!(
            !a1.contains("cure ~"),
            "the last layer has nothing to cure before: {a1}"
        );
    }

    #[test]
    fn countdown_warns_then_expires() {
        let ok = pour_countdown(20 * 60); // 20 min
        assert_eq!(ok.urgency, 0);
        assert!(ok.text.contains("20:00"), "got: {}", ok.text);

        let warn = pour_countdown(4 * 60 + 30); // 4:30 — under 5 min
        assert_eq!(warn.urgency, 1);
        assert!(warn.text.contains("4:30"), "got: {}", warn.text);

        assert_eq!(
            pour_countdown(300).urgency,
            1,
            "5 min is the warn threshold"
        );
        assert_eq!(pour_countdown(301).urgency, 0);

        let dead = pour_countdown(0);
        assert_eq!(dead.urgency, 2);
        assert!(dead.text.contains("time's up"), "got: {}", dead.text);
        assert_eq!(pour_countdown(-10).urgency, 2, "negative = expired");
    }

    fn full_controls() -> RidgeControls {
        RidgeControls {
            enabled: true,
            rings_enabled: true,
            rings: vec![RidgeRing {
                position_frac: 0.4,
                depth_m: 0.002,
                half_width_frac: 0.04,
            }],
            texture_enabled: true,
            texture_depth_m: 0.0015,
            texture_spacing_m: 0.008,
            side_pinch_enabled: true,
            side_pinch_depth_m: 0.0015,
            tip_relief_enabled: true,
            tip_relief_depth_m: 0.003,
            orientation_enabled: true,
            orientation_deg: 30.0,
        }
    }

    #[test]
    fn gate_ridge_options_passes_everything_when_all_on() {
        let o = gate_ridge_options(full_controls());
        assert!(o.enabled);
        assert_eq!(o.rings.len(), 1);
        assert_eq!(o.texture_depth_m, 0.0015);
        assert_eq!(o.side_pinch_depth_m, 0.0015);
        assert_eq!(o.tip_relief_depth_m, 0.003);
        assert_eq!(o.orientation_deg, 30.0);
    }

    #[test]
    fn gate_ridge_options_zeroes_each_disabled_feature_independently() {
        // Each toggle off drops ONLY its own feature — the others pass through.
        let rings_off = gate_ridge_options(RidgeControls {
            rings_enabled: false,
            ..full_controls()
        });
        assert!(rings_off.rings.is_empty(), "rings dropped");
        assert_eq!(rings_off.texture_depth_m, 0.0015, "texture untouched");

        let texture_off = gate_ridge_options(RidgeControls {
            texture_enabled: false,
            ..full_controls()
        });
        assert_eq!(texture_off.texture_depth_m, 0.0, "texture dropped");
        assert_eq!(texture_off.rings.len(), 1, "rings untouched");
        // Spacing is carried regardless (inert when depth is 0).
        assert_eq!(texture_off.texture_spacing_m, 0.008);

        let pinch_off = gate_ridge_options(RidgeControls {
            side_pinch_enabled: false,
            ..full_controls()
        });
        assert_eq!(pinch_off.side_pinch_depth_m, 0.0);
        assert_eq!(pinch_off.tip_relief_depth_m, 0.003, "tip relief untouched");

        let relief_off = gate_ridge_options(RidgeControls {
            tip_relief_enabled: false,
            ..full_controls()
        });
        assert_eq!(relief_off.tip_relief_depth_m, 0.0);

        let orient_off = gate_ridge_options(RidgeControls {
            orientation_enabled: false,
            ..full_controls()
        });
        assert_eq!(orient_off.orientation_deg, 0.0);
        assert_eq!(orient_off.side_pinch_depth_m, 0.0015, "pinch untouched");
    }

    #[test]
    fn quality_index_maps_to_cell_size() {
        // Index 0 (the picker default) must be the 0.5 mm print quality;
        // this is the mapping that was wrong once already.
        assert_eq!(
            cell_size_m_for_quality(0),
            0.0005,
            "default = print quality"
        );
        assert_eq!(cell_size_m_for_quality(1), 0.0015, "fast preview");
        // Out-of-range indices fall back to the safe print-quality default.
        assert_eq!(cell_size_m_for_quality(99), 0.0005);
        assert_eq!(cell_size_m_for_quality(-1), 0.0005);
    }

    #[test]
    fn enumerate_parts_detachable_lists_a_plug_per_layer() {
        let parts = enumerate_parts(2, CastMode::Detachable);
        // 2 layers × (2 cups + 1 plug) + platform + dowels = 8.
        assert_eq!(parts.len(), 2 * 3 + 2);
        assert_eq!(parts[0].1, "Layer 1 — cup (left)");
        assert_eq!(parts[2].1, "Layer 1 — plug");
        assert_eq!(parts[5].1, "Layer 2 — plug");
        assert_eq!(parts[6].0, PartId::Platform);
        assert_eq!(parts[7].0, PartId::Dowel);
    }

    #[test]
    fn enumerate_parts_bonded_lists_only_the_layer0_plug() {
        let parts = enumerate_parts(3, CastMode::Bonded);
        // 3 layers × 2 cups + 1 plug (layer 0 only) + platform + dowels = 9.
        assert_eq!(parts.len(), 3 * 2 + 1 + 2);
        let plugs: Vec<_> = parts
            .iter()
            .filter(|(id, _)| matches!(id, PartId::Plug { .. }))
            .collect();
        assert_eq!(plugs.len(), 1, "bonded lists only one plug");
        assert_eq!(plugs[0].0, PartId::Plug { layer_index: 0 });
    }

    #[test]
    fn all_checked_detachable_yields_the_full_selection() {
        let parts = enumerate_parts(2, CastMode::Detachable);
        let checked = vec![true; parts.len()];
        let sel = part_selection_from_checks(&parts, &checked, CastMode::Detachable);
        assert!(sel.is_all(), "all checked → the validated full-cast path");
    }

    #[test]
    fn all_checked_bonded_is_not_the_full_selection() {
        let parts = enumerate_parts(2, CastMode::Bonded);
        let checked = vec![true; parts.len()];
        let sel = part_selection_from_checks(&parts, &checked, CastMode::Bonded);
        assert!(
            !sel.is_all(),
            "bonded never routes to the full detachable export"
        );
        assert!(sel.includes(PartId::Plug { layer_index: 0 }));
        assert!(
            !sel.includes(PartId::Plug { layer_index: 1 }),
            "no layer-1 plug"
        );
    }

    /// Cast the real `base_mold` at `cell_size_m`, with the selection the app
    /// sends.
    ///
    /// ⚠ Bonded had never been cast before 2026-09-06: all five `#[ignore]`d gates
    /// in `cf-studio-engine` hard-code `Detachable`. Lives here because only here
    /// is the selection built as the app builds it — an inclusion set omitting the
    /// gasket and funnel, unlike `cf-cast`'s `all_except`.
    fn cast_base_mold_as_the_app_would(cell_size_m: f64, label: &str) {
        use cf_studio_core::{DesignDraft, LayerDraft};

        // ⚠ Never `~/scans` in place: `base_dir` would be the user's scan
        // folder, so the run overwrites `base_mold.design.toml` and leaves its
        // output beside it. Both happened — 217 MB, and the design file
        // rewritten next to a `.pre-gate-bak` from the last time.
        let (dir, cleaned, prep) = isolated_base_mold_copy(label);

        // The stack the GUI opens on, as the engine's siblings use.
        let draft = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![
                LayerDraft {
                    thickness_m: 0.018,
                    material_key: "ECOFLEX_00_30".to_string(),
                    slacker_fraction: 0.25,
                },
                LayerDraft {
                    thickness_m: 0.007,
                    material_key: "DRAGON_SKIN_10A".to_string(),
                    slacker_fraction: 0.0,
                },
                LayerDraft {
                    thickness_m: 0.005,
                    material_key: "DRAGON_SKIN_20A".to_string(),
                    slacker_fraction: 0.0,
                },
            ],
        };

        // Exactly what the screen sends: every offered part checked.
        let picker = PartPicker::rebuild(draft.layers.len(), CENDRILLON_CAST_MODE);
        assert!(picker.any_checked(), "a fresh picker is a full cast");
        let selection = picker.selection(CENDRILLON_CAST_MODE);

        let out = cf_studio_engine::generate_molds_for_design(
            &cf_studio_core::PrepInput {
                cleaned_stl: cleaned.clone(),
                prep_toml: prep.clone(),
            },
            &draft,
            cell_size_m,
            &RidgeOptions::default(),
            &selection,
            CENDRILLON_CAST_MODE,
            None,
        )
        .expect("the bonded path must cast");

        assert_eq!(out.mold_stls.len(), 6, "2 halves × 3 layers");
        assert_eq!(
            out.plug_stls.len(),
            1,
            "bonded casts ONE plug — the cured layer N is the plug for N+1"
        );
        assert_eq!(
            out.pour_plan.steps.len(),
            3,
            "the plan still covers 3 layers"
        );
        assert!(out.total_mass_g > 0.0);

        // ⚠ After the assertions, never on the failure path — a red gate keeps
        // its output for inspection.
        if let Err(err) = std::fs::remove_dir_all(&dir) {
            eprintln!("WARN: could not remove fixture {}: {err}", dir.display());
        }
    }

    /// A private copy of the `base_mold` fixture, so the cast's `base_dir` is never
    /// the user's `~/scans`.
    ///
    /// ⚠ PANICS on a missing fixture rather than skipping — these run only when
    /// asked for by name, so a silent pass would report the path as cast.
    /// ▶ A near-copy of the engine's `isolated_base_mold_fixture`; both live in
    /// `#[cfg(test)]`, so the standing fix is a shared test-support module.
    fn isolated_base_mold_copy(label: &str) -> (PathBuf, PathBuf, PathBuf) {
        let scans = PathBuf::from(std::env::var("HOME").expect("HOME")).join("scans");
        let (src_stl, src_prep) = (
            scans.join("base_mold.cleaned.stl"),
            scans.join("base_mold.prep.toml"),
        );
        assert!(
            src_stl.exists() && src_prep.exists(),
            "MISSING FIXTURE: {} and {} are required by this #[ignore]d gate",
            src_stl.display(),
            src_prep.display(),
        );
        let dir =
            std::env::temp_dir().join(format!("cf-studio-gui-{label}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a fixture dir");
        let (stl, prep) = (
            dir.join("base_mold.cleaned.stl"),
            dir.join("base_mold.prep.toml"),
        );
        std::fs::copy(&src_stl, &stl).expect("copy the cleaned scan");
        std::fs::copy(&src_prep, &prep).expect("copy the prep");
        (dir, stl, prep)
    }

    /// The fast preview — the picker's index 1.
    ///
    /// ★★ **PASSED 2026-09-06: 407.67 s.** Detachable, same cell size and machine:
    /// 277.95 s. Bonded is slower while producing fewer pieces — it never collapses
    /// to `PartSelection::all`, so it meshes piece-by-piece.
    #[test]
    #[ignore = "integration: 408 s at 1.5 mm (measured 2026-09-06), needs ~/scans/base_mold files"]
    fn the_app_casts_base_mold_bonded() {
        cast_base_mold_as_the_app_would(0.0015, "bonded");
    }

    /// Print quality — the picker's index 0, and what the user prints from.
    ///
    /// ★★ **PASSED 2026-09-06: 2187.37 s (36.5 min).** Step 5's copy had said
    /// "around fifteen minutes", measured on the detachable path the app does not
    /// use. ⚠ Bonded is 1.47× detachable at 1.5 mm but **2.43×** at 0.5 — the ratio
    /// does not hold across cell sizes, so scaling would have written ~22 minutes.
    #[test]
    #[ignore = "integration: 2187 s / 36 min at 0.5 mm (measured 2026-09-06), needs ~/scans/base_mold files"]
    fn the_app_casts_base_mold_bonded_fine() {
        cast_base_mold_as_the_app_would(0.0005, "bonded-fine");
    }

    // ── the cast mode the app pins ──────────────────────────────────────────
    //
    // ⚠ These read `CENDRILLON_CAST_MODE`, never a `CastMode` literal. The two
    // gates above pin what Bonded *does*; only these pin that the app is in
    // it — and they are the gates that were missing for the whole port, while
    // both of those stayed green over an app committed to no mode at all.

    #[test]
    fn cendrillon_offers_one_plug_however_many_layers_the_design_has() {
        // Swept, not sampled: a one-layer design offers one plug in EITHER
        // mode, so a single-layer check would pass just as happily on
        // detachable.
        for layer_count in 1..=4 {
            let parts = enumerate_parts(layer_count, CENDRILLON_CAST_MODE);
            let plugs: Vec<_> = parts
                .iter()
                .filter(|(id, _)| matches!(id, PartId::Plug { .. }))
                .collect();
            assert_eq!(
                plugs.len(),
                1,
                "a {layer_count}-layer design must still offer exactly one plug"
            );
            assert_eq!(plugs[0].0, PartId::Plug { layer_index: 0 });
        }
    }

    #[test]
    fn cendrillon_never_takes_the_full_cast_shortcut() {
        let parts = enumerate_parts(3, CENDRILLON_CAST_MODE);
        let checked = vec![true; parts.len()];
        let sel = part_selection_from_checks(&parts, &checked, CENDRILLON_CAST_MODE);
        assert!(
            !sel.is_all(),
            "everything-checked must still route the selective bonded path"
        );
        // ★ The other half of the claim. `!is_all()` alone is also true of an
        // EMPTY selection, which would cast nothing at all — so assert the
        // collection, not just the negation.
        for (id, _) in &parts {
            assert!(
                sel.includes(*id),
                "{id:?} was checked, so it must be selected"
            );
        }
    }

    #[test]
    fn picker_selection_keeps_rows_aligned_with_their_checkboxes() {
        // `selection` builds the parts vec and the checked vec in two separate
        // passes over `rows`, and `part_selection_from_checks` pairs them BY
        // INDEX. Nothing in the type system holds those two in step, so a
        // misalignment would quietly export the wrong pieces. Mirrors
        // `subset_selects_only_checked_parts` but drives it through the picker.
        let mut picker = PartPicker::rebuild(2, CastMode::Detachable);
        picker.set_all(false);
        picker.set_checked(2, true); // "Layer 1 — plug" → Plug { layer_index: 0 }

        let sel = picker.selection(CastMode::Detachable);
        assert!(!sel.is_all(), "one checked row is not the full cast");
        assert!(
            sel.includes(PartId::Plug { layer_index: 0 }),
            "the checked row's OWN part must be selected"
        );
        assert!(
            !sel.includes(PartId::Plug { layer_index: 1 }),
            "an off-by-one in the zip would select this instead"
        );

        // ★ The other half of the same claim: what the picker RENDERS must
        // agree with what it EXPORTS. Asserting only `selection` leaves a
        // `rows()` that disagrees with it invisible.
        let checked: Vec<bool> = picker.rows().map(|(_, c)| c).collect();
        assert_eq!(
            checked,
            [false, false, true, false, false, false, false, false],
            "exactly the row set_checked(2) touched may render as checked"
        );
    }

    #[test]
    fn picker_all_checked_matches_the_direct_call() {
        // The all-checked collapse to `PartSelection::all` must survive the
        // adapter, not just the function it delegates to.
        let picker = PartPicker::rebuild(2, CastMode::Detachable);
        assert!(picker.selection(CastMode::Detachable).is_all());
    }

    #[test]
    fn subset_selects_only_checked_parts() {
        let parts = enumerate_parts(2, CastMode::Detachable);
        // Check only "Layer 1 — plug" (index 2).
        let mut checked = vec![false; parts.len()];
        checked[2] = true;
        let sel = part_selection_from_checks(&parts, &checked, CastMode::Detachable);
        assert!(!sel.is_all());
        assert!(sel.includes(PartId::Plug { layer_index: 0 }));
        assert!(!sel.includes(PartId::Plug { layer_index: 1 }));
        assert!(!sel.includes(PartId::Platform));
        assert!(!sel.includes(PartId::Cup {
            layer_index: 0,
            side: PieceSide::Negative
        }));
    }

    /// A folder carrying no record is UNKNOWN, and must say so. Silence here
    /// is the original bug: a stale part that looks exactly like a fresh one.
    #[test]
    fn a_folder_with_no_record_says_unknown_rather_than_nothing() {
        let note = format_stale_parts(None, Path::new("/tmp/scans/session-7/out"))
            .expect("silence would be the bug");
        assert!(note.contains("no record"), "got: {note}");
        assert!(
            note.contains("/tmp/scans/session-7/out"),
            "it must name the folder it read, or step 6 reads it as a claim \
             about the print folder: {note}"
        );
        // ⚠ And it must not name a CAUSE. `folder_provenance` reaches `None`
        // down four paths — no manifest, an unparseable one, a `latest_run`
        // of `UNKNOWN_RUN`, an unlistable folder — and three of those have
        // their own gates. Any single explanation here is wrong three ways.
        assert!(
            !note.contains("written before"),
            "the note explains a cause it has not established: {note}"
        );
    }

    /// Nothing stale, nothing said — a note on every cast is one nobody reads.
    #[test]
    fn a_current_folder_says_nothing() {
        let current = RunProvenance {
            run: 3,
            stale: Vec::new(),
        };
        assert_eq!(
            format_stale_parts(Some(&current), Path::new("/tmp/out")),
            None
        );
    }

    /// ⚠ "Never recorded" and "run 1" are different facts and the line keeps
    /// them apart: one part is of unknown vintage, the other is merely two
    /// casts old, and deleting them is a different decision.
    #[test]
    fn the_note_names_each_part_and_distinguishes_unknown_from_older() {
        let provenance = RunProvenance {
            run: 3,
            stale: vec![
                ManifestEntry {
                    file: "plug_layer_1.stl".to_string(),
                    run: UNKNOWN_RUN,
                },
                ManifestEntry {
                    file: "platform.stl".to_string(),
                    run: 1,
                },
            ],
        };

        let note = format_stale_parts(Some(&provenance), Path::new("/tmp/scans/session-7/out"))
            .expect("two are stale");

        assert!(note.contains("2 part(s)"), "counts them: {note}");
        // ⚠ The referent, asserted rather than trusted to wording. The roster
        // is read from the CAST's folder and this line is also drawn on step
        // 6, where the screen is about a print folder the user picked.
        assert!(
            note.contains("/tmp/scans/session-7/out"),
            "it must name the folder it read: {note}"
        );
        assert!(note.contains("(run 3)"), "names the latest run: {note}");
        assert!(
            note.contains("plug_layer_1.stl — never recorded"),
            "unknown vintage: {note}"
        );
        assert!(
            note.contains("platform.stl — run 1"),
            "merely older: {note}"
        );
    }

    /// A save that replaced everything already there says nothing extra — a
    /// warning on every save is one nobody reads.
    #[test]
    fn a_destination_holding_only_this_saves_files_says_nothing() {
        assert_eq!(format_print_destination_note(4, Some(4)), None);
    }

    /// The number that matters is what the slicer opens, not what was copied.
    #[test]
    fn a_destination_holding_more_names_both_numbers() {
        let note = format_print_destination_note(1, Some(5)).expect("four are left over");
        assert!(
            note.contains("holds 5 printable file(s)"),
            "what the folder holds: {note}"
        );
        assert!(
            note.contains("4 left by an earlier save"),
            "how many this save did not write: {note}"
        );
    }

    /// ⚠ An unreadable folder is not an empty one. `None` must stay quiet
    /// rather than resolve to a count — the same conflation that made an
    /// all-unknown folder report as current one PR ago.
    #[test]
    fn an_unreadable_destination_says_nothing_rather_than_guessing() {
        assert_eq!(format_print_destination_note(4, None), None);
    }

    /// Fewer files than were copied cannot mean "leftovers", and must not
    /// underflow on the way to saying so.
    #[test]
    fn a_destination_smaller_than_the_copy_says_nothing() {
        assert_eq!(format_print_destination_note(4, Some(2)), None);
    }

    #[test]
    fn molds_summary_lists_counts_mass_and_pours() {
        use cf_studio_core::{PourPlan, PourStep};

        let out = MoldOutputs {
            out_dir: PathBuf::from("/tmp/scans/out"),
            mold_stls: vec![PathBuf::from("a.stl"), PathBuf::from("b.stl")],
            plug_stls: vec![PathBuf::from("p.stl")],
            accessory_stls: vec![PathBuf::from("platform.stl")],
            procedure_path: PathBuf::from("procedure.md"),
            total_mass_g: 842.0,
            pour_plan: PourPlan {
                steps: vec![PourStep {
                    layer_index: 0,
                    material_display_name: "Ecoflex 00-30".to_string(),
                    mass_g: 500.0,
                    mix_ratio_a_to_b: "1:1".to_string(),
                    pot_life_minutes: 25,
                    cure_time_hours: 4.0,
                    slacker_fraction: Some(0.25),
                }],
            },
        };
        let s = format_molds_summary(&out);
        assert!(s.contains("2 mold piece(s) + 1 plug(s)"), "got: {s}");
        assert!(s.contains("1 accessory part(s)"), "got: {s}");
        assert!(s.contains("842 g across 1 pour(s)"), "got: {s}");
        // 1-based layer label, display name, grams, pot life.
        assert!(
            s.contains("Layer 1: Ecoflex 00-30 — 500 g (pot life ~25 min)"),
            "got: {s}"
        );
        assert!(s.contains("Saved to: /tmp/scans/out"), "got: {s}");
    }

    #[test]
    fn molds_summary_omits_accessories_when_none() {
        use cf_studio_core::PourPlan;

        let out = MoldOutputs {
            out_dir: PathBuf::from("/tmp/out"),
            mold_stls: vec![PathBuf::from("a.stl")],
            plug_stls: vec![],
            accessory_stls: vec![],
            procedure_path: PathBuf::from("p.md"),
            total_mass_g: 0.0,
            pour_plan: PourPlan { steps: vec![] },
        };
        let s = format_molds_summary(&out);
        assert!(!s.contains("accessory"), "no accessory clause: {s}");
        assert!(s.contains("0 pour(s)"), "got: {s}");
    }

    #[test]
    fn next_gate_opens_only_after_the_step_completes() {
        let d = dir("nav");
        let stl = d.join("s.stl");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();

        let mut p = Project::new("t");
        let before = nav_state(&p, Step::AddScan);
        assert!(!before.can_back, "no Back on the first screen");
        assert!(!before.can_next, "no Next until the scan is added");

        apply_scan(&mut p, &stl).unwrap();
        assert!(
            nav_state(&p, Step::AddScan).can_next,
            "Next opens once scan is done"
        );
        assert!(
            nav_state(&p, Step::CleanScan).can_back,
            "Back available off the first screen"
        );
        assert!(
            !nav_state(&p, Step::Pour).can_next,
            "no Next on the final screen"
        );

        let _ = std::fs::remove_dir_all(&d);
    }
    // ── StepBoxState ────────────────────────────────────────────────────────
    // The four rules from the Slint `StepBox`, each with the jarring behaviour
    // it exists to prevent named in the assertion message.

    #[test]
    fn typing_does_not_clamp_so_overshoot_is_not_snapped_mid_edit() {
        let mut s = StepBoxState::new(10);
        *s.text_mut() = "250".to_string();
        s.on_typed();
        assert_eq!(
            s.value(),
            250,
            "clamping mid-type snaps the field to the bound the moment you \
             overshoot, which is jarring while still typing"
        );
        assert_eq!(
            s.commit(0, 30),
            Some(30),
            "commit is where clamping happens"
        );
        assert_eq!(s.text(), "30", "commit re-renders the clamped value");
    }

    #[test]
    fn blurring_a_clean_field_commits_nothing() {
        let mut s = StepBoxState::new(10);
        assert_eq!(
            s.commit(0, 30),
            None,
            "a bare click-in/out must not trigger a re-mesh"
        );
    }

    #[test]
    fn blurring_a_dirty_field_commits_once() {
        let mut s = StepBoxState::new(10);
        *s.text_mut() = "12".to_string();
        s.on_typed();
        assert_eq!(s.commit(0, 30), Some(12));
        assert_eq!(s.commit(0, 30), None, "the second blur is already clean");
    }

    #[test]
    fn retyping_the_same_number_commits_nothing() {
        let mut s = StepBoxState::new(10);
        *s.text_mut() = "10".to_string();
        s.on_typed();
        assert!(s.is_dirty(), "the text changed, so the edit is pending");
        assert_eq!(
            s.commit(0, 30),
            None,
            "but the VALUE did not change — no re-mesh"
        );
    }

    #[test]
    fn empty_text_keeps_the_old_value() {
        let mut s = StepBoxState::new(17);
        *s.text_mut() = String::new();
        s.on_typed();
        assert_eq!(s.value(), 17, "clearing the box must not read as zero");
        assert_eq!(
            s.commit(0, 30),
            None,
            "and re-rendering the old value is not a change"
        );
        assert_eq!(s.text(), "17", "the box refills rather than staying blank");
    }

    #[test]
    fn unparseable_text_keeps_the_old_value() {
        let mut s = StepBoxState::new(5);
        *s.text_mut() = "3o".to_string();
        s.on_typed();
        assert_eq!(s.value(), 5);
        assert_eq!(s.commit(0, 30), None);
    }

    #[test]
    fn stepping_clamps_at_both_bounds_and_reports_only_real_moves() {
        let mut s = StepBoxState::new(0);
        assert_eq!(s.step(-1, 0, 30), None, "already at min");
        assert_eq!(s.step(1, 0, 30), Some(1));
        let mut s = StepBoxState::new(30);
        assert_eq!(s.step(1, 0, 30), None, "already at max");
        assert_eq!(s.step(-1, 0, 30), Some(29));
    }

    #[test]
    fn stepping_clears_a_pending_edit() {
        let mut s = StepBoxState::new(10);
        *s.text_mut() = "999".to_string();
        s.on_typed();
        assert_eq!(s.step(1, 0, 30), Some(30), "the ± button commits + clamps");
        assert!(!s.is_dirty());
    }

    #[test]
    fn sync_external_discards_a_pending_edit_because_the_model_wins() {
        let mut s = StepBoxState::new(20);
        *s.text_mut() = "25".to_string();
        s.on_typed();
        s.sync_external(8, 0, 30);
        assert_eq!(s.value(), 8, "a shrunk trim bound overrides what was typed");
        assert_eq!(s.text(), "8");
        assert!(!s.is_dirty());
    }

    #[test]
    fn sync_external_clamps_because_the_bounds_may_have_moved_too() {
        let mut s = StepBoxState::new(25);
        // The mesh shrank, so the trim ceiling dropped from 30 to 10 and the
        // model is handing back a value that no longer fits.
        s.sync_external(25, 0, 10);
        assert_eq!(
            s.value(),
            10,
            "an unclamped sync leaves a value commit() can never correct — a \
             clean field returns early"
        );
        assert_eq!(s.text(), "10");
        assert_eq!(s.commit(0, 10), None, "and it is already settled");
    }

    // ── WizardCursor ────────────────────────────────────────────────────────

    #[test]
    fn no_next_on_the_final_screen_even_once_it_is_complete() {
        // ⚠ `nav_state`'s gate is `is_complete(viewed) && viewed != Step::LAST`.
        // `next_gate_opens_only_after_the_step_completes` asserts
        // `!nav_state(&p, Step::Pour).can_next` under the message "no Next on
        // the final screen" — but its project is INCOMPLETE at Pour, so the
        // FIRST conjunct already returns false and the LAST one never runs.
        // Deleting `&& viewed != Step::LAST` left that test green.
        //
        // Driving the wizard to genuinely finished is the only way to isolate
        // the second conjunct, because `Project` enforces step order.
        use cf_studio_core::{MoldOutputs, PourPlan, PourRecord, PrintExport};

        let d = dir("lastscreen");
        let stl = d.join("s.stl");
        let cleaned = d.join("c.stl");
        let prep = d.join("p.prep.toml");
        let design = d.join("x.design.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&cleaned, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();
        std::fs::write(&design, DESIGN_TOML).unwrap();

        let mut p = Project::new("t");
        apply_scan(&mut p, &stl).unwrap();
        apply_prep(&mut p, &cleaned, &prep).unwrap();
        // ⚠ 5 mm, matching DESIGN_TOML's own `inset_m`. A plug that
        // disagreed would put the ignored-inset note on every message
        // these fixtures produce, about a clash they are not testing.
        apply_plug(&mut p, shaped_at_5mm()).unwrap();
        apply_design(&mut p, &design).unwrap();
        p.set_molds(MoldOutputs {
            out_dir: PathBuf::from("/tmp/out"),
            mold_stls: vec![PathBuf::from("a.stl")],
            plug_stls: vec![PathBuf::from("p.stl")],
            accessory_stls: vec![],
            procedure_path: PathBuf::from("proc.md"),
            total_mass_g: 100.0,
            pour_plan: PourPlan { steps: vec![] },
        })
        .unwrap();
        p.set_print(PrintExport {
            export_dir: PathBuf::from("/tmp/print-out"),
        })
        .unwrap();
        p.set_pour(PourRecord { layers_poured: 1 }).unwrap();

        assert!(p.is_complete(Step::Pour), "the final step is now complete");
        assert!(
            !nav_state(&p, Step::Pour).can_next,
            "the wizard must not advance past its last screen"
        );

        // And at the cursor, which is what the button actually drives.
        let mut c = WizardCursor::new(Step::Pour);
        assert!(!c.next(&p), "Next on the last screen must not move");
        assert_eq!(c.viewed(), Step::Pour, "the cursor must stay put");

        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn back_from_the_first_screen_is_a_no_op() {
        let mut c = WizardCursor::default();
        assert_eq!(c.viewed(), Step::FIRST);
        c.back();
        assert_eq!(c.viewed(), Step::FIRST);
    }

    #[test]
    fn next_is_refused_while_the_viewed_step_is_incomplete() {
        let project = Project::new("t");
        let mut c = WizardCursor::default();
        assert!(
            !c.next(&project),
            "the gate must hold even if a disabled button fires"
        );
        assert_eq!(c.viewed(), Step::FIRST);
    }

    #[test]
    fn next_advances_once_the_viewed_step_completes() {
        use cf_studio_core::ScanInput;
        let mut project = Project::new("t");
        project.set_scan(ScanInput {
            source_path: PathBuf::from("/tmp/s.stl"),
        });
        let mut c = WizardCursor::default();
        assert!(c.next(&project));
        assert_eq!(c.viewed(), Step::CleanScan);
    }

    #[test]
    fn back_preserves_completed_work_and_next_returns() {
        use cf_studio_core::ScanInput;
        let mut project = Project::new("t");
        project.set_scan(ScanInput {
            source_path: PathBuf::from("/tmp/s.stl"),
        });
        let mut c = WizardCursor::new(Step::CleanScan);
        c.back();
        assert_eq!(c.viewed(), Step::AddScan);
        assert!(
            c.next(&project),
            "paging back over completed work must not invalidate it"
        );
        assert_eq!(c.viewed(), Step::CleanScan);
    }

    // ── PourSession ─────────────────────────────────────────────────────────

    #[test]
    fn advancing_an_empty_plan_does_nothing() {
        let mut s = PourSession::default();
        assert_eq!(s.advance(0), PourAdvance::NoPlan);
        assert_eq!(
            s.current(),
            0,
            "a project with no molds has no plan; the cursor must not move"
        );
    }

    #[test]
    fn advancing_mid_plan_reports_the_layer() {
        let mut s = PourSession::default();
        assert_eq!(s.advance(3), PourAdvance::Layer { poured: 1 });
        assert_eq!(s.advance(3), PourAdvance::Layer { poured: 2 });
        assert_eq!(s.current(), 2);
    }

    #[test]
    fn the_last_layer_completes_the_project() {
        let mut s = PourSession::default();
        s.advance(2);
        assert_eq!(s.advance(2), PourAdvance::Complete { layers_poured: 2 });
    }

    #[test]
    fn a_single_layer_plan_completes_on_the_first_advance() {
        let mut s = PourSession::default();
        assert_eq!(s.advance(1), PourAdvance::Complete { layers_poured: 1 });
    }

    // ── format_elapsed ──────────────────────────────────────────────────────

    #[test]
    fn elapsed_pads_seconds_and_rolls_minutes() {
        assert_eq!(format_elapsed(0), "0:00");
        assert_eq!(format_elapsed(9), "0:09");
        assert_eq!(format_elapsed(60), "1:00");
        assert_eq!(format_elapsed(61), "1:01");
        // The jobs this labels run 7-36 minutes; the minutes field is the part
        // an off-by-one would hide.
        assert_eq!(format_elapsed(15 * 60 + 7), "15:07");
    }

    #[test]
    fn the_cast_progress_line_carries_the_clock_and_the_reassurance() {
        // ⚠ Exact, not `contains`: the pre-port built this with a trailing `\`
        // continuation and emitted two spaces before the parenthesis, and
        // substring checks left every other word of the line free.
        assert_eq!(
            format_molds_progress(15 * 60 + 7),
            "Making molds… 15:07 elapsed (this can take a while — the window stays responsive)"
        );
    }

    // ── RingRow round-trip ──────────────────────────────────────────────────

    #[test]
    fn ring_row_round_trips_through_the_sdk_units() {
        let ring = RidgeRing {
            position_frac: 0.35,
            depth_m: 0.0012,
            half_width_frac: 0.08,
        };
        let row = RingRow::from_ridge(&ring);
        assert_eq!(row.position.value(), 35);
        assert_eq!(row.depth.value(), 12, "1.2 mm is 12 tenths");
        assert_eq!(row.width.value(), 8);

        let back = row.to_ridge();
        assert!((back.position_frac - 0.35).abs() < 1e-12);
        assert!((back.depth_m - 0.0012).abs() < 1e-12);
        assert!((back.half_width_frac - 0.08).abs() < 1e-12);
    }

    #[test]
    fn tenths_of_a_millimetre_convert_to_meters() {
        assert!(
            (tenths_mm_to_m(10) - 0.001).abs() < 1e-12,
            "10 tenths = 1 mm"
        );
        assert!((tenths_mm_to_m(0) - 0.0).abs() < 1e-12);
    }

    // ── the step-4 layer stack ──────────────────────────────────────────────

    /// The keys and integer fields of a stack, in the order it holds them.
    fn stack_census(stack: &LayerStack) -> Vec<(&'static str, i32, i32)> {
        stack
            .rows()
            .iter()
            .map(|row| {
                (
                    row.material.key,
                    row.thickness_mm.value(),
                    row.slacker_pct.value(),
                )
            })
            .collect()
    }

    /// ★ The whole opening stack, not a length and not a spot check:
    /// `LayerStack`'s `filter_map` silently *shortens* the stack for a
    /// silicone the catalog no longer carries. Asserting the collection is
    /// what turns that into a failure instead of a two-layer default nobody
    /// notices.
    #[test]
    fn the_screen_opens_on_the_pre_ports_stack() {
        let stack = LayerStack::default();

        assert_eq!(
            stack_census(&stack),
            vec![
                ("ECOFLEX_00_30", 18, 25),
                ("DRAGON_SKIN_10A", 8, 0),
                ("DRAGON_SKIN_20A", 5, 0),
            ],
            "soft and slacker-softened inside, firmer outward"
        );
    }

    #[test]
    fn a_layer_row_converts_to_the_sdks_units() {
        let row = LayerRow::new(
            Silicone::from_key("ECOFLEX_00_30").expect("the catalog carries it"),
            18,
            25,
        );

        let draft = row.draft();

        assert!((draft.thickness_m - 0.018).abs() < 1e-12, "18 mm in meters");
        assert!(
            (draft.slacker_fraction - 0.25).abs() < 1e-12,
            "25 % as a fraction"
        );
        assert_eq!(draft.material_key, "ECOFLEX_00_30");
    }

    /// ⚠ The order is the pour order — innermost first — so a stack that
    /// converted as a set would build the device inside out.
    #[test]
    fn the_drafts_keep_the_stacks_order() {
        let keys: Vec<String> = LayerStack::default()
            .drafts()
            .into_iter()
            .map(|draft| draft.material_key)
            .collect();

        assert_eq!(
            keys,
            ["ECOFLEX_00_30", "DRAGON_SKIN_10A", "DRAGON_SKIN_20A"]
        );
    }

    /// A layer stack in the SDK's units, from the given `(key, mm, pct)` rows.
    fn drafts_of(rows: &[(&str, f64, f64)]) -> Vec<LayerDraft> {
        rows.iter()
            .map(|&(key, thickness_mm, slacker_pct)| LayerDraft {
                thickness_m: thickness_mm / 1000.0,
                material_key: key.to_string(),
                slacker_fraction: slacker_pct / 100.0,
            })
            .collect()
    }

    /// A design in whole millimetres is shown exactly as it arrived — the case
    /// the "…or load a file" button has to get right, because the button
    /// beside the rows commits them.
    #[test]
    fn a_whole_millimetre_design_seeds_the_rows_unchanged() {
        let loaded = drafts_of(&[("ECOFLEX_00_30", 7.0, 25.0), ("DRAGON_SKIN_10A", 3.0, 0.0)]);
        let stack = LayerStack::from_drafts(&loaded).expect("a catalog silicone in whole mm");

        assert_eq!(
            stack_census(&stack),
            vec![("ECOFLEX_00_30", 7, 25), ("DRAGON_SKIN_10A", 3, 0)],
        );
        assert_eq!(stack.drafts(), loaded, "and nothing was changed to show it");
        assert_eq!(
            format_inexact_design(&stack, &loaded),
            None,
            "so there is nothing to warn about",
        );
    }

    /// ★★ `base_mold`'s own stack — the one physically validated cast — is
    /// **17.5 / 7.5 / 5 mm**, and this editor edits whole millimetres. The rows
    /// therefore cannot be the design, and the user has to be told before the
    /// button beside them rounds it.
    #[test]
    fn the_validated_stack_does_not_survive_this_editor_and_says_so() {
        let loaded = drafts_of(&[
            ("ECOFLEX_00_30", 17.5, 25.0),
            ("DRAGON_SKIN_10A", 7.5, 0.0),
            ("DRAGON_SKIN_20A", 5.0, 0.0),
        ]);
        let stack = LayerStack::from_drafts(&loaded).expect("every silicone is in the catalog");

        assert_eq!(
            stack_census(&stack),
            vec![
                ("ECOFLEX_00_30", 18, 25),
                ("DRAGON_SKIN_10A", 8, 0),
                ("DRAGON_SKIN_20A", 5, 0),
            ],
            "rounded to the nearest millimetre, not truncated",
        );
        assert_eq!(
            format_inexact_design(&stack, &loaded),
            Some(
                " \u{26a0} Not exactly this file \u{2014} the rows below are what \"Use this \
                 design\" would write."
                    .to_string()
            ),
            "and the user is told before the button beside those rows writes them",
        );
    }

    /// Both refusals. The rows are left as they were rather than showing a
    /// stack the editor made up — one it would then commit as the user's own.
    #[test]
    fn a_design_this_editor_cannot_show_builds_no_rows() {
        assert_eq!(
            LayerStack::from_drafts(&[]),
            None,
            "an empty stack is not a stack",
        );
        assert_eq!(
            LayerStack::from_drafts(&drafts_of(&[
                ("ECOFLEX_00_30", 5.0, 0.0),
                ("NOT_A_SILICONE", 5.0, 0.0),
            ])),
            None,
            "and one unknown silicone refuses the whole stack, not just its row",
        );
    }

    /// A `.design.toml` is bounded by neither of this editor's steppers, and
    /// the clamp is at construction rather than left to `BoundedField::value`:
    /// a row *showing* 0 mm that *commits* 1 mm is a screen telling the user
    /// something untrue. Asserted on the raw state for that reason — reading
    /// through `value()` would clamp either way and prove nothing.
    ///
    /// ★ The note claims no cause, because the steppers do three different
    /// things here. It said "rounded to the nearest millimetre" until a probe
    /// showed 150 mm being shown as 100 — a 50 mm discrepancy described as a
    /// sub-millimetre one, in the only message standing between the user and
    /// the overwrite.
    #[test]
    fn a_layer_outside_the_steppers_is_clamped_where_it_is_shown_and_flagged() {
        for (label, thickness_mm, slacker_pct, shown_mm, shown_pct) in [
            ("below the floor", 0.2, 400.0, 1, 100),
            ("above the ceiling", 150.0, 0.0, 100, 0),
        ] {
            let loaded = drafts_of(&[("ECOFLEX_00_30", thickness_mm, slacker_pct)]);
            let stack = LayerStack::from_drafts(&loaded).expect("the silicone is in the catalog");
            let row = &stack.rows()[0];

            assert_eq!(
                (
                    row.thickness_mm.state.value(),
                    row.slacker_pct.state.value()
                ),
                (shown_mm, shown_pct),
                "{label}: shown at the bound it will be used at",
            );
            let note = format_inexact_design(&stack, &loaded).unwrap_or_default();
            assert!(
                !note.contains("round") && note.contains("rows below"),
                "{label}: the note points at the rows, not at a cause: {note:?}",
            );
        }
    }

    #[test]
    fn dropping_a_layer_takes_the_one_the_index_names() {
        let mut stack = LayerStack::default();

        stack.remove(1);

        assert_eq!(
            stack_census(&stack),
            vec![("ECOFLEX_00_30", 18, 25), ("DRAGON_SKIN_20A", 5, 0)],
            "the middle layer went, and the outer one did not slide into it"
        );
    }

    /// ★ The cast needs a stack. The screen also disables the last ✖, but a
    /// rule that lives only in the button is a rule the next screen can break.
    #[test]
    fn the_last_layer_cannot_be_dropped() {
        let mut stack = LayerStack::default();
        stack.remove(2);
        stack.remove(1);

        stack.remove(0);

        assert_eq!(
            stack_census(&stack),
            vec![("ECOFLEX_00_30", 18, 25)],
            "the one layer left stayed, unchanged"
        );
    }

    /// ⚠ One past the end as well as far past it: `remove` indexes a `Vec`,
    /// and `index <= len` would reach `Vec::remove(len)`, which panics.
    #[test]
    fn dropping_a_layer_that_is_not_there_changes_nothing() {
        let untouched = stack_census(&LayerStack::default());

        for index in [3, 9] {
            let mut stack = LayerStack::default();

            stack.remove(index);

            assert_eq!(stack_census(&stack), untouched, "removing index {index}");
        }
    }

    /// ⚠ On the outside, where the button says it goes: the stack is built
    /// outward off the plug, so an added layer that landed innermost would
    /// change what every layer above it sits on.
    #[test]
    fn an_added_layer_lands_on_the_outside() {
        let mut stack = LayerStack::default();

        stack.add();

        assert_eq!(
            stack_census(&stack),
            vec![
                ("ECOFLEX_00_30", 18, 25),
                ("DRAGON_SKIN_10A", 8, 0),
                ("DRAGON_SKIN_20A", 5, 0),
                ("DRAGON_SKIN_10A", 5, 0),
            ]
        );
    }

    /// ⚠ The picker's list is the SDK's own, whole and in its order: this is
    /// where [`Silicone`] and the catalog it wraps are held together.
    #[test]
    fn the_picker_offers_the_sdks_catalog() {
        let offered: Vec<(&str, &str)> = Silicone::catalog()
            .into_iter()
            .map(|silicone| (silicone.key, silicone.name))
            .collect();

        assert_eq!(offered, silicone_catalog());
        assert!(!offered.is_empty(), "a picker with no silicones is not one");
    }

    #[test]
    fn a_silicone_is_found_by_its_key_and_only_a_real_one_is() {
        assert_eq!(
            Silicone::from_key("ECOFLEX_00_30").map(|s| s.name),
            Some("Ecoflex 00-30 (medium-soft)")
        );
        assert_eq!(Silicone::from_key("NOT_A_SILICONE"), None);
    }

    /// ⚠ Typing does not commit, so an in-flight number reaches `draft()`
    /// unclamped unless the field clamps on read. 500 mm of silicone is not a
    /// layer the cast should be asked for.
    #[test]
    fn a_thickness_typed_past_its_bound_is_read_back_inside_it() {
        let mut row = LayerRow::new(
            Silicone::from_key("ECOFLEX_00_30").expect("the catalog carries it"),
            18,
            25,
        );

        *row.thickness_mm.state.text_mut() = "500".to_string();
        row.thickness_mm.state.on_typed();

        assert!(
            (row.draft().thickness_m - 0.1).abs() < 1e-12,
            "clamped to the 100 mm the stepper offers"
        );
    }

    // ── ridge_options_from_rows ─────────────────────────────────────────────

    #[test]
    fn rows_and_toggles_produce_the_same_options_the_ui_did() {
        let rows = vec![RingRow::from_ridge(&RidgeRing {
            position_frac: 0.5,
            depth_m: 0.002,
            half_width_frac: 0.1,
        })];
        let toggles = RidgeToggles {
            enabled: true,
            rings_enabled: true,
            texture_enabled: true,
            texture_depth_tenths_mm: 3,
            texture_spacing_tenths_mm: 25,
            side_pinch_enabled: true,
            side_pinch_tenths_mm: 4,
            tip_relief_enabled: true,
            tip_relief_tenths_mm: 6,
            orientation_enabled: true,
            orientation_deg: 90,
        };
        let opts = ridge_options_from_rows(&rows, toggles);
        assert!(opts.enabled);
        assert_eq!(opts.rings.len(), 1);
        assert!((opts.texture_depth_m - 0.0003).abs() < 1e-12);
        assert!((opts.orientation_deg - 90.0).abs() < 1e-12);
    }

    #[test]
    fn the_master_toggle_off_zeroes_everything_downstream() {
        let rows = vec![RingRow::from_ridge(&RidgeRing {
            position_frac: 0.5,
            depth_m: 0.002,
            half_width_frac: 0.1,
        })];
        let opts = ridge_options_from_rows(
            &rows,
            RidgeToggles {
                enabled: false,
                rings_enabled: true,
                ..RidgeToggles::default()
            },
        );
        assert!(
            !opts.enabled,
            "the master switch gates the whole feature set"
        );
    }

    #[test]
    fn disabling_rings_drops_the_rows_even_when_present() {
        let rows = vec![RingRow::from_ridge(&RidgeRing {
            position_frac: 0.5,
            depth_m: 0.002,
            half_width_frac: 0.1,
        })];
        let opts = ridge_options_from_rows(
            &rows,
            RidgeToggles {
                enabled: true,
                rings_enabled: false,
                ..RidgeToggles::default()
            },
        );
        assert!(
            opts.rings.is_empty(),
            "an edited-but-disabled ring must not reach the carve"
        );
    }

    // ── PartPicker ──────────────────────────────────────────────────────────

    #[test]
    fn a_rebuilt_picker_starts_all_checked() {
        // ⚠⚠ `rows()` is what the picker RENDERS, and it used to be asserted
        // only as `p.rows().all(|(_, c)| c)` — vacuously true on an EMPTY
        // iterator. Mutation proved it blind: `rows()` returning
        // `iter::empty()`, or one fabricated `("xyzzy", true)`, passed the whole
        // suite, and so did `len()` returning 0 or 1. That left the render half
        // of this type unverified — and a wrong `rows()` shows one set of
        // checkboxes while `selection()` exports another, which is the exact
        // disagreement `PartPicker` exists to make impossible.
        let p = PartPicker::rebuild(2, CastMode::Bonded);

        let labels: Vec<&str> = p.rows().map(|(label, _)| label).collect();
        assert_eq!(
            labels,
            [
                "Layer 1 — cup (left)",
                "Layer 1 — cup (right)",
                "Layer 1 — plug",
                "Layer 2 — cup (left)",
                "Layer 2 — cup (right)",
                "Platform",
                "Dowels",
            ],
            "rows() must render exactly what `enumerate_parts` enumerated, in order"
        );
        assert_eq!(
            p.len(),
            7,
            "bonded: 2 cups x 2 layers + 1 plug + platform + dowels"
        );
        assert_eq!(
            p.rows().count(),
            p.len(),
            "len() must agree with what rows() yields"
        );
        assert!(!p.is_empty());
        assert!(p.any_checked());
        assert!(p.rows().all(|(_, checked)| checked));
    }

    #[test]
    fn an_empty_picker_reports_itself_empty() {
        // The pre-design state: no layers, nothing to pick. `is_empty()` gates
        // the Make-molds button, and a mutant returning `false` unconditionally
        // survived the entire suite because nothing ever asked an EMPTY picker.
        let p = PartPicker::default();
        assert!(p.is_empty());
        assert_eq!(p.len(), 0);
        assert_eq!(p.rows().count(), 0);
        assert!(!p.any_checked());
    }

    #[test]
    fn none_then_any_checked_is_false() {
        let mut p = PartPicker::rebuild(2, CastMode::Bonded);
        p.set_all(false);
        assert!(
            !p.any_checked(),
            "make-molds needs at least one piece; the button gates on this"
        );
    }

    #[test]
    fn checking_one_row_is_enough_to_proceed() {
        let mut p = PartPicker::rebuild(2, CastMode::Bonded);
        p.set_all(false);
        p.set_checked(0, true);
        assert!(p.any_checked());
    }

    #[test]
    fn an_out_of_range_index_is_ignored_rather_than_panicking() {
        let mut p = PartPicker::rebuild(1, CastMode::Bonded);
        p.set_checked(999, false);
        assert!(p.any_checked(), "a stale row index must not corrupt state");
    }

    #[test]
    fn a_rebuild_for_a_new_layer_count_resets_to_all_checked() {
        let mut p = PartPicker::rebuild(1, CastMode::Bonded);
        p.set_all(false);
        let p2 = PartPicker::rebuild(3, CastMode::Bonded);
        assert!(
            p2.any_checked(),
            "changing the design starts the picker fresh"
        );
        assert!(p2.len() >= p.len());
    }
    /// Both halves matter: the weld prompt must win over the loop count, and a
    /// welded-but-closed mesh must NOT be told to weld. Reading only
    /// `looks_unwelded` after a `loop_count == 0` test inverts the first.
    #[test]
    fn unwelded_soup_is_told_to_weld_before_its_loop_count_is_believed() {
        // Soup reports plenty of loops; the count is meaningless.
        assert_eq!(
            FloorReadiness::read(48, true),
            FloorReadiness::Unwelded,
            "many loops on unwelded soup must still ask for a weld"
        );
        assert_eq!(
            FloorReadiness::read(0, false),
            FloorReadiness::NoOpenEdges,
            "a welded, closed mesh has nothing to stand on — and needs no weld"
        );
        assert_eq!(FloorReadiness::read(1, false), FloorReadiness::Ready);
    }

    #[test]
    fn only_a_ready_scan_has_no_blocking_message() {
        assert!(FloorReadiness::Ready.blocked_message().is_none());
        assert!(FloorReadiness::Unwelded.blocked_message().is_some());
        assert!(FloorReadiness::NoOpenEdges.blocked_message().is_some());
    }

    #[test]
    fn the_trim_bound_is_floored_so_the_fields_stay_usable() {
        assert_eq!(trim_bound_mm(0.0), 10, "a traced-nothing centerline");
        assert_eq!(trim_bound_mm(3.4), 10, "and anything under the floor");
        assert_eq!(trim_bound_mm(147.6), 148, "otherwise it rounds");
    }

    /// The cast is `as`, which saturates rather than wrapping. Without the
    /// clamp behind it a NaN arc length would offer a 0 mm bound.
    #[test]
    fn a_non_finite_arc_length_still_yields_a_usable_bound() {
        assert_eq!(trim_bound_mm(f64::NAN), 10);
        assert_eq!(trim_bound_mm(f64::INFINITY), 100_000);
        assert_eq!(trim_bound_mm(f64::NEG_INFINITY), 10);
    }

    /// Tilt is shown to the nearest whole degree, and the two counts must not
    /// swap: they read as "N open loop(s), M-segment centerline".
    #[test]
    fn the_found_floor_line_rounds_the_tilt_and_keeps_its_counts_in_order() {
        let line = format_floor_found(2, 41, 3.7);
        assert!(
            line.contains("2 open loop(s), 41-segment centerline"),
            "counts in the wrong order or the wrong units: {line}"
        );
        assert!(
            line.contains("corrected 4° tilt"),
            "tilt must round to whole degrees: {line}"
        );
    }

    /// The other half of Find floor's report. Its whole job is to say how many
    /// loops *were* found — a message that dropped the count would leave the
    /// user with nothing to act on and no idea the scan was even read.
    #[test]
    fn the_untraceable_centerline_line_still_reports_what_was_found() {
        let line = format_floor_no_centerline(3);
        assert!(
            line.contains("3 loop(s)"),
            "the count is the actionable part: {line}"
        );
    }

    #[test]
    fn the_stats_line_reads_faces_then_vertices() {
        assert_eq!(
            format_scan_stats(193_740, 581_220),
            "193740 faces · 581220 vertices"
        );
    }

    /// ⚠ Distinct operands, deliberately. The target and the elapsed seconds
    /// are both numbers in one sentence, so a swapped pair would go on reading
    /// as a perfectly plausible message.
    #[test]
    fn the_simplify_report_keeps_its_target_and_its_cost_apart() {
        let line = format_simplify_done(200_000, 12.34);
        assert!(
            line.contains("200000 faces"),
            "the target is what the user asked for: {line}"
        );
        assert!(
            line.contains("(12.3s)"),
            "the cost is shown to a tenth of a second: {line}"
        );
    }

    /// ⚠ Both names and the count, not just that it is non-empty. This line is
    /// the only place the user is told which two files to hand on, and that the
    /// step is finished.
    #[test]
    fn a_finished_save_names_both_files_and_points_at_the_next_step() {
        let done = format_save_done("base_mold", 180_236);

        assert!(done.contains("base_mold.cleaned.stl"), "{done}");
        assert!(done.contains("base_mold.prep.toml"), "{done}");
        assert!(done.contains("180236"), "{done}");
        assert!(done.contains("Next"), "{done}");
    }

    /// The estimate is why this message exists at all: without it, forty
    /// seconds of a still window reads as a hung app.
    #[test]
    fn the_simplify_start_line_carries_the_target_and_the_estimate() {
        let line = format_simplify_started(50_000);
        assert!(line.contains("50000 faces"), "the target: {line}");
        assert!(line.contains("~10–40 s"), "the estimate: {line}");
    }

    /// The step-3 question as the screen's opening fields ask it: a 5 mm inset,
    /// no ridges, at step 5's default quality.
    fn opening_question() -> FitQuestion {
        FitQuestion {
            plug: PlugDraft {
                cavity_inset_m: 0.005,
                ridges: RidgeOptions::default(),
            },
            cell_size_m: 0.0005,
            scan: Some((1_234, std::time::UNIX_EPOCH)),
        }
    }

    /// The same question with the ridge master switch on.
    fn ridged(question: &FitQuestion) -> FitQuestion {
        FitQuestion {
            plug: PlugDraft {
                ridges: RidgeOptions {
                    enabled: true,
                    ..RidgeOptions::default()
                },
                ..question.plug.clone()
            },
            ..question.clone()
        }
    }

    /// ★★★ The staleness rule, on every axis the verdict turns on. One
    /// comparison stands in for the inset, the master switch, every ring row
    /// and five scalars — so the gate has to show it actually separates them.
    /// A `fit_view` that compared insets alone passes a case that moves only
    /// the inset, and would then leave a stale verdict up for every ridge edit
    /// on the screen.
    ///
    /// ⚠ Two-sided. Without the first assertion, a `fit_view` that never shows
    /// an answer at all passes the whole loop below.
    #[test]
    fn an_answer_is_shown_only_while_it_describes_the_question_on_screen() {
        let asked = opening_question();
        let answered = (asked.clone(), Ok(PlugFit::Casts));

        assert_eq!(
            fit_view(None, Some(&answered), &asked),
            FitView::Answered(&PlugFit::Casts),
            "the question it was asked about keeps its answer"
        );

        let moved: [(&str, FitQuestion); 5] = [
            (
                "the cavity inset",
                FitQuestion {
                    plug: PlugDraft {
                        cavity_inset_m: 0.006,
                        ..asked.plug.clone()
                    },
                    ..asked.clone()
                },
            ),
            ("the ridge master switch", ridged(&asked)),
            (
                // Three levels down, and the switch above it never moves —
                // the shape of edit a per-field hook is most likely to miss.
                "one ridge scalar",
                FitQuestion {
                    plug: PlugDraft {
                        ridges: RidgeOptions {
                            texture_depth_m: 0.0021,
                            ..RidgeOptions::default()
                        },
                        ..asked.plug.clone()
                    },
                    ..asked.clone()
                },
            ),
            (
                "the cell size",
                FitQuestion {
                    cell_size_m: 0.0015,
                    ..asked.clone()
                },
            ),
            (
                // ⚠ The axis no field on step 3 can carry. Step 2 is reachable
                // from here, and a second Save rewrites the cleaned scan in
                // place: every field reads the same, and the body does not.
                "the cleaned scan under it",
                FitQuestion {
                    scan: Some((5_678, std::time::UNIX_EPOCH)),
                    ..asked.clone()
                },
            ),
        ];

        for (what, current) in moved {
            assert_eq!(
                fit_view(None, Some(&answered), &current),
                FitView::Idle,
                "moving {what} must drop an answer that no longer describes the screen"
            );
        }
    }

    /// ⚠ A check that could not run is its own view, never a refusal: the
    /// operator's inset may be perfectly fine and nobody found out. It goes
    /// stale with the question like any other answer, so a failure about
    /// settings they have left does not sit on the screen either.
    #[test]
    fn a_check_that_could_not_run_reads_as_a_failure_not_as_a_refusal() {
        const BROKEN: &str = "scan.cleaned.stl: no such file";
        let asked = opening_question();
        let outcome: (FitQuestion, Result<PlugFit, String>) =
            (asked.clone(), Err(BROKEN.to_string()));

        assert_eq!(
            fit_view(None, Some(&outcome), &asked),
            FitView::Failed(BROKEN),
            "no verdict was reached, and the line has to say which"
        );
        assert_eq!(
            fit_view(None, Some(&outcome), &ridged(&asked)),
            FitView::Idle,
            "and it goes stale with the question, like any other answer"
        );
    }

    /// ⚠ A running check outranks the answer it is about to replace. With the
    /// arms the other way round, a verdict sits on screen reading as settled
    /// while the screen is busy deciding whether it still is.
    #[test]
    fn a_running_check_replaces_the_answer_it_is_about_to_supersede() {
        let asked = opening_question();
        let answered = (asked.clone(), Ok(PlugFit::Casts));

        assert_eq!(
            fit_view(Some((&asked, 12)), Some(&answered), &asked),
            FitView::Checking {
                elapsed_secs: 12,
                slow: true,
                inset_mm: 5.0,
            },
            "the clock, not the answer under it"
        );
    }

    /// ★★ Every field of the running view is read off the question the run was
    /// STARTED with, not off the screen. Editing step 3 mid-run changes neither
    /// what is running nor what it will cost.
    ///
    /// ⚠ The inset especially. Nothing stops the fields moving while a check
    /// runs, the answer is dropped on arrival when they have, and a line naming
    /// the SCREEN's inset would spend those minutes claiming to check a value
    /// nobody is checking — and then show nothing.
    #[test]
    fn the_running_line_describes_the_check_that_is_actually_running() {
        // Started at the opening 5 mm, smooth, Fast quality.
        let in_flight = FitQuestion {
            cell_size_m: 0.0015,
            ..opening_question()
        };
        // The screen has since moved to 11 mm.
        let on_screen = FitQuestion {
            plug: PlugDraft {
                cavity_inset_m: 0.011,
                ..in_flight.plug.clone()
            },
            ..in_flight.clone()
        };

        assert_eq!(
            fit_view(Some((&in_flight, 30)), None, &ridged(&on_screen)),
            FitView::Checking {
                elapsed_secs: 30,
                slow: false,
                inset_mm: 5.0,
            },
            "the run in flight is a quick 5 mm one, whatever the screen now reads"
        );
        assert_eq!(
            fit_view(Some((&ridged(&in_flight), 30)), None, &on_screen),
            FitView::Checking {
                elapsed_secs: 30,
                slow: true,
                inset_mm: 5.0,
            },
            "and a ridged run stays ridged while the screen reads smooth"
        );
    }

    /// ★★ Both drivers, and the gate needs both: ridges are slow at every
    /// quality, and the print-quality cell size is slow even with none. A
    /// predicate reading only `ridges.enabled` calls step 3's DEFAULT question
    /// quick, and an 80-second wait then arrives with no warning at all.
    #[test]
    fn a_check_is_quick_only_when_it_is_a_fast_quality_smooth_one() {
        let fine_smooth = opening_question();
        let fast_smooth = FitQuestion {
            cell_size_m: 0.0015,
            ..fine_smooth.clone()
        };

        assert!(
            !fit_check_is_slow(&fast_smooth),
            "1.5 mm with no ridges is the one quick case: 6.7 s measured"
        );
        assert!(
            fit_check_is_slow(&fine_smooth),
            "the quality step 5 opens on is 80 s even with no ridges"
        );
        assert!(
            fit_check_is_slow(&ridged(&fast_smooth)),
            "ridges pin the plug at 0.5 mm whatever cell size is passed"
        );
    }

    /// ★★★ What starts a check nobody clicked for, on all three axes at once.
    /// Drop any one and the predicate still passes a happy-path gate: without
    /// the settle it fires on every keystroke of a two-digit inset, without
    /// `askable` it runs against a scan that is not there.
    ///
    /// ⚠ The settle is asserted AT the boundary in both directions. A `>` where
    /// `>=` belongs is a mutant no comfortable margin can kill.
    #[test]
    fn a_question_that_has_stopped_moving_is_checked_without_being_clicked() {
        let just_short = FIT_SETTLE - Duration::from_millis(1);

        assert!(
            fit_check_is_due(&FitView::Idle, FIT_SETTLE, true),
            "settled exactly on the boundary is settled"
        );
        assert!(
            !fit_check_is_due(&FitView::Idle, just_short, true),
            "a question still moving is not one to spend eighty seconds on"
        );
        assert!(
            !fit_check_is_due(&FitView::Idle, FIT_SETTLE, false),
            "and nothing starts itself with the app held or no scan to run on"
        );
    }

    /// ★★ Every state but `Idle`, and `Failed` is the one that matters. It is a
    /// check that could not RUN, so a predicate that fires on it starts the
    /// same doomed check every 1.5 s for as long as the screen is open — and a
    /// gate covering only `Checking` and `Answered` would not notice.
    #[test]
    fn nothing_starts_a_second_check_over_one_running_answered_or_failed() {
        let long_settled = FIT_SETTLE * 4;
        let views = [
            FitView::Checking {
                elapsed_secs: 12,
                slow: true,
                inset_mm: 9.0,
            },
            FitView::Answered(&PlugFit::Casts),
            FitView::Failed("the cleaned scan could not be read"),
        ];

        for view in &views {
            assert!(
                !fit_check_is_due(view, long_settled, true),
                "{view:?} already answers, or is answering, the settings on screen"
            );
        }
    }

    /// ⚠ Two-sided, and the refusal verbatim: the cast's words already name
    /// the operator's levers, and a paraphrase would be a second explanation
    /// of the same failure, free to drift from the one the cast gives.
    #[test]
    fn the_verdict_reads_as_settled_or_refused_and_names_the_inset() {
        let plug = PlugDraft {
            cavity_inset_m: 0.011,
            ..PlugDraft::default()
        };

        let casts = format_fit_verdict(&PlugFit::Casts, &plug);
        assert!(
            matches!(&casts, Ok(text) if text.contains("11.0 mm")),
            "a settled fit reads as one, at the inset asked: {casts:?}"
        );
        // ★★ And claims ONLY what was checked. `plug_fit_verdict` runs the
        // compose half; the export still F4-gates the mesh and can refuse
        // after it, so a settled line saying the inset "casts" promises an
        // outcome nobody established. The two halves guard each other: drop
        // the first and any wording passes, drop the second and the promise
        // comes back.
        assert!(
            matches!(&casts, Ok(text) if text.contains("one piece") && !text.contains("cast")),
            "the settled line claims one-piece-ness, never that the cast succeeds: {casts:?}"
        );

        let reason = "the floor lock did not fuse to the plug";
        let refused = format_fit_verdict(
            &PlugFit::WillNotCast {
                reason: reason.to_string(),
            },
            &plug,
        );
        assert!(
            matches!(&refused, Err(text) if text.contains("11.0 mm") && text.contains(reason)),
            "a refusal reads as one, carrying the cast's own words: {refused:?}"
        );
    }

    /// ⚠⚠ A refusal and a broken check both land in red carrying the cast's
    /// own words, and only the wording keeps them apart. Every other gate here
    /// looks for the REASON — which both carry — so a failure that borrowed the
    /// refusal's phrasing passes all of them while telling the operator their
    /// inset is too large when the real problem is a file that would not open.
    ///
    /// ⚠ The last assertion is what stops the middle one going vacuous: if the
    /// refusal ever stopped claiming "will not cast", not finding that phrase
    /// in the failure would prove nothing.
    #[test]
    fn a_broken_check_does_not_read_as_a_verdict_on_the_inset() {
        const REASON: &str = "scan.cleaned.stl: no such file";
        let refused = format_fit_verdict(
            &PlugFit::WillNotCast {
                reason: REASON.to_string(),
            },
            &PlugDraft {
                cavity_inset_m: 0.011,
                ..PlugDraft::default()
            },
        );

        let broken = format_fit_failure(REASON);

        assert!(
            broken.contains(REASON),
            "the words it came back with are carried: {broken}"
        );
        assert!(
            !broken.contains("will not cast"),
            "a check that never ran must not borrow the refusal's verdict: {broken}"
        );
        assert!(
            matches!(&refused, Err(text) if text.contains("will not cast")),
            "which is the phrase the refusal owns: {refused:?}"
        );
    }

    /// ⚠ The two costs are two orders of magnitude apart — seconds smooth,
    /// minutes with ridges on — so the slow one has to say so or it reads as a
    /// hang. Both still carry the clock.
    #[test]
    fn the_progress_line_warns_only_when_the_check_is_the_slow_one() {
        let quick = format_fit_progress(3, false, 5.0);
        let slow = format_fit_progress(3, true, 5.0);

        assert!(
            !quick.contains("minutes"),
            "a seconds-long check must not promise minutes: {quick}"
        );
        assert!(
            slow.contains("minutes"),
            "a minutes-long one must say so: {slow}"
        );
        for line in [&quick, &slow] {
            assert!(line.contains("3s"), "both carry the clock: {line}");
            assert!(
                line.contains("5.0 mm"),
                "and both name the inset in flight: {line}"
            );
        }
    }
}
