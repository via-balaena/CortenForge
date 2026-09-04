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
//! - [`nav_state`] computes the gated Back/Next availability.
//!
//! The Bevy app, the egui panels and the file-dialog glue live in the
//! binary's modules (they need a display to *run*, but compile headlessly).

use std::fmt::Write as _;
use std::path::Path;
use std::time::Duration;

use cf_studio_core::{
    DesignDraft, MoldOutputs, PourPlan, PourStep, Project, RidgeOptions, RidgeRing, Step,
};
use cf_studio_engine::{
    CastMode, PartId, PartSelection, PieceSide, accept_prep, draft_from_design_toml, load_scan,
};

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

/// Step 2 action — accept a cleaned scan + its `.prep.toml`.
///
/// # Errors
/// The failure message if the prep is invalid or the scan step isn't done.
pub fn apply_prep(project: &mut Project, cleaned_stl: &Path, prep_toml: &Path) -> StepOutcome {
    let prep = accept_prep(cleaned_stl, prep_toml).map_err(|e| e.to_string())?;
    project.set_prep(prep).map_err(|e| e.to_string())?;
    Ok("✔ Accepted cleaned scan + prep.".to_string())
}

/// Step 3 action — load a layer design from a `.design.toml`.
///
/// # Errors
/// The failure message if the design is invalid or the scan isn't cleaned.
pub fn apply_design(project: &mut Project, design_toml: &Path) -> StepOutcome {
    let draft = draft_from_design_toml(design_toml).map_err(|e| e.to_string())?;
    let message = format!(
        "✔ Design set: {} layer(s), {:.1} mm cavity inset.",
        draft.layers.len(),
        draft.cavity_inset_m * 1000.0
    );
    project.set_design(draft).map_err(|e| e.to_string())?;
    Ok(message)
}

/// Step 3 action — set a layer design built in-app (the layer-stack
/// editor), rather than loaded from a file.
///
/// # Errors
/// The failure message if the design is invalid or the scan isn't cleaned.
pub fn apply_design_draft(project: &mut Project, draft: DesignDraft) -> StepOutcome {
    let message = format!(
        "✔ Design set: {} layer(s), {:.1} mm cavity inset.",
        draft.layers.len(),
        draft.cavity_inset_m * 1000.0
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

/// Marching-cubes cell size (meters) for the step-4 quality-picker index.
/// Index 0 = Fine 0.5 mm (the print-quality default — the physical fit-test
/// print was 0.5 mm); index 1 = Fast 1.5 mm preview. Any other index falls
/// back to the print-quality default. 3 mm is never offered (it drops the
/// flange web). **Must stay in lockstep with the quality picker's option order
/// in the step-4 panel.** Nothing checks that pairing; only the index→size
/// mapping is pinned, by `quality_index_maps_to_cell_size`.
#[must_use]
pub fn cell_size_m_for_quality(quality_idx: i32) -> f64 {
    match quality_idx {
        1 => 0.0015,
        _ => 0.0005,
    }
}

/// Enumerate the generatable parts for a design with `layer_count` layers,
/// in display order, as `(id, label)`. Per layer: two cup halves + a plug,
/// then the shared workshop platform + dowels (the apex pour funnel is
/// integral, and the gasket is off, so neither is offered).
///
/// In [`CastMode::Bonded`] only the **layer-0** plug is offered — the
/// per-layer plugs above 0 are redundant (the cured layer N is the plug for
/// layer N+1), so they are not listed (or generated). The step-4 part picker
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

/// A human-readable summary of a completed mold run for the step-4 results
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

/// The step-5 (Print) status line, derived from project state: once the
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
        return format!("Ready to save {pieces} printable file(s) + the step-by-step guide.");
    }
    String::new()
}

// ── step 6: the pour assistant ──────────────────────────────────────

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
/// the step-6 reference panel. Empty plan → a short placeholder.
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

/// The active-layer instruction for the step-6 pour panel: which layer of
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

/// A pot-life countdown's display text + urgency for the step-6 timer.
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
/// jobs it labels run 4.5–15 minutes, so the minutes field is the part that
/// matters and the one an off-by-one would hide.
#[must_use]
pub fn format_elapsed(secs: u64) -> String {
    format!("{}:{:02}", secs / 60, secs % 60)
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

/// One grip ring in the "Shape your piece" editor, in the integer units the UI
/// edits (percent, tenths of a millimetre) rather than the SDK's meters.
///
/// The three `StepBoxState`s live **inside the row** on purpose — see the
/// warning on [`StepBoxState`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RingRow {
    /// Axial position: 0 = opening … 100 = deep end.
    pub position: StepBoxState,
    /// Inward pinch depth, in tenths of a millimetre.
    pub depth: StepBoxState,
    /// Half-width of the ring's axial support, percent.
    pub width: StepBoxState,
}

impl RingRow {
    /// Build a row from an owned SDK ring (meters/fractions → integer UI units).
    #[must_use]
    pub fn from_ridge(ring: &RidgeRing) -> Self {
        Self {
            position: StepBoxState::new(scale_to_i32(ring.position_frac, 100.0)),
            depth: StepBoxState::new(scale_to_i32(ring.depth_m, 10_000.0)),
            width: StepBoxState::new(scale_to_i32(ring.half_width_frac, 100.0)),
        }
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
        "✓ Saved {stem}.cleaned.stl ({face_count} faces) + {stem}.prep.toml — \
         step complete, click Next →."
    )
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use super::*;

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
        apply_plug(&mut p, cf_studio_core::PlugDraft::default()).unwrap();
        let msg = apply_design(&mut p, &design).unwrap();
        assert!(msg.contains("Design set"), "got: {msg}");
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
        apply_plug(&mut p, cf_studio_core::PlugDraft::default()).unwrap();

        // A design built in-app (the layer-stack editor's output).
        let draft = DesignDraft {
            cavity_inset_m: 0.005,
            layers: vec![LayerDraft {
                thickness_m: 0.0175,
                material_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.25,
            }],
        };
        let msg = apply_design_draft(&mut p, draft).unwrap();
        assert!(msg.contains("Design set"), "got: {msg}");
        assert!(p.is_complete(Step::DesignLayers));

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
        apply_plug(&mut p, cf_studio_core::PlugDraft::default()).unwrap();
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
        assert!(s.contains("Ready to save 4 printable"), "got: {s}");

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
        apply_plug(&mut p, cf_studio_core::PlugDraft::default()).unwrap();
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
        // The jobs this labels run 4.5-15 minutes; the minutes field is the part
        // an off-by-one would hide.
        assert_eq!(format_elapsed(15 * 60 + 7), "15:07");
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

    /// The tick must be U+2714 `✔`, not U+2713 `✓`. No bundled font carries
    /// U+2713 — it renders as a tofu box — and a test that only checks the
    /// words cannot see that.
    #[test]
    fn the_simplify_report_uses_a_tick_the_bundled_fonts_have() {
        let line = format_simplify_done(1_000, 0.0);
        assert_eq!(
            line.chars().next().map(u32::from),
            Some(0x2714),
            "wrong tick: {line}"
        );
    }

    /// The estimate is why this message exists at all: without it, forty
    /// seconds of a still window reads as a hung app.
    #[test]
    fn the_simplify_start_line_carries_the_target_and_the_estimate() {
        let line = format_simplify_started(50_000);
        assert!(line.contains("50000 faces"), "the target: {line}");
        assert!(line.contains("~10–40 s"), "the estimate: {line}");
    }
}
