//! Measurement preconditions: **which box**, and **is it idle**.
//!
//! A frame budget is ABSOLUTE, so a gap to it belongs to one machine. The
//! evidence, the pilot data and the thresholds' derivation live in
//! `docs/SIM_SOFT_REALTIME_RECON.md` §2h; this module is the enforcement.
//!
//! **Hard-failed** — `hw.model` and the P/E core split, the hardware the cost
//! scales with. **Stamped only** — RAM, OS, `rustc`: they drift for reasons
//! unrelated to speed, and the toolchain is pinned by `rust-toolchain.toml`.
//!
//! Two invariants a future edit must not break:
//!
//! 1. ⚠⚠ **This file is copied onto the PRE-R0 tree** by `tests/r0_ab.rs`'s A/B
//!    recipe, so it must compile against `ecf4cfef^`. Anything newer than that
//!    commit must be `cfg`-gated — `sim_soft::profile` already broke that arm
//!    once.
//! 2. ⚠ Deliberately NOT `tests/common/mod.rs`, which eight physics tests share
//!    and which has no business depending on `sysctl`.
//!
//! ⚠ **Transfer between boxes is UNVALIDATED and not offered here** — it needs
//! two boxes, and §2d finding 4 has it failing in opposite directions.

// Each integration test binary compiles this whole module but uses only part of
// it, so per-binary dead-code warnings are expected and not informative.
#![allow(dead_code)]

use std::process::Command;
use std::time::Instant;

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, MaterialField, Mesh,
    NullContact, Solver, SolverConfig, Tet4, Vec3, pick_vertices_by_predicate,
};

/// `hw.model` of the reference box, and the strongest single identifier of it.
pub const REF_MODEL: &str = "Mac16,8";
/// Performance cores on the reference box.
pub const REF_P_CORES: usize = 8;
/// Efficiency cores on the reference box.
pub const REF_E_CORES: usize = 4;

/// What the box says it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoxIdentity {
    /// `hw.model`, e.g. `Mac16,8`.
    pub model: String,
    /// `machdep.cpu.brand_string`, e.g. `Apple M4 Pro`.
    pub cpu: String,
    /// Performance-core count.
    pub p_cores: usize,
    /// Efficiency-core count.
    pub e_cores: usize,
    /// Physical memory, `GiB`.
    pub ram_gib: usize,
    /// OS product version.
    pub os: String,
    /// `rustc --version`.
    pub rustc: String,
}

fn run(cmd: &str, args: &[&str]) -> String {
    Command::new(cmd)
        .args(args)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map_or_else(
            || "unknown".to_string(),
            |o| String::from_utf8_lossy(&o.stdout).trim().to_string(),
        )
}

fn sysctl(key: &str) -> String {
    run("sysctl", &["-n", key])
}

impl BoxIdentity {
    /// Read the box's identity. Every field degrades to `unknown` rather than
    /// panicking, so a non-macOS host still produces a printable stamp — and
    /// still fails [`require_reference_box`], which is the correct outcome there
    /// rather than a crash.
    #[must_use]
    pub fn detect() -> Self {
        Self {
            model: sysctl("hw.model"),
            cpu: sysctl("machdep.cpu.brand_string"),
            p_cores: sysctl("hw.perflevel0.logicalcpu").parse().unwrap_or(0),
            e_cores: sysctl("hw.perflevel1.logicalcpu").parse().unwrap_or(0),
            ram_gib: sysctl("hw.memsize").parse::<u64>().map_or(0, |b| {
                usize::try_from(b / (1024 * 1024 * 1024)).unwrap_or(0)
            }),
            os: run("sw_vers", &["-productVersion"]),
            rustc: run("rustc", &["--version"]),
        }
    }

    /// Whether this is the reference box, on the two properties the cost scales
    /// with. RAM, OS and toolchain are deliberately excluded — see module docs.
    #[must_use]
    pub fn is_reference(&self) -> bool {
        self.model == REF_MODEL && self.p_cores == REF_P_CORES && self.e_cores == REF_E_CORES
    }

    /// Print the identity block that accompanies every measurement.
    pub fn stamp(&self) {
        println!("┌─ box ─────────────────────────────────────────────────────");
        println!("│ model   {}  ({})", self.model, self.cpu);
        println!(
            "│ cores   {} P + {} E     ram {} GiB",
            self.p_cores, self.e_cores, self.ram_gib
        );
        println!("│ os      {}     rustc {}", self.os, self.rustc);
        println!(
            "│ {}",
            if self.is_reference() {
                "★ REFERENCE BOX — figures from this run are gate-bearing"
            } else {
                "⚠ NOT the reference box — figures are NOT comparable to the recon"
            }
        );
        println!("└───────────────────────────────────────────────────────────");
    }
}

/// Stamp the box, and **fail** unless it is the reference box.
///
/// Call this from any measurement whose number is spent against a gate: the
/// recon's `10×` kill floor and `13.5–15.8×` frame-budget requirement are both
/// quantities on THIS machine.
pub fn require_reference_box() -> BoxIdentity {
    let id = BoxIdentity::detect();
    id.stamp();
    assert!(
        id.is_reference(),
        "NOT the reference box: model {} ({} P + {} E), expected {REF_MODEL} \
         ({REF_P_CORES} P + {REF_E_CORES} E). This measurement is gate-bearing, and \
         the recon's cross-box data (1.60x / 1.56x / 1.08x / 0.51x across four \
         fixture-arm pairs) shows there is NO factor that would convert it. Re-run on \
         the reference box, or re-baseline the recon deliberately.",
        id.model,
        id.p_cores,
        id.e_cores
    );
    id
}

// ── contention probe ────────────────────────────────────────────────────────

/// Steps the probe times (after discarding [`PROBE_WARMUP`]).
pub const PROBE_STEPS: usize = 20;
/// Steps discarded before timing. Covers first-touch page faults AND the
/// settling of the Newton count: at 3 the probe still caught a `[2, 3, 3, …]`
/// first step and correctly declared itself invalid.
pub const PROBE_WARMUP: usize = 5;
/// `dt` for the probe. §2 records that `1e-3` pins the step to a constant, tiny
/// Newton iteration count, which is the property the probe depends on.
pub const PROBE_DT: f64 = 1.0e-3;

/// What the probe saw.
#[derive(Debug, Clone, Copy)]
pub struct Probe {
    /// Median per-step milliseconds.
    pub p50_ms: f64,
    /// Fastest step.
    pub min_ms: f64,
    /// Slowest step.
    pub max_ms: f64,
    /// Newton iterations per step — constant by construction, and checked.
    pub iters: usize,
}

impl Probe {
    /// Slowest step over the median — the **gross-stall** statistic, and the
    /// WEAKER of the two checks.
    ///
    /// ⚠ It does NOT detect sustained partial load: at 4 busy cores it reads
    /// `1.063×` against a `1.060×` idle maximum, because steady load slows every
    /// step and lands in the median instead. It catches the opposite failure —
    /// a few long stalls (12 busy cores: `2.37–5.87×`). §2h has the pilot data.
    #[must_use]
    pub fn burst(&self) -> f64 {
        self.max_ms / self.p50_ms
    }

    /// One line, for the stamp.
    pub fn report(&self) {
        println!(
            "│ probe   p50 {:.2} ms  min {:.2}  max {:.2}  burst {:.2}x  ({} iters/step)",
            self.p50_ms,
            self.min_ms,
            self.max_ms,
            self.burst(),
            self.iters
        );
    }
}

/// Run the contention probe: `cantilever 40×4` (3 000 free DOF) at [`PROBE_DT`].
///
/// **Measured ~61 % `numeric factor`** (`refbox_pilot::probe_phase_character`;
/// 61.1 / 60.8 % on two runs),
/// so it loads the phase the real measurements are bottlenecked on. §2d's
/// `57.5 %` for this DOF count is NOT the evidence — that row is `dt = 1/60`
/// with many Newton iterations, a different regime.
///
/// A separate probe exists because the measurement fixtures cannot police
/// themselves — the IPC ramp's `max/p50` is `1.31` from physics alone. This does
/// the SAME work every step, so its spread has one source.
#[must_use]
pub fn probe() -> Probe {
    let p = probe_inner_for_profiling();
    // The probe runs real solves, so with `phase-timing` on it has been
    // accumulating into the GLOBAL phase counters. Leave them as we found them:
    // no current caller is harmed (each resets before reading), but a future
    // measurement that snapshots without resetting would report the probe's work
    // as its own.
    //
    // ⚠ `cfg`, not an unconditional call, and that is REQUIRED — see the
    // cross-tree note in the module docs. `sim_soft::profile` landed with the
    // phase-timing feature and does not exist on the pre-R0 tree this file is
    // copied into; naming it unconditionally fails that arm to compile, which is
    // exactly what the first version of this hygiene fix did.
    #[cfg(feature = "phase-timing")]
    sim_soft::profile::reset();
    p
}

/// [`probe`] without the counter reset, so `refbox_pilot`'s phase-mix pilot can
/// read what the probe actually stressed. Not for measurement use.
#[must_use]
pub fn probe_inner_for_profiling() -> Probe {
    const MU: f64 = 3.5e5;
    const LAMBDA: f64 = 1.4e6;
    let field = MaterialField::uniform(MU, LAMBDA);
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(40, 4, 4, 0.20, 0.02, 0.02, &field);
    let n_dof = 3 * mesh.n_vertices();
    let mut x = vec![0.0; n_dof];
    for (c, p) in x.chunks_exact_mut(3).zip(mesh.positions()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    let pins = pick_vertices_by_predicate(&mesh, |p: &Vec3| p.x.abs() < 1e-9);
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = PROBE_DT;
    cfg.density = 1030.0;
    cfg.gravity_z = -9.81;
    cfg.max_newton_iter = 200;
    let solver: CpuTet4NHSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
        Tet4,
        mesh,
        NullContact,
        cfg,
        BoundaryConditions::new(pins, Vec::new()),
    );

    let theta = Tensor::from_slice(&[], &[0]);
    let mut v = vec![0.0; n_dof];
    let mut per_step = Vec::new();
    let mut counts = Vec::new();
    for k in 0..(PROBE_STEPS + PROBE_WARMUP) {
        let t0 = Instant::now();
        let step = solver.replay_step(
            &Tensor::from_slice(&x, &[n_dof]),
            &Tensor::from_slice(&v, &[n_dof]),
            &theta,
            PROBE_DT,
        );
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        if k >= PROBE_WARMUP {
            per_step.push(ms);
            counts.push(step.iter_count);
        }
        for i in 0..n_dof {
            v[i] = (step.x_final[i] - x[i]) / PROBE_DT;
        }
        x = step.x_final;
    }

    // The probe's whole validity rests on doing identical work every step. If
    // the iteration count moves, its spread is physics rather than contention,
    // and the gate would be measuring the wrong thing while looking healthy.
    let iters = counts[0];
    assert!(
        counts.iter().all(|&c| c == iters),
        "contention probe is INVALID: Newton iterations varied across steps \
         ({counts:?}). Its spread statistic assumes constant work per step, so a \
         varying count means it is no longer measuring contention. Re-tune PROBE_DT \
         until the count is constant before trusting any run."
    );

    let mut sorted = per_step.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("no NaN timings"));
    Probe {
        p50_ms: sorted[sorted.len() / 2],
        min_ms: sorted[0],
        max_ms: sorted[sorted.len() - 1],
        iters,
    }
}

// ── the contention gate ─────────────────────────────────────────────────────

/// Probe median on a quiet reference box, from the pilot (10 runs: 23.80–24.70,
/// median 24.47).
pub const REF_PROBE_P50_MS: f64 = 24.47;

/// Upper bound on the probe median before a run is refused.
///
/// **Piloted, not chosen** — it sits in the measured gap between the idle
/// maximum `24.70 ms` and the lightest contaminated case `25.83 ms` (§2h).
///
/// ⚠ Only `3.3 %` headroom, deliberately. A genuine baseline shift will read as
/// contention; the remedy is to re-run `tests/refbox_pilot.rs` and re-baseline
/// both constants, NOT to widen this one.
pub const PROBE_P50_CEILING_MS: f64 = 25.5;

/// Lower bound on the probe median.
///
/// Not a contention check — a floor catches the probe getting CHEAPER, which
/// means the workload or the solver changed underneath the baseline and every
/// constant here is stale. Failing closed forces a re-pilot instead of silently
/// comparing against a number that no longer describes the same work.
pub const PROBE_P50_FLOOR_MS: f64 = 21.0;

/// Upper bound on [`Probe::burst`]. Idle maximum is `1.060×` and a 12-core load
/// reads `2.37×` and up; this sits between, catching gross stalls without
/// firing on the ordinary jitter of a quiet box.
pub const PROBE_BURST_MAX: f64 = 1.30;

// ⚠ MEASURED SENSITIVITY LIMIT. The gate catches sustained EXTERNAL load (2 busy
// cores trip it) but did NOT catch two of this crate's own measurements run
// concurrently — `--test reduced_predictor -- --ignored` without
// `--test-threads=1` passed both gates. Why is not established. So
// `--test-threads=1` stays a REQUIREMENT of the harnesses and this gate is not a
// substitute for it.

/// Identity plus the **scale-free** contention check only, for measurements
/// that run the SAME fixture on more than one tree.
///
/// ⚠ **[`require_quiet_box`] cannot be used across trees.** The probe is a
/// fixture R0 speeds up: pre-R0 it reads **57.39 ms** against post-R0's
/// **24.47 ms**, so the median ceiling would reject the pre-R0 arm every run. A
/// baseline calibrated on one tree is not a property of the box.
/// [`Probe::burst`] IS scale-free (pre-R0 `1.013–1.063×` vs `1.008–1.060×`), so
/// it is checked here.
///
/// ⚠ **The cost is real: this arm has weaker contention protection**, because
/// burst does not detect sustained partial load (see [`Probe::burst`]). What
/// covers it is INTERLEAVING, and that was measured rather than assumed: under a
/// 3-core load the absolute times move `+3.4 %` while the R0 credit shifts
/// `+0.4 %` at 18 750 (`+4.0 %` / `+1.4 %` at 5 202). The load lands on both arms
/// and leaves the ratio. That is why interleaving is mandatory in
/// `tests/r0_ab.rs` rather than advisory.
pub fn require_quiet_box_cross_tree() -> (BoxIdentity, Probe) {
    let id = require_reference_box();
    let p = probe();
    p.report();
    println!("│ (cross-tree: median ceiling NOT applied — the probe is itself subject to R0)");
    assert!(
        p.burst() <= PROBE_BURST_MAX,
        "BOX IS STALLING: probe burst {:.3}x exceeds {PROBE_BURST_MAX:.2}x. Burst is          the only contention check available across trees, so a run that trips it          cannot be rescued by interleaving.",
        p.burst()
    );
    (id, p)
}

/// The full precondition for a gate-bearing measurement on the CURRENT tree:
/// **the reference box, and quiet.**
///
/// Runs the probe (~0.6 s) and refuses the measurement if the box is loaded.
/// Both checks are needed and they catch different things — see
/// [`Probe::burst`] for why the obvious single statistic is the wrong one.
///
/// ⚠ Not for cross-tree measurement — use [`require_quiet_box_cross_tree`].
pub fn require_quiet_box() -> (BoxIdentity, Probe) {
    let id = require_reference_box();
    let p = probe();
    p.report();
    assert!(
        p.p50_ms <= PROBE_P50_CEILING_MS,
        "BOX IS CONTENDED: probe median {:.2} ms exceeds {PROBE_P50_CEILING_MS:.2} ms \
         (quiet reference is {REF_PROBE_P50_MS:.2} ms). Every step is slow, which is \
         sustained background load, not jitter — the recon has a 5.7x spread on one \
         fixture from exactly this. Close what is running (and check you passed \
         --test-threads=1) and re-measure. If the box is genuinely idle, its baseline \
         has moved: re-run tests/refbox_pilot.rs and re-baseline, do not widen the \
         threshold.",
        p.p50_ms
    );
    assert!(
        p.p50_ms >= PROBE_P50_FLOOR_MS,
        "PROBE IS FASTER THAN ITS BASELINE: median {:.2} ms is below \
         {PROBE_P50_FLOOR_MS:.2} ms. The probe workload or the solver changed, so \
         every constant in this module is stale. Re-run tests/refbox_pilot.rs and \
         re-baseline before quoting any figure.",
        p.p50_ms
    );
    assert!(
        p.burst() <= PROBE_BURST_MAX,
        "BOX IS STALLING: probe burst {:.3}x exceeds {PROBE_BURST_MAX:.2}x (median \
         {:.2} ms, max {:.2} ms). The median looks fine, so this is preemption on an \
         otherwise quiet box rather than steady load — the failure mode the median \
         cannot see. Re-measure when nothing else is scheduled.",
        p.burst(),
        p.p50_ms,
        p.max_ms
    );
    (id, p)
}
