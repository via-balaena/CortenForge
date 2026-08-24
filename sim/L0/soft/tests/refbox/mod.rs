//! Measurement preconditions: **which box**, and **is it idle**.
//!
//! ## Why this exists
//!
//! A frame budget is an ABSOLUTE quantity, so the gap to it belongs to the
//! machine it was measured on. `docs/SIM_SOFT_REALTIME_RECON.md` spent four
//! revisions not honouring that:
//!
//! - §2d finding 4 measures three fixtures on two boxes and gets transfer
//!   ratios of `1.60×`, `1.56×`, `1.08×` … and `0.51×`. No single box factor
//!   does that, so a `ms/step` quoted without its box means nothing.
//! - §2's own header records the SAME 3 000-DOF fixture measuring **4.9 ms and
//!   28.0 ms** in two runs under background load — a `5.7×` spread, which dwarfs
//!   any cross-box difference measured since.
//!
//! The second is the bigger threat and the easier one to forget: the box is
//! right there, it looks idle, and the number is wrong anyway.
//!
//! ## What is enforced, and what is only recorded
//!
//! **Hard-failed** — the hardware the cost scales with: `hw.model` and the P/E
//! core split. A different machine is not this machine, and a gate spent
//! against it is meaningless.
//!
//! **Stamped only** — RAM, OS version, `rustc` version. These drift for reasons
//! unrelated to speed, and the toolchain is already pinned by
//! `rust-toolchain.toml`, so failing on them would be friction without
//! protection. They print with every measurement so a number cannot travel
//! without its context.
//!
//! ⚠ **Transfer to another box is UNVALIDATED and this module does not offer
//! it.** Validating a transfer rule needs at least two boxes; one is available.
//! The only cross-box evidence in the recon shows transfer failing in OPPOSITE
//! directions across fixture families, so a plausible-looking scaling factor
//! would be worse than none.
//!
//! ⚠ This is deliberately NOT `tests/common/mod.rs`: that module is shared by
//! eight physics tests and has no business gaining a dependency on `sysctl`.

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
/// Call this from any measurement whose number is spent against a gate. The
/// recon's `10×` kill floor and its `13.5–15.8×` frame-budget requirement are
/// both quantities on THIS machine; produced anywhere else they are a different
/// number wearing the same name, which is the failure that cost four document
/// revisions.
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
    /// Slowest step over the median — the **gross-stall** statistic.
    ///
    /// ⚠ **This is the WEAKER of the two checks, and an earlier revision of this
    /// comment had it backwards.** It was written asserting that burst is "the
    /// contention statistic" and that a median would hide what it catches. The
    /// pilot falsified that: under a 4-core background load burst reads
    /// `1.063×` against an idle maximum of `1.060×` — no separation at all —
    /// while the absolute median moves cleanly out of band. Sustained partial
    /// load slows EVERY step, so it lands in the median and leaves this ratio
    /// alone.
    ///
    /// It is kept because it catches the opposite failure: a few long stalls on
    /// an otherwise quiet box (12-core load reads `2.37–5.87×`), which is
    /// preemption the median genuinely would hide. Two checks, two failure
    /// modes; see [`require_quiet_box`].
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

/// Run the contention probe: `cantilever 40×4` (3 000 free DOF) at [`PROBE_DT`],
/// which §2d puts at 57.5 % `numeric factor` — the same phase the real
/// measurements are bottlenecked on.
///
/// ⚠ **The measurement fixtures cannot police themselves**, which is why this
/// exists separately. The IPC ramp deepens its indenter every step, so its
/// `max/p50` is `1.31` from physics alone; the `cantilever 80×8` transient runs
/// 8× min-to-max. A threshold loose enough for those would never catch
/// contention. This probe does the SAME work every step, so its spread has only
/// one source.
#[must_use]
pub fn probe() -> Probe {
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
/// **Piloted, not chosen.** Measured idle maximum is `24.70 ms`; the lightest
/// contaminated case in the pilot (a 2-core background load) has minimum
/// `25.83 ms`. This sits in that gap, near the maximum-margin separator, so it
/// admits every idle run observed and rejects every contaminated one — a 2-core
/// load already inflates the median `6.5 %`.
///
/// ⚠ Headroom above the observed idle maximum is only `3.3 %`. That is
/// deliberate — this is the sensitive check — but it means a genuine baseline
/// shift (a warmer box, a newer OS, a faster solver) will read as contention.
/// The remedy is to re-run `tests/refbox_pilot.rs` and re-baseline both
/// constants together, NOT to widen this one.
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

/// The full precondition for a gate-bearing measurement: **the reference box,
/// and quiet.**
///
/// Runs the probe (~0.6 s) and refuses the measurement if the box is loaded.
/// Both checks are needed and they catch different things — see
/// [`Probe::burst`] for why the obvious single statistic is the wrong one.
pub fn require_quiet_box() -> (BoxIdentity, Probe) {
    let id = require_reference_box();
    let p = probe();
    p.report();
    assert!(
        p.p50_ms <= PROBE_P50_CEILING_MS,
        "BOX IS CONTENDED: probe median {:.2} ms exceeds {PROBE_P50_CEILING_MS:.2} ms \
         (quiet reference is {REF_PROBE_P50_MS:.2} ms). Every step is slow, which is \
         sustained background load, not jitter — the recon has a 5.7x spread on one \
         fixture from exactly this. Close what is running and re-measure. If the box \
         is genuinely idle, its baseline has moved: re-run tests/refbox_pilot.rs and \
         re-baseline, do not widen the threshold.",
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
