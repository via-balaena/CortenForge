//! Per-phase wall-clock timers for the solver's hot path, behind the
//! `phase-timing` feature.
//!
//! ## Why this exists as a feature and not a scratch patch
//!
//! `docs/SIM_SOFT_REALTIME_RECON.md` §2d's phase-share table is the basis for
//! every "where would an optimisation pay?" argument in the real-time arc — it
//! is what said the numeric factorization dominates above ~1–2 k free DOF, and
//! what R3's whole premise rests on. It was produced by temporary scratch edits
//! that were then reverted (§9), which has cost twice:
//!
//! - **Its two IPC contact rows were still `pre-R0`** and had never been
//!   re-measured. ⚠ These timers re-measure the SHARES, and that is all they do.
//!   Do NOT difference shares across two sessions to recover a whole-step credit:
//!   §2d finding 3 tried it and was wrong by 37 %, because the premise "the other
//!   phases' absolute cost is unchanged" does not hold across two instruments.
//!   A whole-step credit is a ratio of wall times — see `tests/r0_ab.rs`.
//! - Re-deriving anything means re-applying patches from a prose description,
//!   which is exactly how a measurement stops being reproducible.
//!
//! §9 already prescribes the fix: *"the right move is not to re-apply scratch
//! patches but to land the timing slots properly behind a feature flag."* This
//! is that.
//!
//! ## Zero cost when off, and that is a hard requirement
//!
//! The default build must be **byte-identical** to one without this module —
//! these timers sit inside the Newton loop, and a solver that is measurably
//! different when instrumented cannot measure itself. With `phase-timing`
//! disabled, [`Timer::start`] returns a unit struct whose `Drop` is empty and
//! whose construction is `#[inline(always)]`, so the optimiser removes the call
//! sites entirely. There is no branch, no atomic, and no `Instant::now`.
//!
//! ⚠ **What the numbers mean when it IS on.** Timing is wall-clock and
//! ACCUMULATES across every call in the measured window, including the extra
//! internal-force assemblies Armijo backtracking performs — that is deliberate,
//! since §2d's shares are shares of a whole step. `faer` factorizes under rayon,
//! so a phase's wall time spans its parallel region rather than summing per-core
//! work; the shares are therefore comparable to each other and to §2d, but they
//! are not CPU-time.
//!
//! ⚠ **FULL-ORDER PATH ONLY.** These five slots cover
//! `solver::backward_euler`'s hot path. The REDUCED solver
//! (`backward_euler::reduced`) is uninstrumented — its `project_tangent` and its
//! dense `r×r` factorization fall into no slot — so a reduced run's
//! instrumented-vs-wall figure will be far below 100 % and its shares mean
//! nothing. Instrumenting R1's path is a separate job and is not done here.
//!
//! ## Use
//!
//! ```
//! use sim_soft::profile::{self, Phase};
//! profile::reset();
//! // ... run the steps to be measured ...
//! let p = profile::snapshot();
//! let numf = 100.0 * p.share(Phase::NumericFactor);
//! # assert!(numf >= 0.0);
//! ```
//!
//! A real doctest, not an `ignore` fence: it compiles as an EXTERNAL crate, so it
//! checks the surface a consumer actually sees. It passes in both feature states —
//! with the feature off every slot reads zero, which is the documented behaviour.

/// The phases §2d reports, one slot each.
///
/// Deliberately the same five the recon's table uses, in its column order, so a
/// re-measurement drops straight into that table rather than needing a mapping.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Phase {
    /// Internal-force assembly (`asm force`) — including Armijo's re-evaluations.
    AssembleForce,
    /// Free-DOF tangent assembly (`asm tangent`).
    AssembleTangent,
    /// Numeric Cholesky/LU of the free tangent (`numeric factor`).
    NumericFactor,
    /// Triangular solves against that factor (`tri solve`).
    TriangularSolve,
    /// The contact PATH: position marshalling, the active-pair search, and the
    /// gradient/Hessian scatter.
    ///
    /// ⚠ **Not "the cost of contact".** Both timed blocks call
    /// `slice_to_vec3s(x_curr)` — a full `Vec<Vec3>` copy of every vertex —
    /// and then `active_pairs`, before any pair exists. Those run under
    /// `NullContact` too, so a contact-free fixture reports the FLOOR cost of
    /// having the contact path compiled in rather than zero: `cantilever 80×8`
    /// measures **1.6 %** of its frame here with no contact whatsoever.
    ///
    /// ⚠ **NESTED inside BOTH [`Self::AssembleForce`] and
    /// [`Self::AssembleTangent`]** — contact does gradient work in
    /// `assemble_global_int_force` and Hessian work (with its own
    /// `active_pairs` search) in `assemble_free_hessian_triplets`. Both are
    /// booked here. ⚠ An earlier version timed only the force side, which
    /// understated contact AND silently folded the contact Hessian into
    /// `AssembleTangent` — enough to break a cross-session comparison of that
    /// share on a contact fixture, while leaving contact-free fixtures correct.
    /// It is excluded from
    /// [`Phases::total_nanos`] so the four disjoint slots still sum correctly,
    /// and [`Phases::share`] reports it as a fraction of that same total, i.e.
    /// "of which contact" rather than "plus contact". §2d's column reads the
    /// same way, which is why its rows come to ~99 % rather than over 100 %.
    Contact,

    /// Reduced path: `x = x_rest + Φq` reconstruction (`O(n·r)`).
    ReducedExpand,
    /// Reduced path: `Φᵀr`, the residual projection (`O(n·r)`).
    ReducedProjectCovector,
    /// Reduced path: `ΦᵀAΦ`, EXCLUDING the tangent assembly it calls (`O(n·r²)`).
    ///
    /// The assembly is already booked to [`Self::AssembleTangent`], so this slot
    /// starts after the triplets exist and the two stay disjoint.
    ReducedProjectTangent,
    /// Reduced path: the dense `r × r` Cholesky/LU and its solve (`O(r³)`).
    ///
    /// ⚠ The one cost centre here that hyper-reduction (R3) CANNOT touch — ECSW
    /// replaces assembly and projection with a weighted sum over a sampled
    /// element subset, and leaves this untouched. Its share is therefore R3's
    /// Amdahl ceiling.
    ReducedDenseSolve,
}

/// Number of timing slots.
const N_SLOTS: usize = 9;

impl Phase {
    /// Every slot, in §2d's column order.
    pub const ALL: [Self; N_SLOTS] = [
        Self::AssembleForce,
        Self::AssembleTangent,
        Self::NumericFactor,
        Self::TriangularSolve,
        Self::Contact,
        Self::ReducedExpand,
        Self::ReducedProjectCovector,
        Self::ReducedProjectTangent,
        Self::ReducedDenseSolve,
    ];

    /// §2d's column heading for this phase.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::AssembleForce => "asm force",
            Self::AssembleTangent => "asm tangent",
            Self::NumericFactor => "numeric factor",
            Self::TriangularSolve => "tri solve",
            Self::Contact => "contact",
            Self::ReducedExpand => "red expand",
            Self::ReducedProjectCovector => "red proj r",
            Self::ReducedProjectTangent => "red proj K",
            Self::ReducedDenseSolve => "red dense solve",
        }
    }

    /// Whether hyper-reduction (R3/ECSW) can attack this phase.
    ///
    /// ECSW replaces assembly-and-projection over ALL elements with a weighted
    /// sum over a sampled subset, so everything that scales with the element
    /// count or `n` is reducible. The dense `r × r` solve is not, and neither is
    /// [`Self::Contact`], whose pair search is not an element-quadrature cost.
    /// `1 / (1 − reducible share)` is R3's Amdahl ceiling.
    #[must_use]
    pub const fn ecsw_reducible(self) -> bool {
        match self {
            Self::AssembleForce
            | Self::AssembleTangent
            | Self::ReducedExpand
            | Self::ReducedProjectCovector
            | Self::ReducedProjectTangent => true,
            Self::NumericFactor
            | Self::TriangularSolve
            | Self::Contact
            | Self::ReducedDenseSolve => false,
        }
    }

    const fn index(self) -> usize {
        match self {
            Self::AssembleForce => 0,
            Self::AssembleTangent => 1,
            Self::NumericFactor => 2,
            Self::TriangularSolve => 3,
            Self::Contact => 4,
            Self::ReducedExpand => 5,
            Self::ReducedProjectCovector => 6,
            Self::ReducedProjectTangent => 7,
            Self::ReducedDenseSolve => 8,
        }
    }
}

/// Accumulated nanoseconds and call counts per phase.
///
/// ⚠ Two things this is not. [`Self::total_nanos`] sums the four DISJOINT slots
/// — [`Phase::Contact`] is nested inside [`Phase::AssembleForce`] and would
/// double-count — and it is **not** elapsed step time either, since it omits
/// everything outside an instrumented phase (residual evaluation, the line
/// search's arithmetic, allocation). Shares are of *instrumented* work. §2d's
/// rows do not sum to 100 % for the same two reasons.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Phases {
    nanos: [u64; N_SLOTS],
    calls: [u64; N_SLOTS],
}

impl Phases {
    /// Accumulated nanoseconds in `phase`.
    #[must_use]
    pub const fn nanos(&self, phase: Phase) -> u64 {
        self.nanos[phase.index()]
    }

    /// Number of timed entries into `phase`.
    #[must_use]
    pub const fn calls(&self, phase: Phase) -> u64 {
        self.calls[phase.index()]
    }

    /// Milliseconds in `phase`.
    //
    // cast_precision_loss: nanoseconds exceed f64's 2^52 exact-integer range
    // only past ~52 days of ACCUMULATED time in one slot. No measured window
    // comes within six orders of magnitude of that.
    #[allow(clippy::cast_precision_loss)]
    #[must_use]
    pub fn millis(&self, phase: Phase) -> f64 {
        self.nanos(phase) as f64 / 1.0e6
    }

    /// Summed nanoseconds across the four DISJOINT slots. Excludes
    /// [`Phase::Contact`], which is nested inside [`Phase::AssembleForce`].
    #[must_use]
    pub fn total_nanos(&self) -> u64 {
        Phase::ALL
            .iter()
            .filter(|p| !matches!(p, Phase::Contact))
            .map(|p| self.nanos(*p))
            .sum()
    }

    /// `phase` as a fraction of [`Self::total_nanos`]. Zero when nothing ran,
    /// rather than `NaN`, so a caller that prints unconditionally is safe.
    //
    // cast_precision_loss: same bound as `millis`.
    #[allow(clippy::cast_precision_loss)]
    #[must_use]
    pub fn share(&self, phase: Phase) -> f64 {
        let total = self.total_nanos();
        if total == 0 {
            0.0
        } else {
            self.nanos(phase) as f64 / total as f64
        }
    }
}

#[cfg(feature = "phase-timing")]
mod imp {
    use super::{Phase, Phases};
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Instant;

    // `Relaxed` throughout: these are diagnostic tallies, and nothing orders
    // against them. Same reasoning as `CpuNewtonSolver::factorizations`.
    static NANOS: [AtomicU64; super::N_SLOTS] = [const { AtomicU64::new(0) }; super::N_SLOTS];
    static CALLS: [AtomicU64; super::N_SLOTS] = [const { AtomicU64::new(0) }; super::N_SLOTS];

    /// Scope guard: times from construction to drop.
    pub struct Timer {
        slot: usize,
        t0: Instant,
    }

    impl Timer {
        /// Begin timing `phase`; the elapsed time lands in its slot on drop.
        #[must_use]
        pub fn start(phase: Phase) -> Self {
            Self {
                // `index()` is private to the parent module and visible here
                // because this is a child of it — one definition of the slot
                // order, shared with `Phases`, so the two cannot drift.
                slot: phase.index(),
                t0: Instant::now(),
            }
        }
    }

    impl Drop for Timer {
        fn drop(&mut self) {
            // `as u64` on an elapsed duration: `u128` nanoseconds only exceeds
            // `u64` past ~584 years, which no measured window reaches.
            #[allow(clippy::cast_possible_truncation)]
            let ns = self.t0.elapsed().as_nanos() as u64;
            NANOS[self.slot].fetch_add(ns, Ordering::Relaxed);
            CALLS[self.slot].fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Zero every slot. Call before the window to be measured.
    pub fn reset() {
        for i in 0..super::N_SLOTS {
            NANOS[i].store(0, Ordering::Relaxed);
            CALLS[i].store(0, Ordering::Relaxed);
        }
    }

    /// Read every slot.
    #[must_use]
    pub fn snapshot() -> Phases {
        let mut out = Phases::default();
        for i in 0..super::N_SLOTS {
            out.nanos[i] = NANOS[i].load(Ordering::Relaxed);
            out.calls[i] = CALLS[i].load(Ordering::Relaxed);
        }
        out
    }
}

#[cfg(not(feature = "phase-timing"))]
// inline_always: normally a bad idea, and here it is the entire mechanism.
// These bodies are empty; forcing the inline is what lets the optimiser delete
// the call sites so the default build carries no timing code at all. Leaving it
// to the inliner's judgement would make the zero-cost claim a hope.
#[allow(clippy::inline_always)]
mod imp {
    use super::{Phase, Phases};

    /// No-op stand-in. Construction is `#[inline(always)]` and `Drop` is not
    /// implemented, so every call site optimises away entirely — the default
    /// build carries no timing code at all.
    pub struct Timer;

    impl Timer {
        /// No-op stand-in for the timed [`Timer::start`]; compiles away.
        #[inline(always)]
        #[must_use]
        pub const fn start(_phase: Phase) -> Self {
            Self
        }
    }

    /// No-op without the feature.
    #[inline(always)]
    pub const fn reset() {}

    /// All-zero without the feature.
    #[inline(always)]
    #[must_use]
    pub fn snapshot() -> Phases {
        Phases::default()
    }
}

pub use imp::{Timer, reset, snapshot};
