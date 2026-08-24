//! The timing contract — asserted in **both** build configurations.
//!
//! ## Why this is an integration binary and not a `#[cfg(test)] mod`
//!
//! `profile`'s counters are process-global `AtomicU64` statics, not thread-locals
//! (which is why every harness that reads them requires `--test-threads=1`). A
//! unit test inside the crate would share those counters with the 337 other lib
//! tests, several of which construct `Timer`s once `phase-timing` is on, so any
//! exact-count assertion would race. **A separate integration binary gets its own
//! process and therefore its own statics**, which removes the problem rather than
//! working around it. One test function, so nothing races inside here either.
//!
//! ## What it is for
//!
//! Mutation testing measured `profile.rs` at 0 of 29 and the module produces
//! every figure in recon §2d / §2i / §2j. The unit tests added alongside cover
//! the pure slot arithmetic; this covers the part that has side effects —
//! `Timer` recording on drop, `reset` zeroing, `snapshot` reading back — which
//! nothing asserted anywhere. A `Drop` that quietly stopped recording would have
//! left every published measurement wrong and every test green.
//!
//! ⚠ The `#[ignore]`d harnesses do check related properties (`phase_shares.rs`
//! asserts no slot reads zero after a real solve; §2j's split control asserts the
//! two children exhaust their parent), but they are `#[ignore]`d and reference-box
//! gated, so CI never runs them. This one runs by default, in whichever
//! configuration is being built.

#![allow(clippy::panic, clippy::expect_used)]

use sim_soft::profile::{self, Phase};

/// Enough work that `Instant::elapsed()` is unambiguously non-zero, without
/// depending on how coarse the platform clock is.
fn burn() {
    let mut acc = 0u64;
    for i in 0..200_000u64 {
        acc = acc.wrapping_add(i.wrapping_mul(2_654_435_761));
    }
    std::hint::black_box(acc);
}

#[test]
fn timer_records_on_drop_reset_zeroes_and_the_off_build_does_neither() {
    profile::reset();
    let before = profile::snapshot();
    for ph in Phase::ALL {
        assert_eq!(before.nanos(ph), 0, "{} dirty at start", ph.label());
        assert_eq!(before.calls(ph), 0, "{} dirty at start", ph.label());
    }

    {
        let _t = profile::Timer::start(Phase::NumericFactor);
        burn();
    }
    let after = profile::snapshot();

    #[cfg(feature = "phase-timing")]
    {
        // The guard records when it DROPS, into its OWN slot, once.
        assert_eq!(
            after.calls(Phase::NumericFactor),
            1,
            "drop must book exactly one entry"
        );
        assert!(
            after.nanos(Phase::NumericFactor) > 0,
            "drop must book the elapsed time, got 0 ns"
        );
        for ph in Phase::ALL {
            if ph != Phase::NumericFactor {
                assert_eq!(after.calls(ph), 0, "{} was touched", ph.label());
                assert_eq!(after.nanos(ph), 0, "{} was touched", ph.label());
            }
        }

        // `reset` clears every slot, not just the one that moved.
        profile::reset();
        let cleared = profile::snapshot();
        for ph in Phase::ALL {
            assert_eq!(cleared.nanos(ph), 0, "{} survived reset", ph.label());
            assert_eq!(cleared.calls(ph), 0, "{} survived reset", ph.label());
        }
    }

    #[cfg(not(feature = "phase-timing"))]
    {
        // The zero-cost claim, from the outside: with the feature off the guard
        // is a unit struct with no `Drop`, so nothing is recorded and `snapshot`
        // stays all-zero. Documented in `profile`'s module header and, until
        // now, asserted nowhere.
        for ph in Phase::ALL {
            assert_eq!(
                after.nanos(ph),
                0,
                "{} recorded with the feature OFF",
                ph.label()
            );
            assert_eq!(
                after.calls(ph),
                0,
                "{} recorded with the feature OFF",
                ph.label()
            );
        }
    }
}
