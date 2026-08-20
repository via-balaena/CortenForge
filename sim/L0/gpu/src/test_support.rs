//! What a GPU test does when the machine has no adapter — one policy, one place.
//!
//! Every test in this crate needs a [`GpuContext`], and on a machine without a
//! Vulkan or Metal adapter it cannot have one. Skipping is right for a developer
//! on such a machine; it is wrong for CI, where a suite that skips reports `ok`
//! and the code it covers is never executed.
//!
//! ⚠ **That is not hypothetical, it is measured.** On a GPU-less
//! `ubuntu-latest` runner all 51 of this crate's tests early-returned and the
//! step concluded SUCCESS while the log read `no suitable GPU adapter found`
//! ten times over (run 32353063528). Coverage there read 1.2 % of production
//! lines; the same code measures 97.7 % where an adapter exists. Green meant
//! "no GPU", not "correct".
//!
//! So the choice is explicit and belongs to the RUN, not to this code: set
//! [`REQUIRE_GPU`] and a missing adapter is a hard failure. CI sets it on the
//! jobs that install a software adapter, so their green means the pipelines
//! really ran.

use crate::context::{GpuContext, GpuError};

/// Environment variable by which a run declares an adapter MUST be present.
pub const REQUIRE_GPU: &str = "CF_REQUIRE_GPU";

/// What to do when [`GpuContext::new`] finds no adapter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoAdapter {
    /// Report the skip and return — a developer without a GPU.
    Skip,
    /// Fail loudly — a run that declared it must have one.
    Fail,
}

/// The policy, as a pure function of the variable's value.
///
/// ★ Separated from the context call so it can be tested on any machine. The
/// interesting case — no adapter, and the run demanded one — cannot be reached
/// by a test on a machine that HAS an adapter, which is every machine this
/// suite is developed on. A predicate over a string can be tested anywhere.
///
/// Unset, empty and `0` all mean skip, so `CF_REQUIRE_GPU=0` reads the way
/// anyone would expect rather than being a surprising second way to say yes.
pub fn no_adapter_action(require: Option<&str>) -> NoAdapter {
    match require.map(str::trim) {
        None | Some("" | "0") => NoAdapter::Skip,
        Some(_) => NoAdapter::Fail,
    }
}

/// A context for a test, or `None` when this run tolerates having no adapter.
///
/// `suite` names the caller in the skip line, so a partial skip is legible.
///
/// Refuses to continue, rather than returning `None`, when the run required an
/// adapter — see [`report_missing_adapter`].
pub fn gpu_context_or_skip(suite: &str) -> Option<GpuContext> {
    match GpuContext::new() {
        Ok(ctx) => Some(ctx),
        Err(e) => {
            report_missing_adapter(suite, &e, action_from_env());
            None
        }
    }
}

/// The action this run has configured, read from the environment.
///
/// Named for what it returns rather than for the question it answers. A name
/// like `required` yielding a [`NoAdapter`] reads at the call site as "required
/// equals fail", which is the opposite of clarifying.
pub fn action_from_env() -> NoAdapter {
    no_adapter_action(std::env::var(REQUIRE_GPU).ok().as_deref())
}

/// Announce a missing adapter, or refuse to continue without one.
///
/// ★ Split from [`gpu_context_or_skip`] for one reason: the refusal is the whole
/// point of this module and it cannot be reached on a machine that HAS an
/// adapter, so leaving it inside the context call left it permanently untested.
/// Taking the decision as an argument lets a test drive the branch directly.
///
/// # Panics
///
/// When `action` is [`NoAdapter::Fail`].
// Panicking IS the contract: the variable exists to convert a silent skip into a
// failure, and a test harness reports a panic as the failure it is. Returning an
// error would put the decision back on each of the 51 call sites, which is the
// arrangement that produced the fail-open.
#[allow(clippy::panic)]
pub fn report_missing_adapter(suite: &str, err: &GpuError, action: NoAdapter) {
    match action {
        NoAdapter::Skip => eprintln!("  Skipping {suite} (no GPU): {err}"),
        NoAdapter::Fail => panic!(
            "{REQUIRE_GPU} is set, so this run requires a GPU adapter, but none was \
             found: {err}. Install a software Vulkan driver (`mesa-vulkan-drivers` \
             provides lavapipe) or unset {REQUIRE_GPU} to allow skipping."
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ★ The whole point is that an unset variable must NOT fail the run — a
    /// developer without a GPU still runs this suite — while any deliberate
    /// setting must. Both directions are asserted; pinning only one would let a
    /// constant satisfy it.
    #[test]
    fn only_a_deliberate_setting_demands_an_adapter() {
        assert_eq!(no_adapter_action(None), NoAdapter::Skip, "unset");
        assert_eq!(no_adapter_action(Some("")), NoAdapter::Skip, "empty");
        assert_eq!(no_adapter_action(Some("  ")), NoAdapter::Skip, "blank");
        assert_eq!(
            no_adapter_action(Some("0")),
            NoAdapter::Skip,
            "explicit off"
        );

        assert_eq!(
            no_adapter_action(Some("1")),
            NoAdapter::Fail,
            "the CI setting"
        );
        assert_eq!(
            no_adapter_action(Some("true")),
            NoAdapter::Fail,
            "any value"
        );
        assert_eq!(no_adapter_action(Some(" 1 ")), NoAdapter::Fail, "padded");
    }

    /// ★★ The refusal is this module's entire reason to exist, and until the
    /// branch was split out nothing exercised it — mutating the panic to a plain
    /// `None` passed the whole suite. That is the same shape as the defect this
    /// module was written to fix, one level up.
    #[test]
    #[should_panic(expected = "requires a GPU adapter")]
    fn a_required_adapter_that_is_missing_refuses_to_continue() {
        report_missing_adapter("probe", &GpuError::NoAdapter, NoAdapter::Fail);
    }

    /// ★ The negative control for the test above: the same call with the same
    /// error must NOT panic when the run never demanded an adapter, or a
    /// developer without a GPU could not run this suite at all.
    #[test]
    fn a_missing_adapter_that_was_not_required_only_reports() {
        report_missing_adapter("probe", &GpuError::NoAdapter, NoAdapter::Skip);
    }

    /// ★★ Guards the hole the refusal cannot see. `report_missing_adapter` only
    /// ever runs on the `Err` path, so a bug that DISCARDS a working adapter —
    /// returning `None` with a perfectly good device present — skips every test
    /// and never trips the refusal. Under `CF_REQUIRE_GPU` the run has asserted a
    /// device exists, so `None` is then a contradiction and must fail.
    ///
    /// ⚠ Vacuous when the variable is unset, which is deliberate: off a CI runner
    /// there may genuinely be no adapter. It bites exactly where it must.
    #[test]
    fn a_run_that_requires_an_adapter_actually_receives_one() {
        if action_from_env() == NoAdapter::Fail {
            assert!(
                gpu_context_or_skip("required-adapter probe").is_some(),
                "{REQUIRE_GPU} is set, so an adapter was promised, but none was handed back"
            );
        }
    }
}
