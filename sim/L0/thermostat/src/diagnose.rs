//! `Diagnose` — minimal introspection trait for passive components.
//!
//! Per chassis Decision 4, the diagnostic surface is intentionally
//! minimal: a single `diagnostic_summary` method that returns a
//! human-readable string. Components that need richer introspection
//! (per-DOF state dumps, histograms, energy traces) implement them
//! as inherent methods in the per-component module; the trait stays
//! small so a `Vec<&dyn Diagnose>` over a heterogeneous stack remains
//! ergonomic and doesn't need to grow new required methods every
//! time a phase adds a new diagnostic.
//!
//! The string return is deliberately untyped — diagnostic output is
//! for humans reading test failures and notebook cells, not for
//! parsers. If a future phase needs structured diagnostics, the
//! right move is to add a separate trait alongside `Diagnose`, not
//! to bolt on to it.

/// A passive component that can produce a one-shot human-readable
/// summary of its current state.
///
/// Implemented by `LangevinThermostat` (and any future passive
/// component) to support `dbg!`-style debugging and test-failure
/// messages without forcing every caller to know the concrete type.
pub trait Diagnose {
    /// Return a one-line human-readable summary of the component's
    /// current state. Format is up to the implementor, but should
    /// include enough information to identify the component and its
    /// configuration (e.g. type name, key parameters, seed).
    fn diagnostic_summary(&self) -> String;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummyDiagnose {
        label: String,
        value: f64,
    }

    impl Diagnose for DummyDiagnose {
        fn diagnostic_summary(&self) -> String {
            format!(
                "DummyDiagnose(label={}, value={:.3})",
                self.label, self.value
            )
        }
    }

    #[test]
    fn diagnostic_summary_returns_implementor_format() {
        let d = DummyDiagnose {
            label: "test".to_string(),
            value: 2.5,
        };
        assert_eq!(
            d.diagnostic_summary(),
            "DummyDiagnose(label=test, value=2.500)"
        );
    }

    #[test]
    fn diagnose_is_object_safe() {
        // Compile-time test: Diagnose must be dyn-compatible so a
        // PassiveStack can hold a heterogeneous Vec<&dyn Diagnose>.
        let d: Box<dyn Diagnose> = Box::new(DummyDiagnose {
            label: "boxed".to_string(),
            value: 0.0,
        });
        assert!(d.diagnostic_summary().contains("boxed"));
    }
}
