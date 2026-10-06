//! The parameter checks the constructors share, so every `try_` constructor refuses the same
//! way and every panicking one panics with exactly the `try_` error.

use crate::error::ThermostatError;

/// What a real-valued parameter must be. Every domain excludes `NaN` and `±∞`.
#[derive(Clone, Copy, Debug)]
pub enum Domain {
    /// Any finite value.
    Finite,
    /// Finite and `≥ 0`.
    NonNegative,
    /// Finite and `> 0`.
    Positive,
}

impl Domain {
    /// Whether `value` lies in the domain.
    pub fn contains(self, value: f64) -> bool {
        value.is_finite()
            && match self {
                Self::Finite => true,
                Self::NonNegative => value >= 0.0,
                Self::Positive => value > 0.0,
            }
    }

    /// The domain as the refusal states it.
    pub const fn requirement(self) -> &'static str {
        match self {
            Self::Finite => "finite",
            Self::NonNegative => "finite and non-negative",
            Self::Positive => "finite and positive",
        }
    }

    /// `Ok` if `value` lies in the domain, else the refusal naming `parameter`.
    pub fn check(
        self,
        component: &'static str,
        parameter: &str,
        value: f64,
    ) -> Result<(), ThermostatError> {
        if self.contains(value) {
            Ok(())
        } else {
            Err(ThermostatError::InvalidParameter {
                component,
                parameter: parameter.to_owned(),
                value,
                requirement: self.requirement(),
            })
        }
    }

    /// [`Self::check`] on every entry of `values`, naming the first refused one `name[i]`.
    pub fn check_each(
        self,
        component: &'static str,
        name: &str,
        values: &[f64],
    ) -> Result<(), ThermostatError> {
        values
            .iter()
            .position(|&v| !self.contains(v))
            .map_or(Ok(()), |i| {
                self.check(component, &format!("{name}[{i}]"), values[i])
            })
    }
}

/// `Ok` if `parameter` has `expected` entries, one per `per`.
pub const fn check_len(
    component: &'static str,
    parameter: &'static str,
    len: usize,
    expected: usize,
    per: &'static str,
) -> Result<(), ThermostatError> {
    if len == expected {
        Ok(())
    } else {
        Err(ThermostatError::LengthMismatch {
            component,
            parameter,
            len,
            expected,
            per,
        })
    }
}

/// `Ok` if every edge joins two different elements, below `n` when given, and no pair appears
/// twice in either order (two edges on one pair would add their couplings).
pub fn check_edges(
    component: &'static str,
    n: Option<usize>,
    edges: &[(usize, usize)],
) -> Result<(), ThermostatError> {
    let mut seen = std::collections::HashSet::with_capacity(edges.len());
    for &(i, j) in edges {
        if let Some(n) = n
            && (i >= n || j >= n)
        {
            return Err(ThermostatError::EdgeOutOfRange {
                component,
                edge: (i, j),
                n,
            });
        }
        if i == j {
            return Err(ThermostatError::SelfEdge {
                component,
                edge: (i, j),
            });
        }
        if !seen.insert((i.min(j), i.max(j))) {
            return Err(ThermostatError::RepeatedEdge {
                component,
                edge: (i, j),
            });
        }
    }
    Ok(())
}

/// The value of a constructor's `try_` form, or a panic with exactly its refusal.
#[track_caller]
#[allow(clippy::panic)] // the documented refusal of every panicking constructor
pub fn or_panic<T>(result: Result<T, ThermostatError>) -> T {
    match result {
        Ok(value) => value,
        Err(e) => panic!("{e}"),
    }
}

/// The parameter an `InvalidParameter` refusal names, or `None` for any other outcome.
#[cfg(test)]
pub fn refused_parameter<T>(result: Result<T, ThermostatError>) -> Option<String> {
    match result {
        Err(ThermostatError::InvalidParameter { parameter, .. }) => Some(parameter),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_domain_refuses_nan_and_infinities() {
        for domain in [Domain::Finite, Domain::NonNegative, Domain::Positive] {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                assert!(!domain.contains(bad), "{domain:?} took {bad}");
            }
        }
    }

    #[test]
    fn domains_split_at_zero() {
        let at =
            |v: f64| [Domain::Finite, Domain::NonNegative, Domain::Positive].map(|d| d.contains(v));
        assert_eq!(at(-1.0), [true, false, false]);
        assert_eq!(at(0.0), [true, true, false]);
        assert_eq!(at(f64::MIN_POSITIVE), [true, true, true]);
    }

    #[test]
    fn check_each_names_the_first_refused_entry() {
        assert_eq!(
            Domain::NonNegative.check_each("C", "gamma", &[1.0, -2.0, f64::NAN]),
            Err(ThermostatError::InvalidParameter {
                component: "C",
                parameter: "gamma[1]".to_owned(),
                value: -2.0,
                requirement: "finite and non-negative",
            })
        );
        assert_eq!(
            Domain::NonNegative.check_each("C", "gamma", &[0.0, 1.0]),
            Ok(())
        );
    }

    #[test]
    fn check_edges_refuses_self_edges_repeats_and_out_of_range_spins() {
        assert_eq!(
            check_edges("C", None, &[(0, 1), (1, 0)]),
            Err(ThermostatError::RepeatedEdge {
                component: "C",
                edge: (1, 0),
            })
        );
        assert_eq!(
            check_edges("C", None, &[(2, 2)]),
            Err(ThermostatError::SelfEdge {
                component: "C",
                edge: (2, 2),
            })
        );
        assert_eq!(
            check_edges("C", Some(3), &[(0, 3)]),
            Err(ThermostatError::EdgeOutOfRange {
                component: "C",
                edge: (0, 3),
                n: 3,
            })
        );
        assert_eq!(
            check_edges("C", Some(3), &[(3, 0)]),
            Err(ThermostatError::EdgeOutOfRange {
                component: "C",
                edge: (3, 0),
                n: 3,
            })
        );
        assert_eq!(check_edges("C", None, &[(0, 3), (5, 4)]), Ok(()));
        assert_eq!(check_edges("C", Some(3), &[(0, 1), (2, 1)]), Ok(()));
    }
}
