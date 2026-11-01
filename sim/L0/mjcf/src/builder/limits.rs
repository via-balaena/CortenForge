//! One rule for every `limited` and `range` pair: a joint's, a tendon's, and
//! an actuator's control, force and activation ranges, as MuJoCo 3.5.0
//! resolves and checks them (`user_objects.cc`: `islimited` `:184-189`,
//! `checklimited` `:171-181`, the joint's checks `:2895-2924`, the tendon's
//! `:6412-6421`, the actuator's `:6866-6890`).
//!
//! [`resolve`] decides whether a pair limits and finds its first fault; the
//! builder records the fault and names it later, in MuJoCo's order among the
//! checks MuJoCo makes and this crate made after the others before (book
//! L21, ledger L363).

use sim_core::MjJointType;

use crate::error::MjcfError;
use crate::types::TfAuto;

/// Which pair: whose `limited` and `range`, for the words of its errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pair {
    /// A joint's `limited` and `range`, of its kind.
    Joint(MjJointType),
}

impl Pair {
    /// MuJoCo's word for the element, as its errors write it.
    fn entity(self) -> &'static str {
        match self {
            Self::Joint(_) => "joint",
        }
    }

    /// The prefix of the pair's two attributes (`ctrl` for `ctrllimited`).
    fn prefix(self) -> &'static str {
        match self {
            Self::Joint(_) => "",
        }
    }

    /// MuJoCo's error for a limited range whose first bound is not below
    /// its second.
    fn not_increasing(self) -> &'static str {
        match self {
            Self::Joint(_) => "range[0] should be smaller than range[1] in joint",
        }
    }
}

/// The first fault in a pair, in MuJoCo's order of its checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitFault {
    /// `<compiler autolimits="false"/>`, `limited` unwritten or `auto`, and
    /// a range other than `0 0` (MuJoCo's `checklimited`).
    RangeWithoutLimited,
    /// `limited` unwritten or `auto`, and a range other than `0 0` whose
    /// first bound is not below its second, which MuJoCo leaves unlimited:
    /// refused (`D-AUTO-RANGE`, stricter; the parity rule's decision to
    /// refuse an automatic backwards range, 01-decisions).
    AutomaticNotIncreasing,
    /// A limited range whose first bound is not below its second, or no range
    /// (MuJoCo's `0 0`); not a ball joint's.
    NotIncreasing,
    /// A limited ball joint's range whose first bound is not 0.
    BallLowerNotZero,
}

impl LimitFault {
    /// The error for this fault in `pair`, with MuJoCo's words.
    pub fn error(self, pair: Pair) -> MjcfError {
        let (entity, prefix) = (pair.entity(), pair.prefix());
        let attribute = format!("{prefix}range");
        let reason = match self {
            Self::RangeWithoutLimited => format!(
                "{entity} has `{prefix}range` but not `{prefix}limited`. set the \
                 autolimits=\"true\" compiler option, specify `{prefix}limited` explicitly \
                 (\"true\" or \"false\"), or remove the `{prefix}range` attribute."
            ),
            Self::AutomaticNotIncreasing => format!(
                "an automatic `{prefix}limited` needs `{prefix}range` with range[0] < range[1] \
                 (MuJoCo leaves it unlimited); write `{prefix}limited`, or a range that increases"
            ),
            Self::NotIncreasing => pair.not_increasing().to_owned(),
            Self::BallLowerNotZero => "range[0] should be 0 in ball joint".to_owned(),
        };
        MjcfError::invalid_value(entity, attribute, reason)
    }
}

/// A pair as MuJoCo resolves it: whether it limits, and its first fault.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limit {
    /// Whether the limit is active.
    pub limited: bool,
    /// The pair's first fault, if any.
    pub fault: Option<LimitFault>,
}

/// A pair's `limited` as written (`None` unwritten) and its `range` (`None`
/// unwritten, MuJoCo's `0 0`), under `<compiler autolimits>`, as MuJoCo
/// resolves it: limited when written `true`, or when unwritten or `auto` and
/// the range increases; a free joint never.
pub fn resolve(
    pair: Pair,
    written: Option<TfAuto>,
    range: Option<(f64, f64)>,
    autolimits: bool,
) -> Limit {
    if pair == Pair::Joint(MjJointType::Free) {
        return Limit {
            limited: false,
            fault: None,
        };
    }
    let (lo, hi) = range.unwrap_or((0.0, 0.0));
    let automatic = matches!(written, None | Some(TfAuto::Auto));
    #[allow(clippy::float_cmp)] // MuJoCo's test: a range of exactly 0 0 is none.
    let has_range = !(lo == 0.0 && hi == 0.0);
    let increasing = lo < hi;
    let limited = written == Some(TfAuto::True) || (automatic && increasing);
    let ball = pair == Pair::Joint(MjJointType::Ball);
    let fault = if automatic && has_range && !autolimits {
        Some(LimitFault::RangeWithoutLimited)
    } else if automatic && has_range && !increasing {
        Some(LimitFault::AutomaticNotIncreasing)
    } else if limited && ball && lo != 0.0 {
        Some(LimitFault::BallLowerNotZero)
    } else if limited && !ball && !increasing {
        Some(LimitFault::NotIncreasing)
    } else {
        None
    };
    Limit { limited, fault }
}

#[cfg(test)]
mod tests {
    use super::{LimitFault, Pair, resolve};
    use crate::types::TfAuto;
    use sim_core::MjJointType;

    const HINGE: Pair = Pair::Joint(MjJointType::Hinge);
    const BALL: Pair = Pair::Joint(MjJointType::Ball);

    /// Each branch of the rule, MuJoCo's and decision 18's (documents of
    /// each, with MuJoCo 3.5.0's results, are `tests/limits.rs`'s).
    #[test]
    fn each_pair_resolves_as_mujoco() {
        use LimitFault::{
            AutomaticNotIncreasing, BallLowerNotZero, NotIncreasing, RangeWithoutLimited,
        };
        let (t, f, a) = (Some(TfAuto::True), Some(TfAuto::False), Some(TfAuto::Auto));
        let free = Pair::Joint(MjJointType::Free);
        #[rustfmt::skip]
        let cases: [(Pair, Option<TfAuto>, Option<(f64, f64)>, bool, bool, Option<LimitFault>); 18] = [
            (HINGE, t, Some((1.0, -1.0)), true, true, Some(NotIncreasing)),
            (HINGE, t, None, true, true, Some(NotIncreasing)),
            (HINGE, t, Some((-1.0, 1.0)), false, true, None),
            (HINGE, None, Some((-1.0, 1.0)), true, true, None),
            (HINGE, None, Some((1.0, -1.0)), true, false, Some(AutomaticNotIncreasing)),
            (HINGE, a, Some((1.0, 1.0)), true, false, Some(AutomaticNotIncreasing)),
            (HINGE, None, Some((0.0, 0.0)), true, false, None),
            (HINGE, None, None, true, false, None),
            (HINGE, None, Some((-1.0, 1.0)), false, true, Some(RangeWithoutLimited)),
            (HINGE, a, Some((1.0, -1.0)), false, false, Some(RangeWithoutLimited)),
            (HINGE, None, Some((0.0, 0.0)), false, false, None),
            (HINGE, f, Some((1.0, -1.0)), false, false, None),
            (BALL, t, Some((45.0, 0.0)), true, true, Some(BallLowerNotZero)),
            (BALL, t, None, true, true, None),
            (BALL, t, Some((0.0, -10.0)), true, true, None),
            (BALL, None, Some((0.0, -10.0)), true, false, Some(AutomaticNotIncreasing)),
            (BALL, None, Some((0.0, 60.0)), true, true, None),
            (free, t, Some((1.0, -1.0)), false, false, None),
        ];
        for (i, (pair, written, range, autolimits, limited, fault)) in cases.into_iter().enumerate()
        {
            let got = resolve(pair, written, range, autolimits);
            assert_eq!((got.limited, got.fault), (limited, fault), "case {i}");
        }
    }

    /// Each pair's errors carry MuJoCo's words.
    #[test]
    fn each_fault_has_mujocos_words() {
        let checklimited = LimitFault::RangeWithoutLimited.error(HINGE).to_string();
        assert!(
            checklimited.contains("joint has `range` but not `limited`"),
            "{checklimited}"
        );
        let hinge = LimitFault::NotIncreasing.error(HINGE).to_string();
        assert!(
            hinge.contains("range[0] should be smaller than range[1] in joint"),
            "{hinge}"
        );
        let ball = LimitFault::BallLowerNotZero.error(BALL).to_string();
        assert!(
            ball.contains("range[0] should be 0 in ball joint"),
            "{ball}"
        );
    }
}
