//! One rule for a joint's, a tendon's and an actuator's control, force and
//! activation `limited` and `range`, as MuJoCo 3.5.0 resolves and checks them
//! (`user_objects.cc`: `islimited` `:184-189`, `checklimited` `:171-181`, the
//! joint's checks `:2895-2924`, the tendon's `:6412-6421`, the actuator's
//! `:6868-6894`). A joint's and a tendon's `actuatorfrclimited` and
//! `actuatorfrcrange`, which this crate does not read, are not among them
//! (ledger L315).
//!
//! [`resolve`] decides whether a pair limits and finds its first fault; the
//! builder records the fault and names it later (book L21, ledger L363): a
//! fault this crate found before L21 where it found it, MuJoCo's other faults
//! after every check this crate makes before building the model, in MuJoCo's
//! order, and `D-AUTO-RANGE`'s after the keyframes.

use sim_core::MjJointType;

use crate::error::{ElementKind, Location, MjcfError};
use crate::types::TfAuto;

/// Which pair: whose `limited` and `range`, for the words of its errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pair {
    /// A joint's `limited` and `range`, of its kind.
    Joint(MjJointType),
    /// A tendon's `limited` and `range`.
    Tendon,
    /// An actuator's `ctrllimited` and `ctrlrange`.
    Ctrl,
    /// An actuator's `forcelimited` and `forcerange`.
    Force,
    /// An actuator's `actlimited` and `actrange`.
    Act,
}

impl Pair {
    /// MuJoCo's word for the element, as its errors write it.
    fn entity(self) -> &'static str {
        match self {
            Self::Joint(_) => "joint",
            Self::Tendon => "tendon",
            Self::Ctrl | Self::Force | Self::Act => "actuator",
        }
    }

    /// The prefix of the pair's two attributes (`ctrl` for `ctrllimited`).
    fn prefix(self) -> &'static str {
        match self {
            Self::Joint(_) | Self::Tendon => "",
            Self::Ctrl => "ctrl",
            Self::Force => "force",
            Self::Act => "act",
        }
    }

    /// MuJoCo's error for a limited range whose first bound is not below
    /// its second.
    fn not_increasing(self) -> &'static str {
        match self {
            Self::Joint(_) => "range[0] should be smaller than range[1] in joint",
            Self::Tendon => "invalid limits in tendon",
            Self::Ctrl => "invalid control range for actuator",
            Self::Force => "invalid force range for actuator",
            Self::Act => "invalid actrange for actuator",
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
    /// refuse an automatic backwards range, 01-decisions) after the
    /// keyframes, the pair taken as unlimited by every check before.
    AutomaticNotIncreasing,
    /// A limited range whose first bound is not below its second, or no range
    /// (MuJoCo's `0 0`); not a ball joint's.
    NotIncreasing,
    /// A limited ball joint's range whose first bound is not 0.
    BallLowerNotZero,
}

impl LimitFault {
    /// Whether MuJoCo makes this refusal: all but `D-AUTO-RANGE`'s, which is
    /// named after the keyframes, as this crate's other refusals MuJoCo does
    /// not make are.
    pub fn is_mujocos(self) -> bool {
        self != Self::AutomaticNotIncreasing
    }

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
                 (MuJoCo leaves it unlimited, D-AUTO-RANGE)"
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

/// An actuator's first fault, in MuJoCo 3.5.0's order
/// (`mjCActuator::Compile`, `user_objects.cc:6866-6944`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorFault {
    /// A fault of one of its three pairs.
    Limit(Pair, LimitFault),
    /// A limited activation and no dynamics.
    ActivationWithoutDynamics,
    /// A muscle gain's or bias's parameters (`gainprm` or `biasprm`), with
    /// MuJoCo's words.
    Muscle(&'static str, &'static str),
}

impl ActuatorFault {
    /// The error for this fault, with MuJoCo's words.
    pub fn error(self) -> MjcfError {
        match self {
            Self::Limit(pair, fault) => fault.error(pair),
            Self::ActivationWithoutDynamics => MjcfError::invalid_value(
                "actuator",
                "actrange",
                "actrange specified but dyntype is 'none' in actuator",
            ),
            Self::Muscle(attribute, words) => {
                MjcfError::invalid_value("actuator", attribute, words)
            }
        }
    }
}

/// An actuator's first fault MuJoCo makes: each pair's `checklimited`,
/// force, control then activation (`:6868-6880`); then a limited range that
/// does not increase, in the same order (`:6882-6891`); then a limited
/// activation with no dynamics (`:6892-6894`); then the parameters of a
/// muscle gain, then of a muscle bias (`:6914-6945`), each `[range0, range1,
/// force, scale, lmin, lmax, vmax, fpmax, fvmax]`. An automatic range that
/// does not increase is not among them (`D-AUTO-RANGE`, [`automatic_pair`]).
pub fn actuator_fault(
    force: Limit,
    ctrl: Limit,
    act: Limit,
    dynamics: bool,
    muscle: [Option<(&'static str, &[f64; 9])>; 2],
) -> Option<ActuatorFault> {
    let pairs = [(Pair::Force, force), (Pair::Ctrl, ctrl), (Pair::Act, act)];
    let fault_of = |checklimited: bool| {
        pairs.iter().find_map(|&(pair, limit)| {
            limit
                .fault
                .filter(|&f| {
                    f.is_mujocos() && (f == LimitFault::RangeWithoutLimited) == checklimited
                })
                .map(|f| ActuatorFault::Limit(pair, f))
        })
    };
    if let Some(fault) = fault_of(true).or_else(|| fault_of(false)) {
        return Some(fault);
    }
    if act.limited && !dynamics {
        return Some(ActuatorFault::ActivationWithoutDynamics);
    }
    muscle.into_iter().flatten().find_map(|(attribute, prm)| {
        let words = if prm[0] >= prm[1] {
            "range[0]<range[1] required in muscle"
        } else if prm[4] >= 1.0 || prm[5] <= 1.0 {
            "lmin<1<lmax required in muscle"
        } else if prm[3] <= 0.0 || prm[6] <= 0.0 || prm[7] <= 0.0 || prm[8] <= 0.0 {
            "positive scale, vmax, fpmax, fvmax required in muscle"
        } else {
            return None;
        };
        Some(ActuatorFault::Muscle(attribute, words))
    })
}

impl super::ModelBuilder {
    /// The first automatic range that does not increase (`D-AUTO-RANGE`):
    /// a joint's, in the order they were built, then a tendon's, then an
    /// actuator's, located as the element.
    pub fn automatic_range_fault(&self) -> Option<MjcfError> {
        let automatic = Some(LimitFault::AutomaticNotIncreasing);
        let joint = (0..self.jnt_type.len()).find(|&j| self.jnt_limit_fault[j] == automatic);
        if let Some(j) = joint {
            let body = self.jnt_body[j];
            let at = Location::nth(ElementKind::Joint, self.jnt_name[j].as_deref(), j).within_nth(
                ElementKind::Body,
                self.body_name[body].as_deref(),
                body,
            );
            return Some(
                LimitFault::AutomaticNotIncreasing
                    .error(Pair::Joint(self.jnt_type[j]))
                    .at(at),
            );
        }
        let tendon = (0..self.tendon_type.len()).find(|&t| self.tendon_limit_fault[t] == automatic);
        if let Some(t) = tendon {
            let at = Location::nth(ElementKind::Tendon, self.tendon_name[t].as_deref(), t);
            return Some(
                LimitFault::AutomaticNotIncreasing
                    .error(Pair::Tendon)
                    .at(at),
            );
        }
        self.actuator_automatic
            .iter()
            .enumerate()
            .find_map(|(i, pair)| {
                pair.map(|pair| {
                    let at =
                        Location::nth(ElementKind::Actuator, self.actuator_name[i].as_deref(), i);
                    LimitFault::AutomaticNotIncreasing.error(pair).at(at)
                })
            })
    }
}

/// The first of an actuator's pairs, force, control then activation, whose
/// automatic range does not increase (`D-AUTO-RANGE`).
pub fn automatic_pair(force: Limit, ctrl: Limit, act: Limit) -> Option<Pair> {
    [(Pair::Force, force), (Pair::Ctrl, ctrl), (Pair::Act, act)]
        .into_iter()
        .find(|(_, limit)| limit.fault == Some(LimitFault::AutomaticNotIncreasing))
        .map(|(pair, _)| pair)
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
        let cases: [(Pair, Option<TfAuto>, Option<(f64, f64)>, bool, bool, Option<LimitFault>); 22] = [
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
            (Pair::Tendon, t, None, true, true, Some(NotIncreasing)),
            (Pair::Tendon, None, Some((1.0, 0.0)), true, false, Some(AutomaticNotIncreasing)),
            (Pair::Tendon, None, Some((-0.5, 0.5)), false, true, Some(RangeWithoutLimited)),
            (Pair::Ctrl, t, None, true, true, Some(NotIncreasing)),
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
        let ctrl = LimitFault::RangeWithoutLimited
            .error(Pair::Ctrl)
            .to_string();
        assert!(
            ctrl.contains("actuator has `ctrlrange` but not `ctrllimited`"),
            "{ctrl}"
        );
        for (pair, words) in [
            (Pair::Ctrl, "invalid control range for actuator"),
            (Pair::Force, "invalid force range for actuator"),
            (Pair::Act, "invalid actrange for actuator"),
        ] {
            let error = LimitFault::NotIncreasing.error(pair).to_string();
            assert!(error.contains(words), "{error}");
        }
        let hinge = LimitFault::NotIncreasing.error(HINGE).to_string();
        assert!(
            hinge.contains("range[0] should be smaller than range[1] in joint"),
            "{hinge}"
        );
        let tendon = LimitFault::NotIncreasing.error(Pair::Tendon).to_string();
        assert!(tendon.contains("invalid limits in tendon"), "{tendon}");
        let ball = LimitFault::BallLowerNotZero.error(BALL).to_string();
        assert!(
            ball.contains("range[0] should be 0 in ball joint"),
            "{ball}"
        );
    }
}
