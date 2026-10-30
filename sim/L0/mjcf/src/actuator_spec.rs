//! An actuator's values as MuJoCo 3.5.0's spec holds them (`mjsActuator`):
//! what a default class keeps, one for every actuator kind, and what an
//! actuator starts from, a copy of its class's. Each actuator element, in a
//! class or not, writes its spec in place as MuJoCo's reader writes it
//! (`mjXReader::OneActuator`, `xml_native_reader.cc:2287-2470`, and the
//! `mjs_setTo*` functions, `user_api.cc:889-1066`), so a class written by
//! two kinds, or used by another, builds what MuJoCo builds. A value no
//! writer set is `None`, MuJoCo's default (`mjs_defaultActuator`) where the
//! model is built.
//!
//! Where MuJoCo silently builds something other than the document asks, the
//! write refuses it instead: each such rule names its divergence.

use sim_core::{ActuatorDynamics, BiasType, GainType};

use crate::defaults::over;
use crate::error::{MjcfError, Result};
use crate::keywords;
use crate::types::{MjcfActuator, MjcfActuatorType, Prefix, TfAuto};

/// Who writes a spec: a default class's element, onto its class's, or an
/// actuator, onto a copy of its class's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Writer {
    /// A default class's actuator element.
    Class,
    /// An actuator.
    Actuator,
}

/// The `mjsActuator` values an actuator element writes, each `None` (a
/// parameter array's each slot) until one does.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ActuatorSpec {
    pub gaintype: Option<GainType>,
    pub biastype: Option<BiasType>,
    pub dyntype: Option<ActuatorDynamics>,
    pub gainprm: [Option<f64>; 9],
    pub biasprm: [Option<f64>; 9],
    pub dynprm: [Option<f64>; 10],
    pub gear: Option<Prefix<6>>,
    pub ctrlrange: Option<(f64, f64)>,
    pub forcerange: Option<(f64, f64)>,
    pub actrange: Option<(f64, f64)>,
    pub lengthrange: Option<(f64, f64)>,
    pub ctrllimited: Option<TfAuto>,
    pub forcelimited: Option<TfAuto>,
    pub actlimited: Option<TfAuto>,
    pub actearly: Option<bool>,
    pub inheritrange: Option<f64>,
    pub cranklength: Option<f64>,
    pub group: Option<i32>,
    pub nsample: Option<i32>,
    pub interp: Option<String>,
    pub delay: Option<f64>,
    pub user: Option<Vec<f64>>,
    /// What the stricter rules need of the spec's past ([`Provenance`]).
    provenance: Provenance,
}

/// What the stricter rules (D-MUSCLE-SENTINEL, D-DAMPER-CLASS-KV) need to
/// know of a spec's past: which muscle values a writer asked for, and which
/// asked-for values MuJoCo replaced or dropped while a class was written. A
/// class refuses none of them; an actuator that takes one is refused.
#[derive(Debug, Clone, Default, PartialEq)]
struct Provenance {
    /// Each `gainprm` slot, then `dynprm`'s first two, whose value a muscle
    /// attribute or a `<general>`'s parameter wrote; a kind's own write to
    /// the slot (`mjs_setTo*`) clears it.
    asked: [bool; MUSCLE_SLOTS],
    /// The asked-for value a class's `<muscle>` replaced in each slot.
    lost: [Option<f64>; MUSCLE_SLOTS],
    /// The damping a class's `<damper>` that wrote no `kv` dropped.
    lost_damping: Option<f64>,
}

/// The slots [`Provenance`] follows: `gainprm`'s nine, then `dynprm`'s
/// first two (index 9 + k).
const MUSCLE_SLOTS: usize = 11;

/// MuJoCo's `gainprm` where no writer sets one (`mjs_defaultActuator`).
pub const DEFAULT_GAINPRM: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

/// MuJoCo's `dynprm` where no writer sets one (`mjs_defaultActuator`).
pub const DEFAULT_DYNPRM: [f64; 10] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

/// `mjs_setToMuscle`'s replacements (`user_api.cc:1013-1023`): a `dynprm`
/// or `gainprm` slot holding the value `mjs_defaultActuator` gives it is set
/// to the muscle's own default, the slot's attribute named.
const MUSCLE_DYNPRM: [(usize, f64, f64); 2] = [(0, 1.0, 0.01), (1, 0.0, 0.04)];
const MUSCLE_GAINPRM: [(usize, f64, f64, &str); 9] = [
    (0, 1.0, 0.75, "range"),
    (1, 0.0, 1.05, "range"),
    (2, 0.0, -1.0, "force"),
    (3, 0.0, 200.0, "scale"),
    (4, 0.0, 0.5, "lmin"),
    (5, 0.0, 1.6, "lmax"),
    (6, 0.0, 1.5, "vmax"),
    (7, 0.0, 1.3, "fpmax"),
    (8, 0.0, 1.2, "fvmax"),
];

impl ActuatorSpec {
    /// `gainprm[k]`, MuJoCo's default where no writer set it.
    fn gain(&self, k: usize) -> f64 {
        self.gainprm[k].unwrap_or(DEFAULT_GAINPRM[k])
    }

    /// `biasprm[k]`, 0 where no writer set it.
    fn bias(&self, k: usize) -> f64 {
        self.biasprm[k].unwrap_or(0.0)
    }

    /// `dynprm[k]`, MuJoCo's default where no writer set it.
    fn dynamics(&self, k: usize) -> f64 {
        self.dynprm[k].unwrap_or(DEFAULT_DYNPRM[k])
    }

    /// A written range's two values, MuJoCo's `0 0` where none is.
    fn range(range: Option<(f64, f64)>) -> (f64, f64) {
        range.unwrap_or((0.0, 0.0))
    }

    /// `gainprm[k]` set by a kind's own write: no writer asked for it.
    fn set_gain(&mut self, k: usize, value: f64) {
        self.gainprm[k] = Some(value);
        self.provenance.asked[k] = false;
        self.provenance.lost[k] = None;
        if k == 2 {
            self.provenance.lost_damping = None;
        }
    }

    /// `gainprm[k]` as a muscle attribute or a `<general>`'s parameter asks.
    fn ask_gain(&mut self, k: usize, value: f64) {
        self.set_gain(k, value);
        self.provenance.asked[k] = true;
    }

    /// `dynprm[k]` set by a kind's own write: no writer asked for it.
    fn set_dynamics(&mut self, k: usize, value: f64) {
        self.dynprm[k] = Some(value);
        if k < 2 {
            self.provenance.asked[9 + k] = false;
            self.provenance.lost[9 + k] = None;
        }
    }

    /// `dynprm[k]` as a muscle attribute or a `<general>`'s parameter asks.
    fn ask_dynamics(&mut self, k: usize, value: f64) {
        self.set_dynamics(k, value);
        if k < 2 {
            self.provenance.asked[9 + k] = true;
        }
    }

    /// `actuator`, of its kind, written onto this spec as `OneActuator`
    /// writes it: the attributes every kind reads, then its kind's
    /// `mjs_setTo*`, then `user`.
    ///
    /// # Errors
    ///
    /// [`MjcfError::InvalidValue`] for a value MuJoCo refuses, with its
    /// message, or one it would silently replace (each refusal names its
    /// divergence); [`MjcfError::InvalidKeyword`] for a `<general>`'s type
    /// no keyword names (an actuator built in code).
    pub fn write(&mut self, actuator: &MjcfActuator, writer: Writer) -> Result<()> {
        self.write_common(actuator);
        match actuator.actuator_type {
            MjcfActuatorType::General => self.write_general(actuator)?,
            MjcfActuatorType::Motor => self.set_to_motor(),
            MjcfActuatorType::Position => self
                .set_to_position(actuator)
                .map_err(ServoRefusal::into_error)?,
            MjcfActuatorType::IntVelocity => self.set_to_int_velocity(actuator, writer)?,
            MjcfActuatorType::Velocity => {
                let kv = actuator.kv.unwrap_or_else(|| self.gain(0));
                self.set_to_velocity(kv);
            }
            MjcfActuatorType::Damper => self.set_to_damper(actuator, writer)?,
            MjcfActuatorType::Cylinder => self.set_to_cylinder(actuator)?,
            MjcfActuatorType::Muscle => self.set_to_muscle(actuator, writer)?,
            MjcfActuatorType::Adhesion => self.set_to_adhesion(actuator)?,
        }
        if !actuator.user.is_empty() {
            self.user = Some(actuator.user.clone());
        }
        Ok(())
    }

    /// The attributes `OneActuator` reads alike for every kind
    /// (`xml_native_reader.cc:2297-2308`), and `cranklength` (`:2349`).
    fn write_common(&mut self, actuator: &MjcfActuator) {
        set(&mut self.group, actuator.group);
        set(&mut self.nsample, actuator.nsample);
        set(&mut self.interp, actuator.interp.clone());
        set(&mut self.delay, actuator.delay);
        set(&mut self.ctrllimited, actuator.ctrllimited);
        set(&mut self.forcelimited, actuator.forcelimited);
        set(&mut self.actlimited, actuator.actlimited);
        set(&mut self.ctrlrange, actuator.ctrlrange);
        set(&mut self.forcerange, actuator.forcerange);
        set(&mut self.actrange, actuator.actrange);
        set(&mut self.lengthrange, actuator.lengthrange);
        self.gear = over(actuator.gear, self.gear);
        set(&mut self.cranklength, actuator.cranklength);
    }

    /// A `<general>`'s own types, parameters and `actearly`
    /// (`xml_native_reader.cc:2373-2391`): each as written, a parameter
    /// array's first values replaced by those written.
    fn write_general(&mut self, actuator: &MjcfActuator) -> Result<()> {
        if let Some(name) = &actuator.dyntype {
            self.dyntype = Some(
                keywords::dyn_type(name)
                    .ok_or_else(|| MjcfError::invalid_keyword("general", "dyntype", name))?,
            );
        }
        if let Some(name) = &actuator.gaintype {
            self.gaintype = Some(
                keywords::gain_type(name)
                    .ok_or_else(|| MjcfError::invalid_keyword("general", "gaintype", name))?,
            );
        }
        if let Some(name) = &actuator.biastype {
            self.biastype = Some(
                keywords::bias_type(name)
                    .ok_or_else(|| MjcfError::invalid_keyword("general", "biastype", name))?,
            );
        }
        if actuator.actearly.is_some() {
            self.actearly = actuator.actearly;
        }
        if let Some(dynprm) = actuator.dynprm {
            for (k, value) in dynprm.given().iter().enumerate() {
                self.ask_dynamics(k, *value);
            }
        }
        if let Some(gainprm) = actuator.gainprm {
            for (k, value) in gainprm.given().iter().enumerate() {
                self.ask_gain(k, *value);
            }
        }
        write_prefix(&mut self.biasprm, actuator.biasprm);
        Ok(())
    }

    /// `mjs_setToMotor` (`user_api.cc:889-898`).
    fn set_to_motor(&mut self) {
        self.set_gain(0, 1.0);
        self.dyntype = Some(ActuatorDynamics::None);
        self.gaintype = Some(GainType::Fixed);
        self.biastype = Some(BiasType::None);
    }

    /// `mjs_setToPosition` (`user_api.cc:903-937`), with the `kp` and
    /// `inheritrange` `OneActuator` passes it: the element's, else the
    /// spec's (`xml_native_reader.cc:2399-2427`); `kv`, `dampratio` and
    /// `timeconst` only as the element writes them. A refusal leaves the
    /// values MuJoCo set before it.
    fn set_to_position(
        &mut self,
        actuator: &MjcfActuator,
    ) -> std::result::Result<(), ServoRefusal> {
        let element = actuator.actuator_type.as_str();
        let refuse = |attribute: &str, reason: &str| {
            ServoRefusal::Gains(MjcfError::invalid_value(element, attribute, reason))
        };
        let kp = actuator.kp.unwrap_or_else(|| self.gain(0));
        self.set_gain(0, kp);
        self.biasprm[1] = Some(-kp);
        if actuator.kv.is_some() && actuator.dampratio.is_some() {
            return Err(refuse(
                "dampratio",
                "kv and dampratio cannot both be defined",
            ));
        }
        if let Some(kv) = actuator.kv {
            if kv < 0.0 {
                return Err(refuse("kv", "kv cannot be negative"));
            }
            self.biasprm[2] = Some(-kv);
        }
        if let Some(dampratio) = actuator.dampratio {
            if dampratio < 0.0 {
                return Err(refuse("dampratio", "dampratio cannot be negative"));
            }
            self.biasprm[2] = Some(dampratio);
        }
        if let Some(timeconst) = actuator.timeconst {
            if timeconst < 0.0 {
                return Err(refuse("timeconst", "timeconst cannot be negative"));
            }
            self.set_dynamics(0, timeconst);
            self.dyntype = Some(if timeconst == 0.0 {
                ActuatorDynamics::None
            } else {
                ActuatorDynamics::FilterExact
            });
        }
        let inheritrange = actuator.inheritrange.or(self.inheritrange);
        self.inheritrange = inheritrange;
        let (lo, hi) = Self::range(self.ctrlrange);
        if inheritrange.is_some_and(|r| r > 0.0) && (lo != 0.0 || hi != 0.0) {
            return Err(ServoRefusal::Range(MjcfError::invalid_value(
                element,
                "inheritrange",
                "ctrlrange and inheritrange cannot both be defined",
            )));
        }
        self.gaintype = Some(GainType::Fixed);
        self.biastype = Some(BiasType::Affine);
        Ok(())
    }

    /// `mjs_setToIntVelocity` (`user_api.cc:942-954`): `mjs_setToPosition`,
    /// whose refusal it drops, then integrator dynamics, its activation
    /// always limited, and `actrange` beside `inheritrange` refused, the
    /// refusal MuJoCo reports. Dropping a refusal builds the servo with no
    /// bias; that is refused here (D-INTVELOCITY-SERVO): a gain's at once, a
    /// `ctrlrange` beside `inheritrange` after MuJoCo's own, for an actuator
    /// only: a class's loads, as in MuJoCo (measured), and refuses each
    /// actuator that takes both.
    fn set_to_int_velocity(&mut self, actuator: &MjcfActuator, writer: Writer) -> Result<()> {
        let dropped = match self.set_to_position(actuator) {
            Ok(()) => None,
            Err(ServoRefusal::Gains(refusal)) => return Err(refusal),
            Err(ServoRefusal::Range(refusal)) => Some(refusal),
        };
        self.dyntype = Some(ActuatorDynamics::Integrator);
        self.actlimited = Some(TfAuto::True);
        let (lo, hi) = Self::range(self.actrange);
        if self.inheritrange.is_some_and(|r| r > 0.0) && (lo != 0.0 || hi != 0.0) {
            return Err(MjcfError::invalid_value(
                "intvelocity",
                "inheritrange",
                "actrange and inheritrange cannot both be defined",
            ));
        }
        match dropped {
            Some(refusal) if writer == Writer::Actuator => Err(refusal),
            _ => Ok(()),
        }
    }

    /// `mjs_setToVelocity` (`user_api.cc:959-967`), its `kv` the element's,
    /// else the spec's `gainprm[0]` (`xml_native_reader.cc:2432-2436`).
    fn set_to_velocity(&mut self, kv: f64) {
        self.biasprm = [Some(0.0); 9];
        self.set_gain(0, kv);
        self.biasprm[2] = Some(-kv);
        self.dyntype = Some(ActuatorDynamics::None);
        self.gaintype = Some(GainType::Fixed);
        self.biastype = Some(BiasType::Affine);
    }

    /// `mjs_setToDamper` (`user_api.cc:972-988`), its `kv` the element's,
    /// else 0 (`xml_native_reader.cc:2440-2444`), not its class's: a damper
    /// that writes no `kv` drops a damping its class carries (an affine
    /// gain's velocity term) or one a class's damper dropped before it. An
    /// actuator that would be built so is refused here (D-DAMPER-CLASS-KV;
    /// MuJoCo builds kv 0, measured); a class records the drop for its
    /// actuators.
    fn set_to_damper(&mut self, actuator: &MjcfActuator, writer: Writer) -> Result<()> {
        let carried = self.gaintype == Some(GainType::Affine) && self.gain(2) != 0.0;
        let dropped = match actuator.kv {
            Some(_) => None,
            None if carried => Some(self.gain(2)),
            None => self.provenance.lost_damping,
        };
        if let (Some(damping), Writer::Actuator) = (dropped, writer) {
            return Err(MjcfError::invalid_value(
                "damper",
                "kv",
                format!("a damper takes no kv from its class (damping {damping}): write its own"),
            ));
        }
        let kv = actuator.kv.unwrap_or(0.0);
        for k in 0..9 {
            self.set_gain(k, 0.0);
        }
        self.set_gain(2, -kv);
        self.provenance.lost_damping = dropped;
        self.ctrllimited = Some(TfAuto::True);
        self.dyntype = Some(ActuatorDynamics::None);
        self.gaintype = Some(GainType::Affine);
        self.biastype = Some(BiasType::None);
        if kv < 0.0 {
            return Err(MjcfError::invalid_value(
                "damper",
                "kv",
                "damping coefficient cannot be negative",
            ));
        }
        let (lo, hi) = Self::range(self.ctrlrange);
        if lo < 0.0 || hi < 0.0 {
            return Err(MjcfError::invalid_value(
                "damper",
                "ctrlrange",
                "damper control range cannot be negative",
            ));
        }
        Ok(())
    }

    /// `mjs_setToCylinder` (`user_api.cc:992-1004`), its `timeconst`, first
    /// `bias` and `area` the element's, else the spec's, and its `diameter`
    /// the element's alone (`xml_native_reader.cc:2448-2459`). MuJoCo reads
    /// `bias`'s three values into one number; a second or third that is not
    /// 0 is refused (D-CYLINDER-BIAS).
    fn set_to_cylinder(&mut self, actuator: &MjcfActuator) -> Result<()> {
        if actuator
            .bias
            .is_some_and(|[_, b1, b2]| b1 != 0.0 || b2 != 0.0)
        {
            return Err(MjcfError::invalid_value(
                "cylinder",
                "bias",
                "a cylinder's bias takes its first value only: the others must be 0",
            ));
        }
        let timeconst = actuator.timeconst.unwrap_or_else(|| self.dynamics(0));
        let bias = actuator.bias.map_or_else(|| self.bias(0), |[b0, _, _]| b0);
        let area = actuator.area.unwrap_or_else(|| self.gain(0));
        self.set_dynamics(0, timeconst);
        self.biasprm[0] = Some(bias);
        self.set_gain(
            0,
            match actuator.diameter {
                Some(diameter) if diameter >= 0.0 => {
                    std::f64::consts::PI / 4.0 * diameter * diameter
                }
                _ => area,
            },
        );
        self.dyntype = Some(ActuatorDynamics::Filter);
        self.gaintype = Some(GainType::Fixed);
        self.biastype = Some(BiasType::Affine);
        Ok(())
    }

    /// `mjs_setToMuscle` (`user_api.cc:1009-1050`): each `dynprm` and
    /// `gainprm` slot still holding `mjs_defaultActuator`'s value set to the
    /// muscle's default, then each value the element writes, `biasprm` a copy
    /// of `gainprm`. Each value MuJoCo's reader passes as unwritten, a
    /// negative one, is refused here (D-MUSCLE-NEGATIVE). A value a muscle
    /// attribute or a `<general>`'s parameter asked for that the
    /// replacements overwrite, where the element does not write that value
    /// itself, refuses an actuator that would be built so, whether it is
    /// replaced here or was in a class (D-MUSCLE-SENTINEL; measured: a
    /// class's `range="1 2"` builds 0.75 2); a class records it for its
    /// actuators. `tausmooth` is the element's, else the spec's `dynprm[2]`
    /// (`xml_native_reader.cc:2462-2465`).
    fn set_to_muscle(&mut self, actuator: &MjcfActuator, writer: Writer) -> Result<()> {
        let refuse = |attribute: &str, reason: String| {
            Err(MjcfError::invalid_value("muscle", attribute, reason))
        };
        let negative = |pair: Option<(f64, f64)>| pair.is_some_and(|(a, b)| a < 0.0 || b < 0.0);
        if negative(actuator.muscle_timeconst) {
            return refuse("timeconst", "muscle timeconst cannot be negative".into());
        }
        if negative(actuator.range) {
            return refuse("range", "muscle range cannot be negative".into());
        }
        let scalars = [
            (3, "scale", actuator.scale),
            (4, "lmin", actuator.lmin),
            (5, "lmax", actuator.lmax),
            (6, "vmax", actuator.vmax),
            (7, "fpmax", actuator.fpmax),
            (8, "fvmax", actuator.fvmax),
        ];
        if let Some((_, attribute, _)) = scalars.iter().find(|(_, _, v)| v.is_some_and(|v| v < 0.0))
        {
            return refuse(attribute, format!("muscle {attribute} cannot be negative"));
        }
        // The slots the element writes itself: their carried values are
        // overwritten as asked.
        let own_dynprm = [actuator.muscle_timeconst.is_some(); 2];
        let own_gainprm = [
            actuator.range.is_some(),
            actuator.range.is_some(),
            actuator.force.is_some(),
            actuator.scale.is_some(),
            actuator.lmin.is_some(),
            actuator.lmax.is_some(),
            actuator.vmax.is_some(),
            actuator.fpmax.is_some(),
            actuator.fvmax.is_some(),
        ];
        let own = |slot: usize| {
            if slot < 9 {
                own_gainprm[slot]
            } else {
                own_dynprm[slot - 9]
            }
        };
        let attribute = |slot: usize| {
            if slot < 9 {
                MUSCLE_GAINPRM[slot].3
            } else {
                "timeconst"
            }
        };
        let replaced = |attribute: &str, value: f64| {
            refuse(
                attribute,
                format!("a muscle replaces the {attribute} {value} its class asked for"),
            )
        };
        if writer == Writer::Actuator {
            for slot in 0..MUSCLE_SLOTS {
                if let Some(value) = self.provenance.lost[slot]
                    && !own(slot)
                {
                    return replaced(attribute(slot), value);
                }
            }
        }
        let slots = MUSCLE_DYNPRM
            .iter()
            .map(|&(k, unset, muscle)| (9 + k, unset, muscle))
            .chain(
                MUSCLE_GAINPRM
                    .iter()
                    .map(|&(k, unset, muscle, _)| (k, unset, muscle)),
            );
        for (slot, unset, muscle) in slots {
            let current = if slot < 9 {
                self.gain(slot)
            } else {
                self.dynamics(slot - 9)
            };
            // MuJoCo's own test is exact (`user_api.cc:1013-1023`).
            #[allow(clippy::float_cmp)]
            if current == unset {
                let lost = (self.provenance.asked[slot] && !own(slot)).then_some(unset);
                if lost.is_some() && writer == Writer::Actuator {
                    return replaced(attribute(slot), unset);
                }
                if slot < 9 {
                    self.set_gain(slot, muscle);
                } else {
                    self.set_dynamics(slot - 9, muscle);
                }
                self.provenance.lost[slot] = lost;
            }
        }
        let tausmooth = actuator.tausmooth.unwrap_or_else(|| self.dynamics(2));
        if tausmooth < 0.0 {
            return refuse("tausmooth", "muscle tausmooth cannot be negative".into());
        }
        self.set_dynamics(2, tausmooth);
        if let Some(force) = actuator.force {
            let class = self.gain(2);
            if force < 0.0 && class > 0.0 {
                return refuse(
                    "force",
                    format!("a negative force cannot replace its class's force {class}"),
                );
            }
        }
        if let Some((t0, t1)) = actuator.muscle_timeconst {
            self.ask_dynamics(0, t0);
            self.ask_dynamics(1, t1);
        }
        if let Some((r0, r1)) = actuator.range {
            self.ask_gain(0, r0);
            self.ask_gain(1, r1);
        }
        if let Some(force) = actuator.force.filter(|f| *f >= 0.0) {
            self.ask_gain(2, force);
        }
        for (k, _, value) in scalars {
            if let Some(value) = value {
                self.ask_gain(k, value);
            }
        }
        self.biasprm = self.gainprm;
        self.dyntype = Some(ActuatorDynamics::Muscle);
        self.gaintype = Some(GainType::Muscle);
        self.biastype = Some(BiasType::Muscle);
        Ok(())
    }

    /// `mjs_setToAdhesion` (`user_api.cc:1055-1066`), its `gain` the
    /// element's, else the spec's `gainprm[0]` (`xml_native_reader.cc:2481`).
    fn set_to_adhesion(&mut self, actuator: &MjcfActuator) -> Result<()> {
        let gain = actuator.gain.unwrap_or_else(|| self.gain(0));
        self.set_gain(0, gain);
        self.ctrllimited = Some(TfAuto::True);
        self.gaintype = Some(GainType::Fixed);
        self.biastype = Some(BiasType::None);
        if gain < 0.0 {
            return Err(MjcfError::invalid_value(
                "adhesion",
                "gain",
                "adhesion gain cannot be negative",
            ));
        }
        let (lo, hi) = Self::range(self.ctrlrange);
        if lo < 0.0 || hi < 0.0 {
            return Err(MjcfError::invalid_value(
                "adhesion",
                "ctrlrange",
                "adhesion control range cannot be negative",
            ));
        }
        Ok(())
    }
}

/// A refusal of `mjs_setToPosition`'s, by its kind: `mjs_setToIntVelocity`
/// drops either, and a class's range one stays dropped.
#[derive(Debug)]
enum ServoRefusal {
    /// `kv`, `dampratio` or `timeconst`.
    Gains(MjcfError),
    /// `ctrlrange` beside `inheritrange`.
    Range(MjcfError),
}

impl ServoRefusal {
    fn into_error(self) -> MjcfError {
        match self {
            Self::Gains(error) | Self::Range(error) => error,
        }
    }
}

/// `slot` set to `written` when it is written.
fn set<T>(slot: &mut Option<T>, written: Option<T>) {
    if written.is_some() {
        *slot = written;
    }
}

/// The first slots of `slots` set to the values `written` gives
/// (`ReadAttr(..., exact = false)`).
fn write_prefix<const N: usize, const M: usize>(
    slots: &mut [Option<f64>; N],
    written: Option<Prefix<M>>,
) {
    if let Some(written) = written {
        for (slot, value) in slots.iter_mut().zip(written.given()) {
            *slot = Some(*value);
        }
    }
}
