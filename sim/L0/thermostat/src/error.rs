//! Why a passive component or a stack refuses a model, or a constructor refuses a parameter.

use std::fmt;

use sim_core::Integrator;

/// Why a passive component, or a [`PassiveStack`](crate::PassiveStack), refuses a model, or a
/// component's constructor refuses its parameters.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum ThermostatError {
    /// The component acts on a DOF the model does not have.
    DofOutOfRange {
        /// The component's type name.
        component: &'static str,
        /// The DOF index it acts on.
        dof: usize,
        /// The model's DOF count, `model.nv`.
        nv: usize,
    },
    /// The component reads a DOF's position, but the DOF has no position coordinate of its
    /// own: it is one of a ball joint's DOFs or a free joint's rotation DOFs.
    NoPositionCoordinate {
        /// The component's type name.
        component: &'static str,
        /// The DOF index it reads.
        dof: usize,
    },
    /// The component reads a control channel the model does not have.
    CtrlOutOfRange {
        /// The component's type name.
        component: &'static str,
        /// The control index it reads.
        ctrl: usize,
        /// The model's control count, `model.nu`.
        nu: usize,
    },
    /// The component does not support the model's integrator.
    UnsupportedIntegrator {
        /// The component's type name.
        component: &'static str,
        /// The model's integrator.
        integrator: Integrator,
        /// Why.
        reason: &'static str,
    },
    /// A constructor parameter is outside its domain.
    InvalidParameter {
        /// The component's type name.
        component: &'static str,
        /// The parameter, with the entry for a vector (`gamma[3]`).
        parameter: String,
        /// The value supplied.
        value: f64,
        /// What the parameter must be.
        requirement: &'static str,
    },
    /// The component is given more DOFs than it supports.
    TooManyDofs {
        /// The component's type name.
        component: &'static str,
        /// The DOF count supplied.
        dofs: usize,
        /// The most it supports.
        max: usize,
    },
    /// A list parameter has the wrong number of entries.
    LengthMismatch {
        /// The component's type name.
        component: &'static str,
        /// The list.
        parameter: &'static str,
        /// Its entry count.
        len: usize,
        /// The entry count it needs.
        expected: usize,
        /// What it needs one entry per (`"edge"`, `"spin"`).
        per: &'static str,
    },
    /// An edge joins an element to itself.
    SelfEdge {
        /// The component's type name.
        component: &'static str,
        /// The edge, as given.
        edge: (usize, usize),
    },
    /// An edge repeats the pair of an earlier edge, in either order: the two would add their
    /// couplings.
    RepeatedEdge {
        /// The component's type name.
        component: &'static str,
        /// The edge, as given.
        edge: (usize, usize),
    },
    /// An edge names a spin outside `0..n`.
    EdgeOutOfRange {
        /// The component's type name.
        component: &'static str,
        /// The edge, as given.
        edge: (usize, usize),
        /// The spin count.
        n: usize,
    },
    /// An Ising problem has more spins than the exact solvers enumerate.
    TooManySpins {
        /// The component's type name, or the function's.
        component: &'static str,
        /// The spin count supplied.
        spins: usize,
        /// The most it supports.
        max: usize,
    },
    /// A distribution does not list its configurations in order: entry `k` must be
    /// configuration `k`.
    ConfigurationOrder {
        /// The component's type name.
        component: &'static str,
        /// The first entry out of order.
        entry: usize,
        /// The configuration it holds.
        config: u32,
    },
    /// A count parameter is outside its domain.
    InvalidCount {
        /// The component's type name.
        component: &'static str,
        /// The parameter.
        parameter: &'static str,
        /// The value supplied.
        value: usize,
        /// What the parameter must be.
        requirement: &'static str,
    },
    /// The model's timestep is not positive and finite. sim-core's `step` and `forward`
    /// refuse such a model too (`StepError::InvalidTimestep`); install refuses it first.
    InvalidTimestep {
        /// The component's type name.
        component: &'static str,
        /// The model's timestep.
        timestep: f64,
    },
    /// The thermostat's noise variance on a DOF, `2·γ·kT/h` at the largest temperature it
    /// can apply, is not finite at the model's timestep `h`.
    NoiseOverflow {
        /// The component's type name.
        component: &'static str,
        /// The DOF.
        dof: usize,
        /// The model's timestep.
        timestep: f64,
    },
    /// The model already has a passive callback (`Model::cb_passive`), from another stack or
    /// set directly. A model holds one, so installing would replace it; call
    /// `Model::clear_passive_callback` first to replace it on purpose.
    PassiveCallbackInstalled,
    /// Any other reason, for components outside this crate.
    Other {
        /// The component's type name.
        component: &'static str,
        /// Why.
        reason: String,
    },
}

impl fmt::Display for ThermostatError {
    #[allow(clippy::too_many_lines)] // one short arm per variant
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DofOutOfRange { component, dof, nv } => {
                write!(
                    f,
                    "{component} acts on DOF {dof}, but the model has {nv} DOFs"
                )
            }
            Self::NoPositionCoordinate { component, dof } => write!(
                f,
                "{component} reads the position of DOF {dof}, which has no position coordinate \
                 of its own (a ball joint's DOFs and a free joint's rotation DOFs move a \
                 quaternion)"
            ),
            Self::CtrlOutOfRange {
                component,
                ctrl,
                nu,
            } => write!(
                f,
                "{component} reads control {ctrl}, but the model has {nu} controls"
            ),
            Self::UnsupportedIntegrator {
                component,
                integrator,
                reason,
            } => write!(
                f,
                "{component} does not support the {integrator:?} integrator: {reason}"
            ),
            Self::InvalidParameter {
                component,
                parameter,
                value,
                requirement,
            } => write!(
                f,
                "{component}: {parameter} must be {requirement}, got {value}"
            ),
            Self::TooManyDofs {
                component,
                dofs,
                max,
            } => write!(f, "{component} supports at most {max} DOFs, got {dofs}"),
            Self::LengthMismatch {
                component,
                parameter,
                len,
                expected,
                per,
            } => write!(
                f,
                "{component}: {parameter} has length {len}, expected {expected} (one per {per})"
            ),
            Self::SelfEdge {
                component,
                edge: (i, j),
            } => write!(f, "{component}: edge ({i}, {j}) joins an element to itself"),
            Self::RepeatedEdge {
                component,
                edge: (i, j),
            } => write!(
                f,
                "{component}: edge ({i}, {j}) repeats the pair of an earlier edge (in either order)"
            ),
            Self::EdgeOutOfRange {
                component,
                edge: (i, j),
                n,
            } => write!(
                f,
                "{component}: edge ({i}, {j}) names a spin outside 0..{n}"
            ),
            Self::TooManySpins {
                component,
                spins,
                max,
            } => write!(f, "{component} supports at most {max} spins, got {spins}"),
            Self::ConfigurationOrder {
                component,
                entry,
                config,
            } => write!(
                f,
                "{component}: entry {entry} is configuration {config:#b}; entries must be in \
                 configuration order"
            ),
            Self::InvalidCount {
                component,
                parameter,
                value,
                requirement,
            } => write!(
                f,
                "{component}: {parameter} must be {requirement}, got {value}"
            ),
            Self::InvalidTimestep {
                component,
                timestep,
            } => write!(
                f,
                "{component}: the model's timestep must be positive and finite, got {timestep}"
            ),
            Self::NoiseOverflow {
                component,
                dof,
                timestep,
            } => write!(
                f,
                "{component}: the noise variance 2·gamma·k_b_t/timestep on DOF {dof} is not \
                 finite at timestep {timestep}"
            ),
            Self::PassiveCallbackInstalled => write!(
                f,
                "the model already has a passive callback; call Model::clear_passive_callback \
                 first to replace it"
            ),
            Self::Other { component, reason } => write!(f, "{component}: {reason}"),
        }
    }
}

impl std::error::Error for ThermostatError {}
