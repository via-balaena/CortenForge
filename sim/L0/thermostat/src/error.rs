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
