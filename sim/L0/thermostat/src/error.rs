//! Why a passive component, or a whole stack, refuses a model.

use std::fmt;

use sim_core::Integrator;

/// Why a passive component, or a [`PassiveStack`](crate::PassiveStack), refuses a model.
///
/// Returned by [`PassiveComponent::validate`](crate::PassiveComponent::validate) and
/// [`PassiveStack::try_install`](crate::PassiveStack::try_install);
/// [`PassiveStack::install`](crate::PassiveStack::install) panics with it.
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
            Self::Other { component, reason } => write!(f, "{component}: {reason}"),
        }
    }
}

impl std::error::Error for ThermostatError {}
