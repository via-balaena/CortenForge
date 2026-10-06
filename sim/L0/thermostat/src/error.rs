//! Why a passive component, or a whole stack, refuses a model.

use std::fmt;

/// Why a passive component, or a [`PassiveStack`](crate::PassiveStack), refuses a model.
///
/// Returned by [`PassiveComponent::validate`](crate::PassiveComponent::validate) and
/// [`PassiveStack::try_install`](crate::PassiveStack::try_install);
/// [`PassiveStack::install`](crate::PassiveStack::install) panics with it.
#[derive(Debug, Clone, PartialEq, Eq)]
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
    /// The component reads a DOF's position, but that DOF belongs to a joint without a
    /// single position coordinate (a ball or free joint).
    NotScalarJoint {
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
    /// [`PassiveStack::try_install`](crate::PassiveStack::try_install) found a passive
    /// callback already installed on the model.
    AlreadyInstalled,
    /// The component does not support the model's integrator.
    UnsupportedIntegrator {
        /// The component's type name.
        component: &'static str,
        /// Why.
        reason: &'static str,
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
            Self::NotScalarJoint { component, dof } => write!(
                f,
                "{component} reads the position of DOF {dof}, which belongs to a ball or free \
                 joint; only slide and hinge joints have a single position coordinate"
            ),
            Self::CtrlOutOfRange {
                component,
                ctrl,
                nu,
            } => write!(
                f,
                "{component} reads control {ctrl}, but the model has {nu} controls"
            ),
            Self::AlreadyInstalled => {
                write!(f, "the model already has a passive callback installed")
            }
            Self::UnsupportedIntegrator { component, reason } => {
                write!(
                    f,
                    "{component} does not support the model's integrator: {reason}"
                )
            }
        }
    }
}

impl std::error::Error for ThermostatError {}
