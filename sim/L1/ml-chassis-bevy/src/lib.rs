//! Bevy ECS `Resource` adapters for [`sim_ml_chassis`] types.
//!
//! `sim-ml-chassis` is L0 — pure-Rust, no Bevy chain. This crate provides
//! transparent newtype wrappers that derive [`bevy_ecs::prelude::Resource`]
//! so the chassis types can be inserted directly into a Bevy `App`. Each
//! wrapper implements [`Deref`] / [`DerefMut`] to its inner chassis type
//! and [`From<Inner>`](From) for ergonomic `.into()` at the insert site.
//!
//! The orphan rule prevents `impl Resource for sim_ml_chassis::SimEnv`
//! from this crate (neither type is local), so the wrappers are the only
//! way to bridge. Because [`Deref`] / [`DerefMut`] are implemented,
//! method calls on `Res<SimEnv>` / `ResMut<SimEnv>` resolve through to
//! the chassis API unchanged.
//!
//! ```ignore
//! use bevy::prelude::*;
//! use sim_ml_chassis::SimEnv as ChassisSimEnv;
//! use sim_ml_chassis_bevy::SimEnv;
//!
//! let env: ChassisSimEnv = /* build via ChassisSimEnv::builder(...) */;
//! App::new().insert_resource(SimEnv::from(env));
//!
//! fn step_system(mut env: ResMut<SimEnv>) {
//!     // env.step(...), env.data() etc. — auto-deref to chassis SimEnv.
//! }
//! ```

use std::ops::{Deref, DerefMut};

use bevy_ecs::prelude::Resource;

macro_rules! resource_newtype {
    ($Wrapper:ident, $Inner:ty, $doc:expr) => {
        #[doc = $doc]
        #[derive(Resource)]
        pub struct $Wrapper(pub $Inner);

        impl Deref for $Wrapper {
            type Target = $Inner;
            fn deref(&self) -> &Self::Target {
                &self.0
            }
        }

        impl DerefMut for $Wrapper {
            fn deref_mut(&mut self) -> &mut Self::Target {
                &mut self.0
            }
        }

        impl From<$Inner> for $Wrapper {
            fn from(inner: $Inner) -> Self {
                Self(inner)
            }
        }
    };
}

resource_newtype!(
    SimEnv,
    sim_ml_chassis::SimEnv,
    "Bevy `Resource` wrapper for [`sim_ml_chassis::SimEnv`]."
);

resource_newtype!(
    ObservationSpace,
    sim_ml_chassis::ObservationSpace,
    "Bevy `Resource` wrapper for [`sim_ml_chassis::ObservationSpace`]."
);

resource_newtype!(
    ActionSpace,
    sim_ml_chassis::ActionSpace,
    "Bevy `Resource` wrapper for [`sim_ml_chassis::ActionSpace`]."
);

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use std::sync::Arc;

    use bevy_ecs::prelude::Resource;
    use sim_core::test_fixtures;
    use sim_ml_chassis::{
        ActionSpace as ChassisActionSpace, Environment,
        ObservationSpace as ChassisObservationSpace, SimEnv as ChassisSimEnv,
    };

    use super::{ActionSpace, ObservationSpace, SimEnv};

    // Compile-time check: each wrapper satisfies the Resource bound.
    fn assert_resource<T: Resource>() {}

    #[test]
    fn wrappers_implement_resource() {
        assert_resource::<SimEnv>();
        assert_resource::<ObservationSpace>();
        assert_resource::<ActionSpace>();
    }

    fn build_env() -> ChassisSimEnv {
        let model = Arc::new(test_fixtures::pendulums::pendulum_basic());
        let obs = ChassisObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act = ChassisActionSpace::builder()
            .all_ctrl()
            .build(&model)
            .unwrap();
        ChassisSimEnv::builder(model)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .build()
            .unwrap()
    }

    #[test]
    fn sim_env_wrapper_derefs_to_chassis_methods() {
        let inner = build_env();
        let mut wrapped: SimEnv = inner.into();
        // Reset goes through DerefMut → chassis SimEnv::reset.
        let obs = wrapped.reset().unwrap();
        assert_eq!(obs.shape(), wrapped.observation_space().spec().shape);
    }

    #[test]
    fn observation_space_wrapper_derefs_to_chassis_methods() {
        let model = Arc::new(test_fixtures::pendulums::pendulum_basic());
        let inner = ChassisObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let expected_dim = inner.dim();
        let wrapped: ObservationSpace = inner.into();
        assert_eq!(wrapped.dim(), expected_dim);
    }

    #[test]
    fn action_space_wrapper_derefs_to_chassis_methods() {
        let model = Arc::new(test_fixtures::pendulums::pendulum_basic());
        let inner = ChassisActionSpace::builder()
            .all_ctrl()
            .build(&model)
            .unwrap();
        let expected_dim = inner.dim();
        let wrapped: ActionSpace = inner.into();
        assert_eq!(wrapped.dim(), expected_dim);
    }
}
