//! MuJoCo 3.5.0's keyword maps (the `mjMap`s of `xml_native_reader.cc`), for
//! the keyword attributes sim-mjcf keeps as text or converts in more than one
//! place. A keyword matches exactly as written: MuJoCo's `MapValue`
//! (`xml_util.cc`) looks the attribute's whole text up, untrimmed and
//! case-sensitive, and refuses any other text, the empty string included, as
//! an invalid keyword. A keyword that is not MuJoCo's is marked an extension
//! where it is matched.
//!
//! The parser checks a keyword when it reads it (`Attrs::keyword`); the
//! builder converts the text it kept through the same function, so a model
//! built in code is held to the same words.

use sim_core::{
    ActuatorDynamics, BiasType, FlexBendingType, FlexSelfCollide, GainType, InterpolationType,
    SleepPolicy,
};

use crate::types::{AngleUnit, CompositeType, FluidShape, InertiaFromGeom, MeshInertia};

/// `<compiler angle>` (`angle_map`).
pub fn angle(s: &str) -> Option<AngleUnit> {
    match s {
        "radian" => Some(AngleUnit::Radian),
        "degree" => Some(AngleUnit::Degree),
        _ => None,
    }
}

/// `<compiler coordinate>` (`coordinate_map`): whether it names `global`.
pub fn coordinate_is_global(s: &str) -> Option<bool> {
    match s {
        "local" => Some(false),
        "global" => Some(true),
        _ => None,
    }
}

/// `<compiler inertiafromgeom>` (`TFAuto_map`).
pub fn inertia_from_geom(s: &str) -> Option<InertiaFromGeom> {
    match s {
        "false" => Some(InertiaFromGeom::False),
        "true" => Some(InertiaFromGeom::True),
        "auto" => Some(InertiaFromGeom::Auto),
        _ => None,
    }
}

/// A geom's `fluidshape` (`fluid_map`).
pub fn fluid_shape(s: &str) -> Option<FluidShape> {
    match s {
        "none" => Some(FluidShape::None),
        "ellipsoid" => Some(FluidShape::Ellipsoid),
        _ => None,
    }
}

/// A mesh's `inertia` (`meshinertia_map`).
pub fn mesh_inertia(s: &str) -> Option<MeshInertia> {
    match s {
        "convex" => Some(MeshInertia::Convex),
        "legacy" => Some(MeshInertia::Legacy),
        "exact" => Some(MeshInertia::Exact),
        "shell" => Some(MeshInertia::Shell),
        _ => None,
    }
}

/// A body's `sleep` (`bodysleep_map`).
pub fn body_sleep(s: &str) -> Option<SleepPolicy> {
    match s {
        "auto" => Some(SleepPolicy::Auto),
        "never" => Some(SleepPolicy::Never),
        "allowed" => Some(SleepPolicy::Allowed),
        "init" => Some(SleepPolicy::Init),
        _ => None,
    }
}

/// A `<general>` actuator's `gaintype` (`gain_map`); `hillmuscle` and
/// `millardmuscle` are extensions.
pub fn gain_type(s: &str) -> Option<GainType> {
    match s {
        "fixed" => Some(GainType::Fixed),
        "affine" => Some(GainType::Affine),
        "muscle" => Some(GainType::Muscle),
        "user" => Some(GainType::User),
        "hillmuscle" => Some(GainType::HillMuscle),
        "millardmuscle" => Some(GainType::MillardMuscle),
        _ => None,
    }
}

/// A `<general>` actuator's `biastype` (`bias_map`); `hillmuscle` and
/// `millardmuscle` are extensions.
pub fn bias_type(s: &str) -> Option<BiasType> {
    match s {
        "none" => Some(BiasType::None),
        "affine" => Some(BiasType::Affine),
        "muscle" => Some(BiasType::Muscle),
        "user" => Some(BiasType::User),
        "hillmuscle" => Some(BiasType::HillMuscle),
        "millardmuscle" => Some(BiasType::MillardMuscle),
        _ => None,
    }
}

/// An actuator's `dyntype` (`dyn_map`); `hillmuscle` and `millardmuscle` are
/// extensions.
pub fn dyn_type(s: &str) -> Option<ActuatorDynamics> {
    match s {
        "none" => Some(ActuatorDynamics::None),
        "integrator" => Some(ActuatorDynamics::Integrator),
        "filter" => Some(ActuatorDynamics::Filter),
        "filterexact" => Some(ActuatorDynamics::FilterExact),
        "muscle" => Some(ActuatorDynamics::Muscle),
        "user" => Some(ActuatorDynamics::User),
        "hillmuscle" => Some(ActuatorDynamics::HillMuscle),
        "millardmuscle" => Some(ActuatorDynamics::MillardMuscle),
        _ => None,
    }
}

/// An actuator's or a sensor's `interp` (`interp_map`).
pub fn interp(s: &str) -> Option<InterpolationType> {
    s.parse().ok()
}

/// A flex's `selfcollide` (`flexself_map`).
pub fn flex_selfcollide(s: &str) -> Option<FlexSelfCollide> {
    match s {
        "none" => Some(FlexSelfCollide::None),
        "narrow" => Some(FlexSelfCollide::Narrow),
        "bvh" => Some(FlexSelfCollide::Bvh),
        "sap" => Some(FlexSelfCollide::Sap),
        "auto" => Some(FlexSelfCollide::Auto),
        _ => None,
    }
}

/// A flex `<elasticity>`'s `bending_model`, an extension.
pub fn bending_model(s: &str) -> Option<FlexBendingType> {
    match s {
        "cotangent" => Some(FlexBendingType::Cotangent),
        "bridson" => Some(FlexBendingType::Bridson),
        _ => None,
    }
}

/// A `<composite>`'s `type` (`comp_map`).
pub fn composite_type(s: &str) -> Option<CompositeType> {
    match s {
        "particle" => Some(CompositeType::Particle),
        "grid" => Some(CompositeType::Grid),
        "rope" => Some(CompositeType::Rope),
        "loop" => Some(CompositeType::Loop),
        "cable" => Some(CompositeType::Cable),
        "cloth" => Some(CompositeType::Cloth),
        _ => None,
    }
}

/// A `<flexcomp>`'s `type` (`fcomp_map`), as its word.
pub fn flexcomp_type(s: &str) -> Option<&'static str> {
    const WORDS: [&str; 10] = [
        "grid",
        "box",
        "cylinder",
        "ellipsoid",
        "square",
        "disc",
        "circle",
        "mesh",
        "gmsh",
        "direct",
    ];
    WORDS.into_iter().find(|word| *word == s)
}
