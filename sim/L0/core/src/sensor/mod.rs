//! Sensor evaluation pipeline — position, velocity, and acceleration stages.
//!
//! Corresponds to MuJoCo's `engine_sensor.c`. Each stage evaluates sensors
//! that depend on the corresponding pipeline output: `mj_sensor_pos` after FK,
//! `mj_sensor_vel` after velocity kinematics, `mj_sensor_acc` after constraint
//! solve. `mj_sensor_postprocess` applies cutoff clamping.

use crate::plugin::{PluginCapabilityBit, PluginStage};
use crate::types::{Data, MjObjectType, MjSensorType, Model};

use self::postprocess::apply_sensor_cutoff;

pub mod acceleration;
pub(crate) mod geom_distance;
pub(crate) mod position;
pub(crate) mod postprocess;
pub(crate) mod velocity;

// Re-exports added as each sub-module is populated:
pub use acceleration::mj_sensor_acc;
pub(crate) use position::mj_sensor_pos;
pub(crate) use postprocess::mj_sensor_postprocess;
pub(crate) use velocity::mj_sensor_vel;

/// Dispatch plugin sensor computation at the given stage.
///
/// MuJoCo: `compute_plugin_sensors()` in `engine_sensor.c`.
/// Called at the end of each sensor stage function (pos/vel/acc).
pub(crate) fn compute_plugin_sensors(model: &Model, data: &mut Data, stage: PluginStage) {
    if model.nplugin == 0 {
        return;
    }
    for i in 0..model.nplugin {
        if !model.plugin_capabilities[i].contains(PluginCapabilityBit::Sensor) {
            continue;
        }
        // Stage matching: needstage == stage, OR stage is Pos and needstage is None
        let matches = model.plugin_needstage[i] == stage
            || (stage == PluginStage::Pos && model.plugin_needstage[i] == PluginStage::None);
        if !matches {
            continue;
        }
        model.plugin_objects[i].compute(model, data, i, PluginCapabilityBit::Sensor);

        // Apply cutoff to sensors attached to this plugin at this stage.
        // MuJoCo checks sensor_needstage[j] == stage; we derive the sensor's
        // stage from its plugin's needstage (equivalent for Plugin sensors).
        for j in 0..model.nsensor {
            if model.sensor_type[j] == MjSensorType::Plugin && model.sensor_plugin[j] == Some(i) {
                apply_sensor_cutoff(model, data, j);
            }
        }
    }
}

/// Map a sensor to the body it is attached to (if any).
///
/// Returns `None` for multi-body sensors (tendon, actuator) or world-relative
/// sensors, which do not have a single owning body for sleep filtering.
pub(crate) fn sensor_body_id(model: &Model, sensor_id: usize) -> Option<usize> {
    let objid = model.sensor_objid[sensor_id];
    match model.sensor_objtype[sensor_id] {
        MjObjectType::Body | MjObjectType::XBody => Some(objid),
        MjObjectType::Joint => {
            if objid < model.njnt {
                Some(model.jnt_body[objid])
            } else {
                None
            }
        }
        MjObjectType::Geom => {
            if objid < model.ngeom {
                Some(model.geom_body[objid])
            } else {
                None
            }
        }
        MjObjectType::Site => {
            if objid < model.nsite {
                Some(model.site_body[objid])
            } else {
                None
            }
        }
        // Multi-body, actuated, plugin, or world-relative sensors — always compute
        MjObjectType::Tendon
        | MjObjectType::Actuator
        | MjObjectType::Plugin
        | MjObjectType::None => None,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod sensor_tests;
