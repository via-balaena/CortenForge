//! Plugin/extension system (§66).
//!
//! MuJoCo supports 4 plugin types: actuator, sensor, passive force, and SDF.
//! Plugins are registered by name before model compilation. The builder resolves
//! MJCF `<plugin>` references against a [`PluginRegistry`] and stores trait
//! objects in [`Model`] for runtime dispatch.
//!
//! # Architecture
//!
//! - **Registration**: [`PluginRegistry`] maps plugin names to `Arc<dyn Plugin>`.
//!   Passed to the builder at model construction time (no global state).
//! - **Dispatch**: Forward pipeline calls [`Plugin::compute()`] with the active
//!   capability bit, matching MuJoCo's `mjpPlugin::compute(m, d, instance, cap)`.
//! - **State**: Plugin state lives in `Data::plugin_state` as contiguous `f64`
//!   slices. Plugin-managed opaque data lives in `Data::plugin_data`.
//!
//! # MuJoCo Reference
//!
//! - `mjpPlugin` struct: `mjplugin.h:97–156`
//! - Registration: `mjp_registerPlugin()` in `engine_plugin.cc`
//! - Dispatch: `engine_forward.c` (actuator/advance), `engine_passive.c`,
//!   `engine_sensor.c`

use std::collections::HashMap;
use std::sync::Arc;

use crate::types::data::Data;
use crate::types::model::Model;

/// Plugin capability flags (matches `mjtPluginCapabilityBit`).
///
/// A plugin can have multiple capabilities (bitfield OR).
/// The `compute()` callback receives the active capability so
/// it knows which role it's fulfilling on each call.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PluginCapabilityBit {
    /// Actuator plugin — `compute()` called in `mj_fwd_actuation()`.
    Actuator = 1 << 0,
    /// Sensor plugin — `compute()` called at the stage from `need_stage()`.
    Sensor = 1 << 1,
    /// Passive force plugin — `compute()` called in `mj_fwd_passive()`.
    Passive = 1 << 2,
    /// SDF collision plugin — `sdf_distance()`/`sdf_gradient()` called during collision.
    Sdf = 1 << 3,
}

/// Combined plugin capabilities (bitfield).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PluginCapabilities(pub u32);

impl PluginCapabilities {
    /// No capabilities.
    pub const NONE: Self = Self(0);

    /// Check if a specific capability is set.
    #[must_use]
    pub fn contains(self, bit: PluginCapabilityBit) -> bool {
        self.0 & (bit as u32) != 0
    }

    /// Return capabilities with an additional bit set.
    #[must_use]
    pub fn with(self, bit: PluginCapabilityBit) -> Self {
        Self(self.0 | bit as u32)
    }
}

impl std::ops::BitOr<PluginCapabilityBit> for PluginCapabilities {
    type Output = Self;
    fn bitor(self, rhs: PluginCapabilityBit) -> Self {
        Self(self.0 | rhs as u32)
    }
}

impl From<PluginCapabilityBit> for PluginCapabilities {
    fn from(bit: PluginCapabilityBit) -> Self {
        Self(bit as u32)
    }
}

/// Computation stage for sensor plugins (matches `mjtStage`).
///
/// `None` means "no specific stage needed" and is dispatched at the `Pos`
/// stage (earliest available), matching MuJoCo's `needstage == mjSTAGE_NONE`
/// → fires at `mjSTAGE_POS` convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PluginStage {
    /// No specific stage needed — dispatched at Pos stage (earliest).
    None = 0,
    /// Position-dependent computation (default).
    #[default]
    Pos = 1,
    /// Velocity-dependent computation.
    Vel = 2,
    /// Acceleration-dependent computation.
    Acc = 3,
}

/// A simulation plugin (matches `mjpPlugin`).
///
/// Implement this trait to create custom actuators, sensors, passive forces,
/// or SDF collision geometries. Register instances with [`PluginRegistry`]
/// before building a model.
///
/// # Capabilities
///
/// Return the appropriate capability flags from [`Plugin::capabilities()`]. Only
/// methods matching the declared capabilities will be called:
/// - `ACTUATOR`: `actuator_act_dot()` + `compute()` in actuation
/// - `SENSOR`: `compute()` at the stage from `need_stage()`
/// - `PASSIVE`: `compute()` in passive force computation
/// - `SDF`: `sdf_distance()` / `sdf_gradient()` during collision
///
/// # Thread Safety
///
/// Plugins must be `Send + Sync` for multi-threaded simulation support.
pub trait Plugin: Send + Sync {
    /// Globally unique name (e.g., "mujoco.elasticity.cable").
    fn name(&self) -> &str;

    /// Plugin capabilities (bitfield of [`PluginCapabilityBit`]).
    fn capabilities(&self) -> PluginCapabilities;

    /// Configuration attributes this plugin accepts.
    /// Used for validation during model compilation.
    fn attributes(&self) -> &[&str] {
        &[]
    }

    /// Sensor computation stage (only relevant for SENSOR plugins).
    fn need_stage(&self) -> PluginStage {
        PluginStage::Acc
    }

    /// Number of `f64` state values for this instance.
    /// Called during model compilation to allocate `plugin_state`.
    fn nstate(&self, _model: &Model, _instance: usize) -> usize {
        0
    }

    /// Dimension of sensor output (SENSOR plugins only).
    /// Called during model compilation to set `sensor_dim`.
    fn nsensordata(&self, _model: &Model, _instance: usize, _sensor_id: usize) -> usize {
        0
    }

    /// Initialize plugin data when Data is created.
    /// Called from `make_data()`. Return Err to abort.
    ///
    /// # Errors
    /// Returns an error string if plugin initialization fails.
    fn init(&self, _model: &Model, _data: &mut Data, _instance: usize) -> Result<(), String> {
        Ok(())
    }

    /// Reset plugin state to initial values.
    fn reset(&self, _model: &Model, _state: &mut [f64], _instance: usize) {}

    /// Main computation callback.
    /// `capability` indicates which role is being computed (ACTUATOR/SENSOR/PASSIVE/SDF).
    fn compute(
        &self,
        _model: &Model,
        _data: &mut Data,
        _instance: usize,
        _capability: PluginCapabilityBit,
    ) {
    }

    /// Time integration of plugin state (optional).
    /// Called after position/velocity/activation integration.
    fn advance(&self, _model: &Model, _data: &mut Data, _instance: usize) {}

    /// Actuator activation derivative (ACTUATOR plugins, optional).
    /// Called before `compute()` during actuation.
    fn actuator_act_dot(&self, _model: &Model, _data: &mut Data, _instance: usize) {}

    /// SDF signed distance (SDF plugins).
    fn sdf_distance(&self, _point: &[f64; 3], _data: &Data, _instance: usize) -> f64 {
        0.0
    }

    /// SDF gradient of distance (SDF plugins).
    fn sdf_gradient(
        &self,
        _gradient: &mut [f64; 3],
        _point: &[f64; 3],
        _data: &Data,
        _instance: usize,
    ) {
    }
}

// Arc<dyn Plugin> needs Debug for PluginRegistry derive
impl std::fmt::Debug for dyn Plugin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Plugin(\"{}\")", self.name())
    }
}

/// Registry for plugin type registration.
///
/// Plugins are registered by name before model compilation.
/// The builder resolves MJCF `<plugin>` references against this registry.
#[derive(Debug, Default, Clone)]
pub struct PluginRegistry {
    plugins: HashMap<String, Arc<dyn Plugin>>,
}

impl PluginRegistry {
    /// Create an empty registry.
    #[must_use]
    pub fn new() -> Self {
        Self {
            plugins: HashMap::new(),
        }
    }

    /// Register a plugin.
    ///
    /// # Panics
    /// Panics if a plugin with the same name is already registered.
    pub fn register(&mut self, plugin: Arc<dyn Plugin>) {
        let name = plugin.name().to_string();
        assert!(
            !self.plugins.contains_key(&name),
            "plugin already registered: {name}"
        );
        self.plugins.insert(name, plugin);
    }

    /// Look up a plugin by name.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Arc<dyn Plugin>> {
        self.plugins.get(name)
    }

    /// Number of registered plugin types.
    #[must_use]
    pub fn count(&self) -> usize {
        self.plugins.len()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// Minimal test plugin that tracks calls via atomic counters.
    struct TestPlugin {
        plugin_name: String,
        caps: PluginCapabilities,
        stage: PluginStage,
        state_count: usize,
        compute_count: Arc<AtomicU32>,
        advance_count: Arc<AtomicU32>,
        capability_bits_seen: Arc<AtomicU32>,
    }

    impl TestPlugin {
        fn new(name: &str, caps: PluginCapabilities) -> Self {
            Self {
                plugin_name: name.to_string(),
                caps,
                stage: PluginStage::Acc,
                state_count: 0,
                compute_count: Arc::new(AtomicU32::new(0)),
                advance_count: Arc::new(AtomicU32::new(0)),
                capability_bits_seen: Arc::new(AtomicU32::new(0)),
            }
        }

        fn with_nstate(mut self, n: usize) -> Self {
            self.state_count = n;
            self
        }

        #[allow(dead_code)]
        fn with_stage(mut self, stage: PluginStage) -> Self {
            self.stage = stage;
            self
        }
    }

    impl Plugin for TestPlugin {
        fn name(&self) -> &str {
            &self.plugin_name
        }

        fn capabilities(&self) -> PluginCapabilities {
            self.caps
        }

        fn need_stage(&self) -> PluginStage {
            self.stage
        }

        fn nstate(&self, _model: &Model, _instance: usize) -> usize {
            self.state_count
        }

        fn compute(
            &self,
            _model: &Model,
            _data: &mut Data,
            _instance: usize,
            capability: PluginCapabilityBit,
        ) {
            self.compute_count.fetch_add(1, Ordering::Relaxed);
            self.capability_bits_seen
                .fetch_or(capability as u32, Ordering::Relaxed);
        }

        fn advance(&self, _model: &Model, _data: &mut Data, _instance: usize) {
            self.advance_count.fetch_add(1, Ordering::Relaxed);
        }

        #[allow(clippy::cast_precision_loss)]
        fn reset(&self, _model: &Model, state: &mut [f64], _instance: usize) {
            for (i, s) in state.iter_mut().enumerate() {
                *s = (i + 1) as f64;
            }
        }
    }

    // T1: Plugin registry registration and lookup → AC2
    #[test]
    fn t1_registry_register_and_lookup() {
        let mut reg = PluginRegistry::new();
        assert_eq!(reg.count(), 0);

        let plugin = Arc::new(TestPlugin::new(
            "test.actuator",
            PluginCapabilities::from(PluginCapabilityBit::Actuator),
        ));
        reg.register(plugin);

        assert_eq!(reg.count(), 1);
        assert!(reg.get("test.actuator").is_some());
        assert_eq!(reg.get("test.actuator").unwrap().name(), "test.actuator");
        assert!(reg.get("nonexistent").is_none());
    }

    #[test]
    #[should_panic(expected = "plugin already registered")]
    fn t1_registry_duplicate_panics() {
        let mut reg = PluginRegistry::new();
        let p1 = Arc::new(TestPlugin::new("test.dup", PluginCapabilities::NONE));
        let p2 = Arc::new(TestPlugin::new("test.dup", PluginCapabilities::NONE));
        reg.register(p1);
        reg.register(p2);
    }

    // T6: Data plugin state allocation → AC7
    #[test]
    fn t6_data_plugin_state_allocation() {
        let mut model = Model::empty();
        let plugin =
            Arc::new(TestPlugin::new("test.stateful", PluginCapabilities::NONE).with_nstate(5));

        // Manually set up plugin fields (simulates what the builder does)
        model.nplugin = 1;
        model
            .plugin_objects
            .push(Arc::clone(&plugin) as Arc<dyn Plugin>);
        model.plugin_capabilities.push(plugin.capabilities());
        model.plugin_needstage.push(plugin.need_stage());
        model.plugin_stateadr.push(0);
        model.plugin_statenum.push(5);
        model.plugin_name.push(Some("test".to_string()));
        model.plugin_attradr.push(0);
        model.plugin_attrnum.push(0);
        model.npluginstate = 5;

        let data = model.make_data();
        assert_eq!(data.plugin_state.len(), 5);
        assert_eq!(model.plugin_stateadr[0], 0);
        assert_eq!(model.plugin_statenum[0], 5);
    }

    // T16: Plugin reset restores state → AC15
    #[test]
    fn t16_plugin_reset_restores_state() {
        let mut model = Model::empty();
        let plugin =
            Arc::new(TestPlugin::new("test.resetable", PluginCapabilities::NONE).with_nstate(2));

        model.nplugin = 1;
        model
            .plugin_objects
            .push(Arc::clone(&plugin) as Arc<dyn Plugin>);
        model.plugin_capabilities.push(plugin.capabilities());
        model.plugin_needstage.push(plugin.need_stage());
        model.plugin_stateadr.push(0);
        model.plugin_statenum.push(2);
        model.plugin_name.push(Some("test".to_string()));
        model.plugin_attradr.push(0);
        model.plugin_attrnum.push(0);
        model.npluginstate = 2;

        let mut data = model.make_data();

        // After init, reset sets state to [1.0, 2.0] (TestPlugin::reset behavior)
        // Verify initial reset happened via make_data -> init (TestPlugin::init is no-op,
        // but reset is called in Data::reset)
        data.plugin_state[0] = 99.0;
        data.plugin_state[1] = 99.0;

        // Call reset
        data.reset(&model);

        assert_eq!(data.plugin_state[0], 1.0);
        assert_eq!(data.plugin_state[1], 2.0);
    }

    // T1 supplement: capabilities bitfield operations
    #[test]
    fn capabilities_bitfield() {
        let caps = PluginCapabilities::NONE
            .with(PluginCapabilityBit::Actuator)
            .with(PluginCapabilityBit::Passive);

        assert!(caps.contains(PluginCapabilityBit::Actuator));
        assert!(caps.contains(PluginCapabilityBit::Passive));
        assert!(!caps.contains(PluginCapabilityBit::Sensor));
        assert!(!caps.contains(PluginCapabilityBit::Sdf));
    }

    // T1 supplement: PluginCapabilities BitOr and From
    #[test]
    fn capabilities_bitor_and_from() {
        let caps: PluginCapabilities = PluginCapabilityBit::Sensor.into();
        assert!(caps.contains(PluginCapabilityBit::Sensor));

        let combined = caps | PluginCapabilityBit::Sdf;
        assert!(combined.contains(PluginCapabilityBit::Sensor));
        assert!(combined.contains(PluginCapabilityBit::Sdf));
    }

    // T13: Plugin with empty config (supplementary) — verified via state allocation
    #[test]
    fn t13_plugin_with_empty_config() {
        let mut model = Model::empty();
        let plugin = Arc::new(TestPlugin::new("test.noconfig", PluginCapabilities::NONE));

        model.nplugin = 1;
        model
            .plugin_objects
            .push(Arc::clone(&plugin) as Arc<dyn Plugin>);
        model.plugin_capabilities.push(plugin.capabilities());
        model.plugin_needstage.push(plugin.need_stage());
        model.plugin_stateadr.push(0);
        model.plugin_statenum.push(0);
        model.plugin_name.push(None);
        model.plugin_attradr.push(0);
        model.plugin_attrnum.push(0);
        model.npluginstate = 0;

        let data = model.make_data();
        assert_eq!(data.plugin_state.len(), 0);
        assert_eq!(data.plugin_data.len(), 1);
    }

    // T14: Multiple plugin instances — state address computation
    #[test]
    fn t14_multiple_instances_state_addresses() {
        let mut model = Model::empty();
        let p1 = Arc::new(TestPlugin::new("test.a", PluginCapabilities::NONE).with_nstate(3));
        let p2 = Arc::new(TestPlugin::new("test.b", PluginCapabilities::NONE).with_nstate(2));
        let p3 = Arc::new(TestPlugin::new("test.c", PluginCapabilities::NONE).with_nstate(4));

        model.nplugin = 3;
        for (plugin, nstate, adr) in [
            (Arc::clone(&p1) as Arc<dyn Plugin>, 3usize, 0usize),
            (Arc::clone(&p2) as Arc<dyn Plugin>, 2, 3),
            (Arc::clone(&p3) as Arc<dyn Plugin>, 4, 5),
        ] {
            model.plugin_objects.push(plugin);
            model.plugin_capabilities.push(PluginCapabilities::NONE);
            model.plugin_needstage.push(PluginStage::Acc);
            model.plugin_stateadr.push(adr);
            model.plugin_statenum.push(nstate);
            model.plugin_name.push(None);
            model.plugin_attradr.push(0);
            model.plugin_attrnum.push(0);
        }
        model.npluginstate = 9; // 3 + 2 + 4

        let data = model.make_data();
        assert_eq!(data.plugin_state.len(), 9);
        assert_eq!(data.plugin_data.len(), 3);
        assert_eq!(model.plugin_stateadr[0], 0);
        assert_eq!(model.plugin_stateadr[1], 3);
        assert_eq!(model.plugin_stateadr[2], 5);
    }
}
