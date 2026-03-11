//! Plugin resolution for model compilation (§66).
//!
//! Resolves MJCF `<extension>`/`<plugin>` references against a
//! [`PluginRegistry`] and populates Model plugin fields.

use std::collections::HashMap;
use std::sync::Arc;

use sim_core::Model;
use sim_core::plugin::{Plugin, PluginRegistry};

use crate::types::{MjcfModel, MjcfPluginRef};

/// Resolve all MJCF plugin references and populate Model plugin fields.
///
/// 1. Walk `<extension>` declarations → create named instances
/// 2. Walk per-element plugin refs (body, geom, actuator, sensor) → create anonymous instances
/// 3. Populate `model.plugin_*` arrays
#[allow(dead_code)]
pub fn resolve_and_assign_plugins(
    mjcf: &MjcfModel,
    model: &mut Model,
    registry: &PluginRegistry,
) -> Result<(), String> {
    // 1. Resolve named instances from <extension> declarations
    let mut plugins: Vec<Arc<dyn Plugin>> = Vec::new();
    let mut configs: Vec<HashMap<String, String>> = Vec::new();
    let mut names: Vec<Option<String>> = Vec::new();
    let mut instance_map: HashMap<String, usize> = HashMap::new();

    for ext in &mjcf.extensions {
        for ep in &ext.plugins {
            let plugin = registry
                .get(&ep.plugin)
                .ok_or_else(|| format!("unknown plugin: {}", ep.plugin))?;
            for inst in &ep.instances {
                if instance_map.contains_key(&inst.name) {
                    return Err(format!("duplicate plugin instance name: {}", inst.name));
                }
                let config: HashMap<String, String> = inst
                    .config
                    .iter()
                    .map(|c| (c.key.clone(), c.value.clone()))
                    .collect();
                let idx = plugins.len();
                plugins.push(Arc::clone(plugin));
                configs.push(config);
                names.push(Some(inst.name.clone()));
                instance_map.insert(inst.name.clone(), idx);
            }
        }
    }

    // Helper: resolve a plugin ref, creating anonymous instance if inline
    let mut resolve_ref = |pref: &MjcfPluginRef| -> Result<usize, String> {
        if let Some(ref inst_name) = pref.instance {
            instance_map
                .get(inst_name)
                .copied()
                .ok_or_else(|| format!("unknown plugin instance: {inst_name}"))
        } else {
            let plugin = registry
                .get(&pref.plugin)
                .ok_or_else(|| format!("unknown plugin: {}", pref.plugin))?;
            let config = pref
                .config
                .iter()
                .map(|c| (c.key.clone(), c.value.clone()))
                .collect();
            let idx = plugins.len();
            plugins.push(Arc::clone(plugin));
            configs.push(config);
            names.push(None);
            Ok(idx)
        }
    };

    // Assign per-body plugin IDs (passive force plugins)
    assign_body_plugins(&mjcf.worldbody, model, &mut resolve_ref)?;

    // Assign per-actuator plugin IDs
    for (i, act) in mjcf.actuators.iter().enumerate() {
        if let Some(ref pref) = act.plugin {
            let idx = resolve_ref(pref)?;
            if i < model.actuator_plugin.len() {
                model.actuator_plugin[i] = Some(idx);
            }
        }
    }

    // Assign per-sensor plugin IDs
    for (i, sen) in mjcf.sensors.iter().enumerate() {
        if let Some(ref pref) = sen.plugin {
            let idx = resolve_ref(pref)?;
            if i < model.sensor_plugin.len() {
                model.sensor_plugin[i] = Some(idx);
            }
        }
    }

    // Populate model plugin arrays
    model.nplugin = plugins.len();
    let mut stateadr = 0;
    for (i, plugin) in plugins.iter().enumerate() {
        let nstate = plugin.nstate(model, i);
        model.plugin_objects.push(Arc::clone(plugin));
        model.plugin_capabilities.push(plugin.capabilities());
        model.plugin_needstage.push(plugin.need_stage());
        model.plugin_stateadr.push(stateadr);
        model.plugin_statenum.push(nstate);
        model.plugin_name.push(names[i].clone());

        // Flatten config attributes
        let attr_adr = model.plugin_attr.len();
        model.plugin_attradr.push(attr_adr);
        let attr_count = configs[i].len();
        for (k, v) in &configs[i] {
            model.plugin_attr.push(format!("{k}={v}"));
        }
        model.plugin_attrnum.push(attr_count);

        stateadr += nstate;
    }
    model.npluginstate = stateadr;

    // For sensor plugins, query nsensordata to set sensor dimension
    for i in 0..model.nsensor {
        if let Some(plugin_id) = model.sensor_plugin[i] {
            let dim = model.plugin_objects[plugin_id].nsensordata(model, plugin_id, i);
            model.sensor_dim[i] = dim;
        }
    }

    Ok(())
}

/// Recursively assign body plugin IDs from the body tree.
#[allow(dead_code)]
fn assign_body_plugins(
    body: &crate::types::MjcfBody,
    model: &mut Model,
    resolve_ref: &mut dyn FnMut(&MjcfPluginRef) -> Result<usize, String>,
) -> Result<(), String> {
    // Note: body_plugin assignment uses body_name_to_id for lookup.
    // The world body (index 0) is named "world" or "" — it can also have plugins.
    if let Some(ref pref) = body.plugin {
        let idx = resolve_ref(pref)?;
        if let Some(&body_id) = model.body_name_to_id.get(&body.name) {
            if body_id < model.body_plugin.len() {
                model.body_plugin[body_id] = Some(idx);
            }
        }
    }

    // Recurse into children
    for child in &body.children {
        assign_body_plugins(child, model, resolve_ref)?;
    }

    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::Arc;

    use sim_core::plugin::{
        Plugin, PluginCapabilities, PluginCapabilityBit, PluginRegistry, PluginStage,
    };
    use sim_core::types::Model;

    use crate::builder::model_from_mjcf_with_plugins;
    use crate::parser::parse_mjcf_str;

    /// Minimal test plugin for builder tests.
    struct BuilderTestPlugin {
        plugin_name: String,
        caps: PluginCapabilities,
        state_count: usize,
    }

    impl BuilderTestPlugin {
        fn new(name: &str, caps: PluginCapabilities) -> Self {
            Self {
                plugin_name: name.to_string(),
                caps,
                state_count: 0,
            }
        }

        #[allow(dead_code)]
        fn with_nstate(mut self, n: usize) -> Self {
            self.state_count = n;
            self
        }
    }

    impl Plugin for BuilderTestPlugin {
        fn name(&self) -> &str {
            &self.plugin_name
        }
        fn capabilities(&self) -> PluginCapabilities {
            self.caps
        }
        fn nstate(&self, _model: &Model, _instance: usize) -> usize {
            self.state_count
        }
        fn need_stage(&self) -> PluginStage {
            PluginStage::Acc
        }
    }

    // T5: Model plugin fields populated after build → AC6
    #[test]
    fn t5_model_plugin_fields_populated() {
        let xml = r#"
            <mujoco model="plugin_build_test">
                <extension>
                    <plugin plugin="test.actuator">
                        <instance name="act1">
                            <config key="gain" value="100"/>
                        </instance>
                    </plugin>
                </extension>
                <worldbody>
                    <body name="b1">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <actuator>
                    <general joint="j1">
                        <plugin plugin="test.actuator" instance="act1"/>
                    </general>
                </actuator>
            </mujoco>
        "#;
        let mjcf = parse_mjcf_str(xml).expect("should parse");
        let mut registry = PluginRegistry::new();
        registry.register(Arc::new(BuilderTestPlugin::new(
            "test.actuator",
            PluginCapabilities::from(PluginCapabilityBit::Actuator),
        )));
        let model = model_from_mjcf_with_plugins(&mjcf, None, &registry)
            .expect("should build with plugins");

        assert_eq!(model.nplugin, 1);
        assert_eq!(model.actuator_plugin[0], Some(0));
        assert!(model.plugin_capabilities[0].contains(PluginCapabilityBit::Actuator));
        assert_eq!(model.plugin_name[0], Some("act1".to_string()));
        assert_eq!(model.plugin_attrnum[0], 1);
        // Config "gain=100" is stored as flattened string
        assert_eq!(model.plugin_attr[model.plugin_attradr[0]], "gain=100");
    }

    // T12: Unknown plugin name error → AC14
    #[test]
    fn t12_unknown_plugin_name_error() {
        let xml = r#"
            <mujoco model="unknown_plugin">
                <extension>
                    <plugin plugin="nonexistent.plugin">
                        <instance name="inst1"/>
                    </plugin>
                </extension>
                <worldbody>
                    <body name="b"/>
                </worldbody>
            </mujoco>
        "#;
        let mjcf = parse_mjcf_str(xml).expect("should parse MJCF");
        let registry = PluginRegistry::new(); // empty
        let result = model_from_mjcf_with_plugins(&mjcf, None, &registry);

        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("unknown plugin"),
            "expected 'unknown plugin' error, got: {err}"
        );
    }
}
