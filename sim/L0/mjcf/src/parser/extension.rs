//! `<extension>` / `<plugin>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    MjcfExtension, MjcfExtensionPlugin, MjcfPluginConfig, MjcfPluginInstance, MjcfPluginRef,
};

use super::attrs::{Attrs, skip_element};

// ============================================================================
// §66: Plugin/Extension Parsing
// ============================================================================

/// Parse `<extension>` element containing plugin declarations (`empty`: it
/// closed itself, so it declares none).
pub(super) fn parse_extension<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<MjcfExtension> {
    let mut ext = MjcfExtension::default();
    if empty {
        return Ok(ext);
    }
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"plugin" {
                    let plugin_name = Attrs::new(e)?
                        .string("plugin")
                        .ok_or_else(|| MjcfError::missing_attribute("plugin", "plugin"))?;
                    let mut ep = MjcfExtensionPlugin {
                        plugin: plugin_name,
                        instances: Vec::new(),
                    };
                    parse_extension_plugin(reader, &mut ep, empty)?;
                    ext.plugins.push(ep);
                } else {
                    skip_element(reader, e.name().as_ref(), empty)?;
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"extension" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("extension"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(ext)
}

/// Parse children of `<plugin>` inside `<extension>` (`empty`: the plugin
/// closed itself, so it has none).
pub(super) fn parse_extension_plugin<R: BufRead>(
    reader: &mut Reader<R>,
    ep: &mut MjcfExtensionPlugin,
    empty: bool,
) -> Result<()> {
    if empty {
        return Ok(());
    }
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"instance" {
                    let name = Attrs::new(e)?
                        .string("name")
                        .ok_or_else(|| MjcfError::missing_attribute("instance", "name"))?;
                    let config = parse_plugin_config(reader, b"instance", empty)?;
                    ep.instances.push(MjcfPluginInstance { name, config });
                } else {
                    skip_element(reader, e.name().as_ref(), empty)?;
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"plugin" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in("plugin"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(())
}

/// Parse `<config key="..." value="..."/>` children inside a parent element
/// (`empty`: the parent closed itself, so it has none).
pub(super) fn parse_plugin_config<R: BufRead>(
    reader: &mut Reader<R>,
    parent_tag: &[u8],
    empty: bool,
) -> Result<Vec<MjcfPluginConfig>> {
    let mut configs = Vec::new();
    let mut seen_keys = std::collections::HashSet::new();
    if empty {
        return Ok(configs);
    }
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"config" {
                    let key = Attrs::new(e)?
                        .string("key")
                        .ok_or_else(|| MjcfError::missing_attribute("config", "key"))?;
                    if !seen_keys.insert(key.clone()) {
                        return Err(MjcfError::invalid_value(
                            "config",
                            "key",
                            format!("duplicate config key: {key}"),
                        ));
                    }
                    let value = Attrs::new(e)?
                        .string("value")
                        .ok_or_else(|| MjcfError::missing_attribute("config", "value"))?;
                    configs.push(MjcfPluginConfig { key, value });
                }
                skip_element(reader, e.name().as_ref(), empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == parent_tag => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in(String::from_utf8_lossy(parent_tag)));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(configs)
}

/// Parse a `<plugin>` sub-element on a body/geom/etc. (`empty`: it closed
/// itself, so it has no `<config>` children).
pub(super) fn parse_plugin_ref<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    empty: bool,
) -> Result<MjcfPluginRef> {
    let plugin = Attrs::new(start)?
        .string("plugin")
        .ok_or_else(|| MjcfError::missing_attribute("plugin", "plugin"))?;
    let instance = Attrs::new(start)?.string("instance");
    let config = parse_plugin_config(reader, b"plugin", empty)?;

    // Validation: can't have both instance reference and inline config
    if instance.is_some() && !config.is_empty() {
        return Err(MjcfError::invalid_model(
            "plugin configuration attributes cannot be used in an element \
             that references a predefined plugin instance",
        ));
    }

    Ok(MjcfPluginRef {
        plugin,
        instance,
        config,
        active: true,
    })
}
