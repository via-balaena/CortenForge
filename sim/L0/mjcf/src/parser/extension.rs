//! `<extension>` / `<plugin>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    MjcfExtension, MjcfExtensionPlugin, MjcfPluginConfig, MjcfPluginInstance, MjcfPluginRef,
};

use super::attrs::get_attribute_opt;

// ============================================================================
// §66: Plugin/Extension Parsing
// ============================================================================

/// Parse `<extension>` element containing plugin declarations.
pub(super) fn parse_extension<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfExtension> {
    let mut ext = MjcfExtension::default();
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_name =
                    get_attribute_opt(e, "plugin").ok_or_else(|| MjcfError::MissingAttribute {
                        element: "plugin".into(),
                        attribute: "plugin",
                    })?;
                let mut ep = MjcfExtensionPlugin {
                    plugin: plugin_name,
                    instances: Vec::new(),
                };
                parse_extension_plugin(reader, &mut ep)?;
                ext.plugins.push(ep);
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"extension" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in extension".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(ext)
}

/// Parse children of `<plugin>` inside `<extension>`.
pub(super) fn parse_extension_plugin<R: BufRead>(
    reader: &mut Reader<R>,
    ep: &mut MjcfExtensionPlugin,
) -> Result<()> {
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"instance" => {
                let name =
                    get_attribute_opt(e, "name").ok_or_else(|| MjcfError::MissingAttribute {
                        element: "instance".into(),
                        attribute: "name",
                    })?;
                let config = parse_plugin_config(reader, b"instance")?;
                ep.instances.push(MjcfPluginInstance { name, config });
            }
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"instance" => {
                let name =
                    get_attribute_opt(e, "name").ok_or_else(|| MjcfError::MissingAttribute {
                        element: "instance".into(),
                        attribute: "name",
                    })?;
                ep.instances.push(MjcfPluginInstance {
                    name,
                    config: Vec::new(),
                });
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"plugin" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in plugin".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(())
}

/// Parse `<config key="..." value="..."/>` children inside a parent element.
pub(super) fn parse_plugin_config<R: BufRead>(
    reader: &mut Reader<R>,
    parent_tag: &[u8],
) -> Result<Vec<MjcfPluginConfig>> {
    let mut configs = Vec::new();
    let mut seen_keys = std::collections::HashSet::new();
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"config" => {
                let key =
                    get_attribute_opt(e, "key").ok_or_else(|| MjcfError::MissingAttribute {
                        element: "config".into(),
                        attribute: "key",
                    })?;
                if !seen_keys.insert(key.clone()) {
                    return Err(MjcfError::XmlParse(format!("duplicate config key: {key}")));
                }
                let value =
                    get_attribute_opt(e, "value").ok_or_else(|| MjcfError::MissingAttribute {
                        element: "config".into(),
                        attribute: "value",
                    })?;
                configs.push(MjcfPluginConfig { key, value });
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == parent_tag => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse(format!(
                    "unexpected EOF in {}",
                    String::from_utf8_lossy(parent_tag)
                )));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(configs)
}

/// Parse a `<plugin>` sub-element on a body/geom/etc.
pub(super) fn parse_plugin_ref<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfPluginRef> {
    let plugin = get_attribute_opt(start, "plugin").ok_or_else(|| MjcfError::MissingAttribute {
        element: "plugin".into(),
        attribute: "plugin",
    })?;
    let instance = get_attribute_opt(start, "instance");
    let config = parse_plugin_config(reader, b"plugin")?;

    // Validation: can't have both instance reference and inline config
    if instance.is_some() && !config.is_empty() {
        return Err(MjcfError::XmlParse(
            "plugin configuration attributes cannot be used in an element \
             that references a predefined plugin instance"
                .into(),
        ));
    }

    Ok(MjcfPluginRef {
        plugin,
        instance,
        config,
        active: true,
    })
}

/// Parse a self-closing `<plugin plugin="..." instance="..."/>` element (Event::Empty).
/// No child `<config>` elements are possible in a self-closing tag.
pub(super) fn parse_plugin_ref_empty(start: &BytesStart) -> Result<MjcfPluginRef> {
    let plugin = get_attribute_opt(start, "plugin").ok_or_else(|| MjcfError::MissingAttribute {
        element: "plugin".into(),
        attribute: "plugin",
    })?;
    let instance = get_attribute_opt(start, "instance");

    Ok(MjcfPluginRef {
        plugin,
        instance,
        config: Vec::new(),
        active: true,
    })
}
