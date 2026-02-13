//! `<include>` file expansion for MJCF models.
//!
//! Implements pre-parse XML rewriting that expands `<include file="..."/>`
//! elements by replacing them with the children of the included file's root
//! element. This runs before the MJCF parser sees the document.

use std::collections::HashSet;
use std::io::BufRead;
use std::path::{Path, PathBuf};

use quick_xml::Reader;
use quick_xml::Writer;
use quick_xml::events::{BytesStart, Event};

use crate::error::{MjcfError, Result};

/// Expand all `<include file="..."/>` elements in the given XML string.
///
/// Include paths are resolved relative to `base_dir` (the directory containing
/// the main model file). The expansion is recursive: included files may
/// themselves contain `<include>` elements.
///
/// Each file may be included at most once. Duplicate includes produce
/// `MjcfError::DuplicateInclude`.
pub fn expand_includes(xml: &str, base_dir: &Path) -> Result<String> {
    let mut seen = HashSet::new();
    expand_includes_inner(xml, base_dir, &mut seen)
}

fn expand_includes_inner(
    xml: &str,
    base_dir: &Path,
    seen: &mut HashSet<PathBuf>,
) -> Result<String> {
    // Check if there are any includes to expand (fast path)
    if !xml.contains("<include") {
        return Ok(xml.to_string());
    }

    let mut reader = Reader::from_str(xml);
    let mut writer = Writer::new(Vec::new());
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"include" => {
                // Extract the file attribute
                let file_attr = extract_file_attr(e)?;
                let resolved = resolve_include_path(&file_attr, base_dir)?;

                // Check for duplicate includes
                let canonical = resolved.canonicalize().map_err(|e| {
                    MjcfError::IncludeError(format!(
                        "cannot resolve include path '{}': {e}",
                        resolved.display()
                    ))
                })?;
                if !seen.insert(canonical.clone()) {
                    return Err(MjcfError::DuplicateInclude(format!(
                        "file '{}' included more than once",
                        resolved.display()
                    )));
                }

                // Read the included file
                let content = std::fs::read_to_string(&resolved).map_err(|e| {
                    MjcfError::IncludeError(format!(
                        "cannot read include file '{}' (resolved from '{file_attr}'): {e}",
                        resolved.display()
                    ))
                })?;

                // Extract root children and splice them in
                let children_xml = extract_root_children(&content, &resolved)?;

                // Recursively expand any includes in the children
                let expanded = expand_includes_inner(&children_xml, base_dir, seen)?;

                // Write the expanded content as raw XML
                writer
                    .write_event(Event::Text(quick_xml::events::BytesText::from_escaped(
                        &expanded,
                    )))
                    .map_err(|e| MjcfError::XmlParse(format!("write error: {e}")))?;
            }
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"include" => {
                // <include ...> with children is invalid, but handle gracefully
                let file_attr = extract_file_attr(e)?;
                let resolved = resolve_include_path(&file_attr, base_dir)?;

                let canonical = resolved.canonicalize().map_err(|e| {
                    MjcfError::IncludeError(format!(
                        "cannot resolve include path '{}': {e}",
                        resolved.display()
                    ))
                })?;
                if !seen.insert(canonical.clone()) {
                    return Err(MjcfError::DuplicateInclude(format!(
                        "file '{}' included more than once",
                        resolved.display()
                    )));
                }

                let content = std::fs::read_to_string(&resolved).map_err(|e| {
                    MjcfError::IncludeError(format!(
                        "cannot read include file '{}' (resolved from '{file_attr}'): {e}",
                        resolved.display()
                    ))
                })?;

                let children_xml = extract_root_children(&content, &resolved)?;
                let expanded = expand_includes_inner(&children_xml, base_dir, seen)?;

                writer
                    .write_event(Event::Text(quick_xml::events::BytesText::from_escaped(
                        &expanded,
                    )))
                    .map_err(|e| MjcfError::XmlParse(format!("write error: {e}")))?;

                // Skip the closing </include> tag
                skip_include_element(&mut reader)?;
            }
            Ok(Event::Eof) => break,
            Ok(event) => {
                writer
                    .write_event(event)
                    .map_err(|e| MjcfError::XmlParse(format!("write error: {e}")))?;
            }
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    let result = writer.into_inner();
    String::from_utf8(result).map_err(|e| MjcfError::XmlParse(format!("UTF-8 error: {e}")))
}

/// Extract the `file` attribute from an `<include>` element.
fn extract_file_attr(e: &BytesStart) -> Result<String> {
    for attr in e.attributes().flatten() {
        if attr.key.as_ref() == b"file" {
            let val = attr
                .unescape_value()
                .map_err(|e| MjcfError::XmlParse(format!("invalid include file attr: {e}")))?;
            let val = val.to_string();
            if val.is_empty() {
                return Err(MjcfError::IncludeError(
                    "<include> element has empty file attribute".to_string(),
                ));
            }
            return Ok(val);
        }
    }
    Err(MjcfError::IncludeError(
        "<include> element missing required 'file' attribute".to_string(),
    ))
}

/// Resolve an include file path relative to the base directory.
fn resolve_include_path(file: &str, base_dir: &Path) -> Result<PathBuf> {
    let path = Path::new(file);
    if path.is_absolute() {
        Ok(path.to_path_buf())
    } else {
        Ok(base_dir.join(file))
    }
}

/// Extract the XML content of all children of the root element.
///
/// The included file must have exactly one root element (the "wrapper").
/// We extract everything between the root's opening and closing tags.
fn extract_root_children(xml: &str, source_path: &Path) -> Result<String> {
    let mut reader = Reader::from_str(xml);
    let mut buf = Vec::new();

    // Skip to the first start element (the root wrapper)
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(_)) => {
                break;
            }
            Ok(Event::Empty(_)) => {
                // Self-closing root → no children
                return Err(MjcfError::IncludeError(format!(
                    "include file '{}' has empty root element (no children to include)",
                    source_path.display()
                )));
            }
            Ok(Event::Eof) => {
                return Err(MjcfError::IncludeError(format!(
                    "include file '{}' has no root element",
                    source_path.display()
                )));
            }
            Ok(
                Event::Decl(_)
                | Event::Comment(_)
                | Event::Text(_)
                | Event::PI(_)
                | Event::DocType(_)
                | Event::End(_)
                | Event::CData(_),
            ) => {
                // Skip XML declaration, comments, whitespace, processing instructions
            }
            Err(e) => {
                return Err(MjcfError::XmlParse(format!(
                    "error parsing include file '{}': {e}",
                    source_path.display()
                )));
            }
        }
        buf.clear();
    }

    // Now collect everything between root open and root close as raw XML.
    // We need to track depth to know when we hit the matching close tag.
    let mut depth = 1i32;
    let mut children_writer = Writer::new(Vec::new());

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                depth += 1;
                children_writer
                    .write_event(Event::Start(e.clone()))
                    .map_err(|e| MjcfError::XmlParse(format!("write error: {e}")))?;
            }
            Ok(Event::End(ref e)) => {
                depth -= 1;
                if depth == 0 {
                    // This is the root's closing tag — we're done
                    break;
                }
                children_writer
                    .write_event(Event::End(e.clone()))
                    .map_err(|e| MjcfError::XmlParse(format!("write error: {e}")))?;
            }
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse(format!(
                    "unexpected EOF in include file '{}'",
                    source_path.display()
                )));
            }
            Ok(event) => {
                children_writer
                    .write_event(event)
                    .map_err(|e| MjcfError::XmlParse(format!("write error: {e}")))?;
            }
            Err(e) => {
                return Err(MjcfError::XmlParse(format!(
                    "error parsing include file '{}': {e}",
                    source_path.display()
                )));
            }
        }
        buf.clear();
    }

    let bytes = children_writer.into_inner();
    String::from_utf8(bytes).map_err(|e| MjcfError::XmlParse(format!("UTF-8 error: {e}")))
}

/// Skip past a non-self-closing `<include>...</include>` element.
fn skip_include_element<R: BufRead>(reader: &mut Reader<R>) -> Result<()> {
    let mut buf = Vec::new();
    let mut depth = 1i32;
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(_)) => depth += 1,
            Ok(Event::End(_)) => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse(
                    "unexpected EOF while skipping include element".into(),
                ));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use std::fs;

    fn setup_include_files(dir: &Path, files: &[(&str, &str)]) {
        for (name, content) in files {
            let path = dir.join(name);
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent).unwrap();
            }
            fs::write(&path, content).unwrap();
        }
    }

    #[test]
    fn test_no_includes_passthrough() {
        let xml = r"<mujoco><worldbody/></mujoco>";
        let result = expand_includes(xml, Path::new("/tmp")).unwrap();
        assert!(result.contains("mujoco"));
        assert!(result.contains("worldbody"));
    }

    #[test]
    fn test_simple_include() {
        let dir = tempfile::tempdir().unwrap();
        setup_include_files(
            dir.path(),
            &[(
                "bodies.xml",
                r#"<mujoco>
                <body name="arm">
                    <geom type="sphere" size="0.1"/>
                </body>
            </mujoco>"#,
            )],
        );

        let xml = r#"<mujoco>
                <worldbody>
                    <include file="bodies.xml"/>
                </worldbody>
            </mujoco>"#;

        let result = expand_includes(xml, dir.path()).unwrap();
        assert!(
            result.contains("arm"),
            "included body should be present: {result}"
        );
        assert!(
            !result.contains("<include"),
            "include tags should be removed: {result}"
        );
    }

    #[test]
    fn test_nested_includes() {
        let dir = tempfile::tempdir().unwrap();
        setup_include_files(
            dir.path(),
            &[
                (
                    "outer.xml",
                    r#"<wrapper>
                    <body name="outer_body">
                        <geom type="sphere" size="0.1"/>
                    </body>
                    <include file="inner.xml"/>
                </wrapper>"#,
                ),
                (
                    "inner.xml",
                    r#"<wrapper>
                    <body name="inner_body">
                        <geom type="box" size="0.1 0.1 0.1"/>
                    </body>
                </wrapper>"#,
                ),
            ],
        );

        let xml = r#"<mujoco>
                <worldbody>
                    <include file="outer.xml"/>
                </worldbody>
            </mujoco>"#;

        let result = expand_includes(xml, dir.path()).unwrap();
        assert!(
            result.contains("outer_body"),
            "outer body should be present"
        );
        assert!(
            result.contains("inner_body"),
            "nested inner body should be present"
        );
        assert!(
            !result.contains("<include"),
            "all include tags should be removed"
        );
    }

    #[test]
    fn test_duplicate_include_error() {
        let dir = tempfile::tempdir().unwrap();
        setup_include_files(
            dir.path(),
            &[(
                "shared.xml",
                r#"<wrapper>
                <body name="shared">
                    <geom type="sphere" size="0.1"/>
                </body>
            </wrapper>"#,
            )],
        );

        let xml = r#"<mujoco>
                <worldbody>
                    <include file="shared.xml"/>
                    <include file="shared.xml"/>
                </worldbody>
            </mujoco>"#;

        let result = expand_includes(xml, dir.path());
        assert!(result.is_err());
        match result.unwrap_err() {
            MjcfError::DuplicateInclude(msg) => {
                assert!(
                    msg.contains("shared.xml"),
                    "error should mention file: {msg}"
                );
            }
            other => panic!("expected DuplicateInclude, got: {other}"),
        }
    }

    #[test]
    fn test_missing_file_error() {
        let dir = tempfile::tempdir().unwrap();
        let xml = r#"<mujoco>
                <worldbody>
                    <include file="nonexistent.xml"/>
                </worldbody>
            </mujoco>"#;

        let result = expand_includes(xml, dir.path());
        assert!(result.is_err());
        match result.unwrap_err() {
            MjcfError::IncludeError(msg) => {
                assert!(
                    msg.contains("nonexistent.xml"),
                    "error should mention file: {msg}"
                );
            }
            other => panic!("expected IncludeError, got: {other}"),
        }
    }

    #[test]
    fn test_missing_file_attr_error() {
        let dir = tempfile::tempdir().unwrap();
        let xml = r"<mujoco>
                <worldbody>
                    <include/>
                </worldbody>
            </mujoco>";

        let result = expand_includes(xml, dir.path());
        assert!(result.is_err());
        match result.unwrap_err() {
            MjcfError::IncludeError(msg) => {
                assert!(
                    msg.contains("file"),
                    "error should mention missing attr: {msg}"
                );
            }
            other => panic!("expected IncludeError, got: {other}"),
        }
    }

    #[test]
    fn test_include_at_top_level() {
        let dir = tempfile::tempdir().unwrap();
        setup_include_files(
            dir.path(),
            &[(
                "actuators.xml",
                r#"<mujoco>
                <actuator>
                    <motor joint="j1" name="m1"/>
                </actuator>
            </mujoco>"#,
            )],
        );

        let xml = r#"<mujoco>
                <worldbody>
                    <body name="b">
                        <joint name="j1" type="hinge"/>
                        <geom type="sphere" size="0.1"/>
                    </body>
                </worldbody>
                <include file="actuators.xml"/>
            </mujoco>"#;

        let result = expand_includes(xml, dir.path()).unwrap();
        assert!(
            result.contains("actuator"),
            "actuator section should be present"
        );
        assert!(result.contains("m1"), "motor should be present");
    }

    #[test]
    fn test_include_resolves_relative_to_base_dir() {
        let dir = tempfile::tempdir().unwrap();
        setup_include_files(
            dir.path(),
            &[(
                "subdir/part.xml",
                r#"<wrapper>
                <body name="part">
                    <geom type="sphere" size="0.1"/>
                </body>
            </wrapper>"#,
            )],
        );

        let xml = r#"<mujoco>
                <worldbody>
                    <include file="subdir/part.xml"/>
                </worldbody>
            </mujoco>"#;

        let result = expand_includes(xml, dir.path()).unwrap();
        assert!(
            result.contains("part"),
            "body from subdirectory should be present"
        );
    }
}
