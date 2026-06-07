//! A tiny read-only XML DOM, just enough to query the deeply-nested `.osim`.
//!
//! Spike-grade: builds an in-memory tree with `quick-xml` (the same parser the
//! MJCF importer uses), then offers element/attr/text/descendant queries. Not a
//! general XML library — no namespaces, no mixed-content semantics.

use quick_xml::Reader;
use quick_xml::events::Event;

/// One XML element: tag name, attributes, concatenated text, ordered children.
#[derive(Debug, Default, Clone)]
pub struct Node {
    pub name: String,
    pub attrs: Vec<(String, String)>,
    pub text: String,
    pub children: Vec<Node>,
}

impl Node {
    /// Value of attribute `key`, if present.
    pub fn attr(&self, key: &str) -> Option<&str> {
        self.attrs
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }

    /// First direct child named `name`.
    pub fn child(&self, name: &str) -> Option<&Node> {
        self.children.iter().find(|c| c.name == name)
    }

    /// All direct children named `name`, in document order.
    pub fn children_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Node> + 'a {
        self.children.iter().filter(move |c| c.name == name)
    }

    /// First descendant (depth-first, self excluded) named `name`.
    pub fn find(&self, name: &str) -> Option<&Node> {
        for c in &self.children {
            if c.name == name {
                return Some(c);
            }
            if let Some(found) = c.find(name) {
                return Some(found);
            }
        }
        None
    }

    /// First descendant named `name` whose `attr_key` attribute equals `attr_val`.
    pub fn find_with_attr(&self, name: &str, attr_key: &str, attr_val: &str) -> Option<&Node> {
        for c in &self.children {
            if c.name == name && c.attr(attr_key) == Some(attr_val) {
                return Some(c);
            }
            if let Some(found) = c.find_with_attr(name, attr_key, attr_val) {
                return Some(found);
            }
        }
        None
    }

    /// Whitespace-separated floats in this element's text (e.g. `"0 1 0"`).
    pub fn floats(&self) -> Vec<f64> {
        self.text
            .split_whitespace()
            .filter_map(|s| s.parse::<f64>().ok())
            .collect()
    }

    /// This element's text, trimmed (e.g. a `<body>tibia_r</body>` name).
    pub fn text_trim(&self) -> &str {
        self.text.trim()
    }
}

/// Parse an XML document into a tree. The returned node is a synthetic root
/// whose children are the document's top-level elements. Panics on malformed
/// XML (spike-grade — the input is a trusted vendored fixture).
pub fn parse(xml: &str) -> Node {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut stack: Vec<Node> = vec![Node {
        name: "__root__".to_string(),
        ..Default::default()
    }];
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => stack.push(Node {
                name: tag_name(e.name().as_ref()),
                attrs: attrs(e),
                ..Default::default()
            }),
            Ok(Event::Empty(ref e)) => {
                let node = Node {
                    name: tag_name(e.name().as_ref()),
                    attrs: attrs(e),
                    ..Default::default()
                };
                stack.last_mut().unwrap().children.push(node);
            }
            Ok(Event::Text(ref e)) => {
                // .osim text is numbers + identifiers (no XML entities), so a
                // plain decode is sufficient — no unescaping needed. Prepend a
                // space so text fragments split across a comment (`1<!--c-->2`)
                // can't fuse into `12`; callers trim or split on whitespace.
                if let Ok(t) = e.decode() {
                    let node = stack.last_mut().unwrap();
                    node.text.push(' ');
                    node.text.push_str(&t);
                }
            }
            Ok(Event::End(_)) => {
                let node = stack.pop().expect("unbalanced XML end tag");
                stack.last_mut().unwrap().children.push(node);
            }
            Ok(Event::Eof) => break,
            Err(e) => panic!("XML parse error: {e}"),
            _ => {}
        }
        buf.clear();
    }

    stack.pop().expect("missing synthetic root")
}

fn tag_name(raw: &[u8]) -> String {
    String::from_utf8_lossy(raw).into_owned()
}

fn attrs(e: &quick_xml::events::BytesStart) -> Vec<(String, String)> {
    e.attributes()
        .flatten()
        .map(|a| {
            (
                String::from_utf8_lossy(a.key.as_ref()).into_owned(),
                String::from_utf8_lossy(&a.value).into_owned(),
            )
        })
        .collect()
}
