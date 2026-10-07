#!/usr/bin/env python3
"""Self-test: extract_mjcf.py's doc-comment rules against rustdoc's own results.

    python3 sim/L0/tests/scripts/test_extract_mjcf.py

The expected values were measured with rustdoc 1.96.0. Each case below was the doc
comment of an item in a crate whose doctests wrote their MJCF string to a file
(`cargo test --doc`): a case's expected doc is what rustdoc's doctest held. Whether
a fence info string makes a doctest is what `rustdoc --test --test-args --list`
listed. A case rustdoc compiles but this extractor does not model expects
`doc-unmodelled`. Set EXTRACT_MJCF to a path to test another copy of the script.
"""
import importlib.util
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "extract_mjcf", os.environ.get("EXTRACT_MJCF", os.path.join(HERE, "extract_mjcf.py")))
ex = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ex)

# Info strings rustdoc listed as doctests, and the ones it did not.
RUST_TAGS = ["", "rust", "rust,text", "text,rust", "no_run,xml", "should_panic,text", "ignore,text",
             "ignore-wasm32", "rust,ignore", "compile_fail,text", "edition2021", "rust,edition2021",
             "standalone_crate", "test_harness", "{.rust}", "rust {.foo}", "ignore-foo,text"]
OTHER_TAGS = ["text", "xml", "xml,no_run", "text,ignore", "E0001", "edition2021,text", "Rust", "rs",
              "custom", "rust,custom"]

# (name, source, expected records as (kind, docs)); `docs` are the documents a record yields.
CASES = [
    ("plain doctest", '''
/// ```
/// let x = r#"<mujoco model="A">
///   <worldbody/>
/// </mujoco>"#;
/// ```
pub fn a() {}
''', [("doc-literal", ['<mujoco model="A">\n  <worldbody/>\n</mujoco>'])]),
    ("indented hidden line", '''
/// ```
/// let x = r#"<mujoco model="B">
///     # <worldbody/>
/// </mujoco>"#;
/// ```
pub fn b() {}
''', [("doc-literal", ['<mujoco model="B">\n<worldbody/>\n</mujoco>'])]),
    ("hidden line with trailing spaces", '''
/// ```
/// let x = r#"<mujoco model="C">
/// # <worldbody/>\x20\x20\x20
/// </mujoco>"#;
/// ```
pub fn c() {}
''', [("doc-literal", ['<mujoco model="C">\n<worldbody/>\n</mujoco>'])]),
    ("hidden line of a hash and a tab", '''
/// ```
/// let x = r#"<mujoco model="D">
/// #\t
/// </mujoco>"#;
/// ```
pub fn d() {}
''', [("doc-literal", ['<mujoco model="D">\n\n</mujoco>'])]),
    ("a hash and a tab before text is not hidden", '''
/// ```
/// let x = r#"<mujoco model="Y">
/// #\t<worldbody/>
/// </mujoco>"#;
/// ```
pub fn y() {}
''', [("doc-literal", ['<mujoco model="Y">\n#\t<worldbody/>\n</mujoco>'])]),
    ("a Rust tag beside an unknown one", '''
/// ```rust,text
/// let x = "<mujoco model=\\"E\\"/>";
/// ```
pub fn e() {}
''', [("doc-literal", ['<mujoco model="E"/>'])]),
    ("tilde fence", '''
/// ~~~
/// let x = r#"<mujoco model="P"/>"#;
/// ~~~
pub fn p() {}
''', [("doc-literal", ['<mujoco model="P"/>'])]),
    ("fence in a list item", '''
/// Q
/// - item
///   ```
///   let x = r#"<mujoco model="Q"/>"#;
///   ```
pub fn q() {}
''', [("doc-literal", ['<mujoco model="Q"/>'])]),
    ("indented doc comment", '''
///    U
///    ```
///    let x = r#"<mujoco model="U">
///      <worldbody/>
///    </mujoco>"#;
///    ```
pub fn u() {}
''', [("doc-literal", ['<mujoco model="U">\n  <worldbody/>\n</mujoco>'])]),
    ("blank line inside one item's doc", '''
/// ```
/// let x = r#"<mujoco model="R">

/// </mujoco>"#;
/// ```
pub fn r() {}
''', [("doc-literal", ['<mujoco model="R">\n</mujoco>'])]),
    ("plain comment inside one item's doc", '''
/// ```
/// let x = r#"<mujoco model="S">
// an ordinary comment
/// </mujoco>"#;
/// ```
pub fn s() {}
''', [("doc-literal", ['<mujoco model="S">\n</mujoco>'])]),
    ("ignore-target tag", '''
/// ```ignore-wasm32
/// let x = r#"<mujoco model="T"/>"#;
/// ```
pub fn t() {}
''', [("doc-literal", ['<mujoco model="T"/>'])]),
    ("a fence line with an info string does not close", '''
/// ```
/// let x = r#"<mujoco model="V">
/// ```xml
/// </mujoco>"#;
/// ```
pub fn v() {}
''', [("doc-literal", ['<mujoco model="V">\n```xml\n</mujoco>'])]),
    ("MJCF in a doctest's comment", '''
/// ```
/// // the root element is <mujoco>
/// let _a = 1;
/// ```
pub fn i() {}
''', [("doc-comment", [])]),
    ("MJCF named in prose", '''
/// - `<mujoco model="...">` - Root element
pub fn x() {}
''', [("doc-comment", [])]),
    ("four slashes are a plain comment", '''
//// <mujoco model="W"/>
pub fn w() {}
''', [("comment", [])]),
    ("indented code block", '''
/// G
///
///     let x = r#"<mujoco model="G"/>"#;
pub fn g() {}
''', [("doc-unmodelled", [])]),
    ("fence in a block quote", '''
/// H
/// > ```
/// > let x = r#"<mujoco model="H"/>"#;
/// > ```
pub fn h() {}
''', [("doc-unmodelled", [])]),
    ("doc attribute", '''
#[doc = "M\\n```\\nlet x = r#\\"<mujoco model=\\"M\\"/>\\"#;\\n```"]
pub fn m() {}
''', [("doc-unmodelled", [])]),
    ("block doc comment", '''
/**
```
let x = r#"<mujoco model="N"/>"#;
```
*/
pub fn n() {}
''', [("doc-unmodelled", [])]),
]


def records(src):
    return [(r["kind"], ex.doc_spans(r["text"]) if r.get("text") is not None else [])
            for r in ex.rs_records(src)]


class FenceTags(unittest.TestCase):
    def test_rust_tags(self):
        for info in RUST_TAGS:
            with self.subTest(info=info):
                self.assertTrue(ex.is_rust_fence(info))

    def test_other_tags(self):
        for info in OTHER_TAGS:
            with self.subTest(info=info):
                self.assertFalse(ex.is_rust_fence(info))


class DocComments(unittest.TestCase):
    def test_cases(self):
        for name, src, want in CASES:
            with self.subTest(case=name):
                self.assertEqual(records(src), want)


if __name__ == "__main__":
    unittest.main()
