#!/usr/bin/env python3
"""Self-test: extract_mjcf.py's doc-comment rules against rustdoc's own results.

    python3 sim/L0/tests/scripts/test_extract_mjcf.py

The expected values were measured with rustdoc 1.96.0. Each case below was the doc
comment of an item in a crate whose doctests wrote their MJCF string to a file
(`cargo test --doc`): a case's expected doc is what rustdoc's doctest held. Whether
a fence info string makes a doctest is what `rustdoc --test --test-args --list`
listed. A case this extractor does not model expects `doc-unmodelled` (one of
them, a fence indented four spaces after a paragraph, rustdoc reads as prose:
the rule is conservative). `Drift` runs `drift` in a throwaway git repository
(needs git and cargo). Set EXTRACT_MJCF to a path to test another copy of the
script.
"""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.environ.get("EXTRACT_MJCF", os.path.join(HERE, "extract_mjcf.py"))
_spec = importlib.util.spec_from_file_location("extract_mjcf", SCRIPT)
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
    ("a fence's body loses the fence's indentation", '''
/// K
/// - item
///   ```
///   let x = r#"<mujoco model="K">
///     <worldbody/>
///   </mujoco>"#;
///   ```
pub fn k() {}
''', [("doc-literal", ['<mujoco model="K">\n  <worldbody/>\n</mujoco>'])]),
    ("a line starting ## keeps one #", '''
/// ```
/// let x = r#"<mujoco model="HH">
/// ## comment
/// </mujoco>"#;
/// ```
pub fn hh() {}
''', [("doc-literal", ['<mujoco model="HH">\n# comment\n</mujoco>'])]),
    ("a backtick in a backtick fence's info string: not a fence", '''
/// BT
/// ```a`b
/// let x = r#"<mujoco model="BT"/>"#;
/// ```
pub fn bt() {}
''', [("doc-comment", [])]),
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
    ("three stars are a plain comment", '''
/*** <mujoco model="SS"/> */
pub fn ss() {}
''', [("comment", [])]),
    ("a fence indented four spaces is not a fence", '''
/// X
///     ```
///     let x = r#"<mujoco model="X"/>"#;
///     ```
pub fn xx() {}
''', [("doc-unmodelled", [])]),
    ("a doctest that does not lex", '''
/// ```compile_fail
/// let x = r#"<mujoco model="UL"/>;
/// ```
pub fn ul() {}
''', [("doc-unmodelled", [])]),
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


class Drift(unittest.TestCase):
    def drift(self, lib_rs):
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, "src"))
            os.makedirs(os.path.join(d, "sim", "L0", "tests", "assets", "census", "docs"))
            with open(os.path.join(d, "Cargo.toml"), "w") as f:
                f.write('[package]\nname = "t"\nversion = "0.0.0"\nedition = "2021"\n')
            with open(os.path.join(d, "src", "lib.rs"), "w") as f:
                f.write(lib_rs)
            subprocess.run(["git", "init", "-q"], cwd=d, check=True)
            subprocess.run(["git", "add", "-A"], cwd=d, check=True)
            return subprocess.run([sys.executable, "-I", SCRIPT, "drift"], cwd=d, capture_output=True,
                                  text=True, timeout=120)

    def test_fails_on_doc_comment_mjcf_it_does_not_model(self):
        r = self.drift('/**\n```\nlet x = r#"<mujoco model=\\"N\\"/>"#;\n```\n*/\npub fn n() {}\n')
        self.assertEqual(r.returncode, 1, r.stderr)
        self.assertIn("src/lib.rs:1: MJCF in a doc comment this does not model", r.stderr)

    def test_passes_without_it(self):
        r = self.drift("/// No MJCF here.\npub fn n() {}\n")
        self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main()
