//! The translator against the subset: what it emits, what it refuses, and
//! what naga rejects after it.

#![allow(clippy::unwrap_used, clippy::panic)]

use sim_wgsl_gen::{Error, Source, translate};

fn one(text: &str) -> Result<String, Error> {
    translate(&[Source {
        path: "shared/test.rs",
        text,
    }])
}

/// The refusal's line and message, or a panic naming what came back instead.
fn refusal(text: &str) -> (usize, String) {
    match one(text) {
        Err(Error::Refused { line, message, .. }) => (line, message),
        other => panic!("expected a refusal, got {other:?}"),
    }
}

const SUBSET: &str = r"
/// A parameter block.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Block {
    /// The first value.
    pub first: R,
    pub count: u32,
    pub values: [R; 3],
}

/// The cutoff.
pub const CUTOFF: R = 1e-10;

/// Every construct the subset has.
///
/// A second paragraph.
#[must_use]
#[allow(clippy::many_single_char_names)]
pub const fn everything(a: R, b: R, block: Block, flag: bool) -> [R; 4] {
    let sum = a + b * block.first;
    let [x, _, z] = block.values;
    let [p, q] = pair(a);
    let n: u32 = block.count + 1u32;
    let chosen = if flag { x } else if a > b { z } else { 2.0 };
    let magnitude = (sum.abs().max(CUTOFF)).sqrt().ln();
    let clamped = a.clamp(0.0, 1.0) - -b;
    let as_float = n as R + (a as i32) as R;
    [chosen, magnitude + p * q, clamped, as_float % 2.]
}

fn pair(a: R) -> [R; 2] {
    [a, -a]
}

fn build(v: R) -> Block {
    Block { values: [v, v, v], count: 3, first: v }
}

fn long(first_long_name: R, second_long_name: R) -> [R; 4] {
    [first_long_name * second_long_name, first_long_name + second_long_name, (first_long_name), 0.0]
}
";

#[test]
fn the_subset_translates_and_validates() {
    let wgsl = one(SUBSET).unwrap();
    for expected in [
        "//   shared/test.rs",
        "// A parameter block.\nstruct Block {\n    // The first value.\n    first: f32,\n    count: u32,\n    values: array<f32, 3>,\n}",
        "const CUTOFF: f32 = 1e-10;",
        "// Every construct the subset has.\n//\n// A second paragraph.\nfn everything(a: f32, b: f32, block: Block, flag: bool) -> array<f32, 4> {",
        "    let sum = a + (b * block.first);",
        "    let x = block.values[0];\n    let z = block.values[2];",
        "    let destructured_1 = pair(a);\n    let p = destructured_1[0];\n    let q = destructured_1[1];",
        "    let n: u32 = block.count + 1u;",
        "    let chosen = select(select(2.0, z, a > b), x, flag);",
        "    let magnitude = log(sqrt(max(abs(sum), CUTOFF)));",
        "    let clamped = clamp(a, 0.0, 1.0) - -b;",
        "    let as_float = f32(n) + f32(i32(a));",
        "    return array(chosen, magnitude + (p * q), clamped, as_float % 2.);",
        "    return Block(v, 3, array(v, v, v));",
        "    return array(\n        first_long_name * second_long_name,\n        first_long_name + second_long_name,\n        first_long_name,\n        0.0,\n    );",
    ] {
        assert!(wgsl.contains(expected), "missing {expected:?} in:\n{wgsl}");
    }
    assert!(!wgsl.contains(" _ "), "a wildcard was bound:\n{wgsl}");
}

/// Each source leaves the subset once; the refusal must say so.
const REFUSALS: &[(&str, &str)] = &[
    ("fn f(a: R) -> R {\n    let mut b = a;\n    b\n}", "`mut`"),
    (
        "fn f(a: R) -> R {\n    for _ in 0..3 {}\n    a\n}",
        "only `let` bindings",
    ),
    (
        "fn f(a: R) -> R {\n    let b = loop { break a; };\n    b\n}",
        "loops are not in the subset",
    ),
    (
        "fn f(a: R) -> R {\n    let b = { let mut s = a; for i in 0..3 { s = s + a; } s };\n    b\n}",
        "this expression is not in the subset",
    ),
    (
        "fn f(a: [R; 3]) -> R {\n    let s = a.iter().sum();\n    s\n}",
        "method table",
    ),
    ("fn f(a: R) -> R {\n    match a { _ => a }\n}", "`match`"),
    ("fn f(a: R) -> R {\n    return a;\n}", "without `;`"),
    ("fn f(a: R) -> R {\n    (return a)\n}", "`return`"),
    (
        "fn f(a: R) -> R {\n    let b = a;\n    let b = b;\n    b\n}",
        "already bound",
    ),
    (
        "fn f(a: [R; 3], i: u32) -> R {\n    a[i]\n}",
        "integer literal",
    ),
    ("fn f(a: R) -> R {\n    R::max(a, a)\n}", "plain name"),
    ("fn f(a: f64) -> R {\n    a\n}", "write `R`"),
    ("fn f(a: f32) -> R {\n    a\n}", "write `R`"),
    ("fn f<T>(a: T) -> T {\n    a\n}", "generics"),
    (
        "fn f(a: &R) -> R {\n    a\n}",
        "this type is not in the subset",
    ),
    (
        "fn f(a: R) -> R {\n    let c = |x: R| x;\n    a\n}",
        "closures",
    ),
    (
        "fn f(a: R) -> R {\n    let b = a.round();\n    b\n}",
        "`.round()`",
    ),
    (
        "fn f(a: R) -> R {\n    let b = a.max();\n    b\n}",
        "takes 1 argument",
    ),
    (
        "fn f(a: R) -> R {\n    if a > 0.0 { a } else { let b = a; b }\n}",
        "single expression",
    ),
    (
        "fn f(a: R) -> R {\n    if a > 0.0 { a }\n}",
        "needs an `else`",
    ),
    ("fn f(a: R) {\n    let b = a;\n}", "must return a value"),
    ("fn f(a: R) -> R {\n    a;\n}", "without `;`"),
    (
        "fn f(a: R) -> R {\n    let b = [a; 3];\n    a\n}",
        "this expression is not in the subset",
    ),
    (
        "fn f(a: R) -> R {\n    let b = vec![a];\n    a\n}",
        "macros",
    ),
    ("fn f(a: R) -> R {\n    let b = &a;\n    a\n}", "references"),
    (
        "#[cfg(test)]\nfn f(a: R) -> R {\n    a\n}",
        "could make the Rust and the WGSL differ",
    ),
    ("struct S {\n    a: R,\n}", "#[repr(C)]"),
    (
        "#[repr(C, align(16))]\nstruct S {\n    a: R,\n}",
        "only `#[repr(C)]`",
    ),
    ("#[repr(C)]\nstruct S(R);", "named fields"),
    ("use core::f32;", "only `fn`, `struct` and `const`"),
    ("type R = f32;", "only `fn`, `struct` and `const`"),
    ("fn min(a: R) -> R {\n    a\n}", "WGSL builtin"),
    ("fn f(select: R) -> R {\n    select\n}", "WGSL builtin"),
    (
        "fn f(a: R) -> R {\n    let b = 1u8;\n    a\n}",
        "untyped, `u32` or `i32`",
    ),
    (
        "fn f(a: R) -> R {\n    let b = \"text\";\n    a\n}",
        "only numbers and booleans",
    ),
    (
        "#[repr(C)]\nstruct S {\n    a: R,\n}\nfn f(a: R) -> S {\n    S { a }\n}\n#[repr(C)]\nstruct S {\n    a: R,\n}",
        "declared twice",
    ),
    (
        "#[repr(C)]\nstruct S {\n    a: R,\n    b: R,\n}\nfn f(a: R) -> S {\n    S { a }\n}",
        "field `b` of `S` is missing",
    ),
    (
        "fn f(a: R) -> R {\n    let b = a as bool;\n    a\n}",
        "a cast must be to",
    ),
    (
        "fn f(a: R) -> R {\n    let b = *a;\n    a\n}",
        "dereferencing",
    ),
    (
        "fn f(a: R) -> R {\n    let b = R;\n    a\n}",
        "`R` is a type",
    ),
    (
        "fn f(a: [R; 2]) -> R {\n    let [b, ..] = a;\n    b\n}",
        "only a plain name",
    ),
    (
        "fn f(a: (R, R)) -> R {\n    a.0\n}",
        "this type is not in the subset",
    ),
    ("fn f(a: R) -> R {\n    let b;\n    a\n}", "needs a value"),
];

#[test]
fn every_refusal_fires_with_its_line() {
    for (source, expected) in REFUSALS {
        let (line, message) = refusal(source);
        assert!(
            message.contains(expected),
            "for {source:?}: expected {expected:?}, got {message:?}"
        );
        assert!(line >= 1, "for {source:?}: line {line}");
    }
}

#[test]
fn a_refusal_names_the_file_line_and_column() {
    let err =
        one("fn f(a: R) -> R {\n    let s = a;\n    let t = s;\n\n    for i in 0..2 {}\n    t\n}")
            .unwrap_err()
            .to_string();
    assert!(err.starts_with("shared/test.rs:5:5: "), "{err}");
}

#[test]
fn naga_rejects_what_the_subset_cannot_see() {
    // `select` over arrays is valid Rust and inside the subset, but not WGSL.
    let array_select =
        one("fn f(c: bool, a: [R; 2], b: [R; 2]) -> [R; 2] {\n    if c { a } else { b }\n}");
    assert!(
        matches!(
            array_select,
            Err(Error::Invalid {
                stage: "validation" | "parsing",
                ..
            })
        ),
        "{array_select:?}"
    );
    // A WGSL reserved word as a name.
    let reserved = one("fn f(ptr: R) -> R {\n    ptr\n}");
    assert!(
        matches!(reserved, Err(Error::Invalid { .. })),
        "{reserved:?}"
    );
    // A type mismatch the translator does not check.
    let mismatch = one("fn f(a: u32, b: R) -> R {\n    a + b\n}");
    assert!(
        matches!(mismatch, Err(Error::Invalid { .. })),
        "{mismatch:?}"
    );
}

#[test]
fn validate_rejects_invalid_wgsl_and_accepts_valid() {
    assert!(sim_wgsl_gen::validate("fn f() -> f32 { return 1.0; }").is_ok());
    let err = sim_wgsl_gen::validate("fn f() -> f32 { return 1u; }").unwrap_err();
    assert!(
        matches!(
            err,
            Error::Invalid {
                stage: "validation",
                ..
            }
        ),
        "{err:?}"
    );
    let err = sim_wgsl_gen::validate("fn f( -> f32").unwrap_err();
    assert!(
        matches!(
            err,
            Error::Invalid {
                stage: "parsing",
                ..
            }
        ),
        "{err:?}"
    );
}

#[test]
fn a_source_that_is_not_rust_is_refused() {
    let (line, _) = refusal("fn f(a: R) -> R {\n    a +\n}");
    assert_eq!(line, 3);
}

#[test]
fn first_difference_locates_the_first_changed_line() {
    use sim_wgsl_gen::first_difference;
    assert_eq!(first_difference("a\nb\n", "a\nb\n"), None);
    let d = first_difference("a\nb\nc\n", "a\nx\nc\n").unwrap();
    assert!(
        d.starts_with("line 2:") && d.contains("\"b\"") && d.contains("\"x\""),
        "{d}"
    );
    let d = first_difference("a\n", "a\nb\n").unwrap();
    assert!(d.contains("<end of file>"), "{d}");
    let d = first_difference("a\n", "a").unwrap();
    assert!(d.contains("line endings"), "{d}");
}
