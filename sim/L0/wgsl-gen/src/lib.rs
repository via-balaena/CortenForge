//! # sim-wgsl-gen
//!
//! Translates a **loop-free subset of plain Rust** into WGSL, so the physics a
//! CPU executor runs and the physics a GPU executor runs come from one source.
//!
//! The shared math is written once, against a scalar type alias `R`, and
//! compiled by rustc twice (at `f32` and `f64`, by `include!`ing the same file
//! into two modules). This crate reads the same files with `syn` and writes a
//! WGSL module with `R` as `f32`. [naga](https://docs.rs/naga) must parse and
//! validate the output before anything is returned, so a translation that
//! would not build as a shader never reaches a file.
//!
//! Only the shared per-element and per-node math goes through here.
//! Orchestration (storage indexing, gathers, dispatch shape) stays hand-written
//! per backend, with the whole of each language available. The plan behind
//! the split is `docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` §13–§14.
//!
//! ## The subset
//!
//! **Items:** `fn` (plain or `const`), `#[repr(C)]` structs with named fields,
//! and `const`. Doc comments are carried into the WGSL as `//` comments.
//!
//! **In a function body:** `let` bindings (a name, a typed name, or an array
//! pattern `let [a, b, _] = v;`), then one final expression, which is the
//! function's value.
//!
//! **Expressions:** literals, names, arithmetic, comparison and logic
//! operators, calls, the methods in [`METHODS`], field access, array
//! literals, indexing by an integer literal, struct literals, casts between
//! `R`, `u32` and `i32`, and `if`/`else` as an expression, emitted as WGSL
//! `select`. Both of a `select`'s arms are evaluated, so guard a division or a
//! logarithm by selecting its *operand*, not its result.
//!
//! **Types:** `R`, `u32`, `i32`, `bool`, fixed-size arrays `[T; N]`, and the
//! structs declared in the sources. `f32` and `f64` are refused, and so are
//! float literal suffixes: every float is `R`, or the two instantiations
//! would silently differ.
//!
//! **Literals.** Rust and WGSL type a float literal differently when nothing
//! fixes its type (Rust falls back to `f64`, WGSL to `f32`), and fold
//! arithmetic on literals alone differently (Rust rounds each step at `R`,
//! WGSL folds at higher precision). So the translator tracks each
//! expression's type and refuses: a `let` holding only literals without a
//! type annotation, an operation between two literals, and a cast of a float
//! literal. Any operation between two float literals is refused too, and a
//! cast to `R` must come from `u32` or `i32`.
//!
//! **Attributes** are accepted only on items, struct fields and `let`
//! statements, and only `doc`, `allow`, `expect`, `must_use`, `inline`,
//! `derive` and `repr(C)`. Any other attribute, anywhere, is refused: a `cfg`
//! would remove code from the Rust and not from the WGSL.
//!
//! **Refused, with the file, line and column:** loops, mutation, `self`,
//! `match`, `return`, closures, macros, references, paths with `::`, runtime
//! indices, redeclared names, and names that would capture a WGSL builtin the
//! output calls. A runtime index is storage access, and storage access
//! belongs to the executor, not the shared math.
//!
//! Loops are refused because of the spike that chose this design (plan §13a):
//! one kernel, compiled through rust-gpu, ran at 1.07–1.81× hand-written WGSL
//! when written with loops and at 1.00× when written loop-free; this
//! translator's loop-free output also ran at 1.00×. Loops in translated WGSL
//! were not measured.

//!
//! ## Using it
//!
//! [`translate`] turns sources into validated WGSL. The package's `wgsl-gen`
//! binary regenerates a committed file (`write`) or reports whether it is stale
//! (`check`); run it from the workspace root, so the source paths recorded in
//! the file's header match. A consumer's freshness test calls [`translate`]
//! on the same sources and compares the result with the committed file
//! ([`first_difference`] says where they part).

#![deny(clippy::unwrap_used, clippy::expect_used)]

mod emit;

use std::path::Path;

pub use emit::METHODS;

/// One Rust source file of shared math.
#[derive(Clone, Copy, Debug)]
pub struct Source<'a> {
    /// The path written into the generated file's header and into refusals.
    /// Use the path relative to the workspace root.
    pub path: &'a str,
    /// The file's text.
    pub text: &'a str,
}

/// Why a translation, a validation or a file operation failed.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The source uses something outside the subset.
    #[error("{path}:{line}:{column}: {message}")]
    Refused {
        /// The source's path, as given.
        path: String,
        /// 1-based line.
        line: usize,
        /// 1-based column.
        column: usize,
        /// What was refused, and why.
        message: String,
    },
    /// The generated WGSL failed naga's parser or validator. The source was
    /// inside the subset, but something the subset cannot see (a type
    /// mismatch, a `select` over an array, a WGSL keyword used as a name) is
    /// wrong.
    #[error("the generated WGSL failed naga {stage}:\n{report}")]
    Invalid {
        /// `parsing` or `validation`.
        stage: &'static str,
        /// naga's report, with the offending WGSL quoted.
        report: String,
    },
    /// A file could not be read or written.
    #[error("{path}: {source}")]
    Io {
        /// The file.
        path: String,
        /// The underlying error.
        #[source]
        source: std::io::Error,
    },
    /// There was nothing to translate.
    #[error("no sources were given")]
    NoSources,
    /// The committed WGSL differs from what the sources generate.
    #[error("{path} is stale ({difference}); regenerate it with `{command}`")]
    Stale {
        /// The committed file.
        path: String,
        /// Where the committed and generated text first differ.
        difference: String,
        /// The command that regenerates it.
        command: String,
    },
}

/// Translate `sources`, in order, into one WGSL module, and validate it.
///
/// # Errors
/// [`Error::Refused`] if a source leaves the subset, and [`Error::Invalid`] if
/// naga rejects the result.
pub fn translate(sources: &[Source<'_>]) -> Result<String, Error> {
    let wgsl = emit::module(sources)?;
    validate(&wgsl)?;
    Ok(wgsl)
}

/// Parse and validate `wgsl` with naga.
///
/// # Errors
/// [`Error::Invalid`] with naga's report.
pub fn validate(wgsl: &str) -> Result<(), Error> {
    let module = naga::front::wgsl::parse_str(wgsl).map_err(|e| Error::Invalid {
        stage: "parsing",
        report: e.emit_to_string(wgsl),
    })?;
    naga::valid::Validator::new(
        naga::valid::ValidationFlags::all(),
        naga::valid::Capabilities::empty(),
    )
    .validate(&module)
    .map_err(|e| Error::Invalid {
        stage: "validation",
        report: e.emit_to_string(wgsl),
    })?;
    Ok(())
}

/// Read `inputs` (paths relative to the current directory) and translate
/// them.
///
/// # Errors
/// [`Error::Io`] if an input cannot be read, and anything [`translate`]
/// returns.
pub fn generate(inputs: &[&str]) -> Result<String, Error> {
    let texts = inputs
        .iter()
        .map(|path| read(path))
        .collect::<Result<Vec<_>, _>>()?;
    let sources: Vec<Source<'_>> = inputs
        .iter()
        .zip(&texts)
        .map(|(path, text)| Source { path, text })
        .collect();
    translate(&sources)
}

/// Regenerate `output` from `inputs`.
///
/// # Errors
/// Anything [`generate`] returns, and [`Error::Io`] if `output` cannot be
/// written.
pub fn write(output: &str, inputs: &[&str]) -> Result<(), Error> {
    let wgsl = generate(inputs)?;
    std::fs::write(Path::new(output), wgsl).map_err(|source| Error::Io {
        path: output.to_string(),
        source,
    })
}

/// Report whether `output` is what `inputs` generate.
///
/// # Errors
/// [`Error::Stale`] if it differs, naming where and the command that fixes
/// it; otherwise anything [`generate`] returns, or [`Error::Io`] if `output`
/// cannot be read.
pub fn check(output: &str, inputs: &[&str]) -> Result<(), Error> {
    let generated = generate(inputs)?;
    let committed = read(output)?;
    first_difference(&committed, &generated).map_or(Ok(()), |difference| {
        Err(Error::Stale {
            path: output.to_string(),
            difference,
            command: regenerate_command(output, inputs),
        })
    })
}

/// The command that regenerates `output` from `inputs`, run from the
/// workspace root.
#[must_use]
pub fn regenerate_command(output: &str, inputs: &[&str]) -> String {
    let mut command = format!("cargo run -p sim-wgsl-gen -- write {output}");
    for input in inputs {
        command.push(' ');
        command.push_str(input);
    }
    command
}

/// Where `committed` and `generated` first differ, as a line number and both
/// lines, or `None` if they are equal.
#[must_use]
pub fn first_difference(committed: &str, generated: &str) -> Option<String> {
    if committed == generated {
        return None;
    }
    let mut committed_lines = committed.lines();
    let mut generated_lines = generated.lines();
    let mut line = 1;
    loop {
        match (committed_lines.next(), generated_lines.next()) {
            (Some(c), Some(g)) if c == g => line += 1,
            (None, None) => {
                return Some(format!(
                    "after line {line}: the texts differ only in line endings"
                ));
            }
            (c, g) => {
                return Some(format!(
                    "line {line}: committed {:?}, generated {:?}",
                    c.unwrap_or("<end of file>"),
                    g.unwrap_or("<end of file>")
                ));
            }
        }
    }
}

fn read(path: &str) -> Result<String, Error> {
    std::fs::read_to_string(path).map_err(|source| Error::Io {
        path: path.to_string(),
        source,
    })
}
