//! Self-test: the crates.io release set builds no C++, and every other native
//! build step it takes on a desktop target is one we have listed.
//!
//! The SDK's published crates stay Rust: a package that builds C++ is refused
//! outright, and anything else that compiles, links or ships native code needs
//! an entry in [`ALLOWED`] that names exactly what it does. [`LONG_ABOUT`]
//! states the rules and what the signals cannot see.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::LazyLock;

use anyhow::{bail, Context, Result};
use regex::Regex;
use serde_json::Value;

use Signal::{CppRuntime, CppSource, Links, Prebuilt, Tool};

/// Shown by `cargo xtask native-build --help`. The single statement of what this
/// asserts; CI's comment points here rather than restating it.
pub const LONG_ABOUT: &str = "\
Assert that the crates.io release set (the `cortenforge` facade's closure, as
`cargo xtask publish-set` defines it) compiles and ships no C++, and that
every other native build step it takes is listed. Linking a runtime the
operating system ships (libc++ on macOS, the Objective-C runtime) is not
compiling or shipping C++; it is allowed when listed.

What it walks: every package `cargo tree` reaches from the set's crates, with
all their features on, through normal and build dependencies, at the versions
Cargo.lock pins, for each of these targets: x86_64-unknown-linux-gnu,
aarch64-unknown-linux-gnu, x86_64-apple-darwin, aarch64-apple-darwin,
x86_64-pc-windows-msvc, aarch64-pc-windows-msvc, x86_64-pc-windows-gnu.
Build scripts and proc-macros, and what they depend on, are resolved for the
machine running the check, not the target; CI runs it on Linux.

A package counts as native when
  - its manifest has a `links` key;
  - one of its build-dependencies is, or reaches through its own normal
    dependencies, a native build tool (autotools, bindgen, cc, cmake,
    cpp_build, cxx-build, gcc, nasm-rs, pkg-config, system-deps, vcpkg);
  - it ships a native library file (.a .lib .so .so.N .dylib .dll .o .obj)
    and has a build script, which is what can put that file on the link line;
  - its build script (the file its manifest names) names a C++ source (a
    string ending in .cpp .cc .cxx .c++ .mm .cu), passes `-std=c++`, or calls
    `.cpp(true)`; or
  - a Rust file in it links the C++ runtime (c++, stdc++, c++abi or supc++) in
    one of two spellings: `#[link(name = \"...\")]` with the name first, or a
    literal `rustc-link-lib=NAME`, `=dylib=NAME` or `=static=NAME`.

The rules:
  - nothing builds C++: no `autotools`, `cmake`, `cpp_build` or `cxx-build`
    build tool, and no build script naming C++ sources. A listing cannot allow
    these. `autotools` and `cmake` are refused whatever the project's
    language, because this check cannot see inside those projects and every
    user would need the tool installed. An exception for one package is a code
    change here, and needs a project that declares only C, builds without
    downloading anything, and has no pure-Rust alternative of similar quality;
  - every other native package, a C++ runtime link included, is in the
    allowlist with exactly the signals found, so a listed package whose
    signals change fails too;
  - every allowlist entry is still found, so the list cannot go stale.

What the signals cannot see:
  - C++ that a build script compiles through `cc` or `gcc` without naming its
    sources in its own file (sources found by listing a directory, or named in
    another file the script pulls in). It reads as that build tool, which a
    listing allows;
  - native code a build script builds with none of those tools;
  - a C++ runtime link spelled any other way (another attribute order,
    `cfg_attr`, a name built at run time, `rustc-flags`, `rustc-link-arg`),
    and other system libraries linked through `#[link]`;
  - a build-dependency that only a macOS or Windows host uses;
  - what language a native library was written in, whether the package ships
    it or finds it on the machine through pkg-config, system-deps or vcpkg: a
    new listing of one needs a person to confirm it is not C++;
  - a change inside a listed package that keeps its signals: entries name
    signals, not versions;
  - newer compatible versions a user's own lockfile may resolve;
  - other targets (musl, gnullvm, 32-bit, mobile, wasm, fuzzing builds).

Needs the registry (`cargo metadata` and `cargo tree`), and no build.";

/// The targets the rule covers. `cargo tree --target` resolves each without the
/// target installed.
const DESKTOP: [&str; 7] = [
    "x86_64-unknown-linux-gnu",
    "aarch64-unknown-linux-gnu",
    "x86_64-apple-darwin",
    "aarch64-apple-darwin",
    "x86_64-pc-windows-msvc",
    "aarch64-pc-windows-msvc",
    "x86_64-pc-windows-gnu",
];

/// Build-time crates that mean a native toolchain step.
const NATIVE_TOOLS: [&str; 11] = [
    "autotools",
    "bindgen",
    "cc",
    "cmake",
    "cpp_build",
    "cxx-build",
    "gcc",
    "nasm-rs",
    "pkg-config",
    "system-deps",
    "vcpkg",
];

/// Of [`NATIVE_TOOLS`], the ones no listing can allow: they build C++, or a
/// CMake or autotools project this check cannot see into.
const CPP_TOOLS: [&str; 4] = ["autotools", "cmake", "cpp_build", "cxx-build"];

/// File extensions of native libraries and objects. A versioned shared library
/// (`libfoo.so.1`) is matched by name in [`is_library`].
const LIBRARY_EXTENSIONS: [&str; 7] = ["a", "lib", "so", "dylib", "dll", "o", "obj"];

/// A build script naming C++ sources. String literals only: a bare `.cc` would
/// match Rust code such as `config.cc`, and a bare `"cpp"` is also the name of
/// the C preprocessor.
static CPP_SOURCE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r#"\.(cpp|cc|cxx|c\+\+|mm|cu)"|\.cpp\(\s*true\s*,?\s*\)|-std=(c|gnu)\+\+"#)
        .expect("a literal pattern")
});

/// Rust source linking the C++ runtime, by attribute or by build-script directive.
static CPP_RUNTIME: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(
        r#"#\[link\(\s*name\s*=\s*"(c\+\+|stdc\+\+|c\+\+abi|supc\+\+)"|rustc-link-lib=(dylib=|static=)?(c\+\+|stdc\+\+|c\+\+abi|supc\+\+)""#,
    )
    .expect("a literal pattern")
});

/// What made a package count as native.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Signal {
    /// A `links` key in its manifest.
    Links,
    /// A build-dependency that is, or reaches, this entry of [`NATIVE_TOOLS`].
    Tool(&'static str),
    /// A native library file in its package, and a build script.
    Prebuilt,
    /// A build script naming C++ sources ([`CPP_SOURCE`]).
    CppSource,
    /// A Rust file linking the C++ runtime ([`CPP_RUNTIME`]).
    CppRuntime,
}

impl Signal {
    /// The signals that mean C++, which no listing allows.
    fn is_cpp(self) -> bool {
        match self {
            Tool(tool) => CPP_TOOLS.contains(&tool),
            CppSource => true,
            Links | Prebuilt | CppRuntime => false,
        }
    }
}

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Links => write!(f, "a `links` key"),
            Tool(tool) => write!(f, "the build tool `{tool}`"),
            Prebuilt => write!(f, "a shipped native library"),
            CppSource => write!(f, "a build script naming C++ sources"),
            CppRuntime => write!(f, "a link to the C++ runtime"),
        }
    }
}

/// One allowed native package: its name, exactly the signals it shows, and why
/// it is acceptable.
struct Allowed {
    name: &'static str,
    signals: &'static [Signal],
    why: &'static str,
}

/// The native packages the release set may build. C and assembly are allowed
/// when listed, and so is a link to a runtime the operating system ships;
/// compiling or shipping C++ never is.
const ALLOWED: &[Allowed] = &[
    Allowed {
        name: "atomic-wait",
        signals: &[CppRuntime],
        why:
            "links the system C++ runtime on macOS to call its atomic wait; it has no build script",
    },
    Allowed {
        name: "blake3",
        signals: &[Tool("cc")],
        why: "C and assembly SIMD code, built with cc",
    },
    Allowed {
        name: "objc-sys",
        signals: &[Links],
        why: "links the system Objective-C runtime on macOS",
    },
    Allowed {
        name: "rayon-core",
        signals: &[Links],
        why: "its `links` key only keeps a second copy out of a build; it builds no native code",
    },
    Allowed {
        name: "windows_aarch64_msvc",
        signals: &[Prebuilt],
        why: "an import library for Windows system DLLs",
    },
    Allowed {
        name: "windows_x86_64_gnu",
        signals: &[Prebuilt],
        why: "an import library for Windows system DLLs",
    },
    Allowed {
        name: "windows_x86_64_msvc",
        signals: &[Prebuilt],
        why: "an import library for Windows system DLLs",
    },
    Allowed {
        name: "x11-dl",
        signals: &[Tool("pkg-config")],
        why: "asks pkg-config where X11 is, and loads it at run time",
    },
    Allowed {
        name: "zstd-sys",
        signals: &[Links, Tool("cc"), Tool("pkg-config")],
        why: "builds the zstd C library with cc, and one assembly file outside Windows",
    },
];

/// A package as `cargo tree` prints it: name and version.
type Node = (String, String);

/// One target's dependency graph, as read from `cargo tree`.
#[derive(Debug, Default)]
struct Tree {
    /// Names printed at depth 0.
    roots: BTreeSet<String>,
    /// Every package printed, roots included.
    nodes: BTreeSet<Node>,
    /// `(parent, child, build)`, each once; `build` is true for a build-dependency.
    edges: BTreeSet<(Node, Node, bool)>,
    /// The parent each package was first printed under, so a report can name a path.
    first_parent: BTreeMap<Node, Node>,
}

/// What this check needs to know about one package, from `cargo metadata`.
#[derive(Debug)]
struct Info {
    links: bool,
    /// The package's own directory, for the file scans.
    dir: PathBuf,
    build_script: Option<PathBuf>,
}

/// Every native package found, by name: its signals over all desktop targets and
/// versions, the targets it showed up on, and one path to it.
#[derive(Debug, Default)]
struct Found {
    signals: BTreeSet<Signal>,
    targets: BTreeSet<&'static str>,
    chain: String,
}

/// Check the workspace in the current directory.
///
/// # Errors
///
/// When it finds a problem, or cannot read the workspace or the registry.
pub fn check() -> Result<()> {
    check_at(Path::new("."))
}

/// [`check`], rooted at an explicit workspace directory so the unit test can
/// point at the workspace without `set_current_dir`.
pub(crate) fn check_at(root: &Path) -> Result<()> {
    let set = crate::publish_set::release_set(root)?;
    let metadata = cargo(
        root,
        &[
            "metadata",
            "--format-version",
            "1",
            "--all-features",
            "--locked",
        ],
    )?;
    let info =
        package_info(&serde_json::from_str(&metadata).context("parse `cargo metadata` JSON")?)?;
    let trees = read_trees(&set, |target| cargo_tree(root, &set, target))?;
    let fixed = package_signals(&info, trees.iter().flat_map(|(_, tree)| &tree.nodes))?;
    let mut found = BTreeMap::new();
    for (target, tree) in &trees {
        collect(&mut found, tree, &fixed, target);
    }
    let problems = problems(&found, ALLOWED);
    if !problems.is_empty() {
        for problem in &problems {
            eprintln!("  ✗ {problem}");
        }
        bail!(
            "{} problem(s) with native builds in the crates.io release set (see `cargo xtask native-build --help`)",
            problems.len()
        );
    }
    println!(
        "✓ the {} release-set crates build no C++; {} native packages on {} desktop targets, all listed:",
        set.len(),
        found.len(),
        DESKTOP.len(),
    );
    for entry in ALLOWED {
        println!("    {} — {}", entry.name, entry.why);
    }
    Ok(())
}

/// Run cargo in `root` and return its stdout. Colour is forced off: CI sets
/// `CARGO_TERM_COLOR=always`, which puts escape codes into `cargo tree`'s lines.
fn cargo(root: &Path, args: &[&str]) -> Result<String> {
    let out = std::process::Command::new("cargo")
        .args(args)
        .env("CARGO_TERM_COLOR", "never")
        .current_dir(root)
        .output()
        .with_context(|| format!("run `cargo {}`", args.join(" ")))?;
    if !out.status.success() {
        bail!(
            "`cargo {}` failed: {}",
            args.join(" "),
            String::from_utf8_lossy(&out.stderr)
        );
    }
    String::from_utf8(out.stdout).context("cargo printed non-UTF-8")
}

/// One `cargo tree` over the whole set for one target. All the set's crates in
/// one invocation, so features unify across them as they would for a user who
/// depends on all of them.
fn cargo_tree(root: &Path, set: &BTreeSet<String>, target: &str) -> Result<String> {
    let mut args = vec![
        "tree",
        "--locked",
        "--all-features",
        "--edges",
        "normal,build",
        "--target",
        target,
        "--charset",
        "utf8",
        "--format",
        "{p}",
    ];
    for name in set {
        args.extend(["--package", name]);
    }
    cargo(root, &args)
}

/// Every desktop target's tree, from `run(target)`'s `cargo tree` output, each
/// checked by [`read_tree`].
fn read_trees(
    set: &BTreeSet<String>,
    run: impl Fn(&str) -> Result<String>,
) -> Result<Vec<(&'static str, Tree)>> {
    DESKTOP
        .iter()
        .map(|&target| Ok((target, read_tree(set, target, &run(target)?)?)))
        .collect()
}

/// [`parse`] one target's `cargo tree`, and require its roots to be exactly the
/// set, so a change in the output format fails here instead of reading as a
/// smaller, passing graph.
fn read_tree(set: &BTreeSet<String>, target: &str, text: &str) -> Result<Tree> {
    let tree = parse(text).with_context(|| format!("read `cargo tree --target {target}`"))?;
    if &tree.roots != set {
        let missing: Vec<_> = set.difference(&tree.roots).collect();
        let extra: Vec<_> = tree.roots.difference(set).collect();
        bail!(
            "`cargo tree --target {target}` printed roots that are not the release set \
             (missing {missing:?}, extra {extra:?}); its output format may have changed"
        );
    }
    Ok(tree)
}

/// Every package `cargo metadata` knows, keyed by name and version.
fn package_info(metadata: &Value) -> Result<BTreeMap<Node, Info>> {
    let mut out = BTreeMap::new();
    for pkg in metadata["packages"]
        .as_array()
        .context("`cargo metadata`: missing 'packages' array")?
    {
        let name = pkg["name"].as_str().context("package missing 'name'")?;
        let version = pkg["version"]
            .as_str()
            .context("package missing 'version'")?;
        let manifest = pkg["manifest_path"]
            .as_str()
            .context("package missing 'manifest_path'")?;
        let links = pkg
            .get("links")
            .with_context(|| format!("`{name}` has no 'links' field"))?;
        let mut build_script = None;
        for target in pkg["targets"]
            .as_array()
            .with_context(|| format!("`{name}` has no 'targets' array"))?
        {
            let is_build_script = target["kind"]
                .as_array()
                .is_some_and(|kinds| kinds.iter().any(|k| k == "custom-build"));
            if is_build_script {
                let path = target["src_path"]
                    .as_str()
                    .with_context(|| format!("`{name}`'s build script has no 'src_path'"))?;
                build_script = Some(PathBuf::from(path));
            }
        }
        let info = Info {
            links: !links.is_null(),
            dir: Path::new(manifest)
                .parent()
                .context("manifest path has no parent")?
                .to_path_buf(),
            build_script,
        };
        // Two packages with one name and version come from two sources; the
        // `cargo tree` lines this joins against could not tell them apart.
        if out
            .insert((name.to_string(), version.to_string()), info)
            .is_some()
        {
            bail!("two packages are `{name} {version}`; this check cannot tell them apart");
        }
    }
    Ok(out)
}

/// The signals a package shows on every target: a `links` key, a shipped
/// library, C++ sources named by its build script, a C++ runtime link. Read
/// once for each package the trees reach.
fn package_signals<'a>(
    info: &BTreeMap<Node, Info>,
    nodes: impl Iterator<Item = &'a Node>,
) -> Result<BTreeMap<Node, BTreeSet<Signal>>> {
    let mut out = BTreeMap::new();
    for node in nodes {
        if out.contains_key(node) {
            continue;
        }
        let pkg = info
            .get(node)
            .with_context(|| format!("`cargo metadata` does not know `{} {}`", node.0, node.1))?;
        let mut signals = BTreeSet::new();
        if pkg.links {
            signals.insert(Links);
        }
        let mut ships_library = false;
        for file in package_files(&pkg.dir)? {
            ships_library |= is_library(&file);
            if file.extension().is_some_and(|x| x == "rs") {
                let text = std::fs::read_to_string(&file)
                    .with_context(|| format!("read {}", file.display()))?;
                if CPP_RUNTIME.is_match(&text) {
                    signals.insert(CppRuntime);
                }
            }
        }
        if let Some(script) = &pkg.build_script {
            if ships_library {
                signals.insert(Prebuilt);
            }
            let text = std::fs::read_to_string(script).with_context(|| {
                format!(
                    "read the build script of `{} {}` at {}",
                    node.0,
                    node.1,
                    script.display()
                )
            })?;
            if CPP_SOURCE.is_match(&text) {
                signals.insert(CppSource);
            }
        }
        out.insert(node.clone(), signals);
    }
    Ok(out)
}

/// Every file in a package's directory, skipping `target` and `.git`. A missing
/// or unreadable directory is an error: `cargo fetch` puts registry sources there.
fn package_files(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    for entry in walkdir::WalkDir::new(dir)
        .into_iter()
        .filter_entry(|e| e.file_name() != "target" && e.file_name() != ".git")
    {
        let entry = entry.with_context(|| format!("list {}", dir.display()))?;
        if entry.file_type().is_file() {
            out.push(entry.into_path());
        }
    }
    Ok(out)
}

/// A native library or object file, by extension or as `libfoo.so.1`.
fn is_library(path: &Path) -> bool {
    let by_extension = path
        .extension()
        .and_then(|x| x.to_str())
        .is_some_and(|x| LIBRARY_EXTENSIONS.contains(&x));
    let versioned_so = path
        .file_name()
        .and_then(|n| n.to_str())
        .is_some_and(|n| n.contains(".so."));
    by_extension || versioned_so
}

/// Read `cargo tree`'s indented output. Each level indents four characters; a
/// `[build-dependencies]` line sits at its parent's depth and labels the
/// parent's children that follow it.
fn parse(text: &str) -> Result<Tree> {
    let mut tree = Tree::default();
    // stack[d] = (the package printed at depth d, whether its current section is
    // `[build-dependencies]`)
    let mut stack: Vec<(Node, bool)> = Vec::new();
    for (n, line) in text.lines().enumerate() {
        if line.trim().is_empty() {
            stack.clear();
            continue;
        }
        let body = line.trim_start_matches(['│', '├', '└', '─', ' ']);
        let indent = line.chars().count() - body.chars().count();
        if indent % 4 != 0 {
            bail!("line {}: an indent of {indent}, not a multiple of 4", n + 1);
        }
        let depth = indent / 4;
        if let Some(header) = body.strip_prefix('[') {
            if header != "build-dependencies]" {
                bail!("line {}: unexpected section `[{header}`", n + 1);
            }
            let Some(parent) = stack.get_mut(depth) else {
                bail!("line {}: a section header with no package above it", n + 1);
            };
            parent.1 = true;
            continue;
        }
        let mut words = body.split(' ');
        let (Some(name), Some(version)) = (words.next(), words.next()) else {
            bail!("line {}: expected `name vX.Y.Z`, got `{body}`", n + 1);
        };
        let Some(version) = version.strip_prefix('v') else {
            bail!("line {}: expected `name vX.Y.Z`, got `{body}`", n + 1);
        };
        let node = (name.to_string(), version.to_string());
        if depth > stack.len() {
            bail!("line {}: indented past its parent", n + 1);
        }
        stack.truncate(depth);
        if let Some((parent, build)) = stack.last() {
            tree.edges.insert((parent.clone(), node.clone(), *build));
            tree.first_parent
                .entry(node.clone())
                .or_insert_with(|| parent.clone());
        } else {
            tree.roots.insert(name.to_string());
        }
        tree.nodes.insert(node.clone());
        stack.push((node, false));
    }
    Ok(tree)
}

/// `cortenforge → mesh-io → zip → zstd-sys`, walking first parents.
fn chain(tree: &Tree, node: &Node) -> String {
    let mut links = vec![node.0.clone()];
    let mut at = node;
    while let Some(parent) = tree.first_parent.get(at) {
        links.push(parent.0.clone());
        at = parent;
    }
    links.reverse();
    links.join(" → ")
}

/// The [`NATIVE_TOOLS`] entry named `name`, if it is one.
fn tool(name: &str) -> Option<&'static str> {
    NATIVE_TOOLS.iter().copied().find(|t| *t == name)
}

/// The tools a build-dependency brings into its user's build script: itself, if
/// it is one, or those it reaches through its own normal dependencies, stopping
/// at each tool. (A build-dependency's own build-dependencies serve its build
/// script, not its user's; they count for it.)
fn tools_through(normal: &BTreeMap<&Node, Vec<&Node>>, dep: &Node) -> BTreeSet<&'static str> {
    let mut out = BTreeSet::new();
    let mut seen = BTreeSet::from([dep]);
    let mut queue = VecDeque::from([dep]);
    while let Some(at) = queue.pop_front() {
        if let Some(t) = tool(&at.0) {
            out.insert(t);
            continue;
        }
        for next in normal.get(at).into_iter().flatten() {
            if seen.insert(next) {
                queue.push_back(next);
            }
        }
    }
    out
}

/// Add one target's native packages to `found`: each package's [`package_signals`]
/// plus the tools its build-dependencies bring on this target.
fn collect(
    found: &mut BTreeMap<String, Found>,
    tree: &Tree,
    fixed: &BTreeMap<Node, BTreeSet<Signal>>,
    target: &'static str,
) {
    let mut normal: BTreeMap<&Node, Vec<&Node>> = BTreeMap::new();
    for (parent, child, build) in &tree.edges {
        if !*build {
            normal.entry(parent).or_default().push(child);
        }
    }
    let mut signals: BTreeMap<&Node, BTreeSet<Signal>> = tree
        .nodes
        .iter()
        .map(|node| (node, fixed.get(node).cloned().unwrap_or_default()))
        .collect();
    for (parent, child, build) in &tree.edges {
        if *build {
            let tools = tools_through(&normal, child);
            signals
                .entry(parent)
                .or_default()
                .extend(tools.into_iter().map(Tool));
        }
    }
    for (node, signals) in signals {
        if signals.is_empty() {
            continue;
        }
        let entry = found.entry(node.0.clone()).or_default();
        if entry.chain.is_empty() {
            entry.chain = chain(tree, node);
        }
        entry.signals.extend(signals);
        entry.targets.insert(target);
    }
}

/// Every way `found` departs from the rules in [`LONG_ABOUT`], given `allowed`.
fn problems(found: &BTreeMap<String, Found>, allowed: &[Allowed]) -> Vec<String> {
    let list = |signals: &mut dyn Iterator<Item = &Signal>| {
        signals
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(", ")
    };
    let mut out = Vec::new();
    for (name, f) in found {
        let cpp: Vec<_> = f.signals.iter().filter(|s| s.is_cpp()).collect();
        if !cpp.is_empty() {
            out.push(format!(
                "`{name}` may build C++ ({}; {}): the release set builds none, and no listing \
                 allows it",
                list(&mut cpp.into_iter()),
                f.chain
            ));
        } else if let Some(entry) = allowed.iter().find(|a| a.name == name) {
            let wanted: BTreeSet<Signal> = entry.signals.iter().copied().collect();
            if wanted != f.signals {
                out.push(format!(
                    "`{name}` is listed with [{}] but shows [{}] ({}): check what changed \
                     before updating its entry",
                    list(&mut wanted.iter()),
                    list(&mut f.signals.iter()),
                    f.chain
                ));
            }
        } else {
            out.push(format!(
                "`{name}` builds or links native code ({}) and is not listed ({}; on {}): \
                 add it to `ALLOWED` in xtask/src/native_build.rs with a reason, or drop it",
                list(&mut f.signals.iter()),
                f.chain,
                f.targets.iter().copied().collect::<Vec<_>>().join(", ")
            ));
        }
    }
    for entry in allowed {
        if !found.contains_key(entry.name) {
            out.push(format!(
                "`{}` is listed but the release set no longer builds it: remove its entry",
                entry.name
            ));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two roots; `a` has a normal child and a build section, `b` reappears
    /// deduplicated, and the second root's build section sits at depth 0.
    const TREE: &str = "\
a v1.0.0 (/ws/a)
├── b v0.2.0
│   └── c v0.3.0
│   [build-dependencies]
│   └── cc v1.2.0
│       └── shlex v1.0.0
└── d v0.4.0 (proc-macro)

e v1.0.0 (/ws/e)
├── b v0.2.0 (*)
[build-dependencies]
└── pkg-config v0.3.0
";

    fn node(name: &str, version: &str) -> Node {
        (name.to_string(), version.to_string())
    }

    fn edge(parent: &str, child: &str, build: bool) -> (Node, Node, bool) {
        let find = |n: &str| match n {
            "a" | "e" => node(n, "1.0.0"),
            "b" => node(n, "0.2.0"),
            "c" | "pkg-config" => node(n, "0.3.0"),
            "cc" => node(n, "1.2.0"),
            "shlex" => node(n, "1.0.0"),
            "d" => node(n, "0.4.0"),
            other => panic!("{other}"),
        };
        (find(parent), find(child), build)
    }

    fn set(names: &[&str]) -> BTreeSet<String> {
        names.iter().map(ToString::to_string).collect()
    }

    #[test]
    fn the_parser_reads_edges_and_their_kinds() {
        let tree = parse(TREE).unwrap();
        assert_eq!(tree.roots, set(&["a", "e"]));
        let want: BTreeSet<_> = [
            edge("a", "b", false),
            edge("b", "c", false),
            edge("b", "cc", true),
            edge("cc", "shlex", false),
            edge("a", "d", false),
            edge("e", "b", false),
            edge("e", "pkg-config", true),
        ]
        .into();
        assert_eq!(tree.edges, want);
        assert_eq!(tree.nodes.len(), 8);
    }

    /// A section ends where its parent's subtree ends: `d`, printed after `b`'s
    /// build section closed, is `a`'s NORMAL dependency.
    #[test]
    fn a_build_section_does_not_leak_to_the_next_sibling() {
        let tree = parse(TREE).unwrap();
        assert!(tree.edges.contains(&edge("a", "d", false)));
        assert!(!tree.edges.contains(&edge("a", "d", true)));
    }

    #[test]
    fn the_parser_names_a_path() {
        let tree = parse(TREE).unwrap();
        assert_eq!(chain(&tree, &node("shlex", "1.0.0")), "a → b → cc → shlex");
    }

    #[test]
    fn the_parser_refuses_an_unknown_section() {
        let err = parse("a v1.0.0\n[dev-dependencies]\n└── b v1.0.0\n").unwrap_err();
        assert!(err.to_string().contains("unexpected section"), "{err}");
    }

    #[test]
    fn the_parser_refuses_a_line_without_a_version() {
        let err = parse("a v1.0.0\n└── b\n").unwrap_err();
        assert!(err.to_string().contains("expected `name vX.Y.Z`"), "{err}");
        let err = parse("a v1.0.0\n└── b 1.0.0\n").unwrap_err();
        assert!(err.to_string().contains("expected `name vX.Y.Z`"), "{err}");
    }

    /// A skipped level, an indented line straight after a blank one (the blank
    /// ends a tree), and an indent that is not a whole number of levels.
    #[test]
    fn the_parser_refuses_indents_it_cannot_place() {
        let err = parse("a v1.0.0\n│   └── b v1.0.0\n").unwrap_err();
        assert!(err.to_string().contains("indented past"), "{err}");
        let err = parse("a v1.0.0\n\n└── b v1.0.0\n").unwrap_err();
        assert!(err.to_string().contains("indented past"), "{err}");
        let err = parse("a v1.0.0\n└─ b v1.0.0\n").unwrap_err();
        assert!(err.to_string().contains("not a multiple of 4"), "{err}");
        let err = parse("a v1.0.0\n└ b v1.0.0\n").unwrap_err();
        assert!(err.to_string().contains("not a multiple of 4"), "{err}");
    }

    /// The roots must be exactly the set: a crate missing, or an extra root
    /// (which a misread indent produces), both mean the output was not read as
    /// meant.
    #[test]
    fn the_roots_must_be_exactly_the_set() {
        assert!(read_tree(&set(&["a", "e"]), DESKTOP[0], TREE).is_ok());
        let err = read_tree(&set(&["a", "e", "z"]), DESKTOP[0], TREE).unwrap_err();
        assert!(err.to_string().contains(r#"missing ["z"]"#), "{err}");
        let err = read_tree(&set(&["a"]), DESKTOP[0], TREE).unwrap_err();
        assert!(err.to_string().contains(r#"extra ["e"]"#), "{err}");
    }

    /// Every target's output goes through the roots check: one target printing
    /// a tree without a set crate fails the whole read.
    #[test]
    fn every_target_is_read_through_the_roots_check() {
        let all = read_trees(&set(&["a", "e"]), |_| Ok(TREE.to_string())).unwrap();
        assert_eq!(all.len(), DESKTOP.len());
        let one_short = read_trees(&set(&["a", "e"]), |target| {
            Ok(if target == DESKTOP[3] {
                "a v1.0.0\n"
            } else {
                TREE
            }
            .to_string())
        })
        .unwrap_err();
        assert!(one_short.to_string().contains(DESKTOP[3]), "{one_short}");
    }

    /// The fixed signals: `links` for the named packages, plus any extra ones.
    fn fixed(links: &[&str], extra: &[(&str, Signal)]) -> BTreeMap<Node, BTreeSet<Signal>> {
        let tree = parse(TREE).unwrap();
        tree.nodes
            .into_iter()
            .map(|n| {
                let mut signals = BTreeSet::new();
                if links.contains(&n.0.as_str()) {
                    signals.insert(Links);
                }
                for (name, signal) in extra {
                    if n.0 == *name {
                        signals.insert(*signal);
                    }
                }
                (n, signals)
            })
            .collect()
    }

    fn found_in(tree: &Tree, fixed: &BTreeMap<Node, BTreeSet<Signal>>) -> BTreeMap<String, Found> {
        let mut found = BTreeMap::new();
        collect(&mut found, tree, fixed, DESKTOP[0]);
        found
    }

    /// `b` and `e` have build edges to tools; `cc`'s own normal edge to `shlex`
    /// and the normal edges to `cc` elsewhere are not build steps.
    #[test]
    fn a_build_edge_to_a_tool_is_a_signal_and_a_normal_edge_is_not() {
        let tree = parse(TREE).unwrap();
        let found = found_in(&tree, &fixed(&[], &[]));
        assert_eq!(
            found.keys().map(String::as_str).collect::<Vec<_>>(),
            ["b", "e"]
        );
        assert_eq!(found["b"].signals, BTreeSet::from([Tool("cc")]));
        assert_eq!(found["e"].signals, BTreeSet::from([Tool("pkg-config")]));
        assert_eq!(found["b"].chain, "a → b");
    }

    /// A `cc` under `[dependencies]` (as `cmake` itself has it) is not a build step.
    #[test]
    fn a_tool_as_a_normal_dependency_is_not_a_signal() {
        let tree = parse("a v1.0.0\n└── cc v1.2.0\n").unwrap();
        assert!(found_in(&tree, &BTreeMap::new()).is_empty());
    }

    /// A build-dependency's own build-dependency serves its build script, so
    /// `cc` counts for `x`, not for `p`.
    #[test]
    fn a_build_dependencys_own_build_tool_counts_for_it_alone() {
        let tree = parse(
            "p v1.0.0\n[build-dependencies]\n└── x v1.0.0\n    [build-dependencies]\n    └── cc v1.2.0\n",
        )
        .unwrap();
        let found = found_in(&tree, &BTreeMap::new());
        assert_eq!(found["x"].signals, BTreeSet::from([Tool("cc")]));
        assert!(!found.contains_key("p"), "{found:?}");
    }

    /// `kernel`'s build-dependency `helper` calls `cc` for it, the way
    /// `cpp_build` does; `user`'s build-dependency `cmake` reaches `cc` too, but
    /// the walk stops at the first tool.
    #[test]
    fn a_tool_reached_through_a_build_dependency_counts_for_its_user() {
        let tree = parse(
            "\
kernel v1.0.0
[build-dependencies]
└── helper v1.0.0
    └── glue v1.0.0
        └── cc v1.2.0

user v1.0.0
[build-dependencies]
└── cmake v0.1.0
    └── cc v1.2.0 (*)
",
        )
        .unwrap();
        let found = found_in(&tree, &BTreeMap::new());
        assert_eq!(found["kernel"].signals, BTreeSet::from([Tool("cc")]));
        assert_eq!(found["user"].signals, BTreeSet::from([Tool("cmake")]));
        assert!(!found.contains_key("helper"), "{found:?}");
    }

    /// From the tree to the verdict: a `cmake` build edge is refused even for a
    /// listed package, so `NATIVE_TOOLS` and `CPP_TOOLS` must agree.
    #[test]
    fn a_cmake_build_edge_is_refused_end_to_end() {
        for helper in CPP_TOOLS {
            assert!(NATIVE_TOOLS.contains(&helper), "{helper}");
            let text = format!("kernel v1.0.0\n[build-dependencies]\n└── {helper} v1.0.0\n");
            let found = found_in(&parse(&text).unwrap(), &BTreeMap::new());
            let listed = &[Allowed {
                name: "kernel",
                signals: &[],
                why: "test",
            }];
            let got = problems(&found, listed);
            assert_eq!(got.len(), 1, "{helper}: {got:?}");
            assert!(got[0].contains("`kernel` may build C++"), "{got:?}");
        }
    }

    /// `--help` names the tools in the same words as the two lists, so a tool
    /// added to or dropped from either one changes the help too.
    #[test]
    fn the_help_names_the_tools_the_code_uses() {
        let help = LONG_ABOUT.split_whitespace().collect::<Vec<_>>().join(" ");
        let native = format!("a native build tool ({})", NATIVE_TOOLS.join(", "));
        assert!(help.contains(&native), "{native}");
        let quoted: Vec<String> = CPP_TOOLS.iter().map(|tool| format!("`{tool}`")).collect();
        let (last, rest) = quoted.split_last().unwrap();
        let refused = format!("no {} or {last} build tool", rest.join(", "));
        assert!(help.contains(&refused), "{refused}");
    }

    #[test]
    fn the_fixed_signals_are_carried_through() {
        let tree = parse(TREE).unwrap();
        let found = found_in(&tree, &fixed(&["c"], &[("d", Prebuilt)]));
        assert_eq!(found["c"].signals, BTreeSet::from([Links]));
        assert_eq!(found["d"].signals, BTreeSet::from([Prebuilt]));
    }

    fn metadata_package(name: &str) -> Value {
        serde_json::json!({
            "name": name, "version": "1.0.0", "manifest_path": format!("/r/{name}/Cargo.toml"),
            "links": null, "targets": [],
        })
    }

    /// Fields this check reads are required, not defaulted: a missing `links`
    /// would otherwise read as "no links". And two packages sharing a name and
    /// version cannot be told apart in `cargo tree`'s output.
    #[test]
    fn package_info_refuses_what_it_cannot_read() {
        let ok = serde_json::json!({ "packages": [metadata_package("a")] });
        assert!(package_info(&ok).is_ok());
        let mut no_links = metadata_package("a");
        no_links.as_object_mut().unwrap().remove("links");
        let err = package_info(&serde_json::json!({ "packages": [no_links] })).unwrap_err();
        assert!(err.to_string().contains("no 'links' field"), "{err}");
        let mut no_targets = metadata_package("a");
        no_targets.as_object_mut().unwrap().remove("targets");
        let err = package_info(&serde_json::json!({ "packages": [no_targets] })).unwrap_err();
        assert!(err.to_string().contains("no 'targets' array"), "{err}");
        let twice =
            serde_json::json!({ "packages": [metadata_package("a"), metadata_package("a")] });
        let err = package_info(&twice).unwrap_err();
        assert!(
            err.to_string().contains("two packages are `a 1.0.0`"),
            "{err}"
        );
    }

    #[test]
    fn package_signals_refuses_an_unknown_package() {
        let unknown = [node("z", "9.9.9")];
        let err = package_signals(&BTreeMap::new(), unknown.iter()).unwrap_err();
        assert!(err.to_string().contains("does not know `z 9.9.9`"), "{err}");
    }

    /// A missing package directory or build script is an error, not a package
    /// with no signals.
    #[test]
    fn package_signals_refuses_sources_it_cannot_read() {
        let info = |build_script| {
            BTreeMap::from([(
                node("c", "0.3.0"),
                Info {
                    links: false,
                    dir: PathBuf::from("/nowhere"),
                    build_script,
                },
            )])
        };
        let known = [node("c", "0.3.0")];
        let err = package_signals(&info(None), known.iter()).unwrap_err();
        assert!(err.to_string().contains("list /nowhere"), "{err}");
        let root = scratch("unreadable-script");
        let mut info = info(Some(root.join("build.rs")));
        info.get_mut(&known[0]).unwrap().dir.clone_from(&root);
        let err = package_signals(&info, known.iter()).unwrap_err();
        std::fs::remove_dir_all(&root).unwrap();
        assert!(err.to_string().contains("read the build script"), "{err}");
    }

    /// A fresh, empty directory for one test.
    fn scratch(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("xtask-native-build-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn write(path: &Path, text: &str) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
    }

    /// The file-based signals, read from packages on disk:
    /// - `kernel`'s build script, in a subdirectory, names a C++ source, and
    ///   it ships a versioned shared library;
    /// - `runtime` has a `links` key but no build script, and a source file
    ///   links libc++;
    /// - `plain` builds C, keeps a library only under `target/` and a
    ///   directory merely named like one, and has nothing else.
    #[test]
    fn package_signals_reads_the_files_of_each_package() {
        let root = scratch("files");
        let (kernel, runtime, plain) = (
            root.join("kernel"),
            root.join("runtime"),
            root.join("plain"),
        );
        write(
            &kernel.join("build/main.rs"),
            "fn main() { cc::Build::new().file(\"src/kernel.cpp\").compile(\"k\"); }",
        );
        write(&kernel.join("lib/libkernel.so.6"), "");
        write(
            &runtime.join("src/macos.rs"),
            "#[link(name = \"c++\")]\nextern \"C\" {}",
        );
        write(
            &plain.join("build.rs"),
            "fn main() { cc::Build::new().file(\"src/plain.c\").compile(\"p\"); }",
        );
        write(&plain.join("target/debug/libplain.a"), "");
        std::fs::create_dir_all(plain.join("docs.a")).unwrap();
        let info = BTreeMap::from([
            (
                node("kernel", "1.0.0"),
                Info {
                    links: false,
                    build_script: Some(kernel.join("build/main.rs")),
                    dir: kernel,
                },
            ),
            (
                node("runtime", "1.0.0"),
                Info {
                    links: true,
                    build_script: None,
                    dir: runtime,
                },
            ),
            (
                node("plain", "1.0.0"),
                Info {
                    links: false,
                    build_script: Some(plain.join("build.rs")),
                    dir: plain,
                },
            ),
        ]);
        let got = package_signals(&info, info.keys());
        std::fs::remove_dir_all(&root).unwrap();
        let got = got.unwrap();
        assert_eq!(
            got[&node("kernel", "1.0.0")],
            BTreeSet::from([Prebuilt, CppSource])
        );
        assert_eq!(
            got[&node("runtime", "1.0.0")],
            BTreeSet::from([Links, CppRuntime])
        );
        assert_eq!(got[&node("plain", "1.0.0")], BTreeSet::new());
    }

    /// Lines in the shape of real build scripts: meshopt's list of sources, a
    /// `cc` call switched to C++, a C++ standard flag, and the other C++
    /// extensions; then a C build, the C compiler and the C preprocessor by
    /// name, and a feature that shares an extension's spelling.
    #[test]
    fn the_cpp_pattern_matches_cpp_sources_and_not_c() {
        for cpp in [
            r#"        "vendor/src/allocator.cpp","#,
            r#"    build.cpp(true).file("src/shim.c");"#,
            "    build.cpp(\n        true,\n    );",
            r#"    .file("src/tz.cc")"#,
            r#"    .file("src/k.cxx")"#,
            r#"    .file("src/k.c++")"#,
            r#"    .file("src/bridge.mm")"#,
            r#"    .file("src/kernel.cu")"#,
            r#"    .flag("-std=c++17")"#,
            r#"    .flag("-std=gnu++14")"#,
        ] {
            assert!(CPP_SOURCE.is_match(cpp), "{cpp}");
        }
        for c in [
            r#"    cc::Build::new().file("src/zstd.c").compile("zstd");"#,
            r#"    let compiler = Command::new("cc");"#,
            r#"    let pre = Command::new("cpp");"#,
            r#"    let locale = "en_US.C";"#,
            r#"    if cfg!(feature = "cxx") {}"#,
            r"    let flags = config.cc;",
            r"    build.cpp(false);",
            r#"    .flag("-std=c11")"#,
        ] {
            assert!(!CPP_SOURCE.is_match(c), "{c}");
        }
    }

    #[test]
    fn the_runtime_pattern_matches_a_cpp_runtime_link_and_not_c() {
        for cpp in [
            r#"#[link(name = "c++")]"#,
            r#"#[link(name="stdc++", kind = "dylib")]"#,
            r#"    println!("cargo:rustc-link-lib=dylib=stdc++");"#,
            r#"    println!("cargo:rustc-link-lib=c++abi");"#,
            r#"#[link(name = "supc++")]"#,
        ] {
            assert!(CPP_RUNTIME.is_match(cpp), "{cpp}");
        }
        for c in [
            r#"#[link(name = "c")]"#,
            r#"#[link(name = "objc", kind = "dylib")]"#,
            r#"    println!("cargo:rustc-link-lib=dl");"#,
        ] {
            assert!(!CPP_RUNTIME.is_match(c), "{c}");
        }
    }

    fn found(rows: &[(&str, &[Signal])]) -> BTreeMap<String, Found> {
        rows.iter()
            .map(|(name, signals)| {
                let f = Found {
                    signals: signals.iter().copied().collect(),
                    targets: BTreeSet::from([DESKTOP[0]]),
                    chain: format!("root → {name}"),
                };
                (name.to_string(), f)
            })
            .collect()
    }

    const LISTED: &[Allowed] = &[
        Allowed {
            name: "zlib-sys",
            signals: &[Links, Tool("cc")],
            why: "test",
        },
        Allowed {
            name: "import-lib",
            signals: &[Prebuilt],
            why: "test",
        },
    ];

    #[test]
    fn exactly_the_listed_packages_with_their_signals_pass() {
        let got = problems(
            &found(&[
                ("zlib-sys", &[Links, Tool("cc")]),
                ("import-lib", &[Prebuilt]),
            ]),
            LISTED,
        );
        assert_eq!(got, Vec::<String>::new());
    }

    #[test]
    fn an_unlisted_native_package_is_a_problem() {
        let got = problems(
            &found(&[
                ("zlib-sys", &[Links, Tool("cc")]),
                ("import-lib", &[Prebuilt]),
                ("new-sys", &[Tool("cc")]),
            ]),
            LISTED,
        );
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(
            got[0].contains("`new-sys` builds or links native code"),
            "{got:?}"
        );
        assert!(got[0].contains("root → new-sys"), "{got:?}");
    }

    /// Gaining a signal, losing one, and swapping one for another all fail.
    #[test]
    fn a_listed_package_whose_signals_change_is_a_problem() {
        for shown in [
            &[Links, Tool("cc"), Tool("bindgen")][..],
            &[Tool("cc")],
            &[Links, Tool("bindgen")],
        ] {
            let got = problems(
                &found(&[("zlib-sys", shown), ("import-lib", &[Prebuilt])]),
                LISTED,
            );
            assert_eq!(got.len(), 1, "{shown:?}: {got:?}");
            assert!(got[0].contains("`zlib-sys` is listed with"), "{got:?}");
        }
    }

    #[test]
    fn a_listed_package_that_is_gone_is_a_problem() {
        let got = problems(&found(&[("zlib-sys", &[Links, Tool("cc")])]), LISTED);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(got[0].contains("`import-lib` is listed but"), "{got:?}");
    }

    /// Each C++ signal is refused, even for a package whose entry lists it; a
    /// C++ runtime link is not one of them.
    #[test]
    fn cpp_is_refused_even_when_listed() {
        let cases: [&'static [Signal]; 4] = [
            &[Tool("cc"), Tool("cmake")],
            &[Tool("cc"), Tool("cpp_build")],
            &[Tool("cc"), Tool("cxx-build")],
            &[Tool("cc"), CppSource],
        ];
        for signals in cases {
            let listed = &[Allowed {
                name: "kernel-sys",
                signals,
                why: "test",
            }];
            let got = problems(&found(&[("kernel-sys", signals)]), listed);
            assert_eq!(got.len(), 1, "{got:?}");
            assert!(got[0].contains("`kernel-sys` may build C++"), "{got:?}");
        }
        let listed = &[Allowed {
            name: "waiter",
            signals: &[CppRuntime],
            why: "test",
        }];
        assert_eq!(
            problems(&found(&[("waiter", &[CppRuntime])]), listed),
            Vec::<String>::new()
        );
    }

    /// ★ The real workspace passes. The fixtures above are what make a broken
    /// rule fail rather than pass; this makes a broken WORKSPACE fail.
    #[test]
    fn the_release_set_builds_only_listed_native_code() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
        check_at(root).expect("the release set's native builds should all be listed");
    }
}
