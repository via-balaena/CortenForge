//! Self-test: the crates that publish to crates.io are exactly the `cortenforge`
//! facade and everything it pulls in, at one version, each carrying its licence
//! texts, NOTICE and README.
//!
//! crates.io rejects an upload that names a dependency it does not have, and it
//! checks every dependency in the manifest, optional ones included
//! (`add_dependencies` in crates.io's `src/controllers/krate/publish.rs`, "no
//! known crate named"). So the facade cannot publish until every crate it
//! reaches through a normal or build dependency has. That closure is the
//! smallest set that can publish at all, and this check holds the workspace to
//! it exactly. [`LONG_ABOUT`] states the rules.
//!
//! ⚠ **It cannot see name OWNERSHIP.** Whether crates.io will accept each name
//! from us is a fact about the registry, not the workspace; neither this check
//! nor `cargo publish --dry-run` (which never uploads) can answer it.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use serde_json::Value;

/// Shown by `cargo xtask publish-set --help`. The single statement of what this
/// asserts; CI's comment points here rather than restating it.
pub const LONG_ABOUT: &str = "\
Assert that the crates.io release set is exactly the `cortenforge` facade's
closure over normal and build dependencies (optional ones included):

  - no crate inside it is `publish = false` (crates.io needs every one);
  - every crate inside it but the facade is named `cortenforge-<name>`, so the
    set publishes under one prefix rather than taking bare names on crates.io;
  - no crate outside it is publishable (cargo makes a new crate publishable by
    default, and `cargo publish --workspace` ships every crate that is);
  - every crate inside it carries the facade's version, and every versioned
    requirement on a crate inside it, from any workspace member, is exactly
    `=X.Y.Z`. A caret lets the dry-run verify against someone else's
    same-named crate at a higher compatible version;
  - no crate inside it has a dev-dependency WITH a version on a crate outside
    it. cargo keeps such a dev-dependency in the published manifest (it strips
    only versionless ones), and crates.io rejects a manifest naming a crate it
    does not have;
  - every crate inside it has LICENSE-APACHE, LICENSE-MIT and NOTICE identical
    to the workspace root's, and a README.md. Each of the three is a symlink
    to the root's file, which cargo packages as the file itself. Every crate
    but the facade has exactly the README its name, library name and
    description give; the failure prints it;
  - no crate inside it sets `include`, or a `readme` other than README.md, and
    each `exclude` entry is a plain path (letters, digits, `.`, `_`, `-` and
    `/` only) naming none of those four files. This reads the manifest, not
    cargo's own file list, so it refuses what it cannot read rather than guess.

`cargo publish --workspace --dry-run` checks the rest: that each crate packages
and builds from its own tarball. Neither can see name ownership.

Needs no build and no network: `cargo metadata --no-deps` and the files
themselves.";

/// The one crate users depend on. The release set is what it pulls in.
const FACADE: &str = "cortenforge";

/// The files every crate in the set holds as the workspace root's: the two
/// licence texts and the NOTICE that carries the disclaimer.
const ROOT_FILES: [&str; 3] = ["LICENSE-APACHE", "LICENSE-MIT", "NOTICE"];

/// The README every crate in the set ships.
const README: &str = "README.md";

/// What every crate in the set but the facade is named with.
const PREFIX: &str = "cortenforge-";

/// Whether an `exclude` entry is a plain path: letters, digits, `.`, `_`, `-`
/// and `/`. Anything else (a glob, an escape, a brace, whitespace) is syntax
/// cargo may read as a pattern, so the check refuses it rather than list every
/// form.
fn is_plain_path(entry: &str) -> bool {
    entry
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-' | '/'))
}

/// One workspace package, reduced to what publishing depends on.
struct Package {
    version: String,
    /// The manifest's `description`, if it has one.
    description: Option<String>,
    /// The library target's name, which code uses, if the crate has one.
    lib: Option<String>,
    /// The crate's directory. `None` only for a test fixture.
    dir: Option<PathBuf>,
    /// `publish` unset or `true`, which `cargo metadata` both report as `null`.
    /// `publish = false` reads as `[]`.
    publishable: bool,
    /// Path dependencies on other workspace members, in manifest order.
    deps: Vec<Dep>,
}

struct Dep {
    name: String,
    /// `true` for a dev-dependency, `false` for a normal or build one.
    dev: bool,
    /// The version requirement as `cargo metadata` reports it: `=0.9.0` for
    /// `version = "=0.9.0"`, and `*` for a versionless path dependency. An
    /// explicit `version = "*"` also reads `*`; crates.io refuses that outright,
    /// and this check does not tell the two apart.
    req: String,
}

impl Dep {
    fn versioned(&self) -> bool {
        self.req != "*"
    }
}

/// Check the workspace in the current directory.
///
/// # Errors
///
/// When it finds a problem, or cannot read the workspace.
pub fn check() -> Result<()> {
    check_at(Path::new("."))
}

/// [`check`], rooted at an explicit workspace directory so the unit test can
/// point at the workspace without `set_current_dir`.
pub(crate) fn check_at(root: &Path) -> Result<()> {
    let metadata = workspace_metadata(root)?;
    let workspace_root = metadata["workspace_root"]
        .as_str()
        .map(Path::new)
        .context("`cargo metadata`: missing 'workspace_root'")?;
    let packages = packages(&metadata)?;
    let set = closure(&packages)?;
    let mut found = problems(&packages, &set);
    found.extend(prefix_problems(&set));
    for name in set.keys() {
        let pkg = &packages[name];
        let dir = pkg
            .dir
            .as_deref()
            .with_context(|| format!("`cargo metadata` gave no manifest path for `{name}`"))?;
        found.extend(packaging_problems(
            name,
            pkg.lib.as_deref(),
            pkg.description.as_deref(),
            &packaging(workspace_root, dir)?,
        ));
    }
    if !found.is_empty() {
        for problem in &found {
            eprintln!("  ✗ {problem}");
        }
        bail!(
            "{} problem(s) with the crates.io release set (see `cargo xtask publish-set --help`)",
            found.len()
        );
    }
    println!(
        "✓ {} crates publish, all at {}: `{FACADE}` and everything it pulls in",
        set.len(),
        packages[FACADE].version
    );
    Ok(())
}

/// The release set's crate names, for checks that walk what it builds. It is
/// the facade's closure whether or not the rules above hold.
pub(crate) fn release_set(root: &Path) -> Result<BTreeSet<String>> {
    let packages = packages(&workspace_metadata(root)?)?;
    Ok(closure(&packages)?.into_keys().collect())
}

/// [`release_set`], and the version every crate in it carries: the facade's.
pub(crate) fn release_set_and_version(root: &Path) -> Result<(BTreeSet<String>, String)> {
    let packages = packages(&workspace_metadata(root)?)?;
    let version = packages
        .get(FACADE)
        .map(|facade| facade.version.clone())
        .with_context(|| format!("the workspace has no `{FACADE}` package"))?;
    Ok((closure(&packages)?.into_keys().collect(), version))
}

/// `cargo metadata --no-deps` for the workspace at `root`.
fn workspace_metadata(root: &Path) -> Result<Value> {
    let out = std::process::Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .current_dir(root)
        .output()
        .context("run `cargo metadata`")?;
    if !out.status.success() {
        bail!(
            "`cargo metadata` failed: {}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
    serde_json::from_slice(&out.stdout).context("parse `cargo metadata` JSON")
}

/// Workspace members keyed by name, with their path dependencies on each other.
fn packages(metadata: &Value) -> Result<BTreeMap<String, Package>> {
    let list = metadata["packages"]
        .as_array()
        .context("`cargo metadata`: missing 'packages' array")?;
    let members: BTreeSet<&str> = list.iter().filter_map(|p| p["name"].as_str()).collect();
    let mut out = BTreeMap::new();
    for pkg in list {
        let name = pkg["name"]
            .as_str()
            .context("`cargo metadata`: package missing 'name'")?;
        let publishable = match &pkg["publish"] {
            Value::Null => true,
            Value::Array(registries) if registries.is_empty() => false,
            // Any registry list, `["crates-io"]` included: fail closed rather
            // than guess which set it belongs to.
            other => {
                bail!("`{name}` has `publish = {other}`; this check knows only true and false")
            }
        };
        let mut deps = Vec::new();
        for dep in pkg["dependencies"].as_array().into_iter().flatten() {
            let dep_name = dep["name"].as_str().unwrap_or_default();
            // A registry dependency that happens to share a member's name is not
            // an edge inside the workspace; only a `path` makes it one. A path
            // outside the workspace (an `exclude`d or out-of-tree crate) has no
            // member entry to follow, so this check cannot vouch for it.
            if dep["path"].is_null() || !members.contains(dep_name) {
                continue;
            }
            deps.push(Dep {
                name: dep_name.to_string(),
                dev: dep["kind"].as_str() == Some("dev"),
                req: dep["req"].as_str().unwrap_or_default().to_string(),
            });
        }
        let version = pkg["version"]
            .as_str()
            .context("`cargo metadata`: package missing 'version'")?;
        out.insert(
            name.to_string(),
            Package {
                version: version.to_string(),
                description: pkg["description"].as_str().map(str::to_owned),
                lib: pkg["targets"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .find(|target| {
                        target["kind"]
                            .as_array()
                            .is_some_and(|kinds| kinds.iter().any(|kind| kind == "lib"))
                    })
                    .and_then(|target| target["name"].as_str())
                    .map(str::to_owned),
                dir: pkg["manifest_path"]
                    .as_str()
                    .and_then(|manifest| Path::new(manifest).parent())
                    .map(Path::to_path_buf),
                publishable,
                deps,
            },
        );
    }
    Ok(out)
}

/// The facade and every member it reaches through normal or build
/// dependencies, each mapped to the member that first reached it (the facade
/// maps to itself), so a report can name the path.
fn closure(packages: &BTreeMap<String, Package>) -> Result<BTreeMap<String, String>> {
    if !packages.contains_key(FACADE) {
        bail!("no workspace member named `{FACADE}`; the release set has no root");
    }
    let mut reached = BTreeMap::from([(FACADE.to_string(), FACADE.to_string())]);
    let mut queue = VecDeque::from([FACADE.to_string()]);
    while let Some(name) = queue.pop_front() {
        for dep in packages[&name].deps.iter().filter(|d| !d.dev) {
            if !reached.contains_key(&dep.name) {
                reached.insert(dep.name.clone(), name.clone());
                queue.push_back(dep.name.clone());
            }
        }
    }
    Ok(reached)
}

/// `cortenforge → sim → sim-coupling`, walking the closure's parent links.
fn chain(set: &BTreeMap<String, String>, name: &str) -> String {
    let mut links = vec![name.to_string()];
    let mut at = name;
    while at != FACADE {
        at = &set[at];
        links.push(at.to_string());
    }
    links.reverse();
    links.join(" → ")
}

/// Every way the workspace departs from the rules in [`LONG_ABOUT`], given the
/// facade's [`closure`].
fn problems(packages: &BTreeMap<String, Package>, set: &BTreeMap<String, String>) -> Vec<String> {
    let facade_version = &packages[FACADE].version;
    let wanted_req = format!("={facade_version}");
    let mut found = Vec::new();
    for (name, pkg) in packages {
        match (set.contains_key(name), pkg.publishable) {
            (true, false) => found.push(format!(
                "`{name}` is `publish = false`, but the facade pulls it in ({}) and \
                 crates.io needs every crate it names",
                chain(set, name)
            )),
            (false, true) => found.push(format!(
                "`{name}` is publishable but outside the facade's closure: add \
                 `publish = false`, or have the facade pull it in"
            )),
            _ => {}
        }
        for dep in &pkg.deps {
            if dep.versioned() && set.contains_key(&dep.name) && dep.req != wanted_req {
                found.push(format!(
                    "`{name}` asks for `{}` {}, but a requirement on a crate in the set must be \
                     exactly ={facade_version}: \
                     set `version = \"={facade_version}\"` on `{}`'s entry in \
                     `[workspace.dependencies]`",
                    dep.name, dep.req, dep.name
                ));
            }
        }
        if !set.contains_key(name) {
            continue;
        }
        if &pkg.version != facade_version {
            found.push(format!(
                "`{name}` is at {}, the facade at {facade_version}: use \
                 `version.workspace = true`",
                pkg.version
            ));
        }
        for dep in &pkg.deps {
            if dep.dev && dep.versioned() && !set.contains_key(&dep.name) {
                found.push(format!(
                    "`{name}` has a dev-dependency on `{}` WITH a version, and `{}` does \
                     not publish, so crates.io would reject `{name}`: keep `{}`'s entry \
                     path-only so cargo strips it",
                    dep.name, dep.name, dep.name
                ));
            }
        }
    }
    found
}

/// Every crate in the facade's [`closure`] but the facade itself that is not
/// named with [`PREFIX`].
fn prefix_problems(set: &BTreeMap<String, String>) -> Vec<String> {
    set.keys()
        .filter(|name| name.as_str() != FACADE && !name.starts_with(PREFIX))
        .map(|name| {
            format!(
                "`{name}` is in the release set but not named `{PREFIX}…` ({}): rename the \
                 package and keep its `[lib] name`",
                chain(set, name)
            )
        })
        .collect()
}

/// How a crate's copy of one of [`ROOT_FILES`] compares with the root's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Held {
    Missing,
    Differs,
    Same,
}

/// What a crate's directory and manifest say about what it packages.
struct Packaging {
    /// Each of [`ROOT_FILES`], held as the root's or not.
    root_files: Vec<(&'static str, Held)>,
    /// The crate's README.md, if it has one.
    readme: Option<String>,
    /// The manifest's `readme`, when set to anything but `"README.md"`.
    readme_key: Option<String>,
    /// Manifest keys this check cannot read: `include`, or an `exclude` that is
    /// not a list of strings.
    unreadable: Vec<&'static str>,
    /// The manifest's `exclude` entries.
    exclude: Vec<String>,
}

/// Read a crate's packaging from its directory `dir`, against the workspace
/// `root`'s files.
fn packaging(root: &Path, dir: &Path) -> Result<Packaging> {
    let mut root_files = Vec::new();
    for file in ROOT_FILES {
        let wanted = std::fs::read(root.join(file))
            .with_context(|| format!("read the workspace root's `{file}`"))?;
        let held = match std::fs::read(dir.join(file)) {
            Ok(bytes) if bytes == wanted => Held::Same,
            Ok(_) => Held::Differs,
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => Held::Missing,
            Err(err) => {
                return Err(err).with_context(|| format!("read {}", dir.join(file).display()))
            }
        };
        root_files.push((file, held));
    }
    let readme = match std::fs::read_to_string(dir.join(README)) {
        Ok(text) => Some(text),
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => None,
        Err(err) => {
            return Err(err).with_context(|| format!("read {}", dir.join(README).display()))
        }
    };
    let manifest_path = dir.join("Cargo.toml");
    let manifest: toml::Value = toml::from_str(
        &std::fs::read_to_string(&manifest_path)
            .with_context(|| format!("read {}", manifest_path.display()))?,
    )
    .with_context(|| format!("parse {}", manifest_path.display()))?;
    let package = manifest.get("package");
    let mut unreadable = Vec::new();
    if package.and_then(|p| p.get("include")).is_some() {
        unreadable.push("include");
    }
    let mut exclude = Vec::new();
    match package.and_then(|p| p.get("exclude")) {
        None => {}
        Some(toml::Value::Array(entries)) => {
            for entry in entries {
                match entry.as_str() {
                    Some(path) => exclude.push(path.to_owned()),
                    None => unreadable.push("exclude"),
                }
            }
        }
        Some(_) => unreadable.push("exclude"),
    }
    unreadable.dedup();
    let readme_key = match package.and_then(|p| p.get("readme")) {
        None => None,
        Some(toml::Value::String(path)) if path == README => None,
        Some(other) => Some(other.to_string()),
    };
    Ok(Packaging {
        root_files,
        readme,
        readme_key,
        unreadable,
        exclude,
    })
}

/// The README of every crate in the set but the facade: the crate's name and
/// description, the name code uses for it, then what it is part of and its
/// licence.
fn member_readme(name: &str, lib: &str, description: &str) -> String {
    format!(
        "# {name}\n\n{description}\n\nIn code, this crate is `{lib}`.\n\n\
         This crate is part of [CortenForge](https://github.com/via-balaena/CortenForge), \
         a Rust SDK for mechatronics and simulation. Most applications depend on the \
         [`cortenforge`](https://crates.io/crates/cortenforge) crate instead, which brings \
         in the rest of the SDK.\n\n\
         Licensed under either of the Apache License, Version 2.0, or the MIT license, at \
         your option. Both texts ship with this crate, with a `NOTICE` that carries the \
         disclaimer.\n"
    )
}

/// Every way crate `name`'s packaging departs from the rules in [`LONG_ABOUT`].
fn packaging_problems(
    name: &str,
    lib: Option<&str>,
    description: Option<&str>,
    packaging: &Packaging,
) -> Vec<String> {
    let mut found = Vec::new();
    for (file, held) in &packaging.root_files {
        match held {
            Held::Missing => found.push(format!(
                "`{name}` has no `{file}`: add a symlink to the workspace root's"
            )),
            Held::Differs => found.push(format!(
                "`{name}`'s `{file}` differs from the workspace root's: make it a symlink to \
                 the root's"
            )),
            Held::Same => {}
        }
    }
    for key in &packaging.unreadable {
        let what = if *key == "include" {
            "`include`, which"
        } else {
            "`exclude` in a form"
        };
        found.push(format!(
            "`{name}` sets {what} this check cannot read: name plain paths in `exclude` instead"
        ));
    }
    for entry in &packaging.exclude {
        let path = entry.trim_start_matches("./").trim_matches('/');
        if !is_plain_path(entry) {
            found.push(format!(
                "`{name}` excludes {entry:?}, which is not a plain path this check can read: \
                 use letters, digits, `.`, `_`, `-` and `/`"
            ));
        } else if path.is_empty() || path == "." {
            found.push(format!("`{name}` excludes `{entry}`, the whole crate"));
        } else if ROOT_FILES.contains(&path) || path == README {
            found.push(format!(
                "`{name}` excludes `{entry}`, which every crate in the set ships"
            ));
        }
    }
    if let Some(key) = &packaging.readme_key {
        found.push(format!(
            "`{name}` sets `readme = {key}`: leave `readme` unset"
        ));
    }
    let Some(readme) = &packaging.readme else {
        found.push(format!("`{name}` has no {README}"));
        return found;
    };
    if name == FACADE {
        return found;
    }
    let Some(description) = description else {
        found.push(format!(
            "`{name}` has no `description`, which its README is made from"
        ));
        return found;
    };
    let Some(lib) = lib else {
        found.push(format!(
            "`{name}` has no library target, whose name its README gives"
        ));
        return found;
    };
    let wanted = member_readme(name, lib, description);
    if *readme != wanted {
        found.push(format!(
            "`{name}`'s {README} is not the one its name, library name and description \
             give; it should read:\n{wanted}"
        ));
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// A `cargo metadata` package. `deps` rows are `(name, kind, req, path)`,
    /// with `kind` as metadata spells it (`null`, `"build"` or `"dev"`).
    fn pkg(
        name: &str,
        version: &str,
        publishable: bool,
        deps: &[(&str, Value, &str, bool)],
    ) -> Value {
        let deps: Vec<Value> = deps
            .iter()
            .map(|(n, kind, req, path)| {
                let mut d = json!({ "name": n, "kind": kind, "req": req });
                if *path {
                    d["path"] = json!(format!("/ws/{n}"));
                }
                d
            })
            .collect();
        json!({
            "name": name,
            "version": version,
            "publish": if publishable { Value::Null } else { json!([]) },
            "dependencies": deps,
        })
    }

    fn found(list: Vec<Value>) -> Vec<String> {
        let packages = packages(&json!({ "packages": list })).unwrap();
        let set = closure(&packages).unwrap();
        problems(&packages, &set)
    }

    /// The facade reaching `a` through an optional edge (as all its real edges
    /// are), `a` reaching `b` through a build edge, and an unpublished helper
    /// `tool` that `b` uses only as a versionless dev-dependency.
    fn consistent_at(version: &str) -> Vec<Value> {
        let req = format!("={version}");
        let mut facade = pkg(FACADE, version, true, &[("a", Value::Null, &req, true)]);
        facade["dependencies"][0]["optional"] = json!(true);
        vec![
            facade,
            pkg("a", version, true, &[("b", json!("build"), &req, true)]),
            pkg("b", version, true, &[("tool", json!("dev"), "*", true)]),
            pkg("tool", version, false, &[]),
        ]
    }

    fn consistent() -> Vec<Value> {
        consistent_at("0.7.0")
    }

    #[test]
    fn a_consistent_workspace_has_no_problems() {
        assert_eq!(found(consistent()), Vec::<String>::new());
    }

    /// The rules compare against the facade's version, not a number baked in.
    #[test]
    fn a_consistent_workspace_at_another_version_has_no_problems() {
        assert_eq!(found(consistent_at("0.9.3")), Vec::<String>::new());
    }

    /// A caret lets `cargo publish --dry-run` verify against someone else's
    /// same-named crate at a higher patch (measured: `sim` at 0.9.0 resolved to
    /// another project's 0.9.2), so a requirement inside the set must be exact.
    #[test]
    fn a_caret_requirement_is_a_problem() {
        let mut ws = consistent();
        ws[1] = pkg("a", "0.7.0", true, &[("b", json!("build"), "^0.7.0", true)]);
        let got = found(ws);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(got[0].contains("`a` asks for `b` ^0.7.0"), "{got:?}");
    }

    /// A bump that moved the crates but missed a dependency entry.
    #[test]
    fn a_requirement_behind_the_set_version_is_a_problem() {
        let mut ws = consistent_at("0.7.1");
        ws[1] = pkg("a", "0.7.1", true, &[("b", json!("build"), "=0.7.0", true)]);
        let got = found(ws);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(got[0].contains("`a` asks for `b` =0.7.0"), "{got:?}");
    }

    /// The facade's own entry is consumed only from outside the set (by an app),
    /// so the requirement rule must look at every member, not only the set.
    #[test]
    fn a_stale_requirement_from_outside_the_set_is_a_problem() {
        let mut ws = consistent_at("0.7.1");
        ws[3] = pkg(
            "tool",
            "0.7.1",
            false,
            &[(FACADE, Value::Null, "=0.7.0", true)],
        );
        let got = found(ws);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(
            got[0].contains("`tool` asks for `cortenforge` =0.7.0"),
            "{got:?}"
        );
    }

    /// cargo strips a versionless dev-dependency, so one on a crate inside the
    /// set asks for nothing and is not held to the set's version.
    #[test]
    fn a_versionless_dev_dependency_inside_the_set_is_fine() {
        let mut ws = consistent();
        ws[1] = pkg(
            "a",
            "0.7.0",
            true,
            &[
                ("b", json!("build"), "=0.7.0", true),
                ("b", json!("dev"), "*", true),
            ],
        );
        assert_eq!(found(ws), Vec::<String>::new());
    }

    /// Optional and target-specific edges both pull a crate in: crates.io
    /// checks them like any other dependency.
    #[test]
    fn optional_and_target_specific_edges_extend_the_closure() {
        let mut ws = consistent();
        ws[1]["dependencies"][0]["optional"] = json!(true);
        ws[1]["dependencies"][0]["target"] = json!("cfg(unix)");
        let set = closure(&packages(&json!({ "packages": ws })).unwrap()).unwrap();
        assert!(set.contains_key("a") && set.contains_key("b"), "{set:?}");
    }

    /// A path dependency on a crate that is not a workspace member has no entry
    /// to follow: it must be skipped, not looked up.
    #[test]
    fn a_path_dependency_outside_the_workspace_is_not_followed() {
        let mut ws = consistent();
        ws[2] = pkg(
            "b",
            "0.7.0",
            true,
            &[
                ("tool", json!("dev"), "*", true),
                ("vendored", Value::Null, "^1", true),
            ],
        );
        assert_eq!(found(ws), Vec::<String>::new());
    }

    #[test]
    fn an_unpublished_crate_inside_the_closure_is_named_with_its_path() {
        let mut ws = consistent();
        ws[2] = pkg("b", "0.7.0", false, &[]);
        let got = found(ws);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(got[0].contains("`b` is `publish = false`"), "{got:?}");
        assert!(got[0].contains("cortenforge → a → b"), "{got:?}");
    }

    #[test]
    fn a_publishable_crate_outside_the_closure_is_a_problem() {
        let mut ws = consistent();
        ws[3] = pkg("tool", "0.7.0", true, &[]);
        let got = found(ws);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(
            got[0].contains("`tool` is publishable but outside"),
            "{got:?}"
        );
    }

    #[test]
    fn a_crate_off_the_version_line_is_a_problem() {
        let mut ws = consistent();
        ws[1] = pkg("a", "2.1.0", true, &[("b", json!("build"), "=0.7.0", true)]);
        let got = found(ws);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(got[0].contains("`a` is at 2.1.0"), "{got:?}");
    }

    /// Only the outside crate's version is off, so a check comparing every
    /// member instead of the release set would flag it.
    #[test]
    fn a_crate_outside_the_set_may_keep_its_own_version() {
        let mut ws = consistent();
        ws[3] = pkg("tool", "0.1.0", false, &[]);
        assert_eq!(found(ws), Vec::<String>::new());
    }

    #[test]
    fn a_versioned_dev_dependency_on_an_unpublished_crate_is_a_problem() {
        let mut ws = consistent();
        // `^1`, not the set's `=0.7.0`: the requirement rule covers only crates
        // inside the set, so this must draw the dev-dependency message alone.
        ws[2] = pkg("b", "0.7.0", true, &[("tool", json!("dev"), "^1", true)]);
        let got = found(ws);
        assert_eq!(got.len(), 1, "{got:?}");
        assert!(
            got[0].contains("dev-dependency on `tool` WITH a version"),
            "{got:?}"
        );
    }

    #[test]
    fn a_versioned_dev_dependency_inside_the_set_is_fine() {
        let mut ws = consistent();
        ws[1] = pkg(
            "a",
            "0.7.0",
            true,
            &[
                ("b", json!("build"), "=0.7.0", true),
                ("b", json!("dev"), "=0.7.0", true),
            ],
        );
        assert_eq!(found(ws), Vec::<String>::new());
    }

    /// A dev-dependency does not pull a crate into the release set: `tool`
    /// stays outside, so it may stay `publish = false`.
    #[test]
    fn a_dev_dependency_does_not_extend_the_closure() {
        let set = closure(&packages(&json!({ "packages": consistent() })).unwrap()).unwrap();
        assert!(!set.contains_key("tool"), "{set:?}");
        assert_eq!(set.len(), 3, "{set:?}");
    }

    /// A registry dependency sharing a member's name is not an edge: without a
    /// `path`, the facade's `tool` is some crates.io crate, not ours.
    #[test]
    fn a_registry_dependency_named_like_a_member_is_not_followed() {
        let mut ws = consistent();
        ws[0] = pkg(
            FACADE,
            "0.7.0",
            true,
            &[
                ("a", Value::Null, "=0.7.0", true),
                ("tool", Value::Null, "^1", false),
            ],
        );
        assert_eq!(found(ws), Vec::<String>::new());
    }

    #[test]
    fn a_registry_list_fails_closed() {
        let mut ws = consistent();
        ws[3]["publish"] = json!(["internal"]);
        let err = packages(&json!({ "packages": ws }))
            .err()
            .expect("must refuse");
        assert!(
            err.to_string().contains("knows only true and false"),
            "{err}"
        );
    }

    #[test]
    fn a_workspace_without_the_facade_is_refused() {
        let err = closure(&packages(&json!({ "packages": consistent()[1..].to_vec() })).unwrap())
            .expect_err("must refuse");
        assert!(
            err.to_string().contains("no workspace member named"),
            "{err}"
        );
    }

    /// ★ The real workspace passes. The fixtures above are what make a broken
    /// rule fail rather than pass; this makes a broken WORKSPACE fail. It runs
    /// from `xtask/`, so the root's files must be found through `cargo
    /// metadata`, not the directory it is given.
    #[test]
    fn the_workspace_publishes_exactly_the_facade_closure() {
        check_at(Path::new(env!("CARGO_MANIFEST_DIR")))
            .expect("the release set should be the facade's closure");
    }

    /// `check_at` applies the packaging rules to every crate in the set and to
    /// none outside it: the facade and its one dependency, both without the
    /// root's NOTICE, fail twice (an unpublished member with no licences,
    /// NOTICE or README adds nothing), once with only the facade's, pass with
    /// both, and fail again when the dependency's README is not the one its
    /// own name and description give.
    #[test]
    fn check_at_reads_each_crates_packaging() {
        let root = std::env::temp_dir().join(format!("cf-check-at-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let write = |path: std::path::PathBuf, text: &str| {
            std::fs::create_dir_all(path.parent().expect("a parent")).expect("make dirs");
            std::fs::write(&path, text).expect("write fixture file");
        };
        write(
            root.join("Cargo.toml"),
            "[workspace]\nmembers = [\"cortenforge\", \"member\", \"outsider\"]\n",
        );
        for file in ROOT_FILES {
            write(root.join(file), &format!("{file} text"));
        }
        let manifest = |name: &str, rest: &str| {
            format!("[package]\nname = \"{name}\"\nversion = \"0.9.0\"\nedition = \"2021\"\n{rest}")
        };
        write(
            root.join("cortenforge/Cargo.toml"),
            &manifest(
                FACADE,
                "description = \"The facade\"\n\
                 [dependencies]\ncortenforge-member = { path = \"../member\", version = \"=0.9.0\" }\n",
            ),
        );
        write(
            root.join("member/Cargo.toml"),
            &manifest("cortenforge-member", "description = \"Does one thing\"\n"),
        );
        write(
            root.join("outsider/Cargo.toml"),
            &manifest("outsider", "publish = false\n"),
        );
        for name in [FACADE, "member", "outsider"] {
            write(root.join(name).join("src/lib.rs"), "");
        }
        for name in [FACADE, "member"] {
            for file in ["LICENSE-APACHE", "LICENSE-MIT"] {
                write(root.join(name).join(file), &format!("{file} text"));
            }
        }
        write(root.join(FACADE).join(README), "# cortenforge\n");
        write(
            root.join("member").join(README),
            &member_readme("cortenforge-member", "cortenforge_member", "Does one thing"),
        );

        let neither = check_at(&root);
        write(root.join(FACADE).join("NOTICE"), "NOTICE text");
        let facade_only = check_at(&root);
        write(root.join("member").join("NOTICE"), "NOTICE text");
        let both = check_at(&root);
        write(
            root.join("member").join(README),
            &member_readme(
                "cortenforge-member",
                "cortenforge_member",
                "Does two things",
            ),
        );
        let stale_readme = check_at(&root);
        write(
            root.join("cortenforge/Cargo.toml"),
            &manifest(
                FACADE,
                "description = \"The facade\"\n\
                 [dependencies]\nmember = { path = \"../member\", version = \"=0.9.0\" }\n",
            ),
        );
        write(
            root.join("member/Cargo.toml"),
            &manifest("member", "description = \"Does one thing\"\n"),
        );
        write(
            root.join("member").join(README),
            &member_readme("member", "member", "Does one thing"),
        );
        let unprefixed = check_at(&root);
        let _ = std::fs::remove_dir_all(&root);
        let err = neither.expect_err("crates without the NOTICE must fail");
        assert!(err.to_string().starts_with("2 problem(s)"), "{err}");
        let err = facade_only.expect_err("the member without its NOTICE must fail");
        assert!(err.to_string().starts_with("1 problem(s)"), "{err}");
        both.expect("the same workspace with both NOTICEs should pass");
        let err = stale_readme.expect_err("the member's stale README must fail");
        assert!(err.to_string().starts_with("1 problem(s)"), "{err}");
        let err = unprefixed.expect_err("a member without the prefix must fail");
        assert!(err.to_string().starts_with("1 problem(s)"), "{err}");
    }

    /// A crate whose packaging breaks no rule: the root's files, the README its
    /// description gives, and a plain `exclude`.
    fn packaged(name: &str) -> Packaging {
        Packaging {
            root_files: ROOT_FILES.iter().map(|file| (*file, Held::Same)).collect(),
            readme: Some(member_readme(
                name,
                &name.replace('-', "_"),
                "Does one thing",
            )),
            readme_key: None,
            unreadable: Vec::new(),
            exclude: vec!["COMPLETION.md".to_owned(), "docs/".to_owned()],
        }
    }

    fn packaging_found(name: &str, packaging: &Packaging) -> Vec<String> {
        packaging_problems(
            name,
            Some(&name.replace('-', "_")),
            Some("Does one thing"),
            packaging,
        )
    }

    #[test]
    fn a_crate_packaged_by_the_rules_has_no_problems() {
        assert_eq!(packaging_found("a", &packaged("a")), Vec::<String>::new());
    }

    /// A missing file and a copy that drifted from the root's each fail.
    #[test]
    fn each_root_file_must_be_the_roots() {
        let mut packaging = packaged("a");
        packaging.root_files[0].1 = Held::Missing;
        packaging.root_files[2].1 = Held::Differs;
        let found = packaging_found("a", &packaging);
        assert_eq!(found.len(), 2, "{found:?}");
        assert!(
            found[0].contains("`a` has no `LICENSE-APACHE`"),
            "{found:?}"
        );
        assert!(found[1].contains("`a`'s `NOTICE` differs"), "{found:?}");
    }

    /// What it cannot read it refuses: `include`, an entry that is not a plain
    /// path, the whole crate, and a plain path naming a file every crate ships.
    #[test]
    fn an_exclude_it_cannot_vouch_for_fails() {
        let mut packaging = packaged("a");
        packaging.unreadable = vec!["include"];
        let unplain = ["*.md", "\\NOTICE", "{NOTICE,x}", "NOTICE\t", "LICENSE-MIT "];
        packaging.exclude = unplain
            .iter()
            .chain(&["./", "/NOTICE", "README.md/", "./LICENSE-MIT"])
            .map(|entry| (*entry).to_owned())
            .collect();
        let found = packaging_found("a", &packaging);
        assert_eq!(found.len(), 10, "{found:?}");
        assert!(found[0].contains("sets `include`, which"), "{found:?}");
        for (line, entry) in found[1..6].iter().zip(unplain) {
            assert!(
                line.contains(&format!("excludes {entry:?}, which is not a plain path")),
                "{line}"
            );
        }
        assert!(found[6].contains("`./`, the whole crate"), "{found:?}");
        for (line, entry) in found[7..]
            .iter()
            .zip(["/NOTICE", "README.md/", "./LICENSE-MIT"])
        {
            assert!(
                line.contains(&format!("excludes `{entry}`, which every crate")),
                "{line}"
            );
        }
    }

    /// Every crate needs a README; every one but the facade needs exactly the
    /// one its name and description give, so it cannot drift from them.
    #[test]
    fn the_readme_is_the_one_the_manifest_gives() {
        let mut packaging = packaged(FACADE);
        packaging.readme = Some("# Written by hand\n".to_owned());
        assert_eq!(packaging_found(FACADE, &packaging), Vec::<String>::new());

        packaging.readme = None;
        assert_eq!(packaging_found("a", &packaging), ["`a` has no README.md"]);
        assert_eq!(
            packaging_found(FACADE, &packaging),
            ["`cortenforge` has no README.md"]
        );

        let mut packaging = packaged("a");
        packaging.readme_key = Some("false".to_owned());
        assert_eq!(
            packaging_found("a", &packaging),
            ["`a` sets `readme = false`: leave `readme` unset"]
        );

        let mut packaging = packaged("a");
        let stale = packaging_problems("a", Some("a"), Some("Does two things"), &packaging);
        assert_eq!(stale.len(), 1, "{stale:?}");
        assert!(
            stale[0].ends_with(&member_readme("a", "a", "Does two things")),
            "{stale:?}"
        );

        packaging.readme = Some(member_readme("b", "b", "Does one thing"));
        assert_eq!(
            packaging_found("a", &packaging).len(),
            1,
            "another crate's README"
        );

        let undescribed = packaging_problems("a", Some("a"), None, &packaged("a"));
        assert!(
            undescribed[0].contains("no `description`"),
            "{undescribed:?}"
        );

        let other_library =
            packaging_problems("a", Some("b"), Some("Does one thing"), &packaged("a"));
        assert_eq!(other_library.len(), 1, "a README naming another library");
        let no_library = packaging_problems("a", None, Some("Does one thing"), &packaged("a"));
        assert!(
            no_library[0].contains("no library target"),
            "{no_library:?}"
        );
    }

    /// Every crate the facade pulls in carries the prefix; the facade itself
    /// does not need it.
    #[test]
    fn a_set_crate_without_the_prefix_is_named() {
        let set = closure(&packages(&json!({ "packages": consistent() })).unwrap()).unwrap();
        let bare = prefix_problems(&set);
        assert_eq!(bare.len(), 2, "{bare:?}");
        assert!(bare[0].starts_with("`a` is in the release set"), "{bare:?}");
        assert!(bare[1].starts_with("`b` is in the release set"), "{bare:?}");
        let prefixed: BTreeMap<String, String> = set
            .into_iter()
            .map(|(name, via)| {
                let rename = |n: String| {
                    if n == FACADE || n.is_empty() {
                        n
                    } else {
                        format!("{PREFIX}{n}")
                    }
                };
                (rename(name), rename(via))
            })
            .collect();
        assert_eq!(prefix_problems(&prefixed), Vec::<String>::new());
    }

    /// The reader follows a symlink, tells a drifted copy and a missing file
    /// apart, and refuses `include`, a `readme` naming another file, and an
    /// `exclude` that is not a list of paths.
    #[cfg(unix)]
    #[test]
    fn packaging_reads_links_copies_and_the_manifest() {
        let root = std::env::temp_dir().join(format!("cf-publish-set-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let (a, b) = (root.join("crates/a"), root.join("crates/b"));
        for dir in [&a, &b] {
            std::fs::create_dir_all(dir).expect("make dirs");
        }
        for file in ROOT_FILES {
            std::fs::write(root.join(file), format!("{file} text")).expect("write root file");
        }
        std::os::unix::fs::symlink("../../LICENSE-APACHE", a.join("LICENSE-APACHE")).expect("link");
        std::fs::write(a.join("LICENSE-MIT"), "an old copy").expect("write copy");
        std::fs::write(a.join(README), "# a\n").expect("write readme");
        std::fs::write(
            a.join("Cargo.toml"),
            "[package]\nname = \"a\"\ninclude = [\"src/\"]\nreadme = \"docs/README.md\"\n\
             exclude = [\"COMPLETION.md\", 3]\n",
        )
        .expect("write manifest");
        std::fs::write(
            b.join("Cargo.toml"),
            "[package]\nname = \"b\"\nreadme = \"README.md\"\nexclude.workspace = true\n",
        )
        .expect("write manifest");

        let (read_a, read_b) = (packaging(&root, &a), packaging(&root, &b));
        let _ = std::fs::remove_dir_all(&root);
        let (read_a, read_b) = (read_a.expect("read a"), read_b.expect("read b"));
        assert_eq!(
            read_a.root_files,
            [
                ("LICENSE-APACHE", Held::Same),
                ("LICENSE-MIT", Held::Differs),
                ("NOTICE", Held::Missing)
            ]
        );
        assert_eq!(read_a.readme.as_deref(), Some("# a\n"));
        assert_eq!(read_a.readme_key.as_deref(), Some("\"docs/README.md\""));
        assert_eq!(read_a.unreadable, ["include", "exclude"]);
        assert_eq!(read_a.exclude, ["COMPLETION.md"]);
        assert_eq!(read_b.readme, None);
        assert_eq!(read_b.readme_key, None);
        assert_eq!(read_b.unreadable, ["exclude"]);
    }
}
