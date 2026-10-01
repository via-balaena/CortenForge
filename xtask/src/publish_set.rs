//! Self-test: the crates that publish to crates.io are exactly the `cortenforge`
//! facade and everything it pulls in, at one version.
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
use std::path::Path;

use anyhow::{bail, Context, Result};
use serde_json::Value;

/// Shown by `cargo xtask publish-set --help`. The single statement of what this
/// asserts; CI's comment points here rather than restating it.
pub const LONG_ABOUT: &str = "\
Assert that the crates.io release set is exactly the `cortenforge` facade's
closure over normal and build dependencies (optional ones included):

  - no crate inside it is `publish = false` (crates.io needs every one);
  - no crate outside it is publishable (cargo makes a new crate publishable by
    default, and `cargo publish --workspace` ships every crate that is);
  - every crate inside it carries the facade's version, and every versioned
    requirement on a crate inside it, from any workspace member, is exactly
    `=X.Y.Z`. A caret lets the dry-run verify against someone else's
    same-named crate at a higher compatible version;
  - no crate inside it has a dev-dependency WITH a version on a crate outside
    it. cargo keeps such a dev-dependency in the published manifest (it strips
    only versionless ones), and crates.io rejects a manifest naming a crate it
    does not have.

`cargo publish --workspace --dry-run` checks the rest: that each crate packages
and builds from its own tarball. Neither can see name ownership.

Needs no build and no network — only `cargo metadata --no-deps`.";

/// The one crate users depend on. The release set is what it pulls in.
const FACADE: &str = "cortenforge";

/// One workspace package, reduced to what publishing depends on.
struct Package {
    version: String,
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
    let metadata: Value =
        serde_json::from_slice(&out.stdout).context("parse `cargo metadata` JSON")?;
    let packages = packages(&metadata)?;
    let set = closure(&packages)?;
    let found = problems(&packages, &set);
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
    /// rule fail rather than pass; this makes a broken WORKSPACE fail.
    #[test]
    fn the_workspace_publishes_exactly_the_facade_closure() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
        check_at(root).expect("the release set should be the facade's closure");
    }
}
