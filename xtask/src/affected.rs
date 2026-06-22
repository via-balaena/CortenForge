//! Compute the set of workspace crates affected by a diff.
//!
//! This is the engine behind PR-scoped CI: on a pull request we only need to
//! grade and test the crates a change can actually break, not the whole
//! workspace. "Can actually break" is the load-bearing phrase — unlike linting
//! (which the pre-commit hook safely scopes to *changed* crates, since a lint
//! is intra-crate), a test or grade must cover the changed crate **plus its
//! full reverse-dependency closure**: an L0 change (e.g. `sim-core`) can break
//! every crate stacked above it, so testing only the changed crate would be
//! the "passing CI ≠ tests ran" failure mode.
//!
//! Two safety nets make this sound:
//! 1. **Reverse-dependency closure** — from each directly-changed crate we walk
//!    *who depends on it*, transitively, and include all of them.
//! 2. **Full-fallback valve** — a change to a file that can invalidate the
//!    whole graph (the lockfile, the workspace manifest, the toolchain pin, CI
//!    config, or `xtask` itself) trips `needs_full`, and the caller runs the
//!    entire suite. When in doubt, escalate to full.
//!
//! The full suite *always* runs on `main`/merge regardless of this tool, so a
//! miss here can only slow a PR down (by under-scoping feedback), never let a
//! breakage reach `main`.
//!
//! This module is the pure computation + a thin git/cargo-metadata shell. The
//! correctness-critical pieces — file→crate ownership, the full-fallback match,
//! and the reverse-dep closure — are pure functions with unit tests below; the
//! CI wiring that consumes the output lives in a separate change.

use anyhow::{Context, Result};
use std::collections::{BTreeSet, HashMap};
use std::path::Path;
use xshell::{cmd, Shell};

/// The result of an affected-set computation, serialized for CI consumption.
#[derive(Debug, serde::Serialize, PartialEq, Eq)]
pub struct Affected {
    /// When true, a graph-invalidating file changed (lockfile, workspace
    /// manifest, toolchain pin, `.github/`, or `xtask/`) — the caller must run
    /// the FULL suite. `crates` is then the complete workspace list so a
    /// consumer can treat the crate set uniformly without special-casing.
    pub needs_full: bool,
    /// The sorted, de-duplicated set of crates to grade/test: the changed
    /// crates plus their transitive reverse-dependency closure (or every
    /// workspace crate when `needs_full`).
    pub crates: Vec<String>,
}

/// A workspace package's name paired with its directory relative to the
/// workspace root, using `/` separators (e.g. `("sim-core", "sim/L0/core")`).
type CrateDir = (String, String);

/// Decide whether a changed file forces a full-workspace run.
///
/// These are files whose change can invalidate *any* crate's build or the set
/// of jobs that should run, so reverse-dep closure can't bound the blast
/// radius — we escalate to the whole suite:
/// - `Cargo.lock` — a dependency version bump touches everything.
/// - the workspace-root `Cargo.toml` — workspace members / shared `[workspace.
///   dependencies]` / profiles. (A *crate-level* `Cargo.toml` is owned by its
///   crate and handled by ownership + closure, so only the root one trips here.)
/// - `rust-toolchain` / `rust-toolchain.toml` — the compiler itself.
/// - anything under `.github/` — CI definitions decide what runs at all.
/// - anything under `xtask/` — the grader/affected logic is itself the gate.
/// - anything under `.cargo/` — workspace-wide cargo config: `.cargo/config.
///   toml` sets the wasm32 `getrandom_backend` rustflag that the grade
///   WASM-Compat tier builds every L0 crate against, so a change there can
///   flip any L0 crate's wasm build with no dependency edge to follow.
/// - `clippy.toml` — tunes the Clippy lints the grade Clippy tier runs on
///   *every* crate; a threshold change can regrade the whole workspace.
/// - `rustfmt.toml` / `deny.toml` — drive the workspace-wide `format` and
///   cargo-deny gates; included so a policy change re-runs everything it can
///   affect rather than slipping through a crate-scoped PR run.
///
/// Paths are workspace-root-relative with `/` separators, as `git diff
/// --name-only` emits.
fn is_full_fallback(file: &str) -> bool {
    file == "Cargo.lock"
        || file == "Cargo.toml"
        || file == "rust-toolchain"
        || file == "rust-toolchain.toml"
        || file == "clippy.toml"
        || file == "rustfmt.toml"
        || file == "deny.toml"
        || file.starts_with(".github/")
        || file.starts_with("xtask/")
        || file.starts_with(".cargo/")
}

/// Return the name of the workspace crate that owns `file`, if any.
///
/// A crate owns `file` when its directory is a path-prefix of the file *on a
/// directory boundary* (so `mesh/mesh` does not own `mesh/mesh-io/...`). When
/// crates nest, the **deepest** (longest) owning directory wins, so a file in
/// an inner crate is attributed to that inner crate rather than an outer one.
/// Files outside every crate (top-level `docs/`, `README.md`, …) return `None`
/// and affect nothing on their own — graph-invalidating files are caught by
/// [`is_full_fallback`] instead.
fn owning_crate<'a>(file: &str, crate_dirs: &'a [CrateDir]) -> Option<&'a str> {
    crate_dirs
        .iter()
        .filter(|(_, dir)| {
            // Root package (dir == "") owns anything not claimed by a deeper
            // crate; a non-empty dir must match on a `/` boundary.
            dir.is_empty() || file == dir || file.starts_with(&format!("{dir}/"))
        })
        .max_by_key(|(_, dir)| dir.len())
        .map(|(name, _)| name.as_str())
}

/// Expand a seed set of crates to include every crate that transitively depends
/// on any of them.
///
/// `reverse_deps` maps a crate to its direct dependents (the inverse of the
/// dependency edges). The returned set contains the seeds plus the full
/// transitive closure over those reverse edges, so changing a low-level crate
/// pulls in everything stacked above it. Output ordering is deterministic
/// (`BTreeSet`).
fn reverse_dep_closure(
    seeds: &BTreeSet<String>,
    reverse_deps: &HashMap<String, Vec<String>>,
) -> BTreeSet<String> {
    let mut closure: BTreeSet<String> = BTreeSet::new();
    let mut stack: Vec<String> = seeds.iter().cloned().collect();
    while let Some(c) = stack.pop() {
        if !closure.insert(c.clone()) {
            continue; // already visited
        }
        if let Some(dependents) = reverse_deps.get(&c) {
            for d in dependents {
                if !closure.contains(d) {
                    stack.push(d.clone());
                }
            }
        }
    }
    closure
}

/// Compute the affected set from a diff's changed files and the workspace graph.
///
/// Pure orchestration so it can be unit-tested with hand-built inputs:
/// - any full-fallback file ⇒ `needs_full`, `crates` = every workspace member;
/// - otherwise map each changed file to its owning crate, then take the
///   reverse-dependency closure of those crates.
fn compute_affected(
    changed_files: &[String],
    crate_dirs: &[CrateDir],
    reverse_deps: &HashMap<String, Vec<String>>,
    all_crates: &[String],
) -> Affected {
    if changed_files.iter().any(|f| is_full_fallback(f)) {
        let mut crates: Vec<String> = all_crates.to_vec();
        crates.sort();
        crates.dedup();
        return Affected {
            needs_full: true,
            crates,
        };
    }

    let seeds: BTreeSet<String> = changed_files
        .iter()
        .filter_map(|f| owning_crate(f, crate_dirs).map(String::from))
        .collect();

    let closure = reverse_dep_closure(&seeds, reverse_deps);
    Affected {
        needs_full: false,
        crates: closure.into_iter().collect(),
    }
}

/// List the files a PR changed relative to `base`, via `git diff base...HEAD`.
///
/// The three-dot form diffs `HEAD` against the merge-base with `base`, i.e. the
/// net change the branch introduces — the same set a PR shows. Requires `base`
/// to be fetched (CI must check out with enough history); a missing base
/// surfaces as a git error rather than a silent empty diff.
fn changed_files(sh: &Shell, base: &str) -> Result<Vec<String>> {
    let range = format!("{base}...HEAD");
    let out = cmd!(sh, "git diff --name-only {range}")
        .read()
        .with_context(|| format!("`git diff --name-only {range}` failed (is `{base}` fetched?)"))?;
    Ok(out
        .lines()
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty())
        .collect())
}

/// Build `(crate_dirs, reverse_deps, all_crate_names)` from `cargo metadata`.
///
/// `--no-deps` scopes packages to workspace members; each package still lists
/// its `dependencies`, which we filter to workspace members to get the
/// intra-workspace edges and invert into the reverse-dependency map.
fn workspace_graph(
    sh: &Shell,
) -> Result<(Vec<CrateDir>, HashMap<String, Vec<String>>, Vec<String>)> {
    let metadata_json = cmd!(sh, "cargo metadata --format-version 1 --no-deps")
        .read()
        .context("Failed to run `cargo metadata`")?;
    let metadata: serde_json::Value =
        serde_json::from_str(&metadata_json).context("Failed to parse `cargo metadata` JSON")?;

    let workspace_root = metadata["workspace_root"]
        .as_str()
        .context("`cargo metadata`: missing 'workspace_root'")?;
    let packages = metadata["packages"]
        .as_array()
        .context("`cargo metadata`: missing 'packages' array")?;

    let members: BTreeSet<String> = packages
        .iter()
        .filter_map(|p| p["name"].as_str().map(String::from))
        .collect();

    let mut crate_dirs: Vec<CrateDir> = Vec::with_capacity(packages.len());
    let mut reverse_deps: HashMap<String, Vec<String>> = HashMap::new();

    for pkg in packages {
        let name = pkg["name"]
            .as_str()
            .context("`cargo metadata`: package missing 'name'")?
            .to_string();

        let manifest_path = pkg["manifest_path"]
            .as_str()
            .context("`cargo metadata`: package missing 'manifest_path'")?;
        let rel_dir = crate_rel_dir(workspace_root, manifest_path)?;
        crate_dirs.push((name.clone(), rel_dir));

        // Edge: `name` depends on each workspace-member dep `d` ⇒ `d`'s
        // dependents include `name`.
        if let Some(deps) = pkg["dependencies"].as_array() {
            for dep in deps {
                if let Some(dep_name) = dep["name"].as_str() {
                    if members.contains(dep_name) && dep_name != name {
                        reverse_deps
                            .entry(dep_name.to_string())
                            .or_default()
                            .push(name.clone());
                    }
                }
            }
        }
    }

    Ok((crate_dirs, reverse_deps, members.into_iter().collect()))
}

/// Derive a crate's workspace-root-relative directory (with `/` separators)
/// from its absolute `manifest_path`.
fn crate_rel_dir(workspace_root: &str, manifest_path: &str) -> Result<String> {
    let dir = Path::new(manifest_path)
        .parent()
        .context("manifest_path has no parent directory")?;
    let rel = dir
        .strip_prefix(workspace_root)
        .unwrap_or(dir)
        .to_string_lossy()
        .replace('\\', "/");
    Ok(rel.trim_start_matches('/').to_string())
}

/// Entry point: print the crates affected by the diff against `base`.
///
/// With `--json`, emits `{"needs_full": bool, "crates": [...]}` for CI to
/// parse; otherwise a short human summary. Building the affected set never
/// runs any crate's tests — it is pure graph analysis over `git diff` +
/// `cargo metadata`.
pub fn run(base: &str, json: bool) -> Result<()> {
    let sh = Shell::new()?;
    let root = find_workspace_root(&sh)?;
    sh.change_dir(&root);

    let files = changed_files(&sh, base)?;
    let (crate_dirs, reverse_deps, all_crates) = workspace_graph(&sh)?;
    let affected = compute_affected(&files, &crate_dirs, &reverse_deps, &all_crates);

    if json {
        println!("{}", serde_json::to_string(&affected)?);
    } else if affected.needs_full {
        eprintln!(
            "  affected (vs {base}): FULL — a graph-invalidating file changed; \
             run all {} crates.",
            affected.crates.len()
        );
    } else if affected.crates.is_empty() {
        eprintln!("  affected (vs {base}): none — no workspace crate is touched.");
    } else {
        eprintln!(
            "  affected (vs {base}): {} crate(s) (changed + reverse-dep closure):",
            affected.crates.len()
        );
        for c in &affected.crates {
            eprintln!("    - {c}");
        }
    }
    Ok(())
}

/// Locate the workspace root via `cargo locate-project --workspace`.
fn find_workspace_root(sh: &Shell) -> Result<String> {
    let out = cmd!(
        sh,
        "cargo locate-project --workspace --message-format plain"
    )
    .read()?;
    let cargo_toml = out.trim();
    let root = Path::new(cargo_toml)
        .parent()
        .context("Could not find workspace root")?;
    Ok(root.to_string_lossy().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dirs() -> Vec<CrateDir> {
        vec![
            ("sim-core".into(), "sim/L0/core".into()),
            ("sim-soft".into(), "sim/L0/soft".into()),
            ("sim-coupling".into(), "sim/L1/coupling".into()),
            ("mesh-types".into(), "mesh/mesh-types".into()),
            ("mesh".into(), "mesh/mesh".into()),
            ("cf-cast".into(), "design/cf-cast".into()),
        ]
    }

    // ---- is_full_fallback ----------------------------------------------------

    #[test]
    fn full_fallback_trips_on_graph_invalidating_files() {
        for f in [
            "Cargo.lock",
            "Cargo.toml",
            "rust-toolchain",
            "rust-toolchain.toml",
            ".github/workflows/quality-gate.yml",
            ".github/actions/foo/action.yml",
            "xtask/src/affected.rs",
            // Workspace-global config with no dependency edge to follow:
            ".cargo/config.toml", // wasm32 rustflags → every L0 crate's wasm build
            "clippy.toml",        // Clippy grade tier on every crate
            "rustfmt.toml",       // workspace `format` gate
            "deny.toml",          // cargo-deny policy gate
        ] {
            assert!(is_full_fallback(f), "{f} should trip full-fallback");
        }
    }

    #[test]
    fn full_fallback_ignores_crate_local_and_doc_files() {
        for f in [
            "sim/L0/core/src/lib.rs",
            "sim/L0/core/Cargo.toml", // crate-level manifest is NOT the root one
            "docs/ci/optimization-spec.md",
            "README.md",
            "githubbed/notes.md", // not the `.github/` dir
        ] {
            assert!(!is_full_fallback(f), "{f} should NOT trip full-fallback");
        }
    }

    // ---- owning_crate --------------------------------------------------------

    #[test]
    fn owning_crate_maps_file_to_its_crate() {
        let d = dirs();
        assert_eq!(owning_crate("sim/L0/core/src/lib.rs", &d), Some("sim-core"));
        assert_eq!(
            owning_crate("design/cf-cast/tests/seam.rs", &d),
            Some("cf-cast")
        );
    }

    #[test]
    fn owning_crate_picks_deepest_on_directory_boundary() {
        // "mesh/mesh" must NOT swallow "mesh/mesh-types/..."; boundary-aware
        // matching attributes the file to the more specific crate.
        let d = dirs();
        assert_eq!(
            owning_crate("mesh/mesh-types/src/lib.rs", &d),
            Some("mesh-types")
        );
        assert_eq!(owning_crate("mesh/mesh/src/lib.rs", &d), Some("mesh"));
    }

    #[test]
    fn owning_crate_is_none_outside_every_crate() {
        let d = dirs();
        assert_eq!(owning_crate("docs/STANDARDS.md", &d), None);
        assert_eq!(owning_crate("MISSION.md", &d), None);
        // a sibling dir sharing a prefix is not a member match
        assert_eq!(owning_crate("sim/L0/coreX/src/lib.rs", &d), None);
    }

    #[test]
    fn owning_crate_root_package_is_fallback_only() {
        let mut d = dirs();
        d.push(("root-pkg".into(), "".into()));
        // deeper crate still wins
        assert_eq!(owning_crate("sim/L0/core/src/lib.rs", &d), Some("sim-core"));
        // unclaimed file falls to the root package
        assert_eq!(owning_crate("top_level.rs", &d), Some("root-pkg"));
    }

    // ---- reverse_dep_closure -------------------------------------------------

    fn rev_graph() -> HashMap<String, Vec<String>> {
        // Dependency edges: coupling→soft→core, coupling→core, cast→mesh.
        // Reverse map (dep ⇒ dependents):
        let mut m: HashMap<String, Vec<String>> = HashMap::new();
        m.insert(
            "sim-core".into(),
            vec!["sim-soft".into(), "sim-coupling".into()],
        );
        m.insert("sim-soft".into(), vec!["sim-coupling".into()]);
        m.insert("mesh".into(), vec!["cf-cast".into()]);
        m
    }

    #[test]
    fn closure_pulls_in_all_transitive_dependents() {
        let g = rev_graph();
        let seeds: BTreeSet<String> = ["sim-core".to_string()].into_iter().collect();
        let got = reverse_dep_closure(&seeds, &g);
        let want: BTreeSet<String> = ["sim-core", "sim-soft", "sim-coupling"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        assert_eq!(got, want, "a core change must pull every crate above it");
    }

    #[test]
    fn closure_of_leaf_is_just_itself() {
        let g = rev_graph();
        let seeds: BTreeSet<String> = ["sim-coupling".to_string()].into_iter().collect();
        let got = reverse_dep_closure(&seeds, &g);
        let want: BTreeSet<String> = ["sim-coupling".to_string()].into_iter().collect();
        assert_eq!(got, want, "a top-level crate has no dependents to add");
    }

    #[test]
    fn closure_handles_multiple_seeds_and_dedups() {
        let g = rev_graph();
        let seeds: BTreeSet<String> = ["sim-soft", "mesh"].iter().map(|s| s.to_string()).collect();
        let got = reverse_dep_closure(&seeds, &g);
        let want: BTreeSet<String> = ["sim-soft", "sim-coupling", "mesh", "cf-cast"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        assert_eq!(got, want);
    }

    // ---- compute_affected (orchestration) ------------------------------------

    fn all() -> Vec<String> {
        dirs().into_iter().map(|(n, _)| n).collect()
    }

    #[test]
    fn compute_full_fallback_returns_every_crate() {
        let got = compute_affected(
            &[
                "Cargo.lock".to_string(),
                "sim/L0/core/src/lib.rs".to_string(),
            ],
            &dirs(),
            &rev_graph(),
            &all(),
        );
        assert!(got.needs_full);
        let mut want = all();
        want.sort();
        assert_eq!(got.crates, want);
    }

    #[test]
    fn compute_changed_crate_plus_reverse_closure() {
        let got = compute_affected(
            &["sim/L0/core/src/integrate.rs".to_string()],
            &dirs(),
            &rev_graph(),
            &all(),
        );
        assert!(!got.needs_full);
        assert_eq!(got.crates, vec!["sim-core", "sim-coupling", "sim-soft"]);
    }

    #[test]
    fn compute_doc_only_change_affects_nothing() {
        let got = compute_affected(
            &[
                "docs/ci/optimization-spec.md".to_string(),
                "README.md".to_string(),
            ],
            &dirs(),
            &rev_graph(),
            &all(),
        );
        assert!(!got.needs_full);
        assert!(got.crates.is_empty(), "doc-only diffs touch no crate");
    }

    #[test]
    fn crate_rel_dir_strips_root_and_manifest() {
        let got = crate_rel_dir("/ws", "/ws/sim/L0/core/Cargo.toml").unwrap();
        assert_eq!(got, "sim/L0/core");
    }
}
