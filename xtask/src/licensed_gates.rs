//! The licence-gated gate surface: enumerate it, and run it.
//!
//! Some of this workspace's most load-bearing gates cannot run in CI, and never
//! will. They read the `BodyParts3D` anatomy meshes, which are CC BY-SA 2.1 JP
//! and are therefore **not committed** (see
//! `design/cf-fsu-geometry/BODYPARTS3D.md`). Every one of them is `#[ignore]`d
//! and reads its mesh paths from `$CF_L4_STL` / `$CF_L5_STL` / `$CF_DISC_STL`,
//! so `cargo test` in CI skips the lot.
//!
//! That is a deliberate licensing decision, not a defect. The defect is what
//! grew on top of it: **nobody could say how many such gates existed.** The
//! only way to find them was to grep for the environment variables and hope the
//! grep was complete — and twice in the rung-β arc it was not. An inventory
//! recorded as "6 red gates" turned out to be 11, because the command behind it
//! had enumerated one target rather than a crate; a later sweep of "the
//! licence-gated surface" covered two crates when there were six.
//!
//! # What this module does
//!
//! [`survey`] derives the surface from the source tree instead of from anyone's
//! memory of it: every `#[test]` carrying an `#[ignore = "…"]` whose reason
//! names one of [`MESH_VARS`]. Deriving it means a gate added tomorrow is in
//! the list tomorrow, with nothing to update — the failure mode a committed
//! manifest of 39 entries would reintroduce the first time someone forgot it.
//!
//! # The seam that keeps it honest
//!
//! Deriving the list from ignore reasons makes the *reason string* load-bearing:
//! a gate written `#[ignore = "needs the meshes"]` needs the meshes, is invisible
//! to [`survey`], and silently shrinks the inventory back to a number narrower
//! than its name. So the invariant is enforced rather than documented —
//! `--check` fails on any `#[ignore]`d test in a mesh-touching file that names
//! no variable, with `IGNORED_FOR_OTHER_REASONS` as the explicit, rot-checked
//! escape hatch. Modelled on `workflow_gate`, which validates the CI config's
//! own completeness the same way.
//!
//! It is a **command** and not only a unit test for a specific reason. The same
//! assertion exists as a test, but `cargo test -p xtask` is PR-scoped by the
//! affected set, so a PR touching only `sim/L1/fsu-model` never runs xtask's
//! tests — the check would have fired a merge too late, on `main`. As a
//! licence-free command in an unscoped CI job it fires on the PR itself. The
//! test calls the same function, so the two cannot drift.

use std::collections::BTreeSet;
use std::fmt;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use owo_colors::OwoColorize;
use walkdir::WalkDir;

/// The environment variables naming the licensed `BodyParts3D` meshes.
///
/// Canonical provenance — source repo, pinned commit, per-mesh SHA-256 and a
/// fetch-and-verify recipe — is `design/cf-fsu-geometry/BODYPARTS3D.md`.
pub(crate) const MESH_VARS: [&str; 3] = ["CF_L4_STL", "CF_L5_STL", "CF_DISC_STL"];

/// The pinned identity of each licensed mesh: `(env var, bytes, SHA-256)`.
///
/// **These are a copy, not the source.** `BODYPARTS3D.md` is the source, and
/// `the_pinned_digests_match_the_provenance_doc` fails if this table drifts from
/// it — so the doc stays the single place a human edits, while the tool gets a
/// machine-readable form without parsing prose at runtime.
///
/// ★ Why the digests are enforced at all. The meshes were already *version*-
/// locked — a commit-pinned URL plus these digests — but nothing verified them:
/// `load_from_env` hands whatever path the variable names straight to the
/// oracle. The whole guarantee was "the operator ran `shasum -c` correctly this
/// time". ⚠ The threat here is **drift, not an attacker**: a stale copy, a
/// half-finished download, a re-export, the wrong FMA id. Any of those moves
/// every measured number in the FSU arc and reds every anchor-pinned gate —
/// after which someone hunts a physics regression that was a bad file. That is
/// precisely what α.1 turned out to be, and it cost four merges.
const MESH_ASSETS: [(&str, u64, &str); 3] = [
    (
        "CF_L4_STL",
        771_384,
        "3464b94d03d42e43bf5ba01a741b7677bec8113a8359eb4a2caec960c6cf341c",
    ),
    (
        "CF_L5_STL",
        864_184,
        "244f54f1388b1cb02014a6de9c1b32ef40d398967bc8e17ed56b46c637fe11c1",
    ),
    (
        "CF_DISC_STL",
        724_584,
        "865f96b3f3bfed6536808edadd46c7ff742c6ee6e93eb7e8705863643cec7a94",
    ),
];

/// Path to the provenance doc that owns the digests above.
const PROVENANCE_DOC: &str = "design/cf-fsu-geometry/BODYPARTS3D.md";

/// `#[ignore]`d tests that live in a mesh-touching file but are ignored for a
/// reason unrelated to the licence, each needing its own justification.
///
/// Empty today: every `#[ignore]` in a mesh-touching file is licence-gated. It
/// exists so that a future slow-or-flaky `#[ignore]` in one of these files has
/// somewhere honest to go instead of being given a misleading mesh reason to
/// satisfy the check. `no_stale_entries_in_the_other_reasons_allowlist` asserts
/// every name here still exists, so the allowlist cannot rot silently.
const IGNORED_FOR_OTHER_REASONS: &[&str] = &[];

/// Which test binary a gate is compiled into — the unit of a `cargo test` run.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Target {
    /// A `#[cfg(test)]` test under `src/`, compiled into the lib test binary.
    Lib,
    /// An integration test, `tests/<name>.rs`, with its own binary.
    Integration(String),
}

impl fmt::Display for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Lib => write!(f, "lib"),
            Self::Integration(n) => write!(f, "{n}"),
        }
    }
}

/// One licence-gated test: where it lives, and which meshes it needs.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct Gate {
    /// Owning package name, as `cargo test -p` expects it.
    pub krate: String,
    /// The test binary it is compiled into.
    pub target: Target,
    /// Fully-qualified test path (`tests::foo` for a nested module).
    pub name: String,
    /// Mesh variables named in its ignore reason, in [`MESH_VARS`] order.
    pub vars: Vec<String>,
}

/// A `#[test]` found by the scan, before it is classified as licence-gated.
struct FoundTest {
    name: String,
    /// The `#[ignore = "…"]` reason, or `None` when the test is not ignored.
    ignore_reason: Option<String>,
}

/// Mesh variables named in an ignore reason, in [`MESH_VARS`] order.
fn vars_named_in(reason: &str) -> Vec<String> {
    MESH_VARS
        .iter()
        .filter(|v| reason.contains(**v))
        .map(|v| (*v).to_string())
        .collect()
}

/// Collect `#[test]` functions from a parsed file, descending into inline
/// modules so `mod tests { … }` is covered and the path stays qualified.
fn collect_tests(items: &[syn::Item], prefix: &str, out: &mut Vec<FoundTest>) {
    for item in items {
        match item {
            syn::Item::Fn(f) => {
                let is_test = f.attrs.iter().any(|a| a.path().is_ident("test"));
                if !is_test {
                    continue;
                }
                let ignore_reason = f.attrs.iter().find_map(|a| {
                    if !a.path().is_ident("ignore") {
                        return None;
                    }
                    match &a.meta {
                        // `#[ignore = "reason"]`
                        syn::Meta::NameValue(nv) => match &nv.value {
                            syn::Expr::Lit(l) => match &l.lit {
                                syn::Lit::Str(s) => Some(s.value()),
                                _ => Some(String::new()),
                            },
                            _ => Some(String::new()),
                        },
                        // A bare `#[ignore]` carries no reason.
                        _ => Some(String::new()),
                    }
                });
                out.push(FoundTest {
                    name: format!("{prefix}{}", f.sig.ident),
                    ignore_reason,
                });
            }
            syn::Item::Mod(m) => {
                if let Some((_, inner)) = &m.content {
                    collect_tests(inner, &format!("{prefix}{}::", m.ident), out);
                }
            }
            _ => {}
        }
    }
}

/// The package name declared by a `Cargo.toml`.
fn package_name(manifest: &Path) -> Option<String> {
    let text = std::fs::read_to_string(manifest).ok()?;
    let value: toml::Value = toml::from_str(&text).ok()?;
    Some(value.get("package")?.get("name")?.as_str()?.to_string())
}

/// The nearest ancestor directory holding a `Cargo.toml`, and its package name.
fn owning_package(file: &Path) -> Option<(PathBuf, String)> {
    let mut dir = file.parent()?;
    loop {
        let manifest = dir.join("Cargo.toml");
        if manifest.is_file() {
            if let Some(name) = package_name(&manifest) {
                return Some((dir.to_path_buf(), name));
            }
        }
        dir = dir.parent()?;
    }
}

/// Which test binary a source file's tests compile into, or `None` for files
/// whose tests are not run by `cargo test` (examples, benches, build scripts).
fn target_of(crate_root: &Path, file: &Path) -> Option<Target> {
    let rel = file.strip_prefix(crate_root).ok()?;
    let mut parts = rel.components().map(|c| c.as_os_str().to_string_lossy());
    match parts.next()?.as_ref() {
        "src" => Some(Target::Lib),
        "tests" => {
            // Only `tests/<name>.rs` is its own binary; `tests/<dir>/helper.rs`
            // is a module of one and is reached through it.
            let name = rel.file_stem()?.to_string_lossy().to_string();
            (rel.components().count() == 2).then_some(Target::Integration(name))
        }
        _ => None,
    }
}

/// Every Rust source file in the workspace, skipping build output.
fn source_files(root: &Path) -> Vec<PathBuf> {
    WalkDir::new(root)
        .into_iter()
        .filter_entry(|e| {
            let name = e.file_name().to_string_lossy();
            name != "target" && name != ".git"
        })
        .filter_map(Result::ok)
        .filter(|e| e.file_type().is_file())
        .map(walkdir::DirEntry::into_path)
        .filter(|p| p.extension().is_some_and(|x| x == "rs"))
        .collect()
}

/// One pass over the workspace: what was found, and what could not be read.
///
/// The last two fields matter most. This module exists so that a licence-gated
/// gate cannot go missing quietly, so **the survey must never report an empty
/// answer it is not entitled to** — a mesh-touching file it could not read has
/// an unknown number of gates, not zero, and saying "none" there would rebuild
/// the exact blind spot in a new place.
///
/// This is not hypothetical. While this module was being written, a bad
/// `Cargo.toml` parse made `owning_package` return `None` for *every* file;
/// the tool reported "0 licence-gated gates across 0 crates" and looked
/// perfectly healthy. A broken scanner must not read like an empty workspace.
#[derive(Default)]
struct Survey {
    /// Licence-gated gates, sorted.
    gates: Vec<Gate>,
    /// `#[ignore]`d mesh-touching tests naming no variable, with their file.
    unnamed: Vec<(String, PathBuf)>,
    /// Mesh-touching test files `syn` could not parse. Their gates are UNKNOWN.
    unparsed: Vec<PathBuf>,
    /// Mesh-touching files whose owning package could not be resolved. Ditto.
    unowned: Vec<PathBuf>,
}

/// Classify one file, appending to `out`.
fn survey_file(file: &Path, out: &mut Survey) {
    let Ok(src) = std::fs::read_to_string(file) else {
        return;
    };
    // Only files that reference a mesh variable can hold a licence-gated gate,
    // and parsing every file in the workspace to learn otherwise is wasted work.
    if !MESH_VARS.iter().any(|v| src.contains(v)) {
        return;
    }
    let Some((crate_root, krate)) = owning_package(file) else {
        out.unowned.push(file.to_path_buf());
        return;
    };
    // `None` here is a deliberate classification, not a failure: examples and
    // benches reference the meshes but hold no `cargo test` target.
    let Some(target) = target_of(&crate_root, file) else {
        return;
    };
    let Ok(parsed) = syn::parse_file(&src) else {
        out.unparsed.push(file.to_path_buf());
        return;
    };

    let mut found = Vec::new();
    collect_tests(&parsed.items, "", &mut found);

    for t in found {
        let Some(reason) = t.ignore_reason else {
            // Not ignored: CI runs it, so it demonstrably needs no mesh.
            continue;
        };
        let vars = vars_named_in(&reason);
        if vars.is_empty() {
            out.unnamed.push((t.name, file.to_path_buf()));
        } else {
            out.gates.push(Gate {
                krate: krate.clone(),
                target: target.clone(),
                name: t.name,
                vars,
            });
        }
    }
}

/// Survey the whole workspace in a single walk.
fn survey(root: &Path) -> Survey {
    let mut out = Survey::default();
    for f in source_files(root) {
        survey_file(&f, &mut out);
    }
    out.gates.sort();
    out
}

/// Gates invisible to [`survey`]: `#[ignore]`d, mesh-touching, naming no variable,
/// and not excused by `IGNORED_FOR_OTHER_REASONS`.
fn invisible_gates(s: &Survey) -> Vec<&(String, PathBuf)> {
    s.unnamed
        .iter()
        .filter(|(name, _)| {
            let bare = name.rsplit("::").next().unwrap_or(name);
            !IGNORED_FOR_OTHER_REASONS.contains(&bare)
        })
        .collect()
}

/// The workspace root: this crate's manifest directory has it as its parent.
fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap_or(Path::new("."))
        .to_path_buf()
}

/// `""` or `"s"`, so counted nouns read correctly at 1.
fn plural(n: usize) -> &'static str {
    if n == 1 {
        ""
    } else {
        "s"
    }
}

/// Print the surface, grouped by crate.
fn list(gates: &[Gate]) {
    let mut crates: Vec<&str> = gates.iter().map(|g| g.krate.as_str()).collect();
    crates.dedup();
    for krate in crates {
        let owned: Vec<&Gate> = gates.iter().filter(|g| g.krate == krate).collect();
        println!(
            "\n{}  ({} gate{})",
            krate.bold(),
            owned.len(),
            plural(owned.len())
        );
        for g in owned {
            let vars: Vec<&str> = g
                .vars
                .iter()
                .map(|v| v.trim_start_matches("CF_").trim_end_matches("_STL"))
                .collect();
            println!(
                "   {:<38} {:<64} {}",
                g.target.to_string(),
                g.name,
                vars.join("+").dimmed()
            );
        }
    }
    let crate_count = gates
        .iter()
        .map(|g| &g.krate)
        .collect::<BTreeSet<_>>()
        .len();
    println!(
        "\n{} licence-gated gate{} across {crate_count} crate{}",
        gates.len(),
        plural(gates.len()),
        plural(crate_count)
    );
}

/// Mesh variables the given gates need between them, in [`MESH_VARS`] order.
///
/// Only the selected gates' needs are required, not all three: most crates use
/// a subset (`cf-codesign` and `cf-routing-tests` need L4 alone), and demanding
/// the full triad would refuse a run the meshes on hand can perfectly well do.
fn required_vars(gates: &[Gate]) -> Vec<&'static str> {
    MESH_VARS
        .iter()
        .copied()
        .filter(|v| gates.iter().any(|g| g.vars.iter().any(|gv| gv == v)))
        .collect()
}

/// Hex SHA-256 of a file, streamed so a 900 kB mesh never doubles in memory.
fn sha256_of(path: &Path) -> Result<String> {
    use sha2::{Digest, Sha256};
    let mut file =
        std::fs::File::open(path).with_context(|| format!("cannot open {}", path.display()))?;
    let mut hasher = Sha256::new();
    std::io::copy(&mut file, &mut hasher)
        .with_context(|| format!("cannot read {}", path.display()))?;
    Ok(format!("{:x}", hasher.finalize()))
}

/// Verify every mesh the selected gates need against its pinned identity.
///
/// Runs BEFORE a single gate does, because a gate that runs on the wrong mesh
/// does not fail — it silently reports different physics, which is the whole
/// problem. Only the meshes the selection actually names are checked, so an
/// L4-only crate still runs with L4 alone.
///
/// # Errors
/// Returns an error naming every mesh whose size or digest does not match, and
/// the pinned value it should have had.
fn verify_meshes(vars: &[&str]) -> Result<()> {
    let mut bad = Vec::new();
    for var in vars {
        let Some((_, want_len, want_sha)) = MESH_ASSETS.iter().find(|(v, _, _)| v == var) else {
            // A variable with no pinned identity cannot be checked, and silently
            // skipping it would be the "0 gates" failure in a new place.
            bad.push(format!("  {var}: no pinned digest in MESH_ASSETS"));
            continue;
        };
        let Some(raw) = std::env::var_os(var) else {
            continue; // the unset check upstream already reports this
        };
        let path = PathBuf::from(raw);

        // Size first: it is free, and a truncated download is the common case.
        match std::fs::metadata(&path) {
            Ok(m) if m.len() != *want_len => {
                bad.push(format!(
                    "  {var} -> {}\n      size {} bytes, pinned {want_len}",
                    path.display(),
                    m.len()
                ));
                continue;
            }
            Err(e) => {
                bad.push(format!("  {var} -> {}: {e}", path.display()));
                continue;
            }
            Ok(_) => {}
        }

        match sha256_of(&path) {
            Ok(got) if got != *want_sha => bad.push(format!(
                "  {var} -> {}\n      sha256 {got}\n      pinned {want_sha}",
                path.display()
            )),
            Err(e) => bad.push(format!("  {var} -> {}: {e:#}", path.display())),
            Ok(_) => {}
        }
    }

    if bad.is_empty() {
        println!(
            "{} {} licensed mesh{} verified against the pin in {PROVENANCE_DOC}",
            "OK".green().bold(),
            vars.len(),
            if vars.len() == 1 { "" } else { "es" }
        );
        return Ok(());
    }
    bail!(
        "licensed mesh integrity check FAILED:\n{}\n\n\
         These gates measure physics off these exact bytes — a stale copy, a truncated \
         download or the wrong FMA id does not make them fail, it makes them report \
         different numbers. Re-fetch and verify per {PROVENANCE_DOC}.",
        bad.join("\n")
    );
}

/// Run the surface (or one crate's slice of it), one `cargo test` per target.
///
/// # Errors
/// Returns an error if a mesh the selected gates need is unset, or if any gate
/// fails.
fn run_gates(gates: &[Gate], root: &Path) -> Result<()> {
    let missing: Vec<&str> = required_vars(gates)
        .into_iter()
        .filter(|v| std::env::var_os(v).is_none())
        .collect();
    if !missing.is_empty() {
        bail!(
            "licensed meshes not configured: {} unset (needed by the selected gates).\n\
             These gates read the BodyParts3D anatomy (CC BY-SA 2.1 JP, never committed).\n\
             Fetch + verify per design/cf-fsu-geometry/BODYPARTS3D.md, then export the paths.",
            missing.join(", ")
        );
    }

    verify_meshes(&required_vars(gates))?;

    // One `cargo test` invocation per (crate, target) — the unit cargo accepts.
    let mut targets: Vec<(String, Target)> = gates
        .iter()
        .map(|g| (g.krate.clone(), g.target.clone()))
        .collect();
    targets.dedup();

    let mut failed = Vec::new();
    for (krate, target) in &targets {
        let count = gates
            .iter()
            .filter(|g| &g.krate == krate && &g.target == target)
            .count();
        println!(
            "\n{} {krate} :: {target} ({count} gate{})",
            "▶".bold(),
            plural(count)
        );

        let mut cmd = std::process::Command::new("cargo");
        cmd.current_dir(root)
            .args(["test", "-p", krate, "--release"]);
        match target {
            Target::Lib => cmd.arg("--lib"),
            Target::Integration(name) => cmd.args(["--test", name]),
        };
        cmd.args(["--", "--ignored", "--test-threads=1", "--nocapture"]);

        let status = cmd
            .status()
            .with_context(|| format!("failed to spawn cargo test for {krate} :: {target}"))?;
        if !status.success() {
            failed.push(format!("{krate} :: {target}"));
        }
    }

    if failed.is_empty() {
        println!(
            "\n{} all {} gate{} green",
            "PASS".green().bold(),
            gates.len(),
            plural(gates.len())
        );
        Ok(())
    } else {
        bail!("licence-gated targets failed:\n  {}", failed.join("\n  "));
    }
}

/// The `(var, bytes, sha256)` rows of the provenance doc's asset table.
///
/// Parses the one Markdown table whose rows name a mesh variable. Returns the
/// rows found — **an empty result is a parse failure, never "the doc has no
/// pins"**, and every caller must treat it that way.
fn provenance_rows(doc: &str) -> Vec<(String, u64, String)> {
    let mut out = Vec::new();
    for line in doc.lines() {
        if !line.starts_with('|') || !line.contains("CF_") {
            continue;
        }
        let cells: Vec<&str> = line
            .split('|')
            .map(|c| c.trim().trim_matches('`').trim())
            .collect();
        // | part | FMA | path | var | bytes | sha256 |  → 8 cells with the empty ends
        let Some(var) = cells
            .iter()
            .find(|c| c.starts_with("CF_") && c.ends_with("_STL"))
        else {
            continue;
        };
        let sha = cells
            .iter()
            .find(|c| c.len() == 64 && c.chars().all(|b| b.is_ascii_hexdigit()));
        let bytes = cells
            .iter()
            .filter(|c| !c.is_empty())
            .find_map(|c| c.parse::<u64>().ok());
        if let (Some(sha), Some(bytes)) = (sha, bytes) {
            out.push(((*var).to_string(), bytes, (*sha).to_string()));
        }
    }
    out
}

/// Assert `MESH_ASSETS` still agrees with the provenance doc that owns it.
///
/// The doc is where a human edits a pin; this table is the machine-readable
/// copy. Without this they drift silently and the tool enforces yesterday's
/// bytes. Licence-free — it reads two files — so `--check` runs it in CI.
///
/// # Errors
/// Returns an error if the doc cannot be read or parsed, or if any row differs.
fn verify_pins_match_doc(root: &Path) -> Result<()> {
    let path = root.join(PROVENANCE_DOC);
    let doc = std::fs::read_to_string(&path)
        .with_context(|| format!("cannot read {}", path.display()))?;
    let rows = provenance_rows(&doc);

    // ⚠ Zero rows means the table moved or its format changed — NOT that there
    // is nothing to check. Reporting "all pins agree" here would be the same
    // silent-zero failure this module exists to prevent.
    if rows.len() != MESH_ASSETS.len() {
        bail!(
            "{PROVENANCE_DOC} yielded {} asset rows, expected {} — the table moved or its \
             format changed, so the pins in MESH_ASSETS are UNVERIFIED. Fix the parser or the \
             doc; do not assume they still agree.",
            rows.len(),
            MESH_ASSETS.len()
        );
    }

    let mut drift = Vec::new();
    for (var, bytes, sha) in &MESH_ASSETS {
        match rows.iter().find(|(v, _, _)| v == var) {
            None => drift.push(format!("  {var}: pinned here but absent from the doc")),
            Some((_, doc_bytes, doc_sha)) => {
                if doc_bytes != bytes || doc_sha != sha {
                    drift.push(format!(
                        "  {var}\n      code {bytes} bytes / {sha}\n      doc  {doc_bytes} bytes / {doc_sha}"
                    ));
                }
            }
        }
    }
    if !drift.is_empty() {
        bail!(
            "MESH_ASSETS has drifted from {PROVENANCE_DOC}, which is the source:\n{}",
            drift.join("\n")
        );
    }
    Ok(())
}

/// Assert the enumeration can still see every licence-gated gate.
///
/// Licence-free — a source scan, no meshes and no build — so CI can run it on
/// every PR. That matters more than it looks: the same check exists as a unit
/// test, but `cargo test -p xtask` is PR-scoped by the affected set, so a PR
/// that touches only `sim/L1/fsu-model` never runs xtask's tests. The gate this
/// protects would then be caught only after the merge reached `main`. As a
/// command in an unscoped job it fires on the PR that introduces the problem.
///
/// # Errors
/// Returns an error naming every `#[ignore]`d mesh-touching test whose reason
/// omits the variable it needs, or every mesh-touching file that would not
/// parse — both are ways a gate leaves the inventory without anyone noticing.
fn check(root: &Path) -> Result<()> {
    let s = survey(root);
    let rel = |p: &Path| p.strip_prefix(root).unwrap_or(p).display().to_string();

    // Unreadable files come first: they make the count itself untrustworthy,
    // which is worse than a count short by a nameable amount.
    let unreadable: Vec<(&PathBuf, &str)> = s
        .unowned
        .iter()
        .map(|f| (f, "owning package could not be resolved"))
        .chain(s.unparsed.iter().map(|f| (f, "did not parse")))
        .collect();
    if !unreadable.is_empty() {
        let detail = unreadable
            .iter()
            .map(|(f, why)| format!("  {}  ({why})", rel(f)))
            .collect::<Vec<_>>()
            .join("\n");
        bail!(
            "these files reference a licensed mesh but could not be read, so the number of \
             licence-gated gates in them is UNKNOWN — not zero:\n{detail}\n\n\
             Every count this tool prints is silently short while this is true, and a wholly \
             broken scan reports a healthy-looking `0 gates across 0 crates`. Fix the file, or \
             teach the scanner what it now needs to understand."
        );
    }

    let offenders = invisible_gates(&s);
    if !offenders.is_empty() {
        let detail = offenders
            .iter()
            .map(|(name, file)| format!("  {name}  ({})", rel(file)))
            .collect::<Vec<_>>()
            .join("\n");
        bail!(
            "these #[ignore]d tests sit in files that reference a licensed mesh but name no \
             mesh variable in their ignore reason, so `cargo xtask licensed-gates` cannot see \
             them and nothing will ever tell you they went red:\n{detail}\n\n\
             Name the variable(s) each needs — `#[ignore = \"needs $CF_L4_STL …\"]` — or add \
             it to IGNORED_FOR_OTHER_REASONS in xtask/src/licensed_gates.rs with a reason."
        );
    }

    verify_pins_match_doc(root)?;

    let gates = &s.gates;
    let crates = gates
        .iter()
        .map(|g| &g.krate)
        .collect::<BTreeSet<_>>()
        .len();
    println!(
        "{} {} licence-gated gate{} across {crates} crate{} — all visible to the enumerator",
        "OK".green().bold(),
        gates.len(),
        plural(gates.len()),
        plural(crates)
    );
    Ok(())
}

/// `cargo xtask licensed-gates` — list the surface, run it, or check it.
///
/// # Errors
/// Returns an error if `only` names no crate in the surface, if the check finds
/// an invisible gate, or if a run fails.
pub fn run(only: Option<String>, do_run: bool, do_check: bool) -> Result<()> {
    let root = workspace_root();

    if do_check {
        return check(&root);
    }

    let s = survey(&root);
    // A list or a run that quietly skipped a file it could not read would be
    // the failure this whole module is against. Say so on every path.
    for f in s.unparsed.iter().chain(&s.unowned) {
        eprintln!(
            "{} {} references a licensed mesh but could not be read — its gates are NOT in this \
             list",
            "warning:".yellow().bold(),
            f.strip_prefix(&root).unwrap_or(f).display()
        );
    }
    let mut gates = s.gates;

    if let Some(krate) = &only {
        gates.retain(|g| &g.krate == krate);
        if gates.is_empty() {
            bail!("no licence-gated gates in crate `{krate}`");
        }
    }

    if do_run {
        run_gates(&gates, &root)
    } else {
        list(&gates);
        println!(
            "\n{}",
            "these cannot run in CI (licensed meshes); `--run` executes them locally".dimmed()
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The seam that keeps the enumeration honest.
    ///
    /// [`survey`] recognises a licence-gated gate by the mesh variable named in
    /// its ignore reason. A gate ignored with a vaguer reason still needs the
    /// meshes, still cannot run in CI, and is **invisible to every tool built
    /// on this module** — which is precisely how an inventory comes to be
    /// narrower than its name. Licence-free, so CI enforces it.
    #[test]
    fn every_licence_gated_test_names_its_vars() {
        // Exercises the same `check` the CI job runs, so the two cannot drift.
        if let Err(e) = check(&workspace_root()) {
            panic!("{e}");
        }
    }

    /// A mesh-touching file that will not parse must be REPORTED, not counted
    /// as zero gates.
    ///
    /// This is the module's own disease: an enumerator that answers "none" for
    /// a file it could not read produces a number narrower than its name, which
    /// is the exact failure it exists to prevent. `coverage.rs` learned the same
    /// lesson and tracks its unparsed files for the same reason.
    #[test]
    fn an_unparseable_mesh_touching_file_is_reported_not_counted_as_zero() {
        let dir = std::env::temp_dir().join("cf_licensed_gates_unparsed");
        let krate = dir.join("crate-a");
        std::fs::create_dir_all(krate.join("tests")).expect("temp dirs");
        std::fs::write(
            krate.join("Cargo.toml"),
            "[package]\nname = \"crate-a\"\nversion = \"0.1.0\"\n",
        )
        .expect("manifest");
        // References a mesh variable, sits in tests/, and is not valid Rust.
        std::fs::write(
            krate.join("tests").join("broken.rs"),
            "#[test]\n#[ignore = \"needs $CF_L4_STL\"]\nfn g() { this is not rust ((( }\n",
        )
        .expect("source");

        let s = survey(&dir);
        assert_eq!(
            s.unparsed.len(),
            1,
            "the unreadable file must be surfaced, not skipped"
        );
        assert!(
            s.gates.is_empty(),
            "and it contributes no gates, which is exactly why it must be reported"
        );
        let err = check(&dir).expect_err("check must refuse to certify an unknown count");
        let msg = err.to_string();
        assert!(
            msg.contains("UNKNOWN") && msg.contains("broken.rs"),
            "the error must name the file and refuse to call it zero, got: {msg}"
        );

        std::fs::remove_dir_all(&dir).ok();
    }

    /// A scan that cannot resolve owning packages must not look like a healthy
    /// empty workspace.
    ///
    /// This reproduces a bug hit while writing this module: a `Cargo.toml`
    /// parse failure made `owning_package` return `None` for every file, and
    /// the tool announced "0 licence-gated gates across 0 crates" — a green
    /// light over a completely blind scan. The manifest here is unparseable
    /// TOML, which is the same shape.
    #[test]
    fn a_scan_that_resolves_no_packages_refuses_to_report_zero() {
        let dir = std::env::temp_dir().join("cf_licensed_gates_unowned");
        let krate = dir.join("crate-b");
        std::fs::create_dir_all(krate.join("tests")).expect("temp dirs");
        std::fs::write(krate.join("Cargo.toml"), "this is not = = valid toml [[[\n")
            .expect("manifest");
        std::fs::write(
            krate.join("tests").join("gate.rs"),
            "#[test]\n#[ignore = \"needs $CF_L4_STL\"]\nfn g() {}\n",
        )
        .expect("source");

        let s = survey(&dir);
        assert_eq!(s.unowned.len(), 1, "the unresolvable file must be surfaced");
        assert!(s.gates.is_empty());
        let err = check(&dir).expect_err("a blind scan must not certify a count");
        let msg = err.to_string();
        assert!(
            msg.contains("UNKNOWN") && msg.contains("owning package"),
            "the error must say the count is unknown and why, got: {msg}"
        );

        std::fs::remove_dir_all(&dir).ok();
    }

    /// The pinned digests must still agree with the doc that owns them.
    #[test]
    fn the_pinned_digests_match_the_provenance_doc() {
        if let Err(e) = verify_pins_match_doc(&workspace_root()) {
            panic!("{e}");
        }
    }

    /// The doc parser must find every asset row — and an empty parse must be
    /// reported as a failure, never as "there is nothing to check".
    #[test]
    fn provenance_rows_reads_the_asset_table_and_refuses_to_read_nothing() {
        let doc = "\
| part | FMA ID | path | env var | bytes | SHA-256 |\n\
|---|---|---|---|---|---|\n\
| L4 | FMA13075 | `a/b.stl` | `CF_L4_STL` | 771384 | `3464b94d03d42e43bf5ba01a741b7677bec8113a8359eb4a2caec960c6cf341c` |\n\
| unrelated | x | y | z | 1 | 2 |\n";
        let rows = provenance_rows(doc);
        assert_eq!(rows.len(), 1, "only rows naming a mesh variable count");
        assert_eq!(rows[0].0, "CF_L4_STL");
        assert_eq!(rows[0].1, 771_384);
        assert!(rows[0].2.starts_with("3464b94d"));

        // The load-bearing negative: a doc whose table has moved parses to zero
        // rows, and `verify_pins_match_doc` must treat that as UNVERIFIED.
        assert!(
            provenance_rows("# no table here\n").is_empty(),
            "a doc without the table yields no rows — the caller must not read that as agreement"
        );
    }

    /// A mesh whose bytes do not match its pin must be refused before any gate
    /// runs — the failure this whole feature exists for.
    #[test]
    fn a_mesh_that_does_not_match_its_pin_is_refused() {
        let dir = std::env::temp_dir().join("cf_mesh_digest_check");
        std::fs::create_dir_all(&dir).expect("temp dir");
        let bogus = dir.join("not_the_real_L4.stl");
        std::fs::write(&bogus, b"this is not FMA13075").expect("write");

        // SAFETY: single-threaded test process; the variable is restored below.
        unsafe { std::env::set_var("CF_L4_STL", &bogus) };
        let err = verify_meshes(&["CF_L4_STL"]).expect_err("a wrong mesh must be refused");
        unsafe { std::env::remove_var("CF_L4_STL") };

        let msg = err.to_string();
        assert!(
            msg.contains("CF_L4_STL") && msg.contains("771384"),
            "the error must name the variable and the pinned size, got: {msg}"
        );
        assert!(
            msg.contains("different numbers"),
            "and must say WHY it matters — a wrong mesh reports different physics rather than \
             failing, got: {msg}"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The allowlist must not outlive the tests it excuses.
    #[test]
    fn no_stale_entries_in_the_other_reasons_allowlist() {
        let live: BTreeSet<String> = survey(&workspace_root())
            .unnamed
            .into_iter()
            .map(|(n, _)| n.rsplit("::").next().unwrap_or(&n).to_string())
            .collect();
        let stale: Vec<&&str> = IGNORED_FOR_OTHER_REASONS
            .iter()
            .filter(|n| !live.contains(**n))
            .collect();
        assert!(
            stale.is_empty(),
            "IGNORED_FOR_OTHER_REASONS names tests that no longer exist (or are no longer \
             ignored without a mesh variable); remove them: {stale:?}"
        );
    }

    /// A run must demand only the meshes its selected gates actually name.
    ///
    /// Requiring the full triad would refuse an L4-only crate (`cf-codesign`,
    /// `cf-routing-tests`) to anyone who fetched only what it needs — a gate
    /// that fails for a reason unrelated to the thing under test.
    #[test]
    fn a_run_requires_only_the_meshes_its_gates_name() {
        let gate = |vars: &[&str]| Gate {
            krate: "c".to_string(),
            target: Target::Lib,
            name: "t".to_string(),
            vars: vars.iter().map(|v| (*v).to_string()).collect(),
        };
        assert_eq!(required_vars(&[gate(&["CF_L4_STL"])]), ["CF_L4_STL"]);
        assert_eq!(
            required_vars(&[gate(&["CF_L4_STL"]), gate(&["CF_DISC_STL"])]),
            ["CF_L4_STL", "CF_DISC_STL"],
            "the union across gates, in MESH_VARS order"
        );
        assert!(
            required_vars(&[]).is_empty(),
            "no gates selected demands no meshes"
        );
    }

    /// The scan must survive the shapes these gates actually take: nested in
    /// `mod tests`, multi-variable reasons, and un-ignored neighbours.
    #[test]
    fn collect_tests_finds_nested_and_qualified_names() {
        let src = r#"
            #[test]
            fn licence_free() {}

            #[cfg(test)]
            mod tests {
                #[test]
                #[ignore = "needs $CF_L4_STL/$CF_DISC_STL (CC BY-SA, not committed)"]
                fn gated() {}

                #[test]
                #[ignore = "slow"]
                fn ignored_for_another_reason() {}
            }
        "#;
        let parsed = syn::parse_file(src).expect("fixture parses");
        let mut found = Vec::new();
        collect_tests(&parsed.items, "", &mut found);

        let names: Vec<&str> = found.iter().map(|t| t.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "licence_free",
                "tests::gated",
                "tests::ignored_for_another_reason"
            ],
            "every #[test] is found, and nested ones keep a qualified path"
        );

        let gated = found
            .iter()
            .find(|t| t.name == "tests::gated")
            .expect("the gated test is present");
        let reason = gated.ignore_reason.as_deref().unwrap_or_default();
        assert_eq!(
            vars_named_in(reason),
            ["CF_L4_STL", "CF_DISC_STL"],
            "vars come back in MESH_VARS order, not the order the reason lists them"
        );

        let free = found
            .iter()
            .find(|t| t.name == "licence_free")
            .expect("the un-ignored test is present");
        assert!(
            free.ignore_reason.is_none(),
            "an un-ignored test has no reason"
        );

        let other = found
            .iter()
            .find(|t| t.name == "tests::ignored_for_another_reason")
            .expect("the slow test is present");
        assert!(
            vars_named_in(other.ignore_reason.as_deref().unwrap_or_default()).is_empty(),
            "an unrelated ignore reason names no mesh variable — this is what the \
             soundness check catches"
        );
    }

    /// `tests/<name>.rs` is its own binary; a helper module beneath it is not,
    /// and treating one as a target would make `cargo test --test` fail.
    #[test]
    fn target_classification_matches_what_cargo_accepts() {
        let root = Path::new("/w/c");
        assert_eq!(target_of(root, &root.join("src/lib.rs")), Some(Target::Lib));
        assert_eq!(
            target_of(root, &root.join("src/deep/mod.rs")),
            Some(Target::Lib)
        );
        assert_eq!(
            target_of(root, &root.join("tests/gate.rs")),
            Some(Target::Integration("gate".to_string()))
        );
        assert_eq!(target_of(root, &root.join("tests/common/util.rs")), None);
        assert_eq!(target_of(root, &root.join("examples/demo.rs")), None);
        assert_eq!(target_of(root, &root.join("benches/b.rs")), None);
    }

    /// A reason naming no variable yields no gate — the negative case the
    /// soundness check depends on.
    #[test]
    fn vars_named_in_reads_only_the_mesh_variables() {
        assert!(vars_named_in("needs a local mesh").is_empty());
        assert!(vars_named_in("").is_empty());
        assert_eq!(vars_named_in("$CF_L5_STL only"), ["CF_L5_STL"]);
    }
}
