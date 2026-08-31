//! Self-test: every gating CI job is wired into the `quality-gate` aggregator.
//!
//! The `main` ruleset requires exactly one status check — the `quality-gate`
//! job — which `needs:` every other gating job. That indirection is what lets
//! us rename/shard individual jobs without touching branch protection, but it
//! has a failure mode: a NEW job that runs a real check yet is left out of
//! `quality-gate.needs` would execute UN-GATED — its red never blocks a merge,
//! because the required aggregator never waited for it.
//!
//! This module is compiled only under `cfg(test)` (see `main.rs`) and hosts a
//! test that parses the workflow and asserts every job except an explicit
//! advisory allowlist is listed in `quality-gate`'s `needs`. So "added a job,
//! forgot to gate it" becomes a hard test failure — the CI config validates its
//! own completeness. It also catches a `needs` entry naming a job that no longer
//! exists (a rename/typo), and a stale allowlist entry.

/// Jobs intentionally NOT gated by `quality-gate`, each for a concrete reason:
/// - `affected` — infra that computes the PR-scope; there is nothing to gate.
/// - `quality-gate` — the aggregator itself; it cannot `need` itself.
/// - `semver` / `sbom` — advisory (breaking-change detection / SBOM artifact),
///   deliberately non-blocking.
///
/// Adding a job forces an explicit choice: wire it into `needs`, or justify an
/// entry here. The test asserts every name here still exists, so the allowlist
/// can't rot silently.
const UNGATED_JOBS: &[&str] = &["affected", "quality-gate", "semver", "sbom"];

/// Extract the top-level job names from a GitHub Actions workflow.
///
/// A job key is a line indented exactly two spaces under `jobs:`, bare (`  x:`),
/// naming only `[A-Za-z0-9_-]`. Deeper-indented keys, comments, and mapping
/// values (`  runs-on: ubuntu`, which does not end in `:`) are skipped.
fn parse_jobs(yaml: &str) -> Vec<String> {
    let mut in_jobs = false;
    let mut jobs = Vec::new();
    for raw in yaml.lines() {
        let line = raw.trim_end();
        if line == "jobs:" {
            in_jobs = true;
            continue;
        }
        if !in_jobs {
            continue;
        }
        if let Some(rest) = line.strip_prefix("  ") {
            // A deeper key (leading space) or comment is not a job.
            if rest.starts_with(' ') || rest.starts_with('#') {
                continue;
            }
            if let Some(name) = rest.strip_suffix(':') {
                if !name.is_empty()
                    && name
                        .chars()
                        .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
                {
                    jobs.push(name.to_string());
                }
            }
        }
    }
    jobs
}

/// Parse the inline `needs: [a, b, c]` list of a named job. Returns empty if the
/// job or an inline `needs` array is absent (the caller asserts non-empty).
fn parse_needs(yaml: &str, job: &str) -> Vec<String> {
    let header = format!("  {job}:");
    let mut in_job = false;
    for raw in yaml.lines() {
        let line = raw.trim_end();
        if line == header {
            in_job = true;
            continue;
        }
        if !in_job {
            continue;
        }
        // A new top-level job key ends this job's block.
        if let Some(rest) = line.strip_prefix("  ") {
            if !rest.starts_with(' ')
                && !rest.starts_with('#')
                && rest.strip_suffix(':').is_some_and(|n| {
                    !n.is_empty()
                        && n.chars()
                            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
                })
            {
                break;
            }
        }
        if let Some(list) = line.trim_start().strip_prefix("needs:") {
            let inner = list.trim().trim_start_matches('[').trim_end_matches(']');
            return inner
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
        }
    }
    Vec::new()
}

/// The workflow, embedded at compile time so the test needs no runtime path and
/// a moved/renamed workflow breaks the build (a loud signal, not a silent skip).
const WORKFLOW: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../.github/workflows/quality-gate.yml"
));

#[test]
fn every_gating_job_is_in_quality_gate_needs() {
    let jobs = parse_jobs(WORKFLOW);
    assert!(
        jobs.len() >= 10,
        "parser found only {} job(s) — the workflow format likely changed: {jobs:?}",
        jobs.len()
    );

    let needs = parse_needs(WORKFLOW, "quality-gate");
    assert!(
        !needs.is_empty(),
        "could not parse `quality-gate` needs (format changed?)"
    );

    // 1. Every job except the advisory allowlist must be gated — otherwise it
    //    runs where a red result never blocks a merge.
    for job in &jobs {
        if UNGATED_JOBS.contains(&job.as_str()) {
            continue;
        }
        assert!(
            needs.contains(job),
            "job `{job}` is not in `quality-gate` needs and is not on the advisory \
             allowlist — it would run UN-GATED (its failure could not block a merge). \
             Add it to `quality-gate.needs`, or to UNGATED_JOBS if it is intentionally \
             advisory."
        );
    }

    // 2. Every `needs` entry must name a real job (catch a rename/typo that would
    //    make the aggregator wait on — or skip — a phantom).
    for n in &needs {
        assert!(
            jobs.contains(n),
            "`quality-gate` needs `{n}`, which is not a job in this workflow \
             (renamed or misspelled?)."
        );
    }

    // 3. The allowlist itself must not rot: every exempt name must still exist.
    for exempt in UNGATED_JOBS {
        assert!(
            jobs.iter().any(|j| j == exempt),
            "UNGATED_JOBS names `{exempt}`, which is no longer a job — prune the allowlist."
        );
    }
}

/// A GATED job must not be conditional on the event type.
///
/// ⚠⚠ This nearly broke the merge queue for the whole repository. The
/// `doc-theft` job first carried `if: github.event_name == 'pull_request'`.
/// This workflow also runs on `merge_group` and `push`; a job whose `if` is
/// false reports **`skipped`**, and `quality-gate` fails anything that is not
/// exactly `success`. Every merge-queue run would have failed the required
/// check, so nothing could ever have landed on main.
///
/// `semver` carries the same condition and is harmless ONLY because it is
/// advisory — it is on [`UNGATED_JOBS`] and nothing waits on it. That made the
/// pattern look safe by example, which is exactly why this needs a test rather
/// than a habit.
///
/// The fix for such a job is not to drop the condition but to move it: let the
/// job always run, and have the STEP decide what it can do for that event.
#[test]
fn no_gated_job_is_conditional_on_the_event_type() {
    let needs = parse_needs(WORKFLOW, "quality-gate");
    assert!(!needs.is_empty(), "could not parse `quality-gate` needs");

    let mut current: Option<String> = None;
    let mut in_steps = false;
    let mut offenders: Vec<String> = Vec::new();

    for raw in WORKFLOW.lines() {
        // A top-level job key: exactly two spaces of indent, then `name:`.
        if let Some(rest) = raw.strip_prefix("  ") {
            if !rest.starts_with(' ') && !rest.starts_with('#') {
                if let Some(name) = rest.strip_suffix(':') {
                    if !name.contains(' ') {
                        current = Some(name.to_string());
                        in_steps = false;
                        continue;
                    }
                }
            }
        }
        if raw.trim_start().starts_with("steps:") {
            in_steps = true;
        }
        // Only the job-level `if:` matters; a step-level one is the correct fix.
        if !in_steps && raw.trim_start().starts_with("if:") && raw.contains("github.event_name") {
            if let Some(job) = current.as_deref() {
                if needs.iter().any(|n| n == job) {
                    offenders.push(job.to_string());
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "gated job(s) {offenders:?} are conditional on `github.event_name`. A \
         skipped job reports `skipped`, and `quality-gate` fails anything that \
         is not `success` — so every merge_group and push run of the required \
         check would fail and nothing could merge. Move the condition into the \
         STEP and let the job always run."
    );
}

#[test]
fn parse_jobs_extracts_top_level_keys_only() {
    let yaml = "\
name: CI
on:
  push:
    branches: [main]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - run: cargo build
  # a comment
  test:
    needs: [build]
    steps:
      - run: cargo test
";
    assert_eq!(parse_jobs(yaml), vec!["build", "test"]);
}

#[test]
fn parse_needs_reads_inline_array() {
    let yaml = "\
jobs:
  build:
    needs: [affected]
  gate:
    needs: [build, test, lint]
    steps:
      - run: true
  after:
    runs-on: ubuntu-latest
";
    assert_eq!(parse_needs(yaml, "gate"), vec!["build", "test", "lint"]);
    // Stops at the job block boundary — `after` is not conflated in.
    assert_eq!(parse_needs(yaml, "build"), vec!["affected"]);
    assert!(parse_needs(yaml, "missing").is_empty());
}
