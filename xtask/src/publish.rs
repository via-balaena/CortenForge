//! `cargo xtask publish`: put on crates.io the release set's crates it does not
//! yet have at the facade's version, waiting out its rate limits.
//!
//! [`LONG_ABOUT`] states what it does and what it cannot see.

use std::collections::BTreeSet;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::{Command, Stdio};
use std::thread::sleep;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use chrono::{DateTime, Utc};
use regex::Regex;
use serde_json::Value;

use crate::crates_io;

/// Shown by `cargo xtask publish --help`. The single statement of what this
/// does; `publish.yml`'s comments point here rather than restating it.
pub const LONG_ABOUT: &str = "\
Publish to crates.io each crate of the release set (the `cortenforge` facade's
closure, as `cargo xtask publish-set` defines it) that crates.io does not have
at the facade's version.

  1. Ask crates.io about each crate of the set, a second apart, and keep those
     it does not have at that version. `--list` prints them, one per line, and
     stops.
  2. Verify them: `cargo publish --locked --dry-run`, which packages each and
     builds it from its own tarball. `--dry-run` stops here.
  3. Upload them: `cargo publish --locked --no-verify`, which orders them by
     dependency. `--no-verify` starts here, for a run whose step 2 already
     passed. The two modes that upload run `publish-set` and `name-owners`
     first.

crates.io lets an account upload 5 new crates at once and then 1 every 10
minutes, and 30 new versions at once and then 1 a minute. When it refuses an
upload with HTTP 429, step 3 waits until the time the refusal gives, asks again
which crates are missing, and uploads those. Any other failure stops it, as do
a refusal that asks for a wait of more than 30 minutes and two refusals in a
row that leave the same crates missing.

Run again after a stop, it publishes only what is still missing; with nothing
missing it does nothing. `--tag vX.Y.Z` refuses to run unless the facade's
version is X.Y.Z.

What it cannot see: crates.io checks some things only at upload, among them its
reserved names and a crate's keywords, so a crate it refuses for those stops
step 3 at that crate. And a name another account takes while it runs, at the
release version, reads as published.

Needs the network and `curl`. Step 3 needs a crates.io token that cargo can use.";

/// Identifies this command to crates.io, as its data access policy asks.
const USER_AGENT: &str = "CortenForge publish (https://github.com/via-balaena/CortenForge)";

/// What cargo prints when crates.io refuses an upload for its rate limit.
const RATE_LIMITED: &str = "(status 429 Too Many Requests)";

/// The wait after a refusal whose time cannot be read: crates.io's slowest
/// refill, one new crate every 10 minutes.
const FALLBACK_WAIT: Duration = Duration::from_secs(10 * 60);

/// Added to the time a refusal gives, so the next upload lands after it.
const MARGIN: Duration = Duration::from_secs(30);

/// The longest wait taken on crates.io's word. A refusal asking for more is
/// not one of the two limits above, so it stops the run instead.
const LONGEST_WAIT: Duration = Duration::from_secs(30 * 60);

/// How far a run goes after asking crates.io.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Print the crates crates.io lacks, and stop.
    List,
    /// Verify them, and stop.
    DryRun,
    /// Upload them without verifying them.
    NoVerify,
    /// Verify them, then upload them.
    Publish,
}

impl Mode {
    /// The mode `--list`, `--dry-run` and `--no-verify` ask for, in that order
    /// of precedence; with none of them, a full publish.
    pub fn from_flags(list: bool, dry_run: bool, no_verify: bool) -> Self {
        if list {
            Self::List
        } else if dry_run {
            Self::DryRun
        } else if no_verify {
            Self::NoVerify
        } else {
            Self::Publish
        }
    }

    /// Whether a run in this mode verifies, and whether it uploads.
    fn steps(self) -> (bool, bool) {
        match self {
            Self::List => (false, false),
            Self::DryRun => (true, false),
            Self::NoVerify => (false, true),
            Self::Publish => (true, true),
        }
    }
}

/// How one `cargo` run ended, and what it wrote to stderr.
struct Outcome {
    success: bool,
    stderr: String,
}

/// Runs the command in `mode`, refusing first if `tag` is not the release
/// version's.
///
/// # Errors
///
/// When the set fails `publish-set` or `name-owners` (in the modes that
/// upload), the tag does not match, crates.io gives an answer it does not
/// expect, or cargo fails as [`LONG_ABOUT`] describes.
pub fn run(mode: Mode, tag: Option<&str>) -> Result<()> {
    let root = Path::new(".");
    let (_, uploads) = mode.steps();
    if uploads {
        crate::publish_set::check_at(root)?;
        crate::name_owners::check()?;
    }
    let (set, version) = crate::publish_set::release_set_and_version(root)?;
    if let Some(tag) = tag {
        check_tag(tag, &version)?;
    }
    let pending_of = || {
        pending(
            &set,
            |name| {
                ask(
                    name,
                    &version,
                    |url| crates_io::curl(url, USER_AGENT),
                    sleep,
                )
            },
            sleep,
        )
    };
    let missing = pending_of()?;
    eprintln!(
        "{} of the {} release-set crates are not on crates.io at {version}",
        missing.len(),
        set.len()
    );
    if mode == Mode::List {
        for name in &missing {
            println!("{name}");
        }
        return Ok(());
    }
    let uploaded = verify_then_upload(mode, missing, run_cargo, |missing| {
        upload(missing, pending_of, run_cargo, Utc::now, sleep)
    })?;
    if uploaded {
        eprintln!(
            "✓ crates.io has all {} release-set crates at {version}",
            set.len()
        );
    }
    Ok(())
}

/// The steps `mode` takes over `missing`: verify them with `cargo` if it
/// verifies, then hand them to `upload` if it uploads. A failed verify
/// uploads nothing. Whether `upload` ran.
fn verify_then_upload(
    mode: Mode,
    missing: Vec<String>,
    mut cargo: impl FnMut(&[String]) -> Result<Outcome>,
    upload: impl FnOnce(Vec<String>) -> Result<()>,
) -> Result<bool> {
    let (verify, uploads) = mode.steps();
    if missing.is_empty() {
        return Ok(false);
    }
    if verify && !cargo(&publish_args("--dry-run", &missing))?.success {
        bail!("`cargo publish --dry-run` failed, so nothing was uploaded");
    }
    if !uploads {
        return Ok(false);
    }
    upload(missing)?;
    Ok(true)
}

/// Refuses a tag that is not `v` followed by `version`.
fn check_tag(tag: &str, version: &str) -> Result<()> {
    if tag.strip_prefix('v') != Some(version) {
        bail!("tag `{tag}` does not name the release version {version}; expected `v{version}`");
    }
    Ok(())
}

/// The crates of `set`, in name order, that crates.io does not have at the
/// release version, asking with `ask` and a `wait` of a second between crates.
fn pending(
    set: &BTreeSet<String>,
    mut ask: impl FnMut(&str) -> Result<bool>,
    mut wait: impl FnMut(Duration),
) -> Result<Vec<String>> {
    let mut missing = Vec::new();
    for (i, name) in set.iter().enumerate() {
        if i > 0 {
            wait(Duration::from_secs(1));
        }
        if !ask(name)? {
            missing.push(name.clone());
        }
    }
    Ok(missing)
}

/// Whether crates.io has `name` at `version`, asked with `get` and retried
/// as [`crates_io::get_retrying`] does.
fn ask(
    name: &str,
    version: &str,
    get: impl FnMut(&str) -> Result<(String, String)>,
    wait: impl FnMut(Duration),
) -> Result<bool> {
    let url = format!("https://crates.io/api/v1/crates/{name}/{version}");
    let (status, body) = crates_io::get_retrying(&url, name, get, wait)?;
    read_status(name, version, &status, &body)
}

/// Reads one answer, refusing anything but that version (200) or a 404 that
/// says the crate, or that version of it, does not exist.
fn read_status(name: &str, version: &str, status: &str, body: &str) -> Result<bool> {
    let json = || -> Result<Value> {
        serde_json::from_str(body)
            .with_context(|| format!("crates.io's answer for `{name}` is not JSON: {body}"))
    };
    match status {
        "200" => {
            let found = json()?;
            let num = found.pointer("/version/num").and_then(Value::as_str);
            let krate = found.pointer("/version/crate").and_then(Value::as_str);
            if num == Some(version) && krate == Some(name) {
                Ok(true)
            } else {
                bail!("crates.io answered 200 for `{name}` {version} with another version: {body}")
            }
        }
        "404" => {
            let detail = json()?
                .pointer("/errors/0/detail")
                .and_then(Value::as_str)
                .map(str::to_owned);
            let absent = [
                format!("crate `{name}` does not exist"),
                format!("crate `{name}` does not have a version `{version}`"),
            ];
            if detail.as_ref().is_some_and(|d| absent.contains(d)) {
                Ok(false)
            } else {
                bail!(
                    "crates.io answered 404 for `{name}` {version} without saying it does not exist: {body}"
                )
            }
        }
        _ => bail!("crates.io answered HTTP {status} for `{name}` {version}: {body}"),
    }
}

/// `cargo publish --locked <flag> -p <crate>…`.
fn publish_args(flag: &str, crates: &[String]) -> Vec<String> {
    let mut args = vec!["publish".to_owned(), "--locked".to_owned(), flag.to_owned()];
    for name in crates {
        args.push("-p".to_owned());
        args.push(name.clone());
    }
    args
}

/// Uploads `missing` with `cargo`, waiting out each rate-limit refusal as
/// [`LONG_ABOUT`] describes, until `pending_of` reports nothing missing.
fn upload(
    mut missing: Vec<String>,
    mut pending_of: impl FnMut() -> Result<Vec<String>>,
    mut cargo: impl FnMut(&[String]) -> Result<Outcome>,
    now: impl Fn() -> DateTime<Utc>,
    mut wait: impl FnMut(Duration),
) -> Result<()> {
    let mut stalled = false;
    loop {
        let outcome = cargo(&publish_args("--no-verify", &missing))?;
        let left = pending_of()?;
        if left.is_empty() {
            return Ok(());
        }
        if outcome.success {
            bail!(
                "`cargo publish` succeeded, but crates.io still lacks {}",
                left.join(", ")
            );
        }
        if !outcome.stderr.contains(RATE_LIMITED) {
            bail!(
                "`cargo publish` failed (its error is above); crates.io still lacks {}",
                left.join(", ")
            );
        }
        let pause = wait_after(&outcome.stderr, now())?;
        eprintln!(
            "crates.io's rate limit: {} still to publish; waiting {} s",
            left.len(),
            pause.as_secs()
        );
        wait(pause);
        // Asked again after the wait: an upload just before the refusal can
        // read as missing at first, and uploading it twice would stop the run.
        let after = pending_of()?;
        if after.is_empty() {
            return Ok(());
        }
        let same = after.len() == missing.len();
        if same && stalled {
            bail!(
                "crates.io refused two uploads in a row that left {} missing",
                after.join(", ")
            );
        }
        stalled = same;
        missing = after;
    }
}

/// How long to wait after a rate-limit refusal printed in `stderr`, read at
/// `now`: until the time it gives plus [`MARGIN`], or [`FALLBACK_WAIT`] when
/// it gives none that can be read.
fn wait_after(stderr: &str, now: DateTime<Utc>) -> Result<Duration> {
    let after = Regex::new(r"try again after ([A-Za-z]{3}, \d{2} [A-Za-z]{3} \d{4} [\d:]{8} GMT)")
        .expect("a valid pattern");
    let Some(until) = after
        .captures(stderr)
        .and_then(|c| DateTime::parse_from_rfc2822(&c[1]).ok())
    else {
        return Ok(FALLBACK_WAIT);
    };
    let ahead = (until.with_timezone(&Utc) - now)
        .to_std()
        .unwrap_or(Duration::ZERO);
    if ahead > LONGEST_WAIT {
        bail!(
            "crates.io asks for a wait of {} s, more than its rate limits explain: {stderr}",
            ahead.as_secs()
        );
    }
    Ok(ahead + MARGIN)
}

/// Runs `cargo args`, passing its output through and keeping its stderr.
fn run_cargo(args: &[String]) -> Result<Outcome> {
    let mut child = Command::new("cargo")
        .args(args)
        .stderr(Stdio::piped())
        .spawn()
        .context("run `cargo`")?;
    let mut stderr = String::new();
    let pipe = child
        .stderr
        .take()
        .context("cargo's stderr was not piped")?;
    let mut reader = BufReader::new(pipe);
    let mut line = Vec::new();
    // Bytes, not `lines()`: a line that is not UTF-8 must not end the read
    // while cargo is still running.
    while reader
        .read_until(b'\n', &mut line)
        .context("read cargo's stderr")?
        > 0
    {
        let text = String::from_utf8_lossy(&line);
        eprint!("{text}");
        stderr.push_str(&text);
        line.clear();
    }
    let status = child.wait().context("wait for `cargo`")?;
    Ok(Outcome {
        success: status.success(),
        stderr,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| (*s).to_owned()).collect()
    }

    fn at(text: &str) -> DateTime<Utc> {
        DateTime::parse_from_rfc3339(text)
            .unwrap()
            .with_timezone(&Utc)
    }

    /// The three shapes crates.io gave on 2026-10-01: `cortenforge` 0.6.0,
    /// `cortenforge` 0.9.0, and a crate that does not exist.
    #[test]
    fn reads_a_version_and_both_absences() {
        let found = r#"{"version":{"id":1942326,"crate":"cortenforge","num":"0.6.0"}}"#;
        assert!(read_status("cortenforge", "0.6.0", "200", found).unwrap());
        let no_version =
            r#"{"errors":[{"detail":"crate `cortenforge` does not have a version `0.9.0`"}]}"#;
        assert!(!read_status("cortenforge", "0.9.0", "404", no_version).unwrap());
        let no_crate = r#"{"errors":[{"detail":"crate `cortenforge-x` does not exist"}]}"#;
        assert!(!read_status("cortenforge-x", "0.9.0", "404", no_crate).unwrap());
    }

    /// Anything else is an error, never "published" or "missing": another
    /// version or crate in a 200, another crate's or version's 404, a 404 for
    /// another reason, an unexpected status, or a body that is not JSON.
    #[test]
    fn refuses_what_it_does_not_expect() {
        let other = r#"{"version":{"crate":"a","num":"0.8.0"}}"#;
        assert!(read_status("a", "0.9.0", "200", other).is_err());
        let other_crate = r#"{"version":{"crate":"b","num":"0.9.0"}}"#;
        assert!(read_status("a", "0.9.0", "200", other_crate).is_err());
        let wrong_name = r#"{"errors":[{"detail":"crate `b` does not exist"}]}"#;
        assert!(read_status("a", "0.9.0", "404", wrong_name).is_err());
        let wrong_version =
            r#"{"errors":[{"detail":"crate `a` does not have a version `0.8.0`"}]}"#;
        assert!(read_status("a", "0.9.0", "404", wrong_version).is_err());
        let not_found = r#"{"errors":[{"detail":"Not Found"}]}"#;
        assert!(read_status("a", "0.9.0", "404", not_found).is_err());
        assert!(read_status("a", "0.9.0", "403", "{}").is_err());
        assert!(read_status("a", "0.9.0", "200", "<html>").is_err());
    }

    #[test]
    fn a_tag_must_name_the_release_version() {
        assert!(check_tag("v0.9.0", "0.9.0").is_ok());
        for tag in ["0.9.0", "v0.9.1", "v0.9.0-rc.1", "release-0.9.0"] {
            assert!(check_tag(tag, "0.9.0").is_err(), "{tag}");
        }
    }

    /// Asks in name order a second apart, keeps what is missing, and stops at
    /// the first error.
    #[test]
    fn keeps_what_crates_io_lacks_in_name_order() {
        let set: BTreeSet<String> = ["c", "a", "b"].iter().map(|s| (*s).to_owned()).collect();
        let mut asked = Vec::new();
        let mut waits = Vec::new();
        let missing = pending(
            &set,
            |name| {
                asked.push(name.to_owned());
                Ok(name == "b")
            },
            |d| waits.push(d),
        )
        .unwrap();
        assert_eq!(missing, names(&["a", "c"]));
        assert_eq!(asked, ["a", "b", "c"]);
        assert_eq!(waits, [Duration::from_secs(1); 2]);
        let mut asked = Vec::new();
        let err = pending(
            &set,
            |name| {
                asked.push(name.to_owned());
                if name == "b" {
                    bail!("down")
                }
                Ok(false)
            },
            |_| {},
        );
        assert!(err.is_err());
        assert_eq!(asked, ["a", "b"]);
    }

    #[test]
    fn asks_for_the_release_version_of_each_crate() {
        let mut urls = Vec::new();
        let found = ask(
            "a",
            "0.9.0",
            |url| {
                urls.push(url.to_owned());
                Ok((
                    "200".to_owned(),
                    r#"{"version":{"crate":"a","num":"0.9.0"}}"#.to_owned(),
                ))
            },
            |_| {},
        )
        .unwrap();
        assert!(found);
        assert_eq!(urls, ["https://crates.io/api/v1/crates/a/0.9.0"]);
    }

    #[test]
    fn publishes_exactly_the_crates_it_is_given() {
        assert_eq!(
            publish_args("--dry-run", &names(&["a", "b"])),
            names(&["publish", "--locked", "--dry-run", "-p", "a", "-p", "b"])
        );
    }

    /// cargo's line for crates.io's refusal, as crates.io words it.
    fn refusal(until: &str) -> String {
        format!(
            "error: failed to publish to registry at https://crates.io\n\nCaused by:\n  the remote \
             server responded with an error (status 429 Too Many Requests): You have published \
             too many new crates in a short period of time. Please try again after {until} and \
             see https://crates.io/docs/rate-limits for more details.\n"
        )
    }

    #[test]
    fn waits_until_the_time_a_refusal_gives() {
        let now = at("2026-10-01T12:00:00Z");
        let wait = wait_after(&refusal("Thu, 01 Oct 2026 12:07:30 GMT"), now).unwrap();
        assert_eq!(wait, Duration::from_secs(450) + MARGIN);
        let past = wait_after(&refusal("Thu, 01 Oct 2026 11:59:00 GMT"), now).unwrap();
        assert_eq!(past, MARGIN);
        let unreadable = wait_after(&refusal("soon"), now).unwrap();
        assert_eq!(unreadable, FALLBACK_WAIT);
        let too_long = wait_after(&refusal("Thu, 01 Oct 2026 13:00:00 GMT"), now);
        assert!(too_long.is_err());
    }

    /// Replays `outcomes` as cargo's runs and `left` as crates.io's answers
    /// after each run and after each wait, recording each run's crates and
    /// each wait.
    fn replay(
        start: &[&str],
        outcomes: Vec<(bool, String)>,
        left: Vec<Vec<&str>>,
    ) -> (Result<()>, Vec<Vec<String>>, Vec<Duration>) {
        let mut runs = Vec::new();
        let mut waits = Vec::new();
        let mut outcomes = outcomes.into_iter();
        let mut left = left.into_iter();
        let got = upload(
            names(start),
            || Ok(names(&left.next().expect("asked more often than expected"))),
            |args| {
                runs.push(args.to_vec());
                let (success, stderr) =
                    outcomes.next().expect("ran cargo more often than expected");
                Ok(Outcome { success, stderr })
            },
            || at("2026-10-01T12:00:00Z"),
            |d| waits.push(d),
        );
        (got, runs, waits)
    }

    #[test]
    fn waits_out_a_refusal_then_uploads_what_is_left() {
        let limit = refusal("Thu, 01 Oct 2026 12:10:00 GMT");
        let (got, runs, waits) = replay(
            &["a", "b", "c"],
            vec![(false, limit), (true, String::new())],
            vec![vec!["c"], vec!["c"], vec![]],
        );
        got.unwrap();
        assert_eq!(
            runs,
            [
                publish_args("--no-verify", &names(&["a", "b", "c"])),
                publish_args("--no-verify", &names(&["c"])),
            ]
        );
        assert_eq!(waits, [Duration::from_secs(600) + MARGIN]);
    }

    /// A crate uploaded just before the refusal can still read as missing
    /// right after it; what is uploaded next is what crates.io says after the
    /// wait.
    #[test]
    fn uploads_what_is_missing_after_the_wait() {
        let limit = refusal("Thu, 01 Oct 2026 12:10:00 GMT");
        let (got, runs, _) = replay(
            &["a", "b"],
            vec![(false, limit), (true, String::new())],
            vec![vec!["a", "b"], vec!["b"], vec![]],
        );
        got.unwrap();
        assert_eq!(runs[1], publish_args("--no-verify", &names(&["b"])));
        // Nothing missing after the wait is done, with no second run.
        let limit = refusal("Thu, 01 Oct 2026 12:10:00 GMT");
        let (got, runs, waits) = replay(&["a"], vec![(false, limit)], vec![vec!["a"], vec![]]);
        got.unwrap();
        assert_eq!((runs.len(), waits.len()), (1, 1));
        // Two refusals in a row that each published a crate, which read as
        // missing until the wait, are progress, not a stall.
        let limit = refusal("Thu, 01 Oct 2026 12:10:00 GMT");
        let (got, runs, _) = replay(
            &["a", "b", "c"],
            vec![
                (false, limit.clone()),
                (false, limit),
                (true, String::new()),
            ],
            vec![
                vec!["a", "b", "c"],
                vec!["b", "c"],
                vec!["b", "c"],
                vec!["c"],
                vec![],
            ],
        );
        got.unwrap();
        assert_eq!(runs.len(), 3);
    }

    #[test]
    fn any_other_failure_stops_without_waiting() {
        let (got, runs, waits) = replay(
            &["a", "b"],
            vec![(false, "error: invalid keyword\n".to_owned())],
            vec![vec!["b"]],
        );
        assert!(got.unwrap_err().to_string().contains("still lacks b"));
        assert_eq!((runs.len(), waits.len()), (1, 0));
    }

    #[test]
    fn two_refusals_that_publish_nothing_stop_it() {
        let limit = refusal("Thu, 01 Oct 2026 12:01:00 GMT");
        let (got, runs, waits) = replay(
            &["a", "b"],
            vec![(false, limit.clone()), (false, limit)],
            vec![vec!["a", "b"]; 4],
        );
        assert!(got
            .unwrap_err()
            .to_string()
            .contains("two uploads in a row"));
        assert_eq!((runs.len(), waits.len()), (2, 2));
        // One that publishes something in between resets the count.
        let limit = refusal("Thu, 01 Oct 2026 12:01:00 GMT");
        let (got, runs, _) = replay(
            &["a", "b", "c"],
            vec![
                (false, limit.clone()),
                (false, limit.clone()),
                (false, limit),
                (true, String::new()),
            ],
            vec![
                vec!["a", "b", "c"],
                vec!["a", "b", "c"],
                vec!["b", "c"],
                vec!["b", "c"],
                vec!["b", "c"],
                vec!["b", "c"],
                vec![],
            ],
        );
        got.unwrap();
        assert_eq!(runs.len(), 4);
    }

    /// What decides is what crates.io has, not how cargo exited: a failure
    /// after the last upload is done, and a success that left a crate out is
    /// not.
    #[test]
    fn crates_io_decides_when_it_is_done() {
        let (got, _, waits) = replay(
            &["a"],
            vec![(false, "error: timed out waiting for `a`\n".to_owned())],
            vec![vec![]],
        );
        got.unwrap();
        assert!(waits.is_empty());
        let (got, _, _) = replay(&["a", "b"], vec![(true, String::new())], vec![vec!["b"]]);
        assert!(got.unwrap_err().to_string().contains("still lacks b"));
    }

    /// Only the two upload modes upload, and only the full publish does both
    /// steps; the flags pick the mode with `--list` first.
    #[test]
    fn only_the_upload_modes_upload() {
        assert_eq!(Mode::List.steps(), (false, false));
        assert_eq!(Mode::DryRun.steps(), (true, false));
        assert_eq!(Mode::NoVerify.steps(), (false, true));
        assert_eq!(Mode::Publish.steps(), (true, true));
        assert_eq!(Mode::from_flags(false, false, false), Mode::Publish);
        assert_eq!(Mode::from_flags(true, false, false), Mode::List);
        assert_eq!(Mode::from_flags(false, true, false), Mode::DryRun);
        assert_eq!(Mode::from_flags(false, false, true), Mode::NoVerify);
        assert_eq!(Mode::from_flags(true, true, true), Mode::List);
    }

    /// Runs `mode` over `missing` with a cargo whose dry-run passes or fails
    /// as `verifies`, recording cargo's runs and what reached the upload.
    fn steps_taken(
        mode: Mode,
        missing: &[&str],
        verifies: bool,
    ) -> (Result<bool>, Vec<Vec<String>>, Vec<Vec<String>>) {
        let mut runs = Vec::new();
        let mut uploads = Vec::new();
        let got = verify_then_upload(
            mode,
            names(missing),
            |args| {
                runs.push(args.to_vec());
                Ok(Outcome {
                    success: verifies,
                    stderr: String::new(),
                })
            },
            |missing| {
                uploads.push(missing);
                Ok(())
            },
        );
        (got, runs, uploads)
    }

    /// The irreversible step is reached only after a verify that passed, or
    /// by `--no-verify`; `--dry-run` never reaches it, and nothing missing
    /// runs nothing.
    #[test]
    fn a_failed_verify_uploads_nothing() {
        let dry = publish_args("--dry-run", &names(&["a"]));
        let (got, runs, uploads) = steps_taken(Mode::Publish, &["a"], false);
        assert!(got.is_err());
        assert_eq!((runs, uploads.len()), (vec![dry.clone()], 0));
        let (got, _, uploads) = steps_taken(Mode::Publish, &["a"], true);
        assert!(got.unwrap());
        assert_eq!(uploads, [names(&["a"])]);
        let (got, runs, uploads) = steps_taken(Mode::DryRun, &["a"], true);
        assert!(!got.unwrap());
        assert_eq!((runs, uploads.len()), (vec![dry], 0));
        let (got, runs, uploads) = steps_taken(Mode::NoVerify, &["a"], false);
        assert!(got.unwrap());
        assert_eq!((runs.len(), uploads), (0, vec![names(&["a"])]));
        let (got, runs, uploads) = steps_taken(Mode::Publish, &[], true);
        assert!(!got.unwrap());
        assert_eq!((runs.len(), uploads.len()), (0, 0));
    }

    /// The help states the limits this code acts on.
    #[test]
    fn the_help_matches_the_code() {
        let help = LONG_ABOUT.split_whitespace().collect::<Vec<_>>().join(" ");
        assert!(help.contains(&format!(
            "more than {} minutes",
            LONGEST_WAIT.as_secs() / 60
        )));
        assert!(help.contains(&format!("1 every {} minutes", FALLBACK_WAIT.as_secs() / 60)));
        assert!(help.contains("cargo publish --locked --dry-run"));
        assert!(help.contains("cargo publish --locked --no-verify"));
    }
}
