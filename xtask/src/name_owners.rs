//! Self-test: each name in the crates.io release set is ours, not on crates.io,
//! or listed as held by another account.
//!
//! [`LONG_ABOUT`] states the rules and what the check cannot see; [`FOREIGN`]
//! is the list of names another account holds.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::process::Command;
use std::thread::sleep;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use serde_json::Value;

/// Shown by `cargo xtask name-owners --help`. The single statement of what this
/// asserts; CI's comments point here rather than restating it.
pub const LONG_ABOUT: &str = "\
Assert that each name in the crates.io release set (the `cortenforge` facade's
closure, as `cargo xtask publish-set` defines it) is ours, not on crates.io, or
listed in FOREIGN as held by another account. For each name it asks crates.io
who owns that crate.

The rules:
  - a name not on crates.io passes;
  - a crate on crates.io passes when it has an owner and every owner is in
    OURS;
  - any other crate fails unless its name is in FOREIGN;
  - every FOREIGN entry is still a release-set name on crates.io that is not
    ours, so the list cannot go stale.

It fails, rather than passes, on an answer it does not expect: an HTTP status
other than 200 or 404, a 404 that does not say the crate does not exist, or a
body it cannot read. A failed request, or a status of 429 or 5xx, is retried
up to 3 times.

What it cannot see:
  - a name not on crates.io is free only at that moment: anyone can publish it
    before we do, and crates.io refuses a name on its reserved list at publish;
  - crates.io treats `-` and `_` as one name and answers for whichever spelling
    exists; whether it accepts a publish under the other spelling is not
    checked;
  - names that are not in the release set.

Needs the network and `curl`: one request per name, a second apart, as
crates.io's data access policy asks.";

/// The crates.io accounts that are us. A crate is ours when it has an owner and
/// every owner is one of these.
const OURS: [&str; 1] = ["bigmark222"];

/// Release-set names on crates.io that are not ours: another account holds
/// each today.
const FOREIGN: [&str; 0] = [];

/// Tries per name, the first included.
const ATTEMPTS: u32 = 4;

/// Identifies this check to crates.io, as its data access policy asks.
const USER_AGENT: &str =
    "CortenForge name-owners check (https://github.com/via-balaena/CortenForge)";

/// What crates.io says about one name.
#[derive(Debug, PartialEq, Eq)]
enum Answer {
    /// No crate by that name.
    Absent,
    /// The crate's owners, by login (a team's login is `github:org:team`).
    Owned(BTreeSet<String>),
}

/// Asks crates.io about every name in the release set and fails on any
/// departure from the rules in [`LONG_ABOUT`].
///
/// # Errors
///
/// When it finds a problem, or crates.io gives an answer it does not expect.
pub fn check() -> Result<()> {
    let set = crate::publish_set::release_set(Path::new("."))?;
    let answers = ask_all(&set, |name| ask(name, curl, sleep), sleep)?;
    println!("{}", verdict(&answers, &FOREIGN)?);
    Ok(())
}

/// The line to print for `answers`, or an error listing every problem.
fn verdict(answers: &BTreeMap<String, Answer>, foreign: &[&str]) -> Result<String> {
    let problems = problems(answers, foreign);
    if !problems.is_empty() {
        let lines: Vec<String> = problems.iter().map(|p| format!("  ✗ {p}")).collect();
        bail!(
            "{} problem(s) with who holds the crates.io release set's names (see `cargo xtask name-owners --help`):\n{}",
            problems.len(),
            lines.join("\n")
        );
    }
    let ours = answers
        .values()
        .filter(|answer| matches!(answer, Answer::Owned(owners) if is_ours(owners)))
        .count();
    let absent = answers
        .values()
        .filter(|answer| **answer == Answer::Absent)
        .count();
    Ok(format!(
        "✓ the {} release-set names: {ours} ours, {absent} not on crates.io, {} not ours and listed ({})",
        answers.len(),
        foreign.len(),
        foreign.join(", "),
    ))
}

/// One answer per name, in name order, with a `wait` of a second between
/// names.
fn ask_all(
    set: &BTreeSet<String>,
    mut ask: impl FnMut(&str) -> Result<Answer>,
    mut wait: impl FnMut(Duration),
) -> Result<BTreeMap<String, Answer>> {
    let mut answers = BTreeMap::new();
    for (i, name) in set.iter().enumerate() {
        if i > 0 {
            wait(Duration::from_secs(1));
        }
        answers.insert(name.clone(), ask(name)?);
    }
    Ok(answers)
}

/// Asks crates.io who owns `name` with `get`, retrying a failed request or a
/// 429/5xx after a `wait` that doubles each time.
fn ask(
    name: &str,
    mut get: impl FnMut(&str) -> Result<(String, String)>,
    mut wait: impl FnMut(Duration),
) -> Result<Answer> {
    let url = format!("https://crates.io/api/v1/crates/{name}/owners");
    let mut last = String::new();
    for attempt in 0..ATTEMPTS {
        if attempt > 0 {
            wait(Duration::from_secs(2_u64 << attempt));
        }
        let (status, body) = get(&url)?;
        if !transient(&status) {
            return read_answer(name, &status, &body);
        }
        last = if status == "000" {
            body
        } else {
            format!("HTTP {status}")
        };
    }
    bail!("crates.io did not answer for `{name}` in {ATTEMPTS} tries: {last}")
}

/// One GET of `url`: the HTTP status and the body. A request that never got an
/// answer comes back as status `000` with curl's error as the body.
fn curl(url: &str) -> Result<(String, String)> {
    let out = Command::new("curl")
        .args([
            "--silent",
            "--show-error",
            "--max-time",
            "30",
            "--user-agent",
            USER_AGENT,
            "--write-out",
            "\n%{http_code}",
            url,
        ])
        .output()
        .context("run `curl`, which this check needs to ask crates.io")?;
    if !out.status.success() {
        return Ok((
            "000".into(),
            String::from_utf8_lossy(&out.stderr).trim().into(),
        ));
    }
    let text = String::from_utf8(out.stdout).context("crates.io sent non-UTF-8")?;
    let (body, status) = text
        .rsplit_once('\n')
        .with_context(|| format!("curl printed no status line for {url}"))?;
    Ok((status.trim().into(), body.into()))
}

/// Worth asking again: no answer at all, rate limited, or a server error.
fn transient(status: &str) -> bool {
    status == "000" || status == "429" || status.starts_with('5')
}

/// Reads one answer, refusing anything but a list of owners (200) or a 404
/// that says the crate does not exist.
fn read_answer(name: &str, status: &str, body: &str) -> Result<Answer> {
    let json = || -> Result<Value> {
        serde_json::from_str(body)
            .with_context(|| format!("crates.io's answer for `{name}` is not JSON: {body}"))
    };
    match status {
        "200" => {
            let users = json()?
                .get("users")
                .and_then(Value::as_array)
                .cloned()
                .with_context(|| format!("crates.io's answer for `{name}` has no `users` list"))?;
            users
                .iter()
                .map(|user| {
                    user.get("login")
                        .and_then(Value::as_str)
                        .map(str::to_owned)
                        .with_context(|| format!("an owner of `{name}` has no login: {user}"))
                })
                .collect::<Result<_>>()
                .map(Answer::Owned)
        }
        "404" => {
            let detail = json()?
                .pointer("/errors/0/detail")
                .and_then(Value::as_str)
                .map(str::to_owned);
            if detail.as_deref() == Some(&format!("crate `{name}` does not exist")) {
                Ok(Answer::Absent)
            } else {
                bail!("crates.io answered 404 for `{name}` without saying the crate does not exist: {body}")
            }
        }
        _ => bail!("crates.io answered HTTP {status} for `{name}`: {body}"),
    }
}

/// Every way `answers` departs from the rules in [`LONG_ABOUT`], given
/// `foreign`.
fn problems(answers: &BTreeMap<String, Answer>, foreign: &[&str]) -> Vec<String> {
    let mut out = Vec::new();
    for (name, answer) in answers {
        let listed = foreign.contains(&name.as_str());
        match answer {
            Answer::Absent if listed => out.push(format!(
                "`{name}` is listed in `FOREIGN` but is not on crates.io: remove its entry"
            )),
            Answer::Owned(owners) if is_ours(owners) && listed => out.push(format!(
                "`{name}` is listed in `FOREIGN` but every owner is ours: remove its entry"
            )),
            Answer::Owned(owners) if !is_ours(owners) && !listed => out.push(format!(
                "`{name}` is held on crates.io by {}: add it to `FOREIGN` in xtask/src/name_owners.rs, or take it out of the release set",
                if owners.is_empty() {
                    "no owner at all".to_owned()
                } else {
                    owners.iter().cloned().collect::<Vec<_>>().join(", ")
                }
            )),
            _ => {}
        }
    }
    for name in foreign {
        if !answers.contains_key(*name) {
            out.push(format!(
                "`{name}` is listed in `FOREIGN` but is not a release-set name: remove its entry"
            ));
        }
    }
    out
}

/// Every owner is in [`OURS`], and there is at least one.
fn is_ours(owners: &BTreeSet<String>) -> bool {
    !owners.is_empty() && owners.iter().all(|owner| OURS.contains(&owner.as_str()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn owned(logins: &[&str]) -> Answer {
        Answer::Owned(logins.iter().map(|login| (*login).to_owned()).collect())
    }

    fn answers(pairs: Vec<(&str, Answer)>) -> BTreeMap<String, Answer> {
        pairs
            .into_iter()
            .map(|(name, answer)| (name.to_owned(), answer))
            .collect()
    }

    /// The three shapes crates.io gives, as it gave them on 2026-09-30.
    #[test]
    fn reads_owners_and_absence() {
        let user = r#"{"users":[{"kind":"user","id":1,"login":"bigmark222","name":"x"}]}"#;
        assert_eq!(
            read_answer("a", "200", user).unwrap(),
            owned(&["bigmark222"])
        );
        let team = r#"{"users":[{"kind":"team","login":"github:org:team"},{"kind":"user","login":"bigmark222"}]}"#;
        assert_eq!(
            read_answer("a", "200", team).unwrap(),
            owned(&["bigmark222", "github:org:team"])
        );
        let absent = r#"{"errors":[{"detail":"crate `zz-x` does not exist"}]}"#;
        assert_eq!(read_answer("zz-x", "404", absent).unwrap(), Answer::Absent);
    }

    /// Anything else is an error, never a pass: a 404 for another reason,
    /// another name's 404, an unexpected status, or a body that is not the
    /// expected JSON.
    #[test]
    fn refuses_what_it_does_not_expect() {
        let other_404 = r#"{"errors":[{"detail":"Not Found"}]}"#;
        assert!(read_answer("a", "404", other_404).is_err());
        let wrong_name = r#"{"errors":[{"detail":"crate `b` does not exist"}]}"#;
        assert!(read_answer("a", "404", wrong_name).is_err());
        assert!(read_answer("a", "404", "<html>").is_err());
        assert!(read_answer("a", "403", "{}").is_err());
        assert!(read_answer("a", "301", "").is_err());
        assert!(read_answer("a", "200", "<html>").is_err());
        assert!(read_answer("a", "200", r#"{"crate":{}}"#).is_err());
        assert!(read_answer("a", "200", r#"{"users":[{"kind":"user"}]}"#).is_err());
    }

    #[test]
    fn retries_only_what_can_pass_on_a_second_try() {
        for status in ["000", "429", "500", "502", "503", "504", "520"] {
            assert!(transient(status), "{status}");
        }
        for status in ["200", "404", "403", "301", "400"] {
            assert!(!transient(status), "{status}");
        }
    }

    #[test]
    fn a_name_we_hold_or_nobody_holds_passes() {
        let got = answers(vec![("a", owned(&["bigmark222"])), ("b", Answer::Absent)]);
        assert_eq!(problems(&got, &[]), Vec::<String>::new());
    }

    /// Another owner beside us can publish the name too, and a crate with no
    /// owner is not ours.
    #[test]
    fn a_name_anyone_else_holds_fails_unless_listed() {
        let got = answers(vec![
            ("alone", owned(&["someone"])),
            ("shared", owned(&["bigmark222", "someone"])),
            ("orphan", owned(&[])),
        ]);
        let found = problems(&got, &[]);
        assert_eq!(found.len(), 3, "{found:?}");
        assert!(
            found.iter().all(|p| p.contains("add it to `FOREIGN`")),
            "{found:?}"
        );
        assert!(
            found.iter().any(|p| p.contains("no owner at all")),
            "{found:?}"
        );
        assert_eq!(
            problems(&got, &["alone", "shared", "orphan"]),
            Vec::<String>::new()
        );
    }

    /// What `check` prints, or the error it returns.
    #[test]
    fn the_verdict_counts_a_clean_answer_and_lists_every_problem() {
        let clean = answers(vec![
            ("a", owned(&["bigmark222"])),
            ("b", owned(&["bigmark222"])),
            ("c", owned(&["bigmark222"])),
            ("d", Answer::Absent),
            ("e", owned(&["someone"])),
            ("f", owned(&["another"])),
        ]);
        assert_eq!(
            verdict(&clean, &["e", "f"]).unwrap(),
            "✓ the 6 release-set names: 3 ours, 1 not on crates.io, 2 not ours and listed (e, f)"
        );
        let err = verdict(&clean, &[]).unwrap_err().to_string();
        assert!(err.starts_with("2 problem(s)"), "{err}");
        assert!(
            err.contains("\n  ✗ `e` is held on crates.io by someone"),
            "{err}"
        );
        assert!(
            err.contains("\n  ✗ `f` is held on crates.io by another"),
            "{err}"
        );
    }

    #[test]
    fn a_stale_listing_fails() {
        let got = answers(vec![
            ("ours", owned(&["bigmark222"])),
            ("free", Answer::Absent),
        ]);
        let found = problems(&got, &["ours", "free", "gone"]);
        assert_eq!(found.len(), 3, "{found:?}");
        assert!(found[0].contains("`free`") && found[0].contains("not on crates.io"));
        assert!(found[1].contains("`ours`") && found[1].contains("every owner is ours"));
        assert!(found[2].contains("`gone`") && found[2].contains("not a release-set name"));
    }

    /// The asker is called once per name, in order, a second apart, and its
    /// first error stops the walk.
    #[test]
    fn asks_once_per_name_and_stops_on_an_error() {
        let set: BTreeSet<String> = ["b", "a", "c"].iter().map(|s| (*s).to_owned()).collect();
        let mut asked = Vec::new();
        let mut waits = Vec::new();
        let got = ask_all(
            &set,
            |name| {
                asked.push(name.to_owned());
                Ok(Answer::Absent)
            },
            |d| waits.push(d),
        )
        .unwrap();
        assert_eq!(asked, ["a", "b", "c"]);
        assert_eq!(waits, [Duration::from_secs(1); 2]);
        assert_eq!(got.len(), 3);
        let mut asked = Vec::new();
        let err = ask_all(
            &set,
            |name| {
                asked.push(name.to_owned());
                if name == "b" {
                    bail!("down")
                }
                Ok(Answer::Absent)
            },
            |_| {},
        );
        assert!(err.is_err());
        assert_eq!(asked, ["a", "b"]);
    }

    /// Replays `replies` as curl's answers, recording each URL and wait.
    fn replay(
        name: &str,
        replies: &[(&str, &str)],
    ) -> (Result<Answer>, Vec<String>, Vec<Duration>) {
        let mut urls = Vec::new();
        let mut waits = Vec::new();
        let mut next = replies.iter();
        let got = ask(
            name,
            |url| {
                urls.push(url.to_owned());
                let (status, body) = next.next().expect("asked more often than expected");
                Ok(((*status).to_owned(), (*body).to_owned()))
            },
            |d| waits.push(d),
        );
        (got, urls, waits)
    }

    #[test]
    fn retries_a_failed_request_and_a_server_error_then_reads_the_answer() {
        let owners = r#"{"users":[{"login":"bigmark222"}]}"#;
        let (got, urls, waits) = replay(
            "a",
            &[("000", "curl: (7) no route"), ("503", ""), ("200", owners)],
        );
        assert_eq!(got.unwrap(), owned(&["bigmark222"]));
        assert_eq!(urls, ["https://crates.io/api/v1/crates/a/owners"; 3]);
        assert_eq!(waits, [Duration::from_secs(4), Duration::from_secs(8)]);
    }

    #[test]
    fn gives_up_after_its_attempts() {
        let (got, urls, waits) = replay("a", &[("000", "curl: (7) no route"); 4]);
        let err = got.unwrap_err().to_string();
        assert!(
            err.contains("4 tries") && err.contains("curl: (7) no route"),
            "{err}"
        );
        assert_eq!(urls.len(), 4);
        assert_eq!(waits, [4, 8, 16].map(Duration::from_secs));
        let (got, _, _) = replay("a", &[("429", ""); 4]);
        assert!(got.unwrap_err().to_string().contains("HTTP 429"));
    }

    /// An answer that is not worth asking again is read at once, even when it
    /// is an error.
    #[test]
    fn does_not_retry_an_answer() {
        let (got, urls, waits) = replay("a", &[("404", r#"{"errors":[{"detail":"Not Found"}]}"#)]);
        assert!(got.is_err());
        assert_eq!((urls.len(), waits.len()), (1, 0));
        let mut calls = 0;
        let got = ask(
            "a",
            |_| {
                calls += 1;
                bail!("no curl")
            },
            |_| {},
        );
        assert!(got.is_err());
        assert_eq!(calls, 1);
    }

    /// The help states the retry count and the two lists by their code names.
    #[test]
    fn the_help_matches_the_code() {
        let help = LONG_ABOUT.split_whitespace().collect::<Vec<_>>().join(" ");
        assert!(help.contains(&format!("up to {} times", ATTEMPTS - 1)));
        assert!(help.contains("passes when it has an owner and every owner is in OURS"));
        assert!(help.contains("a release-set name on crates.io that is not ours"));
        assert!(help.contains("unless its name is in FOREIGN"));
    }

    /// Each `FOREIGN` entry is a release-set name; the live check asserts that
    /// too, and this catches a typo without the network.
    #[test]
    fn foreign_names_are_release_set_names() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let set = crate::publish_set::release_set(&root).unwrap();
        for name in FOREIGN {
            assert!(set.contains(name), "{name}");
        }
    }
}
