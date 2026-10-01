//! Asking crates.io's API: one GET at a time with `curl`, retried while the
//! answer could still change. `name-owners` and `publish` both ask through it.

use std::process::Command;
use std::time::Duration;

use anyhow::{bail, Context, Result};

/// Tries per request, the first included.
pub(crate) const ATTEMPTS: u32 = 4;

/// One GET of `url`, identified as `user_agent`: the HTTP status and the body.
/// A request that never got an answer comes back as status `000` with curl's
/// error as the body.
pub(crate) fn curl(url: &str, user_agent: &str) -> Result<(String, String)> {
    let out = Command::new("curl")
        .args([
            "--silent",
            "--show-error",
            "--max-time",
            "30",
            "--user-agent",
            user_agent,
            "--write-out",
            "\n%{http_code}",
            url,
        ])
        .output()
        .context("run `curl`, which is how this asks crates.io")?;
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
pub(crate) fn transient(status: &str) -> bool {
    status == "000" || status == "429" || status.starts_with('5')
}

/// GETs `url` with `get`, retrying a failed request or a 429/5xx after a
/// `wait` that doubles each time, and returns the first answer that is not
/// worth asking again. `what` names the request in the error.
pub(crate) fn get_retrying(
    url: &str,
    what: &str,
    mut get: impl FnMut(&str) -> Result<(String, String)>,
    mut wait: impl FnMut(Duration),
) -> Result<(String, String)> {
    let mut last = String::new();
    for attempt in 0..ATTEMPTS {
        if attempt > 0 {
            wait(Duration::from_secs(2_u64 << attempt));
        }
        let (status, body) = get(url)?;
        if !transient(&status) {
            return Ok((status, body));
        }
        last = if status == "000" {
            body
        } else {
            format!("HTTP {status}")
        };
    }
    bail!("crates.io did not answer for `{what}` in {ATTEMPTS} tries: {last}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retries_only_what_can_pass_on_a_second_try() {
        for status in ["000", "429", "500", "502", "503", "504", "520"] {
            assert!(transient(status), "{status}");
        }
        for status in ["200", "404", "403", "301", "400"] {
            assert!(!transient(status), "{status}");
        }
    }
}
