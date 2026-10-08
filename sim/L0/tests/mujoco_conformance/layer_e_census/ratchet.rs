//! Verdicts, their classes, and the ratchet over the expected-verdict file.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::Path;

use serde_json::Value;

use super::compare::{COUNT_FIELDS, dump_diff, model_diff, state_differs};

/// The first difference within one excitation: the earliest step, and at
/// that step a differing state before a differing `forward` dump.
fn dynamics_verdict(golden: &Value, ours: &Value) -> String {
    for key in ["e1", "e2"] {
        let (g, o) = (&golden[key], &ours[key]);
        // (step, rank within the step, label)
        let mut events: Vec<(usize, u8, String)> = Vec::new();
        if let Some(q) = dump_diff(&o["dump"]["0"], &g["dump"]["0"]).first() {
            events.push((0, 1, format!("t0:{q}")));
        }
        let checkpoints: Vec<usize> = g["traj_steps"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(Value::as_u64)
                    .filter_map(|s| usize::try_from(s).ok())
                    .collect()
            })
            .unwrap_or_default();
        let our_steps = o["traj"]["q"].as_array().map_or(0, Vec::len);
        for (i, &step) in checkpoints.iter().enumerate() {
            if step > our_steps {
                events.push((step, 0, format!("steperr@{step}")));
                break;
            }
            if state_differs(&o["traj"], &g["traj"], step - 1, i) {
                events.push((step, 0, format!("state@{step}")));
                break;
            }
        }
        if let Value::Object(dumps) = &g["dump"] {
            for (k, gd) in dumps {
                let step: usize = k.parse().unwrap_or(0);
                if step == 0 {
                    continue;
                }
                match o["dump"].get(k) {
                    None => events.push((step, 1, format!("s{k}:missing"))),
                    Some(od) => {
                        if let Some(q) = dump_diff(od, gd).first() {
                            events.push((step, 1, format!("s{k}:{q}")));
                        }
                    }
                }
            }
        }
        if let Some((_, _, label)) = events.into_iter().min_by_key(|(s, r, _)| (*s, *r)) {
            return format!("{key}:{label}");
        }
    }
    "agree".to_string()
}

/// One doc's verdict: a panic in our loader (`ours-panic`, whatever MuJoCo
/// does), the load statuses, then the first differing model field, then the
/// first difference in the dynamics. A model-differing doc whose counts agree
/// carries its dynamics verdict too (`model:geom_quat;dyn:agree`), so its
/// dynamics are pinned as well.
pub fn verdict(golden: &Value, ours: &Value) -> String {
    if ours["status"] == "panic" {
        return "ours-panic".to_string();
    }
    match (golden["status"] == "ok", ours["status"] == "ok") {
        (false, false) => return "both-refuse".to_string(),
        (false, true) => return "mj-refuses".to_string(),
        (true, false) => return format!("ours-{}", ours["status"].as_str().unwrap_or("?")),
        (true, true) => {}
    }
    let fields = model_diff(&ours["model"], &golden["model"]);
    let Some(first) = fields.first() else {
        return dynamics_verdict(golden, ours);
    };
    if fields.iter().any(|f| COUNT_FIELDS.contains(&f.as_str())) {
        format!("model:{first}")
    } else {
        format!("model:{first};dyn:{}", dynamics_verdict(golden, ours))
    }
}

/// The part of a verdict the ratchet pins. The rest of a verdict — which
/// step or quantity differs first — is a label: recorded on bless, never
/// failed on, because last-bit perturbations move it (A20 §2.4).
fn class(v: &str) -> String {
    if v == "agree" || v.starts_with("ours-") || v == "mj-refuses" || v == "both-refuse" {
        return v.to_string();
    }
    if let Some(rest) = v.strip_prefix("model:") {
        let field = rest.split(';').next().unwrap_or("");
        let dynamics = if rest.ends_with(";dyn:agree") {
            "dyn-agree"
        } else {
            "dyn-differs"
        };
        return format!("model:{field};{dynamics}");
    }
    "dyn".to_string()
}

/// Rank within MuJoCo's fixed status: a move up is an improvement, a move
/// down a regression. Golden ok: agree > model with agreeing dynamics >
/// differing dynamics > ours refused. Golden refused: both refuse > MuJoCo refuses.
fn rank(class: &str) -> u8 {
    match class {
        "agree" => 5,
        "both-refuse" => 4,
        c if c.starts_with("model:") && c.ends_with(";dyn-agree") => 3,
        "mj-refuses" => 1,
        c if c.starts_with("ours-") => 0,
        _ => 2,
    }
}

/// A row of the expected-verdict file: `doc<TAB>verdict[<TAB>note]`.
#[derive(Clone)]
pub struct Row {
    pub verdict: String,
    pub note: String,
}

/// The expected-verdict file: its rows and the agree floor (`# agree_floor N`).
pub struct Expected {
    pub floor: usize,
    pub rows: BTreeMap<String, Row>,
}

const HEADER: &str = "\
# Parity-census verdicts against MuJoCo 3.5.0: one row per snapshot doc, `doc<TAB>verdict[<TAB>note]`.
# Written by mujoco_conformance/layer_e_census.rs with CENSUS_BLESS=1; read its module doc before editing.
";

pub fn read_expected(path: &Path) -> Expected {
    let text = std::fs::read_to_string(path).unwrap_or_default();
    let mut floor = 0;
    let mut rows = BTreeMap::new();
    for line in text.lines() {
        if let Some(n) = line.strip_prefix("# agree_floor ") {
            floor = n.trim().parse().unwrap_or(0);
        } else if !line.starts_with('#') && !line.trim().is_empty() {
            let mut cols = line.split('\t');
            let doc = cols.next().unwrap_or_default().to_string();
            let verdict = cols.next().unwrap_or_default().to_string();
            let note = cols.next().unwrap_or_default().to_string();
            rows.insert(doc, Row { verdict, note });
        }
    }
    Expected { floor, rows }
}

fn write_expected(path: &Path, rows: &BTreeMap<String, Row>) {
    let agree = rows
        .values()
        .filter(|r| r.verdict == "agree" && !r.note.starts_with("nondet="))
        .count();
    let mut s = format!("{HEADER}# agree_floor {agree}\n");
    for (doc, r) in rows {
        if r.note.is_empty() {
            let _ = writeln!(s, "{doc}\t{}", r.verdict);
        } else {
            let _ = writeln!(s, "{doc}\t{}\t{}", r.verdict, r.note);
        }
    }
    std::fs::write(path, s).unwrap_or_else(|e| panic!("write {}: {e}", path.display()));
}

/// Whether `c` is a class (what [`class`] returns), not a verdict.
fn is_class(c: &str) -> bool {
    if c.contains(char::is_whitespace) {
        return false;
    }
    matches!(c, "agree" | "dyn" | "mj-refuses" | "both-refuse")
        || c.strip_prefix("ours-").is_some_and(|s| !s.is_empty())
        || c.strip_prefix("model:")
            .is_some_and(|r| r.ends_with(";dyn-agree") || r.ends_with(";dyn-differs"))
}

/// Whether `id` names a commit of the Rigid series: `P` or `L`, digits, and
/// at most one lowercase letter (`P08a`).
fn is_commit_id(id: &str) -> bool {
    let Some(rest) = id.strip_prefix(['P', 'L']) else {
        return false;
    };
    let digits = rest.trim_end_matches(|c: char| c.is_ascii_lowercase());
    !digits.is_empty()
        && digits.bytes().all(|b| b.is_ascii_digit())
        && rest.len() - digits.len() <= 1
}

/// A `fixed_by=<commit> was=<class>` note: the commit that fixes a regressed
/// doc, and the class the doc had before it regressed. `None` when malformed.
fn fixed_by(note: &str) -> Option<(&str, &str)> {
    let (commit, was) = note.strip_prefix("fixed_by=")?.split_once(" was=")?;
    (is_commit_id(commit) && is_class(was)).then_some((commit, was))
}

/// The divergence registry's IDs (the first column of `divergences.tsv`).
pub fn read_divergence_ids(path: &Path) -> BTreeSet<String> {
    std::fs::read_to_string(path)
        .unwrap_or_default()
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .skip(1) // the column header
        .filter_map(|l| l.split('\t').next().map(str::to_string))
        .collect()
}

/// A verdict that moved: `(doc, expected, got)`.
pub type Move = (String, String, String);

#[derive(Default)]
pub struct Report {
    pub agree: usize,
    pub floor: usize,
    pub improvements: Vec<Move>,
    pub regressions: Vec<Move>,
    /// A class change of equal rank.
    pub shifts: Vec<Move>,
    /// Only the label changed; reported, never failed on.
    pub label_shifts: Vec<Move>,
    /// A `divergence=` doc whose class changed: the deliberate difference was
    /// lost or became another one.
    pub lost_divergences: Vec<String>,
    /// A note the gate cannot accept, with the reason.
    pub bad_notes: Vec<(String, String)>,
    /// Golden docs with no row.
    pub new_docs: Vec<String>,
    /// Rows with no golden doc.
    pub missing: Vec<String>,
}

impl Report {
    /// Whether something fails that blessing cannot record.
    pub fn blocked(&self) -> bool {
        !(self.regressions.is_empty()
            && self.lost_divergences.is_empty()
            && self.bad_notes.is_empty()
            && self.missing.is_empty())
    }

    /// Whether the file already records every verdict and the floor holds.
    pub fn passes(&self) -> bool {
        !self.blocked()
            && self.improvements.is_empty()
            && self.shifts.is_empty()
            && self.new_docs.is_empty()
            && self.agree >= self.floor
    }
}

/// Compare every doc's verdict (`got`) with the expected file and, when
/// `bless`, rewrite the file for improvements, shifts, label shifts and new
/// docs — never for a regression, a lost divergence or a bad note. A
/// `fixed_by=` note is cleared once the doc's class ranks as high as its
/// `was=` class: the later commit fixed it. A `known=` note is cleared when
/// the doc's class rises: it explained the class the doc has left.
pub fn ratchet(
    got: &BTreeMap<String, String>,
    expected_path: &Path,
    divergence_ids: &BTreeSet<String>,
    bless: bool,
) -> Report {
    let expected = read_expected(expected_path);
    let mut report = Report {
        floor: expected.floor,
        ..Report::default()
    };
    report.missing = expected
        .rows
        .keys()
        .filter(|d| !got.contains_key(*d))
        .cloned()
        .collect();
    let mut rows = expected.rows.clone();
    for (doc, v) in got {
        let Some(row) = expected.rows.get(doc) else {
            report.new_docs.push(doc.clone());
            rows.insert(
                doc.clone(),
                Row {
                    verdict: v.clone(),
                    note: String::new(),
                },
            );
            continue;
        };
        if row.note.starts_with("nondet=") {
            continue;
        }
        if let Some(id) = row.note.strip_prefix("divergence=") {
            if !divergence_ids.contains(id) {
                report.bad_notes.push((
                    doc.clone(),
                    format!("divergence ID {id} is not in divergences.tsv"),
                ));
            }
            if class(v) != class(&row.verdict) {
                report.lost_divergences.push(doc.clone());
                continue;
            }
        }
        if row.note.starts_with("fixed_by=") && fixed_by(&row.note).is_none() {
            report.bad_notes.push((
                doc.clone(),
                "a fixed_by= note is `fixed_by=<Pnn|Lnn> was=<class>`".to_string(),
            ));
        }
        if v == "agree" {
            report.agree += 1;
        }
        let explained = ["divergence=", "known=", "fixed_by="]
            .iter()
            .any(|p| row.note.len() > p.len() && row.note.starts_with(p));
        if v.starts_with("ours-") && !explained {
            report.bad_notes.push((
                doc.clone(),
                format!("{v} needs a divergence=, known= or fixed_by= note"),
            ));
        }
        if *v == row.verdict {
            continue;
        }
        let moved = (doc.clone(), row.verdict.clone(), v.clone());
        let (was, now) = (class(&row.verdict), class(v));
        let note = row.note.clone();
        if was == now {
            report.label_shifts.push(moved);
            rows.insert(
                doc.clone(),
                Row {
                    verdict: v.clone(),
                    note,
                },
            );
        } else if rank(&now) > rank(&was) {
            report.improvements.push(moved);
            let paid = note.starts_with("known=")
                || fixed_by(&note).is_some_and(|(_, was)| rank(&now) >= rank(was));
            let note = if paid { String::new() } else { note };
            rows.insert(
                doc.clone(),
                Row {
                    verdict: v.clone(),
                    note,
                },
            );
        } else if rank(&now) < rank(&was) {
            report.regressions.push(moved);
        } else {
            report.shifts.push(moved);
            rows.insert(
                doc.clone(),
                Row {
                    verdict: v.clone(),
                    note,
                },
            );
        }
    }
    if bless {
        write_expected(expected_path, &rows);
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bless clears a `known=` note once the doc's class rises: the note
    /// explained the class the doc has left. A `fixed_by=` note stays until
    /// the doc ranks as high as its `was=` class.
    #[test]
    fn bless_clears_a_known_note_once_the_class_rises() {
        let path =
            std::env::temp_dir().join(format!("census_ratchet_{}_known.tsv", std::process::id()));
        std::fs::write(
            &path,
            "# agree_floor 0\naaaa\tours-refused\tknown=x\nbbbb\tours-refused\tfixed_by=P01 was=agree\n",
        )
        .expect("write");
        let got = BTreeMap::from([
            ("aaaa".to_string(), "e1:state@3".to_string()),
            ("bbbb".to_string(), "e1:state@3".to_string()),
        ]);
        ratchet(&got, &path, &BTreeSet::new(), true);
        let rows = read_expected(&path).rows;
        std::fs::remove_file(&path).expect("remove");
        assert_eq!(rows["aaaa"].note, "");
        assert_eq!(rows["bbbb"].note, "fixed_by=P01 was=agree");
    }
}
