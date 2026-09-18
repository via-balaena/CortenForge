//! Self-test: the disclaimer's seven copies still agree with each other.
//!
//! ⚠ **What this asserts is [`LONG_ABOUT`]** — the text `cargo xtask
//! disclaimer-sync --help` prints, and the one place that definition lives.
//! This header carries only the **why**.
//!
//! # That the hazard is real
//!
//! The same legal text lives in seven places, and until this module the only
//! thing holding them together was a comment in `waiver.rs` asking the next
//! editor to remember. On 2026-09-18 the disclaimer was widened from the body
//! to the farm's hazards (#938) and the clause *"you alone are responsible for
//! choosing body-safe materials and for proper mixing, curing, cleaning, and
//! hygiene"* was dropped from two of the seven on the way. The files parsed,
//! CI was green across all 25 checks, and the new text read **better** than the
//! old. Nothing in the tree could have told anyone.
//!
//! # Why the copies are not compared for equality
//!
//! They are deliberately different, and a gate demanding they match would be
//! wrong rather than strict. `NOTICE` is plain text under an Apache header;
//! the README and site footer are summaries that link to the full document;
//! Cendrillon's launch waiver is scoped to the one thing that application does.
//!
//! So the unit of comparison is the **clause**, not the sentence. A surface is
//! in sync when it carries the clauses its [`Tier`] requires — which lets the
//! wording differ while the substance cannot quietly diverge.
//!
//! # Why the tiers encode a decision, not just a fact
//!
//! [`Tier::Scoped`] is the interesting one. Cendrillon's waiver and footer
//! carry the silicone clause and deliberately **omit** the hydrogen hazards,
//! because that application designs molds from scans and widening its launch
//! gate would dilute terms a user is actually being asked to tick. That was a
//! judgement call. Encoding it here means a future editor who reverses it has
//! to do so on purpose, in this file, rather than by editing one surface.
//!
//! # Why the roster is closed
//!
//! Listing the seven would catch drift *within* them and miss an eighth. So the
//! scan runs the other way too: any tracked file carrying the disclaimer's
//! distinctive phrasing and **not** on the roster fails, because a new surface
//! nobody classified is a surface nothing keeps in sync.
//!
//! # Why files come from git, not the filesystem
//!
//! A walk of the working tree reads whatever happens to be sitting in it. In
//! development that meant a Python `venv/` contributing 16 false positives from
//! numpy and scipy — and, worse, an exclusion written as `starts_with(".git")`
//! silently swallowed the whole `.github/` directory, hiding a real surface.
//! `git ls-files` has neither failure: it is the set of files that ship.
//!
//! # Why a command and not only a unit test
//!
//! Same reason [`crate::release_gates`] gives: `cargo test -p xtask` runs only
//! when xtask is in the PR's affected set, and a PR editing `DISCLAIMER.md`
//! does not touch xtask. It runs as a step in the deliberately unscoped
//! `licensed-gates` job, which is a source scan with no build.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::process::Command;

use anyhow::{bail, Context, Result};
use owo_colors::OwoColorize;

/// Long help for `cargo xtask disclaimer-sync`.
///
/// ★ **The single statement of what this command asserts.** `main.rs` points
/// clap here rather than restating it, and the module header points here rather
/// than restating it — so there is one copy, not three.
pub const LONG_ABOUT: &str = "\
The disclaimer's seven copies still agree.

The same legal text lives in DISCLAIMER.md, NOTICE, README.md, both site
footers, Cendrillon's launch waiver and the release-notes template. They are
deliberately worded differently, so this compares CLAUSES, not sentences.

Asserts, in order:

  1. CANONICAL AGREEMENT — DISCLAIMER.md and NOTICE carry the same clause set.
     These two are the full statement; if one gains or loses a clause and the
     other does not, they have diverged.

  2. THE FLOOR — every surface carries as-is, not-a-medical-device and
     at-your-own-risk. No summary is short enough to drop these.

  3. TIER SCOPE — general surfaces (README, site front page) name the hazards;
     Cendrillon-scoped surfaces (its waiver, its footer) carry the body-safe
     materials clause. Cendrillon deliberately omits the hydrogen hazards, and
     that omission is recorded here so reversing it is a deliberate edit.

  4. A CLOSED ROSTER — no tracked file outside the roster carries the
     disclaimer's distinctive phrasing. An unclassified eighth surface is one
     nothing keeps in sync.

A file on the roster that cannot be read is a FAILURE, not a pass: its clauses
are unknown, and an absence that was never observed is not evidence.";

/// One clause of the disclaimer.
///
/// ⚠ The patterns match **normalised** text (see [`normalise`]) — lower-case,
/// markup stripped, whitespace collapsed. Matching raw source instead is the
/// trap this module was built around: `DISCLAIMER.md` wraps
/// `**without\nwarranty of any kind**` across a newline, so a naive search for
/// the phrase reports it missing from the canonical document itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Clause {
    /// Provided "as is".
    AsIs,
    /// Without warranty of any kind.
    NoWarranty,
    /// Not a medical device, and makes no health claims.
    NotMedical,
    /// You use it, and anything made with it, entirely at your own risk.
    OwnRisk,
    /// Responsibility for choosing body-safe materials, and for mixing,
    /// curing, cleaning and hygiene.
    ///
    /// ⚠⚠ This is the clause #938 dropped, and it is NOT the same statement as
    /// the hazard sentence beside it (*"improperly cured or non-body-safe
    /// silicone can cause serious injury"*). The first draft of this gate keyed
    /// on the substring `body-safe`, which the hazard sentence also contains —
    /// so deleting the responsibility clause left the needle satisfied and the
    /// gate green on the exact defect it exists to catch.
    BodySafeResponsibility,
    /// The hazard classes the farm work introduced — hydrogen and its company.
    Hazards,
    /// Simulation results are approximations and need physical validation.
    SimApprox,
    /// To the fullest extent permitted by law, no liability.
    NoLiability,
}

impl Clause {
    /// Every clause, in the order a reader meets them.
    const ALL: [Clause; 8] = [
        Clause::AsIs,
        Clause::NoWarranty,
        Clause::NotMedical,
        Clause::OwnRisk,
        Clause::BodySafeResponsibility,
        Clause::Hazards,
        Clause::SimApprox,
        Clause::NoLiability,
    ];

    /// The name this clause is reported by.
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Clause::AsIs => "as-is",
            Clause::NoWarranty => "no-warranty",
            Clause::NotMedical => "not-medical",
            Clause::OwnRisk => "own-risk",
            Clause::BodySafeResponsibility => "body-safe-responsibility",
            Clause::Hazards => "hazards",
            Clause::SimApprox => "sim-approx",
            Clause::NoLiability => "no-liability",
        }
    }

    /// Substrings that mean the clause is present; any one is enough.
    ///
    /// ⚠ Deliberately phrase fragments rather than whole sentences. The
    /// surfaces word these differently on purpose, and a gate keyed to one
    /// surface's sentence would fail on the others' correct paraphrases.
    const fn needles(self) -> &'static [&'static str] {
        match self {
            Clause::AsIs => &["as is under", "as is,", "as is without"],
            Clause::NoWarranty => &["without warrant", "no warrant"],
            Clause::NotMedical => &["not a medical device"],
            Clause::OwnRisk => &["at your own risk"],
            Clause::BodySafeResponsibility => &["choosing body-safe"],
            Clause::Hazards => &["hydrogen"],
            Clause::SimApprox => &["simulation results are approximations"],
            Clause::NoLiability => &["no liability", "not liable"],
        }
    }
}

/// What a surface is for, which decides what it must carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Tier {
    /// The full statement. Both copies must agree with each other exactly.
    Canonical,
    /// A summary of the whole project's posture, linking to the full text.
    General,
    /// Scoped to Cendrillon — silicone molds from scans, and nothing else.
    Scoped,
    /// Defers to `DISCLAIMER.md` rather than restating it.
    Pointer,
}

impl Tier {
    /// Clauses this tier must carry beyond [`FLOOR`].
    const fn required(self) -> &'static [Clause] {
        match self {
            // Checked against its sibling instead, which is stricter than any
            // fixed list: the pair must agree whatever the list becomes.
            Tier::Canonical => &[],
            Tier::General => &[Clause::Hazards],
            Tier::Scoped => &[Clause::BodySafeResponsibility],
            Tier::Pointer => &[],
        }
    }
}

/// Every surface carrying this text, and what each one is for.
///
/// ⚠ Adding a surface here is the *second* half of adding one. The first is
/// writing the text; assertion 4 fails until both are done.
pub(crate) const ROSTER: &[(&str, Tier)] = &[
    ("DISCLAIMER.md", Tier::Canonical),
    ("NOTICE", Tier::Canonical),
    ("README.md", Tier::General),
    ("site/index.html", Tier::General),
    ("site/cendrillon/index.html", Tier::Scoped),
    ("tools/cf-studio-gui/src/waiver.rs", Tier::Scoped),
    (".github/workflows/release.yml", Tier::Pointer),
];

/// What no surface is short enough to omit.
const FLOOR: [Clause; 3] = [Clause::AsIs, Clause::NotMedical, Clause::OwnRisk];

/// Paths whose disclaimer-like text is not ours to keep in sync.
///
/// Third-party licences and vendored model assets carry standard warranty
/// language that this repository neither wrote nor may edit.
const NOT_OURS: &[&str] = &["sim/L0/tests/assets/", "LICENSE-MIT", "LICENSE-APACHE"];

/// This module, which necessarily quotes the text it checks.
///
/// ⚠⚠ Found by the gate failing on itself the moment `git add` made it
/// tracked — which is also why it passed when run from an untracked working
/// copy. Its test fixtures contain *"This tool is not a medical device"*, so it
/// reads as an eighth surface. It is not one: nobody is asked to accept these
/// terms by reading a Rust module.
///
/// ⛔ This is the one self-reference, not a category. Widening it to
/// `xtask/` — or to any directory — would let a genuine surface hide behind it,
/// which is why `tests::every_exclusion_is_load_bearing` fails if the
/// exclusions ever hide more than the files named here.
const SELF_SOURCE: &str = "xtask/src/disclaimer_sync.rs";

/// Files above this size are not prose and are not read.
const MAX_BYTES: u64 = 400_000;

/// Lower-case, markup-free, single-spaced text.
///
/// Strips markdown emphasis, HTML tags and entities, and the `\n` escapes that
/// appear inside Rust string literals — so `waiver.rs` and `index.html` are
/// compared on what a reader sees, not on how it is encoded.
pub(crate) fn normalise(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    let mut chars = raw.chars().peekable();
    let mut in_tag = false;
    while let Some(ch) = chars.next() {
        match ch {
            '<' => in_tag = true,
            '>' if in_tag => in_tag = false,
            _ if in_tag => {}
            // `\n` and `\"` inside Rust and YAML string literals.
            '\\' => {
                chars.next();
                out.push(' ');
            }
            '*' | '_' | '`' | '#' => {}
            _ => out.push(ch.to_ascii_lowercase()),
        }
    }
    // HTML entities become spaces: `&mdash;` must not glue two words together.
    let out = out.replace('&', " & ");
    out.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// Which clauses this normalised text carries.
pub(crate) fn clauses_in(normalised: &str) -> BTreeSet<Clause> {
    Clause::ALL
        .into_iter()
        .filter(|clause| {
            clause
                .needles()
                .iter()
                .any(|needle| normalised.contains(needle))
        })
        .collect()
}

/// Whether this text carries the disclaimer's distinctive legal phrasing.
///
/// ⚠ Deliberately narrow. "its own risk" is ordinary English and appears in
/// research prose across `docs/`; the conjunction below does not. Across 2964
/// tracked files this matches the seven roster surfaces and nothing else.
pub(crate) fn is_a_disclaimer_surface(normalised: &str) -> bool {
    let medical = normalised.contains("not a medical device");
    let risk_and_warranty = normalised.contains("at your own risk")
        && Clause::NoWarranty
            .needles()
            .iter()
            .any(|needle| normalised.contains(needle));
    medical || risk_and_warranty
}

/// One way the surfaces can be out of sync.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Finding {
    /// A roster file is missing, or could not be read as text.
    Unreadable { path: String, why: String },
    /// The two canonical copies no longer carry the same clauses.
    CanonicalDrift {
        clause: Clause,
        present_in: String,
        absent_from: String,
    },
    /// A surface dropped a clause no surface may omit.
    FloorBreach { path: String, clause: Clause },
    /// A surface no longer carries what its tier is for.
    ScopeBreach {
        path: String,
        tier: Tier,
        clause: Clause,
    },
    /// A surface exists that nothing classified.
    Unrostered { path: String },
}

impl Finding {
    /// The line reported for this finding.
    fn message(&self) -> String {
        match self {
            Finding::Unreadable { path, why } => {
                format!("{path}: on the roster but unreadable ({why}) — its clauses are UNKNOWN")
            }
            Finding::CanonicalDrift {
                clause,
                present_in,
                absent_from,
            } => format!(
                "the canonical pair disagrees: {} is in {present_in} but not {absent_from}",
                clause.name()
            ),
            Finding::FloorBreach { path, clause } => format!(
                "{path}: dropped {}, which no surface is short enough to omit",
                clause.name()
            ),
            Finding::ScopeBreach { path, tier, clause } => format!(
                "{path}: a {tier:?} surface must carry {} and does not",
                clause.name()
            ),
            Finding::Unrostered { path } => format!(
                "{path}: reads as a disclaimer but is on no tier — classify it in \
                 disclaimer_sync::ROSTER or it is kept in sync by nothing"
            ),
        }
    }
}

/// Every tracked file's path, from git rather than from a directory walk.
fn tracked_files(root: &Path) -> Result<Vec<String>> {
    let out = Command::new("git")
        .arg("-C")
        .arg(root)
        .args(["ls-files", "-z"])
        .output()
        .context("running `git ls-files` to enumerate tracked files")?;
    if !out.status.success() {
        bail!(
            "`git ls-files` failed in {}: {}",
            root.display(),
            String::from_utf8_lossy(&out.stderr).trim()
        );
    }
    Ok(String::from_utf8_lossy(&out.stdout)
        .split('\0')
        .filter(|path| !path.is_empty())
        .map(str::to_owned)
        .collect())
}

/// Whether the closure scan honours its exclusions.
///
/// ⚠ Exists so a test can run the scan with them **off** and prove each one is
/// load-bearing. An exclusion nobody can see the effect of is where a real
/// surface goes to hide.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Exclusions {
    /// Normal operation.
    Applied,
    /// Ignored, so a test can see what they are hiding.
    ///
    /// ⚠ Built only under `cfg(test)`. The allow is narrowed to this variant
    /// rather than put on the enum, so the other one is still checked.
    #[cfg_attr(not(test), allow(dead_code))]
    Ignored,
}

/// Tracked files that read as a disclaimer but are on no tier.
pub(crate) fn unrostered(root: &Path, exclusions: Exclusions) -> Result<Vec<String>> {
    let rostered: BTreeSet<&str> = ROSTER.iter().map(|(path, _)| *path).collect();
    let mut others = Vec::new();
    for path in tracked_files(root)? {
        if rostered.contains(path.as_str()) {
            continue;
        }
        if exclusions == Exclusions::Applied
            && (path == SELF_SOURCE || NOT_OURS.iter().any(|prefix| path.starts_with(prefix)))
        {
            continue;
        }
        if normalised_file(root, &path).is_some_and(|text| is_a_disclaimer_surface(&text)) {
            others.push(path);
        }
    }
    Ok(others)
}

/// Read and normalise one file, if it is prose we may read.
fn normalised_file(root: &Path, path: &str) -> Option<String> {
    let full = root.join(path);
    if std::fs::metadata(&full).ok()?.len() > MAX_BYTES {
        return None;
    }
    std::fs::read_to_string(&full)
        .ok()
        .map(|raw| normalise(&raw))
}

/// Judge a surveyed tree. Pure, so the tests can build one that fails.
///
/// `carried` maps every roster path that could be read to its clauses;
/// `unreadable` names the ones that could not; `others` names non-roster
/// tracked files that read as a disclaimer.
pub(crate) fn audit(
    carried: &BTreeMap<String, BTreeSet<Clause>>,
    unreadable: &[(String, String)],
    others: &[String],
) -> Vec<Finding> {
    let mut findings = Vec::new();

    // ⛔ First, because everything below reasons about clause sets: a file
    // whose clauses could not be read has UNKNOWN clauses, and reporting the
    // rest as clean would be reporting an absence nobody observed.
    for (path, why) in unreadable {
        findings.push(Finding::Unreadable {
            path: path.clone(),
            why: why.clone(),
        });
    }

    // 1. The canonical pair must agree with each other.
    let canonical: Vec<&String> = ROSTER
        .iter()
        .filter(|(_, tier)| *tier == Tier::Canonical)
        .filter_map(|(path, _)| carried.get_key_value(*path).map(|(key, _)| key))
        .collect();
    if let [first, second] = canonical[..] {
        let (left, right) = (&carried[first], &carried[second]);
        for clause in left.symmetric_difference(right) {
            let (present_in, absent_from) = if left.contains(clause) {
                (first, second)
            } else {
                (second, first)
            };
            findings.push(Finding::CanonicalDrift {
                clause: *clause,
                present_in: present_in.clone(),
                absent_from: absent_from.clone(),
            });
        }
    }

    // 2 and 3. The floor, then what each tier is for.
    for (path, tier) in ROSTER {
        let Some(present) = carried.get(*path) else {
            continue; // already reported as unreadable
        };
        for clause in FLOOR {
            if !present.contains(&clause) {
                findings.push(Finding::FloorBreach {
                    path: (*path).to_owned(),
                    clause,
                });
            }
        }
        for clause in tier.required() {
            if !present.contains(clause) {
                findings.push(Finding::ScopeBreach {
                    path: (*path).to_owned(),
                    tier: *tier,
                    clause: *clause,
                });
            }
        }
    }

    // 4. Nothing carries this text unclassified.
    for path in others {
        findings.push(Finding::Unrostered { path: path.clone() });
    }

    findings
}

/// Survey the tree, then judge it.
pub fn check() -> Result<()> {
    check_at(Path::new("."))
}

/// [`check`], rooted at an explicit workspace directory.
///
/// ⚠ Split out so a test can point at the workspace **without**
/// `set_current_dir`, which is a global mutation in a process running tests on
/// parallel threads.
pub(crate) fn check_at(root: &Path) -> Result<()> {
    let mut carried = BTreeMap::new();
    let mut unreadable = Vec::new();
    for (path, _) in ROSTER {
        match normalised_file(root, path) {
            Some(text) => {
                carried.insert((*path).to_owned(), clauses_in(&text));
            }
            None => unreadable.push((
                (*path).to_owned(),
                "missing, too large, or not UTF-8".to_owned(),
            )),
        }
    }

    let findings = audit(
        &carried,
        &unreadable,
        &unrostered(root, Exclusions::Applied)?,
    );
    if findings.is_empty() {
        println!(
            "{} disclaimer: {} surfaces agree",
            "✓".green(),
            ROSTER.len()
        );
        return Ok(());
    }
    for finding in &findings {
        eprintln!("{} {}", "✗".red(), finding.message());
    }
    bail!(
        "{} disclaimer surface(s) out of sync — see `cargo xtask disclaimer-sync --help`",
        findings.len()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The workspace this xtask belongs to.
    fn workspace() -> &'static Path {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap_or(Path::new("."))
    }

    /// ★★ The gate against the real tree. If this fails, the surfaces drifted.
    #[test]
    fn the_shipped_surfaces_are_in_sync() {
        check_at(workspace()).expect("the disclaimer's surfaces agree");
    }

    /// ★★★ Every exclusion earns its place, and hides nothing else.
    ///
    /// ⚠ The half that matters is the second assertion. An exclusion list is
    /// where a real surface hides, and the usual way that happens is someone
    /// widening one entry to a directory to silence a single file. Running the
    /// scan with exclusions OFF shows exactly what they are suppressing, so
    /// this fails the moment that set grows past the files named on purpose.
    #[test]
    fn every_exclusion_is_load_bearing() {
        let hidden =
            unrostered(workspace(), Exclusions::Ignored).expect("the scan reads the tracked files");
        let expected: BTreeSet<&str> = std::iter::once(SELF_SOURCE).collect();
        let hidden: BTreeSet<&str> = hidden.iter().map(String::as_str).collect();

        assert_eq!(
            hidden, expected,
            "the exclusions hide exactly the files named on purpose"
        );
        assert!(
            unrostered(workspace(), Exclusions::Applied)
                .expect("the scan reads the tracked files")
                .is_empty(),
            "and with them applied, nothing is left unclassified"
        );
    }

    /// ⚠ The normaliser's actual job, and the bug that motivated this module:
    /// `DISCLAIMER.md` wraps `**without\nwarranty of any kind**`, so a search
    /// of the raw bytes reports the canonical document missing its own clause.
    #[test]
    fn a_clause_wrapped_across_a_newline_is_still_found() {
        let raw = "at your option, **without\nwarranty of any kind**";
        assert!(
            !raw.contains("without warrant"),
            "the raw text really does hide it"
        );
        assert!(clauses_in(&normalise(raw)).contains(&Clause::NoWarranty));
    }

    /// ⚠ Markup must not glue words together or split them apart: an entity
    /// becomes a space, a tag vanishes, and a Rust `\n` escape is a space.
    #[test]
    fn markup_does_not_change_which_clauses_are_found() {
        let html = "<p>It is <strong>not a medical device</strong>&mdash;use at your own risk.</p>";
        let rust = "\"You use it at your own risk.\\n\\nIt is not a medical device.\"";
        for text in [html, rust] {
            let found = clauses_in(&normalise(text));
            assert!(found.contains(&Clause::NotMedical), "{text}");
            assert!(found.contains(&Clause::OwnRisk), "{text}");
        }
    }

    /// Every surface the roster names is real. A typo here would silently
    /// downgrade the gate to checking fewer files than it claims.
    #[test]
    fn every_rostered_path_exists() {
        for (path, _) in ROSTER {
            assert!(
                workspace().join(path).is_file(),
                "roster names {path}, which is not a file"
            );
        }
    }

    /// A tree with everything present, used as the base each failure mutates.
    fn healthy() -> BTreeMap<String, BTreeSet<Clause>> {
        ROSTER
            .iter()
            .map(|(path, tier)| {
                let mut present: BTreeSet<Clause> = FLOOR.into_iter().collect();
                present.extend(tier.required().iter().copied());
                if *tier == Tier::Canonical {
                    present.extend(Clause::ALL);
                }
                ((*path).to_owned(), present)
            })
            .collect()
    }

    /// ★★★ The gate is not vacuous: the healthy tree passes, so each failure
    /// below is caused by the mutation and not by the fixture.
    #[test]
    fn a_healthy_tree_produces_no_findings() {
        assert_eq!(audit(&healthy(), &[], &[]), vec![]);
    }

    /// ★★★ Assertion 1 fails when the canonical pair diverges — the shape of
    /// the real #938 defect, where one document kept a clause the other lost.
    #[test]
    fn a_clause_dropped_from_one_canonical_copy_is_caught() {
        let mut tree = healthy();
        tree.get_mut("NOTICE")
            .expect("NOTICE is on the roster")
            .remove(&Clause::BodySafeResponsibility);

        let findings = audit(&tree, &[], &[]);
        assert!(
            findings.contains(&Finding::CanonicalDrift {
                clause: Clause::BodySafeResponsibility,
                present_in: "DISCLAIMER.md".to_owned(),
                absent_from: "NOTICE".to_owned(),
            }),
            "{findings:?}"
        );
    }

    /// ★★★ Assertion 2 fails when any surface drops the floor.
    #[test]
    fn a_surface_that_drops_the_floor_is_caught() {
        let mut tree = healthy();
        tree.get_mut("site/index.html")
            .expect("the site front page is on the roster")
            .remove(&Clause::NotMedical);

        assert!(audit(&tree, &[], &[]).contains(&Finding::FloorBreach {
            path: "site/index.html".to_owned(),
            clause: Clause::NotMedical,
        }));
    }

    /// ★★★ Assertion 3 fails when a tier stops carrying what it is for — here,
    /// Cendrillon's waiver losing the silicone clause that is its whole point.
    #[test]
    fn a_scoped_surface_that_drops_its_own_clause_is_caught() {
        let mut tree = healthy();
        tree.get_mut("tools/cf-studio-gui/src/waiver.rs")
            .expect("the waiver is on the roster")
            .remove(&Clause::BodySafeResponsibility);

        assert!(audit(&tree, &[], &[]).contains(&Finding::ScopeBreach {
            path: "tools/cf-studio-gui/src/waiver.rs".to_owned(),
            tier: Tier::Scoped,
            clause: Clause::BodySafeResponsibility,
        }));
    }

    /// ★★★ Assertion 4 fails on an eighth surface nobody classified.
    #[test]
    fn an_unclassified_surface_is_caught() {
        let findings = audit(&healthy(), &[], &["site/about/index.html".to_owned()]);
        assert!(findings.contains(&Finding::Unrostered {
            path: "site/about/index.html".to_owned(),
        }));
    }

    /// ⛔ An unreadable roster file must FAIL, not pass quietly. Its clauses
    /// are unknown, and the rest of the audit would otherwise report a clean
    /// tree while one of the seven went unexamined.
    #[test]
    fn an_unreadable_surface_fails_rather_than_passing() {
        let mut tree = healthy();
        tree.remove("NOTICE");
        let unreadable = [("NOTICE".to_owned(), "not UTF-8".to_owned())];

        let findings = audit(&tree, &unreadable, &[]);
        assert!(
            findings.contains(&Finding::Unreadable {
                path: "NOTICE".to_owned(),
                why: "not UTF-8".to_owned(),
            }),
            "{findings:?}"
        );
    }

    /// ⚠ The marker must be narrow enough that ordinary prose does not trip
    /// assertion 4. "its own risk" appears in research writing across `docs/`;
    /// a gate that cried wolf there would be turned off.
    #[test]
    fn ordinary_prose_does_not_read_as_a_disclaimer() {
        for prose in [
            "Three categorically-distinct paths forward, each with its own risk.",
            "The solver carries its own risk of divergence under load.",
            "This module is provided as is for the benchmark harness.",
        ] {
            assert!(
                !is_a_disclaimer_surface(&normalise(prose)),
                "false positive on: {prose}"
            );
        }
    }

    /// …and wide enough to catch a real one written in someone else's words.
    #[test]
    fn a_real_disclaimer_in_different_words_still_reads_as_one() {
        for real in [
            "This tool is not a medical device.",
            "Use it at your own risk; it comes without warranty of any kind.",
        ] {
            assert!(is_a_disclaimer_surface(&normalise(real)), "missed: {real}");
        }
    }
}
