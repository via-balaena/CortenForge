//! Layer E — the parity census against MuJoCo 3.5.0.
//!
//! The MJCF documents in the repository's Rust string literals and Markdown
//! fences are snapshotted under `assets/census/docs/` (`scripts/extract_mjcf.py`;
//! `manifest.tsv` says where each was found). Each is loaded and run here — `forward` dumps at steps 0, 1
//! and 100 and the state at 19 checkpoints, from two initial velocities — and
//! compared with golden data from MuJoCo 3.5.0 built without fused
//! multiply-adds (`scripts/build_mujoco_oracle.sh`, `scripts/gen_census_golden.py`).
//!
//! Each doc gets one verdict: `agree`; the first differing model field; the
//! first differing step or quantity; or a load status (`ours-panic`,
//! `ours-refused`, `mj-refuses`, `both-refuse`). `assets/census/verdicts.tsv` pins each doc's
//! class (see `ratchet::class`) and is a ratchet:
//!
//! - **An improvement fails until blessed.** Run with `CENSUS_BLESS=1` to
//!   rewrite the file; the agree floor rises with it. The file therefore always
//!   records what is reached, so a later regression cannot pass unseen.
//! - **A regression fails and is never blessed.** A deliberate one is a hand
//!   edit of the row's verdict with a note: `divergence=<ID>` when it is a
//!   permanent deviation (the ID must be a row of `divergences.tsv`), or
//!   `fixed_by=<Pnn|Lnn> was=<class>` when a later commit fixes it (bless
//!   clears that note once the doc is back at its `was=` class). Lower the
//!   `# agree_floor` line if needed. The gate cannot tell a hand-edited row
//!   from a blessed one; `scripts/check_census_append_only.sh` refuses a commit
//!   that lowers a row's class without one of these notes.
//! - **`ours-*` on a doc MuJoCo loads** needs a `divergence=`, `known=<label>`
//!   (a defect not fixed yet) or `fixed_by=` note.
//! - **A `divergence=` doc whose class changes fails**: the deliberate
//!   difference was lost, or became another.
//! - **Label shifts** (same class) are listed in the output and recorded on bless.
//! - **`nondet=`** skips a doc whose verdict varies between processes (none today).
//!
//! The snapshot and the golden are append-only (a CI step checks it): new
//! in-tree MJCF is added as new docs with their golden, never edited in place.
//! What the census cannot see is listed in A20 §2.8 of the Rigid spec book
//! (`docs/studies/a_double_dose_of_detail/`): runtime-generated MJCF, horizons
//! past 100 steps, derivatives, `step1`/`step2`, and more.

mod compare;
mod ours;
mod ratchet;

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use serde_json::Value;

fn census_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/census")
}

fn ids_in(dir: &Path, extension: &str) -> BTreeSet<String> {
    std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .filter_map(|entry| {
            let name = entry.ok()?.file_name().into_string().ok()?;
            name.strip_suffix(extension)
                .filter(|id| *id != "meta")
                .map(str::to_string)
        })
        .collect()
}

fn read_json(path: &Path) -> Value {
    let text =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("parse {}: {e}", path.display()))
}

fn list<T: std::fmt::Debug>(out: &mut String, what: &str, items: &[T]) {
    const SHOWN: usize = 10;
    if items.is_empty() {
        return;
    }
    let _ = writeln!(out, "  {what}: {}", items.len());
    for item in items.iter().take(SHOWN) {
        let _ = writeln!(out, "    {item:?}");
    }
    if items.len() > SHOWN {
        let _ = writeln!(out, "    … and {} more", items.len() - SHOWN);
    }
}

#[test]
fn layer_e_parity_census() {
    let dir = census_dir();
    let docs = ids_in(&dir.join("docs"), ".xml");
    let golden = ids_in(&dir.join("golden"), ".json");
    let unpaired: Vec<&String> = docs.symmetric_difference(&golden).collect();
    assert!(
        unpaired.is_empty(),
        "census docs and golden must pair one to one; unpaired: {unpaired:?}"
    );

    let mut got = BTreeMap::new();
    for doc in &golden {
        let xml_path = dir.join("docs").join(format!("{doc}.xml"));
        let xml = std::fs::read_to_string(&xml_path)
            .unwrap_or_else(|e| panic!("read {}: {e}", xml_path.display()));
        let mj = read_json(&dir.join("golden").join(format!("{doc}.json")));
        got.insert(doc.clone(), ratchet::verdict(&mj, &ours::record(&xml)));
    }

    let bless = std::env::var("CENSUS_BLESS").is_ok_and(|v| v == "1");
    let divergences = ratchet::read_divergence_ids(&dir.join("divergences.tsv"));
    let r = ratchet::ratchet(&got, &dir.join("verdicts.tsv"), &divergences, bless);

    let mut msg = format!("parity census: agree {} (floor {})\n", r.agree, r.floor);
    list(
        &mut msg,
        "improvements — record them with CENSUS_BLESS=1",
        &r.improvements,
    );
    list(
        &mut msg,
        "REGRESSIONS — never blessed; see the module doc",
        &r.regressions,
    );
    list(
        &mut msg,
        "class shifts of equal rank — bless if intended",
        &r.shifts,
    );
    list(&mut msg, "new docs — bless to add their rows", &r.new_docs);
    list(&mut msg, "rows with no golden doc", &r.missing);
    list(
        &mut msg,
        "divergence= docs that now agree",
        &r.lost_divergences,
    );
    list(&mut msg, "notes the gate cannot accept", &r.bad_notes);
    list(&mut msg, "label shifts (not failures)", &r.label_shifts);
    if bless {
        assert!(
            !r.blocked(),
            "{msg}  (verdicts.tsv was rewritten without the failures above)"
        );
    } else {
        assert!(r.passes(), "{msg}");
        if !r.label_shifts.is_empty() {
            println!("{msg}");
        }
    }
}
