//! Production-only line coverage: the denominator the Coverage criterion is about.
//!
//! `cargo llvm-cov --lib` instruments the *test* binary, so a crate's own
//! `#[cfg(test)]` code lands in the report alongside the production code it
//! exercises. Counting it makes the criterion measure two unrelated things:
//!
//! - test bodies that **run** are very nearly 100 % covered, so they inflate
//!   the numerator — measured across seven crates, this was worth up to
//!   +12 percentage points;
//! - test bodies that **cannot** run — `#[ignore]`d licence-gated gates, and
//!   the helpers reachable only from them — sit in the denominator
//!   contributing zero, so improving a crate's instrumentation *lowers* its
//!   letter.
//!
//! Both distortions have one cause and one fix: measure only production lines,
//! which is what "≥75 % coverage" was always meant to assert.
//!
//! Two pieces do the work. [`cfg_test_spans`] finds the line ranges that exist
//! only under `cfg(test)`; [`mapped_lines`] reproduces llvm-cov's own
//! line-coverage rule so those ranges can be subtracted line by line.

use std::collections::BTreeMap;

use syn::spanned::Spanned;
use syn::{Attribute, Fields, ImplItem, Item, Meta, TraitItem};

/// Production line coverage for one crate, plus what it could not account for.
#[derive(Debug, Default)]
pub(crate) struct ProductionCoverage {
    /// Production lines executed at least once.
    pub covered: u64,
    /// Production lines instrumented.
    pub total: u64,
    /// `#[cfg(test)]` lines removed from both sides — reported, never silent.
    pub excluded: u64,
    /// Files whose source could not be parsed, so their test code was *not*
    /// excluded. Their lines are still counted, which understates coverage —
    /// the safe direction, but the grade must say so rather than imply a
    /// clean sweep.
    pub unparsed: Vec<String>,
}

impl ProductionCoverage {
    /// Percentage, or `None` when the crate has no production lines at all.
    pub fn percent(&self) -> Option<f64> {
        (self.total > 0).then(|| 100.0 * self.covered as f64 / self.total as f64)
    }
}

/// True if these attributes gate the item on `test`.
///
/// `#[cfg(test)]` and `#[cfg(all(test, ...))]` both mean "test build only".
/// `#[cfg(any(test, ...))]` does **not** — such an item still exists in a
/// non-test build — so it stays in the denominator as production code.
fn is_cfg_test(attrs: &[Attribute]) -> bool {
    attrs.iter().any(|a| {
        if !a.path().is_ident("cfg") {
            return false;
        }
        let Meta::List(list) = &a.meta else {
            return false;
        };
        let s = list.tokens.to_string().replace(' ', "");
        s == "test" || (s.starts_with("all(") && s.contains("test"))
    })
}

fn push(out: &mut Vec<(usize, usize)>, sp: proc_macro2::Span) {
    out.push((sp.start().line, sp.end().line));
}

fn walk_fields(fields: &Fields, out: &mut Vec<(usize, usize)>) {
    for f in fields.iter() {
        if is_cfg_test(&f.attrs) {
            push(out, f.span());
        }
    }
}

fn walk(items: &[Item], out: &mut Vec<(usize, usize)>) {
    for it in items {
        let attrs: &[Attribute] = match it {
            Item::Fn(f) => &f.attrs,
            Item::Mod(m) => &m.attrs,
            Item::Struct(s) => &s.attrs,
            Item::Enum(e) => &e.attrs,
            Item::Impl(i) => &i.attrs,
            Item::Const(c) => &c.attrs,
            Item::Static(s) => &s.attrs,
            Item::Type(t) => &t.attrs,
            Item::Trait(t) => &t.attrs,
            Item::Use(u) => &u.attrs,
            _ => &[],
        };
        if is_cfg_test(attrs) {
            push(out, it.span());
            continue; // the whole subtree is test-only
        }
        match it {
            Item::Mod(m) => {
                if let Some((_, inner)) = &m.content {
                    walk(inner, out);
                }
            }
            Item::Impl(i) => {
                for ii in &i.items {
                    let a: &[Attribute] = match ii {
                        ImplItem::Fn(f) => &f.attrs,
                        ImplItem::Const(c) => &c.attrs,
                        ImplItem::Type(t) => &t.attrs,
                        _ => &[],
                    };
                    if is_cfg_test(a) {
                        push(out, ii.span());
                    }
                }
            }
            Item::Trait(t) => {
                for ti in &t.items {
                    if let TraitItem::Fn(f) = ti {
                        if is_cfg_test(&f.attrs) {
                            push(out, ti.span());
                        }
                    }
                }
            }
            Item::Struct(s) => walk_fields(&s.fields, out),
            Item::Enum(e) => {
                for v in &e.variants {
                    if is_cfg_test(&v.attrs) {
                        push(out, v.span());
                    } else {
                        walk_fields(&v.fields, out);
                    }
                }
            }
            _ => {}
        }
    }
}

/// Inclusive `(first, last)` line ranges that exist only under `cfg(test)`.
///
/// Returns `None` if the source does not parse.
///
/// A range starts at the item's first token, which is its first attribute —
/// doc comments included. That widens each range upward over non-executable
/// lines only, so it cannot swallow a neighbouring item's code.
pub(crate) fn cfg_test_spans(src: &str) -> Option<Vec<(usize, usize)>> {
    let file = syn::parse_file(src).ok()?;
    let mut out = Vec::new();
    walk(&file.items, &mut out);
    Some(out)
}

/// One llvm-cov segment: `[line, col, count, hasCount, isRegionEntry, isGapRegion]`.
struct Seg {
    line: usize,
    count: u64,
    has_count: bool,
    is_region_entry: bool,
    is_gap: bool,
}

fn is_start_of_region(s: &Seg) -> bool {
    !s.is_gap && s.has_count && s.is_region_entry
}

/// Execution count per *mapped* line of one file.
///
/// Ported from LLVM's `LineCoverageIterator` / `LineCoverageStats`
/// (`CoverageMapping.cpp`). Segments mark region **boundaries**, so lines
/// between two boundaries carry the enclosing ("wrapped") region's count and
/// must be filled in; counting only the lines that own a segment undercounts
/// a file by roughly a third.
pub(crate) fn mapped_lines(segments: &[serde_json::Value]) -> BTreeMap<usize, u64> {
    let mut out = BTreeMap::new();
    let segs: Vec<Seg> = segments
        .iter()
        .filter_map(|s| {
            let a = s.as_array()?;
            Some(Seg {
                line: a.first()?.as_u64()? as usize,
                count: a.get(2)?.as_u64().unwrap_or(0),
                has_count: a.get(3)?.as_bool().unwrap_or(false),
                is_region_entry: a.get(4)?.as_bool().unwrap_or(false),
                is_gap: a
                    .get(5)
                    .and_then(serde_json::Value::as_bool)
                    .unwrap_or(false),
            })
        })
        .collect();
    let Some(first) = segs.first() else {
        return out;
    };

    let mut wrapped: Option<&Seg> = None;
    let mut group: Vec<&Seg> = Vec::new();
    let mut next = 0usize;
    let mut line = first.line;

    while next < segs.len() {
        if let Some(last) = group.last() {
            wrapped = Some(last);
        }
        group.clear();
        while next < segs.len() && segs[next].line == line {
            group.push(&segs[next]);
            next += 1;
        }

        let entries = group.iter().filter(|s| is_start_of_region(s)).count();
        // A line whose first segment opens a *skipped* region is not code.
        let skipped = group
            .first()
            .is_some_and(|s| !s.has_count && s.is_region_entry);
        let mapped = !skipped && (wrapped.is_some_and(|w| w.has_count) || entries > 0);

        if mapped {
            let mut exec = if entries == 0 {
                wrapped.map_or(0, |w| w.count)
            } else {
                0
            };
            for s in &group {
                if is_start_of_region(s) {
                    exec = exec.max(s.count);
                }
            }
            out.insert(line, exec);
        }
        line += 1;
    }
    out
}

/// Production line coverage for `crate_path`, from one llvm-cov JSON export.
///
/// Files are matched exactly as the whole-file aggregation did — by path
/// substring — so the report stays scoped to the crate's own sources.
pub(crate) fn production_coverage(
    json: &serde_json::Value,
    crate_path: &str,
) -> ProductionCoverage {
    let mut acc = ProductionCoverage::default();
    let Some(files) = json["data"][0]["files"].as_array() else {
        return acc;
    };

    for file in files {
        let name = file["filename"].as_str().unwrap_or("");
        if !name.contains(crate_path) {
            continue;
        }
        let Some(segments) = file["segments"].as_array() else {
            continue;
        };

        let spans = match std::fs::read_to_string(name)
            .ok()
            .and_then(|src| cfg_test_spans(&src))
        {
            Some(spans) => spans,
            None => {
                acc.unparsed.push(name.to_string());
                Vec::new()
            }
        };

        for (line, count) in mapped_lines(segments) {
            if spans.iter().any(|(a, b)| line >= *a && line <= *b) {
                acc.excluded += 1;
            } else {
                acc.total += 1;
                acc.covered += u64::from(count > 0);
            }
        }
    }
    acc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `#[cfg(test)]` is not only a module attribute in this workspace — it
    /// appears on free functions, on methods inside production impls, and on
    /// struct fields. A text scan for `mod tests` would miss all three.
    #[test]
    fn spans_cover_every_shape_cfg_test_takes() {
        let src = r#"
pub fn prod() -> i32 { 1 }

#[cfg(test)]
fn helper() -> i32 { 2 }

struct S {
    real: i32,
    #[cfg(test)]
    probe: i32,
}

impl S {
    fn keep(&self) -> i32 { self.real }

    #[cfg(test)]
    fn duplicate(&self) -> i32 { self.real }
}

#[cfg(test)]
mod tests {
    #[test]
    fn t() {}
}
"#;
        let spans = cfg_test_spans(src).expect("parses");
        let covers = |line: usize| spans.iter().any(|(a, b)| line >= *a && line <= *b);

        assert!(covers(5), "free fn under cfg(test)");
        assert!(covers(10), "struct field under cfg(test)");
        assert!(covers(17), "method under cfg(test)");
        assert!(covers(23), "test module");

        // Production lines must survive, or the denominator loses real code.
        assert!(!covers(2), "production fn");
        assert!(!covers(8), "production field");
        assert!(!covers(14), "production method");
    }

    /// `any(test, ...)` items exist in a non-test build, so they are production.
    /// Getting this backwards would silently drop shipped code from the metric.
    #[test]
    fn any_test_is_production_but_all_test_is_not() {
        let any = cfg_test_spans("#[cfg(any(test, feature = \"f\"))]\nfn a() {}\n").unwrap();
        assert!(any.is_empty(), "any(test, ..) must stay in the denominator");

        let all = cfg_test_spans("#[cfg(all(test, unix))]\nfn a() {}\n").unwrap();
        assert_eq!(all.len(), 1, "all(test, ..) is test-only");
    }

    #[test]
    fn unparseable_source_is_reported_not_guessed() {
        assert!(cfg_test_spans("fn broken( {").is_none());
    }

    fn seg(
        line: u64,
        col: u64,
        count: u64,
        has: bool,
        entry: bool,
        gap: bool,
    ) -> serde_json::Value {
        serde_json::json!([line, col, count, has, entry, gap])
    }

    /// Segments mark region *boundaries*. The lines between an entry and its
    /// end belong to the enclosing region and must be filled in — this is the
    /// bug that made a first attempt undercount every file it touched.
    #[test]
    fn lines_between_two_segments_inherit_the_wrapped_count() {
        let segments = vec![
            seg(10, 5, 7, true, true, false),   // region entry, executed 7×
            seg(14, 6, 0, false, false, false), // region end
        ];
        let lines = mapped_lines(&segments);
        assert_eq!(
            lines.keys().copied().collect::<Vec<_>>(),
            vec![10, 11, 12, 13, 14],
            "every line of the region is mapped, not just the two with segments"
        );
        assert!(lines.values().all(|c| *c == 7));
    }

    #[test]
    fn an_unexecuted_region_is_mapped_but_uncovered() {
        let segments = vec![
            seg(3, 1, 0, true, true, false),
            seg(5, 2, 0, false, false, false),
        ];
        let lines = mapped_lines(&segments);
        assert_eq!(lines.len(), 3);
        assert!(
            lines.values().all(|c| *c == 0),
            "an uncovered region still occupies the denominator"
        );
    }

    #[test]
    fn empty_segment_list_yields_no_lines() {
        assert!(mapped_lines(&[]).is_empty());
    }
}
