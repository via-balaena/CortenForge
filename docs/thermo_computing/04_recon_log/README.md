# Recon Log

> Substantive header text folded in from `MASTER_PLAN.md` §7 *Recon Log*
> intro paragraph during the 2026-04-09 doc-tree refactor.

Dated entries documenting what we learned and how the plan changed. Append,
don't rewrite. This is the record of *why* the plan looks the way it does.

## Conventions

### Append by adding files

Each entry is its own file. New entries do not edit existing files;
they create a new one. The file you are reading replaces the old
"append to the bottom of `MASTER_PLAN.md` §7" pattern, which had
become a maintenance burden as the master plan grew past 2000 lines.

### Filename scheme

`YYYY-MM-DD_part_NN_<short_slug>.md` — date-prefixed so chronological
order matches lexicographic order, then a numeric `part_NN` tag for
explicit sequencing, then a short slug for at-a-glance recognition.

### Numbering

Parts `00a` and `00b` are the unnumbered "scaffold" entries from the
original recon round; the rest start at `02`. **There is no part 1.**
This oddity is preserved verbatim from the original master plan
recon log because (a) cross-references between entries already use
"part 4", "part 9 finding 2" etc., and (b) renumbering would break
those references for no real gain.

### Cross-references between entries

References of the form "part N" — e.g., "see part 4", "per part 9
finding 2" — point to the corresponding `2026-04-09_part_NN_*.md`
file. They are kept as plain text rather than rewritten as markdown
links because:

- Plain text is grep-able and survives file renames.
- The references are *historical claims about the record*, not
  navigation aids.
- Adding ~100+ link rewrites across all entries would have violated
  the "split, not rewrite" principle of the 2026-04-09 doc-tree
  refactor.

If you need to find an entry by part number, the filename pattern
makes it trivial: `ls *part_04*.md`.

## Entries

The full list (and clickable links to each entry) lives in
[`../SUMMARY.md`](../SUMMARY.md) under the *Recon Log* section.
