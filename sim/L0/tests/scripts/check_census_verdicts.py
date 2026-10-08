#!/usr/bin/env python3
"""Refuse a hand edit of the census verdicts that the census test cannot see.

    check_census_verdicts.py <from-ref> <to-ref> <label>

The census test compares each doc's verdict with sim/L0/tests/assets/census/
verdicts.tsv, so it cannot tell a row a commit edited by hand from one the
test wrote. This compares the file at the two refs:

- a doc whose class ranks lower at <to-ref> must carry a `divergence=<ID>`,
  `known=<label>` or `fixed_by=<Pnn|Lnn> was=<class>` note there, a was= class
  ranking at least as high as the doc's class at <from-ref>;
- a row with one of those notes at <from-ref> keeps a note until it ranks as
  high as the note's was= class (a fixed_by= note) or higher than it did (the
  others), and a note that replaces it must account for that rank too;
- no row is removed, and none gains a `nondet=` note, which makes the census
  skip it, a new doc included;
- `# agree_floor` falls by at most the number of `agree` rows that stop being
  `agree`.

A doc missing at <from-ref> cannot have been lowered. `klass`, `rank` and the
note forms mirror class, rank, is_class and fixed_by in
sim/L0/tests/mujoco_conformance/layer_e_census/ratchet.rs. Exits 1 on any of
these.
"""
import re
import subprocess
import sys

PATH = 'sim/L0/tests/assets/census/verdicts.tsv'


def rows(ref):
    """The file's rows at `ref`, {doc: (verdict, note)}, and its agree floor."""
    show = subprocess.run(['git', 'show', f'{ref}:{PATH}'], capture_output=True, text=True)
    if show.returncode != 0:
        return {}, 0
    out, floor = {}, 0
    for line in show.stdout.splitlines():
        if line.startswith('# agree_floor '):
            n = line[len('# agree_floor '):].strip()
            floor = int(n) if n.isdigit() else 0
        if line.startswith('#') or not line.strip():
            continue
        cols = line.split('\t')
        out[cols[0]] = (cols[1] if len(cols) > 1 else '', cols[2] if len(cols) > 2 else '')
    return out, floor


def klass(verdict):
    if verdict == 'agree' or verdict.startswith('ours-') or verdict in ('mj-refuses', 'both-refuse'):
        return verdict
    if verdict.startswith('model:'):
        rest = verdict[len('model:'):]
        dynamics = 'dyn-agree' if rest.endswith(';dyn:agree') else 'dyn-differs'
        return f"model:{rest.split(';')[0]};{dynamics}"
    return 'dyn'


def rank(cls):
    if cls == 'agree':
        return 5
    if cls == 'both-refuse':
        return 4
    if cls.startswith('model:') and cls.endswith(';dyn-agree'):
        return 3
    if cls == 'mj-refuses':
        return 1
    if cls.startswith('ours-'):
        return 0
    return 2


def is_class(cls):
    if any(c.isspace() for c in cls):
        return False
    return (cls in ('agree', 'dyn', 'mj-refuses', 'both-refuse')
            or (cls.startswith('ours-') and len(cls) > len('ours-'))
            or (cls.startswith('model:') and cls.endswith((';dyn-agree', ';dyn-differs'))))


def fixed_by_was(note):
    """A well-formed `fixed_by=` note's was= class, else None."""
    m = re.fullmatch(r'fixed_by=[PL][0-9]+[a-z]? was=(.+)', note)
    return m.group(1) if m and is_class(m.group(1)) else None


def explained(note, needed):
    """Whether `note` accounts for a row ranking below the rank `needed`."""
    for prefix in ('divergence=', 'known='):
        if note.startswith(prefix):
            return len(note) > len(prefix)
    was = fixed_by_was(note)
    return was is not None and rank(was) >= needed


def owed(verdict, note):
    """The rank a row with `note` must reach before the note may go: a
    fixed_by= note's was= class, otherwise one above the row's own class, and
    no more than agree: a note an agree row kept owes nothing."""
    was = fixed_by_was(note)
    return rank(was) if was is not None else min(rank(klass(verdict)) + 1, rank('agree'))


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    (before, floor_before), (after, floor_after), label = rows(sys.argv[1]), rows(sys.argv[2]), sys.argv[3]
    lowered = [(doc, before[doc][0], verdict, note) for doc, (verdict, note) in sorted(after.items())
               if doc in before and rank(klass(verdict)) < rank(klass(before[doc][0]))
               and not explained(note, rank(klass(before[doc][0])))]
    for doc, was, now, note in lowered:
        print(f'::error::{label}: {doc} lowered from {was} to {now} without a divergence=<ID>, '
              f'known=<label> or fixed_by=<Pnn|Lnn> was=<class at least {klass(was)}> note '
              f'(has {note!r})')
    skipped = [doc for doc, (_, note) in sorted(after.items())
               if note.startswith('nondet=') and not before.get(doc, ('', ''))[1].startswith('nondet=')]
    for doc in skipped:
        print(f'::error::{label}: {doc} gained a nondet= note; the census would stop checking it')
    removed = sorted(doc for doc in before if doc not in after)
    for doc in removed:
        print(f'::error::{label}: {doc} lost its row; a doc keeps its row as it keeps its file')
    unpaid = [(doc, note, after[doc]) for doc, (verdict, note) in sorted(before.items())
              if note and not note.startswith('nondet=') and doc in after
              and rank(klass(after[doc][0])) < owed(verdict, note)
              and not explained(after[doc][1], owed(verdict, note))]
    for doc, note, (verdict, now) in unpaid:
        print(f'::error::{label}: {doc} ({verdict}) dropped or weakened its note {note!r} '
              f'(now {now!r}) before ranking as high as the note owes')
    left_agree = sum(1 for doc, (verdict, _) in before.items()
                     if verdict == 'agree' and after.get(doc, ('', ''))[0] != 'agree')
    fell = floor_after < floor_before - left_agree
    if fell:
        print(f'::error::{label}: # agree_floor fell from {floor_before} to {floor_after}; it may fall '
              f'only by the agree rows that stop being agree ({left_agree})')
    return 1 if lowered or skipped or removed or unpaid or fell else 0


if __name__ == '__main__':
    sys.exit(main())
