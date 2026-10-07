#!/usr/bin/env python3
"""Refuse a census verdict lowered by hand without a note.

    check_census_verdicts.py <from-ref> <to-ref> <label>

The census test compares each doc's verdict with sim/L0/tests/assets/census/
verdicts.tsv, so it cannot tell a row a commit lowered by hand from one the
test wrote. This compares the file at the two refs: a doc whose class ranks
lower at <to-ref> must carry a `divergence=<ID>`, `known=<label>` or
`fixed_by=<Pnn|Lnn> was=<class>` note there, a was= class ranking at least as
high as the doc's class at <from-ref>; and no row may gain a `nondet=` note,
which makes the census skip it, a new doc included. A doc missing at
<from-ref> cannot have been lowered. `klass`, `rank` and the note forms mirror class, rank, is_class and
fixed_by in sim/L0/tests/mujoco_conformance/layer_e_census/ratchet.rs.
Exits 1 on either.
"""
import re
import subprocess
import sys

PATH = 'sim/L0/tests/assets/census/verdicts.tsv'


def rows(ref):
    show = subprocess.run(['git', 'show', f'{ref}:{PATH}'], capture_output=True, text=True)
    if show.returncode != 0:
        return {}
    out = {}
    for line in show.stdout.splitlines():
        if line.startswith('#') or not line.strip():
            continue
        cols = line.split('\t')
        out[cols[0]] = (cols[1] if len(cols) > 1 else '', cols[2] if len(cols) > 2 else '')
    return out


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


def explained(note, before):
    """Whether `note` accounts for a row lowered from the verdict `before`."""
    for prefix in ('divergence=', 'known='):
        if note.startswith(prefix):
            return len(note) > len(prefix)
    m = re.fullmatch(r'fixed_by=[PL][0-9]+[a-z]? was=(.+)', note)
    return bool(m) and is_class(m.group(1)) and rank(m.group(1)) >= rank(klass(before))


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    before, after, label = rows(sys.argv[1]), rows(sys.argv[2]), sys.argv[3]
    lowered = [(doc, before[doc][0], verdict, note) for doc, (verdict, note) in sorted(after.items())
               if doc in before and rank(klass(verdict)) < rank(klass(before[doc][0]))
               and not explained(note, before[doc][0])]
    for doc, was, now, note in lowered:
        print(f'::error::{label}: {doc} lowered from {was} to {now} without a divergence=<ID>, '
              f'known=<label> or fixed_by=<Pnn|Lnn> was=<class at least {klass(was)}> note '
              f'(has {note!r})')
    skipped = [doc for doc, (_, note) in sorted(after.items())
               if note.startswith('nondet=') and not before.get(doc, ('', ''))[1].startswith('nondet=')]
    for doc in skipped:
        print(f'::error::{label}: {doc} gained a nondet= note; the census would stop checking it')
    return 1 if lowered or skipped else 0


if __name__ == '__main__':
    sys.exit(main())
