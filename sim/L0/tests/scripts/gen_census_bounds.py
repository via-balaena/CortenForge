#!/usr/bin/env python3
"""Write MuJoCo 3.5.0's geom bounds for the census docs whose golden predates them.

Run it with the oracle's own interpreter, as gen_census_golden.py:

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_census_bounds.py \\
        sim/L0/tests/assets/census/docs sim/L0/tests/assets/census/golden \\
        sim/L0/tests/assets/census/bounds.json

A golden's model holds geom_rbound and geom_aabb since Rigid-loading PR RL-9b3.
The goldens written before it are append-only and never rewritten, so their two
fields live in one file beside them, written once, by this script: for every
golden that loads ("ok") and holds no geom_rbound, the doc loaded as
gen_census_golden.py loads it, and the two fields as its model_json writes them.
It refuses an interpreter that is not the oracle golden/meta.json names, and an
existing output file (the file is append-only like the golden: never rewritten).

    {"oracle": <golden/meta.json's oracle>, "docs": {"<id>": {"geom_rbound": [...], "geom_aabb": [[...], ...]}}}
"""
import json
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, floats, oracle, rows  # noqa: E402


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    docs, golden, out = sys.argv[1:]
    if os.path.exists(out):
        sys.exit(f'{out} exists: it is written once and never rewritten')
    built = oracle()
    with open(os.path.join(golden, 'meta.json')) as f:
        blessed = json.load(f)['oracle']
    if blessed != built:
        sys.exit(f'{golden} was blessed by the oracle {blessed}; this one is {built}')
    entries = {}
    for name in sorted(os.listdir(golden)):
        if not name.endswith('.json') or name == 'meta.json':
            continue
        with open(os.path.join(golden, name)) as f:
            record = json.load(f)
        if record.get('status') != 'ok' or 'geom_rbound' in record['model']:
            continue
        doc = name[:-len('.json')]
        with open(os.path.join(docs, doc + '.xml'), encoding='utf-8') as f:
            m = mujoco.MjModel.from_xml_string(f.read())
        if m.ngeom != record['model']['ngeom']:
            sys.exit(f'{doc}: the oracle gives {m.ngeom} geoms, its golden {record["model"]["ngeom"]}')
        entries[doc] = finite_or_string({'geom_rbound': floats(m.geom_rbound),
                                         'geom_aabb': rows(m.geom_aabb, 6)})
    with open(out, 'w') as f:
        f.write(json.dumps({'oracle': blessed, 'docs': entries}, separators=(',', ':'),
                           sort_keys=True) + '\n')
    print(f'{len(entries)} docs -> {out}')


if __name__ == '__main__':
    main()
