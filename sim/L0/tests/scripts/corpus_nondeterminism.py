#!/usr/bin/env python3
"""List the census docs whose model or trajectory varies between processes.

    corpus_nondeterminism.py <corpus_harness binary> [runs=10] [--keep <dir>]

Runs the harness (scripts/corpus_harness.rs) <runs> times, each in its own
process so each gets its own hash seeds, over every doc of the parity-census
snapshot (sim/L0/tests/assets/census/docs), and prints the docs whose
model_fp, traj_fp or load/step outcome differs between runs. --keep writes each
run's output to <dir>/run_<i>.jsonl, for comparing a commit with its parent.
Exits 1 if any doc varies. Read-only on the repository.
"""
import json
import os
import subprocess
import sys


def main():
    args = sys.argv[1:]
    keep = None
    if '--keep' in args:
        i = args.index('--keep')
        keep = args[i + 1]
        del args[i:i + 2]
    if not 1 <= len(args) <= 2:
        sys.exit(__doc__)
    harness, runs = args[0], int(args[1]) if len(args) == 2 else 10
    root = subprocess.run(['git', 'rev-parse', '--show-toplevel'], capture_output=True, text=True,
                          check=True).stdout.strip()
    docs = os.path.join(root, 'sim', 'L0', 'tests', 'assets', 'census', 'docs')
    stdin = ''.join(f'str\t{os.path.join(docs, f)}\n' for f in sorted(os.listdir(docs)) if f.endswith('.xml'))
    results = []
    for i in range(1, runs + 1):
        out = subprocess.run([harness], input=stdin, capture_output=True, text=True, check=True, timeout=600).stdout
        if keep:
            os.makedirs(keep, exist_ok=True)
            with open(os.path.join(keep, f'run_{i}.jsonl'), 'w') as f:
                f.write(out)
        results.append({json.loads(line)['path']: json.loads(line) for line in out.splitlines()})
    varying = {}
    for path in sorted(results[0]):
        fields = [k for k in ('model_fp', 'traj_fp', 'status', 'err', 'traj', 'traj_err')
                  if len({r[path].get(k) for r in results}) > 1]
        if fields:
            varying[os.path.basename(path)[:-len('.xml')]] = fields
    model = sum('model_fp' in f for f in varying.values())
    traj = sum('traj_fp' in f for f in varying.values())
    outcome = sum(any(k not in ('model_fp', 'traj_fp') for k in f) for f in varying.values())
    print(f'{runs} runs, {len(results[0])} docs: {len(varying)} vary '
          f'(model {model}, trajectory {traj}, outcome {outcome})')
    for doc, fields in varying.items():
        print(f'{doc}\t{",".join(fields)}')
    return 1 if varying else 0


if __name__ == '__main__':
    sys.exit(main())
