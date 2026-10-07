#!/usr/bin/env python3
"""Run a command under a memory and time limit; kill its whole process group past either.

    watchdog.py <limit_mb> <timeout_s> <poll_ms> -- <command> [args...]

`timeout(1)` bounds time, not memory: an input that makes the loader allocate
without bound can swap the machine out well before any time limit. This runs the
command in its own process group, sums the group's resident memory every
<poll_ms>, and SIGKILLs the group once the sum exceeds <limit_mb> or the run
exceeds <timeout_s>, and once the command exits, whatever of it is still running. The verification protocol uses `watchdog.py 2500 <secs> 20`
for anything that loads an untrusted or submodule model, one process per file.

Prints one JSON line to stderr: {"rc", "killed", "peak_mb", "secs", "samples"};
"killed" is null or the reason. Exits with the command's status (negative
signal numbers become 128 + signal, as a shell reports them).
"""
import json
import os
import signal
import subprocess
import sys
import time


def group_rss_mb(pgid):
    out = subprocess.run(['ps', '-o', 'pgid=,rss=', '-A'], capture_output=True, text=True).stdout
    kb = sum(int(rss) for g, rss in (line.split() for line in out.splitlines() if len(line.split()) == 2)
             if g == str(pgid))
    return kb / 1024


def main():
    if '--' not in sys.argv or sys.argv.index('--') != 4 or len(sys.argv) < 6:
        sys.exit(__doc__)
    limit_mb, timeout_s, poll_s = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]) / 1000
    command = sys.argv[5:]
    start = time.time()
    proc = subprocess.Popen(command, start_new_session=True)
    peak, killed, samples = 0.0, None, 0
    while proc.poll() is None:
        rss = group_rss_mb(proc.pid)
        samples += 1
        peak = max(peak, rss)
        if rss > limit_mb:
            killed = f'rss {rss:.0f} MB > {limit_mb:.0f} MB'
        elif time.time() - start > timeout_s:
            killed = f'timeout {timeout_s:g} s'
        if killed:
            os.killpg(proc.pid, signal.SIGKILL)
            break
        time.sleep(poll_s)
    rc = proc.wait()
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except OSError:  # the group is already empty
        pass
    print(json.dumps({'rc': rc, 'killed': killed, 'peak_mb': round(peak, 1),
                      'secs': round(time.time() - start, 2), 'samples': samples}), file=sys.stderr)
    sys.exit(128 - rc if rc < 0 else rc)


if __name__ == '__main__':
    main()
