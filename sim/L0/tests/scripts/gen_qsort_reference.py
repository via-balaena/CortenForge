#!/usr/bin/env python3
"""Generate the qsort golden for sim-qhull from macOS's own `qsort`.

Run it on macOS (the `qsort` qhull calls when MuJoCo runs there):

    python3 -I sim/L0/tests/scripts/gen_qsort_reference.py sim/L0/qhull/tests/data/qsort_orders.txt

It refuses any other platform.

Each case is an array of element indices 0..n with a key per element, sorted
by libSystem's `qsort` through `ctypes` under one of three comparators, and
the order `qsort` leaves the indices in; then cases sorted by libSystem's
`heapsort`, which `qsort` falls back to past its depth limit. Two comparators
are qhull's, which never return equal: `F`, `qh_compare_facetmerge` over a
merge type and a distance (`a.type < b.type ? 1 : -1` where the types differ,
else `a.dist < b.dist ? 1 : -1`), and `V`, `qh_comparevisit` over a visit id
(`a > b ? 1 : -1`). `T` is a three-way comparison of a key, so that the
branches for an equal return are taken too. The arrays come from one fixed
xorshift stream: random keys, or those keys sorted, reversed, all equal, in an
organ pipe or a sawtooth. Last come chosen cases of a second stream
(`deep_case`): 41 to 400 elements whose keys are drawn from one, two, three
or five values (an `F` key's distance; its type is one of up to three), each
chosen because a change to the port that the cases before it miss leaves it
in another order (`DEEP_CHOSEN`).

The file is one case per line, '#' lines aside:

    F <type>:<dist>... | <order>...
    V <key>... | <order>...
    T <key>... | <order>...

and `HF`, `HV`, `HT` for a case `heapsort` sorted.

A distance is an integer, `-0` for negative zero.
"""
import ctypes
import platform
import sys

COUNT = 800
HEAP_COUNT = 200
# The second stream's cases the file takes, by index into deep_case(Stream(2)): 929 and 1151, the first `F` and the
# first `V` case of the stream that the port leaves in another order when its right-hand recursion is given one depth
# less (RL-9d2b review, RB-1), a change the cases above miss.
DEEP_CHOSEN = (929, 1151)


class Stream:
    """xorshift64, as the other generators here."""

    def __init__(self, seed):
        self.mask = (1 << 64) - 1
        self.x = (seed * 0x9E3779B97F4A7C15) & self.mask | 1

    def bits(self):
        x = self.x
        x ^= (x << 13) & self.mask
        x ^= x >> 7
        x ^= (x << 17) & self.mask
        self.x = x
        return x

    def below(self, n):
        return self.bits() % n


def make_case(s, small=False):
    """One case: its form and its keys, from the stream (`small`: n below 65)."""
    r = s.below(100)
    n = s.below(17) if r < 60 else 17 + s.below(48) if r < 90 or small else 65 + s.below(136)
    form = 'FVT'[s.below(3)]
    distinct = (1, 2, 3, 5, 10, max(n, 1))[s.below(6)]
    types = 1 + s.below(3)
    keys = []
    for _ in range(n):
        k = s.below(distinct)
        if form == 'F':
            d = -k if s.below(16) == 0 else k
            neg_zero = d == 0 and s.below(2) == 0
            keys.append((1 + s.below(types), d, neg_zero))
        else:
            keys.append((k, 0, False))
    pattern = 0 if s.below(2) == 0 else s.below(6)
    order_key = lambda k: (k[0], k[1])
    if pattern == 1:
        keys.sort(key=order_key)
    elif pattern == 2:
        keys.sort(key=order_key, reverse=True)
    elif pattern == 3 and keys:
        keys = [keys[0]] * n
    elif pattern == 4:
        keys.sort(key=order_key)
        keys[n // 2:] = keys[n // 2:][::-1]
    elif pattern == 5:
        m = 1 + s.below(8)
        keys = [(i % m if form != 'F' else 1 + i % m % types, i % m if form == 'F' else 0, False) for i in range(n)]
    return form, keys


def deep_case(s):
    """A case of the second stream: 41 to 400 elements whose keys are drawn from one, two, three or five values (an
    `F` key's distance; its type is one of up to three), random, sorted, reversed or in an organ pipe."""
    n = 41 + s.below(360)
    form = 'FVT'[s.below(3)]
    distinct = (1, 2, 3, 5)[s.below(4)]
    types = 1 + s.below(3)
    keys = []
    for _ in range(n):
        k = s.below(distinct)
        keys.append((1 + s.below(types), k, False) if form == 'F' else (k, 0, False))
    pattern = s.below(4)
    order_key = lambda k: (k[0], k[1])
    if pattern == 1:
        keys.sort(key=order_key)
    elif pattern == 2:
        keys.sort(key=order_key, reverse=True)
    elif pattern == 3:
        keys.sort(key=order_key)
        keys[n // 2:] = keys[n // 2:][::-1]
    return form, keys


def compare(form, keys, x, y):
    a, b = keys[x], keys[y]
    if form == 'F':
        if a[0] != b[0]:
            return 1 if a[0] < b[0] else -1
        return 1 if a[1] < b[1] else -1
    if form == 'V':
        return 1 if a[0] > b[0] else -1
    return (a[0] > b[0]) - (a[0] < b[0])


def main():
    if len(sys.argv) != 2:
        sys.exit('usage: gen_qsort_reference.py <out.txt>')
    if platform.system() != 'Darwin':
        sys.exit("refusing: the reference is macOS's qsort")
    libc = ctypes.CDLL('/usr/lib/libSystem.B.dylib')
    cmp_t = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t))
    libc.qsort.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, cmp_t]
    libc.qsort.restype = None
    libc.heapsort.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, cmp_t]
    libc.heapsort.restype = ctypes.c_int
    s = Stream(1)
    lines = ["# macOS's qsort (libSystem, %s-%s), from sim/L0/tests/scripts/gen_qsort_reference.py"
             % (platform.system(), platform.machine())]
    cases = [(case >= COUNT,) + make_case(s, small=case >= COUNT) for case in range(COUNT + HEAP_COUNT)]
    d = Stream(2)
    stream2 = [deep_case(d) for _ in range(max(DEEP_CHOSEN, default=-1) + 1)]
    cases += [(False,) + stream2[i] for i in DEEP_CHOSEN]
    for heap, form, keys in cases:
        n = len(keys)
        arr = (ctypes.c_size_t * n)(*range(n))
        cb = cmp_t(lambda p, q: compare(form, keys, p[0], q[0]))
        if heap:
            if libc.heapsort(arr, n, ctypes.sizeof(ctypes.c_size_t), cb) != 0:
                sys.exit('heapsort failed')
        else:
            libc.qsort(arr, n, ctypes.sizeof(ctypes.c_size_t), cb)
        if form == 'F':
            ks = ['%d:%s' % (t, '-0' if nz else str(d)) for t, d, nz in keys]
        else:
            ks = [str(k[0]) for k in keys]
        lines.append(' '.join([('H' if heap else '') + form] + ks) + ' |' + ''.join(' %d' % i for i in arr))
    with open(sys.argv[1], 'w') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
