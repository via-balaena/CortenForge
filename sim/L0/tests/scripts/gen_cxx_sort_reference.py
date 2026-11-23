#!/usr/bin/env python3
"""Generate the libc++ sort golden for sim-core from macOS's own libc++ `std::sort`.

Run it on macOS with the Command Line Tools: the golden is the order the
`std::sort` of the libc++ `/usr/bin/c++` builds against leaves (TriangleMeshDistance
sorts a mesh's faces with it):

    python3 -I sim/L0/tests/scripts/gen_cxx_sort_reference.py sim/L0/core/tests/data/cxx_sort_orders.txt

It refuses any other platform. It writes a C++ program to a temporary
directory, builds it with `/usr/bin/c++ -std=c++17 -O2`, and runs it; the
program prints the libc++ version its headers declare (`_LIBCPP_VERSION`),
which the file's header records. Nothing checks that version against the
libc++ the oracle's MuJoCo was built with (`build_mujoco_oracle.sh` takes
cmake's default compiler); the octree golden, made by that MuJoCo, holds a
case whose octree a stable sort changes.

Each case is an array of elements `{key, id}`, `id` its index, sorted by
`key` (a double, compared with `<`, as TriangleMeshDistance compares a face's
first vertex's coordinate): by `std::sort`, or by `std::partial_sort(first,
last, last)`, the heap sort `std::sort` falls back to past its depth limit;
then the order the ids are left in. In the file's order: arrays from one
fixed xorshift stream (as the other generators here), random keys, or those
keys sorted, reversed, all equal, in an organ pipe or a sawtooth, drawn from
one, two, three, five, ten or n values, mostly short, some past the 128
elements over which `std::sort` takes a ninther for its pivot; arrays of
exactly 128 and 129 elements, about that size; arrays built by McIlroy's
adversary ("A Killer Adversary for Quicksort", 1999), whose keys are fixed as
`std::sort` compares them so that each partition splits off few elements,
driving it to its depth limit and the heap sort, each also with its keys
divided by two and by three so that some tie; and arrays sorted but for a few
keys swapped, from a second stream, of which only those listed in
NEARLY_SORTED_CHOSEN are kept: each was chosen because the port, with the
answer of `std::sort`'s bounded insertion sort at its limit of moves made
"not sorted", leaves it in another order (that answer decides whether the
range is sorted further, and rarely changes an order).

The file is one case per line, '#' lines aside:

    S <key>... | <order>...     (std::sort)
    H <key>... | <order>...     (std::partial_sort(first, last, last))
    A <key>... | <order>...     (std::sort of an adversary's own array)

A key is an integer.
"""
import os
import platform
import subprocess
import sys
import tempfile

COUNT = 600
HEAP_COUNT = 150
# Indices into the nearly sorted stream (Stream(2), `nearly_sorted`) of the arrays kept.
NEARLY_SORTED_CHOSEN = (14342, 16784, 25084, 25520, 29049, 29130)
# The 128 elements past which std::sort takes a ninther, exactly, and once more.
EXACT_SIZES = (128, 129)
ADVERSARY_SIZES = (64, 129, 300, 1000)
# Each adversary's array also with its keys divided by these (integer division): ties are what let an order tell
# one path through the sort from another, and an adversary's keys are distinct but for its leftover gas.
ADVERSARY_DIVISORS = (1, 2, 3)


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


def nearly_sorted(s):
    """24 to 223 keys drawn from 2, 3, 5, 10 or 50 values, sorted, then up to 11 pairs of them swapped."""
    n = 24 + s.below(200)
    distinct = (2, 3, 5, 10, 50)[s.below(5)]
    keys = sorted(s.below(distinct) for _ in range(n))
    for _ in range(s.below(12)):
        i, j = s.below(n), s.below(n)
        keys[i], keys[j] = keys[j], keys[i]
    return keys


def make_case(s):
    r = s.below(100)
    n = s.below(30) if r < 75 else 30 + s.below(130) if r < 97 else 160 + s.below(240)
    distinct = (1, 2, 3, 5, 10, max(n, 1))[s.below(6)]
    keys = [s.below(distinct) for _ in range(n)]
    form = s.below(6)
    if form == 1:
        keys.sort()
    elif form == 2:
        keys.sort(reverse=True)
    elif form == 3:
        keys = [keys[0]] * n if n else []
    elif form == 4:
        keys = [min(i, n - 1 - i) % max(distinct, 1) for i in range(n)]
    elif form == 5:
        keys = [i % max(distinct, 1) for i in range(n)]
    return keys


PROGRAM = r"""
#include <algorithm>
#include <cstdio>
#include <vector>
struct E { double key; int id; };
static int ngas, nsolid, candidate;
static std::vector<int> val;
static bool adversary(const int& x, const int& y) {
  // McIlroy's gas: a key not yet fixed is "gas"; comparing two gases freezes one (the candidate first).
  if (val[x] == ngas && val[y] == ngas) { if (x == candidate) val[x] = nsolid++; else val[y] = nsolid++; }
  if (val[x] == ngas) candidate = x; else if (val[y] == ngas) candidate = y;
  return val[x] < val[y];
}
int main() {
  std::printf("%ld\n", (long)_LIBCPP_VERSION);
  char kind[4]; int n;
  while (std::scanf("%3s %d", kind, &n) == 2) {
    if (kind[0] == 'A') {
      ngas = n; nsolid = 0; candidate = 0; val.assign(n, n);
      std::vector<int> ptr(n);
      for (int i = 0; i < n; i++) ptr[i] = i;
      std::sort(ptr.begin(), ptr.end(), adversary);
      for (int i = 0; i < n; i++) std::printf("%d ", val[i]);
      std::printf("\n");
      continue;
    }
    std::vector<E> v(n);
    for (int i = 0; i < n; i++) { std::scanf("%lf", &v[i].key); v[i].id = i; }
    auto less = [](const E& a, const E& b) { return a.key < b.key; };
    if (kind[0] == 'S') std::sort(v.begin(), v.end(), less);
    else std::partial_sort(v.begin(), v.end(), v.end(), less);
    for (int i = 0; i < n; i++) std::printf("%d ", v[i].id);
    std::printf("\n");
  }
}
"""


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    if platform.system() != "Darwin":
        sys.exit("run this on macOS: the golden is macOS's libc++ std::sort")
    with tempfile.TemporaryDirectory() as d:
        src, exe = os.path.join(d, "sort.cc"), os.path.join(d, "sort")
        with open(src, "w") as f:
            f.write(PROGRAM)
        subprocess.run(["/usr/bin/c++", "-std=c++17", "-O2", "-o", exe, src], check=True)

        def run(lines):
            out = subprocess.run([exe], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
            rows = out.stdout.split("\n")
            return rows[0], [[int(x) for x in r.split()] for r in rows[1:len(lines) + 1]]

        # The adversary's arrays first: their keys are fixed as std::sort compares them.
        version, adv_keys = run([f"A {n}" for n in ADVERSARY_SIZES])
        s = Stream(1)
        cases = [("S", make_case(s)) for _ in range(COUNT)] + [("H", make_case(s)) for _ in range(HEAP_COUNT)]
        for n in EXACT_SIZES:
            for distinct in (2, 3, 5):
                cases.append(("S", [s.below(distinct) for _ in range(n)]))
        # The adversary's own arrays as `A` (the test asks that each takes the heap sort); their keys divided as
        # `S`, for some of them take no heap sort (those of 64 and 129 elements).
        cases += [("A" if d == 1 else "S", [x // d for x in k]) for k in adv_keys for d in ADVERSARY_DIVISORS]
        near = Stream(2)
        stream = [nearly_sorted(near) for _ in range(max(NEARLY_SORTED_CHOSEN, default=-1) + 1)]
        cases += [("S", stream[i]) for i in NEARLY_SORTED_CHOSEN]
        _, orders = run([f"{'S' if k == 'A' else k} {len(keys)} " + " ".join(str(x) for x in keys) for k, keys in cases])
    out = [f"# libc++ {version} (macOS {platform.mac_ver()[0]}, {platform.machine()}), std::sort and "
           "std::partial_sort, from sim/L0/tests/scripts/gen_cxx_sort_reference.py"]
    for (kind, keys), order in zip(cases, orders):
        assert sorted(order) == list(range(len(keys)))
        out.append(f"{kind} {' '.join(str(x) for x in keys)} | {' '.join(str(x) for x in order)}".replace("  ", " "))
    with open(sys.argv[1], "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"{len(cases)} cases, libc++ {version} -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
