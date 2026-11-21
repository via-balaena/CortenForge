// Ported from Apple's Libc (stdlib/FreeBSD/qsort.c and heapsort.c, byte for
// byte the same at tags Libc-1534.100.14, Libc-1669.60.4 and Libc-1752.120.2),
// the `qsort` macOS's libSystem gives qhull, and changed to sort a slice
// through a Rust comparator.
//
// qsort.c:
// Copyright (c) 1992, 1993
// The Regents of the University of California.  All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.
//
// heapsort.c:
// Copyright (c) 1991, 1993
// The Regents of the University of California.  All rights reserved.
//
// This code is derived from software contributed to Berkeley by
// Ronnie Kon at Mindcraft Inc., Kevin Lew and Elmer Yglesias.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 4. Neither the name of the University nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

//! macOS's `qsort`, the sort of qhull's merges and of the vertices
//! `qh_find_newvertex` chooses from: a quicksort on a median of three (of nine
//! past 40 elements), insertion sort for short or unswapped runs, and
//! `heapsort` past a depth limit.
//! qhull's comparators never return equal, so where two elements tie the
//! order they are left in is this algorithm's, and the port makes the same
//! comparisons in the same order. An index stands for a pointer, and `cmp`
//! for C's comparator: [`Ordering::Less`] for a negative return,
//! [`Ordering::Greater`] for a positive one.

use std::cmp::Ordering;

/// `qsort` (`qsort.c`): `_qsort` with `DEPTH(n)`, `2 * (flsl(n) - 1)`.
pub(crate) fn qsort<T: Copy>(v: &mut [T], cmp: &mut impl FnMut(&T, &T) -> Ordering) {
    let n = v.len();
    let depth_limit = 2 * ((usize::BITS - n.leading_zeros()) as i32 - 1);
    qsort_at(v, 0, n, cmp, depth_limit);
}

/// `med3` (`qsort.c`): the median of three, by the comparisons C makes.
fn med3<T: Copy>(
    v: &[T],
    a: usize,
    b: usize,
    c: usize,
    cmp: &mut impl FnMut(&T, &T) -> Ordering,
) -> usize {
    if cmp(&v[a], &v[b]) == Ordering::Less {
        if cmp(&v[b], &v[c]) == Ordering::Less {
            b
        } else if cmp(&v[a], &v[c]) == Ordering::Less {
            c
        } else {
            a
        }
    } else if cmp(&v[b], &v[c]) == Ordering::Greater {
        b
    } else if cmp(&v[a], &v[c]) == Ordering::Less {
        a
    } else {
        c
    }
}

/// `_isort` (`qsort.c`): insertion sort of `v[a..a + n]`; with a nonzero
/// `swap_limit`, `false` once more swaps than it are made, the elements
/// left as they then are.
fn isort<T: Copy>(
    v: &mut [T],
    a: usize,
    n: usize,
    cmp: &mut impl FnMut(&T, &T) -> Ordering,
    swap_limit: usize,
) -> bool {
    let mut swap_cnt = 0;
    for pm in a + 1..a + n {
        let mut pl = pm;
        while pl > a && cmp(&v[pl - 1], &v[pl]) == Ordering::Greater {
            v.swap(pl, pl - 1);
            if swap_limit != 0 {
                swap_cnt += 1;
                if swap_cnt > swap_limit {
                    return false;
                }
            }
            pl -= 1;
        }
    }
    true
}

/// `vecswap` (`qsort.c`): `n` elements from `a` and from `b` swapped in turn.
fn vecswap<T>(v: &mut [T], a: usize, b: usize, n: usize) {
    for i in 0..n {
        v.swap(a + i, b + i);
    }
}

/// `_qsort` (`qsort.c`) on `v[a..a + n]`; its `goto loop` is the `loop`.
fn qsort_at<T: Copy>(
    v: &mut [T],
    mut a: usize,
    mut n: usize,
    cmp: &mut impl FnMut(&T, &T) -> Ordering,
    mut depth_limit: i32,
) {
    loop {
        let mut swap_cnt = false;
        if depth_limit <= 0 {
            heapsort(&mut v[a..a + n], cmp);
            return;
        }
        depth_limit -= 1;
        if n <= 7 {
            isort(v, a, n, cmp, 0);
            return;
        }
        let mut pl = a;
        let mut pm = a + n / 2;
        let mut pn = a + n - 1;
        if n > 40 {
            let d = n / 8;
            pl = med3(v, pl, pl + d, pl + 2 * d, cmp);
            pm = med3(v, pm - d, pm, pm + d, cmp);
            pn = med3(v, pn - 2 * d, pn - d, pn, cmp);
        }
        let pm = med3(v, pl, pm, pn, cmp);
        v.swap(a, pm);
        let mut pa = a + 1;
        let mut pb = a + 1;
        let mut pc = a + n - 1;
        let mut pd = a + n - 1;
        loop {
            while pb <= pc {
                let r = cmp(&v[pb], &v[a]);
                if r == Ordering::Greater {
                    break;
                }
                if r == Ordering::Equal {
                    swap_cnt = true;
                    v.swap(pa, pb);
                    pa += 1;
                }
                pb += 1;
            }
            while pb <= pc {
                let r = cmp(&v[pc], &v[a]);
                if r == Ordering::Less {
                    break;
                }
                if r == Ordering::Equal {
                    swap_cnt = true;
                    v.swap(pc, pd);
                    pd -= 1;
                }
                pc -= 1;
            }
            if pb > pc {
                break;
            }
            v.swap(pb, pc);
            swap_cnt = true;
            pb += 1;
            pc -= 1;
        }
        let pn = a + n;
        let d1 = (pa - a).min(pb - pa);
        vecswap(v, a, pb - d1, d1);
        let d1 = (pd - pc).min(pn - pd - 1);
        vecswap(v, pb, pn - d1, d1);
        if !swap_cnt && isort(v, a, n, cmp, 1 + n / 4) {
            return;
        }
        let d1 = pb - pa;
        let d2 = pd - pc;
        if d1 <= d2 {
            if d1 > 1 {
                qsort_at(v, a, d1, cmp, depth_limit);
            }
            if d2 <= 1 {
                return;
            }
            a = pn - d2;
            n = d2;
        } else {
            if d2 > 1 {
                qsort_at(v, pn - d2, d2, cmp, depth_limit);
            }
            if d1 <= 1 {
                return;
            }
            n = d1;
        }
    }
}

/// `heapsort` (`heapsort.c`): Knuth's, its elements numbered from 1 (`base`
/// one element below `v`), `CREATE` building the heap and `SELECT` taking
/// its top, with `k` the displaced element.
fn heapsort<T: Copy>(v: &mut [T], cmp: &mut impl FnMut(&T, &T) -> Ordering) {
    let mut nmemb = v.len();
    if nmemb <= 1 {
        return;
    }
    for l in (1..=nmemb / 2).rev() {
        heap_create(v, l, nmemb, cmp);
    }
    while nmemb > 1 {
        let k = v[nmemb - 1];
        v[nmemb - 1] = v[0];
        nmemb -= 1;
        heap_select(v, nmemb, k, cmp);
    }
}

/// `CREATE` (`heapsort.c`): element `initval` sifted down.
fn heap_create<T: Copy>(
    v: &mut [T],
    initval: usize,
    nmemb: usize,
    cmp: &mut impl FnMut(&T, &T) -> Ordering,
) {
    let mut par_i = initval;
    loop {
        let mut child_i = par_i * 2;
        if child_i > nmemb {
            return;
        }
        if child_i < nmemb && cmp(&v[child_i - 1], &v[child_i]) == Ordering::Less {
            child_i += 1;
        }
        if cmp(&v[child_i - 1], &v[par_i - 1]) != Ordering::Greater {
            return;
        }
        v.swap(par_i - 1, child_i - 1);
        par_i = child_i;
    }
}

/// `SELECT` (`heapsort.c`): the larger child copied up to the bottom, then
/// `k` placed on the way back.
fn heap_select<T: Copy>(v: &mut [T], nmemb: usize, k: T, cmp: &mut impl FnMut(&T, &T) -> Ordering) {
    let mut par_i = 1;
    loop {
        let mut child_i = par_i * 2;
        if child_i > nmemb {
            break;
        }
        if child_i < nmemb && cmp(&v[child_i - 1], &v[child_i]) == Ordering::Less {
            child_i += 1;
        }
        v[par_i - 1] = v[child_i - 1];
        par_i = child_i;
    }
    loop {
        let child_i = par_i;
        par_i = child_i / 2;
        if child_i == 1 || cmp(&k, &v[par_i - 1]) == Ordering::Less {
            v[child_i - 1] = k;
            return;
        }
        v[child_i - 1] = v[par_i - 1];
    }
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    /// One of `tests/data/qsort_orders.txt`'s comparators: `F` qhull's
    /// `qh_compare_facetmerge` over a merge type and a distance, `V` its
    /// `qh_comparevisit` over a visit id, `T` a three-way comparison.
    fn compare(form: &str, a: (u32, f64), b: (u32, f64)) -> Ordering {
        let greater = match form {
            "F" if a.0 != b.0 => a.0 < b.0,
            "F" => a.1 < b.1,
            "V" => a.0 > b.0,
            _ => return a.0.cmp(&b.0),
        };
        if greater {
            Ordering::Greater
        } else {
            Ordering::Less
        }
    }

    /// Each array of `tests/data/qsort_orders.txt` (made by
    /// `sim/L0/tests/scripts/gen_qsort_reference.py` with macOS's own `qsort`
    /// and, on an `H` line, its `heapsort`) is left in the order they left it.
    #[test]
    fn each_recorded_array_is_left_where_macos_qsort_leaves_it() {
        let data = include_str!("../tests/data/qsort_orders.txt");
        let mut cases = [0, 0];
        for line in data.lines().filter(|l| !l.starts_with('#')) {
            let (head, order) = line.split_once(" |").expect("a case");
            let mut words = head.split(' ');
            let word = words.next().expect("a comparator");
            let (by_heapsort, form) = word
                .strip_prefix('H')
                .map_or((false, word), |form| (true, form));
            let keys: Vec<(u32, f64)> = words
                .map(|k| match k.split_once(':') {
                    Some((t, d)) => (t.parse().expect("a type"), d.parse().expect("a distance")),
                    None => (k.parse().expect("a key"), 0.0),
                })
                .collect();
            let want: Vec<usize> = order
                .split_whitespace()
                .map(|i| i.parse().expect("an index"))
                .collect();
            let mut got: Vec<usize> = (0..keys.len()).collect();
            let mut cmp = |&a: &usize, &b: &usize| compare(form, keys[a], keys[b]);
            if by_heapsort {
                heapsort(&mut got, &mut cmp);
            } else {
                qsort(&mut got, &mut cmp);
            }
            assert_eq!(got, want, "{line}");
            cases[usize::from(by_heapsort)] += 1;
        }
        assert_eq!(cases, [800, 200], "recorded cases");
    }
}
