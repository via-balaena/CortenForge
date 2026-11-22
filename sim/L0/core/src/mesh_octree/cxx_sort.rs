// Ported from LLVM's libc++ (`__algorithm/sort.h`, `partial_sort.h`,
// `make_heap.h`, `sort_heap.h`, `pop_heap.h`, `push_heap.h` and
// `sift_down.h`, as macOS's MacOSX27.2 SDK ships them, `_LIBCPP_VERSION`
// 220106), the `std::sort` MuJoCo 3.5.0's TriangleMeshDistance sorts with when
// MuJoCo is built there, and changed to sort a slice of `Copy` values through
// a Rust comparator; by the CortenForge authors, 2026-11. Each of those files
// carries this notice:
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

//! libc++'s `std::sort` for a type that is not arithmetic: its introsort
//! (`__introsort`, without the bitset partition, which it takes only for an
//! arithmetic type under `std::less` or `std::greater`), its insertion sorts,
//! its two partitions and its heap sort past the depth limit
//! (`__partial_sort`). `std::sort` is not stable: where the comparator holds
//! two elements equal, where it leaves each is this algorithm's, and so is a
//! tree built from its order (TriangleMeshDistance's,
//! [`super::distance`]).

/// Sorts `v` as libc++'s `std::sort(first, last, less)` does, `less` being a
/// strict weak order (`a` before `b`).
pub fn sort<T: Copy>(v: &mut [T], less: &mut impl FnMut(&T, &T) -> bool) {
    if v.is_empty() {
        return;
    }
    // `2 * std::__bit_log2(len)`.
    let depth = 2 * v.len().ilog2() as usize;
    introsort(v, 0, v.len(), less, depth, true);
}

/// `__sort3`: orders `v[x]`, `v[y]`, `v[z]`, as libc++'s branching form does.
fn sort3<T: Copy>(
    v: &mut [T],
    x: usize,
    y: usize,
    z: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) {
    if !less(&v[y], &v[x]) {
        if !less(&v[z], &v[y]) {
            return;
        }
        v.swap(y, z);
        if less(&v[y], &v[x]) {
            v.swap(x, y);
        }
        return;
    }
    if less(&v[z], &v[y]) {
        v.swap(x, z);
        return;
    }
    v.swap(x, y);
    if less(&v[z], &v[y]) {
        v.swap(y, z);
    }
}

/// `__sort4`.
fn sort4<T: Copy>(v: &mut [T], x: [usize; 4], less: &mut impl FnMut(&T, &T) -> bool) {
    sort3(v, x[0], x[1], x[2], less);
    if less(&v[x[3]], &v[x[2]]) {
        v.swap(x[2], x[3]);
        if less(&v[x[2]], &v[x[1]]) {
            v.swap(x[1], x[2]);
            if less(&v[x[1]], &v[x[0]]) {
                v.swap(x[0], x[1]);
            }
        }
    }
}

/// `__sort5`.
fn sort5<T: Copy>(v: &mut [T], x: [usize; 5], less: &mut impl FnMut(&T, &T) -> bool) {
    sort4(v, [x[0], x[1], x[2], x[3]], less);
    if less(&v[x[4]], &v[x[3]]) {
        v.swap(x[3], x[4]);
        if less(&v[x[3]], &v[x[2]]) {
            v.swap(x[2], x[3]);
            if less(&v[x[2]], &v[x[1]]) {
                v.swap(x[1], x[2]);
                if less(&v[x[1]], &v[x[0]]) {
                    v.swap(x[0], x[1]);
                }
            }
        }
    }
}

/// `__insertion_sort` over `v[first..last]`.
fn insertion_sort<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) {
    if first == last {
        return;
    }
    for i in first + 1..last {
        if less(&v[i], &v[i - 1]) {
            let t = v[i];
            let mut k = i - 1;
            let mut j = i;
            loop {
                v[j] = v[k];
                j = k;
                if j == first {
                    break;
                }
                k -= 1;
                if !less(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
        }
    }
}

/// `__insertion_sort_unguarded` over `v[first..last]`: `v[first - 1]` goes
/// before none of them, so the inner loop needs no bound.
fn insertion_sort_unguarded<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) {
    if first == last {
        return;
    }
    for i in first + 1..last {
        if less(&v[i], &v[i - 1]) {
            let t = v[i];
            let mut k = i - 1;
            let mut j = i;
            loop {
                v[j] = v[k];
                j = k;
                k -= 1;
                if !less(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
        }
    }
}

/// `__insertion_sort_incomplete` over `v[first..last]`: sorts it, or stops
/// after its eighth move, returning whether it is sorted.
#[allow(clippy::many_single_char_names)] // libc++'s names
fn insertion_sort_incomplete<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) -> bool {
    const LIMIT: u32 = 8;
    match last - first {
        0 | 1 => return true,
        2 => {
            if less(&v[last - 1], &v[first]) {
                v.swap(first, last - 1);
            }
            return true;
        }
        3 => {
            sort3(v, first, first + 1, last - 1, less);
            return true;
        }
        4 => {
            sort4(v, [first, first + 1, first + 2, last - 1], less);
            return true;
        }
        5 => {
            sort5(v, [first, first + 1, first + 2, first + 3, last - 1], less);
            return true;
        }
        _ => {}
    }
    let mut j = first + 2;
    sort3(v, first, first + 1, j, less);
    let mut count = 0;
    let mut i = j + 1;
    while i != last {
        if less(&v[i], &v[j]) {
            let t = v[i];
            let mut k = j;
            j = i;
            loop {
                v[j] = v[k];
                j = k;
                if j == first {
                    break;
                }
                k -= 1;
                if !less(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
            count += 1;
            if count == LIMIT {
                return i + 1 == last;
            }
        }
        j = i;
        i += 1;
    }
    true
}

/// `__partition_with_equals_on_right` over `v[first..last]` about
/// `v[first]`: the pivot's place, and whether the range was partitioned
/// already.
fn partition_with_equals_on_right<T: Copy>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) -> (usize, bool) {
    let begin = first;
    let pivot = v[first];
    loop {
        first += 1;
        if !less(&v[first], &pivot) {
            break;
        }
    }
    if begin == first - 1 {
        while first < last {
            last -= 1;
            if less(&v[last], &pivot) {
                break;
            }
        }
    } else {
        loop {
            last -= 1;
            if less(&v[last], &pivot) {
                break;
            }
        }
    }
    let already_partitioned = first >= last;
    while first < last {
        v.swap(first, last);
        loop {
            first += 1;
            if !less(&v[first], &pivot) {
                break;
            }
        }
        loop {
            last -= 1;
            if less(&v[last], &pivot) {
                break;
            }
        }
    }
    let pivot_pos = first - 1;
    if begin != pivot_pos {
        v[begin] = v[pivot_pos];
    }
    v[pivot_pos] = pivot;
    (pivot_pos, already_partitioned)
}

/// `__partition_with_equals_on_left` over `v[first..last]` about
/// `v[first]`, which no element before `first` goes after: the elements
/// equal to the pivot go left of it. The first index past them.
fn partition_with_equals_on_left<T: Copy>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) -> usize {
    let begin = first;
    let pivot = v[first];
    if less(&pivot, &v[last - 1]) {
        loop {
            first += 1;
            if less(&pivot, &v[first]) {
                break;
            }
        }
    } else {
        loop {
            first += 1;
            if first >= last || less(&pivot, &v[first]) {
                break;
            }
        }
    }
    if first < last {
        loop {
            last -= 1;
            if !less(&pivot, &v[last]) {
                break;
            }
        }
    }
    while first < last {
        v.swap(first, last);
        loop {
            first += 1;
            if less(&pivot, &v[first]) {
                break;
            }
        }
        loop {
            last -= 1;
            if !less(&pivot, &v[last]) {
                break;
            }
        }
    }
    let pivot_pos = first - 1;
    if begin != pivot_pos {
        v[begin] = v[pivot_pos];
    }
    v[pivot_pos] = pivot;
    first
}

/// `__introsort` over `v[first..last]`, with `depth` partitions left before
/// it takes the heap sort; `leftmost` where nothing lies left of `first`.
fn introsort<T: Copy>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
    mut depth: usize,
    mut leftmost: bool,
) {
    const LIMIT: usize = 24;
    const NINTHER_THRESHOLD: usize = 128;
    loop {
        let len = last - first;
        match len {
            0 | 1 => return,
            2 => {
                if less(&v[last - 1], &v[first]) {
                    v.swap(first, last - 1);
                }
                return;
            }
            3 => {
                sort3(v, first, first + 1, last - 1, less);
                return;
            }
            4 => {
                sort4(v, [first, first + 1, first + 2, last - 1], less);
                return;
            }
            5 => {
                sort5(v, [first, first + 1, first + 2, first + 3, last - 1], less);
                return;
            }
            _ => {}
        }
        if len < LIMIT {
            if leftmost {
                insertion_sort(v, first, last, less);
            } else {
                insertion_sort_unguarded(v, first, last, less);
            }
            return;
        }
        if depth == 0 {
            #[cfg(test)]
            tests::HEAP_SORTS.with(|n| n.set(n.get() + 1));
            heap_sort(v, first, last, less);
            return;
        }
        depth -= 1;
        let half = len / 2;
        if len > NINTHER_THRESHOLD {
            sort3(v, first, first + half, last - 1, less);
            sort3(v, first + 1, first + (half - 1), last - 2, less);
            sort3(v, first + 2, first + (half + 1), last - 3, less);
            sort3(
                v,
                first + (half - 1),
                first + half,
                first + (half + 1),
                less,
            );
            v.swap(first, first + half);
        } else {
            sort3(v, first + half, first, last - 1, less);
        }
        if !leftmost && !less(&v[first - 1], &v[first]) {
            first = partition_with_equals_on_left(v, first, last, less);
            continue;
        }
        let (i, already_partitioned) = partition_with_equals_on_right(v, first, last, less);
        if already_partitioned {
            let left_sorted = insertion_sort_incomplete(v, first, i, less);
            if insertion_sort_incomplete(v, i + 1, last, less) {
                if left_sorted {
                    return;
                }
                last = i;
                continue;
            }
            if left_sorted {
                first = i + 1;
                continue;
            }
        }
        introsort(v, first, i, less, depth, leftmost);
        leftmost = false;
        first = i + 1;
    }
}

/// `__partial_sort(first, last, last)`: a heap of `v[first..last]`
/// (`__make_heap`), then sorted from it (`__sort_heap`).
fn heap_sort<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) {
    let n = last - first;
    if n == 0 {
        return;
    }
    if n > 1 {
        for start in (0..=(n - 2) / 2).rev() {
            sift_down(v, first, n, start, less);
        }
    }
    let mut end = last;
    let mut len = n;
    while len > 1 {
        pop_heap(v, first, end, len, less);
        end -= 1;
        len -= 1;
    }
}

/// `__sift_down` of the heap at `first`, `len` long, from `start`.
fn sift_down<T: Copy>(
    v: &mut [T],
    first: usize,
    len: usize,
    mut start: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) {
    let mut child = start;
    if len < 2 || (len - 2) / 2 < child {
        return;
    }
    child = 2 * child + 1;
    if child + 1 < len && less(&v[first + child], &v[first + child + 1]) {
        child += 1;
    }
    if less(&v[first + child], &v[first + start]) {
        return;
    }
    let top = v[first + start];
    loop {
        v[first + start] = v[first + child];
        start = child;
        if (len - 2) / 2 < child {
            break;
        }
        child = 2 * child + 1;
        if child + 1 < len && less(&v[first + child], &v[first + child + 1]) {
            child += 1;
        }
        if less(&v[first + child], &top) {
            break;
        }
    }
    v[first + start] = top;
}

/// `__pop_heap` of the heap `v[first..last]`, `len` long: its top moved to
/// `last - 1`, through `__floyd_sift_down` and `__sift_up`.
fn pop_heap<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    len: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) {
    if len <= 1 {
        return;
    }
    let top = v[first];
    let mut hole = floyd_sift_down(v, first, len, less);
    let last = last - 1;
    if hole == last {
        v[hole] = top;
    } else {
        v[hole] = v[last];
        hole += 1;
        v[last] = top;
        sift_up(v, first, hole, hole - first, less);
    }
}

/// `__floyd_sift_down`: the hole at the top moved down to a leaf along the
/// larger children; its index.
fn floyd_sift_down<T: Copy>(
    v: &mut [T],
    first: usize,
    len: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) -> usize {
    let mut hole = first;
    let mut child_i = first;
    let mut child = 0;
    loop {
        child_i += child + 1;
        child = 2 * child + 1;
        if child + 1 < len && less(&v[child_i], &v[child_i + 1]) {
            child_i += 1;
            child += 1;
        }
        v[hole] = v[child_i];
        hole = child_i;
        if child > (len - 2) / 2 {
            return hole;
        }
    }
}

/// `__sift_up` of `v[last - 1]` in the heap `v[first..last]`, `len` long.
fn sift_up<T: Copy>(
    v: &mut [T],
    first: usize,
    last: usize,
    len: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) {
    if len <= 1 {
        return;
    }
    let mut len = (len - 2) / 2;
    let mut ptr = first + len;
    let mut last = last - 1;
    if less(&v[ptr], &v[last]) {
        let t = v[last];
        loop {
            v[last] = v[ptr];
            last = ptr;
            if len == 0 {
                break;
            }
            len = (len - 1) / 2;
            ptr = first + len;
            if !less(&v[ptr], &t) {
                break;
            }
        }
        v[last] = t;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use std::cell::Cell;

    use super::{heap_sort, sort};

    thread_local! {
        /// How many times [`super::introsort`] took the heap sort.
        pub(super) static HEAP_SORTS: Cell<usize> = const { Cell::new(0) };
    }

    /// One case of `tests/data/cxx_sort_orders.txt`: its kind, its keys and
    /// the order libc++ left their indices in.
    fn cases() -> Vec<(char, Vec<f64>, Vec<usize>)> {
        include_str!("../../tests/data/cxx_sort_orders.txt")
            .lines()
            .filter(|l| !l.starts_with('#'))
            .map(|line| {
                let (head, order) = line
                    .split_once(" | ")
                    .or_else(|| line.split_once(" |"))
                    .unwrap();
                let mut words = head.split_whitespace();
                let kind = words.next().unwrap().chars().next().unwrap();
                let keys = words.map(|w| w.parse().unwrap()).collect();
                let order = order
                    .split_whitespace()
                    .map(|w| w.parse().unwrap())
                    .collect();
                (kind, keys, order)
            })
            .collect()
    }

    /// Each array of `tests/data/cxx_sort_orders.txt` (made by
    /// `sim/L0/tests/scripts/gen_cxx_sort_reference.py` from macOS's libc++)
    /// is left in libc++'s order: by [`sort`] where libc++'s `std::sort`
    /// sorted it, by [`heap_sort`] where its `std::partial_sort(first, last,
    /// last)` did; and each of McIlroy's adversary's arrays takes the heap
    /// sort past the depth limit.
    #[test]
    fn each_order_is_libcxxs() {
        let cases = cases();
        assert_eq!(cases.len(), 768, "cases");
        let mut failures = Vec::new();
        for (k, (kind, keys, order)) in cases.iter().enumerate() {
            let mut v: Vec<(f64, usize)> = keys.iter().copied().zip(0..).collect();
            HEAP_SORTS.with(|n| n.set(0));
            let mut less = |a: &(f64, usize), b: &(f64, usize)| a.0 < b.0;
            match kind {
                'S' | 'A' => sort(&mut v, &mut less),
                'H' => {
                    let n = v.len();
                    heap_sort(&mut v, 0, n, &mut less);
                }
                other => panic!("kind {other}"),
            }
            let ours: Vec<usize> = v.iter().map(|e| e.1).collect();
            if &ours != order {
                failures.push(format!("case {k} ({kind}, {} elements)", keys.len()));
            }
            if *kind == 'A' && HEAP_SORTS.with(Cell::get) == 0 {
                failures.push(format!("case {k}: the adversary's array took no heap sort"));
            }
        }
        assert!(
            failures.is_empty(),
            "{} of {}: {}",
            failures.len(),
            cases.len(),
            failures.join("; ")
        );
    }
}
