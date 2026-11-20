// Ported from Qhull, libqhull_r (qset_r.c, qset_r.h).
// Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National
// Science and Technology Research Center for Computation and Visualization of
// Geometric Structures (The Geometry Center), University of Minnesota.
// qset_r.c: Copyright (c) 1993-2020 The Geometry Center.
// Modified 2026-11 by Via Balaena (the CortenForge authors): translated to Rust,
// keeping only the 3-d path MuJoCo 3.5.0 runs, to give its mesh graph. See
// COPYING.txt; the original is at www.qhull.org.

//! qhull's sets (`setT`) as `Vec`s, each operation keeping the order qhull's
//! leaves: an unsorted delete moves the last element into the hole. A qhull
//! set may hold NULL slots, and its `FOREACH` loops and searches stop at the
//! first: [`SetElem::NULL`] is that slot here, and [`get`] the loops' read.

use crate::state::{FacetId, RidgeId, VertexId};

/// An element of a qhull set, with qhull's NULL.
pub(crate) trait SetElem: Copy + PartialEq {
    /// qhull's NULL element.
    const NULL: Self;
}

impl SetElem for FacetId {
    const NULL: Self = Self::NULL;
}

impl SetElem for VertexId {
    const NULL: Self = Self(u32::MAX);
}

impl SetElem for RidgeId {
    const NULL: Self = Self(u32::MAX);
}

impl SetElem for u32 {
    const NULL: Self = Self::MAX;
}

/// The `i`th element as a `FOREACHsetelement_` loop reads it: `None` past the
/// end or at a NULL slot, where qhull's loop stops.
pub(crate) fn get<T: SetElem>(set: &[T], i: usize) -> Option<T> {
    set.get(i).copied().filter(|&e| e != T::NULL)
}

/// The index `qh_setdel` finds: the first `elem` before any NULL slot.
fn find<T: SetElem>(set: &[T], elem: T) -> Option<usize> {
    set.iter()
        .take_while(|&&e| e != T::NULL)
        .position(|&e| e == elem)
}

/// `qh_setsize`: the set's size, its NULL slots included.
pub(crate) const fn count<T: SetElem>(set: &[T]) -> usize {
    set.len()
}

/// `qh_setin`: whether `elem` comes before any NULL slot.
pub(crate) fn setin<T: SetElem>(set: &[T], elem: T) -> bool {
    find(set, elem).is_some()
}

/// `qh_setdelsorted` (`qset_r.c:506-525`): the first `elem` removed, the rest
/// shifted down.
pub(crate) fn setdelsorted<T: SetElem>(set: &mut Vec<T>, elem: T) {
    if let Some(i) = find(set, elem) {
        set.remove(i);
    }
}

/// `qh_setdelnthsorted` (`qset_r.c:467-486`).
pub(crate) fn setdelnthsorted<T: SetElem>(set: &mut Vec<T>, nth: usize) {
    set.remove(nth);
}

/// `qh_setreplace` (`qset_r.c:1122-1136`): the first `old` becomes `new`.
pub(crate) fn setreplace<T: SetElem>(set: &mut [T], old: T, new: T) {
    if let Some(i) = find(set, old) {
        set[i] = new;
    }
}

/// `qh_setcompact` (`qset_r.c:278-295`): the NULL slots dropped, the rest in
/// order.
pub(crate) fn setcompact<T: SetElem>(set: &mut Vec<T>) {
    set.retain(|&e| e != T::NULL);
}

/// `qh_setunique`: `elem` appended unless already there.
pub(crate) fn setunique<T: SetElem>(set: &mut Vec<T>, elem: T) {
    if !setin(set, elem) {
        set.push(elem);
    }
}

/// `qh_setdel` (`qset_r.c:346-366`): the first `elem` replaced by the last
/// element. Whether it was there.
pub(crate) fn setdel<T: SetElem>(set: &mut Vec<T>, elem: T) -> bool {
    let found = find(set, elem);
    if let Some(i) = found {
        set.swap_remove(i);
    }
    found.is_some()
}

/// `qh_setdelnth` (`qset_r.c:426-445`): the `nth` element replaced by the
/// last.
pub(crate) fn setdelnth<T: SetElem>(set: &mut Vec<T>, nth: usize) -> T {
    set.swap_remove(nth)
}

/// `qh_setaddnth` (`qset_r.c:63-84`).
pub(crate) fn setaddnth<T: SetElem>(set: &mut Vec<T>, nth: usize, elem: T) {
    set.insert(nth, elem);
}

/// `qh_setappend2ndlast` (`qset_r.c:209-224`): before the last element, of a
/// set qhull only gives it non-empty.
pub(crate) fn setappend2ndlast<T: SetElem>(set: &mut Vec<T>, elem: T) {
    let at = set.len().saturating_sub(1);
    set.insert(at, elem);
}

/// `qh_setnew_delnthsorted` (`qset_r.c:1009-1079`): a copy without its `nth`
/// element, `prepend` slots (NULL here) before it.
pub(crate) fn setnew_delnthsorted<T: SetElem>(set: &[T], nth: usize, prepend: usize) -> Vec<T> {
    let mut out = Vec::with_capacity(set.len() - 1 + prepend);
    out.extend(std::iter::repeat_n(T::NULL, prepend));
    out.extend(
        set.iter()
            .enumerate()
            .filter(|&(i, _)| i != nth)
            .map(|(_, &e)| e),
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unsorted_delete_moves_the_last_element_into_the_hole() {
        let mut set = vec![10_u32, 11, 12, 13];
        assert!(setdel(&mut set, 11));
        assert_eq!(set, [10, 13, 12]);
        assert_eq!(setdelnth(&mut set, 0), 10);
        assert_eq!(set, [12, 13]);
    }

    #[test]
    fn a_search_stops_at_the_first_null_slot() {
        let mut set = vec![10_u32, u32::NULL, 12];
        assert!(!setdel(&mut set, 12));
        assert_eq!(get(&set, 0), Some(10));
        assert_eq!(get(&set, 1), None);
    }

    #[test]
    fn insertions_and_the_copy_without_one() {
        let mut set = vec![10_u32, 13];
        setappend2ndlast(&mut set, 12);
        assert_eq!(set, [10, 12, 13]);
        setaddnth(&mut set, 0, 9);
        assert_eq!(set, [9, 10, 12, 13]);
        assert_eq!(setnew_delnthsorted(&set, 1, 1), [u32::NULL, 9, 12, 13]);
    }
}
