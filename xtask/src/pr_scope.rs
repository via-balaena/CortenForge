//! PR-scoping helpers shared by the sharded CI entry points.
//!
//! Two orthogonal narrowings turn a full-workspace command into a per-PR,
//! per-shard slice, and both `grade-all` and `run-validators` apply them
//! identically:
//!
//! - [`filter_only`] restricts the crate list to an explicit affected set
//!   (changed crates + reverse-dependency closure from `cargo xtask affected`),
//!   or passes it through unchanged on the full-workspace gate.
//! - [`select_shard`] partitions the (already `--only`-filtered) list into a
//!   disjoint `i/N` slice so CI can fan a command out across N parallel jobs.
//!
//! `--only` runs before `--shard`, so each shard takes a slice of the affected
//! subset. Keeping the two functions in one module means the grade job and the
//! validate-examples job share a single tested implementation of the contract
//! rather than drifting copies.

/// Partition a sorted crate list into the `i/N` shard (1-based).
///
/// Assignment is round-robin (`index % N == i-1`) rather than contiguous
/// blocks: the caller passes an alphabetically-sorted list, so round-robin
/// scatters expensive crates (e.g. `cf-cast`, `sim-soft`) across shards
/// instead of letting them cluster in one block and unbalance wall time.
/// `None` returns the full list unchanged. The union of all `1..=N` shards
/// is exactly the input with no overlaps (see `select_shard_partitions`).
pub(crate) fn select_shard(crate_names: &[String], shard: Option<(usize, usize)>) -> Vec<String> {
    match shard {
        None => crate_names.to_vec(),
        Some((i, n)) => crate_names
            .iter()
            .enumerate()
            .filter(|(idx, _)| idx % n == i - 1)
            .map(|(_, name)| name.clone())
            .collect(),
    }
}

/// Restrict the crate list to an explicit affected set (PR-scoped CI).
///
/// `None` returns the list unchanged (the full-workspace gate on
/// `main`/merge). `Some(set)` keeps only crates present in `set`, preserving
/// input order; an *empty* set therefore selects nothing — a deliberate no-op
/// for a PR that touches no crate (e.g. docs-only). `set` entries that are not
/// workspace members are simply ignored. Filtering happens before
/// [`select_shard`], so each shard takes a slice of the affected subset.
///
/// Pairs with `cargo xtask affected`, which computes the affected set
/// (changed crates + reverse-dependency closure) the caller passes via
/// `--only`.
pub(crate) fn filter_only(crate_names: &[String], only: Option<&[String]>) -> Vec<String> {
    match only {
        None => crate_names.to_vec(),
        Some(set) => crate_names
            .iter()
            .filter(|c| set.iter().any(|s| s == *c))
            .cloned()
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::{filter_only, select_shard};

    fn names(n: usize) -> Vec<String> {
        (0..n).map(|i| format!("crate-{i:03}")).collect()
    }

    #[test]
    fn select_shard_none_returns_everything() {
        let all = names(10);
        assert_eq!(select_shard(&all, None), all);
    }

    #[test]
    fn select_shard_partitions() {
        // The union of all N shards must be exactly the input, with no
        // crate appearing in two shards and none dropped — otherwise CI
        // would silently skip (or double-run) a crate.
        let all = names(295);
        for n in [1usize, 2, 3, 4, 7] {
            let mut union: Vec<String> = Vec::new();
            for i in 1..=n {
                union.extend(select_shard(&all, Some((i, n))));
            }
            union.sort();
            let mut expected = all.clone();
            expected.sort();
            assert_eq!(union, expected, "shards of {n} must reconstruct the input");
        }
    }

    #[test]
    fn select_shard_disjoint() {
        let all = names(295);
        let n = 3;
        let s1 = select_shard(&all, Some((1, n)));
        let s2 = select_shard(&all, Some((2, n)));
        let s3 = select_shard(&all, Some((3, n)));
        for c in &s1 {
            assert!(!s2.contains(c) && !s3.contains(c), "{c} in multiple shards");
        }
        for c in &s2 {
            assert!(!s3.contains(c), "{c} in multiple shards");
        }
    }

    #[test]
    fn select_shard_balances_within_one() {
        // Round-robin keeps shard sizes within 1 of each other.
        let all = names(295);
        let n = 3;
        let sizes: Vec<usize> = (1..=n)
            .map(|i| select_shard(&all, Some((i, n))).len())
            .collect();
        let max = sizes.iter().max().unwrap();
        let min = sizes.iter().min().unwrap();
        assert!(
            max - min <= 1,
            "shard sizes {sizes:?} differ by more than 1"
        );
    }

    #[test]
    fn filter_only_none_returns_everything() {
        let all = names(10);
        assert_eq!(filter_only(&all, None), all);
    }

    #[test]
    fn filter_only_keeps_just_the_affected_set_in_order() {
        let all = vec![
            "a".to_string(),
            "b".to_string(),
            "c".to_string(),
            "d".to_string(),
        ];
        let only = vec!["c".to_string(), "a".to_string()];
        // Input order preserved (a before c), not the `--only` order.
        assert_eq!(filter_only(&all, Some(&only)), vec!["a", "c"]);
    }

    #[test]
    fn filter_only_empty_set_selects_nothing() {
        // The deliberate no-op: a PR touching no crate must select zero crates,
        // NOT fall through to the full workspace.
        let all = names(5);
        assert!(filter_only(&all, Some(&[])).is_empty());
    }

    #[test]
    fn filter_only_ignores_non_member_names() {
        let all = vec!["sim-core".to_string(), "mesh".to_string()];
        let only = vec!["sim-core".to_string(), "ghost-crate".to_string()];
        assert_eq!(filter_only(&all, Some(&only)), vec!["sim-core"]);
    }

    #[test]
    fn filter_only_then_shard_slices_the_affected_subset() {
        // `--only` runs before `--shard`, so the union of shards over a
        // filtered list reconstructs exactly the affected set.
        let all = names(20);
        let only: Vec<String> = all.iter().take(7).cloned().collect();
        let scoped = filter_only(&all, Some(&only));
        let n = 3;
        let mut union: Vec<String> = Vec::new();
        for i in 1..=n {
            union.extend(select_shard(&scoped, Some((i, n))));
        }
        union.sort();
        let mut expected = only.clone();
        expected.sort();
        assert_eq!(union, expected);
    }
}
